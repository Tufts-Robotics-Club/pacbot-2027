#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "comms.h"
#include "tof.h"

// holds one motor_cmd_t. size 1 so a new command replaces the old one
static QueueHandle_t cmd_mailbox;

static TaskHandle_t motor_task_handle;

// indexed by ENC_LEFT (0) / ENC_RIGHT (1). each motor has two L293D inputs and gets one PWM channel on each:
//   first input:  left = 1A (pin 2),  right = 3A (pin 10)
//   second input: left = 2A (pin 7),  right = 4A (pin 15)
// these pins are for pwm
static const int first_input_pins[2] = {LEFT_1A, RIGHT_3A}; 
static const int second_input_pins[2] = {LEFT_2A, RIGHT_4A};
static const ledc_channel_t first_input_channels[2] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};
static const ledc_channel_t second_input_channels[2] = {LEDC_CHANNEL_2, LEDC_CHANNEL_3};

#define PWM_MAX 1023  // 10 bit duty so the speed control can make steps smaller than 1%

static dir_t facing = NORTH;  // assume we start facing north

// master on/off for both motors. off = L293D outputs float so the motors coast
static void motors_enable(bool on)
{
    gpio_set_level(LEFT_EN, on);
    gpio_set_level(RIGHT_EN, on);
}

static void pwm_channel_start(int pin, ledc_channel_t channel)
{
    ledc_channel_config_t ch_cfg = { //settings for a pwm channel in a struct
        .gpio_num = pin, //pin used for pwm
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER_0, //attach channel to a timer
        .duty = 0,   // both inputs low = brake
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
}

static void motor_pins_start(void)
{
    // EN starts low (the pull downs hold it there during boot too) until the PWM is set up
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << LEFT_EN) | (1ULL << RIGHT_EN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&en_cfg));
    motors_enable(false);

    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    for (int i = 0; i < 2; i++) {
        pwm_channel_start(first_input_pins[i], first_input_channels[i]);
        pwm_channel_start(second_input_pins[i], second_input_channels[i]);
    }

    motors_enable(true);
}

// last duty written to each channel. the speed control calls set_motor every tick, so only talk to the
// PWM hardware when the duty actually changes
static uint32_t channel_duty[LEDC_CHANNEL_MAX];

static void set_channel(ledc_channel_t channel, float percent)
{
    uint32_t duty = (uint32_t)(percent * PWM_MAX / 100.0f + 0.5f);
    if (duty == channel_duty[channel]) {
        return;
    }
    channel_duty[channel] = duty;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// signed duty in percent, -100..100. negative = backward, 0 = brake
// the PWM goes on one input and the other stays low, so the off part of each cycle brakes instead of coasting
static void set_motor(enc_side_t side, float duty)
{
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;

    // drop the old side to 0 first so both inputs are never high at the same time
    if (duty >= 0) {
        set_channel(first_input_channels[side], 0);
        set_channel(second_input_channels[side], duty);
    } else {
        set_channel(second_input_channels[side], 0);
        set_channel(first_input_channels[side], -duty);
    }
}

static void motors_stop(void)
{
    set_motor(ENC_LEFT, 0);
    set_motor(ENC_RIGHT, 0);
}

// ---------- feedforward: what duty gives what speed ----------
// measured with tune.py on the floor (one caster) in one sitting on a fresh 9V battery at 8.3 V (11/13).
// each row is a duty and the steady speed each wheel reached at that duty, in encoder counts/s.
// to get a speed we look up the duty in between. 10% is where the wheels stop moving on the floor (the deadband)
// a drained battery is slower at the same duty, the PI (todo) makes up the difference
// redo this table on the real chassis
#define FF_TABLE_VOLTS 8.3f   // battery voltage the table was measured at

typedef struct {
    float duty;
    float cps[2];   // [ENC_LEFT], [ENC_RIGHT], counts per second
} ff_point_t;

static const ff_point_t ff_table[] = {
    { 10,  {    0,    0 } },
    { 15,  {  960,  730 } },
    { 20,  { 1830, 1620 } },
    { 30,  { 2550, 2400 } },
    { 40,  { 3060, 2910 } },
    { 60,  { 4000, 3820 } },
    { 80,  { 5030, 4630 } },
    { 100, { 5820, 5340 } },
};
#define FF_POINTS (int)(sizeof(ff_table) / sizeof(ff_table[0]))

// duty (%) that should give this wheel this speed (counts/s). backward is the same table flipped
static float ff_duty(enc_side_t side, float cps)
{
    float speed = fabsf(cps);
    if (speed < 1.0f) {
        return 0;
    }

    // find the two rows the speed falls between (past the last row it keeps the last slope)
    int i = 1;
    while (i < FF_POINTS - 1 && speed > ff_table[i].cps[side]) {
        i++;
    } //then i is first index for point on lut that gives counts/sec higher than desired

    const ff_point_t *lo = &ff_table[i - 1]; //lut point right above desired
    const ff_point_t *hi = &ff_table[i]; //lut point right below desired
    float span = hi->cps[side] - lo->cps[side]; //how far apart the two rows are
    if (span <= 0) {
        // two rows with the same speed (bad table), dont divide by zero
        return cps > 0 ? hi->duty : -hi->duty; // just pick the hi one
    }
    // duty = low duty + (desired cps - low cps)(hi duty - lo duty) / (hi cps - low cps)
    float duty = lo->duty + (speed - lo->cps[side]) * (hi->duty - lo->duty) / span;

    return cps > 0 ? duty : -duty;
}

static float mm_s_to_cps(float mm_s)
{
    return mm_s / MM_PER_COUNT;
}

// ---------- measured speed ----------
// each wheel's speed = how many counts it moved over the last SPEED_WINDOW ticks (~20 ms), updated every tick.
// count_hist keeps the count from each of the last 20 ticks (a ring buffer) and time_hist the clock time of
// each one. dividing by the REAL time between them (not "20 ticks = 20 ms") means a late tick cant fake a speed
#define SPEED_WINDOW 20

static int32_t count_hist[2][SPEED_WINDOW];
static int64_t time_hist[SPEED_WINDOW];   // esp_timer time of each slot, microseconds since boot
static int hist_idx;                      // which slot holds the oldest reading (the one ~20 ticks ago)
static float measured_cps[2];             // latest measured speed of each wheel, counts/s

// called once per motor_task tick
static void speed_measure(void)
{
    int64_t now_us = esp_timer_get_time();
    int64_t then_us = time_hist[hist_idx];               // when the oldest reading was taken
    float seconds = (now_us - then_us) / 1000000.0f;    // real time between them, ~0.020 s

    for (int side = 0; side < 2; side++) {
        int32_t now = encoder_count(side);
        int32_t then = count_hist[side][hist_idx];      // count ~20 ticks ago
        count_hist[side][hist_idx] = now;               // this slot now holds the newest count
        if (then_us > 0 && seconds > 0) {               // then_us is 0 for the first 20 ticks after boot
            measured_cps[side] = (now - then) / seconds;
        }
    }
    time_hist[hist_idx] = now_us;
    hist_idx = (hist_idx + 1) % SPEED_WINDOW;           // move to the next oldest slot, wrap around at 20
}

// ---------- PI gains ----------
// duty = feedforward + kp * error + ki * (error added up over time), error in mm/s
// starting guesses from the step tests: 1% duty ~ 7 mm/s, so kp 0.05 turns a 20 mm/s error into 1% more duty
// change them over wifi with "kp 0.08" / "ki 0.5", see them with "show". copy good values back in here
static float kp = 0.05f;   // % duty per mm/s of error
static float ki = 0.5f;    // % duty per (mm/s of error x seconds)

// the speed control runs every CONTROL_PERIOD_MS ticks, not every tick. on the original ESP32, changing a
// channel's duty waits (busy loops) until the last change has finished, and a change takes up to 2 PWM cycles
// to finish (it starts at the next cycle and finishes at the end of it): up to 6.7 ms at 300 Hz.
// changing the duty every 1 ms made motor_task wait ~3 ms per tick and the PI went wild. every 5 ms still hit
// the wait sometimes (3.7 ms ticks: both motors waiting ~1.7 ms each). 10 ms is longer than 2 cycles, so the
// last change is always done and it never waits. 100 updates a second is still 6x faster than the motor reacts
#define CONTROL_PERIOD_MS 10

static uint32_t motor_ticks;   // counts motor_task ticks since boot

static bool control_due(void)
{
    return motor_ticks % CONTROL_PERIOD_MS == 0;
}

// speed mode (ex: v 300, as opposed do duty mode d 30 30): motor_task calls speed_update every tick to hold target_cps
static struct {
    bool active;
    float target_cps[2]; // speed each wheel should hold (count/sec)
    float integral[2];   // the I part, already in % duty: grows while the wheel is too slow, shrinks while too fast
    float duty[2];       // last duty sent to each motor (for the logs)
    int64_t last_us;     // when speed_update last ran, for the I part's time step. 0 = first run
} speed_ctl; // speed control

static void speed_reset_integral(void)
{
    speed_ctl.integral[ENC_LEFT] = 0;
    speed_ctl.integral[ENC_RIGHT] = 0;
    speed_ctl.last_us = 0;
}

static void speed_update(void)
{
    // real time since the last update (normally CONTROL_PERIOD_MS). capped so a long pause cant dump a huge
    // amount into the I part at once
    int64_t now_us = esp_timer_get_time();
    float dt = speed_ctl.last_us ? (now_us - speed_ctl.last_us) / 1000000.0f : CONTROL_PERIOD_MS / 1000.0f;
    if (dt > 0.05f) dt = 0.05f;
    speed_ctl.last_us = now_us;

    for (int side = 0; side < 2; side++) {
        float target = speed_ctl.target_cps[side];

        // target 0 = brake and forget any built up correction
        if (target == 0) {
            speed_ctl.integral[side] = 0;
            speed_ctl.duty[side] = 0;
            set_motor(side, 0);
            continue;
        }

        float error = (target - measured_cps[side]) * MM_PER_COUNT;   // mm/s, positive = too slow (forward)
        float ff = ff_duty(side, target);                               // feedforward: the table's guess
        float p = kp * error;                                           // P: push harder the further off we are
        float duty = ff + p + speed_ctl.integral[side];

        // anti windup: only let the I part grow if the duty isnt already maxed out in that direction.
        // otherwise it keeps growing while the motor cant go any faster, and overshoots badly later
        bool maxed_out = (duty >= 100 && error > 0) || (duty <= -100 && error < 0);
        if (!maxed_out) {
            speed_ctl.integral[side] += ki * error * dt;
        }

        if (duty > 100) duty = 100;
        if (duty < -100) duty = -100;
        speed_ctl.duty[side] = duty;
        set_motor(side, duty);
    }
}

// longest one motor_task tick has taken since the last step test started, in microseconds
// if a tick ever takes over 1 ms the task never sleeps, the imu task and IDLE1 starve and the watchdog reboots us
static int64_t loop_us_max; //basically just for debugging, doesn't really control motor functionality

// step tests for tune.py: brake for STEP_PRE_MS, run both wheels for duration_ms, then brake for STEP_POST_MS
//   duty step:  fixed duty. every STEP_LOG_MS sends a step_log message {t, value: duty %, left/right: counts/s}
//   speed step: speed mode at a target. step_log {t, value: target mm/s, left/right: mm/s, left/right_duty}
// and a step_done message at the end
#define STEP_PRE_MS   200   // motors stay braked for 200 ms before the run starts, to make sure wheels are fully stopped
#define STEP_POST_MS  500   // after run motors brake for 500 ms while logging continues
#define STEP_LOG_MS   20    // 50 Hz, one speed sample per log line

static struct { // state of the test that's currently running (duty or step)
    bool active;            // step
    bool speed;             // true = speed step, false = duty step (vstep)
    int value;              // duty % for a duty step, target mm/s for a speed step
    uint32_t duration_ms;
    uint32_t t_ms;          // ms since the test started, counted by motor_task's 1 ms ticks
    int32_t last_count[2];  // encoder counts at the last log line
} step;


// for both vstep and step
static void step_start(bool speed, int value, int duration_ms)
{
    if (duration_ms < 100) duration_ms = 100; //clamp
    if (duration_ms > 10000) duration_ms = 10000;

    speed_ctl.active = false;
    speed_reset_integral();   // start the speed test with no leftover correction
    motors_stop();
    motors_enable(true);
    step.active = true;
    step.speed = speed;
    step.value = value;
    step.duration_ms = duration_ms;
    step.t_ms = 0;
    step.last_count[ENC_LEFT] = encoder_count(ENC_LEFT);
    step.last_count[ENC_RIGHT] = encoder_count(ENC_RIGHT);
    loop_us_max = 0;
}

// runs once per motor_task tick while a step test is going
static void step_update(void)
{
    step.t_ms++;
    uint32_t t = step.t_ms;
    uint32_t run_end = STEP_PRE_MS + step.duration_ms;
    bool running = t > STEP_PRE_MS && t <= run_end;

    if (running && step.speed) { // speed test: run the speed control every CONTROL_PERIOD_MS of the run
        speed_ctl.target_cps[ENC_LEFT] = mm_s_to_cps(step.value);
        speed_ctl.target_cps[ENC_RIGHT] = mm_s_to_cps(step.value);
        if (control_due()) {
            speed_update();
        }
    } else if (t == STEP_PRE_MS + 1) { // duty test (step) only once
        // if it's the ms after the pre phase, moment run should start
        set_motor(ENC_LEFT, step.value);
        set_motor(ENC_RIGHT, step.value);
    }
    if (t == run_end + 1) {
        motors_stop();
    }

    if (t % STEP_LOG_MS == 0) {
        // same ~20 ms real time speed the PI uses, so the plot shows exactly what the controller saw
        long left_speed = lroundf(measured_cps[ENC_LEFT]);
        long right_speed = lroundf(measured_cps[ENC_RIGHT]);

        if (step.speed) {
            left_speed = lroundf(left_speed * MM_PER_COUNT);
            right_speed = lroundf(right_speed * MM_PER_COUNT);
        }

        bot_msg_t msg = {
            .type = BOT_MSG_STEP_LOG,
            .step_log = {
                .t_ms = t,
                .value = running ? step.value : 0,
                .left = left_speed,
                .right = right_speed,
                // speed tests also send the duty the controller chose, so tune.py can show the PI working
                .has_duty = step.speed,
                .left_duty = (step.speed && running) ? speed_ctl.duty[ENC_LEFT] : 0,
                .right_duty = (step.speed && running) ? speed_ctl.duty[ENC_RIGHT] : 0,
            },
        };
        comms_send(&msg);
    }

    if (t >= run_end + STEP_POST_MS) {
        step.active = false;
        comms_send_info("slowest motor loop during the test: %ld us (has to stay under 1000)", (long)loop_us_max);
        comms_send(&(bot_msg_t){ .type = BOT_MSG_STEP_DONE });
    }
}

// ---------- driving straight ----------
// both wheels at the same speed with the PI (negative = backward). was_driving = speed mode was already on,
// so the I part is kept unless the wheels change direction (same idea as the v command)
static void drive_straight(float mm_s, bool was_driving)
{
    float cps = mm_s_to_cps(mm_s);
    bool flipped = (cps * speed_ctl.target_cps[ENC_LEFT] < 0) || (cps * speed_ctl.target_cps[ENC_RIGHT] < 0);
    if (!was_driving || flipped) {
        speed_reset_integral();
    }
    speed_ctl.target_cps[ENC_LEFT] = cps;
    speed_ctl.target_cps[ENC_RIGHT] = cps;
    speed_ctl.active = true;   // motor_task's next control tick starts driving
    motors_enable(true);
}

// ---------- turning in place ----------
// tcw 90 / tccw 90: spin in place by that many degrees, then brake.
// in a spin each wheel rolls along a circle whose diameter is the track width, so for an angle
// each wheel has to roll (angle / 360) x pi x TRACK_WIDTH_MM. 90 deg = ~120 mm = ~1020 counts per wheel.
// it runs the normal speed control (one wheel forward, one backward) and checks the counts every 1 ms tick.
// each wheel brakes on its own once it's done, then once both have stopped rolling it reports the real angle
// so we can tune TRACK_WIDTH_MM and TURN_COAST_COUNTS
#define TURN_SPEED_MM_S    100   // wheel speed while turning (tcw 90 150 to try a different one)
#define DRIVE_SPEED_MM_S   200   // straight-line speed for n/s/e/w (try others with v first)
#define TURN_COAST_COUNTS  0     // brake this many counts early to make up for coasting. set from tests
#define TURN_SETTLE_MS     300   // after braking, wait at most this long for the wheels to stop before reporting

static struct {
    bool active;
    bool braking;            // both wheels reached their target and are braked, waiting to settle
    int dir[2];              // +1 = this wheel goes forward, -1 = backward (cw: left +1, right -1)
    float degrees;           // what was asked for (for the report)
    int32_t start_count[2];  // encoder counts when the turn started
    int32_t target_counts;   // how far each wheel has to roll, in counts
    bool done[2];            // this wheel reached its target and is braked
    uint32_t settle_ms;      // ms since both wheels braked
    bool then_drive;         // n/s/e/w turn: drive forward at DRIVE_SPEED_MM_S once it's done (tcw/tccw just stop)
} turn;

static void turn_start(bool clockwise, float degrees, float speed_mm_s)
{
    if (degrees < 0) degrees = -degrees;
    if (degrees > 3600) degrees = 3600;
    if (speed_mm_s <= 0) speed_mm_s = TURN_SPEED_MM_S;

    // clockwise seen from above = left wheel forward, right wheel backward
    turn.dir[ENC_LEFT] = clockwise ? 1 : -1;
    turn.dir[ENC_RIGHT] = clockwise ? -1 : 1;
    turn.degrees = degrees;
    float wheel_mm = degrees / 360.0f * 3.14159265f * TRACK_WIDTH_MM;
    turn.target_counts = (int32_t)lroundf(wheel_mm / MM_PER_COUNT) - TURN_COAST_COUNTS;
    turn.start_count[ENC_LEFT] = encoder_count(ENC_LEFT);
    turn.start_count[ENC_RIGHT] = encoder_count(ENC_RIGHT);
    turn.done[ENC_LEFT] = turn.done[ENC_RIGHT] = turn.target_counts <= 0;
    turn.braking = false;
    turn.settle_ms = 0;
    turn.then_drive = false;   // the n/s/e/w code sets it after calling this
    turn.active = true;

    speed_reset_integral();
    for (int side = 0; side < 2; side++) {
        speed_ctl.target_cps[side] = turn.done[side] ? 0 : turn.dir[side] * mm_s_to_cps(speed_mm_s);
    }
    speed_ctl.active = true;
    motors_enable(true);
}

// runs every motor_task tick while turning (the speed control itself still runs every CONTROL_PERIOD_MS)
static void turn_update(void)
{
    int32_t moved[2];   // how far each wheel has rolled in its own turning direction, in counts
    for (int side = 0; side < 2; side++) {
        moved[side] = (encoder_count(side) - turn.start_count[side]) * turn.dir[side];
        if (!turn.done[side] && moved[side] >= turn.target_counts) {
            // this wheel is there: brake it right now instead of waiting for the next control tick
            turn.done[side] = true;
            speed_ctl.target_cps[side] = 0;
            speed_ctl.integral[side] = 0;
            set_motor(side, 0);
        }
    }

    if (!(turn.done[ENC_LEFT] && turn.done[ENC_RIGHT])) {
        return;
    }

    // both braked. wait until they've actually stopped rolling so the report includes the coast
    turn.braking = true;
    turn.settle_ms++;
    bool stopped = fabsf(measured_cps[ENC_LEFT]) < 20 && fabsf(measured_cps[ENC_RIGHT]) < 20;
    if (!stopped && turn.settle_ms < TURN_SETTLE_MS) {
        return;
    }

    turn.active = false;
    speed_ctl.active = false;
    motors_stop();

    // real angle from what the wheels actually rolled (average of the two)
    float avg_counts = (moved[ENC_LEFT] + moved[ENC_RIGHT]) / 2.0f;
    float actual_deg = avg_counts * MM_PER_COUNT / (3.14159265f * TRACK_WIDTH_MM) * 360.0f;
    int32_t coast = lroundf(avg_counts) - (turn.target_counts + TURN_COAST_COUNTS);
    comms_send_info("turn done: asked %.1f deg, wheels say %.1f deg (L=%ld R=%ld counts, %+ld past target)",
                    turn.degrees, actual_deg, (long)moved[ENC_LEFT], (long)moved[ENC_RIGHT], (long)coast);

    // the turn was the first half of an n/s/e/w command: now drive off the way we're facing
    if (turn.then_drive) {
        drive_straight(DRIVE_SPEED_MM_S, false);
    }
}

// n / s / e / w on the board at DRIVE_SPEED_MM_S, turning 90 first if needed (opposite = reverse)
static void go_direction(dir_t want, bool was_speed_mode)
{
    // how many 90 deg cw steps from where we face to where we want to go, wrapped into 0..3
    int diff = ((want - facing) % 4 + 4) % 4;
    if (diff == 0) {
        // already facing it
        drive_straight(DRIVE_SPEED_MM_S, was_speed_mode);
        comms_send_info("going %c: forward (facing %c)", "NESW"[want], "NESW"[facing]);
    } else if (diff == 2) {
        // it's right behind us: reverse instead of turning 180. facing does NOT change, the front still points
        // the same way, we're just rolling backward
        drive_straight(-DRIVE_SPEED_MM_S, was_speed_mode);
        comms_send_info("going %c: backward (facing %c)", "NESW"[want], "NESW"[facing]);
    } else {
        // diff 1 = it's 90 cw of us, diff 3 = 90 ccw. turn, then turn_update drives forward when it's done
        bool clockwise = diff == 1;
        turn_start(clockwise, 90, 0);
        turn.then_drive = true;
        // set now rather than when the turn ends so the next command is worked out from the new way.
        // TODO: if another command interrupts the turn half way, the bot is really pointing somewhere in between
        facing = want;
        comms_send_info("going %c: turning %s 90 then forward (facing %c)", "NESW"[want],
                        clockwise ? "cw" : "ccw", "NESW"[facing]);
    }
}

// runs one command from the server. comms.c already turned the json into a motor_cmd_t (see messages.c),
// so all we do here is switch on its type and read the matching member of the union
static void handle_command(const motor_cmd_t *cmd)
{
    // gain commands only change a number, so they dont stop whatever is running
    if (cmd->type == MOTOR_CMD_GAINS) {
        if (cmd->gains.has_kp) kp = cmd->gains.kp;
        if (cmd->gains.has_ki) ki = cmd->gains.ki;
        comms_send_info("kp %.4f  ki %.4f", kp, ki);
        return;
    }

    // every other command takes over from a step test, turn or speed mode that's still running
    bool was_speed_mode = speed_ctl.active && !turn.active;
    step.active = false;
    turn.active = false;
    speed_ctl.active = false;

    switch (cmd->type) {
    case MOTOR_CMD_STOP:
        motors_stop();
        if (cmd->stop.coast) {
            motors_enable(false);   // EN low so the motors coast
            comms_send_info("motors off");
        } else {
            comms_send_info("stopped");
        }
        break;

    case MOTOR_CMD_DIRECTION:
        go_direction(cmd->direction, was_speed_mode);
        break;

    case MOTOR_CMD_TURN:
        turn_start(cmd->turn.clockwise, cmd->turn.degrees, cmd->turn.speed_mm_s);
        comms_send_info("turning %s %.1f deg (%ld counts per wheel)", cmd->turn.clockwise ? "cw" : "ccw",
                        turn.degrees, (long)turn.target_counts);
        break;

    case MOTOR_CMD_STEP_TEST:
        step_start(cmd->step.speed, cmd->step.value, cmd->step.duration_ms);
        comms_send_info("step test: %d %s for %d ms", cmd->step.value, cmd->step.speed ? "mm/s" : "%",
                        cmd->step.duration_ms);
        break;

    case MOTOR_CMD_SPEED: {
        float new_left = mm_s_to_cps(cmd->speed.left);
        float new_right = mm_s_to_cps(cmd->speed.right);

        // keep the I part when just changing speed (it's mostly correcting battery/friction, still useful).
        // start it fresh when speed mode was off or a wheel changes direction
        bool flipped = (new_left * speed_ctl.target_cps[ENC_LEFT] < 0) || (new_right * speed_ctl.target_cps[ENC_RIGHT] < 0);
        if (!was_speed_mode || flipped) {
            speed_reset_integral();
        }

        speed_ctl.target_cps[ENC_LEFT] = new_left;
        speed_ctl.target_cps[ENC_RIGHT] = new_right;
        speed_ctl.active = true;   // motor_task's next control tick (within 5 ms) starts driving
        motors_enable(true);
        comms_send_info("speed set to L=%d R=%d mm/s (starting duty L=%.1f%% R=%.1f%%)", cmd->speed.left,
                        cmd->speed.right, ff_duty(ENC_LEFT, new_left), ff_duty(ENC_RIGHT, new_right));
        break;
    }

    case MOTOR_CMD_DUTY:
        set_motor(ENC_LEFT, cmd->duty.left);
        set_motor(ENC_RIGHT, cmd->duty.right);
        motors_enable(true);
        comms_send_info("duty set to L=%d%% R=%d%%", cmd->duty.left, cmd->duty.right);
        break;

    case MOTOR_CMD_GAINS:
        break;   // handled above
    }
}

// timer interrupt every 1 ms that wakes up motor_task
static bool IRAM_ATTR motor_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(motor_task_handle, &woken); // wake up motor task
    return woken == pdTRUE;
}

static void motor_timer_start(void)
{

    // setting up the timer
    gptimer_handle_t timer;
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &timer));

    // attatching the function to the interrupt
    gptimer_event_callbacks_t cbs = {.on_alarm = motor_timer_isr};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    gptimer_alarm_config_t alarm = {
        .alarm_count = 1000000 / MOTOR_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    // enable the timer
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

// prints and/or sends the encoder counts ENC_REPORT_HZ times a second while were testing them
// its own low priority task on core 0 so printing never slows down motor_task
static void enc_report_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    bot_msg_t msg = { .type = BOT_MSG_ENCODERS };
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / ENC_REPORT_HZ));
        msg.encoders.left = encoder_count(ENC_LEFT);
        msg.encoders.right = encoder_count(ENC_RIGHT);
        if (PRINT_ENC) {
            printf("enc L: %ld  R: %ld\n", (long)msg.encoders.left, (long)msg.encoders.right);
        }
        if (SEND_ENC) {
            comms_send(&msg);   // dropped if were not connected
        }
    }
}

static void motor_task(void *arg)
{
    // set up the encoders and timer here so their interrupts run on core 1 not the wifi core
    motor_pins_start();
    encoder_start();
    if (PRINT_ENC || SEND_ENC) {
        xTaskCreatePinnedToCore(enc_report_task, "enc_report", STACK_STATS, NULL, PRIO_STATS, NULL, COMMS_CORE);
    }
    motor_timer_start();
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    tof_data_t tof = {0};
    motor_cmd_t cmd;

    while (1) {
        // sleep until the timer wakes us up (from the ISR)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        esp_task_wdt_reset();
        int64_t loop_start = esp_timer_get_time();
        motor_ticks++;

        speed_measure();   // update each wheel's measured speed every tick, the PI needs it fresh
        tof_latest(&tof);

        // receive takes it out of the mailbox so we never run the same command twice
        if (xQueueReceive(cmd_mailbox, &cmd, 0) == pdTRUE) {
            handle_command(&cmd);
        }

        if (step.active) {
            step_update();
        } else if (turn.active) {
            turn_update();   // checks the counts every tick so it brakes within 1 ms of the target
            if (speed_ctl.active && control_due()) {
                speed_update();
            }
        } else if (speed_ctl.active && control_due()) {
            speed_update();
        }

        int64_t loop_us = esp_timer_get_time() - loop_start;
        if (loop_us > loop_us_max) {
            loop_us_max = loop_us;
        }
    }
}

void motor_start(void)
{
    cmd_mailbox = xQueueCreate(1, sizeof(motor_cmd_t));
    xTaskCreatePinnedToCore(motor_task, "motor", STACK_MOTOR, NULL, PRIO_MOTOR, &motor_task_handle, RT_CORE);
}

void motor_command(const motor_cmd_t *cmd)
{
    xQueueOverwrite(cmd_mailbox, cmd);   // copies the struct in, so cmd can be a local in the caller
}
