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

// size 1 so a new command replaces the old one
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

// speed mode (ex: v 300, as opposed do duty mode d 30 30): motor_task sets the duty every tick to hold target_cps (right now feedforward only, PI comes next)
static struct {
    bool active; 
    float target_cps[2]; // speed each wheel should hold (count/sec)
} speed_ctl; // speed control

static void speed_update(void)
{
    set_motor(ENC_LEFT, ff_duty(ENC_LEFT, speed_ctl.target_cps[ENC_LEFT]));
    set_motor(ENC_RIGHT, ff_duty(ENC_RIGHT, speed_ctl.target_cps[ENC_RIGHT]));
}

// longest one motor_task tick has taken since the last step test started, in microseconds
// if a tick ever takes over 1 ms the task never sleeps, the imu task and IDLE1 starve and the watchdog reboots us
static int64_t loop_us_max; //basically just for debugging, doesn't really control motor functionality

// step tests for tune.py: brake for STEP_PRE_MS, run both wheels for duration_ms, then brake for STEP_POST_MS
//   duty step  (o): fixed duty. every STEP_LOG_MS sends "log <t_ms> <duty%> <left counts/s> <right counts/s>"
//   speed step (p): speed mode at a target. sends "log <t_ms> <target mm/s> <left mm/s> <right mm/s>"
// and "log done" at the end
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

    if (running && step.speed) { // every millisecond of the run
        speed_ctl.target_cps[ENC_LEFT] = mm_s_to_cps(step.value);
        speed_ctl.target_cps[ENC_RIGHT] = mm_s_to_cps(step.value);
        speed_update();
    } else if (t == STEP_PRE_MS + 1) { // duty test (step) only once
        // if it's the ms after the pre phase, moment run should start
        set_motor(ENC_LEFT, step.value);
        set_motor(ENC_RIGHT, step.value);
    }
    if (t == run_end + 1) {
        motors_stop();
    }

    if (t % STEP_LOG_MS == 0) {
        int32_t left = encoder_count(ENC_LEFT);
        int32_t right = encoder_count(ENC_RIGHT);
        long left_speed = (long)(left - step.last_count[ENC_LEFT]) * 1000 / STEP_LOG_MS;
        long right_speed = (long)(right - step.last_count[ENC_RIGHT]) * 1000 / STEP_LOG_MS;
        step.last_count[ENC_LEFT] = left;
        step.last_count[ENC_RIGHT] = right;

        if (step.speed) {
            left_speed = lroundf(left_speed * MM_PER_COUNT);
            right_speed = lroundf(right_speed * MM_PER_COUNT);
        }

        char msg[MSG_LEN];
        snprintf(msg, sizeof(msg), "log %lu %d %ld %ld", (unsigned long)t, running ? step.value : 0, left_speed, right_speed);
        comms_send(msg);
    }

    if (t >= run_end + STEP_POST_MS) {
        step.active = false;
        char msg[MSG_LEN];
        snprintf(msg, sizeof(msg), "slowest motor loop during the test: %ld us (has to stay under 1000)", (long)loop_us_max);
        comms_send(msg);
        comms_send("log done");
    }
}

// commands from console.py, same as the arduino version. one number = both wheels, two = left right
//   d <duty%>          open loop duty, -100..100 (turns the motors back on if x turned them off)
//   s or 0             stop with the brake
//   x                  master off, EN low so the motors coast
//   v <mm/s>           drive at a speed using the feedforward table (one number = both wheels, two = left right)
//   o <duty%> <ms>     duty step test for tune.py (any other command cancels it)
//   p <mm/s> <ms>      speed step test for tune.py's vstep
static void handle_command(const char *cmd)
{
    char reply[MSG_LEN];
    int left = 0, right = 0;
    int n = sscanf(cmd + 1, "%d %d", &left, &right); // n = numbers in the command 

    // a new command always takes over from a step test or speed mode that's still running
    step.active = false;
    speed_ctl.active = false;

    if ((cmd[0] == 'o' || cmd[0] == 'p') && n >= 1) {
        bool speed = cmd[0] == 'p';
        int duration_ms = (n == 2) ? right : 1000;
        step_start(speed, left, duration_ms);
        snprintf(reply, sizeof(reply), "step test: %d %s for %d ms", left, speed ? "mm/s" : "%", duration_ms);
        comms_send(reply);
        return;
    }

    if (n == 1) right = left;

    if (cmd[0] == 'v' && n >= 1) {
        speed_ctl.target_cps[ENC_LEFT] = mm_s_to_cps(left);
        speed_ctl.target_cps[ENC_RIGHT] = mm_s_to_cps(right);
        speed_ctl.active = true;
        speed_update();
        motors_enable(true);
        snprintf(reply, sizeof(reply), "speed set to L=%d R=%d mm/s (duty L=%.1f%% R=%.1f%%)", left, right,
                 ff_duty(ENC_LEFT, speed_ctl.target_cps[ENC_LEFT]), ff_duty(ENC_RIGHT, speed_ctl.target_cps[ENC_RIGHT]));
    } else if (cmd[0] == 'd' && n >= 1) {
        set_motor(ENC_LEFT, left);
        set_motor(ENC_RIGHT, right);
        motors_enable(true);
        snprintf(reply, sizeof(reply), "duty set to L=%d%% R=%d%%", left, right);
    } else if (cmd[0] == 's' || cmd[0] == '0') {
        motors_stop();
        snprintf(reply, sizeof(reply), "stopped");
    } else if (cmd[0] == 'x') {
        motors_stop();
        motors_enable(false);
        snprintf(reply, sizeof(reply), "motors off");
    } else {
        snprintf(reply, sizeof(reply), "unknown command: %.100s", cmd);
    }
    comms_send(reply);
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
    char msg[MSG_LEN];
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / ENC_REPORT_HZ));
        snprintf(msg, sizeof(msg), "enc L: %ld  R: %ld", (long)encoder_count(ENC_LEFT), (long)encoder_count(ENC_RIGHT));
        if (PRINT_ENC) {
            printf("%s\n", msg);
        }
        if (SEND_ENC) {
            comms_send(msg);   // dropped if were not connected
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
    char cmd[MSG_LEN];

    while (1) {
        // sleep until the timer wakes us up (from the ISR)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        esp_task_wdt_reset();
        int64_t loop_start = esp_timer_get_time();

        tof_latest(&tof);

        // receive takes it out of the mailbox so we never run the same command twice
        if (xQueueReceive(cmd_mailbox, cmd, 0) == pdTRUE) {
            // TODO: switch to whatever the ai sends once we know
            handle_command(cmd);
        }

        if (step.active) {
            step_update();
        } else if (speed_ctl.active) {
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
    cmd_mailbox = xQueueCreate(1, MSG_LEN);
    xTaskCreatePinnedToCore(motor_task, "motor", STACK_MOTOR, NULL, PRIO_MOTOR, &motor_task_handle, RT_CORE);
}

void motor_command(const char *text)
{
    char buf[MSG_LEN];
    strlcpy(buf, text, sizeof(buf));
    xQueueOverwrite(cmd_mailbox, buf);
}
