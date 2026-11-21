#pragma once

#include <stdbool.h>

// board directions, in clockwise order so +1 is a 90 deg cw turn and -1 is a 90 deg ccw turn
typedef enum {
    NORTH,  // 0
    EAST,   // 1
    SOUTH,  // 2
    WEST,   // 3
} dir_t;

// every kind of command motor_task understands. comms.c fills one of these in from the server's
// json (see messages.c), so motor_task never has to read text
typedef enum {
    MOTOR_CMD_STOP,       // "stop"       brake, or coast with the motors off
    MOTOR_CMD_DIRECTION,  // "direction"  go n/e/s/w on the board, turning 90 first if needed
    MOTOR_CMD_DUTY,       // "duty"       open loop duty % per wheel
    MOTOR_CMD_SPEED,      // "speed"      closed loop speed in mm/s per wheel
    MOTOR_CMD_TURN,       // "turn"       turn in place by some degrees
    MOTOR_CMD_STEP_TEST,  // "step_test"  duty or speed step test for tune.py
    MOTOR_CMD_GAINS,      // "gains"      set kp and/or ki (neither = just report them)
} motor_cmd_type_t;

// a tagged union: "type" says which one of the structs inside the union is filled in.
// only read the member that matches the type, e.g. cmd.duty.left when type == MOTOR_CMD_DUTY
typedef struct {
    motor_cmd_type_t type;
    union {
        struct { bool coast; } stop;                                 // coast = master off (EN low) instead of brake
        dir_t direction;
        struct { int left, right; } duty;                            // -100..100 %
        struct { int left, right; } speed;                           // mm/s, negative = backward
        struct { float degrees; bool clockwise; float speed_mm_s; } turn;  // speed 0 = TURN_SPEED_MM_S
        struct { bool speed; int value; int duration_ms; } step;     // speed false = duty % test, true = mm/s test
        struct { bool has_kp, has_ki; float kp, ki; } gains;
    };
} motor_cmd_t;

void motor_start(void);

// hands a command to motor_task. a newer one replaces one it hasnt gotten to yet
void motor_command(const motor_cmd_t *cmd);
