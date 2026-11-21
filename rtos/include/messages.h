#pragma once

// json messages between the robot and the server (console.py / tune.py).
// every message is one json object whose "type" field says what the rest of it holds, e.g.
//   server -> robot   {"type":"duty","left":30,"right":30}
//   robot -> server   {"type":"encoders","left":1234,"right":1200}
// this is the c side of server/messages.py. if you add or change a type here, change it there too.
//
// in c a message is a tagged union: an enum saying which kind it is, plus a union holding the
// fields for each kind. msg_parse() turns json into one of those, msg_serialize() turns one back
// into json. nothing outside messages.c ever touches json.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor.h"

#define MSG_TEXT_LEN 160   // longest text in an info/error message (the json around it adds ~30)

// ---------- server -> robot ----------

typedef enum {
    SERVER_MSG_REGISTRATION_RESPONSE,  // "registration_response"  {success, message}
    SERVER_MSG_MOTOR,                  // any motor command type ("stop", "duty", ...), see motor_cmd_type_t
} server_msg_type_t;

typedef struct {
    server_msg_type_t type;
    union {
        struct { bool success; char message[MSG_TEXT_LEN]; } registration_response;
        motor_cmd_t motor;
    };
} server_msg_t;

// reads a json message from the server into out. returns false if it isnt valid json, has a type we
// dont know, or is missing/has a wrong field, and writes the reason into err
bool msg_parse(const char *json, server_msg_t *out, char *err, size_t err_len);

// ---------- robot -> server ----------

typedef enum {
    BOT_MSG_REGISTRATION,  // "registration"  {client_id, role}   has to be the first message
    BOT_MSG_INFO,          // "info"          {message}
    BOT_MSG_ERROR,         // "error"         {message}
    BOT_MSG_ENCODERS,      // "encoders"      {left, right}       raw counts
    BOT_MSG_STEP_LOG,      // "step_log"      {t, value, left, right[, left_duty, right_duty]}
    BOT_MSG_STEP_DONE,     // "step_done"     {}                  last message of a step test
    BOT_MSG_IMU,           // "imu"           {t, a:[x,y,z], g:[x,y,z]}  ms, mg, deg/s
} bot_msg_type_t;

typedef struct {
    bot_msg_type_t type;
    union {
        char text[MSG_TEXT_LEN];   // info and error
        struct { int32_t left, right; } encoders;
        struct {
            uint32_t t_ms;
            int32_t value;         // commanded duty % or mm/s
            int32_t left, right;   // measured counts/s (duty test) or mm/s (speed test)
            bool has_duty;         // speed tests also send the duty the PI chose
            float left_duty, right_duty;
        } step_log;
        struct { int64_t t_ms; float accel_mg[3]; float gyro_dps[3]; } imu;
    };
} bot_msg_t;

// writes m as json into buf. returns the length, or -1 if it didnt fit
int msg_serialize(const bot_msg_t *m, char *buf, size_t len);
