#pragma once

#include "messages.h"

// wifi, websocket server and the task that sends replies
void comms_start(void);

// queues a message for the server. fine to call from motor_task, the json is made on core 0
void comms_send(const bot_msg_t *msg);

// shortcuts for {"type":"info"/"error","message":...} with printf style formatting
void comms_send_info(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void comms_send_error(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
