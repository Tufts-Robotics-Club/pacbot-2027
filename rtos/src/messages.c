#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"

#include "messages.h"

// cJSON turns json text into a tree of cJSON nodes (cJSON_Parse) and a tree back into text
// (cJSON_Print...). both directions malloc, so only call these from core 0 tasks, never motor_task

// ---------- reading fields ----------
// each one returns false and writes why into err if the field is missing or the wrong kind

static bool need_number(const cJSON *root, const char *key, double *out, char *err, size_t err_len)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(item)) {
        snprintf(err, err_len, "\"%s\" must be a number", key);
        return false;
    }
    *out = item->valuedouble;
    return true;
}

static bool need_int(const cJSON *root, const char *key, int *out, char *err, size_t err_len)
{
    double d;
    if (!need_number(root, key, &d, err, err_len)) {
        return false;
    }
    *out = (int)lround(d);
    return true;
}

static bool need_bool(const cJSON *root, const char *key, bool *out, char *err, size_t err_len)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsBool(item)) {
        snprintf(err, err_len, "\"%s\" must be true or false", key);
        return false;
    }
    *out = cJSON_IsTrue(item);
    return true;
}

static bool need_string(const cJSON *root, const char *key, const char **out, char *err, size_t err_len)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsString(item)) {
        snprintf(err, err_len, "\"%s\" must be a string", key);
        return false;
    }
    *out = item->valuestring;
    return true;
}

// optional fields: true if it's there (and fills out), false if it's missing or null
static bool has_number(const cJSON *root, const char *key, double *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsNumber(item)) {
        *out = item->valuedouble;
        return true;
    }
    return false;
}

// ---------- server -> robot ----------

static bool parse_registration_response(const cJSON *root, server_msg_t *out, char *err, size_t err_len)
{
    const char *message;
    if (!need_bool(root, "success", &out->registration_response.success, err, err_len) ||
        !need_string(root, "message", &message, err, err_len)) {
        return false;
    }
    strlcpy(out->registration_response.message, message, sizeof(out->registration_response.message));
    return true;
}

// one entry per motor command. the "type" string on the wire -> the enum value
static const struct {
    const char *name;
    motor_cmd_type_t type;
} MOTOR_TYPES[] = {
    {"stop",      MOTOR_CMD_STOP},
    {"direction", MOTOR_CMD_DIRECTION},
    {"duty",      MOTOR_CMD_DUTY},
    {"speed",     MOTOR_CMD_SPEED},
    {"turn",      MOTOR_CMD_TURN},
    {"step_test", MOTOR_CMD_STEP_TEST},
    {"gains",     MOTOR_CMD_GAINS},
};

static bool parse_motor(const cJSON *root, motor_cmd_type_t type, motor_cmd_t *cmd, char *err, size_t err_len)
{
    cmd->type = type;
    double d;
    const char *s;

    switch (type) {
    case MOTOR_CMD_STOP:
        // {"type":"stop","coast":false}   coast is optional, default brake
        cmd->stop.coast = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root, "coast"));
        return true;

    case MOTOR_CMD_DIRECTION:
        // {"type":"direction","direction":"n"}
        if (!need_string(root, "direction", &s, err, err_len)) {
            return false;
        }
        if (strcmp(s, "n") == 0)      cmd->direction = NORTH;
        else if (strcmp(s, "e") == 0) cmd->direction = EAST;
        else if (strcmp(s, "s") == 0) cmd->direction = SOUTH;
        else if (strcmp(s, "w") == 0) cmd->direction = WEST;
        else {
            snprintf(err, err_len, "\"direction\" must be n, e, s or w");
            return false;
        }
        return true;

    case MOTOR_CMD_DUTY:
        // {"type":"duty","left":30,"right":30}
        return need_int(root, "left", &cmd->duty.left, err, err_len) &&
               need_int(root, "right", &cmd->duty.right, err, err_len);

    case MOTOR_CMD_SPEED:
        // {"type":"speed","left":200,"right":200}
        return need_int(root, "left", &cmd->speed.left, err, err_len) &&
               need_int(root, "right", &cmd->speed.right, err, err_len);

    case MOTOR_CMD_TURN:
        // {"type":"turn","degrees":90,"clockwise":true,"speed":150}   speed is optional
        if (!need_number(root, "degrees", &d, err, err_len) ||
            !need_bool(root, "clockwise", &cmd->turn.clockwise, err, err_len)) {
            return false;
        }
        cmd->turn.degrees = (float)d;
        cmd->turn.speed_mm_s = has_number(root, "speed", &d) ? (float)d : 0;   // 0 = motor.c's default
        return true;

    case MOTOR_CMD_STEP_TEST:
        // {"type":"step_test","mode":"duty","value":40,"duration_ms":1000}
        if (!need_string(root, "mode", &s, err, err_len)) {
            return false;
        }
        if (strcmp(s, "duty") != 0 && strcmp(s, "speed") != 0) {
            snprintf(err, err_len, "\"mode\" must be duty or speed");
            return false;
        }
        cmd->step.speed = strcmp(s, "speed") == 0;
        return need_int(root, "value", &cmd->step.value, err, err_len) &&
               need_int(root, "duration_ms", &cmd->step.duration_ms, err, err_len);

    case MOTOR_CMD_GAINS:
        // {"type":"gains","kp":0.05,"ki":0.5}   both optional, {"type":"gains"} just reports them
        cmd->gains.has_kp = has_number(root, "kp", &d);
        cmd->gains.kp = cmd->gains.has_kp ? (float)d : 0;
        cmd->gains.has_ki = has_number(root, "ki", &d);
        cmd->gains.ki = cmd->gains.has_ki ? (float)d : 0;
        if ((cmd->gains.has_kp && cmd->gains.kp < 0) || (cmd->gains.has_ki && cmd->gains.ki < 0)) {
            snprintf(err, err_len, "gains must be 0 or more");
            return false;
        }
        return true;
    }

    snprintf(err, err_len, "motor command type not handled");
    return false;
}

bool msg_parse(const char *json, server_msg_t *out, char *err, size_t err_len)
{
    memset(out, 0, sizeof(*out));

    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        snprintf(err, err_len, "not valid json");
        return false;
    }

    bool ok = false;
    const char *type;

    if (!cJSON_IsObject(root)) {
        snprintf(err, err_len, "not a json object");
    } else if (!need_string(root, "type", &type, err, err_len)) {
        // err already says what's wrong
    } else if (strcmp(type, "registration_response") == 0) {
        out->type = SERVER_MSG_REGISTRATION_RESPONSE;
        ok = parse_registration_response(root, out, err, err_len);
    } else {
        bool known = false;
        for (size_t i = 0; i < sizeof(MOTOR_TYPES) / sizeof(MOTOR_TYPES[0]); i++) {
            if (strcmp(type, MOTOR_TYPES[i].name) == 0) {
                known = true;
                out->type = SERVER_MSG_MOTOR;
                ok = parse_motor(root, MOTOR_TYPES[i].type, &out->motor, err, err_len);
                break;
            }
        }
        if (!known) {
            snprintf(err, err_len, "unknown message type \"%.40s\"", type);
        }
    }

    // free the tree cJSON_Parse made. type points into it, so this has to come after we're done with it
    cJSON_Delete(root);
    return ok;
}

// ---------- robot -> server ----------

// cJSON prints numbers with up to 17 digits, so 12.3f comes out as 12.300000190734863.
// rounding in double first makes it print the short version
static double round_to(double x, int decimals)
{
    double scale = pow(10, decimals);
    return round(x * scale) / scale;
}

static cJSON *add_float_array(cJSON *root, const char *key, const float *v, int n, int decimals)
{
    cJSON *arr = cJSON_AddArrayToObject(root, key);
    for (int i = 0; i < n; i++) {
        cJSON_AddItemToArray(arr, cJSON_CreateNumber(round_to(v[i], decimals)));
    }
    return arr;
}

int msg_serialize(const bot_msg_t *m, char *buf, size_t len)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return -1;
    }

    switch (m->type) {
    case BOT_MSG_REGISTRATION:
        cJSON_AddStringToObject(root, "type", "registration");
        cJSON_AddStringToObject(root, "client_id", "1");
        cJSON_AddStringToObject(root, "role", "bot");
        break;

    case BOT_MSG_INFO:
    case BOT_MSG_ERROR:
        cJSON_AddStringToObject(root, "type", m->type == BOT_MSG_INFO ? "info" : "error");
        cJSON_AddStringToObject(root, "message", m->text);   // cJSON escapes quotes etc for us
        break;

    case BOT_MSG_ENCODERS:
        cJSON_AddStringToObject(root, "type", "encoders");
        cJSON_AddNumberToObject(root, "left", m->encoders.left);
        cJSON_AddNumberToObject(root, "right", m->encoders.right);
        break;

    case BOT_MSG_STEP_LOG:
        cJSON_AddStringToObject(root, "type", "step_log");
        cJSON_AddNumberToObject(root, "t", m->step_log.t_ms);
        cJSON_AddNumberToObject(root, "value", m->step_log.value);
        cJSON_AddNumberToObject(root, "left", m->step_log.left);
        cJSON_AddNumberToObject(root, "right", m->step_log.right);
        if (m->step_log.has_duty) {
            cJSON_AddNumberToObject(root, "left_duty", round_to(m->step_log.left_duty, 1));
            cJSON_AddNumberToObject(root, "right_duty", round_to(m->step_log.right_duty, 1));
        }
        break;

    case BOT_MSG_STEP_DONE:
        cJSON_AddStringToObject(root, "type", "step_done");
        break;

    case BOT_MSG_IMU:
        // same fields as the club's imu code, plus the type
        cJSON_AddStringToObject(root, "type", "imu");
        cJSON_AddNumberToObject(root, "t", (double)m->imu.t_ms);
        add_float_array(root, "a", m->imu.accel_mg, 3, 1);
        add_float_array(root, "g", m->imu.gyro_dps, 3, 2);
        break;
    }

    // prints straight into buf instead of mallocing a new string. false = didnt fit
    // (cJSON wants a few bytes of slack past the end, hence the -5)
    bool ok = len > 5 && cJSON_PrintPreallocated(root, buf, (int)len - 5, false);
    cJSON_Delete(root);
    return ok ? (int)strlen(buf) : -1;
}
