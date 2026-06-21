/*
 * Bluetooth command console for the green tracking system.
 */
#include "app_bt_console.h"

#include "../app_green_motion/app_green_motion.h"
#include "../app_green_system/app_green_system.h"
#include "../app_vision_comm/app_vision_comm.h"
#include "../bsp_bt/bluetooth.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define APP_BT_CONSOLE_LINE_SIZE (48U)
#define APP_BT_CONSOLE_STATUS_SIZE (1600U)
#define APP_BT_CONSOLE_TRACE_FOREVER_MS (0xffffffffUL)
#define APP_BT_CONSOLE_MJOG_MAX_STEPS (200L)
#define APP_BT_CONSOLE_TELEM_MIN_MS (50U)
#define APP_BT_CONSOLE_TELEM_MAX_MS (5000U)

static char gAppBtConsoleLine[APP_BT_CONSOLE_LINE_SIZE];
static char gAppBtConsoleStatus[APP_BT_CONSOLE_STATUS_SIZE];
static uint8_t gAppBtConsoleTelemetryEnabled;
static uint32_t gAppBtConsoleTelemetryPeriodMs;
static uint32_t gAppBtConsoleLastTelemetryMs;
static volatile uint32_t gAppBtConsoleMs;

static void AppBtConsole_ProcessLine(char *line);
static char *AppBtConsole_TrimLeft(char *text);
static char *AppBtConsole_NextToken(char **cursor);
static bool AppBtConsole_TokenEquals(const char *text, const char *expected);
static bool AppBtConsole_ParseU32(
    const char *text, uint32_t min_value, uint32_t max_value,
    uint32_t *value);
static bool AppBtConsole_ParseI32(
    const char *text, int32_t min_value, int32_t max_value,
    int32_t *value);
static char AppBtConsole_ToUpper(char value);
static void AppBtConsole_SendLine(const char *text);
static void AppBtConsole_SendHelp(void);
static void AppBtConsole_SendGreenStatus(void);
static void AppBtConsole_SendVisionStatus(void);
static void AppBtConsole_SendTrackStatus(void);
static void AppBtConsole_SendMotionStatus(void);
static void AppBtConsole_SendParamStatus(const char *name);
static void AppBtConsole_SetParam(const char *name, const char *value_text);
static void AppBtConsole_StartTelemetry(uint32_t period_ms);
static void AppBtConsole_StopTelemetry(void);
static void AppBtConsole_ServiceTelemetry(void);
static void AppBtConsole_StartTrace(uint32_t duration_ms);
static void AppBtConsole_StopTrace(void);
static void AppBtConsole_SendOkErr(bool ok);
static void AppBtConsole_ForwardVisionByte(uint8_t data);

void AppBtConsole_Init(void)
{
    gAppBtConsoleLine[0] = '\0';
    gAppBtConsoleStatus[0] = '\0';
    gAppBtConsoleTelemetryEnabled = 0U;
    gAppBtConsoleTelemetryPeriodMs = 0U;
    gAppBtConsoleLastTelemetryMs = 0U;
    gAppBtConsoleMs = 0U;
}

void AppBtConsole_Tick1ms(void)
{
    gAppBtConsoleMs++;
}

void AppBtConsole_Poll(void)
{
    AppBtConsole_ServiceTelemetry();

    if (!Bluetooth_readLine(gAppBtConsoleLine,
            (uint16_t) sizeof(gAppBtConsoleLine))) {
        return;
    }

    AppBtConsole_ProcessLine(gAppBtConsoleLine);
}

static void AppBtConsole_ProcessLine(char *line)
{
    char *cursor;
    char *cmd;
    char *arg;
    char *steps_arg;
    uint32_t rawMs;
    uint32_t periodMs;
    int32_t jog_steps;
    uint8_t jog_axis;

    cursor = AppBtConsole_TrimLeft(line);
    cmd = AppBtConsole_NextToken(&cursor);

    if (cmd == NULL) {
        return;
    }

    if (AppBtConsole_TokenEquals(cmd, "HELP")) {
        AppBtConsole_SendHelp();
    } else if (AppBtConsole_TokenEquals(cmd, "START1")) {
        GreenSystem_Command(GREEN_CMD_START1, GREEN_SYSTEM_SOURCE_BT);
        AppBtConsole_SendLine("OK START1");
    } else if (AppBtConsole_TokenEquals(cmd, "START2")) {
        GreenSystem_Command(GREEN_CMD_START2, GREEN_SYSTEM_SOURCE_BT);
        AppBtConsole_SendLine("OK START2");
    } else if (AppBtConsole_TokenEquals(cmd, "PAUSE")) {
        GreenSystem_Command(GREEN_CMD_PAUSE_TOGGLE, GREEN_SYSTEM_SOURCE_BT);
        AppBtConsole_SendLine("OK PAUSE");
    } else if (AppBtConsole_TokenEquals(cmd, "STOP")) {
        GreenSystem_Command(GREEN_CMD_STOP_RESET, GREEN_SYSTEM_SOURCE_BT);
        AppBtConsole_SendLine("OK STOP");
    } else if (AppBtConsole_TokenEquals(cmd, "RESET")) {
        GreenSystem_Command(GREEN_CMD_STOP_RESET, GREEN_SYSTEM_SOURCE_BT);
        AppBtConsole_SendLine("OK RESET");
    } else if (AppBtConsole_TokenEquals(cmd, "GSTAT")) {
        AppBtConsole_SendGreenStatus();
    } else if (AppBtConsole_TokenEquals(cmd, "VSTAT")) {
        AppBtConsole_SendVisionStatus();
    } else if (AppBtConsole_TokenEquals(cmd, "TSTAT")) {
        AppBtConsole_SendTrackStatus();
    } else if (AppBtConsole_TokenEquals(cmd, "MSTAT")) {
        AppBtConsole_SendMotionStatus();
    } else if (AppBtConsole_TokenEquals(cmd, "PGET")) {
        arg = AppBtConsole_NextToken(&cursor);
        AppBtConsole_SendParamStatus(arg);
    } else if (AppBtConsole_TokenEquals(cmd, "PSET")) {
        arg = AppBtConsole_NextToken(&cursor);
        steps_arg = AppBtConsole_NextToken(&cursor);
        AppBtConsole_SetParam(arg, steps_arg);
    } else if (AppBtConsole_TokenEquals(cmd, "PDEF")) {
        GreenSystem_ResetParams();
        AppBtConsole_SendLine("OK PDEF");
        AppBtConsole_SendParamStatus(NULL);
    } else if (AppBtConsole_TokenEquals(cmd, "TMON")) {
        arg = AppBtConsole_NextToken(&cursor);
        if ((arg != NULL) &&
            AppBtConsole_ParseU32(arg, APP_BT_CONSOLE_TELEM_MIN_MS,
                APP_BT_CONSOLE_TELEM_MAX_MS, &periodMs)) {
            AppBtConsole_StartTelemetry(periodMs);
        } else {
            AppBtConsole_SendLine("ERR TMON");
        }
    } else if (AppBtConsole_TokenEquals(cmd, "TMOFF")) {
        AppBtConsole_StopTelemetry();
    } else if (AppBtConsole_TokenEquals(cmd, "MSTOP")) {
        GreenMotion_StopSafe();
        AppBtConsole_SendLine("OK MSTOP");
    } else if (AppBtConsole_TokenEquals(cmd, "MJOG")) {
        arg = AppBtConsole_NextToken(&cursor);
        steps_arg = AppBtConsole_NextToken(&cursor);
        if ((arg == NULL) || (steps_arg == NULL)) {
            AppBtConsole_SendLine("ERR MJOG");
        } else {
            jog_axis = 0xffU;
            if (AppBtConsole_TokenEquals(arg, "X")) {
                jog_axis = GREEN_MOTION_AXIS_X;
            } else if (AppBtConsole_TokenEquals(arg, "Y")) {
                jog_axis = GREEN_MOTION_AXIS_Y;
            }

            if ((jog_axis != 0xffU) &&
                AppBtConsole_ParseI32(steps_arg,
                    -APP_BT_CONSOLE_MJOG_MAX_STEPS,
                    APP_BT_CONSOLE_MJOG_MAX_STEPS, &jog_steps) &&
                (jog_steps != 0) &&
                (GreenMotion_JogAxis(jog_axis, jog_steps) != 0U)) {
                AppBtConsole_SendLine("OK MJOG");
            } else {
                AppBtConsole_SendLine("ERR MJOG");
            }
        }
    } else if (AppBtConsole_TokenEquals(cmd, "VPING")) {
        AppBtConsole_SendOkErr(VisionComm_SendPing());
    } else if (AppBtConsole_TokenEquals(cmd, "VTRACK")) {
        AppBtConsole_SendOkErr(VisionComm_SendModeTrack640());
    } else if (AppBtConsole_TokenEquals(cmd, "VSTOP")) {
        AppBtConsole_SendOkErr(VisionComm_SendStop());
    } else if (AppBtConsole_TokenEquals(cmd, "TRACE")) {
        arg = AppBtConsole_NextToken(&cursor);
        if ((arg != NULL) && AppBtConsole_TokenEquals(arg, "ON")) {
            AppBtConsole_StartTrace(APP_BT_CONSOLE_TRACE_FOREVER_MS);
        } else if ((arg != NULL) && AppBtConsole_TokenEquals(arg, "OFF")) {
            AppBtConsole_StopTrace();
        } else {
            AppBtConsole_SendLine("ERR TRACE");
        }
    } else if (AppBtConsole_TokenEquals(cmd, "RAW")) {
        arg = AppBtConsole_NextToken(&cursor);
        if ((arg != NULL) &&
            AppBtConsole_ParseU32(arg, 1U, APP_BT_CONSOLE_TRACE_FOREVER_MS,
                &rawMs)) {
            AppBtConsole_StartTrace(rawMs);
        } else {
            AppBtConsole_SendLine("ERR RAW");
        }
    } else {
        AppBtConsole_SendLine("ERR CMD");
    }
}

static char *AppBtConsole_TrimLeft(char *text)
{
    while ((text != NULL) &&
           ((*text == ' ') || (*text == '\t') ||
            (*text == '\r') || (*text == '\n'))) {
        text++;
    }

    return text;
}

static char *AppBtConsole_NextToken(char **cursor)
{
    char *token;
    char *scan;

    if ((cursor == NULL) || (*cursor == NULL)) {
        return NULL;
    }

    scan = AppBtConsole_TrimLeft(*cursor);
    if ((scan == NULL) || (*scan == '\0')) {
        *cursor = scan;
        return NULL;
    }

    token = scan;
    while ((*scan != '\0') && (*scan != ' ') && (*scan != '\t') &&
           (*scan != '\r') && (*scan != '\n')) {
        scan++;
    }

    if (*scan != '\0') {
        *scan = '\0';
        scan++;
    }

    *cursor = scan;
    return token;
}

static bool AppBtConsole_TokenEquals(const char *text, const char *expected)
{
    if ((text == NULL) || (expected == NULL)) {
        return false;
    }

    while ((*text != '\0') && (*expected != '\0')) {
        if (AppBtConsole_ToUpper(*text) != AppBtConsole_ToUpper(*expected)) {
            return false;
        }
        text++;
        expected++;
    }

    return ((*text == '\0') && (*expected == '\0'));
}

static bool AppBtConsole_ParseU32(
    const char *text, uint32_t min_value, uint32_t max_value,
    uint32_t *value)
{
    uint32_t parsed = 0U;

    if ((text == NULL) || (*text == '\0') || (value == NULL)) {
        return false;
    }

    while (*text != '\0') {
        uint8_t digit;

        if ((*text < '0') || (*text > '9')) {
            return false;
        }

        digit = (uint8_t)(*text - '0');
        if (parsed > ((max_value - (uint32_t) digit) / 10U)) {
            return false;
        }

        parsed = (parsed * 10U) + (uint32_t) digit;
        text++;
    }

    if ((parsed < min_value) || (parsed > max_value)) {
        return false;
    }

    *value = parsed;
    return true;
}

static bool AppBtConsole_ParseI32(
    const char *text, int32_t min_value, int32_t max_value,
    int32_t *value)
{
    uint32_t parsed = 0U;
    uint32_t limit;
    bool negative = false;

    if ((text == NULL) || (*text == '\0') || (value == NULL) ||
        (min_value > max_value)) {
        return false;
    }

    if ((*text == '-') || (*text == '+')) {
        negative = (*text == '-');
        text++;
    }

    if (*text == '\0') {
        return false;
    }

    limit = negative ? (uint32_t)(-min_value) : (uint32_t) max_value;
    while (*text != '\0') {
        uint8_t digit;

        if ((*text < '0') || (*text > '9')) {
            return false;
        }

        digit = (uint8_t)(*text - '0');
        if (parsed > ((limit - (uint32_t) digit) / 10U)) {
            return false;
        }

        parsed = (parsed * 10U) + (uint32_t) digit;
        text++;
    }

    if (negative) {
        *value = -(int32_t) parsed;
    } else {
        *value = (int32_t) parsed;
    }

    return ((*value >= min_value) && (*value <= max_value));
}

static char AppBtConsole_ToUpper(char value)
{
    if ((value >= 'a') && (value <= 'z')) {
        return (char)(value - ('a' - 'A'));
    }

    return value;
}

static void AppBtConsole_SendLine(const char *text)
{
    if (text != NULL) {
        Bluetooth_sendString(text);
    }
    Bluetooth_sendString("\r\n");
}

static void AppBtConsole_SendHelp(void)
{
    AppBtConsole_SendLine(
        "HELP START1 START2 PAUSE STOP RESET GSTAT VSTAT TSTAT MSTAT");
    AppBtConsole_SendLine(
        "PGET [key] PSET <key> <value> PDEF TMON <ms> TMOFF");
    AppBtConsole_SendLine(
        "MSTOP MJOG <X|Y> <-200..200> VPING VTRACK VSTOP TRACE ON TRACE OFF RAW <ms>");
}

static void AppBtConsole_SendGreenStatus(void)
{
    GreenSystem_FormatStatus(gAppBtConsoleStatus,
        sizeof(gAppBtConsoleStatus));
    Bluetooth_sendString("OK GSTAT ");
    AppBtConsole_SendLine(gAppBtConsoleStatus);
}

static void AppBtConsole_SendVisionStatus(void)
{
    VisionCommStatus_t status;

    VisionComm_GetStatus(&status);
    (void) snprintf(gAppBtConsoleStatus, sizeof(gAppBtConsoleStatus),
        "OK VSTAT init=%u online=%u busy=%u stale=%u ms=%lu rx=%lu track=%lu "
        "mode=%s fps10=%u frame=%lu ack=%s ok=%u err=%s ovf=%lu txerr=%lu",
        (unsigned int) status.initialized,
        (unsigned int) status.online,
        (unsigned int) status.busy,
        (unsigned int) status.track_stale,
        (unsigned long) status.ms,
        (unsigned long) status.last_rx_ms,
        (unsigned long) status.last_track_ms,
        status.mode,
        (unsigned int) status.fps10,
        (unsigned long) status.status_frame,
        status.last_ack_cmd,
        (unsigned int) status.last_ack_ok,
        status.last_err,
        (unsigned long) status.uart_overflow_count,
        (unsigned long) status.uart_tx_timeout_count);
    AppBtConsole_SendLine(gAppBtConsoleStatus);
}

static void AppBtConsole_SendTrackStatus(void)
{
    VisionTrack_t track;

    if (!VisionComm_GetLatestTrack(&track)) {
        AppBtConsole_SendLine("ERR TSTAT NO_TRACK");
        return;
    }

    (void) snprintf(gAppBtConsoleStatus, sizeof(gAppBtConsoleStatus),
        "OK TSTAT seq=%u frame=%lu size=%ux%u valid=%u red=%u green=%u "
        "dist_valid=%u red_xy=%ld,%ld green_xy=%ld,%ld err=%ld,%ld "
        "dist=%ld conf=%u,%u lat=%u last_ms=%lu seq_match=%u err_text=%s",
        (unsigned int) track.seq,
        (unsigned long) track.frame,
        (unsigned int) track.image_width,
        (unsigned int) track.image_height,
        (unsigned int) track.track_valid,
        (unsigned int) track.red_valid,
        (unsigned int) track.green_valid,
        (unsigned int) track.dist_valid,
        (long) track.red.x,
        (long) track.red.y,
        (long) track.green.x,
        (long) track.green.y,
        (long) track.err_x,
        (long) track.err_y,
        (long) track.dist_mm,
        (unsigned int) track.red_confidence,
        (unsigned int) track.green_confidence,
        (unsigned int) track.latency_ms,
        (unsigned long) track.last_update_ms,
        (unsigned int) track.seq_match,
        track.err);
    AppBtConsole_SendLine(gAppBtConsoleStatus);
}

static void AppBtConsole_SendMotionStatus(void)
{
    GreenMotion_FormatStatus(gAppBtConsoleStatus,
        sizeof(gAppBtConsoleStatus));
    Bluetooth_sendString("OK MSTAT ");
    AppBtConsole_SendLine(gAppBtConsoleStatus);
}

static void AppBtConsole_SendParamStatus(const char *name)
{
    int32_t value;

    if ((name != NULL) && (*name != '\0')) {
        if (GreenSystem_GetParam(name, &value) != 0U) {
            (void) snprintf(gAppBtConsoleStatus,
                sizeof(gAppBtConsoleStatus), "OK PGET %s=%ld",
                name, (long) value);
            AppBtConsole_SendLine(gAppBtConsoleStatus);
        } else {
            AppBtConsole_SendLine("ERR PGET");
        }
        return;
    }

    GreenSystem_FormatParams(gAppBtConsoleStatus,
        sizeof(gAppBtConsoleStatus));
    Bluetooth_sendString("OK PGET ");
    AppBtConsole_SendLine(gAppBtConsoleStatus);
}

static void AppBtConsole_SetParam(const char *name, const char *value_text)
{
    int32_t value;
    int32_t readback;

    if ((name == NULL) || (value_text == NULL) ||
        !AppBtConsole_ParseI32(value_text, -10000, 10000, &value) ||
        (GreenSystem_SetParam(name, value) == 0U) ||
        (GreenSystem_GetParam(name, &readback) == 0U)) {
        AppBtConsole_SendLine("ERR PSET");
        return;
    }

    (void) snprintf(gAppBtConsoleStatus, sizeof(gAppBtConsoleStatus),
        "OK PSET %s=%ld", name, (long) readback);
    AppBtConsole_SendLine(gAppBtConsoleStatus);
}

static void AppBtConsole_StartTelemetry(uint32_t period_ms)
{
    gAppBtConsoleTelemetryEnabled = 1U;
    gAppBtConsoleTelemetryPeriodMs = period_ms;
    gAppBtConsoleLastTelemetryMs = gAppBtConsoleMs;
    AppBtConsole_SendLine("OK TMON");
}

static void AppBtConsole_StopTelemetry(void)
{
    gAppBtConsoleTelemetryEnabled = 0U;
    gAppBtConsoleTelemetryPeriodMs = 0U;
    AppBtConsole_SendLine("OK TMOFF");
}

static void AppBtConsole_ServiceTelemetry(void)
{
    if ((gAppBtConsoleTelemetryEnabled == 0U) ||
        (gAppBtConsoleTelemetryPeriodMs == 0U) ||
        ((uint32_t)(gAppBtConsoleMs - gAppBtConsoleLastTelemetryMs) <
            gAppBtConsoleTelemetryPeriodMs)) {
        return;
    }

    gAppBtConsoleLastTelemetryMs = gAppBtConsoleMs;
    GreenSystem_FormatStatus(gAppBtConsoleStatus,
        sizeof(gAppBtConsoleStatus));
    Bluetooth_sendString("TM GSTAT ");
    AppBtConsole_SendLine(gAppBtConsoleStatus);
}

static void AppBtConsole_StartTrace(uint32_t duration_ms)
{
    VisionComm_StartRawRxForward(
        AppBtConsole_ForwardVisionByte, duration_ms);
    AppBtConsole_SendLine("OK TRACE");
}

static void AppBtConsole_StopTrace(void)
{
    VisionComm_StopRawRxForward();
    AppBtConsole_SendLine("OK TRACE OFF");
}

static void AppBtConsole_SendOkErr(bool ok)
{
    AppBtConsole_SendLine(ok ? "OK" : "ERR");
}

static void AppBtConsole_ForwardVisionByte(uint8_t data)
{
    Bluetooth_sendByte(data);
}
