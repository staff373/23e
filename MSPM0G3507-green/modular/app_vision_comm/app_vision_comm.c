/*
 * TI-side vision UART protocol client.
 */
#include "app_vision_comm.h"

#include "../bsp_vision_uart/bsp_vision_uart.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define VISION_COMM_LINE_BUF_SIZE (192U)
#define VISION_COMM_TOKEN_MAX (24U)
#define VISION_COMM_TX_BUF_SIZE (64U)

#define VISION_COMM_ACK_TIMEOUT_MS (500U)
#define VISION_COMM_STATUS_TIMEOUT_MS (500U)
#define VISION_COMM_ONLINE_TIMEOUT_MS (1000U)
#define VISION_COMM_TRACK_STALE_MS (150U)

typedef enum {
    VISION_COMM_PENDING_NONE = 0,
    VISION_COMM_PENDING_ACK,
    VISION_COMM_PENDING_STATUS
} VisionCommPendingType_t;

typedef struct {
    uint8_t active;
    VisionCommPendingType_t type;
    uint16_t seq;
    uint32_t start_ms;
    uint32_t timeout_ms;
    char ack_cmd[VISION_COMM_TEXT_FIELD_SIZE];
} VisionCommPending_t;

static volatile uint32_t gVisionMs;
static char gVisionLineBuffer[VISION_COMM_LINE_BUF_SIZE];
static uint16_t gVisionLineLength;
static uint8_t gVisionLineDiscarding;

static uint16_t gVisionNextSeq;
static uint8_t gVisionHasTrack;

static VisionCommRawRxCallback_t gVisionRawRxCallback;
static uint8_t gVisionRawRxForwardActive;
static uint32_t gVisionRawRxForwardStartMs;
static uint32_t gVisionRawRxForwardDurationMs;
static uint32_t gVisionRawRxForwardByteCount;

static VisionCommPending_t gVisionPending;
static VisionCommStatus_t gVisionStatus;
static VisionTrack_t gVisionTrack;

static uint32_t VisionComm_NowMs(void);
static void VisionComm_ResetLine(void);
static void VisionComm_ProcessByte(uint8_t data);
static void VisionComm_ProcessLine(char *line);
static uint8_t VisionComm_SplitCsv(
    char *line, char *tokens[], uint8_t maxTokens);
static void VisionComm_HandleAck(
    uint16_t seq, char *tokens[], uint8_t tokenCount);
static void VisionComm_HandleStatus(
    uint16_t seq, char *tokens[], uint8_t tokenCount);
static void VisionComm_HandleTrack(
    uint16_t seq, char *tokens[], uint8_t tokenCount);
static void VisionComm_HandleErr(
    uint16_t seq, char *tokens[], uint8_t tokenCount);
static void VisionComm_UpdateRuntimeStatus(void);
static void VisionComm_UpdateTimeouts(void);
static void VisionComm_ClearPending(void);
static void VisionComm_UpdateRawRxForward(void);
static void VisionComm_ForwardRawRxByte(uint8_t data);
static bool VisionComm_SendRequest(VisionCommPendingType_t pendingType,
    uint32_t timeout_ms, const char *ackCmd, const char *payload);
static uint16_t VisionComm_AllocSeq(void);
static void VisionComm_CopyText(
    char *dest, uint16_t destSize, const char *src);
static bool VisionComm_ParseU32(const char *text, uint32_t *value);
static bool VisionComm_ParseI32(const char *text, int32_t *value);
static bool VisionComm_ParseU16(const char *text, uint16_t *value);
static bool VisionComm_ParseI16(const char *text, int16_t *value);
static bool VisionComm_ParseU8(const char *text, uint8_t *value);
static void VisionComm_RecordParseError(const char *err);
static void VisionComm_RecordSeqMismatch(void);

void VisionComm_Init(void)
{
    memset(&gVisionPending, 0, sizeof(gVisionPending));
    memset(&gVisionStatus, 0, sizeof(gVisionStatus));
    memset(&gVisionTrack, 0, sizeof(gVisionTrack));

    gVisionMs = 0U;
    gVisionNextSeq = 1U;
    gVisionHasTrack = 0U;
    gVisionRawRxCallback = NULL;
    gVisionRawRxForwardActive = 0U;
    gVisionRawRxForwardStartMs = 0U;
    gVisionRawRxForwardDurationMs = 0U;
    gVisionRawRxForwardByteCount = 0U;
    VisionComm_ResetLine();

    VisionUart_Init();

    gVisionStatus.initialized = 1U;
    gVisionStatus.next_seq = gVisionNextSeq;
    VisionComm_CopyText(gVisionStatus.mode,
        VISION_COMM_TEXT_FIELD_SIZE, "UNKNOWN");
    VisionComm_CopyText(gVisionStatus.last_err,
        VISION_COMM_TEXT_FIELD_SIZE, "OK");
}

void VisionComm_Poll(void)
{
    uint8_t data;

    VisionComm_UpdateRawRxForward();
    while (VisionUart_ReadByte(&data)) {
        VisionComm_ForwardRawRxByte(data);
        VisionComm_ProcessByte(data);
    }
    VisionComm_UpdateRawRxForward();

    VisionComm_UpdateTimeouts();
    VisionComm_UpdateRuntimeStatus();
}

void VisionComm_Tick1ms(void)
{
    gVisionMs++;
}

void VisionComm_StartRawRxForward(
    VisionCommRawRxCallback_t callback, uint32_t duration_ms)
{
    gVisionRawRxCallback = callback;
    gVisionRawRxForwardByteCount = 0U;

    if ((callback == NULL) || (duration_ms == 0U)) {
        VisionComm_StopRawRxForward();
        return;
    }

    gVisionRawRxForwardStartMs = VisionComm_NowMs();
    gVisionRawRxForwardDurationMs = duration_ms;
    gVisionRawRxForwardActive = 1U;
}

void VisionComm_StopRawRxForward(void)
{
    gVisionRawRxForwardActive = 0U;
    gVisionRawRxForwardDurationMs = 0U;
    gVisionRawRxCallback = NULL;
}

uint8_t VisionComm_IsRawRxForwardActive(void)
{
    VisionComm_UpdateRawRxForward();
    return gVisionRawRxForwardActive;
}

uint32_t VisionComm_GetRawRxForwardByteCount(void)
{
    return gVisionRawRxForwardByteCount;
}

bool VisionComm_SendPing(void)
{
    return VisionComm_SendRequest(VISION_COMM_PENDING_ACK,
        VISION_COMM_ACK_TIMEOUT_MS, "PING", "PING");
}

bool VisionComm_SendModeTrack640(void)
{
    bool sent;

    sent = VisionComm_SendRequest(VISION_COMM_PENDING_ACK,
        VISION_COMM_ACK_TIMEOUT_MS, "MODE", "MODE,TRACK640");

    return sent;
}

bool VisionComm_SendStop(void)
{
    return VisionComm_SendRequest(VISION_COMM_PENDING_ACK,
        VISION_COMM_ACK_TIMEOUT_MS, "STOP", "STOP");
}

bool VisionComm_SendStatus(void)
{
    return VisionComm_SendRequest(VISION_COMM_PENDING_STATUS,
        VISION_COMM_STATUS_TIMEOUT_MS, "", "STATUS");
}

bool VisionComm_GetLatestTrack(VisionTrack_t *track)
{
    if ((track == NULL) || (gVisionHasTrack == 0U)) {
        return false;
    }

    *track = gVisionTrack;
    return true;
}

void VisionComm_GetStatus(VisionCommStatus_t *status)
{
    if (status == NULL) {
        return;
    }

    VisionComm_UpdateRuntimeStatus();
    *status = gVisionStatus;
}

void VisionComm_ResetForNewRun(void)
{
    VisionComm_ClearPending();
    VisionComm_ResetLine();
    VisionUart_ClearRxBuffer();
    memset(&gVisionTrack, 0, sizeof(gVisionTrack));
    gVisionHasTrack = 0U;
    gVisionStatus.last_track_ms = 0U;
    gVisionStatus.track_stale = 1U;
    VisionComm_CopyText(gVisionStatus.last_err,
        VISION_COMM_TEXT_FIELD_SIZE, "OK");
    VisionComm_UpdateRuntimeStatus();
}

static uint32_t VisionComm_NowMs(void)
{
    return gVisionMs;
}

static void VisionComm_ResetLine(void)
{
    gVisionLineLength = 0U;
    gVisionLineDiscarding = 0U;
    gVisionLineBuffer[0] = '\0';
}

static void VisionComm_ProcessByte(uint8_t data)
{
    if (gVisionLineDiscarding != 0U) {
        if (data == '\n') {
            VisionComm_ResetLine();
        }
        return;
    }

    if (data == '\r') {
        return;
    }

    if (data == '\n') {
        gVisionLineBuffer[gVisionLineLength] = '\0';
        VisionComm_ProcessLine(gVisionLineBuffer);
        VisionComm_ResetLine();
        return;
    }

    if (gVisionLineLength >= (uint16_t) (VISION_COMM_LINE_BUF_SIZE - 1U)) {
        gVisionStatus.line_overflow_count++;
        gVisionLineDiscarding = 1U;
        gVisionLineLength = 0U;
        return;
    }

    gVisionLineBuffer[gVisionLineLength] = (char) data;
    gVisionLineLength++;
    gVisionLineBuffer[gVisionLineLength] = '\0';
}

static void VisionComm_ProcessLine(char *line)
{
    char *tokens[VISION_COMM_TOKEN_MAX];
    uint8_t tokenCount;
    uint16_t seq;

    if ((line == NULL) || (line[0] == '\0')) {
        return;
    }

    if (line[0] == '#') {
        gVisionStatus.debug_line_count++;
        return;
    }

    tokenCount = VisionComm_SplitCsv(line, tokens, VISION_COMM_TOKEN_MAX);
    if (tokenCount < 3U) {
        VisionComm_RecordParseError("SHORT");
        return;
    }

    if (strcmp(tokens[0], "<C") != 0) {
        VisionComm_RecordParseError("PREFIX");
        return;
    }

    if (!VisionComm_ParseU16(tokens[1], &seq)) {
        VisionComm_RecordParseError("SEQ");
        return;
    }

    gVisionStatus.rx_line_count++;
    gVisionStatus.last_rx_ms = VisionComm_NowMs();
    gVisionStatus.online = 1U;

    if (strcmp(tokens[2], "ACK") == 0) {
        VisionComm_HandleAck(seq, tokens, tokenCount);
    } else if (strcmp(tokens[2], "STATUS") == 0) {
        VisionComm_HandleStatus(seq, tokens, tokenCount);
    } else if (strcmp(tokens[2], "TRACK") == 0) {
        VisionComm_HandleTrack(seq, tokens, tokenCount);
    } else if (strcmp(tokens[2], "ERR") == 0) {
        VisionComm_HandleErr(seq, tokens, tokenCount);
    } else {
        VisionComm_RecordParseError("TYPE");
    }
}

static uint8_t VisionComm_SplitCsv(
    char *line, char *tokens[], uint8_t maxTokens)
{
    char *cursor;
    uint8_t count;

    if ((line == NULL) || (tokens == NULL) || (maxTokens == 0U)) {
        return 0U;
    }

    count = 1U;
    tokens[0] = line;
    cursor = line;
    while (*cursor != '\0') {
        if (*cursor == ',') {
            *cursor = '\0';
            if (count >= maxTokens) {
                return count;
            }
            tokens[count] = cursor + 1;
            count++;
        }
        cursor++;
    }

    return count;
}

static void VisionComm_HandleAck(
    uint16_t seq, char *tokens[], uint8_t tokenCount)
{
    uint8_t ok;

    if (tokenCount < 6U) {
        VisionComm_RecordParseError("ACK");
        return;
    }

    if (!VisionComm_ParseU8(tokens[4], &ok)) {
        VisionComm_RecordParseError("ACKOK");
        return;
    }

    VisionComm_CopyText(gVisionStatus.last_ack_cmd,
        VISION_COMM_TEXT_FIELD_SIZE, tokens[3]);
    gVisionStatus.last_ack_ok = ok;
    VisionComm_CopyText(gVisionStatus.last_err,
        VISION_COMM_TEXT_FIELD_SIZE, tokens[5]);
    gVisionStatus.last_ack_ms = VisionComm_NowMs();

    if (gVisionPending.active == 0U) {
        return;
    }

    if (seq != gVisionPending.seq) {
        VisionComm_RecordSeqMismatch();
        return;
    }

    if (gVisionPending.type != VISION_COMM_PENDING_ACK) {
        return;
    }

    if ((gVisionPending.ack_cmd[0] != '\0') &&
        (strcmp(tokens[3], gVisionPending.ack_cmd) != 0)) {
        VisionComm_RecordSeqMismatch();
        return;
    }

    VisionComm_ClearPending();
}

static void VisionComm_HandleStatus(
    uint16_t seq, char *tokens[], uint8_t tokenCount)
{
    uint16_t fps10;
    uint32_t frame;

    if (tokenCount < 7U) {
        VisionComm_RecordParseError("STATUS");
        return;
    }

    if (!VisionComm_ParseU16(tokens[4], &fps10) ||
        !VisionComm_ParseU32(tokens[5], &frame)) {
        VisionComm_RecordParseError("STATUSN");
        return;
    }

    VisionComm_CopyText(gVisionStatus.mode,
        VISION_COMM_TEXT_FIELD_SIZE, tokens[3]);
    gVisionStatus.fps10 = fps10;
    gVisionStatus.status_frame = frame;
    VisionComm_CopyText(gVisionStatus.last_err,
        VISION_COMM_TEXT_FIELD_SIZE, tokens[6]);
    gVisionStatus.last_status_ms = VisionComm_NowMs();

    if ((gVisionPending.active != 0U) &&
        (gVisionPending.type == VISION_COMM_PENDING_STATUS)) {
        if (seq == gVisionPending.seq) {
            VisionComm_ClearPending();
        } else {
            VisionComm_RecordSeqMismatch();
        }
    }
}

static void VisionComm_HandleTrack(
    uint16_t seq, char *tokens[], uint8_t tokenCount)
{
    uint32_t frame;
    uint16_t imageWidth;
    uint16_t imageHeight;
    uint8_t trackValid;
    uint8_t redValid;
    uint8_t greenValid;
    uint8_t distValid;
    int16_t redX;
    int16_t redY;
    int16_t greenX;
    int16_t greenY;
    int16_t errX;
    int16_t errY;
    int16_t distMm;
    uint16_t redConfidence;
    uint16_t greenConfidence;
    uint16_t latency;
    uint8_t seqMatch;

    if (tokenCount < 21U) {
        VisionComm_RecordParseError("TRACK");
        return;
    }

    if (!VisionComm_ParseU32(tokens[3], &frame) ||
        !VisionComm_ParseU16(tokens[4], &imageWidth) ||
        !VisionComm_ParseU16(tokens[5], &imageHeight) ||
        !VisionComm_ParseU8(tokens[6], &trackValid) ||
        !VisionComm_ParseU8(tokens[7], &redValid) ||
        !VisionComm_ParseU8(tokens[8], &greenValid) ||
        !VisionComm_ParseU8(tokens[9], &distValid) ||
        !VisionComm_ParseI16(tokens[10], &redX) ||
        !VisionComm_ParseI16(tokens[11], &redY) ||
        !VisionComm_ParseI16(tokens[12], &greenX) ||
        !VisionComm_ParseI16(tokens[13], &greenY) ||
        !VisionComm_ParseI16(tokens[14], &errX) ||
        !VisionComm_ParseI16(tokens[15], &errY) ||
        !VisionComm_ParseI16(tokens[16], &distMm) ||
        !VisionComm_ParseU16(tokens[17], &redConfidence) ||
        !VisionComm_ParseU16(tokens[18], &greenConfidence) ||
        !VisionComm_ParseU16(tokens[19], &latency)) {
        VisionComm_RecordParseError("TRACKN");
        return;
    }

    seqMatch = 1U;

    gVisionTrack.seq = seq;
    gVisionTrack.frame = frame;
    gVisionTrack.image_width = imageWidth;
    gVisionTrack.image_height = imageHeight;
    gVisionTrack.track_valid = trackValid;
    gVisionTrack.red_valid = redValid;
    gVisionTrack.green_valid = greenValid;
    gVisionTrack.dist_valid = distValid;
    gVisionTrack.red.x = redX;
    gVisionTrack.red.y = redY;
    gVisionTrack.green.x = greenX;
    gVisionTrack.green.y = greenY;
    gVisionTrack.err_x = errX;
    gVisionTrack.err_y = errY;
    gVisionTrack.dist_mm = distMm;
    gVisionTrack.red_confidence = redConfidence;
    gVisionTrack.green_confidence = greenConfidence;
    gVisionTrack.latency_ms = latency;
    VisionComm_CopyText(gVisionTrack.err,
        VISION_COMM_TEXT_FIELD_SIZE, tokens[20]);
    gVisionTrack.last_update_ms = VisionComm_NowMs();
    gVisionTrack.seq_match = seqMatch;

    gVisionHasTrack = 1U;
    gVisionStatus.last_track_ms = gVisionTrack.last_update_ms;
}

static void VisionComm_HandleErr(
    uint16_t seq, char *tokens[], uint8_t tokenCount)
{
    if (tokenCount < 4U) {
        VisionComm_RecordParseError("ERR");
        return;
    }

    VisionComm_CopyText(gVisionStatus.last_err,
        VISION_COMM_TEXT_FIELD_SIZE, tokens[3]);

    if (gVisionPending.active == 0U) {
        return;
    }

    if (seq == gVisionPending.seq) {
        VisionComm_ClearPending();
    } else {
        VisionComm_RecordSeqMismatch();
    }
}

static void VisionComm_UpdateRuntimeStatus(void)
{
    uint32_t now = VisionComm_NowMs();

    gVisionStatus.ms = now;
    gVisionStatus.next_seq = gVisionNextSeq;
    gVisionStatus.busy = gVisionPending.active;
    gVisionStatus.pending_seq =
        (gVisionPending.active != 0U) ? gVisionPending.seq : 0U;
    gVisionStatus.uart_overflow_count = VisionUart_GetOverflowCount();
    gVisionStatus.uart_tx_timeout_count = VisionUart_GetTxTimeoutCount();

    if (gVisionStatus.last_rx_ms == 0U) {
        gVisionStatus.online = 0U;
    } else if ((uint32_t)(now - gVisionStatus.last_rx_ms) >
        VISION_COMM_ONLINE_TIMEOUT_MS) {
        gVisionStatus.online = 0U;
    } else {
        gVisionStatus.online = 1U;
    }

    if (gVisionStatus.last_track_ms == 0U) {
        gVisionStatus.track_stale = 1U;
    } else if ((uint32_t)(now - gVisionStatus.last_track_ms) >
        VISION_COMM_TRACK_STALE_MS) {
        gVisionStatus.track_stale = 1U;
    } else {
        gVisionStatus.track_stale = 0U;
    }
}

static void VisionComm_UpdateTimeouts(void)
{
    uint32_t now;

    if (gVisionPending.active == 0U) {
        return;
    }

    now = VisionComm_NowMs();
    if ((uint32_t)(now - gVisionPending.start_ms) <=
        gVisionPending.timeout_ms) {
        return;
    }

    gVisionStatus.timeout_count++;
    VisionComm_CopyText(gVisionStatus.last_err,
        VISION_COMM_TEXT_FIELD_SIZE, "TIMEOUT");
    VisionComm_ClearPending();
}

static void VisionComm_ClearPending(void)
{
    memset(&gVisionPending, 0, sizeof(gVisionPending));
    gVisionStatus.busy = 0U;
    gVisionStatus.pending_seq = 0U;
}

static void VisionComm_UpdateRawRxForward(void)
{
    if (gVisionRawRxForwardActive == 0U) {
        return;
    }

    if ((uint32_t)(VisionComm_NowMs() - gVisionRawRxForwardStartMs) <
        gVisionRawRxForwardDurationMs) {
        return;
    }

    VisionComm_StopRawRxForward();
}

static void VisionComm_ForwardRawRxByte(uint8_t data)
{
    VisionComm_UpdateRawRxForward();
    if ((gVisionRawRxForwardActive == 0U) ||
        (gVisionRawRxCallback == NULL)) {
        return;
    }

    gVisionRawRxCallback(data);
    gVisionRawRxForwardByteCount++;
}

static bool VisionComm_SendRequest(VisionCommPendingType_t pendingType,
    uint32_t timeout_ms, const char *ackCmd, const char *payload)
{
    char txBuffer[VISION_COMM_TX_BUF_SIZE];
    uint16_t seq;
    int written;

    if ((gVisionStatus.initialized == 0U) || (payload == NULL)) {
        return false;
    }

    VisionComm_UpdateTimeouts();
    if (gVisionPending.active != 0U) {
        VisionComm_UpdateRuntimeStatus();
        return false;
    }

    seq = VisionComm_AllocSeq();
    written = snprintf(txBuffer, sizeof(txBuffer), ">T,%u,%s\r\n",
        (unsigned int) seq, payload);
    if ((written <= 0) || (written >= (int) sizeof(txBuffer))) {
        VisionComm_RecordParseError("TXFMT");
        return false;
    }

    if (!VisionUart_SendString(txBuffer)) {
        VisionComm_CopyText(gVisionStatus.last_err,
            VISION_COMM_TEXT_FIELD_SIZE, "TX");
        VisionComm_UpdateRuntimeStatus();
        return false;
    }

    memset(&gVisionPending, 0, sizeof(gVisionPending));
    gVisionPending.active = 1U;
    gVisionPending.type = pendingType;
    gVisionPending.seq = seq;
    gVisionPending.start_ms = VisionComm_NowMs();
    gVisionPending.timeout_ms = timeout_ms;
    VisionComm_CopyText(gVisionPending.ack_cmd,
        VISION_COMM_TEXT_FIELD_SIZE, ackCmd);

    VisionComm_UpdateRuntimeStatus();
    return true;
}

static uint16_t VisionComm_AllocSeq(void)
{
    uint16_t seq = gVisionNextSeq;

    gVisionNextSeq++;
    if (gVisionNextSeq == 0U) {
        gVisionNextSeq = 1U;
    }

    return seq;
}

static void VisionComm_CopyText(
    char *dest, uint16_t destSize, const char *src)
{
    uint16_t i;

    if ((dest == NULL) || (destSize == 0U)) {
        return;
    }

    if (src == NULL) {
        dest[0] = '\0';
        return;
    }

    for (i = 0U; i < (uint16_t)(destSize - 1U); i++) {
        if (src[i] == '\0') {
            break;
        }
        dest[i] = src[i];
    }
    dest[i] = '\0';
}

static bool VisionComm_ParseU32(const char *text, uint32_t *value)
{
    uint32_t parsed = 0U;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return false;
    }

    while (*text != '\0') {
        uint32_t digit;

        if ((*text < '0') || (*text > '9')) {
            return false;
        }

        digit = (uint32_t)(*text - '0');
        if (parsed > ((0xFFFFFFFFU - digit) / 10U)) {
            return false;
        }

        parsed = (parsed * 10U) + digit;
        text++;
    }

    *value = parsed;
    return true;
}

static bool VisionComm_ParseI32(const char *text, int32_t *value)
{
    uint8_t negative = 0U;
    uint32_t magnitude;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return false;
    }

    if (*text == '-') {
        negative = 1U;
        text++;
        if (*text == '\0') {
            return false;
        }
    }

    if (!VisionComm_ParseU32(text, &magnitude)) {
        return false;
    }

    if (negative != 0U) {
        if (magnitude > 2147483648U) {
            return false;
        }
        if (magnitude == 2147483648U) {
            *value = (int32_t)(-2147483647 - 1);
        } else {
            *value = -(int32_t) magnitude;
        }
    } else {
        if (magnitude > 2147483647U) {
            return false;
        }
        *value = (int32_t) magnitude;
    }

    return true;
}

static bool VisionComm_ParseU16(const char *text, uint16_t *value)
{
    uint32_t parsed;

    if (!VisionComm_ParseU32(text, &parsed) || (parsed > 65535U)) {
        return false;
    }

    *value = (uint16_t) parsed;
    return true;
}

static bool VisionComm_ParseI16(const char *text, int16_t *value)
{
    int32_t parsed;

    if (!VisionComm_ParseI32(text, &parsed) ||
        (parsed < -32768L) || (parsed > 32767L)) {
        return false;
    }

    *value = (int16_t) parsed;
    return true;
}

static bool VisionComm_ParseU8(const char *text, uint8_t *value)
{
    uint32_t parsed;

    if (!VisionComm_ParseU32(text, &parsed) || (parsed > 255U)) {
        return false;
    }

    *value = (uint8_t) parsed;
    return true;
}

static void VisionComm_RecordParseError(const char *err)
{
    gVisionStatus.parse_error_count++;
    VisionComm_CopyText(gVisionStatus.last_err,
        VISION_COMM_TEXT_FIELD_SIZE, err);
}

static void VisionComm_RecordSeqMismatch(void)
{
    gVisionStatus.seq_mismatch_count++;
    VisionComm_CopyText(gVisionStatus.last_err,
        VISION_COMM_TEXT_FIELD_SIZE, "SEQ_MISMATCH");
}
