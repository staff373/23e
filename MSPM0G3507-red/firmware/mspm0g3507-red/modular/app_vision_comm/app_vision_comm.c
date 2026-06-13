/*
 * TI-side vision UART protocol client.
 */
#include "app_vision_comm.h"

#include "bsp_vision_uart.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define VISION_COMM_LINE_BUF_SIZE (192U)
#define VISION_COMM_TOKEN_MAX (24U)
#define VISION_COMM_TX_BUF_SIZE (64U)

#define VISION_COMM_ACK_TIMEOUT_MS (500U)
#define VISION_COMM_STATUS_TIMEOUT_MS (500U)
#define VISION_COMM_A4_LOCK_TIMEOUT_MS (5000U)
#define VISION_COMM_ONLINE_TIMEOUT_MS (1000U)
#define VISION_COMM_SPOT_STALE_MS (150U)

typedef enum {
    VISION_COMM_PENDING_NONE = 0,
    VISION_COMM_PENDING_ACK,
    VISION_COMM_PENDING_STATUS,
    VISION_COMM_PENDING_A4_LOCKED
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
static uint16_t gVisionSpotStreamSeq;
static uint16_t gVisionLastA4LockSeq;
static uint8_t gVisionHasSpotStreamSeq;
static uint8_t gVisionHasA4LockSeq;
static uint8_t gVisionHasSpot;
static uint8_t gVisionHasA4Locked;

static VisionCommRawRxCallback_t gVisionRawRxCallback;
static uint8_t gVisionRawRxForwardActive;
static uint32_t gVisionRawRxForwardStartMs;
static uint32_t gVisionRawRxForwardDurationMs;
static uint32_t gVisionRawRxForwardByteCount;

static VisionCommPending_t gVisionPending;
static VisionCommStatus_t gVisionStatus;
static VisionSpot_t gVisionSpot;
static VisionA4Locked_t gVisionA4Locked;

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
static void VisionComm_HandleSpot(
    uint16_t seq, char *tokens[], uint8_t tokenCount);
static void VisionComm_HandleA4Locked(
    uint16_t seq, char *tokens[], uint8_t tokenCount);
static void VisionComm_HandleErr(
    uint16_t seq, char *tokens[], uint8_t tokenCount);
static void VisionComm_ClearA4LockedCache(void);
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
    memset(&gVisionSpot, 0, sizeof(gVisionSpot));
    memset(&gVisionA4Locked, 0, sizeof(gVisionA4Locked));

    gVisionMs = 0U;
    gVisionNextSeq = 1U;
    gVisionSpotStreamSeq = 0U;
    gVisionLastA4LockSeq = 0U;
    gVisionHasSpotStreamSeq = 0U;
    gVisionHasA4LockSeq = 0U;
    gVisionHasSpot = 0U;
    gVisionHasA4Locked = 0U;
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

bool VisionComm_SendModeSpot640(void)
{
    bool sent;

    sent = VisionComm_SendRequest(VISION_COMM_PENDING_ACK,
        VISION_COMM_ACK_TIMEOUT_MS, "MODE", "MODE,SPOT640");
    if (sent) {
        gVisionSpotStreamSeq = gVisionPending.seq;
        gVisionHasSpotStreamSeq = 1U;
    }

    return sent;
}

bool VisionComm_SendModeA4Gray(void)
{
    bool sent;

    sent = VisionComm_SendRequest(VISION_COMM_PENDING_ACK,
        VISION_COMM_ACK_TIMEOUT_MS, "MODE", "MODE,A4GRAY");
    if (sent) {
        VisionComm_ClearA4LockedCache();
    }

    return sent;
}

bool VisionComm_SendLockA4(void)
{
    bool sent;

    sent = VisionComm_SendRequest(VISION_COMM_PENDING_A4_LOCKED,
        VISION_COMM_A4_LOCK_TIMEOUT_MS, "", "LOCK_A4");
    if (sent) {
        VisionComm_ClearA4LockedCache();
        gVisionLastA4LockSeq = gVisionPending.seq;
        gVisionHasA4LockSeq = 1U;
    }

    return sent;
}

bool VisionComm_SendAckA4Locked(void)
{
    return VisionComm_SendRequest(VISION_COMM_PENDING_ACK,
        VISION_COMM_ACK_TIMEOUT_MS, "ACK", "ACK,A4_LOCKED");
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

bool VisionComm_GetLatestSpot(VisionSpot_t *spot)
{
    if ((spot == NULL) || (gVisionHasSpot == 0U)) {
        return false;
    }

    *spot = gVisionSpot;
    return true;
}

bool VisionComm_GetLatestA4Locked(VisionA4Locked_t *a4)
{
    if ((a4 == NULL) || (gVisionHasA4Locked == 0U)) {
        return false;
    }

    *a4 = gVisionA4Locked;
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
    } else if (strcmp(tokens[2], "SPOT") == 0) {
        VisionComm_HandleSpot(seq, tokens, tokenCount);
    } else if (strcmp(tokens[2], "A4_LOCKED") == 0) {
        VisionComm_HandleA4Locked(seq, tokens, tokenCount);
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

static void VisionComm_HandleSpot(
    uint16_t seq, char *tokens[], uint8_t tokenCount)
{
    uint32_t frame;
    uint16_t imageWidth;
    uint16_t imageHeight;
    uint8_t valid;
    int16_t x;
    int16_t y;
    uint16_t confidence;
    uint16_t latency;
    uint8_t seqMatch;

    if (tokenCount < 12U) {
        VisionComm_RecordParseError("SPOT");
        return;
    }

    if (!VisionComm_ParseU32(tokens[3], &frame) ||
        !VisionComm_ParseU16(tokens[4], &imageWidth) ||
        !VisionComm_ParseU16(tokens[5], &imageHeight) ||
        !VisionComm_ParseU8(tokens[6], &valid) ||
        !VisionComm_ParseI16(tokens[7], &x) ||
        !VisionComm_ParseI16(tokens[8], &y) ||
        !VisionComm_ParseU16(tokens[9], &confidence) ||
        !VisionComm_ParseU16(tokens[10], &latency)) {
        VisionComm_RecordParseError("SPOTN");
        return;
    }

    seqMatch = (uint8_t)((gVisionHasSpotStreamSeq == 0U) ||
        (seq == gVisionSpotStreamSeq));
    if (seqMatch == 0U) {
        VisionComm_RecordSeqMismatch();
    }

    gVisionSpot.seq = seq;
    gVisionSpot.frame = frame;
    gVisionSpot.image_width = imageWidth;
    gVisionSpot.image_height = imageHeight;
    gVisionSpot.valid = valid;
    gVisionSpot.x = x;
    gVisionSpot.y = y;
    gVisionSpot.confidence = confidence;
    gVisionSpot.latency_ms = latency;
    VisionComm_CopyText(gVisionSpot.err,
        VISION_COMM_TEXT_FIELD_SIZE, tokens[11]);
    gVisionSpot.last_update_ms = VisionComm_NowMs();
    gVisionSpot.seq_match = seqMatch;

    gVisionHasSpot = 1U;
    gVisionStatus.last_spot_ms = gVisionSpot.last_update_ms;
}

static void VisionComm_HandleA4Locked(
    uint16_t seq, char *tokens[], uint8_t tokenCount)
{
    uint32_t frame;
    uint16_t imageWidth;
    uint16_t imageHeight;
    int16_t angle10;
    uint16_t confidence;
    VisionPoint_t corner[4];
    uint8_t seqMatch;
    uint8_t i;

    if (tokenCount < 17U) {
        VisionComm_RecordParseError("A4");
        return;
    }

    if (!VisionComm_ParseU32(tokens[3], &frame) ||
        !VisionComm_ParseU16(tokens[4], &imageWidth) ||
        !VisionComm_ParseU16(tokens[5], &imageHeight) ||
        !VisionComm_ParseI16(tokens[6], &corner[0].x) ||
        !VisionComm_ParseI16(tokens[7], &corner[0].y) ||
        !VisionComm_ParseI16(tokens[8], &corner[1].x) ||
        !VisionComm_ParseI16(tokens[9], &corner[1].y) ||
        !VisionComm_ParseI16(tokens[10], &corner[2].x) ||
        !VisionComm_ParseI16(tokens[11], &corner[2].y) ||
        !VisionComm_ParseI16(tokens[12], &corner[3].x) ||
        !VisionComm_ParseI16(tokens[13], &corner[3].y) ||
        !VisionComm_ParseI16(tokens[14], &angle10) ||
        !VisionComm_ParseU16(tokens[15], &confidence)) {
        VisionComm_RecordParseError("A4N");
        return;
    }

    seqMatch = (uint8_t)(((gVisionPending.active != 0U) &&
                             (gVisionPending.type ==
                                 VISION_COMM_PENDING_A4_LOCKED) &&
                             (seq == gVisionPending.seq)) ||
        ((gVisionHasA4LockSeq != 0U) && (seq == gVisionLastA4LockSeq)));
    if (seqMatch == 0U) {
        VisionComm_RecordSeqMismatch();
    }

    gVisionA4Locked.seq = seq;
    gVisionA4Locked.frame = frame;
    gVisionA4Locked.image_width = imageWidth;
    gVisionA4Locked.image_height = imageHeight;
    for (i = 0U; i < 4U; i++) {
        gVisionA4Locked.corner[i] = corner[i];
    }
    gVisionA4Locked.angle10 = angle10;
    gVisionA4Locked.confidence = confidence;
    VisionComm_CopyText(gVisionA4Locked.err,
        VISION_COMM_TEXT_FIELD_SIZE, tokens[16]);
    gVisionA4Locked.last_update_ms = VisionComm_NowMs();
    gVisionA4Locked.valid = 1U;
    gVisionA4Locked.seq_match = seqMatch;

    gVisionHasA4Locked = 1U;
    gVisionStatus.last_a4_locked_ms = gVisionA4Locked.last_update_ms;

    if ((gVisionPending.active != 0U) &&
        (gVisionPending.type == VISION_COMM_PENDING_A4_LOCKED)) {
        if (seq == gVisionPending.seq) {
            VisionComm_ClearPending();
        } else {
            VisionComm_RecordSeqMismatch();
        }
    }
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

static void VisionComm_ClearA4LockedCache(void)
{
    memset(&gVisionA4Locked, 0, sizeof(gVisionA4Locked));
    gVisionHasA4Locked = 0U;
    gVisionHasA4LockSeq = 0U;
    gVisionLastA4LockSeq = 0U;
    gVisionStatus.last_a4_locked_ms = 0U;
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

    if (gVisionStatus.last_spot_ms == 0U) {
        gVisionStatus.spot_stale = 1U;
    } else if ((uint32_t)(now - gVisionStatus.last_spot_ms) >
        VISION_COMM_SPOT_STALE_MS) {
        gVisionStatus.spot_stale = 1U;
    } else {
        gVisionStatus.spot_stale = 0U;
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
