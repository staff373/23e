/*
 * TI-side vision UART protocol client.
 */
#ifndef APP_VISION_COMM_H
#define APP_VISION_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define VISION_COMM_TEXT_FIELD_SIZE (16U)

typedef void (*VisionCommRawRxCallback_t)(uint8_t data);

typedef struct {
    int16_t x;
    int16_t y;
} VisionPoint_t;

typedef struct {
    uint16_t seq;
    uint32_t frame;
    uint16_t image_width;
    uint16_t image_height;
    uint8_t valid;
    int16_t x;
    int16_t y;
    uint16_t confidence;
    uint16_t latency_ms;
    char err[VISION_COMM_TEXT_FIELD_SIZE];
    uint32_t last_update_ms;
    uint8_t seq_match;
} VisionSpot_t;

typedef struct {
    uint16_t seq;
    uint32_t frame;
    uint16_t image_width;
    uint16_t image_height;
    VisionPoint_t corner[4];
    int16_t angle10;
    uint16_t confidence;
    char err[VISION_COMM_TEXT_FIELD_SIZE];
    uint32_t last_update_ms;
    uint8_t valid;
    uint8_t seq_match;
} VisionA4Locked_t;

typedef struct {
    uint8_t initialized;
    uint8_t online;
    uint8_t busy;
    uint8_t spot_stale;

    uint16_t next_seq;
    uint16_t pending_seq;
    uint32_t ms;
    uint32_t last_rx_ms;
    uint32_t last_ack_ms;
    uint32_t last_status_ms;
    uint32_t last_spot_ms;
    uint32_t last_a4_locked_ms;

    char mode[VISION_COMM_TEXT_FIELD_SIZE];
    uint16_t fps10;
    uint32_t status_frame;
    char last_ack_cmd[VISION_COMM_TEXT_FIELD_SIZE];
    uint8_t last_ack_ok;
    char last_err[VISION_COMM_TEXT_FIELD_SIZE];

    uint32_t rx_line_count;
    uint32_t debug_line_count;
    uint32_t parse_error_count;
    uint32_t seq_mismatch_count;
    uint32_t timeout_count;
    uint32_t line_overflow_count;
    uint32_t uart_overflow_count;
    uint32_t uart_tx_timeout_count;
} VisionCommStatus_t;

void VisionComm_Init(void);
void VisionComm_Poll(void);
void VisionComm_Tick1ms(void);

void VisionComm_StartRawRxForward(
    VisionCommRawRxCallback_t callback, uint32_t duration_ms);
void VisionComm_StopRawRxForward(void);
uint8_t VisionComm_IsRawRxForwardActive(void);
uint32_t VisionComm_GetRawRxForwardByteCount(void);

bool VisionComm_SendPing(void);
bool VisionComm_SendModeSpot640(void);
bool VisionComm_SendModeA4Gray(void);
bool VisionComm_SendLockA4(void);
bool VisionComm_SendAckA4Locked(void);
bool VisionComm_SendStop(void);
bool VisionComm_SendStatus(void);

bool VisionComm_GetLatestSpot(VisionSpot_t *spot);
bool VisionComm_GetLatestA4Locked(VisionA4Locked_t *a4);
void VisionComm_GetStatus(VisionCommStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISION_COMM_H */
