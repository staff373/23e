/*
 * Top-level green tracking system state machine.
 */
#ifndef APP_GREEN_SYSTEM_H
#define APP_GREEN_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef enum {
    GREEN_SYSTEM_STATE_IDLE = 0,
    GREEN_SYSTEM_STATE_PREPARE,
    GREEN_SYSTEM_STATE_RUN_ACTIVE,
    GREEN_SYSTEM_STATE_PAUSED,
    GREEN_SYSTEM_STATE_FAULT
} GreenSystem_State_t;

typedef enum {
    GREEN_CMD_START1 = 0,
    GREEN_CMD_START2,
    GREEN_CMD_PAUSE_TOGGLE,
    GREEN_CMD_STOP_RESET,
    GREEN_CMD_STATUS
} GreenSystem_Command_t;

typedef enum {
    GREEN_SYSTEM_SOURCE_NONE = 0,
    GREEN_SYSTEM_SOURCE_KEY,
    GREEN_SYSTEM_SOURCE_BT
} GreenSystem_CommandSource_t;

void GreenSystem_Init(void);
void GreenSystem_Tick1ms(void);
void GreenSystem_Poll(void);

void GreenSystem_Command(
    GreenSystem_Command_t cmd, GreenSystem_CommandSource_t source);

GreenSystem_State_t GreenSystem_GetState(void);
const char *GreenSystem_GetStateName(void);
void GreenSystem_ResetParams(void);
uint8_t GreenSystem_SetParam(const char *name, int32_t value);
uint8_t GreenSystem_GetParam(const char *name, int32_t *value);
void GreenSystem_FormatParams(char *buffer, size_t buffer_size);
void GreenSystem_FormatStatus(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* APP_GREEN_SYSTEM_H */
