/*
 * App-level key event mapper.
 */
#ifndef APP_KEYS_H
#define APP_KEYS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_KEY_EVENT_NONE = 0,
    APP_KEY_EVENT_K1_SHORT = (1U << 0),
    APP_KEY_EVENT_K1_LONG = (1U << 1),
    APP_KEY_EVENT_K2_START_SAVE = (1U << 2),
    APP_KEY_EVENT_K3_MODE_SELECT = (1U << 3)
} AppKeyEvent_t;

typedef struct {
    uint32_t ms;
    uint32_t pending_events;
    uint8_t k1_pressed;
    uint8_t k2_pressed;
    uint8_t k3_pressed;
    uint8_t k1_long_active;
    uint32_t k1_short_count;
    uint32_t k1_long_count;
    uint32_t k2_start_save_count;
    uint32_t k3_mode_select_count;
} AppKeysStatus_t;

void AppKeys_Init(void);
void AppKeys_Poll(void);
void AppKeys_Tick1ms(void);

bool AppKeys_PopEvent(AppKeyEvent_t *event);
uint32_t AppKeys_PeekEvents(void);
void AppKeys_GetStatus(AppKeysStatus_t *status);
const char *AppKeys_GetEventName(AppKeyEvent_t event);

#ifdef __cplusplus
}
#endif

#endif /* APP_KEYS_H */
