/*
 * App-level key event mapper.
 */
#include "app_keys.h"

#include "bsp_key_leds.h"
#include "bsp_keys.h"

#include <stddef.h>
#include <string.h>

#define APP_KEYS_DEBOUNCE_MS (30U)
#define APP_KEYS_K1_LONG_MS (800U)

#define APP_KEYS_MASK_K1 ((uint8_t)(1U << (uint8_t) BSP_KEY_K1))
#define APP_KEYS_MASK_K2 ((uint8_t)(1U << (uint8_t) BSP_KEY_K2))
#define APP_KEYS_MASK_K3 ((uint8_t)(1U << (uint8_t) BSP_KEY_K3))

static volatile uint32_t gAppKeysMs;
static uint8_t gAppKeysRawMask;
static uint8_t gAppKeysStableMask;
static uint8_t gAppKeysAcceptedMask;
static uint8_t gAppKeysGroupLocked;
static uint8_t gAppKeysK1LongReported;
static uint32_t gAppKeysRawChangedMs;
static uint32_t gAppKeysPressedStartMs;
static uint32_t gAppKeyPendingEvents;
static AppKeysStatus_t gAppKeysStatus;

static uint32_t AppKeys_NowMs(void);
static uint8_t AppKeys_ReadPressedMask(void);
static void AppKeys_SetEvent(AppKeyEvent_t event);
static void AppKeys_ServiceGroup(void);
static void AppKeys_HandleStableChange(uint8_t oldMask, uint8_t newMask);
static void AppKeys_HandleGroupPress(uint8_t pressedMask);
static void AppKeys_HandleGroupRelease(void);

void AppKeys_Init(void)
{
    memset(&gAppKeysStatus, 0, sizeof(gAppKeysStatus));
    gAppKeysMs = 0U;
    gAppKeyPendingEvents = 0U;
    gAppKeysAcceptedMask = 0U;
    gAppKeysK1LongReported = 0U;
    gAppKeysPressedStartMs = 0U;

    BSP_Keys_Init();
    BSP_KeyLeds_Init();
    BSP_KeyLeds_AllOff();

    gAppKeysRawMask = AppKeys_ReadPressedMask();
    gAppKeysStableMask = gAppKeysRawMask;
    gAppKeysRawChangedMs = AppKeys_NowMs();
    gAppKeysGroupLocked = (gAppKeysStableMask != 0U) ? 1U : 0U;
}

void AppKeys_Poll(void)
{
    AppKeys_ServiceGroup();

    AppKeys_GetStatus(&gAppKeysStatus);
}

void AppKeys_Tick1ms(void)
{
    gAppKeysMs++;
}

bool AppKeys_PopEvent(AppKeyEvent_t *event)
{
    if (event == NULL) {
        return false;
    }

    if ((gAppKeyPendingEvents & (uint32_t) APP_KEY_EVENT_K1_SHORT) != 0U) {
        gAppKeyPendingEvents &= ~((uint32_t) APP_KEY_EVENT_K1_SHORT);
        *event = APP_KEY_EVENT_K1_SHORT;
        return true;
    }
    if ((gAppKeyPendingEvents & (uint32_t) APP_KEY_EVENT_K1_LONG) != 0U) {
        gAppKeyPendingEvents &= ~((uint32_t) APP_KEY_EVENT_K1_LONG);
        *event = APP_KEY_EVENT_K1_LONG;
        return true;
    }
    if ((gAppKeyPendingEvents & (uint32_t) APP_KEY_EVENT_K2_SHORT) != 0U) {
        gAppKeyPendingEvents &= ~((uint32_t) APP_KEY_EVENT_K2_SHORT);
        *event = APP_KEY_EVENT_K2_SHORT;
        return true;
    }
    if ((gAppKeyPendingEvents & (uint32_t) APP_KEY_EVENT_K3_SHORT) != 0U) {
        gAppKeyPendingEvents &= ~((uint32_t) APP_KEY_EVENT_K3_SHORT);
        *event = APP_KEY_EVENT_K3_SHORT;
        return true;
    }

    *event = APP_KEY_EVENT_NONE;
    return false;
}

uint32_t AppKeys_PeekEvents(void)
{
    return gAppKeyPendingEvents;
}

void AppKeys_GetStatus(AppKeysStatus_t *status)
{
    if (status == NULL) {
        return;
    }

    status->ms = AppKeys_NowMs();
    status->pending_events = gAppKeyPendingEvents;
    status->k1_pressed =
        ((gAppKeysStableMask & APP_KEYS_MASK_K1) != 0U) ? 1U : 0U;
    status->k2_pressed =
        ((gAppKeysStableMask & APP_KEYS_MASK_K2) != 0U) ? 1U : 0U;
    status->k3_pressed =
        ((gAppKeysStableMask & APP_KEYS_MASK_K3) != 0U) ? 1U : 0U;
    status->k1_long_active = gAppKeysK1LongReported;
    status->k1_short_count = gAppKeysStatus.k1_short_count;
    status->k1_long_count = gAppKeysStatus.k1_long_count;
    status->k2_short_count = gAppKeysStatus.k2_short_count;
    status->k3_short_count = gAppKeysStatus.k3_short_count;
}

const char *AppKeys_GetEventName(AppKeyEvent_t event)
{
    switch (event) {
        case APP_KEY_EVENT_K1_SHORT:
            return "K1_SHORT";
        case APP_KEY_EVENT_K1_LONG:
            return "K1_LONG";
        case APP_KEY_EVENT_K2_SHORT:
            return "K2_SHORT";
        case APP_KEY_EVENT_K3_SHORT:
            return "K3_SHORT";
        default:
            return "NONE";
    }
}

static uint32_t AppKeys_NowMs(void)
{
    return gAppKeysMs;
}

static uint8_t AppKeys_ReadPressedMask(void)
{
    BSP_KeysRaw_t raw;
    uint8_t pressedMask = 0U;

    BSP_Keys_GetRaw(&raw);
    if (raw.pressed[(uint8_t) BSP_KEY_K1] != 0U) {
        pressedMask |= APP_KEYS_MASK_K1;
    }
    if (raw.pressed[(uint8_t) BSP_KEY_K2] != 0U) {
        pressedMask |= APP_KEYS_MASK_K2;
    }
    if (raw.pressed[(uint8_t) BSP_KEY_K3] != 0U) {
        pressedMask |= APP_KEYS_MASK_K3;
    }

    return pressedMask;
}

static void AppKeys_SetEvent(AppKeyEvent_t event)
{
    gAppKeyPendingEvents |= (uint32_t) event;

    switch (event) {
        case APP_KEY_EVENT_K1_SHORT:
            gAppKeysStatus.k1_short_count++;
            break;
        case APP_KEY_EVENT_K1_LONG:
            gAppKeysStatus.k1_long_count++;
            break;
        case APP_KEY_EVENT_K2_SHORT:
            gAppKeysStatus.k2_short_count++;
            break;
        case APP_KEY_EVENT_K3_SHORT:
            gAppKeysStatus.k3_short_count++;
            break;
        default:
            break;
    }
}

static void AppKeys_ServiceGroup(void)
{
    uint8_t rawMask;
    uint8_t oldStableMask;
    uint32_t now;

    rawMask = AppKeys_ReadPressedMask();
    now = AppKeys_NowMs();

    if (rawMask != gAppKeysRawMask) {
        gAppKeysRawMask = rawMask;
        gAppKeysRawChangedMs = now;
    }

    if ((gAppKeysRawMask != gAppKeysStableMask) &&
        ((uint32_t)(now - gAppKeysRawChangedMs) >= APP_KEYS_DEBOUNCE_MS)) {
        oldStableMask = gAppKeysStableMask;
        gAppKeysStableMask = gAppKeysRawMask;
        AppKeys_HandleStableChange(oldStableMask, gAppKeysStableMask);
    }

    if ((gAppKeysGroupLocked != 0U) &&
        ((gAppKeysAcceptedMask & APP_KEYS_MASK_K1) != 0U) &&
        ((gAppKeysStableMask & APP_KEYS_MASK_K1) != 0U) &&
        (gAppKeysK1LongReported == 0U) &&
        ((uint32_t)(now - gAppKeysPressedStartMs) >= APP_KEYS_K1_LONG_MS)) {
        gAppKeysK1LongReported = 1U;
        AppKeys_SetEvent(APP_KEY_EVENT_K1_LONG);
    }
}

static void AppKeys_HandleStableChange(uint8_t oldMask, uint8_t newMask)
{
    if ((oldMask == 0U) && (newMask != 0U)) {
        AppKeys_HandleGroupPress(newMask);
    } else if ((oldMask != 0U) && (newMask == 0U)) {
        AppKeys_HandleGroupRelease();
    } else {
        /* While any key is held, ignore other key changes until full release. */
    }
}

static void AppKeys_HandleGroupPress(uint8_t pressedMask)
{
    gAppKeysGroupLocked = 1U;
    gAppKeysAcceptedMask = pressedMask;
    gAppKeysPressedStartMs = AppKeys_NowMs();
    gAppKeysK1LongReported = 0U;

    if ((pressedMask & APP_KEYS_MASK_K2) != 0U) {
        AppKeys_SetEvent(APP_KEY_EVENT_K2_SHORT);
    }
    if ((pressedMask & APP_KEYS_MASK_K3) != 0U) {
        AppKeys_SetEvent(APP_KEY_EVENT_K3_SHORT);
    }
}

static void AppKeys_HandleGroupRelease(void)
{
    if ((gAppKeysGroupLocked != 0U) &&
        ((gAppKeysAcceptedMask & APP_KEYS_MASK_K1) != 0U) &&
        (gAppKeysK1LongReported == 0U)) {
        AppKeys_SetEvent(APP_KEY_EVENT_K1_SHORT);
    }

    gAppKeysGroupLocked = 0U;
    gAppKeysAcceptedMask = 0U;
    gAppKeysK1LongReported = 0U;
    gAppKeysPressedStartMs = 0U;
}
