/*
 * Green contest success indicator.
 */
#include "app_green_indicator.h"

#include "../bsp_buzzer/bsp_buzzer.h"

#include "ti_msp_dl_config.h"

#include <stdio.h>

#define GREEN_INDICATOR_SUCCESS_PULSE_MS (2000U)
#define GREEN_INDICATOR_LED_MASK \
    (GPIO_LEDS_LED1_PIN | GPIO_LEDS_LED2_PIN | GPIO_LEDS_LED3_PIN)

static volatile uint32_t gGreenIndicatorMs;
static uint32_t gGreenIndicatorPulseStartMs;
static uint8_t gGreenIndicatorRequestedSuccess;
static uint8_t gGreenIndicatorPulseActive;
static uint8_t gGreenIndicatorOutputOn;

static void GreenIndicator_ApplyOutput(uint8_t on);
static uint32_t GreenIndicator_RemainingMs(void);

void GreenIndicator_Init(void)
{
    gGreenIndicatorMs = 0U;
    gGreenIndicatorPulseStartMs = 0U;
    gGreenIndicatorRequestedSuccess = 0U;
    gGreenIndicatorPulseActive = 0U;
    gGreenIndicatorOutputOn = 0U;
    BSP_Buzzer_Init();
    GreenIndicator_SetSuccess(0U);
}

void GreenIndicator_Tick1ms(void)
{
    gGreenIndicatorMs++;
}

void GreenIndicator_Poll(void)
{
    if ((gGreenIndicatorPulseActive != 0U) &&
        ((uint32_t)(gGreenIndicatorMs - gGreenIndicatorPulseStartMs) >=
            GREEN_INDICATOR_SUCCESS_PULSE_MS)) {
        gGreenIndicatorPulseActive = 0U;
        GreenIndicator_ApplyOutput(0U);
    }
}

void GreenIndicator_SetSuccess(uint8_t success)
{
    success = (success != 0U) ? 1U : 0U;

    if (success == 0U) {
        gGreenIndicatorRequestedSuccess = 0U;
        gGreenIndicatorPulseActive = 0U;
        GreenIndicator_ApplyOutput(0U);
    } else if (gGreenIndicatorRequestedSuccess == 0U) {
        gGreenIndicatorRequestedSuccess = 1U;
        gGreenIndicatorPulseStartMs = gGreenIndicatorMs;
        gGreenIndicatorPulseActive = 1U;
        GreenIndicator_ApplyOutput(1U);
    } else {
        GreenIndicator_Poll();
    }
}

uint8_t GreenIndicator_IsSuccessOn(void)
{
    return gGreenIndicatorOutputOn;
}

void GreenIndicator_FormatStatus(char *buffer, size_t buffer_size)
{
    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    GreenIndicator_Poll();
    (void) snprintf(buffer, buffer_size,
        "IND success=%u led=%u buzzer=%u rem=%lu",
        (unsigned int) gGreenIndicatorOutputOn,
        (unsigned int) gGreenIndicatorOutputOn,
        (unsigned int) BSP_Buzzer_IsOn(),
        (unsigned long) GreenIndicator_RemainingMs());
}

static void GreenIndicator_ApplyOutput(uint8_t on)
{
    on = (on != 0U) ? 1U : 0U;

    if (gGreenIndicatorOutputOn == on) {
        return;
    }

    gGreenIndicatorOutputOn = on;

    if (on != 0U) {
        DL_GPIO_setPins(GPIO_LEDS_PORT, GREEN_INDICATOR_LED_MASK);
        BSP_Buzzer_Start();
    } else {
        DL_GPIO_clearPins(GPIO_LEDS_PORT, GREEN_INDICATOR_LED_MASK);
        BSP_Buzzer_Stop();
    }
}

static uint32_t GreenIndicator_RemainingMs(void)
{
    uint32_t elapsed;

    if (gGreenIndicatorPulseActive == 0U) {
        return 0U;
    }

    elapsed = (uint32_t)(gGreenIndicatorMs - gGreenIndicatorPulseStartMs);
    if (elapsed >= GREEN_INDICATOR_SUCCESS_PULSE_MS) {
        return 0U;
    }

    return GREEN_INDICATOR_SUCCESS_PULSE_MS - elapsed;
}
