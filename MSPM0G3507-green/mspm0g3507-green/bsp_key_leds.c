/*
 * Key feedback LED BSP for MSPM0G3507.
 *
 * LED binding:
 * - K1 feedback: PB15 / IOMUX_PINCM32
 * - K2 feedback: PB16 / IOMUX_PINCM33
 * - K3 feedback: PB13 / IOMUX_PINCM30
 */
#include "bsp_key_leds.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    uint32_t iomux;
} BSP_KeyLedConfig_t;

static const BSP_KeyLedConfig_t gKeyLedConfig[BSP_KEY_LED_COUNT] = {
    {GPIO_LEDS_PORT, GPIO_LEDS_LED1_PIN, GPIO_LEDS_LED1_IOMUX},
    {GPIO_LEDS_PORT, GPIO_LEDS_LED2_PIN, GPIO_LEDS_LED2_IOMUX},
    {GPIO_LEDS_PORT, GPIO_LEDS_LED3_PIN, GPIO_LEDS_LED3_IOMUX}
};

void BSP_KeyLeds_Init(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t) BSP_KEY_LED_COUNT; i++) {
        DL_GPIO_initDigitalOutput(gKeyLedConfig[i].iomux);
        DL_GPIO_clearPins(gKeyLedConfig[i].port, gKeyLedConfig[i].pin);
        DL_GPIO_enableOutput(gKeyLedConfig[i].port, gKeyLedConfig[i].pin);
    }
}

void BSP_KeyLeds_Set(BSP_KeyLedId_t led, uint8_t on)
{
    const BSP_KeyLedConfig_t *config;

    if ((uint8_t) led >= (uint8_t) BSP_KEY_LED_COUNT) {
        return;
    }

    config = &gKeyLedConfig[(uint8_t) led];
    if (on != 0U) {
        DL_GPIO_setPins(config->port, config->pin);
    } else {
        DL_GPIO_clearPins(config->port, config->pin);
    }
}

void BSP_KeyLeds_AllOff(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t) BSP_KEY_LED_COUNT; i++) {
        BSP_KeyLeds_Set((BSP_KeyLedId_t) i, 0U);
    }
}
