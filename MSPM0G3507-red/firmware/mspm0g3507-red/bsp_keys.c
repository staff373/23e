/*
 * Board key GPIO BSP for MSPM0G3507.
 *
 * Key binding:
 * - K1: PA12 / IOMUX_PINCM34, active low
 * - K2: PB23 / IOMUX_PINCM51, active low
 * - K3: PB27 / IOMUX_PINCM58, active low
 */
#include "bsp_keys.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    uint32_t iomux;
} BSP_KeyConfig_t;

static const BSP_KeyConfig_t gBspKeyConfig[BSP_KEY_COUNT] = {
    {GPIO_KEYS_PA12_PORT, GPIO_KEYS_PA12_PIN, GPIO_KEYS_PA12_IOMUX},
    {GPIO_KEYS_PB23_PORT, GPIO_KEYS_PB23_PIN, GPIO_KEYS_PB23_IOMUX},
    {GPIO_KEYS_PB27_PORT, GPIO_KEYS_PB27_PIN, GPIO_KEYS_PB27_IOMUX}
};

void BSP_Keys_Init(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t) BSP_KEY_COUNT; i++) {
        DL_GPIO_initDigitalInputFeatures(gBspKeyConfig[i].iomux,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
            DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    }
}

uint8_t BSP_Keys_IsPressed(BSP_KeyId_t key)
{
    const BSP_KeyConfig_t *config;

    if ((uint8_t) key >= (uint8_t) BSP_KEY_COUNT) {
        return 0U;
    }

    config = &gBspKeyConfig[(uint8_t) key];
    return ((DL_GPIO_readPins(config->port, config->pin) & config->pin) == 0U)
        ? 1U
        : 0U;
}

void BSP_Keys_GetRaw(BSP_KeysRaw_t *raw)
{
    uint8_t i;

    if (raw == NULL) {
        return;
    }

    for (i = 0U; i < (uint8_t) BSP_KEY_COUNT; i++) {
        raw->pressed[i] = BSP_Keys_IsPressed((BSP_KeyId_t) i);
    }
}
