/*
 * Key feedback LED BSP for MSPM0G3507.
 */
#ifndef BSP_KEY_LEDS_H
#define BSP_KEY_LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    BSP_KEY_LED_K1 = 0,
    BSP_KEY_LED_K2,
    BSP_KEY_LED_K3,
    BSP_KEY_LED_COUNT
} BSP_KeyLedId_t;

void BSP_KeyLeds_Init(void);
void BSP_KeyLeds_Set(BSP_KeyLedId_t led, uint8_t on);
void BSP_KeyLeds_AllOff(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEY_LEDS_H */
