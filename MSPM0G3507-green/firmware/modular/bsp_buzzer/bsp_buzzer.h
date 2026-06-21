/*
 * Passive buzzer PWM primitive.
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void BSP_Buzzer_Init(void);
void BSP_Buzzer_Start(void);
void BSP_Buzzer_Stop(void);
uint8_t BSP_Buzzer_IsOn(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUZZER_H */
