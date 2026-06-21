/*
 * Passive buzzer PWM primitive.
 */
#include "bsp_buzzer.h"

#include "ti_msp_dl_config.h"

static uint8_t gBuzzerOn;

void BSP_Buzzer_Init(void)
{
    gBuzzerOn = 0U;
    BSP_Buzzer_Stop();
}

void BSP_Buzzer_Start(void)
{
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_BUZZER_C0_IOMUX, GPIO_PWM_BUZZER_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_BUZZER_C0_PORT, GPIO_PWM_BUZZER_C0_PIN);
    DL_TimerG_setTimerCount(PWM_BUZZER_INST, 0U);
    DL_TimerG_startCounter(PWM_BUZZER_INST);
    gBuzzerOn = 1U;
}

void BSP_Buzzer_Stop(void)
{
    DL_TimerG_stopCounter(PWM_BUZZER_INST);
    DL_TimerG_setTimerCount(PWM_BUZZER_INST, 0U);
    DL_GPIO_initDigitalOutput(GPIO_PWM_BUZZER_C0_IOMUX);
    DL_GPIO_setPins(GPIO_PWM_BUZZER_C0_PORT, GPIO_PWM_BUZZER_C0_PIN);
    DL_GPIO_enableOutput(GPIO_PWM_BUZZER_C0_PORT, GPIO_PWM_BUZZER_C0_PIN);
    gBuzzerOn = 0U;
}

uint8_t BSP_Buzzer_IsOn(void)
{
    return gBuzzerOn;
}
