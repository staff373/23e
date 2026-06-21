/*
 * bsp_dcmotor.h
 * MSPM0 DriverLib DC motor driver wrapper.
 */
#ifndef BSP_DCMOTOR_H
#define BSP_DCMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <ti/driverlib/driverlib.h>

#define DCMOTOR_DEFAULT_STOP_USE_ACTIVE_BRAKE (1U)
#define DCMOTOR_DEFAULT_BRAKE_DUTY_PERCENT    (100.0f)

typedef enum {
    DCMOTOR_PIN_LOW  = 0U,
    DCMOTOR_PIN_HIGH = 1U
} DCMotor_PinState_t;

typedef struct {
    GPTIMER_Regs *pwm_timer;
    DL_TIMER_CC_INDEX pwm_cc_index;
    uint32_t pwm_period_ticks;
    uint8_t pwm_compare_inverted;

    GPIO_Regs *dir_gpio_port;
    uint32_t dir_pin;
    DCMotor_PinState_t dir_forward_state;
    DCMotor_PinState_t dir_reverse_state;

    GPIO_Regs *dir_aux_gpio_port;
    uint32_t dir_aux_pin;
    DCMotor_PinState_t dir_aux_forward_state;
    DCMotor_PinState_t dir_aux_reverse_state;

    uint8_t reverse_flag;

    uint8_t stop_use_active_brake;
    float brake_duty_percent;
    DCMotor_PinState_t brake_dir_state;
    DCMotor_PinState_t brake_aux_dir_state;
} DCMotor_t;

void BSP_DCMotor_Init(DCMotor_t *motor);
void BSP_DCMotor_SetDuty(DCMotor_t *motor, float duty_cycle);
void BSP_DCMotor_Stop(DCMotor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DCMOTOR_H */
