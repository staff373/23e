#include "bsp_dcmotor.h"

#define DCMOTOR_DUTY_MIN_PERCENT (-100.0f)
#define DCMOTOR_DUTY_MAX_PERCENT (100.0f)

static uint8_t DCMotor_IsNaN(float value);
static float DCMotor_ClampFloat(float value, float min_value, float max_value);
static uint8_t DCMotor_IsCcIndexValid(DL_TIMER_CC_INDEX cc_index);
static uint8_t DCMotor_IsHwBindingValid(const DCMotor_t *motor);
static uint8_t DCMotor_IsAuxConfigured(const DCMotor_t *motor);
static void DCMotor_WritePin(
    GPIO_Regs *gpio, uint32_t pin, DCMotor_PinState_t state);
static void DCMotor_ApplyDirectionBySign(DCMotor_t *motor, uint8_t is_forward);
static uint32_t DCMotor_GetPeriodTicks(const DCMotor_t *motor);
static uint32_t DCMotor_DutyPercentToCompare(
    const DCMotor_t *motor, float duty_percent);
static void DCMotor_SetDutyPercentCompare(
    DCMotor_t *motor, float duty_percent);

void BSP_DCMotor_Init(DCMotor_t *motor)
{
    if (DCMotor_IsHwBindingValid(motor) == 0U) {
        return;
    }

    if (DCMotor_IsNaN(motor->brake_duty_percent) != 0U) {
        motor->brake_duty_percent = DCMOTOR_DEFAULT_BRAKE_DUTY_PERCENT;
    }

    motor->brake_duty_percent =
        DCMotor_ClampFloat(motor->brake_duty_percent, 0.0f, 100.0f);

    if (motor->stop_use_active_brake > 1U) {
        motor->stop_use_active_brake = DCMOTOR_DEFAULT_STOP_USE_ACTIVE_BRAKE;
    }

    DCMotor_SetDutyPercentCompare(motor, 0.0f);

    DCMotor_WritePin(
        motor->dir_gpio_port, motor->dir_pin, motor->dir_forward_state);
    if (DCMotor_IsAuxConfigured(motor) != 0U) {
        DCMotor_WritePin(motor->dir_aux_gpio_port, motor->dir_aux_pin,
            motor->dir_aux_forward_state);
    }

    DL_Timer_startCounter(motor->pwm_timer);
}

void BSP_DCMotor_SetDuty(DCMotor_t *motor, float duty_cycle)
{
    float clamped_duty;
    uint8_t is_forward;

    if (DCMotor_IsHwBindingValid(motor) == 0U) {
        return;
    }

    if (DCMotor_IsNaN(duty_cycle) != 0U) {
        duty_cycle = 0.0f;
    }

    clamped_duty = DCMotor_ClampFloat(
        duty_cycle, DCMOTOR_DUTY_MIN_PERCENT, DCMOTOR_DUTY_MAX_PERCENT);

    if (clamped_duty > 0.0f) {
        is_forward = 1U;
    } else if (clamped_duty < 0.0f) {
        is_forward = 0U;
    } else {
        DCMotor_SetDutyPercentCompare(motor, 0.0f);
        return;
    }

    DCMotor_ApplyDirectionBySign(motor, is_forward);
    DCMotor_SetDutyPercentCompare(motor, clamped_duty);
}

void BSP_DCMotor_Stop(DCMotor_t *motor)
{
    if (DCMotor_IsHwBindingValid(motor) == 0U) {
        return;
    }

    if (motor->stop_use_active_brake != 0U) {
        if (DCMotor_IsNaN(motor->brake_duty_percent) != 0U) {
            motor->brake_duty_percent = DCMOTOR_DEFAULT_BRAKE_DUTY_PERCENT;
        }

        motor->brake_duty_percent =
            DCMotor_ClampFloat(motor->brake_duty_percent, 0.0f, 100.0f);

        DCMotor_WritePin(
            motor->dir_gpio_port, motor->dir_pin, motor->brake_dir_state);
        if (DCMotor_IsAuxConfigured(motor) != 0U) {
            DCMotor_WritePin(motor->dir_aux_gpio_port, motor->dir_aux_pin,
                motor->brake_aux_dir_state);
        }

        DCMotor_SetDutyPercentCompare(motor, motor->brake_duty_percent);
    } else {
        DCMotor_SetDutyPercentCompare(motor, 0.0f);
    }
}

static uint8_t DCMotor_IsNaN(float value)
{
    return (value != value) ? 1U : 0U;
}

static float DCMotor_ClampFloat(float value, float min_value, float max_value)
{
    float clamped = value;

    if (clamped < min_value) {
        clamped = min_value;
    }

    if (clamped > max_value) {
        clamped = max_value;
    }

    return clamped;
}

static uint8_t DCMotor_IsCcIndexValid(DL_TIMER_CC_INDEX cc_index)
{
    uint8_t valid = 0U;

    if ((cc_index == DL_TIMER_CC_0_INDEX) ||
        (cc_index == DL_TIMER_CC_1_INDEX) ||
        (cc_index == DL_TIMER_CC_2_INDEX) ||
        (cc_index == DL_TIMER_CC_3_INDEX)) {
        valid = 1U;
    }

    return valid;
}

static uint8_t DCMotor_IsHwBindingValid(const DCMotor_t *motor)
{
    uint8_t valid = 1U;

    if (motor == (const DCMotor_t *) 0) {
        valid = 0U;
    } else if (motor->pwm_timer == (GPTIMER_Regs *) 0) {
        valid = 0U;
    } else if (DCMotor_IsCcIndexValid(motor->pwm_cc_index) == 0U) {
        valid = 0U;
    } else if (motor->dir_gpio_port == (GPIO_Regs *) 0) {
        valid = 0U;
    } else if (motor->dir_pin == 0U) {
        valid = 0U;
    } else if (((motor->dir_aux_gpio_port == (GPIO_Regs *) 0) &&
                   (motor->dir_aux_pin != 0U)) ||
               ((motor->dir_aux_gpio_port != (GPIO_Regs *) 0) &&
                   (motor->dir_aux_pin == 0U))) {
        valid = 0U;
    } else {
        /* Binding is valid. */
    }

    return valid;
}

static uint8_t DCMotor_IsAuxConfigured(const DCMotor_t *motor)
{
    uint8_t configured = 0U;

    if (motor == (const DCMotor_t *) 0) {
        return 0U;
    }

    if ((motor->dir_aux_gpio_port != (GPIO_Regs *) 0) &&
        (motor->dir_aux_pin != 0U)) {
        configured = 1U;
    }

    return configured;
}

static void DCMotor_WritePin(
    GPIO_Regs *gpio, uint32_t pin, DCMotor_PinState_t state)
{
    if (state == DCMOTOR_PIN_LOW) {
        DL_GPIO_clearPins(gpio, pin);
    } else {
        DL_GPIO_setPins(gpio, pin);
    }
}

static void DCMotor_ApplyDirectionBySign(DCMotor_t *motor, uint8_t is_forward)
{
    DCMotor_PinState_t dir_state;
    DCMotor_PinState_t aux_dir_state;

    if (motor->reverse_flag != 0U) {
        is_forward = (is_forward == 0U) ? 1U : 0U;
    }

    if (is_forward != 0U) {
        dir_state     = motor->dir_forward_state;
        aux_dir_state = motor->dir_aux_forward_state;
    } else {
        dir_state     = motor->dir_reverse_state;
        aux_dir_state = motor->dir_aux_reverse_state;
    }

    DCMotor_WritePin(motor->dir_gpio_port, motor->dir_pin, dir_state);
    if (DCMotor_IsAuxConfigured(motor) != 0U) {
        DCMotor_WritePin(
            motor->dir_aux_gpio_port, motor->dir_aux_pin, aux_dir_state);
    }
}

static uint32_t DCMotor_GetPeriodTicks(const DCMotor_t *motor)
{
    if (motor == (const DCMotor_t *) 0) {
        return 0U;
    }

    if (motor->pwm_period_ticks != 0U) {
        return motor->pwm_period_ticks;
    }

    return DL_Timer_getLoadValue(motor->pwm_timer);
}

static uint32_t DCMotor_DutyPercentToCompare(
    const DCMotor_t *motor, float duty_percent)
{
    float abs_duty;
    uint32_t period_ticks;
    uint32_t compare_ticks;

    if (motor == (const DCMotor_t *) 0) {
        return 0U;
    }

    abs_duty = duty_percent;
    if (DCMotor_IsNaN(abs_duty) != 0U) {
        abs_duty = 0.0f;
    }

    if (abs_duty < 0.0f) {
        abs_duty = -abs_duty;
    }

    abs_duty = DCMotor_ClampFloat(abs_duty, 0.0f, 100.0f);
    period_ticks = DCMotor_GetPeriodTicks(motor);
    if (period_ticks == 0U) {
        return 0U;
    }

    compare_ticks =
        (uint32_t) ((abs_duty * (float) period_ticks / 100.0f) + 0.5f);

    if (compare_ticks > period_ticks) {
        compare_ticks = period_ticks;
    }

    if (motor->pwm_compare_inverted != 0U) {
        compare_ticks = period_ticks - compare_ticks;
    }

    return compare_ticks;
}

static void DCMotor_SetDutyPercentCompare(
    DCMotor_t *motor, float duty_percent)
{
    uint32_t compare_ticks;

    compare_ticks = DCMotor_DutyPercentToCompare(motor, duty_percent);
    DL_Timer_setCaptureCompareValue(
        motor->pwm_timer, compare_ticks, motor->pwm_cc_index);
}
