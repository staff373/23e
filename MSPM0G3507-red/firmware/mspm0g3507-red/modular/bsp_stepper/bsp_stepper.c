#include "bsp_stepper.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define STEPPER_X_TIMER (TIMG7)
#define STEPPER_X_IRQN (TIMG7_INT_IRQn)
#define STEPPER_X_STEP_PORT (GPIOA)
#define STEPPER_X_STEP_PIN (DL_GPIO_PIN_27)
#define STEPPER_X_STEP_IOMUX (IOMUX_PINCM60)
#define STEPPER_X_STEP_IOMUX_FUNC (IOMUX_PINCM60_PF_TIMG7_CCP1)
#define STEPPER_X_CC_INDEX (DL_TIMER_CC_1_INDEX)
#define STEPPER_X_CCP_OUTPUT (DL_TIMER_CC1_OUTPUT)
#define STEPPER_X_DIR_PORT (GPIOA)
#define STEPPER_X_DIR_PIN (DL_GPIO_PIN_21)
#define STEPPER_X_DIR_IOMUX (IOMUX_PINCM46)
#define STEPPER_X_EN_PORT (GPIOA)
#define STEPPER_X_EN_PIN (DL_GPIO_PIN_22)
#define STEPPER_X_EN_IOMUX (IOMUX_PINCM47)
/* Current stepper drivers use active-low ENA: high releases, low enables. */
#ifndef STEPPER_X_EN_ACTIVE_HIGH
#define STEPPER_X_EN_ACTIVE_HIGH (0U)
#endif

#define STEPPER_Y_TIMER (TIMG12)
#define STEPPER_Y_IRQN (TIMG12_INT_IRQn)
#define STEPPER_Y_STEP_PORT (GPIOA)
#define STEPPER_Y_STEP_PIN (DL_GPIO_PIN_14)
#define STEPPER_Y_STEP_IOMUX (IOMUX_PINCM36)
#define STEPPER_Y_STEP_IOMUX_FUNC (IOMUX_PINCM36_PF_TIMG12_CCP0)
#define STEPPER_Y_CC_INDEX (DL_TIMER_CC_0_INDEX)
#define STEPPER_Y_CCP_OUTPUT (DL_TIMER_CC0_OUTPUT)
#define STEPPER_Y_DIR_PORT (GPIOA)
#define STEPPER_Y_DIR_PIN (DL_GPIO_PIN_23)
#define STEPPER_Y_DIR_IOMUX (IOMUX_PINCM53)
#define STEPPER_Y_EN_PORT (GPIOA)
#define STEPPER_Y_EN_PIN (DL_GPIO_PIN_18)
#define STEPPER_Y_EN_IOMUX (IOMUX_PINCM40)
/* Current stepper drivers use active-low ENA: high releases, low enables. */
#ifndef STEPPER_Y_EN_ACTIVE_HIGH
#define STEPPER_Y_EN_ACTIVE_HIGH (0U)
#endif

#define STEPPER_TIMER_CLOCK_DIVIDE (8U)
#define STEPPER_TIMER_PRESCALE_VALUE (9U)
#define STEPPER_TIMER_PRESCALE_DIVIDE (STEPPER_TIMER_PRESCALE_VALUE + 1U)
#define STEPPER_TIMER_TICK_HZ \
    (CPUCLK_FREQ / (STEPPER_TIMER_CLOCK_DIVIDE * STEPPER_TIMER_PRESCALE_DIVIDE))
#define STEPPER_TIMER_MIN_LOAD (2U)
#define STEPPER_TIMER_MAX_LOAD (65535U)
#define STEPPER_TIMER_DEFAULT_PERIOD_TICKS \
    (STEPPER_TIMER_TICK_HZ / (uint32_t) STEPPER_X_MIN_SPEED_SPS)

typedef struct {
    GPTIMER_Regs *timer;
    IRQn_Type irqn;
    IOMUX_PINCM step_iomux;
    uint32_t step_iomux_func;
    GPIO_Regs *step_port;
    uint32_t step_pin;
    DL_TIMER_CC_INDEX cc_index;
    uint32_t ccp_output;

    IOMUX_PINCM dir_iomux;
    GPIO_Regs *dir_port;
    uint32_t dir_pin;
    uint8_t dir_positive_high;
    uint8_t dir_negative_high;

    IOMUX_PINCM en_iomux;
    GPIO_Regs *en_port;
    uint32_t en_pin;
    uint8_t en_active_high;
    uint8_t configured;

    float min_speed_sps;
    float max_speed_limit_sps;
    float default_accel_sps2;

    volatile BSP_StepperState_t state;
    volatile uint8_t busy;
    volatile uint8_t hold_enabled;
    volatile uint8_t stop_requested;
    volatile uint8_t hw_ready;
    volatile int32_t position_steps;
    volatile int32_t direction_sign;
    volatile uint32_t total_steps;
    volatile uint32_t remaining_steps;

    float current_speed_sps;
    float max_speed_sps;
    float accel_sps2;
} Stepper_AxisRuntime_t;

static Stepper_AxisRuntime_t gStepperAxis[STEPPER_AXIS_COUNT];
static uint8_t gStepperInitialized;

static uint8_t Stepper_IsAxisValid(uint8_t axis);
static uint32_t Stepper_AbsSteps(int32_t steps);
static float Stepper_AbsFloat(float value);
static float Stepper_ClampFloat(float value, float min_value, float max_value);
static void Stepper_LoadAxisConfig(uint8_t axis);
static uint8_t Stepper_IsHwBindingValid(const Stepper_AxisRuntime_t *axis_obj);
static void Stepper_InitAxisHardware(uint8_t axis);
static void Stepper_SetCounterControlForChannel(
    const Stepper_AxisRuntime_t *obj);
static void Stepper_ApplyEnable(uint8_t axis, uint8_t enable);
static void Stepper_ApplyDirection(uint8_t axis);
static uint8_t Stepper_ConfigPulseTimer(uint8_t axis, float pulse_hz);
static uint8_t Stepper_StartPulse(uint8_t axis);
static void Stepper_StopPulse(uint8_t axis, BSP_StepperState_t final_state);
static void Stepper_OnPeriodElapsed(uint8_t axis);
static void Stepper_UpdateSpeedForNextStep(uint8_t axis);

static const DL_TimerG_ClockConfig gStepperTimerClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = STEPPER_TIMER_PRESCALE_VALUE,
};

static const DL_TimerG_PWMConfig gStepperPwmConfig = {
    .period = STEPPER_TIMER_DEFAULT_PERIOD_TICKS,
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

void BSP_Stepper_Init(void)
{
    uint8_t axis;

    if (gStepperInitialized != 0U) {
        return;
    }

    for (axis = 0U; axis < STEPPER_AXIS_COUNT; axis++) {
        Stepper_LoadAxisConfig(axis);
        gStepperAxis[axis].hw_ready =
            Stepper_IsHwBindingValid(&gStepperAxis[axis]);
        gStepperAxis[axis].state = (gStepperAxis[axis].hw_ready != 0U) ?
            STEPPER_STATE_IDLE :
            STEPPER_STATE_DISABLED;
        gStepperAxis[axis].busy = 0U;
        gStepperAxis[axis].hold_enabled = 0U;
        gStepperAxis[axis].stop_requested = 0U;
        gStepperAxis[axis].position_steps = 0;
        gStepperAxis[axis].direction_sign = 1;
        gStepperAxis[axis].total_steps = 0U;
        gStepperAxis[axis].remaining_steps = 0U;
        gStepperAxis[axis].current_speed_sps = 0.0f;
        gStepperAxis[axis].max_speed_sps = gStepperAxis[axis].min_speed_sps;
        gStepperAxis[axis].accel_sps2 =
            gStepperAxis[axis].default_accel_sps2;

        if (gStepperAxis[axis].hw_ready != 0U) {
            Stepper_InitAxisHardware(axis);
            Stepper_ApplyDirection(axis);
            Stepper_ApplyEnable(axis, 0U);
        }
    }

    gStepperInitialized = 1U;
}

uint8_t BSP_Stepper_MoveSteps(
    uint8_t axis, int32_t steps, float max_speed_sps, float accel_sps2)
{
    uint32_t step_count;
    float speed;
    float accel;

    if (gStepperInitialized == 0U) {
        BSP_Stepper_Init();
    }

    if (Stepper_IsAxisValid(axis) == 0U) {
        return 0U;
    }

    if ((gStepperAxis[axis].hw_ready == 0U) ||
        (gStepperAxis[axis].busy != 0U)) {
        return 0U;
    }

    step_count = Stepper_AbsSteps(steps);
    if (step_count == 0U) {
        gStepperAxis[axis].state = STEPPER_STATE_DONE;
        return 1U;
    }

    speed = max_speed_sps;
    if (speed <= STEPPER_SPEED_EPSILON_SPS) {
        speed = gStepperAxis[axis].max_speed_limit_sps;
    }
    speed = Stepper_ClampFloat(speed, gStepperAxis[axis].min_speed_sps,
        gStepperAxis[axis].max_speed_limit_sps);

    accel = accel_sps2;
    if (accel <= STEPPER_SPEED_EPSILON_SPS) {
        accel = gStepperAxis[axis].default_accel_sps2;
    }

    gStepperAxis[axis].direction_sign = (steps >= 0) ? 1 : -1;
    gStepperAxis[axis].total_steps = step_count;
    gStepperAxis[axis].remaining_steps = step_count;
    gStepperAxis[axis].stop_requested = 0U;
    gStepperAxis[axis].max_speed_sps = speed;
    gStepperAxis[axis].accel_sps2 = accel;
    gStepperAxis[axis].current_speed_sps = gStepperAxis[axis].min_speed_sps;

    Stepper_ApplyDirection(axis);

    if (Stepper_StartPulse(axis) == 0U) {
        gStepperAxis[axis].remaining_steps = 0U;
        gStepperAxis[axis].total_steps = 0U;
        gStepperAxis[axis].current_speed_sps = 0.0f;
        gStepperAxis[axis].state = STEPPER_STATE_ERROR;
        return 0U;
    }

    return 1U;
}

void BSP_Stepper_Stop(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return;
    }

    if ((gStepperAxis[axis].hw_ready == 0U) ||
        (gStepperAxis[axis].busy == 0U)) {
        return;
    }

    gStepperAxis[axis].stop_requested = 1U;
    gStepperAxis[axis].state = STEPPER_STATE_STOPPING;
}

void BSP_Stepper_EmergencyStop(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return;
    }

    if (gStepperAxis[axis].hw_ready == 0U) {
        return;
    }

    gStepperAxis[axis].remaining_steps = 0U;
    gStepperAxis[axis].total_steps = 0U;
    gStepperAxis[axis].current_speed_sps = 0.0f;
    gStepperAxis[axis].stop_requested = 0U;
    Stepper_StopPulse(axis, STEPPER_STATE_IDLE);
}

void BSP_Stepper_SetHoldEnabled(uint8_t axis, uint8_t enabled)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return;
    }

    if (gStepperAxis[axis].hw_ready == 0U) {
        return;
    }

    gStepperAxis[axis].hold_enabled = (enabled != 0U) ? 1U : 0U;
    if (gStepperAxis[axis].busy == 0U) {
        Stepper_ApplyEnable(axis, gStepperAxis[axis].hold_enabled);
    }
}

uint8_t BSP_Stepper_GetHoldEnabled(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return 0U;
    }

    return gStepperAxis[axis].hold_enabled;
}

uint8_t BSP_Stepper_IsBusy(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return 0U;
    }

    return gStepperAxis[axis].busy;
}

uint8_t BSP_Stepper_IsReady(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return 0U;
    }

    return gStepperAxis[axis].hw_ready;
}

int32_t BSP_Stepper_GetPosition(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return 0;
    }

    return gStepperAxis[axis].position_steps;
}

void BSP_Stepper_SetPosition(uint8_t axis, int32_t position_steps)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return;
    }

    if (gStepperAxis[axis].busy != 0U) {
        return;
    }

    gStepperAxis[axis].position_steps = position_steps;
}

int32_t BSP_Stepper_GetRemaining(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return 0;
    }

    if (gStepperAxis[axis].remaining_steps > 2147483647UL) {
        return 2147483647L;
    }

    return (int32_t) gStepperAxis[axis].remaining_steps;
}

BSP_StepperState_t BSP_Stepper_GetState(uint8_t axis)
{
    if (Stepper_IsAxisValid(axis) == 0U) {
        return STEPPER_STATE_DISABLED;
    }

    return gStepperAxis[axis].state;
}

void BSP_Stepper_GetStatus(uint8_t axis, BSP_StepperStatus_t *status)
{
    if (status == (BSP_StepperStatus_t *) 0) {
        return;
    }

    if (Stepper_IsAxisValid(axis) == 0U) {
        status->hw_ready = 0U;
        status->busy = 0U;
        status->hold_enabled = 0U;
        status->state = STEPPER_STATE_DISABLED;
        status->position_steps = 0;
        status->direction_sign = 0;
        status->total_steps = 0U;
        status->remaining_steps = 0U;
        status->current_speed_sps = 0.0f;
        status->max_speed_sps = 0.0f;
        status->accel_sps2 = 0.0f;
        return;
    }

    status->hw_ready = gStepperAxis[axis].hw_ready;
    status->busy = gStepperAxis[axis].busy;
    status->hold_enabled = gStepperAxis[axis].hold_enabled;
    status->state = gStepperAxis[axis].state;
    status->position_steps = gStepperAxis[axis].position_steps;
    status->direction_sign = gStepperAxis[axis].direction_sign;
    status->total_steps = gStepperAxis[axis].total_steps;
    status->remaining_steps = gStepperAxis[axis].remaining_steps;
    status->current_speed_sps = gStepperAxis[axis].current_speed_sps;
    status->max_speed_sps = gStepperAxis[axis].max_speed_sps;
    status->accel_sps2 = gStepperAxis[axis].accel_sps2;
}

const char *BSP_Stepper_GetStateName(BSP_StepperState_t state)
{
    switch (state) {
    case STEPPER_STATE_DISABLED:
        return "DISABLED";
    case STEPPER_STATE_IDLE:
        return "IDLE";
    case STEPPER_STATE_MOVING:
        return "MOVING";
    case STEPPER_STATE_STOPPING:
        return "STOPPING";
    case STEPPER_STATE_DONE:
        return "DONE";
    case STEPPER_STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

void TIMG12_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(STEPPER_Y_TIMER)) {
    case DL_TIMER_IIDX_ZERO:
        Stepper_OnPeriodElapsed(STEPPER_AXIS_Y);
        break;
    default:
        break;
    }
}

void TIMG7_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(STEPPER_X_TIMER)) {
    case DL_TIMER_IIDX_ZERO:
        Stepper_OnPeriodElapsed(STEPPER_AXIS_X);
        break;
    default:
        break;
    }
}

static uint8_t Stepper_IsAxisValid(uint8_t axis)
{
    return (uint8_t) (axis < STEPPER_AXIS_COUNT);
}

static uint32_t Stepper_AbsSteps(int32_t steps)
{
    uint32_t value;

    if (steps >= 0) {
        value = (uint32_t) steps;
    } else {
        value = (uint32_t) (-(steps + 1)) + 1U;
    }

    return value;
}

static float Stepper_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Stepper_ClampFloat(float value, float min_value, float max_value)
{
    float result = value;

    if (result < min_value) {
        result = min_value;
    }
    if (result > max_value) {
        result = max_value;
    }

    return result;
}

static void Stepper_LoadAxisConfig(uint8_t axis)
{
    Stepper_AxisRuntime_t *obj = &gStepperAxis[axis];

    obj->timer = (GPTIMER_Regs *) 0;
    obj->irqn = NonMaskableInt_IRQn;
    obj->step_iomux = (IOMUX_PINCM) 0;
    obj->step_iomux_func = 0U;
    obj->step_port = (GPIO_Regs *) 0;
    obj->step_pin = 0U;
    obj->cc_index = DL_TIMER_CC_0_INDEX;
    obj->ccp_output = 0U;
    obj->dir_iomux = (IOMUX_PINCM) 0;
    obj->dir_port = (GPIO_Regs *) 0;
    obj->dir_pin = 0U;
    obj->dir_positive_high = 0U;
    obj->dir_negative_high = 1U;
    obj->en_iomux = (IOMUX_PINCM) 0;
    obj->en_port = (GPIO_Regs *) 0;
    obj->en_pin = 0U;
    obj->en_active_high = 0U;
    obj->configured = 0U;
    obj->min_speed_sps = STEPPER_X_MIN_SPEED_SPS;
    obj->max_speed_limit_sps = STEPPER_X_MAX_SPEED_SPS;
    obj->default_accel_sps2 = STEPPER_X_ACCEL_SPS2;

    if (axis == STEPPER_AXIS_X) {
        obj->timer = STEPPER_X_TIMER;
        obj->irqn = STEPPER_X_IRQN;
        obj->step_iomux = STEPPER_X_STEP_IOMUX;
        obj->step_iomux_func = STEPPER_X_STEP_IOMUX_FUNC;
        obj->step_port = STEPPER_X_STEP_PORT;
        obj->step_pin = STEPPER_X_STEP_PIN;
        obj->cc_index = STEPPER_X_CC_INDEX;
        obj->ccp_output = STEPPER_X_CCP_OUTPUT;
        obj->dir_iomux = STEPPER_X_DIR_IOMUX;
        obj->dir_port = STEPPER_X_DIR_PORT;
        obj->dir_pin = STEPPER_X_DIR_PIN;
        obj->en_iomux = STEPPER_X_EN_IOMUX;
        obj->en_port = STEPPER_X_EN_PORT;
        obj->en_pin = STEPPER_X_EN_PIN;
        obj->en_active_high = (STEPPER_X_EN_ACTIVE_HIGH != 0U) ? 1U : 0U;
        obj->configured = 1U;
    } else if (axis == STEPPER_AXIS_Y) {
        obj->timer = STEPPER_Y_TIMER;
        obj->irqn = STEPPER_Y_IRQN;
        obj->step_iomux = STEPPER_Y_STEP_IOMUX;
        obj->step_iomux_func = STEPPER_Y_STEP_IOMUX_FUNC;
        obj->step_port = STEPPER_Y_STEP_PORT;
        obj->step_pin = STEPPER_Y_STEP_PIN;
        obj->cc_index = STEPPER_Y_CC_INDEX;
        obj->ccp_output = STEPPER_Y_CCP_OUTPUT;
        obj->dir_iomux = STEPPER_Y_DIR_IOMUX;
        obj->dir_port = STEPPER_Y_DIR_PORT;
        obj->dir_pin = STEPPER_Y_DIR_PIN;
        obj->en_iomux = STEPPER_Y_EN_IOMUX;
        obj->en_port = STEPPER_Y_EN_PORT;
        obj->en_pin = STEPPER_Y_EN_PIN;
        obj->en_active_high = (STEPPER_Y_EN_ACTIVE_HIGH != 0U) ? 1U : 0U;
        obj->min_speed_sps = STEPPER_Y_MIN_SPEED_SPS;
        obj->max_speed_limit_sps = STEPPER_Y_MAX_SPEED_SPS;
        obj->default_accel_sps2 = STEPPER_Y_ACCEL_SPS2;
        obj->configured = 1U;
    }
}

static uint8_t Stepper_IsHwBindingValid(const Stepper_AxisRuntime_t *axis_obj)
{
    if (axis_obj == (const Stepper_AxisRuntime_t *) 0) {
        return 0U;
    }

    if ((axis_obj->configured == 0U) ||
        (axis_obj->timer == (GPTIMER_Regs *) 0) ||
        (axis_obj->step_port == (GPIO_Regs *) 0) ||
        (axis_obj->step_pin == 0U) ||
        (axis_obj->ccp_output == 0U) ||
        (axis_obj->dir_port == (GPIO_Regs *) 0) ||
        (axis_obj->dir_pin == 0U) ||
        (axis_obj->en_port == (GPIO_Regs *) 0) ||
        (axis_obj->en_pin == 0U) ||
        (axis_obj->min_speed_sps <= 0.0f) ||
        (axis_obj->max_speed_limit_sps < axis_obj->min_speed_sps) ||
        (axis_obj->default_accel_sps2 <= 0.0f)) {
        return 0U;
    }

    return 1U;
}

static void Stepper_InitAxisHardware(uint8_t axis)
{
    Stepper_AxisRuntime_t *obj = &gStepperAxis[axis];

    DL_TimerG_reset(obj->timer);
    DL_TimerG_enablePower(obj->timer);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(obj->step_iomux, obj->step_iomux_func);
    DL_GPIO_enableOutput(obj->step_port, obj->step_pin);

    DL_GPIO_initDigitalOutput(obj->dir_iomux);
    DL_GPIO_initDigitalOutput(obj->en_iomux);
    Stepper_ApplyDirection(axis);
    Stepper_ApplyEnable(axis, 0U);
    DL_GPIO_enableOutput(obj->dir_port, obj->dir_pin);
    DL_GPIO_enableOutput(obj->en_port, obj->en_pin);

    DL_TimerG_setClockConfig(obj->timer, &gStepperTimerClockConfig);
    DL_TimerG_initPWMMode(obj->timer, &gStepperPwmConfig);
    Stepper_SetCounterControlForChannel(obj);
    DL_TimerG_setCaptureCompareOutCtl(obj->timer,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, obj->cc_index);
    DL_TimerG_setCaptCompUpdateMethod(
        obj->timer, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, obj->cc_index);
    DL_TimerG_setCaptureCompareValue(obj->timer, 0U, obj->cc_index);
    DL_TimerG_enableClock(obj->timer);
    DL_TimerG_setCCPDirection(obj->timer, obj->ccp_output);
    DL_TimerG_disableInterrupt(obj->timer, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_clearInterruptStatus(obj->timer, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(obj->irqn);
}

static void Stepper_SetCounterControlForChannel(
    const Stepper_AxisRuntime_t *obj)
{
    if (obj->cc_index == DL_TIMER_CC_1_INDEX) {
        DL_TimerG_setCounterControl(obj->timer, DL_TIMER_CZC_CCCTL1_ZCOND,
            DL_TIMER_CAC_CCCTL1_ACOND, DL_TIMER_CLC_CCCTL1_LCOND);
    } else {
        DL_TimerG_setCounterControl(obj->timer, DL_TIMER_CZC_CCCTL0_ZCOND,
            DL_TIMER_CAC_CCCTL0_ACOND, DL_TIMER_CLC_CCCTL0_LCOND);
    }
}

static void Stepper_ApplyEnable(uint8_t axis, uint8_t enable)
{
    Stepper_AxisRuntime_t *obj = &gStepperAxis[axis];
    uint8_t pin_high = (enable != 0U) ? obj->en_active_high :
                                          (uint8_t) !obj->en_active_high;

    if (pin_high != 0U) {
        DL_GPIO_setPins(obj->en_port, obj->en_pin);
    } else {
        DL_GPIO_clearPins(obj->en_port, obj->en_pin);
    }
}

static void Stepper_ApplyDirection(uint8_t axis)
{
    Stepper_AxisRuntime_t *obj = &gStepperAxis[axis];
    uint8_t pin_high = (obj->direction_sign >= 0) ? obj->dir_positive_high :
                                                    obj->dir_negative_high;

    if (pin_high != 0U) {
        DL_GPIO_setPins(obj->dir_port, obj->dir_pin);
    } else {
        DL_GPIO_clearPins(obj->dir_port, obj->dir_pin);
    }
}

static uint8_t Stepper_ConfigPulseTimer(uint8_t axis, float pulse_hz)
{
    Stepper_AxisRuntime_t *obj = &gStepperAxis[axis];
    uint32_t period_ticks;
    uint32_t compare_ticks;

    if (pulse_hz <= STEPPER_SPEED_EPSILON_SPS) {
        return 0U;
    }

    period_ticks = (uint32_t) (((float) STEPPER_TIMER_TICK_HZ / pulse_hz) + 0.5f);
    if (period_ticks < STEPPER_TIMER_MIN_LOAD) {
        period_ticks = STEPPER_TIMER_MIN_LOAD;
    }
    if (period_ticks > STEPPER_TIMER_MAX_LOAD) {
        period_ticks = STEPPER_TIMER_MAX_LOAD;
    }

    compare_ticks = period_ticks / 2U;
    if (compare_ticks == 0U) {
        compare_ticks = 1U;
    }

    DL_TimerG_setLoadValue(obj->timer, period_ticks - 1U);
    DL_TimerG_setCaptureCompareValue(obj->timer, compare_ticks, obj->cc_index);

    return 1U;
}

static uint8_t Stepper_StartPulse(uint8_t axis)
{
    Stepper_AxisRuntime_t *obj = &gStepperAxis[axis];

    if (Stepper_ConfigPulseTimer(axis, obj->current_speed_sps) == 0U) {
        return 0U;
    }

    DL_TimerG_stopCounter(obj->timer);
    DL_TimerG_setTimerCount(obj->timer, 0U);
    DL_TimerG_clearInterruptStatus(obj->timer, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(obj->irqn);

    Stepper_ApplyEnable(axis, 1U);
    obj->busy = 1U;
    obj->state = STEPPER_STATE_MOVING;

    DL_TimerG_enableInterrupt(obj->timer, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(obj->irqn);
    DL_TimerG_startCounter(obj->timer);

    return 1U;
}

static void Stepper_StopPulse(uint8_t axis, BSP_StepperState_t final_state)
{
    Stepper_AxisRuntime_t *obj = &gStepperAxis[axis];

    DL_TimerG_disableInterrupt(obj->timer, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_stopCounter(obj->timer);
    DL_TimerG_setCaptureCompareValue(obj->timer, 0U, obj->cc_index);
    DL_TimerG_clearInterruptStatus(obj->timer, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(obj->irqn);

    obj->busy = 0U;
    obj->stop_requested = 0U;
    obj->current_speed_sps = 0.0f;
    obj->state = final_state;
    Stepper_ApplyEnable(axis, obj->hold_enabled);
    /* Avoid leaving a reverse DIR level that can disturb the other axis. */
    obj->direction_sign = 1;
    Stepper_ApplyDirection(axis);
}

static void Stepper_OnPeriodElapsed(uint8_t axis)
{
    if ((gStepperAxis[axis].state != STEPPER_STATE_MOVING) &&
        (gStepperAxis[axis].state != STEPPER_STATE_STOPPING)) {
        return;
    }

    if (gStepperAxis[axis].remaining_steps > 0U) {
        gStepperAxis[axis].remaining_steps--;
        gStepperAxis[axis].position_steps += gStepperAxis[axis].direction_sign;
    }

    if (gStepperAxis[axis].remaining_steps == 0U) {
        Stepper_StopPulse(axis, STEPPER_STATE_DONE);
        return;
    }

    if (gStepperAxis[axis].stop_requested != 0U) {
        if (gStepperAxis[axis].current_speed_sps <=
            (gStepperAxis[axis].min_speed_sps + STEPPER_SPEED_EPSILON_SPS)) {
            gStepperAxis[axis].remaining_steps = 0U;
            Stepper_StopPulse(axis, STEPPER_STATE_IDLE);
            return;
        }

        gStepperAxis[axis].state = STEPPER_STATE_STOPPING;
    }

    Stepper_UpdateSpeedForNextStep(axis);
    if (Stepper_ConfigPulseTimer(axis, gStepperAxis[axis].current_speed_sps) ==
        0U) {
        gStepperAxis[axis].remaining_steps = 0U;
        Stepper_StopPulse(axis, STEPPER_STATE_ERROR);
    }
}

static void Stepper_UpdateSpeedForNextStep(uint8_t axis)
{
    float current;
    float delta_v;
    float stop_steps;

    current = gStepperAxis[axis].current_speed_sps;
    if (current < gStepperAxis[axis].min_speed_sps) {
        current = gStepperAxis[axis].min_speed_sps;
    }

    delta_v = gStepperAxis[axis].accel_sps2 / current;
    if (delta_v < STEPPER_SPEED_EPSILON_SPS) {
        delta_v = STEPPER_SPEED_EPSILON_SPS;
    }

    stop_steps = (current * current) / (2.0f * gStepperAxis[axis].accel_sps2);

    if ((gStepperAxis[axis].stop_requested != 0U) ||
        (((float) gStepperAxis[axis].remaining_steps) <=
            (stop_steps + 1.0f))) {
        current -= delta_v;
        if (current < gStepperAxis[axis].min_speed_sps) {
            current = gStepperAxis[axis].min_speed_sps;
        }
    } else {
        current += delta_v;
        if (current > gStepperAxis[axis].max_speed_sps) {
            current = gStepperAxis[axis].max_speed_sps;
        }
    }

    if (Stepper_AbsFloat(current) <= STEPPER_SPEED_EPSILON_SPS) {
        current = gStepperAxis[axis].min_speed_sps;
    }

    gStepperAxis[axis].current_speed_sps = current;
}
