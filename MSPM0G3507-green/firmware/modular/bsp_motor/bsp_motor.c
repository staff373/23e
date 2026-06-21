#include "bsp_motor.h"

#include "bsp_dcmotor.h"
#include "ti_msp_dl_config.h"

#define MOTOR_PWM_INST ((GPTIMER_Regs *) TIMA1_BASE)
#define MOTOR_PWM_PERIOD_TICKS (1000U)

#define MOTOR_LEFT_PWM_PORT ((GPIO_Regs *) GPIOB_BASE)
#define MOTOR_LEFT_PWM_PIN (DL_GPIO_PIN_4)
#define MOTOR_LEFT_PWM_IOMUX (IOMUX_PINCM17)
#define MOTOR_LEFT_PWM_IOMUX_FUNC (IOMUX_PINCM17_PF_TIMA1_CCP0)
#define MOTOR_LEFT_PWM_CC_INDEX (DL_TIMER_CC_0_INDEX)
#define MOTOR_LEFT_PWM_CCP_OUTPUT (DL_TIMER_CC0_OUTPUT)

#define MOTOR_RIGHT_PWM_PORT ((GPIO_Regs *) GPIOB_BASE)
#define MOTOR_RIGHT_PWM_PIN (DL_GPIO_PIN_5)
#define MOTOR_RIGHT_PWM_IOMUX (IOMUX_PINCM18)
#define MOTOR_RIGHT_PWM_IOMUX_FUNC (IOMUX_PINCM18_PF_TIMA1_CCP1)
#define MOTOR_RIGHT_PWM_CC_INDEX (DL_TIMER_CC_1_INDEX)
#define MOTOR_RIGHT_PWM_CCP_OUTPUT (DL_TIMER_CC1_OUTPUT)

#define MOTOR_LEFT_AIN1_PORT ((GPIO_Regs *) GPIOB_BASE)
#define MOTOR_LEFT_AIN1_PIN (DL_GPIO_PIN_3)
#define MOTOR_LEFT_AIN1_IOMUX (IOMUX_PINCM16)
#define MOTOR_LEFT_AIN2_PORT ((GPIO_Regs *) GPIOB_BASE)
#define MOTOR_LEFT_AIN2_PIN (DL_GPIO_PIN_2)
#define MOTOR_LEFT_AIN2_IOMUX (IOMUX_PINCM15)

#define MOTOR_RIGHT_BIN1_PORT ((GPIO_Regs *) GPIOA_BASE)
#define MOTOR_RIGHT_BIN1_PIN (DL_GPIO_PIN_1)
#define MOTOR_RIGHT_BIN1_IOMUX (IOMUX_PINCM2)
#define MOTOR_RIGHT_BIN2_PORT ((GPIO_Regs *) GPIOA_BASE)
#define MOTOR_RIGHT_BIN2_PIN (DL_GPIO_PIN_0)
#define MOTOR_RIGHT_BIN2_IOMUX (IOMUX_PINCM1)

#define MOTOR_LEFT_REVERSE_FLAG (1U)
#define MOTOR_RIGHT_REVERSE_FLAG (1U)

static void BSP_Motor_GPIO_Init(void);
static void BSP_Motor_PWM_Init(void);

static uint8_t gMotorInitialized;

static DCMotor_t gLeftMotor = {
    .pwm_timer = MOTOR_PWM_INST,
    .pwm_cc_index = MOTOR_LEFT_PWM_CC_INDEX,
    .pwm_period_ticks = MOTOR_PWM_PERIOD_TICKS,
    .pwm_compare_inverted = 0U,

    .dir_gpio_port = MOTOR_LEFT_AIN1_PORT,
    .dir_pin = MOTOR_LEFT_AIN1_PIN,
    .dir_forward_state = DCMOTOR_PIN_HIGH,
    .dir_reverse_state = DCMOTOR_PIN_LOW,

    .dir_aux_gpio_port = MOTOR_LEFT_AIN2_PORT,
    .dir_aux_pin = MOTOR_LEFT_AIN2_PIN,
    .dir_aux_forward_state = DCMOTOR_PIN_LOW,
    .dir_aux_reverse_state = DCMOTOR_PIN_HIGH,

    .reverse_flag = MOTOR_LEFT_REVERSE_FLAG,
    .stop_use_active_brake = 0U,
    .brake_duty_percent = 100.0f,
    .brake_dir_state = DCMOTOR_PIN_HIGH,
    .brake_aux_dir_state = DCMOTOR_PIN_HIGH,
};

static DCMotor_t gRightMotor = {
    .pwm_timer = MOTOR_PWM_INST,
    .pwm_cc_index = MOTOR_RIGHT_PWM_CC_INDEX,
    .pwm_period_ticks = MOTOR_PWM_PERIOD_TICKS,
    .pwm_compare_inverted = 0U,

    .dir_gpio_port = MOTOR_RIGHT_BIN1_PORT,
    .dir_pin = MOTOR_RIGHT_BIN1_PIN,
    .dir_forward_state = DCMOTOR_PIN_HIGH,
    .dir_reverse_state = DCMOTOR_PIN_LOW,

    .dir_aux_gpio_port = MOTOR_RIGHT_BIN2_PORT,
    .dir_aux_pin = MOTOR_RIGHT_BIN2_PIN,
    .dir_aux_forward_state = DCMOTOR_PIN_LOW,
    .dir_aux_reverse_state = DCMOTOR_PIN_HIGH,

    .reverse_flag = MOTOR_RIGHT_REVERSE_FLAG,
    .stop_use_active_brake = 0U,
    .brake_duty_percent = 100.0f,
    .brake_dir_state = DCMOTOR_PIN_HIGH,
    .brake_aux_dir_state = DCMOTOR_PIN_HIGH,
};

static const DL_TimerA_ClockConfig gMotorPwmClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_4,
    .prescale = 0U,
};

static const DL_TimerA_PWMConfig gMotorPwmConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = MOTOR_PWM_PERIOD_TICKS,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

void BSP_Motor_Init(void)
{
    if (gMotorInitialized != 0U) {
        return;
    }

    BSP_Motor_GPIO_Init();
    BSP_Motor_PWM_Init();

    BSP_DCMotor_Init(&gLeftMotor);
    BSP_DCMotor_Init(&gRightMotor);
    gMotorInitialized = 1U;
    BSP_Motor_Stop();
}

void BSP_Motor_SetDuty(float left_duty, float right_duty)
{
    if (gMotorInitialized == 0U) {
        BSP_Motor_Init();
    }

    BSP_DCMotor_SetDuty(&gLeftMotor, left_duty);
    BSP_DCMotor_SetDuty(&gRightMotor, right_duty);
}

void BSP_Motor_SetLeftDuty(float duty)
{
    if (gMotorInitialized == 0U) {
        BSP_Motor_Init();
    }

    BSP_DCMotor_SetDuty(&gLeftMotor, duty);
}

void BSP_Motor_SetRightDuty(float duty)
{
    if (gMotorInitialized == 0U) {
        BSP_Motor_Init();
    }

    BSP_DCMotor_SetDuty(&gRightMotor, duty);
}

void BSP_Motor_Stop(void)
{
    if (gMotorInitialized == 0U) {
        BSP_Motor_Init();
        return;
    }

    BSP_DCMotor_Stop(&gLeftMotor);
    BSP_DCMotor_Stop(&gRightMotor);
}

static void BSP_Motor_GPIO_Init(void)
{
    DL_GPIO_initDigitalOutput(MOTOR_LEFT_AIN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_LEFT_AIN2_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_RIGHT_BIN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_RIGHT_BIN2_IOMUX);

    DL_GPIO_clearPins(MOTOR_LEFT_AIN1_PORT,
        MOTOR_LEFT_AIN1_PIN | MOTOR_LEFT_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_RIGHT_BIN1_PORT,
        MOTOR_RIGHT_BIN1_PIN | MOTOR_RIGHT_BIN2_PIN);

    DL_GPIO_enableOutput(MOTOR_LEFT_AIN1_PORT,
        MOTOR_LEFT_AIN1_PIN | MOTOR_LEFT_AIN2_PIN);
    DL_GPIO_enableOutput(MOTOR_RIGHT_BIN1_PORT,
        MOTOR_RIGHT_BIN1_PIN | MOTOR_RIGHT_BIN2_PIN);
}

static void BSP_Motor_PWM_Init(void)
{
    DL_TimerA_reset(MOTOR_PWM_INST);
    DL_TimerA_enablePower(MOTOR_PWM_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(
        MOTOR_LEFT_PWM_IOMUX, MOTOR_LEFT_PWM_IOMUX_FUNC);
    DL_GPIO_enableOutput(MOTOR_LEFT_PWM_PORT, MOTOR_LEFT_PWM_PIN);
    DL_GPIO_initPeripheralOutputFunction(
        MOTOR_RIGHT_PWM_IOMUX, MOTOR_RIGHT_PWM_IOMUX_FUNC);
    DL_GPIO_enableOutput(MOTOR_RIGHT_PWM_PORT, MOTOR_RIGHT_PWM_PIN);

    DL_TimerA_setClockConfig(
        MOTOR_PWM_INST, (DL_TimerA_ClockConfig *) &gMotorPwmClockConfig);
    DL_TimerA_initPWMMode(
        MOTOR_PWM_INST, (DL_TimerA_PWMConfig *) &gMotorPwmConfig);

    /* Counter control is global for TIMA1; use the same CCCTL0 source
     * SysConfig emits for two-channel PWM. */
    DL_TimerA_setCounterControl(MOTOR_PWM_INST, DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND, DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, MOTOR_LEFT_PWM_CC_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, MOTOR_LEFT_PWM_CC_INDEX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST, 0U, MOTOR_LEFT_PWM_CC_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, MOTOR_RIGHT_PWM_CC_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, MOTOR_RIGHT_PWM_CC_INDEX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST, 0U, MOTOR_RIGHT_PWM_CC_INDEX);

    DL_TimerA_enableClock(MOTOR_PWM_INST);
    DL_TimerA_setCCPDirection(MOTOR_PWM_INST,
        MOTOR_LEFT_PWM_CCP_OUTPUT | MOTOR_RIGHT_PWM_CCP_OUTPUT);
}
