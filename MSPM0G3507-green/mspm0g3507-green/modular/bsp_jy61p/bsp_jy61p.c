/*
 * JY61P UART attitude sensor BSP for MSPM0G3507.
 *
 * Port binding:
 * - UART0_TX: PA10 / IOMUX_PINCM21
 * - UART0_RX: PA11 / IOMUX_PINCM22
 */
#include "bsp_jy61p.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define BSP_JY61P_UART_INST (UART0)
#define BSP_JY61P_UART_IRQN (UART0_INT_IRQn)

#define BSP_JY61P_TX_IOMUX (IOMUX_PINCM21)
#define BSP_JY61P_TX_FUNC (IOMUX_PINCM21_PF_UART0_TX)
#define BSP_JY61P_RX_IOMUX (IOMUX_PINCM22)
#define BSP_JY61P_RX_FUNC (IOMUX_PINCM22_PF_UART0_RX)

#define BSP_JY61P_FRAME_SIZE (11U)
#define BSP_JY61P_FRAME_HEAD (0x55U)
#define BSP_JY61P_TYPE_ACC (0x51U)
#define BSP_JY61P_TYPE_GYRO (0x52U)
#define BSP_JY61P_TYPE_ANGLE (0x53U)

#define BSP_JY61P_DEFAULT_BAUDRATE (115200U)
#define BSP_JY61P_FALLBACK_BAUDRATE (9600U)
#define BSP_JY61P_DEFAULT_OFFLINE_TIMEOUT_MS (100U)
#define BSP_JY61P_DETECT_TIMEOUT_MS (300U)
#define BSP_JY61P_CONFIG_STARTUP_DELAY_MS (200U)
#define BSP_JY61P_CONFIG_STEP_DELAY_MS (80U)

#define BSP_JY61P_PI (3.14159265358979323846f)
#define BSP_JY61P_DEG_TO_RAD (BSP_JY61P_PI / 180.0f)

typedef struct {
    uint8_t frame[BSP_JY61P_FRAME_SIZE];
    uint8_t frame_index;

    BSP_JY61P_Data_t data;
    uint32_t offline_timeout_ms;
} BSP_JY61P_State_t;

static volatile uint32_t gJy61pMs;
static uint8_t gJy61pTickStarted;
static uint32_t gJy61pBaudrate;
static BSP_JY61P_State_t gJy61p;

static const DL_UART_Main_ClockConfig gJy61pUartClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gJy61pUartConfig = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

static void BSP_JY61P_ResetRuntime(void);
static int BSP_JY61P_StartTick(void);
static uint32_t BSP_JY61P_GetMs(void);
static void BSP_JY61P_DelayMs(uint32_t delay_ms);
static int BSP_JY61P_SetBaud(uint32_t baudrate);
static uint8_t BSP_JY61P_WaitForSample(uint32_t timeout_ms);
static int BSP_JY61P_SendCommand(uint8_t reg, uint16_t value);
static int BSP_JY61P_WriteRegister(uint8_t reg, uint16_t value);
static int BSP_JY61P_ApplyConfiguration(void);
static void BSP_JY61P_ParseByte(uint8_t byte);
static void BSP_JY61P_ProcessFrame(const uint8_t *frame);
static uint8_t BSP_JY61P_IsFrameType(uint8_t type);
static uint8_t BSP_JY61P_Checksum(const uint8_t *frame);
static int16_t BSP_JY61P_ReadInt16(const uint8_t *buf);
static float BSP_JY61P_NormalizeAngle180(float angle_deg);

int BSP_JY61P_Init(void)
{
    if (BSP_JY61P_StartTick() != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_TICK;
    }

    if (gJy61p.data.initialized != 0U) {
        return BSP_JY61P_OK;
    }

    gJy61p.offline_timeout_ms = BSP_JY61P_DEFAULT_OFFLINE_TIMEOUT_MS;
    BSP_JY61P_DelayMs(BSP_JY61P_CONFIG_STARTUP_DELAY_MS);

    if (BSP_JY61P_SetBaud(BSP_JY61P_DEFAULT_BAUDRATE) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_REINIT;
    }

    BSP_JY61P_ResetRuntime();
    if (BSP_JY61P_WaitForSample(BSP_JY61P_DETECT_TIMEOUT_MS) == 0U) {
        if (BSP_JY61P_SetBaud(BSP_JY61P_FALLBACK_BAUDRATE) != BSP_JY61P_OK) {
            return BSP_JY61P_ERR_UART_REINIT;
        }

        BSP_JY61P_ResetRuntime();
        if (BSP_JY61P_WaitForSample(BSP_JY61P_DETECT_TIMEOUT_MS) == 0U) {
            if (BSP_JY61P_SetBaud(BSP_JY61P_DEFAULT_BAUDRATE) !=
                BSP_JY61P_OK) {
                return BSP_JY61P_ERR_UART_REINIT;
            }

            BSP_JY61P_ResetRuntime();
        }
    }

    if (BSP_JY61P_ApplyConfiguration() != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }

    if (gJy61pBaudrate != BSP_JY61P_DEFAULT_BAUDRATE) {
        if (BSP_JY61P_SetBaud(BSP_JY61P_DEFAULT_BAUDRATE) != BSP_JY61P_OK) {
            return BSP_JY61P_ERR_UART_REINIT;
        }
    }

    BSP_JY61P_ResetRuntime();
    gJy61p.data.initialized = 1U;
    return BSP_JY61P_OK;
}

void BSP_JY61P_Update(void)
{
    uint32_t elapsed_ms;

    if (gJy61p.data.initialized == 0U) {
        return;
    }

    elapsed_ms = (uint32_t)(BSP_JY61P_GetMs() - gJy61p.data.last_frame_ms);
    if ((gJy61p.data.data_valid != 0U) &&
        (elapsed_ms <= gJy61p.offline_timeout_ms)) {
        gJy61p.data.online = 1U;
    } else {
        gJy61p.data.online = 0U;
    }
}

void BSP_JY61P_Tick1ms(void)
{
    gJy61pMs++;
}

void BSP_JY61P_GetData(BSP_JY61P_Data_t *data)
{
    uint32_t primask;

    if (data == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *data = gJy61p.data;
    if (primask == 0U) {
        __enable_irq();
    }
}

float BSP_JY61P_GetPitch(void)
{
    return gJy61p.data.pitch_rad;
}

float BSP_JY61P_GetRoll(void)
{
    return gJy61p.data.roll_rad;
}

float BSP_JY61P_GetYaw(void)
{
    return gJy61p.data.yaw_rad;
}

uint8_t BSP_JY61P_IsOnline(void)
{
    return gJy61p.data.online;
}

uint8_t BSP_JY61P_IsDataValid(void)
{
    return gJy61p.data.data_valid;
}

uint32_t BSP_JY61P_GetSampleSeq(void)
{
    return gJy61p.data.sample_seq;
}

static void BSP_JY61P_ResetRuntime(void)
{
    gJy61p.frame_index = 0U;

    gJy61p.data.pitch_rad = 0.0f;
    gJy61p.data.roll_rad = 0.0f;
    gJy61p.data.yaw_rad = 0.0f;

    gJy61p.data.acc_x_g = 0.0f;
    gJy61p.data.acc_y_g = 0.0f;
    gJy61p.data.acc_z_g = 0.0f;
    gJy61p.data.gyro_x_dps = 0.0f;
    gJy61p.data.gyro_y_dps = 0.0f;
    gJy61p.data.gyro_z_dps = 0.0f;
    gJy61p.data.temperature_c = 0.0f;

    gJy61p.data.last_frame_ms = 0U;
    gJy61p.data.sample_seq = 0U;
    gJy61p.data.initialized = 0U;
    gJy61p.data.online = 0U;
    gJy61p.data.data_valid = 0U;
}

static int BSP_JY61P_StartTick(void)
{
    if (gJy61pTickStarted != 0U) {
        return BSP_JY61P_OK;
    }

    if (DL_SYSTICK_config(CPUCLK_FREQ / 1000U) != 0U) {
        return BSP_JY61P_ERR_TICK;
    }

    gJy61pTickStarted = 1U;
    return BSP_JY61P_OK;
}

static uint32_t BSP_JY61P_GetMs(void)
{
    return gJy61pMs;
}

static void BSP_JY61P_DelayMs(uint32_t delay_ms)
{
    uint32_t start_ms = BSP_JY61P_GetMs();

    while ((uint32_t)(BSP_JY61P_GetMs() - start_ms) < delay_ms) {
        __NOP();
    }
}

static int BSP_JY61P_SetBaud(uint32_t baudrate)
{
    NVIC_DisableIRQ(BSP_JY61P_UART_IRQN);
    DL_UART_Main_reset(BSP_JY61P_UART_INST);
    DL_UART_Main_enablePower(BSP_JY61P_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(BSP_JY61P_TX_IOMUX,
        BSP_JY61P_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(BSP_JY61P_RX_IOMUX,
        BSP_JY61P_RX_FUNC);

    DL_UART_Main_setClockConfig(BSP_JY61P_UART_INST,
        (DL_UART_Main_ClockConfig *) &gJy61pUartClockConfig);
    DL_UART_Main_init(
        BSP_JY61P_UART_INST, (DL_UART_Main_Config *) &gJy61pUartConfig);
    DL_UART_Main_configBaudRate(BSP_JY61P_UART_INST, CPUCLK_FREQ, baudrate);
    DL_UART_Main_enableFIFOs(BSP_JY61P_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(BSP_JY61P_UART_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);

    NVIC_ClearPendingIRQ(BSP_JY61P_UART_IRQN);
    DL_UART_Main_enableInterrupt(BSP_JY61P_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(BSP_JY61P_UART_INST);
    NVIC_EnableIRQ(BSP_JY61P_UART_IRQN);

    gJy61pBaudrate = baudrate;
    return BSP_JY61P_OK;
}

static uint8_t BSP_JY61P_WaitForSample(uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint32_t base_seq;

    base_seq = gJy61p.data.sample_seq;
    start_ms = BSP_JY61P_GetMs();

    while ((uint32_t)(BSP_JY61P_GetMs() - start_ms) < timeout_ms) {
        if (gJy61p.data.sample_seq != base_seq) {
            return 1U;
        }
    }

    return 0U;
}

static int BSP_JY61P_SendCommand(uint8_t reg, uint16_t value)
{
    uint8_t cmd[5];
    uint8_t i;

    cmd[0] = 0xFFU;
    cmd[1] = 0xAAU;
    cmd[2] = reg;
    cmd[3] = (uint8_t)(value & 0x00FFU);
    cmd[4] = (uint8_t)((value >> 8) & 0x00FFU);

    for (i = 0U; i < sizeof(cmd); i++) {
        DL_UART_Main_transmitDataBlocking(BSP_JY61P_UART_INST, cmd[i]);
    }

    return BSP_JY61P_OK;
}

static int BSP_JY61P_WriteRegister(uint8_t reg, uint16_t value)
{
    if (BSP_JY61P_SendCommand(0x69U, 0xB588U) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }
    BSP_JY61P_DelayMs(BSP_JY61P_CONFIG_STEP_DELAY_MS);

    if (BSP_JY61P_SendCommand(reg, value) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }
    BSP_JY61P_DelayMs(BSP_JY61P_CONFIG_STEP_DELAY_MS);

    return BSP_JY61P_OK;
}

static int BSP_JY61P_ApplyConfiguration(void)
{
    if (BSP_JY61P_WriteRegister(0x24U, 0x0001U) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }
    if (BSP_JY61P_WriteRegister(0x02U, 0x000EU) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }
    if (BSP_JY61P_WriteRegister(0x03U, 0x0007U) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }
    if (BSP_JY61P_WriteRegister(0x04U, 0x0006U) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }

    if (BSP_JY61P_SendCommand(0x00U, 0x0000U) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }
    BSP_JY61P_DelayMs(BSP_JY61P_CONFIG_STEP_DELAY_MS);

    if (BSP_JY61P_SendCommand(0x00U, 0x00FFU) != BSP_JY61P_OK) {
        return BSP_JY61P_ERR_UART_TX;
    }
    BSP_JY61P_DelayMs(BSP_JY61P_CONFIG_STARTUP_DELAY_MS);

    return BSP_JY61P_OK;
}

static void BSP_JY61P_ParseByte(uint8_t byte)
{
    if (gJy61p.frame_index == 0U) {
        if (byte == BSP_JY61P_FRAME_HEAD) {
            gJy61p.frame[0] = byte;
            gJy61p.frame_index = 1U;
        }
        return;
    }

    if (gJy61p.frame_index == 1U) {
        if (BSP_JY61P_IsFrameType(byte) == 0U) {
            gJy61p.frame_index = (byte == BSP_JY61P_FRAME_HEAD) ? 1U : 0U;
            gJy61p.frame[0] = BSP_JY61P_FRAME_HEAD;
            return;
        }
    }

    gJy61p.frame[gJy61p.frame_index] = byte;
    gJy61p.frame_index++;

    if (gJy61p.frame_index < BSP_JY61P_FRAME_SIZE) {
        return;
    }

    if (BSP_JY61P_Checksum(gJy61p.frame) ==
        gJy61p.frame[BSP_JY61P_FRAME_SIZE - 1U]) {
        BSP_JY61P_ProcessFrame(gJy61p.frame);
    }

    gJy61p.frame_index = 0U;
}

static void BSP_JY61P_ProcessFrame(const uint8_t *frame)
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    if (frame == NULL) {
        return;
    }

    switch (frame[1]) {
        case BSP_JY61P_TYPE_ACC:
            gJy61p.data.acc_x_g =
                ((float) BSP_JY61P_ReadInt16(&frame[2]) / 32768.0f) * 16.0f;
            gJy61p.data.acc_y_g =
                ((float) BSP_JY61P_ReadInt16(&frame[4]) / 32768.0f) * 16.0f;
            gJy61p.data.acc_z_g =
                ((float) BSP_JY61P_ReadInt16(&frame[6]) / 32768.0f) * 16.0f;
            gJy61p.data.temperature_c =
                (float) BSP_JY61P_ReadInt16(&frame[8]) / 100.0f;
            break;

        case BSP_JY61P_TYPE_GYRO:
            gJy61p.data.gyro_x_dps =
                ((float) BSP_JY61P_ReadInt16(&frame[2]) / 32768.0f) * 2000.0f;
            gJy61p.data.gyro_y_dps =
                ((float) BSP_JY61P_ReadInt16(&frame[4]) / 32768.0f) * 2000.0f;
            gJy61p.data.gyro_z_dps =
                ((float) BSP_JY61P_ReadInt16(&frame[6]) / 32768.0f) * 2000.0f;
            break;

        case BSP_JY61P_TYPE_ANGLE:
            roll_deg = BSP_JY61P_NormalizeAngle180(
                ((float) BSP_JY61P_ReadInt16(&frame[2]) / 32768.0f) * 180.0f);
            pitch_deg = BSP_JY61P_NormalizeAngle180(
                ((float) BSP_JY61P_ReadInt16(&frame[4]) / 32768.0f) * 180.0f);
            yaw_deg = BSP_JY61P_NormalizeAngle180(
                ((float) BSP_JY61P_ReadInt16(&frame[6]) / 32768.0f) * 180.0f);

            gJy61p.data.roll_rad = roll_deg * BSP_JY61P_DEG_TO_RAD;
            gJy61p.data.pitch_rad = pitch_deg * BSP_JY61P_DEG_TO_RAD;
            gJy61p.data.yaw_rad = yaw_deg * BSP_JY61P_DEG_TO_RAD;
            gJy61p.data.last_frame_ms = BSP_JY61P_GetMs();
            gJy61p.data.sample_seq++;
            gJy61p.data.online = 1U;
            gJy61p.data.data_valid = 1U;
            break;

        default:
            break;
    }
}

static uint8_t BSP_JY61P_IsFrameType(uint8_t type)
{
    return (uint8_t)((type >= 0x50U) && (type <= 0x5FU));
}

static uint8_t BSP_JY61P_Checksum(const uint8_t *frame)
{
    uint16_t sum = 0U;
    uint8_t i;

    if (frame == NULL) {
        return 0U;
    }

    for (i = 0U; i < (BSP_JY61P_FRAME_SIZE - 1U); i++) {
        sum = (uint16_t)(sum + frame[i]);
    }

    return (uint8_t)(sum & 0x00FFU);
}

static int16_t BSP_JY61P_ReadInt16(const uint8_t *buf)
{
    uint16_t value;

    if (buf == NULL) {
        return 0;
    }

    value = (uint16_t)(((uint16_t) buf[1] << 8U) | (uint16_t) buf[0]);
    return (int16_t) value;
}

static float BSP_JY61P_NormalizeAngle180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

void UART0_IRQHandler(void)
{
    uint8_t data;

    switch (DL_UART_Main_getPendingInterrupt(BSP_JY61P_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_Main_receiveDataCheck(
                BSP_JY61P_UART_INST, &data)) {
                BSP_JY61P_ParseByte(data);
            }
            break;
        default:
            break;
    }
}
