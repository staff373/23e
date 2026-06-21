/*
 * UART1 vision link BSP for MSPM0G3507.
 */
#include "bsp_vision_uart.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define VISION_UART_INST (UART1)
#define VISION_UART_IRQN (UART1_INT_IRQn)

#define VISION_UART_TX_IOMUX (IOMUX_PINCM19)
#define VISION_UART_TX_FUNC (IOMUX_PINCM19_PF_UART1_TX)
#define VISION_UART_RX_IOMUX (IOMUX_PINCM20)
#define VISION_UART_RX_FUNC (IOMUX_PINCM20_PF_UART1_RX)

#ifndef VISION_UART_BAUD_RATE
#define VISION_UART_BAUD_RATE (115200U)
#endif

#ifndef VISION_UART_CLOCK_FREQ
#define VISION_UART_CLOCK_FREQ (CPUCLK_FREQ / 2U)
#endif

#define VISION_UART_RX_BUFFER_SIZE (256U)
#define VISION_UART_TX_TIMEOUT_LOOPS (20000U)

static volatile uint8_t gVisionRxBuffer[VISION_UART_RX_BUFFER_SIZE];
static volatile uint16_t gVisionRxHead;
static volatile uint16_t gVisionRxTail;
static volatile uint32_t gVisionRxOverflowCount;
static volatile uint32_t gVisionTxTimeoutCount;

static const DL_UART_Main_ClockConfig gVisionUartClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gVisionUartConfig = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

static uint16_t VisionUart_AdvanceIndex(uint16_t index)
{
    index++;
    if (index >= VISION_UART_RX_BUFFER_SIZE) {
        index = 0U;
    }

    return index;
}

static void VisionUart_StoreRxByte(uint8_t data)
{
    uint16_t nextHead = VisionUart_AdvanceIndex(gVisionRxHead);

    if (nextHead == gVisionRxTail) {
        gVisionRxOverflowCount++;
        return;
    }

    gVisionRxBuffer[gVisionRxHead] = data;
    gVisionRxHead = nextHead;
}

void VisionUart_Init(void)
{
    gVisionRxHead = 0U;
    gVisionRxTail = 0U;
    gVisionRxOverflowCount = 0U;
    gVisionTxTimeoutCount = 0U;

    DL_UART_Main_reset(VISION_UART_INST);
    DL_UART_Main_enablePower(VISION_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(
        VISION_UART_TX_IOMUX, VISION_UART_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        VISION_UART_RX_IOMUX, VISION_UART_RX_FUNC);

    DL_UART_Main_setClockConfig(VISION_UART_INST,
        (DL_UART_Main_ClockConfig *) &gVisionUartClockConfig);
    DL_UART_Main_init(
        VISION_UART_INST, (DL_UART_Main_Config *) &gVisionUartConfig);
    DL_UART_Main_configBaudRate(
        VISION_UART_INST, VISION_UART_CLOCK_FREQ, VISION_UART_BAUD_RATE);

    DL_UART_Main_enableFIFOs(VISION_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(
        VISION_UART_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);

    NVIC_ClearPendingIRQ(VISION_UART_IRQN);
    DL_UART_Main_enableInterrupt(VISION_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(VISION_UART_INST);
    NVIC_EnableIRQ(VISION_UART_IRQN);
}

void VisionUart_ConfigBaudRate(uint32_t clockFreq, uint32_t baudRate)
{
    NVIC_DisableIRQ(VISION_UART_IRQN);
    DL_UART_Main_disable(VISION_UART_INST);
    DL_UART_Main_configBaudRate(VISION_UART_INST, clockFreq, baudRate);
    DL_UART_Main_enable(VISION_UART_INST);
    gVisionRxHead = 0U;
    gVisionRxTail = 0U;
    NVIC_ClearPendingIRQ(VISION_UART_IRQN);
    NVIC_EnableIRQ(VISION_UART_IRQN);
}

bool VisionUart_SendByte(uint8_t data)
{
    uint32_t wait;

    for (wait = 0U; wait < VISION_UART_TX_TIMEOUT_LOOPS; wait++) {
        if (DL_UART_Main_transmitDataCheck(VISION_UART_INST, data)) {
            return true;
        }
    }

    gVisionTxTimeoutCount++;
    return false;
}

bool VisionUart_SendString(const char *str)
{
    if (str == NULL) {
        return false;
    }

    while (*str != '\0') {
        if (!VisionUart_SendByte((uint8_t) *str)) {
            return false;
        }
        str++;
    }

    return true;
}

bool VisionUart_ReadByte(uint8_t *data)
{
    uint16_t tail;

    if (data == NULL) {
        return false;
    }

    if (gVisionRxHead == gVisionRxTail) {
        return false;
    }

    tail = gVisionRxTail;
    *data = gVisionRxBuffer[tail];
    gVisionRxTail = VisionUart_AdvanceIndex(tail);

    return true;
}

uint16_t VisionUart_ReadAvailable(void)
{
    uint16_t head = gVisionRxHead;
    uint16_t tail = gVisionRxTail;

    if (head >= tail) {
        return head - tail;
    }

    return (VISION_UART_RX_BUFFER_SIZE - tail) + head;
}

void VisionUart_ClearRxBuffer(void)
{
    NVIC_DisableIRQ(VISION_UART_IRQN);
    gVisionRxHead = 0U;
    gVisionRxTail = 0U;
    NVIC_ClearPendingIRQ(VISION_UART_IRQN);
    NVIC_EnableIRQ(VISION_UART_IRQN);
}

uint32_t VisionUart_GetOverflowCount(void)
{
    return gVisionRxOverflowCount;
}

uint32_t VisionUart_GetTxTimeoutCount(void)
{
    return gVisionTxTimeoutCount;
}

void UART1_IRQHandler(void)
{
    uint8_t data;

    switch (DL_UART_Main_getPendingInterrupt(VISION_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_Main_receiveDataCheck(VISION_UART_INST, &data)) {
                VisionUart_StoreRxByte(data);
            }
            break;
        default:
            break;
    }
}
