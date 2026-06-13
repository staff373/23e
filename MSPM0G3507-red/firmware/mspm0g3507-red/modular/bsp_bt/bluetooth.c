/* UART3 Bluetooth low-level driver. */
#include "bluetooth.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define BLUETOOTH_UART_INST (UART3)
#define BLUETOOTH_UART_IRQN (UART3_INT_IRQn)

#define BLUETOOTH_TX_IOMUX (IOMUX_PINCM59)
#define BLUETOOTH_TX_FUNC (IOMUX_PINCM59_PF_UART3_TX)
#define BLUETOOTH_TX_GPIO_PORT (GPIOA)
#define BLUETOOTH_TX_GPIO_PIN (DL_GPIO_PIN_26)
#define BLUETOOTH_RX_IOMUX (IOMUX_PINCM55)
#define BLUETOOTH_RX_FUNC (IOMUX_PINCM55_PF_UART3_RX)

#ifndef BLUETOOTH_BAUD_RATE
#define BLUETOOTH_BAUD_RATE (115200U)
#endif

#ifndef BLUETOOTH_UART_CLOCK_FREQ
#define BLUETOOTH_UART_CLOCK_FREQ (CPUCLK_FREQ)
#endif

#define BLUETOOTH_RX_BUFFER_SIZE (128U)

static volatile uint8_t gRxBuffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint16_t gRxHead;
static volatile uint16_t gRxTail;
static volatile uint32_t gRxOverflowCount;

static const DL_UART_Main_ClockConfig gBluetoothUartClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gBluetoothUartConfig = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

static void Bluetooth_diagSoftTxDelay(uint32_t baudRate);
static void Bluetooth_diagSoftSendByte(uint32_t baudRate, uint8_t data);

static uint16_t Bluetooth_advanceIndex(uint16_t index)
{
    index++;
    if (index >= BLUETOOTH_RX_BUFFER_SIZE) {
        index = 0U;
    }

    return index;
}

static void Bluetooth_storeRxByte(uint8_t data)
{
    uint16_t nextHead = Bluetooth_advanceIndex(gRxHead);

    if (nextHead == gRxTail) {
        gRxOverflowCount++;
        return;
    }

    gRxBuffer[gRxHead] = data;
    gRxHead = nextHead;
}

static bool Bluetooth_scanEscapedLineEnd(uint16_t scan, uint16_t head)
{
    uint16_t next;

    if (scan == head) {
        return false;
    }

    if (gRxBuffer[scan] == 'n') {
        return true;
    }

    if (gRxBuffer[scan] != 'r') {
        return false;
    }

    next = Bluetooth_advanceIndex(scan);
    if (next == head) {
        return false;
    }
    if (gRxBuffer[next] != '\\') {
        return false;
    }

    next = Bluetooth_advanceIndex(next);
    if (next == head) {
        return false;
    }

    return (gRxBuffer[next] == 'n');
}

void Bluetooth_init(void)
{
    gRxHead = 0U;
    gRxTail = 0U;
    gRxOverflowCount = 0U;

    DL_UART_Main_reset(BLUETOOTH_UART_INST);
    DL_UART_Main_enablePower(BLUETOOTH_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(BLUETOOTH_TX_IOMUX, BLUETOOTH_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(BLUETOOTH_RX_IOMUX, BLUETOOTH_RX_FUNC);

    DL_UART_Main_setClockConfig(BLUETOOTH_UART_INST,
        (DL_UART_Main_ClockConfig *) &gBluetoothUartClockConfig);
    DL_UART_Main_init(
        BLUETOOTH_UART_INST, (DL_UART_Main_Config *) &gBluetoothUartConfig);
    DL_UART_Main_configBaudRate(
        BLUETOOTH_UART_INST, BLUETOOTH_UART_CLOCK_FREQ, BLUETOOTH_BAUD_RATE);

    DL_UART_Main_enableFIFOs(BLUETOOTH_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(
        BLUETOOTH_UART_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);

    NVIC_ClearPendingIRQ(BLUETOOTH_UART_IRQN);
    DL_UART_Main_enableInterrupt(
        BLUETOOTH_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(BLUETOOTH_UART_INST);
    NVIC_EnableIRQ(BLUETOOTH_UART_IRQN);
}

void Bluetooth_configBaudRate(uint32_t clockFreq, uint32_t baudRate)
{
    NVIC_DisableIRQ(BLUETOOTH_UART_IRQN);
    DL_UART_Main_disable(BLUETOOTH_UART_INST);
    DL_UART_Main_configBaudRate(BLUETOOTH_UART_INST, clockFreq, baudRate);
    DL_UART_Main_enable(BLUETOOTH_UART_INST);
    gRxHead = 0U;
    gRxTail = 0U;
    NVIC_ClearPendingIRQ(BLUETOOTH_UART_IRQN);
    NVIC_EnableIRQ(BLUETOOTH_UART_IRQN);
}

void Bluetooth_diagSoftTxInit(void)
{
    NVIC_DisableIRQ(BLUETOOTH_UART_IRQN);
    DL_UART_Main_disable(BLUETOOTH_UART_INST);

    DL_GPIO_initDigitalOutput(BLUETOOTH_TX_IOMUX);
    DL_GPIO_setPins(BLUETOOTH_TX_GPIO_PORT, BLUETOOTH_TX_GPIO_PIN);
    DL_GPIO_enableOutput(BLUETOOTH_TX_GPIO_PORT, BLUETOOTH_TX_GPIO_PIN);
}

void Bluetooth_diagSoftSendString(uint32_t baudRate, const char *str)
{
    if ((baudRate == 0U) || (str == NULL)) {
        return;
    }

    while (*str != '\0') {
        Bluetooth_diagSoftSendByte(baudRate, (uint8_t) *str);
        str++;
    }
}

void Bluetooth_sendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(BLUETOOTH_UART_INST, data);
}

void Bluetooth_sendString(const char *str)
{
    if (str == NULL) {
        return;
    }

    while (*str != '\0') {
        Bluetooth_sendByte((uint8_t) *str);
        str++;
    }
}

bool Bluetooth_readByte(uint8_t *data)
{
    uint16_t tail;

    if (data == NULL) {
        return false;
    }

    if (gRxHead == gRxTail) {
        return false;
    }

    tail = gRxTail;
    *data = gRxBuffer[tail];
    gRxTail = Bluetooth_advanceIndex(tail);

    return true;
}

static void Bluetooth_diagSoftTxDelay(uint32_t baudRate)
{
    uint32_t cycles;

    cycles = CPUCLK_FREQ / baudRate;
    if (cycles < 1U) {
        cycles = 1U;
    }

    delay_cycles(cycles);
}

static void Bluetooth_diagSoftSendByte(uint32_t baudRate, uint8_t data)
{
    uint8_t i;

    __disable_irq();

    DL_GPIO_clearPins(BLUETOOTH_TX_GPIO_PORT, BLUETOOTH_TX_GPIO_PIN);
    Bluetooth_diagSoftTxDelay(baudRate);

    for (i = 0U; i < 8U; i++) {
        if ((data & 0x01U) != 0U) {
            DL_GPIO_setPins(BLUETOOTH_TX_GPIO_PORT, BLUETOOTH_TX_GPIO_PIN);
        } else {
            DL_GPIO_clearPins(BLUETOOTH_TX_GPIO_PORT, BLUETOOTH_TX_GPIO_PIN);
        }

        Bluetooth_diagSoftTxDelay(baudRate);
        data >>= 1U;
    }

    DL_GPIO_setPins(BLUETOOTH_TX_GPIO_PORT, BLUETOOTH_TX_GPIO_PIN);
    Bluetooth_diagSoftTxDelay(baudRate);

    __enable_irq();
}

uint16_t Bluetooth_readAvailable(void)
{
    uint16_t head = gRxHead;
    uint16_t tail = gRxTail;

    if (head >= tail) {
        return head - tail;
    }

    return (BLUETOOTH_RX_BUFFER_SIZE - tail) + head;
}

void Bluetooth_clearRxBuffer(void)
{
    NVIC_DisableIRQ(BLUETOOTH_UART_IRQN);
    gRxHead = 0U;
    gRxTail = 0U;
    NVIC_ClearPendingIRQ(BLUETOOTH_UART_IRQN);
    NVIC_EnableIRQ(BLUETOOTH_UART_IRQN);
}

bool Bluetooth_readLine(char *buffer, uint16_t bufferSize)
{
    uint16_t scan;
    uint16_t head;
    uint16_t count = 0U;
    bool hasLine = false;
    bool willFillBuffer = false;
    uint8_t data;

    if ((buffer == NULL) || (bufferSize == 0U)) {
        return false;
    }

    buffer[0] = '\0';
    if (bufferSize == 1U) {
        return false;
    }

    scan = gRxTail;
    head = gRxHead;
    while (scan != head) {
        data = gRxBuffer[scan];
        scan = Bluetooth_advanceIndex(scan);

        if (data == '\r') {
            continue;
        }
        if (data == '\n') {
            hasLine = true;
            break;
        }
        if ((data == '\\') && Bluetooth_scanEscapedLineEnd(scan, head)) {
            hasLine = true;
            break;
        }

        count++;
        if (count >= (uint16_t) (bufferSize - 1U)) {
            willFillBuffer = true;
            break;
        }
    }

    if (!hasLine && !willFillBuffer) {
        return false;
    }

    count = 0U;
    while (Bluetooth_readByte(&data)) {
        if (data == '\r') {
            continue;
        }
        if (data == '\n') {
            buffer[count] = '\0';
            return true;
        }
        if (data == '\\') {
            uint8_t escaped;

            if (!Bluetooth_readByte(&escaped)) {
                buffer[count] = (char) data;
                count++;
                break;
            }
            if (escaped == 'r') {
                continue;
            }
            if (escaped == 'n') {
                buffer[count] = '\0';
                return true;
            }

            buffer[count] = (char) data;
            count++;
            if (count >= (uint16_t) (bufferSize - 1U)) {
                break;
            }

            buffer[count] = (char) escaped;
            count++;
            if (count >= (uint16_t) (bufferSize - 1U)) {
                break;
            }

            continue;
        }

        buffer[count] = (char) data;
        count++;
        if (count >= (uint16_t) (bufferSize - 1U)) {
            break;
        }
    }

    buffer[count] = '\0';

    return true;
}

uint32_t Bluetooth_getOverflowCount(void)
{
    return gRxOverflowCount;
}

void UART3_IRQHandler(void)
{
    uint8_t data;

    switch (DL_UART_Main_getPendingInterrupt(BLUETOOTH_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_Main_receiveDataCheck(
                BLUETOOTH_UART_INST, &data)) {
                Bluetooth_storeRxByte(data);
            }
            break;
        default:
            break;
    }
}
