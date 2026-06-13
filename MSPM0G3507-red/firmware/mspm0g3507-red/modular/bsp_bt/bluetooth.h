/* UART3 Bluetooth low-level driver API. */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>

void Bluetooth_init(void);
void Bluetooth_configBaudRate(uint32_t clockFreq, uint32_t baudRate);
void Bluetooth_diagSoftTxInit(void);
void Bluetooth_diagSoftSendString(uint32_t baudRate, const char *str);
void Bluetooth_sendByte(uint8_t data);
void Bluetooth_sendString(const char *str);
bool Bluetooth_readByte(uint8_t *data);
uint16_t Bluetooth_readAvailable(void);
void Bluetooth_clearRxBuffer(void);
bool Bluetooth_readLine(char *buffer, uint16_t bufferSize);
uint32_t Bluetooth_getOverflowCount(void);

#endif /* BLUETOOTH_H */
