/*
 * UART1 vision link BSP for MSPM0G3507.
 *
 * Port binding:
 * - UART1_TX: PA8 / IOMUX_PINCM19
 * - UART1_RX: PA9 / IOMUX_PINCM20
 */
#ifndef BSP_VISION_UART_H
#define BSP_VISION_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void VisionUart_Init(void);
void VisionUart_ConfigBaudRate(uint32_t clockFreq, uint32_t baudRate);
bool VisionUart_SendByte(uint8_t data);
bool VisionUart_SendString(const char *str);
bool VisionUart_ReadByte(uint8_t *data);
uint16_t VisionUart_ReadAvailable(void);
void VisionUart_ClearRxBuffer(void);
uint32_t VisionUart_GetOverflowCount(void);
uint32_t VisionUart_GetTxTimeoutCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_VISION_UART_H */
