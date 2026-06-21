#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdint.h>

void BSP_OLED_Init(void);
void BSP_OLED_Clear(void);
void BSP_OLED_ShowChar(uint8_t line, uint8_t column, char ch);
void BSP_OLED_ShowString(uint8_t line, uint8_t column, const char *str);
void BSP_OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void BSP_OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length);
void BSP_OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void BSP_OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

#endif
