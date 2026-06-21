/*
 * Green contest success indicator.
 */
#ifndef APP_GREEN_INDICATOR_H
#define APP_GREEN_INDICATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

void GreenIndicator_Init(void);
void GreenIndicator_Tick1ms(void);
void GreenIndicator_Poll(void);
void GreenIndicator_SetSuccess(uint8_t success);
uint8_t GreenIndicator_IsSuccessOn(void);
void GreenIndicator_FormatStatus(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* APP_GREEN_INDICATOR_H */
