/*
 * Bluetooth command console for the green tracking system.
 */
#ifndef APP_BT_CONSOLE_H
#define APP_BT_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

void AppBtConsole_Init(void);
void AppBtConsole_Tick1ms(void);
void AppBtConsole_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BT_CONSOLE_H */
