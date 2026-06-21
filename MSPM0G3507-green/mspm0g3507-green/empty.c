/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include "modular/app_bt_console/app_bt_console.h"
#include "modular/app_green_system/app_green_system.h"
#include "modular/app_keys/app_keys.h"
#include "modular/app_vision_comm/app_vision_comm.h"
#include "modular/bsp_bt/bluetooth.h"

#include <stdio.h>

#define RUN_LED_BLINK_MS (250U)

static volatile uint32_t gMainMs;
static uint32_t gRunLedLastToggleMs;

static void Main_Start1msTick(void);
static uint32_t Main_NowMs(void);
static void Main_InitRuntime(void);
static const char *Main_ResetCauseName(DL_SYSCTL_RESET_CAUSE cause);
static void Main_SendBootBanner(void);
static void Main_DispatchKeyEvents(void);
static void Main_DispatchKeyEvent(AppKeyEvent_t event);
static void Main_ServiceRunLed(void);

static void Main_Start1msTick(void)
{
    (void) DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
}

static uint32_t Main_NowMs(void)
{
    return gMainMs;
}

static void Main_InitRuntime(void)
{
    AppKeys_Init();
    Bluetooth_init();
    VisionComm_Init();
    GreenSystem_Init();
    AppBtConsole_Init();

    Main_SendBootBanner();
}

static const char *Main_ResetCauseName(DL_SYSCTL_RESET_CAUSE cause)
{
    switch (cause) {
    case DL_SYSCTL_RESET_CAUSE_NO_RESET:
        return "NO_RESET";
    case DL_SYSCTL_RESET_CAUSE_POR_HW_FAILURE:
        return "POR_HW";
    case DL_SYSCTL_RESET_CAUSE_POR_EXTERNAL_NRST:
        return "POR_NRST";
    case DL_SYSCTL_RESET_CAUSE_POR_SW_TRIGGERED:
        return "POR_SW";
    case DL_SYSCTL_RESET_CAUSE_BOR_SUPPLY_FAILURE:
        return "BOR_SUPPLY";
    case DL_SYSCTL_RESET_CAUSE_BOR_WAKE_FROM_SHUTDOWN:
        return "BOR_WAKE";
    case DL_SYSCTL_RESET_CAUSE_BOOTRST_NON_PMU_PARITY_FAULT:
        return "BOOT_PARITY";
    case DL_SYSCTL_RESET_CAUSE_BOOTRST_CLOCK_FAULT:
        return "BOOT_CLOCK";
    case DL_SYSCTL_RESET_CAUSE_BOOTRST_SW_TRIGGERED:
        return "BOOT_SW";
    case DL_SYSCTL_RESET_CAUSE_BOOTRST_EXTERNAL_NRST:
        return "BOOT_NRST";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_BSL_EXIT:
        return "SYS_BSL_EXIT";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_BSL_ENTRY:
        return "SYS_BSL_ENTRY";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT0_VIOLATION:
        return "SYS_WWDT0";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT1_VIOLATION:
        return "SYS_WWDT1";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_FLASH_ECC_ERROR:
        return "SYS_FLASH_ECC";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_CPU_LOCKUP_VIOLATION:
        return "SYS_CPU_LOCK";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_DEBUG_TRIGGERED:
        return "SYS_DEBUG";
    case DL_SYSCTL_RESET_CAUSE_SYSRST_SW_TRIGGERED:
        return "SYS_SW";
    case DL_SYSCTL_RESET_CAUSE_CPURST_DEBUG_TRIGGERED:
        return "CPU_DEBUG";
    case DL_SYSCTL_RESET_CAUSE_CPURST_SW_TRIGGERED:
        return "CPU_SW";
    default:
        return "UNKNOWN";
    }
}

static void Main_SendBootBanner(void)
{
    char buffer[80];
    DL_SYSCTL_RESET_CAUSE cause = DL_SYSCTL_getResetCause();

    (void) snprintf(buffer, sizeof(buffer),
        "BOOT GREEN READY reset=%s raw=%lu\r\n",
        Main_ResetCauseName(cause),
        (unsigned long) cause);
    Bluetooth_sendString(buffer);
    Bluetooth_sendString("HELP for commands\r\n");
}

static void Main_DispatchKeyEvents(void)
{
    AppKeyEvent_t event;

    while (AppKeys_PopEvent(&event)) {
        Main_DispatchKeyEvent(event);
    }
}

static void Main_DispatchKeyEvent(AppKeyEvent_t event)
{
    switch (event) {
    case APP_KEY_EVENT_K1_SHORT:
        GreenSystem_Command(GREEN_CMD_START1, GREEN_SYSTEM_SOURCE_KEY);
        break;

    case APP_KEY_EVENT_K2_SHORT:
        GreenSystem_Command(GREEN_CMD_START2, GREEN_SYSTEM_SOURCE_KEY);
        break;

    case APP_KEY_EVENT_K3_SHORT:
        GreenSystem_Command(GREEN_CMD_STOP_RESET, GREEN_SYSTEM_SOURCE_KEY);
        break;

    case APP_KEY_EVENT_K1_LONG:
    case APP_KEY_EVENT_NONE:
    default:
        break;
    }
}

static void Main_ServiceRunLed(void)
{
    uint32_t now = Main_NowMs();

    if ((uint32_t)(now - gRunLedLastToggleMs) < RUN_LED_BLINK_MS) {
        return;
    }

    gRunLedLastToggleMs = now;
    DL_GPIO_togglePins(RUN_LED_PORT, RUN_LED_LED0_PIN);
}

int main(void)
{
    SYSCFG_DL_init();
    Main_Start1msTick();
    DL_GPIO_clearPins(RUN_LED_PORT, RUN_LED_LED0_PIN);
    Main_InitRuntime();

    while (1) {
        AppKeys_Poll();
        Main_DispatchKeyEvents();
        VisionComm_Poll();
        AppBtConsole_Poll();
        GreenSystem_Poll();
        Main_ServiceRunLed();
    }
}

void SysTick_Handler(void)
{
    gMainMs++;
    AppKeys_Tick1ms();
    VisionComm_Tick1ms();
    GreenSystem_Tick1ms();
    AppBtConsole_Tick1ms();
}
