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

#include "bsp_key_leds.h"
#include "bsp_keys.h"
#include "bluetooth.h"
#include "bsp_jy61p.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include "bsp_stepper.h"
#include "app_vision_comm.h"

#define MAIN_LOOP_DELAY_CYCLES (80000U)
#define RUN_LED_BLINK_TICKS (50U)
#define MAIN_ENABLE_BLUETOOTH_STARTUP (1U)
#define MAIN_ENABLE_VISION_STARTUP (1U)
#define MAIN_ENABLE_STEPPER_STARTUP (1U)
#define MAIN_ENABLE_OLED_STARTUP (0U)
#define MAIN_ENABLE_JY61P (0U)
#define MAIN_ENABLE_DCMOTOR_STARTUP (0U)
#define KEY_MASK_K1 ((uint8_t) (1U << (uint8_t) BSP_KEY_K1))
#define KEY_MASK_K2 ((uint8_t) (1U << (uint8_t) BSP_KEY_K2))
#define KEY_MASK_K3 ((uint8_t) (1U << (uint8_t) BSP_KEY_K3))

static volatile uint32_t gMainMs;
static uint8_t gLatchedKeyMask;

static void start_1ms_tick(void)
{
    (void) DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
}

static uint8_t read_key_mask(void)
{
    BSP_KeysRaw_t raw;
    uint8_t pressedMask = 0U;

    BSP_Keys_GetRaw(&raw);

    if (raw.pressed[(uint8_t) BSP_KEY_K1] != 0U) {
        pressedMask |= KEY_MASK_K1;
    }
    if (raw.pressed[(uint8_t) BSP_KEY_K2] != 0U) {
        pressedMask |= KEY_MASK_K2;
    }
    if (raw.pressed[(uint8_t) BSP_KEY_K3] != 0U) {
        pressedMask |= KEY_MASK_K3;
    }

    return pressedMask;
}

static void show_latched_key(uint8_t keyMask)
{
    BSP_KeyLeds_AllOff();

    if (keyMask == KEY_MASK_K1) {
        BSP_KeyLeds_Set(BSP_KEY_LED_K1, 1U);
    } else if (keyMask == KEY_MASK_K2) {
        BSP_KeyLeds_Set(BSP_KEY_LED_K2, 1U);
    } else if (keyMask == KEY_MASK_K3) {
        BSP_KeyLeds_Set(BSP_KEY_LED_K3, 1U);
    } else {
        /* No LED for idle or ambiguous multi-key reads. */
    }
}

static uint8_t is_single_key_mask(uint8_t keyMask)
{
    return ((keyMask == KEY_MASK_K1) || (keyMask == KEY_MASK_K2) ||
            (keyMask == KEY_MASK_K3))
        ? 1U
        : 0U;
}

static void service_key_leds(void)
{
    uint8_t pressedMask = read_key_mask();

    if (gLatchedKeyMask == 0U) {
        if (is_single_key_mask(pressedMask) != 0U) {
            gLatchedKeyMask = pressedMask;
        }
    } else if (pressedMask == 0U) {
        gLatchedKeyMask = 0U;
    } else {
        /* Keep the first accepted key until all keys are released. */
    }

    show_latched_key(gLatchedKeyMask);
}

static void service_run_led(void)
{
    static uint32_t blinkTicks;

    blinkTicks++;
    if (blinkTicks < RUN_LED_BLINK_TICKS) {
        return;
    }

    blinkTicks = 0U;
    DL_GPIO_togglePins(RUN_LED_PORT, RUN_LED_LED0_PIN);
}

static void init_runtime_peripherals(void)
{
#if (MAIN_ENABLE_BLUETOOTH_STARTUP != 0U)
    Bluetooth_init();
    Bluetooth_sendString("BOOT BT OK\r\n");
#endif

#if (MAIN_ENABLE_VISION_STARTUP != 0U)
    VisionComm_Init();
#endif

#if (MAIN_ENABLE_STEPPER_STARTUP != 0U)
    BSP_Stepper_Init();
#endif

#if (MAIN_ENABLE_OLED_STARTUP != 0U)
    BSP_OLED_Init();
    BSP_OLED_ShowString(1U, 1U, "TI RED");
#endif

#if (MAIN_ENABLE_JY61P != 0U)
    (void) BSP_JY61P_Init();
#endif

#if (MAIN_ENABLE_DCMOTOR_STARTUP != 0U)
    BSP_Motor_Init();
    BSP_Motor_Stop();
#endif
}

int main(void)
{
    SYSCFG_DL_init();
    start_1ms_tick();
    BSP_Keys_Init();
    BSP_KeyLeds_Init();
    BSP_KeyLeds_AllOff();
    DL_GPIO_clearPins(RUN_LED_PORT, RUN_LED_LED0_PIN);
    init_runtime_peripherals();

    while (1) {
        service_key_leds();
        service_run_led();
#if (MAIN_ENABLE_VISION_STARTUP != 0U)
        VisionComm_Poll();
#endif
#if (MAIN_ENABLE_JY61P != 0U)
        BSP_JY61P_Update();
#endif
        delay_cycles(MAIN_LOOP_DELAY_CYCLES);
    }
}

void SysTick_Handler(void)
{
    gMainMs++;

#if (MAIN_ENABLE_VISION_STARTUP != 0U)
    VisionComm_Tick1ms();
#endif
#if (MAIN_ENABLE_JY61P != 0U)
    BSP_JY61P_Tick1ms();
#endif
}
