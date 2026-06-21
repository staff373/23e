/*
 * Green gimbal motion safety layer.
 */
#ifndef APP_GREEN_MOTION_H
#define APP_GREEN_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../bsp_stepper/bsp_stepper.h"

#include <stddef.h>
#include <stdint.h>

#define GREEN_MOTION_AXIS_X (STEPPER_AXIS_X)
#define GREEN_MOTION_AXIS_Y (STEPPER_AXIS_Y)

typedef struct {
    uint8_t initialized;
    BSP_StepperStatus_t x;
    BSP_StepperStatus_t y;
    int16_t last_error_x;
    int16_t last_error_y;
    int32_t last_steps_x;
    int32_t last_steps_y;
    uint32_t safe_stop_count;
    uint32_t move_command_count;
    uint32_t busy_skip_count;
} GreenMotionStatus_t;

typedef enum {
    GREEN_MOTION_PROFILE_BASE = 0,
    GREEN_MOTION_PROFILE_Q2_FIRST,
    GREEN_MOTION_PROFILE_Q2_FOLLOW
} GreenMotion_Profile_t;

void GreenMotion_Init(void);
void GreenMotion_Tick1ms(void);
void GreenMotion_Poll(void);
void GreenMotion_StopSafe(void);
uint8_t GreenMotion_JogAxis(uint8_t axis, int32_t steps);
uint8_t GreenMotion_ApplyTrackError(int16_t err_x, int16_t err_y);
void GreenMotion_SetProfile(GreenMotion_Profile_t profile);
GreenMotion_Profile_t GreenMotion_GetProfile(void);
const char *GreenMotion_GetProfileName(void);
void GreenMotion_ResetParams(void);
uint8_t GreenMotion_SetParam(const char *name, int32_t value);
uint8_t GreenMotion_GetParam(const char *name, int32_t *value);
int GreenMotion_FormatParams(char *buffer, size_t buffer_size);
int GreenMotion_FormatStatus(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* APP_GREEN_MOTION_H */
