/*
 * Green gimbal motion safety layer.
 */
#include "app_green_motion.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define GREEN_MOTION_HOLD_ENABLED (1U)
#define GREEN_MOTION_DEFAULT_DEADZONE_PX (1)
#define GREEN_MOTION_DEFAULT_PX_PER_STEP (4)
#define GREEN_MOTION_MIN_COMMAND_STEPS (1)
#define GREEN_MOTION_DEFAULT_MAX_COMMAND_STEPS (25)
#define GREEN_MOTION_DEFAULT_FINE_ERROR_PX (35)
#define GREEN_MOTION_DEFAULT_FINE_MAX_COMMAND_STEPS (1)
#define GREEN_MOTION_DEFAULT_Y_FINE_ERROR_PX (35)
#define GREEN_MOTION_DEFAULT_Y_FINE_MAX_COMMAND_STEPS (3)
#define GREEN_MOTION_DEFAULT_X_GAIN_PERCENT (150)
#define GREEN_MOTION_DEFAULT_Y_GAIN_PERCENT (100)
#define GREEN_MOTION_X_MAX_SPEED_SPS (4000.0f)
#define GREEN_MOTION_Y_MAX_SPEED_SPS (2000.0f)
#define GREEN_MOTION_X_ACCEL_SPS2 (8000.0f)
#define GREEN_MOTION_Y_ACCEL_SPS2 (4000.0f)
#define GREEN_MOTION_DEFAULT_X_TRACK_SIGN (-1)
#define GREEN_MOTION_DEFAULT_Y_TRACK_SIGN (-1)
#define GREEN_MOTION_Q2_FIRST_DEADZONE_PX (2)
#define GREEN_MOTION_Q2_FIRST_PX_PER_STEP (3)
#define GREEN_MOTION_Q2_FIRST_MAX_COMMAND_STEPS (12)
#define GREEN_MOTION_Q2_FIRST_FINE_ERROR_PX (15)
#define GREEN_MOTION_Q2_FIRST_FINE_MAX_COMMAND_STEPS (1)
#define GREEN_MOTION_Q2_FIRST_Y_FINE_ERROR_PX (15)
#define GREEN_MOTION_Q2_FIRST_Y_FINE_MAX_COMMAND_STEPS (1)
#define GREEN_MOTION_Q2_FIRST_X_GAIN_PERCENT (150)
#define GREEN_MOTION_Q2_FIRST_Y_GAIN_PERCENT (100)
#define GREEN_MOTION_Q2_FOLLOW_DEADZONE_PX (5)
#define GREEN_MOTION_Q2_FOLLOW_PX_PER_STEP (4)
#define GREEN_MOTION_Q2_FOLLOW_MAX_COMMAND_STEPS (10)
#define GREEN_MOTION_Q2_FOLLOW_FINE_ERROR_PX (22)
#define GREEN_MOTION_Q2_FOLLOW_FINE_MAX_COMMAND_STEPS (1)
#define GREEN_MOTION_Q2_FOLLOW_Y_FINE_ERROR_PX (22)
#define GREEN_MOTION_Q2_FOLLOW_Y_FINE_MAX_COMMAND_STEPS (1)
#define GREEN_MOTION_Q2_FOLLOW_X_GAIN_PERCENT (50)
#define GREEN_MOTION_Q2_FOLLOW_Y_GAIN_PERCENT (85)
#define GREEN_MOTION_Q2_BOOST_DEADZONE_PX (3)
#define GREEN_MOTION_Q2_BOOST_FINE_ERROR_PX (14)
#define GREEN_MOTION_Q2_BOOST_Y_FINE_ERROR_PX (14)
#define GREEN_MOTION_Q2_LEAK_MIN_PX (2)
#define GREEN_MOTION_Q2_LEAK_SUM_PX (18)

typedef struct {
    int32_t deadzone_px[3];
    int32_t px_per_step[3];
    int32_t max_step[3];
    int32_t fine_error_px[3];
    int32_t fine_max_step[3];
    int32_t y_fine_error_px[3];
    int32_t y_fine_max_step[3];
    int32_t xgain_percent[3];
    int32_t ygain_percent[3];
    int32_t q2_boost_deadzone_px;
    int32_t q2_boost_fine_error_px;
    int32_t q2_boost_y_fine_error_px;
    int32_t q2_leak_min_px;
    int32_t q2_leak_sum_px;
    int32_t xsign;
    int32_t ysign;
} GreenMotionParamSet_t;

static GreenMotionParamSet_t gGreenMotionParams;
static GreenMotionStatus_t gGreenMotionStatus;
static uint8_t gGreenMotionNextTrackAxis;
static GreenMotion_Profile_t gGreenMotionProfile;
static uint32_t gGreenMotionCornerBoostMs;
static int32_t gGreenMotionLeakX;
static int32_t gGreenMotionLeakY;

static void GreenMotion_UpdateStepperStatus(void);
static uint8_t GreenMotion_SetParamChecked(
    int32_t *target, int32_t value, int32_t min_value, int32_t max_value);
static int32_t GreenMotion_ErrorToSteps(uint8_t axis, int16_t error);
static int32_t GreenMotion_ApplyGain(int32_t steps, int32_t gain_percent);
static int32_t GreenMotion_ErrorToStepsWithLimit(
    uint8_t axis, int16_t error, int32_t max_steps);
static int32_t GreenMotion_ApplyDeadzoneLeak(
    uint8_t axis, int32_t value, int32_t abs_value, int32_t deadzone);
static int32_t *GreenMotion_LeakAccumulator(uint8_t axis);
static int32_t GreenMotion_AbsI32(int32_t value);
static int32_t GreenMotion_ClampI32(
    int32_t value, int32_t min_value, int32_t max_value);
static uint8_t GreenMotion_TokenEquals(
    const char *text, const char *expected);
static char GreenMotion_ToUpper(char value);
static int32_t GreenMotion_GetProfileParam(
    GreenMotion_Profile_t profile, const char *name);
static const char *GreenMotion_ProfileName(GreenMotion_Profile_t profile);
static uint8_t GreenMotion_ProfileIndex(
    GreenMotion_Profile_t profile, uint8_t *index);
static void GreenMotion_NormalizeProfile(GreenMotion_Profile_t profile);
static uint8_t GreenMotion_SetProfileParam(
    const char *name, GreenMotion_Profile_t profile, int32_t value);
static uint8_t GreenMotion_GetProfileParamByName(
    const char *name, GreenMotion_Profile_t profile, int32_t *value);
static uint8_t GreenMotion_TryMoveAxis(
    uint8_t axis, int32_t steps, float max_speed_sps, float accel_sps2);

void GreenMotion_Init(void)
{
    memset(&gGreenMotionStatus, 0, sizeof(gGreenMotionStatus));
    gGreenMotionNextTrackAxis = GREEN_MOTION_AXIS_X;
    gGreenMotionProfile = GREEN_MOTION_PROFILE_BASE;
    gGreenMotionCornerBoostMs = 0U;
    gGreenMotionLeakX = 0;
    gGreenMotionLeakY = 0;
    GreenMotion_ResetParams();

    BSP_Stepper_Init();
    BSP_Stepper_SetHoldEnabled(GREEN_MOTION_AXIS_X,
        GREEN_MOTION_HOLD_ENABLED);
    BSP_Stepper_SetHoldEnabled(GREEN_MOTION_AXIS_Y,
        GREEN_MOTION_HOLD_ENABLED);

    gGreenMotionStatus.initialized = 1U;
    GreenMotion_UpdateStepperStatus();
}

void GreenMotion_Tick1ms(void)
{
    BSP_Stepper_Tick1ms();
    if (gGreenMotionCornerBoostMs > 0U) {
        gGreenMotionCornerBoostMs--;
    }
}

void GreenMotion_Poll(void)
{
    if (gGreenMotionStatus.initialized == 0U) {
        return;
    }

    BSP_Stepper_Poll();
    GreenMotion_UpdateStepperStatus();
}

void GreenMotion_StopSafe(void)
{
    if (gGreenMotionStatus.initialized == 0U) {
        GreenMotion_Init();
    }

    BSP_Stepper_EmergencyStop(GREEN_MOTION_AXIS_X);
    BSP_Stepper_EmergencyStop(GREEN_MOTION_AXIS_Y);

    gGreenMotionStatus.last_steps_x = 0;
    gGreenMotionStatus.last_steps_y = 0;
    gGreenMotionNextTrackAxis = GREEN_MOTION_AXIS_X;
    gGreenMotionCornerBoostMs = 0U;
    gGreenMotionLeakX = 0;
    gGreenMotionLeakY = 0;
    gGreenMotionStatus.safe_stop_count++;
    GreenMotion_UpdateStepperStatus();
}

uint8_t GreenMotion_JogAxis(uint8_t axis, int32_t steps)
{
    uint8_t moved;

    if (gGreenMotionStatus.initialized == 0U) {
        GreenMotion_Init();
    }

    if ((axis != GREEN_MOTION_AXIS_X) && (axis != GREEN_MOTION_AXIS_Y)) {
        return 0U;
    }

    moved = GreenMotion_TryMoveAxis(axis, steps,
        (axis == GREEN_MOTION_AXIS_X) ? GREEN_MOTION_X_MAX_SPEED_SPS :
                                        GREEN_MOTION_Y_MAX_SPEED_SPS,
        (axis == GREEN_MOTION_AXIS_X) ? GREEN_MOTION_X_ACCEL_SPS2 :
                                        GREEN_MOTION_Y_ACCEL_SPS2);

    if (moved != 0U) {
        gGreenMotionStatus.last_error_x = 0;
        gGreenMotionStatus.last_error_y = 0;
        if (axis == GREEN_MOTION_AXIS_X) {
            gGreenMotionStatus.last_steps_x = steps;
            gGreenMotionStatus.last_steps_y = 0;
        } else {
            gGreenMotionStatus.last_steps_x = 0;
            gGreenMotionStatus.last_steps_y = steps;
        }
        gGreenMotionStatus.move_command_count++;
    }

    GreenMotion_UpdateStepperStatus();
    return moved;
}

uint8_t GreenMotion_ApplyTrackError(int16_t err_x, int16_t err_y)
{
    int32_t steps_x;
    int32_t steps_y;
    uint8_t moved = 0U;

    if (gGreenMotionStatus.initialized == 0U) {
        GreenMotion_Init();
    }

    GreenMotion_UpdateStepperStatus();

    gGreenMotionStatus.last_error_x = err_x;
    gGreenMotionStatus.last_error_y = err_y;
    gGreenMotionStatus.last_steps_x = 0;
    gGreenMotionStatus.last_steps_y = 0;

    steps_x = GreenMotion_ApplyGain(
        GreenMotion_ErrorToSteps(GREEN_MOTION_AXIS_X, err_x),
        GreenMotion_GetProfileParam(gGreenMotionProfile, "xgain")) *
        gGreenMotionParams.xsign;
    steps_y = GreenMotion_ApplyGain(
        GreenMotion_ErrorToSteps(GREEN_MOTION_AXIS_Y, err_y),
        GreenMotion_GetProfileParam(gGreenMotionProfile, "ygain")) *
        gGreenMotionParams.ysign;

    if ((steps_x == 0) && (steps_y == 0)) {
        GreenMotion_UpdateStepperStatus();
        return 0U;
    }

    if ((steps_x != 0) && (steps_y != 0)) {
        if (gGreenMotionNextTrackAxis == GREEN_MOTION_AXIS_X) {
            steps_y = 0;
            gGreenMotionNextTrackAxis = GREEN_MOTION_AXIS_Y;
        } else {
            steps_x = 0;
            gGreenMotionNextTrackAxis = GREEN_MOTION_AXIS_X;
        }
    } else if (steps_x != 0) {
        gGreenMotionNextTrackAxis = GREEN_MOTION_AXIS_Y;
    } else {
        gGreenMotionNextTrackAxis = GREEN_MOTION_AXIS_X;
    }

    if ((steps_x != 0) &&
        (GreenMotion_TryMoveAxis(GREEN_MOTION_AXIS_X, steps_x,
            GREEN_MOTION_X_MAX_SPEED_SPS,
            GREEN_MOTION_X_ACCEL_SPS2) != 0U)) {
        gGreenMotionStatus.last_steps_x = steps_x;
        moved = 1U;
    } else if ((steps_y != 0) &&
        (GreenMotion_TryMoveAxis(GREEN_MOTION_AXIS_Y, steps_y,
            GREEN_MOTION_Y_MAX_SPEED_SPS,
            GREEN_MOTION_Y_ACCEL_SPS2) != 0U)) {
        gGreenMotionStatus.last_steps_y = steps_y;
        moved = 1U;
    }

    if (moved != 0U) {
        gGreenMotionStatus.move_command_count++;
    }

    GreenMotion_UpdateStepperStatus();
    return moved;
}

void GreenMotion_SetProfile(GreenMotion_Profile_t profile)
{
    uint8_t index;
    GreenMotion_Profile_t old_profile = gGreenMotionProfile;

    if (GreenMotion_ProfileIndex(profile, &index) == 0U) {
        profile = GREEN_MOTION_PROFILE_BASE;
    }

    gGreenMotionProfile = profile;
    if (old_profile != profile) {
        gGreenMotionLeakX = 0;
        gGreenMotionLeakY = 0;
    }
}

GreenMotion_Profile_t GreenMotion_GetProfile(void)
{
    return gGreenMotionProfile;
}

const char *GreenMotion_GetProfileName(void)
{
    return GreenMotion_ProfileName(gGreenMotionProfile);
}

void GreenMotion_StartCornerBoost(uint32_t duration_ms)
{
    if (duration_ms > gGreenMotionCornerBoostMs) {
        gGreenMotionCornerBoostMs = duration_ms;
    }
}

void GreenMotion_ResetParams(void)
{
    gGreenMotionParams.deadzone_px[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_DEADZONE_PX;
    gGreenMotionParams.px_per_step[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_PX_PER_STEP;
    gGreenMotionParams.max_step[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_MAX_COMMAND_STEPS;
    gGreenMotionParams.fine_error_px[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_FINE_ERROR_PX;
    gGreenMotionParams.fine_max_step[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_FINE_MAX_COMMAND_STEPS;
    gGreenMotionParams.y_fine_error_px[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_Y_FINE_ERROR_PX;
    gGreenMotionParams.y_fine_max_step[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_Y_FINE_MAX_COMMAND_STEPS;
    gGreenMotionParams.xgain_percent[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_X_GAIN_PERCENT;
    gGreenMotionParams.ygain_percent[GREEN_MOTION_PROFILE_BASE] =
        GREEN_MOTION_DEFAULT_Y_GAIN_PERCENT;
    gGreenMotionParams.deadzone_px[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_DEADZONE_PX;
    gGreenMotionParams.px_per_step[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_PX_PER_STEP;
    gGreenMotionParams.max_step[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_MAX_COMMAND_STEPS;
    gGreenMotionParams.fine_error_px[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_FINE_ERROR_PX;
    gGreenMotionParams.fine_max_step[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_FINE_MAX_COMMAND_STEPS;
    gGreenMotionParams.y_fine_error_px[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_Y_FINE_ERROR_PX;
    gGreenMotionParams.y_fine_max_step[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_Y_FINE_MAX_COMMAND_STEPS;
    gGreenMotionParams.xgain_percent[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_X_GAIN_PERCENT;
    gGreenMotionParams.ygain_percent[GREEN_MOTION_PROFILE_Q2_FIRST] =
        GREEN_MOTION_Q2_FIRST_Y_GAIN_PERCENT;
    gGreenMotionParams.deadzone_px[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_DEADZONE_PX;
    gGreenMotionParams.px_per_step[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_PX_PER_STEP;
    gGreenMotionParams.max_step[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_MAX_COMMAND_STEPS;
    gGreenMotionParams.fine_error_px[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_FINE_ERROR_PX;
    gGreenMotionParams.fine_max_step[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_FINE_MAX_COMMAND_STEPS;
    gGreenMotionParams.y_fine_error_px[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_Y_FINE_ERROR_PX;
    gGreenMotionParams.y_fine_max_step[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_Y_FINE_MAX_COMMAND_STEPS;
    gGreenMotionParams.xgain_percent[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_X_GAIN_PERCENT;
    gGreenMotionParams.ygain_percent[GREEN_MOTION_PROFILE_Q2_FOLLOW] =
        GREEN_MOTION_Q2_FOLLOW_Y_GAIN_PERCENT;
    gGreenMotionParams.q2_boost_deadzone_px =
        GREEN_MOTION_Q2_BOOST_DEADZONE_PX;
    gGreenMotionParams.q2_boost_fine_error_px =
        GREEN_MOTION_Q2_BOOST_FINE_ERROR_PX;
    gGreenMotionParams.q2_boost_y_fine_error_px =
        GREEN_MOTION_Q2_BOOST_Y_FINE_ERROR_PX;
    gGreenMotionParams.q2_leak_min_px = GREEN_MOTION_Q2_LEAK_MIN_PX;
    gGreenMotionParams.q2_leak_sum_px = GREEN_MOTION_Q2_LEAK_SUM_PX;
    gGreenMotionParams.xsign = GREEN_MOTION_DEFAULT_X_TRACK_SIGN;
    gGreenMotionParams.ysign = GREEN_MOTION_DEFAULT_Y_TRACK_SIGN;
    gGreenMotionProfile = GREEN_MOTION_PROFILE_BASE;
    gGreenMotionCornerBoostMs = 0U;
    gGreenMotionLeakX = 0;
    gGreenMotionLeakY = 0;
}

uint8_t GreenMotion_SetParam(const char *name, int32_t value)
{
    if (GreenMotion_TokenEquals(name, "deadzone")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "pxstep")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "maxstep")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "fine_err")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "fine_step")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "yfine_err")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "yfine_step")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "xsign")) {
        return GreenMotion_SetParamChecked(&gGreenMotionParams.xsign,
            value, -1, 1);
    }
    if (GreenMotion_TokenEquals(name, "ysign")) {
        return GreenMotion_SetParamChecked(&gGreenMotionParams.ysign,
            value, -1, 1);
    }
    if (GreenMotion_TokenEquals(name, "xgain")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "ygain")) {
        return GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_BASE, value);
    }
    if (GreenMotion_TokenEquals(name, "q2boost_deadzone")) {
        return GreenMotion_SetParamChecked(
            &gGreenMotionParams.q2_boost_deadzone_px, value, 0, 80);
    }
    if (GreenMotion_TokenEquals(name, "q2boost_fine_err")) {
        return GreenMotion_SetParamChecked(
            &gGreenMotionParams.q2_boost_fine_error_px, value, 0, 240);
    }
    if (GreenMotion_TokenEquals(name, "q2boost_yfine_err")) {
        return GreenMotion_SetParamChecked(
            &gGreenMotionParams.q2_boost_y_fine_error_px, value, 0, 240);
    }
    if (GreenMotion_TokenEquals(name, "q2leak_min")) {
        return GreenMotion_SetParamChecked(
            &gGreenMotionParams.q2_leak_min_px, value, 0, 80);
    }
    if (GreenMotion_TokenEquals(name, "q2leak_sum")) {
        return GreenMotion_SetParamChecked(
            &gGreenMotionParams.q2_leak_sum_px, value, 1, 240);
    }
    if (GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_Q2_FIRST, value) != 0U) {
        return 1U;
    }
    if (GreenMotion_SetProfileParam(
            name, GREEN_MOTION_PROFILE_Q2_FOLLOW, value) != 0U) {
        return 1U;
    }

    return 0U;
}

uint8_t GreenMotion_GetParam(const char *name, int32_t *value)
{
    if (value == NULL) {
        return 0U;
    }

    if (GreenMotion_GetProfileParamByName(
            name, GREEN_MOTION_PROFILE_BASE, value) != 0U) {
        return 1U;
    } else if (GreenMotion_TokenEquals(name, "xsign")) {
        *value = gGreenMotionParams.xsign;
    } else if (GreenMotion_TokenEquals(name, "ysign")) {
        *value = gGreenMotionParams.ysign;
    } else if (GreenMotion_TokenEquals(name, "q2boost_deadzone")) {
        *value = gGreenMotionParams.q2_boost_deadzone_px;
    } else if (GreenMotion_TokenEquals(name, "q2boost_fine_err")) {
        *value = gGreenMotionParams.q2_boost_fine_error_px;
    } else if (GreenMotion_TokenEquals(name, "q2boost_yfine_err")) {
        *value = gGreenMotionParams.q2_boost_y_fine_error_px;
    } else if (GreenMotion_TokenEquals(name, "q2leak_min")) {
        *value = gGreenMotionParams.q2_leak_min_px;
    } else if (GreenMotion_TokenEquals(name, "q2leak_sum")) {
        *value = gGreenMotionParams.q2_leak_sum_px;
    } else if (GreenMotion_GetProfileParamByName(
            name, GREEN_MOTION_PROFILE_Q2_FIRST, value) != 0U) {
        return 1U;
    } else if (GreenMotion_GetProfileParamByName(
            name, GREEN_MOTION_PROFILE_Q2_FOLLOW, value) != 0U) {
        return 1U;
    } else {
        return 0U;
    }

    return 1U;
}

int GreenMotion_FormatParams(char *buffer, size_t buffer_size)
{
    if ((buffer == NULL) || (buffer_size == 0U)) {
        return 0;
    }

    return snprintf(buffer, buffer_size,
        "motion profile=%s deadzone=%ld pxstep=%ld maxstep=%ld fine_err=%ld fine_step=%ld yfine_err=%ld yfine_step=%ld xsign=%ld ysign=%ld xgain=%ld ygain=%ld q2first_deadzone=%ld q2first_pxstep=%ld q2first_maxstep=%ld q2first_fine_err=%ld q2first_fine_step=%ld q2first_yfine_err=%ld q2first_yfine_step=%ld q2first_xgain=%ld q2first_ygain=%ld q2follow_deadzone=%ld q2follow_pxstep=%ld q2follow_maxstep=%ld q2follow_fine_err=%ld q2follow_fine_step=%ld q2follow_yfine_err=%ld q2follow_yfine_step=%ld q2follow_xgain=%ld q2follow_ygain=%ld q2boost_deadzone=%ld q2boost_fine_err=%ld q2boost_yfine_err=%ld q2leak_min=%ld q2leak_sum=%ld",
        GreenMotion_ProfileName(gGreenMotionProfile),
        (long) gGreenMotionParams.deadzone_px[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.px_per_step[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.max_step[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.fine_error_px[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.fine_max_step[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.y_fine_error_px[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.y_fine_max_step[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.xsign,
        (long) gGreenMotionParams.ysign,
        (long) gGreenMotionParams.xgain_percent[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.ygain_percent[GREEN_MOTION_PROFILE_BASE],
        (long) gGreenMotionParams.deadzone_px[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.px_per_step[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.max_step[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.fine_error_px[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.fine_max_step[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.y_fine_error_px[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.y_fine_max_step[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.xgain_percent[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.ygain_percent[GREEN_MOTION_PROFILE_Q2_FIRST],
        (long) gGreenMotionParams.deadzone_px[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.px_per_step[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.max_step[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.fine_error_px[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.fine_max_step[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.y_fine_error_px[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.y_fine_max_step[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.xgain_percent[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.ygain_percent[GREEN_MOTION_PROFILE_Q2_FOLLOW],
        (long) gGreenMotionParams.q2_boost_deadzone_px,
        (long) gGreenMotionParams.q2_boost_fine_error_px,
        (long) gGreenMotionParams.q2_boost_y_fine_error_px,
        (long) gGreenMotionParams.q2_leak_min_px,
        (long) gGreenMotionParams.q2_leak_sum_px);
}

int GreenMotion_FormatStatus(char *buffer, size_t buffer_size)
{
    if ((buffer == (char *) 0) || (buffer_size == 0U)) {
        return 0;
    }

    GreenMotion_Poll();

    return snprintf(buffer, buffer_size,
        "MOTION profile=%s boost=%lu leak=%ld,%ld x=%s pos=%ld rem=%lu y=%s pos=%ld rem=%lu err=%d,%d step=%ld,%ld move=%lu busy=%lu stop=%lu",
        GreenMotion_ProfileName(gGreenMotionProfile),
        (unsigned long) gGreenMotionCornerBoostMs,
        (long) gGreenMotionLeakX,
        (long) gGreenMotionLeakY,
        BSP_Stepper_GetStateName(gGreenMotionStatus.x.state),
        (long) gGreenMotionStatus.x.position_steps,
        (unsigned long) gGreenMotionStatus.x.remaining_steps,
        BSP_Stepper_GetStateName(gGreenMotionStatus.y.state),
        (long) gGreenMotionStatus.y.position_steps,
        (unsigned long) gGreenMotionStatus.y.remaining_steps,
        (int) gGreenMotionStatus.last_error_x,
        (int) gGreenMotionStatus.last_error_y,
        (long) gGreenMotionStatus.last_steps_x,
        (long) gGreenMotionStatus.last_steps_y,
        (unsigned long) gGreenMotionStatus.move_command_count,
        (unsigned long) gGreenMotionStatus.busy_skip_count,
        (unsigned long) gGreenMotionStatus.safe_stop_count);
}

static void GreenMotion_UpdateStepperStatus(void)
{
    BSP_Stepper_GetStatus(GREEN_MOTION_AXIS_X, &gGreenMotionStatus.x);
    BSP_Stepper_GetStatus(GREEN_MOTION_AXIS_Y, &gGreenMotionStatus.y);
}

static uint8_t GreenMotion_SetParamChecked(
    int32_t *target, int32_t value, int32_t min_value, int32_t max_value)
{
    if ((target == NULL) || (value < min_value) || (value > max_value)) {
        return 0U;
    }

    if (((target == &gGreenMotionParams.xsign) ||
         (target == &gGreenMotionParams.ysign)) &&
        (value == 0)) {
        return 0U;
    }

    *target = value;

    return 1U;
}

static int32_t GreenMotion_ErrorToSteps(uint8_t axis, int16_t error)
{
    return GreenMotion_ErrorToStepsWithLimit(
        axis, error, GreenMotion_GetProfileParam(gGreenMotionProfile, "maxstep"));
}

static int32_t GreenMotion_ApplyGain(int32_t steps, int32_t gain_percent)
{
    int32_t value;
    int32_t abs_value;
    int32_t scaled;

    if (steps == 0) {
        return 0;
    }

    value = steps;
    abs_value = GreenMotion_AbsI32(value);
    scaled = ((abs_value * gain_percent) + 50) / 100;
    scaled = GreenMotion_ClampI32(scaled,
        (int32_t) GREEN_MOTION_MIN_COMMAND_STEPS,
        GreenMotion_GetProfileParam(gGreenMotionProfile, "maxstep") * 3);

    return (value >= 0) ? scaled : -scaled;
}

static int32_t GreenMotion_ErrorToStepsWithLimit(
    uint8_t axis, int16_t error, int32_t max_steps)
{
    int32_t value = (int32_t) error;
    int32_t abs_value = GreenMotion_AbsI32(value);
    int32_t deadzone;
    int32_t fine_error;
    int32_t steps;
    const char *fine_err_name =
        (axis == GREEN_MOTION_AXIS_Y) ? "yfine_err" : "fine_err";
    const char *fine_step_name =
        (axis == GREEN_MOTION_AXIS_Y) ? "yfine_step" : "fine_step";

    deadzone = GreenMotion_GetProfileParam(gGreenMotionProfile, "deadzone");
    fine_error = GreenMotion_GetProfileParam(gGreenMotionProfile,
        fine_err_name);

    if ((gGreenMotionProfile == GREEN_MOTION_PROFILE_Q2_FOLLOW) &&
        (gGreenMotionCornerBoostMs > 0U)) {
        deadzone = GreenMotion_ClampI32(deadzone, 0,
            gGreenMotionParams.q2_boost_deadzone_px);
        fine_error = GreenMotion_ClampI32(fine_error, 0,
            (axis == GREEN_MOTION_AXIS_Y) ?
                gGreenMotionParams.q2_boost_y_fine_error_px :
                gGreenMotionParams.q2_boost_fine_error_px);
    }

    if (abs_value <= deadzone) {
        if (gGreenMotionProfile == GREEN_MOTION_PROFILE_Q2_FOLLOW) {
            return GreenMotion_ApplyDeadzoneLeak(axis, value, abs_value,
                deadzone);
        }
        return 0;
    }

    *GreenMotion_LeakAccumulator(axis) = 0;

    steps = (abs_value +
        (GreenMotion_GetProfileParam(gGreenMotionProfile, "pxstep") - 1)) /
        GreenMotion_GetProfileParam(gGreenMotionProfile, "pxstep");
    steps = GreenMotion_ClampI32(steps,
        (int32_t) GREEN_MOTION_MIN_COMMAND_STEPS,
        max_steps);
    if ((abs_value <= fine_error) &&
        (steps >
            GreenMotion_GetProfileParam(gGreenMotionProfile, fine_step_name))) {
        steps = GreenMotion_GetProfileParam(gGreenMotionProfile,
            fine_step_name);
    }

    return (value >= 0) ? steps : -steps;
}

static int32_t GreenMotion_ApplyDeadzoneLeak(
    uint8_t axis, int32_t value, int32_t abs_value, int32_t deadzone)
{
    int32_t *accum = GreenMotion_LeakAccumulator(axis);
    int32_t leak_min = gGreenMotionParams.q2_leak_min_px;
    int32_t leak_sum = gGreenMotionParams.q2_leak_sum_px;

    if ((deadzone <= 0) || (leak_min <= 0) || (leak_sum <= 0) ||
        (abs_value < leak_min) || (value == 0)) {
        *accum = 0;
        return 0;
    }

    if (((*accum > 0) && (value < 0)) ||
        ((*accum < 0) && (value > 0))) {
        *accum = 0;
    }

    *accum += value;

    if (GreenMotion_AbsI32(*accum) >= leak_sum) {
        *accum = 0;
        return (value >= 0) ? 1 : -1;
    }

    return 0;
}

static int32_t *GreenMotion_LeakAccumulator(uint8_t axis)
{
    return (axis == GREEN_MOTION_AXIS_Y) ? &gGreenMotionLeakY :
                                           &gGreenMotionLeakX;
}

static int32_t GreenMotion_AbsI32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t GreenMotion_ClampI32(
    int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint8_t GreenMotion_TokenEquals(
    const char *text, const char *expected)
{
    if ((text == NULL) || (expected == NULL)) {
        return 0U;
    }

    while ((*text != '\0') && (*expected != '\0')) {
        if (GreenMotion_ToUpper(*text) != GreenMotion_ToUpper(*expected)) {
            return 0U;
        }
        text++;
        expected++;
    }

    return ((*text == '\0') && (*expected == '\0')) ? 1U : 0U;
}

static char GreenMotion_ToUpper(char value)
{
    if ((value >= 'a') && (value <= 'z')) {
        return (char)(value - ('a' - 'A'));
    }

    return value;
}

static int32_t GreenMotion_GetProfileParam(
    GreenMotion_Profile_t profile, const char *name)
{
    uint8_t index;

    if (GreenMotion_ProfileIndex(profile, &index) == 0U) {
        index = (uint8_t) GREEN_MOTION_PROFILE_BASE;
    }

    if (GreenMotion_TokenEquals(name, "deadzone")) {
        return gGreenMotionParams.deadzone_px[index];
    }
    if (GreenMotion_TokenEquals(name, "pxstep")) {
        return gGreenMotionParams.px_per_step[index];
    }
    if (GreenMotion_TokenEquals(name, "maxstep")) {
        return gGreenMotionParams.max_step[index];
    }
    if (GreenMotion_TokenEquals(name, "fine_err")) {
        return gGreenMotionParams.fine_error_px[index];
    }
    if (GreenMotion_TokenEquals(name, "fine_step")) {
        return gGreenMotionParams.fine_max_step[index];
    }
    if (GreenMotion_TokenEquals(name, "yfine_err")) {
        return gGreenMotionParams.y_fine_error_px[index];
    }
    if (GreenMotion_TokenEquals(name, "yfine_step")) {
        return gGreenMotionParams.y_fine_max_step[index];
    }
    if (GreenMotion_TokenEquals(name, "xgain")) {
        return gGreenMotionParams.xgain_percent[index];
    }
    if (GreenMotion_TokenEquals(name, "ygain")) {
        return gGreenMotionParams.ygain_percent[index];
    }

    return 0;
}

static const char *GreenMotion_ProfileName(GreenMotion_Profile_t profile)
{
    switch (profile) {
    case GREEN_MOTION_PROFILE_BASE:
        return "BASE";
    case GREEN_MOTION_PROFILE_Q2_FIRST:
        return "Q2_FIRST";
    case GREEN_MOTION_PROFILE_Q2_FOLLOW:
        return "Q2_FOLLOW";
    default:
        return "UNKNOWN";
    }
}

static uint8_t GreenMotion_ProfileIndex(
    GreenMotion_Profile_t profile, uint8_t *index)
{
    if (index == NULL) {
        return 0U;
    }

    switch (profile) {
    case GREEN_MOTION_PROFILE_BASE:
    case GREEN_MOTION_PROFILE_Q2_FIRST:
    case GREEN_MOTION_PROFILE_Q2_FOLLOW:
        *index = (uint8_t) profile;
        return 1U;
    default:
        return 0U;
    }
}

static void GreenMotion_NormalizeProfile(GreenMotion_Profile_t profile)
{
    uint8_t index;

    if (GreenMotion_ProfileIndex(profile, &index) == 0U) {
        return;
    }

    if (gGreenMotionParams.fine_max_step[index] >
        gGreenMotionParams.max_step[index]) {
        gGreenMotionParams.fine_max_step[index] =
            gGreenMotionParams.max_step[index];
    }
    if (gGreenMotionParams.y_fine_max_step[index] >
        gGreenMotionParams.max_step[index]) {
        gGreenMotionParams.y_fine_max_step[index] =
            gGreenMotionParams.max_step[index];
    }
}

static uint8_t GreenMotion_SetProfileParam(
    const char *name, GreenMotion_Profile_t profile, int32_t value)
{
    uint8_t index;
    uint8_t result = 0U;

    if (GreenMotion_ProfileIndex(profile, &index) == 0U) {
        return 0U;
    }

    if (profile == GREEN_MOTION_PROFILE_Q2_FIRST) {
        if (GreenMotion_TokenEquals(name, "q2first_deadzone")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.deadzone_px[index], value, 0, 80);
        } else if (GreenMotion_TokenEquals(name, "q2first_pxstep")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.px_per_step[index], value, 1, 120);
        } else if (GreenMotion_TokenEquals(name, "q2first_maxstep")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.max_step[index], value, 1, 50);
        } else if (GreenMotion_TokenEquals(name, "q2first_fine_err")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.fine_error_px[index], value, 0, 240);
        } else if (GreenMotion_TokenEquals(name, "q2first_fine_step")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.fine_max_step[index], value, 1, 20);
        } else if (GreenMotion_TokenEquals(name, "q2first_yfine_err")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.y_fine_error_px[index], value, 0, 240);
        } else if (GreenMotion_TokenEquals(name, "q2first_yfine_step")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.y_fine_max_step[index], value, 1, 20);
        } else if (GreenMotion_TokenEquals(name, "q2first_xgain")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.xgain_percent[index], value, 50, 300);
        } else if (GreenMotion_TokenEquals(name, "q2first_ygain")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.ygain_percent[index], value, 50, 300);
        }
    } else if (profile == GREEN_MOTION_PROFILE_Q2_FOLLOW) {
        if (GreenMotion_TokenEquals(name, "q2follow_deadzone")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.deadzone_px[index], value, 0, 80);
        } else if (GreenMotion_TokenEquals(name, "q2follow_pxstep")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.px_per_step[index], value, 1, 120);
        } else if (GreenMotion_TokenEquals(name, "q2follow_maxstep")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.max_step[index], value, 1, 50);
        } else if (GreenMotion_TokenEquals(name, "q2follow_fine_err")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.fine_error_px[index], value, 0, 240);
        } else if (GreenMotion_TokenEquals(name, "q2follow_fine_step")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.fine_max_step[index], value, 1, 20);
        } else if (GreenMotion_TokenEquals(name, "q2follow_yfine_err")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.y_fine_error_px[index], value, 0, 240);
        } else if (GreenMotion_TokenEquals(name, "q2follow_yfine_step")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.y_fine_max_step[index], value, 1, 20);
        } else if (GreenMotion_TokenEquals(name, "q2follow_xgain")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.xgain_percent[index], value, 50, 300);
        } else if (GreenMotion_TokenEquals(name, "q2follow_ygain")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.ygain_percent[index], value, 50, 300);
        }
    } else {
        if (GreenMotion_TokenEquals(name, "deadzone")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.deadzone_px[index], value, 0, 80);
        } else if (GreenMotion_TokenEquals(name, "pxstep")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.px_per_step[index], value, 1, 120);
        } else if (GreenMotion_TokenEquals(name, "maxstep")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.max_step[index], value, 1, 50);
        } else if (GreenMotion_TokenEquals(name, "fine_err")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.fine_error_px[index], value, 0, 240);
        } else if (GreenMotion_TokenEquals(name, "fine_step")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.fine_max_step[index], value, 1, 20);
        } else if (GreenMotion_TokenEquals(name, "yfine_err")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.y_fine_error_px[index], value, 0, 240);
        } else if (GreenMotion_TokenEquals(name, "yfine_step")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.y_fine_max_step[index], value, 1, 20);
        } else if (GreenMotion_TokenEquals(name, "xgain")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.xgain_percent[index], value, 50, 300);
        } else if (GreenMotion_TokenEquals(name, "ygain")) {
            result = GreenMotion_SetParamChecked(
                &gGreenMotionParams.ygain_percent[index], value, 50, 300);
        }
    }

    if (result != 0U) {
        GreenMotion_NormalizeProfile(profile);
    }

    return result;
}

static uint8_t GreenMotion_GetProfileParamByName(
    const char *name, GreenMotion_Profile_t profile, int32_t *value)
{
    uint8_t index;

    if ((value == NULL) ||
        (GreenMotion_ProfileIndex(profile, &index) == 0U)) {
        return 0U;
    }

    if (profile == GREEN_MOTION_PROFILE_Q2_FIRST) {
        if (GreenMotion_TokenEquals(name, "q2first_deadzone")) {
            *value = gGreenMotionParams.deadzone_px[index];
        } else if (GreenMotion_TokenEquals(name, "q2first_pxstep")) {
            *value = gGreenMotionParams.px_per_step[index];
        } else if (GreenMotion_TokenEquals(name, "q2first_maxstep")) {
            *value = gGreenMotionParams.max_step[index];
        } else if (GreenMotion_TokenEquals(name, "q2first_fine_err")) {
            *value = gGreenMotionParams.fine_error_px[index];
        } else if (GreenMotion_TokenEquals(name, "q2first_fine_step")) {
            *value = gGreenMotionParams.fine_max_step[index];
        } else if (GreenMotion_TokenEquals(name, "q2first_yfine_err")) {
            *value = gGreenMotionParams.y_fine_error_px[index];
        } else if (GreenMotion_TokenEquals(name, "q2first_yfine_step")) {
            *value = gGreenMotionParams.y_fine_max_step[index];
        } else if (GreenMotion_TokenEquals(name, "q2first_xgain")) {
            *value = gGreenMotionParams.xgain_percent[index];
        } else if (GreenMotion_TokenEquals(name, "q2first_ygain")) {
            *value = gGreenMotionParams.ygain_percent[index];
        } else {
            return 0U;
        }
    } else if (profile == GREEN_MOTION_PROFILE_Q2_FOLLOW) {
        if (GreenMotion_TokenEquals(name, "q2follow_deadzone")) {
            *value = gGreenMotionParams.deadzone_px[index];
        } else if (GreenMotion_TokenEquals(name, "q2follow_pxstep")) {
            *value = gGreenMotionParams.px_per_step[index];
        } else if (GreenMotion_TokenEquals(name, "q2follow_maxstep")) {
            *value = gGreenMotionParams.max_step[index];
        } else if (GreenMotion_TokenEquals(name, "q2follow_fine_err")) {
            *value = gGreenMotionParams.fine_error_px[index];
        } else if (GreenMotion_TokenEquals(name, "q2follow_fine_step")) {
            *value = gGreenMotionParams.fine_max_step[index];
        } else if (GreenMotion_TokenEquals(name, "q2follow_yfine_err")) {
            *value = gGreenMotionParams.y_fine_error_px[index];
        } else if (GreenMotion_TokenEquals(name, "q2follow_yfine_step")) {
            *value = gGreenMotionParams.y_fine_max_step[index];
        } else if (GreenMotion_TokenEquals(name, "q2follow_xgain")) {
            *value = gGreenMotionParams.xgain_percent[index];
        } else if (GreenMotion_TokenEquals(name, "q2follow_ygain")) {
            *value = gGreenMotionParams.ygain_percent[index];
        } else {
            return 0U;
        }
    } else {
        if (GreenMotion_TokenEquals(name, "deadzone")) {
            *value = gGreenMotionParams.deadzone_px[index];
        } else if (GreenMotion_TokenEquals(name, "pxstep")) {
            *value = gGreenMotionParams.px_per_step[index];
        } else if (GreenMotion_TokenEquals(name, "maxstep")) {
            *value = gGreenMotionParams.max_step[index];
        } else if (GreenMotion_TokenEquals(name, "fine_err")) {
            *value = gGreenMotionParams.fine_error_px[index];
        } else if (GreenMotion_TokenEquals(name, "fine_step")) {
            *value = gGreenMotionParams.fine_max_step[index];
        } else if (GreenMotion_TokenEquals(name, "yfine_err")) {
            *value = gGreenMotionParams.y_fine_error_px[index];
        } else if (GreenMotion_TokenEquals(name, "yfine_step")) {
            *value = gGreenMotionParams.y_fine_max_step[index];
        } else if (GreenMotion_TokenEquals(name, "xgain")) {
            *value = gGreenMotionParams.xgain_percent[index];
        } else if (GreenMotion_TokenEquals(name, "ygain")) {
            *value = gGreenMotionParams.ygain_percent[index];
        } else {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t GreenMotion_TryMoveAxis(
    uint8_t axis, int32_t steps, float max_speed_sps, float accel_sps2)
{
    uint8_t other_axis;

    if (steps == 0) {
        return 0U;
    }

    other_axis = (axis == GREEN_MOTION_AXIS_X) ?
        GREEN_MOTION_AXIS_Y : GREEN_MOTION_AXIS_X;
    BSP_Stepper_Poll();
    if (BSP_Stepper_IsBusy(other_axis) != 0U) {
        gGreenMotionStatus.busy_skip_count++;
        return 0U;
    }

    if (BSP_Stepper_IsBusy(axis) != 0U) {
        gGreenMotionStatus.busy_skip_count++;
        return 0U;
    }

    if (BSP_Stepper_IsReady(axis) == 0U) {
        return 0U;
    }

    return BSP_Stepper_MoveSteps(axis, steps, max_speed_sps, accel_sps2);
}
