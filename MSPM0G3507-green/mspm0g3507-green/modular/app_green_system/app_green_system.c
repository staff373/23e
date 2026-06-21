/*
 * Top-level green tracking system state machine.
 */
#include "app_green_system.h"

#include "../app_green_indicator/app_green_indicator.h"
#include "../app_green_motion/app_green_motion.h"
#include "../app_vision_comm/app_vision_comm.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define GREEN_SYSTEM_REASON_SIZE (16U)
#define GREEN_SYSTEM_MOTION_STATUS_SIZE (160U)
#define GREEN_SYSTEM_INDICATOR_STATUS_SIZE (48U)
#define GREEN_SYSTEM_PARAM_STATUS_SIZE (960U)
#define GREEN_SYSTEM_PREPARE_PHASE_MS (1U)
#define GREEN_SYSTEM_DEFAULT_CONTROL_PERIOD_MS (6U)
#define GREEN_SYSTEM_DEFAULT_FIRST_LOCK_LIMIT_MS (2000U)
#define GREEN_SYSTEM_DEFAULT_SUCCESS_ERR_PX (3)
#define GREEN_SYSTEM_DEFAULT_Q2_MIN_PT (2)
#define GREEN_SYSTEM_DEFAULT_Q2_TARGET_PT (3)
#define GREEN_SYSTEM_DEFAULT_Q2_MAX_PT (4)
#define GREEN_SYSTEM_INDICATOR_DIST_PT (8)
#define GREEN_SYSTEM_Q2_DEFAULT_DIR_X (-1)
#define GREEN_SYSTEM_Q2_DEFAULT_DIR_Y (0)
#define GREEN_SYSTEM_Q2_DIR_UPDATE_MIN_PT (2)
#define GREEN_SYSTEM_DEFAULT_Q2_FIRST_CONTROL_MS (8U)
#define GREEN_SYSTEM_DEFAULT_Q2_FOLLOW_CONTROL_MS (6U)

typedef enum {
    GREEN_PHASE_IDLE = 0,
    GREEN_PHASE_PREPARE,
    GREEN_PHASE_TRACKING,
    GREEN_PHASE_PAUSED,
    GREEN_PHASE_FAULT
} GreenSystem_Phase_t;

typedef enum {
    GREEN_QUALITY_IDLE = 0,
    GREEN_QUALITY_ACQUIRE_FIRST_LOCK,
    GREEN_QUALITY_TRACK_LOCKED,
    GREEN_QUALITY_TRACK_UNLOCKED,
    GREEN_QUALITY_REACQUIRE,
    GREEN_QUALITY_SCORE_FAILED
} GreenSystem_QualityState_t;

typedef enum {
    GREEN_Q2_PHASE_IDLE = 0,
    GREEN_Q2_PHASE_FIRST_ACQUIRE,
    GREEN_Q2_PHASE_FOLLOW,
    GREEN_Q2_PHASE_RED_LOST_GRACE,
    GREEN_Q2_PHASE_REACQUIRE_HOLD
} GreenSystem_Q2Phase_t;

typedef enum {
    GREEN_TASK_NONE = 0,
    GREEN_TASK_Q1,
    GREEN_TASK_Q2
} GreenSystem_Task_t;

typedef struct {
    uint8_t valid;
    uint8_t online;
    uint8_t stale;
    uint8_t track_valid;
    uint8_t red_valid;
    uint8_t green_valid;
    uint8_t dist_valid;
    uint32_t frame;
    uint32_t track_ms;
    uint32_t stale_ms;
    int16_t red_x;
    int16_t red_y;
    int16_t green_x;
    int16_t green_y;
    int16_t err_x;
    int16_t err_y;
    int16_t dist_mm;
} GreenSystem_VisionStatus_t;

typedef struct {
    uint32_t control_ms;
    uint32_t first_lock_ms;
    int32_t success_err_px;
    int32_t q2_min_pt;
    int32_t q2_target_pt;
    int32_t q2_max_pt;
    uint32_t q2_first_control_ms;
    uint32_t q2_follow_control_ms;
} GreenSystemParams_t;

static volatile uint32_t gGreenMs;
static GreenSystem_State_t gState;
static GreenSystem_Phase_t gPhase;
static GreenSystem_Task_t gTask;
static GreenSystem_CommandSource_t gLastSource;
static uint32_t gStateEnterMs;
static uint32_t gFailMs;
static uint32_t gRunStartMs;
static uint32_t gFirstLockMs;
static uint32_t gQualityEnterMs;
static GreenSystem_QualityState_t gQualityState;
static uint8_t gFirstLockOk;
static uint8_t gScoreFailed;
static char gReason[GREEN_SYSTEM_REASON_SIZE];
static GreenSystem_VisionStatus_t gVision;
static uint8_t gTargetLocked;
static int16_t gTargetX;
static int16_t gTargetY;
static uint32_t gTargetFrame;
static uint32_t gLastControlFrame;
static uint32_t gLastMotionCommandMs;
static GreenSystemParams_t gGreenParams;
static uint8_t gQ1FixedTargetValid;
static int16_t gQ1FixedTargetX;
static int16_t gQ1FixedTargetY;
static uint32_t gQ1FixedTargetFrame;
static GreenSystem_Q2Phase_t gQ2Phase;
static uint32_t gQ2PhaseEnterMs;
static uint8_t gQ2LastRedValid;
static int16_t gQ2LastRedX;
static int16_t gQ2LastRedY;
static int16_t gQ2LastDirX;
static int16_t gQ2LastDirY;
static uint32_t gQ2LastRedMs;
static uint32_t gQ2LastRedFrame;
static int32_t gQ1TargetDistPt;
static int32_t gQ1TargetDist2;
static int32_t gQ2RedGreenDistPt;
static int32_t gQ2RedGreenDist2;
static uint8_t gSuccessIndicatorTriggered;

static void GreenSystem_EnterState(
    GreenSystem_State_t next, const char *reason);
static uint32_t GreenSystem_StateElapsedMs(void);
static uint32_t GreenSystem_RunElapsedMs(void);
static void GreenSystem_UpdateVision(void);
static void GreenSystem_ClearVisionSnapshot(void);
static void GreenSystem_Q1Poll(void);
static void GreenSystem_Q1UpdateQuality(void);
static void GreenSystem_Q1CompleteFixedTarget(void);
static uint8_t GreenSystem_Q1FreshRedGreen(void);
static uint8_t GreenSystem_Q1FreshGreen(void);
static uint8_t GreenSystem_Q1InBandNow(void);
static uint8_t GreenSystem_Q1ErrorOkNow(void);
static void GreenSystem_Q2Poll(void);
static void GreenSystem_ResetRunRuntime(void);
static void GreenSystem_ResetQ2Runtime(void);
static void GreenSystem_Q2EnterPhase(
    GreenSystem_Q2Phase_t next, const char *reason);
static uint8_t GreenSystem_Q2FreshRedGreen(void);
static uint8_t GreenSystem_Q2HasLastRed(void);
static uint8_t GreenSystem_Q2SetTarget(
    int16_t red_x, int16_t red_y, int16_t green_x, int16_t green_y);
static void GreenSystem_Q2RememberRed(void);
static void GreenSystem_Q2UpdateQuality(void);
static uint8_t GreenSystem_Q2InBandNow(void);
static uint8_t GreenSystem_IndicatorInBandNow(void);
static void GreenSystem_ServiceSuccessIndicator(void);
static uint8_t GreenSystem_Q2IsLastRedTracking(void);
static uint32_t GreenSystem_Q2ControlPeriodMs(void);
static void GreenSystem_Q2ApplyProfile(void);
static uint8_t GreenSystem_Q2UseFirstProfile(void);
static int32_t GreenSystem_Isqrt(int32_t value);
static int16_t GreenSystem_ClampI16(
    int32_t value, int32_t min_value, int32_t max_value);
static void GreenSystem_ClearTarget(void);
static void GreenSystem_ResetQuality(void);
static void GreenSystem_EnterQuality(
    GreenSystem_QualityState_t next, const char *reason);
static uint8_t GreenSystem_IsSuccessNow(void);
static uint8_t GreenSystem_IsFailureNow(void);
static void GreenSystem_StartNewRun(
    GreenSystem_Task_t task, const char *reason);
static uint8_t GreenSystem_SetParamChecked(
    uint32_t *target, int32_t value, uint32_t min_value,
    uint32_t max_value);
static uint8_t GreenSystem_SetSignedParamChecked(
    int32_t *target, int32_t value, int32_t min_value,
    int32_t max_value);
static uint8_t GreenSystem_TokenEquals(
    const char *text, const char *expected);
static char GreenSystem_ToUpper(char value);
static const char *GreenSystem_StateName(GreenSystem_State_t state);
static const char *GreenSystem_PhaseName(GreenSystem_Phase_t phase);
static const char *GreenSystem_QualityName(
    GreenSystem_QualityState_t quality);
static const char *GreenSystem_Q2PhaseName(GreenSystem_Q2Phase_t phase);
static const char *GreenSystem_TaskName(GreenSystem_Task_t task);
static const char *GreenSystem_SourceName(GreenSystem_CommandSource_t source);
static void GreenSystem_CopyReason(const char *reason);

void GreenSystem_Init(void)
{
    memset(&gVision, 0, sizeof(gVision));

    gGreenMs = 0U;
    gTask = GREEN_TASK_NONE;
    gLastSource = GREEN_SYSTEM_SOURCE_NONE;
    gPhase = GREEN_PHASE_IDLE;
    gStateEnterMs = 0U;
    GreenSystem_ResetParams();
    GreenMotion_Init();
    GreenIndicator_Init();
    GreenSystem_ResetRunRuntime();
    GreenSystem_ResetQuality();
    GreenSystem_CopyReason("BOOT");
    GreenSystem_EnterState(GREEN_SYSTEM_STATE_IDLE, "INIT");
}

void GreenSystem_Tick1ms(void)
{
    gGreenMs++;
    GreenMotion_Tick1ms();
    GreenIndicator_Tick1ms();
}

void GreenSystem_Poll(void)
{
    GreenSystem_UpdateVision();
    GreenMotion_Poll();
    GreenIndicator_Poll();

    switch (gState) {
    case GREEN_SYSTEM_STATE_IDLE:
        break;

    case GREEN_SYSTEM_STATE_PREPARE:
        if (GreenSystem_StateElapsedMs() >= GREEN_SYSTEM_PREPARE_PHASE_MS) {
            if (VisionComm_SendModeTrack640()) {
                GreenSystem_EnterState(
                    GREEN_SYSTEM_STATE_RUN_ACTIVE,
                    (gTask == GREEN_TASK_Q1) ? "START1" : "START2");
            } else {
                GreenMotion_StopSafe();
                GreenSystem_EnterState(
                    GREEN_SYSTEM_STATE_FAULT, "VISION_TX");
            }
        }
        break;

    case GREEN_SYSTEM_STATE_RUN_ACTIVE:
        if (gTask == GREEN_TASK_Q1) {
            GreenSystem_Q1Poll();
        } else if (gTask == GREEN_TASK_Q2) {
            GreenSystem_Q2Poll();
        } else {
            GreenMotion_StopSafe();
            GreenSystem_EnterState(GREEN_SYSTEM_STATE_FAULT, "BAD_TASK");
        }
        break;

    case GREEN_SYSTEM_STATE_PAUSED:
        break;

    case GREEN_SYSTEM_STATE_FAULT:
        break;

    default:
        GreenMotion_StopSafe();
        GreenSystem_EnterState(GREEN_SYSTEM_STATE_FAULT, "BAD_STATE");
        break;
    }
}

void GreenSystem_Command(
    GreenSystem_Command_t cmd, GreenSystem_CommandSource_t source)
{
    gLastSource = source;

    switch (cmd) {
    case GREEN_CMD_START2:
        GreenSystem_StartNewRun(GREEN_TASK_Q2, "START2");
        break;

    case GREEN_CMD_START1:
        GreenSystem_StartNewRun(GREEN_TASK_Q1, "START1");
        break;

    case GREEN_CMD_PAUSE_TOGGLE:
        if (gState == GREEN_SYSTEM_STATE_RUN_ACTIVE) {
            GreenMotion_StopSafe();
            GreenIndicator_SetSuccess(0U);
            GreenSystem_EnterState(GREEN_SYSTEM_STATE_PAUSED, "PAUSE");
        } else if (gState == GREEN_SYSTEM_STATE_PAUSED) {
            GreenSystem_EnterState(GREEN_SYSTEM_STATE_RUN_ACTIVE, "RESUME");
        } else {
            /* Pause is meaningful only while running or paused. */
        }
        break;

    case GREEN_CMD_STOP_RESET:
        GreenMotion_StopSafe();
        (void) VisionComm_SendStop();
        GreenSystem_ResetRunRuntime();
        GreenSystem_ResetQuality();
        gTask = GREEN_TASK_NONE;
        GreenSystem_EnterState(GREEN_SYSTEM_STATE_IDLE, "STOP");
        break;

    case GREEN_CMD_STATUS:
        GreenSystem_UpdateVision();
        break;

    default:
        GreenSystem_EnterState(GREEN_SYSTEM_STATE_FAULT, "BAD_CMD");
        break;
    }
}

GreenSystem_State_t GreenSystem_GetState(void)
{
    return gState;
}

const char *GreenSystem_GetStateName(void)
{
    return GreenSystem_StateName(gState);
}

void GreenSystem_ResetParams(void)
{
    gGreenParams.control_ms = GREEN_SYSTEM_DEFAULT_CONTROL_PERIOD_MS;
    gGreenParams.first_lock_ms = GREEN_SYSTEM_DEFAULT_FIRST_LOCK_LIMIT_MS;
    gGreenParams.success_err_px = GREEN_SYSTEM_DEFAULT_SUCCESS_ERR_PX;
    gGreenParams.q2_min_pt = GREEN_SYSTEM_DEFAULT_Q2_MIN_PT;
    gGreenParams.q2_target_pt = GREEN_SYSTEM_DEFAULT_Q2_TARGET_PT;
    gGreenParams.q2_max_pt = GREEN_SYSTEM_DEFAULT_Q2_MAX_PT;
    gGreenParams.q2_first_control_ms =
        GREEN_SYSTEM_DEFAULT_Q2_FIRST_CONTROL_MS;
    gGreenParams.q2_follow_control_ms =
        GREEN_SYSTEM_DEFAULT_Q2_FOLLOW_CONTROL_MS;
    GreenMotion_ResetParams();
}

uint8_t GreenSystem_SetParam(const char *name, int32_t value)
{
    if (GreenSystem_TokenEquals(name, "control_ms")) {
        return GreenSystem_SetParamChecked(
            &gGreenParams.control_ms, value, 1U, 500U);
    }
    if (GreenSystem_TokenEquals(name, "first_ms")) {
        return GreenSystem_SetParamChecked(
            &gGreenParams.first_lock_ms, value, 200U, 5000U);
    }
    if (GreenSystem_TokenEquals(name, "err_px")) {
        return GreenSystem_SetSignedParamChecked(
            &gGreenParams.success_err_px, value, 2, 80);
    }
    if (GreenSystem_TokenEquals(name, "q2_min_pt")) {
        return GreenSystem_SetSignedParamChecked(
            &gGreenParams.q2_min_pt, value, 1, 80);
    }
    if (GreenSystem_TokenEquals(name, "q2_target_pt")) {
        return GreenSystem_SetSignedParamChecked(
            &gGreenParams.q2_target_pt, value, 1, 80);
    }
    if (GreenSystem_TokenEquals(name, "q2_max_pt")) {
        return GreenSystem_SetSignedParamChecked(
            &gGreenParams.q2_max_pt, value, 1, 120);
    }
    if (GreenSystem_TokenEquals(name, "q2_first_control_ms")) {
        return GreenSystem_SetParamChecked(
            &gGreenParams.q2_first_control_ms, value, 1U, 500U);
    }
    if (GreenSystem_TokenEquals(name, "q2_follow_control_ms")) {
        return GreenSystem_SetParamChecked(
            &gGreenParams.q2_follow_control_ms, value, 1U, 500U);
    }
    return GreenMotion_SetParam(name, value);
}

uint8_t GreenSystem_GetParam(const char *name, int32_t *value)
{
    if (value == NULL) {
        return 0U;
    }

    if (GreenSystem_TokenEquals(name, "control_ms")) {
        *value = (int32_t) gGreenParams.control_ms;
    } else if (GreenSystem_TokenEquals(name, "first_ms")) {
        *value = (int32_t) gGreenParams.first_lock_ms;
    } else if (GreenSystem_TokenEquals(name, "err_px")) {
        *value = gGreenParams.success_err_px;
    } else if (GreenSystem_TokenEquals(name, "q2_min_pt")) {
        *value = gGreenParams.q2_min_pt;
    } else if (GreenSystem_TokenEquals(name, "q2_target_pt")) {
        *value = gGreenParams.q2_target_pt;
    } else if (GreenSystem_TokenEquals(name, "q2_max_pt")) {
        *value = gGreenParams.q2_max_pt;
    } else if (GreenSystem_TokenEquals(name, "q2_first_control_ms")) {
        *value = (int32_t) gGreenParams.q2_first_control_ms;
    } else if (GreenSystem_TokenEquals(name, "q2_follow_control_ms")) {
        *value = (int32_t) gGreenParams.q2_follow_control_ms;
    } else {
        return GreenMotion_GetParam(name, value);
    }

    return 1U;
}

void GreenSystem_FormatParams(char *buffer, size_t buffer_size)
{
    char motionParams[GREEN_SYSTEM_PARAM_STATUS_SIZE];

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    motionParams[0] = '\0';
    GreenMotion_FormatParams(motionParams, sizeof(motionParams));

    (void) snprintf(buffer, buffer_size,
        "system control_ms=%lu first_ms=%lu err_px=%ld q2_min_pt=%ld q2_target_pt=%ld q2_max_pt=%ld q2_first_control_ms=%lu q2_follow_control_ms=%lu %s",
        (unsigned long) gGreenParams.control_ms,
        (unsigned long) gGreenParams.first_lock_ms,
        (long) gGreenParams.success_err_px,
        (long) gGreenParams.q2_min_pt,
        (long) gGreenParams.q2_target_pt,
        (long) gGreenParams.q2_max_pt,
        (unsigned long) gGreenParams.q2_first_control_ms,
        (unsigned long) gGreenParams.q2_follow_control_ms,
        motionParams);
}

void GreenSystem_FormatStatus(char *buffer, size_t buffer_size)
{
    char motionStatus[GREEN_SYSTEM_MOTION_STATUS_SIZE];
    char indicatorStatus[GREEN_SYSTEM_INDICATOR_STATUS_SIZE];
    uint8_t q2_active;
    uint8_t target_ok;
    uint8_t q2_in_band;
    uint32_t q2_ms;
    uint8_t last_red_valid;
    int16_t last_red_x;
    int16_t last_red_y;
    uint32_t last_red_frame;
    uint32_t last_red_ms;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    motionStatus[0] = '\0';
    indicatorStatus[0] = '\0';
    GreenMotion_FormatStatus(motionStatus, sizeof(motionStatus));
    GreenIndicator_FormatStatus(indicatorStatus, sizeof(indicatorStatus));

    q2_active = (gTask == GREEN_TASK_Q2) ? 1U : 0U;
    target_ok = GreenSystem_IsSuccessNow();
    q2_in_band = (q2_active != 0U) ? GreenSystem_Q2InBandNow() : 0U;
    q2_ms = ((q2_active != 0U) && (gQ2Phase != GREEN_Q2_PHASE_IDLE)) ?
        (uint32_t)(gGreenMs - gQ2PhaseEnterMs) : 0U;
    last_red_valid = (q2_active != 0U) ? gQ2LastRedValid : 0U;
    last_red_x = (q2_active != 0U) ? gQ2LastRedX : -1;
    last_red_y = (q2_active != 0U) ? gQ2LastRedY : -1;
    last_red_frame = (q2_active != 0U) ? gQ2LastRedFrame : 0U;
    last_red_ms = ((q2_active != 0U) && (gQ2LastRedValid != 0U)) ?
        (uint32_t)(gGreenMs - gQ2LastRedMs) : 0U;

    (void) snprintf(buffer, buffer_size,
        "state=%s phase=%s task=%s qstate=%s q2=%s q2_ms=%lu src=%s vision=%u online=%u stale=%u stale_ms=%lu frame=%lu track_ms=%lu target=%u target_xy=%ld,%ld target_frame=%lu red=%u green=%u red_xy=%ld,%ld green_xy=%ld,%ld last_red=%u last_red_xy=%ld,%ld last_red_frame=%lu last_red_ms=%lu err=%ld,%ld dist_valid=%u dist=%ld q1_target_dist_pt=%ld q2_rg_dist_pt=%ld target_ok=%u q2_in_band=%u success=%u failure=%u first_ok=%u run_ms=%lu first_ms=%lu fail=%lu score_failed=%u reason=%s indicator=%s motion=%s",
        GreenSystem_StateName(gState),
        GreenSystem_PhaseName(gPhase),
        GreenSystem_TaskName(gTask),
        GreenSystem_QualityName(gQualityState),
        (q2_active != 0U) ? GreenSystem_Q2PhaseName(gQ2Phase) : "NA",
        (unsigned long) q2_ms,
        GreenSystem_SourceName(gLastSource),
        (unsigned int) gVision.valid,
        (unsigned int) gVision.online,
        (unsigned int) gVision.stale,
        (unsigned long) gVision.stale_ms,
        (unsigned long) gVision.frame,
        (unsigned long) gVision.track_ms,
        (unsigned int) gTargetLocked,
        (long) gTargetX,
        (long) gTargetY,
        (unsigned long) gTargetFrame,
        (unsigned int) gVision.red_valid,
        (unsigned int) gVision.green_valid,
        (long) gVision.red_x,
        (long) gVision.red_y,
        (long) gVision.green_x,
        (long) gVision.green_y,
        (unsigned int) last_red_valid,
        (long) last_red_x,
        (long) last_red_y,
        (unsigned long) last_red_frame,
        (unsigned long) last_red_ms,
        (long) gVision.err_x,
        (long) gVision.err_y,
        (unsigned int) gVision.dist_valid,
        (long) gVision.dist_mm,
        (long) gQ1TargetDistPt,
        (long) gQ2RedGreenDistPt,
        (unsigned int) target_ok,
        (unsigned int) q2_in_band,
        (unsigned int) target_ok,
        (unsigned int) GreenSystem_IsFailureNow(),
        (unsigned int) gFirstLockOk,
        (unsigned long) GreenSystem_RunElapsedMs(),
        (unsigned long) gFirstLockMs,
        (unsigned long) gFailMs,
        (unsigned int) gScoreFailed,
        gReason,
        indicatorStatus,
        motionStatus);
}

static void GreenSystem_EnterState(
    GreenSystem_State_t next, const char *reason)
{
    GreenSystem_State_t previous = gState;

    gState = next;
    gStateEnterMs = gGreenMs;
    GreenSystem_CopyReason(reason);

    switch (next) {
    case GREEN_SYSTEM_STATE_IDLE:
        gPhase = GREEN_PHASE_IDLE;
        if (reason != NULL) {
            if (GreenSystem_TokenEquals(reason, "STOP") ||
                GreenSystem_TokenEquals(reason, "Q1_DONE") ||
                GreenSystem_TokenEquals(reason, "INIT")) {
                gTask = GREEN_TASK_NONE;
            }
        }
        GreenMotion_SetProfile(GREEN_MOTION_PROFILE_BASE);
        GreenIndicator_SetSuccess(0U);
        break;

    case GREEN_SYSTEM_STATE_PREPARE:
        gPhase = GREEN_PHASE_PREPARE;
        GreenMotion_SetProfile(GREEN_MOTION_PROFILE_BASE);
        GreenIndicator_SetSuccess(0U);
        break;

    case GREEN_SYSTEM_STATE_RUN_ACTIVE:
        gPhase = GREEN_PHASE_TRACKING;
        if ((previous != GREEN_SYSTEM_STATE_PAUSED) &&
            (gQualityState == GREEN_QUALITY_IDLE)) {
            gRunStartMs = gGreenMs;
            GreenSystem_EnterQuality(
                GREEN_QUALITY_ACQUIRE_FIRST_LOCK, "ACQUIRE");
        }
        if ((gTask == GREEN_TASK_Q2) &&
            (gQ2Phase == GREEN_Q2_PHASE_IDLE)) {
            GreenSystem_Q2EnterPhase(
                GREEN_Q2_PHASE_FIRST_ACQUIRE, "Q2_FIRST");
        }
        break;

    case GREEN_SYSTEM_STATE_PAUSED:
        gPhase = GREEN_PHASE_PAUSED;
        GreenIndicator_SetSuccess(0U);
        break;

    case GREEN_SYSTEM_STATE_FAULT:
        gPhase = GREEN_PHASE_FAULT;
        GreenMotion_SetProfile(GREEN_MOTION_PROFILE_BASE);
        GreenMotion_StopSafe();
        GreenIndicator_SetSuccess(0U);
        break;

    default:
        gPhase = GREEN_PHASE_FAULT;
        break;
    }
}

static uint32_t GreenSystem_StateElapsedMs(void)
{
    return (uint32_t) (gGreenMs - gStateEnterMs);
}

static uint32_t GreenSystem_RunElapsedMs(void)
{
    if ((gState != GREEN_SYSTEM_STATE_RUN_ACTIVE) &&
        (gState != GREEN_SYSTEM_STATE_PAUSED)) {
        return 0U;
    }

    return (uint32_t) (gGreenMs - gRunStartMs);
}

static void GreenSystem_UpdateVision(void)
{
    VisionCommStatus_t status;
    VisionTrack_t track;
    bool hasTrack;

    memset(&status, 0, sizeof(status));
    VisionComm_GetStatus(&status);

    memset(&track, 0, sizeof(track));
    hasTrack = VisionComm_GetLatestTrack(&track);

    gVision.online = status.online;
    gVision.stale = status.track_stale;
    gVision.stale_ms = (uint32_t) (status.ms - status.last_track_ms);
    gVision.valid = 0U;
    gVision.track_valid = 0U;
    gVision.red_valid = 0U;
    gVision.green_valid = 0U;
    gVision.dist_valid = 0U;
    gVision.frame = 0U;
    gVision.track_ms = 0U;
    gVision.red_x = -1;
    gVision.red_y = -1;
    gVision.green_x = -1;
    gVision.green_y = -1;
    gVision.err_x = 0;
    gVision.err_y = 0;
    gVision.dist_mm = -1;

    if (hasTrack) {
        gVision.track_valid = track.track_valid;
        gVision.red_valid = track.red_valid;
        gVision.green_valid = track.green_valid;
        gVision.frame = track.frame;
        gVision.track_ms = track.last_update_ms;
        gVision.red_x = track.red.x;
        gVision.red_y = track.red.y;
        gVision.green_x = track.green.x;
        gVision.green_y = track.green.y;

        if (track.dist_valid != 0U) {
            gVision.dist_valid = 1U;
            gVision.dist_mm = track.dist_mm;
        }

        gVision.valid =
            ((track.red_valid != 0U) && (track.green_valid != 0U)) ? 1U : 0U;
        gVision.err_x = track.err_x;
        gVision.err_y = track.err_y;
    }
}

static void GreenSystem_Q1Poll(void)
{
    uint8_t has_target = 0U;

    GreenMotion_SetProfile(GREEN_MOTION_PROFILE_BASE);

    if (gVision.stale != 0U) {
        GreenMotion_StopSafe();
        GreenSystem_Q1UpdateQuality();
        return;
    }

    if ((gQ1FixedTargetValid == 0U) &&
        (GreenSystem_Q1FreshRedGreen() != 0U)) {
        gQ1FixedTargetValid = 1U;
        gQ1FixedTargetX = gVision.red_x;
        gQ1FixedTargetY = gVision.red_y;
        gQ1FixedTargetFrame = gVision.frame;
        GreenSystem_CopyReason("Q1_FIXED");
    }

    if ((gQ1FixedTargetValid != 0U) &&
        (GreenSystem_Q1FreshGreen() != 0U)) {
        int32_t dx;
        int32_t dy;

        dx = (int32_t) gQ1FixedTargetX - gVision.green_x;
        dy = (int32_t) gQ1FixedTargetY - gVision.green_y;

        gQ1TargetDist2 = (dx * dx) + (dy * dy);
        gQ1TargetDistPt = GreenSystem_Isqrt(gQ1TargetDist2);
        gTargetLocked = 1U;
        gTargetX = gQ1FixedTargetX;
        gTargetY = gQ1FixedTargetY;
        gTargetFrame = gQ1FixedTargetFrame;
        gVision.err_x = (int16_t) dx;
        gVision.err_y = (int16_t) dy;
        has_target = 1U;
    } else {
        GreenMotion_StopSafe();
    }

    GreenSystem_Q1UpdateQuality();
    if (gState != GREEN_SYSTEM_STATE_RUN_ACTIVE) {
        return;
    }

    if ((has_target != 0U) && (GreenSystem_Q1InBandNow() == 0U) &&
        (gVision.frame != gLastControlFrame) &&
        ((uint32_t)(gGreenMs - gLastMotionCommandMs) >=
            gGreenParams.control_ms)) {
        gLastControlFrame = gVision.frame;
        gLastMotionCommandMs = gGreenMs;
        (void) GreenMotion_ApplyTrackError(gVision.err_x, gVision.err_y);
    }

}

static void GreenSystem_Q1UpdateQuality(void)
{
    uint8_t success;
    uint32_t elapsed;

    if (gState != GREEN_SYSTEM_STATE_RUN_ACTIVE) {
        GreenIndicator_SetSuccess(0U);
        return;
    }

    success = GreenSystem_Q1InBandNow();
    elapsed = GreenSystem_RunElapsedMs();

    switch (gQualityState) {
    case GREEN_QUALITY_ACQUIRE_FIRST_LOCK:
        if ((success != 0U) && (elapsed <= gGreenParams.first_lock_ms)) {
            GreenSystem_Q1CompleteFixedTarget();
            return;
        } else if (elapsed > gGreenParams.first_lock_ms) {
            gFailMs = elapsed - gGreenParams.first_lock_ms;
            GreenSystem_EnterQuality(
                GREEN_QUALITY_SCORE_FAILED, "Q1_TIMEOUT");
            gScoreFailed = 1U;
            GreenMotion_StopSafe();
            (void) VisionComm_SendStop();
            GreenSystem_ClearTarget();
            GreenSystem_EnterState(GREEN_SYSTEM_STATE_FAULT, "Q1_TIMEOUT");
        }
        break;

    case GREEN_QUALITY_TRACK_LOCKED:
    case GREEN_QUALITY_TRACK_UNLOCKED:
    case GREEN_QUALITY_REACQUIRE:
    case GREEN_QUALITY_SCORE_FAILED:
    case GREEN_QUALITY_IDLE:
    default:
        break;
    }

    GreenSystem_ServiceSuccessIndicator();
}

static void GreenSystem_Q1CompleteFixedTarget(void)
{
    if ((gFirstLockOk == 0U) &&
        (GreenSystem_RunElapsedMs() <= gGreenParams.first_lock_ms)) {
        gFirstLockOk = 1U;
        gFirstLockMs = GreenSystem_RunElapsedMs();
    }

    GreenSystem_EnterQuality(GREEN_QUALITY_TRACK_LOCKED, "Q1_LOCK");
    GreenMotion_StopSafe();
    (void) VisionComm_SendStop();
    GreenSystem_ClearTarget();
    GreenSystem_EnterState(GREEN_SYSTEM_STATE_IDLE, "Q1_DONE");
    if (gFirstLockOk != 0U) {
        GreenIndicator_SetSuccess(1U);
        gSuccessIndicatorTriggered = 1U;
    }
}

static uint8_t GreenSystem_Q1FreshRedGreen(void)
{
    return ((gVision.stale == 0U) && (gVision.red_valid != 0U) &&
        (gVision.green_valid != 0U) && (gVision.red_x >= 0) &&
        (gVision.red_y >= 0) && (gVision.green_x >= 0) &&
        (gVision.green_y >= 0)) ? 1U : 0U;
}

static uint8_t GreenSystem_Q1FreshGreen(void)
{
    return ((gVision.stale == 0U) && (gVision.green_valid != 0U) &&
        (gVision.green_x >= 0) && (gVision.green_y >= 0)) ? 1U : 0U;
}

static uint8_t GreenSystem_Q1InBandNow(void)
{
    if ((gQ1FixedTargetValid == 0U) ||
        (GreenSystem_Q1FreshGreen() == 0U)) {
        return 0U;
    }

    return GreenSystem_Q1ErrorOkNow();
}

static uint8_t GreenSystem_Q1ErrorOkNow(void)
{
    int32_t abs_x = (gVision.err_x >= 0) ? gVision.err_x : -gVision.err_x;
    int32_t abs_y = (gVision.err_y >= 0) ? gVision.err_y : -gVision.err_y;

    return ((abs_x <= gGreenParams.success_err_px) &&
        (abs_y <= gGreenParams.success_err_px)) ? 1U : 0U;
}

static void GreenSystem_Q2Poll(void)
{
    uint8_t has_target = 0U;

    if (gQ2Phase == GREEN_Q2_PHASE_IDLE) {
        GreenSystem_Q2EnterPhase(GREEN_Q2_PHASE_FIRST_ACQUIRE, "Q2_FIRST");
    }

    if (gVision.stale != 0U) {
        GreenMotion_StopSafe();
        GreenSystem_Q2EnterPhase(GREEN_Q2_PHASE_REACQUIRE_HOLD, "Q2_STALE");
        GreenSystem_Q2UpdateQuality();
        return;
    }

    if (GreenSystem_Q2FreshRedGreen() != 0U) {
        GreenSystem_Q2RememberRed();
        has_target = GreenSystem_Q2SetTarget(gVision.red_x, gVision.red_y,
            gVision.green_x, gVision.green_y);
        if ((gQ2Phase == GREEN_Q2_PHASE_RED_LOST_GRACE) ||
            (gQ2Phase == GREEN_Q2_PHASE_REACQUIRE_HOLD)) {
            GreenSystem_Q2EnterPhase(
                (gFirstLockOk != 0U) ? GREEN_Q2_PHASE_FOLLOW :
                                      GREEN_Q2_PHASE_FIRST_ACQUIRE,
                (gFirstLockOk != 0U) ? "Q2_FOLLOW" : "Q2_FIRST");
        }
    } else if ((gVision.green_valid != 0U) &&
        (GreenSystem_Q2HasLastRed() != 0U)) {
        has_target = GreenSystem_Q2SetTarget(gQ2LastRedX, gQ2LastRedY,
            gVision.green_x, gVision.green_y);
        if (gQ2Phase != GREEN_Q2_PHASE_RED_LOST_GRACE) {
            GreenSystem_Q2EnterPhase(
                GREEN_Q2_PHASE_RED_LOST_GRACE, "RED_LOST");
        }
    } else {
        GreenMotion_StopSafe();
        GreenSystem_Q2EnterPhase(
            GREEN_Q2_PHASE_REACQUIRE_HOLD, "Q2_WAIT");
    }

    if ((has_target != 0U) && (gVision.frame != gLastControlFrame) &&
        ((uint32_t)(gGreenMs - gLastMotionCommandMs) >=
            GreenSystem_Q2ControlPeriodMs())) {
        GreenSystem_Q2ApplyProfile();
        gLastControlFrame = gVision.frame;
        gLastMotionCommandMs = gGreenMs;
        (void) GreenMotion_ApplyTrackError(gVision.err_x, gVision.err_y);
    }

    GreenSystem_Q2UpdateQuality();
}

static void GreenSystem_ResetRunRuntime(void)
{
    GreenSystem_ClearTarget();
    GreenSystem_ResetQ2Runtime();
}

static void GreenSystem_ResetQ2Runtime(void)
{
    gQ2Phase = GREEN_Q2_PHASE_IDLE;
    gQ2PhaseEnterMs = 0U;
    gQ2LastRedValid = 0U;
    gQ2LastRedX = -1;
    gQ2LastRedY = -1;
    gQ2LastDirX = GREEN_SYSTEM_Q2_DEFAULT_DIR_X;
    gQ2LastDirY = GREEN_SYSTEM_Q2_DEFAULT_DIR_Y;
    gQ2LastRedMs = 0U;
    gQ2LastRedFrame = 0U;
    gQ2RedGreenDistPt = -1;
    gQ2RedGreenDist2 = 0;
    GreenMotion_SetProfile(GREEN_MOTION_PROFILE_BASE);
}

static void GreenSystem_Q2EnterPhase(
    GreenSystem_Q2Phase_t next, const char *reason)
{
    if (gQ2Phase == next) {
        return;
    }

    gQ2Phase = next;
    gQ2PhaseEnterMs = gGreenMs;
    GreenSystem_CopyReason(reason);
    GreenSystem_Q2ApplyProfile();
}

static uint8_t GreenSystem_Q2FreshRedGreen(void)
{
    return ((gVision.stale == 0U) && (gVision.red_valid != 0U) &&
        (gVision.green_valid != 0U) && (gVision.red_x >= 0) &&
        (gVision.red_y >= 0) && (gVision.green_x >= 0) &&
        (gVision.green_y >= 0)) ? 1U : 0U;
}

static uint8_t GreenSystem_Q2HasLastRed(void)
{
    if ((gQ2LastRedValid == 0U) || (gQ2LastRedX < 0) ||
        (gQ2LastRedY < 0)) {
        return 0U;
    }

    return 1U;
}

static uint8_t GreenSystem_Q2SetTarget(
    int16_t red_x, int16_t red_y, int16_t green_x, int16_t green_y)
{
    int32_t dx;
    int32_t dy;
    int32_t dist;
    int32_t dir_dist;
    int32_t target_x;
    int32_t target_y;

    if ((red_x < 0) || (red_y < 0) || (green_x < 0) || (green_y < 0)) {
        gVision.valid = 0U;
        return 0U;
    }

    dx = (int32_t) green_x - red_x;
    dy = (int32_t) green_y - red_y;
    gQ2RedGreenDist2 = (dx * dx) + (dy * dy);
    dist = GreenSystem_Isqrt(gQ2RedGreenDist2);
    gQ2RedGreenDistPt = dist;

    if ((gQ2LastDirX == 0) && (gQ2LastDirY == 0)) {
        gQ2LastDirX = GREEN_SYSTEM_Q2_DEFAULT_DIR_X;
        gQ2LastDirY = GREEN_SYSTEM_Q2_DEFAULT_DIR_Y;
    }

    dir_dist = GreenSystem_Isqrt(
        ((int32_t) gQ2LastDirX * gQ2LastDirX) +
        ((int32_t) gQ2LastDirY * gQ2LastDirY));
    if (dir_dist == 0) {
        target_x = (int32_t) red_x +
            (GREEN_SYSTEM_Q2_DEFAULT_DIR_X * gGreenParams.q2_target_pt);
        target_y = (int32_t) red_y +
            (GREEN_SYSTEM_Q2_DEFAULT_DIR_Y * gGreenParams.q2_target_pt);
    } else {
        target_x = (int32_t) red_x +
            (((int32_t) gQ2LastDirX * gGreenParams.q2_target_pt) /
                dir_dist);
        target_y = (int32_t) red_y +
            (((int32_t) gQ2LastDirY * gGreenParams.q2_target_pt) /
                dir_dist);
    }

    gTargetLocked = 1U;
    gTargetX = GreenSystem_ClampI16(target_x, 0, 32767);
    gTargetY = GreenSystem_ClampI16(target_y, 0, 32767);
    gTargetFrame = gVision.frame;
    gVision.valid = (gVision.green_valid != 0U) ? 1U : 0U;
    gVision.err_x = (int16_t)(gTargetX - green_x);
    gVision.err_y = (int16_t)(gTargetY - green_y);

    return gVision.valid;
}

static void GreenSystem_Q2RememberRed(void)
{
    int32_t dx;
    int32_t dy;
    int32_t move2;
    int32_t min_move2;

    if ((gQ2LastRedValid != 0U) && (gQ2LastRedX >= 0) &&
        (gQ2LastRedY >= 0)) {
        dx = (int32_t) gVision.red_x - gQ2LastRedX;
        dy = (int32_t) gVision.red_y - gQ2LastRedY;
        move2 = (dx * dx) + (dy * dy);
        min_move2 = GREEN_SYSTEM_Q2_DIR_UPDATE_MIN_PT *
            GREEN_SYSTEM_Q2_DIR_UPDATE_MIN_PT;
        if (move2 >= min_move2) {
            gQ2LastDirX = GreenSystem_ClampI16(-dx, -32768, 32767);
            gQ2LastDirY = GreenSystem_ClampI16(-dy, -32768, 32767);
        }
    }

    gQ2LastRedValid = 1U;
    gQ2LastRedX = gVision.red_x;
    gQ2LastRedY = gVision.red_y;
    gQ2LastRedMs = gGreenMs;
    gQ2LastRedFrame = gVision.frame;
}

static void GreenSystem_Q2UpdateQuality(void)
{
    uint8_t success;
    uint8_t protected_loss;

    if (gState != GREEN_SYSTEM_STATE_RUN_ACTIVE) {
        GreenIndicator_SetSuccess(0U);
        return;
    }

    success = GreenSystem_Q2InBandNow();
    protected_loss = GreenSystem_Q2IsLastRedTracking();

    switch (gQualityState) {
    case GREEN_QUALITY_ACQUIRE_FIRST_LOCK:
        if (success != 0U) {
            gFirstLockOk = 1U;
            gFirstLockMs = GreenSystem_RunElapsedMs();
            GreenSystem_EnterQuality(GREEN_QUALITY_TRACK_LOCKED, "Q2_LOCK");
            GreenSystem_Q2EnterPhase(GREEN_Q2_PHASE_FOLLOW, "Q2_FOLLOW");
        } else if (GreenSystem_RunElapsedMs() >
            gGreenParams.first_lock_ms) {
            gFailMs = GreenSystem_RunElapsedMs() -
                gGreenParams.first_lock_ms;
            GreenSystem_EnterQuality(
                GREEN_QUALITY_TRACK_UNLOCKED, "Q2_FIRST_TO");
        }
        break;

    case GREEN_QUALITY_TRACK_LOCKED:
        if (success == 0U) {
            GreenSystem_EnterQuality(
                (protected_loss != 0U) ? GREEN_QUALITY_REACQUIRE :
                                         GREEN_QUALITY_TRACK_UNLOCKED,
                (protected_loss != 0U) ? "Q2_RED_LOST" : "Q2_UNLOCK");
        }
        break;

    case GREEN_QUALITY_TRACK_UNLOCKED:
        if (success != 0U) {
            GreenSystem_EnterQuality(GREEN_QUALITY_TRACK_LOCKED, "Q2_RELOCK");
            GreenSystem_Q2EnterPhase(GREEN_Q2_PHASE_FOLLOW, "Q2_FOLLOW");
        } else if (protected_loss != 0U) {
            GreenSystem_EnterQuality(GREEN_QUALITY_REACQUIRE, "Q2_RED_LOST");
        } else {
            gFailMs = (uint32_t)(gGreenMs - gQualityEnterMs);
        }
        break;

    case GREEN_QUALITY_REACQUIRE:
        if (success != 0U) {
            GreenSystem_EnterQuality(GREEN_QUALITY_TRACK_LOCKED, "Q2_RELOCK");
            GreenSystem_Q2EnterPhase(GREEN_Q2_PHASE_FOLLOW, "Q2_FOLLOW");
        } else if (protected_loss == 0U) {
            GreenSystem_EnterQuality(
                GREEN_QUALITY_TRACK_UNLOCKED, "Q2_UNLOCK");
        }
        break;

    case GREEN_QUALITY_SCORE_FAILED:
    case GREEN_QUALITY_IDLE:
    default:
        break;
    }

    GreenSystem_ServiceSuccessIndicator();
}

static uint8_t GreenSystem_Q2InBandNow(void)
{
    if ((gVision.stale != 0U) || (GreenSystem_Q2FreshRedGreen() == 0U)) {
        return 0U;
    }

    return ((gQ2RedGreenDistPt >= gGreenParams.q2_min_pt) &&
        (gQ2RedGreenDistPt <= gGreenParams.q2_max_pt)) ? 1U : 0U;
}

static uint8_t GreenSystem_IndicatorInBandNow(void)
{
    if (gTask == GREEN_TASK_Q1) {
        if ((gQ1FixedTargetValid == 0U) ||
            (GreenSystem_Q1FreshGreen() == 0U) ||
            (gQ1TargetDistPt < 0)) {
            return 0U;
        }

        return (gQ1TargetDistPt <= GREEN_SYSTEM_INDICATOR_DIST_PT) ? 1U : 0U;
    }

    if (gTask == GREEN_TASK_Q2) {
        if ((gVision.stale != 0U) ||
            (GreenSystem_Q2FreshRedGreen() == 0U) ||
            (gQ2RedGreenDistPt < 0)) {
            return 0U;
        }

        return (gQ2RedGreenDistPt <= GREEN_SYSTEM_INDICATOR_DIST_PT) ? 1U : 0U;
    }

    return 0U;
}

static void GreenSystem_ServiceSuccessIndicator(void)
{
    if (gState != GREEN_SYSTEM_STATE_RUN_ACTIVE) {
        GreenIndicator_SetSuccess(0U);
        return;
    }

    if (gSuccessIndicatorTriggered == 0U) {
        if (GreenSystem_IndicatorInBandNow() != 0U) {
            GreenIndicator_SetSuccess(1U);
            gSuccessIndicatorTriggered = 1U;
        } else {
            GreenIndicator_SetSuccess(0U);
        }
    } else {
        GreenIndicator_Poll();
    }
}

static uint8_t GreenSystem_Q2IsLastRedTracking(void)
{
    return ((gVision.stale == 0U) && (gVision.green_valid != 0U) &&
        (gVision.red_valid == 0U) &&
        (GreenSystem_Q2HasLastRed() != 0U)) ? 1U : 0U;
}

static uint32_t GreenSystem_Q2ControlPeriodMs(void)
{
    if (GreenSystem_Q2UseFirstProfile() != 0U) {
        return gGreenParams.q2_first_control_ms;
    }

    return gGreenParams.q2_follow_control_ms;
}

static void GreenSystem_Q2ApplyProfile(void)
{
    if (GreenSystem_Q2UseFirstProfile() != 0U) {
        GreenMotion_SetProfile(GREEN_MOTION_PROFILE_Q2_FIRST);
    } else if ((gQ2Phase == GREEN_Q2_PHASE_FOLLOW) ||
        (gQ2Phase == GREEN_Q2_PHASE_RED_LOST_GRACE)) {
        GreenMotion_SetProfile(GREEN_MOTION_PROFILE_Q2_FOLLOW);
    } else {
        GreenMotion_SetProfile(GREEN_MOTION_PROFILE_BASE);
    }
}

static uint8_t GreenSystem_Q2UseFirstProfile(void)
{
    return ((gQ2Phase == GREEN_Q2_PHASE_FIRST_ACQUIRE) ||
        ((gQ2Phase == GREEN_Q2_PHASE_RED_LOST_GRACE) &&
            (gFirstLockOk == 0U))) ? 1U : 0U;
}

static int32_t GreenSystem_Isqrt(int32_t value)
{
    int32_t root = 0;

    if (value <= 0) {
        return 0;
    }

    while (((root + 1) * (root + 1)) <= value) {
        root++;
    }

    return root;
}

static int16_t GreenSystem_ClampI16(
    int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        value = min_value;
    }
    if (value > max_value) {
        value = max_value;
    }

    return (int16_t) value;
}

static void GreenSystem_ClearVisionSnapshot(void)
{
    memset(&gVision, 0, sizeof(gVision));
    gVision.stale = 1U;
    gVision.red_x = -1;
    gVision.red_y = -1;
    gVision.green_x = -1;
    gVision.green_y = -1;
    gVision.dist_mm = -1;
}

static void GreenSystem_ClearTarget(void)
{
    gTargetLocked = 0U;
    gTargetX = -1;
    gTargetY = -1;
    gTargetFrame = 0U;
    gQ1FixedTargetValid = 0U;
    gQ1FixedTargetX = -1;
    gQ1FixedTargetY = -1;
    gQ1FixedTargetFrame = 0U;
    gQ1TargetDistPt = -1;
    gQ1TargetDist2 = 0;
    gLastControlFrame = 0U;
    gLastMotionCommandMs = 0U;
}

static void GreenSystem_ResetQuality(void)
{
    gFailMs = 0U;
    gRunStartMs = 0U;
    gFirstLockMs = 0U;
    gQualityEnterMs = 0U;
    gQualityState = GREEN_QUALITY_IDLE;
    gFirstLockOk = 0U;
    gScoreFailed = 0U;
    gSuccessIndicatorTriggered = 0U;
    GreenIndicator_SetSuccess(0U);
}

static void GreenSystem_EnterQuality(
    GreenSystem_QualityState_t next, const char *reason)
{
    if (gQualityState == next) {
        return;
    }

    gQualityState = next;
    gQualityEnterMs = gGreenMs;
    if (next != GREEN_QUALITY_TRACK_UNLOCKED) {
        gFailMs = 0U;
    }
    GreenSystem_CopyReason(reason);
}

static uint8_t GreenSystem_IsSuccessNow(void)
{
    if (gTask == GREEN_TASK_Q1) {
        return GreenSystem_Q1InBandNow();
    }
    if (gTask == GREEN_TASK_Q2) {
        return GreenSystem_Q2InBandNow();
    }

    return 0U;
}

static uint8_t GreenSystem_IsFailureNow(void)
{
    if (gState != GREEN_SYSTEM_STATE_RUN_ACTIVE) {
        return 0U;
    }
    if (GreenSystem_IsSuccessNow() != 0U) {
        return 0U;
    }
    if ((gTask == GREEN_TASK_Q2) &&
        (GreenSystem_Q2IsLastRedTracking() != 0U)) {
        return 0U;
    }
    if ((gTask != GREEN_TASK_Q1) && (gTask != GREEN_TASK_Q2)) {
        return 0U;
    }

    return 1U;
}

static void GreenSystem_StartNewRun(
    GreenSystem_Task_t task, const char *reason)
{
    GreenMotion_StopSafe();
    GreenIndicator_SetSuccess(0U);
    VisionComm_ResetForNewRun();
    GreenSystem_ClearVisionSnapshot();
    GreenSystem_ResetRunRuntime();
    GreenSystem_ResetQuality();
    gTask = task;
    GreenSystem_EnterState(GREEN_SYSTEM_STATE_PREPARE, reason);
}

static uint8_t GreenSystem_SetParamChecked(
    uint32_t *target, int32_t value, uint32_t min_value,
    uint32_t max_value)
{
    if ((target == NULL) || (value < 0) ||
        ((uint32_t) value < min_value) ||
        ((uint32_t) value > max_value)) {
        return 0U;
    }

    *target = (uint32_t) value;
    return 1U;
}

static uint8_t GreenSystem_SetSignedParamChecked(
    int32_t *target, int32_t value, int32_t min_value,
    int32_t max_value)
{
    if ((target == NULL) || (value < min_value) || (value > max_value)) {
        return 0U;
    }

    *target = value;
    return 1U;
}

static uint8_t GreenSystem_TokenEquals(
    const char *text, const char *expected)
{
    if ((text == NULL) || (expected == NULL)) {
        return 0U;
    }

    while ((*text != '\0') && (*expected != '\0')) {
        if (GreenSystem_ToUpper(*text) != GreenSystem_ToUpper(*expected)) {
            return 0U;
        }
        text++;
        expected++;
    }

    return ((*text == '\0') && (*expected == '\0')) ? 1U : 0U;
}

static char GreenSystem_ToUpper(char value)
{
    if ((value >= 'a') && (value <= 'z')) {
        return (char)(value - ('a' - 'A'));
    }

    return value;
}

static const char *GreenSystem_QualityName(
    GreenSystem_QualityState_t quality)
{
    switch (quality) {
    case GREEN_QUALITY_IDLE:
        return "IDLE";
    case GREEN_QUALITY_ACQUIRE_FIRST_LOCK:
        return "ACQUIRE_FIRST_LOCK";
    case GREEN_QUALITY_TRACK_LOCKED:
        return "TRACK_LOCKED";
    case GREEN_QUALITY_TRACK_UNLOCKED:
        return "TRACK_UNLOCKED";
    case GREEN_QUALITY_REACQUIRE:
        return "REACQUIRE";
    case GREEN_QUALITY_SCORE_FAILED:
        return "SCORE_FAILED";
    default:
        return "UNKNOWN";
    }
}

static const char *GreenSystem_Q2PhaseName(GreenSystem_Q2Phase_t phase)
{
    switch (phase) {
    case GREEN_Q2_PHASE_IDLE:
        return "IDLE";
    case GREEN_Q2_PHASE_FIRST_ACQUIRE:
        return "FIRST_ACQUIRE";
    case GREEN_Q2_PHASE_FOLLOW:
        return "FOLLOW";
    case GREEN_Q2_PHASE_RED_LOST_GRACE:
        return "RED_LOST_GRACE";
    case GREEN_Q2_PHASE_REACQUIRE_HOLD:
        return "REACQUIRE_HOLD";
    default:
        return "UNKNOWN";
    }
}

static const char *GreenSystem_TaskName(GreenSystem_Task_t task)
{
    switch (task) {
    case GREEN_TASK_NONE:
        return "NONE";
    case GREEN_TASK_Q1:
        return "Q1";
    case GREEN_TASK_Q2:
        return "Q2";
    default:
        return "UNKNOWN";
    }
}

static const char *GreenSystem_StateName(GreenSystem_State_t state)
{
    switch (state) {
    case GREEN_SYSTEM_STATE_IDLE:
        return "IDLE";
    case GREEN_SYSTEM_STATE_PREPARE:
        return "PREPARE";
    case GREEN_SYSTEM_STATE_RUN_ACTIVE:
        return "RUN_ACTIVE";
    case GREEN_SYSTEM_STATE_PAUSED:
        return "PAUSED";
    case GREEN_SYSTEM_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static const char *GreenSystem_PhaseName(GreenSystem_Phase_t phase)
{
    switch (phase) {
    case GREEN_PHASE_IDLE:
        return "IDLE";
    case GREEN_PHASE_PREPARE:
        return "PREPARE";
    case GREEN_PHASE_TRACKING:
        return "TRACKING";
    case GREEN_PHASE_PAUSED:
        return "PAUSED";
    case GREEN_PHASE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

static const char *GreenSystem_SourceName(GreenSystem_CommandSource_t source)
{
    switch (source) {
    case GREEN_SYSTEM_SOURCE_NONE:
        return "NONE";
    case GREEN_SYSTEM_SOURCE_KEY:
        return "KEY";
    case GREEN_SYSTEM_SOURCE_BT:
        return "BT";
    default:
        return "UNKNOWN";
    }
}

static void GreenSystem_CopyReason(const char *reason)
{
    if (reason == NULL) {
        reason = "NONE";
    }

    (void) snprintf(gReason, sizeof(gReason), "%s", reason);
}
