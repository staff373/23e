#ifndef BSP_STEPPER_H
#define BSP_STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define STEPPER_AXIS_X (0U)
#define STEPPER_AXIS_Y (1U)
#define STEPPER_AXIS_COUNT (2U)

#ifndef STEPPER_X_MIN_SPEED_SPS
#define STEPPER_X_MIN_SPEED_SPS (50.0f)
#endif

#ifndef STEPPER_X_MAX_SPEED_SPS
#define STEPPER_X_MAX_SPEED_SPS (8000.0f)
#endif

#ifndef STEPPER_X_ACCEL_SPS2
#define STEPPER_X_ACCEL_SPS2 (4000.0f)
#endif

#ifndef STEPPER_Y_MIN_SPEED_SPS
#define STEPPER_Y_MIN_SPEED_SPS (50.0f)
#endif

#ifndef STEPPER_Y_MAX_SPEED_SPS
#define STEPPER_Y_MAX_SPEED_SPS (8000.0f)
#endif

#ifndef STEPPER_Y_ACCEL_SPS2
#define STEPPER_Y_ACCEL_SPS2 (4000.0f)
#endif

#ifndef STEPPER_SPEED_EPSILON_SPS
#define STEPPER_SPEED_EPSILON_SPS (0.1f)
#endif

typedef enum {
    STEPPER_STATE_DISABLED = 0,
    STEPPER_STATE_IDLE,
    STEPPER_STATE_MOVING,
    STEPPER_STATE_STOPPING,
    STEPPER_STATE_DONE,
    STEPPER_STATE_ERROR
} BSP_StepperState_t;

typedef struct {
    uint8_t hw_ready;
    uint8_t busy;
    uint8_t hold_enabled;
    BSP_StepperState_t state;
    int32_t position_steps;
    int32_t direction_sign;
    uint32_t total_steps;
    uint32_t remaining_steps;
    float current_speed_sps;
    float max_speed_sps;
    float accel_sps2;
} BSP_StepperStatus_t;

void BSP_Stepper_Init(void);
uint8_t BSP_Stepper_MoveSteps(
    uint8_t axis, int32_t steps, float max_speed_sps, float accel_sps2);
void BSP_Stepper_Stop(uint8_t axis);
void BSP_Stepper_EmergencyStop(uint8_t axis);
void BSP_Stepper_SetHoldEnabled(uint8_t axis, uint8_t enabled);
uint8_t BSP_Stepper_GetHoldEnabled(uint8_t axis);
uint8_t BSP_Stepper_IsBusy(uint8_t axis);
uint8_t BSP_Stepper_IsReady(uint8_t axis);
int32_t BSP_Stepper_GetPosition(uint8_t axis);
void BSP_Stepper_SetPosition(uint8_t axis, int32_t position_steps);
int32_t BSP_Stepper_GetRemaining(uint8_t axis);
BSP_StepperState_t BSP_Stepper_GetState(uint8_t axis);
void BSP_Stepper_GetStatus(uint8_t axis, BSP_StepperStatus_t *status);
const char *BSP_Stepper_GetStateName(BSP_StepperState_t state);

#ifdef __cplusplus
}
#endif

#endif /* BSP_STEPPER_H */
