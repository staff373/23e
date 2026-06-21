/*
 * Board motor binding for TB6612 on the MSPM0 main controller board.
 */
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Motor_Init(void);
void BSP_Motor_SetDuty(float left_duty, float right_duty);
void BSP_Motor_SetLeftDuty(float duty);
void BSP_Motor_SetRightDuty(float duty);
void BSP_Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
