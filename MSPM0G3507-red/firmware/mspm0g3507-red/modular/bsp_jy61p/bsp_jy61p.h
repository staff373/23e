/*
 * JY61P UART attitude sensor BSP for MSPM0G3507.
 */
#ifndef BSP_JY61P_H
#define BSP_JY61P_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BSP_JY61P_OK (0)
#define BSP_JY61P_ERR_ARG (-1)
#define BSP_JY61P_ERR_UART_REINIT (-2)
#define BSP_JY61P_ERR_UART_TX (-3)
#define BSP_JY61P_ERR_TICK (-4)

typedef struct {
    float pitch_rad;
    float roll_rad;
    float yaw_rad;

    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;

    uint32_t last_frame_ms;
    uint32_t sample_seq;
    uint8_t initialized;
    uint8_t online;
    uint8_t data_valid;
} BSP_JY61P_Data_t;

int BSP_JY61P_Init(void);
void BSP_JY61P_Update(void);
void BSP_JY61P_Tick1ms(void);

void BSP_JY61P_GetData(BSP_JY61P_Data_t *data);
float BSP_JY61P_GetPitch(void);
float BSP_JY61P_GetRoll(void);
float BSP_JY61P_GetYaw(void);
uint8_t BSP_JY61P_IsOnline(void);
uint8_t BSP_JY61P_IsDataValid(void);
uint32_t BSP_JY61P_GetSampleSeq(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_JY61P_H */
