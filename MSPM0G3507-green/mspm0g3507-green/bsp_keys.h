/*
 * Board key GPIO BSP for MSPM0G3507.
 */
#ifndef BSP_KEYS_H
#define BSP_KEYS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    BSP_KEY_K1 = 0,
    BSP_KEY_K2,
    BSP_KEY_K3,
    BSP_KEY_COUNT
} BSP_KeyId_t;

typedef struct {
    uint8_t pressed[BSP_KEY_COUNT];
} BSP_KeysRaw_t;

void BSP_Keys_Init(void);
uint8_t BSP_Keys_IsPressed(BSP_KeyId_t key);
void BSP_Keys_GetRaw(BSP_KeysRaw_t *raw);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEYS_H */
