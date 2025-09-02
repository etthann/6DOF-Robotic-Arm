#ifndef PCA9685_WRAPPER_H
#define PCA9685_WRAPPER_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif

    void PCA9685_Init();
    void PCA9685_SetServoAngle(uint8_t channel, float deg);

#ifdef __cplusplus
}
#endif
#endif
