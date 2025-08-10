#include "pca9685/pca9685_wrapper.h"
#include "pca9685/pca9685.h"
#include "pca9685/pca9685_defs.h"
#include "i2c.h"

// Create ONE global instance here
static pca9685 pca9685Sensor(&hi2c1, PCA9685_REG::ADDRESS);

extern "C"
{

    void PCA9685_Init(void)
    {
        pca9685Sensor.init();
    }

    void PCA9685_SetServoAngle(uint8_t channel, float deg = 90.0f)
    {
        pca9685Sensor.setPWM(channel, deg);
    }

} // extern "C"
