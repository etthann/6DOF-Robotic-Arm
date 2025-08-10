#pragma once

#include "i2c.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

class pca9685
{
private:
    uint8_t address;
    I2C_HandleTypeDef *hi2c;
    void setPrescaler(int val);

public:
    pca9685(I2C_HandleTypeDef *hi2c, uint8_t address);
    void init();
    void setPWM(int joint, float deg);
};
