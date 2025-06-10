#include "i2c.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

class pca9685
{
private:
    uint8_t address;
    I2C_HandleTypeDef *hi2c;
    void pca9685::setPrescaler(int val);

public:
    pca9685::pca9685(I2C_HandleTypeDef *hi2c, uint8_t address);
    void pca9685::pca9685_init();
};
