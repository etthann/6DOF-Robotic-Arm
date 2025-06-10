#include "i2c.h"
#include <cstdint>
#include "main.h"      
#include "stm32f4xx_hal.h" 

class pca9685
{
private:
    uint8_t address;
    I2C_HandleTypeDef* hi2c;

public:
    pca9685(I2C_HandleTypeDef *hi2c, uint8_t address);
    void pca9685::setPrescaler(int val);
    void pca9685::init();
};

pca9685::pca9685(I2C_HandleTypeDef *hi2c, uint8_t address)
{
    this->address = address;
    this->hi2c = hi2c;
}
