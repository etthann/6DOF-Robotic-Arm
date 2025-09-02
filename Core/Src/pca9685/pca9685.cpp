#include "pca9685/pca9685.h"
#include <cstdint>
#include "cmsis_os2.h"
#include "i2c.h"
#include "pca9685/pca9685_defs.h"
#include "pca9685/pca9685_utils.h"
#include <iostream>

pca9685::pca9685(I2C_HandleTypeDef *hi2c, uint8_t address, int freq)
{
    this->address = address;
    this->hi2c = hi2c;
    this->freq = freq;
}

void pca9685::init()
{
    uint8_t mode1;

    // Read MODE1
    HAL_I2C_Mem_Read(this->hi2c, this->address, PCA9685_REG::MODE1,
                     I2C_MEMADD_SIZE_8BIT, &mode1, 1, HAL_MAX_DELAY);

    // Set prescaler
    this->setPrescaler(this->freq);

    // MODE2: totem pole
    uint8_t mode2 = PCA9685_REG::OUTDRV;
    HAL_I2C_Mem_Write(this->hi2c, this->address, PCA9685_REG::MODE2,
                      I2C_MEMADD_SIZE_8BIT, &mode2, 1, HAL_MAX_DELAY);

    // Enable auto-increment + restart
    uint8_t restart = mode1 | PCA9685_REG::RESTART | PCA9685_REG::AUTO_INCREMENT;
    HAL_I2C_Mem_Write(this->hi2c, this->address, PCA9685_REG::MODE1,
                      I2C_MEMADD_SIZE_8BIT, &restart, 1, HAL_MAX_DELAY);
}

void pca9685::setPrescaler(int updateFreq)
{
    // Calculate prescaler
    float prescaleVal = 25000000.0f / (updateFreq * PCA9685_REG::PWM_STEPS) - 1.0f;
    uint8_t prescale = static_cast<uint8_t>(prescaleVal + 0.5f);

    // Read current MODE1
    uint8_t oldMode;
    HAL_I2C_Mem_Read(this->hi2c, this->address, PCA9685_REG::MODE1,
                     I2C_MEMADD_SIZE_8BIT, &oldMode, 1, HAL_MAX_DELAY);

    // Enter sleep (clear RESTART, set SLEEP)
    uint8_t sleepMode = (oldMode & ~PCA9685_REG::RESTART) | PCA9685_REG::SLEEP;
    HAL_I2C_Mem_Write(this->hi2c, this->address, PCA9685_REG::MODE1,
                      I2C_MEMADD_SIZE_8BIT, &sleepMode, 1, HAL_MAX_DELAY);

    // Write prescaler while in sleep
    HAL_I2C_Mem_Write(this->hi2c, this->address, PCA9685_REG::PRESCALE,
                      I2C_MEMADD_SIZE_8BIT, &prescale, 1, HAL_MAX_DELAY);

    // Wake up
    HAL_I2C_Mem_Write(this->hi2c, this->address, PCA9685_REG::MODE1,
                      I2C_MEMADD_SIZE_8BIT, &oldMode, 1, HAL_MAX_DELAY);

    // Wait for oscillator
    osDelay(1); // 1 ms is safe
}

void pca9685::setPWM(int joint, float angleDeg)
{
    uint16_t counts = degToCounts(angleDeg, 1000.0f, 2000.0f, this->freq);

    uint8_t reg = PCA9685_REG::LED0_ON_L + 4 * joint;
    uint8_t buffer[4] = {0x00, 0x00, (uint8_t)(counts & 0xFF), (uint8_t)((counts >> 8) & 0xFF)};
    HAL_I2C_Mem_Write(this->hi2c, this->address, reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, HAL_MAX_DELAY);
}
