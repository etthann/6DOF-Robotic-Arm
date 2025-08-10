#include "pca9685/pca9685.h"
#include <cstdint>
#include "cmsis_os2.h"
#include "i2c.h"
#include "pca9685/pca9685_defs.h"
#include "pca9685/pca9685_utils.h"

pca9685 pca9685Sensor(&hi2c1, PCA9685_REG::ADDRESS);
pwmChannel pwmValues[16];

pca9685::pca9685(I2C_HandleTypeDef *hi2c, uint8_t address)
{
    this->address = address;
    this->hi2c = hi2c;
}

void pca9685::init()
{

    uint8_t mode1Address = PCA9685_REG::MODE_1_REGISTER;
    uint8_t mode1;

    // retrieve mode1 config byte
    HAL_I2C_Mem_Read(this->hi2c, this->address, mode1Address, I2C_MEMADD_SIZE_8BIT, &mode1, 1, HAL_MAX_DELAY);

    // turn off restart mode and set to sleep mode
    uint8_t sleepMode = (mode1 & PCA9685_REG::RESTART_DISABLED) | PCA9685_REG::SLEEP_ENABLED;

    // set servo driver to sleep mode
    HAL_I2C_Mem_Write(this->hi2c, this->address, mode1Address, I2C_MEMADD_SIZE_8BIT, &sleepMode, 1, HAL_MAX_DELAY);

    osDelay(1);

    // set prescalar
    this->setPrescaler();

    uint8_t wakeMode = mode1 & ~PCA9685_REG::SLEEP_ENABLED;

    // set servo driver to wake mode
    HAL_I2C_Mem_Write(this->hi2c, this->address, mode1Address, I2C_MEMADD_SIZE_8BIT, &wakeMode, 1, HAL_MAX_DELAY);

    osDelay(1);

    uint8_t mode2 = 0x04;
    HAL_I2C_Mem_Write(this->hi2c, this->address, PCA9685_REG::MODE_2_REGISTER, I2C_MEMADD_SIZE_8BIT, &mode2, 1, HAL_MAX_DELAY);

    // restart servo driver and auto increment
    uint8_t restart = mode1 | PCA9685_REG::RESTART_ENABLED | PCA9685_REG::AUTO_INCREMENT;
    HAL_I2C_Mem_Write(this->hi2c, this->address, mode1Address, I2C_MEMADD_SIZE_8BIT, &restart, 1, HAL_MAX_DELAY);

    osDelay(1);
}

void pca9685::setPrescaler(int pwmFreq = 50)
{
    int pwmInternalOscill = 25000000.0f;

    // formula from datasheet
    int prescale_val = pwmInternalOscill / (pwmFreq * PCA9685_REG::PWM_STEPS) - 1;

    uint8_t prescale = static_cast<uint8_t>(prescale_val);

    uint8_t prescalerAddress = PCA9685_REG::PRESCALER_ADDRESS;

    HAL_I2C_Mem_Write(this->hi2c, this->address, prescalerAddress, I2C_MEMADD_SIZE_8BIT, &prescale, 1, HAL_MAX_DELAY);
}

void pca9685::setPWM(int joint, float angleDeg = 90.0f)
{
    uint16_t counts = degToCounts(angleDeg, 1000.0f, 2000.0f, 50.0f);

    uint8_t reg = PCA9685_REG::LED0_ON_L + 4 * joint;
    uint8_t buffer[4] = {0x00, 0x00, (uint8_t)(counts & 0xFF), (uint8_t)((counts >> 8) & 0x0F)};
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_REG::ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, HAL_MAX_DELAY);
}
