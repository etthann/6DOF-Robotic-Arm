#include "pca9685.h"
#include <cstdint>
#include "cmsis_os2.h"

void pca9685::init()
{

    uint8_t mode1Address = 0x00;
    uint8_t mode1;

    // retrieve mode1 config byte
    HAL_I2C_Mem_Read(this->hi2c, this->address, mode1Address, I2C_MEMADD_SIZE_8BIT, &mode1, 1, HAL_MAX_DELAY);

    // turn off restart mode and set to sleep mode
    uint8_t sleepMode = (mode1 & 0x7F) | 0x10;

    // set servo driver to sleep mode
    HAL_I2C_Mem_Write(this->hi2c, this->address, mode1Address, I2C_MEMADD_SIZE_8BIT, &sleepMode, 1, HAL_MAX_DELAY);

    // set prescalar
    this->setPrescaler();

    uint8_t wakeMode = mode1 & ~0x10;

    // set servo driver to wake mode
    HAL_I2C_Mem_Write(this->hi2c, this->address, mode1Address, I2C_MEMADD_SIZE_8BIT, &wakeMode, 1, HAL_MAX_DELAY);

    osDelay(1);

    // restart servo driver
    uint8_t restart = mode1 | 0x80;
    HAL_I2C_Mem_Write(this->hi2c, this->address, mode1Address, I2C_MEMADD_SIZE_8BIT, &restart, 1, HAL_MAX_DELAY);
}

void pca9685::setPrescaler(int pwmFreq = 50)
{
    int prescale_val = 25000000 / (pwmFreq * 4096) - 1;

    uint8_t prescale = static_cast<uint8_t>(prescale_val);

    uint8_t prescalerAddress = 0xFE;

    HAL_I2C_Mem_Write(this->hi2c, this->address, prescalerAddress, I2C_MEMADD_SIZE_8BIT, &prescale, 1, HAL_MAX_DELAY);
}