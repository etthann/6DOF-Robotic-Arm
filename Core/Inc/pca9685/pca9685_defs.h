
#include <cstdint>

namespace PCA9685_REG
{
    constexpr uint8_t ADDRESS = 0x40 << 1;
    constexpr uint8_t MODE1 = 0x00;
    constexpr uint8_t MODE2 = 0x01;
    constexpr uint8_t PRESCALE = 0xFE;

    // MODE1 bits
    constexpr uint8_t RESTART = 1 << 7; // 0x80
    constexpr uint8_t EXTCLK = 1 << 6;
    constexpr uint8_t AUTO_INCREMENT = 1 << 5; // 0x20
    constexpr uint8_t SLEEP = 1 << 4;          // 0x10

    // MODE2 bits
    constexpr uint8_t OUTDRV = 1 << 2; // totem pole

    // LED registers
    constexpr uint8_t LED0_ON_L = 0x06;

    constexpr int PWM_STEPS = 4096;
}

typedef struct
{
    uint8_t on_low;
    uint8_t on_high;
    uint8_t off_low;
    uint8_t off_high;

} pwmChannel;