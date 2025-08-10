
#include <cstdint>

namespace PCA9685_REG
{
    constexpr uint8_t ADDRESS = 0x40 << 1;
    constexpr uint8_t MODE_1_REGISTER = 0x00;
    constexpr uint8_t MODE_2_REGISTER = 0x01;
    constexpr uint8_t PRESCALER_ADDRESS = 0xFE;
    constexpr uint8_t RESTART_ENABLED = 0x80;
    constexpr uint8_t RESTART_DISABLED = 0x7F;
    constexpr uint8_t SLEEP_ENABLED = 0x10;
    constexpr uint8_t SLEEP_DISABLED = 0xEF;
    constexpr uint8_t AUTO_INCREMENT = 0x20;
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