#include "pca9685/pca9685_utils.h"

float degToPulseWidth(float deg, float minUs, float maxUs)
{
    if (deg < 0.0f)
        deg = 0.0f;
    if (deg > 180.0f)
        deg = 180.0f;
    return minUs + (deg / 180.0f) * (maxUs - minUs);
}

uint16_t pulseWidthToCounts(float us, float pwmFreqHz)
{
    float frameUs = 1000000.0f / pwmFreqHz; // frame period in µs
    if (us < 500.0f)
        us = 500.0f;
    if (us > 2500.0f)
        us = 2500.0f;
    return (uint16_t)(us * (4096.0f / frameUs) + 0.5f);
}

uint16_t degToCounts(float deg, float minUs, float maxUs, float pwmFreqHz)
{
    return pulseWidthToCounts(degToPulseWidth(deg, minUs, maxUs), pwmFreqHz);
}
