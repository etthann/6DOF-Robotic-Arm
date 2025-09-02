
#include <stdint.h>

// Convert degrees (0–180) to pulse width in microseconds
float degToPulseWidth(float deg, float minUs, float maxUs);

// Convert pulse width (µs) to PCA9685 counts (0–4095)
uint16_t pulseWidthToCounts(float us, float pwmFreqHz);

// Convenience: degrees → counts in one step
uint16_t degToCounts(float deg, float minUs, float maxUs, float pwmFreqHz);
