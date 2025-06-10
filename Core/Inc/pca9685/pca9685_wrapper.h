#ifndef PCA9685_WRAPPER_H
#define PCA9685_WRAPPER_H

#ifdef __cplusplus
extern "C"
{
#endif

    void PCA9685(I2C_HandleTypeDef *hi2c, uint8_t address);
    void pca9685_init();
    void pca9685_set_angle(uint8_t channel, float angle_deg);

#ifdef __cplusplus
}
#endif

#endif // PCA9685_WRAPPER_H
