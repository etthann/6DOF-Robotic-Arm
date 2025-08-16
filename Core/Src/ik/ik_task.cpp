#include "cmsis_os2.h"
#include "ik/ik_solver.h"
#include "ik/arm_params.h"

extern "C"
{
#include "pca9685/pca9685_wrapper.h"
}

// per-joint mapping: tune these after first power-up
static float sign[6] = {+1, -1, +1, +1, +1, +1};
static float offset[6] = {90, 90, 90, 90, 90, 90}; // mech zero -> 90 as a start
static float min[6] = {0, 0, 0, 0, 0, 0};
static float max[6] = {180, 180, 180, 180, 180, 180};

static inline float to_servo_deg(int j, float ik_deg)
{
    float d = sign[j] * ik_deg + offset[j];
    if (d < min[j])
        d = min[j];
    if (d > max[j])
        d = max[j];
    return d;
}

extern "C" void StartIKTask()
{
    PCA9685_Init();

    // Simple goal pose
    Pose goal{160, 0, 120, 0.0f, 0.0f, 0.0f};

    for (;;)
    {
        float qdeg[6];
        if (ik6_spherical(ArmDimensions, goal, qdeg, /*elbow up=*/false))
        {
            for (int i = 0; i < 6; i++)
            {
                PCA9685_SetServoAngle((uint8_t)i, to_servo_deg(i, qdeg[i]));
                osDelay(10);
            }
        }
        osDelay(50);
    }
}
