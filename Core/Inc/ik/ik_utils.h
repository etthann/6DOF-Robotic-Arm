#ifndef IK_UTILS_H
#define IK_UTILS_H
#include "math.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // define struc
    typedef struct
    {
        float x;     // mm
        float y;     // mm
        float z;     // mm
        float yaw;   // radians
        float pitch; // radians
        float roll;  // radians
    } Pose;

    // Rotation Order -> ZYX (Yaw Pitch Roll)
    static inline void rpyToRotationMatrix(float r, float p, float y, float R[3][3])
    {
        const float cosP = cosf(p);
        const float cosY = cosf(y);
        const float cosR = cosf(r);
        const float sinP = sinf(p);
        const float sinY = sinf(y);
        const float sinR = sinf(r);

        R[0][0] = cosY * cosP;
        R[0][1] = sinY * cosP;
        R[0][2] = -sinP;

        R[1][0] = cosY * sinP - sinY * cosR;
        R[1][1] = sinY * sinP * sinR + cosY * cosR;
        R[1][2] = cosP * sinR;

        R[2][0] = cosY * sinP * cosR + sinY * sinR;
        R[2][1] = cosY * sinP * cosR - cosY * sinR;
        R[2][2] = cosP * cosR;
    }

    // Matrix Multiplication, C = A X B
    static inline void matrixMul3(const float A[3][3], const float B[3][3], float C[3][3])
    {

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
            }
        }
    }

    // Transpose 3x3 Matrix
    static inline void matT3(const float A[3][3], float AT[3][3])
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                AT[i][j] = A[j][i];
    }

    // constrain joint angle
    static inline float constrain(float value, float min, float max)
    {
        // if val < min, return min
        // if val > max, return max
        return value < min ? min : (value > max ? max : value);
    }

    static inline float rad2deg(float rad)
    {
        // 180/pi
        return rad * 57.2957795f;
    }

    static inline float degToRad(float deg)
    {
        // pi/ 180
        return deg * 0.0174532925f;
    }

    // get 3x3 rotation matrix from Joints 1-3
    static inline void r03FromT123(float t1, float t2, float t3, float R03[3][3])
    {
        float cosT1 = cosf(t1);
        float cosT2 = cosf(t2);
        float cosT3 = cosf(t3);
        float sinT1 = sinf(t1);
        float sinT2 = sinf(t2);
        float sinT3 = sinf(t3);

        float R01[3][3] = {
            {cosT1, -sinT1, 0},
            {sinT1, cosT1, 0},
            {0, 0, 1}};

        float R12[3][3] = {
            {cosT2, 0, sinT2},
            {0, 1, 0},
            {-sinT2, 0, cosT2}};

        float R23[3][3] = {
            {cosT3, 0, sinT3},
            {0, 1, 0},
            {-sinT3, 0, cosT3}};

        float R02[3][3];
        // connect from 0 to frame 2
        matrixMul3(R01, R12, R02);
        // connect frame 2 to frame 3
        matrixMul3(R02, R23, R03);
    }

    static inline void wristFromR36(const float R36[3][3], float *t4, float *t5, float *t6)
    {
        *t5 = atan2f(-R36[2][0], sqrtf(R36[0][0] * R36[0][0] + R36[1][0] * R36[1][0]));
        // is pitch angle greater than 0.000001,
        if (fabsf(cosf(*t5)) > 1e-6f)
        {
            *t6 = atan2f(R36[1][0], R36[0][0]);
            *t4 = atan2f(R36[2][1], R36[2][2]);
        }
        else
        {
            *t6 = 0.0f;
            *t4 = atan2f(-R36[0][1], R36[1][1]);
        }
    }

#ifdef __cplusplus
}
#endif

#endif