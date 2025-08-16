#include "ik/ik_solver.h"

bool ik6_spherical(const IKParams &P, const Pose &goal, float out_deg[6], bool elbow_up)
{
    // 1. Desired Orientation
    float R06[3][3];
    rpyToRotationMatrix(goal.roll, goal.pitch, goal.yaw, R06);
    float nx = R06[0][2];
    float ny = R06[1][2];
    float nz = R06[2][2];

    // 2. Calc Wrist Center
    float xw = goal.x - P.d6 * nx;
    float yw = goal.y - P.d6 * ny;
    float zw = goal.z - P.d6 * nz;

    // 3. Get Base Angle
    float t1 = atan2(yw, xw);

    // 4. planar triangle for shoulder/elbow??

    // get polar coords of wrist center
    float r = sqrtf(xw * xw + yw * yw); // horizontal reach from shoulder to wrist center
    float s = zw - P.d1;                // vertical offset from shoulder plane
    float r2s2 = r * r + s * s;         // square distance shoulder->wrist

    // get elbow angle via law of cosines
    float cosT3 = (r2s2 - P.a2 * P.a2 - P.a3 * P.a3) / (2.0f * P.a2 * P.a3);

    if (cosT3 < -1.0f || cosT3 > 1.0f)
    {
        return false; // out of reach
    }

    cosT3 = constrain(cosT3, -1.0f, 1.0f);

    float sinT3 = sqrtf(fmaxf(0.01f, 1.0f - cosT3 * cosT3));

    if (elbow_up)
    {
        sinT3 = -sinT3;
    }

    float t3 = atan2(sinT3, cosT3);

    float t2 = atan2f(s, r) - atan2f(P.a3 * sinT3, P.a2 + P.a3 * cosT3);

    // 5. Wrist
    float R03[3][3];
    r03FromT123(t1, t2, t3, R03);
    float RT03[3][3];
    matT3(R03, RT03);
    float R36[3][3];
    matrixMul3(RT03, R06, R36);

    float t4, t5, t6;
    wristFromR36(R36, t4, t5, t6);

    // 6) to degrees
    out_deg[0] = rad2deg(t1);
    out_deg[1] = rad2deg(t2);
    out_deg[2] = rad2deg(t3);
    out_deg[3] = rad2deg(t4);
    out_deg[4] = rad2deg(t5);
    out_deg[5] = rad2deg(t6);
    return true;
}