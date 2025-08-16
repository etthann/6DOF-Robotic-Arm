#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include "ik_utils.h"
#include "arm_params.h"

#ifdef _cplusplus
extern "C"
{
#endif
#ifdef _cplusplus
}
#endif

bool ik6_spherical(const IKParams &P, const Pose &goal, float out_deg[6], bool elbow_up = false);
#endif