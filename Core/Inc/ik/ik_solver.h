#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <stdbool.h>
#include "ik/ik_utils.h"
#include "ik/arm_params.h"

#ifdef __cplusplus
extern "C"
{
#endif

    bool ik6_spherical(const IKParams *P, const Pose *goal, float out_deg[6], bool elbow_up);

#ifdef __cplusplus
}
#endif

#endif
