#ifndef ARM_PARAMS_H
#define ARM_PARAMS_H

typedef struct
{
    float d1;
    float a2;
    float a3;
    float d6;
} IKParams;

// in mm
static const IKParams ArmDimensions = {
    100.0f, // d1
    90.0f,  // a2
    -50.0f, // a3
    15.0f,  // d6
};
#endif