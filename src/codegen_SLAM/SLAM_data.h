//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_data.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 14-Aug-2015 18:03:58
//
#ifndef __SLAM_DATA_H__
#define __SLAM_DATA_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Variable Declarations
extern double numStates;
extern double numStatesxt;
extern double minFeatureThreshold;
extern double sigma_Init;
extern double maxEKFIterations;
extern unsigned int state[625];
extern boolean_T gravityUpdate;
extern double gravAlignNoise;
extern boolean_T useAirPressure;
extern boolean_T normalGravity;

#endif

//
// File trailer for SLAM_data.h
//
// [EOF]
//
