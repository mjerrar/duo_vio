//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 07-Mar-2016 14:28:04
//
#ifndef __SLAM_H__
#define __SLAM_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "NE10.h"
#include "mw_neon.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Type Definitions
#include <stdio.h>

// Function Declarations
extern void SLAM(int updateVect[48], const float z_all_l[96], const float
                 z_all_r[96], float dt, const VIOMeasurements *measurements,
                 const DUOParameters *cameraParameters, const NoiseParameters
                 *noiseParameters, const VIOParameters *b_VIOParameters,
                 boolean_T vision, boolean_T reset, RobotState *xt_out, float
                 map_out[144], AnchorPose anchor_poses_out[6], float
                 delayedStatus_out[48]);
extern void SLAM_initialize();
extern void SLAM_terminate();

#endif

//
// File trailer for SLAM.h
//
// [EOF]
//
