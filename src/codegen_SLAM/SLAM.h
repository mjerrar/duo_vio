//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 13-Oct-2015 17:09:41
//
#ifndef __SLAM_H__
#define __SLAM_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Type Definitions
#include <stdio.h>

// Function Declarations
extern void SLAM(int updateVect[40], const double z_all_l[80], const double
                 z_all_r[80], double dt, const VIOMeasurements *measurements,
                 const DUOParameters *cameraParameters, const NoiseParameters
                 *noiseParameters, const VIOParameters *b_VIOParameters,
                 boolean_T vision, RobotState *xt_out, double h_u_out[160],
                 double map_out[120], AnchorPose anchor_poses_out[5], double
                 delayedStatus_out[40]);
extern void SLAM_initialize();
extern void SLAM_terminate();

#endif

//
// File trailer for SLAM.h
//
// [EOF]
//
