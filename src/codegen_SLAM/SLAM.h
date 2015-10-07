//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
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

// Function Declarations
extern void SLAM(int updateVect[40], const double z_all_l[80], const double
                 z_all_r[80], double dt, const VIOMeasurements *measurements,
                 const DUOParameters *cameraParameters, const NoiseParameters
                 *noiseParameters, const VIOParameters *b_VIOParameters,
                 boolean_T vision, RobotState *xt_out, emxArray_real_T *h_u_out,
                 emxArray_real_T *map_out, emxArray_AnchorPose *anchor_poses_out,
                 double delayedStatus_out[40]);
extern void SLAM_free();
extern void SLAM_init();
extern void initialized_not_empty_init();

#endif

//
// File trailer for SLAM.h
//
// [EOF]
//
