//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 02-Nov-2015 16:25:43
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
                 z_all_r[80], double dt, const struct0_T *measurements, const
                 struct1_T *cameraParameters, const struct3_T *noiseParameters,
                 const struct5_T *VIOParameters, boolean_T vision, struct6_T
                 *xt_out, double map_out[120], struct8_T anchor_poses_out[5],
                 double delayedStatus_out[40]);
extern void SLAM_initialize();
extern void SLAM_terminate();

#endif

//
// File trailer for SLAM.h
//
// [EOF]
//
