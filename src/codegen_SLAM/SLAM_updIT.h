//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//
#ifndef __SLAM_UPDIT_H__
#define __SLAM_UPDIT_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams_r_lr[3], const double cameraparams_R_lr[9], double
  updateVect[32], const double z_all_l[64], const double z_all_r[64], const
  double imNoise[4], emxArray_real_T *h_u_apo, double map_out[96]);
extern void SLAM_updIT_init();

#endif

//
// File trailer for SLAM_updIT.h
//
// [EOF]
//
