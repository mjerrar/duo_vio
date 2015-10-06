//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//
#ifndef __GETH_R_RES_H__
#define __GETH_R_RES_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const emxArray_b_struct_T *xt_anchor_states, const
  double z_all_l[80], const boolean_T b_status[40], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double noiseParameters_image_noise, emxArray_real_T *r, emxArray_real_T *H,
  emxArray_real_T *R);

#endif

//
// File trailer for getH_R_res.h
//
// [EOF]
//
