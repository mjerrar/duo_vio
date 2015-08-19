//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//
#ifndef __GETH_R_RES_H__
#define __GETH_R_RES_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize,
  double stateSize, const double z_all_l[32], const emxArray_real_T *indMeas,
  const emxArray_real_T *map, const double c_cameraparams_CameraParameters[3],
  const double d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], double b_numAnchors, double
  b_numPointsPerAnchor, const emxArray_real_T *anchorIdx, const emxArray_real_T *
  featureAnchorIdx, const emxArray_real_T *b_m_vect, const double b_imNoise[2],
  const double IMU_measurements[23], emxArray_real_T *r, emxArray_real_T *H,
  emxArray_real_T *h_u, emxArray_real_T *R);

#endif

//
// File trailer for getH_R_res.h
//
// [EOF]
//
