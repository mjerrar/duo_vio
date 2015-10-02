//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//
#ifndef __ONEPOINTRANSAC_EKF_H__
#define __ONEPOINTRANSAC_EKF_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void OnePointRANSAC_EKF(f_struct_T *b_xt, emxArray_real_T *b_P, const
  double z_u_l[80], const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double noiseParameters_image_noise[2],
  double c_VIOParameters_max_ekf_iterati, int updateVect[40]);

#endif

//
// File trailer for OnePointRANSAC_EKF.h
//
// [EOF]
//
