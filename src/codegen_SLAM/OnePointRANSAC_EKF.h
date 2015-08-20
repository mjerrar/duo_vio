//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 20-Aug-2015 15:34:21
//
#ifndef __ONEPOINTRANSAC_EKF_H__
#define __ONEPOINTRANSAC_EKF_H__

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
extern void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P,
  const double z_all_l[32], double c_numStatesxt, double c_numStates, double
  numPointsPerAnchor, const double c_cameraparams_CameraParameters[3], const
  double d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const emxArray_real_T *b_anchorFeatures,
  const emxArray_real_T *b_m_vect, const double imNoise[2], const double
  IMU_measurements[23], double height_offset_pressure, double
  validFeatures_data[], int validFeatures_size[1]);

#endif

//
// File trailer for OnePointRANSAC_EKF.h
//
// [EOF]
//
