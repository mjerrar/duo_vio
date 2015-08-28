//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 21:34:41
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
extern void b_getH_R_res(const emxArray_real_T *b_xt, double errorStateSize,
  double stateSize, const double z_all_l[32], const double indMeas_data[], const
  int indMeas_size[1], const emxArray_real_T *map, const double
  c_cameraparams_CameraParameters[3], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const emxArray_real_T *anchorIdx, const
  emxArray_real_T *featureAnchorIdx, const emxArray_real_T *b_m_vect, const
  double noiseParameters_image_noise[2], double c_noiseParameters_orientation_n,
  double noiseParameters_pressure_noise, double noiseParameters_ext_pos_noise,
  double noiseParameters_ext_att_noise, const VIOMeasurements *measurements,
  double b_height_offset_pressure, const VIOParameters b_VIOParameters, double
  r_data[], int r_size[1], emxArray_real_T *H, double h_u_data[], int h_u_size[1],
  double R_data[], int R_size[2]);
extern void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize,
  double stateSize, const double z_all_l[32], double indMeas, const
  emxArray_real_T *map, const double c_cameraparams_CameraParameters[3], const
  double d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const emxArray_real_T *anchorIdx, const
  emxArray_real_T *featureAnchorIdx, const emxArray_real_T *b_m_vect, const
  double noiseParameters_image_noise[2], double c_noiseParameters_orientation_n,
  double noiseParameters_pressure_noise, double noiseParameters_ext_pos_noise,
  double noiseParameters_ext_att_noise, const VIOMeasurements *measurements,
  double b_height_offset_pressure, const VIOParameters b_VIOParameters, double
  r_data[], int r_size[1], emxArray_real_T *H, double h_u[2], double R_data[],
  int R_size[2]);

#endif

//
// File trailer for getH_R_res.h
//
// [EOF]
//
