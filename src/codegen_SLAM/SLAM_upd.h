//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_upd.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//
#ifndef __SLAM_UPD_H__
#define __SLAM_UPD_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void SLAM_upd(emxArray_real_T *P_apr, f_struct_T *b_xt, const double
                     c_cameraParams_CameraParameters[2], const double
                     d_cameraParams_CameraParameters[2], const double
                     e_cameraParams_CameraParameters[3], const double
                     f_cameraParams_CameraParameters[2], const double
                     g_cameraParams_CameraParameters[2], const double
                     h_cameraParams_CameraParameters[3], const double
                     cameraParams_r_lr[3], const double cameraParams_R_lr[9],
                     const double cameraParams_R_rl[9], int updateVect[40],
                     double z_all_l[80], double z_all_r[80], const double
                     noiseParameters_image_noise[2], double
                     noiseParameters_sigmaInit, double
                     c_VIOParameters_max_ekf_iterati, boolean_T
                     VIOParameters_fixed_feature, boolean_T
                     c_VIOParameters_delayed_initial, boolean_T
                     VIOParameters_mono, emxArray_real_T *h_u_apo,
                     emxArray_real_T *b_map, double b_delayedStatus[40]);

#endif

//
// File trailer for SLAM_upd.h
//
// [EOF]
//
