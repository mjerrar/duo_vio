//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_upd.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
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
                     double z_all_l[80], double z_all_r[80], double
                     noiseParameters_image_noise, double
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, emxArray_real_T *h_u_apo, emxArray_real_T *
                     b_map, double b_delayedStatus[40]);

#endif

//
// File trailer for SLAM_upd.h
//
// [EOF]
//
