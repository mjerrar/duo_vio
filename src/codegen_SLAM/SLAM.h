//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 21:50:35
//
#ifndef __SLAM_H__
#define __SLAM_H__

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
extern void SLAM(double updateVect[24], const double z_all_l[48], const double
                 z_all_r[48], double dt, const VIOMeasurements *measurements,
                 const ReferenceCommand *ref, const VIOParameters
                 *b_VIOParameters, const StereoParameters *cameraParameters,
                 const NoiseParameters *noiseParameters, const ControllerGains
                 *b_ControllerGains, boolean_T resetFlag, emxArray_real_T
                 *h_u_apo_out, emxArray_real_T *xt_out, emxArray_real_T
                 *P_apo_out, emxArray_real_T *map_out, double u_out[4]);
extern void SLAM_free();
extern void SLAM_init();
extern void initialized_not_empty_init();

#endif

//
// File trailer for SLAM.h
//
// [EOF]
//
