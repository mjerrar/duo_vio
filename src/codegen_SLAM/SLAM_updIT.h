//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//
#ifndef __SLAM_UPDIT_H__
#define __SLAM_UPDIT_H__

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
extern void SLAM_updIT(struct_T *b_SLAM_data, double updateVect[16], const
  double z_all_l[32], const double z_all_r[32], const double IMU_measurements[23],
  emxArray_real_T *h_u_apo, emxArray_real_T *map);

#endif

//
// File trailer for SLAM_updIT.h
//
// [EOF]
//
