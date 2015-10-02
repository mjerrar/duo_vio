//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getAnchorPoses.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//
#ifndef __GETANCHORPOSES_H__
#define __GETANCHORPOSES_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void getAnchorPoses(const double xt_origin_pos[3], const double
  xt_origin_att[4], const emxArray_b_struct_T *xt_anchor_states,
  emxArray_c_struct_T *anchor_poses);

#endif

//
// File trailer for getAnchorPoses.h
//
// [EOF]
//
