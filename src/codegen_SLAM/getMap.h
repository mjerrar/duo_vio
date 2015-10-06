//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getMap.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//
#ifndef __GETMAP_H__
#define __GETMAP_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void getMap(const double xt_origin_pos[3], const double xt_origin_att[4],
                   const emxArray_b_struct_T *xt_anchor_states, emxArray_real_T *
                   b_map);

#endif

//
// File trailer for getMap.h
//
// [EOF]
//
