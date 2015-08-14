//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: blkdiag.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 14-Aug-2015 15:27:50
//
#ifndef __BLKDIAG_H__
#define __BLKDIAG_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void b_blkdiag(const double varargin_1[4], const double varargin_2_data[],
                      const int varargin_2_size[2], double y_data[], int y_size
                      [2]);
extern void blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
                    *varargin_2, emxArray_real_T *y);
extern void output_size(const int varargin_1_size[2], const int varargin_2_size
  [2], const int varargin_3_size[2], int *nrows, int *ncols);

#endif

//
// File trailer for blkdiag.h
//
// [EOF]
//
