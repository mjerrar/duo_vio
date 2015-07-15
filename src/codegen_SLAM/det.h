//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: det.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//
#ifndef __DET_H__
#define __DET_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern double det(const emxArray_real_T *x);
extern void eml_lapack_xgetrf(int m, int n, emxArray_real_T *A, int lda, int
  ipiv_data[], int ipiv_size[2], int *info);

#endif

//
// File trailer for det.h
//
// [EOF]
//
