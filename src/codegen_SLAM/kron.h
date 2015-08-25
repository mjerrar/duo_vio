//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: kron.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 25-Aug-2015 17:43:12
//
#ifndef __KRON_H__
#define __KRON_H__

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
extern void kron(const double A_data[], const int A_size[2], const double B[4],
                 double K_data[], int K_size[2]);

#endif

//
// File trailer for kron.h
//
// [EOF]
//
