//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: diag.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "diag.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double v[9]
//                double d[81]
// Return Type  : void
//
void b_diag(const double v[9], double d[81])
{
  int j;
  memset(&d[0], 0, 81U * sizeof(double));
  for (j = 0; j < 9; j++) {
    d[j + 9 * j] = v[j];
  }
}

//
// Arguments    : const creal_T v[16]
//                creal_T d[4]
// Return Type  : void
//
void diag(const creal_T v[16], creal_T d[4])
{
  int j;
  for (j = 0; j < 4; j++) {
    d[j] = v[j * 5];
  }
}

//
// File trailer for diag.cpp
//
// [EOF]
//
