//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: diag.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 23:21:02
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "diag.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double v[12]
//                double d[144]
// Return Type  : void
//
void b_diag(const double v[12], double d[144])
{
  int j;
  memset(&d[0], 0, 144U * sizeof(double));
  for (j = 0; j < 12; j++) {
    d[j + 12 * j] = v[j];
  }
}

//
// Arguments    : const double v[3]
//                double d[9]
// Return Type  : void
//
void diag(const double v[3], double d[9])
{
  int j;
  memset(&d[0], 0, 9U * sizeof(double));
  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

//
// File trailer for diag.cpp
//
// [EOF]
//
