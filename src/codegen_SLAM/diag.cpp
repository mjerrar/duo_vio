//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: diag.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 21:38:20
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "diag.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double v[9]
//                double d[81]
// Return Type  : void
//
void diag(const double v[9], double d[81])
{
  int j;
  memset(&d[0], 0, 81U * sizeof(double));
  for (j = 0; j < 9; j++) {
    d[j + 9 * j] = v[j];
  }
}

//
// File trailer for diag.cpp
//
// [EOF]
//
