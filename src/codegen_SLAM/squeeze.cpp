//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: squeeze.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "squeeze.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double a[12]
//                double b[12]
// Return Type  : void
//
void squeeze(const double a[12], double b[12])
{
  memcpy(&b[0], &a[0], 12U * sizeof(double));
}

//
// File trailer for squeeze.cpp
//
// [EOF]
//
