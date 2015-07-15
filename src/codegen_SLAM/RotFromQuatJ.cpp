//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RotFromQuatJ.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "RotFromQuatJ.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double q[4]
//                double R[9]
// Return Type  : void
//
void RotFromQuatJ(const double q[4], double R[9])
{
  R[0] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
  R[3] = 2.0 * (q[0] * q[1] + q[2] * q[3]);
  R[6] = 2.0 * (q[0] * q[2] - q[1] * q[3]);
  R[1] = 2.0 * (q[0] * q[1] - q[2] * q[3]);
  R[4] = ((-(q[0] * q[0]) + q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
  R[7] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  R[2] = 2.0 * (q[0] * q[2] + q[1] * q[3]);
  R[5] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  R[8] = ((-(q[0] * q[0]) - q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
}

//
// File trailer for RotFromQuatJ.cpp
//
// [EOF]
//
