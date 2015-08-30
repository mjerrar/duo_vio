//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quatmultJ.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 30-Aug-2015 14:06:09
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "quatmultJ.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double q[4]
//                const double p[4]
//                double qp[4]
// Return Type  : void
//
void quatmultJ(const double q[4], const double p[4], double qp[4])
{
  double b_p[16];
  double b_q[4];
  int i8;
  int i9;
  b_p[0] = p[3];
  b_p[4] = -p[2];
  b_p[8] = p[1];
  b_p[12] = p[0];
  b_p[1] = p[2];
  b_p[5] = p[3];
  b_p[9] = -p[0];
  b_p[13] = p[1];
  b_p[2] = -p[1];
  b_p[6] = p[0];
  b_p[10] = p[3];
  b_p[14] = p[2];
  b_p[3] = -p[0];
  b_p[7] = -p[1];
  b_p[11] = -p[2];
  b_p[15] = p[3];
  b_q[0] = q[0];
  b_q[1] = q[1];
  b_q[2] = q[2];
  b_q[3] = q[3];
  for (i8 = 0; i8 < 4; i8++) {
    qp[i8] = 0.0;
    for (i9 = 0; i9 < 4; i9++) {
      qp[i8] += b_p[i8 + (i9 << 2)] * b_q[i9];
    }
  }
}

//
// File trailer for quatmultJ.cpp
//
// [EOF]
//
