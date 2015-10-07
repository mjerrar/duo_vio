//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quatmultJ.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "quatmultJ.h"
#include "norm.h"
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
  int i10;
  int i11;
  double B;
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
  for (i10 = 0; i10 < 4; i10++) {
    qp[i10] = 0.0;
    for (i11 = 0; i11 < 4; i11++) {
      qp[i10] += b_p[i10 + (i11 << 2)] * b_q[i11];
    }
  }

  B = b_norm(qp);
  for (i10 = 0; i10 < 4; i10++) {
    qp[i10] /= B;
  }
}

//
// File trailer for quatmultJ.cpp
//
// [EOF]
//
