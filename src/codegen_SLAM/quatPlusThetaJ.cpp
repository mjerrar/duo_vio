//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quatPlusThetaJ.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 20-Aug-2015 14:00:17
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "quatPlusThetaJ.h"
#include "norm.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double dtheta[3]
//                double dq[4]
// Return Type  : void
//
void quatPlusThetaJ(const double dtheta[3], double dq[4])
{
  double theta;
  int i;
  double B;
  theta = norm(dtheta) * 0.5;
  if (theta < 0.244) {
    for (i = 0; i < 3; i++) {
      dq[i] = 0.5 * dtheta[i];
    }

    dq[3] = 1.0;
  } else {
    dq[0] = 0.5 * dtheta[0] * sin(theta) / theta;
    dq[1] = 0.5 * dtheta[1] * sin(theta) / theta;
    dq[2] = 0.5 * dtheta[2] * sin(theta) / theta;
    dq[3] = cos(theta);
  }

  B = b_norm(dq);
  for (i = 0; i < 4; i++) {
    dq[i] /= B;
  }
}

//
// File trailer for quatPlusThetaJ.cpp
//
// [EOF]
//
