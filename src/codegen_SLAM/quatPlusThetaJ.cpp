//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quatPlusThetaJ.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
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
  int k;
  double y;
  double scale;
  double absxk;
  double t;
  theta = norm(dtheta) * 0.5;
  if (theta < 0.244) {
    for (k = 0; k < 3; k++) {
      dq[k] = 0.5 * dtheta[k];
    }

    dq[3] = 1.0;
  } else {
    dq[0] = 0.5 * dtheta[0] * sin(theta) / theta;
    dq[1] = 0.5 * dtheta[1] * sin(theta) / theta;
    dq[2] = 0.5 * dtheta[2] * sin(theta) / theta;
    dq[3] = cos(theta);
  }

  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 4; k++) {
    absxk = fabs(dq[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  y = scale * sqrt(y);
  for (k = 0; k < 4; k++) {
    dq[k] /= y;
  }
}

//
// File trailer for quatPlusThetaJ.cpp
//
// [EOF]
//
