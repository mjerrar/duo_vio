//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_pred.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 22:40:16
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "Att_pred.h"
#include "quatPlusThetaJ.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// ATT_PRED Prediction step of the attitude estimator
//    INPUT ARGUMENTS:
//    - x:  The current estimated attitude (JPL quaternion)
//    - P:  The state covariance matrix (3 x 3)
//    - w:  The current Gyro measurement (3 x 1)
//    - q:  The proecss noise
//    - dt: The time step
// Arguments    : double x[4]
//                double b_P[9]
//                const double w[3]
//                double q
//                double dt
// Return Type  : void
//
void Att_pred(double x[4], double b_P[9], const double w[3], double q, double dt)
{
  signed char I[9];
  int i16;
  double Phi[9];
  double b_Phi[9];
  int k;
  double c;
  double a[9];
  double c_Phi[9];
  double b_a[9];
  int i17;
  static const signed char c_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char d_a[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double b_w[3];
  double dq[4];
  double b_x[16];
  double b_dq[4];
  for (i16 = 0; i16 < 9; i16++) {
    I[i16] = 0;
  }

  Phi[0] = 0.0;
  Phi[3] = -w[2];
  Phi[6] = w[1];
  Phi[1] = w[2];
  Phi[4] = 0.0;
  Phi[7] = -w[0];
  Phi[2] = -w[1];
  Phi[5] = w[0];
  Phi[8] = 0.0;
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1;
    for (i16 = 0; i16 < 3; i16++) {
      b_Phi[i16 + 3 * k] = (double)I[i16 + 3 * k] + -Phi[i16 + 3 * k] * dt;
    }
  }

  c = dt * dt;
  for (i16 = 0; i16 < 3; i16++) {
    for (k = 0; k < 3; k++) {
      Phi[i16 + 3 * k] = 0.0;
      for (i17 = 0; i17 < 3; i17++) {
        Phi[i16 + 3 * k] += b_Phi[i16 + 3 * i17] * b_P[i17 + 3 * k];
      }

      a[i16 + 3 * k] = 0.0;
      for (i17 = 0; i17 < 3; i17++) {
        a[i16 + 3 * k] += (double)d_a[i16 + 3 * i17] * ((double)c_a[i17 + 3 * k]
          * q * c);
      }
    }

    for (k = 0; k < 3; k++) {
      c_Phi[i16 + 3 * k] = 0.0;
      for (i17 = 0; i17 < 3; i17++) {
        c_Phi[i16 + 3 * k] += Phi[i16 + 3 * i17] * b_Phi[k + 3 * i17];
      }

      b_a[i16 + 3 * k] = 0.0;
      for (i17 = 0; i17 < 3; i17++) {
        b_a[i16 + 3 * k] += a[i16 + 3 * i17] * (double)d_a[i17 + 3 * k];
      }
    }
  }

  for (i16 = 0; i16 < 3; i16++) {
    for (k = 0; k < 3; k++) {
      b_P[k + 3 * i16] = c_Phi[k + 3 * i16] + b_a[k + 3 * i16];
    }

    b_w[i16] = w[i16] * dt;
  }

  quatPlusThetaJ(b_w, dq);
  b_x[0] = x[3];
  b_x[4] = -x[2];
  b_x[8] = x[1];
  b_x[12] = x[0];
  b_x[1] = x[2];
  b_x[5] = x[3];
  b_x[9] = -x[0];
  b_x[13] = x[1];
  b_x[2] = -x[1];
  b_x[6] = x[0];
  b_x[10] = x[3];
  b_x[14] = x[2];
  b_x[3] = -x[0];
  b_x[7] = -x[1];
  b_x[11] = -x[2];
  b_x[15] = x[3];
  b_dq[0] = dq[0];
  b_dq[1] = dq[1];
  b_dq[2] = dq[2];
  b_dq[3] = dq[3];
  for (i16 = 0; i16 < 4; i16++) {
    x[i16] = 0.0;
    for (k = 0; k < 4; k++) {
      x[i16] += b_x[i16 + (k << 2)] * b_dq[k];
    }
  }
}

//
// File trailer for Att_pred.cpp
//
// [EOF]
//
