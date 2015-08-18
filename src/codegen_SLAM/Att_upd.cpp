//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_upd.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 18-Aug-2015 11:00:04
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "Att_upd.h"
#include "quatPlusThetaJ.h"
#include <stdio.h>

// Function Definitions

//
// ATT_UPD Update step of the attitude estimator
//    INPUT ARGUMENTS:
//    - x:  The current estimated attitude (JPL quaternion)
//    - P:  The state covariance matrix (3 x 3)
//    - z:  The current accelerometer measurement (3 x 1)
//    - n:  The measurement noise
//    - dt: The time step
// Arguments    : double x[4]
//                double b_P[9]
//                const double z[3]
//                double n
//                double dt
// Return Type  : void
//
void Att_upd(double x[4], double b_P[9], const double z[3], double n, double dt)
{
  double b_x[9];
  double z_pred[3];
  int rtemp;
  double H[9];
  double maxval;
  int k;
  int r1;
  double S[9];
  double a21;
  static const signed char a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double b_S[9];
  int r2;
  int r3;
  double K[9];
  double b_K[3];
  double b_z[3];
  double q[4];
  double c_x[16];
  double b_q[4];

  //  predicted measurement
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  b_x[0] = ((x[0] * x[0] - x[1] * x[1]) - x[2] * x[2]) + x[3] * x[3];
  b_x[3] = 2.0 * (x[0] * x[1] + x[2] * x[3]);
  b_x[6] = 2.0 * (x[0] * x[2] - x[1] * x[3]);
  b_x[1] = 2.0 * (x[0] * x[1] - x[2] * x[3]);
  b_x[4] = ((-(x[0] * x[0]) + x[1] * x[1]) - x[2] * x[2]) + x[3] * x[3];
  b_x[7] = 2.0 * (x[1] * x[2] + x[0] * x[3]);
  b_x[2] = 2.0 * (x[0] * x[2] + x[1] * x[3]);
  b_x[5] = 2.0 * (x[1] * x[2] - x[0] * x[3]);
  b_x[8] = ((-(x[0] * x[0]) - x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3];
  for (rtemp = 0; rtemp < 3; rtemp++) {
    z_pred[rtemp] = b_x[6 + rtemp] * 9.81;
  }

  H[0] = 0.0;
  H[3] = -z_pred[2];
  H[6] = z_pred[1];
  H[1] = z_pred[2];
  H[4] = 0.0;
  H[7] = -z_pred[0];
  H[2] = -z_pred[1];
  H[5] = z_pred[0];
  H[8] = 0.0;
  maxval = dt * dt;
  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      b_x[rtemp + 3 * k] = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        b_x[rtemp + 3 * k] += H[rtemp + 3 * r1] * b_P[r1 + 3 * k];
      }
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      a21 = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        a21 += b_x[rtemp + 3 * r1] * H[k + 3 * r1];
      }

      S[rtemp + 3 * k] = a21 + (double)a[rtemp + 3 * k] * n * maxval;
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      b_S[rtemp + 3 * k] = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        b_S[rtemp + 3 * k] += b_P[rtemp + 3 * r1] * H[k + 3 * r1];
      }
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(S[0]);
  a21 = fabs(S[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(S[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  S[r2] /= S[r1];
  S[r3] /= S[r1];
  S[3 + r2] -= S[r2] * S[3 + r1];
  S[3 + r3] -= S[r3] * S[3 + r1];
  S[6 + r2] -= S[r2] * S[6 + r1];
  S[6 + r3] -= S[r3] * S[6 + r1];
  if (fabs(S[3 + r3]) > fabs(S[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  S[3 + r3] /= S[3 + r2];
  S[6 + r3] -= S[3 + r3] * S[6 + r2];
  for (k = 0; k < 3; k++) {
    K[k + 3 * r1] = b_S[k] / S[r1];
    K[k + 3 * r2] = b_S[3 + k] - K[k + 3 * r1] * S[3 + r1];
    K[k + 3 * r3] = b_S[6 + k] - K[k + 3 * r1] * S[6 + r1];
    K[k + 3 * r2] /= S[3 + r2];
    K[k + 3 * r3] -= K[k + 3 * r2] * S[6 + r2];
    K[k + 3 * r3] /= S[6 + r3];
    K[k + 3 * r2] -= K[k + 3 * r3] * S[3 + r3];
    K[k + 3 * r1] -= K[k + 3 * r3] * S[r3];
    K[k + 3 * r1] -= K[k + 3 * r2] * S[r2];
    b_z[k] = z[k] - z_pred[k];
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    b_K[rtemp] = 0.0;
    for (k = 0; k < 3; k++) {
      b_K[rtemp] += K[rtemp + 3 * k] * b_z[k];
    }
  }

  quatPlusThetaJ(b_K, q);
  c_x[0] = x[3];
  c_x[4] = -x[2];
  c_x[8] = x[1];
  c_x[12] = x[0];
  c_x[1] = x[2];
  c_x[5] = x[3];
  c_x[9] = -x[0];
  c_x[13] = x[1];
  c_x[2] = -x[1];
  c_x[6] = x[0];
  c_x[10] = x[3];
  c_x[14] = x[2];
  c_x[3] = -x[0];
  c_x[7] = -x[1];
  c_x[11] = -x[2];
  c_x[15] = x[3];
  b_q[0] = q[0];
  b_q[1] = q[1];
  b_q[2] = q[2];
  b_q[3] = q[3];
  for (rtemp = 0; rtemp < 4; rtemp++) {
    x[rtemp] = 0.0;
    for (k = 0; k < 4; k++) {
      x[rtemp] += c_x[rtemp + (k << 2)] * b_q[k];
    }
  }

  //  x(1) = 0;
  //  x = normc(x);
  memset(&S[0], 0, 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    S[k + 3 * k] = 1.0;
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      a21 = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        a21 += K[rtemp + 3 * r1] * H[r1 + 3 * k];
      }

      b_S[rtemp + 3 * k] = S[rtemp + 3 * k] - a21;
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      b_x[rtemp + 3 * k] = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        b_x[rtemp + 3 * k] += b_S[rtemp + 3 * r1] * b_P[r1 + 3 * k];
      }
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      b_P[k + 3 * rtemp] = b_x[k + 3 * rtemp];
    }
  }
}

//
// File trailer for Att_upd.cpp
//
// [EOF]
//
