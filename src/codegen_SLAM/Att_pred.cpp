//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_pred.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 16:22:53
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
// Arguments    : const double x_data[]
//                const double P_data[]
//                const int P_size[2]
//                const double w[3]
//                double q
//                double dt
//                double x[4]
//                double b_P[9]
// Return Type  : void
//
void Att_pred(const double x_data[], const double P_data[], const int P_size[2],
              const double w[3], double q, double dt, double x[4], double b_P[9])
{
  double I[9];
  double a[9];
  double Phi[9];
  int k;
  int i12;
  double c;
  int y_size_idx_1;
  int cr;
  double y_data[9];
  int br;
  int ic;
  int ar;
  int ib;
  int ia;
  static const signed char b_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char c_a[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double b_w[3];
  double dq[4];
  double b_x[16];
  double b_dq[4];
  memset(&I[0], 0, 9U * sizeof(double));
  a[0] = 0.0;
  a[3] = -w[2];
  a[6] = w[1];
  a[1] = w[2];
  a[4] = 0.0;
  a[7] = -w[0];
  a[2] = -w[1];
  a[5] = w[0];
  a[8] = 0.0;
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0;
    for (i12 = 0; i12 < 3; i12++) {
      Phi[i12 + 3 * k] = I[i12 + 3 * k] + -a[i12 + 3 * k] * dt;
    }
  }

  c = dt * dt;
  if (P_size[0] == 1) {
    y_size_idx_1 = P_size[1];
    for (i12 = 0; i12 < 3; i12++) {
      k = P_size[1];
      for (cr = 0; cr < k; cr++) {
        y_data[i12 + 3 * cr] = 0.0;
        for (br = 0; br < 3; br++) {
          y_data[i12 + 3 * cr] += Phi[i12 + 3 * br] * P_data[br + cr];
        }
      }
    }
  } else {
    y_size_idx_1 = (signed char)P_size[1];
    k = 3 * (signed char)P_size[1];
    for (i12 = 0; i12 < k; i12++) {
      y_data[i12] = 0.0;
    }

    k = 3 * (P_size[1] - 1);
    for (cr = 0; cr <= k; cr += 3) {
      for (ic = cr; ic + 1 <= cr + 3; ic++) {
        y_data[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= k; cr += 3) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 3; ib++) {
        if (P_data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 3; ic++) {
            ia++;
            y_data[ic] += P_data[ib] * Phi[ia];
          }
        }

        ar += 3;
      }

      br += 3;
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (cr = 0; cr < 3; cr++) {
      I[cr + 3 * i12] = Phi[i12 + 3 * cr];
    }
  }

  if (y_size_idx_1 == 1) {
    for (i12 = 0; i12 < 3; i12++) {
      for (cr = 0; cr < 3; cr++) {
        Phi[i12 + 3 * cr] = 0.0;
        for (br = 0; br < 3; br++) {
          Phi[i12 + 3 * cr] += y_data[i12 + 3 * br] * I[br + 3 * cr];
        }
      }
    }
  } else {
    memset(&Phi[0], 0, 9U * sizeof(double));
    for (cr = 0; cr < 8; cr += 3) {
      for (ic = cr; ic + 1 <= cr + 3; ic++) {
        Phi[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr < 8; cr += 3) {
      ar = -1;
      i12 = br + y_size_idx_1;
      for (ib = br; ib + 1 <= i12; ib++) {
        if (I[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 3; ic++) {
            ia++;
            Phi[ic] += I[ib] * y_data[ia];
          }
        }

        ar += 3;
      }

      br += y_size_idx_1;
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (cr = 0; cr < 3; cr++) {
      a[i12 + 3 * cr] = 0.0;
      for (br = 0; br < 3; br++) {
        a[i12 + 3 * cr] += (double)c_a[i12 + 3 * br] * ((double)b_a[br + 3 * cr]
          * q * c);
      }
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (cr = 0; cr < 3; cr++) {
      c = 0.0;
      for (br = 0; br < 3; br++) {
        c += a[i12 + 3 * br] * (double)c_a[br + 3 * cr];
      }

      b_P[i12 + 3 * cr] = Phi[i12 + 3 * cr] + c;
    }
  }

  for (k = 0; k < 3; k++) {
    b_w[k] = w[k] * dt;
  }

  quatPlusThetaJ(b_w, dq);
  b_x[0] = x_data[3];
  b_x[4] = -x_data[2];
  b_x[8] = x_data[1];
  b_x[12] = x_data[0];
  b_x[1] = x_data[2];
  b_x[5] = x_data[3];
  b_x[9] = -x_data[0];
  b_x[13] = x_data[1];
  b_x[2] = -x_data[1];
  b_x[6] = x_data[0];
  b_x[10] = x_data[3];
  b_x[14] = x_data[2];
  b_x[3] = -x_data[0];
  b_x[7] = -x_data[1];
  b_x[11] = -x_data[2];
  b_x[15] = x_data[3];
  b_dq[0] = dq[0];
  b_dq[1] = dq[1];
  b_dq[2] = dq[2];
  b_dq[3] = dq[3];
  for (i12 = 0; i12 < 4; i12++) {
    x[i12] = 0.0;
    for (cr = 0; cr < 4; cr++) {
      x[i12] += b_x[i12 + (cr << 2)] * b_dq[cr];
    }
  }
}

//
// File trailer for Att_pred.cpp
//
// [EOF]
//
