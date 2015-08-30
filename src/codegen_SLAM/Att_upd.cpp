//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_upd.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 30-Aug-2015 15:50:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "Att_upd.h"
#include "norm.h"
#include "mrdivide.h"
#include <ros/console.h>
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
// Arguments    : const double x_data[]
//                double P_data[]
//                int P_size[2]
//                const double z[3]
//                double n
//                double dt
//                double x[4]
// Return Type  : void
//
void Att_upd(const double x_data[], double P_data[], int P_size[2], const double
             z[3], double n, double dt, double x[4])
{
  double b_x[9];
  double z_pred[3];
  double r[3];
  int cr;
  double b_z_pred;
  double H[9];
  double c;
  double y_data[9];
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  double y[9];
  int K_size[2];
  double K_data[9];
  static const signed char a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int x_apo_size[1];
  double x_apo_data[3];
  emxArray_real_T b_x_apo_data;
  double theta;
  int dq_size[1];
  double dq_data[4];
  double dv34[4];
  emxArray_real_T b_dq_data;
  double c_x[16];
  double dq[4];
  double C_data[9];

  //  predicted measurement
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  b_x[0] = ((x_data[0] * x_data[0] - x_data[1] * x_data[1]) - x_data[2] *
            x_data[2]) + x_data[3] * x_data[3];
  b_x[3] = 2.0 * (x_data[0] * x_data[1] + x_data[2] * x_data[3]);
  b_x[6] = 2.0 * (x_data[0] * x_data[2] - x_data[1] * x_data[3]);
  b_x[1] = 2.0 * (x_data[0] * x_data[1] - x_data[2] * x_data[3]);
  b_x[4] = ((-(x_data[0] * x_data[0]) + x_data[1] * x_data[1]) - x_data[2] *
            x_data[2]) + x_data[3] * x_data[3];
  b_x[7] = 2.0 * (x_data[1] * x_data[2] + x_data[0] * x_data[3]);
  b_x[2] = 2.0 * (x_data[0] * x_data[2] + x_data[1] * x_data[3]);
  b_x[5] = 2.0 * (x_data[1] * x_data[2] - x_data[0] * x_data[3]);
  b_x[8] = ((-(x_data[0] * x_data[0]) - x_data[1] * x_data[1]) + x_data[2] *
            x_data[2]) + x_data[3] * x_data[3];
  for (cr = 0; cr < 3; cr++) {
    b_z_pred = b_x[6 + cr] * 9.81;
    r[cr] = z[cr] - b_z_pred;
    z_pred[cr] = b_z_pred;
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
  c = dt * dt;
  for (cr = 0; cr < 9; cr++) {
    y_data[cr] = 0.0;
  }

  for (cr = 0; cr <= 6; cr += 3) {
    for (ic = cr + 1; ic <= cr + 3; ic++) {
      y_data[ic - 1] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= 6; cr += 3) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (P_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 3; ic++) {
          ia++;
          y_data[ic] += P_data[ib] * H[ia - 1];
        }
      }

      ar += 3;
    }

    br += 3;
  }

  for (cr = 0; cr < 3; cr++) {
    for (br = 0; br < 3; br++) {
      b_x[br + 3 * cr] = H[cr + 3 * br];
    }
  }

  memset(&y[0], 0, 9U * sizeof(double));
  for (cr = 0; cr < 8; cr += 3) {
    for (ic = cr; ic + 1 <= cr + 3; ic++) {
      y[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr < 8; cr += 3) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (b_x[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 3; ic++) {
          ia++;
          y[ic] += b_x[ib] * y_data[ia - 1];
        }
      }

      ar += 3;
    }

    br += 3;
  }

  for (cr = 0; cr < 3; cr++) {
    for (br = 0; br < 3; br++) {
      b_x[br + 3 * cr] = H[cr + 3 * br];
    }
  }

  K_size[0] = 3;
  K_size[1] = 3;
  memset(&K_data[0], 0, 9U * sizeof(double));
  for (cr = 0; cr < 7; cr += 3) {
    for (ic = cr; ic + 1 <= cr + 3; ic++) {
      K_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr < 7; cr += 3) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (b_x[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 3; ic++) {
          ia++;
          K_data[ic] += b_x[ib] * P_data[ia - 1];
        }
      }

      ar += 3;
    }

    br += 3;
  }

  for (cr = 0; cr < 9; cr++) {
    b_x[cr] = y[cr] + (double)a[cr] * n * c;
  }

  c_mrdivide(K_data, K_size, b_x);
  x_apo_size[0] = 3;
  for (cr = 0; cr < 3; cr++) {
    x_apo_data[cr] = 0.0;
  }

  for (ic = 1; ic < 4; ic++) {
    x_apo_data[ic - 1] = 0.0;
  }

  ar = 0;
  for (ib = 0; ib + 1 < 4; ib++) {
    if (r[ib] != 0.0) {
      ia = ar;
      for (ic = 0; ic + 1 < 4; ic++) {
        ia++;
        x_apo_data[ic] += r[ib] * K_data[ia - 1];
      }
    }

    ar += 3;
  }

  b_x_apo_data.data = (double *)&x_apo_data;
  b_x_apo_data.size = (int *)&x_apo_size;
  b_x_apo_data.allocatedSize = 3;
  b_x_apo_data.numDimensions = 1;
  b_x_apo_data.canFreeData = false;
  theta = d_norm(&b_x_apo_data) * 0.5;
  if (theta < 0.244) {
    dq_size[0] = 4;
    for (cr = 0; cr < 3; cr++) {
      dq_data[cr] = 0.5 * x_apo_data[cr];
    }

    dq_data[3] = 1.0;
  } else {
    dv34[0] = 0.5 * x_apo_data[0] * sin(theta) / theta;
    dv34[1] = 0.5 * x_apo_data[1] * sin(theta) / theta;
    dv34[2] = 0.5 * x_apo_data[2] * sin(theta) / theta;
    dv34[3] = cos(theta);
    dq_size[0] = 4;
    for (cr = 0; cr < 4; cr++) {
      dq_data[cr] = dv34[cr];
    }
  }

  b_dq_data.data = (double *)&dq_data;
  b_dq_data.size = (int *)&dq_size;
  b_dq_data.allocatedSize = 4;
  b_dq_data.numDimensions = 1;
  b_dq_data.canFreeData = false;
  c = d_norm(&b_dq_data);
  dq_size[0] = 4;
  for (cr = 0; cr < 4; cr++) {
    dq_data[cr] /= c;
  }

  c_x[0] = x_data[3];
  c_x[4] = -x_data[2];
  c_x[8] = x_data[1];
  c_x[12] = x_data[0];
  c_x[1] = x_data[2];
  c_x[5] = x_data[3];
  c_x[9] = -x_data[0];
  c_x[13] = x_data[1];
  c_x[2] = -x_data[1];
  c_x[6] = x_data[0];
  c_x[10] = x_data[3];
  c_x[14] = x_data[2];
  c_x[3] = -x_data[0];
  c_x[7] = -x_data[1];
  c_x[11] = -x_data[2];
  c_x[15] = x_data[3];
  dq[0] = dq_data[0];
  dq[1] = dq_data[1];
  dq[2] = dq_data[2];
  dq[3] = dq_data[3];
  for (cr = 0; cr < 4; cr++) {
    x[cr] = 0.0;
    for (br = 0; br < 4; br++) {
      x[cr] += c_x[cr + (br << 2)] * dq[br];
    }
  }

  //  x(1) = 0;
  //  x = normc(x);
  memset(&C_data[0], 0, 9U * sizeof(double));
  for (cr = 0; cr < 7; cr += 3) {
    for (ic = cr; ic + 1 <= cr + 3; ic++) {
      C_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr < 7; cr += 3) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (H[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 3; ic++) {
          ia++;
          C_data[ic] += H[ib] * K_data[ia - 1];
        }
      }

      ar += 3;
    }

    br += 3;
  }

  memset(&H[0], 0, 9U * sizeof(double));
  for (cr = 0; cr < 3; cr++) {
    H[cr + 3 * cr] = 1.0;
  }

  for (cr = 0; cr < 9; cr++) {
    H[cr] -= C_data[cr];
  }

  for (cr = 0; cr < 9; cr++) {
    y_data[cr] = 0.0;
  }

  for (cr = 0; cr <= 6; cr += 3) {
    for (ic = cr + 1; ic <= cr + 3; ic++) {
      y_data[ic - 1] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= 6; cr += 3) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (P_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 3; ic++) {
          ia++;
          y_data[ic] += P_data[ib] * H[ia - 1];
        }
      }

      ar += 3;
    }

    br += 3;
  }

  P_size[0] = 3;
  P_size[1] = 3;
  for (cr = 0; cr < 3; cr++) {
    for (br = 0; br < 3; br++) {
      P_data[br + P_size[0] * cr] = y_data[br + 3 * cr];
    }
  }
}

//
// File trailer for Att_upd.cpp
//
// [EOF]
//
