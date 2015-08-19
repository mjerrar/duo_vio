//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_pred.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "Att_pred.h"
#include "quatPlusThetaJ.h"
#include <stdio.h>

// Function Definitions

//
// ATT_PRED Prediction step of the attitude estimator
//    INPUT ARGUMENTS:
//    - SLAM_data
//    - w:  The current Gyro measurement (3 x 1)
//    - dt: The time step
// Arguments    : struct_T *b_SLAM_data
//                const double w[3]
//                double dt
// Return Type  : void
//
void Att_pred(struct_T *b_SLAM_data, const double w[3], double dt)
{
  double I[9];
  double c_SLAM_data[9];
  double Phi[9];
  int k;
  int i15;
  double d_SLAM_data;
  static const signed char a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double c;
  double b_w[3];
  double y[9];
  int i16;
  double dq[4];
  double e_SLAM_data;
  double f_SLAM_data;
  double g_SLAM_data;
  double h_SLAM_data;
  double i_SLAM_data;
  double j_SLAM_data;
  double k_SLAM_data;
  double l_SLAM_data;
  double m_SLAM_data;
  double n_SLAM_data;
  double o_SLAM_data;
  double p_SLAM_data;
  double q_SLAM_data;
  double r_SLAM_data;
  double s_SLAM_data;
  double t_SLAM_data[16];
  double b_dq[4];
  double u_SLAM_data[4];
  double b_y[9];
  double b_a[9];
  static const signed char c_a[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  memset(&I[0], 0, 9U * sizeof(double));
  c_SLAM_data[0] = 0.0;
  c_SLAM_data[3] = -w[2];
  c_SLAM_data[6] = w[1];
  c_SLAM_data[1] = w[2];
  c_SLAM_data[4] = 0.0;
  c_SLAM_data[7] = -w[0];
  c_SLAM_data[2] = -w[1];
  c_SLAM_data[5] = w[0];
  c_SLAM_data[8] = 0.0;
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0;
    for (i15 = 0; i15 < 3; i15++) {
      Phi[i15 + 3 * k] = I[i15 + 3 * k] + -c_SLAM_data[i15 + 3 * k] * dt;
    }
  }

  d_SLAM_data = b_SLAM_data->processNoise[1];
  for (i15 = 0; i15 < 9; i15++) {
    I[i15] = (double)a[i15] * d_SLAM_data;
  }

  c = dt * dt;
  for (i15 = 0; i15 < 3; i15++) {
    for (k = 0; k < 3; k++) {
      c_SLAM_data[k + 3 * i15] = b_SLAM_data->P->data[(k + b_SLAM_data->P->size
        [0] * (3 + i15)) + 3];
    }
  }

  for (i15 = 0; i15 < 3; i15++) {
    for (k = 0; k < 3; k++) {
      y[i15 + 3 * k] = 0.0;
      for (i16 = 0; i16 < 3; i16++) {
        y[i15 + 3 * k] += Phi[i15 + 3 * i16] * c_SLAM_data[i16 + 3 * k];
      }
    }

    b_w[i15] = w[i15] * dt;
  }

  quatPlusThetaJ(b_w, dq);
  d_SLAM_data = b_SLAM_data->xt->data[6];
  e_SLAM_data = b_SLAM_data->xt->data[5];
  f_SLAM_data = b_SLAM_data->xt->data[4];
  g_SLAM_data = b_SLAM_data->xt->data[3];
  h_SLAM_data = b_SLAM_data->xt->data[5];
  i_SLAM_data = b_SLAM_data->xt->data[6];
  j_SLAM_data = b_SLAM_data->xt->data[3];
  k_SLAM_data = b_SLAM_data->xt->data[4];
  l_SLAM_data = b_SLAM_data->xt->data[4];
  m_SLAM_data = b_SLAM_data->xt->data[3];
  n_SLAM_data = b_SLAM_data->xt->data[6];
  o_SLAM_data = b_SLAM_data->xt->data[5];
  p_SLAM_data = b_SLAM_data->xt->data[3];
  q_SLAM_data = b_SLAM_data->xt->data[4];
  r_SLAM_data = b_SLAM_data->xt->data[5];
  s_SLAM_data = b_SLAM_data->xt->data[6];
  t_SLAM_data[0] = d_SLAM_data;
  t_SLAM_data[4] = -e_SLAM_data;
  t_SLAM_data[8] = f_SLAM_data;
  t_SLAM_data[12] = g_SLAM_data;
  t_SLAM_data[1] = h_SLAM_data;
  t_SLAM_data[5] = i_SLAM_data;
  t_SLAM_data[9] = -j_SLAM_data;
  t_SLAM_data[13] = k_SLAM_data;
  t_SLAM_data[2] = -l_SLAM_data;
  t_SLAM_data[6] = m_SLAM_data;
  t_SLAM_data[10] = n_SLAM_data;
  t_SLAM_data[14] = o_SLAM_data;
  t_SLAM_data[3] = -p_SLAM_data;
  t_SLAM_data[7] = -q_SLAM_data;
  t_SLAM_data[11] = -r_SLAM_data;
  t_SLAM_data[15] = s_SLAM_data;
  b_dq[0] = dq[0];
  b_dq[1] = dq[1];
  b_dq[2] = dq[2];
  b_dq[3] = dq[3];
  for (i15 = 0; i15 < 4; i15++) {
    u_SLAM_data[i15] = 0.0;
    for (k = 0; k < 4; k++) {
      u_SLAM_data[i15] += t_SLAM_data[i15 + (k << 2)] * b_dq[k];
    }
  }

  for (i15 = 0; i15 < 4; i15++) {
    b_SLAM_data->xt->data[3 + i15] = u_SLAM_data[i15];
  }

  for (i15 = 0; i15 < 3; i15++) {
    for (k = 0; k < 3; k++) {
      c_SLAM_data[i15 + 3 * k] = 0.0;
      for (i16 = 0; i16 < 3; i16++) {
        c_SLAM_data[i15 + 3 * k] += (double)c_a[i15 + 3 * i16] * (I[i16 + 3 * k]
          * c);
      }

      b_y[i15 + 3 * k] = 0.0;
      for (i16 = 0; i16 < 3; i16++) {
        b_y[i15 + 3 * k] += y[i15 + 3 * i16] * Phi[k + 3 * i16];
      }
    }

    for (k = 0; k < 3; k++) {
      b_a[i15 + 3 * k] = 0.0;
      for (i16 = 0; i16 < 3; i16++) {
        b_a[i15 + 3 * k] += c_SLAM_data[i15 + 3 * i16] * (double)c_a[i16 + 3 * k];
      }
    }
  }

  for (i15 = 0; i15 < 3; i15++) {
    for (k = 0; k < 3; k++) {
      b_SLAM_data->P->data[(k + b_SLAM_data->P->size[0] * (3 + i15)) + 3] =
        b_y[k + 3 * i15] + b_a[k + 3 * i15];
    }
  }
}

//
// File trailer for Att_pred.cpp
//
// [EOF]
//
