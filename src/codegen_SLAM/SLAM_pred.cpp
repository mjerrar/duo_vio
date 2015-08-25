//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 25-Aug-2015 17:57:02
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_pred.h"
#include "SLAM_emxutil.h"
#include "norm.h"
#include "SLAM_updIT.h"
#include "eye.h"
#include "diag.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Function Declarations
static void b_dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T
  *x, const double P_xx_apr[144], const emxArray_real_T *Phi, const double Q[81],
  const double control_input[4], emxArray_real_T *x_dot, double P_xx_apr_dot[144],
  emxArray_real_T *Phi_dot);
static void dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T *x,
                      const emxArray_real_T *P_xx_apr, const emxArray_real_T
                      *Phi, const double Q[81], const double control_input[4],
                      emxArray_real_T *x_dot, double P_xx_apr_dot[144],
                      emxArray_real_T *Phi_dot);

// Function Definitions

//
// Arguments    : double dt
//                const double meas[6]
//                const emxArray_real_T *x
//                const double P_xx_apr[144]
//                const emxArray_real_T *Phi
//                const double Q[81]
//                const double control_input[4]
//                emxArray_real_T *x_dot
//                double P_xx_apr_dot[144]
//                emxArray_real_T *Phi_dot
// Return Type  : void
//
static void b_dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T
  *x, const double P_xx_apr[144], const emxArray_real_T *Phi, const double Q[81],
  const double control_input[4], emxArray_real_T *x_dot, double P_xx_apr_dot[144],
  emxArray_real_T *Phi_dot)
{
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double R_cw[9];
  double dv16[3];
  int br;
  double w[3];
  int loop_ub;
  double dv17[9];
  double dv18[9];
  double dv19[9];
  int cr;
  static const signed char iv5[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double F[144];
  double G[108];
  static const signed char iv6[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv7[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  unsigned int unnamed_idx_0;
  double dv20[9];
  double dv21[16];
  double dv22[16];
  double b_x[4];
  double dv23[4];
  double b_G[108];
  double FP[144];
  double b_FP[144];
  int ic;
  int ar;
  int ib;
  int ia;

  //  drone mass
  // linearly interpolate between previous and current measurement to get
  // to the corresponding part inside dt (for Runge-Kutta)
  // za = prev_za + (za - prev_za) * part ;
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  a = x->data[3];
  b_a = x->data[4];
  c_a = x->data[5];
  d_a = x->data[6];
  e_a = x->data[3];
  f_a = x->data[4];
  g_a = x->data[5];
  h_a = x->data[6];
  i_a = x->data[3];
  j_a = x->data[4];
  k_a = x->data[5];
  l_a = x->data[6];
  R_cw[0] = ((a * a - b_a * b_a) - c_a * c_a) + d_a * d_a;
  R_cw[3] = 2.0 * (x->data[3] * x->data[4] + x->data[5] * x->data[6]);
  R_cw[6] = 2.0 * (x->data[3] * x->data[5] - x->data[4] * x->data[6]);
  R_cw[1] = 2.0 * (x->data[3] * x->data[4] - x->data[5] * x->data[6]);
  R_cw[4] = ((-(e_a * e_a) + f_a * f_a) - g_a * g_a) + h_a * h_a;
  R_cw[7] = 2.0 * (x->data[4] * x->data[5] + x->data[3] * x->data[6]);
  R_cw[2] = 2.0 * (x->data[3] * x->data[5] + x->data[4] * x->data[6]);
  R_cw[5] = 2.0 * (x->data[4] * x->data[5] - x->data[3] * x->data[6]);
  R_cw[8] = ((-(i_a * i_a) - j_a * j_a) + k_a * k_a) + l_a * l_a;

  // oa=x(14:16)
  //  yaw rate in control frame
  dv16[0] = 0.0;
  dv16[1] = 0.0;
  dv16[2] = control_input[3];
  for (br = 0; br < 3; br++) {
    w[br] = 0.0;
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      w[br] += R_bc[loop_ub + 3 * br] * dv16[loop_ub];
    }
  }

  //  yaw rate in body frame
  dv17[0] = 0.0;
  dv17[3] = -w[2];
  dv17[6] = w[1];
  dv17[1] = w[2];
  dv17[4] = 0.0;
  dv17[7] = -w[0];
  dv17[2] = -w[1];
  dv17[5] = w[0];
  dv17[8] = 0.0;
  dv18[0] = 0.0;
  dv18[3] = -meas[5];
  dv18[6] = meas[4];
  dv18[1] = meas[5];
  dv18[4] = 0.0;
  dv18[7] = -meas[3];
  dv18[2] = -meas[4];
  dv18[5] = meas[3];
  dv18[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      dv19[br + 3 * loop_ub] = 0.0;
      for (cr = 0; cr < 3; cr++) {
        dv19[br + 3 * loop_ub] += -0.0 * R_cw[cr + 3 * br] * dv18[cr + 3 *
          loop_ub];
      }
    }
  }

  for (br = 0; br < 12; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[loop_ub + 12 * br] = iv5[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * br) + 3] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 3)) + 3] = -dv17[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 6)) + 3] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 9)) + 3] = -0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * br) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 3)) + 6] = dv19[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 6)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 9)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 12; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * br) + 9] = 0.0;
    }
  }

  for (br = 0; br < 9; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      G[loop_ub + 12 * br] = 0.0;
    }

    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      G[(loop_ub + 12 * br) + 3] = iv6[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      G[(loop_ub + 12 * br) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      G[(loop_ub + 12 * (br + 3)) + 6] = -R_cw[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      G[(loop_ub + 12 * (br + 6)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 9; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      G[(loop_ub + 12 * br) + 9] = iv7[loop_ub + 3 * br];
    }
  }

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)x->size[0];
  br = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, br, (int)sizeof(double));
  loop_ub = (int)unnamed_idx_0;
  for (br = 0; br < loop_ub; br++) {
    x_dot->data[br] = 0.0;
  }

  for (br = 0; br < 3; br++) {
    x_dot->data[br] = x->data[7 + br];
  }

  //  position
  dv20[0] = 0.0;
  dv20[3] = -w[2];
  dv20[6] = w[1];
  dv20[1] = w[2];
  dv20[4] = 0.0;
  dv20[7] = -w[0];
  dv20[2] = -w[1];
  dv20[5] = w[0];
  dv20[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      dv21[loop_ub + (br << 2)] = -dv20[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    dv21[12 + br] = w[br];
  }

  for (br = 0; br < 3; br++) {
    dv21[3 + (br << 2)] = -w[br];
  }

  dv21[15] = 0.0;
  for (br = 0; br < 4; br++) {
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      dv22[loop_ub + (br << 2)] = 0.5 * dv21[loop_ub + (br << 2)];
    }
  }

  for (br = 0; br < 4; br++) {
    b_x[br] = x->data[3 + br];
  }

  for (br = 0; br < 4; br++) {
    dv23[br] = 0.0;
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      dv23[br] += dv22[br + (loop_ub << 2)] * b_x[loop_ub];
    }
  }

  for (br = 0; br < 4; br++) {
    x_dot->data[3 + br] = dv23[br];
  }

  //  rot angle
  for (br = 0; br < 3; br++) {
    x_dot->data[7 + br] = control_input[br] / 0.5;
  }

  //  NOTE: The control froces are already in world frame
  for (br = 0; br < 3; br++) {
    x_dot->data[10 + br] = 0.0;
  }

  //  gyro bias
  br = x_dot->size[0];
  emxEnsureCapacity((emxArray__common *)x_dot, br, (int)sizeof(double));
  loop_ub = x_dot->size[0];
  for (br = 0; br < loop_ub; br++) {
    x_dot->data[br] *= dt;
  }

  // time derivative of the covariance
  for (br = 0; br < 12; br++) {
    for (loop_ub = 0; loop_ub < 12; loop_ub++) {
      FP[br + 12 * loop_ub] = 0.0;
      for (cr = 0; cr < 12; cr++) {
        FP[br + 12 * loop_ub] += F[br + 12 * cr] * P_xx_apr[cr + 12 * loop_ub];
      }
    }

    for (loop_ub = 0; loop_ub < 9; loop_ub++) {
      b_G[br + 12 * loop_ub] = 0.0;
      for (cr = 0; cr < 9; cr++) {
        b_G[br + 12 * loop_ub] += G[br + 12 * cr] * Q[cr + 9 * loop_ub];
      }
    }
  }

  for (br = 0; br < 12; br++) {
    for (loop_ub = 0; loop_ub < 12; loop_ub++) {
      a = 0.0;
      for (cr = 0; cr < 9; cr++) {
        a += b_G[br + 12 * cr] * G[loop_ub + 12 * cr];
      }

      b_FP[br + 12 * loop_ub] = (FP[br + 12 * loop_ub] + FP[loop_ub + 12 * br])
        + a;
    }
  }

  for (br = 0; br < 12; br++) {
    for (loop_ub = 0; loop_ub < 12; loop_ub++) {
      P_xx_apr_dot[loop_ub + 12 * br] = b_FP[loop_ub + 12 * br] * dt;
    }
  }

  // time derivative of the state transition
  unnamed_idx_0 = (unsigned int)Phi->size[1];
  br = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[0] = 12;
  emxEnsureCapacity((emxArray__common *)Phi_dot, br, (int)sizeof(double));
  br = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[1] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)Phi_dot, br, (int)sizeof(double));
  loop_ub = 12 * (int)unnamed_idx_0;
  for (br = 0; br < loop_ub; br++) {
    Phi_dot->data[br] = 0.0;
  }

  if (Phi->size[1] == 0) {
  } else {
    loop_ub = 12 * (Phi->size[1] - 1);
    for (cr = 0; cr <= loop_ub; cr += 12) {
      for (ic = cr; ic + 1 <= cr + 12; ic++) {
        Phi_dot->data[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= loop_ub; cr += 12) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 12; ib++) {
        if (Phi->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 12; ic++) {
            ia++;
            Phi_dot->data[ic] += Phi->data[ib] * F[ia - 1];
          }
        }

        ar += 12;
      }

      br += 12;
    }
  }

  br = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[0] = 12;
  emxEnsureCapacity((emxArray__common *)Phi_dot, br, (int)sizeof(double));
  loop_ub = Phi_dot->size[0];
  cr = Phi_dot->size[1];
  loop_ub *= cr;
  for (br = 0; br < loop_ub; br++) {
    Phi_dot->data[br] *= dt;
  }
}

//
// Arguments    : double dt
//                const double meas[6]
//                const emxArray_real_T *x
//                const emxArray_real_T *P_xx_apr
//                const emxArray_real_T *Phi
//                const double Q[81]
//                const double control_input[4]
//                emxArray_real_T *x_dot
//                double P_xx_apr_dot[144]
//                emxArray_real_T *Phi_dot
// Return Type  : void
//
static void dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T *x,
                      const emxArray_real_T *P_xx_apr, const emxArray_real_T
                      *Phi, const double Q[81], const double control_input[4],
                      emxArray_real_T *x_dot, double P_xx_apr_dot[144],
                      emxArray_real_T *Phi_dot)
{
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double R_cw[9];
  double dv8[3];
  int c;
  double w[3];
  int br;
  double dv9[9];
  double dv10[9];
  double dv11[9];
  int ar;
  static const signed char iv2[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double F[144];
  double G[108];
  static const signed char iv3[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv4[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  unsigned int unnamed_idx_0;
  int cr;
  double dv12[9];
  double dv13[16];
  double dv14[16];
  double b_x[4];
  double dv15[4];
  emxArray_real_T *FP;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *b_FP;
  double b_G[108];
  double c_FP[144];

  //  drone mass
  // linearly interpolate between previous and current measurement to get
  // to the corresponding part inside dt (for Runge-Kutta)
  // za = prev_za + (za - prev_za) * part ;
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  a = x->data[3];
  b_a = x->data[4];
  c_a = x->data[5];
  d_a = x->data[6];
  e_a = x->data[3];
  f_a = x->data[4];
  g_a = x->data[5];
  h_a = x->data[6];
  i_a = x->data[3];
  j_a = x->data[4];
  k_a = x->data[5];
  l_a = x->data[6];
  R_cw[0] = ((a * a - b_a * b_a) - c_a * c_a) + d_a * d_a;
  R_cw[3] = 2.0 * (x->data[3] * x->data[4] + x->data[5] * x->data[6]);
  R_cw[6] = 2.0 * (x->data[3] * x->data[5] - x->data[4] * x->data[6]);
  R_cw[1] = 2.0 * (x->data[3] * x->data[4] - x->data[5] * x->data[6]);
  R_cw[4] = ((-(e_a * e_a) + f_a * f_a) - g_a * g_a) + h_a * h_a;
  R_cw[7] = 2.0 * (x->data[4] * x->data[5] + x->data[3] * x->data[6]);
  R_cw[2] = 2.0 * (x->data[3] * x->data[5] + x->data[4] * x->data[6]);
  R_cw[5] = 2.0 * (x->data[4] * x->data[5] - x->data[3] * x->data[6]);
  R_cw[8] = ((-(i_a * i_a) - j_a * j_a) + k_a * k_a) + l_a * l_a;

  // oa=x(14:16)
  //  yaw rate in control frame
  dv8[0] = 0.0;
  dv8[1] = 0.0;
  dv8[2] = control_input[3];
  for (c = 0; c < 3; c++) {
    w[c] = 0.0;
    for (br = 0; br < 3; br++) {
      w[c] += R_bc[br + 3 * c] * dv8[br];
    }
  }

  //  yaw rate in body frame
  dv9[0] = 0.0;
  dv9[3] = -w[2];
  dv9[6] = w[1];
  dv9[1] = w[2];
  dv9[4] = 0.0;
  dv9[7] = -w[0];
  dv9[2] = -w[1];
  dv9[5] = w[0];
  dv9[8] = 0.0;
  dv10[0] = 0.0;
  dv10[3] = -meas[5];
  dv10[6] = meas[4];
  dv10[1] = meas[5];
  dv10[4] = 0.0;
  dv10[7] = -meas[3];
  dv10[2] = -meas[4];
  dv10[5] = meas[3];
  dv10[8] = 0.0;
  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      dv11[c + 3 * br] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        dv11[c + 3 * br] += -0.0 * R_cw[ar + 3 * c] * dv10[ar + 3 * br];
      }
    }
  }

  for (c = 0; c < 12; c++) {
    for (br = 0; br < 3; br++) {
      F[br + 12 * c] = iv2[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * c) + 3] = 0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 3)) + 3] = -dv9[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 6)) + 3] = 0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 9)) + 3] = -0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * c) + 6] = 0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 3)) + 6] = dv11[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 6)) + 6] = 0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 9)) + 6] = 0.0;
    }
  }

  for (c = 0; c < 12; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * c) + 9] = 0.0;
    }
  }

  for (c = 0; c < 9; c++) {
    for (br = 0; br < 3; br++) {
      G[br + 12 * c] = 0.0;
    }

    for (br = 0; br < 3; br++) {
      G[(br + 12 * c) + 3] = iv3[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * c) + 6] = 0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * (c + 3)) + 6] = -R_cw[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * (c + 6)) + 6] = 0.0;
    }
  }

  for (c = 0; c < 9; c++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * c) + 9] = iv4[br + 3 * c];
    }
  }

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)x->size[0];
  c = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, c, (int)sizeof(double));
  cr = (int)unnamed_idx_0;
  for (c = 0; c < cr; c++) {
    x_dot->data[c] = 0.0;
  }

  for (c = 0; c < 3; c++) {
    x_dot->data[c] = x->data[7 + c];
  }

  //  position
  dv12[0] = 0.0;
  dv12[3] = -w[2];
  dv12[6] = w[1];
  dv12[1] = w[2];
  dv12[4] = 0.0;
  dv12[7] = -w[0];
  dv12[2] = -w[1];
  dv12[5] = w[0];
  dv12[8] = 0.0;
  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      dv13[br + (c << 2)] = -dv12[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    dv13[12 + c] = w[c];
  }

  for (c = 0; c < 3; c++) {
    dv13[3 + (c << 2)] = -w[c];
  }

  dv13[15] = 0.0;
  for (c = 0; c < 4; c++) {
    for (br = 0; br < 4; br++) {
      dv14[br + (c << 2)] = 0.5 * dv13[br + (c << 2)];
    }
  }

  for (c = 0; c < 4; c++) {
    b_x[c] = x->data[3 + c];
  }

  for (c = 0; c < 4; c++) {
    dv15[c] = 0.0;
    for (br = 0; br < 4; br++) {
      dv15[c] += dv14[c + (br << 2)] * b_x[br];
    }
  }

  for (c = 0; c < 4; c++) {
    x_dot->data[3 + c] = dv15[c];
  }

  //  rot angle
  for (c = 0; c < 3; c++) {
    x_dot->data[7 + c] = control_input[c] / 0.5;
  }

  //  NOTE: The control froces are already in world frame
  for (c = 0; c < 3; c++) {
    x_dot->data[10 + c] = 0.0;
  }

  //  gyro bias
  c = x_dot->size[0];
  emxEnsureCapacity((emxArray__common *)x_dot, c, (int)sizeof(double));
  cr = x_dot->size[0];
  for (c = 0; c < cr; c++) {
    x_dot->data[c] *= dt;
  }

  // time derivative of the covariance
  emxInit_real_T(&FP, 2);
  if (P_xx_apr->size[0] == 1) {
    c = FP->size[0] * FP->size[1];
    FP->size[0] = 12;
    FP->size[1] = P_xx_apr->size[1];
    emxEnsureCapacity((emxArray__common *)FP, c, (int)sizeof(double));
    for (c = 0; c < 12; c++) {
      cr = P_xx_apr->size[1];
      for (br = 0; br < cr; br++) {
        FP->data[c + FP->size[0] * br] = 0.0;
        for (ar = 0; ar < 12; ar++) {
          FP->data[c + FP->size[0] * br] += F[c + 12 * ar] * P_xx_apr->data[ar +
            P_xx_apr->size[0] * br];
        }
      }
    }
  } else {
    unnamed_idx_0 = (unsigned int)P_xx_apr->size[1];
    c = FP->size[0] * FP->size[1];
    FP->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)FP, c, (int)sizeof(double));
    c = FP->size[0] * FP->size[1];
    FP->size[1] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)FP, c, (int)sizeof(double));
    cr = 12 * (int)unnamed_idx_0;
    for (c = 0; c < cr; c++) {
      FP->data[c] = 0.0;
    }

    if (P_xx_apr->size[1] == 0) {
    } else {
      c = 12 * (P_xx_apr->size[1] - 1);
      for (cr = 0; cr <= c; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          FP->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= c; cr += 12) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_xx_apr->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              FP->data[ic] += P_xx_apr->data[ib] * F[ia - 1];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  emxInit_real_T(&b_FP, 2);
  c = b_FP->size[0] * b_FP->size[1];
  b_FP->size[0] = FP->size[1];
  b_FP->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_FP, c, (int)sizeof(double));
  for (c = 0; c < 12; c++) {
    cr = FP->size[1];
    for (br = 0; br < cr; br++) {
      b_FP->data[br + b_FP->size[0] * c] = FP->data[c + FP->size[0] * br];
    }
  }

  for (c = 0; c < 12; c++) {
    for (br = 0; br < 9; br++) {
      b_G[c + 12 * br] = 0.0;
      for (ar = 0; ar < 9; ar++) {
        b_G[c + 12 * br] += G[c + 12 * ar] * Q[ar + 9 * br];
      }
    }
  }

  for (c = 0; c < 12; c++) {
    for (br = 0; br < 12; br++) {
      a = 0.0;
      for (ar = 0; ar < 9; ar++) {
        a += b_G[c + 12 * ar] * G[br + 12 * ar];
      }

      c_FP[c + 12 * br] = (FP->data[c + 12 * br] + b_FP->data[c + 12 * br]) + a;
    }
  }

  emxFree_real_T(&b_FP);
  emxFree_real_T(&FP);
  for (c = 0; c < 12; c++) {
    for (br = 0; br < 12; br++) {
      P_xx_apr_dot[br + 12 * c] = c_FP[br + 12 * c] * dt;
    }
  }

  // time derivative of the state transition
  if (Phi->size[0] == 1) {
    c = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[0] = 12;
    Phi_dot->size[1] = Phi->size[1];
    emxEnsureCapacity((emxArray__common *)Phi_dot, c, (int)sizeof(double));
    for (c = 0; c < 12; c++) {
      cr = Phi->size[1];
      for (br = 0; br < cr; br++) {
        Phi_dot->data[c + Phi_dot->size[0] * br] = 0.0;
        for (ar = 0; ar < 12; ar++) {
          Phi_dot->data[c + Phi_dot->size[0] * br] += F[c + 12 * ar] * Phi->
            data[ar + Phi->size[0] * br];
        }
      }
    }
  } else {
    unnamed_idx_0 = (unsigned int)Phi->size[1];
    c = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)Phi_dot, c, (int)sizeof(double));
    c = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[1] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)Phi_dot, c, (int)sizeof(double));
    cr = 12 * (int)unnamed_idx_0;
    for (c = 0; c < cr; c++) {
      Phi_dot->data[c] = 0.0;
    }

    if (Phi->size[1] == 0) {
    } else {
      c = 12 * (Phi->size[1] - 1);
      for (cr = 0; cr <= c; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          Phi_dot->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= c; cr += 12) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (Phi->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              Phi_dot->data[ic] += Phi->data[ib] * F[ia - 1];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  c = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[0] = 12;
  emxEnsureCapacity((emxArray__common *)Phi_dot, c, (int)sizeof(double));
  c = Phi_dot->size[0];
  cr = Phi_dot->size[1];
  cr *= c;
  for (c = 0; c < cr; c++) {
    Phi_dot->data[c] *= dt;
  }
}

//
// EKF_SLAM: computes the camerapose p and feature position f in an
// interative way
//    Handels a static number of points but can dynamically asign them to new
//    klt points.
// Arguments    : emxArray_real_T *P_apo
//                emxArray_real_T *x
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[23]
//                double c_numStates
//                const double control_input[4]
// Return Type  : void
//
void SLAM_pred(emxArray_real_T *P_apo, emxArray_real_T *x, double dt, const
               double processNoise[4], const double IMU_measurements[23], double
               c_numStates, const double control_input[4])
{
  double c;
  double b_processNoise[9];
  double c_processNoise[9];
  int i24;
  double Q[81];
  int loop_ub;
  int i;
  emxArray_real_T *P_xx_apr;
  int i25;
  emxArray_real_T *Phi;
  double meas_0[6];
  emxArray_real_T *b_x;
  emxArray_real_T *x1;
  emxArray_real_T *b_Phi;
  double b_P_xx_apr[144];
  double u_1[4];
  double meas_1[6];
  emxArray_real_T *xx;
  double b_xx[4];
  double c_P_xx_apr[144];
  emxArray_real_T *c_Phi;
  emxArray_real_T *P_xs_apr;
  emxArray_real_T *x2;
  double P2[144];
  emxArray_real_T *d_Phi;
  emxArray_real_T *x3;
  emxArray_real_T *Phi3;
  double P3[144];
  double b_meas_1[6];
  emxArray_real_T *e_Phi;
  emxArray_real_T *x4;
  emxArray_real_T *Phi4;
  double P4[144];
  emxArray_int32_T *r14;
  emxArray_real_T *c_x;
  double d_P_xx_apr;
  emxArray_real_T *b_P_apo;
  int cr;
  int k;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  emxArray_int32_T *r15;
  emxArray_int32_T *r16;
  emxArray_real_T *b_P_xs_apr;

  //  Xv meaning
  //
  //                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
  //  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
  //  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  //  numStatesFeatures=numAnchors*(7+numPointsPerAnchor);
  //  %% Iterative Camera Pose optimization (EKF)
  //  numStates=22;
  // % compute the linearization F of the non linear model f
  c = dt * dt;
  b_processNoise[0] = processNoise[1];
  b_processNoise[1] = processNoise[1];
  b_processNoise[2] = processNoise[1];
  b_processNoise[3] = processNoise[0];
  b_processNoise[4] = processNoise[0];
  b_processNoise[5] = processNoise[0];
  b_processNoise[6] = processNoise[2];
  b_processNoise[7] = processNoise[2];
  b_processNoise[8] = processNoise[2];
  for (i24 = 0; i24 < 9; i24++) {
    c_processNoise[i24] = b_processNoise[i24] * c;
  }

  diag(c_processNoise, Q);
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    i = 0;
  } else {
    i = (int)c_numStates;
  }

  emxInit_real_T(&P_xx_apr, 2);
  i24 = P_xx_apr->size[0] * P_xx_apr->size[1];
  P_xx_apr->size[0] = loop_ub;
  P_xx_apr->size[1] = i;
  emxEnsureCapacity((emxArray__common *)P_xx_apr, i24, (int)sizeof(double));
  for (i24 = 0; i24 < i; i24++) {
    for (i25 = 0; i25 < loop_ub; i25++) {
      P_xx_apr->data[i25 + P_xx_apr->size[0] * i24] = P_apo->data[i25 +
        P_apo->size[0] * i24];
    }
  }

  emxInit_real_T(&Phi, 2);
  eye(c_numStates, Phi);
  for (i = 0; i < 3; i++) {
    meas_0[i] = IMU_measurements[i];
  }

  for (i = 0; i < 3; i++) {
    meas_0[i + 3] = 0.0 * IMU_measurements[i + 3];
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&b_x, 1);
  i24 = b_x->size[0];
  b_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)b_x, i24, (int)sizeof(double));
  for (i24 = 0; i24 < loop_ub; i24++) {
    b_x->data[i24] = x->data[i24];
  }

  b_emxInit_real_T(&x1, 1);
  emxInit_real_T(&b_Phi, 2);
  dxdt_dPdt(dt, meas_0, b_x, P_xx_apr, Phi, Q, control_input, x1, b_P_xx_apr,
            b_Phi);
  emxFree_real_T(&b_x);
  for (i = 0; i < 4; i++) {
    u_1[i] = control_input[i] + (control_input[i] - control_input[i]) * 0.5;
  }

  for (i = 0; i < 6; i++) {
    meas_1[i] = meas_0[i] + (meas_0[i] - meas_0[i]) * 0.5;
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&xx, 1);
  i24 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i24, (int)sizeof(double));
  for (i24 = 0; i24 < loop_ub; i24++) {
    xx->data[i24] = x->data[i24] + x1->data[i24] / 2.0;
  }

  for (i24 = 0; i24 < 4; i24++) {
    b_xx[i24] = xx->data[3 + i24];
  }

  c = b_norm(b_xx);
  for (i24 = 0; i24 < 4; i24++) {
    b_xx[i24] = xx->data[3 + i24] / c;
  }

  for (i24 = 0; i24 < 4; i24++) {
    xx->data[3 + i24] = b_xx[i24];
  }

  for (i24 = 0; i24 < 144; i24++) {
    c_P_xx_apr[i24] = P_xx_apr->data[i24] + b_P_xx_apr[i24] / 2.0;
  }

  emxInit_real_T(&c_Phi, 2);
  i24 = c_Phi->size[0] * c_Phi->size[1];
  c_Phi->size[0] = Phi->size[0];
  c_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)c_Phi, i24, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i24 = 0; i24 < loop_ub; i24++) {
    c_Phi->data[i24] = Phi->data[i24] + b_Phi->data[i24] / 2.0;
  }

  emxInit_real_T(&P_xs_apr, 2);
  b_emxInit_real_T(&x2, 1);
  b_dxdt_dPdt(dt, meas_1, xx, c_P_xx_apr, c_Phi, Q, u_1, x2, P2, P_xs_apr);
  emxFree_real_T(&c_Phi);
  for (i24 = 0; i24 < 4; i24++) {
    u_1[i24] += (control_input[i24] - u_1[i24]) * 0.5;
  }

  for (i24 = 0; i24 < 6; i24++) {
    meas_1[i24] += (meas_0[i24] - meas_1[i24]) * 0.5;
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i24 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i24, (int)sizeof(double));
  for (i24 = 0; i24 < loop_ub; i24++) {
    xx->data[i24] = x->data[i24] + x2->data[i24] / 2.0;
  }

  for (i24 = 0; i24 < 4; i24++) {
    b_xx[i24] = xx->data[3 + i24];
  }

  c = b_norm(b_xx);
  for (i24 = 0; i24 < 4; i24++) {
    b_xx[i24] = xx->data[3 + i24] / c;
  }

  for (i24 = 0; i24 < 4; i24++) {
    xx->data[3 + i24] = b_xx[i24];
  }

  for (i24 = 0; i24 < 144; i24++) {
    c_P_xx_apr[i24] = P_xx_apr->data[i24] + P2[i24] / 2.0;
  }

  emxInit_real_T(&d_Phi, 2);
  i24 = d_Phi->size[0] * d_Phi->size[1];
  d_Phi->size[0] = Phi->size[0];
  d_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)d_Phi, i24, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i24 = 0; i24 < loop_ub; i24++) {
    d_Phi->data[i24] = Phi->data[i24] + P_xs_apr->data[i24] / 2.0;
  }

  b_emxInit_real_T(&x3, 1);
  emxInit_real_T(&Phi3, 2);
  b_dxdt_dPdt(dt, meas_1, xx, c_P_xx_apr, d_Phi, Q, u_1, x3, P3, Phi3);
  emxFree_real_T(&d_Phi);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i24 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i24, (int)sizeof(double));
  for (i24 = 0; i24 < loop_ub; i24++) {
    xx->data[i24] = x->data[i24] + x3->data[i24];
  }

  for (i24 = 0; i24 < 4; i24++) {
    b_xx[i24] = xx->data[3 + i24];
  }

  c = b_norm(b_xx);
  for (i24 = 0; i24 < 4; i24++) {
    b_xx[i24] = xx->data[3 + i24] / c;
  }

  for (i24 = 0; i24 < 4; i24++) {
    xx->data[3 + i24] = b_xx[i24];
  }

  for (i = 0; i < 6; i++) {
    b_meas_1[i] = meas_1[i] + (meas_0[i] - meas_1[i]);
  }

  for (i24 = 0; i24 < 144; i24++) {
    c_P_xx_apr[i24] = P_xx_apr->data[i24] + P3[i24];
  }

  emxInit_real_T(&e_Phi, 2);
  i24 = e_Phi->size[0] * e_Phi->size[1];
  e_Phi->size[0] = Phi->size[0];
  e_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)e_Phi, i24, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i24 = 0; i24 < loop_ub; i24++) {
    e_Phi->data[i24] = Phi->data[i24] + Phi3->data[i24];
  }

  for (i = 0; i < 4; i++) {
    b_xx[i] = u_1[i] + (control_input[i] - u_1[i]);
  }

  b_emxInit_real_T(&x4, 1);
  emxInit_real_T(&Phi4, 2);
  b_dxdt_dPdt(dt, b_meas_1, xx, c_P_xx_apr, e_Phi, Q, b_xx, x4, P4, Phi4);
  emxFree_real_T(&e_Phi);
  emxFree_real_T(&xx);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  if (1.0 > c_numStates + 1.0) {
    i = 0;
  } else {
    i = (int)(c_numStates + 1.0);
  }

  b_emxInit_int32_T(&r14, 2);
  i24 = r14->size[0] * r14->size[1];
  r14->size[0] = 1;
  r14->size[1] = i;
  emxEnsureCapacity((emxArray__common *)r14, i24, (int)sizeof(int));
  for (i24 = 0; i24 < i; i24++) {
    r14->data[r14->size[0] * i24] = i24;
  }

  b_emxInit_real_T(&c_x, 1);
  i24 = c_x->size[0];
  c_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_x, i24, (int)sizeof(double));
  for (i24 = 0; i24 < loop_ub; i24++) {
    c_x->data[i24] = x->data[i24] + (((x1->data[i24] + 2.0 * x2->data[i24]) +
      2.0 * x3->data[i24]) + x4->data[i24]) / 6.0;
  }

  emxFree_real_T(&x4);
  emxFree_real_T(&x3);
  emxFree_real_T(&x2);
  emxFree_real_T(&x1);
  loop_ub = r14->size[1];
  for (i24 = 0; i24 < loop_ub; i24++) {
    x->data[r14->data[r14->size[0] * i24]] = c_x->data[(*(int (*)[2])r14->size)
      [0] * i24];
  }

  emxFree_real_T(&c_x);
  emxFree_int32_T(&r14);
  for (i24 = 0; i24 < 144; i24++) {
    d_P_xx_apr = P_xx_apr->data[i24] + (((b_P_xx_apr[i24] + 2.0 * P2[i24]) + 2.0
      * P3[i24]) + P4[i24]) / 6.0;
    b_P_xx_apr[i24] = d_P_xx_apr;
  }

  emxFree_real_T(&P_xx_apr);

  //  covariance of the state
  i24 = b_Phi->size[0] * b_Phi->size[1];
  b_Phi->size[0] = Phi->size[0];
  b_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)b_Phi, i24, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i24 = 0; i24 < loop_ub; i24++) {
    b_Phi->data[i24] = Phi->data[i24] + (((b_Phi->data[i24] + 2.0 *
      P_xs_apr->data[i24]) + 2.0 * Phi3->data[i24]) + Phi4->data[i24]) / 6.0;
  }

  emxFree_real_T(&Phi4);
  emxFree_real_T(&Phi3);
  emxFree_real_T(&Phi);
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i24 = 0;
    i25 = 0;
  } else {
    i24 = (int)(c_numStates + 1.0) - 1;
    i25 = P_apo->size[1];
  }

  emxInit_real_T(&b_P_apo, 2);
  if ((b_Phi->size[1] == 1) || (loop_ub == 1)) {
    cr = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = loop_ub;
    b_P_apo->size[1] = i25 - i24;
    emxEnsureCapacity((emxArray__common *)b_P_apo, cr, (int)sizeof(double));
    i = i25 - i24;
    for (i25 = 0; i25 < i; i25++) {
      for (cr = 0; cr < loop_ub; cr++) {
        b_P_apo->data[cr + b_P_apo->size[0] * i25] = P_apo->data[cr +
          P_apo->size[0] * (i24 + i25)];
      }
    }

    i24 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i24, (int)sizeof(double));
    for (i24 = 0; i24 < 12; i24++) {
      loop_ub = b_P_apo->size[1];
      for (i25 = 0; i25 < loop_ub; i25++) {
        P_xs_apr->data[i24 + P_xs_apr->size[0] * i25] = 0.0;
        i = b_Phi->size[1];
        for (cr = 0; cr < i; cr++) {
          P_xs_apr->data[i24 + P_xs_apr->size[0] * i25] += b_Phi->data[i24 +
            b_Phi->size[0] * cr] * b_P_apo->data[cr + b_P_apo->size[0] * i25];
        }
      }
    }
  } else {
    k = b_Phi->size[1];
    cr = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, cr, (int)sizeof(double));
    cr = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = i25 - i24;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, cr, (int)sizeof(double));
    i = 12 * (i25 - i24);
    for (cr = 0; cr < i; cr++) {
      P_xs_apr->data[cr] = 0.0;
    }

    if (i25 - i24 == 0) {
    } else {
      i = 12 * ((i25 - i24) - 1);
      for (cr = 0; cr <= i; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= i; cr += 12) {
        ar = 0;
        i25 = br + k;
        for (ib = br; ib + 1 <= i25; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * (i24 +
               div_nzp_s32_floor(ib, loop_ub))] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                (i24 + div_nzp_s32_floor(ib, loop_ub))] * b_Phi->data[ia - 1];
            }
          }

          ar += 12;
        }

        br += k;
      }
    }
  }

  emxFree_real_T(&b_P_apo);
  emxFree_real_T(&b_Phi);

  //  covariance between current state and trails
  c = b_norm(*(double (*)[4])&x->data[3]);
  for (i24 = 0; i24 < 4; i24++) {
    b_xx[i24] = x->data[3 + i24] / c;
  }

  for (i24 = 0; i24 < 4; i24++) {
    x->data[3 + i24] = b_xx[i24];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    i = 0;
  } else {
    i = (int)c_numStates;
  }

  emxInit_int32_T(&r15, 1);
  i24 = r15->size[0];
  r15->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r15, i24, (int)sizeof(int));
  for (i24 = 0; i24 < loop_ub; i24++) {
    r15->data[i24] = i24;
  }

  emxInit_int32_T(&r16, 1);
  i24 = r16->size[0];
  r16->size[0] = i;
  emxEnsureCapacity((emxArray__common *)r16, i24, (int)sizeof(int));
  for (i24 = 0; i24 < i; i24++) {
    r16->data[i24] = i24;
  }

  for (i24 = 0; i24 < 12; i24++) {
    for (i25 = 0; i25 < 12; i25++) {
      c_P_xx_apr[i25 + 12 * i24] = (b_P_xx_apr[i25 + 12 * i24] + b_P_xx_apr[i24
        + 12 * i25]) / 2.0;
    }
  }

  i = r15->size[0];
  cr = r16->size[0];
  for (i24 = 0; i24 < cr; i24++) {
    for (i25 = 0; i25 < i; i25++) {
      P_apo->data[r15->data[i25] + P_apo->size[0] * r16->data[i24]] =
        c_P_xx_apr[i25 + i * i24];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i24 = 0;
    i25 = 0;
  } else {
    i24 = (int)(c_numStates + 1.0) - 1;
    i25 = P_apo->size[1];
  }

  cr = r15->size[0];
  r15->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r15, cr, (int)sizeof(int));
  for (cr = 0; cr < loop_ub; cr++) {
    r15->data[cr] = cr;
  }

  cr = r16->size[0];
  r16->size[0] = i25 - i24;
  emxEnsureCapacity((emxArray__common *)r16, cr, (int)sizeof(int));
  loop_ub = i25 - i24;
  for (i25 = 0; i25 < loop_ub; i25++) {
    r16->data[i25] = i24 + i25;
  }

  i = r15->size[0];
  cr = r16->size[0];
  for (i24 = 0; i24 < cr; i24++) {
    for (i25 = 0; i25 < i; i25++) {
      P_apo->data[r15->data[i25] + P_apo->size[0] * r16->data[i24]] =
        P_xs_apr->data[i25 + i * i24];
    }
  }

  if (c_numStates + 1.0 > P_apo->size[0]) {
    i24 = 0;
    i25 = 0;
  } else {
    i24 = (int)(c_numStates + 1.0) - 1;
    i25 = P_apo->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  cr = r15->size[0];
  r15->size[0] = i25 - i24;
  emxEnsureCapacity((emxArray__common *)r15, cr, (int)sizeof(int));
  i = i25 - i24;
  for (i25 = 0; i25 < i; i25++) {
    r15->data[i25] = i24 + i25;
  }

  i24 = r16->size[0];
  r16->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r16, i24, (int)sizeof(int));
  for (i24 = 0; i24 < loop_ub; i24++) {
    r16->data[i24] = i24;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i24 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i24, (int)sizeof(double));
  for (i24 = 0; i24 < 12; i24++) {
    loop_ub = P_xs_apr->size[1];
    for (i25 = 0; i25 < loop_ub; i25++) {
      b_P_xs_apr->data[i25 + b_P_xs_apr->size[0] * i24] = P_xs_apr->data[i24 +
        P_xs_apr->size[0] * i25];
    }
  }

  emxFree_real_T(&P_xs_apr);
  i = r15->size[0];
  cr = r16->size[0];
  for (i24 = 0; i24 < cr; i24++) {
    for (i25 = 0; i25 < i; i25++) {
      P_apo->data[r15->data[i25] + P_apo->size[0] * r16->data[i24]] =
        b_P_xs_apr->data[i25 + i * i24];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r16);
  emxFree_int32_T(&r15);

  //  P_apr=(P_apr+P_apr')/2;
  // % =================================================================================================== 
}

//
// File trailer for SLAM_pred.cpp
//
// [EOF]
//
