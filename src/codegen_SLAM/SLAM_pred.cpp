//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 21:43:27
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_pred.h"
#include "norm.h"
#include "SLAM_emxutil.h"
#include "eye.h"
#include "SLAM_updIT.h"
#include "diag.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>
#include <stdio.h>

// Variable Definitions
static double last_imu[6];

// Function Declarations
static void b_dxdt_dPdt(const double meas[6], const emxArray_real_T *x, const
  double P_xx_apr[144], const emxArray_real_T *Phi, const double Q[81], const
  double control_input[4], emxArray_real_T *x_dot, double P_xx_apr_dot[144],
  emxArray_real_T *Phi_dot);
static void dxdt_dPdt(const double meas[6], const emxArray_real_T *x, const
                      emxArray_real_T *P_xx_apr, const emxArray_real_T *Phi,
                      const double Q[81], const double control_input[4],
                      emxArray_real_T *x_dot, double P_xx_apr_dot[144],
                      emxArray_real_T *Phi_dot);

// Function Definitions

//
// linearly interpolate between previous and current measurement to get
// to the corresponding part inside dt (for Runge-Kutta)
// za = prev_za + (za - prev_za) * part ;
// Arguments    : const double meas[6]
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
static void b_dxdt_dPdt(const double meas[6], const emxArray_real_T *x, const
  double P_xx_apr[144], const emxArray_real_T *Phi, const double Q[81], const
  double control_input[4], emxArray_real_T *x_dot, double P_xx_apr_dot[144],
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
  double w[3];
  int br;
  double dv10[9];
  double dv11[9];
  double dv12[9];
  int loop_ub;
  int cr;
  static const signed char iv8[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double F[144];
  static const signed char iv9[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double G[108];
  static const signed char iv10[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv11[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  unsigned int unnamed_idx_0;
  double dv13[9];
  double dv14[16];
  double dv15[16];
  double b_x[4];
  double dv16[4];
  double b_G[108];
  double FP[144];
  int ic;
  int ar;
  int ib;
  int ia;

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
  //  drone mass
  //  w_b = [0; 0; control_input(4)]; % yaw rate in control frame
  //
  //  w = R_bc' * w_b; % yaw rate in body frame
  for (br = 0; br < 3; br++) {
    w[br] = meas[br] - x->data[10 + br];
  }

  dv10[0] = 0.0;
  dv10[3] = -w[2];
  dv10[6] = w[1];
  dv10[1] = w[2];
  dv10[4] = 0.0;
  dv10[7] = -w[0];
  dv10[2] = -w[1];
  dv10[5] = w[0];
  dv10[8] = 0.0;
  dv11[0] = 0.0;
  dv11[3] = -meas[5];
  dv11[6] = meas[4];
  dv11[1] = meas[5];
  dv11[4] = 0.0;
  dv11[7] = -meas[3];
  dv11[2] = -meas[4];
  dv11[5] = meas[3];
  dv11[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      dv12[br + 3 * loop_ub] = 0.0;
      for (cr = 0; cr < 3; cr++) {
        dv12[br + 3 * loop_ub] += -0.0 * R_cw[cr + 3 * br] * dv11[cr + 3 *
          loop_ub];
      }
    }
  }

  for (br = 0; br < 12; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[loop_ub + 12 * br] = iv8[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * br) + 3] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 3)) + 3] = -dv10[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 6)) + 3] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 9)) + 3] = iv9[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * br) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      F[(loop_ub + 12 * (br + 3)) + 6] = dv12[loop_ub + 3 * br];
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
      G[(loop_ub + 12 * br) + 3] = iv10[loop_ub + 3 * br];
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
      G[(loop_ub + 12 * br) + 9] = iv11[loop_ub + 3 * br];
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
  dv13[0] = 0.0;
  dv13[3] = -w[2];
  dv13[6] = w[1];
  dv13[1] = w[2];
  dv13[4] = 0.0;
  dv13[7] = -w[0];
  dv13[2] = -w[1];
  dv13[5] = w[0];
  dv13[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      dv14[loop_ub + (br << 2)] = -dv13[loop_ub + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    dv14[12 + br] = w[br];
  }

  for (br = 0; br < 3; br++) {
    dv14[3 + (br << 2)] = -w[br];
  }

  dv14[15] = 0.0;
  for (br = 0; br < 4; br++) {
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      dv15[loop_ub + (br << 2)] = 0.5 * dv14[loop_ub + (br << 2)];
    }
  }

  for (br = 0; br < 4; br++) {
    b_x[br] = x->data[3 + br];
  }

  for (br = 0; br < 4; br++) {
    dv16[br] = 0.0;
    for (loop_ub = 0; loop_ub < 4; loop_ub++) {
      dv16[br] += dv15[br + (loop_ub << 2)] * b_x[loop_ub];
    }
  }

  for (br = 0; br < 4; br++) {
    x_dot->data[3 + br] = dv16[br];
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

      P_xx_apr_dot[br + 12 * loop_ub] = (FP[br + 12 * loop_ub] + FP[loop_ub + 12
        * br]) + a;
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
}

//
// linearly interpolate between previous and current measurement to get
// to the corresponding part inside dt (for Runge-Kutta)
// za = prev_za + (za - prev_za) * part ;
// Arguments    : const double meas[6]
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
static void dxdt_dPdt(const double meas[6], const emxArray_real_T *x, const
                      emxArray_real_T *P_xx_apr, const emxArray_real_T *Phi,
                      const double Q[81], const double control_input[4],
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
  double w[3];
  int c;
  double dv3[9];
  double dv4[9];
  double dv5[9];
  int br;
  int ar;
  static const signed char iv4[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double F[144];
  static const signed char iv5[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double G[108];
  static const signed char iv6[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv7[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  unsigned int unnamed_idx_0;
  int cr;
  double dv6[9];
  double dv7[16];
  double dv8[16];
  double b_x[4];
  double dv9[4];
  emxArray_real_T *FP;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *b_FP;
  double b_G[108];

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
  //  drone mass
  //  w_b = [0; 0; control_input(4)]; % yaw rate in control frame
  //
  //  w = R_bc' * w_b; % yaw rate in body frame
  for (c = 0; c < 3; c++) {
    w[c] = meas[c] - x->data[10 + c];
  }

  dv3[0] = 0.0;
  dv3[3] = -w[2];
  dv3[6] = w[1];
  dv3[1] = w[2];
  dv3[4] = 0.0;
  dv3[7] = -w[0];
  dv3[2] = -w[1];
  dv3[5] = w[0];
  dv3[8] = 0.0;
  dv4[0] = 0.0;
  dv4[3] = -meas[5];
  dv4[6] = meas[4];
  dv4[1] = meas[5];
  dv4[4] = 0.0;
  dv4[7] = -meas[3];
  dv4[2] = -meas[4];
  dv4[5] = meas[3];
  dv4[8] = 0.0;
  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      dv5[c + 3 * br] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        dv5[c + 3 * br] += -0.0 * R_cw[ar + 3 * c] * dv4[ar + 3 * br];
      }
    }
  }

  for (c = 0; c < 12; c++) {
    for (br = 0; br < 3; br++) {
      F[br + 12 * c] = iv4[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * c) + 3] = 0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 3)) + 3] = -dv3[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 6)) + 3] = 0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 9)) + 3] = iv5[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * c) + 6] = 0.0;
    }
  }

  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (c + 3)) + 6] = dv5[br + 3 * c];
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
      G[(br + 12 * c) + 3] = iv6[br + 3 * c];
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
      G[(br + 12 * c) + 9] = iv7[br + 3 * c];
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
  dv6[0] = 0.0;
  dv6[3] = -w[2];
  dv6[6] = w[1];
  dv6[1] = w[2];
  dv6[4] = 0.0;
  dv6[7] = -w[0];
  dv6[2] = -w[1];
  dv6[5] = w[0];
  dv6[8] = 0.0;
  for (c = 0; c < 3; c++) {
    for (br = 0; br < 3; br++) {
      dv7[br + (c << 2)] = -dv6[br + 3 * c];
    }
  }

  for (c = 0; c < 3; c++) {
    dv7[12 + c] = w[c];
  }

  for (c = 0; c < 3; c++) {
    dv7[3 + (c << 2)] = -w[c];
  }

  dv7[15] = 0.0;
  for (c = 0; c < 4; c++) {
    for (br = 0; br < 4; br++) {
      dv8[br + (c << 2)] = 0.5 * dv7[br + (c << 2)];
    }
  }

  for (c = 0; c < 4; c++) {
    b_x[c] = x->data[3 + c];
  }

  for (c = 0; c < 4; c++) {
    dv9[c] = 0.0;
    for (br = 0; br < 4; br++) {
      dv9[c] += dv8[c + (br << 2)] * b_x[br];
    }
  }

  for (c = 0; c < 4; c++) {
    x_dot->data[3 + c] = dv9[c];
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

      P_xx_apr_dot[c + 12 * br] = (FP->data[c + 12 * br] + b_FP->data[c + 12 *
        br]) + a;
    }
  }

  emxFree_real_T(&b_FP);
  emxFree_real_T(&FP);

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
}

//
// EKF_SLAM: computes the camerapose p and feature position f in an
// interative way
//    Handels a static number of points but can dynamically asign them to new
//    klt points.
//  Xv meaning
//
//                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
//  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
//  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
// Arguments    : const emxArray_real_T *P_apo
//                emxArray_real_T *x
//                double dt
//                const double processNoise[4]
//                const double measurements_gyr_duo[3]
//                const double measurements_acc_duo[3]
//                double c_numStates
//                const double control_input[4]
//                emxArray_real_T *P_apr
// Return Type  : void
//
void SLAM_pred(const emxArray_real_T *P_apo, emxArray_real_T *x, double dt,
               const double processNoise[4], const double measurements_gyr_duo[3],
               const double measurements_acc_duo[3], double c_numStates, const
               double control_input[4], emxArray_real_T *P_apr)
{
  int k;
  double current_imu[6];
  double w[3];
  double b_processNoise[9];
  double Q[81];
  double b_x[9];
  int i26;
  int cr;
  double G[108];
  static const signed char iv13[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv14[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double P_xx_apr[144];
  double dv23[9];
  double b_G[144];
  static const signed char iv15[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv16[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double Phi[144];
  int loop_ub;
  int b_loop_ub;
  emxArray_real_T *P_xs_apr;
  emxArray_real_T *b_P_apo;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  double y[144];
  double c;
  double c_G[108];
  emxArray_real_T *c_P_apo;
  emxArray_int32_T *r17;
  emxArray_int32_T *r18;
  emxArray_real_T *b_P_xs_apr;
  emxArray_real_T *b_P_xx_apr;
  emxArray_real_T *b_Phi;
  emxArray_real_T *c_x;
  emxArray_real_T *x1;
  double u_1[4];
  emxArray_real_T *xx;
  double b_xx[4];
  double dv24[6];
  emxArray_real_T *c_Phi;
  emxArray_real_T *x2;
  emxArray_real_T *d_Phi;
  emxArray_real_T *x3;
  emxArray_real_T *e_Phi;
  emxArray_real_T *x4;
  emxArray_int32_T *r19;
  emxArray_real_T *d_x;

  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  //  numStatesFeatures=numAnchors*(7+numPointsPerAnchor);
  //  %% Iterative Camera Pose optimization (EKF)
  for (k = 0; k < 3; k++) {
    current_imu[k] = measurements_gyr_duo[k];
  }

  for (k = 0; k < 3; k++) {
    current_imu[k + 3] = 0.0 * measurements_acc_duo[k];

    // % compute the linearization F of the non linear model f
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    w[k] = measurements_gyr_duo[k] - x->data[10 + k];
  }

  b_processNoise[0] = processNoise[1];
  b_processNoise[1] = processNoise[1];
  b_processNoise[2] = processNoise[1];
  b_processNoise[3] = processNoise[0];
  b_processNoise[4] = processNoise[0];
  b_processNoise[5] = processNoise[0];
  b_processNoise[6] = processNoise[2];
  b_processNoise[7] = processNoise[2];
  b_processNoise[8] = processNoise[2];
  diag(b_processNoise, Q);
  b_x[0] = ((x->data[3] * x->data[3] - x->data[4] * x->data[4]) - x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
  b_x[3] = 2.0 * (x->data[3] * x->data[4] + x->data[5] * x->data[6]);
  b_x[6] = 2.0 * (x->data[3] * x->data[5] - x->data[4] * x->data[6]);
  b_x[1] = 2.0 * (x->data[3] * x->data[4] - x->data[5] * x->data[6]);
  b_x[4] = ((-(x->data[3] * x->data[3]) + x->data[4] * x->data[4]) - x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
  b_x[7] = 2.0 * (x->data[4] * x->data[5] + x->data[3] * x->data[6]);
  b_x[2] = 2.0 * (x->data[3] * x->data[5] + x->data[4] * x->data[6]);
  b_x[5] = 2.0 * (x->data[4] * x->data[5] - x->data[3] * x->data[6]);
  b_x[8] = ((-(x->data[3] * x->data[3]) - x->data[4] * x->data[4]) + x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
  for (i26 = 0; i26 < 9; i26++) {
    for (cr = 0; cr < 3; cr++) {
      G[cr + 12 * i26] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * i26) + 3] = iv13[cr + 3 * i26];
    }
  }

  for (i26 = 0; i26 < 3; i26++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * i26) + 6] = 0.0;
    }
  }

  for (i26 = 0; i26 < 3; i26++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * (i26 + 3)) + 6] = -b_x[cr + 3 * i26];
    }
  }

  for (i26 = 0; i26 < 3; i26++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * (i26 + 6)) + 6] = 0.0;
    }
  }

  for (i26 = 0; i26 < 9; i26++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 12 * i26) + 9] = iv14[cr + 3 * i26];
    }
  }

  d_eye(P_xx_apr);
  dv23[0] = 0.0;
  dv23[3] = -w[2];
  dv23[6] = w[1];
  dv23[1] = w[2];
  dv23[4] = 0.0;
  dv23[7] = -w[0];
  dv23[2] = -w[1];
  dv23[5] = w[0];
  dv23[8] = 0.0;
  for (i26 = 0; i26 < 12; i26++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[cr + 12 * i26] = iv15[cr + 3 * i26];
    }
  }

  for (i26 = 0; i26 < 3; i26++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * i26) + 3] = 0.0;
    }
  }

  for (i26 = 0; i26 < 3; i26++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i26 + 3)) + 3] = -dv23[cr + 3 * i26];
    }
  }

  for (i26 = 0; i26 < 3; i26++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i26 + 6)) + 3] = 0.0;
    }
  }

  for (i26 = 0; i26 < 3; i26++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * (i26 + 9)) + 3] = iv16[cr + 3 * i26];
    }
  }

  for (i26 = 0; i26 < 12; i26++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * i26) + 6] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 12 * i26) + 9] = 0.0;
    }

    for (cr = 0; cr < 12; cr++) {
      Phi[cr + 12 * i26] = P_xx_apr[cr + 12 * i26] + b_G[cr + 12 * i26] * dt;
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)c_numStates;
  }

  emxInit_real_T(&P_xs_apr, 2);
  if (loop_ub == 1) {
    emxInit_real_T(&b_P_apo, 2);
    i26 = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = 1;
    b_P_apo->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)b_P_apo, i26, (int)sizeof(double));
    for (i26 = 0; i26 < b_loop_ub; i26++) {
      cr = 0;
      while (cr <= 0) {
        b_P_apo->data[b_P_apo->size[0] * i26] = P_apo->data[P_apo->size[0] * i26];
        cr = 1;
      }
    }

    i26 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i26, (int)sizeof(double));
    for (i26 = 0; i26 < 12; i26++) {
      loop_ub = b_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i26 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          P_xs_apr->data[i26 + P_xs_apr->size[0] * cr] += Phi[i26 + 12 * k] *
            b_P_apo->data[k + b_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&b_P_apo);
  } else {
    i26 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i26, (int)sizeof(double));
    i26 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i26, (int)sizeof(double));
    k = 12 * b_loop_ub;
    for (i26 = 0; i26 < k; i26++) {
      P_xs_apr->data[i26] = 0.0;
    }

    if (b_loop_ub == 0) {
    } else {
      b_loop_ub = 12 * (b_loop_ub - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * div_nzp_s32_floor(ib,
               loop_ub)] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                div_nzp_s32_floor(ib, loop_ub)] * Phi[ia - 1];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  for (i26 = 0; i26 < 12; i26++) {
    for (cr = 0; cr < 12; cr++) {
      P_xx_apr[cr + 12 * i26] = Phi[i26 + 12 * cr];
    }
  }

  if (P_xs_apr->size[1] == 1) {
    for (i26 = 0; i26 < 12; i26++) {
      for (cr = 0; cr < 12; cr++) {
        y[i26 + 12 * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          y[i26 + 12 * cr] += P_xs_apr->data[i26 + 12 * k] * P_xx_apr[k + 12 *
            cr];
        }
      }
    }
  } else {
    k = P_xs_apr->size[1];
    memset(&y[0], 0, 144U * sizeof(double));
    for (cr = 0; cr < 134; cr += 12) {
      for (ic = cr; ic + 1 <= cr + 12; ic++) {
        y[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr < 134; cr += 12) {
      ar = 0;
      i26 = br + k;
      for (ib = br; ib + 1 <= i26; ib++) {
        if (P_xx_apr[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 12; ic++) {
            ia++;
            y[ic] += P_xx_apr[ib] * P_xs_apr->data[ia - 1];
          }
        }

        ar += 12;
      }

      br += k;
    }
  }

  c = dt * dt;
  for (i26 = 0; i26 < 12; i26++) {
    for (cr = 0; cr < 9; cr++) {
      c_G[i26 + 12 * cr] = 0.0;
      for (k = 0; k < 9; k++) {
        c_G[i26 + 12 * cr] += G[i26 + 12 * k] * Q[k + 9 * cr];
      }
    }

    for (cr = 0; cr < 12; cr++) {
      b_G[i26 + 12 * cr] = 0.0;
      for (k = 0; k < 9; k++) {
        b_G[i26 + 12 * cr] += c_G[i26 + 12 * k] * G[cr + 12 * k];
      }
    }
  }

  for (i26 = 0; i26 < 12; i26++) {
    for (cr = 0; cr < 12; cr++) {
      P_xx_apr[cr + 12 * i26] = y[cr + 12 * i26] + b_G[cr + 12 * i26] * c;
    }
  }

  //  covariance of the state
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i26 = 0;
    cr = 0;
  } else {
    i26 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[1];
  }

  if (loop_ub == 1) {
    emxInit_real_T(&c_P_apo, 2);
    k = c_P_apo->size[0] * c_P_apo->size[1];
    c_P_apo->size[0] = 1;
    c_P_apo->size[1] = cr - i26;
    emxEnsureCapacity((emxArray__common *)c_P_apo, k, (int)sizeof(double));
    loop_ub = cr - i26;
    for (cr = 0; cr < loop_ub; cr++) {
      k = 0;
      while (k <= 0) {
        c_P_apo->data[c_P_apo->size[0] * cr] = P_apo->data[P_apo->size[0] * (i26
          + cr)];
        k = 1;
      }
    }

    i26 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = c_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i26, (int)sizeof(double));
    for (i26 = 0; i26 < 12; i26++) {
      loop_ub = c_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i26 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          P_xs_apr->data[i26 + P_xs_apr->size[0] * cr] += Phi[i26 + 12 * k] *
            c_P_apo->data[k + c_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&c_P_apo);
  } else {
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = cr - i26;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    b_loop_ub = 12 * (cr - i26);
    for (k = 0; k < b_loop_ub; k++) {
      P_xs_apr->data[k] = 0.0;
    }

    if (cr - i26 == 0) {
    } else {
      b_loop_ub = 12 * ((cr - i26) - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * (i26 +
               div_nzp_s32_floor(ib, loop_ub))] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                (i26 + div_nzp_s32_floor(ib, loop_ub))] * Phi[ia - 1];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  //  covariance between current state and trails
  i26 = P_apr->size[0] * P_apr->size[1];
  P_apr->size[0] = P_apo->size[0];
  P_apr->size[1] = P_apo->size[1];
  emxEnsureCapacity((emxArray__common *)P_apr, i26, (int)sizeof(double));
  loop_ub = P_apo->size[0] * P_apo->size[1];
  for (i26 = 0; i26 < loop_ub; i26++) {
    P_apr->data[i26] = P_apo->data[i26];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)c_numStates;
  }

  emxInit_int32_T(&r17, 1);
  i26 = r17->size[0];
  r17->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r17, i26, (int)sizeof(int));
  for (i26 = 0; i26 < loop_ub; i26++) {
    r17->data[i26] = i26;
  }

  emxInit_int32_T(&r18, 1);
  i26 = r18->size[0];
  r18->size[0] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r18, i26, (int)sizeof(int));
  for (i26 = 0; i26 < b_loop_ub; i26++) {
    r18->data[i26] = i26;
  }

  for (i26 = 0; i26 < 12; i26++) {
    for (cr = 0; cr < 12; cr++) {
      b_G[cr + 12 * i26] = (P_xx_apr[cr + 12 * i26] + P_xx_apr[i26 + 12 * cr]) /
        2.0;
    }
  }

  k = r17->size[0];
  b_loop_ub = r18->size[0];
  for (i26 = 0; i26 < b_loop_ub; i26++) {
    for (cr = 0; cr < k; cr++) {
      P_apr->data[r17->data[cr] + P_apr->size[0] * r18->data[i26]] = b_G[cr + k *
        i26];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apr->size[1]) {
    i26 = 0;
    cr = 0;
  } else {
    i26 = (int)(c_numStates + 1.0) - 1;
    cr = P_apr->size[1];
  }

  k = r17->size[0];
  r17->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r17, k, (int)sizeof(int));
  for (k = 0; k < loop_ub; k++) {
    r17->data[k] = k;
  }

  k = r18->size[0];
  r18->size[0] = cr - i26;
  emxEnsureCapacity((emxArray__common *)r18, k, (int)sizeof(int));
  loop_ub = cr - i26;
  for (cr = 0; cr < loop_ub; cr++) {
    r18->data[cr] = i26 + cr;
  }

  k = r17->size[0];
  b_loop_ub = r18->size[0];
  for (i26 = 0; i26 < b_loop_ub; i26++) {
    for (cr = 0; cr < k; cr++) {
      P_apr->data[r17->data[cr] + P_apr->size[0] * r18->data[i26]] =
        P_xs_apr->data[cr + k * i26];
    }
  }

  if (c_numStates + 1.0 > P_apr->size[0]) {
    i26 = 0;
    cr = 0;
  } else {
    i26 = (int)(c_numStates + 1.0) - 1;
    cr = P_apr->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  k = r17->size[0];
  r17->size[0] = cr - i26;
  emxEnsureCapacity((emxArray__common *)r17, k, (int)sizeof(int));
  b_loop_ub = cr - i26;
  for (cr = 0; cr < b_loop_ub; cr++) {
    r17->data[cr] = i26 + cr;
  }

  i26 = r18->size[0];
  r18->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r18, i26, (int)sizeof(int));
  for (i26 = 0; i26 < loop_ub; i26++) {
    r18->data[i26] = i26;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i26 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i26, (int)sizeof(double));
  for (i26 = 0; i26 < 12; i26++) {
    loop_ub = P_xs_apr->size[1];
    for (cr = 0; cr < loop_ub; cr++) {
      b_P_xs_apr->data[cr + b_P_xs_apr->size[0] * i26] = P_xs_apr->data[i26 +
        P_xs_apr->size[0] * cr];
    }
  }

  k = r17->size[0];
  b_loop_ub = r18->size[0];
  for (i26 = 0; i26 < b_loop_ub; i26++) {
    for (cr = 0; cr < k; cr++) {
      P_apr->data[r17->data[cr] + P_apr->size[0] * r18->data[i26]] =
        b_P_xs_apr->data[cr + k * i26];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r18);
  emxFree_int32_T(&r17);
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)c_numStates;
  }

  emxInit_real_T(&b_P_xx_apr, 2);
  i26 = b_P_xx_apr->size[0] * b_P_xx_apr->size[1];
  b_P_xx_apr->size[0] = loop_ub;
  b_P_xx_apr->size[1] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)b_P_xx_apr, i26, (int)sizeof(double));
  for (i26 = 0; i26 < b_loop_ub; i26++) {
    for (cr = 0; cr < loop_ub; cr++) {
      b_P_xx_apr->data[cr + b_P_xx_apr->size[0] * i26] = P_apo->data[cr +
        P_apo->size[0] * i26];
    }
  }

  emxInit_real_T(&b_Phi, 2);
  b_eye(c_numStates, b_Phi);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&c_x, 1);
  i26 = c_x->size[0];
  c_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_x, i26, (int)sizeof(double));
  for (i26 = 0; i26 < loop_ub; i26++) {
    c_x->data[i26] = x->data[i26];
  }

  b_emxInit_real_T(&x1, 1);
  dxdt_dPdt(last_imu, c_x, b_P_xx_apr, b_Phi, Q, control_input, x1, P_xx_apr,
            P_xs_apr);
  emxFree_real_T(&c_x);
  for (k = 0; k < 4; k++) {
    u_1[k] = control_input[k] + (control_input[k] - control_input[k]) * 0.5;
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&xx, 1);
  i26 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i26, (int)sizeof(double));
  for (i26 = 0; i26 < loop_ub; i26++) {
    xx->data[i26] = x->data[i26] + x1->data[i26] * dt / 2.0;
  }

  for (i26 = 0; i26 < 4; i26++) {
    b_xx[i26] = xx->data[3 + i26];
  }

  c = b_norm(b_xx);
  for (i26 = 0; i26 < 4; i26++) {
    b_xx[i26] = xx->data[3 + i26] / c;
  }

  for (i26 = 0; i26 < 4; i26++) {
    xx->data[3 + i26] = b_xx[i26];
  }

  for (k = 0; k < 6; k++) {
    dv24[k] = last_imu[k] + (current_imu[k] - last_imu[k]) * 0.5;
  }

  for (i26 = 0; i26 < 144; i26++) {
    b_G[i26] = b_P_xx_apr->data[i26] + P_xx_apr[i26] * dt / 2.0;
  }

  emxInit_real_T(&c_Phi, 2);
  i26 = c_Phi->size[0] * c_Phi->size[1];
  c_Phi->size[0] = b_Phi->size[0];
  c_Phi->size[1] = b_Phi->size[1];
  emxEnsureCapacity((emxArray__common *)c_Phi, i26, (int)sizeof(double));
  loop_ub = b_Phi->size[0] * b_Phi->size[1];
  for (i26 = 0; i26 < loop_ub; i26++) {
    c_Phi->data[i26] = b_Phi->data[i26] + P_xs_apr->data[i26] * dt / 2.0;
  }

  b_emxInit_real_T(&x2, 1);
  b_dxdt_dPdt(dv24, xx, b_G, c_Phi, Q, u_1, x2, P_xx_apr, P_xs_apr);
  emxFree_real_T(&c_Phi);
  for (i26 = 0; i26 < 4; i26++) {
    u_1[i26] += (control_input[i26] - u_1[i26]) * 0.5;
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i26 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i26, (int)sizeof(double));
  for (i26 = 0; i26 < loop_ub; i26++) {
    xx->data[i26] = x->data[i26] + x2->data[i26] * dt / 2.0;
  }

  for (i26 = 0; i26 < 4; i26++) {
    b_xx[i26] = xx->data[3 + i26];
  }

  c = b_norm(b_xx);
  for (i26 = 0; i26 < 4; i26++) {
    b_xx[i26] = xx->data[3 + i26] / c;
  }

  for (i26 = 0; i26 < 4; i26++) {
    xx->data[3 + i26] = b_xx[i26];
  }

  for (k = 0; k < 6; k++) {
    dv24[k] = last_imu[k] + (current_imu[k] - last_imu[k]) * 0.5;
  }

  for (i26 = 0; i26 < 144; i26++) {
    b_G[i26] = b_P_xx_apr->data[i26] + P_xx_apr[i26] * dt / 2.0;
  }

  emxInit_real_T(&d_Phi, 2);
  i26 = d_Phi->size[0] * d_Phi->size[1];
  d_Phi->size[0] = b_Phi->size[0];
  d_Phi->size[1] = b_Phi->size[1];
  emxEnsureCapacity((emxArray__common *)d_Phi, i26, (int)sizeof(double));
  loop_ub = b_Phi->size[0] * b_Phi->size[1];
  for (i26 = 0; i26 < loop_ub; i26++) {
    d_Phi->data[i26] = b_Phi->data[i26] + P_xs_apr->data[i26] * dt / 2.0;
  }

  b_emxInit_real_T(&x3, 1);
  b_dxdt_dPdt(dv24, xx, b_G, d_Phi, Q, u_1, x3, P_xx_apr, P_xs_apr);
  emxFree_real_T(&d_Phi);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i26 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i26, (int)sizeof(double));
  for (i26 = 0; i26 < loop_ub; i26++) {
    xx->data[i26] = x->data[i26] + x3->data[i26] * dt;
  }

  for (i26 = 0; i26 < 4; i26++) {
    b_xx[i26] = xx->data[3 + i26];
  }

  c = b_norm(b_xx);
  for (i26 = 0; i26 < 4; i26++) {
    b_xx[i26] = xx->data[3 + i26] / c;
  }

  for (i26 = 0; i26 < 4; i26++) {
    xx->data[3 + i26] = b_xx[i26];
  }

  for (i26 = 0; i26 < 144; i26++) {
    b_G[i26] = b_P_xx_apr->data[i26] + P_xx_apr[i26] * dt;
  }

  emxFree_real_T(&b_P_xx_apr);
  emxInit_real_T(&e_Phi, 2);
  i26 = e_Phi->size[0] * e_Phi->size[1];
  e_Phi->size[0] = b_Phi->size[0];
  e_Phi->size[1] = b_Phi->size[1];
  emxEnsureCapacity((emxArray__common *)e_Phi, i26, (int)sizeof(double));
  loop_ub = b_Phi->size[0] * b_Phi->size[1];
  for (i26 = 0; i26 < loop_ub; i26++) {
    e_Phi->data[i26] = b_Phi->data[i26] + P_xs_apr->data[i26] * dt;
  }

  emxFree_real_T(&b_Phi);
  for (k = 0; k < 4; k++) {
    b_xx[k] = u_1[k] + (control_input[k] - u_1[k]);
  }

  b_emxInit_real_T(&x4, 1);
  b_dxdt_dPdt(current_imu, xx, b_G, e_Phi, Q, b_xx, x4, P_xx_apr, P_xs_apr);
  emxFree_real_T(&e_Phi);
  emxFree_real_T(&xx);
  emxFree_real_T(&P_xs_apr);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  if (1.0 > c_numStates + 1.0) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_int32_T(&r19, 2);
  i26 = r19->size[0] * r19->size[1];
  r19->size[0] = 1;
  r19->size[1] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r19, i26, (int)sizeof(int));
  for (i26 = 0; i26 < b_loop_ub; i26++) {
    r19->data[r19->size[0] * i26] = i26;
  }

  b_emxInit_real_T(&d_x, 1);
  i26 = d_x->size[0];
  d_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)d_x, i26, (int)sizeof(double));
  for (i26 = 0; i26 < loop_ub; i26++) {
    d_x->data[i26] = x->data[i26] + (((x1->data[i26] + 2.0 * x2->data[i26]) +
      2.0 * x3->data[i26]) + x4->data[i26]) * dt / 6.0;
  }

  emxFree_real_T(&x4);
  emxFree_real_T(&x3);
  emxFree_real_T(&x2);
  emxFree_real_T(&x1);
  loop_ub = r19->size[1];
  for (i26 = 0; i26 < loop_ub; i26++) {
    x->data[r19->data[r19->size[0] * i26]] = d_x->data[(*(int (*)[2])r19->size)
      [0] * i26];
  }

  emxFree_real_T(&d_x);
  emxFree_int32_T(&r19);

  //  P_xx_apr = P_xx_apr  + (P1+2*P2+2*P3+P4)*dt/6;     % covariance of the state 
  //  P_xx_apr = (P_xx_apr + P_xx_apr')*dt/2;
  //
  //  Phi = Phi + (Phi1 + 2*Phi2 + 2*Phi3 + Phi4)*dt/6;
  //
  //  P_xs_apr = Phi*P_apo(1:numStates, numStates+1:end); % covariance between current state and trails 
  c = b_norm(*(double (*)[4])&x->data[3]);
  for (i26 = 0; i26 < 4; i26++) {
    b_xx[i26] = x->data[3 + i26] / c;
  }

  for (i26 = 0; i26 < 4; i26++) {
    x->data[3 + i26] = b_xx[i26];
  }

  //  P_apr = P_apo;
  //  P_apr(1:numStates, 1:numStates)     =    P_xx_apr;
  //  P_apr(1:numStates,numStates+1:end)  =    P_xs_apr;
  //  P_apr(numStates+1:end,1:numStates)  =    P_xs_apr';
  //  P_apr=(P_apr+P_apr')/2;
  // % =================================================================================================== 
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_pred_init()
{
  int i;
  for (i = 0; i < 6; i++) {
    last_imu[i] = 0.0;
  }
}

//
// File trailer for SLAM_pred.cpp
//
// [EOF]
//
