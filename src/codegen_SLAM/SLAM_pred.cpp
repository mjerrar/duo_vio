//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Aug-2015 16:03:26
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
  *x, const double P_xx_apr[144], const emxArray_real_T *Phi, const double b_Q
  [81], emxArray_real_T *x_dot, double P_xx_apr_dot[144], emxArray_real_T
  *Phi_dot);
static void dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T *x,
                      const emxArray_real_T *P_xx_apr, const emxArray_real_T
                      *Phi, const double b_Q[81], emxArray_real_T *x_dot, double
                      P_xx_apr_dot[144], emxArray_real_T *Phi_dot);

// Function Definitions

//
// linearly interpolate between previous and current measurement to get
// to the corresponding part inside dt (for Runge-Kutta)
// za = prev_za + (za - prev_za) * part ;
// Arguments    : double dt
//                const double meas[6]
//                const emxArray_real_T *x
//                const double P_xx_apr[144]
//                const emxArray_real_T *Phi
//                const double b_Q[81]
//                emxArray_real_T *x_dot
//                double P_xx_apr_dot[144]
//                emxArray_real_T *Phi_dot
// Return Type  : void
//
static void b_dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T
  *x, const double P_xx_apr[144], const emxArray_real_T *Phi, const double b_Q
  [81], emxArray_real_T *x_dot, double P_xx_apr_dot[144], emxArray_real_T
  *Phi_dot)
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
  double m_a[3];
  int i;
  double w[3];
  int br;
  double dv34[9];
  double b_R_cw[9];
  double dv35[9];
  double c_R_cw[9];
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

  double grav[3];
  static const double dv36[3] = { 0.0, 0.0, 9.81 };

  static const double dv37[3] = { 0.0, 0.0, -9.81 };

  unsigned int unnamed_idx_0;
  double dv38[9];
  double dv39[16];
  double dv40[16];
  double b_x[4];
  double dv41[4];
  double d_R_cw[3];
  double b_G[108];
  double FP[144];
  double b_FP[144];
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
  for (i = 0; i < 3; i++) {
    m_a[i] = 0.0 * meas[i + 3];
  }

  for (br = 0; br < 3; br++) {
    w[br] = 0.0 * (meas[br] - x->data[10 + br]);
  }

  dv34[0] = 0.0;
  dv34[3] = -w[2];
  dv34[6] = w[1];
  dv34[1] = w[2];
  dv34[4] = 0.0;
  dv34[7] = -w[0];
  dv34[2] = -w[1];
  dv34[5] = w[0];
  dv34[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      b_R_cw[i + 3 * br] = -R_cw[br + 3 * i];
    }
  }

  dv35[0] = 0.0;
  dv35[3] = -m_a[2];
  dv35[6] = m_a[1];
  dv35[1] = m_a[2];
  dv35[4] = 0.0;
  dv35[7] = -m_a[0];
  dv35[2] = -m_a[1];
  dv35[5] = m_a[0];
  dv35[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      c_R_cw[br + 3 * i] = 0.0;
      for (cr = 0; cr < 3; cr++) {
        c_R_cw[br + 3 * i] += b_R_cw[br + 3 * cr] * dv35[cr + 3 * i];
      }
    }
  }

  for (br = 0; br < 12; br++) {
    for (i = 0; i < 3; i++) {
      F[i + 12 * br] = iv8[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * br) + 3] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 3)) + 3] = -dv34[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 6)) + 3] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 9)) + 3] = iv9[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * br) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 3)) + 6] = c_R_cw[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 6)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 9)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 12; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * br) + 9] = 0.0;
    }
  }

  for (br = 0; br < 9; br++) {
    for (i = 0; i < 3; i++) {
      G[i + 12 * br] = 0.0;
    }

    for (i = 0; i < 3; i++) {
      G[(i + 12 * br) + 3] = iv10[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      G[(i + 12 * br) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      G[(i + 12 * (br + 3)) + 6] = -R_cw[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      G[(i + 12 * (br + 6)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 9; br++) {
    for (i = 0; i < 3; i++) {
      G[(i + 12 * br) + 9] = iv11[i + 3 * br];
    }
  }

  if (normalGravity) {
    for (i = 0; i < 3; i++) {
      grav[i] = dv36[i];
    }
  } else {
    for (i = 0; i < 3; i++) {
      grav[i] = dv37[i];
    }
  }

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)x->size[0];
  br = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, br, (int)sizeof(double));
  i = (int)unnamed_idx_0;
  for (br = 0; br < i; br++) {
    x_dot->data[br] = 0.0;
  }

  for (br = 0; br < 3; br++) {
    x_dot->data[br] = x->data[7 + br];
  }

  //  position
  dv38[0] = 0.0;
  dv38[3] = -w[2];
  dv38[6] = w[1];
  dv38[1] = w[2];
  dv38[4] = 0.0;
  dv38[7] = -w[0];
  dv38[2] = -w[1];
  dv38[5] = w[0];
  dv38[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      dv39[i + (br << 2)] = -dv38[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    dv39[12 + br] = w[br];
  }

  for (br = 0; br < 3; br++) {
    dv39[3 + (br << 2)] = -w[br];
  }

  dv39[15] = 0.0;
  for (br = 0; br < 4; br++) {
    for (i = 0; i < 4; i++) {
      dv40[i + (br << 2)] = 0.5 * dv39[i + (br << 2)];
    }
  }

  for (br = 0; br < 4; br++) {
    b_x[br] = x->data[3 + br];
  }

  for (br = 0; br < 4; br++) {
    dv41[br] = 0.0;
    for (i = 0; i < 4; i++) {
      dv41[br] += dv40[br + (i << 2)] * b_x[i];
    }
  }

  for (br = 0; br < 4; br++) {
    x_dot->data[3 + br] = dv41[br];
  }

  //  rot angle
  for (br = 0; br < 3; br++) {
    a = 0.0;
    for (i = 0; i < 3; i++) {
      a += R_cw[i + 3 * br] * m_a[i];
    }

    d_R_cw[br] = a - grav[br];
  }

  for (br = 0; br < 3; br++) {
    x_dot->data[7 + br] = 0.0 * d_R_cw[br];
  }

  for (br = 0; br < 3; br++) {
    x_dot->data[10 + br] = 0.0;
  }

  //  gyro bias
  br = x_dot->size[0];
  emxEnsureCapacity((emxArray__common *)x_dot, br, (int)sizeof(double));
  i = x_dot->size[0];
  for (br = 0; br < i; br++) {
    x_dot->data[br] *= dt;
  }

  // time derivative of the covariance
  for (br = 0; br < 12; br++) {
    for (i = 0; i < 12; i++) {
      FP[br + 12 * i] = 0.0;
      for (cr = 0; cr < 12; cr++) {
        FP[br + 12 * i] += F[br + 12 * cr] * P_xx_apr[cr + 12 * i];
      }
    }

    for (i = 0; i < 9; i++) {
      b_G[br + 12 * i] = 0.0;
      for (cr = 0; cr < 9; cr++) {
        b_G[br + 12 * i] += G[br + 12 * cr] * b_Q[cr + 9 * i];
      }
    }
  }

  for (br = 0; br < 12; br++) {
    for (i = 0; i < 12; i++) {
      a = 0.0;
      for (cr = 0; cr < 9; cr++) {
        a += b_G[br + 12 * cr] * G[i + 12 * cr];
      }

      b_FP[br + 12 * i] = (FP[br + 12 * i] + FP[i + 12 * br]) + a;
    }
  }

  for (br = 0; br < 12; br++) {
    for (i = 0; i < 12; i++) {
      P_xx_apr_dot[i + 12 * br] = b_FP[i + 12 * br] * dt;
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
  i = 12 * (int)unnamed_idx_0;
  for (br = 0; br < i; br++) {
    Phi_dot->data[br] = 0.0;
  }

  if (Phi->size[1] == 0) {
  } else {
    i = 12 * (Phi->size[1] - 1);
    for (cr = 0; cr <= i; cr += 12) {
      for (ic = cr + 1; ic <= cr + 12; ic++) {
        Phi_dot->data[ic - 1] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= i; cr += 12) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 12; ib++) {
        if (Phi->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 12; ic++) {
            ia++;
            Phi_dot->data[ic] += Phi->data[ib] * F[ia];
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
  i = Phi_dot->size[0];
  cr = Phi_dot->size[1];
  i *= cr;
  for (br = 0; br < i; br++) {
    Phi_dot->data[br] *= dt;
  }
}

//
// linearly interpolate between previous and current measurement to get
// to the corresponding part inside dt (for Runge-Kutta)
// za = prev_za + (za - prev_za) * part ;
// Arguments    : double dt
//                const double meas[6]
//                const emxArray_real_T *x
//                const emxArray_real_T *P_xx_apr
//                const emxArray_real_T *Phi
//                const double b_Q[81]
//                emxArray_real_T *x_dot
//                double P_xx_apr_dot[144]
//                emxArray_real_T *Phi_dot
// Return Type  : void
//
static void dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T *x,
                      const emxArray_real_T *P_xx_apr, const emxArray_real_T
                      *Phi, const double b_Q[81], emxArray_real_T *x_dot, double
                      P_xx_apr_dot[144], emxArray_real_T *Phi_dot)
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
  double m_a[3];
  int i;
  double w[3];
  double dv26[9];
  double b_R_cw[9];
  int br;
  double dv27[9];
  double c_R_cw[9];
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

  double grav[3];
  static const double dv28[3] = { 0.0, 0.0, 9.81 };

  static const double dv29[3] = { 0.0, 0.0, -9.81 };

  unsigned int unnamed_idx_0;
  int cr;
  double dv30[9];
  double dv31[16];
  double dv32[16];
  double b_x[4];
  double dv33[4];
  double d_R_cw[3];
  emxArray_real_T *FP;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *b_FP;
  double b_G[108];
  double c_FP[144];

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
  for (i = 0; i < 3; i++) {
    m_a[i] = 0.0 * meas[i + 3];
  }

  for (i = 0; i < 3; i++) {
    w[i] = 0.0 * (meas[i] - x->data[10 + i]);
  }

  dv26[0] = 0.0;
  dv26[3] = -w[2];
  dv26[6] = w[1];
  dv26[1] = w[2];
  dv26[4] = 0.0;
  dv26[7] = -w[0];
  dv26[2] = -w[1];
  dv26[5] = w[0];
  dv26[8] = 0.0;
  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      b_R_cw[br + 3 * i] = -R_cw[i + 3 * br];
    }
  }

  dv27[0] = 0.0;
  dv27[3] = -m_a[2];
  dv27[6] = m_a[1];
  dv27[1] = m_a[2];
  dv27[4] = 0.0;
  dv27[7] = -m_a[0];
  dv27[2] = -m_a[1];
  dv27[5] = m_a[0];
  dv27[8] = 0.0;
  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      c_R_cw[i + 3 * br] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        c_R_cw[i + 3 * br] += b_R_cw[i + 3 * ar] * dv27[ar + 3 * br];
      }
    }
  }

  for (i = 0; i < 12; i++) {
    for (br = 0; br < 3; br++) {
      F[br + 12 * i] = iv4[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * i) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 3)) + 3] = -dv26[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 6)) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 9)) + 3] = iv5[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * i) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 3)) + 6] = c_R_cw[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 6)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 9)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 12; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * i) + 9] = 0.0;
    }
  }

  for (i = 0; i < 9; i++) {
    for (br = 0; br < 3; br++) {
      G[br + 12 * i] = 0.0;
    }

    for (br = 0; br < 3; br++) {
      G[(br + 12 * i) + 3] = iv6[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * i) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * (i + 3)) + 6] = -R_cw[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * (i + 6)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 9; i++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * i) + 9] = iv7[br + 3 * i];
    }
  }

  if (normalGravity) {
    for (i = 0; i < 3; i++) {
      grav[i] = dv28[i];
    }
  } else {
    for (i = 0; i < 3; i++) {
      grav[i] = dv29[i];
    }
  }

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)x->size[0];
  i = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, i, (int)sizeof(double));
  cr = (int)unnamed_idx_0;
  for (i = 0; i < cr; i++) {
    x_dot->data[i] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    x_dot->data[i] = x->data[7 + i];
  }

  //  position
  dv30[0] = 0.0;
  dv30[3] = -w[2];
  dv30[6] = w[1];
  dv30[1] = w[2];
  dv30[4] = 0.0;
  dv30[7] = -w[0];
  dv30[2] = -w[1];
  dv30[5] = w[0];
  dv30[8] = 0.0;
  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      dv31[br + (i << 2)] = -dv30[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    dv31[12 + i] = w[i];
  }

  for (i = 0; i < 3; i++) {
    dv31[3 + (i << 2)] = -w[i];
  }

  dv31[15] = 0.0;
  for (i = 0; i < 4; i++) {
    for (br = 0; br < 4; br++) {
      dv32[br + (i << 2)] = 0.5 * dv31[br + (i << 2)];
    }
  }

  for (i = 0; i < 4; i++) {
    b_x[i] = x->data[3 + i];
  }

  for (i = 0; i < 4; i++) {
    dv33[i] = 0.0;
    for (br = 0; br < 4; br++) {
      dv33[i] += dv32[i + (br << 2)] * b_x[br];
    }
  }

  for (i = 0; i < 4; i++) {
    x_dot->data[3 + i] = dv33[i];
  }

  //  rot angle
  for (i = 0; i < 3; i++) {
    a = 0.0;
    for (br = 0; br < 3; br++) {
      a += R_cw[br + 3 * i] * m_a[br];
    }

    d_R_cw[i] = a - grav[i];
  }

  for (i = 0; i < 3; i++) {
    x_dot->data[7 + i] = 0.0 * d_R_cw[i];
  }

  for (i = 0; i < 3; i++) {
    x_dot->data[10 + i] = 0.0;
  }

  //  gyro bias
  i = x_dot->size[0];
  emxEnsureCapacity((emxArray__common *)x_dot, i, (int)sizeof(double));
  cr = x_dot->size[0];
  for (i = 0; i < cr; i++) {
    x_dot->data[i] *= dt;
  }

  // time derivative of the covariance
  emxInit_real_T(&FP, 2);
  if (P_xx_apr->size[0] == 1) {
    i = FP->size[0] * FP->size[1];
    FP->size[0] = 12;
    FP->size[1] = P_xx_apr->size[1];
    emxEnsureCapacity((emxArray__common *)FP, i, (int)sizeof(double));
    for (i = 0; i < 12; i++) {
      cr = P_xx_apr->size[1];
      for (br = 0; br < cr; br++) {
        FP->data[i + FP->size[0] * br] = 0.0;
        for (ar = 0; ar < 12; ar++) {
          FP->data[i + FP->size[0] * br] += F[i + 12 * ar] * P_xx_apr->data[ar +
            P_xx_apr->size[0] * br];
        }
      }
    }
  } else {
    unnamed_idx_0 = (unsigned int)P_xx_apr->size[1];
    i = FP->size[0] * FP->size[1];
    FP->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)FP, i, (int)sizeof(double));
    i = FP->size[0] * FP->size[1];
    FP->size[1] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)FP, i, (int)sizeof(double));
    cr = 12 * (int)unnamed_idx_0;
    for (i = 0; i < cr; i++) {
      FP->data[i] = 0.0;
    }

    if (P_xx_apr->size[1] == 0) {
    } else {
      i = 12 * (P_xx_apr->size[1] - 1);
      for (cr = 0; cr <= i; cr += 12) {
        for (ic = cr + 1; ic <= cr + 12; ic++) {
          FP->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= i; cr += 12) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_xx_apr->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              FP->data[ic] += P_xx_apr->data[ib] * F[ia];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  emxInit_real_T(&b_FP, 2);
  i = b_FP->size[0] * b_FP->size[1];
  b_FP->size[0] = FP->size[1];
  b_FP->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_FP, i, (int)sizeof(double));
  for (i = 0; i < 12; i++) {
    cr = FP->size[1];
    for (br = 0; br < cr; br++) {
      b_FP->data[br + b_FP->size[0] * i] = FP->data[i + FP->size[0] * br];
    }
  }

  for (i = 0; i < 12; i++) {
    for (br = 0; br < 9; br++) {
      b_G[i + 12 * br] = 0.0;
      for (ar = 0; ar < 9; ar++) {
        b_G[i + 12 * br] += G[i + 12 * ar] * b_Q[ar + 9 * br];
      }
    }
  }

  for (i = 0; i < 12; i++) {
    for (br = 0; br < 12; br++) {
      a = 0.0;
      for (ar = 0; ar < 9; ar++) {
        a += b_G[i + 12 * ar] * G[br + 12 * ar];
      }

      c_FP[i + 12 * br] = (FP->data[i + 12 * br] + b_FP->data[i + 12 * br]) + a;
    }
  }

  emxFree_real_T(&b_FP);
  emxFree_real_T(&FP);
  for (i = 0; i < 12; i++) {
    for (br = 0; br < 12; br++) {
      P_xx_apr_dot[br + 12 * i] = c_FP[br + 12 * i] * dt;
    }
  }

  // time derivative of the state transition
  if (Phi->size[0] == 1) {
    i = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[0] = 12;
    Phi_dot->size[1] = Phi->size[1];
    emxEnsureCapacity((emxArray__common *)Phi_dot, i, (int)sizeof(double));
    for (i = 0; i < 12; i++) {
      cr = Phi->size[1];
      for (br = 0; br < cr; br++) {
        Phi_dot->data[i + Phi_dot->size[0] * br] = 0.0;
        for (ar = 0; ar < 12; ar++) {
          Phi_dot->data[i + Phi_dot->size[0] * br] += F[i + 12 * ar] * Phi->
            data[ar + Phi->size[0] * br];
        }
      }
    }
  } else {
    unnamed_idx_0 = (unsigned int)Phi->size[1];
    i = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)Phi_dot, i, (int)sizeof(double));
    i = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[1] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)Phi_dot, i, (int)sizeof(double));
    cr = 12 * (int)unnamed_idx_0;
    for (i = 0; i < cr; i++) {
      Phi_dot->data[i] = 0.0;
    }

    if (Phi->size[1] == 0) {
    } else {
      i = 12 * (Phi->size[1] - 1);
      for (cr = 0; cr <= i; cr += 12) {
        for (ic = cr + 1; ic <= cr + 12; ic++) {
          Phi_dot->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= i; cr += 12) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (Phi->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              Phi_dot->data[ic] += Phi->data[ib] * F[ia];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  i = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[0] = 12;
  emxEnsureCapacity((emxArray__common *)Phi_dot, i, (int)sizeof(double));
  i = Phi_dot->size[0];
  cr = Phi_dot->size[1];
  cr *= i;
  for (i = 0; i < cr; i++) {
    Phi_dot->data[i] *= dt;
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
//                const double IMU_measurements[13]
//                double c_numStates
// Return Type  : void
//
void SLAM_pred(emxArray_real_T *P_apo, emxArray_real_T *x, double dt, const
               double processNoise[4], const double IMU_measurements[13], double
               c_numStates)
{
  double c;
  double b_processNoise[9];
  double c_processNoise[9];
  int i28;
  double b_Q[81];
  int loop_ub;
  int i;
  emxArray_real_T *P_xx_apr;
  int i29;
  emxArray_real_T *Phi;
  double meas_0[6];
  emxArray_real_T *b_x;
  emxArray_real_T *x1;
  emxArray_real_T *b_Phi;
  double b_P_xx_apr[144];
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
  emxArray_int32_T *r16;
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
  emxArray_int32_T *r17;
  emxArray_int32_T *r18;
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
  for (i28 = 0; i28 < 9; i28++) {
    c_processNoise[i28] = b_processNoise[i28] * c;
  }

  b_diag(c_processNoise, b_Q);
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
  i28 = P_xx_apr->size[0] * P_xx_apr->size[1];
  P_xx_apr->size[0] = loop_ub;
  P_xx_apr->size[1] = i;
  emxEnsureCapacity((emxArray__common *)P_xx_apr, i28, (int)sizeof(double));
  for (i28 = 0; i28 < i; i28++) {
    for (i29 = 0; i29 < loop_ub; i29++) {
      P_xx_apr->data[i29 + P_xx_apr->size[0] * i28] = P_apo->data[i29 +
        P_apo->size[0] * i28];
    }
  }

  emxInit_real_T(&Phi, 2);
  b_eye(c_numStates, Phi);
  for (i = 0; i < 3; i++) {
    meas_0[i] = IMU_measurements[i];
  }

  for (i = 0; i < 3; i++) {
    meas_0[i + 3] = IMU_measurements[i + 3];
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&b_x, 1);
  i28 = b_x->size[0];
  b_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)b_x, i28, (int)sizeof(double));
  for (i28 = 0; i28 < loop_ub; i28++) {
    b_x->data[i28] = x->data[i28];
  }

  b_emxInit_real_T(&x1, 1);
  emxInit_real_T(&b_Phi, 2);
  dxdt_dPdt(dt, meas_0, b_x, P_xx_apr, Phi, b_Q, x1, b_P_xx_apr, b_Phi);
  emxFree_real_T(&b_x);
  for (i = 0; i < 6; i++) {
    meas_1[i] = meas_0[i] + (meas_0[i] - meas_0[i]) * 0.5;
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&xx, 1);
  i28 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i28, (int)sizeof(double));
  for (i28 = 0; i28 < loop_ub; i28++) {
    xx->data[i28] = x->data[i28] + x1->data[i28] / 2.0;
  }

  for (i28 = 0; i28 < 4; i28++) {
    b_xx[i28] = xx->data[3 + i28];
  }

  c = b_norm(b_xx);
  for (i28 = 0; i28 < 4; i28++) {
    b_xx[i28] = xx->data[3 + i28] / c;
  }

  for (i28 = 0; i28 < 4; i28++) {
    xx->data[3 + i28] = b_xx[i28];
  }

  for (i28 = 0; i28 < 144; i28++) {
    c_P_xx_apr[i28] = P_xx_apr->data[i28] + b_P_xx_apr[i28] / 2.0;
  }

  emxInit_real_T(&c_Phi, 2);
  i28 = c_Phi->size[0] * c_Phi->size[1];
  c_Phi->size[0] = Phi->size[0];
  c_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)c_Phi, i28, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i28 = 0; i28 < loop_ub; i28++) {
    c_Phi->data[i28] = Phi->data[i28] + b_Phi->data[i28] / 2.0;
  }

  emxInit_real_T(&P_xs_apr, 2);
  b_emxInit_real_T(&x2, 1);
  b_dxdt_dPdt(dt, meas_1, xx, c_P_xx_apr, c_Phi, b_Q, x2, P2, P_xs_apr);
  emxFree_real_T(&c_Phi);
  for (i28 = 0; i28 < 6; i28++) {
    meas_1[i28] += (meas_0[i28] - meas_1[i28]) * 0.5;
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i28 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i28, (int)sizeof(double));
  for (i28 = 0; i28 < loop_ub; i28++) {
    xx->data[i28] = x->data[i28] + x2->data[i28] / 2.0;
  }

  for (i28 = 0; i28 < 4; i28++) {
    b_xx[i28] = xx->data[3 + i28];
  }

  c = b_norm(b_xx);
  for (i28 = 0; i28 < 4; i28++) {
    b_xx[i28] = xx->data[3 + i28] / c;
  }

  for (i28 = 0; i28 < 4; i28++) {
    xx->data[3 + i28] = b_xx[i28];
  }

  for (i28 = 0; i28 < 144; i28++) {
    c_P_xx_apr[i28] = P_xx_apr->data[i28] + P2[i28] / 2.0;
  }

  emxInit_real_T(&d_Phi, 2);
  i28 = d_Phi->size[0] * d_Phi->size[1];
  d_Phi->size[0] = Phi->size[0];
  d_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)d_Phi, i28, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i28 = 0; i28 < loop_ub; i28++) {
    d_Phi->data[i28] = Phi->data[i28] + P_xs_apr->data[i28] / 2.0;
  }

  b_emxInit_real_T(&x3, 1);
  emxInit_real_T(&Phi3, 2);
  b_dxdt_dPdt(dt, meas_1, xx, c_P_xx_apr, d_Phi, b_Q, x3, P3, Phi3);
  emxFree_real_T(&d_Phi);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i28 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i28, (int)sizeof(double));
  for (i28 = 0; i28 < loop_ub; i28++) {
    xx->data[i28] = x->data[i28] + x3->data[i28];
  }

  for (i28 = 0; i28 < 4; i28++) {
    b_xx[i28] = xx->data[3 + i28];
  }

  c = b_norm(b_xx);
  for (i28 = 0; i28 < 4; i28++) {
    b_xx[i28] = xx->data[3 + i28] / c;
  }

  for (i28 = 0; i28 < 4; i28++) {
    xx->data[3 + i28] = b_xx[i28];
  }

  for (i = 0; i < 6; i++) {
    b_meas_1[i] = meas_1[i] + (meas_0[i] - meas_1[i]);
  }

  for (i28 = 0; i28 < 144; i28++) {
    c_P_xx_apr[i28] = P_xx_apr->data[i28] + P3[i28];
  }

  emxInit_real_T(&e_Phi, 2);
  i28 = e_Phi->size[0] * e_Phi->size[1];
  e_Phi->size[0] = Phi->size[0];
  e_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)e_Phi, i28, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i28 = 0; i28 < loop_ub; i28++) {
    e_Phi->data[i28] = Phi->data[i28] + Phi3->data[i28];
  }

  b_emxInit_real_T(&x4, 1);
  emxInit_real_T(&Phi4, 2);
  b_dxdt_dPdt(dt, b_meas_1, xx, c_P_xx_apr, e_Phi, b_Q, x4, P4, Phi4);
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

  b_emxInit_int32_T(&r16, 2);
  i28 = r16->size[0] * r16->size[1];
  r16->size[0] = 1;
  r16->size[1] = i;
  emxEnsureCapacity((emxArray__common *)r16, i28, (int)sizeof(int));
  for (i28 = 0; i28 < i; i28++) {
    r16->data[r16->size[0] * i28] = i28;
  }

  b_emxInit_real_T(&c_x, 1);
  i28 = c_x->size[0];
  c_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_x, i28, (int)sizeof(double));
  for (i28 = 0; i28 < loop_ub; i28++) {
    c_x->data[i28] = x->data[i28] + (((x1->data[i28] + 2.0 * x2->data[i28]) +
      2.0 * x3->data[i28]) + x4->data[i28]) / 6.0;
  }

  emxFree_real_T(&x4);
  emxFree_real_T(&x3);
  emxFree_real_T(&x2);
  emxFree_real_T(&x1);
  loop_ub = r16->size[1];
  for (i28 = 0; i28 < loop_ub; i28++) {
    x->data[r16->data[r16->size[0] * i28]] = c_x->data[(*(int (*)[2])r16->size)
      [0] * i28];
  }

  emxFree_real_T(&c_x);
  emxFree_int32_T(&r16);
  for (i28 = 0; i28 < 144; i28++) {
    d_P_xx_apr = P_xx_apr->data[i28] + (((b_P_xx_apr[i28] + 2.0 * P2[i28]) + 2.0
      * P3[i28]) + P4[i28]) / 6.0;
    b_P_xx_apr[i28] = d_P_xx_apr;
  }

  emxFree_real_T(&P_xx_apr);

  //  covariance of the state
  i28 = b_Phi->size[0] * b_Phi->size[1];
  b_Phi->size[0] = Phi->size[0];
  b_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)b_Phi, i28, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i28 = 0; i28 < loop_ub; i28++) {
    b_Phi->data[i28] = Phi->data[i28] + (((b_Phi->data[i28] + 2.0 *
      P_xs_apr->data[i28]) + 2.0 * Phi3->data[i28]) + Phi4->data[i28]) / 6.0;
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
    i28 = 0;
    i29 = 0;
  } else {
    i28 = (int)(c_numStates + 1.0) - 1;
    i29 = P_apo->size[1];
  }

  emxInit_real_T(&b_P_apo, 2);
  if ((b_Phi->size[1] == 1) || (loop_ub == 1)) {
    cr = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = loop_ub;
    b_P_apo->size[1] = i29 - i28;
    emxEnsureCapacity((emxArray__common *)b_P_apo, cr, (int)sizeof(double));
    i = i29 - i28;
    for (i29 = 0; i29 < i; i29++) {
      for (cr = 0; cr < loop_ub; cr++) {
        b_P_apo->data[cr + b_P_apo->size[0] * i29] = P_apo->data[cr +
          P_apo->size[0] * (i28 + i29)];
      }
    }

    i28 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i28, (int)sizeof(double));
    for (i28 = 0; i28 < 12; i28++) {
      loop_ub = b_P_apo->size[1];
      for (i29 = 0; i29 < loop_ub; i29++) {
        P_xs_apr->data[i28 + P_xs_apr->size[0] * i29] = 0.0;
        i = b_Phi->size[1];
        for (cr = 0; cr < i; cr++) {
          P_xs_apr->data[i28 + P_xs_apr->size[0] * i29] += b_Phi->data[i28 +
            b_Phi->size[0] * cr] * b_P_apo->data[cr + b_P_apo->size[0] * i29];
        }
      }
    }
  } else {
    k = b_Phi->size[1];
    cr = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, cr, (int)sizeof(double));
    cr = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = i29 - i28;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, cr, (int)sizeof(double));
    i = 12 * (i29 - i28);
    for (cr = 0; cr < i; cr++) {
      P_xs_apr->data[cr] = 0.0;
    }

    if (i29 - i28 == 0) {
    } else {
      i = 12 * ((i29 - i28) - 1);
      for (cr = 0; cr <= i; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= i; cr += 12) {
        ar = 0;
        i29 = br + k;
        for (ib = br; ib + 1 <= i29; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * (i28 +
               div_nzp_s32_floor(ib, loop_ub))] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                (i28 + div_nzp_s32_floor(ib, loop_ub))] * b_Phi->data[ia - 1];
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
  for (i28 = 0; i28 < 4; i28++) {
    b_xx[i28] = x->data[3 + i28] / c;
  }

  for (i28 = 0; i28 < 4; i28++) {
    x->data[3 + i28] = b_xx[i28];
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

  emxInit_int32_T(&r17, 1);
  i28 = r17->size[0];
  r17->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r17, i28, (int)sizeof(int));
  for (i28 = 0; i28 < loop_ub; i28++) {
    r17->data[i28] = i28;
  }

  emxInit_int32_T(&r18, 1);
  i28 = r18->size[0];
  r18->size[0] = i;
  emxEnsureCapacity((emxArray__common *)r18, i28, (int)sizeof(int));
  for (i28 = 0; i28 < i; i28++) {
    r18->data[i28] = i28;
  }

  for (i28 = 0; i28 < 12; i28++) {
    for (i29 = 0; i29 < 12; i29++) {
      c_P_xx_apr[i29 + 12 * i28] = (b_P_xx_apr[i29 + 12 * i28] + b_P_xx_apr[i28
        + 12 * i29]) / 2.0;
    }
  }

  i = r17->size[0];
  cr = r18->size[0];
  for (i28 = 0; i28 < cr; i28++) {
    for (i29 = 0; i29 < i; i29++) {
      P_apo->data[r17->data[i29] + P_apo->size[0] * r18->data[i28]] =
        c_P_xx_apr[i29 + i * i28];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i28 = 0;
    i29 = 0;
  } else {
    i28 = (int)(c_numStates + 1.0) - 1;
    i29 = P_apo->size[1];
  }

  cr = r17->size[0];
  r17->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r17, cr, (int)sizeof(int));
  for (cr = 0; cr < loop_ub; cr++) {
    r17->data[cr] = cr;
  }

  cr = r18->size[0];
  r18->size[0] = i29 - i28;
  emxEnsureCapacity((emxArray__common *)r18, cr, (int)sizeof(int));
  loop_ub = i29 - i28;
  for (i29 = 0; i29 < loop_ub; i29++) {
    r18->data[i29] = i28 + i29;
  }

  i = r17->size[0];
  cr = r18->size[0];
  for (i28 = 0; i28 < cr; i28++) {
    for (i29 = 0; i29 < i; i29++) {
      P_apo->data[r17->data[i29] + P_apo->size[0] * r18->data[i28]] =
        P_xs_apr->data[i29 + i * i28];
    }
  }

  if (c_numStates + 1.0 > P_apo->size[0]) {
    i28 = 0;
    i29 = 0;
  } else {
    i28 = (int)(c_numStates + 1.0) - 1;
    i29 = P_apo->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  cr = r17->size[0];
  r17->size[0] = i29 - i28;
  emxEnsureCapacity((emxArray__common *)r17, cr, (int)sizeof(int));
  i = i29 - i28;
  for (i29 = 0; i29 < i; i29++) {
    r17->data[i29] = i28 + i29;
  }

  i28 = r18->size[0];
  r18->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r18, i28, (int)sizeof(int));
  for (i28 = 0; i28 < loop_ub; i28++) {
    r18->data[i28] = i28;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i28 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i28, (int)sizeof(double));
  for (i28 = 0; i28 < 12; i28++) {
    loop_ub = P_xs_apr->size[1];
    for (i29 = 0; i29 < loop_ub; i29++) {
      b_P_xs_apr->data[i29 + b_P_xs_apr->size[0] * i28] = P_xs_apr->data[i28 +
        P_xs_apr->size[0] * i29];
    }
  }

  emxFree_real_T(&P_xs_apr);
  i = r17->size[0];
  cr = r18->size[0];
  for (i28 = 0; i28 < cr; i28++) {
    for (i29 = 0; i29 < i; i29++) {
      P_apo->data[r17->data[i29] + P_apo->size[0] * r18->data[i28]] =
        b_P_xs_apr->data[i29 + i * i28];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r18);
  emxFree_int32_T(&r17);

  //  P_apr=(P_apr+P_apr')/2;
  // % =================================================================================================== 
}

//
// File trailer for SLAM_pred.cpp
//
// [EOF]
//
