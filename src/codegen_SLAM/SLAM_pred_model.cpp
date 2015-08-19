//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred_model.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_pred_model.h"
#include "SLAM_emxutil.h"
#include "norm.h"
#include "SLAM_updIT.h"
#include "diag.h"
#include "eye.h"
#include "SLAM_rtwutil.h"
#include <stdio.h>

// Function Declarations
static void b_dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T
  *x, const double P_xx_apr[144], const emxArray_real_T *Phi, const double
  b_processNoise[4], double b_model, emxArray_real_T *x_dot, double
  P_xx_apr_dot[144], emxArray_real_T *Phi_dot);
static void dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T *x,
                      const emxArray_real_T *P_xx_apr, const emxArray_real_T
                      *Phi, const double b_processNoise[4], double b_model,
                      emxArray_real_T *x_dot, double P_xx_apr_dot[144],
                      emxArray_real_T *Phi_dot);

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
//                const double b_processNoise[4]
//                double b_model
//                emxArray_real_T *x_dot
//                double P_xx_apr_dot[144]
//                emxArray_real_T *Phi_dot
// Return Type  : void
//
static void b_dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T
  *x, const double P_xx_apr[144], const emxArray_real_T *Phi, const double
  b_processNoise[4], double b_model, emxArray_real_T *x_dot, double
  P_xx_apr_dot[144], emxArray_real_T *Phi_dot)
{
  double w[3];
  int br;
  double acc_noise[3];
  unsigned int unnamed_idx_0;
  int loop_ub;
  double F[144];
  static const double dv11[144] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  };

  double a;
  double c;
  double c_processNoise[9];
  double d_processNoise[9];
  double Q[81];
  static const signed char iv7[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv8[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double b_x[3];
  double planar_velocity[3];
  static const signed char b_planar_velocity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  double dv12[16];
  double dv13[16];
  double c_x[4];
  double dv14[4];
  static const double b[3] = { 0.1, 0.1, 1.0E-7 };

  static const signed char iv9[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double d_x[9];
  double G[108];
  static const signed char iv10[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv11[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double b_G[108];
  double FP[144];
  int cr;
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
  // oa=x(14:16)
  for (br = 0; br < 3; br++) {
    w[br] = meas[br] - x->data[10 + br];
  }

  for (br = 0; br < 3; br++) {
    acc_noise[br] = b_processNoise[0];
  }

  unnamed_idx_0 = (unsigned int)x->size[0];
  br = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, br, (int)sizeof(double));
  loop_ub = (int)unnamed_idx_0;
  for (br = 0; br < loop_ub; br++) {
    x_dot->data[br] = 0.0;
  }

  if (b_model == 1.0) {
    //  stationary model
    memcpy(&F[0], &dv11[0], 144U * sizeof(double));
    a = 1.0E-6 * b_processNoise[1];
    c = dt * dt;
    for (br = 0; br < 3; br++) {
      c_processNoise[br] = a;
    }

    for (br = 0; br < 3; br++) {
      c_processNoise[br + 3] = 1.0E-6 * acc_noise[br];
    }

    for (br = 0; br < 3; br++) {
      c_processNoise[br + 6] = b_processNoise[2];
    }

    for (br = 0; br < 9; br++) {
      d_processNoise[br] = c_processNoise[br] * c;
    }

    b_diag(d_processNoise, Q);

    //  elseif model == 2 % rotation only
    //      F=[ O,   O,  O,  O;
    //          O,  -skew(w),   O, -I;
    //          O,   O,  O,  O;
    //          O,   O,  O,  O];
    //
    //
    //      x_dot(4:7)   = 0.5 * omega * x(4:7);      % rot angle
    //      Q = diag([qw * [1 1 1], model_uncertainty_factor * acc_noise, qwo * ones(1,3)]*(dt^2)); 
  } else if (b_model == 2.0) {
    //  planar motion
    for (br = 0; br < 2; br++) {
      w[br << 1] = 0.0;
    }

    c_processNoise[0] = 0.0;
    c_processNoise[3] = -w[2];
    c_processNoise[6] = w[1];
    c_processNoise[1] = w[2];
    c_processNoise[4] = 0.0;
    c_processNoise[7] = -w[0];
    c_processNoise[2] = -w[1];
    c_processNoise[5] = w[0];
    c_processNoise[8] = 0.0;
    for (br = 0; br < 12; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[loop_ub + 12 * br] = iv7[loop_ub + 3 * br];
      }
    }

    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * br) + 3] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * (br + 3)) + 3] = -c_processNoise[loop_ub + 3 * br];
      }
    }

    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * (br + 6)) + 3] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * (br + 9)) + 3] = iv8[loop_ub + 3 * br];
      }
    }

    for (br = 0; br < 12; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * br) + 6] = 0.0;
      }

      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * br) + 9] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      b_x[br] = x->data[7 + br];
    }

    for (br = 0; br < 3; br++) {
      planar_velocity[br] = 0.0;
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        planar_velocity[br] += (double)b_planar_velocity[br + 3 * loop_ub] *
          b_x[loop_ub];
      }
    }

    for (br = 0; br < 3; br++) {
      x_dot->data[br] = planar_velocity[br];
    }

    //  position
    d_processNoise[0] = 0.0;
    d_processNoise[3] = -w[2];
    d_processNoise[6] = w[1];
    d_processNoise[1] = w[2];
    d_processNoise[4] = 0.0;
    d_processNoise[7] = -w[0];
    d_processNoise[2] = -w[1];
    d_processNoise[5] = w[0];
    d_processNoise[8] = 0.0;
    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        dv12[loop_ub + (br << 2)] = -d_processNoise[loop_ub + 3 * br];
      }
    }

    for (br = 0; br < 3; br++) {
      dv12[12 + br] = w[br];
    }

    for (br = 0; br < 3; br++) {
      dv12[3 + (br << 2)] = -w[br];
    }

    dv12[15] = 0.0;
    for (br = 0; br < 4; br++) {
      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv13[loop_ub + (br << 2)] = 0.5 * dv12[loop_ub + (br << 2)];
      }
    }

    for (br = 0; br < 4; br++) {
      c_x[br] = x->data[3 + br];
    }

    for (br = 0; br < 4; br++) {
      dv14[br] = 0.0;
      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv14[br] += dv13[br + (loop_ub << 2)] * c_x[loop_ub];
      }
    }

    for (br = 0; br < 4; br++) {
      x_dot->data[3 + br] = dv14[br];
    }

    //  rot angle
    a = 0.1 * b_processNoise[1];
    c = dt * dt;
    for (br = 0; br < 3; br++) {
      c_processNoise[br] = a;
    }

    for (br = 0; br < 3; br++) {
      c_processNoise[br + 3] = b_processNoise[0] * b[br];
    }

    for (br = 0; br < 3; br++) {
      c_processNoise[br + 6] = b_processNoise[2];
    }

    for (br = 0; br < 9; br++) {
      d_processNoise[br] = c_processNoise[br] * c;
    }

    b_diag(d_processNoise, Q);

    //  elseif model == 4 % translation
    //      F=[ O,   O,  I,  O;
    //          O,  O,   O, -I;
    //          O,   O,  O,  O;
    //          O,   O,  O,  O];
    //
    //
    //      x_dot(1:3)   = x(8:10);                   % position
    //
    //      Q = diag([model_uncertainty_factor * qw * ones(1,3), qv * ones(1,3), qwo * ones(1,3)]*(dt^2)); 
    //  elseif model == 5 % translation, high uncertainty
    //          F=[ O,   O,  I,  O;
    //          O,  O,   O, -I;
    //          O,   O,  O,  O;
    //          O,   O,  O,  O];
    //
    //
    //      x_dot(1:3)   = x(8:10);                   % position
    //
    //      Q = diag([model_uncertainty_factor/factor * qw * ones(1,3), factor*qv * ones(1,3), qwo * ones(1,3)]*(dt^2)); 
  } else {
    //  general motion
    c_processNoise[0] = 0.0;
    c_processNoise[3] = -w[2];
    c_processNoise[6] = w[1];
    c_processNoise[1] = w[2];
    c_processNoise[4] = 0.0;
    c_processNoise[7] = -w[0];
    c_processNoise[2] = -w[1];
    c_processNoise[5] = w[0];
    c_processNoise[8] = 0.0;
    for (br = 0; br < 12; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[loop_ub + 12 * br] = iv9[loop_ub + 3 * br];
      }
    }

    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * br) + 3] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * (br + 3)) + 3] = -c_processNoise[loop_ub + 3 * br];
      }
    }

    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * (br + 6)) + 3] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * (br + 9)) + 3] = iv8[loop_ub + 3 * br];
      }
    }

    for (br = 0; br < 12; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * br) + 6] = 0.0;
      }

      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        F[(loop_ub + 12 * br) + 9] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      x_dot->data[br] = x->data[7 + br];
    }

    //  position
    d_processNoise[0] = 0.0;
    d_processNoise[3] = -w[2];
    d_processNoise[6] = w[1];
    d_processNoise[1] = w[2];
    d_processNoise[4] = 0.0;
    d_processNoise[7] = -w[0];
    d_processNoise[2] = -w[1];
    d_processNoise[5] = w[0];
    d_processNoise[8] = 0.0;
    for (br = 0; br < 3; br++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        dv12[loop_ub + (br << 2)] = -d_processNoise[loop_ub + 3 * br];
      }
    }

    for (br = 0; br < 3; br++) {
      dv12[12 + br] = w[br];
    }

    for (br = 0; br < 3; br++) {
      dv12[3 + (br << 2)] = -w[br];
    }

    dv12[15] = 0.0;
    for (br = 0; br < 4; br++) {
      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv13[loop_ub + (br << 2)] = 0.5 * dv12[loop_ub + (br << 2)];
      }
    }

    for (br = 0; br < 4; br++) {
      c_x[br] = x->data[3 + br];
    }

    for (br = 0; br < 4; br++) {
      dv14[br] = 0.0;
      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv14[br] += dv13[br + (loop_ub << 2)] * c_x[loop_ub];
      }
    }

    for (br = 0; br < 4; br++) {
      x_dot->data[3 + br] = dv14[br];
    }

    //  rot angle
    c = dt * dt;
    for (br = 0; br < 3; br++) {
      c_processNoise[br] = b_processNoise[1];
    }

    for (br = 0; br < 3; br++) {
      c_processNoise[br + 3] = acc_noise[br];
    }

    for (br = 0; br < 3; br++) {
      c_processNoise[br + 6] = b_processNoise[2];
    }

    for (br = 0; br < 9; br++) {
      d_processNoise[br] = c_processNoise[br] * c;
    }

    b_diag(d_processNoise, Q);

    //  elseif model == 5 % general motion, high uncertainty
    //      F=[ O,   O,  I,  O;
    //          O,  -skew(w),   O, -I;
    //          O,   O,  O,  O;
    //          O,   O,  O,  O];
    //
    //
    //      x_dot(1:3)   = x(8:10);                   % position
    //      x_dot(4:7)   = 0.5 * omega * x(4:7);      % rot angle
    //      Q = diag([factor*qw * ones(1,3), factor*qv * ones(1,3), qwo * ones(1,3)]*(dt^2)); 
  }

  //  F=[ O,   O,  I,  O;
  //      O,  -skew(w),   O, -I;
  //      O,   -R_cw'*skew(a),  O,  O;
  //      O,   O,  O,  O];
  //
  //
  //  x_dot(1:3)   = x(8:10);                   % position
  //  x_dot(4:7)   = 0.5 * omega * x(4:7);      % rot angle
  d_x[0] = ((x->data[3] * x->data[3] - x->data[4] * x->data[4]) - x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
  d_x[3] = 2.0 * (x->data[3] * x->data[4] + x->data[5] * x->data[6]);
  d_x[6] = 2.0 * (x->data[3] * x->data[5] - x->data[4] * x->data[6]);
  d_x[1] = 2.0 * (x->data[3] * x->data[4] - x->data[5] * x->data[6]);
  d_x[4] = ((-(x->data[3] * x->data[3]) + x->data[4] * x->data[4]) - x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
  d_x[7] = 2.0 * (x->data[4] * x->data[5] + x->data[3] * x->data[6]);
  d_x[2] = 2.0 * (x->data[3] * x->data[5] + x->data[4] * x->data[6]);
  d_x[5] = 2.0 * (x->data[4] * x->data[5] - x->data[3] * x->data[6]);
  d_x[8] = ((-(x->data[3] * x->data[3]) - x->data[4] * x->data[4]) + x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
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
      G[(loop_ub + 12 * (br + 3)) + 6] = -d_x[loop_ub + 3 * br];
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
      c = 0.0;
      for (cr = 0; cr < 9; cr++) {
        c += b_G[br + 12 * cr] * G[loop_ub + 12 * cr];
      }

      b_FP[br + 12 * loop_ub] = (FP[br + 12 * loop_ub] + FP[loop_ub + 12 * br])
        + c;
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
      for (ic = cr + 1; ic <= cr + 12; ic++) {
        Phi_dot->data[ic - 1] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= loop_ub; cr += 12) {
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
  loop_ub = Phi_dot->size[0];
  cr = Phi_dot->size[1];
  loop_ub *= cr;
  for (br = 0; br < loop_ub; br++) {
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
//                const double b_processNoise[4]
//                double b_model
//                emxArray_real_T *x_dot
//                double P_xx_apr_dot[144]
//                emxArray_real_T *Phi_dot
// Return Type  : void
//
static void dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T *x,
                      const emxArray_real_T *P_xx_apr, const emxArray_real_T
                      *Phi, const double b_processNoise[4], double b_model,
                      emxArray_real_T *x_dot, double P_xx_apr_dot[144],
                      emxArray_real_T *Phi_dot)
{
  double w[3];
  int j;
  double acc_noise[3];
  unsigned int unnamed_idx_0;
  int cr;
  double F[144];
  static const double dv7[144] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  };

  double a;
  double c;
  double b_a[9];
  double y[9];
  double Q[81];
  double c_processNoise[9];
  int br;
  static const signed char iv2[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv3[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double b_x[3];
  double planar_velocity[3];
  static const signed char b_planar_velocity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  double dv8[16];
  double dv9[16];
  double c_x[4];
  double dv10[4];
  static const double b[3] = { 0.1, 0.1, 1.0E-7 };

  static const signed char iv4[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double d_x[9];
  double G[108];
  static const signed char iv5[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv6[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  emxArray_real_T *FP;
  int ar;
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
  // oa=x(14:16)
  for (j = 0; j < 3; j++) {
    w[j] = meas[j] - x->data[10 + j];
  }

  for (j = 0; j < 3; j++) {
    acc_noise[j] = b_processNoise[0];
  }

  unnamed_idx_0 = (unsigned int)x->size[0];
  j = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, j, (int)sizeof(double));
  cr = (int)unnamed_idx_0;
  for (j = 0; j < cr; j++) {
    x_dot->data[j] = 0.0;
  }

  if (b_model == 1.0) {
    //  stationary model
    memcpy(&F[0], &dv7[0], 144U * sizeof(double));
    a = 1.0E-6 * b_processNoise[1];
    c = dt * dt;
    for (j = 0; j < 3; j++) {
      b_a[j] = a;
    }

    for (j = 0; j < 3; j++) {
      b_a[j + 3] = 1.0E-6 * acc_noise[j];
    }

    for (j = 0; j < 3; j++) {
      b_a[j + 6] = b_processNoise[2];
    }

    for (j = 0; j < 9; j++) {
      y[j] = b_a[j] * c;
    }

    memset(&Q[0], 0, 81U * sizeof(double));
    for (j = 0; j < 9; j++) {
      Q[j + 9 * j] = y[j];
    }

    //  elseif model == 2 % rotation only
    //      F=[ O,   O,  O,  O;
    //          O,  -skew(w),   O, -I;
    //          O,   O,  O,  O;
    //          O,   O,  O,  O];
    //
    //
    //      x_dot(4:7)   = 0.5 * omega * x(4:7);      % rot angle
    //      Q = diag([qw * [1 1 1], model_uncertainty_factor * acc_noise, qwo * ones(1,3)]*(dt^2)); 
  } else if (b_model == 2.0) {
    //  planar motion
    for (j = 0; j < 2; j++) {
      w[j << 1] = 0.0;
    }

    c_processNoise[0] = 0.0;
    c_processNoise[3] = -w[2];
    c_processNoise[6] = w[1];
    c_processNoise[1] = w[2];
    c_processNoise[4] = 0.0;
    c_processNoise[7] = -w[0];
    c_processNoise[2] = -w[1];
    c_processNoise[5] = w[0];
    c_processNoise[8] = 0.0;
    for (j = 0; j < 12; j++) {
      for (br = 0; br < 3; br++) {
        F[br + 12 * j] = iv2[br + 3 * j];
      }
    }

    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * j) + 3] = 0.0;
      }
    }

    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * (j + 3)) + 3] = -c_processNoise[br + 3 * j];
      }
    }

    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * (j + 6)) + 3] = 0.0;
      }
    }

    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * (j + 9)) + 3] = iv3[br + 3 * j];
      }
    }

    for (j = 0; j < 12; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * j) + 6] = 0.0;
      }

      for (br = 0; br < 3; br++) {
        F[(br + 12 * j) + 9] = 0.0;
      }
    }

    for (j = 0; j < 3; j++) {
      b_x[j] = x->data[7 + j];
    }

    for (j = 0; j < 3; j++) {
      planar_velocity[j] = 0.0;
      for (br = 0; br < 3; br++) {
        planar_velocity[j] += (double)b_planar_velocity[j + 3 * br] * b_x[br];
      }
    }

    for (j = 0; j < 3; j++) {
      x_dot->data[j] = planar_velocity[j];
    }

    //  position
    b_a[0] = 0.0;
    b_a[3] = -w[2];
    b_a[6] = w[1];
    b_a[1] = w[2];
    b_a[4] = 0.0;
    b_a[7] = -w[0];
    b_a[2] = -w[1];
    b_a[5] = w[0];
    b_a[8] = 0.0;
    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        dv8[br + (j << 2)] = -b_a[br + 3 * j];
      }
    }

    for (j = 0; j < 3; j++) {
      dv8[12 + j] = w[j];
    }

    for (j = 0; j < 3; j++) {
      dv8[3 + (j << 2)] = -w[j];
    }

    dv8[15] = 0.0;
    for (j = 0; j < 4; j++) {
      for (br = 0; br < 4; br++) {
        dv9[br + (j << 2)] = 0.5 * dv8[br + (j << 2)];
      }
    }

    for (j = 0; j < 4; j++) {
      c_x[j] = x->data[3 + j];
    }

    for (j = 0; j < 4; j++) {
      dv10[j] = 0.0;
      for (br = 0; br < 4; br++) {
        dv10[j] += dv9[j + (br << 2)] * c_x[br];
      }
    }

    for (j = 0; j < 4; j++) {
      x_dot->data[3 + j] = dv10[j];
    }

    //  rot angle
    a = 0.1 * b_processNoise[1];
    c = dt * dt;
    for (j = 0; j < 3; j++) {
      b_a[j] = a;
    }

    for (j = 0; j < 3; j++) {
      b_a[j + 3] = b_processNoise[0] * b[j];
    }

    for (j = 0; j < 3; j++) {
      b_a[j + 6] = b_processNoise[2];
    }

    for (j = 0; j < 9; j++) {
      y[j] = b_a[j] * c;
    }

    memset(&Q[0], 0, 81U * sizeof(double));
    for (j = 0; j < 9; j++) {
      Q[j + 9 * j] = y[j];
    }

    //  elseif model == 4 % translation
    //      F=[ O,   O,  I,  O;
    //          O,  O,   O, -I;
    //          O,   O,  O,  O;
    //          O,   O,  O,  O];
    //
    //
    //      x_dot(1:3)   = x(8:10);                   % position
    //
    //      Q = diag([model_uncertainty_factor * qw * ones(1,3), qv * ones(1,3), qwo * ones(1,3)]*(dt^2)); 
    //  elseif model == 5 % translation, high uncertainty
    //          F=[ O,   O,  I,  O;
    //          O,  O,   O, -I;
    //          O,   O,  O,  O;
    //          O,   O,  O,  O];
    //
    //
    //      x_dot(1:3)   = x(8:10);                   % position
    //
    //      Q = diag([model_uncertainty_factor/factor * qw * ones(1,3), factor*qv * ones(1,3), qwo * ones(1,3)]*(dt^2)); 
  } else {
    //  general motion
    c_processNoise[0] = 0.0;
    c_processNoise[3] = -w[2];
    c_processNoise[6] = w[1];
    c_processNoise[1] = w[2];
    c_processNoise[4] = 0.0;
    c_processNoise[7] = -w[0];
    c_processNoise[2] = -w[1];
    c_processNoise[5] = w[0];
    c_processNoise[8] = 0.0;
    for (j = 0; j < 12; j++) {
      for (br = 0; br < 3; br++) {
        F[br + 12 * j] = iv4[br + 3 * j];
      }
    }

    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * j) + 3] = 0.0;
      }
    }

    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * (j + 3)) + 3] = -c_processNoise[br + 3 * j];
      }
    }

    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * (j + 6)) + 3] = 0.0;
      }
    }

    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * (j + 9)) + 3] = iv3[br + 3 * j];
      }
    }

    for (j = 0; j < 12; j++) {
      for (br = 0; br < 3; br++) {
        F[(br + 12 * j) + 6] = 0.0;
      }

      for (br = 0; br < 3; br++) {
        F[(br + 12 * j) + 9] = 0.0;
      }
    }

    for (j = 0; j < 3; j++) {
      x_dot->data[j] = x->data[7 + j];
    }

    //  position
    b_a[0] = 0.0;
    b_a[3] = -w[2];
    b_a[6] = w[1];
    b_a[1] = w[2];
    b_a[4] = 0.0;
    b_a[7] = -w[0];
    b_a[2] = -w[1];
    b_a[5] = w[0];
    b_a[8] = 0.0;
    for (j = 0; j < 3; j++) {
      for (br = 0; br < 3; br++) {
        dv8[br + (j << 2)] = -b_a[br + 3 * j];
      }
    }

    for (j = 0; j < 3; j++) {
      dv8[12 + j] = w[j];
    }

    for (j = 0; j < 3; j++) {
      dv8[3 + (j << 2)] = -w[j];
    }

    dv8[15] = 0.0;
    for (j = 0; j < 4; j++) {
      for (br = 0; br < 4; br++) {
        dv9[br + (j << 2)] = 0.5 * dv8[br + (j << 2)];
      }
    }

    for (j = 0; j < 4; j++) {
      c_x[j] = x->data[3 + j];
    }

    for (j = 0; j < 4; j++) {
      dv10[j] = 0.0;
      for (br = 0; br < 4; br++) {
        dv10[j] += dv9[j + (br << 2)] * c_x[br];
      }
    }

    for (j = 0; j < 4; j++) {
      x_dot->data[3 + j] = dv10[j];
    }

    //  rot angle
    c = dt * dt;
    for (j = 0; j < 3; j++) {
      c_processNoise[j] = b_processNoise[1];
    }

    for (j = 0; j < 3; j++) {
      c_processNoise[j + 3] = acc_noise[j];
    }

    for (j = 0; j < 3; j++) {
      c_processNoise[j + 6] = b_processNoise[2];
    }

    for (j = 0; j < 9; j++) {
      y[j] = c_processNoise[j] * c;
    }

    memset(&Q[0], 0, 81U * sizeof(double));
    for (j = 0; j < 9; j++) {
      Q[j + 9 * j] = y[j];
    }

    //  elseif model == 5 % general motion, high uncertainty
    //      F=[ O,   O,  I,  O;
    //          O,  -skew(w),   O, -I;
    //          O,   O,  O,  O;
    //          O,   O,  O,  O];
    //
    //
    //      x_dot(1:3)   = x(8:10);                   % position
    //      x_dot(4:7)   = 0.5 * omega * x(4:7);      % rot angle
    //      Q = diag([factor*qw * ones(1,3), factor*qv * ones(1,3), qwo * ones(1,3)]*(dt^2)); 
  }

  //  F=[ O,   O,  I,  O;
  //      O,  -skew(w),   O, -I;
  //      O,   -R_cw'*skew(a),  O,  O;
  //      O,   O,  O,  O];
  //
  //
  //  x_dot(1:3)   = x(8:10);                   % position
  //  x_dot(4:7)   = 0.5 * omega * x(4:7);      % rot angle
  d_x[0] = ((x->data[3] * x->data[3] - x->data[4] * x->data[4]) - x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
  d_x[3] = 2.0 * (x->data[3] * x->data[4] + x->data[5] * x->data[6]);
  d_x[6] = 2.0 * (x->data[3] * x->data[5] - x->data[4] * x->data[6]);
  d_x[1] = 2.0 * (x->data[3] * x->data[4] - x->data[5] * x->data[6]);
  d_x[4] = ((-(x->data[3] * x->data[3]) + x->data[4] * x->data[4]) - x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
  d_x[7] = 2.0 * (x->data[4] * x->data[5] + x->data[3] * x->data[6]);
  d_x[2] = 2.0 * (x->data[3] * x->data[5] + x->data[4] * x->data[6]);
  d_x[5] = 2.0 * (x->data[4] * x->data[5] - x->data[3] * x->data[6]);
  d_x[8] = ((-(x->data[3] * x->data[3]) - x->data[4] * x->data[4]) + x->data[5] *
            x->data[5]) + x->data[6] * x->data[6];
  for (j = 0; j < 9; j++) {
    for (br = 0; br < 3; br++) {
      G[br + 12 * j] = 0.0;
    }

    for (br = 0; br < 3; br++) {
      G[(br + 12 * j) + 3] = iv5[br + 3 * j];
    }
  }

  for (j = 0; j < 3; j++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * j) + 6] = 0.0;
    }
  }

  for (j = 0; j < 3; j++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * (j + 3)) + 6] = -d_x[br + 3 * j];
    }
  }

  for (j = 0; j < 3; j++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * (j + 6)) + 6] = 0.0;
    }
  }

  for (j = 0; j < 9; j++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * j) + 9] = iv6[br + 3 * j];
    }
  }

  // time derivative of the state
  j = x_dot->size[0];
  emxEnsureCapacity((emxArray__common *)x_dot, j, (int)sizeof(double));
  cr = x_dot->size[0];
  for (j = 0; j < cr; j++) {
    x_dot->data[j] *= dt;
  }

  // time derivative of the covariance
  b_emxInit_real_T(&FP, 2);
  if (P_xx_apr->size[0] == 1) {
    j = FP->size[0] * FP->size[1];
    FP->size[0] = 12;
    FP->size[1] = P_xx_apr->size[1];
    emxEnsureCapacity((emxArray__common *)FP, j, (int)sizeof(double));
    for (j = 0; j < 12; j++) {
      cr = P_xx_apr->size[1];
      for (br = 0; br < cr; br++) {
        FP->data[j + FP->size[0] * br] = 0.0;
        for (ar = 0; ar < 12; ar++) {
          FP->data[j + FP->size[0] * br] += F[j + 12 * ar] * P_xx_apr->data[ar +
            P_xx_apr->size[0] * br];
        }
      }
    }
  } else {
    unnamed_idx_0 = (unsigned int)P_xx_apr->size[1];
    j = FP->size[0] * FP->size[1];
    FP->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)FP, j, (int)sizeof(double));
    j = FP->size[0] * FP->size[1];
    FP->size[1] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)FP, j, (int)sizeof(double));
    cr = 12 * (int)unnamed_idx_0;
    for (j = 0; j < cr; j++) {
      FP->data[j] = 0.0;
    }

    if (P_xx_apr->size[1] == 0) {
    } else {
      j = 12 * (P_xx_apr->size[1] - 1);
      for (cr = 0; cr <= j; cr += 12) {
        for (ic = cr + 1; ic <= cr + 12; ic++) {
          FP->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= j; cr += 12) {
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

  b_emxInit_real_T(&b_FP, 2);
  j = b_FP->size[0] * b_FP->size[1];
  b_FP->size[0] = FP->size[1];
  b_FP->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_FP, j, (int)sizeof(double));
  for (j = 0; j < 12; j++) {
    cr = FP->size[1];
    for (br = 0; br < cr; br++) {
      b_FP->data[br + b_FP->size[0] * j] = FP->data[j + FP->size[0] * br];
    }
  }

  for (j = 0; j < 12; j++) {
    for (br = 0; br < 9; br++) {
      b_G[j + 12 * br] = 0.0;
      for (ar = 0; ar < 9; ar++) {
        b_G[j + 12 * br] += G[j + 12 * ar] * Q[ar + 9 * br];
      }
    }
  }

  for (j = 0; j < 12; j++) {
    for (br = 0; br < 12; br++) {
      c = 0.0;
      for (ar = 0; ar < 9; ar++) {
        c += b_G[j + 12 * ar] * G[br + 12 * ar];
      }

      c_FP[j + 12 * br] = (FP->data[j + 12 * br] + b_FP->data[j + 12 * br]) + c;
    }
  }

  emxFree_real_T(&b_FP);
  emxFree_real_T(&FP);
  for (j = 0; j < 12; j++) {
    for (br = 0; br < 12; br++) {
      P_xx_apr_dot[br + 12 * j] = c_FP[br + 12 * j] * dt;
    }
  }

  // time derivative of the state transition
  if (Phi->size[0] == 1) {
    j = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[0] = 12;
    Phi_dot->size[1] = Phi->size[1];
    emxEnsureCapacity((emxArray__common *)Phi_dot, j, (int)sizeof(double));
    for (j = 0; j < 12; j++) {
      cr = Phi->size[1];
      for (br = 0; br < cr; br++) {
        Phi_dot->data[j + Phi_dot->size[0] * br] = 0.0;
        for (ar = 0; ar < 12; ar++) {
          Phi_dot->data[j + Phi_dot->size[0] * br] += F[j + 12 * ar] * Phi->
            data[ar + Phi->size[0] * br];
        }
      }
    }
  } else {
    unnamed_idx_0 = (unsigned int)Phi->size[1];
    j = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)Phi_dot, j, (int)sizeof(double));
    j = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[1] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)Phi_dot, j, (int)sizeof(double));
    cr = 12 * (int)unnamed_idx_0;
    for (j = 0; j < cr; j++) {
      Phi_dot->data[j] = 0.0;
    }

    if (Phi->size[1] == 0) {
    } else {
      j = 12 * (Phi->size[1] - 1);
      for (cr = 0; cr <= j; cr += 12) {
        for (ic = cr + 1; ic <= cr + 12; ic++) {
          Phi_dot->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= j; cr += 12) {
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

  j = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[0] = 12;
  emxEnsureCapacity((emxArray__common *)Phi_dot, j, (int)sizeof(double));
  j = Phi_dot->size[0];
  cr = Phi_dot->size[1];
  cr *= j;
  for (j = 0; j < cr; j++) {
    Phi_dot->data[j] *= dt;
  }
}

//
// EKF_SLAM: computes the camerapose p and feature position f in an
// interative way
//    Handels a static number of points but can dynamically asign them to new
//    klt points.
// Arguments    : struct_T *b_SLAM_data
//                double dt
//                const double b_processNoise[4]
//                const double IMU_measurements[23]
// Return Type  : void
//
void SLAM_pred_model(struct_T *b_SLAM_data, double dt, const double
                     b_processNoise[4], const double IMU_measurements[23])
{
  emxArray_real_T *b_xt;
  int i33;
  int loop_ub;
  emxArray_real_T *b_P;
  int i;
  emxArray_real_T *P_xx_apr;
  int i34;
  emxArray_real_T *Phi;
  double meas_0[6];
  emxArray_real_T *c_SLAM_data;
  emxArray_real_T *x1;
  emxArray_real_T *b_Phi;
  double b_P_xx_apr[144];
  double meas_1[6];
  emxArray_real_T *xx;
  double b_xx[4];
  double B;
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
  emxArray_int32_T *r38;
  emxArray_int32_T *r39;
  double d_P_xx_apr;
  emxArray_real_T *d_SLAM_data;
  int cr;
  int k;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  emxArray_int32_T *r40;
  emxArray_int32_T *r41;
  emxArray_real_T *b_P_xs_apr;
  emxInit_real_T(&b_xt, 1);

  //  Xv meaning
  //
  //                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
  //  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
  //  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  //  numStatesFeatures=numAnchors*(7+numPointsPerAnchor);
  //  %% Iterative Camera Pose optimization (EKF)
  i33 = b_xt->size[0];
  b_xt->size[0] = b_SLAM_data->xt->size[0];
  emxEnsureCapacity((emxArray__common *)b_xt, i33, (int)sizeof(double));
  loop_ub = b_SLAM_data->xt->size[0];
  for (i33 = 0; i33 < loop_ub; i33++) {
    b_xt->data[i33] = b_SLAM_data->xt->data[i33];
  }

  b_emxInit_real_T(&b_P, 2);
  i33 = b_P->size[0] * b_P->size[1];
  b_P->size[0] = b_SLAM_data->P->size[0];
  b_P->size[1] = b_SLAM_data->P->size[1];
  emxEnsureCapacity((emxArray__common *)b_P, i33, (int)sizeof(double));
  loop_ub = b_SLAM_data->P->size[0] * b_SLAM_data->P->size[1];
  for (i33 = 0; i33 < loop_ub; i33++) {
    b_P->data[i33] = b_SLAM_data->P->data[i33];
  }

  // % compute the linearization F of the non linear model f
  if (1.0 > b_SLAM_data->numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)b_SLAM_data->numStates;
  }

  if (1.0 > b_SLAM_data->numStates) {
    i = 0;
  } else {
    i = (int)b_SLAM_data->numStates;
  }

  b_emxInit_real_T(&P_xx_apr, 2);
  i33 = P_xx_apr->size[0] * P_xx_apr->size[1];
  P_xx_apr->size[0] = loop_ub;
  P_xx_apr->size[1] = i;
  emxEnsureCapacity((emxArray__common *)P_xx_apr, i33, (int)sizeof(double));
  for (i33 = 0; i33 < i; i33++) {
    for (i34 = 0; i34 < loop_ub; i34++) {
      P_xx_apr->data[i34 + P_xx_apr->size[0] * i33] = b_SLAM_data->P->data[i34 +
        b_SLAM_data->P->size[0] * i33];
    }
  }

  b_emxInit_real_T(&Phi, 2);
  eye(b_SLAM_data->numStates, Phi);
  for (i = 0; i < 3; i++) {
    meas_0[i] = IMU_measurements[i];
  }

  for (i = 0; i < 3; i++) {
    meas_0[i + 3] = IMU_measurements[i + 3];
  }

  if (1.0 > b_SLAM_data->numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(b_SLAM_data->numStates + 1.0);
  }

  emxInit_real_T(&c_SLAM_data, 1);
  i33 = c_SLAM_data->size[0];
  c_SLAM_data->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_SLAM_data, i33, (int)sizeof(double));
  for (i33 = 0; i33 < loop_ub; i33++) {
    c_SLAM_data->data[i33] = b_SLAM_data->xt->data[i33];
  }

  emxInit_real_T(&x1, 1);
  b_emxInit_real_T(&b_Phi, 2);
  dxdt_dPdt(dt, meas_0, c_SLAM_data, P_xx_apr, Phi, b_processNoise,
            b_SLAM_data->model, x1, b_P_xx_apr, b_Phi);
  emxFree_real_T(&c_SLAM_data);
  for (i = 0; i < 6; i++) {
    meas_1[i] = meas_0[i] + (meas_0[i] - meas_0[i]) * 0.5;
  }

  if (1.0 > b_SLAM_data->numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(b_SLAM_data->numStates + 1.0);
  }

  emxInit_real_T(&xx, 1);
  i33 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i33, (int)sizeof(double));
  for (i33 = 0; i33 < loop_ub; i33++) {
    xx->data[i33] = b_SLAM_data->xt->data[i33] + x1->data[i33] / 2.0;
  }

  for (i33 = 0; i33 < 4; i33++) {
    b_xx[i33] = xx->data[3 + i33];
  }

  B = b_norm(b_xx);
  for (i33 = 0; i33 < 4; i33++) {
    b_xx[i33] = xx->data[3 + i33] / B;
  }

  for (i33 = 0; i33 < 4; i33++) {
    xx->data[3 + i33] = b_xx[i33];
  }

  for (i33 = 0; i33 < 144; i33++) {
    c_P_xx_apr[i33] = P_xx_apr->data[i33] + b_P_xx_apr[i33] / 2.0;
  }

  b_emxInit_real_T(&c_Phi, 2);
  i33 = c_Phi->size[0] * c_Phi->size[1];
  c_Phi->size[0] = Phi->size[0];
  c_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)c_Phi, i33, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i33 = 0; i33 < loop_ub; i33++) {
    c_Phi->data[i33] = Phi->data[i33] + b_Phi->data[i33] / 2.0;
  }

  b_emxInit_real_T(&P_xs_apr, 2);
  emxInit_real_T(&x2, 1);
  b_dxdt_dPdt(dt, meas_1, xx, c_P_xx_apr, c_Phi, b_processNoise,
              b_SLAM_data->model, x2, P2, P_xs_apr);
  emxFree_real_T(&c_Phi);
  for (i33 = 0; i33 < 6; i33++) {
    meas_1[i33] += (meas_0[i33] - meas_1[i33]) * 0.5;
  }

  if (1.0 > b_SLAM_data->numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(b_SLAM_data->numStates + 1.0);
  }

  i33 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i33, (int)sizeof(double));
  for (i33 = 0; i33 < loop_ub; i33++) {
    xx->data[i33] = b_SLAM_data->xt->data[i33] + x2->data[i33] / 2.0;
  }

  for (i33 = 0; i33 < 4; i33++) {
    b_xx[i33] = xx->data[3 + i33];
  }

  B = b_norm(b_xx);
  for (i33 = 0; i33 < 4; i33++) {
    b_xx[i33] = xx->data[3 + i33] / B;
  }

  for (i33 = 0; i33 < 4; i33++) {
    xx->data[3 + i33] = b_xx[i33];
  }

  for (i33 = 0; i33 < 144; i33++) {
    c_P_xx_apr[i33] = P_xx_apr->data[i33] + P2[i33] / 2.0;
  }

  b_emxInit_real_T(&d_Phi, 2);
  i33 = d_Phi->size[0] * d_Phi->size[1];
  d_Phi->size[0] = Phi->size[0];
  d_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)d_Phi, i33, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i33 = 0; i33 < loop_ub; i33++) {
    d_Phi->data[i33] = Phi->data[i33] + P_xs_apr->data[i33] / 2.0;
  }

  emxInit_real_T(&x3, 1);
  b_emxInit_real_T(&Phi3, 2);
  b_dxdt_dPdt(dt, meas_1, xx, c_P_xx_apr, d_Phi, b_processNoise,
              b_SLAM_data->model, x3, P3, Phi3);
  emxFree_real_T(&d_Phi);
  if (1.0 > b_SLAM_data->numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(b_SLAM_data->numStates + 1.0);
  }

  i33 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i33, (int)sizeof(double));
  for (i33 = 0; i33 < loop_ub; i33++) {
    xx->data[i33] = b_SLAM_data->xt->data[i33] + x3->data[i33];
  }

  for (i33 = 0; i33 < 4; i33++) {
    b_xx[i33] = xx->data[3 + i33];
  }

  B = b_norm(b_xx);
  for (i33 = 0; i33 < 4; i33++) {
    b_xx[i33] = xx->data[3 + i33] / B;
  }

  for (i33 = 0; i33 < 4; i33++) {
    xx->data[3 + i33] = b_xx[i33];
  }

  for (i = 0; i < 6; i++) {
    b_meas_1[i] = meas_1[i] + (meas_0[i] - meas_1[i]);
  }

  for (i33 = 0; i33 < 144; i33++) {
    c_P_xx_apr[i33] = P_xx_apr->data[i33] + P3[i33];
  }

  b_emxInit_real_T(&e_Phi, 2);
  i33 = e_Phi->size[0] * e_Phi->size[1];
  e_Phi->size[0] = Phi->size[0];
  e_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)e_Phi, i33, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i33 = 0; i33 < loop_ub; i33++) {
    e_Phi->data[i33] = Phi->data[i33] + Phi3->data[i33];
  }

  emxInit_real_T(&x4, 1);
  b_emxInit_real_T(&Phi4, 2);
  b_dxdt_dPdt(dt, b_meas_1, xx, c_P_xx_apr, e_Phi, b_processNoise,
              b_SLAM_data->model, x4, P4, Phi4);
  emxFree_real_T(&e_Phi);
  emxFree_real_T(&xx);
  if (1.0 > b_SLAM_data->numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(b_SLAM_data->numStates + 1.0);
  }

  if (1.0 > b_SLAM_data->numStates + 1.0) {
    i = 0;
  } else {
    i = (int)(b_SLAM_data->numStates + 1.0);
  }

  b_emxInit_int32_T(&r38, 2);
  i33 = r38->size[0] * r38->size[1];
  r38->size[0] = 1;
  r38->size[1] = i;
  emxEnsureCapacity((emxArray__common *)r38, i33, (int)sizeof(int));
  for (i33 = 0; i33 < i; i33++) {
    r38->data[r38->size[0] * i33] = i33;
  }

  emxInit_int32_T(&r39, 1);
  i33 = r39->size[0];
  r39->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r39, i33, (int)sizeof(int));
  for (i33 = 0; i33 < loop_ub; i33++) {
    r39->data[i33] = 1 + i33;
  }

  loop_ub = r38->size[0] * r38->size[1];
  for (i33 = 0; i33 < loop_ub; i33++) {
    b_xt->data[r38->data[i33]] = b_SLAM_data->xt->data[r39->data[i33] - 1] +
      (((x1->data[i33] + 2.0 * x2->data[i33]) + 2.0 * x3->data[i33]) + x4->
       data[i33]) / 6.0;
  }

  emxFree_int32_T(&r39);
  emxFree_int32_T(&r38);
  emxFree_real_T(&x4);
  emxFree_real_T(&x3);
  emxFree_real_T(&x2);
  emxFree_real_T(&x1);
  for (i33 = 0; i33 < 144; i33++) {
    d_P_xx_apr = P_xx_apr->data[i33] + (((b_P_xx_apr[i33] + 2.0 * P2[i33]) + 2.0
      * P3[i33]) + P4[i33]) / 6.0;
    b_P_xx_apr[i33] = d_P_xx_apr;
  }

  emxFree_real_T(&P_xx_apr);

  //  covariance of the state
  i33 = b_Phi->size[0] * b_Phi->size[1];
  b_Phi->size[0] = Phi->size[0];
  b_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)b_Phi, i33, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i33 = 0; i33 < loop_ub; i33++) {
    b_Phi->data[i33] = Phi->data[i33] + (((b_Phi->data[i33] + 2.0 *
      P_xs_apr->data[i33]) + 2.0 * Phi3->data[i33]) + Phi4->data[i33]) / 6.0;
  }

  emxFree_real_T(&Phi4);
  emxFree_real_T(&Phi3);
  emxFree_real_T(&Phi);
  if (1.0 > b_SLAM_data->numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)b_SLAM_data->numStates;
  }

  if (b_SLAM_data->numStates + 1.0 > b_SLAM_data->P->size[1]) {
    i33 = 0;
    i34 = 0;
  } else {
    i33 = (int)(b_SLAM_data->numStates + 1.0) - 1;
    i34 = b_SLAM_data->P->size[1];
  }

  b_emxInit_real_T(&d_SLAM_data, 2);
  if ((b_Phi->size[1] == 1) || (loop_ub == 1)) {
    cr = d_SLAM_data->size[0] * d_SLAM_data->size[1];
    d_SLAM_data->size[0] = loop_ub;
    d_SLAM_data->size[1] = i34 - i33;
    emxEnsureCapacity((emxArray__common *)d_SLAM_data, cr, (int)sizeof(double));
    i = i34 - i33;
    for (i34 = 0; i34 < i; i34++) {
      for (cr = 0; cr < loop_ub; cr++) {
        d_SLAM_data->data[cr + d_SLAM_data->size[0] * i34] = b_SLAM_data->
          P->data[cr + b_SLAM_data->P->size[0] * (i33 + i34)];
      }
    }

    i33 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = d_SLAM_data->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i33, (int)sizeof(double));
    for (i33 = 0; i33 < 12; i33++) {
      loop_ub = d_SLAM_data->size[1];
      for (i34 = 0; i34 < loop_ub; i34++) {
        P_xs_apr->data[i33 + P_xs_apr->size[0] * i34] = 0.0;
        i = b_Phi->size[1];
        for (cr = 0; cr < i; cr++) {
          P_xs_apr->data[i33 + P_xs_apr->size[0] * i34] += b_Phi->data[i33 +
            b_Phi->size[0] * cr] * d_SLAM_data->data[cr + d_SLAM_data->size[0] *
            i34];
        }
      }
    }
  } else {
    k = b_Phi->size[1];
    cr = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, cr, (int)sizeof(double));
    cr = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = i34 - i33;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, cr, (int)sizeof(double));
    i = 12 * (i34 - i33);
    for (cr = 0; cr < i; cr++) {
      P_xs_apr->data[cr] = 0.0;
    }

    if (i34 - i33 == 0) {
    } else {
      i = 12 * ((i34 - i33) - 1);
      for (cr = 0; cr <= i; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= i; cr += 12) {
        ar = 0;
        i34 = br + k;
        for (ib = br; ib + 1 <= i34; ib++) {
          if (b_SLAM_data->P->data[ib % loop_ub + b_SLAM_data->P->size[0] * (i33
               + div_nzp_s32_floor(ib, loop_ub))] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += b_SLAM_data->P->data[ib % loop_ub +
                b_SLAM_data->P->size[0] * (i33 + div_nzp_s32_floor(ib, loop_ub))]
                * b_Phi->data[ia - 1];
            }
          }

          ar += 12;
        }

        br += k;
      }
    }
  }

  emxFree_real_T(&d_SLAM_data);
  emxFree_real_T(&b_Phi);

  //  covariance between current state and trails
  B = b_norm(*(double (*)[4])&b_xt->data[3]);
  for (i33 = 0; i33 < 4; i33++) {
    b_xt->data[3 + i33] /= B;
  }

  if (1.0 > b_SLAM_data->numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)b_SLAM_data->numStates;
  }

  if (1.0 > b_SLAM_data->numStates) {
    i = 0;
  } else {
    i = (int)b_SLAM_data->numStates;
  }

  emxInit_int32_T(&r40, 1);
  i33 = r40->size[0];
  r40->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r40, i33, (int)sizeof(int));
  for (i33 = 0; i33 < loop_ub; i33++) {
    r40->data[i33] = i33;
  }

  emxInit_int32_T(&r41, 1);
  i33 = r41->size[0];
  r41->size[0] = i;
  emxEnsureCapacity((emxArray__common *)r41, i33, (int)sizeof(int));
  for (i33 = 0; i33 < i; i33++) {
    r41->data[i33] = i33;
  }

  for (i33 = 0; i33 < 12; i33++) {
    for (i34 = 0; i34 < 12; i34++) {
      c_P_xx_apr[i34 + 12 * i33] = (b_P_xx_apr[i34 + 12 * i33] + b_P_xx_apr[i33
        + 12 * i34]) / 2.0;
    }
  }

  i = r40->size[0];
  cr = r41->size[0];
  for (i33 = 0; i33 < cr; i33++) {
    for (i34 = 0; i34 < i; i34++) {
      b_P->data[r40->data[i34] + b_P->size[0] * r41->data[i33]] = c_P_xx_apr[i34
        + i * i33];
    }
  }

  if (1.0 > b_SLAM_data->numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)b_SLAM_data->numStates;
  }

  if (b_SLAM_data->numStates + 1.0 > b_P->size[1]) {
    i33 = 0;
    i34 = 0;
  } else {
    i33 = (int)(b_SLAM_data->numStates + 1.0) - 1;
    i34 = b_P->size[1];
  }

  cr = r40->size[0];
  r40->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r40, cr, (int)sizeof(int));
  for (cr = 0; cr < loop_ub; cr++) {
    r40->data[cr] = cr;
  }

  cr = r41->size[0];
  r41->size[0] = i34 - i33;
  emxEnsureCapacity((emxArray__common *)r41, cr, (int)sizeof(int));
  loop_ub = i34 - i33;
  for (i34 = 0; i34 < loop_ub; i34++) {
    r41->data[i34] = i33 + i34;
  }

  i = r40->size[0];
  cr = r41->size[0];
  for (i33 = 0; i33 < cr; i33++) {
    for (i34 = 0; i34 < i; i34++) {
      b_P->data[r40->data[i34] + b_P->size[0] * r41->data[i33]] = P_xs_apr->
        data[i34 + i * i33];
    }
  }

  if (b_SLAM_data->numStates + 1.0 > b_P->size[0]) {
    i33 = 0;
    i34 = 0;
  } else {
    i33 = (int)(b_SLAM_data->numStates + 1.0) - 1;
    i34 = b_P->size[0];
  }

  if (1.0 > b_SLAM_data->numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)b_SLAM_data->numStates;
  }

  cr = r40->size[0];
  r40->size[0] = i34 - i33;
  emxEnsureCapacity((emxArray__common *)r40, cr, (int)sizeof(int));
  i = i34 - i33;
  for (i34 = 0; i34 < i; i34++) {
    r40->data[i34] = i33 + i34;
  }

  i33 = r41->size[0];
  r41->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r41, i33, (int)sizeof(int));
  for (i33 = 0; i33 < loop_ub; i33++) {
    r41->data[i33] = i33;
  }

  b_emxInit_real_T(&b_P_xs_apr, 2);
  i33 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i33, (int)sizeof(double));
  for (i33 = 0; i33 < 12; i33++) {
    loop_ub = P_xs_apr->size[1];
    for (i34 = 0; i34 < loop_ub; i34++) {
      b_P_xs_apr->data[i34 + b_P_xs_apr->size[0] * i33] = P_xs_apr->data[i33 +
        P_xs_apr->size[0] * i34];
    }
  }

  emxFree_real_T(&P_xs_apr);
  i = r40->size[0];
  cr = r41->size[0];
  for (i33 = 0; i33 < cr; i33++) {
    for (i34 = 0; i34 < i; i34++) {
      b_P->data[r40->data[i34] + b_P->size[0] * r41->data[i33]] =
        b_P_xs_apr->data[i34 + i * i33];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r41);
  emxFree_int32_T(&r40);
  i33 = b_SLAM_data->xt->size[0];
  b_SLAM_data->xt->size[0] = b_xt->size[0];
  emxEnsureCapacity((emxArray__common *)b_SLAM_data->xt, i33, (int)sizeof(double));
  loop_ub = b_xt->size[0];
  for (i33 = 0; i33 < loop_ub; i33++) {
    b_SLAM_data->xt->data[i33] = b_xt->data[i33];
  }

  emxFree_real_T(&b_xt);
  i33 = b_SLAM_data->P->size[0] * b_SLAM_data->P->size[1];
  b_SLAM_data->P->size[0] = b_P->size[0];
  b_SLAM_data->P->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b_SLAM_data->P, i33, (int)sizeof(double));
  loop_ub = b_P->size[0] * b_P->size[1];
  for (i33 = 0; i33 < loop_ub; i33++) {
    b_SLAM_data->P->data[i33] = b_P->data[i33];
  }

  emxFree_real_T(&b_P);
}

//
// File trailer for SLAM_pred_model.cpp
//
// [EOF]
//
