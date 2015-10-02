//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred_euler.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_pred_euler.h"
#include "quatmultJ.h"
#include "quatPlusThetaJ.h"
#include "SLAM_emxutil.h"
#include "diag.h"
#include "eye.h"
#include <ros/console.h>

// Function Declarations
static int div_nzp_s32_floor(int numerator, int denominator);

// Function Definitions

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_nzp_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (numerator >= 0) {
    absNumerator = (unsigned int)numerator;
  } else {
    absNumerator = (unsigned int)-numerator;
  }

  if (denominator >= 0) {
    absDenominator = (unsigned int)denominator;
  } else {
    absDenominator = (unsigned int)-denominator;
  }

  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
  }

  if (quotientNeedsNegation) {
    quotient = -(int)tempAbsQuotient;
  } else {
    quotient = (int)tempAbsQuotient;
  }

  return quotient;
}

//
// Arguments    : emxArray_real_T *P_apo
//                f_struct_T *x
//                double dt
//                double processNoise_qv
//                double processNoise_qw
//                double processNoise_qao
//                double processNoise_qwo
//                double processNoise_qR_ci
//                const double measurements_acc_duo[3]
//                const double measurements_gyr_duo[3]
//                double c_numStates
// Return Type  : void
//
void SLAM_pred_euler(emxArray_real_T *P_apo, f_struct_T *x, double dt, double
                     processNoise_qv, double processNoise_qw, double
                     processNoise_qao, double processNoise_qwo, double
                     processNoise_qR_ci, const double measurements_acc_duo[3],
                     const double measurements_gyr_duo[3], double c_numStates)
{
  double R_cw[9];
  double R_ci[9];
  double b_R_ci[9];
  int i31;
  int cr;
  double t_ci[3];
  double w_imu[3];
  double R[9];
  double b_R[9];
  double w_c[3];
  int k;
  double b_measurements_acc_duo[3];
  double d6;
  double b_x[9];
  double a_c[3];
  double grav_origin[3];
  static const double b[3] = { 0.0, 0.0, 9.81 };

  double y[3];
  double dv3[9];
  double b_y[3];
  double c_y[3];
  double G[315];
  static const signed char iv3[45] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0 };

  static const signed char iv4[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv5[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv6[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1 };

  emxArray_real_T *r2;
  double dv4[9];
  double dv5[9];
  double dv6[9];
  double dv7[9];
  double dv8[9];
  double dv9[9];
  double dv10[9];
  double dv11[9];
  double b_G[441];
  static const signed char iv7[63] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv8[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double Phi[441];
  int loop_ub;
  int b_loop_ub;
  emxArray_real_T *P_xs_apr;
  emxArray_real_T *b_P_apo;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  double P_xx_apr[441];
  double d_y[441];
  double b_processNoise_qw[15];
  double dv12[225];
  double c_G[315];
  emxArray_real_T *c_P_apo;
  emxArray_int32_T *r3;
  emxArray_int32_T *r4;
  emxArray_real_T *b_P_xs_apr;
  double c_x[4];
  double dv13[4];

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((x->robot_state.att[0] * x->robot_state.att[0] - x->
              robot_state.att[1] * x->robot_state.att[1]) - x->robot_state.att[2]
             * x->robot_state.att[2]) + x->robot_state.att[3] *
    x->robot_state.att[3];
  R_cw[3] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[1] +
                   x->robot_state.att[2] * x->robot_state.att[3]);
  R_cw[6] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[2] -
                   x->robot_state.att[1] * x->robot_state.att[3]);
  R_cw[1] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[1] -
                   x->robot_state.att[2] * x->robot_state.att[3]);
  R_cw[4] = ((-(x->robot_state.att[0] * x->robot_state.att[0]) +
              x->robot_state.att[1] * x->robot_state.att[1]) -
             x->robot_state.att[2] * x->robot_state.att[2]) + x->
    robot_state.att[3] * x->robot_state.att[3];
  R_cw[7] = 2.0 * (x->robot_state.att[1] * x->robot_state.att[2] +
                   x->robot_state.att[0] * x->robot_state.att[3]);
  R_cw[2] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[2] +
                   x->robot_state.att[1] * x->robot_state.att[3]);
  R_cw[5] = 2.0 * (x->robot_state.att[1] * x->robot_state.att[2] -
                   x->robot_state.att[0] * x->robot_state.att[3]);
  R_cw[8] = ((-(x->robot_state.att[0] * x->robot_state.att[0]) -
              x->robot_state.att[1] * x->robot_state.att[1]) +
             x->robot_state.att[2] * x->robot_state.att[2]) + x->
    robot_state.att[3] * x->robot_state.att[3];

  //  rotation in origin frame
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_ci[0] = ((x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0] -
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) -
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];
  R_ci[3] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[1] +
                   x->robot_state.IMU.att[2] * x->robot_state.IMU.att[3]);
  R_ci[6] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[2] -
                   x->robot_state.IMU.att[1] * x->robot_state.IMU.att[3]);
  R_ci[1] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[1] -
                   x->robot_state.IMU.att[2] * x->robot_state.IMU.att[3]);
  R_ci[4] = ((-(x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0]) +
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) -
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];
  R_ci[7] = 2.0 * (x->robot_state.IMU.att[1] * x->robot_state.IMU.att[2] +
                   x->robot_state.IMU.att[0] * x->robot_state.IMU.att[3]);
  R_ci[2] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[2] +
                   x->robot_state.IMU.att[1] * x->robot_state.IMU.att[3]);
  R_ci[5] = 2.0 * (x->robot_state.IMU.att[1] * x->robot_state.IMU.att[2] -
                   x->robot_state.IMU.att[0] * x->robot_state.IMU.att[3]);
  R_ci[8] = ((-(x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0]) -
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) +
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];

  //  in camera frame
  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_R_ci[cr + 3 * i31] = -R_ci[i31 + 3 * cr];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    t_ci[i31] = 0.0;
    for (cr = 0; cr < 3; cr++) {
      t_ci[i31] += b_R_ci[i31 + 3 * cr] * x->robot_state.IMU.pos[cr];
    }

    //  in imu frame
    w_imu[i31] = measurements_gyr_duo[i31] - x->robot_state.IMU.gyro_bias[i31];
  }

  //  gyro in IMU frame
  //  gyro in camera frame
  //  w = 0*w;
  //  acceleration in IMU frame
  R[0] = 0.0;
  R[3] = -w_imu[2];
  R[6] = w_imu[1];
  R[1] = w_imu[2];
  R[4] = 0.0;
  R[7] = -w_imu[0];
  R[2] = -w_imu[1];
  R[5] = w_imu[0];
  R[8] = 0.0;
  for (i31 = 0; i31 < 3; i31++) {
    w_c[i31] = 0.0;
    for (cr = 0; cr < 3; cr++) {
      w_c[i31] += R_ci[i31 + 3 * cr] * w_imu[cr];
    }

    for (cr = 0; cr < 3; cr++) {
      b_R[i31 + 3 * cr] = 0.0;
      for (k = 0; k < 3; k++) {
        b_R[i31 + 3 * cr] += R[i31 + 3 * k] * R[k + 3 * cr];
      }
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    d6 = 0.0;
    for (cr = 0; cr < 3; cr++) {
      d6 += b_R[i31 + 3 * cr] * t_ci[cr];
    }

    b_measurements_acc_duo[i31] = (measurements_acc_duo[i31] -
      x->robot_state.IMU.acc_bias[i31]) + d6;
  }

  //  a = 0*a;
  // % compute the linearization F of the non linear model f
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  b_x[0] = ((x->origin.att[0] * x->origin.att[0] - x->origin.att[1] *
             x->origin.att[1]) - x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];
  b_x[3] = 2.0 * (x->origin.att[0] * x->origin.att[1] + x->origin.att[2] *
                  x->origin.att[3]);
  b_x[6] = 2.0 * (x->origin.att[0] * x->origin.att[2] - x->origin.att[1] *
                  x->origin.att[3]);
  b_x[1] = 2.0 * (x->origin.att[0] * x->origin.att[1] - x->origin.att[2] *
                  x->origin.att[3]);
  b_x[4] = ((-(x->origin.att[0] * x->origin.att[0]) + x->origin.att[1] *
             x->origin.att[1]) - x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];
  b_x[7] = 2.0 * (x->origin.att[1] * x->origin.att[2] + x->origin.att[0] *
                  x->origin.att[3]);
  b_x[2] = 2.0 * (x->origin.att[0] * x->origin.att[2] + x->origin.att[1] *
                  x->origin.att[3]);
  b_x[5] = 2.0 * (x->origin.att[1] * x->origin.att[2] - x->origin.att[0] *
                  x->origin.att[3]);
  b_x[8] = ((-(x->origin.att[0] * x->origin.att[0]) - x->origin.att[1] *
             x->origin.att[1]) + x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];
  for (i31 = 0; i31 < 3; i31++) {
    a_c[i31] = 0.0;
    for (cr = 0; cr < 3; cr++) {
      a_c[i31] += R_ci[i31 + 3 * cr] * b_measurements_acc_duo[cr];
    }

    grav_origin[i31] = 0.0;
    for (cr = 0; cr < 3; cr++) {
      grav_origin[i31] += b_x[i31 + 3 * cr] * b[cr];
    }

    //  gravity transformed into the origin frame
    //    pos,            rot, vel,                                         gyr_bias,   acc_bias,          origin_att,                                    R_ci 
    y[i31] = 0.0;
    for (cr = 0; cr < 3; cr++) {
      y[i31] += R_ci[i31 + 3 * cr] * x->robot_state.IMU.gyro_bias[cr];
    }
  }

  dv3[0] = 0.0;
  dv3[3] = -w_imu[2];
  dv3[6] = w_imu[1];
  dv3[1] = w_imu[2];
  dv3[4] = 0.0;
  dv3[7] = -w_imu[0];
  dv3[2] = -w_imu[1];
  dv3[5] = w_imu[0];
  dv3[8] = 0.0;
  for (i31 = 0; i31 < 3; i31++) {
    b_y[i31] = 0.0;
    for (cr = 0; cr < 3; cr++) {
      b_y[i31] += R_cw[cr + 3 * i31] * a_c[cr];
    }

    c_y[i31] = 0.0;
    for (cr = 0; cr < 3; cr++) {
      c_y[i31] += dv3[i31 + 3 * cr] * t_ci[cr];
    }
  }

  //  pos
  //  att
  //  vel
  //  gyro bias
  //  acc bias
  //  origin att
  for (i31 = 0; i31 < 15; i31++) {
    for (cr = 0; cr < 3; cr++) {
      G[cr + 21 * i31] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * i31) + 3] = iv3[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * i31) + 6] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * (i31 + 3)) + 6] = -R_cw[i31 + 3 * cr];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * (i31 + 6)) + 6] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * (i31 + 9)) + 6] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * (i31 + 12)) + 6] = 0.0;
    }
  }

  for (i31 = 0; i31 < 15; i31++) {
    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * i31) + 9] = iv4[cr + 3 * i31];
    }

    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * i31) + 12] = iv5[cr + 3 * i31];
    }

    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * i31) + 15] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      G[(cr + 21 * i31) + 18] = iv6[cr + 3 * i31];
    }
  }

  emxInit_real_T(&r2, 2);

  //  R_ci
  b_eye(c_numStates, r2);
  dv4[0] = 0.0;
  dv4[3] = -w_c[2];
  dv4[6] = w_c[1];
  dv4[1] = w_c[2];
  dv4[4] = 0.0;
  dv4[7] = -w_c[0];
  dv4[2] = -w_c[1];
  dv4[5] = w_c[0];
  dv4[8] = 0.0;
  dv5[0] = 0.0;
  dv5[3] = -y[2];
  dv5[6] = y[1];
  dv5[1] = y[2];
  dv5[4] = 0.0;
  dv5[7] = -y[0];
  dv5[2] = -y[1];
  dv5[5] = y[0];
  dv5[8] = 0.0;
  dv6[0] = 0.0;
  dv6[3] = -b_y[2];
  dv6[6] = b_y[1];
  dv6[1] = b_y[2];
  dv6[4] = 0.0;
  dv6[7] = -b_y[0];
  dv6[2] = -b_y[1];
  dv6[5] = b_y[0];
  dv6[8] = 0.0;
  dv7[0] = 0.0;
  dv7[3] = -c_y[2];
  dv7[6] = c_y[1];
  dv7[1] = c_y[2];
  dv7[4] = 0.0;
  dv7[7] = -c_y[0];
  dv7[2] = -c_y[1];
  dv7[5] = c_y[0];
  dv7[8] = 0.0;
  dv8[0] = 0.0;
  dv8[3] = -w_imu[2];
  dv8[6] = w_imu[1];
  dv8[1] = w_imu[2];
  dv8[4] = 0.0;
  dv8[7] = -w_imu[0];
  dv8[2] = -w_imu[1];
  dv8[5] = w_imu[0];
  dv8[8] = 0.0;
  dv9[0] = 0.0;
  dv9[3] = -t_ci[2];
  dv9[6] = t_ci[1];
  dv9[1] = t_ci[2];
  dv9[4] = 0.0;
  dv9[7] = -t_ci[0];
  dv9[2] = -t_ci[1];
  dv9[5] = t_ci[0];
  dv9[8] = 0.0;
  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      R[cr + 3 * i31] = -R_cw[i31 + 3 * cr];
    }
  }

  dv10[0] = 0.0;
  dv10[3] = -grav_origin[2];
  dv10[6] = grav_origin[1];
  dv10[1] = grav_origin[2];
  dv10[4] = 0.0;
  dv10[7] = -grav_origin[0];
  dv10[2] = -grav_origin[1];
  dv10[5] = grav_origin[0];
  dv10[8] = 0.0;
  dv11[0] = 0.0;
  dv11[3] = -a_c[2];
  dv11[6] = a_c[1];
  dv11[1] = a_c[2];
  dv11[4] = 0.0;
  dv11[7] = -a_c[0];
  dv11[2] = -a_c[1];
  dv11[5] = a_c[0];
  dv11[8] = 0.0;
  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      d6 = 0.0;
      for (k = 0; k < 3; k++) {
        d6 += dv8[i31 + 3 * k] * dv9[k + 3 * cr];
      }

      dv3[i31 + 3 * cr] = -dv7[i31 + 3 * cr] - d6;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_R_ci[i31 + 3 * cr] = 0.0;
      for (k = 0; k < 3; k++) {
        b_R_ci[i31 + 3 * cr] += R[i31 + 3 * k] * R_ci[k + 3 * cr];
      }

      b_R[i31 + 3 * cr] = 0.0;
      for (k = 0; k < 3; k++) {
        b_R[i31 + 3 * cr] += R_cw[i31 + 3 * k] * dv11[k + 3 * cr];
      }
    }
  }

  for (i31 = 0; i31 < 21; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[cr + 21 * i31] = iv7[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * i31) + 3] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 3)) + 3] = -dv4[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 6)) + 3] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 9)) + 3] = iv8[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 12)) + 3] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 15)) + 3] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 18)) + 3] = -dv5[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * i31) + 6] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 3)) + 6] = -dv6[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 6)) + 6] = 0.0;
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 9)) + 6] = dv3[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 12)) + 6] = b_R_ci[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 15)) + 6] = -dv10[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 3; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * (i31 + 18)) + 6] = b_R[cr + 3 * i31];
    }
  }

  for (i31 = 0; i31 < 21; i31++) {
    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * i31) + 9] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * i31) + 12] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * i31) + 15] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      b_G[(cr + 21 * i31) + 18] = 0.0;
    }
  }

  for (i31 = 0; i31 < 21; i31++) {
    for (cr = 0; cr < 21; cr++) {
      Phi[cr + 21 * i31] = r2->data[cr + 21 * i31] + b_G[cr + 21 * i31] * dt;
    }
  }

  emxFree_real_T(&r2);
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
    i31 = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = 1;
    b_P_apo->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)b_P_apo, i31, (int)sizeof(double));
    for (i31 = 0; i31 < b_loop_ub; i31++) {
      cr = 0;
      while (cr <= 0) {
        b_P_apo->data[b_P_apo->size[0] * i31] = P_apo->data[P_apo->size[0] * i31];
        cr = 1;
      }
    }

    i31 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 21;
    P_xs_apr->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i31, (int)sizeof(double));
    for (i31 = 0; i31 < 21; i31++) {
      loop_ub = b_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i31 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 21; k++) {
          P_xs_apr->data[i31 + P_xs_apr->size[0] * cr] += Phi[i31 + 21 * k] *
            b_P_apo->data[k + b_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&b_P_apo);
  } else {
    i31 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 21;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i31, (int)sizeof(double));
    i31 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i31, (int)sizeof(double));
    k = 21 * b_loop_ub;
    for (i31 = 0; i31 < k; i31++) {
      P_xs_apr->data[i31] = 0.0;
    }

    if (b_loop_ub == 0) {
    } else {
      b_loop_ub = 21 * (b_loop_ub - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 21) {
        for (ic = cr + 1; ic <= cr + 21; ic++) {
          P_xs_apr->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 21) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 21; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * div_nzp_s32_floor(ib,
               loop_ub)] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 21; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                div_nzp_s32_floor(ib, loop_ub)] * Phi[ia];
            }
          }

          ar += 21;
        }

        br += 21;
      }
    }
  }

  for (i31 = 0; i31 < 21; i31++) {
    for (cr = 0; cr < 21; cr++) {
      P_xx_apr[cr + 21 * i31] = Phi[i31 + 21 * cr];
    }
  }

  if (P_xs_apr->size[1] == 1) {
    for (i31 = 0; i31 < 21; i31++) {
      for (cr = 0; cr < 21; cr++) {
        d_y[i31 + 21 * cr] = 0.0;
        for (k = 0; k < 21; k++) {
          d_y[i31 + 21 * cr] += P_xs_apr->data[i31 + 21 * k] * P_xx_apr[k + 21 *
            cr];
        }
      }
    }
  } else {
    k = P_xs_apr->size[1];
    memset(&d_y[0], 0, 441U * sizeof(double));
    for (cr = 0; cr < 422; cr += 21) {
      for (ic = cr; ic + 1 <= cr + 21; ic++) {
        d_y[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr < 422; cr += 21) {
      ar = -1;
      i31 = br + k;
      for (ib = br; ib + 1 <= i31; ib++) {
        if (P_xx_apr[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 21; ic++) {
            ia++;
            d_y[ic] += P_xx_apr[ib] * P_xs_apr->data[ia];
          }
        }

        ar += 21;
      }

      br += k;
    }
  }

  b_processNoise_qw[0] = processNoise_qw;
  b_processNoise_qw[1] = processNoise_qw;
  b_processNoise_qw[2] = processNoise_qw;
  b_processNoise_qw[3] = processNoise_qv;
  b_processNoise_qw[4] = processNoise_qv;
  b_processNoise_qw[5] = processNoise_qv;
  b_processNoise_qw[6] = processNoise_qwo;
  b_processNoise_qw[7] = processNoise_qwo;
  b_processNoise_qw[8] = processNoise_qwo;
  b_processNoise_qw[9] = 0.0 * processNoise_qao;
  b_processNoise_qw[10] = processNoise_qao;
  b_processNoise_qw[11] = 0.0 * processNoise_qao;
  b_processNoise_qw[12] = processNoise_qR_ci;
  b_processNoise_qw[13] = processNoise_qR_ci;
  b_processNoise_qw[14] = processNoise_qR_ci;
  diag(b_processNoise_qw, dv12);
  for (i31 = 0; i31 < 21; i31++) {
    for (cr = 0; cr < 15; cr++) {
      c_G[i31 + 21 * cr] = 0.0;
      for (k = 0; k < 15; k++) {
        c_G[i31 + 21 * cr] += G[i31 + 21 * k] * dv12[k + 15 * cr];
      }
    }

    for (cr = 0; cr < 21; cr++) {
      b_G[i31 + 21 * cr] = 0.0;
      for (k = 0; k < 15; k++) {
        b_G[i31 + 21 * cr] += c_G[i31 + 21 * k] * G[cr + 21 * k];
      }
    }
  }

  for (i31 = 0; i31 < 21; i31++) {
    for (cr = 0; cr < 21; cr++) {
      P_xx_apr[cr + 21 * i31] = d_y[cr + 21 * i31] + b_G[cr + 21 * i31] * dt;
    }
  }

  //  covariance of the state
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i31 = 1;
    cr = 1;
  } else {
    i31 = (int)(c_numStates + 1.0);
    cr = P_apo->size[1] + 1;
  }

  if (loop_ub == 1) {
    emxInit_real_T(&c_P_apo, 2);
    k = c_P_apo->size[0] * c_P_apo->size[1];
    c_P_apo->size[0] = 1;
    c_P_apo->size[1] = cr - i31;
    emxEnsureCapacity((emxArray__common *)c_P_apo, k, (int)sizeof(double));
    loop_ub = cr - i31;
    for (cr = 0; cr < loop_ub; cr++) {
      k = 0;
      while (k <= 0) {
        c_P_apo->data[c_P_apo->size[0] * cr] = P_apo->data[P_apo->size[0] *
          ((i31 + cr) - 1)];
        k = 1;
      }
    }

    i31 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 21;
    P_xs_apr->size[1] = c_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i31, (int)sizeof(double));
    for (i31 = 0; i31 < 21; i31++) {
      loop_ub = c_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i31 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 21; k++) {
          P_xs_apr->data[i31 + P_xs_apr->size[0] * cr] += Phi[i31 + 21 * k] *
            c_P_apo->data[k + c_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&c_P_apo);
  } else {
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 21;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = cr - i31;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    b_loop_ub = 21 * (cr - i31);
    for (k = 0; k < b_loop_ub; k++) {
      P_xs_apr->data[k] = 0.0;
    }

    if (cr - i31 == 0) {
    } else {
      b_loop_ub = 21 * ((cr - i31) - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 21) {
        for (ic = cr + 1; ic <= cr + 21; ic++) {
          P_xs_apr->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 21) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 21; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * ((i31 +
                div_nzp_s32_floor(ib, loop_ub)) - 1)] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 21; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                ((i31 + div_nzp_s32_floor(ib, loop_ub)) - 1)] * Phi[ia];
            }
          }

          ar += 21;
        }

        br += 21;
      }
    }
  }

  //  covariance between current state and trails
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

  b_emxInit_int32_T(&r3, 1);
  i31 = r3->size[0];
  r3->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r3, i31, (int)sizeof(int));
  for (i31 = 0; i31 < loop_ub; i31++) {
    r3->data[i31] = i31;
  }

  b_emxInit_int32_T(&r4, 1);
  i31 = r4->size[0];
  r4->size[0] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r4, i31, (int)sizeof(int));
  for (i31 = 0; i31 < b_loop_ub; i31++) {
    r4->data[i31] = i31;
  }

  for (i31 = 0; i31 < 21; i31++) {
    for (cr = 0; cr < 21; cr++) {
      b_G[cr + 21 * i31] = (P_xx_apr[cr + 21 * i31] + P_xx_apr[i31 + 21 * cr]) /
        2.0;
    }
  }

  k = r3->size[0];
  b_loop_ub = r4->size[0];
  for (i31 = 0; i31 < b_loop_ub; i31++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r3->data[cr] + P_apo->size[0] * r4->data[i31]] = b_G[cr + k *
        i31];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i31 = 0;
    cr = 0;
  } else {
    i31 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[1];
  }

  k = r3->size[0];
  r3->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r3, k, (int)sizeof(int));
  for (k = 0; k < loop_ub; k++) {
    r3->data[k] = k;
  }

  k = r4->size[0];
  r4->size[0] = cr - i31;
  emxEnsureCapacity((emxArray__common *)r4, k, (int)sizeof(int));
  loop_ub = cr - i31;
  for (cr = 0; cr < loop_ub; cr++) {
    r4->data[cr] = i31 + cr;
  }

  k = r3->size[0];
  b_loop_ub = r4->size[0];
  for (i31 = 0; i31 < b_loop_ub; i31++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r3->data[cr] + P_apo->size[0] * r4->data[i31]] =
        P_xs_apr->data[cr + k * i31];
    }
  }

  if (c_numStates + 1.0 > P_apo->size[0]) {
    i31 = 0;
    cr = 0;
  } else {
    i31 = (int)(c_numStates + 1.0) - 1;
    cr = P_apo->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  k = r3->size[0];
  r3->size[0] = cr - i31;
  emxEnsureCapacity((emxArray__common *)r3, k, (int)sizeof(int));
  b_loop_ub = cr - i31;
  for (cr = 0; cr < b_loop_ub; cr++) {
    r3->data[cr] = i31 + cr;
  }

  i31 = r4->size[0];
  r4->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r4, i31, (int)sizeof(int));
  for (i31 = 0; i31 < loop_ub; i31++) {
    r4->data[i31] = i31;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i31 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 21;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i31, (int)sizeof(double));
  for (i31 = 0; i31 < 21; i31++) {
    loop_ub = P_xs_apr->size[1];
    for (cr = 0; cr < loop_ub; cr++) {
      b_P_xs_apr->data[cr + b_P_xs_apr->size[0] * i31] = P_xs_apr->data[i31 +
        P_xs_apr->size[0] * cr];
    }
  }

  emxFree_real_T(&P_xs_apr);
  k = r3->size[0];
  b_loop_ub = r4->size[0];
  for (i31 = 0; i31 < b_loop_ub; i31++) {
    for (cr = 0; cr < k; cr++) {
      P_apo->data[r3->data[cr] + P_apo->size[0] * r4->data[i31]] =
        b_P_xs_apr->data[cr + k * i31];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r4);
  emxFree_int32_T(&r3);
  for (i31 = 0; i31 < 3; i31++) {
    x->robot_state.pos[i31] += x->robot_state.vel[i31] * dt;
    b_measurements_acc_duo[i31] = w_c[i31] * dt;
  }

  for (k = 0; k < 4; k++) {
    c_x[k] = x->robot_state.att[k];
  }

  quatPlusThetaJ(b_measurements_acc_duo, dv13);
  quatmultJ(dv13, c_x, x->robot_state.att);
  for (i31 = 0; i31 < 3; i31++) {
    d6 = 0.0;
    for (cr = 0; cr < 3; cr++) {
      d6 += R_cw[cr + 3 * i31] * a_c[cr];
    }

    b_measurements_acc_duo[i31] = d6 - grav_origin[i31];
  }

  for (i31 = 0; i31 < 3; i31++) {
    x->robot_state.vel[i31] += b_measurements_acc_duo[i31] * dt;
  }

  //  velocity
  //  P_apr = (P_apr+P_apr')/2;
}

//
// File trailer for SLAM_pred_euler.cpp
//
// [EOF]
//
