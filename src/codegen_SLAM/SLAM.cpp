//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 18-Aug-2015 09:45:37
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "Att_upd.h"
#include "quatPlusThetaJ.h"
#include "fprintf.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_pred.h"
#include "blkdiag.h"
#include "QuatFromRotJ.h"
#include "norm.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static double delayBuffer_k[84];
static double init_counter;
static double P_att[9];
static double x_att[4];

// Function Definitions

//
// Arguments    : double updateVect[32]
//                const double z_all_l[64]
//                const double z_all_r[64]
//                double dt
//                const double processNoise[4]
//                double IMU_measurements[23]
//                const double imNoise[2]
//                double numPointsPerAnchor
//                double numAnchors
//                const stereoParameters *cameraParams
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
// Return Type  : void
//
void SLAM(double updateVect[32], const double z_all_l[64], const double z_all_r
          [64], double dt, const double processNoise[4], double
          IMU_measurements[23], const double imNoise[2], double
          numPointsPerAnchor, double numAnchors, const stereoParameters
          *cameraParams, emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out,
          emxArray_real_T *P_apo_out, emxArray_real_T *map_out)
{
  double B;
  double z_n_b[3];
  int outsize_idx_0;
  double y_n_b[3];
  int ibcol;
  double x_n_b[3];
  double b_x_n_b[9];
  static const double dv36[9] = { 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
    0.001 };

  emxArray_real_T *a;
  emxArray_real_T *b;
  int itilerow;
  int k;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const double b_y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01
  };

  double b_z_n_b[3];
  static const double b_a[3] = { -0.0132, -0.4398, 0.2673 };

  double c_a[3];
  double d_a[3];
  static const double e_a[9] = { -0.0077435500000000435, 0.0086606800000000872,
    0.99989175999999991, -0.99987860000000006, -0.010132990000000119,
    -0.0076556800000000536, 0.010066000000000075, -0.9998704,
    0.0087384500000000087 };

  signed char I[9];
  double f_a[9];
  double Phi[9];
  double dq[4];
  double dv37[16];
  double b_dq[4];
  double b_Phi[9];
  double g_a[9];
  static const signed char b_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char h_a[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  int tmp_data[32];

  //  persistents for attitude estimator
  //  for coder
  // % imu hack
  // % finish imu hack
  if (!initialized_not_empty) {
    //  initialization for attitude filter
    // delayBuffer_k_1=[0;0;0;0;0;0];
    B = norm(*(double (*)[3])&IMU_measurements[3]);
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      z_n_b[outsize_idx_0] = IMU_measurements[outsize_idx_0 + 3] / B;
    }

    // m_n_b=IMU_measurements(11:13);
    y_n_b[0] = z_n_b[1] * 0.0 - z_n_b[2] * 0.0;
    y_n_b[1] = z_n_b[2] - z_n_b[0] * 0.0;
    y_n_b[2] = z_n_b[0] * 0.0 - z_n_b[1];
    B = norm(y_n_b);
    for (ibcol = 0; ibcol < 3; ibcol++) {
      y_n_b[ibcol] /= B;
    }

    x_n_b[0] = y_n_b[1] * z_n_b[2] - y_n_b[2] * z_n_b[1];
    x_n_b[1] = y_n_b[2] * z_n_b[0] - y_n_b[0] * z_n_b[2];
    x_n_b[2] = y_n_b[0] * z_n_b[1] - y_n_b[1] * z_n_b[0];
    B = norm(x_n_b);
    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_x_n_b[ibcol] = x_n_b[ibcol] / B;
      b_x_n_b[3 + ibcol] = y_n_b[ibcol];
      b_x_n_b[6 + ibcol] = z_n_b[ibcol];
    }

    QuatFromRotJ(b_x_n_b, x_att);
    memcpy(&P_att[0], &dv36[0], 9U * sizeof(double));
    b_emxInit_real_T(&a, 1);

    //  other initialization
    initialized_not_empty = true;
    ibcol = a->size[0];
    a->size[0] = 7 + (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)a, ibcol, (int)sizeof(double));
    a->data[0] = 0.0;
    a->data[1] = 0.0;
    a->data[2] = 0.0;
    a->data[3] = 0.0;
    a->data[4] = 0.0;
    a->data[5] = 0.0;
    a->data[6] = 1.0;
    outsize_idx_0 = (int)numPointsPerAnchor;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      a->data[ibcol + 7] = 0.0;
    }

    b_emxInit_real_T(&b, 1);
    outsize_idx_0 = a->size[0] * (int)numAnchors;
    ibcol = b->size[0];
    b->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)b, ibcol, (int)sizeof(double));
    if (!(outsize_idx_0 == 0)) {
      outsize_idx_0 = a->size[0];
      for (itilerow = 1; itilerow <= (int)numAnchors; itilerow++) {
        ibcol = (itilerow - 1) * outsize_idx_0;
        for (k = 0; k + 1 <= outsize_idx_0; k++) {
          b->data[ibcol + k] = a->data[k];
        }
      }
    }

    emxFree_real_T(&a);
    ibcol = xt->size[0];
    xt->size[0] = 13 + b->size[0];
    emxEnsureCapacity((emxArray__common *)xt, ibcol, (int)sizeof(double));
    xt->data[0] = 0.0;
    xt->data[1] = 0.0;
    xt->data[2] = 0.0;
    xt->data[3] = 0.0;
    xt->data[4] = 0.0;
    xt->data[5] = 0.0;
    xt->data[6] = 1.0;
    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    for (ibcol = 0; ibcol < 3; ibcol++) {
      xt->data[ibcol + 10] = 0.0 * IMU_measurements[ibcol];
    }

    outsize_idx_0 = b->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt->data[ibcol + 13] = b->data[ibcol];
    }

    emxFree_real_T(&b);
    emxInit_real_T(&r5, 2);

    //  initial real vector
    B = numAnchors * (6.0 + numPointsPerAnchor);
    ibcol = r5->size[0] * r5->size[1];
    r5->size[0] = (int)numStates;
    r5->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r5, ibcol, (int)sizeof(double));
    outsize_idx_0 = (int)numStates * (int)numStates;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      r5->data[ibcol] = 0.0;
    }

    emxInit_real_T(&r6, 2);
    ibcol = r6->size[0] * r6->size[1];
    r6->size[0] = (int)B;
    r6->size[1] = (int)B;
    emxEnsureCapacity((emxArray__common *)r6, ibcol, (int)sizeof(double));
    outsize_idx_0 = (int)B * (int)B;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      r6->data[ibcol] = 0.0;
    }

    blkdiag(r5, r6, P);

    //  initial error state covariance
    emxFree_real_T(&r6);
    emxFree_real_T(&r5);
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[outsize_idx_0 + P->size[0] * ibcol] = 0.0;
      }
    }

    //  position
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (3 + ibcol)) + 3] =
          y[outsize_idx_0 + 3 * ibcol];
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (6 + ibcol)) + 6] = 0.0;
      }
    }

    //  velocity
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (9 + ibcol)) + 9] =
          b_y[outsize_idx_0 + 3 * ibcol];
      }
    }

    //  gyro bias
  }

  for (k = 0; k < 13; k++) {
    delayBuffer_k[1 + k] = delayBuffer_k[k];
  }

  for (ibcol = 0; ibcol < 3; ibcol++) {
    delayBuffer_k[ibcol] = IMU_measurements[13 + ibcol];
  }

  for (ibcol = 0; ibcol < 3; ibcol++) {
    delayBuffer_k[ibcol + 3] = IMU_measurements[16 + ibcol];
  }

  for (ibcol = 0; ibcol < 3; ibcol++) {
    b_z_n_b[ibcol] = delayBuffer_k[78 + ibcol] + b_a[ibcol];
  }

  for (ibcol = 0; ibcol < 3; ibcol++) {
    c_a[ibcol] = 0.0;
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      c_a[ibcol] += e_a[ibcol + 3 * outsize_idx_0] * b_z_n_b[outsize_idx_0];
    }

    IMU_measurements[ibcol] = c_a[ibcol];
    d_a[ibcol] = 0.0;
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      d_a[ibcol] += e_a[ibcol + 3 * outsize_idx_0] * delayBuffer_k[outsize_idx_0
        + 81];
    }
  }

  for (ibcol = 0; ibcol < 3; ibcol++) {
    IMU_measurements[3 + ibcol] = d_a[ibcol];
  }

  if (init_counter < 10.0) {
    //      zw = IMU_measurements(1:3);
    //      za = IMU_measurements(4:6);
    for (ibcol = 0; ibcol < 3; ibcol++) {
      z_n_b[ibcol] = 0.0;
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        z_n_b[ibcol] += e_a[ibcol + 3 * outsize_idx_0] * IMU_measurements[13 +
          outsize_idx_0];
      }
    }

    // ATT_PRED Prediction step of the attitude estimator
    //    INPUT ARGUMENTS:
    //    - x:  The current estimated attitude (JPL quaternion)
    //    - P:  The state covariance matrix (3 x 3)
    //    - w:  The current Gyro measurement (3 x 1)
    //    - q:  The proecss noise
    //    - dt: The time step
    for (ibcol = 0; ibcol < 9; ibcol++) {
      I[ibcol] = 0;
    }

    f_a[0] = 0.0;
    f_a[3] = -z_n_b[2];
    f_a[6] = z_n_b[1];
    f_a[1] = z_n_b[2];
    f_a[4] = 0.0;
    f_a[7] = -z_n_b[0];
    f_a[2] = -z_n_b[1];
    f_a[5] = z_n_b[0];
    f_a[8] = 0.0;
    B = dt * dt;
    for (k = 0; k < 3; k++) {
      I[k + 3 * k] = 1;
      for (ibcol = 0; ibcol < 3; ibcol++) {
        Phi[ibcol + 3 * k] = (double)I[ibcol + 3 * k] + -f_a[ibcol + 3 * k] * dt;
      }

      b_z_n_b[k] = z_n_b[k] * dt;
    }

    quatPlusThetaJ(b_z_n_b, dq);
    dv37[0] = x_att[3];
    dv37[4] = -x_att[2];
    dv37[8] = x_att[1];
    dv37[12] = x_att[0];
    dv37[1] = x_att[2];
    dv37[5] = x_att[3];
    dv37[9] = -x_att[0];
    dv37[13] = x_att[1];
    dv37[2] = -x_att[1];
    dv37[6] = x_att[0];
    dv37[10] = x_att[3];
    dv37[14] = x_att[2];
    dv37[3] = -x_att[0];
    dv37[7] = -x_att[1];
    dv37[11] = -x_att[2];
    dv37[15] = x_att[3];
    b_dq[0] = dq[0];
    b_dq[1] = dq[1];
    b_dq[2] = dq[2];
    b_dq[3] = dq[3];
    for (ibcol = 0; ibcol < 4; ibcol++) {
      x_att[ibcol] = 0.0;
      for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
        x_att[ibcol] += dv37[ibcol + (outsize_idx_0 << 2)] * b_dq[outsize_idx_0];
      }
    }

    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        b_x_n_b[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          b_x_n_b[ibcol + 3 * outsize_idx_0] += Phi[ibcol + 3 * itilerow] *
            P_att[itilerow + 3 * outsize_idx_0];
        }

        f_a[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          f_a[ibcol + 3 * outsize_idx_0] += (double)h_a[ibcol + 3 * itilerow] *
            ((double)b_b[itilerow + 3 * outsize_idx_0] * processNoise[1] * B);
        }
      }

      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        b_Phi[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          b_Phi[ibcol + 3 * outsize_idx_0] += b_x_n_b[ibcol + 3 * itilerow] *
            Phi[outsize_idx_0 + 3 * itilerow];
        }

        g_a[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          g_a[ibcol + 3 * outsize_idx_0] += f_a[ibcol + 3 * itilerow] * (double)
            h_a[itilerow + 3 * outsize_idx_0];
        }
      }
    }

    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P_att[outsize_idx_0 + 3 * ibcol] = b_Phi[outsize_idx_0 + 3 * ibcol] +
          g_a[outsize_idx_0 + 3 * ibcol];
      }

      c_a[ibcol] = 0.0;
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        c_a[ibcol] += e_a[ibcol + 3 * outsize_idx_0] * IMU_measurements[16 +
          outsize_idx_0];
      }
    }

    Att_upd(x_att, P_att, c_a, processNoise[0], dt);
    ibcol = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_out->data[ibcol] = xt->data[ibcol];
    }

    for (ibcol = 0; ibcol < 4; ibcol++) {
      xt_out->data[3 + ibcol] = x_att[ibcol];
    }

    ibcol = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      P_apo_out->data[ibcol] = P->data[ibcol];
    }

    ibcol = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 128;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, ibcol, (int)sizeof(double));
    for (ibcol = 0; ibcol < 128; ibcol++) {
      h_u_apo_out->data[ibcol] = -100.0;
    }

    ibcol = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 32;
    emxEnsureCapacity((emxArray__common *)map_out, ibcol, (int)sizeof(double));
    for (ibcol = 0; ibcol < 96; ibcol++) {
      map_out->data[ibcol] = rtNaN;
    }

    init_counter++;
  } else if (init_counter == 10.0) {
    //  done initializing attitude. Insert the estimated attitude and the covariance into the whole state, request features 
    b_fprintf();
    for (outsize_idx_0 = 0; outsize_idx_0 < 32; outsize_idx_0++) {
      updateVect[outsize_idx_0] = 0.0;
    }

    if (1.0 > numPointsPerAnchor) {
      outsize_idx_0 = 0;
    } else {
      outsize_idx_0 = (int)numPointsPerAnchor;
    }

    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      tmp_data[ibcol] = ibcol;
    }

    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      updateVect[tmp_data[ibcol]] = 2.0;
    }

    for (ibcol = 0; ibcol < 4; ibcol++) {
      xt->data[3 + ibcol] = x_att[ibcol];
    }

    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (3 + ibcol)) + 3] =
          P_att[outsize_idx_0 + 3 * ibcol];
      }
    }

    ibcol = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_out->data[ibcol] = xt->data[ibcol];
    }

    for (ibcol = 0; ibcol < 4; ibcol++) {
      xt_out->data[3 + ibcol] = x_att[ibcol];
    }

    ibcol = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      P_apo_out->data[ibcol] = P->data[ibcol];
    }

    ibcol = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 128;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, ibcol, (int)sizeof(double));
    for (ibcol = 0; ibcol < 128; ibcol++) {
      h_u_apo_out->data[ibcol] = -100.0;
    }

    ibcol = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 32;
    emxEnsureCapacity((emxArray__common *)map_out, ibcol, (int)sizeof(double));
    for (ibcol = 0; ibcol < 96; ibcol++) {
      map_out->data[ibcol] = rtNaN;
    }

    init_counter = 11.0;
  } else if (init_counter == 11.0) {
    SLAM_updIT(P, xt, cameraParams->CameraParameters1.RadialDistortion,
               cameraParams->CameraParameters1.FocalLength,
               cameraParams->CameraParameters1.PrincipalPoint,
               cameraParams->CameraParameters2.RadialDistortion,
               cameraParams->CameraParameters2.FocalLength,
               cameraParams->CameraParameters2.PrincipalPoint,
               cameraParams->r_lr, cameraParams->R_lr, cameraParams->R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, (1.0 - rt_powd_snf
                (IMU_measurements[9] / 101325.0, 0.190284)) * 145366.45,
               h_u_apo_out, map_out);
    ibcol = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_out->data[ibcol] = xt->data[ibcol];
    }

    ibcol = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      P_apo_out->data[ibcol] = P->data[ibcol];
    }

    init_counter = 12.0;
  } else {
    SLAM_pred(P, xt, dt, processNoise, IMU_measurements, numStates);

    //  [xt,P] =  SLAM_pred_euler(P, xt, dt, processNoise, IMU_measurements, numStates); 
    SLAM_updIT(P, xt, cameraParams->CameraParameters1.RadialDistortion,
               cameraParams->CameraParameters1.FocalLength,
               cameraParams->CameraParameters1.PrincipalPoint,
               cameraParams->CameraParameters2.RadialDistortion,
               cameraParams->CameraParameters2.FocalLength,
               cameraParams->CameraParameters2.PrincipalPoint,
               cameraParams->r_lr, cameraParams->R_lr, cameraParams->R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, 0.0, h_u_apo_out, map_out);
    ibcol = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = xt->size[0];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      xt_out->data[ibcol] = xt->data[ibcol];
    }

    ibcol = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, ibcol, (int)sizeof(double));
    outsize_idx_0 = P->size[0] * P->size[1];
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      P_apo_out->data[ibcol] = P->data[ibcol];
    }

    // % output asserts for coder
  }
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_free()
{
  emxFree_real_T(&P);
  emxFree_real_T(&xt);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_init()
{
  emxInit_real_T(&P, 2);
  b_emxInit_real_T(&xt, 1);
  init_counter = 0.0;
  memset(&delayBuffer_k[0], 0, 84U * sizeof(double));
}

//
// Arguments    : void
// Return Type  : void
//
void initialized_not_empty_init()
{
  initialized_not_empty = false;
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
