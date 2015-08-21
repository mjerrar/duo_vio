//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 21-Aug-2015 15:06:39
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "fprintf.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "blkdiag.h"
#include "QuatFromRotJ.h"
#include "SLAM_pred.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static double delayBuffer_k[322];
static double last_u[4];

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(double)(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// Arguments    : double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[23]
//                const double imNoise[2]
//                double numPointsPerAnchor
//                double numAnchors
//                const stereoParameters *cameraParams
//                boolean_T resetFlag
//                const double ref[4]
//                const double gains[3]
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
//                double u_out[4]
// Return Type  : void
//
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, const double processNoise[4], const double
          IMU_measurements[23], const double imNoise[2], double
          numPointsPerAnchor, double numAnchors, const stereoParameters
          *cameraParams, boolean_T resetFlag, const double ref[4], const double
          gains[3], emxArray_real_T *h_u_apo_out, emxArray_real_T *xt_out,
          emxArray_real_T *P_apo_out, emxArray_real_T *map_out, double u_out[4])
{
  int outsize_idx_0;
  double K_pos[2];
  emxArray_real_T *b;
  emxArray_real_T *a;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  int ibcol;
  int itilerow;
  int k;
  double dv24[9];
  double b_IMU_measurements[9];
  double dv25[4];
  double d5;
  static const double y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };

  double R_bw[9];
  double yaw;
  double dv26[2];
  double dv27[2];
  double d6;
  double dv28[2];
  double d7;
  double u_out_yaw;
  double u_out_x[3];
  double b_R_bw[3];
  b_fprintf();
  for (outsize_idx_0 = 0; outsize_idx_0 < 23; outsize_idx_0++) {
    d_fprintf(IMU_measurements[outsize_idx_0]);
  }

  f_fprintf();
  K_pos[0] = gains[0];
  K_pos[1] = gains[1];

  //  for coder
  // % imu hack
  //  fprintf('got quaternion: (%.3f, %.3f, %.3f, %.3f)\n', IMU_measurements(20), IMU_measurements(21), IMU_measurements(22), IMU_measurements(23)); 
  // % finish imu hack
  for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
    u_out[outsize_idx_0] = 0.0;
  }

  //  for coder
  if (resetFlag) {
    for (outsize_idx_0 = 0; outsize_idx_0 < 16; outsize_idx_0++) {
      updateVect[outsize_idx_0] = 0.0;
    }
  }

  b_emxInit_real_T(&b, 1);
  b_emxInit_real_T(&a, 1);
  emxInit_real_T(&r4, 2);
  emxInit_real_T(&r5, 2);
  if ((!initialized_not_empty) || resetFlag) {
    initialized_not_empty = true;
    memset(&delayBuffer_k[0], 0, 322U * sizeof(double));
    for (outsize_idx_0 = 0; outsize_idx_0 < 14; outsize_idx_0++) {
      memcpy(&delayBuffer_k[23 * outsize_idx_0], &IMU_measurements[0], 23U *
             sizeof(double));
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
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

    b_IMU_measurements[0] = ((IMU_measurements[19] * IMU_measurements[19] -
      IMU_measurements[20] * IMU_measurements[20]) - IMU_measurements[21] *
      IMU_measurements[21]) + IMU_measurements[22] * IMU_measurements[22];
    b_IMU_measurements[3] = 2.0 * (IMU_measurements[19] * IMU_measurements[20] +
      IMU_measurements[21] * IMU_measurements[22]);
    b_IMU_measurements[6] = 2.0 * (IMU_measurements[19] * IMU_measurements[21] -
      IMU_measurements[20] * IMU_measurements[22]);
    b_IMU_measurements[1] = 2.0 * (IMU_measurements[19] * IMU_measurements[20] -
      IMU_measurements[21] * IMU_measurements[22]);
    b_IMU_measurements[4] = ((-(IMU_measurements[19] * IMU_measurements[19]) +
      IMU_measurements[20] * IMU_measurements[20]) - IMU_measurements[21] *
      IMU_measurements[21]) + IMU_measurements[22] * IMU_measurements[22];
    b_IMU_measurements[7] = 2.0 * (IMU_measurements[20] * IMU_measurements[21] +
      IMU_measurements[19] * IMU_measurements[22]);
    b_IMU_measurements[2] = 2.0 * (IMU_measurements[19] * IMU_measurements[21] +
      IMU_measurements[20] * IMU_measurements[22]);
    b_IMU_measurements[5] = 2.0 * (IMU_measurements[20] * IMU_measurements[21] -
      IMU_measurements[19] * IMU_measurements[22]);
    b_IMU_measurements[8] = ((-(IMU_measurements[19] * IMU_measurements[19]) -
      IMU_measurements[20] * IMU_measurements[20]) + IMU_measurements[21] *
      IMU_measurements[21]) + IMU_measurements[22] * IMU_measurements[22];
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        dv24[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          dv24[ibcol + 3 * outsize_idx_0] += R_bc[itilerow + 3 * ibcol] *
            b_IMU_measurements[itilerow + 3 * outsize_idx_0];
        }
      }
    }

    QuatFromRotJ(dv24, dv25);
    ibcol = xt->size[0];
    xt->size[0] = 13 + b->size[0];
    emxEnsureCapacity((emxArray__common *)xt, ibcol, (int)sizeof(double));
    xt->data[0] = 0.0;
    xt->data[1] = 0.0;
    xt->data[2] = 0.0;
    for (ibcol = 0; ibcol < 4; ibcol++) {
      xt->data[ibcol + 3] = dv25[ibcol];
    }

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

    //  initial real vector
    d5 = numAnchors * (6.0 + numPointsPerAnchor);
    ibcol = r4->size[0] * r4->size[1];
    r4->size[0] = (int)numStates;
    r4->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r4, ibcol, (int)sizeof(double));
    outsize_idx_0 = (int)numStates * (int)numStates;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      r4->data[ibcol] = 0.0;
    }

    ibcol = r5->size[0] * r5->size[1];
    r5->size[0] = (int)d5;
    r5->size[1] = (int)d5;
    emxEnsureCapacity((emxArray__common *)r5, ibcol, (int)sizeof(double));
    outsize_idx_0 = (int)d5 * (int)d5;
    for (ibcol = 0; ibcol < outsize_idx_0; ibcol++) {
      r5->data[ibcol] = 0.0;
    }

    blkdiag(r4, r5, P);

    //  initial error state covariance
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[outsize_idx_0 + P->size[0] * ibcol] = 0.0;
      }
    }

    //  position
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        P->data[(outsize_idx_0 + P->size[0] * (3 + ibcol)) + 3] = 0.0;
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
          y[outsize_idx_0 + 3 * ibcol];
      }
    }

    //  gyro bias
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

    for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
      last_u[outsize_idx_0] = 0.0;
    }

    //  the last control outputs (in camera frame)
  } else {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    dv24[0] = ((xt->data[3] * xt->data[3] - xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv24[3] = 2.0 * (xt->data[3] * xt->data[4] + xt->data[5] * xt->data[6]);
    dv24[6] = 2.0 * (xt->data[3] * xt->data[5] - xt->data[4] * xt->data[6]);
    dv24[1] = 2.0 * (xt->data[3] * xt->data[4] - xt->data[5] * xt->data[6]);
    dv24[4] = ((-(xt->data[3] * xt->data[3]) + xt->data[4] * xt->data[4]) -
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    dv24[7] = 2.0 * (xt->data[4] * xt->data[5] + xt->data[3] * xt->data[6]);
    dv24[2] = 2.0 * (xt->data[3] * xt->data[5] + xt->data[4] * xt->data[6]);
    dv24[5] = 2.0 * (xt->data[4] * xt->data[5] - xt->data[3] * xt->data[6]);
    dv24[8] = ((-(xt->data[3] * xt->data[3]) - xt->data[4] * xt->data[4]) +
               xt->data[5] * xt->data[5]) + xt->data[6] * xt->data[6];
    for (ibcol = 0; ibcol < 3; ibcol++) {
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        R_bw[ibcol + 3 * outsize_idx_0] = 0.0;
        for (itilerow = 0; itilerow < 3; itilerow++) {
          R_bw[ibcol + 3 * outsize_idx_0] += R_bc[ibcol + 3 * itilerow] *
            dv24[itilerow + 3 * outsize_idx_0];
        }
      }
    }

    yaw = rt_atan2d_snf(R_bw[3], R_bw[0]);
    dv26[0] = xt->data[0] - ref[0];
    dv26[1] = xt->data[7];
    d5 = 0.0;
    for (ibcol = 0; ibcol < 2; ibcol++) {
      d5 += -K_pos[ibcol] * dv26[ibcol];
    }

    //  control commands in world frame
    dv27[0] = xt->data[1] - ref[1];
    dv27[1] = xt->data[8];
    d6 = 0.0;
    for (ibcol = 0; ibcol < 2; ibcol++) {
      d6 += -K_pos[ibcol] * dv27[ibcol];
    }

    //  control commands in world frame
    dv28[0] = xt->data[2] - ref[2];
    dv28[1] = xt->data[9];
    d7 = 0.0;
    for (ibcol = 0; ibcol < 2; ibcol++) {
      d7 += -K_pos[ibcol] * dv28[ibcol];
    }

    //  control commands in world frame
    u_out_yaw = -gains[2] * (yaw - ref[3]);
    u_out_x[0] = d5;
    u_out_x[1] = d6;
    u_out_x[2] = d7;
    for (ibcol = 0; ibcol < 3; ibcol++) {
      b_R_bw[ibcol] = 0.0;
      for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
        b_R_bw[ibcol] += R_bw[ibcol + 3 * outsize_idx_0] * u_out_x[outsize_idx_0];
      }
    }

    for (ibcol = 0; ibcol < 3; ibcol++) {
      u_out[ibcol] = b_R_bw[ibcol];
    }

    u_out[3] = u_out_yaw;
    v_fprintf(xt->data[0] - ref[0], xt->data[1] - ref[1], xt->data[2] - ref[2],
              yaw - ref[3], d5, d6, d7, u_out_yaw);
    for (k = 0; k < 13; k++) {
      for (ibcol = 0; ibcol < 23; ibcol++) {
        delayBuffer_k[ibcol + 23 * (13 - k)] = delayBuffer_k[ibcol + 23 * (12 -
          k)];
      }
    }

    memcpy(&delayBuffer_k[0], &IMU_measurements[0], 23U * sizeof(double));

    //  IMU_measurements = delayBuffer_k(:,1);
    for (outsize_idx_0 = 0; outsize_idx_0 < 4; outsize_idx_0++) {
      dv25[outsize_idx_0] = 0.0 * last_u[outsize_idx_0];
    }

    SLAM_pred(P, xt, dt, processNoise, IMU_measurements, numStates, dv25);

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

    last_u[0] = d5;
    last_u[1] = d6;
    last_u[2] = d7;
    last_u[3] = u_out_yaw;

    // % output asserts for coder
  }

  emxFree_real_T(&r5);
  emxFree_real_T(&r4);
  emxFree_real_T(&a);
  emxFree_real_T(&b);
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
