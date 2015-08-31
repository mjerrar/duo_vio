//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Aug-2015 09:51:22
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "QuatFromRotJ.h"
#include "predictMeasurement_stereo.h"
#include "predictMeasurement_left.h"
#include "ros_error.h"
#include "any.h"
#include "eye.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
//
//  INPUT ARGUMENTS:
//  - xt:                   The current state
//  - errorStateSize:       The size of the error state
//  - z_all_l:              The feature points in the left camera (2N x 1)
//  - map:                  Map of the estimated feature points (3 x N)
//  - indMeas:              The indices of the valid feature points
//  - cameraparams:         A struct with the fields
//     - focal length, center of projection, radial distortion parameters of
//     both cameras
//     - r_lr:              Translation of right camera in left camera frame
//     - R_lr:              Orientation of right camera in left camera frame
//
//  where N is number of points in the image
//
//  OUTPUT ARGUMENTS:
//  - r:    The residual (residualDim*M x 1)
//  - H_xc: The Jacobian of the measurement function with respect to the camera states (residualDim*M x length(xt))
//
//  where M =  length(indMeas) is the number of valid points in the image
// Arguments    : const emxArray_real_T *b_xt
//                double errorStateSize
//                double stateSize
//                const double z_all_l[32]
//                const double indMeas_data[]
//                const int indMeas_size[1]
//                const emxArray_real_T *map
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double c_noiseParameters_orientation_n
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                const VIOMeasurements *measurements
//                double b_height_offset_pressure
//                const VIOParameters b_VIOParameters
//                double r_data[]
//                int r_size[1]
//                emxArray_real_T *H
//                double h_u_data[]
//                int h_u_size[1]
//                double R_data[]
//                int R_size[2]
// Return Type  : void
//
void b_getH_R_res(const emxArray_real_T *b_xt, double errorStateSize, double
                  stateSize, const double z_all_l[32], const double
                  indMeas_data[], const int indMeas_size[1], const
                  emxArray_real_T *map, const emxArray_real_T *anchorIdx, const
                  emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                  *b_m_vect, const double noiseParameters_image_noise[2], double
                  c_noiseParameters_orientation_n, double
                  noiseParameters_pressure_noise, double
                  noiseParameters_ext_pos_noise, double
                  noiseParameters_ext_att_noise, const VIOMeasurements
                  *measurements, double b_height_offset_pressure, const
                  VIOParameters b_VIOParameters, double r_data[], int r_size[1],
                  emxArray_real_T *H, double h_u_data[], int h_u_size[1], double
                  R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double numPointsPerAnchor;
  double numStatesPerAnchor;
  double numErrorStatesPerAnchor;
  double R_cw[9];
  int ar;
  int b_j1;
  emxArray_real_T *H_xc;
  double z_data[32];
  int k;
  emxArray_real_T *H_orientation;
  emxArray_real_T *H_pressure;
  emxArray_int32_T *r2;
  emxArray_real_T *C;
  boolean_T bv1[3];
  double c_xt;
  double b_map[3];
  double r_orientation[3];
  int br;
  double dv1[2];
  double indMeas;
  double h_u_To_h_ci_l[6];
  signed char b_k;
  signed char iv2[2];
  double d_xt[36];
  double b_h_u_To_h_ci_l[24];
  int i2;
  int kidx;
  double b_stateSize;
  double c_stateSize;
  double d_stateSize;
  double e_stateSize;
  double f_stateSize;
  double g_stateSize;
  double h_stateSize;
  double i_stateSize;
  double j_stateSize;
  double k_stateSize;
  double l_stateSize;
  double m_stateSize;
  double n_stateSize;
  double o_stateSize;
  double p_stateSize;
  double q_stateSize;
  double r_stateSize;
  double s_stateSize;
  double t_stateSize;
  double u_stateSize;
  double v_stateSize;
  double w_stateSize;
  double x_stateSize;
  double y_stateSize;
  double ab_stateSize;
  double bb_stateSize;
  double cb_stateSize;
  double db_stateSize;
  double eb_stateSize;
  double fb_stateSize;
  double gb_stateSize;
  double hb_stateSize;
  double ib_stateSize;
  double jb_stateSize;
  double kb_stateSize;
  double lb_stateSize;
  double mb_stateSize;
  double nb_stateSize;
  double ob_stateSize;
  double pb_stateSize;
  double qb_stateSize;
  double rb_stateSize;
  double sb_stateSize;
  double tb_stateSize;
  double ub_stateSize;
  double vb_stateSize;
  double wb_stateSize;
  double xb_stateSize;
  double e_xt[9];
  double y[3];
  double b_y;
  double f_xt;
  double dv2[9];
  double b_R_cw[3];
  double g_xt[9];
  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double b_anchorIdx;
  double v[2];
  int ic;
  int ia;
  double r_pressure;
  double r_ext_pose[6];
  emxArray_real_T *H_ext_pose;
  double d[4];
  emxArray_real_T *r3;
  double A_data[256];
  double R_vision_data[1024];
  int R_vision_size_idx_0;
  int residual_size;
  int orientation_idx;
  int pressure_idx;
  int ext_pose_idx;
  double b_measurements[9];
  signed char I[9];
  double c_measurements[9];
  int tmp_data[32];
  signed char iv3[3];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  signed char iv4[6];
  emxInit_real_T(&H_xm, 2);
  numPointsPerAnchor = b_VIOParameters.num_points_per_anchor;
  numStatesPerAnchor = 7.0 + b_VIOParameters.num_points_per_anchor;
  numErrorStatesPerAnchor = 6.0 + b_VIOParameters.num_points_per_anchor;

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4]) -
             b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] * b_xt->data[6]);
  R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] * b_xt->data[6]);
  R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] * b_xt->data[6]);
  R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data[4])
             - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] * b_xt->data[6]);
  R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] * b_xt->data[6]);
  R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] * b_xt->data[6]);
  R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data[4])
             + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];

  //  camera parameters for the left and right camera
  //  Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1);
  //  Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2);
  ar = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  ar = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  b_j1 = (indMeas_size[0] << 1) * (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H_xm->data[ar] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ar = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, ar, (int)sizeof(double));
  ar = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ar, (int)sizeof(double));
  b_j1 = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (ar = 0; ar < b_j1; ar++) {
    H_xc->data[ar] = 0.0;
  }

  b_j1 = indMeas_size[0] << 1;
  for (ar = 0; ar < b_j1; ar++) {
    z_data[ar] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  b_j1 = indMeas_size[0] << 1;
  for (ar = 0; ar < b_j1; ar++) {
    h_u_data[ar] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_orientation, 2);
  emxInit_real_T(&H_pressure, 2);
  emxInit_int32_T(&r2, 1);
  emxInit_real_T(&C, 2);
  while (k <= indMeas_size[0] - 1) {
    for (ar = 0; ar < 3; ar++) {
      bv1[ar] = rtIsNaN(map->data[ar + map->size[0] * ((int)indMeas_data[k] - 1)]);
    }

    if (c_any(bv1)) {
      b_ros_error();
    }

    c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * numStatesPerAnchor) + 7.0) + featureAnchorIdx->data[(int)
      indMeas_data[k] - 1]) - 1];
    for (ar = 0; ar < 3; ar++) {
      b_map[ar] = map->data[ar + map->size[0] * ((int)indMeas_data[k] - 1)] -
        b_xt->data[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      r_orientation[ar] = 0.0;
      for (br = 0; br < 3; br++) {
        r_orientation[ar] += c_xt * R_cw[ar + 3 * br] * b_map[br];
      }
    }

    //  = R_cw*(rho*anchorPos + anchorRot'*m - rho*r_wc_pred)
    predictMeasurement_left(r_orientation, dv1);
    b_j1 = k << 1;
    for (ar = 0; ar < 2; ar++) {
      h_u_data[ar + b_j1] = dv1[ar];
    }

    b_j1 = k << 1;
    indMeas = (indMeas_data[k] - 1.0) * 2.0;
    for (ar = 0; ar < 2; ar++) {
      z_data[ar + b_j1] = z_all_l[(int)(indMeas + (1.0 + (double)ar)) - 1];
    }

    //     %% computation of H(x)
    h_u_To_h_ci_l[0] = 1.0 / r_orientation[2];
    h_u_To_h_ci_l[2] = 0.0;
    h_u_To_h_ci_l[4] = -r_orientation[0] / (r_orientation[2] * r_orientation[2]);
    h_u_To_h_ci_l[1] = 0.0;
    h_u_To_h_ci_l[3] = 1.0 / r_orientation[2];
    h_u_To_h_ci_l[5] = -r_orientation[1] / (r_orientation[2] * r_orientation[2]);
    b_k = (signed char)((signed char)k << 1);
    for (ar = 0; ar < 2; ar++) {
      iv2[ar] = (signed char)(ar + b_k);
    }

    b_j1 = H_xc->size[1];
    ar = r2->size[0];
    r2->size[0] = b_j1;
    emxEnsureCapacity((emxArray__common *)r2, ar, (int)sizeof(int));
    for (ar = 0; ar < b_j1; ar++) {
      r2->data[ar] = ar;
    }

    c_xt = -b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * numStatesPerAnchor) + 7.0) + featureAnchorIdx->data[(int)
      indMeas_data[k] - 1]) - 1];
    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        d_xt[br + 3 * ar] = c_xt * R_cw[br + 3 * ar];
      }
    }

    d_xt[9] = 0.0;
    d_xt[12] = -r_orientation[2];
    d_xt[15] = r_orientation[1];
    d_xt[10] = r_orientation[2];
    d_xt[13] = 0.0;
    d_xt[16] = -r_orientation[0];
    d_xt[11] = -r_orientation[1];
    d_xt[14] = r_orientation[0];
    d_xt[17] = 0.0;
    for (ar = 0; ar < 6; ar++) {
      for (br = 0; br < 3; br++) {
        d_xt[br + 3 * (ar + 6)] = 0.0;
      }
    }

    for (ar = 0; ar < 2; ar++) {
      for (br = 0; br < 12; br++) {
        b_h_u_To_h_ci_l[ar + (br << 1)] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          b_h_u_To_h_ci_l[ar + (br << 1)] += h_u_To_h_ci_l[ar + (i2 << 1)] *
            d_xt[i2 + 3 * br];
        }
      }
    }

    kidx = r2->size[0];
    for (ar = 0; ar < kidx; ar++) {
      for (br = 0; br < 2; br++) {
        H_xc->data[iv2[br] + H_xc->size[0] * r2->data[ar]] = b_h_u_To_h_ci_l[br
          + (ar << 1)];
      }
    }

    //     %% anchor state derivatives
    //  fp = anchorPos + anchorRot'*m/rho - r_wc_pred;
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    for (ar = 0; ar < 3; ar++) {
      r_orientation[ar] = b_xt->data[(int)(b_stateSize + (1.0 + (double)ar)) - 1];
    }

    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    c_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    d_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    e_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    f_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    g_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    h_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    i_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    j_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    k_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    l_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    m_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    n_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    o_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    p_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    q_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    r_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    s_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    t_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    u_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    v_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    w_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    x_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    y_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    ab_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    bb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    cb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    db_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    eb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    fb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    gb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    hb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    ib_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    jb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    kb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    lb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    mb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    nb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    ob_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    pb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    qb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    rb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    sb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    tb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    ub_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    vb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    wb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    xb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + numPointsPerAnchor);
    e_xt[0] = ((b_xt->data[(int)(b_stateSize + 4.0) - 1] * b_xt->data[(int)
                (c_stateSize + 4.0) - 1] - b_xt->data[(int)(d_stateSize + 5.0) -
                1] * b_xt->data[(int)(e_stateSize + 5.0) - 1]) - b_xt->data[(int)
               (f_stateSize + 6.0) - 1] * b_xt->data[(int)(g_stateSize + 6.0) -
               1]) + b_xt->data[(int)(h_stateSize + 7.0) - 1] * b_xt->data[(int)
      (i_stateSize + 7.0) - 1];
    e_xt[1] = 2.0 * (b_xt->data[(int)(j_stateSize + 4.0) - 1] * b_xt->data[(int)
                     (k_stateSize + 5.0) - 1] + b_xt->data[(int)(l_stateSize +
      6.0) - 1] * b_xt->data[(int)(m_stateSize + 7.0) - 1]);
    e_xt[2] = 2.0 * (b_xt->data[(int)(n_stateSize + 4.0) - 1] * b_xt->data[(int)
                     (o_stateSize + 6.0) - 1] - b_xt->data[(int)(p_stateSize +
      5.0) - 1] * b_xt->data[(int)(q_stateSize + 7.0) - 1]);
    e_xt[3] = 2.0 * (b_xt->data[(int)(r_stateSize + 4.0) - 1] * b_xt->data[(int)
                     (s_stateSize + 5.0) - 1] - b_xt->data[(int)(t_stateSize +
      6.0) - 1] * b_xt->data[(int)(u_stateSize + 7.0) - 1]);
    e_xt[4] = ((-(b_xt->data[(int)(v_stateSize + 4.0) - 1] * b_xt->data[(int)
                  (w_stateSize + 4.0) - 1]) + b_xt->data[(int)(x_stateSize + 5.0)
                - 1] * b_xt->data[(int)(y_stateSize + 5.0) - 1]) - b_xt->data
               [(int)(ab_stateSize + 6.0) - 1] * b_xt->data[(int)(bb_stateSize +
                6.0) - 1]) + b_xt->data[(int)(cb_stateSize + 7.0) - 1] *
      b_xt->data[(int)(db_stateSize + 7.0) - 1];
    e_xt[5] = 2.0 * (b_xt->data[(int)(eb_stateSize + 5.0) - 1] * b_xt->data[(int)
                     (fb_stateSize + 6.0) - 1] + b_xt->data[(int)(gb_stateSize +
      4.0) - 1] * b_xt->data[(int)(hb_stateSize + 7.0) - 1]);
    e_xt[6] = 2.0 * (b_xt->data[(int)(ib_stateSize + 4.0) - 1] * b_xt->data[(int)
                     (jb_stateSize + 6.0) - 1] + b_xt->data[(int)(kb_stateSize +
      5.0) - 1] * b_xt->data[(int)(lb_stateSize + 7.0) - 1]);
    e_xt[7] = 2.0 * (b_xt->data[(int)(mb_stateSize + 5.0) - 1] * b_xt->data[(int)
                     (nb_stateSize + 6.0) - 1] - b_xt->data[(int)(ob_stateSize +
      4.0) - 1] * b_xt->data[(int)(pb_stateSize + 7.0) - 1]);
    e_xt[8] = ((-(b_xt->data[(int)(qb_stateSize + 4.0) - 1] * b_xt->data[(int)
                  (rb_stateSize + 4.0) - 1]) - b_xt->data[(int)(sb_stateSize +
      5.0) - 1] * b_xt->data[(int)(tb_stateSize + 5.0) - 1]) + b_xt->data[(int)
               (ub_stateSize + 6.0) - 1] * b_xt->data[(int)(vb_stateSize + 6.0)
               - 1]) + b_xt->data[(int)(wb_stateSize + 7.0) - 1] * b_xt->data
      [(int)(xb_stateSize + 7.0) - 1];
    for (ar = 0; ar < 3; ar++) {
      y[ar] = 0.0;
      for (br = 0; br < 3; br++) {
        b_y = y[ar] + e_xt[ar + 3 * br] * b_m_vect->data[br + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
        y[ar] = b_y;
      }
    }

    c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data
      [(int)indMeas_data[k] - 1]) - 1];
    f_xt = -b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data
      [(int)indMeas_data[k] - 1]) - 1];
    dv2[0] = 0.0;
    dv2[3] = -y[2];
    dv2[6] = y[1];
    dv2[1] = y[2];
    dv2[4] = 0.0;
    dv2[7] = -y[0];
    dv2[2] = -y[1];
    dv2[5] = y[0];
    dv2[8] = 0.0;
    kidx = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
    for (ar = 0; ar < 3; ar++) {
      b_map[ar] = r_orientation[ar] - b_xt->data[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      b_R_cw[ar] = 0.0;
      for (br = 0; br < 3; br++) {
        b_R_cw[ar] += R_cw[ar + 3 * br] * b_map[br];
      }
    }

    b_j1 = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas_data[k]
                 - 1]);
    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        g_xt[ar + 3 * br] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          g_xt[ar + 3 * br] += f_xt * R_cw[ar + 3 * i2] * dv2[i2 + 3 * br];
        }
      }
    }

    ar = H_orientation->size[0] * H_orientation->size[1];
    H_orientation->size[0] = 3;
    H_orientation->size[1] = (kidx + b_j1) + 7;
    emxEnsureCapacity((emxArray__common *)H_orientation, ar, (int)sizeof(double));
    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_orientation->data[br + H_orientation->size[0] * ar] = c_xt * R_cw[br +
          3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_orientation->data[br + H_orientation->size[0] * (ar + 3)] = g_xt[br +
          3 * ar];
      }
    }

    for (ar = 0; ar < kidx; ar++) {
      for (br = 0; br < 3; br++) {
        H_orientation->data[br + H_orientation->size[0] * (ar + 6)] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      H_orientation->data[ar + H_orientation->size[0] * (6 + kidx)] = b_R_cw[ar];
    }

    for (ar = 0; ar < b_j1; ar++) {
      for (br = 0; br < 3; br++) {
        H_orientation->data[br + H_orientation->size[0] * ((ar + kidx) + 7)] =
          0.0;
      }
    }

    b_k = (signed char)((signed char)k << 1);
    for (ar = 0; ar < 2; ar++) {
      iv2[ar] = (signed char)(ar + b_k);
    }

    if (rtIsNaN(numErrorStatesPerAnchor)) {
      br = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (6.0 + numPointsPerAnchor < 1.0) {
      br = -1;
      anew = 1.0;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (rtIsInf(numErrorStatesPerAnchor)) {
      br = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else {
      anew = 1.0;
      ndbl = floor((numErrorStatesPerAnchor - 1.0) + 0.5);
      apnd = 1.0 + ndbl;
      cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
      absb = fabs(6.0 + numPointsPerAnchor);
      if ((1.0 >= absb) || rtIsNaN(absb)) {
        absb = 1.0;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = 6.0 + numPointsPerAnchor;
      } else if (cdiff > 0.0) {
        apnd = 1.0 + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        br = (int)ndbl - 1;
      } else {
        br = -1;
      }
    }

    ar = H_pressure->size[0] * H_pressure->size[1];
    H_pressure->size[0] = 1;
    H_pressure->size[1] = br + 1;
    emxEnsureCapacity((emxArray__common *)H_pressure, ar, (int)sizeof(double));
    if (br + 1 > 0) {
      H_pressure->data[0] = anew;
      if (br + 1 > 1) {
        H_pressure->data[br] = apnd;
        kidx = (br + (br < 0)) >> 1;
        for (b_j1 = 1; b_j1 < kidx; b_j1++) {
          H_pressure->data[b_j1] = anew + (double)b_j1;
          H_pressure->data[br - b_j1] = apnd - (double)b_j1;
        }

        if (kidx << 1 == br) {
          H_pressure->data[kidx] = (anew + apnd) / 2.0;
        } else {
          H_pressure->data[kidx] = anew + (double)kidx;
          H_pressure->data[kidx + 1] = apnd - (double)kidx;
        }
      }
    }

    b_anchorIdx = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
      numPointsPerAnchor);
    ar = r2->size[0];
    r2->size[0] = H_pressure->size[1];
    emxEnsureCapacity((emxArray__common *)r2, ar, (int)sizeof(int));
    b_j1 = H_pressure->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      r2->data[ar] = (int)(b_anchorIdx + H_pressure->data[H_pressure->size[0] *
                           ar]) - 1;
    }

    v[1] = H_orientation->size[1];
    ar = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
    ar = C->size[0] * C->size[1];
    C->size[1] = (int)v[1];
    emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
    b_j1 = (int)v[1] << 1;
    for (ar = 0; ar < b_j1; ar++) {
      C->data[ar] = 0.0;
    }

    kidx = (H_orientation->size[1] - 1) << 1;
    for (b_j1 = 0; b_j1 <= kidx; b_j1 += 2) {
      for (ic = b_j1; ic + 1 <= b_j1 + 2; ic++) {
        C->data[ic] = 0.0;
      }
    }

    br = 0;
    for (b_j1 = 0; b_j1 <= kidx; b_j1 += 2) {
      ar = 0;
      for (i2 = br; i2 + 1 <= br + 3; i2++) {
        if (H_orientation->data[i2] != 0.0) {
          ia = ar;
          for (ic = b_j1; ic + 1 <= b_j1 + 2; ic++) {
            ia++;
            C->data[ic] += H_orientation->data[i2] * h_u_To_h_ci_l[ia - 1];
          }
        }

        ar += 2;
      }

      br += 3;
    }

    b_j1 = C->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      for (br = 0; br < 2; br++) {
        H_xm->data[iv2[br] + H_xm->size[0] * r2->data[ar]] = C->data[br +
          C->size[0] * ar];
      }
    }

    k++;
  }

  emxFree_real_T(&C);
  emxFree_int32_T(&r2);
  for (kidx = 0; kidx < 3; kidx++) {
    r_orientation[kidx] = 0.0;
  }

  r_pressure = 0.0;
  for (kidx = 0; kidx < 6; kidx++) {
    r_ext_pose[kidx] = 0.0;
  }

  ar = H_orientation->size[0] * H_orientation->size[1];
  H_orientation->size[0] = 3;
  H_orientation->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_orientation, ar, (int)sizeof(double));
  b_j1 = 3 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H_orientation->data[ar] = 0.0;
  }

  ar = H_pressure->size[0] * H_pressure->size[1];
  H_pressure->size[0] = 1;
  H_pressure->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_pressure, ar, (int)sizeof(double));
  b_j1 = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H_pressure->data[ar] = 0.0;
  }

  emxInit_real_T(&H_ext_pose, 2);
  ar = H_ext_pose->size[0] * H_ext_pose->size[1];
  H_ext_pose->size[0] = 6;
  H_ext_pose->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_ext_pose, ar, (int)sizeof(double));
  b_j1 = 6 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H_ext_pose->data[ar] = 0.0;
  }

  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (ar = 0; ar < 4; ar++) {
    d[ar] = 0.0;
  }

  for (kidx = 0; kidx < 2; kidx++) {
    d[kidx + (kidx << 1)] = v[kidx];
  }

  emxInit_real_T(&r3, 2);
  b_eye((double)indMeas_size[0], r3);
  ia = r3->size[0];
  ic = r3->size[1];
  b_j1 = r3->size[0] * r3->size[1];
  for (ar = 0; ar < b_j1; ar++) {
    A_data[ar] = r3->data[ar];
  }

  emxFree_real_T(&r3);
  R_vision_size_idx_0 = (signed char)(ia << 1);
  kidx = -1;
  for (b_j1 = 1; b_j1 <= ic; b_j1++) {
    for (br = 0; br < 2; br++) {
      for (ar = 1; ar <= ia; ar++) {
        for (i2 = 0; i2 < 2; i2++) {
          kidx++;
          R_vision_data[kidx] = A_data[(ar + ia * (b_j1 - 1)) - 1] * d[i2 + (br <<
            1)];
        }
      }
    }
  }

  residual_size = indMeas_size[0] << 1;
  orientation_idx = 0;
  pressure_idx = 0;
  ext_pose_idx = 0;
  if (b_VIOParameters.use_orientation) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_measurements[0] = ((measurements->att_fmu[0] * measurements->att_fmu[0] -
                          measurements->att_fmu[1] * measurements->att_fmu[1]) -
                         measurements->att_fmu[2] * measurements->att_fmu[2]) +
      measurements->att_fmu[3] * measurements->att_fmu[3];
    b_measurements[3] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] + measurements->att_fmu[2] * measurements->att_fmu[3]);
    b_measurements[6] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] - measurements->att_fmu[1] * measurements->att_fmu[3]);
    b_measurements[1] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] - measurements->att_fmu[2] * measurements->att_fmu[3]);
    b_measurements[4] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          + measurements->att_fmu[1] * measurements->att_fmu[1])
                         - measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    b_measurements[7] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] + measurements->att_fmu[0] * measurements->att_fmu[3]);
    b_measurements[2] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] + measurements->att_fmu[1] * measurements->att_fmu[3]);
    b_measurements[5] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] - measurements->att_fmu[0] * measurements->att_fmu[3]);
    b_measurements[8] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          - measurements->att_fmu[1] * measurements->att_fmu[1])
                         + measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        e_xt[ar + 3 * br] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          e_xt[ar + 3 * br] += R_bc[i2 + 3 * ar] * b_measurements[i2 + 3 * br];
        }
      }

      for (br = 0; br < 3; br++) {
        dv2[ar + 3 * br] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          dv2[ar + 3 * br] += e_xt[ar + 3 * i2] * R_cw[br + 3 * i2];
        }
      }
    }

    QuatFromRotJ(dv2, d);
    for (kidx = 0; kidx < 3; kidx++) {
      r_orientation[kidx] = d[kidx];
    }

    for (ar = 0; ar < 9; ar++) {
      I[ar] = 0;
    }

    for (k = 0; k < 3; k++) {
      I[k + 3 * k] = 1;
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_orientation->data[br + H_orientation->size[0] * (3 + ar)] = I[br + 3 *
          ar];
      }
    }

    orientation_idx = residual_size;
    residual_size += 3;
  }

  if (b_VIOParameters.use_pressure) {
    r_pressure = (1.0 - rt_powd_snf(measurements->bar_fmu / 101325.0, 0.190284))
      * 145366.45 - b_height_offset_pressure;
    H_pressure->data[2] = 1.0;
    pressure_idx = residual_size;
    residual_size++;
  }

  if (b_VIOParameters.use_ext_pose) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    c_measurements[0] = ((measurements->att_ext[0] * measurements->att_ext[0] -
                          measurements->att_ext[1] * measurements->att_ext[1]) -
                         measurements->att_ext[2] * measurements->att_ext[2]) +
      measurements->att_ext[3] * measurements->att_ext[3];
    c_measurements[3] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [1] + measurements->att_ext[2] * measurements->att_ext[3]);
    c_measurements[6] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [2] - measurements->att_ext[1] * measurements->att_ext[3]);
    c_measurements[1] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [1] - measurements->att_ext[2] * measurements->att_ext[3]);
    c_measurements[4] = ((-(measurements->att_ext[0] * measurements->att_ext[0])
                          + measurements->att_ext[1] * measurements->att_ext[1])
                         - measurements->att_ext[2] * measurements->att_ext[2])
      + measurements->att_ext[3] * measurements->att_ext[3];
    c_measurements[7] = 2.0 * (measurements->att_ext[1] * measurements->att_ext
      [2] + measurements->att_ext[0] * measurements->att_ext[3]);
    c_measurements[2] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [2] + measurements->att_ext[1] * measurements->att_ext[3]);
    c_measurements[5] = 2.0 * (measurements->att_ext[1] * measurements->att_ext
      [2] - measurements->att_ext[0] * measurements->att_ext[3]);
    c_measurements[8] = ((-(measurements->att_ext[0] * measurements->att_ext[0])
                          - measurements->att_ext[1] * measurements->att_ext[1])
                         + measurements->att_ext[2] * measurements->att_ext[2])
      + measurements->att_ext[3] * measurements->att_ext[3];
    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        b_measurements[ar + 3 * br] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          b_measurements[ar + 3 * br] += c_measurements[ar + 3 * i2] * R_cw[br +
            3 * i2];
        }
      }
    }

    QuatFromRotJ(b_measurements, d);
    for (ar = 0; ar < 3; ar++) {
      r_ext_pose[ar] = measurements->pos_ext[ar] - b_xt->data[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      r_ext_pose[ar + 3] = d[ar];
    }

    for (ar = 0; ar < 9; ar++) {
      I[ar] = 0;
    }

    for (k = 0; k < 3; k++) {
      I[k + 3 * k] = 1;
    }

    memset(&R_cw[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      R_cw[k + 3 * k] = 1.0;
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_ext_pose->data[br + H_ext_pose->size[0] * ar] = I[br + 3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_ext_pose->data[br + H_ext_pose->size[0] * (ar + 3)] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_ext_pose->data[(br + H_ext_pose->size[0] * ar) + 3] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        H_ext_pose->data[(br + H_ext_pose->size[0] * (ar + 3)) + 3] = R_cw[br +
          3 * ar];
      }
    }

    ext_pose_idx = residual_size;
    residual_size += 6;
  }

  r_size[0] = residual_size;
  for (ar = 0; ar < residual_size; ar++) {
    r_data[ar] = 0.0;
  }

  ar = H->size[0] * H->size[1];
  H->size[0] = residual_size;
  H->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H, ar, (int)sizeof(double));
  b_j1 = residual_size * (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < b_j1; ar++) {
    H->data[ar] = 0.0;
  }

  R_size[0] = residual_size;
  R_size[1] = residual_size;
  b_j1 = residual_size * residual_size;
  for (ar = 0; ar < b_j1; ar++) {
    R_data[ar] = 0.0;
  }

  ar = indMeas_size[0] << 1;
  if (1 > ar) {
    b_j1 = 0;
  } else {
    b_j1 = ar;
  }

  for (ar = 0; ar < b_j1; ar++) {
    tmp_data[ar] = ar;
  }

  for (ar = 0; ar < b_j1; ar++) {
    r_data[tmp_data[ar]] = z_data[ar] - h_u_data[ar];
  }

  b_j1 = H_xc->size[1];
  for (ar = 0; ar < b_j1; ar++) {
    kidx = H_xc->size[0];
    for (br = 0; br < kidx; br++) {
      H->data[br + H->size[0] * ar] = H_xc->data[br + H_xc->size[0] * ar];
    }
  }

  b_j1 = H_xm->size[1];
  for (ar = 0; ar < b_j1; ar++) {
    kidx = H_xm->size[0];
    for (br = 0; br < kidx; br++) {
      H->data[br + H->size[0] * (ar + H_xc->size[1])] = H_xm->data[br +
        H_xm->size[0] * ar];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  b_j1 = (signed char)(ic << 1);
  for (ar = 0; ar < b_j1; ar++) {
    for (br = 0; br < R_vision_size_idx_0; br++) {
      R_data[br + R_size[0] * ar] = R_vision_data[br + R_vision_size_idx_0 * ar];
    }
  }

  if (b_VIOParameters.use_orientation) {
    for (ar = 0; ar < 3; ar++) {
      r_data[ar + orientation_idx] = r_orientation[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      iv3[ar] = (signed char)(ar + (signed char)orientation_idx);
    }

    b_j1 = H_orientation->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      for (br = 0; br < 3; br++) {
        H->data[iv3[br] + H->size[0] * ar] = H_orientation->data[br +
          H_orientation->size[0] * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[(br + orientation_idx) + R_size[0] * (ar + orientation_idx)] =
          c_noiseParameters_orientation_n * (double)b[br + 3 * ar];
      }
    }
  }

  emxFree_real_T(&H_orientation);
  if (b_VIOParameters.use_pressure) {
    r_data[pressure_idx] = r_pressure;
    b_j1 = H_pressure->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      H->data[pressure_idx + H->size[0] * ar] = H_pressure->data
        [H_pressure->size[0] * ar];
    }

    R_data[pressure_idx + R_size[0] * pressure_idx] =
      noiseParameters_pressure_noise;
  }

  emxFree_real_T(&H_pressure);
  if (b_VIOParameters.use_ext_pose) {
    for (ar = 0; ar < 6; ar++) {
      r_data[ar + ext_pose_idx] = r_ext_pose[ar];
    }

    for (ar = 0; ar < 6; ar++) {
      iv4[ar] = (signed char)(ar + (signed char)ext_pose_idx);
    }

    b_j1 = H_ext_pose->size[1];
    for (ar = 0; ar < b_j1; ar++) {
      for (br = 0; br < 6; br++) {
        H->data[iv4[br] + H->size[0] * ar] = H_ext_pose->data[br +
          H_ext_pose->size[0] * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[(br + ext_pose_idx) + R_size[0] * (ar + ext_pose_idx)] =
          noiseParameters_ext_pos_noise * (double)b[br + 3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[(br + ext_pose_idx) + R_size[0] * ((ar + ext_pose_idx) + 3)] =
          0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[((br + ext_pose_idx) + R_size[0] * (ar + ext_pose_idx)) + 3] =
          0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[((br + ext_pose_idx) + R_size[0] * ((ar + ext_pose_idx) + 3)) + 3]
          = noiseParameters_ext_att_noise * (double)b[br + 3 * ar];
      }
    }
  }

  emxFree_real_T(&H_ext_pose);
}

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
//
//  INPUT ARGUMENTS:
//  - xt:                   The current state
//  - errorStateSize:       The size of the error state
//  - z_all_l:              The feature points in the left camera (2N x 1)
//  - map:                  Map of the estimated feature points (3 x N)
//  - indMeas:              The indices of the valid feature points
//  - cameraparams:         A struct with the fields
//     - focal length, center of projection, radial distortion parameters of
//     both cameras
//     - r_lr:              Translation of right camera in left camera frame
//     - R_lr:              Orientation of right camera in left camera frame
//
//  where N is number of points in the image
//
//  OUTPUT ARGUMENTS:
//  - r:    The residual (residualDim*M x 1)
//  - H_xc: The Jacobian of the measurement function with respect to the camera states (residualDim*M x length(xt))
//
//  where M =  length(indMeas) is the number of valid points in the image
// Arguments    : const emxArray_real_T *b_xt
//                double errorStateSize
//                double stateSize
//                const double z_all_l[32]
//                double indMeas
//                const emxArray_real_T *map
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double c_noiseParameters_orientation_n
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                const VIOMeasurements *measurements
//                double b_height_offset_pressure
//                const VIOParameters b_VIOParameters
//                double r_data[]
//                int r_size[1]
//                emxArray_real_T *H
//                double h_u[2]
//                double R_data[]
//                int R_size[2]
// Return Type  : void
//
void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize, double
                stateSize, const double z_all_l[32], double indMeas, const
                emxArray_real_T *map, const emxArray_real_T *anchorIdx, const
                emxArray_real_T *featureAnchorIdx, const emxArray_real_T
                *b_m_vect, const double noiseParameters_image_noise[2], double
                c_noiseParameters_orientation_n, double
                noiseParameters_pressure_noise, double
                noiseParameters_ext_pos_noise, double
                noiseParameters_ext_att_noise, const VIOMeasurements
                *measurements, double b_height_offset_pressure, const
                VIOParameters b_VIOParameters, double r_data[], int r_size[1],
                emxArray_real_T *H, double h_u[2], double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ib;
  int nm1d2;
  emxArray_real_T *H_xc;
  boolean_T bv0[3];
  double c_xt;
  double b_map[3];
  double h_ci_l[3];
  int ar;
  double z[2];
  emxArray_int32_T *r0;
  double h_u_To_h_ci_l[6];
  double d_xt[36];
  double b_h_u_To_h_ci_l[24];
  int n;
  double b_stateSize;
  double c_stateSize;
  double d_stateSize;
  double e_stateSize;
  double f_stateSize;
  double g_stateSize;
  double h_stateSize;
  double i_stateSize;
  double j_stateSize;
  double k_stateSize;
  double l_stateSize;
  double m_stateSize;
  double n_stateSize;
  double o_stateSize;
  double p_stateSize;
  double q_stateSize;
  double r_stateSize;
  double s_stateSize;
  double t_stateSize;
  double u_stateSize;
  double v_stateSize;
  double w_stateSize;
  double x_stateSize;
  double y_stateSize;
  double ab_stateSize;
  double bb_stateSize;
  double cb_stateSize;
  double db_stateSize;
  double eb_stateSize;
  double fb_stateSize;
  double gb_stateSize;
  double hb_stateSize;
  double ib_stateSize;
  double jb_stateSize;
  double kb_stateSize;
  double lb_stateSize;
  double mb_stateSize;
  double nb_stateSize;
  double ob_stateSize;
  double pb_stateSize;
  double qb_stateSize;
  double rb_stateSize;
  double sb_stateSize;
  double tb_stateSize;
  double ub_stateSize;
  double vb_stateSize;
  double wb_stateSize;
  double xb_stateSize;
  double e_xt[9];
  double y[3];
  double f_xt;
  double dv0[9];
  int br;
  emxArray_real_T *H_iy;
  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  emxArray_real_T *H_pressure;
  emxArray_real_T *C;
  double v[2];
  int ic;
  int ia;
  double r_pressure;
  double r_ext_pose[6];
  emxArray_real_T *H_ext_pose;
  double d[4];
  double R_vision[4];
  int residual_size;
  int orientation_idx;
  int pressure_idx;
  int ext_pose_idx;
  double b_measurements[9];
  signed char I[9];
  double c_measurements[9];
  signed char iv0[3];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  signed char iv1[6];
  emxInit_real_T(&H_xm, 2);

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4]) -
             b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] * b_xt->data[6]);
  R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] * b_xt->data[6]);
  R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] * b_xt->data[6]);
  R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data[4])
             - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] * b_xt->data[6]);
  R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] * b_xt->data[6]);
  R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] * b_xt->data[6]);
  R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data[4])
             + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];

  //  camera parameters for the left and right camera
  //  Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1);
  //  Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2);
  ib = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = 2;
  H_xm->size[1] = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ib, (int)sizeof(double));
  nm1d2 = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor)) << 1;
  for (ib = 0; ib < nm1d2; ib++) {
    H_xm->data[ib] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ib = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = 2;
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ib, (int)sizeof(double));
  nm1d2 = (int)errorStateSize << 1;
  for (ib = 0; ib < nm1d2; ib++) {
    H_xc->data[ib] = 0.0;
  }

  for (ib = 0; ib < 3; ib++) {
    bv0[ib] = rtIsNaN(map->data[ib + map->size[0] * ((int)indMeas - 1)]);
  }

  if (c_any(bv0)) {
    b_ros_error();
  }

  c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + b_VIOParameters.num_points_per_anchor)) + 7.0) +
    featureAnchorIdx->data[(int)indMeas - 1]) - 1];
  for (ib = 0; ib < 3; ib++) {
    b_map[ib] = map->data[ib + map->size[0] * ((int)indMeas - 1)] - b_xt->
      data[ib];
  }

  for (ib = 0; ib < 3; ib++) {
    h_ci_l[ib] = 0.0;
    for (ar = 0; ar < 3; ar++) {
      h_ci_l[ib] += c_xt * R_cw[ib + 3 * ar] * b_map[ar];
    }
  }

  //  = R_cw*(rho*anchorPos + anchorRot'*m - rho*r_wc_pred)
  predictMeasurement_left(h_ci_l, h_u);
  c_xt = (indMeas - 1.0) * 2.0;
  for (ib = 0; ib < 2; ib++) {
    z[ib] = z_all_l[(int)(c_xt + (1.0 + (double)ib)) - 1];
  }

  emxInit_int32_T(&r0, 1);

  //     %% computation of H(x)
  h_u_To_h_ci_l[0] = 1.0 / h_ci_l[2];
  h_u_To_h_ci_l[2] = 0.0;
  h_u_To_h_ci_l[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  h_u_To_h_ci_l[1] = 0.0;
  h_u_To_h_ci_l[3] = 1.0 / h_ci_l[2];
  h_u_To_h_ci_l[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  nm1d2 = (int)errorStateSize;
  ib = r0->size[0];
  r0->size[0] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)r0, ib, (int)sizeof(int));
  for (ib = 0; ib < nm1d2; ib++) {
    r0->data[ib] = ib;
  }

  c_xt = -b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] -
    1.0) * (7.0 + b_VIOParameters.num_points_per_anchor)) + 7.0) +
    featureAnchorIdx->data[(int)indMeas - 1]) - 1];
  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      d_xt[ar + 3 * ib] = c_xt * R_cw[ar + 3 * ib];
    }
  }

  d_xt[9] = 0.0;
  d_xt[12] = -h_ci_l[2];
  d_xt[15] = h_ci_l[1];
  d_xt[10] = h_ci_l[2];
  d_xt[13] = 0.0;
  d_xt[16] = -h_ci_l[0];
  d_xt[11] = -h_ci_l[1];
  d_xt[14] = h_ci_l[0];
  d_xt[17] = 0.0;
  for (ib = 0; ib < 6; ib++) {
    for (ar = 0; ar < 3; ar++) {
      d_xt[ar + 3 * (ib + 6)] = 0.0;
    }
  }

  for (ib = 0; ib < 2; ib++) {
    for (ar = 0; ar < 12; ar++) {
      b_h_u_To_h_ci_l[ib + (ar << 1)] = 0.0;
      for (n = 0; n < 3; n++) {
        b_h_u_To_h_ci_l[ib + (ar << 1)] += h_u_To_h_ci_l[ib + (n << 1)] * d_xt[n
          + 3 * ar];
      }
    }
  }

  nm1d2 = r0->size[0];
  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_xc->data[ar + H_xc->size[0] * r0->data[ib]] = b_h_u_To_h_ci_l[ar + (ib <<
        1)];
    }
  }

  //     %% anchor state derivatives
  //  fp = anchorPos + anchorRot'*m/rho - r_wc_pred;
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  for (ib = 0; ib < 3; ib++) {
    h_ci_l[ib] = b_xt->data[(int)(b_stateSize + (1.0 + (double)ib)) - 1];
  }

  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  c_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  d_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  e_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  f_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  g_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  h_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  i_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  j_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  k_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  l_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  m_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  n_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  o_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  p_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  q_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  r_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  s_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  t_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  u_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  v_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  w_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  x_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  y_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  ab_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  bb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  cb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  db_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  eb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  fb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  gb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  hb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  ib_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  jb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  kb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  lb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  mb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  nb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  ob_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  pb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  qb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  rb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  sb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  tb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  ub_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  vb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  wb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  xb_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    b_VIOParameters.num_points_per_anchor);
  e_xt[0] = ((b_xt->data[(int)(b_stateSize + 4.0) - 1] * b_xt->data[(int)
              (c_stateSize + 4.0) - 1] - b_xt->data[(int)(d_stateSize + 5.0) - 1]
              * b_xt->data[(int)(e_stateSize + 5.0) - 1]) - b_xt->data[(int)
             (f_stateSize + 6.0) - 1] * b_xt->data[(int)(g_stateSize + 6.0) - 1])
    + b_xt->data[(int)(h_stateSize + 7.0) - 1] * b_xt->data[(int)(i_stateSize +
    7.0) - 1];
  e_xt[1] = 2.0 * (b_xt->data[(int)(j_stateSize + 4.0) - 1] * b_xt->data[(int)
                   (k_stateSize + 5.0) - 1] + b_xt->data[(int)(l_stateSize + 6.0)
                   - 1] * b_xt->data[(int)(m_stateSize + 7.0) - 1]);
  e_xt[2] = 2.0 * (b_xt->data[(int)(n_stateSize + 4.0) - 1] * b_xt->data[(int)
                   (o_stateSize + 6.0) - 1] - b_xt->data[(int)(p_stateSize + 5.0)
                   - 1] * b_xt->data[(int)(q_stateSize + 7.0) - 1]);
  e_xt[3] = 2.0 * (b_xt->data[(int)(r_stateSize + 4.0) - 1] * b_xt->data[(int)
                   (s_stateSize + 5.0) - 1] - b_xt->data[(int)(t_stateSize + 6.0)
                   - 1] * b_xt->data[(int)(u_stateSize + 7.0) - 1]);
  e_xt[4] = ((-(b_xt->data[(int)(v_stateSize + 4.0) - 1] * b_xt->data[(int)
                (w_stateSize + 4.0) - 1]) + b_xt->data[(int)(x_stateSize + 5.0)
              - 1] * b_xt->data[(int)(y_stateSize + 5.0) - 1]) - b_xt->data[(int)
             (ab_stateSize + 6.0) - 1] * b_xt->data[(int)(bb_stateSize + 6.0) -
             1]) + b_xt->data[(int)(cb_stateSize + 7.0) - 1] * b_xt->data[(int)
    (db_stateSize + 7.0) - 1];
  e_xt[5] = 2.0 * (b_xt->data[(int)(eb_stateSize + 5.0) - 1] * b_xt->data[(int)
                   (fb_stateSize + 6.0) - 1] + b_xt->data[(int)(gb_stateSize +
    4.0) - 1] * b_xt->data[(int)(hb_stateSize + 7.0) - 1]);
  e_xt[6] = 2.0 * (b_xt->data[(int)(ib_stateSize + 4.0) - 1] * b_xt->data[(int)
                   (jb_stateSize + 6.0) - 1] + b_xt->data[(int)(kb_stateSize +
    5.0) - 1] * b_xt->data[(int)(lb_stateSize + 7.0) - 1]);
  e_xt[7] = 2.0 * (b_xt->data[(int)(mb_stateSize + 5.0) - 1] * b_xt->data[(int)
                   (nb_stateSize + 6.0) - 1] - b_xt->data[(int)(ob_stateSize +
    4.0) - 1] * b_xt->data[(int)(pb_stateSize + 7.0) - 1]);
  e_xt[8] = ((-(b_xt->data[(int)(qb_stateSize + 4.0) - 1] * b_xt->data[(int)
                (rb_stateSize + 4.0) - 1]) - b_xt->data[(int)(sb_stateSize + 5.0)
              - 1] * b_xt->data[(int)(tb_stateSize + 5.0) - 1]) + b_xt->data
             [(int)(ub_stateSize + 6.0) - 1] * b_xt->data[(int)(vb_stateSize +
              6.0) - 1]) + b_xt->data[(int)(wb_stateSize + 7.0) - 1] *
    b_xt->data[(int)(xb_stateSize + 7.0) - 1];
  for (ib = 0; ib < 3; ib++) {
    y[ib] = 0.0;
    for (ar = 0; ar < 3; ar++) {
      c_xt = y[ib] + e_xt[ib + 3 * ar] * b_m_vect->data[ar + b_m_vect->size[0] *
        ((int)indMeas - 1)];
      y[ib] = c_xt;
    }
  }

  c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + b_VIOParameters.num_points_per_anchor)) + 7.0) +
    featureAnchorIdx->data[(int)indMeas - 1]) - 1];
  f_xt = -b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] -
    1.0) * (7.0 + b_VIOParameters.num_points_per_anchor)) + 7.0) +
    featureAnchorIdx->data[(int)indMeas - 1]) - 1];
  dv0[0] = 0.0;
  dv0[3] = -y[2];
  dv0[6] = y[1];
  dv0[1] = y[2];
  dv0[4] = 0.0;
  dv0[7] = -y[0];
  dv0[2] = -y[1];
  dv0[5] = y[0];
  dv0[8] = 0.0;
  nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas - 1] - 1.0);
  for (ib = 0; ib < 3; ib++) {
    y[ib] = h_ci_l[ib] - b_xt->data[ib];
  }

  for (ib = 0; ib < 3; ib++) {
    b_map[ib] = 0.0;
    for (ar = 0; ar < 3; ar++) {
      b_map[ib] += R_cw[ib + 3 * ar] * y[ar];
    }
  }

  br = (int)(b_VIOParameters.num_points_per_anchor - featureAnchorIdx->data[(int)
             indMeas - 1]);
  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      e_xt[ib + 3 * ar] = 0.0;
      for (n = 0; n < 3; n++) {
        e_xt[ib + 3 * ar] += f_xt * R_cw[ib + 3 * n] * dv0[n + 3 * ar];
      }
    }
  }

  emxInit_real_T(&H_iy, 2);
  ib = H_iy->size[0] * H_iy->size[1];
  H_iy->size[0] = 3;
  H_iy->size[1] = (nm1d2 + br) + 7;
  emxEnsureCapacity((emxArray__common *)H_iy, ib, (int)sizeof(double));
  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * ib] = c_xt * R_cw[ar + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * (ib + 3)] = e_xt[ar + 3 * ib];
    }
  }

  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * (ib + 6)] = 0.0;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    H_iy->data[ib + H_iy->size[0] * (6 + nm1d2)] = b_map[ib];
  }

  for (ib = 0; ib < br; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * ((ib + nm1d2) + 7)] = 0.0;
    }
  }

  if (rtIsNaN(6.0 + b_VIOParameters.num_points_per_anchor)) {
    n = 0;
    anew = rtNaN;
    apnd = 6.0 + b_VIOParameters.num_points_per_anchor;
  } else if (6.0 + b_VIOParameters.num_points_per_anchor < 1.0) {
    n = -1;
    anew = 1.0;
    apnd = 6.0 + b_VIOParameters.num_points_per_anchor;
  } else if (rtIsInf(6.0 + b_VIOParameters.num_points_per_anchor)) {
    n = 0;
    anew = rtNaN;
    apnd = 6.0 + b_VIOParameters.num_points_per_anchor;
  } else {
    anew = 1.0;
    ndbl = floor(((6.0 + b_VIOParameters.num_points_per_anchor) - 1.0) + 0.5);
    apnd = 1.0 + ndbl;
    cdiff = (1.0 + ndbl) - (6.0 + b_VIOParameters.num_points_per_anchor);
    absb = fabs(6.0 + b_VIOParameters.num_points_per_anchor);
    if ((1.0 >= absb) || rtIsNaN(absb)) {
      absb = 1.0;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = 6.0 + b_VIOParameters.num_points_per_anchor;
    } else if (cdiff > 0.0) {
      apnd = 1.0 + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl - 1;
    } else {
      n = -1;
    }
  }

  emxInit_real_T(&H_pressure, 2);
  ib = H_pressure->size[0] * H_pressure->size[1];
  H_pressure->size[0] = 1;
  H_pressure->size[1] = n + 1;
  emxEnsureCapacity((emxArray__common *)H_pressure, ib, (int)sizeof(double));
  if (n + 1 > 0) {
    H_pressure->data[0] = anew;
    if (n + 1 > 1) {
      H_pressure->data[n] = apnd;
      nm1d2 = (n + (n < 0)) >> 1;
      for (br = 1; br < nm1d2; br++) {
        H_pressure->data[br] = anew + (double)br;
        H_pressure->data[n - br] = apnd - (double)br;
      }

      if (nm1d2 << 1 == n) {
        H_pressure->data[nm1d2] = (anew + apnd) / 2.0;
      } else {
        H_pressure->data[nm1d2] = anew + (double)nm1d2;
        H_pressure->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  c_xt = (anchorIdx->data[(int)indMeas - 1] - 1.0) * (6.0 +
    b_VIOParameters.num_points_per_anchor);
  ib = r0->size[0];
  r0->size[0] = H_pressure->size[1];
  emxEnsureCapacity((emxArray__common *)r0, ib, (int)sizeof(int));
  nm1d2 = H_pressure->size[1];
  for (ib = 0; ib < nm1d2; ib++) {
    r0->data[ib] = (int)(c_xt + H_pressure->data[H_pressure->size[0] * ib]) - 1;
  }

  emxInit_real_T(&C, 2);
  v[1] = H_iy->size[1];
  ib = C->size[0] * C->size[1];
  C->size[0] = 2;
  emxEnsureCapacity((emxArray__common *)C, ib, (int)sizeof(double));
  ib = C->size[0] * C->size[1];
  C->size[1] = (int)v[1];
  emxEnsureCapacity((emxArray__common *)C, ib, (int)sizeof(double));
  nm1d2 = (int)v[1] << 1;
  for (ib = 0; ib < nm1d2; ib++) {
    C->data[ib] = 0.0;
  }

  nm1d2 = (H_iy->size[1] - 1) << 1;
  for (n = 0; n <= nm1d2; n += 2) {
    for (ic = n; ic + 1 <= n + 2; ic++) {
      C->data[ic] = 0.0;
    }
  }

  br = 0;
  for (n = 0; n <= nm1d2; n += 2) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (H_iy->data[ib] != 0.0) {
        ia = ar;
        for (ic = n; ic + 1 <= n + 2; ic++) {
          ia++;
          C->data[ic] += H_iy->data[ib] * h_u_To_h_ci_l[ia - 1];
        }
      }

      ar += 2;
    }

    br += 3;
  }

  nm1d2 = C->size[1];
  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_xm->data[ar + H_xm->size[0] * r0->data[ib]] = C->data[ar + C->size[0] *
        ib];
    }
  }

  emxFree_real_T(&C);
  emxFree_int32_T(&r0);
  for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
    h_ci_l[nm1d2] = 0.0;
  }

  r_pressure = 0.0;
  for (nm1d2 = 0; nm1d2 < 6; nm1d2++) {
    r_ext_pose[nm1d2] = 0.0;
  }

  ib = H_iy->size[0] * H_iy->size[1];
  H_iy->size[0] = 3;
  H_iy->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_iy, ib, (int)sizeof(double));
  nm1d2 = 3 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ib = 0; ib < nm1d2; ib++) {
    H_iy->data[ib] = 0.0;
  }

  ib = H_pressure->size[0] * H_pressure->size[1];
  H_pressure->size[0] = 1;
  H_pressure->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_pressure, ib, (int)sizeof(double));
  nm1d2 = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ib = 0; ib < nm1d2; ib++) {
    H_pressure->data[ib] = 0.0;
  }

  emxInit_real_T(&H_ext_pose, 2);
  ib = H_ext_pose->size[0] * H_ext_pose->size[1];
  H_ext_pose->size[0] = 6;
  H_ext_pose->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_ext_pose, ib, (int)sizeof(double));
  nm1d2 = 6 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ib = 0; ib < nm1d2; ib++) {
    H_ext_pose->data[ib] = 0.0;
  }

  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (ib = 0; ib < 4; ib++) {
    d[ib] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = v[nm1d2];
  }

  nm1d2 = -1;
  for (n = 0; n < 2; n++) {
    for (br = 0; br < 2; br++) {
      nm1d2++;
      R_vision[nm1d2] = d[br + (n << 1)];
    }
  }

  residual_size = 2;
  orientation_idx = 0;
  pressure_idx = 0;
  ext_pose_idx = 0;
  if (b_VIOParameters.use_orientation) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_measurements[0] = ((measurements->att_fmu[0] * measurements->att_fmu[0] -
                          measurements->att_fmu[1] * measurements->att_fmu[1]) -
                         measurements->att_fmu[2] * measurements->att_fmu[2]) +
      measurements->att_fmu[3] * measurements->att_fmu[3];
    b_measurements[3] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] + measurements->att_fmu[2] * measurements->att_fmu[3]);
    b_measurements[6] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] - measurements->att_fmu[1] * measurements->att_fmu[3]);
    b_measurements[1] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [1] - measurements->att_fmu[2] * measurements->att_fmu[3]);
    b_measurements[4] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          + measurements->att_fmu[1] * measurements->att_fmu[1])
                         - measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    b_measurements[7] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] + measurements->att_fmu[0] * measurements->att_fmu[3]);
    b_measurements[2] = 2.0 * (measurements->att_fmu[0] * measurements->att_fmu
      [2] + measurements->att_fmu[1] * measurements->att_fmu[3]);
    b_measurements[5] = 2.0 * (measurements->att_fmu[1] * measurements->att_fmu
      [2] - measurements->att_fmu[0] * measurements->att_fmu[3]);
    b_measurements[8] = ((-(measurements->att_fmu[0] * measurements->att_fmu[0])
                          - measurements->att_fmu[1] * measurements->att_fmu[1])
                         + measurements->att_fmu[2] * measurements->att_fmu[2])
      + measurements->att_fmu[3] * measurements->att_fmu[3];
    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        e_xt[ib + 3 * ar] = 0.0;
        for (n = 0; n < 3; n++) {
          e_xt[ib + 3 * ar] += R_bc[n + 3 * ib] * b_measurements[n + 3 * ar];
        }
      }

      for (ar = 0; ar < 3; ar++) {
        dv0[ib + 3 * ar] = 0.0;
        for (n = 0; n < 3; n++) {
          dv0[ib + 3 * ar] += e_xt[ib + 3 * n] * R_cw[ar + 3 * n];
        }
      }
    }

    QuatFromRotJ(dv0, d);
    for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
      h_ci_l[nm1d2] = d[nm1d2];
    }

    for (ib = 0; ib < 9; ib++) {
      I[ib] = 0;
    }

    for (br = 0; br < 3; br++) {
      I[br + 3 * br] = 1;
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_iy->data[ar + H_iy->size[0] * (3 + ib)] = I[ar + 3 * ib];
      }
    }

    orientation_idx = 2;
    residual_size = 5;
  }

  if (b_VIOParameters.use_pressure) {
    r_pressure = (1.0 - rt_powd_snf(measurements->bar_fmu / 101325.0, 0.190284))
      * 145366.45 - b_height_offset_pressure;
    H_pressure->data[2] = 1.0;
    pressure_idx = residual_size;
    residual_size++;
  }

  if (b_VIOParameters.use_ext_pose) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    c_measurements[0] = ((measurements->att_ext[0] * measurements->att_ext[0] -
                          measurements->att_ext[1] * measurements->att_ext[1]) -
                         measurements->att_ext[2] * measurements->att_ext[2]) +
      measurements->att_ext[3] * measurements->att_ext[3];
    c_measurements[3] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [1] + measurements->att_ext[2] * measurements->att_ext[3]);
    c_measurements[6] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [2] - measurements->att_ext[1] * measurements->att_ext[3]);
    c_measurements[1] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [1] - measurements->att_ext[2] * measurements->att_ext[3]);
    c_measurements[4] = ((-(measurements->att_ext[0] * measurements->att_ext[0])
                          + measurements->att_ext[1] * measurements->att_ext[1])
                         - measurements->att_ext[2] * measurements->att_ext[2])
      + measurements->att_ext[3] * measurements->att_ext[3];
    c_measurements[7] = 2.0 * (measurements->att_ext[1] * measurements->att_ext
      [2] + measurements->att_ext[0] * measurements->att_ext[3]);
    c_measurements[2] = 2.0 * (measurements->att_ext[0] * measurements->att_ext
      [2] + measurements->att_ext[1] * measurements->att_ext[3]);
    c_measurements[5] = 2.0 * (measurements->att_ext[1] * measurements->att_ext
      [2] - measurements->att_ext[0] * measurements->att_ext[3]);
    c_measurements[8] = ((-(measurements->att_ext[0] * measurements->att_ext[0])
                          - measurements->att_ext[1] * measurements->att_ext[1])
                         + measurements->att_ext[2] * measurements->att_ext[2])
      + measurements->att_ext[3] * measurements->att_ext[3];
    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        b_measurements[ib + 3 * ar] = 0.0;
        for (n = 0; n < 3; n++) {
          b_measurements[ib + 3 * ar] += c_measurements[ib + 3 * n] * R_cw[ar +
            3 * n];
        }
      }
    }

    QuatFromRotJ(b_measurements, d);
    for (ib = 0; ib < 3; ib++) {
      r_ext_pose[ib] = measurements->pos_ext[ib] - b_xt->data[ib];
    }

    for (ib = 0; ib < 3; ib++) {
      r_ext_pose[ib + 3] = d[ib];
    }

    for (ib = 0; ib < 9; ib++) {
      I[ib] = 0;
    }

    for (br = 0; br < 3; br++) {
      I[br + 3 * br] = 1;
    }

    memset(&R_cw[0], 0, 9U * sizeof(double));
    for (br = 0; br < 3; br++) {
      R_cw[br + 3 * br] = 1.0;
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_ext_pose->data[ar + H_ext_pose->size[0] * ib] = I[ar + 3 * ib];
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_ext_pose->data[ar + H_ext_pose->size[0] * (ib + 3)] = 0.0;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_ext_pose->data[(ar + H_ext_pose->size[0] * ib) + 3] = 0.0;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_ext_pose->data[(ar + H_ext_pose->size[0] * (ib + 3)) + 3] = R_cw[ar +
          3 * ib];
      }
    }

    ext_pose_idx = residual_size;
    residual_size += 6;
  }

  r_size[0] = residual_size;
  for (ib = 0; ib < residual_size; ib++) {
    r_data[ib] = 0.0;
  }

  ib = H->size[0] * H->size[1];
  H->size[0] = residual_size;
  H->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
  nm1d2 = residual_size * (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  for (ib = 0; ib < nm1d2; ib++) {
    H->data[ib] = 0.0;
  }

  R_size[0] = residual_size;
  R_size[1] = residual_size;
  nm1d2 = residual_size * residual_size;
  for (ib = 0; ib < nm1d2; ib++) {
    R_data[ib] = 0.0;
  }

  for (ib = 0; ib < 2; ib++) {
    r_data[ib] = z[ib] - h_u[ib];
  }

  nm1d2 = H_xc->size[1];
  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H->data[ar + H->size[0] * ib] = H_xc->data[ar + H_xc->size[0] * ib];
    }
  }

  nm1d2 = H_xm->size[1];
  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H->data[ar + H->size[0] * (ib + H_xc->size[1])] = H_xm->data[ar +
        H_xm->size[0] * ib];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  for (ib = 0; ib < 2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      R_data[ar + R_size[0] * ib] = R_vision[ar + (ib << 1)];
    }
  }

  if (b_VIOParameters.use_orientation) {
    for (ib = 0; ib < 3; ib++) {
      r_data[ib + orientation_idx] = h_ci_l[ib];
    }

    for (ib = 0; ib < 3; ib++) {
      iv0[ib] = (signed char)(ib + orientation_idx);
    }

    nm1d2 = H_iy->size[1];
    for (ib = 0; ib < nm1d2; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H->data[iv0[ar] + H->size[0] * ib] = H_iy->data[ar + H_iy->size[0] * ib];
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        R_data[(ar + orientation_idx) + R_size[0] * (ib + orientation_idx)] =
          c_noiseParameters_orientation_n * (double)b[ar + 3 * ib];
      }
    }
  }

  emxFree_real_T(&H_iy);
  if (b_VIOParameters.use_pressure) {
    r_data[pressure_idx] = r_pressure;
    nm1d2 = H_pressure->size[1];
    for (ib = 0; ib < nm1d2; ib++) {
      H->data[pressure_idx + H->size[0] * ib] = H_pressure->data
        [H_pressure->size[0] * ib];
    }

    R_data[pressure_idx + R_size[0] * pressure_idx] =
      noiseParameters_pressure_noise;
  }

  emxFree_real_T(&H_pressure);
  if (b_VIOParameters.use_ext_pose) {
    for (ib = 0; ib < 6; ib++) {
      r_data[ib + ext_pose_idx] = r_ext_pose[ib];
    }

    for (ib = 0; ib < 6; ib++) {
      iv1[ib] = (signed char)(ib + ext_pose_idx);
    }

    nm1d2 = H_ext_pose->size[1];
    for (ib = 0; ib < nm1d2; ib++) {
      for (ar = 0; ar < 6; ar++) {
        H->data[iv1[ar] + H->size[0] * ib] = H_ext_pose->data[ar +
          H_ext_pose->size[0] * ib];
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        R_data[(ar + ext_pose_idx) + R_size[0] * (ib + ext_pose_idx)] =
          noiseParameters_ext_pos_noise * (double)b[ar + 3 * ib];
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        R_data[(ar + ext_pose_idx) + R_size[0] * ((ib + ext_pose_idx) + 3)] =
          0.0;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        R_data[((ar + ext_pose_idx) + R_size[0] * (ib + ext_pose_idx)) + 3] =
          0.0;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        R_data[((ar + ext_pose_idx) + R_size[0] * ((ib + ext_pose_idx) + 3)) + 3]
          = noiseParameters_ext_att_noise * (double)b[ar + 3 * ib];
      }
    }
  }

  emxFree_real_T(&H_ext_pose);
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
