//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 21:11:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_emxutil.h"
#include "fprintf.h"
#include "kron.h"
#include "eye.h"
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
//                const double z_all_l[64]
//                const double indMeas_data[]
//                const int indMeas_size[1]
//                const emxArray_real_T *map
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double c_VIOParameters_num_points_per_
//                double VIOParameters_num_anchors
//                double r_data[]
//                int r_size[1]
//                emxArray_real_T *H
//                double h_u_data[]
//                int h_u_size[1]
//                double R_data[]
//                int R_size[2]
// Return Type  : void
//
void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize, double
                stateSize, const double z_all_l[64], const double indMeas_data[],
                const int indMeas_size[1], const emxArray_real_T *map, const
                emxArray_real_T *anchorIdx, const emxArray_real_T
                *featureAnchorIdx, const emxArray_real_T *b_m_vect, const double
                noiseParameters_image_noise[2], double
                c_VIOParameters_num_points_per_, double
                VIOParameters_num_anchors, double r_data[], int r_size[1],
                emxArray_real_T *H, double h_u_data[], int h_u_size[1], double
                R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ib;
  int cr;
  emxArray_real_T *H_xc;
  int k;
  emxArray_real_T *H_iy;
  emxArray_int32_T *r1;
  emxArray_real_T *C;
  emxArray_real_T *y;
  boolean_T b[3];
  boolean_T b_y;
  boolean_T exitg1;
  char cv4[25];
  static const char cv5[25] = { 'm', 'a', 'p', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', ' ', 'i', 's', ' ', 'n', 'o', 't', ' ', 'v', 'a', 'l', 'i', 'd', '\x00'
  };

  double anew;
  double b_map[3];
  double h_ci_l[3];
  int ar;
  double h_u_To_h_ci_l[6];
  signed char b_k;
  signed char iv0[2];
  double c_xt[36];
  double b_h_u_To_h_ci_l[24];
  int br;
  int nm1d2;
  double ndbl;
  double apnd;
  double cdiff;
  double absb;
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
  double d_xt[9];
  double c_y[3];
  double dv0[9];
  double b_R_cw[3];
  double e_xt[9];
  double v[2];
  int ic;
  int ia;
  double d[4];
  emxArray_real_T *r2;
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
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, ib, (int)sizeof(double));
  ib = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(VIOParameters_num_anchors * (6.0 +
    c_VIOParameters_num_points_per_));
  emxEnsureCapacity((emxArray__common *)H_xm, ib, (int)sizeof(double));
  cr = (indMeas_size[0] << 1) * (int)(VIOParameters_num_anchors * (6.0 +
    c_VIOParameters_num_points_per_));
  for (ib = 0; ib < cr; ib++) {
    H_xm->data[ib] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ib = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, ib, (int)sizeof(double));
  ib = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ib, (int)sizeof(double));
  cr = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (ib = 0; ib < cr; ib++) {
    H_xc->data[ib] = 0.0;
  }

  r_size[0] = indMeas_size[0] << 1;
  cr = indMeas_size[0] << 1;
  for (ib = 0; ib < cr; ib++) {
    r_data[ib] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  cr = indMeas_size[0] << 1;
  for (ib = 0; ib < cr; ib++) {
    h_u_data[ib] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_iy, 2);
  emxInit_int32_T(&r1, 1);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&y, 2);
  while (k <= indMeas_size[0] - 1) {
    for (ib = 0; ib < 3; ib++) {
      b[ib] = rtIsNaN(map->data[ib + map->size[0] * ((int)indMeas_data[k] - 1)]);
    }

    b_y = false;
    cr = 0;
    exitg1 = false;
    while ((!exitg1) && (cr < 3)) {
      if (!!b[cr]) {
        b_y = true;
        exitg1 = true;
      } else {
        cr++;
      }
    }

    if (b_y) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (ib = 0; ib < 25; ib++) {
        cv4[ib] = cv5[ib];
      }

      ROS_ERROR(cv4);
    }

    anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + c_VIOParameters_num_points_per_)) + 7.0) +
      featureAnchorIdx->data[(int)indMeas_data[k] - 1]) - 1];
    for (ib = 0; ib < 3; ib++) {
      b_map[ib] = map->data[ib + map->size[0] * ((int)indMeas_data[k] - 1)] -
        b_xt->data[ib];
    }

    for (ib = 0; ib < 3; ib++) {
      h_ci_l[ib] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        h_ci_l[ib] += anew * R_cw[ib + 3 * ar] * b_map[ar];
      }
    }

    //  = R_cw*(rho*anchorPos + anchorRot'*m - rho*r_wc_pred)
    // PREDICTMEASUREMENT Predict the measurement of a feature given in the left 
    // camera frame
    //    Get the normalized pixel coordinates where a feature given in the left camera 
    //    frame
    //  normalized feature in camera frame
    //  undistorted measurement in normalized pixels
    cr = k << 1;
    for (ib = 0; ib < 2; ib++) {
      h_u_data[ib + cr] = h_ci_l[ib] / h_ci_l[2];
    }

    cr = k << 1;
    anew = (indMeas_data[k] - 1.0) * 2.0;
    for (ib = 0; ib < 2; ib++) {
      r_data[ib + cr] = z_all_l[(int)(anew + (1.0 + (double)ib)) - 1];
    }

    //     %% computation of H(x)
    h_u_To_h_ci_l[0] = 1.0 / h_ci_l[2];
    h_u_To_h_ci_l[2] = 0.0;
    h_u_To_h_ci_l[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
    h_u_To_h_ci_l[1] = 0.0;
    h_u_To_h_ci_l[3] = 1.0 / h_ci_l[2];
    h_u_To_h_ci_l[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
    b_k = (signed char)((signed char)k << 1);
    for (ib = 0; ib < 2; ib++) {
      iv0[ib] = (signed char)(ib + b_k);
    }

    cr = H_xc->size[1];
    ib = r1->size[0];
    r1->size[0] = cr;
    emxEnsureCapacity((emxArray__common *)r1, ib, (int)sizeof(int));
    for (ib = 0; ib < cr; ib++) {
      r1->data[ib] = ib;
    }

    anew = -b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + c_VIOParameters_num_points_per_)) + 7.0) +
      featureAnchorIdx->data[(int)indMeas_data[k] - 1]) - 1];
    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        c_xt[ar + 3 * ib] = anew * R_cw[ar + 3 * ib];
      }
    }

    c_xt[9] = 0.0;
    c_xt[12] = -h_ci_l[2];
    c_xt[15] = h_ci_l[1];
    c_xt[10] = h_ci_l[2];
    c_xt[13] = 0.0;
    c_xt[16] = -h_ci_l[0];
    c_xt[11] = -h_ci_l[1];
    c_xt[14] = h_ci_l[0];
    c_xt[17] = 0.0;
    for (ib = 0; ib < 6; ib++) {
      for (ar = 0; ar < 3; ar++) {
        c_xt[ar + 3 * (ib + 6)] = 0.0;
      }
    }

    for (ib = 0; ib < 2; ib++) {
      for (ar = 0; ar < 12; ar++) {
        b_h_u_To_h_ci_l[ib + (ar << 1)] = 0.0;
        for (br = 0; br < 3; br++) {
          b_h_u_To_h_ci_l[ib + (ar << 1)] += h_u_To_h_ci_l[ib + (br << 1)] *
            c_xt[br + 3 * ar];
        }
      }
    }

    nm1d2 = r1->size[0];
    for (ib = 0; ib < nm1d2; ib++) {
      for (ar = 0; ar < 2; ar++) {
        H_xc->data[iv0[ar] + H_xc->size[0] * r1->data[ib]] = b_h_u_To_h_ci_l[ar
          + (ib << 1)];
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
    anew = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0
      + c_VIOParameters_num_points_per_);
    for (ib = 0; ib < 3; ib++) {
      h_ci_l[ib] = b_xt->data[(int)(anew + (1.0 + (double)ib)) - 1];
    }

    anew = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0
      + c_VIOParameters_num_points_per_);
    ndbl = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0
      + c_VIOParameters_num_points_per_);
    apnd = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0
      + c_VIOParameters_num_points_per_);
    cdiff = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0
      + c_VIOParameters_num_points_per_);
    absb = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0
      + c_VIOParameters_num_points_per_);
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    c_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    d_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    e_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    f_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    g_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    h_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    i_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    j_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    k_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    l_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    m_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    n_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    o_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    p_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    q_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    r_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    s_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    t_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    u_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    v_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    w_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    x_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    y_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + c_VIOParameters_num_points_per_);
    ab_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    bb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    cb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    db_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    eb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    fb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    gb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    hb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    ib_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    jb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    kb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    lb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    mb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    nb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    ob_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    pb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    qb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    rb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    sb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
      * (7.0 + c_VIOParameters_num_points_per_);
    d_xt[0] = ((b_xt->data[(int)(anew + 4.0) - 1] * b_xt->data[(int)(ndbl + 4.0)
                - 1] - b_xt->data[(int)(apnd + 5.0) - 1] * b_xt->data[(int)
                (cdiff + 5.0) - 1]) - b_xt->data[(int)(absb + 6.0) - 1] *
               b_xt->data[(int)(b_stateSize + 6.0) - 1]) + b_xt->data[(int)
      (c_stateSize + 7.0) - 1] * b_xt->data[(int)(d_stateSize + 7.0) - 1];
    d_xt[1] = 2.0 * (b_xt->data[(int)(e_stateSize + 4.0) - 1] * b_xt->data[(int)
                     (f_stateSize + 5.0) - 1] + b_xt->data[(int)(g_stateSize +
      6.0) - 1] * b_xt->data[(int)(h_stateSize + 7.0) - 1]);
    d_xt[2] = 2.0 * (b_xt->data[(int)(i_stateSize + 4.0) - 1] * b_xt->data[(int)
                     (j_stateSize + 6.0) - 1] - b_xt->data[(int)(k_stateSize +
      5.0) - 1] * b_xt->data[(int)(l_stateSize + 7.0) - 1]);
    d_xt[3] = 2.0 * (b_xt->data[(int)(m_stateSize + 4.0) - 1] * b_xt->data[(int)
                     (n_stateSize + 5.0) - 1] - b_xt->data[(int)(o_stateSize +
      6.0) - 1] * b_xt->data[(int)(p_stateSize + 7.0) - 1]);
    d_xt[4] = ((-(b_xt->data[(int)(q_stateSize + 4.0) - 1] * b_xt->data[(int)
                  (r_stateSize + 4.0) - 1]) + b_xt->data[(int)(s_stateSize + 5.0)
                - 1] * b_xt->data[(int)(t_stateSize + 5.0) - 1]) - b_xt->data
               [(int)(u_stateSize + 6.0) - 1] * b_xt->data[(int)(v_stateSize +
                6.0) - 1]) + b_xt->data[(int)(w_stateSize + 7.0) - 1] *
      b_xt->data[(int)(x_stateSize + 7.0) - 1];
    d_xt[5] = 2.0 * (b_xt->data[(int)(y_stateSize + 5.0) - 1] * b_xt->data[(int)
                     (ab_stateSize + 6.0) - 1] + b_xt->data[(int)(bb_stateSize +
      4.0) - 1] * b_xt->data[(int)(cb_stateSize + 7.0) - 1]);
    d_xt[6] = 2.0 * (b_xt->data[(int)(db_stateSize + 4.0) - 1] * b_xt->data[(int)
                     (eb_stateSize + 6.0) - 1] + b_xt->data[(int)(fb_stateSize +
      5.0) - 1] * b_xt->data[(int)(gb_stateSize + 7.0) - 1]);
    d_xt[7] = 2.0 * (b_xt->data[(int)(hb_stateSize + 5.0) - 1] * b_xt->data[(int)
                     (ib_stateSize + 6.0) - 1] - b_xt->data[(int)(jb_stateSize +
      4.0) - 1] * b_xt->data[(int)(kb_stateSize + 7.0) - 1]);
    d_xt[8] = ((-(b_xt->data[(int)(lb_stateSize + 4.0) - 1] * b_xt->data[(int)
                  (mb_stateSize + 4.0) - 1]) - b_xt->data[(int)(nb_stateSize +
      5.0) - 1] * b_xt->data[(int)(ob_stateSize + 5.0) - 1]) + b_xt->data[(int)
               (pb_stateSize + 6.0) - 1] * b_xt->data[(int)(qb_stateSize + 6.0)
               - 1]) + b_xt->data[(int)(rb_stateSize + 7.0) - 1] * b_xt->data
      [(int)(sb_stateSize + 7.0) - 1];
    for (ib = 0; ib < 3; ib++) {
      c_y[ib] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        anew = c_y[ib] + d_xt[ib + 3 * ar] * b_m_vect->data[ar + b_m_vect->size
          [0] * ((int)indMeas_data[k] - 1)];
        c_y[ib] = anew;
      }
    }

    anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + c_VIOParameters_num_points_per_)) + 7.0) +
      featureAnchorIdx->data[(int)indMeas_data[k] - 1]) - 1];
    ndbl = -b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + c_VIOParameters_num_points_per_)) + 7.0) +
      featureAnchorIdx->data[(int)indMeas_data[k] - 1]) - 1];
    dv0[0] = 0.0;
    dv0[3] = -c_y[2];
    dv0[6] = c_y[1];
    dv0[1] = c_y[2];
    dv0[4] = 0.0;
    dv0[7] = -c_y[0];
    dv0[2] = -c_y[1];
    dv0[5] = c_y[0];
    dv0[8] = 0.0;
    nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
    for (ib = 0; ib < 3; ib++) {
      b_map[ib] = h_ci_l[ib] - b_xt->data[ib];
    }

    for (ib = 0; ib < 3; ib++) {
      b_R_cw[ib] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        b_R_cw[ib] += R_cw[ib + 3 * ar] * b_map[ar];
      }
    }

    cr = (int)(c_VIOParameters_num_points_per_ - featureAnchorIdx->data[(int)
               indMeas_data[k] - 1]);
    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        e_xt[ib + 3 * ar] = 0.0;
        for (br = 0; br < 3; br++) {
          e_xt[ib + 3 * ar] += ndbl * R_cw[ib + 3 * br] * dv0[br + 3 * ar];
        }
      }
    }

    ib = H_iy->size[0] * H_iy->size[1];
    H_iy->size[0] = 3;
    H_iy->size[1] = (nm1d2 + cr) + 7;
    emxEnsureCapacity((emxArray__common *)H_iy, ib, (int)sizeof(double));
    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_iy->data[ar + H_iy->size[0] * ib] = anew * R_cw[ar + 3 * ib];
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
      H_iy->data[ib + H_iy->size[0] * (6 + nm1d2)] = b_R_cw[ib];
    }

    for (ib = 0; ib < cr; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_iy->data[ar + H_iy->size[0] * ((ib + nm1d2) + 7)] = 0.0;
      }
    }

    b_k = (signed char)((signed char)k << 1);
    for (ib = 0; ib < 2; ib++) {
      iv0[ib] = (signed char)(ib + b_k);
    }

    if (rtIsNaN(6.0 + c_VIOParameters_num_points_per_)) {
      br = 0;
      anew = rtNaN;
      apnd = 6.0 + c_VIOParameters_num_points_per_;
    } else if (6.0 + c_VIOParameters_num_points_per_ < 1.0) {
      br = -1;
      anew = 1.0;
      apnd = 6.0 + c_VIOParameters_num_points_per_;
    } else if (rtIsInf(6.0 + c_VIOParameters_num_points_per_)) {
      br = 0;
      anew = rtNaN;
      apnd = 6.0 + c_VIOParameters_num_points_per_;
    } else {
      anew = 1.0;
      ndbl = floor(((6.0 + c_VIOParameters_num_points_per_) - 1.0) + 0.5);
      apnd = 1.0 + ndbl;
      cdiff = (1.0 + ndbl) - (6.0 + c_VIOParameters_num_points_per_);
      absb = fabs(6.0 + c_VIOParameters_num_points_per_);
      if ((1.0 >= absb) || rtIsNaN(absb)) {
        absb = 1.0;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = 6.0 + c_VIOParameters_num_points_per_;
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

    ib = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = br + 1;
    emxEnsureCapacity((emxArray__common *)y, ib, (int)sizeof(double));
    if (br + 1 > 0) {
      y->data[0] = anew;
      if (br + 1 > 1) {
        y->data[br] = apnd;
        nm1d2 = (br + (br < 0)) >> 1;
        for (cr = 1; cr < nm1d2; cr++) {
          y->data[cr] = anew + (double)cr;
          y->data[br - cr] = apnd - (double)cr;
        }

        if (nm1d2 << 1 == br) {
          y->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          y->data[nm1d2] = anew + (double)nm1d2;
          y->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    anew = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
      c_VIOParameters_num_points_per_);
    ib = r1->size[0];
    r1->size[0] = y->size[1];
    emxEnsureCapacity((emxArray__common *)r1, ib, (int)sizeof(int));
    cr = y->size[1];
    for (ib = 0; ib < cr; ib++) {
      r1->data[ib] = (int)(anew + y->data[y->size[0] * ib]) - 1;
    }

    v[1] = H_iy->size[1];
    ib = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, ib, (int)sizeof(double));
    ib = C->size[0] * C->size[1];
    C->size[1] = (int)v[1];
    emxEnsureCapacity((emxArray__common *)C, ib, (int)sizeof(double));
    cr = (int)v[1] << 1;
    for (ib = 0; ib < cr; ib++) {
      C->data[ib] = 0.0;
    }

    nm1d2 = (H_iy->size[1] - 1) << 1;
    for (cr = 0; cr <= nm1d2; cr += 2) {
      for (ic = cr; ic + 1 <= cr + 2; ic++) {
        C->data[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= nm1d2; cr += 2) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 3; ib++) {
        if (H_iy->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 2; ic++) {
            ia++;
            C->data[ic] += H_iy->data[ib] * h_u_To_h_ci_l[ia - 1];
          }
        }

        ar += 2;
      }

      br += 3;
    }

    cr = C->size[1];
    for (ib = 0; ib < cr; ib++) {
      for (ar = 0; ar < 2; ar++) {
        H_xm->data[iv0[ar] + H_xm->size[0] * r1->data[ib]] = C->data[ar +
          C->size[0] * ib];
      }
    }

    k++;
  }

  emxFree_real_T(&y);
  emxFree_real_T(&C);
  emxFree_int32_T(&r1);
  emxFree_real_T(&H_iy);
  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (ib = 0; ib < 4; ib++) {
    d[ib] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = v[nm1d2];
  }

  emxInit_real_T(&r2, 2);
  b_eye((double)indMeas_size[0], r2);
  kron(r2->data, r2->size, d, R_data, R_size);
  b_fprintf();
  cr = r_size[0];
  emxFree_real_T(&r2);
  for (ib = 0; ib < cr; ib++) {
    r_data[ib] -= h_u_data[ib];
  }

  ib = H->size[0] * H->size[1];
  H->size[0] = H_xc->size[0];
  H->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
  cr = H_xc->size[1];
  for (ib = 0; ib < cr; ib++) {
    nm1d2 = H_xc->size[0];
    for (ar = 0; ar < nm1d2; ar++) {
      H->data[ar + H->size[0] * ib] = H_xc->data[ar + H_xc->size[0] * ib];
    }
  }

  cr = H_xm->size[1];
  for (ib = 0; ib < cr; ib++) {
    nm1d2 = H_xm->size[0];
    for (ar = 0; ar < nm1d2; ar++) {
      H->data[ar + H->size[0] * (ib + H_xc->size[1])] = H_xm->data[ar +
        H_xm->size[0] * ib];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
