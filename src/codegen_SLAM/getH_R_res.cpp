//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getH_R_res.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 22:40:16
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getH_R_res.h"
#include "SLAM_updIT.h"
#include "SLAM_emxutil.h"
#include "predictMeasurement_left.h"
#include "any.h"
#include "norm.h"
#include "QuatFromRotJ.h"
#include "kron.h"
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
//                const double z_all_l[48]
//                const double indMeas_data[]
//                const int indMeas_size[1]
//                const emxArray_real_T *map
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                double c_noiseParameters_gravity_align
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
void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize, double
                stateSize, const double z_all_l[48], const double indMeas_data[],
                const int indMeas_size[1], const emxArray_real_T *map, const
                emxArray_real_T *anchorIdx, const emxArray_real_T
                *featureAnchorIdx, const emxArray_real_T *b_m_vect, const double
                noiseParameters_image_noise[2], double
                noiseParameters_pressure_noise, double
                noiseParameters_ext_pos_noise, double
                noiseParameters_ext_att_noise, double
                c_noiseParameters_gravity_align, const VIOMeasurements
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
  int cr;
  emxArray_real_T *H_xc;
  double z_data[48];
  int k;
  emxArray_real_T *H_pressure;
  emxArray_real_T *H_grav;
  emxArray_real_T *C;
  emxArray_int32_T *r1;
  boolean_T bv0[3];
  char cv4[28];
  static const char cv5[28] = { 'm', 'a', 'p', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', ' ', '%', 'i', ' ', 'i', 's', ' ', 'n', 'o', 't', ' ', 'v', 'a', 'l',
    'i', 'd', '\x00' };

  double c_xt;
  double b_map[3];
  double r_grav[3];
  int br;
  double dv2[2];
  double indMeas;
  double h_u_To_h_ci_l[6];
  int nm1d2;
  signed char b_k;
  signed char iv0[2];
  double v[2];
  int ic;
  int ib;
  int ia;
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
  double b_measurements[9];
  double y[3];
  double b_y;
  double b_R_cw[9];
  double dv3[9];
  double c_R_cw[3];
  double d_R_cw[9];
  int i6;
  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double b_anchorIdx;
  double d[4];
  emxArray_real_T *r2;
  int R_vision_size[2];
  double R_vision_data[2304];
  double r_pressure;
  double r_ext_pose[6];
  emxArray_real_T *H_ext_pose;
  int residual_size;
  int pressure_idx;
  int ext_pose_idx;
  int grav_idx;
  double c_measurements[9];
  signed char I[9];
  signed char b_I[9];
  double B;
  static const signed char b[3] = { 0, 0, 1 };

  int tmp_data[48];
  signed char iv1[6];
  static const signed char b_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  signed char iv2[3];
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
  ar = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  ar = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ar, (int)sizeof(double));
  cr = (indMeas_size[0] << 1) * (int)(b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < cr; ar++) {
    H_xm->data[ar] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ar = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, ar, (int)sizeof(double));
  ar = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ar, (int)sizeof(double));
  cr = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (ar = 0; ar < cr; ar++) {
    H_xc->data[ar] = 0.0;
  }

  cr = indMeas_size[0] << 1;
  for (ar = 0; ar < cr; ar++) {
    z_data[ar] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  cr = indMeas_size[0] << 1;
  for (ar = 0; ar < cr; ar++) {
    h_u_data[ar] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_pressure, 2);
  emxInit_real_T(&H_grav, 2);
  emxInit_real_T(&C, 2);
  emxInit_int32_T(&r1, 1);
  while (k <= indMeas_size[0] - 1) {
    for (ar = 0; ar < 3; ar++) {
      bv0[ar] = rtIsNaN(map->data[ar + map->size[0] * ((int)indMeas_data[k] - 1)]);
    }

    if (c_any(bv0)) {
      // #coder
      // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
      for (ar = 0; ar < 28; ar++) {
        cv4[ar] = cv5[ar];
      }

      c_xt = rt_roundd_snf(indMeas_data[k]);
      if (c_xt < 2.147483648E+9) {
        if (c_xt >= -2.147483648E+9) {
          ar = (int)c_xt;
        } else {
          ar = MIN_int32_T;
        }
      } else if (c_xt >= 2.147483648E+9) {
        ar = MAX_int32_T;
      } else {
        ar = 0;
      }

      ROS_ERROR(cv4, ar);
    } else {
      c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
        - 1] - 1.0) * numStatesPerAnchor) + 7.0) + featureAnchorIdx->data[(int)
        indMeas_data[k] - 1]) - 1];
      for (ar = 0; ar < 3; ar++) {
        b_map[ar] = map->data[ar + map->size[0] * ((int)indMeas_data[k] - 1)] -
          b_xt->data[ar];
      }

      for (ar = 0; ar < 3; ar++) {
        r_grav[ar] = 0.0;
        for (br = 0; br < 3; br++) {
          r_grav[ar] += c_xt * R_cw[ar + 3 * br] * b_map[br];
        }
      }

      //  = R_cw*(rho*anchorPos + anchorRot'*m - rho*r_wc_pred)
      predictMeasurement_left(r_grav, dv2);
      cr = k << 1;
      for (ar = 0; ar < 2; ar++) {
        h_u_data[ar + cr] = dv2[ar];
      }

      cr = k << 1;
      indMeas = (indMeas_data[k] - 1.0) * 2.0;
      for (ar = 0; ar < 2; ar++) {
        z_data[ar + cr] = z_all_l[(int)(indMeas + (1.0 + (double)ar)) - 1];
      }

      //     %% computation of H(x)
      h_u_To_h_ci_l[0] = 1.0 / r_grav[2];
      h_u_To_h_ci_l[2] = 0.0;
      h_u_To_h_ci_l[4] = -r_grav[0] / (r_grav[2] * r_grav[2]);
      h_u_To_h_ci_l[1] = 0.0;
      h_u_To_h_ci_l[3] = 1.0 / r_grav[2];
      h_u_To_h_ci_l[5] = -r_grav[1] / (r_grav[2] * r_grav[2]);
      c_xt = -b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)
        indMeas_data[k] - 1] - 1.0) * numStatesPerAnchor) + 7.0) +
        featureAnchorIdx->data[(int)indMeas_data[k] - 1]) - 1];
      nm1d2 = (int)(numStates - 6.0);
      ar = H_grav->size[0] * H_grav->size[1];
      H_grav->size[0] = 3;
      H_grav->size[1] = 6 + (int)(numStates - 6.0);
      emxEnsureCapacity((emxArray__common *)H_grav, ar, (int)sizeof(double));
      for (ar = 0; ar < 3; ar++) {
        for (br = 0; br < 3; br++) {
          H_grav->data[br + H_grav->size[0] * ar] = c_xt * R_cw[br + 3 * ar];
        }
      }

      H_grav->data[H_grav->size[0] * 3] = 0.0;
      H_grav->data[H_grav->size[0] << 2] = -r_grav[2];
      H_grav->data[H_grav->size[0] * 5] = r_grav[1];
      H_grav->data[1 + H_grav->size[0] * 3] = r_grav[2];
      H_grav->data[1 + (H_grav->size[0] << 2)] = 0.0;
      H_grav->data[1 + H_grav->size[0] * 5] = -r_grav[0];
      H_grav->data[2 + H_grav->size[0] * 3] = -r_grav[1];
      H_grav->data[2 + (H_grav->size[0] << 2)] = r_grav[0];
      H_grav->data[2 + H_grav->size[0] * 5] = 0.0;
      for (ar = 0; ar < nm1d2; ar++) {
        for (br = 0; br < 3; br++) {
          H_grav->data[br + H_grav->size[0] * (ar + 6)] = 0.0;
        }
      }

      b_k = (signed char)((signed char)k << 1);
      for (ar = 0; ar < 2; ar++) {
        iv0[ar] = (signed char)(ar + b_k);
      }

      v[1] = H_grav->size[1];
      ar = C->size[0] * C->size[1];
      C->size[0] = 2;
      emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
      ar = C->size[0] * C->size[1];
      C->size[1] = (int)v[1];
      emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
      cr = (int)v[1] << 1;
      for (ar = 0; ar < cr; ar++) {
        C->data[ar] = 0.0;
      }

      nm1d2 = (H_grav->size[1] - 1) << 1;
      for (cr = 0; cr <= nm1d2; cr += 2) {
        for (ic = cr; ic + 1 <= cr + 2; ic++) {
          C->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= nm1d2; cr += 2) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 3; ib++) {
          if (H_grav->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 2; ic++) {
              ia++;
              C->data[ic] += H_grav->data[ib] * h_u_To_h_ci_l[ia - 1];
            }
          }

          ar += 2;
        }

        br += 3;
      }

      cr = C->size[1];
      for (ar = 0; ar < cr; ar++) {
        for (br = 0; br < 2; br++) {
          H_xc->data[iv0[br] + H_xc->size[0] * ar] = C->data[br + C->size[0] *
            ar];
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
      b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      for (ar = 0; ar < 3; ar++) {
        r_grav[ar] = b_xt->data[(int)(b_stateSize + (1.0 + (double)ar)) - 1];
      }

      b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      c_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      d_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      e_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      f_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      g_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      h_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      i_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      j_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      k_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      l_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      m_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      n_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      o_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      p_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      q_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      r_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      s_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      t_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      u_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      v_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      w_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      x_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      y_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0)
        * (7.0 + numPointsPerAnchor);
      ab_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      bb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      cb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      db_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      eb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      fb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      gb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      hb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      ib_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      jb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      kb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      lb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      mb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      nb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      ob_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      pb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      qb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      rb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      sb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      tb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      ub_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      vb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      wb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      xb_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] -
        1.0) * (7.0 + numPointsPerAnchor);
      b_measurements[0] = ((b_xt->data[(int)(b_stateSize + 4.0) - 1] *
                            b_xt->data[(int)(c_stateSize + 4.0) - 1] -
                            b_xt->data[(int)(d_stateSize + 5.0) - 1] *
                            b_xt->data[(int)(e_stateSize + 5.0) - 1]) -
                           b_xt->data[(int)(f_stateSize + 6.0) - 1] * b_xt->
                           data[(int)(g_stateSize + 6.0) - 1]) + b_xt->data[(int)
        (h_stateSize + 7.0) - 1] * b_xt->data[(int)(i_stateSize + 7.0) - 1];
      b_measurements[1] = 2.0 * (b_xt->data[(int)(j_stateSize + 4.0) - 1] *
        b_xt->data[(int)(k_stateSize + 5.0) - 1] + b_xt->data[(int)(l_stateSize
        + 6.0) - 1] * b_xt->data[(int)(m_stateSize + 7.0) - 1]);
      b_measurements[2] = 2.0 * (b_xt->data[(int)(n_stateSize + 4.0) - 1] *
        b_xt->data[(int)(o_stateSize + 6.0) - 1] - b_xt->data[(int)(p_stateSize
        + 5.0) - 1] * b_xt->data[(int)(q_stateSize + 7.0) - 1]);
      b_measurements[3] = 2.0 * (b_xt->data[(int)(r_stateSize + 4.0) - 1] *
        b_xt->data[(int)(s_stateSize + 5.0) - 1] - b_xt->data[(int)(t_stateSize
        + 6.0) - 1] * b_xt->data[(int)(u_stateSize + 7.0) - 1]);
      b_measurements[4] = ((-(b_xt->data[(int)(v_stateSize + 4.0) - 1] *
        b_xt->data[(int)(w_stateSize + 4.0) - 1]) + b_xt->data[(int)(x_stateSize
        + 5.0) - 1] * b_xt->data[(int)(y_stateSize + 5.0) - 1]) - b_xt->data
                           [(int)(ab_stateSize + 6.0) - 1] * b_xt->data[(int)
                           (bb_stateSize + 6.0) - 1]) + b_xt->data[(int)
        (cb_stateSize + 7.0) - 1] * b_xt->data[(int)(db_stateSize + 7.0) - 1];
      b_measurements[5] = 2.0 * (b_xt->data[(int)(eb_stateSize + 5.0) - 1] *
        b_xt->data[(int)(fb_stateSize + 6.0) - 1] + b_xt->data[(int)
        (gb_stateSize + 4.0) - 1] * b_xt->data[(int)(hb_stateSize + 7.0) - 1]);
      b_measurements[6] = 2.0 * (b_xt->data[(int)(ib_stateSize + 4.0) - 1] *
        b_xt->data[(int)(jb_stateSize + 6.0) - 1] + b_xt->data[(int)
        (kb_stateSize + 5.0) - 1] * b_xt->data[(int)(lb_stateSize + 7.0) - 1]);
      b_measurements[7] = 2.0 * (b_xt->data[(int)(mb_stateSize + 5.0) - 1] *
        b_xt->data[(int)(nb_stateSize + 6.0) - 1] - b_xt->data[(int)
        (ob_stateSize + 4.0) - 1] * b_xt->data[(int)(pb_stateSize + 7.0) - 1]);
      b_measurements[8] = ((-(b_xt->data[(int)(qb_stateSize + 4.0) - 1] *
        b_xt->data[(int)(rb_stateSize + 4.0) - 1]) - b_xt->data[(int)
                            (sb_stateSize + 5.0) - 1] * b_xt->data[(int)
                            (tb_stateSize + 5.0) - 1]) + b_xt->data[(int)
                           (ub_stateSize + 6.0) - 1] * b_xt->data[(int)
                           (vb_stateSize + 6.0) - 1]) + b_xt->data[(int)
        (wb_stateSize + 7.0) - 1] * b_xt->data[(int)(xb_stateSize + 7.0) - 1];
      for (ar = 0; ar < 3; ar++) {
        y[ar] = 0.0;
        for (br = 0; br < 3; br++) {
          b_y = y[ar] + b_measurements[ar + 3 * br] * b_m_vect->data[br +
            b_m_vect->size[0] * ((int)indMeas_data[k] - 1)];
          y[ar] = b_y;
        }
      }

      c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
        - 1] - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) +
        featureAnchorIdx->data[(int)indMeas_data[k] - 1]) - 1];
      for (ar = 0; ar < 3; ar++) {
        for (br = 0; br < 3; br++) {
          b_R_cw[br + 3 * ar] = -R_cw[br + 3 * ar];
        }
      }

      dv3[0] = 0.0;
      dv3[3] = -y[2];
      dv3[6] = y[1];
      dv3[1] = y[2];
      dv3[4] = 0.0;
      dv3[7] = -y[0];
      dv3[2] = -y[1];
      dv3[5] = y[0];
      dv3[8] = 0.0;
      nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
      for (ar = 0; ar < 3; ar++) {
        b_map[ar] = r_grav[ar] - b_xt->data[ar];
      }

      for (ar = 0; ar < 3; ar++) {
        c_R_cw[ar] = 0.0;
        for (br = 0; br < 3; br++) {
          c_R_cw[ar] += R_cw[ar + 3 * br] * b_map[br];
        }
      }

      cr = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas_data[k]
                 - 1]);
      for (ar = 0; ar < 3; ar++) {
        for (br = 0; br < 3; br++) {
          d_R_cw[ar + 3 * br] = 0.0;
          for (i6 = 0; i6 < 3; i6++) {
            d_R_cw[ar + 3 * br] += b_R_cw[ar + 3 * i6] * dv3[i6 + 3 * br];
          }
        }
      }

      ar = H_grav->size[0] * H_grav->size[1];
      H_grav->size[0] = 3;
      H_grav->size[1] = (nm1d2 + cr) + 7;
      emxEnsureCapacity((emxArray__common *)H_grav, ar, (int)sizeof(double));
      for (ar = 0; ar < 3; ar++) {
        for (br = 0; br < 3; br++) {
          H_grav->data[br + H_grav->size[0] * ar] = c_xt * R_cw[br + 3 * ar];
        }
      }

      for (ar = 0; ar < 3; ar++) {
        for (br = 0; br < 3; br++) {
          H_grav->data[br + H_grav->size[0] * (ar + 3)] = d_R_cw[br + 3 * ar];
        }
      }

      for (ar = 0; ar < nm1d2; ar++) {
        for (br = 0; br < 3; br++) {
          H_grav->data[br + H_grav->size[0] * (ar + 6)] = 0.0;
        }
      }

      for (ar = 0; ar < 3; ar++) {
        H_grav->data[ar + H_grav->size[0] * (6 + nm1d2)] = c_R_cw[ar];
      }

      for (ar = 0; ar < cr; ar++) {
        for (br = 0; br < 3; br++) {
          H_grav->data[br + H_grav->size[0] * ((ar + nm1d2) + 7)] = 0.0;
        }
      }

      b_k = (signed char)((signed char)k << 1);
      for (ar = 0; ar < 2; ar++) {
        iv0[ar] = (signed char)(ar + b_k);
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
          nm1d2 = br / 2;
          for (cr = 1; cr < nm1d2; cr++) {
            H_pressure->data[cr] = anew + (double)cr;
            H_pressure->data[br - cr] = apnd - (double)cr;
          }

          if (nm1d2 << 1 == br) {
            H_pressure->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            H_pressure->data[nm1d2] = anew + (double)nm1d2;
            H_pressure->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      b_anchorIdx = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
        numPointsPerAnchor);
      ar = r1->size[0];
      r1->size[0] = H_pressure->size[1];
      emxEnsureCapacity((emxArray__common *)r1, ar, (int)sizeof(int));
      cr = H_pressure->size[1];
      for (ar = 0; ar < cr; ar++) {
        r1->data[ar] = (int)(b_anchorIdx + H_pressure->data[H_pressure->size[0] *
                             ar]) - 1;
      }

      v[1] = H_grav->size[1];
      ar = C->size[0] * C->size[1];
      C->size[0] = 2;
      emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
      ar = C->size[0] * C->size[1];
      C->size[1] = (int)v[1];
      emxEnsureCapacity((emxArray__common *)C, ar, (int)sizeof(double));
      cr = (int)v[1] << 1;
      for (ar = 0; ar < cr; ar++) {
        C->data[ar] = 0.0;
      }

      nm1d2 = (H_grav->size[1] - 1) << 1;
      for (cr = 0; cr <= nm1d2; cr += 2) {
        for (ic = cr; ic + 1 <= cr + 2; ic++) {
          C->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= nm1d2; cr += 2) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 3; ib++) {
          if (H_grav->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 2; ic++) {
              ia++;
              C->data[ic] += H_grav->data[ib] * h_u_To_h_ci_l[ia - 1];
            }
          }

          ar += 2;
        }

        br += 3;
      }

      cr = C->size[1];
      for (ar = 0; ar < cr; ar++) {
        for (br = 0; br < 2; br++) {
          H_xm->data[iv0[br] + H_xm->size[0] * r1->data[ar]] = C->data[br +
            C->size[0] * ar];
        }
      }
    }

    k++;
  }

  emxFree_int32_T(&r1);
  emxFree_real_T(&C);
  v[0] = noiseParameters_image_noise[0] * noiseParameters_image_noise[0];
  v[1] = noiseParameters_image_noise[1] * noiseParameters_image_noise[1];
  for (ar = 0; ar < 4; ar++) {
    d[ar] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = v[nm1d2];
  }

  emxInit_real_T(&r2, 2);
  b_eye((double)indMeas_size[0], r2);
  kron(r2->data, r2->size, d, R_vision_data, R_vision_size);
  r_pressure = 0.0;
  emxFree_real_T(&r2);
  for (nm1d2 = 0; nm1d2 < 6; nm1d2++) {
    r_ext_pose[nm1d2] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
    r_grav[nm1d2] = 0.0;
  }

  ar = H_pressure->size[0] * H_pressure->size[1];
  H_pressure->size[0] = 1;
  H_pressure->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_pressure, ar, (int)sizeof(double));
  cr = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
              b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < cr; ar++) {
    H_pressure->data[ar] = 0.0;
  }

  emxInit_real_T(&H_ext_pose, 2);
  ar = H_ext_pose->size[0] * H_ext_pose->size[1];
  H_ext_pose->size[0] = 6;
  H_ext_pose->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors *
    (6.0 + b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_ext_pose, ar, (int)sizeof(double));
  cr = 6 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < cr; ar++) {
    H_ext_pose->data[ar] = 0.0;
  }

  ar = H_grav->size[0] * H_grav->size[1];
  H_grav->size[0] = 3;
  H_grav->size[1] = (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  emxEnsureCapacity((emxArray__common *)H_grav, ar, (int)sizeof(double));
  cr = 3 * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0 +
    b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < cr; ar++) {
    H_grav->data[ar] = 0.0;
  }

  residual_size = indMeas_size[0] << 1;
  pressure_idx = 0;
  ext_pose_idx = 0;
  grav_idx = 0;
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
        for (i6 = 0; i6 < 3; i6++) {
          b_measurements[ar + 3 * br] += c_measurements[ar + 3 * i6] * R_cw[br +
            3 * i6];
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

    for (ar = 0; ar < 9; ar++) {
      b_I[ar] = 0;
    }

    for (k = 0; k < 3; k++) {
      b_I[k + 3 * k] = 1;
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
        H_ext_pose->data[(br + H_ext_pose->size[0] * (ar + 3)) + 3] = b_I[br + 3
          * ar];
      }
    }

    ext_pose_idx = residual_size;
    residual_size += 6;
  }

  if (b_VIOParameters.gravity_align) {
    B = norm(measurements->acc_duo);
    for (ar = 0; ar < 3; ar++) {
      c_xt = 0.0;
      for (br = 0; br < 3; br++) {
        c_xt += R_cw[ar + 3 * br] * (double)b[br];
      }

      r_grav[ar] = measurements->acc_duo[ar] / B - c_xt;
    }

    H_grav->data[H_grav->size[0] * 3] = 0.0;
    H_grav->data[H_grav->size[0] << 2] = -R_cw[8];
    H_grav->data[H_grav->size[0] * 5] = R_cw[7];
    H_grav->data[1 + H_grav->size[0] * 3] = R_cw[8];
    H_grav->data[1 + (H_grav->size[0] << 2)] = 0.0;
    H_grav->data[1 + H_grav->size[0] * 5] = -R_cw[6];
    H_grav->data[2 + H_grav->size[0] * 3] = -R_cw[7];
    H_grav->data[2 + (H_grav->size[0] << 2)] = R_cw[6];
    H_grav->data[2 + H_grav->size[0] * 5] = 0.0;
    grav_idx = residual_size;
    residual_size += 3;
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
  cr = residual_size * (int)(errorStateSize + b_VIOParameters.num_anchors * (6.0
    + b_VIOParameters.num_points_per_anchor));
  for (ar = 0; ar < cr; ar++) {
    H->data[ar] = 0.0;
  }

  R_size[0] = residual_size;
  R_size[1] = residual_size;
  cr = residual_size * residual_size;
  for (ar = 0; ar < cr; ar++) {
    R_data[ar] = 0.0;
  }

  ar = indMeas_size[0] << 1;
  if (1 > ar) {
    cr = 0;
  } else {
    cr = ar;
  }

  for (ar = 0; ar < cr; ar++) {
    tmp_data[ar] = ar;
  }

  for (ar = 0; ar < cr; ar++) {
    r_data[tmp_data[ar]] = z_data[ar] - h_u_data[ar];
  }

  cr = H_xc->size[1];
  for (ar = 0; ar < cr; ar++) {
    nm1d2 = H_xc->size[0];
    for (br = 0; br < nm1d2; br++) {
      H->data[br + H->size[0] * ar] = H_xc->data[br + H_xc->size[0] * ar];
    }
  }

  cr = H_xm->size[1];
  for (ar = 0; ar < cr; ar++) {
    nm1d2 = H_xm->size[0];
    for (br = 0; br < nm1d2; br++) {
      H->data[br + H->size[0] * (ar + H_xc->size[1])] = H_xm->data[br +
        H_xm->size[0] * ar];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  cr = R_vision_size[1];
  for (ar = 0; ar < cr; ar++) {
    nm1d2 = R_vision_size[0];
    for (br = 0; br < nm1d2; br++) {
      R_data[br + R_size[0] * ar] = R_vision_data[br + R_vision_size[0] * ar];
    }
  }

  if (b_VIOParameters.use_pressure) {
    r_data[pressure_idx] = r_pressure;
    cr = H_pressure->size[1];
    for (ar = 0; ar < cr; ar++) {
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
      iv1[ar] = (signed char)(ar + (signed char)ext_pose_idx);
    }

    cr = H_ext_pose->size[1];
    for (ar = 0; ar < cr; ar++) {
      for (br = 0; br < 6; br++) {
        H->data[iv1[br] + H->size[0] * ar] = H_ext_pose->data[br +
          H_ext_pose->size[0] * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[(br + ext_pose_idx) + R_size[0] * (ar + ext_pose_idx)] =
          noiseParameters_ext_pos_noise * (double)b_b[br + 3 * ar];
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
          = noiseParameters_ext_att_noise * (double)b_b[br + 3 * ar];
      }
    }
  }

  emxFree_real_T(&H_ext_pose);
  if (b_VIOParameters.gravity_align) {
    for (ar = 0; ar < 3; ar++) {
      r_data[ar + grav_idx] = r_grav[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      iv2[ar] = (signed char)(ar + (signed char)grav_idx);
    }

    cr = H_grav->size[1];
    for (ar = 0; ar < cr; ar++) {
      for (br = 0; br < 3; br++) {
        H->data[iv2[br] + H->size[0] * ar] = H_grav->data[br + H_grav->size[0] *
          ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (br = 0; br < 3; br++) {
        R_data[(br + grav_idx) + R_size[0] * (ar + grav_idx)] =
          c_noiseParameters_gravity_align * (double)b_b[br + 3 * ar];
      }
    }
  }

  emxFree_real_T(&H_grav);
}

//
// File trailer for getH_R_res.cpp
//
// [EOF]
//
