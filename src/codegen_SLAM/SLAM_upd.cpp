//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_upd.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_upd.h"
#include "ros_info.h"
#include "SLAM_emxutil.h"
#include "norm.h"
#include "predictMeasurementStereo.h"
#include "any.h"
#include "initializePoint.h"
#include "det.h"
#include "anyActiveAnchorFeatures.h"
#include "predictMeasurementStereoDistorted.h"
#include "getScaledMap.h"
#include "getMap.h"
#include "QuatFromRotJ.h"
#include "eye.h"
#include "ros_error.h"
#include "getNumValidFeatures.h"
#include "eml_sort.h"
#include "getTotalNumDelayedFeatures.h"
#include "getTotalNumActiveFeatures.h"
#include "median.h"
#include "OnePointRANSAC_EKF.h"
#include "undistortPoint.h"
#include "multiplyIdx.h"
#include "SLAM_data.h"
#include <ros/console.h>

// Function Declarations
static double rt_roundd_snf(double u);

// Function Definitions

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// % Iterative Camera Pose optimization (EKF)
// Arguments    : emxArray_real_T *P_apr
//                f_struct_T *b_xt
//                const double c_cameraParams_CameraParameters[2]
//                const double d_cameraParams_CameraParameters[2]
//                const double e_cameraParams_CameraParameters[3]
//                const double f_cameraParams_CameraParameters[2]
//                const double g_cameraParams_CameraParameters[2]
//                const double h_cameraParams_CameraParameters[3]
//                const double cameraParams_r_lr[3]
//                const double cameraParams_R_lr[9]
//                const double cameraParams_R_rl[9]
//                int updateVect[40]
//                double z_all_l[80]
//                double z_all_r[80]
//                const double noiseParameters_image_noise[2]
//                double noiseParameters_sigmaInit
//                double c_VIOParameters_max_ekf_iterati
//                boolean_T VIOParameters_fixed_feature
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_mono
//                emxArray_real_T *h_u_apo
//                emxArray_real_T *b_map
//                double b_delayedStatus[40]
// Return Type  : void
//
void SLAM_upd(emxArray_real_T *P_apr, f_struct_T *b_xt, const double
              c_cameraParams_CameraParameters[2], const double
              d_cameraParams_CameraParameters[2], const double
              e_cameraParams_CameraParameters[3], const double
              f_cameraParams_CameraParameters[2], const double
              g_cameraParams_CameraParameters[2], const double
              h_cameraParams_CameraParameters[3], const double
              cameraParams_r_lr[3], const double cameraParams_R_lr[9], const
              double cameraParams_R_rl[9], int updateVect[40], double z_all_l[80],
              double z_all_r[80], const double noiseParameters_image_noise[2],
              double noiseParameters_sigmaInit, double
              c_VIOParameters_max_ekf_iterati, boolean_T
              VIOParameters_fixed_feature, boolean_T
              c_VIOParameters_delayed_initial, boolean_T VIOParameters_mono,
              emxArray_real_T *h_u_apo, emxArray_real_T *b_map, double
              b_delayedStatus[40])
{
  double numStatesPerAnchor;
  double c_numTrackFeatures;
  boolean_T x[40];
  int br;
  int idx;
  signed char ii_data[40];
  int ixstart;
  boolean_T exitg6;
  boolean_T guard2 = false;
  int ar;
  double b_ii_data[40];
  int ii_size[1];
  int i32;
  int ind_l2_size[1];
  double ind_l2_data[80];
  double z_all_l_data[80];
  int z_all_l_size[1];
  double tmp_data[80];
  boolean_T exitg5;
  boolean_T guard1 = false;
  int ind_r_size[1];
  double ind_r_data[40];
  int z_all_r_size[1];
  int anchorIdx;
  emxArray_struct_T *r5;
  int i33;
  int ib;
  int i34;
  emxArray_real_T *uncertainties;
  emxArray_real_T *active_feature;
  emxArray_real_T *status_ind;
  emxArray_int32_T *iidx;
  boolean_T fix_new_feature;
  emxArray_real_T *J;
  emxArray_int32_T *r6;
  emxArray_real_T *y;
  emxArray_real_T *b;
  emxArray_real_T *new_m;
  double z_curr_l[2];
  double z_curr_r[2];
  boolean_T success;
  double b_m[6];
  double new_origin_pos_rel[3];
  double m_l[3];
  boolean_T bv0[3];
  double h_u_r[2];
  double h_u_l[2];
  double b_P_apr[2];
  boolean_T b_guard1 = false;
  double B;
  int k;
  emxArray_real_T *triangulated_depths;
  emxArray_real_T *triangulated_status_ind;
  emxArray_real_T *b_triangulated_status_ind;
  emxArray_real_T *untriangulated_depths;
  emxArray_real_T *untriangulated_status_ind;
  emxArray_real_T *b_untriangulated_status_ind;
  emxArray_real_T *b_new_m;
  emxArray_real_T *c_new_m;
  emxArray_real_T *d_new_m;
  unsigned int new_feature_idx;
  emxArray_real_T *b_y;
  e_struct_T expl_temp;
  boolean_T exitg3;
  double c_xt[3];
  double d_xt[4];
  double d7;
  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double e_xt;
  static const signed char iv9[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int c_m;
  int ic;
  int ia;
  boolean_T exitg4;
  boolean_T has_active_features;
  emxArray_real_T *b_uncertainties;
  double median_uncertainty;
  double totalNumActiveFeatues;
  double numDelayedFeatures;
  unsigned int delayedIdx;
  double numActivatedFeatures;
  boolean_T request_new_features;
  e_struct_T b_expl_temp;
  boolean_T exitg2;
  e_struct_T c_expl_temp;
  double c_P_apr[36];
  double mtmp;
  boolean_T exitg1;
  double new_origin_att_rel[9];
  double f_xt[9];
  double g_xt[9];
  double h_xt[9];
  double i_xt[9];
  double j_xt[9];

  // debug
  numStatesPerAnchor = 6.0 + numPointsPerAnchor;
  c_numTrackFeatures = numAnchors * numPointsPerAnchor;

  //  undistort all valid points
  for (br = 0; br < 40; br++) {
    x[br] = (updateVect[br] != 0);
  }

  idx = 0;
  ixstart = 1;
  exitg6 = false;
  while ((!exitg6) && (ixstart < 41)) {
    guard2 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)ixstart;
      if (idx >= 40) {
        exitg6 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      ixstart++;
    }
  }

  if (1 > idx) {
    ar = 0;
  } else {
    ar = idx;
  }

  ii_size[0] = ar;
  for (i32 = 0; i32 < ar; i32++) {
    b_ii_data[i32] = ii_data[i32];
  }

  multiplyIdx(b_ii_data, ii_size, ind_l2_data, ind_l2_size);
  z_all_l_size[0] = ind_l2_size[0];
  ar = ind_l2_size[0];
  for (i32 = 0; i32 < ar; i32++) {
    z_all_l_data[i32] = z_all_l[(int)ind_l2_data[i32] - 1];
  }

  undistortPoint(z_all_l_data, z_all_l_size, c_cameraParams_CameraParameters,
                 d_cameraParams_CameraParameters,
                 e_cameraParams_CameraParameters, tmp_data, ii_size);
  ar = ii_size[0];
  for (i32 = 0; i32 < ar; i32++) {
    z_all_l[(int)ind_l2_data[i32] - 1] = tmp_data[i32];
  }

  for (br = 0; br < 40; br++) {
    x[br] = (updateVect[br] == 2);
  }

  idx = 0;
  ixstart = 1;
  exitg5 = false;
  while ((!exitg5) && (ixstart < 41)) {
    guard1 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)ixstart;
      if (idx >= 40) {
        exitg5 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      ixstart++;
    }
  }

  if (1 > idx) {
    ar = 0;
  } else {
    ar = idx;
  }

  if (1 > idx) {
    ixstart = 0;
  } else {
    ixstart = idx;
  }

  ind_r_size[0] = ar;
  for (i32 = 0; i32 < ar; i32++) {
    ind_r_data[i32] = ii_data[i32];
  }

  multiplyIdx(ind_r_data, ind_r_size, ind_l2_data, ind_l2_size);
  z_all_r_size[0] = ind_l2_size[0];
  br = ind_l2_size[0];
  for (i32 = 0; i32 < br; i32++) {
    z_all_l_data[i32] = z_all_r[(int)ind_l2_data[i32] - 1];
  }

  undistortPoint(z_all_l_data, z_all_r_size, f_cameraParams_CameraParameters,
                 g_cameraParams_CameraParameters,
                 h_cameraParams_CameraParameters, tmp_data, ii_size);
  br = ii_size[0];
  for (i32 = 0; i32 < br; i32++) {
    z_all_r[(int)ind_l2_data[i32] - 1] = tmp_data[i32];
  }

  //  check for lost features
  i32 = (int)numAnchors;
  anchorIdx = 0;
  b_emxInit_struct_T(&r5, 1);
  while (anchorIdx <= i32 - 1) {
    i33 = (int)numPointsPerAnchor;
    for (ib = 0; ib < i33; ib++) {
      if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].status
          != 0.0) {
        i34 = r5->size[0];
        r5->size[0] = b_xt->anchor_states->data[anchorIdx].feature_states->size
          [0];
        emxEnsureCapacity((emxArray__common *)r5, i34, (int)sizeof(d_struct_T));
        br = b_xt->anchor_states->data[anchorIdx].feature_states->size[0];
        for (i34 = 0; i34 < br; i34++) {
          r5->data[i34] = b_xt->anchor_states->data[anchorIdx]
            .feature_states->data[i34];
        }

        if (updateVect[(int)b_xt->anchor_states->data[anchorIdx].
            feature_states->data[ib].status_idx - 1] != 1) {
          br = P_apr->size[1];
          idx = (int)b_xt->anchor_states->data[anchorIdx].feature_states->
            data[ib].P_idx;
          for (i34 = 0; i34 < br; i34++) {
            P_apr->data[(idx + P_apr->size[0] * i34) - 1] = 0.0;
          }

          br = P_apr->size[0];
          idx = (int)b_xt->anchor_states->data[anchorIdx].feature_states->
            data[ib].P_idx;
          for (i34 = 0; i34 < br; i34++) {
            P_apr->data[i34 + P_apr->size[0] * (idx - 1)] = 0.0;
          }

          b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].status =
            0.0;
          b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
            status_idx = 0.0;
          ros_info(r5->data[ib].status_idx, 1.0 + (double)ib, 1.0 + (double)
                   anchorIdx);
        }
      }
    }

    anchorIdx++;
  }

  b_emxFree_struct_T(&r5);
  b_emxInit_real_T(&uncertainties, 1);
  b_emxInit_real_T(&active_feature, 1);
  b_emxInit_real_T(&status_ind, 1);
  b_emxInit_int32_T(&iidx, 1);
  if (VIOParameters_fixed_feature) {
    fix_new_feature = false;
    if (b_xt->origin.anchor_idx != 0.0) {
      if (b_xt->fixed_feature != 0.0) {
        if (b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
            feature_states->data[(int)b_xt->fixed_feature - 1].status != 1.0) {
          fix_new_feature = true;
          b_ros_info(b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1]
                     .feature_states->data[(int)b_xt->fixed_feature - 1].
                     status_idx, b_xt->fixed_feature, b_xt->origin.anchor_idx);
        }
      } else {
        fix_new_feature = true;
      }
    }

    if (fix_new_feature) {
      i32 = uncertainties->size[0];
      uncertainties->size[0] = (int)numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)uncertainties, i32, (int)sizeof
                        (double));
      br = (int)numPointsPerAnchor;
      for (i32 = 0; i32 < br; i32++) {
        uncertainties->data[i32] = 0.0;
      }

      i32 = active_feature->size[0];
      active_feature->size[0] = (int)numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)active_feature, i32, (int)sizeof
                        (double));
      br = (int)numPointsPerAnchor;
      for (i32 = 0; i32 < br; i32++) {
        active_feature->data[i32] = 0.0;
      }

      for (ib = 0; ib < (int)numPointsPerAnchor; ib++) {
        if (b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
            feature_states->data[ib].status == 1.0) {
          uncertainties->data[ib] = P_apr->data[((int)b_xt->anchor_states->data
            [(int)b_xt->origin.anchor_idx - 1].feature_states->data[ib].P_idx +
            P_apr->size[0] * ((int)b_xt->anchor_states->data[(int)
                              b_xt->origin.anchor_idx - 1].feature_states->
                              data[ib].P_idx - 1)) - 1];
          active_feature->data[ib] = 1.0;
        } else {
          uncertainties->data[ib] = 1000.0;

          //  dont fix an inactive feature
        }
      }

      eml_sort(uncertainties, iidx);
      i32 = status_ind->size[0];
      status_ind->size[0] = iidx->size[0];
      emxEnsureCapacity((emxArray__common *)status_ind, i32, (int)sizeof(double));
      br = iidx->size[0];
      for (i32 = 0; i32 < br; i32++) {
        status_ind->data[i32] = iidx->data[i32];
      }

      if (!(active_feature->data[(int)status_ind->data[0] - 1] != 0.0)) {
        ros_error();
      }

      b_xt->fixed_feature = status_ind->data[0];
      br = P_apr->size[1];
      idx = (int)b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
        feature_states->data[(int)status_ind->data[0] - 1].P_idx;
      for (i32 = 0; i32 < br; i32++) {
        P_apr->data[(idx + P_apr->size[0] * i32) - 1] = 0.0;
      }

      //  fix the feature depth
      br = P_apr->size[0];
      idx = (int)b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
        feature_states->data[(int)status_ind->data[0] - 1].P_idx;
      for (i32 = 0; i32 < br; i32++) {
        P_apr->data[i32 + P_apr->size[0] * (idx - 1)] = 0.0;
      }

      c_ros_info(b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
                 feature_states->data[(int)status_ind->data[0] - 1].status_idx,
                 status_ind->data[0], b_xt->origin.anchor_idx);
    }
  }

  // % do the update
  for (br = 0; br < 40; br++) {
    x[br] = (updateVect[br] == 1);
  }

  if (any(x)) {
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, c_cameraParams_CameraParameters,
                       d_cameraParams_CameraParameters,
                       noiseParameters_image_noise,
                       c_VIOParameters_max_ekf_iterati, updateVect);
  }

  // % Initialize new anchors/features
  emxInit_real_T(&J, 2);
  b_emxInit_int32_T(&r6, 1);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b, 2);
  if (ixstart >= minFeatureThreshold) {
    //  try to triangulate all new features
    i32 = active_feature->size[0];
    active_feature->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)active_feature, i32, (int)sizeof
                      (double));
    for (i32 = 0; i32 < ixstart; i32++) {
      active_feature->data[i32] = 0.0;
    }

    emxInit_real_T(&new_m, 2);
    i32 = new_m->size[0] * new_m->size[1];
    new_m->size[0] = 3;
    emxEnsureCapacity((emxArray__common *)new_m, i32, (int)sizeof(double));
    i32 = new_m->size[0] * new_m->size[1];
    new_m->size[1] = ar;
    emxEnsureCapacity((emxArray__common *)new_m, i32, (int)sizeof(double));
    br = 3 * ixstart;
    for (i32 = 0; i32 < br; i32++) {
      new_m->data[i32] = 0.0;
    }

    i32 = triangulation_success->size[0];
    triangulation_success->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)triangulation_success, i32, (int)
                      sizeof(boolean_T));
    for (i32 = 0; i32 < ixstart; i32++) {
      triangulation_success->data[i32] = false;
    }

    for (br = 0; br < ar; br++) {
      idx = ((int)ind_r_data[br] - 1) * 2;
      ixstart = ((int)ind_r_data[br] - 1) * 2;
      for (i32 = 0; i32 < 2; i32++) {
        z_curr_l[i32] = z_all_l[idx + i32];
        z_curr_r[i32] = z_all_r[ixstart + i32];
      }

      if (!VIOParameters_mono) {
        initializePoint(z_curr_l, z_curr_r, c_cameraParams_CameraParameters,
                        d_cameraParams_CameraParameters,
                        f_cameraParams_CameraParameters,
                        g_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, new_origin_pos_rel, b_m, &success);
        for (i32 = 0; i32 < 3; i32++) {
          m_l[i32] = b_m[i32];
        }

        if (success) {
          //  perform further checks
          for (ixstart = 0; ixstart < 3; ixstart++) {
            bv0[ixstart] = rtIsNaN(new_origin_pos_rel[ixstart]);
          }

          if (b_any(bv0)) {
            for (i32 = 0; i32 < 3; i32++) {
              new_origin_pos_rel[i32] = b_m[i32];
            }

            success = false;
          } else {
            //  check reprojection error
            predictMeasurementStereo(new_origin_pos_rel,
              c_cameraParams_CameraParameters, d_cameraParams_CameraParameters,
              f_cameraParams_CameraParameters, g_cameraParams_CameraParameters,
              cameraParams_r_lr, cameraParams_R_rl, h_u_l, h_u_r);
            for (ixstart = 0; ixstart < 2; ixstart++) {
              b_P_apr[ixstart] = h_u_l[ixstart] - z_curr_l[ixstart];
            }

            b_guard1 = false;
            if (c_norm(b_P_apr) > 2.0) {
              b_guard1 = true;
            } else {
              for (ixstart = 0; ixstart < 2; ixstart++) {
                b_P_apr[ixstart] = h_u_r[ixstart] - z_curr_r[ixstart];
              }

              if (c_norm(b_P_apr) > 2.0) {
                b_guard1 = true;
              } else {
                if (norm(new_origin_pos_rel) < 0.1) {
                  //  feature triangulated very close
                  for (i32 = 0; i32 < 3; i32++) {
                    new_origin_pos_rel[i32] = b_m[i32];
                  }

                  success = false;
                }
              }
            }

            if (b_guard1) {
              //                          ros_info('Bad triangulation (reprojection error) for point %d', int8(ind_r(i))); 
              for (i32 = 0; i32 < 3; i32++) {
                new_origin_pos_rel[i32] = b_m[i32];
              }

              success = false;
            }
          }
        } else {
          for (i32 = 0; i32 < 3; i32++) {
            new_origin_pos_rel[i32] = b_m[i32];
          }
        }
      } else {
        //  mono
        m_l[0] = (z_all_l[((int)ind_r_data[br] - 1) * 2] -
                  d_cameraParams_CameraParameters[0]) /
          c_cameraParams_CameraParameters[0];
        m_l[1] = (z_all_l[((int)ind_r_data[br] - 1) * 2 + 1] -
                  d_cameraParams_CameraParameters[1]) /
          c_cameraParams_CameraParameters[1];
        m_l[2] = 1.0;
        B = norm(m_l);
        for (ixstart = 0; ixstart < 3; ixstart++) {
          new_origin_pos_rel[ixstart] = m_l[ixstart] / B;
        }

        success = true;
      }

      active_feature->data[br] = norm(new_origin_pos_rel);
      for (i32 = 0; i32 < 3; i32++) {
        new_m->data[i32 + new_m->size[0] * br] = m_l[i32];
      }

      triangulation_success->data[br] = success;
    }

    ar = 0;
    for (k = 0; k < triangulation_success->size[0]; k++) {
      if (triangulation_success->data[k]) {
        ar++;
      }
    }

    e_ros_info(ar, triangulation_success->size[0]);
    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        ixstart++;
      }
    }

    b_emxInit_real_T(&triangulated_depths, 1);
    i32 = triangulated_depths->size[0];
    triangulated_depths->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)triangulated_depths, i32, (int)sizeof
                      (double));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        triangulated_depths->data[ixstart] = active_feature->data[br];
        ixstart++;
      }
    }

    eml_sort(triangulated_depths, iidx);
    i32 = status_ind->size[0];
    status_ind->size[0] = iidx->size[0];
    emxEnsureCapacity((emxArray__common *)status_ind, i32, (int)sizeof(double));
    ar = iidx->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      status_ind->data[i32] = iidx->data[i32];
    }

    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        ixstart++;
      }
    }

    i32 = r6->size[0];
    r6->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)r6, i32, (int)sizeof(int));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        r6->data[ixstart] = br + 1;
        ixstart++;
      }
    }

    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        ixstart++;
      }
    }

    b_emxInit_real_T(&triangulated_status_ind, 1);
    i32 = triangulated_status_ind->size[0];
    triangulated_status_ind->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)triangulated_status_ind, i32, (int)
                      sizeof(double));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        triangulated_status_ind->data[ixstart] = ind_r_data[br];
        ixstart++;
      }
    }

    b_emxInit_real_T(&b_triangulated_status_ind, 1);
    i32 = b_triangulated_status_ind->size[0];
    b_triangulated_status_ind->size[0] = status_ind->size[0];
    emxEnsureCapacity((emxArray__common *)b_triangulated_status_ind, i32, (int)
                      sizeof(double));
    ar = status_ind->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      b_triangulated_status_ind->data[i32] = triangulated_status_ind->data[(int)
        status_ind->data[i32] - 1];
    }

    i32 = triangulated_status_ind->size[0];
    triangulated_status_ind->size[0] = b_triangulated_status_ind->size[0];
    emxEnsureCapacity((emxArray__common *)triangulated_status_ind, i32, (int)
                      sizeof(double));
    ar = b_triangulated_status_ind->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      triangulated_status_ind->data[i32] = b_triangulated_status_ind->data[i32];
    }

    emxFree_real_T(&b_triangulated_status_ind);
    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        ixstart++;
      }
    }

    b_emxInit_real_T(&untriangulated_depths, 1);
    i32 = untriangulated_depths->size[0];
    untriangulated_depths->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)untriangulated_depths, i32, (int)
                      sizeof(double));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        untriangulated_depths->data[ixstart] = active_feature->data[br];
        ixstart++;
      }
    }

    eml_sort(untriangulated_depths, iidx);
    i32 = uncertainties->size[0];
    uncertainties->size[0] = iidx->size[0];
    emxEnsureCapacity((emxArray__common *)uncertainties, i32, (int)sizeof(double));
    ar = iidx->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      uncertainties->data[i32] = iidx->data[i32];
    }

    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        ixstart++;
      }
    }

    i32 = iidx->size[0];
    iidx->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)iidx, i32, (int)sizeof(int));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        iidx->data[ixstart] = br + 1;
        ixstart++;
      }
    }

    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        ixstart++;
      }
    }

    b_emxInit_real_T(&untriangulated_status_ind, 1);
    i32 = untriangulated_status_ind->size[0];
    untriangulated_status_ind->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)untriangulated_status_ind, i32, (int)
                      sizeof(double));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        untriangulated_status_ind->data[ixstart] = ind_r_data[br];
        ixstart++;
      }
    }

    b_emxInit_real_T(&b_untriangulated_status_ind, 1);
    i32 = b_untriangulated_status_ind->size[0];
    b_untriangulated_status_ind->size[0] = uncertainties->size[0];
    emxEnsureCapacity((emxArray__common *)b_untriangulated_status_ind, i32, (int)
                      sizeof(double));
    ar = uncertainties->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      b_untriangulated_status_ind->data[i32] = untriangulated_status_ind->data
        [(int)uncertainties->data[i32] - 1];
    }

    i32 = untriangulated_status_ind->size[0];
    untriangulated_status_ind->size[0] = b_untriangulated_status_ind->size[0];
    emxEnsureCapacity((emxArray__common *)untriangulated_status_ind, i32, (int)
                      sizeof(double));
    ar = b_untriangulated_status_ind->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      untriangulated_status_ind->data[i32] = b_untriangulated_status_ind->
        data[i32];
    }

    emxFree_real_T(&b_untriangulated_status_ind);
    i32 = active_feature->size[0];
    active_feature->size[0] = triangulated_depths->size[0] +
      untriangulated_depths->size[0];
    emxEnsureCapacity((emxArray__common *)active_feature, i32, (int)sizeof
                      (double));
    ar = triangulated_depths->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      active_feature->data[i32] = triangulated_depths->data[i32];
    }

    ar = untriangulated_depths->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      active_feature->data[i32 + triangulated_depths->size[0]] =
        untriangulated_depths->data[i32];
    }

    emxFree_real_T(&untriangulated_depths);
    emxFree_real_T(&triangulated_depths);
    emxInit_real_T(&b_new_m, 2);
    i32 = b_new_m->size[0] * b_new_m->size[1];
    b_new_m->size[0] = 3;
    b_new_m->size[1] = r6->size[0];
    emxEnsureCapacity((emxArray__common *)b_new_m, i32, (int)sizeof(double));
    ar = r6->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      for (i33 = 0; i33 < 3; i33++) {
        b_new_m->data[i33 + b_new_m->size[0] * i32] = new_m->data[i33 +
          new_m->size[0] * (r6->data[i32] - 1)];
      }
    }

    emxInit_real_T(&c_new_m, 2);
    i32 = c_new_m->size[0] * c_new_m->size[1];
    c_new_m->size[0] = 3;
    c_new_m->size[1] = iidx->size[0];
    emxEnsureCapacity((emxArray__common *)c_new_m, i32, (int)sizeof(double));
    ar = iidx->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      for (i33 = 0; i33 < 3; i33++) {
        c_new_m->data[i33 + c_new_m->size[0] * i32] = new_m->data[i33 +
          new_m->size[0] * (iidx->data[i32] - 1)];
      }
    }

    emxInit_real_T(&d_new_m, 2);
    i32 = d_new_m->size[0] * d_new_m->size[1];
    d_new_m->size[0] = 3;
    d_new_m->size[1] = status_ind->size[0] + uncertainties->size[0];
    emxEnsureCapacity((emxArray__common *)d_new_m, i32, (int)sizeof(double));
    ar = status_ind->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      for (i33 = 0; i33 < 3; i33++) {
        d_new_m->data[i33 + d_new_m->size[0] * i32] = b_new_m->data[i33 +
          b_new_m->size[0] * ((int)status_ind->data[i32] - 1)];
      }
    }

    emxFree_real_T(&b_new_m);
    ar = uncertainties->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      for (i33 = 0; i33 < 3; i33++) {
        d_new_m->data[i33 + d_new_m->size[0] * (i32 + status_ind->size[0])] =
          c_new_m->data[i33 + c_new_m->size[0] * ((int)uncertainties->data[i32]
          - 1)];
      }
    }

    emxFree_real_T(&c_new_m);
    i32 = new_m->size[0] * new_m->size[1];
    new_m->size[0] = 3;
    new_m->size[1] = d_new_m->size[1];
    emxEnsureCapacity((emxArray__common *)new_m, i32, (int)sizeof(double));
    ar = d_new_m->size[1];
    for (i32 = 0; i32 < ar; i32++) {
      for (i33 = 0; i33 < 3; i33++) {
        new_m->data[i33 + new_m->size[0] * i32] = d_new_m->data[i33 +
          d_new_m->size[0] * i32];
      }
    }

    emxFree_real_T(&d_new_m);
    i32 = status_ind->size[0];
    status_ind->size[0] = triangulated_status_ind->size[0] +
      untriangulated_status_ind->size[0];
    emxEnsureCapacity((emxArray__common *)status_ind, i32, (int)sizeof(double));
    ar = triangulated_status_ind->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      status_ind->data[i32] = triangulated_status_ind->data[i32];
    }

    ar = untriangulated_status_ind->size[0];
    for (i32 = 0; i32 < ar; i32++) {
      status_ind->data[i32 + triangulated_status_ind->size[0]] =
        untriangulated_status_ind->data[i32];
    }

    emxFree_real_T(&untriangulated_status_ind);
    emxFree_real_T(&triangulated_status_ind);

    //  insert new features into the state. first the closest triangulated ones, 
    //  then the un-triangulated ones
    new_feature_idx = 1U;
    i32 = (int)numAnchors;
    anchorIdx = 0;
    emxInit_real_T(&b_y, 2);
    b_emxInitStruct_struct_T(&expl_temp);
    exitg3 = false;
    while ((!exitg3) && (anchorIdx <= i32 - 1)) {
      //          if new_feature_idx > length(new_depths)
      ar = 0;
      for (k = 0; k < triangulation_success->size[0]; k++) {
        if (triangulation_success->data[k]) {
          ar++;
        }
      }

      if (((double)new_feature_idx > ar) || ((active_feature->size[0] - (int)
            new_feature_idx) + 1 < minFeatureThreshold)) {
        exitg3 = true;
      } else {
        emxCopyStruct_struct_T(&expl_temp, &b_xt->anchor_states->data[anchorIdx]);
        if (getNumValidFeatures(expl_temp.feature_states) < minFeatureThreshold)
        {
          //  anchor needs to be initialized
          //  free up updateVect
          for (ib = 0; ib < (int)numPointsPerAnchor; ib++) {
            if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                status != 0.0) {
              //                      ros_info('clearing up feature %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              updateVect[(int)b_xt->anchor_states->data[anchorIdx].
                feature_states->data[ib].status_idx - 1] = 0;
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                status_idx = 0.0;
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                status = 0.0;
            }
          }

          if (b_xt->origin.anchor_idx == 1.0 + (double)anchorIdx) {
            b_xt->origin.anchor_idx = 0.0;
            f_ros_info(1.0 + (double)anchorIdx);
          }

          for (i33 = 0; i33 < 3; i33++) {
            c_xt[i33] = b_xt->robot_state.pos[i33];
          }

          for (i33 = 0; i33 < 3; i33++) {
            b_xt->anchor_states->data[anchorIdx].pos[i33] = c_xt[i33];
          }

          for (i33 = 0; i33 < 4; i33++) {
            d_xt[i33] = b_xt->robot_state.att[i33];
          }

          for (i33 = 0; i33 < 4; i33++) {
            b_xt->anchor_states->data[anchorIdx].att[i33] = d_xt[i33];
          }

          d7 = numStates + ((1.0 + (double)anchorIdx) - 1.0) *
            numStatesPerAnchor;
          for (i33 = 0; i33 < 6; i33++) {
            b_xt->anchor_states->data[anchorIdx].P_idx[i33] = d7 + (1.0 +
              (double)i33);
          }

          ar = P_apr->size[1];
          for (i33 = 0; i33 < ar; i33++) {
            for (i34 = 0; i34 < 6; i34++) {
              P_apr->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i34]
                           + P_apr->size[0] * i33) - 1] = 0.0;
            }
          }

          ar = P_apr->size[0];
          for (i33 = 0; i33 < 6; i33++) {
            for (i34 = 0; i34 < ar; i34++) {
              P_apr->data[i34 + P_apr->size[0] * ((int)b_xt->anchor_states->
                data[anchorIdx].P_idx[i33] - 1)] = 0.0;
            }
          }

          if (rtIsNaN(numPointsPerAnchor)) {
            ar = 0;
            anew = rtNaN;
            apnd = numPointsPerAnchor;
          } else if (numPointsPerAnchor < 1.0) {
            ar = -1;
            anew = 1.0;
            apnd = numPointsPerAnchor;
          } else if (rtIsInf(numPointsPerAnchor)) {
            ar = 0;
            anew = rtNaN;
            apnd = numPointsPerAnchor;
          } else {
            anew = 1.0;
            ndbl = floor((numPointsPerAnchor - 1.0) + 0.5);
            apnd = 1.0 + ndbl;
            cdiff = (1.0 + ndbl) - numPointsPerAnchor;
            absb = fabs(numPointsPerAnchor);
            if ((1.0 >= absb) || rtIsNaN(absb)) {
              absb = 1.0;
            }

            if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
              ndbl++;
              apnd = numPointsPerAnchor;
            } else if (cdiff > 0.0) {
              apnd = 1.0 + (ndbl - 1.0);
            } else {
              ndbl++;
            }

            if (ndbl >= 0.0) {
              ar = (int)ndbl - 1;
            } else {
              ar = -1;
            }
          }

          i33 = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = ar + 1;
          emxEnsureCapacity((emxArray__common *)b_y, i33, (int)sizeof(double));
          if (ar + 1 > 0) {
            b_y->data[0] = anew;
            if (ar + 1 > 1) {
              b_y->data[ar] = apnd;
              idx = (ar + (ar < 0)) >> 1;
              for (k = 1; k < idx; k++) {
                b_y->data[k] = anew + (double)k;
                b_y->data[ar - k] = apnd - (double)k;
              }

              if (idx << 1 == ar) {
                b_y->data[idx] = (anew + apnd) / 2.0;
              } else {
                b_y->data[idx] = anew + (double)idx;
                b_y->data[idx + 1] = apnd - (double)idx;
              }
            }
          }

          e_xt = b_xt->anchor_states->data[anchorIdx].P_idx[5];
          i33 = r6->size[0];
          r6->size[0] = b_y->size[1];
          emxEnsureCapacity((emxArray__common *)r6, i33, (int)sizeof(int));
          ar = b_y->size[1];
          for (i33 = 0; i33 < ar; i33++) {
            r6->data[i33] = (int)(e_xt + b_y->data[b_y->size[0] * i33]);
          }

          ar = P_apr->size[1];
          idx = r6->size[0];
          for (i33 = 0; i33 < ar; i33++) {
            for (i34 = 0; i34 < idx; i34++) {
              P_apr->data[(r6->data[i34] + P_apr->size[0] * i33) - 1] = 0.0;
            }
          }

          if (rtIsNaN(numPointsPerAnchor)) {
            ar = 0;
            anew = rtNaN;
            apnd = numPointsPerAnchor;
          } else if (numPointsPerAnchor < 1.0) {
            ar = -1;
            anew = 1.0;
            apnd = numPointsPerAnchor;
          } else if (rtIsInf(numPointsPerAnchor)) {
            ar = 0;
            anew = rtNaN;
            apnd = numPointsPerAnchor;
          } else {
            anew = 1.0;
            ndbl = floor((numPointsPerAnchor - 1.0) + 0.5);
            apnd = 1.0 + ndbl;
            cdiff = (1.0 + ndbl) - numPointsPerAnchor;
            absb = fabs(numPointsPerAnchor);
            if ((1.0 >= absb) || rtIsNaN(absb)) {
              absb = 1.0;
            }

            if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
              ndbl++;
              apnd = numPointsPerAnchor;
            } else if (cdiff > 0.0) {
              apnd = 1.0 + (ndbl - 1.0);
            } else {
              ndbl++;
            }

            if (ndbl >= 0.0) {
              ar = (int)ndbl - 1;
            } else {
              ar = -1;
            }
          }

          i33 = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = ar + 1;
          emxEnsureCapacity((emxArray__common *)b_y, i33, (int)sizeof(double));
          if (ar + 1 > 0) {
            b_y->data[0] = anew;
            if (ar + 1 > 1) {
              b_y->data[ar] = apnd;
              idx = (ar + (ar < 0)) >> 1;
              for (k = 1; k < idx; k++) {
                b_y->data[k] = anew + (double)k;
                b_y->data[ar - k] = apnd - (double)k;
              }

              if (idx << 1 == ar) {
                b_y->data[idx] = (anew + apnd) / 2.0;
              } else {
                b_y->data[idx] = anew + (double)idx;
                b_y->data[idx + 1] = apnd - (double)idx;
              }
            }
          }

          e_xt = b_xt->anchor_states->data[anchorIdx].P_idx[5];
          i33 = r6->size[0];
          r6->size[0] = b_y->size[1];
          emxEnsureCapacity((emxArray__common *)r6, i33, (int)sizeof(int));
          ar = b_y->size[1];
          for (i33 = 0; i33 < ar; i33++) {
            r6->data[i33] = (int)(e_xt + b_y->data[b_y->size[0] * i33]);
          }

          ar = P_apr->size[0];
          ixstart = r6->size[0];
          for (i33 = 0; i33 < ixstart; i33++) {
            for (i34 = 0; i34 < ar; i34++) {
              P_apr->data[i34 + P_apr->size[0] * (r6->data[i33] - 1)] = 0.0;
            }
          }

          for (i33 = 0; i33 < 2; i33++) {
            b_P_apr[i33] = P_apr->size[i33];
          }

          c_eye(b_P_apr, J);
          if (rtIsNaN(numStatesPerAnchor)) {
            ar = 0;
            anew = rtNaN;
            apnd = numStatesPerAnchor;
          } else if (numStatesPerAnchor < 1.0) {
            ar = -1;
            anew = 1.0;
            apnd = numStatesPerAnchor;
          } else if (rtIsInf(numStatesPerAnchor)) {
            ar = 0;
            anew = rtNaN;
            apnd = numStatesPerAnchor;
          } else {
            anew = 1.0;
            ndbl = floor((numStatesPerAnchor - 1.0) + 0.5);
            apnd = 1.0 + ndbl;
            cdiff = (1.0 + ndbl) - numStatesPerAnchor;
            if (fabs(cdiff) < 4.4408920985006262E-16 * numStatesPerAnchor) {
              ndbl++;
              apnd = numStatesPerAnchor;
            } else if (cdiff > 0.0) {
              apnd = 1.0 + (ndbl - 1.0);
            } else {
              ndbl++;
            }

            if (ndbl >= 0.0) {
              ar = (int)ndbl - 1;
            } else {
              ar = -1;
            }
          }

          i33 = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = ar + 1;
          emxEnsureCapacity((emxArray__common *)b_y, i33, (int)sizeof(double));
          if (ar + 1 > 0) {
            b_y->data[0] = anew;
            if (ar + 1 > 1) {
              b_y->data[ar] = apnd;
              idx = (ar + (ar < 0)) >> 1;
              for (k = 1; k < idx; k++) {
                b_y->data[k] = anew + (double)k;
                b_y->data[ar - k] = apnd - (double)k;
              }

              if (idx << 1 == ar) {
                b_y->data[idx] = (anew + apnd) / 2.0;
              } else {
                b_y->data[idx] = anew + (double)idx;
                b_y->data[idx + 1] = apnd - (double)idx;
              }
            }
          }

          d7 = numStates + ((1.0 + (double)anchorIdx) - 1.0) *
            numStatesPerAnchor;
          i33 = r6->size[0];
          r6->size[0] = b_y->size[1];
          emxEnsureCapacity((emxArray__common *)r6, i33, (int)sizeof(int));
          ar = b_y->size[1];
          for (i33 = 0; i33 < ar; i33++) {
            r6->data[i33] = (int)(d7 + b_y->data[b_y->size[0] * i33]);
          }

          ar = J->size[1];
          idx = r6->size[0];
          for (i33 = 0; i33 < ar; i33++) {
            for (i34 = 0; i34 < idx; i34++) {
              J->data[(r6->data[i34] + J->size[0] * i33) - 1] = 0.0;
            }
          }

          if (rtIsNaN(numStatesPerAnchor)) {
            ar = 0;
            anew = rtNaN;
            apnd = numStatesPerAnchor;
          } else if (numStatesPerAnchor < 1.0) {
            ar = -1;
            anew = 1.0;
            apnd = numStatesPerAnchor;
          } else if (rtIsInf(numStatesPerAnchor)) {
            ar = 0;
            anew = rtNaN;
            apnd = numStatesPerAnchor;
          } else {
            anew = 1.0;
            ndbl = floor((numStatesPerAnchor - 1.0) + 0.5);
            apnd = 1.0 + ndbl;
            cdiff = (1.0 + ndbl) - numStatesPerAnchor;
            if (fabs(cdiff) < 4.4408920985006262E-16 * numStatesPerAnchor) {
              ndbl++;
              apnd = numStatesPerAnchor;
            } else if (cdiff > 0.0) {
              apnd = 1.0 + (ndbl - 1.0);
            } else {
              ndbl++;
            }

            if (ndbl >= 0.0) {
              ar = (int)ndbl - 1;
            } else {
              ar = -1;
            }
          }

          i33 = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = ar + 1;
          emxEnsureCapacity((emxArray__common *)b_y, i33, (int)sizeof(double));
          if (ar + 1 > 0) {
            b_y->data[0] = anew;
            if (ar + 1 > 1) {
              b_y->data[ar] = apnd;
              idx = (ar + (ar < 0)) >> 1;
              for (k = 1; k < idx; k++) {
                b_y->data[k] = anew + (double)k;
                b_y->data[ar - k] = apnd - (double)k;
              }

              if (idx << 1 == ar) {
                b_y->data[idx] = (anew + apnd) / 2.0;
              } else {
                b_y->data[idx] = anew + (double)idx;
                b_y->data[idx + 1] = apnd - (double)idx;
              }
            }
          }

          d7 = numStates + ((1.0 + (double)anchorIdx) - 1.0) *
            numStatesPerAnchor;
          i33 = r6->size[0];
          r6->size[0] = b_y->size[1];
          emxEnsureCapacity((emxArray__common *)r6, i33, (int)sizeof(int));
          ar = b_y->size[1];
          for (i33 = 0; i33 < ar; i33++) {
            r6->data[i33] = (int)(d7 + b_y->data[b_y->size[0] * i33]) - 1;
          }

          for (i33 = 0; i33 < 3; i33++) {
            for (i34 = 0; i34 < 3; i34++) {
              J->data[r6->data[i34] + J->size[0] * i33] = iv9[i34 + 3 * i33];
            }
          }

          for (i33 = 0; i33 < 3; i33++) {
            for (i34 = 0; i34 < 3; i34++) {
              J->data[r6->data[i34] + J->size[0] * (i33 + 3)] = 0.0;
            }
          }

          ar = (int)(numStates - 6.0);
          for (i33 = 0; i33 < ar; i33++) {
            for (i34 = 0; i34 < 3; i34++) {
              J->data[r6->data[i34] + J->size[0] * (i33 + 6)] = 0.0;
            }
          }

          for (i33 = 0; i33 < 3; i33++) {
            for (i34 = 0; i34 < 3; i34++) {
              J->data[r6->data[i34 + 3] + J->size[0] * i33] = 0.0;
            }
          }

          for (i33 = 0; i33 < 3; i33++) {
            for (i34 = 0; i34 < 3; i34++) {
              J->data[r6->data[i34 + 3] + J->size[0] * (i33 + 3)] = iv9[i34 + 3 *
                i33];
            }
          }

          ar = (int)(numStates - 6.0);
          for (i33 = 0; i33 < ar; i33++) {
            for (i34 = 0; i34 < 3; i34++) {
              J->data[r6->data[i34 + 3] + J->size[0] * (i33 + 6)] = 0.0;
            }
          }

          ar = (int)numStates;
          for (i33 = 0; i33 < ar; i33++) {
            br = (int)numPointsPerAnchor;
            for (i34 = 0; i34 < br; i34++) {
              J->data[r6->data[i34 + 6] + J->size[0] * i33] = 0.0;
            }
          }

          if ((J->size[1] == 1) || (P_apr->size[0] == 1)) {
            i33 = y->size[0] * y->size[1];
            y->size[0] = J->size[0];
            y->size[1] = P_apr->size[1];
            emxEnsureCapacity((emxArray__common *)y, i33, (int)sizeof(double));
            ar = J->size[0];
            for (i33 = 0; i33 < ar; i33++) {
              br = P_apr->size[1];
              for (i34 = 0; i34 < br; i34++) {
                y->data[i33 + y->size[0] * i34] = 0.0;
                idx = J->size[1];
                for (ixstart = 0; ixstart < idx; ixstart++) {
                  y->data[i33 + y->size[0] * i34] += J->data[i33 + J->size[0] *
                    ixstart] * P_apr->data[ixstart + P_apr->size[0] * i34];
                }
              }
            }
          } else {
            k = J->size[1];
            idx = J->size[0];
            ixstart = P_apr->size[1];
            c_m = J->size[0];
            i33 = y->size[0] * y->size[1];
            y->size[0] = idx;
            emxEnsureCapacity((emxArray__common *)y, i33, (int)sizeof(double));
            i33 = y->size[0] * y->size[1];
            y->size[1] = ixstart;
            emxEnsureCapacity((emxArray__common *)y, i33, (int)sizeof(double));
            ar = idx * ixstart;
            for (i33 = 0; i33 < ar; i33++) {
              y->data[i33] = 0.0;
            }

            if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
            } else {
              idx = J->size[0] * (P_apr->size[1] - 1);
              ixstart = 0;
              while ((c_m > 0) && (ixstart <= idx)) {
                i33 = ixstart + c_m;
                for (ic = ixstart; ic + 1 <= i33; ic++) {
                  y->data[ic] = 0.0;
                }

                ixstart += c_m;
              }

              br = 0;
              ixstart = 0;
              while ((c_m > 0) && (ixstart <= idx)) {
                ar = 0;
                i33 = br + k;
                for (ib = br; ib + 1 <= i33; ib++) {
                  if (P_apr->data[ib] != 0.0) {
                    ia = ar;
                    i34 = ixstart + c_m;
                    for (ic = ixstart; ic + 1 <= i34; ic++) {
                      ia++;
                      y->data[ic] += P_apr->data[ib] * J->data[ia - 1];
                    }
                  }

                  ar += c_m;
                }

                br += k;
                ixstart += c_m;
              }
            }
          }

          i33 = b->size[0] * b->size[1];
          b->size[0] = J->size[1];
          b->size[1] = J->size[0];
          emxEnsureCapacity((emxArray__common *)b, i33, (int)sizeof(double));
          ar = J->size[0];
          for (i33 = 0; i33 < ar; i33++) {
            br = J->size[1];
            for (i34 = 0; i34 < br; i34++) {
              b->data[i34 + b->size[0] * i33] = J->data[i33 + J->size[0] * i34];
            }
          }

          if ((y->size[1] == 1) || (b->size[0] == 1)) {
            i33 = P_apr->size[0] * P_apr->size[1];
            P_apr->size[0] = y->size[0];
            P_apr->size[1] = b->size[1];
            emxEnsureCapacity((emxArray__common *)P_apr, i33, (int)sizeof(double));
            ar = y->size[0];
            for (i33 = 0; i33 < ar; i33++) {
              br = b->size[1];
              for (i34 = 0; i34 < br; i34++) {
                P_apr->data[i33 + P_apr->size[0] * i34] = 0.0;
                idx = y->size[1];
                for (ixstart = 0; ixstart < idx; ixstart++) {
                  P_apr->data[i33 + P_apr->size[0] * i34] += y->data[i33 +
                    y->size[0] * ixstart] * b->data[ixstart + b->size[0] * i34];
                }
              }
            }
          } else {
            k = y->size[1];
            idx = y->size[0];
            ixstart = b->size[1];
            c_m = y->size[0];
            i33 = P_apr->size[0] * P_apr->size[1];
            P_apr->size[0] = idx;
            P_apr->size[1] = ixstart;
            emxEnsureCapacity((emxArray__common *)P_apr, i33, (int)sizeof(double));
            for (i33 = 0; i33 < ixstart; i33++) {
              for (i34 = 0; i34 < idx; i34++) {
                P_apr->data[i34 + P_apr->size[0] * i33] = 0.0;
              }
            }

            if ((y->size[0] == 0) || (b->size[1] == 0)) {
            } else {
              idx = y->size[0] * (b->size[1] - 1);
              ixstart = 0;
              while ((c_m > 0) && (ixstart <= idx)) {
                i33 = ixstart + c_m;
                for (ic = ixstart; ic + 1 <= i33; ic++) {
                  P_apr->data[ic] = 0.0;
                }

                ixstart += c_m;
              }

              br = 0;
              ixstart = 0;
              while ((c_m > 0) && (ixstart <= idx)) {
                ar = 0;
                i33 = br + k;
                for (ib = br; ib + 1 <= i33; ib++) {
                  if (b->data[ib] != 0.0) {
                    ia = ar;
                    i34 = ixstart + c_m;
                    for (ic = ixstart; ic + 1 <= i34; ic++) {
                      ia++;
                      P_apr->data[ic] += b->data[ib] * y->data[ia - 1];
                    }
                  }

                  ar += c_m;
                }

                br += k;
                ixstart += c_m;
              }
            }
          }

          i33 = (int)numPointsPerAnchor;
          ib = 0;
          exitg4 = false;
          while ((!exitg4) && (ib <= i33 - 1)) {
            b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
              inverse_depth = 1.0 / active_feature->data[(int)new_feature_idx -
              1];
            for (i34 = 0; i34 < 3; i34++) {
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ib]
                .m[i34] = new_m->data[i34 + new_m->size[0] * ((int)
                new_feature_idx - 1)];
            }

            if (c_VIOParameters_delayed_initial) {
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                status = 2.0;
            } else {
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                status = 1.0;
            }

            b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
              status_idx = status_ind->data[(int)new_feature_idx - 1];
            b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].P_idx =
              ((numStates + ((1.0 + (double)anchorIdx) - 1.0) *
                numStatesPerAnchor) + 6.0) + (1.0 + (double)ib);
            ar = 0;
            for (k = 0; k < triangulation_success->size[0]; k++) {
              if (triangulation_success->data[k]) {
                ar++;
              }
            }

            if ((int)new_feature_idx > ar) {
              g_ros_info(b_xt->anchor_states->data[anchorIdx]
                         .feature_states->data[ib].status_idx);
              P_apr->data[((int)b_xt->anchor_states->data[anchorIdx].
                           feature_states->data[ib].P_idx + P_apr->size[0] *
                           ((int)b_xt->anchor_states->data[anchorIdx].
                            feature_states->data[ib].P_idx - 1)) - 1] =
                noiseParameters_sigmaInit * 10.0;

              //  TODO: Maybe push the mean value further away?
            } else {
              P_apr->data[((int)b_xt->anchor_states->data[anchorIdx].
                           feature_states->data[ib].P_idx + P_apr->size[0] *
                           ((int)b_xt->anchor_states->data[anchorIdx].
                            feature_states->data[ib].P_idx - 1)) - 1] =
                noiseParameters_sigmaInit;

              // *new_depths(new_feature_idx);
            }

            d7 = rt_roundd_snf(1.0 + (double)ib);
            if (d7 < 2.147483648E+9) {
              i34 = (int)d7;
            } else {
              i34 = MAX_int32_T;
            }

            d7 = rt_roundd_snf(1.0 + (double)anchorIdx);
            if (d7 < 2.147483648E+9) {
              ixstart = (int)d7;
            } else {
              ixstart = MAX_int32_T;
            }

            h_ros_info((int)status_ind->data[(int)new_feature_idx - 1], i34,
                       ixstart);
            updateVect[(int)status_ind->data[(int)new_feature_idx - 1] - 1] = 1;
            new_feature_idx++;

            //                  if new_feature_idx > length(new_depths)
            ar = 0;
            for (k = 0; k < triangulation_success->size[0]; k++) {
              if (triangulation_success->data[k]) {
                ar++;
              }
            }

            if ((double)new_feature_idx > ar) {
              exitg4 = true;
            } else {
              ib++;
            }
          }
        }

        anchorIdx++;
      }
    }

    emxFreeStruct_struct_T(&expl_temp);
    emxFree_real_T(&b_y);
    emxFree_real_T(&new_m);
    for (br = 0; br < 40; br++) {
      if (updateVect[br] == 2) {
        updateVect[br] = 0;
      }
    }

    //  remove features that were not inserted
  }

  if (c_VIOParameters_delayed_initial) {
    //  get the median uncertainty of the active features as a benchmark on
    //  the delayed features
    i32 = uncertainties->size[0];
    uncertainties->size[0] = (int)c_numTrackFeatures;
    emxEnsureCapacity((emxArray__common *)uncertainties, i32, (int)sizeof(double));
    ar = (int)c_numTrackFeatures;
    for (i32 = 0; i32 < ar; i32++) {
      uncertainties->data[i32] = -1.0;
    }

    has_active_features = false;
    for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
      for (ib = 0; ib < (int)numPointsPerAnchor; ib++) {
        if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].status
            == 1.0) {
          has_active_features = true;
          uncertainties->data[(int)b_xt->anchor_states->data[anchorIdx].
            feature_states->data[ib].status_idx - 1] = P_apr->data[((int)
            b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].P_idx
            + P_apr->size[0] * ((int)b_xt->anchor_states->data[anchorIdx].
                                feature_states->data[ib].P_idx - 1)) - 1];
        }
      }
    }

    if (has_active_features) {
      idx = uncertainties->size[0] - 1;
      ixstart = 0;
      for (br = 0; br <= idx; br++) {
        if (uncertainties->data[br] > 0.0) {
          ixstart++;
        }
      }

      i32 = r6->size[0];
      r6->size[0] = ixstart;
      emxEnsureCapacity((emxArray__common *)r6, i32, (int)sizeof(int));
      ixstart = 0;
      for (br = 0; br <= idx; br++) {
        if (uncertainties->data[br] > 0.0) {
          r6->data[ixstart] = br + 1;
          ixstart++;
        }
      }

      b_emxInit_real_T(&b_uncertainties, 1);
      i32 = b_uncertainties->size[0];
      b_uncertainties->size[0] = r6->size[0];
      emxEnsureCapacity((emxArray__common *)b_uncertainties, i32, (int)sizeof
                        (double));
      ar = r6->size[0];
      for (i32 = 0; i32 < ar; i32++) {
        b_uncertainties->data[i32] = uncertainties->data[r6->data[i32] - 1];
      }

      median_uncertainty = median(b_uncertainties);

      //  because coder does not support nanflag
      //  check if a delayed initialization feature has converged
      i32 = (int)numAnchors;
      anchorIdx = 0;
      emxFree_real_T(&b_uncertainties);
      while (anchorIdx <= i32 - 1) {
        i33 = (int)numPointsPerAnchor;
        for (ib = 0; ib < i33; ib++) {
          if ((b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
               status == 2.0) && (P_apr->data[((int)b_xt->anchor_states->
                data[anchorIdx].feature_states->data[ib].P_idx + P_apr->size[0] *
                ((int)b_xt->anchor_states->data[anchorIdx].feature_states->
                 data[ib].P_idx - 1)) - 1] < median_uncertainty * 2.0)) {
            //  this feature is not active yet
            if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                inverse_depth < 0.0) {
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                status = 0.0;
              updateVect[(int)b_xt->anchor_states->data[anchorIdx].
                feature_states->data[ib].status_idx - 1] = 0;
            } else {
              i_ros_info(b_xt->anchor_states->data[anchorIdx]
                         .feature_states->data[ib].status_idx, 1.0 + (double)ib,
                         1.0 + (double)anchorIdx);
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                status = 1.0;
            }
          }
        }

        anchorIdx++;
      }
    }

    //  check if delayed features need to be forced active due to too few active 
    //  features
    totalNumActiveFeatues = getTotalNumActiveFeatures(b_xt->anchor_states);
    if (totalNumActiveFeatues < 0.4 * c_numTrackFeatures) {
      //  find the best features and activate them
      numDelayedFeatures = getTotalNumDelayedFeatures(b_xt->anchor_states);
      i32 = status_ind->size[0];
      status_ind->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)status_ind, i32, (int)sizeof(double));
      ar = (int)numDelayedFeatures;
      for (i32 = 0; i32 < ar; i32++) {
        status_ind->data[i32] = 0.0;
      }

      //  quality measures of each delayed feature
      i32 = uncertainties->size[0];
      uncertainties->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)uncertainties, i32, (int)sizeof
                        (double));
      ar = (int)numDelayedFeatures;
      for (i32 = 0; i32 < ar; i32++) {
        uncertainties->data[i32] = 0.0;
      }

      i32 = active_feature->size[0];
      active_feature->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)active_feature, i32, (int)sizeof
                        (double));
      ar = (int)numDelayedFeatures;
      for (i32 = 0; i32 < ar; i32++) {
        active_feature->data[i32] = 0.0;
      }

      delayedIdx = 1U;
      for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
        for (ib = 0; ib < (int)numPointsPerAnchor; ib++) {
          if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
              status == 2.0) {
            status_ind->data[(int)delayedIdx - 1] = P_apr->data[((int)
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
              P_idx + P_apr->size[0] * ((int)b_xt->anchor_states->data[anchorIdx]
              .feature_states->data[ib].P_idx - 1)) - 1] /
              noiseParameters_sigmaInit;
            uncertainties->data[(int)delayedIdx - 1] = 1.0 + (double)anchorIdx;
            active_feature->data[(int)delayedIdx - 1] = 1.0 + (double)ib;
            delayedIdx++;
          }
        }
      }

      eml_sort(status_ind, iidx);
      i32 = status_ind->size[0];
      status_ind->size[0] = iidx->size[0];
      emxEnsureCapacity((emxArray__common *)status_ind, i32, (int)sizeof(double));
      ar = iidx->size[0];
      for (i32 = 0; i32 < ar; i32++) {
        status_ind->data[i32] = iidx->data[i32];
      }

      numActivatedFeatures = 0.0;
      br = 0;
      while ((br <= status_ind->size[0] - 1) && (!(numActivatedFeatures > ceil
               (0.4 * c_numTrackFeatures - totalNumActiveFeatues))) &&
             (!(numActivatedFeatures > numDelayedFeatures))) {
        if (b_xt->anchor_states->data[(int)uncertainties->data[(int)
            status_ind->data[br] - 1] - 1].feature_states->data[(int)
            active_feature->data[(int)status_ind->data[br] - 1] - 1].
            inverse_depth < 0.0) {
          b_xt->anchor_states->data[(int)uncertainties->data[(int)
            status_ind->data[br] - 1] - 1].feature_states->data[(int)
            active_feature->data[(int)status_ind->data[br] - 1] - 1].status =
            0.0;
          updateVect[(int)b_xt->anchor_states->data[(int)uncertainties->data
            [(int)status_ind->data[br] - 1] - 1].feature_states->data[(int)
            active_feature->data[(int)status_ind->data[br] - 1] - 1].status_idx
            - 1] = 0;
        } else {
          b_xt->anchor_states->data[(int)uncertainties->data[(int)
            status_ind->data[br] - 1] - 1].feature_states->data[(int)
            active_feature->data[(int)status_ind->data[br] - 1] - 1].status =
            1.0;
          j_ros_info(b_xt->anchor_states->data[(int)uncertainties->data[(int)
                     status_ind->data[br] - 1] - 1].feature_states->data[(int)
                     active_feature->data[(int)status_ind->data[br] - 1] - 1].
                     status_idx, active_feature->data[(int)status_ind->data[br]
                     - 1], uncertainties->data[(int)status_ind->data[br] - 1]);
          numActivatedFeatures++;
        }

        br++;
      }
    }
  }

  emxFree_int32_T(&iidx);
  emxFree_int32_T(&r6);
  emxFree_real_T(&active_feature);

  //  check if new features need to be requested
  request_new_features = false;
  ar = 0;
  for (br = 0; br < 40; br++) {
    if (updateVect[br] == 0) {
      ar++;
    }
  }

  if (ar > minFeatureThreshold) {
    //  if a new anchor can be filled enough
    i32 = (int)numAnchors;
    anchorIdx = 0;
    b_emxInitStruct_struct_T(&b_expl_temp);
    exitg2 = false;
    while ((!exitg2) && (anchorIdx <= i32 - 1)) {
      emxCopyStruct_struct_T(&b_expl_temp, &b_xt->anchor_states->data[anchorIdx]);
      if (getNumValidFeatures(b_expl_temp.feature_states) < minFeatureThreshold)
      {
        request_new_features = true;
        exitg2 = true;
      } else {
        anchorIdx++;
      }
    }

    emxFreeStruct_struct_T(&b_expl_temp);
  } else {
    //  debug check
  }

  if (request_new_features) {
    for (br = 0; br < 40; br++) {
      if (updateVect[br] == 0) {
        updateVect[br] = 2;
      }
    }

    //  get as many new features as possible
    k_ros_info();
  }

  // % robocentric update
  if (b_xt->origin.anchor_idx == 0.0) {
    //  need to update the origin anchor and the state
    b_xt->fixed_feature = 0.0;

    //  ensure that a new feature will be fixed, if this option is enabled
    //  choose the best anchor as the new origin anchor
    i32 = uncertainties->size[0];
    uncertainties->size[0] = (int)numAnchors;
    emxEnsureCapacity((emxArray__common *)uncertainties, i32, (int)sizeof(double));
    ar = (int)numAnchors;
    for (i32 = 0; i32 < ar; i32++) {
      uncertainties->data[i32] = 0.0;
    }

    //  uncertainties of the anchors reduced to a scalar
    i32 = status_ind->size[0];
    status_ind->size[0] = (int)numAnchors;
    emxEnsureCapacity((emxArray__common *)status_ind, i32, (int)sizeof(double));
    ar = (int)numAnchors;
    for (i32 = 0; i32 < ar; i32++) {
      status_ind->data[i32] = 0.0;
    }

    i32 = (int)numAnchors;
    anchorIdx = 0;
    b_emxInitStruct_struct_T(&c_expl_temp);
    while (anchorIdx <= i32 - 1) {
      emxCopyStruct_struct_T(&c_expl_temp, &b_xt->anchor_states->data[anchorIdx]);
      if (anyActiveAnchorFeatures(c_expl_temp.feature_states)) {
        for (i33 = 0; i33 < 6; i33++) {
          for (i34 = 0; i34 < 6; i34++) {
            c_P_apr[i34 + 6 * i33] = P_apr->data[((int)b_xt->anchor_states->
              data[anchorIdx].P_idx[i34] + P_apr->size[0] * ((int)
              b_xt->anchor_states->data[anchorIdx].P_idx[i33] - 1)) - 1];
          }
        }

        uncertainties->data[anchorIdx] = det(c_P_apr);
        status_ind->data[anchorIdx] = 1.0;
      } else {
        uncertainties->data[anchorIdx] = 1000.0;

        //  dont fix an anchor with no active features
      }

      anchorIdx++;
    }

    emxFreeStruct_struct_T(&c_expl_temp);
    if (!c_any(status_ind)) {
      //  can happen if outlier rejection rejected all features
    } else {
      ixstart = 1;
      ar = uncertainties->size[0];
      mtmp = uncertainties->data[0];
      br = 0;
      if (uncertainties->size[0] > 1) {
        if (rtIsNaN(uncertainties->data[0])) {
          idx = 1;
          exitg1 = false;
          while ((!exitg1) && (idx + 1 <= ar)) {
            ixstart = idx + 1;
            if (!rtIsNaN(uncertainties->data[idx])) {
              mtmp = uncertainties->data[idx];
              br = idx;
              exitg1 = true;
            } else {
              idx++;
            }
          }
        }

        if (ixstart < uncertainties->size[0]) {
          while (ixstart + 1 <= ar) {
            if (uncertainties->data[ixstart] < mtmp) {
              mtmp = uncertainties->data[ixstart];
              br = ixstart;
            }

            ixstart++;
          }
        }
      }

      if (!(status_ind->data[br] != 0.0)) {
        //  debug check
        b_ros_error();
      }

      b_xt->origin.anchor_idx = br + 1;
      l_ros_info((double)(br + 1));
      for (i32 = 0; i32 < 3; i32++) {
        new_origin_pos_rel[i32] = b_xt->anchor_states->data[br].pos[i32];
      }

      //  in old origin frame
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      new_origin_att_rel[0] = ((b_xt->anchor_states->data[br].att[0] *
        b_xt->anchor_states->data[br].att[0] - b_xt->anchor_states->data[br]
        .att[1] * b_xt->anchor_states->data[br].att[1]) - b_xt->
        anchor_states->data[br].att[2] * b_xt->anchor_states->data[br].att[2]) +
        b_xt->anchor_states->data[br].att[3] * b_xt->anchor_states->data[br]
        .att[3];
      new_origin_att_rel[3] = 2.0 * (b_xt->anchor_states->data[br].att[0] *
        b_xt->anchor_states->data[br].att[1] + b_xt->anchor_states->data[br]
        .att[2] * b_xt->anchor_states->data[br].att[3]);
      new_origin_att_rel[6] = 2.0 * (b_xt->anchor_states->data[br].att[0] *
        b_xt->anchor_states->data[br].att[2] - b_xt->anchor_states->data[br]
        .att[1] * b_xt->anchor_states->data[br].att[3]);
      new_origin_att_rel[1] = 2.0 * (b_xt->anchor_states->data[br].att[0] *
        b_xt->anchor_states->data[br].att[1] - b_xt->anchor_states->data[br]
        .att[2] * b_xt->anchor_states->data[br].att[3]);
      new_origin_att_rel[4] = ((-(b_xt->anchor_states->data[br].att[0] *
        b_xt->anchor_states->data[br].att[0]) + b_xt->anchor_states->data[br].
        att[1] * b_xt->anchor_states->data[br].att[1]) - b_xt->
        anchor_states->data[br].att[2] * b_xt->anchor_states->data[br].att[2]) +
        b_xt->anchor_states->data[br].att[3] * b_xt->anchor_states->data[br]
        .att[3];
      new_origin_att_rel[7] = 2.0 * (b_xt->anchor_states->data[br].att[1] *
        b_xt->anchor_states->data[br].att[2] + b_xt->anchor_states->data[br]
        .att[0] * b_xt->anchor_states->data[br].att[3]);
      new_origin_att_rel[2] = 2.0 * (b_xt->anchor_states->data[br].att[0] *
        b_xt->anchor_states->data[br].att[2] + b_xt->anchor_states->data[br]
        .att[1] * b_xt->anchor_states->data[br].att[3]);
      new_origin_att_rel[5] = 2.0 * (b_xt->anchor_states->data[br].att[1] *
        b_xt->anchor_states->data[br].att[2] - b_xt->anchor_states->data[br]
        .att[0] * b_xt->anchor_states->data[br].att[3]);
      new_origin_att_rel[8] = ((-(b_xt->anchor_states->data[br].att[0] *
        b_xt->anchor_states->data[br].att[0]) - b_xt->anchor_states->data[br].
        att[1] * b_xt->anchor_states->data[br].att[1]) + b_xt->
        anchor_states->data[br].att[2] * b_xt->anchor_states->data[br].att[2]) +
        b_xt->anchor_states->data[br].att[3] * b_xt->anchor_states->data[br]
        .att[3];

      //  in old origin frame, = R_o{k+1}o{k}
      for (i32 = 0; i32 < 2; i32++) {
        b_P_apr[i32] = P_apr->size[i32];
      }

      c_eye(b_P_apr, J);

      //  robot position and orientation
      eye(f_xt);
      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[i33 + J->size[0] * i32] = new_origin_att_rel[i33 + 3 * i32];
        }
      }

      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[i33 + J->size[0] * (i32 + 3)] = 0.0;
        }
      }

      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[(i33 + J->size[0] * i32) + 3] = 0.0;
        }
      }

      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[(i33 + J->size[0] * (i32 + 3)) + 3] = f_xt[i33 + 3 * i32];
        }
      }

      for (i32 = 0; i32 < 3; i32++) {
        c_xt[i32] = b_xt->robot_state.pos[i32] - b_xt->anchor_states->data[br].
          pos[i32];
      }

      for (i32 = 0; i32 < 3; i32++) {
        m_l[i32] = 0.0;
        for (i33 = 0; i33 < 3; i33++) {
          m_l[i32] += new_origin_att_rel[i32 + 3 * i33] * c_xt[i33];
        }
      }

      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[i33 + J->size[0] * ((int)b_xt->anchor_states->data[br]
            .P_idx[i32] - 1)] = -new_origin_att_rel[i33 + 3 * i32];
        }
      }

      J->data[J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] - 1)] =
        0.0;
      J->data[J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] - 1)] =
        -m_l[2];
      J->data[J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] - 1)] =
        m_l[1];
      J->data[1 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] - 1)]
        = m_l[2];
      J->data[1 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] - 1)]
        = 0.0;
      J->data[1 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] - 1)]
        = -m_l[0];
      J->data[2 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] - 1)]
        = -m_l[1];
      J->data[2 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] - 1)]
        = m_l[0];
      J->data[2 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] - 1)]
        = 0.0;
      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[(i33 + J->size[0] * ((int)b_xt->anchor_states->data[br]
                    .P_idx[i32] - 1)) + 3] = 0.0;
        }
      }

      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[(i33 + J->size[0] * ((int)b_xt->anchor_states->data[br]
                    .P_idx[i32 + 3] - 1)) + 3] = -new_origin_att_rel[i32 + 3 *
            i33];
        }
      }

      //  robot velocity
      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[(i33 + J->size[0] * (6 + i32)) + 6] = new_origin_att_rel[i33 +
            3 * i32];
        }
      }

      //  velocity
      //  origin rotation
      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          J->data[(i33 + J->size[0] * (15 + i32)) + 15] = new_origin_att_rel[i33
            + 3 * i32];
        }
      }

      //  origin rotation
      for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
        if (1.0 + (double)anchorIdx == br + 1) {
          //  remove yaw uncertainty, but not pitch or roll
          for (i32 = 0; i32 < 6; i32++) {
            for (i33 = 0; i33 < 6; i33++) {
              J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i33] +
                       J->size[0] * ((int)b_xt->anchor_states->data[anchorIdx].
                        P_idx[i32] - 1)) - 1] = 0.0;
            }
          }

          //  TODO: allow roll/pitch uncertainty
        } else {
          eye(f_xt);
          for (i32 = 0; i32 < 3; i32++) {
            for (i33 = 0; i33 < 3; i33++) {
              J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i33] +
                       J->size[0] * ((int)b_xt->anchor_states->data[anchorIdx].
                        P_idx[i32] - 1)) - 1] = new_origin_att_rel[i33 + 3 * i32];
            }
          }

          for (i32 = 0; i32 < 3; i32++) {
            for (i33 = 0; i33 < 3; i33++) {
              J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i33] +
                       J->size[0] * ((int)b_xt->anchor_states->data[anchorIdx].
                        P_idx[i32 + 3] - 1)) - 1] = 0.0;
            }
          }

          for (i32 = 0; i32 < 3; i32++) {
            for (i33 = 0; i33 < 3; i33++) {
              J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i33 + 3]
                       + J->size[0] * ((int)b_xt->anchor_states->data[anchorIdx]
                        .P_idx[i32] - 1)) - 1] = 0.0;
            }
          }

          for (i32 = 0; i32 < 3; i32++) {
            for (i33 = 0; i33 < 3; i33++) {
              J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i33 + 3]
                       + J->size[0] * ((int)b_xt->anchor_states->data[anchorIdx]
                        .P_idx[i32 + 3] - 1)) - 1] = f_xt[i33 + 3 * i32];
            }
          }

          for (i32 = 0; i32 < 3; i32++) {
            c_xt[i32] = b_xt->anchor_states->data[anchorIdx].pos[i32] -
              new_origin_pos_rel[i32];
          }

          for (i32 = 0; i32 < 3; i32++) {
            m_l[i32] = 0.0;
            for (i33 = 0; i33 < 3; i33++) {
              m_l[i32] += new_origin_att_rel[i32 + 3 * i33] * c_xt[i33];
            }
          }

          for (i32 = 0; i32 < 3; i32++) {
            for (i33 = 0; i33 < 3; i33++) {
              J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i33] +
                       J->size[0] * ((int)b_xt->anchor_states->data[br]
                        .P_idx[i32] - 1)) - 1] = -new_origin_att_rel[i33 + 3 *
                i32];
            }
          }

          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[0] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[3] - 1)) - 1]
            = 0.0;
          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[0] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[4] - 1)) - 1]
            = -m_l[2];
          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[0] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[5] - 1)) - 1]
            = m_l[1];
          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[1] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[3] - 1)) - 1]
            = m_l[2];
          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[1] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[4] - 1)) - 1]
            = 0.0;
          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[1] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[5] - 1)) - 1]
            = -m_l[0];
          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[2] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[3] - 1)) - 1]
            = -m_l[1];
          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[2] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[4] - 1)) - 1]
            = m_l[0];
          J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[2] + J->size
                   [0] * ((int)b_xt->anchor_states->data[br].P_idx[5] - 1)) - 1]
            = 0.0;
          for (i32 = 0; i32 < 3; i32++) {
            for (i33 = 0; i33 < 3; i33++) {
              J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i33 + 3]
                       + J->size[0] * ((int)b_xt->anchor_states->data[br]
                        .P_idx[i32] - 1)) - 1] = 0.0;
            }
          }

          for (i32 = 0; i32 < 3; i32++) {
            for (i33 = 0; i33 < 3; i33++) {
              J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i33 + 3]
                       + J->size[0] * ((int)b_xt->anchor_states->data[br]
                        .P_idx[i32 + 3] - 1)) - 1] = -new_origin_att_rel[i32 + 3
                * i33];
            }
          }
        }

        for (i32 = 0; i32 < 3; i32++) {
          m_l[i32] = b_xt->anchor_states->data[anchorIdx].pos[i32] -
            new_origin_pos_rel[i32];
        }

        for (i32 = 0; i32 < 3; i32++) {
          b_xt->anchor_states->data[anchorIdx].pos[i32] = 0.0;
        }

        for (i32 = 0; i32 < 3; i32++) {
          b_xt->anchor_states->data[anchorIdx].pos[i32] = 0.0;
          for (i33 = 0; i33 < 3; i33++) {
            b_xt->anchor_states->data[anchorIdx].pos[i32] +=
              new_origin_att_rel[i32 + 3 * i33] * m_l[i33];
          }
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        g_xt[0] = ((b_xt->anchor_states->data[anchorIdx].att[0] *
                    b_xt->anchor_states->data[anchorIdx].att[0] -
                    b_xt->anchor_states->data[anchorIdx].att[1] *
                    b_xt->anchor_states->data[anchorIdx].att[1]) -
                   b_xt->anchor_states->data[anchorIdx].att[2] *
                   b_xt->anchor_states->data[anchorIdx].att[2]) +
          b_xt->anchor_states->data[anchorIdx].att[3] * b_xt->
          anchor_states->data[anchorIdx].att[3];
        g_xt[3] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[0] *
                         b_xt->anchor_states->data[anchorIdx].att[1] +
                         b_xt->anchor_states->data[anchorIdx].att[2] *
                         b_xt->anchor_states->data[anchorIdx].att[3]);
        g_xt[6] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[0] *
                         b_xt->anchor_states->data[anchorIdx].att[2] -
                         b_xt->anchor_states->data[anchorIdx].att[1] *
                         b_xt->anchor_states->data[anchorIdx].att[3]);
        g_xt[1] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[0] *
                         b_xt->anchor_states->data[anchorIdx].att[1] -
                         b_xt->anchor_states->data[anchorIdx].att[2] *
                         b_xt->anchor_states->data[anchorIdx].att[3]);
        g_xt[4] = ((-(b_xt->anchor_states->data[anchorIdx].att[0] *
                      b_xt->anchor_states->data[anchorIdx].att[0]) +
                    b_xt->anchor_states->data[anchorIdx].att[1] *
                    b_xt->anchor_states->data[anchorIdx].att[1]) -
                   b_xt->anchor_states->data[anchorIdx].att[2] *
                   b_xt->anchor_states->data[anchorIdx].att[2]) +
          b_xt->anchor_states->data[anchorIdx].att[3] * b_xt->
          anchor_states->data[anchorIdx].att[3];
        g_xt[7] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[1] *
                         b_xt->anchor_states->data[anchorIdx].att[2] +
                         b_xt->anchor_states->data[anchorIdx].att[0] *
                         b_xt->anchor_states->data[anchorIdx].att[3]);
        g_xt[2] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[0] *
                         b_xt->anchor_states->data[anchorIdx].att[2] +
                         b_xt->anchor_states->data[anchorIdx].att[1] *
                         b_xt->anchor_states->data[anchorIdx].att[3]);
        g_xt[5] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[1] *
                         b_xt->anchor_states->data[anchorIdx].att[2] -
                         b_xt->anchor_states->data[anchorIdx].att[0] *
                         b_xt->anchor_states->data[anchorIdx].att[3]);
        g_xt[8] = ((-(b_xt->anchor_states->data[anchorIdx].att[0] *
                      b_xt->anchor_states->data[anchorIdx].att[0]) -
                    b_xt->anchor_states->data[anchorIdx].att[1] *
                    b_xt->anchor_states->data[anchorIdx].att[1]) +
                   b_xt->anchor_states->data[anchorIdx].att[2] *
                   b_xt->anchor_states->data[anchorIdx].att[2]) +
          b_xt->anchor_states->data[anchorIdx].att[3] * b_xt->
          anchor_states->data[anchorIdx].att[3];
        for (i32 = 0; i32 < 3; i32++) {
          for (i33 = 0; i33 < 3; i33++) {
            f_xt[i32 + 3 * i33] = 0.0;
            for (i34 = 0; i34 < 3; i34++) {
              f_xt[i32 + 3 * i33] += g_xt[i32 + 3 * i34] *
                new_origin_att_rel[i33 + 3 * i34];
            }
          }
        }

        QuatFromRotJ(f_xt, b_xt->anchor_states->data[anchorIdx].att);
      }

      if ((J->size[1] == 1) || (P_apr->size[0] == 1)) {
        i32 = y->size[0] * y->size[1];
        y->size[0] = J->size[0];
        y->size[1] = P_apr->size[1];
        emxEnsureCapacity((emxArray__common *)y, i32, (int)sizeof(double));
        ar = J->size[0];
        for (i32 = 0; i32 < ar; i32++) {
          br = P_apr->size[1];
          for (i33 = 0; i33 < br; i33++) {
            y->data[i32 + y->size[0] * i33] = 0.0;
            idx = J->size[1];
            for (i34 = 0; i34 < idx; i34++) {
              y->data[i32 + y->size[0] * i33] += J->data[i32 + J->size[0] * i34]
                * P_apr->data[i34 + P_apr->size[0] * i33];
            }
          }
        }
      } else {
        k = J->size[1];
        idx = J->size[0];
        ixstart = P_apr->size[1];
        c_m = J->size[0];
        i32 = y->size[0] * y->size[1];
        y->size[0] = idx;
        emxEnsureCapacity((emxArray__common *)y, i32, (int)sizeof(double));
        i32 = y->size[0] * y->size[1];
        y->size[1] = ixstart;
        emxEnsureCapacity((emxArray__common *)y, i32, (int)sizeof(double));
        ar = idx * ixstart;
        for (i32 = 0; i32 < ar; i32++) {
          y->data[i32] = 0.0;
        }

        if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
        } else {
          idx = J->size[0] * (P_apr->size[1] - 1);
          ixstart = 0;
          while ((c_m > 0) && (ixstart <= idx)) {
            i32 = ixstart + c_m;
            for (ic = ixstart; ic + 1 <= i32; ic++) {
              y->data[ic] = 0.0;
            }

            ixstart += c_m;
          }

          br = 0;
          ixstart = 0;
          while ((c_m > 0) && (ixstart <= idx)) {
            ar = 0;
            i32 = br + k;
            for (ib = br; ib + 1 <= i32; ib++) {
              if (P_apr->data[ib] != 0.0) {
                ia = ar;
                i33 = ixstart + c_m;
                for (ic = ixstart; ic + 1 <= i33; ic++) {
                  ia++;
                  y->data[ic] += P_apr->data[ib] * J->data[ia - 1];
                }
              }

              ar += c_m;
            }

            br += k;
            ixstart += c_m;
          }
        }
      }

      i32 = b->size[0] * b->size[1];
      b->size[0] = J->size[1];
      b->size[1] = J->size[0];
      emxEnsureCapacity((emxArray__common *)b, i32, (int)sizeof(double));
      ar = J->size[0];
      for (i32 = 0; i32 < ar; i32++) {
        br = J->size[1];
        for (i33 = 0; i33 < br; i33++) {
          b->data[i33 + b->size[0] * i32] = J->data[i32 + J->size[0] * i33];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i32 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = y->size[0];
        P_apr->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i32, (int)sizeof(double));
        ar = y->size[0];
        for (i32 = 0; i32 < ar; i32++) {
          br = b->size[1];
          for (i33 = 0; i33 < br; i33++) {
            P_apr->data[i32 + P_apr->size[0] * i33] = 0.0;
            idx = y->size[1];
            for (i34 = 0; i34 < idx; i34++) {
              P_apr->data[i32 + P_apr->size[0] * i33] += y->data[i32 + y->size[0]
                * i34] * b->data[i34 + b->size[0] * i33];
            }
          }
        }
      } else {
        k = y->size[1];
        idx = y->size[0];
        ixstart = b->size[1];
        c_m = y->size[0];
        i32 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = idx;
        P_apr->size[1] = ixstart;
        emxEnsureCapacity((emxArray__common *)P_apr, i32, (int)sizeof(double));
        for (i32 = 0; i32 < ixstart; i32++) {
          for (i33 = 0; i33 < idx; i33++) {
            P_apr->data[i33 + P_apr->size[0] * i32] = 0.0;
          }
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          idx = y->size[0] * (b->size[1] - 1);
          ixstart = 0;
          while ((c_m > 0) && (ixstart <= idx)) {
            i32 = ixstart + c_m;
            for (ic = ixstart; ic + 1 <= i32; ic++) {
              P_apr->data[ic] = 0.0;
            }

            ixstart += c_m;
          }

          br = 0;
          ixstart = 0;
          while ((c_m > 0) && (ixstart <= idx)) {
            ar = 0;
            i32 = br + k;
            for (ib = br; ib + 1 <= i32; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i33 = ixstart + c_m;
                for (ic = ixstart; ic + 1 <= i33; ic++) {
                  ia++;
                  P_apr->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += c_m;
            }

            br += k;
            ixstart += c_m;
          }
        }
      }

      for (i32 = 0; i32 < 3; i32++) {
        c_xt[i32] = b_xt->robot_state.pos[i32] - new_origin_pos_rel[i32];
      }

      for (i32 = 0; i32 < 3; i32++) {
        b_xt->robot_state.pos[i32] = 0.0;
        for (i33 = 0; i33 < 3; i33++) {
          b_xt->robot_state.pos[i32] += new_origin_att_rel[i32 + 3 * i33] *
            c_xt[i33];
        }
      }

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      h_xt[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
                  b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                 b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
        b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
      h_xt[3] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                       b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
      h_xt[6] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                       b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
      h_xt[1] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                       b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
      h_xt[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
                  b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                 b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
        b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
      h_xt[7] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                       b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
      h_xt[2] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                       b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
      h_xt[5] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                       b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
      h_xt[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
                  b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
                 b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
        b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          f_xt[i32 + 3 * i33] = 0.0;
          for (i34 = 0; i34 < 3; i34++) {
            f_xt[i32 + 3 * i33] += h_xt[i32 + 3 * i34] * new_origin_att_rel[i33
              + 3 * i34];
          }
        }
      }

      QuatFromRotJ(f_xt, b_xt->robot_state.att);
      for (i32 = 0; i32 < 3; i32++) {
        c_xt[i32] = 0.0;
        for (i33 = 0; i33 < 3; i33++) {
          c_xt[i32] += new_origin_att_rel[i32 + 3 * i33] * b_xt->
            robot_state.vel[i33];
        }
      }

      for (i32 = 0; i32 < 3; i32++) {
        b_xt->robot_state.vel[i32] = c_xt[i32];
      }

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      i_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->origin.att[1]
                  * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                 b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->origin.att[3];
      i_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                       b_xt->origin.att[2] * b_xt->origin.att[3]);
      i_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                       b_xt->origin.att[1] * b_xt->origin.att[3]);
      i_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                       b_xt->origin.att[2] * b_xt->origin.att[3]);
      i_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                  b_xt->origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att
                 [2] * b_xt->origin.att[2]) + b_xt->origin.att[3] *
        b_xt->origin.att[3];
      i_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                       b_xt->origin.att[0] * b_xt->origin.att[3]);
      i_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                       b_xt->origin.att[1] * b_xt->origin.att[3]);
      i_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                       b_xt->origin.att[0] * b_xt->origin.att[3]);
      i_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                  b_xt->origin.att[1] * b_xt->origin.att[1]) + b_xt->origin.att
                 [2] * b_xt->origin.att[2]) + b_xt->origin.att[3] *
        b_xt->origin.att[3];
      for (i32 = 0; i32 < 3; i32++) {
        d7 = 0.0;
        for (i33 = 0; i33 < 3; i33++) {
          d7 += i_xt[i32 + 3 * i33] * new_origin_pos_rel[i33];
        }

        b_xt->origin.pos[i32] += d7;
      }

      //  in world frame
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      j_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->origin.att[1]
                  * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                 b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->origin.att[3];
      j_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                       b_xt->origin.att[2] * b_xt->origin.att[3]);
      j_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                       b_xt->origin.att[1] * b_xt->origin.att[3]);
      j_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                       b_xt->origin.att[2] * b_xt->origin.att[3]);
      j_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                  b_xt->origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att
                 [2] * b_xt->origin.att[2]) + b_xt->origin.att[3] *
        b_xt->origin.att[3];
      j_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                       b_xt->origin.att[0] * b_xt->origin.att[3]);
      j_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                       b_xt->origin.att[1] * b_xt->origin.att[3]);
      j_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                       b_xt->origin.att[0] * b_xt->origin.att[3]);
      j_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                  b_xt->origin.att[1] * b_xt->origin.att[1]) + b_xt->origin.att
                 [2] * b_xt->origin.att[2]) + b_xt->origin.att[3] *
        b_xt->origin.att[3];
      for (i32 = 0; i32 < 3; i32++) {
        for (i33 = 0; i33 < 3; i33++) {
          f_xt[i32 + 3 * i33] = 0.0;
          for (i34 = 0; i34 < 3; i34++) {
            f_xt[i32 + 3 * i33] += new_origin_att_rel[i32 + 3 * i34] * j_xt[i34
              + 3 * i33];
          }
        }
      }

      QuatFromRotJ(f_xt, b_xt->origin.att);

      //  in world frame
    }
  }

  emxFree_real_T(&b);
  emxFree_real_T(&y);
  emxFree_real_T(&J);
  emxFree_real_T(&status_ind);
  emxFree_real_T(&uncertainties);

  // % aposteriori measurement prediction
  getMap(b_xt->origin.pos, b_xt->origin.att, b_xt->anchor_states, b_map);

  //  get map for output
  getScaledMap(b_xt);

  //  update the scaled map for measurement prediction
  i32 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(c_numTrackFeatures * 4.0);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i32, (int)sizeof(double));
  ar = (int)(c_numTrackFeatures * 4.0);
  for (i32 = 0; i32 < ar; i32++) {
    h_u_apo->data[i32] = rtNaN;
  }

  for (br = 0; br < 40; br++) {
    b_delayedStatus[br] = 0.0;
  }

  for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
    for (ib = 0; ib < (int)numPointsPerAnchor; ib++) {
      if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].status
          != 0.0) {
        c_predictMeasurementStereoDisto(b_xt->anchor_states->data[anchorIdx].
          feature_states->data[ib].scaled_map_point,
          c_cameraParams_CameraParameters, d_cameraParams_CameraParameters,
          e_cameraParams_CameraParameters, f_cameraParams_CameraParameters,
          g_cameraParams_CameraParameters, h_cameraParams_CameraParameters,
          cameraParams_r_lr, cameraParams_R_rl, h_u_l, h_u_r);
        e_xt = (b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].
                status_idx - 1.0) * 4.0;
        for (i32 = 0; i32 < 2; i32++) {
          h_u_apo->data[(int)(e_xt + (1.0 + (double)i32)) - 1] = h_u_l[i32];
        }

        for (i32 = 0; i32 < 2; i32++) {
          h_u_apo->data[(int)(e_xt + (1.0 + ((double)i32 + 2.0))) - 1] =
            h_u_r[i32];
        }

        if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ib].status
            == 2.0) {
          b_delayedStatus[(int)b_xt->anchor_states->data[anchorIdx].
            feature_states->data[ib].status_idx - 1] = 1.0;
        }
      }
    }
  }
}

//
// File trailer for SLAM_upd.cpp
//
// [EOF]
//
