//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_upd.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_upd.h"
#include "norm.h"
#include "predictMeasurementStereo.h"
#include "any.h"
#include "initializePoint.h"
#include "det.h"
#include "anyActiveAnchorFeatures.h"
#include "SLAM_emxutil.h"
#include "predictMeasurementStereoDistorted.h"
#include "getScaledMap.h"
#include "getMap.h"
#include "QuatFromRotJ.h"
#include "eye.h"
#include "getNumValidFeatures.h"
#include "ros_info.h"
#include "eml_sort.h"
#include "getTotalNumDelayedFeatures.h"
#include "getTotalNumActiveFeatures.h"
#include "median.h"
#include "OnePointRANSAC_EKF.h"
#include "ros_error.h"
#include "undistortPoint.h"
#include "multiplyIdx.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

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
//                double noiseParameters_image_noise
//                double c_noiseParameters_inv_depth_ini
//                const VIOParameters b_VIOParameters
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
              double z_all_r[80], double noiseParameters_image_noise, double
              c_noiseParameters_inv_depth_ini, const VIOParameters
              b_VIOParameters, emxArray_real_T *h_u_apo, emxArray_real_T *b_map,
              double b_delayedStatus[40])
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
  int ib;
  double b_ii_data[40];
  int ii_size[1];
  int i28;
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
  int ar;
  emxArray_real_T *uncertainties;
  emxArray_real_T *active_feature;
  emxArray_real_T *status_ind;
  emxArray_int32_T *iidx;
  boolean_T fix_new_feature;
  emxArray_real_T *J;
  emxArray_int32_T *r5;
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
  emxArray_real_T *triangulated_depths;
  emxArray_real_T *triangulated_status_ind;
  emxArray_real_T *b_triangulated_status_ind;
  emxArray_real_T *untriangulated_depths;
  emxArray_real_T *untriangulated_status_ind;
  emxArray_real_T *b_untriangulated_status_ind;
  emxArray_real_T *b_new_m;
  int i29;
  emxArray_real_T *c_new_m;
  emxArray_real_T *d_new_m;
  unsigned int new_feature_idx;
  emxArray_real_T *b_y;
  e_struct_T expl_temp;
  boolean_T exitg3;
  int k;
  double c_xt[3];
  double d_xt[4];
  double d7;
  int i30;
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
  char cv18[63];
  static const char cv19[63] = { 'P', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'w', 'i', 't', 'h', ' ', 'n', 'o',
    ' ', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e',
    's', ' ', 'a', 's', ' ', 'o', 'r', 'i', 'g', 'i', 'n', ' ', '(', 'a', 'n',
    'c', 'h', 'o', 'r', ' ', '%', 'd', ')', '\x00' };

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
    ib = 0;
  } else {
    ib = idx;
  }

  ii_size[0] = ib;
  for (i28 = 0; i28 < ib; i28++) {
    b_ii_data[i28] = ii_data[i28];
  }

  multiplyIdx(b_ii_data, ii_size, ind_l2_data, ind_l2_size);
  z_all_l_size[0] = ind_l2_size[0];
  ib = ind_l2_size[0];
  for (i28 = 0; i28 < ib; i28++) {
    z_all_l_data[i28] = z_all_l[(int)ind_l2_data[i28] - 1];
  }

  undistortPoint(z_all_l_data, z_all_l_size, c_cameraParams_CameraParameters,
                 d_cameraParams_CameraParameters,
                 e_cameraParams_CameraParameters, tmp_data, ii_size);
  ib = ii_size[0];
  for (i28 = 0; i28 < ib; i28++) {
    z_all_l[(int)ind_l2_data[i28] - 1] = tmp_data[i28];
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
    ib = 0;
  } else {
    ib = idx;
  }

  if (1 > idx) {
    ixstart = 0;
  } else {
    ixstart = idx;
  }

  ind_r_size[0] = ib;
  for (i28 = 0; i28 < ib; i28++) {
    ind_r_data[i28] = ii_data[i28];
  }

  multiplyIdx(ind_r_data, ind_r_size, ind_l2_data, ind_l2_size);
  z_all_r_size[0] = ind_l2_size[0];
  br = ind_l2_size[0];
  for (i28 = 0; i28 < br; i28++) {
    z_all_l_data[i28] = z_all_r[(int)ind_l2_data[i28] - 1];
  }

  undistortPoint(z_all_l_data, z_all_r_size, f_cameraParams_CameraParameters,
                 g_cameraParams_CameraParameters,
                 h_cameraParams_CameraParameters, tmp_data, ii_size);
  br = ii_size[0];
  for (i28 = 0; i28 < br; i28++) {
    z_all_r[(int)ind_l2_data[i28] - 1] = tmp_data[i28];
  }

  //  check for lost features
  for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
    for (ar = 0; ar < (int)numPointsPerAnchor; ar++) {
      if ((b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].status
           != 0.0) && (updateVect[(int)b_xt->anchor_states->data[anchorIdx].
                       feature_states->data[ar].status_idx - 1] != 1)) {
        br = P_apr->size[1];
        idx = (int)b_xt->anchor_states->data[anchorIdx].feature_states->data[ar]
          .P_idx;
        for (i28 = 0; i28 < br; i28++) {
          P_apr->data[(idx + P_apr->size[0] * i28) - 1] = 0.0;
        }

        br = P_apr->size[0];
        idx = (int)b_xt->anchor_states->data[anchorIdx].feature_states->data[ar]
          .P_idx;
        for (i28 = 0; i28 < br; i28++) {
          P_apr->data[i28 + P_apr->size[0] * (idx - 1)] = 0.0;
        }

        b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].status =
          0.0;
        b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].status_idx
          = 0.0;

        //                  ros_info('Lost feature %i, which was %i on anchor %i', idx, featureIdx, anchorIdx) 
      }
    }
  }

  b_emxInit_real_T(&uncertainties, 1);
  b_emxInit_real_T(&active_feature, 1);
  b_emxInit_real_T(&status_ind, 1);
  b_emxInit_int32_T(&iidx, 1);
  if (b_VIOParameters.fixed_feature) {
    fix_new_feature = false;
    if (b_xt->origin.anchor_idx != 0.0) {
      if (b_xt->fixed_feature != 0.0) {
        if (b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
            feature_states->data[(int)b_xt->fixed_feature - 1].status != 1.0) {
          fix_new_feature = true;
          ros_info(b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
                   feature_states->data[(int)b_xt->fixed_feature - 1].status_idx,
                   b_xt->fixed_feature, b_xt->origin.anchor_idx);
        }
      } else {
        fix_new_feature = true;
      }
    }

    if (fix_new_feature) {
      i28 = uncertainties->size[0];
      uncertainties->size[0] = (int)numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)uncertainties, i28, (int)sizeof
                        (double));
      br = (int)numPointsPerAnchor;
      for (i28 = 0; i28 < br; i28++) {
        uncertainties->data[i28] = 0.0;
      }

      i28 = active_feature->size[0];
      active_feature->size[0] = (int)numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)active_feature, i28, (int)sizeof
                        (double));
      br = (int)numPointsPerAnchor;
      for (i28 = 0; i28 < br; i28++) {
        active_feature->data[i28] = 0.0;
      }

      for (ar = 0; ar < (int)numPointsPerAnchor; ar++) {
        if (b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
            feature_states->data[ar].status == 1.0) {
          uncertainties->data[ar] = P_apr->data[((int)b_xt->anchor_states->data
            [(int)b_xt->origin.anchor_idx - 1].feature_states->data[ar].P_idx +
            P_apr->size[0] * ((int)b_xt->anchor_states->data[(int)
                              b_xt->origin.anchor_idx - 1].feature_states->
                              data[ar].P_idx - 1)) - 1];
          active_feature->data[ar] = 1.0;
        } else {
          uncertainties->data[ar] = 1000.0;

          //  dont fix an inactive feature
        }
      }

      eml_sort(uncertainties, iidx);
      i28 = status_ind->size[0];
      status_ind->size[0] = iidx->size[0];
      emxEnsureCapacity((emxArray__common *)status_ind, i28, (int)sizeof(double));
      br = iidx->size[0];
      for (i28 = 0; i28 < br; i28++) {
        status_ind->data[i28] = iidx->data[i28];
      }

      if (!(active_feature->data[(int)status_ind->data[0] - 1] != 0.0)) {
        ros_error();
      }

      b_xt->fixed_feature = status_ind->data[0];
      br = P_apr->size[1];
      idx = (int)b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
        feature_states->data[(int)status_ind->data[0] - 1].P_idx;
      for (i28 = 0; i28 < br; i28++) {
        P_apr->data[(idx + P_apr->size[0] * i28) - 1] = 0.0;
      }

      //  fix the feature depth
      br = P_apr->size[0];
      idx = (int)b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
        feature_states->data[(int)status_ind->data[0] - 1].P_idx;
      for (i28 = 0; i28 < br; i28++) {
        P_apr->data[i28 + P_apr->size[0] * (idx - 1)] = 0.0;
      }

      b_ros_info(b_xt->anchor_states->data[(int)b_xt->origin.anchor_idx - 1].
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
                       b_VIOParameters.max_ekf_iterations,
                       b_VIOParameters.delayed_initialization,
                       b_VIOParameters.RANSAC, updateVect);
  }

  // % Initialize new anchors/features
  emxInit_real_T(&J, 2);
  b_emxInit_int32_T(&r5, 1);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b, 2);
  if (ixstart >= minFeatureThreshold) {
    //  try to triangulate all new features
    i28 = active_feature->size[0];
    active_feature->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)active_feature, i28, (int)sizeof
                      (double));
    for (i28 = 0; i28 < ixstart; i28++) {
      active_feature->data[i28] = 0.0;
    }

    emxInit_real_T(&new_m, 2);
    i28 = new_m->size[0] * new_m->size[1];
    new_m->size[0] = 3;
    emxEnsureCapacity((emxArray__common *)new_m, i28, (int)sizeof(double));
    i28 = new_m->size[0] * new_m->size[1];
    new_m->size[1] = ib;
    emxEnsureCapacity((emxArray__common *)new_m, i28, (int)sizeof(double));
    br = 3 * ixstart;
    for (i28 = 0; i28 < br; i28++) {
      new_m->data[i28] = 0.0;
    }

    i28 = triangulation_success->size[0];
    triangulation_success->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)triangulation_success, i28, (int)
                      sizeof(boolean_T));
    for (i28 = 0; i28 < ixstart; i28++) {
      triangulation_success->data[i28] = false;
    }

    for (br = 0; br < ib; br++) {
      idx = ((int)ind_r_data[br] - 1) * 2;
      ixstart = ((int)ind_r_data[br] - 1) * 2;
      for (i28 = 0; i28 < 2; i28++) {
        z_curr_l[i28] = z_all_l[idx + i28];
        z_curr_r[i28] = z_all_r[ixstart + i28];
      }

      if (!b_VIOParameters.mono) {
        initializePoint(z_curr_l, z_curr_r, c_cameraParams_CameraParameters,
                        d_cameraParams_CameraParameters,
                        f_cameraParams_CameraParameters,
                        g_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, new_origin_pos_rel, b_m, &success);
        for (i28 = 0; i28 < 3; i28++) {
          m_l[i28] = b_m[i28];
        }

        if (success) {
          //  perform further checks
          for (ixstart = 0; ixstart < 3; ixstart++) {
            bv0[ixstart] = rtIsNaN(new_origin_pos_rel[ixstart]);
          }

          if (b_any(bv0)) {
            for (i28 = 0; i28 < 3; i28++) {
              new_origin_pos_rel[i28] = b_m[i28];
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
                  for (i28 = 0; i28 < 3; i28++) {
                    new_origin_pos_rel[i28] = b_m[i28];
                  }

                  success = false;
                }
              }
            }

            if (b_guard1) {
              //                          ros_info('Bad triangulation (reprojection error) for point %d', int8(ind_r(i))); 
              for (i28 = 0; i28 < 3; i28++) {
                new_origin_pos_rel[i28] = b_m[i28];
              }

              success = false;
            }
          }
        } else {
          for (i28 = 0; i28 < 3; i28++) {
            new_origin_pos_rel[i28] = b_m[i28];
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
      for (i28 = 0; i28 < 3; i28++) {
        new_m->data[i28 + new_m->size[0] * br] = m_l[i28];
      }

      triangulation_success->data[br] = success;
    }

    //      ros_info('successfully triangulated %d of %d features', int32(nnz(triangulation_success)), int32(length(triangulation_success))) 
    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        ixstart++;
      }
    }

    b_emxInit_real_T(&triangulated_depths, 1);
    i28 = triangulated_depths->size[0];
    triangulated_depths->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)triangulated_depths, i28, (int)sizeof
                      (double));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        triangulated_depths->data[ixstart] = active_feature->data[br];
        ixstart++;
      }
    }

    eml_sort(triangulated_depths, iidx);
    i28 = status_ind->size[0];
    status_ind->size[0] = iidx->size[0];
    emxEnsureCapacity((emxArray__common *)status_ind, i28, (int)sizeof(double));
    ib = iidx->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      status_ind->data[i28] = iidx->data[i28];
    }

    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        ixstart++;
      }
    }

    i28 = r5->size[0];
    r5->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)r5, i28, (int)sizeof(int));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        r5->data[ixstart] = br + 1;
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
    i28 = triangulated_status_ind->size[0];
    triangulated_status_ind->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)triangulated_status_ind, i28, (int)
                      sizeof(double));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (triangulation_success->data[br]) {
        triangulated_status_ind->data[ixstart] = ind_r_data[br];
        ixstart++;
      }
    }

    b_emxInit_real_T(&b_triangulated_status_ind, 1);
    i28 = b_triangulated_status_ind->size[0];
    b_triangulated_status_ind->size[0] = status_ind->size[0];
    emxEnsureCapacity((emxArray__common *)b_triangulated_status_ind, i28, (int)
                      sizeof(double));
    ib = status_ind->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      b_triangulated_status_ind->data[i28] = triangulated_status_ind->data[(int)
        status_ind->data[i28] - 1];
    }

    i28 = triangulated_status_ind->size[0];
    triangulated_status_ind->size[0] = b_triangulated_status_ind->size[0];
    emxEnsureCapacity((emxArray__common *)triangulated_status_ind, i28, (int)
                      sizeof(double));
    ib = b_triangulated_status_ind->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      triangulated_status_ind->data[i28] = b_triangulated_status_ind->data[i28];
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
    i28 = untriangulated_depths->size[0];
    untriangulated_depths->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)untriangulated_depths, i28, (int)
                      sizeof(double));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        untriangulated_depths->data[ixstart] = active_feature->data[br];
        ixstart++;
      }
    }

    eml_sort(untriangulated_depths, iidx);
    i28 = uncertainties->size[0];
    uncertainties->size[0] = iidx->size[0];
    emxEnsureCapacity((emxArray__common *)uncertainties, i28, (int)sizeof(double));
    ib = iidx->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      uncertainties->data[i28] = iidx->data[i28];
    }

    idx = triangulation_success->size[0] - 1;
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        ixstart++;
      }
    }

    i28 = iidx->size[0];
    iidx->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)iidx, i28, (int)sizeof(int));
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
    i28 = untriangulated_status_ind->size[0];
    untriangulated_status_ind->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)untriangulated_status_ind, i28, (int)
                      sizeof(double));
    ixstart = 0;
    for (br = 0; br <= idx; br++) {
      if (!triangulation_success->data[br]) {
        untriangulated_status_ind->data[ixstart] = ind_r_data[br];
        ixstart++;
      }
    }

    b_emxInit_real_T(&b_untriangulated_status_ind, 1);
    i28 = b_untriangulated_status_ind->size[0];
    b_untriangulated_status_ind->size[0] = uncertainties->size[0];
    emxEnsureCapacity((emxArray__common *)b_untriangulated_status_ind, i28, (int)
                      sizeof(double));
    ib = uncertainties->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      b_untriangulated_status_ind->data[i28] = untriangulated_status_ind->data
        [(int)uncertainties->data[i28] - 1];
    }

    i28 = untriangulated_status_ind->size[0];
    untriangulated_status_ind->size[0] = b_untriangulated_status_ind->size[0];
    emxEnsureCapacity((emxArray__common *)untriangulated_status_ind, i28, (int)
                      sizeof(double));
    ib = b_untriangulated_status_ind->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      untriangulated_status_ind->data[i28] = b_untriangulated_status_ind->
        data[i28];
    }

    emxFree_real_T(&b_untriangulated_status_ind);
    i28 = active_feature->size[0];
    active_feature->size[0] = triangulated_depths->size[0] +
      untriangulated_depths->size[0];
    emxEnsureCapacity((emxArray__common *)active_feature, i28, (int)sizeof
                      (double));
    ib = triangulated_depths->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      active_feature->data[i28] = triangulated_depths->data[i28];
    }

    ib = untriangulated_depths->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      active_feature->data[i28 + triangulated_depths->size[0]] =
        untriangulated_depths->data[i28];
    }

    emxFree_real_T(&untriangulated_depths);
    emxFree_real_T(&triangulated_depths);
    emxInit_real_T(&b_new_m, 2);
    i28 = b_new_m->size[0] * b_new_m->size[1];
    b_new_m->size[0] = 3;
    b_new_m->size[1] = r5->size[0];
    emxEnsureCapacity((emxArray__common *)b_new_m, i28, (int)sizeof(double));
    ib = r5->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      for (i29 = 0; i29 < 3; i29++) {
        b_new_m->data[i29 + b_new_m->size[0] * i28] = new_m->data[i29 +
          new_m->size[0] * (r5->data[i28] - 1)];
      }
    }

    emxInit_real_T(&c_new_m, 2);
    i28 = c_new_m->size[0] * c_new_m->size[1];
    c_new_m->size[0] = 3;
    c_new_m->size[1] = iidx->size[0];
    emxEnsureCapacity((emxArray__common *)c_new_m, i28, (int)sizeof(double));
    ib = iidx->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      for (i29 = 0; i29 < 3; i29++) {
        c_new_m->data[i29 + c_new_m->size[0] * i28] = new_m->data[i29 +
          new_m->size[0] * (iidx->data[i28] - 1)];
      }
    }

    emxInit_real_T(&d_new_m, 2);
    i28 = d_new_m->size[0] * d_new_m->size[1];
    d_new_m->size[0] = 3;
    d_new_m->size[1] = status_ind->size[0] + uncertainties->size[0];
    emxEnsureCapacity((emxArray__common *)d_new_m, i28, (int)sizeof(double));
    ib = status_ind->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      for (i29 = 0; i29 < 3; i29++) {
        d_new_m->data[i29 + d_new_m->size[0] * i28] = b_new_m->data[i29 +
          b_new_m->size[0] * ((int)status_ind->data[i28] - 1)];
      }
    }

    emxFree_real_T(&b_new_m);
    ib = uncertainties->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      for (i29 = 0; i29 < 3; i29++) {
        d_new_m->data[i29 + d_new_m->size[0] * (i28 + status_ind->size[0])] =
          c_new_m->data[i29 + c_new_m->size[0] * ((int)uncertainties->data[i28]
          - 1)];
      }
    }

    emxFree_real_T(&c_new_m);
    i28 = new_m->size[0] * new_m->size[1];
    new_m->size[0] = 3;
    new_m->size[1] = d_new_m->size[1];
    emxEnsureCapacity((emxArray__common *)new_m, i28, (int)sizeof(double));
    ib = d_new_m->size[1];
    for (i28 = 0; i28 < ib; i28++) {
      for (i29 = 0; i29 < 3; i29++) {
        new_m->data[i29 + new_m->size[0] * i28] = d_new_m->data[i29 +
          d_new_m->size[0] * i28];
      }
    }

    emxFree_real_T(&d_new_m);
    i28 = status_ind->size[0];
    status_ind->size[0] = triangulated_status_ind->size[0] +
      untriangulated_status_ind->size[0];
    emxEnsureCapacity((emxArray__common *)status_ind, i28, (int)sizeof(double));
    ib = triangulated_status_ind->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      status_ind->data[i28] = triangulated_status_ind->data[i28];
    }

    ib = untriangulated_status_ind->size[0];
    for (i28 = 0; i28 < ib; i28++) {
      status_ind->data[i28 + triangulated_status_ind->size[0]] =
        untriangulated_status_ind->data[i28];
    }

    emxFree_real_T(&untriangulated_status_ind);
    emxFree_real_T(&triangulated_status_ind);

    //  insert new features into the state. first the closest triangulated ones, 
    //  then the un-triangulated ones
    new_feature_idx = 1U;
    i28 = (int)numAnchors;
    anchorIdx = 0;
    emxInit_real_T(&b_y, 2);
    b_emxInitStruct_struct_T(&expl_temp);
    exitg3 = false;
    while ((!exitg3) && (anchorIdx <= i28 - 1)) {
      //          if new_feature_idx > length(new_depths)
      ib = 0;
      for (k = 0; k < triangulation_success->size[0]; k++) {
        if (triangulation_success->data[k]) {
          ib++;
        }
      }

      if (((double)new_feature_idx > ib) || ((active_feature->size[0] - (int)
            new_feature_idx) + 1 < minFeatureThreshold)) {
        exitg3 = true;
      } else {
        emxCopyStruct_struct_T(&expl_temp, &b_xt->anchor_states->data[anchorIdx]);
        if (getNumValidFeatures(expl_temp.feature_states) < minFeatureThreshold)
        {
          //  anchor needs to be initialized
          //  free up updateVect
          for (ar = 0; ar < (int)numPointsPerAnchor; ar++) {
            if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
                status != 0.0) {
              //                      ros_info('clearing up feature %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              updateVect[(int)b_xt->anchor_states->data[anchorIdx].
                feature_states->data[ar].status_idx - 1] = 0;
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
                status_idx = 0.0;
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
                status = 0.0;
            }
          }

          if (b_xt->origin.anchor_idx == 1.0 + (double)anchorIdx) {
            b_xt->origin.anchor_idx = 0.0;

            //                  ros_info('Initializing anchor %i, which was the origin anchor', int32(anchorIdx)) 
          }

          for (i29 = 0; i29 < 3; i29++) {
            c_xt[i29] = b_xt->robot_state.pos[i29];
          }

          for (i29 = 0; i29 < 3; i29++) {
            b_xt->anchor_states->data[anchorIdx].pos[i29] = c_xt[i29];
          }

          for (i29 = 0; i29 < 4; i29++) {
            d_xt[i29] = b_xt->robot_state.att[i29];
          }

          for (i29 = 0; i29 < 4; i29++) {
            b_xt->anchor_states->data[anchorIdx].att[i29] = d_xt[i29];
          }

          d7 = numStates + ((1.0 + (double)anchorIdx) - 1.0) *
            numStatesPerAnchor;
          for (i29 = 0; i29 < 6; i29++) {
            b_xt->anchor_states->data[anchorIdx].P_idx[i29] = d7 + (1.0 +
              (double)i29);
          }

          ib = P_apr->size[1];
          for (i29 = 0; i29 < ib; i29++) {
            for (i30 = 0; i30 < 6; i30++) {
              P_apr->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i30]
                           + P_apr->size[0] * i29) - 1] = 0.0;
            }
          }

          ib = P_apr->size[0];
          for (i29 = 0; i29 < 6; i29++) {
            for (i30 = 0; i30 < ib; i30++) {
              P_apr->data[i30 + P_apr->size[0] * ((int)b_xt->anchor_states->
                data[anchorIdx].P_idx[i29] - 1)] = 0.0;
            }
          }

          if (rtIsNaN(numPointsPerAnchor)) {
            ib = 0;
            anew = rtNaN;
            apnd = numPointsPerAnchor;
          } else if (numPointsPerAnchor < 1.0) {
            ib = -1;
            anew = 1.0;
            apnd = numPointsPerAnchor;
          } else if (rtIsInf(numPointsPerAnchor)) {
            ib = 0;
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
              ib = (int)ndbl - 1;
            } else {
              ib = -1;
            }
          }

          i29 = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = ib + 1;
          emxEnsureCapacity((emxArray__common *)b_y, i29, (int)sizeof(double));
          if (ib + 1 > 0) {
            b_y->data[0] = anew;
            if (ib + 1 > 1) {
              b_y->data[ib] = apnd;
              idx = (ib + (ib < 0)) >> 1;
              for (k = 1; k < idx; k++) {
                b_y->data[k] = anew + (double)k;
                b_y->data[ib - k] = apnd - (double)k;
              }

              if (idx << 1 == ib) {
                b_y->data[idx] = (anew + apnd) / 2.0;
              } else {
                b_y->data[idx] = anew + (double)idx;
                b_y->data[idx + 1] = apnd - (double)idx;
              }
            }
          }

          e_xt = b_xt->anchor_states->data[anchorIdx].P_idx[5];
          i29 = r5->size[0];
          r5->size[0] = b_y->size[1];
          emxEnsureCapacity((emxArray__common *)r5, i29, (int)sizeof(int));
          ib = b_y->size[1];
          for (i29 = 0; i29 < ib; i29++) {
            r5->data[i29] = (int)(e_xt + b_y->data[b_y->size[0] * i29]);
          }

          ib = P_apr->size[1];
          idx = r5->size[0];
          for (i29 = 0; i29 < ib; i29++) {
            for (i30 = 0; i30 < idx; i30++) {
              P_apr->data[(r5->data[i30] + P_apr->size[0] * i29) - 1] = 0.0;
            }
          }

          if (rtIsNaN(numPointsPerAnchor)) {
            ib = 0;
            anew = rtNaN;
            apnd = numPointsPerAnchor;
          } else if (numPointsPerAnchor < 1.0) {
            ib = -1;
            anew = 1.0;
            apnd = numPointsPerAnchor;
          } else if (rtIsInf(numPointsPerAnchor)) {
            ib = 0;
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
              ib = (int)ndbl - 1;
            } else {
              ib = -1;
            }
          }

          i29 = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = ib + 1;
          emxEnsureCapacity((emxArray__common *)b_y, i29, (int)sizeof(double));
          if (ib + 1 > 0) {
            b_y->data[0] = anew;
            if (ib + 1 > 1) {
              b_y->data[ib] = apnd;
              idx = (ib + (ib < 0)) >> 1;
              for (k = 1; k < idx; k++) {
                b_y->data[k] = anew + (double)k;
                b_y->data[ib - k] = apnd - (double)k;
              }

              if (idx << 1 == ib) {
                b_y->data[idx] = (anew + apnd) / 2.0;
              } else {
                b_y->data[idx] = anew + (double)idx;
                b_y->data[idx + 1] = apnd - (double)idx;
              }
            }
          }

          e_xt = b_xt->anchor_states->data[anchorIdx].P_idx[5];
          i29 = r5->size[0];
          r5->size[0] = b_y->size[1];
          emxEnsureCapacity((emxArray__common *)r5, i29, (int)sizeof(int));
          ib = b_y->size[1];
          for (i29 = 0; i29 < ib; i29++) {
            r5->data[i29] = (int)(e_xt + b_y->data[b_y->size[0] * i29]);
          }

          ib = P_apr->size[0];
          ixstart = r5->size[0];
          for (i29 = 0; i29 < ixstart; i29++) {
            for (i30 = 0; i30 < ib; i30++) {
              P_apr->data[i30 + P_apr->size[0] * (r5->data[i29] - 1)] = 0.0;
            }
          }

          for (i29 = 0; i29 < 2; i29++) {
            b_P_apr[i29] = P_apr->size[i29];
          }

          c_eye(b_P_apr, J);
          if (rtIsNaN(numStatesPerAnchor)) {
            ib = 0;
            anew = rtNaN;
            apnd = numStatesPerAnchor;
          } else if (numStatesPerAnchor < 1.0) {
            ib = -1;
            anew = 1.0;
            apnd = numStatesPerAnchor;
          } else if (rtIsInf(numStatesPerAnchor)) {
            ib = 0;
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
              ib = (int)ndbl - 1;
            } else {
              ib = -1;
            }
          }

          i29 = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = ib + 1;
          emxEnsureCapacity((emxArray__common *)b_y, i29, (int)sizeof(double));
          if (ib + 1 > 0) {
            b_y->data[0] = anew;
            if (ib + 1 > 1) {
              b_y->data[ib] = apnd;
              idx = (ib + (ib < 0)) >> 1;
              for (k = 1; k < idx; k++) {
                b_y->data[k] = anew + (double)k;
                b_y->data[ib - k] = apnd - (double)k;
              }

              if (idx << 1 == ib) {
                b_y->data[idx] = (anew + apnd) / 2.0;
              } else {
                b_y->data[idx] = anew + (double)idx;
                b_y->data[idx + 1] = apnd - (double)idx;
              }
            }
          }

          d7 = numStates + ((1.0 + (double)anchorIdx) - 1.0) *
            numStatesPerAnchor;
          i29 = r5->size[0];
          r5->size[0] = b_y->size[1];
          emxEnsureCapacity((emxArray__common *)r5, i29, (int)sizeof(int));
          ib = b_y->size[1];
          for (i29 = 0; i29 < ib; i29++) {
            r5->data[i29] = (int)(d7 + b_y->data[b_y->size[0] * i29]);
          }

          ib = J->size[1];
          idx = r5->size[0];
          for (i29 = 0; i29 < ib; i29++) {
            for (i30 = 0; i30 < idx; i30++) {
              J->data[(r5->data[i30] + J->size[0] * i29) - 1] = 0.0;
            }
          }

          if (rtIsNaN(numStatesPerAnchor)) {
            ib = 0;
            anew = rtNaN;
            apnd = numStatesPerAnchor;
          } else if (numStatesPerAnchor < 1.0) {
            ib = -1;
            anew = 1.0;
            apnd = numStatesPerAnchor;
          } else if (rtIsInf(numStatesPerAnchor)) {
            ib = 0;
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
              ib = (int)ndbl - 1;
            } else {
              ib = -1;
            }
          }

          i29 = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = ib + 1;
          emxEnsureCapacity((emxArray__common *)b_y, i29, (int)sizeof(double));
          if (ib + 1 > 0) {
            b_y->data[0] = anew;
            if (ib + 1 > 1) {
              b_y->data[ib] = apnd;
              idx = (ib + (ib < 0)) >> 1;
              for (k = 1; k < idx; k++) {
                b_y->data[k] = anew + (double)k;
                b_y->data[ib - k] = apnd - (double)k;
              }

              if (idx << 1 == ib) {
                b_y->data[idx] = (anew + apnd) / 2.0;
              } else {
                b_y->data[idx] = anew + (double)idx;
                b_y->data[idx + 1] = apnd - (double)idx;
              }
            }
          }

          d7 = numStates + ((1.0 + (double)anchorIdx) - 1.0) *
            numStatesPerAnchor;
          i29 = r5->size[0];
          r5->size[0] = b_y->size[1];
          emxEnsureCapacity((emxArray__common *)r5, i29, (int)sizeof(int));
          ib = b_y->size[1];
          for (i29 = 0; i29 < ib; i29++) {
            r5->data[i29] = (int)(d7 + b_y->data[b_y->size[0] * i29]) - 1;
          }

          for (i29 = 0; i29 < 3; i29++) {
            for (i30 = 0; i30 < 3; i30++) {
              J->data[r5->data[i30] + J->size[0] * i29] = iv9[i30 + 3 * i29];
            }
          }

          for (i29 = 0; i29 < 3; i29++) {
            for (i30 = 0; i30 < 3; i30++) {
              J->data[r5->data[i30] + J->size[0] * (i29 + 3)] = 0.0;
            }
          }

          ib = (int)(numStates - 6.0);
          for (i29 = 0; i29 < ib; i29++) {
            for (i30 = 0; i30 < 3; i30++) {
              J->data[r5->data[i30] + J->size[0] * (i29 + 6)] = 0.0;
            }
          }

          for (i29 = 0; i29 < 3; i29++) {
            for (i30 = 0; i30 < 3; i30++) {
              J->data[r5->data[i30 + 3] + J->size[0] * i29] = 0.0;
            }
          }

          for (i29 = 0; i29 < 3; i29++) {
            for (i30 = 0; i30 < 3; i30++) {
              J->data[r5->data[i30 + 3] + J->size[0] * (i29 + 3)] = iv9[i30 + 3 *
                i29];
            }
          }

          ib = (int)(numStates - 6.0);
          for (i29 = 0; i29 < ib; i29++) {
            for (i30 = 0; i30 < 3; i30++) {
              J->data[r5->data[i30 + 3] + J->size[0] * (i29 + 6)] = 0.0;
            }
          }

          ib = (int)numStates;
          for (i29 = 0; i29 < ib; i29++) {
            br = (int)numPointsPerAnchor;
            for (i30 = 0; i30 < br; i30++) {
              J->data[r5->data[i30 + 6] + J->size[0] * i29] = 0.0;
            }
          }

          if ((J->size[1] == 1) || (P_apr->size[0] == 1)) {
            i29 = y->size[0] * y->size[1];
            y->size[0] = J->size[0];
            y->size[1] = P_apr->size[1];
            emxEnsureCapacity((emxArray__common *)y, i29, (int)sizeof(double));
            ib = J->size[0];
            for (i29 = 0; i29 < ib; i29++) {
              br = P_apr->size[1];
              for (i30 = 0; i30 < br; i30++) {
                y->data[i29 + y->size[0] * i30] = 0.0;
                idx = J->size[1];
                for (ixstart = 0; ixstart < idx; ixstart++) {
                  y->data[i29 + y->size[0] * i30] += J->data[i29 + J->size[0] *
                    ixstart] * P_apr->data[ixstart + P_apr->size[0] * i30];
                }
              }
            }
          } else {
            k = J->size[1];
            idx = J->size[0];
            ixstart = P_apr->size[1];
            c_m = J->size[0];
            i29 = y->size[0] * y->size[1];
            y->size[0] = idx;
            emxEnsureCapacity((emxArray__common *)y, i29, (int)sizeof(double));
            i29 = y->size[0] * y->size[1];
            y->size[1] = ixstart;
            emxEnsureCapacity((emxArray__common *)y, i29, (int)sizeof(double));
            ib = idx * ixstart;
            for (i29 = 0; i29 < ib; i29++) {
              y->data[i29] = 0.0;
            }

            if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
            } else {
              idx = J->size[0] * (P_apr->size[1] - 1);
              ixstart = 0;
              while ((c_m > 0) && (ixstart <= idx)) {
                i29 = ixstart + c_m;
                for (ic = ixstart; ic + 1 <= i29; ic++) {
                  y->data[ic] = 0.0;
                }

                ixstart += c_m;
              }

              br = 0;
              ixstart = 0;
              while ((c_m > 0) && (ixstart <= idx)) {
                ar = 0;
                i29 = br + k;
                for (ib = br; ib + 1 <= i29; ib++) {
                  if (P_apr->data[ib] != 0.0) {
                    ia = ar;
                    i30 = ixstart + c_m;
                    for (ic = ixstart; ic + 1 <= i30; ic++) {
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

          i29 = b->size[0] * b->size[1];
          b->size[0] = J->size[1];
          b->size[1] = J->size[0];
          emxEnsureCapacity((emxArray__common *)b, i29, (int)sizeof(double));
          ib = J->size[0];
          for (i29 = 0; i29 < ib; i29++) {
            br = J->size[1];
            for (i30 = 0; i30 < br; i30++) {
              b->data[i30 + b->size[0] * i29] = J->data[i29 + J->size[0] * i30];
            }
          }

          if ((y->size[1] == 1) || (b->size[0] == 1)) {
            i29 = P_apr->size[0] * P_apr->size[1];
            P_apr->size[0] = y->size[0];
            P_apr->size[1] = b->size[1];
            emxEnsureCapacity((emxArray__common *)P_apr, i29, (int)sizeof(double));
            ib = y->size[0];
            for (i29 = 0; i29 < ib; i29++) {
              br = b->size[1];
              for (i30 = 0; i30 < br; i30++) {
                P_apr->data[i29 + P_apr->size[0] * i30] = 0.0;
                idx = y->size[1];
                for (ixstart = 0; ixstart < idx; ixstart++) {
                  P_apr->data[i29 + P_apr->size[0] * i30] += y->data[i29 +
                    y->size[0] * ixstart] * b->data[ixstart + b->size[0] * i30];
                }
              }
            }
          } else {
            k = y->size[1];
            idx = y->size[0];
            ixstart = b->size[1];
            c_m = y->size[0];
            i29 = P_apr->size[0] * P_apr->size[1];
            P_apr->size[0] = idx;
            P_apr->size[1] = ixstart;
            emxEnsureCapacity((emxArray__common *)P_apr, i29, (int)sizeof(double));
            for (i29 = 0; i29 < ixstart; i29++) {
              for (i30 = 0; i30 < idx; i30++) {
                P_apr->data[i30 + P_apr->size[0] * i29] = 0.0;
              }
            }

            if ((y->size[0] == 0) || (b->size[1] == 0)) {
            } else {
              idx = y->size[0] * (b->size[1] - 1);
              ixstart = 0;
              while ((c_m > 0) && (ixstart <= idx)) {
                i29 = ixstart + c_m;
                for (ic = ixstart; ic + 1 <= i29; ic++) {
                  P_apr->data[ic] = 0.0;
                }

                ixstart += c_m;
              }

              br = 0;
              ixstart = 0;
              while ((c_m > 0) && (ixstart <= idx)) {
                ar = 0;
                i29 = br + k;
                for (ib = br; ib + 1 <= i29; ib++) {
                  if (b->data[ib] != 0.0) {
                    ia = ar;
                    i30 = ixstart + c_m;
                    for (ic = ixstart; ic + 1 <= i30; ic++) {
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

          i29 = (int)numPointsPerAnchor;
          ar = 0;
          exitg4 = false;
          while ((!exitg4) && (ar <= i29 - 1)) {
            b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
              inverse_depth = 1.0 / active_feature->data[(int)new_feature_idx -
              1];
            for (i30 = 0; i30 < 3; i30++) {
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ar]
                .m[i30] = new_m->data[i30 + new_m->size[0] * ((int)
                new_feature_idx - 1)];
            }

            if (b_VIOParameters.delayed_initialization) {
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
                status = 2.0;
            } else {
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
                status = 1.0;
            }

            b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
              status_idx = status_ind->data[(int)new_feature_idx - 1];
            b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].P_idx =
              ((numStates + ((1.0 + (double)anchorIdx) - 1.0) *
                numStatesPerAnchor) + 6.0) + (1.0 + (double)ar);
            ib = 0;
            for (k = 0; k < triangulation_success->size[0]; k++) {
              if (triangulation_success->data[k]) {
                ib++;
              }
            }

            if ((int)new_feature_idx > ib) {
              c_ros_info(b_xt->anchor_states->data[anchorIdx]
                         .feature_states->data[ar].status_idx);
              P_apr->data[((int)b_xt->anchor_states->data[anchorIdx].
                           feature_states->data[ar].P_idx + P_apr->size[0] *
                           ((int)b_xt->anchor_states->data[anchorIdx].
                            feature_states->data[ar].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini * 10.0;

              //  TODO: Maybe push the mean value further away?
            } else {
              P_apr->data[((int)b_xt->anchor_states->data[anchorIdx].
                           feature_states->data[ar].P_idx + P_apr->size[0] *
                           ((int)b_xt->anchor_states->data[anchorIdx].
                            feature_states->data[ar].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini;

              // *new_depths(new_feature_idx);
            }

            //                  ros_info('Inserting feature %d as feature %i on anchor %i', int32(status_ind(new_feature_idx)), int32(featureIdx), int32(anchorIdx)) 
            updateVect[(int)status_ind->data[(int)new_feature_idx - 1] - 1] = 1;
            new_feature_idx++;

            //                  if new_feature_idx > length(new_depths)
            ib = 0;
            for (k = 0; k < triangulation_success->size[0]; k++) {
              if (triangulation_success->data[k]) {
                ib++;
              }
            }

            if ((double)new_feature_idx > ib) {
              exitg4 = true;
            } else {
              ar++;
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

  if (b_VIOParameters.delayed_initialization) {
    //  get the median uncertainty of the active features as a benchmark on
    //  the delayed features
    i28 = uncertainties->size[0];
    uncertainties->size[0] = (int)c_numTrackFeatures;
    emxEnsureCapacity((emxArray__common *)uncertainties, i28, (int)sizeof(double));
    ib = (int)c_numTrackFeatures;
    for (i28 = 0; i28 < ib; i28++) {
      uncertainties->data[i28] = -1.0;
    }

    has_active_features = false;
    for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
      for (ar = 0; ar < (int)numPointsPerAnchor; ar++) {
        if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].status
            == 1.0) {
          has_active_features = true;
          uncertainties->data[(int)b_xt->anchor_states->data[anchorIdx].
            feature_states->data[ar].status_idx - 1] = P_apr->data[((int)
            b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].P_idx
            + P_apr->size[0] * ((int)b_xt->anchor_states->data[anchorIdx].
                                feature_states->data[ar].P_idx - 1)) - 1];
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

      i28 = r5->size[0];
      r5->size[0] = ixstart;
      emxEnsureCapacity((emxArray__common *)r5, i28, (int)sizeof(int));
      ixstart = 0;
      for (br = 0; br <= idx; br++) {
        if (uncertainties->data[br] > 0.0) {
          r5->data[ixstart] = br + 1;
          ixstart++;
        }
      }

      b_emxInit_real_T(&b_uncertainties, 1);
      i28 = b_uncertainties->size[0];
      b_uncertainties->size[0] = r5->size[0];
      emxEnsureCapacity((emxArray__common *)b_uncertainties, i28, (int)sizeof
                        (double));
      ib = r5->size[0];
      for (i28 = 0; i28 < ib; i28++) {
        b_uncertainties->data[i28] = uncertainties->data[r5->data[i28] - 1];
      }

      median_uncertainty = median(b_uncertainties);

      //  because coder does not support nanflag
      //  check if a delayed initialization feature has converged
      anchorIdx = 0;
      emxFree_real_T(&b_uncertainties);
      while (anchorIdx <= (int)numAnchors - 1) {
        for (ar = 0; ar < (int)numPointsPerAnchor; ar++) {
          if ((b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
               status == 2.0) && (P_apr->data[((int)b_xt->anchor_states->
                data[anchorIdx].feature_states->data[ar].P_idx + P_apr->size[0] *
                ((int)b_xt->anchor_states->data[anchorIdx].feature_states->
                 data[ar].P_idx - 1)) - 1] < median_uncertainty * 2.0)) {
            //  this feature is not active yet
            if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
                inverse_depth < 0.0) {
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
                status = 0.0;
              updateVect[(int)b_xt->anchor_states->data[anchorIdx].
                feature_states->data[ar].status_idx - 1] = 0;
            } else {
              //                              ros_info('Feature %i (%i on anchor %i) has converged', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx); 
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
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
      i28 = status_ind->size[0];
      status_ind->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)status_ind, i28, (int)sizeof(double));
      ib = (int)numDelayedFeatures;
      for (i28 = 0; i28 < ib; i28++) {
        status_ind->data[i28] = 0.0;
      }

      //  quality measures of each delayed feature
      i28 = uncertainties->size[0];
      uncertainties->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)uncertainties, i28, (int)sizeof
                        (double));
      ib = (int)numDelayedFeatures;
      for (i28 = 0; i28 < ib; i28++) {
        uncertainties->data[i28] = 0.0;
      }

      i28 = active_feature->size[0];
      active_feature->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)active_feature, i28, (int)sizeof
                        (double));
      ib = (int)numDelayedFeatures;
      for (i28 = 0; i28 < ib; i28++) {
        active_feature->data[i28] = 0.0;
      }

      delayedIdx = 1U;
      for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
        for (ar = 0; ar < (int)numPointsPerAnchor; ar++) {
          if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
              status == 2.0) {
            status_ind->data[(int)delayedIdx - 1] = P_apr->data[((int)
              b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
              P_idx + P_apr->size[0] * ((int)b_xt->anchor_states->data[anchorIdx]
              .feature_states->data[ar].P_idx - 1)) - 1] /
              c_noiseParameters_inv_depth_ini;
            uncertainties->data[(int)delayedIdx - 1] = 1.0 + (double)anchorIdx;
            active_feature->data[(int)delayedIdx - 1] = 1.0 + (double)ar;
            delayedIdx++;
          }
        }
      }

      eml_sort(status_ind, iidx);
      i28 = status_ind->size[0];
      status_ind->size[0] = iidx->size[0];
      emxEnsureCapacity((emxArray__common *)status_ind, i28, (int)sizeof(double));
      ib = iidx->size[0];
      for (i28 = 0; i28 < ib; i28++) {
        status_ind->data[i28] = iidx->data[i28];
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
          d_ros_info(b_xt->anchor_states->data[(int)uncertainties->data[(int)
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
  emxFree_int32_T(&r5);
  emxFree_real_T(&active_feature);

  //  check if new features need to be requested
  request_new_features = false;
  ib = 0;
  for (br = 0; br < 40; br++) {
    if (updateVect[br] == 0) {
      ib++;
    }
  }

  if (ib > minFeatureThreshold) {
    //  if a new anchor can be filled enough
    i28 = (int)numAnchors;
    anchorIdx = 0;
    b_emxInitStruct_struct_T(&b_expl_temp);
    exitg2 = false;
    while ((!exitg2) && (anchorIdx <= i28 - 1)) {
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
  }

  // % robocentric update
  if (b_xt->origin.anchor_idx == 0.0) {
    //  need to update the origin anchor and the state
    b_xt->fixed_feature = 0.0;

    //  ensure that a new feature will be fixed, if this option is enabled
    //  choose the best anchor as the new origin anchor
    i28 = uncertainties->size[0];
    uncertainties->size[0] = (int)numAnchors;
    emxEnsureCapacity((emxArray__common *)uncertainties, i28, (int)sizeof(double));
    ib = (int)numAnchors;
    for (i28 = 0; i28 < ib; i28++) {
      uncertainties->data[i28] = 0.0;
    }

    //  uncertainties of the anchors reduced to a scalar
    i28 = status_ind->size[0];
    status_ind->size[0] = (int)numAnchors;
    emxEnsureCapacity((emxArray__common *)status_ind, i28, (int)sizeof(double));
    ib = (int)numAnchors;
    for (i28 = 0; i28 < ib; i28++) {
      status_ind->data[i28] = 0.0;
    }

    i28 = (int)numAnchors;
    anchorIdx = 0;
    b_emxInitStruct_struct_T(&c_expl_temp);
    while (anchorIdx <= i28 - 1) {
      emxCopyStruct_struct_T(&c_expl_temp, &b_xt->anchor_states->data[anchorIdx]);
      if (anyActiveAnchorFeatures(c_expl_temp.feature_states)) {
        for (i29 = 0; i29 < 6; i29++) {
          for (i30 = 0; i30 < 6; i30++) {
            c_P_apr[i30 + 6 * i29] = P_apr->data[((int)b_xt->anchor_states->
              data[anchorIdx].P_idx[i30] + P_apr->size[0] * ((int)
              b_xt->anchor_states->data[anchorIdx].P_idx[i29] - 1)) - 1];
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
      ib = uncertainties->size[0];
      mtmp = uncertainties->data[0];
      br = 0;
      if (uncertainties->size[0] > 1) {
        if (rtIsNaN(uncertainties->data[0])) {
          idx = 1;
          exitg1 = false;
          while ((!exitg1) && (idx + 1 <= ib)) {
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
          while (ixstart + 1 <= ib) {
            if (uncertainties->data[ixstart] < mtmp) {
              mtmp = uncertainties->data[ixstart];
              br = ixstart;
            }

            ixstart++;
          }
        }
      }

      b_xt->origin.anchor_idx = br + 1;
      if (!(status_ind->data[br] != 0.0)) {
        //  debug check
        // #coder
        // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
        for (i28 = 0; i28 < 63; i28++) {
          cv18[i28] = cv19[i28];
        }

        ROS_ERROR(cv18, br + 1);
      } else {
        //          ros_info('Setting anchor %i as origin', int32(xt.origin.anchor_idx)) 
        for (i28 = 0; i28 < 3; i28++) {
          new_origin_pos_rel[i28] = b_xt->anchor_states->data[br].pos[i28];
        }

        //  in old origin frame
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        new_origin_att_rel[0] = ((b_xt->anchor_states->data[br].att[0] *
          b_xt->anchor_states->data[br].att[0] - b_xt->anchor_states->data[br].
          att[1] * b_xt->anchor_states->data[br].att[1]) - b_xt->
          anchor_states->data[br].att[2] * b_xt->anchor_states->data[br].att[2])
          + b_xt->anchor_states->data[br].att[3] * b_xt->anchor_states->data[br]
          .att[3];
        new_origin_att_rel[3] = 2.0 * (b_xt->anchor_states->data[br].att[0] *
          b_xt->anchor_states->data[br].att[1] + b_xt->anchor_states->data[br].
          att[2] * b_xt->anchor_states->data[br].att[3]);
        new_origin_att_rel[6] = 2.0 * (b_xt->anchor_states->data[br].att[0] *
          b_xt->anchor_states->data[br].att[2] - b_xt->anchor_states->data[br].
          att[1] * b_xt->anchor_states->data[br].att[3]);
        new_origin_att_rel[1] = 2.0 * (b_xt->anchor_states->data[br].att[0] *
          b_xt->anchor_states->data[br].att[1] - b_xt->anchor_states->data[br].
          att[2] * b_xt->anchor_states->data[br].att[3]);
        new_origin_att_rel[4] = ((-(b_xt->anchor_states->data[br].att[0] *
          b_xt->anchor_states->data[br].att[0]) + b_xt->anchor_states->data[br].
          att[1] * b_xt->anchor_states->data[br].att[1]) - b_xt->
          anchor_states->data[br].att[2] * b_xt->anchor_states->data[br].att[2])
          + b_xt->anchor_states->data[br].att[3] * b_xt->anchor_states->data[br]
          .att[3];
        new_origin_att_rel[7] = 2.0 * (b_xt->anchor_states->data[br].att[1] *
          b_xt->anchor_states->data[br].att[2] + b_xt->anchor_states->data[br].
          att[0] * b_xt->anchor_states->data[br].att[3]);
        new_origin_att_rel[2] = 2.0 * (b_xt->anchor_states->data[br].att[0] *
          b_xt->anchor_states->data[br].att[2] + b_xt->anchor_states->data[br].
          att[1] * b_xt->anchor_states->data[br].att[3]);
        new_origin_att_rel[5] = 2.0 * (b_xt->anchor_states->data[br].att[1] *
          b_xt->anchor_states->data[br].att[2] - b_xt->anchor_states->data[br].
          att[0] * b_xt->anchor_states->data[br].att[3]);
        new_origin_att_rel[8] = ((-(b_xt->anchor_states->data[br].att[0] *
          b_xt->anchor_states->data[br].att[0]) - b_xt->anchor_states->data[br].
          att[1] * b_xt->anchor_states->data[br].att[1]) + b_xt->
          anchor_states->data[br].att[2] * b_xt->anchor_states->data[br].att[2])
          + b_xt->anchor_states->data[br].att[3] * b_xt->anchor_states->data[br]
          .att[3];

        //  in old origin frame, = R_o{k+1}o{k}
        for (i28 = 0; i28 < 2; i28++) {
          b_P_apr[i28] = P_apr->size[i28];
        }

        c_eye(b_P_apr, J);

        //  robot position and orientation
        eye(f_xt);
        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[i29 + J->size[0] * i28] = new_origin_att_rel[i29 + 3 * i28];
          }
        }

        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[i29 + J->size[0] * (i28 + 3)] = 0.0;
          }
        }

        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[(i29 + J->size[0] * i28) + 3] = 0.0;
          }
        }

        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[(i29 + J->size[0] * (i28 + 3)) + 3] = f_xt[i29 + 3 * i28];
          }
        }

        for (i28 = 0; i28 < 3; i28++) {
          c_xt[i28] = b_xt->robot_state.pos[i28] - b_xt->anchor_states->data[br]
            .pos[i28];
        }

        for (i28 = 0; i28 < 3; i28++) {
          m_l[i28] = 0.0;
          for (i29 = 0; i29 < 3; i29++) {
            m_l[i28] += new_origin_att_rel[i28 + 3 * i29] * c_xt[i29];
          }
        }

        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[i29 + J->size[0] * ((int)b_xt->anchor_states->data[br]
              .P_idx[i28] - 1)] = -new_origin_att_rel[i29 + 3 * i28];
          }
        }

        J->data[J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] - 1)] =
          0.0;
        J->data[J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] - 1)] =
          -m_l[2];
        J->data[J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] - 1)] =
          m_l[1];
        J->data[1 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] -
          1)] = m_l[2];
        J->data[1 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] -
          1)] = 0.0;
        J->data[1 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] -
          1)] = -m_l[0];
        J->data[2 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] -
          1)] = -m_l[1];
        J->data[2 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] -
          1)] = m_l[0];
        J->data[2 + J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] -
          1)] = 0.0;
        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[(i29 + J->size[0] * ((int)b_xt->anchor_states->data[br].
                      P_idx[i28] - 1)) + 3] = 0.0;
          }
        }

        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[(i29 + J->size[0] * ((int)b_xt->anchor_states->data[br].
                      P_idx[i28 + 3] - 1)) + 3] = -new_origin_att_rel[i28 + 3 *
              i29];
          }
        }

        //  robot velocity
        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[(i29 + J->size[0] * (6 + i28)) + 6] = new_origin_att_rel[i29
              + 3 * i28];
          }
        }

        //  velocity
        //  origin rotation
        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            J->data[(i29 + J->size[0] * (15 + i28)) + 15] =
              new_origin_att_rel[i29 + 3 * i28];
          }
        }

        //  origin rotation
        for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
          if (1.0 + (double)anchorIdx == br + 1) {
            //  remove yaw uncertainty, but not pitch or roll
            for (i28 = 0; i28 < 6; i28++) {
              for (i29 = 0; i29 < 6; i29++) {
                J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i29] +
                         J->size[0] * ((int)b_xt->anchor_states->data[anchorIdx]
                          .P_idx[i28] - 1)) - 1] = 0.0;
              }
            }

            //  TODO: allow roll/pitch uncertainty
          } else {
            eye(f_xt);
            for (i28 = 0; i28 < 3; i28++) {
              for (i29 = 0; i29 < 3; i29++) {
                J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i29] +
                         J->size[0] * ((int)b_xt->anchor_states->data[anchorIdx]
                          .P_idx[i28] - 1)) - 1] = new_origin_att_rel[i29 + 3 *
                  i28];
              }
            }

            for (i28 = 0; i28 < 3; i28++) {
              for (i29 = 0; i29 < 3; i29++) {
                J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i29] +
                         J->size[0] * ((int)b_xt->anchor_states->data[anchorIdx]
                          .P_idx[i28 + 3] - 1)) - 1] = 0.0;
              }
            }

            for (i28 = 0; i28 < 3; i28++) {
              for (i29 = 0; i29 < 3; i29++) {
                J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i29 + 3]
                         + J->size[0] * ((int)b_xt->anchor_states->
                          data[anchorIdx].P_idx[i28] - 1)) - 1] = 0.0;
              }
            }

            for (i28 = 0; i28 < 3; i28++) {
              for (i29 = 0; i29 < 3; i29++) {
                J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i29 + 3]
                         + J->size[0] * ((int)b_xt->anchor_states->
                          data[anchorIdx].P_idx[i28 + 3] - 1)) - 1] = f_xt[i29 +
                  3 * i28];
              }
            }

            for (i28 = 0; i28 < 3; i28++) {
              c_xt[i28] = b_xt->anchor_states->data[anchorIdx].pos[i28] -
                new_origin_pos_rel[i28];
            }

            for (i28 = 0; i28 < 3; i28++) {
              m_l[i28] = 0.0;
              for (i29 = 0; i29 < 3; i29++) {
                m_l[i28] += new_origin_att_rel[i28 + 3 * i29] * c_xt[i29];
              }
            }

            for (i28 = 0; i28 < 3; i28++) {
              for (i29 = 0; i29 < 3; i29++) {
                J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i29] +
                         J->size[0] * ((int)b_xt->anchor_states->data[br]
                          .P_idx[i28] - 1)) - 1] = -new_origin_att_rel[i29 + 3 *
                  i28];
              }
            }

            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[0] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] -
                      1)) - 1] = 0.0;
            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[0] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] -
                      1)) - 1] = -m_l[2];
            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[0] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] -
                      1)) - 1] = m_l[1];
            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[1] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] -
                      1)) - 1] = m_l[2];
            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[1] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] -
                      1)) - 1] = 0.0;
            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[1] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] -
                      1)) - 1] = -m_l[0];
            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[2] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[3] -
                      1)) - 1] = -m_l[1];
            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[2] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[4] -
                      1)) - 1] = m_l[0];
            J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[2] +
                     J->size[0] * ((int)b_xt->anchor_states->data[br].P_idx[5] -
                      1)) - 1] = 0.0;
            for (i28 = 0; i28 < 3; i28++) {
              for (i29 = 0; i29 < 3; i29++) {
                J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i29 + 3]
                         + J->size[0] * ((int)b_xt->anchor_states->data[br].
                          P_idx[i28] - 1)) - 1] = 0.0;
              }
            }

            for (i28 = 0; i28 < 3; i28++) {
              for (i29 = 0; i29 < 3; i29++) {
                J->data[((int)b_xt->anchor_states->data[anchorIdx].P_idx[i29 + 3]
                         + J->size[0] * ((int)b_xt->anchor_states->data[br].
                          P_idx[i28 + 3] - 1)) - 1] = -new_origin_att_rel[i28 +
                  3 * i29];
              }
            }
          }

          for (i28 = 0; i28 < 3; i28++) {
            m_l[i28] = b_xt->anchor_states->data[anchorIdx].pos[i28] -
              new_origin_pos_rel[i28];
          }

          for (i28 = 0; i28 < 3; i28++) {
            b_xt->anchor_states->data[anchorIdx].pos[i28] = 0.0;
          }

          for (i28 = 0; i28 < 3; i28++) {
            b_xt->anchor_states->data[anchorIdx].pos[i28] = 0.0;
            for (i29 = 0; i29 < 3; i29++) {
              b_xt->anchor_states->data[anchorIdx].pos[i28] +=
                new_origin_att_rel[i28 + 3 * i29] * m_l[i29];
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
          for (i28 = 0; i28 < 3; i28++) {
            for (i29 = 0; i29 < 3; i29++) {
              f_xt[i28 + 3 * i29] = 0.0;
              for (i30 = 0; i30 < 3; i30++) {
                f_xt[i28 + 3 * i29] += g_xt[i28 + 3 * i30] *
                  new_origin_att_rel[i29 + 3 * i30];
              }
            }
          }

          QuatFromRotJ(f_xt, b_xt->anchor_states->data[anchorIdx].att);
        }

        if ((J->size[1] == 1) || (P_apr->size[0] == 1)) {
          i28 = y->size[0] * y->size[1];
          y->size[0] = J->size[0];
          y->size[1] = P_apr->size[1];
          emxEnsureCapacity((emxArray__common *)y, i28, (int)sizeof(double));
          ib = J->size[0];
          for (i28 = 0; i28 < ib; i28++) {
            br = P_apr->size[1];
            for (i29 = 0; i29 < br; i29++) {
              y->data[i28 + y->size[0] * i29] = 0.0;
              idx = J->size[1];
              for (i30 = 0; i30 < idx; i30++) {
                y->data[i28 + y->size[0] * i29] += J->data[i28 + J->size[0] *
                  i30] * P_apr->data[i30 + P_apr->size[0] * i29];
              }
            }
          }
        } else {
          k = J->size[1];
          idx = J->size[0];
          ixstart = P_apr->size[1];
          c_m = J->size[0];
          i28 = y->size[0] * y->size[1];
          y->size[0] = idx;
          emxEnsureCapacity((emxArray__common *)y, i28, (int)sizeof(double));
          i28 = y->size[0] * y->size[1];
          y->size[1] = ixstart;
          emxEnsureCapacity((emxArray__common *)y, i28, (int)sizeof(double));
          ib = idx * ixstart;
          for (i28 = 0; i28 < ib; i28++) {
            y->data[i28] = 0.0;
          }

          if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
          } else {
            idx = J->size[0] * (P_apr->size[1] - 1);
            ixstart = 0;
            while ((c_m > 0) && (ixstart <= idx)) {
              i28 = ixstart + c_m;
              for (ic = ixstart; ic + 1 <= i28; ic++) {
                y->data[ic] = 0.0;
              }

              ixstart += c_m;
            }

            br = 0;
            ixstart = 0;
            while ((c_m > 0) && (ixstart <= idx)) {
              ar = 0;
              i28 = br + k;
              for (ib = br; ib + 1 <= i28; ib++) {
                if (P_apr->data[ib] != 0.0) {
                  ia = ar;
                  i29 = ixstart + c_m;
                  for (ic = ixstart; ic + 1 <= i29; ic++) {
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

        i28 = b->size[0] * b->size[1];
        b->size[0] = J->size[1];
        b->size[1] = J->size[0];
        emxEnsureCapacity((emxArray__common *)b, i28, (int)sizeof(double));
        ib = J->size[0];
        for (i28 = 0; i28 < ib; i28++) {
          br = J->size[1];
          for (i29 = 0; i29 < br; i29++) {
            b->data[i29 + b->size[0] * i28] = J->data[i28 + J->size[0] * i29];
          }
        }

        if ((y->size[1] == 1) || (b->size[0] == 1)) {
          i28 = P_apr->size[0] * P_apr->size[1];
          P_apr->size[0] = y->size[0];
          P_apr->size[1] = b->size[1];
          emxEnsureCapacity((emxArray__common *)P_apr, i28, (int)sizeof(double));
          ib = y->size[0];
          for (i28 = 0; i28 < ib; i28++) {
            br = b->size[1];
            for (i29 = 0; i29 < br; i29++) {
              P_apr->data[i28 + P_apr->size[0] * i29] = 0.0;
              idx = y->size[1];
              for (i30 = 0; i30 < idx; i30++) {
                P_apr->data[i28 + P_apr->size[0] * i29] += y->data[i28 + y->
                  size[0] * i30] * b->data[i30 + b->size[0] * i29];
              }
            }
          }
        } else {
          k = y->size[1];
          idx = y->size[0];
          ixstart = b->size[1];
          c_m = y->size[0];
          i28 = P_apr->size[0] * P_apr->size[1];
          P_apr->size[0] = idx;
          P_apr->size[1] = ixstart;
          emxEnsureCapacity((emxArray__common *)P_apr, i28, (int)sizeof(double));
          for (i28 = 0; i28 < ixstart; i28++) {
            for (i29 = 0; i29 < idx; i29++) {
              P_apr->data[i29 + P_apr->size[0] * i28] = 0.0;
            }
          }

          if ((y->size[0] == 0) || (b->size[1] == 0)) {
          } else {
            idx = y->size[0] * (b->size[1] - 1);
            ixstart = 0;
            while ((c_m > 0) && (ixstart <= idx)) {
              i28 = ixstart + c_m;
              for (ic = ixstart; ic + 1 <= i28; ic++) {
                P_apr->data[ic] = 0.0;
              }

              ixstart += c_m;
            }

            br = 0;
            ixstart = 0;
            while ((c_m > 0) && (ixstart <= idx)) {
              ar = 0;
              i28 = br + k;
              for (ib = br; ib + 1 <= i28; ib++) {
                if (b->data[ib] != 0.0) {
                  ia = ar;
                  i29 = ixstart + c_m;
                  for (ic = ixstart; ic + 1 <= i29; ic++) {
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

        for (i28 = 0; i28 < 3; i28++) {
          c_xt[i28] = b_xt->robot_state.pos[i28] - new_origin_pos_rel[i28];
        }

        for (i28 = 0; i28 < 3; i28++) {
          b_xt->robot_state.pos[i28] = 0.0;
          for (i29 = 0; i29 < 3; i29++) {
            b_xt->robot_state.pos[i28] += new_origin_att_rel[i28 + 3 * i29] *
              c_xt[i29];
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
        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            f_xt[i28 + 3 * i29] = 0.0;
            for (i30 = 0; i30 < 3; i30++) {
              f_xt[i28 + 3 * i29] += h_xt[i28 + 3 * i30] *
                new_origin_att_rel[i29 + 3 * i30];
            }
          }
        }

        QuatFromRotJ(f_xt, b_xt->robot_state.att);
        for (i28 = 0; i28 < 3; i28++) {
          c_xt[i28] = 0.0;
          for (i29 = 0; i29 < 3; i29++) {
            c_xt[i28] += new_origin_att_rel[i28 + 3 * i29] *
              b_xt->robot_state.vel[i29];
          }
        }

        for (i28 = 0; i28 < 3; i28++) {
          b_xt->robot_state.vel[i28] = c_xt[i28];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        i_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        i_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        i_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        i_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        i_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        i_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        i_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        i_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        i_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        for (i28 = 0; i28 < 3; i28++) {
          d7 = 0.0;
          for (i29 = 0; i29 < 3; i29++) {
            d7 += i_xt[i28 + 3 * i29] * new_origin_pos_rel[i29];
          }

          b_xt->origin.pos[i28] += d7;
        }

        //  in world frame
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        j_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        j_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        j_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        j_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        j_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        j_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        j_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        j_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        j_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        for (i28 = 0; i28 < 3; i28++) {
          for (i29 = 0; i29 < 3; i29++) {
            f_xt[i28 + 3 * i29] = 0.0;
            for (i30 = 0; i30 < 3; i30++) {
              f_xt[i28 + 3 * i29] += new_origin_att_rel[i28 + 3 * i30] *
                j_xt[i30 + 3 * i29];
            }
          }
        }

        QuatFromRotJ(f_xt, b_xt->origin.att);

        //  in world frame
      }
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
  i28 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(c_numTrackFeatures * 4.0);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i28, (int)sizeof(double));
  ib = (int)(c_numTrackFeatures * 4.0);
  for (i28 = 0; i28 < ib; i28++) {
    h_u_apo->data[i28] = rtNaN;
  }

  for (br = 0; br < 40; br++) {
    b_delayedStatus[br] = 0.0;
  }

  for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
    for (ar = 0; ar < (int)numPointsPerAnchor; ar++) {
      if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].status
          != 0.0) {
        c_predictMeasurementStereoDisto(b_xt->anchor_states->data[anchorIdx].
          feature_states->data[ar].scaled_map_point,
          c_cameraParams_CameraParameters, d_cameraParams_CameraParameters,
          e_cameraParams_CameraParameters, f_cameraParams_CameraParameters,
          g_cameraParams_CameraParameters, h_cameraParams_CameraParameters,
          cameraParams_r_lr, cameraParams_R_rl, h_u_l, h_u_r);
        e_xt = (b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].
                status_idx - 1.0) * 4.0;
        for (i28 = 0; i28 < 2; i28++) {
          h_u_apo->data[(int)(e_xt + (1.0 + (double)i28)) - 1] = h_u_l[i28];
        }

        for (i28 = 0; i28 < 2; i28++) {
          h_u_apo->data[(int)(e_xt + (1.0 + ((double)i28 + 2.0))) - 1] =
            h_u_r[i28];
        }

        if (b_xt->anchor_states->data[anchorIdx].feature_states->data[ar].status
            == 2.0) {
          b_delayedStatus[(int)b_xt->anchor_states->data[anchorIdx].
            feature_states->data[ar].status_idx - 1] = 1.0;
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
