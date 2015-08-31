//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Aug-2015 20:50:24
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_updIT.h"
#include "SLAM_emxutil.h"
#include "predictMeasurement_stereo.h"
#include "any.h"
#include "getMap.h"
#include "eye.h"
#include "fprintf.h"
#include "norm.h"
#include "initializePoint.h"
#include "OnePointRANSAC_EKF.h"
#include "ros_error.h"
#include "sum.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Variable Definitions
static emxArray_real_T *m_vect;
static boolean_T m_vect_not_empty;
static emxArray_real_T *anchorFeatures;
static double anchorAges[4];
static double fixedFeatureIdx;
static double oldestAnchorIdx;

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
//                emxArray_real_T *b_xt
//                const double c_cameraParams_CameraParameters[3]
//                const double d_cameraParams_CameraParameters[2]
//                const double e_cameraParams_CameraParameters[2]
//                const double f_cameraParams_CameraParameters[3]
//                const double g_cameraParams_CameraParameters[2]
//                const double h_cameraParams_CameraParameters[2]
//                const double cameraParams_r_lr[3]
//                const double cameraParams_R_lr[9]
//                const double cameraParams_R_rl[9]
//                double updateVect[24]
//                const double z_all_l[48]
//                const double z_all_r[48]
//                const double noiseParameters_image_noise[2]
//                double noiseParameters_sigmaInit
//                double c_noiseParameters_orientation_n
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                const VIOMeasurements *measurements
//                double b_height_offset_pressure
//                const VIOParameters b_VIOParameters
//                emxArray_real_T *h_u_apo
//                emxArray_real_T *map
// Return Type  : void
//
void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const double
                c_cameraParams_CameraParameters[3], const double
                d_cameraParams_CameraParameters[2], const double
                e_cameraParams_CameraParameters[2], const double
                f_cameraParams_CameraParameters[3], const double
                g_cameraParams_CameraParameters[2], const double
                h_cameraParams_CameraParameters[2], const double
                cameraParams_r_lr[3], const double cameraParams_R_lr[9], const
                double cameraParams_R_rl[9], double updateVect[24], const double
                z_all_l[48], const double z_all_r[48], const double
                noiseParameters_image_noise[2], double noiseParameters_sigmaInit,
                double c_noiseParameters_orientation_n, double
                noiseParameters_pressure_noise, double
                noiseParameters_ext_pos_noise, double
                noiseParameters_ext_att_noise, const VIOMeasurements
                *measurements, double b_height_offset_pressure, const
                VIOParameters b_VIOParameters, emxArray_real_T *h_u_apo,
                emxArray_real_T *map)
{
  double numAnchors;
  double numPointsPerAnchor;
  double numStatesPerAnchor;
  double c_numTrackFeatures;
  int i15;
  int loop_ub;
  int i;
  emxArray_boolean_T *r11;
  double dv22[24];
  boolean_T x[24];
  emxArray_real_T *anchorIdx;
  emxArray_int32_T *r12;
  emxArray_boolean_T *b_x;
  emxArray_int32_T *ii;
  emxArray_real_T *b_anchorIdx;
  int nx;
  int idx;
  int ixstart;
  boolean_T exitg10;
  boolean_T guard7 = false;
  int n;
  int i16;
  int unnamed_idx_0;
  emxArray_real_T *depthUncertainties;
  int ii_data[24];
  double uncertainty;
  double mtmp;
  boolean_T exitg9;
  emxArray_boolean_T *r13;
  boolean_T exitg8;
  boolean_T guard6 = false;
  signed char indMeas_data[24];
  int i17;
  double b_z_all_l[48];
  int validFeatures_size[1];
  double validFeatures_data[24];
  emxArray_real_T *c_anchorIdx;
  boolean_T exitg7;
  boolean_T guard5 = false;
  double initializeNewAnchor;
  int minNumValidFeatures;
  boolean_T exitg6;
  boolean_T guard4 = false;
  double R_cw[9];
  double r_wc[3];
  double d3;
  double c_xt[7];
  emxArray_boolean_T *r14;
  boolean_T exitg5;
  boolean_T guard3 = false;
  int ii_size_idx_0;
  unsigned int featureAnchorIdx;
  int unusedFeatureIdx;
  double z_curr_l[2];
  double z_curr_r[2];
  double m[3];
  double fp[3];
  boolean_T bv2[3];
  signed char i18;
  double b_R_cw[3];
  double b_fp[3];
  double h_u_r[2];
  double h_u_l[2];
  double b_h_u_l[2];
  boolean_T guard1 = false;
  double rhoInit;
  double sigmaInit;
  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  emxArray_real_T *J;
  static const signed char iv12[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  emxArray_real_T *y;
  int b_m;
  int cr;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  emxArray_real_T *b;
  boolean_T exitg4;
  boolean_T exitg3;
  boolean_T exitg2;
  boolean_T guard2 = false;
  double newFeaturesRequested;
  emxArray_boolean_T *r15;
  boolean_T exitg1;
  boolean_T b_guard1 = false;
  emxArray_real_T *unusedU4;
  emxArray_boolean_T *r16;
  numAnchors = b_VIOParameters.num_anchors;
  numPointsPerAnchor = b_VIOParameters.num_points_per_anchor;
  numStatesPerAnchor = 6.0 + b_VIOParameters.num_points_per_anchor;
  c_numTrackFeatures = b_VIOParameters.num_anchors *
    b_VIOParameters.num_points_per_anchor;
  if (!m_vect_not_empty) {
    i15 = m_vect->size[0] * m_vect->size[1];
    m_vect->size[0] = 3;
    m_vect->size[1] = (int)(b_VIOParameters.num_points_per_anchor *
      b_VIOParameters.num_anchors);
    emxEnsureCapacity((emxArray__common *)m_vect, i15, (int)sizeof(double));
    loop_ub = 3 * (int)(b_VIOParameters.num_points_per_anchor *
                        b_VIOParameters.num_anchors);
    for (i15 = 0; i15 < loop_ub; i15++) {
      m_vect->data[i15] = rtNaN;
    }

    m_vect_not_empty = !(m_vect->size[1] == 0);

    //  a matrix containing the m vectors for each feature
    i15 = anchorFeatures->size[0] * anchorFeatures->size[1];
    anchorFeatures->size[0] = 24;
    anchorFeatures->size[1] = (int)b_VIOParameters.num_anchors;
    emxEnsureCapacity((emxArray__common *)anchorFeatures, i15, (int)sizeof
                      (double));
    loop_ub = 24 * (int)b_VIOParameters.num_anchors;
    for (i15 = 0; i15 < loop_ub; i15++) {
      anchorFeatures->data[i15] = 0.0;
    }

    //  describes which feature belongs to which anchor
    for (i = 0; i < 4; i++) {
      anchorAges[i] = 0.0;
    }

    fixedFeatureIdx = 0.0;
    oldestAnchorIdx = 0.0;
  }

  emxInit_boolean_T(&r11, 2);

  //  debug check
  i15 = r11->size[0] * r11->size[1];
  r11->size[0] = 24;
  r11->size[1] = anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)r11, i15, (int)sizeof(boolean_T));
  loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
  for (i15 = 0; i15 < loop_ub; i15++) {
    r11->data[i15] = (anchorFeatures->data[i15] == 1.0);
  }

  sum(r11, dv22);
  emxFree_boolean_T(&r11);
  for (i = 0; i < 24; i++) {
    x[i] = (dv22[i] > 1.0);
  }

  if (any(x)) {
    ros_error();
  }

  // % check for lost features
  i = 0;
  emxInit_real_T(&anchorIdx, 2);
  emxInit_int32_T(&r12, 1);
  emxInit_boolean_T(&b_x, 2);
  b_emxInit_int32_T(&ii, 2);
  b_emxInit_real_T(&b_anchorIdx, 1);
  while (i <= (int)c_numTrackFeatures - 1) {
    loop_ub = anchorFeatures->size[1];
    i15 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = 1;
    b_x->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)b_x, i15, (int)sizeof(boolean_T));
    for (i15 = 0; i15 < loop_ub; i15++) {
      b_x->data[b_x->size[0] * i15] = (anchorFeatures->data[i +
        anchorFeatures->size[0] * i15] == 1.0);
    }

    nx = b_x->size[1];
    idx = 0;
    i15 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = b_x->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i15, (int)sizeof(int));
    ixstart = 1;
    exitg10 = false;
    while ((!exitg10) && (ixstart <= nx)) {
      guard7 = false;
      if (b_x->data[ixstart - 1]) {
        idx++;
        ii->data[idx - 1] = ixstart;
        if (idx >= nx) {
          exitg10 = true;
        } else {
          guard7 = true;
        }
      } else {
        guard7 = true;
      }

      if (guard7) {
        ixstart++;
      }
    }

    if (b_x->size[1] == 1) {
      if (idx == 0) {
        i15 = ii->size[0] * ii->size[1];
        ii->size[0] = 1;
        ii->size[1] = 0;
        emxEnsureCapacity((emxArray__common *)ii, i15, (int)sizeof(int));
      }
    } else {
      i15 = ii->size[0] * ii->size[1];
      if (1 > idx) {
        ii->size[1] = 0;
      } else {
        ii->size[1] = idx;
      }

      emxEnsureCapacity((emxArray__common *)ii, i15, (int)sizeof(int));
    }

    i15 = anchorIdx->size[0] * anchorIdx->size[1];
    anchorIdx->size[0] = 1;
    anchorIdx->size[1] = ii->size[1];
    emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
    loop_ub = ii->size[0] * ii->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      anchorIdx->data[i15] = ii->data[i15];
    }

    if ((!(anchorIdx->size[1] == 0)) && (updateVect[i] != 1.0)) {
      n = 0;
      i15 = b_anchorIdx->size[0];
      b_anchorIdx->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)b_anchorIdx, i15, (int)sizeof(double));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        b_anchorIdx->data[i15] = anchorIdx->data[anchorIdx->size[0] * i15];
      }

      i15 = (i + 1) * b_anchorIdx->size[0];
      for (nx = 0; nx < i15; nx++) {
        i16 = r12->size[0];
        r12->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r12, i16, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i16 = 0; i16 < loop_ub; i16++) {
          r12->data[i16] = (int)anchorIdx->data[anchorIdx->size[0] * i16];
        }

        if (anchorFeatures->data[nx % (i + 1) + anchorFeatures->size[0] *
            (r12->data[div_nzp_s32_floor(nx, i + 1)] - 1)] != 0.0) {
          n++;
        }
      }

      //  remove covariance of this feature with rest of state
      i15 = r12->size[0];
      r12->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r12->data[i15] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0]
          * i15] - 1.0) * numStatesPerAnchor) + 6.0) + (double)n);
      }

      loop_ub = P_apr->size[1];
      unnamed_idx_0 = r12->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < unnamed_idx_0; i16++) {
          P_apr->data[(r12->data[i16] + P_apr->size[0] * i15) - 1] = 0.0;
        }
      }

      i15 = r12->size[0];
      r12->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r12->data[i15] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0]
          * i15] - 1.0) * numStatesPerAnchor) + 6.0) + (double)n);
      }

      loop_ub = P_apr->size[0];
      ixstart = r12->size[0];
      for (i15 = 0; i15 < ixstart; i15++) {
        for (i16 = 0; i16 < loop_ub; i16++) {
          P_apr->data[i16 + P_apr->size[0] * (r12->data[i15] - 1)] = 0.0;
        }
      }

      i15 = r12->size[0];
      r12->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r12->data[i15] = (int)anchorIdx->data[anchorIdx->size[0] * i15];
      }

      ixstart = r12->size[0];
      for (i15 = 0; i15 < ixstart; i15++) {
        anchorFeatures->data[i + anchorFeatures->size[0] * (r12->data[i15] - 1)]
          = -1.0;
      }

      //  mark feature as lost
    }

    i++;
  }

  emxFree_real_T(&b_anchorIdx);

  // % check if the fixed feature is still valid
  b_emxInit_real_T(&depthUncertainties, 1);
  if (b_VIOParameters.fixed_anchor && (fixedFeatureIdx != 0.0)) {
    //  only do if features have already been initialized
    for (i15 = 0; i15 < 24; i15++) {
      x[i15] = (anchorFeatures->data[i15 + anchorFeatures->size[0] * ((int)
                 oldestAnchorIdx - 1)] != 0.0);
    }

    nx = 0;
    for (i = 0; i < 24; i++) {
      if (x[i]) {
        nx++;
      }
    }

    if (!(nx == 0)) {
      //  otherwise the anchor hasnt been initialized (only happens at the very beginning) 
      for (i15 = 0; i15 < 24; i15++) {
        x[i15] = (anchorFeatures->data[i15 + anchorFeatures->size[0] * ((int)
                   oldestAnchorIdx - 1)] != 0.0);
      }

      ixstart = 0;
      for (i = 0; i < 24; i++) {
        if (x[i]) {
          ii_data[ixstart] = i + 1;
          ixstart++;
        }
      }

      if (anchorFeatures->data[(ii_data[(int)fixedFeatureIdx - 1] +
           anchorFeatures->size[0] * ((int)oldestAnchorIdx - 1)) - 1] != 1.0) {
        //  the feature was lost
        //  set the depth uncertainty of the most-converged feature of the
        //  oldest anchor to 0
        i15 = depthUncertainties->size[0];
        depthUncertainties->size[0] = (int)b_VIOParameters.num_points_per_anchor;
        emxEnsureCapacity((emxArray__common *)depthUncertainties, i15, (int)
                          sizeof(double));
        loop_ub = (int)b_VIOParameters.num_points_per_anchor;
        for (i15 = 0; i15 < loop_ub; i15++) {
          depthUncertainties->data[i15] = 0.0;
        }

        for (ixstart = 0; ixstart < (int)numPointsPerAnchor; ixstart++) {
          uncertainty = P_apr->data[((int)(((numStates + (oldestAnchorIdx - 1.0)
            * (6.0 + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ixstart)) +
            P_apr->size[0] * ((int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ixstart)) - 1)) - 1];
          if (P_apr->data[((int)(((numStates + (oldestAnchorIdx - 1.0) *
                  numStatesPerAnchor) + 6.0) + (1.0 + (double)ixstart)) +
                           P_apr->size[0] * ((int)(((numStates +
                   (oldestAnchorIdx - 1.0) * numStatesPerAnchor) + 6.0) + (1.0 +
                  (double)ixstart)) - 1)) - 1] < 2.2204460492503131E-16) {
            uncertainty = 100.0;

            //  dont use lost features
          }

          depthUncertainties->data[ixstart] = uncertainty;
        }

        ixstart = 1;
        n = depthUncertainties->size[0];
        mtmp = depthUncertainties->data[0];
        nx = 1;
        if (depthUncertainties->size[0] > 1) {
          if (rtIsNaN(depthUncertainties->data[0])) {
            idx = 2;
            exitg9 = false;
            while ((!exitg9) && (idx <= n)) {
              ixstart = idx;
              if (!rtIsNaN(depthUncertainties->data[idx - 1])) {
                mtmp = depthUncertainties->data[idx - 1];
                nx = idx;
                exitg9 = true;
              } else {
                idx++;
              }
            }
          }

          if (ixstart < depthUncertainties->size[0]) {
            while (ixstart + 1 <= n) {
              if (depthUncertainties->data[ixstart] < mtmp) {
                mtmp = depthUncertainties->data[ixstart];
                nx = ixstart + 1;
              }

              ixstart++;
            }
          }
        }

        fixedFeatureIdx = nx;
        loop_ub = P_apr->size[1];
        i15 = (int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0 +
          b_VIOParameters.num_points_per_anchor)) + 6.0) + fixedFeatureIdx);
        for (i16 = 0; i16 < loop_ub; i16++) {
          P_apr->data[(i15 + P_apr->size[0] * i16) - 1] = 0.0;
        }

        loop_ub = P_apr->size[0];
        i15 = (int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0 +
          b_VIOParameters.num_points_per_anchor)) + 6.0) + fixedFeatureIdx);
        for (i16 = 0; i16 < loop_ub; i16++) {
          P_apr->data[i16 + P_apr->size[0] * (i15 - 1)] = 0.0;
        }

        //              fprintf('Fixing feature %i of anchor %i\n', fixedFeatureIdx, oldestAnchorIdx); 
      }
    }
  }

  emxInit_boolean_T(&r13, 2);

  // % do the update
  i15 = r13->size[0] * r13->size[1];
  r13->size[0] = 24;
  r13->size[1] = anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)r13, i15, (int)sizeof(boolean_T));
  loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
  for (i15 = 0; i15 < loop_ub; i15++) {
    r13->data[i15] = (anchorFeatures->data[i15] == 1.0);
  }

  b_any(r13, x);
  idx = 0;
  ixstart = 1;
  emxFree_boolean_T(&r13);
  exitg8 = false;
  while ((!exitg8) && (ixstart < 25)) {
    guard6 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = ixstart;
      if (idx >= 24) {
        exitg8 = true;
      } else {
        guard6 = true;
      }
    } else {
      guard6 = true;
    }

    if (guard6) {
      ixstart++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  for (i15 = 0; i15 < loop_ub; i15++) {
    indMeas_data[i15] = (signed char)ii_data[i15];
  }

  if (1 > idx) {
    i17 = 0;
  } else {
    i17 = idx;
  }

  if (!(i17 == 0)) {
    memcpy(&b_z_all_l[0], &z_all_l[0], 48U * sizeof(double));
    OnePointRANSAC_EKF(b_xt, P_apr, b_z_all_l, numStatesxt, numStates,
                       c_cameraParams_CameraParameters,
                       d_cameraParams_CameraParameters,
                       e_cameraParams_CameraParameters, anchorFeatures, m_vect,
                       noiseParameters_image_noise,
                       c_noiseParameters_orientation_n,
                       noiseParameters_pressure_noise,
                       noiseParameters_ext_pos_noise,
                       noiseParameters_ext_att_noise, measurements,
                       b_height_offset_pressure, b_VIOParameters,
                       validFeatures_data, validFeatures_size);
    for (i15 = 0; i15 < loop_ub; i15++) {
      ii_data[i15] = indMeas_data[i15];
    }

    for (i15 = 0; i15 < loop_ub; i15++) {
      updateVect[ii_data[i15] - 1] = 0.0;
    }

    loop_ub = validFeatures_size[0];
    for (i15 = 0; i15 < loop_ub; i15++) {
      ii_data[i15] = (int)validFeatures_data[i15];
    }

    loop_ub = validFeatures_size[0];
    for (i15 = 0; i15 < loop_ub; i15++) {
      updateVect[ii_data[i15] - 1] = 1.0;
    }

    //  check for lost features
    i = 0;
    b_emxInit_real_T(&c_anchorIdx, 1);
    while (i <= (int)c_numTrackFeatures - 1) {
      loop_ub = anchorFeatures->size[1];
      i15 = b_x->size[0] * b_x->size[1];
      b_x->size[0] = 1;
      b_x->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)b_x, i15, (int)sizeof(boolean_T));
      for (i15 = 0; i15 < loop_ub; i15++) {
        b_x->data[b_x->size[0] * i15] = (anchorFeatures->data[i +
          anchorFeatures->size[0] * i15] == 1.0);
      }

      nx = b_x->size[1];
      idx = 0;
      i15 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = b_x->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i15, (int)sizeof(int));
      ixstart = 1;
      exitg7 = false;
      while ((!exitg7) && (ixstart <= nx)) {
        guard5 = false;
        if (b_x->data[ixstart - 1]) {
          idx++;
          ii->data[idx - 1] = ixstart;
          if (idx >= nx) {
            exitg7 = true;
          } else {
            guard5 = true;
          }
        } else {
          guard5 = true;
        }

        if (guard5) {
          ixstart++;
        }
      }

      if (b_x->size[1] == 1) {
        if (idx == 0) {
          i15 = ii->size[0] * ii->size[1];
          ii->size[0] = 1;
          ii->size[1] = 0;
          emxEnsureCapacity((emxArray__common *)ii, i15, (int)sizeof(int));
        }
      } else {
        i15 = ii->size[0] * ii->size[1];
        if (1 > idx) {
          ii->size[1] = 0;
        } else {
          ii->size[1] = idx;
        }

        emxEnsureCapacity((emxArray__common *)ii, i15, (int)sizeof(int));
      }

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = ii->size[1];
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      loop_ub = ii->size[0] * ii->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        anchorIdx->data[i15] = ii->data[i15];
      }

      if ((!(anchorIdx->size[1] == 0)) && (updateVect[i] != 1.0)) {
        n = 0;
        i15 = c_anchorIdx->size[0];
        c_anchorIdx->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)c_anchorIdx, i15, (int)sizeof
                          (double));
        loop_ub = anchorIdx->size[1];
        for (i15 = 0; i15 < loop_ub; i15++) {
          c_anchorIdx->data[i15] = anchorIdx->data[anchorIdx->size[0] * i15];
        }

        i15 = (i + 1) * c_anchorIdx->size[0];
        for (nx = 0; nx < i15; nx++) {
          i16 = r12->size[0];
          r12->size[0] = anchorIdx->size[1];
          emxEnsureCapacity((emxArray__common *)r12, i16, (int)sizeof(int));
          loop_ub = anchorIdx->size[1];
          for (i16 = 0; i16 < loop_ub; i16++) {
            r12->data[i16] = (int)anchorIdx->data[anchorIdx->size[0] * i16];
          }

          if (anchorFeatures->data[nx % (i + 1) + anchorFeatures->size[0] *
              (r12->data[div_nzp_s32_floor(nx, i + 1)] - 1)] != 0.0) {
            n++;
          }
        }

        //  remove covariance of this feature with rest of state
        i15 = r12->size[0];
        r12->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i15 = 0; i15 < loop_ub; i15++) {
          r12->data[i15] = (int)(((numStates + (anchorIdx->data[anchorIdx->size
            [0] * i15] - 1.0) * numStatesPerAnchor) + 6.0) + (double)n);
        }

        loop_ub = P_apr->size[1];
        unnamed_idx_0 = r12->size[0];
        for (i15 = 0; i15 < loop_ub; i15++) {
          for (i16 = 0; i16 < unnamed_idx_0; i16++) {
            P_apr->data[(r12->data[i16] + P_apr->size[0] * i15) - 1] = 0.0;
          }
        }

        i15 = r12->size[0];
        r12->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i15 = 0; i15 < loop_ub; i15++) {
          r12->data[i15] = (int)(((numStates + (anchorIdx->data[anchorIdx->size
            [0] * i15] - 1.0) * numStatesPerAnchor) + 6.0) + (double)n);
        }

        loop_ub = P_apr->size[0];
        ixstart = r12->size[0];
        for (i15 = 0; i15 < ixstart; i15++) {
          for (i16 = 0; i16 < loop_ub; i16++) {
            P_apr->data[i16 + P_apr->size[0] * (r12->data[i15] - 1)] = 0.0;
          }
        }

        i15 = r12->size[0];
        r12->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i15 = 0; i15 < loop_ub; i15++) {
          r12->data[i15] = (int)anchorIdx->data[anchorIdx->size[0] * i15];
        }

        ixstart = r12->size[0];
        for (i15 = 0; i15 < ixstart; i15++) {
          anchorFeatures->data[i + anchorFeatures->size[0] * (r12->data[i15] - 1)]
            = -1.0;
        }

        //  mark feature as lost
      }

      i++;
    }

    emxFree_real_T(&c_anchorIdx);
  }

  emxFree_int32_T(&ii);
  emxFree_boolean_T(&b_x);

  // % Initialize new anchors
  for (i = 0; i < 24; i++) {
    x[i] = (updateVect[i] == 2.0);
  }

  if (any(x)) {
    //  if there are any features with stereo measurements
    initializeNewAnchor = 0.0;
    minNumValidFeatures = 10000;
    ixstart = 0;
    exitg6 = false;
    while ((!exitg6) && (ixstart <= (int)numAnchors - 1)) {
      for (i15 = 0; i15 < 24; i15++) {
        x[i15] = ((anchorFeatures->data[i15 + anchorFeatures->size[0] * ixstart]
                   == 1.0) && (updateVect[i15] == 1.0));
      }

      n = 0;
      for (nx = 0; nx < 24; nx++) {
        if (x[nx]) {
          n++;
        }
      }

      guard4 = false;
      if ((n < minFeatureThreshold) && (n < minNumValidFeatures)) {
        minNumValidFeatures = n;
        initializeNewAnchor = 1.0 + (double)ixstart;
        if (!(n != 0)) {
          exitg6 = true;
        } else {
          guard4 = true;
        }
      } else {
        guard4 = true;
      }

      if (guard4) {
        ixstart++;
      }
    }

    if (initializeNewAnchor > 0.0) {
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4])
                 - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
      R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] *
                       b_xt->data[6]);
      R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] *
                       b_xt->data[6]);
      R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] *
                       b_xt->data[6]);
      R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data
                  [4]) - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] *
        b_xt->data[6];
      R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] *
                       b_xt->data[6]);
      R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] *
                       b_xt->data[6]);
      R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] *
                       b_xt->data[6]);
      R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data
                  [4]) + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] *
        b_xt->data[6];
      for (i15 = 0; i15 < 3; i15++) {
        r_wc[i15] = b_xt->data[i15];
      }

      for (i15 = 0; i15 < 24; i15++) {
        x[i15] = (anchorFeatures->data[i15 + anchorFeatures->size[0] * ((int)
                   initializeNewAnchor - 1)] == 1.0);
      }

      nx = 0;
      for (i = 0; i < 24; i++) {
        if (x[i]) {
          nx++;
        }
      }

      ixstart = 0;
      for (i = 0; i < 24; i++) {
        if (x[i]) {
          ii_data[ixstart] = i + 1;
          ixstart++;
        }
      }

      for (i15 = 0; i15 < nx; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          m_vect->data[i16 + m_vect->size[0] * (ii_data[i15] - 1)] = rtNaN;
        }
      }

      for (i15 = 0; i15 < 24; i15++) {
        anchorFeatures->data[i15 + anchorFeatures->size[0] * ((int)
          initializeNewAnchor - 1)] = 0.0;
      }

      d3 = numStatesxt + (initializeNewAnchor - 1.0) * (7.0 +
        b_VIOParameters.num_points_per_anchor);
      for (i15 = 0; i15 < 7; i15++) {
        c_xt[i15] = b_xt->data[i15];
      }

      for (i15 = 0; i15 < 7; i15++) {
        b_xt->data[(int)(d3 + (1.0 + (double)i15)) - 1] = c_xt[i15];
      }

      i15 = depthUncertainties->size[0];
      depthUncertainties->size[0] = (int)b_VIOParameters.num_points_per_anchor;
      emxEnsureCapacity((emxArray__common *)depthUncertainties, i15, (int)sizeof
                        (double));
      loop_ub = (int)b_VIOParameters.num_points_per_anchor;
      for (i15 = 0; i15 < loop_ub; i15++) {
        depthUncertainties->data[i15] = 0.0;
      }

      emxInit_boolean_T(&r14, 2);
      i15 = r14->size[0] * r14->size[1];
      r14->size[0] = 24;
      r14->size[1] = anchorFeatures->size[1];
      emxEnsureCapacity((emxArray__common *)r14, i15, (int)sizeof(boolean_T));
      loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r14->data[i15] = (anchorFeatures->data[i15] == 1.0);
      }

      b_any(r14, x);
      emxFree_boolean_T(&r14);
      for (i15 = 0; i15 < 24; i15++) {
        x[i15] = !x[i15];
      }

      idx = 0;
      ixstart = 1;
      exitg5 = false;
      while ((!exitg5) && (ixstart < 25)) {
        guard3 = false;
        if (x[ixstart - 1] && (updateVect[ixstart - 1] == 2.0)) {
          idx++;
          ii_data[idx - 1] = ixstart;
          if (idx >= 24) {
            exitg5 = true;
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }

        if (guard3) {
          ixstart++;
        }
      }

      if (1 > idx) {
        ii_size_idx_0 = 0;
      } else {
        ii_size_idx_0 = idx;
      }

      if (1 > idx) {
        loop_ub = 0;
      } else {
        loop_ub = idx;
      }

      for (i15 = 0; i15 < loop_ub; i15++) {
        indMeas_data[i15] = (signed char)ii_data[i15];
      }

      featureAnchorIdx = 1U;
      unusedFeatureIdx = 0;
      while ((unusedFeatureIdx <= ii_size_idx_0 - 1) && (!(featureAnchorIdx >
               numPointsPerAnchor))) {
        ixstart = (indMeas_data[unusedFeatureIdx] - 1) * 2;
        nx = (indMeas_data[unusedFeatureIdx] - 1) * 2;
        for (i15 = 0; i15 < 2; i15++) {
          z_curr_l[i15] = z_all_l[ixstart + i15];
          z_curr_r[i15] = z_all_r[nx + i15];
        }

        initializePoint(b_xt, c_cameraParams_CameraParameters,
                        d_cameraParams_CameraParameters,
                        e_cameraParams_CameraParameters,
                        f_cameraParams_CameraParameters,
                        g_cameraParams_CameraParameters,
                        h_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, z_curr_l, z_curr_r, fp, m);
        for (i = 0; i < 3; i++) {
          bv2[i] = rtIsNaN(fp[i]);
        }

        if (c_any(bv2)) {
          updateVect[indMeas_data[unusedFeatureIdx] - 1] = 0.0;
          i15 = (int)rt_roundd_snf((double)indMeas_data[unusedFeatureIdx]);
          i18 = (signed char)i15;
          b_fprintf(i18);
        } else {
          //  check reprojection error
          for (i15 = 0; i15 < 3; i15++) {
            b_fp[i15] = fp[i15] - r_wc[i15];
          }

          for (i15 = 0; i15 < 3; i15++) {
            b_R_cw[i15] = 0.0;
            for (i16 = 0; i16 < 3; i16++) {
              b_R_cw[i15] += R_cw[i15 + 3 * i16] * b_fp[i16];
            }
          }

          predictMeasurement_stereo(b_R_cw, c_cameraParams_CameraParameters,
            d_cameraParams_CameraParameters, e_cameraParams_CameraParameters,
            f_cameraParams_CameraParameters, g_cameraParams_CameraParameters,
            h_cameraParams_CameraParameters, cameraParams_r_lr,
            cameraParams_R_rl, h_u_l, h_u_r);
          for (i = 0; i < 2; i++) {
            b_h_u_l[i] = h_u_l[i] - z_curr_l[i];
          }

          guard1 = false;
          if (c_norm(b_h_u_l) > 10.0) {
            guard1 = true;
          } else {
            for (i = 0; i < 2; i++) {
              b_h_u_l[i] = h_u_r[i] - z_curr_r[i];
            }

            if (c_norm(b_h_u_l) > 10.0) {
              guard1 = true;
            } else {
              for (i15 = 0; i15 < 3; i15++) {
                m_vect->data[i15 + m_vect->size[0] *
                  (indMeas_data[unusedFeatureIdx] - 1)] = m[i15];
              }

              for (i15 = 0; i15 < 3; i15++) {
                b_fp[i15] = b_xt->data[i15] - fp[i15];
              }

              rhoInit = 1.0 / norm(b_fp);
              if (b_VIOParameters.fixed_anchor) {
                sigmaInit = noiseParameters_sigmaInit / rhoInit;
              } else {
                sigmaInit = noiseParameters_sigmaInit;
              }

              b_xt->data[(int)(((numStatesxt + (initializeNewAnchor - 1.0) *
                                 (7.0 + numPointsPerAnchor)) + 7.0) + (double)
                               featureAnchorIdx) - 1] = rhoInit;
              depthUncertainties->data[(int)featureAnchorIdx - 1] = sigmaInit;
              anchorFeatures->data[(indMeas_data[unusedFeatureIdx] +
                                    anchorFeatures->size[0] * ((int)
                initializeNewAnchor - 1)) - 1] = 1.0;
              updateVect[indMeas_data[unusedFeatureIdx] - 1] = 1.0;
              featureAnchorIdx++;
            }
          }

          if (guard1) {
            i15 = (int)rt_roundd_snf((double)indMeas_data[unusedFeatureIdx]);
            i18 = (signed char)i15;
            d_fprintf(i18);
            updateVect[indMeas_data[unusedFeatureIdx] - 1] = 0.0;
          }
        }

        unusedFeatureIdx++;
      }

      d3 = rt_roundd_snf((double)featureAnchorIdx - 1.0);
      if (d3 < 2.147483648E+9) {
        i15 = (int)d3;
      } else {
        i15 = MAX_int32_T;
      }

      f_fprintf(i15, (int)initializeNewAnchor);
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

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          ixstart = (n + (n < 0)) >> 1;
          for (nx = 1; nx < ixstart; nx++) {
            anchorIdx->data[nx] = anew + (double)nx;
            anchorIdx->data[n - nx] = apnd - (double)nx;
          }

          if (ixstart << 1 == n) {
            anchorIdx->data[ixstart] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[ixstart] = anew + (double)ixstart;
            anchorIdx->data[ixstart + 1] = apnd - (double)ixstart;
          }
        }
      }

      d3 = numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_VIOParameters.num_points_per_anchor);
      i15 = r12->size[0];
      r12->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r12->data[i15] = (int)(d3 + anchorIdx->data[anchorIdx->size[0] * i15]);
      }

      loop_ub = P_apr->size[1];
      unnamed_idx_0 = r12->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < unnamed_idx_0; i16++) {
          P_apr->data[(r12->data[i16] + P_apr->size[0] * i15) - 1] = 0.0;
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

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          ixstart = (n + (n < 0)) >> 1;
          for (nx = 1; nx < ixstart; nx++) {
            anchorIdx->data[nx] = anew + (double)nx;
            anchorIdx->data[n - nx] = apnd - (double)nx;
          }

          if (ixstart << 1 == n) {
            anchorIdx->data[ixstart] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[ixstart] = anew + (double)ixstart;
            anchorIdx->data[ixstart + 1] = apnd - (double)ixstart;
          }
        }
      }

      d3 = numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_VIOParameters.num_points_per_anchor);
      i15 = r12->size[0];
      r12->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r12->data[i15] = (int)(d3 + anchorIdx->data[anchorIdx->size[0] * i15]);
      }

      loop_ub = P_apr->size[0];
      ixstart = r12->size[0];
      for (i15 = 0; i15 < ixstart; i15++) {
        for (i16 = 0; i16 < loop_ub; i16++) {
          P_apr->data[i16 + P_apr->size[0] * (r12->data[i15] - 1)] = 0.0;
        }
      }

      for (i15 = 0; i15 < 2; i15++) {
        b_h_u_l[i15] = P_apr->size[i15];
      }

      emxInit_real_T(&J, 2);
      c_eye(b_h_u_l, J);
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

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          ixstart = (n + (n < 0)) >> 1;
          for (nx = 1; nx < ixstart; nx++) {
            anchorIdx->data[nx] = anew + (double)nx;
            anchorIdx->data[n - nx] = apnd - (double)nx;
          }

          if (ixstart << 1 == n) {
            anchorIdx->data[ixstart] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[ixstart] = anew + (double)ixstart;
            anchorIdx->data[ixstart + 1] = apnd - (double)ixstart;
          }
        }
      }

      d3 = numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_VIOParameters.num_points_per_anchor);
      i15 = r12->size[0];
      r12->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r12->data[i15] = (int)(d3 + anchorIdx->data[anchorIdx->size[0] * i15]);
      }

      loop_ub = J->size[1];
      unnamed_idx_0 = r12->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < unnamed_idx_0; i16++) {
          J->data[(r12->data[i16] + J->size[0] * i15) - 1] = 0.0;
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

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          ixstart = (n + (n < 0)) >> 1;
          for (nx = 1; nx < ixstart; nx++) {
            anchorIdx->data[nx] = anew + (double)nx;
            anchorIdx->data[n - nx] = apnd - (double)nx;
          }

          if (ixstart << 1 == n) {
            anchorIdx->data[ixstart] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[ixstart] = anew + (double)ixstart;
            anchorIdx->data[ixstart + 1] = apnd - (double)ixstart;
          }
        }
      }

      d3 = numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_VIOParameters.num_points_per_anchor);
      i15 = r12->size[0];
      r12->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r12->data[i15] = (int)(d3 + anchorIdx->data[anchorIdx->size[0] * i15]) -
          1;
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r12->data[i16] + J->size[0] * i15] = iv12[i16 + 3 * i15];
        }
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r12->data[i16] + J->size[0] * (i15 + 3)] = 0.0;
        }
      }

      loop_ub = (int)(numStates - 6.0);
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r12->data[i16] + J->size[0] * (i15 + 6)] = 0.0;
        }
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r12->data[i16 + 3] + J->size[0] * i15] = 0.0;
        }
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r12->data[i16 + 3] + J->size[0] * (i15 + 3)] = iv12[i16 + 3 *
            i15];
        }
      }

      loop_ub = (int)(numStates - 6.0);
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r12->data[i16 + 3] + J->size[0] * (i15 + 6)] = 0.0;
        }
      }

      loop_ub = (int)numStates;
      for (i15 = 0; i15 < loop_ub; i15++) {
        ixstart = (int)b_VIOParameters.num_points_per_anchor;
        for (i16 = 0; i16 < ixstart; i16++) {
          J->data[r12->data[i16 + 6] + J->size[0] * i15] = 0.0;
        }
      }

      emxInit_real_T(&y, 2);
      if ((J->size[1] == 1) || (P_apr->size[0] == 1)) {
        i15 = y->size[0] * y->size[1];
        y->size[0] = J->size[0];
        y->size[1] = P_apr->size[1];
        emxEnsureCapacity((emxArray__common *)y, i15, (int)sizeof(double));
        loop_ub = J->size[0];
        for (i15 = 0; i15 < loop_ub; i15++) {
          ixstart = P_apr->size[1];
          for (i16 = 0; i16 < ixstart; i16++) {
            y->data[i15 + y->size[0] * i16] = 0.0;
            idx = J->size[1];
            for (n = 0; n < idx; n++) {
              y->data[i15 + y->size[0] * i16] += J->data[i15 + J->size[0] * n] *
                P_apr->data[n + P_apr->size[0] * i16];
            }
          }
        }
      } else {
        nx = J->size[1];
        unnamed_idx_0 = J->size[0];
        ixstart = P_apr->size[1];
        b_m = J->size[0];
        i15 = y->size[0] * y->size[1];
        y->size[0] = unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)y, i15, (int)sizeof(double));
        i15 = y->size[0] * y->size[1];
        y->size[1] = ixstart;
        emxEnsureCapacity((emxArray__common *)y, i15, (int)sizeof(double));
        loop_ub = unnamed_idx_0 * ixstart;
        for (i15 = 0; i15 < loop_ub; i15++) {
          y->data[i15] = 0.0;
        }

        if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
        } else {
          ixstart = J->size[0] * (P_apr->size[1] - 1);
          cr = 0;
          while ((b_m > 0) && (cr <= ixstart)) {
            i15 = cr + b_m;
            for (ic = cr; ic + 1 <= i15; ic++) {
              y->data[ic] = 0.0;
            }

            cr += b_m;
          }

          br = 0;
          cr = 0;
          while ((b_m > 0) && (cr <= ixstart)) {
            ar = 0;
            i15 = br + nx;
            for (ib = br; ib + 1 <= i15; ib++) {
              if (P_apr->data[ib] != 0.0) {
                ia = ar;
                i16 = cr + b_m;
                for (ic = cr; ic + 1 <= i16; ic++) {
                  ia++;
                  y->data[ic] += P_apr->data[ib] * J->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += nx;
            cr += b_m;
          }
        }
      }

      emxInit_real_T(&b, 2);
      i15 = b->size[0] * b->size[1];
      b->size[0] = J->size[1];
      b->size[1] = J->size[0];
      emxEnsureCapacity((emxArray__common *)b, i15, (int)sizeof(double));
      loop_ub = J->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        ixstart = J->size[1];
        for (i16 = 0; i16 < ixstart; i16++) {
          b->data[i16 + b->size[0] * i15] = J->data[i15 + J->size[0] * i16];
        }
      }

      emxFree_real_T(&J);
      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i15 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = y->size[0];
        P_apr->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i15, (int)sizeof(double));
        loop_ub = y->size[0];
        for (i15 = 0; i15 < loop_ub; i15++) {
          ixstart = b->size[1];
          for (i16 = 0; i16 < ixstart; i16++) {
            P_apr->data[i15 + P_apr->size[0] * i16] = 0.0;
            idx = y->size[1];
            for (n = 0; n < idx; n++) {
              P_apr->data[i15 + P_apr->size[0] * i16] += y->data[i15 + y->size[0]
                * n] * b->data[n + b->size[0] * i16];
            }
          }
        }
      } else {
        nx = y->size[1];
        unnamed_idx_0 = y->size[0];
        ixstart = b->size[1];
        b_m = y->size[0];
        i15 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = unnamed_idx_0;
        P_apr->size[1] = ixstart;
        emxEnsureCapacity((emxArray__common *)P_apr, i15, (int)sizeof(double));
        for (i15 = 0; i15 < ixstart; i15++) {
          for (i16 = 0; i16 < unnamed_idx_0; i16++) {
            P_apr->data[i16 + P_apr->size[0] * i15] = 0.0;
          }
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ixstart = y->size[0] * (b->size[1] - 1);
          cr = 0;
          while ((b_m > 0) && (cr <= ixstart)) {
            i15 = cr + b_m;
            for (ic = cr; ic + 1 <= i15; ic++) {
              P_apr->data[ic] = 0.0;
            }

            cr += b_m;
          }

          br = 0;
          cr = 0;
          while ((b_m > 0) && (cr <= ixstart)) {
            ar = 0;
            i15 = br + nx;
            for (ib = br; ib + 1 <= i15; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i16 = cr + b_m;
                for (ic = cr; ic + 1 <= i16; ic++) {
                  ia++;
                  P_apr->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += nx;
            cr += b_m;
          }
        }
      }

      emxFree_real_T(&b);
      emxFree_real_T(&y);
      for (ixstart = 0; ixstart < (int)numPointsPerAnchor; ixstart++) {
        P_apr->data[((int)(((numStates + (initializeNewAnchor - 1.0) * (6.0 +
          numPointsPerAnchor)) + 6.0) + (1.0 + (double)ixstart)) + P_apr->size[0]
                     * ((int)(((numStates + (initializeNewAnchor - 1.0) * (6.0 +
          numPointsPerAnchor)) + 6.0) + (1.0 + (double)ixstart)) - 1)) - 1] =
          depthUncertainties->data[ixstart];
      }

      if (b_VIOParameters.fixed_anchor) {
        //  fix the position of the oldest anchor
        for (i15 = 0; i15 < 4; i15++) {
          anchorAges[i15]++;
        }

        anchorAges[(int)initializeNewAnchor - 1] = 0.0;

        //  set the position uncertainty of the oldest anchor to 0
        ixstart = 1;
        mtmp = anchorAges[0];
        nx = 1;
        if (rtIsNaN(anchorAges[0])) {
          idx = 2;
          exitg4 = false;
          while ((!exitg4) && (idx < 5)) {
            ixstart = idx;
            if (!rtIsNaN(anchorAges[idx - 1])) {
              mtmp = anchorAges[idx - 1];
              nx = idx;
              exitg4 = true;
            } else {
              idx++;
            }
          }
        }

        if (ixstart < 4) {
          while (ixstart + 1 < 5) {
            if (anchorAges[ixstart] > mtmp) {
              mtmp = anchorAges[ixstart];
              nx = ixstart + 1;
            }

            ixstart++;
          }
        }

        if (oldestAnchorIdx != nx) {
          oldestAnchorIdx = nx;
          loop_ub = P_apr->size[1];
          d3 = numStates + (oldestAnchorIdx - 1.0) * (6.0 +
            b_VIOParameters.num_points_per_anchor);
          for (i15 = 0; i15 < loop_ub; i15++) {
            for (i16 = 0; i16 < 3; i16++) {
              P_apr->data[((int)(d3 + (1.0 + (double)i16)) + P_apr->size[0] *
                           i15) - 1] = 0.0;
            }
          }

          loop_ub = P_apr->size[0];
          d3 = numStates + (oldestAnchorIdx - 1.0) * (6.0 +
            b_VIOParameters.num_points_per_anchor);
          for (i15 = 0; i15 < 3; i15++) {
            for (i16 = 0; i16 < loop_ub; i16++) {
              P_apr->data[i16 + P_apr->size[0] * ((int)(d3 + (1.0 + (double)i15))
                - 1)] = 0.0;
            }
          }

          //  set the depth uncertainty of the most-converged feature of the
          //  oldest anchor to 0
          i15 = depthUncertainties->size[0];
          depthUncertainties->size[0] = (int)
            b_VIOParameters.num_points_per_anchor;
          emxEnsureCapacity((emxArray__common *)depthUncertainties, i15, (int)
                            sizeof(double));
          loop_ub = (int)b_VIOParameters.num_points_per_anchor;
          for (i15 = 0; i15 < loop_ub; i15++) {
            depthUncertainties->data[i15] = 0.0;
          }

          for (ixstart = 0; ixstart < (int)numPointsPerAnchor; ixstart++) {
            uncertainty = P_apr->data[((int)(((numStates + (oldestAnchorIdx -
              1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ixstart))
              + P_apr->size[0] * ((int)(((numStates + (oldestAnchorIdx - 1.0) *
              (6.0 + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ixstart)) - 1))
              - 1];
            if (P_apr->data[((int)(((numStates + (oldestAnchorIdx - 1.0) *
                    numStatesPerAnchor) + 6.0) + (1.0 + (double)ixstart)) +
                             P_apr->size[0] * ((int)(((numStates +
                     (oldestAnchorIdx - 1.0) * numStatesPerAnchor) + 6.0) + (1.0
                    + (double)ixstart)) - 1)) - 1] < 2.2204460492503131E-16) {
              uncertainty = 100.0;

              //  dont use features that we were not able to initialize
            }

            depthUncertainties->data[ixstart] = uncertainty;
          }

          ixstart = 1;
          n = depthUncertainties->size[0];
          mtmp = depthUncertainties->data[0];
          nx = 1;
          if (depthUncertainties->size[0] > 1) {
            if (rtIsNaN(depthUncertainties->data[0])) {
              idx = 2;
              exitg3 = false;
              while ((!exitg3) && (idx <= n)) {
                ixstart = idx;
                if (!rtIsNaN(depthUncertainties->data[idx - 1])) {
                  mtmp = depthUncertainties->data[idx - 1];
                  nx = idx;
                  exitg3 = true;
                } else {
                  idx++;
                }
              }
            }

            if (ixstart < depthUncertainties->size[0]) {
              while (ixstart + 1 <= n) {
                if (depthUncertainties->data[ixstart] < mtmp) {
                  mtmp = depthUncertainties->data[ixstart];
                  nx = ixstart + 1;
                }

                ixstart++;
              }
            }
          }

          fixedFeatureIdx = nx;
          loop_ub = P_apr->size[1];
          i15 = (int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0 +
            b_VIOParameters.num_points_per_anchor)) + 6.0) + fixedFeatureIdx);
          for (i16 = 0; i16 < loop_ub; i16++) {
            P_apr->data[(i15 + P_apr->size[0] * i16) - 1] = 0.0;
          }

          loop_ub = P_apr->size[0];
          i15 = (int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0 +
            b_VIOParameters.num_points_per_anchor)) + 6.0) + fixedFeatureIdx);
          for (i16 = 0; i16 < loop_ub; i16++) {
            P_apr->data[i16 + P_apr->size[0] * (i15 - 1)] = 0.0;
          }

          //              fprintf('Fixing feature %i of anchor %i\n', fixedFeatureIdx, oldestAnchorIdx); 
        }
      }
    } else {
      h_fprintf();
    }
  }

  emxFree_int32_T(&r12);
  emxFree_real_T(&anchorIdx);

  //  determine if a new anchor needs to be initialized, and request stereo
  //  measurements for it
  minNumValidFeatures = 10000;
  ixstart = 0;
  exitg2 = false;
  while ((!exitg2) && (ixstart <= (int)numAnchors - 1)) {
    for (i15 = 0; i15 < 24; i15++) {
      x[i15] = ((anchorFeatures->data[i15 + anchorFeatures->size[0] * ixstart] ==
                 1.0) && (updateVect[i15] == 1.0));
    }

    n = 0;
    for (nx = 0; nx < 24; nx++) {
      if (x[nx]) {
        n++;
      }
    }

    guard2 = false;
    if ((n < minFeatureThreshold) && (n < minNumValidFeatures)) {
      minNumValidFeatures = n;
      initializeNewAnchor = 1.0 + (double)ixstart;
      if (!(n != 0)) {
        exitg2 = true;
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

  if (minNumValidFeatures < minFeatureThreshold) {
    newFeaturesRequested = 0.0;
    i = 0;
    emxInit_boolean_T(&r15, 2);
    exitg1 = false;
    while ((!exitg1) && (i < 24)) {
      loop_ub = anchorFeatures->size[1];
      i15 = r15->size[0] * r15->size[1];
      r15->size[0] = 1;
      r15->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)r15, i15, (int)sizeof(boolean_T));
      for (i15 = 0; i15 < loop_ub; i15++) {
        r15->data[r15->size[0] * i15] = (anchorFeatures->data[i +
          anchorFeatures->size[0] * i15] == 1.0);
      }

      b_guard1 = false;
      if ((!d_any(r15)) || (anchorFeatures->data[i + anchorFeatures->size[0] *
                            ((int)initializeNewAnchor - 1)] == 1.0)) {
        updateVect[i] = 2.0;
        newFeaturesRequested++;
        if (newFeaturesRequested == numPointsPerAnchor) {
          exitg1 = true;
        } else {
          b_guard1 = true;
        }
      } else {
        b_guard1 = true;
      }

      if (b_guard1) {
        i++;
      }
    }

    emxFree_boolean_T(&r15);

    //      fprintf('Requesting %i new features\n', int8(newFeaturesRequested))
  }

  b_emxInit_real_T(&unusedU4, 1);

  // % aposteriori measurement prediction
  getMap(b_xt, anchorFeatures, m_vect, c_numTrackFeatures, numStatesxt, 7.0 +
         b_VIOParameters.num_points_per_anchor, map, depthUncertainties,
         unusedU4);
  i15 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(c_numTrackFeatures * 4.0);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i15, (int)sizeof(double));
  loop_ub = (int)(c_numTrackFeatures * 4.0);
  emxFree_real_T(&unusedU4);
  emxFree_real_T(&depthUncertainties);
  for (i15 = 0; i15 < loop_ub; i15++) {
    h_u_apo->data[i15] = rtNaN;
  }

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
  emxInit_boolean_T(&r16, 2);
  for (i = 0; i < 24; i++) {
    loop_ub = anchorFeatures->size[1];
    i15 = r16->size[0] * r16->size[1];
    r16->size[0] = 1;
    r16->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)r16, i15, (int)sizeof(boolean_T));
    for (i15 = 0; i15 < loop_ub; i15++) {
      r16->data[r16->size[0] * i15] = (anchorFeatures->data[i +
        anchorFeatures->size[0] * i15] == 1.0);
    }

    if (d_any(r16)) {
      for (i15 = 0; i15 < 3; i15++) {
        b_fp[i15] = map->data[i15 + map->size[0] * i] - b_xt->data[i15];
      }

      for (i15 = 0; i15 < 3; i15++) {
        b_R_cw[i15] = 0.0;
        for (i16 = 0; i16 < 3; i16++) {
          b_R_cw[i15] += R_cw[i15 + 3 * i16] * b_fp[i16];
        }
      }

      predictMeasurement_stereo(b_R_cw, c_cameraParams_CameraParameters,
        d_cameraParams_CameraParameters, e_cameraParams_CameraParameters,
        f_cameraParams_CameraParameters, g_cameraParams_CameraParameters,
        h_cameraParams_CameraParameters, cameraParams_r_lr, cameraParams_R_rl,
        h_u_l, h_u_r);
      ixstart = i << 2;
      for (i15 = 0; i15 < 2; i15++) {
        h_u_apo->data[i15 + ixstart] = h_u_l[i15];
      }

      for (i15 = 0; i15 < 2; i15++) {
        h_u_apo->data[(i15 + ixstart) + 2] = h_u_r[i15];
      }
    }
  }

  emxFree_boolean_T(&r16);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_updIT_free()
{
  emxFree_real_T(&anchorFeatures);
  emxFree_real_T(&m_vect);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_updIT_init()
{
  emxInit_real_T(&anchorFeatures, 2);
  emxInit_real_T(&m_vect, 2);
  m_vect_not_empty = false;
}

//
// File trailer for SLAM_updIT.cpp
//
// [EOF]
//
