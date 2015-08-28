//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 11:22:17
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
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
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
//                double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                const double noiseParameters_image_noise[2]
//                double noiseParameters_sigmaInit
//                double c_noiseParameters_orientation_n
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                const VIOMeasurements *measurements
//                double height_offset_pressure
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
                double cameraParams_R_rl[9], double updateVect[16], const double
                z_all_l[32], const double z_all_r[32], const double
                noiseParameters_image_noise[2], double noiseParameters_sigmaInit,
                double c_noiseParameters_orientation_n, double
                noiseParameters_pressure_noise, double
                noiseParameters_ext_pos_noise, double
                noiseParameters_ext_att_noise, const VIOMeasurements
                *measurements, double height_offset_pressure, const
                VIOParameters b_VIOParameters, emxArray_real_T *h_u_apo,
                emxArray_real_T *map)
{
  double numAnchors;
  double numPointsPerAnchor;
  double numStatesPerAnchor;
  double c_numTrackFeatures;
  int i8;
  int loop_ub;
  int i;
  emxArray_real_T *anchorIdx;
  emxArray_int32_T *r6;
  emxArray_boolean_T *x;
  emxArray_int32_T *ii;
  emxArray_real_T *b_anchorIdx;
  int nx;
  int idx;
  int ixstart;
  boolean_T exitg10;
  boolean_T guard7 = false;
  int n;
  int i9;
  int unnamed_idx_0;
  emxArray_real_T *depthUncertainties;
  boolean_T b_x[16];
  int ii_data[16];
  double uncertainty;
  double mtmp;
  boolean_T exitg9;
  emxArray_boolean_T *r7;
  boolean_T exitg8;
  boolean_T guard6 = false;
  signed char indMeas_data[16];
  int i10;
  int validFeatures_size[1];
  double validFeatures_data[16];
  emxArray_real_T *c_anchorIdx;
  boolean_T exitg7;
  boolean_T guard5 = false;
  double initializeNewAnchor;
  int minNumValidFeatures;
  boolean_T exitg6;
  boolean_T guard4 = false;
  double R_cw[9];
  double r_wc[3];
  double d6;
  double c_xt[7];
  emxArray_boolean_T *r8;
  boolean_T exitg5;
  boolean_T guard3 = false;
  int ii_size_idx_0;
  unsigned int featureAnchorIdx;
  int unusedFeatureIdx;
  double z_curr_l[2];
  double z_curr_r[2];
  double m[3];
  double fp[3];
  boolean_T bv0[3];
  signed char i11;
  double b_R_cw[3];
  double b_fp[3];
  double h_u_r[2];
  double h_u_l[2];
  double b_h_u_l[2];
  boolean_T guard1 = false;
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
  emxArray_boolean_T *r9;
  boolean_T exitg1;
  boolean_T b_guard1 = false;
  emxArray_real_T *unusedU4;
  emxArray_boolean_T *r10;
  numAnchors = b_VIOParameters.num_anchors;
  numPointsPerAnchor = b_VIOParameters.num_points_per_anchor;
  numStatesPerAnchor = 6.0 + b_VIOParameters.num_points_per_anchor;
  c_numTrackFeatures = b_VIOParameters.num_anchors *
    b_VIOParameters.num_points_per_anchor;
  if (!m_vect_not_empty) {
    i8 = m_vect->size[0] * m_vect->size[1];
    m_vect->size[0] = 3;
    m_vect->size[1] = (int)(b_VIOParameters.num_points_per_anchor *
      b_VIOParameters.num_anchors);
    emxEnsureCapacity((emxArray__common *)m_vect, i8, (int)sizeof(double));
    loop_ub = 3 * (int)(b_VIOParameters.num_points_per_anchor *
                        b_VIOParameters.num_anchors);
    for (i8 = 0; i8 < loop_ub; i8++) {
      m_vect->data[i8] = rtNaN;
    }

    m_vect_not_empty = !(m_vect->size[1] == 0);

    //  a matrix containing the m vectors for each feature
    i8 = anchorFeatures->size[0] * anchorFeatures->size[1];
    anchorFeatures->size[0] = 16;
    anchorFeatures->size[1] = (int)b_VIOParameters.num_anchors;
    emxEnsureCapacity((emxArray__common *)anchorFeatures, i8, (int)sizeof(double));
    loop_ub = (int)b_VIOParameters.num_anchors << 4;
    for (i8 = 0; i8 < loop_ub; i8++) {
      anchorFeatures->data[i8] = 0.0;
    }

    //  describes which feature belongs to which anchor
    for (i = 0; i < 4; i++) {
      anchorAges[i] = 0.0;
    }

    fixedFeatureIdx = 0.0;
    oldestAnchorIdx = 0.0;
  }

  //  debug check
  // % check for lost features
  i = 0;
  emxInit_real_T(&anchorIdx, 2);
  emxInit_int32_T(&r6, 1);
  emxInit_boolean_T(&x, 2);
  b_emxInit_int32_T(&ii, 2);
  b_emxInit_real_T(&b_anchorIdx, 1);
  while (i <= (int)c_numTrackFeatures - 1) {
    loop_ub = anchorFeatures->size[1];
    i8 = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)x, i8, (int)sizeof(boolean_T));
    for (i8 = 0; i8 < loop_ub; i8++) {
      x->data[x->size[0] * i8] = (anchorFeatures->data[i + anchorFeatures->size
        [0] * i8] == 1.0);
    }

    nx = x->size[1];
    idx = 0;
    i8 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = x->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i8, (int)sizeof(int));
    ixstart = 1;
    exitg10 = false;
    while ((!exitg10) && (ixstart <= nx)) {
      guard7 = false;
      if (x->data[ixstart - 1]) {
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

    if (x->size[1] == 1) {
      if (idx == 0) {
        i8 = ii->size[0] * ii->size[1];
        ii->size[0] = 1;
        ii->size[1] = 0;
        emxEnsureCapacity((emxArray__common *)ii, i8, (int)sizeof(int));
      }
    } else {
      i8 = ii->size[0] * ii->size[1];
      if (1 > idx) {
        ii->size[1] = 0;
      } else {
        ii->size[1] = idx;
      }

      emxEnsureCapacity((emxArray__common *)ii, i8, (int)sizeof(int));
    }

    i8 = anchorIdx->size[0] * anchorIdx->size[1];
    anchorIdx->size[0] = 1;
    anchorIdx->size[1] = ii->size[1];
    emxEnsureCapacity((emxArray__common *)anchorIdx, i8, (int)sizeof(double));
    loop_ub = ii->size[0] * ii->size[1];
    for (i8 = 0; i8 < loop_ub; i8++) {
      anchorIdx->data[i8] = ii->data[i8];
    }

    if ((!(anchorIdx->size[1] == 0)) && (updateVect[i] != 1.0)) {
      n = 0;
      i8 = b_anchorIdx->size[0];
      b_anchorIdx->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)b_anchorIdx, i8, (int)sizeof(double));
      loop_ub = anchorIdx->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        b_anchorIdx->data[i8] = anchorIdx->data[anchorIdx->size[0] * i8];
      }

      i8 = (i + 1) * b_anchorIdx->size[0];
      for (nx = 0; nx < i8; nx++) {
        i9 = r6->size[0];
        r6->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r6, i9, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i9 = 0; i9 < loop_ub; i9++) {
          r6->data[i9] = (int)anchorIdx->data[anchorIdx->size[0] * i9];
        }

        if (anchorFeatures->data[nx % (i + 1) + anchorFeatures->size[0] *
            (r6->data[div_nzp_s32_floor(nx, i + 1)] - 1)] != 0.0) {
          n++;
        }
      }

      //  remove covariance of this feature with rest of state
      i8 = r6->size[0];
      r6->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        r6->data[i8] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0] *
          i8] - 1.0) * numStatesPerAnchor) + 6.0) + (double)n);
      }

      loop_ub = P_apr->size[1];
      unnamed_idx_0 = r6->size[0];
      for (i8 = 0; i8 < loop_ub; i8++) {
        for (i9 = 0; i9 < unnamed_idx_0; i9++) {
          P_apr->data[(r6->data[i9] + P_apr->size[0] * i8) - 1] = 0.0;
        }
      }

      i8 = r6->size[0];
      r6->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        r6->data[i8] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0] *
          i8] - 1.0) * numStatesPerAnchor) + 6.0) + (double)n);
      }

      loop_ub = P_apr->size[0];
      ixstart = r6->size[0];
      for (i8 = 0; i8 < ixstart; i8++) {
        for (i9 = 0; i9 < loop_ub; i9++) {
          P_apr->data[i9 + P_apr->size[0] * (r6->data[i8] - 1)] = 0.0;
        }
      }

      i8 = r6->size[0];
      r6->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        r6->data[i8] = (int)anchorIdx->data[anchorIdx->size[0] * i8];
      }

      ixstart = r6->size[0];
      for (i8 = 0; i8 < ixstart; i8++) {
        anchorFeatures->data[i + anchorFeatures->size[0] * (r6->data[i8] - 1)] =
          -1.0;
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
    for (i8 = 0; i8 < 16; i8++) {
      b_x[i8] = (anchorFeatures->data[i8 + anchorFeatures->size[0] * ((int)
                  oldestAnchorIdx - 1)] != 0.0);
    }

    nx = 0;
    for (i = 0; i < 16; i++) {
      if (b_x[i]) {
        nx++;
      }
    }

    if (!(nx == 0)) {
      //  otherwise the anchor hasnt been initialized (only happens at the very beginning) 
      for (i8 = 0; i8 < 16; i8++) {
        b_x[i8] = (anchorFeatures->data[i8 + anchorFeatures->size[0] * ((int)
                    oldestAnchorIdx - 1)] != 0.0);
      }

      ixstart = 0;
      for (i = 0; i < 16; i++) {
        if (b_x[i]) {
          ii_data[ixstart] = i + 1;
          ixstart++;
        }
      }

      if (anchorFeatures->data[(ii_data[(int)fixedFeatureIdx - 1] +
           anchorFeatures->size[0] * ((int)oldestAnchorIdx - 1)) - 1] != 1.0) {
        //  the feature was lost
        //  set the depth uncertainty of the most-converged feature of the
        //  oldest anchor to 0
        i8 = depthUncertainties->size[0];
        depthUncertainties->size[0] = (int)b_VIOParameters.num_points_per_anchor;
        emxEnsureCapacity((emxArray__common *)depthUncertainties, i8, (int)
                          sizeof(double));
        loop_ub = (int)b_VIOParameters.num_points_per_anchor;
        for (i8 = 0; i8 < loop_ub; i8++) {
          depthUncertainties->data[i8] = 0.0;
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
        i8 = (int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0 +
          b_VIOParameters.num_points_per_anchor)) + 6.0) + fixedFeatureIdx);
        for (i9 = 0; i9 < loop_ub; i9++) {
          P_apr->data[(i8 + P_apr->size[0] * i9) - 1] = 0.0;
        }

        loop_ub = P_apr->size[0];
        i8 = (int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0 +
          b_VIOParameters.num_points_per_anchor)) + 6.0) + fixedFeatureIdx);
        for (i9 = 0; i9 < loop_ub; i9++) {
          P_apr->data[i9 + P_apr->size[0] * (i8 - 1)] = 0.0;
        }

        //              fprintf('Fixing feature %i of anchor %i\n', fixedFeatureIdx, oldestAnchorIdx); 
      }
    }
  }

  emxInit_boolean_T(&r7, 2);

  // % do the update
  i8 = r7->size[0] * r7->size[1];
  r7->size[0] = 16;
  r7->size[1] = anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)r7, i8, (int)sizeof(boolean_T));
  loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
  for (i8 = 0; i8 < loop_ub; i8++) {
    r7->data[i8] = (anchorFeatures->data[i8] == 1.0);
  }

  b_any(r7, b_x);
  idx = 0;
  ixstart = 1;
  emxFree_boolean_T(&r7);
  exitg8 = false;
  while ((!exitg8) && (ixstart < 17)) {
    guard6 = false;
    if (b_x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = ixstart;
      if (idx >= 16) {
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

  for (i8 = 0; i8 < loop_ub; i8++) {
    indMeas_data[i8] = (signed char)ii_data[i8];
  }

  if (1 > idx) {
    i10 = 0;
  } else {
    i10 = idx;
  }

  if (!(i10 == 0)) {
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, numStatesxt, numStates,
                       c_cameraParams_CameraParameters,
                       d_cameraParams_CameraParameters,
                       e_cameraParams_CameraParameters, anchorFeatures, m_vect,
                       noiseParameters_image_noise,
                       c_noiseParameters_orientation_n,
                       noiseParameters_pressure_noise,
                       noiseParameters_ext_pos_noise,
                       noiseParameters_ext_att_noise, measurements,
                       height_offset_pressure, b_VIOParameters,
                       validFeatures_data, validFeatures_size);
    for (i8 = 0; i8 < loop_ub; i8++) {
      ii_data[i8] = indMeas_data[i8];
    }

    for (i8 = 0; i8 < loop_ub; i8++) {
      updateVect[ii_data[i8] - 1] = 0.0;
    }

    loop_ub = validFeatures_size[0];
    for (i8 = 0; i8 < loop_ub; i8++) {
      ii_data[i8] = (int)validFeatures_data[i8];
    }

    loop_ub = validFeatures_size[0];
    for (i8 = 0; i8 < loop_ub; i8++) {
      updateVect[ii_data[i8] - 1] = 1.0;
    }

    //  check for lost features
    i = 0;
    b_emxInit_real_T(&c_anchorIdx, 1);
    while (i <= (int)c_numTrackFeatures - 1) {
      loop_ub = anchorFeatures->size[1];
      i8 = x->size[0] * x->size[1];
      x->size[0] = 1;
      x->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)x, i8, (int)sizeof(boolean_T));
      for (i8 = 0; i8 < loop_ub; i8++) {
        x->data[x->size[0] * i8] = (anchorFeatures->data[i +
          anchorFeatures->size[0] * i8] == 1.0);
      }

      nx = x->size[1];
      idx = 0;
      i8 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = x->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i8, (int)sizeof(int));
      ixstart = 1;
      exitg7 = false;
      while ((!exitg7) && (ixstart <= nx)) {
        guard5 = false;
        if (x->data[ixstart - 1]) {
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

      if (x->size[1] == 1) {
        if (idx == 0) {
          i8 = ii->size[0] * ii->size[1];
          ii->size[0] = 1;
          ii->size[1] = 0;
          emxEnsureCapacity((emxArray__common *)ii, i8, (int)sizeof(int));
        }
      } else {
        i8 = ii->size[0] * ii->size[1];
        if (1 > idx) {
          ii->size[1] = 0;
        } else {
          ii->size[1] = idx;
        }

        emxEnsureCapacity((emxArray__common *)ii, i8, (int)sizeof(int));
      }

      i8 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = ii->size[1];
      emxEnsureCapacity((emxArray__common *)anchorIdx, i8, (int)sizeof(double));
      loop_ub = ii->size[0] * ii->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        anchorIdx->data[i8] = ii->data[i8];
      }

      if ((!(anchorIdx->size[1] == 0)) && (updateVect[i] != 1.0)) {
        n = 0;
        i8 = c_anchorIdx->size[0];
        c_anchorIdx->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)c_anchorIdx, i8, (int)sizeof
                          (double));
        loop_ub = anchorIdx->size[1];
        for (i8 = 0; i8 < loop_ub; i8++) {
          c_anchorIdx->data[i8] = anchorIdx->data[anchorIdx->size[0] * i8];
        }

        i8 = (i + 1) * c_anchorIdx->size[0];
        for (nx = 0; nx < i8; nx++) {
          i9 = r6->size[0];
          r6->size[0] = anchorIdx->size[1];
          emxEnsureCapacity((emxArray__common *)r6, i9, (int)sizeof(int));
          loop_ub = anchorIdx->size[1];
          for (i9 = 0; i9 < loop_ub; i9++) {
            r6->data[i9] = (int)anchorIdx->data[anchorIdx->size[0] * i9];
          }

          if (anchorFeatures->data[nx % (i + 1) + anchorFeatures->size[0] *
              (r6->data[div_nzp_s32_floor(nx, i + 1)] - 1)] != 0.0) {
            n++;
          }
        }

        //  remove covariance of this feature with rest of state
        i8 = r6->size[0];
        r6->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i8 = 0; i8 < loop_ub; i8++) {
          r6->data[i8] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0]
            * i8] - 1.0) * numStatesPerAnchor) + 6.0) + (double)n);
        }

        loop_ub = P_apr->size[1];
        unnamed_idx_0 = r6->size[0];
        for (i8 = 0; i8 < loop_ub; i8++) {
          for (i9 = 0; i9 < unnamed_idx_0; i9++) {
            P_apr->data[(r6->data[i9] + P_apr->size[0] * i8) - 1] = 0.0;
          }
        }

        i8 = r6->size[0];
        r6->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i8 = 0; i8 < loop_ub; i8++) {
          r6->data[i8] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0]
            * i8] - 1.0) * numStatesPerAnchor) + 6.0) + (double)n);
        }

        loop_ub = P_apr->size[0];
        ixstart = r6->size[0];
        for (i8 = 0; i8 < ixstart; i8++) {
          for (i9 = 0; i9 < loop_ub; i9++) {
            P_apr->data[i9 + P_apr->size[0] * (r6->data[i8] - 1)] = 0.0;
          }
        }

        i8 = r6->size[0];
        r6->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i8 = 0; i8 < loop_ub; i8++) {
          r6->data[i8] = (int)anchorIdx->data[anchorIdx->size[0] * i8];
        }

        ixstart = r6->size[0];
        for (i8 = 0; i8 < ixstart; i8++) {
          anchorFeatures->data[i + anchorFeatures->size[0] * (r6->data[i8] - 1)]
            = -1.0;
        }

        //  mark feature as lost
      }

      i++;
    }

    emxFree_real_T(&c_anchorIdx);
  }

  emxFree_int32_T(&ii);
  emxFree_boolean_T(&x);

  // % Initialize new anchors
  for (i = 0; i < 16; i++) {
    b_x[i] = (updateVect[i] == 2.0);
  }

  if (any(b_x)) {
    //  if there are any features with stereo measurements
    initializeNewAnchor = 0.0;
    minNumValidFeatures = 10000;
    ixstart = 0;
    exitg6 = false;
    while ((!exitg6) && (ixstart <= (int)numAnchors - 1)) {
      for (i8 = 0; i8 < 16; i8++) {
        b_x[i8] = ((anchorFeatures->data[i8 + anchorFeatures->size[0] * ixstart]
                    == 1.0) && (updateVect[i8] == 1.0));
      }

      n = 0;
      for (nx = 0; nx < 16; nx++) {
        if (b_x[nx]) {
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
      for (i8 = 0; i8 < 3; i8++) {
        r_wc[i8] = b_xt->data[i8];
      }

      for (i8 = 0; i8 < 16; i8++) {
        b_x[i8] = (anchorFeatures->data[i8 + anchorFeatures->size[0] * ((int)
                    initializeNewAnchor - 1)] == 1.0);
      }

      nx = 0;
      for (i = 0; i < 16; i++) {
        if (b_x[i]) {
          nx++;
        }
      }

      ixstart = 0;
      for (i = 0; i < 16; i++) {
        if (b_x[i]) {
          ii_data[ixstart] = i + 1;
          ixstart++;
        }
      }

      for (i8 = 0; i8 < nx; i8++) {
        for (i9 = 0; i9 < 3; i9++) {
          m_vect->data[i9 + m_vect->size[0] * (ii_data[i8] - 1)] = rtNaN;
        }
      }

      for (i8 = 0; i8 < 16; i8++) {
        anchorFeatures->data[i8 + anchorFeatures->size[0] * ((int)
          initializeNewAnchor - 1)] = 0.0;
      }

      d6 = numStatesxt + (initializeNewAnchor - 1.0) * (7.0 +
        b_VIOParameters.num_points_per_anchor);
      for (i8 = 0; i8 < 7; i8++) {
        c_xt[i8] = b_xt->data[i8];
      }

      for (i8 = 0; i8 < 7; i8++) {
        b_xt->data[(int)(d6 + (1.0 + (double)i8)) - 1] = c_xt[i8];
      }

      i8 = depthUncertainties->size[0];
      depthUncertainties->size[0] = (int)b_VIOParameters.num_points_per_anchor;
      emxEnsureCapacity((emxArray__common *)depthUncertainties, i8, (int)sizeof
                        (double));
      loop_ub = (int)b_VIOParameters.num_points_per_anchor;
      for (i8 = 0; i8 < loop_ub; i8++) {
        depthUncertainties->data[i8] = 0.0;
      }

      emxInit_boolean_T(&r8, 2);
      i8 = r8->size[0] * r8->size[1];
      r8->size[0] = 16;
      r8->size[1] = anchorFeatures->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i8, (int)sizeof(boolean_T));
      loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        r8->data[i8] = (anchorFeatures->data[i8] == 1.0);
      }

      b_any(r8, b_x);
      emxFree_boolean_T(&r8);
      for (i8 = 0; i8 < 16; i8++) {
        b_x[i8] = !b_x[i8];
      }

      idx = 0;
      ixstart = 1;
      exitg5 = false;
      while ((!exitg5) && (ixstart < 17)) {
        guard3 = false;
        if (b_x[ixstart - 1] && (updateVect[ixstart - 1] == 2.0)) {
          idx++;
          ii_data[idx - 1] = ixstart;
          if (idx >= 16) {
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

      for (i8 = 0; i8 < loop_ub; i8++) {
        indMeas_data[i8] = (signed char)ii_data[i8];
      }

      featureAnchorIdx = 1U;
      unusedFeatureIdx = 0;
      while ((unusedFeatureIdx <= ii_size_idx_0 - 1) && (!(featureAnchorIdx >
               numPointsPerAnchor))) {
        ixstart = (indMeas_data[unusedFeatureIdx] - 1) * 2;
        nx = (indMeas_data[unusedFeatureIdx] - 1) * 2;
        for (i8 = 0; i8 < 2; i8++) {
          z_curr_l[i8] = z_all_l[ixstart + i8];
          z_curr_r[i8] = z_all_r[nx + i8];
        }

        initializePoint(b_xt, c_cameraParams_CameraParameters,
                        d_cameraParams_CameraParameters,
                        e_cameraParams_CameraParameters,
                        f_cameraParams_CameraParameters,
                        g_cameraParams_CameraParameters,
                        h_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, z_curr_l, z_curr_r, fp, m);
        for (i = 0; i < 3; i++) {
          bv0[i] = rtIsNaN(fp[i]);
        }

        if (c_any(bv0)) {
          updateVect[indMeas_data[unusedFeatureIdx] - 1] = 0.0;
          i8 = (int)rt_roundd_snf((double)indMeas_data[unusedFeatureIdx]);
          i11 = (signed char)i8;
          f_fprintf(i11);
        } else {
          //  check reprojection error
          for (i8 = 0; i8 < 3; i8++) {
            b_fp[i8] = fp[i8] - r_wc[i8];
          }

          for (i8 = 0; i8 < 3; i8++) {
            b_R_cw[i8] = 0.0;
            for (i9 = 0; i9 < 3; i9++) {
              b_R_cw[i8] += R_cw[i8 + 3 * i9] * b_fp[i9];
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
              for (i8 = 0; i8 < 3; i8++) {
                m_vect->data[i8 + m_vect->size[0] *
                  (indMeas_data[unusedFeatureIdx] - 1)] = m[i8];
              }

              for (i8 = 0; i8 < 3; i8++) {
                b_fp[i8] = b_xt->data[i8] - fp[i8];
              }

              b_xt->data[(int)(((numStatesxt + (initializeNewAnchor - 1.0) *
                                 (7.0 + numPointsPerAnchor)) + 7.0) + (double)
                               featureAnchorIdx) - 1] = 1.0 / norm(b_fp);
              depthUncertainties->data[(int)featureAnchorIdx - 1] =
                noiseParameters_sigmaInit;
              anchorFeatures->data[(indMeas_data[unusedFeatureIdx] +
                                    anchorFeatures->size[0] * ((int)
                initializeNewAnchor - 1)) - 1] = 1.0;
              updateVect[indMeas_data[unusedFeatureIdx] - 1] = 1.0;
              featureAnchorIdx++;
            }
          }

          if (guard1) {
            i8 = (int)rt_roundd_snf((double)indMeas_data[unusedFeatureIdx]);
            i11 = (signed char)i8;
            h_fprintf(i11);
            updateVect[indMeas_data[unusedFeatureIdx] - 1] = 0.0;
          }
        }

        unusedFeatureIdx++;
      }

      d6 = rt_roundd_snf((double)featureAnchorIdx - 1.0);
      if (d6 < 2.147483648E+9) {
        i8 = (int)d6;
      } else {
        i8 = MAX_int32_T;
      }

      j_fprintf(i8, (int)initializeNewAnchor);
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

      i8 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i8, (int)sizeof(double));
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

      d6 = numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_VIOParameters.num_points_per_anchor);
      i8 = r6->size[0];
      r6->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        r6->data[i8] = (int)(d6 + anchorIdx->data[anchorIdx->size[0] * i8]);
      }

      loop_ub = P_apr->size[1];
      unnamed_idx_0 = r6->size[0];
      for (i8 = 0; i8 < loop_ub; i8++) {
        for (i9 = 0; i9 < unnamed_idx_0; i9++) {
          P_apr->data[(r6->data[i9] + P_apr->size[0] * i8) - 1] = 0.0;
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

      i8 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i8, (int)sizeof(double));
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

      d6 = numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_VIOParameters.num_points_per_anchor);
      i8 = r6->size[0];
      r6->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        r6->data[i8] = (int)(d6 + anchorIdx->data[anchorIdx->size[0] * i8]);
      }

      loop_ub = P_apr->size[0];
      ixstart = r6->size[0];
      for (i8 = 0; i8 < ixstart; i8++) {
        for (i9 = 0; i9 < loop_ub; i9++) {
          P_apr->data[i9 + P_apr->size[0] * (r6->data[i8] - 1)] = 0.0;
        }
      }

      for (i8 = 0; i8 < 2; i8++) {
        b_h_u_l[i8] = P_apr->size[i8];
      }

      emxInit_real_T(&J, 2);
      b_eye(b_h_u_l, J);
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

      i8 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i8, (int)sizeof(double));
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

      d6 = numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_VIOParameters.num_points_per_anchor);
      i8 = r6->size[0];
      r6->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        r6->data[i8] = (int)(d6 + anchorIdx->data[anchorIdx->size[0] * i8]);
      }

      loop_ub = J->size[1];
      unnamed_idx_0 = r6->size[0];
      for (i8 = 0; i8 < loop_ub; i8++) {
        for (i9 = 0; i9 < unnamed_idx_0; i9++) {
          J->data[(r6->data[i9] + J->size[0] * i8) - 1] = 0.0;
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

      i8 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i8, (int)sizeof(double));
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

      d6 = numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_VIOParameters.num_points_per_anchor);
      i8 = r6->size[0];
      r6->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r6, i8, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i8 = 0; i8 < loop_ub; i8++) {
        r6->data[i8] = (int)(d6 + anchorIdx->data[anchorIdx->size[0] * i8]) - 1;
      }

      for (i8 = 0; i8 < 3; i8++) {
        for (i9 = 0; i9 < 3; i9++) {
          J->data[r6->data[i9] + J->size[0] * i8] = iv12[i9 + 3 * i8];
        }
      }

      for (i8 = 0; i8 < 3; i8++) {
        for (i9 = 0; i9 < 3; i9++) {
          J->data[r6->data[i9] + J->size[0] * (i8 + 3)] = 0.0;
        }
      }

      loop_ub = (int)(numStates - 6.0);
      for (i8 = 0; i8 < loop_ub; i8++) {
        for (i9 = 0; i9 < 3; i9++) {
          J->data[r6->data[i9] + J->size[0] * (i8 + 6)] = 0.0;
        }
      }

      for (i8 = 0; i8 < 3; i8++) {
        for (i9 = 0; i9 < 3; i9++) {
          J->data[r6->data[i9 + 3] + J->size[0] * i8] = 0.0;
        }
      }

      for (i8 = 0; i8 < 3; i8++) {
        for (i9 = 0; i9 < 3; i9++) {
          J->data[r6->data[i9 + 3] + J->size[0] * (i8 + 3)] = iv12[i9 + 3 * i8];
        }
      }

      loop_ub = (int)(numStates - 6.0);
      for (i8 = 0; i8 < loop_ub; i8++) {
        for (i9 = 0; i9 < 3; i9++) {
          J->data[r6->data[i9 + 3] + J->size[0] * (i8 + 6)] = 0.0;
        }
      }

      loop_ub = (int)numStates;
      for (i8 = 0; i8 < loop_ub; i8++) {
        ixstart = (int)b_VIOParameters.num_points_per_anchor;
        for (i9 = 0; i9 < ixstart; i9++) {
          J->data[r6->data[i9 + 6] + J->size[0] * i8] = 0.0;
        }
      }

      emxInit_real_T(&y, 2);
      if ((J->size[1] == 1) || (P_apr->size[0] == 1)) {
        i8 = y->size[0] * y->size[1];
        y->size[0] = J->size[0];
        y->size[1] = P_apr->size[1];
        emxEnsureCapacity((emxArray__common *)y, i8, (int)sizeof(double));
        loop_ub = J->size[0];
        for (i8 = 0; i8 < loop_ub; i8++) {
          ixstart = P_apr->size[1];
          for (i9 = 0; i9 < ixstart; i9++) {
            y->data[i8 + y->size[0] * i9] = 0.0;
            idx = J->size[1];
            for (n = 0; n < idx; n++) {
              y->data[i8 + y->size[0] * i9] += J->data[i8 + J->size[0] * n] *
                P_apr->data[n + P_apr->size[0] * i9];
            }
          }
        }
      } else {
        nx = J->size[1];
        unnamed_idx_0 = J->size[0];
        ixstart = P_apr->size[1];
        b_m = J->size[0];
        i8 = y->size[0] * y->size[1];
        y->size[0] = unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)y, i8, (int)sizeof(double));
        i8 = y->size[0] * y->size[1];
        y->size[1] = ixstart;
        emxEnsureCapacity((emxArray__common *)y, i8, (int)sizeof(double));
        loop_ub = unnamed_idx_0 * ixstart;
        for (i8 = 0; i8 < loop_ub; i8++) {
          y->data[i8] = 0.0;
        }

        if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
        } else {
          ixstart = J->size[0] * (P_apr->size[1] - 1);
          cr = 0;
          while ((b_m > 0) && (cr <= ixstart)) {
            i8 = cr + b_m;
            for (ic = cr; ic + 1 <= i8; ic++) {
              y->data[ic] = 0.0;
            }

            cr += b_m;
          }

          br = 0;
          cr = 0;
          while ((b_m > 0) && (cr <= ixstart)) {
            ar = 0;
            i8 = br + nx;
            for (ib = br; ib + 1 <= i8; ib++) {
              if (P_apr->data[ib] != 0.0) {
                ia = ar;
                i9 = cr + b_m;
                for (ic = cr; ic + 1 <= i9; ic++) {
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
      i8 = b->size[0] * b->size[1];
      b->size[0] = J->size[1];
      b->size[1] = J->size[0];
      emxEnsureCapacity((emxArray__common *)b, i8, (int)sizeof(double));
      loop_ub = J->size[0];
      for (i8 = 0; i8 < loop_ub; i8++) {
        ixstart = J->size[1];
        for (i9 = 0; i9 < ixstart; i9++) {
          b->data[i9 + b->size[0] * i8] = J->data[i8 + J->size[0] * i9];
        }
      }

      emxFree_real_T(&J);
      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i8 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = y->size[0];
        P_apr->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i8, (int)sizeof(double));
        loop_ub = y->size[0];
        for (i8 = 0; i8 < loop_ub; i8++) {
          ixstart = b->size[1];
          for (i9 = 0; i9 < ixstart; i9++) {
            P_apr->data[i8 + P_apr->size[0] * i9] = 0.0;
            idx = y->size[1];
            for (n = 0; n < idx; n++) {
              P_apr->data[i8 + P_apr->size[0] * i9] += y->data[i8 + y->size[0] *
                n] * b->data[n + b->size[0] * i9];
            }
          }
        }
      } else {
        nx = y->size[1];
        unnamed_idx_0 = y->size[0];
        ixstart = b->size[1];
        b_m = y->size[0];
        i8 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = unnamed_idx_0;
        P_apr->size[1] = ixstart;
        emxEnsureCapacity((emxArray__common *)P_apr, i8, (int)sizeof(double));
        for (i8 = 0; i8 < ixstart; i8++) {
          for (i9 = 0; i9 < unnamed_idx_0; i9++) {
            P_apr->data[i9 + P_apr->size[0] * i8] = 0.0;
          }
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ixstart = y->size[0] * (b->size[1] - 1);
          cr = 0;
          while ((b_m > 0) && (cr <= ixstart)) {
            i8 = cr + b_m;
            for (ic = cr; ic + 1 <= i8; ic++) {
              P_apr->data[ic] = 0.0;
            }

            cr += b_m;
          }

          br = 0;
          cr = 0;
          while ((b_m > 0) && (cr <= ixstart)) {
            ar = 0;
            i8 = br + nx;
            for (ib = br; ib + 1 <= i8; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i9 = cr + b_m;
                for (ic = cr; ic + 1 <= i9; ic++) {
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
        for (i8 = 0; i8 < 4; i8++) {
          anchorAges[i8]++;
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
          d6 = numStates + (oldestAnchorIdx - 1.0) * (6.0 +
            b_VIOParameters.num_points_per_anchor);
          for (i8 = 0; i8 < loop_ub; i8++) {
            for (i9 = 0; i9 < 3; i9++) {
              P_apr->data[((int)(d6 + (1.0 + (double)i9)) + P_apr->size[0] * i8)
                - 1] = 0.0;
            }
          }

          loop_ub = P_apr->size[0];
          d6 = numStates + (oldestAnchorIdx - 1.0) * (6.0 +
            b_VIOParameters.num_points_per_anchor);
          for (i8 = 0; i8 < 3; i8++) {
            for (i9 = 0; i9 < loop_ub; i9++) {
              P_apr->data[i9 + P_apr->size[0] * ((int)(d6 + (1.0 + (double)i8))
                - 1)] = 0.0;
            }
          }

          //  set the depth uncertainty of the most-converged feature of the
          //  oldest anchor to 0
          i8 = depthUncertainties->size[0];
          depthUncertainties->size[0] = (int)
            b_VIOParameters.num_points_per_anchor;
          emxEnsureCapacity((emxArray__common *)depthUncertainties, i8, (int)
                            sizeof(double));
          loop_ub = (int)b_VIOParameters.num_points_per_anchor;
          for (i8 = 0; i8 < loop_ub; i8++) {
            depthUncertainties->data[i8] = 0.0;
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
          i8 = (int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0 +
            b_VIOParameters.num_points_per_anchor)) + 6.0) + fixedFeatureIdx);
          for (i9 = 0; i9 < loop_ub; i9++) {
            P_apr->data[(i8 + P_apr->size[0] * i9) - 1] = 0.0;
          }

          loop_ub = P_apr->size[0];
          i8 = (int)(((numStates + (oldestAnchorIdx - 1.0) * (6.0 +
            b_VIOParameters.num_points_per_anchor)) + 6.0) + fixedFeatureIdx);
          for (i9 = 0; i9 < loop_ub; i9++) {
            P_apr->data[i9 + P_apr->size[0] * (i8 - 1)] = 0.0;
          }

          //              fprintf('Fixing feature %i of anchor %i\n', fixedFeatureIdx, oldestAnchorIdx); 
        }
      }
    } else {
      l_fprintf();
    }
  }

  emxFree_int32_T(&r6);
  emxFree_real_T(&anchorIdx);

  //  determine if a new anchor needs to be initialized, and request stereo
  //  measurements for it
  minNumValidFeatures = 10000;
  ixstart = 0;
  exitg2 = false;
  while ((!exitg2) && (ixstart <= (int)numAnchors - 1)) {
    for (i8 = 0; i8 < 16; i8++) {
      b_x[i8] = ((anchorFeatures->data[i8 + anchorFeatures->size[0] * ixstart] ==
                  1.0) && (updateVect[i8] == 1.0));
    }

    n = 0;
    for (nx = 0; nx < 16; nx++) {
      if (b_x[nx]) {
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
    emxInit_boolean_T(&r9, 2);
    exitg1 = false;
    while ((!exitg1) && (i < 16)) {
      loop_ub = anchorFeatures->size[1];
      i8 = r9->size[0] * r9->size[1];
      r9->size[0] = 1;
      r9->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)r9, i8, (int)sizeof(boolean_T));
      for (i8 = 0; i8 < loop_ub; i8++) {
        r9->data[r9->size[0] * i8] = (anchorFeatures->data[i +
          anchorFeatures->size[0] * i8] == 1.0);
      }

      b_guard1 = false;
      if ((!d_any(r9)) || (anchorFeatures->data[i + anchorFeatures->size[0] *
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

    emxFree_boolean_T(&r9);

    //      fprintf('Requesting %i new features\n', int8(newFeaturesRequested))
  }

  b_emxInit_real_T(&unusedU4, 1);

  // % aposteriori measurement prediction
  getMap(b_xt, anchorFeatures, m_vect, c_numTrackFeatures, numStatesxt, 7.0 +
         b_VIOParameters.num_points_per_anchor, map, depthUncertainties,
         unusedU4);
  i8 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(c_numTrackFeatures * 4.0);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i8, (int)sizeof(double));
  loop_ub = (int)(c_numTrackFeatures * 4.0);
  emxFree_real_T(&unusedU4);
  emxFree_real_T(&depthUncertainties);
  for (i8 = 0; i8 < loop_ub; i8++) {
    h_u_apo->data[i8] = rtNaN;
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
  emxInit_boolean_T(&r10, 2);
  for (i = 0; i < 16; i++) {
    loop_ub = anchorFeatures->size[1];
    i8 = r10->size[0] * r10->size[1];
    r10->size[0] = 1;
    r10->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)r10, i8, (int)sizeof(boolean_T));
    for (i8 = 0; i8 < loop_ub; i8++) {
      r10->data[r10->size[0] * i8] = (anchorFeatures->data[i +
        anchorFeatures->size[0] * i8] == 1.0);
    }

    if (d_any(r10)) {
      for (i8 = 0; i8 < 3; i8++) {
        b_fp[i8] = map->data[i8 + map->size[0] * i] - b_xt->data[i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        b_R_cw[i8] = 0.0;
        for (i9 = 0; i9 < 3; i9++) {
          b_R_cw[i8] += R_cw[i8 + 3 * i9] * b_fp[i9];
        }
      }

      predictMeasurement_stereo(b_R_cw, c_cameraParams_CameraParameters,
        d_cameraParams_CameraParameters, e_cameraParams_CameraParameters,
        f_cameraParams_CameraParameters, g_cameraParams_CameraParameters,
        h_cameraParams_CameraParameters, cameraParams_r_lr, cameraParams_R_rl,
        h_u_l, h_u_r);
      ixstart = i << 2;
      for (i8 = 0; i8 < 2; i8++) {
        h_u_apo->data[i8 + ixstart] = h_u_l[i8];
      }

      for (i8 = 0; i8 < 2; i8++) {
        h_u_apo->data[(i8 + ixstart) + 2] = h_u_r[i8];
      }
    }
  }

  emxFree_boolean_T(&r10);
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
