//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Aug-2015 16:40:53
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

// Function Definitions

//
// % Iterative Camera Pose optimization (EKF)
// Arguments    : emxArray_real_T *P_apr
//                emxArray_real_T *b_xt
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                const double cameraparams_R_rl[9]
//                double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                const double imNoise[2]
//                const double IMU_measurements[13]
//                double numPointsPerAnchor
//                double numAnchors
//                double height_offset_pressure
//                emxArray_real_T *h_u_apo
//                emxArray_real_T *map
// Return Type  : void
//
void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const double
                cameraparams_r_lr[3], const double cameraparams_R_lr[9], const
                double cameraparams_R_rl[9], double updateVect[16], const double
                z_all_l[32], const double z_all_r[32], const double imNoise[2],
                const double IMU_measurements[13], double numPointsPerAnchor,
                double numAnchors, double height_offset_pressure,
                emxArray_real_T *h_u_apo, emxArray_real_T *map)
{
  double c_numTrackFeatures;
  int i15;
  int loop_ub;
  int i;
  emxArray_real_T *anchorIdx;
  emxArray_int32_T *r8;
  emxArray_boolean_T *x;
  emxArray_int32_T *ii;
  emxArray_real_T *b_anchorIdx;
  int nx;
  int idx;
  int nm1d2;
  boolean_T exitg7;
  boolean_T guard7 = false;
  int n;
  int k;
  int i16;
  int unnamed_idx_0;
  emxArray_boolean_T *r9;
  boolean_T b_x[16];
  int ii_data[16];
  boolean_T exitg6;
  boolean_T guard6 = false;
  signed char indMeas_data[16];
  int i17;
  int validFeatures_size[1];
  double validFeatures_data[16];
  emxArray_real_T *c_anchorIdx;
  boolean_T exitg5;
  boolean_T guard5 = false;
  emxArray_real_T *sigmaInits;
  double initializeNewAnchor;
  int minNumValidFeatures;
  boolean_T exitg4;
  boolean_T guard4 = false;
  double R_cw[9];
  double r_wc[3];
  double anew;
  double c_xt[7];
  emxArray_boolean_T *r10;
  boolean_T exitg3;
  boolean_T guard3 = false;
  int ii_size_idx_0;
  unsigned int featureAnchorIdx;
  int unusedFeatureIdx;
  double z_curr_l[2];
  double z_curr_r[2];
  double m[3];
  double fp[3];
  boolean_T bv0[3];
  double b_R_cw[3];
  double b_fp[3];
  double h_u_r[2];
  double h_u_l[2];
  double b_h_u_l[2];
  boolean_T guard1 = false;
  signed char i18;
  double apnd;
  double ndbl;
  double cdiff;
  emxArray_real_T *J;
  static const signed char iv12[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  emxArray_real_T *y;
  int b_loop_ub;
  int i19;
  int b_m;
  int cr;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  emxArray_real_T *b;
  boolean_T exitg2;
  boolean_T guard2 = false;
  double newFeaturesRequested;
  emxArray_boolean_T *r11;
  boolean_T exitg1;
  boolean_T b_guard1 = false;
  emxArray_real_T *unusedU1;
  emxArray_boolean_T *r12;
  c_numTrackFeatures = numAnchors * numPointsPerAnchor;
  if (!m_vect_not_empty) {
    i15 = m_vect->size[0] * m_vect->size[1];
    m_vect->size[0] = 3;
    m_vect->size[1] = (int)(numPointsPerAnchor * numAnchors);
    emxEnsureCapacity((emxArray__common *)m_vect, i15, (int)sizeof(double));
    loop_ub = 3 * (int)(numPointsPerAnchor * numAnchors);
    for (i15 = 0; i15 < loop_ub; i15++) {
      m_vect->data[i15] = rtNaN;
    }

    m_vect_not_empty = !(m_vect->size[1] == 0);

    //  a matrix containing the m vectors for each feature
    i15 = anchorFeatures->size[0] * anchorFeatures->size[1];
    anchorFeatures->size[0] = 16;
    anchorFeatures->size[1] = (int)numAnchors;
    emxEnsureCapacity((emxArray__common *)anchorFeatures, i15, (int)sizeof
                      (double));
    loop_ub = (int)numAnchors << 4;
    for (i15 = 0; i15 < loop_ub; i15++) {
      anchorFeatures->data[i15] = 0.0;
    }

    //  describes which feature belongs to which anchor
  }

  //  debug check
  // % check for lost features
  i = 0;
  emxInit_real_T(&anchorIdx, 2);
  emxInit_int32_T(&r8, 1);
  emxInit_boolean_T(&x, 2);
  b_emxInit_int32_T(&ii, 2);
  b_emxInit_real_T(&b_anchorIdx, 1);
  while (i <= (int)c_numTrackFeatures - 1) {
    loop_ub = anchorFeatures->size[1];
    i15 = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)x, i15, (int)sizeof(boolean_T));
    for (i15 = 0; i15 < loop_ub; i15++) {
      x->data[x->size[0] * i15] = (anchorFeatures->data[i + anchorFeatures->
        size[0] * i15] == 1.0);
    }

    nx = x->size[1];
    idx = 0;
    i15 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = x->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i15, (int)sizeof(int));
    nm1d2 = 1;
    exitg7 = false;
    while ((!exitg7) && (nm1d2 <= nx)) {
      guard7 = false;
      if (x->data[nm1d2 - 1]) {
        idx++;
        ii->data[idx - 1] = nm1d2;
        if (idx >= nx) {
          exitg7 = true;
        } else {
          guard7 = true;
        }
      } else {
        guard7 = true;
      }

      if (guard7) {
        nm1d2++;
      }
    }

    if (x->size[1] == 1) {
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
      for (k = 0; k < i15; k++) {
        i16 = r8->size[0];
        r8->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r8, i16, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i16 = 0; i16 < loop_ub; i16++) {
          r8->data[i16] = (int)anchorIdx->data[anchorIdx->size[0] * i16];
        }

        if (anchorFeatures->data[k % (i + 1) + anchorFeatures->size[0] *
            (r8->data[div_nzp_s32_floor(k, i + 1)] - 1)] != 0.0) {
          n++;
        }
      }

      //  remove covariance of this feature with rest of state
      i15 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r8->data[i15] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0] *
          i15] - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (double)n);
      }

      loop_ub = P_apr->size[1];
      unnamed_idx_0 = r8->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < unnamed_idx_0; i16++) {
          P_apr->data[(r8->data[i16] + P_apr->size[0] * i15) - 1] = 0.0;
        }
      }

      i15 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r8->data[i15] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0] *
          i15] - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (double)n);
      }

      loop_ub = P_apr->size[0];
      nm1d2 = r8->size[0];
      for (i15 = 0; i15 < nm1d2; i15++) {
        for (i16 = 0; i16 < loop_ub; i16++) {
          P_apr->data[i16 + P_apr->size[0] * (r8->data[i15] - 1)] = 0.0;
        }
      }

      i15 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r8->data[i15] = (int)anchorIdx->data[anchorIdx->size[0] * i15];
      }

      nm1d2 = r8->size[0];
      for (i15 = 0; i15 < nm1d2; i15++) {
        anchorFeatures->data[i + anchorFeatures->size[0] * (r8->data[i15] - 1)] =
          -1.0;
      }

      //  mark feature as lost
    }

    i++;
  }

  emxFree_real_T(&b_anchorIdx);
  emxInit_boolean_T(&r9, 2);

  // % do the update
  i15 = r9->size[0] * r9->size[1];
  r9->size[0] = 16;
  r9->size[1] = anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)r9, i15, (int)sizeof(boolean_T));
  loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
  for (i15 = 0; i15 < loop_ub; i15++) {
    r9->data[i15] = (anchorFeatures->data[i15] == 1.0);
  }

  b_any(r9, b_x);
  idx = 0;
  nm1d2 = 1;
  emxFree_boolean_T(&r9);
  exitg6 = false;
  while ((!exitg6) && (nm1d2 < 17)) {
    guard6 = false;
    if (b_x[nm1d2 - 1]) {
      idx++;
      ii_data[idx - 1] = nm1d2;
      if (idx >= 16) {
        exitg6 = true;
      } else {
        guard6 = true;
      }
    } else {
      guard6 = true;
    }

    if (guard6) {
      nm1d2++;
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
    //   [ xt, P_apo, validFeatures ] = Mahalanobis_EKF(xt, P_apr, z_all_l, numStatesxt, numStates, numPointsPerAnchor, cameraparams, anchorFeatures, m_vect, imNoise,IMU_measurements,height_offset_pressure); 
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, numStatesxt, numStates,
                       numPointsPerAnchor, anchorFeatures, m_vect, imNoise,
                       IMU_measurements, height_offset_pressure,
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
      i15 = x->size[0] * x->size[1];
      x->size[0] = 1;
      x->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)x, i15, (int)sizeof(boolean_T));
      for (i15 = 0; i15 < loop_ub; i15++) {
        x->data[x->size[0] * i15] = (anchorFeatures->data[i +
          anchorFeatures->size[0] * i15] == 1.0);
      }

      nx = x->size[1];
      idx = 0;
      i15 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = x->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i15, (int)sizeof(int));
      nm1d2 = 1;
      exitg5 = false;
      while ((!exitg5) && (nm1d2 <= nx)) {
        guard5 = false;
        if (x->data[nm1d2 - 1]) {
          idx++;
          ii->data[idx - 1] = nm1d2;
          if (idx >= nx) {
            exitg5 = true;
          } else {
            guard5 = true;
          }
        } else {
          guard5 = true;
        }

        if (guard5) {
          nm1d2++;
        }
      }

      if (x->size[1] == 1) {
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
        for (k = 0; k < i15; k++) {
          i16 = r8->size[0];
          r8->size[0] = anchorIdx->size[1];
          emxEnsureCapacity((emxArray__common *)r8, i16, (int)sizeof(int));
          loop_ub = anchorIdx->size[1];
          for (i16 = 0; i16 < loop_ub; i16++) {
            r8->data[i16] = (int)anchorIdx->data[anchorIdx->size[0] * i16];
          }

          if (anchorFeatures->data[k % (i + 1) + anchorFeatures->size[0] *
              (r8->data[div_nzp_s32_floor(k, i + 1)] - 1)] != 0.0) {
            n++;
          }
        }

        //  remove covariance of this feature with rest of state
        i15 = r8->size[0];
        r8->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i15 = 0; i15 < loop_ub; i15++) {
          r8->data[i15] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0]
            * i15] - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (double)n);
        }

        loop_ub = P_apr->size[1];
        unnamed_idx_0 = r8->size[0];
        for (i15 = 0; i15 < loop_ub; i15++) {
          for (i16 = 0; i16 < unnamed_idx_0; i16++) {
            P_apr->data[(r8->data[i16] + P_apr->size[0] * i15) - 1] = 0.0;
          }
        }

        i15 = r8->size[0];
        r8->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i15 = 0; i15 < loop_ub; i15++) {
          r8->data[i15] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0]
            * i15] - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (double)n);
        }

        loop_ub = P_apr->size[0];
        nm1d2 = r8->size[0];
        for (i15 = 0; i15 < nm1d2; i15++) {
          for (i16 = 0; i16 < loop_ub; i16++) {
            P_apr->data[i16 + P_apr->size[0] * (r8->data[i15] - 1)] = 0.0;
          }
        }

        i15 = r8->size[0];
        r8->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i15 = 0; i15 < loop_ub; i15++) {
          r8->data[i15] = (int)anchorIdx->data[anchorIdx->size[0] * i15];
        }

        nm1d2 = r8->size[0];
        for (i15 = 0; i15 < nm1d2; i15++) {
          anchorFeatures->data[i + anchorFeatures->size[0] * (r8->data[i15] - 1)]
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

  b_emxInit_real_T(&sigmaInits, 1);
  if (any(b_x)) {
    //  if there are any features with stereo measurements
    initializeNewAnchor = 0.0;
    minNumValidFeatures = 10000;
    nm1d2 = 0;
    exitg4 = false;
    while ((!exitg4) && (nm1d2 <= (int)numAnchors - 1)) {
      for (i15 = 0; i15 < 16; i15++) {
        b_x[i15] = ((anchorFeatures->data[i15 + anchorFeatures->size[0] * nm1d2]
                     == 1.0) && (updateVect[i15] == 1.0));
      }

      n = 0;
      for (k = 0; k < 16; k++) {
        if (b_x[k]) {
          n++;
        }
      }

      guard4 = false;
      if ((n < minFeatureThreshold) && (n < minNumValidFeatures)) {
        minNumValidFeatures = n;
        initializeNewAnchor = 1.0 + (double)nm1d2;
        if (!(n != 0)) {
          exitg4 = true;
        } else {
          guard4 = true;
        }
      } else {
        guard4 = true;
      }

      if (guard4) {
        nm1d2++;
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

      for (i15 = 0; i15 < 16; i15++) {
        b_x[i15] = (anchorFeatures->data[i15 + anchorFeatures->size[0] * ((int)
          initializeNewAnchor - 1)] == 1.0);
      }

      n = 0;
      for (i = 0; i < 16; i++) {
        if (b_x[i]) {
          n++;
        }
      }

      nm1d2 = 0;
      for (i = 0; i < 16; i++) {
        if (b_x[i]) {
          ii_data[nm1d2] = i + 1;
          nm1d2++;
        }
      }

      for (i15 = 0; i15 < n; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          m_vect->data[i16 + m_vect->size[0] * (ii_data[i15] - 1)] = rtNaN;
        }
      }

      for (i15 = 0; i15 < 16; i15++) {
        anchorFeatures->data[i15 + anchorFeatures->size[0] * ((int)
          initializeNewAnchor - 1)] = 0.0;
      }

      anew = numStatesxt + (initializeNewAnchor - 1.0) * (7.0 +
        numPointsPerAnchor);
      for (i15 = 0; i15 < 7; i15++) {
        c_xt[i15] = b_xt->data[i15];
      }

      for (i15 = 0; i15 < 7; i15++) {
        b_xt->data[(int)(anew + (1.0 + (double)i15)) - 1] = c_xt[i15];
      }

      i15 = sigmaInits->size[0];
      sigmaInits->size[0] = (int)numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)sigmaInits, i15, (int)sizeof(double));
      loop_ub = (int)numPointsPerAnchor;
      for (i15 = 0; i15 < loop_ub; i15++) {
        sigmaInits->data[i15] = 0.0;
      }

      emxInit_boolean_T(&r10, 2);
      i15 = r10->size[0] * r10->size[1];
      r10->size[0] = 16;
      r10->size[1] = anchorFeatures->size[1];
      emxEnsureCapacity((emxArray__common *)r10, i15, (int)sizeof(boolean_T));
      loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r10->data[i15] = (anchorFeatures->data[i15] == 1.0);
      }

      b_any(r10, b_x);
      emxFree_boolean_T(&r10);
      for (i15 = 0; i15 < 16; i15++) {
        b_x[i15] = !b_x[i15];
      }

      idx = 0;
      nm1d2 = 1;
      exitg3 = false;
      while ((!exitg3) && (nm1d2 < 17)) {
        guard3 = false;
        if (b_x[nm1d2 - 1] && (updateVect[nm1d2 - 1] == 2.0)) {
          idx++;
          ii_data[idx - 1] = nm1d2;
          if (idx >= 16) {
            exitg3 = true;
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }

        if (guard3) {
          nm1d2++;
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
        nm1d2 = (indMeas_data[unusedFeatureIdx] - 1) * 2;
        k = (indMeas_data[unusedFeatureIdx] - 1) * 2;
        for (i15 = 0; i15 < 2; i15++) {
          z_curr_l[i15] = z_all_l[nm1d2 + i15];
          z_curr_r[i15] = z_all_r[k + i15];
        }

        initializePoint(b_xt, cameraparams_r_lr, cameraparams_R_lr, z_curr_l,
                        z_curr_r, fp, m);
        for (i = 0; i < 3; i++) {
          bv0[i] = rtIsNaN(fp[i]);
        }

        if (c_any(bv0)) {
          updateVect[indMeas_data[unusedFeatureIdx] - 1] = 0.0;
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

          predictMeasurement_stereo(b_R_cw, cameraparams_r_lr, cameraparams_R_rl,
            h_u_l, h_u_r);
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

              b_xt->data[(int)(((numStatesxt + (initializeNewAnchor - 1.0) *
                                 (7.0 + numPointsPerAnchor)) + 7.0) + (double)
                               featureAnchorIdx) - 1] = 1.0 / norm(b_fp);
              sigmaInits->data[(int)featureAnchorIdx - 1] = sigma_Init;
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

      anew = rt_roundd_snf((double)featureAnchorIdx - 1.0);
      if (anew < 2.147483648E+9) {
        i15 = (int)anew;
      } else {
        i15 = MAX_int32_T;
      }

      f_fprintf(i15, (int)initializeNewAnchor);
      if (rtIsInf(6.0 + numPointsPerAnchor)) {
        n = 0;
        anew = rtNaN;
        apnd = 6.0 + numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
        if (fabs(cdiff) < 4.4408920985006262E-16 * (6.0 + numPointsPerAnchor)) {
          ndbl++;
          apnd = 6.0 + numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        n = (int)ndbl - 1;
      }

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          nm1d2 = (n + (n < 0)) >> 1;
          for (k = 1; k < nm1d2; k++) {
            anchorIdx->data[k] = anew + (double)k;
            anchorIdx->data[n - k] = apnd - (double)k;
          }

          if (nm1d2 << 1 == n) {
            anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[nm1d2] = anew + (double)nm1d2;
            anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      anew = numStates + (initializeNewAnchor - 1.0) * (6.0 + numPointsPerAnchor);
      i15 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r8->data[i15] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i15]);
      }

      loop_ub = P_apr->size[1];
      unnamed_idx_0 = r8->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < unnamed_idx_0; i16++) {
          P_apr->data[(r8->data[i16] + P_apr->size[0] * i15) - 1] = 0.0;
        }
      }

      if (rtIsInf(6.0 + numPointsPerAnchor)) {
        n = 0;
        anew = rtNaN;
        apnd = 6.0 + numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
        if (fabs(cdiff) < 4.4408920985006262E-16 * (6.0 + numPointsPerAnchor)) {
          ndbl++;
          apnd = 6.0 + numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        n = (int)ndbl - 1;
      }

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          nm1d2 = (n + (n < 0)) >> 1;
          for (k = 1; k < nm1d2; k++) {
            anchorIdx->data[k] = anew + (double)k;
            anchorIdx->data[n - k] = apnd - (double)k;
          }

          if (nm1d2 << 1 == n) {
            anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[nm1d2] = anew + (double)nm1d2;
            anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      anew = numStates + (initializeNewAnchor - 1.0) * (6.0 + numPointsPerAnchor);
      i15 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r8->data[i15] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i15]);
      }

      loop_ub = P_apr->size[0];
      nm1d2 = r8->size[0];
      for (i15 = 0; i15 < nm1d2; i15++) {
        for (i16 = 0; i16 < loop_ub; i16++) {
          P_apr->data[i16 + P_apr->size[0] * (r8->data[i15] - 1)] = 0.0;
        }
      }

      for (i15 = 0; i15 < 2; i15++) {
        b_h_u_l[i15] = P_apr->size[i15];
      }

      emxInit_real_T(&J, 2);
      c_eye(b_h_u_l, J);
      if (rtIsInf(6.0 + numPointsPerAnchor)) {
        n = 0;
        anew = rtNaN;
        apnd = 6.0 + numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
        if (fabs(cdiff) < 4.4408920985006262E-16 * (6.0 + numPointsPerAnchor)) {
          ndbl++;
          apnd = 6.0 + numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        n = (int)ndbl - 1;
      }

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          nm1d2 = (n + (n < 0)) >> 1;
          for (k = 1; k < nm1d2; k++) {
            anchorIdx->data[k] = anew + (double)k;
            anchorIdx->data[n - k] = apnd - (double)k;
          }

          if (nm1d2 << 1 == n) {
            anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[nm1d2] = anew + (double)nm1d2;
            anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      anew = numStates + (initializeNewAnchor - 1.0) * (6.0 + numPointsPerAnchor);
      i15 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r8->data[i15] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i15]);
      }

      loop_ub = J->size[1];
      unnamed_idx_0 = r8->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < unnamed_idx_0; i16++) {
          J->data[(r8->data[i16] + J->size[0] * i15) - 1] = 0.0;
        }
      }

      if (rtIsInf(6.0 + numPointsPerAnchor)) {
        n = 0;
        anew = rtNaN;
        apnd = 6.0 + numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
        if (fabs(cdiff) < 4.4408920985006262E-16 * (6.0 + numPointsPerAnchor)) {
          ndbl++;
          apnd = 6.0 + numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        n = (int)ndbl - 1;
      }

      i15 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          nm1d2 = (n + (n < 0)) >> 1;
          for (k = 1; k < nm1d2; k++) {
            anchorIdx->data[k] = anew + (double)k;
            anchorIdx->data[n - k] = apnd - (double)k;
          }

          if (nm1d2 << 1 == n) {
            anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[nm1d2] = anew + (double)nm1d2;
            anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      anew = numStates + (initializeNewAnchor - 1.0) * (6.0 + numPointsPerAnchor);
      i15 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        r8->data[i15] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i15])
          - 1;
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r8->data[i16] + J->size[0] * i15] = iv12[i16 + 3 * i15];
        }
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r8->data[i16] + J->size[0] * (i15 + 3)] = 0.0;
        }
      }

      loop_ub = (int)(numStates - 6.0);
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r8->data[i16] + J->size[0] * (i15 + 6)] = 0.0;
        }
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r8->data[i16 + 3] + J->size[0] * i15] = 0.0;
        }
      }

      for (i15 = 0; i15 < 3; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r8->data[i16 + 3] + J->size[0] * (i15 + 3)] = iv12[i16 + 3 *
            i15];
        }
      }

      loop_ub = (int)(numStates - 6.0);
      for (i15 = 0; i15 < loop_ub; i15++) {
        for (i16 = 0; i16 < 3; i16++) {
          J->data[r8->data[i16 + 3] + J->size[0] * (i15 + 6)] = 0.0;
        }
      }

      loop_ub = (int)numStates;
      for (i15 = 0; i15 < loop_ub; i15++) {
        nm1d2 = (int)numPointsPerAnchor;
        for (i16 = 0; i16 < nm1d2; i16++) {
          J->data[r8->data[i16 + 6] + J->size[0] * i15] = 0.0;
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
          nm1d2 = P_apr->size[1];
          for (i16 = 0; i16 < nm1d2; i16++) {
            y->data[i15 + y->size[0] * i16] = 0.0;
            b_loop_ub = J->size[1];
            for (i19 = 0; i19 < b_loop_ub; i19++) {
              y->data[i15 + y->size[0] * i16] += J->data[i15 + J->size[0] * i19]
                * P_apr->data[i19 + P_apr->size[0] * i16];
            }
          }
        }
      } else {
        k = J->size[1];
        unnamed_idx_0 = J->size[0];
        nm1d2 = P_apr->size[1];
        b_m = J->size[0];
        i15 = y->size[0] * y->size[1];
        y->size[0] = unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)y, i15, (int)sizeof(double));
        i15 = y->size[0] * y->size[1];
        y->size[1] = nm1d2;
        emxEnsureCapacity((emxArray__common *)y, i15, (int)sizeof(double));
        loop_ub = unnamed_idx_0 * nm1d2;
        for (i15 = 0; i15 < loop_ub; i15++) {
          y->data[i15] = 0.0;
        }

        if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
        } else {
          nm1d2 = J->size[0] * (P_apr->size[1] - 1);
          cr = 0;
          while ((b_m > 0) && (cr <= nm1d2)) {
            i15 = cr + b_m;
            for (ic = cr; ic + 1 <= i15; ic++) {
              y->data[ic] = 0.0;
            }

            cr += b_m;
          }

          br = 0;
          cr = 0;
          while ((b_m > 0) && (cr <= nm1d2)) {
            ar = 0;
            i15 = br + k;
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

            br += k;
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
        nm1d2 = J->size[1];
        for (i16 = 0; i16 < nm1d2; i16++) {
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
          nm1d2 = b->size[1];
          for (i16 = 0; i16 < nm1d2; i16++) {
            P_apr->data[i15 + P_apr->size[0] * i16] = 0.0;
            b_loop_ub = y->size[1];
            for (i19 = 0; i19 < b_loop_ub; i19++) {
              P_apr->data[i15 + P_apr->size[0] * i16] += y->data[i15 + y->size[0]
                * i19] * b->data[i19 + b->size[0] * i16];
            }
          }
        }
      } else {
        k = y->size[1];
        unnamed_idx_0 = y->size[0];
        nm1d2 = b->size[1];
        b_m = y->size[0];
        i15 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = unnamed_idx_0;
        P_apr->size[1] = nm1d2;
        emxEnsureCapacity((emxArray__common *)P_apr, i15, (int)sizeof(double));
        for (i15 = 0; i15 < nm1d2; i15++) {
          for (i16 = 0; i16 < unnamed_idx_0; i16++) {
            P_apr->data[i16 + P_apr->size[0] * i15] = 0.0;
          }
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          nm1d2 = y->size[0] * (b->size[1] - 1);
          cr = 0;
          while ((b_m > 0) && (cr <= nm1d2)) {
            i15 = cr + b_m;
            for (ic = cr; ic + 1 <= i15; ic++) {
              P_apr->data[ic] = 0.0;
            }

            cr += b_m;
          }

          br = 0;
          cr = 0;
          while ((b_m > 0) && (cr <= nm1d2)) {
            ar = 0;
            i15 = br + k;
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

            br += k;
            cr += b_m;
          }
        }
      }

      emxFree_real_T(&b);
      emxFree_real_T(&y);
      for (nm1d2 = 0; nm1d2 < (int)numPointsPerAnchor; nm1d2++) {
        P_apr->data[((int)(((numStates + (initializeNewAnchor - 1.0) * (6.0 +
          numPointsPerAnchor)) + 6.0) + (1.0 + (double)nm1d2)) + P_apr->size[0] *
                     ((int)(((numStates + (initializeNewAnchor - 1.0) * (6.0 +
          numPointsPerAnchor)) + 6.0) + (1.0 + (double)nm1d2)) - 1)) - 1] =
          sigmaInits->data[nm1d2];
      }
    } else {
      h_fprintf();
    }
  }

  emxFree_int32_T(&r8);
  emxFree_real_T(&anchorIdx);

  //  determine if a new anchor needs to be initialized, and request stereo
  //  measurements for it
  minNumValidFeatures = 10000;
  nm1d2 = 0;
  exitg2 = false;
  while ((!exitg2) && (nm1d2 <= (int)numAnchors - 1)) {
    for (i15 = 0; i15 < 16; i15++) {
      b_x[i15] = ((anchorFeatures->data[i15 + anchorFeatures->size[0] * nm1d2] ==
                   1.0) && (updateVect[i15] == 1.0));
    }

    n = 0;
    for (k = 0; k < 16; k++) {
      if (b_x[k]) {
        n++;
      }
    }

    guard2 = false;
    if ((n < minFeatureThreshold) && (n < minNumValidFeatures)) {
      minNumValidFeatures = n;
      initializeNewAnchor = 1.0 + (double)nm1d2;
      if (!(n != 0)) {
        exitg2 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      nm1d2++;
    }
  }

  if (minNumValidFeatures < minFeatureThreshold) {
    newFeaturesRequested = 0.0;
    i = 0;
    emxInit_boolean_T(&r11, 2);
    exitg1 = false;
    while ((!exitg1) && (i < 16)) {
      loop_ub = anchorFeatures->size[1];
      i15 = r11->size[0] * r11->size[1];
      r11->size[0] = 1;
      r11->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)r11, i15, (int)sizeof(boolean_T));
      for (i15 = 0; i15 < loop_ub; i15++) {
        r11->data[r11->size[0] * i15] = (anchorFeatures->data[i +
          anchorFeatures->size[0] * i15] == 1.0);
      }

      b_guard1 = false;
      if ((!d_any(r11)) || (anchorFeatures->data[i + anchorFeatures->size[0] *
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

    emxFree_boolean_T(&r11);

    //      fprintf('Requesting %i new features\n', int8(newFeaturesRequested))
  }

  b_emxInit_real_T(&unusedU1, 1);

  // % aposteriori measurement prediction
  getMap(b_xt, anchorFeatures, m_vect, c_numTrackFeatures, numStatesxt, 7.0 +
         numPointsPerAnchor, map, sigmaInits, unusedU1);
  i15 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(c_numTrackFeatures * 4.0);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i15, (int)sizeof(double));
  loop_ub = (int)(c_numTrackFeatures * 4.0);
  emxFree_real_T(&unusedU1);
  emxFree_real_T(&sigmaInits);
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
  emxInit_boolean_T(&r12, 2);
  for (i = 0; i < 16; i++) {
    loop_ub = anchorFeatures->size[1];
    i15 = r12->size[0] * r12->size[1];
    r12->size[0] = 1;
    r12->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)r12, i15, (int)sizeof(boolean_T));
    for (i15 = 0; i15 < loop_ub; i15++) {
      r12->data[r12->size[0] * i15] = (anchorFeatures->data[i +
        anchorFeatures->size[0] * i15] == 1.0);
    }

    if (d_any(r12)) {
      for (i15 = 0; i15 < 3; i15++) {
        b_fp[i15] = map->data[i15 + map->size[0] * i] - b_xt->data[i15];
      }

      for (i15 = 0; i15 < 3; i15++) {
        b_R_cw[i15] = 0.0;
        for (i16 = 0; i16 < 3; i16++) {
          b_R_cw[i15] += R_cw[i15 + 3 * i16] * b_fp[i16];
        }
      }

      predictMeasurement_stereo(b_R_cw, cameraparams_r_lr, cameraparams_R_rl,
        h_u_l, h_u_r);
      nm1d2 = i << 2;
      for (i15 = 0; i15 < 2; i15++) {
        h_u_apo->data[i15 + nm1d2] = h_u_l[i15];
      }

      for (i15 = 0; i15 < 2; i15++) {
        h_u_apo->data[(i15 + nm1d2) + 2] = h_u_r[i15];
      }
    }
  }

  emxFree_boolean_T(&r12);
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
