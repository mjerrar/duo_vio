//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
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
#include "Mahalanobis_EKF.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : struct_T *b_SLAM_data
//                double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                const double IMU_measurements[23]
//                emxArray_real_T *h_u_apo
//                emxArray_real_T *map
// Return Type  : void
//
void SLAM_updIT(struct_T *b_SLAM_data, double updateVect[16], const double
                z_all_l[32], const double z_all_r[32], const double
                IMU_measurements[23], emxArray_real_T *h_u_apo, emxArray_real_T *
                map)
{
  emxArray_real_T *b_xt;
  double c_numStates;
  double b_numPointsPerAnchor;
  int i17;
  int ar;
  emxArray_real_T *P_apr;
  emxArray_real_T *b_anchorFeatures;
  emxArray_real_T *b_m_vect;
  double b_numStatesPerAnchor;
  double c_numTrackFeatures;
  int i;
  emxArray_real_T *anchorIdx;
  emxArray_int32_T *ii;
  emxArray_boolean_T *x;
  emxArray_int32_T *b_ii;
  emxArray_real_T *b_anchorIdx;
  int nx;
  int idx;
  int nm1d2;
  boolean_T exitg4;
  boolean_T guard4 = false;
  int k;
  int i18;
  emxArray_boolean_T *c_anchorFeatures;
  emxArray_boolean_T *b_x;
  boolean_T exitg3;
  boolean_T guard3 = false;
  emxArray_real_T *indMeas;
  emxArray_real_T *validFeatures;
  boolean_T b_updateVect[16];
  double initializeNewAnchor;
  int minNumValidFeatures;
  boolean_T exitg2;
  boolean_T guard2 = false;
  double R_cw[9];
  double r_wc[3];
  double anew;
  double c_xt[7];
  emxArray_boolean_T *d_anchorFeatures;
  signed char ii_data[16];
  boolean_T exitg1;
  boolean_T guard1 = false;
  signed char unusedFeatures_data[16];
  unsigned int featureAnchorIdx;
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
  boolean_T b_guard1 = false;
  double rhoInit;
  signed char i19;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  emxArray_real_T *J;
  static const signed char iv14[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  emxArray_real_T *y;
  int b_m;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *b;
  emxArray_boolean_T *e_anchorFeatures;
  emxInit_real_T(&b_xt, 1);
  c_numStates = b_SLAM_data->numStates;
  b_numPointsPerAnchor = b_SLAM_data->numPointsPerAnchor;
  i17 = b_xt->size[0];
  b_xt->size[0] = b_SLAM_data->xt->size[0];
  emxEnsureCapacity((emxArray__common *)b_xt, i17, (int)sizeof(double));
  ar = b_SLAM_data->xt->size[0];
  for (i17 = 0; i17 < ar; i17++) {
    b_xt->data[i17] = b_SLAM_data->xt->data[i17];
  }

  b_emxInit_real_T(&P_apr, 2);
  i17 = P_apr->size[0] * P_apr->size[1];
  P_apr->size[0] = b_SLAM_data->P->size[0];
  P_apr->size[1] = b_SLAM_data->P->size[1];
  emxEnsureCapacity((emxArray__common *)P_apr, i17, (int)sizeof(double));
  ar = b_SLAM_data->P->size[0] * b_SLAM_data->P->size[1];
  for (i17 = 0; i17 < ar; i17++) {
    P_apr->data[i17] = b_SLAM_data->P->data[i17];
  }

  b_emxInit_real_T(&b_anchorFeatures, 2);
  i17 = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  b_anchorFeatures->size[0] = b_SLAM_data->anchorFeatures->size[0];
  b_anchorFeatures->size[1] = b_SLAM_data->anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)b_anchorFeatures, i17, (int)sizeof
                    (double));
  ar = b_SLAM_data->anchorFeatures->size[0] * b_SLAM_data->anchorFeatures->size
    [1];
  for (i17 = 0; i17 < ar; i17++) {
    b_anchorFeatures->data[i17] = b_SLAM_data->anchorFeatures->data[i17];
  }

  b_emxInit_real_T(&b_m_vect, 2);
  i17 = b_m_vect->size[0] * b_m_vect->size[1];
  b_m_vect->size[0] = 3;
  b_m_vect->size[1] = b_SLAM_data->m_vect->size[1];
  emxEnsureCapacity((emxArray__common *)b_m_vect, i17, (int)sizeof(double));
  ar = b_SLAM_data->m_vect->size[0] * b_SLAM_data->m_vect->size[1];
  for (i17 = 0; i17 < ar; i17++) {
    b_m_vect->data[i17] = b_SLAM_data->m_vect->data[i17];
  }

  // % Iterative Camera Pose optimization (EKF)
  b_numStatesPerAnchor = 6.0 + b_SLAM_data->numPointsPerAnchor;
  c_numTrackFeatures = b_SLAM_data->numAnchors * b_SLAM_data->numPointsPerAnchor;

  //  debug check
  // % check for lost features
  i = 0;
  b_emxInit_real_T(&anchorIdx, 2);
  emxInit_int32_T(&ii, 1);
  emxInit_boolean_T(&x, 2);
  b_emxInit_int32_T(&b_ii, 2);
  emxInit_real_T(&b_anchorIdx, 1);
  while (i <= (int)c_numTrackFeatures - 1) {
    ar = b_anchorFeatures->size[1];
    i17 = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = ar;
    emxEnsureCapacity((emxArray__common *)x, i17, (int)sizeof(boolean_T));
    for (i17 = 0; i17 < ar; i17++) {
      x->data[x->size[0] * i17] = (b_anchorFeatures->data[i +
        b_anchorFeatures->size[0] * i17] == 1.0);
    }

    nx = x->size[1];
    idx = 0;
    i17 = b_ii->size[0] * b_ii->size[1];
    b_ii->size[0] = 1;
    b_ii->size[1] = x->size[1];
    emxEnsureCapacity((emxArray__common *)b_ii, i17, (int)sizeof(int));
    nm1d2 = 1;
    exitg4 = false;
    while ((!exitg4) && (nm1d2 <= nx)) {
      guard4 = false;
      if (x->data[nm1d2 - 1]) {
        idx++;
        b_ii->data[idx - 1] = nm1d2;
        if (idx >= nx) {
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

    if (x->size[1] == 1) {
      if (idx == 0) {
        i17 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = 0;
        emxEnsureCapacity((emxArray__common *)b_ii, i17, (int)sizeof(int));
      }
    } else {
      i17 = b_ii->size[0] * b_ii->size[1];
      if (1 > idx) {
        b_ii->size[1] = 0;
      } else {
        b_ii->size[1] = idx;
      }

      emxEnsureCapacity((emxArray__common *)b_ii, i17, (int)sizeof(int));
    }

    i17 = anchorIdx->size[0] * anchorIdx->size[1];
    anchorIdx->size[0] = 1;
    anchorIdx->size[1] = b_ii->size[1];
    emxEnsureCapacity((emxArray__common *)anchorIdx, i17, (int)sizeof(double));
    ar = b_ii->size[0] * b_ii->size[1];
    for (i17 = 0; i17 < ar; i17++) {
      anchorIdx->data[i17] = b_ii->data[i17];
    }

    if ((!(anchorIdx->size[1] == 0)) && (updateVect[i] != 1.0)) {
      nx = 0;
      i17 = b_anchorIdx->size[0];
      b_anchorIdx->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)b_anchorIdx, i17, (int)sizeof(double));
      ar = anchorIdx->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        b_anchorIdx->data[i17] = anchorIdx->data[anchorIdx->size[0] * i17];
      }

      i17 = (i + 1) * b_anchorIdx->size[0];
      for (k = 0; k < i17; k++) {
        i18 = ii->size[0];
        ii->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)ii, i18, (int)sizeof(int));
        ar = anchorIdx->size[1];
        for (i18 = 0; i18 < ar; i18++) {
          ii->data[i18] = (int)anchorIdx->data[anchorIdx->size[0] * i18];
        }

        if (b_anchorFeatures->data[k % (i + 1) + b_anchorFeatures->size[0] *
            (ii->data[div_nzp_s32_floor(k, i + 1)] - 1)] != 0.0) {
          nx++;
        }
      }

      //  remove covariance of this feature with rest of state
      i17 = ii->size[0];
      ii->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
      ar = anchorIdx->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        ii->data[i17] = (int)(((c_numStates + (anchorIdx->data[anchorIdx->size[0]
          * i17] - 1.0) * b_numStatesPerAnchor) + 6.0) + (double)nx);
      }

      ar = P_apr->size[1];
      nm1d2 = ii->size[0];
      for (i17 = 0; i17 < ar; i17++) {
        for (i18 = 0; i18 < nm1d2; i18++) {
          P_apr->data[(ii->data[i18] + P_apr->size[0] * i17) - 1] = 0.0;
        }
      }

      i17 = ii->size[0];
      ii->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
      ar = anchorIdx->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        ii->data[i17] = (int)(((c_numStates + (anchorIdx->data[anchorIdx->size[0]
          * i17] - 1.0) * b_numStatesPerAnchor) + 6.0) + (double)nx);
      }

      ar = P_apr->size[0];
      nm1d2 = ii->size[0];
      for (i17 = 0; i17 < nm1d2; i17++) {
        for (i18 = 0; i18 < ar; i18++) {
          P_apr->data[i18 + P_apr->size[0] * (ii->data[i17] - 1)] = 0.0;
        }
      }

      i17 = ii->size[0];
      ii->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
      ar = anchorIdx->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        ii->data[i17] = (int)anchorIdx->data[anchorIdx->size[0] * i17];
      }

      nm1d2 = ii->size[0];
      for (i17 = 0; i17 < nm1d2; i17++) {
        b_anchorFeatures->data[i + b_anchorFeatures->size[0] * (ii->data[i17] -
          1)] = -1.0;
      }

      //  mark feature as lost
    }

    i++;
  }

  emxFree_real_T(&b_anchorIdx);
  emxFree_int32_T(&b_ii);
  emxFree_boolean_T(&x);
  emxInit_boolean_T(&c_anchorFeatures, 2);

  // % do the update
  i17 = c_anchorFeatures->size[0] * c_anchorFeatures->size[1];
  c_anchorFeatures->size[0] = b_anchorFeatures->size[0];
  c_anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)c_anchorFeatures, i17, (int)sizeof
                    (boolean_T));
  ar = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i17 = 0; i17 < ar; i17++) {
    c_anchorFeatures->data[i17] = (b_anchorFeatures->data[i17] == 1.0);
  }

  b_emxInit_boolean_T(&b_x, 1);
  b_any(c_anchorFeatures, b_x);
  nx = b_x->size[0];
  idx = 0;
  i17 = ii->size[0];
  ii->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
  nm1d2 = 1;
  emxFree_boolean_T(&c_anchorFeatures);
  exitg3 = false;
  while ((!exitg3) && (nm1d2 <= nx)) {
    guard3 = false;
    if (b_x->data[nm1d2 - 1]) {
      idx++;
      ii->data[idx - 1] = nm1d2;
      if (idx >= nx) {
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

  if (b_x->size[0] == 1) {
    if (idx == 0) {
      i17 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
    }
  } else {
    i17 = ii->size[0];
    if (1 > idx) {
      ii->size[0] = 0;
    } else {
      ii->size[0] = idx;
    }

    emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
  }

  emxInit_real_T(&indMeas, 1);
  i17 = indMeas->size[0];
  indMeas->size[0] = ii->size[0];
  emxEnsureCapacity((emxArray__common *)indMeas, i17, (int)sizeof(double));
  ar = ii->size[0];
  for (i17 = 0; i17 < ar; i17++) {
    indMeas->data[i17] = ii->data[i17];
  }

  emxInit_real_T(&validFeatures, 1);
  if (!(indMeas->size[0] == 0)) {
    //      fprintf('Update of model %i\n', SLAM_data.model)
    Mahalanobis_EKF(b_xt, P_apr, z_all_l, b_SLAM_data->numStatesxt,
                    b_SLAM_data->numStates, b_SLAM_data->numPointsPerAnchor,
                    b_SLAM_data->cameraparams.CameraParameters1.RadialDistortion,
                    b_SLAM_data->cameraparams.CameraParameters1.FocalLength,
                    b_SLAM_data->cameraparams.CameraParameters1.PrincipalPoint,
                    b_anchorFeatures, b_SLAM_data->m_vect, b_SLAM_data->imNoise,
                    IMU_measurements, validFeatures, &b_SLAM_data->model_prob);

    //          [ xt, P_apo, validFeatures, model_prob ] = OnePointRANSAC_EKF(xt, P_apr, z_all_l, numStatesxt, numStates, numPointsPerAnchor, cameraparams, anchorFeatures, m_vect, imNoise,IMU_measurements,height_offset_pressure); 
    i17 = ii->size[0];
    ii->size[0] = indMeas->size[0];
    emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
    ar = indMeas->size[0];
    for (i17 = 0; i17 < ar; i17++) {
      ii->data[i17] = (int)indMeas->data[i17];
    }

    ar = ii->size[0];
    for (i17 = 0; i17 < ar; i17++) {
      updateVect[ii->data[i17] - 1] = 0.0;
    }

    i17 = ii->size[0];
    ii->size[0] = validFeatures->size[0];
    emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
    ar = validFeatures->size[0];
    for (i17 = 0; i17 < ar; i17++) {
      ii->data[i17] = (int)validFeatures->data[i17];
    }

    ar = ii->size[0];
    for (i17 = 0; i17 < ar; i17++) {
      updateVect[ii->data[i17] - 1] = 1.0;
    }

    //  check for lost features
    //      for i = 1:numTrackFeatures
    //          anchorIdx = find(anchorFeatures(i, :)==1);
    //          if ~isempty(anchorIdx) && updateVect(i) ~= 1
    //              featureAnchorIdx = nnz(anchorFeatures(1:i, anchorIdx));
    //
    //              % remove covariance of this feature with rest of state
    //              P_apo(numStates + (anchorIdx - 1)*numStatesPerAnchor + 6 + featureAnchorIdx, :) = 0; 
    //              P_apo(:, numStates + (anchorIdx - 1)*numStatesPerAnchor + 6 + featureAnchorIdx) = 0; 
    //
    //              anchorFeatures(i, anchorIdx) = -1; % mark feature as lost
    //          end
    //      end
  }

  // % Initialize new anchors
  for (i = 0; i < 16; i++) {
    b_updateVect[i] = (updateVect[i] == 2.0);
  }

  if (d_any(b_updateVect)) {
    //  if there are any features with stereo measurements
    initializeNewAnchor = 0.0;
    minNumValidFeatures = 10000;
    nm1d2 = 0;
    exitg2 = false;
    while ((!exitg2) && (nm1d2 <= (int)b_SLAM_data->numAnchors - 1)) {
      ar = b_anchorFeatures->size[0];
      i17 = b_x->size[0];
      b_x->size[0] = ar;
      emxEnsureCapacity((emxArray__common *)b_x, i17, (int)sizeof(boolean_T));
      for (i17 = 0; i17 < ar; i17++) {
        b_x->data[i17] = (b_anchorFeatures->data[i17 + b_anchorFeatures->size[0]
                          * nm1d2] == 1.0);
      }

      nx = 0;
      for (k = 0; k < b_x->size[0]; k++) {
        if (b_x->data[k]) {
          nx++;
        }
      }

      guard2 = false;
      if ((nx < minFeatureThreshold) && (nx < minNumValidFeatures)) {
        minNumValidFeatures = nx;
        initializeNewAnchor = 1.0 + (double)nm1d2;
        if (!(nx != 0)) {
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
      for (i17 = 0; i17 < 3; i17++) {
        r_wc[i17] = b_xt->data[i17];
      }

      ar = b_anchorFeatures->size[0];
      i17 = b_x->size[0];
      b_x->size[0] = ar;
      emxEnsureCapacity((emxArray__common *)b_x, i17, (int)sizeof(boolean_T));
      for (i17 = 0; i17 < ar; i17++) {
        b_x->data[i17] = (b_anchorFeatures->data[i17 + b_anchorFeatures->size[0]
                          * ((int)initializeNewAnchor - 1)] == 1.0);
      }

      nx = b_x->size[0] - 1;
      nm1d2 = 0;
      for (i = 0; i <= nx; i++) {
        if (b_x->data[i]) {
          nm1d2++;
        }
      }

      i17 = ii->size[0];
      ii->size[0] = nm1d2;
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
      nm1d2 = 0;
      for (i = 0; i <= nx; i++) {
        if (b_x->data[i]) {
          ii->data[nm1d2] = i + 1;
          nm1d2++;
        }
      }

      nm1d2 = ii->size[0];
      for (i17 = 0; i17 < nm1d2; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          b_m_vect->data[i18 + b_m_vect->size[0] * (ii->data[i17] - 1)] = rtNaN;
        }
      }

      ar = b_anchorFeatures->size[0];
      for (i17 = 0; i17 < ar; i17++) {
        b_anchorFeatures->data[i17 + b_anchorFeatures->size[0] * ((int)
          initializeNewAnchor - 1)] = 0.0;
      }

      anew = b_SLAM_data->numStatesxt + (initializeNewAnchor - 1.0) * (7.0 +
        b_SLAM_data->numPointsPerAnchor);
      for (i17 = 0; i17 < 7; i17++) {
        c_xt[i17] = b_xt->data[i17];
      }

      for (i17 = 0; i17 < 7; i17++) {
        b_xt->data[(int)(anew + (1.0 + (double)i17)) - 1] = c_xt[i17];
      }

      i17 = indMeas->size[0];
      indMeas->size[0] = (int)b_SLAM_data->numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)indMeas, i17, (int)sizeof(double));
      ar = (int)b_SLAM_data->numPointsPerAnchor;
      for (i17 = 0; i17 < ar; i17++) {
        indMeas->data[i17] = 0.0;
      }

      emxInit_boolean_T(&d_anchorFeatures, 2);
      i17 = d_anchorFeatures->size[0] * d_anchorFeatures->size[1];
      d_anchorFeatures->size[0] = b_anchorFeatures->size[0];
      d_anchorFeatures->size[1] = b_anchorFeatures->size[1];
      emxEnsureCapacity((emxArray__common *)d_anchorFeatures, i17, (int)sizeof
                        (boolean_T));
      ar = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        d_anchorFeatures->data[i17] = (b_anchorFeatures->data[i17] == 1.0);
      }

      b_any(d_anchorFeatures, b_x);
      i17 = b_x->size[0];
      emxEnsureCapacity((emxArray__common *)b_x, i17, (int)sizeof(boolean_T));
      ar = b_x->size[0];
      emxFree_boolean_T(&d_anchorFeatures);
      for (i17 = 0; i17 < ar; i17++) {
        b_x->data[i17] = !b_x->data[i17];
      }

      for (i17 = 0; i17 < 16; i17++) {
        b_updateVect[i17] = (b_x->data[i17] && (updateVect[i17] == 2.0));
      }

      idx = 0;
      nm1d2 = 1;
      exitg1 = false;
      while ((!exitg1) && (nm1d2 < 17)) {
        guard1 = false;
        if (b_updateVect[nm1d2 - 1]) {
          idx++;
          ii_data[idx - 1] = (signed char)nm1d2;
          if (idx >= 16) {
            exitg1 = true;
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          nm1d2++;
        }
      }

      if (1 > idx) {
        k = 0;
      } else {
        k = idx;
      }

      if (1 > idx) {
        ar = 0;
      } else {
        ar = idx;
      }

      for (i17 = 0; i17 < ar; i17++) {
        unusedFeatures_data[i17] = ii_data[i17];
      }

      featureAnchorIdx = 1U;
      idx = 0;
      while ((idx <= k - 1) && (!(featureAnchorIdx > b_numPointsPerAnchor))) {
        nm1d2 = (unusedFeatures_data[idx] - 1) * 2;
        nx = (unusedFeatures_data[idx] - 1) * 2;
        for (i17 = 0; i17 < 2; i17++) {
          z_curr_l[i17] = z_all_l[nm1d2 + i17];
          z_curr_r[i17] = z_all_r[nx + i17];
        }

        initializePoint(b_xt,
                        b_SLAM_data->cameraparams.CameraParameters1.RadialDistortion,
                        b_SLAM_data->cameraparams.CameraParameters1.FocalLength,
                        b_SLAM_data->cameraparams.CameraParameters1.PrincipalPoint,
                        b_SLAM_data->cameraparams.CameraParameters2.RadialDistortion,
                        b_SLAM_data->cameraparams.CameraParameters2.FocalLength,
                        b_SLAM_data->cameraparams.CameraParameters2.PrincipalPoint,
                        b_SLAM_data->cameraparams.r_lr,
                        b_SLAM_data->cameraparams.R_lr, z_curr_l, z_curr_r, fp,
                        m);
        for (i = 0; i < 3; i++) {
          bv0[i] = rtIsNaN(fp[i]);
        }

        if (c_any(bv0)) {
          updateVect[unusedFeatures_data[idx] - 1] = 0.0;
        } else {
          //  check reprojection error
          for (i17 = 0; i17 < 3; i17++) {
            b_fp[i17] = fp[i17] - r_wc[i17];
          }

          for (i17 = 0; i17 < 3; i17++) {
            b_R_cw[i17] = 0.0;
            for (i18 = 0; i18 < 3; i18++) {
              b_R_cw[i17] += R_cw[i17 + 3 * i18] * b_fp[i18];
            }
          }

          predictMeasurement_stereo(b_R_cw,
            b_SLAM_data->cameraparams.CameraParameters1.RadialDistortion,
            b_SLAM_data->cameraparams.CameraParameters1.FocalLength,
            b_SLAM_data->cameraparams.CameraParameters1.PrincipalPoint,
            b_SLAM_data->cameraparams.CameraParameters2.RadialDistortion,
            b_SLAM_data->cameraparams.CameraParameters2.FocalLength,
            b_SLAM_data->cameraparams.CameraParameters2.PrincipalPoint,
            b_SLAM_data->cameraparams.r_lr, b_SLAM_data->cameraparams.R_rl,
            h_u_l, h_u_r);
          for (i = 0; i < 2; i++) {
            b_h_u_l[i] = h_u_l[i] - z_curr_l[i];
          }

          b_guard1 = false;
          if (d_norm(b_h_u_l) > 10.0) {
            b_guard1 = true;
          } else {
            for (i = 0; i < 2; i++) {
              b_h_u_l[i] = h_u_r[i] - z_curr_r[i];
            }

            if (d_norm(b_h_u_l) > 10.0) {
              b_guard1 = true;
            } else {
              for (i17 = 0; i17 < 3; i17++) {
                b_m_vect->data[i17 + b_m_vect->size[0] *
                  (unusedFeatures_data[idx] - 1)] = m[i17];
              }

              for (i17 = 0; i17 < 3; i17++) {
                b_fp[i17] = b_xt->data[i17] - fp[i17];
              }

              rhoInit = 1.0 / norm(b_fp);

              //                      sigmaInit = 0.00001/(rhoInit);
              b_xt->data[(int)(((b_SLAM_data->numStatesxt + (initializeNewAnchor
                - 1.0) * (7.0 + b_numPointsPerAnchor)) + 7.0) + (double)
                               featureAnchorIdx) - 1] = rhoInit;
              indMeas->data[(int)featureAnchorIdx - 1] = sigma_Init / rhoInit;
              b_anchorFeatures->data[(unusedFeatures_data[idx] +
                b_anchorFeatures->size[0] * ((int)initializeNewAnchor - 1)) - 1]
                = 1.0;
              updateVect[unusedFeatures_data[idx] - 1] = 1.0;
              featureAnchorIdx++;
            }
          }

          if (b_guard1) {
            i17 = (int)rt_roundd_snf((double)unusedFeatures_data[idx]);
            i19 = (signed char)i17;
            f_fprintf(i19);
            updateVect[unusedFeatures_data[idx] - 1] = 0.0;
          }
        }

        idx++;
      }

      anew = rt_roundd_snf((double)featureAnchorIdx - 1.0);
      if (anew < 2.147483648E+9) {
        i17 = (int)anew;
      } else {
        i17 = MAX_int32_T;
      }

      h_fprintf(i17, (int)initializeNewAnchor);
      if (rtIsNaN(b_numStatesPerAnchor)) {
        nx = 0;
        anew = rtNaN;
        apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
      } else if (rtIsInf(b_numStatesPerAnchor)) {
        nx = 0;
        anew = rtNaN;
        apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor((b_numStatesPerAnchor - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + b_SLAM_data->numPointsPerAnchor);
        absb = fabs(6.0 + b_SLAM_data->numPointsPerAnchor);
        if ((1.0 >= absb) || rtIsNaN(absb)) {
          absb = 1.0;
        }

        if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
          ndbl++;
          apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        nx = (int)ndbl - 1;
      }

      i17 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = nx + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i17, (int)sizeof(double));
      anchorIdx->data[0] = anew;
      if (nx + 1 > 1) {
        anchorIdx->data[nx] = apnd;
        nm1d2 = (nx + (nx < 0)) >> 1;
        for (k = 1; k < nm1d2; k++) {
          anchorIdx->data[k] = anew + (double)k;
          anchorIdx->data[nx - k] = apnd - (double)k;
        }

        if (nm1d2 << 1 == nx) {
          anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          anchorIdx->data[nm1d2] = anew + (double)nm1d2;
          anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }

      anew = b_SLAM_data->numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_SLAM_data->numPointsPerAnchor);
      i17 = ii->size[0];
      ii->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
      ar = anchorIdx->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        ii->data[i17] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i17]);
      }

      ar = P_apr->size[1];
      nm1d2 = ii->size[0];
      for (i17 = 0; i17 < ar; i17++) {
        for (i18 = 0; i18 < nm1d2; i18++) {
          P_apr->data[(ii->data[i18] + P_apr->size[0] * i17) - 1] = 0.0;
        }
      }

      if (rtIsNaN(b_numStatesPerAnchor)) {
        nx = 0;
        anew = rtNaN;
        apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
      } else if (rtIsInf(b_numStatesPerAnchor)) {
        nx = 0;
        anew = rtNaN;
        apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor((b_numStatesPerAnchor - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + b_SLAM_data->numPointsPerAnchor);
        absb = fabs(6.0 + b_SLAM_data->numPointsPerAnchor);
        if ((1.0 >= absb) || rtIsNaN(absb)) {
          absb = 1.0;
        }

        if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
          ndbl++;
          apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        nx = (int)ndbl - 1;
      }

      i17 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = nx + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i17, (int)sizeof(double));
      anchorIdx->data[0] = anew;
      if (nx + 1 > 1) {
        anchorIdx->data[nx] = apnd;
        nm1d2 = (nx + (nx < 0)) >> 1;
        for (k = 1; k < nm1d2; k++) {
          anchorIdx->data[k] = anew + (double)k;
          anchorIdx->data[nx - k] = apnd - (double)k;
        }

        if (nm1d2 << 1 == nx) {
          anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          anchorIdx->data[nm1d2] = anew + (double)nm1d2;
          anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }

      anew = b_SLAM_data->numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_SLAM_data->numPointsPerAnchor);
      i17 = ii->size[0];
      ii->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
      ar = anchorIdx->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        ii->data[i17] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i17]);
      }

      ar = P_apr->size[0];
      nm1d2 = ii->size[0];
      for (i17 = 0; i17 < nm1d2; i17++) {
        for (i18 = 0; i18 < ar; i18++) {
          P_apr->data[i18 + P_apr->size[0] * (ii->data[i17] - 1)] = 0.0;
        }
      }

      for (i17 = 0; i17 < 2; i17++) {
        b_h_u_l[i17] = P_apr->size[i17];
      }

      b_emxInit_real_T(&J, 2);
      b_eye(b_h_u_l, J);
      if (rtIsNaN(b_numStatesPerAnchor)) {
        nx = 0;
        anew = rtNaN;
        apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
      } else if (rtIsInf(b_numStatesPerAnchor)) {
        nx = 0;
        anew = rtNaN;
        apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor((b_numStatesPerAnchor - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + b_SLAM_data->numPointsPerAnchor);
        absb = fabs(6.0 + b_SLAM_data->numPointsPerAnchor);
        if ((1.0 >= absb) || rtIsNaN(absb)) {
          absb = 1.0;
        }

        if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
          ndbl++;
          apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        nx = (int)ndbl - 1;
      }

      i17 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = nx + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i17, (int)sizeof(double));
      anchorIdx->data[0] = anew;
      if (nx + 1 > 1) {
        anchorIdx->data[nx] = apnd;
        nm1d2 = (nx + (nx < 0)) >> 1;
        for (k = 1; k < nm1d2; k++) {
          anchorIdx->data[k] = anew + (double)k;
          anchorIdx->data[nx - k] = apnd - (double)k;
        }

        if (nm1d2 << 1 == nx) {
          anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          anchorIdx->data[nm1d2] = anew + (double)nm1d2;
          anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }

      anew = b_SLAM_data->numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_SLAM_data->numPointsPerAnchor);
      i17 = ii->size[0];
      ii->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
      ar = anchorIdx->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        ii->data[i17] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i17]);
      }

      ar = J->size[1];
      nm1d2 = ii->size[0];
      for (i17 = 0; i17 < ar; i17++) {
        for (i18 = 0; i18 < nm1d2; i18++) {
          J->data[(ii->data[i18] + J->size[0] * i17) - 1] = 0.0;
        }
      }

      if (rtIsNaN(b_numStatesPerAnchor)) {
        nx = 0;
        anew = rtNaN;
        apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
      } else if (rtIsInf(b_numStatesPerAnchor)) {
        nx = 0;
        anew = rtNaN;
        apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor((b_numStatesPerAnchor - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + b_SLAM_data->numPointsPerAnchor);
        absb = fabs(6.0 + b_SLAM_data->numPointsPerAnchor);
        if ((1.0 >= absb) || rtIsNaN(absb)) {
          absb = 1.0;
        }

        if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
          ndbl++;
          apnd = 6.0 + b_SLAM_data->numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        nx = (int)ndbl - 1;
      }

      i17 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = nx + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i17, (int)sizeof(double));
      anchorIdx->data[0] = anew;
      if (nx + 1 > 1) {
        anchorIdx->data[nx] = apnd;
        nm1d2 = (nx + (nx < 0)) >> 1;
        for (k = 1; k < nm1d2; k++) {
          anchorIdx->data[k] = anew + (double)k;
          anchorIdx->data[nx - k] = apnd - (double)k;
        }

        if (nm1d2 << 1 == nx) {
          anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          anchorIdx->data[nm1d2] = anew + (double)nm1d2;
          anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }

      anew = b_SLAM_data->numStates + (initializeNewAnchor - 1.0) * (6.0 +
        b_SLAM_data->numPointsPerAnchor);
      i17 = ii->size[0];
      ii->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i17, (int)sizeof(int));
      ar = anchorIdx->size[1];
      for (i17 = 0; i17 < ar; i17++) {
        ii->data[i17] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i17])
          - 1;
      }

      nm1d2 = (int)b_SLAM_data->numPointsPerAnchor;
      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          J->data[ii->data[i18] + J->size[0] * i17] = iv14[i18 + 3 * i17];
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          J->data[ii->data[i18] + J->size[0] * (i17 + 3)] = 0.0;
        }
      }

      ar = (int)(b_SLAM_data->numStates - 6.0);
      for (i17 = 0; i17 < ar; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          J->data[ii->data[i18] + J->size[0] * (i17 + 6)] = 0.0;
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          J->data[ii->data[i18 + 3] + J->size[0] * i17] = 0.0;
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          J->data[ii->data[i18 + 3] + J->size[0] * (i17 + 3)] = iv14[i18 + 3 *
            i17];
        }
      }

      ar = (int)(b_SLAM_data->numStates - 6.0);
      for (i17 = 0; i17 < ar; i17++) {
        for (i18 = 0; i18 < 3; i18++) {
          J->data[ii->data[i18 + 3] + J->size[0] * (i17 + 6)] = 0.0;
        }
      }

      ar = (int)b_SLAM_data->numStates;
      for (i17 = 0; i17 < ar; i17++) {
        for (i18 = 0; i18 < nm1d2; i18++) {
          J->data[ii->data[i18 + 6] + J->size[0] * i17] = 0.0;
        }
      }

      b_emxInit_real_T(&y, 2);
      if ((J->size[1] == 1) || (P_apr->size[0] == 1)) {
        i17 = y->size[0] * y->size[1];
        y->size[0] = J->size[0];
        y->size[1] = P_apr->size[1];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        ar = J->size[0];
        for (i17 = 0; i17 < ar; i17++) {
          nm1d2 = P_apr->size[1];
          for (i18 = 0; i18 < nm1d2; i18++) {
            y->data[i17 + y->size[0] * i18] = 0.0;
            nx = J->size[1];
            for (idx = 0; idx < nx; idx++) {
              y->data[i17 + y->size[0] * i18] += J->data[i17 + J->size[0] * idx]
                * P_apr->data[idx + P_apr->size[0] * i18];
            }
          }
        }
      } else {
        k = J->size[1];
        nm1d2 = J->size[0];
        nx = P_apr->size[1];
        b_m = J->size[0];
        i17 = y->size[0] * y->size[1];
        y->size[0] = nm1d2;
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        i17 = y->size[0] * y->size[1];
        y->size[1] = nx;
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        ar = nm1d2 * nx;
        for (i17 = 0; i17 < ar; i17++) {
          y->data[i17] = 0.0;
        }

        if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
        } else {
          nm1d2 = J->size[0] * (P_apr->size[1] - 1);
          nx = 0;
          while ((b_m > 0) && (nx <= nm1d2)) {
            i17 = nx + b_m;
            for (ic = nx; ic + 1 <= i17; ic++) {
              y->data[ic] = 0.0;
            }

            nx += b_m;
          }

          idx = 0;
          nx = 0;
          while ((b_m > 0) && (nx <= nm1d2)) {
            ar = 0;
            i17 = idx + k;
            for (ib = idx; ib + 1 <= i17; ib++) {
              if (P_apr->data[ib] != 0.0) {
                ia = ar;
                i18 = nx + b_m;
                for (ic = nx; ic + 1 <= i18; ic++) {
                  ia++;
                  y->data[ic] += P_apr->data[ib] * J->data[ia - 1];
                }
              }

              ar += b_m;
            }

            idx += k;
            nx += b_m;
          }
        }
      }

      b_emxInit_real_T(&b, 2);
      i17 = b->size[0] * b->size[1];
      b->size[0] = J->size[1];
      b->size[1] = J->size[0];
      emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
      ar = J->size[0];
      for (i17 = 0; i17 < ar; i17++) {
        nm1d2 = J->size[1];
        for (i18 = 0; i18 < nm1d2; i18++) {
          b->data[i18 + b->size[0] * i17] = J->data[i17 + J->size[0] * i18];
        }
      }

      emxFree_real_T(&J);
      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i17 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = y->size[0];
        P_apr->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i17, (int)sizeof(double));
        ar = y->size[0];
        for (i17 = 0; i17 < ar; i17++) {
          nm1d2 = b->size[1];
          for (i18 = 0; i18 < nm1d2; i18++) {
            P_apr->data[i17 + P_apr->size[0] * i18] = 0.0;
            nx = y->size[1];
            for (idx = 0; idx < nx; idx++) {
              P_apr->data[i17 + P_apr->size[0] * i18] += y->data[i17 + y->size[0]
                * idx] * b->data[idx + b->size[0] * i18];
            }
          }
        }
      } else {
        k = y->size[1];
        nm1d2 = y->size[0];
        nx = b->size[1];
        b_m = y->size[0];
        i17 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = nm1d2;
        emxEnsureCapacity((emxArray__common *)P_apr, i17, (int)sizeof(double));
        i17 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[1] = nx;
        emxEnsureCapacity((emxArray__common *)P_apr, i17, (int)sizeof(double));
        ar = nm1d2 * nx;
        for (i17 = 0; i17 < ar; i17++) {
          P_apr->data[i17] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          nm1d2 = y->size[0] * (b->size[1] - 1);
          nx = 0;
          while ((b_m > 0) && (nx <= nm1d2)) {
            i17 = nx + b_m;
            for (ic = nx; ic + 1 <= i17; ic++) {
              P_apr->data[ic] = 0.0;
            }

            nx += b_m;
          }

          idx = 0;
          nx = 0;
          while ((b_m > 0) && (nx <= nm1d2)) {
            ar = 0;
            i17 = idx + k;
            for (ib = idx; ib + 1 <= i17; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i18 = nx + b_m;
                for (ic = nx; ic + 1 <= i18; ic++) {
                  ia++;
                  P_apr->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += b_m;
            }

            idx += k;
            nx += b_m;
          }
        }
      }

      emxFree_real_T(&b);
      emxFree_real_T(&y);
      for (nm1d2 = 0; nm1d2 < (int)b_numPointsPerAnchor; nm1d2++) {
        P_apr->data[((int)(((c_numStates + (initializeNewAnchor - 1.0) * (6.0 +
          b_numPointsPerAnchor)) + 6.0) + (1.0 + (double)nm1d2)) + P_apr->size[0]
                     * ((int)(((c_numStates + (initializeNewAnchor - 1.0) * (6.0
          + b_numPointsPerAnchor)) + 6.0) + (1.0 + (double)nm1d2)) - 1)) - 1] =
          indMeas->data[nm1d2];
      }
    } else {
      j_fprintf();
    }
  }

  emxFree_int32_T(&ii);
  emxFree_boolean_T(&b_x);
  emxFree_real_T(&anchorIdx);

  //  % determine if a new anchor needs to be initialized, and request stereo
  //  % measurements for it
  //  minNumValidFeatures = 10000;
  //  for anchorIdx = 1:numAnchors
  //      numValidFeatures = nnz(anchorFeatures(:,anchorIdx) == 1 & updateVect == 1); 
  //      if numValidFeatures < minFeatureThreshold && numValidFeatures < minNumValidFeatures 
  //          minNumValidFeatures = numValidFeatures;
  //          initializeNewAnchor = anchorIdx;
  //          if ~minNumValidFeatures
  //              break
  //          end
  //      end
  //  end
  //  if minNumValidFeatures < minFeatureThreshold
  //      newFeaturesRequested = 0;
  //      for i = 1:length(updateVect)
  //          if ~any(anchorFeatures(i,:)==1) || anchorFeatures(i,initializeNewAnchor) == 1 
  //  %             updateVect(i) = 2;
  //              newFeaturesRequested = newFeaturesRequested + 1;
  //              if newFeaturesRequested == numPointsPerAnchor
  //                  break;
  //              end
  //          end
  //      end
  //      fprintf('Requesting %i new features\n', int8(newFeaturesRequested))
  //  end
  // % aposteriori measurement prediction
  getMap(b_xt, b_anchorFeatures, b_m_vect, c_numTrackFeatures,
         b_SLAM_data->numStatesxt, 7.0 + b_SLAM_data->numPointsPerAnchor, map,
         indMeas, validFeatures);
  i17 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(c_numTrackFeatures * 4.0);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i17, (int)sizeof(double));
  ar = (int)(c_numTrackFeatures * 4.0);
  emxFree_real_T(&validFeatures);
  emxFree_real_T(&indMeas);
  for (i17 = 0; i17 < ar; i17++) {
    h_u_apo->data[i17] = rtNaN;
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
  emxInit_boolean_T(&e_anchorFeatures, 2);
  for (i = 0; i < 16; i++) {
    ar = b_anchorFeatures->size[1];
    i17 = e_anchorFeatures->size[0] * e_anchorFeatures->size[1];
    e_anchorFeatures->size[0] = 1;
    e_anchorFeatures->size[1] = ar;
    emxEnsureCapacity((emxArray__common *)e_anchorFeatures, i17, (int)sizeof
                      (boolean_T));
    for (i17 = 0; i17 < ar; i17++) {
      e_anchorFeatures->data[e_anchorFeatures->size[0] * i17] =
        (b_anchorFeatures->data[i + b_anchorFeatures->size[0] * i17] == 1.0);
    }

    if (e_any(e_anchorFeatures)) {
      for (i17 = 0; i17 < 3; i17++) {
        b_fp[i17] = map->data[i17 + map->size[0] * i] - b_xt->data[i17];
      }

      for (i17 = 0; i17 < 3; i17++) {
        b_R_cw[i17] = 0.0;
        for (i18 = 0; i18 < 3; i18++) {
          b_R_cw[i17] += R_cw[i17 + 3 * i18] * b_fp[i18];
        }
      }

      predictMeasurement_stereo(b_R_cw,
        b_SLAM_data->cameraparams.CameraParameters1.RadialDistortion,
        b_SLAM_data->cameraparams.CameraParameters1.FocalLength,
        b_SLAM_data->cameraparams.CameraParameters1.PrincipalPoint,
        b_SLAM_data->cameraparams.CameraParameters2.RadialDistortion,
        b_SLAM_data->cameraparams.CameraParameters2.FocalLength,
        b_SLAM_data->cameraparams.CameraParameters2.PrincipalPoint,
        b_SLAM_data->cameraparams.r_lr, b_SLAM_data->cameraparams.R_rl, h_u_l,
        h_u_r);
      nm1d2 = i << 2;
      for (i17 = 0; i17 < 2; i17++) {
        h_u_apo->data[i17 + nm1d2] = h_u_l[i17];
      }

      for (i17 = 0; i17 < 2; i17++) {
        h_u_apo->data[(i17 + nm1d2) + 2] = h_u_r[i17];
      }
    }
  }

  emxFree_boolean_T(&e_anchorFeatures);

  // %
  i17 = b_SLAM_data->xt->size[0];
  b_SLAM_data->xt->size[0] = b_xt->size[0];
  emxEnsureCapacity((emxArray__common *)b_SLAM_data->xt, i17, (int)sizeof(double));
  ar = b_xt->size[0];
  for (i17 = 0; i17 < ar; i17++) {
    b_SLAM_data->xt->data[i17] = b_xt->data[i17];
  }

  emxFree_real_T(&b_xt);
  i17 = b_SLAM_data->P->size[0] * b_SLAM_data->P->size[1];
  b_SLAM_data->P->size[0] = P_apr->size[0];
  b_SLAM_data->P->size[1] = P_apr->size[1];
  emxEnsureCapacity((emxArray__common *)b_SLAM_data->P, i17, (int)sizeof(double));
  ar = P_apr->size[0] * P_apr->size[1];
  for (i17 = 0; i17 < ar; i17++) {
    b_SLAM_data->P->data[i17] = P_apr->data[i17];
  }

  emxFree_real_T(&P_apr);
  i17 = b_SLAM_data->anchorFeatures->size[0] * b_SLAM_data->anchorFeatures->
    size[1];
  b_SLAM_data->anchorFeatures->size[0] = b_anchorFeatures->size[0];
  b_SLAM_data->anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)b_SLAM_data->anchorFeatures, i17, (int)
                    sizeof(double));
  ar = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i17 = 0; i17 < ar; i17++) {
    b_SLAM_data->anchorFeatures->data[i17] = b_anchorFeatures->data[i17];
  }

  emxFree_real_T(&b_anchorFeatures);
  i17 = b_SLAM_data->m_vect->size[0] * b_SLAM_data->m_vect->size[1];
  b_SLAM_data->m_vect->size[0] = 3;
  b_SLAM_data->m_vect->size[1] = b_m_vect->size[1];
  emxEnsureCapacity((emxArray__common *)b_SLAM_data->m_vect, i17, (int)sizeof
                    (double));
  ar = b_m_vect->size[0] * b_m_vect->size[1];
  for (i17 = 0; i17 < ar; i17++) {
    b_SLAM_data->m_vect->data[i17] = b_m_vect->data[i17];
  }

  emxFree_real_T(&b_m_vect);
}

//
// File trailer for SLAM_updIT.cpp
//
// [EOF]
//
