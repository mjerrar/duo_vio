//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 20-Aug-2015 15:34:21
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "OnePointRANSAC_EKF.h"
#include "quatmultJ.h"
#include "quatPlusThetaJ.h"
#include "any.h"
#include "norm.h"
#include "predictMeasurement_left.h"
#include "SLAM_emxutil.h"
#include "mrdivide.h"
#include "getH_R_res.h"
#include "rand.h"
#include "fprintf.h"
#include "eye.h"
#include "getMap.h"
#include "SLAM_data.h"
#include <stdio.h>

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : emxArray_real_T *b_xt
//                emxArray_real_T *b_P
//                const double z_all_l[32]
//                double c_numStatesxt
//                double c_numStates
//                double numPointsPerAnchor
//                const double c_cameraparams_CameraParameters[3]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const emxArray_real_T *b_anchorFeatures
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[23]
//                double height_offset_pressure
//                double validFeatures_data[]
//                int validFeatures_size[1]
// Return Type  : void
//
void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P, const
  double z_all_l[32], double c_numStatesxt, double c_numStates, double
  numPointsPerAnchor, const double c_cameraparams_CameraParameters[3], const
  double d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const emxArray_real_T *b_anchorFeatures,
  const emxArray_real_T *b_m_vect, const double imNoise[2], const double
  IMU_measurements[23], double height_offset_pressure, double
  validFeatures_data[], int validFeatures_size[1])
{
  emxArray_boolean_T *c_anchorFeatures;
  int numAnchors;
  int i16;
  int loop_ub;
  boolean_T x[16];
  int idx;
  int ii_data[16];
  int ii;
  boolean_T exitg3;
  boolean_T guard2 = false;
  int ar;
  signed char indMeas_data[16];
  emxArray_real_T *K;
  emxArray_real_T *H;
  int numMeas;
  double n_hyp;
  int LI_inlierStatus_size_idx_0;
  boolean_T LI_inlierStatus_data[16];
  emxArray_real_T *map;
  emxArray_real_T *anchorInd;
  emxArray_real_T *featureAnchorInd;
  double i;
  emxArray_real_T *K_i;
  emxArray_real_T *x_apo;
  emxArray_real_T *x_apo_prev;
  emxArray_real_T *H_i;
  emxArray_real_T *C;
  emxArray_int32_T *r11;
  emxArray_real_T *b;
  emxArray_real_T *y;
  emxArray_real_T *r12;
  emxArray_int32_T *r13;
  emxArray_int32_T *r14;
  emxArray_real_T *b_x_apo_prev;
  double d_numStatesxt;
  int R_size[2];
  double R_data[36];
  double r[2];
  int unusedU2_size[1];
  double c_xt[4];
  int ib;
  int i17;
  int i18;
  int k;
  double a[2];
  int m;
  int ic;
  int br;
  int ia;
  double C_data[36];
  int C_size[2];
  double d_xt[3];
  double dv26[4];
  double dv27[4];
  double e_numStatesxt;
  double d_numStates;
  double c_x_apo_prev[4];
  double dv28[4];
  double R_cw[9];
  boolean_T HI_inlierStatus_data[16];
  int indMeasIdx;
  double featureAnchorIdx;
  boolean_T exitg2;
  boolean_T guard1 = false;
  signed char featureIdxVect_data[16];
  double anchorPos[3];
  double d_x_apo_prev[9];
  double b_anchorPos[3];
  static const signed char iv9[3] = { 0, 1, 2 };

  double b_z_all_l[2];
  static const signed char iv10[2] = { 1, 2 };

  emxArray_real_T *b_a;
  int iter;
  emxArray_real_T *e_xt;
  double b_indMeas_data[16];
  int indMeas_size[1];
  int b_R_size[2];
  double b_R_data[1296];
  double unusedU2_data[32];
  int r_size[1];
  double r_data[34];
  double b_C_data[1296];
  int b_C_size[2];
  double dv29[4];
  double dv30[4];
  emxArray_real_T *c_a;
  int it;
  emxArray_real_T *b_x_apo;
  emxArray_real_T *f_xt;
  boolean_T exitg1;
  int b_indMeas_size[1];
  double S_data[1296];
  int c_C_size[2];
  double dv31[4];
  double dv32[4];
  emxArray_real_T *d_a;
  emxInit_boolean_T(&c_anchorFeatures, 2);

  //  threshold for LI innovation test
  // 9.487729036781154; % HI mahalanobis gate
  // 9.487729036781154; % HI mahalanobis gate
  //  threshold for minimum LI supporter
  numAnchors = b_anchorFeatures->size[1];
  i16 = c_anchorFeatures->size[0] * c_anchorFeatures->size[1];
  c_anchorFeatures->size[0] = 16;
  c_anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)c_anchorFeatures, i16, (int)sizeof
                    (boolean_T));
  loop_ub = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i16 = 0; i16 < loop_ub; i16++) {
    c_anchorFeatures->data[i16] = (b_anchorFeatures->data[i16] == 1.0);
  }

  b_any(c_anchorFeatures, x);
  idx = 0;
  ii = 1;
  emxFree_boolean_T(&c_anchorFeatures);
  exitg3 = false;
  while ((!exitg3) && (ii < 17)) {
    guard2 = false;
    if (x[ii - 1]) {
      idx++;
      ii_data[idx - 1] = ii;
      if (idx >= 16) {
        exitg3 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      ii++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  if (1 > idx) {
    ar = 0;
  } else {
    ar = idx;
  }

  for (i16 = 0; i16 < loop_ub; i16++) {
    indMeas_data[i16] = (signed char)ii_data[i16];
  }

  emxInit_real_T(&K, 2);
  emxInit_real_T(&H, 2);
  numMeas = ar;
  i16 = K->size[0] * K->size[1];
  K->size[0] = 1;
  K->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)K, i16, (int)sizeof(double));
  K->data[0] = 0.0;

  //  for coder
  i16 = H->size[0] * H->size[1];
  H->size[0] = 1;
  H->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)H, i16, (int)sizeof(double));
  H->data[0] = 0.0;

  //  for coder
  // % B 1-point hypotheses generation and evaluation
  n_hyp = 100.0;
  LI_inlierStatus_size_idx_0 = ar;
  for (i16 = 0; i16 < ar; i16++) {
    LI_inlierStatus_data[i16] = false;
  }

  emxInit_real_T(&map, 2);
  b_emxInit_real_T(&anchorInd, 1);
  b_emxInit_real_T(&featureAnchorInd, 1);

  //  low innovation inlier status
  //  build the map according to the current estimate
  getMap(b_xt, b_anchorFeatures, b_m_vect, (double)b_anchorFeatures->size[1] *
         numPointsPerAnchor, c_numStatesxt, 7.0 + numPointsPerAnchor, map,
         anchorInd, featureAnchorInd);
  i = 1.0;
  emxInit_real_T(&K_i, 2);
  b_emxInit_real_T(&x_apo, 1);
  b_emxInit_real_T(&x_apo_prev, 1);
  emxInit_real_T(&H_i, 2);
  emxInit_real_T(&C, 2);
  b_emxInit_int32_T(&r11, 2);
  emxInit_real_T(&b, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&r12, 2);
  emxInit_int32_T(&r13, 1);
  emxInit_int32_T(&r14, 1);
  b_emxInit_real_T(&b_x_apo_prev, 1);
  while (i < n_hyp) {
    //  select a random measurement
    d_numStatesxt = b_rand();

    //  EKF state update
    getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, (double)indMeas_data
               [(int)(1.0 + floor(d_numStatesxt * (double)numMeas)) - 1], map,
               c_cameraparams_CameraParameters, d_cameraparams_CameraParameters,
               e_cameraparams_CameraParameters, (double)numAnchors,
               numPointsPerAnchor, anchorInd, featureAnchorInd, b_m_vect,
               imNoise, IMU_measurements, height_offset_pressure, c_xt,
               unusedU2_size, H_i, r, R_data, R_size);
    if ((H_i->size[1] == 1) || (b_P->size[0] == 1)) {
      i16 = y->size[0] * y->size[1];
      y->size[0] = H_i->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
      ib = H_i->size[0];
      for (i16 = 0; i16 < ib; i16++) {
        idx = b_P->size[1];
        for (i17 = 0; i17 < idx; i17++) {
          y->data[i16 + y->size[0] * i17] = 0.0;
          ii = H_i->size[1];
          for (i18 = 0; i18 < ii; i18++) {
            y->data[i16 + y->size[0] * i17] += H_i->data[i16 + H_i->size[0] *
              i18] * b_P->data[i18 + b_P->size[0] * i17];
          }
        }
      }
    } else {
      k = H_i->size[1];
      a[0] = H_i->size[0];
      a[1] = b_P->size[1];
      m = H_i->size[0];
      i16 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
      i16 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i16 = 0; i16 < ib; i16++) {
        y->data[i16] = 0.0;
      }

      if ((H_i->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H_i->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i16 = idx + m;
          for (ic = idx; ic + 1 <= i16; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i16 = br + k;
          for (ib = br; ib + 1 <= i16; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i17 = idx + m;
              for (ic = idx; ic + 1 <= i17; ic++) {
                ia++;
                y->data[ic] += b_P->data[ib] * H_i->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    i16 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = H_i->size[0];
    emxEnsureCapacity((emxArray__common *)b, i16, (int)sizeof(double));
    ib = H_i->size[0];
    for (i16 = 0; i16 < ib; i16++) {
      idx = H_i->size[1];
      for (i17 = 0; i17 < idx; i17++) {
        b->data[i17 + b->size[0] * i16] = H_i->data[i16 + H_i->size[0] * i17];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i16 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
      ib = y->size[0];
      for (i16 = 0; i16 < ib; i16++) {
        idx = b->size[1];
        for (i17 = 0; i17 < idx; i17++) {
          C->data[i16 + C->size[0] * i17] = 0.0;
          ii = y->size[1];
          for (i18 = 0; i18 < ii; i18++) {
            C->data[i16 + C->size[0] * i17] += y->data[i16 + y->size[0] * i18] *
              b->data[i18 + b->size[0] * i17];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i16 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
      i16 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i16 = 0; i16 < ib; i16++) {
        C->data[i16] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i16 = idx + m;
          for (ic = idx; ic + 1 <= i16; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i16 = br + k;
          for (ib = br; ib + 1 <= i16; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i17 = idx + m;
              for (ic = idx; ic + 1 <= i17; ic++) {
                ia++;
                C->data[ic] += b->data[ib] * y->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    i16 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = H_i->size[0];
    emxEnsureCapacity((emxArray__common *)b, i16, (int)sizeof(double));
    ib = H_i->size[0];
    for (i16 = 0; i16 < ib; i16++) {
      idx = H_i->size[1];
      for (i17 = 0; i17 < idx; i17++) {
        b->data[i17 + b->size[0] * i16] = H_i->data[i16 + H_i->size[0] * i17];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i16 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
      ib = b_P->size[0];
      for (i16 = 0; i16 < ib; i16++) {
        idx = b->size[1];
        for (i17 = 0; i17 < idx; i17++) {
          y->data[i16 + y->size[0] * i17] = 0.0;
          ii = b_P->size[1];
          for (i18 = 0; i18 < ii; i18++) {
            y->data[i16 + y->size[0] * i17] += b_P->data[i16 + b_P->size[0] *
              i18] * b->data[i18 + b->size[0] * i17];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i16 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
      i16 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i16, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i16 = 0; i16 < ib; i16++) {
        y->data[i16] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i16 = idx + m;
          for (ic = idx; ic + 1 <= i16; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i16 = br + k;
          for (ib = br; ib + 1 <= i16; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i17 = idx + m;
              for (ic = idx; ic + 1 <= i17; ic++) {
                ia++;
                y->data[ic] += b->data[ib] * b_P->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    C_size[0] = C->size[0];
    C_size[1] = C->size[1];
    ib = C->size[0] * C->size[1];
    for (i16 = 0; i16 < ib; i16++) {
      C_data[i16] = C->data[i16] + R_data[i16];
    }

    mrdivide(y, C_data, C_size, r12);
    i16 = K_i->size[0] * K_i->size[1];
    K_i->size[0] = r12->size[0];
    K_i->size[1] = r12->size[1];
    emxEnsureCapacity((emxArray__common *)K_i, i16, (int)sizeof(double));
    ib = r12->size[0] * r12->size[1];
    for (i16 = 0; i16 < ib; i16++) {
      K_i->data[i16] = r12->data[i16];
    }

    if ((K_i->size[1] == 1) || (unusedU2_size[0] == 1)) {
      i16 = x_apo->size[0];
      x_apo->size[0] = K_i->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i16, (int)sizeof(double));
      ib = K_i->size[0];
      for (i16 = 0; i16 < ib; i16++) {
        x_apo->data[i16] = 0.0;
        idx = K_i->size[1];
        for (i17 = 0; i17 < idx; i17++) {
          x_apo->data[i16] += K_i->data[i16 + K_i->size[0] * i17] * c_xt[i17];
        }
      }
    } else {
      k = K_i->size[1];
      a[0] = K_i->size[0];
      m = K_i->size[0];
      i16 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i16, (int)sizeof(double));
      ib = (int)a[0];
      for (i16 = 0; i16 < ib; i16++) {
        x_apo->data[i16] = 0.0;
      }

      if (K_i->size[0] == 0) {
      } else {
        idx = 0;
        while ((m > 0) && (idx <= 0)) {
          for (ic = 1; ic <= m; ic++) {
            x_apo->data[ic - 1] = 0.0;
          }

          idx = m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= 0)) {
          ar = 0;
          i16 = br + k;
          for (ib = br; ib + 1 <= i16; ib++) {
            if (c_xt[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                x_apo->data[ic] += c_xt[ib] * K_i->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx = m;
        }
      }
    }

    i16 = x_apo_prev->size[0];
    x_apo_prev->size[0] = b_xt->size[0];
    emxEnsureCapacity((emxArray__common *)x_apo_prev, i16, (int)sizeof(double));
    ib = b_xt->size[0];
    for (i16 = 0; i16 < ib; i16++) {
      x_apo_prev->data[i16] = b_xt->data[i16];
    }

    for (i16 = 0; i16 < 3; i16++) {
      x_apo_prev->data[i16] = b_xt->data[i16] + x_apo->data[i16];
    }

    for (i16 = 0; i16 < 3; i16++) {
      d_xt[i16] = x_apo->data[3 + i16];
    }

    quatPlusThetaJ(d_xt, dv26);
    quatmultJ(dv26, *(double (*)[4])&x_apo_prev->data[3], dv27);
    for (i16 = 0; i16 < 4; i16++) {
      x_apo_prev->data[3 + i16] = dv27[i16];
    }

    if (8.0 > c_numStatesxt) {
      i16 = 1;
      i17 = 0;
    } else {
      i16 = 8;
      i17 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      i18 = 1;
      ar = 0;
    } else {
      i18 = 7;
      ar = (int)c_numStates;
    }

    if (8.0 > c_numStatesxt) {
      idx = 0;
      br = 0;
    } else {
      idx = 7;
      br = (int)c_numStatesxt;
    }

    ii = r11->size[0] * r11->size[1];
    r11->size[0] = 1;
    r11->size[1] = br - idx;
    emxEnsureCapacity((emxArray__common *)r11, ii, (int)sizeof(int));
    ib = br - idx;
    for (br = 0; br < ib; br++) {
      r11->data[r11->size[0] * br] = idx + br;
    }

    idx = r13->size[0];
    r13->size[0] = (i17 - i16) + 1;
    emxEnsureCapacity((emxArray__common *)r13, idx, (int)sizeof(int));
    ib = i17 - i16;
    for (i17 = 0; i17 <= ib; i17++) {
      r13->data[i17] = i16 + i17;
    }

    i16 = r14->size[0];
    r14->size[0] = (ar - i18) + 1;
    emxEnsureCapacity((emxArray__common *)r14, i16, (int)sizeof(int));
    ib = ar - i18;
    for (i16 = 0; i16 <= ib; i16++) {
      r14->data[i16] = i18 + i16;
    }

    i16 = b_x_apo_prev->size[0];
    b_x_apo_prev->size[0] = r11->size[0] * r11->size[1];
    emxEnsureCapacity((emxArray__common *)b_x_apo_prev, i16, (int)sizeof(double));
    ib = r11->size[0] * r11->size[1];
    for (i16 = 0; i16 < ib; i16++) {
      b_x_apo_prev->data[i16] = x_apo_prev->data[r13->data[i16] - 1] +
        x_apo->data[r14->data[i16] - 1];
    }

    ib = b_x_apo_prev->size[0];
    for (i16 = 0; i16 < ib; i16++) {
      x_apo_prev->data[r11->data[i16]] = b_x_apo_prev->data[i16];
    }

    for (br = 0; br < numAnchors; br++) {
      for (i16 = 0; i16 < 16; i16++) {
        x[i16] = (b_anchorFeatures->data[i16 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i16 = 0; i16 < 3; i16++) {
          d_xt[i16] = x_apo_prev->data[(int)(d_numStatesxt + (1.0 + (double)i16))
            - 1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i16)) - 1];
        }

        for (i16 = 0; i16 < 3; i16++) {
          x_apo_prev->data[(int)(e_numStatesxt + (1.0 + (double)i16)) - 1] =
            d_xt[i16];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i16 = 0; i16 < 3; i16++) {
          d_xt[i16] = x_apo->data[(int)(d_numStates + (4.0 + (double)i16)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i16 = 0; i16 < 4; i16++) {
          c_x_apo_prev[i16] = x_apo_prev->data[(int)(e_numStatesxt + (4.0 +
            (double)i16)) - 1];
        }

        quatPlusThetaJ(d_xt, dv28);
        quatmultJ(dv28, c_x_apo_prev, dv27);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i16 = 0; i16 < 4; i16++) {
          x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i16)) - 1] =
            dv27[i16];
        }

        for (ii = 0; ii < (int)numPointsPerAnchor; ii++) {
          x_apo_prev->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) + (1.0 + (double)ii)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ii)) - 1];
        }
      }
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    R_cw[0] = ((x_apo_prev->data[3] * x_apo_prev->data[3] - x_apo_prev->data[4] *
                x_apo_prev->data[4]) - x_apo_prev->data[5] * x_apo_prev->data[5])
      + x_apo_prev->data[6] * x_apo_prev->data[6];
    R_cw[3] = 2.0 * (x_apo_prev->data[3] * x_apo_prev->data[4] +
                     x_apo_prev->data[5] * x_apo_prev->data[6]);
    R_cw[6] = 2.0 * (x_apo_prev->data[3] * x_apo_prev->data[5] -
                     x_apo_prev->data[4] * x_apo_prev->data[6]);
    R_cw[1] = 2.0 * (x_apo_prev->data[3] * x_apo_prev->data[4] -
                     x_apo_prev->data[5] * x_apo_prev->data[6]);
    R_cw[4] = ((-(x_apo_prev->data[3] * x_apo_prev->data[3]) + x_apo_prev->data
                [4] * x_apo_prev->data[4]) - x_apo_prev->data[5] *
               x_apo_prev->data[5]) + x_apo_prev->data[6] * x_apo_prev->data[6];
    R_cw[7] = 2.0 * (x_apo_prev->data[4] * x_apo_prev->data[5] +
                     x_apo_prev->data[3] * x_apo_prev->data[6]);
    R_cw[2] = 2.0 * (x_apo_prev->data[3] * x_apo_prev->data[5] +
                     x_apo_prev->data[4] * x_apo_prev->data[6]);
    R_cw[5] = 2.0 * (x_apo_prev->data[4] * x_apo_prev->data[5] -
                     x_apo_prev->data[3] * x_apo_prev->data[6]);
    R_cw[8] = ((-(x_apo_prev->data[3] * x_apo_prev->data[3]) - x_apo_prev->data
                [4] * x_apo_prev->data[4]) + x_apo_prev->data[5] *
               x_apo_prev->data[5]) + x_apo_prev->data[6] * x_apo_prev->data[6];
    for (i16 = 0; i16 < numMeas; i16++) {
      HI_inlierStatus_data[i16] = false;
    }

    //  inliers of this iteration
    indMeasIdx = 0;
    for (br = 0; br < numAnchors; br++) {
      featureAnchorIdx = 1.0;
      idx = 0;
      ii = 1;
      exitg2 = false;
      while ((!exitg2) && (ii < 17)) {
        guard1 = false;
        if (b_anchorFeatures->data[(ii + b_anchorFeatures->size[0] * br) - 1] !=
            0.0) {
          idx++;
          ii_data[idx - 1] = ii;
          if (idx >= 16) {
            exitg2 = true;
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          ii++;
        }
      }

      if (1 > idx) {
        ar = 0;
      } else {
        ar = idx;
      }

      if (1 > idx) {
        ib = 0;
      } else {
        ib = idx;
      }

      for (i16 = 0; i16 < ib; i16++) {
        featureIdxVect_data[i16] = (signed char)ii_data[i16];
      }

      for (idx = 0; idx < ar; idx++) {
        if (b_anchorFeatures->data[(featureIdxVect_data[idx] +
             b_anchorFeatures->size[0] * br) - 1] == 1.0) {
          //  if this is not a lost feature
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i16 = 0; i16 < 3; i16++) {
            anchorPos[i16] = x_apo_prev->data[(int)(e_numStatesxt + (1.0 +
              (double)i16)) - 1];
          }

          //  if ~all(size(q) == [4, 1])
          //      error('q does not have the size of a quaternion')
          //  end
          //  if abs(norm(q) - 1) > 1e-3
          //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
          //  end
          d_x_apo_prev[0] = ((x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] -
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1]) -
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1]) +
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
               numPointsPerAnchor)) + 7.0) - 1];
          d_x_apo_prev[1] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 5.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 6.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          d_x_apo_prev[2] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 5.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          d_x_apo_prev[3] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 5.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 6.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          d_x_apo_prev[4] = ((-(x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                                x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]) +
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1]) -
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1]) +
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
               numPointsPerAnchor)) + 7.0) - 1];
          d_x_apo_prev[5] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 4.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          d_x_apo_prev[6] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 5.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          d_x_apo_prev[7] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 4.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          d_x_apo_prev[8] = ((-(x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                                x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]) -
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1]) +
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1]) +
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
               numPointsPerAnchor)) + 7.0) - 1];
          d_numStates = x_apo_prev->data[(int)(((c_numStatesxt + ((1.0 + (double)
            br) - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx)
            - 1];
          for (i16 = 0; i16 < 3; i16++) {
            d_numStatesxt = 0.0;
            for (i17 = 0; i17 < 3; i17++) {
              d_numStatesxt += d_x_apo_prev[i16 + 3 * i17] * b_m_vect->data[i17
                + b_m_vect->size[0] * (featureIdxVect_data[idx] - 1)];
            }

            b_anchorPos[i16] = (anchorPos[i16] + d_numStatesxt / d_numStates) -
              x_apo_prev->data[iv9[i16]];
          }

          for (i16 = 0; i16 < 3; i16++) {
            d_xt[i16] = 0.0;
            for (i17 = 0; i17 < 3; i17++) {
              d_xt[i16] += R_cw[i16 + 3 * i17] * b_anchorPos[i17];
            }
          }

          predictMeasurement_left(d_xt, c_cameraparams_CameraParameters,
            d_cameraparams_CameraParameters, e_cameraparams_CameraParameters, r);
          ii = (featureIdxVect_data[idx] - 1) * 2;
          for (i16 = 0; i16 < 2; i16++) {
            b_z_all_l[i16] = z_all_l[(ii + iv10[i16]) - 1] - r[i16];
          }

          HI_inlierStatus_data[indMeasIdx] = (c_norm(b_z_all_l) < 4.0);
          indMeasIdx++;
        }

        featureAnchorIdx++;
      }
    }

    idx = 0;
    for (k = 0; k < numMeas; k++) {
      if (HI_inlierStatus_data[k]) {
        idx++;
      }
    }

    ii = 0;
    for (k = 0; k < LI_inlierStatus_size_idx_0; k++) {
      if (LI_inlierStatus_data[k]) {
        ii++;
      }
    }

    if (idx > ii) {
      for (i16 = 0; i16 < numMeas; i16++) {
        LI_inlierStatus_data[i16] = HI_inlierStatus_data[i16];
      }

      idx = 0;
      for (k = 0; k < numMeas; k++) {
        if (HI_inlierStatus_data[k]) {
          idx++;
        }
      }

      n_hyp = -4.60517018598809 / log(1.0 - (double)idx / (double)numMeas);
    }

    i++;
  }

  emxFree_real_T(&b_x_apo_prev);
  emxFree_int32_T(&r14);
  emxFree_int32_T(&r13);
  emxFree_real_T(&r12);
  emxFree_real_T(&H_i);
  emxFree_real_T(&K_i);
  idx = 0;
  for (k = 0; k < LI_inlierStatus_size_idx_0; k++) {
    if (LI_inlierStatus_data[k]) {
      idx++;
    }
  }

  emxInit_real_T(&b_a, 2);
  if (idx > 3) {
    //      fprintf('Ended hypothesis test after %i iterations. %i of %i features are LI inliers\n', i, nnz(LI_inlierStatus), numMeas) 
    // % C Partial EKF update using low-innovation inliers
    i16 = (int)maxEKFIterations;
    iter = 0;
    b_emxInit_real_T(&e_xt, 1);
    while (iter <= i16 - 1) {
      ii = LI_inlierStatus_size_idx_0 - 1;
      br = 0;
      for (idx = 0; idx <= ii; idx++) {
        if (LI_inlierStatus_data[idx]) {
          br++;
        }
      }

      ar = 0;
      for (idx = 0; idx <= ii; idx++) {
        if (LI_inlierStatus_data[idx]) {
          ii_data[ar] = idx + 1;
          ar++;
        }
      }

      indMeas_size[0] = br;
      for (i17 = 0; i17 < br; i17++) {
        b_indMeas_data[i17] = indMeas_data[ii_data[i17] - 1];
      }

      b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                   indMeas_size, map, c_cameraparams_CameraParameters,
                   d_cameraparams_CameraParameters,
                   e_cameraparams_CameraParameters, (double)numAnchors,
                   numPointsPerAnchor, anchorInd, featureAnchorInd, b_m_vect,
                   imNoise, IMU_measurements, height_offset_pressure, r_data,
                   r_size, H, unusedU2_data, unusedU2_size, b_R_data, b_R_size);
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i17 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        ib = H->size[0];
        for (i17 = 0; i17 < ib; i17++) {
          idx = b_P->size[1];
          for (i18 = 0; i18 < idx; i18++) {
            y->data[i17 + y->size[0] * i18] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i17 + y->size[0] * i18] += H->data[i17 + H->size[0] * ar] *
                b_P->data[ar + b_P->size[0] * i18];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i17 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        i17 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i17 = 0; i17 < ib; i17++) {
          y->data[i17] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i17 = idx + m;
            for (ic = idx; ic + 1 <= i17; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i17 = br + k;
            for (ib = br; ib + 1 <= i17; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i18 = idx + m;
                for (ic = idx; ic + 1 <= i18; ic++) {
                  ia++;
                  y->data[ic] += b_P->data[ib] * H->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      i17 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
      ib = H->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = H->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          b->data[i18 + b->size[0] * i17] = H->data[i17 + H->size[0] * i18];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i17 = C->size[0] * C->size[1];
        C->size[0] = y->size[0];
        C->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
        ib = y->size[0];
        for (i17 = 0; i17 < ib; i17++) {
          idx = b->size[1];
          for (i18 = 0; i18 < idx; i18++) {
            C->data[i17 + C->size[0] * i18] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              C->data[i17 + C->size[0] * i18] += y->data[i17 + y->size[0] * ar] *
                b->data[ar + b->size[0] * i18];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i17 = C->size[0] * C->size[1];
        C->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
        i17 = C->size[0] * C->size[1];
        C->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i17 = 0; i17 < ib; i17++) {
          C->data[i17] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i17 = idx + m;
            for (ic = idx; ic + 1 <= i17; ic++) {
              C->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i17 = br + k;
            for (ib = br; ib + 1 <= i17; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i18 = idx + m;
                for (ic = idx; ic + 1 <= i18; ic++) {
                  ia++;
                  C->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      i17 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
      ib = H->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = H->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          b->data[i18 + b->size[0] * i17] = H->data[i17 + H->size[0] * i18];
        }
      }

      if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
        i17 = y->size[0] * y->size[1];
        y->size[0] = b_P->size[0];
        y->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        ib = b_P->size[0];
        for (i17 = 0; i17 < ib; i17++) {
          idx = b->size[1];
          for (i18 = 0; i18 < idx; i18++) {
            y->data[i17 + y->size[0] * i18] = 0.0;
            ii = b_P->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i17 + y->size[0] * i18] += b_P->data[i17 + b_P->size[0] *
                ar] * b->data[ar + b->size[0] * i18];
            }
          }
        }
      } else {
        k = b_P->size[1];
        a[0] = b_P->size[0];
        a[1] = b->size[1];
        m = b_P->size[0];
        i17 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        i17 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i17 = 0; i17 < ib; i17++) {
          y->data[i17] = 0.0;
        }

        if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = b_P->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i17 = idx + m;
            for (ic = idx; ic + 1 <= i17; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i17 = br + k;
            for (ib = br; ib + 1 <= i17; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i18 = idx + m;
                for (ic = idx; ic + 1 <= i18; ic++) {
                  ia++;
                  y->data[ic] += b->data[ib] * b_P->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      b_C_size[0] = C->size[0];
      b_C_size[1] = C->size[1];
      ib = C->size[0] * C->size[1];
      for (i17 = 0; i17 < ib; i17++) {
        b_C_data[i17] = C->data[i17] + b_R_data[i17];
      }

      mrdivide(y, b_C_data, b_C_size, K);
      if ((K->size[1] == 1) || (r_size[0] == 1)) {
        i17 = x_apo->size[0];
        x_apo->size[0] = K->size[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i17, (int)sizeof(double));
        ib = K->size[0];
        for (i17 = 0; i17 < ib; i17++) {
          x_apo->data[i17] = 0.0;
          idx = K->size[1];
          for (i18 = 0; i18 < idx; i18++) {
            x_apo->data[i17] += K->data[i17 + K->size[0] * i18] * r_data[i18];
          }
        }
      } else {
        k = K->size[1];
        a[0] = K->size[0];
        m = K->size[0];
        i17 = x_apo->size[0];
        x_apo->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i17, (int)sizeof(double));
        ib = (int)a[0];
        for (i17 = 0; i17 < ib; i17++) {
          x_apo->data[i17] = 0.0;
        }

        if (K->size[0] == 0) {
        } else {
          idx = 0;
          while ((m > 0) && (idx <= 0)) {
            for (ic = 1; ic <= m; ic++) {
              x_apo->data[ic - 1] = 0.0;
            }

            idx = m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= 0)) {
            ar = 0;
            i17 = br + k;
            for (ib = br; ib + 1 <= i17; ib++) {
              if (r_data[ib] != 0.0) {
                ia = ar;
                for (ic = 0; ic + 1 <= m; ic++) {
                  ia++;
                  x_apo->data[ic] += r_data[ib] * K->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx = m;
          }
        }
      }

      for (i17 = 0; i17 < 3; i17++) {
        d_xt[i17] = b_xt->data[i17] + x_apo->data[i17];
      }

      for (i17 = 0; i17 < 3; i17++) {
        b_xt->data[i17] = d_xt[i17];
      }

      for (i17 = 0; i17 < 3; i17++) {
        d_xt[i17] = x_apo->data[3 + i17];
      }

      quatPlusThetaJ(d_xt, dv29);
      quatmultJ(dv29, *(double (*)[4])&b_xt->data[3], dv27);
      for (i17 = 0; i17 < 4; i17++) {
        b_xt->data[3 + i17] = dv27[i17];
      }

      if (8.0 > c_numStatesxt) {
        i17 = 0;
        i18 = 0;
      } else {
        i17 = 7;
        i18 = (int)c_numStatesxt;
      }

      if (7.0 > c_numStates) {
        ar = 0;
      } else {
        ar = 6;
      }

      if (8.0 > c_numStatesxt) {
        idx = 0;
        br = 0;
      } else {
        idx = 7;
        br = (int)c_numStatesxt;
      }

      ii = r11->size[0] * r11->size[1];
      r11->size[0] = 1;
      r11->size[1] = br - idx;
      emxEnsureCapacity((emxArray__common *)r11, ii, (int)sizeof(int));
      ib = br - idx;
      for (br = 0; br < ib; br++) {
        r11->data[r11->size[0] * br] = idx + br;
      }

      idx = e_xt->size[0];
      e_xt->size[0] = i18 - i17;
      emxEnsureCapacity((emxArray__common *)e_xt, idx, (int)sizeof(double));
      ib = i18 - i17;
      for (i18 = 0; i18 < ib; i18++) {
        e_xt->data[i18] = b_xt->data[i17 + i18] + x_apo->data[ar + i18];
      }

      ib = r11->size[1];
      for (i17 = 0; i17 < ib; i17++) {
        b_xt->data[r11->data[r11->size[0] * i17]] = e_xt->data[(*(int (*)[2])
          r11->size)[0] * i17];
      }

      for (br = 0; br < numAnchors; br++) {
        for (i17 = 0; i17 < 16; i17++) {
          x[i17] = (b_anchorFeatures->data[i17 + b_anchorFeatures->size[0] * br]
                    == 1.0);
        }

        if (any(x)) {
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
            numPointsPerAnchor);
          for (i17 = 0; i17 < 3; i17++) {
            d_xt[i17] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i17)) -
              1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i17)) - 1];
          }

          for (i17 = 0; i17 < 3; i17++) {
            b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i17)) - 1] =
              d_xt[i17];
          }

          d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
            numPointsPerAnchor);
          for (i17 = 0; i17 < 3; i17++) {
            d_xt[i17] = x_apo->data[(int)(d_numStates + (4.0 + (double)i17)) - 1];
          }

          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i17 = 0; i17 < 4; i17++) {
            c_xt[i17] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i17)) -
              1];
          }

          quatPlusThetaJ(d_xt, dv30);
          quatmultJ(dv30, c_xt, dv27);
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i17 = 0; i17 < 4; i17++) {
            b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i17)) - 1] =
              dv27[i17];
          }

          for (ii = 0; ii < (int)numPointsPerAnchor; ii++) {
            b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0
              + numPointsPerAnchor)) + 7.0) + (1.0 + (double)ii)) - 1] +=
              x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) *
                                  (6.0 + numPointsPerAnchor)) + 6.0) + (1.0 +
              (double)ii)) - 1];
          }
        }
      }

      iter++;
    }

    emxFree_real_T(&e_xt);
    eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
         numPointsPerAnchor), b_a);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i16 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
      ib = K->size[0];
      for (i16 = 0; i16 < ib; i16++) {
        idx = H->size[1];
        for (i17 = 0; i17 < idx; i17++) {
          C->data[i16 + C->size[0] * i17] = 0.0;
          ii = K->size[1];
          for (i18 = 0; i18 < ii; i18++) {
            C->data[i16 + C->size[0] * i17] += K->data[i16 + K->size[0] * i18] *
              H->data[i18 + H->size[0] * i17];
          }
        }
      }
    } else {
      k = K->size[1];
      a[0] = (unsigned int)K->size[0];
      a[1] = (unsigned int)H->size[1];
      m = K->size[0];
      i16 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
      i16 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i16 = 0; i16 < ib; i16++) {
        C->data[i16] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i16 = idx + m;
          for (ic = idx; ic + 1 <= i16; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i16 = br + k;
          for (ib = br; ib + 1 <= i16; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i17 = idx + m;
              for (ic = idx; ic + 1 <= i17; ic++) {
                ia++;
                C->data[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    i16 = b_a->size[0] * b_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_a, i16, (int)sizeof(double));
    idx = b_a->size[0];
    ii = b_a->size[1];
    ib = idx * ii;
    for (i16 = 0; i16 < ib; i16++) {
      b_a->data[i16] -= C->data[i16];
    }

    i16 = b->size[0] * b->size[1];
    b->size[0] = b_P->size[0];
    b->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)b, i16, (int)sizeof(double));
    ib = b_P->size[0] * b_P->size[1];
    for (i16 = 0; i16 < ib; i16++) {
      b->data[i16] = b_P->data[i16];
    }

    emxInit_real_T(&c_a, 2);
    if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
      i16 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = b_a->size[0];
      c_a->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)c_a, i16, (int)sizeof(double));
      ib = b_a->size[0];
      for (i16 = 0; i16 < ib; i16++) {
        idx = b_P->size[1];
        for (i17 = 0; i17 < idx; i17++) {
          c_a->data[i16 + c_a->size[0] * i17] = 0.0;
          ii = b_a->size[1];
          for (i18 = 0; i18 < ii; i18++) {
            c_a->data[i16 + c_a->size[0] * i17] += b_a->data[i16 + b_a->size[0] *
              i18] * b_P->data[i18 + b_P->size[0] * i17];
          }
        }
      }

      i16 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = c_a->size[0];
      b_P->size[1] = c_a->size[1];
      emxEnsureCapacity((emxArray__common *)b_P, i16, (int)sizeof(double));
      ib = c_a->size[1];
      for (i16 = 0; i16 < ib; i16++) {
        idx = c_a->size[0];
        for (i17 = 0; i17 < idx; i17++) {
          b_P->data[i17 + b_P->size[0] * i16] = c_a->data[i17 + c_a->size[0] *
            i16];
        }
      }
    } else {
      k = b_a->size[1];
      a[0] = b_a->size[0];
      a[1] = b_P->size[1];
      m = b_a->size[0];
      i16 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = (int)a[0];
      b_P->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)b_P, i16, (int)sizeof(double));
      ib = (int)a[1];
      for (i16 = 0; i16 < ib; i16++) {
        idx = (int)a[0];
        for (i17 = 0; i17 < idx; i17++) {
          b_P->data[i17 + b_P->size[0] * i16] = 0.0;
        }
      }

      if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_a->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i16 = idx + m;
          for (ic = idx; ic + 1 <= i16; ic++) {
            b_P->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i16 = br + k;
          for (ib = br; ib + 1 <= i16; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i17 = idx + m;
              for (ic = idx; ic + 1 <= i17; ic++) {
                ia++;
                b_P->data[ic] += b->data[ib] * b_a->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    emxFree_real_T(&c_a);
  } else {
    idx = 0;
    for (k = 0; k < LI_inlierStatus_size_idx_0; k++) {
      if (LI_inlierStatus_data[k]) {
        idx++;
      }
    }

    // #coder
    // ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
    //      coder.cinclude('<ros/console.h>')
    //      coder.ceval('ROS_WARN', [str, 0], varargin{:});
    d_fprintf(idx);
  }

  // % D Partial EKF update using high-innovation inliers
  for (i16 = 0; i16 < loop_ub; i16++) {
    HI_inlierStatus_data[i16] = true;
  }

  //  high innovation inliers
  i16 = x_apo_prev->size[0];
  x_apo_prev->size[0] = b_P->size[0];
  emxEnsureCapacity((emxArray__common *)x_apo_prev, i16, (int)sizeof(double));
  ib = b_P->size[0];
  for (i16 = 0; i16 < ib; i16++) {
    x_apo_prev->data[i16] = 0.0;
  }

  i16 = (int)maxEKFIterations;
  it = 0;
  b_emxInit_real_T(&b_x_apo, 1);
  b_emxInit_real_T(&f_xt, 1);
  exitg1 = false;
  while ((!exitg1) && (it <= i16 - 1)) {
    ii = loop_ub - 1;
    br = 0;
    for (idx = 0; idx <= ii; idx++) {
      if (HI_inlierStatus_data[idx]) {
        br++;
      }
    }

    ar = 0;
    for (idx = 0; idx <= ii; idx++) {
      if (HI_inlierStatus_data[idx]) {
        ii_data[ar] = idx + 1;
        ar++;
      }
    }

    b_indMeas_size[0] = br;
    for (i17 = 0; i17 < br; i17++) {
      b_indMeas_data[i17] = indMeas_data[ii_data[i17] - 1];
    }

    b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                 b_indMeas_size, map, c_cameraparams_CameraParameters,
                 d_cameraparams_CameraParameters,
                 e_cameraparams_CameraParameters, (double)numAnchors,
                 numPointsPerAnchor, anchorInd, featureAnchorInd, b_m_vect,
                 imNoise, IMU_measurements, height_offset_pressure, r_data,
                 r_size, H, unusedU2_data, unusedU2_size, b_R_data, b_R_size);
    if (1.0 + (double)it == 1.0) {
      //  only do outlier rejection in first iteration
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i17 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        ib = H->size[0];
        for (i17 = 0; i17 < ib; i17++) {
          idx = b_P->size[1];
          for (i18 = 0; i18 < idx; i18++) {
            y->data[i17 + y->size[0] * i18] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i17 + y->size[0] * i18] += H->data[i17 + H->size[0] * ar] *
                b_P->data[ar + b_P->size[0] * i18];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i17 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        i17 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i17 = 0; i17 < ib; i17++) {
          y->data[i17] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i17 = idx + m;
            for (ic = idx; ic + 1 <= i17; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i17 = br + k;
            for (ib = br; ib + 1 <= i17; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i18 = idx + m;
                for (ic = idx; ic + 1 <= i18; ic++) {
                  ia++;
                  y->data[ic] += b_P->data[ib] * H->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      i17 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
      ib = H->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = H->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          b->data[i18 + b->size[0] * i17] = H->data[i17 + H->size[0] * i18];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i17 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = y->size[0];
        b_a->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_a, i17, (int)sizeof(double));
        ib = y->size[0];
        for (i17 = 0; i17 < ib; i17++) {
          idx = b->size[1];
          for (i18 = 0; i18 < idx; i18++) {
            b_a->data[i17 + b_a->size[0] * i18] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              b_a->data[i17 + b_a->size[0] * i18] += y->data[i17 + y->size[0] *
                ar] * b->data[ar + b->size[0] * i18];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i17 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)b_a, i17, (int)sizeof(double));
        i17 = b_a->size[0] * b_a->size[1];
        b_a->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)b_a, i17, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i17 = 0; i17 < ib; i17++) {
          b_a->data[i17] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i17 = idx + m;
            for (ic = idx; ic + 1 <= i17; ic++) {
              b_a->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i17 = br + k;
            for (ib = br; ib + 1 <= i17; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i18 = idx + m;
                for (ic = idx; ic + 1 <= i18; ic++) {
                  ia++;
                  b_a->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      br = b_a->size[0];
      ib = b_a->size[0] * b_a->size[1];
      for (i17 = 0; i17 < ib; i17++) {
        S_data[i17] = b_a->data[i17] + b_R_data[i17];
      }

      for (k = 0; k < numMeas; k++) {
        ar = k << 1;
        ii = k << 1;
        idx = k << 1;
        for (i17 = 0; i17 < 2; i17++) {
          r[i17] = r_data[i17 + ar];
          for (i18 = 0; i18 < 2; i18++) {
            c_xt[i18 + (i17 << 1)] = S_data[(i18 + ii) + br * (i17 + idx)];
          }
        }

        b_mrdivide(r, c_xt, a);
        ar = k << 1;
        for (i17 = 0; i17 < 2; i17++) {
          r[i17] = r_data[i17 + ar];
        }

        if (LI_inlierStatus_data[k]) {
          //  if this feature is a LI inlier, don't also do HI update with this feature 
          ar = k << 1;
          for (i17 = 0; i17 < 2; i17++) {
            r_data[i17 + ar] = 0.0;
          }

          ib = H->size[1];
          ar = k << 1;
          for (i17 = 0; i17 < ib; i17++) {
            for (i18 = 0; i18 < 2; i18++) {
              H->data[(i18 + ar) + H->size[0] * i17] = 0.0;
            }
          }

          HI_inlierStatus_data[k] = false;
        } else {
          //  otherwise check if HI inlier for both cams
          d_numStatesxt = 0.0;
          for (i17 = 0; i17 < 2; i17++) {
            d_numStatesxt += a[i17] * r[i17];
          }

          if (d_numStatesxt > 6.0) {
            ar = k << 1;
            for (i17 = 0; i17 < 2; i17++) {
              r_data[i17 + ar] = 0.0;
            }

            ib = H->size[1];
            ar = k << 1;
            for (i17 = 0; i17 < ib; i17++) {
              for (i18 = 0; i18 < 2; i18++) {
                H->data[(i18 + ar) + H->size[0] * i17] = 0.0;
              }
            }

            HI_inlierStatus_data[k] = false;
          }
        }
      }
    }

    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i17 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
      ib = H->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = b_P->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          y->data[i17 + y->size[0] * i18] = 0.0;
          ii = H->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i17 + y->size[0] * i18] += H->data[i17 + H->size[0] * ar] *
              b_P->data[ar + b_P->size[0] * i18];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      m = H->size[0];
      i17 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
      i17 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i17 = 0; i17 < ib; i17++) {
        y->data[i17] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i17 = idx + m;
          for (ic = idx; ic + 1 <= i17; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i17 = br + k;
          for (ib = br; ib + 1 <= i17; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i18 = idx + m;
              for (ic = idx; ic + 1 <= i18; ic++) {
                ia++;
                y->data[ic] += b_P->data[ib] * H->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    i17 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
    ib = H->size[0];
    for (i17 = 0; i17 < ib; i17++) {
      idx = H->size[1];
      for (i18 = 0; i18 < idx; i18++) {
        b->data[i18 + b->size[0] * i17] = H->data[i17 + H->size[0] * i18];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i17 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
      ib = y->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = b->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          C->data[i17 + C->size[0] * i18] = 0.0;
          ii = y->size[1];
          for (ar = 0; ar < ii; ar++) {
            C->data[i17 + C->size[0] * i18] += y->data[i17 + y->size[0] * ar] *
              b->data[ar + b->size[0] * i18];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i17 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
      i17 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i17 = 0; i17 < ib; i17++) {
        C->data[i17] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i17 = idx + m;
          for (ic = idx; ic + 1 <= i17; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i17 = br + k;
          for (ib = br; ib + 1 <= i17; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i18 = idx + m;
              for (ic = idx; ic + 1 <= i18; ic++) {
                ia++;
                C->data[ic] += b->data[ib] * y->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    i17 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
    ib = H->size[0];
    for (i17 = 0; i17 < ib; i17++) {
      idx = H->size[1];
      for (i18 = 0; i18 < idx; i18++) {
        b->data[i18 + b->size[0] * i17] = H->data[i17 + H->size[0] * i18];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i17 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
      ib = b_P->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = b->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          y->data[i17 + y->size[0] * i18] = 0.0;
          ii = b_P->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i17 + y->size[0] * i18] += b_P->data[i17 + b_P->size[0] * ar]
              * b->data[ar + b->size[0] * i18];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i17 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
      i17 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i17 = 0; i17 < ib; i17++) {
        y->data[i17] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i17 = idx + m;
          for (ic = idx; ic + 1 <= i17; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i17 = br + k;
          for (ib = br; ib + 1 <= i17; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i18 = idx + m;
              for (ic = idx; ic + 1 <= i18; ic++) {
                ia++;
                y->data[ic] += b->data[ib] * b_P->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    c_C_size[0] = C->size[0];
    c_C_size[1] = C->size[1];
    ib = C->size[0] * C->size[1];
    for (i17 = 0; i17 < ib; i17++) {
      b_C_data[i17] = C->data[i17] + b_R_data[i17];
    }

    mrdivide(y, b_C_data, c_C_size, K);
    if ((K->size[1] == 1) || (r_size[0] == 1)) {
      i17 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i17, (int)sizeof(double));
      ib = K->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        x_apo->data[i17] = 0.0;
        idx = K->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          x_apo->data[i17] += K->data[i17 + K->size[0] * i18] * r_data[i18];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      m = K->size[0];
      i17 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i17, (int)sizeof(double));
      ib = (int)a[0];
      for (i17 = 0; i17 < ib; i17++) {
        x_apo->data[i17] = 0.0;
      }

      if (K->size[0] == 0) {
      } else {
        idx = 0;
        while ((m > 0) && (idx <= 0)) {
          for (ic = 1; ic <= m; ic++) {
            x_apo->data[ic - 1] = 0.0;
          }

          idx = m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= 0)) {
          ar = 0;
          i17 = br + k;
          for (ib = br; ib + 1 <= i17; ib++) {
            if (r_data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                x_apo->data[ic] += r_data[ib] * K->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx = m;
        }
      }
    }

    for (i17 = 0; i17 < 3; i17++) {
      d_xt[i17] = b_xt->data[i17] + x_apo->data[i17];
    }

    for (i17 = 0; i17 < 3; i17++) {
      b_xt->data[i17] = d_xt[i17];
    }

    for (i17 = 0; i17 < 3; i17++) {
      d_xt[i17] = x_apo->data[3 + i17];
    }

    quatPlusThetaJ(d_xt, dv31);
    quatmultJ(dv31, *(double (*)[4])&b_xt->data[3], dv27);
    for (i17 = 0; i17 < 4; i17++) {
      b_xt->data[3 + i17] = dv27[i17];
    }

    if (8.0 > c_numStatesxt) {
      i17 = 0;
      i18 = 0;
    } else {
      i17 = 7;
      i18 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      ar = 0;
    } else {
      ar = 6;
    }

    if (8.0 > c_numStatesxt) {
      idx = 0;
      br = 0;
    } else {
      idx = 7;
      br = (int)c_numStatesxt;
    }

    ii = r11->size[0] * r11->size[1];
    r11->size[0] = 1;
    r11->size[1] = br - idx;
    emxEnsureCapacity((emxArray__common *)r11, ii, (int)sizeof(int));
    ib = br - idx;
    for (br = 0; br < ib; br++) {
      r11->data[r11->size[0] * br] = idx + br;
    }

    idx = f_xt->size[0];
    f_xt->size[0] = i18 - i17;
    emxEnsureCapacity((emxArray__common *)f_xt, idx, (int)sizeof(double));
    ib = i18 - i17;
    for (i18 = 0; i18 < ib; i18++) {
      f_xt->data[i18] = b_xt->data[i17 + i18] + x_apo->data[ar + i18];
    }

    ib = r11->size[1];
    for (i17 = 0; i17 < ib; i17++) {
      b_xt->data[r11->data[r11->size[0] * i17]] = f_xt->data[(*(int (*)[2])
        r11->size)[0] * i17];
    }

    for (br = 0; br < numAnchors; br++) {
      for (i17 = 0; i17 < 16; i17++) {
        x[i17] = (b_anchorFeatures->data[i17 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i17 = 0; i17 < 3; i17++) {
          d_xt[i17] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i17)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i17)) - 1];
        }

        for (i17 = 0; i17 < 3; i17++) {
          b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i17)) - 1] = d_xt[i17];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i17 = 0; i17 < 3; i17++) {
          d_xt[i17] = x_apo->data[(int)(d_numStates + (4.0 + (double)i17)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i17 = 0; i17 < 4; i17++) {
          c_xt[i17] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i17)) - 1];
        }

        quatPlusThetaJ(d_xt, dv32);
        quatmultJ(dv32, c_xt, dv27);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i17 = 0; i17 < 4; i17++) {
          b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i17)) - 1] = dv27[i17];
        }

        for (ii = 0; ii < (int)numPointsPerAnchor; ii++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) + (1.0 + (double)ii)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ii)) - 1];
        }
      }
    }

    i17 = b_x_apo->size[0];
    b_x_apo->size[0] = x_apo->size[0];
    emxEnsureCapacity((emxArray__common *)b_x_apo, i17, (int)sizeof(double));
    ib = x_apo->size[0];
    for (i17 = 0; i17 < ib; i17++) {
      b_x_apo->data[i17] = x_apo->data[i17] - x_apo_prev->data[i17];
    }

    if (d_norm(b_x_apo) < 0.001) {
      exitg1 = true;
    } else {
      i17 = x_apo_prev->size[0];
      x_apo_prev->size[0] = x_apo->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo_prev, i17, (int)sizeof(double));
      ib = x_apo->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        x_apo_prev->data[i17] = x_apo->data[i17];
      }

      it++;
    }
  }

  emxFree_real_T(&f_xt);
  emxFree_real_T(&b_x_apo);
  emxFree_real_T(&y);
  emxFree_int32_T(&r11);
  emxFree_real_T(&featureAnchorInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&map);
  emxFree_real_T(&x_apo_prev);
  emxFree_real_T(&x_apo);
  eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
       numPointsPerAnchor), b_a);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i16 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
    loop_ub = K->size[0];
    for (i16 = 0; i16 < loop_ub; i16++) {
      ib = H->size[1];
      for (i17 = 0; i17 < ib; i17++) {
        C->data[i16 + C->size[0] * i17] = 0.0;
        idx = K->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          C->data[i16 + C->size[0] * i17] += K->data[i16 + K->size[0] * i18] *
            H->data[i18 + H->size[0] * i17];
        }
      }
    }
  } else {
    k = K->size[1];
    a[0] = (unsigned int)K->size[0];
    a[1] = (unsigned int)H->size[1];
    m = K->size[0];
    i16 = C->size[0] * C->size[1];
    C->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
    i16 = C->size[0] * C->size[1];
    C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)C, i16, (int)sizeof(double));
    loop_ub = (int)a[0] * (int)a[1];
    for (i16 = 0; i16 < loop_ub; i16++) {
      C->data[i16] = 0.0;
    }

    if ((K->size[0] == 0) || (H->size[1] == 0)) {
    } else {
      ii = K->size[0] * (H->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i16 = idx + m;
        for (ic = idx; ic + 1 <= i16; ic++) {
          C->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i16 = br + k;
        for (ib = br; ib + 1 <= i16; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            i17 = idx + m;
            for (ic = idx; ic + 1 <= i17; ic++) {
              ia++;
              C->data[ic] += H->data[ib] * K->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        idx += m;
      }
    }
  }

  emxFree_real_T(&H);
  emxFree_real_T(&K);
  i16 = b_a->size[0] * b_a->size[1];
  emxEnsureCapacity((emxArray__common *)b_a, i16, (int)sizeof(double));
  idx = b_a->size[0];
  ii = b_a->size[1];
  loop_ub = idx * ii;
  for (i16 = 0; i16 < loop_ub; i16++) {
    b_a->data[i16] -= C->data[i16];
  }

  emxFree_real_T(&C);
  i16 = b->size[0] * b->size[1];
  b->size[0] = b_P->size[0];
  b->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b, i16, (int)sizeof(double));
  loop_ub = b_P->size[0] * b_P->size[1];
  for (i16 = 0; i16 < loop_ub; i16++) {
    b->data[i16] = b_P->data[i16];
  }

  emxInit_real_T(&d_a, 2);
  if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
    i16 = d_a->size[0] * d_a->size[1];
    d_a->size[0] = b_a->size[0];
    d_a->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)d_a, i16, (int)sizeof(double));
    loop_ub = b_a->size[0];
    for (i16 = 0; i16 < loop_ub; i16++) {
      ib = b_P->size[1];
      for (i17 = 0; i17 < ib; i17++) {
        d_a->data[i16 + d_a->size[0] * i17] = 0.0;
        idx = b_a->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          d_a->data[i16 + d_a->size[0] * i17] += b_a->data[i16 + b_a->size[0] *
            i18] * b_P->data[i18 + b_P->size[0] * i17];
        }
      }
    }

    i16 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = d_a->size[0];
    b_P->size[1] = d_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i16, (int)sizeof(double));
    loop_ub = d_a->size[1];
    for (i16 = 0; i16 < loop_ub; i16++) {
      ib = d_a->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        b_P->data[i17 + b_P->size[0] * i16] = d_a->data[i17 + d_a->size[0] * i16];
      }
    }
  } else {
    k = b_a->size[1];
    a[0] = b_a->size[0];
    a[1] = b_P->size[1];
    m = b_a->size[0];
    i16 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = (int)a[0];
    b_P->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)b_P, i16, (int)sizeof(double));
    loop_ub = (int)a[1];
    for (i16 = 0; i16 < loop_ub; i16++) {
      ib = (int)a[0];
      for (i17 = 0; i17 < ib; i17++) {
        b_P->data[i17 + b_P->size[0] * i16] = 0.0;
      }
    }

    if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = b_a->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i16 = idx + m;
        for (ic = idx; ic + 1 <= i16; ic++) {
          b_P->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i16 = br + k;
        for (ib = br; ib + 1 <= i16; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i17 = idx + m;
            for (ic = idx; ic + 1 <= i17; ic++) {
              ia++;
              b_P->data[ic] += b->data[ib] * b_a->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        idx += m;
      }
    }
  }

  emxFree_real_T(&d_a);
  emxFree_real_T(&b);
  emxFree_real_T(&b_a);
  ii = LI_inlierStatus_size_idx_0 - 1;
  br = 0;
  for (idx = 0; idx <= ii; idx++) {
    if (LI_inlierStatus_data[idx] || HI_inlierStatus_data[idx]) {
      br++;
    }
  }

  validFeatures_size[0] = br;
  ar = 0;
  for (idx = 0; idx <= ii; idx++) {
    if (LI_inlierStatus_data[idx] || HI_inlierStatus_data[idx]) {
      validFeatures_data[ar] = indMeas_data[idx];
      ar++;
    }
  }

  //  if length(validFeatures) ~= numMeas
  //      rejected = setdiff(indMeas, validFeatures);
  //      fprintf('%i of %i features are valid after RANSAC.\n', int8(length(validFeatures)), numMeas) 
  //      fprintf(' Rejected: %s\n', mat2str(rejected))
  //  end
  br = 0;
  for (idx = 0; idx < LI_inlierStatus_size_idx_0; idx++) {
    if (LI_inlierStatus_data[idx] || HI_inlierStatus_data[idx]) {
      br++;
    }
  }

  if (br == 0) {
    // #coder
    // ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
    //      coder.cinclude('<ros/console.h>')
    //      coder.ceval('ROS_WARN', [str, 0], varargin{:});
    f_fprintf();
  }
}

//
// File trailer for OnePointRANSAC_EKF.cpp
//
// [EOF]
//
