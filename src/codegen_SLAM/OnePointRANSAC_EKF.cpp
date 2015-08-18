//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 18-Aug-2015 09:45:37
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
//                const double z_all_l[64]
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
  double z_all_l[64], double c_numStatesxt, double c_numStates, double
  numPointsPerAnchor, const double c_cameraparams_CameraParameters[3], const
  double d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const emxArray_real_T *b_anchorFeatures,
  const emxArray_real_T *b_m_vect, const double imNoise[2], const double
  IMU_measurements[23], double height_offset_pressure, double
  validFeatures_data[], int validFeatures_size[1])
{
  emxArray_boolean_T *c_anchorFeatures;
  int numAnchors;
  int i17;
  int loop_ub;
  boolean_T x[32];
  int idx;
  int ii_data[32];
  int ii;
  boolean_T exitg3;
  boolean_T guard2 = false;
  int ar;
  signed char indMeas_data[32];
  emxArray_real_T *K;
  emxArray_real_T *H;
  int numMeas;
  double n_hyp;
  int LI_inlierStatus_size_idx_0;
  boolean_T LI_inlierStatus_data[32];
  emxArray_real_T *map;
  emxArray_real_T *anchorInd;
  emxArray_real_T *featureAnchorInd;
  double i;
  emxArray_real_T *K_i;
  emxArray_real_T *x_apo;
  emxArray_real_T *x_apo_prev;
  emxArray_real_T *H_i;
  emxArray_real_T *C;
  emxArray_int32_T *r12;
  emxArray_real_T *b;
  emxArray_real_T *y;
  emxArray_real_T *r13;
  emxArray_int32_T *r14;
  emxArray_int32_T *r15;
  emxArray_real_T *b_x_apo_prev;
  double d_numStatesxt;
  int R_size[2];
  double R_data[36];
  double r[2];
  int unusedU2_size[1];
  double r_data[6];
  int ib;
  int i18;
  int i19;
  int k;
  double a[2];
  int m;
  int ic;
  int br;
  int ia;
  double C_data[36];
  int C_size[2];
  double c_xt[3];
  double dv38[4];
  double dv39[4];
  double e_numStatesxt;
  double d_numStates;
  double d_xt[4];
  double dv40[4];
  double R_cw[9];
  boolean_T HI_inlierStatus_data[32];
  int indMeasIdx;
  double featureAnchorIdx;
  boolean_T exitg2;
  boolean_T guard1 = false;
  signed char featureIdxVect_data[32];
  double anchorPos[3];
  double c_x_apo_prev[9];
  double b_anchorPos[3];
  static const signed char iv10[3] = { 0, 1, 2 };

  double b_z_all_l[2];
  static const signed char iv11[2] = { 1, 2 };

  emxArray_real_T *b_a;
  int iter;
  emxArray_real_T *e_xt;
  double b_indMeas_data[32];
  int indMeas_size[1];
  int b_R_size[2];
  double b_R_data[4624];
  double unusedU2_data[64];
  int r_size[1];
  double b_r_data[68];
  double b_C_data[4624];
  int b_C_size[2];
  double dv41[4];
  double dv42[4];
  emxArray_real_T *c_a;
  int it;
  emxArray_real_T *b_x_apo;
  emxArray_real_T *f_xt;
  boolean_T exitg1;
  int b_indMeas_size[1];
  double S_data[4624];
  int c_C_size[2];
  double dv43[4];
  double dv44[4];
  emxArray_real_T *d_a;
  emxInit_boolean_T(&c_anchorFeatures, 2);

  //  threshold for LI innovation test
  // 9.487729036781154; % HI mahalanobis gate
  // 9.487729036781154; % HI mahalanobis gate
  //  threshold for minimum LI supporter
  numAnchors = b_anchorFeatures->size[1];
  i17 = c_anchorFeatures->size[0] * c_anchorFeatures->size[1];
  c_anchorFeatures->size[0] = 32;
  c_anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)c_anchorFeatures, i17, (int)sizeof
                    (boolean_T));
  loop_ub = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i17 = 0; i17 < loop_ub; i17++) {
    c_anchorFeatures->data[i17] = (b_anchorFeatures->data[i17] == 1.0);
  }

  b_any(c_anchorFeatures, x);
  idx = 0;
  ii = 1;
  emxFree_boolean_T(&c_anchorFeatures);
  exitg3 = false;
  while ((!exitg3) && (ii < 33)) {
    guard2 = false;
    if (x[ii - 1]) {
      idx++;
      ii_data[idx - 1] = ii;
      if (idx >= 32) {
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

  for (i17 = 0; i17 < loop_ub; i17++) {
    indMeas_data[i17] = (signed char)ii_data[i17];
  }

  emxInit_real_T(&K, 2);
  emxInit_real_T(&H, 2);
  numMeas = ar;
  i17 = K->size[0] * K->size[1];
  K->size[0] = 1;
  K->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)K, i17, (int)sizeof(double));
  K->data[0] = 0.0;

  //  for coder
  i17 = H->size[0] * H->size[1];
  H->size[0] = 1;
  H->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)H, i17, (int)sizeof(double));
  H->data[0] = 0.0;

  //  for coder
  // % B 1-point hypotheses generation and evaluation
  n_hyp = 100.0;
  LI_inlierStatus_size_idx_0 = ar;
  for (i17 = 0; i17 < ar; i17++) {
    LI_inlierStatus_data[i17] = false;
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
  b_emxInit_int32_T(&r12, 2);
  emxInit_real_T(&b, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&r13, 2);
  emxInit_int32_T(&r14, 1);
  emxInit_int32_T(&r15, 1);
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
               imNoise, IMU_measurements, height_offset_pressure, r_data,
               unusedU2_size, H_i, r, R_data, R_size);
    if ((H_i->size[1] == 1) || (b_P->size[0] == 1)) {
      i17 = y->size[0] * y->size[1];
      y->size[0] = H_i->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i17, (int)sizeof(double));
      ib = H_i->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = b_P->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          y->data[i17 + y->size[0] * i18] = 0.0;
          ii = H_i->size[1];
          for (i19 = 0; i19 < ii; i19++) {
            y->data[i17 + y->size[0] * i18] += H_i->data[i17 + H_i->size[0] *
              i19] * b_P->data[i19 + b_P->size[0] * i18];
          }
        }
      }
    } else {
      k = H_i->size[1];
      a[0] = H_i->size[0];
      a[1] = b_P->size[1];
      m = H_i->size[0];
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

      if ((H_i->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H_i->size[0] * (b_P->size[1] - 1);
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

    i17 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = H_i->size[0];
    emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
    ib = H_i->size[0];
    for (i17 = 0; i17 < ib; i17++) {
      idx = H_i->size[1];
      for (i18 = 0; i18 < idx; i18++) {
        b->data[i18 + b->size[0] * i17] = H_i->data[i17 + H_i->size[0] * i18];
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
          for (i19 = 0; i19 < ii; i19++) {
            C->data[i17 + C->size[0] * i18] += y->data[i17 + y->size[0] * i19] *
              b->data[i19 + b->size[0] * i18];
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
    b->size[0] = H_i->size[1];
    b->size[1] = H_i->size[0];
    emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
    ib = H_i->size[0];
    for (i17 = 0; i17 < ib; i17++) {
      idx = H_i->size[1];
      for (i18 = 0; i18 < idx; i18++) {
        b->data[i18 + b->size[0] * i17] = H_i->data[i17 + H_i->size[0] * i18];
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
          for (i19 = 0; i19 < ii; i19++) {
            y->data[i17 + y->size[0] * i18] += b_P->data[i17 + b_P->size[0] *
              i19] * b->data[i19 + b->size[0] * i18];
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

    C_size[0] = C->size[0];
    C_size[1] = C->size[1];
    ib = C->size[0] * C->size[1];
    for (i17 = 0; i17 < ib; i17++) {
      C_data[i17] = C->data[i17] + R_data[i17];
    }

    mrdivide(y, C_data, C_size, r13);
    i17 = K_i->size[0] * K_i->size[1];
    K_i->size[0] = r13->size[0];
    K_i->size[1] = r13->size[1];
    emxEnsureCapacity((emxArray__common *)K_i, i17, (int)sizeof(double));
    ib = r13->size[0] * r13->size[1];
    for (i17 = 0; i17 < ib; i17++) {
      K_i->data[i17] = r13->data[i17];
    }

    if ((K_i->size[1] == 1) || (unusedU2_size[0] == 1)) {
      i17 = x_apo->size[0];
      x_apo->size[0] = K_i->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i17, (int)sizeof(double));
      ib = K_i->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        x_apo->data[i17] = 0.0;
        idx = K_i->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          x_apo->data[i17] += K_i->data[i17 + K_i->size[0] * i18] * r_data[i18];
        }
      }
    } else {
      k = K_i->size[1];
      a[0] = K_i->size[0];
      m = K_i->size[0];
      i17 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i17, (int)sizeof(double));
      ib = (int)a[0];
      for (i17 = 0; i17 < ib; i17++) {
        x_apo->data[i17] = 0.0;
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
          i17 = br + k;
          for (ib = br; ib + 1 <= i17; ib++) {
            if (r_data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                x_apo->data[ic] += r_data[ib] * K_i->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx = m;
        }
      }
    }

    i17 = x_apo_prev->size[0];
    x_apo_prev->size[0] = b_xt->size[0];
    emxEnsureCapacity((emxArray__common *)x_apo_prev, i17, (int)sizeof(double));
    ib = b_xt->size[0];
    for (i17 = 0; i17 < ib; i17++) {
      x_apo_prev->data[i17] = b_xt->data[i17];
    }

    for (i17 = 0; i17 < 3; i17++) {
      x_apo_prev->data[i17] = b_xt->data[i17] + x_apo->data[i17];
    }

    for (i17 = 0; i17 < 3; i17++) {
      c_xt[i17] = x_apo->data[3 + i17];
    }

    quatPlusThetaJ(c_xt, dv38);
    quatmultJ(dv38, *(double (*)[4])&x_apo_prev->data[3], dv39);
    for (i17 = 0; i17 < 4; i17++) {
      x_apo_prev->data[3 + i17] = dv39[i17];
    }

    if (8.0 > c_numStatesxt) {
      i17 = 1;
      i18 = 0;
    } else {
      i17 = 8;
      i18 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      i19 = 1;
      ar = 0;
    } else {
      i19 = 7;
      ar = (int)c_numStates;
    }

    if (8.0 > c_numStatesxt) {
      idx = 0;
      br = 0;
    } else {
      idx = 7;
      br = (int)c_numStatesxt;
    }

    ii = r12->size[0] * r12->size[1];
    r12->size[0] = 1;
    r12->size[1] = br - idx;
    emxEnsureCapacity((emxArray__common *)r12, ii, (int)sizeof(int));
    ib = br - idx;
    for (br = 0; br < ib; br++) {
      r12->data[r12->size[0] * br] = idx + br;
    }

    idx = r14->size[0];
    r14->size[0] = (i18 - i17) + 1;
    emxEnsureCapacity((emxArray__common *)r14, idx, (int)sizeof(int));
    ib = i18 - i17;
    for (i18 = 0; i18 <= ib; i18++) {
      r14->data[i18] = i17 + i18;
    }

    i17 = r15->size[0];
    r15->size[0] = (ar - i19) + 1;
    emxEnsureCapacity((emxArray__common *)r15, i17, (int)sizeof(int));
    ib = ar - i19;
    for (i17 = 0; i17 <= ib; i17++) {
      r15->data[i17] = i19 + i17;
    }

    i17 = b_x_apo_prev->size[0];
    b_x_apo_prev->size[0] = r12->size[0] * r12->size[1];
    emxEnsureCapacity((emxArray__common *)b_x_apo_prev, i17, (int)sizeof(double));
    ib = r12->size[0] * r12->size[1];
    for (i17 = 0; i17 < ib; i17++) {
      b_x_apo_prev->data[i17] = x_apo_prev->data[r14->data[i17] - 1] +
        x_apo->data[r15->data[i17] - 1];
    }

    ib = b_x_apo_prev->size[0];
    for (i17 = 0; i17 < ib; i17++) {
      x_apo_prev->data[r12->data[i17]] = b_x_apo_prev->data[i17];
    }

    for (br = 0; br < numAnchors; br++) {
      for (i17 = 0; i17 < 32; i17++) {
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
          c_xt[i17] = x_apo_prev->data[(int)(d_numStatesxt + (1.0 + (double)i17))
            - 1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i17)) - 1];
        }

        for (i17 = 0; i17 < 3; i17++) {
          x_apo_prev->data[(int)(e_numStatesxt + (1.0 + (double)i17)) - 1] =
            c_xt[i17];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i17 = 0; i17 < 3; i17++) {
          c_xt[i17] = x_apo->data[(int)(d_numStates + (4.0 + (double)i17)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i17 = 0; i17 < 4; i17++) {
          d_xt[i17] = x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i17))
            - 1];
        }

        quatPlusThetaJ(c_xt, dv40);
        quatmultJ(dv40, d_xt, dv39);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i17 = 0; i17 < 4; i17++) {
          x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i17)) - 1] =
            dv39[i17];
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
    for (i17 = 0; i17 < numMeas; i17++) {
      HI_inlierStatus_data[i17] = false;
    }

    //  inliers of this iteration
    indMeasIdx = 0;
    for (br = 0; br < numAnchors; br++) {
      featureAnchorIdx = 1.0;
      idx = 0;
      ii = 1;
      exitg2 = false;
      while ((!exitg2) && (ii < 33)) {
        guard1 = false;
        if (b_anchorFeatures->data[(ii + b_anchorFeatures->size[0] * br) - 1] !=
            0.0) {
          idx++;
          ii_data[idx - 1] = ii;
          if (idx >= 32) {
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

      for (i17 = 0; i17 < ib; i17++) {
        featureIdxVect_data[i17] = (signed char)ii_data[i17];
      }

      for (idx = 0; idx < ar; idx++) {
        if (b_anchorFeatures->data[(featureIdxVect_data[idx] +
             b_anchorFeatures->size[0] * br) - 1] == 1.0) {
          //  if this is not a lost feature
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i17 = 0; i17 < 3; i17++) {
            anchorPos[i17] = x_apo_prev->data[(int)(e_numStatesxt + (1.0 +
              (double)i17)) - 1];
          }

          //  if ~all(size(q) == [4, 1])
          //      error('q does not have the size of a quaternion')
          //  end
          //  if abs(norm(q) - 1) > 1e-3
          //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
          //  end
          c_x_apo_prev[0] = ((x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
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
          c_x_apo_prev[1] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 5.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 6.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[2] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 5.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[3] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 5.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 6.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[4] = ((-(x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
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
          c_x_apo_prev[5] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 4.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[6] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 5.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[7] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 4.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[8] = ((-(x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
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
          for (i17 = 0; i17 < 3; i17++) {
            d_numStatesxt = 0.0;
            for (i18 = 0; i18 < 3; i18++) {
              d_numStatesxt += c_x_apo_prev[i17 + 3 * i18] * b_m_vect->data[i18
                + b_m_vect->size[0] * (featureIdxVect_data[idx] - 1)];
            }

            b_anchorPos[i17] = (anchorPos[i17] + d_numStatesxt / d_numStates) -
              x_apo_prev->data[iv10[i17]];
          }

          for (i17 = 0; i17 < 3; i17++) {
            c_xt[i17] = 0.0;
            for (i18 = 0; i18 < 3; i18++) {
              c_xt[i17] += R_cw[i17 + 3 * i18] * b_anchorPos[i18];
            }
          }

          predictMeasurement_left(c_xt, c_cameraparams_CameraParameters,
            d_cameraparams_CameraParameters, e_cameraparams_CameraParameters, r);
          ii = (featureIdxVect_data[idx] - 1) * 2;
          for (i17 = 0; i17 < 2; i17++) {
            b_z_all_l[i17] = z_all_l[(ii + iv11[i17]) - 1] - r[i17];
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
      for (i17 = 0; i17 < numMeas; i17++) {
        LI_inlierStatus_data[i17] = HI_inlierStatus_data[i17];
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
  emxFree_int32_T(&r15);
  emxFree_int32_T(&r14);
  emxFree_real_T(&r13);
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
    i17 = (int)maxEKFIterations;
    iter = 0;
    b_emxInit_real_T(&e_xt, 1);
    while (iter <= i17 - 1) {
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
      for (i18 = 0; i18 < br; i18++) {
        b_indMeas_data[i18] = indMeas_data[ii_data[i18] - 1];
      }

      b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                   indMeas_size, map, c_cameraparams_CameraParameters,
                   d_cameraparams_CameraParameters,
                   e_cameraparams_CameraParameters, (double)numAnchors,
                   numPointsPerAnchor, anchorInd, featureAnchorInd, b_m_vect,
                   imNoise, IMU_measurements, height_offset_pressure, b_r_data,
                   r_size, H, unusedU2_data, unusedU2_size, b_R_data, b_R_size);
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i18 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        ib = H->size[0];
        for (i18 = 0; i18 < ib; i18++) {
          idx = b_P->size[1];
          for (i19 = 0; i19 < idx; i19++) {
            y->data[i18 + y->size[0] * i19] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i18 + y->size[0] * i19] += H->data[i18 + H->size[0] * ar] *
                b_P->data[ar + b_P->size[0] * i19];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i18 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        i18 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i18 = 0; i18 < ib; i18++) {
          y->data[i18] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i18 = idx + m;
            for (ic = idx; ic + 1 <= i18; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i18 = br + k;
            for (ib = br; ib + 1 <= i18; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i19 = idx + m;
                for (ic = idx; ic + 1 <= i19; ic++) {
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

      i18 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
      ib = H->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        idx = H->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          b->data[i19 + b->size[0] * i18] = H->data[i18 + H->size[0] * i19];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i18 = C->size[0] * C->size[1];
        C->size[0] = y->size[0];
        C->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
        ib = y->size[0];
        for (i18 = 0; i18 < ib; i18++) {
          idx = b->size[1];
          for (i19 = 0; i19 < idx; i19++) {
            C->data[i18 + C->size[0] * i19] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              C->data[i18 + C->size[0] * i19] += y->data[i18 + y->size[0] * ar] *
                b->data[ar + b->size[0] * i19];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i18 = C->size[0] * C->size[1];
        C->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
        i18 = C->size[0] * C->size[1];
        C->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i18 = 0; i18 < ib; i18++) {
          C->data[i18] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i18 = idx + m;
            for (ic = idx; ic + 1 <= i18; ic++) {
              C->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i18 = br + k;
            for (ib = br; ib + 1 <= i18; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i19 = idx + m;
                for (ic = idx; ic + 1 <= i19; ic++) {
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

      i18 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
      ib = H->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        idx = H->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          b->data[i19 + b->size[0] * i18] = H->data[i18 + H->size[0] * i19];
        }
      }

      if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
        i18 = y->size[0] * y->size[1];
        y->size[0] = b_P->size[0];
        y->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        ib = b_P->size[0];
        for (i18 = 0; i18 < ib; i18++) {
          idx = b->size[1];
          for (i19 = 0; i19 < idx; i19++) {
            y->data[i18 + y->size[0] * i19] = 0.0;
            ii = b_P->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i18 + y->size[0] * i19] += b_P->data[i18 + b_P->size[0] *
                ar] * b->data[ar + b->size[0] * i19];
            }
          }
        }
      } else {
        k = b_P->size[1];
        a[0] = b_P->size[0];
        a[1] = b->size[1];
        m = b_P->size[0];
        i18 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        i18 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i18 = 0; i18 < ib; i18++) {
          y->data[i18] = 0.0;
        }

        if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = b_P->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i18 = idx + m;
            for (ic = idx; ic + 1 <= i18; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i18 = br + k;
            for (ib = br; ib + 1 <= i18; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i19 = idx + m;
                for (ic = idx; ic + 1 <= i19; ic++) {
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
      for (i18 = 0; i18 < ib; i18++) {
        b_C_data[i18] = C->data[i18] + b_R_data[i18];
      }

      mrdivide(y, b_C_data, b_C_size, K);
      if ((K->size[1] == 1) || (r_size[0] == 1)) {
        i18 = x_apo->size[0];
        x_apo->size[0] = K->size[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i18, (int)sizeof(double));
        ib = K->size[0];
        for (i18 = 0; i18 < ib; i18++) {
          x_apo->data[i18] = 0.0;
          idx = K->size[1];
          for (i19 = 0; i19 < idx; i19++) {
            x_apo->data[i18] += K->data[i18 + K->size[0] * i19] * b_r_data[i19];
          }
        }
      } else {
        k = K->size[1];
        a[0] = K->size[0];
        m = K->size[0];
        i18 = x_apo->size[0];
        x_apo->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i18, (int)sizeof(double));
        ib = (int)a[0];
        for (i18 = 0; i18 < ib; i18++) {
          x_apo->data[i18] = 0.0;
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
            i18 = br + k;
            for (ib = br; ib + 1 <= i18; ib++) {
              if (b_r_data[ib] != 0.0) {
                ia = ar;
                for (ic = 0; ic + 1 <= m; ic++) {
                  ia++;
                  x_apo->data[ic] += b_r_data[ib] * K->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx = m;
          }
        }
      }

      for (i18 = 0; i18 < 3; i18++) {
        c_xt[i18] = b_xt->data[i18] + x_apo->data[i18];
      }

      for (i18 = 0; i18 < 3; i18++) {
        b_xt->data[i18] = c_xt[i18];
      }

      for (i18 = 0; i18 < 3; i18++) {
        c_xt[i18] = x_apo->data[3 + i18];
      }

      quatPlusThetaJ(c_xt, dv41);
      quatmultJ(dv41, *(double (*)[4])&b_xt->data[3], dv39);
      for (i18 = 0; i18 < 4; i18++) {
        b_xt->data[3 + i18] = dv39[i18];
      }

      if (8.0 > c_numStatesxt) {
        i18 = 0;
        i19 = 0;
      } else {
        i18 = 7;
        i19 = (int)c_numStatesxt;
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

      ii = r12->size[0] * r12->size[1];
      r12->size[0] = 1;
      r12->size[1] = br - idx;
      emxEnsureCapacity((emxArray__common *)r12, ii, (int)sizeof(int));
      ib = br - idx;
      for (br = 0; br < ib; br++) {
        r12->data[r12->size[0] * br] = idx + br;
      }

      idx = e_xt->size[0];
      e_xt->size[0] = i19 - i18;
      emxEnsureCapacity((emxArray__common *)e_xt, idx, (int)sizeof(double));
      ib = i19 - i18;
      for (i19 = 0; i19 < ib; i19++) {
        e_xt->data[i19] = b_xt->data[i18 + i19] + x_apo->data[ar + i19];
      }

      ib = r12->size[1];
      for (i18 = 0; i18 < ib; i18++) {
        b_xt->data[r12->data[r12->size[0] * i18]] = e_xt->data[(*(int (*)[2])
          r12->size)[0] * i18];
      }

      for (br = 0; br < numAnchors; br++) {
        for (i18 = 0; i18 < 32; i18++) {
          x[i18] = (b_anchorFeatures->data[i18 + b_anchorFeatures->size[0] * br]
                    == 1.0);
        }

        if (any(x)) {
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
            numPointsPerAnchor);
          for (i18 = 0; i18 < 3; i18++) {
            c_xt[i18] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i18)) -
              1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i18)) - 1];
          }

          for (i18 = 0; i18 < 3; i18++) {
            b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i18)) - 1] =
              c_xt[i18];
          }

          d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
            numPointsPerAnchor);
          for (i18 = 0; i18 < 3; i18++) {
            c_xt[i18] = x_apo->data[(int)(d_numStates + (4.0 + (double)i18)) - 1];
          }

          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i18 = 0; i18 < 4; i18++) {
            d_xt[i18] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i18)) -
              1];
          }

          quatPlusThetaJ(c_xt, dv42);
          quatmultJ(dv42, d_xt, dv39);
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i18 = 0; i18 < 4; i18++) {
            b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i18)) - 1] =
              dv39[i18];
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
      i17 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
      ib = K->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = H->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          C->data[i17 + C->size[0] * i18] = 0.0;
          ii = K->size[1];
          for (i19 = 0; i19 < ii; i19++) {
            C->data[i17 + C->size[0] * i18] += K->data[i17 + K->size[0] * i19] *
              H->data[i19 + H->size[0] * i18];
          }
        }
      }
    } else {
      k = K->size[1];
      a[0] = (unsigned int)K->size[0];
      a[1] = (unsigned int)H->size[1];
      m = K->size[0];
      i17 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
      i17 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i17 = 0; i17 < ib; i17++) {
        C->data[i17] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
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
            if (H->data[ib] != 0.0) {
              ia = ar;
              i18 = idx + m;
              for (ic = idx; ic + 1 <= i18; ic++) {
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

    i17 = b_a->size[0] * b_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_a, i17, (int)sizeof(double));
    idx = b_a->size[0];
    ii = b_a->size[1];
    ib = idx * ii;
    for (i17 = 0; i17 < ib; i17++) {
      b_a->data[i17] -= C->data[i17];
    }

    i17 = b->size[0] * b->size[1];
    b->size[0] = b_P->size[0];
    b->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
    ib = b_P->size[0] * b_P->size[1];
    for (i17 = 0; i17 < ib; i17++) {
      b->data[i17] = b_P->data[i17];
    }

    emxInit_real_T(&c_a, 2);
    if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
      i17 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = b_a->size[0];
      c_a->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)c_a, i17, (int)sizeof(double));
      ib = b_a->size[0];
      for (i17 = 0; i17 < ib; i17++) {
        idx = b_P->size[1];
        for (i18 = 0; i18 < idx; i18++) {
          c_a->data[i17 + c_a->size[0] * i18] = 0.0;
          ii = b_a->size[1];
          for (i19 = 0; i19 < ii; i19++) {
            c_a->data[i17 + c_a->size[0] * i18] += b_a->data[i17 + b_a->size[0] *
              i19] * b_P->data[i19 + b_P->size[0] * i18];
          }
        }
      }

      i17 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = c_a->size[0];
      b_P->size[1] = c_a->size[1];
      emxEnsureCapacity((emxArray__common *)b_P, i17, (int)sizeof(double));
      ib = c_a->size[1];
      for (i17 = 0; i17 < ib; i17++) {
        idx = c_a->size[0];
        for (i18 = 0; i18 < idx; i18++) {
          b_P->data[i18 + b_P->size[0] * i17] = c_a->data[i18 + c_a->size[0] *
            i17];
        }
      }
    } else {
      k = b_a->size[1];
      a[0] = b_a->size[0];
      a[1] = b_P->size[1];
      m = b_a->size[0];
      i17 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = (int)a[0];
      b_P->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)b_P, i17, (int)sizeof(double));
      ib = (int)a[1];
      for (i17 = 0; i17 < ib; i17++) {
        idx = (int)a[0];
        for (i18 = 0; i18 < idx; i18++) {
          b_P->data[i18 + b_P->size[0] * i17] = 0.0;
        }
      }

      if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_a->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i17 = idx + m;
          for (ic = idx; ic + 1 <= i17; ic++) {
            b_P->data[ic] = 0.0;
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
    f_fprintf(idx);
  }

  // % D Partial EKF update using high-innovation inliers
  for (i17 = 0; i17 < loop_ub; i17++) {
    HI_inlierStatus_data[i17] = true;
  }

  //  high innovation inliers
  i17 = x_apo_prev->size[0];
  x_apo_prev->size[0] = b_P->size[0];
  emxEnsureCapacity((emxArray__common *)x_apo_prev, i17, (int)sizeof(double));
  ib = b_P->size[0];
  for (i17 = 0; i17 < ib; i17++) {
    x_apo_prev->data[i17] = 0.0;
  }

  i17 = (int)maxEKFIterations;
  it = 0;
  b_emxInit_real_T(&b_x_apo, 1);
  b_emxInit_real_T(&f_xt, 1);
  exitg1 = false;
  while ((!exitg1) && (it <= i17 - 1)) {
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
    for (i18 = 0; i18 < br; i18++) {
      b_indMeas_data[i18] = indMeas_data[ii_data[i18] - 1];
    }

    b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                 b_indMeas_size, map, c_cameraparams_CameraParameters,
                 d_cameraparams_CameraParameters,
                 e_cameraparams_CameraParameters, (double)numAnchors,
                 numPointsPerAnchor, anchorInd, featureAnchorInd, b_m_vect,
                 imNoise, IMU_measurements, height_offset_pressure, b_r_data,
                 r_size, H, unusedU2_data, unusedU2_size, b_R_data, b_R_size);
    if (1.0 + (double)it == 1.0) {
      //  only do outlier rejection in first iteration
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i18 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        ib = H->size[0];
        for (i18 = 0; i18 < ib; i18++) {
          idx = b_P->size[1];
          for (i19 = 0; i19 < idx; i19++) {
            y->data[i18 + y->size[0] * i19] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i18 + y->size[0] * i19] += H->data[i18 + H->size[0] * ar] *
                b_P->data[ar + b_P->size[0] * i19];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i18 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        i18 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i18 = 0; i18 < ib; i18++) {
          y->data[i18] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i18 = idx + m;
            for (ic = idx; ic + 1 <= i18; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i18 = br + k;
            for (ib = br; ib + 1 <= i18; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i19 = idx + m;
                for (ic = idx; ic + 1 <= i19; ic++) {
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

      i18 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
      ib = H->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        idx = H->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          b->data[i19 + b->size[0] * i18] = H->data[i18 + H->size[0] * i19];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i18 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = y->size[0];
        b_a->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_a, i18, (int)sizeof(double));
        ib = y->size[0];
        for (i18 = 0; i18 < ib; i18++) {
          idx = b->size[1];
          for (i19 = 0; i19 < idx; i19++) {
            b_a->data[i18 + b_a->size[0] * i19] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              b_a->data[i18 + b_a->size[0] * i19] += y->data[i18 + y->size[0] *
                ar] * b->data[ar + b->size[0] * i19];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i18 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)b_a, i18, (int)sizeof(double));
        i18 = b_a->size[0] * b_a->size[1];
        b_a->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)b_a, i18, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i18 = 0; i18 < ib; i18++) {
          b_a->data[i18] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i18 = idx + m;
            for (ic = idx; ic + 1 <= i18; ic++) {
              b_a->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i18 = br + k;
            for (ib = br; ib + 1 <= i18; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i19 = idx + m;
                for (ic = idx; ic + 1 <= i19; ic++) {
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
      for (i18 = 0; i18 < ib; i18++) {
        S_data[i18] = b_a->data[i18] + b_R_data[i18];
      }

      for (k = 0; k < numMeas; k++) {
        ar = k << 1;
        ii = k << 1;
        idx = k << 1;
        for (i18 = 0; i18 < 2; i18++) {
          r[i18] = b_r_data[i18 + ar];
          for (i19 = 0; i19 < 2; i19++) {
            d_xt[i19 + (i18 << 1)] = S_data[(i19 + ii) + br * (i18 + idx)];
          }
        }

        b_mrdivide(r, d_xt, a);
        ar = k << 1;
        for (i18 = 0; i18 < 2; i18++) {
          r[i18] = b_r_data[i18 + ar];
        }

        if (LI_inlierStatus_data[k]) {
          //  if this feature is a LI inlier, don't also do HI update with this feature 
          ar = k << 1;
          for (i18 = 0; i18 < 2; i18++) {
            b_r_data[i18 + ar] = 0.0;
          }

          ib = H->size[1];
          ar = k << 1;
          for (i18 = 0; i18 < ib; i18++) {
            for (i19 = 0; i19 < 2; i19++) {
              H->data[(i19 + ar) + H->size[0] * i18] = 0.0;
            }
          }

          HI_inlierStatus_data[k] = false;
        } else {
          //  otherwise check if HI inlier for both cams
          d_numStatesxt = 0.0;
          for (i18 = 0; i18 < 2; i18++) {
            d_numStatesxt += a[i18] * r[i18];
          }

          if (d_numStatesxt > 6.0) {
            ar = k << 1;
            for (i18 = 0; i18 < 2; i18++) {
              b_r_data[i18 + ar] = 0.0;
            }

            ib = H->size[1];
            ar = k << 1;
            for (i18 = 0; i18 < ib; i18++) {
              for (i19 = 0; i19 < 2; i19++) {
                H->data[(i19 + ar) + H->size[0] * i18] = 0.0;
              }
            }

            HI_inlierStatus_data[k] = false;
          }
        }
      }
    }

    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i18 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
      ib = H->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        idx = b_P->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          y->data[i18 + y->size[0] * i19] = 0.0;
          ii = H->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i18 + y->size[0] * i19] += H->data[i18 + H->size[0] * ar] *
              b_P->data[ar + b_P->size[0] * i19];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      m = H->size[0];
      i18 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
      i18 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i18 = 0; i18 < ib; i18++) {
        y->data[i18] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i18 = idx + m;
          for (ic = idx; ic + 1 <= i18; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i18 = br + k;
          for (ib = br; ib + 1 <= i18; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i19 = idx + m;
              for (ic = idx; ic + 1 <= i19; ic++) {
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

    i18 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
    ib = H->size[0];
    for (i18 = 0; i18 < ib; i18++) {
      idx = H->size[1];
      for (i19 = 0; i19 < idx; i19++) {
        b->data[i19 + b->size[0] * i18] = H->data[i18 + H->size[0] * i19];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i18 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
      ib = y->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        idx = b->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          C->data[i18 + C->size[0] * i19] = 0.0;
          ii = y->size[1];
          for (ar = 0; ar < ii; ar++) {
            C->data[i18 + C->size[0] * i19] += y->data[i18 + y->size[0] * ar] *
              b->data[ar + b->size[0] * i19];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i18 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
      i18 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i18 = 0; i18 < ib; i18++) {
        C->data[i18] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i18 = idx + m;
          for (ic = idx; ic + 1 <= i18; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i18 = br + k;
          for (ib = br; ib + 1 <= i18; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i19 = idx + m;
              for (ic = idx; ic + 1 <= i19; ic++) {
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

    i18 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
    ib = H->size[0];
    for (i18 = 0; i18 < ib; i18++) {
      idx = H->size[1];
      for (i19 = 0; i19 < idx; i19++) {
        b->data[i19 + b->size[0] * i18] = H->data[i18 + H->size[0] * i19];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i18 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
      ib = b_P->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        idx = b->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          y->data[i18 + y->size[0] * i19] = 0.0;
          ii = b_P->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i18 + y->size[0] * i19] += b_P->data[i18 + b_P->size[0] * ar]
              * b->data[ar + b->size[0] * i19];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i18 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
      i18 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i18 = 0; i18 < ib; i18++) {
        y->data[i18] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i18 = idx + m;
          for (ic = idx; ic + 1 <= i18; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i18 = br + k;
          for (ib = br; ib + 1 <= i18; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i19 = idx + m;
              for (ic = idx; ic + 1 <= i19; ic++) {
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
    for (i18 = 0; i18 < ib; i18++) {
      b_C_data[i18] = C->data[i18] + b_R_data[i18];
    }

    mrdivide(y, b_C_data, c_C_size, K);
    if ((K->size[1] == 1) || (r_size[0] == 1)) {
      i18 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i18, (int)sizeof(double));
      ib = K->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        x_apo->data[i18] = 0.0;
        idx = K->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          x_apo->data[i18] += K->data[i18 + K->size[0] * i19] * b_r_data[i19];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      m = K->size[0];
      i18 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i18, (int)sizeof(double));
      ib = (int)a[0];
      for (i18 = 0; i18 < ib; i18++) {
        x_apo->data[i18] = 0.0;
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
          i18 = br + k;
          for (ib = br; ib + 1 <= i18; ib++) {
            if (b_r_data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                x_apo->data[ic] += b_r_data[ib] * K->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx = m;
        }
      }
    }

    for (i18 = 0; i18 < 3; i18++) {
      c_xt[i18] = b_xt->data[i18] + x_apo->data[i18];
    }

    for (i18 = 0; i18 < 3; i18++) {
      b_xt->data[i18] = c_xt[i18];
    }

    for (i18 = 0; i18 < 3; i18++) {
      c_xt[i18] = x_apo->data[3 + i18];
    }

    quatPlusThetaJ(c_xt, dv43);
    quatmultJ(dv43, *(double (*)[4])&b_xt->data[3], dv39);
    for (i18 = 0; i18 < 4; i18++) {
      b_xt->data[3 + i18] = dv39[i18];
    }

    if (8.0 > c_numStatesxt) {
      i18 = 0;
      i19 = 0;
    } else {
      i18 = 7;
      i19 = (int)c_numStatesxt;
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

    ii = r12->size[0] * r12->size[1];
    r12->size[0] = 1;
    r12->size[1] = br - idx;
    emxEnsureCapacity((emxArray__common *)r12, ii, (int)sizeof(int));
    ib = br - idx;
    for (br = 0; br < ib; br++) {
      r12->data[r12->size[0] * br] = idx + br;
    }

    idx = f_xt->size[0];
    f_xt->size[0] = i19 - i18;
    emxEnsureCapacity((emxArray__common *)f_xt, idx, (int)sizeof(double));
    ib = i19 - i18;
    for (i19 = 0; i19 < ib; i19++) {
      f_xt->data[i19] = b_xt->data[i18 + i19] + x_apo->data[ar + i19];
    }

    ib = r12->size[1];
    for (i18 = 0; i18 < ib; i18++) {
      b_xt->data[r12->data[r12->size[0] * i18]] = f_xt->data[(*(int (*)[2])
        r12->size)[0] * i18];
    }

    for (br = 0; br < numAnchors; br++) {
      for (i18 = 0; i18 < 32; i18++) {
        x[i18] = (b_anchorFeatures->data[i18 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i18 = 0; i18 < 3; i18++) {
          c_xt[i18] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i18)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i18)) - 1];
        }

        for (i18 = 0; i18 < 3; i18++) {
          b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i18)) - 1] = c_xt[i18];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i18 = 0; i18 < 3; i18++) {
          c_xt[i18] = x_apo->data[(int)(d_numStates + (4.0 + (double)i18)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i18 = 0; i18 < 4; i18++) {
          d_xt[i18] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i18)) - 1];
        }

        quatPlusThetaJ(c_xt, dv44);
        quatmultJ(dv44, d_xt, dv39);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i18 = 0; i18 < 4; i18++) {
          b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i18)) - 1] = dv39[i18];
        }

        for (ii = 0; ii < (int)numPointsPerAnchor; ii++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) + (1.0 + (double)ii)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ii)) - 1];
        }
      }
    }

    i18 = b_x_apo->size[0];
    b_x_apo->size[0] = x_apo->size[0];
    emxEnsureCapacity((emxArray__common *)b_x_apo, i18, (int)sizeof(double));
    ib = x_apo->size[0];
    for (i18 = 0; i18 < ib; i18++) {
      b_x_apo->data[i18] = x_apo->data[i18] - x_apo_prev->data[i18];
    }

    if (d_norm(b_x_apo) < 0.001) {
      exitg1 = true;
    } else {
      i18 = x_apo_prev->size[0];
      x_apo_prev->size[0] = x_apo->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo_prev, i18, (int)sizeof(double));
      ib = x_apo->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        x_apo_prev->data[i18] = x_apo->data[i18];
      }

      it++;
    }
  }

  emxFree_real_T(&f_xt);
  emxFree_real_T(&b_x_apo);
  emxFree_real_T(&y);
  emxFree_int32_T(&r12);
  emxFree_real_T(&featureAnchorInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&map);
  emxFree_real_T(&x_apo_prev);
  emxFree_real_T(&x_apo);
  eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
       numPointsPerAnchor), b_a);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i17 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
    loop_ub = K->size[0];
    for (i17 = 0; i17 < loop_ub; i17++) {
      ib = H->size[1];
      for (i18 = 0; i18 < ib; i18++) {
        C->data[i17 + C->size[0] * i18] = 0.0;
        idx = K->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          C->data[i17 + C->size[0] * i18] += K->data[i17 + K->size[0] * i19] *
            H->data[i19 + H->size[0] * i18];
        }
      }
    }
  } else {
    k = K->size[1];
    a[0] = (unsigned int)K->size[0];
    a[1] = (unsigned int)H->size[1];
    m = K->size[0];
    i17 = C->size[0] * C->size[1];
    C->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
    i17 = C->size[0] * C->size[1];
    C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)C, i17, (int)sizeof(double));
    loop_ub = (int)a[0] * (int)a[1];
    for (i17 = 0; i17 < loop_ub; i17++) {
      C->data[i17] = 0.0;
    }

    if ((K->size[0] == 0) || (H->size[1] == 0)) {
    } else {
      ii = K->size[0] * (H->size[1] - 1);
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
          if (H->data[ib] != 0.0) {
            ia = ar;
            i18 = idx + m;
            for (ic = idx; ic + 1 <= i18; ic++) {
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
  i17 = b_a->size[0] * b_a->size[1];
  emxEnsureCapacity((emxArray__common *)b_a, i17, (int)sizeof(double));
  idx = b_a->size[0];
  ii = b_a->size[1];
  loop_ub = idx * ii;
  for (i17 = 0; i17 < loop_ub; i17++) {
    b_a->data[i17] -= C->data[i17];
  }

  emxFree_real_T(&C);
  i17 = b->size[0] * b->size[1];
  b->size[0] = b_P->size[0];
  b->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b, i17, (int)sizeof(double));
  loop_ub = b_P->size[0] * b_P->size[1];
  for (i17 = 0; i17 < loop_ub; i17++) {
    b->data[i17] = b_P->data[i17];
  }

  emxInit_real_T(&d_a, 2);
  if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
    i17 = d_a->size[0] * d_a->size[1];
    d_a->size[0] = b_a->size[0];
    d_a->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)d_a, i17, (int)sizeof(double));
    loop_ub = b_a->size[0];
    for (i17 = 0; i17 < loop_ub; i17++) {
      ib = b_P->size[1];
      for (i18 = 0; i18 < ib; i18++) {
        d_a->data[i17 + d_a->size[0] * i18] = 0.0;
        idx = b_a->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          d_a->data[i17 + d_a->size[0] * i18] += b_a->data[i17 + b_a->size[0] *
            i19] * b_P->data[i19 + b_P->size[0] * i18];
        }
      }
    }

    i17 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = d_a->size[0];
    b_P->size[1] = d_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i17, (int)sizeof(double));
    loop_ub = d_a->size[1];
    for (i17 = 0; i17 < loop_ub; i17++) {
      ib = d_a->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        b_P->data[i18 + b_P->size[0] * i17] = d_a->data[i18 + d_a->size[0] * i17];
      }
    }
  } else {
    k = b_a->size[1];
    a[0] = b_a->size[0];
    a[1] = b_P->size[1];
    m = b_a->size[0];
    i17 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = (int)a[0];
    b_P->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)b_P, i17, (int)sizeof(double));
    loop_ub = (int)a[1];
    for (i17 = 0; i17 < loop_ub; i17++) {
      ib = (int)a[0];
      for (i18 = 0; i18 < ib; i18++) {
        b_P->data[i18 + b_P->size[0] * i17] = 0.0;
      }
    }

    if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = b_a->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i17 = idx + m;
        for (ic = idx; ic + 1 <= i17; ic++) {
          b_P->data[ic] = 0.0;
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
    h_fprintf();
  }
}

//
// File trailer for OnePointRANSAC_EKF.cpp
//
// [EOF]
//
