//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Aug-2015 16:03:26
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
//                const emxArray_real_T *b_anchorFeatures
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[13]
//                double height_offset_pressure
//                double validFeatures_data[]
//                int validFeatures_size[1]
// Return Type  : void
//
void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P, const
  double z_all_l[32], double c_numStatesxt, double c_numStates, double
  numPointsPerAnchor, const emxArray_real_T *b_anchorFeatures, const
  emxArray_real_T *b_m_vect, const double imNoise[2], const double
  IMU_measurements[13], double height_offset_pressure, double
  validFeatures_data[], int validFeatures_size[1])
{
  emxArray_boolean_T *c_anchorFeatures;
  int numAnchors;
  int i20;
  int loop_ub;
  boolean_T x[16];
  int idx;
  int ii_data[16];
  int ii;
  boolean_T exitg3;
  boolean_T guard2 = false;
  int ar;
  signed char indMeas_data[16];
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
  emxArray_real_T *K;
  emxArray_real_T *x_apo_prev;
  emxArray_real_T *H_i;
  emxArray_real_T *C;
  emxArray_int32_T *r13;
  emxArray_real_T *y;
  emxArray_real_T *b;
  emxArray_int32_T *r14;
  emxArray_int32_T *r15;
  emxArray_real_T *b_x_apo_prev;
  double d_numStatesxt;
  int R_size[2];
  double R_data[36];
  double r[2];
  int unusedU1_size[1];
  double r_data[6];
  int ib;
  int i21;
  int i22;
  int k;
  double a[2];
  int m;
  int ic;
  int br;
  int ia;
  double C_data[36];
  int C_size[2];
  double c_xt[3];
  double dv49[4];
  double dv50[4];
  double e_numStatesxt;
  double d_numStates;
  double d_xt[4];
  double dv51[4];
  double R_cw[9];
  boolean_T HI_inlierStatus_data[16];
  int indMeasIdx;
  double featureAnchorIdx;
  boolean_T exitg2;
  boolean_T guard1 = false;
  signed char featureIdxVect_data[16];
  double anchorPos[3];
  double c_x_apo_prev[9];
  double b_anchorPos[3];
  static const signed char iv13[3] = { 0, 1, 2 };

  double b_z_all_l[2];
  static const signed char iv14[2] = { 1, 2 };

  emxArray_real_T *H;
  emxArray_real_T *b_a;
  double b_indMeas_data[16];
  int indMeas_size[1];
  int b_R_size[2];
  double b_R_data[1296];
  double unusedU1_data[32];
  int r_size[1];
  double b_r_data[36];
  double b_C_data[1296];
  int b_C_size[2];
  double dv52[4];
  emxArray_real_T *e_xt;
  double dv53[4];
  emxArray_real_T *c_a;
  int it;
  emxArray_real_T *b_x_apo;
  emxArray_real_T *f_xt;
  boolean_T exitg1;
  int b_indMeas_size[1];
  double S_data[1296];
  int c_C_size[2];
  double dv54[4];
  double dv55[4];
  emxArray_real_T *d_a;
  emxInit_boolean_T(&c_anchorFeatures, 2);

  //  threshold for LI innovation test
  // 9.487729036781154; % HI mahalanobis gate
  //  threshold for minimum LI supporter
  numAnchors = b_anchorFeatures->size[1] - 1;
  i20 = c_anchorFeatures->size[0] * c_anchorFeatures->size[1];
  c_anchorFeatures->size[0] = 16;
  c_anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)c_anchorFeatures, i20, (int)sizeof
                    (boolean_T));
  loop_ub = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i20 = 0; i20 < loop_ub; i20++) {
    c_anchorFeatures->data[i20] = (b_anchorFeatures->data[i20] == 1.0);
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

  for (i20 = 0; i20 < loop_ub; i20++) {
    indMeas_data[i20] = (signed char)ii_data[i20];
  }

  numMeas = ar;

  // % B 1-point hypotheses generation and evaluation
  n_hyp = 100.0;
  LI_inlierStatus_size_idx_0 = ar;
  for (i20 = 0; i20 < ar; i20++) {
    LI_inlierStatus_data[i20] = false;
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
  emxInit_real_T(&K, 2);
  b_emxInit_real_T(&x_apo_prev, 1);
  emxInit_real_T(&H_i, 2);
  emxInit_real_T(&C, 2);
  b_emxInit_int32_T(&r13, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b, 2);
  emxInit_int32_T(&r14, 1);
  emxInit_int32_T(&r15, 1);
  b_emxInit_real_T(&b_x_apo_prev, 1);
  while (i < n_hyp) {
    //  select a random measurement
    d_numStatesxt = b_rand();

    //  EKF state update
    getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, (double)indMeas_data
               [(int)(1.0 + floor(d_numStatesxt * (double)numMeas)) - 1], map,
               (double)(numAnchors + 1), numPointsPerAnchor, anchorInd,
               featureAnchorInd, b_m_vect, imNoise, IMU_measurements,
               height_offset_pressure, r_data, unusedU1_size, H_i, r, R_data,
               R_size);
    if ((H_i->size[1] == 1) || (b_P->size[0] == 1)) {
      i20 = y->size[0] * y->size[1];
      y->size[0] = H_i->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = H_i->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b_P->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          y->data[i20 + y->size[0] * i21] = 0.0;
          ii = H_i->size[1];
          for (i22 = 0; i22 < ii; i22++) {
            y->data[i20 + y->size[0] * i21] += H_i->data[i20 + H_i->size[0] *
              i22] * b_P->data[i22 + b_P->size[0] * i21];
          }
        }
      }
    } else {
      k = H_i->size[1];
      a[0] = H_i->size[0];
      a[1] = b_P->size[1];
      m = H_i->size[0];
      i20 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      i20 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        y->data[i20] = 0.0;
      }

      if ((H_i->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H_i->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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

    i20 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = H_i->size[0];
    emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
    ib = H_i->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      idx = H_i->size[1];
      for (i21 = 0; i21 < idx; i21++) {
        b->data[i21 + b->size[0] * i20] = H_i->data[i20 + H_i->size[0] * i21];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i20 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      ib = y->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          C->data[i20 + C->size[0] * i21] = 0.0;
          ii = y->size[1];
          for (i22 = 0; i22 < ii; i22++) {
            C->data[i20 + C->size[0] * i21] += y->data[i20 + y->size[0] * i22] *
              b->data[i22 + b->size[0] * i21];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i20 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      i20 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i20 = 0; i20 < ib; i20++) {
        C->data[i20] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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

    i20 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = H_i->size[0];
    emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
    ib = H_i->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      idx = H_i->size[1];
      for (i21 = 0; i21 < idx; i21++) {
        b->data[i21 + b->size[0] * i20] = H_i->data[i20 + H_i->size[0] * i21];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i20 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = b_P->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          y->data[i20 + y->size[0] * i21] = 0.0;
          ii = b_P->size[1];
          for (i22 = 0; i22 < ii; i22++) {
            y->data[i20 + y->size[0] * i21] += b_P->data[i20 + b_P->size[0] *
              i22] * b->data[i22 + b->size[0] * i21];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i20 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      i20 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        y->data[i20] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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
    for (i20 = 0; i20 < ib; i20++) {
      C_data[i20] = C->data[i20] + R_data[i20];
    }

    mrdivide(y, C_data, C_size, K);
    i20 = K_i->size[0] * K_i->size[1];
    K_i->size[0] = K->size[0];
    K_i->size[1] = K->size[1];
    emxEnsureCapacity((emxArray__common *)K_i, i20, (int)sizeof(double));
    ib = K->size[0] * K->size[1];
    for (i20 = 0; i20 < ib; i20++) {
      K_i->data[i20] = K->data[i20];
    }

    if ((K_i->size[1] == 1) || (unusedU1_size[0] == 1)) {
      i20 = x_apo->size[0];
      x_apo->size[0] = K_i->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
      ib = K_i->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo->data[i20] = 0.0;
        idx = K_i->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          x_apo->data[i20] += K_i->data[i20 + K_i->size[0] * i21] * r_data[i21];
        }
      }
    } else {
      k = K_i->size[1];
      a[0] = K_i->size[0];
      m = K_i->size[0];
      i20 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
      ib = (int)a[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo->data[i20] = 0.0;
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
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
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

    i20 = x_apo_prev->size[0];
    x_apo_prev->size[0] = b_xt->size[0];
    emxEnsureCapacity((emxArray__common *)x_apo_prev, i20, (int)sizeof(double));
    ib = b_xt->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      x_apo_prev->data[i20] = b_xt->data[i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      x_apo_prev->data[i20] = b_xt->data[i20] + x_apo->data[i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      c_xt[i20] = x_apo->data[3 + i20];
    }

    quatPlusThetaJ(c_xt, dv49);
    quatmultJ(dv49, *(double (*)[4])&x_apo_prev->data[3], dv50);
    for (i20 = 0; i20 < 4; i20++) {
      x_apo_prev->data[3 + i20] = dv50[i20];
    }

    if (8.0 > c_numStatesxt) {
      i20 = 1;
      i21 = 0;
    } else {
      i20 = 8;
      i21 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      i22 = 1;
      ar = 0;
    } else {
      i22 = 7;
      ar = (int)c_numStates;
    }

    if (8.0 > c_numStatesxt) {
      idx = 0;
      br = 0;
    } else {
      idx = 7;
      br = (int)c_numStatesxt;
    }

    ii = r13->size[0] * r13->size[1];
    r13->size[0] = 1;
    r13->size[1] = br - idx;
    emxEnsureCapacity((emxArray__common *)r13, ii, (int)sizeof(int));
    ib = br - idx;
    for (br = 0; br < ib; br++) {
      r13->data[r13->size[0] * br] = idx + br;
    }

    idx = r14->size[0];
    r14->size[0] = (i21 - i20) + 1;
    emxEnsureCapacity((emxArray__common *)r14, idx, (int)sizeof(int));
    ib = i21 - i20;
    for (i21 = 0; i21 <= ib; i21++) {
      r14->data[i21] = i20 + i21;
    }

    i20 = r15->size[0];
    r15->size[0] = (ar - i22) + 1;
    emxEnsureCapacity((emxArray__common *)r15, i20, (int)sizeof(int));
    ib = ar - i22;
    for (i20 = 0; i20 <= ib; i20++) {
      r15->data[i20] = i22 + i20;
    }

    i20 = b_x_apo_prev->size[0];
    b_x_apo_prev->size[0] = r13->size[0] * r13->size[1];
    emxEnsureCapacity((emxArray__common *)b_x_apo_prev, i20, (int)sizeof(double));
    ib = r13->size[0] * r13->size[1];
    for (i20 = 0; i20 < ib; i20++) {
      b_x_apo_prev->data[i20] = x_apo_prev->data[r14->data[i20] - 1] +
        x_apo->data[r15->data[i20] - 1];
    }

    ib = b_x_apo_prev->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      x_apo_prev->data[r13->data[i20]] = b_x_apo_prev->data[i20];
    }

    for (br = 0; br <= numAnchors; br++) {
      for (i20 = 0; i20 < 16; i20++) {
        x[i20] = (b_anchorFeatures->data[i20 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 3; i20++) {
          c_xt[i20] = x_apo_prev->data[(int)(d_numStatesxt + (1.0 + (double)i20))
            - 1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i20)) - 1];
        }

        for (i20 = 0; i20 < 3; i20++) {
          x_apo_prev->data[(int)(e_numStatesxt + (1.0 + (double)i20)) - 1] =
            c_xt[i20];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 3; i20++) {
          c_xt[i20] = x_apo->data[(int)(d_numStates + (4.0 + (double)i20)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 4; i20++) {
          d_xt[i20] = x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i20))
            - 1];
        }

        quatPlusThetaJ(c_xt, dv51);
        quatmultJ(dv51, d_xt, dv50);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 4; i20++) {
          x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i20)) - 1] =
            dv50[i20];
        }

        for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
          x_apo_prev->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) + (1.0 + (double)idx)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)idx)) - 1];
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
    for (i20 = 0; i20 < numMeas; i20++) {
      HI_inlierStatus_data[i20] = false;
    }

    //  inliers of this iteration
    indMeasIdx = 0;
    for (br = 0; br <= numAnchors; br++) {
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

      for (i20 = 0; i20 < ib; i20++) {
        featureIdxVect_data[i20] = (signed char)ii_data[i20];
      }

      for (idx = 0; idx < ar; idx++) {
        if (b_anchorFeatures->data[(featureIdxVect_data[idx] +
             b_anchorFeatures->size[0] * br) - 1] == 1.0) {
          //  if this is not a lost feature
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i20 = 0; i20 < 3; i20++) {
            anchorPos[i20] = x_apo_prev->data[(int)(e_numStatesxt + (1.0 +
              (double)i20)) - 1];
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
          for (i20 = 0; i20 < 3; i20++) {
            d_numStatesxt = 0.0;
            for (i21 = 0; i21 < 3; i21++) {
              d_numStatesxt += c_x_apo_prev[i20 + 3 * i21] * b_m_vect->data[i21
                + b_m_vect->size[0] * (featureIdxVect_data[idx] - 1)];
            }

            b_anchorPos[i20] = (anchorPos[i20] + d_numStatesxt / d_numStates) -
              x_apo_prev->data[iv13[i20]];
          }

          for (i20 = 0; i20 < 3; i20++) {
            c_xt[i20] = 0.0;
            for (i21 = 0; i21 < 3; i21++) {
              c_xt[i20] += R_cw[i20 + 3 * i21] * b_anchorPos[i21];
            }
          }

          predictMeasurement_left(c_xt, r);
          ii = (featureIdxVect_data[idx] - 1) * 2;
          for (i20 = 0; i20 < 2; i20++) {
            b_z_all_l[i20] = z_all_l[(ii + iv14[i20]) - 1] - r[i20];
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
      for (i20 = 0; i20 < numMeas; i20++) {
        LI_inlierStatus_data[i20] = HI_inlierStatus_data[i20];
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
  emxFree_real_T(&H_i);
  emxFree_real_T(&K_i);
  idx = 0;
  for (k = 0; k < LI_inlierStatus_size_idx_0; k++) {
    if (LI_inlierStatus_data[k]) {
      idx++;
    }
  }

  emxInit_real_T(&H, 2);
  emxInit_real_T(&b_a, 2);
  if (idx > 3) {
    //      fprintf('Ended hypothesis test after %i iterations. %i of %i features are LI inliers\n', i, nnz(LI_inlierStatus), numMeas) 
    // % C Partial EKF update using low-innovation inliers
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
    for (i20 = 0; i20 < br; i20++) {
      b_indMeas_data[i20] = indMeas_data[ii_data[i20] - 1];
    }

    b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                 indMeas_size, map, (double)b_anchorFeatures->size[1],
                 numPointsPerAnchor, anchorInd, featureAnchorInd, b_m_vect,
                 imNoise, IMU_measurements, height_offset_pressure, b_r_data,
                 r_size, H, unusedU1_data, unusedU1_size, b_R_data, b_R_size);
    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i20 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = H->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b_P->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          y->data[i20 + y->size[0] * i21] = 0.0;
          ii = H->size[1];
          for (i22 = 0; i22 < ii; i22++) {
            y->data[i20 + y->size[0] * i21] += H->data[i20 + H->size[0] * i22] *
              b_P->data[i22 + b_P->size[0] * i21];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      m = H->size[0];
      i20 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      i20 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        y->data[i20] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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

    i20 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
    ib = H->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      idx = H->size[1];
      for (i21 = 0; i21 < idx; i21++) {
        b->data[i21 + b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i20 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      ib = y->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          C->data[i20 + C->size[0] * i21] = 0.0;
          ii = y->size[1];
          for (i22 = 0; i22 < ii; i22++) {
            C->data[i20 + C->size[0] * i21] += y->data[i20 + y->size[0] * i22] *
              b->data[i22 + b->size[0] * i21];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i20 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      i20 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i20 = 0; i20 < ib; i20++) {
        C->data[i20] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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

    i20 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
    ib = H->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      idx = H->size[1];
      for (i21 = 0; i21 < idx; i21++) {
        b->data[i21 + b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i20 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = b_P->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          y->data[i20 + y->size[0] * i21] = 0.0;
          ii = b_P->size[1];
          for (i22 = 0; i22 < ii; i22++) {
            y->data[i20 + y->size[0] * i21] += b_P->data[i20 + b_P->size[0] *
              i22] * b->data[i22 + b->size[0] * i21];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i20 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      i20 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        y->data[i20] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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
    for (i20 = 0; i20 < ib; i20++) {
      b_C_data[i20] = C->data[i20] + b_R_data[i20];
    }

    mrdivide(y, b_C_data, b_C_size, K);
    if ((K->size[1] == 1) || (r_size[0] == 1)) {
      i20 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
      ib = K->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo->data[i20] = 0.0;
        idx = K->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          x_apo->data[i20] += K->data[i20 + K->size[0] * i21] * b_r_data[i21];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      m = K->size[0];
      i20 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
      ib = (int)a[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo->data[i20] = 0.0;
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
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
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

    for (i20 = 0; i20 < 3; i20++) {
      c_xt[i20] = b_xt->data[i20] + x_apo->data[i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      b_xt->data[i20] = c_xt[i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      c_xt[i20] = x_apo->data[3 + i20];
    }

    quatPlusThetaJ(c_xt, dv52);
    quatmultJ(dv52, *(double (*)[4])&b_xt->data[3], dv50);
    for (i20 = 0; i20 < 4; i20++) {
      b_xt->data[3 + i20] = dv50[i20];
    }

    if (8.0 > c_numStatesxt) {
      i20 = 0;
      i21 = 0;
    } else {
      i20 = 7;
      i21 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      i22 = 0;
    } else {
      i22 = 6;
    }

    if (8.0 > c_numStatesxt) {
      ar = 0;
      idx = 0;
    } else {
      ar = 7;
      idx = (int)c_numStatesxt;
    }

    br = r13->size[0] * r13->size[1];
    r13->size[0] = 1;
    r13->size[1] = idx - ar;
    emxEnsureCapacity((emxArray__common *)r13, br, (int)sizeof(int));
    ib = idx - ar;
    for (idx = 0; idx < ib; idx++) {
      r13->data[r13->size[0] * idx] = ar + idx;
    }

    b_emxInit_real_T(&e_xt, 1);
    ar = e_xt->size[0];
    e_xt->size[0] = i21 - i20;
    emxEnsureCapacity((emxArray__common *)e_xt, ar, (int)sizeof(double));
    ib = i21 - i20;
    for (i21 = 0; i21 < ib; i21++) {
      e_xt->data[i21] = b_xt->data[i20 + i21] + x_apo->data[i22 + i21];
    }

    ib = r13->size[1];
    for (i20 = 0; i20 < ib; i20++) {
      b_xt->data[r13->data[r13->size[0] * i20]] = e_xt->data[(*(int (*)[2])
        r13->size)[0] * i20];
    }

    emxFree_real_T(&e_xt);
    for (br = 0; br <= numAnchors; br++) {
      for (i20 = 0; i20 < 16; i20++) {
        x[i20] = (b_anchorFeatures->data[i20 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 3; i20++) {
          c_xt[i20] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i20)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i20)) - 1];
        }

        for (i20 = 0; i20 < 3; i20++) {
          b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i20)) - 1] = c_xt[i20];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 3; i20++) {
          c_xt[i20] = x_apo->data[(int)(d_numStates + (4.0 + (double)i20)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 4; i20++) {
          d_xt[i20] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i20)) - 1];
        }

        quatPlusThetaJ(c_xt, dv53);
        quatmultJ(dv53, d_xt, dv50);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 4; i20++) {
          b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i20)) - 1] = dv50[i20];
        }

        for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) + (1.0 + (double)idx)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)idx)) - 1];
        }
      }
    }

    b_eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
           numPointsPerAnchor), b_a);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i20 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      ib = K->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = H->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          C->data[i20 + C->size[0] * i21] = 0.0;
          ii = K->size[1];
          for (i22 = 0; i22 < ii; i22++) {
            C->data[i20 + C->size[0] * i21] += K->data[i20 + K->size[0] * i22] *
              H->data[i22 + H->size[0] * i21];
          }
        }
      }
    } else {
      k = K->size[1];
      a[0] = (unsigned int)K->size[0];
      a[1] = (unsigned int)H->size[1];
      m = K->size[0];
      i20 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      i20 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        C->data[i20] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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

    i20 = b_a->size[0] * b_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_a, i20, (int)sizeof(double));
    idx = b_a->size[0];
    ii = b_a->size[1];
    ib = idx * ii;
    for (i20 = 0; i20 < ib; i20++) {
      b_a->data[i20] -= C->data[i20];
    }

    i20 = b->size[0] * b->size[1];
    b->size[0] = b_P->size[0];
    b->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
    ib = b_P->size[0] * b_P->size[1];
    for (i20 = 0; i20 < ib; i20++) {
      b->data[i20] = b_P->data[i20];
    }

    emxInit_real_T(&c_a, 2);
    if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
      i20 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = b_a->size[0];
      c_a->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)c_a, i20, (int)sizeof(double));
      ib = b_a->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b_P->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          c_a->data[i20 + c_a->size[0] * i21] = 0.0;
          ii = b_a->size[1];
          for (i22 = 0; i22 < ii; i22++) {
            c_a->data[i20 + c_a->size[0] * i21] += b_a->data[i20 + b_a->size[0] *
              i22] * b_P->data[i22 + b_P->size[0] * i21];
          }
        }
      }

      i20 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = c_a->size[0];
      b_P->size[1] = c_a->size[1];
      emxEnsureCapacity((emxArray__common *)b_P, i20, (int)sizeof(double));
      ib = c_a->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        idx = c_a->size[0];
        for (i21 = 0; i21 < idx; i21++) {
          b_P->data[i21 + b_P->size[0] * i20] = c_a->data[i21 + c_a->size[0] *
            i20];
        }
      }
    } else {
      k = b_a->size[1];
      a[0] = b_a->size[0];
      a[1] = b_P->size[1];
      m = b_a->size[0];
      i20 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = (int)a[0];
      b_P->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)b_P, i20, (int)sizeof(double));
      ib = (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        idx = (int)a[0];
        for (i21 = 0; i21 < idx; i21++) {
          b_P->data[i21 + b_P->size[0] * i20] = 0.0;
        }
      }

      if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_a->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            b_P->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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
  }

  // % D Partial EKF update using high-innovation inliers
  for (i20 = 0; i20 < loop_ub; i20++) {
    HI_inlierStatus_data[i20] = true;
  }

  //  high innovation inliers
  i20 = x_apo_prev->size[0];
  x_apo_prev->size[0] = b_P->size[0];
  emxEnsureCapacity((emxArray__common *)x_apo_prev, i20, (int)sizeof(double));
  ib = b_P->size[0];
  for (i20 = 0; i20 < ib; i20++) {
    x_apo_prev->data[i20] = 0.0;
  }

  i20 = K->size[0] * K->size[1];
  K->size[0] = 1;
  K->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)K, i20, (int)sizeof(double));
  K->data[0] = 0.0;

  //  for coder
  i20 = H->size[0] * H->size[1];
  H->size[0] = 1;
  H->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)H, i20, (int)sizeof(double));
  H->data[0] = 0.0;

  //  for coder
  i20 = (int)maxEKFIterations;
  it = 0;
  b_emxInit_real_T(&b_x_apo, 1);
  b_emxInit_real_T(&f_xt, 1);
  exitg1 = false;
  while ((!exitg1) && (it <= i20 - 1)) {
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
    for (i21 = 0; i21 < br; i21++) {
      b_indMeas_data[i21] = indMeas_data[ii_data[i21] - 1];
    }

    b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                 b_indMeas_size, map, (double)(numAnchors + 1),
                 numPointsPerAnchor, anchorInd, featureAnchorInd, b_m_vect,
                 imNoise, IMU_measurements, height_offset_pressure, b_r_data,
                 r_size, H, unusedU1_data, unusedU1_size, b_R_data, b_R_size);
    if (1.0 + (double)it == 1.0) {
      //  only do outlier rejection in first iteration
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i21 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
        ib = H->size[0];
        for (i21 = 0; i21 < ib; i21++) {
          idx = b_P->size[1];
          for (i22 = 0; i22 < idx; i22++) {
            y->data[i21 + y->size[0] * i22] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i21 + y->size[0] * i22] += H->data[i21 + H->size[0] * ar] *
                b_P->data[ar + b_P->size[0] * i22];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i21 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
        i21 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i21 = 0; i21 < ib; i21++) {
          y->data[i21] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i21 = idx + m;
            for (ic = idx; ic + 1 <= i21; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i21 = br + k;
            for (ib = br; ib + 1 <= i21; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i22 = idx + m;
                for (ic = idx; ic + 1 <= i22; ic++) {
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

      i21 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i21, (int)sizeof(double));
      ib = H->size[0];
      for (i21 = 0; i21 < ib; i21++) {
        idx = H->size[1];
        for (i22 = 0; i22 < idx; i22++) {
          b->data[i22 + b->size[0] * i21] = H->data[i21 + H->size[0] * i22];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i21 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = y->size[0];
        b_a->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_a, i21, (int)sizeof(double));
        ib = y->size[0];
        for (i21 = 0; i21 < ib; i21++) {
          idx = b->size[1];
          for (i22 = 0; i22 < idx; i22++) {
            b_a->data[i21 + b_a->size[0] * i22] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              b_a->data[i21 + b_a->size[0] * i22] += y->data[i21 + y->size[0] *
                ar] * b->data[ar + b->size[0] * i22];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i21 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)b_a, i21, (int)sizeof(double));
        i21 = b_a->size[0] * b_a->size[1];
        b_a->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)b_a, i21, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i21 = 0; i21 < ib; i21++) {
          b_a->data[i21] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i21 = idx + m;
            for (ic = idx; ic + 1 <= i21; ic++) {
              b_a->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i21 = br + k;
            for (ib = br; ib + 1 <= i21; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i22 = idx + m;
                for (ic = idx; ic + 1 <= i22; ic++) {
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
      for (i21 = 0; i21 < ib; i21++) {
        S_data[i21] = b_a->data[i21] + b_R_data[i21];
      }

      for (k = 0; k < numMeas; k++) {
        ar = k << 1;
        ii = k << 1;
        idx = k << 1;
        for (i21 = 0; i21 < 2; i21++) {
          r[i21] = b_r_data[i21 + ar];
          for (i22 = 0; i22 < 2; i22++) {
            d_xt[i22 + (i21 << 1)] = S_data[(i22 + ii) + br * (i21 + idx)];
          }
        }

        b_mrdivide(r, d_xt, a);
        ar = k << 1;
        for (i21 = 0; i21 < 2; i21++) {
          r[i21] = b_r_data[i21 + ar];
        }

        if (LI_inlierStatus_data[k]) {
          //  if this feature is a LI inlier, don't also do HI update with this feature 
          ar = k << 1;
          for (i21 = 0; i21 < 2; i21++) {
            b_r_data[i21 + ar] = 0.0;
          }

          ib = H->size[1];
          ar = k << 1;
          for (i21 = 0; i21 < ib; i21++) {
            for (i22 = 0; i22 < 2; i22++) {
              H->data[(i22 + ar) + H->size[0] * i21] = 0.0;
            }
          }

          HI_inlierStatus_data[k] = false;
        } else {
          //  otherwise check if HI inlier for both cams
          d_numStatesxt = 0.0;
          for (i21 = 0; i21 < 2; i21++) {
            d_numStatesxt += a[i21] * r[i21];
          }

          if (d_numStatesxt > 9.0) {
            ar = k << 1;
            for (i21 = 0; i21 < 2; i21++) {
              b_r_data[i21 + ar] = 0.0;
            }

            ib = H->size[1];
            ar = k << 1;
            for (i21 = 0; i21 < ib; i21++) {
              for (i22 = 0; i22 < 2; i22++) {
                H->data[(i22 + ar) + H->size[0] * i21] = 0.0;
              }
            }

            HI_inlierStatus_data[k] = false;
          }
        }
      }

      //          S = (H*P*H'+R);
      //      else
      //          for k = 1:numMeas
      //              if ~HI_inlierStatus(k)
      //                  r((k-1)*residualDim + (1:2)) = 0;
      //                  H((k-1)*residualDim + (1:2), :) = 0;
      //                  HI_inlierStatus(k) = false;
      //              end
      //          end
      //
      //          S = (H*P*H'+R);
    }

    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i21 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
      ib = H->size[0];
      for (i21 = 0; i21 < ib; i21++) {
        idx = b_P->size[1];
        for (i22 = 0; i22 < idx; i22++) {
          y->data[i21 + y->size[0] * i22] = 0.0;
          ii = H->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i21 + y->size[0] * i22] += H->data[i21 + H->size[0] * ar] *
              b_P->data[ar + b_P->size[0] * i22];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      m = H->size[0];
      i21 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
      i21 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i21 = 0; i21 < ib; i21++) {
        y->data[i21] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i21 = idx + m;
          for (ic = idx; ic + 1 <= i21; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i21 = br + k;
          for (ib = br; ib + 1 <= i21; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i22 = idx + m;
              for (ic = idx; ic + 1 <= i22; ic++) {
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

    i21 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i21, (int)sizeof(double));
    ib = H->size[0];
    for (i21 = 0; i21 < ib; i21++) {
      idx = H->size[1];
      for (i22 = 0; i22 < idx; i22++) {
        b->data[i22 + b->size[0] * i21] = H->data[i21 + H->size[0] * i22];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i21 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i21, (int)sizeof(double));
      ib = y->size[0];
      for (i21 = 0; i21 < ib; i21++) {
        idx = b->size[1];
        for (i22 = 0; i22 < idx; i22++) {
          C->data[i21 + C->size[0] * i22] = 0.0;
          ii = y->size[1];
          for (ar = 0; ar < ii; ar++) {
            C->data[i21 + C->size[0] * i22] += y->data[i21 + y->size[0] * ar] *
              b->data[ar + b->size[0] * i22];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i21 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i21, (int)sizeof(double));
      i21 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i21, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i21 = 0; i21 < ib; i21++) {
        C->data[i21] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i21 = idx + m;
          for (ic = idx; ic + 1 <= i21; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i21 = br + k;
          for (ib = br; ib + 1 <= i21; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i22 = idx + m;
              for (ic = idx; ic + 1 <= i22; ic++) {
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

    i21 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i21, (int)sizeof(double));
    ib = H->size[0];
    for (i21 = 0; i21 < ib; i21++) {
      idx = H->size[1];
      for (i22 = 0; i22 < idx; i22++) {
        b->data[i22 + b->size[0] * i21] = H->data[i21 + H->size[0] * i22];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i21 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
      ib = b_P->size[0];
      for (i21 = 0; i21 < ib; i21++) {
        idx = b->size[1];
        for (i22 = 0; i22 < idx; i22++) {
          y->data[i21 + y->size[0] * i22] = 0.0;
          ii = b_P->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i21 + y->size[0] * i22] += b_P->data[i21 + b_P->size[0] * ar]
              * b->data[ar + b->size[0] * i22];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i21 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
      i21 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i21, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i21 = 0; i21 < ib; i21++) {
        y->data[i21] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i21 = idx + m;
          for (ic = idx; ic + 1 <= i21; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i21 = br + k;
          for (ib = br; ib + 1 <= i21; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i22 = idx + m;
              for (ic = idx; ic + 1 <= i22; ic++) {
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
    for (i21 = 0; i21 < ib; i21++) {
      b_C_data[i21] = C->data[i21] + b_R_data[i21];
    }

    mrdivide(y, b_C_data, c_C_size, K);
    if ((K->size[1] == 1) || (r_size[0] == 1)) {
      i21 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i21, (int)sizeof(double));
      ib = K->size[0];
      for (i21 = 0; i21 < ib; i21++) {
        x_apo->data[i21] = 0.0;
        idx = K->size[1];
        for (i22 = 0; i22 < idx; i22++) {
          x_apo->data[i21] += K->data[i21 + K->size[0] * i22] * b_r_data[i22];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      m = K->size[0];
      i21 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i21, (int)sizeof(double));
      ib = (int)a[0];
      for (i21 = 0; i21 < ib; i21++) {
        x_apo->data[i21] = 0.0;
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
          i21 = br + k;
          for (ib = br; ib + 1 <= i21; ib++) {
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

    for (i21 = 0; i21 < 3; i21++) {
      c_xt[i21] = b_xt->data[i21] + x_apo->data[i21];
    }

    for (i21 = 0; i21 < 3; i21++) {
      b_xt->data[i21] = c_xt[i21];
    }

    for (i21 = 0; i21 < 3; i21++) {
      c_xt[i21] = x_apo->data[3 + i21];
    }

    quatPlusThetaJ(c_xt, dv54);
    quatmultJ(dv54, *(double (*)[4])&b_xt->data[3], dv50);
    for (i21 = 0; i21 < 4; i21++) {
      b_xt->data[3 + i21] = dv50[i21];
    }

    if (8.0 > c_numStatesxt) {
      i21 = 0;
      i22 = 0;
    } else {
      i21 = 7;
      i22 = (int)c_numStatesxt;
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

    ii = r13->size[0] * r13->size[1];
    r13->size[0] = 1;
    r13->size[1] = br - idx;
    emxEnsureCapacity((emxArray__common *)r13, ii, (int)sizeof(int));
    ib = br - idx;
    for (br = 0; br < ib; br++) {
      r13->data[r13->size[0] * br] = idx + br;
    }

    idx = f_xt->size[0];
    f_xt->size[0] = i22 - i21;
    emxEnsureCapacity((emxArray__common *)f_xt, idx, (int)sizeof(double));
    ib = i22 - i21;
    for (i22 = 0; i22 < ib; i22++) {
      f_xt->data[i22] = b_xt->data[i21 + i22] + x_apo->data[ar + i22];
    }

    ib = r13->size[1];
    for (i21 = 0; i21 < ib; i21++) {
      b_xt->data[r13->data[r13->size[0] * i21]] = f_xt->data[(*(int (*)[2])
        r13->size)[0] * i21];
    }

    for (br = 0; br <= numAnchors; br++) {
      for (i21 = 0; i21 < 16; i21++) {
        x[i21] = (b_anchorFeatures->data[i21 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i21 = 0; i21 < 3; i21++) {
          c_xt[i21] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i21)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i21)) - 1];
        }

        for (i21 = 0; i21 < 3; i21++) {
          b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i21)) - 1] = c_xt[i21];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i21 = 0; i21 < 3; i21++) {
          c_xt[i21] = x_apo->data[(int)(d_numStates + (4.0 + (double)i21)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i21 = 0; i21 < 4; i21++) {
          d_xt[i21] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i21)) - 1];
        }

        quatPlusThetaJ(c_xt, dv55);
        quatmultJ(dv55, d_xt, dv50);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i21 = 0; i21 < 4; i21++) {
          b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i21)) - 1] = dv50[i21];
        }

        for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) + (1.0 + (double)idx)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)idx)) - 1];
        }
      }
    }

    i21 = b_x_apo->size[0];
    b_x_apo->size[0] = x_apo->size[0];
    emxEnsureCapacity((emxArray__common *)b_x_apo, i21, (int)sizeof(double));
    ib = x_apo->size[0];
    for (i21 = 0; i21 < ib; i21++) {
      b_x_apo->data[i21] = x_apo->data[i21] - x_apo_prev->data[i21];
    }

    if (d_norm(b_x_apo) < 0.001) {
      exitg1 = true;
    } else {
      i21 = x_apo_prev->size[0];
      x_apo_prev->size[0] = x_apo->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo_prev, i21, (int)sizeof(double));
      ib = x_apo->size[0];
      for (i21 = 0; i21 < ib; i21++) {
        x_apo_prev->data[i21] = x_apo->data[i21];
      }

      it++;
    }
  }

  emxFree_real_T(&f_xt);
  emxFree_real_T(&b_x_apo);
  emxFree_real_T(&y);
  emxFree_int32_T(&r13);
  emxFree_real_T(&featureAnchorInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&map);
  emxFree_real_T(&x_apo_prev);
  emxFree_real_T(&x_apo);
  b_eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
         numPointsPerAnchor), b_a);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i20 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
    loop_ub = K->size[0];
    for (i20 = 0; i20 < loop_ub; i20++) {
      ib = H->size[1];
      for (i21 = 0; i21 < ib; i21++) {
        C->data[i20 + C->size[0] * i21] = 0.0;
        idx = K->size[1];
        for (i22 = 0; i22 < idx; i22++) {
          C->data[i20 + C->size[0] * i21] += K->data[i20 + K->size[0] * i22] *
            H->data[i22 + H->size[0] * i21];
        }
      }
    }
  } else {
    k = K->size[1];
    a[0] = (unsigned int)K->size[0];
    a[1] = (unsigned int)H->size[1];
    m = K->size[0];
    i20 = C->size[0] * C->size[1];
    C->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
    i20 = C->size[0] * C->size[1];
    C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
    loop_ub = (int)a[0] * (int)a[1];
    for (i20 = 0; i20 < loop_ub; i20++) {
      C->data[i20] = 0.0;
    }

    if ((K->size[0] == 0) || (H->size[1] == 0)) {
    } else {
      ii = K->size[0] * (H->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i20 = idx + m;
        for (ic = idx; ic + 1 <= i20; ic++) {
          C->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i20 = br + k;
        for (ib = br; ib + 1 <= i20; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            i21 = idx + m;
            for (ic = idx; ic + 1 <= i21; ic++) {
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
  i20 = b_a->size[0] * b_a->size[1];
  emxEnsureCapacity((emxArray__common *)b_a, i20, (int)sizeof(double));
  idx = b_a->size[0];
  ii = b_a->size[1];
  loop_ub = idx * ii;
  for (i20 = 0; i20 < loop_ub; i20++) {
    b_a->data[i20] -= C->data[i20];
  }

  emxFree_real_T(&C);
  i20 = b->size[0] * b->size[1];
  b->size[0] = b_P->size[0];
  b->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
  loop_ub = b_P->size[0] * b_P->size[1];
  for (i20 = 0; i20 < loop_ub; i20++) {
    b->data[i20] = b_P->data[i20];
  }

  emxInit_real_T(&d_a, 2);
  if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
    i20 = d_a->size[0] * d_a->size[1];
    d_a->size[0] = b_a->size[0];
    d_a->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)d_a, i20, (int)sizeof(double));
    loop_ub = b_a->size[0];
    for (i20 = 0; i20 < loop_ub; i20++) {
      ib = b_P->size[1];
      for (i21 = 0; i21 < ib; i21++) {
        d_a->data[i20 + d_a->size[0] * i21] = 0.0;
        idx = b_a->size[1];
        for (i22 = 0; i22 < idx; i22++) {
          d_a->data[i20 + d_a->size[0] * i21] += b_a->data[i20 + b_a->size[0] *
            i22] * b_P->data[i22 + b_P->size[0] * i21];
        }
      }
    }

    i20 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = d_a->size[0];
    b_P->size[1] = d_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i20, (int)sizeof(double));
    loop_ub = d_a->size[1];
    for (i20 = 0; i20 < loop_ub; i20++) {
      ib = d_a->size[0];
      for (i21 = 0; i21 < ib; i21++) {
        b_P->data[i21 + b_P->size[0] * i20] = d_a->data[i21 + d_a->size[0] * i20];
      }
    }
  } else {
    k = b_a->size[1];
    a[0] = b_a->size[0];
    a[1] = b_P->size[1];
    m = b_a->size[0];
    i20 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = (int)a[0];
    b_P->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)b_P, i20, (int)sizeof(double));
    loop_ub = (int)a[1];
    for (i20 = 0; i20 < loop_ub; i20++) {
      ib = (int)a[0];
      for (i21 = 0; i21 < ib; i21++) {
        b_P->data[i21 + b_P->size[0] * i20] = 0.0;
      }
    }

    if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = b_a->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i20 = idx + m;
        for (ic = idx; ic + 1 <= i20; ic++) {
          b_P->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i20 = br + k;
        for (ib = br; ib + 1 <= i20; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i21 = idx + m;
            for (ic = idx; ic + 1 <= i21; ic++) {
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
  emxFree_real_T(&b_a);
  emxFree_real_T(&b);
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
}

//
// File trailer for OnePointRANSAC_EKF.cpp
//
// [EOF]
//
