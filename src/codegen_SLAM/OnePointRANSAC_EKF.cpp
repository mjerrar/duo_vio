//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Jul-2015 14:58:50
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
//                const double z_all_l[64]
//                double c_numStatesxt
//                double c_numStates
//                double numPointsPerAnchor
//                const emxArray_real_T *b_anchorFeatures
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                double validFeatures_data[]
//                int validFeatures_size[1]
// Return Type  : void
//
void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P, const
  double z_all_l[64], double c_numStatesxt, double c_numStates, double
  numPointsPerAnchor, const emxArray_real_T *b_anchorFeatures, const
  emxArray_real_T *b_m_vect, const double imNoise[2], double validFeatures_data[],
  int validFeatures_size[1])
{
  emxArray_boolean_T *c_anchorFeatures;
  int i19;
  int loop_ub;
  boolean_T x[32];
  int idx;
  int ii_data[32];
  int ii;
  boolean_T exitg3;
  boolean_T guard2 = false;
  int ar;
  signed char indMeas_data[32];
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
  emxArray_int32_T *r13;
  emxArray_real_T *y;
  emxArray_real_T *b;
  emxArray_int32_T *r14;
  emxArray_int32_T *r15;
  emxArray_real_T *b_x_apo_prev;
  double d_numStatesxt;
  double R[4];
  double unusedU0[2];
  double r[2];
  int ib;
  int i20;
  int i21;
  int k;
  double a[2];
  int ic;
  int br;
  int ia;
  double b_y[4];
  int m;
  double c_xt[4];
  double d_xt[3];
  double dv328[4];
  double dv329[4];
  double e_numStatesxt;
  double d_numStates;
  double dv330[4];
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
  static const signed char iv15[3] = { 0, 1, 2 };

  double b_z_all_l[2];
  static const signed char iv16[2] = { 1, 2 };

  emxArray_real_T *K;
  emxArray_real_T *H;
  emxArray_real_T *C;
  emxArray_real_T *c_y;
  emxArray_real_T *b_b;
  emxArray_real_T *b_a;
  double b_indMeas_data[32];
  int indMeas_size[1];
  int R_size[2];
  double R_data[4096];
  int unusedU1_size[1];
  double unusedU1_data[64];
  int r_size[1];
  double r_data[64];
  double C_data[4096];
  int C_size[2];
  double dv331[4];
  emxArray_real_T *e_xt;
  double dv332[4];
  emxArray_real_T *c_a;
  int it;
  emxArray_real_T *b_x_apo;
  emxArray_real_T *f_xt;
  boolean_T exitg1;
  int b_indMeas_size[1];
  int S_size[2];
  double S_data[4096];
  double dv333[4];
  double dv334[4];
  emxArray_real_T *d_a;
  emxInit_boolean_T(&c_anchorFeatures, 2);

  //  threshold for LI innovation test
  // 9.487729036781154; % HI mahalanobis gate
  //  threshold for minimum LI supporter
  numAnchors = b_anchorFeatures->size[1];
  i19 = c_anchorFeatures->size[0] * c_anchorFeatures->size[1];
  c_anchorFeatures->size[0] = 32;
  c_anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)c_anchorFeatures, i19, (int)sizeof
                    (boolean_T));
  loop_ub = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i19 = 0; i19 < loop_ub; i19++) {
    c_anchorFeatures->data[i19] = (b_anchorFeatures->data[i19] == 1.0);
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

  for (i19 = 0; i19 < loop_ub; i19++) {
    indMeas_data[i19] = (signed char)ii_data[i19];
  }

  numMeas = ar;
  numStatesPerAnchor = 6.0 + numPointsPerAnchor;

  // % B 1-point hypotheses generation and evaluation
  n_hyp = 100.0;
  LI_inlierStatus_size_idx_0 = ar;
  for (i19 = 0; i19 < ar; i19++) {
    LI_inlierStatus_data[i19] = false;
  }

  emxInit_real_T(&map, 2);
  b_emxInit_real_T(&anchorInd, 1);
  b_emxInit_real_T(&featureAnchorInd, 1);

  //  low innovation inlier status
  //  build the map according to the current estimate
  getMap(b_xt, b_anchorFeatures, b_m_vect, numAnchors * numPointsPerAnchor,
         c_numStatesxt, 7.0 + numPointsPerAnchor, map, anchorInd,
         featureAnchorInd);
  i = 1.0;
  emxInit_real_T(&K_i, 2);
  b_emxInit_real_T(&x_apo, 1);
  b_emxInit_real_T(&x_apo_prev, 1);
  emxInit_real_T(&H_i, 2);
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
    //      [r, H_xc, ~] = getJacobianAndResidual(xt, numStates, z_all_l, idx, map, cameraparams);  % with respect to Cam states 
    //      H_xm = getAnchorStateJacobian(xt, numStatesxt, numAnchors, numPointsPerAnchor, idx, ... % whih respect to Anchor states 
    //          map, cameraparams, anchorInd, featureAnchorInd, m_vect);
    getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, (double)indMeas_data
               [(int)(1.0 + floor(d_numStatesxt * (double)numMeas)) - 1], map,
               numAnchors, numPointsPerAnchor, anchorInd, featureAnchorInd,
               b_m_vect, imNoise, r, H_i, unusedU0, R);

    //  H = [H_xc, H_xm];
    if ((H_i->size[1] == 1) || (b_P->size[0] == 1)) {
      i19 = y->size[0] * y->size[1];
      y->size[0] = 2;
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      for (i19 = 0; i19 < 2; i19++) {
        ib = b_P->size[1];
        for (i20 = 0; i20 < ib; i20++) {
          y->data[i19 + y->size[0] * i20] = 0.0;
          idx = H_i->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            y->data[i19 + y->size[0] * i20] += H_i->data[i19 + H_i->size[0] *
              i21] * b_P->data[i21 + b_P->size[0] * i20];
          }
        }
      }
    } else {
      k = H_i->size[1];
      a[1] = b_P->size[1];
      i19 = y->size[0] * y->size[1];
      y->size[0] = 2;
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      i19 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ib = (int)a[1] << 1;
      for (i19 = 0; i19 < ib; i19++) {
        y->data[i19] = 0.0;
      }

      if (b_P->size[1] == 0) {
      } else {
        ii = (b_P->size[1] - 1) << 1;
        for (idx = 0; idx <= ii; idx += 2) {
          for (ic = idx; ic + 1 <= idx + 2; ic++) {
            y->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += 2) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 2; ic++) {
                ia++;
                y->data[ic] += b_P->data[ib] * H_i->data[ia - 1];
              }
            }

            ar += 2;
          }

          br += k;
        }
      }
    }

    i19 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
    for (i19 = 0; i19 < 2; i19++) {
      ib = H_i->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        b->data[i20 + b->size[0] * i19] = H_i->data[i19 + H_i->size[0] * i20];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      for (i19 = 0; i19 < 2; i19++) {
        for (i20 = 0; i20 < 2; i20++) {
          b_y[i19 + (i20 << 1)] = 0.0;
          ib = y->size[1];
          for (i21 = 0; i21 < ib; i21++) {
            b_y[i19 + (i20 << 1)] += y->data[i19 + y->size[0] * i21] * b->
              data[i21 + b->size[0] * i20];
          }
        }
      }
    } else {
      k = y->size[1];
      for (i19 = 0; i19 < 4; i19++) {
        b_y[i19] = 0.0;
      }

      for (idx = 0; idx < 4; idx += 2) {
        for (ic = idx; ic + 1 <= idx + 2; ic++) {
          b_y[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx < 4; idx += 2) {
        ar = 0;
        i19 = br + k;
        for (ib = br; ib + 1 <= i19; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            for (ic = idx; ic + 1 <= idx + 2; ic++) {
              ia++;
              b_y[ic] += b->data[ib] * y->data[ia - 1];
            }
          }

          ar += 2;
        }

        br += k;
      }
    }

    i19 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
    for (i19 = 0; i19 < 2; i19++) {
      ib = H_i->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        b->data[i20 + b->size[0] * i19] = H_i->data[i19 + H_i->size[0] * i20];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i19 = K_i->size[0] * K_i->size[1];
      K_i->size[0] = b_P->size[0];
      K_i->size[1] = 2;
      emxEnsureCapacity((emxArray__common *)K_i, i19, (int)sizeof(double));
      ib = b_P->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        for (i20 = 0; i20 < 2; i20++) {
          K_i->data[i19 + K_i->size[0] * i20] = 0.0;
          idx = b_P->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            K_i->data[i19 + K_i->size[0] * i20] += b_P->data[i19 + b_P->size[0] *
              i21] * b->data[i21 + b->size[0] * i20];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      m = b_P->size[0];
      i19 = K_i->size[0] * K_i->size[1];
      K_i->size[0] = (int)a[0];
      K_i->size[1] = 2;
      emxEnsureCapacity((emxArray__common *)K_i, i19, (int)sizeof(double));
      ib = (int)a[0] << 1;
      for (i19 = 0; i19 < ib; i19++) {
        K_i->data[i19] = 0.0;
      }

      if (b_P->size[0] == 0) {
      } else {
        idx = 0;
        while ((m > 0) && (idx <= m)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            K_i->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= m)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
                ia++;
                K_i->data[ic] += b->data[ib] * b_P->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    for (i19 = 0; i19 < 4; i19++) {
      c_xt[i19] = b_y[i19] + R[i19];
    }

    c_mrdivide(K_i, c_xt);
    a[0] = K_i->size[0];
    m = K_i->size[0];
    i19 = x_apo->size[0];
    x_apo->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
    ib = (int)a[0];
    for (i19 = 0; i19 < ib; i19++) {
      x_apo->data[i19] = 0.0;
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
        for (ib = br; ib + 1 <= br + 2; ib++) {
          if (r[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= m; ic++) {
              ia++;
              x_apo->data[ic] += r[ib] * K_i->data[ia - 1];
            }
          }

          ar += m;
        }

        br += 2;
        idx = m;
      }
    }

    i19 = x_apo_prev->size[0];
    x_apo_prev->size[0] = b_xt->size[0];
    emxEnsureCapacity((emxArray__common *)x_apo_prev, i19, (int)sizeof(double));
    ib = b_xt->size[0];
    for (i19 = 0; i19 < ib; i19++) {
      x_apo_prev->data[i19] = b_xt->data[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      x_apo_prev->data[i19] = b_xt->data[i19] + x_apo->data[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      d_xt[i19] = x_apo->data[3 + i19];
    }

    quatPlusThetaJ(d_xt, dv328);
    quatmultJ(dv328, *(double (*)[4])&x_apo_prev->data[3], dv329);
    for (i19 = 0; i19 < 4; i19++) {
      x_apo_prev->data[3 + i19] = dv329[i19];
    }

    if (8.0 > c_numStatesxt) {
      i19 = 1;
      i20 = 0;
    } else {
      i19 = 8;
      i20 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      i21 = 1;
      ar = 0;
    } else {
      i21 = 7;
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
    r14->size[0] = (i20 - i19) + 1;
    emxEnsureCapacity((emxArray__common *)r14, idx, (int)sizeof(int));
    ib = i20 - i19;
    for (i20 = 0; i20 <= ib; i20++) {
      r14->data[i20] = i19 + i20;
    }

    i19 = r15->size[0];
    r15->size[0] = (ar - i21) + 1;
    emxEnsureCapacity((emxArray__common *)r15, i19, (int)sizeof(int));
    ib = ar - i21;
    for (i19 = 0; i19 <= ib; i19++) {
      r15->data[i19] = i21 + i19;
    }

    i19 = b_x_apo_prev->size[0];
    b_x_apo_prev->size[0] = r13->size[0] * r13->size[1];
    emxEnsureCapacity((emxArray__common *)b_x_apo_prev, i19, (int)sizeof(double));
    ib = r13->size[0] * r13->size[1];
    for (i19 = 0; i19 < ib; i19++) {
      b_x_apo_prev->data[i19] = x_apo_prev->data[r14->data[i19] - 1] +
        x_apo->data[r15->data[i19] - 1];
    }

    ib = b_x_apo_prev->size[0];
    for (i19 = 0; i19 < ib; i19++) {
      x_apo_prev->data[r13->data[i19]] = b_x_apo_prev->data[i19];
    }

    for (br = 0; br < (int)numAnchors; br++) {
      for (i19 = 0; i19 < 32; i19++) {
        x[i19] = (b_anchorFeatures->data[i19 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) *
          numStatesPerAnchor;
        for (i19 = 0; i19 < 3; i19++) {
          d_xt[i19] = x_apo_prev->data[(int)(d_numStatesxt + (1.0 + (double)i19))
            - 1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i19)) - 1];
        }

        for (i19 = 0; i19 < 3; i19++) {
          x_apo_prev->data[(int)(e_numStatesxt + (1.0 + (double)i19)) - 1] =
            d_xt[i19];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) *
          numStatesPerAnchor;
        for (i19 = 0; i19 < 3; i19++) {
          d_xt[i19] = x_apo->data[(int)(d_numStates + (4.0 + (double)i19)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          c_xt[i19] = x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i19))
            - 1];
        }

        quatPlusThetaJ(d_xt, dv330);
        quatmultJ(dv330, c_xt, dv329);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i19)) - 1] =
            dv329[i19];
        }

        for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
          x_apo_prev->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) + (1.0 + (double)idx)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) *
                                numStatesPerAnchor) + 6.0) + (1.0 + (double)idx))
            - 1];
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
    for (i19 = 0; i19 < numMeas; i19++) {
      HI_inlierStatus_data[i19] = false;
    }

    //  inliers of this iteration
    indMeasIdx = 0;
    for (br = 0; br < (int)numAnchors; br++) {
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

      for (i19 = 0; i19 < ib; i19++) {
        featureIdxVect_data[i19] = (signed char)ii_data[i19];
      }

      for (idx = 0; idx < ar; idx++) {
        if (b_anchorFeatures->data[(featureIdxVect_data[idx] +
             b_anchorFeatures->size[0] * br) - 1] == 1.0) {
          //  if this is not a lost feature
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i19 = 0; i19 < 3; i19++) {
            anchorPos[i19] = x_apo_prev->data[(int)(e_numStatesxt + (1.0 +
              (double)i19)) - 1];
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
          for (i19 = 0; i19 < 3; i19++) {
            d_numStatesxt = 0.0;
            for (i20 = 0; i20 < 3; i20++) {
              d_numStatesxt += c_x_apo_prev[i19 + 3 * i20] * b_m_vect->data[i20
                + b_m_vect->size[0] * (featureIdxVect_data[idx] - 1)];
            }

            b_anchorPos[i19] = (anchorPos[i19] + d_numStatesxt / d_numStates) -
              x_apo_prev->data[iv15[i19]];
          }

          for (i19 = 0; i19 < 3; i19++) {
            d_xt[i19] = 0.0;
            for (i20 = 0; i20 < 3; i20++) {
              d_xt[i19] += R_cw[i19 + 3 * i20] * b_anchorPos[i20];
            }
          }

          predictMeasurement_left(d_xt, r);
          ii = (featureIdxVect_data[idx] - 1) * 2;
          for (i19 = 0; i19 < 2; i19++) {
            b_z_all_l[i19] = z_all_l[(ii + iv16[i19]) - 1] - r[i19];
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
      for (i19 = 0; i19 < numMeas; i19++) {
        LI_inlierStatus_data[i19] = HI_inlierStatus_data[i19];
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
  emxFree_real_T(&b);
  emxFree_real_T(&y);
  emxFree_real_T(&H_i);
  emxFree_real_T(&K_i);
  idx = 0;
  for (k = 0; k < LI_inlierStatus_size_idx_0; k++) {
    if (LI_inlierStatus_data[k]) {
      idx++;
    }
  }

  emxInit_real_T(&K, 2);
  emxInit_real_T(&H, 2);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&c_y, 2);
  emxInit_real_T(&b_b, 2);
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
    for (i19 = 0; i19 < br; i19++) {
      b_indMeas_data[i19] = indMeas_data[ii_data[i19] - 1];
    }

    b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                 indMeas_size, map, numAnchors, numPointsPerAnchor, anchorInd,
                 featureAnchorInd, b_m_vect, imNoise, r_data, r_size, H,
                 unusedU1_data, unusedU1_size, R_data, R_size);
    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = H->size[0];
      c_y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int)sizeof(double));
      ib = H->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = b_P->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          c_y->data[i19 + c_y->size[0] * i20] = 0.0;
          ii = H->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            c_y->data[i19 + c_y->size[0] * i20] += H->data[i19 + H->size[0] *
              i21] * b_P->data[i21 + b_P->size[0] * i20];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      m = H->size[0];
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int)sizeof(double));
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ib; i19++) {
        c_y->data[i19] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            c_y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
                ia++;
                c_y->data[ic] += b_P->data[ib] * H->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    i19 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = H->size[1];
    b_b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i19, (int)sizeof(double));
    ib = H->size[0];
    for (i19 = 0; i19 < ib; i19++) {
      idx = H->size[1];
      for (i20 = 0; i20 < idx; i20++) {
        b_b->data[i20 + b_b->size[0] * i19] = H->data[i19 + H->size[0] * i20];
      }
    }

    if ((c_y->size[1] == 1) || (b_b->size[0] == 1)) {
      i19 = C->size[0] * C->size[1];
      C->size[0] = c_y->size[0];
      C->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ib = c_y->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = b_b->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          C->data[i19 + C->size[0] * i20] = 0.0;
          ii = c_y->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            C->data[i19 + C->size[0] * i20] += c_y->data[i19 + c_y->size[0] *
              i21] * b_b->data[i21 + b_b->size[0] * i20];
          }
        }
      }
    } else {
      k = c_y->size[1];
      a[0] = (signed char)c_y->size[0];
      a[1] = (signed char)b_b->size[1];
      m = c_y->size[0];
      i19 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      i19 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i19 = 0; i19 < ib; i19++) {
        C->data[i19] = 0.0;
      }

      if ((c_y->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        ii = c_y->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
                ia++;
                C->data[ic] += b_b->data[ib] * c_y->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    i19 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = H->size[1];
    b_b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i19, (int)sizeof(double));
    ib = H->size[0];
    for (i19 = 0; i19 < ib; i19++) {
      idx = H->size[1];
      for (i20 = 0; i20 < idx; i20++) {
        b_b->data[i20 + b_b->size[0] * i19] = H->data[i19 + H->size[0] * i20];
      }
    }

    if ((b_P->size[1] == 1) || (b_b->size[0] == 1)) {
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = b_P->size[0];
      c_y->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int)sizeof(double));
      ib = b_P->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = b_b->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          c_y->data[i19 + c_y->size[0] * i20] = 0.0;
          ii = b_P->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            c_y->data[i19 + c_y->size[0] * i20] += b_P->data[i19 + b_P->size[0] *
              i21] * b_b->data[i21 + b_b->size[0] * i20];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b_b->size[1];
      m = b_P->size[0];
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int)sizeof(double));
      i19 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)c_y, i19, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ib; i19++) {
        c_y->data[i19] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            c_y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
                ia++;
                c_y->data[ic] += b_b->data[ib] * b_P->data[ia - 1];
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
    for (i19 = 0; i19 < ib; i19++) {
      C_data[i19] = C->data[i19] + R_data[i19];
    }

    mrdivide(c_y, C_data, C_size, K);
    if ((K->size[1] == 1) || (r_size[0] == 1)) {
      i19 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
      ib = K->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        x_apo->data[i19] = 0.0;
        idx = K->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          x_apo->data[i19] += K->data[i19 + K->size[0] * i20] * r_data[i20];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      m = K->size[0];
      i19 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
      ib = (int)a[0];
      for (i19 = 0; i19 < ib; i19++) {
        x_apo->data[i19] = 0.0;
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
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
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

    for (i19 = 0; i19 < 3; i19++) {
      d_xt[i19] = b_xt->data[i19] + x_apo->data[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      b_xt->data[i19] = d_xt[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      d_xt[i19] = x_apo->data[3 + i19];
    }

    quatPlusThetaJ(d_xt, dv331);
    quatmultJ(dv331, *(double (*)[4])&b_xt->data[3], dv329);
    for (i19 = 0; i19 < 4; i19++) {
      b_xt->data[3 + i19] = dv329[i19];
    }

    if (8.0 > c_numStatesxt) {
      i19 = 0;
      i20 = 0;
    } else {
      i19 = 7;
      i20 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      i21 = 0;
    } else {
      i21 = 6;
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
    e_xt->size[0] = i20 - i19;
    emxEnsureCapacity((emxArray__common *)e_xt, ar, (int)sizeof(double));
    ib = i20 - i19;
    for (i20 = 0; i20 < ib; i20++) {
      e_xt->data[i20] = b_xt->data[i19 + i20] + x_apo->data[i21 + i20];
    }

    ib = r13->size[1];
    for (i19 = 0; i19 < ib; i19++) {
      b_xt->data[r13->data[r13->size[0] * i19]] = e_xt->data[(*(int (*)[2])
        r13->size)[0] * i19];
    }

    emxFree_real_T(&e_xt);
    for (br = 0; br < (int)numAnchors; br++) {
      for (i19 = 0; i19 < 32; i19++) {
        x[i19] = (b_anchorFeatures->data[i19 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) *
          numStatesPerAnchor;
        for (i19 = 0; i19 < 3; i19++) {
          d_xt[i19] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i19)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i19)) - 1];
        }

        for (i19 = 0; i19 < 3; i19++) {
          b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i19)) - 1] = d_xt[i19];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) *
          numStatesPerAnchor;
        for (i19 = 0; i19 < 3; i19++) {
          d_xt[i19] = x_apo->data[(int)(d_numStates + (4.0 + (double)i19)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          c_xt[i19] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i19)) - 1];
        }

        quatPlusThetaJ(d_xt, dv332);
        quatmultJ(dv332, c_xt, dv329);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i19)) - 1] = dv329[i19];
        }

        for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) + (1.0 + (double)idx)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) *
                                numStatesPerAnchor) + 6.0) + (1.0 + (double)idx))
            - 1];
        }
      }
    }

    b_eye(c_numStates + numAnchors * numStatesPerAnchor, b_a);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i19 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ib = K->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = H->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          C->data[i19 + C->size[0] * i20] = 0.0;
          ii = K->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            C->data[i19 + C->size[0] * i20] += K->data[i19 + K->size[0] * i21] *
              H->data[i21 + H->size[0] * i20];
          }
        }
      }
    } else {
      k = K->size[1];
      a[0] = (unsigned int)K->size[0];
      a[1] = (unsigned int)H->size[1];
      m = K->size[0];
      i19 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      i19 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ib; i19++) {
        C->data[i19] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
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

    i19 = b_a->size[0] * b_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_a, i19, (int)sizeof(double));
    idx = b_a->size[0];
    ii = b_a->size[1];
    ib = idx * ii;
    for (i19 = 0; i19 < ib; i19++) {
      b_a->data[i19] -= C->data[i19];
    }

    i19 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = b_P->size[0];
    b_b->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)b_b, i19, (int)sizeof(double));
    ib = b_P->size[0] * b_P->size[1];
    for (i19 = 0; i19 < ib; i19++) {
      b_b->data[i19] = b_P->data[i19];
    }

    emxInit_real_T(&c_a, 2);
    if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
      i19 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = b_a->size[0];
      c_a->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)c_a, i19, (int)sizeof(double));
      ib = b_a->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = b_P->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          c_a->data[i19 + c_a->size[0] * i20] = 0.0;
          ii = b_a->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            c_a->data[i19 + c_a->size[0] * i20] += b_a->data[i19 + b_a->size[0] *
              i21] * b_P->data[i21 + b_P->size[0] * i20];
          }
        }
      }

      i19 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = c_a->size[0];
      b_P->size[1] = c_a->size[1];
      emxEnsureCapacity((emxArray__common *)b_P, i19, (int)sizeof(double));
      ib = c_a->size[1];
      for (i19 = 0; i19 < ib; i19++) {
        idx = c_a->size[0];
        for (i20 = 0; i20 < idx; i20++) {
          b_P->data[i20 + b_P->size[0] * i19] = c_a->data[i20 + c_a->size[0] *
            i19];
        }
      }
    } else {
      k = b_a->size[1];
      a[0] = b_a->size[0];
      a[1] = b_P->size[1];
      m = b_a->size[0];
      i19 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = (int)a[0];
      b_P->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)b_P, i19, (int)sizeof(double));
      ib = (int)a[1];
      for (i19 = 0; i19 < ib; i19++) {
        idx = (int)a[0];
        for (i20 = 0; i20 < idx; i20++) {
          b_P->data[i20 + b_P->size[0] * i19] = 0.0;
        }
      }

      if ((b_a->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        ii = b_a->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            b_P->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
                ia++;
                b_P->data[ic] += b_b->data[ib] * b_a->data[ia - 1];
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
  for (i19 = 0; i19 < loop_ub; i19++) {
    HI_inlierStatus_data[i19] = true;
  }

  //  high innovation inliers
  i19 = x_apo_prev->size[0];
  x_apo_prev->size[0] = b_P->size[0];
  emxEnsureCapacity((emxArray__common *)x_apo_prev, i19, (int)sizeof(double));
  ib = b_P->size[0];
  for (i19 = 0; i19 < ib; i19++) {
    x_apo_prev->data[i19] = 0.0;
  }

  i19 = K->size[0] * K->size[1];
  K->size[0] = 1;
  K->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)K, i19, (int)sizeof(double));
  K->data[0] = 0.0;

  //  for coder
  i19 = H->size[0] * H->size[1];
  H->size[0] = 1;
  H->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)H, i19, (int)sizeof(double));
  H->data[0] = 0.0;

  //  for coder
  i19 = (int)maxEKFIterations;
  it = 0;
  b_emxInit_real_T(&b_x_apo, 1);
  b_emxInit_real_T(&f_xt, 1);
  exitg1 = false;
  while ((!exitg1) && (it <= i19 - 1)) {
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
    for (i20 = 0; i20 < br; i20++) {
      b_indMeas_data[i20] = indMeas_data[ii_data[i20] - 1];
    }

    b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                 b_indMeas_size, map, numAnchors, numPointsPerAnchor, anchorInd,
                 featureAnchorInd, b_m_vect, imNoise, r_data, r_size, H,
                 unusedU1_data, unusedU1_size, R_data, R_size);
    if (1.0 + (double)it == 1.0) {
      //  only do outlier rejection in first iteration
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = H->size[0];
        c_y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        ib = H->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b_P->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            c_y->data[i20 + c_y->size[0] * i21] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              c_y->data[i20 + c_y->size[0] * i21] += H->data[i20 + H->size[0] *
                ar] * b_P->data[ar + b_P->size[0] * i21];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i20 = 0; i20 < ib; i20++) {
          c_y->data[i20] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              c_y->data[ic] = 0.0;
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
                  c_y->data[ic] += b_P->data[ib] * H->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      i20 = b_b->size[0] * b_b->size[1];
      b_b->size[0] = H->size[1];
      b_b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b_b, i20, (int)sizeof(double));
      ib = H->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = H->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          b_b->data[i21 + b_b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
        }
      }

      if ((c_y->size[1] == 1) || (b_b->size[0] == 1)) {
        i20 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = c_y->size[0];
        b_a->size[1] = b_b->size[1];
        emxEnsureCapacity((emxArray__common *)b_a, i20, (int)sizeof(double));
        ib = c_y->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b_b->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            b_a->data[i20 + b_a->size[0] * i21] = 0.0;
            ii = c_y->size[1];
            for (ar = 0; ar < ii; ar++) {
              b_a->data[i20 + b_a->size[0] * i21] += c_y->data[i20 + c_y->size[0]
                * ar] * b_b->data[ar + b_b->size[0] * i21];
            }
          }
        }
      } else {
        k = c_y->size[1];
        a[0] = (signed char)c_y->size[0];
        a[1] = (signed char)b_b->size[1];
        m = c_y->size[0];
        i20 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)b_a, i20, (int)sizeof(double));
        i20 = b_a->size[0] * b_a->size[1];
        b_a->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)b_a, i20, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i20 = 0; i20 < ib; i20++) {
          b_a->data[i20] = 0.0;
        }

        if ((c_y->size[0] == 0) || (b_b->size[1] == 0)) {
        } else {
          ii = c_y->size[0] * (b_b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              b_a->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i20 = br + k;
            for (ib = br; ib + 1 <= i20; ib++) {
              if (b_b->data[ib] != 0.0) {
                ia = ar;
                i21 = idx + m;
                for (ic = idx; ic + 1 <= i21; ic++) {
                  ia++;
                  b_a->data[ic] += b_b->data[ib] * c_y->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      S_size[0] = b_a->size[0];
      ib = b_a->size[0] * b_a->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        S_data[i20] = b_a->data[i20] + R_data[i20];
      }

      for (k = 0; k < numMeas; k++) {
        br = k << 1;
        ii = k << 1;
        idx = k << 1;
        for (i20 = 0; i20 < 2; i20++) {
          r[i20] = r_data[i20 + br];
          for (i21 = 0; i21 < 2; i21++) {
            c_xt[i21 + (i20 << 1)] = S_data[(i21 + ii) + S_size[0] * (i20 + idx)];
          }
        }

        b_mrdivide(r, c_xt, a);
        br = k << 1;
        for (i20 = 0; i20 < 2; i20++) {
          r[i20] = r_data[i20 + br];
        }

        if (LI_inlierStatus_data[k]) {
          //  if this feature is a LI inlier, don't also do HI update with this feature 
          br = k << 1;
          for (i20 = 0; i20 < 2; i20++) {
            r_data[i20 + br] = 0.0;
          }

          ib = H->size[1];
          br = k << 1;
          for (i20 = 0; i20 < ib; i20++) {
            for (i21 = 0; i21 < 2; i21++) {
              H->data[(i21 + br) + H->size[0] * i20] = 0.0;
            }
          }

          HI_inlierStatus_data[k] = false;
        } else {
          //  otherwise check if HI inlier for both cams
          d_numStatesxt = 0.0;
          for (i20 = 0; i20 < 2; i20++) {
            d_numStatesxt += a[i20] * r[i20];
          }

          if (d_numStatesxt > 9.0) {
            br = k << 1;
            for (i20 = 0; i20 < 2; i20++) {
              r_data[i20 + br] = 0.0;
            }

            ib = H->size[1];
            br = k << 1;
            for (i20 = 0; i20 < ib; i20++) {
              for (i21 = 0; i21 < 2; i21++) {
                H->data[(i21 + br) + H->size[0] * i20] = 0.0;
              }
            }

            HI_inlierStatus_data[k] = false;
          }
        }
      }

      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = H->size[0];
        c_y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        ib = H->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b_P->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            c_y->data[i20 + c_y->size[0] * i21] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              c_y->data[i20 + c_y->size[0] * i21] += H->data[i20 + H->size[0] *
                ar] * b_P->data[ar + b_P->size[0] * i21];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i20 = 0; i20 < ib; i20++) {
          c_y->data[i20] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              c_y->data[ic] = 0.0;
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
                  c_y->data[ic] += b_P->data[ib] * H->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      i20 = b_b->size[0] * b_b->size[1];
      b_b->size[0] = H->size[1];
      b_b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b_b, i20, (int)sizeof(double));
      ib = H->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = H->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          b_b->data[i21 + b_b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
        }
      }

      if ((c_y->size[1] == 1) || (b_b->size[0] == 1)) {
        i20 = C->size[0] * C->size[1];
        C->size[0] = c_y->size[0];
        C->size[1] = b_b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
        ib = c_y->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b_b->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            C->data[i20 + C->size[0] * i21] = 0.0;
            ii = c_y->size[1];
            for (ar = 0; ar < ii; ar++) {
              C->data[i20 + C->size[0] * i21] += c_y->data[i20 + c_y->size[0] *
                ar] * b_b->data[ar + b_b->size[0] * i21];
            }
          }
        }
      } else {
        k = c_y->size[1];
        a[0] = (signed char)c_y->size[0];
        a[1] = (signed char)b_b->size[1];
        m = c_y->size[0];
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

        if ((c_y->size[0] == 0) || (b_b->size[1] == 0)) {
        } else {
          ii = c_y->size[0] * (b_b->size[1] - 1);
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
              if (b_b->data[ib] != 0.0) {
                ia = ar;
                i21 = idx + m;
                for (ic = idx; ic + 1 <= i21; ic++) {
                  ia++;
                  C->data[ic] += b_b->data[ib] * c_y->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      S_size[0] = C->size[0];
      S_size[1] = C->size[1];
      ib = C->size[0] * C->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        S_data[i20] = C->data[i20] + R_data[i20];
      }
    } else {
      for (k = 0; k < numMeas; k++) {
        if (!HI_inlierStatus_data[k]) {
          br = k << 1;
          for (i20 = 0; i20 < 2; i20++) {
            r_data[i20 + br] = 0.0;
          }

          ib = H->size[1];
          br = k << 1;
          for (i20 = 0; i20 < ib; i20++) {
            for (i21 = 0; i21 < 2; i21++) {
              H->data[(i21 + br) + H->size[0] * i20] = 0.0;
            }
          }

          HI_inlierStatus_data[k] = false;
        }
      }

      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = H->size[0];
        c_y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        ib = H->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b_P->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            c_y->data[i20 + c_y->size[0] * i21] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              c_y->data[i20 + c_y->size[0] * i21] += H->data[i20 + H->size[0] *
                ar] * b_P->data[ar + b_P->size[0] * i21];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        i20 = c_y->size[0] * c_y->size[1];
        c_y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i20 = 0; i20 < ib; i20++) {
          c_y->data[i20] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              c_y->data[ic] = 0.0;
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
                  c_y->data[ic] += b_P->data[ib] * H->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      i20 = b_b->size[0] * b_b->size[1];
      b_b->size[0] = H->size[1];
      b_b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b_b, i20, (int)sizeof(double));
      ib = H->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = H->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          b_b->data[i21 + b_b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
        }
      }

      if ((c_y->size[1] == 1) || (b_b->size[0] == 1)) {
        i20 = C->size[0] * C->size[1];
        C->size[0] = c_y->size[0];
        C->size[1] = b_b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
        ib = c_y->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b_b->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            C->data[i20 + C->size[0] * i21] = 0.0;
            ii = c_y->size[1];
            for (ar = 0; ar < ii; ar++) {
              C->data[i20 + C->size[0] * i21] += c_y->data[i20 + c_y->size[0] *
                ar] * b_b->data[ar + b_b->size[0] * i21];
            }
          }
        }
      } else {
        k = c_y->size[1];
        a[0] = (signed char)c_y->size[0];
        a[1] = (signed char)b_b->size[1];
        m = c_y->size[0];
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

        if ((c_y->size[0] == 0) || (b_b->size[1] == 0)) {
        } else {
          ii = c_y->size[0] * (b_b->size[1] - 1);
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
              if (b_b->data[ib] != 0.0) {
                ia = ar;
                i21 = idx + m;
                for (ic = idx; ic + 1 <= i21; ic++) {
                  ia++;
                  C->data[ic] += b_b->data[ib] * c_y->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      S_size[0] = C->size[0];
      S_size[1] = C->size[1];
      ib = C->size[0] * C->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        S_data[i20] = C->data[i20] + R_data[i20];
      }
    }

    i20 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = H->size[1];
    b_b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i20, (int)sizeof(double));
    ib = H->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      idx = H->size[1];
      for (i21 = 0; i21 < idx; i21++) {
        b_b->data[i21 + b_b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
      }
    }

    if ((b_P->size[1] == 1) || (b_b->size[0] == 1)) {
      i20 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = b_P->size[0];
      c_y->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
      ib = b_P->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b_b->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          c_y->data[i20 + c_y->size[0] * i21] = 0.0;
          ii = b_P->size[1];
          for (ar = 0; ar < ii; ar++) {
            c_y->data[i20 + c_y->size[0] * i21] += b_P->data[i20 + b_P->size[0] *
              ar] * b_b->data[ar + b_b->size[0] * i21];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b_b->size[1];
      m = b_P->size[0];
      i20 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
      i20 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)c_y, i20, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        c_y->data[i20] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            c_y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
                ia++;
                c_y->data[ic] += b_b->data[ib] * b_P->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    mrdivide(c_y, S_data, S_size, K);
    if ((K->size[1] == 1) || (r_size[0] == 1)) {
      i20 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
      ib = K->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo->data[i20] = 0.0;
        idx = K->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          x_apo->data[i20] += K->data[i20 + K->size[0] * i21] * r_data[i21];
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

    for (i20 = 0; i20 < 3; i20++) {
      d_xt[i20] = b_xt->data[i20] + x_apo->data[i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      b_xt->data[i20] = d_xt[i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      d_xt[i20] = x_apo->data[3 + i20];
    }

    quatPlusThetaJ(d_xt, dv333);
    quatmultJ(dv333, *(double (*)[4])&b_xt->data[3], dv329);
    for (i20 = 0; i20 < 4; i20++) {
      b_xt->data[3 + i20] = dv329[i20];
    }

    if (8.0 > c_numStatesxt) {
      i20 = 0;
      i21 = 0;
    } else {
      i20 = 7;
      i21 = (int)c_numStatesxt;
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
    f_xt->size[0] = i21 - i20;
    emxEnsureCapacity((emxArray__common *)f_xt, idx, (int)sizeof(double));
    ib = i21 - i20;
    for (i21 = 0; i21 < ib; i21++) {
      f_xt->data[i21] = b_xt->data[i20 + i21] + x_apo->data[ar + i21];
    }

    ib = r13->size[1];
    for (i20 = 0; i20 < ib; i20++) {
      b_xt->data[r13->data[r13->size[0] * i20]] = f_xt->data[(*(int (*)[2])
        r13->size)[0] * i20];
    }

    for (br = 0; br < (int)numAnchors; br++) {
      for (i20 = 0; i20 < 32; i20++) {
        x[i20] = (b_anchorFeatures->data[i20 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) *
          numStatesPerAnchor;
        for (i20 = 0; i20 < 3; i20++) {
          d_xt[i20] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i20)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i20)) - 1];
        }

        for (i20 = 0; i20 < 3; i20++) {
          b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i20)) - 1] = d_xt[i20];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) *
          numStatesPerAnchor;
        for (i20 = 0; i20 < 3; i20++) {
          d_xt[i20] = x_apo->data[(int)(d_numStates + (4.0 + (double)i20)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 4; i20++) {
          c_xt[i20] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i20)) - 1];
        }

        quatPlusThetaJ(d_xt, dv334);
        quatmultJ(dv334, c_xt, dv329);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 4; i20++) {
          b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i20)) - 1] = dv329[i20];
        }

        for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) + (1.0 + (double)idx)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) *
                                numStatesPerAnchor) + 6.0) + (1.0 + (double)idx))
            - 1];
        }
      }
    }

    i20 = b_x_apo->size[0];
    b_x_apo->size[0] = x_apo->size[0];
    emxEnsureCapacity((emxArray__common *)b_x_apo, i20, (int)sizeof(double));
    ib = x_apo->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      b_x_apo->data[i20] = x_apo->data[i20] - x_apo_prev->data[i20];
    }

    if (d_norm(b_x_apo) < 0.001) {
      exitg1 = true;
    } else {
      i20 = x_apo_prev->size[0];
      x_apo_prev->size[0] = x_apo->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo_prev, i20, (int)sizeof(double));
      ib = x_apo->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo_prev->data[i20] = x_apo->data[i20];
      }

      it++;
    }
  }

  emxFree_real_T(&f_xt);
  emxFree_real_T(&b_x_apo);
  emxFree_real_T(&c_y);
  emxFree_int32_T(&r13);
  emxFree_real_T(&featureAnchorInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&map);
  emxFree_real_T(&x_apo_prev);
  emxFree_real_T(&x_apo);
  b_eye(c_numStates + numAnchors * numStatesPerAnchor, b_a);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i19 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
    loop_ub = K->size[0];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ib = H->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        C->data[i19 + C->size[0] * i20] = 0.0;
        idx = K->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          C->data[i19 + C->size[0] * i20] += K->data[i19 + K->size[0] * i21] *
            H->data[i21 + H->size[0] * i20];
        }
      }
    }
  } else {
    k = K->size[1];
    a[0] = (unsigned int)K->size[0];
    a[1] = (unsigned int)H->size[1];
    m = K->size[0];
    i19 = C->size[0] * C->size[1];
    C->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
    i19 = C->size[0] * C->size[1];
    C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
    loop_ub = (int)a[0] * (int)a[1];
    for (i19 = 0; i19 < loop_ub; i19++) {
      C->data[i19] = 0.0;
    }

    if ((K->size[0] == 0) || (H->size[1] == 0)) {
    } else {
      ii = K->size[0] * (H->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i19 = idx + m;
        for (ic = idx; ic + 1 <= i19; ic++) {
          C->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i19 = br + k;
        for (ib = br; ib + 1 <= i19; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
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
  i19 = b_a->size[0] * b_a->size[1];
  emxEnsureCapacity((emxArray__common *)b_a, i19, (int)sizeof(double));
  idx = b_a->size[0];
  ii = b_a->size[1];
  loop_ub = idx * ii;
  for (i19 = 0; i19 < loop_ub; i19++) {
    b_a->data[i19] -= C->data[i19];
  }

  emxFree_real_T(&C);
  i19 = b_b->size[0] * b_b->size[1];
  b_b->size[0] = b_P->size[0];
  b_b->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b_b, i19, (int)sizeof(double));
  loop_ub = b_P->size[0] * b_P->size[1];
  for (i19 = 0; i19 < loop_ub; i19++) {
    b_b->data[i19] = b_P->data[i19];
  }

  emxInit_real_T(&d_a, 2);
  if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
    i19 = d_a->size[0] * d_a->size[1];
    d_a->size[0] = b_a->size[0];
    d_a->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)d_a, i19, (int)sizeof(double));
    loop_ub = b_a->size[0];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ib = b_P->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        d_a->data[i19 + d_a->size[0] * i20] = 0.0;
        idx = b_a->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          d_a->data[i19 + d_a->size[0] * i20] += b_a->data[i19 + b_a->size[0] *
            i21] * b_P->data[i21 + b_P->size[0] * i20];
        }
      }
    }

    i19 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = d_a->size[0];
    b_P->size[1] = d_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i19, (int)sizeof(double));
    loop_ub = d_a->size[1];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ib = d_a->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        b_P->data[i20 + b_P->size[0] * i19] = d_a->data[i20 + d_a->size[0] * i19];
      }
    }
  } else {
    k = b_a->size[1];
    a[0] = b_a->size[0];
    a[1] = b_P->size[1];
    m = b_a->size[0];
    i19 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = (int)a[0];
    b_P->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)b_P, i19, (int)sizeof(double));
    loop_ub = (int)a[1];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ib = (int)a[0];
      for (i20 = 0; i20 < ib; i20++) {
        b_P->data[i20 + b_P->size[0] * i19] = 0.0;
      }
    }

    if ((b_a->size[0] == 0) || (b_b->size[1] == 0)) {
    } else {
      ii = b_a->size[0] * (b_b->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i19 = idx + m;
        for (ic = idx; ic + 1 <= i19; ic++) {
          b_P->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i19 = br + k;
        for (ib = br; ib + 1 <= i19; ib++) {
          if (b_b->data[ib] != 0.0) {
            ia = ar;
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              ia++;
              b_P->data[ic] += b_b->data[ib] * b_a->data[ia - 1];
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
  emxFree_real_T(&b_b);
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
