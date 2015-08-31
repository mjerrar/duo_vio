//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Aug-2015 09:51:22
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
#include "ros_warn.h"
#include "eye.h"
#include "getMap.h"
#include "undistortPoint.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : emxArray_real_T *b_xt
//                emxArray_real_T *b_P
//                double z_all_l[32]
//                double c_numStatesxt
//                double c_numStates
//                const double c_cameraparams_CameraParameters[3]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const emxArray_real_T *b_anchorFeatures
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double c_noiseParameters_orientation_n
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                const VIOMeasurements *IMU_measurements
//                double b_height_offset_pressure
//                const VIOParameters b_VIOParameters
//                double validFeatures_data[]
//                int validFeatures_size[1]
// Return Type  : void
//
void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P, double
  z_all_l[32], double c_numStatesxt, double c_numStates, const double
  c_cameraparams_CameraParameters[3], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const emxArray_real_T *b_anchorFeatures,
  const emxArray_real_T *b_m_vect, const double noiseParameters_image_noise[2],
  double c_noiseParameters_orientation_n, double noiseParameters_pressure_noise,
  double noiseParameters_ext_pos_noise, double noiseParameters_ext_att_noise,
  const VIOMeasurements *IMU_measurements, double b_height_offset_pressure,
  const VIOParameters b_VIOParameters, double validFeatures_data[], int
  validFeatures_size[1])
{
  emxArray_boolean_T *c_anchorFeatures;
  double numPointsPerAnchor;
  int numAnchors;
  int i19;
  int loop_ub;
  boolean_T x[16];
  int idx;
  int ii_data[16];
  int nx;
  boolean_T exitg3;
  boolean_T guard2 = false;
  int ii_size_idx_0;
  signed char indMeas_data[16];
  emxArray_real_T *K;
  emxArray_real_T *H;
  int numMeas;
  signed char x_data[32];
  double indMeas_z_data[32];
  int k;
  double z_all_l_data[32];
  int z_all_l_size[1];
  int indMeas_z_size[1];
  double tmp_data[32];
  int ar;
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
  emxArray_int32_T *r17;
  emxArray_real_T *b;
  emxArray_real_T *y;
  emxArray_real_T *r18;
  emxArray_int32_T *r19;
  emxArray_int32_T *r20;
  emxArray_real_T *b_x_apo_prev;
  double d_numStatesxt;
  int R_size[2];
  double R_data[144];
  double r[2];
  int r_size[1];
  double r_data[12];
  int i20;
  int anchorIdx;
  double a[2];
  int m;
  int ic;
  int br;
  int ib;
  int ia;
  double C_data[144];
  int C_size[2];
  double c_xt[3];
  double dv23[4];
  double dv24[4];
  double e_numStatesxt;
  double d_numStates;
  double d_xt[4];
  double dv25[4];
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

  emxArray_real_T *b_a;
  int iter;
  emxArray_real_T *e_xt;
  double b_indMeas_data[16];
  int indMeas_size[1];
  int b_R_size[2];
  double b_R_data[1764];
  int b_r_size[1];
  double b_r_data[42];
  double b_C_data[1764];
  int b_C_size[2];
  double dv26[4];
  double dv27[4];
  emxArray_real_T *c_a;
  char cv10[118];
  static const char cv11[118] = { '1', '-', 'P', 'o', 'i', 'n', 't', ' ', 'R',
    'A', 'N', 'S', 'A', 'C', ':', ' ', 'E', 'n', 'd', 'e', 'd', ' ', 'h', 'y',
    'p', 'o', 't', 'h', 'e', 's', 'i', 's', ' ', 't', 'e', 's', 't', '.', ' ',
    'F', 'o', 'u', 'n', 'd', ' ', '%', 'i', ' ', 'L', 'I', ' ', 'i', 'n', 'l',
    'i', 'e', 'r', 's', ',', ' ', 'w', 'h', 'i', 'c', 'h', ' ', 'i', 's', ' ',
    'b', 'e', 'l', 'o', 'w', ' ', 't', 'h', 'e', ' ', 't', 'h', 'r', 'e', 's',
    'h', 'o', 'l', 'd', ' ', '%', 'i', '.', ' ', 'N', 'o', 't', ' ', 'd', 'o',
    'i', 'n', 'g', ' ', 'L', 'I', ' ', 'E', 'K', 'F', ' ', 'u', 'p', 'd', 'a',
    't', 'e', '.', '\x00' };

  int it;
  emxArray_real_T *b_x_apo;
  emxArray_real_T *f_xt;
  boolean_T exitg1;
  int b_indMeas_size[1];
  double S_data[1764];
  int c_C_size[2];
  double dv28[4];
  double dv29[4];
  emxArray_real_T *d_a;
  emxInit_boolean_T(&c_anchorFeatures, 2);

  //  threshold for LI innovation test
  // 9.487729036781154; % HI mahalanobis gate
  //  threshold for minimum LI supporter
  numPointsPerAnchor = b_VIOParameters.num_points_per_anchor;
  numAnchors = b_anchorFeatures->size[1] - 1;
  i19 = c_anchorFeatures->size[0] * c_anchorFeatures->size[1];
  c_anchorFeatures->size[0] = 16;
  c_anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)c_anchorFeatures, i19, (int)sizeof
                    (boolean_T));
  loop_ub = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i19 = 0; i19 < loop_ub; i19++) {
    c_anchorFeatures->data[i19] = (b_anchorFeatures->data[i19] == 1.0);
  }

  b_any(c_anchorFeatures, x);
  idx = 0;
  nx = 1;
  emxFree_boolean_T(&c_anchorFeatures);
  exitg3 = false;
  while ((!exitg3) && (nx < 17)) {
    guard2 = false;
    if (x[nx - 1]) {
      idx++;
      ii_data[idx - 1] = nx;
      if (idx >= 16) {
        exitg3 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      nx++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  if (1 > idx) {
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  for (i19 = 0; i19 < loop_ub; i19++) {
    indMeas_data[i19] = (signed char)ii_data[i19];
  }

  emxInit_real_T(&K, 2);
  emxInit_real_T(&H, 2);
  numMeas = ii_size_idx_0;
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
  for (i19 = 0; i19 < ii_size_idx_0; i19++) {
    x_data[i19 << 1] = (signed char)((indMeas_data[i19] - 1) * 2 + 1);
  }

  for (i19 = 0; i19 < ii_size_idx_0; i19++) {
    x_data[1 + (i19 << 1)] = (signed char)((indMeas_data[i19] - 1) * 2 + 2);
  }

  nx = loop_ub << 1;
  for (k = 0; k + 1 <= nx; k++) {
    indMeas_z_data[k] = x_data[k];
  }

  z_all_l_size[0] = nx;
  for (i19 = 0; i19 < nx; i19++) {
    z_all_l_data[i19] = z_all_l[(int)indMeas_z_data[i19] - 1];
  }

  undistortPoint(z_all_l_data, z_all_l_size, c_cameraparams_CameraParameters,
                 d_cameraparams_CameraParameters,
                 e_cameraparams_CameraParameters, tmp_data, indMeas_z_size);
  ar = indMeas_z_size[0];
  for (i19 = 0; i19 < ar; i19++) {
    z_all_l[(int)indMeas_z_data[i19] - 1] = tmp_data[i19];
  }

  // % B 1-point hypotheses generation and evaluation
  n_hyp = 100.0;
  LI_inlierStatus_size_idx_0 = ii_size_idx_0;
  for (i19 = 0; i19 < ii_size_idx_0; i19++) {
    LI_inlierStatus_data[i19] = false;
  }

  emxInit_real_T(&map, 2);
  b_emxInit_real_T(&anchorInd, 1);
  b_emxInit_real_T(&featureAnchorInd, 1);

  //  low innovation inlier status
  //  build the map according to the current estimate
  getMap(b_xt, b_anchorFeatures, b_m_vect, (double)b_anchorFeatures->size[1] *
         b_VIOParameters.num_points_per_anchor, c_numStatesxt, 7.0 +
         b_VIOParameters.num_points_per_anchor, map, anchorInd, featureAnchorInd);
  i = 1.0;
  emxInit_real_T(&K_i, 2);
  b_emxInit_real_T(&x_apo, 1);
  b_emxInit_real_T(&x_apo_prev, 1);
  emxInit_real_T(&H_i, 2);
  emxInit_real_T(&C, 2);
  b_emxInit_int32_T(&r17, 2);
  emxInit_real_T(&b, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&r18, 2);
  emxInit_int32_T(&r19, 1);
  emxInit_int32_T(&r20, 1);
  b_emxInit_real_T(&b_x_apo_prev, 1);
  while (i < n_hyp) {
    //  select a random measurement
    d_numStatesxt = b_rand();

    //  EKF state update
    getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, (double)indMeas_data
               [(int)(1.0 + floor(d_numStatesxt * (double)numMeas)) - 1], map,
               anchorInd, featureAnchorInd, b_m_vect,
               noiseParameters_image_noise, c_noiseParameters_orientation_n,
               noiseParameters_pressure_noise, noiseParameters_ext_pos_noise,
               noiseParameters_ext_att_noise, IMU_measurements,
               b_height_offset_pressure, b_VIOParameters, r_data, r_size, H_i, r,
               R_data, R_size);
    if ((H_i->size[1] == 1) || (b_P->size[0] == 1)) {
      i19 = y->size[0] * y->size[1];
      y->size[0] = H_i->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ar = H_i->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = b_P->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          y->data[i19 + y->size[0] * i20] = 0.0;
          nx = H_i->size[1];
          for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
            y->data[i19 + y->size[0] * i20] += H_i->data[i19 + H_i->size[0] *
              anchorIdx] * b_P->data[anchorIdx + b_P->size[0] * i20];
          }
        }
      }
    } else {
      k = H_i->size[1];
      a[0] = H_i->size[0];
      a[1] = b_P->size[1];
      m = H_i->size[0];
      i19 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      i19 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ar = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ar; i19++) {
        y->data[i19] = 0.0;
      }

      if ((H_i->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        nx = H_i->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
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

    i19 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = H_i->size[0];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
    ar = H_i->size[0];
    for (i19 = 0; i19 < ar; i19++) {
      idx = H_i->size[1];
      for (i20 = 0; i20 < idx; i20++) {
        b->data[i20 + b->size[0] * i19] = H_i->data[i19 + H_i->size[0] * i20];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i19 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ar = y->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = b->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          C->data[i19 + C->size[0] * i20] = 0.0;
          nx = y->size[1];
          for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
            C->data[i19 + C->size[0] * i20] += y->data[i19 + y->size[0] *
              anchorIdx] * b->data[anchorIdx + b->size[0] * i20];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i19 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      i19 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ar = (int)((float)a[0] * (float)a[1]);
      for (i19 = 0; i19 < ar; i19++) {
        C->data[i19] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        nx = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
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

    i19 = b->size[0] * b->size[1];
    b->size[0] = H_i->size[1];
    b->size[1] = H_i->size[0];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
    ar = H_i->size[0];
    for (i19 = 0; i19 < ar; i19++) {
      idx = H_i->size[1];
      for (i20 = 0; i20 < idx; i20++) {
        b->data[i20 + b->size[0] * i19] = H_i->data[i19 + H_i->size[0] * i20];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i19 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ar = b_P->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = b->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          y->data[i19 + y->size[0] * i20] = 0.0;
          nx = b_P->size[1];
          for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
            y->data[i19 + y->size[0] * i20] += b_P->data[i19 + b_P->size[0] *
              anchorIdx] * b->data[anchorIdx + b->size[0] * i20];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i19 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      i19 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ar = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ar; i19++) {
        y->data[i19] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        nx = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
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
    ar = C->size[0] * C->size[1];
    for (i19 = 0; i19 < ar; i19++) {
      C_data[i19] = C->data[i19] + R_data[i19];
    }

    mrdivide(y, C_data, C_size, r18);
    i19 = K_i->size[0] * K_i->size[1];
    K_i->size[0] = r18->size[0];
    K_i->size[1] = r18->size[1];
    emxEnsureCapacity((emxArray__common *)K_i, i19, (int)sizeof(double));
    ar = r18->size[0] * r18->size[1];
    for (i19 = 0; i19 < ar; i19++) {
      K_i->data[i19] = r18->data[i19];
    }

    if ((K_i->size[1] == 1) || (r_size[0] == 1)) {
      i19 = x_apo->size[0];
      x_apo->size[0] = K_i->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
      ar = K_i->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        x_apo->data[i19] = 0.0;
        idx = K_i->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          x_apo->data[i19] += K_i->data[i19 + K_i->size[0] * i20] * r_data[i20];
        }
      }
    } else {
      k = K_i->size[1];
      a[0] = K_i->size[0];
      m = K_i->size[0];
      i19 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
      ar = (int)a[0];
      for (i19 = 0; i19 < ar; i19++) {
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
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
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

    i19 = x_apo_prev->size[0];
    x_apo_prev->size[0] = b_xt->size[0];
    emxEnsureCapacity((emxArray__common *)x_apo_prev, i19, (int)sizeof(double));
    ar = b_xt->size[0];
    for (i19 = 0; i19 < ar; i19++) {
      x_apo_prev->data[i19] = b_xt->data[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      x_apo_prev->data[i19] = b_xt->data[i19] + x_apo->data[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      c_xt[i19] = x_apo->data[3 + i19];
    }

    quatPlusThetaJ(c_xt, dv23);
    quatmultJ(dv23, *(double (*)[4])&x_apo_prev->data[3], dv24);
    for (i19 = 0; i19 < 4; i19++) {
      x_apo_prev->data[3 + i19] = dv24[i19];
    }

    if (8.0 > c_numStatesxt) {
      i19 = 1;
      i20 = 0;
    } else {
      i19 = 8;
      i20 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      anchorIdx = 1;
      idx = 0;
    } else {
      anchorIdx = 7;
      idx = (int)c_numStates;
    }

    if (8.0 > c_numStatesxt) {
      ii_size_idx_0 = 0;
      br = 0;
    } else {
      ii_size_idx_0 = 7;
      br = (int)c_numStatesxt;
    }

    nx = r17->size[0] * r17->size[1];
    r17->size[0] = 1;
    r17->size[1] = br - ii_size_idx_0;
    emxEnsureCapacity((emxArray__common *)r17, nx, (int)sizeof(int));
    ar = br - ii_size_idx_0;
    for (br = 0; br < ar; br++) {
      r17->data[r17->size[0] * br] = ii_size_idx_0 + br;
    }

    ii_size_idx_0 = r19->size[0];
    r19->size[0] = (i20 - i19) + 1;
    emxEnsureCapacity((emxArray__common *)r19, ii_size_idx_0, (int)sizeof(int));
    ar = i20 - i19;
    for (i20 = 0; i20 <= ar; i20++) {
      r19->data[i20] = i19 + i20;
    }

    i19 = r20->size[0];
    r20->size[0] = (idx - anchorIdx) + 1;
    emxEnsureCapacity((emxArray__common *)r20, i19, (int)sizeof(int));
    ar = idx - anchorIdx;
    for (i19 = 0; i19 <= ar; i19++) {
      r20->data[i19] = anchorIdx + i19;
    }

    i19 = b_x_apo_prev->size[0];
    b_x_apo_prev->size[0] = r17->size[0] * r17->size[1];
    emxEnsureCapacity((emxArray__common *)b_x_apo_prev, i19, (int)sizeof(double));
    ar = r17->size[0] * r17->size[1];
    for (i19 = 0; i19 < ar; i19++) {
      b_x_apo_prev->data[i19] = x_apo_prev->data[r19->data[i19] - 1] +
        x_apo->data[r20->data[i19] - 1];
    }

    ar = b_x_apo_prev->size[0];
    for (i19 = 0; i19 < ar; i19++) {
      x_apo_prev->data[r17->data[i19]] = b_x_apo_prev->data[i19];
    }

    for (anchorIdx = 0; anchorIdx <= numAnchors; anchorIdx++) {
      for (i19 = 0; i19 < 16; i19++) {
        x[i19] = (b_anchorFeatures->data[i19 + b_anchorFeatures->size[0] *
                  anchorIdx] == 1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0
          + numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0
          + numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 3; i19++) {
          c_xt[i19] = x_apo_prev->data[(int)(d_numStatesxt + (1.0 + (double)i19))
            - 1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i19)) - 1];
        }

        for (i19 = 0; i19 < 3; i19++) {
          x_apo_prev->data[(int)(e_numStatesxt + (1.0 + (double)i19)) - 1] =
            c_xt[i19];
        }

        d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 3; i19++) {
          c_xt[i19] = x_apo->data[(int)(d_numStates + (4.0 + (double)i19)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0
          + numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          d_xt[i19] = x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i19))
            - 1];
        }

        quatPlusThetaJ(c_xt, dv25);
        quatmultJ(dv25, d_xt, dv24);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0
          + numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i19)) - 1] =
            dv24[i19];
        }

        for (nx = 0; nx < (int)numPointsPerAnchor; nx++) {
          x_apo_prev->data[(int)(((c_numStatesxt + ((1.0 + (double)anchorIdx) -
            1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + (1.0 + (double)nx)) - 1]
            += x_apo->data[(int)(((c_numStates + ((1.0 + (double)anchorIdx) -
            1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (1.0 + (double)nx)) - 1];
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
    for (anchorIdx = 0; anchorIdx <= numAnchors; anchorIdx++) {
      featureAnchorIdx = 1.0;
      idx = 0;
      nx = 1;
      exitg2 = false;
      while ((!exitg2) && (nx < 17)) {
        guard1 = false;
        if (b_anchorFeatures->data[(nx + b_anchorFeatures->size[0] * anchorIdx)
            - 1] != 0.0) {
          idx++;
          ii_data[idx - 1] = nx;
          if (idx >= 16) {
            exitg2 = true;
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          nx++;
        }
      }

      if (1 > idx) {
        ii_size_idx_0 = 0;
      } else {
        ii_size_idx_0 = idx;
      }

      if (1 > idx) {
        ar = 0;
      } else {
        ar = idx;
      }

      for (i19 = 0; i19 < ar; i19++) {
        featureIdxVect_data[i19] = (signed char)ii_data[i19];
      }

      for (idx = 0; idx < ii_size_idx_0; idx++) {
        if (b_anchorFeatures->data[(featureIdxVect_data[idx] +
             b_anchorFeatures->size[0] * anchorIdx) - 1] == 1.0) {
          //  if this is not a lost feature
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            (7.0 + numPointsPerAnchor);
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
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]
                              - x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1])
                             - x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1])
            + x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
                     numPointsPerAnchor)) + 7.0) - 1];
          c_x_apo_prev[1] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]
            * x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] + x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 6.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[2] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]
            * x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] - x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 5.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[3] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]
            * x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] - x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 6.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[4] = ((-(x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                                x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1])
                              + x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1])
                             - x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1])
            + x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
                     numPointsPerAnchor)) + 7.0) - 1];
          c_x_apo_prev[5] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1]
            * x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] + x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 4.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[6] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]
            * x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] + x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 5.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[7] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1]
            * x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] - x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 4.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[8] = ((-(x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                                x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1])
                              - x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1])
                             + x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1])
            + x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
            - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data
            [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0 +
                     numPointsPerAnchor)) + 7.0) - 1];
          d_numStates = x_apo_prev->data[(int)(((c_numStatesxt + ((1.0 + (double)
            anchorIdx) - 1.0) * (7.0 + b_VIOParameters.num_points_per_anchor)) +
            7.0) + featureAnchorIdx) - 1];
          for (i19 = 0; i19 < 3; i19++) {
            d_numStatesxt = 0.0;
            for (i20 = 0; i20 < 3; i20++) {
              d_numStatesxt += c_x_apo_prev[i19 + 3 * i20] * b_m_vect->data[i20
                + b_m_vect->size[0] * (featureIdxVect_data[idx] - 1)];
            }

            b_anchorPos[i19] = (anchorPos[i19] + d_numStatesxt / d_numStates) -
              x_apo_prev->data[iv13[i19]];
          }

          for (i19 = 0; i19 < 3; i19++) {
            c_xt[i19] = 0.0;
            for (i20 = 0; i20 < 3; i20++) {
              c_xt[i19] += R_cw[i19 + 3 * i20] * b_anchorPos[i20];
            }
          }

          predictMeasurement_left(c_xt, r);
          nx = (featureIdxVect_data[idx] - 1) * 2;
          for (i19 = 0; i19 < 2; i19++) {
            b_z_all_l[i19] = z_all_l[(nx + iv14[i19]) - 1] - r[i19];
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

    nx = 0;
    for (k = 0; k < LI_inlierStatus_size_idx_0; k++) {
      if (LI_inlierStatus_data[k]) {
        nx++;
      }
    }

    if (idx > nx) {
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
  emxFree_int32_T(&r20);
  emxFree_int32_T(&r19);
  emxFree_real_T(&r18);
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
    iter = 0;
    b_emxInit_real_T(&e_xt, 1);
    while (iter <= (int)b_VIOParameters.max_ekf_iterations - 1) {
      nx = LI_inlierStatus_size_idx_0 - 1;
      br = 0;
      for (idx = 0; idx <= nx; idx++) {
        if (LI_inlierStatus_data[idx]) {
          br++;
        }
      }

      ii_size_idx_0 = 0;
      for (idx = 0; idx <= nx; idx++) {
        if (LI_inlierStatus_data[idx]) {
          ii_data[ii_size_idx_0] = idx + 1;
          ii_size_idx_0++;
        }
      }

      indMeas_size[0] = br;
      for (i19 = 0; i19 < br; i19++) {
        b_indMeas_data[i19] = indMeas_data[ii_data[i19] - 1];
      }

      b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                   indMeas_size, map, anchorInd, featureAnchorInd, b_m_vect,
                   noiseParameters_image_noise, c_noiseParameters_orientation_n,
                   noiseParameters_pressure_noise, noiseParameters_ext_pos_noise,
                   noiseParameters_ext_att_noise, IMU_measurements,
                   b_height_offset_pressure, b_VIOParameters, b_r_data, b_r_size,
                   H, indMeas_z_data, indMeas_z_size, b_R_data, b_R_size);
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i19 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        ar = H->size[0];
        for (i19 = 0; i19 < ar; i19++) {
          idx = b_P->size[1];
          for (i20 = 0; i20 < idx; i20++) {
            y->data[i19 + y->size[0] * i20] = 0.0;
            nx = H->size[1];
            for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
              y->data[i19 + y->size[0] * i20] += H->data[i19 + H->size[0] *
                anchorIdx] * b_P->data[anchorIdx + b_P->size[0] * i20];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i19 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        i19 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        ar = (int)a[0] * (int)a[1];
        for (i19 = 0; i19 < ar; i19++) {
          y->data[i19] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          nx = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            i19 = idx + m;
            for (ic = idx; ic + 1 <= i19; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            ar = 0;
            i19 = br + k;
            for (ib = br; ib + 1 <= i19; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i20 = idx + m;
                for (ic = idx; ic + 1 <= i20; ic++) {
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

      i19 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
      ar = H->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = H->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          b->data[i20 + b->size[0] * i19] = H->data[i19 + H->size[0] * i20];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i19 = C->size[0] * C->size[1];
        C->size[0] = y->size[0];
        C->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
        ar = y->size[0];
        for (i19 = 0; i19 < ar; i19++) {
          idx = b->size[1];
          for (i20 = 0; i20 < idx; i20++) {
            C->data[i19 + C->size[0] * i20] = 0.0;
            nx = y->size[1];
            for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
              C->data[i19 + C->size[0] * i20] += y->data[i19 + y->size[0] *
                anchorIdx] * b->data[anchorIdx + b->size[0] * i20];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i19 = C->size[0] * C->size[1];
        C->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
        i19 = C->size[0] * C->size[1];
        C->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
        ar = (int)((float)a[0] * (float)a[1]);
        for (i19 = 0; i19 < ar; i19++) {
          C->data[i19] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          nx = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            i19 = idx + m;
            for (ic = idx; ic + 1 <= i19; ic++) {
              C->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            ar = 0;
            i19 = br + k;
            for (ib = br; ib + 1 <= i19; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i20 = idx + m;
                for (ic = idx; ic + 1 <= i20; ic++) {
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

      i19 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
      ar = H->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = H->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          b->data[i20 + b->size[0] * i19] = H->data[i19 + H->size[0] * i20];
        }
      }

      if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
        i19 = y->size[0] * y->size[1];
        y->size[0] = b_P->size[0];
        y->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        ar = b_P->size[0];
        for (i19 = 0; i19 < ar; i19++) {
          idx = b->size[1];
          for (i20 = 0; i20 < idx; i20++) {
            y->data[i19 + y->size[0] * i20] = 0.0;
            nx = b_P->size[1];
            for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
              y->data[i19 + y->size[0] * i20] += b_P->data[i19 + b_P->size[0] *
                anchorIdx] * b->data[anchorIdx + b->size[0] * i20];
            }
          }
        }
      } else {
        k = b_P->size[1];
        a[0] = b_P->size[0];
        a[1] = b->size[1];
        m = b_P->size[0];
        i19 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        i19 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        ar = (int)a[0] * (int)a[1];
        for (i19 = 0; i19 < ar; i19++) {
          y->data[i19] = 0.0;
        }

        if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          nx = b_P->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            i19 = idx + m;
            for (ic = idx; ic + 1 <= i19; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            ar = 0;
            i19 = br + k;
            for (ib = br; ib + 1 <= i19; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i20 = idx + m;
                for (ic = idx; ic + 1 <= i20; ic++) {
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
      ar = C->size[0] * C->size[1];
      for (i19 = 0; i19 < ar; i19++) {
        b_C_data[i19] = C->data[i19] + b_R_data[i19];
      }

      mrdivide(y, b_C_data, b_C_size, K);
      if ((K->size[1] == 1) || (b_r_size[0] == 1)) {
        i19 = x_apo->size[0];
        x_apo->size[0] = K->size[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
        ar = K->size[0];
        for (i19 = 0; i19 < ar; i19++) {
          x_apo->data[i19] = 0.0;
          idx = K->size[1];
          for (i20 = 0; i20 < idx; i20++) {
            x_apo->data[i19] += K->data[i19 + K->size[0] * i20] * b_r_data[i20];
          }
        }
      } else {
        k = K->size[1];
        a[0] = K->size[0];
        m = K->size[0];
        i19 = x_apo->size[0];
        x_apo->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
        ar = (int)a[0];
        for (i19 = 0; i19 < ar; i19++) {
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

      for (i19 = 0; i19 < 3; i19++) {
        c_xt[i19] = b_xt->data[i19] + x_apo->data[i19];
      }

      for (i19 = 0; i19 < 3; i19++) {
        b_xt->data[i19] = c_xt[i19];
      }

      for (i19 = 0; i19 < 3; i19++) {
        c_xt[i19] = x_apo->data[3 + i19];
      }

      quatPlusThetaJ(c_xt, dv26);
      quatmultJ(dv26, *(double (*)[4])&b_xt->data[3], dv24);
      for (i19 = 0; i19 < 4; i19++) {
        b_xt->data[3 + i19] = dv24[i19];
      }

      if (8.0 > c_numStatesxt) {
        i19 = 0;
        i20 = 0;
      } else {
        i19 = 7;
        i20 = (int)c_numStatesxt;
      }

      if (7.0 > c_numStates) {
        anchorIdx = 0;
      } else {
        anchorIdx = 6;
      }

      if (8.0 > c_numStatesxt) {
        idx = 0;
        ii_size_idx_0 = 0;
      } else {
        idx = 7;
        ii_size_idx_0 = (int)c_numStatesxt;
      }

      br = r17->size[0] * r17->size[1];
      r17->size[0] = 1;
      r17->size[1] = ii_size_idx_0 - idx;
      emxEnsureCapacity((emxArray__common *)r17, br, (int)sizeof(int));
      ar = ii_size_idx_0 - idx;
      for (ii_size_idx_0 = 0; ii_size_idx_0 < ar; ii_size_idx_0++) {
        r17->data[r17->size[0] * ii_size_idx_0] = idx + ii_size_idx_0;
      }

      idx = e_xt->size[0];
      e_xt->size[0] = i20 - i19;
      emxEnsureCapacity((emxArray__common *)e_xt, idx, (int)sizeof(double));
      ar = i20 - i19;
      for (i20 = 0; i20 < ar; i20++) {
        e_xt->data[i20] = b_xt->data[i19 + i20] + x_apo->data[anchorIdx + i20];
      }

      ar = r17->size[1];
      for (i19 = 0; i19 < ar; i19++) {
        b_xt->data[r17->data[r17->size[0] * i19]] = e_xt->data[(*(int (*)[2])
          r17->size)[0] * i19];
      }

      for (anchorIdx = 0; anchorIdx <= numAnchors; anchorIdx++) {
        for (i19 = 0; i19 < 16; i19++) {
          x[i19] = (b_anchorFeatures->data[i19 + b_anchorFeatures->size[0] *
                    anchorIdx] == 1.0);
        }

        if (any(x)) {
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            (7.0 + numPointsPerAnchor);
          d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            (7.0 + numPointsPerAnchor);
          d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) * (6.0 +
            numPointsPerAnchor);
          for (i19 = 0; i19 < 3; i19++) {
            c_xt[i19] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i19)) -
              1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i19)) - 1];
          }

          for (i19 = 0; i19 < 3; i19++) {
            b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i19)) - 1] =
              c_xt[i19];
          }

          d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) * (6.0 +
            numPointsPerAnchor);
          for (i19 = 0; i19 < 3; i19++) {
            c_xt[i19] = x_apo->data[(int)(d_numStates + (4.0 + (double)i19)) - 1];
          }

          e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            (7.0 + numPointsPerAnchor);
          for (i19 = 0; i19 < 4; i19++) {
            d_xt[i19] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i19)) -
              1];
          }

          quatPlusThetaJ(c_xt, dv27);
          quatmultJ(dv27, d_xt, dv24);
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            (7.0 + numPointsPerAnchor);
          for (i19 = 0; i19 < 4; i19++) {
            b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i19)) - 1] =
              dv24[i19];
          }

          for (nx = 0; nx < (int)numPointsPerAnchor; nx++) {
            b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0)
                               * (7.0 + numPointsPerAnchor)) + 7.0) + (1.0 +
              (double)nx)) - 1] += x_apo->data[(int)(((c_numStates + ((1.0 +
              (double)anchorIdx) - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) +
              (1.0 + (double)nx)) - 1];
          }
        }
      }

      iter++;
    }

    emxFree_real_T(&e_xt);
    b_eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
           b_VIOParameters.num_points_per_anchor), b_a);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i19 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ar = K->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = H->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          C->data[i19 + C->size[0] * i20] = 0.0;
          nx = K->size[1];
          for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
            C->data[i19 + C->size[0] * i20] += K->data[i19 + K->size[0] *
              anchorIdx] * H->data[anchorIdx + H->size[0] * i20];
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
      ar = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ar; i19++) {
        C->data[i19] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        nx = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
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
    nx = b_a->size[0];
    idx = b_a->size[1];
    ar = nx * idx;
    for (i19 = 0; i19 < ar; i19++) {
      b_a->data[i19] -= C->data[i19];
    }

    i19 = b->size[0] * b->size[1];
    b->size[0] = b_P->size[0];
    b->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
    ar = b_P->size[0] * b_P->size[1];
    for (i19 = 0; i19 < ar; i19++) {
      b->data[i19] = b_P->data[i19];
    }

    emxInit_real_T(&c_a, 2);
    if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
      i19 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = b_a->size[0];
      c_a->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)c_a, i19, (int)sizeof(double));
      ar = b_a->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = b_P->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          c_a->data[i19 + c_a->size[0] * i20] = 0.0;
          nx = b_a->size[1];
          for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
            c_a->data[i19 + c_a->size[0] * i20] += b_a->data[i19 + b_a->size[0] *
              anchorIdx] * b_P->data[anchorIdx + b_P->size[0] * i20];
          }
        }
      }

      i19 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = c_a->size[0];
      b_P->size[1] = c_a->size[1];
      emxEnsureCapacity((emxArray__common *)b_P, i19, (int)sizeof(double));
      ar = c_a->size[1];
      for (i19 = 0; i19 < ar; i19++) {
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
      ar = (int)a[1];
      for (i19 = 0; i19 < ar; i19++) {
        idx = (int)a[0];
        for (i20 = 0; i20 < idx; i20++) {
          b_P->data[i20 + b_P->size[0] * i19] = 0.0;
        }
      }

      if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        nx = b_a->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            b_P->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
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
    memcpy(&cv10[0], &cv11[0], 118U * sizeof(char));
    ROS_WARN(cv10, idx, 3);
  }

  // % D Partial EKF update using high-innovation inliers
  for (i19 = 0; i19 < loop_ub; i19++) {
    HI_inlierStatus_data[i19] = true;
  }

  //  high innovation inliers
  i19 = x_apo_prev->size[0];
  x_apo_prev->size[0] = b_P->size[0];
  emxEnsureCapacity((emxArray__common *)x_apo_prev, i19, (int)sizeof(double));
  ar = b_P->size[0];
  for (i19 = 0; i19 < ar; i19++) {
    x_apo_prev->data[i19] = 0.0;
  }

  it = 0;
  b_emxInit_real_T(&b_x_apo, 1);
  b_emxInit_real_T(&f_xt, 1);
  exitg1 = false;
  while ((!exitg1) && (it <= (int)b_VIOParameters.max_ekf_iterations - 1)) {
    nx = loop_ub - 1;
    br = 0;
    for (idx = 0; idx <= nx; idx++) {
      if (HI_inlierStatus_data[idx]) {
        br++;
      }
    }

    ii_size_idx_0 = 0;
    for (idx = 0; idx <= nx; idx++) {
      if (HI_inlierStatus_data[idx]) {
        ii_data[ii_size_idx_0] = idx + 1;
        ii_size_idx_0++;
      }
    }

    b_indMeas_size[0] = br;
    for (i19 = 0; i19 < br; i19++) {
      b_indMeas_data[i19] = indMeas_data[ii_data[i19] - 1];
    }

    b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                 b_indMeas_size, map, anchorInd, featureAnchorInd, b_m_vect,
                 noiseParameters_image_noise, c_noiseParameters_orientation_n,
                 noiseParameters_pressure_noise, noiseParameters_ext_pos_noise,
                 noiseParameters_ext_att_noise, IMU_measurements,
                 b_height_offset_pressure, b_VIOParameters, b_r_data, b_r_size,
                 H, indMeas_z_data, indMeas_z_size, b_R_data, b_R_size);
    if (1.0 + (double)it == 1.0) {
      //  only do outlier rejection in first iteration
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i19 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        ar = H->size[0];
        for (i19 = 0; i19 < ar; i19++) {
          idx = b_P->size[1];
          for (i20 = 0; i20 < idx; i20++) {
            y->data[i19 + y->size[0] * i20] = 0.0;
            nx = H->size[1];
            for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
              y->data[i19 + y->size[0] * i20] += H->data[i19 + H->size[0] *
                anchorIdx] * b_P->data[anchorIdx + b_P->size[0] * i20];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i19 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        i19 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
        ar = (int)a[0] * (int)a[1];
        for (i19 = 0; i19 < ar; i19++) {
          y->data[i19] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          nx = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            i19 = idx + m;
            for (ic = idx; ic + 1 <= i19; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            ar = 0;
            i19 = br + k;
            for (ib = br; ib + 1 <= i19; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i20 = idx + m;
                for (ic = idx; ic + 1 <= i20; ic++) {
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

      i19 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
      ar = H->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = H->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          b->data[i20 + b->size[0] * i19] = H->data[i19 + H->size[0] * i20];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i19 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = y->size[0];
        b_a->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_a, i19, (int)sizeof(double));
        ar = y->size[0];
        for (i19 = 0; i19 < ar; i19++) {
          idx = b->size[1];
          for (i20 = 0; i20 < idx; i20++) {
            b_a->data[i19 + b_a->size[0] * i20] = 0.0;
            nx = y->size[1];
            for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
              b_a->data[i19 + b_a->size[0] * i20] += y->data[i19 + y->size[0] *
                anchorIdx] * b->data[anchorIdx + b->size[0] * i20];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i19 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)b_a, i19, (int)sizeof(double));
        i19 = b_a->size[0] * b_a->size[1];
        b_a->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)b_a, i19, (int)sizeof(double));
        ar = (int)((float)a[0] * (float)a[1]);
        for (i19 = 0; i19 < ar; i19++) {
          b_a->data[i19] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          nx = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            i19 = idx + m;
            for (ic = idx; ic + 1 <= i19; ic++) {
              b_a->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= nx)) {
            ar = 0;
            i19 = br + k;
            for (ib = br; ib + 1 <= i19; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i20 = idx + m;
                for (ic = idx; ic + 1 <= i20; ic++) {
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

      ii_size_idx_0 = b_a->size[0];
      ar = b_a->size[0] * b_a->size[1];
      for (i19 = 0; i19 < ar; i19++) {
        S_data[i19] = b_a->data[i19] + b_R_data[i19];
      }

      for (k = 0; k < numMeas; k++) {
        br = k << 1;
        nx = k << 1;
        idx = k << 1;
        for (i19 = 0; i19 < 2; i19++) {
          r[i19] = b_r_data[i19 + br];
          for (i20 = 0; i20 < 2; i20++) {
            d_xt[i20 + (i19 << 1)] = S_data[(i20 + nx) + ii_size_idx_0 * (i19 +
              idx)];
          }
        }

        b_mrdivide(r, d_xt, a);
        br = k << 1;
        for (i19 = 0; i19 < 2; i19++) {
          r[i19] = b_r_data[i19 + br];
        }

        if (LI_inlierStatus_data[k]) {
          //  if this feature is a LI inlier, don't also do HI update with this feature 
          br = k << 1;
          for (i19 = 0; i19 < 2; i19++) {
            b_r_data[i19 + br] = 0.0;
          }

          ar = H->size[1];
          br = k << 1;
          for (i19 = 0; i19 < ar; i19++) {
            for (i20 = 0; i20 < 2; i20++) {
              H->data[(i20 + br) + H->size[0] * i19] = 0.0;
            }
          }

          HI_inlierStatus_data[k] = false;
        } else {
          //  otherwise check if HI inlier for both cams
          d_numStatesxt = 0.0;
          for (i19 = 0; i19 < 2; i19++) {
            d_numStatesxt += a[i19] * r[i19];
          }

          if (d_numStatesxt > 6.0) {
            br = k << 1;
            for (i19 = 0; i19 < 2; i19++) {
              b_r_data[i19 + br] = 0.0;
            }

            ar = H->size[1];
            br = k << 1;
            for (i19 = 0; i19 < ar; i19++) {
              for (i20 = 0; i20 < 2; i20++) {
                H->data[(i20 + br) + H->size[0] * i19] = 0.0;
              }
            }

            HI_inlierStatus_data[k] = false;
          }
        }
      }
    }

    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i19 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ar = H->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = b_P->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          y->data[i19 + y->size[0] * i20] = 0.0;
          nx = H->size[1];
          for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
            y->data[i19 + y->size[0] * i20] += H->data[i19 + H->size[0] *
              anchorIdx] * b_P->data[anchorIdx + b_P->size[0] * i20];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      m = H->size[0];
      i19 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      i19 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ar = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ar; i19++) {
        y->data[i19] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        nx = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
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

    i19 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
    ar = H->size[0];
    for (i19 = 0; i19 < ar; i19++) {
      idx = H->size[1];
      for (i20 = 0; i20 < idx; i20++) {
        b->data[i20 + b->size[0] * i19] = H->data[i19 + H->size[0] * i20];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i19 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ar = y->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = b->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          C->data[i19 + C->size[0] * i20] = 0.0;
          nx = y->size[1];
          for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
            C->data[i19 + C->size[0] * i20] += y->data[i19 + y->size[0] *
              anchorIdx] * b->data[anchorIdx + b->size[0] * i20];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i19 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      i19 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ar = (int)((float)a[0] * (float)a[1]);
      for (i19 = 0; i19 < ar; i19++) {
        C->data[i19] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        nx = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
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

    i19 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
    ar = H->size[0];
    for (i19 = 0; i19 < ar; i19++) {
      idx = H->size[1];
      for (i20 = 0; i20 < idx; i20++) {
        b->data[i20 + b->size[0] * i19] = H->data[i19 + H->size[0] * i20];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i19 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ar = b_P->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        idx = b->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          y->data[i19 + y->size[0] * i20] = 0.0;
          nx = b_P->size[1];
          for (anchorIdx = 0; anchorIdx < nx; anchorIdx++) {
            y->data[i19 + y->size[0] * i20] += b_P->data[i19 + b_P->size[0] *
              anchorIdx] * b->data[anchorIdx + b->size[0] * i20];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i19 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      i19 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ar = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ar; i19++) {
        y->data[i19] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        nx = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= nx)) {
          ar = 0;
          i19 = br + k;
          for (ib = br; ib + 1 <= i19; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i20 = idx + m;
              for (ic = idx; ic + 1 <= i20; ic++) {
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
    ar = C->size[0] * C->size[1];
    for (i19 = 0; i19 < ar; i19++) {
      b_C_data[i19] = C->data[i19] + b_R_data[i19];
    }

    mrdivide(y, b_C_data, c_C_size, K);
    if ((K->size[1] == 1) || (b_r_size[0] == 1)) {
      i19 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
      ar = K->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        x_apo->data[i19] = 0.0;
        idx = K->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          x_apo->data[i19] += K->data[i19 + K->size[0] * i20] * b_r_data[i20];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      m = K->size[0];
      i19 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
      ar = (int)a[0];
      for (i19 = 0; i19 < ar; i19++) {
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

    for (i19 = 0; i19 < 3; i19++) {
      c_xt[i19] = b_xt->data[i19] + x_apo->data[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      b_xt->data[i19] = c_xt[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      c_xt[i19] = x_apo->data[3 + i19];
    }

    quatPlusThetaJ(c_xt, dv28);
    quatmultJ(dv28, *(double (*)[4])&b_xt->data[3], dv24);
    for (i19 = 0; i19 < 4; i19++) {
      b_xt->data[3 + i19] = dv24[i19];
    }

    if (8.0 > c_numStatesxt) {
      i19 = 0;
      i20 = 0;
    } else {
      i19 = 7;
      i20 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      anchorIdx = 0;
    } else {
      anchorIdx = 6;
    }

    if (8.0 > c_numStatesxt) {
      idx = 0;
      ii_size_idx_0 = 0;
    } else {
      idx = 7;
      ii_size_idx_0 = (int)c_numStatesxt;
    }

    br = r17->size[0] * r17->size[1];
    r17->size[0] = 1;
    r17->size[1] = ii_size_idx_0 - idx;
    emxEnsureCapacity((emxArray__common *)r17, br, (int)sizeof(int));
    ar = ii_size_idx_0 - idx;
    for (ii_size_idx_0 = 0; ii_size_idx_0 < ar; ii_size_idx_0++) {
      r17->data[r17->size[0] * ii_size_idx_0] = idx + ii_size_idx_0;
    }

    idx = f_xt->size[0];
    f_xt->size[0] = i20 - i19;
    emxEnsureCapacity((emxArray__common *)f_xt, idx, (int)sizeof(double));
    ar = i20 - i19;
    for (i20 = 0; i20 < ar; i20++) {
      f_xt->data[i20] = b_xt->data[i19 + i20] + x_apo->data[anchorIdx + i20];
    }

    ar = r17->size[1];
    for (i19 = 0; i19 < ar; i19++) {
      b_xt->data[r17->data[r17->size[0] * i19]] = f_xt->data[(*(int (*)[2])
        r17->size)[0] * i19];
    }

    for (anchorIdx = 0; anchorIdx <= numAnchors; anchorIdx++) {
      for (i19 = 0; i19 < 16; i19++) {
        x[i19] = (b_anchorFeatures->data[i19 + b_anchorFeatures->size[0] *
                  anchorIdx] == 1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0
          + numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0
          + numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 3; i19++) {
          c_xt[i19] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i19)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i19)) - 1];
        }

        for (i19 = 0; i19 < 3; i19++) {
          b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i19)) - 1] = c_xt[i19];
        }

        d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 3; i19++) {
          c_xt[i19] = x_apo->data[(int)(d_numStates + (4.0 + (double)i19)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0
          + numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          d_xt[i19] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i19)) - 1];
        }

        quatPlusThetaJ(c_xt, dv29);
        quatmultJ(dv29, d_xt, dv24);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) * (7.0
          + numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i19)) - 1] = dv24[i19];
        }

        for (nx = 0; nx < (int)numPointsPerAnchor; nx++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                             (7.0 + numPointsPerAnchor)) + 7.0) + (1.0 + (double)
            nx)) - 1] += x_apo->data[(int)(((c_numStates + ((1.0 + (double)
            anchorIdx) - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (1.0 +
            (double)nx)) - 1];
        }
      }
    }

    i19 = b_x_apo->size[0];
    b_x_apo->size[0] = x_apo->size[0];
    emxEnsureCapacity((emxArray__common *)b_x_apo, i19, (int)sizeof(double));
    ar = x_apo->size[0];
    for (i19 = 0; i19 < ar; i19++) {
      b_x_apo->data[i19] = x_apo->data[i19] - x_apo_prev->data[i19];
    }

    if (d_norm(b_x_apo) < 0.001) {
      exitg1 = true;
    } else {
      i19 = x_apo_prev->size[0];
      x_apo_prev->size[0] = x_apo->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo_prev, i19, (int)sizeof(double));
      ar = x_apo->size[0];
      for (i19 = 0; i19 < ar; i19++) {
        x_apo_prev->data[i19] = x_apo->data[i19];
      }

      it++;
    }
  }

  emxFree_real_T(&f_xt);
  emxFree_real_T(&b_x_apo);
  emxFree_real_T(&y);
  emxFree_int32_T(&r17);
  emxFree_real_T(&featureAnchorInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&map);
  emxFree_real_T(&x_apo_prev);
  emxFree_real_T(&x_apo);
  b_eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
         b_VIOParameters.num_points_per_anchor), b_a);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i19 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
    loop_ub = K->size[0];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ar = H->size[1];
      for (i20 = 0; i20 < ar; i20++) {
        C->data[i19 + C->size[0] * i20] = 0.0;
        idx = K->size[1];
        for (anchorIdx = 0; anchorIdx < idx; anchorIdx++) {
          C->data[i19 + C->size[0] * i20] += K->data[i19 + K->size[0] *
            anchorIdx] * H->data[anchorIdx + H->size[0] * i20];
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
      nx = K->size[0] * (H->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= nx)) {
        i19 = idx + m;
        for (ic = idx; ic + 1 <= i19; ic++) {
          C->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= nx)) {
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
  nx = b_a->size[0];
  idx = b_a->size[1];
  loop_ub = nx * idx;
  for (i19 = 0; i19 < loop_ub; i19++) {
    b_a->data[i19] -= C->data[i19];
  }

  emxFree_real_T(&C);
  i19 = b->size[0] * b->size[1];
  b->size[0] = b_P->size[0];
  b->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
  loop_ub = b_P->size[0] * b_P->size[1];
  for (i19 = 0; i19 < loop_ub; i19++) {
    b->data[i19] = b_P->data[i19];
  }

  emxInit_real_T(&d_a, 2);
  if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
    i19 = d_a->size[0] * d_a->size[1];
    d_a->size[0] = b_a->size[0];
    d_a->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)d_a, i19, (int)sizeof(double));
    loop_ub = b_a->size[0];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ar = b_P->size[1];
      for (i20 = 0; i20 < ar; i20++) {
        d_a->data[i19 + d_a->size[0] * i20] = 0.0;
        idx = b_a->size[1];
        for (anchorIdx = 0; anchorIdx < idx; anchorIdx++) {
          d_a->data[i19 + d_a->size[0] * i20] += b_a->data[i19 + b_a->size[0] *
            anchorIdx] * b_P->data[anchorIdx + b_P->size[0] * i20];
        }
      }
    }

    i19 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = d_a->size[0];
    b_P->size[1] = d_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i19, (int)sizeof(double));
    loop_ub = d_a->size[1];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ar = d_a->size[0];
      for (i20 = 0; i20 < ar; i20++) {
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
      ar = (int)a[0];
      for (i20 = 0; i20 < ar; i20++) {
        b_P->data[i20 + b_P->size[0] * i19] = 0.0;
      }
    }

    if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      nx = b_a->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= nx)) {
        i19 = idx + m;
        for (ic = idx; ic + 1 <= i19; ic++) {
          b_P->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= nx)) {
        ar = 0;
        i19 = br + k;
        for (ib = br; ib + 1 <= i19; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
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
  nx = LI_inlierStatus_size_idx_0 - 1;
  br = 0;
  for (idx = 0; idx <= nx; idx++) {
    if (LI_inlierStatus_data[idx] || HI_inlierStatus_data[idx]) {
      br++;
    }
  }

  validFeatures_size[0] = br;
  ii_size_idx_0 = 0;
  for (idx = 0; idx <= nx; idx++) {
    if (LI_inlierStatus_data[idx] || HI_inlierStatus_data[idx]) {
      validFeatures_data[ii_size_idx_0] = indMeas_data[idx];
      ii_size_idx_0++;
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
    ros_warn();
  }
}

//
// File trailer for OnePointRANSAC_EKF.cpp
//
// [EOF]
//
