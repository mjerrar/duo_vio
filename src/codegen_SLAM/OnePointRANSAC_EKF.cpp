//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 21:11:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "OnePointRANSAC_EKF.h"
#include "predictMeasurement_stereo.h"
#include "get_r_u.h"
#include "mrdivide.h"
#include "quatmultJ.h"
#include "quatPlusThetaJ.h"
#include "any.h"
#include "ros_warn.h"
#include "SLAM_emxutil.h"
#include "eye.h"
#include "norm.h"
#include "getH_R_res.h"
#include "getMap.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : emxArray_real_T *b_xt
//                emxArray_real_T *b_P
//                double z_all_l[64]
//                double c_numStatesxt
//                double c_numStates
//                const double c_cameraparams_CameraParameters[3]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const emxArray_real_T *b_anchorFeatures
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double c_VIOParameters_num_points_per_
//                double VIOParameters_num_anchors
//                double c_VIOParameters_max_ekf_iterati
//                double validFeatures_data[]
//                int validFeatures_size[1]
// Return Type  : void
//
void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P, double
  z_all_l[64], double c_numStatesxt, double c_numStates, const double
  c_cameraparams_CameraParameters[3], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const emxArray_real_T *b_anchorFeatures,
  const emxArray_real_T *b_m_vect, const double noiseParameters_image_noise[2],
  double c_VIOParameters_num_points_per_, double VIOParameters_num_anchors,
  double c_VIOParameters_max_ekf_iterati, double validFeatures_data[], int
  validFeatures_size[1])
{
  emxArray_boolean_T *c_anchorFeatures;
  int i18;
  int loop_ub;
  boolean_T x[32];
  int idx;
  int ii_data[32];
  int ii;
  boolean_T exitg2;
  boolean_T guard1 = false;
  signed char indMeas_data[32];
  emxArray_real_T *K;
  emxArray_real_T *H;
  signed char x_data[64];
  int nx;
  double indMeas_z_data[64];
  int indMeas_z_size[1];
  int k;
  int unusedU2_size[1];
  double unusedU2_data[64];
  double k1_l;
  double k2_l;
  double k3_l;
  double d_numStatesxt;
  int ar;
  double pt_d_n_idx_0;
  double pt_d_n_idx_1;
  double r_u;
  double coeff;
  emxArray_real_T *map;
  emxArray_real_T *anchorInd;
  emxArray_real_T *featureAnchorInd;
  boolean_T HI_inlierStatus_data[32];
  emxArray_real_T *x_apo_prev;
  int ib;
  int it;
  emxArray_real_T *x_apo;
  emxArray_real_T *b_H;
  emxArray_real_T *C;
  emxArray_int32_T *r16;
  emxArray_real_T *y;
  emxArray_real_T *b_y;
  emxArray_real_T *b;
  emxArray_real_T *b_x_apo;
  emxArray_real_T *c_H;
  emxArray_real_T *d_H;
  emxArray_real_T *e_H;
  emxArray_real_T *c_xt;
  boolean_T exitg1;
  double b_indMeas_data[32];
  int indMeas_size[1];
  int R_size[2];
  double R_data[4096];
  double r_data[74];
  int i19;
  double a[2];
  int m;
  int ic;
  int ia;
  static double S_data[5476];
  double r[2];
  double S[4];
  double C_data[4096];
  int C_size[2];
  double d_xt[3];
  double dv20[4];
  double e_numStatesxt;
  double d_numStates;
  double e_xt[4];
  double dv21[4];
  emxArray_real_T *c_y;
  emxInit_boolean_T(&c_anchorFeatures, 2);

  //  threshold for LI innovation test
  // 9.487729036781154; % HI mahalanobis gate
  //  threshold for minimum LI supporter
  i18 = c_anchorFeatures->size[0] * c_anchorFeatures->size[1];
  c_anchorFeatures->size[0] = 32;
  c_anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)c_anchorFeatures, i18, (int)sizeof
                    (boolean_T));
  loop_ub = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i18 = 0; i18 < loop_ub; i18++) {
    c_anchorFeatures->data[i18] = (b_anchorFeatures->data[i18] == 1.0);
  }

  b_any(c_anchorFeatures, x);
  idx = 0;
  ii = 1;
  emxFree_boolean_T(&c_anchorFeatures);
  exitg2 = false;
  while ((!exitg2) && (ii < 33)) {
    guard1 = false;
    if (x[ii - 1]) {
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
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  if (1 > idx) {
    idx = 0;
  }

  for (i18 = 0; i18 < loop_ub; i18++) {
    indMeas_data[i18] = (signed char)ii_data[i18];
  }

  emxInit_real_T(&K, 2);
  emxInit_real_T(&H, 2);
  i18 = K->size[0] * K->size[1];
  K->size[0] = 1;
  K->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)K, i18, (int)sizeof(double));
  K->data[0] = 0.0;

  //  for coder
  i18 = H->size[0] * H->size[1];
  H->size[0] = 1;
  H->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)H, i18, (int)sizeof(double));
  H->data[0] = 0.0;

  //  for coder
  for (i18 = 0; i18 < idx; i18++) {
    x_data[i18 << 1] = (signed char)((indMeas_data[i18] - 1) * 2 + 1);
  }

  for (i18 = 0; i18 < idx; i18++) {
    x_data[1 + (i18 << 1)] = (signed char)((indMeas_data[i18] - 1) * 2 + 2);
  }

  nx = loop_ub << 1;
  indMeas_z_size[0] = nx;
  for (k = 0; k + 1 <= nx; k++) {
    indMeas_z_data[k] = x_data[k];
  }

  // undistortPoint Undistort a distorted pixel point
  //    pt_d is a distorted pixel point (or a column vector of distorted points) 
  //    from a camera with camera parameters cameraParameters.
  //    pt_u is a undistorted pixel point (or a column vector of undistorted points) 
  //    in normalized pixel coordinates, i.e.
  //    the principal point is at (0, 0), the focal length is (1, 1)
  unusedU2_size[0] = nx;
  for (i18 = 0; i18 < nx; i18++) {
    unusedU2_data[i18] = z_all_l[(int)indMeas_z_data[i18] - 1];
  }

  k1_l = c_cameraparams_CameraParameters[0];
  k2_l = c_cameraparams_CameraParameters[1];
  k3_l = c_cameraparams_CameraParameters[2];
  d_numStatesxt = (double)nx / 2.0;
  for (ar = 0; ar < (int)d_numStatesxt; ar++) {
    pt_d_n_idx_0 = (z_all_l[(int)indMeas_z_data[ar << 1] - 1] -
                    e_cameraparams_CameraParameters[0]) /
      d_cameraparams_CameraParameters[0];
    pt_d_n_idx_1 = (z_all_l[(int)indMeas_z_data[(ar << 1) + 1] - 1] -
                    e_cameraparams_CameraParameters[1]) /
      d_cameraparams_CameraParameters[1];
    r_u = get_r_u(k1_l, k2_l, k3_l, sqrt(pt_d_n_idx_0 * pt_d_n_idx_0 +
      pt_d_n_idx_1 * pt_d_n_idx_1));
    coeff = ((1.0 + k1_l * (r_u * r_u)) + k2_l * rt_powd_snf(r_u, 4.0)) + k3_l *
      rt_powd_snf(r_u, 6.0);
    ii = ar << 1;
    unusedU2_data[ii] = pt_d_n_idx_0 / coeff;
    unusedU2_data[1 + ii] = pt_d_n_idx_1 / coeff;
  }

  for (i18 = 0; i18 < nx; i18++) {
    z_all_l[(int)indMeas_z_data[i18] - 1] = unusedU2_data[i18];
  }

  emxInit_real_T(&map, 2);
  b_emxInit_real_T(&anchorInd, 1);
  b_emxInit_real_T(&featureAnchorInd, 1);

  // % B 1-point hypotheses generation and evaluation
  //  low innovation inlier status
  //  build the map according to the current estimate
  getMap(b_xt, b_anchorFeatures, b_m_vect, (double)b_anchorFeatures->size[1] *
         c_VIOParameters_num_points_per_, c_numStatesxt, 7.0 +
         c_VIOParameters_num_points_per_, map, anchorInd, featureAnchorInd);

  //      ros_warn('1-Point RANSAC: Ended hypothesis test. Found %i LI inliers, which is below the threshold %i. Not doing LI EKF update.', int32(nnz(LI_inlierStatus)), int32(minimum_support_thresh)) 
  // % D Partial EKF update using high-innovation inliers
  for (i18 = 0; i18 < idx; i18++) {
    HI_inlierStatus_data[i18] = true;
  }

  b_emxInit_real_T(&x_apo_prev, 1);

  //  high innovation inliers
  i18 = x_apo_prev->size[0];
  x_apo_prev->size[0] = b_P->size[0];
  emxEnsureCapacity((emxArray__common *)x_apo_prev, i18, (int)sizeof(double));
  ib = b_P->size[0];
  for (i18 = 0; i18 < ib; i18++) {
    x_apo_prev->data[i18] = 0.0;
  }

  it = 0;
  b_emxInit_real_T(&x_apo, 1);
  emxInit_real_T(&b_H, 2);
  emxInit_real_T(&C, 2);
  b_emxInit_int32_T(&r16, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b_y, 2);
  emxInit_real_T(&b, 2);
  b_emxInit_real_T(&b_x_apo, 1);
  emxInit_real_T(&c_H, 2);
  emxInit_real_T(&d_H, 2);
  emxInit_real_T(&e_H, 2);
  b_emxInit_real_T(&c_xt, 1);
  exitg1 = false;
  while ((!exitg1) && (it <= (int)c_VIOParameters_max_ekf_iterati - 1)) {
    ii = loop_ub - 1;
    nx = 0;
    for (ar = 0; ar <= ii; ar++) {
      if (HI_inlierStatus_data[ar]) {
        nx++;
      }
    }

    idx = 0;
    for (ar = 0; ar <= ii; ar++) {
      if (HI_inlierStatus_data[ar]) {
        ii_data[idx] = ar + 1;
        idx++;
      }
    }

    indMeas_size[0] = nx;
    for (i18 = 0; i18 < nx; i18++) {
      b_indMeas_data[i18] = indMeas_data[ii_data[i18] - 1];
    }

    getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
               indMeas_size, map, anchorInd, featureAnchorInd, b_m_vect,
               noiseParameters_image_noise, c_VIOParameters_num_points_per_,
               VIOParameters_num_anchors, indMeas_z_data, indMeas_z_size, b_H,
               unusedU2_data, unusedU2_size, R_data, R_size);
    ib = indMeas_z_size[0];
    for (i18 = 0; i18 < ib; i18++) {
      r_data[i18] = indMeas_z_data[i18];
    }

    i18 = H->size[0] * H->size[1];
    H->size[0] = b_H->size[0];
    H->size[1] = b_H->size[1];
    emxEnsureCapacity((emxArray__common *)H, i18, (int)sizeof(double));
    ib = b_H->size[0] * b_H->size[1];
    for (i18 = 0; i18 < ib; i18++) {
      H->data[i18] = b_H->data[i18];
    }

    if (1.0 + (double)it == 1.0) {
      //  only do outlier rejection in first iteration
      if ((b_H->size[1] == 1) || (b_P->size[0] == 1)) {
        i18 = y->size[0] * y->size[1];
        y->size[0] = b_H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
        ib = b_H->size[0];
        for (i18 = 0; i18 < ib; i18++) {
          idx = b_P->size[1];
          for (i19 = 0; i19 < idx; i19++) {
            y->data[i18 + y->size[0] * i19] = 0.0;
            ii = b_H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i18 + y->size[0] * i19] += b_H->data[i18 + b_H->size[0] *
                ar] * b_P->data[ar + b_P->size[0] * i19];
            }
          }
        }
      } else {
        k = b_H->size[1];
        a[0] = b_H->size[0];
        a[1] = b_P->size[1];
        m = b_H->size[0];
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

        if ((b_H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = b_H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i18 = idx + m;
            for (ic = idx; ic + 1 <= i18; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          nx = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i18 = nx + k;
            for (ib = nx; ib + 1 <= i18; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i19 = idx + m;
                for (ic = idx; ic + 1 <= i19; ic++) {
                  ia++;
                  y->data[ic] += b_P->data[ib] * b_H->data[ia - 1];
                }
              }

              ar += m;
            }

            nx += k;
            idx += m;
          }
        }
      }

      i18 = c_H->size[0] * c_H->size[1];
      c_H->size[0] = b_H->size[1];
      c_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)c_H, i18, (int)sizeof(double));
      ib = b_H->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        idx = b_H->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          c_H->data[i19 + c_H->size[0] * i18] = b_H->data[i18 + b_H->size[0] *
            i19];
        }
      }

      ii = b_H->size[1];
      idx = b_H->size[0];
      i18 = b->size[0] * b->size[1];
      b->size[0] = ii;
      b->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
      for (i18 = 0; i18 < idx; i18++) {
        for (i19 = 0; i19 < ii; i19++) {
          b->data[i19 + b->size[0] * i18] = c_H->data[i19 + ii * i18];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i18 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = y->size[0];
        b_y->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_y, i18, (int)sizeof(double));
        ib = y->size[0];
        for (i18 = 0; i18 < ib; i18++) {
          idx = b->size[1];
          for (i19 = 0; i19 < idx; i19++) {
            b_y->data[i18 + b_y->size[0] * i19] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              b_y->data[i18 + b_y->size[0] * i19] += y->data[i18 + y->size[0] *
                ar] * b->data[ar + b->size[0] * i19];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i18 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)b_y, i18, (int)sizeof(double));
        i18 = b_y->size[0] * b_y->size[1];
        b_y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)b_y, i18, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i18 = 0; i18 < ib; i18++) {
          b_y->data[i18] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i18 = idx + m;
            for (ic = idx; ic + 1 <= i18; ic++) {
              b_y->data[ic] = 0.0;
            }

            idx += m;
          }

          nx = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i18 = nx + k;
            for (ib = nx; ib + 1 <= i18; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i19 = idx + m;
                for (ic = idx; ic + 1 <= i19; ic++) {
                  ia++;
                  b_y->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += m;
            }

            nx += k;
            idx += m;
          }
        }
      }

      nx = b_y->size[0];
      ib = b_y->size[0] * b_y->size[1];
      for (i18 = 0; i18 < ib; i18++) {
        S_data[i18] = b_y->data[i18] + R_data[i18];
      }

      for (k = 0; k < loop_ub; k++) {
        ar = k << 1;
        ii = k << 1;
        idx = k << 1;
        for (i18 = 0; i18 < 2; i18++) {
          r[i18] = r_data[i18 + ar];
          for (i19 = 0; i19 < 2; i19++) {
            S[i19 + (i18 << 1)] = S_data[(i19 + ii) + nx * (i18 + idx)];
          }
        }

        b_mrdivide(r, S, a);
        ar = k << 1;

        //  otherwise check if HI inlier for both cams
        d_numStatesxt = 0.0;
        for (i18 = 0; i18 < 2; i18++) {
          d_numStatesxt += a[i18] * r_data[i18 + ar];
        }

        if (d_numStatesxt > 6.0) {
          ar = k << 1;
          for (i18 = 0; i18 < 2; i18++) {
            r_data[i18 + ar] = 0.0;
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

        nx = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i18 = nx + k;
          for (ib = nx; ib + 1 <= i18; ib++) {
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

          nx += k;
          idx += m;
        }
      }
    }

    i18 = d_H->size[0] * d_H->size[1];
    d_H->size[0] = H->size[1];
    d_H->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)d_H, i18, (int)sizeof(double));
    ib = H->size[0];
    for (i18 = 0; i18 < ib; i18++) {
      idx = H->size[1];
      for (i19 = 0; i19 < idx; i19++) {
        d_H->data[i19 + d_H->size[0] * i18] = H->data[i18 + H->size[0] * i19];
      }
    }

    ii = H->size[1];
    idx = H->size[0];
    i18 = b->size[0] * b->size[1];
    b->size[0] = ii;
    b->size[1] = idx;
    emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
    for (i18 = 0; i18 < idx; i18++) {
      for (i19 = 0; i19 < ii; i19++) {
        b->data[i19 + b->size[0] * i18] = d_H->data[i19 + ii * i18];
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

        nx = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i18 = nx + k;
          for (ib = nx; ib + 1 <= i18; ib++) {
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

          nx += k;
          idx += m;
        }
      }
    }

    i18 = e_H->size[0] * e_H->size[1];
    e_H->size[0] = H->size[1];
    e_H->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)e_H, i18, (int)sizeof(double));
    ib = H->size[0];
    for (i18 = 0; i18 < ib; i18++) {
      idx = H->size[1];
      for (i19 = 0; i19 < idx; i19++) {
        e_H->data[i19 + e_H->size[0] * i18] = H->data[i18 + H->size[0] * i19];
      }
    }

    ii = H->size[1];
    idx = H->size[0];
    i18 = b->size[0] * b->size[1];
    b->size[0] = ii;
    b->size[1] = idx;
    emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
    for (i18 = 0; i18 < idx; i18++) {
      for (i19 = 0; i19 < ii; i19++) {
        b->data[i19 + b->size[0] * i18] = e_H->data[i19 + ii * i18];
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

        nx = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i18 = nx + k;
          for (ib = nx; ib + 1 <= i18; ib++) {
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

          nx += k;
          idx += m;
        }
      }
    }

    C_size[0] = C->size[0];
    C_size[1] = C->size[1];
    ib = C->size[0] * C->size[1];
    for (i18 = 0; i18 < ib; i18++) {
      C_data[i18] = C->data[i18] + R_data[i18];
    }

    mrdivide(y, C_data, C_size, K);
    if ((K->size[1] == 1) || (indMeas_z_size[0] == 1)) {
      i18 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i18, (int)sizeof(double));
      ib = K->size[0];
      for (i18 = 0; i18 < ib; i18++) {
        x_apo->data[i18] = 0.0;
        idx = K->size[1];
        for (i19 = 0; i19 < idx; i19++) {
          x_apo->data[i18] += K->data[i18 + K->size[0] * i19] * r_data[i19];
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

        nx = 0;
        idx = 0;
        while ((m > 0) && (idx <= 0)) {
          ar = 0;
          i18 = nx + k;
          for (ib = nx; ib + 1 <= i18; ib++) {
            if (r_data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                x_apo->data[ic] += r_data[ib] * K->data[ia - 1];
              }
            }

            ar += m;
          }

          nx += k;
          idx = m;
        }
      }
    }

    for (i18 = 0; i18 < 3; i18++) {
      d_xt[i18] = b_xt->data[i18] + x_apo->data[i18];
    }

    for (i18 = 0; i18 < 3; i18++) {
      b_xt->data[i18] = d_xt[i18];
    }

    for (i18 = 0; i18 < 3; i18++) {
      d_xt[i18] = x_apo->data[3 + i18];
    }

    quatPlusThetaJ(d_xt, dv20);
    quatmultJ(dv20, *(double (*)[4])&b_xt->data[3], S);
    for (i18 = 0; i18 < 4; i18++) {
      b_xt->data[3 + i18] = S[i18];
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
      nx = 0;
    } else {
      idx = 7;
      nx = (int)c_numStatesxt;
    }

    ii = r16->size[0] * r16->size[1];
    r16->size[0] = 1;
    r16->size[1] = nx - idx;
    emxEnsureCapacity((emxArray__common *)r16, ii, (int)sizeof(int));
    ib = nx - idx;
    for (nx = 0; nx < ib; nx++) {
      r16->data[r16->size[0] * nx] = idx + nx;
    }

    idx = c_xt->size[0];
    c_xt->size[0] = i19 - i18;
    emxEnsureCapacity((emxArray__common *)c_xt, idx, (int)sizeof(double));
    ib = i19 - i18;
    for (i19 = 0; i19 < ib; i19++) {
      c_xt->data[i19] = b_xt->data[i18 + i19] + x_apo->data[ar + i19];
    }

    ib = r16->size[1];
    for (i18 = 0; i18 < ib; i18++) {
      b_xt->data[r16->data[r16->size[0] * i18]] = c_xt->data[(*(int (*)[2])
        r16->size)[0] * i18];
    }

    for (idx = 0; idx < b_anchorFeatures->size[1]; idx++) {
      for (i18 = 0; i18 < 32; i18++) {
        x[i18] = (b_anchorFeatures->data[i18 + b_anchorFeatures->size[0] * idx] ==
                  1.0);
      }

      if (any(x)) {
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0 +
          c_VIOParameters_num_points_per_);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0 +
          c_VIOParameters_num_points_per_);
        d_numStates = c_numStates + ((1.0 + (double)idx) - 1.0) * (6.0 +
          c_VIOParameters_num_points_per_);
        for (i18 = 0; i18 < 3; i18++) {
          d_xt[i18] = b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i18)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i18)) - 1];
        }

        for (i18 = 0; i18 < 3; i18++) {
          b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i18)) - 1] = d_xt[i18];
        }

        d_numStates = c_numStates + ((1.0 + (double)idx) - 1.0) * (6.0 +
          c_VIOParameters_num_points_per_);
        for (i18 = 0; i18 < 3; i18++) {
          d_xt[i18] = x_apo->data[(int)(d_numStates + (4.0 + (double)i18)) - 1];
        }

        d_numStatesxt = c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0 +
          c_VIOParameters_num_points_per_);
        for (i18 = 0; i18 < 4; i18++) {
          e_xt[i18] = b_xt->data[(int)(d_numStatesxt + (4.0 + (double)i18)) - 1];
        }

        quatPlusThetaJ(d_xt, dv21);
        quatmultJ(dv21, e_xt, S);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0 +
          c_VIOParameters_num_points_per_);
        for (i18 = 0; i18 < 4; i18++) {
          b_xt->data[(int)(d_numStatesxt + (4.0 + (double)i18)) - 1] = S[i18];
        }

        for (ii = 0; ii < (int)c_VIOParameters_num_points_per_; ii++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0
            + c_VIOParameters_num_points_per_)) + 7.0) + (1.0 + (double)ii)) - 1]
            += x_apo->data[(int)(((c_numStates + ((1.0 + (double)idx) - 1.0) *
            (6.0 + c_VIOParameters_num_points_per_)) + 6.0) + (1.0 + (double)ii))
            - 1];
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

  emxFree_real_T(&c_xt);
  emxFree_real_T(&e_H);
  emxFree_real_T(&d_H);
  emxFree_real_T(&c_H);
  emxFree_real_T(&b_x_apo);
  emxFree_real_T(&y);
  emxFree_int32_T(&r16);
  emxFree_real_T(&b_H);
  emxFree_real_T(&featureAnchorInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&map);
  emxFree_real_T(&x_apo_prev);
  emxFree_real_T(&x_apo);
  b_eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
         c_VIOParameters_num_points_per_), b_y);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i18 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
    ib = K->size[0];
    for (i18 = 0; i18 < ib; i18++) {
      idx = H->size[1];
      for (i19 = 0; i19 < idx; i19++) {
        C->data[i18 + C->size[0] * i19] = 0.0;
        ii = K->size[1];
        for (ar = 0; ar < ii; ar++) {
          C->data[i18 + C->size[0] * i19] += K->data[i18 + K->size[0] * ar] *
            H->data[ar + H->size[0] * i19];
        }
      }
    }
  } else {
    k = K->size[1];
    a[0] = (unsigned int)K->size[0];
    a[1] = (unsigned int)H->size[1];
    m = K->size[0];
    i18 = C->size[0] * C->size[1];
    C->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
    i18 = C->size[0] * C->size[1];
    C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)C, i18, (int)sizeof(double));
    ib = (int)a[0] * (int)a[1];
    for (i18 = 0; i18 < ib; i18++) {
      C->data[i18] = 0.0;
    }

    if ((K->size[0] == 0) || (H->size[1] == 0)) {
    } else {
      ii = K->size[0] * (H->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i18 = idx + m;
        for (ic = idx; ic + 1 <= i18; ic++) {
          C->data[ic] = 0.0;
        }

        idx += m;
      }

      nx = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i18 = nx + k;
        for (ib = nx; ib + 1 <= i18; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            i19 = idx + m;
            for (ic = idx; ic + 1 <= i19; ic++) {
              ia++;
              C->data[ic] += H->data[ib] * K->data[ia - 1];
            }
          }

          ar += m;
        }

        nx += k;
        idx += m;
      }
    }
  }

  emxFree_real_T(&H);
  emxFree_real_T(&K);
  i18 = b_y->size[0] * b_y->size[1];
  emxEnsureCapacity((emxArray__common *)b_y, i18, (int)sizeof(double));
  ii = b_y->size[0];
  idx = b_y->size[1];
  ib = ii * idx;
  for (i18 = 0; i18 < ib; i18++) {
    b_y->data[i18] -= C->data[i18];
  }

  emxFree_real_T(&C);
  i18 = b->size[0] * b->size[1];
  b->size[0] = b_P->size[0];
  b->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b, i18, (int)sizeof(double));
  ib = b_P->size[0] * b_P->size[1];
  for (i18 = 0; i18 < ib; i18++) {
    b->data[i18] = b_P->data[i18];
  }

  emxInit_real_T(&c_y, 2);
  if ((b_y->size[1] == 1) || (b_P->size[0] == 1)) {
    i18 = c_y->size[0] * c_y->size[1];
    c_y->size[0] = b_y->size[0];
    c_y->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)c_y, i18, (int)sizeof(double));
    ib = b_y->size[0];
    for (i18 = 0; i18 < ib; i18++) {
      idx = b_P->size[1];
      for (i19 = 0; i19 < idx; i19++) {
        c_y->data[i18 + c_y->size[0] * i19] = 0.0;
        ii = b_y->size[1];
        for (ar = 0; ar < ii; ar++) {
          c_y->data[i18 + c_y->size[0] * i19] += b_y->data[i18 + b_y->size[0] *
            ar] * b_P->data[ar + b_P->size[0] * i19];
        }
      }
    }

    i18 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = c_y->size[0];
    b_P->size[1] = c_y->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i18, (int)sizeof(double));
    ib = c_y->size[1];
    for (i18 = 0; i18 < ib; i18++) {
      idx = c_y->size[0];
      for (i19 = 0; i19 < idx; i19++) {
        b_P->data[i19 + b_P->size[0] * i18] = c_y->data[i19 + c_y->size[0] * i18];
      }
    }
  } else {
    k = b_y->size[1];
    a[0] = b_y->size[0];
    a[1] = b_P->size[1];
    m = b_y->size[0];
    i18 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = (int)a[0];
    b_P->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)b_P, i18, (int)sizeof(double));
    ib = (int)a[1];
    for (i18 = 0; i18 < ib; i18++) {
      idx = (int)a[0];
      for (i19 = 0; i19 < idx; i19++) {
        b_P->data[i19 + b_P->size[0] * i18] = 0.0;
      }
    }

    if ((b_y->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = b_y->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i18 = idx + m;
        for (ic = idx; ic + 1 <= i18; ic++) {
          b_P->data[ic] = 0.0;
        }

        idx += m;
      }

      nx = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i18 = nx + k;
        for (ib = nx; ib + 1 <= i18; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i19 = idx + m;
            for (ic = idx; ic + 1 <= i19; ic++) {
              ia++;
              b_P->data[ic] += b->data[ib] * b_y->data[ia - 1];
            }
          }

          ar += m;
        }

        nx += k;
        idx += m;
      }
    }
  }

  emxFree_real_T(&c_y);
  emxFree_real_T(&b);
  emxFree_real_T(&b_y);
  ii = loop_ub - 1;
  nx = 0;
  for (ar = 0; ar <= ii; ar++) {
    if (HI_inlierStatus_data[ar]) {
      nx++;
    }
  }

  validFeatures_size[0] = nx;
  idx = 0;
  for (ar = 0; ar <= ii; ar++) {
    if (HI_inlierStatus_data[ar]) {
      validFeatures_data[idx] = indMeas_data[ar];
      idx++;
    }
  }

  //  if length(validFeatures) ~= numMeas
  //      rejected = setdiff(indMeas, validFeatures);
  //      fprintf('%i of %i features are valid after RANSAC.\n', int8(length(validFeatures)), numMeas) 
  //      fprintf(' Rejected: %s\n', mat2str(rejected))
  //  end
  nx = 0;
  for (ar = 0; ar < loop_ub; ar++) {
    if (HI_inlierStatus_data[ar]) {
      nx++;
    }
  }

  if (nx == 0) {
    ros_warn();
  }
}

//
// File trailer for OnePointRANSAC_EKF.cpp
//
// [EOF]
//
