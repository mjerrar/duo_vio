//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 22:40:16
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "OnePointRANSAC_EKF.h"
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
//                double z_all_l[48]
//                double c_numStatesxt
//                double c_numStates
//                const double c_cameraparams_CameraParameters[3]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const emxArray_real_T *b_anchorFeatures
//                const emxArray_real_T *b_m_vect
//                const double noiseParameters_image_noise[2]
//                double noiseParameters_pressure_noise
//                double noiseParameters_ext_pos_noise
//                double noiseParameters_ext_att_noise
//                double c_noiseParameters_gravity_align
//                const VIOMeasurements *IMU_measurements
//                double b_height_offset_pressure
//                const VIOParameters b_VIOParameters
//                double validFeatures_data[]
//                int validFeatures_size[1]
// Return Type  : void
//
void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P, double
  z_all_l[48], double c_numStatesxt, double c_numStates, const double
  c_cameraparams_CameraParameters[3], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const emxArray_real_T *b_anchorFeatures,
  const emxArray_real_T *b_m_vect, const double noiseParameters_image_noise[2],
  double noiseParameters_pressure_noise, double noiseParameters_ext_pos_noise,
  double noiseParameters_ext_att_noise, double c_noiseParameters_gravity_align,
  const VIOMeasurements *IMU_measurements, double b_height_offset_pressure,
  const VIOParameters b_VIOParameters, double validFeatures_data[], int
  validFeatures_size[1])
{
  emxArray_boolean_T *c_anchorFeatures;
  double numPointsPerAnchor;
  int i22;
  int loop_ub;
  boolean_T x[24];
  int idx;
  int ii_data[24];
  int ii;
  boolean_T exitg2;
  boolean_T guard1 = false;
  signed char indMeas_data[24];
  emxArray_real_T *K;
  emxArray_real_T *H;
  signed char x_data[48];
  int nx;
  double indMeas_z_data[48];
  int indMeas_z_size[1];
  int k;
  double tmp_data[48];
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
  boolean_T HI_inlierStatus_data[24];
  emxArray_real_T *x_apo_prev;
  int ib;
  int it;
  emxArray_real_T *x_apo;
  emxArray_real_T *C;
  emxArray_int32_T *r10;
  emxArray_real_T *y;
  emxArray_real_T *b_y;
  emxArray_real_T *b;
  emxArray_real_T *b_x_apo;
  emxArray_real_T *c_xt;
  boolean_T exitg1;
  double b_indMeas_data[24];
  int indMeas_size[1];
  int R_size[2];
  double R_data[3364];
  int r_size[1];
  double r_data[58];
  int i23;
  double a[2];
  int m;
  int ic;
  int ia;
  double S_data[3364];
  double r[2];
  double S[4];
  double C_data[3364];
  int C_size[2];
  double d_xt[3];
  double dv11[4];
  double e_numStatesxt;
  double d_numStates;
  double e_xt[4];
  double dv12[4];
  emxArray_real_T *c_y;
  b_emxInit_boolean_T(&c_anchorFeatures, 2);

  //  threshold for LI innovation test
  // 9.487729036781154; % HI mahalanobis gate
  //  threshold for minimum LI supporter
  numPointsPerAnchor = b_VIOParameters.num_points_per_anchor;
  i22 = c_anchorFeatures->size[0] * c_anchorFeatures->size[1];
  c_anchorFeatures->size[0] = 24;
  c_anchorFeatures->size[1] = b_anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)c_anchorFeatures, i22, (int)sizeof
                    (boolean_T));
  loop_ub = b_anchorFeatures->size[0] * b_anchorFeatures->size[1];
  for (i22 = 0; i22 < loop_ub; i22++) {
    c_anchorFeatures->data[i22] = (b_anchorFeatures->data[i22] == 1.0);
  }

  b_any(c_anchorFeatures, x);
  idx = 0;
  ii = 1;
  emxFree_boolean_T(&c_anchorFeatures);
  exitg2 = false;
  while ((!exitg2) && (ii < 25)) {
    guard1 = false;
    if (x[ii - 1]) {
      idx++;
      ii_data[idx - 1] = ii;
      if (idx >= 24) {
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

  for (i22 = 0; i22 < loop_ub; i22++) {
    indMeas_data[i22] = (signed char)ii_data[i22];
  }

  emxInit_real_T(&K, 2);
  emxInit_real_T(&H, 2);
  i22 = K->size[0] * K->size[1];
  K->size[0] = 1;
  K->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)K, i22, (int)sizeof(double));
  K->data[0] = 0.0;

  //  for coder
  i22 = H->size[0] * H->size[1];
  H->size[0] = 1;
  H->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)H, i22, (int)sizeof(double));
  H->data[0] = 0.0;

  //  for coder
  for (i22 = 0; i22 < idx; i22++) {
    x_data[i22 << 1] = (signed char)((indMeas_data[i22] - 1) * 2 + 1);
  }

  for (i22 = 0; i22 < idx; i22++) {
    x_data[1 + (i22 << 1)] = (signed char)((indMeas_data[i22] - 1) * 2 + 2);
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
  for (i22 = 0; i22 < nx; i22++) {
    tmp_data[i22] = z_all_l[(int)indMeas_z_data[i22] - 1];
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
    tmp_data[ii] = pt_d_n_idx_0 / coeff;
    tmp_data[1 + ii] = pt_d_n_idx_1 / coeff;
  }

  for (i22 = 0; i22 < nx; i22++) {
    z_all_l[(int)indMeas_z_data[i22] - 1] = tmp_data[i22];
  }

  emxInit_real_T(&map, 2);
  b_emxInit_real_T(&anchorInd, 1);
  b_emxInit_real_T(&featureAnchorInd, 1);

  // % B 1-point hypotheses generation and evaluation
  //  low innovation inlier status
  //  build the map according to the current estimate
  getMap(b_xt, b_anchorFeatures, b_m_vect, (double)b_anchorFeatures->size[1] *
         b_VIOParameters.num_points_per_anchor, c_numStatesxt, 7.0 +
         b_VIOParameters.num_points_per_anchor, map, anchorInd, featureAnchorInd);

  //      ros_warn('1-Point RANSAC: Ended hypothesis test. Found %i LI inliers, which is below the threshold %i. Not doing LI EKF update.', int32(nnz(LI_inlierStatus)), int32(minimum_support_thresh)) 
  // % D Partial EKF update using high-innovation inliers
  for (i22 = 0; i22 < idx; i22++) {
    HI_inlierStatus_data[i22] = true;
  }

  b_emxInit_real_T(&x_apo_prev, 1);

  //  high innovation inliers
  i22 = x_apo_prev->size[0];
  x_apo_prev->size[0] = b_P->size[0];
  emxEnsureCapacity((emxArray__common *)x_apo_prev, i22, (int)sizeof(double));
  ib = b_P->size[0];
  for (i22 = 0; i22 < ib; i22++) {
    x_apo_prev->data[i22] = 0.0;
  }

  it = 0;
  b_emxInit_real_T(&x_apo, 1);
  emxInit_real_T(&C, 2);
  b_emxInit_int32_T(&r10, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b_y, 2);
  emxInit_real_T(&b, 2);
  b_emxInit_real_T(&b_x_apo, 1);
  b_emxInit_real_T(&c_xt, 1);
  exitg1 = false;
  while ((!exitg1) && (it <= (int)b_VIOParameters.max_ekf_iterations - 1)) {
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
    for (i22 = 0; i22 < nx; i22++) {
      b_indMeas_data[i22] = indMeas_data[ii_data[i22] - 1];
    }

    getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
               indMeas_size, map, anchorInd, featureAnchorInd, b_m_vect,
               noiseParameters_image_noise, noiseParameters_pressure_noise,
               noiseParameters_ext_pos_noise, noiseParameters_ext_att_noise,
               c_noiseParameters_gravity_align, IMU_measurements,
               b_height_offset_pressure, b_VIOParameters, r_data, r_size, H,
               indMeas_z_data, indMeas_z_size, R_data, R_size);
    if (1.0 + (double)it == 1.0) {
      //  only do outlier rejection in first iteration
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i22 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
        ib = H->size[0];
        for (i22 = 0; i22 < ib; i22++) {
          idx = b_P->size[1];
          for (i23 = 0; i23 < idx; i23++) {
            y->data[i22 + y->size[0] * i23] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i22 + y->size[0] * i23] += H->data[i22 + H->size[0] * ar] *
                b_P->data[ar + b_P->size[0] * i23];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i22 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
        i22 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i22 = 0; i22 < ib; i22++) {
          y->data[i22] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i22 = idx + m;
            for (ic = idx; ic + 1 <= i22; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          nx = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i22 = nx + k;
            for (ib = nx; ib + 1 <= i22; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i23 = idx + m;
                for (ic = idx; ic + 1 <= i23; ic++) {
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

      i22 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i22, (int)sizeof(double));
      ib = H->size[0];
      for (i22 = 0; i22 < ib; i22++) {
        idx = H->size[1];
        for (i23 = 0; i23 < idx; i23++) {
          b->data[i23 + b->size[0] * i22] = H->data[i22 + H->size[0] * i23];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i22 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = y->size[0];
        b_y->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_y, i22, (int)sizeof(double));
        ib = y->size[0];
        for (i22 = 0; i22 < ib; i22++) {
          idx = b->size[1];
          for (i23 = 0; i23 < idx; i23++) {
            b_y->data[i22 + b_y->size[0] * i23] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              b_y->data[i22 + b_y->size[0] * i23] += y->data[i22 + y->size[0] *
                ar] * b->data[ar + b->size[0] * i23];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i22 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)b_y, i22, (int)sizeof(double));
        i22 = b_y->size[0] * b_y->size[1];
        b_y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)b_y, i22, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i22 = 0; i22 < ib; i22++) {
          b_y->data[i22] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i22 = idx + m;
            for (ic = idx; ic + 1 <= i22; ic++) {
              b_y->data[ic] = 0.0;
            }

            idx += m;
          }

          nx = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i22 = nx + k;
            for (ib = nx; ib + 1 <= i22; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i23 = idx + m;
                for (ic = idx; ic + 1 <= i23; ic++) {
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
      for (i22 = 0; i22 < ib; i22++) {
        S_data[i22] = b_y->data[i22] + R_data[i22];
      }

      for (k = 0; k < loop_ub; k++) {
        ar = k << 1;
        ii = k << 1;
        idx = k << 1;
        for (i22 = 0; i22 < 2; i22++) {
          r[i22] = r_data[i22 + ar];
          for (i23 = 0; i23 < 2; i23++) {
            S[i23 + (i22 << 1)] = S_data[(i23 + ii) + nx * (i22 + idx)];
          }
        }

        b_mrdivide(r, S, a);
        ar = k << 1;

        //  otherwise check if HI inlier
        d_numStatesxt = 0.0;
        for (i22 = 0; i22 < 2; i22++) {
          d_numStatesxt += a[i22] * r_data[i22 + ar];
        }

        if (d_numStatesxt > 6.0) {
          ar = k << 1;
          for (i22 = 0; i22 < 2; i22++) {
            r_data[i22 + ar] = 0.0;
          }

          ib = H->size[1];
          ar = k << 1;
          for (i22 = 0; i22 < ib; i22++) {
            for (i23 = 0; i23 < 2; i23++) {
              H->data[(i23 + ar) + H->size[0] * i22] = 0.0;
            }
          }

          HI_inlierStatus_data[k] = false;
        }
      }
    }

    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i22 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
      ib = H->size[0];
      for (i22 = 0; i22 < ib; i22++) {
        idx = b_P->size[1];
        for (i23 = 0; i23 < idx; i23++) {
          y->data[i22 + y->size[0] * i23] = 0.0;
          ii = H->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i22 + y->size[0] * i23] += H->data[i22 + H->size[0] * ar] *
              b_P->data[ar + b_P->size[0] * i23];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      m = H->size[0];
      i22 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
      i22 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i22 = 0; i22 < ib; i22++) {
        y->data[i22] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i22 = idx + m;
          for (ic = idx; ic + 1 <= i22; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        nx = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i22 = nx + k;
          for (ib = nx; ib + 1 <= i22; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i23 = idx + m;
              for (ic = idx; ic + 1 <= i23; ic++) {
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

    i22 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i22, (int)sizeof(double));
    ib = H->size[0];
    for (i22 = 0; i22 < ib; i22++) {
      idx = H->size[1];
      for (i23 = 0; i23 < idx; i23++) {
        b->data[i23 + b->size[0] * i22] = H->data[i22 + H->size[0] * i23];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i22 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i22, (int)sizeof(double));
      ib = y->size[0];
      for (i22 = 0; i22 < ib; i22++) {
        idx = b->size[1];
        for (i23 = 0; i23 < idx; i23++) {
          C->data[i22 + C->size[0] * i23] = 0.0;
          ii = y->size[1];
          for (ar = 0; ar < ii; ar++) {
            C->data[i22 + C->size[0] * i23] += y->data[i22 + y->size[0] * ar] *
              b->data[ar + b->size[0] * i23];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i22 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i22, (int)sizeof(double));
      i22 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i22, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i22 = 0; i22 < ib; i22++) {
        C->data[i22] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i22 = idx + m;
          for (ic = idx; ic + 1 <= i22; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        nx = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i22 = nx + k;
          for (ib = nx; ib + 1 <= i22; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i23 = idx + m;
              for (ic = idx; ic + 1 <= i23; ic++) {
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

    i22 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i22, (int)sizeof(double));
    ib = H->size[0];
    for (i22 = 0; i22 < ib; i22++) {
      idx = H->size[1];
      for (i23 = 0; i23 < idx; i23++) {
        b->data[i23 + b->size[0] * i22] = H->data[i22 + H->size[0] * i23];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i22 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
      ib = b_P->size[0];
      for (i22 = 0; i22 < ib; i22++) {
        idx = b->size[1];
        for (i23 = 0; i23 < idx; i23++) {
          y->data[i22 + y->size[0] * i23] = 0.0;
          ii = b_P->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i22 + y->size[0] * i23] += b_P->data[i22 + b_P->size[0] * ar]
              * b->data[ar + b->size[0] * i23];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i22 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
      i22 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i22 = 0; i22 < ib; i22++) {
        y->data[i22] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i22 = idx + m;
          for (ic = idx; ic + 1 <= i22; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        nx = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i22 = nx + k;
          for (ib = nx; ib + 1 <= i22; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i23 = idx + m;
              for (ic = idx; ic + 1 <= i23; ic++) {
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
    for (i22 = 0; i22 < ib; i22++) {
      C_data[i22] = C->data[i22] + R_data[i22];
    }

    mrdivide(y, C_data, C_size, K);
    if ((K->size[1] == 1) || (r_size[0] == 1)) {
      i22 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i22, (int)sizeof(double));
      ib = K->size[0];
      for (i22 = 0; i22 < ib; i22++) {
        x_apo->data[i22] = 0.0;
        idx = K->size[1];
        for (i23 = 0; i23 < idx; i23++) {
          x_apo->data[i22] += K->data[i22 + K->size[0] * i23] * r_data[i23];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      m = K->size[0];
      i22 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i22, (int)sizeof(double));
      ib = (int)a[0];
      for (i22 = 0; i22 < ib; i22++) {
        x_apo->data[i22] = 0.0;
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
          i22 = nx + k;
          for (ib = nx; ib + 1 <= i22; ib++) {
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

    for (i22 = 0; i22 < 3; i22++) {
      d_xt[i22] = b_xt->data[i22] + x_apo->data[i22];
    }

    for (i22 = 0; i22 < 3; i22++) {
      b_xt->data[i22] = d_xt[i22];
    }

    for (i22 = 0; i22 < 3; i22++) {
      d_xt[i22] = x_apo->data[3 + i22];
    }

    quatPlusThetaJ(d_xt, dv11);
    quatmultJ(dv11, *(double (*)[4])&b_xt->data[3], S);
    for (i22 = 0; i22 < 4; i22++) {
      b_xt->data[3 + i22] = S[i22];
    }

    if (8.0 > c_numStatesxt) {
      i22 = 0;
      i23 = 0;
    } else {
      i22 = 7;
      i23 = (int)c_numStatesxt;
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

    ii = r10->size[0] * r10->size[1];
    r10->size[0] = 1;
    r10->size[1] = nx - idx;
    emxEnsureCapacity((emxArray__common *)r10, ii, (int)sizeof(int));
    ib = nx - idx;
    for (nx = 0; nx < ib; nx++) {
      r10->data[r10->size[0] * nx] = idx + nx;
    }

    idx = c_xt->size[0];
    c_xt->size[0] = i23 - i22;
    emxEnsureCapacity((emxArray__common *)c_xt, idx, (int)sizeof(double));
    ib = i23 - i22;
    for (i23 = 0; i23 < ib; i23++) {
      c_xt->data[i23] = b_xt->data[i22 + i23] + x_apo->data[ar + i23];
    }

    ib = r10->size[1];
    for (i22 = 0; i22 < ib; i22++) {
      b_xt->data[r10->data[r10->size[0] * i22]] = c_xt->data[(*(int (*)[2])
        r10->size)[0] * i22];
    }

    for (idx = 0; idx < b_anchorFeatures->size[1]; idx++) {
      for (i22 = 0; i22 < 24; i22++) {
        x[i22] = (b_anchorFeatures->data[i22 + b_anchorFeatures->size[0] * idx] ==
                  1.0);
      }

      if (any(x)) {
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0 +
          numPointsPerAnchor);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)idx) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i22 = 0; i22 < 3; i22++) {
          d_xt[i22] = b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i22)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i22)) - 1];
        }

        for (i22 = 0; i22 < 3; i22++) {
          b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i22)) - 1] = d_xt[i22];
        }

        d_numStates = c_numStates + ((1.0 + (double)idx) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i22 = 0; i22 < 3; i22++) {
          d_xt[i22] = x_apo->data[(int)(d_numStates + (4.0 + (double)i22)) - 1];
        }

        d_numStatesxt = c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i22 = 0; i22 < 4; i22++) {
          e_xt[i22] = b_xt->data[(int)(d_numStatesxt + (4.0 + (double)i22)) - 1];
        }

        quatPlusThetaJ(d_xt, dv12);
        quatmultJ(dv12, e_xt, S);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i22 = 0; i22 < 4; i22++) {
          b_xt->data[(int)(d_numStatesxt + (4.0 + (double)i22)) - 1] = S[i22];
        }

        for (ii = 0; ii < (int)numPointsPerAnchor; ii++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)idx) - 1.0) * (7.0
            + numPointsPerAnchor)) + 7.0) + (1.0 + (double)ii)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)idx) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ii)) - 1];
        }
      }
    }

    i22 = b_x_apo->size[0];
    b_x_apo->size[0] = x_apo->size[0];
    emxEnsureCapacity((emxArray__common *)b_x_apo, i22, (int)sizeof(double));
    ib = x_apo->size[0];
    for (i22 = 0; i22 < ib; i22++) {
      b_x_apo->data[i22] = x_apo->data[i22] - x_apo_prev->data[i22];
    }

    if (d_norm(b_x_apo) < 0.001) {
      exitg1 = true;
    } else {
      i22 = x_apo_prev->size[0];
      x_apo_prev->size[0] = x_apo->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo_prev, i22, (int)sizeof(double));
      ib = x_apo->size[0];
      for (i22 = 0; i22 < ib; i22++) {
        x_apo_prev->data[i22] = x_apo->data[i22];
      }

      it++;
    }
  }

  emxFree_real_T(&c_xt);
  emxFree_real_T(&b_x_apo);
  emxFree_real_T(&y);
  emxFree_int32_T(&r10);
  emxFree_real_T(&featureAnchorInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&map);
  emxFree_real_T(&x_apo_prev);
  emxFree_real_T(&x_apo);
  b_eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
         b_VIOParameters.num_points_per_anchor), b_y);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i22 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i22, (int)sizeof(double));
    ib = K->size[0];
    for (i22 = 0; i22 < ib; i22++) {
      idx = H->size[1];
      for (i23 = 0; i23 < idx; i23++) {
        C->data[i22 + C->size[0] * i23] = 0.0;
        ii = K->size[1];
        for (ar = 0; ar < ii; ar++) {
          C->data[i22 + C->size[0] * i23] += K->data[i22 + K->size[0] * ar] *
            H->data[ar + H->size[0] * i23];
        }
      }
    }
  } else {
    k = K->size[1];
    a[0] = (unsigned int)K->size[0];
    a[1] = (unsigned int)H->size[1];
    m = K->size[0];
    i22 = C->size[0] * C->size[1];
    C->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)C, i22, (int)sizeof(double));
    i22 = C->size[0] * C->size[1];
    C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)C, i22, (int)sizeof(double));
    ib = (int)a[0] * (int)a[1];
    for (i22 = 0; i22 < ib; i22++) {
      C->data[i22] = 0.0;
    }

    if ((K->size[0] == 0) || (H->size[1] == 0)) {
    } else {
      ii = K->size[0] * (H->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i22 = idx + m;
        for (ic = idx; ic + 1 <= i22; ic++) {
          C->data[ic] = 0.0;
        }

        idx += m;
      }

      nx = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i22 = nx + k;
        for (ib = nx; ib + 1 <= i22; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            i23 = idx + m;
            for (ic = idx; ic + 1 <= i23; ic++) {
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
  i22 = b_y->size[0] * b_y->size[1];
  emxEnsureCapacity((emxArray__common *)b_y, i22, (int)sizeof(double));
  ii = b_y->size[0];
  idx = b_y->size[1];
  ib = ii * idx;
  for (i22 = 0; i22 < ib; i22++) {
    b_y->data[i22] -= C->data[i22];
  }

  emxFree_real_T(&C);
  i22 = b->size[0] * b->size[1];
  b->size[0] = b_P->size[0];
  b->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b, i22, (int)sizeof(double));
  ib = b_P->size[0] * b_P->size[1];
  for (i22 = 0; i22 < ib; i22++) {
    b->data[i22] = b_P->data[i22];
  }

  emxInit_real_T(&c_y, 2);
  if ((b_y->size[1] == 1) || (b_P->size[0] == 1)) {
    i22 = c_y->size[0] * c_y->size[1];
    c_y->size[0] = b_y->size[0];
    c_y->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)c_y, i22, (int)sizeof(double));
    ib = b_y->size[0];
    for (i22 = 0; i22 < ib; i22++) {
      idx = b_P->size[1];
      for (i23 = 0; i23 < idx; i23++) {
        c_y->data[i22 + c_y->size[0] * i23] = 0.0;
        ii = b_y->size[1];
        for (ar = 0; ar < ii; ar++) {
          c_y->data[i22 + c_y->size[0] * i23] += b_y->data[i22 + b_y->size[0] *
            ar] * b_P->data[ar + b_P->size[0] * i23];
        }
      }
    }

    i22 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = c_y->size[0];
    b_P->size[1] = c_y->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i22, (int)sizeof(double));
    ib = c_y->size[1];
    for (i22 = 0; i22 < ib; i22++) {
      idx = c_y->size[0];
      for (i23 = 0; i23 < idx; i23++) {
        b_P->data[i23 + b_P->size[0] * i22] = c_y->data[i23 + c_y->size[0] * i22];
      }
    }
  } else {
    k = b_y->size[1];
    a[0] = b_y->size[0];
    a[1] = b_P->size[1];
    m = b_y->size[0];
    i22 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = (int)a[0];
    b_P->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)b_P, i22, (int)sizeof(double));
    ib = (int)a[1];
    for (i22 = 0; i22 < ib; i22++) {
      idx = (int)a[0];
      for (i23 = 0; i23 < idx; i23++) {
        b_P->data[i23 + b_P->size[0] * i22] = 0.0;
      }
    }

    if ((b_y->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = b_y->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i22 = idx + m;
        for (ic = idx; ic + 1 <= i22; ic++) {
          b_P->data[ic] = 0.0;
        }

        idx += m;
      }

      nx = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        ar = 0;
        i22 = nx + k;
        for (ib = nx; ib + 1 <= i22; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i23 = idx + m;
            for (ic = idx; ic + 1 <= i23; ic++) {
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
