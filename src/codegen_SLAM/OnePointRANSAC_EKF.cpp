//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: OnePointRANSAC_EKF.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "OnePointRANSAC_EKF.h"
#include "norm.h"
#include "predictMeasurementMono.h"
#include "quatmultJ.h"
#include "quatPlusThetaJ.h"
#include "SLAM_emxutil.h"
#include "mrdivide.h"
#include "getH_R_res.h"
#include "ros_info.h"
#include "getScaledMap.h"
#include "eye.h"
#include "any.h"
#include "SLAM_data.h"
#include <ros/console.h>

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : f_struct_T *b_xt
//                emxArray_real_T *b_P
//                const double z_u_l[80]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                const double noiseParameters_image_noise[2]
//                double c_VIOParameters_max_ekf_iterati
//                int updateVect[40]
// Return Type  : void
//
void OnePointRANSAC_EKF(f_struct_T *b_xt, emxArray_real_T *b_P, const double
  z_u_l[80], const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double noiseParameters_image_noise[2],
  double c_VIOParameters_max_ekf_iterati, int updateVect[40])
{
  double numStatesPerAnchor;
  boolean_T activeFeatures[40];
  boolean_T delayedFeatures[40];
  int br;
  int ii;
  int idx;
  int i36;
  signed char ii_data[40];
  boolean_T exitg2;
  boolean_T guard2 = false;
  int loop_ub;
  signed char hyp_ind_data[40];
  boolean_T LI_inlier_status[40];
  double num_hyp;
  int hyp_it;
  emxArray_real_T *K;
  emxArray_real_T *x_apo;
  emxArray_real_T *r;
  emxArray_real_T *H;
  emxArray_real_T *R;
  emxArray_real_T *C;
  emxArray_real_T *y;
  emxArray_real_T *b;
  emxArray_real_T *b_C;
  boolean_T LI_inlier_status_i[40];
  boolean_T hyp_status[40];
  int ar;
  int i37;
  int k;
  double a[2];
  int b_m;
  int ic;
  int ib;
  int ia;
  double b_x_apo[3];
  double dv14[4];
  double c_xt[4];
  double R_cw[9];
  double r_wc[3];
  double d_xt;
  double anchorPos[3];
  double dv15[4];
  double anchorRot[9];
  double rho;
  double z_u[2];
  double b_rho[3];
  double b_r[2];
  double dv16[2];
  emxArray_real_T *c_C;
  double dv17[4];
  double dv18[4];
  double dv19[4];
  e_struct_T e_xt;
  double dv20[4];
  emxArray_real_T *b_R;
  boolean_T b_activeFeatures;
  boolean_T exitg1;
  boolean_T guard1 = false;
  int ii_size_idx_0;
  int it;
  emxArray_real_T *d_C;
  e_struct_T f_xt;
  double dv21[4];
  double dv22[4];
  double dv23[4];
  double dv24[4];
  emxArray_real_T *c_R;
  emxArray_real_T *e_C;
  emxArray_real_T *d_R;
  numStatesPerAnchor = 6.0 + numPointsPerAnchor;

  //  HI mahalanobis gate
  //  build the map according to the current estimate
  getScaledMap(b_xt);
  for (br = 0; br < 40; br++) {
    activeFeatures[br] = false;
    delayedFeatures[br] = false;
  }

  for (ii = 0; ii < (int)numAnchors; ii++) {
    for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
      if (b_xt->anchor_states->data[ii].feature_states->data[idx].status == 1.0)
      {
        activeFeatures[(int)b_xt->anchor_states->data[ii].feature_states->
          data[idx].status_idx - 1] = true;
      } else {
        if (b_xt->anchor_states->data[ii].feature_states->data[idx].status ==
            2.0) {
          delayedFeatures[(int)b_xt->anchor_states->data[ii]
            .feature_states->data[idx].status_idx - 1] = true;
        }
      }
    }
  }

  for (i36 = 0; i36 < 40; i36++) {
    activeFeatures[i36] = (activeFeatures[i36] && (updateVect[i36] == 1));
    delayedFeatures[i36] = (delayedFeatures[i36] && (updateVect[i36] == 1));
  }

  // % B 1-point hypotheses generation and evaluation
  //  randomly permute the active feature indices for 1-point RANSAC
  idx = 0;
  ii = 1;
  exitg2 = false;
  while ((!exitg2) && (ii < 41)) {
    guard2 = false;
    if (activeFeatures[ii - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)ii;
      if (idx >= 40) {
        exitg2 = true;
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

  for (i36 = 0; i36 < loop_ub; i36++) {
    hyp_ind_data[i36] = ii_data[i36];
  }

  //  hyp_ind = hyp_ind(randperm(length(hyp_ind)));
  for (br = 0; br < 40; br++) {
    LI_inlier_status[br] = false;
  }

  if (1 > idx) {
    num_hyp = 0.0;
  } else {
    num_hyp = idx;
  }

  hyp_it = 1;
  emxInit_real_T(&K, 2);
  b_emxInit_real_T(&x_apo, 1);
  b_emxInit_real_T(&r, 1);
  emxInit_real_T(&H, 2);
  emxInit_real_T(&R, 2);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b, 2);
  emxInit_real_T(&b_C, 2);
  while ((hyp_it < num_hyp) && (hyp_it < loop_ub)) {
    for (br = 0; br < 40; br++) {
      LI_inlier_status_i[br] = false;
      hyp_status[br] = false;
    }

    hyp_status[hyp_ind_data[hyp_it - 1] - 1] = true;

    //  used to signal which feature to compute the derivatives for
    getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att, b_xt->anchor_states,
               z_u_l, hyp_status, cameraparams_FocalLength,
               cameraparams_PrincipalPoint, noiseParameters_image_noise, r, H, R);
    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i36 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      ar = H->size[0];
      for (i36 = 0; i36 < ar; i36++) {
        idx = b_P->size[1];
        for (i37 = 0; i37 < idx; i37++) {
          y->data[i36 + y->size[0] * i37] = 0.0;
          ii = H->size[1];
          for (br = 0; br < ii; br++) {
            y->data[i36 + y->size[0] * i37] += H->data[i36 + H->size[0] * br] *
              b_P->data[br + b_P->size[0] * i37];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      b_m = H->size[0];
      i36 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      i36 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      ar = (int)a[0] * (int)a[1];
      for (i36 = 0; i36 < ar; i36++) {
        y->data[i36] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                y->data[ic] += b_P->data[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    i36 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
    ar = H->size[0];
    for (i36 = 0; i36 < ar; i36++) {
      idx = H->size[1];
      for (i37 = 0; i37 < idx; i37++) {
        b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i36 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      ar = y->size[0];
      for (i36 = 0; i36 < ar; i36++) {
        idx = b->size[1];
        for (i37 = 0; i37 < idx; i37++) {
          C->data[i36 + C->size[0] * i37] = 0.0;
          ii = y->size[1];
          for (br = 0; br < ii; br++) {
            C->data[i36 + C->size[0] * i37] += y->data[i36 + y->size[0] * br] *
              b->data[br + b->size[0] * i37];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (unsigned int)y->size[0];
      a[1] = (unsigned int)b->size[1];
      b_m = y->size[0];
      i36 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      i36 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      ar = (int)a[0] * (int)a[1];
      for (i36 = 0; i36 < ar; i36++) {
        C->data[i36] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                C->data[ic] += b->data[ib] * y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    i36 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
    ar = H->size[0];
    for (i36 = 0; i36 < ar; i36++) {
      idx = H->size[1];
      for (i37 = 0; i37 < idx; i37++) {
        b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i36 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      ar = b_P->size[0];
      for (i36 = 0; i36 < ar; i36++) {
        idx = b->size[1];
        for (i37 = 0; i37 < idx; i37++) {
          y->data[i36 + y->size[0] * i37] = 0.0;
          ii = b_P->size[1];
          for (br = 0; br < ii; br++) {
            y->data[i36 + y->size[0] * i37] += b_P->data[i36 + b_P->size[0] * br]
              * b->data[br + b->size[0] * i37];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      b_m = b_P->size[0];
      i36 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      i36 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      ar = (int)a[0] * (int)a[1];
      for (i36 = 0; i36 < ar; i36++) {
        y->data[i36] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                y->data[ic] += b->data[ib] * b_P->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    i36 = b_C->size[0] * b_C->size[1];
    b_C->size[0] = C->size[0];
    b_C->size[1] = C->size[1];
    emxEnsureCapacity((emxArray__common *)b_C, i36, (int)sizeof(double));
    ar = C->size[0] * C->size[1];
    for (i36 = 0; i36 < ar; i36++) {
      b_C->data[i36] = C->data[i36] + R->data[i36];
    }

    mrdivide(y, b_C, K);
    if ((K->size[1] == 1) || (r->size[0] == 1)) {
      i36 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i36, (int)sizeof(double));
      ar = K->size[0];
      for (i36 = 0; i36 < ar; i36++) {
        x_apo->data[i36] = 0.0;
        idx = K->size[1];
        for (i37 = 0; i37 < idx; i37++) {
          x_apo->data[i36] += K->data[i36 + K->size[0] * i37] * r->data[i37];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      b_m = K->size[0];
      i36 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i36, (int)sizeof(double));
      ar = (int)a[0];
      for (i36 = 0; i36 < ar; i36++) {
        x_apo->data[i36] = 0.0;
      }

      if (K->size[0] == 0) {
      } else {
        idx = 0;
        while ((b_m > 0) && (idx <= 0)) {
          for (ic = 1; ic <= b_m; ic++) {
            x_apo->data[ic - 1] = 0.0;
          }

          idx = b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= 0)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (r->data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= b_m; ic++) {
                ia++;
                x_apo->data[ic] += r->data[ib] * K->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx = b_m;
        }
      }
    }

    for (i36 = 0; i36 < 3; i36++) {
      b_x_apo[i36] = x_apo->data[3 + i36];
    }

    quatPlusThetaJ(b_x_apo, dv14);
    quatmultJ(dv14, b_xt->robot_state.att, c_xt);

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    R_cw[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2]) +
      c_xt[3] * c_xt[3];
    R_cw[3] = 2.0 * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
    R_cw[6] = 2.0 * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
    R_cw[1] = 2.0 * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
    R_cw[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2]) +
      c_xt[3] * c_xt[3];
    R_cw[7] = 2.0 * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
    R_cw[2] = 2.0 * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
    R_cw[5] = 2.0 * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
    R_cw[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2] * c_xt[2]) +
      c_xt[3] * c_xt[3];
    for (i36 = 0; i36 < 3; i36++) {
      r_wc[i36] = b_xt->robot_state.pos[i36] + x_apo->data[i36];
    }

    for (ii = 0; ii < (int)numAnchors; ii++) {
      d_xt = numStates + ((1.0 + (double)ii) - 1.0) * numStatesPerAnchor;
      for (i36 = 0; i36 < 3; i36++) {
        anchorPos[i36] = b_xt->anchor_states->data[ii].pos[i36] + x_apo->data
          [(int)(d_xt + (1.0 + (double)i36)) - 1];
      }

      d_xt = numStates + ((1.0 + (double)ii) - 1.0) * numStatesPerAnchor;
      for (i36 = 0; i36 < 3; i36++) {
        b_x_apo[i36] = x_apo->data[(int)(d_xt + (4.0 + (double)i36)) - 1];
      }

      quatPlusThetaJ(b_x_apo, dv15);
      quatmultJ(dv15, b_xt->anchor_states->data[ii].att, c_xt);

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      anchorRot[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];
      anchorRot[3] = 2.0 * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
      anchorRot[6] = 2.0 * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
      anchorRot[1] = 2.0 * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
      anchorRot[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2] *
                      c_xt[2]) + c_xt[3] * c_xt[3];
      anchorRot[7] = 2.0 * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
      anchorRot[2] = 2.0 * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
      anchorRot[5] = 2.0 * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
      anchorRot[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2] *
                      c_xt[2]) + c_xt[3] * c_xt[3];
      for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
        if (b_xt->anchor_states->data[ii].feature_states->data[idx].status ==
            1.0) {
          //  only update active features
          rho = b_xt->anchor_states->data[ii].feature_states->data[idx].
            inverse_depth + x_apo->data[(int)(((numStates + ((1.0 + (double)ii)
            - 1.0) * numStatesPerAnchor) + 6.0) + (1.0 + (double)idx)) - 1];
          d_xt = (b_xt->anchor_states->data[ii].feature_states->data[idx].
                  status_idx - 1.0) * 2.0;
          for (i36 = 0; i36 < 2; i36++) {
            z_u[i36] = z_u_l[(int)(d_xt + (1.0 + (double)i36)) - 1];
          }

          for (i36 = 0; i36 < 3; i36++) {
            d_xt = 0.0;
            for (i37 = 0; i37 < 3; i37++) {
              d_xt += anchorRot[i37 + 3 * i36] * b_xt->anchor_states->data[ii].
                feature_states->data[idx].m[i37];
            }

            b_rho[i36] = (rho * anchorPos[i36] + d_xt) - r_wc[i36] * rho;
          }

          for (i36 = 0; i36 < 3; i36++) {
            b_x_apo[i36] = 0.0;
            for (i37 = 0; i37 < 3; i37++) {
              b_x_apo[i36] += R_cw[i36 + 3 * i37] * b_rho[i37];
            }
          }

          predictMeasurementMono(b_x_apo, cameraparams_FocalLength,
            cameraparams_PrincipalPoint, b_r);
          for (br = 0; br < 2; br++) {
            dv16[br] = b_r[br] - z_u[br];
          }

          if (c_norm(dv16) < 3.0) {
            LI_inlier_status_i[(int)b_xt->anchor_states->data[ii].
              feature_states->data[idx].status_idx - 1] = true;
          }
        }
      }
    }

    ii = 0;
    idx = 0;
    for (k = 0; k < 40; k++) {
      if (LI_inlier_status_i[k]) {
        ii++;
      }

      if (LI_inlier_status[k]) {
        idx++;
      }
    }

    if (ii > idx) {
      ii = 0;
      idx = 0;
      for (br = 0; br < 40; br++) {
        LI_inlier_status[br] = LI_inlier_status_i[br];
        if (LI_inlier_status_i[br]) {
          ii++;
        }

        if (activeFeatures[br]) {
          idx++;
        }
      }

      num_hyp = -4.60517018598809 / log(1.0 - (double)ii / (double)idx);
    }

    hyp_it++;
  }

  emxFree_real_T(&b_C);

  //  ros_info('Found %i LI inliers in %i active features', nnz(LI_inlier_status), nnz(activeFeatures)) 
  ii = 0;
  for (k = 0; k < 40; k++) {
    if (LI_inlier_status[k]) {
      ii++;
    }
  }

  if (ii > 3) {
    getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att, b_xt->anchor_states,
               z_u_l, LI_inlier_status, cameraparams_FocalLength,
               cameraparams_PrincipalPoint, noiseParameters_image_noise, r, H, R);
    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i36 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = b_P->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          y->data[i36 + y->size[0] * i37] = 0.0;
          idx = H->size[1];
          for (br = 0; br < idx; br++) {
            y->data[i36 + y->size[0] * i37] += H->data[i36 + H->size[0] * br] *
              b_P->data[br + b_P->size[0] * i37];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      b_m = H->size[0];
      i36 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      i36 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      loop_ub = (int)a[0] * (int)a[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        y->data[i36] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                y->data[ic] += b_P->data[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    i36 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
    loop_ub = H->size[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = H->size[1];
      for (i37 = 0; i37 < ar; i37++) {
        b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i36 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      loop_ub = y->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = b->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          C->data[i36 + C->size[0] * i37] = 0.0;
          idx = y->size[1];
          for (br = 0; br < idx; br++) {
            C->data[i36 + C->size[0] * i37] += y->data[i36 + y->size[0] * br] *
              b->data[br + b->size[0] * i37];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (unsigned int)y->size[0];
      a[1] = (unsigned int)b->size[1];
      b_m = y->size[0];
      i36 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      i36 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      loop_ub = (int)a[0] * (int)a[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        C->data[i36] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                C->data[ic] += b->data[ib] * y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    i36 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
    loop_ub = H->size[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = H->size[1];
      for (i37 = 0; i37 < ar; i37++) {
        b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i36 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      loop_ub = b_P->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = b->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          y->data[i36 + y->size[0] * i37] = 0.0;
          idx = b_P->size[1];
          for (br = 0; br < idx; br++) {
            y->data[i36 + y->size[0] * i37] += b_P->data[i36 + b_P->size[0] * br]
              * b->data[br + b->size[0] * i37];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      b_m = b_P->size[0];
      i36 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      i36 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
      loop_ub = (int)a[0] * (int)a[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        y->data[i36] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                y->data[ic] += b->data[ib] * b_P->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    emxInit_real_T(&c_C, 2);
    i36 = c_C->size[0] * c_C->size[1];
    c_C->size[0] = C->size[0];
    c_C->size[1] = C->size[1];
    emxEnsureCapacity((emxArray__common *)c_C, i36, (int)sizeof(double));
    loop_ub = C->size[0] * C->size[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      c_C->data[i36] = C->data[i36] + R->data[i36];
    }

    mrdivide(y, c_C, K);
    emxFree_real_T(&c_C);
    if ((K->size[1] == 1) || (r->size[0] == 1)) {
      i36 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i36, (int)sizeof(double));
      loop_ub = K->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        x_apo->data[i36] = 0.0;
        ar = K->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          x_apo->data[i36] += K->data[i36 + K->size[0] * i37] * r->data[i37];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      b_m = K->size[0];
      i36 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i36, (int)sizeof(double));
      loop_ub = (int)a[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        x_apo->data[i36] = 0.0;
      }

      if (K->size[0] == 0) {
      } else {
        idx = 0;
        while ((b_m > 0) && (idx <= 0)) {
          for (ic = 1; ic <= b_m; ic++) {
            x_apo->data[ic - 1] = 0.0;
          }

          idx = b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= 0)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (r->data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= b_m; ic++) {
                ia++;
                x_apo->data[ic] += r->data[ib] * K->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx = b_m;
        }
      }
    }

    for (i36 = 0; i36 < 3; i36++) {
      b_xt->robot_state.pos[i36] += x_apo->data[i36];
    }

    for (i36 = 0; i36 < 3; i36++) {
      b_x_apo[i36] = x_apo->data[3 + i36];
    }

    for (br = 0; br < 4; br++) {
      c_xt[br] = b_xt->robot_state.att[br];
    }

    quatPlusThetaJ(b_x_apo, dv17);
    quatmultJ(dv17, c_xt, b_xt->robot_state.att);
    for (i36 = 0; i36 < 3; i36++) {
      b_xt->robot_state.vel[i36] += x_apo->data[6 + i36];
    }

    for (i36 = 0; i36 < 3; i36++) {
      b_xt->robot_state.IMU.gyro_bias[i36] += x_apo->data[9 + i36];
    }

    for (i36 = 0; i36 < 3; i36++) {
      b_xt->robot_state.IMU.acc_bias[i36] += x_apo->data[12 + i36];
    }

    for (i36 = 0; i36 < 3; i36++) {
      b_x_apo[i36] = x_apo->data[15 + i36];
    }

    for (br = 0; br < 4; br++) {
      c_xt[br] = b_xt->origin.att[br];
    }

    quatPlusThetaJ(b_x_apo, dv18);
    quatmultJ(dv18, c_xt, b_xt->origin.att);
    for (i36 = 0; i36 < 3; i36++) {
      b_x_apo[i36] = x_apo->data[18 + i36];
    }

    for (br = 0; br < 4; br++) {
      c_xt[br] = b_xt->robot_state.IMU.att[br];
    }

    quatPlusThetaJ(b_x_apo, dv19);
    quatmultJ(dv19, c_xt, b_xt->robot_state.IMU.att);
    for (i36 = 0; i36 < 3; i36++) {
      b_xt->robot_state.IMU.pos[i36] += x_apo->data[21 + i36];
    }

    ii = 0;
    b_emxInitStruct_struct_T(&e_xt);
    while (ii <= (int)numAnchors - 1) {
      emxCopyStruct_struct_T(&e_xt, &b_xt->anchor_states->data[ii]);
      d_xt = numStates + ((1.0 + (double)ii) - 1.0) * numStatesPerAnchor;
      for (i36 = 0; i36 < 3; i36++) {
        b_xt->anchor_states->data[ii].pos[i36] = e_xt.pos[i36] + x_apo->data
          [(int)(d_xt + (1.0 + (double)i36)) - 1];
      }

      d_xt = numStates + ((1.0 + (double)ii) - 1.0) * numStatesPerAnchor;
      for (i36 = 0; i36 < 3; i36++) {
        b_x_apo[i36] = x_apo->data[(int)(d_xt + (4.0 + (double)i36)) - 1];
      }

      for (i36 = 0; i36 < 4; i36++) {
        c_xt[i36] = b_xt->anchor_states->data[ii].att[i36];
      }

      quatPlusThetaJ(b_x_apo, dv20);
      quatmultJ(dv20, c_xt, b_xt->anchor_states->data[ii].att);
      for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
        if (b_xt->anchor_states->data[ii].feature_states->data[idx].status ==
            1.0) {
          //  only update active features
          b_xt->anchor_states->data[ii].feature_states->data[idx].inverse_depth +=
            x_apo->data[(int)(((numStates + ((1.0 + (double)ii) - 1.0) *
                                numStatesPerAnchor) + 6.0) + (1.0 + (double)idx))
            - 1];
          if (b_xt->anchor_states->data[ii].feature_states->data[idx].
              inverse_depth < 0.0) {
            updateVect[(int)b_xt->anchor_states->data[ii].feature_states->
              data[idx].status_idx - 1] = 0;
            b_xt->anchor_states->data[ii].feature_states->data[idx].status = 0.0;
            b_xt->anchor_states->data[ii].feature_states->data[idx].status_idx =
              0.0;
          }
        }
      }

      ii++;
    }

    emxFreeStruct_struct_T(&e_xt);
    b_eye(numStates + numAnchors * numStatesPerAnchor, R);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i36 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      loop_ub = K->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = H->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          C->data[i36 + C->size[0] * i37] = 0.0;
          idx = K->size[1];
          for (br = 0; br < idx; br++) {
            C->data[i36 + C->size[0] * i37] += K->data[i36 + K->size[0] * br] *
              H->data[br + H->size[0] * i37];
          }
        }
      }
    } else {
      k = K->size[1];
      a[0] = (unsigned int)K->size[0];
      a[1] = (unsigned int)H->size[1];
      b_m = K->size[0];
      i36 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      i36 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      loop_ub = (int)a[0] * (int)a[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        C->data[i36] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                C->data[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    i36 = R->size[0] * R->size[1];
    emxEnsureCapacity((emxArray__common *)R, i36, (int)sizeof(double));
    ii = R->size[0];
    idx = R->size[1];
    loop_ub = ii * idx;
    for (i36 = 0; i36 < loop_ub; i36++) {
      R->data[i36] -= C->data[i36];
    }

    i36 = b->size[0] * b->size[1];
    b->size[0] = b_P->size[0];
    b->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
    loop_ub = b_P->size[0] * b_P->size[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      b->data[i36] = b_P->data[i36];
    }

    emxInit_real_T(&b_R, 2);
    if ((R->size[1] == 1) || (b_P->size[0] == 1)) {
      i36 = b_R->size[0] * b_R->size[1];
      b_R->size[0] = R->size[0];
      b_R->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)b_R, i36, (int)sizeof(double));
      loop_ub = R->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = b_P->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          b_R->data[i36 + b_R->size[0] * i37] = 0.0;
          idx = R->size[1];
          for (br = 0; br < idx; br++) {
            b_R->data[i36 + b_R->size[0] * i37] += R->data[i36 + R->size[0] * br]
              * b_P->data[br + b_P->size[0] * i37];
          }
        }
      }

      i36 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = b_R->size[0];
      b_P->size[1] = b_R->size[1];
      emxEnsureCapacity((emxArray__common *)b_P, i36, (int)sizeof(double));
      loop_ub = b_R->size[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = b_R->size[0];
        for (i37 = 0; i37 < ar; i37++) {
          b_P->data[i37 + b_P->size[0] * i36] = b_R->data[i37 + b_R->size[0] *
            i36];
        }
      }
    } else {
      k = R->size[1];
      a[0] = R->size[0];
      a[1] = b_P->size[1];
      b_m = R->size[0];
      i36 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = (int)a[0];
      b_P->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)b_P, i36, (int)sizeof(double));
      loop_ub = (int)a[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = (int)a[0];
        for (i37 = 0; i37 < ar; i37++) {
          b_P->data[i37 + b_P->size[0] * i36] = 0.0;
        }
      }

      if ((R->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = R->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            b_P->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                b_P->data[ic] += b->data[ib] * R->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    emxFree_real_T(&b_R);
  } else {
    for (br = 0; br < 40; br++) {
      LI_inlier_status[br] = false;
    }
  }

  // % D Partial EKF update using high-innovation inliers
  for (i36 = 0; i36 < 40; i36++) {
    b_activeFeatures = (activeFeatures[i36] && (!LI_inlier_status[i36]));

    //  high innovation inliers (ordered like updateVect)
    activeFeatures[i36] = b_activeFeatures;
    LI_inlier_status[i36] = b_activeFeatures;
  }

  idx = 0;
  ii = 1;
  exitg1 = false;
  while ((!exitg1) && (ii < 41)) {
    guard1 = false;
    if (activeFeatures[ii - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)ii;
      if (idx >= 40) {
        exitg1 = true;
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
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  for (i36 = 0; i36 < loop_ub; i36++) {
    hyp_ind_data[i36] = ii_data[i36];
  }

  if (any(activeFeatures)) {
    i36 = K->size[0] * K->size[1];
    K->size[0] = 1;
    K->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)K, i36, (int)sizeof(double));
    K->data[0] = 0.0;

    //  for coder
    i36 = H->size[0] * H->size[1];
    H->size[0] = 1;
    H->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)H, i36, (int)sizeof(double));
    H->data[0] = 0.0;

    //  for coder
    it = 0;
    emxInit_real_T(&d_C, 2);
    b_emxInitStruct_struct_T(&f_xt);
    while (it <= (int)c_VIOParameters_max_ekf_iterati - 1) {
      getScaledMap(b_xt);

      //  build the map according to the current estimate
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, LI_inlier_status,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, R);

      //  the residual is ordered by anchors/features, not like updateVect
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i36 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        loop_ub = H->size[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          ar = b_P->size[1];
          for (i37 = 0; i37 < ar; i37++) {
            y->data[i36 + y->size[0] * i37] = 0.0;
            idx = H->size[1];
            for (br = 0; br < idx; br++) {
              y->data[i36 + y->size[0] * i37] += H->data[i36 + H->size[0] * br] *
                b_P->data[br + b_P->size[0] * i37];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        b_m = H->size[0];
        i36 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        i36 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        loop_ub = (int)a[0] * (int)a[1];
        for (i36 = 0; i36 < loop_ub; i36++) {
          y->data[i36] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            i36 = idx + b_m;
            for (ic = idx; ic + 1 <= i36; ic++) {
              y->data[ic] = 0.0;
            }

            idx += b_m;
          }

          br = 0;
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            ar = 0;
            i36 = br + k;
            for (ib = br; ib + 1 <= i36; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i37 = idx + b_m;
                for (ic = idx; ic + 1 <= i37; ic++) {
                  ia++;
                  y->data[ic] += b_P->data[ib] * H->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            idx += b_m;
          }
        }
      }

      i36 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = H->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i36 = K->size[0] * K->size[1];
        K->size[0] = y->size[0];
        K->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)K, i36, (int)sizeof(double));
        loop_ub = y->size[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          ar = b->size[1];
          for (i37 = 0; i37 < ar; i37++) {
            K->data[i36 + K->size[0] * i37] = 0.0;
            idx = y->size[1];
            for (br = 0; br < idx; br++) {
              K->data[i36 + K->size[0] * i37] += y->data[i36 + y->size[0] * br] *
                b->data[br + b->size[0] * i37];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (unsigned int)y->size[0];
        a[1] = (unsigned int)b->size[1];
        b_m = y->size[0];
        i36 = K->size[0] * K->size[1];
        K->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)K, i36, (int)sizeof(double));
        i36 = K->size[0] * K->size[1];
        K->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)K, i36, (int)sizeof(double));
        loop_ub = (int)a[0] * (int)a[1];
        for (i36 = 0; i36 < loop_ub; i36++) {
          K->data[i36] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            i36 = idx + b_m;
            for (ic = idx; ic + 1 <= i36; ic++) {
              K->data[ic] = 0.0;
            }

            idx += b_m;
          }

          br = 0;
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            ar = 0;
            i36 = br + k;
            for (ib = br; ib + 1 <= i36; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i37 = idx + b_m;
                for (ic = idx; ic + 1 <= i37; ic++) {
                  ia++;
                  K->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            idx += b_m;
          }
        }
      }

      i36 = K->size[0] * K->size[1];
      emxEnsureCapacity((emxArray__common *)K, i36, (int)sizeof(double));
      ii = K->size[0];
      idx = K->size[1];
      loop_ub = ii * idx;
      for (i36 = 0; i36 < loop_ub; i36++) {
        K->data[i36] += R->data[i36];
      }

      for (k = 0; k < ii_size_idx_0; k++) {
        idx = k << 1;
        for (i36 = 0; i36 < 2; i36++) {
          b_r[i36] = r->data[i36 + idx];
        }

        idx = k << 1;
        ii = k << 1;
        for (i36 = 0; i36 < 2; i36++) {
          for (i37 = 0; i37 < 2; i37++) {
            c_xt[i37 + (i36 << 1)] = K->data[(i37 + idx) + K->size[0] * (i36 +
              ii)];
          }
        }

        b_mrdivide(b_r, c_xt, a);
        idx = k << 1;
        for (i36 = 0; i36 < 2; i36++) {
          z_u[i36] = r->data[i36 + idx];
        }

        d_xt = 0.0;
        for (i36 = 0; i36 < 2; i36++) {
          d_xt += a[i36] * z_u[i36];
        }

        if (d_xt > 6.0) {
          idx = k << 1;
          for (i36 = 0; i36 < 2; i36++) {
            r->data[i36 + idx] = 0.0;
          }

          loop_ub = H->size[1];
          idx = k << 1;
          for (i36 = 0; i36 < loop_ub; i36++) {
            for (i37 = 0; i37 < 2; i37++) {
              H->data[(i37 + idx) + H->size[0] * i36] = 0.0;
            }
          }

          if (1.0 + (double)it == c_VIOParameters_max_ekf_iterati) {
            LI_inlier_status[hyp_ind_data[k] - 1] = false;

            //  only reject the feature if its still bad in last iteration, otherwise just dont use for this update 
          }

          //          ros_info('rejecting %i', HI_ind(k))
        }
      }

      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i36 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        loop_ub = H->size[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          ar = b_P->size[1];
          for (i37 = 0; i37 < ar; i37++) {
            y->data[i36 + y->size[0] * i37] = 0.0;
            idx = H->size[1];
            for (br = 0; br < idx; br++) {
              y->data[i36 + y->size[0] * i37] += H->data[i36 + H->size[0] * br] *
                b_P->data[br + b_P->size[0] * i37];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        b_m = H->size[0];
        i36 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        i36 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        loop_ub = (int)a[0] * (int)a[1];
        for (i36 = 0; i36 < loop_ub; i36++) {
          y->data[i36] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            i36 = idx + b_m;
            for (ic = idx; ic + 1 <= i36; ic++) {
              y->data[ic] = 0.0;
            }

            idx += b_m;
          }

          br = 0;
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            ar = 0;
            i36 = br + k;
            for (ib = br; ib + 1 <= i36; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i37 = idx + b_m;
                for (ic = idx; ic + 1 <= i37; ic++) {
                  ia++;
                  y->data[ic] += b_P->data[ib] * H->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            idx += b_m;
          }
        }
      }

      i36 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = H->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i36 = C->size[0] * C->size[1];
        C->size[0] = y->size[0];
        C->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
        loop_ub = y->size[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          ar = b->size[1];
          for (i37 = 0; i37 < ar; i37++) {
            C->data[i36 + C->size[0] * i37] = 0.0;
            idx = y->size[1];
            for (br = 0; br < idx; br++) {
              C->data[i36 + C->size[0] * i37] += y->data[i36 + y->size[0] * br] *
                b->data[br + b->size[0] * i37];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (unsigned int)y->size[0];
        a[1] = (unsigned int)b->size[1];
        b_m = y->size[0];
        i36 = C->size[0] * C->size[1];
        C->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
        i36 = C->size[0] * C->size[1];
        C->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
        loop_ub = (int)a[0] * (int)a[1];
        for (i36 = 0; i36 < loop_ub; i36++) {
          C->data[i36] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            i36 = idx + b_m;
            for (ic = idx; ic + 1 <= i36; ic++) {
              C->data[ic] = 0.0;
            }

            idx += b_m;
          }

          br = 0;
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            ar = 0;
            i36 = br + k;
            for (ib = br; ib + 1 <= i36; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i37 = idx + b_m;
                for (ic = idx; ic + 1 <= i37; ic++) {
                  ia++;
                  C->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            idx += b_m;
          }
        }
      }

      i36 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = H->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
        }
      }

      if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
        i36 = y->size[0] * y->size[1];
        y->size[0] = b_P->size[0];
        y->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        loop_ub = b_P->size[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          ar = b->size[1];
          for (i37 = 0; i37 < ar; i37++) {
            y->data[i36 + y->size[0] * i37] = 0.0;
            idx = b_P->size[1];
            for (br = 0; br < idx; br++) {
              y->data[i36 + y->size[0] * i37] += b_P->data[i36 + b_P->size[0] *
                br] * b->data[br + b->size[0] * i37];
            }
          }
        }
      } else {
        k = b_P->size[1];
        a[0] = b_P->size[0];
        a[1] = b->size[1];
        b_m = b_P->size[0];
        i36 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        i36 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
        loop_ub = (int)a[0] * (int)a[1];
        for (i36 = 0; i36 < loop_ub; i36++) {
          y->data[i36] = 0.0;
        }

        if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = b_P->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            i36 = idx + b_m;
            for (ic = idx; ic + 1 <= i36; ic++) {
              y->data[ic] = 0.0;
            }

            idx += b_m;
          }

          br = 0;
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            ar = 0;
            i36 = br + k;
            for (ib = br; ib + 1 <= i36; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i37 = idx + b_m;
                for (ic = idx; ic + 1 <= i37; ic++) {
                  ia++;
                  y->data[ic] += b->data[ib] * b_P->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            idx += b_m;
          }
        }
      }

      i36 = d_C->size[0] * d_C->size[1];
      d_C->size[0] = C->size[0];
      d_C->size[1] = C->size[1];
      emxEnsureCapacity((emxArray__common *)d_C, i36, (int)sizeof(double));
      loop_ub = C->size[0] * C->size[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        d_C->data[i36] = C->data[i36] + R->data[i36];
      }

      mrdivide(y, d_C, K);
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        i36 = x_apo->size[0];
        x_apo->size[0] = K->size[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i36, (int)sizeof(double));
        loop_ub = K->size[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          x_apo->data[i36] = 0.0;
          ar = K->size[1];
          for (i37 = 0; i37 < ar; i37++) {
            x_apo->data[i36] += K->data[i36 + K->size[0] * i37] * r->data[i37];
          }
        }
      } else {
        k = K->size[1];
        a[0] = K->size[0];
        b_m = K->size[0];
        i36 = x_apo->size[0];
        x_apo->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i36, (int)sizeof(double));
        loop_ub = (int)a[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          x_apo->data[i36] = 0.0;
        }

        if (K->size[0] == 0) {
        } else {
          idx = 0;
          while ((b_m > 0) && (idx <= 0)) {
            for (ic = 1; ic <= b_m; ic++) {
              x_apo->data[ic - 1] = 0.0;
            }

            idx = b_m;
          }

          br = 0;
          idx = 0;
          while ((b_m > 0) && (idx <= 0)) {
            ar = 0;
            i36 = br + k;
            for (ib = br; ib + 1 <= i36; ib++) {
              if (r->data[ib] != 0.0) {
                ia = ar;
                for (ic = 0; ic + 1 <= b_m; ic++) {
                  ia++;
                  x_apo->data[ic] += r->data[ib] * K->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            idx = b_m;
          }
        }
      }

      for (i36 = 0; i36 < 3; i36++) {
        b_xt->robot_state.pos[i36] += x_apo->data[i36];
      }

      for (i36 = 0; i36 < 3; i36++) {
        b_x_apo[i36] = x_apo->data[3 + i36];
      }

      for (br = 0; br < 4; br++) {
        c_xt[br] = b_xt->robot_state.att[br];
      }

      quatPlusThetaJ(b_x_apo, dv21);
      quatmultJ(dv21, c_xt, b_xt->robot_state.att);
      for (i36 = 0; i36 < 3; i36++) {
        b_xt->robot_state.vel[i36] += x_apo->data[6 + i36];
      }

      for (i36 = 0; i36 < 3; i36++) {
        b_xt->robot_state.IMU.gyro_bias[i36] += x_apo->data[9 + i36];
      }

      for (i36 = 0; i36 < 3; i36++) {
        b_xt->robot_state.IMU.acc_bias[i36] += x_apo->data[12 + i36];
      }

      for (i36 = 0; i36 < 3; i36++) {
        b_x_apo[i36] = x_apo->data[15 + i36];
      }

      for (br = 0; br < 4; br++) {
        c_xt[br] = b_xt->origin.att[br];
      }

      quatPlusThetaJ(b_x_apo, dv22);
      quatmultJ(dv22, c_xt, b_xt->origin.att);
      for (i36 = 0; i36 < 3; i36++) {
        b_x_apo[i36] = x_apo->data[18 + i36];
      }

      for (br = 0; br < 4; br++) {
        c_xt[br] = b_xt->robot_state.IMU.att[br];
      }

      quatPlusThetaJ(b_x_apo, dv23);
      quatmultJ(dv23, c_xt, b_xt->robot_state.IMU.att);
      for (i36 = 0; i36 < 3; i36++) {
        b_xt->robot_state.IMU.pos[i36] += x_apo->data[21 + i36];
      }

      i36 = (int)numAnchors;
      for (ii = 0; ii < i36; ii++) {
        emxCopyStruct_struct_T(&f_xt, &b_xt->anchor_states->data[ii]);
        d_xt = numStates + ((1.0 + (double)ii) - 1.0) * numStatesPerAnchor;
        for (i37 = 0; i37 < 3; i37++) {
          b_xt->anchor_states->data[ii].pos[i37] = f_xt.pos[i37] + x_apo->data
            [(int)(d_xt + (1.0 + (double)i37)) - 1];
        }

        d_xt = numStates + ((1.0 + (double)ii) - 1.0) * numStatesPerAnchor;
        for (i37 = 0; i37 < 3; i37++) {
          b_x_apo[i37] = x_apo->data[(int)(d_xt + (4.0 + (double)i37)) - 1];
        }

        for (i37 = 0; i37 < 4; i37++) {
          c_xt[i37] = b_xt->anchor_states->data[ii].att[i37];
        }

        quatPlusThetaJ(b_x_apo, dv24);
        quatmultJ(dv24, c_xt, b_xt->anchor_states->data[ii].att);
        i37 = (int)numPointsPerAnchor;
        for (idx = 0; idx < i37; idx++) {
          if (b_xt->anchor_states->data[ii].feature_states->data[idx].status ==
              1.0) {
            //  only update active features
            if (LI_inlier_status[(int)b_xt->anchor_states->data[ii].
                feature_states->data[idx].status_idx - 1]) {
              b_xt->anchor_states->data[ii].feature_states->data[idx].
                inverse_depth += x_apo->data[(int)(((numStates + ((1.0 + (double)
                ii) - 1.0) * numStatesPerAnchor) + 6.0) + (1.0 + (double)idx)) -
                1];
              if ((b_xt->anchor_states->data[ii].feature_states->data[idx].
                   inverse_depth < 0.0) && (1.0 + (double)it ==
                   c_VIOParameters_max_ekf_iterati)) {
                //  only reject if we are done iterating
                updateVect[(int)b_xt->anchor_states->data[ii]
                  .feature_states->data[idx].status_idx - 1] = 0;
                b_xt->anchor_states->data[ii].feature_states->data[idx].status =
                  0.0;
                b_xt->anchor_states->data[ii].feature_states->data[idx].
                  status_idx = 0.0;
              }
            } else {
              if (activeFeatures[(int)b_xt->anchor_states->data[ii].
                  feature_states->data[idx].status_idx - 1] && (1.0 + (double)it
                   == c_VIOParameters_max_ekf_iterati)) {
                //  if it is not a HI inlier, but was a candidate, it was rejected by mahalanobis 
                //  only reject if we are done iterating
                d_ros_info(b_xt->anchor_states->data[ii].feature_states->
                           data[idx].status_idx, 1.0 + (double)idx, 1.0 +
                           (double)ii);
                updateVect[(int)b_xt->anchor_states->data[ii]
                  .feature_states->data[idx].status_idx - 1] = 0;
                b_xt->anchor_states->data[ii].feature_states->data[idx].status =
                  0.0;
                b_xt->anchor_states->data[ii].feature_states->data[idx].
                  status_idx = 0.0;
              }
            }
          }
        }
      }

      it++;
    }

    emxFreeStruct_struct_T(&f_xt);
    emxFree_real_T(&d_C);
    b_eye(numStates + numAnchors * numStatesPerAnchor, R);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i36 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      loop_ub = K->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = H->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          C->data[i36 + C->size[0] * i37] = 0.0;
          idx = K->size[1];
          for (br = 0; br < idx; br++) {
            C->data[i36 + C->size[0] * i37] += K->data[i36 + K->size[0] * br] *
              H->data[br + H->size[0] * i37];
          }
        }
      }
    } else {
      k = K->size[1];
      a[0] = (unsigned int)K->size[0];
      a[1] = (unsigned int)H->size[1];
      b_m = K->size[0];
      i36 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      i36 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
      loop_ub = (int)a[0] * (int)a[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        C->data[i36] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                C->data[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    i36 = R->size[0] * R->size[1];
    emxEnsureCapacity((emxArray__common *)R, i36, (int)sizeof(double));
    ii = R->size[0];
    idx = R->size[1];
    loop_ub = ii * idx;
    for (i36 = 0; i36 < loop_ub; i36++) {
      R->data[i36] -= C->data[i36];
    }

    i36 = b->size[0] * b->size[1];
    b->size[0] = b_P->size[0];
    b->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
    loop_ub = b_P->size[0] * b_P->size[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      b->data[i36] = b_P->data[i36];
    }

    emxInit_real_T(&c_R, 2);
    if ((R->size[1] == 1) || (b_P->size[0] == 1)) {
      i36 = c_R->size[0] * c_R->size[1];
      c_R->size[0] = R->size[0];
      c_R->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)c_R, i36, (int)sizeof(double));
      loop_ub = R->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = b_P->size[1];
        for (i37 = 0; i37 < ar; i37++) {
          c_R->data[i36 + c_R->size[0] * i37] = 0.0;
          idx = R->size[1];
          for (br = 0; br < idx; br++) {
            c_R->data[i36 + c_R->size[0] * i37] += R->data[i36 + R->size[0] * br]
              * b_P->data[br + b_P->size[0] * i37];
          }
        }
      }

      i36 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = c_R->size[0];
      b_P->size[1] = c_R->size[1];
      emxEnsureCapacity((emxArray__common *)b_P, i36, (int)sizeof(double));
      loop_ub = c_R->size[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = c_R->size[0];
        for (i37 = 0; i37 < ar; i37++) {
          b_P->data[i37 + b_P->size[0] * i36] = c_R->data[i37 + c_R->size[0] *
            i36];
        }
      }
    } else {
      k = R->size[1];
      a[0] = R->size[0];
      a[1] = b_P->size[1];
      b_m = R->size[0];
      i36 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = (int)a[0];
      b_P->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)b_P, i36, (int)sizeof(double));
      loop_ub = (int)a[1];
      for (i36 = 0; i36 < loop_ub; i36++) {
        ar = (int)a[0];
        for (i37 = 0; i37 < ar; i37++) {
          b_P->data[i37 + b_P->size[0] * i36] = 0.0;
        }
      }

      if ((R->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = R->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i36 = idx + b_m;
          for (ic = idx; ic + 1 <= i36; ic++) {
            b_P->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i36 = br + k;
          for (ib = br; ib + 1 <= i36; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i37 = idx + b_m;
              for (ic = idx; ic + 1 <= i37; ic++) {
                ia++;
                b_P->data[ic] += b->data[ib] * R->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    emxFree_real_T(&c_R);
  }

  // % Update the delayed initialization features
  getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att, b_xt->anchor_states,
             z_u_l, delayedFeatures, cameraparams_FocalLength,
             cameraparams_PrincipalPoint, noiseParameters_image_noise, r, H, R);
  if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
    i36 = y->size[0] * y->size[1];
    y->size[0] = H->size[0];
    y->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
    loop_ub = H->size[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = b_P->size[1];
      for (i37 = 0; i37 < ar; i37++) {
        y->data[i36 + y->size[0] * i37] = 0.0;
        idx = H->size[1];
        for (br = 0; br < idx; br++) {
          y->data[i36 + y->size[0] * i37] += H->data[i36 + H->size[0] * br] *
            b_P->data[br + b_P->size[0] * i37];
        }
      }
    }
  } else {
    k = H->size[1];
    a[0] = H->size[0];
    a[1] = b_P->size[1];
    b_m = H->size[0];
    i36 = y->size[0] * y->size[1];
    y->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
    i36 = y->size[0] * y->size[1];
    y->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
    loop_ub = (int)a[0] * (int)a[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      y->data[i36] = 0.0;
    }

    if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
    } else {
      ii = H->size[0] * (b_P->size[1] - 1);
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i36 = idx + b_m;
        for (ic = idx; ic + 1 <= i36; ic++) {
          y->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        i36 = br + k;
        for (ib = br; ib + 1 <= i36; ib++) {
          if (b_P->data[ib] != 0.0) {
            ia = ar;
            i37 = idx + b_m;
            for (ic = idx; ic + 1 <= i37; ic++) {
              ia++;
              y->data[ic] += b_P->data[ib] * H->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
        idx += b_m;
      }
    }
  }

  i36 = b->size[0] * b->size[1];
  b->size[0] = H->size[1];
  b->size[1] = H->size[0];
  emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
  loop_ub = H->size[0];
  for (i36 = 0; i36 < loop_ub; i36++) {
    ar = H->size[1];
    for (i37 = 0; i37 < ar; i37++) {
      b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
    }
  }

  if ((y->size[1] == 1) || (b->size[0] == 1)) {
    i36 = C->size[0] * C->size[1];
    C->size[0] = y->size[0];
    C->size[1] = b->size[1];
    emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
    loop_ub = y->size[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = b->size[1];
      for (i37 = 0; i37 < ar; i37++) {
        C->data[i36 + C->size[0] * i37] = 0.0;
        idx = y->size[1];
        for (br = 0; br < idx; br++) {
          C->data[i36 + C->size[0] * i37] += y->data[i36 + y->size[0] * br] *
            b->data[br + b->size[0] * i37];
        }
      }
    }
  } else {
    k = y->size[1];
    a[0] = (unsigned int)y->size[0];
    a[1] = (unsigned int)b->size[1];
    b_m = y->size[0];
    i36 = C->size[0] * C->size[1];
    C->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
    i36 = C->size[0] * C->size[1];
    C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
    loop_ub = (int)a[0] * (int)a[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      C->data[i36] = 0.0;
    }

    if ((y->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = y->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i36 = idx + b_m;
        for (ic = idx; ic + 1 <= i36; ic++) {
          C->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        i36 = br + k;
        for (ib = br; ib + 1 <= i36; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i37 = idx + b_m;
            for (ic = idx; ic + 1 <= i37; ic++) {
              ia++;
              C->data[ic] += b->data[ib] * y->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
        idx += b_m;
      }
    }
  }

  i36 = b->size[0] * b->size[1];
  b->size[0] = H->size[1];
  b->size[1] = H->size[0];
  emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
  loop_ub = H->size[0];
  for (i36 = 0; i36 < loop_ub; i36++) {
    ar = H->size[1];
    for (i37 = 0; i37 < ar; i37++) {
      b->data[i37 + b->size[0] * i36] = H->data[i36 + H->size[0] * i37];
    }
  }

  if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
    i36 = y->size[0] * y->size[1];
    y->size[0] = b_P->size[0];
    y->size[1] = b->size[1];
    emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
    loop_ub = b_P->size[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = b->size[1];
      for (i37 = 0; i37 < ar; i37++) {
        y->data[i36 + y->size[0] * i37] = 0.0;
        idx = b_P->size[1];
        for (br = 0; br < idx; br++) {
          y->data[i36 + y->size[0] * i37] += b_P->data[i36 + b_P->size[0] * br] *
            b->data[br + b->size[0] * i37];
        }
      }
    }
  } else {
    k = b_P->size[1];
    a[0] = b_P->size[0];
    a[1] = b->size[1];
    b_m = b_P->size[0];
    i36 = y->size[0] * y->size[1];
    y->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
    i36 = y->size[0] * y->size[1];
    y->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)y, i36, (int)sizeof(double));
    loop_ub = (int)a[0] * (int)a[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      y->data[i36] = 0.0;
    }

    if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = b_P->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i36 = idx + b_m;
        for (ic = idx; ic + 1 <= i36; ic++) {
          y->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        i36 = br + k;
        for (ib = br; ib + 1 <= i36; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i37 = idx + b_m;
            for (ic = idx; ic + 1 <= i37; ic++) {
              ia++;
              y->data[ic] += b->data[ib] * b_P->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
        idx += b_m;
      }
    }
  }

  emxInit_real_T(&e_C, 2);
  i36 = e_C->size[0] * e_C->size[1];
  e_C->size[0] = C->size[0];
  e_C->size[1] = C->size[1];
  emxEnsureCapacity((emxArray__common *)e_C, i36, (int)sizeof(double));
  loop_ub = C->size[0] * C->size[1];
  for (i36 = 0; i36 < loop_ub; i36++) {
    e_C->data[i36] = C->data[i36] + R->data[i36];
  }

  mrdivide(y, e_C, K);
  emxFree_real_T(&e_C);
  emxFree_real_T(&y);
  if ((K->size[1] == 1) || (r->size[0] == 1)) {
    i36 = x_apo->size[0];
    x_apo->size[0] = K->size[0];
    emxEnsureCapacity((emxArray__common *)x_apo, i36, (int)sizeof(double));
    loop_ub = K->size[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      x_apo->data[i36] = 0.0;
      ar = K->size[1];
      for (i37 = 0; i37 < ar; i37++) {
        x_apo->data[i36] += K->data[i36 + K->size[0] * i37] * r->data[i37];
      }
    }
  } else {
    k = K->size[1];
    a[0] = K->size[0];
    b_m = K->size[0];
    i36 = x_apo->size[0];
    x_apo->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)x_apo, i36, (int)sizeof(double));
    loop_ub = (int)a[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      x_apo->data[i36] = 0.0;
    }

    if (K->size[0] == 0) {
    } else {
      idx = 0;
      while ((b_m > 0) && (idx <= 0)) {
        for (ic = 1; ic <= b_m; ic++) {
          x_apo->data[ic - 1] = 0.0;
        }

        idx = b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= 0)) {
        ar = 0;
        i36 = br + k;
        for (ib = br; ib + 1 <= i36; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= b_m; ic++) {
              ia++;
              x_apo->data[ic] += r->data[ib] * K->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
        idx = b_m;
      }
    }
  }

  emxFree_real_T(&r);
  for (ii = 0; ii < (int)numAnchors; ii++) {
    for (idx = 0; idx < (int)numPointsPerAnchor; idx++) {
      if (b_xt->anchor_states->data[ii].feature_states->data[idx].status == 2.0)
      {
        b_xt->anchor_states->data[ii].feature_states->data[idx].inverse_depth +=
          x_apo->data[(int)(((numStates + ((1.0 + (double)ii) - 1.0) *
                              numStatesPerAnchor) + 6.0) + (1.0 + (double)idx))
          - 1];
      }
    }
  }

  emxFree_real_T(&x_apo);
  b_eye(numStates + numAnchors * numStatesPerAnchor, R);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i36 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
    loop_ub = K->size[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = H->size[1];
      for (i37 = 0; i37 < ar; i37++) {
        C->data[i36 + C->size[0] * i37] = 0.0;
        idx = K->size[1];
        for (br = 0; br < idx; br++) {
          C->data[i36 + C->size[0] * i37] += K->data[i36 + K->size[0] * br] *
            H->data[br + H->size[0] * i37];
        }
      }
    }
  } else {
    k = K->size[1];
    a[0] = (unsigned int)K->size[0];
    a[1] = (unsigned int)H->size[1];
    b_m = K->size[0];
    i36 = C->size[0] * C->size[1];
    C->size[0] = (int)a[0];
    emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
    i36 = C->size[0] * C->size[1];
    C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)C, i36, (int)sizeof(double));
    loop_ub = (int)a[0] * (int)a[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      C->data[i36] = 0.0;
    }

    if ((K->size[0] == 0) || (H->size[1] == 0)) {
    } else {
      ii = K->size[0] * (H->size[1] - 1);
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i36 = idx + b_m;
        for (ic = idx; ic + 1 <= i36; ic++) {
          C->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        i36 = br + k;
        for (ib = br; ib + 1 <= i36; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            i37 = idx + b_m;
            for (ic = idx; ic + 1 <= i37; ic++) {
              ia++;
              C->data[ic] += H->data[ib] * K->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
        idx += b_m;
      }
    }
  }

  emxFree_real_T(&H);
  emxFree_real_T(&K);
  i36 = R->size[0] * R->size[1];
  emxEnsureCapacity((emxArray__common *)R, i36, (int)sizeof(double));
  ii = R->size[0];
  idx = R->size[1];
  loop_ub = ii * idx;
  for (i36 = 0; i36 < loop_ub; i36++) {
    R->data[i36] -= C->data[i36];
  }

  emxFree_real_T(&C);
  i36 = b->size[0] * b->size[1];
  b->size[0] = b_P->size[0];
  b->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)b, i36, (int)sizeof(double));
  loop_ub = b_P->size[0] * b_P->size[1];
  for (i36 = 0; i36 < loop_ub; i36++) {
    b->data[i36] = b_P->data[i36];
  }

  emxInit_real_T(&d_R, 2);
  if ((R->size[1] == 1) || (b_P->size[0] == 1)) {
    i36 = d_R->size[0] * d_R->size[1];
    d_R->size[0] = R->size[0];
    d_R->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)d_R, i36, (int)sizeof(double));
    loop_ub = R->size[0];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = b_P->size[1];
      for (i37 = 0; i37 < ar; i37++) {
        d_R->data[i36 + d_R->size[0] * i37] = 0.0;
        idx = R->size[1];
        for (br = 0; br < idx; br++) {
          d_R->data[i36 + d_R->size[0] * i37] += R->data[i36 + R->size[0] * br] *
            b_P->data[br + b_P->size[0] * i37];
        }
      }
    }

    i36 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = d_R->size[0];
    b_P->size[1] = d_R->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i36, (int)sizeof(double));
    loop_ub = d_R->size[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = d_R->size[0];
      for (i37 = 0; i37 < ar; i37++) {
        b_P->data[i37 + b_P->size[0] * i36] = d_R->data[i37 + d_R->size[0] * i36];
      }
    }
  } else {
    k = R->size[1];
    a[0] = R->size[0];
    a[1] = b_P->size[1];
    b_m = R->size[0];
    i36 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = (int)a[0];
    b_P->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)b_P, i36, (int)sizeof(double));
    loop_ub = (int)a[1];
    for (i36 = 0; i36 < loop_ub; i36++) {
      ar = (int)a[0];
      for (i37 = 0; i37 < ar; i37++) {
        b_P->data[i37 + b_P->size[0] * i36] = 0.0;
      }
    }

    if ((R->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = R->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i36 = idx + b_m;
        for (ic = idx; ic + 1 <= i36; ic++) {
          b_P->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        i36 = br + k;
        for (ib = br; ib + 1 <= i36; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i37 = idx + b_m;
            for (ic = idx; ic + 1 <= i37; ic++) {
              ia++;
              b_P->data[ic] += b->data[ib] * R->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
        idx += b_m;
      }
    }
  }

  emxFree_real_T(&d_R);
  emxFree_real_T(&b);
  emxFree_real_T(&R);

  // %
}

//
// File trailer for OnePointRANSAC_EKF.cpp
//
// [EOF]
//
