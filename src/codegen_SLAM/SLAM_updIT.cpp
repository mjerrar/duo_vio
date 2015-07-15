//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_updIT.h"
#include "mrdivide.h"
#include "quatmultJ.h"
#include "quatPlusThetaJ.h"
#include "any.h"
#include "initializePoint.h"
#include "SLAM_emxutil.h"
#include "eye.h"
#include "fprintf.h"
#include "det.h"
#include "getJacobianAndResidual_std.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static double map[96];
static double pointInMap[32];

// Function Definitions

//
// Arguments    : emxArray_real_T *P_apr
//                emxArray_real_T *b_xt
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                double updateVect[32]
//                const double z_all_l[64]
//                const double z_all_r[64]
//                const double imNoise[4]
//                emxArray_real_T *h_u_apo
//                double map_out[96]
// Return Type  : void
//
void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const double
                cameraparams_r_lr[3], const double cameraparams_R_lr[9], double
                updateVect[32], const double z_all_l[64], const double z_all_r
                [64], const double imNoise[4], emxArray_real_T *h_u_apo, double
                map_out[96])
{
  int i13;
  int ar;
  int idx;
  signed char ii_data[32];
  int kidx;
  boolean_T exitg1;
  boolean_T guard1 = false;
  int ii_size_idx_0;
  int indMeas_size[1];
  double indMeas_data[32];
  emxArray_real_T *H_xc;
  emxArray_real_T *r2;
  int h_u_size[1];
  double h_u_data[128];
  int r_size[1];
  double r_data[128];
  emxArray_real_T *H;
  int i14;
  emxArray_boolean_T *r3;
  emxArray_boolean_T *r4;
  emxArray_boolean_T *r5;
  double y[4];
  int k;
  double d[16];
  emxArray_real_T *b_y;
  int ia;
  int ic;
  double A_data[1024];
  emxArray_real_T *R;
  int br;
  int ib;
  emxArray_real_T *c_y;
  int m;
  emxArray_real_T *b;
  emxArray_real_T *S;
  double b_S[16];
  double d3;
  emxArray_real_T *C;
  emxArray_real_T *K;
  emxArray_real_T *x_apo;
  emxArray_real_T *d_y;
  double c_xt[3];
  double dv624[4];
  double d_xt[6];
  double d4;
  double d5;
  double e_xt[4];
  double dv625[4];
  double b_z_all_l[2];
  double b_z_all_r[2];
  double newestTrailPos[3];
  boolean_T bv2[3];

  // % Iterative Camera Pose optimization (EKF)
  i13 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(numTrackFeatures * 4.0);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i13, (int)sizeof(double));
  ar = (int)(numTrackFeatures * 4.0);
  for (i13 = 0; i13 < ar; i13++) {
    h_u_apo->data[i13] = 1.0;
  }

  i13 = h_u_apo->size[0];
  emxEnsureCapacity((emxArray__common *)h_u_apo, i13, (int)sizeof(double));
  ar = h_u_apo->size[0];
  for (i13 = 0; i13 < ar; i13++) {
    h_u_apo->data[i13] *= -100.0;
  }

  //  -100 serves as nan
  // % ================================================================================================= 
  idx = 0;
  kidx = 1;
  exitg1 = false;
  while ((!exitg1) && (kidx < 33)) {
    guard1 = false;
    if ((updateVect[kidx - 1] == 1.0) && (pointInMap[kidx - 1] == 1.0)) {
      idx++;
      ii_data[idx - 1] = (signed char)kidx;
      if (idx >= 32) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      kidx++;
    }
  }

  if (1 > idx) {
    ar = 0;
  } else {
    ar = idx;
  }

  if (1 > idx) {
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  indMeas_size[0] = ar;
  for (i13 = 0; i13 < ar; i13++) {
    indMeas_data[i13] = ii_data[i13];
  }

  if (ii_size_idx_0 > 0) {
    emxInit_real_T(&H_xc, 2);
    emxInit_real_T(&r2, 2);
    getJacobianAndResidual_std(b_xt, (double)P_apr->size[0], z_all_l, z_all_r,
      map, indMeas_data, indMeas_size, cameraparams_r_lr, cameraparams_R_lr,
      r_data, r_size, H_xc, h_u_data, h_u_size);
    kidx = H_xc->size[0];
    i13 = r2->size[0] * r2->size[1];
    r2->size[0] = kidx;
    emxEnsureCapacity((emxArray__common *)r2, i13, (int)sizeof(double));
    i13 = r2->size[0] * r2->size[1];
    r2->size[1] = (int)(trailSize * 6.0);
    emxEnsureCapacity((emxArray__common *)r2, i13, (int)sizeof(double));
    ar = H_xc->size[0] * (int)(trailSize * 6.0);
    for (i13 = 0; i13 < ar; i13++) {
      r2->data[i13] = 0.0;
    }

    emxInit_real_T(&H, 2);
    i13 = H->size[0] * H->size[1];
    H->size[0] = H_xc->size[0];
    H->size[1] = H_xc->size[1] + r2->size[1];
    emxEnsureCapacity((emxArray__common *)H, i13, (int)sizeof(double));
    ar = H_xc->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      kidx = H_xc->size[0];
      for (i14 = 0; i14 < kidx; i14++) {
        H->data[i14 + H->size[0] * i13] = H_xc->data[i14 + H_xc->size[0] * i13];
      }
    }

    ar = r2->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      kidx = r2->size[0];
      for (i14 = 0; i14 < kidx; i14++) {
        H->data[i14 + H->size[0] * (i13 + H_xc->size[1])] = r2->data[i14 +
          r2->size[0] * i13];
      }
    }

    emxFree_real_T(&r2);
    emxFree_real_T(&H_xc);
    emxInit_boolean_T(&r3, 2);
    i13 = r3->size[0] * r3->size[1];
    r3->size[0] = H->size[0];
    r3->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)r3, i13, (int)sizeof(boolean_T));
    ar = H->size[0] * H->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      r3->data[i13] = rtIsNaN(H->data[i13]);
    }

    emxInit_boolean_T(&r4, 2);
    e_any(r3, r4);
    if (f_any(r4)) {
      mb_fprintf();
    }

    b_emxInit_boolean_T(&r5, 1);
    i13 = r5->size[0];
    r5->size[0] = h_u_size[0];
    emxEnsureCapacity((emxArray__common *)r5, i13, (int)sizeof(boolean_T));
    ar = h_u_size[0];
    for (i13 = 0; i13 < ar; i13++) {
      r5->data[i13] = rtIsNaN(h_u_data[i13]);
    }

    if (g_any(r5)) {
      ob_fprintf();
    }

    for (k = 0; k < 4; k++) {
      y[k] = imNoise[k] * imNoise[k];
    }

    memset(&d[0], 0, sizeof(double) << 4);
    for (kidx = 0; kidx < 4; kidx++) {
      d[kidx + (kidx << 2)] = y[kidx];
    }

    emxInit_real_T(&b_y, 2);
    eye((double)ii_size_idx_0, b_y);
    ia = b_y->size[0];
    ic = b_y->size[1];
    ar = b_y->size[0] * b_y->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      A_data[i13] = b_y->data[i13];
    }

    emxInit_real_T(&R, 2);
    i13 = R->size[0] * R->size[1];
    R->size[0] = (unsigned char)(ia << 2);
    R->size[1] = (unsigned char)(ic << 2);
    emxEnsureCapacity((emxArray__common *)R, i13, (int)sizeof(double));
    kidx = -1;
    for (idx = 1; idx <= ic; idx++) {
      for (br = 0; br < 4; br++) {
        for (ar = 1; ar <= ia; ar++) {
          for (ib = 0; ib < 4; ib++) {
            kidx++;
            R->data[kidx] = A_data[(ar + ia * (idx - 1)) - 1] * d[ib + (br << 2)];
          }
        }
      }
    }

    //      if gravityUpdate
    //          r = [r; za/norm(za) - h_rz];
    //          Hg = zeros(3,numStates + trailSize*6);
    //          Hg(:,4:6) = 1*skew(h_rz);
    //          H = [H; Hg];
    //          R = blkdiag(R,1e-4*eye(3));
    //      end
    //      if msckfUPD
    //          r=[r;r_0];
    //          H=[H;zeros(size(H_xc_0,1),12),H_xc_0];
    //          R=blkdiag(R,1*eye(length(r_0)));
    //  %         fprintf('msckf update\n')
    //      end
    i13 = r5->size[0];
    r5->size[0] = r_size[0];
    emxEnsureCapacity((emxArray__common *)r5, i13, (int)sizeof(boolean_T));
    ar = r_size[0];
    for (i13 = 0; i13 < ar; i13++) {
      r5->data[i13] = rtIsNaN(r_data[i13]);
    }

    if (g_any(r5)) {
      qb_fprintf();
    }

    emxInit_real_T(&c_y, 2);
    if ((H->size[1] == 1) || (P_apr->size[0] == 1)) {
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = H->size[0];
      c_y->size[1] = P_apr->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      ar = H->size[0];
      for (i13 = 0; i13 < ar; i13++) {
        kidx = P_apr->size[1];
        for (i14 = 0; i14 < kidx; i14++) {
          c_y->data[i13 + c_y->size[0] * i14] = 0.0;
          idx = H->size[1];
          for (br = 0; br < idx; br++) {
            c_y->data[i13 + c_y->size[0] * i14] += H->data[i13 + H->size[0] * br]
              * P_apr->data[br + P_apr->size[0] * i14];
          }
        }
      }
    } else {
      k = H->size[1];
      kidx = H->size[0];
      idx = P_apr->size[1];
      m = H->size[0];
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = kidx;
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      ar = kidx * idx;
      for (i13 = 0; i13 < ar; i13++) {
        c_y->data[i13] = 0.0;
      }

      if ((H->size[0] == 0) || (P_apr->size[1] == 0)) {
      } else {
        idx = H->size[0] * (P_apr->size[1] - 1);
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          i13 = kidx + m;
          for (ic = kidx; ic + 1 <= i13; ic++) {
            c_y->data[ic] = 0.0;
          }

          kidx += m;
        }

        br = 0;
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          ar = -1;
          i13 = br + k;
          for (ib = br; ib + 1 <= i13; ib++) {
            if (P_apr->data[ib] != 0.0) {
              ia = ar;
              i14 = kidx + m;
              for (ic = kidx; ic + 1 <= i14; ic++) {
                ia++;
                c_y->data[ic] += P_apr->data[ib] * H->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          kidx += m;
        }
      }
    }

    emxInit_real_T(&b, 2);
    i13 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i13, (int)sizeof(double));
    ar = H->size[0];
    for (i13 = 0; i13 < ar; i13++) {
      kidx = H->size[1];
      for (i14 = 0; i14 < kidx; i14++) {
        b->data[i14 + b->size[0] * i13] = H->data[i13 + H->size[0] * i14];
      }
    }

    if ((c_y->size[1] == 1) || (b->size[0] == 1)) {
      i13 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = c_y->size[0];
      b_y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)b_y, i13, (int)sizeof(double));
      ar = c_y->size[0];
      for (i13 = 0; i13 < ar; i13++) {
        kidx = b->size[1];
        for (i14 = 0; i14 < kidx; i14++) {
          b_y->data[i13 + b_y->size[0] * i14] = 0.0;
          idx = c_y->size[1];
          for (br = 0; br < idx; br++) {
            b_y->data[i13 + b_y->size[0] * i14] += c_y->data[i13 + c_y->size[0] *
              br] * b->data[br + b->size[0] * i14];
          }
        }
      }
    } else {
      k = c_y->size[1];
      kidx = c_y->size[0];
      idx = b->size[1];
      m = c_y->size[0];
      i13 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = kidx;
      emxEnsureCapacity((emxArray__common *)b_y, i13, (int)sizeof(double));
      i13 = b_y->size[0] * b_y->size[1];
      b_y->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)b_y, i13, (int)sizeof(double));
      ar = kidx * idx;
      for (i13 = 0; i13 < ar; i13++) {
        b_y->data[i13] = 0.0;
      }

      if ((c_y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        idx = c_y->size[0] * (b->size[1] - 1);
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          i13 = kidx + m;
          for (ic = kidx; ic + 1 <= i13; ic++) {
            b_y->data[ic] = 0.0;
          }

          kidx += m;
        }

        br = 0;
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          ar = -1;
          i13 = br + k;
          for (ib = br; ib + 1 <= i13; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i14 = kidx + m;
              for (ic = kidx; ic + 1 <= i14; ic++) {
                ia++;
                b_y->data[ic] += b->data[ib] * c_y->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          kidx += m;
        }
      }
    }

    emxInit_real_T(&S, 2);
    i13 = S->size[0] * S->size[1];
    S->size[0] = b_y->size[0];
    S->size[1] = b_y->size[1];
    emxEnsureCapacity((emxArray__common *)S, i13, (int)sizeof(double));
    ar = b_y->size[0] * b_y->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      S->data[i13] = b_y->data[i13] + R->data[i13];
    }

    for (idx = 1; idx - 1 < ii_size_idx_0; idx++) {
      br = (idx << 2) - 4;
      for (i13 = 0; i13 < 4; i13++) {
        y[i13] = r_data[i13 + br];
      }

      br = (idx << 2) - 4;
      kidx = (idx << 2) - 4;
      for (i13 = 0; i13 < 4; i13++) {
        for (i14 = 0; i14 < 4; i14++) {
          b_S[i14 + (i13 << 2)] = S->data[(i14 + br) + S->size[0] * (i13 + kidx)];
        }
      }

      b_mrdivide(y, b_S);
      br = (idx << 2) - 4;

      //  mahalanobis distant for left image
      d3 = 0.0;
      for (i13 = 0; i13 < 4; i13++) {
        d3 += y[i13] * r_data[i13 + br];
      }

      if (d3 > 948.77290367811543) {
        //              fprintf('Rejecting feature %i due to mahalanobis (%f), residual: (%f, %f, %f, %f)\n', int32(j), mal, r((i-1)*4 + 1), r((i-1)*4 + 2), r((i-1)*4 + 3), r((i-1)*4 + 4)); 
        updateVect[(int)indMeas_data[idx - 1] - 1] = 0.0;
        ar = H->size[1];
        br = (idx << 2) - 4;
        for (i13 = 0; i13 < ar; i13++) {
          for (i14 = 0; i14 < 4; i14++) {
            H->data[(i14 + br) + H->size[0] * i13] = 0.0;
          }
        }

        br = (idx << 2) - 4;
        for (i13 = 0; i13 < 4; i13++) {
          r_data[i13 + br] = 0.0;
        }

        for (i13 = 0; i13 < 3; i13++) {
          map[i13 + 3 * ((int)indMeas_data[idx - 1] - 1)] = -100.0;
        }

        //  only important for plot
      }
    }

    if ((H->size[1] == 1) || (P_apr->size[0] == 1)) {
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = H->size[0];
      c_y->size[1] = P_apr->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      ar = H->size[0];
      for (i13 = 0; i13 < ar; i13++) {
        kidx = P_apr->size[1];
        for (i14 = 0; i14 < kidx; i14++) {
          c_y->data[i13 + c_y->size[0] * i14] = 0.0;
          idx = H->size[1];
          for (br = 0; br < idx; br++) {
            c_y->data[i13 + c_y->size[0] * i14] += H->data[i13 + H->size[0] * br]
              * P_apr->data[br + P_apr->size[0] * i14];
          }
        }
      }
    } else {
      k = H->size[1];
      kidx = H->size[0];
      idx = P_apr->size[1];
      m = H->size[0];
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = kidx;
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      ar = kidx * idx;
      for (i13 = 0; i13 < ar; i13++) {
        c_y->data[i13] = 0.0;
      }

      if ((H->size[0] == 0) || (P_apr->size[1] == 0)) {
      } else {
        idx = H->size[0] * (P_apr->size[1] - 1);
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          i13 = kidx + m;
          for (ic = kidx; ic + 1 <= i13; ic++) {
            c_y->data[ic] = 0.0;
          }

          kidx += m;
        }

        br = 0;
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          ar = -1;
          i13 = br + k;
          for (ib = br; ib + 1 <= i13; ib++) {
            if (P_apr->data[ib] != 0.0) {
              ia = ar;
              i14 = kidx + m;
              for (ic = kidx; ic + 1 <= i14; ic++) {
                ia++;
                c_y->data[ic] += P_apr->data[ib] * H->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          kidx += m;
        }
      }
    }

    i13 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i13, (int)sizeof(double));
    ar = H->size[0];
    for (i13 = 0; i13 < ar; i13++) {
      kidx = H->size[1];
      for (i14 = 0; i14 < kidx; i14++) {
        b->data[i14 + b->size[0] * i13] = H->data[i13 + H->size[0] * i14];
      }
    }

    emxInit_real_T(&C, 2);
    if ((c_y->size[1] == 1) || (b->size[0] == 1)) {
      i13 = C->size[0] * C->size[1];
      C->size[0] = c_y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i13, (int)sizeof(double));
      ar = c_y->size[0];
      for (i13 = 0; i13 < ar; i13++) {
        kidx = b->size[1];
        for (i14 = 0; i14 < kidx; i14++) {
          C->data[i13 + C->size[0] * i14] = 0.0;
          idx = c_y->size[1];
          for (br = 0; br < idx; br++) {
            C->data[i13 + C->size[0] * i14] += c_y->data[i13 + c_y->size[0] * br]
              * b->data[br + b->size[0] * i14];
          }
        }
      }
    } else {
      k = c_y->size[1];
      kidx = c_y->size[0];
      idx = b->size[1];
      m = c_y->size[0];
      i13 = C->size[0] * C->size[1];
      C->size[0] = kidx;
      emxEnsureCapacity((emxArray__common *)C, i13, (int)sizeof(double));
      i13 = C->size[0] * C->size[1];
      C->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)C, i13, (int)sizeof(double));
      ar = kidx * idx;
      for (i13 = 0; i13 < ar; i13++) {
        C->data[i13] = 0.0;
      }

      if ((c_y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        idx = c_y->size[0] * (b->size[1] - 1);
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          i13 = kidx + m;
          for (ic = kidx; ic + 1 <= i13; ic++) {
            C->data[ic] = 0.0;
          }

          kidx += m;
        }

        br = 0;
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          ar = -1;
          i13 = br + k;
          for (ib = br; ib + 1 <= i13; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i14 = kidx + m;
              for (ic = kidx; ic + 1 <= i14; ic++) {
                ia++;
                C->data[ic] += b->data[ib] * c_y->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          kidx += m;
        }
      }
    }

    i13 = S->size[0] * S->size[1];
    S->size[0] = C->size[0];
    S->size[1] = C->size[1];
    emxEnsureCapacity((emxArray__common *)S, i13, (int)sizeof(double));
    ar = C->size[0] * C->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      S->data[i13] = C->data[i13] + R->data[i13];
    }

    emxFree_real_T(&R);
    i13 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i13, (int)sizeof(double));
    ar = H->size[0];
    for (i13 = 0; i13 < ar; i13++) {
      kidx = H->size[1];
      for (i14 = 0; i14 < kidx; i14++) {
        b->data[i14 + b->size[0] * i13] = H->data[i13 + H->size[0] * i14];
      }
    }

    if ((P_apr->size[1] == 1) || (b->size[0] == 1)) {
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = P_apr->size[0];
      c_y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      ar = P_apr->size[0];
      for (i13 = 0; i13 < ar; i13++) {
        kidx = b->size[1];
        for (i14 = 0; i14 < kidx; i14++) {
          c_y->data[i13 + c_y->size[0] * i14] = 0.0;
          idx = P_apr->size[1];
          for (br = 0; br < idx; br++) {
            c_y->data[i13 + c_y->size[0] * i14] += P_apr->data[i13 + P_apr->
              size[0] * br] * b->data[br + b->size[0] * i14];
          }
        }
      }
    } else {
      k = P_apr->size[1];
      kidx = P_apr->size[0];
      idx = b->size[1];
      m = P_apr->size[0];
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = kidx;
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      i13 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)c_y, i13, (int)sizeof(double));
      ar = kidx * idx;
      for (i13 = 0; i13 < ar; i13++) {
        c_y->data[i13] = 0.0;
      }

      if ((P_apr->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        idx = P_apr->size[0] * (b->size[1] - 1);
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          i13 = kidx + m;
          for (ic = kidx; ic + 1 <= i13; ic++) {
            c_y->data[ic] = 0.0;
          }

          kidx += m;
        }

        br = 0;
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          ar = -1;
          i13 = br + k;
          for (ib = br; ib + 1 <= i13; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i14 = kidx + m;
              for (ic = kidx; ic + 1 <= i14; ic++) {
                ia++;
                c_y->data[ic] += b->data[ib] * P_apr->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          kidx += m;
        }
      }
    }

    emxInit_real_T(&K, 2);
    mrdivide(c_y, S, K);
    emxFree_real_T(&c_y);
    b_emxInit_real_T(&x_apo, 1);
    if (r_size[0] == 1) {
      i13 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i13, (int)sizeof(double));
      ar = K->size[0];
      for (i13 = 0; i13 < ar; i13++) {
        x_apo->data[i13] = 0.0;
        kidx = K->size[1];
        for (i14 = 0; i14 < kidx; i14++) {
          x_apo->data[i13] += K->data[i13 + K->size[0] * i14] * r_data[i14];
        }
      }
    } else {
      k = K->size[1];
      kidx = K->size[0];
      m = K->size[0];
      i13 = x_apo->size[0];
      x_apo->size[0] = kidx;
      emxEnsureCapacity((emxArray__common *)x_apo, i13, (int)sizeof(double));
      for (i13 = 0; i13 < kidx; i13++) {
        x_apo->data[i13] = 0.0;
      }

      if (K->size[0] == 0) {
      } else {
        kidx = 0;
        while ((m > 0) && (kidx <= 0)) {
          for (ic = 1; ic <= m; ic++) {
            x_apo->data[ic - 1] = 0.0;
          }

          kidx = m;
        }

        br = 0;
        kidx = 0;
        while ((m > 0) && (kidx <= 0)) {
          ar = -1;
          i13 = br + k;
          for (ib = br; ib + 1 <= i13; ib++) {
            if (r_data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                x_apo->data[ic] += r_data[ib] * K->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          kidx = m;
        }
      }
    }

    i13 = r5->size[0];
    r5->size[0] = x_apo->size[0];
    emxEnsureCapacity((emxArray__common *)r5, i13, (int)sizeof(boolean_T));
    ar = x_apo->size[0];
    for (i13 = 0; i13 < ar; i13++) {
      r5->data[i13] = rtIsNaN(x_apo->data[i13]);
    }

    if (d_any(g_any(r5))) {
      sb_fprintf();
    }

    emxFree_boolean_T(&r5);
    i13 = r3->size[0] * r3->size[1];
    r3->size[0] = K->size[0];
    r3->size[1] = K->size[1];
    emxEnsureCapacity((emxArray__common *)r3, i13, (int)sizeof(boolean_T));
    ar = K->size[0] * K->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      r3->data[i13] = rtIsNaN(K->data[i13]);
    }

    e_any(r3, r4);
    if (f_any(r4)) {
      ub_fprintf();
    }

    i13 = r3->size[0] * r3->size[1];
    r3->size[0] = S->size[0];
    r3->size[1] = S->size[1];
    emxEnsureCapacity((emxArray__common *)r3, i13, (int)sizeof(boolean_T));
    ar = S->size[0] * S->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      r3->data[i13] = rtIsNaN(S->data[i13]);
    }

    e_any(r3, r4);
    emxFree_boolean_T(&r3);
    if (f_any(r4)) {
      wb_fprintf();
    }

    emxFree_boolean_T(&r4);
    if (det(S) < 0.001) {
      yb_fprintf();
    }

    emxFree_real_T(&S);
    eye(numStates + trailSize * 6.0, b_y);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i13 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i13, (int)sizeof(double));
      ar = K->size[0];
      for (i13 = 0; i13 < ar; i13++) {
        kidx = H->size[1];
        for (i14 = 0; i14 < kidx; i14++) {
          C->data[i13 + C->size[0] * i14] = 0.0;
          idx = K->size[1];
          for (br = 0; br < idx; br++) {
            C->data[i13 + C->size[0] * i14] += K->data[i13 + K->size[0] * br] *
              H->data[br + H->size[0] * i14];
          }
        }
      }
    } else {
      k = K->size[1];
      kidx = K->size[0];
      idx = H->size[1];
      m = K->size[0];
      i13 = C->size[0] * C->size[1];
      C->size[0] = kidx;
      emxEnsureCapacity((emxArray__common *)C, i13, (int)sizeof(double));
      i13 = C->size[0] * C->size[1];
      C->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)C, i13, (int)sizeof(double));
      ar = kidx * idx;
      for (i13 = 0; i13 < ar; i13++) {
        C->data[i13] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        idx = K->size[0] * (H->size[1] - 1);
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          i13 = kidx + m;
          for (ic = kidx; ic + 1 <= i13; ic++) {
            C->data[ic] = 0.0;
          }

          kidx += m;
        }

        br = 0;
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          ar = -1;
          i13 = br + k;
          for (ib = br; ib + 1 <= i13; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i14 = kidx + m;
              for (ic = kidx; ic + 1 <= i14; ic++) {
                ia++;
                C->data[ic] += H->data[ib] * K->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          kidx += m;
        }
      }
    }

    emxFree_real_T(&K);
    emxFree_real_T(&H);
    i13 = b_y->size[0] * b_y->size[1];
    emxEnsureCapacity((emxArray__common *)b_y, i13, (int)sizeof(double));
    kidx = b_y->size[0];
    idx = b_y->size[1];
    ar = kidx * idx;
    for (i13 = 0; i13 < ar; i13++) {
      b_y->data[i13] -= C->data[i13];
    }

    emxFree_real_T(&C);
    i13 = b->size[0] * b->size[1];
    b->size[0] = P_apr->size[0];
    b->size[1] = P_apr->size[1];
    emxEnsureCapacity((emxArray__common *)b, i13, (int)sizeof(double));
    ar = P_apr->size[0] * P_apr->size[1];
    for (i13 = 0; i13 < ar; i13++) {
      b->data[i13] = P_apr->data[i13];
    }

    emxInit_real_T(&d_y, 2);
    if ((b_y->size[1] == 1) || (P_apr->size[0] == 1)) {
      i13 = d_y->size[0] * d_y->size[1];
      d_y->size[0] = b_y->size[0];
      d_y->size[1] = P_apr->size[1];
      emxEnsureCapacity((emxArray__common *)d_y, i13, (int)sizeof(double));
      ar = b_y->size[0];
      for (i13 = 0; i13 < ar; i13++) {
        kidx = P_apr->size[1];
        for (i14 = 0; i14 < kidx; i14++) {
          d_y->data[i13 + d_y->size[0] * i14] = 0.0;
          idx = b_y->size[1];
          for (br = 0; br < idx; br++) {
            d_y->data[i13 + d_y->size[0] * i14] += b_y->data[i13 + b_y->size[0] *
              br] * P_apr->data[br + P_apr->size[0] * i14];
          }
        }
      }

      i13 = P_apr->size[0] * P_apr->size[1];
      P_apr->size[0] = d_y->size[0];
      P_apr->size[1] = d_y->size[1];
      emxEnsureCapacity((emxArray__common *)P_apr, i13, (int)sizeof(double));
      ar = d_y->size[1];
      for (i13 = 0; i13 < ar; i13++) {
        kidx = d_y->size[0];
        for (i14 = 0; i14 < kidx; i14++) {
          P_apr->data[i14 + P_apr->size[0] * i13] = d_y->data[i14 + d_y->size[0]
            * i13];
        }
      }
    } else {
      k = b_y->size[1];
      kidx = b_y->size[0];
      idx = P_apr->size[1];
      m = b_y->size[0];
      i13 = P_apr->size[0] * P_apr->size[1];
      P_apr->size[0] = kidx;
      P_apr->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)P_apr, i13, (int)sizeof(double));
      for (i13 = 0; i13 < idx; i13++) {
        for (i14 = 0; i14 < kidx; i14++) {
          P_apr->data[i14 + P_apr->size[0] * i13] = 0.0;
        }
      }

      if ((b_y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        idx = b_y->size[0] * (b->size[1] - 1);
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          i13 = kidx + m;
          for (ic = kidx; ic + 1 <= i13; ic++) {
            P_apr->data[ic] = 0.0;
          }

          kidx += m;
        }

        br = 0;
        kidx = 0;
        while ((m > 0) && (kidx <= idx)) {
          ar = -1;
          i13 = br + k;
          for (ib = br; ib + 1 <= i13; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i14 = kidx + m;
              for (ic = kidx; ic + 1 <= i14; ic++) {
                ia++;
                P_apr->data[ic] += b->data[ib] * b_y->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          kidx += m;
        }
      }
    }

    emxFree_real_T(&d_y);
    emxFree_real_T(&b);
    emxFree_real_T(&b_y);

    //  update the current state
    for (i13 = 0; i13 < 3; i13++) {
      c_xt[i13] = b_xt->data[i13] + x_apo->data[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      b_xt->data[i13] = c_xt[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      c_xt[i13] = x_apo->data[3 + i13];
    }

    quatPlusThetaJ(c_xt, dv624);
    quatmultJ(dv624, *(double (*)[4])&b_xt->data[3], y);
    for (i13 = 0; i13 < 4; i13++) {
      b_xt->data[3 + i13] = y[i13];
    }

    for (i13 = 0; i13 < 6; i13++) {
      d_xt[i13] = b_xt->data[7 + i13] + x_apo->data[6 + i13];
    }

    for (i13 = 0; i13 < 6; i13++) {
      b_xt->data[7 + i13] = d_xt[i13];
    }

    //  update the trailing poses
    for (kidx = 0; kidx < (int)trailSize; kidx++) {
      d3 = numStatesxt + ((1.0 + (double)kidx) - 1.0) * 7.0;
      d4 = numStatesxt + ((1.0 + (double)kidx) - 1.0) * 7.0;
      d5 = numStates + ((1.0 + (double)kidx) - 1.0) * 6.0;
      for (i13 = 0; i13 < 3; i13++) {
        c_xt[i13] = b_xt->data[(int)(d4 + (1.0 + (double)i13)) - 1] +
          x_apo->data[(int)(d5 + (1.0 + (double)i13)) - 1];
      }

      for (i13 = 0; i13 < 3; i13++) {
        b_xt->data[(int)(d3 + (1.0 + (double)i13)) - 1] = c_xt[i13];
      }

      d3 = numStates + ((1.0 + (double)kidx) - 1.0) * 6.0;
      for (i13 = 0; i13 < 3; i13++) {
        c_xt[i13] = x_apo->data[(int)(d3 + (4.0 + (double)i13)) - 1];
      }

      d3 = numStatesxt + ((1.0 + (double)kidx) - 1.0) * 7.0;
      for (i13 = 0; i13 < 4; i13++) {
        e_xt[i13] = b_xt->data[(int)(d3 + (4.0 + (double)i13)) - 1];
      }

      quatPlusThetaJ(c_xt, dv625);
      quatmultJ(dv625, e_xt, y);
      d3 = numStatesxt + ((1.0 + (double)kidx) - 1.0) * 7.0;
      for (i13 = 0; i13 < 4; i13++) {
        b_xt->data[(int)(d3 + (4.0 + (double)i13)) - 1] = y[i13];
      }
    }

    emxFree_real_T(&x_apo);

    //  check if the camera has moved enough since the last trail pose to
    //  create a new trail pose
    for (idx = 0; idx < ii_size_idx_0; idx++) {
      kidx = ((int)indMeas_data[idx] - 1) * 4;
      br = idx << 2;
      for (i13 = 0; i13 < 4; i13++) {
        h_u_apo->data[kidx + i13] = h_u_data[i13 + br];
      }
    }
  }

  //  for i = 1:length(updateVect_out)
  //      fprintf('h_u_apo(%i): (%f, %f) (%f, %f)', int32(i), h_u_apo((i-1)*4 + 1), h_u_apo((i-1)*4 + 2), h_u_apo((i-1)*4 + 3), h_u_apo((i-1)*4 + 4)); 
  //      if updateVect_out(i) == 1
  //          fprintf('\n');
  //      elseif updateVect_out(i) == 2
  //          fprintf('(new) \n');
  //      else
  //          fprintf('(rejected) \n');
  //      end
  //  end
  // %
  for (idx = 0; idx < 32; idx++) {
    if (updateVect[idx] == 2.0) {
      //  a new feature
      //  fprintf('Feature %i: ', i)
      //          measurementHistory((i-1)*measDim + (1:measDim), :) = nan; % clean up the measurement  history for this feature 
      pointInMap[idx] = 0.0;
      for (i13 = 0; i13 < 3; i13++) {
        map[i13 + 3 * idx] = rtNaN;
      }

      //  fill in nan for un-used map points so they won't be plotted
      br = idx << 1;
      kidx = idx << 1;
      for (i13 = 0; i13 < 2; i13++) {
        b_z_all_l[i13] = z_all_l[i13 + br];
        b_z_all_r[i13] = z_all_r[i13 + kidx];
      }

      initializePoint(b_xt, cameraparams_r_lr, cameraparams_R_lr, b_z_all_l,
                      b_z_all_r, newestTrailPos);
      for (br = 0; br < 3; br++) {
        bv2[br] = rtIsNaN(newestTrailPos[br]);
      }

      if (!c_any(bv2)) {
        for (i13 = 0; i13 < 3; i13++) {
          map[i13 + 3 * idx] = newestTrailPos[i13];
        }

        pointInMap[idx] = 1.0;
        updateVect[idx] = 1.0;
      } else {
        for (i13 = 0; i13 < 3; i13++) {
          map[i13 + 3 * idx] = rtNaN;
        }

        pointInMap[idx] = 0.0;
        updateVect[idx] = 0.0;
      }
    }

    memcpy(&map_out[0], &map[0], 96U * sizeof(double));
  }

  //  h_u_apo = zeros(size(updateVect_out,1)*4,1);
  //                  h_ci = zeros(3,1);
  //                  h_ci(3) = f*b/z_curr(3);
  //                  h_ci(1) = (z_curr(1)-Cx)/f*h_ci(3);
  //                  h_ci(2) = (z_curr(2)-Cy)/f*h_ci(3);
  //
  //                  fp = R_cw'*h_ci+r_cw;
  //              else % postpone the initialization of this feature
  //  %                 fprintf('delaying initialization\n')
  //                  age(i) = 0;
  //                  updateVect_out(i) = 1;
  //                  pointInMap(i) = 0;
  //              end
  //          end
  //      if updateVect_out(i) == 1 && pointInMap(i) == 0  && createNewTrail % an existing, yet to be initialized, feature 
  //          age(i) = age(i) + 1; % only increment the age if we created a new trailing pose 
  //          poses(i*7-6:i*7,age(i)) = xt(1:7);
  //          measurementHistory((i-1)*measDim + (1:measDim), age(i)) = z_curr;
  //      end
  //
  //      % check if a feature has reached maximum age and needs to be
  //      % triangulated and inserted into the map
  //      %     if pointInMap(i) == 0 && age(i) == trailSize
  //      % %         fprintf('Feature %i: triangulating\n', i)
  //      %         noiseParams.u_var_prime = 10;
  //      %         noiseParams.v_var_prime = 10;
  //      %
  //      %         for j=1:trailSize
  //      %             camStates1{j}.q_CG = xt(numStatesxt + (j-1)*7 + (4:7));
  //      %             camStates1{j}.p_C_G = xt(numStatesxt + (j-1)*7 + (1:3)) ; 
  //      %             pose = poses(i*7-6:i*7,j);
  //      %             camStates{j}.q_CG = pose(4:7);
  //      %             camStates{j}.p_C_G = pose(1:3);
  //      %             z_curr = measurementHistory((i-1)*measDim + (1:3),j);
  //      %             observations(:,j) = (z_curr(1:2) -[Cx;Cy])/f;
  //      %
  //      %         end
  //      %         z_curr = z_all((i-1)*measDim + (1:measDim));% debug
  //
  //      %   [ fp2 ] = PointTriangulation(xt, z_cur, cameraparams, 0);
  //
  //      %         h_ci = zeros(3,1);% debug
  //      %         h_ci(3) = f*b/z_curr(3);% debug
  //      %         h_ci(1) = (z_curr(1)-Cx)/f*h_ci(3);% debug
  //      %         h_ci(2) = (z_curr(2)-Cy)/f*h_ci(3);% debug
  //      %
  //      %         fp = R_cw'*h_ci+r_cw;% debug
  //      %         %[fp, Jnew, RCOND] = triangulationReference(camStates1, observations, noiseParams,fp); 
  //      %
  //      %         [ fp ] = PointTriangulation(xt,  measurementHistory((i-1)*measDim + (1:measDim), :), cameraparams, 2); 
  //      %         if any(isnan(fp))
  //      %             warning(['Failed to triangulate feature ', num2str(i)])
  //      %         else
  //      %            % map(:, i) = fp;
  //      %             pointInMap(i) = 0;
  //      %             age(i) = 0;
  //      %             [H_xc_0i,r_0i  ] = MSCKF_Update(  P_apr,xt,fp, measurementHistory((i-1)*measDim + (1:measDim), :),cameraparams ); 
  //      %             if any(isnan(r_0i))
  //      %                 error('err')
  //      %             end
  //      %         H_xc_0=[H_xc_0;H_xc_0i]; r_0=[r_0;r_0i];
  //      %         measurementHistory((i-1)*measDim + (1:measDim), :) = nan; % clean up the measurement  history for this feature 
  //      %         msckfUPD=1;
  //      %         end
  //
  //      % [ fp ] = PointTriangulation(xt, measurementHistory((i-1)*measDim + (1:measDim), :), cameraparams, 2); 
  //      %
  //
  //
  //      %end
  //  end
  //
  //  numFeaturesInMap = nnz(pointInMap);
  //  if numFeaturesInMap < minFeatureThreshold
  //      %  disp(['Forcing the insertion of ', num2str(minFeatureThreshold - numFeaturesInMap), ' map features']) 
  //      % need to initialize some features. Find the ones with highest
  //      % disparity
  //      disparities = z_all(3:3:end);
  //      disparities(pointInMap == 1) = 0; % only look at features not yet in map 
  //      disparities(isnan(disparities)) = 0; % remove invalid disparities (NaN is bigger than Inf in matlab) 
  //      [~, maxDispIndx] = sort(disparities, 'descend');
  //      forceTriangulation = zeros(size(updateVect_out));
  //      forceTriangulation(maxDispIndx(1:minFeatureThreshold - numFeaturesInMap)) = 1; % triangulate the features with the highest disparities 
  //
  //      %     for i = 1:size(updateVect_out)
  //      %
  //      %         if forceTriangulation(i)
  //      %             % mode 0
  //      % %             fprintf('Feature %i: forcing initialization\n', i)
  //      %             z_curr = z_all((i-1)*measDim + (1:measDim));
  //      %             z_curr(3) = max(0, z_curr(3)); % negative disparities don't make sense 
  //      %
  //      %             h_ci = zeros(3,1);
  //      %             h_ci(3) = f*b/z_curr(3);
  //      %             h_ci(1) = (z_curr(1)-Cx)/f*h_ci(3);
  //      %             h_ci(2) = (z_curr(2)-Cy)/f*h_ci(3);
  //      %
  //      %             fp = R_cw'*h_ci+r_cw;
  //      %
  //      %             map(:, i) = fp;
  //      %             pointInMap(i) = 1;
  //      %             updateVect_out(i) = 1;
  //      %             age(i) = 0;
  //      %             measurementHistory((i-1)*measDim + (1:measDim), :) = nan; % clean up the measurement  history for this feature 
  //      %         end
  //      %     end
  //  end
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_updIT_init()
{
  int i10;

  //      poses=[];
  //  the index of the oldest trailing state in xt
  for (i10 = 0; i10 < 96; i10++) {
    map[i10] = -100.0;
  }

  //  the map holding the feature positions in XYZ world coordinates
  memset(&pointInMap[0], 0, sizeof(double) << 5);

  //  stores whether a feature is in the map and can be used to update the state 
  //  the age of each feature to be initialized (i.e. the number of trailing poses in which it was observed) 
  //      measurementHistory = zeros(length(z_all_l), trailSize); % the old measurements for each trailing position 
}

//
// File trailer for SLAM_updIT.cpp
//
// [EOF]
//
