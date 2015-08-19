//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "Att_upd.h"
#include "Att_pred.h"
#include "fprintf.h"
#include "SLAM_updIT.h"
#include "SLAM_pred_model.h"
#include "any.h"
#include "initializePoint.h"
#include "eig.h"
#include "QuatFromRotJ.h"
#include "sum.h"
#include "all.h"
#include "diag.h"
#include "squeeze.h"
#include "predictMeasurement_stereo.h"
#include "blkdiag.h"
#include "repmat.h"
#include "rdivide.h"
#include "norm.h"
#include "cross.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Variable Definitions
static double init_counter;
static boolean_T init_counter_not_empty;
static double model_state[3];
static double model_transition[9];
static struct_T SLAM_data[3];

// Function Declarations
static boolean_T eml_relop(const creal_T a, const creal_T b);
static void filterReinitialization(struct_T b_SLAM_data[3], const double
  b_model_state[3], const double b_model_transition[9], double
  model_state_prior[3]);
static void fuseEstimates(const struct_T b_SLAM_data[3], const double
  b_model_state[3], emxArray_real_T *x_fused, emxArray_real_T *P_fused);
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : const creal_T a
//                const creal_T b
// Return Type  : boolean_T
//
static boolean_T eml_relop(const creal_T a, const creal_T b)
{
  boolean_T p;
  double absbi;
  double y;
  double absxk;
  int exponent;
  double absar;
  double absbr;
  double Ma;
  int b_exponent;
  int c_exponent;
  int d_exponent;
  if ((fabs(a.re) > 8.9884656743115785E+307) || (fabs(a.im) >
       8.9884656743115785E+307) || (fabs(b.re) > 8.9884656743115785E+307) ||
      (fabs(b.im) > 8.9884656743115785E+307)) {
    absbi = rt_hypotd_snf(a.re / 2.0, a.im / 2.0);
    y = rt_hypotd_snf(b.re / 2.0, b.im / 2.0);
  } else {
    absbi = rt_hypotd_snf(a.re, a.im);
    y = rt_hypotd_snf(b.re, b.im);
  }

  absxk = y / 2.0;
  if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
    if (absxk <= 2.2250738585072014E-308) {
      absxk = 4.94065645841247E-324;
    } else {
      frexp(absxk, &exponent);
      absxk = ldexp(1.0, exponent - 53);
    }
  } else {
    absxk = rtNaN;
  }

  if ((fabs(y - absbi) < absxk) || (rtIsInf(absbi) && rtIsInf(y) && ((absbi >
         0.0) == (y > 0.0)))) {
    p = true;
  } else {
    p = false;
  }

  if (p) {
    absar = fabs(a.re);
    absxk = fabs(a.im);
    absbr = fabs(b.re);
    absbi = fabs(b.im);
    if (absar > absxk) {
      Ma = absar;
      absar = absxk;
    } else {
      Ma = absxk;
    }

    if (absbr > absbi) {
      absxk = absbr;
      absbr = absbi;
    } else {
      absxk = absbi;
    }

    if (Ma > absxk) {
      if (absar < absbr) {
        absbi = Ma - absxk;
        y = (absar / 2.0 + absbr / 2.0) / (Ma / 2.0 + absxk / 2.0) * (absbr -
          absar);
      } else {
        absbi = Ma;
        y = absxk;
      }
    } else if (Ma < absxk) {
      if (absar > absbr) {
        y = absxk - Ma;
        absbi = (absar / 2.0 + absbr / 2.0) / (Ma / 2.0 + absxk / 2.0) * (absar
          - absbr);
      } else {
        absbi = Ma;
        y = absxk;
      }
    } else {
      absbi = absar;
      y = absbr;
    }

    absxk = fabs(y / 2.0);
    if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
      if (absxk <= 2.2250738585072014E-308) {
        absxk = 4.94065645841247E-324;
      } else {
        frexp(absxk, &b_exponent);
        absxk = ldexp(1.0, b_exponent - 53);
      }
    } else {
      absxk = rtNaN;
    }

    if ((fabs(y - absbi) < absxk) || (rtIsInf(absbi) && rtIsInf(y) && ((absbi >
           0.0) == (y > 0.0)))) {
      p = true;
    } else {
      p = false;
    }

    if (p) {
      absbi = rt_atan2d_snf(a.im, a.re);
      y = rt_atan2d_snf(b.im, b.re);
      absxk = fabs(y / 2.0);
      if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
        if (absxk <= 2.2250738585072014E-308) {
          absxk = 4.94065645841247E-324;
        } else {
          frexp(absxk, &c_exponent);
          absxk = ldexp(1.0, c_exponent - 53);
        }
      } else {
        absxk = rtNaN;
      }

      if ((fabs(y - absbi) < absxk) || (rtIsInf(absbi) && rtIsInf(y) && ((absbi >
             0.0) == (y > 0.0)))) {
        p = true;
      } else {
        p = false;
      }

      if (p) {
        if (absbi > 0.78539816339744828) {
          if (absbi > 2.3561944901923448) {
            absbi = -a.im;
            y = -b.im;
          } else {
            absbi = -a.re;
            y = -b.re;
          }
        } else if (absbi > -0.78539816339744828) {
          absbi = a.im;
          y = b.im;
        } else if (absbi > -2.3561944901923448) {
          absbi = a.re;
          y = b.re;
        } else {
          absbi = -a.im;
          y = -b.im;
        }

        absxk = fabs(y / 2.0);
        if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
          if (absxk <= 2.2250738585072014E-308) {
            absxk = 4.94065645841247E-324;
          } else {
            frexp(absxk, &d_exponent);
            absxk = ldexp(1.0, d_exponent - 53);
          }
        } else {
          absxk = rtNaN;
        }

        if ((fabs(y - absbi) < absxk) || (rtIsInf(absbi) && rtIsInf(y) &&
             ((absbi > 0.0) == (y > 0.0)))) {
          p = true;
        } else {
          p = false;
        }

        if (p) {
          absbi = 0.0;
          y = 0.0;
        }
      }
    }
  }

  return absbi > y;
}

//
// Arguments    : struct_T b_SLAM_data[3]
//                const double b_model_state[3]
//                const double b_model_transition[9]
//                double model_state_prior[3]
// Return Type  : void
//
static void filterReinitialization(struct_T b_SLAM_data[3], const double
  b_model_state[3], const double b_model_transition[9], double
  model_state_prior[3])
{
  double b_numAnchors;
  double c_numStatesxt;
  double c_numStates;
  double b_numStatesPerAnchorxt;
  double b_numStatesPerAnchor;
  double b_numPointsPerAnchor;
  double mixing_weight[9];
  int i29;
  int i30;
  int ixstart;
  emxArray_real_T *x_fused;
  emxArray_real_T *P_fused;
  emxArray_real_T *M;
  emxArray_real_T *x_diff;
  emxArray_int32_T *r33;
  struct_T c_SLAM_data;
  struct_T d_SLAM_data;
  emxArray_int32_T *r34;
  emxArray_int32_T *r35;
  emxArray_real_T *e_SLAM_data;
  emxArray_int32_T *r36;
  emxArray_int32_T *r37;
  emxArray_real_T *b_x_fused;
  int i;
  int ix;
  int modelIt;
  int i31;
  int itmp;
  int anchorIdx;
  int i32;
  double d_numStatesxt;
  double b[3];
  double e_numStatesxt;
  double c_x_fused[3];
  double errQuat[4];
  double b_M[12];
  double Q[12];
  double b_Q[16];
  creal_T D[16];
  creal_T V[16];
  creal_T varargin_1[4];
  creal_T mtmp;
  boolean_T exitg2;
  boolean_T exitg1;
  double f_SLAM_data[9];
  double g_SLAM_data[9];
  double d_x_fused[9];
  double d_numStates;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double h_SLAM_data[9];
  double j_a[9];
  b_numAnchors = b_SLAM_data[0].numAnchors;
  c_numStatesxt = b_SLAM_data[0].numStatesxt;
  c_numStates = b_SLAM_data[0].numStates;
  b_numStatesPerAnchorxt = b_SLAM_data[0].numStatesPerAnchorxt;
  b_numStatesPerAnchor = b_SLAM_data[0].numStatesPerAnchor;
  b_numPointsPerAnchor = b_SLAM_data[0].numPointsPerAnchor;

  //  predict model state
  for (i29 = 0; i29 < 3; i29++) {
    model_state_prior[i29] = 0.0;
    for (i30 = 0; i30 < 3; i30++) {
      model_state_prior[i29] += b_model_transition[i29 + 3 * i30] *
        b_model_state[i30];
    }

    for (ixstart = 0; ixstart < 3; ixstart++) {
      mixing_weight[ixstart + 3 * i29] = b_model_transition[i29 + 3 * ixstart] *
        b_model_state[ixstart] / model_state_prior[i29];
    }
  }

  emxInit_real_T(&x_fused, 1);
  b_emxInit_real_T(&P_fused, 2);
  c_emxInit_real_T(&M, 3);
  emxInit_real_T(&x_diff, 1);
  b_emxInit_int32_T(&r33, 2);
  emxInitStruct_struct_T(&c_SLAM_data);
  emxInitStruct_struct_T(&d_SLAM_data);
  emxInit_int32_T(&r34, 1);
  emxInit_int32_T(&r35, 1);
  b_emxInit_real_T(&e_SLAM_data, 2);
  emxInit_int32_T(&r36, 1);
  emxInit_int32_T(&r37, 1);
  emxInit_real_T(&b_x_fused, 1);
  for (i = 0; i < 3; i++) {
    i29 = x_fused->size[0];
    x_fused->size[0] = b_SLAM_data[0].xt->size[0];
    emxEnsureCapacity((emxArray__common *)x_fused, i29, (int)sizeof(double));
    ix = b_SLAM_data[0].xt->size[0];
    for (i29 = 0; i29 < ix; i29++) {
      x_fused->data[i29] = 0.0;
    }

    ixstart = b_SLAM_data[0].P->size[0];
    i29 = P_fused->size[0] * P_fused->size[1];
    P_fused->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)P_fused, i29, (int)sizeof(double));
    ixstart = b_SLAM_data[0].P->size[0];
    i29 = P_fused->size[0] * P_fused->size[1];
    P_fused->size[1] = ixstart;
    emxEnsureCapacity((emxArray__common *)P_fused, i29, (int)sizeof(double));
    ix = b_SLAM_data[0].P->size[0] * b_SLAM_data[0].P->size[0];
    for (i29 = 0; i29 < ix; i29++) {
      P_fused->data[i29] = 0.0;
    }

    //  the fused covariance. This is only needed for the output of SLAM. Otherwise the calculation of this could be skipped 
    i29 = M->size[0] * M->size[1] * M->size[2];
    M->size[0] = (int)(b_numAnchors + 1.0);
    M->size[1] = 4;
    M->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)M, i29, (int)sizeof(double));
    ix = ((int)(b_numAnchors + 1.0) << 2) * 3;
    for (i29 = 0; i29 < ix; i29++) {
      M->data[i29] = 0.0;
    }

    //  quaternion averaging matrix
    for (modelIt = 0; modelIt < 3; modelIt++) {
      for (i29 = 0; i29 < 3; i29++) {
        x_fused->data[i29] += mixing_weight[modelIt + 3 * i] *
          b_SLAM_data[modelIt].xt->data[i29];
      }

      for (i29 = 0; i29 < 4; i29++) {
        M->data[M->size[0] * i29 + M->size[0] * M->size[1] * modelIt] =
          mixing_weight[modelIt + 3 * i] * b_SLAM_data[modelIt].xt->data[3 + i29];
      }

      if (8.0 > c_numStatesxt) {
        i29 = 1;
        i30 = 0;
      } else {
        i29 = 8;
        i30 = (int)c_numStatesxt;
      }

      if (8.0 > c_numStatesxt) {
        i31 = 1;
        itmp = 0;
      } else {
        i31 = 8;
        itmp = (int)c_numStatesxt;
      }

      if (8.0 > c_numStatesxt) {
        anchorIdx = 0;
        i32 = 0;
      } else {
        anchorIdx = 7;
        i32 = (int)c_numStatesxt;
      }

      ixstart = r33->size[0] * r33->size[1];
      r33->size[0] = 1;
      r33->size[1] = i32 - anchorIdx;
      emxEnsureCapacity((emxArray__common *)r33, ixstart, (int)sizeof(int));
      ix = i32 - anchorIdx;
      for (i32 = 0; i32 < ix; i32++) {
        r33->data[r33->size[0] * i32] = anchorIdx + i32;
      }

      anchorIdx = r36->size[0];
      r36->size[0] = (i30 - i29) + 1;
      emxEnsureCapacity((emxArray__common *)r36, anchorIdx, (int)sizeof(int));
      ix = i30 - i29;
      for (i30 = 0; i30 <= ix; i30++) {
        r36->data[i30] = i29 + i30;
      }

      i29 = r37->size[0];
      r37->size[0] = (itmp - i31) + 1;
      emxEnsureCapacity((emxArray__common *)r37, i29, (int)sizeof(int));
      ix = itmp - i31;
      for (i29 = 0; i29 <= ix; i29++) {
        r37->data[i29] = i31 + i29;
      }

      i29 = b_x_fused->size[0];
      b_x_fused->size[0] = r33->size[0] * r33->size[1];
      emxEnsureCapacity((emxArray__common *)b_x_fused, i29, (int)sizeof(double));
      ix = r33->size[0] * r33->size[1];
      for (i29 = 0; i29 < ix; i29++) {
        b_x_fused->data[i29] = x_fused->data[r36->data[i29] - 1] +
          mixing_weight[modelIt + 3 * i] * b_SLAM_data[modelIt].xt->data
          [r37->data[i29] - 1];
      }

      ix = b_x_fused->size[0];
      for (i29 = 0; i29 < ix; i29++) {
        x_fused->data[r33->data[i29]] = b_x_fused->data[i29];
      }

      for (anchorIdx = 0; anchorIdx < (int)b_numAnchors; anchorIdx++) {
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt;
        for (i29 = 0; i29 < 3; i29++) {
          b[i29] = b_SLAM_data[modelIt].xt->data[(int)(d_numStatesxt + (1.0 +
            (double)i29)) - 1];
        }

        d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt;
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt;
        for (i29 = 0; i29 < 3; i29++) {
          c_x_fused[i29] = x_fused->data[(int)(e_numStatesxt + (1.0 + (double)
            i29)) - 1] + mixing_weight[modelIt + 3 * i] * b[i29];
        }

        for (i29 = 0; i29 < 3; i29++) {
          x_fused->data[(int)(d_numStatesxt + (1.0 + (double)i29)) - 1] =
            c_x_fused[i29];
        }

        d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt;
        for (i29 = 0; i29 < 4; i29++) {
          errQuat[i29] = b_SLAM_data[modelIt].xt->data[(int)(d_numStatesxt +
            (4.0 + (double)i29)) - 1];
        }

        for (i29 = 0; i29 < 4; i29++) {
          M->data[(((int)((1.0 + (double)anchorIdx) + 1.0) + M->size[0] * i29) +
                   M->size[0] * M->size[1] * modelIt) - 1] =
            mixing_weight[modelIt + 3 * i] * errQuat[i29];
        }

        for (ixstart = 0; ixstart < (int)b_numPointsPerAnchor; ixstart++) {
          x_fused->data[(int)(((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0)
                                * b_numStatesPerAnchorxt) + 7.0) + (1.0 +
            (double)ixstart)) - 1] += mixing_weight[modelIt + 3 * i] *
            b_SLAM_data[modelIt].xt->data[(int)(((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) + (1.0 +
            (double)ixstart)) - 1];
        }
      }
    }

    //  get the average quaternions
    for (i29 = 0; i29 < 3; i29++) {
      for (i30 = 0; i30 < 4; i30++) {
        b_M[i30 + (i29 << 2)] = M->data[M->size[0] * i30 + M->size[0] * M->size
          [1] * i29];
      }
    }

    squeeze(b_M, Q);
    for (i29 = 0; i29 < 4; i29++) {
      for (i30 = 0; i30 < 4; i30++) {
        b_Q[i29 + (i30 << 2)] = 0.0;
        for (i31 = 0; i31 < 3; i31++) {
          b_Q[i29 + (i30 << 2)] += Q[i29 + (i31 << 2)] * Q[i30 + (i31 << 2)];
        }
      }
    }

    eig(b_Q, V, D);
    diag(D, varargin_1);
    ixstart = 1;
    mtmp = varargin_1[0];
    itmp = 0;
    if (rtIsNaN(varargin_1[0].re) || rtIsNaN(varargin_1[0].im)) {
      ix = 2;
      exitg2 = false;
      while ((!exitg2) && (ix < 5)) {
        ixstart = ix;
        if (!(rtIsNaN(varargin_1[ix - 1].re) || rtIsNaN(varargin_1[ix - 1].im)))
        {
          mtmp = varargin_1[ix - 1];
          itmp = ix - 1;
          exitg2 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < 4) {
      while (ixstart + 1 < 5) {
        if (eml_relop(varargin_1[ixstart], mtmp)) {
          mtmp = varargin_1[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }

    for (i29 = 0; i29 < 4; i29++) {
      x_fused->data[3 + i29] = V[i29 + (itmp << 2)].re;
    }

    for (anchorIdx = 0; anchorIdx < (int)b_numAnchors; anchorIdx++) {
      for (ixstart = 0; ixstart < 12; ixstart++) {
        Q[ixstart] = M->data[(((int)((1.0 + (double)anchorIdx) + 1.0) + M->size
          [0] * (ixstart % 4)) + M->size[0] * M->size[1] * (ixstart >> 2)) - 1];
      }

      for (i29 = 0; i29 < 4; i29++) {
        for (i30 = 0; i30 < 4; i30++) {
          b_Q[i29 + (i30 << 2)] = 0.0;
          for (i31 = 0; i31 < 3; i31++) {
            b_Q[i29 + (i30 << 2)] += Q[i29 + (i31 << 2)] * Q[i30 + (i31 << 2)];
          }
        }
      }

      eig(b_Q, V, D);
      for (ixstart = 0; ixstart < 4; ixstart++) {
        varargin_1[ixstart] = D[ixstart * 5];
      }

      ixstart = 1;
      mtmp = varargin_1[0];
      itmp = 0;
      if (rtIsNaN(varargin_1[0].re) || rtIsNaN(varargin_1[0].im)) {
        ix = 2;
        exitg1 = false;
        while ((!exitg1) && (ix < 5)) {
          ixstart = ix;
          if (!(rtIsNaN(varargin_1[ix - 1].re) || rtIsNaN(varargin_1[ix - 1].im)))
          {
            mtmp = varargin_1[ix - 1];
            itmp = ix - 1;
            exitg1 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < 4) {
        while (ixstart + 1 < 5) {
          if (eml_relop(varargin_1[ixstart], mtmp)) {
            mtmp = varargin_1[ixstart];
            itmp = ixstart;
          }

          ixstart++;
        }
      }

      d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt;
      for (i29 = 0; i29 < 4; i29++) {
        x_fused->data[(int)(d_numStatesxt + (4.0 + (double)i29)) - 1] = V[i29 +
          (itmp << 2)].re;
      }
    }

    for (modelIt = 0; modelIt < 3; modelIt++) {
      i29 = x_diff->size[0];
      x_diff->size[0] = b_SLAM_data[0].P->size[0];
      emxEnsureCapacity((emxArray__common *)x_diff, i29, (int)sizeof(double));
      ix = b_SLAM_data[0].P->size[0];
      for (i29 = 0; i29 < ix; i29++) {
        x_diff->data[i29] = 0.0;
      }

      //  difference between fused state and model state (error state)
      for (i29 = 0; i29 < 3; i29++) {
        x_diff->data[i29] = x_fused->data[i29] - b_SLAM_data[modelIt].xt->
          data[i29];
      }

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      g_SLAM_data[0] = ((b_SLAM_data[modelIt].xt->data[3] * b_SLAM_data[modelIt]
                         .xt->data[3] - b_SLAM_data[modelIt].xt->data[4] *
                         b_SLAM_data[modelIt].xt->data[4]) - b_SLAM_data[modelIt]
                        .xt->data[5] * b_SLAM_data[modelIt].xt->data[5]) +
        b_SLAM_data[modelIt].xt->data[6] * b_SLAM_data[modelIt].xt->data[6];
      g_SLAM_data[1] = 2.0 * (b_SLAM_data[modelIt].xt->data[3] *
        b_SLAM_data[modelIt].xt->data[4] + b_SLAM_data[modelIt].xt->data[5] *
        b_SLAM_data[modelIt].xt->data[6]);
      g_SLAM_data[2] = 2.0 * (b_SLAM_data[modelIt].xt->data[3] *
        b_SLAM_data[modelIt].xt->data[5] - b_SLAM_data[modelIt].xt->data[4] *
        b_SLAM_data[modelIt].xt->data[6]);
      g_SLAM_data[3] = 2.0 * (b_SLAM_data[modelIt].xt->data[3] *
        b_SLAM_data[modelIt].xt->data[4] - b_SLAM_data[modelIt].xt->data[5] *
        b_SLAM_data[modelIt].xt->data[6]);
      g_SLAM_data[4] = ((-(b_SLAM_data[modelIt].xt->data[3] *
                           b_SLAM_data[modelIt].xt->data[3]) +
                         b_SLAM_data[modelIt].xt->data[4] * b_SLAM_data[modelIt]
                         .xt->data[4]) - b_SLAM_data[modelIt].xt->data[5] *
                        b_SLAM_data[modelIt].xt->data[5]) + b_SLAM_data[modelIt]
        .xt->data[6] * b_SLAM_data[modelIt].xt->data[6];
      g_SLAM_data[5] = 2.0 * (b_SLAM_data[modelIt].xt->data[4] *
        b_SLAM_data[modelIt].xt->data[5] + b_SLAM_data[modelIt].xt->data[3] *
        b_SLAM_data[modelIt].xt->data[6]);
      g_SLAM_data[6] = 2.0 * (b_SLAM_data[modelIt].xt->data[3] *
        b_SLAM_data[modelIt].xt->data[5] + b_SLAM_data[modelIt].xt->data[4] *
        b_SLAM_data[modelIt].xt->data[6]);
      g_SLAM_data[7] = 2.0 * (b_SLAM_data[modelIt].xt->data[4] *
        b_SLAM_data[modelIt].xt->data[5] - b_SLAM_data[modelIt].xt->data[3] *
        b_SLAM_data[modelIt].xt->data[6]);
      g_SLAM_data[8] = ((-(b_SLAM_data[modelIt].xt->data[3] *
                           b_SLAM_data[modelIt].xt->data[3]) -
                         b_SLAM_data[modelIt].xt->data[4] * b_SLAM_data[modelIt]
                         .xt->data[4]) + b_SLAM_data[modelIt].xt->data[5] *
                        b_SLAM_data[modelIt].xt->data[5]) + b_SLAM_data[modelIt]
        .xt->data[6] * b_SLAM_data[modelIt].xt->data[6];
      d_x_fused[0] = ((x_fused->data[3] * x_fused->data[3] - x_fused->data[4] *
                       x_fused->data[4]) - x_fused->data[5] * x_fused->data[5])
        + x_fused->data[6] * x_fused->data[6];
      d_x_fused[3] = 2.0 * (x_fused->data[3] * x_fused->data[4] + x_fused->data
                            [5] * x_fused->data[6]);
      d_x_fused[6] = 2.0 * (x_fused->data[3] * x_fused->data[5] - x_fused->data
                            [4] * x_fused->data[6]);
      d_x_fused[1] = 2.0 * (x_fused->data[3] * x_fused->data[4] - x_fused->data
                            [5] * x_fused->data[6]);
      d_x_fused[4] = ((-(x_fused->data[3] * x_fused->data[3]) + x_fused->data[4]
                       * x_fused->data[4]) - x_fused->data[5] * x_fused->data[5])
        + x_fused->data[6] * x_fused->data[6];
      d_x_fused[7] = 2.0 * (x_fused->data[4] * x_fused->data[5] + x_fused->data
                            [3] * x_fused->data[6]);
      d_x_fused[2] = 2.0 * (x_fused->data[3] * x_fused->data[5] + x_fused->data
                            [4] * x_fused->data[6]);
      d_x_fused[5] = 2.0 * (x_fused->data[4] * x_fused->data[5] - x_fused->data
                            [3] * x_fused->data[6]);
      d_x_fused[8] = ((-(x_fused->data[3] * x_fused->data[3]) - x_fused->data[4]
                       * x_fused->data[4]) + x_fused->data[5] * x_fused->data[5])
        + x_fused->data[6] * x_fused->data[6];
      for (i29 = 0; i29 < 3; i29++) {
        for (i30 = 0; i30 < 3; i30++) {
          f_SLAM_data[i29 + 3 * i30] = 0.0;
          for (i31 = 0; i31 < 3; i31++) {
            f_SLAM_data[i29 + 3 * i30] += g_SLAM_data[i29 + 3 * i31] *
              d_x_fused[i31 + 3 * i30];
          }
        }
      }

      QuatFromRotJ(f_SLAM_data, errQuat);
      for (i29 = 0; i29 < 3; i29++) {
        x_diff->data[3 + i29] = errQuat[i29];
      }

      if (8.0 > c_numStatesxt) {
        i29 = 1;
        i30 = 0;
      } else {
        i29 = 8;
        i30 = (int)c_numStatesxt;
      }

      if (8.0 > c_numStatesxt) {
        i31 = 1;
        itmp = 0;
      } else {
        i31 = 8;
        itmp = (int)c_numStatesxt;
      }

      if (7.0 > c_numStates) {
        anchorIdx = 0;
        i32 = 0;
      } else {
        anchorIdx = 6;
        i32 = (int)c_numStates;
      }

      ixstart = r33->size[0] * r33->size[1];
      r33->size[0] = 1;
      r33->size[1] = i32 - anchorIdx;
      emxEnsureCapacity((emxArray__common *)r33, ixstart, (int)sizeof(int));
      ix = i32 - anchorIdx;
      for (i32 = 0; i32 < ix; i32++) {
        r33->data[r33->size[0] * i32] = anchorIdx + i32;
      }

      anchorIdx = r34->size[0];
      r34->size[0] = (i30 - i29) + 1;
      emxEnsureCapacity((emxArray__common *)r34, anchorIdx, (int)sizeof(int));
      ix = i30 - i29;
      for (i30 = 0; i30 <= ix; i30++) {
        r34->data[i30] = i29 + i30;
      }

      i29 = r35->size[0];
      r35->size[0] = (itmp - i31) + 1;
      emxEnsureCapacity((emxArray__common *)r35, i29, (int)sizeof(int));
      ix = itmp - i31;
      for (i29 = 0; i29 <= ix; i29++) {
        r35->data[i29] = i31 + i29;
      }

      ix = r33->size[0] * r33->size[1];
      for (i29 = 0; i29 < ix; i29++) {
        x_diff->data[r33->data[i29]] = x_fused->data[r34->data[i29] - 1] -
          b_SLAM_data[modelIt].xt->data[r35->data[i29] - 1];
      }

      for (anchorIdx = 0; anchorIdx < (int)b_numAnchors; anchorIdx++) {
        d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchor;
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt;
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt;
        for (i29 = 0; i29 < 3; i29++) {
          x_diff->data[(int)(d_numStates + (1.0 + (double)i29)) - 1] =
            x_fused->data[(int)(d_numStatesxt + (1.0 + (double)i29)) - 1] -
            b_SLAM_data[modelIt].xt->data[(int)(e_numStatesxt + (1.0 + (double)
            i29)) - 1];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        d_numStates = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1];
        d_numStatesxt = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1];
        e_numStatesxt = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1];
        a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        b_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 4.0) - 1];
        c_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 5.0) - 1];
        d_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 6.0) - 1];
        e_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        f_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 4.0) - 1];
        g_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 5.0) - 1];
        h_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 6.0) - 1];
        i_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        h_SLAM_data[0] = ((b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1]
                           * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) -
                           1] - b_SLAM_data[modelIt].xt->data[(int)
                           ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                             b_numStatesPerAnchorxt) + 5.0) - 1] *
                           b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1])
                          - b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1]
                          * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1])
          + b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        h_SLAM_data[1] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)
          ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 4.0) - 1] * b_SLAM_data[modelIt].xt->data
          [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                  b_numStatesPerAnchorxt) + 5.0) - 1] + b_SLAM_data[modelIt].
          xt->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 6.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        h_SLAM_data[2] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)
          ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 4.0) - 1] * b_SLAM_data[modelIt].xt->data
          [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                  b_numStatesPerAnchorxt) + 6.0) - 1] - b_SLAM_data[modelIt].
          xt->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 5.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        h_SLAM_data[3] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)
          ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 4.0) - 1] * b_SLAM_data[modelIt].xt->data
          [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                  b_numStatesPerAnchorxt) + 5.0) - 1] - b_SLAM_data[modelIt].
          xt->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 6.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        h_SLAM_data[4] = ((-(b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) -
                             1] * b_SLAM_data[modelIt].xt->data[(int)
                             ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0)
          * b_numStatesPerAnchorxt) + 4.0) - 1]) + b_SLAM_data[modelIt].xt->
                           data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
          - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] * b_SLAM_data[modelIt].
                           xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1]) -
                          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1]
                          * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1])
          + b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        h_SLAM_data[5] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)
          ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 5.0) - 1] * b_SLAM_data[modelIt].xt->data
          [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                  b_numStatesPerAnchorxt) + 6.0) - 1] + b_SLAM_data[modelIt].
          xt->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 4.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        h_SLAM_data[6] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)
          ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 4.0) - 1] * b_SLAM_data[modelIt].xt->data
          [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                  b_numStatesPerAnchorxt) + 6.0) - 1] + b_SLAM_data[modelIt].
          xt->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 5.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        h_SLAM_data[7] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)
          ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 5.0) - 1] * b_SLAM_data[modelIt].xt->data
          [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                  b_numStatesPerAnchorxt) + 6.0) - 1] - b_SLAM_data[modelIt].
          xt->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 4.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        h_SLAM_data[8] = ((-(b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) -
                             1] * b_SLAM_data[modelIt].xt->data[(int)
                             ((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0)
          * b_numStatesPerAnchorxt) + 4.0) - 1]) - b_SLAM_data[modelIt].xt->
                           data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx)
          - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] * b_SLAM_data[modelIt].
                           xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1]) +
                          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1]
                          * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1])
          + b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1] *
          b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        j_a[0] = ((d_numStates * d_numStates - d_numStatesxt * d_numStatesxt) -
                  e_numStatesxt * e_numStatesxt) + a * a;
        j_a[3] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] +
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        j_a[6] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] -
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        j_a[1] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] -
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        j_a[4] = ((-(b_a * b_a) + c_a * c_a) - d_a * d_a) + e_a * e_a;
        j_a[7] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] +
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        j_a[2] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] +
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        j_a[5] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] -
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                        x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
        j_a[8] = ((-(f_a * f_a) - g_a * g_a) + h_a * h_a) + i_a * i_a;
        for (i29 = 0; i29 < 3; i29++) {
          for (i30 = 0; i30 < 3; i30++) {
            f_SLAM_data[i29 + 3 * i30] = 0.0;
            for (i31 = 0; i31 < 3; i31++) {
              f_SLAM_data[i29 + 3 * i30] += h_SLAM_data[i29 + 3 * i31] * j_a[i31
                + 3 * i30];
            }
          }
        }

        QuatFromRotJ(f_SLAM_data, errQuat);
        d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchor;
        for (i29 = 0; i29 < 3; i29++) {
          x_diff->data[(int)(d_numStates + (4.0 + (double)i29)) - 1] =
            errQuat[i29];
        }

        for (ixstart = 0; ixstart < (int)b_numPointsPerAnchor; ixstart++) {
          x_diff->data[(int)(((c_numStates + ((1.0 + (double)anchorIdx) - 1.0) *
                               b_numStatesPerAnchor) + 6.0) + (1.0 + (double)
            ixstart)) - 1] = x_fused->data[(int)(((c_numStatesxt + ((1.0 +
            (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) + (1.0 +
            (double)ixstart)) - 1] - b_SLAM_data[modelIt].xt->data[(int)
            (((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
               b_numStatesPerAnchorxt) + 7.0) + (1.0 + (double)ixstart)) - 1];
        }
      }

      i29 = e_SLAM_data->size[0] * e_SLAM_data->size[1];
      e_SLAM_data->size[0] = x_diff->size[0];
      e_SLAM_data->size[1] = x_diff->size[0];
      emxEnsureCapacity((emxArray__common *)e_SLAM_data, i29, (int)sizeof(double));
      ix = x_diff->size[0];
      for (i29 = 0; i29 < ix; i29++) {
        ixstart = x_diff->size[0];
        for (i30 = 0; i30 < ixstart; i30++) {
          d_numStates = x_diff->data[i29] * x_diff->data[i30];
          e_SLAM_data->data[i29 + e_SLAM_data->size[0] * i30] =
            b_SLAM_data[modelIt].P->data[i29 + b_SLAM_data[modelIt].P->size[0] *
            i30] + d_numStates;
        }
      }

      i29 = P_fused->size[0] * P_fused->size[1];
      emxEnsureCapacity((emxArray__common *)P_fused, i29, (int)sizeof(double));
      ix = P_fused->size[1];
      for (i29 = 0; i29 < ix; i29++) {
        ixstart = P_fused->size[0];
        for (i30 = 0; i30 < ixstart; i30++) {
          P_fused->data[i30 + P_fused->size[0] * i29] += e_SLAM_data->data[i30 +
            e_SLAM_data->size[0] * i29] * mixing_weight[modelIt + 3 * i];
        }
      }
    }

    i29 = b_SLAM_data[i].xt->size[0];
    b_SLAM_data[i].xt->size[0] = x_fused->size[0];
    emxEnsureCapacity((emxArray__common *)b_SLAM_data[i].xt, i29, (int)sizeof
                      (double));
    ix = x_fused->size[0];
    for (i29 = 0; i29 < ix; i29++) {
      b_SLAM_data[i].xt->data[i29] = x_fused->data[i29];
    }

    i29 = b_SLAM_data[i].P->size[0] * b_SLAM_data[i].P->size[1];
    b_SLAM_data[i].P->size[0] = P_fused->size[0];
    b_SLAM_data[i].P->size[1] = P_fused->size[1];
    emxEnsureCapacity((emxArray__common *)b_SLAM_data[i].P, i29, (int)sizeof
                      (double));
    emxCopyStruct_struct_T(&c_SLAM_data, &b_SLAM_data[i]);
    emxCopyStruct_struct_T(&d_SLAM_data, &b_SLAM_data[i]);
    ix = P_fused->size[0] * P_fused->size[1];
    for (i29 = 0; i29 < ix; i29++) {
      b_SLAM_data[i].P->data[i29] = P_fused->data[i29];
    }
  }

  emxFree_real_T(&b_x_fused);
  emxFree_int32_T(&r37);
  emxFree_int32_T(&r36);
  emxFree_real_T(&e_SLAM_data);
  emxFree_int32_T(&r35);
  emxFree_int32_T(&r34);
  emxFreeStruct_struct_T(&d_SLAM_data);
  emxFreeStruct_struct_T(&c_SLAM_data);
  emxFree_int32_T(&r33);
  emxFree_real_T(&x_diff);
  emxFree_real_T(&M);
  emxFree_real_T(&P_fused);
  emxFree_real_T(&x_fused);
}

//
// Arguments    : const struct_T b_SLAM_data[3]
//                const double b_model_state[3]
//                emxArray_real_T *x_fused
//                emxArray_real_T *P_fused
// Return Type  : void
//
static void fuseEstimates(const struct_T b_SLAM_data[3], const double
  b_model_state[3], emxArray_real_T *x_fused, emxArray_real_T *P_fused)
{
  double b_numAnchors;
  double c_numStatesxt;
  double c_numStates;
  double b_numStatesPerAnchorxt;
  double b_numStatesPerAnchor;
  double b_numPointsPerAnchor;
  int i11;
  int ix;
  int ixstart;
  emxArray_real_T *M;
  emxArray_int32_T *r1;
  emxArray_int32_T *r2;
  emxArray_int32_T *r3;
  emxArray_real_T *b_x_fused;
  int modelIt;
  int i12;
  int i13;
  int itmp;
  int anchorIdx;
  int i14;
  double d_numStatesxt;
  double b[3];
  double e_numStatesxt;
  double c_x_fused[3];
  double errQuat[4];
  double Q[12];
  double b_Q[16];
  creal_T v[16];
  creal_T V[16];
  creal_T d[4];
  creal_T mtmp;
  boolean_T exitg2;
  boolean_T exitg1;
  emxArray_real_T *x_diff;
  emxArray_int32_T *r4;
  emxArray_int32_T *r5;
  emxArray_real_T *c_SLAM_data;
  double d_SLAM_data[9];
  double e_SLAM_data[9];
  double d_x_fused[9];
  double d_numStates;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double f_SLAM_data[9];
  double j_a[9];
  b_numAnchors = b_SLAM_data[0].numAnchors;
  c_numStatesxt = b_SLAM_data[0].numStatesxt;
  c_numStates = b_SLAM_data[0].numStates;
  b_numStatesPerAnchorxt = b_SLAM_data[0].numStatesPerAnchorxt;
  b_numStatesPerAnchor = b_SLAM_data[0].numStatesPerAnchor;
  b_numPointsPerAnchor = b_SLAM_data[0].numPointsPerAnchor;
  i11 = x_fused->size[0];
  x_fused->size[0] = b_SLAM_data[0].xt->size[0];
  emxEnsureCapacity((emxArray__common *)x_fused, i11, (int)sizeof(double));
  ix = b_SLAM_data[0].xt->size[0];
  for (i11 = 0; i11 < ix; i11++) {
    x_fused->data[i11] = 0.0;
  }

  ixstart = b_SLAM_data[0].P->size[0];
  i11 = P_fused->size[0] * P_fused->size[1];
  P_fused->size[0] = ixstart;
  emxEnsureCapacity((emxArray__common *)P_fused, i11, (int)sizeof(double));
  ixstart = b_SLAM_data[0].P->size[0];
  i11 = P_fused->size[0] * P_fused->size[1];
  P_fused->size[1] = ixstart;
  emxEnsureCapacity((emxArray__common *)P_fused, i11, (int)sizeof(double));
  ix = b_SLAM_data[0].P->size[0] * b_SLAM_data[0].P->size[0];
  for (i11 = 0; i11 < ix; i11++) {
    P_fused->data[i11] = 0.0;
  }

  c_emxInit_real_T(&M, 3);

  //  the fused covariance. This is only needed for the output of SLAM. Otherwise the calculation of this could be skipped 
  i11 = M->size[0] * M->size[1] * M->size[2];
  M->size[0] = (int)(b_SLAM_data[0].numAnchors + 1.0);
  M->size[1] = 4;
  M->size[2] = 3;
  emxEnsureCapacity((emxArray__common *)M, i11, (int)sizeof(double));
  ix = ((int)(b_SLAM_data[0].numAnchors + 1.0) << 2) * 3;
  for (i11 = 0; i11 < ix; i11++) {
    M->data[i11] = 0.0;
  }

  b_emxInit_int32_T(&r1, 2);

  //  quaternion averaging matrix
  emxInit_int32_T(&r2, 1);
  emxInit_int32_T(&r3, 1);
  emxInit_real_T(&b_x_fused, 1);
  for (modelIt = 0; modelIt < 3; modelIt++) {
    for (i11 = 0; i11 < 3; i11++) {
      x_fused->data[i11] += b_model_state[modelIt] * b_SLAM_data[modelIt]
        .xt->data[i11];
    }

    for (i11 = 0; i11 < 4; i11++) {
      M->data[M->size[0] * i11 + M->size[0] * M->size[1] * modelIt] =
        b_model_state[modelIt] * b_SLAM_data[modelIt].xt->data[3 + i11];
    }

    if (8.0 > c_numStatesxt) {
      i11 = 1;
      i12 = 0;
    } else {
      i11 = 8;
      i12 = (int)c_numStatesxt;
    }

    if (8.0 > c_numStatesxt) {
      i13 = 1;
      itmp = 0;
    } else {
      i13 = 8;
      itmp = (int)c_numStatesxt;
    }

    if (8.0 > c_numStatesxt) {
      anchorIdx = 0;
      i14 = 0;
    } else {
      anchorIdx = 7;
      i14 = (int)c_numStatesxt;
    }

    ixstart = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    r1->size[1] = i14 - anchorIdx;
    emxEnsureCapacity((emxArray__common *)r1, ixstart, (int)sizeof(int));
    ix = i14 - anchorIdx;
    for (i14 = 0; i14 < ix; i14++) {
      r1->data[r1->size[0] * i14] = anchorIdx + i14;
    }

    anchorIdx = r2->size[0];
    r2->size[0] = (i12 - i11) + 1;
    emxEnsureCapacity((emxArray__common *)r2, anchorIdx, (int)sizeof(int));
    ix = i12 - i11;
    for (i12 = 0; i12 <= ix; i12++) {
      r2->data[i12] = i11 + i12;
    }

    i11 = r3->size[0];
    r3->size[0] = (itmp - i13) + 1;
    emxEnsureCapacity((emxArray__common *)r3, i11, (int)sizeof(int));
    ix = itmp - i13;
    for (i11 = 0; i11 <= ix; i11++) {
      r3->data[i11] = i13 + i11;
    }

    i11 = b_x_fused->size[0];
    b_x_fused->size[0] = r1->size[0] * r1->size[1];
    emxEnsureCapacity((emxArray__common *)b_x_fused, i11, (int)sizeof(double));
    ix = r1->size[0] * r1->size[1];
    for (i11 = 0; i11 < ix; i11++) {
      b_x_fused->data[i11] = x_fused->data[r2->data[i11] - 1] +
        b_model_state[modelIt] * b_SLAM_data[modelIt].xt->data[r3->data[i11] - 1];
    }

    ix = b_x_fused->size[0];
    for (i11 = 0; i11 < ix; i11++) {
      x_fused->data[r1->data[i11]] = b_x_fused->data[i11];
    }

    for (anchorIdx = 0; anchorIdx < (int)b_numAnchors; anchorIdx++) {
      d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt;
      for (i11 = 0; i11 < 3; i11++) {
        b[i11] = b_SLAM_data[modelIt].xt->data[(int)(d_numStatesxt + (1.0 +
          (double)i11)) - 1];
      }

      d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt;
      e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt;
      for (i11 = 0; i11 < 3; i11++) {
        c_x_fused[i11] = x_fused->data[(int)(e_numStatesxt + (1.0 + (double)i11))
          - 1] + b_model_state[modelIt] * b[i11];
      }

      for (i11 = 0; i11 < 3; i11++) {
        x_fused->data[(int)(d_numStatesxt + (1.0 + (double)i11)) - 1] =
          c_x_fused[i11];
      }

      d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt;
      for (i11 = 0; i11 < 4; i11++) {
        errQuat[i11] = b_SLAM_data[modelIt].xt->data[(int)(d_numStatesxt + (4.0
          + (double)i11)) - 1];
      }

      for (i11 = 0; i11 < 4; i11++) {
        M->data[(((int)((1.0 + (double)anchorIdx) + 1.0) + M->size[0] * i11) +
                 M->size[0] * M->size[1] * modelIt) - 1] = b_model_state[modelIt]
          * errQuat[i11];
      }

      for (ixstart = 0; ixstart < (int)b_numPointsPerAnchor; ixstart++) {
        x_fused->data[(int)(((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                              b_numStatesPerAnchorxt) + 7.0) + (1.0 + (double)
          ixstart)) - 1] += b_model_state[modelIt] * b_SLAM_data[modelIt]
          .xt->data[(int)(((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
                            b_numStatesPerAnchorxt) + 7.0) + (1.0 + (double)
          ixstart)) - 1];
      }
    }
  }

  emxFree_real_T(&b_x_fused);
  emxFree_int32_T(&r3);
  emxFree_int32_T(&r2);

  //  get the average quaternions
  for (ixstart = 0; ixstart < 12; ixstart++) {
    Q[ixstart] = M->data[M->size[0] * (ixstart % 4) + M->size[0] * M->size[1] *
      (ixstart >> 2)];
  }

  for (i11 = 0; i11 < 4; i11++) {
    for (i12 = 0; i12 < 4; i12++) {
      b_Q[i11 + (i12 << 2)] = 0.0;
      for (i13 = 0; i13 < 3; i13++) {
        b_Q[i11 + (i12 << 2)] += Q[i11 + (i13 << 2)] * Q[i12 + (i13 << 2)];
      }
    }
  }

  eig(b_Q, V, v);
  for (ixstart = 0; ixstart < 4; ixstart++) {
    d[ixstart] = v[ixstart * 5];
  }

  ixstart = 1;
  mtmp = d[0];
  itmp = 0;
  if (rtIsNaN(d[0].re) || rtIsNaN(d[0].im)) {
    ix = 2;
    exitg2 = false;
    while ((!exitg2) && (ix < 5)) {
      ixstart = ix;
      if (!(rtIsNaN(d[ix - 1].re) || rtIsNaN(d[ix - 1].im))) {
        mtmp = d[ix - 1];
        itmp = ix - 1;
        exitg2 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 4) {
    while (ixstart + 1 < 5) {
      if (eml_relop(d[ixstart], mtmp)) {
        mtmp = d[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  for (i11 = 0; i11 < 4; i11++) {
    x_fused->data[3 + i11] = V[i11 + (itmp << 2)].re;
  }

  for (anchorIdx = 0; anchorIdx < (int)b_numAnchors; anchorIdx++) {
    for (ixstart = 0; ixstart < 12; ixstart++) {
      Q[ixstart] = M->data[(((int)((1.0 + (double)anchorIdx) + 1.0) + M->size[0]
        * (ixstart % 4)) + M->size[0] * M->size[1] * (ixstart >> 2)) - 1];
    }

    for (i11 = 0; i11 < 4; i11++) {
      for (i12 = 0; i12 < 4; i12++) {
        b_Q[i11 + (i12 << 2)] = 0.0;
        for (i13 = 0; i13 < 3; i13++) {
          b_Q[i11 + (i12 << 2)] += Q[i11 + (i13 << 2)] * Q[i12 + (i13 << 2)];
        }
      }
    }

    eig(b_Q, V, v);
    for (ixstart = 0; ixstart < 4; ixstart++) {
      d[ixstart] = v[ixstart * 5];
    }

    ixstart = 1;
    mtmp = d[0];
    itmp = 0;
    if (rtIsNaN(d[0].re) || rtIsNaN(d[0].im)) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix < 5)) {
        ixstart = ix;
        if (!(rtIsNaN(d[ix - 1].re) || rtIsNaN(d[ix - 1].im))) {
          mtmp = d[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < 4) {
      while (ixstart + 1 < 5) {
        if (eml_relop(d[ixstart], mtmp)) {
          mtmp = d[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }

    d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
      b_numStatesPerAnchorxt;
    for (i11 = 0; i11 < 4; i11++) {
      x_fused->data[(int)(d_numStatesxt + (4.0 + (double)i11)) - 1] = V[i11 +
        (itmp << 2)].re;
    }
  }

  emxFree_real_T(&M);
  emxInit_real_T(&x_diff, 1);
  emxInit_int32_T(&r4, 1);
  emxInit_int32_T(&r5, 1);
  b_emxInit_real_T(&c_SLAM_data, 2);
  for (modelIt = 0; modelIt < 3; modelIt++) {
    i11 = x_diff->size[0];
    x_diff->size[0] = b_SLAM_data[0].P->size[0];
    emxEnsureCapacity((emxArray__common *)x_diff, i11, (int)sizeof(double));
    ix = b_SLAM_data[0].P->size[0];
    for (i11 = 0; i11 < ix; i11++) {
      x_diff->data[i11] = 0.0;
    }

    //  difference between fused state and model state (error state)
    for (i11 = 0; i11 < 3; i11++) {
      x_diff->data[i11] = x_fused->data[i11] - b_SLAM_data[modelIt].xt->data[i11];
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    e_SLAM_data[0] = ((b_SLAM_data[modelIt].xt->data[3] * b_SLAM_data[modelIt].
                       xt->data[3] - b_SLAM_data[modelIt].xt->data[4] *
                       b_SLAM_data[modelIt].xt->data[4]) - b_SLAM_data[modelIt].
                      xt->data[5] * b_SLAM_data[modelIt].xt->data[5]) +
      b_SLAM_data[modelIt].xt->data[6] * b_SLAM_data[modelIt].xt->data[6];
    e_SLAM_data[1] = 2.0 * (b_SLAM_data[modelIt].xt->data[3] *
      b_SLAM_data[modelIt].xt->data[4] + b_SLAM_data[modelIt].xt->data[5] *
      b_SLAM_data[modelIt].xt->data[6]);
    e_SLAM_data[2] = 2.0 * (b_SLAM_data[modelIt].xt->data[3] *
      b_SLAM_data[modelIt].xt->data[5] - b_SLAM_data[modelIt].xt->data[4] *
      b_SLAM_data[modelIt].xt->data[6]);
    e_SLAM_data[3] = 2.0 * (b_SLAM_data[modelIt].xt->data[3] *
      b_SLAM_data[modelIt].xt->data[4] - b_SLAM_data[modelIt].xt->data[5] *
      b_SLAM_data[modelIt].xt->data[6]);
    e_SLAM_data[4] = ((-(b_SLAM_data[modelIt].xt->data[3] * b_SLAM_data[modelIt]
                         .xt->data[3]) + b_SLAM_data[modelIt].xt->data[4] *
                       b_SLAM_data[modelIt].xt->data[4]) - b_SLAM_data[modelIt].
                      xt->data[5] * b_SLAM_data[modelIt].xt->data[5]) +
      b_SLAM_data[modelIt].xt->data[6] * b_SLAM_data[modelIt].xt->data[6];
    e_SLAM_data[5] = 2.0 * (b_SLAM_data[modelIt].xt->data[4] *
      b_SLAM_data[modelIt].xt->data[5] + b_SLAM_data[modelIt].xt->data[3] *
      b_SLAM_data[modelIt].xt->data[6]);
    e_SLAM_data[6] = 2.0 * (b_SLAM_data[modelIt].xt->data[3] *
      b_SLAM_data[modelIt].xt->data[5] + b_SLAM_data[modelIt].xt->data[4] *
      b_SLAM_data[modelIt].xt->data[6]);
    e_SLAM_data[7] = 2.0 * (b_SLAM_data[modelIt].xt->data[4] *
      b_SLAM_data[modelIt].xt->data[5] - b_SLAM_data[modelIt].xt->data[3] *
      b_SLAM_data[modelIt].xt->data[6]);
    e_SLAM_data[8] = ((-(b_SLAM_data[modelIt].xt->data[3] * b_SLAM_data[modelIt]
                         .xt->data[3]) - b_SLAM_data[modelIt].xt->data[4] *
                       b_SLAM_data[modelIt].xt->data[4]) + b_SLAM_data[modelIt].
                      xt->data[5] * b_SLAM_data[modelIt].xt->data[5]) +
      b_SLAM_data[modelIt].xt->data[6] * b_SLAM_data[modelIt].xt->data[6];
    d_x_fused[0] = ((x_fused->data[3] * x_fused->data[3] - x_fused->data[4] *
                     x_fused->data[4]) - x_fused->data[5] * x_fused->data[5]) +
      x_fused->data[6] * x_fused->data[6];
    d_x_fused[3] = 2.0 * (x_fused->data[3] * x_fused->data[4] + x_fused->data[5]
                          * x_fused->data[6]);
    d_x_fused[6] = 2.0 * (x_fused->data[3] * x_fused->data[5] - x_fused->data[4]
                          * x_fused->data[6]);
    d_x_fused[1] = 2.0 * (x_fused->data[3] * x_fused->data[4] - x_fused->data[5]
                          * x_fused->data[6]);
    d_x_fused[4] = ((-(x_fused->data[3] * x_fused->data[3]) + x_fused->data[4] *
                     x_fused->data[4]) - x_fused->data[5] * x_fused->data[5]) +
      x_fused->data[6] * x_fused->data[6];
    d_x_fused[7] = 2.0 * (x_fused->data[4] * x_fused->data[5] + x_fused->data[3]
                          * x_fused->data[6]);
    d_x_fused[2] = 2.0 * (x_fused->data[3] * x_fused->data[5] + x_fused->data[4]
                          * x_fused->data[6]);
    d_x_fused[5] = 2.0 * (x_fused->data[4] * x_fused->data[5] - x_fused->data[3]
                          * x_fused->data[6]);
    d_x_fused[8] = ((-(x_fused->data[3] * x_fused->data[3]) - x_fused->data[4] *
                     x_fused->data[4]) + x_fused->data[5] * x_fused->data[5]) +
      x_fused->data[6] * x_fused->data[6];
    for (i11 = 0; i11 < 3; i11++) {
      for (i12 = 0; i12 < 3; i12++) {
        d_SLAM_data[i11 + 3 * i12] = 0.0;
        for (i13 = 0; i13 < 3; i13++) {
          d_SLAM_data[i11 + 3 * i12] += e_SLAM_data[i11 + 3 * i13] *
            d_x_fused[i13 + 3 * i12];
        }
      }
    }

    QuatFromRotJ(d_SLAM_data, errQuat);
    for (i11 = 0; i11 < 3; i11++) {
      x_diff->data[3 + i11] = errQuat[i11];
    }

    if (8.0 > c_numStatesxt) {
      i11 = 1;
      i12 = 0;
    } else {
      i11 = 8;
      i12 = (int)c_numStatesxt;
    }

    if (8.0 > c_numStatesxt) {
      i13 = 1;
      itmp = 0;
    } else {
      i13 = 8;
      itmp = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      anchorIdx = 0;
      i14 = 0;
    } else {
      anchorIdx = 6;
      i14 = (int)c_numStates;
    }

    ixstart = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    r1->size[1] = i14 - anchorIdx;
    emxEnsureCapacity((emxArray__common *)r1, ixstart, (int)sizeof(int));
    ix = i14 - anchorIdx;
    for (i14 = 0; i14 < ix; i14++) {
      r1->data[r1->size[0] * i14] = anchorIdx + i14;
    }

    anchorIdx = r4->size[0];
    r4->size[0] = (i12 - i11) + 1;
    emxEnsureCapacity((emxArray__common *)r4, anchorIdx, (int)sizeof(int));
    ix = i12 - i11;
    for (i12 = 0; i12 <= ix; i12++) {
      r4->data[i12] = i11 + i12;
    }

    i11 = r5->size[0];
    r5->size[0] = (itmp - i13) + 1;
    emxEnsureCapacity((emxArray__common *)r5, i11, (int)sizeof(int));
    ix = itmp - i13;
    for (i11 = 0; i11 <= ix; i11++) {
      r5->data[i11] = i13 + i11;
    }

    ix = r1->size[0] * r1->size[1];
    for (i11 = 0; i11 < ix; i11++) {
      x_diff->data[r1->data[i11]] = x_fused->data[r4->data[i11] - 1] -
        b_SLAM_data[modelIt].xt->data[r5->data[i11] - 1];
    }

    for (anchorIdx = 0; anchorIdx < (int)b_numAnchors; anchorIdx++) {
      d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchor;
      d_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt;
      e_numStatesxt = c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt;
      for (i11 = 0; i11 < 3; i11++) {
        x_diff->data[(int)(d_numStates + (1.0 + (double)i11)) - 1] =
          x_fused->data[(int)(d_numStatesxt + (1.0 + (double)i11)) - 1] -
          b_SLAM_data[modelIt].xt->data[(int)(e_numStatesxt + (1.0 + (double)i11))
          - 1];
      }

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      d_numStates = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1];
      d_numStatesxt = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1];
      e_numStatesxt = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1];
      a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0)
        * b_numStatesPerAnchorxt) + 7.0) - 1];
      b_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
        1.0) * b_numStatesPerAnchorxt) + 4.0) - 1];
      c_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
        1.0) * b_numStatesPerAnchorxt) + 5.0) - 1];
      d_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
        1.0) * b_numStatesPerAnchorxt) + 6.0) - 1];
      e_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
        1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
      f_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
        1.0) * b_numStatesPerAnchorxt) + 4.0) - 1];
      g_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
        1.0) * b_numStatesPerAnchorxt) + 5.0) - 1];
      h_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
        1.0) * b_numStatesPerAnchorxt) + 6.0) - 1];
      i_a = x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) -
        1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
      f_SLAM_data[0] = ((b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                         b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1]
                         - b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
                         b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1])
                        - b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] *
                        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1])
        + b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
      f_SLAM_data[1] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1]
        * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] +
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      f_SLAM_data[2] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1]
        * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] -
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      f_SLAM_data[3] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1]
        * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] -
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      f_SLAM_data[4] = ((-(b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                           b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1])
                         + b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
                         b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1])
                        - b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] *
                        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1])
        + b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
      f_SLAM_data[5] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1]
        * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] +
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      f_SLAM_data[6] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1]
        * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] +
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      f_SLAM_data[7] = 2.0 * (b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1]
        * b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] -
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      f_SLAM_data[8] = ((-(b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] *
                           b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1])
                         - b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] *
                         b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1])
                        + b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] *
                        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt +
        ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1])
        + b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1] *
        b_SLAM_data[modelIt].xt->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
      j_a[0] = ((d_numStates * d_numStates - d_numStatesxt * d_numStatesxt) -
                e_numStatesxt * e_numStatesxt) + a * a;
      j_a[3] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x_fused->data
                      [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt) + 5.0) - 1] + x_fused->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1]
                      * x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      j_a[6] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x_fused->data
                      [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt) + 6.0) - 1] - x_fused->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1]
                      * x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      j_a[1] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x_fused->data
                      [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt) + 5.0) - 1] - x_fused->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1]
                      * x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      j_a[4] = ((-(b_a * b_a) + c_a * c_a) - d_a * d_a) + e_a * e_a;
      j_a[7] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] * x_fused->data
                      [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt) + 6.0) - 1] + x_fused->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1]
                      * x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      j_a[2] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x_fused->data
                      [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt) + 6.0) - 1] + x_fused->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1]
                      * x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      j_a[5] = 2.0 * (x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] * x_fused->data
                      [(int)((c_numStatesxt + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchorxt) + 6.0) - 1] - x_fused->data[(int)((c_numStatesxt
        + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1]
                      * x_fused->data[(int)((c_numStatesxt + ((1.0 + (double)
        anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1]);
      j_a[8] = ((-(f_a * f_a) - g_a * g_a) + h_a * h_a) + i_a * i_a;
      for (i11 = 0; i11 < 3; i11++) {
        for (i12 = 0; i12 < 3; i12++) {
          d_SLAM_data[i11 + 3 * i12] = 0.0;
          for (i13 = 0; i13 < 3; i13++) {
            d_SLAM_data[i11 + 3 * i12] += f_SLAM_data[i11 + 3 * i13] * j_a[i13 +
              3 * i12];
          }
        }
      }

      QuatFromRotJ(d_SLAM_data, errQuat);
      d_numStates = c_numStates + ((1.0 + (double)anchorIdx) - 1.0) *
        b_numStatesPerAnchor;
      for (i11 = 0; i11 < 3; i11++) {
        x_diff->data[(int)(d_numStates + (4.0 + (double)i11)) - 1] = errQuat[i11];
      }

      for (ixstart = 0; ixstart < (int)b_numPointsPerAnchor; ixstart++) {
        x_diff->data[(int)(((c_numStates + ((1.0 + (double)anchorIdx) - 1.0) *
                             b_numStatesPerAnchor) + 6.0) + (1.0 + (double)
          ixstart)) - 1] = x_fused->data[(int)(((c_numStatesxt + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) + (1.0 + (double)
          ixstart)) - 1] - b_SLAM_data[modelIt].xt->data[(int)(((c_numStatesxt +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) +
          (1.0 + (double)ixstart)) - 1];
      }
    }

    i11 = c_SLAM_data->size[0] * c_SLAM_data->size[1];
    c_SLAM_data->size[0] = x_diff->size[0];
    c_SLAM_data->size[1] = x_diff->size[0];
    emxEnsureCapacity((emxArray__common *)c_SLAM_data, i11, (int)sizeof(double));
    ix = x_diff->size[0];
    for (i11 = 0; i11 < ix; i11++) {
      ixstart = x_diff->size[0];
      for (i12 = 0; i12 < ixstart; i12++) {
        d_numStates = x_diff->data[i11] * x_diff->data[i12];
        c_SLAM_data->data[i11 + c_SLAM_data->size[0] * i12] =
          b_SLAM_data[modelIt].P->data[i11 + b_SLAM_data[modelIt].P->size[0] *
          i12] + d_numStates;
      }
    }

    i11 = P_fused->size[0] * P_fused->size[1];
    emxEnsureCapacity((emxArray__common *)P_fused, i11, (int)sizeof(double));
    ix = P_fused->size[1];
    for (i11 = 0; i11 < ix; i11++) {
      ixstart = P_fused->size[0];
      for (i12 = 0; i12 < ixstart; i12++) {
        P_fused->data[i12 + P_fused->size[0] * i11] += c_SLAM_data->data[i12 +
          c_SLAM_data->size[0] * i11] * b_model_state[modelIt];
      }
    }
  }

  emxFree_real_T(&c_SLAM_data);
  emxFree_int32_T(&r5);
  emxFree_int32_T(&r4);
  emxFree_int32_T(&r1);
  emxFree_real_T(&x_diff);
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(double)(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// function [ h_u_apo_out, xt_out, updateVect, P_apo_out, map_out, model_state_out, model_probs ] = SLAM(updateVect, z_all_l, z_all_r, dt,processNoise, IMU_measurements, imNoise, numPointsPerAnchor, numAnchors, cameraParams, resetFlag) %#codegen
// Arguments    : double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                double dt
//                const double b_processNoise[4]
//                const double IMU_measurements[23]
//                const double b_imNoise[2]
//                double b_numPointsPerAnchor
//                double b_numAnchors
//                const stereoParameters *cameraParams
//                boolean_T resetFlag
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
// Return Type  : void
//
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, const double b_processNoise[4], const double
          IMU_measurements[23], const double b_imNoise[2], double
          b_numPointsPerAnchor, double b_numAnchors, const stereoParameters
          *cameraParams, boolean_T resetFlag, emxArray_real_T *h_u_apo_out,
          emxArray_real_T *xt_out, emxArray_real_T *P_apo_out, emxArray_real_T
          *map_out)
{
  struct_T SLAM_data_first;
  emxArray_real_T *b_P;
  emxArray_real_T *r7;
  emxArray_real_T *r8;
  emxArray_real_T *r9;
  emxArray_real_T *r10;
  double B;
  double z_n_b[3];
  int i;
  double y_n_b[3];
  static const double dv20[3] = { 1.0, 0.0, 0.0 };

  double b_y_n_b[3];
  double x_n_b[3];
  double b_x_n_b[3];
  double c_x_n_b[9];
  int i15;
  double dv21[4];
  int loop_ub;
  static const signed char iv12[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int modelIt;
  int tmp_data[16];
  struct_T r11;
  struct_T r12;
  struct_T r13;
  struct_T r14;
  struct_T r15;
  struct_T r16;
  struct_T r17;
  struct_T r18;
  struct_T r19;
  struct_T r20;
  struct_T r21;
  struct_T r22;
  struct_T r23;
  struct_T r24;
  struct_T r25;
  struct_T r26;
  double model_updateVects[48];
  double model_probs[3];
  double updateVect_model[16];
  boolean_T b_model_probs[3];
  emxArray_boolean_T *anchorIdx;
  emxArray_int32_T *r27;
  int featureIdx;
  double keep_score;
  int k;
  int minNumValidFeatures;
  int b_anchorIdx;
  emxArray_boolean_T *r28;
  boolean_T exitg2;
  boolean_T s[16];
  boolean_T guard2 = false;
  double initializeNewAnchor;
  double newFeaturesRequested;
  emxArray_boolean_T *r29;
  boolean_T exitg1;
  boolean_T guard1 = false;
  signed char i16;

  //  persistents for attitude estimator
  emxInitStruct_struct_T(&SLAM_data_first);
  b_emxInit_real_T(&b_P, 2);
  emxInit_real_T(&r7, 1);
  b_emxInit_real_T(&r8, 2);
  b_emxInit_real_T(&r9, 2);
  emxInit_real_T(&r10, 1);
  if ((!init_counter_not_empty) || resetFlag) {
    B = norm(*(double (*)[3])&IMU_measurements[3]);
    for (i = 0; i < 3; i++) {
      z_n_b[i] = IMU_measurements[i + 3] / B;
    }

    cross(z_n_b, dv20, y_n_b);
    for (i = 0; i < 3; i++) {
      b_y_n_b[i] = y_n_b[i];
    }

    rdivide(b_y_n_b, norm(y_n_b), y_n_b);
    cross(y_n_b, z_n_b, x_n_b);
    for (i = 0; i < 3; i++) {
      b_x_n_b[i] = x_n_b[i];
    }

    rdivide(b_x_n_b, norm(x_n_b), x_n_b);

    //  create the first structure for coder compatibility
    for (i15 = 0; i15 < 3; i15++) {
      c_x_n_b[i15] = x_n_b[i15];
      c_x_n_b[3 + i15] = y_n_b[i15];
      c_x_n_b[6 + i15] = z_n_b[i15];
    }

    QuatFromRotJ(c_x_n_b, dv21);
    i15 = r10->size[0];
    r10->size[0] = 7 + (int)b_numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)r10, i15, (int)sizeof(double));
    r10->data[0] = 0.0;
    r10->data[1] = 0.0;
    r10->data[2] = 0.0;
    r10->data[3] = 0.0;
    r10->data[4] = 0.0;
    r10->data[5] = 0.0;
    r10->data[6] = 1.0;
    loop_ub = (int)b_numPointsPerAnchor;
    for (i15 = 0; i15 < loop_ub; i15++) {
      r10->data[i15 + 7] = 0.0;
    }

    repmat(r10, b_numAnchors, r7);
    i15 = SLAM_data_first.xt->size[0];
    SLAM_data_first.xt->size[0] = 13 + r7->size[0];
    emxEnsureCapacity((emxArray__common *)SLAM_data_first.xt, i15, (int)sizeof
                      (double));
    SLAM_data_first.xt->data[0] = 0.0;
    SLAM_data_first.xt->data[1] = 0.0;
    SLAM_data_first.xt->data[2] = 0.0;
    for (i15 = 0; i15 < 4; i15++) {
      SLAM_data_first.xt->data[i15 + 3] = dv21[i15];
    }

    SLAM_data_first.xt->data[7] = 0.0;
    SLAM_data_first.xt->data[8] = 0.0;
    SLAM_data_first.xt->data[9] = 0.0;
    for (i15 = 0; i15 < 3; i15++) {
      SLAM_data_first.xt->data[i15 + 10] = IMU_measurements[i15];
    }

    loop_ub = r7->size[0];
    for (i15 = 0; i15 < loop_ub; i15++) {
      SLAM_data_first.xt->data[i15 + 13] = r7->data[i15];
    }

    //  initial real vector
    B = b_numAnchors * (6.0 + b_numPointsPerAnchor);
    i15 = r8->size[0] * r8->size[1];
    r8->size[0] = (int)numStates;
    r8->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(double));
    loop_ub = (int)numStates * (int)numStates;
    for (i15 = 0; i15 < loop_ub; i15++) {
      r8->data[i15] = 0.0;
    }

    i15 = r9->size[0] * r9->size[1];
    r9->size[0] = (int)B;
    r9->size[1] = (int)B;
    emxEnsureCapacity((emxArray__common *)r9, i15, (int)sizeof(double));
    loop_ub = (int)B * (int)B;
    for (i15 = 0; i15 < loop_ub; i15++) {
      r9->data[i15] = 0.0;
    }

    blkdiag(r8, r9, b_P);
    for (i15 = 0; i15 < 3; i15++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_P->data[loop_ub + b_P->size[0] * i15] = 0.0;
      }
    }

    //  position
    for (i15 = 0; i15 < 3; i15++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_P->data[(loop_ub + b_P->size[0] * (3 + i15)) + 3] = iv12[loop_ub + 3 *
          i15];
      }
    }

    //  orientation
    for (i15 = 0; i15 < 3; i15++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_P->data[(loop_ub + b_P->size[0] * (6 + i15)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i15 = 0; i15 < 3; i15++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_P->data[(loop_ub + b_P->size[0] * (9 + i15)) + 9] = 0.0;
      }
    }

    //  gyro bias
    i15 = SLAM_data_first.P->size[0] * SLAM_data_first.P->size[1];
    SLAM_data_first.P->size[0] = b_P->size[0];
    SLAM_data_first.P->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)SLAM_data_first.P, i15, (int)sizeof
                      (double));
    loop_ub = b_P->size[0] * b_P->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      SLAM_data_first.P->data[i15] = b_P->data[i15];
    }

    SLAM_data_first.cameraparams = *cameraParams;
    SLAM_data_first.height_offset_pressure = (1.0 - rt_powd_snf
      (IMU_measurements[9] / 101325.0, 0.190284)) * 145366.45;
    for (i15 = 0; i15 < 4; i15++) {
      SLAM_data_first.processNoise[i15] = b_processNoise[i15];
    }

    for (i15 = 0; i15 < 2; i15++) {
      SLAM_data_first.imNoise[i15] = b_imNoise[i15];
    }

    SLAM_data_first.numPointsPerAnchor = b_numPointsPerAnchor;
    SLAM_data_first.numAnchors = b_numAnchors;
    SLAM_data_first.numStates = numStates;
    SLAM_data_first.numStatesxt = numStatesxt;
    SLAM_data_first.numStatesPerAnchorxt = 7.0 + b_numPointsPerAnchor;
    SLAM_data_first.numStatesPerAnchor = 6.0 + b_numPointsPerAnchor;
    i15 = SLAM_data_first.m_vect->size[0] * SLAM_data_first.m_vect->size[1];
    SLAM_data_first.m_vect->size[0] = 3;
    SLAM_data_first.m_vect->size[1] = (int)(b_numPointsPerAnchor * b_numAnchors);
    emxEnsureCapacity((emxArray__common *)SLAM_data_first.m_vect, i15, (int)
                      sizeof(double));
    loop_ub = 3 * (int)(b_numPointsPerAnchor * b_numAnchors);
    for (i15 = 0; i15 < loop_ub; i15++) {
      SLAM_data_first.m_vect->data[i15] = rtNaN;
    }

    //  a matrix containing the m vectors for each feature
    i15 = SLAM_data_first.anchorFeatures->size[0] *
      SLAM_data_first.anchorFeatures->size[1];
    SLAM_data_first.anchorFeatures->size[0] = (int)(b_numAnchors *
      b_numPointsPerAnchor);
    SLAM_data_first.anchorFeatures->size[1] = (int)b_numAnchors;
    emxEnsureCapacity((emxArray__common *)SLAM_data_first.anchorFeatures, i15,
                      (int)sizeof(double));
    loop_ub = (int)(b_numAnchors * b_numPointsPerAnchor) * (int)b_numAnchors;
    for (i15 = 0; i15 < loop_ub; i15++) {
      SLAM_data_first.anchorFeatures->data[i15] = 0.0;
    }

    //  describes which feature belongs to which anchor
    SLAM_data_first.model = 1.0;
    SLAM_data_first.model_prob = 0.0;
    for (i15 = 0; i15 < 3; i15++) {
      emxCopyStruct_struct_T(&SLAM_data[i15], &SLAM_data_first);
    }

    for (modelIt = 0; modelIt < 3; modelIt++) {
      SLAM_data[modelIt].model = 1.0 + (double)modelIt;
      SLAM_data[modelIt].model_prob = 0.33333333333333331;
    }

    for (i = 0; i < 3; i++) {
      model_state[i] = 0.33333333333333331;
    }

    //  probabilities of each model
    //      model_transition = 0.05*ones(num_models) + (1-0.05*num_models)*eye(num_models); 
    //      model_transition = [0.95, 0.02, 0.02, 0.01;
    //                          0.02, 0.95, 0.02, 0.01;
    //                          0.02, 0.02, 0.95, 0.01;
    //                          0.01, 0.05, 0.05, 0.8];
    for (i15 = 0; i15 < 9; i15++) {
      model_transition[i15] = 4.9999999999994493E-5;
    }

    for (modelIt = 0; modelIt < 3; modelIt++) {
      model_transition[modelIt + 3 * modelIt] = 0.9999;
    }

    init_counter = 0.0;
    init_counter_not_empty = true;
  }

  emxFree_real_T(&r10);
  emxFree_real_T(&r9);
  emxFree_real_T(&r8);
  emxFree_real_T(&r7);
  emxFree_real_T(&b_P);
  if (init_counter < 10.0) {
    //  during initialization, only use one tracker
    emxCopyStruct_struct_T(&SLAM_data_first, &SLAM_data[0]);
    Att_pred(&SLAM_data_first, *(double (*)[3])&IMU_measurements[0], dt);
    emxCopyStruct_struct_T(&SLAM_data[0], &SLAM_data_first);
    emxCopyStruct_struct_T(&SLAM_data_first, &SLAM_data[0]);
    Att_upd(&SLAM_data_first, *(double (*)[3])&IMU_measurements[3], dt);
    emxCopyStruct_struct_T(&SLAM_data[0], &SLAM_data_first);
    init_counter++;
    i15 = xt_out->size[0];
    xt_out->size[0] = SLAM_data[0].xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i15, (int)sizeof(double));
    loop_ub = SLAM_data[0].xt->size[0];
    for (i15 = 0; i15 < loop_ub; i15++) {
      xt_out->data[i15] = SLAM_data[0].xt->data[i15];
    }

    i15 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = SLAM_data[0].P->size[0];
    P_apo_out->size[1] = SLAM_data[0].P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i15, (int)sizeof(double));
    loop_ub = SLAM_data[0].P->size[0] * SLAM_data[0].P->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      P_apo_out->data[i15] = SLAM_data[0].P->data[i15];
    }

    i15 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i15, (int)sizeof(double));
    for (i15 = 0; i15 < 64; i15++) {
      h_u_apo_out->data[i15] = -100.0;
    }

    i15 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, i15, (int)sizeof(double));
    for (i15 = 0; i15 < 48; i15++) {
      map_out->data[i15] = rtNaN;
    }
  } else if (init_counter == 10.0) {
    //  done initializing attitude. Insert the estimated attitude and the covariance into the whole state, request features 
    b_fprintf();
    for (i = 0; i < 16; i++) {
      updateVect[i] = 0.0;
    }

    if (1.0 > b_numPointsPerAnchor) {
      loop_ub = 0;
    } else {
      loop_ub = (int)b_numPointsPerAnchor;
    }

    for (i15 = 0; i15 < loop_ub; i15++) {
      tmp_data[i15] = i15;
    }

    for (i15 = 0; i15 < loop_ub; i15++) {
      updateVect[tmp_data[i15]] = 2.0;
    }

    emxInitStruct_struct_T(&r11);
    emxInitStruct_struct_T(&r12);
    emxInitStruct_struct_T(&r13);
    emxInitStruct_struct_T(&r14);
    emxInitStruct_struct_T(&r15);
    emxInitStruct_struct_T(&r16);
    for (modelIt = 0; modelIt < 2; modelIt++) {
      emxCopyStruct_struct_T(&r11, &SLAM_data[0]);
      i15 = SLAM_data[modelIt + 1].xt->size[0];
      SLAM_data[modelIt + 1].xt->size[0] = r11.xt->size[0];
      emxEnsureCapacity((emxArray__common *)SLAM_data[modelIt + 1].xt, i15, (int)
                        sizeof(double));
      loop_ub = r11.xt->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        SLAM_data[modelIt + 1].xt->data[i15] = r11.xt->data[i15];
      }

      i15 = SLAM_data[modelIt + 1].P->size[0] * SLAM_data[modelIt + 1].P->size[1];
      SLAM_data[modelIt + 1].P->size[0] = SLAM_data[0].P->size[0];
      SLAM_data[modelIt + 1].P->size[1] = SLAM_data[0].P->size[1];
      emxEnsureCapacity((emxArray__common *)SLAM_data[modelIt + 1].P, i15, (int)
                        sizeof(double));
      emxCopyStruct_struct_T(&r12, &SLAM_data[modelIt + 1]);
      emxCopyStruct_struct_T(&r13, &SLAM_data[modelIt + 1]);
      emxCopyStruct_struct_T(&r14, &SLAM_data[0]);
      emxCopyStruct_struct_T(&r15, &SLAM_data[0]);
      emxCopyStruct_struct_T(&r16, &SLAM_data[0]);
      loop_ub = r15.P->size[0] * r16.P->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        SLAM_data[modelIt + 1].P->data[i15] = r14.P->data[i15];
      }
    }

    emxFreeStruct_struct_T(&r16);
    emxFreeStruct_struct_T(&r15);
    emxFreeStruct_struct_T(&r14);
    emxFreeStruct_struct_T(&r13);
    emxFreeStruct_struct_T(&r12);
    emxFreeStruct_struct_T(&r11);
    init_counter = 11.0;
    i15 = xt_out->size[0];
    xt_out->size[0] = SLAM_data[0].xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i15, (int)sizeof(double));
    loop_ub = SLAM_data[0].xt->size[0];
    for (i15 = 0; i15 < loop_ub; i15++) {
      xt_out->data[i15] = SLAM_data[0].xt->data[i15];
    }

    i15 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = SLAM_data[0].P->size[0];
    P_apo_out->size[1] = SLAM_data[0].P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i15, (int)sizeof(double));
    loop_ub = SLAM_data[0].P->size[0] * SLAM_data[0].P->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      P_apo_out->data[i15] = SLAM_data[0].P->data[i15];
    }

    i15 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i15, (int)sizeof(double));
    for (i15 = 0; i15 < 64; i15++) {
      h_u_apo_out->data[i15] = -100.0;
    }

    i15 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, i15, (int)sizeof(double));
    for (i15 = 0; i15 < 48; i15++) {
      map_out->data[i15] = rtNaN;
    }
  } else if (init_counter == 11.0) {
    //  for feature initialization, only use one tracker
    emxCopyStruct_struct_T(&SLAM_data_first, &SLAM_data[0]);
    SLAM_updIT(&SLAM_data_first, updateVect, z_all_l, z_all_r, IMU_measurements,
               h_u_apo_out, map_out);
    emxCopyStruct_struct_T(&SLAM_data[0], &SLAM_data_first);
    emxInitStruct_struct_T(&r11);
    emxInitStruct_struct_T(&r12);
    emxInitStruct_struct_T(&r13);
    emxInitStruct_struct_T(&r14);
    emxInitStruct_struct_T(&r15);
    emxInitStruct_struct_T(&r16);
    emxInitStruct_struct_T(&r17);
    emxInitStruct_struct_T(&r18);
    emxInitStruct_struct_T(&r19);
    emxInitStruct_struct_T(&r20);
    emxInitStruct_struct_T(&r21);
    emxInitStruct_struct_T(&r22);
    emxInitStruct_struct_T(&r23);
    emxInitStruct_struct_T(&r24);
    emxInitStruct_struct_T(&r25);
    emxInitStruct_struct_T(&r26);
    for (modelIt = 0; modelIt < 2; modelIt++) {
      emxCopyStruct_struct_T(&r11, &SLAM_data[0]);
      i15 = SLAM_data[modelIt + 1].xt->size[0];
      SLAM_data[modelIt + 1].xt->size[0] = r11.xt->size[0];
      emxEnsureCapacity((emxArray__common *)SLAM_data[modelIt + 1].xt, i15, (int)
                        sizeof(double));
      loop_ub = r11.xt->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        SLAM_data[modelIt + 1].xt->data[i15] = r11.xt->data[i15];
      }

      i15 = SLAM_data[modelIt + 1].P->size[0] * SLAM_data[modelIt + 1].P->size[1];
      SLAM_data[modelIt + 1].P->size[0] = SLAM_data[0].P->size[0];
      SLAM_data[modelIt + 1].P->size[1] = SLAM_data[0].P->size[1];
      emxEnsureCapacity((emxArray__common *)SLAM_data[modelIt + 1].P, i15, (int)
                        sizeof(double));
      emxCopyStruct_struct_T(&r12, &SLAM_data[modelIt + 1]);
      emxCopyStruct_struct_T(&r13, &SLAM_data[modelIt + 1]);
      emxCopyStruct_struct_T(&r14, &SLAM_data[0]);
      emxCopyStruct_struct_T(&r15, &SLAM_data[0]);
      emxCopyStruct_struct_T(&r16, &SLAM_data[0]);
      loop_ub = r15.P->size[0] * r16.P->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        SLAM_data[modelIt + 1].P->data[i15] = r14.P->data[i15];
      }

      i15 = SLAM_data[modelIt + 1].m_vect->size[0] * SLAM_data[modelIt + 1].
        m_vect->size[1];
      SLAM_data[modelIt + 1].m_vect->size[0] = 3;
      SLAM_data[modelIt + 1].m_vect->size[1] = SLAM_data[0].m_vect->size[1];
      emxEnsureCapacity((emxArray__common *)SLAM_data[modelIt + 1].m_vect, i15,
                        (int)sizeof(double));
      emxCopyStruct_struct_T(&r17, &SLAM_data[modelIt + 1]);
      emxCopyStruct_struct_T(&r18, &SLAM_data[modelIt + 1]);
      emxCopyStruct_struct_T(&r19, &SLAM_data[0]);
      emxCopyStruct_struct_T(&r20, &SLAM_data[0]);
      emxCopyStruct_struct_T(&r21, &SLAM_data[0]);
      loop_ub = r20.m_vect->size[0] * r21.m_vect->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        SLAM_data[modelIt + 1].m_vect->data[i15] = r19.m_vect->data[i15];
      }

      i15 = SLAM_data[modelIt + 1].anchorFeatures->size[0] * SLAM_data[modelIt +
        1].anchorFeatures->size[1];
      SLAM_data[modelIt + 1].anchorFeatures->size[0] = SLAM_data[0].
        anchorFeatures->size[0];
      SLAM_data[modelIt + 1].anchorFeatures->size[1] = SLAM_data[0].
        anchorFeatures->size[1];
      emxEnsureCapacity((emxArray__common *)SLAM_data[modelIt + 1].
                        anchorFeatures, i15, (int)sizeof(double));
      emxCopyStruct_struct_T(&r22, &SLAM_data[modelIt + 1]);
      emxCopyStruct_struct_T(&r23, &SLAM_data[modelIt + 1]);
      emxCopyStruct_struct_T(&r24, &SLAM_data[0]);
      emxCopyStruct_struct_T(&r25, &SLAM_data[0]);
      emxCopyStruct_struct_T(&r26, &SLAM_data[0]);
      loop_ub = r25.anchorFeatures->size[0] * r26.anchorFeatures->size[1];
      for (i15 = 0; i15 < loop_ub; i15++) {
        SLAM_data[modelIt + 1].anchorFeatures->data[i15] =
          r24.anchorFeatures->data[i15];
      }
    }

    emxFreeStruct_struct_T(&r26);
    emxFreeStruct_struct_T(&r25);
    emxFreeStruct_struct_T(&r24);
    emxFreeStruct_struct_T(&r23);
    emxFreeStruct_struct_T(&r22);
    emxFreeStruct_struct_T(&r21);
    emxFreeStruct_struct_T(&r20);
    emxFreeStruct_struct_T(&r19);
    emxFreeStruct_struct_T(&r18);
    emxFreeStruct_struct_T(&r17);
    emxFreeStruct_struct_T(&r16);
    emxFreeStruct_struct_T(&r15);
    emxFreeStruct_struct_T(&r14);
    emxFreeStruct_struct_T(&r13);
    emxFreeStruct_struct_T(&r12);
    emxFreeStruct_struct_T(&r11);
    i15 = xt_out->size[0];
    xt_out->size[0] = SLAM_data[0].xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i15, (int)sizeof(double));
    loop_ub = SLAM_data[0].xt->size[0];
    for (i15 = 0; i15 < loop_ub; i15++) {
      xt_out->data[i15] = SLAM_data[0].xt->data[i15];
    }

    i15 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = SLAM_data[0].P->size[0];
    P_apo_out->size[1] = SLAM_data[0].P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i15, (int)sizeof(double));
    loop_ub = SLAM_data[0].P->size[0] * SLAM_data[0].P->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      P_apo_out->data[i15] = SLAM_data[0].P->data[i15];
    }

    init_counter = 12.0;
  } else {
    //  debug
    // % udate each model
    filterReinitialization(SLAM_data, model_state, model_transition, z_n_b);
    for (i = 0; i < 3; i++) {
      model_state[i] = z_n_b[i];
    }

    //  model_probs = zeros(num_models, 1); % at the top for debug
    for (modelIt = 0; modelIt < 3; modelIt++) {
      emxCopyStruct_struct_T(&SLAM_data_first, &SLAM_data[modelIt]);
      SLAM_pred_model(&SLAM_data_first, dt, b_processNoise, IMU_measurements);
      emxCopyStruct_struct_T(&SLAM_data[modelIt], &SLAM_data_first);
      emxCopyStruct_struct_T(&SLAM_data_first, &SLAM_data[modelIt]);
      for (i = 0; i < 16; i++) {
        updateVect_model[i] = updateVect[i];
      }

      SLAM_updIT(&SLAM_data_first, updateVect_model, z_all_l, z_all_r,
                 IMU_measurements, h_u_apo_out, map_out);
      emxCopyStruct_struct_T(&SLAM_data[modelIt], &SLAM_data_first);
      model_probs[modelIt] = SLAM_data[modelIt].model_prob;
      memcpy(&model_updateVects[modelIt << 4], &updateVect_model[0], sizeof
             (double) << 4);
    }

    //  debug
    //  fprintf('model probs: (%.3f, %.3f, %.3f, %.3f)\n', model_probs/sum(model_probs)) 
    for (i = 0; i < 3; i++) {
      b_model_probs[i] = (model_probs[i] == 0.0);
    }

    if (all(b_model_probs)) {
      l_fprintf();
    } else {
      for (i15 = 0; i15 < 3; i15++) {
        model_state[i15] *= model_probs[i15];
      }
    }

    B = sum(model_state);
    for (i15 = 0; i15 < 3; i15++) {
      model_state[i15] /= B;
    }

    //  debug for plotting
    fuseEstimates(SLAM_data, model_state, xt_out, P_apo_out);

    //  assume the last model is the most general one, take the update vector from 
    //  that one
    //  for modelIt = 1:num_models-1
    //      SLAM_data(modelIt).m_vect = SLAM_data(num_models).m_vect;
    //      SLAM_data(modelIt).anchorFeatures = SLAM_data(num_models).anchorFeatures; 
    //  end
    //  keep all features that were not rejected by all models
    for (i = 0; i < 16; i++) {
      if (updateVect[i] == 2.0) {
        updateVect[i] = 1.0;
      }
    }

    //  assume all new features were initialized
    emxInit_boolean_T(&anchorIdx, 2);
    b_emxInit_int32_T(&r27, 2);
    for (featureIdx = 0; featureIdx < 16; featureIdx++) {
      B = updateVect[featureIdx];
      for (i15 = 0; i15 < 3; i15++) {
        b_model_probs[i15] = (model_updateVects[featureIdx + (i15 << 4)] != B);
      }

      if (f_any(b_model_probs)) {
        keep_score = 0.0;
        for (modelIt = 0; modelIt < 3; modelIt++) {
          keep_score += model_state[modelIt] * model_updateVects[featureIdx +
            (modelIt << 4)];
        }

        if (keep_score < 0.5) {
          updateVect[featureIdx] = 0.0;
          loop_ub = SLAM_data[0].anchorFeatures->size[1];
          i15 = anchorIdx->size[0] * anchorIdx->size[1];
          anchorIdx->size[0] = 1;
          anchorIdx->size[1] = loop_ub;
          emxEnsureCapacity((emxArray__common *)anchorIdx, i15, (int)sizeof
                            (boolean_T));
          for (i15 = 0; i15 < loop_ub; i15++) {
            anchorIdx->data[anchorIdx->size[0] * i15] = (SLAM_data[0].
              anchorFeatures->data[featureIdx + SLAM_data[0]
              .anchorFeatures->size[0] * i15] == 1.0);
          }

          for (modelIt = 0; modelIt < 3; modelIt++) {
            k = anchorIdx->size[1] - 1;
            loop_ub = 0;
            for (i = 0; i <= k; i++) {
              if (anchorIdx->data[i]) {
                loop_ub++;
              }
            }

            i15 = r27->size[0] * r27->size[1];
            r27->size[0] = 1;
            r27->size[1] = loop_ub;
            emxEnsureCapacity((emxArray__common *)r27, i15, (int)sizeof(int));
            loop_ub = 0;
            for (i = 0; i <= k; i++) {
              if (anchorIdx->data[i]) {
                r27->data[loop_ub] = i + 1;
                loop_ub++;
              }
            }

            loop_ub = r27->size[1];
            for (i15 = 0; i15 < loop_ub; i15++) {
              SLAM_data[modelIt].anchorFeatures->data[featureIdx +
                SLAM_data[modelIt].anchorFeatures->size[0] * (r27->data
                [r27->size[0] * i15] - 1)] = -1.0;
            }

            //  mark feature as lost
          }

          n_fprintf((signed char)(1 + featureIdx));
        }
      }
    }

    emxFree_int32_T(&r27);
    emxFree_boolean_T(&anchorIdx);

    //  determine if a new anchor needs to be initialized, and request stereo
    //  measurements for it
    minNumValidFeatures = 10000;
    b_anchorIdx = 0;
    b_emxInit_boolean_T(&r28, 1);
    exitg2 = false;
    while ((!exitg2) && (b_anchorIdx <= (int)b_numAnchors - 1)) {
      loop_ub = SLAM_data[0].anchorFeatures->size[0];
      i15 = r28->size[0];
      r28->size[0] = loop_ub;
      emxEnsureCapacity((emxArray__common *)r28, i15, (int)sizeof(boolean_T));
      for (i15 = 0; i15 < loop_ub; i15++) {
        r28->data[i15] = (SLAM_data[0].anchorFeatures->data[i15 + SLAM_data[0].
                          anchorFeatures->size[0] * b_anchorIdx] == 1.0);
      }

      for (i15 = 0; i15 < 16; i15++) {
        s[i15] = (r28->data[i15] && (updateVect[i15] == 1.0));
      }

      loop_ub = 0;
      for (k = 0; k < 16; k++) {
        if (s[k]) {
          loop_ub++;
        }
      }

      guard2 = false;
      if ((loop_ub < minFeatureThreshold) && (loop_ub < minNumValidFeatures)) {
        minNumValidFeatures = loop_ub;
        initializeNewAnchor = 1.0 + (double)b_anchorIdx;
        if (!(loop_ub != 0)) {
          exitg2 = true;
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        b_anchorIdx++;
      }
    }

    emxFree_boolean_T(&r28);
    if (minNumValidFeatures < minFeatureThreshold) {
      newFeaturesRequested = 0.0;
      i = 0;
      emxInit_boolean_T(&r29, 2);
      exitg1 = false;
      while ((!exitg1) && (i < 16)) {
        loop_ub = SLAM_data[0].anchorFeatures->size[1];
        i15 = r29->size[0] * r29->size[1];
        r29->size[0] = 1;
        r29->size[1] = loop_ub;
        emxEnsureCapacity((emxArray__common *)r29, i15, (int)sizeof(boolean_T));
        for (i15 = 0; i15 < loop_ub; i15++) {
          r29->data[r29->size[0] * i15] = (SLAM_data[0].anchorFeatures->data[i +
            SLAM_data[0].anchorFeatures->size[0] * i15] == 1.0);
        }

        guard1 = false;
        if ((!e_any(r29)) || (SLAM_data[0].anchorFeatures->data[i + SLAM_data[0]
                              .anchorFeatures->size[0] * ((int)
              initializeNewAnchor - 1)] == 1.0)) {
          updateVect[i] = 2.0;
          newFeaturesRequested++;
          if (newFeaturesRequested == b_numPointsPerAnchor) {
            exitg1 = true;
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          i++;
        }
      }

      emxFree_boolean_T(&r29);
      B = rt_roundd_snf(newFeaturesRequested);
      if (B < 128.0) {
        i16 = (signed char)B;
      } else {
        i16 = MAX_int8_T;
      }

      p_fprintf(i16);
    }

    //  debug
    //  updateVect = updateVect_model;
    // % output asserts for coder
  }

  emxFreeStruct_struct_T(&SLAM_data_first);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_free()
{
  emxFreeMatrix_struct_T(SLAM_data);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_init()
{
  emxInitMatrix_struct_T(SLAM_data);
}

//
// Arguments    : void
// Return Type  : void
//
void init_counter_not_empty_init()
{
  init_counter_not_empty = false;
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
