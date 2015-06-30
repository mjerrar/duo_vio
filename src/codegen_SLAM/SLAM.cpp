//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 30-Jun-2015 17:08:06
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"

// Type Definitions
#ifndef struct_emxArray__common
#define struct_emxArray__common

struct emxArray__common
{
  void *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray__common

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_int32_T

// Named Constants
#define b_UseAccToPredict              (true)
#define b_normalGravity                (true)
#define b_trailSize                    (0.0)
#define b_numStates                    (12.0)
#define b_msckfUPD                     (0.0)
#define b_numStatesxt                  (13.0)
#define b_baselineThresold             (0.3)
#define b_minFeatureThreshold          (10.0)
#define b_disparityThreshold           (40.0)

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P_apo;
static double trailSize;
static double numStates;
static double map[96];
static boolean_T map_not_empty;
static double oldestTrail;
static double pointInMap[32];
static emxArray_real_T *measurementHistory;
static double age[32];
static double msckfUPD;
static double numStatesxt;
static double baselineThresold;
static double minFeatureThreshold;
static double disparityThreshold;
static boolean_T UseAccToPredict;
static boolean_T normalGravity;

// Function Declarations
static void PointTriangulation(const emxArray_real_T *b_xt, const double z_cur[3],
  const double cameraparams[4], double fp[3]);
static void QuatFromRotJ(const double R[9], double Q[4]);
static void RotFromQuatJ(const double q[4], double R[9]);
static void SLAM_free();
static void SLAM_init();
static void SLAM_pred_euler(emxArray_real_T *b_P_apo, emxArray_real_T *x, double
  dt, const double processNoise[3], const double IMU_measurements[9], double
  c_numStates);
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams[4], double updateVect[32], const double z_all[96], const
  double imNoise[3], emxArray_real_T *b_P_apo, double map_out[96]);
static void SLAM_updIT_free();
static void SLAM_updIT_init();
static boolean_T any(const boolean_T x[3]);
static void b_diag(const double v[9], double d[81]);
static double b_eml_matlab_zlarfg();
static void b_eml_sort(double x[32], int idx[32]);
static double b_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void b_eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0);
static void b_eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
  emxArray_real_T *B, int ldb);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);
static void b_eye(double I[144]);
static double b_mod(double x, double y);
static double b_norm(const double x[4]);
static double c_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void diag(const double v[3], double d[9]);
static int eml_ixamax(int n, const double x_data[], int ix0);
static void eml_lusolve(const emxArray_real_T *A, const emxArray_real_T *B,
  emxArray_real_T *X);
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, emxArray_real_T *
  C, int ic0, int ldc, double work_data[]);
static double eml_matlab_zlarfg(int n, double *alpha1, emxArray_real_T *x, int
  ix0);
static void eml_qrsolve(const emxArray_real_T *A, const emxArray_real_T *B,
  emxArray_real_T *Y);
static void eml_signed_integer_colon(int b, int y_data[], int y_size[2]);
static void eml_sort(double x[32], int idx[32]);
static void eml_xgeqp3(emxArray_real_T *A, double tau_data[], int tau_size[1],
  int jpvt_data[], int jpvt_size[2]);
static void eml_xgetrf(int m, int n, emxArray_real_T *A, int lda, int ipiv_data[],
  int ipiv_size[2], int *info);
static double eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void eml_xswap(int n, emxArray_real_T *x, int ix0, int incx, int iy0, int
                      incy);
static void eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
                      emxArray_real_T *B, int ldb);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);
static void eye(double varargin_1, emxArray_real_T *I);
static void kron(const double A_data[], const int A_size[2], const double B[9],
                 emxArray_real_T *K);
static void merge(int idx[32], double x[32], int offset, int np, int nq);
static void mrdivide(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *y);
static double norm(const double x[3]);
static void power(const double a[3], double y[3]);
static void quatPlusThetaJ(const double dtheta[3], double dq[4]);
static void quatmultJ(const double q[4], const double p[4], double qp[4]);
static int rankFromQR(const emxArray_real_T *A);
static double rt_hypotd_snf(double u0, double u1);
static double rt_roundd_snf(double u);

// Function Definitions

//
// TRIANGULATION Summary of this function goes here
//    Detailed explanation goes here
// Arguments    : const emxArray_real_T *b_xt
//                const double z_cur[3]
//                const double cameraparams[4]
//                double fp[3]
// Return Type  : void
//
static void PointTriangulation(const emxArray_real_T *b_xt, const double z_cur[3],
  const double cameraparams[4], double fp[3])
{
  double A;
  double y;
  double h_cil[3];
  double c_xt[9];
  int i7;
  int i8;
  A = cameraparams[0] * cameraparams[3];
  y = A / z_cur[2];
  h_cil[2] = A / z_cur[2];
  h_cil[0] = (z_cur[0] - cameraparams[1]) / cameraparams[0] * y;
  h_cil[1] = (z_cur[1] - cameraparams[2]) / cameraparams[0] * y;
  c_xt[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4]) -
             b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  c_xt[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] * b_xt->data[6]);
  c_xt[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] * b_xt->data[6]);
  c_xt[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] * b_xt->data[6]);
  c_xt[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data[4])
             - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  c_xt[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] * b_xt->data[6]);
  c_xt[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] * b_xt->data[6]);
  c_xt[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] * b_xt->data[6]);
  c_xt[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data[4])
             + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  for (i7 = 0; i7 < 3; i7++) {
    A = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      A += c_xt[i7 + 3 * i8] * h_cil[i8];
    }

    fp[i7] = A + b_xt->data[i7];
  }
}

//
// THIS IS OK, It is according to the NASA memo found
//  https://claraty.jpl.nasa.gov/man/software/development/conventions/standards_docs/unadopted/JPL_Quaternions_Breckenridge.pdf
// QUATFROMROTJ Get equivalent quaternion of the rotation matrix R
//    Returns a quaternion in JPL notation
//  The implementation is copied from qGetQ(R), but we are careful about the
//  ordering of the output vector
// Arguments    : const double R[9]
//                double Q[4]
// Return Type  : void
//
static void QuatFromRotJ(const double R[9], double Q[4])
{
  double T;
  double x;
  double pivot[4];
  int ixstart;
  double mtmp;
  int idx;
  boolean_T exitg2;
  signed char ii_data[4];
  boolean_T exitg1;
  boolean_T guard1 = false;
  int loop_ub;
  int i2;
  signed char i_data[4];
  signed char index_data[4];

  //  if( r ~= 3 || c ~= 3 )
  //      error( 'R must be a 3x3 matrix\n\r' );
  //  end
  T = (R[0] + R[4]) + R[8];
  x = sqrt((1.0 + 2.0 * R[0]) - T);
  pivot[0] = x / 2.0;
  pivot[1] = sqrt((1.0 + 2.0 * R[4]) - T) / 2.0;
  pivot[2] = sqrt((1.0 + 2.0 * R[8]) - T) / 2.0;
  pivot[3] = sqrt(1.0 + T) / 2.0;
  ixstart = 1;
  mtmp = x / 2.0;
  if (rtIsNaN(x / 2.0)) {
    idx = 2;
    exitg2 = false;
    while ((!exitg2) && (idx < 5)) {
      ixstart = idx;
      if (!rtIsNaN(pivot[idx - 1])) {
        mtmp = pivot[idx - 1];
        exitg2 = true;
      } else {
        idx++;
      }
    }
  }

  if (ixstart < 4) {
    while (ixstart + 1 < 5) {
      if (pivot[ixstart] > mtmp) {
        mtmp = pivot[ixstart];
      }

      ixstart++;
    }
  }

  idx = 0;
  ixstart = 1;
  exitg1 = false;
  while ((!exitg1) && (ixstart < 5)) {
    guard1 = false;
    if (pivot[ixstart - 1] == mtmp) {
      idx++;
      ii_data[idx - 1] = (signed char)ixstart;
      if (idx >= 4) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      ixstart++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  for (i2 = 0; i2 < loop_ub; i2++) {
    i_data[i2] = ii_data[i2];
  }

  if (1 > idx) {
    ixstart = 0;
  } else {
    ixstart = idx;
  }

  for (i2 = 0; i2 < ixstart; i2++) {
    index_data[i2] = i_data[i2];
  }

  for (ixstart = 0; ixstart < loop_ub; ixstart++) {
    index_data[ixstart] = (signed char)rt_roundd_snf((double)index_data[ixstart]);
  }

  if (index_data[0] == 1) {
    x = sqrt((1.0 + 2.0 * R[0]) - T);
    mtmp = x / 2.0;
    Q[0] = x / 2.0;
    Q[1] = (R[3] + R[1]) / (4.0 * mtmp);
    Q[2] = (R[6] + R[2]) / (4.0 * mtmp);
    Q[3] = (R[7] - R[5]) / (4.0 * mtmp);
  } else if (index_data[0] == 2) {
    x = sqrt((1.0 + 2.0 * R[4]) - T);
    mtmp = x / 2.0;
    Q[1] = x / 2.0;
    Q[0] = (R[3] + R[1]) / (4.0 * mtmp);
    Q[2] = (R[7] + R[5]) / (4.0 * mtmp);
    Q[3] = (R[2] - R[6]) / (4.0 * mtmp);
  } else if (index_data[0] == 3) {
    x = sqrt((1.0 + 2.0 * R[8]) - T);
    mtmp = x / 2.0;
    Q[2] = x / 2.0;
    Q[0] = (R[6] + R[2]) / (4.0 * mtmp);
    Q[1] = (R[7] + R[5]) / (4.0 * mtmp);
    Q[3] = (R[3] - R[1]) / (4.0 * mtmp);
  } else {
    x = sqrt(1.0 + T);
    mtmp = x / 2.0;
    Q[3] = x / 2.0;
    Q[0] = (R[7] - R[5]) / (4.0 * mtmp);
    Q[1] = (R[2] - R[6]) / (4.0 * mtmp);
    Q[2] = (R[3] - R[1]) / (4.0 * mtmp);
  }
}

//
// Arguments    : const double q[4]
//                double R[9]
// Return Type  : void
//
static void RotFromQuatJ(const double q[4], double R[9])
{
  R[0] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
  R[3] = 2.0 * (q[0] * q[1] + q[2] * q[3]);
  R[6] = 2.0 * (q[0] * q[2] - q[1] * q[3]);
  R[1] = 2.0 * (q[0] * q[1] - q[2] * q[3]);
  R[4] = ((-(q[0] * q[0]) + q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
  R[7] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  R[2] = 2.0 * (q[0] * q[2] + q[1] * q[3]);
  R[5] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  R[8] = ((-(q[0] * q[0]) - q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_free()
{
  emxFree_real_T(&P_apo);
  emxFree_real_T(&xt);
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_init()
{
  emxInit_real_T(&P_apo, 2);
  b_emxInit_real_T(&xt, 1);
}

//
// EKF_SLAM: computes the camerapose p and feature position f in an
// interative way
//    Handels a static number of points but can dynamically asign them to new
//    klt points.
//  Xv meaning
//
//                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
//  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
//  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
// Arguments    : emxArray_real_T *b_P_apo
//                emxArray_real_T *x
//                double dt
//                const double processNoise[3]
//                const double IMU_measurements[9]
//                double c_numStates
// Return Type  : void
//
static void SLAM_pred_euler(emxArray_real_T *b_P_apo, emxArray_real_T *x, double
  dt, const double processNoise[3], const double IMU_measurements[9], double
  c_numStates)
{
  double c;
  double R_cw[9];
  double w[3];
  int i15;
  double dv12[9];
  double b_R_cw[9];
  int cr;
  double dv13[9];
  double c_R_cw[9];
  int k;
  static const signed char iv2[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double Fc[144];
  static const signed char iv3[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double Phi[144];
  double G1[108];
  static const signed char iv4[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv5[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int loop_ub;
  int b_loop_ub;
  emxArray_real_T *P_xs_apr;
  emxArray_real_T *c_P_apo;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  double y[144];
  double b_processNoise[9];
  double dv14[81];
  double b_G1[108];
  double c_G1[144];
  emxArray_real_T *d_P_apo;
  emxArray_int32_T *r4;
  emxArray_int32_T *r5;
  emxArray_real_T *b_P_xs_apr;
  double grav[3];
  static const double dv15[3] = { 0.0, 0.0, 9.81 };

  static const double dv16[3] = { 0.0, 0.0, -9.81 };

  double b_x[3];
  double b_w[3];
  double dq[4];
  double c_x;
  double d_x;
  double e_x;
  double f_x;
  double g_x;
  double h_x;
  double i_x;
  double j_x;
  double k_x;
  double l_x;
  double m_x;
  double n_x;
  double o_x;
  double p_x;
  double q_x;
  double r_x[16];
  double b_dq[4];
  double s_x[4];

  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  // % compute the linearization F of the non linear model f
  c = dt * dt;
  RotFromQuatJ(*(double (*)[4])&x->data[3], R_cw);

  //  want R_wi imu to world
  // ===========ACC=====================================
  for (i15 = 0; i15 < 3; i15++) {
    w[i15] = IMU_measurements[i15] - x->data[10 + i15];
  }

  if (UseAccToPredict) {
    dv12[0] = 0.0;
    dv12[3] = -w[2];
    dv12[6] = w[1];
    dv12[1] = w[2];
    dv12[4] = 0.0;
    dv12[7] = -w[0];
    dv12[2] = -w[1];
    dv12[5] = w[0];
    dv12[8] = 0.0;
    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        b_R_cw[cr + 3 * i15] = -R_cw[i15 + 3 * cr];
      }
    }

    dv13[0] = 0.0;
    dv13[3] = -IMU_measurements[5];
    dv13[6] = IMU_measurements[4];
    dv13[1] = IMU_measurements[5];
    dv13[4] = 0.0;
    dv13[7] = -IMU_measurements[3];
    dv13[2] = -IMU_measurements[4];
    dv13[5] = IMU_measurements[3];
    dv13[8] = 0.0;
    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        c_R_cw[i15 + 3 * cr] = 0.0;
        for (k = 0; k < 3; k++) {
          c_R_cw[i15 + 3 * cr] += b_R_cw[i15 + 3 * k] * dv13[k + 3 * cr];
        }
      }
    }

    for (i15 = 0; i15 < 12; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[cr + 12 * i15] = iv2[cr + 3 * i15];
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * i15) + 3] = 0.0;
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 3)) + 3] = -dv12[cr + 3 * i15];
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 6)) + 3] = 0.0;
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 9)) + 3] = iv3[cr + 3 * i15];
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * i15) + 6] = 0.0;
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 3)) + 6] = c_R_cw[cr + 3 * i15];
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 6)) + 6] = 0.0;
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 9)) + 6] = 0.0;
      }
    }

    for (i15 = 0; i15 < 12; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * i15) + 9] = 0.0;
      }
    }
  } else {
    dv12[0] = 0.0;
    dv12[3] = -w[2];
    dv12[6] = w[1];
    dv12[1] = w[2];
    dv12[4] = 0.0;
    dv12[7] = -w[0];
    dv12[2] = -w[1];
    dv12[5] = w[0];
    dv12[8] = 0.0;
    dv13[0] = 0.0;
    dv13[3] = -IMU_measurements[5];
    dv13[6] = IMU_measurements[4];
    dv13[1] = IMU_measurements[5];
    dv13[4] = 0.0;
    dv13[7] = -IMU_measurements[3];
    dv13[2] = -IMU_measurements[4];
    dv13[5] = IMU_measurements[3];
    dv13[8] = 0.0;
    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        b_R_cw[i15 + 3 * cr] = 0.0;
        for (k = 0; k < 3; k++) {
          b_R_cw[i15 + 3 * cr] += -0.0 * R_cw[k + 3 * i15] * dv13[k + 3 * cr];
        }
      }
    }

    for (i15 = 0; i15 < 12; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[cr + 12 * i15] = iv2[cr + 3 * i15];
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * i15) + 3] = 0.0;
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 3)) + 3] = -dv12[cr + 3 * i15];
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 6)) + 3] = 0.0;
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 9)) + 3] = iv3[cr + 3 * i15];
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * i15) + 6] = 0.0;
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 3)) + 6] = b_R_cw[cr + 3 * i15];
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 6)) + 6] = 0.0;
      }
    }

    for (i15 = 0; i15 < 3; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * (i15 + 9)) + 6] = 0.0;
      }
    }

    for (i15 = 0; i15 < 12; i15++) {
      for (cr = 0; cr < 3; cr++) {
        Fc[(cr + 12 * i15) + 9] = 0.0;
      }
    }
  }

  b_eye(Phi);
  for (i15 = 0; i15 < 144; i15++) {
    Phi[i15] += Fc[i15] * dt;
  }

  for (i15 = 0; i15 < 9; i15++) {
    for (cr = 0; cr < 3; cr++) {
      G1[cr + 12 * i15] = 0.0;
    }

    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * i15) + 3] = iv4[cr + 3 * i15];
    }
  }

  for (i15 = 0; i15 < 3; i15++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * i15) + 6] = 0.0;
    }
  }

  for (i15 = 0; i15 < 3; i15++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * (i15 + 3)) + 6] = -R_cw[i15 + 3 * cr];
    }
  }

  for (i15 = 0; i15 < 3; i15++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * (i15 + 6)) + 6] = 0.0;
    }
  }

  for (i15 = 0; i15 < 9; i15++) {
    for (cr = 0; cr < 3; cr++) {
      G1[(cr + 12 * i15) + 9] = iv5[cr + 3 * i15];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)c_numStates;
  }

  emxInit_real_T(&P_xs_apr, 2);
  if (loop_ub == 1) {
    emxInit_real_T(&c_P_apo, 2);
    i15 = c_P_apo->size[0] * c_P_apo->size[1];
    c_P_apo->size[0] = 1;
    c_P_apo->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)c_P_apo, i15, (int)sizeof(double));
    for (i15 = 0; i15 < b_loop_ub; i15++) {
      cr = 0;
      while (cr <= 0) {
        c_P_apo->data[c_P_apo->size[0] * i15] = b_P_apo->data[b_P_apo->size[0] *
          i15];
        cr = 1;
      }
    }

    i15 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = c_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i15, (int)sizeof(double));
    for (i15 = 0; i15 < 12; i15++) {
      loop_ub = c_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i15 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          P_xs_apr->data[i15 + P_xs_apr->size[0] * cr] += Phi[i15 + 12 * k] *
            c_P_apo->data[k + c_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&c_P_apo);
  } else {
    i15 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i15, (int)sizeof(double));
    i15 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = b_loop_ub;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i15, (int)sizeof(double));
    k = 12 * b_loop_ub;
    for (i15 = 0; i15 < k; i15++) {
      P_xs_apr->data[i15] = 0.0;
    }

    if (b_loop_ub == 0) {
    } else {
      b_loop_ub = 12 * (b_loop_ub - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (b_P_apo->data[ib % loop_ub + b_P_apo->size[0] * (ib / loop_ub)] !=
              0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += b_P_apo->data[ib % loop_ub + b_P_apo->size[0]
                * (ib / loop_ub)] * Phi[ia - 1];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  for (i15 = 0; i15 < 12; i15++) {
    for (cr = 0; cr < 12; cr++) {
      Fc[cr + 12 * i15] = Phi[i15 + 12 * cr];
    }
  }

  if (P_xs_apr->size[1] == 1) {
    for (i15 = 0; i15 < 12; i15++) {
      for (cr = 0; cr < 12; cr++) {
        y[i15 + 12 * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          y[i15 + 12 * cr] += P_xs_apr->data[i15 + 12 * k] * Fc[k + 12 * cr];
        }
      }
    }
  } else {
    k = P_xs_apr->size[1];
    memset(&y[0], 0, 144U * sizeof(double));
    for (cr = 0; cr < 134; cr += 12) {
      for (ic = cr; ic + 1 <= cr + 12; ic++) {
        y[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr < 134; cr += 12) {
      ar = 0;
      i15 = br + k;
      for (ib = br; ib + 1 <= i15; ib++) {
        if (Fc[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 12; ic++) {
            ia++;
            y[ic] += Fc[ib] * P_xs_apr->data[ia - 1];
          }
        }

        ar += 12;
      }

      br += k;
    }
  }

  b_processNoise[0] = processNoise[1];
  b_processNoise[1] = processNoise[1];
  b_processNoise[2] = processNoise[1];
  b_processNoise[3] = processNoise[0];
  b_processNoise[4] = processNoise[0];
  b_processNoise[5] = processNoise[0];
  b_processNoise[6] = processNoise[2];
  b_processNoise[7] = processNoise[2];
  b_processNoise[8] = processNoise[2];
  for (i15 = 0; i15 < 9; i15++) {
    b_R_cw[i15] = b_processNoise[i15] * c;
  }

  b_diag(b_R_cw, dv14);
  for (i15 = 0; i15 < 12; i15++) {
    for (cr = 0; cr < 9; cr++) {
      b_G1[i15 + 12 * cr] = 0.0;
      for (k = 0; k < 9; k++) {
        b_G1[i15 + 12 * cr] += G1[i15 + 12 * k] * dv14[k + 9 * cr];
      }
    }

    for (cr = 0; cr < 12; cr++) {
      c_G1[i15 + 12 * cr] = 0.0;
      for (k = 0; k < 9; k++) {
        c_G1[i15 + 12 * cr] += b_G1[i15 + 12 * k] * G1[cr + 12 * k];
      }
    }
  }

  for (i15 = 0; i15 < 12; i15++) {
    for (cr = 0; cr < 12; cr++) {
      Fc[cr + 12 * i15] = y[cr + 12 * i15] + c_G1[cr + 12 * i15] * dt;
    }
  }

  //  covariance of the state
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > b_P_apo->size[1]) {
    i15 = 0;
    cr = 0;
  } else {
    i15 = (int)(c_numStates + 1.0) - 1;
    cr = b_P_apo->size[1];
  }

  if (loop_ub == 1) {
    emxInit_real_T(&d_P_apo, 2);
    k = d_P_apo->size[0] * d_P_apo->size[1];
    d_P_apo->size[0] = 1;
    d_P_apo->size[1] = cr - i15;
    emxEnsureCapacity((emxArray__common *)d_P_apo, k, (int)sizeof(double));
    loop_ub = cr - i15;
    for (cr = 0; cr < loop_ub; cr++) {
      k = 0;
      while (k <= 0) {
        d_P_apo->data[d_P_apo->size[0] * cr] = b_P_apo->data[b_P_apo->size[0] *
          (i15 + cr)];
        k = 1;
      }
    }

    i15 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = d_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i15, (int)sizeof(double));
    for (i15 = 0; i15 < 12; i15++) {
      loop_ub = d_P_apo->size[1];
      for (cr = 0; cr < loop_ub; cr++) {
        P_xs_apr->data[i15 + P_xs_apr->size[0] * cr] = 0.0;
        for (k = 0; k < 12; k++) {
          P_xs_apr->data[i15 + P_xs_apr->size[0] * cr] += Phi[i15 + 12 * k] *
            d_P_apo->data[k + d_P_apo->size[0] * cr];
        }
      }
    }

    emxFree_real_T(&d_P_apo);
  } else {
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    k = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = cr - i15;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, k, (int)sizeof(double));
    b_loop_ub = 12 * (cr - i15);
    for (k = 0; k < b_loop_ub; k++) {
      P_xs_apr->data[k] = 0.0;
    }

    if (cr - i15 == 0) {
    } else {
      b_loop_ub = 12 * ((cr - i15) - 1);
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= b_loop_ub; cr += 12) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (b_P_apo->data[ib % loop_ub + b_P_apo->size[0] * (i15 + ib /
               loop_ub)] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += b_P_apo->data[ib % loop_ub + b_P_apo->size[0]
                * (i15 + ib / loop_ub)] * Phi[ia - 1];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  //  covariance between current state and trails
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    b_loop_ub = 0;
  } else {
    b_loop_ub = (int)c_numStates;
  }

  emxInit_int32_T(&r4, 1);
  i15 = r4->size[0];
  r4->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r4, i15, (int)sizeof(int));
  for (i15 = 0; i15 < loop_ub; i15++) {
    r4->data[i15] = i15;
  }

  emxInit_int32_T(&r5, 1);
  i15 = r5->size[0];
  r5->size[0] = b_loop_ub;
  emxEnsureCapacity((emxArray__common *)r5, i15, (int)sizeof(int));
  for (i15 = 0; i15 < b_loop_ub; i15++) {
    r5->data[i15] = i15;
  }

  for (i15 = 0; i15 < 12; i15++) {
    for (cr = 0; cr < 12; cr++) {
      c_G1[cr + 12 * i15] = (Fc[cr + 12 * i15] + Fc[i15 + 12 * cr]) / 2.0;
    }
  }

  k = r4->size[0];
  b_loop_ub = r5->size[0];
  for (i15 = 0; i15 < b_loop_ub; i15++) {
    for (cr = 0; cr < k; cr++) {
      b_P_apo->data[r4->data[cr] + b_P_apo->size[0] * r5->data[i15]] = c_G1[cr +
        k * i15];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > b_P_apo->size[1]) {
    i15 = 0;
    cr = 0;
  } else {
    i15 = (int)(c_numStates + 1.0) - 1;
    cr = b_P_apo->size[1];
  }

  k = r4->size[0];
  r4->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r4, k, (int)sizeof(int));
  for (k = 0; k < loop_ub; k++) {
    r4->data[k] = k;
  }

  k = r5->size[0];
  r5->size[0] = cr - i15;
  emxEnsureCapacity((emxArray__common *)r5, k, (int)sizeof(int));
  loop_ub = cr - i15;
  for (cr = 0; cr < loop_ub; cr++) {
    r5->data[cr] = i15 + cr;
  }

  k = r4->size[0];
  b_loop_ub = r5->size[0];
  for (i15 = 0; i15 < b_loop_ub; i15++) {
    for (cr = 0; cr < k; cr++) {
      b_P_apo->data[r4->data[cr] + b_P_apo->size[0] * r5->data[i15]] =
        P_xs_apr->data[cr + k * i15];
    }
  }

  if (c_numStates + 1.0 > b_P_apo->size[0]) {
    i15 = 0;
    cr = 0;
  } else {
    i15 = (int)(c_numStates + 1.0) - 1;
    cr = b_P_apo->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  k = r4->size[0];
  r4->size[0] = cr - i15;
  emxEnsureCapacity((emxArray__common *)r4, k, (int)sizeof(int));
  b_loop_ub = cr - i15;
  for (cr = 0; cr < b_loop_ub; cr++) {
    r4->data[cr] = i15 + cr;
  }

  i15 = r5->size[0];
  r5->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r5, i15, (int)sizeof(int));
  for (i15 = 0; i15 < loop_ub; i15++) {
    r5->data[i15] = i15;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i15 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i15, (int)sizeof(double));
  for (i15 = 0; i15 < 12; i15++) {
    loop_ub = P_xs_apr->size[1];
    for (cr = 0; cr < loop_ub; cr++) {
      b_P_xs_apr->data[cr + b_P_xs_apr->size[0] * i15] = P_xs_apr->data[i15 +
        P_xs_apr->size[0] * cr];
    }
  }

  emxFree_real_T(&P_xs_apr);
  k = r4->size[0];
  b_loop_ub = r5->size[0];
  for (i15 = 0; i15 < b_loop_ub; i15++) {
    for (cr = 0; cr < k; cr++) {
      b_P_apo->data[r4->data[cr] + b_P_apo->size[0] * r5->data[i15]] =
        b_P_xs_apr->data[cr + k * i15];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r5);
  emxFree_int32_T(&r4);
  if (normalGravity) {
    for (k = 0; k < 3; k++) {
      grav[k] = dv15[k];
    }
  } else {
    for (k = 0; k < 3; k++) {
      grav[k] = dv16[k];
    }
  }

  for (i15 = 0; i15 < 3; i15++) {
    b_x[i15] = x->data[i15] + x->data[7 + i15] * dt;
  }

  for (i15 = 0; i15 < 3; i15++) {
    x->data[i15] = b_x[i15];
  }

  // +1/2*dt^2*(R_cw'*a-grav);             % position
  for (k = 0; k < 3; k++) {
    b_w[k] = w[k] * dt;
  }

  quatPlusThetaJ(b_w, dq);
  c = x->data[6];
  c_x = x->data[5];
  d_x = x->data[4];
  e_x = x->data[3];
  f_x = x->data[5];
  g_x = x->data[6];
  h_x = x->data[3];
  i_x = x->data[4];
  j_x = x->data[4];
  k_x = x->data[3];
  l_x = x->data[6];
  m_x = x->data[5];
  n_x = x->data[3];
  o_x = x->data[4];
  p_x = x->data[5];
  q_x = x->data[6];
  r_x[0] = c;
  r_x[4] = -c_x;
  r_x[8] = d_x;
  r_x[12] = e_x;
  r_x[1] = f_x;
  r_x[5] = g_x;
  r_x[9] = -h_x;
  r_x[13] = i_x;
  r_x[2] = -j_x;
  r_x[6] = k_x;
  r_x[10] = l_x;
  r_x[14] = m_x;
  r_x[3] = -n_x;
  r_x[7] = -o_x;
  r_x[11] = -p_x;
  r_x[15] = q_x;
  b_dq[0] = dq[0];
  b_dq[1] = dq[1];
  b_dq[2] = dq[2];
  b_dq[3] = dq[3];
  for (i15 = 0; i15 < 4; i15++) {
    s_x[i15] = 0.0;
    for (cr = 0; cr < 4; cr++) {
      s_x[i15] += r_x[i15 + (cr << 2)] * b_dq[cr];
    }
  }

  for (i15 = 0; i15 < 4; i15++) {
    x->data[3 + i15] = s_x[i15];
  }

  c = b_norm(*(double (*)[4])&x->data[3]);
  for (i15 = 0; i15 < 4; i15++) {
    s_x[i15] = x->data[3 + i15] / c;
  }

  for (i15 = 0; i15 < 4; i15++) {
    x->data[3 + i15] = s_x[i15];
  }

  if (UseAccToPredict) {
    for (i15 = 0; i15 < 3; i15++) {
      c = 0.0;
      for (cr = 0; cr < 3; cr++) {
        c += R_cw[cr + 3 * i15] * IMU_measurements[3 + cr];
      }

      b_w[i15] = c - grav[i15];
    }

    for (i15 = 0; i15 < 3; i15++) {
      b_x[i15] = x->data[7 + i15] + b_w[i15] * dt;
    }

    for (i15 = 0; i15 < 3; i15++) {
      x->data[7 + i15] = b_x[i15];
    }

    //  velocity
  }

  //  disp('R*a-g')
  //  disp(R_cw'*a - grav)
}

//
// Arguments    : emxArray_real_T *P_apr
//                emxArray_real_T *b_xt
//                const double cameraparams[4]
//                double updateVect[32]
//                const double z_all[96]
//                const double imNoise[3]
//                emxArray_real_T *b_P_apo
//                double map_out[96]
// Return Type  : void
//
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams[4], double updateVect[32], const double z_all[96], const
  double imNoise[3], emxArray_real_T *b_P_apo, double map_out[96])
{
  double f;
  double Cx;
  double Cy;
  double b;
  int i9;
  int ib;
  int idx;
  signed char ii_data[32];
  int ii;
  boolean_T exitg1;
  boolean_T guard1 = false;
  int ar;
  signed char indMeas_data[32];
  emxArray_real_T *H_xc;
  double h_u_data[96];
  double R_cw[9];
  int z_size_idx_0;
  double z_data[96];
  int k;
  emxArray_int32_T *r2;
  int i10;
  double z_curr_data[96];
  boolean_T bv0[3];
  double c_xt[3];
  double h_ci[3];
  int br;
  double h_ui[3];
  double h_u_To_h_c[9];
  int tmp_data[96];
  int b_tmp_data[96];
  double b_R_cw[36];
  double b_h_u_To_h_c[36];
  emxArray_real_T *r3;
  emxArray_real_T *H;
  emxArray_real_T *R;
  emxArray_real_T *y;
  double dv7[3];
  double dv8[9];
  unsigned int unnamed_idx_0;
  unsigned int unnamed_idx_1;
  int m;
  int ic;
  int ia;
  emxArray_real_T *b_b;
  emxArray_real_T *C;
  emxArray_real_T *b_C;
  emxArray_real_T *K;
  emxArray_real_T *x_apo;
  emxArray_real_T *b_y;
  double dv9[4];
  double dv10[4];
  double d_xt[6];
  double d0;
  double d1;
  double d2;
  double e_xt[4];
  double dv11[4];
  double newestTrail;
  double b_h_ui;
  static const signed char iv0[3] = { 0, 1, 2 };

  boolean_T b_guard1 = false;
  boolean_T b0;
  boolean_T createNewTrail;
  double f_xt[7];
  int iv1[6];
  double A;
  double c_y;
  double fp[3];
  double disparities[32];
  boolean_T bv1[32];
  double b_disparities;
  signed char c_tmp_data[32];
  int iidx[32];
  int d_tmp_data[32];

  // % Iterative Camera Pose optimization (EKF)
  //  camera parameters
  f = cameraparams[0];
  Cx = cameraparams[1];
  Cy = cameraparams[2];
  b = cameraparams[3];
  if (!map_not_empty) {
    //  the index of the oldest trailing state in xt
    map_not_empty = true;

    //  the map holding the feature positions in XYZ world coordinates
    //  stores whether a feature is in the map and can be used to update the state 
    //  the age of each feature to be initialized (i.e. the number of trailing poses in which it was observed) 
    i9 = measurementHistory->size[0] * measurementHistory->size[1];
    measurementHistory->size[0] = 96;
    measurementHistory->size[1] = (int)trailSize;
    emxEnsureCapacity((emxArray__common *)measurementHistory, i9, (int)sizeof
                      (double));
    ib = 96 * (int)trailSize;
    for (i9 = 0; i9 < ib; i9++) {
      measurementHistory->data[i9] = 0.0;
    }

    //  the old measurements for each trailing position
    // residual_plot = figure;
    msckfUPD = 0.0;
  }

  // % ================================================================================================= 
  idx = 0;
  ii = 1;
  exitg1 = false;
  while ((!exitg1) && (ii < 33)) {
    guard1 = false;
    if ((updateVect[ii - 1] == 1.0) && (pointInMap[ii - 1] == 1.0)) {
      idx++;
      ii_data[idx - 1] = (signed char)ii;
      if (idx >= 32) {
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
    ib = 0;
  } else {
    ib = idx;
  }

  if (1 > idx) {
    ar = 0;
  } else {
    ar = idx;
  }

  for (i9 = 0; i9 < ib; i9++) {
    indMeas_data[i9] = ii_data[i9];
  }

  emxInit_real_T(&H_xc, 2);
  i9 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = ib * 3;
  emxEnsureCapacity((emxArray__common *)H_xc, i9, (int)sizeof(double));
  i9 = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)numStates;
  emxEnsureCapacity((emxArray__common *)H_xc, i9, (int)sizeof(double));
  ib = ar * 3 * (int)numStates;
  for (i9 = 0; i9 < ib; i9++) {
    H_xc->data[i9] = 0.0;
  }

  //  derivatives wrt camera state
  ib = 3 * ar;
  for (i9 = 0; i9 < ib; i9++) {
    h_u_data[i9] = 0.0;
  }

  //  predicted measurements of current frame
  RotFromQuatJ(*(double (*)[4])&b_xt->data[3], R_cw);
  z_size_idx_0 = 3 * ar;
  ib = 3 * ar;
  for (i9 = 0; i9 < ib; i9++) {
    z_data[i9] = 0.0;
  }

  k = 1;
  emxInit_int32_T(&r2, 1);
  while ((k <= ar) && (!(k > ar))) {
    i9 = indMeas_data[k - 1] * 3 - 2;
    i10 = indMeas_data[k - 1] * 3;
    if (i9 > i10) {
      i9 = 0;
      i10 = 0;
    } else {
      i9--;
    }

    ib = i10 - i9;
    for (i10 = 0; i10 < ib; i10++) {
      z_curr_data[i10] = z_all[i9 + i10];
    }

    for (i10 = 0; i10 < 3; i10++) {
      bv0[i10] = rtIsNaN(map[i10 + 3 * (indMeas_data[k - 1] - 1)]);
    }

    if (!any(bv0)) {
      for (i10 = 0; i10 < 3; i10++) {
        c_xt[i10] = map[i10 + 3 * (indMeas_data[k - 1] - 1)] - b_xt->data[i10];
      }

      for (i10 = 0; i10 < 3; i10++) {
        h_ci[i10] = 0.0;
        for (br = 0; br < 3; br++) {
          h_ci[i10] += R_cw[i10 + 3 * br] * c_xt[br];
        }
      }

      h_ui[0] = Cx + f * (h_ci[0] / h_ci[2]);
      h_ui[1] = Cy + f * (h_ci[1] / h_ci[2]);
      h_ui[2] = f * b / h_ci[2];
      if (z_all[i9 + 2] < -500.0) {
        // invalid disparity
        h_u_To_h_c[0] = f / h_ci[2];
        h_u_To_h_c[3] = 0.0;
        h_u_To_h_c[6] = -(h_ci[0] * f) / (h_ci[2] * h_ci[2]);
        h_u_To_h_c[1] = 0.0;
        h_u_To_h_c[4] = f / h_ci[2];
        h_u_To_h_c[7] = -(h_ci[1] * f) / (h_ci[2] * h_ci[2]);
        for (i9 = 0; i9 < 3; i9++) {
          h_u_To_h_c[2 + 3 * i9] = 0.0;
        }

        //  make sure there are no NaNs in the residual
        z_curr_data[2] = h_ui[2];

        //  residual will be zero
      } else {
        // valid disparity
        h_u_To_h_c[0] = f / h_ci[2];
        h_u_To_h_c[3] = 0.0;
        h_u_To_h_c[6] = -(h_ci[0] * f) / (h_ci[2] * h_ci[2]);
        h_u_To_h_c[1] = 0.0;
        h_u_To_h_c[4] = f / h_ci[2];
        h_u_To_h_c[7] = -(h_ci[1] * f) / (h_ci[2] * h_ci[2]);
        h_u_To_h_c[2] = 0.0;
        h_u_To_h_c[5] = 0.0;
        h_u_To_h_c[8] = -b * f / (h_ci[2] * h_ci[2]);
      }

      i9 = k * 3 - 2;
      i10 = k * 3;
      if (i9 > i10) {
        i9 = 0;
        i10 = 0;
      } else {
        i9--;
      }

      ib = i10 - i9;
      for (br = 0; br < ib; br++) {
        tmp_data[br] = i9 + br;
      }

      ib = i10 - i9;
      for (i9 = 0; i9 < ib; i9++) {
        z_data[tmp_data[i9]] = z_curr_data[i9];
      }

      // %%%%%%%%%%%%%%%%%%%%%%%
      //         %% computation of H(x)%%
      // %%%%%%%%%%%%%%%%%%%%%%%
      //  nonlinear predicted measurement
      i9 = 3 * k - 2;
      i10 = 3 * k;
      if (i9 > i10) {
        i9 = 0;
        i10 = 0;
      } else {
        i9--;
      }

      ib = i10 - i9;
      for (br = 0; br < ib; br++) {
        tmp_data[br] = i9 + br;
      }

      ib = i10 - i9;
      for (i9 = 0; i9 < ib; i9++) {
        h_u_data[tmp_data[i9]] = h_ui[i9];
      }

      //  stack the matrices for all feature measurements
      i9 = 3 * k - 2;
      i10 = 3 * k;
      if (i9 > i10) {
        i9 = 0;
        i10 = 0;
      } else {
        i9--;
      }

      ib = i10 - i9;
      for (br = 0; br < ib; br++) {
        b_tmp_data[br] = i9 + br;
      }

      ib = H_xc->size[1];
      br = r2->size[0];
      r2->size[0] = ib;
      emxEnsureCapacity((emxArray__common *)r2, br, (int)sizeof(int));
      for (br = 0; br < ib; br++) {
        r2->data[br] = br;
      }

      for (br = 0; br < 3; br++) {
        for (ii = 0; ii < 3; ii++) {
          b_R_cw[ii + 3 * br] = -R_cw[ii + 3 * br];
        }
      }

      b_R_cw[9] = 0.0;
      b_R_cw[12] = -h_ci[2];
      b_R_cw[15] = h_ci[1];
      b_R_cw[10] = h_ci[2];
      b_R_cw[13] = 0.0;
      b_R_cw[16] = -h_ci[0];
      b_R_cw[11] = -h_ci[1];
      b_R_cw[14] = h_ci[0];
      b_R_cw[17] = 0.0;
      for (br = 0; br < 6; br++) {
        for (ii = 0; ii < 3; ii++) {
          b_R_cw[ii + 3 * (br + 6)] = 0.0;
        }
      }

      for (br = 0; br < 3; br++) {
        for (ii = 0; ii < 12; ii++) {
          b_h_u_To_h_c[br + 3 * ii] = 0.0;
          for (idx = 0; idx < 3; idx++) {
            b_h_u_To_h_c[br + 3 * ii] += h_u_To_h_c[br + 3 * idx] * b_R_cw[idx +
              3 * ii];
          }
        }
      }

      ii = i10 - i9;
      idx = r2->size[0];
      for (i9 = 0; i9 < idx; i9++) {
        for (i10 = 0; i10 < ii; i10++) {
          H_xc->data[b_tmp_data[i10] + H_xc->size[0] * r2->data[i9]] =
            b_h_u_To_h_c[i10 + ii * i9];
        }
      }
    }

    k++;
  }

  emxFree_int32_T(&r2);

  // ===========ACC=====================================
  if (ar > 0) {
    //  residual
    for (i9 = 0; i9 < z_size_idx_0; i9++) {
      z_data[i9] -= h_u_data[i9];
    }

    emxInit_real_T(&r3, 2);
    ii = H_xc->size[0];
    i9 = r3->size[0] * r3->size[1];
    r3->size[0] = ii;
    emxEnsureCapacity((emxArray__common *)r3, i9, (int)sizeof(double));
    i9 = r3->size[0] * r3->size[1];
    r3->size[1] = (int)(trailSize * 6.0);
    emxEnsureCapacity((emxArray__common *)r3, i9, (int)sizeof(double));
    ib = H_xc->size[0] * (int)(trailSize * 6.0);
    for (i9 = 0; i9 < ib; i9++) {
      r3->data[i9] = 0.0;
    }

    emxInit_real_T(&H, 2);
    i9 = H->size[0] * H->size[1];
    H->size[0] = H_xc->size[0];
    H->size[1] = H_xc->size[1] + r3->size[1];
    emxEnsureCapacity((emxArray__common *)H, i9, (int)sizeof(double));
    ib = H_xc->size[1];
    for (i9 = 0; i9 < ib; i9++) {
      idx = H_xc->size[0];
      for (i10 = 0; i10 < idx; i10++) {
        H->data[i10 + H->size[0] * i9] = H_xc->data[i10 + H_xc->size[0] * i9];
      }
    }

    ib = r3->size[1];
    for (i9 = 0; i9 < ib; i9++) {
      idx = r3->size[0];
      for (i10 = 0; i10 < idx; i10++) {
        H->data[i10 + H->size[0] * (i9 + H_xc->size[1])] = r3->data[i10 +
          r3->size[0] * i9];
      }
    }

    emxFree_real_T(&r3);
    emxInit_real_T(&R, 2);
    emxInit_real_T(&y, 2);
    eye((double)ar, y);
    power(imNoise, dv7);
    diag(dv7, dv8);
    kron(y->data, y->size, dv8, R);
    msckfUPD = 0.0;

    //  simple residual outlier rejection
    //      for i = 1:length(indMeas)
    //          k = indMeas(i);
    //          %         plot((k-1)*3 + (1:2), r((i-1)*3 + (1:2)), '.-')
    //          %         text((k-1)*3 + 1, 0, num2str(k))
    //
    //          if abs(r((i-1)*3 + 1) - mean_residual_ux) > rejection_threshold_ux 
    //              %disp(['Reject feature ', num2str(k), ' due to ux residual']) 
    //              r((i-1)*3 + (1:3)) = 0;
    //              updateVect_out(k) = 0;
    //          end
    //
    //          if abs(r((i-1)*3 + 2) - mean_residual_uy) > rejection_threshold_uy 
    //              %disp(['Reject feature ', num2str(k), ' due to uy residual']) 
    //              r((i-1)*3 + (1:3)) = 0;
    //              updateVect_out(k) = 0;
    //          end
    //      end
    if ((H->size[1] == 1) || (P_apr->size[0] == 1)) {
      i9 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = P_apr->size[1];
      emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
      ib = H->size[0];
      for (i9 = 0; i9 < ib; i9++) {
        idx = P_apr->size[1];
        for (i10 = 0; i10 < idx; i10++) {
          y->data[i9 + y->size[0] * i10] = 0.0;
          ii = H->size[1];
          for (br = 0; br < ii; br++) {
            y->data[i9 + y->size[0] * i10] += H->data[i9 + H->size[0] * br] *
              P_apr->data[br + P_apr->size[0] * i10];
          }
        }
      }
    } else {
      k = H->size[1];
      unnamed_idx_0 = (unsigned int)H->size[0];
      unnamed_idx_1 = (unsigned int)P_apr->size[1];
      m = H->size[0];
      i9 = y->size[0] * y->size[1];
      y->size[0] = (int)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
      i9 = y->size[0] * y->size[1];
      y->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
      ib = (int)unnamed_idx_0 * (int)unnamed_idx_1;
      for (i9 = 0; i9 < ib; i9++) {
        y->data[i9] = 0.0;
      }

      if ((H->size[0] == 0) || (P_apr->size[1] == 0)) {
      } else {
        ii = H->size[0] * (P_apr->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i9 = idx + m;
          for (ic = idx; ic + 1 <= i9; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i9 = br + k;
          for (ib = br; ib + 1 <= i9; ib++) {
            if (P_apr->data[ib] != 0.0) {
              ia = ar;
              i10 = idx + m;
              for (ic = idx; ic + 1 <= i10; ic++) {
                ia++;
                y->data[ic] += P_apr->data[ib] * H->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    emxInit_real_T(&b_b, 2);
    i9 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = H->size[1];
    b_b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i9, (int)sizeof(double));
    ib = H->size[0];
    for (i9 = 0; i9 < ib; i9++) {
      idx = H->size[1];
      for (i10 = 0; i10 < idx; i10++) {
        b_b->data[i10 + b_b->size[0] * i9] = H->data[i9 + H->size[0] * i10];
      }
    }

    emxInit_real_T(&C, 2);
    if ((y->size[1] == 1) || (b_b->size[0] == 1)) {
      i9 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i9, (int)sizeof(double));
      ib = y->size[0];
      for (i9 = 0; i9 < ib; i9++) {
        idx = b_b->size[1];
        for (i10 = 0; i10 < idx; i10++) {
          C->data[i9 + C->size[0] * i10] = 0.0;
          ii = y->size[1];
          for (br = 0; br < ii; br++) {
            C->data[i9 + C->size[0] * i10] += y->data[i9 + y->size[0] * br] *
              b_b->data[br + b_b->size[0] * i10];
          }
        }
      }
    } else {
      k = y->size[1];
      unnamed_idx_0 = (unsigned int)y->size[0];
      unnamed_idx_1 = (unsigned int)b_b->size[1];
      m = y->size[0];
      i9 = C->size[0] * C->size[1];
      C->size[0] = (int)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)C, i9, (int)sizeof(double));
      i9 = C->size[0] * C->size[1];
      C->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)C, i9, (int)sizeof(double));
      ib = (int)unnamed_idx_0 * (int)unnamed_idx_1;
      for (i9 = 0; i9 < ib; i9++) {
        C->data[i9] = 0.0;
      }

      if ((y->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i9 = idx + m;
          for (ic = idx; ic + 1 <= i9; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i9 = br + k;
          for (ib = br; ib + 1 <= i9; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i10 = idx + m;
              for (ic = idx; ic + 1 <= i10; ic++) {
                ia++;
                C->data[ic] += b_b->data[ib] * y->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    i9 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = H->size[1];
    b_b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i9, (int)sizeof(double));
    ib = H->size[0];
    for (i9 = 0; i9 < ib; i9++) {
      idx = H->size[1];
      for (i10 = 0; i10 < idx; i10++) {
        b_b->data[i10 + b_b->size[0] * i9] = H->data[i9 + H->size[0] * i10];
      }
    }

    if ((P_apr->size[1] == 1) || (b_b->size[0] == 1)) {
      i9 = y->size[0] * y->size[1];
      y->size[0] = P_apr->size[0];
      y->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
      ib = P_apr->size[0];
      for (i9 = 0; i9 < ib; i9++) {
        idx = b_b->size[1];
        for (i10 = 0; i10 < idx; i10++) {
          y->data[i9 + y->size[0] * i10] = 0.0;
          ii = P_apr->size[1];
          for (br = 0; br < ii; br++) {
            y->data[i9 + y->size[0] * i10] += P_apr->data[i9 + P_apr->size[0] *
              br] * b_b->data[br + b_b->size[0] * i10];
          }
        }
      }
    } else {
      k = P_apr->size[1];
      unnamed_idx_0 = (unsigned int)P_apr->size[0];
      unnamed_idx_1 = (unsigned int)b_b->size[1];
      m = P_apr->size[0];
      i9 = y->size[0] * y->size[1];
      y->size[0] = (int)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
      i9 = y->size[0] * y->size[1];
      y->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
      ib = (int)unnamed_idx_0 * (int)unnamed_idx_1;
      for (i9 = 0; i9 < ib; i9++) {
        y->data[i9] = 0.0;
      }

      if ((P_apr->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        ii = P_apr->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i9 = idx + m;
          for (ic = idx; ic + 1 <= i9; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i9 = br + k;
          for (ib = br; ib + 1 <= i9; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i10 = idx + m;
              for (ic = idx; ic + 1 <= i10; ic++) {
                ia++;
                y->data[ic] += b_b->data[ib] * P_apr->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    emxInit_real_T(&b_C, 2);
    i9 = b_C->size[0] * b_C->size[1];
    b_C->size[0] = C->size[0];
    b_C->size[1] = C->size[1];
    emxEnsureCapacity((emxArray__common *)b_C, i9, (int)sizeof(double));
    ib = C->size[0] * C->size[1];
    for (i9 = 0; i9 < ib; i9++) {
      b_C->data[i9] = C->data[i9] + R->data[i9];
    }

    emxFree_real_T(&R);
    emxInit_real_T(&K, 2);
    mrdivide(y, b_C, K);
    emxFree_real_T(&b_C);
    b_emxInit_real_T(&x_apo, 1);
    if (z_size_idx_0 == 1) {
      i9 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i9, (int)sizeof(double));
      ib = K->size[0];
      for (i9 = 0; i9 < ib; i9++) {
        x_apo->data[i9] = 0.0;
        idx = K->size[1];
        for (i10 = 0; i10 < idx; i10++) {
          x_apo->data[i9] += K->data[i9 + K->size[0] * i10] * z_data[i10];
        }
      }
    } else {
      k = K->size[1];
      unnamed_idx_0 = (unsigned int)K->size[0];
      m = K->size[0];
      i9 = x_apo->size[0];
      x_apo->size[0] = (int)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)x_apo, i9, (int)sizeof(double));
      ib = (int)unnamed_idx_0;
      for (i9 = 0; i9 < ib; i9++) {
        x_apo->data[i9] = 0.0;
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
          i9 = br + k;
          for (ib = br; ib + 1 <= i9; ib++) {
            if (z_data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                x_apo->data[ic] += z_data[ib] * K->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx = m;
        }
      }
    }

    eye(numStates + trailSize * 6.0, y);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i9 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i9, (int)sizeof(double));
      ib = K->size[0];
      for (i9 = 0; i9 < ib; i9++) {
        idx = H->size[1];
        for (i10 = 0; i10 < idx; i10++) {
          C->data[i9 + C->size[0] * i10] = 0.0;
          ii = K->size[1];
          for (br = 0; br < ii; br++) {
            C->data[i9 + C->size[0] * i10] += K->data[i9 + K->size[0] * br] *
              H->data[br + H->size[0] * i10];
          }
        }
      }
    } else {
      k = K->size[1];
      unnamed_idx_0 = (unsigned int)K->size[0];
      unnamed_idx_1 = (unsigned int)H->size[1];
      m = K->size[0];
      i9 = C->size[0] * C->size[1];
      C->size[0] = (int)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)C, i9, (int)sizeof(double));
      i9 = C->size[0] * C->size[1];
      C->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)C, i9, (int)sizeof(double));
      ib = (int)unnamed_idx_0 * (int)unnamed_idx_1;
      for (i9 = 0; i9 < ib; i9++) {
        C->data[i9] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i9 = idx + m;
          for (ic = idx; ic + 1 <= i9; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i9 = br + k;
          for (ib = br; ib + 1 <= i9; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i10 = idx + m;
              for (ic = idx; ic + 1 <= i10; ic++) {
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

    emxFree_real_T(&K);
    emxFree_real_T(&H);
    i9 = y->size[0] * y->size[1];
    emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
    ii = y->size[0];
    idx = y->size[1];
    ib = ii * idx;
    for (i9 = 0; i9 < ib; i9++) {
      y->data[i9] -= C->data[i9];
    }

    emxFree_real_T(&C);
    i9 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = P_apr->size[0];
    b_b->size[1] = P_apr->size[1];
    emxEnsureCapacity((emxArray__common *)b_b, i9, (int)sizeof(double));
    ib = P_apr->size[0] * P_apr->size[1];
    for (i9 = 0; i9 < ib; i9++) {
      b_b->data[i9] = P_apr->data[i9];
    }

    emxInit_real_T(&b_y, 2);
    if ((y->size[1] == 1) || (P_apr->size[0] == 1)) {
      i9 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = y->size[0];
      b_y->size[1] = P_apr->size[1];
      emxEnsureCapacity((emxArray__common *)b_y, i9, (int)sizeof(double));
      ib = y->size[0];
      for (i9 = 0; i9 < ib; i9++) {
        idx = P_apr->size[1];
        for (i10 = 0; i10 < idx; i10++) {
          b_y->data[i9 + b_y->size[0] * i10] = 0.0;
          ii = y->size[1];
          for (br = 0; br < ii; br++) {
            b_y->data[i9 + b_y->size[0] * i10] += y->data[i9 + y->size[0] * br] *
              P_apr->data[br + P_apr->size[0] * i10];
          }
        }
      }

      i9 = P_apr->size[0] * P_apr->size[1];
      P_apr->size[0] = b_y->size[0];
      P_apr->size[1] = b_y->size[1];
      emxEnsureCapacity((emxArray__common *)P_apr, i9, (int)sizeof(double));
      ib = b_y->size[1];
      for (i9 = 0; i9 < ib; i9++) {
        idx = b_y->size[0];
        for (i10 = 0; i10 < idx; i10++) {
          P_apr->data[i10 + P_apr->size[0] * i9] = b_y->data[i10 + b_y->size[0] *
            i9];
        }
      }
    } else {
      k = y->size[1];
      unnamed_idx_0 = (unsigned int)y->size[0];
      unnamed_idx_1 = (unsigned int)P_apr->size[1];
      m = y->size[0];
      i9 = P_apr->size[0] * P_apr->size[1];
      P_apr->size[0] = (int)unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)P_apr, i9, (int)sizeof(double));
      i9 = P_apr->size[0] * P_apr->size[1];
      P_apr->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)P_apr, i9, (int)sizeof(double));
      ib = (int)unnamed_idx_0 * (int)unnamed_idx_1;
      for (i9 = 0; i9 < ib; i9++) {
        P_apr->data[i9] = 0.0;
      }

      if ((y->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i9 = idx + m;
          for (ic = idx; ic + 1 <= i9; ic++) {
            P_apr->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i9 = br + k;
          for (ib = br; ib + 1 <= i9; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i10 = idx + m;
              for (ic = idx; ic + 1 <= i10; ic++) {
                ia++;
                P_apr->data[ic] += b_b->data[ib] * y->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    emxFree_real_T(&b_y);
    emxFree_real_T(&b_b);
    emxFree_real_T(&y);

    //  update the current state
    for (i9 = 0; i9 < 3; i9++) {
      c_xt[i9] = b_xt->data[i9] + x_apo->data[i9];
    }

    for (i9 = 0; i9 < 3; i9++) {
      b_xt->data[i9] = c_xt[i9];
    }

    for (i9 = 0; i9 < 3; i9++) {
      c_xt[i9] = x_apo->data[3 + i9];
    }

    quatPlusThetaJ(c_xt, dv9);
    quatmultJ(dv9, *(double (*)[4])&b_xt->data[3], dv10);
    for (i9 = 0; i9 < 4; i9++) {
      b_xt->data[3 + i9] = dv10[i9];
    }

    for (i9 = 0; i9 < 6; i9++) {
      d_xt[i9] = b_xt->data[7 + i9] + x_apo->data[6 + i9];
    }

    for (i9 = 0; i9 < 6; i9++) {
      b_xt->data[7 + i9] = d_xt[i9];
    }

    //  update the trailing poses
    for (ii = 0; ii < (int)trailSize; ii++) {
      d0 = numStatesxt + ((1.0 + (double)ii) - 1.0) * 7.0;
      d1 = numStatesxt + ((1.0 + (double)ii) - 1.0) * 7.0;
      d2 = numStates + ((1.0 + (double)ii) - 1.0) * 6.0;
      for (i9 = 0; i9 < 3; i9++) {
        c_xt[i9] = b_xt->data[(int)(d1 + (1.0 + (double)i9)) - 1] + x_apo->data
          [(int)(d2 + (1.0 + (double)i9)) - 1];
      }

      for (i9 = 0; i9 < 3; i9++) {
        b_xt->data[(int)(d0 + (1.0 + (double)i9)) - 1] = c_xt[i9];
      }

      d0 = numStates + ((1.0 + (double)ii) - 1.0) * 6.0;
      for (i9 = 0; i9 < 3; i9++) {
        c_xt[i9] = x_apo->data[(int)(d0 + (4.0 + (double)i9)) - 1];
      }

      d0 = numStatesxt + ((1.0 + (double)ii) - 1.0) * 7.0;
      for (i9 = 0; i9 < 4; i9++) {
        e_xt[i9] = b_xt->data[(int)(d0 + (4.0 + (double)i9)) - 1];
      }

      quatPlusThetaJ(c_xt, dv11);
      quatmultJ(dv11, e_xt, dv10);
      d0 = numStatesxt + ((1.0 + (double)ii) - 1.0) * 7.0;
      for (i9 = 0; i9 < 4; i9++) {
        b_xt->data[(int)(d0 + (4.0 + (double)i9)) - 1] = dv10[i9];
      }
    }

    emxFree_real_T(&x_apo);

    //  check if the camera has moved enough since the last trail pose to
    //  create a new trail pose
    newestTrail = oldestTrail - 1.0;
    if (oldestTrail - 1.0 < 1.0) {
      newestTrail = trailSize;
    }

    d0 = numStatesxt + (newestTrail - 1.0) * 7.0;
    for (i9 = 0; i9 < 3; i9++) {
      b_h_ui = b_xt->data[(int)(d0 + (1.0 + (double)i9)) - 1];
      c_xt[i9] = b_xt->data[iv0[i9]] - b_h_ui;
    }

    b_guard1 = false;
    if (norm(c_xt) > baselineThresold) {
      b_guard1 = true;
    } else {
      br = 0;
      for (k = 0; k < 32; k++) {
        if (updateVect[k] == 1.0) {
          br++;
        }
      }

      if (br < minFeatureThreshold) {
        b_guard1 = true;
      } else {
        b0 = false;
      }
    }

    if (b_guard1) {
      b0 = true;
    }

    createNewTrail = b0;
  } else {
    createNewTrail = true;

    //      P_apo = P_apr;
  }

  emxFree_real_T(&H_xc);
  if ((trailSize != 0.0) && createNewTrail) {
    //  disp('Creating new trail pose')
    //  move the state down by one
    d0 = numStatesxt + (oldestTrail - 1.0) * 7.0;
    for (i9 = 0; i9 < 7; i9++) {
      f_xt[i9] = b_xt->data[i9];
    }

    for (i9 = 0; i9 < 7; i9++) {
      b_xt->data[(int)(d0 + (1.0 + (double)i9)) - 1] = f_xt[i9];
    }

    //  update the covariance matrix
    i9 = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = P_apr->size[0];
    b_P_apo->size[1] = P_apr->size[1];
    emxEnsureCapacity((emxArray__common *)b_P_apo, i9, (int)sizeof(double));
    ib = P_apr->size[0] * P_apr->size[1];
    for (i9 = 0; i9 < ib; i9++) {
      b_P_apo->data[i9] = P_apr->data[i9];
    }

    ib = P_apr->size[1];
    d0 = numStates + (oldestTrail - 1.0) * 6.0;
    for (i9 = 0; i9 < ib; i9++) {
      for (i10 = 0; i10 < 6; i10++) {
        b_P_apo->data[((int)(d0 + (1.0 + (double)i10)) + b_P_apo->size[0] * i9)
          - 1] = 0.0;
      }
    }

    ib = b_P_apo->size[0];
    d0 = numStates + (oldestTrail - 1.0) * 6.0;
    for (i9 = 0; i9 < 6; i9++) {
      for (i10 = 0; i10 < ib; i10++) {
        b_P_apo->data[i10 + b_P_apo->size[0] * ((int)(d0 + (1.0 + (double)i9)) -
          1)] = 0.0;
      }
    }

    d0 = numStates + (oldestTrail - 1.0) * 6.0;
    d1 = numStates + (oldestTrail - 1.0) * 6.0;
    for (i9 = 0; i9 < 6; i9++) {
      for (i10 = 0; i10 < 6; i10++) {
        b_P_apo->data[((int)(d0 + (1.0 + (double)i10)) + b_P_apo->size[0] *
                       ((int)(d1 + (1.0 + (double)i9)) - 1)) - 1] = P_apr->
          data[i10 + P_apr->size[0] * i9];
      }
    }

    if (1.0 > numStates) {
      ib = -1;
    } else {
      ib = (int)numStates - 1;
    }

    d0 = numStates + (oldestTrail - 1.0) * 6.0;
    for (i9 = 0; i9 < 6; i9++) {
      iv1[i9] = (int)(d0 + (1.0 + (double)i9)) - 1;
    }

    for (i9 = 0; i9 < 6; i9++) {
      for (i10 = 0; i10 <= ib; i10++) {
        b_P_apo->data[i10 + b_P_apo->size[0] * iv1[i9]] = P_apr->data[i10 +
          P_apr->size[0] * i9];
      }
    }

    if (1.0 > numStates) {
      ib = -1;
    } else {
      ib = (int)numStates - 1;
    }

    d0 = numStates + (oldestTrail - 1.0) * 6.0;
    for (i9 = 0; i9 < 6; i9++) {
      iv1[i9] = (int)(d0 + (1.0 + (double)i9)) - 1;
    }

    for (i9 = 0; i9 <= ib; i9++) {
      for (i10 = 0; i10 < 6; i10++) {
        b_P_apo->data[iv1[i10] + b_P_apo->size[0] * i9] = P_apr->data[i10 +
          P_apr->size[0] * i9];
      }
    }

    oldestTrail = b_mod(oldestTrail, trailSize) + 1.0;
  } else {
    i9 = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = P_apr->size[0];
    b_P_apo->size[1] = P_apr->size[1];
    emxEnsureCapacity((emxArray__common *)b_P_apo, i9, (int)sizeof(double));
    ib = P_apr->size[0] * P_apr->size[1];
    for (i9 = 0; i9 < ib; i9++) {
      b_P_apo->data[i9] = P_apr->data[i9];
    }
  }

  // %
  RotFromQuatJ(*(double (*)[4])&b_xt->data[3], R_cw);
  for (ar = 0; ar < 32; ar++) {
    ii = ar * 3;
    for (i9 = 0; i9 < 3; i9++) {
      h_ui[i9] = z_all[i9 + ii];
    }

    if (updateVect[ar] == 2.0) {
      //  a new feature
      //   fprintf('Feature %i: ', i)
      ib = measurementHistory->size[1];
      ii = ar * 3;
      for (i9 = 0; i9 < ib; i9++) {
        for (i10 = 0; i10 < 3; i10++) {
          measurementHistory->data[(i10 + ii) + measurementHistory->size[0] * i9]
            = 0.0;
        }
      }

      //  clean up the measurement  history for this feature
      //          poses(i*7-6:i*7,:)=zeros(7,trailSize);
      pointInMap[ar] = 0.0;
      for (i9 = 0; i9 < 3; i9++) {
        map[i9 + 3 * ar] = rtNaN;
      }

      //  fill in nan for un-used map points so they won't be plotted
      if (z_all[ar * 3 + 2] < -500.0) {
        //  nan disparity means the measurement is useless
        updateVect[ar] = 0.0;

        //   fprintf('rejecting due to invalid disparity\n')
      } else if (z_all[ar * 3 + 2] > disparityThreshold) {
        //  insert into the map immediately
        //      fprintf('inserting immediately\n')
        A = f * b;
        c_y = A / z_all[ar * 3 + 2];
        h_ci[2] = A / z_all[ar * 3 + 2];
        h_ci[0] = (z_all[ar * 3] - Cx) / f * c_y;
        h_ci[1] = (z_all[ar * 3 + 1] - Cy) / f * c_y;
        for (i9 = 0; i9 < 3; i9++) {
          d0 = 0.0;
          for (i10 = 0; i10 < 3; i10++) {
            d0 += R_cw[i10 + 3 * i9] * h_ci[i10];
          }

          map[i9 + 3 * ar] = d0 + b_xt->data[i9];
        }

        pointInMap[ar] = 1.0;

        //  corresponds to pointInitialized
        updateVect[ar] = 1.0;
      } else {
        //  we need to postpone the initialization of this feature
        //   fprintf('delaying initialization\n')
        age[ar] = 0.0;
        updateVect[ar] = 1.0;
        pointInMap[ar] = 0.0;
      }
    }

    if ((updateVect[ar] == 1.0) && (pointInMap[ar] == 0.0) && createNewTrail &&
        (trailSize > 0.0)) {
      //  an existing, yet to be initialized, feature
      age[ar]++;

      //  only increment the age if we created a new trailing pose
      ii = ar * 3;
      for (i9 = 0; i9 < 3; i9++) {
        measurementHistory->data[(i9 + ii) + measurementHistory->size[0] * ((int)
          age[ar] - 1)] = h_ui[i9];
      }
    }

    //  check if a feature has reached maximum age and needs to be
    //  triangulated and inserted into the map
    if ((pointInMap[ar] == 0.0) && (age[ar] == trailSize)) {
      //    fprintf('Feature %i: triangulating\n', i)
      pointInMap[ar] = 0.0;
      PointTriangulation(b_xt, h_ui, cameraparams, fp);
      for (ii = 0; ii < 3; ii++) {
        bv0[ii] = rtIsNaN(fp[ii]);
      }

      if (any(bv0)) {
        pointInMap[ar] = 0.0;
      } else {
        pointInMap[ar] = 1.0;
        for (i9 = 0; i9 < 3; i9++) {
          map[i9 + 3 * ar] = fp[i9];
        }
      }
    }
  }

  br = 0;
  for (k = 0; k < 32; k++) {
    if (pointInMap[k] != 0.0) {
      br++;
    }
  }

  if (br < minFeatureThreshold) {
    //   disp(['Forcing the insertion of ', num2str(minFeatureThreshold - numFeaturesInMap), ' map features']) 
    //  need to initialize some features. Find the ones with highest
    //  disparity
    idx = 0;
    for (i9 = 0; i9 < 32; i9++) {
      b_disparities = z_all[2 + 3 * i9];
      if (pointInMap[i9] == 1.0) {
        b_disparities = 0.0;
      }

      //  only look at features not yet in map
      b0 = rtIsNaN(b_disparities);
      if (b0) {
        idx++;
      }

      disparities[i9] = b_disparities;
      bv1[i9] = b0;
    }

    ii = 0;
    for (ar = 0; ar < 32; ar++) {
      if (bv1[ar]) {
        c_tmp_data[ii] = (signed char)(ar + 1);
        ii++;
      }
    }

    for (i9 = 0; i9 < idx; i9++) {
      disparities[c_tmp_data[i9] - 1] = 0.0;
    }

    //  remove invalid disparities (NaN is bigger than Inf in matlab)
    eml_sort(disparities, iidx);
    memset(&disparities[0], 0, sizeof(double) << 5);
    d0 = minFeatureThreshold - (double)br;
    if (1.0 > d0) {
      ib = 0;
    } else {
      ib = (int)d0;
    }

    for (i9 = 0; i9 < ib; i9++) {
      d_tmp_data[i9] = iidx[i9];
    }

    for (i9 = 0; i9 < ib; i9++) {
      disparities[d_tmp_data[i9] - 1] = 1.0;
    }

    //  triangulate the features with the highest disparities
    for (ar = 0; ar < 32; ar++) {
      if (disparities[ar] != 0.0) {
        //        fprintf('Feature %i: forcing initialization\n', i)
        A = f * b;
        c_y = A / z_all[ar * 3 + 2];
        h_ci[2] = A / z_all[ar * 3 + 2];
        h_ci[0] = (z_all[ar * 3] - Cx) / f * c_y;
        h_ci[1] = (z_all[ar * 3 + 1] - Cy) / f * c_y;
        for (i9 = 0; i9 < 3; i9++) {
          d0 = 0.0;
          for (i10 = 0; i10 < 3; i10++) {
            d0 += R_cw[i10 + 3 * i9] * h_ci[i10];
          }

          map[i9 + 3 * ar] = d0 + b_xt->data[i9];
        }

        pointInMap[ar] = 1.0;

        //  corresponds to pointInitialized
        updateVect[ar] = 1.0;
      }
    }
  }

  memcpy(&map_out[0], &map[0], 96U * sizeof(double));

  // updateVect_out(:)=1;
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_updIT_free()
{
  emxFree_real_T(&measurementHistory);
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_updIT_init()
{
  int i;
  emxInit_real_T(&measurementHistory, 2);
  oldestTrail = 1.0;
  for (i = 0; i < 96; i++) {
    map[i] = rtNaN;
  }

  for (i = 0; i < 32; i++) {
    pointInMap[i] = 0.0;
    age[i] = 0.0;
  }
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
static boolean_T any(const boolean_T x[3])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!!x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : const double v[9]
//                double d[81]
// Return Type  : void
//
static void b_diag(const double v[9], double d[81])
{
  int j;
  memset(&d[0], 0, 81U * sizeof(double));
  for (j = 0; j < 9; j++) {
    d[j + 9 * j] = v[j];
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double b_eml_matlab_zlarfg()
{
  return 0.0;
}

//
// Arguments    : double x[32]
//                int idx[32]
// Return Type  : void
//
static void b_eml_sort(double x[32], int idx[32])
{
  double x4[4];
  signed char idx4[4];
  int m;
  double xwork[32];
  int nNaNs;
  int ib;
  int k;
  int bLen;
  int nPairs;
  int i4;
  signed char perm[4];
  int iwork[32];
  memset(&idx[0], 0, sizeof(int) << 5);
  for (m = 0; m < 4; m++) {
    x4[m] = 0.0;
    idx4[m] = 0;
  }

  nNaNs = 0;
  ib = 0;
  for (k = 0; k < 32; k++) {
    if (rtIsNaN(x[k])) {
      idx[31 - nNaNs] = k + 1;
      xwork[31 - nNaNs] = x[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = (signed char)(k + 1);
      x4[ib - 1] = x[k];
      if (ib == 4) {
        ib = k - nNaNs;
        if (x4[0] >= x4[1]) {
          m = 1;
          bLen = 2;
        } else {
          m = 2;
          bLen = 1;
        }

        if (x4[2] >= x4[3]) {
          nPairs = 3;
          i4 = 4;
        } else {
          nPairs = 4;
          i4 = 3;
        }

        if (x4[m - 1] >= x4[nPairs - 1]) {
          if (x4[bLen - 1] >= x4[nPairs - 1]) {
            perm[0] = (signed char)m;
            perm[1] = (signed char)bLen;
            perm[2] = (signed char)nPairs;
            perm[3] = (signed char)i4;
          } else if (x4[bLen - 1] >= x4[i4 - 1]) {
            perm[0] = (signed char)m;
            perm[1] = (signed char)nPairs;
            perm[2] = (signed char)bLen;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)m;
            perm[1] = (signed char)nPairs;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)bLen;
          }
        } else if (x4[m - 1] >= x4[i4 - 1]) {
          if (x4[bLen - 1] >= x4[i4 - 1]) {
            perm[0] = (signed char)nPairs;
            perm[1] = (signed char)m;
            perm[2] = (signed char)bLen;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)nPairs;
            perm[1] = (signed char)m;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)bLen;
          }
        } else {
          perm[0] = (signed char)nPairs;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)m;
          perm[3] = (signed char)bLen;
        }

        idx[ib - 3] = idx4[perm[0] - 1];
        idx[ib - 2] = idx4[perm[1] - 1];
        idx[ib - 1] = idx4[perm[2] - 1];
        idx[ib] = idx4[perm[3] - 1];
        x[ib - 3] = x4[perm[0] - 1];
        x[ib - 2] = x4[perm[1] - 1];
        x[ib - 1] = x4[perm[2] - 1];
        x[ib] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  if (ib > 0) {
    for (m = 0; m < 4; m++) {
      perm[m] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] >= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] >= x4[1]) {
      if (x4[1] >= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] >= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] >= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] >= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (k = 32; k - 31 <= ib; k++) {
      idx[(k - nNaNs) - ib] = idx4[perm[k - 32] - 1];
      x[(k - nNaNs) - ib] = x4[perm[k - 32] - 1];
    }
  }

  m = nNaNs >> 1;
  for (k = 32; k - 31 <= m; k++) {
    ib = idx[k - nNaNs];
    idx[k - nNaNs] = idx[63 - k];
    idx[63 - k] = ib;
    x[k - nNaNs] = xwork[63 - k];
    x[63 - k] = xwork[k - nNaNs];
  }

  if ((nNaNs & 1) != 0) {
    x[(m - nNaNs) + 32] = xwork[(m - nNaNs) + 32];
  }

  if (32 - nNaNs > 1) {
    nPairs = (32 - nNaNs) >> 2;
    bLen = 4;
    while (nPairs > 1) {
      if ((nPairs & 1) != 0) {
        nPairs--;
        ib = bLen * nPairs;
        m = 32 - (nNaNs + ib);
        if (m > bLen) {
          merge(idx, x, ib, bLen, m - bLen);
        }
      }

      ib = bLen << 1;
      nPairs >>= 1;
      for (k = 1; k <= nPairs; k++) {
        merge(idx, x, (k - 1) * ib, bLen, bLen);
      }

      bLen = ib;
    }

    if (32 - nNaNs > bLen) {
      merge(idx, x, 0, bLen, 32 - (nNaNs + bLen));
    }
  }

  if ((nNaNs > 0) && (32 - nNaNs > 0)) {
    for (k = 32; k - 31 <= nNaNs; k++) {
      xwork[k - 32] = x[k - nNaNs];
      iwork[k - 32] = idx[k - nNaNs];
    }

    for (k = 31 - nNaNs; k + 1 > 0; k--) {
      x[nNaNs + k] = x[k];
      idx[nNaNs + k] = idx[k];
    }

    for (k = 0; k + 1 <= nNaNs; k++) {
      x[k] = xwork[k];
      idx[k] = iwork[k];
    }
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double b_eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x->data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

//
// Arguments    : int n
//                emxArray_real_T *x
//                int ix0
//                int iy0
// Return Type  : void
//
static void b_eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = x->data[ix];
    x->data[ix] = x->data[iy];
    x->data[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : int m
//                int n
//                const emxArray_real_T *A
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
static void b_eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
  emxArray_real_T *B, int ldb)
{
  int j;
  int jBcol;
  int jAcol;
  int k;
  int kBcol;
  int i;
  if (B->size[0] == 0) {
  } else {
    for (j = n; j > 0; j--) {
      jBcol = ldb * (j - 1);
      jAcol = lda * (j - 1);
      for (k = j; k + 1 <= n; k++) {
        kBcol = ldb * k;
        if (A->data[k + jAcol] != 0.0) {
          for (i = 0; i + 1 <= m; i++) {
            B->data[i + jBcol] -= A->data[k + jAcol] * B->data[i + kBcol];
          }
        }
      }
    }
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : double I[144]
// Return Type  : void
//
static void b_eye(double I[144])
{
  int k;
  memset(&I[0], 0, 144U * sizeof(double));
  for (k = 0; k < 12; k++) {
    I[k + 12 * k] = 1.0;
  }
}

//
// Arguments    : double x
//                double y
// Return Type  : double
//
static double b_mod(double x, double y)
{
  double r;
  if (y == 0.0) {
    r = x;
  } else if (y == floor(y)) {
    r = x - floor(x / y) * y;
  } else {
    r = x / y;
    if (fabs(r - rt_roundd_snf(r)) <= 2.2204460492503131E-16 * fabs(r)) {
      r = 0.0;
    } else {
      r = (r - floor(r)) * y;
    }
  }

  return r;
}

//
// Arguments    : const double x[4]
// Return Type  : double
//
static double b_norm(const double x[4])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 4; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double c_eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x->data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

//
// Arguments    : const double v[3]
//                double d[9]
// Return Type  : void
//
static void diag(const double v[3], double d[9])
{
  int j;
  memset(&d[0], 0, 9U * sizeof(double));
  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

//
// Arguments    : int n
//                const double x_data[]
//                int ix0
// Return Type  : int
//
static int eml_ixamax(int n, const double x_data[], int ix0)
{
  int idxmax;
  int ix;
  double smax;
  int k;
  double s;
  if (n < 1) {
    idxmax = 0;
  } else {
    idxmax = 1;
    if (n > 1) {
      ix = ix0 - 1;
      smax = fabs(x_data[ix0 - 1]);
      for (k = 2; k <= n; k++) {
        ix++;
        s = fabs(x_data[ix]);
        if (s > smax) {
          idxmax = k;
          smax = s;
        }
      }
    }
  }

  return idxmax;
}

//
// Arguments    : const emxArray_real_T *A
//                const emxArray_real_T *B
//                emxArray_real_T *X
// Return Type  : void
//
static void eml_lusolve(const emxArray_real_T *A, const emxArray_real_T *B,
  emxArray_real_T *X)
{
  emxArray_real_T *b_A;
  int info;
  int jp;
  int ipiv_size[2];
  int ipiv_data[96];
  int xi;
  double temp;
  emxInit_real_T(&b_A, 2);
  info = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, info, (int)sizeof(double));
  jp = A->size[0] * A->size[1];
  for (info = 0; info < jp; info++) {
    b_A->data[info] = A->data[info];
  }

  eml_xgetrf(A->size[1], A->size[1], b_A, A->size[1], ipiv_data, ipiv_size,
             &info);
  info = X->size[0] * X->size[1];
  X->size[0] = B->size[0];
  X->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)X, info, (int)sizeof(double));
  jp = B->size[0] * B->size[1];
  for (info = 0; info < jp; info++) {
    X->data[info] = B->data[info];
  }

  eml_xtrsm(B->size[0], A->size[1], b_A, A->size[1], X, B->size[0]);
  b_eml_xtrsm(B->size[0], A->size[1], b_A, A->size[1], X, B->size[0]);
  info = A->size[1] - 2;
  emxFree_real_T(&b_A);
  while (info + 1 > 0) {
    if (ipiv_data[info] != info + 1) {
      jp = ipiv_data[info] - 1;
      for (xi = 0; xi + 1 <= B->size[0]; xi++) {
        temp = X->data[xi + X->size[0] * info];
        X->data[xi + X->size[0] * info] = X->data[xi + X->size[0] * jp];
        X->data[xi + X->size[0] * jp] = temp;
      }
    }

    info--;
  }
}

//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                emxArray_real_T *C
//                int ic0
//                int ldc
//                double work_data[]
// Return Type  : void
//
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, emxArray_real_T *
  C, int ic0, int ldc, double work_data[])
{
  int lastv;
  int i;
  int lastc;
  boolean_T exitg2;
  int ia;
  int32_T exitg1;
  int i14;
  int jy;
  int ix;
  double c;
  int j;
  if (tau != 0.0) {
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C->data[i - 2] == 0.0)) {
      lastv--;
      i--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      i = ic0 + (lastc - 1) * ldc;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C->data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    if (lastc == 0) {
    } else {
      for (i = 1; i <= lastc; i++) {
        work_data[i - 1] = 0.0;
      }

      i = 0;
      i14 = ic0 + ldc * (lastc - 1);
      for (jy = ic0; jy <= i14; jy += ldc) {
        ix = iv0;
        c = 0.0;
        j = (jy + lastv) - 1;
        for (ia = jy; ia <= j; ia++) {
          c += C->data[ia - 1] * C->data[ix - 1];
          ix++;
        }

        work_data[i] += c;
        i++;
      }
    }

    if (-tau == 0.0) {
    } else {
      i = ic0 - 1;
      jy = 0;
      for (j = 1; j <= lastc; j++) {
        if (work_data[jy] != 0.0) {
          c = work_data[jy] * -tau;
          ix = iv0;
          i14 = lastv + i;
          for (ia = i; ia + 1 <= i14; ia++) {
            C->data[ia] += C->data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        i += ldc;
      }
    }
  }
}

//
// Arguments    : int n
//                double *alpha1
//                emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double eml_matlab_zlarfg(int n, double *alpha1, emxArray_real_T *x, int
  ix0)
{
  double tau;
  double xnorm;
  int knt;
  int i13;
  int k;
  tau = 0.0;
  if (n <= 0) {
  } else {
    xnorm = b_eml_xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          i13 = (ix0 + n) - 2;
          for (k = ix0; k <= i13; k++) {
            x->data[k - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = b_eml_xnrm2(n - 1, x, ix0);
        xnorm = rt_hypotd_snf(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i13 = (ix0 + n) - 2;
        for (k = ix0; k <= i13; k++) {
          x->data[k - 1] *= *alpha1;
        }

        for (k = 1; k <= knt; k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i13 = (ix0 + n) - 2;
        for (k = ix0; k <= i13; k++) {
          x->data[k - 1] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

//
// Arguments    : const emxArray_real_T *A
//                const emxArray_real_T *B
//                emxArray_real_T *Y
// Return Type  : void
//
static void eml_qrsolve(const emxArray_real_T *A, const emxArray_real_T *B,
  emxArray_real_T *Y)
{
  emxArray_real_T *b_A;
  int i;
  int j;
  emxArray_real_T *b_B;
  int jpvt_size[2];
  int jpvt_data[96];
  int tau_size[1];
  double tau_data[96];
  int rankR;
  int m;
  int nb;
  int mn;
  int k;
  double wj;
  emxInit_real_T(&b_A, 2);
  i = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, i, (int)sizeof(double));
  j = A->size[0] * A->size[1];
  for (i = 0; i < j; i++) {
    b_A->data[i] = A->data[i];
  }

  emxInit_real_T(&b_B, 2);
  eml_xgeqp3(b_A, tau_data, tau_size, jpvt_data, jpvt_size);
  rankR = rankFromQR(b_A);
  i = b_B->size[0] * b_B->size[1];
  b_B->size[0] = B->size[0];
  b_B->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)b_B, i, (int)sizeof(double));
  j = B->size[0] * B->size[1];
  for (i = 0; i < j; i++) {
    b_B->data[i] = B->data[i];
  }

  m = b_A->size[0];
  nb = B->size[1];
  j = b_A->size[0];
  mn = b_A->size[1];
  if (j <= mn) {
    mn = j;
  }

  j = b_A->size[1];
  k = B->size[1];
  i = Y->size[0] * Y->size[1];
  Y->size[0] = j;
  emxEnsureCapacity((emxArray__common *)Y, i, (int)sizeof(double));
  i = Y->size[0] * Y->size[1];
  Y->size[1] = k;
  emxEnsureCapacity((emxArray__common *)Y, i, (int)sizeof(double));
  j *= k;
  for (i = 0; i < j; i++) {
    Y->data[i] = 0.0;
  }

  for (j = 0; j + 1 <= mn; j++) {
    if (tau_data[j] != 0.0) {
      for (k = 0; k + 1 <= nb; k++) {
        wj = b_B->data[j + b_B->size[0] * k];
        for (i = j + 1; i + 1 <= m; i++) {
          wj += b_A->data[i + b_A->size[0] * j] * b_B->data[i + b_B->size[0] * k];
        }

        wj *= tau_data[j];
        if (wj != 0.0) {
          b_B->data[j + b_B->size[0] * k] -= wj;
          for (i = j + 1; i + 1 <= m; i++) {
            b_B->data[i + b_B->size[0] * k] -= b_A->data[i + b_A->size[0] * j] *
              wj;
          }
        }
      }
    }
  }

  for (k = 0; k + 1 <= nb; k++) {
    for (i = 0; i + 1 <= rankR; i++) {
      Y->data[(jpvt_data[i] + Y->size[0] * k) - 1] = b_B->data[i + b_B->size[0] *
        k];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
      Y->data[(jpvt_data[j] + Y->size[0] * k) - 1] /= b_A->data[j + b_A->size[0]
        * j];
      for (i = 0; i + 1 <= j; i++) {
        Y->data[(jpvt_data[i] + Y->size[0] * k) - 1] -= Y->data[(jpvt_data[j] +
          Y->size[0] * k) - 1] * b_A->data[i + b_A->size[0] * j];
      }
    }
  }

  emxFree_real_T(&b_B);
  emxFree_real_T(&b_A);
}

//
// Arguments    : int b
//                int y_data[]
//                int y_size[2]
// Return Type  : void
//
static void eml_signed_integer_colon(int b, int y_data[], int y_size[2])
{
  int yk;
  int k;
  y_size[0] = 1;
  y_size[1] = b;
  y_data[0] = 1;
  yk = 1;
  for (k = 2; k <= b; k++) {
    yk++;
    y_data[k - 1] = yk;
  }
}

//
// Arguments    : double x[32]
//                int idx[32]
// Return Type  : void
//
static void eml_sort(double x[32], int idx[32])
{
  b_eml_sort(x, idx);
}

//
// Arguments    : emxArray_real_T *A
//                double tau_data[]
//                int tau_size[1]
//                int jpvt_data[]
//                int jpvt_size[2]
// Return Type  : void
//
static void eml_xgeqp3(emxArray_real_T *A, double tau_data[], int tau_size[1],
  int jpvt_data[], int jpvt_size[2])
{
  int m;
  int n;
  int mn;
  signed char unnamed_idx_0;
  int k;
  int itemp;
  double work_data[96];
  double vn1_data[96];
  double vn2_data[96];
  int i;
  int i_i;
  int nmi;
  int mmi;
  double atmp;
  double temp2;
  m = A->size[0];
  n = A->size[1];
  if (A->size[0] <= A->size[1]) {
    mn = A->size[0];
  } else {
    mn = A->size[1];
  }

  tau_size[0] = (signed char)mn;
  eml_signed_integer_colon(A->size[1], jpvt_data, jpvt_size);
  unnamed_idx_0 = (signed char)A->size[1];
  k = unnamed_idx_0;
  for (itemp = 0; itemp < k; itemp++) {
    work_data[itemp] = 0.0;
  }

  k = 1;
  for (itemp = 0; itemp + 1 <= n; itemp++) {
    vn1_data[itemp] = eml_xnrm2(m, A, k);
    vn2_data[itemp] = vn1_data[itemp];
    k += m;
  }

  for (i = 0; i + 1 <= mn; i++) {
    i_i = i + i * m;
    nmi = (n - i) - 1;
    mmi = m - i;
    k = eml_ixamax(1 + nmi, vn1_data, i + 1);
    k = (i + k) - 1;
    if (k + 1 != i + 1) {
      b_eml_xswap(m, A, 1 + m * k, 1 + m * i);
      itemp = jpvt_data[k];
      jpvt_data[k] = jpvt_data[i];
      jpvt_data[i] = itemp;
      vn1_data[k] = vn1_data[i];
      vn2_data[k] = vn2_data[i];
    }

    if (i + 1 < m) {
      atmp = A->data[i_i];
      tau_data[i] = eml_matlab_zlarfg(mmi, &atmp, A, i_i + 2);
    } else {
      atmp = A->data[i_i];
      tau_data[i] = b_eml_matlab_zlarfg();
    }

    A->data[i_i] = atmp;
    if (i + 1 < n) {
      atmp = A->data[i_i];
      A->data[i_i] = 1.0;
      eml_matlab_zlarf(mmi, nmi, i_i + 1, tau_data[i], A, (i + (i + 1) * m) + 1,
                       m, work_data);
      A->data[i_i] = atmp;
    }

    for (itemp = i + 1; itemp + 1 <= n; itemp++) {
      if (vn1_data[itemp] != 0.0) {
        atmp = fabs(A->data[i + A->size[0] * itemp]) / vn1_data[itemp];
        atmp = 1.0 - atmp * atmp;
        if (atmp < 0.0) {
          atmp = 0.0;
        }

        temp2 = vn1_data[itemp] / vn2_data[itemp];
        temp2 = atmp * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          if (i + 1 < m) {
            vn1_data[itemp] = c_eml_xnrm2(mmi - 1, A, (i + m * itemp) + 2);
            vn2_data[itemp] = vn1_data[itemp];
          } else {
            vn1_data[itemp] = 0.0;
            vn2_data[itemp] = 0.0;
          }
        } else {
          vn1_data[itemp] *= sqrt(atmp);
        }
      }
    }
  }
}

//
// Arguments    : int m
//                int n
//                emxArray_real_T *A
//                int lda
//                int ipiv_data[]
//                int ipiv_size[2]
//                int *info
// Return Type  : void
//
static void eml_xgetrf(int m, int n, emxArray_real_T *A, int lda, int ipiv_data[],
  int ipiv_size[2], int *info)
{
  int b_m;
  int i11;
  int j;
  int mmj;
  int c;
  int i;
  int ix;
  double smax;
  int jA;
  double s;
  int i12;
  int jy;
  int b_j;
  int ijA;
  if (m <= n) {
    b_m = m;
  } else {
    b_m = n;
  }

  eml_signed_integer_colon(b_m, ipiv_data, ipiv_size);
  *info = 0;
  if (m - 1 <= n) {
    i11 = m - 1;
  } else {
    i11 = n;
  }

  for (j = 1; j <= i11; j++) {
    mmj = (m - j) + 1;
    c = (j - 1) * (lda + 1);
    if (mmj < 1) {
      i = -1;
    } else {
      i = 0;
      if (mmj > 1) {
        ix = c;
        smax = fabs(A->data[c]);
        for (jA = 1; jA + 1 <= mmj; jA++) {
          ix++;
          s = fabs(A->data[ix]);
          if (s > smax) {
            i = jA;
            smax = s;
          }
        }
      }
    }

    if (A->data[c + i] != 0.0) {
      if (i != 0) {
        ipiv_data[j - 1] = j + i;
        eml_xswap(n, A, j, lda, j + i, lda);
      }

      i12 = c + mmj;
      for (i = c + 1; i + 1 <= i12; i++) {
        A->data[i] /= A->data[c];
      }
    } else {
      *info = j;
    }

    i = n - j;
    jA = c + lda;
    jy = c + lda;
    for (b_j = 1; b_j <= i; b_j++) {
      smax = A->data[jy];
      if (A->data[jy] != 0.0) {
        ix = c + 1;
        i12 = mmj + jA;
        for (ijA = 1 + jA; ijA + 1 <= i12; ijA++) {
          A->data[ijA] += A->data[ix] * -smax;
          ix++;
        }
      }

      jy += lda;
      jA += lda;
    }
  }

  if ((*info == 0) && (m <= n) && (!(A->data[(m + A->size[0] * (m - 1)) - 1] !=
        0.0))) {
    *info = m;
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x->data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

//
// Arguments    : int n
//                emxArray_real_T *x
//                int ix0
//                int incx
//                int iy0
//                int incy
// Return Type  : void
//
static void eml_xswap(int n, emxArray_real_T *x, int ix0, int incx, int iy0, int
                      incy)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = x->data[ix];
    x->data[ix] = x->data[iy];
    x->data[iy] = temp;
    ix += incx;
    iy += incy;
  }
}

//
// Arguments    : int m
//                int n
//                const emxArray_real_T *A
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
static void eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
                      emxArray_real_T *B, int ldb)
{
  int j;
  int jBcol;
  int jAcol;
  int k;
  int kBcol;
  int i;
  double temp;
  if (B->size[0] == 0) {
  } else {
    for (j = 0; j + 1 <= n; j++) {
      jBcol = ldb * j;
      jAcol = lda * j;
      for (k = 0; k + 1 <= j; k++) {
        kBcol = ldb * k;
        if (A->data[k + jAcol] != 0.0) {
          for (i = 0; i + 1 <= m; i++) {
            B->data[i + jBcol] -= A->data[k + jAcol] * B->data[i + kBcol];
          }
        }
      }

      temp = 1.0 / A->data[j + jAcol];
      for (i = 0; i + 1 <= m; i++) {
        B->data[i + jBcol] *= temp;
      }
    }
  }
}

//
// Arguments    : emxArray__common *emxArray
//                int oldNumel
//                int elementSize
// Return Type  : void
//
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize)
{
  int newNumel;
  int i;
  void *newData;
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      i <<= 1;
    }

    newData = calloc((unsigned int)i, (unsigned int)elementSize);
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, (unsigned int)(elementSize * oldNumel));
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

//
// Arguments    : emxArray_int32_T **pEmxArray
// Return Type  : void
//
static void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
// Return Type  : void
//
static void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

//
// Arguments    : emxArray_int32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : double varargin_1
//                emxArray_real_T *I
// Return Type  : void
//
static void eye(double varargin_1, emxArray_real_T *I)
{
  int k;
  int loop_ub;
  k = I->size[0] * I->size[1];
  I->size[0] = (int)varargin_1;
  I->size[1] = (int)varargin_1;
  emxEnsureCapacity((emxArray__common *)I, k, (int)sizeof(double));
  loop_ub = (int)varargin_1 * (int)varargin_1;
  for (k = 0; k < loop_ub; k++) {
    I->data[k] = 0.0;
  }

  if ((int)varargin_1 > 0) {
    for (k = 0; k + 1 <= (int)varargin_1; k++) {
      I->data[k + I->size[0] * k] = 1.0;
    }
  }
}

//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const double B[9]
//                emxArray_real_T *K
// Return Type  : void
//
static void kron(const double A_data[], const int A_size[2], const double B[9],
                 emxArray_real_T *K)
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  kidx = K->size[0] * K->size[1];
  K->size[0] = (signed char)(A_size[0] * 3);
  K->size[1] = (signed char)(A_size[1] * 3);
  emxEnsureCapacity((emxArray__common *)K, kidx, (int)sizeof(double));
  kidx = -1;
  for (b_j1 = 1; b_j1 <= A_size[1]; b_j1++) {
    for (j2 = 0; j2 < 3; j2++) {
      for (i1 = 1; i1 <= A_size[0]; i1++) {
        for (i2 = 0; i2 < 3; i2++) {
          kidx++;
          K->data[kidx] = A_data[(i1 + A_size[0] * (b_j1 - 1)) - 1] * B[i2 + 3 *
            j2];
        }
      }
    }
  }
}

//
// Arguments    : int idx[32]
//                double x[32]
//                int offset
//                int np
//                int nq
// Return Type  : void
//
static void merge(int idx[32], double x[32], int offset, int np, int nq)
{
  int iwork[32];
  double xwork[32];
  int n;
  int qend;
  int p;
  int iout;
  int32_T exitg1;
  if (nq == 0) {
  } else {
    n = np + nq;
    for (qend = 0; qend + 1 <= n; qend++) {
      iwork[qend] = idx[offset + qend];
      xwork[qend] = x[offset + qend];
    }

    p = 0;
    n = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] >= xwork[n]) {
        idx[iout] = iwork[p];
        x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[n];
        x[iout] = xwork[n];
        if (n + 1 < qend) {
          n++;
        } else {
          n = (iout - p) + 1;
          while (p + 1 <= np) {
            idx[n + p] = iwork[p];
            x[n + p] = xwork[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

//
// Arguments    : const emxArray_real_T *A
//                const emxArray_real_T *B
//                emxArray_real_T *y
// Return Type  : void
//
static void mrdivide(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *y)
{
  signed char unnamed_idx_1;
  int i3;
  emxArray_real_T *b_B;
  int loop_ub;
  int A_idx_1;
  int i4;
  emxArray_real_T *b_A;
  emxArray_real_T *c_A;
  emxArray_real_T *r1;
  if (A->size[0] == 0) {
    unnamed_idx_1 = (signed char)B->size[0];
    i3 = y->size[0] * y->size[1];
    y->size[0] = 0;
    emxEnsureCapacity((emxArray__common *)y, i3, (int)sizeof(double));
    i3 = y->size[0] * y->size[1];
    y->size[1] = unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, i3, (int)sizeof(double));
  } else if (B->size[0] == B->size[1]) {
    eml_lusolve(B, A, y);
  } else {
    emxInit_real_T(&b_B, 2);
    i3 = b_B->size[0] * b_B->size[1];
    b_B->size[0] = B->size[1];
    b_B->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_B, i3, (int)sizeof(double));
    loop_ub = B->size[0];
    for (i3 = 0; i3 < loop_ub; i3++) {
      A_idx_1 = B->size[1];
      for (i4 = 0; i4 < A_idx_1; i4++) {
        b_B->data[i4 + b_B->size[0] * i3] = B->data[i3 + B->size[0] * i4];
      }
    }

    emxInit_real_T(&b_A, 2);
    emxInit_real_T(&c_A, 2);
    i3 = c_A->size[0] * c_A->size[1];
    c_A->size[0] = A->size[1];
    c_A->size[1] = A->size[0];
    emxEnsureCapacity((emxArray__common *)c_A, i3, (int)sizeof(double));
    loop_ub = A->size[0];
    for (i3 = 0; i3 < loop_ub; i3++) {
      A_idx_1 = A->size[1];
      for (i4 = 0; i4 < A_idx_1; i4++) {
        c_A->data[i4 + c_A->size[0] * i3] = A->data[i3 + A->size[0] * i4];
      }
    }

    loop_ub = A->size[1];
    A_idx_1 = A->size[0];
    i3 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = loop_ub;
    b_A->size[1] = A_idx_1;
    emxEnsureCapacity((emxArray__common *)b_A, i3, (int)sizeof(double));
    for (i3 = 0; i3 < A_idx_1; i3++) {
      for (i4 = 0; i4 < loop_ub; i4++) {
        b_A->data[i4 + b_A->size[0] * i3] = c_A->data[i4 + loop_ub * i3];
      }
    }

    emxFree_real_T(&c_A);
    emxInit_real_T(&r1, 2);
    eml_qrsolve(b_B, b_A, r1);
    i3 = y->size[0] * y->size[1];
    y->size[0] = r1->size[1];
    y->size[1] = r1->size[0];
    emxEnsureCapacity((emxArray__common *)y, i3, (int)sizeof(double));
    loop_ub = r1->size[0];
    emxFree_real_T(&b_A);
    emxFree_real_T(&b_B);
    for (i3 = 0; i3 < loop_ub; i3++) {
      A_idx_1 = r1->size[1];
      for (i4 = 0; i4 < A_idx_1; i4++) {
        y->data[i4 + y->size[0] * i3] = r1->data[i3 + r1->size[0] * i4];
      }
    }

    emxFree_real_T(&r1);
  }
}

//
// Arguments    : const double x[3]
// Return Type  : double
//
static double norm(const double x[3])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 3; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// Arguments    : const double a[3]
//                double y[3]
// Return Type  : void
//
static void power(const double a[3], double y[3])
{
  int k;
  for (k = 0; k < 3; k++) {
    y[k] = a[k] * a[k];
  }
}

//
// Arguments    : const double dtheta[3]
//                double dq[4]
// Return Type  : void
//
static void quatPlusThetaJ(const double dtheta[3], double dq[4])
{
  double theta;
  int i;
  double B;
  theta = norm(dtheta) * 0.5;
  if (theta < 0.244) {
    for (i = 0; i < 3; i++) {
      dq[i] = 0.5 * dtheta[i];
    }

    dq[3] = 1.0;
  } else {
    dq[0] = 0.5 * dtheta[0] * sin(theta) / theta;
    dq[1] = 0.5 * dtheta[1] * sin(theta) / theta;
    dq[2] = 0.5 * dtheta[2] * sin(theta) / theta;
    dq[3] = cos(theta);
  }

  B = b_norm(dq);
  for (i = 0; i < 4; i++) {
    dq[i] /= B;
  }
}

//
// Arguments    : const double q[4]
//                const double p[4]
//                double qp[4]
// Return Type  : void
//
static void quatmultJ(const double q[4], const double p[4], double qp[4])
{
  double b_p[16];
  double b_q[4];
  int i5;
  int i6;
  b_p[0] = p[3];
  b_p[4] = -p[2];
  b_p[8] = p[1];
  b_p[12] = p[0];
  b_p[1] = p[2];
  b_p[5] = p[3];
  b_p[9] = -p[0];
  b_p[13] = p[1];
  b_p[2] = -p[1];
  b_p[6] = p[0];
  b_p[10] = p[3];
  b_p[14] = p[2];
  b_p[3] = -p[0];
  b_p[7] = -p[1];
  b_p[11] = -p[2];
  b_p[15] = p[3];
  b_q[0] = q[0];
  b_q[1] = q[1];
  b_q[2] = q[2];
  b_q[3] = q[3];
  for (i5 = 0; i5 < 4; i5++) {
    qp[i5] = 0.0;
    for (i6 = 0; i6 < 4; i6++) {
      qp[i5] += b_p[i5 + (i6 << 2)] * b_q[i6];
    }
  }
}

//
// Arguments    : const emxArray_real_T *A
// Return Type  : int
//
static int rankFromQR(const emxArray_real_T *A)
{
  int r;
  int minmn;
  int maxmn;
  double tol;
  r = 0;
  if (A->size[0] < A->size[1]) {
    minmn = A->size[0];
    maxmn = A->size[1];
  } else {
    minmn = A->size[1];
    maxmn = A->size[0];
  }

  tol = (double)maxmn * fabs(A->data[0]) * 2.2204460492503131E-16;
  while ((r < minmn) && (fabs(A->data[r + A->size[0] * r]) >= tol)) {
    r++;
  }

  return r;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// Arguments    : double updateVect[32]
//                const double z_all[96]
//                const double cameraparams[4]
//                double dt
//                const double processNoise[3]
//                const double IMU_measurements[9]
//                const double imNoise[3]
//                double numPointsPerAnchor
//                double numAnchors
//                double h_u_apo[96]
//                emxArray_real_T *xt_out
//                double updateVect_out[32]
//                emxArray_real_T *P_apo_out
//                double b_map[96]
// Return Type  : void
//
void SLAM(double updateVect[32], const double z_all[96], const double
          cameraparams[4], double dt, const double processNoise[3], const double
          IMU_measurements[9], const double imNoise[3], double, double, double
          h_u_apo[96], emxArray_real_T *xt_out, double updateVect_out[32],
          emxArray_real_T *P_apo_out, double b_map[96])
{
  emxArray_real_T *P_apr;
  double B;
  double z_n_b[3];
  int outsize_idx_0;
  double y_n_b[3];
  int i0;
  double x_n_b[3];
  double b_x_n_b[9];
  emxArray_real_T *b;
  double q1[4];
  int outsize_idx_1;
  int k;
  static const signed char a[7] = { 0, 0, 100, 0, 0, 0, 1 };

  int loop_ub;
  int i1;
  static const double dv0[9] = { 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1 };

  emxArray_real_T *r0;
  emxInit_real_T(&P_apr, 2);
  if (!initialized_not_empty) {
    initialized_not_empty = true;
    B = norm(*(double (*)[3])&IMU_measurements[3]);
    for (outsize_idx_0 = 0; outsize_idx_0 < 3; outsize_idx_0++) {
      z_n_b[outsize_idx_0] = IMU_measurements[outsize_idx_0 + 3] / B;
    }

    y_n_b[0] = z_n_b[1] * 0.0 - z_n_b[2] * 0.0;
    y_n_b[1] = z_n_b[2] - z_n_b[0] * 0.0;
    y_n_b[2] = z_n_b[0] * 0.0 - z_n_b[1];
    B = norm(y_n_b);
    for (i0 = 0; i0 < 3; i0++) {
      y_n_b[i0] /= B;
    }

    x_n_b[0] = y_n_b[1] * z_n_b[2] - y_n_b[2] * z_n_b[1];
    x_n_b[1] = y_n_b[2] * z_n_b[0] - y_n_b[0] * z_n_b[2];
    x_n_b[2] = y_n_b[0] * z_n_b[1] - y_n_b[1] * z_n_b[0];
    B = norm(x_n_b);
    for (i0 = 0; i0 < 3; i0++) {
      b_x_n_b[i0] = x_n_b[i0] / B;
      b_x_n_b[3 + i0] = y_n_b[i0];
      b_x_n_b[6 + i0] = z_n_b[i0];
    }

    b_emxInit_real_T(&b, 1);
    QuatFromRotJ(b_x_n_b, q1);

    //  initialize the trail poses with large values to make sure we create
    //  a new pose at the start
    outsize_idx_0 = 7 * (int)trailSize;
    i0 = b->size[0];
    b->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)b, i0, (int)sizeof(double));
    if (!(outsize_idx_0 == 0)) {
      for (outsize_idx_0 = 1; outsize_idx_0 <= (int)trailSize; outsize_idx_0++)
      {
        outsize_idx_1 = (outsize_idx_0 - 1) * 7;
        for (k = 0; k < 7; k++) {
          b->data[outsize_idx_1 + k] = a[k];
        }
      }
    }

    i0 = xt->size[0];
    xt->size[0] = 13 + b->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i0, (int)sizeof(double));
    xt->data[0] = 0.0;
    xt->data[1] = 0.0;
    xt->data[2] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      xt->data[i0 + 3] = q1[i0];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    xt->data[10] = 0.0;
    xt->data[11] = 0.0;
    xt->data[12] = 0.0;
    outsize_idx_0 = b->size[0];
    for (i0 = 0; i0 < outsize_idx_0; i0++) {
      xt->data[i0 + 13] = b->data[i0];
    }

    emxFree_real_T(&b);

    //  initial real vector
    B = 6.0 * trailSize;
    outsize_idx_0 = (int)numStates + (int)B;
    outsize_idx_1 = (int)numStates + (int)B;
    i0 = P_apr->size[0] * P_apr->size[1];
    P_apr->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)P_apr, i0, (int)sizeof(double));
    i0 = P_apr->size[0] * P_apr->size[1];
    P_apr->size[1] = outsize_idx_1;
    emxEnsureCapacity((emxArray__common *)P_apr, i0, (int)sizeof(double));
    outsize_idx_0 *= outsize_idx_1;
    for (i0 = 0; i0 < outsize_idx_0; i0++) {
      P_apr->data[i0] = 0.0;
    }

    if ((int)numStates > 0) {
      outsize_idx_0 = (int)numStates;
      for (i0 = 0; i0 < outsize_idx_0; i0++) {
        loop_ub = (int)numStates;
        for (outsize_idx_1 = 0; outsize_idx_1 < loop_ub; outsize_idx_1++) {
          P_apr->data[outsize_idx_1 + P_apr->size[0] * i0] = 0.0;
        }
      }
    }

    if ((int)B > 0) {
      if ((int)numStates + 1 > (int)numStates + (int)B) {
        i0 = 1;
      } else {
        i0 = (int)numStates + 1;
      }

      if ((int)numStates + 1 > (int)numStates + (int)B) {
        outsize_idx_1 = 1;
      } else {
        outsize_idx_1 = (int)numStates + 1;
      }

      outsize_idx_0 = (int)B;
      for (k = 0; k < outsize_idx_0; k++) {
        loop_ub = (int)B;
        for (i1 = 0; i1 < loop_ub; i1++) {
          P_apr->data[((i0 + i1) + P_apr->size[0] * ((outsize_idx_1 + k) - 1)) -
            1] = 0.0;
        }
      }
    }

    //  initial error state covariance
    for (i0 = 0; i0 < 3; i0++) {
      for (outsize_idx_1 = 0; outsize_idx_1 < 3; outsize_idx_1++) {
        P_apr->data[(outsize_idx_1 + P_apr->size[0] * (9 + i0)) + 9] =
          dv0[outsize_idx_1 + 3 * i0];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (outsize_idx_1 = 0; outsize_idx_1 < 3; outsize_idx_1++) {
        P_apr->data[outsize_idx_1 + P_apr->size[0] * i0] = 0.0;
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (outsize_idx_1 = 0; outsize_idx_1 < 3; outsize_idx_1++) {
        P_apr->data[(outsize_idx_1 + P_apr->size[0] * (3 + i0)) + 3] = 0.0;
      }
    }

    SLAM_updIT(P_apr, xt, cameraparams, updateVect, z_all, imNoise, P_apo, b_map);
  }

  SLAM_pred_euler(P_apo, xt, dt, processNoise, IMU_measurements, numStates);
  memcpy(&updateVect_out[0], &updateVect[0], sizeof(double) << 5);
  emxInit_real_T(&r0, 2);
  i0 = r0->size[0] * r0->size[1];
  r0->size[0] = P_apo->size[0];
  r0->size[1] = P_apo->size[1];
  emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(double));
  outsize_idx_0 = P_apo->size[0] * P_apo->size[1];
  for (i0 = 0; i0 < outsize_idx_0; i0++) {
    r0->data[i0] = P_apo->data[i0];
  }

  SLAM_updIT(r0, xt, cameraparams, updateVect_out, z_all, imNoise, P_apr, b_map);
  emxFree_real_T(&r0);
  for (outsize_idx_0 = 0; outsize_idx_0 < 96; outsize_idx_0++) {
    h_u_apo[outsize_idx_0] = 0.0;
  }

  i0 = P_apo->size[0] * P_apo->size[1];
  P_apo->size[0] = P_apr->size[0];
  P_apo->size[1] = P_apr->size[1];
  emxEnsureCapacity((emxArray__common *)P_apo, i0, (int)sizeof(double));
  outsize_idx_0 = P_apr->size[0] * P_apr->size[1];
  for (i0 = 0; i0 < outsize_idx_0; i0++) {
    P_apo->data[i0] = P_apr->data[i0];
  }

  emxFree_real_T(&P_apr);
  i0 = xt_out->size[0];
  xt_out->size[0] = xt->size[0];
  emxEnsureCapacity((emxArray__common *)xt_out, i0, (int)sizeof(double));
  outsize_idx_0 = xt->size[0];
  for (i0 = 0; i0 < outsize_idx_0; i0++) {
    xt_out->data[i0] = xt->data[i0];
  }

  i0 = P_apo_out->size[0] * P_apo_out->size[1];
  P_apo_out->size[0] = P_apo->size[0];
  P_apo_out->size[1] = P_apo->size[1];
  emxEnsureCapacity((emxArray__common *)P_apo_out, i0, (int)sizeof(double));
  outsize_idx_0 = P_apo->size[0] * P_apo->size[1];
  for (i0 = 0; i0 < outsize_idx_0; i0++) {
    P_apo_out->data[i0] = P_apo->data[i0];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  rt_InitInfAndNaN(8U);
  normalGravity = b_normalGravity;
  UseAccToPredict = b_UseAccToPredict;
  disparityThreshold = b_disparityThreshold;
  minFeatureThreshold = b_minFeatureThreshold;
  baselineThresold = b_baselineThresold;
  numStatesxt = b_numStatesxt;
  msckfUPD = b_msckfUPD;
  map_not_empty = false;
  numStates = b_numStates;
  trailSize = b_trailSize;
  initialized_not_empty = false;
  SLAM_init();
  SLAM_updIT_init();
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_terminate()
{
  SLAM_free();
  SLAM_updIT_free();
}

//
// Arguments    : int b_numDimensions
//                int *b_size
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateND_real_T(int b_numDimensions, int *b_size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  b_emxInit_real_T(&emx, b_numDimensions);
  numEl = 1;
  for (i = 0; i < b_numDimensions; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = b_numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : double *b_data
//                int b_numDimensions
//                int *b_size
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateWrapperND_real_T(double *b_data, int b_numDimensions,
  int *b_size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  b_emxInit_real_T(&emx, b_numDimensions);
  numEl = 1;
  for (i = 0; i < b_numDimensions; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = b_data;
  emx->numDimensions = b_numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : double *b_data
//                int rows
//                int cols
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateWrapper_real_T(double *b_data, int rows, int cols)
{
  emxArray_real_T *emx;
  int b_size[2];
  int numEl;
  int i;
  b_size[0] = rows;
  b_size[1] = cols;
  b_emxInit_real_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = b_data;
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : int rows
//                int cols
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreate_real_T(int rows, int cols)
{
  emxArray_real_T *emx;
  int b_size[2];
  int numEl;
  int i;
  b_size[0] = rows;
  b_size[1] = cols;
  b_emxInit_real_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : emxArray_real_T *emxArray
// Return Type  : void
//
void emxDestroyArray_real_T(emxArray_real_T *emxArray)
{
  emxFree_real_T(&emxArray);
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
void emxInitArray_real_T(emxArray_real_T **pEmxArray, int b_numDimensions)
{
  b_emxInit_real_T(pEmxArray, b_numDimensions);
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
