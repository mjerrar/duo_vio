//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-May-2015 18:39:23
//

// Include Files
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

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P_apo;
static boolean_T b_initialized_not_empty;
static double prev_meas[6];
static boolean_T c_initialized_not_empty;
static emxArray_real_T *anchor_u;
static emxArray_real_T *anchor_pose;

// Function Declarations
static void RotFromQuat(const double q[4], double R[9]);
static void SLAM_free();
static void SLAM_init();
static void SLAM_pred(emxArray_real_T *b_P_apo, emxArray_real_T *x, double dt,
                      const double processNoise[4], const double
                      IMU_measurements[9]);
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams[4], const double updateVect[32], const double z_all[96],
  double imNoise[3], double numPointsPerAnchor, double numAnchors,
  emxArray_real_T *h_u_apo, double updateVect_out[32], emxArray_real_T
  *anchor_u_out, emxArray_real_T *anchor_pose_out);
static void SLAM_updIT_free();
static void SLAM_updIT_init();
static void b_RotFromQuat(const double q[4], double R[9]);
static double b_eml_matlab_zlarfg();
static double b_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void b_eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0);
static void b_eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
  emxArray_real_T *B, int ldb);
static void b_emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static double b_mod(double x, double y);
static void b_mrdivide(const double A[2], const double B[4], double y[2]);
static void blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
                    *varargin_2, const emxArray_real_T *varargin_3,
                    emxArray_real_T *y);
static double c_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void diag(const double v[3], double d[9]);
static void dxdt_dPdt(double part, double dt, const double b_prev_meas[6], const
                      double curr_meas[6], const double x[22], const double P
                      [441], const double Phi[441], const double Q[144], double
                      x_dot[22], double P_dot[441], double Phi_dot[441]);
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
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void eye(double varargin_1, emxArray_real_T *I);
static void kron(const double A_data[], const int A_size[2], const double B[9],
                 emxArray_real_T *K);
static void mrdivide(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *y);
static double norm(const double x[4]);
static void power(const double a[3], double y[3]);
static void quatPlusTheta(const double dtheta[3], double dq[4]);
static void quatmultGoogle(const double q1[4], const double q2[4], double q[4]);
static int rankFromQR(const emxArray_real_T *A);
static double rt_hypotd(double u0, double u1);
static double rt_roundd(double u);

// Function Definitions

//
// Arguments    : const double q[4]
//                double R[9]
// Return Type  : void
//
static void RotFromQuat(const double q[4], double R[9])
{
  R[0] = ((q[0] * q[0] + q[1] * q[1]) - q[2] * q[2]) - q[3] * q[3];
  R[3] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  R[6] = 2.0 * (q[3] * q[1] + q[0] * q[2]);
  R[1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  R[4] = ((q[0] * q[0] - q[1] * q[1]) + q[2] * q[2]) - q[3] * q[3];
  R[7] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  R[2] = 2.0 * (q[3] * q[1] - q[0] * q[2]);
  R[5] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
  R[8] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
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
// Arguments    : emxArray_real_T *b_P_apo
//                emxArray_real_T *x
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[9]
// Return Type  : void
//
static void SLAM_pred(emxArray_real_T *b_P_apo, emxArray_real_T *x, double dt,
                      const double processNoise[4], const double
                      IMU_measurements[9])
{
  int j;
  double curr_meas[6];
  double c;
  double b_processNoise[12];
  double y[12];
  int i10;
  double Q[144];
  double P[441];
  int i11;
  double Phi[441];
  double P1[441];
  double x1[22];
  static const double b_Phi[441] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  double xx[22];
  double b_P[441];
  double c_Phi[441];
  double Phi2[441];
  double P2[441];
  double x2[22];
  double Phi3[441];
  double P3[441];
  double x3[22];
  double Phi4[441];
  double P4[441];
  double x4[22];
  double b_x[22];
  double c_x[4];
  int i12;
  emxArray_real_T *C;
  int i13;
  int cr;
  int ic;
  int br;
  int ar;
  int ib;
  static const signed char iv6[21] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18, 19, 20 };

  int ia;
  emxArray_real_T *b_C;
  int m;
  emxArray_real_T *c_P_apo;

  //  Xv meaning
  //
  //                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
  //  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
  //  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  //  numStatesFeatures=numAnchors*(7+numPointsPerAnchor);
  //  %% Iterative Camera Pose optimization (EKF)
  //  numStates=22;
  if (!b_initialized_not_empty) {
    b_initialized_not_empty = true;
    for (j = 0; j < 6; j++) {
      prev_meas[j] = 0.0;
    }

    //    x(11:13) = -offsetGyr;
    //      x(14:16) = -offsetAcc_body;
    // x(4:7)=q_init;
  }

  // % compute the linearization F of the non linear model f
  for (j = 0; j < 3; j++) {
    curr_meas[j] = IMU_measurements[j];
  }

  for (j = 0; j < 3; j++) {
    curr_meas[j + 3] = IMU_measurements[j + 3];
  }

  c = dt * dt;
  b_processNoise[0] = processNoise[1];
  b_processNoise[1] = processNoise[1];
  b_processNoise[2] = processNoise[1];
  b_processNoise[3] = processNoise[0];
  b_processNoise[4] = processNoise[0];
  b_processNoise[5] = processNoise[0];
  b_processNoise[6] = processNoise[2];
  b_processNoise[7] = processNoise[2];
  b_processNoise[8] = processNoise[2];
  b_processNoise[9] = processNoise[3];
  b_processNoise[10] = processNoise[3];
  b_processNoise[11] = processNoise[3];
  for (i10 = 0; i10 < 12; i10++) {
    y[i10] = b_processNoise[i10] * c;
  }

  memset(&Q[0], 0, 144U * sizeof(double));
  for (j = 0; j < 12; j++) {
    Q[j + 12 * j] = y[j];
  }

  for (i10 = 0; i10 < 21; i10++) {
    for (i11 = 0; i11 < 21; i11++) {
      P[i11 + 21 * i10] = b_P_apo->data[i11 + b_P_apo->size[0] * i10];
    }
  }

  dxdt_dPdt(0.0, dt, prev_meas, curr_meas, *(double (*)[22])&x->data[0], P,
            b_Phi, Q, x1, P1, Phi);
  for (i10 = 0; i10 < 22; i10++) {
    xx[i10] = x->data[i10] + x1[i10] / 2.0;
  }

  c = norm(*(double (*)[4])&xx[3]);
  for (i10 = 0; i10 < 4; i10++) {
    xx[3 + i10] /= c;
  }

  for (i10 = 0; i10 < 441; i10++) {
    b_P[i10] = P[i10] + P1[i10] / 2.0;
    c_Phi[i10] = b_Phi[i10] + Phi[i10] / 2.0;
  }

  dxdt_dPdt(0.5, dt, prev_meas, curr_meas, xx, b_P, c_Phi, Q, x2, P2, Phi2);
  for (i10 = 0; i10 < 22; i10++) {
    xx[i10] = x->data[i10] + x2[i10] / 2.0;
  }

  c = norm(*(double (*)[4])&xx[3]);
  for (i10 = 0; i10 < 4; i10++) {
    xx[3 + i10] /= c;
  }

  for (i10 = 0; i10 < 441; i10++) {
    b_P[i10] = P[i10] + P2[i10] / 2.0;
    c_Phi[i10] = b_Phi[i10] + Phi2[i10] / 2.0;
  }

  dxdt_dPdt(0.5, dt, prev_meas, curr_meas, xx, b_P, c_Phi, Q, x3, P3, Phi3);
  for (i10 = 0; i10 < 22; i10++) {
    xx[i10] = x->data[i10] + x3[i10];
  }

  c = norm(*(double (*)[4])&xx[3]);
  for (i10 = 0; i10 < 4; i10++) {
    xx[3 + i10] /= c;
  }

  for (i10 = 0; i10 < 441; i10++) {
    b_P[i10] = P[i10] + P3[i10];
    c_Phi[i10] = b_Phi[i10] + Phi3[i10];
  }

  dxdt_dPdt(1.0, dt, prev_meas, curr_meas, xx, b_P, c_Phi, Q, x4, P4, Phi4);
  for (i10 = 0; i10 < 22; i10++) {
    b_x[i10] = x->data[i10] + (((x1[i10] + 2.0 * x2[i10]) + 2.0 * x3[i10]) +
      x4[i10]) / 6.0;
  }

  for (i10 = 0; i10 < 22; i10++) {
    x->data[i10] = b_x[i10];
  }

  for (i10 = 0; i10 < 441; i10++) {
    P[i10] += (((P1[i10] + 2.0 * P2[i10]) + 2.0 * P3[i10]) + P4[i10]) / 6.0;
    Phi[i10] = b_Phi[i10] + (((Phi[i10] + 2.0 * Phi2[i10]) + 2.0 * Phi3[i10]) +
      Phi4[i10]) / 6.0;
  }

  c = norm(*(double (*)[4])&x->data[3]);
  for (i10 = 0; i10 < 4; i10++) {
    c_x[i10] = x->data[3 + i10] / c;
  }

  for (i10 = 0; i10 < 4; i10++) {
    x->data[3 + i10] = c_x[i10];
  }

  if (22 > b_P_apo->size[1]) {
    i10 = 0;
    i11 = 0;
  } else {
    i10 = 21;
    i11 = b_P_apo->size[1];
  }

  if (22 > b_P_apo->size[1]) {
    i12 = 0;
  } else {
    i12 = 21;
  }

  emxInit_real_T(&C, 2);
  i13 = C->size[0] * C->size[1];
  C->size[0] = 21;
  emxEnsureCapacity((emxArray__common *)C, i13, (int)sizeof(double));
  i13 = C->size[0] * C->size[1];
  C->size[1] = i11 - i10;
  emxEnsureCapacity((emxArray__common *)C, i13, (int)sizeof(double));
  cr = 21 * (i11 - i10);
  for (i13 = 0; i13 < cr; i13++) {
    C->data[i13] = 0.0;
  }

  if (i11 - i10 == 0) {
  } else {
    j = 21 * ((i11 - i10) - 1);
    for (cr = 0; cr <= j; cr += 21) {
      for (ic = cr; ic + 1 <= cr + 21; ic++) {
        C->data[ic] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= j; cr += 21) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 21; ib++) {
        if (b_P_apo->data[iv6[ib % 21] + b_P_apo->size[0] * (i10 + ib / 21)] !=
            0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 21; ic++) {
            ia++;
            C->data[ic] += b_P_apo->data[ib % 21 + b_P_apo->size[0] * (i10 + ib /
              21)] * Phi[ia - 1];
          }
        }

        ar += 21;
      }

      br += 21;
    }
  }

  cr = C->size[1];
  for (i10 = 0; i10 < cr; i10++) {
    for (i11 = 0; i11 < 21; i11++) {
      b_P_apo->data[i11 + b_P_apo->size[0] * (i12 + i10)] = C->data[i11 +
        C->size[0] * i10];
    }
  }

  emxFree_real_T(&C);
  if (22 > b_P_apo->size[0]) {
    i10 = 0;
    i11 = 0;
  } else {
    i10 = 21;
    i11 = b_P_apo->size[0];
  }

  if (22 > b_P_apo->size[0]) {
    i12 = 0;
  } else {
    i12 = 21;
  }

  for (i13 = 0; i13 < 21; i13++) {
    for (j = 0; j < 21; j++) {
      P1[j + 21 * i13] = Phi[i13 + 21 * j];
    }
  }

  emxInit_real_T(&b_C, 2);
  m = i11 - i10;
  i13 = b_C->size[0] * b_C->size[1];
  b_C->size[0] = i11 - i10;
  b_C->size[1] = 21;
  emxEnsureCapacity((emxArray__common *)b_C, i13, (int)sizeof(double));
  cr = (i11 - i10) * 21;
  for (i13 = 0; i13 < cr; i13++) {
    b_C->data[i13] = 0.0;
  }

  if (i11 - i10 == 0) {
  } else {
    j = (i11 - i10) * 20;
    cr = 0;
    while ((m > 0) && (cr <= j)) {
      i13 = cr + m;
      for (ic = cr; ic + 1 <= i13; ic++) {
        b_C->data[ic] = 0.0;
      }

      cr += m;
    }

    br = 0;
    cr = 0;
    while ((m > 0) && (cr <= j)) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 21; ib++) {
        if (P1[ib] != 0.0) {
          ia = ar;
          i13 = cr + m;
          for (ic = cr; ic + 1 <= i13; ic++) {
            ia++;
            b_C->data[ic] += P1[ib] * b_P_apo->data[(i10 + (ia - 1) % (i11 - i10))
              + b_P_apo->size[0] * ((ia - 1) / (i11 - i10))];
          }
        }

        ar += m;
      }

      br += 21;
      cr += m;
    }
  }

  for (i10 = 0; i10 < 21; i10++) {
    cr = b_C->size[0];
    for (i11 = 0; i11 < cr; i11++) {
      b_P_apo->data[(i12 + i11) + b_P_apo->size[0] * i10] = b_C->data[i11 +
        b_C->size[0] * i10];
    }
  }

  emxFree_real_T(&b_C);
  for (i10 = 0; i10 < 21; i10++) {
    for (i11 = 0; i11 < 21; i11++) {
      b_P_apo->data[i11 + b_P_apo->size[0] * i10] = P[i11 + 21 * i10];
    }
  }

  emxInit_real_T(&c_P_apo, 2);
  i10 = c_P_apo->size[0] * c_P_apo->size[1];
  c_P_apo->size[0] = b_P_apo->size[0];
  c_P_apo->size[1] = b_P_apo->size[1];
  emxEnsureCapacity((emxArray__common *)c_P_apo, i10, (int)sizeof(double));
  cr = b_P_apo->size[1];
  for (i10 = 0; i10 < cr; i10++) {
    j = b_P_apo->size[0];
    for (i11 = 0; i11 < j; i11++) {
      c_P_apo->data[i11 + c_P_apo->size[0] * i10] = (b_P_apo->data[i11 +
        b_P_apo->size[0] * i10] + b_P_apo->data[i10 + b_P_apo->size[0] * i11]) /
        2.0;
    }
  }

  i10 = b_P_apo->size[0] * b_P_apo->size[1];
  b_P_apo->size[0] = c_P_apo->size[0];
  b_P_apo->size[1] = c_P_apo->size[1];
  emxEnsureCapacity((emxArray__common *)b_P_apo, i10, (int)sizeof(double));
  cr = c_P_apo->size[1];
  for (i10 = 0; i10 < cr; i10++) {
    j = c_P_apo->size[0];
    for (i11 = 0; i11 < j; i11++) {
      b_P_apo->data[i11 + b_P_apo->size[0] * i10] = c_P_apo->data[i11 +
        c_P_apo->size[0] * i10];
    }
  }

  emxFree_real_T(&c_P_apo);
  for (j = 0; j < 6; j++) {
    prev_meas[j] = curr_meas[j];
  }

  // % ========================================================================================================================================================================================================================= 
}

//
// EKF_SLAM Update: computes the camerapose p and feature position f in an
//
//  Input arguments:
//    P_apr
// Arguments    : emxArray_real_T *P_apr
//                emxArray_real_T *b_xt
//                const double cameraparams[4]
//                const double updateVect[32]
//                const double z_all[96]
//                double imNoise[3]
//                double numPointsPerAnchor
//                double numAnchors
//                emxArray_real_T *h_u_apo
//                double updateVect_out[32]
//                emxArray_real_T *anchor_u_out
//                emxArray_real_T *anchor_pose_out
// Return Type  : void
//
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams[4], const double updateVect[32], const double z_all[96],
  double imNoise[3], double numPointsPerAnchor, double numAnchors,
  emxArray_real_T *h_u_apo, double updateVect_out[32], emxArray_real_T
  *anchor_u_out, emxArray_real_T *anchor_pose_out)
{
  double numStatesFeatures;
  double f;
  double Cx;
  double Cy;
  double baseline;
  int i14;
  int ib;
  emxArray_real_T *H_xc_tmp;
  emxArray_real_T *H_xm_tmp;
  double R_ci[9];
  double R_iw[9];
  int i15;
  double R_cw[9];
  int br;
  int idx;
  signed char ii_data[32];
  int ii;
  boolean_T exitg2;
  boolean_T guard2 = false;
  int ii_size_idx_0;
  signed char indMeas_data[32];
  double h_u_data[96];
  int z_size_idx_0;
  double z_data[96];
  int k;
  emxArray_real_T *H_iy;
  emxArray_real_T *H_iyc;
  emxArray_int32_T *r1;
  double y;
  double x;
  double x_i;
  double y_i;
  double z_i;
  double fq_cw0;
  double fq_cw1;
  double fq_cw2;
  double fq_cw3;
  double fp[3];
  double b_fq_cw0[4];
  double h_u_To_h_c[9];
  double fR_wc[9];
  double featureOffset;
  double current_anchor_u_idx_0;
  double current_anchor_u_idx_1;
  double m[3];
  double c_xt;
  double d_xt[3];
  double d1;
  double h_ii[3];
  double b_h_ii[3];
  double h_ci[3];
  double h_ui[3];
  double tmp1[3];
  double dv11[9];
  double b_R_cw[9];
  double a[2];
  int ic;
  int ar;
  int ia;
  int tmp_data[96];
  int b_tmp_data[96];
  double dv12[9];
  double dv13[9];
  double c_R_cw[63];
  double b_h_u_To_h_c[63];
  emxArray_real_T *h_u_apr;
  emxArray_real_T *J2;
  emxArray_real_T *b;
  emxArray_real_T *R;
  emxArray_real_T *H;
  double dv14[3];
  double dv15[9];
  int b_m;
  emxArray_real_T *K;
  int i;
  double b_z_all[2];
  emxArray_real_T *b_H_xm_tmp;
  double dv16[4];
  double q_tmp[4];
  double dv17[4];
  double dqtmp[3];
  double qOld_tmp[4];
  double dv18[4];
  double q31;
  double q32;
  double q33;
  double normq;
  emxArray_real_T *Jtmp;
  emxArray_int32_T *r2;
  emxArray_real_T *r3;
  double z_all_data[96];
  double rhoInit;
  double rhoSigma;
  static const signed char y_To_x_c[147] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  int c_tmp_data[32];
  boolean_T exitg1;
  boolean_T guard1 = false;
  double c_fq_cw0[4];
  double b_x_i[3];
  double b_Cx[3];
  signed char d_tmp_data[64];
  double c_Cx[2];

  // interative way
  //    Handels a static number of points but can dynamically asign them to new
  //    klt points.
  //  Xv meaning
  //
  //                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
  //  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
  //  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
  //  (C) Tobias Naegeli naegelit@inf.ethz.ch/ Petri Tanskanen
  //  performance optimized code
  //  copy upd_vector
  memcpy(&updateVect_out[0], &updateVect[0], sizeof(double) << 5);

  //  debug stuff
  // % Iterative Camera Pose optimization (EKF)
  //  numPointsPerAnchor=4;
  //  numAnchors=4;
  numStatesFeatures = numAnchors * (6.0 + numPointsPerAnchor);

  //  numPoints=numPointsPerAnchor*numAnchors;
  //  camera parameters
  f = cameraparams[0];
  Cx = cameraparams[1];
  Cy = cameraparams[2];
  baseline = cameraparams[3];
  if (!c_initialized_not_empty) {
    i14 = anchor_u->size[0];
    anchor_u->size[0] = (int)(numAnchors * 2.0 * numPointsPerAnchor);
    emxEnsureCapacity((emxArray__common *)anchor_u, i14, (int)sizeof(double));
    ib = (int)(numAnchors * 2.0 * numPointsPerAnchor);
    for (i14 = 0; i14 < ib; i14++) {
      anchor_u->data[i14] = 0.0;
    }

    i14 = anchor_pose->size[0];
    anchor_pose->size[0] = (int)(7.0 * numAnchors);
    emxEnsureCapacity((emxArray__common *)anchor_pose, i14, (int)sizeof(double));
    ib = (int)(7.0 * numAnchors);
    for (i14 = 0; i14 < ib; i14++) {
      anchor_pose->data[i14] = 0.0;
    }

    c_initialized_not_empty = true;
  }

  emxInit_real_T(&H_xc_tmp, 2);

  // % ========================================================================================================================================================================================================================= 
  //  The measurement model can be divided in three steps. In the first of
  //  them, fatures referred to the world reference frame W are converted into
  //  Euclidean ones referred to the camera reference frame C. In the case of
  //  an inverse depth feature y=[x,y,z,theta,phi,rho]
  //  h_Cr=R_CW(rho([x,y,z]'-r_wc)+m(theta,phi))
  //  pinhole camera model
  //  h_u=[u_u,u_v]'=[Cx-f/d_x*h_Cx/h_Cz;Cy-f/d_y*h_Cy/h_Cz]
  //  gives us the 2D image coordinates h_u assuming a pure projective model.
  //  in order to cope with the distortions coming from real lenses, we add a
  //  radial distortion model to the ideal undistorted coordinates. the ideal
  //  projective undistorted coordinates h_u=[u_u,v_u]' are recovered from the
  //  real distorted ones h_d. It is assumed that h_u is given.
  i14 = H_xc_tmp->size[0] * H_xc_tmp->size[1];
  H_xc_tmp->size[0] = (int)(3.0 * numAnchors * numPointsPerAnchor);
  H_xc_tmp->size[1] = 21;
  emxEnsureCapacity((emxArray__common *)H_xc_tmp, i14, (int)sizeof(double));
  ib = (int)(3.0 * numAnchors * numPointsPerAnchor) * 21;
  for (i14 = 0; i14 < ib; i14++) {
    H_xc_tmp->data[i14] = 0.0;
  }

  emxInit_real_T(&H_xm_tmp, 2);
  i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
  H_xm_tmp->size[0] = (int)(3.0 * numAnchors * numPointsPerAnchor);
  H_xm_tmp->size[1] = (int)numStatesFeatures;
  emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
  ib = (int)(3.0 * numAnchors * numPointsPerAnchor) * (int)numStatesFeatures;
  for (i14 = 0; i14 < ib; i14++) {
    H_xm_tmp->data[i14] = 0.0;
  }

  //  imu displacement
  b_RotFromQuat(*(double (*)[4])&b_xt->data[16], R_ci);
  b_RotFromQuat(*(double (*)[4])&b_xt->data[3], R_iw);

  //  IMU to global
  for (i14 = 0; i14 < 3; i14++) {
    for (i15 = 0; i15 < 3; i15++) {
      R_cw[i14 + 3 * i15] = 0.0;
      for (br = 0; br < 3; br++) {
        R_cw[i14 + 3 * i15] += R_ci[i14 + 3 * br] * R_iw[br + 3 * i15];
      }
    }
  }

  // global to camera             R_ci * R_iw
  //  R_cw=(R_wc)';
  idx = 0;
  ii = 1;
  exitg2 = false;
  while ((!exitg2) && (ii < 33)) {
    guard2 = false;
    if (updateVect[ii - 1] == 1.0) {
      idx++;
      ii_data[idx - 1] = (signed char)ii;
      if (idx >= 32) {
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
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  if (1 > idx) {
    ib = 0;
  } else {
    ib = idx;
  }

  for (i14 = 0; i14 < ib; i14++) {
    indMeas_data[i14] = ii_data[i14];
  }

  ib = 3 * ii_size_idx_0;
  for (i14 = 0; i14 < ib; i14++) {
    h_u_data[i14] = 0.0;
  }

  z_size_idx_0 = 3 * ii_size_idx_0;
  ib = 3 * ii_size_idx_0;
  for (i14 = 0; i14 < ib; i14++) {
    z_data[i14] = 0.0;
  }

  k = 1;
  emxInit_real_T(&H_iy, 2);
  emxInit_real_T(&H_iyc, 2);
  emxInit_int32_T(&r1, 1);
  while ((k <= ii_size_idx_0) && (!(k > ii_size_idx_0))) {
    i14 = indMeas_data[k - 1] * 3 - 2;
    i15 = indMeas_data[k - 1] * 3;
    if (i14 > i15) {
      i14 = 1;
      i15 = 0;
    }

    br = k * 3 - 2;
    if (br > k * 3) {
      br = 0;
    } else {
      br--;
    }

    ib = i15 - i14;
    for (i15 = 0; i15 <= ib; i15++) {
      z_data[br + i15] = z_all[(i14 + i15) - 1];
    }

    y = ((double)indMeas_data[k - 1] - 1.0) / numPointsPerAnchor;
    x = floor(y);
    x_i = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (6.0 + numPointsPerAnchor)) - 1];
    y_i = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (5.0 + numPointsPerAnchor)) - 1];
    z_i = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (4.0 + numPointsPerAnchor)) - 1];
    fq_cw0 = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (3.0 + numPointsPerAnchor)) - 1];
    fq_cw1 = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (2.0 + numPointsPerAnchor)) - 1];
    fq_cw2 = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (1.0 + numPointsPerAnchor)) - 1];
    fq_cw3 = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) - 1];

    // compute feature derivs
    fp[0] = x_i;
    fp[1] = y_i;
    fp[2] = z_i;
    b_fq_cw0[0] = fq_cw0;
    b_fq_cw0[1] = fq_cw1;
    b_fq_cw0[2] = fq_cw2;
    b_fq_cw0[3] = fq_cw3;
    b_RotFromQuat(b_fq_cw0, h_u_To_h_c);
    for (i14 = 0; i14 < 3; i14++) {
      for (i15 = 0; i15 < 3; i15++) {
        fR_wc[i15 + 3 * i14] = h_u_To_h_c[i14 + 3 * i15];
      }
    }

    //              fR_iw=RotFromQuat([fq_cw0;fq_cw1;fq_cw2;fq_cw3]);
    //              fR_cw=R_ci*fR_iw;
    //              fR_wc=fR_cw';
    featureOffset = b_mod((double)indMeas_data[k - 1] - 1.0, numPointsPerAnchor)
      + 1.0;
    current_anchor_u_idx_0 = anchor_u->data[(int)((((x + 1.0) *
      numPointsPerAnchor - numPointsPerAnchor) + featureOffset) * 2.0 - 1.0) - 1];
    current_anchor_u_idx_1 = anchor_u->data[(int)((((x + 1.0) *
      numPointsPerAnchor - numPointsPerAnchor) + featureOffset) * 2.0) - 1];

    // m = [(Cx-current_anchor_u(1))/f;(Cy-current_anchor_u(2))/f;1];
    m[0] = (-Cx + current_anchor_u_idx_0) / f;
    m[1] = (-Cy + current_anchor_u_idx_1) / f;
    m[2] = 1.0;
    c_xt = b_xt->data[(int)(((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    for (i14 = 0; i14 < 3; i14++) {
      d1 = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        d1 += fR_wc[i14 + 3 * i15] * m[i15];
      }

      d_xt[i14] = c_xt * (fp[i14] - b_xt->data[i14]) + d1;
    }

    for (i14 = 0; i14 < 3; i14++) {
      b_h_ii[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        b_h_ii[i14] += R_iw[i14 + 3 * i15] * d_xt[i15];
      }

      h_ii[i14] = b_h_ii[i14] - b_xt->data[20 + i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      h_ci[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        h_ci[i14] += R_ci[i14 + 3 * i15] * h_ii[i15];
      }
    }

    //  Equation (1)
    //            h_ui=[Cx-f*(h_ci(1)/h_ci(3));
    //                  Cy-f*(h_ci(2)/h_ci(3));
    //                  rho* f*b/h_ci(3)      ];
    h_ui[0] = Cx + f * (h_ci[0] / h_ci[2]);
    h_ui[1] = Cy + f * (h_ci[1] / h_ci[2]);
    h_ui[2] = b_xt->data[(int)(((23.0 + (floor(y) + 1.0) * (7.0 +
      numPointsPerAnchor)) - numPointsPerAnchor) + featureOffset) - 1] * f *
      baseline / h_ci[2];
    if (z_data[2] < -50.0) {
      // invalid disparity
      //              h_u_To_h_c=[-f/h_ci(3), 0,             (h_ci(1)*f)/h_ci(3)^2; 
      //                          0,          -f/h_ci(3),    (h_ci(2)*f)/h_ci(3)^2; 
      //                          0,          0,              0                  ]; 
      h_u_To_h_c[0] = f / h_ci[2];
      h_u_To_h_c[3] = 0.0;
      h_u_To_h_c[6] = -(h_ci[0] * f) / (h_ci[2] * h_ci[2]);
      h_u_To_h_c[1] = 0.0;
      h_u_To_h_c[4] = f / h_ci[2];
      h_u_To_h_c[7] = -(h_ci[1] * f) / (h_ci[2] * h_ci[2]);
      for (i14 = 0; i14 < 3; i14++) {
        imNoise[i14] = 1.0;
        h_u_To_h_c[2 + 3 * i14] = 0.0;
      }
    } else {
      // valid disparity
      for (i14 = 0; i14 < 3; i14++) {
        imNoise[i14] = 1.0;
      }

      //              h_u_To_h_c=[-f/h_ci(3), 0,             (h_ci(1)*f)/h_ci(3)^2; 
      //                          0,          -f/h_ci(3),    (h_ci(2)*f)/h_ci(3)^2; 
      //                          0,          0,              -b*f/(h_ci(3)^2)   ]; 
      h_u_To_h_c[0] = f / h_ci[2];
      h_u_To_h_c[3] = 0.0;
      h_u_To_h_c[6] = -(h_ci[0] * f) / (h_ci[2] * h_ci[2]);
      h_u_To_h_c[1] = 0.0;
      h_u_To_h_c[4] = f / h_ci[2];
      h_u_To_h_c[7] = -(h_ci[1] * f) / (h_ci[2] * h_ci[2]);
      h_u_To_h_c[2] = 0.0;
      h_u_To_h_c[5] = 0.0;
      h_u_To_h_c[8] = -baseline * f / (h_ci[2] * h_ci[2]);
    }

    // %%%%%%%%%%%%%%%%%%%%%%%
    //         %% computation of H(x)%%
    // %%%%%%%%%%%%%%%%%%%%%%%
    //  derivatives of (1) with respect to camerastates
    //  h_ci=R_ci*(R_iw*(rho*(fp-r_wc_pred)+fR_wc*m)-t_ci); % Equation (1)
    // p_iw,q,v,ow,oa,q_ci,t_ci
    //  correct
    // H_ix=[-R_cw*rho,-R_ci*skew(h_ii),zeros(3,9),zeros(3),zeros(3)];
    //  derivatives of (1) with respect to anchorstates
    i14 = H_iy->size[0] * H_iy->size[1];
    H_iy->size[0] = 3;
    H_iy->size[1] = (int)(6.0 + numPointsPerAnchor);
    emxEnsureCapacity((emxArray__common *)H_iy, i14, (int)sizeof(double));
    ib = 3 * (int)(6.0 + numPointsPerAnchor);
    for (i14 = 0; i14 < ib; i14++) {
      H_iy->data[i14] = 0.0;
    }

    for (i14 = 0; i14 < 3; i14++) {
      tmp1[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        tmp1[i14] += fR_wc[i14 + 3 * i15] * m[i15];
      }
    }

    c_xt = b_xt->data[(int)(((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    dv11[0] = 0.0;
    dv11[3] = -tmp1[2];
    dv11[6] = tmp1[1];
    dv11[1] = tmp1[2];
    dv11[4] = 0.0;
    dv11[7] = -tmp1[0];
    dv11[2] = -tmp1[1];
    dv11[5] = tmp1[0];
    dv11[8] = 0.0;
    for (i14 = 0; i14 < 3; i14++) {
      for (i15 = 0; i15 < 3; i15++) {
        b_R_cw[i14 + 3 * i15] = 0.0;
        for (br = 0; br < 3; br++) {
          b_R_cw[i14 + 3 * i15] += R_cw[i14 + 3 * br] * dv11[br + 3 * i15];
        }
      }
    }

    for (i14 = 0; i14 < 3; i14++) {
      for (i15 = 0; i15 < 3; i15++) {
        H_iy->data[i15 + H_iy->size[0] * i14] = R_cw[i15 + 3 * i14] * c_xt;
      }
    }

    for (i14 = 0; i14 < 3; i14++) {
      for (i15 = 0; i15 < 3; i15++) {
        H_iy->data[i15 + H_iy->size[0] * (i14 + 3)] = b_R_cw[i15 + 3 * i14];
      }
    }

    //  should be  [R_cw*rho,-R_cw*hat_tmp1]; but because state is f_wc and not f_cw ther comes a - sign 
    for (i14 = 0; i14 < 3; i14++) {
      h_ii[i14] = fp[i14] - b_xt->data[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      H_iy->data[i14 + H_iy->size[0] * ((int)(6.0 + featureOffset) - 1)] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        H_iy->data[i14 + H_iy->size[0] * ((int)(6.0 + featureOffset) - 1)] +=
          R_cw[i14 + 3 * i15] * h_ii[i15];
      }
    }

    a[1] = H_iy->size[1];
    i14 = H_iyc->size[0] * H_iyc->size[1];
    H_iyc->size[0] = 3;
    emxEnsureCapacity((emxArray__common *)H_iyc, i14, (int)sizeof(double));
    i14 = H_iyc->size[0] * H_iyc->size[1];
    H_iyc->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)H_iyc, i14, (int)sizeof(double));
    ib = 3 * (int)a[1];
    for (i14 = 0; i14 < ib; i14++) {
      H_iyc->data[i14] = 0.0;
    }

    if (H_iy->size[1] == 0) {
    } else {
      ii = 3 * (H_iy->size[1] - 1);
      for (idx = 0; idx <= ii; idx += 3) {
        for (ic = idx + 1; ic <= idx + 3; ic++) {
          H_iyc->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= ii; idx += 3) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 3; ib++) {
          if (H_iy->data[ib] != 0.0) {
            ia = ar;
            for (ic = idx; ic + 1 <= idx + 3; ic++) {
              ia++;
              H_iyc->data[ic] += H_iy->data[ib] * h_u_To_h_c[ia - 1];
            }
          }

          ar += 3;
        }

        br += 3;
      }
    }

    //  nonlinear predicted measurement
    i14 = 3 * k - 2;
    i15 = 3 * k;
    if (i14 > i15) {
      i14 = 0;
      i15 = 0;
    } else {
      i14--;
    }

    ib = i15 - i14;
    for (br = 0; br < ib; br++) {
      tmp_data[br] = i14 + br;
    }

    ib = i15 - i14;
    for (i14 = 0; i14 < ib; i14++) {
      h_u_data[tmp_data[i14]] = h_ui[i14];
    }

    //  stack the matrices for all feature measurements
    i14 = 3 * k - 2;
    i15 = 3 * k;
    if (i14 > i15) {
      i14 = 0;
      i15 = 0;
    } else {
      i14--;
    }

    ib = i15 - i14;
    for (br = 0; br < ib; br++) {
      b_tmp_data[br] = i14 + br;
    }

    c_xt = b_xt->data[(int)(((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    dv12[0] = 0.0;
    dv12[3] = -b_h_ii[2];
    dv12[6] = b_h_ii[1];
    dv12[1] = b_h_ii[2];
    dv12[4] = 0.0;
    dv12[7] = -b_h_ii[0];
    dv12[2] = -b_h_ii[1];
    dv12[5] = b_h_ii[0];
    dv12[8] = 0.0;
    dv13[0] = 0.0;
    dv13[3] = -h_ci[2];
    dv13[6] = h_ci[1];
    dv13[1] = h_ci[2];
    dv13[4] = 0.0;
    dv13[7] = -h_ci[0];
    dv13[2] = -h_ci[1];
    dv13[5] = h_ci[0];
    dv13[8] = 0.0;
    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 3; idx++) {
        c_R_cw[idx + 3 * br] = -R_cw[idx + 3 * br] * c_xt;
      }
    }

    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 3; idx++) {
        c_R_cw[idx + 3 * (br + 3)] = -dv12[idx + 3 * br];
      }
    }

    for (br = 0; br < 9; br++) {
      for (idx = 0; idx < 3; idx++) {
        c_R_cw[idx + 3 * (br + 6)] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 3; idx++) {
        c_R_cw[idx + 3 * (br + 15)] = -dv13[idx + 3 * br];
      }
    }

    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 3; idx++) {
        c_R_cw[idx + 3 * (br + 18)] = -R_ci[idx + 3 * br];
      }
    }

    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 21; idx++) {
        b_h_u_To_h_c[br + 3 * idx] = 0.0;
        for (ii = 0; ii < 3; ii++) {
          b_h_u_To_h_c[br + 3 * idx] += h_u_To_h_c[br + 3 * ii] * c_R_cw[ii + 3 *
            idx];
        }
      }
    }

    idx = i15 - i14;
    for (i14 = 0; i14 < 21; i14++) {
      for (i15 = 0; i15 < idx; i15++) {
        H_xc_tmp->data[b_tmp_data[i15] + H_xc_tmp->size[0] * i14] =
          b_h_u_To_h_c[i15 + idx * i14];
      }
    }

    i14 = 3 * k - 2;
    i15 = 3 * k;
    if (i14 > i15) {
      i14 = 0;
      i15 = 0;
    } else {
      i14--;
    }

    d1 = (6.0 + numPointsPerAnchor) * (x + 1.0) - ((6.0 + numPointsPerAnchor) -
      1.0);
    c_xt = (6.0 + numPointsPerAnchor) * (x + 1.0);
    if (d1 > c_xt) {
      br = 0;
      idx = 0;
    } else {
      br = (int)d1 - 1;
      idx = (int)c_xt;
    }

    ib = i15 - i14;
    for (ii = 0; ii < ib; ii++) {
      b_tmp_data[ii] = i14 + ii;
    }

    ii = r1->size[0];
    r1->size[0] = idx - br;
    emxEnsureCapacity((emxArray__common *)r1, ii, (int)sizeof(int));
    ib = idx - br;
    for (idx = 0; idx < ib; idx++) {
      r1->data[idx] = br + idx;
    }

    idx = i15 - i14;
    ii = r1->size[0];
    for (i14 = 0; i14 < ii; i14++) {
      for (i15 = 0; i15 < idx; i15++) {
        H_xm_tmp->data[b_tmp_data[i15] + H_xm_tmp->size[0] * r1->data[i14]] =
          H_iyc->data[i15 + idx * i14];
      }
    }

    k++;
  }

  emxFree_real_T(&H_iyc);
  emxFree_real_T(&H_iy);
  b_emxInit_real_T(&h_u_apr, 1);
  emxInit_real_T(&J2, 2);
  emxInit_real_T(&b, 2);
  if (ii_size_idx_0 > 0) {
    emxInit_real_T(&R, 2);
    emxInit_real_T(&H, 2);

    //  R=imNoise.^2*eye(numMeas*numMeasDim);
    eye((double)ii_size_idx_0, J2);
    power(imNoise, dv14);
    diag(dv14, dv15);
    kron(J2->data, J2->size, dv15, R);
    ib = ii_size_idx_0 * 3 - 1;
    ar = ii_size_idx_0 * 3;
    ii = H_xm_tmp->size[1] - 1;
    i14 = H->size[0] * H->size[1];
    H->size[0] = ar;
    H->size[1] = ii + 22;
    emxEnsureCapacity((emxArray__common *)H, i14, (int)sizeof(double));
    for (i14 = 0; i14 < 21; i14++) {
      for (i15 = 0; i15 < ar; i15++) {
        H->data[i15 + H->size[0] * i14] = H_xc_tmp->data[i15 + H_xc_tmp->size[0]
          * i14];
      }
    }

    for (i14 = 0; i14 <= ii; i14++) {
      for (i15 = 0; i15 <= ib; i15++) {
        H->data[i15 + H->size[0] * (i14 + 21)] = H_xm_tmp->data[i15 +
          H_xm_tmp->size[0] * i14];
      }
    }

    if (P_apr->size[0] == 1) {
      i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
      H_xm_tmp->size[0] = H->size[0];
      H_xm_tmp->size[1] = P_apr->size[1];
      emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
      ib = H->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = P_apr->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] = 0.0;
          ii = H->size[1];
          for (br = 0; br < ii; br++) {
            H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] += H->data[i14 +
              H->size[0] * br] * P_apr->data[br + P_apr->size[0] * i15];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = P_apr->size[1];
      b_m = H->size[0];
      i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
      H_xm_tmp->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
      i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
      H_xm_tmp->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i14 = 0; i14 < ib; i14++) {
        H_xm_tmp->data[i14] = 0.0;
      }

      if (P_apr->size[1] == 0) {
      } else {
        ii = H->size[0] * (P_apr->size[1] - 1);
        for (idx = 0; idx <= ii; idx += b_m) {
          i14 = idx + b_m;
          for (ic = idx; ic + 1 <= i14; ic++) {
            H_xm_tmp->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += b_m) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (P_apr->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + b_m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                H_xm_tmp->data[ic] += P_apr->data[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
        }
      }
    }

    i14 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i14, (int)sizeof(double));
    ib = H->size[0];
    for (i14 = 0; i14 < ib; i14++) {
      ar = H->size[1];
      for (i15 = 0; i15 < ar; i15++) {
        b->data[i15 + b->size[0] * i14] = H->data[i14 + H->size[0] * i15];
      }
    }

    if (H_xm_tmp->size[1] == 1) {
      i14 = J2->size[0] * J2->size[1];
      J2->size[0] = H_xm_tmp->size[0];
      J2->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)J2, i14, (int)sizeof(double));
      ib = H_xm_tmp->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = b->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          J2->data[i14 + J2->size[0] * i15] = 0.0;
          ii = H_xm_tmp->size[1];
          for (br = 0; br < ii; br++) {
            J2->data[i14 + J2->size[0] * i15] += H_xm_tmp->data[i14 +
              H_xm_tmp->size[0] * br] * b->data[br + b->size[0] * i15];
          }
        }
      }
    } else {
      k = H_xm_tmp->size[1];
      a[0] = (signed char)H_xm_tmp->size[0];
      a[1] = (signed char)b->size[1];
      b_m = H_xm_tmp->size[0];
      i14 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)J2, i14, (int)sizeof(double));
      i14 = J2->size[0] * J2->size[1];
      J2->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)J2, i14, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i14 = 0; i14 < ib; i14++) {
        J2->data[i14] = 0.0;
      }

      ii = H_xm_tmp->size[0] * (b->size[1] - 1);
      for (idx = 0; idx <= ii; idx += b_m) {
        i14 = idx + b_m;
        for (ic = idx; ic + 1 <= i14; ic++) {
          J2->data[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= ii; idx += b_m) {
        ar = 0;
        i14 = br + k;
        for (ib = br; ib + 1 <= i14; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i15 = idx + b_m;
            for (ic = idx; ic + 1 <= i15; ic++) {
              ia++;
              J2->data[ic] += b->data[ib] * H_xm_tmp->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
      }
    }

    i14 = R->size[0] * R->size[1];
    R->size[0] = J2->size[0];
    R->size[1] = J2->size[1];
    emxEnsureCapacity((emxArray__common *)R, i14, (int)sizeof(double));
    ib = J2->size[0] * J2->size[1];
    for (i14 = 0; i14 < ib; i14++) {
      R->data[i14] += J2->data[i14];
    }

    i14 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i14, (int)sizeof(double));
    ib = H->size[0];
    for (i14 = 0; i14 < ib; i14++) {
      ar = H->size[1];
      for (i15 = 0; i15 < ar; i15++) {
        b->data[i15 + b->size[0] * i14] = H->data[i14 + H->size[0] * i15];
      }
    }

    if (P_apr->size[1] == 1) {
      i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
      H_xm_tmp->size[0] = P_apr->size[0];
      H_xm_tmp->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
      ib = P_apr->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = b->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] = 0.0;
          ii = P_apr->size[1];
          for (br = 0; br < ii; br++) {
            H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] += P_apr->data[i14 +
              P_apr->size[0] * br] * b->data[br + b->size[0] * i15];
          }
        }
      }
    } else {
      k = P_apr->size[1];
      a[0] = P_apr->size[0];
      a[1] = b->size[1];
      b_m = P_apr->size[0];
      i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
      H_xm_tmp->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
      i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
      H_xm_tmp->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i14 = 0; i14 < ib; i14++) {
        H_xm_tmp->data[i14] = 0.0;
      }

      if (P_apr->size[0] == 0) {
      } else {
        ii = P_apr->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i14 = idx + b_m;
          for (ic = idx; ic + 1 <= i14; ic++) {
            H_xm_tmp->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + b_m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                H_xm_tmp->data[ic] += b->data[ib] * P_apr->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    emxInit_real_T(&K, 2);
    mrdivide(H_xm_tmp, R, K);

    //  K = p_km1_k*H'*inv(H*p_km1_k*H' + R);
    //  xt=xt; %x_k_k = x_km1_k + K*( z - h );
    for (i14 = 0; i14 < z_size_idx_0; i14++) {
      z_data[i14] -= h_u_data[i14];
    }

    for (i = 1; i - 1 < ii_size_idx_0; i++) {
      ii = i * 3 - 2;
      for (i14 = 0; i14 < 2; i14++) {
        b_z_all[i14] = z_data[i14 + ii];
      }

      ii = i * 3 - 2;
      idx = i * 3 - 2;
      for (i14 = 0; i14 < 2; i14++) {
        for (i15 = 0; i15 < 2; i15++) {
          b_fq_cw0[i15 + (i14 << 1)] = R->data[(i15 + ii) + R->size[0] * (i14 +
            idx)];
        }
      }

      b_mrdivide(b_z_all, b_fq_cw0, a);
      ii = i * 3 - 2;

      //  mahalanobis
      // if mal>55.76
      d1 = 0.0;
      for (i14 = 0; i14 < 2; i14++) {
        d1 += a[i14] * z_data[i14 + ii];
      }

      if (d1 > 70.0) {
        // r(j*2-1:j*2)=[0;0];
        updateVect_out[indMeas_data[i - 1] - 1] = 0.0;
      }
    }

    emxFree_real_T(&R);
    if (z_size_idx_0 == 1) {
      i14 = h_u_apr->size[0];
      h_u_apr->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)h_u_apr, i14, (int)sizeof(double));
      ib = K->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        h_u_apr->data[i14] = 0.0;
        ar = K->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          h_u_apr->data[i14] += K->data[i14 + K->size[0] * i15] * z_data[i15];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      b_m = K->size[0];
      i14 = h_u_apr->size[0];
      h_u_apr->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)h_u_apr, i14, (int)sizeof(double));
      ib = (int)a[0];
      for (i14 = 0; i14 < ib; i14++) {
        h_u_apr->data[i14] = 0.0;
      }

      if (K->size[0] == 0) {
      } else {
        idx = 0;
        while ((b_m > 0) && (idx <= 0)) {
          for (ic = 1; ic <= b_m; ic++) {
            h_u_apr->data[ic - 1] = 0.0;
          }

          idx = b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= 0)) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (z_data[ib] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= b_m; ic++) {
                ia++;
                h_u_apr->data[ic] += z_data[ib] * K->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx = b_m;
        }
      }
    }

    eye(21.0 + numStatesFeatures, H_xm_tmp);
    if (K->size[1] == 1) {
      i14 = J2->size[0] * J2->size[1];
      J2->size[0] = K->size[0];
      J2->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)J2, i14, (int)sizeof(double));
      ib = K->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = H->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          J2->data[i14 + J2->size[0] * i15] = 0.0;
          ii = K->size[1];
          for (br = 0; br < ii; br++) {
            J2->data[i14 + J2->size[0] * i15] += K->data[i14 + K->size[0] * br] *
              H->data[br + H->size[0] * i15];
          }
        }
      }
    } else {
      k = K->size[1];
      a[0] = (unsigned int)K->size[0];
      a[1] = (unsigned int)H->size[1];
      b_m = K->size[0];
      i14 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)J2, i14, (int)sizeof(double));
      i14 = J2->size[0] * J2->size[1];
      J2->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)J2, i14, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i14 = 0; i14 < ib; i14++) {
        J2->data[i14] = 0.0;
      }

      if (K->size[0] == 0) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i14 = idx + b_m;
          for (ic = idx; ic + 1 <= i14; ic++) {
            J2->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + b_m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                J2->data[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    emxFree_real_T(&K);
    emxFree_real_T(&H);
    i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
    emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
    ii = H_xm_tmp->size[0];
    idx = H_xm_tmp->size[1];
    ib = ii * idx;
    for (i14 = 0; i14 < ib; i14++) {
      H_xm_tmp->data[i14] -= J2->data[i14];
    }

    i14 = b->size[0] * b->size[1];
    b->size[0] = P_apr->size[0];
    b->size[1] = P_apr->size[1];
    emxEnsureCapacity((emxArray__common *)b, i14, (int)sizeof(double));
    ib = P_apr->size[0] * P_apr->size[1];
    for (i14 = 0; i14 < ib; i14++) {
      b->data[i14] = P_apr->data[i14];
    }

    if (P_apr->size[0] == 1) {
      emxInit_real_T(&b_H_xm_tmp, 2);
      i14 = b_H_xm_tmp->size[0] * b_H_xm_tmp->size[1];
      b_H_xm_tmp->size[0] = H_xm_tmp->size[0];
      b_H_xm_tmp->size[1] = P_apr->size[1];
      emxEnsureCapacity((emxArray__common *)b_H_xm_tmp, i14, (int)sizeof(double));
      ib = H_xm_tmp->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = P_apr->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          b_H_xm_tmp->data[i14 + b_H_xm_tmp->size[0] * i15] = 0.0;
          ii = H_xm_tmp->size[1];
          for (br = 0; br < ii; br++) {
            b_H_xm_tmp->data[i14 + b_H_xm_tmp->size[0] * i15] += H_xm_tmp->
              data[i14 + H_xm_tmp->size[0] * br] * P_apr->data[br + P_apr->size
              [0] * i15];
          }
        }
      }

      i14 = P_apr->size[0] * P_apr->size[1];
      P_apr->size[0] = b_H_xm_tmp->size[0];
      P_apr->size[1] = b_H_xm_tmp->size[1];
      emxEnsureCapacity((emxArray__common *)P_apr, i14, (int)sizeof(double));
      ib = b_H_xm_tmp->size[1];
      for (i14 = 0; i14 < ib; i14++) {
        ar = b_H_xm_tmp->size[0];
        for (i15 = 0; i15 < ar; i15++) {
          P_apr->data[i15 + P_apr->size[0] * i14] = b_H_xm_tmp->data[i15 +
            b_H_xm_tmp->size[0] * i14];
        }
      }

      emxFree_real_T(&b_H_xm_tmp);
    } else {
      k = H_xm_tmp->size[1];
      a[0] = H_xm_tmp->size[0];
      a[1] = P_apr->size[1];
      b_m = H_xm_tmp->size[0];
      i14 = P_apr->size[0] * P_apr->size[1];
      P_apr->size[0] = (int)a[0];
      P_apr->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)P_apr, i14, (int)sizeof(double));
      ib = (int)a[1];
      for (i14 = 0; i14 < ib; i14++) {
        ar = (int)a[0];
        for (i15 = 0; i15 < ar; i15++) {
          P_apr->data[i15 + P_apr->size[0] * i14] = 0.0;
        }
      }

      if ((H_xm_tmp->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = H_xm_tmp->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i14 = idx + b_m;
          for (ic = idx; ic + 1 <= i14; ic++) {
            P_apr->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + b_m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                P_apr->data[ic] += b->data[ib] * H_xm_tmp->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += k;
          idx += b_m;
        }
      }
    }

    for (i14 = 0; i14 < 3; i14++) {
      d_xt[i14] = b_xt->data[i14] + h_u_apr->data[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      b_xt->data[i14] = d_xt[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      h_ii[i14] = h_u_apr->data[3 + i14];
    }

    quatPlusTheta(h_ii, dv16);
    quatmultGoogle(dv16, *(double (*)[4])&b_xt->data[3], q_tmp);
    for (i14 = 0; i14 < 4; i14++) {
      b_xt->data[3 + i14] = q_tmp[i14];
    }

    for (i14 = 0; i14 < 9; i14++) {
      b_R_cw[i14] = b_xt->data[7 + i14] + h_u_apr->data[6 + i14];
    }

    for (i14 = 0; i14 < 9; i14++) {
      b_xt->data[7 + i14] = b_R_cw[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      h_ii[i14] = h_u_apr->data[15 + i14];
    }

    quatPlusTheta(h_ii, dv17);
    quatmultGoogle(dv17, *(double (*)[4])&b_xt->data[16], q_tmp);
    for (i14 = 0; i14 < 4; i14++) {
      b_xt->data[16 + i14] = q_tmp[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      d_xt[i14] = b_xt->data[20 + i14] + h_u_apr->data[18 + i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      b_xt->data[20 + i14] = d_xt[i14];
    }

    for (ii = 0; ii < (int)numAnchors; ii++) {
      b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (6.0 + numPointsPerAnchor)) - 1] += h_u_apr->data[(int)
        ((21.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) - (5.0 +
          numPointsPerAnchor)) - 1];
      b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (5.0 + numPointsPerAnchor)) - 1] += h_u_apr->data[(int)
        ((21.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) - (4.0 +
          numPointsPerAnchor)) - 1];
      b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (4.0 + numPointsPerAnchor)) - 1] += h_u_apr->data[(int)
        ((21.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) - (3.0 +
          numPointsPerAnchor)) - 1];
      dqtmp[0] = h_u_apr->data[(int)((21.0 + (1.0 + (double)ii) * (6.0 +
        numPointsPerAnchor)) - (2.0 + numPointsPerAnchor)) - 1];
      dqtmp[1] = h_u_apr->data[(int)((21.0 + (1.0 + (double)ii) * (6.0 +
        numPointsPerAnchor)) - (1.0 + numPointsPerAnchor)) - 1];
      dqtmp[2] = h_u_apr->data[(int)((21.0 + (1.0 + (double)ii) * (6.0 +
        numPointsPerAnchor)) - numPointsPerAnchor) - 1];
      qOld_tmp[0] = b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 +
        numPointsPerAnchor)) - (3.0 + numPointsPerAnchor)) - 1];
      qOld_tmp[1] = b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 +
        numPointsPerAnchor)) - (2.0 + numPointsPerAnchor)) - 1];
      qOld_tmp[2] = b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 +
        numPointsPerAnchor)) - (1.0 + numPointsPerAnchor)) - 1];
      qOld_tmp[3] = b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 +
        numPointsPerAnchor)) - numPointsPerAnchor) - 1];
      quatPlusTheta(dqtmp, dv18);
      quatmultGoogle(dv18, qOld_tmp, q_tmp);
      b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (3.0 + numPointsPerAnchor)) - 1] = q_tmp[0];
      b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (2.0 + numPointsPerAnchor)) - 1] = q_tmp[1];
      b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (1.0 + numPointsPerAnchor)) - 1] = q_tmp[2];
      b_xt->data[(int)((23.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - numPointsPerAnchor) - 1] = q_tmp[3];
      b_xt->data[(int)(((23.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                        - numPointsPerAnchor) + 1.0) - 1] += h_u_apr->data[(int)
        (((21.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) -
          numPointsPerAnchor) + 1.0) - 1];

      //              for feat=1:numPointsPerAnchor
      //                  xt(numStatesxt+ank*numStatesPerAnchorxt-(0+numPointsPerAnchor)+feat)=xt(numStatesxt+ank*numStatesPerAnchorxt-(0+numPointsPerAnchor)+feat)+x_apo( numStates+ank*numStatesPerAnchor-(0+numPointsPerAnchor)+feat); 
      //              end
    }
  }

  emxFree_real_T(&H_xc_tmp);

  // ============================================================
  //  normalization of the state vector
  //   x_k_k( 4:7 ) = x_k_k( 4:7 ) / norm( x_k_k( 4:7 ) );
  q31 = b_xt->data[4];
  q32 = b_xt->data[5];
  q33 = b_xt->data[6];
  normq = sqrt(((b_xt->data[3] * b_xt->data[3] + b_xt->data[4] * b_xt->data[4])
                + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6]);

  //  % normalization of the error covariance matrix
  b_xt->data[3] /= normq;
  b_xt->data[4] = q31 / normq;
  b_xt->data[5] = q32 / normq;
  b_xt->data[6] = q33 / normq;

  //  P_apo(4:7,:)=q_norm_To_q*P_apo(4:7,:);
  //  P_apo(:,4:7)=P_apo(:,4:7)*q_norm_To_q';
  // also normalize anchor pose orientation
  //      for na = 1:numAnchors
  //          q30=xt(numStatesxt+na*numStatesPerAnchorxt-(3+numPointsPerAnchor)); 
  //          q31=xt(numStatesxt+na*numStatesPerAnchorxt-(2+numPointsPerAnchor)); 
  //          q32=xt(numStatesxt+na*numStatesPerAnchorxt-(1+numPointsPerAnchor)); 
  //          q33=xt(numStatesxt+na*numStatesPerAnchorxt-(0+numPointsPerAnchor)); 
  //          normq=sqrt(q30^2+q31^2+q32^2+q33^2);
  //
  //
  //          xt(numStatesxt+na*numStatesPerAnchorxt-(3+numPointsPerAnchor))=q30/normq; 
  //          xt(numStatesxt+na*numStatesPerAnchorxt-(2+numPointsPerAnchor))=q31/normq; 
  //          xt(numStatesxt+na*numStatesPerAnchorxt-(1+numPointsPerAnchor))=q32/normq; 
  //          xt(numStatesxt+na*numStatesPerAnchorxt-(0+numPointsPerAnchor))=q33/normq; 
  //
  //      end
  i = 0;
  emxInit_real_T(&Jtmp, 2);
  b_emxInit_int32_T(&r2, 2);
  emxInit_real_T(&r3, 2);
  while (i <= (int)numAnchors - 1) {
    if (updateVect[i] == 2.0) {
      d1 = (1.0 + (double)i) * numPointsPerAnchor * 3.0 - (numPointsPerAnchor *
        3.0 - 1.0);
      c_xt = (1.0 + (double)i) * numPointsPerAnchor * 3.0;
      if (d1 > c_xt) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d1 - 1;
        i15 = (int)c_xt;
      }

      d1 = (1.0 + (double)i) * numPointsPerAnchor * 2.0 - (numPointsPerAnchor *
        2.0 - 1.0);
      c_xt = (1.0 + (double)i) * numPointsPerAnchor * 2.0;
      if (d1 > c_xt) {
        br = 0;
        idx = 0;
      } else {
        br = (int)d1 - 1;
        idx = (int)c_xt;
      }

      ii = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = idx - br;
      emxEnsureCapacity((emxArray__common *)r2, ii, (int)sizeof(int));
      ib = idx - br;
      for (idx = 0; idx < ib; idx++) {
        r2->data[r2->size[0] * idx] = br + idx;
      }

      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        z_all_data[i15] = z_all[i14 + i15];
      }

      for (i15 = 0; i15 < 2; i15++) {
        b_z_all[i15] = z_all_data[i15];
      }

      ib = r2->size[1];
      for (i15 = 0; i15 < ib; i15++) {
        anchor_u->data[r2->data[r2->size[0] * i15]] = b_z_all[(*(int (*)[2])
          r2->size)[0] * i15];
      }

      i15 = h_u_apr->size[0];
      h_u_apr->size[0] = (int)(7.0 + numPointsPerAnchor);
      emxEnsureCapacity((emxArray__common *)h_u_apr, i15, (int)sizeof(double));
      ib = (int)(7.0 + numPointsPerAnchor);
      for (i15 = 0; i15 < ib; i15++) {
        h_u_apr->data[i15] = 0.0;
      }

      for (i15 = 0; i15 < 7; i15++) {
        h_u_apr->data[i15] = b_xt->data[i15];
      }

      // x_f_tmp(5:7)=-xt(5:7);
      //     x_f_tmp(8:8+numPointsPerAnchor-1) = rhoInit * ones(numPointsPerAnchor,1);   
      if (z_all[i14 + 2] > -50.0) {
        rhoInit = z_all[i14 + 2] / (baseline * f);
        rhoSigma = 0.1;
      } else {
        rhoInit = 2.0;
        rhoSigma = 1000.0;
      }

      if (8.0 > (8.0 + numPointsPerAnchor) - 1.0) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = 7;
        i15 = (int)((8.0 + numPointsPerAnchor) - 1.0);
      }

      br = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)r2, br, (int)sizeof(int));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        r2->data[r2->size[0] * i15] = i14 + i15;
      }

      ib = r2->size[0] * r2->size[1];
      for (i14 = 0; i14 < ib; i14++) {
        h_u_apr->data[r2->data[i14]] = rhoInit;
      }

      d1 = (23.0 + (1.0 + (double)i) * (7.0 + numPointsPerAnchor)) - (6.0 +
        numPointsPerAnchor);
      c_xt = 23.0 + (1.0 + (double)i) * (7.0 + numPointsPerAnchor);
      if (d1 > c_xt) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d1 - 1;
        i15 = (int)c_xt;
      }

      br = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)r2, br, (int)sizeof(int));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        r2->data[r2->size[0] * i15] = i14 + i15;
      }

      ib = r2->size[1];
      for (i14 = 0; i14 < ib; i14++) {
        b_xt->data[r2->data[r2->size[0] * i14]] = h_u_apr->data[(*(int (*)[2])
          r2->size)[0] * i14];
      }

      // derivative wrt camera state
      //          y_To_r_wc=[eye(3); zeros(3+numPointsPerAnchor,3)];
      //          y_To_q_wc=[zeros(3,3); eye(3); zeros(numPointsPerAnchor,3)];
      // now create J and multiply with P
      eye(21.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0),
          H_xm_tmp);
      eye((numAnchors - (1.0 + (double)i)) * (6.0 + numPointsPerAnchor), J2);

      // J=zeros(144);
      i14 = r3->size[0] * r3->size[1];
      r3->size[0] = (int)(6.0 + numPointsPerAnchor);
      r3->size[1] = (int)(6.0 + numPointsPerAnchor);
      emxEnsureCapacity((emxArray__common *)r3, i14, (int)sizeof(double));
      ib = (int)(6.0 + numPointsPerAnchor) * (int)(6.0 + numPointsPerAnchor);
      for (i14 = 0; i14 < ib; i14++) {
        r3->data[i14] = 0.0;
      }

      blkdiag(H_xm_tmp, r3, J2, Jtmp);
      d1 = (21.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) - ((6.0 +
        numPointsPerAnchor) - 1.0);
      c_xt = 21.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i);
      if (d1 > c_xt) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d1 - 1;
        i15 = (int)c_xt;
      }

      br = r1->size[0];
      r1->size[0] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)r1, br, (int)sizeof(int));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        r1->data[i15] = i14 + i15;
      }

      idx = r1->size[0];
      for (i14 = 0; i14 < 21; i14++) {
        for (i15 = 0; i15 < idx; i15++) {
          Jtmp->data[r1->data[i15] + Jtmp->size[0] * i14] = y_To_x_c[i15 + idx *
            i14];
        }
      }

      // state covariance
      d1 = (21.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 1.0;
      if (d1 > 21.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d1 - 1;
      }

      i15 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)(21.0 + numAnchors * (6.0 + numPointsPerAnchor));
      J2->size[1] = (int)(6.0 + numPointsPerAnchor);
      emxEnsureCapacity((emxArray__common *)J2, i15, (int)sizeof(double));
      ib = (int)(21.0 + numAnchors * (6.0 + numPointsPerAnchor)) * (int)(6.0 +
        numPointsPerAnchor);
      for (i15 = 0; i15 < ib; i15++) {
        J2->data[i15] = 0.0;
      }

      ib = J2->size[1];
      for (i15 = 0; i15 < ib; i15++) {
        ar = J2->size[0];
        for (br = 0; br < ar; br++) {
          P_apr->data[br + P_apr->size[0] * (i14 + i15)] = J2->data[br +
            J2->size[0] * i15];
        }
      }

      d1 = (21.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 1.0;
      if (d1 > 21.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d1 - 1;
      }

      i15 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)(6.0 + numPointsPerAnchor);
      J2->size[1] = (int)(21.0 + numAnchors * (6.0 + numPointsPerAnchor));
      emxEnsureCapacity((emxArray__common *)J2, i15, (int)sizeof(double));
      ib = (int)(6.0 + numPointsPerAnchor) * (int)(21.0 + numAnchors * (6.0 +
        numPointsPerAnchor));
      for (i15 = 0; i15 < ib; i15++) {
        J2->data[i15] = 0.0;
      }

      ib = J2->size[1];
      for (i15 = 0; i15 < ib; i15++) {
        ar = J2->size[0];
        for (br = 0; br < ar; br++) {
          P_apr->data[(i14 + br) + P_apr->size[0] * i15] = J2->data[br +
            J2->size[0] * i15];
        }
      }

      if ((Jtmp->size[1] == 1) || (P_apr->size[0] == 1)) {
        i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
        H_xm_tmp->size[0] = Jtmp->size[0];
        H_xm_tmp->size[1] = P_apr->size[1];
        emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
        ib = Jtmp->size[0];
        for (i14 = 0; i14 < ib; i14++) {
          ar = P_apr->size[1];
          for (i15 = 0; i15 < ar; i15++) {
            H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] = 0.0;
            ii = Jtmp->size[1];
            for (br = 0; br < ii; br++) {
              H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] += Jtmp->data[i14 +
                Jtmp->size[0] * br] * P_apr->data[br + P_apr->size[0] * i15];
            }
          }
        }
      } else {
        k = Jtmp->size[1];
        a[0] = Jtmp->size[0];
        a[1] = P_apr->size[1];
        b_m = Jtmp->size[0];
        i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
        H_xm_tmp->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
        i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
        H_xm_tmp->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i14 = 0; i14 < ib; i14++) {
          H_xm_tmp->data[i14] = 0.0;
        }

        if ((Jtmp->size[0] == 0) || (P_apr->size[1] == 0)) {
        } else {
          ii = Jtmp->size[0] * (P_apr->size[1] - 1);
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            i14 = idx + b_m;
            for (ic = idx; ic + 1 <= i14; ic++) {
              H_xm_tmp->data[ic] = 0.0;
            }

            idx += b_m;
          }

          br = 0;
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            ar = 0;
            i14 = br + k;
            for (ib = br; ib + 1 <= i14; ib++) {
              if (P_apr->data[ib] != 0.0) {
                ia = ar;
                i15 = idx + b_m;
                for (ic = idx; ic + 1 <= i15; ic++) {
                  ia++;
                  H_xm_tmp->data[ic] += P_apr->data[ib] * Jtmp->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            idx += b_m;
          }
        }
      }

      i14 = b->size[0] * b->size[1];
      b->size[0] = Jtmp->size[1];
      b->size[1] = Jtmp->size[0];
      emxEnsureCapacity((emxArray__common *)b, i14, (int)sizeof(double));
      ib = Jtmp->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = Jtmp->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          b->data[i15 + b->size[0] * i14] = Jtmp->data[i14 + Jtmp->size[0] * i15];
        }
      }

      if ((H_xm_tmp->size[1] == 1) || (b->size[0] == 1)) {
        i14 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = H_xm_tmp->size[0];
        P_apr->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i14, (int)sizeof(double));
        ib = H_xm_tmp->size[0];
        for (i14 = 0; i14 < ib; i14++) {
          ar = b->size[1];
          for (i15 = 0; i15 < ar; i15++) {
            P_apr->data[i14 + P_apr->size[0] * i15] = 0.0;
            ii = H_xm_tmp->size[1];
            for (br = 0; br < ii; br++) {
              P_apr->data[i14 + P_apr->size[0] * i15] += H_xm_tmp->data[i14 +
                H_xm_tmp->size[0] * br] * b->data[br + b->size[0] * i15];
            }
          }
        }
      } else {
        k = H_xm_tmp->size[1];
        a[0] = (unsigned int)H_xm_tmp->size[0];
        a[1] = (unsigned int)b->size[1];
        b_m = H_xm_tmp->size[0];
        i14 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = (int)a[0];
        P_apr->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i14, (int)sizeof(double));
        ib = (int)a[1];
        for (i14 = 0; i14 < ib; i14++) {
          ar = (int)a[0];
          for (i15 = 0; i15 < ar; i15++) {
            P_apr->data[i15 + P_apr->size[0] * i14] = 0.0;
          }
        }

        if ((H_xm_tmp->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = H_xm_tmp->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            i14 = idx + b_m;
            for (ic = idx; ic + 1 <= i14; ic++) {
              P_apr->data[ic] = 0.0;
            }

            idx += b_m;
          }

          br = 0;
          idx = 0;
          while ((b_m > 0) && (idx <= ii)) {
            ar = 0;
            i14 = br + k;
            for (ib = br; ib + 1 <= i14; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i15 = idx + b_m;
                for (ic = idx; ic + 1 <= i15; ic++) {
                  ia++;
                  P_apr->data[ic] += b->data[ib] * H_xm_tmp->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            idx += b_m;
          }
        }
      }

      // inverse depth variance
      // inverse depth variance
      d1 = (21.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 7.0;
      if (d1 > 21.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d1 - 1;
      }

      i15 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)(21.0 + numAnchors * (6.0 + numPointsPerAnchor));
      J2->size[1] = (int)numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)J2, i15, (int)sizeof(double));
      ib = (int)(21.0 + numAnchors * (6.0 + numPointsPerAnchor)) * (int)
        numPointsPerAnchor;
      for (i15 = 0; i15 < ib; i15++) {
        J2->data[i15] = 0.0;
      }

      ib = J2->size[1];
      for (i15 = 0; i15 < ib; i15++) {
        ar = J2->size[0];
        for (br = 0; br < ar; br++) {
          P_apr->data[br + P_apr->size[0] * (i14 + i15)] = J2->data[br +
            J2->size[0] * i15];
        }
      }

      d1 = (21.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 7.0;
      if (d1 > 21.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d1 - 1;
      }

      i15 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)numPointsPerAnchor;
      J2->size[1] = (int)(21.0 + numAnchors * (6.0 + numPointsPerAnchor));
      emxEnsureCapacity((emxArray__common *)J2, i15, (int)sizeof(double));
      ib = (int)numPointsPerAnchor * (int)(21.0 + numAnchors * (6.0 +
        numPointsPerAnchor));
      for (i15 = 0; i15 < ib; i15++) {
        J2->data[i15] = 0.0;
      }

      ib = J2->size[1];
      for (i15 = 0; i15 < ib; i15++) {
        ar = J2->size[0];
        for (br = 0; br < ar; br++) {
          P_apr->data[(i14 + br) + P_apr->size[0] * i15] = J2->data[br +
            J2->size[0] * i15];
        }
      }

      d1 = (21.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 7.0;
      if (d1 > 21.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d1 - 1;
      }

      d1 = (21.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 7.0;
      if (d1 > 21.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i15 = 0;
      } else {
        i15 = (int)d1 - 1;
      }

      c_xt = rhoSigma * rhoSigma;
      eye(numPointsPerAnchor, b);
      ib = b->size[1];
      for (br = 0; br < ib; br++) {
        ar = b->size[0];
        for (idx = 0; idx < ar; idx++) {
          P_apr->data[(i14 + idx) + P_apr->size[0] * (i15 + br)] = c_xt *
            b->data[idx + b->size[0] * br];
        }
      }

      d1 = (1.0 + (double)i) * numPointsPerAnchor - (numPointsPerAnchor - 1.0);
      c_xt = (1.0 + (double)i) * numPointsPerAnchor;
      if (d1 > c_xt) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d1 - 1;
        i15 = (int)c_xt;
      }

      ib = i15 - i14;
      for (br = 0; br < ib; br++) {
        c_tmp_data[br] = i14 + br;
      }

      ib = i15 - i14;
      for (i14 = 0; i14 < ib; i14++) {
        updateVect_out[c_tmp_data[i14]] = 1.0;
      }
    }

    i++;
  }

  emxFree_real_T(&r3);
  emxFree_real_T(&b);
  emxFree_int32_T(&r2);
  emxFree_int32_T(&r1);
  emxFree_real_T(&Jtmp);
  emxFree_real_T(&J2);
  emxFree_real_T(&h_u_apr);
  emxFree_real_T(&H_xm_tmp);
  idx = 0;
  ii = 1;
  exitg1 = false;
  while ((!exitg1) && (ii < 33)) {
    guard1 = false;
    if (updateVect_out[ii - 1] == 1.0) {
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
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  if (1 > idx) {
    ib = 0;
  } else {
    ib = idx;
  }

  for (i14 = 0; i14 < ib; i14++) {
    indMeas_data[i14] = ii_data[i14];
  }

  i14 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(3.0 * numAnchors * numPointsPerAnchor);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i14, (int)sizeof(double));
  ib = (int)(3.0 * numAnchors * numPointsPerAnchor);
  for (i14 = 0; i14 < ib; i14++) {
    h_u_apo->data[i14] = 0.0;
  }

  b_RotFromQuat(*(double (*)[4])&b_xt->data[3], R_cw);
  for (k = 0; k < ii_size_idx_0; k++) {
    x = floor(((double)indMeas_data[k] - 1.0) / numPointsPerAnchor);
    x_i = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (6.0 + numPointsPerAnchor)) - 1];
    y_i = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (5.0 + numPointsPerAnchor)) - 1];
    z_i = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (4.0 + numPointsPerAnchor)) - 1];
    fq_cw0 = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (3.0 + numPointsPerAnchor)) - 1];
    fq_cw1 = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (2.0 + numPointsPerAnchor)) - 1];
    fq_cw2 = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (1.0 + numPointsPerAnchor)) - 1];
    fq_cw3 = b_xt->data[(int)((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) - 1];

    // compute feature derivs
    // compute feature derivs
    featureOffset = b_mod((double)indMeas_data[k] - 1.0, numPointsPerAnchor) +
      1.0;
    current_anchor_u_idx_0 = anchor_u->data[(int)((((x + 1.0) *
      numPointsPerAnchor - numPointsPerAnchor) + featureOffset) * 2.0 - 1.0) - 1];
    current_anchor_u_idx_1 = anchor_u->data[(int)((((x + 1.0) *
      numPointsPerAnchor - numPointsPerAnchor) + featureOffset) * 2.0) - 1];

    // m = [(Cx-current_anchor_u(1))/f;(Cy-current_anchor_u(2))/f;1];
    c_fq_cw0[0] = fq_cw0;
    c_fq_cw0[1] = fq_cw1;
    c_fq_cw0[2] = fq_cw2;
    c_fq_cw0[3] = fq_cw3;
    b_RotFromQuat(c_fq_cw0, h_u_To_h_c);
    c_xt = b_xt->data[(int)(((23.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    b_x_i[0] = x_i;
    b_x_i[1] = y_i;
    b_x_i[2] = z_i;
    b_Cx[0] = (-Cx + current_anchor_u_idx_0) / f;
    b_Cx[1] = (-Cy + current_anchor_u_idx_1) / f;
    b_Cx[2] = 1.0;
    for (i14 = 0; i14 < 3; i14++) {
      d1 = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        d1 += h_u_To_h_c[i15 + 3 * i14] * b_Cx[i15];
      }

      d_xt[i14] = c_xt * (b_x_i[i14] - b_xt->data[i14]) + d1;
    }

    for (i14 = 0; i14 < 3; i14++) {
      h_ci[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        h_ci[i14] += R_cw[i14 + 3 * i15] * d_xt[i15];
      }
    }

    //      h_ui=[Cx-f*(h_ci(1)/h_ci(3));
    //          Cy-f*(h_ci(2)/h_ci(3))];
    i14 = 2 * indMeas_data[k] - 1;
    i15 = 2 * indMeas_data[k];
    if (i14 > i15) {
      i14 = 0;
      i15 = 0;
    } else {
      i14--;
    }

    ib = i15 - i14;
    for (br = 0; br < ib; br++) {
      d_tmp_data[br] = (signed char)(i14 + br);
    }

    c_Cx[0] = Cx + f * (h_ci[0] / h_ci[2]);
    c_Cx[1] = Cy + f * (h_ci[1] / h_ci[2]);
    ib = i15 - i14;
    for (i14 = 0; i14 < ib; i14++) {
      h_u_apo->data[d_tmp_data[i14]] = c_Cx[i14];
    }
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  i14 = anchor_u_out->size[0];
  anchor_u_out->size[0] = anchor_u->size[0];
  emxEnsureCapacity((emxArray__common *)anchor_u_out, i14, (int)sizeof(double));
  ib = anchor_u->size[0];
  for (i14 = 0; i14 < ib; i14++) {
    anchor_u_out->data[i14] = anchor_u->data[i14];
  }

  i14 = anchor_pose_out->size[0];
  anchor_pose_out->size[0] = anchor_pose->size[0];
  emxEnsureCapacity((emxArray__common *)anchor_pose_out, i14, (int)sizeof(double));
  ib = anchor_pose->size[0];
  for (i14 = 0; i14 < ib; i14++) {
    anchor_pose_out->data[i14] = anchor_pose->data[i14];
  }
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_updIT_free()
{
  emxFree_real_T(&anchor_pose);
  emxFree_real_T(&anchor_u);
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_updIT_init()
{
  b_emxInit_real_T(&anchor_pose, 1);
  b_emxInit_real_T(&anchor_u, 1);
}

//
// Arguments    : const double q[4]
//                double R[9]
// Return Type  : void
//
static void b_RotFromQuat(const double q[4], double R[9])
{
  R[0] = ((q[0] * q[0] + q[1] * q[1]) - q[2] * q[2]) - q[3] * q[3];
  R[3] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  R[6] = 2.0 * (q[3] * q[1] + q[0] * q[2]);
  R[1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  R[4] = ((q[0] * q[0] - q[1] * q[1]) + q[2] * q[2]) - q[3] * q[3];
  R[7] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  R[2] = 2.0 * (q[3] * q[1] - q[0] * q[2]);
  R[5] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
  R[8] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
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
// Arguments    : emxArray_int32_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
static void b_emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
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
    if (fabs(r - rt_roundd(r)) <= 2.2204460492503131E-16 * fabs(r)) {
      r = 0.0;
    } else {
      r = (r - floor(r)) * y;
    }
  }

  return r;
}

//
// Arguments    : const double A[2]
//                const double B[4]
//                double y[2]
// Return Type  : void
//
static void b_mrdivide(const double A[2], const double B[4], double y[2])
{
  int r1;
  int r2;
  double a21;
  if (fabs(B[1]) > fabs(B[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = B[r2] / B[r1];
  y[r1] = A[0] / B[r1];
  y[r2] = (A[1] - y[r1] * B[2 + r1]) / (B[2 + r2] - a21 * B[2 + r1]);
  y[r1] -= y[r2] * a21;
}

//
// Arguments    : const emxArray_real_T *varargin_1
//                const emxArray_real_T *varargin_2
//                const emxArray_real_T *varargin_3
//                emxArray_real_T *y
// Return Type  : void
//
static void blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
                    *varargin_2, const emxArray_real_T *varargin_3,
                    emxArray_real_T *y)
{
  int r;
  int c;
  int i7;
  int loop_ub;
  int i8;
  int i9;
  r = (varargin_1->size[0] + varargin_2->size[0]) + varargin_3->size[0];
  c = (varargin_1->size[1] + varargin_2->size[1]) + varargin_3->size[1];
  i7 = y->size[0] * y->size[1];
  y->size[0] = r;
  emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
  i7 = y->size[0] * y->size[1];
  y->size[1] = c;
  emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
  r *= c;
  for (i7 = 0; i7 < r; i7++) {
    y->data[i7] = 0.0;
  }

  if ((varargin_1->size[0] > 0) && (varargin_1->size[1] > 0)) {
    r = varargin_1->size[1];
    for (i7 = 0; i7 < r; i7++) {
      loop_ub = varargin_1->size[0];
      for (i8 = 0; i8 < loop_ub; i8++) {
        y->data[i8 + y->size[0] * i7] = varargin_1->data[i8 + varargin_1->size[0]
          * i7];
      }
    }
  }

  if ((varargin_2->size[0] > 0) && (varargin_2->size[1] > 0)) {
    i7 = varargin_1->size[0] + varargin_2->size[0];
    if (varargin_1->size[0] + 1 > i7) {
      i7 = 1;
    } else {
      i7 = varargin_1->size[0] + 1;
    }

    i8 = varargin_1->size[1] + varargin_2->size[1];
    if (varargin_1->size[1] + 1 > i8) {
      i8 = 1;
    } else {
      i8 = varargin_1->size[1] + 1;
    }

    r = varargin_2->size[1];
    for (c = 0; c < r; c++) {
      loop_ub = varargin_2->size[0];
      for (i9 = 0; i9 < loop_ub; i9++) {
        y->data[((i7 + i9) + y->size[0] * ((i8 + c) - 1)) - 1] =
          varargin_2->data[i9 + varargin_2->size[0] * c];
      }
    }
  }

  r = varargin_1->size[0] + varargin_2->size[0];
  c = varargin_1->size[1] + varargin_2->size[1];
  if ((varargin_3->size[0] > 0) && (varargin_3->size[1] > 0)) {
    i7 = r + varargin_3->size[0];
    if (r + 1 > i7) {
      i7 = 1;
    } else {
      i7 = r + 1;
    }

    i8 = c + varargin_3->size[1];
    if (c + 1 > i8) {
      i8 = 1;
    } else {
      i8 = c + 1;
    }

    r = varargin_3->size[1];
    for (c = 0; c < r; c++) {
      loop_ub = varargin_3->size[0];
      for (i9 = 0; i9 < loop_ub; i9++) {
        y->data[((i7 + i9) + y->size[0] * ((i8 + c) - 1)) - 1] =
          varargin_3->data[i9 + varargin_3->size[0] * c];
      }
    }
  }
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
// linearly interpolate between previous and current measurement to get
// to the corresponding part inside dt (for Runge-Kutta)
// Arguments    : double part
//                double dt
//                const double b_prev_meas[6]
//                const double curr_meas[6]
//                const double x[22]
//                const double P[441]
//                const double Phi[441]
//                const double Q[144]
//                double x_dot[22]
//                double P_dot[441]
//                double Phi_dot[441]
// Return Type  : void
//
static void dxdt_dPdt(double part, double dt, const double b_prev_meas[6], const
                      double curr_meas[6], const double x[22], const double P
                      [441], const double Phi[441], const double Q[144], double
                      x_dot[22], double P_dot[441], double Phi_dot[441])
{
  double meas[6];
  int i;
  double R_cw[9];
  double W[9];
  double w[3];
  double b_meas[3];
  double y[3];
  int i1;
  static const signed char iv1[63] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double F[441];
  static const signed char iv2[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double G[252];
  static const signed char iv3[36] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv4[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv5[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double dv0[16];
  double dv1[16];
  double dv2[4];
  double d0;
  static const double dv3[3] = { 0.0, 0.0, -9.81 };

  double b_G[252];
  double FP[441];
  int i2;
  double b_FP[441];
  double b_F[441];
  for (i = 0; i < 6; i++) {
    meas[i] = b_prev_meas[i] + (curr_meas[i] - b_prev_meas[i]) * part;
  }

  RotFromQuat(*(double (*)[4])&x[3], R_cw);

  // -R_cw* [0;0;-10];
  W[0] = 0.0;
  for (i = 0; i < 3; i++) {
    w[i] = meas[i] + x[i + 10];
    b_meas[i] = meas[3 + i] - x[13 + i];
  }

  W[3] = -w[2];
  W[6] = w[1];
  W[1] = w[2];
  W[4] = 0.0;
  W[7] = -w[0];
  W[2] = -w[1];
  W[5] = w[0];
  W[8] = 0.0;
  for (i = 0; i < 3; i++) {
    y[i] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      y[i] += R_cw[i1 + 3 * i] * b_meas[i1];
    }
  }

  for (i = 0; i < 21; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[i1 + 21 * i] = iv1[i1 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * i) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 3)) + 3] = -W[i1 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 6)) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 9)) + 3] = iv2[i1 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 12)) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 15)) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 18)) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * i) + 6] = 0.0;
    }
  }

  F[69] = 0.0;
  F[90] = -y[2];
  F[111] = y[1];
  F[70] = y[2];
  F[91] = 0.0;
  F[112] = -y[0];
  F[71] = -y[1];
  F[92] = y[0];
  F[113] = 0.0;
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 6)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 9)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 12)) + 6] = -R_cw[i + 3 * i1];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 15)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * (i + 18)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 21; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * i) + 9] = 0.0;
    }

    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * i) + 12] = 0.0;
    }

    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * i) + 15] = 0.0;
    }

    for (i1 = 0; i1 < 3; i1++) {
      F[(i1 + 21 * i) + 18] = 0.0;
    }
  }

  for (i = 0; i < 12; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      G[i1 + 21 * i] = 0.0;
    }

    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * i) + 3] = iv3[i1 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * i) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * (i + 3)) + 6] = -R_cw[i + 3 * i1];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * (i + 6)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * (i + 9)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 12; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * i) + 9] = iv4[i1 + 3 * i];
    }

    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * i) + 12] = iv5[i1 + 3 * i];
    }

    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * i) + 15] = 0.0;
    }

    for (i1 = 0; i1 < 3; i1++) {
      G[(i1 + 21 * i) + 18] = 0.0;
    }
  }

  //     F=[ O,   O,  I,  O,  O;
  //      O,  W,  O, -I,  O;
  //      O, R*A,  O,  O, -R;
  //      O,   O,  O,  O,  O;
  //      O,   O,  O,  O,  O];
  //
  //  G=[ O, O,O,O;
  //     -I, O,O,O;
  //      O,-R,O,O;
  //      O, O,I,O;
  //      O, O,O,I];
  // omega=[0,-w';w,-W];
  // time derivative of the state
  memset(&x_dot[0], 0, 22U * sizeof(double));

  //  position
  dv0[0] = 0.0;
  for (i = 0; i < 3; i++) {
    x_dot[i] = x[i + 7];
    dv0[(i + 1) << 2] = w[i];
  }

  for (i = 0; i < 3; i++) {
    dv0[i + 1] = -w[i];
  }

  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      dv0[(i1 + ((i + 1) << 2)) + 1] = -W[i1 + 3 * i];
    }
  }

  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 4; i1++) {
      dv1[i1 + (i << 2)] = 0.5 * dv0[i1 + (i << 2)];
    }
  }

  for (i = 0; i < 4; i++) {
    dv2[i] = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      dv2[i] += dv1[i + (i1 << 2)] * x[3 + i1];
    }

    x_dot[3 + i] = dv2[i];
  }

  //  rot angle
  for (i = 0; i < 3; i++) {
    b_meas[i] = meas[3 + i] - x[13 + i];
  }

  for (i = 0; i < 3; i++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d0 += R_cw[i1 + 3 * i] * b_meas[i1];
    }

    y[i] = d0 - dv3[i];
  }

  for (i = 0; i < 3; i++) {
    x_dot[7 + i] = y[i];
  }

  for (i = 0; i < 3; i++) {
    x_dot[i + 10] = 0.0;
  }

  //  gyro bias
  for (i = 0; i < 3; i++) {
    x_dot[i + 13] = 0.0;
  }

  //  acc bias
  for (i = 0; i < 22; i++) {
    x_dot[i] *= dt;
  }

  // time derivative of the covariance
  for (i = 0; i < 21; i++) {
    for (i1 = 0; i1 < 21; i1++) {
      FP[i + 21 * i1] = 0.0;
      for (i2 = 0; i2 < 21; i2++) {
        FP[i + 21 * i1] += F[i + 21 * i2] * P[i2 + 21 * i1];
      }
    }

    for (i1 = 0; i1 < 12; i1++) {
      b_G[i + 21 * i1] = 0.0;
      for (i2 = 0; i2 < 12; i2++) {
        b_G[i + 21 * i1] += G[i + 21 * i2] * Q[i2 + 12 * i1];
      }
    }
  }

  for (i = 0; i < 21; i++) {
    for (i1 = 0; i1 < 21; i1++) {
      d0 = 0.0;
      for (i2 = 0; i2 < 12; i2++) {
        d0 += b_G[i + 21 * i2] * G[i1 + 21 * i2];
      }

      b_FP[i + 21 * i1] = (FP[i + 21 * i1] + FP[i1 + 21 * i]) + d0;
    }
  }

  // time derivative of the state transition
  for (i = 0; i < 21; i++) {
    for (i1 = 0; i1 < 21; i1++) {
      P_dot[i1 + 21 * i] = b_FP[i1 + 21 * i] * dt;
      b_F[i + 21 * i1] = 0.0;
      for (i2 = 0; i2 < 21; i2++) {
        b_F[i + 21 * i1] += F[i + 21 * i2] * Phi[i2 + 21 * i1];
      }
    }
  }

  for (i = 0; i < 21; i++) {
    for (i1 = 0; i1 < 21; i1++) {
      Phi_dot[i1 + 21 * i] = b_F[i1 + 21 * i] * dt;
    }
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
  int i19;
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
      i19 = ic0 + ldc * (lastc - 1);
      for (jy = ic0; jy <= i19; jy += ldc) {
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
          i19 = lastv + i;
          for (ia = i; ia + 1 <= i19; ia++) {
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
  int i18;
  int k;
  tau = 0.0;
  if (n <= 0) {
  } else {
    xnorm = b_eml_xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          i18 = (ix0 + n) - 2;
          for (k = ix0; k <= i18; k++) {
            x->data[k - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = b_eml_xnrm2(n - 1, x, ix0);
        xnorm = rt_hypotd(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i18 = (ix0 + n) - 2;
        for (k = ix0; k <= i18; k++) {
          x->data[k - 1] *= *alpha1;
        }

        for (k = 1; k <= knt; k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i18 = (ix0 + n) - 2;
        for (k = ix0; k <= i18; k++) {
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
  int i16;
  int j;
  int mmj;
  int c;
  int i;
  int ix;
  double smax;
  int jA;
  double s;
  int i17;
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
    i16 = m - 1;
  } else {
    i16 = n;
  }

  for (j = 1; j <= i16; j++) {
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

      i17 = c + mmj;
      for (i = c + 1; i + 1 <= i17; i++) {
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
        i17 = mmj + jA;
        for (ijA = 1 + jA; ijA + 1 <= i17; ijA++) {
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
//                int numDimensions
// Return Type  : void
//
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
static void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
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
  emxArray_real_T *r0;
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
    emxInit_real_T(&r0, 2);
    eml_qrsolve(b_B, b_A, r0);
    i3 = y->size[0] * y->size[1];
    y->size[0] = r0->size[1];
    y->size[1] = r0->size[0];
    emxEnsureCapacity((emxArray__common *)y, i3, (int)sizeof(double));
    loop_ub = r0->size[0];
    emxFree_real_T(&b_A);
    emxFree_real_T(&b_B);
    for (i3 = 0; i3 < loop_ub; i3++) {
      A_idx_1 = r0->size[1];
      for (i4 = 0; i4 < A_idx_1; i4++) {
        y->data[i4 + y->size[0] * i3] = r0->data[i3 + r0->size[0] * i4];
      }
    }

    emxFree_real_T(&r0);
  }
}

//
// Arguments    : const double x[4]
// Return Type  : double
//
static double norm(const double x[4])
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
static void quatPlusTheta(const double dtheta[3], double dq[4])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  double theta;
  double B;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 3; k++) {
    absxk = fabs(dtheta[k]);
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
  theta = y * 0.5;
  if (theta < 0.244) {
    dq[0] = 1.0;
    for (k = 0; k < 3; k++) {
      dq[k + 1] = 0.5 * dtheta[k];
    }
  } else {
    dq[0] = cos(theta);
    dq[1] = 0.5 * dtheta[0] * sin(theta) / theta;
    dq[2] = 0.5 * dtheta[1] * sin(theta) / theta;
    dq[3] = 0.5 * dtheta[2] * sin(theta) / theta;
  }

  B = norm(dq);
  for (k = 0; k < 4; k++) {
    dq[k] /= B;
  }
}

//
// Arguments    : const double q1[4]
//                const double q2[4]
//                double q[4]
// Return Type  : void
//
static void quatmultGoogle(const double q1[4], const double q2[4], double q[4])
{
  double L[16];
  double dv4[9];
  int i5;
  int i6;
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double B;
  memset(&L[0], 0, sizeof(double) << 4);
  dv4[0] = 0.0;
  dv4[3] = -q2[3];
  dv4[6] = q2[2];
  dv4[1] = q2[3];
  dv4[4] = 0.0;
  dv4[7] = -q2[1];
  dv4[2] = -q2[2];
  dv4[5] = q2[1];
  dv4[8] = 0.0;
  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      L[(i6 + ((1 + i5) << 2)) + 1] = q2[0] * (double)b[i6 + 3 * i5] - dv4[i6 +
        3 * i5];
    }
  }

  for (i5 = 0; i5 < 4; i5++) {
    L[i5 << 2] = -q2[i5];
  }

  for (i5 = 0; i5 < 4; i5++) {
    L[i5] = q2[i5];
    q[i5] = 0.0;
    for (i6 = 0; i6 < 4; i6++) {
      q[i5] += L[i5 + (i6 << 2)] * q1[i6];
    }
  }

  B = norm(q);
  for (i5 = 0; i5 < 4; i5++) {
    q[i5] /= B;
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
static double rt_hypotd(double u0, double u1)
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
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// SLAM Summary of this function goes here
//    Detailed explanation goes here
// Arguments    : const double updateVect[32]
//                const double z_all[96]
//                const double cameraparams[4]
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[9]
//                const double imNoise[3]
//                double numPointsPerAnchor
//                double numAnchors
//                emxArray_real_T *h_u_apo
//                emxArray_real_T *xt_out
//                double updateVect_out[32]
//                emxArray_real_T *anchor_u_out
//                emxArray_real_T *anchor_pose_out
// Return Type  : void
//
void SLAM(const double updateVect[32], const double z_all[96], const double
          cameraparams[4], double dt, const double processNoise[4], const double
          IMU_measurements[9], const double imNoise[3], double
          numPointsPerAnchor, double numAnchors, emxArray_real_T *h_u_apo,
          emxArray_real_T *xt_out, double updateVect_out[32], emxArray_real_T
          *anchor_u_out, emxArray_real_T *anchor_pose_out)
{
  int i0;
  static const signed char iv0[4] = { 1, 0, 0, 0 };

  int loop_ub;
  static const short y[144] = { 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1000 };

  double b_imNoise[3];
  if (!initialized_not_empty) {
    initialized_not_empty = true;

    //  initial quaternion
    i0 = xt->size[0];
    xt->size[0] = 23 + (int)(numAnchors * (7.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)xt, i0, (int)sizeof(double));
    xt->data[0] = 0.0;
    xt->data[1] = 0.0;
    xt->data[2] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      xt->data[i0 + 3] = iv0[i0];
    }

    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      xt->data[i0 + 10] = -IMU_measurements[i0];
    }

    xt->data[13] = 0.0;
    xt->data[14] = 0.0;
    xt->data[15] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      xt->data[i0 + 16] = iv0[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      xt->data[i0 + 20] = 0.0;
    }

    loop_ub = (int)(numAnchors * (7.0 + numPointsPerAnchor));
    for (i0 = 0; i0 < loop_ub; i0++) {
      xt->data[i0 + 23] = 1.0;
    }

    //  initial real vector
    eye(21.0 + numAnchors * (6.0 + numPointsPerAnchor), P_apo);
    i0 = P_apo->size[0] * P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo, i0, (int)sizeof(double));
    i0 = P_apo->size[0];
    loop_ub = P_apo->size[1];
    loop_ub *= i0;
    for (i0 = 0; i0 < loop_ub; i0++) {
      P_apo->data[i0] = 0.0;
    }

    //  initial error state covariance
    for (i0 = 0; i0 < 12; i0++) {
      for (loop_ub = 0; loop_ub < 12; loop_ub++) {
        P_apo->data[(loop_ub + P_apo->size[0] * (9 + i0)) + 9] = y[loop_ub + 12 *
          i0];
      }
    }
  }

  SLAM_pred(P_apo, xt, dt, processNoise, IMU_measurements);

  // P_apo=P_apr;
  for (i0 = 0; i0 < 3; i0++) {
    b_imNoise[i0] = imNoise[i0];
  }

  SLAM_updIT(P_apo, xt, cameraparams, updateVect, z_all, b_imNoise,
             numPointsPerAnchor, numAnchors, h_u_apo, updateVect_out,
             anchor_u_out, anchor_pose_out);
  i0 = xt_out->size[0];
  xt_out->size[0] = xt->size[0];
  emxEnsureCapacity((emxArray__common *)xt_out, i0, (int)sizeof(double));
  loop_ub = xt->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    xt_out->data[i0] = xt->data[i0];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  c_initialized_not_empty = false;
  b_initialized_not_empty = false;
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
// Arguments    : int numDimensions
//                int *size
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  b_emxInit_real_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : double *data
//                int numDimensions
//                int *size
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateWrapperND_real_T(double *data, int numDimensions, int *
  size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  b_emxInit_real_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = data;
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : double *data
//                int rows
//                int cols
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols)
{
  emxArray_real_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  b_emxInit_real_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = data;
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
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  b_emxInit_real_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
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
//                int numDimensions
// Return Type  : void
//
void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  b_emxInit_real_T(pEmxArray, numDimensions);
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
