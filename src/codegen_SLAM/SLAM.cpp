//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Jun-2015 13:22:06
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
static boolean_T c_initialized_not_empty;
static emxArray_real_T *anchor_u;
static emxArray_real_T *anchor_pose;

// Function Declarations
static void RotFromQuatJ(const double q[4], double R[9]);
static void SLAM_free();
static void SLAM_init();
static void SLAM_pred_euler(const emxArray_real_T *b_P_apo, emxArray_real_T *x,
  double dt, const double processNoise[4], const double IMU_measurements[9],
  double numStatesFeatures, emxArray_real_T *P_apr);
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams[4], const double updateVect[32], const double z_all[96],
  double imNoise[3], double numPointsPerAnchor, double numAnchors, const double
  za[3], emxArray_real_T *h_u_apo, double updateVect_out[32], emxArray_real_T
  *anchor_u_out, emxArray_real_T *anchor_pose_out);
static void SLAM_updIT_free();
static void SLAM_updIT_init();
static void b_blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
                      *varargin_2, const emxArray_real_T *varargin_3,
                      emxArray_real_T *y);
static void b_diag(const double v[3], double d[9]);
static double b_eml_matlab_zlarfg();
static double b_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void b_eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0);
static void b_eml_xtrsm(int m, int n, const emxArray_real_T *A, int lda,
  emxArray_real_T *B, int ldb);
static void b_emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void b_eye(double I[144]);
static double b_mod(double x, double y);
static void b_mrdivide(const double A[2], const double B[4], double y[2]);
static double b_norm(const double x[4]);
static void blkdiag(const emxArray_real_T *varargin_1, emxArray_real_T *y);
static double c_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void diag(const double v[9], double d[81]);
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
static double norm(const double x[3]);
static void power(const double a[3], double y[3]);
static void quatPlusThetaJ(const double dtheta[3], double dq[4]);
static void quatmultJ(const double q[4], const double p[4], double qp[4]);
static int rankFromQR(const emxArray_real_T *A);
static double rt_hypotd(double u0, double u1);
static double rt_roundd(double u);

// Function Definitions

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
// Arguments    : const emxArray_real_T *b_P_apo
//                emxArray_real_T *x
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[9]
//                double numStatesFeatures
//                emxArray_real_T *P_apr
// Return Type  : void
//
static void SLAM_pred_euler(const emxArray_real_T *b_P_apo, emxArray_real_T *x,
  double dt, const double processNoise[4], const double IMU_measurements[9],
  double numStatesFeatures, emxArray_real_T *P_apr)
{
  int i12;
  static const signed char iv1[4] = { 0, 0, 0, 1 };

  int i;
  double curr_meas[6];
  double c;
  double b_processNoise[9];
  double c_processNoise[9];
  double Q[81];
  double R_cw[9];
  double w[3];
  emxArray_real_T *F;
  double dv6[144];
  double dv7[9];
  double dv8[144];
  int i13;
  static const signed char iv2[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv3[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  int cr;
  emxArray_real_T *G;
  int ar;
  static const signed char iv4[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv5[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  emxArray_real_T *y;
  int br;
  int k;
  unsigned int unnamed_idx_0;
  unsigned int unnamed_idx_1;
  int m;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *b;
  emxArray_real_T *b_y;
  emxArray_real_T *b_b;
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
  static const double dv9[3] = { 0.0, 0.0, 9.81 };

  emxArray_real_T *b_P_apr;

  //  Xv meaning
  //
  //                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
  //  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
  //  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  //  numStatesFeatures=numAnchors*(7+numPointsPerAnchor);
  //  %% Iterative Camera Pose optimization (EKF)
  //  numStates=16;
  if (!b_initialized_not_empty) {
    b_initialized_not_empty = true;

    //      z_n_b=(-za(1:3)/norm(za(1:3)));
    //      m_n_b = [1;0;0];
    //      y_n_b=cross(z_n_b,m_n_b);
    //      y_n_b=y_n_b./norm(y_n_b);
    //      x_n_b=(cross(y_n_b,z_n_b));
    //      x_n_b=x_n_b./norm(x_n_b);
    //
    //      R_iw_init=[x_n_b,y_n_b,z_n_b];
    //  R_iw_init;
    //  wtmpm=logm(R_iw_init);
    //  wtmp(1)=wtmpm(3,2);
    //  wtmp(2)=wtmpm(1,3);
    //  wtmp(3)=wtmpm(2,1);
    //   x(4:7)=quatPlusThetaJ(-wtmp')
    //
    //   q_init = qGetQ( R_iw_init );
    for (i12 = 0; i12 < 4; i12++) {
      x->data[3 + i12] = iv1[i12];
    }
  }

  // % compute the linearization F of the non linear model f
  for (i = 0; i < 3; i++) {
    curr_meas[i] = IMU_measurements[i];
  }

  for (i = 0; i < 3; i++) {
    curr_meas[i + 3] = IMU_measurements[i + 3];
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
  for (i12 = 0; i12 < 9; i12++) {
    c_processNoise[i12] = b_processNoise[i12] * c;
  }

  diag(c_processNoise, Q);
  RotFromQuatJ(*(double (*)[4])&x->data[3], R_cw);

  //  want R_wi imu to world
  // ===========ACC=====================================
  for (i12 = 0; i12 < 3; i12++) {
    w[i12] = curr_meas[i12] - x->data[10 + i12];
  }

  emxInit_real_T(&F, 2);
  eye(12.0 + numStatesFeatures, F);
  b_eye(dv6);
  dv7[0] = 0.0;
  dv7[3] = -w[2];
  dv7[6] = w[1];
  dv7[1] = w[2];
  dv7[4] = 0.0;
  dv7[7] = -w[0];
  dv7[2] = -w[1];
  dv7[5] = w[0];
  dv7[8] = 0.0;
  for (i12 = 0; i12 < 12; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      dv8[i13 + 12 * i12] = iv2[i13 + 3 * i12];
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      dv8[(i13 + 12 * i12) + 3] = 0.0;
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      dv8[(i13 + 12 * (i12 + 3)) + 3] = -dv7[i13 + 3 * i12];
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      dv8[(i13 + 12 * (i12 + 6)) + 3] = 0.0;
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      dv8[(i13 + 12 * (i12 + 9)) + 3] = iv3[i13 + 3 * i12];
    }
  }

  for (i12 = 0; i12 < 12; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      dv8[(i13 + 12 * i12) + 6] = 0.0;
    }

    for (i13 = 0; i13 < 3; i13++) {
      dv8[(i13 + 12 * i12) + 9] = 0.0;
    }
  }

  for (i12 = 0; i12 < 12; i12++) {
    for (i13 = 0; i13 < 12; i13++) {
      F->data[i13 + F->size[0] * i12] = dv6[i13 + 12 * i12] + dv8[i13 + 12 * i12]
        * dt;
    }
  }

  if ((0 == F->size[0]) || (0 == F->size[1])) {
    cr = 0;
  } else {
    i = F->size[0];
    cr = F->size[1];
    if (i >= cr) {
      cr = i;
    }
  }

  emxInit_real_T(&G, 2);
  i12 = G->size[0] * G->size[1];
  G->size[0] = cr;
  G->size[1] = 9;
  emxEnsureCapacity((emxArray__common *)G, i12, (int)sizeof(double));
  ar = cr * 9;
  for (i12 = 0; i12 < ar; i12++) {
    G->data[i12] = 0.0;
  }

  for (i12 = 0; i12 < 9; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[i13 + G->size[0] * i12] = 0.0;
    }
  }

  for (i12 = 0; i12 < 9; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * i12) + 3] = iv4[i13 + 3 * i12];
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * i12) + 6] = 0.0;
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * (i12 + 3)) + 6] = -R_cw[i13 + 3 * i12];
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * (i12 + 6)) + 6] = 0.0;
    }
  }

  for (i12 = 0; i12 < 9; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * i12) + 9] = iv5[i13 + 3 * i12];
    }
  }

  emxInit_real_T(&y, 2);
  if ((F->size[1] == 1) || (b_P_apo->size[0] == 1)) {
    i12 = y->size[0] * y->size[1];
    y->size[0] = F->size[0];
    y->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)y, i12, (int)sizeof(double));
    ar = F->size[0];
    for (i12 = 0; i12 < ar; i12++) {
      i = b_P_apo->size[1];
      for (i13 = 0; i13 < i; i13++) {
        y->data[i12 + y->size[0] * i13] = 0.0;
        cr = F->size[1];
        for (br = 0; br < cr; br++) {
          y->data[i12 + y->size[0] * i13] += F->data[i12 + F->size[0] * br] *
            b_P_apo->data[br + b_P_apo->size[0] * i13];
        }
      }
    }
  } else {
    k = F->size[1];
    unnamed_idx_0 = (unsigned int)F->size[0];
    unnamed_idx_1 = (unsigned int)b_P_apo->size[1];
    m = F->size[0];
    i12 = y->size[0] * y->size[1];
    y->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)y, i12, (int)sizeof(double));
    i12 = y->size[0] * y->size[1];
    y->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, i12, (int)sizeof(double));
    ar = (int)unnamed_idx_0 * (int)unnamed_idx_1;
    for (i12 = 0; i12 < ar; i12++) {
      y->data[i12] = 0.0;
    }

    if ((F->size[0] == 0) || (b_P_apo->size[1] == 0)) {
    } else {
      i = F->size[0] * (b_P_apo->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= i)) {
        i12 = cr + m;
        for (ic = cr; ic + 1 <= i12; ic++) {
          y->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= i)) {
        ar = 0;
        i12 = br + k;
        for (ib = br; ib + 1 <= i12; ib++) {
          if (b_P_apo->data[ib] != 0.0) {
            ia = ar;
            i13 = cr + m;
            for (ic = cr; ic + 1 <= i13; ic++) {
              ia++;
              y->data[ic] += b_P_apo->data[ib] * F->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  emxInit_real_T(&b, 2);
  i12 = b->size[0] * b->size[1];
  b->size[0] = F->size[1];
  b->size[1] = F->size[0];
  emxEnsureCapacity((emxArray__common *)b, i12, (int)sizeof(double));
  ar = F->size[0];
  for (i12 = 0; i12 < ar; i12++) {
    i = F->size[1];
    for (i13 = 0; i13 < i; i13++) {
      b->data[i13 + b->size[0] * i12] = F->data[i12 + F->size[0] * i13];
    }
  }

  emxFree_real_T(&F);
  if ((y->size[1] == 1) || (b->size[0] == 1)) {
    i12 = P_apr->size[0] * P_apr->size[1];
    P_apr->size[0] = y->size[0];
    P_apr->size[1] = b->size[1];
    emxEnsureCapacity((emxArray__common *)P_apr, i12, (int)sizeof(double));
    ar = y->size[0];
    for (i12 = 0; i12 < ar; i12++) {
      i = b->size[1];
      for (i13 = 0; i13 < i; i13++) {
        P_apr->data[i12 + P_apr->size[0] * i13] = 0.0;
        cr = y->size[1];
        for (br = 0; br < cr; br++) {
          P_apr->data[i12 + P_apr->size[0] * i13] += y->data[i12 + y->size[0] *
            br] * b->data[br + b->size[0] * i13];
        }
      }
    }
  } else {
    k = y->size[1];
    unnamed_idx_0 = (unsigned int)y->size[0];
    unnamed_idx_1 = (unsigned int)b->size[1];
    m = y->size[0];
    i12 = P_apr->size[0] * P_apr->size[1];
    P_apr->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)P_apr, i12, (int)sizeof(double));
    i12 = P_apr->size[0] * P_apr->size[1];
    P_apr->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)P_apr, i12, (int)sizeof(double));
    ar = (int)unnamed_idx_0 * (int)unnamed_idx_1;
    for (i12 = 0; i12 < ar; i12++) {
      P_apr->data[i12] = 0.0;
    }

    if ((y->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      i = y->size[0] * (b->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= i)) {
        i12 = cr + m;
        for (ic = cr; ic + 1 <= i12; ic++) {
          P_apr->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= i)) {
        ar = 0;
        i12 = br + k;
        for (ib = br; ib + 1 <= i12; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            i13 = cr + m;
            for (ic = cr; ic + 1 <= i13; ic++) {
              ia++;
              P_apr->data[ic] += b->data[ib] * y->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  emxFree_real_T(&b);
  emxInit_real_T(&b_y, 2);
  unnamed_idx_0 = (unsigned int)G->size[0];
  m = G->size[0];
  i12 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = (int)unnamed_idx_0;
  b_y->size[1] = 9;
  emxEnsureCapacity((emxArray__common *)b_y, i12, (int)sizeof(double));
  ar = (int)unnamed_idx_0 * 9;
  for (i12 = 0; i12 < ar; i12++) {
    b_y->data[i12] = 0.0;
  }

  if (G->size[0] == 0) {
  } else {
    i = G->size[0] << 3;
    cr = 0;
    while ((m > 0) && (cr <= i)) {
      i12 = cr + m;
      for (ic = cr; ic + 1 <= i12; ic++) {
        b_y->data[ic] = 0.0;
      }

      cr += m;
    }

    br = 0;
    cr = 0;
    while ((m > 0) && (cr <= i)) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 9; ib++) {
        if (Q[ib] != 0.0) {
          ia = ar;
          i12 = cr + m;
          for (ic = cr; ic + 1 <= i12; ic++) {
            ia++;
            b_y->data[ic] += Q[ib] * G->data[ia - 1];
          }
        }

        ar += m;
      }

      br += 9;
      cr += m;
    }
  }

  emxInit_real_T(&b_b, 2);
  i12 = b_b->size[0] * b_b->size[1];
  b_b->size[0] = 9;
  b_b->size[1] = G->size[0];
  emxEnsureCapacity((emxArray__common *)b_b, i12, (int)sizeof(double));
  ar = G->size[0];
  for (i12 = 0; i12 < ar; i12++) {
    for (i13 = 0; i13 < 9; i13++) {
      b_b->data[i13 + b_b->size[0] * i12] = G->data[i12 + G->size[0] * i13];
    }
  }

  emxFree_real_T(&G);
  unnamed_idx_0 = (unsigned int)b_y->size[0];
  unnamed_idx_1 = (unsigned int)b_b->size[1];
  m = b_y->size[0];
  i12 = y->size[0] * y->size[1];
  y->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)y, i12, (int)sizeof(double));
  i12 = y->size[0] * y->size[1];
  y->size[1] = (int)unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)y, i12, (int)sizeof(double));
  ar = (int)unnamed_idx_0 * (int)unnamed_idx_1;
  for (i12 = 0; i12 < ar; i12++) {
    y->data[i12] = 0.0;
  }

  if ((b_y->size[0] == 0) || (b_b->size[1] == 0)) {
  } else {
    i = b_y->size[0] * (b_b->size[1] - 1);
    cr = 0;
    while ((m > 0) && (cr <= i)) {
      i12 = cr + m;
      for (ic = cr; ic + 1 <= i12; ic++) {
        y->data[ic] = 0.0;
      }

      cr += m;
    }

    br = 0;
    cr = 0;
    while ((m > 0) && (cr <= i)) {
      ar = 0;
      for (ib = br; ib + 1 <= br + 9; ib++) {
        if (b_b->data[ib] != 0.0) {
          ia = ar;
          i12 = cr + m;
          for (ic = cr; ic + 1 <= i12; ic++) {
            ia++;
            y->data[ic] += b_b->data[ib] * b_y->data[ia - 1];
          }
        }

        ar += m;
      }

      br += 9;
      cr += m;
    }
  }

  emxFree_real_T(&b_b);
  emxFree_real_T(&b_y);
  i12 = P_apr->size[0] * P_apr->size[1];
  emxEnsureCapacity((emxArray__common *)P_apr, i12, (int)sizeof(double));
  i = P_apr->size[0];
  cr = P_apr->size[1];
  ar = i * cr;
  for (i12 = 0; i12 < ar; i12++) {
    P_apr->data[i12] += y->data[i12] * dt;
  }

  emxFree_real_T(&y);
  for (i12 = 0; i12 < 3; i12++) {
    b_x[i12] = x->data[i12] + x->data[7 + i12] * dt;
  }

  for (i12 = 0; i12 < 3; i12++) {
    x->data[i12] = b_x[i12];
  }

  // +1/2*dt^2*(R_cw'*a-[0;0;-9.81]);             % position
  for (i = 0; i < 3; i++) {
    b_w[i] = w[i] * dt;
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
  for (i12 = 0; i12 < 4; i12++) {
    s_x[i12] = 0.0;
    for (i13 = 0; i13 < 4; i13++) {
      s_x[i12] += r_x[i12 + (i13 << 2)] * b_dq[i13];
    }
  }

  for (i12 = 0; i12 < 4; i12++) {
    x->data[3 + i12] = s_x[i12];
  }

  c = b_norm(*(double (*)[4])&x->data[3]);
  for (i12 = 0; i12 < 4; i12++) {
    s_x[i12] = x->data[3 + i12] / c;
  }

  for (i12 = 0; i12 < 4; i12++) {
    x->data[3 + i12] = s_x[i12];
  }

  for (i12 = 0; i12 < 3; i12++) {
    c = 0.0;
    for (i13 = 0; i13 < 3; i13++) {
      c += R_cw[i13 + 3 * i12] * curr_meas[3 + i13];
    }

    b_w[i12] = c - dv9[i12];
  }

  for (i12 = 0; i12 < 3; i12++) {
    b_x[i12] = x->data[7 + i12] + b_w[i12] * dt;
  }

  for (i12 = 0; i12 < 3; i12++) {
    x->data[7 + i12] = b_x[i12];
  }

  emxInit_real_T(&b_P_apr, 2);

  //  velocity
  i12 = b_P_apr->size[0] * b_P_apr->size[1];
  b_P_apr->size[0] = P_apr->size[0];
  b_P_apr->size[1] = P_apr->size[1];
  emxEnsureCapacity((emxArray__common *)b_P_apr, i12, (int)sizeof(double));
  ar = P_apr->size[1];
  for (i12 = 0; i12 < ar; i12++) {
    i = P_apr->size[0];
    for (i13 = 0; i13 < i; i13++) {
      b_P_apr->data[i13 + b_P_apr->size[0] * i12] = (P_apr->data[i13 +
        P_apr->size[0] * i12] + P_apr->data[i12 + P_apr->size[0] * i13]) / 2.0;
    }
  }

  i12 = P_apr->size[0] * P_apr->size[1];
  P_apr->size[0] = b_P_apr->size[0];
  P_apr->size[1] = b_P_apr->size[1];
  emxEnsureCapacity((emxArray__common *)P_apr, i12, (int)sizeof(double));
  ar = b_P_apr->size[1];
  for (i12 = 0; i12 < ar; i12++) {
    i = b_P_apr->size[0];
    for (i13 = 0; i13 < i; i13++) {
      P_apr->data[i13 + P_apr->size[0] * i12] = b_P_apr->data[i13 +
        b_P_apr->size[0] * i12];
    }
  }

  emxFree_real_T(&b_P_apr);
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
//                const double za[3]
//                emxArray_real_T *h_u_apo
//                double updateVect_out[32]
//                emxArray_real_T *anchor_u_out
//                emxArray_real_T *anchor_pose_out
// Return Type  : void
//
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams[4], const double updateVect[32], const double z_all[96],
  double imNoise[3], double numPointsPerAnchor, double numAnchors, const double
  za[3], emxArray_real_T *h_u_apo, double updateVect_out[32], emxArray_real_T
  *anchor_u_out, emxArray_real_T *anchor_pose_out)
{
  double h_rz[3];
  int i;
  static const signed char b[3] = { 0, 0, 1 };

  double numStatesFeatures;
  double f;
  double Cx;
  double Cy;
  double baseline;
  int i14;
  int ib;
  emxArray_real_T *H_xc_tmp;
  emxArray_real_T *H_xm_tmp;
  double R_cw[9];
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
  emxArray_int32_T *r1;
  int i15;
  int br;
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
  double featureOffset;
  double current_anchor_u_idx_0;
  double current_anchor_u_idx_1;
  double B;
  double b_fq_cw0[4];
  double h_u_To_h_c[9];
  double c_xt;
  double b_h_rz[3];
  double d_xt[3];
  double d0;
  double h_ci[3];
  double h_ui[3];
  int tmp_data[96];
  int b_tmp_data[96];
  double b_R_cw[36];
  double b_h_u_To_h_c[36];
  double c_R_cw[21];
  double c_h_u_To_h_c[21];
  emxArray_real_T *Hg;
  emxArray_real_T *h_u_apr;
  emxArray_real_T *J2;
  emxArray_real_T *b_b;
  emxArray_real_T *R;
  emxArray_real_T *r2;
  double dv10[3];
  double dv11[9];
  emxArray_real_T *b_R;
  emxArray_real_T *H;
  int ar;
  double a[2];
  int m;
  int ic;
  int ia;
  emxArray_real_T *K;
  double b_z_data[99];
  double b_h_u_data[99];
  double r_data[99];
  double b_z_all[2];
  emxArray_real_T *b_H_xm_tmp;
  double dv12[4];
  double q_tmp[4];
  double e_xt[6];
  double dqtmp[3];
  double qOld_tmp[4];
  double dv13[4];
  double q31;
  double q32;
  double q33;
  double normq;
  emxArray_real_T *Jtmp;
  emxArray_int32_T *r3;
  emxArray_real_T *r4;
  double rhoInit;
  double rhoSigma;
  static const signed char y_To_x_c[84] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  int c_tmp_data[32];
  boolean_T exitg1;
  boolean_T guard1 = false;
  double c_fq_cw0[4];
  double b_x_i[3];
  signed char d_tmp_data[64];
  double b_Cx[2];

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
  for (i = 0; i < 3; i++) {
    h_rz[i] = b[i];
  }

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
  i14 = H_xc_tmp->size[0] * H_xc_tmp->size[1];
  H_xc_tmp->size[0] = (int)(3.0 * numAnchors * numPointsPerAnchor);
  H_xc_tmp->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)H_xc_tmp, i14, (int)sizeof(double));
  ib = (int)(3.0 * numAnchors * numPointsPerAnchor) * 12;
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

  RotFromQuatJ(*(double (*)[4])&b_xt->data[3], R_cw);

  //  Camera from
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

  i = 3 * ii_size_idx_0;
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
    x_i = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (6.0 + numPointsPerAnchor)) - 1];
    y_i = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (5.0 + numPointsPerAnchor)) - 1];
    z_i = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (4.0 + numPointsPerAnchor)) - 1];
    fq_cw0 = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (3.0 + numPointsPerAnchor)) - 1];
    fq_cw1 = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (2.0 + numPointsPerAnchor)) - 1];
    fq_cw2 = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (1.0 + numPointsPerAnchor)) - 1];
    fq_cw3 = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) - 1];
    fp[0] = x_i;
    fp[1] = y_i;
    fp[2] = z_i;
    featureOffset = b_mod((double)indMeas_data[k - 1] - 1.0, numPointsPerAnchor)
      + 1.0;
    current_anchor_u_idx_0 = anchor_u->data[(int)((((x + 1.0) *
      numPointsPerAnchor - numPointsPerAnchor) + featureOffset) * 2.0 - 1.0) - 1];
    current_anchor_u_idx_1 = anchor_u->data[(int)((((x + 1.0) *
      numPointsPerAnchor - numPointsPerAnchor) + featureOffset) * 2.0) - 1];
    h_rz[0] = (-Cx + current_anchor_u_idx_0) / f;
    h_rz[1] = (-Cy + current_anchor_u_idx_1) / f;
    h_rz[2] = 1.0;
    B = norm(h_rz);
    b_fq_cw0[0] = fq_cw0;
    b_fq_cw0[1] = fq_cw1;
    b_fq_cw0[2] = fq_cw2;
    b_fq_cw0[3] = fq_cw3;
    RotFromQuatJ(b_fq_cw0, h_u_To_h_c);
    c_xt = b_xt->data[(int)(((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    for (i14 = 0; i14 < 3; i14++) {
      b_h_rz[i14] = h_rz[i14] / B;
    }

    for (i14 = 0; i14 < 3; i14++) {
      d0 = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        d0 += h_u_To_h_c[i15 + 3 * i14] * b_h_rz[i15];
      }

      d_xt[i14] = c_xt * (fp[i14] - b_xt->data[i14]) + d0;
    }

    for (i14 = 0; i14 < 3; i14++) {
      h_ci[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        h_ci[i14] += R_cw[i14 + 3 * i15] * d_xt[i15];
      }

      h_rz[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        h_rz[i14] += R_cw[i14 + 3 * i15] * (double)b[i15];
      }
    }

    h_ui[0] = Cx + f * (h_ci[0] / h_ci[2]);
    h_ui[1] = Cy + f * (h_ci[1] / h_ci[2]);
    h_ui[2] = b_xt->data[(int)(((13.0 + (floor(y) + 1.0) * (7.0 +
      numPointsPerAnchor)) - numPointsPerAnchor) + featureOffset) - 1] * f *
      baseline / h_ci[2];

    //  h_rx=R_cw*[1;0;0];
    if (z_data[2] < -50.0) {
      // invalid disparity
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
    //          R=[0,-w(3),w(2);
    //      w(3),0,-w(1);
    //      -w(2),w(1),0];
    //  correct
    //  test with x update
    //  derivatives of (1) with respect to anchorstates
    //  should be  [R_cw*rho,-R_cw*hat_tmp1]; but because state is f_wc and not f_cw ther comes a - sign 
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

    c_xt = b_xt->data[(int)(((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 3; idx++) {
        b_R_cw[idx + 3 * br] = -R_cw[idx + 3 * br] * c_xt;
      }
    }

    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 3; idx++) {
        b_R_cw[idx + 3 * (br + 3)] = 0.0;
      }
    }

    for (br = 0; br < 6; br++) {
      for (idx = 0; idx < 3; idx++) {
        b_R_cw[idx + 3 * (br + 6)] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 12; idx++) {
        b_h_u_To_h_c[br + 3 * idx] = 0.0;
        for (ii = 0; ii < 3; ii++) {
          b_h_u_To_h_c[br + 3 * idx] += h_u_To_h_c[br + 3 * ii] * b_R_cw[ii + 3 *
            idx];
        }
      }
    }

    idx = i15 - i14;
    for (i14 = 0; i14 < 12; i14++) {
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

    d0 = (6.0 + numPointsPerAnchor) * (x + 1.0) - ((6.0 + numPointsPerAnchor) -
      1.0);
    c_xt = (6.0 + numPointsPerAnchor) * (x + 1.0);
    if (d0 > c_xt) {
      br = 0;
      idx = 0;
    } else {
      br = (int)d0 - 1;
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

    c_xt = b_xt->data[(int)(((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    for (br = 0; br < 3; br++) {
      b_h_rz[br] = fp[br] - b_xt->data[br];
    }

    for (br = 0; br < 3; br++) {
      d_xt[br] = 0.0;
      for (idx = 0; idx < 3; idx++) {
        d_xt[br] += R_cw[br + 3 * idx] * b_h_rz[idx];
      }

      for (idx = 0; idx < 3; idx++) {
        c_R_cw[idx + 3 * br] = R_cw[idx + 3 * br] * c_xt;
      }
    }

    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 3; idx++) {
        c_R_cw[idx + 3 * (br + 3)] = 0.0;
      }
    }

    for (br = 0; br < 3; br++) {
      c_R_cw[18 + br] = d_xt[br];
    }

    for (br = 0; br < 3; br++) {
      for (idx = 0; idx < 7; idx++) {
        c_h_u_To_h_c[br + 3 * idx] = 0.0;
        for (ii = 0; ii < 3; ii++) {
          c_h_u_To_h_c[br + 3 * idx] += h_u_To_h_c[br + 3 * ii] * c_R_cw[ii + 3 *
            idx];
        }
      }
    }

    idx = i15 - i14;
    ii = r1->size[0];
    for (i14 = 0; i14 < ii; i14++) {
      for (i15 = 0; i15 < idx; i15++) {
        H_xm_tmp->data[b_tmp_data[i15] + H_xm_tmp->size[0] * r1->data[i14]] =
          c_h_u_To_h_c[i15 + idx * i14];
      }
    }

    k++;
  }

  emxInit_real_T(&Hg, 2);
  i14 = Hg->size[0] * Hg->size[1];
  Hg->size[0] = 3;
  Hg->size[1] = (int)(12.0 + numStatesFeatures);
  emxEnsureCapacity((emxArray__common *)Hg, i14, (int)sizeof(double));
  ib = 3 * (int)(12.0 + numStatesFeatures);
  for (i14 = 0; i14 < ib; i14++) {
    Hg->data[i14] = 0.0;
  }

  b_emxInit_real_T(&h_u_apr, 1);
  emxInit_real_T(&J2, 2);
  emxInit_real_T(&b_b, 2);
  if (ii_size_idx_0 > 0) {
    emxInit_real_T(&R, 2);
    emxInit_real_T(&r2, 2);
    Hg->data[Hg->size[0] * 3] = 0.0;
    Hg->data[Hg->size[0] << 2] = -h_rz[2];
    Hg->data[Hg->size[0] * 5] = h_rz[1];
    Hg->data[1 + Hg->size[0] * 3] = h_rz[2];
    Hg->data[1 + (Hg->size[0] << 2)] = 0.0;
    Hg->data[1 + Hg->size[0] * 5] = -h_rz[0];
    Hg->data[2 + Hg->size[0] * 3] = -h_rz[1];
    Hg->data[2 + (Hg->size[0] << 2)] = h_rz[0];
    Hg->data[2 + Hg->size[0] * 5] = 0.0;

    //  R=imNoise.^2*eye(numMeas*numMeasDim);
    eye((double)ii_size_idx_0, J2);
    power(imNoise, dv10);
    b_diag(dv10, dv11);
    kron(J2->data, J2->size, dv11, r2);
    i14 = R->size[0] * R->size[1];
    R->size[0] = r2->size[0];
    R->size[1] = r2->size[1];
    emxEnsureCapacity((emxArray__common *)R, i14, (int)sizeof(double));
    ib = r2->size[0] * r2->size[1];
    for (i14 = 0; i14 < ib; i14++) {
      R->data[i14] = r2->data[i14];
    }

    emxFree_real_T(&r2);
    emxInit_real_T(&b_R, 2);
    i14 = b_R->size[0] * b_R->size[1];
    b_R->size[0] = R->size[0];
    b_R->size[1] = R->size[1];
    emxEnsureCapacity((emxArray__common *)b_R, i14, (int)sizeof(double));
    ib = R->size[0] * R->size[1];
    for (i14 = 0; i14 < ib; i14++) {
      b_R->data[i14] = R->data[i14];
    }

    emxInit_real_T(&H, 2);
    blkdiag(b_R, R);
    ib = ii_size_idx_0 * 3 - 1;
    ar = ii_size_idx_0 * 3;
    ii = H_xm_tmp->size[1] - 1;
    i14 = H->size[0] * H->size[1];
    H->size[0] = ar + 3;
    H->size[1] = ii + 13;
    emxEnsureCapacity((emxArray__common *)H, i14, (int)sizeof(double));
    emxFree_real_T(&b_R);
    for (i14 = 0; i14 < 12; i14++) {
      for (i15 = 0; i15 < ar; i15++) {
        H->data[i15 + H->size[0] * i14] = H_xc_tmp->data[i15 + H_xc_tmp->size[0]
          * i14];
      }
    }

    for (i14 = 0; i14 <= ii; i14++) {
      for (i15 = 0; i15 <= ib; i15++) {
        H->data[i15 + H->size[0] * (i14 + 12)] = H_xm_tmp->data[i15 +
          H_xm_tmp->size[0] * i14];
      }
    }

    ib = Hg->size[1];
    for (i14 = 0; i14 < ib; i14++) {
      for (i15 = 0; i15 < 3; i15++) {
        H->data[(i15 + ar) + H->size[0] * i14] = Hg->data[i15 + Hg->size[0] *
          i14];
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
      m = H->size[0];
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
        for (idx = 0; idx <= ii; idx += m) {
          i14 = idx + m;
          for (ic = idx; ic + 1 <= i14; ic++) {
            H_xm_tmp->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += m) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (P_apr->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                H_xm_tmp->data[ic] += P_apr->data[ib] * H->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
        }
      }
    }

    i14 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = H->size[1];
    b_b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i14, (int)sizeof(double));
    ib = H->size[0];
    for (i14 = 0; i14 < ib; i14++) {
      ar = H->size[1];
      for (i15 = 0; i15 < ar; i15++) {
        b_b->data[i15 + b_b->size[0] * i14] = H->data[i14 + H->size[0] * i15];
      }
    }

    if (H_xm_tmp->size[1] == 1) {
      i14 = J2->size[0] * J2->size[1];
      J2->size[0] = H_xm_tmp->size[0];
      J2->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)J2, i14, (int)sizeof(double));
      ib = H_xm_tmp->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = b_b->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          J2->data[i14 + J2->size[0] * i15] = 0.0;
          ii = H_xm_tmp->size[1];
          for (br = 0; br < ii; br++) {
            J2->data[i14 + J2->size[0] * i15] += H_xm_tmp->data[i14 +
              H_xm_tmp->size[0] * br] * b_b->data[br + b_b->size[0] * i15];
          }
        }
      }
    } else {
      k = H_xm_tmp->size[1];
      a[0] = (signed char)H_xm_tmp->size[0];
      a[1] = (signed char)b_b->size[1];
      m = H_xm_tmp->size[0];
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

      ii = H_xm_tmp->size[0] * (b_b->size[1] - 1);
      for (idx = 0; idx <= ii; idx += m) {
        i14 = idx + m;
        for (ic = idx; ic + 1 <= i14; ic++) {
          J2->data[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= ii; idx += m) {
        ar = 0;
        i14 = br + k;
        for (ib = br; ib + 1 <= i14; ib++) {
          if (b_b->data[ib] != 0.0) {
            ia = ar;
            i15 = idx + m;
            for (ic = idx; ic + 1 <= i15; ic++) {
              ia++;
              J2->data[ic] += b_b->data[ib] * H_xm_tmp->data[ia - 1];
            }
          }

          ar += m;
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

    i14 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = H->size[1];
    b_b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b_b, i14, (int)sizeof(double));
    ib = H->size[0];
    for (i14 = 0; i14 < ib; i14++) {
      ar = H->size[1];
      for (i15 = 0; i15 < ar; i15++) {
        b_b->data[i15 + b_b->size[0] * i14] = H->data[i14 + H->size[0] * i15];
      }
    }

    if (P_apr->size[1] == 1) {
      i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
      H_xm_tmp->size[0] = P_apr->size[0];
      H_xm_tmp->size[1] = b_b->size[1];
      emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
      ib = P_apr->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = b_b->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] = 0.0;
          ii = P_apr->size[1];
          for (br = 0; br < ii; br++) {
            H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] += P_apr->data[i14 +
              P_apr->size[0] * br] * b_b->data[br + b_b->size[0] * i15];
          }
        }
      }
    } else {
      k = P_apr->size[1];
      a[0] = P_apr->size[0];
      a[1] = b_b->size[1];
      m = P_apr->size[0];
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
        ii = P_apr->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i14 = idx + m;
          for (ic = idx; ic + 1 <= i14; ic++) {
            H_xm_tmp->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                H_xm_tmp->data[ic] += b_b->data[ib] * P_apr->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
        }
      }
    }

    emxInit_real_T(&K, 2);
    mrdivide(H_xm_tmp, R, K);

    //  K = p_km1_k*H'*inv(H*p_km1_k*H' + R);
    B = norm(za);
    for (i14 = 0; i14 < z_size_idx_0; i14++) {
      b_z_data[i14] = z_data[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      b_z_data[i14 + z_size_idx_0] = za[i14] / B;
    }

    for (i14 = 0; i14 < i; i14++) {
      b_h_u_data[i14] = h_u_data[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      b_h_u_data[i14 + i] = h_rz[i14];
    }

    ib = z_size_idx_0 + 3;
    for (i14 = 0; i14 < ib; i14++) {
      r_data[i14] = b_z_data[i14] - b_h_u_data[i14];
    }

    for (i = 1; i - 1 < ii_size_idx_0; i++) {
      ii = i * 3 - 2;
      for (i14 = 0; i14 < 2; i14++) {
        b_z_all[i14] = r_data[i14 + ii];
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
      d0 = 0.0;
      for (i14 = 0; i14 < 2; i14++) {
        d0 += a[i14] * r_data[i14 + ii];
      }

      if (d0 > 80.0) {
        updateVect_out[indMeas_data[i - 1] - 1] = 0.0;
      }
    }

    emxFree_real_T(&R);
    k = K->size[1];
    a[0] = K->size[0];
    m = K->size[0];
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
      while ((m > 0) && (idx <= 0)) {
        for (ic = 1; ic <= m; ic++) {
          h_u_apr->data[ic - 1] = 0.0;
        }

        idx = m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= 0)) {
        ar = 0;
        i14 = br + k;
        for (ib = br; ib + 1 <= i14; ib++) {
          if (r_data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= m; ic++) {
              ia++;
              h_u_apr->data[ic] += r_data[ib] * K->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        idx = m;
      }
    }

    eye(12.0 + numStatesFeatures, H_xm_tmp);
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
      m = K->size[0];
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
        while ((m > 0) && (idx <= ii)) {
          i14 = idx + m;
          for (ic = idx; ic + 1 <= i14; ic++) {
            J2->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                J2->data[ic] += H->data[ib] * K->data[ia - 1];
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
    i14 = H_xm_tmp->size[0] * H_xm_tmp->size[1];
    emxEnsureCapacity((emxArray__common *)H_xm_tmp, i14, (int)sizeof(double));
    ii = H_xm_tmp->size[0];
    idx = H_xm_tmp->size[1];
    ib = ii * idx;
    for (i14 = 0; i14 < ib; i14++) {
      H_xm_tmp->data[i14] -= J2->data[i14];
    }

    i14 = b_b->size[0] * b_b->size[1];
    b_b->size[0] = P_apr->size[0];
    b_b->size[1] = P_apr->size[1];
    emxEnsureCapacity((emxArray__common *)b_b, i14, (int)sizeof(double));
    ib = P_apr->size[0] * P_apr->size[1];
    for (i14 = 0; i14 < ib; i14++) {
      b_b->data[i14] = P_apr->data[i14];
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
      m = H_xm_tmp->size[0];
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

      if ((H_xm_tmp->size[0] == 0) || (b_b->size[1] == 0)) {
      } else {
        ii = H_xm_tmp->size[0] * (b_b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i14 = idx + m;
          for (ic = idx; ic + 1 <= i14; ic++) {
            P_apr->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i14 = br + k;
          for (ib = br; ib + 1 <= i14; ib++) {
            if (b_b->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                P_apr->data[ic] += b_b->data[ib] * H_xm_tmp->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          idx += m;
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
      b_h_rz[i14] = h_u_apr->data[3 + i14];
    }

    quatPlusThetaJ(b_h_rz, dv12);
    quatmultJ(dv12, *(double (*)[4])&b_xt->data[3], q_tmp);
    for (i14 = 0; i14 < 4; i14++) {
      b_xt->data[3 + i14] = q_tmp[i14];
    }

    // xt(4:7) = quatmultJ(xt(4:7),quatPlusThetaJ(x_apo(4:6)));
    for (i14 = 0; i14 < 6; i14++) {
      e_xt[i14] = b_xt->data[7 + i14] + h_u_apr->data[6 + i14];
    }

    for (i14 = 0; i14 < 6; i14++) {
      b_xt->data[7 + i14] = e_xt[i14];
    }

    for (ii = 0; ii < (int)numAnchors; ii++) {
      b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (6.0 + numPointsPerAnchor)) - 1] += h_u_apr->data[(int)
        ((12.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) - (5.0 +
          numPointsPerAnchor)) - 1];
      b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (5.0 + numPointsPerAnchor)) - 1] += h_u_apr->data[(int)
        ((12.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) - (4.0 +
          numPointsPerAnchor)) - 1];
      b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (4.0 + numPointsPerAnchor)) - 1] += h_u_apr->data[(int)
        ((12.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) - (3.0 +
          numPointsPerAnchor)) - 1];
      dqtmp[0] = h_u_apr->data[(int)((12.0 + (1.0 + (double)ii) * (6.0 +
        numPointsPerAnchor)) - (2.0 + numPointsPerAnchor)) - 1];
      dqtmp[1] = h_u_apr->data[(int)((12.0 + (1.0 + (double)ii) * (6.0 +
        numPointsPerAnchor)) - (1.0 + numPointsPerAnchor)) - 1];
      dqtmp[2] = h_u_apr->data[(int)((12.0 + (1.0 + (double)ii) * (6.0 +
        numPointsPerAnchor)) - numPointsPerAnchor) - 1];
      qOld_tmp[0] = b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 +
        numPointsPerAnchor)) - (3.0 + numPointsPerAnchor)) - 1];
      qOld_tmp[1] = b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 +
        numPointsPerAnchor)) - (2.0 + numPointsPerAnchor)) - 1];
      qOld_tmp[2] = b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 +
        numPointsPerAnchor)) - (1.0 + numPointsPerAnchor)) - 1];
      qOld_tmp[3] = b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 +
        numPointsPerAnchor)) - numPointsPerAnchor) - 1];
      quatPlusThetaJ(dqtmp, dv13);
      quatmultJ(dv13, qOld_tmp, q_tmp);
      b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (3.0 + numPointsPerAnchor)) - 1] = q_tmp[0];
      b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (2.0 + numPointsPerAnchor)) - 1] = q_tmp[1];
      b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - (1.0 + numPointsPerAnchor)) - 1] = q_tmp[2];
      b_xt->data[(int)((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                       - numPointsPerAnchor) - 1] = q_tmp[3];
      b_xt->data[(int)(((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                        - numPointsPerAnchor) + 1.0) - 1] += h_u_apr->data[(int)
        (((12.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) -
          numPointsPerAnchor) + 1.0) - 1];
    }
  }

  emxFree_real_T(&Hg);
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
  i = 0;
  emxInit_real_T(&Jtmp, 2);
  b_emxInit_int32_T(&r3, 2);
  emxInit_real_T(&r4, 2);
  while (i <= (int)numAnchors - 1) {
    if (updateVect[i] == 2.0) {
      d0 = (1.0 + (double)i) * numPointsPerAnchor * 3.0 - (numPointsPerAnchor *
        3.0 - 1.0);
      if (d0 > (1.0 + (double)i) * numPointsPerAnchor * 3.0) {
        i14 = 1;
      } else {
        i14 = (int)d0;
      }

      d0 = (1.0 + (double)i) * numPointsPerAnchor * 2.0 - (numPointsPerAnchor *
        2.0 - 1.0);
      c_xt = (1.0 + (double)i) * numPointsPerAnchor * 2.0;
      if (d0 > c_xt) {
        i15 = 0;
        br = 0;
      } else {
        i15 = (int)d0 - 1;
        br = (int)c_xt;
      }

      idx = r3->size[0] * r3->size[1];
      r3->size[0] = 1;
      r3->size[1] = br - i15;
      emxEnsureCapacity((emxArray__common *)r3, idx, (int)sizeof(int));
      ib = br - i15;
      for (br = 0; br < ib; br++) {
        r3->data[r3->size[0] * br] = i15 + br;
      }

      for (i15 = 0; i15 < 2; i15++) {
        b_z_all[i15] = z_all[(i14 + i15) - 1];
      }

      ib = r3->size[1];
      for (i15 = 0; i15 < ib; i15++) {
        anchor_u->data[r3->data[r3->size[0] * i15]] = b_z_all[(*(int (*)[2])
          r3->size)[0] * i15];
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

      if (z_all[i14 + 1] > -50.0) {
        rhoInit = z_all[i14 + 1] / (baseline * f);
        rhoSigma = 0.1;
      } else {
        rhoInit = 0.5;
        rhoSigma = 5.0;
      }

      if (8.0 > (8.0 + numPointsPerAnchor) - 1.0) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = 7;
        i15 = (int)((8.0 + numPointsPerAnchor) - 1.0);
      }

      br = r3->size[0] * r3->size[1];
      r3->size[0] = 1;
      r3->size[1] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)r3, br, (int)sizeof(int));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        r3->data[r3->size[0] * i15] = i14 + i15;
      }

      ib = r3->size[0] * r3->size[1];
      for (i14 = 0; i14 < ib; i14++) {
        h_u_apr->data[r3->data[i14]] = rhoInit;
      }

      d0 = (13.0 + (1.0 + (double)i) * (7.0 + numPointsPerAnchor)) - (6.0 +
        numPointsPerAnchor);
      c_xt = 13.0 + (1.0 + (double)i) * (7.0 + numPointsPerAnchor);
      if (d0 > c_xt) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
        i15 = (int)c_xt;
      }

      br = r3->size[0] * r3->size[1];
      r3->size[0] = 1;
      r3->size[1] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)r3, br, (int)sizeof(int));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        r3->data[r3->size[0] * i15] = i14 + i15;
      }

      ib = r3->size[1];
      for (i14 = 0; i14 < ib; i14++) {
        b_xt->data[r3->data[r3->size[0] * i14]] = h_u_apr->data[(*(int (*)[2])
          r3->size)[0] * i14];
      }

      // now create J and multiply with P
      eye(12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0),
          H_xm_tmp);
      eye((numAnchors - (1.0 + (double)i)) * (6.0 + numPointsPerAnchor), J2);

      // J=zeros(144);
      i14 = r4->size[0] * r4->size[1];
      r4->size[0] = (int)(6.0 + numPointsPerAnchor);
      r4->size[1] = (int)(6.0 + numPointsPerAnchor);
      emxEnsureCapacity((emxArray__common *)r4, i14, (int)sizeof(double));
      ib = (int)(6.0 + numPointsPerAnchor) * (int)(6.0 + numPointsPerAnchor);
      for (i14 = 0; i14 < ib; i14++) {
        r4->data[i14] = 0.0;
      }

      b_blkdiag(H_xm_tmp, r4, J2, Jtmp);
      d0 = (12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) - ((6.0 +
        numPointsPerAnchor) - 1.0);
      c_xt = 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i);
      if (d0 > c_xt) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
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
      for (i14 = 0; i14 < 12; i14++) {
        for (i15 = 0; i15 < idx; i15++) {
          Jtmp->data[r1->data[i15] + Jtmp->size[0] * i14] = y_To_x_c[i15 + idx *
            i14];
        }
      }

      // state covariance
      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 1.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d0 - 1;
      }

      i15 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)(12.0 + numAnchors * (6.0 + numPointsPerAnchor));
      J2->size[1] = (int)(6.0 + numPointsPerAnchor);
      emxEnsureCapacity((emxArray__common *)J2, i15, (int)sizeof(double));
      ib = (int)(12.0 + numAnchors * (6.0 + numPointsPerAnchor)) * (int)(6.0 +
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

      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 1.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d0 - 1;
      }

      i15 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)(6.0 + numPointsPerAnchor);
      J2->size[1] = (int)(12.0 + numAnchors * (6.0 + numPointsPerAnchor));
      emxEnsureCapacity((emxArray__common *)J2, i15, (int)sizeof(double));
      ib = (int)(6.0 + numPointsPerAnchor) * (int)(12.0 + numAnchors * (6.0 +
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
        m = Jtmp->size[0];
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
          while ((m > 0) && (idx <= ii)) {
            i14 = idx + m;
            for (ic = idx; ic + 1 <= i14; ic++) {
              H_xm_tmp->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i14 = br + k;
            for (ib = br; ib + 1 <= i14; ib++) {
              if (P_apr->data[ib] != 0.0) {
                ia = ar;
                i15 = idx + m;
                for (ic = idx; ic + 1 <= i15; ic++) {
                  ia++;
                  H_xm_tmp->data[ic] += P_apr->data[ib] * Jtmp->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      i14 = b_b->size[0] * b_b->size[1];
      b_b->size[0] = Jtmp->size[1];
      b_b->size[1] = Jtmp->size[0];
      emxEnsureCapacity((emxArray__common *)b_b, i14, (int)sizeof(double));
      ib = Jtmp->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        ar = Jtmp->size[1];
        for (i15 = 0; i15 < ar; i15++) {
          b_b->data[i15 + b_b->size[0] * i14] = Jtmp->data[i14 + Jtmp->size[0] *
            i15];
        }
      }

      if ((H_xm_tmp->size[1] == 1) || (b_b->size[0] == 1)) {
        i14 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = H_xm_tmp->size[0];
        P_apr->size[1] = b_b->size[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i14, (int)sizeof(double));
        ib = H_xm_tmp->size[0];
        for (i14 = 0; i14 < ib; i14++) {
          ar = b_b->size[1];
          for (i15 = 0; i15 < ar; i15++) {
            P_apr->data[i14 + P_apr->size[0] * i15] = 0.0;
            ii = H_xm_tmp->size[1];
            for (br = 0; br < ii; br++) {
              P_apr->data[i14 + P_apr->size[0] * i15] += H_xm_tmp->data[i14 +
                H_xm_tmp->size[0] * br] * b_b->data[br + b_b->size[0] * i15];
            }
          }
        }
      } else {
        k = H_xm_tmp->size[1];
        a[0] = (unsigned int)H_xm_tmp->size[0];
        a[1] = (unsigned int)b_b->size[1];
        m = H_xm_tmp->size[0];
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

        if ((H_xm_tmp->size[0] == 0) || (b_b->size[1] == 0)) {
        } else {
          ii = H_xm_tmp->size[0] * (b_b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i14 = idx + m;
            for (ic = idx; ic + 1 <= i14; ic++) {
              P_apr->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i14 = br + k;
            for (ib = br; ib + 1 <= i14; ib++) {
              if (b_b->data[ib] != 0.0) {
                ia = ar;
                i15 = idx + m;
                for (ic = idx; ic + 1 <= i15; ic++) {
                  ia++;
                  P_apr->data[ic] += b_b->data[ib] * H_xm_tmp->data[ia - 1];
                }
              }

              ar += m;
            }

            br += k;
            idx += m;
          }
        }
      }

      // inverse depth variance
      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 7.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d0 - 1;
      }

      i15 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)(12.0 + numAnchors * (6.0 + numPointsPerAnchor));
      J2->size[1] = (int)numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)J2, i15, (int)sizeof(double));
      ib = (int)(12.0 + numAnchors * (6.0 + numPointsPerAnchor)) * (int)
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

      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 7.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d0 - 1;
      }

      i15 = J2->size[0] * J2->size[1];
      J2->size[0] = (int)numPointsPerAnchor;
      J2->size[1] = (int)(12.0 + numAnchors * (6.0 + numPointsPerAnchor));
      emxEnsureCapacity((emxArray__common *)J2, i15, (int)sizeof(double));
      ib = (int)numPointsPerAnchor * (int)(12.0 + numAnchors * (6.0 +
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

      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 7.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i14 = 0;
      } else {
        i14 = (int)d0 - 1;
      }

      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)i) - 1.0)) + 7.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)i)) {
        i15 = 0;
      } else {
        i15 = (int)d0 - 1;
      }

      c_xt = rhoSigma * rhoSigma;
      eye(numPointsPerAnchor, b_b);
      ib = b_b->size[1];
      for (br = 0; br < ib; br++) {
        ar = b_b->size[0];
        for (idx = 0; idx < ar; idx++) {
          P_apr->data[(i14 + idx) + P_apr->size[0] * (i15 + br)] = c_xt *
            b_b->data[idx + b_b->size[0] * br];
        }
      }

      d0 = (1.0 + (double)i) * numPointsPerAnchor - (numPointsPerAnchor - 1.0);
      c_xt = (1.0 + (double)i) * numPointsPerAnchor;
      if (d0 > c_xt) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
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

  emxFree_real_T(&r4);
  emxFree_real_T(&b_b);
  emxFree_int32_T(&r3);
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

  RotFromQuatJ(*(double (*)[4])&b_xt->data[3], R_cw);
  for (k = 0; k < ii_size_idx_0; k++) {
    x = floor(((double)indMeas_data[k] - 1.0) / numPointsPerAnchor);
    x_i = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (6.0 + numPointsPerAnchor)) - 1];
    y_i = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (5.0 + numPointsPerAnchor)) - 1];
    z_i = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (4.0 + numPointsPerAnchor)) - 1];
    fq_cw0 = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (3.0 + numPointsPerAnchor)) - 1];
    fq_cw1 = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (2.0 + numPointsPerAnchor)) - 1];
    fq_cw2 = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      (1.0 + numPointsPerAnchor)) - 1];
    fq_cw3 = b_xt->data[(int)((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
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
    h_rz[0] = (-Cx + current_anchor_u_idx_0) / f;
    h_rz[1] = (-Cy + current_anchor_u_idx_1) / f;
    h_rz[2] = 1.0;
    B = norm(h_rz);
    c_fq_cw0[0] = fq_cw0;
    c_fq_cw0[1] = fq_cw1;
    c_fq_cw0[2] = fq_cw2;
    c_fq_cw0[3] = fq_cw3;
    RotFromQuatJ(c_fq_cw0, h_u_To_h_c);
    c_xt = b_xt->data[(int)(((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    b_x_i[0] = x_i;
    b_x_i[1] = y_i;
    b_x_i[2] = z_i;
    for (i14 = 0; i14 < 3; i14++) {
      b_h_rz[i14] = h_rz[i14] / B;
    }

    for (i14 = 0; i14 < 3; i14++) {
      d0 = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        d0 += h_u_To_h_c[i15 + 3 * i14] * b_h_rz[i15];
      }

      d_xt[i14] = c_xt * (b_x_i[i14] - b_xt->data[i14]) + d0;
    }

    for (i14 = 0; i14 < 3; i14++) {
      h_ci[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        h_ci[i14] += R_cw[i14 + 3 * i15] * d_xt[i15];
      }
    }

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

    b_Cx[0] = Cx + f * (h_ci[0] / h_ci[2]);
    b_Cx[1] = Cy + f * (h_ci[1] / h_ci[2]);
    ib = i15 - i14;
    for (i14 = 0; i14 < ib; i14++) {
      h_u_apo->data[d_tmp_data[i14]] = b_Cx[i14];
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
// Arguments    : const emxArray_real_T *varargin_1
//                const emxArray_real_T *varargin_2
//                const emxArray_real_T *varargin_3
//                emxArray_real_T *y
// Return Type  : void
//
static void b_blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
                      *varargin_2, const emxArray_real_T *varargin_3,
                      emxArray_real_T *y)
{
  int r;
  int c;
  int i9;
  int loop_ub;
  int i10;
  int i11;
  r = (varargin_1->size[0] + varargin_2->size[0]) + varargin_3->size[0];
  c = (varargin_1->size[1] + varargin_2->size[1]) + varargin_3->size[1];
  i9 = y->size[0] * y->size[1];
  y->size[0] = r;
  emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
  i9 = y->size[0] * y->size[1];
  y->size[1] = c;
  emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
  r *= c;
  for (i9 = 0; i9 < r; i9++) {
    y->data[i9] = 0.0;
  }

  if ((varargin_1->size[0] > 0) && (varargin_1->size[1] > 0)) {
    r = varargin_1->size[1];
    for (i9 = 0; i9 < r; i9++) {
      loop_ub = varargin_1->size[0];
      for (i10 = 0; i10 < loop_ub; i10++) {
        y->data[i10 + y->size[0] * i9] = varargin_1->data[i10 + varargin_1->
          size[0] * i9];
      }
    }
  }

  if ((varargin_2->size[0] > 0) && (varargin_2->size[1] > 0)) {
    i9 = varargin_1->size[0] + varargin_2->size[0];
    if (varargin_1->size[0] + 1 > i9) {
      i9 = 1;
    } else {
      i9 = varargin_1->size[0] + 1;
    }

    i10 = varargin_1->size[1] + varargin_2->size[1];
    if (varargin_1->size[1] + 1 > i10) {
      i10 = 1;
    } else {
      i10 = varargin_1->size[1] + 1;
    }

    r = varargin_2->size[1];
    for (c = 0; c < r; c++) {
      loop_ub = varargin_2->size[0];
      for (i11 = 0; i11 < loop_ub; i11++) {
        y->data[((i9 + i11) + y->size[0] * ((i10 + c) - 1)) - 1] =
          varargin_2->data[i11 + varargin_2->size[0] * c];
      }
    }
  }

  r = varargin_1->size[0] + varargin_2->size[0];
  c = varargin_1->size[1] + varargin_2->size[1];
  if ((varargin_3->size[0] > 0) && (varargin_3->size[1] > 0)) {
    i9 = r + varargin_3->size[0];
    if (r + 1 > i9) {
      i9 = 1;
    } else {
      i9 = r + 1;
    }

    i10 = c + varargin_3->size[1];
    if (c + 1 > i10) {
      i10 = 1;
    } else {
      i10 = c + 1;
    }

    r = varargin_3->size[1];
    for (c = 0; c < r; c++) {
      loop_ub = varargin_3->size[0];
      for (i11 = 0; i11 < loop_ub; i11++) {
        y->data[((i9 + i11) + y->size[0] * ((i10 + c) - 1)) - 1] =
          varargin_3->data[i11 + varargin_3->size[0] * c];
      }
    }
  }
}

//
// Arguments    : const double v[3]
//                double d[9]
// Return Type  : void
//
static void b_diag(const double v[3], double d[9])
{
  int j;
  memset(&d[0], 0, 9U * sizeof(double));
  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
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
// Arguments    : const emxArray_real_T *varargin_1
//                emxArray_real_T *y
// Return Type  : void
//
static void blkdiag(const emxArray_real_T *varargin_1, emxArray_real_T *y)
{
  signed char unnamed_idx_0;
  signed char unnamed_idx_1;
  int i3;
  int loop_ub;
  int b_unnamed_idx_0;
  int i4;
  int i5;
  int i6;
  int tmp_data[99];
  int b_tmp_data[99];
  static const signed char varargin_2[9] = { 10, 0, 0, 0, 10, 0, 0, 0, 10 };

  unnamed_idx_0 = (signed char)(varargin_1->size[0] + 3);
  unnamed_idx_1 = (signed char)(varargin_1->size[1] + 3);
  i3 = y->size[0] * y->size[1];
  y->size[0] = unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)y, i3, (int)sizeof(double));
  i3 = y->size[0] * y->size[1];
  y->size[1] = unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)y, i3, (int)sizeof(double));
  loop_ub = unnamed_idx_0 * unnamed_idx_1;
  for (i3 = 0; i3 < loop_ub; i3++) {
    y->data[i3] = 0.0;
  }

  loop_ub = varargin_1->size[1];
  for (i3 = 0; i3 < loop_ub; i3++) {
    b_unnamed_idx_0 = varargin_1->size[0];
    for (i4 = 0; i4 < b_unnamed_idx_0; i4++) {
      y->data[i4 + y->size[0] * i3] = varargin_1->data[i4 + varargin_1->size[0] *
        i3];
    }
  }

  if (varargin_1->size[0] + 1 > varargin_1->size[0] + 3) {
    i3 = 1;
    i4 = 0;
  } else {
    i3 = varargin_1->size[0] + 1;
    i4 = varargin_1->size[0] + 3;
  }

  if (varargin_1->size[1] + 1 > varargin_1->size[1] + 3) {
    i5 = 1;
    i6 = 0;
  } else {
    i5 = varargin_1->size[1] + 1;
    i6 = varargin_1->size[1] + 3;
  }

  loop_ub = (i4 - i3) + 1;
  for (b_unnamed_idx_0 = 0; b_unnamed_idx_0 < loop_ub; b_unnamed_idx_0++) {
    tmp_data[b_unnamed_idx_0] = (i3 + b_unnamed_idx_0) - 1;
  }

  loop_ub = (i6 - i5) + 1;
  for (b_unnamed_idx_0 = 0; b_unnamed_idx_0 < loop_ub; b_unnamed_idx_0++) {
    b_tmp_data[b_unnamed_idx_0] = (i5 + b_unnamed_idx_0) - 1;
  }

  b_unnamed_idx_0 = (i4 - i3) + 1;
  loop_ub = (i6 - i5) + 1;
  for (i3 = 0; i3 < loop_ub; i3++) {
    for (i4 = 0; i4 < b_unnamed_idx_0; i4++) {
      y->data[tmp_data[i4] + y->size[0] * b_tmp_data[i3]] = varargin_2[i4 +
        b_unnamed_idx_0 * i3];
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
// Arguments    : const double v[9]
//                double d[81]
// Return Type  : void
//
static void diag(const double v[9], double d[81])
{
  int j;
  memset(&d[0], 0, 81U * sizeof(double));
  for (j = 0; j < 9; j++) {
    d[j + 9 * j] = v[j];
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
  int ipiv_data[99];
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
  int jpvt_data[99];
  int tau_size[1];
  double tau_data[99];
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
  double work_data[99];
  double vn1_data[99];
  double vn2_data[99];
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
  int i7;
  emxArray_real_T *b_B;
  int loop_ub;
  int A_idx_1;
  int i8;
  emxArray_real_T *b_A;
  emxArray_real_T *c_A;
  emxArray_real_T *r0;
  if (A->size[0] == 0) {
    unnamed_idx_1 = (signed char)B->size[0];
    i7 = y->size[0] * y->size[1];
    y->size[0] = 0;
    emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
    i7 = y->size[0] * y->size[1];
    y->size[1] = unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
  } else if (B->size[0] == B->size[1]) {
    eml_lusolve(B, A, y);
  } else {
    emxInit_real_T(&b_B, 2);
    i7 = b_B->size[0] * b_B->size[1];
    b_B->size[0] = B->size[1];
    b_B->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_B, i7, (int)sizeof(double));
    loop_ub = B->size[0];
    for (i7 = 0; i7 < loop_ub; i7++) {
      A_idx_1 = B->size[1];
      for (i8 = 0; i8 < A_idx_1; i8++) {
        b_B->data[i8 + b_B->size[0] * i7] = B->data[i7 + B->size[0] * i8];
      }
    }

    emxInit_real_T(&b_A, 2);
    emxInit_real_T(&c_A, 2);
    i7 = c_A->size[0] * c_A->size[1];
    c_A->size[0] = A->size[1];
    c_A->size[1] = A->size[0];
    emxEnsureCapacity((emxArray__common *)c_A, i7, (int)sizeof(double));
    loop_ub = A->size[0];
    for (i7 = 0; i7 < loop_ub; i7++) {
      A_idx_1 = A->size[1];
      for (i8 = 0; i8 < A_idx_1; i8++) {
        c_A->data[i8 + c_A->size[0] * i7] = A->data[i7 + A->size[0] * i8];
      }
    }

    loop_ub = A->size[1];
    A_idx_1 = A->size[0];
    i7 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = loop_ub;
    b_A->size[1] = A_idx_1;
    emxEnsureCapacity((emxArray__common *)b_A, i7, (int)sizeof(double));
    for (i7 = 0; i7 < A_idx_1; i7++) {
      for (i8 = 0; i8 < loop_ub; i8++) {
        b_A->data[i8 + b_A->size[0] * i7] = c_A->data[i8 + loop_ub * i7];
      }
    }

    emxFree_real_T(&c_A);
    emxInit_real_T(&r0, 2);
    eml_qrsolve(b_B, b_A, r0);
    i7 = y->size[0] * y->size[1];
    y->size[0] = r0->size[1];
    y->size[1] = r0->size[0];
    emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
    loop_ub = r0->size[0];
    emxFree_real_T(&b_A);
    emxFree_real_T(&b_B);
    for (i7 = 0; i7 < loop_ub; i7++) {
      A_idx_1 = r0->size[1];
      for (i8 = 0; i8 < A_idx_1; i8++) {
        y->data[i8 + y->size[0] * i7] = r0->data[i7 + r0->size[0] * i8];
      }
    }

    emxFree_real_T(&r0);
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
  int i1;
  int i2;
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
  for (i1 = 0; i1 < 4; i1++) {
    qp[i1] = 0.0;
    for (i2 = 0; i2 < 4; i2++) {
      qp[i1] += b_p[i1 + (i2 << 2)] * b_q[i2];
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
//                emxArray_real_T *P_apo_out
// Return Type  : void
//
void SLAM(const double updateVect[32], const double z_all[96], const double
          cameraparams[4], double dt, const double processNoise[4], const double
          IMU_measurements[9], const double imNoise[3], double
          numPointsPerAnchor, double numAnchors, emxArray_real_T *h_u_apo,
          emxArray_real_T *xt_out, double updateVect_out[32], emxArray_real_T
          *anchor_u_out, emxArray_real_T *anchor_pose_out, emxArray_real_T
          *P_apo_out)
{
  int i0;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  int loop_ub;
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const short b_y[9] = { 1000, 0, 0, 0, 1000, 0, 0, 0, 1000 };

  emxArray_real_T *P_apr;
  double b_imNoise[3];
  if (!initialized_not_empty) {
    initialized_not_empty = true;
    i0 = xt->size[0];
    xt->size[0] = 13 + (int)(numAnchors * (7.0 + numPointsPerAnchor));
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
    xt->data[10] = 0.0;
    xt->data[11] = 0.0;
    xt->data[12] = 0.0;
    loop_ub = (int)(numAnchors * (7.0 + numPointsPerAnchor));
    for (i0 = 0; i0 < loop_ub; i0++) {
      xt->data[i0 + 13] = 1.0;
    }

    //  initial real vector
    eye(12.0 + numAnchors * (6.0 + numPointsPerAnchor), P_apo);
    i0 = P_apo->size[0] * P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo, i0, (int)sizeof(double));
    i0 = P_apo->size[0];
    loop_ub = P_apo->size[1];
    loop_ub *= i0;
    for (i0 = 0; i0 < loop_ub; i0++) {
      P_apo->data[i0] = 0.0;
    }

    //  initial error state covariance
    for (i0 = 0; i0 < 3; i0++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        P_apo->data[(loop_ub + P_apo->size[0] * (9 + i0)) + 9] = y[loop_ub + 3 *
          i0];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        P_apo->data[loop_ub + P_apo->size[0] * i0] = 0.0;
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        P_apo->data[(loop_ub + P_apo->size[0] * (3 + i0)) + 3] = b_y[loop_ub + 3
          * i0];
      }
    }
  }

  emxInit_real_T(&P_apr, 2);
  SLAM_pred_euler(P_apo, xt, dt, processNoise, IMU_measurements, numAnchors *
                  (6.0 + numPointsPerAnchor), P_apr);

  // P_apo=P_apr;
  for (i0 = 0; i0 < 3; i0++) {
    b_imNoise[i0] = imNoise[i0];
  }

  SLAM_updIT(P_apr, xt, cameraparams, updateVect, z_all, b_imNoise,
             numPointsPerAnchor, numAnchors, *(double (*)[3])&IMU_measurements[3],
             h_u_apo, updateVect_out, anchor_u_out, anchor_pose_out);
  i0 = P_apo->size[0] * P_apo->size[1];
  P_apo->size[0] = P_apr->size[0];
  P_apo->size[1] = P_apr->size[1];
  emxEnsureCapacity((emxArray__common *)P_apo, i0, (int)sizeof(double));
  loop_ub = P_apr->size[0] * P_apr->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    P_apo->data[i0] = P_apr->data[i0];
  }

  emxFree_real_T(&P_apr);
  i0 = xt_out->size[0];
  xt_out->size[0] = xt->size[0];
  emxEnsureCapacity((emxArray__common *)xt_out, i0, (int)sizeof(double));
  loop_ub = xt->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    xt_out->data[i0] = xt->data[i0];
  }

  i0 = P_apo_out->size[0] * P_apo_out->size[1];
  P_apo_out->size[0] = P_apo->size[0];
  P_apo_out->size[1] = P_apo->size[1];
  emxEnsureCapacity((emxArray__common *)P_apo_out, i0, (int)sizeof(double));
  loop_ub = P_apo->size[0] * P_apo->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    P_apo_out->data[i0] = P_apo->data[i0];
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
