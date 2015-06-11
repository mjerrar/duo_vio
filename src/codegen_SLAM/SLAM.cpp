//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 11-Jun-2015 14:38:13
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
static emxArray_real_T *P;
static boolean_T b_initialized_not_empty;
static emxArray_real_T *anchor_u;
static emxArray_real_T *anchor_pose;

// Function Declarations
static void RotFromQuatJ(const double q[4], double R[9]);
static void SLAM_free();
static void SLAM_init();
static void SLAM_pred_euler(const emxArray_real_T *P_apo, emxArray_real_T *x,
  double dt, const double processNoise[4], const double IMU_measurements[9],
  double numStatesFeatures, emxArray_real_T *P_apr);
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams[4], const double updateVect[16], const double z_all[48],
  double imNoise[3], double numPointsPerAnchor, double numAnchors, const double
  za[3], emxArray_real_T *h_u_apo, double updateVect_out[16], emxArray_real_T
  *anchor_u_out, emxArray_real_T *anchor_pose_out);
static void SLAM_updIT_free();
static void SLAM_updIT_init();
static void b_blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
                      *varargin_2, const emxArray_real_T *varargin_3,
                      emxArray_real_T *y);
static void b_diag(const double v[3], double d[9]);
static double b_eml_matlab_zlarfg();
static double b_eml_xnrm2(int n, const double x_data[], int ix0);
static void b_eml_xswap(int n, double x_data[], int ix0, int iy0);
static void b_eml_xtrsm(int m, int n, const double A_data[], int lda,
  emxArray_real_T *B, int ldb);
static void b_emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void b_eye(double I[144]);
static double b_mod(double x, double y);
static void b_mrdivide(const double A[2], const double B[4], double y[2]);
static double b_norm(const double x[4]);
static void blkdiag(const double varargin_1_data[], const int varargin_1_size[2],
                    double y_data[], int y_size[2]);
static double c_eml_xnrm2(int n, const double x_data[], int ix0);
static void diag(const double v[9], double d[81]);
static int eml_ixamax(int n, const double x_data[], int ix0);
static void eml_lusolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *X);
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, double C_data[],
  int ic0, int ldc, double work_data[]);
static double eml_matlab_zlarfg(int n, double *alpha1, double x_data[], int ix0);
static void eml_qrsolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *Y);
static void eml_signed_integer_colon(int b, int y_data[], int y_size[2]);
static void eml_xgeqp3(double A_data[], int A_size[2], double tau_data[], int
  tau_size[1], int jpvt_data[], int jpvt_size[2]);
static void eml_xgetrf(int m, int n, double A_data[], int A_size[2], int lda,
  int ipiv_data[], int ipiv_size[2], int *info);
static double eml_xnrm2(int n, const double x_data[], int ix0);
static void eml_xswap(int n, double x_data[], int ix0, int incx, int iy0, int
                      incy);
static void eml_xtrsm(int m, int n, const double A_data[], int lda,
                      emxArray_real_T *B, int ldb);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void eye(double varargin_1, emxArray_real_T *I);
static void kron(const double A_data[], const int A_size[2], const double B[9],
                 double K_data[], int K_size[2]);
static void mrdivide(const emxArray_real_T *A, const double B_data[], const int
                     B_size[2], emxArray_real_T *y);
static double norm(const double x[3]);
static void power(const double a[3], double y[3]);
static void quatPlusThetaJ(const double dtheta[3], double dq[4]);
static void quatmultJ(const double q[4], const double p[4], double qp[4]);
static int rankFromQR(const double A_data[], const int A_size[2]);
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
  emxFree_real_T(&P);
  emxFree_real_T(&xt);
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_init()
{
  emxInit_real_T(&P, 2);
  b_emxInit_real_T(&xt, 1);
}

//
// EKF_SLAM: computes the camerapose p and feature position f in an
// interative way
//    Handels a static number of points but can dynamically asign them to new
//    klt points.
// Arguments    : const emxArray_real_T *P_apo
//                emxArray_real_T *x
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[9]
//                double numStatesFeatures
//                emxArray_real_T *P_apr
// Return Type  : void
//
static void SLAM_pred_euler(const emxArray_real_T *P_apo, emxArray_real_T *x,
  double dt, const double processNoise[4], const double IMU_measurements[9],
  double numStatesFeatures, emxArray_real_T *P_apr)
{
  double c;
  double b_processNoise[9];
  double c_processNoise[9];
  int i12;
  double Q[81];
  double b_IMU_measurements[6];
  double w[3];
  emxArray_real_T *F;
  double dv6[144];
  double dv7[9];
  double dv8[144];
  int i13;
  static const signed char iv1[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv2[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  int cr;
  int i;
  emxArray_real_T *G;
  int ar;
  static const signed char iv3[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv4[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
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
  // % compute the linearization F of the non linear model f
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

  // ===========ACC=====================================
  for (i12 = 0; i12 < 3; i12++) {
    b_IMU_measurements[i12] = IMU_measurements[i12];
  }

  for (i12 = 0; i12 < 3; i12++) {
    b_IMU_measurements[i12 + 3] = IMU_measurements[3 + i12];
  }

  for (i12 = 0; i12 < 3; i12++) {
    w[i12] = b_IMU_measurements[i12] - x->data[10 + i12];
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
      dv8[i13 + 12 * i12] = iv1[i13 + 3 * i12];
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
      dv8[(i13 + 12 * (i12 + 9)) + 3] = iv2[i13 + 3 * i12];
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

  RotFromQuatJ(*(double (*)[4])&x->data[3], dv7);
  for (i12 = 0; i12 < 9; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[i13 + G->size[0] * i12] = 0.0;
    }
  }

  for (i12 = 0; i12 < 9; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * i12) + 3] = iv3[i13 + 3 * i12];
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * i12) + 6] = 0.0;
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * (i12 + 3)) + 6] = -dv7[i13 + 3 * i12];
    }
  }

  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * (i12 + 6)) + 6] = 0.0;
    }
  }

  for (i12 = 0; i12 < 9; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      G->data[(i13 + G->size[0] * i12) + 9] = iv4[i13 + 3 * i12];
    }
  }

  emxInit_real_T(&y, 2);
  if ((F->size[1] == 1) || (P_apo->size[0] == 1)) {
    i12 = y->size[0] * y->size[1];
    y->size[0] = F->size[0];
    y->size[1] = P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)y, i12, (int)sizeof(double));
    ar = F->size[0];
    for (i12 = 0; i12 < ar; i12++) {
      i = P_apo->size[1];
      for (i13 = 0; i13 < i; i13++) {
        y->data[i12 + y->size[0] * i13] = 0.0;
        cr = F->size[1];
        for (br = 0; br < cr; br++) {
          y->data[i12 + y->size[0] * i13] += F->data[i12 + F->size[0] * br] *
            P_apo->data[br + P_apo->size[0] * i13];
        }
      }
    }
  } else {
    k = F->size[1];
    unnamed_idx_0 = (unsigned int)F->size[0];
    unnamed_idx_1 = (unsigned int)P_apo->size[1];
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

    if ((F->size[0] == 0) || (P_apo->size[1] == 0)) {
    } else {
      i = F->size[0] * (P_apo->size[1] - 1);
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
          if (P_apo->data[ib] != 0.0) {
            ia = ar;
            i13 = cr + m;
            for (ic = cr; ic + 1 <= i13; ic++) {
              ia++;
              y->data[ic] += P_apo->data[ib] * F->data[ia - 1];
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

  //  position
  for (i = 0; i < 3; i++) {
    b_x[i] = w[i] * dt;
  }

  quatPlusThetaJ(b_x, dq);
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

  emxInit_real_T(&b_P_apr, 2);
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
//                const double updateVect[16]
//                const double z_all[48]
//                double imNoise[3]
//                double numPointsPerAnchor
//                double numAnchors
//                const double za[3]
//                emxArray_real_T *h_u_apo
//                double updateVect_out[16]
//                emxArray_real_T *anchor_u_out
//                emxArray_real_T *anchor_pose_out
// Return Type  : void
//
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams[4], const double updateVect[16], const double z_all[48],
  double imNoise[3], double numPointsPerAnchor, double numAnchors, const double
  za[3], emxArray_real_T *h_u_apo, double updateVect_out[16], emxArray_real_T
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
  double R_cw[9];
  int idx;
  signed char ii_data[16];
  int ii;
  boolean_T exitg2;
  boolean_T guard2 = false;
  int ii_size_idx_0;
  signed char indMeas_data[16];
  int h_u_size_idx_0;
  double h_u_data[48];
  int z_size_idx_0;
  double z_data[48];
  int k;
  emxArray_int32_T *r1;
  int i15;
  int ar;
  double y;
  double x;
  double indAnchor;
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
  double mt[3];
  double B;
  double c_xt;
  double d_xt[3];
  double d0;
  double e_xt[3];
  double h_ci[3];
  double h_ui[3];
  double b_y[3];
  int tmp_data[48];
  int b_tmp_data[48];
  double b_R_cw[36];
  double b_h_u_To_h_c[36];
  double d1;
  double dv9[9];
  double c_R_cw[9];
  double d_R_cw[9];
  double e_R_cw[21];
  double c_h_u_To_h_c[21];
  emxArray_real_T *Hg;
  emxArray_real_T *h_u_apr;
  emxArray_real_T *J2;
  emxArray_real_T *b;
  double dv10[3];
  double dv11[9];
  int tmp_size[2];
  double c_tmp_data[2304];
  double R_data[2601];
  double b_R_data[2601];
  int R_size[2];
  emxArray_real_T *H;
  int br;
  double a[2];
  int m;
  int ic;
  int ia;
  double c_R_data;
  emxArray_real_T *K;
  double b_z_data[51];
  double b_h_u_data[51];
  double r_data[51];
  double b_z_all[2];
  emxArray_real_T *b_H_xm_tmp;
  double dv12[4];
  double q_tmp[4];
  double f_xt[6];
  emxArray_real_T *g_xt;
  double dqtmp[3];
  double qOld_tmp[4];
  double dv13[4];
  double q31;
  double q32;
  double q33;
  double normq;
  emxArray_real_T *Jtmp;
  emxArray_int32_T *r2;
  emxArray_real_T *r3;
  emxArray_real_T *h_xt;
  double z_all_data[48];
  double rhoInit;
  double rhoSigma;
  static const signed char y_To_x_c[84] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  int d_tmp_data[16];
  boolean_T exitg1;
  boolean_T guard1 = false;
  double c_fq_cw0[4];
  double b_x_i[3];
  signed char e_tmp_data[32];
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
  memcpy(&updateVect_out[0], &updateVect[0], sizeof(double) << 4);

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
  if (!b_initialized_not_empty) {
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

    b_initialized_not_empty = true;
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
  while ((!exitg2) && (ii < 17)) {
    guard2 = false;
    if (updateVect[ii - 1] == 1.0) {
      idx++;
      ii_data[idx - 1] = (signed char)ii;
      if (idx >= 16) {
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

  h_u_size_idx_0 = 3 * ii_size_idx_0;
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

    ar = k * 3 - 2;
    if (ar > k * 3) {
      ar = 0;
    } else {
      ar--;
    }

    ib = i15 - i14;
    for (i15 = 0; i15 <= ib; i15++) {
      z_data[ar + i15] = z_all[(i14 + i15) - 1];
    }

    y = ((double)indMeas_data[k - 1] - 1.0) / numPointsPerAnchor;
    x = floor(y);
    indAnchor = floor(y) + 1.0;
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
    b_fq_cw0[0] = fq_cw0;
    b_fq_cw0[1] = fq_cw1;
    b_fq_cw0[2] = fq_cw2;
    b_fq_cw0[3] = fq_cw3;
    RotFromQuatJ(b_fq_cw0, h_u_To_h_c);
    for (i14 = 0; i14 < 3; i14++) {
      for (i15 = 0; i15 < 3; i15++) {
        fR_wc[i15 + 3 * i14] = h_u_To_h_c[i14 + 3 * i15];
      }
    }

    featureOffset = b_mod((double)indMeas_data[k - 1] - 1.0, numPointsPerAnchor)
      + 1.0;
    current_anchor_u_idx_0 = anchor_u->data[(int)((((x + 1.0) *
      numPointsPerAnchor - numPointsPerAnchor) + featureOffset) * 2.0 - 1.0) - 1];
    current_anchor_u_idx_1 = anchor_u->data[(int)((((x + 1.0) *
      numPointsPerAnchor - numPointsPerAnchor) + featureOffset) * 2.0) - 1];
    mt[0] = (-Cx + current_anchor_u_idx_0) / f;
    mt[1] = (-Cy + current_anchor_u_idx_1) / f;
    mt[2] = 1.0;
    B = norm(mt);
    for (i14 = 0; i14 < 3; i14++) {
      mt[i14] /= B;
    }

    c_xt = b_xt->data[(int)(((13.0 + indAnchor * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    for (i14 = 0; i14 < 3; i14++) {
      d0 = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        d0 += fR_wc[i14 + 3 * i15] * mt[i15];
      }

      d_xt[i14] = c_xt * (fp[i14] - b_xt->data[i14]) + d0;
    }

    for (i14 = 0; i14 < 3; i14++) {
      e_xt[i14] = d_xt[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      h_ci[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        h_ci[i14] += R_cw[i14 + 3 * i15] * e_xt[i15];
      }
    }

    h_ui[0] = Cx + f * (h_ci[0] / h_ci[2]);
    h_ui[1] = Cy + f * (h_ci[1] / h_ci[2]);
    h_ui[2] = b_xt->data[(int)(((13.0 + indAnchor * (7.0 + numPointsPerAnchor))
      - numPointsPerAnchor) + featureOffset) - 1] * f * baseline / h_ci[2];

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
    //          H_ix=[-R_cw*rho,zeros(3),zeros(3,6)];
    //  correct
    //  derivatives of (1) with respect to anchorstates
    for (i14 = 0; i14 < 3; i14++) {
      b_y[i14] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        b_y[i14] += fR_wc[i14 + 3 * i15] * mt[i15];
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
    for (ar = 0; ar < ib; ar++) {
      tmp_data[ar] = i14 + ar;
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
    for (ar = 0; ar < ib; ar++) {
      b_tmp_data[ar] = i14 + ar;
    }

    c_xt = b_xt->data[(int)(((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    for (ar = 0; ar < 3; ar++) {
      for (idx = 0; idx < 3; idx++) {
        b_R_cw[idx + 3 * ar] = -R_cw[idx + 3 * ar] * c_xt;
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
    for (ar = 0; ar < 6; ar++) {
      for (idx = 0; idx < 3; idx++) {
        b_R_cw[idx + 3 * (ar + 6)] = 0.0;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (idx = 0; idx < 12; idx++) {
        b_h_u_To_h_c[ar + 3 * idx] = 0.0;
        for (ii = 0; ii < 3; ii++) {
          b_h_u_To_h_c[ar + 3 * idx] += h_u_To_h_c[ar + 3 * ii] * b_R_cw[ii + 3 *
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
    d1 = (6.0 + numPointsPerAnchor) * (x + 1.0);
    if (d0 > d1) {
      ar = 0;
      idx = 0;
    } else {
      ar = (int)d0 - 1;
      idx = (int)d1;
    }

    ib = i15 - i14;
    for (ii = 0; ii < ib; ii++) {
      b_tmp_data[ii] = i14 + ii;
    }

    ii = r1->size[0];
    r1->size[0] = idx - ar;
    emxEnsureCapacity((emxArray__common *)r1, ii, (int)sizeof(int));
    ib = idx - ar;
    for (idx = 0; idx < ib; idx++) {
      r1->data[idx] = ar + idx;
    }

    c_xt = b_xt->data[(int)(((13.0 + (x + 1.0) * (7.0 + numPointsPerAnchor)) -
      numPointsPerAnchor) + featureOffset) - 1];
    dv9[0] = 0.0;
    dv9[3] = -b_y[2];
    dv9[6] = b_y[1];
    dv9[1] = b_y[2];
    dv9[4] = 0.0;
    dv9[7] = -b_y[0];
    dv9[2] = -b_y[1];
    dv9[5] = b_y[0];
    dv9[8] = 0.0;
    for (ar = 0; ar < 3; ar++) {
      for (idx = 0; idx < 3; idx++) {
        c_R_cw[idx + 3 * ar] = -R_cw[idx + 3 * ar];
      }

      e_xt[ar] = fp[ar] - b_xt->data[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      d_xt[ar] = 0.0;
      for (idx = 0; idx < 3; idx++) {
        d_xt[ar] += R_cw[ar + 3 * idx] * e_xt[idx];
      }

      for (idx = 0; idx < 3; idx++) {
        d_R_cw[ar + 3 * idx] = 0.0;
        for (ii = 0; ii < 3; ii++) {
          d_R_cw[ar + 3 * idx] += c_R_cw[ar + 3 * ii] * dv9[ii + 3 * idx];
        }

        e_R_cw[idx + 3 * ar] = R_cw[idx + 3 * ar] * c_xt;
      }
    }

    for (ar = 0; ar < 3; ar++) {
      for (idx = 0; idx < 3; idx++) {
        e_R_cw[idx + 3 * (ar + 3)] = d_R_cw[idx + 3 * ar];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      e_R_cw[18 + ar] = d_xt[ar];
    }

    for (ar = 0; ar < 3; ar++) {
      for (idx = 0; idx < 7; idx++) {
        c_h_u_To_h_c[ar + 3 * idx] = 0.0;
        for (ii = 0; ii < 3; ii++) {
          c_h_u_To_h_c[ar + 3 * idx] += h_u_To_h_c[ar + 3 * ii] * e_R_cw[ii + 3 *
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
  emxInit_real_T(&b, 2);
  if (ii_size_idx_0 > 0) {
    for (i14 = 0; i14 < 3; i14++) {
      for (i15 = 0; i15 < 3; i15++) {
        Hg->data[i15 + Hg->size[0] * (3 + i14)] = 0.0;
      }
    }

    eye((double)ii_size_idx_0, J2);
    power(imNoise, dv10);
    b_diag(dv10, dv11);
    kron(J2->data, J2->size, dv11, c_tmp_data, tmp_size);
    ib = tmp_size[0] * tmp_size[1];
    for (i14 = 0; i14 < ib; i14++) {
      R_data[i14] = c_tmp_data[i14];
    }

    R_size[0] = tmp_size[0];
    R_size[1] = tmp_size[1];
    ib = tmp_size[0] * tmp_size[1];
    for (i14 = 0; i14 < ib; i14++) {
      b_R_data[i14] = R_data[i14];
    }

    emxInit_real_T(&H, 2);
    blkdiag(b_R_data, R_size, R_data, tmp_size);
    ib = ii_size_idx_0 * 3 - 1;
    br = ii_size_idx_0 * 3;
    ii = H_xm_tmp->size[1] - 1;
    i14 = H->size[0] * H->size[1];
    H->size[0] = br + 3;
    H->size[1] = ii + 13;
    emxEnsureCapacity((emxArray__common *)H, i14, (int)sizeof(double));
    for (i14 = 0; i14 < 12; i14++) {
      for (i15 = 0; i15 < br; i15++) {
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
        H->data[(i15 + br) + H->size[0] * i14] = Hg->data[i15 + Hg->size[0] *
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
        br = P_apr->size[1];
        for (i15 = 0; i15 < br; i15++) {
          H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] = 0.0;
          ii = H->size[1];
          for (ar = 0; ar < ii; ar++) {
            H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] += H->data[i14 +
              H->size[0] * ar] * P_apr->data[ar + P_apr->size[0] * i15];
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

    i14 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i14, (int)sizeof(double));
    ib = H->size[0];
    for (i14 = 0; i14 < ib; i14++) {
      br = H->size[1];
      for (i15 = 0; i15 < br; i15++) {
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
        br = b->size[1];
        for (i15 = 0; i15 < br; i15++) {
          J2->data[i14 + J2->size[0] * i15] = 0.0;
          ii = H_xm_tmp->size[1];
          for (ar = 0; ar < ii; ar++) {
            J2->data[i14 + J2->size[0] * i15] += H_xm_tmp->data[i14 +
              H_xm_tmp->size[0] * ar] * b->data[ar + b->size[0] * i15];
          }
        }
      }
    } else {
      k = H_xm_tmp->size[1];
      a[0] = (signed char)H_xm_tmp->size[0];
      a[1] = (signed char)b->size[1];
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

      ii = H_xm_tmp->size[0] * (b->size[1] - 1);
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
          if (b->data[ib] != 0.0) {
            ia = ar;
            i15 = idx + m;
            for (ic = idx; ic + 1 <= i15; ic++) {
              ia++;
              J2->data[ic] += b->data[ib] * H_xm_tmp->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
      }
    }

    tmp_size[0] = J2->size[0];
    tmp_size[1] = J2->size[1];
    ib = J2->size[0] * J2->size[1];
    for (i14 = 0; i14 < ib; i14++) {
      c_R_data = J2->data[i14] + R_data[i14];
      R_data[i14] = c_R_data;
    }

    i14 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i14, (int)sizeof(double));
    ib = H->size[0];
    for (i14 = 0; i14 < ib; i14++) {
      br = H->size[1];
      for (i15 = 0; i15 < br; i15++) {
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
        br = b->size[1];
        for (i15 = 0; i15 < br; i15++) {
          H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] = 0.0;
          ii = P_apr->size[1];
          for (ar = 0; ar < ii; ar++) {
            H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] += P_apr->data[i14 +
              P_apr->size[0] * ar] * b->data[ar + b->size[0] * i15];
          }
        }
      }
    } else {
      k = P_apr->size[1];
      a[0] = P_apr->size[0];
      a[1] = b->size[1];
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
        ii = P_apr->size[0] * (b->size[1] - 1);
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
            if (b->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                H_xm_tmp->data[ic] += b->data[ib] * P_apr->data[ia - 1];
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
    mrdivide(H_xm_tmp, R_data, tmp_size, K);

    //  K = p_km1_k*H'*inv(H*p_km1_k*H' + R);
    B = norm(za);
    for (i14 = 0; i14 < z_size_idx_0; i14++) {
      b_z_data[i14] = z_data[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      b_z_data[i14 + z_size_idx_0] = za[i14] / B;
    }

    for (i14 = 0; i14 < h_u_size_idx_0; i14++) {
      b_h_u_data[i14] = h_u_data[i14];
    }

    for (i14 = 0; i14 < 3; i14++) {
      b_h_u_data[i14 + h_u_size_idx_0] = 0.0;
    }

    ib = z_size_idx_0 + 3;
    for (i14 = 0; i14 < ib; i14++) {
      r_data[i14] = b_z_data[i14] - b_h_u_data[i14];
    }

    for (h_u_size_idx_0 = 1; h_u_size_idx_0 - 1 < ii_size_idx_0; h_u_size_idx_0
         ++) {
      ii = h_u_size_idx_0 * 3 - 2;
      idx = h_u_size_idx_0 * 3 - 2;
      br = h_u_size_idx_0 * 3 - 2;
      for (i14 = 0; i14 < 2; i14++) {
        b_z_all[i14] = r_data[i14 + ii];
        for (i15 = 0; i15 < 2; i15++) {
          b_fq_cw0[i15 + (i14 << 1)] = R_data[(i15 + idx) + tmp_size[0] * (i14 +
            br)];
        }
      }

      b_mrdivide(b_z_all, b_fq_cw0, a);
      ii = h_u_size_idx_0 * 3 - 2;

      //  mahalanobis
      // if mal>55.76
      d0 = 0.0;
      for (i14 = 0; i14 < 2; i14++) {
        d0 += a[i14] * r_data[i14 + ii];
      }

      if (d0 > 100.0) {
        updateVect_out[indMeas_data[h_u_size_idx_0 - 1] - 1] = 0.0;
      }
    }

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
        br = H->size[1];
        for (i15 = 0; i15 < br; i15++) {
          J2->data[i14 + J2->size[0] * i15] = 0.0;
          ii = K->size[1];
          for (ar = 0; ar < ii; ar++) {
            J2->data[i14 + J2->size[0] * i15] += K->data[i14 + K->size[0] * ar] *
              H->data[ar + H->size[0] * i15];
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
        br = P_apr->size[1];
        for (i15 = 0; i15 < br; i15++) {
          b_H_xm_tmp->data[i14 + b_H_xm_tmp->size[0] * i15] = 0.0;
          ii = H_xm_tmp->size[1];
          for (ar = 0; ar < ii; ar++) {
            b_H_xm_tmp->data[i14 + b_H_xm_tmp->size[0] * i15] += H_xm_tmp->
              data[i14 + H_xm_tmp->size[0] * ar] * P_apr->data[ar + P_apr->size
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
        br = b_H_xm_tmp->size[0];
        for (i15 = 0; i15 < br; i15++) {
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
        br = (int)a[0];
        for (i15 = 0; i15 < br; i15++) {
          P_apr->data[i15 + P_apr->size[0] * i14] = 0.0;
        }
      }

      if ((H_xm_tmp->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = H_xm_tmp->size[0] * (b->size[1] - 1);
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
            if (b->data[ib] != 0.0) {
              ia = ar;
              i15 = idx + m;
              for (ic = idx; ic + 1 <= i15; ic++) {
                ia++;
                P_apr->data[ic] += b->data[ib] * H_xm_tmp->data[ia - 1];
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
      e_xt[i14] = h_u_apr->data[3 + i14];
    }

    quatPlusThetaJ(e_xt, dv12);
    quatmultJ(dv12, *(double (*)[4])&b_xt->data[3], q_tmp);
    for (i14 = 0; i14 < 4; i14++) {
      b_xt->data[3 + i14] = q_tmp[i14];
    }

    for (i14 = 0; i14 < 6; i14++) {
      f_xt[i14] = b_xt->data[7 + i14] + h_u_apr->data[6 + i14];
    }

    for (i14 = 0; i14 < 6; i14++) {
      b_xt->data[7 + i14] = f_xt[i14];
    }

    ii = 0;
    emxInit_real_T(&g_xt, 2);
    while (ii <= (int)numAnchors - 1) {
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
      d0 = (13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor)) - (6.0 +
        numPointsPerAnchor);
      d1 = (13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor)) -
        numPointsPerAnchor;
      if (d0 > d1) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
        i15 = (int)d1;
      }

      d0 = (1.0 + (double)ii) * ((7.0 + numPointsPerAnchor) - numPointsPerAnchor);
      ar = g_xt->size[0] * g_xt->size[1];
      g_xt->size[0] = 1;
      g_xt->size[1] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)g_xt, ar, (int)sizeof(double));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        g_xt->data[g_xt->size[0] * i15] = b_xt->data[i14 + i15];
      }

      for (i14 = 0; i14 < 7; i14++) {
        anchor_pose->data[(int)(d0 + (-6.0 + (double)i14)) - 1] = g_xt->data[i14];
      }

      b_xt->data[(int)(((13.0 + (1.0 + (double)ii) * (7.0 + numPointsPerAnchor))
                        - numPointsPerAnchor) + 1.0) - 1] += h_u_apr->data[(int)
        (((12.0 + (1.0 + (double)ii) * (6.0 + numPointsPerAnchor)) -
          numPointsPerAnchor) + 1.0) - 1];
      ii++;
    }

    emxFree_real_T(&g_xt);
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
  h_u_size_idx_0 = 0;
  emxInit_real_T(&Jtmp, 2);
  b_emxInit_int32_T(&r2, 2);
  emxInit_real_T(&r3, 2);
  emxInit_real_T(&h_xt, 2);
  while (h_u_size_idx_0 <= (int)numAnchors - 1) {
    if (updateVect[h_u_size_idx_0] == 2.0) {
      d0 = (1.0 + (double)h_u_size_idx_0) * numPointsPerAnchor * 3.0 -
        (numPointsPerAnchor * 3.0 - 1.0);
      d1 = (1.0 + (double)h_u_size_idx_0) * numPointsPerAnchor * 3.0;
      if (d0 > d1) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
        i15 = (int)d1;
      }

      d0 = (1.0 + (double)h_u_size_idx_0) * numPointsPerAnchor * 2.0 -
        (numPointsPerAnchor * 2.0 - 1.0);
      d1 = (1.0 + (double)h_u_size_idx_0) * numPointsPerAnchor * 2.0;
      if (d0 > d1) {
        ar = 0;
        idx = 0;
      } else {
        ar = (int)d0 - 1;
        idx = (int)d1;
      }

      ii = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = idx - ar;
      emxEnsureCapacity((emxArray__common *)r2, ii, (int)sizeof(int));
      ib = idx - ar;
      for (idx = 0; idx < ib; idx++) {
        r2->data[r2->size[0] * idx] = ar + idx;
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

      if (z_all[i14 + 2] > -50.0) {
        rhoInit = z_all[i14 + 2] / (baseline * f);
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

      ar = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)r2, ar, (int)sizeof(int));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        r2->data[r2->size[0] * i15] = i14 + i15;
      }

      ib = r2->size[0] * r2->size[1];
      for (i14 = 0; i14 < ib; i14++) {
        h_u_apr->data[r2->data[i14]] = rhoInit;
      }

      d0 = (13.0 + (1.0 + (double)h_u_size_idx_0) * (7.0 + numPointsPerAnchor))
        - (6.0 + numPointsPerAnchor);
      d1 = 13.0 + (1.0 + (double)h_u_size_idx_0) * (7.0 + numPointsPerAnchor);
      if (d0 > d1) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
        i15 = (int)d1;
      }

      ar = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)r2, ar, (int)sizeof(int));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        r2->data[r2->size[0] * i15] = i14 + i15;
      }

      ib = r2->size[1];
      for (i14 = 0; i14 < ib; i14++) {
        b_xt->data[r2->data[r2->size[0] * i14]] = h_u_apr->data[(*(int (*)[2])
          r2->size)[0] * i14];
      }

      d0 = (13.0 + (1.0 + (double)h_u_size_idx_0) * (7.0 + numPointsPerAnchor))
        - (6.0 + numPointsPerAnchor);
      d1 = (13.0 + (1.0 + (double)h_u_size_idx_0) * (7.0 + numPointsPerAnchor))
        - numPointsPerAnchor;
      if (d0 > d1) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
        i15 = (int)d1;
      }

      d0 = (1.0 + (double)h_u_size_idx_0) * ((7.0 + numPointsPerAnchor) -
        numPointsPerAnchor);
      ar = h_xt->size[0] * h_xt->size[1];
      h_xt->size[0] = 1;
      h_xt->size[1] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)h_xt, ar, (int)sizeof(double));
      ib = i15 - i14;
      for (i15 = 0; i15 < ib; i15++) {
        h_xt->data[h_xt->size[0] * i15] = b_xt->data[i14 + i15];
      }

      for (i14 = 0; i14 < 7; i14++) {
        anchor_pose->data[(int)(d0 + (-6.0 + (double)i14)) - 1] = h_xt->data[i14];
      }

      // now create J and multiply with P
      eye(12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)h_u_size_idx_0) -
           1.0), H_xm_tmp);
      eye((numAnchors - (1.0 + (double)h_u_size_idx_0)) * (6.0 +
           numPointsPerAnchor), J2);

      // J=zeros(144);
      i14 = r3->size[0] * r3->size[1];
      r3->size[0] = (int)(6.0 + numPointsPerAnchor);
      r3->size[1] = (int)(6.0 + numPointsPerAnchor);
      emxEnsureCapacity((emxArray__common *)r3, i14, (int)sizeof(double));
      ib = (int)(6.0 + numPointsPerAnchor) * (int)(6.0 + numPointsPerAnchor);
      for (i14 = 0; i14 < ib; i14++) {
        r3->data[i14] = 0.0;
      }

      b_blkdiag(H_xm_tmp, r3, J2, Jtmp);
      d0 = (12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)h_u_size_idx_0))
        - ((6.0 + numPointsPerAnchor) - 1.0);
      d1 = 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)h_u_size_idx_0);
      if (d0 > d1) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
        i15 = (int)d1;
      }

      ar = r1->size[0];
      r1->size[0] = i15 - i14;
      emxEnsureCapacity((emxArray__common *)r1, ar, (int)sizeof(int));
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
      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)h_u_size_idx_0)
             - 1.0)) + 1.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)h_u_size_idx_0))
      {
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
        br = J2->size[0];
        for (ar = 0; ar < br; ar++) {
          P_apr->data[ar + P_apr->size[0] * (i14 + i15)] = J2->data[ar +
            J2->size[0] * i15];
        }
      }

      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)h_u_size_idx_0)
             - 1.0)) + 1.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)h_u_size_idx_0))
      {
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
        br = J2->size[0];
        for (ar = 0; ar < br; ar++) {
          P_apr->data[(i14 + ar) + P_apr->size[0] * i15] = J2->data[ar +
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
          br = P_apr->size[1];
          for (i15 = 0; i15 < br; i15++) {
            H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] = 0.0;
            ii = Jtmp->size[1];
            for (ar = 0; ar < ii; ar++) {
              H_xm_tmp->data[i14 + H_xm_tmp->size[0] * i15] += Jtmp->data[i14 +
                Jtmp->size[0] * ar] * P_apr->data[ar + P_apr->size[0] * i15];
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

      i14 = b->size[0] * b->size[1];
      b->size[0] = Jtmp->size[1];
      b->size[1] = Jtmp->size[0];
      emxEnsureCapacity((emxArray__common *)b, i14, (int)sizeof(double));
      ib = Jtmp->size[0];
      for (i14 = 0; i14 < ib; i14++) {
        br = Jtmp->size[1];
        for (i15 = 0; i15 < br; i15++) {
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
          br = b->size[1];
          for (i15 = 0; i15 < br; i15++) {
            P_apr->data[i14 + P_apr->size[0] * i15] = 0.0;
            ii = H_xm_tmp->size[1];
            for (ar = 0; ar < ii; ar++) {
              P_apr->data[i14 + P_apr->size[0] * i15] += H_xm_tmp->data[i14 +
                H_xm_tmp->size[0] * ar] * b->data[ar + b->size[0] * i15];
            }
          }
        }
      } else {
        k = H_xm_tmp->size[1];
        a[0] = (unsigned int)H_xm_tmp->size[0];
        a[1] = (unsigned int)b->size[1];
        m = H_xm_tmp->size[0];
        i14 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = (int)a[0];
        P_apr->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i14, (int)sizeof(double));
        ib = (int)a[1];
        for (i14 = 0; i14 < ib; i14++) {
          br = (int)a[0];
          for (i15 = 0; i15 < br; i15++) {
            P_apr->data[i15 + P_apr->size[0] * i14] = 0.0;
          }
        }

        if ((H_xm_tmp->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = H_xm_tmp->size[0] * (b->size[1] - 1);
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
              if (b->data[ib] != 0.0) {
                ia = ar;
                i15 = idx + m;
                for (ic = idx; ic + 1 <= i15; ic++) {
                  ia++;
                  P_apr->data[ic] += b->data[ib] * H_xm_tmp->data[ia - 1];
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
      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)h_u_size_idx_0)
             - 1.0)) + 7.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)h_u_size_idx_0))
      {
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
        br = J2->size[0];
        for (ar = 0; ar < br; ar++) {
          P_apr->data[ar + P_apr->size[0] * (i14 + i15)] = J2->data[ar +
            J2->size[0] * i15];
        }
      }

      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)h_u_size_idx_0)
             - 1.0)) + 7.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)h_u_size_idx_0))
      {
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
        br = J2->size[0];
        for (ar = 0; ar < br; ar++) {
          P_apr->data[(i14 + ar) + P_apr->size[0] * i15] = J2->data[ar +
            J2->size[0] * i15];
        }
      }

      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)h_u_size_idx_0)
             - 1.0)) + 7.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)h_u_size_idx_0))
      {
        i14 = 0;
      } else {
        i14 = (int)d0 - 1;
      }

      d0 = (12.0 + (6.0 + numPointsPerAnchor) * ((1.0 + (double)h_u_size_idx_0)
             - 1.0)) + 7.0;
      if (d0 > 12.0 + (6.0 + numPointsPerAnchor) * (1.0 + (double)h_u_size_idx_0))
      {
        i15 = 0;
      } else {
        i15 = (int)d0 - 1;
      }

      y = rhoSigma * rhoSigma;
      eye(numPointsPerAnchor, b);
      ib = b->size[1];
      for (ar = 0; ar < ib; ar++) {
        br = b->size[0];
        for (idx = 0; idx < br; idx++) {
          P_apr->data[(i14 + idx) + P_apr->size[0] * (i15 + ar)] = y * b->
            data[idx + b->size[0] * ar];
        }
      }

      d0 = (1.0 + (double)h_u_size_idx_0) * numPointsPerAnchor -
        (numPointsPerAnchor - 1.0);
      d1 = (1.0 + (double)h_u_size_idx_0) * numPointsPerAnchor;
      if (d0 > d1) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
        i15 = (int)d1;
      }

      ib = i15 - i14;
      for (ar = 0; ar < ib; ar++) {
        d_tmp_data[ar] = i14 + ar;
      }

      ib = i15 - i14;
      for (i14 = 0; i14 < ib; i14++) {
        updateVect_out[d_tmp_data[i14]] = 1.0;
      }
    }

    h_u_size_idx_0++;
  }

  emxFree_real_T(&h_xt);
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
  while ((!exitg1) && (ii < 17)) {
    guard1 = false;
    if (updateVect_out[ii - 1] == 1.0) {
      idx++;
      ii_data[idx - 1] = (signed char)ii;
      if (idx >= 16) {
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
    mt[0] = (-Cx + current_anchor_u_idx_0) / f;
    mt[1] = (-Cy + current_anchor_u_idx_1) / f;
    mt[2] = 1.0;
    B = norm(mt);
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
      e_xt[i14] = mt[i14] / B;
    }

    for (i14 = 0; i14 < 3; i14++) {
      d0 = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        d0 += h_u_To_h_c[i15 + 3 * i14] * e_xt[i15];
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
    for (ar = 0; ar < ib; ar++) {
      e_tmp_data[ar] = (signed char)(i14 + ar);
    }

    b_Cx[0] = Cx + f * (h_ci[0] / h_ci[2]);
    b_Cx[1] = Cy + f * (h_ci[1] / h_ci[2]);
    ib = i15 - i14;
    for (i14 = 0; i14 < ib; i14++) {
      h_u_apo->data[e_tmp_data[i14]] = b_Cx[i14];
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
//                const double x_data[]
//                int ix0
// Return Type  : double
//
static double b_eml_xnrm2(int n, const double x_data[], int ix0)
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
    y = fabs(x_data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x_data[k - 1]);
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
//                double x_data[]
//                int ix0
//                int iy0
// Return Type  : void
//
static void b_eml_xswap(int n, double x_data[], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = x_data[ix];
    x_data[ix] = x_data[iy];
    x_data[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
static void b_eml_xtrsm(int m, int n, const double A_data[], int lda,
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
        if (A_data[k + jAcol] != 0.0) {
          for (i = 0; i + 1 <= m; i++) {
            B->data[i + jBcol] -= A_data[k + jAcol] * B->data[i + kBcol];
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
// Arguments    : const double varargin_1_data[]
//                const int varargin_1_size[2]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
static void blkdiag(const double varargin_1_data[], const int varargin_1_size[2],
                    double y_data[], int y_size[2])
{
  int loop_ub;
  int i3;
  int unnamed_idx_0;
  int i4;
  int i5;
  int i6;
  int tmp_data[51];
  int b_tmp_data[51];
  static const double varargin_2[9] = { 1.0E-5, 0.0, 0.0, 0.0, 1.0E-5, 0.0, 0.0,
    0.0, 1.0E-5 };

  y_size[0] = (signed char)(varargin_1_size[0] + 3);
  y_size[1] = (signed char)(varargin_1_size[1] + 3);
  loop_ub = (signed char)(varargin_1_size[0] + 3) * (signed char)
    (varargin_1_size[1] + 3);
  for (i3 = 0; i3 < loop_ub; i3++) {
    y_data[i3] = 0.0;
  }

  loop_ub = varargin_1_size[1];
  for (i3 = 0; i3 < loop_ub; i3++) {
    unnamed_idx_0 = varargin_1_size[0];
    for (i4 = 0; i4 < unnamed_idx_0; i4++) {
      y_data[i4 + y_size[0] * i3] = varargin_1_data[i4 + varargin_1_size[0] * i3];
    }
  }

  if (varargin_1_size[0] + 1 > varargin_1_size[0] + 3) {
    i3 = 1;
    i4 = 0;
  } else {
    i3 = varargin_1_size[0] + 1;
    i4 = varargin_1_size[0] + 3;
  }

  if (varargin_1_size[1] + 1 > varargin_1_size[1] + 3) {
    i5 = 1;
    i6 = 0;
  } else {
    i5 = varargin_1_size[1] + 1;
    i6 = varargin_1_size[1] + 3;
  }

  loop_ub = (i4 - i3) + 1;
  for (unnamed_idx_0 = 0; unnamed_idx_0 < loop_ub; unnamed_idx_0++) {
    tmp_data[unnamed_idx_0] = (i3 + unnamed_idx_0) - 1;
  }

  loop_ub = (i6 - i5) + 1;
  for (unnamed_idx_0 = 0; unnamed_idx_0 < loop_ub; unnamed_idx_0++) {
    b_tmp_data[unnamed_idx_0] = (i5 + unnamed_idx_0) - 1;
  }

  unnamed_idx_0 = (i4 - i3) + 1;
  loop_ub = (i6 - i5) + 1;
  for (i3 = 0; i3 < loop_ub; i3++) {
    for (i4 = 0; i4 < unnamed_idx_0; i4++) {
      y_data[tmp_data[i4] + y_size[0] * b_tmp_data[i3]] = varargin_2[i4 +
        unnamed_idx_0 * i3];
    }
  }
}

//
// Arguments    : int n
//                const double x_data[]
//                int ix0
// Return Type  : double
//
static double c_eml_xnrm2(int n, const double x_data[], int ix0)
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
    y = fabs(x_data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x_data[k - 1]);
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
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const emxArray_real_T *B
//                emxArray_real_T *X
// Return Type  : void
//
static void eml_lusolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *X)
{
  int b_A_size[2];
  int info;
  int jp;
  double b_A_data[2601];
  int ipiv_size[2];
  int ipiv_data[51];
  int xi;
  double temp;
  b_A_size[0] = A_size[0];
  b_A_size[1] = A_size[1];
  info = A_size[0] * A_size[1];
  for (jp = 0; jp < info; jp++) {
    b_A_data[jp] = A_data[jp];
  }

  eml_xgetrf(A_size[1], A_size[1], b_A_data, b_A_size, A_size[1], ipiv_data,
             ipiv_size, &info);
  jp = X->size[0] * X->size[1];
  X->size[0] = B->size[0];
  X->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)X, jp, (int)sizeof(double));
  info = B->size[0] * B->size[1];
  for (jp = 0; jp < info; jp++) {
    X->data[jp] = B->data[jp];
  }

  eml_xtrsm(B->size[0], A_size[1], b_A_data, A_size[1], X, B->size[0]);
  b_eml_xtrsm(B->size[0], A_size[1], b_A_data, A_size[1], X, B->size[0]);
  for (info = A_size[1] - 2; info + 1 > 0; info--) {
    if (ipiv_data[info] != info + 1) {
      jp = ipiv_data[info] - 1;
      for (xi = 0; xi + 1 <= B->size[0]; xi++) {
        temp = X->data[xi + X->size[0] * info];
        X->data[xi + X->size[0] * info] = X->data[xi + X->size[0] * jp];
        X->data[xi + X->size[0] * jp] = temp;
      }
    }
  }
}

//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                double C_data[]
//                int ic0
//                int ldc
//                double work_data[]
// Return Type  : void
//
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, double C_data[],
  int ic0, int ldc, double work_data[])
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
    while ((lastv > 0) && (C_data[i - 2] == 0.0)) {
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
          if (C_data[ia - 1] != 0.0) {
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
          c += C_data[ia - 1] * C_data[ix - 1];
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
            C_data[ia] += C_data[ix - 1] * c;
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
//                double x_data[]
//                int ix0
// Return Type  : double
//
static double eml_matlab_zlarfg(int n, double *alpha1, double x_data[], int ix0)
{
  double tau;
  double xnorm;
  int knt;
  int i18;
  int k;
  tau = 0.0;
  if (n <= 0) {
  } else {
    xnorm = b_eml_xnrm2(n - 1, x_data, ix0);
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
            x_data[k - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = b_eml_xnrm2(n - 1, x_data, ix0);
        xnorm = rt_hypotd(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i18 = (ix0 + n) - 2;
        for (k = ix0; k <= i18; k++) {
          x_data[k - 1] *= *alpha1;
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
          x_data[k - 1] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const emxArray_real_T *B
//                emxArray_real_T *Y
// Return Type  : void
//
static void eml_qrsolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *Y)
{
  int b_A_size[2];
  int j;
  int k;
  double b_A_data[2601];
  emxArray_real_T *b_B;
  int jpvt_size[2];
  int jpvt_data[51];
  int tau_size[1];
  double tau_data[51];
  int rankR;
  int m;
  int nb;
  int mn;
  double wj;
  int i;
  b_A_size[0] = A_size[0];
  b_A_size[1] = A_size[1];
  j = A_size[0] * A_size[1];
  for (k = 0; k < j; k++) {
    b_A_data[k] = A_data[k];
  }

  emxInit_real_T(&b_B, 2);
  eml_xgeqp3(b_A_data, b_A_size, tau_data, tau_size, jpvt_data, jpvt_size);
  rankR = rankFromQR(b_A_data, b_A_size);
  k = b_B->size[0] * b_B->size[1];
  b_B->size[0] = B->size[0];
  b_B->size[1] = B->size[1];
  emxEnsureCapacity((emxArray__common *)b_B, k, (int)sizeof(double));
  j = B->size[0] * B->size[1];
  for (k = 0; k < j; k++) {
    b_B->data[k] = B->data[k];
  }

  m = b_A_size[0];
  nb = B->size[1];
  j = b_A_size[0];
  mn = b_A_size[1];
  if (j <= mn) {
    mn = j;
  }

  j = B->size[1];
  k = Y->size[0] * Y->size[1];
  Y->size[0] = b_A_size[1];
  emxEnsureCapacity((emxArray__common *)Y, k, (int)sizeof(double));
  k = Y->size[0] * Y->size[1];
  Y->size[1] = j;
  emxEnsureCapacity((emxArray__common *)Y, k, (int)sizeof(double));
  j *= b_A_size[1];
  for (k = 0; k < j; k++) {
    Y->data[k] = 0.0;
  }

  for (j = 0; j + 1 <= mn; j++) {
    if (tau_data[j] != 0.0) {
      for (k = 0; k + 1 <= nb; k++) {
        wj = b_B->data[j + b_B->size[0] * k];
        for (i = j + 1; i + 1 <= m; i++) {
          wj += b_A_data[i + b_A_size[0] * j] * b_B->data[i + b_B->size[0] * k];
        }

        wj *= tau_data[j];
        if (wj != 0.0) {
          b_B->data[j + b_B->size[0] * k] -= wj;
          for (i = j + 1; i + 1 <= m; i++) {
            b_B->data[i + b_B->size[0] * k] -= b_A_data[i + b_A_size[0] * j] *
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
      Y->data[(jpvt_data[j] + Y->size[0] * k) - 1] /= b_A_data[j + b_A_size[0] *
        j];
      for (i = 0; i + 1 <= j; i++) {
        Y->data[(jpvt_data[i] + Y->size[0] * k) - 1] -= Y->data[(jpvt_data[j] +
          Y->size[0] * k) - 1] * b_A_data[i + b_A_size[0] * j];
      }
    }
  }

  emxFree_real_T(&b_B);
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
// Arguments    : double A_data[]
//                int A_size[2]
//                double tau_data[]
//                int tau_size[1]
//                int jpvt_data[]
//                int jpvt_size[2]
// Return Type  : void
//
static void eml_xgeqp3(double A_data[], int A_size[2], double tau_data[], int
  tau_size[1], int jpvt_data[], int jpvt_size[2])
{
  int m;
  int n;
  int mn;
  int k;
  int itemp;
  double work_data[51];
  double vn1_data[51];
  double vn2_data[51];
  int i;
  int i_i;
  int nmi;
  int mmi;
  double atmp;
  double temp2;
  m = A_size[0];
  n = A_size[1];
  if (A_size[0] <= A_size[1]) {
    mn = A_size[0];
  } else {
    mn = A_size[1];
  }

  tau_size[0] = (signed char)mn;
  eml_signed_integer_colon(A_size[1], jpvt_data, jpvt_size);
  k = (signed char)A_size[1];
  for (itemp = 0; itemp < k; itemp++) {
    work_data[itemp] = 0.0;
  }

  k = 1;
  for (itemp = 0; itemp + 1 <= n; itemp++) {
    vn1_data[itemp] = eml_xnrm2(m, A_data, k);
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
      b_eml_xswap(m, A_data, 1 + m * k, 1 + m * i);
      itemp = jpvt_data[k];
      jpvt_data[k] = jpvt_data[i];
      jpvt_data[i] = itemp;
      vn1_data[k] = vn1_data[i];
      vn2_data[k] = vn2_data[i];
    }

    if (i + 1 < m) {
      atmp = A_data[i_i];
      tau_data[i] = eml_matlab_zlarfg(mmi, &atmp, A_data, i_i + 2);
    } else {
      atmp = A_data[i_i];
      tau_data[i] = b_eml_matlab_zlarfg();
    }

    A_data[i_i] = atmp;
    if (i + 1 < n) {
      atmp = A_data[i_i];
      A_data[i_i] = 1.0;
      eml_matlab_zlarf(mmi, nmi, i_i + 1, tau_data[i], A_data, (i + (i + 1) * m)
                       + 1, m, work_data);
      A_data[i_i] = atmp;
    }

    for (itemp = i + 1; itemp + 1 <= n; itemp++) {
      if (vn1_data[itemp] != 0.0) {
        atmp = fabs(A_data[i + A_size[0] * itemp]) / vn1_data[itemp];
        atmp = 1.0 - atmp * atmp;
        if (atmp < 0.0) {
          atmp = 0.0;
        }

        temp2 = vn1_data[itemp] / vn2_data[itemp];
        temp2 = atmp * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          if (i + 1 < m) {
            vn1_data[itemp] = c_eml_xnrm2(mmi - 1, A_data, (i + m * itemp) + 2);
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
//                double A_data[]
//                int A_size[2]
//                int lda
//                int ipiv_data[]
//                int ipiv_size[2]
//                int *info
// Return Type  : void
//
static void eml_xgetrf(int m, int n, double A_data[], int A_size[2], int lda,
  int ipiv_data[], int ipiv_size[2], int *info)
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
        smax = fabs(A_data[c]);
        for (jA = 1; jA + 1 <= mmj; jA++) {
          ix++;
          s = fabs(A_data[ix]);
          if (s > smax) {
            i = jA;
            smax = s;
          }
        }
      }
    }

    if (A_data[c + i] != 0.0) {
      if (i != 0) {
        ipiv_data[j - 1] = j + i;
        eml_xswap(n, A_data, j, lda, j + i, lda);
      }

      i17 = c + mmj;
      for (i = c + 1; i + 1 <= i17; i++) {
        A_data[i] /= A_data[c];
      }
    } else {
      *info = j;
    }

    i = n - j;
    jA = c + lda;
    jy = c + lda;
    for (b_j = 1; b_j <= i; b_j++) {
      smax = A_data[jy];
      if (A_data[jy] != 0.0) {
        ix = c + 1;
        i17 = mmj + jA;
        for (ijA = 1 + jA; ijA + 1 <= i17; ijA++) {
          A_data[ijA] += A_data[ix] * -smax;
          ix++;
        }
      }

      jy += lda;
      jA += lda;
    }
  }

  if ((*info == 0) && (m <= n) && (!(A_data[(m + A_size[0] * (m - 1)) - 1] !=
        0.0))) {
    *info = m;
  }
}

//
// Arguments    : int n
//                const double x_data[]
//                int ix0
// Return Type  : double
//
static double eml_xnrm2(int n, const double x_data[], int ix0)
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
    y = fabs(x_data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x_data[k - 1]);
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
//                double x_data[]
//                int ix0
//                int incx
//                int iy0
//                int incy
// Return Type  : void
//
static void eml_xswap(int n, double x_data[], int ix0, int incx, int iy0, int
                      incy)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = x_data[ix];
    x_data[ix] = x_data[iy];
    x_data[iy] = temp;
    ix += incx;
    iy += incy;
  }
}

//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
static void eml_xtrsm(int m, int n, const double A_data[], int lda,
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
        if (A_data[k + jAcol] != 0.0) {
          for (i = 0; i + 1 <= m; i++) {
            B->data[i + jBcol] -= A_data[k + jAcol] * B->data[i + kBcol];
          }
        }
      }

      temp = 1.0 / A_data[j + jAcol];
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
//                double K_data[]
//                int K_size[2]
// Return Type  : void
//
static void kron(const double A_data[], const int A_size[2], const double B[9],
                 double K_data[], int K_size[2])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  K_size[0] = (signed char)(A_size[0] * 3);
  K_size[1] = (signed char)(A_size[1] * 3);
  kidx = -1;
  for (b_j1 = 1; b_j1 <= A_size[1]; b_j1++) {
    for (j2 = 0; j2 < 3; j2++) {
      for (i1 = 1; i1 <= A_size[0]; i1++) {
        for (i2 = 0; i2 < 3; i2++) {
          kidx++;
          K_data[kidx] = A_data[(i1 + A_size[0] * (b_j1 - 1)) - 1] * B[i2 + 3 *
            j2];
        }
      }
    }
  }
}

//
// Arguments    : const emxArray_real_T *A
//                const double B_data[]
//                const int B_size[2]
//                emxArray_real_T *y
// Return Type  : void
//
static void mrdivide(const emxArray_real_T *A, const double B_data[], const int
                     B_size[2], emxArray_real_T *y)
{
  int i7;
  double b_B_data[2601];
  int b_B_size[2];
  int loop_ub;
  int A_idx_1;
  int i8;
  emxArray_real_T *b_A;
  emxArray_real_T *c_A;
  emxArray_real_T *r0;
  if (A->size[0] == 0) {
    i7 = y->size[0] * y->size[1];
    y->size[0] = 0;
    emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
    i7 = y->size[0] * y->size[1];
    y->size[1] = (signed char)B_size[0];
    emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
  } else if (B_size[0] == B_size[1]) {
    eml_lusolve(B_data, B_size, A, y);
  } else {
    b_B_size[0] = B_size[1];
    b_B_size[1] = B_size[0];
    loop_ub = B_size[0];
    for (i7 = 0; i7 < loop_ub; i7++) {
      A_idx_1 = B_size[1];
      for (i8 = 0; i8 < A_idx_1; i8++) {
        b_B_data[i8 + b_B_size[0] * i7] = B_data[i7 + B_size[0] * i8];
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
    eml_qrsolve(b_B_data, b_B_size, b_A, r0);
    i7 = y->size[0] * y->size[1];
    y->size[0] = r0->size[1];
    y->size[1] = r0->size[0];
    emxEnsureCapacity((emxArray__common *)y, i7, (int)sizeof(double));
    loop_ub = r0->size[0];
    emxFree_real_T(&b_A);
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
// Arguments    : const double A_data[]
//                const int A_size[2]
// Return Type  : int
//
static int rankFromQR(const double A_data[], const int A_size[2])
{
  int r;
  int minmn;
  int maxmn;
  double tol;
  r = 0;
  if (A_size[0] < A_size[1]) {
    minmn = A_size[0];
    maxmn = A_size[1];
  } else {
    minmn = A_size[1];
    maxmn = A_size[0];
  }

  tol = (double)maxmn * fabs(A_data[0]) * 2.2204460492503131E-16;
  while ((r < minmn) && (fabs(A_data[r + A_size[0] * r]) >= tol)) {
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
// Arguments    : const double updateVect[16]
//                const double z_all[48]
//                const double cameraparams[4]
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[9]
//                const double imNoise[3]
//                double numPointsPerAnchor
//                double numAnchors
//                emxArray_real_T *h_u_apo
//                emxArray_real_T *xt_out
//                double updateVect_out[16]
//                emxArray_real_T *anchor_u_out
//                emxArray_real_T *anchor_pose_out
//                emxArray_real_T *P_apo_out
// Return Type  : void
//
void SLAM(const double updateVect[16], const double z_all[48], const double
          cameraparams[4], double dt, const double processNoise[4], const double
          IMU_measurements[9], const double imNoise[3], double
          numPointsPerAnchor, double numAnchors, emxArray_real_T *h_u_apo,
          emxArray_real_T *xt_out, double updateVect_out[16], emxArray_real_T
          *anchor_u_out, emxArray_real_T *anchor_pose_out, emxArray_real_T
          *P_apo_out)
{
  int i0;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  int loop_ub;
  emxArray_real_T *b_P;
  double b_imNoise[3];
  if (!initialized_not_empty) {
    initialized_not_empty = true;

    //      q1 = QuatFromRotJ(diag([1 -1 -1])); % initialize facing downward
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
    eye(12.0 + numAnchors * (6.0 + numPointsPerAnchor), P);

    //  initial error state covariance
  }

  emxInit_real_T(&b_P, 2);
  SLAM_pred_euler(P, xt, dt, processNoise, IMU_measurements, numAnchors * (6.0 +
    numPointsPerAnchor), b_P);
  i0 = P->size[0] * P->size[1];
  P->size[0] = b_P->size[0];
  P->size[1] = b_P->size[1];
  emxEnsureCapacity((emxArray__common *)P, i0, (int)sizeof(double));
  loop_ub = b_P->size[0] * b_P->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    P->data[i0] = b_P->data[i0];
  }

  emxFree_real_T(&b_P);
  for (i0 = 0; i0 < 3; i0++) {
    b_imNoise[i0] = imNoise[i0];
  }

  SLAM_updIT(P, xt, cameraparams, updateVect, z_all, b_imNoise,
             numPointsPerAnchor, numAnchors, *(double (*)[3])&IMU_measurements[3],
             h_u_apo, updateVect_out, anchor_u_out, anchor_pose_out);
  i0 = xt_out->size[0];
  xt_out->size[0] = xt->size[0];
  emxEnsureCapacity((emxArray__common *)xt_out, i0, (int)sizeof(double));
  loop_ub = xt->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    xt_out->data[i0] = xt->data[i0];
  }

  i0 = P_apo_out->size[0] * P_apo_out->size[1];
  P_apo_out->size[0] = P->size[0];
  P_apo_out->size[1] = P->size[1];
  emxEnsureCapacity((emxArray__common *)P_apo_out, i0, (int)sizeof(double));
  loop_ub = P->size[0] * P->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    P_apo_out->data[i0] = P->data[i0];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
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
