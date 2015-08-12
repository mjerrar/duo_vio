//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 12-Aug-2015 16:47:49
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include <stdio.h>

// Type Definitions
#include <stdio.h>
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

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_boolean_T

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

typedef struct {
  char Name[30];
  char Version[3];
  char Release[8];
  char Date[11];
} struct_T;

typedef struct {
  double RadialDistortion[3];
  double TangentialDistortion[2];
  boolean_T EstimateSkew;
  double NumRadialDistortionCoefficients;
  boolean_T EstimateTangentialDistortion;
  double NumPatterns;
  double IntrinsicMatrix[9];
  double FocalLength[2];
  double PrincipalPoint[2];
  double Skew;
  double MeanReprojectionError;
  double IntrinsicMatrixInternal[9];
  struct_T Version;
} b_struct_T;

typedef struct {
  double Xmap[4];
  double Ymap[4];
  double XmapSingle[4];
  double YmapSingle[4];
  double NewOrigin[2];
} c_struct_T;

typedef struct {
  double T[9];
  double Dimensionality;
} d_struct_T;

typedef struct {
  d_struct_T H1;
  d_struct_T H2;
  double Q[16];
  double XBounds[2];
  double YBounds[2];
  boolean_T Initialized;
  double RectifiedImageSize[2];
} e_struct_T;

typedef struct {
  b_struct_T CameraParameters1;
  b_struct_T CameraParameters2;
  double RotationOfCamera2[9];
  double TranslationOfCamera2[3];
  double FundamentalMatrix[9];
  double EssentialMatrix[9];
  double MeanReprojectionError;
  double NumPatterns;
  double WorldPoints[80];
  char WorldUnits[2];
  c_struct_T RectifyMap1;
  c_struct_T RectifyMap2;
  e_struct_T RectificationParams;
  struct_T Version;
  double r_lr[3];
  double R_lr[9];
  double R_rl[9];
} f_struct_T;

// Named Constants
#define b_gravityUpdate                (false)
#define b_useAirPressure               (false)
#define b_normalGravity                (true)
#define b_numStates                    (12.0)
#define b_numTrackFeatures             (16.0)
#define b_numStatesxt                  (13.0)
#define b_minFeatureThreshold          (2.0)
#define b_sigma_Init                   (1.0E-5)
#define b_maxEKFIterations             (1.0)
#define b_gravAlignNoise               (0.01)

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static f_struct_T cameraparams;
static double delayBuffer_k_0[6];
static double init_counter;
static double P_att[9];
static double x_att[4];
static double numStates;
static double numTrackFeatures;
static double numStatesxt;
static emxArray_real_T *m_vect;
static boolean_T m_vect_not_empty;
static emxArray_real_T *anchorFeatures;
static double minFeatureThreshold;
static double sigma_Init;
static double maxEKFIterations;
static unsigned int state[625];
static boolean_T gravityUpdate;
static double gravAlignNoise;
static boolean_T useAirPressure;
static boolean_T normalGravity;

// Function Declarations
static void Ch_dn_To_h_un(double x, double y, double h_dn_l_To_h_un_l[4]);
static void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P,
  const double z_all_l[32], double c_numStatesxt, double c_numStates, double
  numPointsPerAnchor, const emxArray_real_T *b_anchorFeatures, const
  emxArray_real_T *b_m_vect, const double imNoise[2], const double
  IMU_measurements[23], double height_offset_pressure, double
  validFeatures_data[], int validFeatures_size[1]);
static void QuatFromRotJ(const double R[9], double b_Q[4]);
static void SLAM_free();
static void SLAM_init();
static void SLAM_pred(emxArray_real_T *P_apo, emxArray_real_T *x, double dt,
                      const double processNoise[4], const double
                      IMU_measurements[23], double c_numStates);
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams_r_lr[3], const double cameraparams_R_lr[9], const double
  cameraparams_R_rl[9], double updateVect[16], const double z_all_l[32], const
  double z_all_r[32], const double imNoise[2], const double IMU_measurements[23],
  double numPointsPerAnchor, double numAnchors, double height_offset_pressure,
  emxArray_real_T *h_u_apo, emxArray_real_T *map);
static void SLAM_updIT_free();
static void SLAM_updIT_init();
static boolean_T any(const boolean_T x[16]);
static void b_QuatFromRotJ(double b_Q[4]);
static void b_any(const emxArray_boolean_T *x, boolean_T y[16]);
static void b_blkdiag(const double varargin_1[4], const double varargin_2_data[],
                      const int varargin_2_size[2], double y_data[], int y_size
                      [2]);
static void b_diag(const double v[9], double d[81]);
static void b_dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T
  *x, const double P_xx_apr[144], const emxArray_real_T *Phi, const double b_Q
  [81], emxArray_real_T *x_dot, double P_xx_apr_dot[144], emxArray_real_T
  *Phi_dot);
static double b_eml_matlab_zlarfg();
static void b_eml_xaxpy(int n, double a, const double x[30], int ix0, double y[6],
  int iy0);
static void b_eml_xgeqp3(double A[30], double tau[5], int jpvt[5]);
static double b_eml_xnrm2(int n, const double x_data[], int ix0);
static void b_eml_xswap(int n, double x_data[], int ix0, int iy0);
static void b_eml_xtrsm(int m, int n, const double A_data[], int lda,
  emxArray_real_T *B, int ldb);
static void b_emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);
static void b_eye(const double varargin_1[2], emxArray_real_T *I);
static void b_fprintf();
static void b_getH_R_res(const emxArray_real_T *b_xt, double errorStateSize,
  double stateSize, const double z_all_l[32], const double indMeas_data[], const
  int indMeas_size[1], const emxArray_real_T *map, double numAnchors, double
  numPointsPerAnchor, const emxArray_real_T *anchorIdx, const emxArray_real_T
  *featureAnchorIdx, const emxArray_real_T *b_m_vect, const double imNoise[2],
  const double IMU_measurements[23], double height_offset_pressure, double
  r_data[], int r_size[1], emxArray_real_T *H, double h_u_data[], int h_u_size[1],
  double R_data[], int R_size[2]);
static void b_mrdivide(const double A[2], const double B[4], double y[2]);
static double b_norm(const double x[4]);
static double b_rand();
static void blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
                    *varargin_2, emxArray_real_T *y);
static boolean_T c_any(const boolean_T x[3]);
static void c_eml_rand_mt19937ar_stateful_i();
static void c_eml_xaxpy(int n, double a, const double x[6], int ix0, double y[30],
  int iy0);
static double c_eml_xnrm2(int n, const double x_data[], int ix0);
static double c_fprintf();
static double c_norm(const double x[2]);
static boolean_T d_any(const emxArray_boolean_T *x);
static double d_eml_xnrm2(int n, const double x[30], int ix0);
static void d_fprintf(signed char formatSpec);
static double d_norm(const emxArray_real_T *x);
static void diag(const double v[2], double d[4]);
static int div_nzp_s32_floor(int numerator, int denominator);
static void dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T *x,
                      const emxArray_real_T *P_xx_apr, const emxArray_real_T
                      *Phi, const double b_Q[81], emxArray_real_T *x_dot, double
                      P_xx_apr_dot[144], emxArray_real_T *Phi_dot);
static double e_eml_xnrm2(int n, const double x[5], int ix0);
static double e_fprintf(signed char varargin_1);
static int eml_ixamax(int n, const double x_data[], int ix0);
static void eml_lusolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *X);
static void eml_matlab_zlarf(int m, int n, int iv0, double tau, double C_data[],
  int ic0, int ldc, double work_data[]);
static double eml_matlab_zlarfg(int n, double *alpha1, double x_data[], int ix0);
static void eml_qrsolve(const double A_data[], const int A_size[2], const
  emxArray_real_T *B, emxArray_real_T *Y);
static void eml_signed_integer_colon(int b, int y_data[], int y_size[2]);
static void eml_xaxpy(int n, double a, int ix0, double y[30], int iy0);
static double eml_xdotc(int n, const double x[30], int ix0, const double y[30],
  int iy0);
static void eml_xgeqp3(double A_data[], int A_size[2], double tau_data[], int
  tau_size[1], int jpvt_data[], int jpvt_size[2]);
static void eml_xgetrf(int m, int n, double A_data[], int A_size[2], int lda,
  int ipiv_data[], int ipiv_size[2], int *info);
static double eml_xnrm2(int n, const double x_data[], int ix0);
static void eml_xrotg(double *a, double *b, double *c, double *s);
static void eml_xswap(int n, double x_data[], int ix0, int incx, int iy0, int
                      incy);
static void eml_xtrsm(int m, int n, const double A_data[], int lda,
                      emxArray_real_T *B, int ldb);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
  b_numDimensions);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);
static void eye(double varargin_1, emxArray_real_T *I);
static double f_eml_xnrm2(int n, const double x[30], int ix0);
static void f_fprintf(int formatSpec, int varargin_1);
static void fileManager(FILE * *f, boolean_T *a);
static double g_fprintf(int varargin_1, int varargin_2);
static void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize,
  double stateSize, const double z_all_l[32], double indMeas, const
  emxArray_real_T *map, double numAnchors, double numPointsPerAnchor, const
  emxArray_real_T *anchorIdx, const emxArray_real_T *featureAnchorIdx, const
  emxArray_real_T *b_m_vect, const double imNoise[2], const double
  IMU_measurements[23], double height_offset_pressure, double r_data[], int
  r_size[1], emxArray_real_T *H, double h_u[2], double R_data[], int R_size[2]);
static void getMap(const emxArray_real_T *x, const emxArray_real_T
                   *b_anchorFeatures, const emxArray_real_T *b_m_vect, double
                   c_numTrackFeatures, double stateSize, double
                   numStatesPerAnchorxt, emxArray_real_T *map, emxArray_real_T
                   *anchorInd, emxArray_real_T *featureAnchorInd);
static void h_fprintf();
static double i_fprintf();
static void initializePoint(const emxArray_real_T *b_xt, const double
  cameraparams_r_lr[3], const double cameraparams_R_lr[9], const double z_l[2],
  const double z_r[2], double fp[3], double m_out[3]);
static void kron(const double A_data[], const int A_size[2], const double B[4],
                 double K_data[], int K_size[2]);
static void mrdivide(const emxArray_real_T *A, const double B_data[], const int
                     B_size[2], emxArray_real_T *y);
static double norm(const double x[3]);
static void output_size(const int varargin_1_size[2], const int varargin_2_size
  [2], const int varargin_3_size[2], int *nrows, int *ncols);
static void power(const double a[2], double y[2]);
static void predictMeasurement_left(const double fp_l[3], double h_u_l[2]);
static void predictMeasurement_stereo(const double fp_l[3], const double
  cameraparams_r_lr[3], const double cameraparams_R_rl[9], double h_u_l[2],
  double h_u_r[2]);
static void quatPlusThetaJ(const double dtheta[3], double dq[4]);
static void quatmultJ(const double q[4], const double p[4], double qp[4]);
static int rankFromQR(const double A_data[], const int A_size[2]);
static void repmat(const emxArray_real_T *a, double varargin_1, emxArray_real_T *
                   b);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static double rt_roundd_snf(double u);
static void svd(const double A[30], double U[5]);

// Function Definitions

//
// UNTITLED Summary of this function goes here
//   computs the derivatives with respect to the normalised undistorted point
// Arguments    : double x
//                double y
//                double h_dn_l_To_h_un_l[4]
// Return Type  : void
//
static void Ch_dn_To_h_un(double x, double y, double h_dn_l_To_h_un_l[4])
{
  double x2;
  double y2;
  double a;
  x2 = x * x;
  y2 = y * y;
  a = x2 + y2;
  h_dn_l_To_h_un_l[0] = (((1.0 + -0.414085141240295 * (x2 + y2)) +
    0.236451305145822 * (a * a)) + -0.0871296995623235 * rt_powd_snf(x2 + y2,
    4.0)) + x * ((-0.82817028248059 * x + 0.945805220583288 * x * (x2 + y2)) +
                 -0.697037596498588 * x * rt_powd_snf(x2 + y2, 3.0));
  h_dn_l_To_h_un_l[2] = 2.0 * x * y * ((-0.414085141240295 + 0.472902610291644 *
    (x2 + y2)) + -0.348518798249294 * rt_powd_snf(x2 + y2, 3.0));
  h_dn_l_To_h_un_l[1] = 2.0 * x * y * ((-0.414085141240295 + 0.472902610291644 *
    (x2 + y2)) + -0.348518798249294 * rt_powd_snf(x2 + y2, 3.0));
  a = x2 + y2;
  h_dn_l_To_h_un_l[3] = (((1.0 + -0.414085141240295 * (x2 + y2)) +
    0.236451305145822 * (a * a)) + -0.0871296995623235 * rt_powd_snf(x2 + y2,
    4.0)) + y * ((-0.82817028248059 * y + 0.945805220583288 * y * (x2 + y2)) +
                 -0.697037596498588 * y * rt_powd_snf(x2 + y2, 3.0));

  //  h_dn_l_To_h_un_l(1,1)=1 + k1 * (3*  x2 + y2) + k2 * (5 * x2^2 + 6 * x2 * y2 + y2^2); 
  //  h_dn_l_To_h_un_l(1,2)=2 * x * y * (k1 + 2 * k2 * (x2 + y2));
  //  h_dn_l_To_h_un_l(2,1)=2 * x * y * (k1 + 2 * k2  *(x2 + y2));
  //  h_dn_l_To_h_un_l(2,2)=1 + k1 * (x2 + 3 * y2) + k2 * (x2^2 + 6 * x2 * y2 + 5 * y2^2); 
}

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : emxArray_real_T *b_xt
//                emxArray_real_T *b_P
//                const double z_all_l[32]
//                double c_numStatesxt
//                double c_numStates
//                double numPointsPerAnchor
//                const emxArray_real_T *b_anchorFeatures
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[23]
//                double height_offset_pressure
//                double validFeatures_data[]
//                int validFeatures_size[1]
// Return Type  : void
//
static void OnePointRANSAC_EKF(emxArray_real_T *b_xt, emxArray_real_T *b_P,
  const double z_all_l[32], double c_numStatesxt, double c_numStates, double
  numPointsPerAnchor, const emxArray_real_T *b_anchorFeatures, const
  emxArray_real_T *b_m_vect, const double imNoise[2], const double
  IMU_measurements[23], double height_offset_pressure, double
  validFeatures_data[], int validFeatures_size[1])
{
  emxArray_boolean_T *c_anchorFeatures;
  int numAnchors;
  int i19;
  int loop_ub;
  boolean_T x[16];
  int idx;
  int ii_data[16];
  int ii;
  boolean_T exitg3;
  boolean_T guard2 = false;
  int ar;
  signed char indMeas_data[16];
  emxArray_real_T *K;
  emxArray_real_T *H;
  int numMeas;
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
  emxArray_int32_T *r13;
  emxArray_real_T *b;
  emxArray_real_T *y;
  emxArray_real_T *r14;
  emxArray_int32_T *r15;
  emxArray_int32_T *r16;
  emxArray_real_T *b_x_apo_prev;
  double d_numStatesxt;
  int R_size[2];
  double R_data[36];
  double r[2];
  int unusedU2_size[1];
  double r_data[6];
  int ib;
  int i20;
  int i21;
  int k;
  double a[2];
  int m;
  int ic;
  int br;
  int ia;
  double C_data[36];
  int C_size[2];
  double c_xt[3];
  double dv44[4];
  double dv45[4];
  double e_numStatesxt;
  double d_numStates;
  double d_xt[4];
  double dv46[4];
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
  double b_R_data[1296];
  double unusedU2_data[32];
  int r_size[1];
  double b_r_data[36];
  double b_C_data[1296];
  int b_C_size[2];
  double dv47[4];
  double dv48[4];
  emxArray_real_T *c_a;
  int it;
  emxArray_real_T *b_x_apo;
  emxArray_real_T *f_xt;
  boolean_T exitg1;
  int b_indMeas_size[1];
  double S_data[1296];
  int c_C_size[2];
  double dv49[4];
  double dv50[4];
  emxArray_real_T *d_a;
  emxInit_boolean_T(&c_anchorFeatures, 2);

  //  threshold for LI innovation test
  // 9.487729036781154; % HI mahalanobis gate
  // 9.487729036781154; % HI mahalanobis gate
  //  threshold for minimum LI supporter
  numAnchors = b_anchorFeatures->size[1];
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
  ii = 1;
  emxFree_boolean_T(&c_anchorFeatures);
  exitg3 = false;
  while ((!exitg3) && (ii < 17)) {
    guard2 = false;
    if (x[ii - 1]) {
      idx++;
      ii_data[idx - 1] = ii;
      if (idx >= 16) {
        exitg3 = true;
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

  if (1 > idx) {
    ar = 0;
  } else {
    ar = idx;
  }

  for (i19 = 0; i19 < loop_ub; i19++) {
    indMeas_data[i19] = (signed char)ii_data[i19];
  }

  emxInit_real_T(&K, 2);
  emxInit_real_T(&H, 2);
  numMeas = ar;
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
  // % B 1-point hypotheses generation and evaluation
  n_hyp = 100.0;
  LI_inlierStatus_size_idx_0 = ar;
  for (i19 = 0; i19 < ar; i19++) {
    LI_inlierStatus_data[i19] = false;
  }

  emxInit_real_T(&map, 2);
  b_emxInit_real_T(&anchorInd, 1);
  b_emxInit_real_T(&featureAnchorInd, 1);

  //  low innovation inlier status
  //  build the map according to the current estimate
  getMap(b_xt, b_anchorFeatures, b_m_vect, (double)b_anchorFeatures->size[1] *
         numPointsPerAnchor, c_numStatesxt, 7.0 + numPointsPerAnchor, map,
         anchorInd, featureAnchorInd);
  i = 1.0;
  emxInit_real_T(&K_i, 2);
  b_emxInit_real_T(&x_apo, 1);
  b_emxInit_real_T(&x_apo_prev, 1);
  emxInit_real_T(&H_i, 2);
  emxInit_real_T(&C, 2);
  b_emxInit_int32_T(&r13, 2);
  emxInit_real_T(&b, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&r14, 2);
  emxInit_int32_T(&r15, 1);
  emxInit_int32_T(&r16, 1);
  b_emxInit_real_T(&b_x_apo_prev, 1);
  while (i < n_hyp) {
    //  select a random measurement
    d_numStatesxt = b_rand();

    //  EKF state update
    getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, (double)indMeas_data
               [(int)(1.0 + floor(d_numStatesxt * (double)numMeas)) - 1], map,
               (double)numAnchors, numPointsPerAnchor, anchorInd,
               featureAnchorInd, b_m_vect, imNoise, IMU_measurements,
               height_offset_pressure, r_data, unusedU2_size, H_i, r, R_data,
               R_size);
    if ((H_i->size[1] == 1) || (b_P->size[0] == 1)) {
      i19 = y->size[0] * y->size[1];
      y->size[0] = H_i->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i19, (int)sizeof(double));
      ib = H_i->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = b_P->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          y->data[i19 + y->size[0] * i20] = 0.0;
          ii = H_i->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            y->data[i19 + y->size[0] * i20] += H_i->data[i19 + H_i->size[0] *
              i21] * b_P->data[i21 + b_P->size[0] * i20];
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
      ib = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ib; i19++) {
        y->data[i19] = 0.0;
      }

      if ((H_i->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H_i->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
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
    ib = H_i->size[0];
    for (i19 = 0; i19 < ib; i19++) {
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
      ib = y->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = b->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          C->data[i19 + C->size[0] * i20] = 0.0;
          ii = y->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            C->data[i19 + C->size[0] * i20] += y->data[i19 + y->size[0] * i21] *
              b->data[i21 + b->size[0] * i20];
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
      ib = (int)((float)a[0] * (float)a[1]);
      for (i19 = 0; i19 < ib; i19++) {
        C->data[i19] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
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
    ib = H_i->size[0];
    for (i19 = 0; i19 < ib; i19++) {
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
      ib = b_P->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = b->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          y->data[i19 + y->size[0] * i20] = 0.0;
          ii = b_P->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            y->data[i19 + y->size[0] * i20] += b_P->data[i19 + b_P->size[0] *
              i21] * b->data[i21 + b->size[0] * i20];
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
      ib = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ib; i19++) {
        y->data[i19] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
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
    ib = C->size[0] * C->size[1];
    for (i19 = 0; i19 < ib; i19++) {
      C_data[i19] = C->data[i19] + R_data[i19];
    }

    mrdivide(y, C_data, C_size, r14);
    i19 = K_i->size[0] * K_i->size[1];
    K_i->size[0] = r14->size[0];
    K_i->size[1] = r14->size[1];
    emxEnsureCapacity((emxArray__common *)K_i, i19, (int)sizeof(double));
    ib = r14->size[0] * r14->size[1];
    for (i19 = 0; i19 < ib; i19++) {
      K_i->data[i19] = r14->data[i19];
    }

    if ((K_i->size[1] == 1) || (unusedU2_size[0] == 1)) {
      i19 = x_apo->size[0];
      x_apo->size[0] = K_i->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i19, (int)sizeof(double));
      ib = K_i->size[0];
      for (i19 = 0; i19 < ib; i19++) {
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
      ib = (int)a[0];
      for (i19 = 0; i19 < ib; i19++) {
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
    ib = b_xt->size[0];
    for (i19 = 0; i19 < ib; i19++) {
      x_apo_prev->data[i19] = b_xt->data[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      x_apo_prev->data[i19] = b_xt->data[i19] + x_apo->data[i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      c_xt[i19] = x_apo->data[3 + i19];
    }

    quatPlusThetaJ(c_xt, dv44);
    quatmultJ(dv44, *(double (*)[4])&x_apo_prev->data[3], dv45);
    for (i19 = 0; i19 < 4; i19++) {
      x_apo_prev->data[3 + i19] = dv45[i19];
    }

    if (8.0 > c_numStatesxt) {
      i19 = 1;
      i20 = 0;
    } else {
      i19 = 8;
      i20 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      i21 = 1;
      ar = 0;
    } else {
      i21 = 7;
      ar = (int)c_numStates;
    }

    if (8.0 > c_numStatesxt) {
      idx = 0;
      br = 0;
    } else {
      idx = 7;
      br = (int)c_numStatesxt;
    }

    ii = r13->size[0] * r13->size[1];
    r13->size[0] = 1;
    r13->size[1] = br - idx;
    emxEnsureCapacity((emxArray__common *)r13, ii, (int)sizeof(int));
    ib = br - idx;
    for (br = 0; br < ib; br++) {
      r13->data[r13->size[0] * br] = idx + br;
    }

    idx = r15->size[0];
    r15->size[0] = (i20 - i19) + 1;
    emxEnsureCapacity((emxArray__common *)r15, idx, (int)sizeof(int));
    ib = i20 - i19;
    for (i20 = 0; i20 <= ib; i20++) {
      r15->data[i20] = i19 + i20;
    }

    i19 = r16->size[0];
    r16->size[0] = (ar - i21) + 1;
    emxEnsureCapacity((emxArray__common *)r16, i19, (int)sizeof(int));
    ib = ar - i21;
    for (i19 = 0; i19 <= ib; i19++) {
      r16->data[i19] = i21 + i19;
    }

    i19 = b_x_apo_prev->size[0];
    b_x_apo_prev->size[0] = r13->size[0] * r13->size[1];
    emxEnsureCapacity((emxArray__common *)b_x_apo_prev, i19, (int)sizeof(double));
    ib = r13->size[0] * r13->size[1];
    for (i19 = 0; i19 < ib; i19++) {
      b_x_apo_prev->data[i19] = x_apo_prev->data[r15->data[i19] - 1] +
        x_apo->data[r16->data[i19] - 1];
    }

    ib = b_x_apo_prev->size[0];
    for (i19 = 0; i19 < ib; i19++) {
      x_apo_prev->data[r13->data[i19]] = b_x_apo_prev->data[i19];
    }

    for (br = 0; br < numAnchors; br++) {
      for (i19 = 0; i19 < 16; i19++) {
        x[i19] = (b_anchorFeatures->data[i19 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 3; i19++) {
          c_xt[i19] = x_apo_prev->data[(int)(d_numStatesxt + (1.0 + (double)i19))
            - 1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i19)) - 1];
        }

        for (i19 = 0; i19 < 3; i19++) {
          x_apo_prev->data[(int)(e_numStatesxt + (1.0 + (double)i19)) - 1] =
            c_xt[i19];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 3; i19++) {
          c_xt[i19] = x_apo->data[(int)(d_numStates + (4.0 + (double)i19)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          d_xt[i19] = x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i19))
            - 1];
        }

        quatPlusThetaJ(c_xt, dv46);
        quatmultJ(dv46, d_xt, dv45);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i19 = 0; i19 < 4; i19++) {
          x_apo_prev->data[(int)(e_numStatesxt + (4.0 + (double)i19)) - 1] =
            dv45[i19];
        }

        for (ii = 0; ii < (int)numPointsPerAnchor; ii++) {
          x_apo_prev->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) + (1.0 + (double)ii)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ii)) - 1];
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
    for (br = 0; br < numAnchors; br++) {
      featureAnchorIdx = 1.0;
      idx = 0;
      ii = 1;
      exitg2 = false;
      while ((!exitg2) && (ii < 17)) {
        guard1 = false;
        if (b_anchorFeatures->data[(ii + b_anchorFeatures->size[0] * br) - 1] !=
            0.0) {
          idx++;
          ii_data[idx - 1] = ii;
          if (idx >= 16) {
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
        ar = 0;
      } else {
        ar = idx;
      }

      if (1 > idx) {
        ib = 0;
      } else {
        ib = idx;
      }

      for (i19 = 0; i19 < ib; i19++) {
        featureIdxVect_data[i19] = (signed char)ii_data[i19];
      }

      for (idx = 0; idx < ar; idx++) {
        if (b_anchorFeatures->data[(featureIdxVect_data[idx] +
             b_anchorFeatures->size[0] * br) - 1] == 1.0) {
          //  if this is not a lost feature
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
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
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] -
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1]) -
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1]) +
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
               numPointsPerAnchor)) + 7.0) - 1];
          c_x_apo_prev[1] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 5.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 6.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[2] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 5.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[3] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 5.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 6.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[4] = ((-(x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                                x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]) +
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1]) -
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1]) +
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
               numPointsPerAnchor)) + 7.0) - 1];
          c_x_apo_prev[5] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 4.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[6] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] + x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 5.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[7] = 2.0 * (x_apo_prev->data[(int)((c_numStatesxt + ((1.0
            + (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 6.0) - 1] - x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 4.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) - 1]);
          c_x_apo_prev[8] = ((-(x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1] *
                                x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 4.0) - 1]) -
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1] *
                              x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 5.0) - 1]) +
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1] *
                             x_apo_prev->data[(int)((c_numStatesxt + ((1.0 +
            (double)br) - 1.0) * (7.0 + numPointsPerAnchor)) + 6.0) - 1]) +
            x_apo_prev->data[(int)((c_numStatesxt + ((1.0 + (double)br) - 1.0) *
            (7.0 + numPointsPerAnchor)) + 7.0) - 1] * x_apo_prev->data[(int)
            ((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
               numPointsPerAnchor)) + 7.0) - 1];
          d_numStates = x_apo_prev->data[(int)(((c_numStatesxt + ((1.0 + (double)
            br) - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx)
            - 1];
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
          ii = (featureIdxVect_data[idx] - 1) * 2;
          for (i19 = 0; i19 < 2; i19++) {
            b_z_all_l[i19] = z_all_l[(ii + iv14[i19]) - 1] - r[i19];
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

    ii = 0;
    for (k = 0; k < LI_inlierStatus_size_idx_0; k++) {
      if (LI_inlierStatus_data[k]) {
        ii++;
      }
    }

    if (idx > ii) {
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
  emxFree_int32_T(&r16);
  emxFree_int32_T(&r15);
  emxFree_real_T(&r14);
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
    i19 = (int)maxEKFIterations;
    iter = 0;
    b_emxInit_real_T(&e_xt, 1);
    while (iter <= i19 - 1) {
      ii = LI_inlierStatus_size_idx_0 - 1;
      br = 0;
      for (idx = 0; idx <= ii; idx++) {
        if (LI_inlierStatus_data[idx]) {
          br++;
        }
      }

      ar = 0;
      for (idx = 0; idx <= ii; idx++) {
        if (LI_inlierStatus_data[idx]) {
          ii_data[ar] = idx + 1;
          ar++;
        }
      }

      indMeas_size[0] = br;
      for (i20 = 0; i20 < br; i20++) {
        b_indMeas_data[i20] = indMeas_data[ii_data[i20] - 1];
      }

      b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                   indMeas_size, map, (double)numAnchors, numPointsPerAnchor,
                   anchorInd, featureAnchorInd, b_m_vect, imNoise,
                   IMU_measurements, height_offset_pressure, b_r_data, r_size, H,
                   unusedU2_data, unusedU2_size, b_R_data, b_R_size);
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i20 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        ib = H->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b_P->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            y->data[i20 + y->size[0] * i21] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i20 + y->size[0] * i21] += H->data[i20 + H->size[0] * ar] *
                b_P->data[ar + b_P->size[0] * i21];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i20 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        i20 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i20 = 0; i20 < ib; i20++) {
          y->data[i20] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i20 = br + k;
            for (ib = br; ib + 1 <= i20; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i21 = idx + m;
                for (ic = idx; ic + 1 <= i21; ic++) {
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

      i20 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
      ib = H->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = H->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          b->data[i21 + b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i20 = C->size[0] * C->size[1];
        C->size[0] = y->size[0];
        C->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
        ib = y->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            C->data[i20 + C->size[0] * i21] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              C->data[i20 + C->size[0] * i21] += y->data[i20 + y->size[0] * ar] *
                b->data[ar + b->size[0] * i21];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i20 = C->size[0] * C->size[1];
        C->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
        i20 = C->size[0] * C->size[1];
        C->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i20 = 0; i20 < ib; i20++) {
          C->data[i20] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              C->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i20 = br + k;
            for (ib = br; ib + 1 <= i20; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i21 = idx + m;
                for (ic = idx; ic + 1 <= i21; ic++) {
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

      i20 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
      ib = H->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = H->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          b->data[i21 + b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
        }
      }

      if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
        i20 = y->size[0] * y->size[1];
        y->size[0] = b_P->size[0];
        y->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        ib = b_P->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            y->data[i20 + y->size[0] * i21] = 0.0;
            ii = b_P->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i20 + y->size[0] * i21] += b_P->data[i20 + b_P->size[0] *
                ar] * b->data[ar + b->size[0] * i21];
            }
          }
        }
      } else {
        k = b_P->size[1];
        a[0] = b_P->size[0];
        a[1] = b->size[1];
        m = b_P->size[0];
        i20 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        i20 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i20 = 0; i20 < ib; i20++) {
          y->data[i20] = 0.0;
        }

        if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = b_P->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i20 = br + k;
            for (ib = br; ib + 1 <= i20; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i21 = idx + m;
                for (ic = idx; ic + 1 <= i21; ic++) {
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
      ib = C->size[0] * C->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        b_C_data[i20] = C->data[i20] + b_R_data[i20];
      }

      mrdivide(y, b_C_data, b_C_size, K);
      if ((K->size[1] == 1) || (r_size[0] == 1)) {
        i20 = x_apo->size[0];
        x_apo->size[0] = K->size[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
        ib = K->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          x_apo->data[i20] = 0.0;
          idx = K->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            x_apo->data[i20] += K->data[i20 + K->size[0] * i21] * b_r_data[i21];
          }
        }
      } else {
        k = K->size[1];
        a[0] = K->size[0];
        m = K->size[0];
        i20 = x_apo->size[0];
        x_apo->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
        ib = (int)a[0];
        for (i20 = 0; i20 < ib; i20++) {
          x_apo->data[i20] = 0.0;
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
            i20 = br + k;
            for (ib = br; ib + 1 <= i20; ib++) {
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

      for (i20 = 0; i20 < 3; i20++) {
        c_xt[i20] = b_xt->data[i20] + x_apo->data[i20];
      }

      for (i20 = 0; i20 < 3; i20++) {
        b_xt->data[i20] = c_xt[i20];
      }

      for (i20 = 0; i20 < 3; i20++) {
        c_xt[i20] = x_apo->data[3 + i20];
      }

      quatPlusThetaJ(c_xt, dv47);
      quatmultJ(dv47, *(double (*)[4])&b_xt->data[3], dv45);
      for (i20 = 0; i20 < 4; i20++) {
        b_xt->data[3 + i20] = dv45[i20];
      }

      if (8.0 > c_numStatesxt) {
        i20 = 0;
        i21 = 0;
      } else {
        i20 = 7;
        i21 = (int)c_numStatesxt;
      }

      if (7.0 > c_numStates) {
        ar = 0;
      } else {
        ar = 6;
      }

      if (8.0 > c_numStatesxt) {
        idx = 0;
        br = 0;
      } else {
        idx = 7;
        br = (int)c_numStatesxt;
      }

      ii = r13->size[0] * r13->size[1];
      r13->size[0] = 1;
      r13->size[1] = br - idx;
      emxEnsureCapacity((emxArray__common *)r13, ii, (int)sizeof(int));
      ib = br - idx;
      for (br = 0; br < ib; br++) {
        r13->data[r13->size[0] * br] = idx + br;
      }

      idx = e_xt->size[0];
      e_xt->size[0] = i21 - i20;
      emxEnsureCapacity((emxArray__common *)e_xt, idx, (int)sizeof(double));
      ib = i21 - i20;
      for (i21 = 0; i21 < ib; i21++) {
        e_xt->data[i21] = b_xt->data[i20 + i21] + x_apo->data[ar + i21];
      }

      ib = r13->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        b_xt->data[r13->data[r13->size[0] * i20]] = e_xt->data[(*(int (*)[2])
          r13->size)[0] * i20];
      }

      for (br = 0; br < numAnchors; br++) {
        for (i20 = 0; i20 < 16; i20++) {
          x[i20] = (b_anchorFeatures->data[i20 + b_anchorFeatures->size[0] * br]
                    == 1.0);
        }

        if (any(x)) {
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
            numPointsPerAnchor);
          for (i20 = 0; i20 < 3; i20++) {
            c_xt[i20] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i20)) -
              1] + x_apo->data[(int)(d_numStates + (1.0 + (double)i20)) - 1];
          }

          for (i20 = 0; i20 < 3; i20++) {
            b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i20)) - 1] =
              c_xt[i20];
          }

          d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
            numPointsPerAnchor);
          for (i20 = 0; i20 < 3; i20++) {
            c_xt[i20] = x_apo->data[(int)(d_numStates + (4.0 + (double)i20)) - 1];
          }

          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i20 = 0; i20 < 4; i20++) {
            d_xt[i20] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i20)) -
              1];
          }

          quatPlusThetaJ(c_xt, dv48);
          quatmultJ(dv48, d_xt, dv45);
          e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor);
          for (i20 = 0; i20 < 4; i20++) {
            b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i20)) - 1] =
              dv45[i20];
          }

          for (ii = 0; ii < (int)numPointsPerAnchor; ii++) {
            b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0
              + numPointsPerAnchor)) + 7.0) + (1.0 + (double)ii)) - 1] +=
              x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) *
                                  (6.0 + numPointsPerAnchor)) + 6.0) + (1.0 +
              (double)ii)) - 1];
          }
        }
      }

      iter++;
    }

    emxFree_real_T(&e_xt);
    eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
         numPointsPerAnchor), b_a);
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      i19 = C->size[0] * C->size[1];
      C->size[0] = K->size[0];
      C->size[1] = H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
      ib = K->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = H->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          C->data[i19 + C->size[0] * i20] = 0.0;
          ii = K->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            C->data[i19 + C->size[0] * i20] += K->data[i19 + K->size[0] * i21] *
              H->data[i21 + H->size[0] * i20];
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
      ib = (int)a[0] * (int)a[1];
      for (i19 = 0; i19 < ib; i19++) {
        C->data[i19] = 0.0;
      }

      if ((K->size[0] == 0) || (H->size[1] == 0)) {
      } else {
        ii = K->size[0] * (H->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
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
    idx = b_a->size[0];
    ii = b_a->size[1];
    ib = idx * ii;
    for (i19 = 0; i19 < ib; i19++) {
      b_a->data[i19] -= C->data[i19];
    }

    i19 = b->size[0] * b->size[1];
    b->size[0] = b_P->size[0];
    b->size[1] = b_P->size[1];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(double));
    ib = b_P->size[0] * b_P->size[1];
    for (i19 = 0; i19 < ib; i19++) {
      b->data[i19] = b_P->data[i19];
    }

    emxInit_real_T(&c_a, 2);
    if ((b_a->size[1] == 1) || (b_P->size[0] == 1)) {
      i19 = c_a->size[0] * c_a->size[1];
      c_a->size[0] = b_a->size[0];
      c_a->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)c_a, i19, (int)sizeof(double));
      ib = b_a->size[0];
      for (i19 = 0; i19 < ib; i19++) {
        idx = b_P->size[1];
        for (i20 = 0; i20 < idx; i20++) {
          c_a->data[i19 + c_a->size[0] * i20] = 0.0;
          ii = b_a->size[1];
          for (i21 = 0; i21 < ii; i21++) {
            c_a->data[i19 + c_a->size[0] * i20] += b_a->data[i19 + b_a->size[0] *
              i21] * b_P->data[i21 + b_P->size[0] * i20];
          }
        }
      }

      i19 = b_P->size[0] * b_P->size[1];
      b_P->size[0] = c_a->size[0];
      b_P->size[1] = c_a->size[1];
      emxEnsureCapacity((emxArray__common *)b_P, i19, (int)sizeof(double));
      ib = c_a->size[1];
      for (i19 = 0; i19 < ib; i19++) {
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
      ib = (int)a[1];
      for (i19 = 0; i19 < ib; i19++) {
        idx = (int)a[0];
        for (i20 = 0; i20 < idx; i20++) {
          b_P->data[i20 + b_P->size[0] * i19] = 0.0;
        }
      }

      if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_a->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i19 = idx + m;
          for (ic = idx; ic + 1 <= i19; ic++) {
            b_P->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
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
  }

  // % D Partial EKF update using high-innovation inliers
  for (i19 = 0; i19 < loop_ub; i19++) {
    HI_inlierStatus_data[i19] = true;
  }

  //  high innovation inliers
  i19 = x_apo_prev->size[0];
  x_apo_prev->size[0] = b_P->size[0];
  emxEnsureCapacity((emxArray__common *)x_apo_prev, i19, (int)sizeof(double));
  ib = b_P->size[0];
  for (i19 = 0; i19 < ib; i19++) {
    x_apo_prev->data[i19] = 0.0;
  }

  i19 = (int)maxEKFIterations;
  it = 0;
  b_emxInit_real_T(&b_x_apo, 1);
  b_emxInit_real_T(&f_xt, 1);
  exitg1 = false;
  while ((!exitg1) && (it <= i19 - 1)) {
    ii = loop_ub - 1;
    br = 0;
    for (idx = 0; idx <= ii; idx++) {
      if (HI_inlierStatus_data[idx]) {
        br++;
      }
    }

    ar = 0;
    for (idx = 0; idx <= ii; idx++) {
      if (HI_inlierStatus_data[idx]) {
        ii_data[ar] = idx + 1;
        ar++;
      }
    }

    b_indMeas_size[0] = br;
    for (i20 = 0; i20 < br; i20++) {
      b_indMeas_data[i20] = indMeas_data[ii_data[i20] - 1];
    }

    b_getH_R_res(b_xt, c_numStates, c_numStatesxt, z_all_l, b_indMeas_data,
                 b_indMeas_size, map, (double)numAnchors, numPointsPerAnchor,
                 anchorInd, featureAnchorInd, b_m_vect, imNoise,
                 IMU_measurements, height_offset_pressure, b_r_data, r_size, H,
                 unusedU2_data, unusedU2_size, b_R_data, b_R_size);
    if (1.0 + (double)it == 1.0) {
      //  only do outlier rejection in first iteration
      if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
        i20 = y->size[0] * y->size[1];
        y->size[0] = H->size[0];
        y->size[1] = b_P->size[1];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        ib = H->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b_P->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            y->data[i20 + y->size[0] * i21] = 0.0;
            ii = H->size[1];
            for (ar = 0; ar < ii; ar++) {
              y->data[i20 + y->size[0] * i21] += H->data[i20 + H->size[0] * ar] *
                b_P->data[ar + b_P->size[0] * i21];
            }
          }
        }
      } else {
        k = H->size[1];
        a[0] = H->size[0];
        a[1] = b_P->size[1];
        m = H->size[0];
        i20 = y->size[0] * y->size[1];
        y->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        i20 = y->size[0] * y->size[1];
        y->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
        ib = (int)a[0] * (int)a[1];
        for (i20 = 0; i20 < ib; i20++) {
          y->data[i20] = 0.0;
        }

        if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
        } else {
          ii = H->size[0] * (b_P->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              y->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i20 = br + k;
            for (ib = br; ib + 1 <= i20; ib++) {
              if (b_P->data[ib] != 0.0) {
                ia = ar;
                i21 = idx + m;
                for (ic = idx; ic + 1 <= i21; ic++) {
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

      i20 = b->size[0] * b->size[1];
      b->size[0] = H->size[1];
      b->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
      ib = H->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = H->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          b->data[i21 + b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
        }
      }

      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i20 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = y->size[0];
        b_a->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)b_a, i20, (int)sizeof(double));
        ib = y->size[0];
        for (i20 = 0; i20 < ib; i20++) {
          idx = b->size[1];
          for (i21 = 0; i21 < idx; i21++) {
            b_a->data[i20 + b_a->size[0] * i21] = 0.0;
            ii = y->size[1];
            for (ar = 0; ar < ii; ar++) {
              b_a->data[i20 + b_a->size[0] * i21] += y->data[i20 + y->size[0] *
                ar] * b->data[ar + b->size[0] * i21];
            }
          }
        }
      } else {
        k = y->size[1];
        a[0] = (signed char)y->size[0];
        a[1] = (signed char)b->size[1];
        m = y->size[0];
        i20 = b_a->size[0] * b_a->size[1];
        b_a->size[0] = (int)a[0];
        emxEnsureCapacity((emxArray__common *)b_a, i20, (int)sizeof(double));
        i20 = b_a->size[0] * b_a->size[1];
        b_a->size[1] = (int)a[1];
        emxEnsureCapacity((emxArray__common *)b_a, i20, (int)sizeof(double));
        ib = (int)((float)a[0] * (float)a[1]);
        for (i20 = 0; i20 < ib; i20++) {
          b_a->data[i20] = 0.0;
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          ii = y->size[0] * (b->size[1] - 1);
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            i20 = idx + m;
            for (ic = idx; ic + 1 <= i20; ic++) {
              b_a->data[ic] = 0.0;
            }

            idx += m;
          }

          br = 0;
          idx = 0;
          while ((m > 0) && (idx <= ii)) {
            ar = 0;
            i20 = br + k;
            for (ib = br; ib + 1 <= i20; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i21 = idx + m;
                for (ic = idx; ic + 1 <= i21; ic++) {
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

      br = b_a->size[0];
      ib = b_a->size[0] * b_a->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        S_data[i20] = b_a->data[i20] + b_R_data[i20];
      }

      for (k = 0; k < numMeas; k++) {
        ar = k << 1;
        ii = k << 1;
        idx = k << 1;
        for (i20 = 0; i20 < 2; i20++) {
          r[i20] = b_r_data[i20 + ar];
          for (i21 = 0; i21 < 2; i21++) {
            d_xt[i21 + (i20 << 1)] = S_data[(i21 + ii) + br * (i20 + idx)];
          }
        }

        b_mrdivide(r, d_xt, a);
        ar = k << 1;
        for (i20 = 0; i20 < 2; i20++) {
          r[i20] = b_r_data[i20 + ar];
        }

        if (LI_inlierStatus_data[k]) {
          //  if this feature is a LI inlier, don't also do HI update with this feature 
          ar = k << 1;
          for (i20 = 0; i20 < 2; i20++) {
            b_r_data[i20 + ar] = 0.0;
          }

          ib = H->size[1];
          ar = k << 1;
          for (i20 = 0; i20 < ib; i20++) {
            for (i21 = 0; i21 < 2; i21++) {
              H->data[(i21 + ar) + H->size[0] * i20] = 0.0;
            }
          }

          HI_inlierStatus_data[k] = false;
        } else {
          //  otherwise check if HI inlier for both cams
          d_numStatesxt = 0.0;
          for (i20 = 0; i20 < 2; i20++) {
            d_numStatesxt += a[i20] * r[i20];
          }

          if (d_numStatesxt > 6.0) {
            ar = k << 1;
            for (i20 = 0; i20 < 2; i20++) {
              b_r_data[i20 + ar] = 0.0;
            }

            ib = H->size[1];
            ar = k << 1;
            for (i20 = 0; i20 < ib; i20++) {
              for (i21 = 0; i21 < 2; i21++) {
                H->data[(i21 + ar) + H->size[0] * i20] = 0.0;
              }
            }

            HI_inlierStatus_data[k] = false;
          }
        }
      }

      //          S = (H*P*H'+R);
      //      else
      //          for k = 1:numMeas
      //              if ~HI_inlierStatus(k)
      //                  r((k-1)*residualDim + (1:2)) = 0;
      //                  H((k-1)*residualDim + (1:2), :) = 0;
      //                  HI_inlierStatus(k) = false;
      //              end
      //          end
      //
      //          S = (H*P*H'+R);
    }

    if ((H->size[1] == 1) || (b_P->size[0] == 1)) {
      i20 = y->size[0] * y->size[1];
      y->size[0] = H->size[0];
      y->size[1] = b_P->size[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = H->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b_P->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          y->data[i20 + y->size[0] * i21] = 0.0;
          ii = H->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i20 + y->size[0] * i21] += H->data[i20 + H->size[0] * ar] *
              b_P->data[ar + b_P->size[0] * i21];
          }
        }
      }
    } else {
      k = H->size[1];
      a[0] = H->size[0];
      a[1] = b_P->size[1];
      m = H->size[0];
      i20 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      i20 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        y->data[i20] = 0.0;
      }

      if ((H->size[0] == 0) || (b_P->size[1] == 0)) {
      } else {
        ii = H->size[0] * (b_P->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b_P->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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

    i20 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
    ib = H->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      idx = H->size[1];
      for (i21 = 0; i21 < idx; i21++) {
        b->data[i21 + b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      i20 = C->size[0] * C->size[1];
      C->size[0] = y->size[0];
      C->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      ib = y->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          C->data[i20 + C->size[0] * i21] = 0.0;
          ii = y->size[1];
          for (ar = 0; ar < ii; ar++) {
            C->data[i20 + C->size[0] * i21] += y->data[i20 + y->size[0] * ar] *
              b->data[ar + b->size[0] * i21];
          }
        }
      }
    } else {
      k = y->size[1];
      a[0] = (signed char)y->size[0];
      a[1] = (signed char)b->size[1];
      m = y->size[0];
      i20 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      i20 = C->size[0] * C->size[1];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i20, (int)sizeof(double));
      ib = (int)((float)a[0] * (float)a[1]);
      for (i20 = 0; i20 < ib; i20++) {
        C->data[i20] = 0.0;
      }

      if ((y->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = y->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            C->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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

    i20 = b->size[0] * b->size[1];
    b->size[0] = H->size[1];
    b->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)b, i20, (int)sizeof(double));
    ib = H->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      idx = H->size[1];
      for (i21 = 0; i21 < idx; i21++) {
        b->data[i21 + b->size[0] * i20] = H->data[i20 + H->size[0] * i21];
      }
    }

    if ((b_P->size[1] == 1) || (b->size[0] == 1)) {
      i20 = y->size[0] * y->size[1];
      y->size[0] = b_P->size[0];
      y->size[1] = b->size[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = b_P->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        idx = b->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          y->data[i20 + y->size[0] * i21] = 0.0;
          ii = b_P->size[1];
          for (ar = 0; ar < ii; ar++) {
            y->data[i20 + y->size[0] * i21] += b_P->data[i20 + b_P->size[0] * ar]
              * b->data[ar + b->size[0] * i21];
          }
        }
      }
    } else {
      k = b_P->size[1];
      a[0] = b_P->size[0];
      a[1] = b->size[1];
      m = b_P->size[0];
      i20 = y->size[0] * y->size[1];
      y->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      i20 = y->size[0] * y->size[1];
      y->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
      ib = (int)a[0] * (int)a[1];
      for (i20 = 0; i20 < ib; i20++) {
        y->data[i20] = 0.0;
      }

      if ((b_P->size[0] == 0) || (b->size[1] == 0)) {
      } else {
        ii = b_P->size[0] * (b->size[1] - 1);
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          i20 = idx + m;
          for (ic = idx; ic + 1 <= i20; ic++) {
            y->data[ic] = 0.0;
          }

          idx += m;
        }

        br = 0;
        idx = 0;
        while ((m > 0) && (idx <= ii)) {
          ar = 0;
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
            if (b->data[ib] != 0.0) {
              ia = ar;
              i21 = idx + m;
              for (ic = idx; ic + 1 <= i21; ic++) {
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
    ib = C->size[0] * C->size[1];
    for (i20 = 0; i20 < ib; i20++) {
      b_C_data[i20] = C->data[i20] + b_R_data[i20];
    }

    mrdivide(y, b_C_data, c_C_size, K);
    if ((K->size[1] == 1) || (r_size[0] == 1)) {
      i20 = x_apo->size[0];
      x_apo->size[0] = K->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
      ib = K->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo->data[i20] = 0.0;
        idx = K->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          x_apo->data[i20] += K->data[i20 + K->size[0] * i21] * b_r_data[i21];
        }
      }
    } else {
      k = K->size[1];
      a[0] = K->size[0];
      m = K->size[0];
      i20 = x_apo->size[0];
      x_apo->size[0] = (int)a[0];
      emxEnsureCapacity((emxArray__common *)x_apo, i20, (int)sizeof(double));
      ib = (int)a[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo->data[i20] = 0.0;
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
          i20 = br + k;
          for (ib = br; ib + 1 <= i20; ib++) {
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

    for (i20 = 0; i20 < 3; i20++) {
      c_xt[i20] = b_xt->data[i20] + x_apo->data[i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      b_xt->data[i20] = c_xt[i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      c_xt[i20] = x_apo->data[3 + i20];
    }

    quatPlusThetaJ(c_xt, dv49);
    quatmultJ(dv49, *(double (*)[4])&b_xt->data[3], dv45);
    for (i20 = 0; i20 < 4; i20++) {
      b_xt->data[3 + i20] = dv45[i20];
    }

    if (8.0 > c_numStatesxt) {
      i20 = 0;
      i21 = 0;
    } else {
      i20 = 7;
      i21 = (int)c_numStatesxt;
    }

    if (7.0 > c_numStates) {
      ar = 0;
    } else {
      ar = 6;
    }

    if (8.0 > c_numStatesxt) {
      idx = 0;
      br = 0;
    } else {
      idx = 7;
      br = (int)c_numStatesxt;
    }

    ii = r13->size[0] * r13->size[1];
    r13->size[0] = 1;
    r13->size[1] = br - idx;
    emxEnsureCapacity((emxArray__common *)r13, ii, (int)sizeof(int));
    ib = br - idx;
    for (br = 0; br < ib; br++) {
      r13->data[r13->size[0] * br] = idx + br;
    }

    idx = f_xt->size[0];
    f_xt->size[0] = i21 - i20;
    emxEnsureCapacity((emxArray__common *)f_xt, idx, (int)sizeof(double));
    ib = i21 - i20;
    for (i21 = 0; i21 < ib; i21++) {
      f_xt->data[i21] = b_xt->data[i20 + i21] + x_apo->data[ar + i21];
    }

    ib = r13->size[1];
    for (i20 = 0; i20 < ib; i20++) {
      b_xt->data[r13->data[r13->size[0] * i20]] = f_xt->data[(*(int (*)[2])
        r13->size)[0] * i20];
    }

    for (br = 0; br < numAnchors; br++) {
      for (i20 = 0; i20 < 16; i20++) {
        x[i20] = (b_anchorFeatures->data[i20 + b_anchorFeatures->size[0] * br] ==
                  1.0);
      }

      if (any(x)) {
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 3; i20++) {
          c_xt[i20] = b_xt->data[(int)(d_numStatesxt + (1.0 + (double)i20)) - 1]
            + x_apo->data[(int)(d_numStates + (1.0 + (double)i20)) - 1];
        }

        for (i20 = 0; i20 < 3; i20++) {
          b_xt->data[(int)(e_numStatesxt + (1.0 + (double)i20)) - 1] = c_xt[i20];
        }

        d_numStates = c_numStates + ((1.0 + (double)br) - 1.0) * (6.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 3; i20++) {
          c_xt[i20] = x_apo->data[(int)(d_numStates + (4.0 + (double)i20)) - 1];
        }

        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 4; i20++) {
          d_xt[i20] = b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i20)) - 1];
        }

        quatPlusThetaJ(c_xt, dv50);
        quatmultJ(dv50, d_xt, dv45);
        e_numStatesxt = c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
          numPointsPerAnchor);
        for (i20 = 0; i20 < 4; i20++) {
          b_xt->data[(int)(e_numStatesxt + (4.0 + (double)i20)) - 1] = dv45[i20];
        }

        for (ii = 0; ii < (int)numPointsPerAnchor; ii++) {
          b_xt->data[(int)(((c_numStatesxt + ((1.0 + (double)br) - 1.0) * (7.0 +
            numPointsPerAnchor)) + 7.0) + (1.0 + (double)ii)) - 1] +=
            x_apo->data[(int)(((c_numStates + ((1.0 + (double)br) - 1.0) * (6.0
            + numPointsPerAnchor)) + 6.0) + (1.0 + (double)ii)) - 1];
        }
      }
    }

    i20 = b_x_apo->size[0];
    b_x_apo->size[0] = x_apo->size[0];
    emxEnsureCapacity((emxArray__common *)b_x_apo, i20, (int)sizeof(double));
    ib = x_apo->size[0];
    for (i20 = 0; i20 < ib; i20++) {
      b_x_apo->data[i20] = x_apo->data[i20] - x_apo_prev->data[i20];
    }

    if (d_norm(b_x_apo) < 0.001) {
      exitg1 = true;
    } else {
      i20 = x_apo_prev->size[0];
      x_apo_prev->size[0] = x_apo->size[0];
      emxEnsureCapacity((emxArray__common *)x_apo_prev, i20, (int)sizeof(double));
      ib = x_apo->size[0];
      for (i20 = 0; i20 < ib; i20++) {
        x_apo_prev->data[i20] = x_apo->data[i20];
      }

      it++;
    }
  }

  emxFree_real_T(&f_xt);
  emxFree_real_T(&b_x_apo);
  emxFree_real_T(&y);
  emxFree_int32_T(&r13);
  emxFree_real_T(&featureAnchorInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&map);
  emxFree_real_T(&x_apo_prev);
  emxFree_real_T(&x_apo);
  eye(c_numStates + (double)b_anchorFeatures->size[1] * (6.0 +
       numPointsPerAnchor), b_a);
  if ((K->size[1] == 1) || (H->size[0] == 1)) {
    i19 = C->size[0] * C->size[1];
    C->size[0] = K->size[0];
    C->size[1] = H->size[1];
    emxEnsureCapacity((emxArray__common *)C, i19, (int)sizeof(double));
    loop_ub = K->size[0];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ib = H->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        C->data[i19 + C->size[0] * i20] = 0.0;
        idx = K->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          C->data[i19 + C->size[0] * i20] += K->data[i19 + K->size[0] * i21] *
            H->data[i21 + H->size[0] * i20];
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
      ii = K->size[0] * (H->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i19 = idx + m;
        for (ic = idx; ic + 1 <= i19; ic++) {
          C->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
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
  idx = b_a->size[0];
  ii = b_a->size[1];
  loop_ub = idx * ii;
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
      ib = b_P->size[1];
      for (i20 = 0; i20 < ib; i20++) {
        d_a->data[i19 + d_a->size[0] * i20] = 0.0;
        idx = b_a->size[1];
        for (i21 = 0; i21 < idx; i21++) {
          d_a->data[i19 + d_a->size[0] * i20] += b_a->data[i19 + b_a->size[0] *
            i21] * b_P->data[i21 + b_P->size[0] * i20];
        }
      }
    }

    i19 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = d_a->size[0];
    b_P->size[1] = d_a->size[1];
    emxEnsureCapacity((emxArray__common *)b_P, i19, (int)sizeof(double));
    loop_ub = d_a->size[1];
    for (i19 = 0; i19 < loop_ub; i19++) {
      ib = d_a->size[0];
      for (i20 = 0; i20 < ib; i20++) {
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
      ib = (int)a[0];
      for (i20 = 0; i20 < ib; i20++) {
        b_P->data[i20 + b_P->size[0] * i19] = 0.0;
      }
    }

    if ((b_a->size[0] == 0) || (b->size[1] == 0)) {
    } else {
      ii = b_a->size[0] * (b->size[1] - 1);
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
        i19 = idx + m;
        for (ic = idx; ic + 1 <= i19; ic++) {
          b_P->data[ic] = 0.0;
        }

        idx += m;
      }

      br = 0;
      idx = 0;
      while ((m > 0) && (idx <= ii)) {
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
  ii = LI_inlierStatus_size_idx_0 - 1;
  br = 0;
  for (idx = 0; idx <= ii; idx++) {
    if (LI_inlierStatus_data[idx] || HI_inlierStatus_data[idx]) {
      br++;
    }
  }

  validFeatures_size[0] = br;
  ar = 0;
  for (idx = 0; idx <= ii; idx++) {
    if (LI_inlierStatus_data[idx] || HI_inlierStatus_data[idx]) {
      validFeatures_data[ar] = indMeas_data[idx];
      ar++;
    }
  }

  //  if length(validFeatures) ~= numMeas
  //      rejected = setdiff(indMeas, validFeatures);
  //      fprintf('%i of %i features are valid after RANSAC.\n', int8(length(validFeatures)), numMeas) 
  //      fprintf(' Rejected: %s\n', mat2str(rejected))
  //  end
}

//
// THIS IS OK, It is according to the NASA memo found
//  https://claraty.jpl.nasa.gov/man/software/development/conventions/standards_docs/unadopted/JPL_Quaternions_Breckenridge.pdf
// QUATFROMROTJ Get equivalent quaternion of the rotation matrix R
//    Returns a quaternion in JPL notation
//  The implementation is copied from qGetQ(R), but we are careful about the
//  ordering of the output vector
// Arguments    : const double R[9]
//                double b_Q[4]
// Return Type  : void
//
static void QuatFromRotJ(const double R[9], double b_Q[4])
{
  double b_T;
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
  int i0;
  signed char i_data[4];
  signed char index_data[4];

  //  if( r ~= 3 || c ~= 3 )
  //      error( 'R must be a 3x3 matrix\n\r' );
  //  end
  b_T = (R[0] + R[4]) + R[8];
  x = sqrt((1.0 + 2.0 * R[0]) - b_T);
  pivot[0] = x / 2.0;
  pivot[1] = sqrt((1.0 + 2.0 * R[4]) - b_T) / 2.0;
  pivot[2] = sqrt((1.0 + 2.0 * R[8]) - b_T) / 2.0;
  pivot[3] = sqrt(1.0 + b_T) / 2.0;
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

  for (i0 = 0; i0 < loop_ub; i0++) {
    i_data[i0] = ii_data[i0];
  }

  if (1 > idx) {
    ixstart = 0;
  } else {
    ixstart = idx;
  }

  for (i0 = 0; i0 < ixstart; i0++) {
    index_data[i0] = i_data[i0];
  }

  for (ixstart = 0; ixstart < loop_ub; ixstart++) {
    index_data[ixstart] = (signed char)rt_roundd_snf((double)index_data[ixstart]);
  }

  if (index_data[0] == 1) {
    x = sqrt((1.0 + 2.0 * R[0]) - b_T);
    mtmp = x / 2.0;
    b_Q[0] = x / 2.0;
    b_Q[1] = (R[3] + R[1]) / (4.0 * mtmp);
    b_Q[2] = (R[6] + R[2]) / (4.0 * mtmp);
    b_Q[3] = (R[7] - R[5]) / (4.0 * mtmp);
  } else if (index_data[0] == 2) {
    x = sqrt((1.0 + 2.0 * R[4]) - b_T);
    mtmp = x / 2.0;
    b_Q[1] = x / 2.0;
    b_Q[0] = (R[3] + R[1]) / (4.0 * mtmp);
    b_Q[2] = (R[7] + R[5]) / (4.0 * mtmp);
    b_Q[3] = (R[2] - R[6]) / (4.0 * mtmp);
  } else if (index_data[0] == 3) {
    x = sqrt((1.0 + 2.0 * R[8]) - b_T);
    mtmp = x / 2.0;
    b_Q[2] = x / 2.0;
    b_Q[0] = (R[6] + R[2]) / (4.0 * mtmp);
    b_Q[1] = (R[7] + R[5]) / (4.0 * mtmp);
    b_Q[3] = (R[3] - R[1]) / (4.0 * mtmp);
  } else {
    x = sqrt(1.0 + b_T);
    mtmp = x / 2.0;
    b_Q[3] = x / 2.0;
    b_Q[0] = (R[7] - R[5]) / (4.0 * mtmp);
    b_Q[1] = (R[2] - R[6]) / (4.0 * mtmp);
    b_Q[2] = (R[3] - R[1]) / (4.0 * mtmp);
  }
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
  int i;
  emxInit_real_T(&P, 2);
  b_emxInit_real_T(&xt, 1);
  init_counter = 0.0;
  for (i = 0; i < 6; i++) {
    delayBuffer_k_0[i] = 0.0;
  }
}

//
// EKF_SLAM: computes the camerapose p and feature position f in an
// interative way
//    Handels a static number of points but can dynamically asign them to new
//    klt points.
// Arguments    : emxArray_real_T *P_apo
//                emxArray_real_T *x
//                double dt
//                const double processNoise[4]
//                const double IMU_measurements[23]
//                double c_numStates
// Return Type  : void
//
static void SLAM_pred(emxArray_real_T *P_apo, emxArray_real_T *x, double dt,
                      const double processNoise[4], const double
                      IMU_measurements[23], double c_numStates)
{
  double c;
  double b_processNoise[9];
  double c_processNoise[9];
  int i27;
  double b_Q[81];
  int loop_ub;
  int i;
  emxArray_real_T *P_xx_apr;
  int i28;
  emxArray_real_T *Phi;
  double meas_0[6];
  emxArray_real_T *b_x;
  emxArray_real_T *x1;
  emxArray_real_T *b_Phi;
  double b_P_xx_apr[144];
  double meas_1[6];
  emxArray_real_T *xx;
  double b_xx[4];
  double c_P_xx_apr[144];
  emxArray_real_T *c_Phi;
  emxArray_real_T *P_xs_apr;
  emxArray_real_T *x2;
  double P2[144];
  emxArray_real_T *d_Phi;
  emxArray_real_T *x3;
  emxArray_real_T *Phi3;
  double P3[144];
  double b_meas_1[6];
  emxArray_real_T *e_Phi;
  emxArray_real_T *x4;
  emxArray_real_T *Phi4;
  double P4[144];
  emxArray_int32_T *r17;
  emxArray_real_T *c_x;
  double d_P_xx_apr;
  emxArray_real_T *b_P_apo;
  int cr;
  int k;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  emxArray_int32_T *r18;
  emxArray_int32_T *r19;
  emxArray_real_T *b_P_xs_apr;

  //  Xv meaning
  //
  //                X Y Z qR qX qY qZ Vx Vy Vz Wx Wy Wz
  //  C++ index     0 1 2  3  4  5  6  7  8  9 10 11 12
  //  Matlab index  1 2 3  4  5  6  7  8  9 10 11 12 13
  //  (C) Tobias Naegeli naegelit@inf.ethz.ch
  //  numStatesFeatures=numAnchors*(7+numPointsPerAnchor);
  //  %% Iterative Camera Pose optimization (EKF)
  //  numStates=22;
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
  for (i27 = 0; i27 < 9; i27++) {
    c_processNoise[i27] = b_processNoise[i27] * c;
  }

  b_diag(c_processNoise, b_Q);
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    i = 0;
  } else {
    i = (int)c_numStates;
  }

  emxInit_real_T(&P_xx_apr, 2);
  i27 = P_xx_apr->size[0] * P_xx_apr->size[1];
  P_xx_apr->size[0] = loop_ub;
  P_xx_apr->size[1] = i;
  emxEnsureCapacity((emxArray__common *)P_xx_apr, i27, (int)sizeof(double));
  for (i27 = 0; i27 < i; i27++) {
    for (i28 = 0; i28 < loop_ub; i28++) {
      P_xx_apr->data[i28 + P_xx_apr->size[0] * i27] = P_apo->data[i28 +
        P_apo->size[0] * i27];
    }
  }

  emxInit_real_T(&Phi, 2);
  eye(c_numStates, Phi);
  for (i = 0; i < 3; i++) {
    meas_0[i] = IMU_measurements[i];
  }

  for (i = 0; i < 3; i++) {
    meas_0[i + 3] = IMU_measurements[i + 3];
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&b_x, 1);
  i27 = b_x->size[0];
  b_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)b_x, i27, (int)sizeof(double));
  for (i27 = 0; i27 < loop_ub; i27++) {
    b_x->data[i27] = x->data[i27];
  }

  b_emxInit_real_T(&x1, 1);
  emxInit_real_T(&b_Phi, 2);
  dxdt_dPdt(dt, meas_0, b_x, P_xx_apr, Phi, b_Q, x1, b_P_xx_apr, b_Phi);
  emxFree_real_T(&b_x);
  for (i = 0; i < 6; i++) {
    meas_1[i] = meas_0[i] + (meas_0[i] - meas_0[i]) * 0.5;
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  b_emxInit_real_T(&xx, 1);
  i27 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i27, (int)sizeof(double));
  for (i27 = 0; i27 < loop_ub; i27++) {
    xx->data[i27] = x->data[i27] + x1->data[i27] / 2.0;
  }

  for (i27 = 0; i27 < 4; i27++) {
    b_xx[i27] = xx->data[3 + i27];
  }

  c = b_norm(b_xx);
  for (i27 = 0; i27 < 4; i27++) {
    b_xx[i27] = xx->data[3 + i27] / c;
  }

  for (i27 = 0; i27 < 4; i27++) {
    xx->data[3 + i27] = b_xx[i27];
  }

  for (i27 = 0; i27 < 144; i27++) {
    c_P_xx_apr[i27] = P_xx_apr->data[i27] + b_P_xx_apr[i27] / 2.0;
  }

  emxInit_real_T(&c_Phi, 2);
  i27 = c_Phi->size[0] * c_Phi->size[1];
  c_Phi->size[0] = Phi->size[0];
  c_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)c_Phi, i27, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i27 = 0; i27 < loop_ub; i27++) {
    c_Phi->data[i27] = Phi->data[i27] + b_Phi->data[i27] / 2.0;
  }

  emxInit_real_T(&P_xs_apr, 2);
  b_emxInit_real_T(&x2, 1);
  b_dxdt_dPdt(dt, meas_1, xx, c_P_xx_apr, c_Phi, b_Q, x2, P2, P_xs_apr);
  emxFree_real_T(&c_Phi);
  for (i27 = 0; i27 < 6; i27++) {
    meas_1[i27] += (meas_0[i27] - meas_1[i27]) * 0.5;
  }

  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i27 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i27, (int)sizeof(double));
  for (i27 = 0; i27 < loop_ub; i27++) {
    xx->data[i27] = x->data[i27] + x2->data[i27] / 2.0;
  }

  for (i27 = 0; i27 < 4; i27++) {
    b_xx[i27] = xx->data[3 + i27];
  }

  c = b_norm(b_xx);
  for (i27 = 0; i27 < 4; i27++) {
    b_xx[i27] = xx->data[3 + i27] / c;
  }

  for (i27 = 0; i27 < 4; i27++) {
    xx->data[3 + i27] = b_xx[i27];
  }

  for (i27 = 0; i27 < 144; i27++) {
    c_P_xx_apr[i27] = P_xx_apr->data[i27] + P2[i27] / 2.0;
  }

  emxInit_real_T(&d_Phi, 2);
  i27 = d_Phi->size[0] * d_Phi->size[1];
  d_Phi->size[0] = Phi->size[0];
  d_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)d_Phi, i27, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i27 = 0; i27 < loop_ub; i27++) {
    d_Phi->data[i27] = Phi->data[i27] + P_xs_apr->data[i27] / 2.0;
  }

  b_emxInit_real_T(&x3, 1);
  emxInit_real_T(&Phi3, 2);
  b_dxdt_dPdt(dt, meas_1, xx, c_P_xx_apr, d_Phi, b_Q, x3, P3, Phi3);
  emxFree_real_T(&d_Phi);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  i27 = xx->size[0];
  xx->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)xx, i27, (int)sizeof(double));
  for (i27 = 0; i27 < loop_ub; i27++) {
    xx->data[i27] = x->data[i27] + x3->data[i27];
  }

  for (i27 = 0; i27 < 4; i27++) {
    b_xx[i27] = xx->data[3 + i27];
  }

  c = b_norm(b_xx);
  for (i27 = 0; i27 < 4; i27++) {
    b_xx[i27] = xx->data[3 + i27] / c;
  }

  for (i27 = 0; i27 < 4; i27++) {
    xx->data[3 + i27] = b_xx[i27];
  }

  for (i = 0; i < 6; i++) {
    b_meas_1[i] = meas_1[i] + (meas_0[i] - meas_1[i]);
  }

  for (i27 = 0; i27 < 144; i27++) {
    c_P_xx_apr[i27] = P_xx_apr->data[i27] + P3[i27];
  }

  emxInit_real_T(&e_Phi, 2);
  i27 = e_Phi->size[0] * e_Phi->size[1];
  e_Phi->size[0] = Phi->size[0];
  e_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)e_Phi, i27, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i27 = 0; i27 < loop_ub; i27++) {
    e_Phi->data[i27] = Phi->data[i27] + Phi3->data[i27];
  }

  b_emxInit_real_T(&x4, 1);
  emxInit_real_T(&Phi4, 2);
  b_dxdt_dPdt(dt, b_meas_1, xx, c_P_xx_apr, e_Phi, b_Q, x4, P4, Phi4);
  emxFree_real_T(&e_Phi);
  emxFree_real_T(&xx);
  if (1.0 > c_numStates + 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(c_numStates + 1.0);
  }

  if (1.0 > c_numStates + 1.0) {
    i = 0;
  } else {
    i = (int)(c_numStates + 1.0);
  }

  b_emxInit_int32_T(&r17, 2);
  i27 = r17->size[0] * r17->size[1];
  r17->size[0] = 1;
  r17->size[1] = i;
  emxEnsureCapacity((emxArray__common *)r17, i27, (int)sizeof(int));
  for (i27 = 0; i27 < i; i27++) {
    r17->data[r17->size[0] * i27] = i27;
  }

  b_emxInit_real_T(&c_x, 1);
  i27 = c_x->size[0];
  c_x->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_x, i27, (int)sizeof(double));
  for (i27 = 0; i27 < loop_ub; i27++) {
    c_x->data[i27] = x->data[i27] + (((x1->data[i27] + 2.0 * x2->data[i27]) +
      2.0 * x3->data[i27]) + x4->data[i27]) / 6.0;
  }

  emxFree_real_T(&x4);
  emxFree_real_T(&x3);
  emxFree_real_T(&x2);
  emxFree_real_T(&x1);
  loop_ub = r17->size[1];
  for (i27 = 0; i27 < loop_ub; i27++) {
    x->data[r17->data[r17->size[0] * i27]] = c_x->data[(*(int (*)[2])r17->size)
      [0] * i27];
  }

  emxFree_real_T(&c_x);
  emxFree_int32_T(&r17);
  for (i27 = 0; i27 < 144; i27++) {
    d_P_xx_apr = P_xx_apr->data[i27] + (((b_P_xx_apr[i27] + 2.0 * P2[i27]) + 2.0
      * P3[i27]) + P4[i27]) / 6.0;
    b_P_xx_apr[i27] = d_P_xx_apr;
  }

  emxFree_real_T(&P_xx_apr);

  //  covariance of the state
  i27 = b_Phi->size[0] * b_Phi->size[1];
  b_Phi->size[0] = Phi->size[0];
  b_Phi->size[1] = Phi->size[1];
  emxEnsureCapacity((emxArray__common *)b_Phi, i27, (int)sizeof(double));
  loop_ub = Phi->size[0] * Phi->size[1];
  for (i27 = 0; i27 < loop_ub; i27++) {
    b_Phi->data[i27] = Phi->data[i27] + (((b_Phi->data[i27] + 2.0 *
      P_xs_apr->data[i27]) + 2.0 * Phi3->data[i27]) + Phi4->data[i27]) / 6.0;
  }

  emxFree_real_T(&Phi4);
  emxFree_real_T(&Phi3);
  emxFree_real_T(&Phi);
  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i27 = 0;
    i28 = 0;
  } else {
    i27 = (int)(c_numStates + 1.0) - 1;
    i28 = P_apo->size[1];
  }

  emxInit_real_T(&b_P_apo, 2);
  if ((b_Phi->size[1] == 1) || (loop_ub == 1)) {
    cr = b_P_apo->size[0] * b_P_apo->size[1];
    b_P_apo->size[0] = loop_ub;
    b_P_apo->size[1] = i28 - i27;
    emxEnsureCapacity((emxArray__common *)b_P_apo, cr, (int)sizeof(double));
    i = i28 - i27;
    for (i28 = 0; i28 < i; i28++) {
      for (cr = 0; cr < loop_ub; cr++) {
        b_P_apo->data[cr + b_P_apo->size[0] * i28] = P_apo->data[cr +
          P_apo->size[0] * (i27 + i28)];
      }
    }

    i27 = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    P_xs_apr->size[1] = b_P_apo->size[1];
    emxEnsureCapacity((emxArray__common *)P_xs_apr, i27, (int)sizeof(double));
    for (i27 = 0; i27 < 12; i27++) {
      loop_ub = b_P_apo->size[1];
      for (i28 = 0; i28 < loop_ub; i28++) {
        P_xs_apr->data[i27 + P_xs_apr->size[0] * i28] = 0.0;
        i = b_Phi->size[1];
        for (cr = 0; cr < i; cr++) {
          P_xs_apr->data[i27 + P_xs_apr->size[0] * i28] += b_Phi->data[i27 +
            b_Phi->size[0] * cr] * b_P_apo->data[cr + b_P_apo->size[0] * i28];
        }
      }
    }
  } else {
    k = b_Phi->size[1];
    cr = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, cr, (int)sizeof(double));
    cr = P_xs_apr->size[0] * P_xs_apr->size[1];
    P_xs_apr->size[1] = i28 - i27;
    emxEnsureCapacity((emxArray__common *)P_xs_apr, cr, (int)sizeof(double));
    i = 12 * (i28 - i27);
    for (cr = 0; cr < i; cr++) {
      P_xs_apr->data[cr] = 0.0;
    }

    if (i28 - i27 == 0) {
    } else {
      i = 12 * ((i28 - i27) - 1);
      for (cr = 0; cr <= i; cr += 12) {
        for (ic = cr; ic + 1 <= cr + 12; ic++) {
          P_xs_apr->data[ic] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= i; cr += 12) {
        ar = 0;
        i28 = br + k;
        for (ib = br; ib + 1 <= i28; ib++) {
          if (P_apo->data[ib % loop_ub + P_apo->size[0] * (i27 +
               div_nzp_s32_floor(ib, loop_ub))] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              P_xs_apr->data[ic] += P_apo->data[ib % loop_ub + P_apo->size[0] *
                (i27 + div_nzp_s32_floor(ib, loop_ub))] * b_Phi->data[ia - 1];
            }
          }

          ar += 12;
        }

        br += k;
      }
    }
  }

  emxFree_real_T(&b_P_apo);
  emxFree_real_T(&b_Phi);

  //  covariance between current state and trails
  c = b_norm(*(double (*)[4])&x->data[3]);
  for (i27 = 0; i27 < 4; i27++) {
    b_xx[i27] = x->data[3 + i27] / c;
  }

  for (i27 = 0; i27 < 4; i27++) {
    x->data[3 + i27] = b_xx[i27];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (1.0 > c_numStates) {
    i = 0;
  } else {
    i = (int)c_numStates;
  }

  emxInit_int32_T(&r18, 1);
  i27 = r18->size[0];
  r18->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r18, i27, (int)sizeof(int));
  for (i27 = 0; i27 < loop_ub; i27++) {
    r18->data[i27] = i27;
  }

  emxInit_int32_T(&r19, 1);
  i27 = r19->size[0];
  r19->size[0] = i;
  emxEnsureCapacity((emxArray__common *)r19, i27, (int)sizeof(int));
  for (i27 = 0; i27 < i; i27++) {
    r19->data[i27] = i27;
  }

  for (i27 = 0; i27 < 12; i27++) {
    for (i28 = 0; i28 < 12; i28++) {
      c_P_xx_apr[i28 + 12 * i27] = (b_P_xx_apr[i28 + 12 * i27] + b_P_xx_apr[i27
        + 12 * i28]) / 2.0;
    }
  }

  i = r18->size[0];
  cr = r19->size[0];
  for (i27 = 0; i27 < cr; i27++) {
    for (i28 = 0; i28 < i; i28++) {
      P_apo->data[r18->data[i28] + P_apo->size[0] * r19->data[i27]] =
        c_P_xx_apr[i28 + i * i27];
    }
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  if (c_numStates + 1.0 > P_apo->size[1]) {
    i27 = 0;
    i28 = 0;
  } else {
    i27 = (int)(c_numStates + 1.0) - 1;
    i28 = P_apo->size[1];
  }

  cr = r18->size[0];
  r18->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r18, cr, (int)sizeof(int));
  for (cr = 0; cr < loop_ub; cr++) {
    r18->data[cr] = cr;
  }

  cr = r19->size[0];
  r19->size[0] = i28 - i27;
  emxEnsureCapacity((emxArray__common *)r19, cr, (int)sizeof(int));
  loop_ub = i28 - i27;
  for (i28 = 0; i28 < loop_ub; i28++) {
    r19->data[i28] = i27 + i28;
  }

  i = r18->size[0];
  cr = r19->size[0];
  for (i27 = 0; i27 < cr; i27++) {
    for (i28 = 0; i28 < i; i28++) {
      P_apo->data[r18->data[i28] + P_apo->size[0] * r19->data[i27]] =
        P_xs_apr->data[i28 + i * i27];
    }
  }

  if (c_numStates + 1.0 > P_apo->size[0]) {
    i27 = 0;
    i28 = 0;
  } else {
    i27 = (int)(c_numStates + 1.0) - 1;
    i28 = P_apo->size[0];
  }

  if (1.0 > c_numStates) {
    loop_ub = 0;
  } else {
    loop_ub = (int)c_numStates;
  }

  cr = r18->size[0];
  r18->size[0] = i28 - i27;
  emxEnsureCapacity((emxArray__common *)r18, cr, (int)sizeof(int));
  i = i28 - i27;
  for (i28 = 0; i28 < i; i28++) {
    r18->data[i28] = i27 + i28;
  }

  i27 = r19->size[0];
  r19->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)r19, i27, (int)sizeof(int));
  for (i27 = 0; i27 < loop_ub; i27++) {
    r19->data[i27] = i27;
  }

  emxInit_real_T(&b_P_xs_apr, 2);
  i27 = b_P_xs_apr->size[0] * b_P_xs_apr->size[1];
  b_P_xs_apr->size[0] = P_xs_apr->size[1];
  b_P_xs_apr->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_P_xs_apr, i27, (int)sizeof(double));
  for (i27 = 0; i27 < 12; i27++) {
    loop_ub = P_xs_apr->size[1];
    for (i28 = 0; i28 < loop_ub; i28++) {
      b_P_xs_apr->data[i28 + b_P_xs_apr->size[0] * i27] = P_xs_apr->data[i27 +
        P_xs_apr->size[0] * i28];
    }
  }

  emxFree_real_T(&P_xs_apr);
  i = r18->size[0];
  cr = r19->size[0];
  for (i27 = 0; i27 < cr; i27++) {
    for (i28 = 0; i28 < i; i28++) {
      P_apo->data[r18->data[i28] + P_apo->size[0] * r19->data[i27]] =
        b_P_xs_apr->data[i28 + i * i27];
    }
  }

  emxFree_real_T(&b_P_xs_apr);
  emxFree_int32_T(&r19);
  emxFree_int32_T(&r18);

  //  P_apr=(P_apr+P_apr')/2;
  // % =================================================================================================== 
}

//
// % Iterative Camera Pose optimization (EKF)
// Arguments    : emxArray_real_T *P_apr
//                emxArray_real_T *b_xt
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                const double cameraparams_R_rl[9]
//                double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                const double imNoise[2]
//                const double IMU_measurements[23]
//                double numPointsPerAnchor
//                double numAnchors
//                double height_offset_pressure
//                emxArray_real_T *h_u_apo
//                emxArray_real_T *map
// Return Type  : void
//
static void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double cameraparams_r_lr[3], const double cameraparams_R_lr[9], const double
  cameraparams_R_rl[9], double updateVect[16], const double z_all_l[32], const
  double z_all_r[32], const double imNoise[2], const double IMU_measurements[23],
  double numPointsPerAnchor, double numAnchors, double height_offset_pressure,
  emxArray_real_T *h_u_apo, emxArray_real_T *map)
{
  double c_numTrackFeatures;
  int i14;
  int loop_ub;
  int i;
  emxArray_real_T *anchorIdx;
  emxArray_int32_T *r8;
  emxArray_boolean_T *x;
  emxArray_int32_T *ii;
  emxArray_real_T *b_anchorIdx;
  int nx;
  int idx;
  int nm1d2;
  boolean_T exitg7;
  boolean_T guard7 = false;
  int n;
  int k;
  int i15;
  int unnamed_idx_0;
  emxArray_boolean_T *r9;
  boolean_T b_x[16];
  int ii_data[16];
  boolean_T exitg6;
  boolean_T guard6 = false;
  signed char indMeas_data[16];
  int i16;
  int validFeatures_size[1];
  double validFeatures_data[16];
  emxArray_real_T *c_anchorIdx;
  boolean_T exitg5;
  boolean_T guard5 = false;
  emxArray_real_T *sigmaInits;
  double initializeNewAnchor;
  int minNumValidFeatures;
  boolean_T exitg4;
  boolean_T guard4 = false;
  double R_cw[9];
  double r_wc[3];
  double anew;
  double c_xt[7];
  emxArray_boolean_T *r10;
  boolean_T exitg3;
  boolean_T guard3 = false;
  int ii_size_idx_0;
  unsigned int featureAnchorIdx;
  int unusedFeatureIdx;
  double z_curr_l[2];
  double z_curr_r[2];
  double m[3];
  double fp[3];
  boolean_T bv0[3];
  double b_R_cw[3];
  double b_fp[3];
  double h_u_r[2];
  double h_u_l[2];
  double b_h_u_l[2];
  boolean_T guard1 = false;
  signed char i17;
  double apnd;
  double ndbl;
  double cdiff;
  emxArray_real_T *J;
  static const signed char iv12[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  emxArray_real_T *y;
  int b_loop_ub;
  int i18;
  int b_m;
  int cr;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  emxArray_real_T *b;
  boolean_T exitg2;
  boolean_T guard2 = false;
  double newFeaturesRequested;
  emxArray_boolean_T *r11;
  boolean_T exitg1;
  boolean_T b_guard1 = false;
  emxArray_real_T *unusedU1;
  emxArray_boolean_T *r12;
  c_numTrackFeatures = numAnchors * numPointsPerAnchor;
  if (!m_vect_not_empty) {
    i14 = m_vect->size[0] * m_vect->size[1];
    m_vect->size[0] = 3;
    m_vect->size[1] = (int)(numPointsPerAnchor * numAnchors);
    emxEnsureCapacity((emxArray__common *)m_vect, i14, (int)sizeof(double));
    loop_ub = 3 * (int)(numPointsPerAnchor * numAnchors);
    for (i14 = 0; i14 < loop_ub; i14++) {
      m_vect->data[i14] = rtNaN;
    }

    m_vect_not_empty = !(m_vect->size[1] == 0);

    //  a matrix containing the m vectors for each feature
    i14 = anchorFeatures->size[0] * anchorFeatures->size[1];
    anchorFeatures->size[0] = 16;
    anchorFeatures->size[1] = (int)numAnchors;
    emxEnsureCapacity((emxArray__common *)anchorFeatures, i14, (int)sizeof
                      (double));
    loop_ub = (int)numAnchors << 4;
    for (i14 = 0; i14 < loop_ub; i14++) {
      anchorFeatures->data[i14] = 0.0;
    }

    //  describes which feature belongs to which anchor
  }

  //  debug check
  // % check for lost features
  i = 0;
  emxInit_real_T(&anchorIdx, 2);
  emxInit_int32_T(&r8, 1);
  emxInit_boolean_T(&x, 2);
  b_emxInit_int32_T(&ii, 2);
  b_emxInit_real_T(&b_anchorIdx, 1);
  while (i <= (int)c_numTrackFeatures - 1) {
    loop_ub = anchorFeatures->size[1];
    i14 = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)x, i14, (int)sizeof(boolean_T));
    for (i14 = 0; i14 < loop_ub; i14++) {
      x->data[x->size[0] * i14] = (anchorFeatures->data[i + anchorFeatures->
        size[0] * i14] == 1.0);
    }

    nx = x->size[1];
    idx = 0;
    i14 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = x->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i14, (int)sizeof(int));
    nm1d2 = 1;
    exitg7 = false;
    while ((!exitg7) && (nm1d2 <= nx)) {
      guard7 = false;
      if (x->data[nm1d2 - 1]) {
        idx++;
        ii->data[idx - 1] = nm1d2;
        if (idx >= nx) {
          exitg7 = true;
        } else {
          guard7 = true;
        }
      } else {
        guard7 = true;
      }

      if (guard7) {
        nm1d2++;
      }
    }

    if (x->size[1] == 1) {
      if (idx == 0) {
        i14 = ii->size[0] * ii->size[1];
        ii->size[0] = 1;
        ii->size[1] = 0;
        emxEnsureCapacity((emxArray__common *)ii, i14, (int)sizeof(int));
      }
    } else {
      i14 = ii->size[0] * ii->size[1];
      if (1 > idx) {
        ii->size[1] = 0;
      } else {
        ii->size[1] = idx;
      }

      emxEnsureCapacity((emxArray__common *)ii, i14, (int)sizeof(int));
    }

    i14 = anchorIdx->size[0] * anchorIdx->size[1];
    anchorIdx->size[0] = 1;
    anchorIdx->size[1] = ii->size[1];
    emxEnsureCapacity((emxArray__common *)anchorIdx, i14, (int)sizeof(double));
    loop_ub = ii->size[0] * ii->size[1];
    for (i14 = 0; i14 < loop_ub; i14++) {
      anchorIdx->data[i14] = ii->data[i14];
    }

    if ((!(anchorIdx->size[1] == 0)) && (updateVect[i] != 1.0)) {
      n = 0;
      i14 = b_anchorIdx->size[0];
      b_anchorIdx->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)b_anchorIdx, i14, (int)sizeof(double));
      loop_ub = anchorIdx->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        b_anchorIdx->data[i14] = anchorIdx->data[anchorIdx->size[0] * i14];
      }

      i14 = (i + 1) * b_anchorIdx->size[0];
      for (k = 0; k < i14; k++) {
        i15 = r8->size[0];
        r8->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i15 = 0; i15 < loop_ub; i15++) {
          r8->data[i15] = (int)anchorIdx->data[anchorIdx->size[0] * i15];
        }

        if (anchorFeatures->data[k % (i + 1) + anchorFeatures->size[0] *
            (r8->data[div_nzp_s32_floor(k, i + 1)] - 1)] != 0.0) {
          n++;
        }
      }

      //  remove covariance of this feature with rest of state
      i14 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        r8->data[i14] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0] *
          i14] - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (double)n);
      }

      loop_ub = P_apr->size[1];
      unnamed_idx_0 = r8->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        for (i15 = 0; i15 < unnamed_idx_0; i15++) {
          P_apr->data[(r8->data[i15] + P_apr->size[0] * i14) - 1] = 0.0;
        }
      }

      i14 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        r8->data[i14] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0] *
          i14] - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (double)n);
      }

      loop_ub = P_apr->size[0];
      nm1d2 = r8->size[0];
      for (i14 = 0; i14 < nm1d2; i14++) {
        for (i15 = 0; i15 < loop_ub; i15++) {
          P_apr->data[i15 + P_apr->size[0] * (r8->data[i14] - 1)] = 0.0;
        }
      }

      i14 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        r8->data[i14] = (int)anchorIdx->data[anchorIdx->size[0] * i14];
      }

      nm1d2 = r8->size[0];
      for (i14 = 0; i14 < nm1d2; i14++) {
        anchorFeatures->data[i + anchorFeatures->size[0] * (r8->data[i14] - 1)] =
          -1.0;
      }

      //  mark feature as lost
    }

    i++;
  }

  emxFree_real_T(&b_anchorIdx);
  emxInit_boolean_T(&r9, 2);

  // % do the update
  i14 = r9->size[0] * r9->size[1];
  r9->size[0] = 16;
  r9->size[1] = anchorFeatures->size[1];
  emxEnsureCapacity((emxArray__common *)r9, i14, (int)sizeof(boolean_T));
  loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
  for (i14 = 0; i14 < loop_ub; i14++) {
    r9->data[i14] = (anchorFeatures->data[i14] == 1.0);
  }

  b_any(r9, b_x);
  idx = 0;
  nm1d2 = 1;
  emxFree_boolean_T(&r9);
  exitg6 = false;
  while ((!exitg6) && (nm1d2 < 17)) {
    guard6 = false;
    if (b_x[nm1d2 - 1]) {
      idx++;
      ii_data[idx - 1] = nm1d2;
      if (idx >= 16) {
        exitg6 = true;
      } else {
        guard6 = true;
      }
    } else {
      guard6 = true;
    }

    if (guard6) {
      nm1d2++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  for (i14 = 0; i14 < loop_ub; i14++) {
    indMeas_data[i14] = (signed char)ii_data[i14];
  }

  if (1 > idx) {
    i16 = 0;
  } else {
    i16 = idx;
  }

  if (!(i16 == 0)) {
    //   [ xt, P_apo, validFeatures ] = Mahalanobis_EKF(xt, P_apr, z_all_l, numStatesxt, numStates, numPointsPerAnchor, cameraparams, anchorFeatures, m_vect, imNoise,IMU_measurements,height_offset_pressure); 
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, numStatesxt, numStates,
                       numPointsPerAnchor, anchorFeatures, m_vect, imNoise,
                       IMU_measurements, height_offset_pressure,
                       validFeatures_data, validFeatures_size);
    for (i14 = 0; i14 < loop_ub; i14++) {
      ii_data[i14] = indMeas_data[i14];
    }

    for (i14 = 0; i14 < loop_ub; i14++) {
      updateVect[ii_data[i14] - 1] = 0.0;
    }

    loop_ub = validFeatures_size[0];
    for (i14 = 0; i14 < loop_ub; i14++) {
      ii_data[i14] = (int)validFeatures_data[i14];
    }

    loop_ub = validFeatures_size[0];
    for (i14 = 0; i14 < loop_ub; i14++) {
      updateVect[ii_data[i14] - 1] = 1.0;
    }

    //  check for lost features
    i = 0;
    b_emxInit_real_T(&c_anchorIdx, 1);
    while (i <= (int)c_numTrackFeatures - 1) {
      loop_ub = anchorFeatures->size[1];
      i14 = x->size[0] * x->size[1];
      x->size[0] = 1;
      x->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)x, i14, (int)sizeof(boolean_T));
      for (i14 = 0; i14 < loop_ub; i14++) {
        x->data[x->size[0] * i14] = (anchorFeatures->data[i +
          anchorFeatures->size[0] * i14] == 1.0);
      }

      nx = x->size[1];
      idx = 0;
      i14 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = x->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i14, (int)sizeof(int));
      nm1d2 = 1;
      exitg5 = false;
      while ((!exitg5) && (nm1d2 <= nx)) {
        guard5 = false;
        if (x->data[nm1d2 - 1]) {
          idx++;
          ii->data[idx - 1] = nm1d2;
          if (idx >= nx) {
            exitg5 = true;
          } else {
            guard5 = true;
          }
        } else {
          guard5 = true;
        }

        if (guard5) {
          nm1d2++;
        }
      }

      if (x->size[1] == 1) {
        if (idx == 0) {
          i14 = ii->size[0] * ii->size[1];
          ii->size[0] = 1;
          ii->size[1] = 0;
          emxEnsureCapacity((emxArray__common *)ii, i14, (int)sizeof(int));
        }
      } else {
        i14 = ii->size[0] * ii->size[1];
        if (1 > idx) {
          ii->size[1] = 0;
        } else {
          ii->size[1] = idx;
        }

        emxEnsureCapacity((emxArray__common *)ii, i14, (int)sizeof(int));
      }

      i14 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = ii->size[1];
      emxEnsureCapacity((emxArray__common *)anchorIdx, i14, (int)sizeof(double));
      loop_ub = ii->size[0] * ii->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        anchorIdx->data[i14] = ii->data[i14];
      }

      if ((!(anchorIdx->size[1] == 0)) && (updateVect[i] != 1.0)) {
        n = 0;
        i14 = c_anchorIdx->size[0];
        c_anchorIdx->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)c_anchorIdx, i14, (int)sizeof
                          (double));
        loop_ub = anchorIdx->size[1];
        for (i14 = 0; i14 < loop_ub; i14++) {
          c_anchorIdx->data[i14] = anchorIdx->data[anchorIdx->size[0] * i14];
        }

        i14 = (i + 1) * c_anchorIdx->size[0];
        for (k = 0; k < i14; k++) {
          i15 = r8->size[0];
          r8->size[0] = anchorIdx->size[1];
          emxEnsureCapacity((emxArray__common *)r8, i15, (int)sizeof(int));
          loop_ub = anchorIdx->size[1];
          for (i15 = 0; i15 < loop_ub; i15++) {
            r8->data[i15] = (int)anchorIdx->data[anchorIdx->size[0] * i15];
          }

          if (anchorFeatures->data[k % (i + 1) + anchorFeatures->size[0] *
              (r8->data[div_nzp_s32_floor(k, i + 1)] - 1)] != 0.0) {
            n++;
          }
        }

        //  remove covariance of this feature with rest of state
        i14 = r8->size[0];
        r8->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i14 = 0; i14 < loop_ub; i14++) {
          r8->data[i14] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0]
            * i14] - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (double)n);
        }

        loop_ub = P_apr->size[1];
        unnamed_idx_0 = r8->size[0];
        for (i14 = 0; i14 < loop_ub; i14++) {
          for (i15 = 0; i15 < unnamed_idx_0; i15++) {
            P_apr->data[(r8->data[i15] + P_apr->size[0] * i14) - 1] = 0.0;
          }
        }

        i14 = r8->size[0];
        r8->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i14 = 0; i14 < loop_ub; i14++) {
          r8->data[i14] = (int)(((numStates + (anchorIdx->data[anchorIdx->size[0]
            * i14] - 1.0) * (6.0 + numPointsPerAnchor)) + 6.0) + (double)n);
        }

        loop_ub = P_apr->size[0];
        nm1d2 = r8->size[0];
        for (i14 = 0; i14 < nm1d2; i14++) {
          for (i15 = 0; i15 < loop_ub; i15++) {
            P_apr->data[i15 + P_apr->size[0] * (r8->data[i14] - 1)] = 0.0;
          }
        }

        i14 = r8->size[0];
        r8->size[0] = anchorIdx->size[1];
        emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
        loop_ub = anchorIdx->size[1];
        for (i14 = 0; i14 < loop_ub; i14++) {
          r8->data[i14] = (int)anchorIdx->data[anchorIdx->size[0] * i14];
        }

        nm1d2 = r8->size[0];
        for (i14 = 0; i14 < nm1d2; i14++) {
          anchorFeatures->data[i + anchorFeatures->size[0] * (r8->data[i14] - 1)]
            = -1.0;
        }

        //  mark feature as lost
      }

      i++;
    }

    emxFree_real_T(&c_anchorIdx);
  }

  emxFree_int32_T(&ii);
  emxFree_boolean_T(&x);

  // % Initialize new anchors
  for (i = 0; i < 16; i++) {
    b_x[i] = (updateVect[i] == 2.0);
  }

  b_emxInit_real_T(&sigmaInits, 1);
  if (any(b_x)) {
    //  if there are any features with stereo measurements
    initializeNewAnchor = 0.0;
    minNumValidFeatures = 10000;
    nm1d2 = 0;
    exitg4 = false;
    while ((!exitg4) && (nm1d2 <= (int)numAnchors - 1)) {
      for (i14 = 0; i14 < 16; i14++) {
        b_x[i14] = ((anchorFeatures->data[i14 + anchorFeatures->size[0] * nm1d2]
                     == 1.0) && (updateVect[i14] == 1.0));
      }

      n = 0;
      for (k = 0; k < 16; k++) {
        if (b_x[k]) {
          n++;
        }
      }

      guard4 = false;
      if ((n < minFeatureThreshold) && (n < minNumValidFeatures)) {
        minNumValidFeatures = n;
        initializeNewAnchor = 1.0 + (double)nm1d2;
        if (!(n != 0)) {
          exitg4 = true;
        } else {
          guard4 = true;
        }
      } else {
        guard4 = true;
      }

      if (guard4) {
        nm1d2++;
      }
    }

    if (initializeNewAnchor > 0.0) {
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4])
                 - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
      R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] *
                       b_xt->data[6]);
      R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] *
                       b_xt->data[6]);
      R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] *
                       b_xt->data[6]);
      R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data
                  [4]) - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] *
        b_xt->data[6];
      R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] *
                       b_xt->data[6]);
      R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] *
                       b_xt->data[6]);
      R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] *
                       b_xt->data[6]);
      R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data
                  [4]) + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] *
        b_xt->data[6];
      for (i14 = 0; i14 < 3; i14++) {
        r_wc[i14] = b_xt->data[i14];
      }

      for (i14 = 0; i14 < 16; i14++) {
        b_x[i14] = (anchorFeatures->data[i14 + anchorFeatures->size[0] * ((int)
          initializeNewAnchor - 1)] == 1.0);
      }

      n = 0;
      for (i = 0; i < 16; i++) {
        if (b_x[i]) {
          n++;
        }
      }

      nm1d2 = 0;
      for (i = 0; i < 16; i++) {
        if (b_x[i]) {
          ii_data[nm1d2] = i + 1;
          nm1d2++;
        }
      }

      for (i14 = 0; i14 < n; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          m_vect->data[i15 + m_vect->size[0] * (ii_data[i14] - 1)] = rtNaN;
        }
      }

      for (i14 = 0; i14 < 16; i14++) {
        anchorFeatures->data[i14 + anchorFeatures->size[0] * ((int)
          initializeNewAnchor - 1)] = 0.0;
      }

      anew = numStatesxt + (initializeNewAnchor - 1.0) * (7.0 +
        numPointsPerAnchor);
      for (i14 = 0; i14 < 7; i14++) {
        c_xt[i14] = b_xt->data[i14];
      }

      for (i14 = 0; i14 < 7; i14++) {
        b_xt->data[(int)(anew + (1.0 + (double)i14)) - 1] = c_xt[i14];
      }

      i14 = sigmaInits->size[0];
      sigmaInits->size[0] = (int)numPointsPerAnchor;
      emxEnsureCapacity((emxArray__common *)sigmaInits, i14, (int)sizeof(double));
      loop_ub = (int)numPointsPerAnchor;
      for (i14 = 0; i14 < loop_ub; i14++) {
        sigmaInits->data[i14] = 0.0;
      }

      emxInit_boolean_T(&r10, 2);
      i14 = r10->size[0] * r10->size[1];
      r10->size[0] = 16;
      r10->size[1] = anchorFeatures->size[1];
      emxEnsureCapacity((emxArray__common *)r10, i14, (int)sizeof(boolean_T));
      loop_ub = anchorFeatures->size[0] * anchorFeatures->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        r10->data[i14] = (anchorFeatures->data[i14] == 1.0);
      }

      b_any(r10, b_x);
      emxFree_boolean_T(&r10);
      for (i14 = 0; i14 < 16; i14++) {
        b_x[i14] = !b_x[i14];
      }

      idx = 0;
      nm1d2 = 1;
      exitg3 = false;
      while ((!exitg3) && (nm1d2 < 17)) {
        guard3 = false;
        if (b_x[nm1d2 - 1] && (updateVect[nm1d2 - 1] == 2.0)) {
          idx++;
          ii_data[idx - 1] = nm1d2;
          if (idx >= 16) {
            exitg3 = true;
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }

        if (guard3) {
          nm1d2++;
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

      for (i14 = 0; i14 < loop_ub; i14++) {
        indMeas_data[i14] = (signed char)ii_data[i14];
      }

      featureAnchorIdx = 1U;
      unusedFeatureIdx = 0;
      while ((unusedFeatureIdx <= ii_size_idx_0 - 1) && (!(featureAnchorIdx >
               numPointsPerAnchor))) {
        nm1d2 = (indMeas_data[unusedFeatureIdx] - 1) * 2;
        k = (indMeas_data[unusedFeatureIdx] - 1) * 2;
        for (i14 = 0; i14 < 2; i14++) {
          z_curr_l[i14] = z_all_l[nm1d2 + i14];
          z_curr_r[i14] = z_all_r[k + i14];
        }

        initializePoint(b_xt, cameraparams_r_lr, cameraparams_R_lr, z_curr_l,
                        z_curr_r, fp, m);
        for (i = 0; i < 3; i++) {
          bv0[i] = rtIsNaN(fp[i]);
        }

        if (c_any(bv0)) {
          updateVect[indMeas_data[unusedFeatureIdx] - 1] = 0.0;
        } else {
          //  check reprojection error
          for (i14 = 0; i14 < 3; i14++) {
            b_fp[i14] = fp[i14] - r_wc[i14];
          }

          for (i14 = 0; i14 < 3; i14++) {
            b_R_cw[i14] = 0.0;
            for (i15 = 0; i15 < 3; i15++) {
              b_R_cw[i14] += R_cw[i14 + 3 * i15] * b_fp[i15];
            }
          }

          predictMeasurement_stereo(b_R_cw, cameraparams_r_lr, cameraparams_R_rl,
            h_u_l, h_u_r);
          for (i = 0; i < 2; i++) {
            b_h_u_l[i] = h_u_l[i] - z_curr_l[i];
          }

          guard1 = false;
          if (c_norm(b_h_u_l) > 10.0) {
            guard1 = true;
          } else {
            for (i = 0; i < 2; i++) {
              b_h_u_l[i] = h_u_r[i] - z_curr_r[i];
            }

            if (c_norm(b_h_u_l) > 10.0) {
              guard1 = true;
            } else {
              for (i14 = 0; i14 < 3; i14++) {
                m_vect->data[i14 + m_vect->size[0] *
                  (indMeas_data[unusedFeatureIdx] - 1)] = m[i14];
              }

              for (i14 = 0; i14 < 3; i14++) {
                b_fp[i14] = b_xt->data[i14] - fp[i14];
              }

              b_xt->data[(int)(((numStatesxt + (initializeNewAnchor - 1.0) *
                                 (7.0 + numPointsPerAnchor)) + 7.0) + (double)
                               featureAnchorIdx) - 1] = 1.0 / norm(b_fp);
              sigmaInits->data[(int)featureAnchorIdx - 1] = sigma_Init;
              anchorFeatures->data[(indMeas_data[unusedFeatureIdx] +
                                    anchorFeatures->size[0] * ((int)
                initializeNewAnchor - 1)) - 1] = 1.0;
              updateVect[indMeas_data[unusedFeatureIdx] - 1] = 1.0;
              featureAnchorIdx++;
            }
          }

          if (guard1) {
            i14 = (int)rt_roundd_snf((double)indMeas_data[unusedFeatureIdx]);
            i17 = (signed char)i14;
            d_fprintf(i17);
            updateVect[indMeas_data[unusedFeatureIdx] - 1] = 0.0;
          }
        }

        unusedFeatureIdx++;
      }

      anew = rt_roundd_snf((double)featureAnchorIdx - 1.0);
      if (anew < 2.147483648E+9) {
        i14 = (int)anew;
      } else {
        i14 = MAX_int32_T;
      }

      f_fprintf(i14, (int)initializeNewAnchor);
      if (rtIsInf(6.0 + numPointsPerAnchor)) {
        n = 0;
        anew = rtNaN;
        apnd = 6.0 + numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
        if (fabs(cdiff) < 4.4408920985006262E-16 * (6.0 + numPointsPerAnchor)) {
          ndbl++;
          apnd = 6.0 + numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        n = (int)ndbl - 1;
      }

      i14 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i14, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          nm1d2 = (n + (n < 0)) >> 1;
          for (k = 1; k < nm1d2; k++) {
            anchorIdx->data[k] = anew + (double)k;
            anchorIdx->data[n - k] = apnd - (double)k;
          }

          if (nm1d2 << 1 == n) {
            anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[nm1d2] = anew + (double)nm1d2;
            anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      anew = numStates + (initializeNewAnchor - 1.0) * (6.0 + numPointsPerAnchor);
      i14 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        r8->data[i14] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i14]);
      }

      loop_ub = P_apr->size[1];
      unnamed_idx_0 = r8->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        for (i15 = 0; i15 < unnamed_idx_0; i15++) {
          P_apr->data[(r8->data[i15] + P_apr->size[0] * i14) - 1] = 0.0;
        }
      }

      if (rtIsInf(6.0 + numPointsPerAnchor)) {
        n = 0;
        anew = rtNaN;
        apnd = 6.0 + numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
        if (fabs(cdiff) < 4.4408920985006262E-16 * (6.0 + numPointsPerAnchor)) {
          ndbl++;
          apnd = 6.0 + numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        n = (int)ndbl - 1;
      }

      i14 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i14, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          nm1d2 = (n + (n < 0)) >> 1;
          for (k = 1; k < nm1d2; k++) {
            anchorIdx->data[k] = anew + (double)k;
            anchorIdx->data[n - k] = apnd - (double)k;
          }

          if (nm1d2 << 1 == n) {
            anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[nm1d2] = anew + (double)nm1d2;
            anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      anew = numStates + (initializeNewAnchor - 1.0) * (6.0 + numPointsPerAnchor);
      i14 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        r8->data[i14] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i14]);
      }

      loop_ub = P_apr->size[0];
      nm1d2 = r8->size[0];
      for (i14 = 0; i14 < nm1d2; i14++) {
        for (i15 = 0; i15 < loop_ub; i15++) {
          P_apr->data[i15 + P_apr->size[0] * (r8->data[i14] - 1)] = 0.0;
        }
      }

      for (i14 = 0; i14 < 2; i14++) {
        b_h_u_l[i14] = P_apr->size[i14];
      }

      emxInit_real_T(&J, 2);
      b_eye(b_h_u_l, J);
      if (rtIsInf(6.0 + numPointsPerAnchor)) {
        n = 0;
        anew = rtNaN;
        apnd = 6.0 + numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
        if (fabs(cdiff) < 4.4408920985006262E-16 * (6.0 + numPointsPerAnchor)) {
          ndbl++;
          apnd = 6.0 + numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        n = (int)ndbl - 1;
      }

      i14 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i14, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          nm1d2 = (n + (n < 0)) >> 1;
          for (k = 1; k < nm1d2; k++) {
            anchorIdx->data[k] = anew + (double)k;
            anchorIdx->data[n - k] = apnd - (double)k;
          }

          if (nm1d2 << 1 == n) {
            anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[nm1d2] = anew + (double)nm1d2;
            anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      anew = numStates + (initializeNewAnchor - 1.0) * (6.0 + numPointsPerAnchor);
      i14 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        r8->data[i14] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i14]);
      }

      loop_ub = J->size[1];
      unnamed_idx_0 = r8->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        for (i15 = 0; i15 < unnamed_idx_0; i15++) {
          J->data[(r8->data[i15] + J->size[0] * i14) - 1] = 0.0;
        }
      }

      if (rtIsInf(6.0 + numPointsPerAnchor)) {
        n = 0;
        anew = rtNaN;
        apnd = 6.0 + numPointsPerAnchor;
      } else {
        anew = 1.0;
        ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
        apnd = 1.0 + ndbl;
        cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
        if (fabs(cdiff) < 4.4408920985006262E-16 * (6.0 + numPointsPerAnchor)) {
          ndbl++;
          apnd = 6.0 + numPointsPerAnchor;
        } else if (cdiff > 0.0) {
          apnd = 1.0 + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        n = (int)ndbl - 1;
      }

      i14 = anchorIdx->size[0] * anchorIdx->size[1];
      anchorIdx->size[0] = 1;
      anchorIdx->size[1] = n + 1;
      emxEnsureCapacity((emxArray__common *)anchorIdx, i14, (int)sizeof(double));
      if (n + 1 > 0) {
        anchorIdx->data[0] = anew;
        if (n + 1 > 1) {
          anchorIdx->data[n] = apnd;
          nm1d2 = (n + (n < 0)) >> 1;
          for (k = 1; k < nm1d2; k++) {
            anchorIdx->data[k] = anew + (double)k;
            anchorIdx->data[n - k] = apnd - (double)k;
          }

          if (nm1d2 << 1 == n) {
            anchorIdx->data[nm1d2] = (anew + apnd) / 2.0;
          } else {
            anchorIdx->data[nm1d2] = anew + (double)nm1d2;
            anchorIdx->data[nm1d2 + 1] = apnd - (double)nm1d2;
          }
        }
      }

      anew = numStates + (initializeNewAnchor - 1.0) * (6.0 + numPointsPerAnchor);
      i14 = r8->size[0];
      r8->size[0] = anchorIdx->size[1];
      emxEnsureCapacity((emxArray__common *)r8, i14, (int)sizeof(int));
      loop_ub = anchorIdx->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        r8->data[i14] = (int)(anew + anchorIdx->data[anchorIdx->size[0] * i14])
          - 1;
      }

      for (i14 = 0; i14 < 3; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          J->data[r8->data[i15] + J->size[0] * i14] = iv12[i15 + 3 * i14];
        }
      }

      for (i14 = 0; i14 < 3; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          J->data[r8->data[i15] + J->size[0] * (i14 + 3)] = 0.0;
        }
      }

      loop_ub = (int)(numStates - 6.0);
      for (i14 = 0; i14 < loop_ub; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          J->data[r8->data[i15] + J->size[0] * (i14 + 6)] = 0.0;
        }
      }

      for (i14 = 0; i14 < 3; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          J->data[r8->data[i15 + 3] + J->size[0] * i14] = 0.0;
        }
      }

      for (i14 = 0; i14 < 3; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          J->data[r8->data[i15 + 3] + J->size[0] * (i14 + 3)] = iv12[i15 + 3 *
            i14];
        }
      }

      loop_ub = (int)(numStates - 6.0);
      for (i14 = 0; i14 < loop_ub; i14++) {
        for (i15 = 0; i15 < 3; i15++) {
          J->data[r8->data[i15 + 3] + J->size[0] * (i14 + 6)] = 0.0;
        }
      }

      loop_ub = (int)numStates;
      for (i14 = 0; i14 < loop_ub; i14++) {
        nm1d2 = (int)numPointsPerAnchor;
        for (i15 = 0; i15 < nm1d2; i15++) {
          J->data[r8->data[i15 + 6] + J->size[0] * i14] = 0.0;
        }
      }

      emxInit_real_T(&y, 2);
      if ((J->size[1] == 1) || (P_apr->size[0] == 1)) {
        i14 = y->size[0] * y->size[1];
        y->size[0] = J->size[0];
        y->size[1] = P_apr->size[1];
        emxEnsureCapacity((emxArray__common *)y, i14, (int)sizeof(double));
        loop_ub = J->size[0];
        for (i14 = 0; i14 < loop_ub; i14++) {
          nm1d2 = P_apr->size[1];
          for (i15 = 0; i15 < nm1d2; i15++) {
            y->data[i14 + y->size[0] * i15] = 0.0;
            b_loop_ub = J->size[1];
            for (i18 = 0; i18 < b_loop_ub; i18++) {
              y->data[i14 + y->size[0] * i15] += J->data[i14 + J->size[0] * i18]
                * P_apr->data[i18 + P_apr->size[0] * i15];
            }
          }
        }
      } else {
        k = J->size[1];
        unnamed_idx_0 = J->size[0];
        nm1d2 = P_apr->size[1];
        b_m = J->size[0];
        i14 = y->size[0] * y->size[1];
        y->size[0] = unnamed_idx_0;
        emxEnsureCapacity((emxArray__common *)y, i14, (int)sizeof(double));
        i14 = y->size[0] * y->size[1];
        y->size[1] = nm1d2;
        emxEnsureCapacity((emxArray__common *)y, i14, (int)sizeof(double));
        loop_ub = unnamed_idx_0 * nm1d2;
        for (i14 = 0; i14 < loop_ub; i14++) {
          y->data[i14] = 0.0;
        }

        if ((J->size[0] == 0) || (P_apr->size[1] == 0)) {
        } else {
          nm1d2 = J->size[0] * (P_apr->size[1] - 1);
          cr = 0;
          while ((b_m > 0) && (cr <= nm1d2)) {
            i14 = cr + b_m;
            for (ic = cr; ic + 1 <= i14; ic++) {
              y->data[ic] = 0.0;
            }

            cr += b_m;
          }

          br = 0;
          cr = 0;
          while ((b_m > 0) && (cr <= nm1d2)) {
            ar = 0;
            i14 = br + k;
            for (ib = br; ib + 1 <= i14; ib++) {
              if (P_apr->data[ib] != 0.0) {
                ia = ar;
                i15 = cr + b_m;
                for (ic = cr; ic + 1 <= i15; ic++) {
                  ia++;
                  y->data[ic] += P_apr->data[ib] * J->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            cr += b_m;
          }
        }
      }

      emxInit_real_T(&b, 2);
      i14 = b->size[0] * b->size[1];
      b->size[0] = J->size[1];
      b->size[1] = J->size[0];
      emxEnsureCapacity((emxArray__common *)b, i14, (int)sizeof(double));
      loop_ub = J->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        nm1d2 = J->size[1];
        for (i15 = 0; i15 < nm1d2; i15++) {
          b->data[i15 + b->size[0] * i14] = J->data[i14 + J->size[0] * i15];
        }
      }

      emxFree_real_T(&J);
      if ((y->size[1] == 1) || (b->size[0] == 1)) {
        i14 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = y->size[0];
        P_apr->size[1] = b->size[1];
        emxEnsureCapacity((emxArray__common *)P_apr, i14, (int)sizeof(double));
        loop_ub = y->size[0];
        for (i14 = 0; i14 < loop_ub; i14++) {
          nm1d2 = b->size[1];
          for (i15 = 0; i15 < nm1d2; i15++) {
            P_apr->data[i14 + P_apr->size[0] * i15] = 0.0;
            b_loop_ub = y->size[1];
            for (i18 = 0; i18 < b_loop_ub; i18++) {
              P_apr->data[i14 + P_apr->size[0] * i15] += y->data[i14 + y->size[0]
                * i18] * b->data[i18 + b->size[0] * i15];
            }
          }
        }
      } else {
        k = y->size[1];
        unnamed_idx_0 = y->size[0];
        nm1d2 = b->size[1];
        b_m = y->size[0];
        i14 = P_apr->size[0] * P_apr->size[1];
        P_apr->size[0] = unnamed_idx_0;
        P_apr->size[1] = nm1d2;
        emxEnsureCapacity((emxArray__common *)P_apr, i14, (int)sizeof(double));
        for (i14 = 0; i14 < nm1d2; i14++) {
          for (i15 = 0; i15 < unnamed_idx_0; i15++) {
            P_apr->data[i15 + P_apr->size[0] * i14] = 0.0;
          }
        }

        if ((y->size[0] == 0) || (b->size[1] == 0)) {
        } else {
          nm1d2 = y->size[0] * (b->size[1] - 1);
          cr = 0;
          while ((b_m > 0) && (cr <= nm1d2)) {
            i14 = cr + b_m;
            for (ic = cr; ic + 1 <= i14; ic++) {
              P_apr->data[ic] = 0.0;
            }

            cr += b_m;
          }

          br = 0;
          cr = 0;
          while ((b_m > 0) && (cr <= nm1d2)) {
            ar = 0;
            i14 = br + k;
            for (ib = br; ib + 1 <= i14; ib++) {
              if (b->data[ib] != 0.0) {
                ia = ar;
                i15 = cr + b_m;
                for (ic = cr; ic + 1 <= i15; ic++) {
                  ia++;
                  P_apr->data[ic] += b->data[ib] * y->data[ia - 1];
                }
              }

              ar += b_m;
            }

            br += k;
            cr += b_m;
          }
        }
      }

      emxFree_real_T(&b);
      emxFree_real_T(&y);
      for (nm1d2 = 0; nm1d2 < (int)numPointsPerAnchor; nm1d2++) {
        P_apr->data[((int)(((numStates + (initializeNewAnchor - 1.0) * (6.0 +
          numPointsPerAnchor)) + 6.0) + (1.0 + (double)nm1d2)) + P_apr->size[0] *
                     ((int)(((numStates + (initializeNewAnchor - 1.0) * (6.0 +
          numPointsPerAnchor)) + 6.0) + (1.0 + (double)nm1d2)) - 1)) - 1] =
          sigmaInits->data[nm1d2];
      }
    } else {
      h_fprintf();
    }
  }

  emxFree_int32_T(&r8);
  emxFree_real_T(&anchorIdx);

  //  determine if a new anchor needs to be initialized, and request stereo
  //  measurements for it
  minNumValidFeatures = 10000;
  nm1d2 = 0;
  exitg2 = false;
  while ((!exitg2) && (nm1d2 <= (int)numAnchors - 1)) {
    for (i14 = 0; i14 < 16; i14++) {
      b_x[i14] = ((anchorFeatures->data[i14 + anchorFeatures->size[0] * nm1d2] ==
                   1.0) && (updateVect[i14] == 1.0));
    }

    n = 0;
    for (k = 0; k < 16; k++) {
      if (b_x[k]) {
        n++;
      }
    }

    guard2 = false;
    if ((n < minFeatureThreshold) && (n < minNumValidFeatures)) {
      minNumValidFeatures = n;
      initializeNewAnchor = 1.0 + (double)nm1d2;
      if (!(n != 0)) {
        exitg2 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      nm1d2++;
    }
  }

  if (minNumValidFeatures < minFeatureThreshold) {
    newFeaturesRequested = 0.0;
    i = 0;
    emxInit_boolean_T(&r11, 2);
    exitg1 = false;
    while ((!exitg1) && (i < 16)) {
      loop_ub = anchorFeatures->size[1];
      i14 = r11->size[0] * r11->size[1];
      r11->size[0] = 1;
      r11->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)r11, i14, (int)sizeof(boolean_T));
      for (i14 = 0; i14 < loop_ub; i14++) {
        r11->data[r11->size[0] * i14] = (anchorFeatures->data[i +
          anchorFeatures->size[0] * i14] == 1.0);
      }

      b_guard1 = false;
      if ((!d_any(r11)) || (anchorFeatures->data[i + anchorFeatures->size[0] *
                            ((int)initializeNewAnchor - 1)] == 1.0)) {
        updateVect[i] = 2.0;
        newFeaturesRequested++;
        if (newFeaturesRequested == numPointsPerAnchor) {
          exitg1 = true;
        } else {
          b_guard1 = true;
        }
      } else {
        b_guard1 = true;
      }

      if (b_guard1) {
        i++;
      }
    }

    emxFree_boolean_T(&r11);

    //      fprintf('Requesting %i new features\n', int8(newFeaturesRequested))
  }

  b_emxInit_real_T(&unusedU1, 1);

  // % aposteriori measurement prediction
  getMap(b_xt, anchorFeatures, m_vect, c_numTrackFeatures, numStatesxt, 7.0 +
         numPointsPerAnchor, map, sigmaInits, unusedU1);
  i14 = h_u_apo->size[0];
  h_u_apo->size[0] = (int)(c_numTrackFeatures * 4.0);
  emxEnsureCapacity((emxArray__common *)h_u_apo, i14, (int)sizeof(double));
  loop_ub = (int)(c_numTrackFeatures * 4.0);
  emxFree_real_T(&unusedU1);
  emxFree_real_T(&sigmaInits);
  for (i14 = 0; i14 < loop_ub; i14++) {
    h_u_apo->data[i14] = rtNaN;
  }

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4]) -
             b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] * b_xt->data[6]);
  R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] * b_xt->data[6]);
  R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] * b_xt->data[6]);
  R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data[4])
             - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] * b_xt->data[6]);
  R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] * b_xt->data[6]);
  R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] * b_xt->data[6]);
  R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data[4])
             + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  emxInit_boolean_T(&r12, 2);
  for (i = 0; i < 16; i++) {
    loop_ub = anchorFeatures->size[1];
    i14 = r12->size[0] * r12->size[1];
    r12->size[0] = 1;
    r12->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)r12, i14, (int)sizeof(boolean_T));
    for (i14 = 0; i14 < loop_ub; i14++) {
      r12->data[r12->size[0] * i14] = (anchorFeatures->data[i +
        anchorFeatures->size[0] * i14] == 1.0);
    }

    if (d_any(r12)) {
      for (i14 = 0; i14 < 3; i14++) {
        b_fp[i14] = map->data[i14 + map->size[0] * i] - b_xt->data[i14];
      }

      for (i14 = 0; i14 < 3; i14++) {
        b_R_cw[i14] = 0.0;
        for (i15 = 0; i15 < 3; i15++) {
          b_R_cw[i14] += R_cw[i14 + 3 * i15] * b_fp[i15];
        }
      }

      predictMeasurement_stereo(b_R_cw, cameraparams_r_lr, cameraparams_R_rl,
        h_u_l, h_u_r);
      nm1d2 = i << 2;
      for (i14 = 0; i14 < 2; i14++) {
        h_u_apo->data[i14 + nm1d2] = h_u_l[i14];
      }

      for (i14 = 0; i14 < 2; i14++) {
        h_u_apo->data[(i14 + nm1d2) + 2] = h_u_r[i14];
      }
    }
  }

  emxFree_boolean_T(&r12);
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_updIT_free()
{
  emxFree_real_T(&anchorFeatures);
  emxFree_real_T(&m_vect);
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_updIT_init()
{
  emxInit_real_T(&anchorFeatures, 2);
  emxInit_real_T(&m_vect, 2);
  m_vect_not_empty = false;
}

//
// Arguments    : const boolean_T x[16]
// Return Type  : boolean_T
//
static boolean_T any(const boolean_T x[16])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
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
// THIS IS OK, It is according to the NASA memo found
//  https://claraty.jpl.nasa.gov/man/software/development/conventions/standards_docs/unadopted/JPL_Quaternions_Breckenridge.pdf
// QUATFROMROTJ Get equivalent quaternion of the rotation matrix R
//    Returns a quaternion in JPL notation
//  The implementation is copied from qGetQ(R), but we are careful about the
//  ordering of the output vector
// Arguments    : double b_Q[4]
// Return Type  : void
//
static void b_QuatFromRotJ(double b_Q[4])
{
  int idx;
  signed char ii_data[4];
  int ii;
  boolean_T exitg1;
  boolean_T guard1 = false;
  static const boolean_T x[4] = { false, false, false, true };

  int loop_ub;
  int i11;
  signed char i_data[4];
  signed char index_data[4];

  //  if( r ~= 3 || c ~= 3 )
  //      error( 'R must be a 3x3 matrix\n\r' );
  //  end
  idx = 0;
  ii = 1;
  exitg1 = false;
  while ((!exitg1) && (ii < 5)) {
    guard1 = false;
    if (x[ii - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)ii;
      if (idx >= 4) {
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
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  for (i11 = 0; i11 < loop_ub; i11++) {
    i_data[i11] = ii_data[i11];
  }

  if (1 > idx) {
    ii = 0;
  } else {
    ii = idx;
  }

  for (i11 = 0; i11 < ii; i11++) {
    index_data[i11] = i_data[i11];
  }

  for (ii = 0; ii < loop_ub; ii++) {
    index_data[ii] = (signed char)rt_roundd_snf((double)index_data[ii]);
  }

  if (index_data[0] == 1) {
    b_Q[0] = 0.0;
    b_Q[1] = rtNaN;
    b_Q[2] = rtNaN;
    b_Q[3] = rtNaN;
  } else if (index_data[0] == 2) {
    b_Q[1] = 0.0;
    b_Q[0] = rtNaN;
    b_Q[2] = rtNaN;
    b_Q[3] = rtNaN;
  } else if (index_data[0] == 3) {
    b_Q[2] = 0.0;
    b_Q[0] = rtNaN;
    b_Q[1] = rtNaN;
    b_Q[3] = rtNaN;
  } else {
    b_Q[3] = 1.0;
    b_Q[0] = 0.0;
    b_Q[1] = 0.0;
    b_Q[2] = 0.0;
  }
}

//
// Arguments    : const emxArray_boolean_T *x
//                boolean_T y[16]
// Return Type  : void
//
static void b_any(const emxArray_boolean_T *x, boolean_T y[16])
{
  int iy;
  int i1;
  int i2;
  int j;
  int ix;
  boolean_T exitg1;
  boolean_T b0;
  for (iy = 0; iy < 16; iy++) {
    y[iy] = false;
  }

  iy = -1;
  i1 = 0;
  i2 = (x->size[1] - 1) << 4;
  for (j = 0; j < 16; j++) {
    i1++;
    i2++;
    iy++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2)) {
      b0 = !x->data[ix - 1];
      if (!b0) {
        y[iy] = true;
        exitg1 = true;
      } else {
        ix += 16;
      }
    }
  }
}

//
// Arguments    : const double varargin_1[4]
//                const double varargin_2_data[]
//                const int varargin_2_size[2]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
static void b_blkdiag(const double varargin_1[4], const double varargin_2_data[],
                      const int varargin_2_size[2], double y_data[], int y_size
                      [2])
{
  int loop_ub;
  int i7;
  int i8;
  int b_loop_ub;
  y_size[0] = (signed char)(2 + varargin_2_size[0]);
  y_size[1] = (signed char)(2 + varargin_2_size[1]);
  loop_ub = (signed char)(2 + varargin_2_size[0]) * (signed char)(2 +
    varargin_2_size[1]);
  for (i7 = 0; i7 < loop_ub; i7++) {
    y_data[i7] = 0.0;
  }

  for (i7 = 0; i7 < 2; i7++) {
    for (i8 = 0; i8 < 2; i8++) {
      y_data[i8 + y_size[0] * i7] = varargin_1[i8 + (i7 << 1)];
    }
  }

  if ((varargin_2_size[0] > 0) && (varargin_2_size[1] > 0)) {
    loop_ub = varargin_2_size[1];
    for (i7 = 0; i7 < loop_ub; i7++) {
      b_loop_ub = varargin_2_size[0];
      for (i8 = 0; i8 < b_loop_ub; i8++) {
        y_data[(i8 + y_size[0] * (2 + i7)) + 2] = varargin_2_data[i8 +
          varargin_2_size[0] * i7];
      }
    }
  }
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
// linearly interpolate between previous and current measurement to get
// to the corresponding part inside dt (for Runge-Kutta)
// za = prev_za + (za - prev_za) * part ;
// Arguments    : double dt
//                const double meas[6]
//                const emxArray_real_T *x
//                const double P_xx_apr[144]
//                const emxArray_real_T *Phi
//                const double b_Q[81]
//                emxArray_real_T *x_dot
//                double P_xx_apr_dot[144]
//                emxArray_real_T *Phi_dot
// Return Type  : void
//
static void b_dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T
  *x, const double P_xx_apr[144], const emxArray_real_T *Phi, const double b_Q
  [81], emxArray_real_T *x_dot, double P_xx_apr_dot[144], emxArray_real_T
  *Phi_dot)
{
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double R_cw[9];
  double w[3];
  int br;
  double dv24[9];
  double b_R_cw[9];
  int i;
  double dv25[9];
  double c_R_cw[9];
  int cr;
  static const signed char iv5[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double F[144];
  static const signed char iv6[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double G[108];
  static const signed char iv7[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv8[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double grav[3];
  static const double dv26[3] = { 0.0, 0.0, 9.81 };

  static const double dv27[3] = { 0.0, 0.0, -9.81 };

  unsigned int unnamed_idx_0;
  double dv28[9];
  double dv29[16];
  double dv30[16];
  double b_x[4];
  double dv31[4];
  double d_R_cw[3];
  double b_G[108];
  double FP[144];
  double b_FP[144];
  int ic;
  int ar;
  int ib;
  int ia;

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  a = x->data[3];
  b_a = x->data[4];
  c_a = x->data[5];
  d_a = x->data[6];
  e_a = x->data[3];
  f_a = x->data[4];
  g_a = x->data[5];
  h_a = x->data[6];
  i_a = x->data[3];
  j_a = x->data[4];
  k_a = x->data[5];
  l_a = x->data[6];
  R_cw[0] = ((a * a - b_a * b_a) - c_a * c_a) + d_a * d_a;
  R_cw[3] = 2.0 * (x->data[3] * x->data[4] + x->data[5] * x->data[6]);
  R_cw[6] = 2.0 * (x->data[3] * x->data[5] - x->data[4] * x->data[6]);
  R_cw[1] = 2.0 * (x->data[3] * x->data[4] - x->data[5] * x->data[6]);
  R_cw[4] = ((-(e_a * e_a) + f_a * f_a) - g_a * g_a) + h_a * h_a;
  R_cw[7] = 2.0 * (x->data[4] * x->data[5] + x->data[3] * x->data[6]);
  R_cw[2] = 2.0 * (x->data[3] * x->data[5] + x->data[4] * x->data[6]);
  R_cw[5] = 2.0 * (x->data[4] * x->data[5] - x->data[3] * x->data[6]);
  R_cw[8] = ((-(i_a * i_a) - j_a * j_a) + k_a * k_a) + l_a * l_a;

  // oa=x(14:16)
  for (br = 0; br < 3; br++) {
    w[br] = meas[br] - x->data[10 + br];
  }

  dv24[0] = 0.0;
  dv24[3] = -w[2];
  dv24[6] = w[1];
  dv24[1] = w[2];
  dv24[4] = 0.0;
  dv24[7] = -w[0];
  dv24[2] = -w[1];
  dv24[5] = w[0];
  dv24[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      b_R_cw[i + 3 * br] = -R_cw[br + 3 * i];
    }
  }

  dv25[0] = 0.0;
  dv25[3] = -meas[5];
  dv25[6] = meas[4];
  dv25[1] = meas[5];
  dv25[4] = 0.0;
  dv25[7] = -meas[3];
  dv25[2] = -meas[4];
  dv25[5] = meas[3];
  dv25[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      c_R_cw[br + 3 * i] = 0.0;
      for (cr = 0; cr < 3; cr++) {
        c_R_cw[br + 3 * i] += b_R_cw[br + 3 * cr] * dv25[cr + 3 * i];
      }
    }
  }

  for (br = 0; br < 12; br++) {
    for (i = 0; i < 3; i++) {
      F[i + 12 * br] = iv5[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * br) + 3] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 3)) + 3] = -dv24[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 6)) + 3] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 9)) + 3] = iv6[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * br) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 3)) + 6] = c_R_cw[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 6)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * (br + 9)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 12; br++) {
    for (i = 0; i < 3; i++) {
      F[(i + 12 * br) + 9] = 0.0;
    }
  }

  for (br = 0; br < 9; br++) {
    for (i = 0; i < 3; i++) {
      G[i + 12 * br] = 0.0;
    }

    for (i = 0; i < 3; i++) {
      G[(i + 12 * br) + 3] = iv7[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      G[(i + 12 * br) + 6] = 0.0;
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      G[(i + 12 * (br + 3)) + 6] = -R_cw[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      G[(i + 12 * (br + 6)) + 6] = 0.0;
    }
  }

  for (br = 0; br < 9; br++) {
    for (i = 0; i < 3; i++) {
      G[(i + 12 * br) + 9] = iv8[i + 3 * br];
    }
  }

  if (normalGravity) {
    for (i = 0; i < 3; i++) {
      grav[i] = dv26[i];
    }
  } else {
    for (i = 0; i < 3; i++) {
      grav[i] = dv27[i];
    }
  }

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)x->size[0];
  br = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, br, (int)sizeof(double));
  i = (int)unnamed_idx_0;
  for (br = 0; br < i; br++) {
    x_dot->data[br] = 0.0;
  }

  for (br = 0; br < 3; br++) {
    x_dot->data[br] = x->data[7 + br];
  }

  //  position
  dv28[0] = 0.0;
  dv28[3] = -w[2];
  dv28[6] = w[1];
  dv28[1] = w[2];
  dv28[4] = 0.0;
  dv28[7] = -w[0];
  dv28[2] = -w[1];
  dv28[5] = w[0];
  dv28[8] = 0.0;
  for (br = 0; br < 3; br++) {
    for (i = 0; i < 3; i++) {
      dv29[i + (br << 2)] = -dv28[i + 3 * br];
    }
  }

  for (br = 0; br < 3; br++) {
    dv29[12 + br] = w[br];
  }

  for (br = 0; br < 3; br++) {
    dv29[3 + (br << 2)] = -w[br];
  }

  dv29[15] = 0.0;
  for (br = 0; br < 4; br++) {
    for (i = 0; i < 4; i++) {
      dv30[i + (br << 2)] = 0.5 * dv29[i + (br << 2)];
    }
  }

  for (br = 0; br < 4; br++) {
    b_x[br] = x->data[3 + br];
  }

  for (br = 0; br < 4; br++) {
    dv31[br] = 0.0;
    for (i = 0; i < 4; i++) {
      dv31[br] += dv30[br + (i << 2)] * b_x[i];
    }
  }

  for (br = 0; br < 4; br++) {
    x_dot->data[3 + br] = dv31[br];
  }

  //  rot angle
  for (br = 0; br < 3; br++) {
    a = 0.0;
    for (i = 0; i < 3; i++) {
      a += R_cw[i + 3 * br] * meas[3 + i];
    }

    d_R_cw[br] = a - grav[br];
  }

  for (br = 0; br < 3; br++) {
    x_dot->data[7 + br] = d_R_cw[br];
  }

  for (br = 0; br < 3; br++) {
    x_dot->data[10 + br] = 0.0;
  }

  //  gyro bias
  br = x_dot->size[0];
  emxEnsureCapacity((emxArray__common *)x_dot, br, (int)sizeof(double));
  i = x_dot->size[0];
  for (br = 0; br < i; br++) {
    x_dot->data[br] *= dt;
  }

  // time derivative of the covariance
  for (br = 0; br < 12; br++) {
    for (i = 0; i < 12; i++) {
      FP[br + 12 * i] = 0.0;
      for (cr = 0; cr < 12; cr++) {
        FP[br + 12 * i] += F[br + 12 * cr] * P_xx_apr[cr + 12 * i];
      }
    }

    for (i = 0; i < 9; i++) {
      b_G[br + 12 * i] = 0.0;
      for (cr = 0; cr < 9; cr++) {
        b_G[br + 12 * i] += G[br + 12 * cr] * b_Q[cr + 9 * i];
      }
    }
  }

  for (br = 0; br < 12; br++) {
    for (i = 0; i < 12; i++) {
      a = 0.0;
      for (cr = 0; cr < 9; cr++) {
        a += b_G[br + 12 * cr] * G[i + 12 * cr];
      }

      b_FP[br + 12 * i] = (FP[br + 12 * i] + FP[i + 12 * br]) + a;
    }
  }

  for (br = 0; br < 12; br++) {
    for (i = 0; i < 12; i++) {
      P_xx_apr_dot[i + 12 * br] = b_FP[i + 12 * br] * dt;
    }
  }

  // time derivative of the state transition
  unnamed_idx_0 = (unsigned int)Phi->size[1];
  br = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[0] = 12;
  emxEnsureCapacity((emxArray__common *)Phi_dot, br, (int)sizeof(double));
  br = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[1] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)Phi_dot, br, (int)sizeof(double));
  i = 12 * (int)unnamed_idx_0;
  for (br = 0; br < i; br++) {
    Phi_dot->data[br] = 0.0;
  }

  if (Phi->size[1] == 0) {
  } else {
    i = 12 * (Phi->size[1] - 1);
    for (cr = 0; cr <= i; cr += 12) {
      for (ic = cr + 1; ic <= cr + 12; ic++) {
        Phi_dot->data[ic - 1] = 0.0;
      }
    }

    br = 0;
    for (cr = 0; cr <= i; cr += 12) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 12; ib++) {
        if (Phi->data[ib] != 0.0) {
          ia = ar;
          for (ic = cr; ic + 1 <= cr + 12; ic++) {
            ia++;
            Phi_dot->data[ic] += Phi->data[ib] * F[ia];
          }
        }

        ar += 12;
      }

      br += 12;
    }
  }

  br = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[0] = 12;
  emxEnsureCapacity((emxArray__common *)Phi_dot, br, (int)sizeof(double));
  i = Phi_dot->size[0];
  cr = Phi_dot->size[1];
  i *= cr;
  for (br = 0; br < i; br++) {
    Phi_dot->data[br] *= dt;
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
//                double a
//                const double x[30]
//                int ix0
//                double y[6]
//                int iy0
// Return Type  : void
//
static void b_eml_xaxpy(int n, double a, const double x[30], int ix0, double y[6],
  int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : double A[30]
//                double tau[5]
//                int jpvt[5]
// Return Type  : void
//
static void b_eml_xgeqp3(double A[30], double tau[5], int jpvt[5])
{
  double work[5];
  int i26;
  double vn1[5];
  double vn2[5];
  int k;
  int iy;
  double smax;
  double temp2;
  int itemp;
  double absxk;
  double t;
  int i;
  int i_i;
  int ix;
  int pvt;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int32_T exitg1;
  for (i26 = 0; i26 < 5; i26++) {
    jpvt[i26] = 1 + i26;
    work[i26] = 0.0;
  }

  k = 1;
  for (iy = 0; iy < 5; iy++) {
    smax = 0.0;
    temp2 = 2.2250738585072014E-308;
    for (itemp = k; itemp <= k + 5; itemp++) {
      absxk = fabs(A[itemp - 1]);
      if (absxk > temp2) {
        t = temp2 / absxk;
        smax = 1.0 + smax * t * t;
        temp2 = absxk;
      } else {
        t = absxk / temp2;
        smax += t * t;
      }
    }

    smax = temp2 * sqrt(smax);
    vn1[iy] = smax;
    vn2[iy] = vn1[iy];
    k += 6;
  }

  for (i = 0; i < 5; i++) {
    i_i = i + i * 6;
    itemp = 0;
    if (5 - i > 1) {
      ix = i;
      smax = fabs(vn1[i]);
      for (k = 1; k + 1 <= 5 - i; k++) {
        ix++;
        temp2 = fabs(vn1[ix]);
        if (temp2 > smax) {
          itemp = k;
          smax = temp2;
        }
      }
    }

    pvt = i + itemp;
    if (pvt + 1 != i + 1) {
      ix = 6 * pvt;
      iy = 6 * i;
      for (k = 0; k < 6; k++) {
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
        ix++;
        iy++;
      }

      itemp = jpvt[pvt];
      jpvt[pvt] = jpvt[i];
      jpvt[i] = itemp;
      vn1[pvt] = vn1[i];
      vn2[pvt] = vn2[i];
    }

    absxk = A[i_i];
    temp2 = 0.0;
    smax = f_eml_xnrm2(5 - i, A, i_i + 2);
    if (smax != 0.0) {
      smax = rt_hypotd_snf(A[i_i], smax);
      if (A[i_i] >= 0.0) {
        smax = -smax;
      }

      if (fabs(smax) < 1.0020841800044864E-292) {
        itemp = 0;
        do {
          itemp++;
          i26 = i_i - i;
          for (k = i_i + 1; k + 1 <= i26 + 6; k++) {
            A[k] *= 9.9792015476736E+291;
          }

          smax *= 9.9792015476736E+291;
          absxk *= 9.9792015476736E+291;
        } while (!(fabs(smax) >= 1.0020841800044864E-292));

        smax = rt_hypotd_snf(absxk, f_eml_xnrm2(5 - i, A, i_i + 2));
        if (absxk >= 0.0) {
          smax = -smax;
        }

        temp2 = (smax - absxk) / smax;
        absxk = 1.0 / (absxk - smax);
        i26 = i_i - i;
        for (k = i_i + 1; k + 1 <= i26 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 1.0020841800044864E-292;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0 / (A[i_i] - smax);
        i26 = i_i - i;
        for (k = i_i + 1; k + 1 <= i26 + 6; k++) {
          A[k] *= absxk;
        }

        absxk = smax;
      }
    }

    tau[i] = temp2;
    A[i_i] = absxk;
    if (i + 1 < 5) {
      absxk = A[i_i];
      A[i_i] = 1.0;
      i_ip1 = (i + (i + 1) * 6) + 1;
      if (tau[i] != 0.0) {
        lastv = 6 - i;
        itemp = i_i - i;
        while ((lastv > 0) && (A[itemp + 5] == 0.0)) {
          lastv--;
          itemp--;
        }

        lastc = 4 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          itemp = i_ip1 + (lastc - 1) * 6;
          k = itemp;
          do {
            exitg1 = 0;
            if (k <= (itemp + lastv) - 1) {
              if (A[k - 1] != 0.0) {
                exitg1 = 1;
              } else {
                k++;
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
          for (iy = 1; iy <= lastc; iy++) {
            work[iy - 1] = 0.0;
          }

          iy = 0;
          i26 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i26; itemp += 6) {
            ix = i_i;
            smax = 0.0;
            pvt = (itemp + lastv) - 1;
            for (k = itemp; k <= pvt; k++) {
              smax += A[k - 1] * A[ix];
              ix++;
            }

            work[iy] += smax;
            iy++;
          }
        }

        if (-tau[i] == 0.0) {
        } else {
          itemp = i_ip1 - 1;
          pvt = 0;
          for (iy = 1; iy <= lastc; iy++) {
            if (work[pvt] != 0.0) {
              smax = work[pvt] * -tau[i];
              ix = i_i;
              i26 = lastv + itemp;
              for (k = itemp; k + 1 <= i26; k++) {
                A[k] += A[ix] * smax;
                ix++;
              }
            }

            pvt++;
            itemp += 6;
          }
        }
      }

      A[i_i] = absxk;
    }

    for (iy = i + 1; iy + 1 < 6; iy++) {
      if (vn1[iy] != 0.0) {
        smax = fabs(A[i + 6 * iy]) / vn1[iy];
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        temp2 = vn1[iy] / vn2[iy];
        temp2 = smax * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          vn1[iy] = d_eml_xnrm2(5 - i, A, (i + 6 * iy) + 2);
          vn2[iy] = vn1[iy];
        } else {
          vn1[iy] *= sqrt(smax);
        }
      }
    }
  }
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
  if ((n == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
  } else {
    for (j = n; j > 0; j--) {
      jBcol = ldb * (j - 1) - 1;
      jAcol = lda * (j - 1) - 1;
      for (k = j + 1; k <= n; k++) {
        kBcol = ldb * (k - 1);
        if (A_data[k + jAcol] != 0.0) {
          for (i = 1; i <= m; i++) {
            B->data[i + jBcol] -= A_data[k + jAcol] * B->data[(i + kBcol) - 1];
          }
        }
      }
    }
  }
}

//
// Arguments    : emxArray_int32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void b_emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions)
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
// Arguments    : const double varargin_1[2]
//                emxArray_real_T *I
// Return Type  : void
//
static void b_eye(const double varargin_1[2], emxArray_real_T *I)
{
  double minval;
  int k;
  int loop_ub;
  if ((varargin_1[0] <= varargin_1[1]) || rtIsNaN(varargin_1[1])) {
    minval = varargin_1[0];
  } else {
    minval = varargin_1[1];
  }

  k = I->size[0] * I->size[1];
  I->size[0] = (int)varargin_1[0];
  emxEnsureCapacity((emxArray__common *)I, k, (int)sizeof(double));
  k = I->size[0] * I->size[1];
  I->size[1] = (int)varargin_1[1];
  emxEnsureCapacity((emxArray__common *)I, k, (int)sizeof(double));
  loop_ub = (int)varargin_1[0] * (int)varargin_1[1];
  for (k = 0; k < loop_ub; k++) {
    I->data[k] = 0.0;
  }

  if ((int)minval > 0) {
    for (k = 0; k + 1 <= (int)minval; k++) {
      I->data[k + I->size[0] * k] = 1.0;
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
static void b_fprintf()
{
  c_fprintf();
}

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
//    Does not take into account the derivative with respect to anchor states
//    (static map)
//
//  INPUT ARGUMENTS:
//  - xt:                   The current state
//  - errorStateSize:       The size of the error state
//  - z_all_l:              The feature points in the left camera (2N x 1)
//  - map:                  Map of the estimated feature points (3 x N)
//  - indMeas:              The indices of the valid feature points
//  - cameraparams:         A struct with the fields
//     - focal length, center of projection, radial distortion parameters of
//     both cameras
//     - r_lr:              Translation of right camera in left camera frame
//     - R_lr:              Orientation of right camera in left camera frame
//
//  where N is number of points in the image
//
//  OUTPUT ARGUMENTS:
//  - r:    The residual (residualDim*M x 1)
//  - H_xc: The Jacobian of the measurement function with respect to the camera states (residualDim*M x length(xt))
//
//  where M =  length(indMeas) is the number of valid points in the image
// Arguments    : const emxArray_real_T *b_xt
//                double errorStateSize
//                double stateSize
//                const double z_all_l[32]
//                const double indMeas_data[]
//                const int indMeas_size[1]
//                const emxArray_real_T *map
//                double numAnchors
//                double numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[23]
//                double height_offset_pressure
//                double r_data[]
//                int r_size[1]
//                emxArray_real_T *H
//                double h_u_data[]
//                int h_u_size[1]
//                double R_data[]
//                int R_size[2]
// Return Type  : void
//
static void b_getH_R_res(const emxArray_real_T *b_xt, double errorStateSize,
  double stateSize, const double z_all_l[32], const double indMeas_data[], const
  int indMeas_size[1], const emxArray_real_T *map, double numAnchors, double
  numPointsPerAnchor, const emxArray_real_T *anchorIdx, const emxArray_real_T
  *featureAnchorIdx, const emxArray_real_T *b_m_vect, const double imNoise[2],
  const double IMU_measurements[23], double height_offset_pressure, double
  r_data[], int r_size[1], emxArray_real_T *H, double h_u_data[], int h_u_size[1],
  double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ic;
  int ib;
  emxArray_real_T *H_xc;
  int z_size_idx_0;
  double z_data[32];
  int k;
  emxArray_real_T *H_iy;
  emxArray_int32_T *r2;
  emxArray_real_T *C;
  emxArray_real_T *y;
  double b_map[3];
  double h_rz[3];
  int ia;
  double h_cin_l[3];
  int nm1d2;
  double dv5[2];
  int r;
  double indMeas;
  signed char b_k;
  signed char iv0[2];
  double dv6[4];
  double h_un_To_h_d_l[4];
  double dv7[6];
  double b_h_un_To_h_d_l[6];
  double b_R_cw[36];
  static const double c_h_un_To_h_d_l[4] = { 268.155648020127, 0.0, 0.0,
    268.867732741683 };

  double d_h_un_To_h_d_l[24];
  double b_stateSize;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double c_stateSize;
  double d_stateSize;
  double e_stateSize;
  double f_stateSize;
  double g_stateSize;
  double h_stateSize;
  double i_stateSize;
  double j_stateSize;
  double k_stateSize;
  double l_stateSize;
  double m_stateSize;
  double n_stateSize;
  double o_stateSize;
  double p_stateSize;
  double q_stateSize;
  double r_stateSize;
  double s_stateSize;
  double t_stateSize;
  double u_stateSize;
  double v_stateSize;
  double w_stateSize;
  double x_stateSize;
  double y_stateSize;
  double anchorRot[9];
  double c_xt;
  double d2;
  double b_y[3];
  double c;
  double dv8[9];
  double b_anchorRot[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  int ar;
  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  double b_anchorIdx;
  double dv9[6];
  double c_y[6];
  int b_c;
  double b_z_data;
  emxArray_real_T *H_v;
  emxArray_real_T *r3;
  double dv10[2];
  double dv11[4];
  int R_v_size[2];
  double R_v_data[1024];
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  int R_g_size[2];
  int R_p_size[2];
  static const signed char b_b[3] = { 0, 0, 1 };

  double dv12[9];
  double R_g_data[9];
  double r_g_data[3];
  static const double dv13[3] = { 0.0934, -0.9946, 0.0459 };

  double R_p_data[1];
  double r_p_data[1];
  emxArray_real_T b_R_v_data;
  emxArray_real_T b_R_p_data;
  emxInit_real_T(&H_xm, 2);

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4]) -
             b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] * b_xt->data[6]);
  R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] * b_xt->data[6]);
  R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] * b_xt->data[6]);
  R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data[4])
             - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] * b_xt->data[6]);
  R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] * b_xt->data[6]);
  R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] * b_xt->data[6]);
  R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data[4])
             + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];

  //  camera parameters for the left and right camera
  //  Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1);
  //  Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2);
  ic = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xm, ic, (int)sizeof(double));
  ic = H_xm->size[0] * H_xm->size[1];
  H_xm->size[1] = (int)(numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ic, (int)sizeof(double));
  ib = (indMeas_size[0] << 1) * (int)(numAnchors * (6.0 + numPointsPerAnchor));
  for (ic = 0; ic < ib; ic++) {
    H_xm->data[ic] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ic = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = indMeas_size[0] << 1;
  emxEnsureCapacity((emxArray__common *)H_xc, ic, (int)sizeof(double));
  ic = H_xc->size[0] * H_xc->size[1];
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ic, (int)sizeof(double));
  ib = (indMeas_size[0] << 1) * (int)errorStateSize;
  for (ic = 0; ic < ib; ic++) {
    H_xc->data[ic] = 0.0;
  }

  z_size_idx_0 = indMeas_size[0] << 1;
  ib = indMeas_size[0] << 1;
  for (ic = 0; ic < ib; ic++) {
    z_data[ic] = 0.0;
  }

  h_u_size[0] = indMeas_size[0] << 1;
  ib = indMeas_size[0] << 1;
  for (ic = 0; ic < ib; ic++) {
    h_u_data[ic] = 0.0;
  }

  k = 0;
  emxInit_real_T(&H_iy, 2);
  emxInit_int32_T(&r2, 1);
  emxInit_real_T(&C, 2);
  emxInit_real_T(&y, 2);
  while (k <= indMeas_size[0] - 1) {
    for (ic = 0; ic < 3; ic++) {
      b_map[ic] = map->data[ic + map->size[0] * ((int)indMeas_data[k] - 1)] -
        b_xt->data[ic];
    }

    for (ic = 0; ic < 3; ic++) {
      h_rz[ic] = 0.0;
      for (ia = 0; ia < 3; ia++) {
        h_rz[ic] += R_cw[ic + 3 * ia] * b_map[ia];
      }
    }

    for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
      h_cin_l[nm1d2] = h_rz[nm1d2] / h_rz[2];
    }

    predictMeasurement_left(h_rz, dv5);
    r = k << 1;
    for (ic = 0; ic < 2; ic++) {
      h_u_data[ic + r] = dv5[ic];
    }

    r = k << 1;
    indMeas = (indMeas_data[k] - 1.0) * 2.0;
    for (ic = 0; ic < 2; ic++) {
      z_data[ic + r] = z_all_l[(int)(indMeas + (1.0 + (double)ic)) - 1];
    }

    //     %% computation of H(x)
    b_k = (signed char)((signed char)k << 1);
    for (ic = 0; ic < 2; ic++) {
      iv0[ic] = (signed char)(ic + b_k);
    }

    ib = H_xc->size[1];
    ic = r2->size[0];
    r2->size[0] = ib;
    emxEnsureCapacity((emxArray__common *)r2, ic, (int)sizeof(int));
    for (ic = 0; ic < ib; ic++) {
      r2->data[ic] = ic;
    }

    Ch_dn_To_h_un(h_cin_l[0], h_cin_l[1], dv6);
    dv7[0] = 1.0 / h_rz[2];
    dv7[2] = 0.0;
    dv7[4] = -h_rz[0] / (h_rz[2] * h_rz[2]);
    dv7[1] = 0.0;
    dv7[3] = 1.0 / h_rz[2];
    dv7[5] = -h_rz[1] / (h_rz[2] * h_rz[2]);
    for (ic = 0; ic < 2; ic++) {
      for (ia = 0; ia < 2; ia++) {
        h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          h_un_To_h_d_l[ic + (ia << 1)] += c_h_un_To_h_d_l[ic + (r << 1)] *
            dv6[r + (ia << 1)];
        }
      }

      for (ia = 0; ia < 3; ia++) {
        b_h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          b_h_un_To_h_d_l[ic + (ia << 1)] += h_un_To_h_d_l[ic + (r << 1)] *
            dv7[r + (ia << 1)];
        }
      }
    }

    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        b_R_cw[ia + 3 * ic] = -R_cw[ia + 3 * ic];
      }
    }

    b_R_cw[9] = 0.0;
    b_R_cw[12] = -h_rz[2];
    b_R_cw[15] = h_rz[1];
    b_R_cw[10] = h_rz[2];
    b_R_cw[13] = 0.0;
    b_R_cw[16] = -h_rz[0];
    b_R_cw[11] = -h_rz[1];
    b_R_cw[14] = h_rz[0];
    b_R_cw[17] = 0.0;
    for (ic = 0; ic < 6; ic++) {
      for (ia = 0; ia < 3; ia++) {
        b_R_cw[ia + 3 * (ic + 6)] = 0.0;
      }
    }

    for (ic = 0; ic < 2; ic++) {
      for (ia = 0; ia < 12; ia++) {
        d_h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 3; r++) {
          d_h_un_To_h_d_l[ic + (ia << 1)] += b_h_un_To_h_d_l[ic + (r << 1)] *
            b_R_cw[r + 3 * ia];
        }
      }
    }

    nm1d2 = r2->size[0];
    for (ic = 0; ic < nm1d2; ic++) {
      for (ia = 0; ia < 2; ia++) {
        H_xc->data[iv0[ia] + H_xc->size[0] * r2->data[ic]] = d_h_un_To_h_d_l[ia
          + (ic << 1)];
      }
    }

    //     %% anchor state derivatives
    //  fp = anchorPos + anchorRot'*m/rho - r_wc_pred;
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    b_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    c_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    d_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    e_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    f_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    g_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    h_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    i_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    j_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    k_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    l_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
    b_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    c_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    d_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    e_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    f_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    g_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    h_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    i_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    j_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    k_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    l_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    m_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    n_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    o_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    p_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    q_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    r_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    s_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    t_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    u_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    v_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    w_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    x_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    y_stateSize = stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) *
      (7.0 + numPointsPerAnchor);
    anchorRot[0] = ((a * a - b_a * b_a) - c_a * c_a) + d_a * d_a;
    anchorRot[3] = 2.0 * (b_xt->data[(int)(b_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(c_stateSize + 5.0) - 1] + b_xt->data[(int)
                          (d_stateSize + 6.0) - 1] * b_xt->data[(int)
                          (e_stateSize + 7.0) - 1]);
    anchorRot[6] = 2.0 * (b_xt->data[(int)(f_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(g_stateSize + 6.0) - 1] - b_xt->data[(int)
                          (h_stateSize + 5.0) - 1] * b_xt->data[(int)
                          (i_stateSize + 7.0) - 1]);
    anchorRot[1] = 2.0 * (b_xt->data[(int)(j_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(k_stateSize + 5.0) - 1] - b_xt->data[(int)
                          (l_stateSize + 6.0) - 1] * b_xt->data[(int)
                          (m_stateSize + 7.0) - 1]);
    anchorRot[4] = ((-(e_a * e_a) + f_a * f_a) - g_a * g_a) + h_a * h_a;
    anchorRot[7] = 2.0 * (b_xt->data[(int)(n_stateSize + 5.0) - 1] * b_xt->data
                          [(int)(o_stateSize + 6.0) - 1] + b_xt->data[(int)
                          (p_stateSize + 4.0) - 1] * b_xt->data[(int)
                          (q_stateSize + 7.0) - 1]);
    anchorRot[2] = 2.0 * (b_xt->data[(int)(r_stateSize + 4.0) - 1] * b_xt->data
                          [(int)(s_stateSize + 6.0) - 1] + b_xt->data[(int)
                          (t_stateSize + 5.0) - 1] * b_xt->data[(int)
                          (u_stateSize + 7.0) - 1]);
    anchorRot[5] = 2.0 * (b_xt->data[(int)(v_stateSize + 5.0) - 1] * b_xt->data
                          [(int)(w_stateSize + 6.0) - 1] - b_xt->data[(int)
                          (x_stateSize + 4.0) - 1] * b_xt->data[(int)
                          (y_stateSize + 7.0) - 1]);
    anchorRot[8] = ((-(i_a * i_a) - j_a * j_a) + k_a * k_a) + l_a * l_a;
    c_xt = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k]
      - 1] - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data
      [(int)indMeas_data[k] - 1]) - 1];
    for (ic = 0; ic < 3; ic++) {
      d2 = 0.0;
      for (ia = 0; ia < 3; ia++) {
        d2 += anchorRot[ia + 3 * ic] * b_m_vect->data[ia + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_y[ic] = d2 / c_xt;
    }

    c = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1]
      - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)
                         indMeas_data[k] - 1]) - 1] * b_xt->data[(int)
      (((stateSize + (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (7.0 +
          numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)
       indMeas_data[k] - 1]) - 1];
    dv8[0] = 0.0;
    dv8[3] = -b_y[2];
    dv8[6] = b_y[1];
    dv8[1] = b_y[2];
    dv8[4] = 0.0;
    dv8[7] = -b_y[0];
    dv8[2] = -b_y[1];
    dv8[5] = b_y[0];
    dv8[8] = 0.0;
    nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas_data[k] - 1] - 1.0);
    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        b_anchorRot[ia + 3 * ic] = -anchorRot[ic + 3 * ia];
      }
    }

    for (ic = 0; ic < 3; ic++) {
      d2 = 0.0;
      for (ia = 0; ia < 3; ia++) {
        d2 += b_anchorRot[ic + 3 * ia] * b_m_vect->data[ia + b_m_vect->size[0] *
          ((int)indMeas_data[k] - 1)];
      }

      b_map[ic] = d2 / c;
    }

    r = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas_data[k] -
              1]);
    ic = H_iy->size[0] * H_iy->size[1];
    H_iy->size[0] = 3;
    H_iy->size[1] = (nm1d2 + r) + 7;
    emxEnsureCapacity((emxArray__common *)H_iy, ic, (int)sizeof(double));
    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_iy->data[ia + H_iy->size[0] * ic] = b[ia + 3 * ic];
      }
    }

    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_iy->data[ia + H_iy->size[0] * (ic + 3)] = -dv8[ia + 3 * ic];
      }
    }

    for (ic = 0; ic < nm1d2; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_iy->data[ia + H_iy->size[0] * (ic + 6)] = 0.0;
      }
    }

    for (ic = 0; ic < 3; ic++) {
      H_iy->data[ic + H_iy->size[0] * (6 + nm1d2)] = b_map[ic];
    }

    for (ic = 0; ic < r; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_iy->data[ia + H_iy->size[0] * ((ic + nm1d2) + 7)] = 0.0;
      }
    }

    b_k = (signed char)((signed char)k << 1);
    for (ic = 0; ic < 2; ic++) {
      iv0[ic] = (signed char)(ic + b_k);
    }

    if (rtIsNaN(6.0 + numPointsPerAnchor)) {
      ar = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (6.0 + numPointsPerAnchor < 1.0) {
      ar = -1;
      anew = 1.0;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (rtIsInf(6.0 + numPointsPerAnchor)) {
      ar = 0;
      anew = rtNaN;
      apnd = 6.0 + numPointsPerAnchor;
    } else {
      anew = 1.0;
      ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
      apnd = 1.0 + ndbl;
      cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
      absb = fabs(6.0 + numPointsPerAnchor);
      if ((1.0 >= absb) || rtIsNaN(absb)) {
        absb = 1.0;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = 6.0 + numPointsPerAnchor;
      } else if (cdiff > 0.0) {
        apnd = 1.0 + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        ar = (int)ndbl - 1;
      } else {
        ar = -1;
      }
    }

    ic = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = ar + 1;
    emxEnsureCapacity((emxArray__common *)y, ic, (int)sizeof(double));
    if (ar + 1 > 0) {
      y->data[0] = anew;
      if (ar + 1 > 1) {
        y->data[ar] = apnd;
        nm1d2 = (ar + (ar < 0)) >> 1;
        for (r = 1; r < nm1d2; r++) {
          y->data[r] = anew + (double)r;
          y->data[ar - r] = apnd - (double)r;
        }

        if (nm1d2 << 1 == ar) {
          y->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          y->data[nm1d2] = anew + (double)nm1d2;
          y->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    b_anchorIdx = (anchorIdx->data[(int)indMeas_data[k] - 1] - 1.0) * (6.0 +
      numPointsPerAnchor);
    ic = r2->size[0];
    r2->size[0] = y->size[1];
    emxEnsureCapacity((emxArray__common *)r2, ic, (int)sizeof(int));
    ib = y->size[1];
    for (ic = 0; ic < ib; ic++) {
      r2->data[ic] = (int)(b_anchorIdx + y->data[y->size[0] * ic]) - 1;
    }

    Ch_dn_To_h_un(h_cin_l[0], h_cin_l[1], dv6);
    dv9[0] = 1.0 / h_rz[2];
    dv9[2] = 0.0;
    dv9[4] = -h_rz[0] / (h_rz[2] * h_rz[2]);
    dv9[1] = 0.0;
    dv9[3] = 1.0 / h_rz[2];
    dv9[5] = -h_rz[1] / (h_rz[2] * h_rz[2]);
    for (ic = 0; ic < 2; ic++) {
      for (ia = 0; ia < 2; ia++) {
        h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          h_un_To_h_d_l[ic + (ia << 1)] += c_h_un_To_h_d_l[ic + (r << 1)] *
            dv6[r + (ia << 1)];
        }
      }

      for (ia = 0; ia < 3; ia++) {
        b_h_un_To_h_d_l[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 2; r++) {
          b_h_un_To_h_d_l[ic + (ia << 1)] += h_un_To_h_d_l[ic + (r << 1)] *
            dv9[r + (ia << 1)];
        }
      }

      for (ia = 0; ia < 3; ia++) {
        c_y[ic + (ia << 1)] = 0.0;
        for (r = 0; r < 3; r++) {
          c_y[ic + (ia << 1)] += b_h_un_To_h_d_l[ic + (r << 1)] * R_cw[r + 3 *
            ia];
        }
      }
    }

    nm1d2 = H_iy->size[1];
    ic = C->size[0] * C->size[1];
    C->size[0] = 2;
    emxEnsureCapacity((emxArray__common *)C, ic, (int)sizeof(double));
    ic = C->size[0] * C->size[1];
    C->size[1] = nm1d2;
    emxEnsureCapacity((emxArray__common *)C, ic, (int)sizeof(double));
    ib = nm1d2 << 1;
    for (ic = 0; ic < ib; ic++) {
      C->data[ic] = 0.0;
    }

    b_c = (H_iy->size[1] - 1) << 1;
    for (nm1d2 = 0; nm1d2 <= b_c; nm1d2 += 2) {
      for (ic = nm1d2 + 1; ic <= nm1d2 + 2; ic++) {
        C->data[ic - 1] = 0.0;
      }
    }

    r = 0;
    for (nm1d2 = 0; nm1d2 <= b_c; nm1d2 += 2) {
      ar = 0;
      for (ib = r; ib + 1 <= r + 3; ib++) {
        if (H_iy->data[ib] != 0.0) {
          ia = ar;
          for (ic = nm1d2; ic + 1 <= nm1d2 + 2; ic++) {
            ia++;
            C->data[ic] += H_iy->data[ib] * c_y[ia - 1];
          }
        }

        ar += 2;
      }

      r += 3;
    }

    ib = C->size[1];
    for (ic = 0; ic < ib; ic++) {
      for (ia = 0; ia < 2; ia++) {
        H_xm->data[iv0[ia] + H_xm->size[0] * r2->data[ic]] = C->data[ia +
          C->size[0] * ic];
      }
    }

    k++;
  }

  emxFree_real_T(&y);
  emxFree_real_T(&C);
  emxFree_int32_T(&r2);
  emxFree_real_T(&H_iy);
  for (ic = 0; ic < z_size_idx_0; ic++) {
    b_z_data = z_data[ic] - h_u_data[ic];
    z_data[ic] = b_z_data;
  }

  emxInit_real_T(&H_v, 2);
  emxInit_real_T(&r3, 2);

  //  residual with respect to camera measurements
  eye((double)indMeas_size[0], r3);
  power(imNoise, dv10);
  diag(dv10, dv11);
  kron(r3->data, r3->size, dv11, R_v_data, R_v_size);
  nm1d2 = 0;

  //  gravity residual
  r = 0;

  //  pressure residual
  ic = H_v->size[0] * H_v->size[1];
  H_v->size[0] = H_xc->size[0];
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, ic, (int)sizeof(double));
  ib = H_xc->size[1];
  for (ic = 0; ic < ib; ic++) {
    ar = H_xc->size[0];
    for (ia = 0; ia < ar; ia++) {
      H_v->data[ia + H_v->size[0] * ic] = H_xc->data[ia + H_xc->size[0] * ic];
    }
  }

  ib = H_xm->size[1];
  for (ic = 0; ic < ib; ic++) {
    ar = H_xm->size[0];
    for (ia = 0; ia < ar; ia++) {
      H_v->data[ia + H_v->size[0] * (ic + H_xc->size[1])] = H_xm->data[ia +
        H_xm->size[0] * ic];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  emxInit_real_T(&H_g, 2);
  emxInit_real_T(&H_p, 2);
  ic = H_g->size[0] * H_g->size[1];
  H_g->size[0] = 0;
  H_g->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_g, ic, (int)sizeof(double));

  //  jacobian for gravity residual
  ic = H_p->size[0] * H_p->size[1];
  H_p->size[0] = 0;
  H_p->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_p, ic, (int)sizeof(double));

  //  jacobian for pressure residual
  R_g_size[0] = 0;
  R_g_size[1] = 0;
  R_p_size[0] = 0;
  R_p_size[1] = 0;
  if (gravityUpdate) {
    //  normalize the acceleration measurement
    //  normalize the magnetometer measurement
    for (ic = 0; ic < 3; ic++) {
      h_rz[ic] = 0.0;
      for (ia = 0; ia < 3; ia++) {
        h_rz[ic] += R_cw[ic + 3 * ia] * (double)b_b[ia];
      }
    }

    //  the earth-z axis transformed into the body frame
    ic = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_g, ic, (int)sizeof(double));
    ib = 3 * (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ic = 0; ic < ib; ic++) {
      H_g->data[ic] = 0.0;
    }

    dv12[0] = 0.0;
    dv12[3] = -h_rz[2];
    dv12[6] = h_rz[1];
    dv12[1] = h_rz[2];
    dv12[4] = 0.0;
    dv12[7] = -h_rz[0];
    dv12[2] = -h_rz[1];
    dv12[5] = h_rz[0];
    dv12[8] = 0.0;
    for (ic = 0; ic < 3; ic++) {
      for (ia = 0; ia < 3; ia++) {
        H_g->data[ia + H_g->size[0] * (3 + ic)] = dv12[ia + 3 * ic];
      }
    }

    R_g_size[0] = 3;
    R_g_size[1] = 3;
    for (ic = 0; ic < 9; ic++) {
      R_g_data[ic] = gravAlignNoise * (double)b[ic];
    }

    nm1d2 = 3;
    for (ic = 0; ic < 3; ic++) {
      r_g_data[ic] = dv13[ic] - h_rz[ic];
    }
  }

  if (useAirPressure) {
    ic = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_p, ic, (int)sizeof(double));
    ib = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ic = 0; ic < ib; ic++) {
      H_p->data[ic] = 0.0;
    }

    H_p->data[2] = 1.0;
    R_p_size[0] = 1;
    R_p_size[1] = 1;
    R_p_data[0] = 2.0;
    r = 1;
    r_p_data[0] = (1.0 - rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
      145366.45 - height_offset_pressure;
  }

  if (gravityUpdate) {
    if (useAirPressure) {
      r_size[0] = (z_size_idx_0 + nm1d2) + r;
      for (ic = 0; ic < z_size_idx_0; ic++) {
        r_data[ic] = z_data[ic];
      }

      for (ic = 0; ic < nm1d2; ic++) {
        r_data[ic + z_size_idx_0] = r_g_data[ic];
      }

      ic = 0;
      while (ic <= r - 1) {
        r_data[z_size_idx_0 + nm1d2] = r_p_data[0];
        ic = 1;
      }

      ic = H->size[0] * H->size[1];
      H->size[0] = (H_v->size[0] + H_g->size[0]) + H_p->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ic, (int)sizeof(double));
      ib = H_v->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_v->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[ia + H->size[0] * ic] = H_v->data[ia + H_v->size[0] * ic];
        }
      }

      ib = H_g->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_g->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[(ia + H_v->size[0]) + H->size[0] * ic] = H_g->data[ia +
            H_g->size[0] * ic];
        }
      }

      ib = H_p->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_p->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[((ia + H_v->size[0]) + H_g->size[0]) + H->size[0] * ic] =
            H_p->data[ia + H_p->size[0] * ic];
        }
      }

      output_size(R_v_size, R_g_size, R_p_size, &nm1d2, &r);
      R_size[0] = nm1d2;
      R_size[1] = r;
      ib = nm1d2 * r;
      for (ic = 0; ic < ib; ic++) {
        R_data[ic] = 0.0;
      }

      if ((R_v_size[0] > 0) && (R_v_size[1] > 0)) {
        ib = R_v_size[1];
        for (ic = 0; ic < ib; ic++) {
          ar = R_v_size[0];
          for (ia = 0; ia < ar; ia++) {
            R_data[ia + R_size[0] * ic] = R_v_data[ia + R_v_size[0] * ic];
          }
        }
      }

      if ((R_g_size[0] > 0) && (R_g_size[1] > 0)) {
        ic = R_v_size[0] + R_g_size[0];
        if (R_v_size[0] + 1 > ic) {
          ic = 1;
        } else {
          ic = R_v_size[0] + 1;
        }

        ia = R_v_size[1] + R_g_size[1];
        if (R_v_size[1] + 1 > ia) {
          ia = 1;
        } else {
          ia = R_v_size[1] + 1;
        }

        ib = R_g_size[1];
        for (r = 0; r < ib; r++) {
          ar = R_g_size[0];
          for (nm1d2 = 0; nm1d2 < ar; nm1d2++) {
            R_data[((ic + nm1d2) + R_size[0] * ((ia + r) - 1)) - 1] =
              R_g_data[nm1d2 + R_g_size[0] * r];
          }
        }
      }

      r = R_v_size[0] + R_g_size[0];
      b_c = R_v_size[1] + R_g_size[1];
      if ((R_p_size[0] > 0) && (R_p_size[1] > 0)) {
        ib = R_p_size[1];
        for (ic = 0; ic < ib; ic++) {
          ar = R_p_size[0];
          for (ia = 0; ia < ar; ia++) {
            R_data[(r + ia) + R_size[0] * (b_c + ic)] = R_p_data[ia + R_p_size[0]
              * ic];
          }
        }
      }
    } else {
      r_size[0] = z_size_idx_0 + nm1d2;
      for (ic = 0; ic < z_size_idx_0; ic++) {
        r_data[ic] = z_data[ic];
      }

      for (ic = 0; ic < nm1d2; ic++) {
        r_data[ic + z_size_idx_0] = r_g_data[ic];
      }

      ic = H->size[0] * H->size[1];
      H->size[0] = H_v->size[0] + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ic, (int)sizeof(double));
      ib = H_v->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_v->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[ia + H->size[0] * ic] = H_v->data[ia + H_v->size[0] * ic];
        }
      }

      ib = H_g->size[1];
      for (ic = 0; ic < ib; ic++) {
        ar = H_g->size[0];
        for (ia = 0; ia < ar; ia++) {
          H->data[(ia + H_v->size[0]) + H->size[0] * ic] = H_g->data[ia +
            H_g->size[0] * ic];
        }
      }

      b_R_v_data.data = (double *)&R_v_data;
      b_R_v_data.size = (int *)&R_v_size;
      b_R_v_data.allocatedSize = 1024;
      b_R_v_data.numDimensions = 2;
      b_R_v_data.canFreeData = false;
      b_R_p_data.data = (double *)&R_g_data;
      b_R_p_data.size = (int *)&R_g_size;
      b_R_p_data.allocatedSize = 9;
      b_R_p_data.numDimensions = 2;
      b_R_p_data.canFreeData = false;
      blkdiag(&b_R_v_data, &b_R_p_data, r3);
      R_size[0] = r3->size[0];
      R_size[1] = r3->size[1];
      ib = r3->size[0] * r3->size[1];
      for (ic = 0; ic < ib; ic++) {
        R_data[ic] = r3->data[ic];
      }
    }
  } else if (useAirPressure) {
    r_size[0] = z_size_idx_0 + r;
    for (ic = 0; ic < z_size_idx_0; ic++) {
      r_data[ic] = z_data[ic];
    }

    ic = 0;
    while (ic <= r - 1) {
      r_data[z_size_idx_0] = r_p_data[0];
      ic = 1;
    }

    ic = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0] + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ic, (int)sizeof(double));
    ib = H_v->size[1];
    for (ic = 0; ic < ib; ic++) {
      ar = H_v->size[0];
      for (ia = 0; ia < ar; ia++) {
        H->data[ia + H->size[0] * ic] = H_v->data[ia + H_v->size[0] * ic];
      }
    }

    ib = H_p->size[1];
    for (ic = 0; ic < ib; ic++) {
      ar = H_p->size[0];
      for (ia = 0; ia < ar; ia++) {
        H->data[(ia + H_v->size[0]) + H->size[0] * ic] = H_p->data[ia +
          H_p->size[0] * ic];
      }
    }

    b_R_v_data.data = (double *)&R_v_data;
    b_R_v_data.size = (int *)&R_v_size;
    b_R_v_data.allocatedSize = 1024;
    b_R_v_data.numDimensions = 2;
    b_R_v_data.canFreeData = false;
    b_R_p_data.data = (double *)&R_p_data;
    b_R_p_data.size = (int *)&R_p_size;
    b_R_p_data.allocatedSize = 1;
    b_R_p_data.numDimensions = 2;
    b_R_p_data.canFreeData = false;
    blkdiag(&b_R_v_data, &b_R_p_data, r3);
    R_size[0] = r3->size[0];
    R_size[1] = r3->size[1];
    ib = r3->size[0] * r3->size[1];
    for (ic = 0; ic < ib; ic++) {
      R_data[ic] = r3->data[ic];
    }
  } else {
    r_size[0] = z_size_idx_0;
    for (ic = 0; ic < z_size_idx_0; ic++) {
      r_data[ic] = z_data[ic];
    }

    ic = H->size[0] * H->size[1];
    H->size[0] = H_v->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ic, (int)sizeof(double));
    ib = H_v->size[0] * H_v->size[1];
    for (ic = 0; ic < ib; ic++) {
      H->data[ic] = H_v->data[ic];
    }

    R_size[0] = R_v_size[0];
    R_size[1] = R_v_size[1];
    ib = R_v_size[0] * R_v_size[1];
    for (ic = 0; ic < ib; ic++) {
      R_data[ic] = R_v_data[ic];
    }
  }

  emxFree_real_T(&r3);
  emxFree_real_T(&H_p);
  emxFree_real_T(&H_g);
  emxFree_real_T(&H_v);
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
// Arguments    : void
// Return Type  : double
//
static double b_rand()
{
  double r;
  int32_T exitg1;
  unsigned int u[2];
  int k;
  unsigned int b_r;
  int kk;
  unsigned int y;
  unsigned int b_y;
  unsigned int c_y;
  unsigned int d_y;
  boolean_T isvalid;
  boolean_T exitg2;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    exitg1 = 0;
    for (k = 0; k < 2; k++) {
      b_r = state[624] + 1U;
      if (b_r >= 625U) {
        for (kk = 0; kk < 227; kk++) {
          y = (state[kk] & 2147483648U) | (state[1 + kk] & 2147483647U);
          if ((int)(y & 1U) == 0) {
            b_y = y >> 1U;
          } else {
            b_y = y >> 1U ^ 2567483615U;
          }

          state[kk] = state[397 + kk] ^ b_y;
        }

        for (kk = 0; kk < 396; kk++) {
          y = (state[kk + 227] & 2147483648U) | (state[228 + kk] & 2147483647U);
          if ((int)(y & 1U) == 0) {
            c_y = y >> 1U;
          } else {
            c_y = y >> 1U ^ 2567483615U;
          }

          state[kk + 227] = state[kk] ^ c_y;
        }

        y = (state[623] & 2147483648U) | (state[0] & 2147483647U);
        if ((int)(y & 1U) == 0) {
          d_y = y >> 1U;
        } else {
          d_y = y >> 1U ^ 2567483615U;
        }

        state[623] = state[396] ^ d_y;
        b_r = 1U;
      }

      y = state[(int)b_r - 1];
      state[624] = b_r;
      y ^= y >> 11U;
      y ^= y << 7U & 2636928640U;
      y ^= y << 15U & 4022730752U;
      y ^= y >> 18U;
      u[k] = y;
    }

    r = 1.1102230246251565E-16 * ((double)(u[0] >> 5U) * 6.7108864E+7 + (double)
      (u[1] >> 6U));
    if (r == 0.0) {
      if ((state[624] >= 1U) && (state[624] < 625U)) {
        isvalid = true;
      } else {
        isvalid = false;
      }

      if (isvalid) {
        isvalid = false;
        k = 1;
        exitg2 = false;
        while ((!exitg2) && (k < 625)) {
          if (state[k - 1] == 0U) {
            k++;
          } else {
            isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!isvalid) {
        b_r = 5489U;
        state[0] = 5489U;
        for (k = 0; k < 623; k++) {
          b_r = (b_r ^ b_r >> 30U) * 1812433253U + (1 + k);
          state[k + 1] = b_r;
        }

        state[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

//
// Arguments    : const emxArray_real_T *varargin_1
//                const emxArray_real_T *varargin_2
//                emxArray_real_T *y
// Return Type  : void
//
static void blkdiag(const emxArray_real_T *varargin_1, const emxArray_real_T
                    *varargin_2, emxArray_real_T *y)
{
  int unnamed_idx_0;
  int unnamed_idx_1;
  int i1;
  int loop_ub;
  int i2;
  int i3;
  unnamed_idx_0 = varargin_1->size[0] + varargin_2->size[0];
  unnamed_idx_1 = varargin_1->size[1] + varargin_2->size[1];
  i1 = y->size[0] * y->size[1];
  y->size[0] = unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)y, i1, (int)sizeof(double));
  i1 = y->size[0] * y->size[1];
  y->size[1] = unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)y, i1, (int)sizeof(double));
  unnamed_idx_0 *= unnamed_idx_1;
  for (i1 = 0; i1 < unnamed_idx_0; i1++) {
    y->data[i1] = 0.0;
  }

  if ((varargin_1->size[0] > 0) && (varargin_1->size[1] > 0)) {
    unnamed_idx_0 = varargin_1->size[1];
    for (i1 = 0; i1 < unnamed_idx_0; i1++) {
      loop_ub = varargin_1->size[0];
      for (unnamed_idx_1 = 0; unnamed_idx_1 < loop_ub; unnamed_idx_1++) {
        y->data[unnamed_idx_1 + y->size[0] * i1] = varargin_1->
          data[unnamed_idx_1 + varargin_1->size[0] * i1];
      }
    }
  }

  if ((varargin_2->size[0] > 0) && (varargin_2->size[1] > 0)) {
    i1 = varargin_1->size[0] + varargin_2->size[0];
    if (varargin_1->size[0] + 1 > i1) {
      i1 = 1;
    } else {
      i1 = varargin_1->size[0] + 1;
    }

    unnamed_idx_1 = varargin_1->size[1] + varargin_2->size[1];
    if (varargin_1->size[1] + 1 > unnamed_idx_1) {
      unnamed_idx_1 = 1;
    } else {
      unnamed_idx_1 = varargin_1->size[1] + 1;
    }

    unnamed_idx_0 = varargin_2->size[1];
    for (i2 = 0; i2 < unnamed_idx_0; i2++) {
      loop_ub = varargin_2->size[0];
      for (i3 = 0; i3 < loop_ub; i3++) {
        y->data[((i1 + i3) + y->size[0] * ((unnamed_idx_1 + i2) - 1)) - 1] =
          varargin_2->data[i3 + varargin_2->size[0] * i2];
      }
    }
  }
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
static boolean_T c_any(const boolean_T x[3])
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
// Arguments    : void
// Return Type  : void
//
static void c_eml_rand_mt19937ar_stateful_i()
{
  unsigned int r;
  int mti;
  memset(&state[0], 0, 625U * sizeof(unsigned int));
  r = 5489U;
  state[0] = 5489U;
  for (mti = 0; mti < 623; mti++) {
    r = (r ^ r >> 30U) * 1812433253U + (1 + mti);
    state[mti + 1] = r;
  }

  state[624] = 624U;
}

//
// Arguments    : int n
//                double a
//                const double x[6]
//                int ix0
//                double y[30]
//                int iy0
// Return Type  : void
//
static void c_eml_xaxpy(int n, double a, const double x[6], int ix0, double y[30],
  int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
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
// Arguments    : void
// Return Type  : double
//
static double c_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[47] = { 'F', 'i', 'n', 'i', 's', 'h', 'e', 'd', ' ',
    'i', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'i', 'n', 'g', ' ', 'a', 't',
    't', 'i', 't', 'u', 'd', 'e', '.', ' ', 'S', 't', 'a', 'r', 't', 'i', 'n',
    'g', ' ', 'S', 'L', 'A', 'M', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double x[2]
// Return Type  : double
//
static double c_norm(const double x[2])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 2; k++) {
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
// Arguments    : const emxArray_boolean_T *x
// Return Type  : boolean_T
//
static boolean_T d_any(const emxArray_boolean_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b1;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[1])) {
    b1 = !x->data[ix - 1];
    if (!b1) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

//
// Arguments    : int n
//                const double x[30]
//                int ix0
// Return Type  : double
//
static double d_eml_xnrm2(int n, const double x[30], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x[k - 1]);
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
// Arguments    : signed char formatSpec
// Return Type  : void
//
static void d_fprintf(signed char formatSpec)
{
  e_fprintf(formatSpec);
}

//
// Arguments    : const emxArray_real_T *x
// Return Type  : double
//
static double d_norm(const emxArray_real_T *x)
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (x->size[0] < 1) {
  } else if (x->size[0] == 1) {
    y = fabs(x->data[0]);
  } else {
    scale = 2.2250738585072014E-308;
    for (k = 1; k <= x->size[0]; k++) {
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
// Arguments    : const double v[2]
//                double d[4]
// Return Type  : void
//
static void diag(const double v[2], double d[4])
{
  int j;
  for (j = 0; j < 4; j++) {
    d[j] = 0.0;
  }

  for (j = 0; j < 2; j++) {
    d[j + (j << 1)] = v[j];
  }
}

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_nzp_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (numerator >= 0) {
    absNumerator = (unsigned int)numerator;
  } else {
    absNumerator = (unsigned int)-numerator;
  }

  if (denominator >= 0) {
    absDenominator = (unsigned int)denominator;
  } else {
    absDenominator = (unsigned int)-denominator;
  }

  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
  }

  if (quotientNeedsNegation) {
    quotient = -(int)tempAbsQuotient;
  } else {
    quotient = (int)tempAbsQuotient;
  }

  return quotient;
}

//
// linearly interpolate between previous and current measurement to get
// to the corresponding part inside dt (for Runge-Kutta)
// za = prev_za + (za - prev_za) * part ;
// Arguments    : double dt
//                const double meas[6]
//                const emxArray_real_T *x
//                const emxArray_real_T *P_xx_apr
//                const emxArray_real_T *Phi
//                const double b_Q[81]
//                emxArray_real_T *x_dot
//                double P_xx_apr_dot[144]
//                emxArray_real_T *Phi_dot
// Return Type  : void
//
static void dxdt_dPdt(double dt, const double meas[6], const emxArray_real_T *x,
                      const emxArray_real_T *P_xx_apr, const emxArray_real_T
                      *Phi, const double b_Q[81], emxArray_real_T *x_dot, double
                      P_xx_apr_dot[144], emxArray_real_T *Phi_dot)
{
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double R_cw[9];
  double w[3];
  int i;
  double dv16[9];
  double b_R_cw[9];
  int br;
  double dv17[9];
  double c_R_cw[9];
  int ar;
  static const signed char iv1[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double F[144];
  static const signed char iv2[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double G[108];
  static const signed char iv3[27] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv4[27] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double grav[3];
  static const double dv18[3] = { 0.0, 0.0, 9.81 };

  static const double dv19[3] = { 0.0, 0.0, -9.81 };

  unsigned int unnamed_idx_0;
  int cr;
  double dv20[9];
  double dv21[16];
  double dv22[16];
  double b_x[4];
  double dv23[4];
  double d_R_cw[3];
  emxArray_real_T *FP;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *b_FP;
  double b_G[108];
  double c_FP[144];

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  a = x->data[3];
  b_a = x->data[4];
  c_a = x->data[5];
  d_a = x->data[6];
  e_a = x->data[3];
  f_a = x->data[4];
  g_a = x->data[5];
  h_a = x->data[6];
  i_a = x->data[3];
  j_a = x->data[4];
  k_a = x->data[5];
  l_a = x->data[6];
  R_cw[0] = ((a * a - b_a * b_a) - c_a * c_a) + d_a * d_a;
  R_cw[3] = 2.0 * (x->data[3] * x->data[4] + x->data[5] * x->data[6]);
  R_cw[6] = 2.0 * (x->data[3] * x->data[5] - x->data[4] * x->data[6]);
  R_cw[1] = 2.0 * (x->data[3] * x->data[4] - x->data[5] * x->data[6]);
  R_cw[4] = ((-(e_a * e_a) + f_a * f_a) - g_a * g_a) + h_a * h_a;
  R_cw[7] = 2.0 * (x->data[4] * x->data[5] + x->data[3] * x->data[6]);
  R_cw[2] = 2.0 * (x->data[3] * x->data[5] + x->data[4] * x->data[6]);
  R_cw[5] = 2.0 * (x->data[4] * x->data[5] - x->data[3] * x->data[6]);
  R_cw[8] = ((-(i_a * i_a) - j_a * j_a) + k_a * k_a) + l_a * l_a;

  // oa=x(14:16)
  for (i = 0; i < 3; i++) {
    w[i] = meas[i] - x->data[10 + i];
  }

  dv16[0] = 0.0;
  dv16[3] = -w[2];
  dv16[6] = w[1];
  dv16[1] = w[2];
  dv16[4] = 0.0;
  dv16[7] = -w[0];
  dv16[2] = -w[1];
  dv16[5] = w[0];
  dv16[8] = 0.0;
  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      b_R_cw[br + 3 * i] = -R_cw[i + 3 * br];
    }
  }

  dv17[0] = 0.0;
  dv17[3] = -meas[5];
  dv17[6] = meas[4];
  dv17[1] = meas[5];
  dv17[4] = 0.0;
  dv17[7] = -meas[3];
  dv17[2] = -meas[4];
  dv17[5] = meas[3];
  dv17[8] = 0.0;
  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      c_R_cw[i + 3 * br] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        c_R_cw[i + 3 * br] += b_R_cw[i + 3 * ar] * dv17[ar + 3 * br];
      }
    }
  }

  for (i = 0; i < 12; i++) {
    for (br = 0; br < 3; br++) {
      F[br + 12 * i] = iv1[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * i) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 3)) + 3] = -dv16[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 6)) + 3] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 9)) + 3] = iv2[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * i) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 3)) + 6] = c_R_cw[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 6)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * (i + 9)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 12; i++) {
    for (br = 0; br < 3; br++) {
      F[(br + 12 * i) + 9] = 0.0;
    }
  }

  for (i = 0; i < 9; i++) {
    for (br = 0; br < 3; br++) {
      G[br + 12 * i] = 0.0;
    }

    for (br = 0; br < 3; br++) {
      G[(br + 12 * i) + 3] = iv3[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * i) + 6] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * (i + 3)) + 6] = -R_cw[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * (i + 6)) + 6] = 0.0;
    }
  }

  for (i = 0; i < 9; i++) {
    for (br = 0; br < 3; br++) {
      G[(br + 12 * i) + 9] = iv4[br + 3 * i];
    }
  }

  if (normalGravity) {
    for (i = 0; i < 3; i++) {
      grav[i] = dv18[i];
    }
  } else {
    for (i = 0; i < 3; i++) {
      grav[i] = dv19[i];
    }
  }

  // time derivative of the state
  unnamed_idx_0 = (unsigned int)x->size[0];
  i = x_dot->size[0];
  x_dot->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)x_dot, i, (int)sizeof(double));
  cr = (int)unnamed_idx_0;
  for (i = 0; i < cr; i++) {
    x_dot->data[i] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    x_dot->data[i] = x->data[7 + i];
  }

  //  position
  dv20[0] = 0.0;
  dv20[3] = -w[2];
  dv20[6] = w[1];
  dv20[1] = w[2];
  dv20[4] = 0.0;
  dv20[7] = -w[0];
  dv20[2] = -w[1];
  dv20[5] = w[0];
  dv20[8] = 0.0;
  for (i = 0; i < 3; i++) {
    for (br = 0; br < 3; br++) {
      dv21[br + (i << 2)] = -dv20[br + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    dv21[12 + i] = w[i];
  }

  for (i = 0; i < 3; i++) {
    dv21[3 + (i << 2)] = -w[i];
  }

  dv21[15] = 0.0;
  for (i = 0; i < 4; i++) {
    for (br = 0; br < 4; br++) {
      dv22[br + (i << 2)] = 0.5 * dv21[br + (i << 2)];
    }
  }

  for (i = 0; i < 4; i++) {
    b_x[i] = x->data[3 + i];
  }

  for (i = 0; i < 4; i++) {
    dv23[i] = 0.0;
    for (br = 0; br < 4; br++) {
      dv23[i] += dv22[i + (br << 2)] * b_x[br];
    }
  }

  for (i = 0; i < 4; i++) {
    x_dot->data[3 + i] = dv23[i];
  }

  //  rot angle
  for (i = 0; i < 3; i++) {
    a = 0.0;
    for (br = 0; br < 3; br++) {
      a += R_cw[br + 3 * i] * meas[3 + br];
    }

    d_R_cw[i] = a - grav[i];
  }

  for (i = 0; i < 3; i++) {
    x_dot->data[7 + i] = d_R_cw[i];
  }

  for (i = 0; i < 3; i++) {
    x_dot->data[10 + i] = 0.0;
  }

  //  gyro bias
  i = x_dot->size[0];
  emxEnsureCapacity((emxArray__common *)x_dot, i, (int)sizeof(double));
  cr = x_dot->size[0];
  for (i = 0; i < cr; i++) {
    x_dot->data[i] *= dt;
  }

  // time derivative of the covariance
  emxInit_real_T(&FP, 2);
  if (P_xx_apr->size[0] == 1) {
    i = FP->size[0] * FP->size[1];
    FP->size[0] = 12;
    FP->size[1] = P_xx_apr->size[1];
    emxEnsureCapacity((emxArray__common *)FP, i, (int)sizeof(double));
    for (i = 0; i < 12; i++) {
      cr = P_xx_apr->size[1];
      for (br = 0; br < cr; br++) {
        FP->data[i + FP->size[0] * br] = 0.0;
        for (ar = 0; ar < 12; ar++) {
          FP->data[i + FP->size[0] * br] += F[i + 12 * ar] * P_xx_apr->data[ar +
            P_xx_apr->size[0] * br];
        }
      }
    }
  } else {
    unnamed_idx_0 = (unsigned int)P_xx_apr->size[1];
    i = FP->size[0] * FP->size[1];
    FP->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)FP, i, (int)sizeof(double));
    i = FP->size[0] * FP->size[1];
    FP->size[1] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)FP, i, (int)sizeof(double));
    cr = 12 * (int)unnamed_idx_0;
    for (i = 0; i < cr; i++) {
      FP->data[i] = 0.0;
    }

    if (P_xx_apr->size[1] == 0) {
    } else {
      i = 12 * (P_xx_apr->size[1] - 1);
      for (cr = 0; cr <= i; cr += 12) {
        for (ic = cr + 1; ic <= cr + 12; ic++) {
          FP->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= i; cr += 12) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (P_xx_apr->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              FP->data[ic] += P_xx_apr->data[ib] * F[ia];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  emxInit_real_T(&b_FP, 2);
  i = b_FP->size[0] * b_FP->size[1];
  b_FP->size[0] = FP->size[1];
  b_FP->size[1] = 12;
  emxEnsureCapacity((emxArray__common *)b_FP, i, (int)sizeof(double));
  for (i = 0; i < 12; i++) {
    cr = FP->size[1];
    for (br = 0; br < cr; br++) {
      b_FP->data[br + b_FP->size[0] * i] = FP->data[i + FP->size[0] * br];
    }
  }

  for (i = 0; i < 12; i++) {
    for (br = 0; br < 9; br++) {
      b_G[i + 12 * br] = 0.0;
      for (ar = 0; ar < 9; ar++) {
        b_G[i + 12 * br] += G[i + 12 * ar] * b_Q[ar + 9 * br];
      }
    }
  }

  for (i = 0; i < 12; i++) {
    for (br = 0; br < 12; br++) {
      a = 0.0;
      for (ar = 0; ar < 9; ar++) {
        a += b_G[i + 12 * ar] * G[br + 12 * ar];
      }

      c_FP[i + 12 * br] = (FP->data[i + 12 * br] + b_FP->data[i + 12 * br]) + a;
    }
  }

  emxFree_real_T(&b_FP);
  emxFree_real_T(&FP);
  for (i = 0; i < 12; i++) {
    for (br = 0; br < 12; br++) {
      P_xx_apr_dot[br + 12 * i] = c_FP[br + 12 * i] * dt;
    }
  }

  // time derivative of the state transition
  if (Phi->size[0] == 1) {
    i = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[0] = 12;
    Phi_dot->size[1] = Phi->size[1];
    emxEnsureCapacity((emxArray__common *)Phi_dot, i, (int)sizeof(double));
    for (i = 0; i < 12; i++) {
      cr = Phi->size[1];
      for (br = 0; br < cr; br++) {
        Phi_dot->data[i + Phi_dot->size[0] * br] = 0.0;
        for (ar = 0; ar < 12; ar++) {
          Phi_dot->data[i + Phi_dot->size[0] * br] += F[i + 12 * ar] * Phi->
            data[ar + Phi->size[0] * br];
        }
      }
    }
  } else {
    unnamed_idx_0 = (unsigned int)Phi->size[1];
    i = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[0] = 12;
    emxEnsureCapacity((emxArray__common *)Phi_dot, i, (int)sizeof(double));
    i = Phi_dot->size[0] * Phi_dot->size[1];
    Phi_dot->size[1] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)Phi_dot, i, (int)sizeof(double));
    cr = 12 * (int)unnamed_idx_0;
    for (i = 0; i < cr; i++) {
      Phi_dot->data[i] = 0.0;
    }

    if (Phi->size[1] == 0) {
    } else {
      i = 12 * (Phi->size[1] - 1);
      for (cr = 0; cr <= i; cr += 12) {
        for (ic = cr + 1; ic <= cr + 12; ic++) {
          Phi_dot->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (cr = 0; cr <= i; cr += 12) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 12; ib++) {
          if (Phi->data[ib] != 0.0) {
            ia = ar;
            for (ic = cr; ic + 1 <= cr + 12; ic++) {
              ia++;
              Phi_dot->data[ic] += Phi->data[ib] * F[ia];
            }
          }

          ar += 12;
        }

        br += 12;
      }
    }
  }

  i = Phi_dot->size[0] * Phi_dot->size[1];
  Phi_dot->size[0] = 12;
  emxEnsureCapacity((emxArray__common *)Phi_dot, i, (int)sizeof(double));
  i = Phi_dot->size[0];
  cr = Phi_dot->size[1];
  cr *= i;
  for (i = 0; i < cr; i++) {
    Phi_dot->data[i] *= dt;
  }
}

//
// Arguments    : int n
//                const double x[5]
//                int ix0
// Return Type  : double
//
static double e_eml_xnrm2(int n, const double x[5], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabs(x[k - 1]);
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
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double e_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[40] = { 'B', 'a', 'd', ' ', 't', 'r', 'i', 'a', 'n',
    'g', 'u', 'l', 'a', 't', 'i', 'o', 'n', '.', ' ', 'D', 'i', 's', 'c', 'a',
    'r', 'd', 'i', 'n', 'g', ' ', 'p', 'o', 'i', 'n', 't', ' ', '%', 'd', '\x0a',
    '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
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
  double b_A_data[1296];
  int ipiv_size[2];
  int ipiv_data[36];
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
  int i25;
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
      i25 = ic0 + ldc * (lastc - 1);
      jy = ic0;
      while ((ldc > 0) && (jy <= i25)) {
        ix = iv0;
        c = 0.0;
        j = (jy + lastv) - 1;
        for (ia = jy; ia <= j; ia++) {
          c += C_data[ia - 1] * C_data[ix - 1];
          ix++;
        }

        work_data[i] += c;
        i++;
        jy += ldc;
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
          i25 = lastv + i;
          for (ia = i; ia + 1 <= i25; ia++) {
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
  int i24;
  int k;
  tau = 0.0;
  if (n <= 0) {
  } else {
    xnorm = b_eml_xnrm2(n - 1, x_data, ix0);
    if (xnorm != 0.0) {
      xnorm = rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          i24 = (ix0 + n) - 2;
          for (k = ix0; k <= i24; k++) {
            x_data[k - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = b_eml_xnrm2(n - 1, x_data, ix0);
        xnorm = rt_hypotd_snf(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i24 = (ix0 + n) - 2;
        for (k = ix0; k <= i24; k++) {
          x_data[k - 1] *= *alpha1;
        }

        for (k = 1; k <= knt; k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        i24 = (ix0 + n) - 2;
        for (k = ix0; k <= i24; k++) {
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
  double b_A_data[1296];
  emxArray_real_T *b_B;
  int jpvt_size[2];
  int jpvt_data[36];
  int tau_size[1];
  double tau_data[36];
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
  int n;
  int yk;
  int k;
  if (b < 1) {
    n = 0;
  } else {
    n = b;
  }

  y_size[0] = 1;
  y_size[1] = n;
  if (n > 0) {
    y_data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      y_data[k - 1] = yk;
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[30]
//                int iy0
// Return Type  : void
//
static void eml_xaxpy(int n, double a, int ix0, double y[30], int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                const double x[30]
//                int ix0
//                const double y[30]
//                int iy0
// Return Type  : double
//
static double eml_xdotc(int n, const double x[30], int ix0, const double y[30],
  int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  ix = ix0;
  iy = iy0;
  for (k = 1; k <= n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
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
  double work_data[36];
  double vn1_data[36];
  double vn2_data[36];
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
  if ((A_size[0] == 0) || (A_size[1] == 0)) {
  } else {
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

    for (i = 1; i <= mn; i++) {
      i_i = (i + (i - 1) * m) - 1;
      nmi = n - i;
      mmi = m - i;
      k = eml_ixamax(1 + nmi, vn1_data, i);
      k = (i + k) - 2;
      if (k + 1 != i) {
        b_eml_xswap(m, A_data, 1 + m * k, 1 + m * (i - 1));
        itemp = jpvt_data[k];
        jpvt_data[k] = jpvt_data[i - 1];
        jpvt_data[i - 1] = itemp;
        vn1_data[k] = vn1_data[i - 1];
        vn2_data[k] = vn2_data[i - 1];
      }

      if (i < m) {
        atmp = A_data[i_i];
        tau_data[i - 1] = eml_matlab_zlarfg(mmi + 1, &atmp, A_data, i_i + 2);
      } else {
        atmp = A_data[i_i];
        tau_data[i - 1] = b_eml_matlab_zlarfg();
      }

      A_data[i_i] = atmp;
      if (i < n) {
        atmp = A_data[i_i];
        A_data[i_i] = 1.0;
        eml_matlab_zlarf(mmi + 1, nmi, i_i + 1, tau_data[i - 1], A_data, i + i *
                         m, m, work_data);
        A_data[i_i] = atmp;
      }

      for (itemp = i; itemp + 1 <= n; itemp++) {
        if (vn1_data[itemp] != 0.0) {
          atmp = fabs(A_data[(i + A_size[0] * itemp) - 1]) / vn1_data[itemp];
          atmp = 1.0 - atmp * atmp;
          if (atmp < 0.0) {
            atmp = 0.0;
          }

          temp2 = vn1_data[itemp] / vn2_data[itemp];
          temp2 = atmp * (temp2 * temp2);
          if (temp2 <= 1.4901161193847656E-8) {
            if (i < m) {
              vn1_data[itemp] = c_eml_xnrm2(mmi, A_data, (i + m * itemp) + 1);
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
  int i22;
  int j;
  int mmj;
  int c;
  int i;
  int ix;
  double smax;
  int jA;
  double s;
  int i23;
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
  if ((m < 1) || (n < 1)) {
  } else {
    if (m - 1 <= n) {
      i22 = m - 1;
    } else {
      i22 = n;
    }

    for (j = 1; j <= i22; j++) {
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

        i23 = c + mmj;
        for (i = c + 1; i + 1 <= i23; i++) {
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
          i23 = mmj + jA;
          for (ijA = 1 + jA; ijA + 1 <= i23; ijA++) {
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
// Arguments    : double *a
//                double *b
//                double *c
//                double *s
// Return Type  : void
//
static void eml_xrotg(double *a, double *b, double *c, double *s)
{
  double roe;
  double absa;
  double absb;
  double scale;
  double ads;
  double bds;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    scale = 0.0;
    *b = 0.0;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }
  }

  *a = scale;
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
  if ((n == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
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
// Arguments    : emxArray_boolean_T **pEmxArray
// Return Type  : void
//
static void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
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
// Arguments    : emxArray_boolean_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
  b_numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
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
// Arguments    : int n
//                const double x[30]
//                int ix0
// Return Type  : double
//
static double f_eml_xnrm2(int n, const double x[30], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x[k - 1]);
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
// Arguments    : int formatSpec
//                int varargin_1
// Return Type  : void
//
static void f_fprintf(int formatSpec, int varargin_1)
{
  g_fprintf(formatSpec, varargin_1);
}

//
// Arguments    : FILE * *f
//                boolean_T *a
// Return Type  : void
//
static void fileManager(FILE * *f, boolean_T *a)
{
  *f = stdout;
  *a = true;
}

//
// Arguments    : int varargin_1
//                int varargin_2
// Return Type  : double
//
static double g_fprintf(int varargin_1, int varargin_2)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[42] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z',
    'e', 'd', ' ', '%', 'i', ' ', 'n', 'e', 'w', ' ', 'f', 'e', 'a', 't', 'u',
    'r', 'e', 's', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%',
    'i', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1, varargin_2);
    fflush(filestar);
  }

  return nbytesint;
}

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
//    Does not take into account the derivative with respect to anchor states
//    (static map)
//
//  INPUT ARGUMENTS:
//  - xt:                   The current state
//  - errorStateSize:       The size of the error state
//  - z_all_l:              The feature points in the left camera (2N x 1)
//  - map:                  Map of the estimated feature points (3 x N)
//  - indMeas:              The indices of the valid feature points
//  - cameraparams:         A struct with the fields
//     - focal length, center of projection, radial distortion parameters of
//     both cameras
//     - r_lr:              Translation of right camera in left camera frame
//     - R_lr:              Orientation of right camera in left camera frame
//
//  where N is number of points in the image
//
//  OUTPUT ARGUMENTS:
//  - r:    The residual (residualDim*M x 1)
//  - H_xc: The Jacobian of the measurement function with respect to the camera states (residualDim*M x length(xt))
//
//  where M =  length(indMeas) is the number of valid points in the image
// Arguments    : const emxArray_real_T *b_xt
//                double errorStateSize
//                double stateSize
//                const double z_all_l[32]
//                double indMeas
//                const emxArray_real_T *map
//                double numAnchors
//                double numPointsPerAnchor
//                const emxArray_real_T *anchorIdx
//                const emxArray_real_T *featureAnchorIdx
//                const emxArray_real_T *b_m_vect
//                const double imNoise[2]
//                const double IMU_measurements[23]
//                double height_offset_pressure
//                double r_data[]
//                int r_size[1]
//                emxArray_real_T *H
//                double h_u[2]
//                double R_data[]
//                int R_size[2]
// Return Type  : void
//
static void getH_R_res(const emxArray_real_T *b_xt, double errorStateSize,
  double stateSize, const double z_all_l[32], double indMeas, const
  emxArray_real_T *map, double numAnchors, double numPointsPerAnchor, const
  emxArray_real_T *anchorIdx, const emxArray_real_T *featureAnchorIdx, const
  emxArray_real_T *b_m_vect, const double imNoise[2], const double
  IMU_measurements[23], double height_offset_pressure, double r_data[], int
  r_size[1], emxArray_real_T *H, double h_u[2], double R_data[], int R_size[2])
{
  emxArray_real_T *H_xm;
  double R_cw[9];
  int ib;
  int br;
  emxArray_real_T *H_xc;
  double b_map[3];
  double h_ci_l[3];
  int ar;
  double h_cin_l[3];
  int nm1d2;
  double anew;
  double z[2];
  emxArray_int32_T *r0;
  double d[4];
  double h_un_To_h_d_l[4];
  double dv0[6];
  double b_h_un_To_h_d_l[6];
  double b_R_cw[36];
  static const double c_h_un_To_h_d_l[4] = { 268.155648020127, 0.0, 0.0,
    268.867732741683 };

  double d_h_un_To_h_d_l[24];
  double b_stateSize;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double c_stateSize;
  double d_stateSize;
  double e_stateSize;
  double f_stateSize;
  double g_stateSize;
  double h_stateSize;
  double i_stateSize;
  double j_stateSize;
  double k_stateSize;
  double l_stateSize;
  double m_stateSize;
  double n_stateSize;
  double o_stateSize;
  double p_stateSize;
  double q_stateSize;
  double r_stateSize;
  double s_stateSize;
  double t_stateSize;
  double u_stateSize;
  double v_stateSize;
  double w_stateSize;
  double x_stateSize;
  double y_stateSize;
  double anchorRot[9];
  double d1;
  double y[3];
  double dv1[9];
  double b_anchorRot[9];
  emxArray_real_T *H_iy;
  int k;
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double apnd;
  double ndbl;
  double cdiff;
  double absb;
  emxArray_real_T *b_y;
  double dv2[6];
  double c_y[6];
  emxArray_real_T *H_v;
  double d_y[2];
  int ic;
  int ia;
  double b_z;
  double R_v[4];
  emxArray_real_T *H_g;
  emxArray_real_T *H_p;
  int R_g_size[2];
  int R_p_size[2];
  static const signed char b_b[3] = { 0, 0, 1 };

  double dv3[9];
  double R_g_data[9];
  double r_g_data[3];
  static const double dv4[3] = { 0.0934, -0.9946, 0.0459 };

  double R_p_data[1];
  double r_p_data[1];
  signed char unnamed_idx_0;
  signed char unnamed_idx_1;
  int tmp_size[2];
  double tmp_data[25];
  emxInit_real_T(&H_xm, 2);

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4]) -
             b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] * b_xt->data[6]);
  R_cw[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] * b_xt->data[6]);
  R_cw[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] * b_xt->data[6]);
  R_cw[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data[4])
             - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
  R_cw[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] * b_xt->data[6]);
  R_cw[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] * b_xt->data[6]);
  R_cw[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] * b_xt->data[6]);
  R_cw[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data[4])
             + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];

  //  camera parameters for the left and right camera
  //  Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1);
  //  Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2);
  ib = H_xm->size[0] * H_xm->size[1];
  H_xm->size[0] = 2;
  H_xm->size[1] = (int)(numAnchors * (6.0 + numPointsPerAnchor));
  emxEnsureCapacity((emxArray__common *)H_xm, ib, (int)sizeof(double));
  br = (int)(numAnchors * (6.0 + numPointsPerAnchor)) << 1;
  for (ib = 0; ib < br; ib++) {
    H_xm->data[ib] = 0.0;
  }

  emxInit_real_T(&H_xc, 2);
  ib = H_xc->size[0] * H_xc->size[1];
  H_xc->size[0] = 2;
  H_xc->size[1] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)H_xc, ib, (int)sizeof(double));
  br = (int)errorStateSize << 1;
  for (ib = 0; ib < br; ib++) {
    H_xc->data[ib] = 0.0;
  }

  for (ib = 0; ib < 3; ib++) {
    b_map[ib] = map->data[ib + map->size[0] * ((int)indMeas - 1)] - b_xt->
      data[ib];
  }

  for (ib = 0; ib < 3; ib++) {
    h_ci_l[ib] = 0.0;
    for (ar = 0; ar < 3; ar++) {
      h_ci_l[ib] += R_cw[ib + 3 * ar] * b_map[ar];
    }
  }

  for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
    h_cin_l[nm1d2] = h_ci_l[nm1d2] / h_ci_l[2];
  }

  predictMeasurement_left(h_ci_l, h_u);
  anew = (indMeas - 1.0) * 2.0;
  for (ib = 0; ib < 2; ib++) {
    z[ib] = z_all_l[(int)(anew + (1.0 + (double)ib)) - 1];
  }

  emxInit_int32_T(&r0, 1);

  //     %% computation of H(x)
  br = (int)errorStateSize;
  ib = r0->size[0];
  r0->size[0] = (int)errorStateSize;
  emxEnsureCapacity((emxArray__common *)r0, ib, (int)sizeof(int));
  for (ib = 0; ib < br; ib++) {
    r0->data[ib] = ib;
  }

  Ch_dn_To_h_un(h_cin_l[0], h_cin_l[1], d);
  dv0[0] = 1.0 / h_ci_l[2];
  dv0[2] = 0.0;
  dv0[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  dv0[1] = 0.0;
  dv0[3] = 1.0 / h_ci_l[2];
  dv0[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  for (ib = 0; ib < 2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        h_un_To_h_d_l[ib + (ar << 1)] += c_h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          d[nm1d2 + (ar << 1)];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      b_h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        b_h_un_To_h_d_l[ib + (ar << 1)] += h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          dv0[nm1d2 + (ar << 1)];
      }
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      b_R_cw[ar + 3 * ib] = -R_cw[ar + 3 * ib];
    }
  }

  b_R_cw[9] = 0.0;
  b_R_cw[12] = -h_ci_l[2];
  b_R_cw[15] = h_ci_l[1];
  b_R_cw[10] = h_ci_l[2];
  b_R_cw[13] = 0.0;
  b_R_cw[16] = -h_ci_l[0];
  b_R_cw[11] = -h_ci_l[1];
  b_R_cw[14] = h_ci_l[0];
  b_R_cw[17] = 0.0;
  for (ib = 0; ib < 6; ib++) {
    for (ar = 0; ar < 3; ar++) {
      b_R_cw[ar + 3 * (ib + 6)] = 0.0;
    }
  }

  for (ib = 0; ib < 2; ib++) {
    for (ar = 0; ar < 12; ar++) {
      d_h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        d_h_un_To_h_d_l[ib + (ar << 1)] += b_h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          b_R_cw[nm1d2 + 3 * ar];
      }
    }
  }

  nm1d2 = r0->size[0];
  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_xc->data[ar + H_xc->size[0] * r0->data[ib]] = d_h_un_To_h_d_l[ar + (ib <<
        1)];
    }
  }

  //     %% anchor state derivatives
  //  fp = anchorPos + anchorRot'*m/rho - r_wc_pred;
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  b_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  c_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  d_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  e_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  f_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  g_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  h_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  i_a = b_xt->data[(int)(b_stateSize + 4.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  j_a = b_xt->data[(int)(b_stateSize + 5.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  k_a = b_xt->data[(int)(b_stateSize + 6.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  l_a = b_xt->data[(int)(b_stateSize + 7.0) - 1];
  b_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  c_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  d_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  e_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  f_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  g_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  h_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  i_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  j_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  k_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  l_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  m_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  n_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  o_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  p_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  q_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  r_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  s_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  t_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  u_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  v_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  w_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  x_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  y_stateSize = stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0) * (7.0 +
    numPointsPerAnchor);
  anchorRot[0] = ((a * a - b_a * b_a) - c_a * c_a) + d_a * d_a;
  anchorRot[3] = 2.0 * (b_xt->data[(int)(b_stateSize + 4.0) - 1] * b_xt->data
                        [(int)(c_stateSize + 5.0) - 1] + b_xt->data[(int)
                        (d_stateSize + 6.0) - 1] * b_xt->data[(int)(e_stateSize
    + 7.0) - 1]);
  anchorRot[6] = 2.0 * (b_xt->data[(int)(f_stateSize + 4.0) - 1] * b_xt->data
                        [(int)(g_stateSize + 6.0) - 1] - b_xt->data[(int)
                        (h_stateSize + 5.0) - 1] * b_xt->data[(int)(i_stateSize
    + 7.0) - 1]);
  anchorRot[1] = 2.0 * (b_xt->data[(int)(j_stateSize + 4.0) - 1] * b_xt->data
                        [(int)(k_stateSize + 5.0) - 1] - b_xt->data[(int)
                        (l_stateSize + 6.0) - 1] * b_xt->data[(int)(m_stateSize
    + 7.0) - 1]);
  anchorRot[4] = ((-(e_a * e_a) + f_a * f_a) - g_a * g_a) + h_a * h_a;
  anchorRot[7] = 2.0 * (b_xt->data[(int)(n_stateSize + 5.0) - 1] * b_xt->data
                        [(int)(o_stateSize + 6.0) - 1] + b_xt->data[(int)
                        (p_stateSize + 4.0) - 1] * b_xt->data[(int)(q_stateSize
    + 7.0) - 1]);
  anchorRot[2] = 2.0 * (b_xt->data[(int)(r_stateSize + 4.0) - 1] * b_xt->data
                        [(int)(s_stateSize + 6.0) - 1] + b_xt->data[(int)
                        (t_stateSize + 5.0) - 1] * b_xt->data[(int)(u_stateSize
    + 7.0) - 1]);
  anchorRot[5] = 2.0 * (b_xt->data[(int)(v_stateSize + 5.0) - 1] * b_xt->data
                        [(int)(w_stateSize + 6.0) - 1] - b_xt->data[(int)
                        (x_stateSize + 4.0) - 1] * b_xt->data[(int)(y_stateSize
    + 7.0) - 1]);
  anchorRot[8] = ((-(i_a * i_a) - j_a * j_a) + k_a * k_a) + l_a * l_a;
  anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas -
    1]) - 1];
  for (ib = 0; ib < 3; ib++) {
    d1 = 0.0;
    for (ar = 0; ar < 3; ar++) {
      d1 += anchorRot[ar + 3 * ib] * b_m_vect->data[ar + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    y[ib] = d1 / anew;
  }

  anew = b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1] - 1.0)
    * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)indMeas -
    1]) - 1] * b_xt->data[(int)(((stateSize + (anchorIdx->data[(int)indMeas - 1]
    - 1.0) * (7.0 + numPointsPerAnchor)) + 7.0) + featureAnchorIdx->data[(int)
    indMeas - 1]) - 1];
  dv1[0] = 0.0;
  dv1[3] = -y[2];
  dv1[6] = y[1];
  dv1[1] = y[2];
  dv1[4] = 0.0;
  dv1[7] = -y[0];
  dv1[2] = -y[1];
  dv1[5] = y[0];
  dv1[8] = 0.0;
  nm1d2 = (int)(featureAnchorIdx->data[(int)indMeas - 1] - 1.0);
  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      b_anchorRot[ar + 3 * ib] = -anchorRot[ib + 3 * ar];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    d1 = 0.0;
    for (ar = 0; ar < 3; ar++) {
      d1 += b_anchorRot[ib + 3 * ar] * b_m_vect->data[ar + b_m_vect->size[0] *
        ((int)indMeas - 1)];
    }

    b_map[ib] = d1 / anew;
  }

  emxInit_real_T(&H_iy, 2);
  k = (int)(numPointsPerAnchor - featureAnchorIdx->data[(int)indMeas - 1]);
  ib = H_iy->size[0] * H_iy->size[1];
  H_iy->size[0] = 3;
  H_iy->size[1] = (nm1d2 + k) + 7;
  emxEnsureCapacity((emxArray__common *)H_iy, ib, (int)sizeof(double));
  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * ib] = b[ar + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * (ib + 3)] = -dv1[ar + 3 * ib];
    }
  }

  for (ib = 0; ib < nm1d2; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * (ib + 6)] = 0.0;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    H_iy->data[ib + H_iy->size[0] * (6 + nm1d2)] = b_map[ib];
  }

  for (ib = 0; ib < k; ib++) {
    for (ar = 0; ar < 3; ar++) {
      H_iy->data[ar + H_iy->size[0] * ((ib + nm1d2) + 7)] = 0.0;
    }
  }

  if (rtIsNaN(6.0 + numPointsPerAnchor)) {
    br = 0;
    anew = rtNaN;
    apnd = 6.0 + numPointsPerAnchor;
  } else if (6.0 + numPointsPerAnchor < 1.0) {
    br = -1;
    anew = 1.0;
    apnd = 6.0 + numPointsPerAnchor;
  } else if (rtIsInf(6.0 + numPointsPerAnchor)) {
    br = 0;
    anew = rtNaN;
    apnd = 6.0 + numPointsPerAnchor;
  } else {
    anew = 1.0;
    ndbl = floor(((6.0 + numPointsPerAnchor) - 1.0) + 0.5);
    apnd = 1.0 + ndbl;
    cdiff = (1.0 + ndbl) - (6.0 + numPointsPerAnchor);
    absb = fabs(6.0 + numPointsPerAnchor);
    if ((1.0 >= absb) || rtIsNaN(absb)) {
      absb = 1.0;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = 6.0 + numPointsPerAnchor;
    } else if (cdiff > 0.0) {
      apnd = 1.0 + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      br = (int)ndbl - 1;
    } else {
      br = -1;
    }
  }

  emxInit_real_T(&b_y, 2);
  ib = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = br + 1;
  emxEnsureCapacity((emxArray__common *)b_y, ib, (int)sizeof(double));
  if (br + 1 > 0) {
    b_y->data[0] = anew;
    if (br + 1 > 1) {
      b_y->data[br] = apnd;
      nm1d2 = (br + (br < 0)) >> 1;
      for (k = 1; k < nm1d2; k++) {
        b_y->data[k] = anew + (double)k;
        b_y->data[br - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == br) {
        b_y->data[nm1d2] = (anew + apnd) / 2.0;
      } else {
        b_y->data[nm1d2] = anew + (double)nm1d2;
        b_y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  anew = (anchorIdx->data[(int)indMeas - 1] - 1.0) * (6.0 + numPointsPerAnchor);
  ib = r0->size[0];
  r0->size[0] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)r0, ib, (int)sizeof(int));
  br = b_y->size[1];
  for (ib = 0; ib < br; ib++) {
    r0->data[ib] = (int)(anew + b_y->data[b_y->size[0] * ib]) - 1;
  }

  emxFree_real_T(&b_y);
  Ch_dn_To_h_un(h_cin_l[0], h_cin_l[1], d);
  dv2[0] = 1.0 / h_ci_l[2];
  dv2[2] = 0.0;
  dv2[4] = -h_ci_l[0] / (h_ci_l[2] * h_ci_l[2]);
  dv2[1] = 0.0;
  dv2[3] = 1.0 / h_ci_l[2];
  dv2[5] = -h_ci_l[1] / (h_ci_l[2] * h_ci_l[2]);
  for (ib = 0; ib < 2; ib++) {
    for (ar = 0; ar < 2; ar++) {
      h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        h_un_To_h_d_l[ib + (ar << 1)] += c_h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          d[nm1d2 + (ar << 1)];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      b_h_un_To_h_d_l[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
        b_h_un_To_h_d_l[ib + (ar << 1)] += h_un_To_h_d_l[ib + (nm1d2 << 1)] *
          dv2[nm1d2 + (ar << 1)];
      }
    }

    for (ar = 0; ar < 3; ar++) {
      c_y[ib + (ar << 1)] = 0.0;
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        c_y[ib + (ar << 1)] += b_h_un_To_h_d_l[ib + (nm1d2 << 1)] * R_cw[nm1d2 +
          3 * ar];
      }
    }
  }

  emxInit_real_T(&H_v, 2);
  d_y[1] = H_iy->size[1];
  ib = H_v->size[0] * H_v->size[1];
  H_v->size[0] = 2;
  emxEnsureCapacity((emxArray__common *)H_v, ib, (int)sizeof(double));
  ib = H_v->size[0] * H_v->size[1];
  H_v->size[1] = (int)d_y[1];
  emxEnsureCapacity((emxArray__common *)H_v, ib, (int)sizeof(double));
  br = (int)d_y[1] << 1;
  for (ib = 0; ib < br; ib++) {
    H_v->data[ib] = 0.0;
  }

  nm1d2 = (H_iy->size[1] - 1) << 1;
  for (k = 0; k <= nm1d2; k += 2) {
    for (ic = k + 1; ic <= k + 2; ic++) {
      H_v->data[ic - 1] = 0.0;
    }
  }

  br = 0;
  for (k = 0; k <= nm1d2; k += 2) {
    ar = 0;
    for (ib = br; ib + 1 <= br + 3; ib++) {
      if (H_iy->data[ib] != 0.0) {
        ia = ar;
        for (ic = k; ic + 1 <= k + 2; ic++) {
          ia++;
          H_v->data[ic] += H_iy->data[ib] * c_y[ia - 1];
        }
      }

      ar += 2;
    }

    br += 3;
  }

  emxFree_real_T(&H_iy);
  br = H_v->size[1];
  for (ib = 0; ib < br; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_xm->data[ar + H_xm->size[0] * r0->data[ib]] = H_v->data[ar + H_v->size[0]
        * ib];
    }
  }

  emxFree_int32_T(&r0);
  for (ib = 0; ib < 2; ib++) {
    b_z = z[ib] - h_u[ib];
    z[ib] = b_z;
  }

  //  residual with respect to camera measurements
  for (k = 0; k < 2; k++) {
    d_y[k] = imNoise[k] * imNoise[k];
  }

  for (ib = 0; ib < 4; ib++) {
    d[ib] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    d[nm1d2 + (nm1d2 << 1)] = d_y[nm1d2];
  }

  nm1d2 = -1;
  for (k = 0; k < 2; k++) {
    for (br = 0; br < 2; br++) {
      nm1d2++;
      R_v[nm1d2] = d[br + (k << 1)];
    }
  }

  nm1d2 = 0;

  //  gravity residual
  k = 0;

  //  pressure residual
  ib = H_v->size[0] * H_v->size[1];
  H_v->size[0] = 2;
  H_v->size[1] = H_xc->size[1] + H_xm->size[1];
  emxEnsureCapacity((emxArray__common *)H_v, ib, (int)sizeof(double));
  br = H_xc->size[1];
  for (ib = 0; ib < br; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_v->data[ar + H_v->size[0] * ib] = H_xc->data[ar + H_xc->size[0] * ib];
    }
  }

  br = H_xm->size[1];
  for (ib = 0; ib < br; ib++) {
    for (ar = 0; ar < 2; ar++) {
      H_v->data[ar + H_v->size[0] * (ib + H_xc->size[1])] = H_xm->data[ar +
        H_xm->size[0] * ib];
    }
  }

  emxFree_real_T(&H_xc);
  emxFree_real_T(&H_xm);
  emxInit_real_T(&H_g, 2);
  emxInit_real_T(&H_p, 2);
  ib = H_g->size[0] * H_g->size[1];
  H_g->size[0] = 0;
  H_g->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_g, ib, (int)sizeof(double));

  //  jacobian for gravity residual
  ib = H_p->size[0] * H_p->size[1];
  H_p->size[0] = 0;
  H_p->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)H_p, ib, (int)sizeof(double));

  //  jacobian for pressure residual
  R_g_size[0] = 0;
  R_g_size[1] = 0;
  R_p_size[0] = 0;
  R_p_size[1] = 0;
  if (gravityUpdate) {
    //  normalize the acceleration measurement
    //  normalize the magnetometer measurement
    for (ib = 0; ib < 3; ib++) {
      h_ci_l[ib] = 0.0;
      for (ar = 0; ar < 3; ar++) {
        h_ci_l[ib] += R_cw[ib + 3 * ar] * (double)b_b[ar];
      }
    }

    //  the earth-z axis transformed into the body frame
    ib = H_g->size[0] * H_g->size[1];
    H_g->size[0] = 3;
    H_g->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_g, ib, (int)sizeof(double));
    br = 3 * (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ib = 0; ib < br; ib++) {
      H_g->data[ib] = 0.0;
    }

    dv3[0] = 0.0;
    dv3[3] = -h_ci_l[2];
    dv3[6] = h_ci_l[1];
    dv3[1] = h_ci_l[2];
    dv3[4] = 0.0;
    dv3[7] = -h_ci_l[0];
    dv3[2] = -h_ci_l[1];
    dv3[5] = h_ci_l[0];
    dv3[8] = 0.0;
    for (ib = 0; ib < 3; ib++) {
      for (ar = 0; ar < 3; ar++) {
        H_g->data[ar + H_g->size[0] * (3 + ib)] = dv3[ar + 3 * ib];
      }
    }

    R_g_size[0] = 3;
    R_g_size[1] = 3;
    for (ib = 0; ib < 9; ib++) {
      R_g_data[ib] = gravAlignNoise * (double)b[ib];
    }

    nm1d2 = 3;
    for (ib = 0; ib < 3; ib++) {
      r_g_data[ib] = dv4[ib] - h_ci_l[ib];
    }
  }

  if (useAirPressure) {
    ib = H_p->size[0] * H_p->size[1];
    H_p->size[0] = 1;
    H_p->size[1] = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    emxEnsureCapacity((emxArray__common *)H_p, ib, (int)sizeof(double));
    br = (int)(errorStateSize + numAnchors * (6.0 + numPointsPerAnchor));
    for (ib = 0; ib < br; ib++) {
      H_p->data[ib] = 0.0;
    }

    H_p->data[2] = 1.0;
    R_p_size[0] = 1;
    R_p_size[1] = 1;
    R_p_data[0] = 2.0;
    k = 1;
    r_p_data[0] = (1.0 - rt_powd_snf(IMU_measurements[9] / 101325.0, 0.190284)) *
      145366.45 - height_offset_pressure;
  }

  if (gravityUpdate) {
    if (useAirPressure) {
      r_size[0] = (nm1d2 + k) + 2;
      for (ib = 0; ib < 2; ib++) {
        r_data[ib] = z[ib];
      }

      for (ib = 0; ib < nm1d2; ib++) {
        r_data[ib + 2] = r_g_data[ib];
      }

      ib = 0;
      while (ib <= k - 1) {
        r_data[nm1d2 + 2] = r_p_data[0];
        ib = 1;
      }

      ib = H->size[0] * H->size[1];
      H->size[0] = (H_g->size[0] + H_p->size[0]) + 2;
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
      br = H_v->size[1];
      for (ib = 0; ib < br; ib++) {
        for (ar = 0; ar < 2; ar++) {
          H->data[ar + H->size[0] * ib] = H_v->data[ar + H_v->size[0] * ib];
        }
      }

      br = H_g->size[1];
      for (ib = 0; ib < br; ib++) {
        nm1d2 = H_g->size[0];
        for (ar = 0; ar < nm1d2; ar++) {
          H->data[(ar + H->size[0] * ib) + 2] = H_g->data[ar + H_g->size[0] * ib];
        }
      }

      br = H_p->size[1];
      for (ib = 0; ib < br; ib++) {
        nm1d2 = H_p->size[0];
        for (ar = 0; ar < nm1d2; ar++) {
          H->data[((ar + H_g->size[0]) + H->size[0] * ib) + 2] = H_p->data[ar +
            H_p->size[0] * ib];
        }
      }

      unnamed_idx_0 = (signed char)((R_g_size[0] + R_p_size[0]) + 2);
      unnamed_idx_1 = (signed char)((R_g_size[1] + R_p_size[1]) + 2);
      R_size[0] = unnamed_idx_0;
      R_size[1] = unnamed_idx_1;
      br = unnamed_idx_0 * unnamed_idx_1;
      for (ib = 0; ib < br; ib++) {
        R_data[ib] = 0.0;
      }

      for (ib = 0; ib < 2; ib++) {
        for (ar = 0; ar < 2; ar++) {
          R_data[ar + R_size[0] * ib] = R_v[ar + (ib << 1)];
        }
      }

      if ((R_g_size[0] > 0) && (R_g_size[1] > 0)) {
        br = R_g_size[1];
        for (ib = 0; ib < br; ib++) {
          nm1d2 = R_g_size[0];
          for (ar = 0; ar < nm1d2; ar++) {
            R_data[(ar + R_size[0] * (2 + ib)) + 2] = R_g_data[ar + R_g_size[0] *
              ib];
          }
        }
      }

      if ((R_p_size[0] > 0) && (R_p_size[1] > 0)) {
        ib = 0;
        while (ib <= 0) {
          ib = 0;
          while (ib <= 0) {
            R_data[(R_g_size[0] + R_size[0] * (R_g_size[1] + 2)) + 2] = 2.0;
            ib = 1;
          }

          ib = 1;
        }
      }
    } else {
      r_size[0] = 2 + nm1d2;
      for (ib = 0; ib < 2; ib++) {
        r_data[ib] = z[ib];
      }

      for (ib = 0; ib < nm1d2; ib++) {
        r_data[ib + 2] = r_g_data[ib];
      }

      ib = H->size[0] * H->size[1];
      H->size[0] = 2 + H_g->size[0];
      H->size[1] = H_v->size[1];
      emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
      br = H_v->size[1];
      for (ib = 0; ib < br; ib++) {
        for (ar = 0; ar < 2; ar++) {
          H->data[ar + H->size[0] * ib] = H_v->data[ar + H_v->size[0] * ib];
        }
      }

      br = H_g->size[1];
      for (ib = 0; ib < br; ib++) {
        nm1d2 = H_g->size[0];
        for (ar = 0; ar < nm1d2; ar++) {
          H->data[(ar + H->size[0] * ib) + 2] = H_g->data[ar + H_g->size[0] * ib];
        }
      }

      b_blkdiag(R_v, R_g_data, R_g_size, tmp_data, tmp_size);
      R_size[0] = tmp_size[0];
      R_size[1] = tmp_size[1];
      br = tmp_size[0] * tmp_size[1];
      for (ib = 0; ib < br; ib++) {
        R_data[ib] = tmp_data[ib];
      }
    }
  } else if (useAirPressure) {
    r_size[0] = 2 + k;
    for (ib = 0; ib < 2; ib++) {
      r_data[ib] = z[ib];
    }

    ib = 0;
    while (ib <= k - 1) {
      r_data[2] = r_p_data[0];
      ib = 1;
    }

    ib = H->size[0] * H->size[1];
    H->size[0] = 2 + H_p->size[0];
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
    br = H_v->size[1];
    for (ib = 0; ib < br; ib++) {
      for (ar = 0; ar < 2; ar++) {
        H->data[ar + H->size[0] * ib] = H_v->data[ar + H_v->size[0] * ib];
      }
    }

    br = H_p->size[1];
    for (ib = 0; ib < br; ib++) {
      nm1d2 = H_p->size[0];
      for (ar = 0; ar < nm1d2; ar++) {
        H->data[(ar + H->size[0] * ib) + 2] = H_p->data[ar + H_p->size[0] * ib];
      }
    }

    b_blkdiag(R_v, R_p_data, R_p_size, tmp_data, tmp_size);
    R_size[0] = tmp_size[0];
    R_size[1] = tmp_size[1];
    br = tmp_size[0] * tmp_size[1];
    for (ib = 0; ib < br; ib++) {
      R_data[ib] = tmp_data[ib];
    }
  } else {
    r_size[0] = 2;
    for (ib = 0; ib < 2; ib++) {
      r_data[ib] = z[ib];
    }

    ib = H->size[0] * H->size[1];
    H->size[0] = 2;
    H->size[1] = H_v->size[1];
    emxEnsureCapacity((emxArray__common *)H, ib, (int)sizeof(double));
    br = H_v->size[0] * H_v->size[1];
    for (ib = 0; ib < br; ib++) {
      H->data[ib] = H_v->data[ib];
    }

    R_size[0] = 2;
    R_size[1] = 2;
    for (ib = 0; ib < 4; ib++) {
      R_data[ib] = R_v[ib];
    }
  }

  emxFree_real_T(&H_p);
  emxFree_real_T(&H_g);
  emxFree_real_T(&H_v);
}

//
// GETMAP Get the feature points from the current state estimate
//
//  INPUT ARGUMENTS:
//  - x:                    The current state estimate including anchor poses and inverse depths
//  - anchorFeatures:       A matrix describing which features belong to which
//                          anchor
//  - m_vect:               A matrix containing the rays in the left camera anchor frame of
//                          each feature
//  - numTrackFeatures:     The number of tracked features (== length of
//                          updateVect)
//  - stateSize:            The size of the robot state in xt
//  - numstatesPerAnchorxt: The size of each anchorstate in xt
//
//  OUTPUT ARGUMENTS:
//  - map:                  The map of feature points in world coordinates (3 x
//                          numTrackFeatures)
//  - anchorInd:            A vector describing which anchor each feature belongs to
//  - featureAnchorInd:     A vector describing the index of each feature in its
//                          anchor
// Arguments    : const emxArray_real_T *x
//                const emxArray_real_T *b_anchorFeatures
//                const emxArray_real_T *b_m_vect
//                double c_numTrackFeatures
//                double stateSize
//                double numStatesPerAnchorxt
//                emxArray_real_T *map
//                emxArray_real_T *anchorInd
//                emxArray_real_T *featureAnchorInd
// Return Type  : void
//
static void getMap(const emxArray_real_T *x, const emxArray_real_T
                   *b_anchorFeatures, const emxArray_real_T *b_m_vect, double
                   c_numTrackFeatures, double stateSize, double
                   numStatesPerAnchorxt, emxArray_real_T *map, emxArray_real_T
                   *anchorInd, emxArray_real_T *featureAnchorInd)
{
  int i6;
  int ii;
  int anchorIdx;
  int idx;
  signed char ii_data[16];
  boolean_T exitg1;
  boolean_T guard1 = false;
  int ii_size_idx_0;
  signed char featureIdxVect_data[16];
  double b_stateSize;
  double anchorPos[3];
  double b_x[9];
  double d0;
  i6 = map->size[0] * map->size[1];
  map->size[0] = 3;
  map->size[1] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)map, i6, (int)sizeof(double));
  ii = 3 * (int)c_numTrackFeatures;
  for (i6 = 0; i6 < ii; i6++) {
    map->data[i6] = rtNaN;
  }

  i6 = anchorInd->size[0];
  anchorInd->size[0] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)anchorInd, i6, (int)sizeof(double));
  ii = (int)c_numTrackFeatures;
  for (i6 = 0; i6 < ii; i6++) {
    anchorInd->data[i6] = 0.0;
  }

  i6 = featureAnchorInd->size[0];
  featureAnchorInd->size[0] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)featureAnchorInd, i6, (int)sizeof(double));
  ii = (int)c_numTrackFeatures;
  for (i6 = 0; i6 < ii; i6++) {
    featureAnchorInd->data[i6] = 0.0;
  }

  for (anchorIdx = 0; anchorIdx < b_anchorFeatures->size[1]; anchorIdx++) {
    idx = 0;
    ii = 1;
    exitg1 = false;
    while ((!exitg1) && (ii < 17)) {
      guard1 = false;
      if (b_anchorFeatures->data[(ii + b_anchorFeatures->size[0] * anchorIdx) -
          1] != 0.0) {
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
      ii = 0;
    } else {
      ii = idx;
    }

    for (i6 = 0; i6 < ii; i6++) {
      featureIdxVect_data[i6] = ii_data[i6];
    }

    //  the transpose prevents going into the loop if find returns empty
    for (ii = 0; ii < ii_size_idx_0; ii++) {
      if (b_anchorFeatures->data[(featureIdxVect_data[ii] +
           b_anchorFeatures->size[0] * anchorIdx) - 1] == 1.0) {
        //  if this is not a lost feature
        b_stateSize = stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          numStatesPerAnchorxt;
        for (i6 = 0; i6 < 3; i6++) {
          anchorPos[i6] = x->data[(int)(b_stateSize + (1.0 + (double)i6)) - 1];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        b_x[0] = ((x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize + ((1.0 +
          (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] -
                   x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize + ((1.0 +
          (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 5.0) - 1]) -
                  x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                    numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)((stateSize
                    + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt)
                   + 6.0) - 1]) + x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1] * x->data[(int)
          ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt)
           + 7.0) - 1];
        b_x[1] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 5.0) - 1]
                        + x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
          - 1.0) * numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[2] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 6.0) - 1]
                        - x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
          - 1.0) * numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[3] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 5.0) - 1]
                        - x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
          - 1.0) * numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[4] = ((-(x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0)
          * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize + ((1.0
          + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 4.0) - 1]) +
                   x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize + ((1.0 +
          (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 5.0) - 1]) -
                  x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                    numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)((stateSize
                    + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt)
                   + 6.0) - 1]) + x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1] * x->data[(int)
          ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt)
           + 7.0) - 1];
        b_x[5] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 6.0) - 1]
                        + x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
          - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[6] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 6.0) - 1]
                        + x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
          - 1.0) * numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[7] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 6.0) - 1]
                        - x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
          - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[8] = ((-(x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0)
          * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize + ((1.0
          + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 4.0) - 1]) -
                   x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize + ((1.0 +
          (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 5.0) - 1]) +
                  x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                    numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)((stateSize
                    + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt)
                   + 6.0) - 1]) + x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1] * x->data[(int)
          ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt)
           + 7.0) - 1];
        b_stateSize = x->data[(int)(((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * numStatesPerAnchorxt) + 7.0) + (1.0 + (double)ii)) - 1];
        for (i6 = 0; i6 < 3; i6++) {
          d0 = 0.0;
          for (idx = 0; idx < 3; idx++) {
            d0 += b_x[i6 + 3 * idx] * b_m_vect->data[idx + b_m_vect->size[0] *
              (featureIdxVect_data[ii] - 1)];
          }

          map->data[i6 + map->size[0] * (featureIdxVect_data[ii] - 1)] =
            anchorPos[i6] + d0 / b_stateSize;
        }

        anchorInd->data[featureIdxVect_data[ii] - 1] = 1.0 + (double)anchorIdx;
        featureAnchorInd->data[featureIdxVect_data[ii] - 1] = 1.0 + (double)ii;
      }
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
static void h_fprintf()
{
  i_fprintf();
}

//
// Arguments    : void
// Return Type  : double
//
static double i_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[79] = { 'T', 'h', 'e', 'r', 'e', ' ', 'a', 'r', 'e',
    ' ', 's', 't', 'e', 'r', 'e', 'o', ' ', 'm', 'e', 'a', 's', 'u', 'r', 'e',
    'm', 'e', 'n', 't', 's', ' ', 'a', 'v', 'a', 'i', 'l', 'a', 'b', 'l', 'e',
    ' ', 'b', 'u', 't', ' ', 'n', 'o', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ',
    'n', 'e', 'e', 'd', 's', ' ', 't', 'o', ' ', 'b', 'e', ' ', 'i', 'n', 'i',
    't', 'i', 'a', 'l', 'i', 'z', 'e', 'd', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt);
    fflush(filestar);
  }

  return nbytesint;
}

//
// INITIALIZEPOINT Initialize a feature point from current measurements
//
//  INPUT ARGUMENTS:
//  - xt: The current camera state
//  - cameraparams: The camera parameter struct
//  - z_l: The measurement of the feature in the left camera
//  - z_r: The measurement of the feature in the right camera
//
//  OUTPUT ARGUMENTS:
//  - fp: The feature point in the world frame
//  - m:  The ray in the left camera frame to the feature
// Arguments    : const emxArray_real_T *b_xt
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                const double z_l[2]
//                const double z_r[2]
//                double fp[3]
//                double m_out[3]
// Return Type  : void
//
static void initializePoint(const emxArray_real_T *b_xt, const double
  cameraparams_r_lr[3], const double cameraparams_R_lr[9], const double z_l[2],
  const double z_r[2], double fp[3], double m_out[3])
{
  double pos[6];
  int j;
  double dv14[4];
  double b_cameraparams_R_lr[9];
  int rankR;
  double dv15[4];
  double rot[8];
  double zn_d_l[2];
  double zn_d_r[2];
  double rad_d_l;
  double rad_d_r;
  double r_u_l;
  double r_u_r;
  int i;
  double b_r_u_l[2];
  boolean_T b[2];
  boolean_T y;
  boolean_T exitg3;
  boolean_T guard1 = false;
  double c_r_u_l[2];
  boolean_T exitg2;
  double B;
  double absxk;
  double mr[3];
  double m[6];
  double A[30];
  double b_b[6];
  int anchorIdx;
  double b_rot[9];
  signed char I[9];
  double s[5];
  int exponent;
  double unusedExpr[5];
  int jpvt[5];
  double x[5];
  boolean_T exitg1;
  double c_xt[9];

  //  camera parameters for the left and right camera
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  //  R_lw = R_cw;
  //  R_rw = R_lr'*R_cw;
  //  pos  = [r_cw,r_cw + R_cw*r_lr];
  //  rot  = [QuatFromRotJ(R_lw),QuatFromRotJ(R_rw)];
  for (j = 0; j < 3; j++) {
    pos[j] = 0.0;
    pos[3 + j] = cameraparams_r_lr[j];
  }

  b_QuatFromRotJ(dv14);
  for (j = 0; j < 3; j++) {
    for (rankR = 0; rankR < 3; rankR++) {
      b_cameraparams_R_lr[rankR + 3 * j] = cameraparams_R_lr[j + 3 * rankR];
    }
  }

  QuatFromRotJ(b_cameraparams_R_lr, dv15);
  for (j = 0; j < 4; j++) {
    rot[j] = dv14[j];
    rot[4 + j] = dv15[j];
  }

  zn_d_l[0] = (z_l[0] - 155.972717007495) / 268.155648020127;
  zn_d_l[1] = (z_l[1] - 113.206085625994) / 268.867732741683;
  zn_d_r[0] = (z_r[0] - 167.100031218981) / 268.839577384212;
  zn_d_r[1] = (z_r[1] - 107.901779803044) / 269.510643351885;
  rad_d_l = sqrt(zn_d_l[0] * zn_d_l[0] + zn_d_l[1] * zn_d_l[1]);

  //  the radius for the undistortion
  rad_d_r = sqrt(zn_d_r[0] * zn_d_r[0] + zn_d_r[1] * zn_d_r[1]);

  // R_U Summary of this function goes here
  //    Detailed explanation goes here
  r_u_l = 1.0;

  // R_U Summary of this function goes here
  //    Detailed explanation goes here
  r_u_r = 1.0;
  for (i = 0; i < 10; i++) {
    // ru=ru-(ru+k1*ru^3+k2*ru^5-rd)/(1+3*k1*ru^2+5*k2*ru^4);
    r_u_l -= ((((r_u_l + -0.414085141240295 * rt_powd_snf(r_u_l, 3.0)) +
                0.236451305145822 * rt_powd_snf(r_u_l, 5.0)) +
               -0.0871296995623235 * rt_powd_snf(r_u_l, 7.0)) - rad_d_l) /
      (((1.0 + -1.2422554237208849 * (r_u_l * r_u_l)) + 1.18225652572911 *
        rt_powd_snf(r_u_l, 4.0)) + -0.60990789693626446 * rt_powd_snf(r_u_l, 6.0));

    // ru=ru-(ru+k1*ru^3+k2*ru^5-rd)/(1+3*k1*ru^2+5*k2*ru^4);
    r_u_r -= ((((r_u_r + -0.410786366925601 * rt_powd_snf(r_u_r, 3.0)) +
                0.222940449996276 * rt_powd_snf(r_u_r, 5.0)) +
               -0.0755554113677893 * rt_powd_snf(r_u_r, 7.0)) - rad_d_r) /
      (((1.0 + -1.2323591007768031 * (r_u_r * r_u_r)) + 1.11470224998138 *
        rt_powd_snf(r_u_r, 4.0)) + -0.5288878795745251 * rt_powd_snf(r_u_r, 6.0));
  }

  b_r_u_l[0] = r_u_l;
  b_r_u_l[1] = r_u_r;
  for (j = 0; j < 2; j++) {
    b[j] = rtIsNaN(b_r_u_l[j]);
  }

  y = false;
  rankR = 0;
  exitg3 = false;
  while ((!exitg3) && (rankR < 2)) {
    if (!!b[rankR]) {
      y = true;
      exitg3 = true;
    } else {
      rankR++;
    }
  }

  guard1 = false;
  if (!y) {
    c_r_u_l[0] = r_u_l;
    c_r_u_l[1] = r_u_r;
    for (j = 0; j < 2; j++) {
      b[j] = rtIsInf(c_r_u_l[j]);
    }

    y = false;
    rankR = 0;
    exitg2 = false;
    while ((!exitg2) && (rankR < 2)) {
      if (!!b[rankR]) {
        y = true;
        exitg2 = true;
      } else {
        rankR++;
      }
    }

    if (!y) {
      B = ((1.0 + -0.414085141240295 * (r_u_l * r_u_l)) + 0.236451305145822 *
           rt_powd_snf(r_u_l, 4.0)) + -0.0871296995623235 * rt_powd_snf(r_u_l,
        6.0);

      // undistort points
      absxk = ((1.0 + -0.410786366925601 * (r_u_r * r_u_r)) + 0.222940449996276 *
               rt_powd_snf(r_u_r, 4.0)) + -0.0755554113677893 * rt_powd_snf
        (r_u_r, 6.0);
      for (j = 0; j < 2; j++) {
        zn_d_l[j] /= B;
        zn_d_r[j] /= absxk;
      }

      m_out[0] = zn_d_l[0];
      m_out[1] = zn_d_l[1];
      m_out[2] = 1.0;
      mr[0] = zn_d_r[0];
      mr[1] = zn_d_r[1];
      mr[2] = 1.0;
      B = norm(m_out);
      absxk = norm(mr);
      for (j = 0; j < 3; j++) {
        m[j] = m_out[j] / B;
        m[3 + j] = mr[j] / absxk;
      }

      // TRIANGULATIONN Triangulate a point from several measurements
      //    Calculate the world position of a point given measurements of it from 
      //    different positions. The solution minimizes the least squares error
      //
      //  INPUTS:
      //  pos: a 3 x n matrix containing the positions of the cameras
      //  rot: a 4 x n matrix containing the JPL quaternions of the cameras
      //  m: a 3 x n matrix containing the unit vectors pointing towards the
      //      feature in the camera frame
      //  where n is the number of poses
      //
      //  OUTPUTS:
      //  pf: the feature position in world coordinates
      //  condition: the condition number of the least squares problem. High
      //      numbers mean a badly conditiond problem and a bad result
      memset(&A[0], 0, 30U * sizeof(double));
      for (i = 0; i < 2; i++) {
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        anchorIdx = i * 3;
        b_rot[0] = ((rot[i << 2] * rot[i << 2] - rot[1 + (i << 2)] * rot[1 + (i <<
          2)]) - rot[2 + (i << 2)] * rot[2 + (i << 2)]) + rot[3 + (i << 2)] *
          rot[3 + (i << 2)];
        b_rot[1] = 2.0 * (rot[i << 2] * rot[1 + (i << 2)] + rot[2 + (i << 2)] *
                          rot[3 + (i << 2)]);
        b_rot[2] = 2.0 * (rot[i << 2] * rot[2 + (i << 2)] - rot[1 + (i << 2)] *
                          rot[3 + (i << 2)]);
        b_rot[3] = 2.0 * (rot[i << 2] * rot[1 + (i << 2)] - rot[2 + (i << 2)] *
                          rot[3 + (i << 2)]);
        b_rot[4] = ((-(rot[i << 2] * rot[i << 2]) + rot[1 + (i << 2)] * rot[1 +
                     (i << 2)]) - rot[2 + (i << 2)] * rot[2 + (i << 2)]) + rot[3
          + (i << 2)] * rot[3 + (i << 2)];
        b_rot[5] = 2.0 * (rot[1 + (i << 2)] * rot[2 + (i << 2)] + rot[i << 2] *
                          rot[3 + (i << 2)]);
        b_rot[6] = 2.0 * (rot[i << 2] * rot[2 + (i << 2)] + rot[1 + (i << 2)] *
                          rot[3 + (i << 2)]);
        b_rot[7] = 2.0 * (rot[1 + (i << 2)] * rot[2 + (i << 2)] - rot[i << 2] *
                          rot[3 + (i << 2)]);
        b_rot[8] = ((-(rot[i << 2] * rot[i << 2]) - rot[1 + (i << 2)] * rot[1 +
                     (i << 2)]) + rot[2 + (i << 2)] * rot[2 + (i << 2)]) + rot[3
          + (i << 2)] * rot[3 + (i << 2)];
        for (j = 0; j < 3; j++) {
          A[(j + anchorIdx) + 6 * i] = 0.0;
          for (rankR = 0; rankR < 3; rankR++) {
            A[(j + anchorIdx) + 6 * i] += b_rot[j + 3 * rankR] * m[rankR + 3 * i];
          }
        }

        for (j = 0; j < 9; j++) {
          I[j] = 0;
        }

        anchorIdx = i * 3;
        for (rankR = 0; rankR < 3; rankR++) {
          I[rankR + 3 * rankR] = 1;
          for (j = 0; j < 3; j++) {
            A[(j + anchorIdx) + 6 * (2 + rankR)] = -(double)I[j + 3 * rankR];
          }
        }

        anchorIdx = i * 3;
        for (j = 0; j < 3; j++) {
          b_b[j + anchorIdx] = -pos[j + 3 * i];
        }
      }

      B = 0.0;
      svd(A, s);
      absxk = fabs(s[0]);
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

      absxk *= 6.0;
      rankR = 0;
      while ((rankR < 5) && (s[rankR] > absxk)) {
        B++;
        rankR++;
      }

      if (B < 4.0) {
        for (i = 0; i < 3; i++) {
          mr[i] = rtNaN;
        }
      } else {
        svd(A, unusedExpr);
        b_eml_xgeqp3(A, s, jpvt);
        rankR = 0;
        absxk = 6.0 * fabs(A[0]) * 2.2204460492503131E-16;
        while ((rankR < 5) && (fabs(A[rankR + 6 * rankR]) >= absxk)) {
          rankR++;
        }

        for (i = 0; i < 5; i++) {
          x[i] = 0.0;
        }

        for (j = 0; j < 5; j++) {
          if (s[j] != 0.0) {
            absxk = b_b[j];
            for (i = j + 1; i + 1 < 7; i++) {
              absxk += A[i + 6 * j] * b_b[i];
            }

            absxk *= s[j];
            if (absxk != 0.0) {
              b_b[j] -= absxk;
              for (i = j + 1; i + 1 < 7; i++) {
                b_b[i] -= A[i + 6 * j] * absxk;
              }
            }
          }
        }

        for (i = 0; i + 1 <= rankR; i++) {
          x[jpvt[i] - 1] = b_b[i];
        }

        for (j = rankR - 1; j + 1 > 0; j--) {
          x[jpvt[j] - 1] /= A[j + 6 * j];
          for (i = 0; i + 1 <= j; i++) {
            x[jpvt[i] - 1] -= x[jpvt[j] - 1] * A[i + 6 * j];
          }
        }

        for (i = 0; i < 2; i++) {
          b[i] = (x[i] < 0.0);
        }

        y = false;
        rankR = 0;
        exitg1 = false;
        while ((!exitg1) && (rankR < 2)) {
          if (!!b[rankR]) {
            y = true;
            exitg1 = true;
          } else {
            rankR++;
          }
        }

        if (y) {
          for (i = 0; i < 3; i++) {
            mr[i] = rtNaN;
          }
        } else {
          for (i = 0; i < 3; i++) {
            mr[i] = x[i + 2];
          }
        }
      }

      // transform to world coordinates
      c_xt[0] = ((b_xt->data[3] * b_xt->data[3] - b_xt->data[4] * b_xt->data[4])
                 - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] * b_xt->data[6];
      c_xt[1] = 2.0 * (b_xt->data[3] * b_xt->data[4] + b_xt->data[5] *
                       b_xt->data[6]);
      c_xt[2] = 2.0 * (b_xt->data[3] * b_xt->data[5] - b_xt->data[4] *
                       b_xt->data[6]);
      c_xt[3] = 2.0 * (b_xt->data[3] * b_xt->data[4] - b_xt->data[5] *
                       b_xt->data[6]);
      c_xt[4] = ((-(b_xt->data[3] * b_xt->data[3]) + b_xt->data[4] * b_xt->data
                  [4]) - b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] *
        b_xt->data[6];
      c_xt[5] = 2.0 * (b_xt->data[4] * b_xt->data[5] + b_xt->data[3] *
                       b_xt->data[6]);
      c_xt[6] = 2.0 * (b_xt->data[3] * b_xt->data[5] + b_xt->data[4] *
                       b_xt->data[6]);
      c_xt[7] = 2.0 * (b_xt->data[4] * b_xt->data[5] - b_xt->data[3] *
                       b_xt->data[6]);
      c_xt[8] = ((-(b_xt->data[3] * b_xt->data[3]) - b_xt->data[4] * b_xt->data
                  [4]) + b_xt->data[5] * b_xt->data[5]) + b_xt->data[6] *
        b_xt->data[6];
      for (j = 0; j < 3; j++) {
        absxk = 0.0;
        for (rankR = 0; rankR < 3; rankR++) {
          absxk += c_xt[j + 3 * rankR] * mr[rankR];
        }

        fp[j] = absxk + b_xt->data[j];
      }

      B = norm(m_out);
      for (j = 0; j < 3; j++) {
        m_out[j] /= B;
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    for (i = 0; i < 3; i++) {
      fp[i] = rtNaN;
      m_out[i] = rtNaN;
    }
  }
}

//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const double B[4]
//                double K_data[]
//                int K_size[2]
// Return Type  : void
//
static void kron(const double A_data[], const int A_size[2], const double B[4],
                 double K_data[], int K_size[2])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  K_size[0] = (signed char)(A_size[0] << 1);
  K_size[1] = (signed char)(A_size[1] << 1);
  kidx = -1;
  for (b_j1 = 1; b_j1 <= A_size[1]; b_j1++) {
    for (j2 = 0; j2 < 2; j2++) {
      for (i1 = 1; i1 <= A_size[0]; i1++) {
        for (i2 = 0; i2 < 2; i2++) {
          kidx++;
          K_data[kidx] = A_data[(i1 + A_size[0] * (b_j1 - 1)) - 1] * B[i2 + (j2 <<
            1)];
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
  emxArray_real_T *r1;
  emxArray_real_T *b_A;
  emxArray_real_T *c_A;
  unsigned int unnamed_idx_0;
  int i9;
  int loop_ub;
  double b_B_data[1296];
  int b_B_size[2];
  int A_idx_1;
  int i10;
  emxInit_real_T(&r1, 2);
  emxInit_real_T(&b_A, 2);
  emxInit_real_T(&c_A, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B_size[0] == 0) || (B_size[1] ==
        0))) {
    unnamed_idx_0 = (unsigned int)A->size[0];
    i9 = y->size[0] * y->size[1];
    y->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
    i9 = y->size[0] * y->size[1];
    y->size[1] = B_size[0];
    emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
    loop_ub = (int)unnamed_idx_0 * B_size[0];
    for (i9 = 0; i9 < loop_ub; i9++) {
      y->data[i9] = 0.0;
    }
  } else if (B_size[0] == B_size[1]) {
    eml_lusolve(B_data, B_size, A, y);
  } else {
    b_B_size[0] = B_size[1];
    b_B_size[1] = B_size[0];
    loop_ub = B_size[0];
    for (i9 = 0; i9 < loop_ub; i9++) {
      A_idx_1 = B_size[1];
      for (i10 = 0; i10 < A_idx_1; i10++) {
        b_B_data[i10 + b_B_size[0] * i9] = B_data[i9 + B_size[0] * i10];
      }
    }

    i9 = c_A->size[0] * c_A->size[1];
    c_A->size[0] = A->size[1];
    c_A->size[1] = A->size[0];
    emxEnsureCapacity((emxArray__common *)c_A, i9, (int)sizeof(double));
    loop_ub = A->size[0];
    for (i9 = 0; i9 < loop_ub; i9++) {
      A_idx_1 = A->size[1];
      for (i10 = 0; i10 < A_idx_1; i10++) {
        c_A->data[i10 + c_A->size[0] * i9] = A->data[i9 + A->size[0] * i10];
      }
    }

    loop_ub = A->size[1];
    A_idx_1 = A->size[0];
    i9 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = loop_ub;
    b_A->size[1] = A_idx_1;
    emxEnsureCapacity((emxArray__common *)b_A, i9, (int)sizeof(double));
    for (i9 = 0; i9 < A_idx_1; i9++) {
      for (i10 = 0; i10 < loop_ub; i10++) {
        b_A->data[i10 + b_A->size[0] * i9] = c_A->data[i10 + loop_ub * i9];
      }
    }

    eml_qrsolve(b_B_data, b_B_size, b_A, r1);
    i9 = y->size[0] * y->size[1];
    y->size[0] = r1->size[1];
    y->size[1] = r1->size[0];
    emxEnsureCapacity((emxArray__common *)y, i9, (int)sizeof(double));
    loop_ub = r1->size[0];
    for (i9 = 0; i9 < loop_ub; i9++) {
      A_idx_1 = r1->size[1];
      for (i10 = 0; i10 < A_idx_1; i10++) {
        y->data[i10 + y->size[0] * i9] = r1->data[i9 + r1->size[0] * i10];
      }
    }
  }

  emxFree_real_T(&c_A);
  emxFree_real_T(&b_A);
  emxFree_real_T(&r1);
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
// Arguments    : const int varargin_1_size[2]
//                const int varargin_2_size[2]
//                const int varargin_3_size[2]
//                int *nrows
//                int *ncols
// Return Type  : void
//
static void output_size(const int varargin_1_size[2], const int varargin_2_size
  [2], const int varargin_3_size[2], int *nrows, int *ncols)
{
  *nrows = (varargin_1_size[0] + varargin_2_size[0]) + varargin_3_size[0];
  *ncols = (varargin_1_size[1] + varargin_2_size[1]) + varargin_3_size[1];
}

//
// Arguments    : const double a[2]
//                double y[2]
// Return Type  : void
//
static void power(const double a[2], double y[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    y[k] = a[k] * a[k];
  }
}

//
// PREDICTMEASUREMENT Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel coordinates where a feature given in the left camera
//    frame would be visible in both cameras
// Arguments    : const double fp_l[3]
//                double h_u_l[2]
// Return Type  : void
//
static void predictMeasurement_left(const double fp_l[3], double h_u_l[2])
{
  double h_cin_l[3];
  int i;
  double radsq_l;
  double b;

  //  camera parameters for the left and right camera
  //  r_lr = cameraparams.r_lr;
  //  R_lr = cameraparams.R_lr;
  //  R_rl=R_lr';
  //  if fp_l(3) == 0
  //      ROS_ERROR('h_ci_l(3)==0')
  //  end
  //  if fp_r(3) == 0
  //      ROS_ERROR('h_ci_r(3)==0')
  //  end
  for (i = 0; i < 3; i++) {
    h_cin_l[i] = fp_l[i] / fp_l[2];
  }

  //  if any(isnan(h_cin_l))
  //      ROS_ERROR('h_cin_l')
  //      ROS_ERROR('h_ci_l(3) = %f', h_cin_l(3))
  //  end
  //  if any(isnan(h_cin_r))
  //      ROS_ERROR('h_cin_r')
  //      ROS_ERROR('h_ci_r(3) = %f', h_cin_r(3))
  //  end
  //  h_uin_l = h_cin_l(1:2);
  //  h_uin_r = h_cin_r(1:2);
  //  if any(isnan(h_uin_l))
  //      ROS_ERROR('h_uin_l')
  //  end
  //  if any(isnan(h_uin_r))
  //      ROS_ERROR('h_uin_r')
  //  end
  //  rad_l=sqrt(h_uin_l(1)^2+h_uin_l(2)^2);
  //  rad_r=sqrt(h_uin_r(1)^2+h_uin_r(2)^2);
  radsq_l = h_cin_l[0] * h_cin_l[0] + h_cin_l[1] * h_cin_l[1];
  b = ((1.0 + -0.414085141240295 * radsq_l) + 0.236451305145822 * (radsq_l *
        radsq_l)) + -0.0871296995623235 * rt_powd_snf(radsq_l, 4.0);
  for (i = 0; i < 3; i++) {
    h_cin_l[i] *= b;
  }

  //  if any(isnan(rad_l))
  //      ROS_ERROR('rad_l')
  //  end
  //  if any(isnan(rad_r))
  //      ROS_ERROR('rad_r')
  //  end
  //  if any(isnan(h_din_l))
  //      ROS_ERROR('h_din_l')
  //  end
  //  if any(isnan(h_din_r))
  //      ROS_ERROR('h_din_r')
  //  end
  h_u_l[0] = 155.972717007495 + 268.155648020127 * h_cin_l[0];
  h_u_l[1] = 113.206085625994 + 268.867732741683 * h_cin_l[1];

  //  if any(isnan(h_u_l))
  //      ROS_ERROR('h_di_l')
  //  end
  //  if any(isnan(h_u_r))
  //      ROS_ERROR('h_di_r')
  //  end
}

//
// PREDICTMEASUREMENT Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel coordinates where a feature given in the left camera
//    frame would be visible in both cameras
// Arguments    : const double fp_l[3]
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_rl[9]
//                double h_u_l[2]
//                double h_u_r[2]
// Return Type  : void
//
static void predictMeasurement_stereo(const double fp_l[3], const double
  cameraparams_r_lr[3], const double cameraparams_R_rl[9], double h_u_l[2],
  double h_u_r[2])
{
  double fp_r[3];
  int i;
  double b_fp_r;
  int i12;
  double h_cin_l[3];
  double radsq_l;
  double b;
  double radsq_r;

  //  camera parameters for the left and right camera
  //  r_lr = cameraparams.r_lr;
  //  R_lr = cameraparams.R_lr;
  //  R_rl=R_lr';
  for (i = 0; i < 3; i++) {
    b_fp_r = 0.0;
    for (i12 = 0; i12 < 3; i12++) {
      b_fp_r += cameraparams_R_rl[i + 3 * i12] * fp_l[i12];
    }

    fp_r[i] = b_fp_r - cameraparams_r_lr[i];
  }

  //  if fp_l(3) == 0
  //      ROS_ERROR('h_ci_l(3)==0')
  //  end
  //  if fp_r(3) == 0
  //      ROS_ERROR('h_ci_r(3)==0')
  //  end
  b_fp_r = fp_r[2];
  for (i = 0; i < 3; i++) {
    h_cin_l[i] = fp_l[i] / fp_l[2];
    fp_r[i] /= b_fp_r;
  }

  //  if any(isnan(h_cin_l))
  //      ROS_ERROR('h_cin_l')
  //      ROS_ERROR('h_ci_l(3) = %f', h_cin_l(3))
  //  end
  //  if any(isnan(h_cin_r))
  //      ROS_ERROR('h_cin_r')
  //      ROS_ERROR('h_ci_r(3) = %f', h_cin_r(3))
  //  end
  //  h_uin_l = h_cin_l(1:2);
  //  h_uin_r = h_cin_r(1:2);
  //  if any(isnan(h_uin_l))
  //      ROS_ERROR('h_uin_l')
  //  end
  //  if any(isnan(h_uin_r))
  //      ROS_ERROR('h_uin_r')
  //  end
  //  rad_l=sqrt(h_uin_l(1)^2+h_uin_l(2)^2);
  //  rad_r=sqrt(h_uin_r(1)^2+h_uin_r(2)^2);
  radsq_l = h_cin_l[0] * h_cin_l[0] + h_cin_l[1] * h_cin_l[1];
  b = ((1.0 + -0.414085141240295 * radsq_l) + 0.236451305145822 * (radsq_l *
        radsq_l)) + -0.0871296995623235 * rt_powd_snf(radsq_l, 4.0);
  radsq_r = fp_r[0] * fp_r[0] + fp_r[1] * fp_r[1];
  b_fp_r = ((1.0 + -0.410786366925601 * radsq_r) + 0.222940449996276 * (radsq_r *
             radsq_r)) + -0.0755554113677893 * rt_powd_snf(radsq_r, 4.0);
  for (i = 0; i < 3; i++) {
    h_cin_l[i] *= b;
    fp_r[i] *= b_fp_r;
  }

  //  if any(isnan(rad_l))
  //      ROS_ERROR('rad_l')
  //  end
  //  if any(isnan(rad_r))
  //      ROS_ERROR('rad_r')
  //  end
  //  if any(isnan(h_din_l))
  //      ROS_ERROR('h_din_l')
  //  end
  //  if any(isnan(h_din_r))
  //      ROS_ERROR('h_din_r')
  //  end
  h_u_l[0] = 155.972717007495 + 268.155648020127 * h_cin_l[0];
  h_u_l[1] = 113.206085625994 + 268.867732741683 * h_cin_l[1];
  h_u_r[0] = 167.100031218981 + 268.839577384212 * fp_r[0];
  h_u_r[1] = 107.901779803044 + 269.510643351885 * fp_r[1];

  //  if any(isnan(h_u_l))
  //      ROS_ERROR('h_di_l')
  //  end
  //  if any(isnan(h_u_r))
  //      ROS_ERROR('h_di_r')
  //  end
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
  int i4;
  int i5;
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
  for (i4 = 0; i4 < 4; i4++) {
    qp[i4] = 0.0;
    for (i5 = 0; i5 < 4; i5++) {
      qp[i4] += b_p[i4 + (i5 << 2)] * b_q[i5];
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

  if (minmn > 0) {
    tol = (double)maxmn * fabs(A_data[0]) * 2.2204460492503131E-16;
    while ((r < minmn) && (fabs(A_data[r + A_size[0] * r]) >= tol)) {
      r++;
    }
  }

  return r;
}

//
// Arguments    : const emxArray_real_T *a
//                double varargin_1
//                emxArray_real_T *b
// Return Type  : void
//
static void repmat(const emxArray_real_T *a, double varargin_1, emxArray_real_T *
                   b)
{
  int outsize_idx_0;
  int itilerow;
  int ibcol;
  int k;
  outsize_idx_0 = a->size[0] * (int)varargin_1;
  itilerow = b->size[0];
  b->size[0] = outsize_idx_0;
  emxEnsureCapacity((emxArray__common *)b, itilerow, (int)sizeof(double));
  if (!(outsize_idx_0 == 0)) {
    outsize_idx_0 = a->size[0];
    for (itilerow = 1; itilerow <= (int)varargin_1; itilerow++) {
      ibcol = (itilerow - 1) * outsize_idx_0;
      for (k = 0; k + 1 <= outsize_idx_0; k++) {
        b->data[ibcol + k] = a->data[k];
      }
    }
  }
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
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d3;
  double d4;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d3 = fabs(u0);
    d4 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d3 == 1.0) {
        y = rtNaN;
      } else if (d3 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d4 == 0.0) {
      y = 1.0;
    } else if (d4 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
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
// Arguments    : const double A[30]
//                double U[5]
// Return Type  : void
//
static void svd(const double A[30], double U[5])
{
  double b_A[30];
  double s[5];
  double e[5];
  int kase;
  double work[6];
  int q;
  int qs;
  boolean_T apply_transform;
  double ztest0;
  int ixstart;
  int m;
  double ztest;
  double rt;
  int iter;
  double snorm;
  int32_T exitg3;
  boolean_T exitg2;
  double f;
  double varargin_1[5];
  double mtmp;
  boolean_T exitg1;
  double sqds;
  memcpy(&b_A[0], &A[0], 30U * sizeof(double));
  for (kase = 0; kase < 5; kase++) {
    s[kase] = 0.0;
    e[kase] = 0.0;
  }

  for (kase = 0; kase < 6; kase++) {
    work[kase] = 0.0;
  }

  for (q = 0; q < 5; q++) {
    qs = q + 6 * q;
    apply_transform = false;
    ztest0 = d_eml_xnrm2(6 - q, b_A, qs + 1);
    if (ztest0 > 0.0) {
      apply_transform = true;
      if (b_A[qs] < 0.0) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      if (fabs(s[q]) >= 1.0020841800044864E-292) {
        ztest0 = 1.0 / s[q];
        kase = (qs - q) + 6;
        for (ixstart = qs; ixstart + 1 <= kase; ixstart++) {
          b_A[ixstart] *= ztest0;
        }
      } else {
        kase = (qs - q) + 6;
        for (ixstart = qs; ixstart + 1 <= kase; ixstart++) {
          b_A[ixstart] /= s[q];
        }
      }

      b_A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }

    for (kase = q + 1; kase + 1 < 6; kase++) {
      ixstart = q + 6 * kase;
      if (apply_transform) {
        eml_xaxpy(6 - q, -(eml_xdotc(6 - q, b_A, qs + 1, b_A, ixstart + 1) /
                           b_A[q + 6 * q]), qs + 1, b_A, ixstart + 1);
      }

      e[kase] = b_A[ixstart];
    }

    if (q + 1 <= 3) {
      ztest0 = e_eml_xnrm2(4 - q, e, q + 2);
      if (ztest0 == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -ztest0;
        } else {
          e[q] = ztest0;
        }

        ztest0 = e[q];
        if (fabs(e[q]) >= 1.0020841800044864E-292) {
          ztest0 = 1.0 / e[q];
          for (ixstart = q + 1; ixstart + 1 < 6; ixstart++) {
            e[ixstart] *= ztest0;
          }
        } else {
          for (ixstart = q + 1; ixstart + 1 < 6; ixstart++) {
            e[ixstart] /= ztest0;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (kase = q + 1; kase + 1 < 7; kase++) {
          work[kase] = 0.0;
        }

        for (kase = q + 1; kase + 1 < 6; kase++) {
          b_eml_xaxpy(5 - q, e[kase], b_A, (q + 6 * kase) + 2, work, q + 2);
        }

        for (kase = q + 1; kase + 1 < 6; kase++) {
          c_eml_xaxpy(5 - q, -e[kase] / e[q + 1], work, q + 2, b_A, (q + 6 *
            kase) + 2);
        }
      }
    }
  }

  m = 3;
  e[3] = b_A[27];
  e[4] = 0.0;
  for (q = 0; q < 5; q++) {
    ztest = e[q];
    if (s[q] != 0.0) {
      rt = fabs(s[q]);
      ztest0 = s[q] / rt;
      s[q] = rt;
      if (q + 1 < 5) {
        ztest = e[q] / ztest0;
      }
    }

    if ((q + 1 < 5) && (ztest != 0.0)) {
      rt = fabs(ztest);
      ztest0 = ztest;
      ztest = rt;
      s[q + 1] *= rt / ztest0;
    }

    e[q] = ztest;
  }

  iter = 0;
  snorm = 0.0;
  for (kase = 0; kase < 5; kase++) {
    ztest0 = fabs(s[kase]);
    ztest = fabs(e[kase]);
    if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
    } else {
      ztest0 = ztest;
    }

    if ((snorm >= ztest0) || rtIsNaN(ztest0)) {
    } else {
      snorm = ztest0;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    kase = m;
    do {
      exitg3 = 0;
      q = kase + 1;
      if (kase + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = fabs(e[kase]);
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s[kase]) + fabs(s[kase + 1])))
            || (ztest0 <= 1.0020841800044864E-292) || ((iter > 20) && (ztest0 <=
              2.2204460492503131E-16 * snorm))) {
          e[kase] = 0.0;
          exitg3 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg3 == 0);

    if (kase + 1 == m + 1) {
      kase = 4;
    } else {
      qs = m + 2;
      ixstart = m + 2;
      exitg2 = false;
      while ((!exitg2) && (ixstart >= kase + 1)) {
        qs = ixstart;
        if (ixstart == kase + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0;
          if (ixstart < m + 2) {
            ztest0 = fabs(e[ixstart - 1]);
          }

          if (ixstart > kase + 2) {
            ztest0 += fabs(e[ixstart - 2]);
          }

          ztest = fabs(s[ixstart - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s[ixstart - 1] = 0.0;
            exitg2 = true;
          } else {
            ixstart--;
          }
        }
      }

      if (qs == kase + 1) {
        kase = 3;
      } else if (qs == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0;
      for (ixstart = m; ixstart + 1 >= q + 1; ixstart--) {
        ztest0 = s[ixstart];
        eml_xrotg(&ztest0, &f, &ztest, &rt);
        s[ixstart] = ztest0;
        if (ixstart + 1 > q + 1) {
          f = -rt * e[ixstart - 1];
          e[ixstart - 1] *= ztest;
        }
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      while (q + 1 <= m + 2) {
        eml_xrotg(&s[q], &f, &ztest, &rt);
        f = -rt * e[q];
        e[q] *= ztest;
        q++;
      }
      break;

     case 3:
      varargin_1[0] = fabs(s[m + 1]);
      varargin_1[1] = fabs(s[m]);
      varargin_1[2] = fabs(e[m]);
      varargin_1[3] = fabs(s[q]);
      varargin_1[4] = fabs(e[q]);
      ixstart = 1;
      mtmp = varargin_1[0];
      if (rtIsNaN(varargin_1[0])) {
        kase = 2;
        exitg1 = false;
        while ((!exitg1) && (kase < 6)) {
          ixstart = kase;
          if (!rtIsNaN(varargin_1[kase - 1])) {
            mtmp = varargin_1[kase - 1];
            exitg1 = true;
          } else {
            kase++;
          }
        }
      }

      if (ixstart < 5) {
        while (ixstart + 1 < 6) {
          if (varargin_1[ixstart] > mtmp) {
            mtmp = varargin_1[ixstart];
          }

          ixstart++;
        }
      }

      f = s[m + 1] / mtmp;
      ztest0 = s[m] / mtmp;
      ztest = e[m] / mtmp;
      sqds = s[q] / mtmp;
      rt = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0;
      ztest0 = f * ztest;
      ztest0 *= ztest0;
      if ((rt != 0.0) || (ztest0 != 0.0)) {
        ztest = sqrt(rt * rt + ztest0);
        if (rt < 0.0) {
          ztest = -ztest;
        }

        ztest = ztest0 / (rt + ztest);
      } else {
        ztest = 0.0;
      }

      f = (sqds + f) * (sqds - f) + ztest;
      ztest0 = sqds * (e[q] / mtmp);
      for (ixstart = q + 1; ixstart <= m + 1; ixstart++) {
        eml_xrotg(&f, &ztest0, &ztest, &rt);
        if (ixstart > q + 1) {
          e[ixstart - 2] = f;
        }

        f = ztest * s[ixstart - 1] + rt * e[ixstart - 1];
        e[ixstart - 1] = ztest * e[ixstart - 1] - rt * s[ixstart - 1];
        ztest0 = rt * s[ixstart];
        s[ixstart] *= ztest;
        s[ixstart - 1] = f;
        eml_xrotg(&s[ixstart - 1], &ztest0, &ztest, &rt);
        f = ztest * e[ixstart - 1] + rt * s[ixstart];
        s[ixstart] = -rt * e[ixstart - 1] + ztest * s[ixstart];
        ztest0 = rt * e[ixstart];
        e[ixstart] *= ztest;
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
      }

      kase = q + 1;
      while ((q + 1 < 5) && (s[q] < s[kase])) {
        rt = s[q];
        s[q] = s[kase];
        s[kase] = rt;
        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (ixstart = 0; ixstart < 5; ixstart++) {
    U[ixstart] = s[ixstart];
  }
}

//
// Arguments    : double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                double dt
//                const double processNoise[4]
//                double IMU_measurements[23]
//                const double imNoise[2]
//                double numPointsPerAnchor
//                double numAnchors
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
// Return Type  : void
//
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, const double processNoise[4], double
          IMU_measurements[23], const double imNoise[2], double
          numPointsPerAnchor, double numAnchors, emxArray_real_T *h_u_apo_out,
          emxArray_real_T *xt_out, emxArray_real_T *P_apo_out, emxArray_real_T
          *map_out)
{
  double B;
  double z_n_b[3];
  int k;
  double y_n_b[3];
  int i13;
  double x_n_b[3];
  double b_x_n_b[9];
  static const double dv32[3] = { -0.414085141240295, 0.236451305145822,
    -0.0871296995623235 };

  static const double dv33[9] = { 268.155648020127, 0.0, 155.972717007495, 0.0,
    268.867732741683, 113.206085625994, 0.0, 0.0, 1.0 };

  static const double dv34[9] = { 268.155648020127, 0.0, 0.0, 0.0,
    268.867732741683, 0.0, 155.972717007495, 113.206085625994, 1.0 };

  static const char cv0[30] = { 'C', 'o', 'm', 'p', 'u', 't', 'e', 'r', ' ', 'V',
    'i', 's', 'i', 'o', 'n', ' ', 'S', 'y', 's', 't', 'e', 'm', ' ', 'T', 'o',
    'o', 'l', 'b', 'o', 'x' };

  static const char cv1[3] = { '6', '.', '2' };

  static const char cv2[8] = { '(', 'R', '2', '0', '1', '5', 'a', ')' };

  static const char cv3[11] = { '0', '9', '-', 'F', 'e', 'b', '-', '2', '0', '1',
    '5' };

  static const double dv35[3] = { -0.410786366925601, 0.222940449996276,
    -0.0755554113677893 };

  static const double dv36[9] = { 268.839577384212, 0.0, 167.100031218981, 0.0,
    269.510643351885, 107.901779803044, 0.0, 0.0, 1.0 };

  static const double dv37[9] = { 268.839577384212, 0.0, 0.0, 0.0,
    269.510643351885, 0.0, 167.100031218981, 107.901779803044, 1.0 };

  static const double dv38[9] = { 0.99986163923822, -0.00276356090334069,
    0.0164032042904086, 0.00285714926837293, 0.999979759652258,
    -0.00568480288764035, -0.0163871619848462, 0.00573088273711954,
    0.999849297596961 };

  static const double dv39[3] = { -29.5590877728364, -0.181335935104241,
    0.253273247293606 };

  static const double dv40[9] = { 3.15512205825112E-8, 3.49016833440671E-6,
    0.000299677211697942, -1.02113807360447E-5, 2.30896601934282E-6,
    -0.108449956429672, 0.000172793765317693, 0.108866982432858,
    0.732410597409066 };

  static const double dv41[9] = { 0.0022745543446244, 0.252237261907493,
    0.182760086623752, -0.738101959590127, 0.167313948053832, -29.5504827175476,
    -0.262999184633548, 29.5579713827866, 0.172371247184576 };

  static const short iv9[80] = { 0, 0, 0, 0, 0, 120, 120, 120, 120, 120, 240,
    240, 240, 240, 240, 360, 360, 360, 360, 360, 480, 480, 480, 480, 480, 600,
    600, 600, 600, 600, 720, 720, 720, 720, 720, 840, 840, 840, 840, 840, 0, 120,
    240, 360, 480, 0, 120, 240, 360, 480, 0, 120, 240, 360, 480, 0, 120, 240,
    360, 480, 0, 120, 240, 360, 480, 0, 120, 240, 360, 480, 0, 120, 240, 360,
    480, 0, 120, 240, 360, 480 };

  static const signed char iv10[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char iv11[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  static const double dv42[3] = { 0.0295590877728364, 0.000181335935104241,
    -0.000253273247293606 };

  static const double dv43[9] = { 0.99986163923822, 0.00285714926837293,
    -0.0163871619848462, -0.00276356090334069, 0.999979759652258,
    0.00573088273711954, 0.0164032042904086, -0.00568480288764035,
    0.999849297596961 };

  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  static const signed char y[9] = { 100, 0, 0, 0, 100, 0, 0, 0, 100 };

  static const double b_y[9] = { 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
    0.001 };

  double delayBuffer_k_1[6];
  double b_delayBuffer_k_1[3];
  static const double a[3] = { 0.0216, -0.4627, 0.2823 };

  double b_a[3];
  double c_a[3];
  static const double d_a[9] = { 0.0015685000000000004, 0.0070072000000001022,
    1.0000073600000001, -0.99999136000000011, -0.0089944000000001245,
    0.0016314999999999524, 0.0090056000000001135, -0.99996814,
    0.0069928000000000212 };

  int tmp_data[16];

  //  persistents for attitude estimator
  //  for coder
  // % imu hack
  // % finish imu hack
  if (!initialized_not_empty) {
    //  initialization for attitude filter
    // delayBuffer_k_1=[0;0;0;0;0;0];
    B = norm(*(double (*)[3])&IMU_measurements[3]);
    for (k = 0; k < 3; k++) {
      z_n_b[k] = IMU_measurements[k + 3] / B;
    }

    // m_n_b=IMU_measurements(11:13);
    y_n_b[0] = z_n_b[1] * 0.0 - z_n_b[2] * 0.0;
    y_n_b[1] = z_n_b[2] - z_n_b[0] * 0.0;
    y_n_b[2] = z_n_b[0] * 0.0 - z_n_b[1];
    B = norm(y_n_b);
    for (i13 = 0; i13 < 3; i13++) {
      y_n_b[i13] /= B;
    }

    x_n_b[0] = y_n_b[1] * z_n_b[2] - y_n_b[2] * z_n_b[1];
    x_n_b[1] = y_n_b[2] * z_n_b[0] - y_n_b[0] * z_n_b[2];
    x_n_b[2] = y_n_b[0] * z_n_b[1] - y_n_b[1] * z_n_b[0];
    B = norm(x_n_b);
    for (i13 = 0; i13 < 3; i13++) {
      b_x_n_b[i13] = x_n_b[i13] / B;
      b_x_n_b[3 + i13] = y_n_b[i13];
      b_x_n_b[6 + i13] = z_n_b[i13];
    }

    QuatFromRotJ(b_x_n_b, x_att);
    memset(&P_att[0], 0, 9U * sizeof(double));

    //  other initialization
    initialized_not_empty = true;
    for (k = 0; k < 3; k++) {
      P_att[k + 3 * k] = 1.0;

      //  Autogenerated function that constructs and returns a hard coded struct. 
      //  Generated on 07-Aug-2015 09:51:40.
      cameraparams.CameraParameters1.RadialDistortion[k] = dv32[k];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.CameraParameters1.TangentialDistortion[i13] = 0.0;
    }

    cameraparams.CameraParameters1.EstimateSkew = false;
    cameraparams.CameraParameters1.NumRadialDistortionCoefficients = 3.0;
    cameraparams.CameraParameters1.EstimateTangentialDistortion = false;
    cameraparams.CameraParameters1.NumPatterns = 103.0;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.CameraParameters1.IntrinsicMatrix[i13] = dv33[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.CameraParameters1.FocalLength[i13] = 268.155648020127 +
        0.71208472155603886 * (double)i13;
      cameraparams.CameraParameters1.PrincipalPoint[i13] = 155.972717007495 +
        -42.766631381500986 * (double)i13;
    }

    cameraparams.CameraParameters1.Skew = 0.0;
    cameraparams.CameraParameters1.MeanReprojectionError = 0.178947558747657;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.CameraParameters1.IntrinsicMatrixInternal[i13] = dv34[i13];
    }

    for (i13 = 0; i13 < 30; i13++) {
      cameraparams.CameraParameters1.Version.Name[i13] = cv0[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.CameraParameters1.Version.Version[i13] = cv1[i13];
    }

    for (i13 = 0; i13 < 8; i13++) {
      cameraparams.CameraParameters1.Version.Release[i13] = cv2[i13];
    }

    for (i13 = 0; i13 < 11; i13++) {
      cameraparams.CameraParameters1.Version.Date[i13] = cv3[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.CameraParameters2.RadialDistortion[i13] = dv35[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.CameraParameters2.TangentialDistortion[i13] = 0.0;
    }

    cameraparams.CameraParameters2.EstimateSkew = false;
    cameraparams.CameraParameters2.NumRadialDistortionCoefficients = 3.0;
    cameraparams.CameraParameters2.EstimateTangentialDistortion = false;
    cameraparams.CameraParameters2.NumPatterns = 103.0;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.CameraParameters2.IntrinsicMatrix[i13] = dv36[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.CameraParameters2.FocalLength[i13] = 268.839577384212 +
        0.67106596767297333 * (double)i13;
      cameraparams.CameraParameters2.PrincipalPoint[i13] = 167.100031218981 +
        -59.198251415937008 * (double)i13;
    }

    cameraparams.CameraParameters2.Skew = 0.0;
    cameraparams.CameraParameters2.MeanReprojectionError = 0.176667688316263;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.CameraParameters2.IntrinsicMatrixInternal[i13] = dv37[i13];
    }

    for (i13 = 0; i13 < 30; i13++) {
      cameraparams.CameraParameters2.Version.Name[i13] = cv0[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.CameraParameters2.Version.Version[i13] = cv1[i13];
    }

    for (i13 = 0; i13 < 8; i13++) {
      cameraparams.CameraParameters2.Version.Release[i13] = cv2[i13];
    }

    for (i13 = 0; i13 < 11; i13++) {
      cameraparams.CameraParameters2.Version.Date[i13] = cv3[i13];
    }

    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.RotationOfCamera2[i13] = dv38[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.TranslationOfCamera2[i13] = dv39[i13];
    }

    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.FundamentalMatrix[i13] = dv40[i13];
      cameraparams.EssentialMatrix[i13] = dv41[i13];
    }

    cameraparams.MeanReprojectionError = 0.17780762353196;
    cameraparams.NumPatterns = 103.0;
    for (i13 = 0; i13 < 80; i13++) {
      cameraparams.WorldPoints[i13] = iv9[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.WorldUnits[i13] = 'm';
    }

    for (i13 = 0; i13 < 4; i13++) {
      cameraparams.RectifyMap1.Xmap[i13] = 0.0;
      cameraparams.RectifyMap1.Ymap[i13] = 0.0;
      cameraparams.RectifyMap1.XmapSingle[i13] = 0.0;
      cameraparams.RectifyMap1.YmapSingle[i13] = 0.0;
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.RectifyMap1.NewOrigin[i13] = 0.0;
    }

    for (i13 = 0; i13 < 4; i13++) {
      cameraparams.RectifyMap2.Xmap[i13] = 0.0;
      cameraparams.RectifyMap2.Ymap[i13] = 0.0;
      cameraparams.RectifyMap2.XmapSingle[i13] = 0.0;
      cameraparams.RectifyMap2.YmapSingle[i13] = 0.0;
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.RectifyMap2.NewOrigin[i13] = 0.0;
    }

    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.RectificationParams.H1.T[i13] = iv10[i13];
    }

    cameraparams.RectificationParams.H1.Dimensionality = 2.0;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.RectificationParams.H2.T[i13] = iv10[i13];
    }

    cameraparams.RectificationParams.H2.Dimensionality = 2.0;
    for (i13 = 0; i13 < 16; i13++) {
      cameraparams.RectificationParams.Q[i13] = iv11[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.RectificationParams.XBounds[i13] = 0.0;
      cameraparams.RectificationParams.YBounds[i13] = 0.0;
    }

    cameraparams.RectificationParams.Initialized = true;
    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.RectificationParams.RectifiedImageSize[i13] = 1.0;
    }

    for (i13 = 0; i13 < 30; i13++) {
      cameraparams.Version.Name[i13] = cv0[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.Version.Version[i13] = cv1[i13];
    }

    for (i13 = 0; i13 < 8; i13++) {
      cameraparams.Version.Release[i13] = cv2[i13];
    }

    for (i13 = 0; i13 < 11; i13++) {
      cameraparams.Version.Date[i13] = cv3[i13];
    }

    for (k = 0; k < 3; k++) {
      cameraparams.r_lr[k] = dv42[k];
    }

    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.R_lr[i13] = dv38[i13];
      cameraparams.R_rl[i13] = dv43[i13];
    }

    b_emxInit_real_T(&r4, 1);
    i13 = r4->size[0];
    r4->size[0] = 7 + (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)r4, i13, (int)sizeof(double));
    r4->data[0] = 0.0;
    r4->data[1] = 0.0;
    r4->data[2] = 0.0;
    r4->data[3] = 0.0;
    r4->data[4] = 0.0;
    r4->data[5] = 0.0;
    r4->data[6] = 1.0;
    k = (int)numPointsPerAnchor;
    for (i13 = 0; i13 < k; i13++) {
      r4->data[i13 + 7] = 0.0;
    }

    b_emxInit_real_T(&r5, 1);
    repmat(r4, numAnchors, r5);
    i13 = xt->size[0];
    xt->size[0] = 13 + r5->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i13, (int)sizeof(double));
    xt->data[0] = 0.0;
    xt->data[1] = 0.0;
    xt->data[2] = 0.0;
    xt->data[3] = 0.0;
    xt->data[4] = 0.0;
    xt->data[5] = 0.0;
    xt->data[6] = 1.0;
    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    emxFree_real_T(&r4);
    for (i13 = 0; i13 < 3; i13++) {
      xt->data[i13 + 10] = IMU_measurements[i13];
    }

    k = r5->size[0];
    for (i13 = 0; i13 < k; i13++) {
      xt->data[i13 + 13] = r5->data[i13];
    }

    emxFree_real_T(&r5);
    emxInit_real_T(&r6, 2);

    //  initial real vector
    B = numAnchors * (6.0 + numPointsPerAnchor);
    i13 = r6->size[0] * r6->size[1];
    r6->size[0] = (int)numStates;
    r6->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r6, i13, (int)sizeof(double));
    k = (int)numStates * (int)numStates;
    for (i13 = 0; i13 < k; i13++) {
      r6->data[i13] = 0.0;
    }

    emxInit_real_T(&r7, 2);
    i13 = r7->size[0] * r7->size[1];
    r7->size[0] = (int)B;
    r7->size[1] = (int)B;
    emxEnsureCapacity((emxArray__common *)r7, i13, (int)sizeof(double));
    k = (int)B * (int)B;
    for (i13 = 0; i13 < k; i13++) {
      r7->data[i13] = 0.0;
    }

    blkdiag(r6, r7, P);

    //  initial error state covariance
    emxFree_real_T(&r7);
    emxFree_real_T(&r6);
    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[k + P->size[0] * i13] = 0.0;
      }
    }

    //  position
    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[(k + P->size[0] * (3 + i13)) + 3] = y[k + 3 * i13];
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[(k + P->size[0] * (6 + i13)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[(k + P->size[0] * (9 + i13)) + 9] = b_y[k + 3 * i13];
      }
    }

    //  gyro bias
  }

  for (k = 0; k < 6; k++) {
    delayBuffer_k_1[k] = delayBuffer_k_0[k];
  }

  for (k = 0; k < 3; k++) {
    delayBuffer_k_0[k] = IMU_measurements[k + 13];
  }

  for (k = 0; k < 3; k++) {
    delayBuffer_k_0[k + 3] = IMU_measurements[k + 16];
    b_delayBuffer_k_1[k] = delayBuffer_k_1[k] + a[k];
  }

  for (i13 = 0; i13 < 3; i13++) {
    b_a[i13] = 0.0;
    for (k = 0; k < 3; k++) {
      b_a[i13] += d_a[i13 + 3 * k] * b_delayBuffer_k_1[k];
    }

    IMU_measurements[i13] = b_a[i13];
    c_a[i13] = 0.0;
    for (k = 0; k < 3; k++) {
      c_a[i13] += d_a[i13 + 3 * k] * delayBuffer_k_1[3 + k];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    IMU_measurements[3 + i13] = c_a[i13];
  }

  if (init_counter == 0.0) {
    //  done initializing attitude. Insert the estimated attitude and the covariance into the whole state, request features 
    b_fprintf();
    for (k = 0; k < 16; k++) {
      updateVect[k] = 0.0;
    }

    if (1.0 > numPointsPerAnchor) {
      k = 0;
    } else {
      k = (int)numPointsPerAnchor;
    }

    for (i13 = 0; i13 < k; i13++) {
      tmp_data[i13] = i13;
    }

    for (i13 = 0; i13 < k; i13++) {
      updateVect[tmp_data[i13]] = 2.0;
    }

    for (i13 = 0; i13 < 4; i13++) {
      xt->data[3 + i13] = x_att[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[(k + P->size[0] * (3 + i13)) + 3] = P_att[k + 3 * i13];
      }
    }

    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    k = xt->size[0];
    for (i13 = 0; i13 < k; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    for (i13 = 0; i13 < 4; i13++) {
      xt_out->data[3 + i13] = x_att[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i13 = 0; i13 < k; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }

    i13 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i13, (int)sizeof(double));
    for (i13 = 0; i13 < 64; i13++) {
      h_u_apo_out->data[i13] = -100.0;
    }

    i13 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, i13, (int)sizeof(double));
    for (i13 = 0; i13 < 48; i13++) {
      map_out->data[i13] = rtNaN;
    }

    init_counter = 1.0;
  } else if (init_counter == 1.0) {
    SLAM_updIT(P, xt, cameraparams.r_lr, cameraparams.R_lr, cameraparams.R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, (1.0 - rt_powd_snf
                (IMU_measurements[9] / 101325.0, 0.190284)) * 145366.45,
               h_u_apo_out, map_out);
    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    k = xt->size[0];
    for (i13 = 0; i13 < k; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i13 = 0; i13 < k; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }

    init_counter = 2.0;
  } else {
    SLAM_pred(P, xt, dt, processNoise, IMU_measurements, numStates);

    //  [xt,P] =  SLAM_pred_euler(P, xt, dt, processNoise, IMU_measurements, numStates); 
    SLAM_updIT(P, xt, cameraparams.r_lr, cameraparams.R_lr, cameraparams.R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, 0.0, h_u_apo_out, map_out);
    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    k = xt->size[0];
    for (i13 = 0; i13 < k; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i13 = 0; i13 < k; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }

    // % output asserts for coder
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
  useAirPressure = b_useAirPressure;
  gravAlignNoise = b_gravAlignNoise;
  gravityUpdate = b_gravityUpdate;
  maxEKFIterations = b_maxEKFIterations;
  sigma_Init = b_sigma_Init;
  minFeatureThreshold = b_minFeatureThreshold;
  numStatesxt = b_numStatesxt;
  numTrackFeatures = b_numTrackFeatures;
  numStates = b_numStates;
  initialized_not_empty = false;
  SLAM_init();
  SLAM_updIT_init();
  c_eml_rand_mt19937ar_stateful_i();
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
