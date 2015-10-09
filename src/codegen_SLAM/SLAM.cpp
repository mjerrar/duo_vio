//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 09-Oct-2015 14:25:26
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include <ros/console.h>
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

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

typedef struct {
  double pos[3];
  double att[4];
  double gyro_bias[3];
  double acc_bias[3];
} b_struct_T;

typedef struct {
  b_struct_T IMU;
  double pos[3];
  double att[4];
  double vel[3];
} c_struct_T;

typedef struct {
  int anchor_idx;
  double pos[3];
  double att[4];
} d_struct_T;

typedef struct {
  double inverse_depth;
  double m[3];
  double scaled_map_point[3];
  int status;
  int status_idx;
  int P_idx;
} e_struct_T;

typedef struct {
  double pos[3];
  double att[4];
  int P_idx[6];
  e_struct_T feature_states[8];
} f_struct_T;

typedef struct {
  c_struct_T robot_state;
  int fixed_feature;
  d_struct_T origin;
  f_struct_T anchor_states[5];
} g_struct_T;

typedef struct {
  double pos[3];
  double att[4];
} struct_T;

// Named Constants
#define b_debug_level                  (2.0)

// Variable Definitions
static boolean_T initialized_not_empty;
static g_struct_T xt;
static double P[8281];
static double h_u[160];
static double map[120];
static double delayedStatus[40];
static double debug_level;

// Function Declarations
static void OnePointRANSAC_EKF(g_struct_T *b_xt, double b_P[8281], const double
  z_u_l[80], const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], double noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_RANSAC, int updateVect[40]);
static void QuatFromRotJ(const double R[9], double Q[4]);
static void SLAM_pred_euler(double P_apo[8281], g_struct_T *x, double dt, double
  processNoise_qv, double processNoise_qw, double processNoise_qao, double
  processNoise_qwo, double processNoise_qR_ci, const double
  measurements_acc_duo[3], const double measurements_gyr_duo[3]);
static void SLAM_upd(double P_apr[8281], g_struct_T *b_xt, const double
                     c_cameraParams_CameraParameters[2], const double
                     d_cameraParams_CameraParameters[2], const double
                     e_cameraParams_CameraParameters[3], const double
                     f_cameraParams_CameraParameters[2], const double
                     g_cameraParams_CameraParameters[2], const double
                     h_cameraParams_CameraParameters[3], const double
                     cameraParams_r_lr[3], const double cameraParams_R_lr[9],
                     const double cameraParams_R_rl[9], int updateVect[40],
                     double z_all_l[80], double z_all_r[80], double
                     noiseParameters_image_noise, double
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, double h_u_apo[160], double b_map[120],
                     double b_delayedStatus[40]);
static boolean_T any(const boolean_T x[40]);
static boolean_T anyActiveAnchorFeatures(const e_struct_T
  anchor_state_feature_states[8]);
static boolean_T b_any(const boolean_T x[3]);
static void b_diag(const double v[15], double d[225]);
static void b_eye(double I[441]);
static double b_fprintf();
static void b_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void b_mrdivide(const double A[2], const double B[4], double y[2]);
static double b_norm(const double x[4]);
static void b_ros_error();
static void b_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void b_sort(double x[8], int idx[8]);
static void b_xgeqp3(double A[30], double tau[5], int jpvt[5]);
static double b_xnrm2(int n, const emxArray_real_T *x, int ix0);
static boolean_T c_any(const double x[5]);
static void c_eye(double I[8281]);
static double c_fprintf(double varargin_1);
static double c_norm(const double x[2]);
static void c_predictMeasurementStereoDisto(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[3], const double
  f_stereoParams_CameraParameters[2], const double
  g_stereoParams_CameraParameters[2], const double
  h_stereoParams_CameraParameters[3], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_d_l[2], double h_d_r[2]);
static void c_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void c_sort(emxArray_real_T *x, emxArray_int32_T *idx);
static double c_xnrm2(int n, const double x[30], int ix0);
static void cast(const struct_T x[5], AnchorPose y[5]);
static void cross(const double a[3], const double b[3], double c[3]);
static void d_eye(double I[8281]);
static double d_fprintf(double varargin_1);
static void d_ros_info(int varargin_1, int varargin_2);
static void d_sort(emxArray_real_T *x, int dim, emxArray_int32_T *idx);
static double det(const double x[36]);
static void diag(const double v[3], double d[9]);
static double e_fprintf(double varargin_1);
static void e_ros_info(int varargin_1);
static void eml_signed_integer_colon(int b, emxArray_int32_T *y);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_int32_T1(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);
static void emxInit_real_T1(emxArray_real_T **pEmxArray, int b_numDimensions);
static void eye(double I[9]);
static double f_fprintf(double varargin_1);
static void f_ros_info(int varargin_1);
static void fileManager(FILE * *f, boolean_T *a);
static double g_fprintf(double varargin_1);
static void g_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void getAnchorPoses(const double xt_origin_pos[3], const double
  xt_origin_att[4], const f_struct_T xt_anchor_states[5], struct_T anchor_poses
  [5]);
static void getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[5], const double
  z_all_l[80], const boolean_T b_status[40], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double noiseParameters_image_noise, emxArray_real_T *r, emxArray_real_T *H,
  emxArray_real_T *R);
static void getMap(const double xt_origin_pos[3], const double xt_origin_att[4],
                   const f_struct_T xt_anchor_states[5], double b_map[120]);
static double getNumValidFeatures(const e_struct_T anchor_state_feature_states[8]);
static void getScaledMap(g_struct_T *b_xt);
static double getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[5]);
static double getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[5]);
static void getWorldState(const double xt_robot_state_IMU_pos[3], const double
  xt_robot_state_IMU_att[4], const double xt_robot_state_IMU_gyro_bias[3], const
  double xt_robot_state_IMU_acc_bias[3], const double xt_robot_state_pos[3],
  const double xt_robot_state_att[4], const double xt_origin_pos[3], const
  double xt_origin_att[4], double world_state_pos[3], double world_state_att[4],
  double world_state_vel[3], double world_state_IMU_gyro_bias[3], double
  world_state_IMU_acc_bias[3], double world_state_IMU_pos[3], double
  world_state_IMU_att[4]);
static double h_fprintf(double varargin_1, double varargin_2, double varargin_3);
static void h_ros_info(int varargin_1, double varargin_2, double varargin_3);
static double i_fprintf(double varargin_1, double varargin_2, double varargin_3);
static void i_ros_info(int varargin_1);
static void initializePoint(const double z_u_l[2], const double z_u_r[2], const
  double c_cameraparams_CameraParameters[2], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const double
  f_cameraparams_CameraParameters[2], const double cameraparams_r_lr[3], const
  double cameraparams_R_lr[9], double fp[3], double b_m[6], boolean_T *success);
static double j_fprintf(double varargin_1);
static double k_fprintf(double varargin_1);
static double l_fprintf(int varargin_1);
static void lusolve(const emxArray_real_T *A, emxArray_real_T *B);
static double m_fprintf(int varargin_1);
static double median(const double x_data[], const int x_size[1]);
static void merge(int idx[8], double x[8], int offset, int np, int nq, int
                  iwork[8], double xwork[8]);
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void mrdivide(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *y);
static void multiplyIdx(const double idx_data[], const int idx_size[1], double
  idx_mult_data[], int idx_mult_size[1]);
static double n_fprintf(int varargin_1);
static double norm(const double x[3]);
static double o_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static double p_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void predictMeasurementDistortedPB(const double fp[3], const double
  cameraParams_FocalLength[2], const double cameraParams_PrincipalPoint[2],
  const double cameraParams_RadialDistortion[3], double h_d[2]);
static void predictMeasurementMono(const double fp[3], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double px[2]);
static void predictMeasurementStereo(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_cin_l[2], double h_cin_r[2]);
static void printParams(double c_noiseParameters_process_noise, double
  d_noiseParameters_process_noise, double e_noiseParameters_process_noise,
  double f_noiseParameters_process_noise, double g_noiseParameters_process_noise,
  const double c_noiseParameters_gyro_bias_ini[3], const double
  c_noiseParameters_acc_bias_init[3], double noiseParameters_image_noise, double
  c_noiseParameters_inv_depth_ini, int c_VIOParameters_num_points_per_, int
  VIOParameters_num_anchors, int c_VIOParameters_max_ekf_iterati, boolean_T
  VIOParameters_fixed_feature, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_mono, boolean_T VIOParameters_RANSAC);
static double q_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void quatPlusThetaJ(const double dtheta[3], double dq[4]);
static void quatmultJ(const double q[4], const double p[4], double qp[4]);
static double r_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void rdivide(const double x[3], double y, double z[3]);
static void ros_error();
static void ros_info(int varargin_1, int varargin_2, int varargin_3);
static double rt_powd_snf(double u0, double u1);
static void sort(double x[8], int idx[8]);
static void sortIdx(emxArray_real_T *x, emxArray_int32_T *idx);
static void undistortPoint(const double pt_d_data[], const int pt_d_size[1],
  const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double cameraparams_RadialDistortion[3],
  double pt_u_data[], int pt_u_size[1]);
static void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T
                   *jpvt);
static double xnrm2(int n, const emxArray_real_T *x, int ix0);
static void xscal(int n, double a, emxArray_real_T *x, int ix0);

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : g_struct_T *b_xt
//                double b_P[8281]
//                const double z_u_l[80]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                double noiseParameters_image_noise
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_RANSAC
//                int updateVect[40]
// Return Type  : void
//
static void OnePointRANSAC_EKF(g_struct_T *b_xt, double b_P[8281], const double
  z_u_l[80], const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], double noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_RANSAC, int updateVect[40])
{
  int ii;
  boolean_T activeFeatures[40];
  boolean_T delayedFeatures[40];
  int anchorIdx;
  int featureIdx;
  boolean_T LI_inlier_status[40];
  emxArray_real_T *S;
  emxArray_real_T *K;
  emxArray_real_T *r;
  emxArray_real_T *H;
  emxArray_real_T *R;
  int idx;
  signed char ii_data[40];
  boolean_T exitg2;
  boolean_T guard2 = false;
  int loop_ub;
  int i40;
  signed char hyp_ind_data[40];
  double num_hyp;
  int hyp_it;
  emxArray_real_T *C;
  emxArray_real_T *y;
  emxArray_real_T *b_y;
  emxArray_real_T *b_C;
  boolean_T LI_inlier_status_i[40];
  boolean_T hyp_status[40];
  unsigned int H_idx_0;
  int b_m;
  int br;
  int i41;
  int ic;
  int ar;
  int ib;
  int ia;
  double a[2];
  double x_apo[91];
  double b_x_apo;
  double dv17[4];
  double c_xt[4];
  double R_cw[9];
  double r_wc[3];
  double anchorPos[3];
  double c_x_apo[3];
  double dv18[4];
  double anchorRot[9];
  double rho;
  long i42;
  double z_u[2];
  double b_rho[3];
  double d9;
  double b_r[2];
  double dv19[2];
  int k;
  emxArray_real_T *c_y;
  emxArray_real_T *d_y;
  emxArray_real_T *b_S;
  double dv20[4];
  static double e_y[8281];
  static double dv21[8281];
  static double dv22[8281];
  static double dv23[8281];
  boolean_T b_activeFeatures;
  boolean_T exitg1;
  boolean_T guard1 = false;
  int ii_size_idx_0;
  emxArray_real_T *b_K;
  emxArray_real_T *b_H;
  int it;
  emxArray_real_T *c_C;
  emxArray_real_T *d_C;
  emxArray_real_T *f_y;
  emxArray_real_T *g_y;
  emxArray_real_T *h_y;
  emxArray_real_T *e_C;
  emxArray_real_T *c_H;
  emxArray_real_T *d_H;
  emxArray_real_T *e_H;
  double x_apo_data[91];
  double b_x_apo_data;
  double dv24[4];
  emxArray_real_T *f_C;
  double dv25[8281];
  emxArray_real_T *i_y;
  emxArray_real_T *g_C;
  emxArray_real_T *j_y;
  emxArray_real_T *h_C;
  double dv26[8281];

  //  HI mahalanobis gate
  for (ii = 0; ii < 40; ii++) {
    activeFeatures[ii] = false;
    delayedFeatures[ii] = false;
  }

  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status == 1)
      {
        activeFeatures[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
          .status_idx - 1] = true;
      } else {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          delayedFeatures[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = true;
        }
      }
    }
  }

  for (ii = 0; ii < 40; ii++) {
    LI_inlier_status[ii] = false;
    activeFeatures[ii] = (activeFeatures[ii] && (updateVect[ii] == 1));
    delayedFeatures[ii] = (delayedFeatures[ii] && (updateVect[ii] == 1));
  }

  // % B 1-point hypotheses generation and evaluation
  emxInit_real_T1(&S, 2);
  emxInit_real_T1(&K, 2);
  emxInit_real_T(&r, 1);
  emxInit_real_T1(&H, 2);
  emxInit_real_T1(&R, 2);
  if (VIOParameters_RANSAC) {
    //  build the map according to the current estimate
    getScaledMap(b_xt);

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

    for (i40 = 0; i40 < loop_ub; i40++) {
      hyp_ind_data[i40] = ii_data[i40];
    }

    //  hyp_ind = hyp_ind(randperm(length(hyp_ind)));
    if (1 > idx) {
      num_hyp = 0.0;
    } else {
      num_hyp = idx;
    }

    hyp_it = 1;
    emxInit_real_T1(&C, 2);
    emxInit_real_T1(&y, 2);
    emxInit_real_T1(&b_y, 2);
    emxInit_real_T1(&b_C, 2);
    while ((hyp_it < num_hyp) && (hyp_it < loop_ub)) {
      for (ii = 0; ii < 40; ii++) {
        LI_inlier_status_i[ii] = false;
        hyp_status[ii] = false;
      }

      hyp_status[hyp_ind_data[hyp_it - 1] - 1] = true;

      //  used to signal which feature to compute the derivatives for
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, hyp_status,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, R);
      H_idx_0 = (unsigned int)H->size[0];
      i40 = y->size[0] * y->size[1];
      y->size[0] = (int)H_idx_0;
      y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)y, i40, (int)sizeof(double));
      b_m = H->size[0];
      i40 = y->size[0] * y->size[1];
      y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)y, i40, (int)sizeof(double));
      for (i40 = 0; i40 < 91; i40++) {
        br = y->size[0];
        for (i41 = 0; i41 < br; i41++) {
          y->data[i41 + y->size[0] * i40] = 0.0;
        }
      }

      if (H->size[0] == 0) {
      } else {
        ii = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i40 = idx + b_m;
          for (ic = idx; ic + 1 <= i40; ic++) {
            y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i40 = idx + b_m;
              for (ic = idx; ic + 1 <= i40; ic++) {
                ia++;
                y->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i40 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
      br = H->size[0];
      for (i40 = 0; i40 < br; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          K->data[i41 + K->size[0] * i40] = H->data[i40 + H->size[0] * i41];
        }
      }

      a[0] = (unsigned int)y->size[0];
      a[1] = (unsigned int)K->size[1];
      i40 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i40, (int)sizeof(double));
      b_m = y->size[0];
      i40 = C->size[0] * C->size[1];
      emxEnsureCapacity((emxArray__common *)C, i40, (int)sizeof(double));
      br = C->size[1];
      for (i40 = 0; i40 < br; i40++) {
        ii = C->size[0];
        for (i41 = 0; i41 < ii; i41++) {
          C->data[i41 + C->size[0] * i40] = 0.0;
        }
      }

      if ((y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        ii = y->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i40 = idx + b_m;
          for (ic = idx; ic + 1 <= i40; ic++) {
            C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i40 = idx + b_m;
              for (ic = idx; ic + 1 <= i40; ic++) {
                ia++;
                C->data[ic] += K->data[ib] * y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i40 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
      br = H->size[0];
      for (i40 = 0; i40 < br; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          K->data[i41 + K->size[0] * i40] = H->data[i40 + H->size[0] * i41];
        }
      }

      H_idx_0 = (unsigned int)K->size[1];
      i40 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = 91;
      b_y->size[1] = (int)H_idx_0;
      b_y->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)b_y, i40, (int)sizeof(double));
      br = b_y->size[1];
      for (i40 = 0; i40 < br; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          b_y->data[i41 + b_y->size[0] * i40] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        ii = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= ii; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            b_y->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                b_y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      i40 = b_C->size[0] * b_C->size[1];
      b_C->size[0] = C->size[0];
      b_C->size[1] = C->size[1];
      emxEnsureCapacity((emxArray__common *)b_C, i40, (int)sizeof(double));
      br = C->size[0] * C->size[1];
      for (i40 = 0; i40 < br; i40++) {
        b_C->data[i40] = C->data[i40] + R->data[i40];
      }

      mrdivide(b_y, b_C, K);
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i40 = 0; i40 < 91; i40++) {
          x_apo[i40] = 0.0;
          br = K->size[1];
          for (i41 = 0; i41 < br; i41++) {
            b_x_apo = x_apo[i40] + K->data[i40 + K->size[0] * i41] * r->data[i41];
            x_apo[i40] = b_x_apo;
          }
        }
      } else {
        memset(&x_apo[0], 0, 91U * sizeof(double));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic < 91; ic++) {
              ia++;
              b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
              x_apo[ic] = b_x_apo;
            }
          }

          ar += 91;
        }
      }

      quatPlusThetaJ(*(double (*)[3])&x_apo[3], dv17);
      quatmultJ(dv17, b_xt->robot_state.att, c_xt);

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
      R_cw[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];
      R_cw[7] = 2.0 * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
      R_cw[2] = 2.0 * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
      R_cw[5] = 2.0 * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
      R_cw[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];
      for (ii = 0; ii < 3; ii++) {
        r_wc[ii] = b_xt->robot_state.pos[ii] + x_apo[ii];
      }

      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        i40 = anchorIdx * 14;
        i41 = anchorIdx * 14;
        for (idx = 0; idx < 3; idx++) {
          anchorPos[idx] = b_xt->anchor_states[anchorIdx].pos[idx] + x_apo[(idx
            + i40) + 21];
          c_x_apo[idx] = x_apo[(idx + i41) + 24];
        }

        quatPlusThetaJ(c_x_apo, dv18);
        quatmultJ(dv18, b_xt->anchor_states[anchorIdx].att, c_xt);

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        anchorRot[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] *
                        c_xt[2]) + c_xt[3] * c_xt[3];
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
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            rho = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth + x_apo[(anchorIdx * 14 + featureIdx) + 27];
            i42 = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              status_idx - 1L;
            if (i42 > 2147483647L) {
              i42 = 2147483647L;
            } else {
              if (i42 < -2147483648L) {
                i42 = -2147483648L;
              }
            }

            i40 = (int)i42;
            if (i40 > 1073741823) {
              ii = MAX_int32_T;
            } else if (i40 <= -1073741824) {
              ii = MIN_int32_T;
            } else {
              ii = i40 << 1;
            }

            for (i40 = 0; i40 < 2; i40++) {
              i42 = (long)ii + (1 + i40);
              if (i42 > 2147483647L) {
                i42 = 2147483647L;
              } else {
                if (i42 < -2147483648L) {
                  i42 = -2147483648L;
                }
              }

              z_u[i40] = z_u_l[(int)i42 - 1];
            }

            for (i40 = 0; i40 < 3; i40++) {
              d9 = 0.0;
              for (i41 = 0; i41 < 3; i41++) {
                d9 += anchorRot[i41 + 3 * i40] * b_xt->anchor_states[anchorIdx].
                  feature_states[featureIdx].m[i41];
              }

              b_rho[i40] = (rho * anchorPos[i40] + d9) - r_wc[i40] * rho;
            }

            for (i40 = 0; i40 < 3; i40++) {
              c_x_apo[i40] = 0.0;
              for (i41 = 0; i41 < 3; i41++) {
                c_x_apo[i40] += R_cw[i40 + 3 * i41] * b_rho[i41];
              }
            }

            predictMeasurementMono(c_x_apo, cameraparams_FocalLength,
              cameraparams_PrincipalPoint, b_r);
            for (ii = 0; ii < 2; ii++) {
              dv19[ii] = b_r[ii] - z_u[ii];
            }

            if (c_norm(dv19) < 3.0) {
              LI_inlier_status_i[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = true;
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
        for (k = 0; k < 40; k++) {
          LI_inlier_status[k] = LI_inlier_status_i[k];
          if (LI_inlier_status_i[k]) {
            ii++;
          }

          if (activeFeatures[k]) {
            idx++;
          }
        }

        num_hyp = -4.60517018598809 / log(1.0 - (double)ii / (double)idx);
      }

      hyp_it++;
    }

    emxFree_real_T(&b_C);
    emxFree_real_T(&b_y);
    emxFree_real_T(&y);
    emxFree_real_T(&C);

    //  ros_info('Found %i LI inliers in %i active features', nnz(LI_inlier_status), nnz(activeFeatures)) 
    ii = 0;
    for (k = 0; k < 40; k++) {
      if (LI_inlier_status[k]) {
        ii++;
      }
    }

    if (ii > 3) {
      emxInit_real_T1(&c_y, 2);
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, LI_inlier_status,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, R);
      H_idx_0 = (unsigned int)H->size[0];
      i40 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = (int)H_idx_0;
      c_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)c_y, i40, (int)sizeof(double));
      b_m = H->size[0];
      i40 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)c_y, i40, (int)sizeof(double));
      for (i40 = 0; i40 < 91; i40++) {
        loop_ub = c_y->size[0];
        for (i41 = 0; i41 < loop_ub; i41++) {
          c_y->data[i41 + c_y->size[0] * i40] = 0.0;
        }
      }

      if (H->size[0] == 0) {
      } else {
        ii = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i40 = idx + b_m;
          for (ic = idx; ic + 1 <= i40; ic++) {
            c_y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i40 = idx + b_m;
              for (ic = idx; ic + 1 <= i40; ic++) {
                ia++;
                c_y->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i40 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i40 = 0; i40 < loop_ub; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          K->data[i41 + K->size[0] * i40] = H->data[i40 + H->size[0] * i41];
        }
      }

      a[0] = (unsigned int)c_y->size[0];
      a[1] = (unsigned int)K->size[1];
      i40 = S->size[0] * S->size[1];
      S->size[0] = (int)a[0];
      S->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)S, i40, (int)sizeof(double));
      b_m = c_y->size[0];
      i40 = S->size[0] * S->size[1];
      emxEnsureCapacity((emxArray__common *)S, i40, (int)sizeof(double));
      loop_ub = S->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        br = S->size[0];
        for (i41 = 0; i41 < br; i41++) {
          S->data[i41 + S->size[0] * i40] = 0.0;
        }
      }

      if ((c_y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        ii = c_y->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i40 = idx + b_m;
          for (ic = idx; ic + 1 <= i40; ic++) {
            S->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i40 = idx + b_m;
              for (ic = idx; ic + 1 <= i40; ic++) {
                ia++;
                S->data[ic] += K->data[ib] * c_y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      emxFree_real_T(&c_y);
      i40 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i40 = 0; i40 < loop_ub; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          K->data[i41 + K->size[0] * i40] = H->data[i40 + H->size[0] * i41];
        }
      }

      emxInit_real_T1(&d_y, 2);
      H_idx_0 = (unsigned int)K->size[1];
      i40 = d_y->size[0] * d_y->size[1];
      d_y->size[0] = 91;
      d_y->size[1] = (int)H_idx_0;
      d_y->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)d_y, i40, (int)sizeof(double));
      loop_ub = d_y->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          d_y->data[i41 + d_y->size[0] * i40] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        ii = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= ii; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            d_y->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                d_y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      emxInit_real_T1(&b_S, 2);
      i40 = b_S->size[0] * b_S->size[1];
      b_S->size[0] = S->size[0];
      b_S->size[1] = S->size[1];
      emxEnsureCapacity((emxArray__common *)b_S, i40, (int)sizeof(double));
      loop_ub = S->size[0] * S->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        b_S->data[i40] = S->data[i40] + R->data[i40];
      }

      mrdivide(d_y, b_S, K);
      emxFree_real_T(&b_S);
      emxFree_real_T(&d_y);
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i40 = 0; i40 < 91; i40++) {
          x_apo[i40] = 0.0;
          loop_ub = K->size[1];
          for (i41 = 0; i41 < loop_ub; i41++) {
            b_x_apo = x_apo[i40] + K->data[i40 + K->size[0] * i41] * r->data[i41];
            x_apo[i40] = b_x_apo;
          }
        }
      } else {
        memset(&x_apo[0], 0, 91U * sizeof(double));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic < 91; ic++) {
              ia++;
              b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
              x_apo[ic] = b_x_apo;
            }
          }

          ar += 91;
        }
      }

      for (i40 = 0; i40 < 3; i40++) {
        b_xt->robot_state.pos[i40] += x_apo[i40];
      }

      quatPlusThetaJ(*(double (*)[3])&x_apo[3], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->robot_state.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->robot_state.att);
      for (i40 = 0; i40 < 3; i40++) {
        b_xt->robot_state.vel[i40] += x_apo[6 + i40];
        b_xt->robot_state.IMU.gyro_bias[i40] += x_apo[9 + i40];
        b_xt->robot_state.IMU.acc_bias[i40] += x_apo[12 + i40];
      }

      quatPlusThetaJ(*(double (*)[3])&x_apo[15], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->origin.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->origin.att);
      quatPlusThetaJ(*(double (*)[3])&x_apo[18], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->robot_state.IMU.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->robot_state.IMU.att);
      for (i40 = 0; i40 < 3; i40++) {
        b_xt->robot_state.IMU.pos[i40] += x_apo[21 + i40];
      }

      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        i40 = anchorIdx * 14;
        i41 = anchorIdx * 14;
        for (idx = 0; idx < 3; idx++) {
          b_xt->anchor_states[anchorIdx].pos[idx] += x_apo[(idx + i40) + 21];
          c_x_apo[idx] = x_apo[(idx + i41) + 24];
        }

        for (ii = 0; ii < 4; ii++) {
          c_xt[ii] = b_xt->anchor_states[anchorIdx].att[ii];
        }

        quatPlusThetaJ(c_x_apo, dv20);
        quatmultJ(dv20, c_xt, b_xt->anchor_states[anchorIdx].att);
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth += x_apo[(anchorIdx * 14 + featureIdx) + 27];
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0) {
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;
            }
          }
        }
      }

      if ((K->size[1] == 1) || (H->size[0] == 1)) {
        for (i40 = 0; i40 < 91; i40++) {
          for (i41 = 0; i41 < 91; i41++) {
            e_y[i40 + 91 * i41] = 0.0;
            loop_ub = K->size[1];
            for (idx = 0; idx < loop_ub; idx++) {
              e_y[i40 + 91 * i41] += K->data[i40 + K->size[0] * idx] * H->
                data[idx + H->size[0] * i41];
            }
          }
        }
      } else {
        k = K->size[1];
        memset(&e_y[0], 0, 8281U * sizeof(double));
        for (idx = 0; idx <= 8191; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            e_y[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= 8191; idx += 91) {
          ar = 0;
          i40 = br + k;
          for (ib = br; ib + 1 <= i40; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                e_y[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += 91;
          }

          br += k;
        }
      }

      c_eye(dv21);
      for (i40 = 0; i40 < 91; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          dv22[i41 + 91 * i40] = dv21[i41 + 91 * i40] - e_y[i41 + 91 * i40];
        }
      }

      for (i40 = 0; i40 < 91; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          dv23[i40 + 91 * i41] = 0.0;
          for (idx = 0; idx < 91; idx++) {
            dv23[i40 + 91 * i41] += dv22[i40 + 91 * idx] * b_P[idx + 91 * i41];
          }
        }
      }

      for (i40 = 0; i40 < 91; i40++) {
        memcpy(&b_P[i40 * 91], &dv23[i40 * 91], 91U * sizeof(double));
      }
    } else {
      for (ii = 0; ii < 40; ii++) {
        LI_inlier_status[ii] = false;
      }
    }
  }

  // % D Partial EKF update using high-innovation inliers
  //  high innovation inliers (ordered like updateVect)
  for (ii = 0; ii < 40; ii++) {
    b_activeFeatures = (activeFeatures[ii] && (!LI_inlier_status[ii]));
    activeFeatures[ii] = b_activeFeatures;
    LI_inlier_status[ii] = b_activeFeatures;
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

  for (i40 = 0; i40 < loop_ub; i40++) {
    hyp_ind_data[i40] = ii_data[i40];
  }

  if (any(activeFeatures)) {
    emxInit_real_T1(&b_K, 2);
    emxInit_real_T1(&b_H, 2);
    i40 = b_K->size[0] * b_K->size[1];
    b_K->size[0] = 1;
    b_K->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_K, i40, (int)sizeof(double));
    b_K->data[0] = 0.0;

    //  for coder
    i40 = b_H->size[0] * b_H->size[1];
    b_H->size[0] = 1;
    b_H->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_H, i40, (int)sizeof(double));
    b_H->data[0] = 0.0;

    //  for coder
    it = 1;
    emxInit_real_T1(&c_C, 2);
    emxInit_real_T1(&d_C, 2);
    emxInit_real_T1(&f_y, 2);
    emxInit_real_T1(&g_y, 2);
    emxInit_real_T1(&h_y, 2);
    emxInit_real_T1(&e_C, 2);
    emxInit_real_T1(&c_H, 2);
    emxInit_real_T1(&d_H, 2);
    emxInit_real_T1(&e_H, 2);
    while (it <= c_VIOParameters_max_ekf_iterati) {
      getScaledMap(b_xt);

      //  build the map according to the current estimate
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, LI_inlier_status,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, R);
      i40 = b_H->size[0] * b_H->size[1];
      b_H->size[0] = H->size[0];
      b_H->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)b_H, i40, (int)sizeof(double));
      loop_ub = H->size[0] * H->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        b_H->data[i40] = H->data[i40];
      }

      //  the residual is ordered by anchors/features, not like updateVect
      H_idx_0 = (unsigned int)H->size[0];
      i40 = f_y->size[0] * f_y->size[1];
      f_y->size[0] = (int)H_idx_0;
      f_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)f_y, i40, (int)sizeof(double));
      b_m = H->size[0];
      i40 = f_y->size[0] * f_y->size[1];
      f_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)f_y, i40, (int)sizeof(double));
      for (i40 = 0; i40 < 91; i40++) {
        loop_ub = f_y->size[0];
        for (i41 = 0; i41 < loop_ub; i41++) {
          f_y->data[i41 + f_y->size[0] * i40] = 0.0;
        }
      }

      if (H->size[0] == 0) {
      } else {
        ii = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i40 = idx + b_m;
          for (ic = idx; ic + 1 <= i40; ic++) {
            f_y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i40 = idx + b_m;
              for (ic = idx; ic + 1 <= i40; ic++) {
                ia++;
                f_y->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i40 = c_H->size[0] * c_H->size[1];
      c_H->size[0] = 91;
      c_H->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)c_H, i40, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i40 = 0; i40 < loop_ub; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          c_H->data[i41 + c_H->size[0] * i40] = H->data[i40 + H->size[0] * i41];
        }
      }

      ii = H->size[0];
      i40 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = ii;
      emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
      for (i40 = 0; i40 < ii; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          K->data[i41 + K->size[0] * i40] = c_H->data[i41 + 91 * i40];
        }
      }

      a[0] = (unsigned int)f_y->size[0];
      a[1] = (unsigned int)K->size[1];
      i40 = c_C->size[0] * c_C->size[1];
      c_C->size[0] = (int)a[0];
      c_C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)c_C, i40, (int)sizeof(double));
      b_m = f_y->size[0];
      i40 = c_C->size[0] * c_C->size[1];
      emxEnsureCapacity((emxArray__common *)c_C, i40, (int)sizeof(double));
      loop_ub = c_C->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        br = c_C->size[0];
        for (i41 = 0; i41 < br; i41++) {
          c_C->data[i41 + c_C->size[0] * i40] = 0.0;
        }
      }

      if ((f_y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        ii = f_y->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i40 = idx + b_m;
          for (ic = idx; ic + 1 <= i40; ic++) {
            c_C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i40 = idx + b_m;
              for (ic = idx; ic + 1 <= i40; ic++) {
                ia++;
                c_C->data[ic] += K->data[ib] * f_y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i40 = S->size[0] * S->size[1];
      S->size[0] = c_C->size[0];
      S->size[1] = c_C->size[1];
      emxEnsureCapacity((emxArray__common *)S, i40, (int)sizeof(double));
      loop_ub = c_C->size[0] * c_C->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        S->data[i40] = c_C->data[i40] + R->data[i40];
      }

      for (k = 0; k < ii_size_idx_0; k++) {
        ii = k << 1;
        for (i40 = 0; i40 < 2; i40++) {
          b_r[i40] = r->data[i40 + ii];
        }

        ii = k << 1;
        idx = k << 1;
        for (i40 = 0; i40 < 2; i40++) {
          for (i41 = 0; i41 < 2; i41++) {
            c_xt[i41 + (i40 << 1)] = S->data[(i41 + ii) + S->size[0] * (i40 +
              idx)];
          }
        }

        b_mrdivide(b_r, c_xt, a);
        ii = k << 1;
        for (i40 = 0; i40 < 2; i40++) {
          z_u[i40] = r->data[i40 + ii];
        }

        d9 = 0.0;
        for (i40 = 0; i40 < 2; i40++) {
          d9 += a[i40] * z_u[i40];
        }

        if (d9 > 6.0) {
          ii = k << 1;
          for (i40 = 0; i40 < 2; i40++) {
            r->data[i40 + ii] = 0.0;
          }

          ii = k << 1;
          for (i40 = 0; i40 < 91; i40++) {
            for (i41 = 0; i41 < 2; i41++) {
              b_H->data[(i41 + ii) + b_H->size[0] * i40] = 0.0;
            }
          }

          if (it == c_VIOParameters_max_ekf_iterati) {
            LI_inlier_status[hyp_ind_data[k] - 1] = false;

            //  only reject the feature if its still bad in last iteration, otherwise just dont use for this update 
          }

          //          ros_info('rejecting %i', HI_ind(k))
          if (updateVect[hyp_ind_data[k] - 1] == 2) {
            b_ros_error();
          }
        }
      }

      H_idx_0 = (unsigned int)b_H->size[0];
      i40 = g_y->size[0] * g_y->size[1];
      g_y->size[0] = (int)H_idx_0;
      g_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)g_y, i40, (int)sizeof(double));
      b_m = b_H->size[0];
      i40 = g_y->size[0] * g_y->size[1];
      g_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)g_y, i40, (int)sizeof(double));
      for (i40 = 0; i40 < 91; i40++) {
        loop_ub = g_y->size[0];
        for (i41 = 0; i41 < loop_ub; i41++) {
          g_y->data[i41 + g_y->size[0] * i40] = 0.0;
        }
      }

      if (b_H->size[0] == 0) {
      } else {
        ii = b_H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i40 = idx + b_m;
          for (ic = idx; ic + 1 <= i40; ic++) {
            g_y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i40 = idx + b_m;
              for (ic = idx; ic + 1 <= i40; ic++) {
                ia++;
                g_y->data[ic] += b_P[ib] * b_H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i40 = d_H->size[0] * d_H->size[1];
      d_H->size[0] = b_H->size[1];
      d_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)d_H, i40, (int)sizeof(double));
      loop_ub = b_H->size[0];
      for (i40 = 0; i40 < loop_ub; i40++) {
        br = b_H->size[1];
        for (i41 = 0; i41 < br; i41++) {
          d_H->data[i41 + d_H->size[0] * i40] = b_H->data[i40 + b_H->size[0] *
            i41];
        }
      }

      ii = b_H->size[0];
      i40 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = ii;
      emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
      for (i40 = 0; i40 < ii; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          K->data[i41 + K->size[0] * i40] = d_H->data[i41 + 91 * i40];
        }
      }

      a[0] = (unsigned int)g_y->size[0];
      a[1] = (unsigned int)K->size[1];
      i40 = d_C->size[0] * d_C->size[1];
      d_C->size[0] = (int)a[0];
      d_C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)d_C, i40, (int)sizeof(double));
      b_m = g_y->size[0];
      i40 = d_C->size[0] * d_C->size[1];
      emxEnsureCapacity((emxArray__common *)d_C, i40, (int)sizeof(double));
      loop_ub = d_C->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        br = d_C->size[0];
        for (i41 = 0; i41 < br; i41++) {
          d_C->data[i41 + d_C->size[0] * i40] = 0.0;
        }
      }

      if ((g_y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        ii = g_y->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i40 = idx + b_m;
          for (ic = idx; ic + 1 <= i40; ic++) {
            d_C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i40 = idx + b_m;
              for (ic = idx; ic + 1 <= i40; ic++) {
                ia++;
                d_C->data[ic] += K->data[ib] * g_y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i40 = e_H->size[0] * e_H->size[1];
      e_H->size[0] = b_H->size[1];
      e_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)e_H, i40, (int)sizeof(double));
      loop_ub = b_H->size[0];
      for (i40 = 0; i40 < loop_ub; i40++) {
        br = b_H->size[1];
        for (i41 = 0; i41 < br; i41++) {
          e_H->data[i41 + e_H->size[0] * i40] = b_H->data[i40 + b_H->size[0] *
            i41];
        }
      }

      ii = b_H->size[0];
      i40 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = ii;
      emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
      for (i40 = 0; i40 < ii; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          K->data[i41 + K->size[0] * i40] = e_H->data[i41 + 91 * i40];
        }
      }

      H_idx_0 = (unsigned int)K->size[1];
      i40 = h_y->size[0] * h_y->size[1];
      h_y->size[0] = 91;
      h_y->size[1] = (int)H_idx_0;
      h_y->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)h_y, i40, (int)sizeof(double));
      loop_ub = h_y->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          h_y->data[i41 + h_y->size[0] * i40] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        ii = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= ii; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            h_y->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                h_y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      i40 = e_C->size[0] * e_C->size[1];
      e_C->size[0] = d_C->size[0];
      e_C->size[1] = d_C->size[1];
      emxEnsureCapacity((emxArray__common *)e_C, i40, (int)sizeof(double));
      loop_ub = d_C->size[0] * d_C->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        e_C->data[i40] = d_C->data[i40] + R->data[i40];
      }

      mrdivide(h_y, e_C, K);
      i40 = b_K->size[0] * b_K->size[1];
      b_K->size[0] = 91;
      b_K->size[1] = K->size[1];
      emxEnsureCapacity((emxArray__common *)b_K, i40, (int)sizeof(double));
      loop_ub = K->size[0] * K->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        b_K->data[i40] = K->data[i40];
      }

      if ((b_K->size[1] == 1) || (r->size[0] == 1)) {
        loop_ub = b_K->size[0];
        for (i40 = 0; i40 < loop_ub; i40++) {
          x_apo_data[i40] = 0.0;
          br = b_K->size[1];
          for (i41 = 0; i41 < br; i41++) {
            b_x_apo_data = x_apo_data[i40] + b_K->data[i40 + b_K->size[0] * i41]
              * r->data[i41];
            x_apo_data[i40] = b_x_apo_data;
          }
        }
      } else {
        k = b_K->size[1];
        memset(&x_apo_data[0], 0, 91U * sizeof(double));
        for (ic = 1; ic < 92; ic++) {
          x_apo_data[ic - 1] = 0.0;
        }

        ar = 0;
        for (ib = 0; ib + 1 <= k; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 < 92; ic++) {
              ia++;
              x_apo_data[ic] += r->data[ib] * b_K->data[ia - 1];
            }
          }

          ar += 91;
        }
      }

      for (i40 = 0; i40 < 3; i40++) {
        b_xt->robot_state.pos[i40] += x_apo_data[i40];
      }

      quatPlusThetaJ(*(double (*)[3])&x_apo_data[3], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->robot_state.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->robot_state.att);
      for (i40 = 0; i40 < 3; i40++) {
        b_xt->robot_state.vel[i40] += x_apo_data[6 + i40];
        b_xt->robot_state.IMU.gyro_bias[i40] += x_apo_data[9 + i40];
        b_xt->robot_state.IMU.acc_bias[i40] += x_apo_data[12 + i40];
      }

      quatPlusThetaJ(*(double (*)[3])&x_apo_data[15], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->origin.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->origin.att);
      quatPlusThetaJ(*(double (*)[3])&x_apo_data[18], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->robot_state.IMU.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->robot_state.IMU.att);
      for (i40 = 0; i40 < 3; i40++) {
        b_xt->robot_state.IMU.pos[i40] += x_apo_data[21 + i40];
      }

      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        i40 = anchorIdx * 14;
        i41 = anchorIdx * 14;
        for (idx = 0; idx < 3; idx++) {
          b_xt->anchor_states[anchorIdx].pos[idx] += x_apo_data[(idx + i40) + 21];
          c_x_apo[idx] = x_apo_data[(idx + i41) + 24];
        }

        for (ii = 0; ii < 4; ii++) {
          c_xt[ii] = b_xt->anchor_states[anchorIdx].att[ii];
        }

        quatPlusThetaJ(c_x_apo, dv24);
        quatmultJ(dv24, c_xt, b_xt->anchor_states[anchorIdx].att);
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            if (LI_inlier_status[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1]) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth += x_apo_data[(anchorIdx * 14 + featureIdx) + 27];
              if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                   inverse_depth < 0.0) && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                //  only reject if we are done iterating
                updateVect[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            } else {
              if (activeFeatures[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                //  if it is not a HI inlier, but was a candidate, it was rejected by mahalanobis 
                //  only reject if we are done iterating
                c_ros_info(b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].status_idx, featureIdx +
                           1, anchorIdx + 1);
                updateVect[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            }
          }
        }
      }

      it++;
    }

    emxFree_real_T(&e_H);
    emxFree_real_T(&d_H);
    emxFree_real_T(&c_H);
    emxFree_real_T(&e_C);
    emxFree_real_T(&h_y);
    emxFree_real_T(&g_y);
    emxFree_real_T(&f_y);
    emxFree_real_T(&d_C);
    emxFree_real_T(&c_C);
    emxInit_real_T1(&f_C, 2);
    if ((b_K->size[1] == 1) || (b_H->size[0] == 1)) {
      i40 = f_C->size[0] * f_C->size[1];
      f_C->size[0] = b_K->size[0];
      f_C->size[1] = b_H->size[1];
      emxEnsureCapacity((emxArray__common *)f_C, i40, (int)sizeof(double));
      loop_ub = b_K->size[0];
      for (i40 = 0; i40 < loop_ub; i40++) {
        br = b_H->size[1];
        for (i41 = 0; i41 < br; i41++) {
          f_C->data[i40 + f_C->size[0] * i41] = 0.0;
          ii = b_K->size[1];
          for (idx = 0; idx < ii; idx++) {
            f_C->data[i40 + f_C->size[0] * i41] += b_K->data[i40 + b_K->size[0] *
              idx] * b_H->data[idx + b_H->size[0] * i41];
          }
        }
      }
    } else {
      k = b_K->size[1];
      a[0] = (signed char)b_K->size[0];
      a[1] = (signed char)b_H->size[1];
      i40 = f_C->size[0] * f_C->size[1];
      f_C->size[0] = (int)a[0];
      f_C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)f_C, i40, (int)sizeof(double));
      b_m = b_K->size[0];
      i40 = f_C->size[0] * f_C->size[1];
      emxEnsureCapacity((emxArray__common *)f_C, i40, (int)sizeof(double));
      loop_ub = f_C->size[1];
      for (i40 = 0; i40 < loop_ub; i40++) {
        br = f_C->size[0];
        for (i41 = 0; i41 < br; i41++) {
          f_C->data[i41 + f_C->size[0] * i40] = 0.0;
        }
      }

      ii = b_K->size[0] * (b_H->size[1] - 1);
      for (idx = 0; idx <= ii; idx += b_m) {
        i40 = idx + b_m;
        for (ic = idx; ic + 1 <= i40; ic++) {
          f_C->data[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= ii; idx += b_m) {
        ar = 0;
        i40 = br + k;
        for (ib = br; ib + 1 <= i40; ib++) {
          if (b_H->data[ib] != 0.0) {
            ia = ar;
            i41 = idx + b_m;
            for (ic = idx; ic + 1 <= i41; ic++) {
              ia++;
              f_C->data[ic] += b_H->data[ib] * b_K->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
      }
    }

    emxFree_real_T(&b_H);
    emxFree_real_T(&b_K);
    c_eye(dv21);
    for (i40 = 0; i40 < 91; i40++) {
      for (i41 = 0; i41 < 91; i41++) {
        dv22[i41 + 91 * i40] = dv21[i41 + 91 * i40] - f_C->data[i41 + 91 * i40];
      }
    }

    emxFree_real_T(&f_C);
    for (i40 = 0; i40 < 91; i40++) {
      for (i41 = 0; i41 < 91; i41++) {
        dv25[i40 + 91 * i41] = 0.0;
        for (idx = 0; idx < 91; idx++) {
          dv25[i40 + 91 * i41] += dv22[i40 + 91 * idx] * b_P[idx + 91 * i41];
        }
      }
    }

    for (i40 = 0; i40 < 91; i40++) {
      memcpy(&b_P[i40 * 91], &dv25[i40 * 91], 91U * sizeof(double));
    }
  }

  emxFree_real_T(&S);

  // % Update the delayed initialization features
  if (c_VIOParameters_delayed_initial) {
    emxInit_real_T1(&i_y, 2);
    getScaledMap(b_xt);
    getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att, b_xt->anchor_states,
               z_u_l, delayedFeatures, cameraparams_FocalLength,
               cameraparams_PrincipalPoint, noiseParameters_image_noise, r, H, R);
    H_idx_0 = (unsigned int)H->size[0];
    i40 = i_y->size[0] * i_y->size[1];
    i_y->size[0] = (int)H_idx_0;
    i_y->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)i_y, i40, (int)sizeof(double));
    b_m = H->size[0];
    i40 = i_y->size[0] * i_y->size[1];
    i_y->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)i_y, i40, (int)sizeof(double));
    for (i40 = 0; i40 < 91; i40++) {
      loop_ub = i_y->size[0];
      for (i41 = 0; i41 < loop_ub; i41++) {
        i_y->data[i41 + i_y->size[0] * i40] = 0.0;
      }
    }

    if (H->size[0] == 0) {
    } else {
      ii = H->size[0] * 90;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i40 = idx + b_m;
        for (ic = idx; ic + 1 <= i40; ic++) {
          i_y->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (b_P[ib] != 0.0) {
            ia = ar;
            i40 = idx + b_m;
            for (ic = idx; ic + 1 <= i40; ic++) {
              ia++;
              i_y->data[ic] += b_P[ib] * H->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 91;
        idx += b_m;
      }
    }

    i40 = K->size[0] * K->size[1];
    K->size[0] = 91;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
    loop_ub = H->size[0];
    for (i40 = 0; i40 < loop_ub; i40++) {
      for (i41 = 0; i41 < 91; i41++) {
        K->data[i41 + K->size[0] * i40] = H->data[i40 + H->size[0] * i41];
      }
    }

    emxInit_real_T1(&g_C, 2);
    a[0] = (unsigned int)i_y->size[0];
    a[1] = (unsigned int)K->size[1];
    i40 = g_C->size[0] * g_C->size[1];
    g_C->size[0] = (int)a[0];
    g_C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)g_C, i40, (int)sizeof(double));
    b_m = i_y->size[0];
    i40 = g_C->size[0] * g_C->size[1];
    emxEnsureCapacity((emxArray__common *)g_C, i40, (int)sizeof(double));
    loop_ub = g_C->size[1];
    for (i40 = 0; i40 < loop_ub; i40++) {
      br = g_C->size[0];
      for (i41 = 0; i41 < br; i41++) {
        g_C->data[i41 + g_C->size[0] * i40] = 0.0;
      }
    }

    if ((i_y->size[0] == 0) || (K->size[1] == 0)) {
    } else {
      ii = i_y->size[0] * (K->size[1] - 1);
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i40 = idx + b_m;
        for (ic = idx; ic + 1 <= i40; ic++) {
          g_C->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (K->data[ib] != 0.0) {
            ia = ar;
            i40 = idx + b_m;
            for (ic = idx; ic + 1 <= i40; ic++) {
              ia++;
              g_C->data[ic] += K->data[ib] * i_y->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 91;
        idx += b_m;
      }
    }

    emxFree_real_T(&i_y);
    i40 = K->size[0] * K->size[1];
    K->size[0] = 91;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i40, (int)sizeof(double));
    loop_ub = H->size[0];
    for (i40 = 0; i40 < loop_ub; i40++) {
      for (i41 = 0; i41 < 91; i41++) {
        K->data[i41 + K->size[0] * i40] = H->data[i40 + H->size[0] * i41];
      }
    }

    emxInit_real_T1(&j_y, 2);
    H_idx_0 = (unsigned int)K->size[1];
    i40 = j_y->size[0] * j_y->size[1];
    j_y->size[0] = 91;
    j_y->size[1] = (int)H_idx_0;
    j_y->size[0] = 91;
    emxEnsureCapacity((emxArray__common *)j_y, i40, (int)sizeof(double));
    loop_ub = j_y->size[1];
    for (i40 = 0; i40 < loop_ub; i40++) {
      for (i41 = 0; i41 < 91; i41++) {
        j_y->data[i41 + j_y->size[0] * i40] = 0.0;
      }
    }

    if (K->size[1] == 0) {
    } else {
      ii = 91 * (K->size[1] - 1);
      for (idx = 0; idx <= ii; idx += 91) {
        for (ic = idx; ic + 1 <= idx + 91; ic++) {
          j_y->data[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= ii; idx += 91) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (K->data[ib] != 0.0) {
            ia = ar;
            for (ic = idx; ic + 1 <= idx + 91; ic++) {
              ia++;
              j_y->data[ic] += K->data[ib] * b_P[ia - 1];
            }
          }

          ar += 91;
        }

        br += 91;
      }
    }

    emxInit_real_T1(&h_C, 2);
    i40 = h_C->size[0] * h_C->size[1];
    h_C->size[0] = g_C->size[0];
    h_C->size[1] = g_C->size[1];
    emxEnsureCapacity((emxArray__common *)h_C, i40, (int)sizeof(double));
    loop_ub = g_C->size[0] * g_C->size[1];
    for (i40 = 0; i40 < loop_ub; i40++) {
      h_C->data[i40] = g_C->data[i40] + R->data[i40];
    }

    emxFree_real_T(&g_C);
    mrdivide(j_y, h_C, K);
    emxFree_real_T(&h_C);
    emxFree_real_T(&j_y);
    if ((K->size[1] == 1) || (r->size[0] == 1)) {
      for (i40 = 0; i40 < 91; i40++) {
        x_apo[i40] = 0.0;
        loop_ub = K->size[1];
        for (i41 = 0; i41 < loop_ub; i41++) {
          b_x_apo = x_apo[i40] + K->data[i40 + K->size[0] * i41] * r->data[i41];
          x_apo[i40] = b_x_apo;
        }
      }
    } else {
      memset(&x_apo[0], 0, 91U * sizeof(double));
      ar = 0;
      for (ib = 0; ib + 1 <= K->size[1]; ib++) {
        if (r->data[ib] != 0.0) {
          ia = ar;
          for (ic = 0; ic < 91; ic++) {
            ia++;
            b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
            x_apo[ic] = b_x_apo;
          }
        }

        ar += 91;
      }
    }

    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth += x_apo[(anchorIdx * 14 + featureIdx) + 27];
        }
      }
    }

    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      for (i40 = 0; i40 < 91; i40++) {
        for (i41 = 0; i41 < 91; i41++) {
          e_y[i40 + 91 * i41] = 0.0;
          loop_ub = K->size[1];
          for (idx = 0; idx < loop_ub; idx++) {
            e_y[i40 + 91 * i41] += K->data[i40 + K->size[0] * idx] * H->data[idx
              + H->size[0] * i41];
          }
        }
      }
    } else {
      k = K->size[1];
      memset(&e_y[0], 0, 8281U * sizeof(double));
      for (idx = 0; idx <= 8191; idx += 91) {
        for (ic = idx; ic + 1 <= idx + 91; ic++) {
          e_y[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= 8191; idx += 91) {
        ar = 0;
        i40 = br + k;
        for (ib = br; ib + 1 <= i40; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            for (ic = idx; ic + 1 <= idx + 91; ic++) {
              ia++;
              e_y[ic] += H->data[ib] * K->data[ia - 1];
            }
          }

          ar += 91;
        }

        br += k;
      }
    }

    c_eye(dv21);
    for (i40 = 0; i40 < 91; i40++) {
      for (i41 = 0; i41 < 91; i41++) {
        dv22[i41 + 91 * i40] = dv21[i41 + 91 * i40] - e_y[i41 + 91 * i40];
      }
    }

    for (i40 = 0; i40 < 91; i40++) {
      for (i41 = 0; i41 < 91; i41++) {
        dv26[i40 + 91 * i41] = 0.0;
        for (idx = 0; idx < 91; idx++) {
          dv26[i40 + 91 * i41] += dv22[i40 + 91 * idx] * b_P[idx + 91 * i41];
        }
      }
    }

    for (i40 = 0; i40 < 91; i40++) {
      memcpy(&b_P[i40 * 91], &dv26[i40 * 91], 91U * sizeof(double));
    }
  }

  emxFree_real_T(&R);
  emxFree_real_T(&H);
  emxFree_real_T(&r);
  emxFree_real_T(&K);

  // %
}

//
// THIS IS OK, It is according to the NASA memo found
//  https://claraty.jpl.nasa.gov/man/software/development/conventions/standards_docs/unadopted/JPL_Quaternions_Breckenridge.pdf
// QUATFROMROTJ Get equivalent quaternion of the rotation matrix R
//    Returns a quaternion in JPL notation
//  The implementation is copied from qGetQ(R), but we are careful about the
//  ordering of the output vector
//  Q = [0;0;0;1];
//  % [r,c] = size( R );
//  % if( r ~= 3 || c ~= 3 )
//  %     error( 'R must be a 3x3 matrix\n\r' );
//  % end
//
//  if abs(det(R) - 1) > 1e-2
//      error('The provided matrix is not a valid rotation matrix. It must have det(R) = 1, but is %i.', det(R))
//  end
//  T = R(1,1)+R(2,2)+R(3,3);
//  pivot = zeros(4,1);
//  pivot(1) = sqrt(max(1+2*R(1,1)-T, eps))/2;
//  pivot(2) = sqrt(max(1+2*R(2,2)-T, eps))/2;
//  pivot(3) = sqrt(max(1+2*R(3,3)-T, eps))/2;
//  pivot(4) = sqrt(max(1+T, eps))/2;
//  if ~all(isreal(pivot))
//      error('complex')
//  end
//  index = round(find(pivot==max(pivot)));
//  if(index(1)==1)
//      Q(1) = sqrt(1+2*R(1,1)-T)/2;
//      Q(2) = (R(1,2)+R(2,1))/(4*(Q(1)));
//      Q(3) = (R(1,3)+R(3,1))/(4*(Q(1)));
//      Q(4) = (R(2,3)-R(3,2))/(4*(Q(1)));
//  elseif(index(1)==2)
//      Q(2) = sqrt(1+2*R(2,2)-T)/2;
//      Q(1) = (R(1,2)+R(2,1))/(4*Q(2));
//      Q(3) = (R(2,3)+R(3,2))/(4*Q(2));
//      Q(4) = (R(3,1)-R(1,3))/(4*Q(2));
//  elseif(index(1)==3)
//      Q(3) = sqrt(1+2*R(3,3)-T)/2;
//      Q(1) = (R(1,3)+R(3,1))/(4*Q(3));
//      Q(2) = (R(2,3)+R(3,2))/(4*Q(3));
//      Q(4) = (R(1,2)-R(2,1))/(4*Q(3));
//  elseif(index(1)==4)
//      Q(4) = sqrt(1+T)/2;
//      Q(1) = (R(2,3)-R(3,2))/(4*Q(4));
//      Q(2) = (R(3,1)-R(1,3))/(4*Q(4));
//      Q(3) = (R(1,2)-R(2,1))/(4*Q(4));
//  end
// Arguments    : const double R[9]
//                double Q[4]
// Return Type  : void
//
static void QuatFromRotJ(const double R[9], double Q[4])
{
  double varargin_1[4];
  int ixstart;
  double mtmp;
  int itmp;
  int ix;
  boolean_T exitg1;

  // % speed optimization
  varargin_1[0] = (1.0 + R[0]) - (R[4] + R[8]);
  varargin_1[1] = (1.0 + R[4]) - (R[0] + R[8]);
  varargin_1[2] = (1.0 + R[8]) - (R[0] + R[4]);
  varargin_1[3] = 1.0 + ((R[0] + R[4]) + R[8]);
  ixstart = 1;
  mtmp = varargin_1[0];
  itmp = 1;
  if (rtIsNaN(varargin_1[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 5)) {
      ixstart = ix;
      if (!rtIsNaN(varargin_1[ix - 1])) {
        mtmp = varargin_1[ix - 1];
        itmp = ix;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 4) {
    while (ixstart + 1 < 5) {
      if (varargin_1[ixstart] > mtmp) {
        mtmp = varargin_1[ixstart];
        itmp = ixstart + 1;
      }

      ixstart++;
    }
  }

  if (itmp == 1) {
    Q[0] = sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[1] = (R[3] + R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[2] = (R[6] + R[2]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[3] = (R[7] - R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else if (itmp == 2) {
    Q[0] = (R[3] + R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[1] = sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[2] = (R[7] + R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[3] = (R[2] - R[6]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else if (itmp == 3) {
    Q[0] = (R[6] + R[2]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[1] = (R[7] + R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[2] = sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[3] = (R[3] - R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else {
    Q[0] = (R[7] - R[5]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[1] = (R[2] - R[6]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[2] = (R[3] - R[1]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[3] = sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0;
  }
}

//
// Arguments    : double P_apo[8281]
//                g_struct_T *x
//                double dt
//                double processNoise_qv
//                double processNoise_qw
//                double processNoise_qao
//                double processNoise_qwo
//                double processNoise_qR_ci
//                const double measurements_acc_duo[3]
//                const double measurements_gyr_duo[3]
// Return Type  : void
//
static void SLAM_pred_euler(double P_apo[8281], g_struct_T *x, double dt, double
  processNoise_qv, double processNoise_qw, double processNoise_qao, double
  processNoise_qwo, double processNoise_qR_ci, const double
  measurements_acc_duo[3], const double measurements_gyr_duo[3])
{
  double R_cw[9];
  double R_ci[9];
  double b_R_ci[9];
  int i36;
  int i;
  double t_ci[3];
  double w_imu[3];
  double R[9];
  double b_R[9];
  double b_measurements_acc_duo[3];
  double w_c[3];
  double d8;
  int i37;
  double b_x[9];
  double a_c[3];
  double grav_origin[3];
  double y[3];
  static const double b[3] = { 0.0, 0.0, 9.81 };

  double dv6[9];
  double b_y[3];
  double c_y[3];
  double G[315];
  static const signed char iv3[45] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0 };

  static const signed char iv4[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv5[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv6[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1 };

  double P_xx_apr[441];
  double dv7[9];
  double dv8[9];
  double dv9[9];
  double dv10[9];
  double dv11[9];
  double dv12[9];
  double dv13[9];
  double dv14[9];
  double Phi[441];
  static const signed char iv7[63] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv8[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double b_Phi[441];
  double b_processNoise_qw[15];
  double dv15[225];
  double b_G[315];
  double c_G[441];
  double P_xs_apr[1470];
  double c_x[4];
  double dv16[4];

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((x->robot_state.att[0] * x->robot_state.att[0] - x->
              robot_state.att[1] * x->robot_state.att[1]) - x->robot_state.att[2]
             * x->robot_state.att[2]) + x->robot_state.att[3] *
    x->robot_state.att[3];
  R_cw[3] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[1] +
                   x->robot_state.att[2] * x->robot_state.att[3]);
  R_cw[6] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[2] -
                   x->robot_state.att[1] * x->robot_state.att[3]);
  R_cw[1] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[1] -
                   x->robot_state.att[2] * x->robot_state.att[3]);
  R_cw[4] = ((-(x->robot_state.att[0] * x->robot_state.att[0]) +
              x->robot_state.att[1] * x->robot_state.att[1]) -
             x->robot_state.att[2] * x->robot_state.att[2]) + x->
    robot_state.att[3] * x->robot_state.att[3];
  R_cw[7] = 2.0 * (x->robot_state.att[1] * x->robot_state.att[2] +
                   x->robot_state.att[0] * x->robot_state.att[3]);
  R_cw[2] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[2] +
                   x->robot_state.att[1] * x->robot_state.att[3]);
  R_cw[5] = 2.0 * (x->robot_state.att[1] * x->robot_state.att[2] -
                   x->robot_state.att[0] * x->robot_state.att[3]);
  R_cw[8] = ((-(x->robot_state.att[0] * x->robot_state.att[0]) -
              x->robot_state.att[1] * x->robot_state.att[1]) +
             x->robot_state.att[2] * x->robot_state.att[2]) + x->
    robot_state.att[3] * x->robot_state.att[3];

  //  rotation in origin frame
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_ci[0] = ((x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0] -
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) -
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];
  R_ci[3] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[1] +
                   x->robot_state.IMU.att[2] * x->robot_state.IMU.att[3]);
  R_ci[6] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[2] -
                   x->robot_state.IMU.att[1] * x->robot_state.IMU.att[3]);
  R_ci[1] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[1] -
                   x->robot_state.IMU.att[2] * x->robot_state.IMU.att[3]);
  R_ci[4] = ((-(x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0]) +
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) -
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];
  R_ci[7] = 2.0 * (x->robot_state.IMU.att[1] * x->robot_state.IMU.att[2] +
                   x->robot_state.IMU.att[0] * x->robot_state.IMU.att[3]);
  R_ci[2] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[2] +
                   x->robot_state.IMU.att[1] * x->robot_state.IMU.att[3]);
  R_ci[5] = 2.0 * (x->robot_state.IMU.att[1] * x->robot_state.IMU.att[2] -
                   x->robot_state.IMU.att[0] * x->robot_state.IMU.att[3]);
  R_ci[8] = ((-(x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0]) -
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) +
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];

  //  in camera frame
  for (i36 = 0; i36 < 3; i36++) {
    for (i = 0; i < 3; i++) {
      b_R_ci[i + 3 * i36] = -R_ci[i36 + 3 * i];
    }
  }

  //  in imu frame
  for (i = 0; i < 3; i++) {
    t_ci[i] = 0.0;
    for (i36 = 0; i36 < 3; i36++) {
      t_ci[i] += b_R_ci[i + 3 * i36] * x->robot_state.IMU.pos[i36];
    }

    w_imu[i] = measurements_gyr_duo[i] - x->robot_state.IMU.gyro_bias[i];
  }

  //  gyro in IMU frame
  //  gyro in camera frame
  //  w = 0*w;
  //  acceleration in IMU frame
  R[0] = 0.0;
  R[3] = -w_imu[2];
  R[6] = w_imu[1];
  R[1] = w_imu[2];
  R[4] = 0.0;
  R[7] = -w_imu[0];
  R[2] = -w_imu[1];
  R[5] = w_imu[0];
  R[8] = 0.0;
  for (i36 = 0; i36 < 3; i36++) {
    w_c[i36] = 0.0;
    d8 = 0.0;
    for (i = 0; i < 3; i++) {
      w_c[i36] += R_ci[i36 + 3 * i] * w_imu[i];
      b_R[i36 + 3 * i] = 0.0;
      for (i37 = 0; i37 < 3; i37++) {
        b_R[i36 + 3 * i] += R[i36 + 3 * i37] * R[i37 + 3 * i];
      }

      d8 += b_R[i36 + 3 * i] * t_ci[i];
    }

    b_measurements_acc_duo[i36] = (measurements_acc_duo[i36] -
      x->robot_state.IMU.acc_bias[i36]) + d8;
  }

  //  a = 0*a;
  // % compute the linearization F of the non linear model f
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  b_x[0] = ((x->origin.att[0] * x->origin.att[0] - x->origin.att[1] *
             x->origin.att[1]) - x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];
  b_x[3] = 2.0 * (x->origin.att[0] * x->origin.att[1] + x->origin.att[2] *
                  x->origin.att[3]);
  b_x[6] = 2.0 * (x->origin.att[0] * x->origin.att[2] - x->origin.att[1] *
                  x->origin.att[3]);
  b_x[1] = 2.0 * (x->origin.att[0] * x->origin.att[1] - x->origin.att[2] *
                  x->origin.att[3]);
  b_x[4] = ((-(x->origin.att[0] * x->origin.att[0]) + x->origin.att[1] *
             x->origin.att[1]) - x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];
  b_x[7] = 2.0 * (x->origin.att[1] * x->origin.att[2] + x->origin.att[0] *
                  x->origin.att[3]);
  b_x[2] = 2.0 * (x->origin.att[0] * x->origin.att[2] + x->origin.att[1] *
                  x->origin.att[3]);
  b_x[5] = 2.0 * (x->origin.att[1] * x->origin.att[2] - x->origin.att[0] *
                  x->origin.att[3]);
  b_x[8] = ((-(x->origin.att[0] * x->origin.att[0]) - x->origin.att[1] *
             x->origin.att[1]) + x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];

  //  gravity transformed into the origin frame
  //    pos,            rot, vel,                                         gyr_bias,   acc_bias,          origin_att,                                    R_ci 
  for (i36 = 0; i36 < 3; i36++) {
    a_c[i36] = 0.0;
    grav_origin[i36] = 0.0;
    y[i36] = 0.0;
    for (i = 0; i < 3; i++) {
      a_c[i36] += R_ci[i36 + 3 * i] * b_measurements_acc_duo[i];
      grav_origin[i36] += b_x[i36 + 3 * i] * b[i];
      y[i36] += R_ci[i36 + 3 * i] * x->robot_state.IMU.gyro_bias[i];
    }
  }

  dv6[0] = 0.0;
  dv6[3] = -w_imu[2];
  dv6[6] = w_imu[1];
  dv6[1] = w_imu[2];
  dv6[4] = 0.0;
  dv6[7] = -w_imu[0];
  dv6[2] = -w_imu[1];
  dv6[5] = w_imu[0];
  dv6[8] = 0.0;
  for (i36 = 0; i36 < 3; i36++) {
    b_y[i36] = 0.0;
    c_y[i36] = 0.0;
    for (i = 0; i < 3; i++) {
      b_y[i36] += R_cw[i + 3 * i36] * a_c[i];
      c_y[i36] += dv6[i36 + 3 * i] * t_ci[i];
    }
  }

  //  pos
  //  att
  //  vel
  //  gyro bias
  //  acc bias
  //  origin att
  for (i36 = 0; i36 < 15; i36++) {
    for (i = 0; i < 3; i++) {
      G[i + 21 * i36] = 0.0;
      G[(i + 21 * i36) + 3] = iv3[i + 3 * i36];
    }
  }

  for (i36 = 0; i36 < 3; i36++) {
    for (i = 0; i < 3; i++) {
      G[(i + 21 * i36) + 6] = 0.0;
      G[(i + 21 * (i36 + 3)) + 6] = -R_cw[i36 + 3 * i];
      G[(i + 21 * (i36 + 6)) + 6] = 0.0;
      G[(i + 21 * (i36 + 9)) + 6] = 0.0;
      G[(i + 21 * (i36 + 12)) + 6] = 0.0;
    }
  }

  for (i36 = 0; i36 < 15; i36++) {
    for (i = 0; i < 3; i++) {
      G[(i + 21 * i36) + 9] = iv4[i + 3 * i36];
      G[(i + 21 * i36) + 12] = iv5[i + 3 * i36];
      G[(i + 21 * i36) + 15] = 0.0;
      G[(i + 21 * i36) + 18] = iv6[i + 3 * i36];
    }
  }

  //  R_ci
  b_eye(P_xx_apr);
  dv7[0] = 0.0;
  dv7[3] = -w_c[2];
  dv7[6] = w_c[1];
  dv7[1] = w_c[2];
  dv7[4] = 0.0;
  dv7[7] = -w_c[0];
  dv7[2] = -w_c[1];
  dv7[5] = w_c[0];
  dv7[8] = 0.0;
  dv8[0] = 0.0;
  dv8[3] = -y[2];
  dv8[6] = y[1];
  dv8[1] = y[2];
  dv8[4] = 0.0;
  dv8[7] = -y[0];
  dv8[2] = -y[1];
  dv8[5] = y[0];
  dv8[8] = 0.0;
  dv9[0] = 0.0;
  dv9[3] = -b_y[2];
  dv9[6] = b_y[1];
  dv9[1] = b_y[2];
  dv9[4] = 0.0;
  dv9[7] = -b_y[0];
  dv9[2] = -b_y[1];
  dv9[5] = b_y[0];
  dv9[8] = 0.0;
  dv10[0] = 0.0;
  dv10[3] = -c_y[2];
  dv10[6] = c_y[1];
  dv10[1] = c_y[2];
  dv10[4] = 0.0;
  dv10[7] = -c_y[0];
  dv10[2] = -c_y[1];
  dv10[5] = c_y[0];
  dv10[8] = 0.0;
  dv11[0] = 0.0;
  dv11[3] = -w_imu[2];
  dv11[6] = w_imu[1];
  dv11[1] = w_imu[2];
  dv11[4] = 0.0;
  dv11[7] = -w_imu[0];
  dv11[2] = -w_imu[1];
  dv11[5] = w_imu[0];
  dv11[8] = 0.0;
  dv12[0] = 0.0;
  dv12[3] = -t_ci[2];
  dv12[6] = t_ci[1];
  dv12[1] = t_ci[2];
  dv12[4] = 0.0;
  dv12[7] = -t_ci[0];
  dv12[2] = -t_ci[1];
  dv12[5] = t_ci[0];
  dv12[8] = 0.0;
  dv13[0] = 0.0;
  dv13[3] = -grav_origin[2];
  dv13[6] = grav_origin[1];
  dv13[1] = grav_origin[2];
  dv13[4] = 0.0;
  dv13[7] = -grav_origin[0];
  dv13[2] = -grav_origin[1];
  dv13[5] = grav_origin[0];
  dv13[8] = 0.0;
  dv14[0] = 0.0;
  dv14[3] = -a_c[2];
  dv14[6] = a_c[1];
  dv14[1] = a_c[2];
  dv14[4] = 0.0;
  dv14[7] = -a_c[0];
  dv14[2] = -a_c[1];
  dv14[5] = a_c[0];
  dv14[8] = 0.0;
  for (i36 = 0; i36 < 3; i36++) {
    for (i = 0; i < 3; i++) {
      R[i + 3 * i36] = -R_cw[i36 + 3 * i];
      d8 = 0.0;
      for (i37 = 0; i37 < 3; i37++) {
        d8 += dv11[i36 + 3 * i37] * dv12[i37 + 3 * i];
      }

      dv6[i36 + 3 * i] = -dv10[i36 + 3 * i] - d8;
    }
  }

  for (i36 = 0; i36 < 3; i36++) {
    for (i = 0; i < 3; i++) {
      b_R_ci[i36 + 3 * i] = 0.0;
      b_R[i36 + 3 * i] = 0.0;
      for (i37 = 0; i37 < 3; i37++) {
        b_R_ci[i36 + 3 * i] += R[i36 + 3 * i37] * R_ci[i37 + 3 * i];
        b_R[i36 + 3 * i] += R_cw[i36 + 3 * i37] * dv14[i37 + 3 * i];
      }
    }
  }

  for (i36 = 0; i36 < 21; i36++) {
    for (i = 0; i < 3; i++) {
      Phi[i + 21 * i36] = iv7[i + 3 * i36];
    }
  }

  for (i36 = 0; i36 < 3; i36++) {
    for (i = 0; i < 3; i++) {
      Phi[(i + 21 * i36) + 3] = 0.0;
      Phi[(i + 21 * (i36 + 3)) + 3] = -dv7[i + 3 * i36];
      Phi[(i + 21 * (i36 + 6)) + 3] = 0.0;
      Phi[(i + 21 * (i36 + 9)) + 3] = iv8[i + 3 * i36];
      Phi[(i + 21 * (i36 + 12)) + 3] = 0.0;
      Phi[(i + 21 * (i36 + 15)) + 3] = 0.0;
      Phi[(i + 21 * (i36 + 18)) + 3] = -dv8[i + 3 * i36];
      Phi[(i + 21 * i36) + 6] = 0.0;
      Phi[(i + 21 * (i36 + 3)) + 6] = -dv9[i + 3 * i36];
      Phi[(i + 21 * (i36 + 6)) + 6] = 0.0;
      Phi[(i + 21 * (i36 + 9)) + 6] = dv6[i + 3 * i36];
      Phi[(i + 21 * (i36 + 12)) + 6] = b_R_ci[i + 3 * i36];
      Phi[(i + 21 * (i36 + 15)) + 6] = -dv13[i + 3 * i36];
      Phi[(i + 21 * (i36 + 18)) + 6] = b_R[i + 3 * i36];
    }
  }

  for (i36 = 0; i36 < 21; i36++) {
    for (i = 0; i < 3; i++) {
      Phi[(i + 21 * i36) + 9] = 0.0;
      Phi[(i + 21 * i36) + 12] = 0.0;
      Phi[(i + 21 * i36) + 15] = 0.0;
      Phi[(i + 21 * i36) + 18] = 0.0;
    }

    for (i = 0; i < 21; i++) {
      b_Phi[i + 21 * i36] = P_xx_apr[i + 21 * i36] + Phi[i + 21 * i36] * dt;
    }
  }

  b_processNoise_qw[0] = processNoise_qw;
  b_processNoise_qw[1] = processNoise_qw;
  b_processNoise_qw[2] = processNoise_qw;
  b_processNoise_qw[3] = processNoise_qv;
  b_processNoise_qw[4] = processNoise_qv;
  b_processNoise_qw[5] = processNoise_qv;
  b_processNoise_qw[6] = processNoise_qwo;
  b_processNoise_qw[7] = processNoise_qwo;
  b_processNoise_qw[8] = processNoise_qwo;
  b_processNoise_qw[9] = 0.0 * processNoise_qao;
  b_processNoise_qw[10] = processNoise_qao;
  b_processNoise_qw[11] = 0.0 * processNoise_qao;
  b_processNoise_qw[12] = processNoise_qR_ci;
  b_processNoise_qw[13] = processNoise_qR_ci;
  b_processNoise_qw[14] = processNoise_qR_ci;
  b_diag(b_processNoise_qw, dv15);

  //  covariance of the state
  for (i36 = 0; i36 < 21; i36++) {
    for (i = 0; i < 21; i++) {
      Phi[i36 + 21 * i] = 0.0;
      for (i37 = 0; i37 < 21; i37++) {
        Phi[i36 + 21 * i] += b_Phi[i36 + 21 * i37] * P_apo[i37 + 91 * i];
      }
    }

    for (i = 0; i < 15; i++) {
      b_G[i36 + 21 * i] = 0.0;
      for (i37 = 0; i37 < 15; i37++) {
        b_G[i36 + 21 * i] += G[i36 + 21 * i37] * dv15[i37 + 15 * i];
      }
    }

    for (i = 0; i < 21; i++) {
      c_G[i36 + 21 * i] = 0.0;
      for (i37 = 0; i37 < 15; i37++) {
        c_G[i36 + 21 * i] += b_G[i36 + 21 * i37] * G[i + 21 * i37];
      }

      d8 = 0.0;
      for (i37 = 0; i37 < 21; i37++) {
        d8 += Phi[i36 + 21 * i37] * b_Phi[i + 21 * i37];
      }

      P_xx_apr[i36 + 21 * i] = d8 + c_G[i36 + 21 * i] * dt;
    }

    for (i = 0; i < 70; i++) {
      P_xs_apr[i36 + 21 * i] = 0.0;
      for (i37 = 0; i37 < 21; i37++) {
        P_xs_apr[i36 + 21 * i] += b_Phi[i36 + 21 * i37] * P_apo[i37 + 91 * (21 +
          i)];
      }
    }
  }

  //  covariance between current state and trails
  for (i36 = 0; i36 < 21; i36++) {
    for (i = 0; i < 21; i++) {
      P_apo[i + 91 * i36] = (P_xx_apr[i + 21 * i36] + P_xx_apr[i36 + 21 * i]) /
        2.0;
    }
  }

  for (i36 = 0; i36 < 70; i36++) {
    memcpy(&P_apo[i36 * 91 + 1911], &P_xs_apr[i36 * 21], 21U * sizeof(double));
  }

  for (i36 = 0; i36 < 21; i36++) {
    for (i = 0; i < 70; i++) {
      P_apo[(i + 91 * i36) + 21] = P_xs_apr[i36 + 21 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    x->robot_state.pos[i] += x->robot_state.vel[i] * dt;
    b_measurements_acc_duo[i] = w_c[i] * dt;
  }

  for (i = 0; i < 4; i++) {
    c_x[i] = x->robot_state.att[i];
  }

  quatPlusThetaJ(b_measurements_acc_duo, dv16);
  quatmultJ(dv16, c_x, x->robot_state.att);
  for (i36 = 0; i36 < 3; i36++) {
    d8 = 0.0;
    for (i = 0; i < 3; i++) {
      d8 += R_cw[i + 3 * i36] * a_c[i];
    }

    b_measurements_acc_duo[i36] = d8 - grav_origin[i36];
    x->robot_state.vel[i36] += b_measurements_acc_duo[i36] * dt;
  }

  //  velocity
  //  P_apr = (P_apr+P_apr')/2;
}

//
// % Iterative Camera Pose optimization (EKF)
// Arguments    : double P_apr[8281]
//                g_struct_T *b_xt
//                const double c_cameraParams_CameraParameters[2]
//                const double d_cameraParams_CameraParameters[2]
//                const double e_cameraParams_CameraParameters[3]
//                const double f_cameraParams_CameraParameters[2]
//                const double g_cameraParams_CameraParameters[2]
//                const double h_cameraParams_CameraParameters[3]
//                const double cameraParams_r_lr[3]
//                const double cameraParams_R_lr[9]
//                const double cameraParams_R_rl[9]
//                int updateVect[40]
//                double z_all_l[80]
//                double z_all_r[80]
//                double noiseParameters_image_noise
//                double c_noiseParameters_inv_depth_ini
//                const VIOParameters b_VIOParameters
//                double h_u_apo[160]
//                double b_map[120]
//                double b_delayedStatus[40]
// Return Type  : void
//
static void SLAM_upd(double P_apr[8281], g_struct_T *b_xt, const double
                     c_cameraParams_CameraParameters[2], const double
                     d_cameraParams_CameraParameters[2], const double
                     e_cameraParams_CameraParameters[3], const double
                     f_cameraParams_CameraParameters[2], const double
                     g_cameraParams_CameraParameters[2], const double
                     h_cameraParams_CameraParameters[3], const double
                     cameraParams_r_lr[3], const double cameraParams_R_lr[9],
                     const double cameraParams_R_rl[9], int updateVect[40],
                     double z_all_l[80], double z_all_r[80], double
                     noiseParameters_image_noise, double
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, double h_u_apo[160], double b_map[120],
                     double b_delayedStatus[40])
{
  boolean_T x[40];
  int i;
  int idx;
  int ii_data[40];
  int ixstart;
  boolean_T exitg6;
  boolean_T guard2 = false;
  int loop_ub;
  double b_ii_data[40];
  int ii_size[1];
  int i38;
  int ind_l2_size[1];
  double ind_l2_data[80];
  double z_all_l_data[80];
  int z_all_l_size[1];
  double status_ind_data[80];
  boolean_T exitg5;
  boolean_T guard1 = false;
  int ii_size_idx_0;
  int ind_r_size[1];
  double ind_r_data[40];
  int z_all_r_size[1];
  int anchorIdx;
  int featureIdx;
  boolean_T fix_new_feature;
  double uncertainties[8];
  signed char active_feature[8];
  int iidx[8];
  emxArray_real_T *qualities;
  emxArray_real_T *anchorInd;
  emxArray_real_T *featureInd;
  emxArray_int32_T *b_iidx;
  double new_m_data[240];
  boolean_T triangulation_success_data[40];
  double z_curr_l[2];
  double z_curr_r[2];
  boolean_T success;
  double b_m[6];
  double new_origin_pos_rel[3];
  double m_l[3];
  boolean_T bv0[3];
  double h_u_r[2];
  double h_u_l[2];
  double b_h_u_l[2];
  boolean_T b_guard1 = false;
  double mtmp;
  int trueCount;
  double triangulated_depths_data[40];
  signed char triangulated_status_ind_data[40];
  signed char b_triangulated_status_ind_data[40];
  int c_triangulated_status_ind_size_;
  int ix;
  double untriangulated_depths_data[40];
  emxArray_real_T *untriangulated_idx;
  int tmp_data[40];
  signed char untriangulated_status_ind_data[40];
  double b_new_m_data[120];
  double c_new_m_data[120];
  int new_feature_idx;
  boolean_T exitg3;
  long i39;
  static double J[8281];
  static const signed char iv9[294] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static double b_J[8281];
  boolean_T exitg4;
  double b_uncertainties[40];
  boolean_T has_active_features;
  int uncertainties_size[1];
  double median_uncertainty;
  double totalNumActiveFeatues;
  double numDelayedFeatures;
  unsigned int delayedIdx;
  double numActivatedFeatures;
  boolean_T request_new_features;
  boolean_T exitg2;
  double c_uncertainties[5];
  double b_has_active_features[5];
  double b_P_apr[36];
  boolean_T exitg1;
  char cv28[63];
  static const char cv29[63] = { 'P', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'w', 'i', 't', 'h', ' ', 'n', 'o',
    ' ', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e',
    's', ' ', 'a', 's', ' ', 'o', 'r', 'i', 'g', 'i', 'n', ' ', '(', 'a', 'n',
    'c', 'h', 'o', 'r', ' ', '%', 'd', ')', '\x00' };

  double new_origin_att_rel[9];
  double c_xt[9];
  double d_xt[3];
  double e_xt[9];
  double f_xt[9];
  double g_xt[9];
  double h_xt[9];

  //  undistort all valid points
  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] != 0);
  }

  idx = 0;
  ixstart = 1;
  exitg6 = false;
  while ((!exitg6) && (ixstart < 41)) {
    guard2 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = ixstart;
      if (idx >= 40) {
        exitg6 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      ixstart++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  ii_size[0] = loop_ub;
  for (i38 = 0; i38 < loop_ub; i38++) {
    b_ii_data[i38] = ii_data[i38];
  }

  multiplyIdx(b_ii_data, ii_size, ind_l2_data, ind_l2_size);
  z_all_l_size[0] = ind_l2_size[0];
  loop_ub = ind_l2_size[0];
  for (i38 = 0; i38 < loop_ub; i38++) {
    z_all_l_data[i38] = z_all_l[(int)ind_l2_data[i38] - 1];
  }

  undistortPoint(z_all_l_data, z_all_l_size, c_cameraParams_CameraParameters,
                 d_cameraParams_CameraParameters,
                 e_cameraParams_CameraParameters, status_ind_data, ii_size);
  loop_ub = ii_size[0];
  for (i38 = 0; i38 < loop_ub; i38++) {
    z_all_l[(int)ind_l2_data[i38] - 1] = status_ind_data[i38];
  }

  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] == 2);
  }

  idx = 0;
  ixstart = 1;
  exitg5 = false;
  while ((!exitg5) && (ixstart < 41)) {
    guard1 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = ixstart;
      if (idx >= 40) {
        exitg5 = true;
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

  if (1 > idx) {
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  ind_r_size[0] = loop_ub;
  for (i38 = 0; i38 < loop_ub; i38++) {
    ind_r_data[i38] = ii_data[i38];
  }

  multiplyIdx(ind_r_data, ind_r_size, ind_l2_data, ind_l2_size);
  z_all_r_size[0] = ind_l2_size[0];
  idx = ind_l2_size[0];
  for (i38 = 0; i38 < idx; i38++) {
    z_all_l_data[i38] = z_all_r[(int)ind_l2_data[i38] - 1];
  }

  undistortPoint(z_all_l_data, z_all_r_size, f_cameraParams_CameraParameters,
                 g_cameraParams_CameraParameters,
                 h_cameraParams_CameraParameters, status_ind_data, ii_size);
  idx = ii_size[0];
  for (i38 = 0; i38 < idx; i38++) {
    z_all_r[(int)ind_l2_data[i38] - 1] = status_ind_data[i38];
  }

  //  check for lost features
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
          && (updateVect[b_xt->anchor_states[anchorIdx]
              .feature_states[featureIdx].status_idx - 1] != 1)) {
        idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx;
        for (i38 = 0; i38 < 91; i38++) {
          P_apr[(idx + 91 * i38) - 1] = 0.0;
        }

        idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx;
        for (i38 = 0; i38 < 91; i38++) {
          P_apr[i38 + 91 * (idx - 1)] = 0.0;
        }

        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx = 0;

        //                  ros_info('Lost feature %i, which was %i on anchor %i', idx, featureIdx, anchorIdx) 
      }
    }
  }

  if (b_VIOParameters.fixed_feature) {
    fix_new_feature = false;
    if (b_xt->origin.anchor_idx != 0) {
      if (b_xt->fixed_feature != 0) {
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
            [b_xt->fixed_feature - 1].status != 1) {
          fix_new_feature = true;
          ros_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                   feature_states[b_xt->fixed_feature - 1].status_idx,
                   b_xt->fixed_feature, b_xt->origin.anchor_idx);
        }
      } else {
        fix_new_feature = true;
      }
    }

    if (fix_new_feature) {
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        active_feature[featureIdx] = 0;
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
            .feature_states[featureIdx].status == 1) {
          uncertainties[featureIdx] = P_apr[(b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].feature_states[featureIdx].P_idx + 91 *
            (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
             .feature_states[featureIdx].P_idx - 1)) - 1];
          active_feature[featureIdx] = 1;
        } else {
          uncertainties[featureIdx] = 1000.0;

          //  dont fix an inactive feature
        }
      }

      sort(uncertainties, iidx);
      for (i = 0; i < 8; i++) {
        uncertainties[i] = iidx[i];
      }

      if (!(active_feature[(int)uncertainties[0] - 1] != 0)) {
        ros_error();
      }

      b_xt->fixed_feature = (int)uncertainties[0];
      idx = b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
        [b_xt->fixed_feature - 1].P_idx;
      for (i38 = 0; i38 < 91; i38++) {
        P_apr[(idx + 91 * i38) - 1] = 0.0;
      }

      //  fix the feature depth
      idx = b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
        [b_xt->fixed_feature - 1].P_idx;
      for (i38 = 0; i38 < 91; i38++) {
        P_apr[i38 + 91 * (idx - 1)] = 0.0;
      }

      b_ros_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                 .feature_states[b_xt->fixed_feature - 1].status_idx,
                 b_xt->fixed_feature, b_xt->origin.anchor_idx);
    }
  }

  // % do the update
  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] == 1);
  }

  if (any(x)) {
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, c_cameraParams_CameraParameters,
                       d_cameraParams_CameraParameters,
                       noiseParameters_image_noise,
                       b_VIOParameters.max_ekf_iterations,
                       b_VIOParameters.delayed_initialization,
                       b_VIOParameters.RANSAC, updateVect);
  }

  // % Initialize new anchors/features
  emxInit_real_T(&qualities, 1);
  emxInit_real_T(&anchorInd, 1);
  emxInit_real_T(&featureInd, 1);
  emxInit_int32_T1(&b_iidx, 1);
  if (ii_size_idx_0 >= 4) {
    //  try to triangulate all new features
    for (i38 = 0; i38 < ii_size_idx_0; i38++) {
      ind_l2_data[i38] = 0.0;
    }

    idx = 3 * ii_size_idx_0;
    for (i38 = 0; i38 < idx; i38++) {
      new_m_data[i38] = 0.0;
    }

    for (i38 = 0; i38 < ii_size_idx_0; i38++) {
      triangulation_success_data[i38] = false;
    }

    for (i = 0; i < loop_ub; i++) {
      idx = ((int)ind_r_data[i] - 1) * 2;
      ixstart = ((int)ind_r_data[i] - 1) * 2;
      for (i38 = 0; i38 < 2; i38++) {
        z_curr_l[i38] = z_all_l[idx + i38];
        z_curr_r[i38] = z_all_r[ixstart + i38];
      }

      if (!b_VIOParameters.mono) {
        initializePoint(z_curr_l, z_curr_r, c_cameraParams_CameraParameters,
                        d_cameraParams_CameraParameters,
                        f_cameraParams_CameraParameters,
                        g_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, new_origin_pos_rel, b_m, &success);
        for (i38 = 0; i38 < 3; i38++) {
          m_l[i38] = b_m[i38];
        }

        if (success) {
          //  perform further checks
          for (idx = 0; idx < 3; idx++) {
            bv0[idx] = rtIsNaN(new_origin_pos_rel[idx]);
          }

          if (b_any(bv0)) {
            for (i38 = 0; i38 < 3; i38++) {
              new_origin_pos_rel[i38] = b_m[i38];
            }

            success = false;
          } else {
            //  check reprojection error
            predictMeasurementStereo(new_origin_pos_rel,
              c_cameraParams_CameraParameters, d_cameraParams_CameraParameters,
              f_cameraParams_CameraParameters, g_cameraParams_CameraParameters,
              cameraParams_r_lr, cameraParams_R_rl, h_u_l, h_u_r);
            for (idx = 0; idx < 2; idx++) {
              b_h_u_l[idx] = h_u_l[idx] - z_curr_l[idx];
            }

            b_guard1 = false;
            if (c_norm(b_h_u_l) > 2.0) {
              b_guard1 = true;
            } else {
              for (idx = 0; idx < 2; idx++) {
                b_h_u_l[idx] = h_u_r[idx] - z_curr_r[idx];
              }

              if (c_norm(b_h_u_l) > 2.0) {
                b_guard1 = true;
              } else {
                if (norm(new_origin_pos_rel) < 0.1) {
                  //  feature triangulated very close
                  for (i38 = 0; i38 < 3; i38++) {
                    new_origin_pos_rel[i38] = b_m[i38];
                  }

                  success = false;
                }
              }
            }

            if (b_guard1) {
              //                          ros_info('Bad triangulation (reprojection error) for point %d', int8(ind_r(i))); 
              for (i38 = 0; i38 < 3; i38++) {
                new_origin_pos_rel[i38] = b_m[i38];
              }

              success = false;
            }
          }
        } else {
          for (i38 = 0; i38 < 3; i38++) {
            new_origin_pos_rel[i38] = b_m[i38];
          }
        }
      } else {
        //  mono
        m_l[0] = (z_all_l[((int)ind_r_data[i] - 1) * 2] -
                  d_cameraParams_CameraParameters[0]) /
          c_cameraParams_CameraParameters[0];
        m_l[1] = (z_all_l[((int)ind_r_data[i] - 1) * 2 + 1] -
                  d_cameraParams_CameraParameters[1]) /
          c_cameraParams_CameraParameters[1];
        m_l[2] = 1.0;
        mtmp = norm(m_l);
        for (idx = 0; idx < 3; idx++) {
          new_origin_pos_rel[idx] = m_l[idx] / mtmp;
        }

        success = true;
      }

      ind_l2_data[i] = norm(new_origin_pos_rel);
      for (i38 = 0; i38 < 3; i38++) {
        new_m_data[i38 + 3 * i] = m_l[i38];
      }

      triangulation_success_data[i] = success;
    }

    idx = 0;
    for (ixstart = 0; ixstart < ii_size_idx_0; ixstart++) {
      if (triangulation_success_data[ixstart]) {
        idx++;
      }
    }

    d_ros_info(idx, loop_ub);
    idx = loop_ub - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        trueCount++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_depths_data[ixstart] = ind_l2_data[i];
        ixstart++;
      }
    }

    i38 = qualities->size[0];
    qualities->size[0] = trueCount;
    emxEnsureCapacity((emxArray__common *)qualities, i38, (int)sizeof(double));
    for (i38 = 0; i38 < trueCount; i38++) {
      qualities->data[i38] = triangulated_depths_data[i38];
    }

    c_sort(qualities, b_iidx);
    i38 = anchorInd->size[0];
    anchorInd->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)anchorInd, i38, (int)sizeof(double));
    idx = b_iidx->size[0];
    for (i38 = 0; i38 < idx; i38++) {
      anchorInd->data[i38] = b_iidx->data[i38];
    }

    idx = loop_ub - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        trueCount++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        ii_data[ixstart] = i + 1;
        ixstart++;
      }
    }

    idx = loop_ub - 1;
    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_status_ind_data[ixstart] = (signed char)ind_r_data[i];
        ixstart++;
      }
    }

    c_triangulated_status_ind_size_ = anchorInd->size[0];
    idx = anchorInd->size[0];
    for (i38 = 0; i38 < idx; i38++) {
      b_triangulated_status_ind_data[i38] = triangulated_status_ind_data[(int)
        anchorInd->data[i38] - 1];
    }

    for (i38 = 0; i38 < c_triangulated_status_ind_size_; i38++) {
      triangulated_status_ind_data[i38] = b_triangulated_status_ind_data[i38];
    }

    idx = loop_ub - 1;
    ix = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        ix++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_depths_data[ixstart] = ind_l2_data[i];
        ixstart++;
      }
    }

    i38 = featureInd->size[0];
    featureInd->size[0] = ix;
    emxEnsureCapacity((emxArray__common *)featureInd, i38, (int)sizeof(double));
    for (i38 = 0; i38 < ix; i38++) {
      featureInd->data[i38] = untriangulated_depths_data[i38];
    }

    emxInit_real_T(&untriangulated_idx, 1);
    c_sort(featureInd, b_iidx);
    i38 = untriangulated_idx->size[0];
    untriangulated_idx->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)untriangulated_idx, i38, (int)sizeof
                      (double));
    idx = b_iidx->size[0];
    for (i38 = 0; i38 < idx; i38++) {
      untriangulated_idx->data[i38] = b_iidx->data[i38];
    }

    idx = loop_ub - 1;
    ix = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        ix++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        tmp_data[ixstart] = i + 1;
        ixstart++;
      }
    }

    idx = loop_ub - 1;
    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_status_ind_data[ixstart] = (signed char)ind_r_data[i];
        ixstart++;
      }
    }

    idx = untriangulated_idx->size[0];
    loop_ub = untriangulated_idx->size[0];
    for (i38 = 0; i38 < loop_ub; i38++) {
      b_triangulated_status_ind_data[i38] = untriangulated_status_ind_data[(int)
        untriangulated_idx->data[i38] - 1];
    }

    for (i38 = 0; i38 < idx; i38++) {
      untriangulated_status_ind_data[i38] = b_triangulated_status_ind_data[i38];
    }

    ind_l2_size[0] = qualities->size[0] + featureInd->size[0];
    loop_ub = qualities->size[0];
    for (i38 = 0; i38 < loop_ub; i38++) {
      ind_l2_data[i38] = qualities->data[i38];
    }

    loop_ub = featureInd->size[0];
    for (i38 = 0; i38 < loop_ub; i38++) {
      ind_l2_data[i38 + qualities->size[0]] = featureInd->data[i38];
    }

    for (i38 = 0; i38 < trueCount; i38++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        b_new_m_data[ixstart + 3 * i38] = new_m_data[ixstart + 3 * (ii_data[i38]
          - 1)];
      }
    }

    for (i38 = 0; i38 < ix; i38++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        c_new_m_data[ixstart + 3 * i38] = new_m_data[ixstart + 3 * (tmp_data[i38]
          - 1)];
      }
    }

    loop_ub = anchorInd->size[0];
    for (i38 = 0; i38 < loop_ub; i38++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        new_m_data[ixstart + 3 * i38] = b_new_m_data[ixstart + 3 * ((int)
          anchorInd->data[i38] - 1)];
      }
    }

    loop_ub = untriangulated_idx->size[0];
    for (i38 = 0; i38 < loop_ub; i38++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        new_m_data[ixstart + 3 * (i38 + anchorInd->size[0])] =
          c_new_m_data[ixstart + 3 * ((int)untriangulated_idx->data[i38] - 1)];
      }
    }

    emxFree_real_T(&untriangulated_idx);
    for (i38 = 0; i38 < c_triangulated_status_ind_size_; i38++) {
      status_ind_data[i38] = triangulated_status_ind_data[i38];
    }

    for (i38 = 0; i38 < idx; i38++) {
      status_ind_data[i38 + c_triangulated_status_ind_size_] =
        untriangulated_status_ind_data[i38];
    }

    //  insert new features into the state. first the closest triangulated ones, 
    //  then the un-triangulated ones
    new_feature_idx = 0;
    anchorIdx = 0;
    exitg3 = false;
    while ((!exitg3) && (anchorIdx + 1 < 6)) {
      //          if new_feature_idx > length(new_depths)
      idx = 0;
      for (ixstart = 0; ixstart < ii_size_idx_0; ixstart++) {
        if (triangulation_success_data[ixstart]) {
          idx++;
        }
      }

      if ((new_feature_idx + 1 > idx) || (ind_l2_size[0] - new_feature_idx < 4))
      {
        exitg3 = true;
      } else {
        if (getNumValidFeatures(b_xt->anchor_states[anchorIdx].feature_states) <
            4.0) {
          //  anchor needs to be initialized
          //  free up updateVect
          for (featureIdx = 0; featureIdx < 8; featureIdx++) {
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                != 0) {
              //                      ros_info('clearing up feature %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
            }
          }

          if (b_xt->origin.anchor_idx == anchorIdx + 1) {
            b_xt->origin.anchor_idx = 0;
            e_ros_info(anchorIdx + 1);
          }

          for (i38 = 0; i38 < 3; i38++) {
            b_xt->anchor_states[anchorIdx].pos[i38] = b_xt->robot_state.pos[i38];
          }

          for (i38 = 0; i38 < 4; i38++) {
            b_xt->anchor_states[anchorIdx].att[i38] = b_xt->robot_state.att[i38];
          }

          i38 = anchorIdx * 14;
          for (ixstart = 0; ixstart < 6; ixstart++) {
            b_xt->anchor_states[anchorIdx].P_idx[ixstart] = (ixstart + i38) + 22;
          }

          for (i38 = 0; i38 < 91; i38++) {
            for (ixstart = 0; ixstart < 6; ixstart++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[ixstart] + 91 * i38) -
                1] = 0.0;
            }
          }

          for (i38 = 0; i38 < 6; i38++) {
            for (ixstart = 0; ixstart < 91; ixstart++) {
              P_apr[ixstart + 91 * (b_xt->anchor_states[anchorIdx].P_idx[i38] -
                                    1)] = 0.0;
            }
          }

          idx = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i38 = 0; i38 < 91; i38++) {
            for (ixstart = 0; ixstart < 8; ixstart++) {
              i39 = (long)idx + (1 + ixstart);
              if (i39 > 2147483647L) {
                i39 = 2147483647L;
              } else {
                if (i39 < -2147483648L) {
                  i39 = -2147483648L;
                }
              }

              P_apr[((int)i39 + 91 * i38) - 1] = 0.0;
            }
          }

          idx = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i38 = 0; i38 < 8; i38++) {
            for (ixstart = 0; ixstart < 91; ixstart++) {
              i39 = (long)idx + (1 + i38);
              if (i39 > 2147483647L) {
                i39 = 2147483647L;
              } else {
                if (i39 < -2147483648L) {
                  i39 = -2147483648L;
                }
              }

              P_apr[ixstart + 91 * ((int)i39 - 1)] = 0.0;
            }
          }

          d_eye(J);
          i38 = anchorIdx * 14;
          for (ixstart = 0; ixstart < 91; ixstart++) {
            memset(&J[(i38 + 91 * ixstart) + 21], 0, 14U * sizeof(double));
          }

          i38 = anchorIdx * 14;
          for (ixstart = 0; ixstart < 21; ixstart++) {
            for (idx = 0; idx < 14; idx++) {
              J[((idx + i38) + 91 * ixstart) + 21] = iv9[idx + 14 * ixstart];
            }
          }

          for (i38 = 0; i38 < 91; i38++) {
            for (ixstart = 0; ixstart < 91; ixstart++) {
              b_J[i38 + 91 * ixstart] = 0.0;
              for (idx = 0; idx < 91; idx++) {
                b_J[i38 + 91 * ixstart] += J[i38 + 91 * idx] * P_apr[idx + 91 *
                  ixstart];
              }
            }
          }

          for (i38 = 0; i38 < 91; i38++) {
            for (ixstart = 0; ixstart < 91; ixstart++) {
              P_apr[i38 + 91 * ixstart] = 0.0;
              for (idx = 0; idx < 91; idx++) {
                P_apr[i38 + 91 * ixstart] += b_J[i38 + 91 * idx] * J[ixstart +
                  91 * idx];
              }
            }
          }

          featureIdx = 0;
          exitg4 = false;
          while ((!exitg4) && (featureIdx + 1 < 9)) {
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth = 1.0 / ind_l2_data[new_feature_idx];
            for (i38 = 0; i38 < 3; i38++) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].m[i38] =
                new_m_data[i38 + 3 * new_feature_idx];
            }

            if (b_VIOParameters.delayed_initialization) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                2;
            } else {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }

            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx
              = (int)status_ind_data[new_feature_idx];
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx =
              (anchorIdx * 14 + featureIdx) + 28;
            idx = 0;
            for (ixstart = 0; ixstart < ii_size_idx_0; ixstart++) {
              if (triangulation_success_data[ixstart]) {
                idx++;
              }
            }

            if (new_feature_idx + 1 > idx) {
              f_ros_info(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx);
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 91 * (b_xt->anchor_states[anchorIdx]
                                   .feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini * 10.0;

              //  TODO: Maybe push the mean value further away?
            } else {
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 91 * (b_xt->anchor_states[anchorIdx]
                                   .feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini;

              // *new_depths(new_feature_idx);
            }

            g_ros_info((int)status_ind_data[new_feature_idx], featureIdx + 1,
                       anchorIdx + 1);
            updateVect[(int)status_ind_data[new_feature_idx] - 1] = 1;
            new_feature_idx++;

            //                  if new_feature_idx > length(new_depths)
            idx = 0;
            for (ixstart = 0; ixstart < ii_size_idx_0; ixstart++) {
              if (triangulation_success_data[ixstart]) {
                idx++;
              }
            }

            if (new_feature_idx + 1 > idx) {
              exitg4 = true;
            } else {
              featureIdx++;
            }
          }
        }

        anchorIdx++;
      }
    }

    for (i = 0; i < 40; i++) {
      if (updateVect[i] == 2) {
        updateVect[i] = 0;
      }
    }

    //  remove features that were not inserted
  }

  if (b_VIOParameters.delayed_initialization) {
    //  get the median uncertainty of the active features as a benchmark on
    //  the delayed features
    for (i = 0; i < 40; i++) {
      b_uncertainties[i] = -1.0;
    }

    has_active_features = false;
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            1) {
          has_active_features = true;
          b_uncertainties[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = P_apr
            [(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx +
              91 * (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                    P_idx - 1)) - 1];
        }
      }
    }

    if (has_active_features) {
      trueCount = 0;
      for (i = 0; i < 40; i++) {
        if (b_uncertainties[i] > 0.0) {
          trueCount++;
        }
      }

      ixstart = 0;
      for (i = 0; i < 40; i++) {
        if (b_uncertainties[i] > 0.0) {
          ii_data[ixstart] = i + 1;
          ixstart++;
        }
      }

      uncertainties_size[0] = trueCount;
      for (i38 = 0; i38 < trueCount; i38++) {
        b_ii_data[i38] = b_uncertainties[ii_data[i38] - 1];
      }

      median_uncertainty = median(b_ii_data, uncertainties_size);

      //  because coder does not support nanflag
      //  check if a delayed initialization feature has converged
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
               2) && (P_apr[(b_xt->anchor_states[anchorIdx]
                             .feature_states[featureIdx].P_idx + 91 *
                             (b_xt->anchor_states[anchorIdx]
                              .feature_states[featureIdx].P_idx - 1)) - 1] <
                      median_uncertainty * 2.0)) {
            //  this feature is not active yet
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
            } else {
              //                              ros_info('Feature %i (%i on anchor %i) has converged', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }
          }
        }
      }
    }

    //  check if delayed features need to be forced active due to too few active 
    //  features
    totalNumActiveFeatues = getTotalNumActiveFeatures(b_xt->anchor_states);
    if (totalNumActiveFeatues < 16.0) {
      //  find the best features and activate them
      numDelayedFeatures = getTotalNumDelayedFeatures(b_xt->anchor_states);
      i38 = qualities->size[0];
      qualities->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)qualities, i38, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i38 = 0; i38 < loop_ub; i38++) {
        qualities->data[i38] = 0.0;
      }

      //  quality measures of each delayed feature
      i38 = anchorInd->size[0];
      anchorInd->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)anchorInd, i38, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i38 = 0; i38 < loop_ub; i38++) {
        anchorInd->data[i38] = 0.0;
      }

      i38 = featureInd->size[0];
      featureInd->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)featureInd, i38, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i38 = 0; i38 < loop_ub; i38++) {
        featureInd->data[i38] = 0.0;
      }

      delayedIdx = 1U;
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              2) {
            qualities->data[(int)delayedIdx - 1] = P_apr[(b_xt->
              anchor_states[anchorIdx].feature_states[featureIdx].P_idx + 91 *
              (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx -
               1)) - 1] / c_noiseParameters_inv_depth_ini;
            anchorInd->data[(int)delayedIdx - 1] = (double)anchorIdx + 1.0;
            featureInd->data[(int)delayedIdx - 1] = (double)featureIdx + 1.0;
            delayedIdx++;
          }
        }
      }

      c_sort(qualities, b_iidx);
      i38 = qualities->size[0];
      qualities->size[0] = b_iidx->size[0];
      emxEnsureCapacity((emxArray__common *)qualities, i38, (int)sizeof(double));
      loop_ub = b_iidx->size[0];
      for (i38 = 0; i38 < loop_ub; i38++) {
        qualities->data[i38] = b_iidx->data[i38];
      }

      numActivatedFeatures = 0.0;
      i = 0;
      while ((i <= qualities->size[0] - 1) && (!(numActivatedFeatures > 16.0F -
               (float)totalNumActiveFeatues)) && (!(numActivatedFeatures >
               numDelayedFeatures))) {
        if (b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].inverse_depth < 0.0) {
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].status = 0;
          updateVect[b_xt->anchor_states[(int)anchorInd->data[(int)
            qualities->data[i] - 1] - 1].feature_states[(int)featureInd->data
            [(int)qualities->data[i] - 1] - 1].status_idx - 1] = 0;
        } else {
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].status = 1;
          h_ros_info(b_xt->anchor_states[(int)anchorInd->data[(int)
                     qualities->data[i] - 1] - 1].feature_states[(int)
                     featureInd->data[(int)qualities->data[i] - 1] - 1].
                     status_idx, featureInd->data[(int)qualities->data[i] - 1],
                     anchorInd->data[(int)qualities->data[i] - 1]);
          numActivatedFeatures++;
        }

        i++;
      }
    }
  }

  emxFree_int32_T(&b_iidx);
  emxFree_real_T(&featureInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&qualities);

  //  check if new features need to be requested
  request_new_features = false;
  idx = 0;
  for (ixstart = 0; ixstart < 40; ixstart++) {
    if (updateVect[ixstart] == 0) {
      idx++;
    }
  }

  if (idx > 4) {
    //  if a new anchor can be filled enough
    anchorIdx = 1;
    exitg2 = false;
    while ((!exitg2) && (anchorIdx < 6)) {
      if (getNumValidFeatures(b_xt->anchor_states[anchorIdx - 1].feature_states)
          < 4.0) {
        request_new_features = true;
        exitg2 = true;
      } else {
        anchorIdx++;
      }
    }
  } else {
    //  debug check
  }

  if (request_new_features) {
    for (i = 0; i < 40; i++) {
      if (updateVect[i] == 0) {
        updateVect[i] = 2;
      }
    }

    //  get as many new features as possible
  }

  // % robocentric update
  if (b_xt->origin.anchor_idx == 0) {
    //  need to update the origin anchor and the state
    b_xt->fixed_feature = 0;

    //  ensure that a new feature will be fixed, if this option is enabled
    //  choose the best anchor as the new origin anchor
    //  uncertainties of the anchors reduced to a scalar
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      b_has_active_features[anchorIdx] = 0.0;
      if (anyActiveAnchorFeatures(b_xt->anchor_states[anchorIdx].feature_states))
      {
        for (i38 = 0; i38 < 6; i38++) {
          for (ixstart = 0; ixstart < 6; ixstart++) {
            b_P_apr[ixstart + 6 * i38] = P_apr[(b_xt->anchor_states[anchorIdx].
              P_idx[ixstart] + 91 * (b_xt->anchor_states[anchorIdx].P_idx[i38] -
              1)) - 1];
          }
        }

        c_uncertainties[anchorIdx] = det(b_P_apr);
        b_has_active_features[anchorIdx] = 1.0;
      } else {
        c_uncertainties[anchorIdx] = 1000.0;

        //  dont fix an anchor with no active features
      }
    }

    if (!c_any(b_has_active_features)) {
      //  can happen if outlier rejection rejected all features
    } else {
      ixstart = 1;
      mtmp = c_uncertainties[0];
      idx = 1;
      if (rtIsNaN(c_uncertainties[0])) {
        ix = 2;
        exitg1 = false;
        while ((!exitg1) && (ix < 6)) {
          ixstart = ix;
          if (!rtIsNaN(c_uncertainties[ix - 1])) {
            mtmp = c_uncertainties[ix - 1];
            idx = ix;
            exitg1 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < 5) {
        while (ixstart + 1 < 6) {
          if (c_uncertainties[ixstart] < mtmp) {
            mtmp = c_uncertainties[ixstart];
            idx = ixstart + 1;
          }

          ixstart++;
        }
      }

      b_xt->origin.anchor_idx = idx;
      if (!(b_has_active_features[b_xt->origin.anchor_idx - 1] != 0.0)) {
        //  debug check
        // #coder
        // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
        for (i38 = 0; i38 < 63; i38++) {
          cv28[i38] = cv29[i38];
        }

        ROS_ERROR(cv28, b_xt->origin.anchor_idx);
      } else {
        i_ros_info(b_xt->origin.anchor_idx);

        //  in old origin frame
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        new_origin_att_rel[0] = ((b_xt->anchor_states[b_xt->origin.anchor_idx -
          1].att[0] * b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[0] -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1]) -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3];
        new_origin_att_rel[3] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[6] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] - b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[1] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] - b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[4] = ((-(b_xt->anchor_states[b_xt->origin.anchor_idx
          - 1].att[0] * b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[0])
          + b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1]) -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3];
        new_origin_att_rel[7] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[2] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[5] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] - b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[8] = ((-(b_xt->anchor_states[b_xt->origin.anchor_idx
          - 1].att[0] * b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[0])
          - b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3];

        //  in old origin frame, = R_o{k+1}o{k}
        d_eye(J);

        //  robot position and orientation
        eye(c_xt);
        for (i = 0; i < 3; i++) {
          new_origin_pos_rel[i] = b_xt->anchor_states[b_xt->origin.anchor_idx -
            1].pos[i];
          for (i38 = 0; i38 < 3; i38++) {
            J[i38 + 91 * i] = new_origin_att_rel[i38 + 3 * i];
            J[i38 + 91 * (i + 3)] = 0.0;
            J[(i38 + 91 * i) + 3] = 0.0;
            J[(i38 + 91 * (i + 3)) + 3] = c_xt[i38 + 3 * i];
          }

          d_xt[i] = b_xt->robot_state.pos[i] - b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].pos[i];
        }

        for (i38 = 0; i38 < 3; i38++) {
          m_l[i38] = 0.0;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            J[ixstart + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                              P_idx[i38] - 1)] = -new_origin_att_rel[ixstart + 3
              * i38];
            J[(ixstart + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                               P_idx[i38] - 1)) + 3] = 0.0;
            m_l[i38] += new_origin_att_rel[i38 + 3 * ixstart] * d_xt[ixstart];
          }
        }

        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1)] =
          0.0;
        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1)] =
          -m_l[2];
        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1)] =
          m_l[1];
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                    1)] = m_l[2];
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                    1)] = 0.0;
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                    1)] = -m_l[0];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                    1)] = -m_l[1];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                    1)] = m_l[0];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                    1)] = 0.0;

        //  robot velocity
        //  velocity
        //  origin rotation
        for (i38 = 0; i38 < 3; i38++) {
          for (ixstart = 0; ixstart < 3; ixstart++) {
            J[(ixstart + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                               P_idx[i38 + 3] - 1)) + 3] =
              -new_origin_att_rel[i38 + 3 * ixstart];
            J[(ixstart + 91 * (6 + i38)) + 6] = new_origin_att_rel[ixstart + 3 *
              i38];
            J[(ixstart + 91 * (15 + i38)) + 15] = new_origin_att_rel[ixstart + 3
              * i38];
          }
        }

        //  origin rotation
        for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
          if (anchorIdx + 1 == b_xt->origin.anchor_idx) {
            //  remove yaw uncertainty, but not pitch or roll
            for (i38 = 0; i38 < 6; i38++) {
              for (ixstart = 0; ixstart < 6; ixstart++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[ixstart] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i38] - 1)) - 1] = 0.0;
              }
            }

            //  TODO: allow roll/pitch uncertainty
          } else {
            eye(c_xt);
            for (i38 = 0; i38 < 3; i38++) {
              for (ixstart = 0; ixstart < 3; ixstart++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[ixstart] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i38] - 1)) - 1] =
                  new_origin_att_rel[ixstart + 3 * i38];
              }
            }

            for (i38 = 0; i38 < 3; i38++) {
              for (ixstart = 0; ixstart < 3; ixstart++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[ixstart] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i38 + 3] - 1)) - 1] =
                  0.0;
              }
            }

            for (i38 = 0; i38 < 3; i38++) {
              for (ixstart = 0; ixstart < 3; ixstart++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[ixstart + 3] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i38] - 1)) - 1] = 0.0;
              }
            }

            for (i38 = 0; i38 < 3; i38++) {
              for (ixstart = 0; ixstart < 3; ixstart++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[ixstart + 3] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i38 + 3] - 1)) - 1] =
                  c_xt[ixstart + 3 * i38];
              }

              d_xt[i38] = b_xt->anchor_states[anchorIdx].pos[i38] -
                new_origin_pos_rel[i38];
            }

            for (i38 = 0; i38 < 3; i38++) {
              m_l[i38] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[ixstart] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i38]
                    - 1)) - 1] = -new_origin_att_rel[ixstart + 3 * i38];
                m_l[i38] += new_origin_att_rel[i38 + 3 * ixstart] * d_xt[ixstart];
              }
            }

            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = 0.0;
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = -m_l[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = m_l[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = m_l[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = 0.0;
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = -m_l[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = -m_l[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = m_l[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = 0.0;
            for (i38 = 0; i38 < 3; i38++) {
              for (ixstart = 0; ixstart < 3; ixstart++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[ixstart + 3] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i38]
                    - 1)) - 1] = 0.0;
              }
            }

            for (i38 = 0; i38 < 3; i38++) {
              for (ixstart = 0; ixstart < 3; ixstart++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[ixstart + 3] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i38 +
                    3] - 1)) - 1] = -new_origin_att_rel[i38 + 3 * ixstart];
              }
            }
          }

          for (i = 0; i < 3; i++) {
            m_l[i] = b_xt->anchor_states[anchorIdx].pos[i] -
              new_origin_pos_rel[i];
            b_xt->anchor_states[anchorIdx].pos[i] = 0.0;
          }

          //  if ~all(size(q) == [4, 1])
          //      error('q does not have the size of a quaternion')
          //  end
          //  if abs(norm(q) - 1) > 1e-3
          //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
          //  end
          e_xt[0] = ((b_xt->anchor_states[anchorIdx].att[0] *
                      b_xt->anchor_states[anchorIdx].att[0] -
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) -
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          e_xt[3] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[1] +
                           b_xt->anchor_states[anchorIdx].att[2] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[6] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[2] -
                           b_xt->anchor_states[anchorIdx].att[1] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[1] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[1] -
                           b_xt->anchor_states[anchorIdx].att[2] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0]) +
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) -
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          e_xt[7] = 2.0 * (b_xt->anchor_states[anchorIdx].att[1] *
                           b_xt->anchor_states[anchorIdx].att[2] +
                           b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[2] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[2] +
                           b_xt->anchor_states[anchorIdx].att[1] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[5] = 2.0 * (b_xt->anchor_states[anchorIdx].att[1] *
                           b_xt->anchor_states[anchorIdx].att[2] -
                           b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0]) -
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) +
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          for (i38 = 0; i38 < 3; i38++) {
            b_xt->anchor_states[anchorIdx].pos[i38] = 0.0;
            for (ixstart = 0; ixstart < 3; ixstart++) {
              b_xt->anchor_states[anchorIdx].pos[i38] += new_origin_att_rel[i38
                + 3 * ixstart] * m_l[ixstart];
              c_xt[i38 + 3 * ixstart] = 0.0;
              for (idx = 0; idx < 3; idx++) {
                c_xt[i38 + 3 * ixstart] += e_xt[i38 + 3 * idx] *
                  new_origin_att_rel[ixstart + 3 * idx];
              }
            }
          }

          QuatFromRotJ(c_xt, b_xt->anchor_states[anchorIdx].att);
        }

        for (i38 = 0; i38 < 91; i38++) {
          for (ixstart = 0; ixstart < 91; ixstart++) {
            b_J[i38 + 91 * ixstart] = 0.0;
            for (idx = 0; idx < 91; idx++) {
              b_J[i38 + 91 * ixstart] += J[i38 + 91 * idx] * P_apr[idx + 91 *
                ixstart];
            }
          }
        }

        for (i38 = 0; i38 < 91; i38++) {
          for (ixstart = 0; ixstart < 91; ixstart++) {
            P_apr[i38 + 91 * ixstart] = 0.0;
            for (idx = 0; idx < 91; idx++) {
              P_apr[i38 + 91 * ixstart] += b_J[i38 + 91 * idx] * J[ixstart + 91 *
                idx];
            }
          }
        }

        for (i38 = 0; i38 < 3; i38++) {
          d_xt[i38] = b_xt->robot_state.pos[i38] - new_origin_pos_rel[i38];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        f_xt[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        f_xt[3] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                         b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        f_xt[6] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                         b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        f_xt[1] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                         b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        f_xt[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        f_xt[7] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                         b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        f_xt[2] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                         b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        f_xt[5] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                         b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        f_xt[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        for (i38 = 0; i38 < 3; i38++) {
          b_xt->robot_state.pos[i38] = 0.0;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            b_xt->robot_state.pos[i38] += new_origin_att_rel[i38 + 3 * ixstart] *
              d_xt[ixstart];
            c_xt[i38 + 3 * ixstart] = 0.0;
            for (idx = 0; idx < 3; idx++) {
              c_xt[i38 + 3 * ixstart] += f_xt[i38 + 3 * idx] *
                new_origin_att_rel[ixstart + 3 * idx];
            }
          }
        }

        QuatFromRotJ(c_xt, b_xt->robot_state.att);
        for (i38 = 0; i38 < 3; i38++) {
          d_xt[i38] = 0.0;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            d_xt[i38] += new_origin_att_rel[i38 + 3 * ixstart] *
              b_xt->robot_state.vel[ixstart];
          }
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        g_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        g_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        g_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        g_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        g_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        g_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        g_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        g_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        g_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];

        //  in world frame
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        h_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        h_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        h_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        h_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        h_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        h_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        h_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        h_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        h_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        for (i38 = 0; i38 < 3; i38++) {
          b_xt->robot_state.vel[i38] = d_xt[i38];
          mtmp = 0.0;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            mtmp += g_xt[i38 + 3 * ixstart] * new_origin_pos_rel[ixstart];
            c_xt[i38 + 3 * ixstart] = 0.0;
            for (idx = 0; idx < 3; idx++) {
              c_xt[i38 + 3 * ixstart] += new_origin_att_rel[i38 + 3 * idx] *
                h_xt[idx + 3 * ixstart];
            }
          }

          b_xt->origin.pos[i38] += mtmp;
        }

        QuatFromRotJ(c_xt, b_xt->origin.att);

        //  in world frame
      }
    }
  }

  // % aposteriori measurement prediction
  getMap(b_xt->origin.pos, b_xt->origin.att, b_xt->anchor_states, b_map);

  //  get map for output
  getScaledMap(b_xt);

  //  update the scaled map for measurement prediction
  memset(&h_u_apo[0], 0, 160U * sizeof(double));
  memset(&b_delayedStatus[0], 0, 40U * sizeof(double));
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
      {
        c_predictMeasurementStereoDisto(b_xt->anchor_states[anchorIdx].
          feature_states[featureIdx].scaled_map_point,
          c_cameraParams_CameraParameters, d_cameraParams_CameraParameters,
          e_cameraParams_CameraParameters, f_cameraParams_CameraParameters,
          g_cameraParams_CameraParameters, h_cameraParams_CameraParameters,
          cameraParams_r_lr, cameraParams_R_rl, h_u_l, h_u_r);
        i39 = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1L;
        if (i39 > 2147483647L) {
          i39 = 2147483647L;
        } else {
          if (i39 < -2147483648L) {
            i39 = -2147483648L;
          }
        }

        i38 = (int)i39;
        if (i38 > 536870911) {
          idx = MAX_int32_T;
        } else if (i38 <= -536870912) {
          idx = MIN_int32_T;
        } else {
          idx = i38 << 2;
        }

        for (i = 0; i < 2; i++) {
          i39 = (long)idx + (i + 1);
          if (i39 > 2147483647L) {
            i39 = 2147483647L;
          } else {
            if (i39 < -2147483648L) {
              i39 = -2147483648L;
            }
          }

          h_u_apo[(int)i39 - 1] = h_u_l[i];
        }

        for (i = 0; i < 2; i++) {
          i39 = (long)idx + (i + 3);
          if (i39 > 2147483647L) {
            i39 = 2147483647L;
          } else {
            if (i39 < -2147483648L) {
              i39 = -2147483648L;
            }
          }

          h_u_apo[(int)i39 - 1] = h_u_r[i];
        }

        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          b_delayedStatus[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = 1.0;
        }
      }
    }
  }
}

//
// Arguments    : const boolean_T x[40]
// Return Type  : boolean_T
//
static boolean_T any(const boolean_T x[40])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 40)) {
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
// Arguments    : const e_struct_T anchor_state_feature_states[8]
// Return Type  : boolean_T
//
static boolean_T anyActiveAnchorFeatures(const e_struct_T
  anchor_state_feature_states[8])
{
  boolean_T ret;
  int featureIdx;
  boolean_T exitg1;
  ret = false;
  featureIdx = 1;
  exitg1 = false;
  while ((!exitg1) && (featureIdx < 9)) {
    if (anchor_state_feature_states[featureIdx - 1].status == 1) {
      ret = true;
      exitg1 = true;
    } else {
      featureIdx++;
    }
  }

  return ret;
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
static boolean_T b_any(const boolean_T x[3])
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
// Arguments    : const double v[15]
//                double d[225]
// Return Type  : void
//
static void b_diag(const double v[15], double d[225])
{
  int j;
  memset(&d[0], 0, 225U * sizeof(double));
  for (j = 0; j < 15; j++) {
    d[j + 15 * j] = v[j];
  }
}

//
// Arguments    : double I[441]
// Return Type  : void
//
static void b_eye(double I[441])
{
  int k;
  memset(&I[0], 0, 441U * sizeof(double));
  for (k = 0; k < 21; k++) {
    I[k + 21 * k] = 1.0;
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double b_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[2] = { '\x0a', '\x00' };

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
// Arguments    : emxArray_int32_T *idx
//                emxArray_real_T *x
//                int offset
//                int np
//                int nq
//                emxArray_int32_T *iwork
//                emxArray_real_T *xwork
// Return Type  : void
//
static void b_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  int n;
  int qend;
  int p;
  int iout;
  int exitg1;
  if (nq == 0) {
  } else {
    n = np + nq;
    for (qend = 0; qend + 1 <= n; qend++) {
      iwork->data[qend] = idx->data[offset + qend];
      xwork->data[qend] = x->data[offset + qend];
    }

    p = 0;
    n = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork->data[p] <= xwork->data[n]) {
        idx->data[iout] = iwork->data[p];
        x->data[iout] = xwork->data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx->data[iout] = iwork->data[n];
        x->data[iout] = xwork->data[n];
        if (n + 1 < qend) {
          n++;
        } else {
          n = (iout - p) + 1;
          while (p + 1 <= np) {
            idx->data[n + p] = iwork->data[p];
            x->data[n + p] = xwork->data[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
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
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
static void b_ros_error()
{
  char cv12[14];
  int i16;
  static const char cv13[14] = { 'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't',
    'e', 'n', 'c', 'y', '\x00' };

  for (i16 = 0; i16 < 14; i16++) {
    cv12[i16] = cv13[i16];
  }

  ROS_ERROR(cv12);
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void b_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv10[44];
  int i14;
  static const char cv11[44] = { 'F', 'i', 'x', 'i', 'n', 'g', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', 'f', 'e', 'a', 't', 'u',
    'r', 'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r',
    ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i14 = 0; i14 < 44; i14++) {
      cv10[i14] = cv11[i14];
    }

    ROS_INFO(cv10, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : double x[8]
//                int idx[8]
// Return Type  : void
//
static void b_sort(double x[8], int idx[8])
{
  int b_m;
  double x4[4];
  signed char idx4[4];
  double xwork[8];
  int nNaNs;
  int ib;
  int k;
  int bLen;
  int nPairs;
  int i4;
  signed char perm[4];
  int iwork[8];
  for (b_m = 0; b_m < 8; b_m++) {
    idx[b_m] = 0;
  }

  for (b_m = 0; b_m < 4; b_m++) {
    x4[b_m] = 0.0;
    idx4[b_m] = 0;
  }

  memset(&xwork[0], 0, sizeof(double) << 3);
  nNaNs = -7;
  ib = 0;
  for (k = 0; k < 8; k++) {
    if (rtIsNaN(x[k])) {
      idx[-nNaNs] = k + 1;
      xwork[-nNaNs] = x[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = (signed char)(k + 1);
      x4[ib - 1] = x[k];
      if (ib == 4) {
        ib = (k - nNaNs) - 10;
        if (x4[0] <= x4[1]) {
          b_m = 1;
          bLen = 2;
        } else {
          b_m = 2;
          bLen = 1;
        }

        if (x4[2] <= x4[3]) {
          nPairs = 3;
          i4 = 4;
        } else {
          nPairs = 4;
          i4 = 3;
        }

        if (x4[b_m - 1] <= x4[nPairs - 1]) {
          if (x4[bLen - 1] <= x4[nPairs - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)bLen;
            perm[2] = (signed char)nPairs;
            perm[3] = (signed char)i4;
          } else if (x4[bLen - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)nPairs;
            perm[2] = (signed char)bLen;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)nPairs;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)bLen;
          }
        } else if (x4[b_m - 1] <= x4[i4 - 1]) {
          if (x4[bLen - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)nPairs;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)bLen;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)nPairs;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)bLen;
          }
        } else {
          perm[0] = (signed char)nPairs;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)b_m;
          perm[3] = (signed char)bLen;
        }

        idx[ib] = idx4[perm[0] - 1];
        idx[ib + 1] = idx4[perm[1] - 1];
        idx[ib + 2] = idx4[perm[2] - 1];
        idx[ib + 3] = idx4[perm[3] - 1];
        x[ib] = x4[perm[0] - 1];
        x[ib + 1] = x4[perm[1] - 1];
        x[ib + 2] = x4[perm[2] - 1];
        x[ib + 3] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  if (ib > 0) {
    for (b_m = 0; b_m < 4; b_m++) {
      perm[b_m] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (k = 1; k <= ib; k++) {
      idx[(k - nNaNs) - ib] = idx4[perm[k - 1] - 1];
      x[(k - nNaNs) - ib] = x4[perm[k - 1] - 1];
    }
  }

  b_m = (nNaNs + 7) >> 1;
  for (k = 1; k <= b_m; k++) {
    ib = idx[k - nNaNs];
    idx[k - nNaNs] = idx[8 - k];
    idx[8 - k] = ib;
    x[k - nNaNs] = xwork[8 - k];
    x[8 - k] = xwork[k - nNaNs];
  }

  if (((nNaNs + 7) & 1) != 0) {
    x[(b_m - nNaNs) + 1] = xwork[(b_m - nNaNs) + 1];
  }

  if (1 - nNaNs > 1) {
    for (b_m = 0; b_m < 8; b_m++) {
      iwork[b_m] = 0;
    }

    nPairs = (1 - nNaNs) >> 2;
    bLen = 4;
    while (nPairs > 1) {
      if ((nPairs & 1) != 0) {
        nPairs--;
        ib = bLen * nPairs;
        b_m = 1 - (nNaNs + ib);
        if (b_m > bLen) {
          merge(idx, x, ib, bLen, b_m - bLen, iwork, xwork);
        }
      }

      ib = bLen << 1;
      nPairs >>= 1;
      for (k = 1; k <= nPairs; k++) {
        merge(idx, x, (k - 1) * ib, bLen, bLen, iwork, xwork);
      }

      bLen = ib;
    }

    if (1 - nNaNs > bLen) {
      merge(idx, x, 0, bLen, 1 - (nNaNs + bLen), iwork, xwork);
    }
  }
}

//
// Arguments    : double A[30]
//                double tau[5]
//                int jpvt[5]
// Return Type  : void
//
static void b_xgeqp3(double A[30], double tau[5], int jpvt[5])
{
  double vn1[5];
  double vn2[5];
  int k;
  int iy;
  double work[5];
  double smax;
  double temp2;
  int itemp;
  double absxk;
  double t;
  int i;
  int i_i;
  int ix;
  int pvt;
  int i49;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  k = 1;
  for (iy = 0; iy < 5; iy++) {
    jpvt[iy] = 1 + iy;
    work[iy] = 0.0;
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
      smax = vn1[i];
      for (k = 1; k + 1 <= 5 - i; k++) {
        ix++;
        if (vn1[ix] > smax) {
          itemp = k;
          smax = vn1[ix];
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
    smax = c_xnrm2(5 - i, A, i_i + 2);
    if (smax != 0.0) {
      smax = hypot(A[i_i], smax);
      if (A[i_i] >= 0.0) {
        smax = -smax;
      }

      if (fabs(smax) < 1.0020841800044864E-292) {
        itemp = 0;
        do {
          itemp++;
          i49 = i_i - i;
          for (k = i_i + 1; k + 1 <= i49 + 6; k++) {
            A[k] *= 9.9792015476736E+291;
          }

          smax *= 9.9792015476736E+291;
          absxk *= 9.9792015476736E+291;
        } while (!(fabs(smax) >= 1.0020841800044864E-292));

        smax = hypot(absxk, c_xnrm2(5 - i, A, i_i + 2));
        if (absxk >= 0.0) {
          smax = -smax;
        }

        temp2 = (smax - absxk) / smax;
        absxk = 1.0 / (absxk - smax);
        i49 = i_i - i;
        for (k = i_i + 1; k + 1 <= i49 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 1.0020841800044864E-292;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0 / (A[i_i] - smax);
        i49 = i_i - i;
        for (k = i_i + 1; k + 1 <= i49 + 6; k++) {
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
          i49 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i49; itemp += 6) {
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
              i49 = lastv + itemp;
              for (k = itemp; k + 1 <= i49; k++) {
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
      itemp = (i + 6 * iy) + 1;
      if (vn1[iy] != 0.0) {
        smax = fabs(A[i + 6 * iy]) / vn1[iy];
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        temp2 = vn1[iy] / vn2[iy];
        temp2 = smax * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          smax = 0.0;
          if (5 - i == 1) {
            smax = fabs(A[itemp]);
          } else {
            temp2 = 2.2250738585072014E-308;
            pvt = (itemp - i) + 5;
            while (itemp + 1 <= pvt) {
              absxk = fabs(A[itemp]);
              if (absxk > temp2) {
                t = temp2 / absxk;
                smax = 1.0 + smax * t * t;
                temp2 = absxk;
              } else {
                t = absxk / temp2;
                smax += t * t;
              }

              itemp++;
            }

            smax = temp2 * sqrt(smax);
          }

          vn1[iy] = smax;
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
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double b_xnrm2(int n, const emxArray_real_T *x, int ix0)
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
// Arguments    : const double x[5]
// Return Type  : boolean_T
//
static boolean_T c_any(const double x[5])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  boolean_T b0;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 5)) {
    if ((x[k] == 0.0) || rtIsNaN(x[k])) {
      b0 = true;
    } else {
      b0 = false;
    }

    if (!b0) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : double I[8281]
// Return Type  : void
//
static void c_eye(double I[8281])
{
  int k;
  memset(&I[0], 0, 8281U * sizeof(double));
  for (k = 0; k < 91; k++) {
    I[k + 91 * k] = 1.0;
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double c_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[8] = { 'q', 'v', ':', ' ', '%', 'f', '\x0a', '\x00' };

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
// PREDICTMEASUREMENT Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel coordinates where a feature given in the left camera
//    frame would be visible in both cameras. Depending on the
//    camera model used to calibrate the camera, the appropriate undistortion
//    is applied
// Arguments    : const double fp_l[3]
//                const double c_stereoParams_CameraParameters[2]
//                const double d_stereoParams_CameraParameters[2]
//                const double e_stereoParams_CameraParameters[3]
//                const double f_stereoParams_CameraParameters[2]
//                const double g_stereoParams_CameraParameters[2]
//                const double h_stereoParams_CameraParameters[3]
//                const double stereoParams_r_lr[3]
//                const double stereoParams_R_rl[9]
//                double h_d_l[2]
//                double h_d_r[2]
// Return Type  : void
//
static void c_predictMeasurementStereoDisto(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[3], const double
  f_stereoParams_CameraParameters[2], const double
  g_stereoParams_CameraParameters[2], const double
  h_stereoParams_CameraParameters[3], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_d_l[2], double h_d_r[2])
{
  double b_stereoParams_R_rl[3];
  int i27;
  double d3;
  int i28;
  predictMeasurementDistortedPB(fp_l, c_stereoParams_CameraParameters,
    d_stereoParams_CameraParameters, e_stereoParams_CameraParameters, h_d_l);
  for (i27 = 0; i27 < 3; i27++) {
    d3 = 0.0;
    for (i28 = 0; i28 < 3; i28++) {
      d3 += stereoParams_R_rl[i27 + 3 * i28] * fp_l[i28];
    }

    b_stereoParams_R_rl[i27] = d3 - stereoParams_r_lr[i27];
  }

  predictMeasurementDistortedPB(b_stereoParams_R_rl,
    f_stereoParams_CameraParameters, g_stereoParams_CameraParameters,
    h_stereoParams_CameraParameters, h_d_r);
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void c_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv14[24];
  int i17;
  static const char cv15[24] = { 'R', 'e', 'j', 'e', 'c', 't', 'i', 'n', 'g',
    ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', '%', 'i', ')', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i17 = 0; i17 < 24; i17++) {
      cv14[i17] = cv15[i17];
    }

    ROS_INFO(cv14, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : emxArray_real_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void c_sort(emxArray_real_T *x, emxArray_int32_T *idx)
{
  int dim;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  d_sort(x, dim, idx);
}

//
// Arguments    : int n
//                const double x[30]
//                int ix0
// Return Type  : double
//
static double c_xnrm2(int n, const double x[30], int ix0)
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
// Arguments    : const struct_T x[5]
//                AnchorPose y[5]
// Return Type  : void
//
static void cast(const struct_T x[5], AnchorPose y[5])
{
  int j;
  int i;
  for (j = 0; j < 5; j++) {
    for (i = 0; i < 3; i++) {
      y[j].pos[i] = x[j].pos[i];
    }

    for (i = 0; i < 4; i++) {
      y[j].att[i] = x[j].att[i];
    }
  }
}

//
// Arguments    : const double a[3]
//                const double b[3]
//                double c[3]
// Return Type  : void
//
static void cross(const double a[3], const double b[3], double c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

//
// Arguments    : double I[8281]
// Return Type  : void
//
static void d_eye(double I[8281])
{
  int k;
  memset(&I[0], 0, 8281U * sizeof(double));
  for (k = 0; k < 91; k++) {
    I[k + 91 * k] = 1.0;
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double d_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[8] = { 'q', 'w', ':', ' ', '%', 'f', '\x0a', '\x00' };

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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
// Return Type  : void
//
static void d_ros_info(int varargin_1, int varargin_2)
{
  char cv16[44];
  int i20;
  static const char cv17[44] = { 's', 'u', 'c', 'c', 'e', 's', 's', 'f', 'u',
    'l', 'l', 'y', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a', 't', 'e',
    'd', ' ', '%', 'd', ' ', 'o', 'f', ' ', '%', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i20 = 0; i20 < 44; i20++) {
      cv16[i20] = cv17[i20];
    }

    ROS_INFO(cv16, varargin_1, varargin_2);
  }
}

//
// Arguments    : emxArray_real_T *x
//                int dim
//                emxArray_int32_T *idx
// Return Type  : void
//
static void d_sort(emxArray_real_T *x, int dim, emxArray_int32_T *idx)
{
  int i50;
  emxArray_real_T *vwork;
  int vstride;
  int x_idx_0;
  int j;
  emxArray_int32_T *iidx;
  if (dim <= 1) {
    i50 = x->size[0];
  } else {
    i50 = 1;
  }

  emxInit_real_T(&vwork, 1);
  vstride = vwork->size[0];
  vwork->size[0] = i50;
  emxEnsureCapacity((emxArray__common *)vwork, vstride, (int)sizeof(double));
  x_idx_0 = x->size[0];
  vstride = idx->size[0];
  idx->size[0] = x_idx_0;
  emxEnsureCapacity((emxArray__common *)idx, vstride, (int)sizeof(int));
  vstride = 1;
  x_idx_0 = 1;
  while (x_idx_0 <= dim - 1) {
    vstride *= x->size[0];
    x_idx_0 = 2;
  }

  j = 0;
  emxInit_int32_T1(&iidx, 1);
  while (j + 1 <= vstride) {
    for (x_idx_0 = 0; x_idx_0 + 1 <= i50; x_idx_0++) {
      vwork->data[x_idx_0] = x->data[j + x_idx_0 * vstride];
    }

    sortIdx(vwork, iidx);
    for (x_idx_0 = 0; x_idx_0 + 1 <= i50; x_idx_0++) {
      x->data[j + x_idx_0 * vstride] = vwork->data[x_idx_0];
      idx->data[j + x_idx_0 * vstride] = iidx->data[x_idx_0];
    }

    j++;
  }

  emxFree_int32_T(&iidx);
  emxFree_real_T(&vwork);
}

//
// Arguments    : const double x[36]
// Return Type  : double
//
static double det(const double x[36])
{
  double y;
  double A[36];
  signed char ipiv[6];
  int i25;
  int j;
  int c;
  int iy;
  int ix;
  double smax;
  int jy;
  double s;
  int b_j;
  int ijA;
  boolean_T isodd;
  memcpy(&A[0], &x[0], 36U * sizeof(double));
  for (i25 = 0; i25 < 6; i25++) {
    ipiv[i25] = (signed char)(1 + i25);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    smax = fabs(A[c]);
    for (jy = 1; jy + 1 <= 6 - j; jy++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        iy = jy;
        smax = s;
      }
    }

    if (A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (jy = 0; jy < 6; jy++) {
          smax = A[ix];
          A[ix] = A[iy];
          A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      i25 = (c - j) + 6;
      for (iy = c + 1; iy + 1 <= i25; iy++) {
        A[iy] /= A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (b_j = 1; b_j <= 5 - j; b_j++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i25 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i25; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  y = A[0];
  isodd = false;
  for (jy = 0; jy < 5; jy++) {
    y *= A[(jy + 6 * (jy + 1)) + 1];
    if (ipiv[jy] > 1 + jy) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    y = -y;
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double e_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[9] = { 'q', 'a', 'o', ':', ' ', '%', 'f', '\x0a',
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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void e_ros_info(int varargin_1)
{
  char cv18[52];
  int i21;
  static const char cv19[52] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z',
    'i', 'n', 'g', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ',', ' ',
    'w', 'h', 'i', 'c', 'h', ' ', 'w', 'a', 's', ' ', 't', 'h', 'e', ' ', 'o',
    'r', 'i', 'g', 'i', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i21 = 0; i21 < 52; i21++) {
      cv18[i21] = cv19[i21];
    }

    ROS_INFO(cv18, varargin_1);
  }
}

//
// Arguments    : int b
//                emxArray_int32_T *y
// Return Type  : void
//
static void eml_signed_integer_colon(int b, emxArray_int32_T *y)
{
  int n;
  int yk;
  int k;
  if (b < 1) {
    n = 0;
  } else {
    n = b;
  }

  yk = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = n;
  emxEnsureCapacity((emxArray__common *)y, yk, (int)sizeof(int));
  if (n > 0) {
    y->data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      y->data[k - 1] = yk;
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
// Arguments    : emxArray_int32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_int32_T1(emxArray_int32_T **pEmxArray, int b_numDimensions)
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
// Arguments    : emxArray_real_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_real_T1(emxArray_real_T **pEmxArray, int b_numDimensions)
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
// Arguments    : double I[9]
// Return Type  : void
//
static void eye(double I[9])
{
  int k;
  memset(&I[0], 0, 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0;
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double f_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[9] = { 'q', 'w', 'o', ':', ' ', '%', 'f', '\x0a',
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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void f_ros_info(int varargin_1)
{
  char cv20[45];
  int i22;
  static const char cv21[45] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'd', ' ', 'i', 's', ' ', 't', 'o', 'o', ' ', 'f', 'a', 'r', ' ', 'a', 'w',
    'a', 'y', ' ', 't', 'o', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a',
    't', 'e', '.', '\\', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i22 = 0; i22 < 45; i22++) {
      cv20[i22] = cv21[i22];
    }

    ROS_INFO(cv20, varargin_1);
  }
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double g_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[11] = { 'q', 'R', '_', 'c', 'i', ':', ' ', '%', 'f',
    '\x0a', '\x00' };

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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void g_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv22[48];
  int i23;
  static const char cv23[48] = { 'I', 'n', 's', 'e', 'r', 't', 'i', 'n', 'g',
    ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'd', ' ', 'a', 's', ' ',
    'f', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i23 = 0; i23 < 48; i23++) {
      cv22[i23] = cv23[i23];
    }

    ROS_INFO(cv22, varargin_1, varargin_2, varargin_3);
  }
}

//
// getAnchorPoses Get the anchor poses in the world frame
// Arguments    : const double xt_origin_pos[3]
//                const double xt_origin_att[4]
//                const f_struct_T xt_anchor_states[5]
//                struct_T anchor_poses[5]
// Return Type  : void
//
static void getAnchorPoses(const double xt_origin_pos[3], const double
  xt_origin_att[4], const f_struct_T xt_anchor_states[5], struct_T anchor_poses
  [5])
{
  double R_ow[9];
  int anchorIdx;
  double b_xt_anchor_states[9];
  double c_xt_anchor_states[9];
  int i31;
  double d5;
  int i32;
  int i33;

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[6] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[1] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[2] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[5] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    c_xt_anchor_states[0] = ((xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    c_xt_anchor_states[3] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[6] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[1] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    c_xt_anchor_states[7] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[2] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[5] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    for (i31 = 0; i31 < 3; i31++) {
      d5 = 0.0;
      for (i32 = 0; i32 < 3; i32++) {
        d5 += R_ow[i32 + 3 * i31] * xt_anchor_states[anchorIdx].pos[i32];
        b_xt_anchor_states[i31 + 3 * i32] = 0.0;
        for (i33 = 0; i33 < 3; i33++) {
          b_xt_anchor_states[i31 + 3 * i32] += c_xt_anchor_states[i31 + 3 * i33]
            * R_ow[i33 + 3 * i32];
        }
      }

      anchor_poses[anchorIdx].pos[i31] = xt_origin_pos[i31] + d5;
    }

    QuatFromRotJ(b_xt_anchor_states, anchor_poses[anchorIdx].att);
  }
}

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
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
// Arguments    : const double xt_robot_state_pos[3]
//                const double xt_robot_state_att[4]
//                const f_struct_T xt_anchor_states[5]
//                const double z_all_l[80]
//                const boolean_T b_status[40]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                double noiseParameters_image_noise
//                emxArray_real_T *r
//                emxArray_real_T *H
//                emxArray_real_T *R
// Return Type  : void
//
static void getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[5], const double
  z_all_l[80], const boolean_T b_status[40], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double noiseParameters_image_noise, emxArray_real_T *r, emxArray_real_T *H,
  emxArray_real_T *R)
{
  double fx;
  double fy;
  int n;
  int k;
  double R_cw[9];
  int ia;
  emxArray_real_T *b_h_u;
  double res_idx;
  int anchorIdx;
  double anchorRot[9];
  int featureIdx;
  long i15;
  double z_curr_l[2];
  double h_c_n_l[2];
  double varargin_2;
  double h_u_To_h_ci_l[6];
  double h_ci_l_To_R_cw[9];
  int br;
  double H_robot[63];
  double b_xt_anchor_states[3];
  double y[3];
  double h_ci_l_To_rho[3];
  int ar;
  int ib;
  double H_map_data[63];
  double b_R_cw[9];
  double dv0[9];
  double c_R_cw[9];
  int iv0[2];
  int outsize_idx_1;
  double b_data[462];
  double C_data[308];
  int ic;
  fx = cameraparams_FocalLength[0];
  fy = cameraparams_FocalLength[1];
  n = 0;
  for (k = 0; k < 40; k++) {
    if (b_status[k]) {
      n++;
    }
  }

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[3] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1] +
                   xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[6] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2] -
                   xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[1] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1] -
                   xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[7] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2] +
                   xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[2] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2] +
                   xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[5] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2] -
                   xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
              xt_robot_state_att[1] * xt_robot_state_att[1]) +
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  ia = H->size[0] * H->size[1];
  H->size[0] = (int)((double)n * 2.0);
  H->size[1] = 91;
  emxEnsureCapacity((emxArray__common *)H, ia, (int)sizeof(double));
  k = (int)((double)n * 2.0) * 91;
  for (ia = 0; ia < k; ia++) {
    H->data[ia] = 0.0;
  }

  ia = r->size[0];
  r->size[0] = (int)((double)n * 2.0);
  emxEnsureCapacity((emxArray__common *)r, ia, (int)sizeof(double));
  k = (int)((double)n * 2.0);
  for (ia = 0; ia < k; ia++) {
    r->data[ia] = 0.0;
  }

  emxInit_real_T(&b_h_u, 1);
  ia = b_h_u->size[0];
  b_h_u->size[0] = (int)((double)n * 2.0);
  emxEnsureCapacity((emxArray__common *)b_h_u, ia, (int)sizeof(double));
  k = (int)((double)n * 2.0);
  for (ia = 0; ia < k; ia++) {
    b_h_u->data[ia] = 0.0;
  }

  res_idx = 1.0;
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    anchorRot[0] = ((xt_anchor_states[anchorIdx].att[0] *
                     xt_anchor_states[anchorIdx].att[0] -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[1] +
                          xt_anchor_states[anchorIdx].att[2] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[2] -
                          xt_anchor_states[anchorIdx].att[1] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[1] -
                          xt_anchor_states[anchorIdx].att[2] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) +
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
                          xt_anchor_states[anchorIdx].att[2] +
                          xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[2] +
                          xt_anchor_states[anchorIdx].att[1] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
                          xt_anchor_states[anchorIdx].att[2] -
                          xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) +
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) &&
          b_status[xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1]) {
        i15 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx
          - 1L;
        if (i15 > 2147483647L) {
          i15 = 2147483647L;
        } else {
          if (i15 < -2147483648L) {
            i15 = -2147483648L;
          }
        }

        ia = (int)i15;
        if (ia > 1073741823) {
          k = MAX_int32_T;
        } else if (ia <= -1073741824) {
          k = MIN_int32_T;
        } else {
          k = ia << 1;
        }

        // predictMeasurementLeft Predict the measurement of a feature given in the left 
        // camera frame
        //    Get the normalized pixel coordinates where a feature given in the left camera 
        //    frame
        for (ia = 0; ia < 2; ia++) {
          i15 = (long)k + (1 + ia);
          if (i15 > 2147483647L) {
            i15 = 2147483647L;
          } else {
            if (i15 < -2147483648L) {
              i15 = -2147483648L;
            }
          }

          z_curr_l[ia] = z_all_l[(int)i15 - 1];
          h_c_n_l[ia] = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[ia] / xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
        }

        //  normalized feature in camera frame
        varargin_2 = (res_idx - 1.0) * 2.0;
        b_h_u->data[(int)(varargin_2 + 1.0) - 1] = h_c_n_l[0] *
          cameraparams_FocalLength[0] + cameraparams_PrincipalPoint[0];
        b_h_u->data[(int)(varargin_2 + 2.0) - 1] = h_c_n_l[1] *
          cameraparams_FocalLength[1] + cameraparams_PrincipalPoint[1];
        varargin_2 = (res_idx - 1.0) * 2.0;
        for (ia = 0; ia < 2; ia++) {
          r->data[(int)(varargin_2 + (1.0 + (double)ia)) - 1] = z_curr_l[ia];
        }

        h_u_To_h_ci_l[0] = fx / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_To_h_ci_l[2] = 0.0;
        h_u_To_h_ci_l[4] = -fx * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        h_u_To_h_ci_l[1] = 0.0;
        h_u_To_h_ci_l[3] = fy / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_To_h_ci_l[5] = -fy * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        h_ci_l_To_R_cw[0] = 0.0;
        h_ci_l_To_R_cw[3] = -xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_ci_l_To_R_cw[6] = xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1];
        h_ci_l_To_R_cw[1] = xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_ci_l_To_R_cw[4] = 0.0;
        h_ci_l_To_R_cw[7] = -xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0];
        h_ci_l_To_R_cw[2] = -xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1];
        h_ci_l_To_R_cw[5] = xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0];
        h_ci_l_To_R_cw[8] = 0.0;
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0)) {
          //  delayed initialization or feature behind anchor
          for (ia = 0; ia < 3; ia++) {
            for (br = 0; br < 3; br++) {
              H_robot[br + 3 * ia] = 0.0;
              H_robot[br + 3 * (ia + 3)] = 0.0 * h_ci_l_To_R_cw[br + 3 * ia];
            }
          }

          for (ia = 0; ia < 15; ia++) {
            for (br = 0; br < 3; br++) {
              H_robot[br + 3 * (ia + 6)] = 0.0;
            }
          }
        } else {
          for (ia = 0; ia < 3; ia++) {
            for (br = 0; br < 3; br++) {
              H_robot[br + 3 * ia] = -xt_anchor_states[anchorIdx]
                .feature_states[featureIdx].inverse_depth * R_cw[br + 3 * ia];
              H_robot[br + 3 * (ia + 3)] = h_ci_l_To_R_cw[br + 3 * ia];
            }
          }

          for (ia = 0; ia < 15; ia++) {
            for (br = 0; br < 3; br++) {
              H_robot[br + 3 * (ia + 6)] = 0.0;
            }
          }
        }

        //             %% anchor state derivatives
        for (ia = 0; ia < 3; ia++) {
          y[ia] = 0.0;
          for (br = 0; br < 3; br++) {
            y[ia] += anchorRot[br + 3 * ia] * xt_anchor_states[anchorIdx].
              feature_states[featureIdx].m[br];
          }

          b_xt_anchor_states[ia] = xt_anchor_states[anchorIdx].pos[ia] -
            xt_robot_state_pos[ia];
        }

        for (ia = 0; ia < 3; ia++) {
          h_ci_l_To_rho[ia] = 0.0;
          for (br = 0; br < 3; br++) {
            h_ci_l_To_rho[ia] += R_cw[ia + 3 * br] * b_xt_anchor_states[br];
          }
        }

        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0)) {
          //  delayed initialization or feature behind anchor
          ar = featureIdx + 6;
          k = 7 - featureIdx;
          ib = (ar + k) + 1;
          for (ia = 0; ia < ar; ia++) {
            for (br = 0; br < 3; br++) {
              H_map_data[br + 3 * ia] = 0.0;
            }
          }

          for (ia = 0; ia < 3; ia++) {
            H_map_data[ia + 3 * ar] = h_ci_l_To_rho[ia];
          }

          for (ia = 0; ia < k; ia++) {
            for (br = 0; br < 3; br++) {
              H_map_data[br + 3 * ((ia + ar) + 1)] = 0.0;
            }
          }
        } else {
          for (ia = 0; ia < 3; ia++) {
            for (br = 0; br < 3; br++) {
              b_R_cw[br + 3 * ia] = -R_cw[br + 3 * ia];
            }
          }

          dv0[0] = 0.0;
          dv0[3] = -y[2];
          dv0[6] = y[1];
          dv0[1] = y[2];
          dv0[4] = 0.0;
          dv0[7] = -y[0];
          dv0[2] = -y[1];
          dv0[5] = y[0];
          dv0[8] = 0.0;
          ar = 7 - featureIdx;
          ib = (featureIdx + ar) + 7;
          for (ia = 0; ia < 3; ia++) {
            for (br = 0; br < 3; br++) {
              c_R_cw[ia + 3 * br] = 0.0;
              for (k = 0; k < 3; k++) {
                c_R_cw[ia + 3 * br] += b_R_cw[ia + 3 * k] * dv0[k + 3 * br];
              }

              H_map_data[br + 3 * ia] = xt_anchor_states[anchorIdx].
                feature_states[featureIdx].inverse_depth * R_cw[br + 3 * ia];
            }
          }

          for (ia = 0; ia < 3; ia++) {
            for (br = 0; br < 3; br++) {
              H_map_data[br + 3 * (ia + 3)] = c_R_cw[br + 3 * ia];
            }
          }

          for (ia = 0; ia < featureIdx; ia++) {
            for (br = 0; br < 3; br++) {
              H_map_data[br + 3 * (ia + 6)] = 0.0;
            }
          }

          for (ia = 0; ia < 3; ia++) {
            H_map_data[ia + 3 * (6 + featureIdx)] = h_ci_l_To_rho[ia];
          }

          for (ia = 0; ia < ar; ia++) {
            for (br = 0; br < 3; br++) {
              H_map_data[br + 3 * ((ia + featureIdx) + 7)] = 0.0;
            }
          }
        }

        varargin_2 = (res_idx - 1.0) * 2.0;
        for (ia = 0; ia < 2; ia++) {
          iv0[ia] = (int)(varargin_2 + (1.0 + (double)ia)) - 1;
        }

        ar = anchorIdx * 14;
        k = (4 - anchorIdx) * 14;
        outsize_idx_1 = ((ar + ib) + k) + 21;
        for (ia = 0; ia < 21; ia++) {
          for (br = 0; br < 3; br++) {
            b_data[br + 3 * ia] = H_robot[br + 3 * ia];
          }
        }

        for (ia = 0; ia < ar; ia++) {
          for (br = 0; br < 3; br++) {
            b_data[br + 3 * (ia + 21)] = 0.0;
          }
        }

        for (ia = 0; ia < ib; ia++) {
          for (br = 0; br < 3; br++) {
            b_data[br + 3 * ((ia + ar) + 21)] = H_map_data[br + 3 * ia];
          }
        }

        for (ia = 0; ia < k; ia++) {
          for (br = 0; br < 3; br++) {
            b_data[br + 3 * (((ia + ar) + ib) + 21)] = 0.0;
          }
        }

        for (ia = 0; ia < outsize_idx_1; ia++) {
          for (br = 0; br < 2; br++) {
            C_data[br + (ia << 1)] = 0.0;
          }
        }

        k = (outsize_idx_1 - 1) << 1;
        for (outsize_idx_1 = 0; outsize_idx_1 <= k; outsize_idx_1 += 2) {
          for (ic = outsize_idx_1; ic + 1 <= outsize_idx_1 + 2; ic++) {
            C_data[ic] = 0.0;
          }
        }

        br = 0;
        for (outsize_idx_1 = 0; outsize_idx_1 <= k; outsize_idx_1 += 2) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 3; ib++) {
            if (b_data[ib] != 0.0) {
              ia = ar;
              for (ic = outsize_idx_1; ic + 1 <= outsize_idx_1 + 2; ic++) {
                ia++;
                C_data[ic] += b_data[ib] * h_u_To_h_ci_l[ia - 1];
              }
            }

            ar += 2;
          }

          br += 3;
        }

        for (ia = 0; ia < 91; ia++) {
          for (br = 0; br < 2; br++) {
            H->data[iv0[br] + H->size[0] * ia] = C_data[br + 2 * ia];
          }
        }

        res_idx++;
      }
    }
  }

  ia = r->size[0];
  emxEnsureCapacity((emxArray__common *)r, ia, (int)sizeof(double));
  k = r->size[0];
  for (ia = 0; ia < k; ia++) {
    r->data[ia] -= b_h_u->data[ia];
  }

  emxFree_real_T(&b_h_u);
  varargin_2 = (double)n * 2.0;
  outsize_idx_1 = (int)((double)n * 2.0);
  ia = R->size[0] * R->size[1];
  R->size[0] = (int)varargin_2;
  R->size[1] = (int)varargin_2;
  emxEnsureCapacity((emxArray__common *)R, ia, (int)sizeof(double));
  k = (int)varargin_2 * (int)varargin_2;
  for (ia = 0; ia < k; ia++) {
    R->data[ia] = 0.0;
  }

  for (k = 0; k + 1 <= outsize_idx_1; k++) {
    R->data[k + R->size[0] * k] = noiseParameters_image_noise;
  }
}

//
// GETMAP Get the feature points from the current state estimate in the world
// frame
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
// Arguments    : const double xt_origin_pos[3]
//                const double xt_origin_att[4]
//                const f_struct_T xt_anchor_states[5]
//                double b_map[120]
// Return Type  : void
//
static void getMap(const double xt_origin_pos[3], const double xt_origin_att[4],
                   const f_struct_T xt_anchor_states[5], double b_map[120])
{
  double R_ow[9];
  double anchorRot[9];
  int anchorIdx;
  double b_xt_anchor_states[9];
  double anchorPos[3];
  int i0;
  double d0;
  int i1;
  int y;
  int featureIdx;
  long i2;
  double b_anchorPos[3];

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[6] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[1] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[2] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[5] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  memset(&b_map[0], 0, 120U * sizeof(double));
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_xt_anchor_states[0] = ((xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[3] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[6] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[1] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[7] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[2] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[5] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    for (i0 = 0; i0 < 3; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        d0 += R_ow[i1 + 3 * i0] * xt_anchor_states[anchorIdx].pos[i1];
        anchorRot[i0 + 3 * i1] = 0.0;
        for (y = 0; y < 3; y++) {
          anchorRot[i0 + 3 * i1] += b_xt_anchor_states[i0 + 3 * y] * R_ow[y + 3 *
            i1];
        }
      }

      anchorPos[i0] = xt_origin_pos[i0] + d0;
    }

    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) {
        i2 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx -
          1L;
        if (i2 > 2147483647L) {
          i2 = 2147483647L;
        } else {
          if (i2 < -2147483648L) {
            i2 = -2147483648L;
          }
        }

        i2 = (int)i2 * 3L;
        if (i2 > 2147483647L) {
          i2 = 2147483647L;
        } else {
          if (i2 < -2147483648L) {
            i2 = -2147483648L;
          }
        }

        y = (int)i2;
        for (i0 = 0; i0 < 3; i0++) {
          d0 = 0.0;
          for (i1 = 0; i1 < 3; i1++) {
            d0 += anchorRot[i1 + 3 * i0] * xt_anchor_states[anchorIdx].
              feature_states[featureIdx].m[i1];
          }

          b_anchorPos[i0] = anchorPos[i0] + d0 / xt_anchor_states[anchorIdx].
            feature_states[featureIdx].inverse_depth;
          i2 = (long)y + (1 + i0);
          if (i2 > 2147483647L) {
            i2 = 2147483647L;
          } else {
            if (i2 < -2147483648L) {
              i2 = -2147483648L;
            }
          }

          b_map[(int)i2 - 1] = b_anchorPos[i0];
        }
      }
    }
  }
}

//
// getNumValidFeatures Get the number of valid features of an anchor
// Arguments    : const e_struct_T anchor_state_feature_states[8]
// Return Type  : double
//
static double getNumValidFeatures(const e_struct_T anchor_state_feature_states[8])
{
  double n;
  int featureIdx;
  boolean_T x[2];
  int k;
  boolean_T y;
  boolean_T exitg1;

  // getNumFeaturesOfType Get the number of features of type type of an anchor
  //  type can be a scalar or a row vector of types
  n = 0.0;
  for (featureIdx = 0; featureIdx < 8; featureIdx++) {
    for (k = 0; k < 2; k++) {
      x[k] = (anchor_state_feature_states[featureIdx].status == 1 + k);
    }

    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (!!x[k]) {
        y = true;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (y) {
      n++;
    }
  }

  return n;
}

//
// getScaledMap Get the sacled feature points from the current state estimate
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
//  - map:                  The scaled map of feature points in world coordinates (3 x
//                          numTrackFeatures)
//  - anchorInd:            A vector describing which anchor each feature belongs to
//  - featureAnchorInd:     A vector describing the index of each feature in its
//                          anchor
// Arguments    : g_struct_T *b_xt
// Return Type  : void
//
static void getScaledMap(g_struct_T *b_xt)
{
  double R_cw[9];
  int anchorIdx;
  double anchorRot[9];
  int featureIdx;
  double c_xt[3];
  int i43;
  double d10;
  int i44;

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  R_cw[3] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
  R_cw[6] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                   b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
  R_cw[1] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
  R_cw[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  R_cw[7] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                   b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
  R_cw[2] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                   b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
  R_cw[5] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                   b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
  R_cw[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    anchorRot[0] = ((b_xt->anchor_states[anchorIdx].att[0] * b_xt->
                     anchor_states[anchorIdx].att[0] - b_xt->
                     anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) - b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[1] +
                          b_xt->anchor_states[anchorIdx].att[2] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[2] -
                          b_xt->anchor_states[anchorIdx].att[1] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[1] -
                          b_xt->anchor_states[anchorIdx].att[2] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                       b_xt->anchor_states[anchorIdx].att[0]) +
                     b_xt->anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) - b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0 * (b_xt->anchor_states[anchorIdx].att[1] *
                          b_xt->anchor_states[anchorIdx].att[2] +
                          b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[2] +
                          b_xt->anchor_states[anchorIdx].att[1] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0 * (b_xt->anchor_states[anchorIdx].att[1] *
                          b_xt->anchor_states[anchorIdx].att[2] -
                          b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                       b_xt->anchor_states[anchorIdx].att[0]) -
                     b_xt->anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) + b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
      {
        for (i43 = 0; i43 < 3; i43++) {
          d10 = 0.0;
          for (i44 = 0; i44 < 3; i44++) {
            d10 += anchorRot[i44 + 3 * i43] * b_xt->anchor_states[anchorIdx].
              feature_states[featureIdx].m[i44];
          }

          c_xt[i43] = (b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
                       .inverse_depth * b_xt->anchor_states[anchorIdx].pos[i43]
                       + d10) - b_xt->robot_state.pos[i43] * b_xt->
            anchor_states[anchorIdx].feature_states[featureIdx].inverse_depth;
        }

        for (i43 = 0; i43 < 3; i43++) {
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i43] = 0.0;
          for (i44 = 0; i44 < 3; i44++) {
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              scaled_map_point[i43] += R_cw[i43 + 3 * i44] * c_xt[i44];
          }
        }
      }
    }
  }
}

//
// getTotalNumActiveFeatures Get the number of active features of all anchors
// Arguments    : const f_struct_T xt_anchor_states[5]
// Return Type  : double
//
static double getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[5])
{
  double n;
  int anchorIdx;
  double b_n;
  int featureIdx;
  n = 0.0;
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // getNumActiveFeatures Get the number of active features of an anchor
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    b_n = 0.0;
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (!!(xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 1))
      {
        b_n++;
      }
    }

    n += b_n;
  }

  return n;
}

//
// getTotalNumDelayedFeatures Get the number of delayed features of all anchors
// Arguments    : const f_struct_T xt_anchor_states[5]
// Return Type  : double
//
static double getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[5])
{
  double n;
  int anchorIdx;
  double b_n;
  int featureIdx;
  n = 0.0;
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    b_n = 0.0;
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (!!(xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2))
      {
        b_n++;
      }
    }

    n += b_n;
  }

  return n;
}

//
// getWorldState Get the state of the robot in the world frame
// Arguments    : const double xt_robot_state_IMU_pos[3]
//                const double xt_robot_state_IMU_att[4]
//                const double xt_robot_state_IMU_gyro_bias[3]
//                const double xt_robot_state_IMU_acc_bias[3]
//                const double xt_robot_state_pos[3]
//                const double xt_robot_state_att[4]
//                const double xt_origin_pos[3]
//                const double xt_origin_att[4]
//                double world_state_pos[3]
//                double world_state_att[4]
//                double world_state_vel[3]
//                double world_state_IMU_gyro_bias[3]
//                double world_state_IMU_acc_bias[3]
//                double world_state_IMU_pos[3]
//                double world_state_IMU_att[4]
// Return Type  : void
//
static void getWorldState(const double xt_robot_state_IMU_pos[3], const double
  xt_robot_state_IMU_att[4], const double xt_robot_state_IMU_gyro_bias[3], const
  double xt_robot_state_IMU_acc_bias[3], const double xt_robot_state_pos[3],
  const double xt_robot_state_att[4], const double xt_origin_pos[3], const
  double xt_origin_att[4], double world_state_pos[3], double world_state_att[4],
  double world_state_vel[3], double world_state_IMU_gyro_bias[3], double
  world_state_IMU_acc_bias[3], double world_state_IMU_pos[3], double
  world_state_IMU_att[4])
{
  double R_ow[9];
  double b_xt_robot_state_att[9];
  double c_xt_robot_state_att[9];
  int i;
  double d4;
  int i29;
  int i30;

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
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[6] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[1] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[2] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[5] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  c_xt_robot_state_att[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
    xt_robot_state_att[1] * xt_robot_state_att[1]) - xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  c_xt_robot_state_att[3] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1]
    + xt_robot_state_att[2] * xt_robot_state_att[3]);
  c_xt_robot_state_att[6] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2]
    - xt_robot_state_att[1] * xt_robot_state_att[3]);
  c_xt_robot_state_att[1] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1]
    - xt_robot_state_att[2] * xt_robot_state_att[3]);
  c_xt_robot_state_att[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
    xt_robot_state_att[1] * xt_robot_state_att[1]) - xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  c_xt_robot_state_att[7] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2]
    + xt_robot_state_att[0] * xt_robot_state_att[3]);
  c_xt_robot_state_att[2] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2]
    + xt_robot_state_att[1] * xt_robot_state_att[3]);
  c_xt_robot_state_att[5] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2]
    - xt_robot_state_att[0] * xt_robot_state_att[3]);
  c_xt_robot_state_att[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
    xt_robot_state_att[1] * xt_robot_state_att[1]) + xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  for (i = 0; i < 3; i++) {
    d4 = 0.0;
    for (i29 = 0; i29 < 3; i29++) {
      d4 += R_ow[i29 + 3 * i] * xt_robot_state_pos[i29];
      b_xt_robot_state_att[i + 3 * i29] = 0.0;
      for (i30 = 0; i30 < 3; i30++) {
        b_xt_robot_state_att[i + 3 * i29] += c_xt_robot_state_att[i + 3 * i30] *
          R_ow[i30 + 3 * i29];
      }
    }

    world_state_pos[i] = xt_origin_pos[i] + d4;
    world_state_vel[i] = 0.0;
    for (i29 = 0; i29 < 3; i29++) {
      world_state_vel[i] += R_ow[i29 + 3 * i] * xt_robot_state_pos[i29];
    }

    world_state_IMU_gyro_bias[i] = xt_robot_state_IMU_gyro_bias[i];
    world_state_IMU_acc_bias[i] = xt_robot_state_IMU_acc_bias[i];
    world_state_IMU_pos[i] = xt_robot_state_IMU_pos[i];
  }

  QuatFromRotJ(b_xt_robot_state_att, world_state_att);
  for (i = 0; i < 4; i++) {
    world_state_IMU_att[i] = xt_robot_state_IMU_att[i];
  }
}

//
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : double
//
static double h_fprintf(double varargin_1, double varargin_2, double varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[37] = { 'g', 'y', 'r', 'o', ' ', 'b', 'i', 'a', 's',
    ' ', 'i', 'n', 'i', 't', 'i', 'a', 'l', ' ', 'u', 'n', 'c', ':', ' ', '[',
    '%', 'f', ',', ' ', '%', 'f', ',', ' ', '%', 'f', ']', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1, varargin_2, varargin_3);
    fflush(filestar);
  }

  return nbytesint;
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : void
//
static void h_ros_info(int varargin_1, double varargin_2, double varargin_3)
{
  char cv24[51];
  int i24;
  static const char cv25[51] = { 'F', 'o', 'r', 'c', 'i', 'n', 'g', ' ', 'a',
    'c', 't', 'i', 'v', 'a', 't', 'i', 'o', 'n', ' ', 'o', 'f', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i24 = 0; i24 < 51; i24++) {
      cv24[i24] = cv25[i24];
    }

    ROS_INFO(cv24, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : double
//
static double i_fprintf(double varargin_1, double varargin_2, double varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[36] = { 'a', 'c', 'c', ' ', 'b', 'i', 'a', 's', ' ',
    'i', 'n', 'i', 't', 'i', 'a', 'l', ' ', 'u', 'n', 'c', ':', ' ', '[', '%',
    'f', ',', ' ', '%', 'f', ',', ' ', '%', 'f', ']', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1, varargin_2, varargin_3);
    fflush(filestar);
  }

  return nbytesint;
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void i_ros_info(int varargin_1)
{
  char cv26[28];
  int i26;
  static const char cv27[28] = { 'S', 'e', 't', 't', 'i', 'n', 'g', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ' ', 'a', 's', ' ', 'o', 'r', 'i',
    'g', 'i', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i26 = 0; i26 < 28; i26++) {
      cv26[i26] = cv27[i26];
    }

    ROS_INFO(cv26, varargin_1);
  }
}

//
// INITIALIZEPOINT Initialize a feature point from undistorted measurements
//
//  INPUT ARGUMENTS:
//  - cameraparams: The camera parameter struct
//  - z_l: The undistorted measurement of the feature in the left camera
//  - z_r: The undistorted measurement of the feature in the right camera
//
//  OUTPUT ARGUMENTS:
//  - fp: The feature point in the camera frame
//  - m:  The ray in the left and right camera frame to the feature
// Arguments    : const double z_u_l[2]
//                const double z_u_r[2]
//                const double c_cameraparams_CameraParameters[2]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const double f_cameraparams_CameraParameters[2]
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                double fp[3]
//                double b_m[6]
//                boolean_T *success
// Return Type  : void
//
static void initializePoint(const double z_u_l[2], const double z_u_r[2], const
  double c_cameraparams_CameraParameters[2], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const double
  f_cameraparams_CameraParameters[2], const double cameraparams_r_lr[3], const
  double cameraparams_R_lr[9], double fp[3], double b_m[6], boolean_T *success)
{
  double ml[3];
  double mr[3];
  double tol;
  double B;
  double b_pos[6];
  int k;
  int i18;
  static const signed char iv1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double rot[18];
  double A[30];
  double b[6];
  int anchorIdx;
  int rankR;
  double b_rot[9];
  signed char I[9];
  int j;
  int jpvt[5];
  double tau[5];
  double x[5];
  ml[0] = (z_u_l[0] - d_cameraparams_CameraParameters[0]) /
    c_cameraparams_CameraParameters[0];
  ml[1] = (z_u_l[1] - d_cameraparams_CameraParameters[1]) /
    c_cameraparams_CameraParameters[1];
  ml[2] = 1.0;
  mr[0] = (z_u_r[0] - f_cameraparams_CameraParameters[0]) /
    e_cameraparams_CameraParameters[0];
  mr[1] = (z_u_r[1] - f_cameraparams_CameraParameters[1]) /
    e_cameraparams_CameraParameters[1];
  mr[2] = 1.0;
  tol = norm(ml);
  B = norm(mr);
  for (k = 0; k < 3; k++) {
    b_pos[k] = 0.0;
    b_pos[3 + k] = cameraparams_r_lr[k];
    for (i18 = 0; i18 < 3; i18++) {
      rot[i18 + 3 * k] = iv1[i18 + 3 * k];
      rot[i18 + 3 * (k + 3)] = cameraparams_R_lr[k + 3 * i18];
    }

    b_m[k] = ml[k] / tol;
    b_m[3 + k] = mr[k] / B;
  }

  //  normalized rays in left frame
  for (k = 0; k < 3; k++) {
    ml[k] = 0.0;
    for (i18 = 0; i18 < 3; i18++) {
      ml[k] += cameraparams_R_lr[k + 3 * i18] * b_m[3 + i18];
    }
  }

  if (b_m[2] * ml[0] - b_m[0] * ml[2] > 0.0) {
    *success = false;
    for (k = 0; k < 3; k++) {
      fp[k] = b_m[k];
    }
  } else {
    *success = true;

    // triangulatePoint Triangulate a point from several measurements
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
    for (k = 0; k < 6; k++) {
      b[k] = rtNaN;
    }

    for (anchorIdx = 0; anchorIdx < 2; anchorIdx++) {
      rankR = anchorIdx * 3;
      for (k = 0; k < 3; k++) {
        for (i18 = 0; i18 < 3; i18++) {
          b_rot[i18 + 3 * k] = rot[k + 3 * (i18 + rankR)];
        }
      }

      rankR = anchorIdx * 3;
      for (k = 0; k < 3; k++) {
        A[(k + rankR) + 6 * anchorIdx] = 0.0;
        for (i18 = 0; i18 < 3; i18++) {
          A[(k + rankR) + 6 * anchorIdx] += b_rot[k + 3 * i18] * b_m[i18 + 3 *
            anchorIdx];
        }
      }

      for (k = 0; k < 9; k++) {
        I[k] = 0;
      }

      for (k = 0; k < 3; k++) {
        I[k + 3 * k] = 1;
      }

      rankR = anchorIdx * 3;
      j = anchorIdx * 3;
      for (k = 0; k < 3; k++) {
        for (i18 = 0; i18 < 3; i18++) {
          A[(i18 + rankR) + 6 * (2 + k)] = -(double)I[i18 + 3 * k];
        }

        b[k + j] = -b_pos[k + 3 * anchorIdx];
      }
    }

    //  condition = cond(A);
    b_xgeqp3(A, tau, jpvt);
    rankR = 0;
    tol = 6.0 * fabs(A[0]) * 2.2204460492503131E-16;
    while ((rankR < 5) && (fabs(A[rankR + 6 * rankR]) >= tol)) {
      rankR++;
    }

    for (j = 0; j < 5; j++) {
      x[j] = 0.0;
      if (tau[j] != 0.0) {
        tol = b[j];
        for (k = j + 1; k + 1 < 7; k++) {
          tol += A[k + 6 * j] * b[k];
        }

        tol *= tau[j];
        if (tol != 0.0) {
          b[j] -= tol;
          for (k = j + 1; k + 1 < 7; k++) {
            b[k] -= A[k + 6 * j] * tol;
          }
        }
      }
    }

    for (k = 0; k + 1 <= rankR; k++) {
      x[jpvt[k] - 1] = b[k];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
      x[jpvt[j] - 1] /= A[j + 6 * j];
      for (k = 0; k + 1 <= j; k++) {
        x[jpvt[k] - 1] -= x[jpvt[j] - 1] * A[k + 6 * j];
      }
    }

    for (k = 0; k < 3; k++) {
      fp[k] = x[k + 2];
    }

    //  is this better?
    //  A = [cameraparams.R_rl * ml, -mr];
    //  b = cameraparams.r_lr;
    //  x = A\b;
    //  fp = ml*x(1);
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double j_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[17] = { 'i', 'm', 'a', 'g', 'e', '_', 'n', 'o', 'i',
    's', 'e', ':', ' ', '%', 'f', '\x0a', '\x00' };

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
// Arguments    : double varargin_1
// Return Type  : double
//
static double k_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[27] = { 'i', 'n', 'v', '_', 'd', 'e', 'p', 't', 'h',
    '_', 'i', 'n', 'i', 't', 'i', 'a', 'l', '_', 'u', 'n', 'c', ':', ' ', '%',
    'f', '\x0a', '\x00' };

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
// Arguments    : int varargin_1
// Return Type  : double
//
static double l_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[17] = { 'n', 'u', 'm', '_', 'a', 'n', 'c', 'h', 'o',
    'r', 's', ':', ' ', '%', 'd', '\x0a', '\x00' };

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
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *B
// Return Type  : void
//
static void lusolve(const emxArray_real_T *A, emxArray_real_T *B)
{
  emxArray_real_T *b_A;
  int n;
  int i45;
  int iy;
  emxArray_int32_T *ipiv;
  int j;
  int mmj;
  int c;
  int ix;
  double smax;
  int k;
  double s;
  int i46;
  int i;
  int jAcol;
  int jA;
  emxInit_real_T1(&b_A, 2);
  n = A->size[1];
  i45 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, i45, (int)sizeof(double));
  iy = A->size[0] * A->size[1];
  for (i45 = 0; i45 < iy; i45++) {
    b_A->data[i45] = A->data[i45];
  }

  emxInit_int32_T(&ipiv, 2);
  iy = A->size[1];
  eml_signed_integer_colon(iy, ipiv);
  if (A->size[1] < 1) {
  } else {
    if (A->size[1] - 1 <= A->size[1]) {
      i45 = A->size[1] - 1;
    } else {
      i45 = A->size[1];
    }

    for (j = 0; j + 1 <= i45; j++) {
      mmj = n - j;
      c = j * (n + 1);
      if (mmj < 1) {
        iy = -1;
      } else {
        iy = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(b_A->data[c]);
          for (k = 1; k + 1 <= mmj; k++) {
            ix++;
            s = fabs(b_A->data[ix]);
            if (s > smax) {
              iy = k;
              smax = s;
            }
          }
        }
      }

      if (b_A->data[c + iy] != 0.0) {
        if (iy != 0) {
          ipiv->data[j] = (j + iy) + 1;
          ix = j;
          iy += j;
          for (k = 1; k <= n; k++) {
            smax = b_A->data[ix];
            b_A->data[ix] = b_A->data[iy];
            b_A->data[iy] = smax;
            ix += n;
            iy += n;
          }
        }

        i46 = c + mmj;
        for (i = c + 1; i + 1 <= i46; i++) {
          b_A->data[i] /= b_A->data[c];
        }
      }

      jAcol = (n - j) - 1;
      jA = c + n;
      iy = c + n;
      for (i = 1; i <= jAcol; i++) {
        smax = b_A->data[iy];
        if (b_A->data[iy] != 0.0) {
          ix = c + 1;
          i46 = mmj + jA;
          for (k = 1 + jA; k + 1 <= i46; k++) {
            b_A->data[k] += b_A->data[ix] * -smax;
            ix++;
          }
        }

        iy += n;
        jA += n;
      }
    }
  }

  if ((A->size[1] == 0) || (B->size[1] == 0)) {
  } else {
    for (j = 0; j + 1 <= n; j++) {
      iy = 91 * j;
      jAcol = n * j;
      for (k = 0; k + 1 <= j; k++) {
        jA = 91 * k;
        if (b_A->data[k + jAcol] != 0.0) {
          for (i = 0; i < 91; i++) {
            B->data[i + iy] -= b_A->data[k + jAcol] * B->data[i + jA];
          }
        }
      }

      smax = 1.0 / b_A->data[j + jAcol];
      for (i = 0; i < 91; i++) {
        B->data[i + iy] *= smax;
      }
    }
  }

  if ((A->size[1] == 0) || (B->size[1] == 0)) {
  } else {
    for (j = A->size[1]; j > 0; j--) {
      iy = 91 * (j - 1);
      jAcol = n * (j - 1);
      for (k = j; k + 1 <= n; k++) {
        jA = 91 * k;
        if (b_A->data[k + jAcol] != 0.0) {
          for (i = 0; i < 91; i++) {
            B->data[i + iy] -= b_A->data[k + jAcol] * B->data[i + jA];
          }
        }
      }
    }
  }

  emxFree_real_T(&b_A);
  for (iy = A->size[1] - 2; iy + 1 > 0; iy--) {
    if (ipiv->data[iy] != iy + 1) {
      jAcol = ipiv->data[iy] - 1;
      for (jA = 0; jA < 91; jA++) {
        smax = B->data[jA + B->size[0] * iy];
        B->data[jA + B->size[0] * iy] = B->data[jA + B->size[0] * jAcol];
        B->data[jA + B->size[0] * jAcol] = smax;
      }
    }
  }

  emxFree_int32_T(&ipiv);
}

//
// Arguments    : int varargin_1
// Return Type  : double
//
static double m_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[27] = { 'n', 'u', 'm', '_', 'p', 'o', 'i', 'n', 't',
    's', '_', 'p', 'e', 'r', '_', 'a', 'n', 'c', 'h', 'o', 'r', ':', ' ', '%',
    'd', '\x0a', '\x00' };

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
// Arguments    : const double x_data[]
//                const int x_size[1]
// Return Type  : double
//
static double median(const double x_data[], const int x_size[1])
{
  double y;
  int k;
  int midm1;
  int i;
  int idx_data[40];
  emxArray_int32_T *iwork;
  int n;
  int iwork_data[40];
  boolean_T p;
  int i2;
  int j;
  int pEnd;
  int b_p;
  int q;
  int qEnd;
  int kEnd;
  if (x_size[0] == 0) {
    y = rtNaN;
  } else {
    k = x_size[0];
    midm1 = k / 2;
    i = (signed char)x_size[0];
    for (k = 0; k < i; k++) {
      idx_data[k] = 0;
    }

    emxInit_int32_T1(&iwork, 1);
    n = x_size[0] + 1;
    k = iwork->size[0];
    iwork->size[0] = (signed char)x_size[0];
    emxEnsureCapacity((emxArray__common *)iwork, k, (int)sizeof(int));
    i = iwork->size[0];
    for (k = 0; k < i; k++) {
      iwork_data[k] = iwork->data[k];
    }

    emxFree_int32_T(&iwork);
    for (k = 1; k <= n - 2; k += 2) {
      if ((x_data[k - 1] <= x_data[k]) || rtIsNaN(x_data[k])) {
        p = true;
      } else {
        p = false;
      }

      if (p) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }

    if ((x_size[0] & 1) != 0) {
      idx_data[x_size[0] - 1] = x_size[0];
    }

    i = 2;
    while (i < n - 1) {
      i2 = i << 1;
      j = 1;
      for (pEnd = 1 + i; pEnd < n; pEnd = qEnd + i) {
        b_p = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          if ((x_data[idx_data[b_p - 1] - 1] <= x_data[idx_data[q] - 1]) ||
              rtIsNaN(x_data[idx_data[q] - 1])) {
            p = true;
          } else {
            p = false;
          }

          if (p) {
            iwork_data[k] = idx_data[b_p - 1];
            b_p++;
            if (b_p == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork_data[k] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[k] = idx_data[q];
            q++;
            if (q + 1 == qEnd) {
              while (b_p < pEnd) {
                k++;
                iwork_data[k] = idx_data[b_p - 1];
                b_p++;
              }
            }
          }

          k++;
        }

        for (k = 0; k + 1 <= kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }

        j = qEnd;
      }

      i = i2;
    }

    if (rtIsNaN(x_data[idx_data[(signed char)x_size[0] - 1] - 1])) {
      y = x_data[idx_data[(signed char)x_size[0] - 1] - 1];
    } else if (midm1 << 1 == x_size[0]) {
      if (((x_data[idx_data[midm1 - 1] - 1] < 0.0) && (x_data[idx_data[midm1] -
            1] >= 0.0)) || rtIsInf(x_data[idx_data[midm1 - 1] - 1]) || rtIsInf
          (x_data[idx_data[midm1] - 1])) {
        y = (x_data[idx_data[midm1 - 1] - 1] + x_data[idx_data[midm1] - 1]) /
          2.0;
      } else {
        y = x_data[idx_data[midm1 - 1] - 1] + (x_data[idx_data[midm1] - 1] -
          x_data[idx_data[midm1 - 1] - 1]) / 2.0;
      }
    } else {
      y = x_data[idx_data[midm1] - 1];
    }
  }

  return y;
}

//
// Arguments    : int idx[8]
//                double x[8]
//                int offset
//                int np
//                int nq
//                int iwork[8]
//                double xwork[8]
// Return Type  : void
//
static void merge(int idx[8], double x[8], int offset, int np, int nq, int
                  iwork[8], double xwork[8])
{
  int n;
  int qend;
  int p;
  int iout;
  int exitg1;
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
      if (xwork[p] <= xwork[n]) {
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
// Arguments    : emxArray_int32_T *idx
//                emxArray_real_T *x
//                int offset
//                int n
//                int preSortLevel
//                emxArray_int32_T *iwork
//                emxArray_real_T *xwork
// Return Type  : void
//
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  int nPairs;
  int bLen;
  int tailOffset;
  int nTail;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        b_merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 1; nTail <= nPairs; nTail++) {
      b_merge(idx, x, offset + (nTail - 1) * tailOffset, bLen, bLen, iwork,
              xwork);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    b_merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
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
  emxArray_real_T *Y;
  emxArray_real_T *b_A;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  emxArray_real_T *b_B;
  unsigned int unnamed_idx_1;
  int b_m;
  int maxmn;
  int minmn;
  int i;
  int rankR;
  double tol;
  int mn;
  emxInit_real_T1(&Y, 2);
  emxInit_real_T1(&b_A, 2);
  emxInit_real_T(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  emxInit_real_T1(&b_B, 2);
  if ((A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
    unnamed_idx_1 = (unsigned int)B->size[0];
    b_m = y->size[0] * y->size[1];
    y->size[0] = 91;
    y->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, b_m, (int)sizeof(double));
    maxmn = 91 * (int)unnamed_idx_1;
    for (b_m = 0; b_m < maxmn; b_m++) {
      y->data[b_m] = 0.0;
    }
  } else if (B->size[0] == B->size[1]) {
    b_m = y->size[0] * y->size[1];
    y->size[0] = 91;
    y->size[1] = A->size[1];
    emxEnsureCapacity((emxArray__common *)y, b_m, (int)sizeof(double));
    maxmn = A->size[0] * A->size[1];
    for (b_m = 0; b_m < maxmn; b_m++) {
      y->data[b_m] = A->data[b_m];
    }

    lusolve(B, y);
  } else {
    b_m = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[1];
    b_A->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_A, b_m, (int)sizeof(double));
    maxmn = B->size[0];
    for (b_m = 0; b_m < maxmn; b_m++) {
      minmn = B->size[1];
      for (i = 0; i < minmn; i++) {
        b_A->data[i + b_A->size[0] * b_m] = B->data[b_m + B->size[0] * i];
      }
    }

    xgeqp3(b_A, tau, jpvt);
    rankR = 0;
    if (b_A->size[0] < b_A->size[1]) {
      minmn = b_A->size[0];
      maxmn = b_A->size[1];
    } else {
      minmn = b_A->size[1];
      maxmn = b_A->size[0];
    }

    if (minmn > 0) {
      tol = (double)maxmn * fabs(b_A->data[0]) * 2.2204460492503131E-16;
      while ((rankR < minmn) && (fabs(b_A->data[rankR + b_A->size[0] * rankR]) >=
              tol)) {
        rankR++;
      }
    }

    minmn = b_A->size[1];
    b_m = Y->size[0] * Y->size[1];
    Y->size[0] = minmn;
    Y->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)Y, b_m, (int)sizeof(double));
    maxmn = minmn * 91;
    for (b_m = 0; b_m < maxmn; b_m++) {
      Y->data[b_m] = 0.0;
    }

    b_m = b_B->size[0] * b_B->size[1];
    b_B->size[0] = A->size[1];
    b_B->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)b_B, b_m, (int)sizeof(double));
    for (b_m = 0; b_m < 91; b_m++) {
      maxmn = A->size[1];
      for (i = 0; i < maxmn; i++) {
        b_B->data[i + b_B->size[0] * b_m] = A->data[b_m + A->size[0] * i];
      }
    }

    b_m = b_A->size[0];
    minmn = b_A->size[0];
    mn = b_A->size[1];
    if (minmn <= mn) {
      mn = minmn;
    }

    for (minmn = 0; minmn + 1 <= mn; minmn++) {
      if (tau->data[minmn] != 0.0) {
        for (maxmn = 0; maxmn < 91; maxmn++) {
          tol = b_B->data[minmn + b_B->size[0] * maxmn];
          for (i = minmn + 1; i + 1 <= b_m; i++) {
            tol += b_A->data[i + b_A->size[0] * minmn] * b_B->data[i + b_B->
              size[0] * maxmn];
          }

          tol *= tau->data[minmn];
          if (tol != 0.0) {
            b_B->data[minmn + b_B->size[0] * maxmn] -= tol;
            for (i = minmn + 1; i + 1 <= b_m; i++) {
              b_B->data[i + b_B->size[0] * maxmn] -= b_A->data[i + b_A->size[0] *
                minmn] * tol;
            }
          }
        }
      }
    }

    for (maxmn = 0; maxmn < 91; maxmn++) {
      for (i = 0; i + 1 <= rankR; i++) {
        Y->data[(jpvt->data[i] + Y->size[0] * maxmn) - 1] = b_B->data[i +
          b_B->size[0] * maxmn];
      }

      for (minmn = rankR - 1; minmn + 1 > 0; minmn--) {
        Y->data[(jpvt->data[minmn] + Y->size[0] * maxmn) - 1] /= b_A->data[minmn
          + b_A->size[0] * minmn];
        for (i = 0; i + 1 <= minmn; i++) {
          Y->data[(jpvt->data[i] + Y->size[0] * maxmn) - 1] -= Y->data
            [(jpvt->data[minmn] + Y->size[0] * maxmn) - 1] * b_A->data[i +
            b_A->size[0] * minmn];
        }
      }
    }

    b_m = y->size[0] * y->size[1];
    y->size[0] = 91;
    y->size[1] = Y->size[0];
    emxEnsureCapacity((emxArray__common *)y, b_m, (int)sizeof(double));
    maxmn = Y->size[0];
    for (b_m = 0; b_m < maxmn; b_m++) {
      for (i = 0; i < 91; i++) {
        y->data[i + y->size[0] * b_m] = Y->data[b_m + Y->size[0] * i];
      }
    }
  }

  emxFree_real_T(&b_B);
  emxFree_int32_T(&jpvt);
  emxFree_real_T(&tau);
  emxFree_real_T(&b_A);
  emxFree_real_T(&Y);
}

//
// # coder
// multiplyIdx Create a longer index array from idx
//    for example:
//    idx_mult = multiplyIdx([1 4 5], 2)
//    idx_mult = [1 2 7 8 9 10]
// Arguments    : const double idx_data[]
//                const int idx_size[1]
//                double idx_mult_data[]
//                int idx_mult_size[1]
// Return Type  : void
//
static void multiplyIdx(const double idx_data[], const int idx_size[1], double
  idx_mult_data[], int idx_mult_size[1])
{
  int loop_ub;
  int i10;
  int i;
  int j;
  idx_mult_size[0] = idx_size[0] << 1;
  loop_ub = idx_size[0] << 1;
  for (i10 = 0; i10 < loop_ub; i10++) {
    idx_mult_data[i10] = 0.0;
  }

  for (i = 0; i < idx_size[0]; i++) {
    for (j = 0; j < 2; j++) {
      idx_mult_data[(i << 1) + j] = (idx_data[i] - 1.0) * 2.0 + (1.0 + (double)j);
    }
  }
}

//
// Arguments    : int varargin_1
// Return Type  : double
//
static double n_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[24] = { 'm', 'a', 'x', '_', 'e', 'k', 'f', '_', 'i',
    't', 'e', 'r', 'a', 't', 'i', 'o', 'n', 's', ':', ' ', '%', 'd', '\x0a',
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
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double o_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i4;
  char varargout_1_data[6];
  static const char cfmt[28] = { 'd', 'e', 'l', 'a', 'y', 'e', 'd', '_', 'i',
    'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'a', 't', 'i', 'o', 'n', ':', ' ',
    '%', 's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i4 = 0; i4 < nbytesint; i4++) {
      varargout_1_data[i4] = varargin_1_data[i4];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double p_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i5;
  char varargout_1_data[6];
  static const char cfmt[19] = { 'f', 'i', 'x', 'e', 'd', '_', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ':', ' ', '%', 's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i5 = 0; i5 < nbytesint; i5++) {
      varargout_1_data[i5] = varargin_1_data[i5];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double fp[3]
//                const double cameraParams_FocalLength[2]
//                const double cameraParams_PrincipalPoint[2]
//                const double cameraParams_RadialDistortion[3]
//                double h_d[2]
// Return Type  : void
//
static void predictMeasurementDistortedPB(const double fp[3], const double
  cameraParams_FocalLength[2], const double cameraParams_PrincipalPoint[2],
  const double cameraParams_RadialDistortion[3], double h_d[2])
{
  double h_cin[2];
  int i;
  double radsq;
  double b;
  for (i = 0; i < 2; i++) {
    h_cin[i] = fp[i] / fp[2];
  }

  radsq = h_cin[0] * h_cin[0] + h_cin[1] * h_cin[1];
  b = ((1.0 + cameraParams_RadialDistortion[0] * radsq) +
       cameraParams_RadialDistortion[1] * (radsq * radsq)) +
    cameraParams_RadialDistortion[2] * rt_powd_snf(radsq, 4.0);
  for (i = 0; i < 2; i++) {
    h_cin[i] *= b;
  }

  h_d[0] = cameraParams_PrincipalPoint[0] + cameraParams_FocalLength[0] * h_cin
    [0];
  h_d[1] = cameraParams_PrincipalPoint[1] + cameraParams_FocalLength[1] * h_cin
    [1];
}

//
// predictMeasurementLeft Predict the measurement of a feature given in the left
// camera frame
//    Get the normalized pixel coordinates where a feature given in the left camera
//    frame
// Arguments    : const double fp[3]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                double px[2]
// Return Type  : void
//
static void predictMeasurementMono(const double fp[3], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double px[2])
{
  double h_c_n_l[2];
  int i;
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp[i] / fp[2];
  }

  //  normalized feature in camera frame
  px[0] = h_c_n_l[0] * cameraparams_FocalLength[0] +
    cameraparams_PrincipalPoint[0];
  px[1] = h_c_n_l[1] * cameraparams_FocalLength[1] +
    cameraparams_PrincipalPoint[1];
}

//
// predictMeasurementStereo Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel undistorted coordinates where a feature given in the left camera
//    frame would be visible in both cameras
// Arguments    : const double fp_l[3]
//                const double c_stereoParams_CameraParameters[2]
//                const double d_stereoParams_CameraParameters[2]
//                const double e_stereoParams_CameraParameters[2]
//                const double f_stereoParams_CameraParameters[2]
//                const double stereoParams_r_lr[3]
//                const double stereoParams_R_rl[9]
//                double h_cin_l[2]
//                double h_cin_r[2]
// Return Type  : void
//
static void predictMeasurementStereo(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_cin_l[2], double h_cin_r[2])
{
  double h_c_n_l[2];
  int i;
  double fp_r[3];
  double d2;
  int i19;

  // predictMeasurementLeft Predict the measurement of a feature given in the left 
  // camera frame
  //    Get the normalized pixel coordinates where a feature given in the left camera 
  //    frame
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp_l[i] / fp_l[2];
  }

  //  normalized feature in camera frame
  h_cin_l[0] = h_c_n_l[0] * c_stereoParams_CameraParameters[0] +
    d_stereoParams_CameraParameters[0];
  h_cin_l[1] = h_c_n_l[1] * c_stereoParams_CameraParameters[1] +
    d_stereoParams_CameraParameters[1];
  for (i = 0; i < 3; i++) {
    d2 = 0.0;
    for (i19 = 0; i19 < 3; i19++) {
      d2 += stereoParams_R_rl[i + 3 * i19] * fp_l[i19];
    }

    fp_r[i] = d2 - stereoParams_r_lr[i];
  }

  // predictMeasurementLeft Predict the measurement of a feature given in the left 
  // camera frame
  //    Get the normalized pixel coordinates where a feature given in the left camera 
  //    frame
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp_r[i] / fp_r[2];
  }

  //  normalized feature in camera frame
  h_cin_r[0] = h_c_n_l[0] * e_stereoParams_CameraParameters[0] +
    f_stereoParams_CameraParameters[0];
  h_cin_r[1] = h_c_n_l[1] * e_stereoParams_CameraParameters[1] +
    f_stereoParams_CameraParameters[1];
}

//
// print all parameters for debug check
// Arguments    : double c_noiseParameters_process_noise
//                double d_noiseParameters_process_noise
//                double e_noiseParameters_process_noise
//                double f_noiseParameters_process_noise
//                double g_noiseParameters_process_noise
//                const double c_noiseParameters_gyro_bias_ini[3]
//                const double c_noiseParameters_acc_bias_init[3]
//                double noiseParameters_image_noise
//                double c_noiseParameters_inv_depth_ini
//                int c_VIOParameters_num_points_per_
//                int VIOParameters_num_anchors
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T VIOParameters_fixed_feature
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_mono
//                boolean_T VIOParameters_RANSAC
// Return Type  : void
//
static void printParams(double c_noiseParameters_process_noise, double
  d_noiseParameters_process_noise, double e_noiseParameters_process_noise,
  double f_noiseParameters_process_noise, double g_noiseParameters_process_noise,
  const double c_noiseParameters_gyro_bias_ini[3], const double
  c_noiseParameters_acc_bias_init[3], double noiseParameters_image_noise, double
  c_noiseParameters_inv_depth_ini, int c_VIOParameters_num_points_per_, int
  VIOParameters_num_anchors, int c_VIOParameters_max_ekf_iterati, boolean_T
  VIOParameters_fixed_feature, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_mono, boolean_T VIOParameters_RANSAC)
{
  char cv0[18];
  int i3;
  static const char cv1[18] = { 'N', 'o', 'i', 's', 'e', ' ', 'p', 'a', 'r', 'a',
    'm', 'e', 't', 'e', 'r', 's', ':', '\x00' };

  char cv2[15];
  static const char cv3[15] = { 'V', 'I', 'O', ' ', 'p', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '\x00' };

  int s_size[2];
  static const char cv4[4] = { 'T', 'r', 'u', 'e' };

  char s_data[5];
  static const char cv5[5] = { 'F', 'a', 'l', 's', 'e' };

  b_fprintf();

  // #coder
  // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i3 = 0; i3 < 18; i3++) {
      cv0[i3] = cv1[i3];
    }

    ROS_INFO(cv0);
  }

  c_fprintf(c_noiseParameters_process_noise);
  d_fprintf(d_noiseParameters_process_noise);
  e_fprintf(e_noiseParameters_process_noise);
  f_fprintf(f_noiseParameters_process_noise);
  g_fprintf(g_noiseParameters_process_noise);
  h_fprintf(c_noiseParameters_gyro_bias_ini[0], c_noiseParameters_gyro_bias_ini
            [1], c_noiseParameters_gyro_bias_ini[2]);
  i_fprintf(c_noiseParameters_acc_bias_init[0], c_noiseParameters_acc_bias_init
            [1], c_noiseParameters_acc_bias_init[2]);
  j_fprintf(noiseParameters_image_noise);
  k_fprintf(c_noiseParameters_inv_depth_ini);
  b_fprintf();

  // #coder
  // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i3 = 0; i3 < 15; i3++) {
      cv2[i3] = cv3[i3];
    }

    ROS_INFO(cv2);
  }

  l_fprintf(VIOParameters_num_anchors);
  m_fprintf(c_VIOParameters_num_points_per_);
  n_fprintf(c_VIOParameters_max_ekf_iterati);
  if (c_VIOParameters_delayed_initial) {
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  o_fprintf(s_data, s_size);
  if (VIOParameters_fixed_feature) {
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  p_fprintf(s_data, s_size);
  if (VIOParameters_mono) {
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  q_fprintf(s_data, s_size);
  if (VIOParameters_RANSAC) {
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  r_fprintf(s_data, s_size);
  b_fprintf();
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double q_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i6;
  char varargout_1_data[6];
  static const char cfmt[10] = { 'm', 'o', 'n', 'o', ':', ' ', '%', 's', '\x0a',
    '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i6 = 0; i6 < nbytesint; i6++) {
      varargout_1_data[i6] = varargin_1_data[i6];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
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
  int i8;
  int i9;
  double B;
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
  for (i8 = 0; i8 < 4; i8++) {
    qp[i8] = 0.0;
    for (i9 = 0; i9 < 4; i9++) {
      qp[i8] += b_p[i8 + (i9 << 2)] * b_q[i9];
    }
  }

  B = b_norm(qp);
  for (i8 = 0; i8 < 4; i8++) {
    qp[i8] /= B;
  }
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double r_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i7;
  char varargout_1_data[6];
  static const char cfmt[12] = { 'R', 'A', 'N', 'S', 'A', 'C', ':', ' ', '%',
    's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i7 = 0; i7 < nbytesint; i7++) {
      varargout_1_data[i7] = varargin_1_data[i7];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double x[3]
//                double y
//                double z[3]
// Return Type  : void
//
static void rdivide(const double x[3], double y, double z[3])
{
  int i;
  for (i = 0; i < 3; i++) {
    z[i] = x[i] / y;
  }
}

//
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
static void ros_error()
{
  char cv8[27];
  int i13;
  static const char cv9[27] = { 'p', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n', ' ',
    'i', 'n', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', '\x00' };

  for (i13 = 0; i13 < 27; i13++) {
    cv8[i13] = cv9[i13];
  }

  ROS_ERROR(cv8);
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv6[54];
  int i12;
  static const char cv7[54] = { 'F', 'i', 'x', 'e', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', ' ', 'i', 's', ' ', 'n', 'o',
    ' ', 'l', 'o', 'n', 'g', 'e', 'r', ' ', 'v', 'a', 'l', 'i', 'd', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i12 = 0; i12 < 54; i12++) {
      cv6[i12] = cv7[i12];
    }

    ROS_INFO(cv6, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d6;
  double d7;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d6 = fabs(u0);
    d7 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d6 == 1.0) {
        y = rtNaN;
      } else if (d6 > 1.0) {
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
    } else if (d7 == 0.0) {
      y = 1.0;
    } else if (d7 == 1.0) {
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
// Arguments    : double x[8]
//                int idx[8]
// Return Type  : void
//
static void sort(double x[8], int idx[8])
{
  b_sort(x, idx);
}

//
// Arguments    : emxArray_real_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void sortIdx(emxArray_real_T *x, emxArray_int32_T *idx)
{
  emxArray_real_T *b_x;
  int ib;
  int wOffset;
  int b_m;
  int n;
  double x4[4];
  int idx4[4];
  emxArray_int32_T *iwork;
  emxArray_real_T *xwork;
  int nNaNs;
  int k;
  int i3;
  int i4;
  signed char perm[4];
  int nNonNaN;
  int nBlocks;
  int b_iwork[256];
  double b_xwork[256];
  int bLen2;
  int nPairs;
  int exitg1;
  emxInit_real_T(&b_x, 1);
  ib = x->size[0];
  wOffset = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, wOffset, (int)sizeof(double));
  b_m = x->size[0];
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    b_x->data[wOffset] = x->data[wOffset];
  }

  wOffset = idx->size[0];
  idx->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)idx, wOffset, (int)sizeof(int));
  for (wOffset = 0; wOffset < ib; wOffset++) {
    idx->data[wOffset] = 0;
  }

  n = x->size[0];
  for (b_m = 0; b_m < 4; b_m++) {
    x4[b_m] = 0.0;
    idx4[b_m] = 0;
  }

  emxInit_int32_T1(&iwork, 1);
  wOffset = iwork->size[0];
  iwork->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)iwork, wOffset, (int)sizeof(int));
  b_m = iwork->size[0];
  wOffset = iwork->size[0];
  iwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)iwork, wOffset, (int)sizeof(int));
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    iwork->data[wOffset] = 0;
  }

  emxInit_real_T(&xwork, 1);
  b_m = x->size[0];
  wOffset = xwork->size[0];
  xwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)xwork, wOffset, (int)sizeof(double));
  b_m = xwork->size[0];
  wOffset = xwork->size[0];
  xwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)xwork, wOffset, (int)sizeof(double));
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    xwork->data[wOffset] = 0.0;
  }

  nNaNs = 1;
  ib = 0;
  for (k = 0; k + 1 <= n; k++) {
    if (rtIsNaN(b_x->data[k])) {
      idx->data[n - nNaNs] = k + 1;
      xwork->data[n - nNaNs] = b_x->data[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = k + 1;
      x4[ib - 1] = b_x->data[k];
      if (ib == 4) {
        ib = k - nNaNs;
        if (x4[0] <= x4[1]) {
          b_m = 1;
          wOffset = 2;
        } else {
          b_m = 2;
          wOffset = 1;
        }

        if (x4[2] <= x4[3]) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }

        if (x4[b_m - 1] <= x4[i3 - 1]) {
          if (x4[wOffset - 1] <= x4[i3 - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)wOffset;
            perm[2] = (signed char)i3;
            perm[3] = (signed char)i4;
          } else if (x4[wOffset - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)i3;
            perm[2] = (signed char)wOffset;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)i3;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)wOffset;
          }
        } else if (x4[b_m - 1] <= x4[i4 - 1]) {
          if (x4[wOffset - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)wOffset;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)wOffset;
          }
        } else {
          perm[0] = (signed char)i3;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)b_m;
          perm[3] = (signed char)wOffset;
        }

        idx->data[ib - 2] = idx4[perm[0] - 1];
        idx->data[ib - 1] = idx4[perm[1] - 1];
        idx->data[ib] = idx4[perm[2] - 1];
        idx->data[ib + 1] = idx4[perm[3] - 1];
        b_x->data[ib - 2] = x4[perm[0] - 1];
        b_x->data[ib - 1] = x4[perm[1] - 1];
        b_x->data[ib] = x4[perm[2] - 1];
        b_x->data[ib + 1] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  wOffset = x->size[0] - nNaNs;
  if (ib > 0) {
    for (b_m = 0; b_m < 4; b_m++) {
      perm[b_m] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (k = 1; k <= ib; k++) {
      idx->data[(wOffset - ib) + k] = idx4[perm[k - 1] - 1];
      b_x->data[(wOffset - ib) + k] = x4[perm[k - 1] - 1];
    }
  }

  b_m = (nNaNs - 1) >> 1;
  for (k = 1; k <= b_m; k++) {
    ib = idx->data[wOffset + k];
    idx->data[wOffset + k] = idx->data[n - k];
    idx->data[n - k] = ib;
    b_x->data[wOffset + k] = xwork->data[n - k];
    b_x->data[n - k] = xwork->data[wOffset + k];
  }

  if (((nNaNs - 1) & 1) != 0) {
    b_x->data[(wOffset + b_m) + 1] = xwork->data[(wOffset + b_m) + 1];
  }

  nNonNaN = (x->size[0] - nNaNs) + 1;
  b_m = 2;
  if (nNonNaN > 1) {
    if (x->size[0] >= 256) {
      nBlocks = nNonNaN >> 8;
      if (nBlocks > 0) {
        for (i3 = 1; i3 <= nBlocks; i3++) {
          i4 = ((i3 - 1) << 8) - 1;
          for (nNaNs = 0; nNaNs < 6; nNaNs++) {
            n = 1 << (nNaNs + 2);
            bLen2 = n << 1;
            nPairs = 256 >> (nNaNs + 3);
            for (k = 1; k <= nPairs; k++) {
              b_m = i4 + (k - 1) * bLen2;
              for (ib = 1; ib <= bLen2; ib++) {
                b_iwork[ib - 1] = idx->data[b_m + ib];
                b_xwork[ib - 1] = b_x->data[b_m + ib];
              }

              wOffset = 0;
              ib = n;
              do {
                exitg1 = 0;
                b_m++;
                if (b_xwork[wOffset] <= b_xwork[ib]) {
                  idx->data[b_m] = b_iwork[wOffset];
                  b_x->data[b_m] = b_xwork[wOffset];
                  if (wOffset + 1 < n) {
                    wOffset++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  idx->data[b_m] = b_iwork[ib];
                  b_x->data[b_m] = b_xwork[ib];
                  if (ib + 1 < bLen2) {
                    ib++;
                  } else {
                    ib = b_m - wOffset;
                    while (wOffset + 1 <= n) {
                      idx->data[(ib + wOffset) + 1] = b_iwork[wOffset];
                      b_x->data[(ib + wOffset) + 1] = b_xwork[wOffset];
                      wOffset++;
                    }

                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
        }

        b_m = nBlocks << 8;
        ib = nNonNaN - b_m;
        if (ib > 0) {
          merge_block(idx, b_x, b_m, ib, 2, iwork, xwork);
        }

        b_m = 8;
      }
    }

    merge_block(idx, b_x, 0, nNonNaN, b_m, iwork, xwork);
  }

  emxFree_real_T(&xwork);
  emxFree_int32_T(&iwork);
  wOffset = x->size[0];
  x->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)x, wOffset, (int)sizeof(double));
  b_m = b_x->size[0];
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    x->data[wOffset] = b_x->data[wOffset];
  }

  emxFree_real_T(&b_x);
}

//
// UNDISTORTPOINT Undistort a point (or points) that are from a camera with
// the calibration cameraparams
//    Undistort a point or set of points from one camera. Depending on the
//    camera model used to calibrate the camera, the appropriate undistortion
//    is applied
// Arguments    : const double pt_d_data[]
//                const int pt_d_size[1]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                const double cameraparams_RadialDistortion[3]
//                double pt_u_data[]
//                int pt_u_size[1]
// Return Type  : void
//
static void undistortPoint(const double pt_d_data[], const int pt_d_size[1],
  const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double cameraparams_RadialDistortion[3],
  double pt_u_data[], int pt_u_size[1])
{
  int loop_ub;
  int i11;
  double fx;
  double fy;
  double Cx;
  double Cy;
  double k1;
  double k2;
  double k3;
  double d1;
  int i;
  double pt_d_n[2];
  double r_d_sq;
  double r_u_sq;
  int b_i;
  boolean_T exitg1;
  double a;
  double b_a;
  double diff;
  double coeff;

  // % Plumb Bob
  pt_u_size[0] = pt_d_size[0];
  loop_ub = pt_d_size[0];
  for (i11 = 0; i11 < loop_ub; i11++) {
    pt_u_data[i11] = pt_d_data[i11];
  }

  fx = cameraparams_FocalLength[0];
  fy = cameraparams_FocalLength[1];
  Cx = cameraparams_PrincipalPoint[0];
  Cy = cameraparams_PrincipalPoint[1];
  k1 = cameraparams_RadialDistortion[0];
  k2 = cameraparams_RadialDistortion[1];
  k3 = cameraparams_RadialDistortion[2];
  d1 = (double)pt_d_size[0] / 2.0;
  for (i = 0; i < (int)d1; i++) {
    pt_d_n[0] = (pt_d_data[i << 1] - Cx) / fx;
    pt_d_n[1] = (pt_d_data[(i << 1) + 1] - Cy) / fy;
    r_d_sq = pt_d_n[0] * pt_d_n[0] + pt_d_n[1] * pt_d_n[1];

    // get_r_u Get undistorted radius from distorted radius
    //    Get the pixel radius of the undistorted pixels from a distorted pixel
    //    radius and distortion parameters
    r_u_sq = r_d_sq;
    b_i = 0;
    exitg1 = false;
    while ((!exitg1) && (b_i < 100)) {
      a = ((1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powd_snf
        (r_u_sq, 3.0);
      b_a = (1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq) * k3 * rt_powd_snf
        (r_u_sq, 3.0);
      diff = (r_u_sq * (a * a) - r_d_sq) / (b_a * b_a + 2.0 * r_u_sq * (((1.0 +
        2.0 * k1 * r_u_sq) + 2.0 * k2 * (r_u_sq * r_u_sq)) + 2.0 * k3 *
        rt_powd_snf(r_u_sq, 3.0)));
      r_u_sq -= diff;
      if ((diff < 1.0E-6) && (diff > -1.0E-6)) {
        exitg1 = true;
      } else {
        b_i++;
      }
    }

    coeff = ((1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powd_snf
      (r_u_sq, 3.0);
    for (i11 = 0; i11 < 2; i11++) {
      pt_d_n[i11] /= coeff;
    }

    b_i = i << 1;
    pt_u_data[b_i] = pt_d_n[0] * fx + Cx;
    pt_u_data[1 + b_i] = pt_d_n[1] * fy + Cy;
  }
}

//
// Arguments    : emxArray_real_T *A
//                emxArray_real_T *tau
//                emxArray_int32_T *jpvt
// Return Type  : void
//
static void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T
                   *jpvt)
{
  int b_m;
  int n;
  int mn;
  int i47;
  emxArray_real_T *work;
  int itemp;
  emxArray_real_T *vn1;
  emxArray_real_T *vn2;
  int k;
  int nmi;
  int i;
  int i_i;
  int mmi;
  int ix;
  double smax;
  double s;
  int pvt;
  int iy;
  double absxk;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  double t;
  b_m = A->size[0];
  n = A->size[1];
  if (A->size[0] <= A->size[1]) {
    mn = A->size[0];
  } else {
    mn = A->size[1];
  }

  i47 = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity((emxArray__common *)tau, i47, (int)sizeof(double));
  eml_signed_integer_colon(A->size[1], jpvt);
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
  } else {
    emxInit_real_T(&work, 1);
    itemp = A->size[1];
    i47 = work->size[0];
    work->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)work, i47, (int)sizeof(double));
    for (i47 = 0; i47 < itemp; i47++) {
      work->data[i47] = 0.0;
    }

    emxInit_real_T(&vn1, 1);
    emxInit_real_T(&vn2, 1);
    itemp = A->size[1];
    i47 = vn1->size[0];
    vn1->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)vn1, i47, (int)sizeof(double));
    i47 = vn2->size[0];
    vn2->size[0] = vn1->size[0];
    emxEnsureCapacity((emxArray__common *)vn2, i47, (int)sizeof(double));
    k = 1;
    for (nmi = 0; nmi + 1 <= n; nmi++) {
      vn1->data[nmi] = xnrm2(b_m, A, k);
      vn2->data[nmi] = vn1->data[nmi];
      k += b_m;
    }

    for (i = 0; i + 1 <= mn; i++) {
      i_i = i + i * b_m;
      nmi = n - i;
      mmi = (b_m - i) - 1;
      if (nmi < 1) {
        itemp = -1;
      } else {
        itemp = 0;
        if (nmi > 1) {
          ix = i;
          smax = fabs(vn1->data[i]);
          for (k = 0; k + 2 <= nmi; k++) {
            ix++;
            s = fabs(vn1->data[ix]);
            if (s > smax) {
              itemp = k + 1;
              smax = s;
            }
          }
        }
      }

      pvt = i + itemp;
      if (pvt + 1 != i + 1) {
        ix = b_m * pvt;
        iy = b_m * i;
        for (k = 1; k <= b_m; k++) {
          smax = A->data[ix];
          A->data[ix] = A->data[iy];
          A->data[iy] = smax;
          ix++;
          iy++;
        }

        itemp = jpvt->data[pvt];
        jpvt->data[pvt] = jpvt->data[i];
        jpvt->data[i] = itemp;
        vn1->data[pvt] = vn1->data[i];
        vn2->data[pvt] = vn2->data[i];
      }

      if (i + 1 < b_m) {
        absxk = A->data[i_i];
        s = 0.0;
        if (1 + mmi <= 0) {
        } else {
          smax = b_xnrm2(mmi, A, i_i + 2);
          if (smax != 0.0) {
            smax = hypot(A->data[i_i], smax);
            if (A->data[i_i] >= 0.0) {
              smax = -smax;
            }

            if (fabs(smax) < 1.0020841800044864E-292) {
              pvt = 0;
              do {
                pvt++;
                xscal(mmi, 9.9792015476736E+291, A, i_i + 2);
                smax *= 9.9792015476736E+291;
                absxk *= 9.9792015476736E+291;
              } while (!(fabs(smax) >= 1.0020841800044864E-292));

              smax = b_xnrm2(mmi, A, i_i + 2);
              smax = hypot(absxk, smax);
              if (absxk >= 0.0) {
                smax = -smax;
              }

              s = (smax - absxk) / smax;
              xscal(mmi, 1.0 / (absxk - smax), A, i_i + 2);
              for (k = 1; k <= pvt; k++) {
                smax *= 1.0020841800044864E-292;
              }

              absxk = smax;
            } else {
              s = (smax - A->data[i_i]) / smax;
              xscal(mmi, 1.0 / (A->data[i_i] - smax), A, i_i + 2);
              absxk = smax;
            }
          }
        }

        tau->data[i] = s;
        A->data[i_i] = absxk;
      } else {
        tau->data[i] = 0.0;
      }

      if (i + 1 < n) {
        absxk = A->data[i_i];
        A->data[i_i] = 1.0;
        i_ip1 = (i + (i + 1) * b_m) + 1;
        if (tau->data[i] != 0.0) {
          lastv = mmi;
          itemp = i_i + mmi;
          while ((lastv + 1 > 0) && (A->data[itemp] == 0.0)) {
            lastv--;
            itemp--;
          }

          lastc = nmi - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            itemp = i_ip1 + (lastc - 1) * b_m;
            k = itemp;
            do {
              exitg1 = 0;
              if (k <= itemp + lastv) {
                if (A->data[k - 1] != 0.0) {
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
          lastv = -1;
          lastc = 0;
        }

        if (lastv + 1 > 0) {
          if (lastc == 0) {
          } else {
            for (iy = 1; iy <= lastc; iy++) {
              work->data[iy - 1] = 0.0;
            }

            iy = 0;
            i47 = i_ip1 + b_m * (lastc - 1);
            itemp = i_ip1;
            while ((b_m > 0) && (itemp <= i47)) {
              ix = i_i;
              smax = 0.0;
              pvt = itemp + lastv;
              for (k = itemp; k <= pvt; k++) {
                smax += A->data[k - 1] * A->data[ix];
                ix++;
              }

              work->data[iy] += smax;
              iy++;
              itemp += b_m;
            }
          }

          if (-tau->data[i] == 0.0) {
          } else {
            itemp = 0;
            for (nmi = 1; nmi <= lastc; nmi++) {
              if (work->data[itemp] != 0.0) {
                smax = work->data[itemp] * -tau->data[i];
                ix = i_i;
                i47 = lastv + i_ip1;
                for (pvt = i_ip1; pvt <= i47; pvt++) {
                  A->data[pvt - 1] += A->data[ix] * smax;
                  ix++;
                }
              }

              itemp++;
              i_ip1 += b_m;
            }
          }
        }

        A->data[i_i] = absxk;
      }

      for (nmi = i + 1; nmi + 1 <= n; nmi++) {
        itemp = (i + b_m * nmi) + 1;
        if (vn1->data[nmi] != 0.0) {
          smax = fabs(A->data[i + A->size[0] * nmi]) / vn1->data[nmi];
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }

          s = vn1->data[nmi] / vn2->data[nmi];
          s = smax * (s * s);
          if (s <= 1.4901161193847656E-8) {
            if (i + 1 < b_m) {
              smax = 0.0;
              if (mmi < 1) {
              } else if (mmi == 1) {
                smax = fabs(A->data[itemp]);
              } else {
                s = 2.2250738585072014E-308;
                pvt = itemp + mmi;
                while (itemp + 1 <= pvt) {
                  absxk = fabs(A->data[itemp]);
                  if (absxk > s) {
                    t = s / absxk;
                    smax = 1.0 + smax * t * t;
                    s = absxk;
                  } else {
                    t = absxk / s;
                    smax += t * t;
                  }

                  itemp++;
                }

                smax = s * sqrt(smax);
              }

              vn1->data[nmi] = smax;
              vn2->data[nmi] = vn1->data[nmi];
            } else {
              vn1->data[nmi] = 0.0;
              vn2->data[nmi] = 0.0;
            }
          } else {
            vn1->data[nmi] *= sqrt(smax);
          }
        }
      }
    }

    emxFree_real_T(&vn2);
    emxFree_real_T(&vn1);
    emxFree_real_T(&work);
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double xnrm2(int n, const emxArray_real_T *x, int ix0)
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
//                double a
//                emxArray_real_T *x
//                int ix0
// Return Type  : void
//
static void xscal(int n, double a, emxArray_real_T *x, int ix0)
{
  int i48;
  int k;
  i48 = (ix0 + n) - 1;
  for (k = ix0; k <= i48; k++) {
    x->data[k - 1] *= a;
  }
}

//
// input
//  NOTE: Comment this out for MEXing
// Arguments    : int updateVect[40]
//                const double z_all_l[80]
//                const double z_all_r[80]
//                double dt
//                const VIOMeasurements *measurements
//                const DUOParameters *cameraParameters
//                const NoiseParameters *noiseParameters
//                const VIOParameters *b_VIOParameters
//                boolean_T vision
//                RobotState *xt_out
//                double h_u_out[160]
//                double map_out[120]
//                AnchorPose anchor_poses_out[5]
//                double delayedStatus_out[40]
// Return Type  : void
//
void SLAM(int updateVect[40], const double z_all_l[80], const double z_all_r[80],
          double dt, const VIOMeasurements *measurements, const DUOParameters
          *cameraParameters, const NoiseParameters *noiseParameters, const
          VIOParameters *b_VIOParameters, boolean_T vision, RobotState *xt_out,
          double h_u_out[160], double map_out[120], AnchorPose anchor_poses_out
          [5], double delayedStatus_out[40])
{
  int i;
  static const signed char iv2[4] = { 0, 0, 0, 1 };

  static const f_struct_T rv0[5] = { { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 },
      { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0,
          0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, {
            0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0,
        0.0 }, { 0.0, 0.0, 0.0, 1.0 }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0,
            0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0,
            0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 }, {
        0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0,
          0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, {
            0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0,
        0.0 }, { 0.0, 0.0, 0.0, 1.0 }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0,
            0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0,
            0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 }, {
        0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0,
          0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, {
            0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 } } } };

  int anchorIdx;
  int i34;
  int i35;
  double z_b[3];
  double B;
  double y_n_b[3];
  static const double dv3[3] = { 0.0, 0.0, 1.0 };

  double b_y_n_b[3];
  double x_n_b[3];
  double dv4[9];
  double b_x_n_b[9];
  double R_cw_init[9];
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double dv5[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  double b_z_all_l[80];
  double b_z_all_r[80];
  double t0_IMU_att[4];
  double t0_IMU_pos[3];
  double t0_IMU_acc_bias[3];
  double t0_IMU_gyro_bias[3];
  double t0_vel[3];
  double t0_att[4];
  struct_T rv1[5];

  //  TODO
  //  coder.cstructname(ControllerGains, 'ControllerGains', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  //  coder.cstructname(ref, 'ReferenceCommand', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  if (!initialized_not_empty) {
    for (i = 0; i < 40; i++) {
      updateVect[i] = 0;
    }

    QuatFromRotJ(cameraParameters->R_ci, xt.robot_state.IMU.att);
    for (i = 0; i < 3; i++) {
      xt.robot_state.IMU.pos[i] = cameraParameters->t_ci[i];
      xt.robot_state.IMU.gyro_bias[i] = cameraParameters->gyro_bias[i];
      xt.robot_state.IMU.acc_bias[i] = cameraParameters->acc_bias[i];
      xt.robot_state.pos[i] = 0.0;
    }

    //  position relative to the origin frame
    for (i = 0; i < 4; i++) {
      xt.robot_state.att[i] = iv2[i];
    }

    //  orientation relative to the origin frame
    //  velocity in the origin frame
    xt.fixed_feature = 0;
    xt.origin.anchor_idx = 0;

    //  idx of the anchor that is at the origin
    for (i = 0; i < 3; i++) {
      xt.robot_state.vel[i] = 0.0;
      xt.origin.pos[i] = 0.0;
    }

    //  position of the origin in the world frame
    for (i = 0; i < 4; i++) {
      xt.origin.att[i] = iv2[i];
    }

    //  orientation of the origin in the world frame
    memset(&P[0], 0, 8281U * sizeof(double));

    //  initial error state covariance
    memcpy(&xt.anchor_states[0], &rv0[0], 5U * sizeof(f_struct_T));
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      i34 = anchorIdx * 14;
      for (i35 = 0; i35 < 6; i35++) {
        xt.anchor_states[anchorIdx].P_idx[i35] = (i35 + i34) + 22;
      }
    }

    if (vision) {
      memset(&h_u[0], 0, 160U * sizeof(double));
      memset(&map[0], 0, 120U * sizeof(double));
      memset(&delayedStatus[0], 0, 40U * sizeof(double));
    } else {
      for (i = 0; i < 3; i++) {
        z_b[i] = measurements->acc_duo[i] - xt.robot_state.IMU.acc_bias[i];
      }

      B = norm(z_b);
      for (i34 = 0; i34 < 3; i34++) {
        z_b[i34] /= B;
      }

      cross(z_b, dv3, y_n_b);
      for (i = 0; i < 3; i++) {
        b_y_n_b[i] = y_n_b[i];
      }

      rdivide(b_y_n_b, norm(y_n_b), y_n_b);
      cross(y_n_b, z_b, x_n_b);
      for (i = 0; i < 3; i++) {
        b_y_n_b[i] = x_n_b[i];
      }

      rdivide(b_y_n_b, norm(x_n_b), x_n_b);

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      dv4[0] = ((xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0] -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      dv4[3] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] +
                      xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      dv4[6] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] -
                      xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      dv4[1] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] -
                      xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      dv4[4] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) +
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      dv4[7] = 2.0 * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] +
                      xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      dv4[2] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] +
                      xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      dv4[5] = 2.0 * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] -
                      xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      dv4[8] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) +
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      for (i34 = 0; i34 < 3; i34++) {
        b_x_n_b[i34] = x_n_b[i34];
        b_x_n_b[3 + i34] = y_n_b[i34];
        b_x_n_b[6 + i34] = z_b[i34];
      }

      for (i34 = 0; i34 < 3; i34++) {
        for (i35 = 0; i35 < 3; i35++) {
          R_cw_init[i34 + 3 * i35] = 0.0;
          for (i = 0; i < 3; i++) {
            R_cw_init[i34 + 3 * i35] += dv4[i34 + 3 * i] * b_x_n_b[i + 3 * i35];
          }

          P[i35 + 91 * i34] = 0.0;
          P[(i35 + 91 * (3 + i34)) + 3] = 0.0;
          P[(i35 + 91 * (6 + i34)) + 6] = y[i35 + 3 * i34];
        }
      }

      QuatFromRotJ(R_cw_init, xt.origin.att);

      //  orientation of the origin in the world frame
      //  position
      //  orientation of camera in origin frame
      //  velocity
      diag(noiseParameters->gyro_bias_initial_unc, dv4);

      //  gyro bias
      diag(noiseParameters->acc_bias_initial_unc, b_x_n_b);

      //  acc bias
      for (i34 = 0; i34 < 3; i34++) {
        for (i35 = 0; i35 < 3; i35++) {
          P[(i35 + 91 * (9 + i34)) + 9] = dv4[i35 + 3 * i34];
          P[(i35 + 91 * (12 + i34)) + 12] = b_x_n_b[i35 + 3 * i34];
          dv5[i34 + 3 * i35] = 0.0;
          for (i = 0; i < 3; i++) {
            dv5[i34 + 3 * i35] += 0.1 * R_cw_init[i34 + 3 * i] * (double)b[i + 3
              * i35];
          }
        }
      }

      for (i34 = 0; i34 < 3; i34++) {
        for (i35 = 0; i35 < 3; i35++) {
          P[(i34 + 91 * (15 + i35)) + 15] = 0.0;
          for (i = 0; i < 3; i++) {
            P[(i34 + 91 * (15 + i35)) + 15] += dv5[i34 + 3 * i] * R_cw_init[i35
              + 3 * i];
          }
        }
      }

      //  origin orientation
      for (i34 = 0; i34 < 3; i34++) {
        for (i35 = 0; i35 < 3; i35++) {
          P[(i35 + 91 * (18 + i34)) + 18] = 0.0;
        }
      }

      //  R_ci
      memset(&h_u[0], 0, 160U * sizeof(double));
      getMap(xt.origin.pos, xt.origin.att, xt.anchor_states, map);
      memset(&delayedStatus[0], 0, 40U * sizeof(double));
      printParams(noiseParameters->process_noise.qv,
                  noiseParameters->process_noise.qw,
                  noiseParameters->process_noise.qao,
                  noiseParameters->process_noise.qwo,
                  noiseParameters->process_noise.qR_ci,
                  noiseParameters->gyro_bias_initial_unc,
                  noiseParameters->acc_bias_initial_unc,
                  noiseParameters->image_noise,
                  noiseParameters->inv_depth_initial_unc,
                  b_VIOParameters->num_points_per_anchor,
                  b_VIOParameters->num_anchors,
                  b_VIOParameters->max_ekf_iterations,
                  b_VIOParameters->fixed_feature,
                  b_VIOParameters->delayed_initialization, b_VIOParameters->mono,
                  b_VIOParameters->RANSAC);
      initialized_not_empty = true;
    }
  } else if (!vision) {
    //      [xt,P] =  SLAM_pred(P, xt, dt, noiseParameters.process_noise, measurements, numStates); 
    SLAM_pred_euler(P, &xt, dt, noiseParameters->process_noise.qv,
                    noiseParameters->process_noise.qw,
                    noiseParameters->process_noise.qao,
                    noiseParameters->process_noise.qwo,
                    noiseParameters->process_noise.qR_ci, measurements->acc_duo,
                    measurements->gyr_duo);
  } else {
    memcpy(&b_z_all_l[0], &z_all_l[0], 80U * sizeof(double));
    memcpy(&b_z_all_r[0], &z_all_r[0], 80U * sizeof(double));
    SLAM_upd(P, &xt, cameraParameters->CameraParameters1.FocalLength,
             cameraParameters->CameraParameters1.PrincipalPoint,
             cameraParameters->CameraParameters1.RadialDistortion,
             cameraParameters->CameraParameters2.FocalLength,
             cameraParameters->CameraParameters2.PrincipalPoint,
             cameraParameters->CameraParameters2.RadialDistortion,
             cameraParameters->r_lr, cameraParameters->R_lr,
             cameraParameters->R_rl, updateVect, b_z_all_l, b_z_all_r,
             noiseParameters->image_noise,
             noiseParameters->inv_depth_initial_unc, *b_VIOParameters, h_u, map,
             delayedStatus);
  }

  memcpy(&h_u_out[0], &h_u[0], 160U * sizeof(double));
  memcpy(&map_out[0], &map[0], 120U * sizeof(double));
  getWorldState(xt.robot_state.IMU.pos, xt.robot_state.IMU.att,
                xt.robot_state.IMU.gyro_bias, xt.robot_state.IMU.acc_bias,
                xt.robot_state.pos, xt.robot_state.att, xt.origin.pos,
                xt.origin.att, b_y_n_b, t0_att, t0_vel, t0_IMU_gyro_bias,
                t0_IMU_acc_bias, t0_IMU_pos, t0_IMU_att);
  for (i = 0; i < 3; i++) {
    xt_out->pos[i] = b_y_n_b[i];
  }

  for (i = 0; i < 4; i++) {
    xt_out->att[i] = t0_att[i];
  }

  for (i = 0; i < 3; i++) {
    xt_out->vel[i] = t0_vel[i];
    xt_out->IMU.gyro_bias[i] = t0_IMU_gyro_bias[i];
    xt_out->IMU.acc_bias[i] = t0_IMU_acc_bias[i];
    xt_out->IMU.pos[i] = t0_IMU_pos[i];
  }

  for (i = 0; i < 4; i++) {
    xt_out->IMU.att[i] = t0_IMU_att[i];
  }

  getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, rv1);
  cast(rv1, anchor_poses_out);
  memcpy(&delayedStatus_out[0], &delayedStatus[0], 40U * sizeof(double));

  //  output
  //  NOTE: Comment this out for MEXing
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  rt_InitInfAndNaN(8U);
  debug_level = b_debug_level;
  initialized_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_terminate()
{
  // (no terminate code required)
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
