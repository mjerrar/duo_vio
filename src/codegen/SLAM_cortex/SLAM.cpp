//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 10-Mar-2016 13:31:13
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include <stdio.h>
#include <vio_logging.h>

// Custom Source Code
//***************************************************************************
//
//    Copyright (c) 2015-2016 AIT, ETH Zurich. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  1. Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//  3. Neither the name AIT nor the names of its contributors may be
//     used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//
// **************************************************************************

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

#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T

struct emxArray_real32_T
{
  float *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real32_T

#ifndef struct_emxArray_real32_T_1x1
#define struct_emxArray_real32_T_1x1

struct emxArray_real32_T_1x1
{
  float data[1];
  int size[2];
};

#endif                                 //struct_emxArray_real32_T_1x1

typedef struct {
  float pos[3];
  float att[4];
  __attribute__((aligned(16))) float gyro_bias[3];
  __attribute__((aligned(16))) float acc_bias[3];
} b_struct_T;

typedef struct {
  b_struct_T IMU;
  __attribute__((aligned(16))) float pos[3];
  __attribute__((aligned(16))) float att[4];
  __attribute__((aligned(16))) float vel[3];
} c_struct_T;

typedef struct {
  int anchor_idx;
  __attribute__((aligned(16))) float pos[3];
  float att[4];
} d_struct_T;

typedef struct {
  float inverse_depth;
  float m[3];
  float scaled_map_point[3];
  int status;
  int status_idx;
  int P_idx;
} e_struct_T;

typedef struct {
  __attribute__((aligned(16))) float pos[3];
  float att[4];
  int P_idx[6];
  e_struct_T feature_states[8];
} f_struct_T;

typedef struct {
  c_struct_T robot_state;
  int fixed_feature;
  d_struct_T origin;
  f_struct_T anchor_states[6];
} g_struct_T;

typedef struct {
  __attribute__((aligned(16))) float pos[3];
  float att[4];
} struct_T;

// Named Constants
#define b_debug_level                  (1.0F)

// Variable Definitions
static emxArray_real32_T_1x1 initialized;
static boolean_T initialized_not_empty;
static g_struct_T xt;
static float P[10404];
static float map[144];
static float delayedStatus[48];
static float debug_level;

// Function Declarations
static void OnePointRANSAC_EKF(g_struct_T *b_xt, float b_P[10404], const float
  z_u_l[96], const float z_u_r[96], const float c_stereoParams_CameraParameters
  [2], const float d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], float noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_full_stereo, boolean_T VIOParameters_RANSAC, int
  updateVect[48]);
static void SLAM_free();
static void SLAM_init();
static void SLAM_upd(float P_apr[10404], g_struct_T *b_xt, int
                     c_cameraParams_CameraParameters, const float
                     d_cameraParams_CameraParameters[2], const float
                     e_cameraParams_CameraParameters[2], const float
                     f_cameraParams_CameraParameters[3], int
                     g_cameraParams_CameraParameters, int
                     h_cameraParams_CameraParameters, const float
                     i_cameraParams_CameraParameters[2], const float
                     j_cameraParams_CameraParameters[2], const float
                     k_cameraParams_CameraParameters[3], int
                     l_cameraParams_CameraParameters, const float
                     cameraParams_r_lr[3], const float cameraParams_R_lr[9],
                     const float cameraParams_R_rl[9], int updateVect[48], float
                     z_all_l[96], float z_all_r[96], float
                     noiseParameters_image_noise, float
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, float b_map[144], float b_delayedStatus[48]);
static boolean_T any(const boolean_T x[48]);
static boolean_T anyActiveAnchorFeatures(const e_struct_T
  anchor_state_feature_states[8]);
static boolean_T b_any(const boolean_T x[3]);
static double b_fprintf();
static void b_getH_R_res(const float xt_robot_state_pos[3], const float
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[6], const float
  z_all_l[96], const float z_all_r[96], const boolean_T b_status[48], const
  float c_stereoParams_CameraParameters[2], const float
  d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], boolean_T VIOParameters_full_stereo,
  emxArray_real32_T *r, emxArray_real32_T *H);
static void b_log_error();
static void b_log_info(int varargin_1, int varargin_2, int varargin_3);
static void b_log_warn();
static void b_merge(emxArray_int32_T *idx, emxArray_real32_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real32_T
                    *xwork);
static float b_norm(const float x[4]);
static void b_sort(float x[8], int idx[8]);
static void b_xaxpy(int n, float a, const float x[36], int ix0, float y[6], int
                    iy0);
static void b_xgeqp3(float A[30], float tau[5], int jpvt[5]);
static float b_xnrm2(int n, const emxArray_real32_T *x, int ix0);
static boolean_T c_any(const float x[6]);
static double c_fprintf(float varargin_1);
static void c_log_info(int varargin_1, int varargin_2);
static void c_log_warn(int varargin_1, float varargin_2);
static float c_norm(const float x[2]);
static void c_sort(emxArray_real32_T *x, emxArray_int32_T *idx);
static void c_xaxpy(int n, float a, const float x[6], int ix0, float y[36], int
                    iy0);
static float c_xnrm2(int n, const emxArray_real32_T *x, int ix0);
static void cast(const struct_T x[6], AnchorPose y[6]);
static void cross(const float a[3], const float b[3], float c[3]);
static double d_fprintf(float varargin_1);
static void d_log_info(int varargin_1);
static void d_log_warn(signed char varargin_1);
static float d_norm(const float x[36]);
static void d_sort(emxArray_real32_T *x, int dim, emxArray_int32_T *idx);
static float d_xnrm2(int n, const float x[30], int ix0);
static void diag(const float v[3], float d[9]);
static int div_s32_floor(int numerator, int denominator);
static double e_fprintf(float varargin_1);
static void e_log_info(int varargin_1);
static void e_log_warn(int varargin_1, float varargin_2);
static float e_xnrm2(int n, const float x[36], int ix0);
static void eml_signed_integer_colon(int b, emxArray_int32_T *y);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_real32_T(emxArray_real32_T **pEmxArray);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_int32_T1(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int b_numDimensions);
static void emxInit_real32_T1(emxArray_real32_T **pEmxArray, int b_numDimensions);
static double f_fprintf(float varargin_1);
static void f_log_info(int varargin_1, float varargin_2, float varargin_3);
static void f_log_warn(float varargin_1);
static float f_xnrm2(int n, const float x[6], int ix0);
static void fileManager(FILE * *f, boolean_T *a);
static double g_fprintf(float varargin_1);
static void g_log_info(int varargin_1);
static void g_log_warn(int varargin_1, int varargin_2, int varargin_3);
static void getAnchorPoses(const float xt_origin_pos[3], const float
  xt_origin_att[4], const f_struct_T xt_anchor_states[6], struct_T anchor_poses
  [6]);
static void getH_R_res(const float xt_robot_state_pos[3], const float
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[6], const float
  z_all_l[96], const float z_all_r[96], const boolean_T b_status[48], const
  float c_stereoParams_CameraParameters[2], const float
  d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], boolean_T VIOParameters_full_stereo,
  emxArray_real32_T *r, emxArray_real32_T *H, emxArray_int32_T *ind);
static void getMap(const float xt_origin_pos[3], const float xt_origin_att[4],
                   const f_struct_T xt_anchor_states[6], float b_map[144]);
static float getNumValidFeatures(const e_struct_T anchor_state_feature_states[8]);
static void getScaledMap(g_struct_T *b_xt);
static float getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[6]);
static float getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[6]);
static void getWorldState(const float xt_robot_state_IMU_pos[3], const float
  xt_robot_state_IMU_att[4], const float xt_robot_state_IMU_gyro_bias[3], const
  float xt_robot_state_IMU_acc_bias[3], const float xt_robot_state_pos[3], const
  float xt_robot_state_att[4], const float xt_robot_state_vel[3], const float
  xt_origin_pos[3], const float xt_origin_att[4], float world_state_pos[3],
  float world_state_att[4], float world_state_vel[3], float
  world_state_IMU_gyro_bias[3], float world_state_IMU_acc_bias[3], float
  world_state_IMU_pos[3], float world_state_IMU_att[4]);
static double h_fprintf(float varargin_1, float varargin_2, float varargin_3);
static void h_log_warn(int varargin_1);
static double i_fprintf(float varargin_1, float varargin_2, float varargin_3);
static void i_log_warn();
static void initializePoint(const float z_u_l[2], const float z_u_r[2], const
  float c_cameraparams_CameraParameters[2], const float
  d_cameraparams_CameraParameters[2], const float
  e_cameraparams_CameraParameters[2], const float
  f_cameraparams_CameraParameters[2], const float cameraparams_r_lr[3], const
  float cameraparams_R_lr[9], float fp[3], float b_m[6], boolean_T *success);
static double j_fprintf(float varargin_1);
static double k_fprintf(float varargin_1);
static double l_fprintf(int varargin_1);
static void log_error();
static void log_info(int varargin_1, int varargin_2, int varargin_3);
static void log_warn(int varargin_1, int varargin_2, int varargin_3);
static double m_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static float median(const float x_data[], const int x_size[1]);
static void merge(int idx[8], float x[8], int offset, int np, int nq, int iwork
                  [8], float xwork[8]);
static void merge_block(emxArray_int32_T *idx, emxArray_real32_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real32_T *xwork);
static int mul_s32_s32_s32_sat(int a, int b);
static void mul_wide_s32(int in0, int in1, unsigned int *ptrOutBitsHi, unsigned
  int *ptrOutBitsLo);
static void multiplyIdx(const float idx_data[], const int idx_size[1], float
  idx_mult_data[], int idx_mult_size[1]);
static double n_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static float norm(const float x[3]);
static double o_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static double p_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void predictMeasurementStereo(const float fp_l[3], const float
  c_stereoParams_CameraParameters[2], const float
  d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], float h_cin_l[2], float h_cin_r[2]);
static void printParams(float c_noiseParameters_process_noise, float
  d_noiseParameters_process_noise, float e_noiseParameters_process_noise, float
  f_noiseParameters_process_noise, float g_noiseParameters_process_noise, const
  float c_noiseParameters_gyro_bias_ini[3], const float
  c_noiseParameters_acc_bias_init[3], float noiseParameters_image_noise, float
  c_noiseParameters_inv_depth_ini, int c_VIOParameters_max_ekf_iterati,
  boolean_T VIOParameters_fixed_feature, boolean_T
  c_VIOParameters_delayed_initial, boolean_T VIOParameters_mono, boolean_T
  VIOParameters_full_stereo, boolean_T VIOParameters_RANSAC);
static double q_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void quatmultJ(const float q[4], const float p[4], float qp[4]);
static int rankFromQR(const emxArray_real32_T *A);
static void rdivide(const float x[3], float y, float z[3]);
static void sort(float x[8], int idx[8]);
static void sortIdx(emxArray_real32_T *x, emxArray_int32_T *idx);
static void svd(const float A[36], float U[6]);
static void undistortPoint(const float pt_d_data[], const int pt_d_size[1], int
  cameraparams_ATAN, const float cameraparams_FocalLength[2], const float
  cameraparams_PrincipalPoint[2], const float cameraparams_RadialDistortion[3],
  int cameraparams_DistortionModel, float pt_u_data[], int pt_u_size[1]);
static void xaxpy(int n, float a, int ix0, float y[36], int iy0);
static float xdotc(int n, const float x[36], int ix0, const float y[36], int iy0);
static void xgeqp3(emxArray_real32_T *A, emxArray_real32_T *tau,
                   emxArray_int32_T *jpvt);
static void xgetrf(int b_m, int n, emxArray_real32_T *A, int lda,
                   emxArray_int32_T *ipiv, int *info);
static float xnrm2(const float x[3]);
static void xrotg(float *a, float *b, float *c, float *s);
static void xscal(int n, float a, emxArray_real32_T *x, int ix0);

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : g_struct_T *b_xt
//                float b_P[10404]
//                const float z_u_l[96]
//                const float z_u_r[96]
//                const float c_stereoParams_CameraParameters[2]
//                const float d_stereoParams_CameraParameters[2]
//                const float e_stereoParams_CameraParameters[2]
//                const float f_stereoParams_CameraParameters[2]
//                const float stereoParams_r_lr[3]
//                const float stereoParams_R_rl[9]
//                float noiseParameters_image_noise
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_full_stereo
//                boolean_T VIOParameters_RANSAC
//                int updateVect[48]
// Return Type  : void
//
static void OnePointRANSAC_EKF(g_struct_T *b_xt, float b_P[10404], const float
  z_u_l[96], const float z_u_r[96], const float c_stereoParams_CameraParameters
  [2], const float d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], float noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_full_stereo, boolean_T VIOParameters_RANSAC, int
  updateVect[48])
{
  int residualDim;
  int mahalanobis_thresh;
  float LI_residual_thresh;
  int ic;
  boolean_T activeFeatures[48];
  boolean_T delayedFeatures[48];
  int anchorIdx;
  int featureIdx;
  boolean_T LI_inlier_status[48];
  emxArray_real32_T *S;
  emxArray_real32_T *K;
  emxArray_int32_T *ind;
  emxArray_real32_T *r;
  emxArray_real32_T *H;
  emxArray_real32_T *out;
  emxArray_real32_T *c;
  emxArray_real32_T *Y;
  emxArray_real32_T *A;
  emxArray_real32_T *tau;
  emxArray_int32_T *jpvt;
  emxArray_real32_T *B;
  int idx;
  signed char ii_data[48];
  int mn;
  boolean_T exitg2;
  boolean_T guard2 = false;
  int loop_ub;
  int i35;
  signed char i_data[48];
  signed char hyp_ind_data[48];
  float num_hyp;
  int hyp_it;
  emxArray_real32_T *b_c;
  emxArray_real32_T *c_c;
  boolean_T HI_inlierCandidates[48];
  boolean_T HI_inlierStatus[48];
  int qY;
  int ndbl;
  int d_c[6];
  int b_qY;
  int i36;
  float H_b_data[24];
  float H_c_data[4];
  float P_b[36];
  int e_c[6];
  int c_qY;
  int f_c;
  float P_d[36];
  int g_c[6];
  int d_qY;
  int h_c[6];
  int e_qY;
  int f_qY;
  int g_qY;
  int h_qY;
  int i_qY;
  float P_f[6];
  int br;
  float c_data[24];
  int b_m;
  int ar;
  int ib;
  int ia;
  float H_a_data[24];
  float b_c_data[24];
  float b_data[24];
  float S_feature_data[16];
  float c_c_data[24];
  float d_c_data[24];
  float e_c_data[16];
  float f_c_data[16];
  float B_data[4];
  float vec_data[4];
  static const signed char iv12[6] = { 1, 2, 3, 4, 5, 6 };

  float wj;
  float c_xt[4];
  float g_c_data[16];
  float h_c_data[16];
  unsigned int unnamed_idx_1;
  int n;
  int k;
  __attribute__((aligned(16))) float x_it[102];
  float b_x_it;
  float theta;
  float dq[4];
  float R_cw[9];
  __attribute__((aligned(16))) float r_wc[3];
  __attribute__((aligned(16))) float anchorPos[3];
  __attribute__((aligned(16))) float fv28[3];
  float fp[3];
  float gryro_bias_cov[9];
  float rho;
  __attribute__((aligned(16))) float fv29[3];
  __attribute__((aligned(16))) float h_u_r[2];
  __attribute__((aligned(16))) float h_c_n_l[2];
  __attribute__((aligned(16))) float z_l[2];
  __attribute__((aligned(16))) float z_r[2];
  __attribute__((aligned(16))) float fv30[2];
  __attribute__((aligned(16))) float fv31[2];
  float innov;
  __attribute__((aligned(16))) float fv32[2];
  emxArray_real32_T *i_c;
  unsigned int H_idx_0;
  float size_S;
  emxArray_real32_T *b_S;
  __attribute__((aligned(16))) float x_it2[3];
  __attribute__((aligned(16))) float fv33[3];
  __attribute__((aligned(16))) static float j_c[10404];
  __attribute__((aligned(16))) static float fv34[10404];
  static const signed char iv13[10404] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  __attribute__((aligned(16))) static float fv35[10404];
  static float fv36[10404];
  boolean_T b_HI_inlierCandidates;
  boolean_T exitg1;
  boolean_T guard1 = false;
  emxArray_real32_T *b_K;
  emxArray_real32_T *b_H;
  g_struct_T xt_it;
  int it;
  emxArray_real32_T *k_c;
  emxArray_real32_T *l_c;
  emxArray_real32_T *m_c;
  emxArray_real32_T *c_H;
  emxArray_real32_T *c_S;
  emxArray_real32_T *d_H;
  int b_ndbl;
  float i_c_data[4];
  int c_ndbl;
  float j_c_data[4];
  int d_ndbl;
  float k_c_data[24];
  int tmp_data[4];
  float l_c_data[24];
  float m_c_data[16];
  float n_c_data[24];
  float o_c_data[24];
  float p_c_data[16];
  float q_c_data[4];
  float r_c_data[4];
  float b_S_feature_data[6];
  int b_tmp_data[4];
  float x_it2_data[102];
  float b_x_it2_data;
  float s_c_data[10404];
  float fv37[10404];
  emxArray_real32_T *n_c;
  emxArray_real32_T *d_S;
  emxArray_real32_T *o_c;
  float fv38[10404];
  float rejected_ratio;
  char cv48[66];
  static const char cv49[66] = { '1', '-', 'p', 'o', 'i', 'n', 't', ' ', 'R',
    'A', 'N', 'S', 'A', 'C', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'e', 'd', ' ',
    '%', 'd', '%', '%', ' ', 'o', 'f', ' ', 'a', 'l', 'l', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', 's', '!', ' ', 'R', 'e', 's', 'e', 't', 't', 'i', 'n',
    'g', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '.', '\x00' };

  float d_xt[9];
  float e_xt[9];
  float acc_bias_cov[9];
  float R_ci_cov[9];
  static const signed char iv14[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  // 'OnePointRANSAC_EKF:5' numStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'OnePointRANSAC_EKF:8' LI_min_support_thresh = 3;
  // 'OnePointRANSAC_EKF:10' if VIOParameters.full_stereo
  if (VIOParameters_full_stereo) {
    // 'OnePointRANSAC_EKF:11' residualDim = 4;
    residualDim = 4;

    // 'OnePointRANSAC_EKF:12' mahalanobis_thresh = 9;
    mahalanobis_thresh = 9;

    //  HI mahalanobis gate
    // 'OnePointRANSAC_EKF:13' LI_residual_thresh = 4.24;
    LI_residual_thresh = 4.24F;
  } else {
    // 'OnePointRANSAC_EKF:14' else
    // 'OnePointRANSAC_EKF:15' residualDim = 2;
    residualDim = 2;

    // 'OnePointRANSAC_EKF:16' mahalanobis_thresh = 6;
    mahalanobis_thresh = 6;

    //  HI mahalanobis gate
    // 'OnePointRANSAC_EKF:17' LI_residual_thresh = 2;
    LI_residual_thresh = 2.0F;
  }

  // 'OnePointRANSAC_EKF:20' activeFeatures = false(size(updateVect));
  // 'OnePointRANSAC_EKF:21' delayedFeatures = activeFeatures;
  for (ic = 0; ic < 48; ic++) {
    activeFeatures[ic] = false;
    delayedFeatures[ic] = false;
  }

  // 'OnePointRANSAC_EKF:22' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'OnePointRANSAC_EKF:23' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'OnePointRANSAC_EKF:24' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status == 1)
      {
        // 'OnePointRANSAC_EKF:25' activeFeatures(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
        activeFeatures[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
          .status_idx - 1] = true;
      } else {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          // 'OnePointRANSAC_EKF:26' elseif xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          // 'OnePointRANSAC_EKF:27' delayedFeatures(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
          delayedFeatures[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = true;
        }
      }
    }
  }

  // 'OnePointRANSAC_EKF:32' activeFeatures = activeFeatures & (updateVect==1);
  // 'OnePointRANSAC_EKF:33' delayedFeatures = delayedFeatures & (updateVect==1); 
  // 'OnePointRANSAC_EKF:35' LI_inlier_status = false(size(updateVect));
  for (ic = 0; ic < 48; ic++) {
    LI_inlier_status[ic] = false;
    activeFeatures[ic] = (activeFeatures[ic] && (updateVect[ic] == 1));
    delayedFeatures[ic] = (delayedFeatures[ic] && (updateVect[ic] == 1));
  }

  // % B 1-point hypotheses generation and evaluation
  // 'OnePointRANSAC_EKF:38' if VIOParameters.RANSAC
  emxInit_real32_T1(&S, 2);
  emxInit_real32_T1(&K, 2);
  emxInit_int32_T1(&ind, 2);
  emxInit_real32_T(&r, 1);
  emxInit_real32_T1(&H, 2);
  emxInit_real32_T(&out, 1);
  emxInit_real32_T1(&c, 2);
  emxInit_real32_T1(&Y, 2);
  emxInit_real32_T1(&A, 2);
  emxInit_real32_T(&tau, 1);
  emxInit_int32_T1(&jpvt, 2);
  emxInit_real32_T1(&B, 2);
  if (VIOParameters_RANSAC) {
    //  build the map according to the current estimate
    // 'OnePointRANSAC_EKF:40' xt = getScaledMap(xt);
    getScaledMap(b_xt);

    //  randomly permute the active feature indices for 1-point RANSAC
    // 'OnePointRANSAC_EKF:42' hyp_ind = find(activeFeatures);
    idx = 0;
    mn = 1;
    exitg2 = false;
    while ((!exitg2) && (mn < 49)) {
      guard2 = false;
      if (activeFeatures[mn - 1]) {
        idx++;
        ii_data[idx - 1] = (signed char)mn;
        if (idx >= 48) {
          exitg2 = true;
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        mn++;
      }
    }

    if (1 > idx) {
      loop_ub = 0;
    } else {
      loop_ub = idx;
    }

    if (1 > idx) {
      mn = 0;
    } else {
      mn = idx;
    }

    for (i35 = 0; i35 < loop_ub; i35++) {
      i_data[i35] = ii_data[i35];
    }

    i35 = out->size[0];
    out->size[0] = mn;
    emxEnsureCapacity((emxArray__common *)out, i35, (int)sizeof(float));
    for (i35 = 0; i35 < mn; i35++) {
      out->data[i35] = i_data[i35];
    }

    for (i35 = 0; i35 < mn; i35++) {
      hyp_ind_data[i35] = i_data[i35];
    }

    //  hyp_ind = hyp_ind(randperm(length(hyp_ind)));
    // 'OnePointRANSAC_EKF:45' hyp_status = LI_inlier_status;
    // 'OnePointRANSAC_EKF:46' LI_inlier_status_i = LI_inlier_status;
    // 'OnePointRANSAC_EKF:48' num_hyp = length(hyp_ind);
    num_hyp = (float)out->size[0];

    // 'OnePointRANSAC_EKF:49' hyp_it = 1;
    hyp_it = 1;

    // 'OnePointRANSAC_EKF:50' while hyp_it < num_hyp && hyp_it < length(hyp_ind) 
    emxInit_real32_T1(&b_c, 2);
    emxInit_real32_T1(&c_c, 2);
    while ((hyp_it < num_hyp) && (hyp_it < loop_ub)) {
      // 'OnePointRANSAC_EKF:51' hyp_idx = hyp_ind(hyp_it);
      // 'OnePointRANSAC_EKF:53' LI_inlier_status_i(:) = false;
      // 'OnePointRANSAC_EKF:55' hyp_status(:) = false;
      for (ic = 0; ic < 48; ic++) {
        HI_inlierCandidates[ic] = false;
        HI_inlierStatus[ic] = false;
      }

      // 'OnePointRANSAC_EKF:56' hyp_status(hyp_idx) = true;
      HI_inlierStatus[hyp_ind_data[hyp_it - 1] - 1] = true;

      //  used to signal which feature to compute the derivatives for
      // 'OnePointRANSAC_EKF:57' [r, H, ind] = getH_R_res(xt, z_u_l, z_u_r, hyp_status, stereoParams, VIOParameters); 
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, z_u_r, HI_inlierStatus,
                 c_stereoParams_CameraParameters,
                 d_stereoParams_CameraParameters,
                 e_stereoParams_CameraParameters,
                 f_stereoParams_CameraParameters, stereoParams_r_lr,
                 stereoParams_R_rl, VIOParameters_full_stereo, r, H, ind);

      // 'OnePointRANSAC_EKF:59' P_a = P(1:6, 1:6);
      // 'OnePointRANSAC_EKF:61' anchorIdx = ind(1, 1);
      // 'OnePointRANSAC_EKF:62' featureIdx = ind(1, 2);
      featureIdx = ind->data[ind->size[0]];

      // 'OnePointRANSAC_EKF:63' H_a = H(1:residualDim, 1:6);
      // 'OnePointRANSAC_EKF:64' H_b = H(1:residualDim, numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
      idx = ind->data[0];
      qY = idx - 1;
      if ((idx < 0) && (qY >= 0)) {
        qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(qY, 14);
      qY = 18 + ndbl;
      if ((ndbl > 0) && (qY <= 0)) {
        qY = MAX_int32_T;
      }

      for (i35 = 0; i35 < 6; i35++) {
        ndbl = 1 + i35;
        b_qY = qY + ndbl;
        if ((qY > 0) && (b_qY <= 0)) {
          b_qY = MAX_int32_T;
        }

        d_c[i35] = b_qY;
      }

      for (i35 = 0; i35 < 6; i35++) {
        for (i36 = 0; i36 < residualDim; i36++) {
          ndbl = 1 + i35;
          b_qY = qY + ndbl;
          if ((qY > 0) && (b_qY <= 0)) {
            b_qY = MAX_int32_T;
          }

          H_b_data[i36 + residualDim * i35] = H->data[i36 + H->size[0] * (b_qY -
            1)];
        }
      }

      // 'OnePointRANSAC_EKF:65' H_c = H(1:residualDim, numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
      idx = ind->data[0];
      qY = idx - 1;
      if ((idx < 0) && (qY >= 0)) {
        qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(qY, 14);
      qY = 18 + ndbl;
      if ((ndbl > 0) && (qY <= 0)) {
        qY = MAX_int32_T;
      }

      b_qY = qY + 6;
      if ((qY > 0) && (b_qY <= 0)) {
        b_qY = MAX_int32_T;
      }

      ndbl = ind->data[ind->size[0]];
      qY = b_qY + ndbl;
      if ((b_qY < 0) && ((ndbl < 0) && (qY >= 0))) {
        qY = MIN_int32_T;
      } else {
        if ((b_qY > 0) && ((ndbl > 0) && (qY <= 0))) {
          qY = MAX_int32_T;
        }
      }

      for (i35 = 0; i35 < residualDim; i35++) {
        H_c_data[i35] = H->data[i35 + H->size[0] * (qY - 1)];
      }

      // 'OnePointRANSAC_EKF:66' P_b = P(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6), 1:6); 
      idx = ind->data[0];
      qY = idx - 1;
      if ((idx < 0) && (qY >= 0)) {
        qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(qY, 14);
      qY = 18 + ndbl;
      if ((ndbl > 0) && (qY <= 0)) {
        qY = MAX_int32_T;
      }

      for (i35 = 0; i35 < 6; i35++) {
        ndbl = 1 + i35;
        b_qY = qY + ndbl;
        if ((qY > 0) && (b_qY <= 0)) {
          b_qY = MAX_int32_T;
        }

        e_c[i35] = b_qY;
        for (i36 = 0; i36 < 6; i36++) {
          ndbl = 1 + i36;
          b_qY = qY + ndbl;
          if ((qY > 0) && (b_qY <= 0)) {
            b_qY = MAX_int32_T;
          }

          P_b[i36 + 6 * i35] = b_P[(b_qY + 102 * i35) - 1];
        }
      }

      // 'OnePointRANSAC_EKF:67' P_c = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, 1:6); 
      idx = ind->data[0];
      b_qY = idx - 1;
      if ((idx < 0) && (b_qY >= 0)) {
        b_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(b_qY, 14);
      b_qY = 18 + ndbl;
      if ((ndbl > 0) && (b_qY <= 0)) {
        b_qY = MAX_int32_T;
      }

      c_qY = b_qY + 6;
      if ((b_qY > 0) && (c_qY <= 0)) {
        c_qY = MAX_int32_T;
      }

      ndbl = ind->data[ind->size[0]];
      b_qY = c_qY + ndbl;
      if ((c_qY < 0) && ((ndbl < 0) && (b_qY >= 0))) {
        b_qY = MIN_int32_T;
      } else {
        if ((c_qY > 0) && ((ndbl > 0) && (b_qY <= 0))) {
          b_qY = MAX_int32_T;
        }
      }

      f_c = b_qY - 1;

      // 'OnePointRANSAC_EKF:68' P_d = P_b';
      for (i35 = 0; i35 < 6; i35++) {
        for (i36 = 0; i36 < 6; i36++) {
          P_d[i36 + 6 * i35] = P_b[i35 + 6 * i36];
        }
      }

      // 'OnePointRANSAC_EKF:69' P_e = P(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6), numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
      idx = ind->data[0];
      b_qY = idx - 1;
      if ((idx < 0) && (b_qY >= 0)) {
        b_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(b_qY, 14);
      b_qY = 18 + ndbl;
      if ((ndbl > 0) && (b_qY <= 0)) {
        b_qY = MAX_int32_T;
      }

      for (i35 = 0; i35 < 6; i35++) {
        ndbl = 1 + i35;
        d_qY = b_qY + ndbl;
        if ((b_qY > 0) && (d_qY <= 0)) {
          d_qY = MAX_int32_T;
        }

        g_c[i35] = d_qY;
      }

      idx = ind->data[0];
      d_qY = idx - 1;
      if ((idx < 0) && (d_qY >= 0)) {
        d_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(d_qY, 14);
      d_qY = 18 + ndbl;
      if ((ndbl > 0) && (d_qY <= 0)) {
        d_qY = MAX_int32_T;
      }

      for (i35 = 0; i35 < 6; i35++) {
        ndbl = 1 + i35;
        e_qY = d_qY + ndbl;
        if ((d_qY > 0) && (e_qY <= 0)) {
          e_qY = MAX_int32_T;
        }

        h_c[i35] = e_qY;
      }

      // 'OnePointRANSAC_EKF:70' P_f = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
      idx = ind->data[0];
      e_qY = idx - 1;
      if ((idx < 0) && (e_qY >= 0)) {
        e_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(e_qY, 14);
      e_qY = 18 + ndbl;
      if ((ndbl > 0) && (e_qY <= 0)) {
        e_qY = MAX_int32_T;
      }

      idx = ind->data[0];
      f_qY = idx - 1;
      if ((idx < 0) && (f_qY >= 0)) {
        f_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(f_qY, 14);
      f_qY = 18 + ndbl;
      if ((ndbl > 0) && (f_qY <= 0)) {
        f_qY = MAX_int32_T;
      }

      g_qY = f_qY + 6;
      if ((f_qY > 0) && (g_qY <= 0)) {
        g_qY = MAX_int32_T;
      }

      idx = ind->data[0];
      f_qY = idx - 1;
      if ((idx < 0) && (f_qY >= 0)) {
        f_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(f_qY, 14);
      f_qY = 18 + ndbl;
      if ((ndbl > 0) && (f_qY <= 0)) {
        f_qY = MAX_int32_T;
      }

      idx = ind->data[0];
      h_qY = idx - 1;
      if ((idx < 0) && (h_qY >= 0)) {
        h_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(h_qY, 14);
      h_qY = 18 + ndbl;
      if ((ndbl > 0) && (h_qY <= 0)) {
        h_qY = MAX_int32_T;
      }

      i_qY = h_qY + 6;
      if ((h_qY > 0) && (i_qY <= 0)) {
        i_qY = MAX_int32_T;
      }

      ndbl = ind->data[ind->size[0]];
      h_qY = i_qY + ndbl;
      if ((i_qY < 0) && ((ndbl < 0) && (h_qY >= 0))) {
        h_qY = MIN_int32_T;
      } else {
        if ((i_qY > 0) && ((ndbl > 0) && (h_qY <= 0))) {
          h_qY = MAX_int32_T;
        }
      }

      for (i35 = 0; i35 < 6; i35++) {
        ndbl = 1 + i35;
        i_qY = f_qY + ndbl;
        if ((f_qY > 0) && (i_qY <= 0)) {
          i_qY = MAX_int32_T;
        }

        P_f[i35] = b_P[(h_qY + 102 * (i_qY - 1)) - 1];
      }

      // 'OnePointRANSAC_EKF:71' P_g = P_c';
      // 'OnePointRANSAC_EKF:72' P_h = P_f';
      // 'OnePointRANSAC_EKF:73' P_i = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
      idx = ind->data[0];
      h_qY = idx - 1;
      if ((idx < 0) && (h_qY >= 0)) {
        h_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(h_qY, 14);
      h_qY = 18 + ndbl;
      if ((ndbl > 0) && (h_qY <= 0)) {
        h_qY = MAX_int32_T;
      }

      i_qY = h_qY + 6;
      if ((h_qY > 0) && (i_qY <= 0)) {
        i_qY = MAX_int32_T;
      }

      ndbl = ind->data[ind->size[0]];
      h_qY = i_qY + ndbl;
      if ((i_qY < 0) && ((ndbl < 0) && (h_qY >= 0))) {
        h_qY = MIN_int32_T;
      } else {
        if ((i_qY > 0) && ((ndbl > 0) && (h_qY <= 0))) {
          h_qY = MAX_int32_T;
        }
      }

      idx = ind->data[0];
      i_qY = idx - 1;
      if ((idx < 0) && (i_qY >= 0)) {
        i_qY = MIN_int32_T;
      }

      ndbl = mul_s32_s32_s32_sat(i_qY, 14);
      i_qY = 18 + ndbl;
      if ((ndbl > 0) && (i_qY <= 0)) {
        i_qY = MAX_int32_T;
      }

      idx = i_qY + 6;
      if ((i_qY > 0) && (idx <= 0)) {
        idx = MAX_int32_T;
      }

      ndbl = ind->data[ind->size[0]];
      i_qY = idx + ndbl;
      if ((idx < 0) && ((ndbl < 0) && (i_qY >= 0))) {
        i_qY = MIN_int32_T;
      } else {
        if ((idx > 0) && ((ndbl > 0) && (i_qY <= 0))) {
          i_qY = MAX_int32_T;
        }
      }

      // 'OnePointRANSAC_EKF:74' S = (H_a*P_a + H_b*P_b + H_c*P_c)*H_a' + ...
      // 'OnePointRANSAC_EKF:75'             (H_a*P_d + H_b*P_e + H_c*P_f)*H_b' + ... 
      // 'OnePointRANSAC_EKF:76'             (H_a*P_g + H_b*P_h + H_c*P_i)*H_c'; 
      for (i35 = 0; i35 < 6; i35++) {
        br = residualDim;
        for (i36 = 0; i36 < br; i36++) {
          c_data[i36 + residualDim * i35] = 0.0F;
        }
      }

      b_m = residualDim * 5;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        i35 = mn + residualDim;
        for (ic = mn; ic + 1 <= i35; ic++) {
          c_data[ic] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (b_P[ib % 6 + 102 * (ib / 6)] != 0.0F) {
            ia = ar;
            i35 = mn + residualDim;
            for (ic = mn; ic + 1 <= i35; ic++) {
              ia++;
              c_data[ic] += b_P[ib % 6 + 102 * (ib / 6)] * H->data[(ia - 1) %
                residualDim + H->size[0] * ((ia - 1) / residualDim)];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i35 = 0; i35 < 6; i35++) {
        br = residualDim;
        for (i36 = 0; i36 < br; i36++) {
          H_a_data[i36 + residualDim * i35] = 0.0F;
        }
      }

      b_m = residualDim * 5;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        i35 = mn + residualDim;
        for (ic = mn; ic + 1 <= i35; ic++) {
          H_a_data[ic] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (b_P[(e_c[ib % 6] + 102 * (ib / 6)) - 1] != 0.0F) {
            ia = ar;
            i35 = mn + residualDim;
            for (ic = mn; ic + 1 <= i35; ic++) {
              ia++;
              ndbl = 1 + ib % 6;
              idx = qY + ndbl;
              if ((qY < 0) && ((ndbl < 0) && (idx >= 0))) {
                idx = MIN_int32_T;
              } else {
                if ((qY > 0) && ((ndbl > 0) && (idx <= 0))) {
                  idx = MAX_int32_T;
                }
              }

              H_a_data[ic] += b_P[(idx + 102 * (ib / 6)) - 1] * H->data[(ia - 1)
                % residualDim + H->size[0] * (d_c[(ia - 1) / residualDim] - 1)];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i35 = 0; i35 < residualDim; i35++) {
        for (i36 = 0; i36 < 6; i36++) {
          b_c_data[i35 + residualDim * i36] = (c_data[i35 + residualDim * i36] +
            H_a_data[i35 + residualDim * i36]) + H_c_data[i35] * b_P[f_c + 102 *
            i36];
        }
      }

      for (i35 = 0; i35 < residualDim; i35++) {
        for (i36 = 0; i36 < 6; i36++) {
          b_data[i36 + 6 * i35] = H->data[i35 + H->size[0] * i36];
        }
      }

      br = residualDim;
      for (i35 = 0; i35 < br; i35++) {
        for (i36 = 0; i36 < br; i36++) {
          S_feature_data[i36 + residualDim * i35] = 0.0F;
        }
      }

      b_m = residualDim * (residualDim - 1);
      for (mn = 0; mn <= b_m; mn += residualDim) {
        i35 = mn + residualDim;
        for (ic = mn; ic + 1 <= i35; ic++) {
          S_feature_data[ic] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (b_data[ib] != 0.0F) {
            ia = ar;
            i35 = mn + residualDim;
            for (ic = mn; ic + 1 <= i35; ic++) {
              ia++;
              S_feature_data[ic] += b_data[ib] * b_c_data[ia - 1];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i35 = 0; i35 < 6; i35++) {
        br = residualDim;
        for (i36 = 0; i36 < br; i36++) {
          c_c_data[i36 + residualDim * i35] = 0.0F;
        }
      }

      b_m = residualDim * 5;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        i35 = mn + residualDim;
        for (ic = mn; ic + 1 <= i35; ic++) {
          c_c_data[ic] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (P_d[ib] != 0.0F) {
            ia = ar;
            i35 = mn + residualDim;
            for (ic = mn; ic + 1 <= i35; ic++) {
              ia++;
              c_c_data[ic] += P_d[ib] * H->data[(ia - 1) % residualDim + H->
                size[0] * ((ia - 1) / residualDim)];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i35 = 0; i35 < 6; i35++) {
        br = residualDim;
        for (i36 = 0; i36 < br; i36++) {
          d_c_data[i36 + residualDim * i35] = 0.0F;
        }
      }

      b_m = residualDim * 5;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        i35 = mn + residualDim;
        for (ic = mn; ic + 1 <= i35; ic++) {
          d_c_data[ic] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (b_P[(g_c[ib % 6] + 102 * (h_c[ib / 6] - 1)) - 1] != 0.0F) {
            ia = ar;
            i35 = mn + residualDim;
            for (ic = mn; ic + 1 <= i35; ic++) {
              ia++;
              ndbl = 1 + ib % 6;
              qY = b_qY + ndbl;
              if ((b_qY < 0) && ((ndbl < 0) && (qY >= 0))) {
                qY = MIN_int32_T;
              } else {
                if ((b_qY > 0) && ((ndbl > 0) && (qY <= 0))) {
                  qY = MAX_int32_T;
                }
              }

              ndbl = 1 + ib / 6;
              idx = d_qY + ndbl;
              if ((d_qY < 0) && ((ndbl < 0) && (idx >= 0))) {
                idx = MIN_int32_T;
              } else {
                if ((d_qY > 0) && ((ndbl > 0) && (idx <= 0))) {
                  idx = MAX_int32_T;
                }
              }

              d_c_data[ic] += b_P[(qY + 102 * (idx - 1)) - 1] * H->data[(ia - 1)
                % residualDim + H->size[0] * (d_c[(ia - 1) / residualDim] - 1)];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i35 = 0; i35 < residualDim; i35++) {
        for (i36 = 0; i36 < 6; i36++) {
          b_c_data[i35 + residualDim * i36] = (c_c_data[i35 + residualDim * i36]
            + d_c_data[i35 + residualDim * i36]) + H_c_data[i35] * P_f[i36];
        }
      }

      for (i35 = 0; i35 < residualDim; i35++) {
        for (i36 = 0; i36 < 6; i36++) {
          b_data[i36 + 6 * i35] = H_b_data[i35 + residualDim * i36];
        }
      }

      br = residualDim;
      for (i35 = 0; i35 < br; i35++) {
        for (i36 = 0; i36 < br; i36++) {
          e_c_data[i36 + residualDim * i35] = 0.0F;
        }
      }

      b_m = residualDim * (residualDim - 1);
      for (mn = 0; mn <= b_m; mn += residualDim) {
        i35 = mn + residualDim;
        for (ic = mn; ic + 1 <= i35; ic++) {
          e_c_data[ic] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= b_m; mn += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (b_data[ib] != 0.0F) {
            ia = ar;
            i35 = mn + residualDim;
            for (ic = mn; ic + 1 <= i35; ic++) {
              ia++;
              e_c_data[ic] += b_data[ib] * b_c_data[ia - 1];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      br = residualDim * residualDim;
      for (i35 = 0; i35 < br; i35++) {
        f_c_data[i35] = S_feature_data[i35] + e_c_data[i35];
      }

      idx = residualDim;
      for (i35 = 0; i35 < idx; i35++) {
        B_data[i35] = 0.0F;
      }

      mn = 0;
      while (mn <= 0) {
        for (ic = 1; ic <= residualDim; ic++) {
          B_data[ic - 1] = 0.0F;
        }

        mn = residualDim;
      }

      br = 6;
      mn = 0;
      while (mn <= 0) {
        ar = 0;
        for (ib = br - 5; ib <= br; ib++) {
          ndbl = featureIdx;
          qY = c_qY + ndbl;
          if ((c_qY < 0) && ((ndbl < 0) && (qY >= 0))) {
            qY = MIN_int32_T;
          } else {
            if ((c_qY > 0) && ((ndbl > 0) && (qY <= 0))) {
              qY = MAX_int32_T;
            }
          }

          if (b_P[(qY + 102 * (ib - 1)) - 1] != 0.0F) {
            ia = ar;
            for (ic = 0; ic + 1 <= residualDim; ic++) {
              ia++;
              B_data[ic] += b_P[f_c + 102 * (ib - 1)] * H->data[(ia - 1) %
                residualDim + H->size[0] * ((ia - 1) / residualDim)];
            }
          }

          ar += residualDim;
        }

        br += 6;
        mn = residualDim;
      }

      f_c = residualDim;
      for (i35 = 0; i35 < f_c; i35++) {
        vec_data[i35] = 0.0F;
      }

      mn = 0;
      while (mn <= 0) {
        for (ic = 1; ic <= residualDim; ic++) {
          vec_data[ic - 1] = 0.0F;
        }

        mn = residualDim;
      }

      br = 6;
      mn = 0;
      while (mn <= 0) {
        ar = 0;
        for (ib = br - 5; ib <= br; ib++) {
          ndbl = featureIdx;
          qY = g_qY + ndbl;
          if ((g_qY < 0) && ((ndbl < 0) && (qY >= 0))) {
            qY = MIN_int32_T;
          } else {
            if ((g_qY > 0) && ((ndbl > 0) && (qY <= 0))) {
              qY = MAX_int32_T;
            }
          }

          ndbl = iv12[ib - 1];
          b_qY = f_qY + ndbl;
          if ((f_qY > 0) && (b_qY <= 0)) {
            b_qY = MAX_int32_T;
          }

          if (b_P[(qY + 102 * (b_qY - 1)) - 1] != 0.0F) {
            ia = ar;
            for (ic = 0; ic + 1 <= residualDim; ic++) {
              ia++;
              qY = e_qY + 6;
              if ((e_qY > 0) && (qY <= 0)) {
                qY = MAX_int32_T;
              }

              ndbl = ind->data[ind->size[0]];
              b_qY = qY + ndbl;
              if ((qY < 0) && ((ndbl < 0) && (b_qY >= 0))) {
                b_qY = MIN_int32_T;
              } else {
                if ((qY > 0) && ((ndbl > 0) && (b_qY <= 0))) {
                  b_qY = MAX_int32_T;
                }
              }

              qY = f_qY + ib;
              if ((f_qY < 0) && ((ib < 0) && (qY >= 0))) {
                qY = MIN_int32_T;
              } else {
                if ((f_qY > 0) && ((ib > 0) && (qY <= 0))) {
                  qY = MAX_int32_T;
                }
              }

              vec_data[ic] += b_P[(b_qY + 102 * (qY - 1)) - 1] * H->data[(ia - 1)
                % residualDim + H->size[0] * (d_c[(ia - 1) / residualDim] - 1)];
            }
          }

          ar += residualDim;
        }

        br += 6;
        mn = residualDim;
      }

      wj = b_P[(h_qY + 102 * (i_qY - 1)) - 1];
      br = residualDim;
      for (i35 = 0; i35 < br; i35++) {
        c_xt[i35] = (B_data[i35] + vec_data[i35]) + H_c_data[i35] * wj;
      }

      for (i35 = 0; i35 < br; i35++) {
        for (i36 = 0; i36 < residualDim; i36++) {
          g_c_data[i35 + residualDim * i36] = c_xt[i35] * H_c_data[i36];
        }
      }

      br = residualDim * residualDim;
      for (i35 = 0; i35 < br; i35++) {
        h_c_data[i35] = f_c_data[i35] + g_c_data[i35];
      }

      i35 = S->size[0] * S->size[1];
      S->size[0] = residualDim;
      S->size[1] = residualDim;
      emxEnsureCapacity((emxArray__common *)S, i35, (int)sizeof(float));
      br = residualDim * residualDim;
      for (i35 = 0; i35 < br; i35++) {
        S->data[i35] = f_c_data[i35] + g_c_data[i35];
      }

      // 'OnePointRANSAC_EKF:78' size_S = residualDim;
      // 'OnePointRANSAC_EKF:79' S(1:(size_S+1):size_S^2) = S(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
      i35 = c_c->size[0] * c_c->size[1];
      c_c->size[0] = residualDim;
      c_c->size[1] = residualDim;
      emxEnsureCapacity((emxArray__common *)c_c, i35, (int)sizeof(float));
      br = residualDim;
      for (i35 = 0; i35 < br; i35++) {
        for (i36 = 0; i36 < br; i36++) {
          c_c->data[i36 + c_c->size[0] * i35] = h_c_data[i36 + residualDim * i35];
        }
      }

      i35 = residualDim + 1;
      br = div_s32_floor(residualDim * residualDim - 1, i35);
      for (i36 = 0; i36 <= br; i36++) {
        S->data[(residualDim + 1) * i36] = c_c->data[i35 * i36] +
          noiseParameters_image_noise;
      }

      //  add R to HPH'
      // 'OnePointRANSAC_EKF:81' if coder.target('MATLAB')
      // 'OnePointRANSAC_EKF:87' K = (P*H')/S;
      i35 = K->size[0] * K->size[1];
      K->size[0] = 102;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
      br = H->size[0];
      for (i35 = 0; i35 < br; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          K->data[i36 + K->size[0] * i35] = H->data[i35 + H->size[0] * i36];
        }
      }

      unnamed_idx_1 = (unsigned int)K->size[1];
      i35 = b_c->size[0] * b_c->size[1];
      b_c->size[0] = 102;
      b_c->size[1] = (int)unnamed_idx_1;
      b_c->size[0] = 102;
      emxEnsureCapacity((emxArray__common *)b_c, i35, (int)sizeof(float));
      br = b_c->size[1];
      for (i35 = 0; i35 < br; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          b_c->data[i36 + b_c->size[0] * i35] = 0.0F;
        }
      }

      if (K->size[1] == 0) {
      } else {
        f_c = 102 * (K->size[1] - 1);
        for (mn = 0; mn <= f_c; mn += 102) {
          for (ic = mn + 1; ic <= mn + 102; ic++) {
            b_c->data[ic - 1] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= f_c; mn += 102) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 102; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              for (ic = mn; ic + 1 <= mn + 102; ic++) {
                ia++;
                b_c->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 102;
          }

          br += 102;
        }
      }

      if (b_c->size[1] == 0) {
        unnamed_idx_1 = (unsigned int)S->size[0];
        i35 = K->size[0] * K->size[1];
        K->size[0] = 102;
        K->size[1] = (int)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
        br = 102 * (int)unnamed_idx_1;
        for (i35 = 0; i35 < br; i35++) {
          K->data[i35] = 0.0F;
        }
      } else if (S->size[0] == S->size[1]) {
        n = S->size[1];
        i35 = A->size[0] * A->size[1];
        A->size[0] = S->size[0];
        A->size[1] = S->size[1];
        emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
        br = S->size[0] * S->size[1];
        for (i35 = 0; i35 < br; i35++) {
          A->data[i35] = S->data[i35];
        }

        xgetrf(S->size[1], S->size[1], A, S->size[1], jpvt, &idx);
        i35 = K->size[0] * K->size[1];
        K->size[0] = 102;
        K->size[1] = b_c->size[1];
        emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
        br = b_c->size[0] * b_c->size[1];
        for (i35 = 0; i35 < br; i35++) {
          K->data[i35] = b_c->data[i35];
        }

        for (ib = 0; ib + 1 <= n; ib++) {
          idx = 102 * ib;
          br = n * ib;
          for (k = 1; k <= ib; k++) {
            mn = 102 * (k - 1);
            if (A->data[(k + br) - 1] != 0.0F) {
              for (ic = 0; ic < 102; ic++) {
                K->data[ic + idx] -= A->data[(k + br) - 1] * K->data[ic + mn];
              }
            }
          }

          wj = 1.0F / A->data[ib + br];
          for (ic = 0; ic < 102; ic++) {
            K->data[ic + idx] *= wj;
          }
        }

        for (ib = S->size[1]; ib > 0; ib--) {
          idx = 102 * (ib - 1);
          br = n * (ib - 1) - 1;
          for (k = ib + 1; k <= n; k++) {
            mn = 102 * (k - 1);
            if (A->data[k + br] != 0.0F) {
              for (ic = 0; ic < 102; ic++) {
                K->data[ic + idx] -= A->data[k + br] * K->data[ic + mn];
              }
            }
          }
        }

        for (ar = S->size[1] - 2; ar + 1 > 0; ar--) {
          if (jpvt->data[ar] != ar + 1) {
            mn = jpvt->data[ar] - 1;
            for (idx = 0; idx < 102; idx++) {
              wj = K->data[idx + K->size[0] * ar];
              K->data[idx + K->size[0] * ar] = K->data[idx + K->size[0] * mn];
              K->data[idx + K->size[0] * mn] = wj;
            }
          }
        }
      } else {
        i35 = A->size[0] * A->size[1];
        A->size[0] = S->size[1];
        A->size[1] = S->size[0];
        emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
        br = S->size[0];
        for (i35 = 0; i35 < br; i35++) {
          mn = S->size[1];
          for (i36 = 0; i36 < mn; i36++) {
            A->data[i36 + A->size[0] * i35] = S->data[i35 + S->size[0] * i36];
          }
        }

        xgeqp3(A, tau, jpvt);
        ar = rankFromQR(A);
        ia = A->size[1];
        i35 = Y->size[0] * Y->size[1];
        Y->size[0] = ia;
        Y->size[1] = 102;
        emxEnsureCapacity((emxArray__common *)Y, i35, (int)sizeof(float));
        br = ia * 102;
        for (i35 = 0; i35 < br; i35++) {
          Y->data[i35] = 0.0F;
        }

        i35 = B->size[0] * B->size[1];
        B->size[0] = b_c->size[1];
        B->size[1] = 102;
        emxEnsureCapacity((emxArray__common *)B, i35, (int)sizeof(float));
        for (i35 = 0; i35 < 102; i35++) {
          br = b_c->size[1];
          for (i36 = 0; i36 < br; i36++) {
            B->data[i36 + B->size[0] * i35] = b_c->data[i35 + b_c->size[0] * i36];
          }
        }

        b_m = A->size[0];
        idx = A->size[0];
        mn = A->size[1];
        if (idx <= mn) {
          mn = idx;
        }

        for (ib = 0; ib + 1 <= mn; ib++) {
          if (tau->data[ib] != 0.0F) {
            for (k = 0; k < 102; k++) {
              wj = B->data[ib + B->size[0] * k];
              for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                wj += A->data[ic + A->size[0] * ib] * B->data[ic + B->size[0] *
                  k];
              }

              wj *= tau->data[ib];
              if (wj != 0.0F) {
                B->data[ib + B->size[0] * k] -= wj;
                for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                  B->data[ic + B->size[0] * k] -= A->data[ic + A->size[0] * ib] *
                    wj;
                }
              }
            }
          }
        }

        for (k = 0; k < 102; k++) {
          for (ic = 0; ic + 1 <= ar; ic++) {
            Y->data[(jpvt->data[ic] + Y->size[0] * k) - 1] = B->data[ic +
              B->size[0] * k];
          }

          for (ib = ar - 1; ib + 1 > 0; ib--) {
            Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] /= A->data[ib +
              A->size[0] * ib];
            for (ic = 0; ic + 1 <= ib; ic++) {
              Y->data[(jpvt->data[ic] + Y->size[0] * k) - 1] -= Y->data
                [(jpvt->data[ib] + Y->size[0] * k) - 1] * A->data[ic + A->size[0]
                * ib];
            }
          }
        }

        i35 = K->size[0] * K->size[1];
        K->size[0] = 102;
        K->size[1] = Y->size[0];
        emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
        br = Y->size[0];
        for (i35 = 0; i35 < br; i35++) {
          for (i36 = 0; i36 < 102; i36++) {
            K->data[i36 + K->size[0] * i35] = Y->data[i35 + Y->size[0] * i36];
          }
        }
      }

      // 'OnePointRANSAC_EKF:89' x_it = K*r;
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i35 = 0; i35 < 102; i35++) {
          x_it[i35] = 0.0F;
          br = K->size[1];
          for (i36 = 0; i36 < br; i36++) {
            b_x_it = x_it[i35] + K->data[i35 + K->size[0] * i36] * r->data[i36];
            x_it[i35] = b_x_it;
          }
        }
      } else {
        memset(&x_it[0], 0, 102U * sizeof(float));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0F) {
            ia = ar;
            for (ic = 0; ic < 102; ic++) {
              ia++;
              b_x_it = x_it[ic] + r->data[ib] * K->data[ia - 1];
              x_it[ic] = b_x_it;
            }
          }

          ar += 102;
        }
      }

      // 'OnePointRANSAC_EKF:91' R_cw = RotFromQuatJ(quatmultJ(quatPlusThetaJ(x_it(4:6)), xt.robot_state.att)); 
      // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
      theta = norm(*(float (*)[3])&x_it[3]) * 0.5F;

      // 'quatPlusThetaJ:3' if theta < 0.244
      if (theta < 0.244F) {
        // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
        for (ic = 0; ic < 3; ic++) {
          dq[ic] = 0.5F * x_it[ic + 3];
        }

        dq[3] = 1.0F;
      } else {
        // 'quatPlusThetaJ:5' else
        // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
        // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
        // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
        // 'quatPlusThetaJ:9'          cos(theta)];
        dq[0] = 0.5F * x_it[3] * sinf(theta) / theta;
        dq[1] = 0.5F * x_it[4] * sinf(theta) / theta;
        dq[2] = 0.5F * x_it[5] * sinf(theta) / theta;
        dq[3] = cosf(theta);
      }

      // 'quatPlusThetaJ:11' dq = dq/norm(dq);
      wj = b_norm(dq);
      for (i35 = 0; i35 < 4; i35++) {
        dq[i35] /= wj;
      }

      quatmultJ(dq, b_xt->robot_state.att, c_xt);

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
      // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
      // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
      R_cw[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2]) +
        c_xt[3] * c_xt[3];
      R_cw[3] = 2.0F * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
      R_cw[6] = 2.0F * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
      R_cw[1] = 2.0F * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
      R_cw[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];
      R_cw[7] = 2.0F * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
      R_cw[2] = 2.0F * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
      R_cw[5] = 2.0F * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
      R_cw[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];

      // 'OnePointRANSAC_EKF:92' r_wc = xt.robot_state.pos + x_it(1:3);
      mw_neon_mm_add_f32x4(b_xt->robot_state.pos, 3, 1, *(float (*)[3])&x_it[0],
                           &r_wc[0]);

      // 'OnePointRANSAC_EKF:94' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
        // 'OnePointRANSAC_EKF:95' anchorPos = xt.anchor_states(anchorIdx).pos + x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        f_c = anchorIdx * 14 + 18;
        for (i35 = 0; i35 < 3; i35++) {
          anchorPos[i35] = x_it[i35 + f_c];
        }

        mw_neon_mm_add_f32x4(b_xt->anchor_states[anchorIdx].pos, 3, 1, anchorPos,
                             &fv28[0]);

        // 'OnePointRANSAC_EKF:96' anchorRot = RotFromQuatJ(quatmultJ(quatPlusThetaJ(x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att)); 
        f_c = 18 + anchorIdx * 14;
        for (ic = 0; ic < 3; ic++) {
          anchorPos[ic] = fv28[ic];
          fp[ic] = x_it[(ic + f_c) + 3];
        }

        // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
        theta = norm(fp) * 0.5F;

        // 'quatPlusThetaJ:3' if theta < 0.244
        if (theta < 0.244F) {
          // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
          for (ic = 0; ic < 3; ic++) {
            dq[ic] = 0.5F * fp[ic];
          }

          dq[3] = 1.0F;
        } else {
          // 'quatPlusThetaJ:5' else
          // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
          // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
          // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
          // 'quatPlusThetaJ:9'          cos(theta)];
          dq[0] = 0.5F * x_it[f_c + 3] * sinf(theta) / theta;
          dq[1] = 0.5F * x_it[f_c + 4] * sinf(theta) / theta;
          dq[2] = 0.5F * x_it[f_c + 5] * sinf(theta) / theta;
          dq[3] = cosf(theta);
        }

        // 'quatPlusThetaJ:11' dq = dq/norm(dq);
        wj = b_norm(dq);
        for (i35 = 0; i35 < 4; i35++) {
          dq[i35] /= wj;
        }

        quatmultJ(dq, b_xt->anchor_states[anchorIdx].att, c_xt);

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        gryro_bias_cov[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] *
                             c_xt[2]) + c_xt[3] * c_xt[3];
        gryro_bias_cov[3] = 2.0F * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
        gryro_bias_cov[6] = 2.0F * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
        gryro_bias_cov[1] = 2.0F * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
        gryro_bias_cov[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2]
                             * c_xt[2]) + c_xt[3] * c_xt[3];
        gryro_bias_cov[7] = 2.0F * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
        gryro_bias_cov[2] = 2.0F * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
        gryro_bias_cov[5] = 2.0F * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
        gryro_bias_cov[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2]
                             * c_xt[2]) + c_xt[3] * c_xt[3];

        // 'OnePointRANSAC_EKF:98' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'OnePointRANSAC_EKF:99' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:100' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            rho = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth + x_it[(anchorIdx * 14 + featureIdx) + 24];

            // 'OnePointRANSAC_EKF:101' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
            // 'OnePointRANSAC_EKF:102' fp = R_cw*(anchorPos + anchorRot'*m/rho - r_wc); 
            for (i35 = 0; i35 < 3; i35++) {
              wj = 0.0F;
              for (i36 = 0; i36 < 3; i36++) {
                wj += gryro_bias_cov[i36 + 3 * i35] * b_xt->
                  anchor_states[anchorIdx].feature_states[featureIdx].m[i36];
              }

              fv28[i35] = wj / rho;
            }

            mw_neon_mm_add_f32x4(anchorPos, 3, 1, fv28, &fv29[0]);
            mw_neon_mm_sub_f32x4(fv29, 3, 1, r_wc, &fv28[0]);
            for (i35 = 0; i35 < 3; i35++) {
              fp[i35] = 0.0F;
              for (i36 = 0; i36 < 3; i36++) {
                fp[i35] += R_cw[i35 + 3 * i36] * fv28[i36];
              }
            }

            // 'OnePointRANSAC_EKF:103' if VIOParameters.full_stereo
            if (VIOParameters_full_stereo) {
              // 'OnePointRANSAC_EKF:104' [ h_u_l, h_u_r ] = predictMeasurementStereo( fp, stereoParams ); 
              predictMeasurementStereo(fp, c_stereoParams_CameraParameters,
                d_stereoParams_CameraParameters, e_stereoParams_CameraParameters,
                f_stereoParams_CameraParameters, stereoParams_r_lr,
                stereoParams_R_rl, h_c_n_l, h_u_r);

              // 'OnePointRANSAC_EKF:105' z_l = z_u_l((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*2 + int32(1:2)); 
              idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx;
              qY = idx - 1;
              if ((idx < 0) && (qY >= 0)) {
                qY = MIN_int32_T;
              }

              if (qY > 1073741823) {
                f_c = MAX_int32_T;
              } else if (qY <= -1073741824) {
                f_c = MIN_int32_T;
              } else {
                f_c = qY << 1;
              }

              // 'OnePointRANSAC_EKF:106' z_r = z_u_r((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*2 + int32(1:2)); 
              idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx;
              qY = idx - 1;
              if ((idx < 0) && (qY >= 0)) {
                qY = MIN_int32_T;
              }

              if (qY > 1073741823) {
                b_m = MAX_int32_T;
              } else if (qY <= -1073741824) {
                b_m = MIN_int32_T;
              } else {
                b_m = qY << 1;
              }

              for (i35 = 0; i35 < 2; i35++) {
                ndbl = 1 + i35;
                qY = f_c + ndbl;
                if ((f_c > 0) && (qY <= 0)) {
                  qY = MAX_int32_T;
                }

                z_l[i35] = z_u_l[qY - 1];
                ndbl = 1 + i35;
                qY = b_m + ndbl;
                if ((b_m > 0) && (qY <= 0)) {
                  qY = MAX_int32_T;
                }

                z_r[i35] = z_u_r[qY - 1];
              }

              // 'OnePointRANSAC_EKF:107' innov = norm([h_u_l - z_l; h_u_r - z_r]); 
              mw_neon_mm_sub_f32x4(h_u_r, 2, 1, z_r, &fv30[0]);
              mw_neon_mm_sub_f32x4(h_c_n_l, 2, 1, z_l, &fv31[0]);
              for (ic = 0; ic < 2; ic++) {
                c_xt[ic] = fv31[ic];
                c_xt[ic + 2] = fv30[ic];
              }

              innov = b_norm(c_xt);
            } else {
              // 'OnePointRANSAC_EKF:108' else
              // 'OnePointRANSAC_EKF:109' h_u_l = predictMeasurementMono( fp, stereoParams.CameraParameters1 ); 
              // predictMeasurementLeft Predict the measurement of a feature given in the left 
              // camera frame
              //    Get the normalized pixel coordinates where a feature given in the left camera 
              //    frame
              // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
              // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
              // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1); 
              // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2); 
              // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
              //  normalized feature in camera frame
              // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
              // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
              // 'OnePointRANSAC_EKF:110' z_l = z_u_l((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*2 + int32(1:2)); 
              idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx;
              qY = idx - 1;
              if ((idx < 0) && (qY >= 0)) {
                qY = MIN_int32_T;
              }

              if (qY > 1073741823) {
                f_c = MAX_int32_T;
              } else if (qY <= -1073741824) {
                f_c = MIN_int32_T;
              } else {
                f_c = qY << 1;
              }

              for (ic = 0; ic < 2; ic++) {
                h_c_n_l[ic] = fp[ic] / fp[2];
                ndbl = 1 + ic;
                qY = f_c + ndbl;
                if ((f_c > 0) && (qY <= 0)) {
                  qY = MAX_int32_T;
                }

                z_l[ic] = z_u_l[qY - 1];
              }

              // 'OnePointRANSAC_EKF:111' innov = norm(h_u_l - z_l);
              fv32[0] = h_c_n_l[0] * c_stereoParams_CameraParameters[0] +
                d_stereoParams_CameraParameters[0];
              fv32[1] = h_c_n_l[1] * c_stereoParams_CameraParameters[1] +
                d_stereoParams_CameraParameters[1];
              mw_neon_mm_sub_f32x4(fv32, 2, 1, z_l, &h_c_n_l[0]);
              innov = c_norm(h_c_n_l);
            }

            // 'OnePointRANSAC_EKF:113' if innov < LI_residual_thresh
            if (innov < LI_residual_thresh) {
              // 'OnePointRANSAC_EKF:114' LI_inlier_status_i(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = true; 
              HI_inlierCandidates[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = true;
            }
          }
        }
      }

      // 'OnePointRANSAC_EKF:120' if nnz(LI_inlier_status_i) > nnz(LI_inlier_status) 
      n = 0;
      ar = 0;
      for (k = 0; k < 48; k++) {
        if (HI_inlierCandidates[k]) {
          n++;
        }

        if (LI_inlier_status[k]) {
          ar++;
        }
      }

      if ((float)n > ar) {
        // 'OnePointRANSAC_EKF:121' LI_inlier_status = LI_inlier_status_i;
        // 'OnePointRANSAC_EKF:122' epsilon = 1 - nnz(LI_inlier_status_i)/nnz(activeFeatures); 
        n = 0;
        ar = 0;
        for (k = 0; k < 48; k++) {
          LI_inlier_status[k] = HI_inlierCandidates[k];
          if (HI_inlierCandidates[k]) {
            n++;
          }

          if (activeFeatures[k]) {
            ar++;
          }
        }

        // 'OnePointRANSAC_EKF:123' assert(epsilon <= 1)
        // 'OnePointRANSAC_EKF:124' num_hyp = log(1-0.99)/log(epsilon);
        num_hyp = -4.6051712F / logf(1.0F - (float)n / (float)ar);
      }

      // 'OnePointRANSAC_EKF:127' hyp_it = hyp_it + 1;
      hyp_it++;
    }

    emxFree_real32_T(&c_c);
    emxFree_real32_T(&b_c);

    //  log_info('Found %i LI inliers in %i active features', nnz(LI_inlier_status), nnz(activeFeatures)) 
    // 'OnePointRANSAC_EKF:130' if nnz(LI_inlier_status) > LI_min_support_thresh 
    n = 0;
    for (k = 0; k < 48; k++) {
      if (LI_inlier_status[k]) {
        n++;
      }
    }

    if (n > 3.0F) {
      emxInit_real32_T1(&i_c, 2);

      // 'OnePointRANSAC_EKF:131' [r, H] = getH_R_res(xt, z_u_l, z_u_r, LI_inlier_status, stereoParams, VIOParameters); 
      b_getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                   b_xt->anchor_states, z_u_l, z_u_r, LI_inlier_status,
                   c_stereoParams_CameraParameters,
                   d_stereoParams_CameraParameters,
                   e_stereoParams_CameraParameters,
                   f_stereoParams_CameraParameters, stereoParams_r_lr,
                   stereoParams_R_rl, VIOParameters_full_stereo, r, H);

      // 'OnePointRANSAC_EKF:133' S = (H*P*H');
      H_idx_0 = (unsigned int)H->size[0];
      i35 = i_c->size[0] * i_c->size[1];
      i_c->size[0] = (int)H_idx_0;
      i_c->size[1] = 102;
      emxEnsureCapacity((emxArray__common *)i_c, i35, (int)sizeof(float));
      b_m = H->size[0];
      i35 = i_c->size[0] * i_c->size[1];
      i_c->size[1] = 102;
      emxEnsureCapacity((emxArray__common *)i_c, i35, (int)sizeof(float));
      for (i35 = 0; i35 < 102; i35++) {
        loop_ub = i_c->size[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          i_c->data[i36 + i_c->size[0] * i35] = 0.0F;
        }
      }

      if (H->size[0] == 0) {
      } else {
        f_c = H->size[0] * 101;
        mn = 0;
        while ((b_m > 0) && (mn <= f_c)) {
          i35 = mn + b_m;
          for (ic = mn; ic + 1 <= i35; ic++) {
            i_c->data[ic] = 0.0F;
          }

          mn += b_m;
        }

        br = 0;
        mn = 0;
        while ((b_m > 0) && (mn <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 102; ib++) {
            if (b_P[ib] != 0.0F) {
              ia = ar;
              i35 = mn + b_m;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                i_c->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 102;
          mn += b_m;
        }
      }

      i35 = K->size[0] * K->size[1];
      K->size[0] = 102;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
      loop_ub = H->size[0];
      for (i35 = 0; i35 < loop_ub; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          K->data[i36 + K->size[0] * i35] = H->data[i35 + H->size[0] * i36];
        }
      }

      H_idx_0 = (unsigned int)i_c->size[0];
      unnamed_idx_1 = (unsigned int)K->size[1];
      i35 = S->size[0] * S->size[1];
      S->size[0] = (int)H_idx_0;
      S->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)S, i35, (int)sizeof(float));
      b_m = i_c->size[0];
      i35 = S->size[0] * S->size[1];
      emxEnsureCapacity((emxArray__common *)S, i35, (int)sizeof(float));
      loop_ub = S->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        br = S->size[0];
        for (i36 = 0; i36 < br; i36++) {
          S->data[i36 + S->size[0] * i35] = 0.0F;
        }
      }

      if ((i_c->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        f_c = i_c->size[0] * (K->size[1] - 1);
        mn = 0;
        while ((b_m > 0) && (mn <= f_c)) {
          i35 = mn + b_m;
          for (ic = mn; ic + 1 <= i35; ic++) {
            S->data[ic] = 0.0F;
          }

          mn += b_m;
        }

        br = 0;
        mn = 0;
        while ((b_m > 0) && (mn <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 102; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              i35 = mn + b_m;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                S->data[ic] += K->data[ib] * i_c->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 102;
          mn += b_m;
        }
      }

      emxFree_real32_T(&i_c);

      // 'OnePointRANSAC_EKF:134' size_S = nnz(LI_inlier_status)*residualDim;
      n = 0;
      for (k = 0; k < 48; k++) {
        if (LI_inlier_status[k]) {
          n++;
        }
      }

      size_S = (float)n * (float)residualDim;

      // 'OnePointRANSAC_EKF:135' S(1:(size_S+1):size_S^2) = S(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
      wj = size_S * size_S;
      if ((size_S + 1.0F == 0.0F) || (((size_S + 1.0F > 0.0F) && (1.0F > wj)) ||
           ((0.0F > size_S + 1.0F) && (wj > 1.0F)))) {
        i35 = 1;
        i36 = -1;
      } else {
        i35 = (int)(size_S + 1.0F);
        i36 = (int)wj - 1;
      }

      wj = size_S * size_S;
      if ((size_S + 1.0F == 0.0F) || (((size_S + 1.0F > 0.0F) && (1.0F > wj)) ||
           ((0.0F > size_S + 1.0F) && (wj > 1.0F)))) {
        ar = 1;
      } else {
        ar = (int)(size_S + 1.0F);
      }

      emxInit_real32_T1(&b_S, 2);
      mn = b_S->size[0] * b_S->size[1];
      b_S->size[0] = 1;
      b_S->size[1] = div_s32_floor(i36, i35) + 1;
      emxEnsureCapacity((emxArray__common *)b_S, mn, (int)sizeof(float));
      loop_ub = div_s32_floor(i36, i35);
      for (i36 = 0; i36 <= loop_ub; i36++) {
        b_S->data[b_S->size[0] * i36] = S->data[i35 * i36] +
          noiseParameters_image_noise;
      }

      loop_ub = b_S->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        S->data[ar * i35] = b_S->data[b_S->size[0] * i35];
      }

      emxFree_real32_T(&b_S);

      //  add R to HPH'
      // 'OnePointRANSAC_EKF:137' K = (P*H')/S;
      i35 = K->size[0] * K->size[1];
      K->size[0] = 102;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
      loop_ub = H->size[0];
      for (i35 = 0; i35 < loop_ub; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          K->data[i36 + K->size[0] * i35] = H->data[i35 + H->size[0] * i36];
        }
      }

      unnamed_idx_1 = (unsigned int)K->size[1];
      i35 = c->size[0] * c->size[1];
      c->size[0] = 102;
      c->size[1] = (int)unnamed_idx_1;
      c->size[0] = 102;
      emxEnsureCapacity((emxArray__common *)c, i35, (int)sizeof(float));
      loop_ub = c->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          c->data[i36 + c->size[0] * i35] = 0.0F;
        }
      }

      if (K->size[1] == 0) {
      } else {
        f_c = 102 * (K->size[1] - 1);
        for (mn = 0; mn <= f_c; mn += 102) {
          for (ic = mn + 1; ic <= mn + 102; ic++) {
            c->data[ic - 1] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= f_c; mn += 102) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 102; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              for (ic = mn; ic + 1 <= mn + 102; ic++) {
                ia++;
                c->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 102;
          }

          br += 102;
        }
      }

      if ((c->size[1] == 0) || ((S->size[0] == 0) || (S->size[1] == 0))) {
        unnamed_idx_1 = (unsigned int)S->size[0];
        i35 = K->size[0] * K->size[1];
        K->size[0] = 102;
        K->size[1] = (int)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
        loop_ub = 102 * (int)unnamed_idx_1;
        for (i35 = 0; i35 < loop_ub; i35++) {
          K->data[i35] = 0.0F;
        }
      } else if (S->size[0] == S->size[1]) {
        n = S->size[1];
        i35 = A->size[0] * A->size[1];
        A->size[0] = S->size[0];
        A->size[1] = S->size[1];
        emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
        loop_ub = S->size[0] * S->size[1];
        for (i35 = 0; i35 < loop_ub; i35++) {
          A->data[i35] = S->data[i35];
        }

        xgetrf(S->size[1], S->size[1], A, S->size[1], jpvt, &idx);
        i35 = K->size[0] * K->size[1];
        K->size[0] = 102;
        K->size[1] = c->size[1];
        emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
        loop_ub = c->size[0] * c->size[1];
        for (i35 = 0; i35 < loop_ub; i35++) {
          K->data[i35] = c->data[i35];
        }

        for (ib = 0; ib + 1 <= n; ib++) {
          idx = 102 * ib;
          br = n * ib;
          for (k = 1; k <= ib; k++) {
            mn = 102 * (k - 1);
            if (A->data[(k + br) - 1] != 0.0F) {
              for (ic = 0; ic < 102; ic++) {
                K->data[ic + idx] -= A->data[(k + br) - 1] * K->data[ic + mn];
              }
            }
          }

          wj = 1.0F / A->data[ib + br];
          for (ic = 0; ic < 102; ic++) {
            K->data[ic + idx] *= wj;
          }
        }

        for (ib = S->size[1]; ib > 0; ib--) {
          idx = 102 * (ib - 1);
          br = n * (ib - 1) - 1;
          for (k = ib + 1; k <= n; k++) {
            mn = 102 * (k - 1);
            if (A->data[k + br] != 0.0F) {
              for (ic = 0; ic < 102; ic++) {
                K->data[ic + idx] -= A->data[k + br] * K->data[ic + mn];
              }
            }
          }
        }

        for (ar = S->size[1] - 2; ar + 1 > 0; ar--) {
          if (jpvt->data[ar] != ar + 1) {
            mn = jpvt->data[ar] - 1;
            for (idx = 0; idx < 102; idx++) {
              wj = K->data[idx + K->size[0] * ar];
              K->data[idx + K->size[0] * ar] = K->data[idx + K->size[0] * mn];
              K->data[idx + K->size[0] * mn] = wj;
            }
          }
        }
      } else {
        i35 = A->size[0] * A->size[1];
        A->size[0] = S->size[1];
        A->size[1] = S->size[0];
        emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
        loop_ub = S->size[0];
        for (i35 = 0; i35 < loop_ub; i35++) {
          br = S->size[1];
          for (i36 = 0; i36 < br; i36++) {
            A->data[i36 + A->size[0] * i35] = S->data[i35 + S->size[0] * i36];
          }
        }

        xgeqp3(A, tau, jpvt);
        ar = rankFromQR(A);
        ia = A->size[1];
        i35 = Y->size[0] * Y->size[1];
        Y->size[0] = ia;
        Y->size[1] = 102;
        emxEnsureCapacity((emxArray__common *)Y, i35, (int)sizeof(float));
        loop_ub = ia * 102;
        for (i35 = 0; i35 < loop_ub; i35++) {
          Y->data[i35] = 0.0F;
        }

        i35 = B->size[0] * B->size[1];
        B->size[0] = c->size[1];
        B->size[1] = 102;
        emxEnsureCapacity((emxArray__common *)B, i35, (int)sizeof(float));
        for (i35 = 0; i35 < 102; i35++) {
          loop_ub = c->size[1];
          for (i36 = 0; i36 < loop_ub; i36++) {
            B->data[i36 + B->size[0] * i35] = c->data[i35 + c->size[0] * i36];
          }
        }

        b_m = A->size[0];
        idx = A->size[0];
        mn = A->size[1];
        if (idx <= mn) {
          mn = idx;
        }

        for (ib = 0; ib + 1 <= mn; ib++) {
          if (tau->data[ib] != 0.0F) {
            for (k = 0; k < 102; k++) {
              wj = B->data[ib + B->size[0] * k];
              for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                wj += A->data[ic + A->size[0] * ib] * B->data[ic + B->size[0] *
                  k];
              }

              wj *= tau->data[ib];
              if (wj != 0.0F) {
                B->data[ib + B->size[0] * k] -= wj;
                for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                  B->data[ic + B->size[0] * k] -= A->data[ic + A->size[0] * ib] *
                    wj;
                }
              }
            }
          }
        }

        for (k = 0; k < 102; k++) {
          for (ic = 0; ic + 1 <= ar; ic++) {
            Y->data[(jpvt->data[ic] + Y->size[0] * k) - 1] = B->data[ic +
              B->size[0] * k];
          }

          for (ib = ar - 1; ib + 1 > 0; ib--) {
            Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] /= A->data[ib +
              A->size[0] * ib];
            for (ic = 0; ic + 1 <= ib; ic++) {
              Y->data[(jpvt->data[ic] + Y->size[0] * k) - 1] -= Y->data
                [(jpvt->data[ib] + Y->size[0] * k) - 1] * A->data[ic + A->size[0]
                * ib];
            }
          }
        }

        i35 = K->size[0] * K->size[1];
        K->size[0] = 102;
        K->size[1] = Y->size[0];
        emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
        loop_ub = Y->size[0];
        for (i35 = 0; i35 < loop_ub; i35++) {
          for (i36 = 0; i36 < 102; i36++) {
            K->data[i36 + K->size[0] * i35] = Y->data[i35 + Y->size[0] * i36];
          }
        }
      }

      // 'OnePointRANSAC_EKF:139' x_it = K*r;
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i35 = 0; i35 < 102; i35++) {
          x_it[i35] = 0.0F;
          loop_ub = K->size[1];
          for (i36 = 0; i36 < loop_ub; i36++) {
            b_x_it = x_it[i35] + K->data[i35 + K->size[0] * i36] * r->data[i36];
            x_it[i35] = b_x_it;
          }
        }
      } else {
        memset(&x_it[0], 0, 102U * sizeof(float));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0F) {
            ia = ar;
            for (ic = 0; ic < 102; ic++) {
              ia++;
              b_x_it = x_it[ic] + r->data[ib] * K->data[ia - 1];
              x_it[ic] = b_x_it;
            }
          }

          ar += 102;
        }
      }

      // 'OnePointRANSAC_EKF:141' xt.robot_state.pos           = xt.robot_state.pos       + x_it(1:3); 
      mw_neon_mm_add_f32x4(b_xt->robot_state.pos, 3, 1, *(float (*)[3])&x_it[0],
                           &x_it2[0]);
      for (ic = 0; ic < 3; ic++) {
        b_xt->robot_state.pos[ic] = x_it2[ic];
      }

      // 'OnePointRANSAC_EKF:142' xt.robot_state.att           = quatmultJ(quatPlusThetaJ(x_it(4:6)), xt.robot_state.att); 
      // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
      theta = norm(*(float (*)[3])&x_it[3]) * 0.5F;

      // 'quatPlusThetaJ:3' if theta < 0.244
      if (theta < 0.244F) {
        // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
        for (ic = 0; ic < 3; ic++) {
          dq[ic] = 0.5F * x_it[ic + 3];
        }

        dq[3] = 1.0F;
      } else {
        // 'quatPlusThetaJ:5' else
        // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
        // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
        // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
        // 'quatPlusThetaJ:9'          cos(theta)];
        dq[0] = 0.5F * x_it[3] * sinf(theta) / theta;
        dq[1] = 0.5F * x_it[4] * sinf(theta) / theta;
        dq[2] = 0.5F * x_it[5] * sinf(theta) / theta;
        dq[3] = cosf(theta);
      }

      // 'quatPlusThetaJ:11' dq = dq/norm(dq);
      wj = b_norm(dq);
      for (ic = 0; ic < 4; ic++) {
        c_xt[ic] = b_xt->robot_state.att[ic];
        dq[ic] /= wj;
      }

      quatmultJ(dq, c_xt, b_xt->robot_state.att);

      // 'OnePointRANSAC_EKF:143' xt.robot_state.vel           = xt.robot_state.vel       + x_it(7:9); 
      for (ic = 0; ic < 3; ic++) {
        x_it2[ic] = x_it[ic + 6];
      }

      mw_neon_mm_add_f32x4(b_xt->robot_state.vel, 3, 1, x_it2, &fv33[0]);

      // 'OnePointRANSAC_EKF:144' xt.robot_state.IMU.gyro_bias = xt.robot_state.IMU.gyro_bias + x_it(10:12); 
      for (ic = 0; ic < 3; ic++) {
        b_xt->robot_state.vel[ic] = fv33[ic];
        x_it2[ic] = x_it[ic + 9];
      }

      mw_neon_mm_add_f32x4(b_xt->robot_state.IMU.gyro_bias, 3, 1, x_it2, &fv33[0]);

      // 'OnePointRANSAC_EKF:145' xt.robot_state.IMU.acc_bias  = xt.robot_state.IMU.acc_bias + x_it(13:15); 
      for (ic = 0; ic < 3; ic++) {
        b_xt->robot_state.IMU.gyro_bias[ic] = fv33[ic];
        x_it2[ic] = x_it[ic + 12];
      }

      mw_neon_mm_add_f32x4(b_xt->robot_state.IMU.acc_bias, 3, 1, x_it2, &fv33[0]);
      for (ic = 0; ic < 3; ic++) {
        b_xt->robot_state.IMU.acc_bias[ic] = fv33[ic];
      }

      // 'OnePointRANSAC_EKF:146' xt.origin.att                = quatmultJ(quatPlusThetaJ(x_it(16:18)), xt.origin.att); 
      // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
      theta = norm(*(float (*)[3])&x_it[15]) * 0.5F;

      // 'quatPlusThetaJ:3' if theta < 0.244
      if (theta < 0.244F) {
        // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
        for (ic = 0; ic < 3; ic++) {
          dq[ic] = 0.5F * x_it[ic + 15];
        }

        dq[3] = 1.0F;
      } else {
        // 'quatPlusThetaJ:5' else
        // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
        // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
        // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
        // 'quatPlusThetaJ:9'          cos(theta)];
        dq[0] = 0.5F * x_it[15] * sinf(theta) / theta;
        dq[1] = 0.5F * x_it[16] * sinf(theta) / theta;
        dq[2] = 0.5F * x_it[17] * sinf(theta) / theta;
        dq[3] = cosf(theta);
      }

      // 'quatPlusThetaJ:11' dq = dq/norm(dq);
      wj = b_norm(dq);
      for (ic = 0; ic < 4; ic++) {
        c_xt[ic] = b_xt->origin.att[ic];
        dq[ic] /= wj;
      }

      quatmultJ(dq, c_xt, b_xt->origin.att);

      // 'OnePointRANSAC_EKF:148' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
        // 'OnePointRANSAC_EKF:149' xt.anchor_states(anchorIdx).pos = xt.anchor_states(anchorIdx).pos + x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        f_c = anchorIdx * 14 + 18;
        for (i35 = 0; i35 < 3; i35++) {
          x_it2[i35] = x_it[i35 + f_c];
        }

        mw_neon_mm_add_f32x4(b_xt->anchor_states[anchorIdx].pos, 3, 1, x_it2,
                             &fv33[0]);

        // 'OnePointRANSAC_EKF:150' xt.anchor_states(anchorIdx).att = quatmultJ(quatPlusThetaJ(x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att); 
        f_c = 18 + anchorIdx * 14;
        for (ic = 0; ic < 3; ic++) {
          b_xt->anchor_states[anchorIdx].pos[ic] = fv33[ic];
          fp[ic] = x_it[(ic + f_c) + 3];
        }

        // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
        theta = norm(fp) * 0.5F;

        // 'quatPlusThetaJ:3' if theta < 0.244
        if (theta < 0.244F) {
          // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
          for (ic = 0; ic < 3; ic++) {
            dq[ic] = 0.5F * fp[ic];
          }

          dq[3] = 1.0F;
        } else {
          // 'quatPlusThetaJ:5' else
          // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
          // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
          // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
          // 'quatPlusThetaJ:9'          cos(theta)];
          dq[0] = 0.5F * x_it[f_c + 3] * sinf(theta) / theta;
          dq[1] = 0.5F * x_it[f_c + 4] * sinf(theta) / theta;
          dq[2] = 0.5F * x_it[f_c + 5] * sinf(theta) / theta;
          dq[3] = cosf(theta);
        }

        // 'quatPlusThetaJ:11' dq = dq/norm(dq);
        wj = b_norm(dq);
        for (ic = 0; ic < 4; ic++) {
          c_xt[ic] = b_xt->anchor_states[anchorIdx].att[ic];
          dq[ic] /= wj;
        }

        quatmultJ(dq, c_xt, b_xt->anchor_states[anchorIdx].att);

        // 'OnePointRANSAC_EKF:152' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'OnePointRANSAC_EKF:153' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:154' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth += x_it[(anchorIdx * 14 + featureIdx) + 24];

            // 'OnePointRANSAC_EKF:155' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F) {
              // 'OnePointRANSAC_EKF:156' log_warn('Feature %i (%i on %i) is behind its anchor, rejecting', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), int32(featureIdx), int32(anchorIdx)) 
              log_warn(b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
                       .status_idx, featureIdx + 1, anchorIdx + 1);

              // 'OnePointRANSAC_EKF:157' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;

              // 'OnePointRANSAC_EKF:158' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;

              // 'OnePointRANSAC_EKF:159' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;
            }
          }
        }
      }

      // 'OnePointRANSAC_EKF:166' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
      if ((K->size[1] == 1) || (H->size[0] == 1)) {
        for (i35 = 0; i35 < 102; i35++) {
          for (i36 = 0; i36 < 102; i36++) {
            j_c[i35 + 102 * i36] = 0.0F;
            loop_ub = K->size[1];
            for (ar = 0; ar < loop_ub; ar++) {
              j_c[i35 + 102 * i36] += K->data[i35 + K->size[0] * ar] * H->
                data[ar + H->size[0] * i36];
            }
          }
        }
      } else {
        k = K->size[1];
        memset(&j_c[0], 0, 10404U * sizeof(float));
        for (mn = 0; mn <= 10303; mn += 102) {
          for (ic = mn; ic + 1 <= mn + 102; ic++) {
            j_c[ic] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= 10303; mn += 102) {
          ar = 0;
          i35 = br + k;
          for (ib = br; ib + 1 <= i35; ib++) {
            if (H->data[ib] != 0.0F) {
              ia = ar;
              for (ic = mn; ic + 1 <= mn + 102; ic++) {
                ia++;
                j_c[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += 102;
          }

          br += k;
        }
      }

      for (i35 = 0; i35 < 10404; i35++) {
        fv34[i35] = iv13[i35];
      }

      mw_neon_mm_sub_f32x4(fv34, 102, 102, j_c, &fv35[0]);
      for (i35 = 0; i35 < 102; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          fv36[i35 + 102 * i36] = 0.0F;
          for (ar = 0; ar < 102; ar++) {
            fv36[i35 + 102 * i36] += fv35[i35 + 102 * ar] * b_P[ar + 102 * i36];
          }
        }
      }

      for (i35 = 0; i35 < 102; i35++) {
        memcpy(&b_P[i35 * 102], &fv36[i35 * 102], 102U * sizeof(float));
      }
    } else {
      // 'OnePointRANSAC_EKF:167' else
      // 'OnePointRANSAC_EKF:168' LI_inlier_status(:) = false;
      for (ic = 0; ic < 48; ic++) {
        LI_inlier_status[ic] = false;
      }

      // 'OnePointRANSAC_EKF:169' log_warn('1-Point RANSAC didnt find enough LI inliers') 
      b_log_warn();
    }
  }

  // % D Partial EKF update using high-innovation inliers
  // 'OnePointRANSAC_EKF:174' HI_inlierCandidates = activeFeatures & ~LI_inlier_status; 
  //  high innovation inliers (ordered like updateVect)
  // 'OnePointRANSAC_EKF:175' HI_inlierStatus = HI_inlierCandidates;
  for (ic = 0; ic < 48; ic++) {
    b_HI_inlierCandidates = (activeFeatures[ic] && (!LI_inlier_status[ic]));
    HI_inlierStatus[ic] = b_HI_inlierCandidates;
    HI_inlierCandidates[ic] = b_HI_inlierCandidates;
  }

  // 'OnePointRANSAC_EKF:176' HI_ind = find(HI_inlierStatus);
  idx = 0;
  mn = 1;
  exitg1 = false;
  while ((!exitg1) && (mn < 49)) {
    guard1 = false;
    if (HI_inlierCandidates[mn - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)mn;
      if (idx >= 48) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      mn++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  i35 = out->size[0];
  out->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)out, i35, (int)sizeof(float));
  for (i35 = 0; i35 < loop_ub; i35++) {
    out->data[i35] = ii_data[i35];
  }

  // 'OnePointRANSAC_EKF:177' num_HI_inlierCandidates = nnz(HI_inlierCandidates); 
  n = 0;
  for (k = 0; k < 48; k++) {
    if (HI_inlierCandidates[k]) {
      n++;
    }
  }

  // 'OnePointRANSAC_EKF:179' if any(HI_inlierCandidates)
  if (any(HI_inlierCandidates)) {
    emxInit_real32_T1(&b_K, 2);
    emxInit_real32_T1(&b_H, 2);

    // 'OnePointRANSAC_EKF:180' K = 0;
    i35 = b_K->size[0] * b_K->size[1];
    b_K->size[0] = 1;
    b_K->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_K, i35, (int)sizeof(float));
    b_K->data[0] = 0.0F;

    //  for coder
    // 'OnePointRANSAC_EKF:181' H = 0;
    i35 = b_H->size[0] * b_H->size[1];
    b_H->size[0] = 1;
    b_H->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_H, i35, (int)sizeof(float));
    b_H->data[0] = 0.0F;

    //  for coder
    // 'OnePointRANSAC_EKF:182' xt_it = xt;
    xt_it = *b_xt;

    //  total state for iteration
    // 'OnePointRANSAC_EKF:183' x_it = zeros(numStates + numAnchors*(6 + numPointsPerAnchor), 1); 
    memset(&x_it[0], 0, 102U * sizeof(float));

    //  error state for iteration
    // 'OnePointRANSAC_EKF:184' for it = 1:VIOParameters.max_ekf_iterations
    it = 1;
    emxInit_real32_T1(&k_c, 2);
    emxInit_real32_T1(&l_c, 2);
    emxInit_real32_T(&m_c, 1);
    emxInit_real32_T1(&c_H, 2);
    emxInit_real32_T1(&c_S, 2);
    emxInit_real32_T1(&d_H, 2);
    while (it <= c_VIOParameters_max_ekf_iterati) {
      // 'OnePointRANSAC_EKF:185' xt_it = getScaledMap(xt_it);
      getScaledMap(&xt_it);

      //  build the map according to the current estimate
      // 'OnePointRANSAC_EKF:187' [r, H, ind] = getH_R_res(xt_it, z_u_l, z_u_r, HI_inlierStatus, stereoParams, VIOParameters); 
      getH_R_res(xt_it.robot_state.pos, xt_it.robot_state.att,
                 xt_it.anchor_states, z_u_l, z_u_r, HI_inlierStatus,
                 c_stereoParams_CameraParameters,
                 d_stereoParams_CameraParameters,
                 e_stereoParams_CameraParameters,
                 f_stereoParams_CameraParameters, stereoParams_r_lr,
                 stereoParams_R_rl, VIOParameters_full_stereo, r, H, ind);
      i35 = b_H->size[0] * b_H->size[1];
      b_H->size[0] = H->size[0];
      b_H->size[1] = 102;
      emxEnsureCapacity((emxArray__common *)b_H, i35, (int)sizeof(float));
      loop_ub = H->size[0] * H->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        b_H->data[i35] = H->data[i35];
      }

      //  the residual is ordered by anchors/features, not like updateVect
      // 'OnePointRANSAC_EKF:190' P_a = P(1:6, 1:6);
      // 'OnePointRANSAC_EKF:192' for k = 1:length(HI_ind)
      for (k = 0; k < out->size[0]; k++) {
        // 'OnePointRANSAC_EKF:193' anchorIdx = ind(k, 1);
        // 'OnePointRANSAC_EKF:194' featureIdx = ind(k, 2);
        featureIdx = ind->data[k + ind->size[0]];

        // 'OnePointRANSAC_EKF:195' H_a = H((k-1)*residualDim + (1:residualDim), 1:6); 
        f_c = k * residualDim;
        b_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        ar = b_ndbl + 1;
        idx = (b_ndbl - residualDim) + 1;
        if (fabs((double)idx) < 2.38418579E-7F * (float)residualDim) {
          b_ndbl++;
          ar = residualDim;
        } else if (idx > 0) {
          ar = b_ndbl;
        } else {
          b_ndbl++;
        }

        c_xt[0] = 1.0F;
        if (b_ndbl > 1) {
          c_xt[b_ndbl - 1] = (float)ar;
          i35 = b_ndbl - 1;
          idx = i35 / 2;
          mn = 1;
          while (mn <= idx - 1) {
            c_xt[1] = 2.0F;
            c_xt[b_ndbl - 2] = (float)ar - 1.0F;
            mn = 2;
          }

          if (idx << 1 == b_ndbl - 1) {
            c_xt[idx] = (1.0F + (float)ar) / 2.0F;
          } else {
            c_xt[idx] = 1.0F + (float)idx;
            c_xt[idx + 1] = (float)(ar - idx);
          }
        }

        for (i35 = 0; i35 < b_ndbl; i35++) {
          i_c_data[i35] = (float)f_c + c_xt[i35];
        }

        for (i35 = 0; i35 < 6; i35++) {
          for (i36 = 0; i36 < b_ndbl; i36++) {
            H_a_data[i36 + b_ndbl * i35] = b_H->data[((int)((float)f_c +
              c_xt[i36]) + b_H->size[0] * i35) - 1];
          }
        }

        // 'OnePointRANSAC_EKF:196' H_b = H((k-1)*residualDim + (1:residualDim), numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
        f_c = k * residualDim;
        c_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        ar = c_ndbl + 1;
        idx = (c_ndbl - residualDim) + 1;
        if (fabs((double)idx) < 2.38418579E-7F * (float)residualDim) {
          c_ndbl++;
          ar = residualDim;
        } else if (idx > 0) {
          ar = c_ndbl;
        } else {
          c_ndbl++;
        }

        vec_data[0] = 1.0F;
        if (c_ndbl > 1) {
          vec_data[c_ndbl - 1] = (float)ar;
          i35 = c_ndbl - 1;
          idx = i35 / 2;
          mn = 1;
          while (mn <= idx - 1) {
            vec_data[1] = 2.0F;
            vec_data[c_ndbl - 2] = (float)ar - 1.0F;
            mn = 2;
          }

          if (idx << 1 == c_ndbl - 1) {
            vec_data[idx] = (1.0F + (float)ar) / 2.0F;
          } else {
            vec_data[idx] = 1.0F + (float)idx;
            vec_data[idx + 1] = (float)(ar - idx);
          }
        }

        for (i35 = 0; i35 < c_ndbl; i35++) {
          j_c_data[i35] = (float)f_c + vec_data[i35];
        }

        idx = ind->data[k];
        qY = idx - 1;
        if ((idx < 0) && (qY >= 0)) {
          qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(qY, 14);
        qY = 18 + ndbl;
        if ((ndbl > 0) && (qY <= 0)) {
          qY = MAX_int32_T;
        }

        for (i35 = 0; i35 < 6; i35++) {
          ndbl = 1 + i35;
          b_qY = qY + ndbl;
          if ((qY > 0) && (b_qY <= 0)) {
            b_qY = MAX_int32_T;
          }

          d_c[i35] = b_qY;
        }

        for (i35 = 0; i35 < 6; i35++) {
          for (i36 = 0; i36 < c_ndbl; i36++) {
            ndbl = 1 + i35;
            b_qY = qY + ndbl;
            if ((qY > 0) && (b_qY <= 0)) {
              b_qY = MAX_int32_T;
            }

            H_b_data[i36 + c_ndbl * i35] = b_H->data[((int)((float)f_c +
              vec_data[i36]) + b_H->size[0] * (b_qY - 1)) - 1];
          }
        }

        // 'OnePointRANSAC_EKF:197' H_c = H((k-1)*residualDim + (1:residualDim), numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
        f_c = k * residualDim;
        d_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        ar = d_ndbl + 1;
        idx = (d_ndbl - residualDim) + 1;
        if (fabs((double)idx) < 2.38418579E-7F * (float)residualDim) {
          d_ndbl++;
          ar = residualDim;
        } else if (idx > 0) {
          ar = d_ndbl;
        } else {
          d_ndbl++;
        }

        c_xt[0] = 1.0F;
        if (d_ndbl > 1) {
          c_xt[d_ndbl - 1] = (float)ar;
          i35 = d_ndbl - 1;
          idx = i35 / 2;
          mn = 1;
          while (mn <= idx - 1) {
            c_xt[1] = 2.0F;
            c_xt[d_ndbl - 2] = (float)ar - 1.0F;
            mn = 2;
          }

          if (idx << 1 == d_ndbl - 1) {
            c_xt[idx] = (1.0F + (float)ar) / 2.0F;
          } else {
            c_xt[idx] = 1.0F + (float)idx;
            c_xt[idx + 1] = (float)(ar - idx);
          }
        }

        idx = ind->data[k];
        qY = idx - 1;
        if ((idx < 0) && (qY >= 0)) {
          qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(qY, 14);
        qY = 18 + ndbl;
        if ((ndbl > 0) && (qY <= 0)) {
          qY = MAX_int32_T;
        }

        b_qY = qY + 6;
        if ((qY > 0) && (b_qY <= 0)) {
          b_qY = MAX_int32_T;
        }

        ndbl = ind->data[k + ind->size[0]];
        qY = b_qY + ndbl;
        if ((b_qY < 0) && ((ndbl < 0) && (qY >= 0))) {
          qY = MIN_int32_T;
        } else {
          if ((b_qY > 0) && ((ndbl > 0) && (qY <= 0))) {
            qY = MAX_int32_T;
          }
        }

        for (i35 = 0; i35 < d_ndbl; i35++) {
          H_c_data[i35] = b_H->data[((int)((float)f_c + c_xt[i35]) + b_H->size[0]
            * (qY - 1)) - 1];
        }

        // 'OnePointRANSAC_EKF:198' P_b = P(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6), 1:6); 
        idx = ind->data[k];
        qY = idx - 1;
        if ((idx < 0) && (qY >= 0)) {
          qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(qY, 14);
        qY = 18 + ndbl;
        if ((ndbl > 0) && (qY <= 0)) {
          qY = MAX_int32_T;
        }

        for (i35 = 0; i35 < 6; i35++) {
          ndbl = 1 + i35;
          b_qY = qY + ndbl;
          if ((qY > 0) && (b_qY <= 0)) {
            b_qY = MAX_int32_T;
          }

          e_c[i35] = b_qY;
          for (i36 = 0; i36 < 6; i36++) {
            ndbl = 1 + i36;
            b_qY = qY + ndbl;
            if ((qY > 0) && (b_qY <= 0)) {
              b_qY = MAX_int32_T;
            }

            P_b[i36 + 6 * i35] = b_P[(b_qY + 102 * i35) - 1];
          }
        }

        // 'OnePointRANSAC_EKF:199' P_c = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, 1:6); 
        idx = ind->data[k];
        b_qY = idx - 1;
        if ((idx < 0) && (b_qY >= 0)) {
          b_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(b_qY, 14);
        b_qY = 18 + ndbl;
        if ((ndbl > 0) && (b_qY <= 0)) {
          b_qY = MAX_int32_T;
        }

        c_qY = b_qY + 6;
        if ((b_qY > 0) && (c_qY <= 0)) {
          c_qY = MAX_int32_T;
        }

        ndbl = ind->data[k + ind->size[0]];
        b_qY = c_qY + ndbl;
        if ((c_qY < 0) && ((ndbl < 0) && (b_qY >= 0))) {
          b_qY = MIN_int32_T;
        } else {
          if ((c_qY > 0) && ((ndbl > 0) && (b_qY <= 0))) {
            b_qY = MAX_int32_T;
          }
        }

        f_c = b_qY - 1;

        // 'OnePointRANSAC_EKF:200' P_d = P_b';
        for (i35 = 0; i35 < 6; i35++) {
          for (i36 = 0; i36 < 6; i36++) {
            P_d[i36 + 6 * i35] = P_b[i35 + 6 * i36];
          }
        }

        // 'OnePointRANSAC_EKF:201' P_e = P(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6), numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
        idx = ind->data[k];
        b_qY = idx - 1;
        if ((idx < 0) && (b_qY >= 0)) {
          b_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(b_qY, 14);
        b_qY = 18 + ndbl;
        if ((ndbl > 0) && (b_qY <= 0)) {
          b_qY = MAX_int32_T;
        }

        for (i35 = 0; i35 < 6; i35++) {
          ndbl = 1 + i35;
          d_qY = b_qY + ndbl;
          if ((b_qY > 0) && (d_qY <= 0)) {
            d_qY = MAX_int32_T;
          }

          g_c[i35] = d_qY;
        }

        idx = ind->data[k];
        d_qY = idx - 1;
        if ((idx < 0) && (d_qY >= 0)) {
          d_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(d_qY, 14);
        d_qY = 18 + ndbl;
        if ((ndbl > 0) && (d_qY <= 0)) {
          d_qY = MAX_int32_T;
        }

        for (i35 = 0; i35 < 6; i35++) {
          ndbl = 1 + i35;
          e_qY = d_qY + ndbl;
          if ((d_qY > 0) && (e_qY <= 0)) {
            e_qY = MAX_int32_T;
          }

          h_c[i35] = e_qY;
        }

        // 'OnePointRANSAC_EKF:202' P_f = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
        idx = ind->data[k];
        e_qY = idx - 1;
        if ((idx < 0) && (e_qY >= 0)) {
          e_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(e_qY, 14);
        e_qY = 18 + ndbl;
        if ((ndbl > 0) && (e_qY <= 0)) {
          e_qY = MAX_int32_T;
        }

        idx = ind->data[k];
        f_qY = idx - 1;
        if ((idx < 0) && (f_qY >= 0)) {
          f_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(f_qY, 14);
        f_qY = 18 + ndbl;
        if ((ndbl > 0) && (f_qY <= 0)) {
          f_qY = MAX_int32_T;
        }

        g_qY = f_qY + 6;
        if ((f_qY > 0) && (g_qY <= 0)) {
          g_qY = MAX_int32_T;
        }

        idx = ind->data[k];
        f_qY = idx - 1;
        if ((idx < 0) && (f_qY >= 0)) {
          f_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(f_qY, 14);
        f_qY = 18 + ndbl;
        if ((ndbl > 0) && (f_qY <= 0)) {
          f_qY = MAX_int32_T;
        }

        idx = ind->data[k];
        h_qY = idx - 1;
        if ((idx < 0) && (h_qY >= 0)) {
          h_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(h_qY, 14);
        h_qY = 18 + ndbl;
        if ((ndbl > 0) && (h_qY <= 0)) {
          h_qY = MAX_int32_T;
        }

        i_qY = h_qY + 6;
        if ((h_qY > 0) && (i_qY <= 0)) {
          i_qY = MAX_int32_T;
        }

        ndbl = ind->data[k + ind->size[0]];
        h_qY = i_qY + ndbl;
        if ((i_qY < 0) && ((ndbl < 0) && (h_qY >= 0))) {
          h_qY = MIN_int32_T;
        } else {
          if ((i_qY > 0) && ((ndbl > 0) && (h_qY <= 0))) {
            h_qY = MAX_int32_T;
          }
        }

        for (i35 = 0; i35 < 6; i35++) {
          ndbl = 1 + i35;
          i_qY = f_qY + ndbl;
          if ((f_qY > 0) && (i_qY <= 0)) {
            i_qY = MAX_int32_T;
          }

          P_f[i35] = b_P[(h_qY + 102 * (i_qY - 1)) - 1];
        }

        // 'OnePointRANSAC_EKF:203' P_g = P_c';
        // 'OnePointRANSAC_EKF:204' P_h = P_f';
        // 'OnePointRANSAC_EKF:205' P_i = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
        idx = ind->data[k];
        h_qY = idx - 1;
        if ((idx < 0) && (h_qY >= 0)) {
          h_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(h_qY, 14);
        h_qY = 18 + ndbl;
        if ((ndbl > 0) && (h_qY <= 0)) {
          h_qY = MAX_int32_T;
        }

        i_qY = h_qY + 6;
        if ((h_qY > 0) && (i_qY <= 0)) {
          i_qY = MAX_int32_T;
        }

        ndbl = ind->data[k + ind->size[0]];
        h_qY = i_qY + ndbl;
        if ((i_qY < 0) && ((ndbl < 0) && (h_qY >= 0))) {
          h_qY = MIN_int32_T;
        } else {
          if ((i_qY > 0) && ((ndbl > 0) && (h_qY <= 0))) {
            h_qY = MAX_int32_T;
          }
        }

        idx = ind->data[k];
        i_qY = idx - 1;
        if ((idx < 0) && (i_qY >= 0)) {
          i_qY = MIN_int32_T;
        }

        ndbl = mul_s32_s32_s32_sat(i_qY, 14);
        i_qY = 18 + ndbl;
        if ((ndbl > 0) && (i_qY <= 0)) {
          i_qY = MAX_int32_T;
        }

        idx = i_qY + 6;
        if ((i_qY > 0) && (idx <= 0)) {
          idx = MAX_int32_T;
        }

        ndbl = ind->data[k + ind->size[0]];
        i_qY = idx + ndbl;
        if ((idx < 0) && ((ndbl < 0) && (i_qY >= 0))) {
          i_qY = MIN_int32_T;
        } else {
          if ((idx > 0) && ((ndbl > 0) && (i_qY <= 0))) {
            i_qY = MAX_int32_T;
          }
        }

        // 'OnePointRANSAC_EKF:206' S_feature = (H_a*P_a + H_b*P_b + H_c*P_c)*H_a' + ... 
        // 'OnePointRANSAC_EKF:207'                         (H_a*P_d + H_b*P_e + H_c*P_f)*H_b' + ... 
        // 'OnePointRANSAC_EKF:208'                         (H_a*P_g + H_b*P_h + H_c*P_i)*H_c'; 
        for (i35 = 0; i35 < 6; i35++) {
          for (i36 = 0; i36 < b_ndbl; i36++) {
            k_c_data[i36 + b_ndbl * i35] = 0.0F;
          }
        }

        b_m = b_ndbl * 5;
        for (mn = 0; mn <= b_m; mn += b_ndbl) {
          i35 = mn + b_ndbl;
          for (ic = mn; ic + 1 <= i35; ic++) {
            k_c_data[ic] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= b_m; mn += b_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (b_P[ib % 6 + 102 * (ib / 6)] != 0.0F) {
              ia = ar;
              i35 = mn + b_ndbl;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                for (i36 = 0; i36 < b_ndbl; i36++) {
                  tmp_data[i36] = (int)i_c_data[i36];
                }

                k_c_data[ic] += b_P[ib % 6 + 102 * (ib / 6)] * b_H->data
                  [(tmp_data[(ia - 1) % b_ndbl] + b_H->size[0] * ((ia - 1) /
                     b_ndbl)) - 1];
              }
            }

            ar += b_ndbl;
          }

          br += 6;
        }

        for (i35 = 0; i35 < 6; i35++) {
          for (i36 = 0; i36 < c_ndbl; i36++) {
            l_c_data[i36 + c_ndbl * i35] = 0.0F;
          }
        }

        b_m = c_ndbl * 5;
        for (mn = 0; mn <= b_m; mn += c_ndbl) {
          i35 = mn + c_ndbl;
          for (ic = mn; ic + 1 <= i35; ic++) {
            l_c_data[ic] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= b_m; mn += c_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (b_P[(e_c[ib % 6] + 102 * (ib / 6)) - 1] != 0.0F) {
              ia = ar;
              i35 = mn + c_ndbl;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                for (i36 = 0; i36 < c_ndbl; i36++) {
                  tmp_data[i36] = (int)j_c_data[i36];
                }

                ndbl = 1 + ib % 6;
                idx = qY + ndbl;
                if ((qY < 0) && ((ndbl < 0) && (idx >= 0))) {
                  idx = MIN_int32_T;
                } else {
                  if ((qY > 0) && ((ndbl > 0) && (idx <= 0))) {
                    idx = MAX_int32_T;
                  }
                }

                l_c_data[ic] += b_P[(idx + 102 * (ib / 6)) - 1] * b_H->data
                  [(tmp_data[(ia - 1) % c_ndbl] + b_H->size[0] * (d_c[(ia - 1) /
                     c_ndbl] - 1)) - 1];
              }
            }

            ar += c_ndbl;
          }

          br += 6;
        }

        for (i35 = 0; i35 < d_ndbl; i35++) {
          for (i36 = 0; i36 < 6; i36++) {
            c_data[i35 + d_ndbl * i36] = (k_c_data[i35 + b_ndbl * i36] +
              l_c_data[i35 + c_ndbl * i36]) + H_c_data[i35] * b_P[f_c + 102 *
              i36];
          }
        }

        for (i35 = 0; i35 < b_ndbl; i35++) {
          for (i36 = 0; i36 < 6; i36++) {
            b_data[i36 + 6 * i35] = H_a_data[i35 + b_ndbl * i36];
          }
        }

        for (i35 = 0; i35 < b_ndbl; i35++) {
          for (i36 = 0; i36 < d_ndbl; i36++) {
            m_c_data[i36 + d_ndbl * i35] = 0.0F;
          }
        }

        b_m = d_ndbl * (b_ndbl - 1);
        for (mn = 0; mn <= b_m; mn += d_ndbl) {
          i35 = mn + d_ndbl;
          for (ic = mn; ic + 1 <= i35; ic++) {
            m_c_data[ic] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= b_m; mn += d_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (b_data[ib] != 0.0F) {
              ia = ar;
              i35 = mn + d_ndbl;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                m_c_data[ic] += b_data[ib] * c_data[ia - 1];
              }
            }

            ar += d_ndbl;
          }

          br += 6;
        }

        for (i35 = 0; i35 < 6; i35++) {
          for (i36 = 0; i36 < b_ndbl; i36++) {
            n_c_data[i36 + b_ndbl * i35] = 0.0F;
          }
        }

        b_m = b_ndbl * 5;
        for (mn = 0; mn <= b_m; mn += b_ndbl) {
          i35 = mn + b_ndbl;
          for (ic = mn; ic + 1 <= i35; ic++) {
            n_c_data[ic] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= b_m; mn += b_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (P_d[ib] != 0.0F) {
              ia = ar;
              i35 = mn + b_ndbl;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                for (i36 = 0; i36 < b_ndbl; i36++) {
                  tmp_data[i36] = (int)i_c_data[i36];
                }

                n_c_data[ic] += P_d[ib] * b_H->data[(tmp_data[(ia - 1) % b_ndbl]
                  + b_H->size[0] * ((ia - 1) / b_ndbl)) - 1];
              }
            }

            ar += b_ndbl;
          }

          br += 6;
        }

        for (i35 = 0; i35 < 6; i35++) {
          for (i36 = 0; i36 < c_ndbl; i36++) {
            o_c_data[i36 + c_ndbl * i35] = 0.0F;
          }
        }

        b_m = c_ndbl * 5;
        for (mn = 0; mn <= b_m; mn += c_ndbl) {
          i35 = mn + c_ndbl;
          for (ic = mn; ic + 1 <= i35; ic++) {
            o_c_data[ic] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= b_m; mn += c_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (b_P[(g_c[ib % 6] + 102 * (h_c[ib / 6] - 1)) - 1] != 0.0F) {
              ia = ar;
              i35 = mn + c_ndbl;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                for (i36 = 0; i36 < c_ndbl; i36++) {
                  tmp_data[i36] = (int)j_c_data[i36];
                }

                ndbl = 1 + ib % 6;
                qY = b_qY + ndbl;
                if ((b_qY < 0) && ((ndbl < 0) && (qY >= 0))) {
                  qY = MIN_int32_T;
                } else {
                  if ((b_qY > 0) && ((ndbl > 0) && (qY <= 0))) {
                    qY = MAX_int32_T;
                  }
                }

                ndbl = 1 + ib / 6;
                idx = d_qY + ndbl;
                if ((d_qY < 0) && ((ndbl < 0) && (idx >= 0))) {
                  idx = MIN_int32_T;
                } else {
                  if ((d_qY > 0) && ((ndbl > 0) && (idx <= 0))) {
                    idx = MAX_int32_T;
                  }
                }

                o_c_data[ic] += b_P[(qY + 102 * (idx - 1)) - 1] * b_H->data
                  [(tmp_data[(ia - 1) % c_ndbl] + b_H->size[0] * (d_c[(ia - 1) /
                     c_ndbl] - 1)) - 1];
              }
            }

            ar += c_ndbl;
          }

          br += 6;
        }

        for (i35 = 0; i35 < d_ndbl; i35++) {
          for (i36 = 0; i36 < 6; i36++) {
            c_data[i35 + d_ndbl * i36] = (n_c_data[i35 + b_ndbl * i36] +
              o_c_data[i35 + c_ndbl * i36]) + H_c_data[i35] * P_f[i36];
          }
        }

        for (i35 = 0; i35 < c_ndbl; i35++) {
          for (i36 = 0; i36 < 6; i36++) {
            b_data[i36 + 6 * i35] = H_b_data[i35 + c_ndbl * i36];
          }
        }

        for (i35 = 0; i35 < c_ndbl; i35++) {
          for (i36 = 0; i36 < d_ndbl; i36++) {
            p_c_data[i36 + d_ndbl * i35] = 0.0F;
          }
        }

        b_m = d_ndbl * (c_ndbl - 1);
        for (mn = 0; mn <= b_m; mn += d_ndbl) {
          i35 = mn + d_ndbl;
          for (ic = mn; ic + 1 <= i35; ic++) {
            p_c_data[ic] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= b_m; mn += d_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (b_data[ib] != 0.0F) {
              ia = ar;
              i35 = mn + d_ndbl;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                p_c_data[ic] += b_data[ib] * c_data[ia - 1];
              }
            }

            ar += d_ndbl;
          }

          br += 6;
        }

        for (i35 = 0; i35 < b_ndbl; i35++) {
          q_c_data[i35] = 0.0F;
        }

        mn = 0;
        while (mn <= 0) {
          for (ic = 1; ic <= b_ndbl; ic++) {
            q_c_data[ic - 1] = 0.0F;
          }

          mn = b_ndbl;
        }

        br = 6;
        mn = 0;
        while (mn <= 0) {
          ar = 0;
          for (ib = br - 5; ib <= br; ib++) {
            ndbl = featureIdx;
            qY = c_qY + ndbl;
            if ((c_qY < 0) && ((ndbl < 0) && (qY >= 0))) {
              qY = MIN_int32_T;
            } else {
              if ((c_qY > 0) && ((ndbl > 0) && (qY <= 0))) {
                qY = MAX_int32_T;
              }
            }

            if (b_P[(qY + 102 * (ib - 1)) - 1] != 0.0F) {
              ia = ar;
              for (ic = 0; ic + 1 <= b_ndbl; ic++) {
                ia++;
                for (i35 = 0; i35 < b_ndbl; i35++) {
                  tmp_data[i35] = (int)i_c_data[i35];
                }

                q_c_data[ic] += b_P[f_c + 102 * (ib - 1)] * b_H->data[(tmp_data
                  [(ia - 1) % b_ndbl] + b_H->size[0] * ((ia - 1) / b_ndbl)) - 1];
              }
            }

            ar += b_ndbl;
          }

          br += 6;
          mn = b_ndbl;
        }

        for (i35 = 0; i35 < c_ndbl; i35++) {
          r_c_data[i35] = 0.0F;
        }

        mn = 0;
        while (mn <= 0) {
          for (ic = 1; ic <= c_ndbl; ic++) {
            r_c_data[ic - 1] = 0.0F;
          }

          mn = c_ndbl;
        }

        br = 6;
        mn = 0;
        while (mn <= 0) {
          ar = 0;
          for (ib = br - 5; ib <= br; ib++) {
            ndbl = featureIdx;
            qY = g_qY + ndbl;
            if ((g_qY < 0) && ((ndbl < 0) && (qY >= 0))) {
              qY = MIN_int32_T;
            } else {
              if ((g_qY > 0) && ((ndbl > 0) && (qY <= 0))) {
                qY = MAX_int32_T;
              }
            }

            ndbl = iv12[ib - 1];
            b_qY = f_qY + ndbl;
            if ((f_qY > 0) && (b_qY <= 0)) {
              b_qY = MAX_int32_T;
            }

            if (b_P[(qY + 102 * (b_qY - 1)) - 1] != 0.0F) {
              ia = ar;
              for (ic = 0; ic + 1 <= c_ndbl; ic++) {
                ia++;
                for (i35 = 0; i35 < c_ndbl; i35++) {
                  tmp_data[i35] = (int)j_c_data[i35];
                }

                qY = e_qY + 6;
                if ((e_qY > 0) && (qY <= 0)) {
                  qY = MAX_int32_T;
                }

                ndbl = ind->data[k + ind->size[0]];
                b_qY = qY + ndbl;
                if ((qY < 0) && ((ndbl < 0) && (b_qY >= 0))) {
                  b_qY = MIN_int32_T;
                } else {
                  if ((qY > 0) && ((ndbl > 0) && (b_qY <= 0))) {
                    b_qY = MAX_int32_T;
                  }
                }

                qY = f_qY + ib;
                if ((f_qY < 0) && ((ib < 0) && (qY >= 0))) {
                  qY = MIN_int32_T;
                } else {
                  if ((f_qY > 0) && ((ib > 0) && (qY <= 0))) {
                    qY = MAX_int32_T;
                  }
                }

                r_c_data[ic] += b_P[(b_qY + 102 * (qY - 1)) - 1] * b_H->data
                  [(tmp_data[(ia - 1) % c_ndbl] + b_H->size[0] * (d_c[(ia - 1) /
                     c_ndbl] - 1)) - 1];
              }
            }

            ar += c_ndbl;
          }

          br += 6;
          mn = c_ndbl;
        }

        wj = b_P[(h_qY + 102 * (i_qY - 1)) - 1];
        for (i35 = 0; i35 < b_ndbl; i35++) {
          vec_data[i35] = (q_c_data[i35] + r_c_data[i35]) + H_c_data[i35] * wj;
        }

        for (i35 = 0; i35 < b_ndbl; i35++) {
          for (i36 = 0; i36 < d_ndbl; i36++) {
            S_feature_data[i35 + b_ndbl * i36] = (m_c_data[i35 + d_ndbl * i36] +
              p_c_data[i35 + d_ndbl * i36]) + vec_data[i35] * H_c_data[i36];
          }
        }

        // 'OnePointRANSAC_EKF:210' size_S = residualDim;
        // 'OnePointRANSAC_EKF:211' S_feature(1:(size_S+1):size_S^2) = S_feature(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
        i35 = residualDim * residualDim - 1;
        i36 = residualDim + 1;
        loop_ub = div_s32_floor(i35, i36);
        for (ar = 0; ar <= loop_ub; ar++) {
          b_S_feature_data[ar] = S_feature_data[i36 * ar] +
            noiseParameters_image_noise;
        }

        loop_ub = div_s32_floor(i35, i36) + 1;
        for (i35 = 0; i35 < loop_ub; i35++) {
          S_feature_data[(residualDim + 1) * i35] = b_S_feature_data[i35];
        }

        //  add R to HPH'
        // 'OnePointRANSAC_EKF:213' if coder.target('MATLAB')
        // 'OnePointRANSAC_EKF:221' innov = r((k-1)*residualDim + (1:residualDim))' / S_feature * r((k-1)*residualDim + (1:residualDim)); 
        f_c = k * residualDim;
        c_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        ar = c_ndbl + 1;
        idx = (c_ndbl - residualDim) + 1;
        if (fabs((double)idx) < 2.38418579E-7F * (float)residualDim) {
          c_ndbl++;
          ar = residualDim;
        } else if (idx > 0) {
          ar = c_ndbl;
        } else {
          c_ndbl++;
        }

        c_xt[0] = 1.0F;
        if (c_ndbl > 1) {
          c_xt[c_ndbl - 1] = (float)ar;
          i35 = c_ndbl - 1;
          idx = i35 / 2;
          mn = 1;
          while (mn <= idx - 1) {
            c_xt[1] = 2.0F;
            c_xt[c_ndbl - 2] = (float)ar - 1.0F;
            mn = 2;
          }

          if (idx << 1 == c_ndbl - 1) {
            c_xt[idx] = (1.0F + (float)ar) / 2.0F;
          } else {
            c_xt[idx] = 1.0F + (float)idx;
            c_xt[idx + 1] = (float)(ar - idx);
          }
        }

        for (i35 = 0; i35 < c_ndbl; i35++) {
          c_xt[i35] += (float)f_c;
        }

        f_c = k * residualDim;
        ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        ar = ndbl + 1;
        idx = (ndbl - residualDim) + 1;
        if (fabs((double)idx) < 2.38418579E-7F * (float)residualDim) {
          ndbl++;
          ar = residualDim;
        } else if (idx > 0) {
          ar = ndbl;
        } else {
          ndbl++;
        }

        vec_data[0] = 1.0F;
        if (ndbl > 1) {
          vec_data[ndbl - 1] = (float)ar;
          i35 = ndbl - 1;
          idx = i35 / 2;
          mn = 1;
          while (mn <= idx - 1) {
            vec_data[1] = 2.0F;
            vec_data[ndbl - 2] = (float)ar - 1.0F;
            mn = 2;
          }

          if (idx << 1 == ndbl - 1) {
            vec_data[idx] = (1.0F + (float)ar) / 2.0F;
          } else {
            vec_data[idx] = 1.0F + (float)idx;
            vec_data[idx + 1] = (float)(ar - idx);
          }
        }

        for (i35 = 0; i35 < ndbl; i35++) {
          vec_data[i35] += (float)f_c;
        }

        if (b_ndbl == d_ndbl) {
          i35 = A->size[0] * A->size[1];
          A->size[0] = b_ndbl;
          A->size[1] = d_ndbl;
          emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
          loop_ub = b_ndbl * d_ndbl;
          for (i35 = 0; i35 < loop_ub; i35++) {
            A->data[i35] = S_feature_data[i35];
          }

          xgetrf(d_ndbl, d_ndbl, A, d_ndbl, jpvt, &idx);
          ia = c_ndbl;
          for (i35 = 0; i35 < c_ndbl; i35++) {
            i_c_data[i35] = r->data[(int)c_xt[i35] - 1];
          }

          for (ib = 0; ib + 1 <= d_ndbl; ib++) {
            br = d_ndbl * ib;
            for (mn = 0; mn + 1 <= ib; mn++) {
              if (A->data[mn + br] != 0.0F) {
                i_c_data[ib] -= A->data[mn + br] * i_c_data[mn];
              }
            }

            wj = A->data[ib + br];
            i_c_data[ib] *= 1.0F / wj;
          }

          for (ib = d_ndbl; ib > 0; ib--) {
            br = d_ndbl * (ib - 1);
            for (mn = ib; mn + 1 <= d_ndbl; mn++) {
              if (A->data[mn + br] != 0.0F) {
                i_c_data[ib - 1] -= A->data[mn + br] * i_c_data[mn];
              }
            }
          }

          for (ar = d_ndbl - 2; ar + 1 > 0; ar--) {
            if (jpvt->data[ar] != ar + 1) {
              wj = i_c_data[ar];
              i_c_data[ar] = i_c_data[jpvt->data[ar] - 1];
              i_c_data[jpvt->data[ar] - 1] = wj;
            }
          }
        } else {
          i35 = A->size[0] * A->size[1];
          A->size[0] = d_ndbl;
          A->size[1] = b_ndbl;
          emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
          for (i35 = 0; i35 < b_ndbl; i35++) {
            for (i36 = 0; i36 < d_ndbl; i36++) {
              A->data[i36 + A->size[0] * i35] = S_feature_data[i35 + b_ndbl *
                i36];
            }
          }

          xgeqp3(A, tau, jpvt);
          ar = rankFromQR(A);
          ia = A->size[1];
          for (i35 = 0; i35 < ia; i35++) {
            H_c_data[i35] = 0.0F;
          }

          for (i35 = 0; i35 < c_ndbl; i35++) {
            B_data[i35] = r->data[(int)c_xt[i35] - 1];
          }

          b_m = A->size[0];
          idx = A->size[0];
          mn = A->size[1];
          if (idx <= mn) {
            mn = idx;
          }

          for (ib = 0; ib + 1 <= mn; ib++) {
            if (tau->data[ib] != 0.0F) {
              wj = B_data[ib];
              for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                wj += A->data[ic + A->size[0] * ib] * B_data[ic];
              }

              wj *= tau->data[ib];
              if (wj != 0.0F) {
                B_data[ib] -= wj;
                for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                  B_data[ic] -= A->data[ic + A->size[0] * ib] * wj;
                }
              }
            }
          }

          for (ic = 0; ic + 1 <= ar; ic++) {
            H_c_data[jpvt->data[ic] - 1] = B_data[ic];
          }

          for (ib = ar - 1; ib + 1 > 0; ib--) {
            idx = jpvt->data[ib];
            H_c_data[jpvt->data[ib] - 1] /= A->data[ib + A->size[0] * ib];
            for (ic = 0; ic + 1 <= ib; ic++) {
              H_c_data[jpvt->data[ic] - 1] -= H_c_data[idx - 1] * A->data[ic +
                A->size[0] * ib];
            }
          }

          for (i35 = 0; i35 < ia; i35++) {
            i_c_data[i35] = H_c_data[i35];
          }
        }

        for (i35 = 0; i35 < ndbl; i35++) {
          H_c_data[i35] = r->data[(int)vec_data[i35] - 1];
        }

        if ((ia == 1) || (ndbl == 1)) {
          wj = 0.0F;
          for (i35 = 0; i35 < ia; i35++) {
            wj += i_c_data[i35] * H_c_data[i35];
          }

          innov = wj;
        } else {
          wj = 0.0F;
          for (i35 = 0; i35 < ia; i35++) {
            wj += i_c_data[i35] * H_c_data[i35];
          }

          innov = wj;
        }

        // 'OnePointRANSAC_EKF:222' if innov > mahalanobis_thresh
        if (innov > mahalanobis_thresh) {
          // 'OnePointRANSAC_EKF:223' r((k-1)*residualDim + (1:residualDim)) = 0; 
          f_c = k * residualDim;
          b_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          ar = b_ndbl + 1;
          idx = (b_ndbl - residualDim) + 1;
          if (fabs((double)idx) < 2.38418579E-7F * (float)residualDim) {
            b_ndbl++;
            ar = residualDim;
          } else if (idx > 0) {
            ar = b_ndbl;
          } else {
            b_ndbl++;
          }

          c_xt[0] = 1.0F;
          if (b_ndbl > 1) {
            c_xt[b_ndbl - 1] = (float)ar;
            i35 = b_ndbl - 1;
            idx = i35 / 2;
            mn = 1;
            while (mn <= idx - 1) {
              c_xt[1] = 2.0F;
              c_xt[b_ndbl - 2] = (float)ar - 1.0F;
              mn = 2;
            }

            if (idx << 1 == b_ndbl - 1) {
              c_xt[idx] = (1.0F + (float)ar) / 2.0F;
            } else {
              c_xt[idx] = 1.0F + (float)idx;
              c_xt[idx + 1] = (float)(ar - idx);
            }
          }

          for (i35 = 0; i35 < b_ndbl; i35++) {
            b_tmp_data[i35] = (int)((float)f_c + c_xt[i35]);
          }

          for (i35 = 0; i35 < b_ndbl; i35++) {
            r->data[b_tmp_data[i35] - 1] = 0.0F;
          }

          // 'OnePointRANSAC_EKF:224' H((k-1)*residualDim + (1:residualDim), :) = 0; 
          f_c = k * residualDim;
          b_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          ar = b_ndbl + 1;
          idx = (b_ndbl - residualDim) + 1;
          if (fabs((double)idx) < 2.38418579E-7F * (float)residualDim) {
            b_ndbl++;
            ar = residualDim;
          } else if (idx > 0) {
            ar = b_ndbl;
          } else {
            b_ndbl++;
          }

          c_xt[0] = 1.0F;
          if (b_ndbl > 1) {
            c_xt[b_ndbl - 1] = (float)ar;
            i35 = b_ndbl - 1;
            idx = i35 / 2;
            mn = 1;
            while (mn <= idx - 1) {
              c_xt[1] = 2.0F;
              c_xt[b_ndbl - 2] = (float)ar - 1.0F;
              mn = 2;
            }

            if (idx << 1 == b_ndbl - 1) {
              c_xt[idx] = (1.0F + (float)ar) / 2.0F;
            } else {
              c_xt[idx] = 1.0F + (float)idx;
              c_xt[idx + 1] = (float)(ar - idx);
            }
          }

          for (i35 = 0; i35 < b_ndbl; i35++) {
            tmp_data[i35] = (int)((float)f_c + c_xt[i35]);
          }

          for (i35 = 0; i35 < 102; i35++) {
            for (i36 = 0; i36 < b_ndbl; i36++) {
              b_H->data[(tmp_data[i36] + b_H->size[0] * i35) - 1] = 0.0F;
            }
          }

          // 'OnePointRANSAC_EKF:225' if it == VIOParameters.max_ekf_iterations
          if (it == c_VIOParameters_max_ekf_iterati) {
            // 'OnePointRANSAC_EKF:226' HI_inlierStatus(HI_ind(k)) = false;
            HI_inlierStatus[(int)out->data[k] - 1] = false;

            //  only reject the feature if its still bad in last iteration, otherwise just dont use for this update 
          }

          //                  log_info('rejecting %i', HI_ind(k))
          // 'OnePointRANSAC_EKF:229' if updateVect(HI_ind(k)) == 2
          if (updateVect[(int)out->data[k] - 1] == 2) {
            // 'OnePointRANSAC_EKF:230' log_error('inconsistency')
            b_log_error();
          }
        }
      }

      // 'OnePointRANSAC_EKF:235' S = (H*P*H');
      H_idx_0 = (unsigned int)b_H->size[0];
      i35 = k_c->size[0] * k_c->size[1];
      k_c->size[0] = (int)H_idx_0;
      k_c->size[1] = 102;
      emxEnsureCapacity((emxArray__common *)k_c, i35, (int)sizeof(float));
      b_m = b_H->size[0];
      i35 = k_c->size[0] * k_c->size[1];
      k_c->size[1] = 102;
      emxEnsureCapacity((emxArray__common *)k_c, i35, (int)sizeof(float));
      for (i35 = 0; i35 < 102; i35++) {
        loop_ub = k_c->size[0];
        for (i36 = 0; i36 < loop_ub; i36++) {
          k_c->data[i36 + k_c->size[0] * i35] = 0.0F;
        }
      }

      if (b_H->size[0] == 0) {
      } else {
        f_c = b_H->size[0] * 101;
        mn = 0;
        while ((b_m > 0) && (mn <= f_c)) {
          i35 = mn + b_m;
          for (ic = mn; ic + 1 <= i35; ic++) {
            k_c->data[ic] = 0.0F;
          }

          mn += b_m;
        }

        br = 0;
        mn = 0;
        while ((b_m > 0) && (mn <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 102; ib++) {
            if (b_P[ib] != 0.0F) {
              ia = ar;
              i35 = mn + b_m;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                k_c->data[ic] += b_P[ib] * b_H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 102;
          mn += b_m;
        }
      }

      i35 = c_H->size[0] * c_H->size[1];
      c_H->size[0] = b_H->size[1];
      c_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)c_H, i35, (int)sizeof(float));
      loop_ub = b_H->size[0];
      for (i35 = 0; i35 < loop_ub; i35++) {
        br = b_H->size[1];
        for (i36 = 0; i36 < br; i36++) {
          c_H->data[i36 + c_H->size[0] * i35] = b_H->data[i35 + b_H->size[0] *
            i36];
        }
      }

      mn = b_H->size[0];
      i35 = K->size[0] * K->size[1];
      K->size[0] = 102;
      K->size[1] = mn;
      emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
      for (i35 = 0; i35 < mn; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          K->data[i36 + K->size[0] * i35] = c_H->data[i36 + 102 * i35];
        }
      }

      H_idx_0 = (unsigned int)k_c->size[0];
      unnamed_idx_1 = (unsigned int)K->size[1];
      i35 = S->size[0] * S->size[1];
      S->size[0] = (int)H_idx_0;
      S->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)S, i35, (int)sizeof(float));
      b_m = k_c->size[0];
      i35 = S->size[0] * S->size[1];
      emxEnsureCapacity((emxArray__common *)S, i35, (int)sizeof(float));
      loop_ub = S->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        br = S->size[0];
        for (i36 = 0; i36 < br; i36++) {
          S->data[i36 + S->size[0] * i35] = 0.0F;
        }
      }

      if ((k_c->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        f_c = k_c->size[0] * (K->size[1] - 1);
        mn = 0;
        while ((b_m > 0) && (mn <= f_c)) {
          i35 = mn + b_m;
          for (ic = mn; ic + 1 <= i35; ic++) {
            S->data[ic] = 0.0F;
          }

          mn += b_m;
        }

        br = 0;
        mn = 0;
        while ((b_m > 0) && (mn <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 102; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              i35 = mn + b_m;
              for (ic = mn; ic + 1 <= i35; ic++) {
                ia++;
                S->data[ic] += K->data[ib] * k_c->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 102;
          mn += b_m;
        }
      }

      // 'OnePointRANSAC_EKF:236' size_S = num_HI_inlierCandidates*residualDim;
      size_S = (float)n * (float)residualDim;

      // 'OnePointRANSAC_EKF:237' S(1:(size_S+1):size_S^2) = S(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
      wj = size_S * size_S;
      if ((size_S + 1.0F == 0.0F) || (((size_S + 1.0F > 0.0F) && (1.0F > wj)) ||
           ((0.0F > size_S + 1.0F) && (wj > 1.0F)))) {
        i35 = 1;
        i36 = -1;
      } else {
        i35 = (int)(size_S + 1.0F);
        i36 = (int)wj - 1;
      }

      wj = size_S * size_S;
      if ((size_S + 1.0F == 0.0F) || (((size_S + 1.0F > 0.0F) && (1.0F > wj)) ||
           ((0.0F > size_S + 1.0F) && (wj > 1.0F)))) {
        ar = 1;
      } else {
        ar = (int)(size_S + 1.0F);
      }

      mn = c_S->size[0] * c_S->size[1];
      c_S->size[0] = 1;
      c_S->size[1] = div_s32_floor(i36, i35) + 1;
      emxEnsureCapacity((emxArray__common *)c_S, mn, (int)sizeof(float));
      loop_ub = div_s32_floor(i36, i35);
      for (i36 = 0; i36 <= loop_ub; i36++) {
        c_S->data[c_S->size[0] * i36] = S->data[i35 * i36] +
          noiseParameters_image_noise;
      }

      loop_ub = c_S->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        S->data[ar * i35] = c_S->data[c_S->size[0] * i35];
      }

      //  add R to HPH'
      // 'OnePointRANSAC_EKF:239' K = (P*H')/S;
      i35 = d_H->size[0] * d_H->size[1];
      d_H->size[0] = b_H->size[1];
      d_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)d_H, i35, (int)sizeof(float));
      loop_ub = b_H->size[0];
      for (i35 = 0; i35 < loop_ub; i35++) {
        br = b_H->size[1];
        for (i36 = 0; i36 < br; i36++) {
          d_H->data[i36 + d_H->size[0] * i35] = b_H->data[i35 + b_H->size[0] *
            i36];
        }
      }

      mn = b_H->size[0];
      i35 = K->size[0] * K->size[1];
      K->size[0] = 102;
      K->size[1] = mn;
      emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
      for (i35 = 0; i35 < mn; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          K->data[i36 + K->size[0] * i35] = d_H->data[i36 + 102 * i35];
        }
      }

      unnamed_idx_1 = (unsigned int)K->size[1];
      i35 = l_c->size[0] * l_c->size[1];
      l_c->size[0] = 102;
      l_c->size[1] = (int)unnamed_idx_1;
      l_c->size[0] = 102;
      emxEnsureCapacity((emxArray__common *)l_c, i35, (int)sizeof(float));
      loop_ub = l_c->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          l_c->data[i36 + l_c->size[0] * i35] = 0.0F;
        }
      }

      if (K->size[1] == 0) {
      } else {
        f_c = 102 * (K->size[1] - 1);
        for (mn = 0; mn <= f_c; mn += 102) {
          for (ic = mn + 1; ic <= mn + 102; ic++) {
            l_c->data[ic - 1] = 0.0F;
          }
        }

        br = 0;
        for (mn = 0; mn <= f_c; mn += 102) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 102; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              for (ic = mn; ic + 1 <= mn + 102; ic++) {
                ia++;
                l_c->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 102;
          }

          br += 102;
        }
      }

      if ((l_c->size[1] == 0) || ((S->size[0] == 0) || (S->size[1] == 0))) {
        unnamed_idx_1 = (unsigned int)S->size[0];
        i35 = c->size[0] * c->size[1];
        c->size[0] = 102;
        c->size[1] = (int)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)c, i35, (int)sizeof(float));
        loop_ub = 102 * (int)unnamed_idx_1;
        for (i35 = 0; i35 < loop_ub; i35++) {
          c->data[i35] = 0.0F;
        }
      } else if (S->size[0] == S->size[1]) {
        ar = S->size[1];
        i35 = A->size[0] * A->size[1];
        A->size[0] = S->size[0];
        A->size[1] = S->size[1];
        emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
        loop_ub = S->size[0] * S->size[1];
        for (i35 = 0; i35 < loop_ub; i35++) {
          A->data[i35] = S->data[i35];
        }

        xgetrf(S->size[1], S->size[1], A, S->size[1], jpvt, &idx);
        i35 = c->size[0] * c->size[1];
        c->size[0] = 102;
        c->size[1] = l_c->size[1];
        emxEnsureCapacity((emxArray__common *)c, i35, (int)sizeof(float));
        loop_ub = l_c->size[0] * l_c->size[1];
        for (i35 = 0; i35 < loop_ub; i35++) {
          c->data[i35] = l_c->data[i35];
        }

        for (ib = 0; ib + 1 <= ar; ib++) {
          idx = 102 * ib;
          br = ar * ib;
          for (k = 1; k <= ib; k++) {
            mn = 102 * (k - 1);
            if (A->data[(k + br) - 1] != 0.0F) {
              for (ic = 0; ic < 102; ic++) {
                c->data[ic + idx] -= A->data[(k + br) - 1] * c->data[ic + mn];
              }
            }
          }

          wj = 1.0F / A->data[ib + br];
          for (ic = 0; ic < 102; ic++) {
            c->data[ic + idx] *= wj;
          }
        }

        for (ib = S->size[1]; ib > 0; ib--) {
          idx = 102 * (ib - 1);
          br = ar * (ib - 1) - 1;
          for (k = ib + 1; k <= ar; k++) {
            mn = 102 * (k - 1);
            if (A->data[k + br] != 0.0F) {
              for (ic = 0; ic < 102; ic++) {
                c->data[ic + idx] -= A->data[k + br] * c->data[ic + mn];
              }
            }
          }
        }

        for (ar = S->size[1] - 2; ar + 1 > 0; ar--) {
          if (jpvt->data[ar] != ar + 1) {
            mn = jpvt->data[ar] - 1;
            for (idx = 0; idx < 102; idx++) {
              wj = c->data[idx + c->size[0] * ar];
              c->data[idx + c->size[0] * ar] = c->data[idx + c->size[0] * mn];
              c->data[idx + c->size[0] * mn] = wj;
            }
          }
        }
      } else {
        i35 = A->size[0] * A->size[1];
        A->size[0] = S->size[1];
        A->size[1] = S->size[0];
        emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
        loop_ub = S->size[0];
        for (i35 = 0; i35 < loop_ub; i35++) {
          br = S->size[1];
          for (i36 = 0; i36 < br; i36++) {
            A->data[i36 + A->size[0] * i35] = S->data[i35 + S->size[0] * i36];
          }
        }

        xgeqp3(A, tau, jpvt);
        ar = rankFromQR(A);
        ia = A->size[1];
        i35 = Y->size[0] * Y->size[1];
        Y->size[0] = ia;
        Y->size[1] = 102;
        emxEnsureCapacity((emxArray__common *)Y, i35, (int)sizeof(float));
        loop_ub = ia * 102;
        for (i35 = 0; i35 < loop_ub; i35++) {
          Y->data[i35] = 0.0F;
        }

        i35 = B->size[0] * B->size[1];
        B->size[0] = l_c->size[1];
        B->size[1] = 102;
        emxEnsureCapacity((emxArray__common *)B, i35, (int)sizeof(float));
        for (i35 = 0; i35 < 102; i35++) {
          loop_ub = l_c->size[1];
          for (i36 = 0; i36 < loop_ub; i36++) {
            B->data[i36 + B->size[0] * i35] = l_c->data[i35 + l_c->size[0] * i36];
          }
        }

        b_m = A->size[0];
        idx = A->size[0];
        mn = A->size[1];
        if (idx <= mn) {
          mn = idx;
        }

        for (ib = 0; ib + 1 <= mn; ib++) {
          if (tau->data[ib] != 0.0F) {
            for (k = 0; k < 102; k++) {
              wj = B->data[ib + B->size[0] * k];
              for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                wj += A->data[ic + A->size[0] * ib] * B->data[ic + B->size[0] *
                  k];
              }

              wj *= tau->data[ib];
              if (wj != 0.0F) {
                B->data[ib + B->size[0] * k] -= wj;
                for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                  B->data[ic + B->size[0] * k] -= A->data[ic + A->size[0] * ib] *
                    wj;
                }
              }
            }
          }
        }

        for (k = 0; k < 102; k++) {
          for (ic = 0; ic + 1 <= ar; ic++) {
            Y->data[(jpvt->data[ic] + Y->size[0] * k) - 1] = B->data[ic +
              B->size[0] * k];
          }

          for (ib = ar - 1; ib + 1 > 0; ib--) {
            Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] /= A->data[ib +
              A->size[0] * ib];
            for (ic = 0; ic + 1 <= ib; ic++) {
              Y->data[(jpvt->data[ic] + Y->size[0] * k) - 1] -= Y->data
                [(jpvt->data[ib] + Y->size[0] * k) - 1] * A->data[ic + A->size[0]
                * ib];
            }
          }
        }

        i35 = c->size[0] * c->size[1];
        c->size[0] = 102;
        c->size[1] = Y->size[0];
        emxEnsureCapacity((emxArray__common *)c, i35, (int)sizeof(float));
        loop_ub = Y->size[0];
        for (i35 = 0; i35 < loop_ub; i35++) {
          for (i36 = 0; i36 < 102; i36++) {
            c->data[i36 + c->size[0] * i35] = Y->data[i35 + Y->size[0] * i36];
          }
        }
      }

      i35 = b_K->size[0] * b_K->size[1];
      b_K->size[0] = 102;
      b_K->size[1] = c->size[1];
      emxEnsureCapacity((emxArray__common *)b_K, i35, (int)sizeof(float));
      loop_ub = c->size[0] * c->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        b_K->data[i35] = c->data[i35];
      }

      // 'OnePointRANSAC_EKF:241' x_it2 = K*(r + H * x_it);
      H_idx_0 = (unsigned int)b_H->size[0];
      i35 = m_c->size[0];
      m_c->size[0] = (int)H_idx_0;
      emxEnsureCapacity((emxArray__common *)m_c, i35, (int)sizeof(float));
      b_m = b_H->size[0];
      f_c = m_c->size[0];
      i35 = m_c->size[0];
      m_c->size[0] = f_c;
      emxEnsureCapacity((emxArray__common *)m_c, i35, (int)sizeof(float));
      for (i35 = 0; i35 < f_c; i35++) {
        m_c->data[i35] = 0.0F;
      }

      if (b_H->size[0] == 0) {
      } else {
        mn = 0;
        while ((b_m > 0) && (mn <= 0)) {
          for (ic = 1; ic <= b_m; ic++) {
            m_c->data[ic - 1] = 0.0F;
          }

          mn = b_m;
        }

        br = 0;
        mn = 0;
        while ((b_m > 0) && (mn <= 0)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 102; ib++) {
            if (x_it[ib] != 0.0F) {
              ia = ar;
              for (ic = 0; ic + 1 <= b_m; ic++) {
                ia++;
                m_c->data[ic] += x_it[ib] * b_H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 102;
          mn = b_m;
        }
      }

      i35 = r->size[0];
      emxEnsureCapacity((emxArray__common *)r, i35, (int)sizeof(float));
      loop_ub = r->size[0];
      for (i35 = 0; i35 < loop_ub; i35++) {
        r->data[i35] += m_c->data[i35];
      }

      if ((c->size[1] == 1) || (r->size[0] == 1)) {
        for (i35 = 0; i35 < 102; i35++) {
          x_it2_data[i35] = 0.0F;
          loop_ub = c->size[1];
          for (i36 = 0; i36 < loop_ub; i36++) {
            b_x_it2_data = x_it2_data[i35] + c->data[i35 + c->size[0] * i36] *
              r->data[i36];
            x_it2_data[i35] = b_x_it2_data;
          }
        }
      } else {
        k = c->size[1];
        memset(&x_it2_data[0], 0, 102U * sizeof(float));
        for (ic = 1; ic < 103; ic++) {
          x_it2_data[ic - 1] = 0.0F;
        }

        ar = 0;
        for (ib = 0; ib + 1 <= k; ib++) {
          if (r->data[ib] != 0.0F) {
            ia = ar;
            for (ic = 0; ic + 1 < 103; ic++) {
              ia++;
              x_it2_data[ic] += r->data[ib] * c->data[ia - 1];
            }
          }

          ar += 102;
        }
      }

      // 'OnePointRANSAC_EKF:242' x_it = x_it2(int32(1:(numStates + numAnchors*(6+numPointsPerAnchor)))); 
      memcpy(&x_it[0], &x_it2_data[0], 102U * sizeof(float));

      // 'OnePointRANSAC_EKF:244' xt_it.robot_state.pos           = xt.robot_state.pos       + x_it(1:3); 
      for (i35 = 0; i35 < 3; i35++) {
        x_it2[i35] = x_it2_data[i35];
      }

      mw_neon_mm_add_f32x4(b_xt->robot_state.pos, 3, 1, x_it2,
                           &xt_it.robot_state.pos[0]);

      // 'OnePointRANSAC_EKF:245' xt_it.robot_state.att           = quatmultJ(quatPlusThetaJ(x_it(4:6)), xt.robot_state.att); 
      // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
      for (i35 = 0; i35 < 3; i35++) {
        x_it2[i35] = x_it2_data[3 + i35];
      }

      theta = norm(x_it2) * 0.5F;

      // 'quatPlusThetaJ:3' if theta < 0.244
      if (theta < 0.244F) {
        // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
        for (i35 = 0; i35 < 3; i35++) {
          c_xt[i35] = 0.5F * x_it2_data[3 + i35];
        }

        c_xt[3] = 1.0F;
        for (i35 = 0; i35 < 4; i35++) {
          dq[i35] = c_xt[i35];
        }
      } else {
        // 'quatPlusThetaJ:5' else
        // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
        // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
        // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
        // 'quatPlusThetaJ:9'          cos(theta)];
        dq[0] = 0.5F * x_it2_data[3] * sinf(theta) / theta;
        dq[1] = 0.5F * x_it2_data[4] * sinf(theta) / theta;
        dq[2] = 0.5F * x_it2_data[5] * sinf(theta) / theta;
        dq[3] = cosf(theta);
      }

      // 'quatPlusThetaJ:11' dq = dq/norm(dq);
      wj = b_norm(dq);
      for (i35 = 0; i35 < 4; i35++) {
        dq[i35] /= wj;
      }

      quatmultJ(dq, b_xt->robot_state.att, xt_it.robot_state.att);

      // 'OnePointRANSAC_EKF:246' xt_it.robot_state.vel           = xt.robot_state.vel       + x_it(7:9); 
      for (i35 = 0; i35 < 3; i35++) {
        x_it2[i35] = x_it2_data[6 + i35];
      }

      mw_neon_mm_add_f32x4(b_xt->robot_state.vel, 3, 1, x_it2,
                           &xt_it.robot_state.vel[0]);

      // 'OnePointRANSAC_EKF:247' xt_it.robot_state.IMU.gyro_bias = xt.robot_state.IMU.gyro_bias + x_it(10:12); 
      for (i35 = 0; i35 < 3; i35++) {
        x_it2[i35] = x_it2_data[9 + i35];
      }

      mw_neon_mm_add_f32x4(b_xt->robot_state.IMU.gyro_bias, 3, 1, x_it2,
                           &xt_it.robot_state.IMU.gyro_bias[0]);

      // 'OnePointRANSAC_EKF:248' xt_it.robot_state.IMU.acc_bias  = xt.robot_state.IMU.acc_bias + x_it(13:15); 
      for (i35 = 0; i35 < 3; i35++) {
        x_it2[i35] = x_it2_data[12 + i35];
      }

      mw_neon_mm_add_f32x4(b_xt->robot_state.IMU.acc_bias, 3, 1, x_it2,
                           &xt_it.robot_state.IMU.acc_bias[0]);

      // 'OnePointRANSAC_EKF:249' xt_it.origin.att                = quatmultJ(quatPlusThetaJ(x_it(16:18)), xt.origin.att); 
      // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
      for (i35 = 0; i35 < 3; i35++) {
        x_it2[i35] = x_it2_data[15 + i35];
      }

      theta = norm(x_it2) * 0.5F;

      // 'quatPlusThetaJ:3' if theta < 0.244
      if (theta < 0.244F) {
        // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
        for (i35 = 0; i35 < 3; i35++) {
          c_xt[i35] = 0.5F * x_it2_data[15 + i35];
        }

        c_xt[3] = 1.0F;
        for (i35 = 0; i35 < 4; i35++) {
          dq[i35] = c_xt[i35];
        }
      } else {
        // 'quatPlusThetaJ:5' else
        // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
        // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
        // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
        // 'quatPlusThetaJ:9'          cos(theta)];
        dq[0] = 0.5F * x_it2_data[15] * sinf(theta) / theta;
        dq[1] = 0.5F * x_it2_data[16] * sinf(theta) / theta;
        dq[2] = 0.5F * x_it2_data[17] * sinf(theta) / theta;
        dq[3] = cosf(theta);
      }

      // 'quatPlusThetaJ:11' dq = dq/norm(dq);
      wj = b_norm(dq);
      for (i35 = 0; i35 < 4; i35++) {
        dq[i35] /= wj;
      }

      quatmultJ(dq, b_xt->origin.att, xt_it.origin.att);

      // 'OnePointRANSAC_EKF:251' meas_idx = 1;
      // 'OnePointRANSAC_EKF:252' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
        // 'OnePointRANSAC_EKF:253' xt_it.anchor_states(anchorIdx).pos = xt.anchor_states(anchorIdx).pos + x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        f_c = anchorIdx * 14 + 18;
        for (i35 = 0; i35 < 3; i35++) {
          x_it2[i35] = x_it2_data[i35 + f_c];
        }

        mw_neon_mm_add_f32x4(b_xt->anchor_states[anchorIdx].pos, 3, 1, x_it2,
                             &xt_it.anchor_states[anchorIdx].pos[0]);

        // 'OnePointRANSAC_EKF:254' xt_it.anchor_states(anchorIdx).att = quatmultJ(quatPlusThetaJ(x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att); 
        f_c = 18 + anchorIdx * 14;
        for (i35 = 0; i35 < 3; i35++) {
          fp[i35] = x_it2_data[(i35 + f_c) + 3];
        }

        // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
        theta = norm(fp) * 0.5F;

        // 'quatPlusThetaJ:3' if theta < 0.244
        if (theta < 0.244F) {
          // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
          for (ic = 0; ic < 3; ic++) {
            dq[ic] = 0.5F * fp[ic];
          }

          dq[3] = 1.0F;
        } else {
          // 'quatPlusThetaJ:5' else
          // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
          // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
          // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
          // 'quatPlusThetaJ:9'          cos(theta)];
          dq[0] = 0.5F * x_it2_data[f_c + 3] * sinf(theta) / theta;
          dq[1] = 0.5F * x_it2_data[f_c + 4] * sinf(theta) / theta;
          dq[2] = 0.5F * x_it2_data[f_c + 5] * sinf(theta) / theta;
          dq[3] = cosf(theta);
        }

        // 'quatPlusThetaJ:11' dq = dq/norm(dq);
        wj = b_norm(dq);
        for (i35 = 0; i35 < 4; i35++) {
          dq[i35] /= wj;
        }

        quatmultJ(dq, b_xt->anchor_states[anchorIdx].att,
                  xt_it.anchor_states[anchorIdx].att);

        // 'OnePointRANSAC_EKF:256' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'OnePointRANSAC_EKF:257' if xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (xt_it.anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:258' if HI_inlierStatus(xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
            if (HI_inlierStatus[xt_it.anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1]) {
              // 'OnePointRANSAC_EKF:259' xt_it.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
              xt_it.anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth = b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].inverse_depth + x_it2_data
                [(anchorIdx * 14 + featureIdx) + 24];

              // 'OnePointRANSAC_EKF:260' if it == VIOParameters.max_ekf_iterations && xt_it.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth > 10 
              if ((it == c_VIOParameters_max_ekf_iterati) &&
                  (xt_it.anchor_states[anchorIdx].feature_states[featureIdx].
                   inverse_depth > 10.0F)) {
                // 'OnePointRANSAC_EKF:261' log_warn('Feature %i is very close. Depth: %f', int32(xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), 1/xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth) 
                c_log_warn(xt_it.anchor_states[anchorIdx]
                           .feature_states[featureIdx].status_idx, 1.0F /
                           b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].inverse_depth);
              }

              // 'OnePointRANSAC_EKF:263' if xt_it.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
              if ((xt_it.anchor_states[anchorIdx].feature_states[featureIdx].
                   inverse_depth < 0.0F) && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                // 'OnePointRANSAC_EKF:264' if it == VIOParameters.max_ekf_iterations 
                //  only reject if we are done iterating
                // 'OnePointRANSAC_EKF:265' log_warn('Feature %i (%i on %i) is behind its anchor, rejecting', int32(xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), int32(featureIdx), int32(anchorIdx)) 
                log_warn(xt_it.anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx, featureIdx + 1,
                         anchorIdx + 1);

                // 'OnePointRANSAC_EKF:266' updateVect(xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
                updateVect[xt_it.anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;

                // 'OnePointRANSAC_EKF:267' xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
                xt_it.anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;

                // 'OnePointRANSAC_EKF:268' xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
                xt_it.anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            } else {
              if (HI_inlierCandidates[xt_it.anchor_states[anchorIdx].
                  feature_states[featureIdx].status_idx - 1] && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                // 'OnePointRANSAC_EKF:271' elseif HI_inlierCandidates(xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
                //  if it is not a HI inlier, but was a candidate, it was rejected by mahalanobis 
                // 'OnePointRANSAC_EKF:272' if it == VIOParameters.max_ekf_iterations 
                //  only reject if we are done iterating
                // 'OnePointRANSAC_EKF:273' updateVect(xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
                updateVect[xt_it.anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;

                // 'OnePointRANSAC_EKF:274' xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
                xt_it.anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;

                // 'OnePointRANSAC_EKF:275' xt_it.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
                xt_it.anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            }
          }

          // 'OnePointRANSAC_EKF:279' meas_idx = meas_idx + 1;
        }
      }

      it++;
    }

    emxFree_real32_T(&d_H);
    emxFree_real32_T(&c_S);
    emxFree_real32_T(&c_H);
    emxFree_real32_T(&m_c);
    emxFree_real32_T(&l_c);
    emxFree_real32_T(&k_c);

    // 'OnePointRANSAC_EKF:285' xt = xt_it;
    *b_xt = xt_it;

    // 'OnePointRANSAC_EKF:287' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
    if ((b_K->size[1] == 1) || (b_H->size[0] == 1)) {
      idx = b_K->size[0];
      loop_ub = b_K->size[0];
      for (i35 = 0; i35 < loop_ub; i35++) {
        br = b_H->size[1];
        for (i36 = 0; i36 < br; i36++) {
          s_c_data[i35 + idx * i36] = 0.0F;
          mn = b_K->size[1];
          for (ar = 0; ar < mn; ar++) {
            s_c_data[i35 + idx * i36] += b_K->data[i35 + b_K->size[0] * ar] *
              b_H->data[ar + b_H->size[0] * i36];
          }
        }
      }
    } else {
      k = b_K->size[1];
      H_idx_0 = (unsigned int)b_K->size[0];
      unnamed_idx_1 = (unsigned int)b_H->size[1];
      idx = (int)H_idx_0;
      b_m = b_K->size[0];
      loop_ub = (int)unnamed_idx_1;
      for (i35 = 0; i35 < loop_ub; i35++) {
        for (i36 = 0; i36 < idx; i36++) {
          s_c_data[i36 + (int)H_idx_0 * i35] = 0.0F;
        }
      }

      f_c = b_K->size[0] * (b_H->size[1] - 1);
      for (mn = 0; mn <= f_c; mn += b_m) {
        i35 = mn + b_m;
        for (ic = mn; ic + 1 <= i35; ic++) {
          s_c_data[ic] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= f_c; mn += b_m) {
        ar = 0;
        i35 = br + k;
        for (ib = br; ib + 1 <= i35; ib++) {
          if (b_H->data[ib] != 0.0F) {
            ia = ar;
            i36 = mn + b_m;
            for (ic = mn; ic + 1 <= i36; ic++) {
              ia++;
              s_c_data[ic] += b_H->data[ib] * b_K->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
      }
    }

    emxFree_real32_T(&b_H);
    emxFree_real32_T(&b_K);
    for (i35 = 0; i35 < 102; i35++) {
      for (i36 = 0; i36 < 102; i36++) {
        fv34[i36 + 102 * i35] = (float)iv13[i36 + 102 * i35] - s_c_data[i36 +
          102 * i35];
      }
    }

    for (i35 = 0; i35 < 102; i35++) {
      for (i36 = 0; i36 < 102; i36++) {
        fv37[i35 + 102 * i36] = 0.0F;
        for (ar = 0; ar < 102; ar++) {
          fv37[i35 + 102 * i36] += fv34[i35 + 102 * ar] * b_P[ar + 102 * i36];
        }
      }
    }

    for (i35 = 0; i35 < 102; i35++) {
      memcpy(&b_P[i35 * 102], &fv37[i35 * 102], 102U * sizeof(float));
    }
  }

  emxFree_real32_T(&c);
  emxFree_real32_T(&out);
  emxFree_int32_T(&ind);

  // % Update the delayed initialization features
  // 'OnePointRANSAC_EKF:292' if VIOParameters.delayed_initialization
  if (c_VIOParameters_delayed_initial) {
    emxInit_real32_T1(&n_c, 2);

    // 'OnePointRANSAC_EKF:293' xt = getScaledMap(xt);
    getScaledMap(b_xt);

    // 'OnePointRANSAC_EKF:294' [r, H] = getH_R_res(xt, z_u_l, z_u_r, delayedFeatures, stereoParams, VIOParameters); 
    b_getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, z_u_r, delayedFeatures,
                 c_stereoParams_CameraParameters,
                 d_stereoParams_CameraParameters,
                 e_stereoParams_CameraParameters,
                 f_stereoParams_CameraParameters, stereoParams_r_lr,
                 stereoParams_R_rl, VIOParameters_full_stereo, r, H);

    // 'OnePointRANSAC_EKF:296' S = (H*P*H');
    H_idx_0 = (unsigned int)H->size[0];
    i35 = n_c->size[0] * n_c->size[1];
    n_c->size[0] = (int)H_idx_0;
    n_c->size[1] = 102;
    emxEnsureCapacity((emxArray__common *)n_c, i35, (int)sizeof(float));
    b_m = H->size[0];
    i35 = n_c->size[0] * n_c->size[1];
    n_c->size[1] = 102;
    emxEnsureCapacity((emxArray__common *)n_c, i35, (int)sizeof(float));
    for (i35 = 0; i35 < 102; i35++) {
      loop_ub = n_c->size[0];
      for (i36 = 0; i36 < loop_ub; i36++) {
        n_c->data[i36 + n_c->size[0] * i35] = 0.0F;
      }
    }

    if (H->size[0] == 0) {
    } else {
      f_c = H->size[0] * 101;
      mn = 0;
      while ((b_m > 0) && (mn <= f_c)) {
        i35 = mn + b_m;
        for (ic = mn; ic + 1 <= i35; ic++) {
          n_c->data[ic] = 0.0F;
        }

        mn += b_m;
      }

      br = 0;
      mn = 0;
      while ((b_m > 0) && (mn <= f_c)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 102; ib++) {
          if (b_P[ib] != 0.0F) {
            ia = ar;
            i35 = mn + b_m;
            for (ic = mn; ic + 1 <= i35; ic++) {
              ia++;
              n_c->data[ic] += b_P[ib] * H->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 102;
        mn += b_m;
      }
    }

    i35 = K->size[0] * K->size[1];
    K->size[0] = 102;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
    loop_ub = H->size[0];
    for (i35 = 0; i35 < loop_ub; i35++) {
      for (i36 = 0; i36 < 102; i36++) {
        K->data[i36 + K->size[0] * i35] = H->data[i35 + H->size[0] * i36];
      }
    }

    H_idx_0 = (unsigned int)n_c->size[0];
    unnamed_idx_1 = (unsigned int)K->size[1];
    i35 = S->size[0] * S->size[1];
    S->size[0] = (int)H_idx_0;
    S->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)S, i35, (int)sizeof(float));
    b_m = n_c->size[0];
    i35 = S->size[0] * S->size[1];
    emxEnsureCapacity((emxArray__common *)S, i35, (int)sizeof(float));
    loop_ub = S->size[1];
    for (i35 = 0; i35 < loop_ub; i35++) {
      br = S->size[0];
      for (i36 = 0; i36 < br; i36++) {
        S->data[i36 + S->size[0] * i35] = 0.0F;
      }
    }

    if ((n_c->size[0] == 0) || (K->size[1] == 0)) {
    } else {
      f_c = n_c->size[0] * (K->size[1] - 1);
      mn = 0;
      while ((b_m > 0) && (mn <= f_c)) {
        i35 = mn + b_m;
        for (ic = mn; ic + 1 <= i35; ic++) {
          S->data[ic] = 0.0F;
        }

        mn += b_m;
      }

      br = 0;
      mn = 0;
      while ((b_m > 0) && (mn <= f_c)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 102; ib++) {
          if (K->data[ib] != 0.0F) {
            ia = ar;
            i35 = mn + b_m;
            for (ic = mn; ic + 1 <= i35; ic++) {
              ia++;
              S->data[ic] += K->data[ib] * n_c->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 102;
        mn += b_m;
      }
    }

    emxFree_real32_T(&n_c);

    // 'OnePointRANSAC_EKF:297' size_S = nnz(delayedFeatures)*residualDim;
    n = 0;
    for (k = 0; k < 48; k++) {
      if (delayedFeatures[k]) {
        n++;
      }
    }

    size_S = (float)n * (float)residualDim;

    // 'OnePointRANSAC_EKF:298' S(1:(size_S+1):size_S^2) = S(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
    wj = size_S * size_S;
    if ((size_S + 1.0F == 0.0F) || (((size_S + 1.0F > 0.0F) && (1.0F > wj)) ||
         ((0.0F > size_S + 1.0F) && (wj > 1.0F)))) {
      i35 = 1;
      i36 = -1;
    } else {
      i35 = (int)(size_S + 1.0F);
      i36 = (int)wj - 1;
    }

    wj = size_S * size_S;
    if ((size_S + 1.0F == 0.0F) || (((size_S + 1.0F > 0.0F) && (1.0F > wj)) ||
         ((0.0F > size_S + 1.0F) && (wj > 1.0F)))) {
      ar = 1;
    } else {
      ar = (int)(size_S + 1.0F);
    }

    emxInit_real32_T1(&d_S, 2);
    mn = d_S->size[0] * d_S->size[1];
    d_S->size[0] = 1;
    d_S->size[1] = div_s32_floor(i36, i35) + 1;
    emxEnsureCapacity((emxArray__common *)d_S, mn, (int)sizeof(float));
    loop_ub = div_s32_floor(i36, i35);
    for (i36 = 0; i36 <= loop_ub; i36++) {
      d_S->data[d_S->size[0] * i36] = S->data[i35 * i36] +
        noiseParameters_image_noise;
    }

    loop_ub = d_S->size[1];
    for (i35 = 0; i35 < loop_ub; i35++) {
      S->data[ar * i35] = d_S->data[d_S->size[0] * i35];
    }

    emxFree_real32_T(&d_S);

    //  add R to HPH'
    // 'OnePointRANSAC_EKF:299' K = (P*H')/S;
    i35 = K->size[0] * K->size[1];
    K->size[0] = 102;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
    loop_ub = H->size[0];
    for (i35 = 0; i35 < loop_ub; i35++) {
      for (i36 = 0; i36 < 102; i36++) {
        K->data[i36 + K->size[0] * i35] = H->data[i35 + H->size[0] * i36];
      }
    }

    emxInit_real32_T1(&o_c, 2);
    unnamed_idx_1 = (unsigned int)K->size[1];
    i35 = o_c->size[0] * o_c->size[1];
    o_c->size[0] = 102;
    o_c->size[1] = (int)unnamed_idx_1;
    o_c->size[0] = 102;
    emxEnsureCapacity((emxArray__common *)o_c, i35, (int)sizeof(float));
    loop_ub = o_c->size[1];
    for (i35 = 0; i35 < loop_ub; i35++) {
      for (i36 = 0; i36 < 102; i36++) {
        o_c->data[i36 + o_c->size[0] * i35] = 0.0F;
      }
    }

    if (K->size[1] == 0) {
    } else {
      f_c = 102 * (K->size[1] - 1);
      for (mn = 0; mn <= f_c; mn += 102) {
        for (ic = mn + 1; ic <= mn + 102; ic++) {
          o_c->data[ic - 1] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= f_c; mn += 102) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 102; ib++) {
          if (K->data[ib] != 0.0F) {
            ia = ar;
            for (ic = mn; ic + 1 <= mn + 102; ic++) {
              ia++;
              o_c->data[ic] += K->data[ib] * b_P[ia - 1];
            }
          }

          ar += 102;
        }

        br += 102;
      }
    }

    if ((o_c->size[1] == 0) || ((S->size[0] == 0) || (S->size[1] == 0))) {
      unnamed_idx_1 = (unsigned int)S->size[0];
      i35 = K->size[0] * K->size[1];
      K->size[0] = 102;
      K->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
      loop_ub = 102 * (int)unnamed_idx_1;
      for (i35 = 0; i35 < loop_ub; i35++) {
        K->data[i35] = 0.0F;
      }
    } else if (S->size[0] == S->size[1]) {
      n = S->size[1];
      i35 = A->size[0] * A->size[1];
      A->size[0] = S->size[0];
      A->size[1] = S->size[1];
      emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
      loop_ub = S->size[0] * S->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        A->data[i35] = S->data[i35];
      }

      xgetrf(S->size[1], S->size[1], A, S->size[1], jpvt, &idx);
      i35 = K->size[0] * K->size[1];
      K->size[0] = 102;
      K->size[1] = o_c->size[1];
      emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
      loop_ub = o_c->size[0] * o_c->size[1];
      for (i35 = 0; i35 < loop_ub; i35++) {
        K->data[i35] = o_c->data[i35];
      }

      for (ib = 0; ib + 1 <= n; ib++) {
        idx = 102 * ib;
        br = n * ib;
        for (k = 1; k <= ib; k++) {
          mn = 102 * (k - 1);
          if (A->data[(k + br) - 1] != 0.0F) {
            for (ic = 0; ic < 102; ic++) {
              K->data[ic + idx] -= A->data[(k + br) - 1] * K->data[ic + mn];
            }
          }
        }

        wj = 1.0F / A->data[ib + br];
        for (ic = 0; ic < 102; ic++) {
          K->data[ic + idx] *= wj;
        }
      }

      for (ib = S->size[1]; ib > 0; ib--) {
        idx = 102 * (ib - 1);
        br = n * (ib - 1) - 1;
        for (k = ib + 1; k <= n; k++) {
          mn = 102 * (k - 1);
          if (A->data[k + br] != 0.0F) {
            for (ic = 0; ic < 102; ic++) {
              K->data[ic + idx] -= A->data[k + br] * K->data[ic + mn];
            }
          }
        }
      }

      for (ar = S->size[1] - 2; ar + 1 > 0; ar--) {
        if (jpvt->data[ar] != ar + 1) {
          mn = jpvt->data[ar] - 1;
          for (idx = 0; idx < 102; idx++) {
            wj = K->data[idx + K->size[0] * ar];
            K->data[idx + K->size[0] * ar] = K->data[idx + K->size[0] * mn];
            K->data[idx + K->size[0] * mn] = wj;
          }
        }
      }
    } else {
      i35 = A->size[0] * A->size[1];
      A->size[0] = S->size[1];
      A->size[1] = S->size[0];
      emxEnsureCapacity((emxArray__common *)A, i35, (int)sizeof(float));
      loop_ub = S->size[0];
      for (i35 = 0; i35 < loop_ub; i35++) {
        br = S->size[1];
        for (i36 = 0; i36 < br; i36++) {
          A->data[i36 + A->size[0] * i35] = S->data[i35 + S->size[0] * i36];
        }
      }

      xgeqp3(A, tau, jpvt);
      ar = rankFromQR(A);
      ia = A->size[1];
      i35 = Y->size[0] * Y->size[1];
      Y->size[0] = ia;
      Y->size[1] = 102;
      emxEnsureCapacity((emxArray__common *)Y, i35, (int)sizeof(float));
      loop_ub = ia * 102;
      for (i35 = 0; i35 < loop_ub; i35++) {
        Y->data[i35] = 0.0F;
      }

      i35 = B->size[0] * B->size[1];
      B->size[0] = o_c->size[1];
      B->size[1] = 102;
      emxEnsureCapacity((emxArray__common *)B, i35, (int)sizeof(float));
      for (i35 = 0; i35 < 102; i35++) {
        loop_ub = o_c->size[1];
        for (i36 = 0; i36 < loop_ub; i36++) {
          B->data[i36 + B->size[0] * i35] = o_c->data[i35 + o_c->size[0] * i36];
        }
      }

      b_m = A->size[0];
      idx = A->size[0];
      mn = A->size[1];
      if (idx <= mn) {
        mn = idx;
      }

      for (ib = 0; ib + 1 <= mn; ib++) {
        if (tau->data[ib] != 0.0F) {
          for (k = 0; k < 102; k++) {
            wj = B->data[ib + B->size[0] * k];
            for (ic = ib + 1; ic + 1 <= b_m; ic++) {
              wj += A->data[ic + A->size[0] * ib] * B->data[ic + B->size[0] * k];
            }

            wj *= tau->data[ib];
            if (wj != 0.0F) {
              B->data[ib + B->size[0] * k] -= wj;
              for (ic = ib + 1; ic + 1 <= b_m; ic++) {
                B->data[ic + B->size[0] * k] -= A->data[ic + A->size[0] * ib] *
                  wj;
              }
            }
          }
        }
      }

      for (k = 0; k < 102; k++) {
        for (ic = 0; ic + 1 <= ar; ic++) {
          Y->data[(jpvt->data[ic] + Y->size[0] * k) - 1] = B->data[ic + B->size
            [0] * k];
        }

        for (ib = ar - 1; ib + 1 > 0; ib--) {
          Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] /= A->data[ib + A->
            size[0] * ib];
          for (ic = 0; ic + 1 <= ib; ic++) {
            Y->data[(jpvt->data[ic] + Y->size[0] * k) - 1] -= Y->data
              [(jpvt->data[ib] + Y->size[0] * k) - 1] * A->data[ic + A->size[0] *
              ib];
          }
        }
      }

      i35 = K->size[0] * K->size[1];
      K->size[0] = 102;
      K->size[1] = Y->size[0];
      emxEnsureCapacity((emxArray__common *)K, i35, (int)sizeof(float));
      loop_ub = Y->size[0];
      for (i35 = 0; i35 < loop_ub; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          K->data[i36 + K->size[0] * i35] = Y->data[i35 + Y->size[0] * i36];
        }
      }
    }

    emxFree_real32_T(&o_c);

    // 'OnePointRANSAC_EKF:301' x_it = K*r;
    if ((K->size[1] == 1) || (r->size[0] == 1)) {
      for (i35 = 0; i35 < 102; i35++) {
        x_it[i35] = 0.0F;
        loop_ub = K->size[1];
        for (i36 = 0; i36 < loop_ub; i36++) {
          b_x_it = x_it[i35] + K->data[i35 + K->size[0] * i36] * r->data[i36];
          x_it[i35] = b_x_it;
        }
      }
    } else {
      memset(&x_it[0], 0, 102U * sizeof(float));
      ar = 0;
      for (ib = 0; ib + 1 <= K->size[1]; ib++) {
        if (r->data[ib] != 0.0F) {
          ia = ar;
          for (ic = 0; ic < 102; ic++) {
            ia++;
            b_x_it = x_it[ic] + r->data[ib] * K->data[ia - 1];
            x_it[ic] = b_x_it;
          }
        }

        ar += 102;
      }
    }

    // 'OnePointRANSAC_EKF:303' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
      // 'OnePointRANSAC_EKF:304' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        // 'OnePointRANSAC_EKF:305' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          // 'OnePointRANSAC_EKF:306' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_it(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth += x_it[(anchorIdx * 14 + featureIdx) + 24];
        }
      }
    }

    // 'OnePointRANSAC_EKF:311' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      for (i35 = 0; i35 < 102; i35++) {
        for (i36 = 0; i36 < 102; i36++) {
          j_c[i35 + 102 * i36] = 0.0F;
          loop_ub = K->size[1];
          for (ar = 0; ar < loop_ub; ar++) {
            j_c[i35 + 102 * i36] += K->data[i35 + K->size[0] * ar] * H->data[ar
              + H->size[0] * i36];
          }
        }
      }
    } else {
      k = K->size[1];
      memset(&j_c[0], 0, 10404U * sizeof(float));
      for (mn = 0; mn <= 10303; mn += 102) {
        for (ic = mn; ic + 1 <= mn + 102; ic++) {
          j_c[ic] = 0.0F;
        }
      }

      br = 0;
      for (mn = 0; mn <= 10303; mn += 102) {
        ar = 0;
        i35 = br + k;
        for (ib = br; ib + 1 <= i35; ib++) {
          if (H->data[ib] != 0.0F) {
            ia = ar;
            for (ic = mn; ic + 1 <= mn + 102; ic++) {
              ia++;
              j_c[ic] += H->data[ib] * K->data[ia - 1];
            }
          }

          ar += 102;
        }

        br += k;
      }
    }

    for (i35 = 0; i35 < 10404; i35++) {
      fv34[i35] = iv13[i35];
    }

    mw_neon_mm_sub_f32x4(fv34, 102, 102, j_c, &fv35[0]);
    for (i35 = 0; i35 < 102; i35++) {
      for (i36 = 0; i36 < 102; i36++) {
        fv38[i35 + 102 * i36] = 0.0F;
        for (ar = 0; ar < 102; ar++) {
          fv38[i35 + 102 * i36] += fv35[i35 + 102 * ar] * b_P[ar + 102 * i36];
        }
      }
    }

    for (i35 = 0; i35 < 102; i35++) {
      memcpy(&b_P[i35 * 102], &fv38[i35 * 102], 102U * sizeof(float));
    }
  }

  emxFree_real32_T(&B);
  emxFree_int32_T(&jpvt);
  emxFree_real32_T(&tau);
  emxFree_real32_T(&A);
  emxFree_real32_T(&Y);
  emxFree_real32_T(&H);
  emxFree_real32_T(&r);
  emxFree_real32_T(&K);
  emxFree_real32_T(&S);

  // %
  // 'OnePointRANSAC_EKF:315' num_active_features_before = nnz(activeFeatures);
  n = 0;

  // 'OnePointRANSAC_EKF:316' num_active_features_after  = nnz(HI_inlierStatus | LI_inlier_status); 
  ar = 0;
  for (k = 0; k < 48; k++) {
    if (activeFeatures[k]) {
      n++;
    }

    if (HI_inlierStatus[k] || LI_inlier_status[k]) {
      ar++;
    }
  }

  // 'OnePointRANSAC_EKF:317' rejected_ratio = num_active_features_after/num_active_features_before; 
  rejected_ratio = (float)ar / (float)n;

  // 'OnePointRANSAC_EKF:319' if rejected_ratio < 0.1
  if (rejected_ratio < 0.1F) {
    //  if more than 90% were rejected
    // 'OnePointRANSAC_EKF:320' log_error('1-point RANSAC rejected %d%% of all features! Resetting velocity.', int32(100-rejected_ratio*100)) 
    // log_error Print to console in Matlab
    //  in C++, vio_logging.h needs to be created to define what LOG_ERROR does, 
    //  e.g. redefine ROS_ERROR
    // 'log_error:6' if coder.target('MATLAB')
    // 'log_error:8' elseif ~coder.target('MEX')
    // 'log_error:9' coder.cinclude('<vio_logging.h>')
    // 'log_error:10' coder.ceval('LOG_ERROR', [str, 0], varargin{:});
    memcpy(&cv48[0], &cv49[0], 66U * sizeof(char));
    wj = roundf(100.0F - rejected_ratio * 100.0F);
    if (wj < 2.14748365E+9F) {
      i35 = (int)wj;
    } else {
      i35 = MAX_int32_T;
    }

    LOG_ERROR(cv48, i35);

    // 'OnePointRANSAC_EKF:322' gryro_bias_cov = P(10:12, 10:12);
    // 'OnePointRANSAC_EKF:323' acc_bias_cov = P(13:15, 13:15);
    // 'OnePointRANSAC_EKF:324' R_ci_cov = P(19:21, 19:21);
    // 'OnePointRANSAC_EKF:326' R_cw = RotFromQuatJ(xt.robot_state.att)*RotFromQuatJ(xt.origin.att); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    d_xt[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
                b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
               b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
      b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
    d_xt[3] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                      b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
    d_xt[6] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                      b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
    d_xt[1] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                      b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
    d_xt[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
                b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
               b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
      b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
    d_xt[7] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                      b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
    d_xt[2] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                      b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
    d_xt[5] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                      b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
    d_xt[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
                b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
               b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
      b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
    e_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->origin.att[1] *
                b_xt->origin.att[1]) - b_xt->origin.att[2] * b_xt->origin.att[2])
      + b_xt->origin.att[3] * b_xt->origin.att[3];
    e_xt[3] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                      b_xt->origin.att[2] * b_xt->origin.att[3]);
    e_xt[6] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                      b_xt->origin.att[1] * b_xt->origin.att[3]);
    e_xt[1] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                      b_xt->origin.att[2] * b_xt->origin.att[3]);
    e_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) + b_xt->origin.att
                [1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
               b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->origin.att[3];
    e_xt[7] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                      b_xt->origin.att[0] * b_xt->origin.att[3]);
    e_xt[2] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                      b_xt->origin.att[1] * b_xt->origin.att[3]);
    e_xt[5] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                      b_xt->origin.att[0] * b_xt->origin.att[3]);
    e_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) - b_xt->origin.att
                [1] * b_xt->origin.att[1]) + b_xt->origin.att[2] *
               b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->origin.att[3];
    for (i35 = 0; i35 < 3; i35++) {
      for (i36 = 0; i36 < 3; i36++) {
        gryro_bias_cov[i36 + 3 * i35] = b_P[(i36 + 102 * (9 + i35)) + 9];
        acc_bias_cov[i36 + 3 * i35] = b_P[(i36 + 102 * (12 + i35)) + 12];
        R_ci_cov[i36 + 3 * i35] = b_P[(i36 + 102 * (18 + i35)) + 18];
        R_cw[i35 + 3 * i36] = 0.0F;
        for (ar = 0; ar < 3; ar++) {
          R_cw[i35 + 3 * i36] += d_xt[i35 + 3 * ar] * e_xt[ar + 3 * i36];
        }
      }
    }

    // 'OnePointRANSAC_EKF:328' P(:, :) = 0;
    for (i35 = 0; i35 < 10404; i35++) {
      b_P[i35] = 0.0F;
    }

    // 'OnePointRANSAC_EKF:330' P(  4:6,   4:6) = zeros(3);
    //  orientation of camera in origin frame
    // 'OnePointRANSAC_EKF:331' P(  7:9,   7:9) = 1*eye(3);
    //  velocity
    // 'OnePointRANSAC_EKF:332' P(10:12, 10:12) = gryro_bias_cov;
    //  gyro bias
    // 'OnePointRANSAC_EKF:333' P(13:15, 13:15) = acc_bias_cov;
    //  acc bias
    // 'OnePointRANSAC_EKF:334' P(16:18, 16:18) = 0.1*R_cw * diag([1 1 0]) * R_cw'; 
    for (i35 = 0; i35 < 3; i35++) {
      for (i36 = 0; i36 < 3; i36++) {
        b_P[(i36 + 102 * (3 + i35)) + 3] = 0.0F;
        b_P[(i36 + 102 * (6 + i35)) + 6] = iv14[i36 + 3 * i35];
        b_P[(i36 + 102 * (9 + i35)) + 9] = gryro_bias_cov[i36 + 3 * i35];
        b_P[(i36 + 102 * (12 + i35)) + 12] = acc_bias_cov[i36 + 3 * i35];
        d_xt[i35 + 3 * i36] = 0.0F;
        for (ar = 0; ar < 3; ar++) {
          d_xt[i35 + 3 * i36] += 0.1F * R_cw[i35 + 3 * ar] * (float)b[ar + 3 *
            i36];
        }
      }
    }

    for (i35 = 0; i35 < 3; i35++) {
      for (i36 = 0; i36 < 3; i36++) {
        b_P[(i35 + 102 * (15 + i36)) + 15] = 0.0F;
        for (ar = 0; ar < 3; ar++) {
          b_P[(i35 + 102 * (15 + i36)) + 15] += d_xt[i35 + 3 * ar] * R_cw[i36 +
            3 * ar];
        }
      }
    }

    //  origin orientation
    // 'OnePointRANSAC_EKF:335' P(19:21, 19:21) = R_ci_cov;
    for (i35 = 0; i35 < 3; i35++) {
      for (i36 = 0; i36 < 3; i36++) {
        b_P[(i36 + 102 * (18 + i35)) + 18] = R_ci_cov[i36 + 3 * i35];
      }
    }

    //  R_ci
    //  set all features inactive
    // 'OnePointRANSAC_EKF:338' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
      // 'OnePointRANSAC_EKF:339' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        // 'OnePointRANSAC_EKF:340' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status !=
            0) {
          // 'OnePointRANSAC_EKF:341' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
          updateVect[b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            status_idx - 1] = 0;

          // 'OnePointRANSAC_EKF:342' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;

          // 'OnePointRANSAC_EKF:343' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx =
            0;
        }
      }
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_free()
{
}

//
// Arguments    : void
// Return Type  : void
//
static void SLAM_init()
{
  initialized.size[1] = 0;
  initialized_not_empty = false;
}

//
// % Iterative Camera Pose optimization (EKF)
// Arguments    : float P_apr[10404]
//                g_struct_T *b_xt
//                int c_cameraParams_CameraParameters
//                const float d_cameraParams_CameraParameters[2]
//                const float e_cameraParams_CameraParameters[2]
//                const float f_cameraParams_CameraParameters[3]
//                int g_cameraParams_CameraParameters
//                int h_cameraParams_CameraParameters
//                const float i_cameraParams_CameraParameters[2]
//                const float j_cameraParams_CameraParameters[2]
//                const float k_cameraParams_CameraParameters[3]
//                int l_cameraParams_CameraParameters
//                const float cameraParams_r_lr[3]
//                const float cameraParams_R_lr[9]
//                const float cameraParams_R_rl[9]
//                int updateVect[48]
//                float z_all_l[96]
//                float z_all_r[96]
//                float noiseParameters_image_noise
//                float c_noiseParameters_inv_depth_ini
//                const VIOParameters b_VIOParameters
//                float b_map[144]
//                float b_delayedStatus[48]
// Return Type  : void
//
static void SLAM_upd(float P_apr[10404], g_struct_T *b_xt, int
                     c_cameraParams_CameraParameters, const float
                     d_cameraParams_CameraParameters[2], const float
                     e_cameraParams_CameraParameters[2], const float
                     f_cameraParams_CameraParameters[3], int
                     g_cameraParams_CameraParameters, int
                     h_cameraParams_CameraParameters, const float
                     i_cameraParams_CameraParameters[2], const float
                     j_cameraParams_CameraParameters[2], const float
                     k_cameraParams_CameraParameters[3], int
                     l_cameraParams_CameraParameters, const float
                     cameraParams_r_lr[3], const float cameraParams_R_lr[9],
                     const float cameraParams_R_rl[9], int updateVect[48], float
                     z_all_l[96], float z_all_r[96], float
                     noiseParameters_image_noise, float
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, float b_map[144], float b_delayedStatus[48])
{
  boolean_T x[48];
  int i;
  int idx;
  int ii_data[48];
  int yk;
  boolean_T exitg10;
  boolean_T guard3 = false;
  int loop_ub;
  emxArray_real32_T *ii;
  int i32;
  int ind_l2_size[1];
  float ind_l2_data[96];
  float z_all_l_data[96];
  int z_all_l_size[1];
  int status_ind_size[1];
  float status_ind_data[96];
  boolean_T exitg9;
  boolean_T guard2 = false;
  signed char i_data[48];
  emxArray_real32_T *qualities;
  int z_all_r_size[1];
  int anchorIdx;
  int featureIdx;
  int ix;
  boolean_T fix_new_feature;
  float uncertainties[8];
  signed char active_feature[8];
  int iidx[8];
  float mtmp;
  emxArray_real32_T *anchorInd;
  emxArray_real32_T *featureInd;
  emxArray_int32_T *b_iidx;
  float new_m_data[288];
  int c_triangulation_success_size_id;
  boolean_T triangulation_success_data[48];
  float fv25[2];
  float fv26[2];
  __attribute__((aligned(16))) float z_curr_l[2];
  float c;
  __attribute__((aligned(16))) float z_curr_r[2];
  boolean_T success;
  float b_m[6];
  __attribute__((aligned(16))) float fp[3];
  __attribute__((aligned(16))) float new_origin_pos_rel[3];
  boolean_T bv0[3];
  signed char i33;
  __attribute__((aligned(16))) float h_u_r[2];
  __attribute__((aligned(16))) float h_u_l[2];
  __attribute__((aligned(16))) float b_h_u_l[2];
  boolean_T guard1 = false;
  float triangulated_depths_data[48];
  float triangulated_status_ind_data[48];
  float b_triangulated_status_ind_data[48];
  int c_triangulated_status_ind_size_;
  int trueCount;
  float untriangulated_depths_data[48];
  emxArray_real32_T *auto_gen_tmp_4;
  emxArray_real32_T *out;
  int tmp_data[48];
  float untriangulated_status_ind_data[48];
  float b_new_m_data[144];
  int i34;
  float c_new_m_data[144];
  int new_feature_idx;
  boolean_T exitg7;
  float b_P_apr[36];
  int vec_data[82];
  int b_tmp_data[82];
  float c_tmp_data[492];
  float d_tmp_data[492];
  int b_vec_data[102];
  int e_tmp_data[102];
  float f_tmp_data[612];
  float g_tmp_data[612];
  boolean_T exitg8;
  float b_uncertainties[48];
  boolean_T has_active_features;
  int uncertainties_size[1];
  float totalNumActiveFeatues;
  float numDelayedFeatures;
  float delayedIdx;
  float numActivatedFeatures;
  boolean_T request_new_features;
  boolean_T exitg6;
  int request_idx;
  boolean_T exitg5;
  boolean_T b_guard1 = false;
  float b_has_active_features[6];
  float c_uncertainties[6];
  boolean_T exitg4;
  char cv46[111];
  static const char cv47[111] = { 'P', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'w', 'i', 't', 'h', ' ', 'n', 'o',
    ' ', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e',
    's', ' ', 'a', 's', ' ', 'o', 'r', 'i', 'g', 'i', 'n', ' ', '(', 'a', 'n',
    'c', 'h', 'o', 'r', ' ', '%', 'd', ')', '.', ' ', 'P', 'r', 'o', 'b', 'a',
    'b', 'l', 'y', ' ', 'b', 'e', 'c', 'a', 'u', 's', 'e', ' ', 'n', 'o', ' ',
    'a', 'n', 'c', 'h', 'o', 'r', 's', ' ', 'h', 'a', 'v', 'e', ' ', 'a', 'n',
    'y', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', 's', '.', '\x00' };

  float new_origin_att_rel[9];
  static float J[10404];
  static const signed char iv10[10404] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const signed char iv11[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 1 };

  __attribute__((aligned(16))) float fv27[3];
  float c_xt[9];
  float d_xt[9];
  float e_xt[9];
  float f_xt[9];
  float b_c[9];
  float varargin_1[4];
  boolean_T exitg3;
  static float b_J[10404];
  float g_xt[9];
  boolean_T exitg2;
  __attribute__((aligned(16))) float b_new_origin_att_rel[3];
  float h_xt[9];
  float i_xt[9];
  boolean_T exitg1;

  // 'SLAM_upd:5' numStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'SLAM_upd:6' numTrackFeatures = numAnchors*numPointsPerAnchor;
  //  undistort all valid points
  // 'SLAM_upd:9' ind_l = find(updateVect ~=0);
  for (i = 0; i < 48; i++) {
    x[i] = (updateVect[i] != 0);
  }

  idx = 0;
  yk = 1;
  exitg10 = false;
  while ((!exitg10) && (yk < 49)) {
    guard3 = false;
    if (x[yk - 1]) {
      idx++;
      ii_data[idx - 1] = yk;
      if (idx >= 48) {
        exitg10 = true;
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }

    if (guard3) {
      yk++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  emxInit_real32_T(&ii, 1);

  // 'SLAM_upd:10' ind_l2 = multiplyIdx(ind_l, 2);
  i32 = ii->size[0];
  ii->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)ii, i32, (int)sizeof(float));
  for (i32 = 0; i32 < loop_ub; i32++) {
    ii->data[i32] = (float)ii_data[i32];
  }

  multiplyIdx(ii->data, ii->size, ind_l2_data, ind_l2_size);

  // 'SLAM_upd:11' z_all_l(ind_l2) = undistortPoint(z_all_l(ind_l2), cameraParams.CameraParameters1); 
  z_all_l_size[0] = ind_l2_size[0];
  loop_ub = ind_l2_size[0];
  emxFree_real32_T(&ii);
  for (i32 = 0; i32 < loop_ub; i32++) {
    z_all_l_data[i32] = z_all_l[(int)ind_l2_data[i32] - 1];
  }

  undistortPoint(z_all_l_data, z_all_l_size, c_cameraParams_CameraParameters,
                 d_cameraParams_CameraParameters,
                 e_cameraParams_CameraParameters,
                 f_cameraParams_CameraParameters,
                 g_cameraParams_CameraParameters, status_ind_data,
                 status_ind_size);
  loop_ub = status_ind_size[0];
  for (i32 = 0; i32 < loop_ub; i32++) {
    z_all_l[(int)ind_l2_data[i32] - 1] = status_ind_data[i32];
  }

  // 'SLAM_upd:12' ind_r = find(updateVect == 2);
  for (i = 0; i < 48; i++) {
    x[i] = (updateVect[i] == 2);
  }

  idx = 0;
  yk = 1;
  exitg9 = false;
  while ((!exitg9) && (yk < 49)) {
    guard2 = false;
    if (x[yk - 1]) {
      idx++;
      ii_data[idx - 1] = yk;
      if (idx >= 48) {
        exitg9 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      yk++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  if (1 > idx) {
    yk = 0;
  } else {
    yk = idx;
  }

  for (i32 = 0; i32 < loop_ub; i32++) {
    i_data[i32] = (signed char)ii_data[i32];
  }

  emxInit_real32_T(&qualities, 1);
  i32 = qualities->size[0];
  qualities->size[0] = yk;
  emxEnsureCapacity((emxArray__common *)qualities, i32, (int)sizeof(float));
  for (i32 = 0; i32 < yk; i32++) {
    qualities->data[i32] = i_data[i32];
  }

  // 'SLAM_upd:13' if VIOParameters.full_stereo
  if (b_VIOParameters.full_stereo) {
    // 'SLAM_upd:14' z_all_r(ind_l2) = undistortPoint(z_all_r(ind_l2), cameraParams.CameraParameters2); 
    z_all_r_size[0] = ind_l2_size[0];
    yk = ind_l2_size[0];
    for (i32 = 0; i32 < yk; i32++) {
      z_all_l_data[i32] = z_all_r[(int)ind_l2_data[i32] - 1];
    }

    undistortPoint(z_all_l_data, z_all_r_size, h_cameraParams_CameraParameters,
                   i_cameraParams_CameraParameters,
                   j_cameraParams_CameraParameters,
                   k_cameraParams_CameraParameters,
                   l_cameraParams_CameraParameters, status_ind_data,
                   status_ind_size);
    yk = status_ind_size[0];
    for (i32 = 0; i32 < yk; i32++) {
      z_all_r[(int)ind_l2_data[i32] - 1] = status_ind_data[i32];
    }
  } else {
    // 'SLAM_upd:15' else
    // 'SLAM_upd:16' ind_r2 = multiplyIdx(ind_r, 2);
    multiplyIdx(qualities->data, qualities->size, ind_l2_data, ind_l2_size);

    // 'SLAM_upd:17' z_all_r(ind_r2) = undistortPoint(z_all_r(ind_r2), cameraParams.CameraParameters2); 
    z_all_r_size[0] = ind_l2_size[0];
    yk = ind_l2_size[0];
    for (i32 = 0; i32 < yk; i32++) {
      z_all_l_data[i32] = z_all_r[(int)ind_l2_data[i32] - 1];
    }

    undistortPoint(z_all_l_data, z_all_r_size, h_cameraParams_CameraParameters,
                   i_cameraParams_CameraParameters,
                   j_cameraParams_CameraParameters,
                   k_cameraParams_CameraParameters,
                   l_cameraParams_CameraParameters, status_ind_data,
                   status_ind_size);
    yk = status_ind_size[0];
    for (i32 = 0; i32 < yk; i32++) {
      z_all_r[(int)ind_l2_data[i32] - 1] = status_ind_data[i32];
    }
  }

  //  check for lost features
  // 'SLAM_upd:21' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'SLAM_upd:22' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'SLAM_upd:23' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
          && (updateVect[b_xt->anchor_states[anchorIdx]
              .feature_states[featureIdx].status_idx - 1] != 1)) {
        // 'SLAM_upd:24' idx = xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx; 
        // 'SLAM_upd:25' if updateVect(idx) ~= 1
        // 'SLAM_upd:26' P_apr(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, :) = 0; 
        ix = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx;
        for (i32 = 0; i32 < 102; i32++) {
          P_apr[(ix + 102 * i32) - 1] = 0.0F;
        }

        // 'SLAM_upd:27' P_apr(:, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = 0; 
        ix = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx;
        for (i32 = 0; i32 < 102; i32++) {
          P_apr[i32 + 102 * (ix - 1)] = 0.0F;
        }

        // 'SLAM_upd:28' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;

        // 'SLAM_upd:29' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx = 0;

        //                  log_info('Lost feature %i, which was %i on anchor %i', idx, featureIdx, anchorIdx) 
      }
    }
  }

  // 'SLAM_upd:36' if VIOParameters.fixed_feature
  if (b_VIOParameters.fixed_feature) {
    // 'SLAM_upd:37' fix_new_feature = false;
    fix_new_feature = false;

    // 'SLAM_upd:39' if xt.origin.anchor_idx
    if (b_xt->origin.anchor_idx != 0) {
      // 'SLAM_upd:40' if xt.fixed_feature
      if (b_xt->fixed_feature != 0) {
        // 'SLAM_upd:41' if xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status ~= 1 
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
            [b_xt->fixed_feature - 1].status != 1) {
          // 'SLAM_upd:42' fix_new_feature = true;
          fix_new_feature = true;

          // 'SLAM_upd:43' log_info('Fixed feature %i (%i on anchor %i) is no longer valid', xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status_idx, xt.fixed_feature, xt.origin.anchor_idx) 
          log_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                   feature_states[b_xt->fixed_feature - 1].status_idx,
                   b_xt->fixed_feature, b_xt->origin.anchor_idx);
        }
      } else {
        // 'SLAM_upd:45' else
        // 'SLAM_upd:46' fix_new_feature = true;
        fix_new_feature = true;
      }
    }

    // 'SLAM_upd:50' if fix_new_feature
    if (fix_new_feature) {
      // 'SLAM_upd:51' uncertainties = zeros(numPointsPerAnchor, 1);
      // 'SLAM_upd:52' active_feature = uncertainties;
      // 'SLAM_upd:53' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        active_feature[featureIdx] = 0;

        // 'SLAM_upd:54' if xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).status == 1 
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
            .feature_states[featureIdx].status == 1) {
          // 'SLAM_upd:55' uncertainties(featureIdx) = P_apr(xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).P_idx, xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).P_idx); 
          uncertainties[featureIdx] = P_apr[(b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].feature_states[featureIdx].P_idx + 102
            * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
               .feature_states[featureIdx].P_idx - 1)) - 1];

          // 'SLAM_upd:56' active_feature(featureIdx) = 1;
          active_feature[featureIdx] = 1;
        } else {
          // 'SLAM_upd:57' else
          // 'SLAM_upd:58' uncertainties(featureIdx) = 1000;
          uncertainties[featureIdx] = 1000.0F;

          //  dont fix an inactive feature
        }
      }

      // 'SLAM_upd:61' [~, sortIdx] = sort(uncertainties, 'ascend');
      sort(uncertainties, iidx);
      for (i = 0; i < 8; i++) {
        uncertainties[i] = (float)iidx[i];
      }

      // 'SLAM_upd:61' ~
      // 'SLAM_upd:62' if ~active_feature(sortIdx(1))
      if (!(active_feature[(int)uncertainties[0] - 1] != 0)) {
        // 'SLAM_upd:63' log_error('picked an inactive feature')
        log_error();
      }

      // 'SLAM_upd:65' xt.fixed_feature = int32(sortIdx(1));
      mtmp = roundf(uncertainties[0]);
      if (mtmp < 2.14748365E+9F) {
        i32 = (int)mtmp;
      } else {
        i32 = MAX_int32_T;
      }

      b_xt->fixed_feature = i32;

      // 'SLAM_upd:66' P_apr(xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).P_idx, :) = 0; 
      ix = b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
        [b_xt->fixed_feature - 1].P_idx;
      for (i32 = 0; i32 < 102; i32++) {
        P_apr[(ix + 102 * i32) - 1] = 0.0F;
      }

      //  fix the feature depth
      // 'SLAM_upd:67' P_apr(:, xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).P_idx) = 0; 
      ix = b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
        [b_xt->fixed_feature - 1].P_idx;
      for (i32 = 0; i32 < 102; i32++) {
        P_apr[i32 + 102 * (ix - 1)] = 0.0F;
      }

      // 'SLAM_upd:68' log_info('Fixing feature %i (feature %i on anchor %i)', xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status_idx, xt.fixed_feature, xt.origin.anchor_idx) 
      b_log_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                 .feature_states[b_xt->fixed_feature - 1].status_idx,
                 b_xt->fixed_feature, b_xt->origin.anchor_idx);
    }
  }

  // % do the update
  // 'SLAM_upd:73' if any(updateVect == 1)
  for (i = 0; i < 48; i++) {
    x[i] = (updateVect[i] == 1);
  }

  if (any(x)) {
    // 'SLAM_upd:74' [ xt, P_apo, updateVect ] = OnePointRANSAC_EKF(xt, P_apr, z_all_l, z_all_r, cameraParams, noiseParameters, VIOParameters, updateVect); 
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, z_all_r,
                       d_cameraParams_CameraParameters,
                       e_cameraParams_CameraParameters,
                       i_cameraParams_CameraParameters,
                       j_cameraParams_CameraParameters, cameraParams_r_lr,
                       cameraParams_R_rl, noiseParameters_image_noise,
                       b_VIOParameters.max_ekf_iterations,
                       b_VIOParameters.delayed_initialization,
                       b_VIOParameters.full_stereo, b_VIOParameters.RANSAC,
                       updateVect);
  } else {
    // 'SLAM_upd:75' else
    // 'SLAM_upd:76' P_apo = P_apr;
  }

  // % Initialize new anchors/features
  // 'SLAM_upd:80' if length(ind_r) >= minFeatureThreshold
  emxInit_real32_T(&anchorInd, 1);
  emxInit_real32_T(&featureInd, 1);
  emxInit_int32_T(&b_iidx, 1);
  if (loop_ub >= 4) {
    //  try to triangulate all new features
    // 'SLAM_upd:82' new_depths = zeros(length(ind_r), 1);
    for (i32 = 0; i32 < loop_ub; i32++) {
      ind_l2_data[i32] = 0.0F;
    }

    // 'SLAM_upd:83' new_m = zeros(3, length(ind_r));
    loop_ub *= 3;
    for (i32 = 0; i32 < loop_ub; i32++) {
      new_m_data[i32] = 0.0F;
    }

    // 'SLAM_upd:84' triangulation_success = false(length(ind_r), 1);
    c_triangulation_success_size_id = qualities->size[0];
    loop_ub = qualities->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      triangulation_success_data[i32] = false;
    }

    // 'SLAM_upd:86' for i = 1:length(ind_r)
    for (i = 0; i < qualities->size[0]; i++) {
      // 'SLAM_upd:87' z_curr_l = z_all_l((ind_r(i)-1)*2 + (1:2));
      mtmp = (qualities->data[i] - 1.0F) * 2.0F;
      for (i32 = 0; i32 < 2; i32++) {
        fv25[i32] = 1.0F + (float)i32;
      }

      ne10_addc_float_neon(&fv26[0], fv25, mtmp, 2U);
      for (i32 = 0; i32 < 2; i32++) {
        z_curr_l[i32] = z_all_l[(int)fv26[i32] - 1];
      }

      // 'SLAM_upd:88' z_curr_r = z_all_r((ind_r(i)-1)*2 + (1:2));
      c = (qualities->data[i] - 1.0F) * 2.0F;
      for (i32 = 0; i32 < 2; i32++) {
        fv25[i32] = 1.0F + (float)i32;
      }

      ne10_addc_float_neon(&fv26[0], fv25, c, 2U);
      for (i32 = 0; i32 < 2; i32++) {
        z_curr_r[i32] = z_all_r[(int)fv26[i32] - 1];
      }

      // 'SLAM_upd:89' if ~VIOParameters.mono
      if (!b_VIOParameters.mono) {
        // 'SLAM_upd:90' [ fp, m, success ] = initializePoint(z_curr_l, z_curr_r, cameraParams); 
        initializePoint(z_curr_l, z_curr_r, d_cameraParams_CameraParameters,
                        e_cameraParams_CameraParameters,
                        i_cameraParams_CameraParameters,
                        j_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, fp, b_m, &success);

        // 'SLAM_upd:91' m_l = m(:,1);
        for (i32 = 0; i32 < 3; i32++) {
          new_origin_pos_rel[i32] = b_m[i32];
        }

        // 'SLAM_upd:93' if success
        if (success) {
          //  perform further checks
          // 'SLAM_upd:94' if any(isnan(fp))
          for (yk = 0; yk < 3; yk++) {
            bv0[yk] = rtIsNaNF(fp[yk]);
          }

          if (b_any(bv0)) {
            // 'SLAM_upd:95' log_warn('Bad triangulation (nan) for point %d', int8(ind_r(i))); 
            mtmp = roundf(qualities->data[i]);
            if (mtmp < 128.0F) {
              if (mtmp >= -128.0F) {
                i33 = (signed char)mtmp;
              } else {
                i33 = MIN_int8_T;
              }
            } else if (mtmp >= 128.0F) {
              i33 = MAX_int8_T;
            } else {
              i33 = 0;
            }

            d_log_warn(i33);

            // 'SLAM_upd:96' fp = m_l;
            for (i32 = 0; i32 < 3; i32++) {
              fp[i32] = b_m[i32];
            }

            // 'SLAM_upd:97' success = false;
            success = false;
          } else {
            // 'SLAM_upd:98' else
            //  check reprojection error
            // 'SLAM_upd:100' [h_u_l, h_u_r] = predictMeasurementStereo(fp, cameraParams); 
            predictMeasurementStereo(fp, d_cameraParams_CameraParameters,
              e_cameraParams_CameraParameters, i_cameraParams_CameraParameters,
              j_cameraParams_CameraParameters, cameraParams_r_lr,
              cameraParams_R_rl, h_u_l, h_u_r);

            // 'SLAM_upd:102' reprojection_error_thresh = 5;
            // 'SLAM_upd:103' if norm(h_u_l - z_curr_l) > reprojection_error_thresh || norm(h_u_r - z_curr_r) > reprojection_error_thresh 
            mw_neon_mm_sub_f32x4(h_u_l, 2, 1, z_curr_l, &b_h_u_l[0]);
            guard1 = false;
            if (c_norm(b_h_u_l) > 5.0F) {
              guard1 = true;
            } else {
              mw_neon_mm_sub_f32x4(h_u_r, 2, 1, z_curr_r, &b_h_u_l[0]);
              if (c_norm(b_h_u_l) > 5.0F) {
                guard1 = true;
              } else {
                // 'SLAM_upd:107' else
                // 'SLAM_upd:108' if norm(fp) < 0.1
                if (norm(fp) < 0.1F) {
                  //  feature triangulated very close
                  // 'SLAM_upd:109' log_warn('Feature %i is triangulated very close. Depth: %f', int32(ind_r(i)), norm(fp)); 
                  mtmp = roundf(qualities->data[i]);
                  if (mtmp < 2.14748365E+9F) {
                    if (mtmp >= -2.14748365E+9F) {
                      i32 = (int)mtmp;
                    } else {
                      i32 = MIN_int32_T;
                    }
                  } else if (mtmp >= 2.14748365E+9F) {
                    i32 = MAX_int32_T;
                  } else {
                    i32 = 0;
                  }

                  e_log_warn(i32, norm(fp));

                  // 'SLAM_upd:110' fp = m_l;
                  for (i32 = 0; i32 < 3; i32++) {
                    fp[i32] = b_m[i32];
                  }

                  // 'SLAM_upd:111' success = false;
                  success = false;
                }
              }
            }

            if (guard1) {
              //                          log_info('Bad triangulation (reprojection error) for point %d', int8(ind_r(i))); 
              // 'SLAM_upd:105' fp = m_l;
              for (i32 = 0; i32 < 3; i32++) {
                fp[i32] = b_m[i32];
              }

              // 'SLAM_upd:106' success = false;
              success = false;
            }
          }
        } else {
          // 'SLAM_upd:115' else
          // 'SLAM_upd:116' fp = m_l;
          for (i32 = 0; i32 < 3; i32++) {
            fp[i32] = b_m[i32];
          }
        }
      } else {
        // 'SLAM_upd:118' else
        //  mono
        // 'SLAM_upd:119' z_n_l = [(z_curr_l(1) - cameraParams.CameraParameters1.PrincipalPoint(1))/cameraParams.CameraParameters1.FocalLength(1); 
        // 'SLAM_upd:120'                 (z_curr_l(2) - cameraParams.CameraParameters1.PrincipalPoint(2))/cameraParams.CameraParameters1.FocalLength(2)]; 
        // 'SLAM_upd:121' m_l = [z_n_l; 1];
        for (i32 = 0; i32 < 2; i32++) {
          fv25[i32] = 1.0F + (float)i32;
        }

        ne10_addc_float_neon(&fv26[0], fv25, mtmp, 2U);
        for (i32 = 0; i32 < 2; i32++) {
          fv25[i32] = 1.0F + (float)i32;
        }

        ne10_addc_float_neon(&b_h_u_l[0], fv25, mtmp, 2U);
        new_origin_pos_rel[0] = (z_all_l[(int)fv26[0] - 1] -
          e_cameraParams_CameraParameters[0]) / d_cameraParams_CameraParameters
          [0];
        new_origin_pos_rel[1] = (z_all_l[(int)b_h_u_l[1] - 1] -
          e_cameraParams_CameraParameters[1]) / d_cameraParams_CameraParameters
          [1];
        new_origin_pos_rel[2] = 1.0F;

        // 'SLAM_upd:122' fp = 2*m_l/norm(m_l);
        mtmp = norm(new_origin_pos_rel);
        for (yk = 0; yk < 3; yk++) {
          fp[yk] = 2.0F * new_origin_pos_rel[yk] / mtmp;
        }

        // 'SLAM_upd:123' success = true;
        success = true;
      }

      // 'SLAM_upd:125' new_depths(i) = norm(fp);
      ind_l2_data[i] = norm(fp);

      // 'SLAM_upd:126' new_m(:, i) = m_l;
      for (i32 = 0; i32 < 3; i32++) {
        new_m_data[i32 + 3 * i] = new_origin_pos_rel[i32];
      }

      // 'SLAM_upd:127' triangulation_success(i) = success;
      triangulation_success_data[i] = success;
    }

    // 'SLAM_upd:130' log_info('Successfully triangulated %d of %d features', int32(nnz(triangulation_success)), int32(length(triangulation_success))) 
    ix = 0;
    for (idx = 0; idx < c_triangulation_success_size_id; idx++) {
      if (triangulation_success_data[idx]) {
        ix++;
      }
    }

    c_log_info(ix, c_triangulation_success_size_id);

    // 'SLAM_upd:132' triangulated_depths = new_depths(triangulation_success);
    idx = c_triangulation_success_size_id - 1;
    ix = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        ix++;
      }
    }

    yk = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_depths_data[yk] = ind_l2_data[i];
        yk++;
      }
    }

    // 'SLAM_upd:133' [triangulated_depths, triangulated_idx] = sort(triangulated_depths, 'ascend'); 
    i32 = anchorInd->size[0];
    anchorInd->size[0] = ix;
    emxEnsureCapacity((emxArray__common *)anchorInd, i32, (int)sizeof(float));
    for (i32 = 0; i32 < ix; i32++) {
      anchorInd->data[i32] = triangulated_depths_data[i32];
    }

    c_sort(anchorInd, b_iidx);
    i32 = featureInd->size[0];
    featureInd->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)featureInd, i32, (int)sizeof(float));
    loop_ub = b_iidx->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      featureInd->data[i32] = (float)b_iidx->data[i32];
    }

    // 'SLAM_upd:134' triangulated_m = new_m(:, triangulation_success);
    idx = c_triangulation_success_size_id - 1;
    ix = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        ix++;
      }
    }

    yk = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        ii_data[yk] = i + 1;
        yk++;
      }
    }

    // 'SLAM_upd:135' triangulated_m = triangulated_m(:, triangulated_idx);
    // 'SLAM_upd:136' triangulated_status_ind = ind_r(triangulation_success);
    idx = c_triangulation_success_size_id - 1;
    yk = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_status_ind_data[yk] = qualities->data[i];
        yk++;
      }
    }

    // 'SLAM_upd:137' triangulated_status_ind = triangulated_status_ind(triangulated_idx); 
    c_triangulated_status_ind_size_ = featureInd->size[0];
    loop_ub = featureInd->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      b_triangulated_status_ind_data[i32] = triangulated_status_ind_data[(int)
        featureInd->data[i32] - 1];
    }

    for (i32 = 0; i32 < c_triangulated_status_ind_size_; i32++) {
      triangulated_status_ind_data[i32] = b_triangulated_status_ind_data[i32];
    }

    // 'SLAM_upd:139' untriangulated_depths = new_depths(~triangulation_success); 
    idx = c_triangulation_success_size_id - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        trueCount++;
      }
    }

    yk = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_depths_data[yk] = ind_l2_data[i];
        yk++;
      }
    }

    emxInit_real32_T(&auto_gen_tmp_4, 1);

    // 'SLAM_upd:140' [untriangulated_depths, untriangulated_idx] = sort(untriangulated_depths, 'ascend'); 
    i32 = auto_gen_tmp_4->size[0];
    auto_gen_tmp_4->size[0] = trueCount;
    emxEnsureCapacity((emxArray__common *)auto_gen_tmp_4, i32, (int)sizeof(float));
    for (i32 = 0; i32 < trueCount; i32++) {
      auto_gen_tmp_4->data[i32] = untriangulated_depths_data[i32];
    }

    emxInit_real32_T(&out, 1);
    c_sort(auto_gen_tmp_4, b_iidx);
    i32 = out->size[0];
    out->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)out, i32, (int)sizeof(float));
    loop_ub = b_iidx->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      out->data[i32] = (float)b_iidx->data[i32];
    }

    // 'SLAM_upd:141' untriangulated_m = new_m(:, ~triangulation_success);
    idx = c_triangulation_success_size_id - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        trueCount++;
      }
    }

    yk = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        tmp_data[yk] = i + 1;
        yk++;
      }
    }

    // 'SLAM_upd:142' untriangulated_m = untriangulated_m(:, untriangulated_idx); 
    // 'SLAM_upd:143' untriangulated_status_ind = ind_r(~triangulation_success); 
    idx = c_triangulation_success_size_id - 1;
    yk = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_status_ind_data[yk] = qualities->data[i];
        yk++;
      }
    }

    // 'SLAM_upd:144' untriangulated_status_ind = untriangulated_status_ind(untriangulated_idx); 
    idx = out->size[0];
    loop_ub = out->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      b_triangulated_status_ind_data[i32] = untriangulated_status_ind_data[(int)
        out->data[i32] - 1];
    }

    for (i32 = 0; i32 < idx; i32++) {
      untriangulated_status_ind_data[i32] = b_triangulated_status_ind_data[i32];
    }

    // 'SLAM_upd:146' new_depths = [triangulated_depths; untriangulated_depths]; 
    ind_l2_size[0] = anchorInd->size[0] + auto_gen_tmp_4->size[0];
    loop_ub = anchorInd->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      ind_l2_data[i32] = anchorInd->data[i32];
    }

    loop_ub = auto_gen_tmp_4->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      ind_l2_data[i32 + anchorInd->size[0]] = auto_gen_tmp_4->data[i32];
    }

    emxFree_real32_T(&auto_gen_tmp_4);

    // 'SLAM_upd:147' new_m = [triangulated_m, untriangulated_m];
    for (i32 = 0; i32 < ix; i32++) {
      for (i34 = 0; i34 < 3; i34++) {
        b_new_m_data[i34 + 3 * i32] = new_m_data[i34 + 3 * (ii_data[i32] - 1)];
      }
    }

    for (i32 = 0; i32 < trueCount; i32++) {
      for (i34 = 0; i34 < 3; i34++) {
        c_new_m_data[i34 + 3 * i32] = new_m_data[i34 + 3 * (tmp_data[i32] - 1)];
      }
    }

    loop_ub = featureInd->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      for (i34 = 0; i34 < 3; i34++) {
        new_m_data[i34 + 3 * i32] = b_new_m_data[i34 + 3 * ((int)
          featureInd->data[i32] - 1)];
      }
    }

    loop_ub = out->size[0];
    for (i32 = 0; i32 < loop_ub; i32++) {
      for (i34 = 0; i34 < 3; i34++) {
        new_m_data[i34 + 3 * (i32 + featureInd->size[0])] = c_new_m_data[i34 + 3
          * ((int)out->data[i32] - 1)];
      }
    }

    emxFree_real32_T(&out);

    // 'SLAM_upd:148' status_ind = [triangulated_status_ind; untriangulated_status_ind]; 
    for (i32 = 0; i32 < c_triangulated_status_ind_size_; i32++) {
      status_ind_data[i32] = triangulated_status_ind_data[i32];
    }

    for (i32 = 0; i32 < idx; i32++) {
      status_ind_data[i32 + c_triangulated_status_ind_size_] =
        untriangulated_status_ind_data[i32];
    }

    //  insert new features into the state. first the closest triangulated ones, 
    //  then the un-triangulated ones
    // 'SLAM_upd:152' new_feature_idx = 1;
    new_feature_idx = 1;

    // 'SLAM_upd:153' for anchorIdx = 1:numAnchors
    anchorIdx = 0;
    exitg7 = false;
    while ((!exitg7) && (anchorIdx + 1 < 7)) {
      //          if new_feature_idx > length(new_depths)
      // 'SLAM_upd:155' if new_feature_idx > nnz(triangulation_success)
      ix = 0;
      for (idx = 0; idx < c_triangulation_success_size_id; idx++) {
        if (triangulation_success_data[idx]) {
          ix++;
        }
      }

      if (((float)new_feature_idx > ix) || ((float)ind_l2_size[0] - ((float)
            new_feature_idx - 1.0F) < 4.0F)) {
        exitg7 = true;
      } else {
        // 'SLAM_upd:158' if length(new_depths) - (new_feature_idx -1) < minFeatureThreshold 
        // 'SLAM_upd:161' if getNumValidFeatures(xt.anchor_states(anchorIdx)) < minFeatureThreshold 
        if (getNumValidFeatures(b_xt->anchor_states[anchorIdx].feature_states) <
            4.0F) {
          //  anchor needs to be initialized
          //  free up updateVect
          // 'SLAM_upd:164' for featureIdx = 1:numPointsPerAnchor
          for (featureIdx = 0; featureIdx < 8; featureIdx++) {
            // 'SLAM_upd:165' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                != 0) {
              //                      log_info('clearing up feature %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              // 'SLAM_upd:167' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;

              // 'SLAM_upd:168' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;

              // 'SLAM_upd:169' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
            }
          }

          // 'SLAM_upd:173' if xt.origin.anchor_idx == anchorIdx
          if (b_xt->origin.anchor_idx == anchorIdx + 1) {
            // 'SLAM_upd:174' xt.origin.anchor_idx = int32(0);
            b_xt->origin.anchor_idx = 0;

            // 'SLAM_upd:175' log_info('Initializing anchor %i, which was the origin anchor', int32(anchorIdx)) 
            d_log_info(anchorIdx + 1);
          }

          // 'SLAM_upd:178' xt.anchor_states(anchorIdx).pos = xt.robot_state.pos; 
          for (i32 = 0; i32 < 3; i32++) {
            b_xt->anchor_states[anchorIdx].pos[i32] = b_xt->robot_state.pos[i32];
          }

          // 'SLAM_upd:179' xt.anchor_states(anchorIdx).att = xt.robot_state.att; 
          for (i32 = 0; i32 < 4; i32++) {
            b_xt->anchor_states[anchorIdx].att[i32] = b_xt->robot_state.att[i32];
          }

          // 'SLAM_upd:180' xt.anchor_states(anchorIdx).P_idx = numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6); 
          idx = anchorIdx * 14 + 18;
          for (i32 = 0; i32 < 6; i32++) {
            b_xt->anchor_states[anchorIdx].P_idx[i32] = (i32 + idx) + 1;
          }

          // 'SLAM_upd:182' P_apo(xt.anchor_states(anchorIdx).P_idx, :) = 0;
          for (i32 = 0; i32 < 102; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 * i32) - 1]
                = 0.0F;
            }
          }

          // 'SLAM_upd:183' P_apo(:, xt.anchor_states(anchorIdx).P_idx) = 0;
          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < 102; i34++) {
              P_apr[i34 + 102 * (b_xt->anchor_states[anchorIdx].P_idx[i32] - 1)]
                = 0.0F;
            }
          }

          // 'SLAM_upd:184' P_apo(xt.anchor_states(anchorIdx).P_idx(end) + (1:numPointsPerAnchor), :) = 0; 
          ix = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i32 = 0; i32 < 102; i32++) {
            for (i34 = 0; i34 < 8; i34++) {
              idx = 1 + i34;
              yk = ix + idx;
              if ((ix > 0) && (yk <= 0)) {
                yk = MAX_int32_T;
              }

              P_apr[(yk + 102 * i32) - 1] = 0.0F;
            }
          }

          // 'SLAM_upd:185' P_apo(:, xt.anchor_states(anchorIdx).P_idx(end) + (1:numPointsPerAnchor)) = 0; 
          ix = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i32 = 0; i32 < 8; i32++) {
            for (i34 = 0; i34 < 102; i34++) {
              idx = 1 + i32;
              yk = ix + idx;
              if ((ix > 0) && (yk <= 0)) {
                yk = MAX_int32_T;
              }

              P_apr[i34 + 102 * (yk - 1)] = 0.0F;
            }
          }

          //  covariance with robot states
          // 'SLAM_upd:189' P_apo(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx)        = P_apo(1:6, 1:6); 
          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              b_P_apr[i34 + 6 * i32] = P_apr[i34 + 102 * i32];
            }
          }

          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 *
                     (b_xt->anchor_states[anchorIdx].P_idx[i32] - 1)) - 1] =
                b_P_apr[i34 + 6 * i32];
            }
          }

          //  anchor position, attitude autocovariance
          // 'SLAM_upd:190' P_apo(xt.anchor_states(anchorIdx).P_idx, 1:6)                                      = P_apo(1:6, 1:6); 
          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              b_P_apr[i34 + 6 * i32] = P_apr[i34 + 102 * i32];
            }
          }

          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 * i32) - 1]
                = b_P_apr[i34 + 6 * i32];
            }
          }

          //  anchor position, attitude covariance with robot state
          // 'SLAM_upd:191' P_apo(1:6, xt.anchor_states(anchorIdx).P_idx)                                      = P_apo(1:6, 1:6); 
          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              b_P_apr[i34 + 6 * i32] = P_apr[i34 + 102 * i32];
            }
          }

          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              P_apr[i34 + 102 * (b_xt->anchor_states[anchorIdx].P_idx[i32] - 1)]
                = b_P_apr[i34 + 6 * i32];
            }
          }

          //  anchor position, attitude covariance with robot state
          // 'SLAM_upd:192' P_apo(int32(7:xt.anchor_states(anchorIdx).P_idx(1)-1), xt.anchor_states(anchorIdx).P_idx) = P_apo(int32(7:xt.anchor_states(anchorIdx).P_idx(1)-1), 1:6); 
          idx = b_xt->anchor_states[anchorIdx].P_idx[0];
          yk = idx - 1;
          if ((idx < 0) && (yk >= 0)) {
            yk = MIN_int32_T;
          }

          if (yk < 7) {
            ix = 0;
          } else {
            ix = yk - 6;
          }

          if (ix > 0) {
            vec_data[0] = 7;
            yk = 7;
            for (idx = 2; idx <= ix; idx++) {
              yk++;
              vec_data[idx - 1] = yk;
            }
          }

          for (i32 = 0; i32 < ix; i32++) {
            b_tmp_data[i32] = vec_data[i32] - 1;
          }

          idx = b_xt->anchor_states[anchorIdx].P_idx[0];
          yk = idx - 1;
          if ((idx < 0) && (yk >= 0)) {
            yk = MIN_int32_T;
          }

          if (yk < 7) {
            ix = 0;
          } else {
            ix = yk - 6;
          }

          if (ix > 0) {
            vec_data[0] = 7;
            yk = 7;
            for (idx = 2; idx <= ix; idx++) {
              yk++;
              vec_data[idx - 1] = yk;
            }
          }

          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < ix; i34++) {
              c_tmp_data[i34 + ix * i32] = P_apr[(vec_data[i34] + 102 * i32) - 1];
            }
          }

          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < ix; i34++) {
              P_apr[b_tmp_data[i34] + 102 * (b_xt->anchor_states[anchorIdx].
                P_idx[i32] - 1)] = c_tmp_data[i34 + ix * i32];
            }
          }

          //  covariance with bias states etc
          // 'SLAM_upd:193' P_apo(xt.anchor_states(anchorIdx).P_idx, int32(7:xt.anchor_states(anchorIdx).P_idx(1)-1)) = P_apo(1:6, int32(7:xt.anchor_states(anchorIdx).P_idx(1)-1)); 
          idx = b_xt->anchor_states[anchorIdx].P_idx[0];
          yk = idx - 1;
          if ((idx < 0) && (yk >= 0)) {
            yk = MIN_int32_T;
          }

          if (yk < 7) {
            ix = 0;
          } else {
            ix = yk - 6;
          }

          if (ix > 0) {
            vec_data[0] = 7;
            yk = 7;
            for (idx = 2; idx <= ix; idx++) {
              yk++;
              vec_data[idx - 1] = yk;
            }
          }

          for (i32 = 0; i32 < ix; i32++) {
            b_tmp_data[i32] = vec_data[i32] - 1;
          }

          idx = b_xt->anchor_states[anchorIdx].P_idx[0];
          yk = idx - 1;
          if ((idx < 0) && (yk >= 0)) {
            yk = MIN_int32_T;
          }

          if (yk < 7) {
            ix = 0;
          } else {
            ix = yk - 6;
          }

          if (ix > 0) {
            vec_data[0] = 7;
            yk = 7;
            for (idx = 2; idx <= ix; idx++) {
              yk++;
              vec_data[idx - 1] = yk;
            }
          }

          for (i32 = 0; i32 < ix; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              d_tmp_data[i34 + 6 * i32] = P_apr[i34 + 102 * (vec_data[i32] - 1)];
            }
          }

          for (i32 = 0; i32 < ix; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 *
                     b_tmp_data[i32]) - 1] = d_tmp_data[i34 + 6 * i32];
            }
          }

          //  covariance with bias states etc
          //  covariance with other anchor states
          // 'SLAM_upd:195' P_apo(int32(xt.anchor_states(anchorIdx).P_idx(1)+1:int32(end)), xt.anchor_states(anchorIdx).P_idx) = P_apo(int32(xt.anchor_states(anchorIdx).P_idx(1)+1:int32(end)), 1:6); 
          idx = b_xt->anchor_states[anchorIdx].P_idx[0];
          yk = idx + 1;
          if ((idx > 0) && (yk <= 0)) {
            yk = MAX_int32_T;
          }

          ix = 103 - yk;
          b_vec_data[0] = yk;
          for (idx = 2; idx <= ix; idx++) {
            yk++;
            b_vec_data[idx - 1] = yk;
          }

          for (i32 = 0; i32 < ix; i32++) {
            e_tmp_data[i32] = b_vec_data[i32] - 1;
          }

          idx = b_xt->anchor_states[anchorIdx].P_idx[0];
          yk = idx + 1;
          if ((idx > 0) && (yk <= 0)) {
            yk = MAX_int32_T;
          }

          ix = 103 - yk;
          b_vec_data[0] = yk;
          for (idx = 2; idx <= ix; idx++) {
            yk++;
            b_vec_data[idx - 1] = yk;
          }

          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < ix; i34++) {
              f_tmp_data[i34 + ix * i32] = P_apr[(b_vec_data[i34] + 102 * i32) -
                1];
            }
          }

          for (i32 = 0; i32 < 6; i32++) {
            for (i34 = 0; i34 < ix; i34++) {
              P_apr[e_tmp_data[i34] + 102 * (b_xt->anchor_states[anchorIdx].
                P_idx[i32] - 1)] = f_tmp_data[i34 + ix * i32];
            }
          }

          // 'SLAM_upd:196' P_apo(xt.anchor_states(anchorIdx).P_idx, int32(xt.anchor_states(anchorIdx).P_idx(1)+1:int32(end))) = P_apo(1:6, int32(xt.anchor_states(anchorIdx).P_idx(1)+1:int32(end))); 
          idx = b_xt->anchor_states[anchorIdx].P_idx[0];
          yk = idx + 1;
          if ((idx > 0) && (yk <= 0)) {
            yk = MAX_int32_T;
          }

          ix = 103 - yk;
          b_vec_data[0] = yk;
          for (idx = 2; idx <= ix; idx++) {
            yk++;
            b_vec_data[idx - 1] = yk;
          }

          for (i32 = 0; i32 < ix; i32++) {
            e_tmp_data[i32] = b_vec_data[i32] - 1;
          }

          idx = b_xt->anchor_states[anchorIdx].P_idx[0];
          yk = idx + 1;
          if ((idx > 0) && (yk <= 0)) {
            yk = MAX_int32_T;
          }

          ix = 103 - yk;
          b_vec_data[0] = yk;
          for (idx = 2; idx <= ix; idx++) {
            yk++;
            b_vec_data[idx - 1] = yk;
          }

          for (i32 = 0; i32 < ix; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              g_tmp_data[i34 + 6 * i32] = P_apr[i34 + 102 * (b_vec_data[i32] - 1)];
            }
          }

          for (i32 = 0; i32 < ix; i32++) {
            for (i34 = 0; i34 < 6; i34++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 *
                     e_tmp_data[i32]) - 1] = g_tmp_data[i34 + 6 * i32];
            }
          }

          // 'SLAM_upd:198' if coder.target('MATLAB')
          // 'SLAM_upd:214' for featureIdx = 1:numPointsPerAnchor
          featureIdx = 0;
          exitg8 = false;
          while ((!exitg8) && (featureIdx + 1 < 9)) {
            // 'SLAM_upd:215' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = 1/new_depths(new_feature_idx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth = 1.0F / ind_l2_data[new_feature_idx - 1];

            // 'SLAM_upd:216' xt.anchor_states(anchorIdx).feature_states(featureIdx).m = new_m(:,new_feature_idx); 
            for (i32 = 0; i32 < 3; i32++) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].m[i32] =
                new_m_data[i32 + 3 * (new_feature_idx - 1)];
            }

            // 'SLAM_upd:217' if VIOParameters.delayed_initialization
            if (b_VIOParameters.delayed_initialization) {
              // 'SLAM_upd:218' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(2); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                2;
            } else {
              // 'SLAM_upd:219' else
              // 'SLAM_upd:220' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(1); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }

            // 'SLAM_upd:222' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(status_ind(new_feature_idx)); 
            mtmp = roundf(status_ind_data[new_feature_idx - 1]);
            if (mtmp < 2.14748365E+9F) {
              if (mtmp >= -2.14748365E+9F) {
                i32 = (int)mtmp;
              } else {
                i32 = MIN_int32_T;
              }
            } else if (mtmp >= 2.14748365E+9F) {
              i32 = MAX_int32_T;
            } else {
              i32 = 0;
            }

            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx
              = i32;

            // 'SLAM_upd:223' xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx = int32(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx =
              (anchorIdx * 14 + featureIdx) + 25;

            // 'SLAM_upd:225' if new_feature_idx > nnz(triangulation_success)
            ix = 0;
            for (idx = 0; idx < c_triangulation_success_size_id; idx++) {
              if (triangulation_success_data[idx]) {
                ix++;
              }
            }

            if ((float)new_feature_idx > ix) {
              // 'SLAM_upd:226' log_info('Feature %d is too far away to triangulate.\n', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
              e_log_info(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx);

              // 'SLAM_upd:227' P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = noiseParameters.inv_depth_initial_unc*10; 
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 102 * (b_xt->anchor_states[anchorIdx].
                                    feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini * 10.0F;

              //  TODO: Maybe push the mean value further away?
            } else {
              // 'SLAM_upd:229' else
              // 'SLAM_upd:230' P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = noiseParameters.inv_depth_initial_unc; 
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 102 * (b_xt->anchor_states[anchorIdx].
                                    feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini;

              // *new_depths(new_feature_idx);
            }

            //                  log_info('Inserting feature %d as feature %i on anchor %i', int32(status_ind(new_feature_idx)), int32(featureIdx), int32(anchorIdx)) 
            // 'SLAM_upd:235' updateVect(status_ind(new_feature_idx)) = int32(1); 
            updateVect[(int)status_ind_data[new_feature_idx - 1] - 1] = 1;

            // 'SLAM_upd:237' new_feature_idx = new_feature_idx + 1;
            new_feature_idx++;

            //                  if new_feature_idx > length(new_depths)
            // 'SLAM_upd:239' if new_feature_idx > nnz(triangulation_success)
            ix = 0;
            for (idx = 0; idx < c_triangulation_success_size_id; idx++) {
              if (triangulation_success_data[idx]) {
                ix++;
              }
            }

            if ((float)new_feature_idx > ix) {
              exitg8 = true;
            } else {
              featureIdx++;
            }
          }
        }

        anchorIdx++;
      }
    }
  } else {
    if (!(qualities->size[0] == 0)) {
      // 'SLAM_upd:245' elseif ~isempty(ind_r)
      // 'SLAM_upd:246' log_warn('Got %d new feautures but not enough for a new anchor (min %d)', length(ind_r), int32(minFeatureThreshold)) 
      f_log_warn((float)loop_ub);
    }
  }

  // 'SLAM_upd:248' updateVect(updateVect==int32(2)) = int32(0);
  for (i = 0; i < 48; i++) {
    if (updateVect[i] == 2) {
      updateVect[i] = 0;
    }
  }

  //  remove features that were not inserted
  // 'SLAM_upd:250' if VIOParameters.delayed_initialization
  if (b_VIOParameters.delayed_initialization) {
    //  get the median uncertainty of the active features as a benchmark on
    //  the delayed features
    // 'SLAM_upd:253' uncertainties = -1*ones(numTrackFeatures, 1);
    for (i = 0; i < 48; i++) {
      b_uncertainties[i] = -1.0F;
    }

    // 'SLAM_upd:254' has_active_features = false;
    has_active_features = false;

    // 'SLAM_upd:255' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
      // 'SLAM_upd:256' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        // 'SLAM_upd:257' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            1) {
          // 'SLAM_upd:258' has_active_features = true;
          has_active_features = true;

          // 'SLAM_upd:259' uncertainties(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx); 
          b_uncertainties[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = P_apr
            [(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx +
              102 * (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx - 1)) - 1];
        }
      }
    }

    // 'SLAM_upd:264' if has_active_features
    if (has_active_features) {
      // 'SLAM_upd:265' median_uncertainty = median(uncertainties(uncertainties > 0), 1); 
      ix = 0;
      for (i = 0; i < 48; i++) {
        if (b_uncertainties[i] > 0.0F) {
          ix++;
        }
      }

      yk = 0;
      for (i = 0; i < 48; i++) {
        if (b_uncertainties[i] > 0.0F) {
          ii_data[yk] = i + 1;
          yk++;
        }
      }

      uncertainties_size[0] = ix;
      for (i32 = 0; i32 < ix; i32++) {
        b_triangulated_status_ind_data[i32] = b_uncertainties[ii_data[i32] - 1];
      }

      mtmp = median(b_triangulated_status_ind_data, uncertainties_size);

      //  because coder does not support nanflag
      //  check if a delayed initialization feature has converged
      // 'SLAM_upd:268' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
        // 'SLAM_upd:269' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'SLAM_upd:270' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
               2) && (P_apr[(b_xt->anchor_states[anchorIdx]
                             .feature_states[featureIdx].P_idx + 102 *
                             (b_xt->anchor_states[anchorIdx]
                              .feature_states[featureIdx].P_idx - 1)) - 1] <
                      mtmp * 2.0F)) {
            //  this feature is not active yet
            // 'SLAM_upd:271' if P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) < median_uncertainty*2 
            // 'SLAM_upd:272' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F) {
              // 'SLAM_upd:273' log_warn('Feature %i (%i on anchor %i) converged behind its anchor', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              g_log_warn(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx, featureIdx + 1,
                         anchorIdx + 1);

              // 'SLAM_upd:274' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;

              // 'SLAM_upd:275' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
            } else {
              // 'SLAM_upd:276' else
              //                              log_info('Feature %i (%i on anchor %i) has converged', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx); 
              // 'SLAM_upd:278' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(1); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }
          }
        }
      }
    }

    //  check if delayed features need to be forced active due to too few active 
    //  features
    // 'SLAM_upd:288' minActiveFeatureRatio = 0.4;
    // 'SLAM_upd:289' totalNumActiveFeatues = getTotalNumActiveFeatures(xt);
    totalNumActiveFeatues = getTotalNumActiveFeatures(b_xt->anchor_states);

    // 'SLAM_upd:290' if totalNumActiveFeatues < minActiveFeatureRatio*single(numTrackFeatures) 
    if (totalNumActiveFeatues < 19.2F) {
      //  find the best features and activate them
      // 'SLAM_upd:292' numDelayedFeatures = getTotalNumDelayedFeatures(xt);
      numDelayedFeatures = getTotalNumDelayedFeatures(b_xt->anchor_states);

      // 'SLAM_upd:293' qualities = zeros(numDelayedFeatures, 1);
      i32 = qualities->size[0];
      qualities->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)qualities, i32, (int)sizeof(float));
      loop_ub = (int)numDelayedFeatures;
      for (i32 = 0; i32 < loop_ub; i32++) {
        qualities->data[i32] = 0.0F;
      }

      //  quality measures of each delayed feature
      // 'SLAM_upd:294' anchorInd = qualities;
      i32 = anchorInd->size[0];
      anchorInd->size[0] = qualities->size[0];
      emxEnsureCapacity((emxArray__common *)anchorInd, i32, (int)sizeof(float));
      loop_ub = qualities->size[0];
      for (i32 = 0; i32 < loop_ub; i32++) {
        anchorInd->data[i32] = qualities->data[i32];
      }

      // 'SLAM_upd:295' featureInd = qualities;
      i32 = featureInd->size[0];
      featureInd->size[0] = qualities->size[0];
      emxEnsureCapacity((emxArray__common *)featureInd, i32, (int)sizeof(float));
      loop_ub = qualities->size[0];
      for (i32 = 0; i32 < loop_ub; i32++) {
        featureInd->data[i32] = qualities->data[i32];
      }

      // 'SLAM_upd:297' delayedIdx = 1;
      delayedIdx = 1.0F;

      // 'SLAM_upd:298' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
        // 'SLAM_upd:299' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'SLAM_upd:300' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              2) {
            // 'SLAM_upd:301' rho_unc = P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx); 
            // 'SLAM_upd:303' quality = rho_unc/noiseParameters.inv_depth_initial_unc; 
            // 'SLAM_upd:305' qualities(delayedIdx) = quality;
            qualities->data[(int)delayedIdx - 1] = P_apr[(b_xt->
              anchor_states[anchorIdx].feature_states[featureIdx].P_idx + 102 *
              (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx -
               1)) - 1] / c_noiseParameters_inv_depth_ini;

            // 'SLAM_upd:306' anchorInd(delayedIdx) = anchorIdx;
            anchorInd->data[(int)delayedIdx - 1] = (float)anchorIdx + 1.0F;

            // 'SLAM_upd:307' featureInd(delayedIdx) = featureIdx;
            featureInd->data[(int)delayedIdx - 1] = (float)featureIdx + 1.0F;

            // 'SLAM_upd:309' delayedIdx = delayedIdx + 1;
            delayedIdx++;
          }
        }
      }

      // 'SLAM_upd:314' [~, sortInd] = sort(qualities, 'ascend');
      c_sort(qualities, b_iidx);
      i32 = qualities->size[0];
      qualities->size[0] = b_iidx->size[0];
      emxEnsureCapacity((emxArray__common *)qualities, i32, (int)sizeof(float));
      loop_ub = b_iidx->size[0];
      for (i32 = 0; i32 < loop_ub; i32++) {
        qualities->data[i32] = (float)b_iidx->data[i32];
      }

      // 'SLAM_upd:314' ~
      // 'SLAM_upd:316' numActivatedFeatures = 0;
      numActivatedFeatures = 0.0F;

      // 'SLAM_upd:318' for i = 1:length(sortInd)
      i = 0;
      while ((i <= (int)(float)qualities->size[0] - 1) &&
             (!(numActivatedFeatures > ceilf(19.2F - totalNumActiveFeatues))) &&
             (!(numActivatedFeatures > numDelayedFeatures))) {
        // 'SLAM_upd:319' if numActivatedFeatures > ceil(minActiveFeatureRatio*single(numTrackFeatures) - totalNumActiveFeatues) || numActivatedFeatures > numDelayedFeatures 
        // 'SLAM_upd:322' if xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).inverse_depth < 0 
        if (b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[(int)
            (1.0F + (float)i) - 1] - 1] - 1].feature_states[(int)
            featureInd->data[(int)qualities->data[(int)(1.0F + (float)i) - 1] -
            1] - 1].inverse_depth < 0.0F) {
          // 'SLAM_upd:323' xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status = int32(0); 
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[(int)
            (1.0F + (float)i) - 1] - 1] - 1].feature_states[(int)
            featureInd->data[(int)qualities->data[(int)(1.0F + (float)i) - 1] -
            1] - 1].status = 0;

          // 'SLAM_upd:324' updateVect(xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx) = int32(0); 
          updateVect[b_xt->anchor_states[(int)anchorInd->data[(int)
            qualities->data[(int)(1.0F + (float)i) - 1] - 1] - 1]
            .feature_states[(int)featureInd->data[(int)qualities->data[(int)
            (1.0F + (float)i) - 1] - 1] - 1].status_idx - 1] = 0;

          // 'SLAM_upd:325' log_warn('Trying to force insert feature %i behind its anchor', xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx) 
          h_log_warn(b_xt->anchor_states[(int)anchorInd->data[(int)
                     qualities->data[(int)(1.0F + (float)i) - 1] - 1] - 1].
                     feature_states[(int)featureInd->data[(int)qualities->data
                     [(int)(1.0F + (float)i) - 1] - 1] - 1].status_idx);
        } else {
          // 'SLAM_upd:326' else
          // 'SLAM_upd:327' xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status = int32(1); 
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[(int)
            (1.0F + (float)i) - 1] - 1] - 1].feature_states[(int)
            featureInd->data[(int)qualities->data[(int)(1.0F + (float)i) - 1] -
            1] - 1].status = 1;

          // 'SLAM_upd:328' log_info('Forcing activation of feature %i (%i on anchor %i)', xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx, featureInd(sortInd(i)), anchorInd(sortInd(i))); 
          f_log_info(b_xt->anchor_states[(int)anchorInd->data[(int)
                     qualities->data[(int)(1.0F + (float)i) - 1] - 1] - 1].
                     feature_states[(int)featureInd->data[(int)qualities->data
                     [(int)(1.0F + (float)i) - 1] - 1] - 1].status_idx,
                     featureInd->data[(int)qualities->data[(int)(1.0F + (float)i)
                     - 1] - 1], anchorInd->data[(int)qualities->data[(int)(1.0F
                      + (float)i) - 1] - 1]);

          // 'SLAM_upd:329' numActivatedFeatures = numActivatedFeatures + 1;
          numActivatedFeatures++;
        }

        i++;
      }
    }
  }

  emxFree_int32_T(&b_iidx);
  emxFree_real32_T(&featureInd);
  emxFree_real32_T(&anchorInd);
  emxFree_real32_T(&qualities);

  //  check if new features need to be requested
  // 'SLAM_upd:336' request_new_features = false;
  request_new_features = false;

  // 'SLAM_upd:337' if nnz(updateVect == 0) > minFeatureThreshold
  ix = 0;
  for (idx = 0; idx < 48; idx++) {
    if (updateVect[idx] == 0) {
      ix++;
    }
  }

  if (ix > 4.0F) {
    //  if a new anchor can be filled enough
    // 'SLAM_upd:338' for anchorIdx = 1:numAnchors
    anchorIdx = 0;
    exitg6 = false;
    while ((!exitg6) && (anchorIdx + 1 < 7)) {
      // 'SLAM_upd:339' if getNumValidFeatures(xt.anchor_states(anchorIdx)) < minFeatureThreshold 
      if (getNumValidFeatures(b_xt->anchor_states[anchorIdx].feature_states) <
          4.0F) {
        //  discard all features of this anchor
        // 'SLAM_upd:341' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'SLAM_upd:342' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              status_idx != 0) {
            // 'SLAM_upd:343' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
            updateVect[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
              .status_idx - 1] = 0;

            // 'SLAM_upd:344' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;

            // 'SLAM_upd:345' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx
              = 0;
          }
        }

        // 'SLAM_upd:348' request_new_features = true;
        request_new_features = true;
        exitg6 = true;
      } else {
        anchorIdx++;
      }
    }

    //  else % debug check
    //      if coder.target('MATLAB')
    //          for anchorIdx = 1:numAnchors
    //              if getNumValidFeatures(xt.anchor_states(anchorIdx)) < minFeatureThreshold 
    //                  log_error('anchor %i needs new features but cant ask for them', anchorIdx) 
    //              end
    //          end
    //      end
  }

  // 'SLAM_upd:362' if request_new_features
  if (request_new_features) {
    // 'SLAM_upd:363' request_idx = 1;
    request_idx = 1;

    // 'SLAM_upd:364' for i = 1:length(updateVect)
    i = 0;
    exitg5 = false;
    while ((!exitg5) && (i < 48)) {
      // 'SLAM_upd:365' if updateVect(i) == 0
      b_guard1 = false;
      if (updateVect[i] == 0) {
        // 'SLAM_upd:366' updateVect(i) = 2;
        updateVect[i] = 2;

        // 'SLAM_upd:367' request_idx = request_idx +1;
        request_idx++;

        // 'SLAM_upd:368' if request_idx > max_features_to_request
        if (request_idx > 16) {
          exitg5 = true;
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

    //      log_info('Requesting %d new features', int32(request_idx-1))
    //      updateVect(updateVect == 0) = int32(2); % get as many new features as possible 
  }

  // % robocentric update
  // 'SLAM_upd:378' if xt.origin.anchor_idx == 0
  if (b_xt->origin.anchor_idx == 0) {
    //  need to update the origin anchor and the state
    // 'SLAM_upd:379' xt.fixed_feature = int32(0);
    b_xt->fixed_feature = 0;

    //  ensure that a new feature will be fixed, if this option is enabled
    //  choose the best anchor as the new origin anchor
    // 'SLAM_upd:381' uncertainties = zeros(numAnchors, 1);
    //  uncertainties of the anchors reduced to a scalar
    // 'SLAM_upd:382' has_active_features = uncertainties;
    for (i = 0; i < 6; i++) {
      b_has_active_features[i] = 0.0F;
    }

    // 'SLAM_upd:383' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
      // 'SLAM_upd:384' if anyActiveAnchorFeatures(xt.anchor_states(anchorIdx))
      if (anyActiveAnchorFeatures(b_xt->anchor_states[anchorIdx].feature_states))
      {
        // 'SLAM_upd:385' uncertainties(anchorIdx) = norm(P_apo(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx)); 
        for (i32 = 0; i32 < 6; i32++) {
          for (i34 = 0; i34 < 6; i34++) {
            b_P_apr[i34 + 6 * i32] = P_apr[(b_xt->anchor_states[anchorIdx]
              .P_idx[i34] + 102 * (b_xt->anchor_states[anchorIdx].P_idx[i32] - 1))
              - 1];
          }
        }

        mtmp = d_norm(b_P_apr);
        c_uncertainties[anchorIdx] = mtmp;

        // 'SLAM_upd:386' has_active_features(anchorIdx) = 1;
        b_has_active_features[anchorIdx] = 1.0F;
      } else {
        // 'SLAM_upd:387' else
        // 'SLAM_upd:388' uncertainties(anchorIdx) = 1000;
        c_uncertainties[anchorIdx] = 1000.0F;

        //  dont fix an anchor with no active features
      }
    }

    // 'SLAM_upd:391' if ~any(has_active_features)
    if (!c_any(b_has_active_features)) {
      //  can happen if outlier rejection rejected all features
      // 'SLAM_upd:392' log_warn('Can''t fix an anchor because none have active features') 
      i_log_warn();
    } else {
      // 'SLAM_upd:393' else
      // 'SLAM_upd:394' [~, sortIdx] = min(uncertainties);
      yk = 1;
      mtmp = c_uncertainties[0];
      idx = 1;
      if (rtIsNaNF(c_uncertainties[0])) {
        ix = 2;
        exitg4 = false;
        while ((!exitg4) && (ix < 7)) {
          yk = ix;
          if (!rtIsNaNF(c_uncertainties[ix - 1])) {
            mtmp = c_uncertainties[ix - 1];
            idx = ix;
            exitg4 = true;
          } else {
            ix++;
          }
        }
      }

      if (yk < 6) {
        while (yk + 1 < 7) {
          if (c_uncertainties[yk] < mtmp) {
            mtmp = c_uncertainties[yk];
            idx = yk + 1;
          }

          yk++;
        }
      }

      // 'SLAM_upd:394' ~
      // 'SLAM_upd:395' xt.origin.anchor_idx = int32(sortIdx(1));
      b_xt->origin.anchor_idx = idx;

      // 'SLAM_upd:396' if ~has_active_features(xt.origin.anchor_idx)
      if (!(b_has_active_features[b_xt->origin.anchor_idx - 1] != 0.0F)) {
        //  debug check
        // 'SLAM_upd:397' log_error('Picked an anchor with no active features as origin (anchor %d). Probably because no anchors have any features.', int32(xt.origin.anchor_idx)) 
        // log_error Print to console in Matlab
        //  in C++, vio_logging.h needs to be created to define what LOG_ERROR does, 
        //  e.g. redefine ROS_ERROR
        // 'log_error:6' if coder.target('MATLAB')
        // 'log_error:8' elseif ~coder.target('MEX')
        // 'log_error:9' coder.cinclude('<vio_logging.h>')
        // 'log_error:10' coder.ceval('LOG_ERROR', [str, 0], varargin{:});
        memcpy(&cv46[0], &cv47[0], 111U * sizeof(char));
        LOG_ERROR(cv46, b_xt->origin.anchor_idx);
      } else {
        // 'SLAM_upd:398' else
        // 'SLAM_upd:399' log_info('Setting anchor %i as origin', int32(xt.origin.anchor_idx)) 
        g_log_info(b_xt->origin.anchor_idx);

        // 'SLAM_upd:401' new_origin_pos_rel = xt.anchor_states(xt.origin.anchor_idx).pos; 
        for (i = 0; i < 3; i++) {
          new_origin_pos_rel[i] = b_xt->anchor_states[b_xt->origin.anchor_idx -
            1].pos[i];
        }

        //  in old origin frame
        // 'SLAM_upd:402' new_origin_att_rel = RotFromQuatJ(xt.anchor_states(xt.origin.anchor_idx).att); 
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        new_origin_att_rel[0] = ((b_xt->anchor_states[b_xt->origin.anchor_idx -
          1].att[0] * b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[0] -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1]) -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3];
        new_origin_att_rel[3] = 2.0F * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[6] = 2.0F * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] - b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[1] = 2.0F * (b_xt->anchor_states
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
        new_origin_att_rel[7] = 2.0F * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[2] = 2.0F * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[5] = 2.0F * (b_xt->anchor_states
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
        // 'SLAM_upd:404' J = eye(size(P_apo));
        for (i32 = 0; i32 < 10404; i32++) {
          J[i32] = iv10[i32];
        }

        //  robot position and orientation
        // 'SLAM_upd:406' J(1:6, 1:6) = [new_origin_att_rel, zeros(3);...
        // 'SLAM_upd:407'                                      zeros(3), eye(3)]; 
        for (i32 = 0; i32 < 3; i32++) {
          for (i34 = 0; i34 < 3; i34++) {
            J[i34 + 102 * i32] = new_origin_att_rel[i34 + 3 * i32];
            J[i34 + 102 * (i32 + 3)] = 0.0F;
          }
        }

        for (i32 = 0; i32 < 6; i32++) {
          for (i34 = 0; i34 < 3; i34++) {
            J[(i34 + 102 * i32) + 3] = iv11[i34 + 3 * i32];
          }
        }

        // 'SLAM_upd:408' J(1:6, xt.anchor_states(xt.origin.anchor_idx).P_idx) = [-new_origin_att_rel, skew(new_origin_att_rel * (xt.robot_state.pos - new_origin_pos_rel));... 
        // 'SLAM_upd:409'                                                                                zeros(3), -RotFromQuatJ(xt.robot_state.att) * new_origin_att_rel']; 
        mw_neon_mm_sub_f32x4(b_xt->robot_state.pos, 3, 1, b_xt->
                             anchor_states[b_xt->origin.anchor_idx - 1].pos,
                             &fv27[0]);

        // 'skew:2' R=[0,-w(3),w(2);
        // 'skew:3'     w(3),0,-w(1);
        // 'skew:4'     -w(2),w(1),0];
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        c_xt[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        c_xt[3] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                          b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        c_xt[6] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                          b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        c_xt[1] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                          b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        c_xt[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        c_xt[7] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                          b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        c_xt[2] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                          b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        c_xt[5] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                          b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        c_xt[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        for (i32 = 0; i32 < 3; i32++) {
          fp[i32] = 0.0F;
          for (i34 = 0; i34 < 3; i34++) {
            d_xt[i34 + 3 * i32] = -c_xt[i34 + 3 * i32];
            fp[i32] += new_origin_att_rel[i32 + 3 * i34] * fv27[i34];
          }
        }

        for (i32 = 0; i32 < 3; i32++) {
          for (i34 = 0; i34 < 3; i34++) {
            c_xt[i32 + 3 * i34] = 0.0F;
            for (idx = 0; idx < 3; idx++) {
              c_xt[i32 + 3 * i34] += d_xt[i32 + 3 * idx] *
                new_origin_att_rel[i34 + 3 * idx];
            }

            J[i34 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                           P_idx[i32] - 1)] = -new_origin_att_rel[i34 + 3 * i32];
            J[(i34 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                            P_idx[i32] - 1)) + 3] = 0.0F;
          }
        }

        J[102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1)]
          = 0.0F;
        J[102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1)]
          = -fp[2];
        J[102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1)]
          = fp[1];
        J[1 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                     1)] = fp[2];
        J[1 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                     1)] = 0.0F;
        J[1 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                     1)] = -fp[0];
        J[2 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                     1)] = -fp[1];
        J[2 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                     1)] = fp[0];
        J[2 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                     1)] = 0.0F;

        //  robot velocity
        // 'SLAM_upd:411' J(7:9, 7:9) = new_origin_att_rel;
        //  velocity
        // 'SLAM_upd:412' J(7:9, xt.anchor_states(xt.origin.anchor_idx).P_idx) = [zeros(3), skew(new_origin_att_rel * xt.robot_state.vel)]; 
        for (i32 = 0; i32 < 3; i32++) {
          fp[i32] = 0.0F;
          for (i34 = 0; i34 < 3; i34++) {
            J[(i34 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                            P_idx[i32 + 3] - 1)) + 3] = c_xt[i34 + 3 * i32];
            J[(i34 + 102 * (6 + i32)) + 6] = new_origin_att_rel[i34 + 3 * i32];
            fp[i32] += new_origin_att_rel[i32 + 3 * i34] * b_xt->
              robot_state.vel[i34];
          }
        }

        // 'skew:2' R=[0,-w(3),w(2);
        // 'skew:3'     w(3),0,-w(1);
        // 'skew:4'     -w(2),w(1),0];
        for (i32 = 0; i32 < 3; i32++) {
          for (i34 = 0; i34 < 3; i34++) {
            J[(i34 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                            P_idx[i32] - 1)) + 6] = 0.0F;
            J[(i34 + 102 * (15 + i32)) + 15] = new_origin_att_rel[i34 + 3 * i32];
          }
        }

        J[6 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                     1)] = 0.0F;
        J[6 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                     1)] = -fp[2];
        J[6 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                     1)] = fp[1];
        J[7 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                     1)] = fp[2];
        J[7 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                     1)] = 0.0F;
        J[7 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                     1)] = -fp[0];
        J[8 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                     1)] = -fp[1];
        J[8 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                     1)] = fp[0];
        J[8 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                     1)] = 0.0F;

        //  origin rotation
        // 'SLAM_upd:414' J(16:18, 16:18) = new_origin_att_rel;
        //  origin rotation
        // 'SLAM_upd:415' J(16:18, xt.anchor_states(xt.origin.anchor_idx).P_idx) = [zeros(3), eye(3)]; 
        for (i32 = 0; i32 < 6; i32++) {
          for (i34 = 0; i34 < 3; i34++) {
            J[(i34 + 102 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                            P_idx[i32] - 1)) + 15] = iv11[i34 + 3 * i32];
          }
        }

        // 'SLAM_upd:417' for anchorIdx = 1:numAnchors
        for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
          // 'SLAM_upd:418' if anchorIdx == xt.origin.anchor_idx
          if (anchorIdx + 1 == b_xt->origin.anchor_idx) {
            // 'SLAM_upd:419' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx) = [zeros(3), zeros(3); zeros(3), zeros(3)]; 
            for (i32 = 0; i32 < 6; i32++) {
              for (i34 = 0; i34 < 6; i34++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i32] - 1)) - 1] = 0.0F;
              }
            }
          } else {
            // 'SLAM_upd:420' else
            // 'SLAM_upd:421' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx) = [new_origin_att_rel, zeros(3);... 
            // 'SLAM_upd:422'                                                                                                          zeros(3), eye(3)]; 
            for (i32 = 0; i32 < 3; i32++) {
              for (i34 = 0; i34 < 3; i34++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i32] - 1)) - 1] =
                  new_origin_att_rel[i34 + 3 * i32];
              }
            }

            for (i32 = 0; i32 < 3; i32++) {
              for (i34 = 0; i34 < 3; i34++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i32 + 3] - 1)) - 1] =
                  0.0F;
              }
            }

            for (i32 = 0; i32 < 6; i32++) {
              for (i34 = 0; i34 < 3; i34++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i34 + 3] + 102 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i32] - 1)) - 1] =
                  iv11[i34 + 3 * i32];
              }
            }

            // 'SLAM_upd:423' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(xt.origin.anchor_idx).P_idx) = [-new_origin_att_rel, skew(new_origin_att_rel * (xt.anchor_states(anchorIdx).pos - new_origin_pos_rel));... 
            // 'SLAM_upd:424'                                                                                                                      zeros(3), -RotFromQuatJ(xt.anchor_states(anchorIdx).att) * new_origin_att_rel']; 
            mw_neon_mm_sub_f32x4(b_xt->anchor_states[anchorIdx].pos, 3, 1,
                                 new_origin_pos_rel, &fv27[0]);

            // 'skew:2' R=[0,-w(3),w(2);
            // 'skew:3'     w(3),0,-w(1);
            // 'skew:4'     -w(2),w(1),0];
            //  if ~all(size(q) == [4, 1])
            //      error('q does not have the size of a quaternion')
            //  end
            //  if abs(norm(q) - 1) > 1e-3
            //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
            //  end
            // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
            // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
            // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
            e_xt[0] = ((b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0] -
                        b_xt->anchor_states[anchorIdx].att[1] *
                        b_xt->anchor_states[anchorIdx].att[1]) -
                       b_xt->anchor_states[anchorIdx].att[2] *
                       b_xt->anchor_states[anchorIdx].att[2]) +
              b_xt->anchor_states[anchorIdx].att[3] * b_xt->
              anchor_states[anchorIdx].att[3];
            e_xt[3] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                              b_xt->anchor_states[anchorIdx].att[1] +
                              b_xt->anchor_states[anchorIdx].att[2] *
                              b_xt->anchor_states[anchorIdx].att[3]);
            e_xt[6] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                              b_xt->anchor_states[anchorIdx].att[2] -
                              b_xt->anchor_states[anchorIdx].att[1] *
                              b_xt->anchor_states[anchorIdx].att[3]);
            e_xt[1] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                              b_xt->anchor_states[anchorIdx].att[1] -
                              b_xt->anchor_states[anchorIdx].att[2] *
                              b_xt->anchor_states[anchorIdx].att[3]);
            e_xt[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[0]) +
                        b_xt->anchor_states[anchorIdx].att[1] *
                        b_xt->anchor_states[anchorIdx].att[1]) -
                       b_xt->anchor_states[anchorIdx].att[2] *
                       b_xt->anchor_states[anchorIdx].att[2]) +
              b_xt->anchor_states[anchorIdx].att[3] * b_xt->
              anchor_states[anchorIdx].att[3];
            e_xt[7] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
                              b_xt->anchor_states[anchorIdx].att[2] +
                              b_xt->anchor_states[anchorIdx].att[0] *
                              b_xt->anchor_states[anchorIdx].att[3]);
            e_xt[2] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                              b_xt->anchor_states[anchorIdx].att[2] +
                              b_xt->anchor_states[anchorIdx].att[1] *
                              b_xt->anchor_states[anchorIdx].att[3]);
            e_xt[5] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
                              b_xt->anchor_states[anchorIdx].att[2] -
                              b_xt->anchor_states[anchorIdx].att[0] *
                              b_xt->anchor_states[anchorIdx].att[3]);
            e_xt[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[0]) -
                        b_xt->anchor_states[anchorIdx].att[1] *
                        b_xt->anchor_states[anchorIdx].att[1]) +
                       b_xt->anchor_states[anchorIdx].att[2] *
                       b_xt->anchor_states[anchorIdx].att[2]) +
              b_xt->anchor_states[anchorIdx].att[3] * b_xt->
              anchor_states[anchorIdx].att[3];
            for (i32 = 0; i32 < 3; i32++) {
              fp[i32] = 0.0F;
              for (i34 = 0; i34 < 3; i34++) {
                c_xt[i34 + 3 * i32] = -e_xt[i34 + 3 * i32];
                fp[i32] += new_origin_att_rel[i32 + 3 * i34] * fv27[i34];
              }
            }

            for (i32 = 0; i32 < 3; i32++) {
              for (i34 = 0; i34 < 3; i34++) {
                d_xt[i32 + 3 * i34] = 0.0F;
                for (idx = 0; idx < 3; idx++) {
                  d_xt[i32 + 3 * i34] += c_xt[i32 + 3 * idx] *
                    new_origin_att_rel[i34 + 3 * idx];
                }

                J[(b_xt->anchor_states[anchorIdx].P_idx[i34] + 102 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i32]
                    - 1)) - 1] = -new_origin_att_rel[i34 + 3 * i32];
              }
            }

            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = 0.0F;
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = -fp[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = fp[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = fp[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = 0.0F;
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = -fp[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = -fp[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = fp[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 102 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = 0.0F;
            for (i32 = 0; i32 < 3; i32++) {
              for (i34 = 0; i34 < 3; i34++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i34 + 3] + 102 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i32]
                    - 1)) - 1] = 0.0F;
              }
            }

            for (i32 = 0; i32 < 3; i32++) {
              for (i34 = 0; i34 < 3; i34++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i34 + 3] + 102 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i32 +
                    3] - 1)) - 1] = d_xt[i34 + 3 * i32];
              }
            }
          }

          // 'SLAM_upd:427' xt.anchor_states(anchorIdx).pos = new_origin_att_rel * (xt.anchor_states(anchorIdx).pos - new_origin_pos_rel); 
          mw_neon_mm_sub_f32x4(b_xt->anchor_states[anchorIdx].pos, 3, 1,
                               new_origin_pos_rel, &fp[0]);

          // 'SLAM_upd:428' xt.anchor_states(anchorIdx).att = QuatFromRotJ(RotFromQuatJ(xt.anchor_states(anchorIdx).att) * new_origin_att_rel'); 
          //  if ~all(size(q) == [4, 1])
          //      error('q does not have the size of a quaternion')
          //  end
          //  if abs(norm(q) - 1) > 1e-3
          //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
          //  end
          // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
          // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
          // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
          f_xt[0] = ((b_xt->anchor_states[anchorIdx].att[0] *
                      b_xt->anchor_states[anchorIdx].att[0] -
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) -
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          f_xt[3] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[1] +
                            b_xt->anchor_states[anchorIdx].att[2] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          f_xt[6] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[2] -
                            b_xt->anchor_states[anchorIdx].att[1] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          f_xt[1] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[1] -
                            b_xt->anchor_states[anchorIdx].att[2] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          f_xt[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0]) +
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) -
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          f_xt[7] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
                            b_xt->anchor_states[anchorIdx].att[2] +
                            b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          f_xt[2] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[2] +
                            b_xt->anchor_states[anchorIdx].att[1] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          f_xt[5] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
                            b_xt->anchor_states[anchorIdx].att[2] -
                            b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          f_xt[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0]) -
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) +
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          for (i = 0; i < 3; i++) {
            b_xt->anchor_states[anchorIdx].pos[i] = 0.0F;
            b_xt->anchor_states[anchorIdx].pos[i] = 0.0F;
            for (i32 = 0; i32 < 3; i32++) {
              b_xt->anchor_states[anchorIdx].pos[i] += new_origin_att_rel[i + 3 *
                i32] * fp[i32];
              b_c[i + 3 * i32] = 0.0F;
              for (i34 = 0; i34 < 3; i34++) {
                b_c[i + 3 * i32] += f_xt[i + 3 * i34] * new_origin_att_rel[i32 +
                  3 * i34];
              }
            }
          }

          //  THIS IS OK, It is according to the NASA memo found
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
          // % speed optimization
          // 'QuatFromRotJ:50' [~, index] = max([1+R(1,1)-(R(2,2)+R(3,3)), 1+R(2,2)-(R(1,1)+R(3,3)), 1+R(3,3)-(R(1,1)+R(2,2)), 1+(R(1,1)+R(2,2)+R(3,3))]); 
          varargin_1[0] = (1.0F + b_c[0]) - (b_c[4] + b_c[8]);
          varargin_1[1] = (1.0F + b_c[4]) - (b_c[0] + b_c[8]);
          varargin_1[2] = (1.0F + b_c[8]) - (b_c[0] + b_c[4]);
          varargin_1[3] = 1.0F + ((b_c[0] + b_c[4]) + b_c[8]);
          yk = 1;
          mtmp = varargin_1[0];
          idx = 1;
          if (rtIsNaNF(varargin_1[0])) {
            ix = 2;
            exitg3 = false;
            while ((!exitg3) && (ix < 5)) {
              yk = ix;
              if (!rtIsNaNF(varargin_1[ix - 1])) {
                mtmp = varargin_1[ix - 1];
                idx = ix;
                exitg3 = true;
              } else {
                ix++;
              }
            }
          }

          if (yk < 4) {
            while (yk + 1 < 5) {
              if (varargin_1[yk] > mtmp) {
                mtmp = varargin_1[yk];
                idx = yk + 1;
              }

              yk++;
            }
          }

          // 'QuatFromRotJ:50' ~
          // 'QuatFromRotJ:52' if(index==1)
          if (idx == 1) {
            // 'QuatFromRotJ:53' Q = [sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2; 
            // 'QuatFromRotJ:54'         (R(1,2)+R(2,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:55'         (R(1,3)+R(3,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:56'         (R(2,3)-R(3,2))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
            b_xt->anchor_states[anchorIdx].att[0] = sqrtf((1.0F + 2.0F * b_c[0])
              - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F;
            b_xt->anchor_states[anchorIdx].att[1] = (b_c[3] + b_c[1]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
            b_xt->anchor_states[anchorIdx].att[2] = (b_c[6] + b_c[2]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
            b_xt->anchor_states[anchorIdx].att[3] = (b_c[7] - b_c[5]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
          } else if (idx == 2) {
            // 'QuatFromRotJ:57' elseif(index==2)
            // 'QuatFromRotJ:58' Q = [(R(1,2)+R(2,1))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:59'         (sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2); 
            // 'QuatFromRotJ:60'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:61'         (R(3,1)-R(1,3))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
            b_xt->anchor_states[anchorIdx].att[0] = (b_c[3] + b_c[1]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
            b_xt->anchor_states[anchorIdx].att[1] = sqrtf((1.0F + 2.0F * b_c[4])
              - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F;
            b_xt->anchor_states[anchorIdx].att[2] = (b_c[7] + b_c[5]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
            b_xt->anchor_states[anchorIdx].att[3] = (b_c[2] - b_c[6]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
          } else if (idx == 3) {
            // 'QuatFromRotJ:62' elseif(index==3)
            // 'QuatFromRotJ:63' Q = [(R(1,3)+R(3,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:64'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:65'         (sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2); 
            // 'QuatFromRotJ:66'         (R(1,2)-R(2,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
            b_xt->anchor_states[anchorIdx].att[0] = (b_c[6] + b_c[2]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
            b_xt->anchor_states[anchorIdx].att[1] = (b_c[7] + b_c[5]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
            b_xt->anchor_states[anchorIdx].att[2] = sqrtf((1.0F + 2.0F * b_c[8])
              - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F;
            b_xt->anchor_states[anchorIdx].att[3] = (b_c[3] - b_c[1]) / (4.0F *
              (sqrtf((1.0F + 2.0F * b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) /
               2.0F));
          } else {
            // 'QuatFromRotJ:67' else
            // 'QuatFromRotJ:68' Q = [(R(2,3)-R(3,2))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:69'         (R(3,1)-R(1,3))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:70'         (R(1,2)-R(2,1))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
            // 'QuatFromRotJ:71'         (sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2);];
            b_xt->anchor_states[anchorIdx].att[0] = (b_c[7] - b_c[5]) / (4.0F *
              (sqrtf(1.0F + ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[1] = (b_c[2] - b_c[6]) / (4.0F *
              (sqrtf(1.0F + ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[2] = (b_c[3] - b_c[1]) / (4.0F *
              (sqrtf(1.0F + ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[3] = sqrtf(1.0F + ((b_c[0] + b_c
              [4]) + b_c[8])) / 2.0F;
          }
        }

        // 'SLAM_upd:431' P_apo = J * P_apo * J';
        for (i32 = 0; i32 < 102; i32++) {
          for (i34 = 0; i34 < 102; i34++) {
            b_J[i32 + 102 * i34] = 0.0F;
            for (idx = 0; idx < 102; idx++) {
              b_J[i32 + 102 * i34] += J[i32 + 102 * idx] * P_apr[idx + 102 * i34];
            }
          }
        }

        for (i32 = 0; i32 < 102; i32++) {
          for (i34 = 0; i34 < 102; i34++) {
            P_apr[i32 + 102 * i34] = 0.0F;
            for (idx = 0; idx < 102; idx++) {
              P_apr[i32 + 102 * i34] += b_J[i32 + 102 * idx] * J[i34 + 102 * idx];
            }
          }
        }

        // 'SLAM_upd:433' xt.robot_state.pos = new_origin_att_rel * (xt.robot_state.pos - new_origin_pos_rel); 
        mw_neon_mm_sub_f32x4(b_xt->robot_state.pos, 3, 1, new_origin_pos_rel,
                             &fv27[0]);

        // 'SLAM_upd:434' xt.robot_state.att = QuatFromRotJ(RotFromQuatJ(xt.robot_state.att) * new_origin_att_rel'); 
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        g_xt[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        g_xt[3] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                          b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        g_xt[6] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                          b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        g_xt[1] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                          b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        g_xt[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        g_xt[7] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                          b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        g_xt[2] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                          b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        g_xt[5] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                          b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        g_xt[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        for (i32 = 0; i32 < 3; i32++) {
          b_xt->robot_state.pos[i32] = 0.0F;
          for (i34 = 0; i34 < 3; i34++) {
            b_xt->robot_state.pos[i32] += new_origin_att_rel[i32 + 3 * i34] *
              fv27[i34];
            b_c[i32 + 3 * i34] = 0.0F;
            for (idx = 0; idx < 3; idx++) {
              b_c[i32 + 3 * i34] += g_xt[i32 + 3 * idx] * new_origin_att_rel[i34
                + 3 * idx];
            }
          }
        }

        //  THIS IS OK, It is according to the NASA memo found
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
        // % speed optimization
        // 'QuatFromRotJ:50' [~, index] = max([1+R(1,1)-(R(2,2)+R(3,3)), 1+R(2,2)-(R(1,1)+R(3,3)), 1+R(3,3)-(R(1,1)+R(2,2)), 1+(R(1,1)+R(2,2)+R(3,3))]); 
        varargin_1[0] = (1.0F + b_c[0]) - (b_c[4] + b_c[8]);
        varargin_1[1] = (1.0F + b_c[4]) - (b_c[0] + b_c[8]);
        varargin_1[2] = (1.0F + b_c[8]) - (b_c[0] + b_c[4]);
        varargin_1[3] = 1.0F + ((b_c[0] + b_c[4]) + b_c[8]);
        yk = 1;
        mtmp = varargin_1[0];
        idx = 1;
        if (rtIsNaNF(varargin_1[0])) {
          ix = 2;
          exitg2 = false;
          while ((!exitg2) && (ix < 5)) {
            yk = ix;
            if (!rtIsNaNF(varargin_1[ix - 1])) {
              mtmp = varargin_1[ix - 1];
              idx = ix;
              exitg2 = true;
            } else {
              ix++;
            }
          }
        }

        if (yk < 4) {
          while (yk + 1 < 5) {
            if (varargin_1[yk] > mtmp) {
              mtmp = varargin_1[yk];
              idx = yk + 1;
            }

            yk++;
          }
        }

        // 'QuatFromRotJ:50' ~
        // 'QuatFromRotJ:52' if(index==1)
        if (idx == 1) {
          // 'QuatFromRotJ:53' Q = [sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2;
          // 'QuatFromRotJ:54'         (R(1,2)+R(2,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:55'         (R(1,3)+R(3,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:56'         (R(2,3)-R(3,2))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
          b_xt->robot_state.att[0] = sqrtf((1.0F + 2.0F * b_c[0]) - ((b_c[0] +
            b_c[4]) + b_c[8])) / 2.0F;
          b_xt->robot_state.att[1] = (b_c[3] + b_c[1]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[2] = (b_c[6] + b_c[2]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[3] = (b_c[7] - b_c[5]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
        } else if (idx == 2) {
          // 'QuatFromRotJ:57' elseif(index==2)
          // 'QuatFromRotJ:58' Q = [(R(1,2)+R(2,1))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:59'         (sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2); 
          // 'QuatFromRotJ:60'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:61'         (R(3,1)-R(1,3))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
          b_xt->robot_state.att[0] = (b_c[3] + b_c[1]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[1] = sqrtf((1.0F + 2.0F * b_c[4]) - ((b_c[0] +
            b_c[4]) + b_c[8])) / 2.0F;
          b_xt->robot_state.att[2] = (b_c[7] + b_c[5]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[3] = (b_c[2] - b_c[6]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
        } else if (idx == 3) {
          // 'QuatFromRotJ:62' elseif(index==3)
          // 'QuatFromRotJ:63' Q = [(R(1,3)+R(3,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:64'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:65'         (sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2); 
          // 'QuatFromRotJ:66'         (R(1,2)-R(2,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
          b_xt->robot_state.att[0] = (b_c[6] + b_c[2]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[1] = (b_c[7] + b_c[5]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[2] = sqrtf((1.0F + 2.0F * b_c[8]) - ((b_c[0] +
            b_c[4]) + b_c[8])) / 2.0F;
          b_xt->robot_state.att[3] = (b_c[3] - b_c[1]) / (4.0F * (sqrtf((1.0F +
            2.0F * b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
        } else {
          // 'QuatFromRotJ:67' else
          // 'QuatFromRotJ:68' Q = [(R(2,3)-R(3,2))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:69'         (R(3,1)-R(1,3))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:70'         (R(1,2)-R(2,1))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:71'         (sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2);];
          b_xt->robot_state.att[0] = (b_c[7] - b_c[5]) / (4.0F * (sqrtf(1.0F +
            ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[1] = (b_c[2] - b_c[6]) / (4.0F * (sqrtf(1.0F +
            ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[2] = (b_c[3] - b_c[1]) / (4.0F * (sqrtf(1.0F +
            ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->robot_state.att[3] = sqrtf(1.0F + ((b_c[0] + b_c[4]) + b_c[8])) /
            2.0F;
        }

        // 'SLAM_upd:435' xt.robot_state.vel = new_origin_att_rel * xt.robot_state.vel; 
        for (i32 = 0; i32 < 3; i32++) {
          b_new_origin_att_rel[i32] = 0.0F;
          for (i34 = 0; i34 < 3; i34++) {
            b_new_origin_att_rel[i32] += new_origin_att_rel[i32 + 3 * i34] *
              b_xt->robot_state.vel[i34];
          }
        }

        // 'SLAM_upd:437' xt.origin.pos = xt.origin.pos + RotFromQuatJ(xt.origin.att)' * new_origin_pos_rel; 
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        h_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        h_xt[1] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                          b_xt->origin.att[2] * b_xt->origin.att[3]);
        h_xt[2] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                          b_xt->origin.att[1] * b_xt->origin.att[3]);
        h_xt[3] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                          b_xt->origin.att[2] * b_xt->origin.att[3]);
        h_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        h_xt[5] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                          b_xt->origin.att[0] * b_xt->origin.att[3]);
        h_xt[6] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                          b_xt->origin.att[1] * b_xt->origin.att[3]);
        h_xt[7] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                          b_xt->origin.att[0] * b_xt->origin.att[3]);
        h_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        for (i32 = 0; i32 < 3; i32++) {
          b_xt->robot_state.vel[i32] = b_new_origin_att_rel[i32];
          fv27[i32] = 0.0F;
          for (i34 = 0; i34 < 3; i34++) {
            fv27[i32] += h_xt[i32 + 3 * i34] * new_origin_pos_rel[i34];
          }
        }

        mw_neon_mm_add_f32x4(b_xt->origin.pos, 3, 1, fv27,
                             &b_new_origin_att_rel[0]);

        //  in world frame
        // 'SLAM_upd:438' xt.origin.att = QuatFromRotJ(new_origin_att_rel * RotFromQuatJ(xt.origin.att)); 
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        i_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        i_xt[3] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                          b_xt->origin.att[2] * b_xt->origin.att[3]);
        i_xt[6] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                          b_xt->origin.att[1] * b_xt->origin.att[3]);
        i_xt[1] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                          b_xt->origin.att[2] * b_xt->origin.att[3]);
        i_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        i_xt[7] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                          b_xt->origin.att[0] * b_xt->origin.att[3]);
        i_xt[2] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                          b_xt->origin.att[1] * b_xt->origin.att[3]);
        i_xt[5] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                          b_xt->origin.att[0] * b_xt->origin.att[3]);
        i_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        for (i = 0; i < 3; i++) {
          b_xt->origin.pos[i] = b_new_origin_att_rel[i];
          for (i32 = 0; i32 < 3; i32++) {
            b_c[i + 3 * i32] = 0.0F;
            for (i34 = 0; i34 < 3; i34++) {
              b_c[i + 3 * i32] += new_origin_att_rel[i + 3 * i34] * i_xt[i34 + 3
                * i32];
            }
          }
        }

        //  THIS IS OK, It is according to the NASA memo found
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
        // % speed optimization
        // 'QuatFromRotJ:50' [~, index] = max([1+R(1,1)-(R(2,2)+R(3,3)), 1+R(2,2)-(R(1,1)+R(3,3)), 1+R(3,3)-(R(1,1)+R(2,2)), 1+(R(1,1)+R(2,2)+R(3,3))]); 
        varargin_1[0] = (1.0F + b_c[0]) - (b_c[4] + b_c[8]);
        varargin_1[1] = (1.0F + b_c[4]) - (b_c[0] + b_c[8]);
        varargin_1[2] = (1.0F + b_c[8]) - (b_c[0] + b_c[4]);
        varargin_1[3] = 1.0F + ((b_c[0] + b_c[4]) + b_c[8]);
        yk = 1;
        mtmp = varargin_1[0];
        idx = 1;
        if (rtIsNaNF(varargin_1[0])) {
          ix = 2;
          exitg1 = false;
          while ((!exitg1) && (ix < 5)) {
            yk = ix;
            if (!rtIsNaNF(varargin_1[ix - 1])) {
              mtmp = varargin_1[ix - 1];
              idx = ix;
              exitg1 = true;
            } else {
              ix++;
            }
          }
        }

        if (yk < 4) {
          while (yk + 1 < 5) {
            if (varargin_1[yk] > mtmp) {
              mtmp = varargin_1[yk];
              idx = yk + 1;
            }

            yk++;
          }
        }

        // 'QuatFromRotJ:50' ~
        // 'QuatFromRotJ:52' if(index==1)
        if (idx == 1) {
          // 'QuatFromRotJ:53' Q = [sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2;
          // 'QuatFromRotJ:54'         (R(1,2)+R(2,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:55'         (R(1,3)+R(3,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:56'         (R(2,3)-R(3,2))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
          b_xt->origin.att[0] = sqrtf((1.0F + 2.0F * b_c[0]) - ((b_c[0] + b_c[4])
            + b_c[8])) / 2.0F;
          b_xt->origin.att[1] = (b_c[3] + b_c[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[2] = (b_c[6] + b_c[2]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[3] = (b_c[7] - b_c[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[0]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
        } else if (idx == 2) {
          // 'QuatFromRotJ:57' elseif(index==2)
          // 'QuatFromRotJ:58' Q = [(R(1,2)+R(2,1))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:59'         (sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2); 
          // 'QuatFromRotJ:60'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:61'         (R(3,1)-R(1,3))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
          b_xt->origin.att[0] = (b_c[3] + b_c[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[1] = sqrtf((1.0F + 2.0F * b_c[4]) - ((b_c[0] + b_c[4])
            + b_c[8])) / 2.0F;
          b_xt->origin.att[2] = (b_c[7] + b_c[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[3] = (b_c[2] - b_c[6]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[4]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
        } else if (idx == 3) {
          // 'QuatFromRotJ:62' elseif(index==3)
          // 'QuatFromRotJ:63' Q = [(R(1,3)+R(3,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:64'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:65'         (sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2); 
          // 'QuatFromRotJ:66'         (R(1,2)-R(2,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
          b_xt->origin.att[0] = (b_c[6] + b_c[2]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[1] = (b_c[7] + b_c[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[2] = sqrtf((1.0F + 2.0F * b_c[8]) - ((b_c[0] + b_c[4])
            + b_c[8])) / 2.0F;
          b_xt->origin.att[3] = (b_c[3] - b_c[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
            b_c[8]) - ((b_c[0] + b_c[4]) + b_c[8])) / 2.0F));
        } else {
          // 'QuatFromRotJ:67' else
          // 'QuatFromRotJ:68' Q = [(R(2,3)-R(3,2))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:69'         (R(3,1)-R(1,3))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:70'         (R(1,2)-R(2,1))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
          // 'QuatFromRotJ:71'         (sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2);];
          b_xt->origin.att[0] = (b_c[7] - b_c[5]) / (4.0F * (sqrtf(1.0F + ((b_c
            [0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[1] = (b_c[2] - b_c[6]) / (4.0F * (sqrtf(1.0F + ((b_c
            [0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[2] = (b_c[3] - b_c[1]) / (4.0F * (sqrtf(1.0F + ((b_c
            [0] + b_c[4]) + b_c[8])) / 2.0F));
          b_xt->origin.att[3] = sqrtf(1.0F + ((b_c[0] + b_c[4]) + b_c[8])) /
            2.0F;
        }

        //  in world frame
      }
    }
  }

  // % aposteriori measurement prediction
  // 'SLAM_upd:444' [map] = getMap(xt);
  getMap(b_xt->origin.pos, b_xt->origin.att, b_xt->anchor_states, b_map);

  //  get map for output
  // 'SLAM_upd:446' delayedStatus = zeros(size(updateVect));
  memset(&b_delayedStatus[0], 0, 48U * sizeof(float));

  // 'SLAM_upd:447' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'SLAM_upd:448' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'SLAM_upd:449' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
          && (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              2)) {
        // 'SLAM_upd:450' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
        // 'SLAM_upd:451' delayedStatus(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
        b_delayedStatus[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
          .status_idx - 1] = 1.0F;
      }
    }
  }
}

//
// Arguments    : const boolean_T x[48]
// Return Type  : boolean_T
//
static boolean_T any(const boolean_T x[48])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 48)) {
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

  // 'anyActiveAnchorFeatures:3' ret = false;
  ret = false;

  // 'anyActiveAnchorFeatures:4' for featureIdx = 1:numPointsPerAnchor
  featureIdx = 1;
  exitg1 = false;
  while ((!exitg1) && (featureIdx < 9)) {
    // 'anyActiveAnchorFeatures:5' if anchor_state.feature_states(featureIdx).status == 1 
    if (anchor_state_feature_states[featureIdx - 1].status == 1) {
      // 'anyActiveAnchorFeatures:6' ret = true;
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
// Arguments    : const float xt_robot_state_pos[3]
//                const float xt_robot_state_att[4]
//                const f_struct_T xt_anchor_states[6]
//                const float z_all_l[96]
//                const float z_all_r[96]
//                const boolean_T b_status[48]
//                const float c_stereoParams_CameraParameters[2]
//                const float d_stereoParams_CameraParameters[2]
//                const float e_stereoParams_CameraParameters[2]
//                const float f_stereoParams_CameraParameters[2]
//                const float stereoParams_r_lr[3]
//                const float stereoParams_R_rl[9]
//                boolean_T VIOParameters_full_stereo
//                emxArray_real32_T *r
//                emxArray_real32_T *H
// Return Type  : void
//
static void b_getH_R_res(const float xt_robot_state_pos[3], const float
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[6], const float
  z_all_l[96], const float z_all_r[96], const boolean_T b_status[48], const
  float c_stereoParams_CameraParameters[2], const float
  d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], boolean_T VIOParameters_full_stereo,
  emxArray_real32_T *r, emxArray_real32_T *H)
{
  float fx_l;
  float fy_l;
  float fx_r;
  float fy_r;
  int nm1d2;
  int cdiff;
  int residualDim;
  float R_cw[9];
  float c;
  int i15;
  float res_idx;
  int anchorIdx;
  float anchorRot[9];
  int featureIdx;
  int b_c;
  int apnd;
  __attribute__((aligned(16))) float z_curr_l[2];
  __attribute__((aligned(16))) float z_curr_r[2];
  float fp_l[3];
  __attribute__((aligned(16))) float fv9[3];
  __attribute__((aligned(16))) float fp_r[3];
  float h_ci_r[3];
  __attribute__((aligned(16))) float h_u_r[2];
  __attribute__((aligned(16))) float h_u_l[2];
  int ndbl;
  float vec_data[4];
  int tmp_data[4];
  __attribute__((aligned(16))) float h_u_r_To_h_ci_r[2];
  __attribute__((aligned(16))) float h_u_l_To_h_ci_l[2];
  __attribute__((aligned(16))) float fv10[2];
  float b_h_u_l_To_h_ci_l[6];
  float b_h_u_r_To_h_ci_r[6];
  float fp_r_norm_inv;
  float fp_r_32_inv;
  float h_ci_r_To_fp_r[9];
  float h_ci_l_To_r_wc[9];
  float h_ci_l_To_R_cw[9];
  float fp_l_To_r_wc[9];
  float fp_l_To_R_cw[9];
  float h_ci_l_To_anchorPos[9];
  float h_ci_l_To_anchorRot[9];
  float fp_l_To_anchorPos[9];
  float fp_l_To_anchorRot[9];
  float b_R_cw[9];
  float c_R_cw[9];
  float f2;
  float fv11[9];
  float fv12[9];
  float d_R_cw[9];
  int b_tmp_data[4];
  float c_h_u_r_To_h_ci_r[6];
  float d_h_u_r_To_h_ci_r[6];
  float c_h_u_l_To_h_ci_l[6];
  float e_h_u_r_To_h_ci_r[6];
  float d_h_u_l_To_h_ci_l[12];

  // 'getH_R_res:26' fx_l = stereoParams.CameraParameters1.FocalLength(1);
  fx_l = c_stereoParams_CameraParameters[0];

  // 'getH_R_res:27' fy_l = stereoParams.CameraParameters1.FocalLength(2);
  fy_l = c_stereoParams_CameraParameters[1];

  // 'getH_R_res:29' fx_r = stereoParams.CameraParameters2.FocalLength(1);
  fx_r = e_stereoParams_CameraParameters[0];

  // 'getH_R_res:30' fy_r = stereoParams.CameraParameters2.FocalLength(2);
  fy_r = e_stereoParams_CameraParameters[1];

  // 'getH_R_res:32' numErrorStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'getH_R_res:34' numMeas = nnz(status);
  nm1d2 = 0;
  for (cdiff = 0; cdiff < 48; cdiff++) {
    if (b_status[cdiff]) {
      nm1d2++;
    }
  }

  // 'getH_R_res:35' if VIOParameters.full_stereo
  if (VIOParameters_full_stereo) {
    // 'getH_R_res:36' residualDim = 4;
    residualDim = 4;
  } else {
    // 'getH_R_res:37' else
    // 'getH_R_res:38' residualDim = 2;
    residualDim = 2;
  }

  // 'getH_R_res:41' R_cw = RotFromQuatJ(xt.robot_state.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_cw[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[3] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1] +
                    xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[6] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2] -
                    xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[1] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1] -
                    xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[7] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2] +
                    xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[2] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2] +
                    xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[5] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2] -
                    xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
              xt_robot_state_att[1] * xt_robot_state_att[1]) +
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];

  // 'getH_R_res:42' r_wc = xt.robot_state.pos;
  // 'getH_R_res:44' H = zeros(numMeas*residualDim, numStates + numAnchors*numErrorStatesPerAnchor); 
  c = (float)nm1d2 * (float)residualDim;
  i15 = H->size[0] * H->size[1];
  H->size[0] = (int)c;
  H->size[1] = 102;
  emxEnsureCapacity((emxArray__common *)H, i15, (int)sizeof(float));
  cdiff = (int)c * 102;
  for (i15 = 0; i15 < cdiff; i15++) {
    H->data[i15] = 0.0F;
  }

  // 'getH_R_res:45' r = zeros(numMeas*residualDim, 1);
  c = (float)nm1d2 * (float)residualDim;
  i15 = r->size[0];
  r->size[0] = (int)c;
  emxEnsureCapacity((emxArray__common *)r, i15, (int)sizeof(float));
  cdiff = (int)c;
  for (i15 = 0; i15 < cdiff; i15++) {
    r->data[i15] = 0.0F;
  }

  // 'getH_R_res:47' ind = int32(zeros(numMeas, 2));
  // 'getH_R_res:49' res_idx = 1;
  res_idx = 1.0F;

  // 'getH_R_res:51' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'getH_R_res:52' anchorPos = xt.anchor_states(anchorIdx).pos;
    // 'getH_R_res:53' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    anchorRot[0] = ((xt_anchor_states[anchorIdx].att[0] *
                     xt_anchor_states[anchorIdx].att[0] -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) +
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) +
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];

    // 'getH_R_res:55' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getH_R_res:57' idx = xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx; 
      // 'getH_R_res:59' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status && status(idx) 
      if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) &&
          b_status[xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1]) {
        // 'getH_R_res:60' z_curr_l = z_all_l((idx-1)*2 + int32(1:2));
        cdiff = xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx;
        nm1d2 = cdiff - 1;
        if ((cdiff < 0) && (nm1d2 >= 0)) {
          nm1d2 = MIN_int32_T;
        }

        if (nm1d2 > 1073741823) {
          b_c = MAX_int32_T;
        } else if (nm1d2 <= -1073741824) {
          b_c = MIN_int32_T;
        } else {
          b_c = nm1d2 << 1;
        }

        // 'getH_R_res:61' z_curr_r = z_all_r((idx-1)*2 + int32(1:2));
        cdiff = xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx;
        nm1d2 = cdiff - 1;
        if ((cdiff < 0) && (nm1d2 >= 0)) {
          nm1d2 = MIN_int32_T;
        }

        if (nm1d2 > 1073741823) {
          apnd = MAX_int32_T;
        } else if (nm1d2 <= -1073741824) {
          apnd = MIN_int32_T;
        } else {
          apnd = nm1d2 << 1;
        }

        for (i15 = 0; i15 < 2; i15++) {
          cdiff = 1 + i15;
          nm1d2 = b_c + cdiff;
          if ((b_c > 0) && (nm1d2 <= 0)) {
            nm1d2 = MAX_int32_T;
          }

          z_curr_l[i15] = z_all_l[nm1d2 - 1];
          cdiff = 1 + i15;
          nm1d2 = apnd + cdiff;
          if ((apnd > 0) && (nm1d2 <= 0)) {
            nm1d2 = MAX_int32_T;
          }

          z_curr_r[i15] = z_all_r[nm1d2 - 1];
        }

        // 'getH_R_res:63' h_ci_l = xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point; 
        // 'getH_R_res:64' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getH_R_res:65' fp_l = h_ci_l/rho;
        for (i15 = 0; i15 < 3; i15++) {
          fp_l[i15] = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i15] / xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
        }

        // 'getH_R_res:66' fp_r = stereoParams.R_rl*fp_l - stereoParams.r_lr;
        for (i15 = 0; i15 < 3; i15++) {
          fv9[i15] = 0.0F;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            fv9[i15] += stereoParams_R_rl[i15 + 3 * cdiff] * fp_l[cdiff];
          }
        }

        mw_neon_mm_sub_f32x4(fv9, 3, 1, *(float (*)[3])&stereoParams_r_lr[0],
                             &fp_r[0]);

        // 'getH_R_res:67' h_ci_r = fp_r / norm(fp_r);
        c = norm(fp_r);
        for (cdiff = 0; cdiff < 3; cdiff++) {
          h_ci_r[cdiff] = fp_r[cdiff] / c;
        }

        //  necessary?
        // 'getH_R_res:69' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getH_R_res:71' if VIOParameters.full_stereo
        if (VIOParameters_full_stereo) {
          // 'getH_R_res:73' [ h_u_l, h_u_r ] = predictMeasurementStereo( fp_l, stereoParams ); 
          predictMeasurementStereo(fp_l, c_stereoParams_CameraParameters,
            d_stereoParams_CameraParameters, e_stereoParams_CameraParameters,
            f_stereoParams_CameraParameters, stereoParams_r_lr,
            stereoParams_R_rl, h_u_l, h_u_r);

          // 'getH_R_res:75' r((res_idx-1)*residualDim + (1:residualDim)) = [z_curr_l - h_u_l; z_curr_r - h_u_r]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            tmp_data[i15] = (int)(c + vec_data[i15]);
          }

          mw_neon_mm_sub_f32x4(z_curr_r, 2, 1, h_u_r, &h_u_r_To_h_ci_r[0]);
          mw_neon_mm_sub_f32x4(z_curr_l, 2, 1, h_u_l, &h_u_l_To_h_ci_l[0]);
          for (i15 = 0; i15 < 2; i15++) {
            vec_data[i15] = h_u_l_To_h_ci_l[i15];
            vec_data[i15 + 2] = h_u_r_To_h_ci_r[i15];
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            r->data[tmp_data[i15] - 1] = vec_data[i15];
          }
        } else {
          // 'getH_R_res:76' else
          // 'getH_R_res:77' h_u_l = predictMeasurementMono( fp_l, stereoParams.CameraParameters1 ); 
          // predictMeasurementLeft Predict the measurement of a feature given in the left 
          // camera frame
          //    Get the normalized pixel coordinates where a feature given in the left camera 
          //    frame
          // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
          // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
          // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1);
          // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2);
          // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
          for (cdiff = 0; cdiff < 2; cdiff++) {
            z_curr_r[cdiff] = fp_l[cdiff] / fp_l[2];
          }

          //  normalized feature in camera frame
          // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
          // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
          // 'getH_R_res:79' r((res_idx-1)*residualDim + (1:residualDim)) = z_curr_l - h_u_l; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            tmp_data[i15] = (int)(c + vec_data[i15]);
          }

          fv10[0] = z_curr_r[0] * c_stereoParams_CameraParameters[0] +
            d_stereoParams_CameraParameters[0];
          fv10[1] = z_curr_r[1] * c_stereoParams_CameraParameters[1] +
            d_stereoParams_CameraParameters[1];
          mw_neon_mm_sub_f32x4(z_curr_l, 2, 1, fv10, &h_u_r_To_h_ci_r[0]);
          for (i15 = 0; i15 < ndbl; i15++) {
            r->data[tmp_data[i15] - 1] = h_u_r_To_h_ci_r[i15];
          }
        }

        // 'getH_R_res:82' h_u_l_To_h_ci_l = [fx_l/h_ci_l(3),              0, -fx_l*(h_ci_l(1))/h_ci_l(3)^2; 
        // 'getH_R_res:83'                                             0, fy_l/h_ci_l(3), -fy_l*(h_ci_l(2))/h_ci_l(3)^2]; 
        b_h_u_l_To_h_ci_l[0] = fx_l / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        b_h_u_l_To_h_ci_l[2] = 0.0F;
        b_h_u_l_To_h_ci_l[4] = -fx_l * xt_anchor_states[anchorIdx].
          feature_states[featureIdx].scaled_map_point[0] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        b_h_u_l_To_h_ci_l[1] = 0.0F;
        b_h_u_l_To_h_ci_l[3] = fy_l / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        b_h_u_l_To_h_ci_l[5] = -fy_l * xt_anchor_states[anchorIdx].
          feature_states[featureIdx].scaled_map_point[1] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);

        // 'getH_R_res:86' h_u_r_To_h_ci_r = [fx_r/h_ci_r(3), 0,              -fx_r*(h_ci_r(1))/h_ci_r(3)^2; 
        // 'getH_R_res:87'                                             0, fy_r/h_ci_r(3), -fy_r*(h_ci_r(2))/h_ci_r(3)^2]; 
        b_h_u_r_To_h_ci_r[0] = fx_r / h_ci_r[2];
        b_h_u_r_To_h_ci_r[2] = 0.0F;
        b_h_u_r_To_h_ci_r[4] = -fx_r * h_ci_r[0] / (h_ci_r[2] * h_ci_r[2]);
        b_h_u_r_To_h_ci_r[1] = 0.0F;
        b_h_u_r_To_h_ci_r[3] = fy_r / h_ci_r[2];
        b_h_u_r_To_h_ci_r[5] = -fy_r * h_ci_r[1] / (h_ci_r[2] * h_ci_r[2]);

        // 'getH_R_res:89' fp_r_norm_inv = 1/norm(fp_r);
        fp_r_norm_inv = 1.0F / norm(fp_r);

        // 'getH_R_res:90' fp_r_32_inv = fp_r_norm_inv^3;
        fp_r_32_inv = powf(fp_r_norm_inv, 3.0F);

        // 'getH_R_res:92' h_ci_r_To_fp_r = [fp_r_norm_inv - fp_r(1)*fp_r(1)*fp_r_32_inv,               - fp_r(1)*fp_r(2)*fp_r_32_inv,               - fp_r(1)*fp_r(3)*fp_r_32_inv; 
        // 'getH_R_res:93'                                             - fp_r(2)*fp_r(1)*fp_r_32_inv, fp_r_norm_inv - fp_r(2)*fp_r(2)*fp_r_32_inv,               - fp_r(2)*fp_r(3)*fp_r_32_inv; 
        // 'getH_R_res:94'                                             - fp_r(3)*fp_r(1)*fp_r_32_inv,               - fp_r(3)*fp_r(2)*fp_r_32_inv, fp_r_norm_inv - fp_r(3)*fp_r(3)*fp_r_32_inv]; 
        h_ci_r_To_fp_r[0] = fp_r_norm_inv - fp_r[0] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[3] = -fp_r[0] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[6] = -fp_r[0] * fp_r[2] * fp_r_32_inv;
        h_ci_r_To_fp_r[1] = -fp_r[1] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[4] = fp_r_norm_inv - fp_r[1] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[7] = -fp_r[1] * fp_r[2] * fp_r_32_inv;
        h_ci_r_To_fp_r[2] = -fp_r[2] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[5] = -fp_r[2] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[8] = fp_r_norm_inv - fp_r[2] * fp_r[2] * fp_r_32_inv;

        // necessary?
        // 'getH_R_res:96' fp_r_To_fp_l = stereoParams.R_rl;
        //              h_ci_l_To_r_wc = -rho*R_cw;
        //              h_ci_l_To_R_cw = skew(h_ci_l);
        // 'getH_R_res:101' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F)) {
          //  delayed initialization or feature behind anchor
          //                  H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:103' h_ci_l_To_r_wc = zeros(3);
          // 'getH_R_res:104' h_ci_l_To_R_cw = zeros(3);
          // 'getH_R_res:106' fp_l_To_r_wc = zeros(3);
          // 'getH_R_res:107' fp_l_To_R_cw = zeros(3);
          for (i15 = 0; i15 < 9; i15++) {
            h_ci_l_To_r_wc[i15] = 0.0F;
            h_ci_l_To_R_cw[i15] = 0.0F;
            fp_l_To_r_wc[i15] = 0.0F;
            fp_l_To_R_cw[i15] = 0.0F;
          }
        } else {
          // 'getH_R_res:108' else
          //                  H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:110' h_ci_l_To_r_wc = -rho*R_cw;
          // 'getH_R_res:111' h_ci_l_To_R_cw = skew(h_ci_l);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          h_ci_l_To_R_cw[0] = 0.0F;
          h_ci_l_To_R_cw[3] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
          h_ci_l_To_R_cw[6] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[1];
          h_ci_l_To_R_cw[1] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
          h_ci_l_To_R_cw[4] = 0.0F;
          h_ci_l_To_R_cw[7] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[0];
          h_ci_l_To_R_cw[2] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[1];
          h_ci_l_To_R_cw[5] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[0];
          h_ci_l_To_R_cw[8] = 0.0F;

          // 'getH_R_res:113' fp_l_To_r_wc = -R_cw;
          for (i15 = 0; i15 < 9; i15++) {
            h_ci_l_To_r_wc[i15] = -xt_anchor_states[anchorIdx]
              .feature_states[featureIdx].inverse_depth * R_cw[i15];
            fp_l_To_r_wc[i15] = -R_cw[i15];
          }

          // 'getH_R_res:114' fp_l_To_R_cw = skew(fp_l);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          fp_l_To_R_cw[0] = 0.0F;
          fp_l_To_R_cw[3] = -fp_l[2];
          fp_l_To_R_cw[6] = fp_l[1];
          fp_l_To_R_cw[1] = fp_l[2];
          fp_l_To_R_cw[4] = 0.0F;
          fp_l_To_R_cw[7] = -fp_l[0];
          fp_l_To_R_cw[2] = -fp_l[1];
          fp_l_To_R_cw[5] = fp_l[0];
          fp_l_To_R_cw[8] = 0.0F;
        }

        //             %% anchor state derivatives
        //              h_ci_l_To_anchorPos = rho*R_cw;
        //              h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //              h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        // 'getH_R_res:123' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F)) {
          //  delayed initialization or feature behind anchor
          //                  H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:125' h_ci_l_To_anchorPos = zeros(3);
          // 'getH_R_res:126' h_ci_l_To_anchorRot = zeros(3);
          for (i15 = 0; i15 < 9; i15++) {
            h_ci_l_To_anchorPos[i15] = 0.0F;
            h_ci_l_To_anchorRot[i15] = 0.0F;
          }

          // 'getH_R_res:127' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          mw_neon_mm_sub_f32x4(*(float (*)[3])&xt_anchor_states[anchorIdx].pos[0],
                               3, 1, *(float (*)[3])&xt_robot_state_pos[0],
                               &fv9[0]);
          for (i15 = 0; i15 < 3; i15++) {
            fp_r[i15] = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_r[i15] += R_cw[i15 + 3 * cdiff] * fv9[cdiff];
            }
          }

          // 'getH_R_res:129' fp_l_To_anchorPos = zeros(3);
          // 'getH_R_res:130' fp_l_To_anchorRot = zeros(3);
          for (i15 = 0; i15 < 9; i15++) {
            fp_l_To_anchorPos[i15] = 0.0F;
            fp_l_To_anchorRot[i15] = 0.0F;
          }

          // 'getH_R_res:131' fp_l_To_rho       = -R_cw * anchorRot'*m/rho^2;
          c = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i15] = -R_cw[cdiff + 3 * i15];
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            f2 = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_R_cw[i15 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_R_cw[i15 + 3 * cdiff] += b_R_cw[i15 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }

              f2 += c_R_cw[i15 + 3 * cdiff] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
            }

            fp_l[i15] = f2 / c;
          }
        } else {
          // 'getH_R_res:132' else
          //                  H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:134' h_ci_l_To_anchorPos = rho*R_cw;
          for (i15 = 0; i15 < 9; i15++) {
            h_ci_l_To_anchorPos[i15] = xt_anchor_states[anchorIdx].
              feature_states[featureIdx].inverse_depth * R_cw[i15];
          }

          // 'getH_R_res:135' h_ci_l_To_anchorRot = -R_cw * anchorRot'*skew(m);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i15] = -R_cw[cdiff + 3 * i15];
            }
          }

          fv11[0] = 0.0F;
          fv11[3] = -xt_anchor_states[anchorIdx].feature_states[featureIdx].m[2];
          fv11[6] = xt_anchor_states[anchorIdx].feature_states[featureIdx].m[1];
          fv11[1] = xt_anchor_states[anchorIdx].feature_states[featureIdx].m[2];
          fv11[4] = 0.0F;
          fv11[7] = -xt_anchor_states[anchorIdx].feature_states[featureIdx].m[0];
          fv11[2] = -xt_anchor_states[anchorIdx].feature_states[featureIdx].m[1];
          fv11[5] = xt_anchor_states[anchorIdx].feature_states[featureIdx].m[0];
          fv11[8] = 0.0F;
          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_R_cw[i15 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_R_cw[i15 + 3 * cdiff] += b_R_cw[i15 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              h_ci_l_To_anchorRot[i15 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                h_ci_l_To_anchorRot[i15 + 3 * cdiff] += c_R_cw[i15 + 3 * nm1d2] *
                  fv11[nm1d2 + 3 * cdiff];
              }
            }
          }

          // 'getH_R_res:136' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          mw_neon_mm_sub_f32x4(*(float (*)[3])&xt_anchor_states[anchorIdx].pos[0],
                               3, 1, *(float (*)[3])&xt_robot_state_pos[0],
                               &fv9[0]);
          for (i15 = 0; i15 < 3; i15++) {
            fp_r[i15] = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_r[i15] += R_cw[i15 + 3 * cdiff] * fv9[cdiff];
            }
          }

          // 'getH_R_res:138' fp_l_To_anchorPos = R_cw;
          for (i15 = 0; i15 < 9; i15++) {
            fp_l_To_anchorPos[i15] = R_cw[i15];
          }

          // 'getH_R_res:139' fp_l_To_anchorRot = -R_cw * anchorRot'*skew(m/rho); 
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i15 = 0; i15 < 3; i15++) {
            fp_l[i15] = xt_anchor_states[anchorIdx].feature_states[featureIdx]
              .m[i15] / xt_anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i15] = -R_cw[cdiff + 3 * i15];
            }
          }

          fv12[0] = 0.0F;
          fv12[3] = -fp_l[2];
          fv12[6] = fp_l[1];
          fv12[1] = fp_l[2];
          fv12[4] = 0.0F;
          fv12[7] = -fp_l[0];
          fv12[2] = -fp_l[1];
          fv12[5] = fp_l[0];
          fv12[8] = 0.0F;

          // 'getH_R_res:140' fp_l_To_rho       = -R_cw * anchorRot'*m/rho^2;
          c = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_R_cw[i15 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_R_cw[i15 + 3 * cdiff] += b_R_cw[i15 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_l_To_anchorRot[i15 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                fp_l_To_anchorRot[i15 + 3 * cdiff] += c_R_cw[i15 + 3 * nm1d2] *
                  fv12[nm1d2 + 3 * cdiff];
              }

              d_R_cw[cdiff + 3 * i15] = -R_cw[cdiff + 3 * i15];
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            f2 = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[i15 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_R_cw[i15 + 3 * cdiff] += d_R_cw[i15 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }

              f2 += b_R_cw[i15 + 3 * cdiff] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
            }

            fp_l[i15] = f2 / c;
          }
        }

        //              tmp = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        // 'getH_R_res:144' if VIOParameters.full_stereo
        if (VIOParameters_full_stereo) {
          // 'getH_R_res:145' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = [h_u_l_To_h_ci_l * h_ci_l_To_r_wc; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_r_wc]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i15 +
                  (nm1d2 << 1)] * h_ci_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              e_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                e_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += d_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * fp_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              d_h_u_l_To_h_ci_l[cdiff + (i15 << 2)] = c_h_u_l_To_h_ci_l[cdiff +
                (i15 << 1)];
              d_h_u_l_To_h_ci_l[(cdiff + (i15 << 2)) + 2] =
                e_h_u_r_To_h_ci_r[cdiff + (i15 << 1)];
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * i15] =
                d_h_u_l_To_h_ci_l[cdiff + ndbl * i15];
            }
          }

          // 'getH_R_res:146' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = [h_u_l_To_h_ci_l * h_ci_l_To_R_cw; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_R_cw]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i15 +
                  (nm1d2 << 1)] * h_ci_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              e_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                e_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += d_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * fp_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              d_h_u_l_To_h_ci_l[cdiff + (i15 << 2)] = c_h_u_l_To_h_ci_l[cdiff +
                (i15 << 1)];
              d_h_u_l_To_h_ci_l[(cdiff + (i15 << 2)) + 2] =
                e_h_u_r_To_h_ci_r[cdiff + (i15 << 1)];
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (3 + i15)] =
                d_h_u_l_To_h_ci_l[cdiff + ndbl * i15];
            }
          }

          // 'getH_R_res:147' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = [h_u_l_To_h_ci_l*h_ci_l_To_anchorPos; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_anchorPos]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          b_c = anchorIdx * 14 + 18;
          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i15 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              e_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                e_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += d_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * fp_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              d_h_u_l_To_h_ci_l[cdiff + (i15 << 2)] = c_h_u_l_To_h_ci_l[cdiff +
                (i15 << 1)];
              d_h_u_l_To_h_ci_l[(cdiff + (i15 << 2)) + 2] =
                e_h_u_r_To_h_ci_r[cdiff + (i15 << 1)];
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (i15 + b_c)] =
                d_h_u_l_To_h_ci_l[cdiff + ndbl * i15];
            }
          }

          // 'getH_R_res:148' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = [h_u_l_To_h_ci_l*h_ci_l_To_anchorRot; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_anchorRot]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          b_c = anchorIdx * 14 + 18;
          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i15 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              e_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                e_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += d_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * fp_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              d_h_u_l_To_h_ci_l[cdiff + (i15 << 2)] = c_h_u_l_To_h_ci_l[cdiff +
                (i15 << 1)];
              d_h_u_l_To_h_ci_l[(cdiff + (i15 << 2)) + 2] =
                e_h_u_r_To_h_ci_r[cdiff + (i15 << 1)];
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * ((i15 + b_c) + 3)] =
                d_h_u_l_To_h_ci_l[cdiff + ndbl * i15];
            }
          }

          // 'getH_R_res:149' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = [h_u_l_To_h_ci_l*h_ci_l_To_rho; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_rho]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            h_u_l_To_h_ci_l[i15] = 0.0F;
            h_u_r_To_h_ci_r[i15] = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i15 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
              }

              h_u_l_To_h_ci_l[i15] += b_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] *
                fp_r[cdiff];
              h_u_r_To_h_ci_r[i15] += d_h_u_r_To_h_ci_r[i15 + (cdiff << 1)] *
                fp_l[cdiff];
            }

            vec_data[i15] = h_u_l_To_h_ci_l[i15];
            vec_data[i15 + 2] = h_u_r_To_h_ci_r[i15];
          }

          nm1d2 = anchorIdx * 14 + featureIdx;
          for (i15 = 0; i15 < ndbl; i15++) {
            H->data[b_tmp_data[i15] + H->size[0] * (nm1d2 + 24)] = vec_data[i15];
          }
        } else {
          // 'getH_R_res:150' else
          // 'getH_R_res:151' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = h_u_l_To_h_ci_l * h_ci_l_To_r_wc; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i15 +
                  (nm1d2 << 1)] * h_ci_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * i15] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i15];
            }
          }

          // 'getH_R_res:152' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = h_u_l_To_h_ci_l * h_ci_l_To_R_cw; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i15 +
                  (nm1d2 << 1)] * h_ci_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (3 + i15)] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i15];
            }
          }

          // 'getH_R_res:153' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = h_u_l_To_h_ci_l*h_ci_l_To_anchorPos; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          b_c = anchorIdx * 14 + 18;
          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i15 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (i15 + b_c)] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i15];
            }
          }

          // 'getH_R_res:154' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = h_u_l_To_h_ci_l*h_ci_l_To_anchorRot; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          b_c = anchorIdx * 14 + 18;
          for (i15 = 0; i15 < 2; i15++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i15 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i15 = 0; i15 < 3; i15++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * ((i15 + b_c) + 3)] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i15];
            }
          }

          // 'getH_R_res:155' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = h_u_l_To_h_ci_l*h_ci_l_To_rho; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i15 = ndbl - 1;
            nm1d2 = i15 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i15 = 0; i15 < ndbl; i15++) {
            b_tmp_data[i15] = (int)(c + vec_data[i15]) - 1;
          }

          for (i15 = 0; i15 < 2; i15++) {
            h_u_l_To_h_ci_l[i15] = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              h_u_l_To_h_ci_l[i15] += b_h_u_l_To_h_ci_l[i15 + (cdiff << 1)] *
                fp_r[cdiff];
            }
          }

          nm1d2 = anchorIdx * 14 + featureIdx;
          for (i15 = 0; i15 < ndbl; i15++) {
            H->data[b_tmp_data[i15] + H->size[0] * (nm1d2 + 24)] =
              h_u_l_To_h_ci_l[i15];
          }
        }

        //              if coder.target('MATLAB')
        //
        //                  h_ci_l_To_r_wc = -rho*R_cw;
        //                  h_ci_l_To_R_cw = skew(h_ci_l);
        //
        //                  if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 % delayed initialization or feature behind anchor 
        //                      H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
        //                  else
        //                      H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
        //                  end
        //
        //                  %% anchor state derivatives
        //
        //                  h_ci_l_To_anchorPos = rho*R_cw;
        //                  h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //                  h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        //
        //                  if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 % delayed initialization or feature behind anchor 
        //                      H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
        //                  else
        //                      H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
        //                  end
        //
        //                  H_t = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        //
        //                  if any(any(abs(H((res_idx-1)*residualDim + (1:residualDim), :) - H_t) > 1e-8)) 
        //                      figure; imagesc(H_t - H((res_idx-1)*residualDim + (1:residualDim), :)) 
        //                      error('H inconsistency')
        //                  end
        //              end
        // 'getH_R_res:189' ind(res_idx, 1) = anchorIdx;
        // 'getH_R_res:190' ind(res_idx, 2) = featureIdx;
        // 'getH_R_res:192' res_idx = res_idx + 1;
        res_idx++;
      }
    }
  }
}

//
// log_error Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_ERROR does,
//  e.g. redefine ROS_ERROR
// Arguments    : void
// Return Type  : void
//
static void b_log_error()
{
  char cv20[14];
  int i18;
  static const char cv21[14] = { 'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't',
    'e', 'n', 'c', 'y', '\x00' };

  // 'log_error:6' if coder.target('MATLAB')
  // 'log_error:8' elseif ~coder.target('MEX')
  // 'log_error:9' coder.cinclude('<vio_logging.h>')
  // 'log_error:10' coder.ceval('LOG_ERROR', [str, 0], varargin{:});
  for (i18 = 0; i18 < 14; i18++) {
    cv20[i18] = cv21[i18];
  }

  LOG_ERROR(cv20);
}

//
// log_info Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
//  e.g. redefine ROS_INFO
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void b_log_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv14[44];
  int i12;
  static const char cv15[44] = { 'F', 'i', 'x', 'i', 'n', 'g', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', 'f', 'e', 'a', 't', 'u',
    'r', 'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r',
    ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i12 = 0; i12 < 44; i12++) {
      cv14[i12] = cv15[i12];
    }

    LOG_INFO(cv14, varargin_1, varargin_2, varargin_3);
  }
}

//
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : void
// Return Type  : void
//
static void b_log_warn()
{
  char cv18[44];
  int i17;
  static const char cv19[44] = { '1', '-', 'P', 'o', 'i', 'n', 't', ' ', 'R',
    'A', 'N', 'S', 'A', 'C', ' ', 'd', 'i', 'd', 'n', 't', ' ', 'f', 'i', 'n',
    'd', ' ', 'e', 'n', 'o', 'u', 'g', 'h', ' ', 'L', 'I', ' ', 'i', 'n', 'l',
    'i', 'e', 'r', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i17 = 0; i17 < 44; i17++) {
      cv18[i17] = cv19[i17];
    }

    LOG_WARN(cv18);
  }
}

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real32_T *x
//                int offset
//                int np
//                int nq
//                emxArray_int32_T *iwork
//                emxArray_real32_T *xwork
// Return Type  : void
//
static void b_merge(emxArray_int32_T *idx, emxArray_real32_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real32_T
                    *xwork)
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
// Arguments    : const float x[4]
// Return Type  : float
//
static float b_norm(const float x[4])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 4; k++) {
    absxk = fabsf(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
}

//
// Arguments    : float x[8]
//                int idx[8]
// Return Type  : void
//
static void b_sort(float x[8], int idx[8])
{
  int b_m;
  float x4[4];
  signed char idx4[4];
  float xwork[8];
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
    x4[b_m] = 0.0F;
    idx4[b_m] = 0;
  }

  for (b_m = 0; b_m < 8; b_m++) {
    xwork[b_m] = 0.0F;
  }

  nNaNs = -7;
  ib = 0;
  for (k = 0; k < 8; k++) {
    if (rtIsNaNF(x[k])) {
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
// Arguments    : int n
//                float a
//                const float x[36]
//                int ix0
//                float y[6]
//                int iy0
// Return Type  : void
//
static void b_xaxpy(int n, float a, const float x[36], int ix0, float y[6], int
                    iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0F) {
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
// Arguments    : float A[30]
//                float tau[5]
//                int jpvt[5]
// Return Type  : void
//
static void b_xgeqp3(float A[30], float tau[5], int jpvt[5])
{
  float vn1[5];
  float vn2[5];
  int k;
  int iy;
  float work[5];
  float smax;
  float temp2;
  int itemp;
  float absxk;
  float t;
  int i;
  int i_i;
  int ix;
  int pvt;
  int i43;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  k = 1;
  for (iy = 0; iy < 5; iy++) {
    jpvt[iy] = 1 + iy;
    work[iy] = 0.0F;
    smax = 0.0F;
    temp2 = 1.17549435E-38F;
    for (itemp = k; itemp <= k + 5; itemp++) {
      absxk = fabsf(A[itemp - 1]);
      if (absxk > temp2) {
        t = temp2 / absxk;
        smax = 1.0F + smax * t * t;
        temp2 = absxk;
      } else {
        t = absxk / temp2;
        smax += t * t;
      }
    }

    smax = temp2 * sqrtf(smax);
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
    temp2 = 0.0F;
    smax = d_xnrm2(5 - i, A, i_i + 2);
    if (smax != 0.0F) {
      smax = hypotf(A[i_i], smax);
      if (A[i_i] >= 0.0F) {
        smax = -smax;
      }

      if (fabsf(smax) < 9.86076132E-32F) {
        itemp = 0;
        do {
          itemp++;
          i43 = i_i - i;
          for (k = i_i + 1; k + 1 <= i43 + 6; k++) {
            A[k] *= 1.01412048E+31F;
          }

          smax *= 1.01412048E+31F;
          absxk *= 1.01412048E+31F;
        } while (!(fabsf(smax) >= 9.86076132E-32F));

        smax = hypotf(absxk, d_xnrm2(5 - i, A, i_i + 2));
        if (absxk >= 0.0F) {
          smax = -smax;
        }

        temp2 = (smax - absxk) / smax;
        absxk = 1.0F / (absxk - smax);
        i43 = i_i - i;
        for (k = i_i + 1; k + 1 <= i43 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 9.86076132E-32F;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0F / (A[i_i] - smax);
        i43 = i_i - i;
        for (k = i_i + 1; k + 1 <= i43 + 6; k++) {
          A[k] *= absxk;
        }

        absxk = smax;
      }
    }

    tau[i] = temp2;
    A[i_i] = absxk;
    if (i + 1 < 5) {
      absxk = A[i_i];
      A[i_i] = 1.0F;
      i_ip1 = (i + (i + 1) * 6) + 1;
      if (tau[i] != 0.0F) {
        lastv = 6 - i;
        itemp = i_i - i;
        while ((lastv > 0) && (A[itemp + 5] == 0.0F)) {
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
              if (A[k - 1] != 0.0F) {
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
            work[iy - 1] = 0.0F;
          }

          iy = 0;
          i43 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i43; itemp += 6) {
            ix = i_i;
            smax = 0.0F;
            pvt = (itemp + lastv) - 1;
            for (k = itemp; k <= pvt; k++) {
              smax += A[k - 1] * A[ix];
              ix++;
            }

            work[iy] += smax;
            iy++;
          }
        }

        if (-tau[i] == 0.0F) {
        } else {
          itemp = i_ip1 - 1;
          pvt = 0;
          for (iy = 1; iy <= lastc; iy++) {
            if (work[pvt] != 0.0F) {
              smax = work[pvt] * -tau[i];
              ix = i_i;
              i43 = lastv + itemp;
              for (k = itemp; k + 1 <= i43; k++) {
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
      if (vn1[iy] != 0.0F) {
        smax = fabsf(A[i + 6 * iy]) / vn1[iy];
        smax = 1.0F - smax * smax;
        if (smax < 0.0F) {
          smax = 0.0F;
        }

        temp2 = vn1[iy] / vn2[iy];
        temp2 = smax * (temp2 * temp2);
        if (temp2 <= 0.000345266977F) {
          smax = 0.0F;
          if (5 - i == 1) {
            smax = fabsf(A[itemp]);
          } else {
            temp2 = 1.17549435E-38F;
            pvt = (itemp - i) + 5;
            while (itemp + 1 <= pvt) {
              absxk = fabsf(A[itemp]);
              if (absxk > temp2) {
                t = temp2 / absxk;
                smax = 1.0F + smax * t * t;
                temp2 = absxk;
              } else {
                t = absxk / temp2;
                smax += t * t;
              }

              itemp++;
            }

            smax = temp2 * sqrtf(smax);
          }

          vn1[iy] = smax;
          vn2[iy] = vn1[iy];
        } else {
          vn1[iy] *= sqrtf(smax);
        }
      }
    }
  }
}

//
// Arguments    : int n
//                const emxArray_real32_T *x
//                int ix0
// Return Type  : float
//
static float b_xnrm2(int n, const emxArray_real32_T *x, int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = fabsf(x->data[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabsf(x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrtf(y);
  }

  return y;
}

//
// Arguments    : const float x[6]
// Return Type  : boolean_T
//
static boolean_T c_any(const float x[6])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  boolean_T b0;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((x[k] == 0.0F) || rtIsNaNF(x[k])) {
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
// Arguments    : float varargin_1
// Return Type  : double
//
static double c_fprintf(float varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[9] = { '	', 'q', 'v', ':', ' ', '%', 'f', '\x0a',
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
// log_info Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
//  e.g. redefine ROS_INFO
// Arguments    : int varargin_1
//                int varargin_2
// Return Type  : void
//
static void c_log_info(int varargin_1, int varargin_2)
{
  char cv28[44];
  int i23;
  static const char cv29[44] = { 'S', 'u', 'c', 'c', 'e', 's', 's', 'f', 'u',
    'l', 'l', 'y', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a', 't', 'e',
    'd', ' ', '%', 'd', ' ', 'o', 'f', ' ', '%', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i23 = 0; i23 < 44; i23++) {
      cv28[i23] = cv29[i23];
    }

    LOG_INFO(cv28, varargin_1, varargin_2);
  }
}

//
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : int varargin_1
//                float varargin_2
// Return Type  : void
//
static void c_log_warn(int varargin_1, float varargin_2)
{
  char cv22[36];
  int i19;
  static const char cv23[36] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', 'i', 's', ' ', 'v', 'e', 'r', 'y', ' ', 'c', 'l', 'o', 's', 'e',
    '.', ' ', 'D', 'e', 'p', 't', 'h', ':', ' ', '%', 'f', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i19 = 0; i19 < 36; i19++) {
      cv22[i19] = cv23[i19];
    }

    LOG_WARN(cv22, varargin_1, varargin_2);
  }
}

//
// Arguments    : const float x[2]
// Return Type  : float
//
static float c_norm(const float x[2])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 2; k++) {
    absxk = fabsf(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
}

//
// Arguments    : emxArray_real32_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void c_sort(emxArray_real32_T *x, emxArray_int32_T *idx)
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
//                float a
//                const float x[6]
//                int ix0
//                float y[36]
//                int iy0
// Return Type  : void
//
static void c_xaxpy(int n, float a, const float x[6], int ix0, float y[36], int
                    iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0F) {
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
//                const emxArray_real32_T *x
//                int ix0
// Return Type  : float
//
static float c_xnrm2(int n, const emxArray_real32_T *x, int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = fabsf(x->data[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabsf(x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrtf(y);
  }

  return y;
}

//
// Arguments    : const struct_T x[6]
//                AnchorPose y[6]
// Return Type  : void
//
static void cast(const struct_T x[6], AnchorPose y[6])
{
  int j;
  int i;
  for (j = 0; j < 6; j++) {
    for (i = 0; i < 3; i++) {
      y[j].pos[i] = x[j].pos[i];
    }

    for (i = 0; i < 4; i++) {
      y[j].att[i] = x[j].att[i];
    }
  }
}

//
// Arguments    : const float a[3]
//                const float b[3]
//                float c[3]
// Return Type  : void
//
static void cross(const float a[3], const float b[3], float c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

//
// Arguments    : float varargin_1
// Return Type  : double
//
static double d_fprintf(float varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[9] = { '	', 'q', 'w', ':', ' ', '%', 'f', '\x0a',
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
// log_info Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
//  e.g. redefine ROS_INFO
// Arguments    : int varargin_1
// Return Type  : void
//
static void d_log_info(int varargin_1)
{
  char cv30[52];
  int i24;
  static const char cv31[52] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z',
    'i', 'n', 'g', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ',', ' ',
    'w', 'h', 'i', 'c', 'h', ' ', 'w', 'a', 's', ' ', 't', 'h', 'e', ' ', 'o',
    'r', 'i', 'g', 'i', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i24 = 0; i24 < 52; i24++) {
      cv30[i24] = cv31[i24];
    }

    LOG_INFO(cv30, varargin_1);
  }
}

//
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : signed char varargin_1
// Return Type  : void
//
static void d_log_warn(signed char varargin_1)
{
  char cv24[37];
  int i21;
  static const char cv25[37] = { 'B', 'a', 'd', ' ', 't', 'r', 'i', 'a', 'n',
    'g', 'u', 'l', 'a', 't', 'i', 'o', 'n', ' ', '(', 'n', 'a', 'n', ')', ' ',
    'f', 'o', 'r', ' ', 'p', 'o', 'i', 'n', 't', ' ', '%', 'd', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i21 = 0; i21 < 37; i21++) {
      cv24[i21] = cv25[i21];
    }

    LOG_WARN(cv24, varargin_1);
  }
}

//
// Arguments    : const float x[36]
// Return Type  : float
//
static float d_norm(const float x[36])
{
  float y;
  int k;
  boolean_T exitg1;
  float absx;
  float s[6];
  y = 0.0F;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 36)) {
    absx = fabsf(x[k]);
    if (rtIsNaNF(absx)) {
      y = ((real32_T)rtNaN);
      exitg1 = true;
    } else {
      if (absx > y) {
        y = absx;
      }

      k++;
    }
  }

  if ((!rtIsInfF(y)) && (!rtIsNaNF(y))) {
    svd(x, s);
    y = s[0];
  }

  return y;
}

//
// Arguments    : emxArray_real32_T *x
//                int dim
//                emxArray_int32_T *idx
// Return Type  : void
//
static void d_sort(emxArray_real32_T *x, int dim, emxArray_int32_T *idx)
{
  int i44;
  emxArray_real32_T *vwork;
  int vstride;
  int x_idx_0;
  int j;
  emxArray_int32_T *iidx;
  if (dim <= 1) {
    i44 = x->size[0];
  } else {
    i44 = 1;
  }

  emxInit_real32_T(&vwork, 1);
  vstride = vwork->size[0];
  vwork->size[0] = i44;
  emxEnsureCapacity((emxArray__common *)vwork, vstride, (int)sizeof(float));
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
  emxInit_int32_T(&iidx, 1);
  while (j + 1 <= vstride) {
    for (x_idx_0 = 0; x_idx_0 + 1 <= i44; x_idx_0++) {
      vwork->data[x_idx_0] = x->data[j + x_idx_0 * vstride];
    }

    sortIdx(vwork, iidx);
    for (x_idx_0 = 0; x_idx_0 + 1 <= i44; x_idx_0++) {
      x->data[j + x_idx_0 * vstride] = vwork->data[x_idx_0];
      idx->data[j + x_idx_0 * vstride] = iidx->data[x_idx_0];
    }

    j++;
  }

  emxFree_int32_T(&iidx);
  emxFree_real32_T(&vwork);
}

//
// Arguments    : int n
//                const float x[30]
//                int ix0
// Return Type  : float
//
static float d_xnrm2(int n, const float x[30], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n == 1) {
    y = fabsf(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabsf(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrtf(y);
  }

  return y;
}

//
// Arguments    : const float v[3]
//                float d[9]
// Return Type  : void
//
static void diag(const float v[3], float d[9])
{
  int j;
  for (j = 0; j < 9; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
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
  }

  return quotient;
}

//
// Arguments    : float varargin_1
// Return Type  : double
//
static double e_fprintf(float varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[10] = { '	', 'q', 'a', 'o', ':', ' ', '%', 'f', '\x0a',
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
// log_info Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
//  e.g. redefine ROS_INFO
// Arguments    : int varargin_1
// Return Type  : void
//
static void e_log_info(int varargin_1)
{
  char cv32[45];
  int i25;
  static const char cv33[45] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'd', ' ', 'i', 's', ' ', 't', 'o', 'o', ' ', 'f', 'a', 'r', ' ', 'a', 'w',
    'a', 'y', ' ', 't', 'o', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a',
    't', 'e', '.', '\\', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i25 = 0; i25 < 45; i25++) {
      cv32[i25] = cv33[i25];
    }

    LOG_INFO(cv32, varargin_1);
  }
}

//
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : int varargin_1
//                float varargin_2
// Return Type  : void
//
static void e_log_warn(int varargin_1, float varargin_2)
{
  char cv26[49];
  int i22;
  static const char cv27[49] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', 'i', 's', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a', 't',
    'e', 'd', ' ', 'v', 'e', 'r', 'y', ' ', 'c', 'l', 'o', 's', 'e', '.', ' ',
    'D', 'e', 'p', 't', 'h', ':', ' ', '%', 'f', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i22 = 0; i22 < 49; i22++) {
      cv26[i22] = cv27[i22];
    }

    LOG_WARN(cv26, varargin_1, varargin_2);
  }
}

//
// Arguments    : int n
//                const float x[36]
//                int ix0
// Return Type  : float
//
static float e_xnrm2(int n, const float x[36], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabsf(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
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
// Arguments    : emxArray_real32_T **pEmxArray
// Return Type  : void
//
static void emxFree_real32_T(emxArray_real32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real32_T *)NULL) {
    if (((*pEmxArray)->data != (float *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_real32_T *)NULL;
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
// Arguments    : emxArray_real32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int b_numDimensions)
{
  emxArray_real32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (float *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_real32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_real32_T1(emxArray_real32_T **pEmxArray, int b_numDimensions)
{
  emxArray_real32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (float *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : float varargin_1
// Return Type  : double
//
static double f_fprintf(float varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[10] = { '	', 'q', 'w', 'o', ':', ' ', '%', 'f', '\x0a',
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
// log_info Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
//  e.g. redefine ROS_INFO
// Arguments    : int varargin_1
//                float varargin_2
//                float varargin_3
// Return Type  : void
//
static void f_log_info(int varargin_1, float varargin_2, float varargin_3)
{
  char cv40[51];
  int i29;
  static const char cv41[51] = { 'F', 'o', 'r', 'c', 'i', 'n', 'g', ' ', 'a',
    'c', 't', 'i', 'v', 'a', 't', 'i', 'o', 'n', ' ', 'o', 'f', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i29 = 0; i29 < 51; i29++) {
      cv40[i29] = cv41[i29];
    }

    LOG_INFO(cv40, varargin_1, varargin_2, varargin_3);
  }
}

//
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : float varargin_1
// Return Type  : void
//
static void f_log_warn(float varargin_1)
{
  char cv34[62];
  int i26;
  static const char cv35[62] = { 'G', 'o', 't', ' ', '%', 'd', ' ', 'n', 'e',
    'w', ' ', 'f', 'e', 'a', 'u', 't', 'u', 'r', 'e', 's', ' ', 'b', 'u', 't',
    ' ', 'n', 'o', 't', ' ', 'e', 'n', 'o', 'u', 'g', 'h', ' ', 'f', 'o', 'r',
    ' ', 'a', ' ', 'n', 'e', 'w', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '(',
    'm', 'i', 'n', ' ', '%', 'd', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i26 = 0; i26 < 62; i26++) {
      cv34[i26] = cv35[i26];
    }

    LOG_WARN(cv34, varargin_1, 4);
  }
}

//
// Arguments    : int n
//                const float x[6]
//                int ix0
// Return Type  : float
//
static float f_xnrm2(int n, const float x[6], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabsf(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
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
// Arguments    : float varargin_1
// Return Type  : double
//
static double g_fprintf(float varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[12] = { '	', 'q', 'R', '_', 'c', 'i', ':', ' ', '%',
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
// log_info Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
//  e.g. redefine ROS_INFO
// Arguments    : int varargin_1
// Return Type  : void
//
static void g_log_info(int varargin_1)
{
  char cv44[28];
  int i31;
  static const char cv45[28] = { 'S', 'e', 't', 't', 'i', 'n', 'g', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ' ', 'a', 's', ' ', 'o', 'r', 'i',
    'g', 'i', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i31 = 0; i31 < 28; i31++) {
      cv44[i31] = cv45[i31];
    }

    LOG_INFO(cv44, varargin_1);
  }
}

//
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void g_log_warn(int varargin_1, int varargin_2, int varargin_3)
{
  char cv36[57];
  int i27;
  static const char cv37[57] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r',
    ' ', '%', 'i', ')', ' ', 'c', 'o', 'n', 'v', 'e', 'r', 'g', 'e', 'd', ' ',
    'b', 'e', 'h', 'i', 'n', 'd', ' ', 'i', 't', 's', ' ', 'a', 'n', 'c', 'h',
    'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i27 = 0; i27 < 57; i27++) {
      cv36[i27] = cv37[i27];
    }

    LOG_WARN(cv36, varargin_1, varargin_2, varargin_3);
  }
}

//
// getAnchorPoses Get the anchor poses in the world frame
// Arguments    : const float xt_origin_pos[3]
//                const float xt_origin_att[4]
//                const f_struct_T xt_anchor_states[6]
//                struct_T anchor_poses[6]
// Return Type  : void
//
static void getAnchorPoses(const float xt_origin_pos[3], const float
  xt_origin_att[4], const f_struct_T xt_anchor_states[6], struct_T anchor_poses
  [6])
{
  float R_ow[9];
  int anchorIdx;
  __attribute__((aligned(16))) float fv14[3];
  int ixstart;
  int itmp;
  float b_xt_anchor_states[9];
  float c[9];
  int ix;
  float varargin_1[4];
  float mtmp;
  boolean_T exitg1;

  // 'getAnchorPoses:4' r_ow = xt.origin.pos;
  // 'getAnchorPoses:5' R_ow = RotFromQuatJ(xt.origin.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[6] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[1] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[2] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[5] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];

  // 'getAnchorPoses:7' anchor_pose.pos = zeros(3,1);
  // 'getAnchorPoses:8' anchor_pose.att = zeros(4,1);
  // 'getAnchorPoses:10' anchor_poses = repmat(anchor_pose, numAnchors, 1);
  // 'getAnchorPoses:12' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'getAnchorPoses:13' anchor_poses(anchorIdx).pos = r_ow + R_ow' * xt.anchor_states(anchorIdx).pos; 
    for (ixstart = 0; ixstart < 3; ixstart++) {
      fv14[ixstart] = 0.0F;
      for (itmp = 0; itmp < 3; itmp++) {
        fv14[ixstart] += R_ow[itmp + 3 * ixstart] * xt_anchor_states[anchorIdx].
          pos[itmp];
      }
    }

    mw_neon_mm_add_f32x4(*(float (*)[3])&xt_origin_pos[0], 3, 1, fv14,
                         &anchor_poses[anchorIdx].pos[0]);

    // 'getAnchorPoses:14' anchor_poses(anchorIdx).att = QuatFromRotJ(RotFromQuatJ(xt.anchor_states(anchorIdx).att) * R_ow); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    b_xt_anchor_states[0] = ((xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[3] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[6] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[1] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[7] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[2] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[5] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    for (ixstart = 0; ixstart < 3; ixstart++) {
      for (itmp = 0; itmp < 3; itmp++) {
        c[ixstart + 3 * itmp] = 0.0F;
        for (ix = 0; ix < 3; ix++) {
          c[ixstart + 3 * itmp] += b_xt_anchor_states[ixstart + 3 * ix] *
            R_ow[ix + 3 * itmp];
        }
      }
    }

    //  THIS IS OK, It is according to the NASA memo found
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
    // % speed optimization
    // 'QuatFromRotJ:50' [~, index] = max([1+R(1,1)-(R(2,2)+R(3,3)), 1+R(2,2)-(R(1,1)+R(3,3)), 1+R(3,3)-(R(1,1)+R(2,2)), 1+(R(1,1)+R(2,2)+R(3,3))]); 
    varargin_1[0] = (1.0F + c[0]) - (c[4] + c[8]);
    varargin_1[1] = (1.0F + c[4]) - (c[0] + c[8]);
    varargin_1[2] = (1.0F + c[8]) - (c[0] + c[4]);
    varargin_1[3] = 1.0F + ((c[0] + c[4]) + c[8]);
    ixstart = 1;
    mtmp = varargin_1[0];
    itmp = 1;
    if (rtIsNaNF(varargin_1[0])) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix < 5)) {
        ixstart = ix;
        if (!rtIsNaNF(varargin_1[ix - 1])) {
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

    // 'QuatFromRotJ:50' ~
    // 'QuatFromRotJ:52' if(index==1)
    if (itmp == 1) {
      // 'QuatFromRotJ:53' Q = [sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2;
      // 'QuatFromRotJ:54'         (R(1,2)+R(2,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:55'         (R(1,3)+R(3,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:56'         (R(2,3)-R(3,2))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
      anchor_poses[anchorIdx].att[0] = sqrtf((1.0F + 2.0F * c[0]) - ((c[0] + c[4])
        + c[8])) / 2.0F;
      anchor_poses[anchorIdx].att[1] = (c[3] + c[1]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[0]) - ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[2] = (c[6] + c[2]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[0]) - ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[3] = (c[7] - c[5]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[0]) - ((c[0] + c[4]) + c[8])) / 2.0F));
    } else if (itmp == 2) {
      // 'QuatFromRotJ:57' elseif(index==2)
      // 'QuatFromRotJ:58' Q = [(R(1,2)+R(2,1))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:59'         (sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2);
      // 'QuatFromRotJ:60'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:61'         (R(3,1)-R(1,3))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
      anchor_poses[anchorIdx].att[0] = (c[3] + c[1]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[4]) - ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[1] = sqrtf((1.0F + 2.0F * c[4]) - ((c[0] + c[4])
        + c[8])) / 2.0F;
      anchor_poses[anchorIdx].att[2] = (c[7] + c[5]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[4]) - ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[3] = (c[2] - c[6]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[4]) - ((c[0] + c[4]) + c[8])) / 2.0F));
    } else if (itmp == 3) {
      // 'QuatFromRotJ:62' elseif(index==3)
      // 'QuatFromRotJ:63' Q = [(R(1,3)+R(3,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:64'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:65'         (sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2);
      // 'QuatFromRotJ:66'         (R(1,2)-R(2,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
      anchor_poses[anchorIdx].att[0] = (c[6] + c[2]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[8]) - ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[1] = (c[7] + c[5]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[8]) - ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[2] = sqrtf((1.0F + 2.0F * c[8]) - ((c[0] + c[4])
        + c[8])) / 2.0F;
      anchor_poses[anchorIdx].att[3] = (c[3] - c[1]) / (4.0F * (sqrtf((1.0F +
        2.0F * c[8]) - ((c[0] + c[4]) + c[8])) / 2.0F));
    } else {
      // 'QuatFromRotJ:67' else
      // 'QuatFromRotJ:68' Q = [(R(2,3)-R(3,2))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:69'         (R(3,1)-R(1,3))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:70'         (R(1,2)-R(2,1))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:71'         (sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2);];
      anchor_poses[anchorIdx].att[0] = (c[7] - c[5]) / (4.0F * (sqrtf(1.0F +
        ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[1] = (c[2] - c[6]) / (4.0F * (sqrtf(1.0F +
        ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[2] = (c[3] - c[1]) / (4.0F * (sqrtf(1.0F +
        ((c[0] + c[4]) + c[8])) / 2.0F));
      anchor_poses[anchorIdx].att[3] = sqrtf(1.0F + ((c[0] + c[4]) + c[8])) /
        2.0F;
    }
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
// Arguments    : const float xt_robot_state_pos[3]
//                const float xt_robot_state_att[4]
//                const f_struct_T xt_anchor_states[6]
//                const float z_all_l[96]
//                const float z_all_r[96]
//                const boolean_T b_status[48]
//                const float c_stereoParams_CameraParameters[2]
//                const float d_stereoParams_CameraParameters[2]
//                const float e_stereoParams_CameraParameters[2]
//                const float f_stereoParams_CameraParameters[2]
//                const float stereoParams_r_lr[3]
//                const float stereoParams_R_rl[9]
//                boolean_T VIOParameters_full_stereo
//                emxArray_real32_T *r
//                emxArray_real32_T *H
//                emxArray_int32_T *ind
// Return Type  : void
//
static void getH_R_res(const float xt_robot_state_pos[3], const float
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[6], const float
  z_all_l[96], const float z_all_r[96], const boolean_T b_status[48], const
  float c_stereoParams_CameraParameters[2], const float
  d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], boolean_T VIOParameters_full_stereo,
  emxArray_real32_T *r, emxArray_real32_T *H, emxArray_int32_T *ind)
{
  float fx_l;
  float fy_l;
  float fx_r;
  float fy_r;
  int nm1d2;
  int cdiff;
  int residualDim;
  float R_cw[9];
  float c;
  int i13;
  float res_idx;
  int anchorIdx;
  float anchorRot[9];
  int featureIdx;
  int b_c;
  int apnd;
  __attribute__((aligned(16))) float z_curr_l[2];
  __attribute__((aligned(16))) float z_curr_r[2];
  float fp_l[3];
  __attribute__((aligned(16))) float fv4[3];
  __attribute__((aligned(16))) float fp_r[3];
  float h_ci_r[3];
  __attribute__((aligned(16))) float h_u_r[2];
  __attribute__((aligned(16))) float h_u_l[2];
  int ndbl;
  float vec_data[4];
  int tmp_data[4];
  __attribute__((aligned(16))) float h_u_r_To_h_ci_r[2];
  __attribute__((aligned(16))) float h_u_l_To_h_ci_l[2];
  __attribute__((aligned(16))) float fv5[2];
  float b_h_u_l_To_h_ci_l[6];
  float b_h_u_r_To_h_ci_r[6];
  float fp_r_norm_inv;
  float fp_r_32_inv;
  float h_ci_r_To_fp_r[9];
  float h_ci_l_To_r_wc[9];
  float h_ci_l_To_R_cw[9];
  float fp_l_To_r_wc[9];
  float fp_l_To_R_cw[9];
  float h_ci_l_To_anchorPos[9];
  float h_ci_l_To_anchorRot[9];
  float fp_l_To_anchorPos[9];
  float fp_l_To_anchorRot[9];
  float b_R_cw[9];
  float c_R_cw[9];
  float f1;
  float fv6[9];
  float fv7[9];
  float d_R_cw[9];
  int b_tmp_data[4];
  float c_h_u_r_To_h_ci_r[6];
  float d_h_u_r_To_h_ci_r[6];
  float c_h_u_l_To_h_ci_l[6];
  float e_h_u_r_To_h_ci_r[6];
  float d_h_u_l_To_h_ci_l[12];

  // 'getH_R_res:26' fx_l = stereoParams.CameraParameters1.FocalLength(1);
  fx_l = c_stereoParams_CameraParameters[0];

  // 'getH_R_res:27' fy_l = stereoParams.CameraParameters1.FocalLength(2);
  fy_l = c_stereoParams_CameraParameters[1];

  // 'getH_R_res:29' fx_r = stereoParams.CameraParameters2.FocalLength(1);
  fx_r = e_stereoParams_CameraParameters[0];

  // 'getH_R_res:30' fy_r = stereoParams.CameraParameters2.FocalLength(2);
  fy_r = e_stereoParams_CameraParameters[1];

  // 'getH_R_res:32' numErrorStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'getH_R_res:34' numMeas = nnz(status);
  nm1d2 = 0;
  for (cdiff = 0; cdiff < 48; cdiff++) {
    if (b_status[cdiff]) {
      nm1d2++;
    }
  }

  // 'getH_R_res:35' if VIOParameters.full_stereo
  if (VIOParameters_full_stereo) {
    // 'getH_R_res:36' residualDim = 4;
    residualDim = 4;
  } else {
    // 'getH_R_res:37' else
    // 'getH_R_res:38' residualDim = 2;
    residualDim = 2;
  }

  // 'getH_R_res:41' R_cw = RotFromQuatJ(xt.robot_state.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_cw[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[3] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1] +
                    xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[6] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2] -
                    xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[1] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1] -
                    xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[7] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2] +
                    xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[2] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2] +
                    xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[5] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2] -
                    xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
              xt_robot_state_att[1] * xt_robot_state_att[1]) +
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];

  // 'getH_R_res:42' r_wc = xt.robot_state.pos;
  // 'getH_R_res:44' H = zeros(numMeas*residualDim, numStates + numAnchors*numErrorStatesPerAnchor); 
  c = (float)nm1d2 * (float)residualDim;
  i13 = H->size[0] * H->size[1];
  H->size[0] = (int)c;
  H->size[1] = 102;
  emxEnsureCapacity((emxArray__common *)H, i13, (int)sizeof(float));
  cdiff = (int)c * 102;
  for (i13 = 0; i13 < cdiff; i13++) {
    H->data[i13] = 0.0F;
  }

  // 'getH_R_res:45' r = zeros(numMeas*residualDim, 1);
  c = (float)nm1d2 * (float)residualDim;
  i13 = r->size[0];
  r->size[0] = (int)c;
  emxEnsureCapacity((emxArray__common *)r, i13, (int)sizeof(float));
  cdiff = (int)c;
  for (i13 = 0; i13 < cdiff; i13++) {
    r->data[i13] = 0.0F;
  }

  // 'getH_R_res:47' ind = int32(zeros(numMeas, 2));
  i13 = ind->size[0] * ind->size[1];
  ind->size[0] = (int)(float)nm1d2;
  ind->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)ind, i13, (int)sizeof(int));
  cdiff = (int)(float)nm1d2 << 1;
  for (i13 = 0; i13 < cdiff; i13++) {
    ind->data[i13] = 0;
  }

  // 'getH_R_res:49' res_idx = 1;
  res_idx = 1.0F;

  // 'getH_R_res:51' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'getH_R_res:52' anchorPos = xt.anchor_states(anchorIdx).pos;
    // 'getH_R_res:53' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    anchorRot[0] = ((xt_anchor_states[anchorIdx].att[0] *
                     xt_anchor_states[anchorIdx].att[0] -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) +
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) +
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];

    // 'getH_R_res:55' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getH_R_res:57' idx = xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx; 
      // 'getH_R_res:59' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status && status(idx) 
      if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) &&
          b_status[xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1]) {
        // 'getH_R_res:60' z_curr_l = z_all_l((idx-1)*2 + int32(1:2));
        cdiff = xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx;
        nm1d2 = cdiff - 1;
        if ((cdiff < 0) && (nm1d2 >= 0)) {
          nm1d2 = MIN_int32_T;
        }

        if (nm1d2 > 1073741823) {
          b_c = MAX_int32_T;
        } else if (nm1d2 <= -1073741824) {
          b_c = MIN_int32_T;
        } else {
          b_c = nm1d2 << 1;
        }

        // 'getH_R_res:61' z_curr_r = z_all_r((idx-1)*2 + int32(1:2));
        cdiff = xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx;
        nm1d2 = cdiff - 1;
        if ((cdiff < 0) && (nm1d2 >= 0)) {
          nm1d2 = MIN_int32_T;
        }

        if (nm1d2 > 1073741823) {
          apnd = MAX_int32_T;
        } else if (nm1d2 <= -1073741824) {
          apnd = MIN_int32_T;
        } else {
          apnd = nm1d2 << 1;
        }

        for (i13 = 0; i13 < 2; i13++) {
          cdiff = 1 + i13;
          nm1d2 = b_c + cdiff;
          if ((b_c > 0) && (nm1d2 <= 0)) {
            nm1d2 = MAX_int32_T;
          }

          z_curr_l[i13] = z_all_l[nm1d2 - 1];
          cdiff = 1 + i13;
          nm1d2 = apnd + cdiff;
          if ((apnd > 0) && (nm1d2 <= 0)) {
            nm1d2 = MAX_int32_T;
          }

          z_curr_r[i13] = z_all_r[nm1d2 - 1];
        }

        // 'getH_R_res:63' h_ci_l = xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point; 
        // 'getH_R_res:64' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getH_R_res:65' fp_l = h_ci_l/rho;
        for (i13 = 0; i13 < 3; i13++) {
          fp_l[i13] = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i13] / xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
        }

        // 'getH_R_res:66' fp_r = stereoParams.R_rl*fp_l - stereoParams.r_lr;
        for (i13 = 0; i13 < 3; i13++) {
          fv4[i13] = 0.0F;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            fv4[i13] += stereoParams_R_rl[i13 + 3 * cdiff] * fp_l[cdiff];
          }
        }

        mw_neon_mm_sub_f32x4(fv4, 3, 1, *(float (*)[3])&stereoParams_r_lr[0],
                             &fp_r[0]);

        // 'getH_R_res:67' h_ci_r = fp_r / norm(fp_r);
        c = norm(fp_r);
        for (cdiff = 0; cdiff < 3; cdiff++) {
          h_ci_r[cdiff] = fp_r[cdiff] / c;
        }

        //  necessary?
        // 'getH_R_res:69' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getH_R_res:71' if VIOParameters.full_stereo
        if (VIOParameters_full_stereo) {
          // 'getH_R_res:73' [ h_u_l, h_u_r ] = predictMeasurementStereo( fp_l, stereoParams ); 
          predictMeasurementStereo(fp_l, c_stereoParams_CameraParameters,
            d_stereoParams_CameraParameters, e_stereoParams_CameraParameters,
            f_stereoParams_CameraParameters, stereoParams_r_lr,
            stereoParams_R_rl, h_u_l, h_u_r);

          // 'getH_R_res:75' r((res_idx-1)*residualDim + (1:residualDim)) = [z_curr_l - h_u_l; z_curr_r - h_u_r]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            tmp_data[i13] = (int)(c + vec_data[i13]);
          }

          mw_neon_mm_sub_f32x4(z_curr_r, 2, 1, h_u_r, &h_u_r_To_h_ci_r[0]);
          mw_neon_mm_sub_f32x4(z_curr_l, 2, 1, h_u_l, &h_u_l_To_h_ci_l[0]);
          for (i13 = 0; i13 < 2; i13++) {
            vec_data[i13] = h_u_l_To_h_ci_l[i13];
            vec_data[i13 + 2] = h_u_r_To_h_ci_r[i13];
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            r->data[tmp_data[i13] - 1] = vec_data[i13];
          }
        } else {
          // 'getH_R_res:76' else
          // 'getH_R_res:77' h_u_l = predictMeasurementMono( fp_l, stereoParams.CameraParameters1 ); 
          // predictMeasurementLeft Predict the measurement of a feature given in the left 
          // camera frame
          //    Get the normalized pixel coordinates where a feature given in the left camera 
          //    frame
          // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
          // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
          // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1);
          // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2);
          // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
          for (cdiff = 0; cdiff < 2; cdiff++) {
            z_curr_r[cdiff] = fp_l[cdiff] / fp_l[2];
          }

          //  normalized feature in camera frame
          // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
          // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
          // 'getH_R_res:79' r((res_idx-1)*residualDim + (1:residualDim)) = z_curr_l - h_u_l; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            tmp_data[i13] = (int)(c + vec_data[i13]);
          }

          fv5[0] = z_curr_r[0] * c_stereoParams_CameraParameters[0] +
            d_stereoParams_CameraParameters[0];
          fv5[1] = z_curr_r[1] * c_stereoParams_CameraParameters[1] +
            d_stereoParams_CameraParameters[1];
          mw_neon_mm_sub_f32x4(z_curr_l, 2, 1, fv5, &h_u_r_To_h_ci_r[0]);
          for (i13 = 0; i13 < ndbl; i13++) {
            r->data[tmp_data[i13] - 1] = h_u_r_To_h_ci_r[i13];
          }
        }

        // 'getH_R_res:82' h_u_l_To_h_ci_l = [fx_l/h_ci_l(3),              0, -fx_l*(h_ci_l(1))/h_ci_l(3)^2; 
        // 'getH_R_res:83'                                             0, fy_l/h_ci_l(3), -fy_l*(h_ci_l(2))/h_ci_l(3)^2]; 
        b_h_u_l_To_h_ci_l[0] = fx_l / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        b_h_u_l_To_h_ci_l[2] = 0.0F;
        b_h_u_l_To_h_ci_l[4] = -fx_l * xt_anchor_states[anchorIdx].
          feature_states[featureIdx].scaled_map_point[0] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        b_h_u_l_To_h_ci_l[1] = 0.0F;
        b_h_u_l_To_h_ci_l[3] = fy_l / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        b_h_u_l_To_h_ci_l[5] = -fy_l * xt_anchor_states[anchorIdx].
          feature_states[featureIdx].scaled_map_point[1] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);

        // 'getH_R_res:86' h_u_r_To_h_ci_r = [fx_r/h_ci_r(3), 0,              -fx_r*(h_ci_r(1))/h_ci_r(3)^2; 
        // 'getH_R_res:87'                                             0, fy_r/h_ci_r(3), -fy_r*(h_ci_r(2))/h_ci_r(3)^2]; 
        b_h_u_r_To_h_ci_r[0] = fx_r / h_ci_r[2];
        b_h_u_r_To_h_ci_r[2] = 0.0F;
        b_h_u_r_To_h_ci_r[4] = -fx_r * h_ci_r[0] / (h_ci_r[2] * h_ci_r[2]);
        b_h_u_r_To_h_ci_r[1] = 0.0F;
        b_h_u_r_To_h_ci_r[3] = fy_r / h_ci_r[2];
        b_h_u_r_To_h_ci_r[5] = -fy_r * h_ci_r[1] / (h_ci_r[2] * h_ci_r[2]);

        // 'getH_R_res:89' fp_r_norm_inv = 1/norm(fp_r);
        fp_r_norm_inv = 1.0F / norm(fp_r);

        // 'getH_R_res:90' fp_r_32_inv = fp_r_norm_inv^3;
        fp_r_32_inv = powf(fp_r_norm_inv, 3.0F);

        // 'getH_R_res:92' h_ci_r_To_fp_r = [fp_r_norm_inv - fp_r(1)*fp_r(1)*fp_r_32_inv,               - fp_r(1)*fp_r(2)*fp_r_32_inv,               - fp_r(1)*fp_r(3)*fp_r_32_inv; 
        // 'getH_R_res:93'                                             - fp_r(2)*fp_r(1)*fp_r_32_inv, fp_r_norm_inv - fp_r(2)*fp_r(2)*fp_r_32_inv,               - fp_r(2)*fp_r(3)*fp_r_32_inv; 
        // 'getH_R_res:94'                                             - fp_r(3)*fp_r(1)*fp_r_32_inv,               - fp_r(3)*fp_r(2)*fp_r_32_inv, fp_r_norm_inv - fp_r(3)*fp_r(3)*fp_r_32_inv]; 
        h_ci_r_To_fp_r[0] = fp_r_norm_inv - fp_r[0] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[3] = -fp_r[0] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[6] = -fp_r[0] * fp_r[2] * fp_r_32_inv;
        h_ci_r_To_fp_r[1] = -fp_r[1] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[4] = fp_r_norm_inv - fp_r[1] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[7] = -fp_r[1] * fp_r[2] * fp_r_32_inv;
        h_ci_r_To_fp_r[2] = -fp_r[2] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[5] = -fp_r[2] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[8] = fp_r_norm_inv - fp_r[2] * fp_r[2] * fp_r_32_inv;

        // necessary?
        // 'getH_R_res:96' fp_r_To_fp_l = stereoParams.R_rl;
        //              h_ci_l_To_r_wc = -rho*R_cw;
        //              h_ci_l_To_R_cw = skew(h_ci_l);
        // 'getH_R_res:101' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F)) {
          //  delayed initialization or feature behind anchor
          //                  H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:103' h_ci_l_To_r_wc = zeros(3);
          // 'getH_R_res:104' h_ci_l_To_R_cw = zeros(3);
          // 'getH_R_res:106' fp_l_To_r_wc = zeros(3);
          // 'getH_R_res:107' fp_l_To_R_cw = zeros(3);
          for (i13 = 0; i13 < 9; i13++) {
            h_ci_l_To_r_wc[i13] = 0.0F;
            h_ci_l_To_R_cw[i13] = 0.0F;
            fp_l_To_r_wc[i13] = 0.0F;
            fp_l_To_R_cw[i13] = 0.0F;
          }
        } else {
          // 'getH_R_res:108' else
          //                  H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:110' h_ci_l_To_r_wc = -rho*R_cw;
          // 'getH_R_res:111' h_ci_l_To_R_cw = skew(h_ci_l);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          h_ci_l_To_R_cw[0] = 0.0F;
          h_ci_l_To_R_cw[3] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
          h_ci_l_To_R_cw[6] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[1];
          h_ci_l_To_R_cw[1] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
          h_ci_l_To_R_cw[4] = 0.0F;
          h_ci_l_To_R_cw[7] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[0];
          h_ci_l_To_R_cw[2] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[1];
          h_ci_l_To_R_cw[5] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[0];
          h_ci_l_To_R_cw[8] = 0.0F;

          // 'getH_R_res:113' fp_l_To_r_wc = -R_cw;
          for (i13 = 0; i13 < 9; i13++) {
            h_ci_l_To_r_wc[i13] = -xt_anchor_states[anchorIdx]
              .feature_states[featureIdx].inverse_depth * R_cw[i13];
            fp_l_To_r_wc[i13] = -R_cw[i13];
          }

          // 'getH_R_res:114' fp_l_To_R_cw = skew(fp_l);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          fp_l_To_R_cw[0] = 0.0F;
          fp_l_To_R_cw[3] = -fp_l[2];
          fp_l_To_R_cw[6] = fp_l[1];
          fp_l_To_R_cw[1] = fp_l[2];
          fp_l_To_R_cw[4] = 0.0F;
          fp_l_To_R_cw[7] = -fp_l[0];
          fp_l_To_R_cw[2] = -fp_l[1];
          fp_l_To_R_cw[5] = fp_l[0];
          fp_l_To_R_cw[8] = 0.0F;
        }

        //             %% anchor state derivatives
        //              h_ci_l_To_anchorPos = rho*R_cw;
        //              h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //              h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        // 'getH_R_res:123' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F)) {
          //  delayed initialization or feature behind anchor
          //                  H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:125' h_ci_l_To_anchorPos = zeros(3);
          // 'getH_R_res:126' h_ci_l_To_anchorRot = zeros(3);
          for (i13 = 0; i13 < 9; i13++) {
            h_ci_l_To_anchorPos[i13] = 0.0F;
            h_ci_l_To_anchorRot[i13] = 0.0F;
          }

          // 'getH_R_res:127' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          mw_neon_mm_sub_f32x4(*(float (*)[3])&xt_anchor_states[anchorIdx].pos[0],
                               3, 1, *(float (*)[3])&xt_robot_state_pos[0],
                               &fv4[0]);
          for (i13 = 0; i13 < 3; i13++) {
            fp_r[i13] = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_r[i13] += R_cw[i13 + 3 * cdiff] * fv4[cdiff];
            }
          }

          // 'getH_R_res:129' fp_l_To_anchorPos = zeros(3);
          // 'getH_R_res:130' fp_l_To_anchorRot = zeros(3);
          for (i13 = 0; i13 < 9; i13++) {
            fp_l_To_anchorPos[i13] = 0.0F;
            fp_l_To_anchorRot[i13] = 0.0F;
          }

          // 'getH_R_res:131' fp_l_To_rho       = -R_cw * anchorRot'*m/rho^2;
          c = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i13] = -R_cw[cdiff + 3 * i13];
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            f1 = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_R_cw[i13 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_R_cw[i13 + 3 * cdiff] += b_R_cw[i13 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }

              f1 += c_R_cw[i13 + 3 * cdiff] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
            }

            fp_l[i13] = f1 / c;
          }
        } else {
          // 'getH_R_res:132' else
          //                  H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:134' h_ci_l_To_anchorPos = rho*R_cw;
          for (i13 = 0; i13 < 9; i13++) {
            h_ci_l_To_anchorPos[i13] = xt_anchor_states[anchorIdx].
              feature_states[featureIdx].inverse_depth * R_cw[i13];
          }

          // 'getH_R_res:135' h_ci_l_To_anchorRot = -R_cw * anchorRot'*skew(m);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i13] = -R_cw[cdiff + 3 * i13];
            }
          }

          fv6[0] = 0.0F;
          fv6[3] = -xt_anchor_states[anchorIdx].feature_states[featureIdx].m[2];
          fv6[6] = xt_anchor_states[anchorIdx].feature_states[featureIdx].m[1];
          fv6[1] = xt_anchor_states[anchorIdx].feature_states[featureIdx].m[2];
          fv6[4] = 0.0F;
          fv6[7] = -xt_anchor_states[anchorIdx].feature_states[featureIdx].m[0];
          fv6[2] = -xt_anchor_states[anchorIdx].feature_states[featureIdx].m[1];
          fv6[5] = xt_anchor_states[anchorIdx].feature_states[featureIdx].m[0];
          fv6[8] = 0.0F;
          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_R_cw[i13 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_R_cw[i13 + 3 * cdiff] += b_R_cw[i13 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              h_ci_l_To_anchorRot[i13 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                h_ci_l_To_anchorRot[i13 + 3 * cdiff] += c_R_cw[i13 + 3 * nm1d2] *
                  fv6[nm1d2 + 3 * cdiff];
              }
            }
          }

          // 'getH_R_res:136' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          mw_neon_mm_sub_f32x4(*(float (*)[3])&xt_anchor_states[anchorIdx].pos[0],
                               3, 1, *(float (*)[3])&xt_robot_state_pos[0],
                               &fv4[0]);
          for (i13 = 0; i13 < 3; i13++) {
            fp_r[i13] = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_r[i13] += R_cw[i13 + 3 * cdiff] * fv4[cdiff];
            }
          }

          // 'getH_R_res:138' fp_l_To_anchorPos = R_cw;
          for (i13 = 0; i13 < 9; i13++) {
            fp_l_To_anchorPos[i13] = R_cw[i13];
          }

          // 'getH_R_res:139' fp_l_To_anchorRot = -R_cw * anchorRot'*skew(m/rho); 
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i13 = 0; i13 < 3; i13++) {
            fp_l[i13] = xt_anchor_states[anchorIdx].feature_states[featureIdx]
              .m[i13] / xt_anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i13] = -R_cw[cdiff + 3 * i13];
            }
          }

          fv7[0] = 0.0F;
          fv7[3] = -fp_l[2];
          fv7[6] = fp_l[1];
          fv7[1] = fp_l[2];
          fv7[4] = 0.0F;
          fv7[7] = -fp_l[0];
          fv7[2] = -fp_l[1];
          fv7[5] = fp_l[0];
          fv7[8] = 0.0F;

          // 'getH_R_res:140' fp_l_To_rho       = -R_cw * anchorRot'*m/rho^2;
          c = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_R_cw[i13 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_R_cw[i13 + 3 * cdiff] += b_R_cw[i13 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_l_To_anchorRot[i13 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                fp_l_To_anchorRot[i13 + 3 * cdiff] += c_R_cw[i13 + 3 * nm1d2] *
                  fv7[nm1d2 + 3 * cdiff];
              }

              d_R_cw[cdiff + 3 * i13] = -R_cw[cdiff + 3 * i13];
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            f1 = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[i13 + 3 * cdiff] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_R_cw[i13 + 3 * cdiff] += d_R_cw[i13 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }

              f1 += b_R_cw[i13 + 3 * cdiff] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
            }

            fp_l[i13] = f1 / c;
          }
        }

        //              tmp = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        // 'getH_R_res:144' if VIOParameters.full_stereo
        if (VIOParameters_full_stereo) {
          // 'getH_R_res:145' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = [h_u_l_To_h_ci_l * h_ci_l_To_r_wc; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_r_wc]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i13 +
                  (nm1d2 << 1)] * h_ci_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              e_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                e_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += d_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * fp_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              d_h_u_l_To_h_ci_l[cdiff + (i13 << 2)] = c_h_u_l_To_h_ci_l[cdiff +
                (i13 << 1)];
              d_h_u_l_To_h_ci_l[(cdiff + (i13 << 2)) + 2] =
                e_h_u_r_To_h_ci_r[cdiff + (i13 << 1)];
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * i13] =
                d_h_u_l_To_h_ci_l[cdiff + ndbl * i13];
            }
          }

          // 'getH_R_res:146' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = [h_u_l_To_h_ci_l * h_ci_l_To_R_cw; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_R_cw]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i13 +
                  (nm1d2 << 1)] * h_ci_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              e_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                e_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += d_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * fp_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              d_h_u_l_To_h_ci_l[cdiff + (i13 << 2)] = c_h_u_l_To_h_ci_l[cdiff +
                (i13 << 1)];
              d_h_u_l_To_h_ci_l[(cdiff + (i13 << 2)) + 2] =
                e_h_u_r_To_h_ci_r[cdiff + (i13 << 1)];
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (3 + i13)] =
                d_h_u_l_To_h_ci_l[cdiff + ndbl * i13];
            }
          }

          // 'getH_R_res:147' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = [h_u_l_To_h_ci_l*h_ci_l_To_anchorPos; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_anchorPos]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          b_c = anchorIdx * 14 + 18;
          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i13 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              e_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                e_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += d_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * fp_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              d_h_u_l_To_h_ci_l[cdiff + (i13 << 2)] = c_h_u_l_To_h_ci_l[cdiff +
                (i13 << 1)];
              d_h_u_l_To_h_ci_l[(cdiff + (i13 << 2)) + 2] =
                e_h_u_r_To_h_ci_r[cdiff + (i13 << 1)];
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (i13 + b_c)] =
                d_h_u_l_To_h_ci_l[cdiff + ndbl * i13];
            }
          }

          // 'getH_R_res:148' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = [h_u_l_To_h_ci_l*h_ci_l_To_anchorRot; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_anchorRot]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          b_c = anchorIdx * 14 + 18;
          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i13 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              e_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                e_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += d_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * fp_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              d_h_u_l_To_h_ci_l[cdiff + (i13 << 2)] = c_h_u_l_To_h_ci_l[cdiff +
                (i13 << 1)];
              d_h_u_l_To_h_ci_l[(cdiff + (i13 << 2)) + 2] =
                e_h_u_r_To_h_ci_r[cdiff + (i13 << 1)];
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * ((i13 + b_c) + 3)] =
                d_h_u_l_To_h_ci_l[cdiff + ndbl * i13];
            }
          }

          // 'getH_R_res:149' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = [h_u_l_To_h_ci_l*h_ci_l_To_rho; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_rho]; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            h_u_l_To_h_ci_l[i13] = 0.0F;
            h_u_r_To_h_ci_r[i13] = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i13 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
              }

              h_u_l_To_h_ci_l[i13] += b_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] *
                fp_r[cdiff];
              h_u_r_To_h_ci_r[i13] += d_h_u_r_To_h_ci_r[i13 + (cdiff << 1)] *
                fp_l[cdiff];
            }

            vec_data[i13] = h_u_l_To_h_ci_l[i13];
            vec_data[i13 + 2] = h_u_r_To_h_ci_r[i13];
          }

          nm1d2 = anchorIdx * 14 + featureIdx;
          for (i13 = 0; i13 < ndbl; i13++) {
            H->data[b_tmp_data[i13] + H->size[0] * (nm1d2 + 24)] = vec_data[i13];
          }
        } else {
          // 'getH_R_res:150' else
          // 'getH_R_res:151' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = h_u_l_To_h_ci_l * h_ci_l_To_r_wc; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i13 +
                  (nm1d2 << 1)] * h_ci_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * i13] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i13];
            }
          }

          // 'getH_R_res:152' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = h_u_l_To_h_ci_l * h_ci_l_To_R_cw; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i13 +
                  (nm1d2 << 1)] * h_ci_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (3 + i13)] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i13];
            }
          }

          // 'getH_R_res:153' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = h_u_l_To_h_ci_l*h_ci_l_To_anchorPos; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          b_c = anchorIdx * 14 + 18;
          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i13 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (i13 + b_c)] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i13];
            }
          }

          // 'getH_R_res:154' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = h_u_l_To_h_ci_l*h_ci_l_To_anchorRot; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          b_c = anchorIdx * 14 + 18;
          for (i13 = 0; i13 < 2; i13++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] = 0.0F;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] += b_h_u_l_To_h_ci_l[i13 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i13 = 0; i13 < 3; i13++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * ((i13 + b_c) + 3)] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i13];
            }
          }

          // 'getH_R_res:155' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = h_u_l_To_h_ci_l*h_ci_l_To_rho; 
          c = (res_idx - 1.0F) * (float)residualDim;
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 2.38418579E-7F * (float)residualDim) {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          vec_data[0] = 1.0F;
          if (ndbl > 1) {
            vec_data[ndbl - 1] = (float)apnd;
            i13 = ndbl - 1;
            nm1d2 = i13 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              vec_data[1] = 2.0F;
              vec_data[ndbl - 2] = (float)apnd - 1.0F;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              vec_data[nm1d2] = (1.0F + (float)apnd) / 2.0F;
            } else {
              vec_data[nm1d2] = 1.0F + (float)nm1d2;
              vec_data[nm1d2 + 1] = (float)(apnd - nm1d2);
            }
          }

          for (i13 = 0; i13 < ndbl; i13++) {
            b_tmp_data[i13] = (int)(c + vec_data[i13]) - 1;
          }

          for (i13 = 0; i13 < 2; i13++) {
            h_u_l_To_h_ci_l[i13] = 0.0F;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              h_u_l_To_h_ci_l[i13] += b_h_u_l_To_h_ci_l[i13 + (cdiff << 1)] *
                fp_r[cdiff];
            }
          }

          nm1d2 = anchorIdx * 14 + featureIdx;
          for (i13 = 0; i13 < ndbl; i13++) {
            H->data[b_tmp_data[i13] + H->size[0] * (nm1d2 + 24)] =
              h_u_l_To_h_ci_l[i13];
          }
        }

        //              if coder.target('MATLAB')
        //
        //                  h_ci_l_To_r_wc = -rho*R_cw;
        //                  h_ci_l_To_R_cw = skew(h_ci_l);
        //
        //                  if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 % delayed initialization or feature behind anchor 
        //                      H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
        //                  else
        //                      H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
        //                  end
        //
        //                  %% anchor state derivatives
        //
        //                  h_ci_l_To_anchorPos = rho*R_cw;
        //                  h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //                  h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        //
        //                  if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 % delayed initialization or feature behind anchor 
        //                      H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
        //                  else
        //                      H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
        //                  end
        //
        //                  H_t = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        //
        //                  if any(any(abs(H((res_idx-1)*residualDim + (1:residualDim), :) - H_t) > 1e-8)) 
        //                      figure; imagesc(H_t - H((res_idx-1)*residualDim + (1:residualDim), :)) 
        //                      error('H inconsistency')
        //                  end
        //              end
        // 'getH_R_res:189' ind(res_idx, 1) = anchorIdx;
        ind->data[(int)res_idx - 1] = anchorIdx + 1;

        // 'getH_R_res:190' ind(res_idx, 2) = featureIdx;
        ind->data[((int)res_idx + ind->size[0]) - 1] = featureIdx + 1;

        // 'getH_R_res:192' res_idx = res_idx + 1;
        res_idx++;
      }
    }
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
// Arguments    : const float xt_origin_pos[3]
//                const float xt_origin_att[4]
//                const f_struct_T xt_anchor_states[6]
//                float b_map[144]
// Return Type  : void
//
static void getMap(const float xt_origin_pos[3], const float xt_origin_att[4],
                   const f_struct_T xt_anchor_states[6], float b_map[144])
{
  float R_ow[9];
  float anchorRot[9];
  int anchorIdx;
  __attribute__((aligned(16))) float fv0[3];
  int i0;
  int qY;
  __attribute__((aligned(16))) float anchorPos[3];
  float b_xt_anchor_states[9];
  int q0;
  int featureIdx;
  int c;
  float f0;
  __attribute__((aligned(16))) float fv1[3];

  // 'getMap:23' r_ow = xt.origin.pos;
  // 'getMap:24' R_ow = RotFromQuatJ(xt.origin.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[6] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[1] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[2] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[5] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];

  // 'getMap:26' map = zeros(numTrackFeatures*3, 1);
  memset(&b_map[0], 0, 144U * sizeof(float));

  // 'getMap:28' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'getMap:29' anchorPos = r_ow + R_ow' * xt.anchor_states(anchorIdx).pos;
    for (i0 = 0; i0 < 3; i0++) {
      fv0[i0] = 0.0F;
      for (qY = 0; qY < 3; qY++) {
        fv0[i0] += R_ow[qY + 3 * i0] * xt_anchor_states[anchorIdx].pos[qY];
      }
    }

    mw_neon_mm_add_f32x4(*(float (*)[3])&xt_origin_pos[0], 3, 1, fv0,
                         &anchorPos[0]);

    // 'getMap:30' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att) * R_ow; 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    b_xt_anchor_states[0] = ((xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[3] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[6] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[1] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[7] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[2] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[5] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    for (i0 = 0; i0 < 3; i0++) {
      for (qY = 0; qY < 3; qY++) {
        anchorRot[i0 + 3 * qY] = 0.0F;
        for (q0 = 0; q0 < 3; q0++) {
          anchorRot[i0 + 3 * qY] += b_xt_anchor_states[i0 + 3 * q0] * R_ow[q0 +
            3 * qY];
        }
      }
    }

    // 'getMap:32' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getMap:33' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if (xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) {
        // 'getMap:34' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getMap:36' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getMap:38' fp = anchorPos + anchorRot'*m/rho;
        // 'getMap:40' map((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*3 + int32(1:3))= fp; 
        q0 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx;
        qY = q0 - 1;
        if ((q0 < 0) && (qY >= 0)) {
          qY = MIN_int32_T;
        }

        c = mul_s32_s32_s32_sat(qY, 3);
        for (i0 = 0; i0 < 3; i0++) {
          f0 = 0.0F;
          for (qY = 0; qY < 3; qY++) {
            f0 += anchorRot[qY + 3 * i0] * xt_anchor_states[anchorIdx].
              feature_states[featureIdx].m[qY];
          }

          fv0[i0] = f0 / xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth;
        }

        mw_neon_mm_add_f32x4(anchorPos, 3, 1, fv0, &fv1[0]);
        for (i0 = 0; i0 < 3; i0++) {
          q0 = 1 + i0;
          qY = c + q0;
          if ((c > 0) && (qY <= 0)) {
            qY = MAX_int32_T;
          }

          b_map[qY - 1] = fv1[i0];
        }
      }
    }
  }
}

//
// getNumValidFeatures Get the number of valid features of an anchor
// Arguments    : const e_struct_T anchor_state_feature_states[8]
// Return Type  : float
//
static float getNumValidFeatures(const e_struct_T anchor_state_feature_states[8])
{
  float n;
  int featureIdx;
  boolean_T x[2];
  int k;
  boolean_T y;
  boolean_T exitg1;

  // 'getNumValidFeatures:4' n = getNumFeaturesOfType(anchor_state, [1 2]);
  // getNumFeaturesOfType Get the number of features of type type of an anchor
  //  type can be a scalar or a row vector of types
  // 'getNumFeaturesOfType:5' n = 0;
  n = 0.0F;

  // 'getNumFeaturesOfType:6' for featureIdx = 1:numPointsPerAnchor
  for (featureIdx = 0; featureIdx < 8; featureIdx++) {
    // 'getNumFeaturesOfType:7' if any(anchor_state.feature_states(featureIdx).status == type) 
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
      // 'getNumFeaturesOfType:8' n = n + 1;
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
  float R_cw[9];
  int anchorIdx;
  float anchorRot[9];
  int featureIdx;
  __attribute__((aligned(16))) float fv39[3];
  __attribute__((aligned(16))) float fv40[3];
  int i37;
  int i38;
  __attribute__((aligned(16))) float fv41[3];

  // 'getScaledMap:22' R_cw = RotFromQuatJ(xt.robot_state.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_cw[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  R_cw[3] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                    b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
  R_cw[6] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
  R_cw[1] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                    b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
  R_cw[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  R_cw[7] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                    b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
  R_cw[2] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
  R_cw[5] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                    b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
  R_cw[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];

  // 'getScaledMap:24' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'getScaledMap:25' anchorPos = xt.anchor_states(anchorIdx).pos;
    // 'getScaledMap:26' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    anchorRot[0] = ((b_xt->anchor_states[anchorIdx].att[0] * b_xt->
                     anchor_states[anchorIdx].att[0] - b_xt->
                     anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) - b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
      b_xt->anchor_states[anchorIdx].att[1] + b_xt->anchor_states[anchorIdx]
      .att[2] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
      b_xt->anchor_states[anchorIdx].att[2] - b_xt->anchor_states[anchorIdx]
      .att[1] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
      b_xt->anchor_states[anchorIdx].att[1] - b_xt->anchor_states[anchorIdx]
      .att[2] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                       b_xt->anchor_states[anchorIdx].att[0]) +
                     b_xt->anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) - b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
      b_xt->anchor_states[anchorIdx].att[2] + b_xt->anchor_states[anchorIdx]
      .att[0] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
      b_xt->anchor_states[anchorIdx].att[2] + b_xt->anchor_states[anchorIdx]
      .att[1] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
      b_xt->anchor_states[anchorIdx].att[2] - b_xt->anchor_states[anchorIdx]
      .att[0] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                       b_xt->anchor_states[anchorIdx].att[0]) -
                     b_xt->anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) + b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];

    // 'getScaledMap:28' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getScaledMap:29' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
      {
        // 'getScaledMap:30' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getScaledMap:32' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getScaledMap:34' fp_scaled = R_cw*(rho*anchorPos + anchorRot'*m - xt.robot_state.pos*rho); 
        // 'getScaledMap:36' xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point = fp_scaled; 
        for (i37 = 0; i37 < 3; i37++) {
          fv39[i37] = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * b_xt->anchor_states[anchorIdx].pos[i37];
          fv40[i37] = 0.0F;
          for (i38 = 0; i38 < 3; i38++) {
            fv40[i37] += anchorRot[i38 + 3 * i37] * b_xt->
              anchor_states[anchorIdx].feature_states[featureIdx].m[i38];
          }
        }

        mw_neon_mm_add_f32x4(fv39, 3, 1, fv40, &fv41[0]);
        for (i37 = 0; i37 < 3; i37++) {
          fv39[i37] = b_xt->robot_state.pos[i37] * b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
        }

        mw_neon_mm_sub_f32x4(fv41, 3, 1, fv39, &fv40[0]);
        for (i37 = 0; i37 < 3; i37++) {
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i37] = 0.0F;
          for (i38 = 0; i38 < 3; i38++) {
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              scaled_map_point[i37] += R_cw[i37 + 3 * i38] * fv40[i38];
          }
        }
      }
    }
  }
}

//
// getTotalNumActiveFeatures Get the number of active features of all anchors
// Arguments    : const f_struct_T xt_anchor_states[6]
// Return Type  : float
//
static float getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[6])
{
  float n;
  int anchorIdx;
  float b_n;
  int featureIdx;

  // 'getTotalNumActiveFeatures:4' n = 0;
  n = 0.0F;

  // 'getTotalNumActiveFeatures:5' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'getTotalNumActiveFeatures:6' n = n + getNumActiveFeatures(xt.anchor_states(anchorIdx)); 
    // getNumActiveFeatures Get the number of active features of an anchor
    // 'getNumActiveFeatures:4' n = getNumFeaturesOfType(anchor_state, 1);
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    // 'getNumFeaturesOfType:5' n = 0;
    b_n = 0.0F;

    // 'getNumFeaturesOfType:6' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getNumFeaturesOfType:7' if any(anchor_state.feature_states(featureIdx).status == type) 
      if (!!(xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 1))
      {
        // 'getNumFeaturesOfType:8' n = n + 1;
        b_n++;
      }
    }

    n += b_n;
  }

  return n;
}

//
// getTotalNumDelayedFeatures Get the number of delayed features of all anchors
// Arguments    : const f_struct_T xt_anchor_states[6]
// Return Type  : float
//
static float getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[6])
{
  float n;
  int anchorIdx;
  float b_n;
  int featureIdx;

  // 'getTotalNumDelayedFeatures:4' n = 0;
  n = 0.0F;

  // 'getTotalNumDelayedFeatures:5' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
    // 'getTotalNumDelayedFeatures:6' n = n + getNumFeaturesOfType(xt.anchor_states(anchorIdx), 2); 
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    // 'getNumFeaturesOfType:5' n = 0;
    b_n = 0.0F;

    // 'getNumFeaturesOfType:6' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getNumFeaturesOfType:7' if any(anchor_state.feature_states(featureIdx).status == type) 
      if (!!(xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2))
      {
        // 'getNumFeaturesOfType:8' n = n + 1;
        b_n++;
      }
    }

    n += b_n;
  }

  return n;
}

//
// getWorldState Get the state of the robot in the world frame
// Arguments    : const float xt_robot_state_IMU_pos[3]
//                const float xt_robot_state_IMU_att[4]
//                const float xt_robot_state_IMU_gyro_bias[3]
//                const float xt_robot_state_IMU_acc_bias[3]
//                const float xt_robot_state_pos[3]
//                const float xt_robot_state_att[4]
//                const float xt_robot_state_vel[3]
//                const float xt_origin_pos[3]
//                const float xt_origin_att[4]
//                float world_state_pos[3]
//                float world_state_att[4]
//                float world_state_vel[3]
//                float world_state_IMU_gyro_bias[3]
//                float world_state_IMU_acc_bias[3]
//                float world_state_IMU_pos[3]
//                float world_state_IMU_att[4]
// Return Type  : void
//
static void getWorldState(const float xt_robot_state_IMU_pos[3], const float
  xt_robot_state_IMU_att[4], const float xt_robot_state_IMU_gyro_bias[3], const
  float xt_robot_state_IMU_acc_bias[3], const float xt_robot_state_pos[3], const
  float xt_robot_state_att[4], const float xt_robot_state_vel[3], const float
  xt_origin_pos[3], const float xt_origin_att[4], float world_state_pos[3],
  float world_state_att[4], float world_state_vel[3], float
  world_state_IMU_gyro_bias[3], float world_state_IMU_acc_bias[3], float
  world_state_IMU_pos[3], float world_state_IMU_att[4])
{
  float R_ow[9];
  __attribute__((aligned(16))) float fv13[3];
  int ix;
  int ixstart;
  float b_xt_robot_state_att[9];
  float c[9];
  int itmp;
  float varargin_1[4];
  float mtmp;
  boolean_T exitg1;

  // 'getWorldState:4' R_co = RotFromQuatJ(xt.robot_state.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  // 'getWorldState:5' R_ow = RotFromQuatJ(xt.origin.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[6] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[1] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[2] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[5] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];

  // 'getWorldState:7' world_state.pos = xt.origin.pos + R_ow' * xt.robot_state.pos; 
  for (ix = 0; ix < 3; ix++) {
    fv13[ix] = 0.0F;
    for (ixstart = 0; ixstart < 3; ixstart++) {
      fv13[ix] += R_ow[ixstart + 3 * ix] * xt_robot_state_pos[ixstart];
    }
  }

  mw_neon_mm_add_f32x4(*(float (*)[3])&xt_origin_pos[0], 3, 1, fv13,
                       &world_state_pos[0]);

  // 'getWorldState:8' world_state.att = QuatFromRotJ(R_co * R_ow);
  b_xt_robot_state_att[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
    xt_robot_state_att[1] * xt_robot_state_att[1]) - xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  b_xt_robot_state_att[3] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1]
    + xt_robot_state_att[2] * xt_robot_state_att[3]);
  b_xt_robot_state_att[6] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2]
    - xt_robot_state_att[1] * xt_robot_state_att[3]);
  b_xt_robot_state_att[1] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1]
    - xt_robot_state_att[2] * xt_robot_state_att[3]);
  b_xt_robot_state_att[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
    xt_robot_state_att[1] * xt_robot_state_att[1]) - xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  b_xt_robot_state_att[7] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2]
    + xt_robot_state_att[0] * xt_robot_state_att[3]);
  b_xt_robot_state_att[2] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2]
    + xt_robot_state_att[1] * xt_robot_state_att[3]);
  b_xt_robot_state_att[5] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2]
    - xt_robot_state_att[0] * xt_robot_state_att[3]);
  b_xt_robot_state_att[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
    xt_robot_state_att[1] * xt_robot_state_att[1]) + xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  for (ix = 0; ix < 3; ix++) {
    for (ixstart = 0; ixstart < 3; ixstart++) {
      c[ix + 3 * ixstart] = 0.0F;
      for (itmp = 0; itmp < 3; itmp++) {
        c[ix + 3 * ixstart] += b_xt_robot_state_att[ix + 3 * itmp] * R_ow[itmp +
          3 * ixstart];
      }
    }
  }

  //  THIS IS OK, It is according to the NASA memo found
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
  // % speed optimization
  // 'QuatFromRotJ:50' [~, index] = max([1+R(1,1)-(R(2,2)+R(3,3)), 1+R(2,2)-(R(1,1)+R(3,3)), 1+R(3,3)-(R(1,1)+R(2,2)), 1+(R(1,1)+R(2,2)+R(3,3))]); 
  varargin_1[0] = (1.0F + c[0]) - (c[4] + c[8]);
  varargin_1[1] = (1.0F + c[4]) - (c[0] + c[8]);
  varargin_1[2] = (1.0F + c[8]) - (c[0] + c[4]);
  varargin_1[3] = 1.0F + ((c[0] + c[4]) + c[8]);
  ixstart = 1;
  mtmp = varargin_1[0];
  itmp = 1;
  if (rtIsNaNF(varargin_1[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 5)) {
      ixstart = ix;
      if (!rtIsNaNF(varargin_1[ix - 1])) {
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

  // 'QuatFromRotJ:50' ~
  // 'QuatFromRotJ:52' if(index==1)
  if (itmp == 1) {
    // 'QuatFromRotJ:53' Q = [sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2;
    // 'QuatFromRotJ:54'         (R(1,2)+R(2,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:55'         (R(1,3)+R(3,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:56'         (R(2,3)-R(3,2))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
    world_state_att[0] = sqrtf((1.0F + 2.0F * c[0]) - ((c[0] + c[4]) + c[8])) /
      2.0F;
    world_state_att[1] = (c[3] + c[1]) / (4.0F * (sqrtf((1.0F + 2.0F * c[0]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
    world_state_att[2] = (c[6] + c[2]) / (4.0F * (sqrtf((1.0F + 2.0F * c[0]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
    world_state_att[3] = (c[7] - c[5]) / (4.0F * (sqrtf((1.0F + 2.0F * c[0]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
  } else if (itmp == 2) {
    // 'QuatFromRotJ:57' elseif(index==2)
    // 'QuatFromRotJ:58' Q = [(R(1,2)+R(2,1))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:59'         (sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2);
    // 'QuatFromRotJ:60'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:61'         (R(3,1)-R(1,3))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
    world_state_att[0] = (c[3] + c[1]) / (4.0F * (sqrtf((1.0F + 2.0F * c[4]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
    world_state_att[1] = sqrtf((1.0F + 2.0F * c[4]) - ((c[0] + c[4]) + c[8])) /
      2.0F;
    world_state_att[2] = (c[7] + c[5]) / (4.0F * (sqrtf((1.0F + 2.0F * c[4]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
    world_state_att[3] = (c[2] - c[6]) / (4.0F * (sqrtf((1.0F + 2.0F * c[4]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
  } else if (itmp == 3) {
    // 'QuatFromRotJ:62' elseif(index==3)
    // 'QuatFromRotJ:63' Q = [(R(1,3)+R(3,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:64'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:65'         (sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2);
    // 'QuatFromRotJ:66'         (R(1,2)-R(2,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
    world_state_att[0] = (c[6] + c[2]) / (4.0F * (sqrtf((1.0F + 2.0F * c[8]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
    world_state_att[1] = (c[7] + c[5]) / (4.0F * (sqrtf((1.0F + 2.0F * c[8]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
    world_state_att[2] = sqrtf((1.0F + 2.0F * c[8]) - ((c[0] + c[4]) + c[8])) /
      2.0F;
    world_state_att[3] = (c[3] - c[1]) / (4.0F * (sqrtf((1.0F + 2.0F * c[8]) -
      ((c[0] + c[4]) + c[8])) / 2.0F));
  } else {
    // 'QuatFromRotJ:67' else
    // 'QuatFromRotJ:68' Q = [(R(2,3)-R(3,2))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:69'         (R(3,1)-R(1,3))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:70'         (R(1,2)-R(2,1))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:71'         (sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2);];
    world_state_att[0] = (c[7] - c[5]) / (4.0F * (sqrtf(1.0F + ((c[0] + c[4]) +
      c[8])) / 2.0F));
    world_state_att[1] = (c[2] - c[6]) / (4.0F * (sqrtf(1.0F + ((c[0] + c[4]) +
      c[8])) / 2.0F));
    world_state_att[2] = (c[3] - c[1]) / (4.0F * (sqrtf(1.0F + ((c[0] + c[4]) +
      c[8])) / 2.0F));
    world_state_att[3] = sqrtf(1.0F + ((c[0] + c[4]) + c[8])) / 2.0F;
  }

  // 'getWorldState:9' world_state.vel = R_ow' * xt.robot_state.vel;
  // 'getWorldState:10' world_state.IMU.gyro_bias = xt.robot_state.IMU.gyro_bias; 
  // 'getWorldState:11' world_state.IMU.acc_bias = xt.robot_state.IMU.acc_bias;
  // 'getWorldState:12' world_state.IMU.pos = xt.robot_state.IMU.pos;
  for (ixstart = 0; ixstart < 3; ixstart++) {
    world_state_vel[ixstart] = 0.0F;
    for (ix = 0; ix < 3; ix++) {
      world_state_vel[ixstart] += R_ow[ix + 3 * ixstart] * xt_robot_state_vel[ix];
    }

    world_state_IMU_gyro_bias[ixstart] = xt_robot_state_IMU_gyro_bias[ixstart];
    world_state_IMU_acc_bias[ixstart] = xt_robot_state_IMU_acc_bias[ixstart];
    world_state_IMU_pos[ixstart] = xt_robot_state_IMU_pos[ixstart];
  }

  // 'getWorldState:13' world_state.IMU.att = xt.robot_state.IMU.att;
  for (ixstart = 0; ixstart < 4; ixstart++) {
    world_state_IMU_att[ixstart] = xt_robot_state_IMU_att[ixstart];
  }
}

//
// Arguments    : float varargin_1
//                float varargin_2
//                float varargin_3
// Return Type  : double
//
static double h_fprintf(float varargin_1, float varargin_2, float varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[38] = { '	', 'g', 'y', 'r', 'o', ' ', 'b', 'i', 'a',
    's', ' ', 'i', 'n', 'i', 't', 'i', 'a', 'l', ' ', 'u', 'n', 'c', ':', ' ',
    '[', '%', 'f', ',', ' ', '%', 'f', ',', ' ', '%', 'f', ']', '\x0a', '\x00' };

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
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : int varargin_1
// Return Type  : void
//
static void h_log_warn(int varargin_1)
{
  char cv38[52];
  int i28;
  static const char cv39[52] = { 'T', 'r', 'y', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', 'f', 'o', 'r', 'c', 'e', ' ', 'i', 'n', 's', 'e', 'r', 't', ' ', 'f',
    'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', 'b', 'e', 'h', 'i', 'n',
    'd', ' ', 'i', 't', 's', ' ', 'a', 'n', 'c', 'h', 'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i28 = 0; i28 < 52; i28++) {
      cv38[i28] = cv39[i28];
    }

    LOG_WARN(cv38, varargin_1);
  }
}

//
// Arguments    : float varargin_1
//                float varargin_2
//                float varargin_3
// Return Type  : double
//
static double i_fprintf(float varargin_1, float varargin_2, float varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[37] = { '	', 'a', 'c', 'c', ' ', 'b', 'i', 'a', 's',
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
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : void
// Return Type  : void
//
static void i_log_warn()
{
  char cv42[54];
  int i30;
  static const char cv43[54] = { 'C', 'a', 'n', '\'', 't', ' ', 'f', 'i', 'x',
    ' ', 'a', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'b', 'e', 'c', 'a',
    'u', 's', 'e', ' ', 'n', 'o', 'n', 'e', ' ', 'h', 'a', 'v', 'e', ' ', 'a',
    'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', 's', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i30 = 0; i30 < 54; i30++) {
      cv42[i30] = cv43[i30];
    }

    LOG_WARN(cv42);
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
// Arguments    : const float z_u_l[2]
//                const float z_u_r[2]
//                const float c_cameraparams_CameraParameters[2]
//                const float d_cameraparams_CameraParameters[2]
//                const float e_cameraparams_CameraParameters[2]
//                const float f_cameraparams_CameraParameters[2]
//                const float cameraparams_r_lr[3]
//                const float cameraparams_R_lr[9]
//                float fp[3]
//                float b_m[6]
//                boolean_T *success
// Return Type  : void
//
static void initializePoint(const float z_u_l[2], const float z_u_r[2], const
  float c_cameraparams_CameraParameters[2], const float
  d_cameraparams_CameraParameters[2], const float
  e_cameraparams_CameraParameters[2], const float
  f_cameraparams_CameraParameters[2], const float cameraparams_r_lr[3], const
  float cameraparams_R_lr[9], float fp[3], float b_m[6], boolean_T *success)
{
  float ml[3];
  float mr[3];
  float tol;
  float fcnOutput;
  float b_pos[6];
  int i;
  int i20;
  static const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  float rot[18];
  float A[30];
  float b[6];
  int anchorIdx;
  int rankR;
  int j;
  float b_rot[9];
  static const signed char iv1[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  int jpvt[5];
  float tau[5];
  float x[5];

  // 'initializePoint:13' fx_l = cameraparams.CameraParameters1.FocalLength(1);
  // 'initializePoint:14' fy_l = cameraparams.CameraParameters1.FocalLength(2);
  // 'initializePoint:15' Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1); 
  // 'initializePoint:16' Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2); 
  // 'initializePoint:18' fx_r = cameraparams.CameraParameters2.FocalLength(1);
  // 'initializePoint:19' fy_r = cameraparams.CameraParameters2.FocalLength(2);
  // 'initializePoint:20' Cx_r = cameraparams.CameraParameters2.PrincipalPoint(1); 
  // 'initializePoint:21' Cy_r = cameraparams.CameraParameters2.PrincipalPoint(2); 
  // 'initializePoint:23' pos = [zeros(3,1), cameraparams.r_lr];
  // 'initializePoint:24' rot = [eye(3), cameraparams.R_lr'];
  // 'initializePoint:26' z_n_l = [(z_u_l(1) - Cx_l)/fx_l; (z_u_l(2) - Cy_l)/fy_l]; 
  // 'initializePoint:27' z_n_r = [(z_u_r(1) - Cx_r)/fx_r; (z_u_r(2) - Cy_r)/fy_r]; 
  // 'initializePoint:29' ml = [z_n_l; 1];
  ml[0] = (z_u_l[0] - d_cameraparams_CameraParameters[0]) /
    c_cameraparams_CameraParameters[0];
  ml[1] = (z_u_l[1] - d_cameraparams_CameraParameters[1]) /
    c_cameraparams_CameraParameters[1];
  ml[2] = 1.0F;

  // 'initializePoint:30' mr = [z_n_r; 1];
  mr[0] = (z_u_r[0] - f_cameraparams_CameraParameters[0]) /
    e_cameraparams_CameraParameters[0];
  mr[1] = (z_u_r[1] - f_cameraparams_CameraParameters[1]) /
    e_cameraparams_CameraParameters[1];
  mr[2] = 1.0F;

  // 'initializePoint:31' m = [ml/norm(ml),mr/norm(mr)];
  tol = norm(ml);
  fcnOutput = norm(mr);
  for (i = 0; i < 3; i++) {
    b_pos[i] = 0.0F;
    b_pos[3 + i] = cameraparams_r_lr[i];
    for (i20 = 0; i20 < 3; i20++) {
      rot[i20 + 3 * i] = iv0[i20 + 3 * i];
      rot[i20 + 3 * (i + 3)] = cameraparams_R_lr[i + 3 * i20];
    }

    b_m[i] = ml[i] / tol;
    b_m[3 + i] = mr[i] / fcnOutput;
  }

  //  normalized rays in left frame
  // 'initializePoint:34' ml_n_l = m(:,1);
  // 'initializePoint:35' mr_n_r = cameraparams.R_lr * m(:,2);
  for (i = 0; i < 3; i++) {
    ml[i] = 0.0F;
    for (i20 = 0; i20 < 3; i20++) {
      ml[i] += cameraparams_R_lr[i + 3 * i20] * b_m[3 + i20];
    }
  }

  // 'initializePoint:36' cross_prod = cross(ml_n_l, mr_n_r);
  // 'initializePoint:37' if cross_prod(2) > 0
  if (b_m[2] * ml[0] - b_m[0] * ml[2] > 0.0F) {
    // 'initializePoint:38' success = false;
    *success = false;

    // 'initializePoint:39' fp = m(:,1);
    for (i = 0; i < 3; i++) {
      fp[i] = b_m[i];
    }
  } else {
    // 'initializePoint:42' success = true;
    *success = true;

    // 'initializePoint:44' [ fp ] = triangulatePoint( pos, rot, m );
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
    // 'triangulatePoint:18' numAnchors = size(pos, 2);
    // 'triangulatePoint:20' A = zeros(3*numAnchors, numAnchors + 3);
    memset(&A[0], 0, 30U * sizeof(float));

    // 'triangulatePoint:21' b = zeros(3*numAnchors, 1);
    for (i = 0; i < 6; i++) {
      b[i] = 0.0F;
    }

    // 'triangulatePoint:23' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 2; anchorIdx++) {
      // 'triangulatePoint:24' currentAnchorPos = pos(:,anchorIdx);
      // 'triangulatePoint:25' currentAnchorRot = rot(:, (anchorIdx-1)*3 + (1:3)); 
      rankR = anchorIdx * 3;

      // 'triangulatePoint:26' currentM = m(:,anchorIdx);
      // 'triangulatePoint:28' A((anchorIdx-1)*3 + (1:3), anchorIdx) = currentAnchorRot'*currentM; 
      j = anchorIdx * 3;
      for (i = 0; i < 3; i++) {
        for (i20 = 0; i20 < 3; i20++) {
          b_rot[i20 + 3 * i] = rot[i + 3 * (i20 + rankR)];
        }
      }

      for (i = 0; i < 3; i++) {
        A[(i + j) + 6 * anchorIdx] = 0.0F;
        for (i20 = 0; i20 < 3; i20++) {
          A[(i + j) + 6 * anchorIdx] += b_rot[i + 3 * i20] * b_m[i20 + 3 *
            anchorIdx];
        }
      }

      // 'triangulatePoint:29' A((anchorIdx-1)*3 + (1:3), end - 3 + (1:3)) = -eye(3); 
      rankR = anchorIdx * 3;

      // 'triangulatePoint:31' b((anchorIdx-1)*3 + (1:3)) = - currentAnchorPos;
      j = anchorIdx * 3;
      for (i = 0; i < 3; i++) {
        for (i20 = 0; i20 < 3; i20++) {
          A[(i20 + rankR) + 6 * (2 + i)] = iv1[i20 + 3 * i];
        }

        b[i + j] = -b_pos[i + 3 * anchorIdx];
      }
    }

    //  condition = cond(A);
    // 'triangulatePoint:35' x = A\b;
    b_xgeqp3(A, tau, jpvt);
    rankR = 0;
    tol = 6.0F * fabsf(A[0]) * 1.1920929E-7F;
    while ((rankR < 5) && (fabsf(A[rankR + 6 * rankR]) >= tol)) {
      rankR++;
    }

    for (j = 0; j < 5; j++) {
      x[j] = 0.0F;
      if (tau[j] != 0.0F) {
        tol = b[j];
        for (i = j + 1; i + 1 < 7; i++) {
          tol += A[i + 6 * j] * b[i];
        }

        tol *= tau[j];
        if (tol != 0.0F) {
          b[j] -= tol;
          for (i = j + 1; i + 1 < 7; i++) {
            b[i] -= A[i + 6 * j] * tol;
          }
        }
      }
    }

    for (i = 0; i + 1 <= rankR; i++) {
      x[jpvt[i] - 1] = b[i];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
      x[jpvt[j] - 1] /= A[j + 6 * j];
      for (i = 0; i + 1 <= j; i++) {
        x[jpvt[i] - 1] -= x[jpvt[j] - 1] * A[i + 6 * j];
      }
    }

    // 'triangulatePoint:36' pf = x(end-2:end);
    for (i = 0; i < 3; i++) {
      fp[i] = x[i + 2];
    }

    //  is this better?
    //  A = [cameraparams.R_rl * ml, -mr];
    //  b = cameraparams.r_lr;
    //  x = A\b;
    //  fp = ml*x(1);
  }
}

//
// Arguments    : float varargin_1
// Return Type  : double
//
static double j_fprintf(float varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[18] = { '	', 'i', 'm', 'a', 'g', 'e', '_', 'n', 'o',
    'i', 's', 'e', ':', ' ', '%', 'f', '\x0a', '\x00' };

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
// Arguments    : float varargin_1
// Return Type  : double
//
static double k_fprintf(float varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[28] = { '	', 'i', 'n', 'v', '_', 'd', 'e', 'p', 't',
    'h', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l', '_', 'u', 'n', 'c', ':', ' ',
    '%', 'f', '\x0a', '\x00' };

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
  static const char cfmt[25] = { '	', 'm', 'a', 'x', '_', 'e', 'k', 'f', '_',
    'i', 't', 'e', 'r', 'a', 't', 'i', 'o', 'n', 's', ':', ' ', '%', 'd', '\x0a',
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
// log_error Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_ERROR does,
//  e.g. redefine ROS_ERROR
// Arguments    : void
// Return Type  : void
//
static void log_error()
{
  char cv12[27];
  int i11;
  static const char cv13[27] = { 'p', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'i', 'n', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u',
    'r', 'e', '\x00' };

  // 'log_error:6' if coder.target('MATLAB')
  // 'log_error:8' elseif ~coder.target('MEX')
  // 'log_error:9' coder.cinclude('<vio_logging.h>')
  // 'log_error:10' coder.ceval('LOG_ERROR', [str, 0], varargin{:});
  for (i11 = 0; i11 < 27; i11++) {
    cv12[i11] = cv13[i11];
  }

  LOG_ERROR(cv12);
}

//
// log_info Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
//  e.g. redefine ROS_INFO
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void log_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv10[54];
  int i10;
  static const char cv11[54] = { 'F', 'i', 'x', 'e', 'd', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ',
    'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', ' ', 'i', 's', ' ', 'n',
    'o', ' ', 'l', 'o', 'n', 'g', 'e', 'r', ' ', 'v', 'a', 'l', 'i', 'd', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i10 = 0; i10 < 54; i10++) {
      cv10[i10] = cv11[i10];
    }

    LOG_INFO(cv10, varargin_1, varargin_2, varargin_3);
  }
}

//
// log_warn Print to console in Matlab
//  in C++, vio_logging.h needs to be created to define what LOG_WARN does,
//  e.g. redefine ROS_WARN
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void log_warn(int varargin_1, int varargin_2, int varargin_3)
{
  char cv16[54];
  int i16;
  static const char cv17[54] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', '%', 'i', ')', ' ', 'i', 's',
    ' ', 'b', 'e', 'h', 'i', 'n', 'd', ' ', 'i', 't', 's', ' ', 'a', 'n', 'c',
    'h', 'o', 'r', ',', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'i', 'n', 'g', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_warn:8' if coder.target('MATLAB')
  // 'log_warn:12' elseif ~coder.target('MEX')
  // 'log_warn:13' coder.cinclude('<vio_logging.h>')
  // 'log_warn:14' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
    for (i16 = 0; i16 < 54; i16++) {
      cv16[i16] = cv17[i16];
    }

    LOG_WARN(cv16, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double m_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i2;
  char varargout_1_data[6];
  static const char cfmt[29] = { '	', 'd', 'e', 'l', 'a', 'y', 'e', 'd', '_',
    'i', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'a', 't', 'i', 'o', 'n', ':',
    ' ', '%', 's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i2 = 0; i2 < nbytesint; i2++) {
      varargout_1_data[i2] = varargin_1_data[i2];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const float x_data[]
//                const int x_size[1]
// Return Type  : float
//
static float median(const float x_data[], const int x_size[1])
{
  float y;
  int k;
  int midm1;
  int i;
  int idx_data[48];
  emxArray_int32_T *iwork;
  int n;
  int iwork_data[48];
  boolean_T p;
  int i2;
  int j;
  int pEnd;
  int b_p;
  int q;
  int qEnd;
  int kEnd;
  if (x_size[0] == 0) {
    y = ((real32_T)rtNaN);
  } else {
    k = x_size[0];
    midm1 = k / 2;
    i = (signed char)x_size[0];
    for (k = 0; k < i; k++) {
      idx_data[k] = 0;
    }

    emxInit_int32_T(&iwork, 1);
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
      if ((x_data[k - 1] <= x_data[k]) || rtIsNaNF(x_data[k])) {
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
              rtIsNaNF(x_data[idx_data[q] - 1])) {
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

    if (rtIsNaNF(x_data[idx_data[(signed char)x_size[0] - 1] - 1])) {
      y = x_data[idx_data[(signed char)x_size[0] - 1] - 1];
    } else if (midm1 << 1 == x_size[0]) {
      if (((x_data[idx_data[midm1 - 1] - 1] < 0.0F) && (x_data[idx_data[midm1] -
            1] >= 0.0F)) || rtIsInfF(x_data[idx_data[midm1 - 1] - 1]) ||
          rtIsInfF(x_data[idx_data[midm1] - 1])) {
        y = (x_data[idx_data[midm1 - 1] - 1] + x_data[idx_data[midm1] - 1]) /
          2.0F;
      } else {
        y = x_data[idx_data[midm1 - 1] - 1] + (x_data[idx_data[midm1] - 1] -
          x_data[idx_data[midm1 - 1] - 1]) / 2.0F;
      }
    } else {
      y = x_data[idx_data[midm1] - 1];
    }
  }

  return y;
}

//
// Arguments    : int idx[8]
//                float x[8]
//                int offset
//                int np
//                int nq
//                int iwork[8]
//                float xwork[8]
// Return Type  : void
//
static void merge(int idx[8], float x[8], int offset, int np, int nq, int iwork
                  [8], float xwork[8])
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
//                emxArray_real32_T *x
//                int offset
//                int n
//                int preSortLevel
//                emxArray_int32_T *iwork
//                emxArray_real32_T *xwork
// Return Type  : void
//
static void merge_block(emxArray_int32_T *idx, emxArray_real32_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real32_T *xwork)
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
// Arguments    : int a
//                int b
// Return Type  : int
//
static int mul_s32_s32_s32_sat(int a, int b)
{
  int result;
  unsigned int u32_clo;
  unsigned int u32_chi;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  if (((int)u32_chi > 0) || ((u32_chi == 0U) && (u32_clo >= 2147483648U))) {
    result = MAX_int32_T;
  } else if (((int)u32_chi < -1) || (((int)u32_chi == -1) && (u32_clo <
               2147483648U))) {
    result = MIN_int32_T;
  } else {
    result = (int)u32_clo;
  }

  return result;
}

//
// Arguments    : int in0
//                int in1
//                unsigned int *ptrOutBitsHi
//                unsigned int *ptrOutBitsLo
// Return Type  : void
//
static void mul_wide_s32(int in0, int in1, unsigned int *ptrOutBitsHi, unsigned
  int *ptrOutBitsLo)
{
  unsigned int absIn0;
  unsigned int absIn1;
  int negativeProduct;
  int in0Hi;
  int in0Lo;
  int in1Hi;
  int in1Lo;
  unsigned int productLoHi;
  unsigned int productLoLo;
  unsigned int outBitsLo;
  if (in0 < 0) {
    absIn0 = (unsigned int)-in0;
  } else {
    absIn0 = (unsigned int)in0;
  }

  if (in1 < 0) {
    absIn1 = (unsigned int)-in1;
  } else {
    absIn1 = (unsigned int)in1;
  }

  negativeProduct = !((in0 == 0) || ((in1 == 0) || ((in0 > 0) == (in1 > 0))));
  in0Hi = (int)(absIn0 >> 16U);
  in0Lo = (int)(absIn0 & 65535U);
  in1Hi = (int)(absIn1 >> 16U);
  in1Lo = (int)(absIn1 & 65535U);
  absIn0 = (unsigned int)in0Hi * in1Hi;
  absIn1 = (unsigned int)in0Hi * in1Lo;
  productLoHi = (unsigned int)in0Lo * in1Hi;
  productLoLo = (unsigned int)in0Lo * in1Lo;
  in0Hi = 0;
  outBitsLo = productLoLo + (productLoHi << 16U);
  if (outBitsLo < productLoLo) {
    in0Hi = 1;
  }

  productLoLo = outBitsLo;
  outBitsLo += absIn1 << 16U;
  if (outBitsLo < productLoLo) {
    in0Hi++;
  }

  absIn0 = ((in0Hi + absIn0) + (productLoHi >> 16U)) + (absIn1 >> 16U);
  if (negativeProduct) {
    absIn0 = ~absIn0;
    outBitsLo = ~outBitsLo;
    outBitsLo++;
    if (outBitsLo == 0U) {
      absIn0++;
    }
  }

  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = outBitsLo;
}

//
// # coder
// multiplyIdx Create a longer index array from idx
//    for example:
//    idx_mult = multiplyIdx([1 4 5], 2)
//    idx_mult = [1 2 7 8 9 10]
// Arguments    : const float idx_data[]
//                const int idx_size[1]
//                float idx_mult_data[]
//                int idx_mult_size[1]
// Return Type  : void
//
static void multiplyIdx(const float idx_data[], const int idx_size[1], float
  idx_mult_data[], int idx_mult_size[1])
{
  int c;
  int i8;
  int i;
  int j;

  // 'multiplyIdx:7' idx_mult = zeros(length(idx) * factor, 1);
  c = idx_size[0] << 1;
  idx_mult_size[0] = c;
  for (i8 = 0; i8 < c; i8++) {
    idx_mult_data[i8] = 0.0F;
  }

  // 'multiplyIdx:8' for i = 1:length(idx)
  for (i = 0; i < idx_size[0]; i++) {
    // 'multiplyIdx:9' for j = 1:factor
    for (j = 0; j < 2; j++) {
      // 'multiplyIdx:10' idx_mult((i-1)*factor + j) = (idx(i) - 1)*factor + j;
      idx_mult_data[(i << 1) + j] = (idx_data[i] - 1.0F) * 2.0F + (1.0F + (float)
        j);
    }
  }
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double n_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i3;
  char varargout_1_data[6];
  static const char cfmt[20] = { '	', 'f', 'i', 'x', 'e', 'd', '_', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ':', ' ', '%', 's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i3 = 0; i3 < nbytesint; i3++) {
      varargout_1_data[i3] = varargin_1_data[i3];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const float x[3]
// Return Type  : float
//
static float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 3; k++) {
    absxk = fabsf(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
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
  static const char cfmt[11] = { '	', 'm', 'o', 'n', 'o', ':', ' ', '%', 's',
    '\x0a', '\x00' };

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
  static const char cfmt[18] = { '	', 'f', 'u', 'l', 'l', '_', 's', 't', 'e',
    'r', 'e', 'o', ':', ' ', '%', 's', '\x0a', '\x00' };

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
// predictMeasurementStereo Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel undistorted coordinates where a feature given in the left camera
//    frame would be visible in both cameras
// Arguments    : const float fp_l[3]
//                const float c_stereoParams_CameraParameters[2]
//                const float d_stereoParams_CameraParameters[2]
//                const float e_stereoParams_CameraParameters[2]
//                const float f_stereoParams_CameraParameters[2]
//                const float stereoParams_r_lr[3]
//                const float stereoParams_R_rl[9]
//                float h_cin_l[2]
//                float h_cin_r[2]
// Return Type  : void
//
static void predictMeasurementStereo(const float fp_l[3], const float
  c_stereoParams_CameraParameters[2], const float
  d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], float h_cin_l[2], float h_cin_r[2])
{
  float h_c_n_l[2];
  int i;
  __attribute__((aligned(16))) float fv8[3];
  int i14;
  __attribute__((aligned(16))) float fp_r[3];

  // 'predictMeasurementStereo:7' h_cin_l = predictMeasurementMono(fp_l, stereoParams.CameraParameters1); 
  // predictMeasurementLeft Predict the measurement of a feature given in the left 
  // camera frame
  //    Get the normalized pixel coordinates where a feature given in the left camera 
  //    frame
  // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
  // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
  // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1);
  // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2);
  // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp_l[i] / fp_l[2];
  }

  //  normalized feature in camera frame
  // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
  // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
  h_cin_l[0] = h_c_n_l[0] * c_stereoParams_CameraParameters[0] +
    d_stereoParams_CameraParameters[0];
  h_cin_l[1] = h_c_n_l[1] * c_stereoParams_CameraParameters[1] +
    d_stereoParams_CameraParameters[1];

  // 'predictMeasurementStereo:9' fp_r = stereoParams.R_rl*fp_l - stereoParams.r_lr; 
  for (i = 0; i < 3; i++) {
    fv8[i] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      fv8[i] += stereoParams_R_rl[i + 3 * i14] * fp_l[i14];
    }
  }

  mw_neon_mm_sub_f32x4(fv8, 3, 1, *(float (*)[3])&stereoParams_r_lr[0], &fp_r[0]);

  // 'predictMeasurementStereo:11' h_cin_r = predictMeasurementMono(fp_r, stereoParams.CameraParameters2); 
  // predictMeasurementLeft Predict the measurement of a feature given in the left 
  // camera frame
  //    Get the normalized pixel coordinates where a feature given in the left camera 
  //    frame
  // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
  // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
  // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1);
  // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2);
  // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp_r[i] / fp_r[2];
  }

  //  normalized feature in camera frame
  // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
  // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
  h_cin_r[0] = h_c_n_l[0] * e_stereoParams_CameraParameters[0] +
    f_stereoParams_CameraParameters[0];
  h_cin_r[1] = h_c_n_l[1] * e_stereoParams_CameraParameters[1] +
    f_stereoParams_CameraParameters[1];
}

//
// print all parameters for debug check
// Arguments    : float c_noiseParameters_process_noise
//                float d_noiseParameters_process_noise
//                float e_noiseParameters_process_noise
//                float f_noiseParameters_process_noise
//                float g_noiseParameters_process_noise
//                const float c_noiseParameters_gyro_bias_ini[3]
//                const float c_noiseParameters_acc_bias_init[3]
//                float noiseParameters_image_noise
//                float c_noiseParameters_inv_depth_ini
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T VIOParameters_fixed_feature
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_mono
//                boolean_T VIOParameters_full_stereo
//                boolean_T VIOParameters_RANSAC
// Return Type  : void
//
static void printParams(float c_noiseParameters_process_noise, float
  d_noiseParameters_process_noise, float e_noiseParameters_process_noise, float
  f_noiseParameters_process_noise, float g_noiseParameters_process_noise, const
  float c_noiseParameters_gyro_bias_ini[3], const float
  c_noiseParameters_acc_bias_init[3], float noiseParameters_image_noise, float
  c_noiseParameters_inv_depth_ini, int c_VIOParameters_max_ekf_iterati,
  boolean_T VIOParameters_fixed_feature, boolean_T
  c_VIOParameters_delayed_initial, boolean_T VIOParameters_mono, boolean_T
  VIOParameters_full_stereo, boolean_T VIOParameters_RANSAC)
{
  char cv0[18];
  int i1;
  static const char cv1[18] = { 'N', 'o', 'i', 's', 'e', ' ', 'p', 'a', 'r', 'a',
    'm', 'e', 't', 'e', 'r', 's', ':', '\x00' };

  char cv2[15];
  static const char cv3[15] = { 'V', 'I', 'O', ' ', 'p', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '\x00' };

  int s_size[2];
  static const char cv4[4] = { 'T', 'r', 'u', 'e' };

  char s_data[5];
  static const char cv5[5] = { 'F', 'a', 'l', 's', 'e' };

  // 'SLAM:132' fprintf('\n');
  b_fprintf();

  // 'SLAM:133' log_info('Noise parameters:');
  // log_info Print to console in Matlab
  //  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
  //  e.g. redefine ROS_INFO
  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i1 = 0; i1 < 18; i1++) {
      cv0[i1] = cv1[i1];
    }

    LOG_INFO(cv0);
  }

  // 'SLAM:134' fprintf('\tqv: %f\n', noiseParameters.process_noise.qv);
  c_fprintf(c_noiseParameters_process_noise);

  // 'SLAM:135' fprintf('\tqw: %f\n', noiseParameters.process_noise.qw);
  d_fprintf(d_noiseParameters_process_noise);

  // 'SLAM:136' fprintf('\tqao: %f\n', noiseParameters.process_noise.qao);
  e_fprintf(e_noiseParameters_process_noise);

  // 'SLAM:137' fprintf('\tqwo: %f\n', noiseParameters.process_noise.qwo);
  f_fprintf(f_noiseParameters_process_noise);

  // 'SLAM:138' fprintf('\tqR_ci: %f\n', noiseParameters.process_noise.qR_ci);
  g_fprintf(g_noiseParameters_process_noise);

  // 'SLAM:139' fprintf('\tgyro bias initial unc: [%f, %f, %f]\n', noiseParameters.gyro_bias_initial_unc(1), noiseParameters.gyro_bias_initial_unc(2), noiseParameters.gyro_bias_initial_unc(3)); 
  h_fprintf(c_noiseParameters_gyro_bias_ini[0], c_noiseParameters_gyro_bias_ini
            [1], c_noiseParameters_gyro_bias_ini[2]);

  // 'SLAM:140' fprintf('\tacc bias initial unc: [%f, %f, %f]\n', noiseParameters.acc_bias_initial_unc(1), noiseParameters.acc_bias_initial_unc(2), noiseParameters.acc_bias_initial_unc(3)); 
  i_fprintf(c_noiseParameters_acc_bias_init[0], c_noiseParameters_acc_bias_init
            [1], c_noiseParameters_acc_bias_init[2]);

  // 'SLAM:141' fprintf('\timage_noise: %f\n', noiseParameters.image_noise);
  j_fprintf(noiseParameters_image_noise);

  // 'SLAM:142' fprintf('\tinv_depth_initial_unc: %f\n', noiseParameters.inv_depth_initial_unc); 
  k_fprintf(c_noiseParameters_inv_depth_ini);

  // 'SLAM:144' fprintf('\n');
  b_fprintf();

  // 'SLAM:145' log_info('VIO parameters');
  // log_info Print to console in Matlab
  //  in C++, vio_logging.h needs to be created to define what LOG_INFO does,
  //  e.g. redefine ROS_INFO
  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'log_info:7' if coder.target('MATLAB')
  // 'log_info:11' elseif ~coder.target('MEX')
  // 'log_info:12' coder.cinclude('<vio_logging.h>')
  // 'log_info:13' if debug_level >= 2
  if (debug_level >= 2.0F) {
    // 'log_info:14' coder.ceval('LOG_INFO', [str, 0], varargin{:});
    for (i1 = 0; i1 < 15; i1++) {
      cv2[i1] = cv3[i1];
    }

    LOG_INFO(cv2);
  }

  // 'SLAM:146' fprintf('\tmax_ekf_iterations: %d\n', int32(VIOParameters.max_ekf_iterations)); 
  l_fprintf(c_VIOParameters_max_ekf_iterati);

  // 'SLAM:147' fprintf('\tdelayed_initialization: %s\n', bool2str(VIOParameters.delayed_initialization)); 
  // 'SLAM:156' if bool
  if (c_VIOParameters_delayed_initial) {
    // 'SLAM:157' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    // 'SLAM:158' else
    // 'SLAM:159' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  m_fprintf(s_data, s_size);

  // 'SLAM:148' fprintf('\tfixed_feature: %s\n', bool2str(VIOParameters.fixed_feature)); 
  // 'SLAM:156' if bool
  if (VIOParameters_fixed_feature) {
    // 'SLAM:157' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    // 'SLAM:158' else
    // 'SLAM:159' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  n_fprintf(s_data, s_size);

  // 'SLAM:149' fprintf('\tmono: %s\n', bool2str(VIOParameters.mono));
  // 'SLAM:156' if bool
  if (VIOParameters_mono) {
    // 'SLAM:157' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    // 'SLAM:158' else
    // 'SLAM:159' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  o_fprintf(s_data, s_size);

  // 'SLAM:150' fprintf('\tfull_stereo: %s\n', bool2str(VIOParameters.full_stereo)); 
  // 'SLAM:156' if bool
  if (VIOParameters_full_stereo) {
    // 'SLAM:157' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    // 'SLAM:158' else
    // 'SLAM:159' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  p_fprintf(s_data, s_size);

  // 'SLAM:151' fprintf('\tRANSAC: %s\n', bool2str(VIOParameters.RANSAC));
  // 'SLAM:156' if bool
  if (VIOParameters_RANSAC) {
    // 'SLAM:157' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    // 'SLAM:158' else
    // 'SLAM:159' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  q_fprintf(s_data, s_size);

  // 'SLAM:152' fprintf('\n');
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
  static const char cfmt[13] = { '	', 'R', 'A', 'N', 'S', 'A', 'C', ':', ' ',
    '%', 's', '\x0a', '\x00' };

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
// Arguments    : const float q[4]
//                const float p[4]
//                float qp[4]
// Return Type  : void
//
static void quatmultJ(const float q[4], const float p[4], float qp[4])
{
  __attribute__((aligned(16))) float fv2[16];
  __attribute__((aligned(16))) float fv3[4];
  float fcnOutput;
  int i7;

  // 'quatmultJ:2' if coder.target('MATLAB')
  // 'quatmultJ:10' p1=p(1);
  // 'quatmultJ:11' p2=p(2);
  // 'quatmultJ:12' p3=p(3);
  // 'quatmultJ:13' p4=p(4);
  // 'quatmultJ:15' q1=q(1);
  // 'quatmultJ:16' q2=q(2);
  // 'quatmultJ:17' q3=q(3);
  // 'quatmultJ:18' q4=q(4);
  // 'quatmultJ:19' qp=[p4,-p3,p2,p1;
  // 'quatmultJ:20'     p3,p4,-p1,p2;
  // 'quatmultJ:21'     -p2,p1,p4,p3;
  // 'quatmultJ:22'     -p1,-p2,-p3,p4]*[q1;q2;q3;q4];
  fv2[0] = p[3];
  fv2[4] = -p[2];
  fv2[8] = p[1];
  fv2[12] = p[0];
  fv2[1] = p[2];
  fv2[5] = p[3];
  fv2[9] = -p[0];
  fv2[13] = p[1];
  fv2[2] = -p[1];
  fv2[6] = p[0];
  fv2[10] = p[3];
  fv2[14] = p[2];
  fv2[3] = -p[0];
  fv2[7] = -p[1];
  fv2[11] = -p[2];
  fv2[15] = p[3];
  fv3[0] = q[0];
  fv3[1] = q[1];
  fv3[2] = q[2];
  fv3[3] = q[3];
  mw_neon_mv_mul_f32x4(fv2, 4, 4, fv3, &qp[0]);

  // 'quatmultJ:23' qp = qp/norm(qp);
  fcnOutput = b_norm(qp);
  for (i7 = 0; i7 < 4; i7++) {
    qp[i7] /= fcnOutput;
  }
}

//
// Arguments    : const emxArray_real32_T *A
// Return Type  : int
//
static int rankFromQR(const emxArray_real32_T *A)
{
  int r;
  int minmn;
  int maxmn;
  float tol;
  r = 0;
  if (A->size[0] < A->size[1]) {
    minmn = A->size[0];
    maxmn = A->size[1];
  } else {
    minmn = A->size[1];
    maxmn = A->size[0];
  }

  if (minmn > 0) {
    tol = (float)maxmn * fabsf(A->data[0]) * 1.1920929E-7F;
    while ((r < minmn) && (fabsf(A->data[r + A->size[0] * r]) >= tol)) {
      r++;
    }
  }

  return r;
}

//
// Arguments    : const float x[3]
//                float y
//                float z[3]
// Return Type  : void
//
static void rdivide(const float x[3], float y, float z[3])
{
  int i;
  for (i = 0; i < 3; i++) {
    z[i] = x[i] / y;
  }
}

//
// Arguments    : float x[8]
//                int idx[8]
// Return Type  : void
//
static void sort(float x[8], int idx[8])
{
  b_sort(x, idx);
}

//
// Arguments    : emxArray_real32_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void sortIdx(emxArray_real32_T *x, emxArray_int32_T *idx)
{
  emxArray_real32_T *b_x;
  int ib;
  int wOffset;
  int b_m;
  int n;
  float x4[4];
  int idx4[4];
  emxArray_int32_T *iwork;
  emxArray_real32_T *xwork;
  int nNaNs;
  int k;
  int i3;
  int i4;
  signed char perm[4];
  int nNonNaN;
  int nBlocks;
  int b_iwork[256];
  float b_xwork[256];
  int bLen2;
  int nPairs;
  int exitg1;
  emxInit_real32_T(&b_x, 1);
  ib = x->size[0];
  wOffset = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, wOffset, (int)sizeof(float));
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
    x4[b_m] = 0.0F;
    idx4[b_m] = 0;
  }

  emxInit_int32_T(&iwork, 1);
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

  emxInit_real32_T(&xwork, 1);
  b_m = x->size[0];
  wOffset = xwork->size[0];
  xwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)xwork, wOffset, (int)sizeof(float));
  b_m = xwork->size[0];
  wOffset = xwork->size[0];
  xwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)xwork, wOffset, (int)sizeof(float));
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    xwork->data[wOffset] = 0.0F;
  }

  nNaNs = 1;
  ib = 0;
  for (k = 0; k + 1 <= n; k++) {
    if (rtIsNaNF(b_x->data[k])) {
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

  emxFree_real32_T(&xwork);
  emxFree_int32_T(&iwork);
  wOffset = x->size[0];
  x->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)x, wOffset, (int)sizeof(float));
  b_m = b_x->size[0];
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    x->data[wOffset] = b_x->data[wOffset];
  }

  emxFree_real32_T(&b_x);
}

//
// Arguments    : const float A[36]
//                float U[6]
// Return Type  : void
//
static void svd(const float A[36], float U[6])
{
  float b_A[36];
  float s[6];
  float e[6];
  float work[6];
  int kase;
  int q;
  int qs;
  boolean_T apply_transform;
  float ztest0;
  int ixstart;
  int b_m;
  int iter;
  float snorm;
  float b;
  float ztest;
  int exitg3;
  boolean_T exitg2;
  float f;
  float varargin_1[5];
  float mtmp;
  boolean_T exitg1;
  float sqds;
  memcpy(&b_A[0], &A[0], 36U * sizeof(float));
  for (kase = 0; kase < 6; kase++) {
    s[kase] = 0.0F;
    e[kase] = 0.0F;
    work[kase] = 0.0F;
  }

  for (q = 0; q < 5; q++) {
    qs = q + 6 * q;
    apply_transform = false;
    ztest0 = e_xnrm2(6 - q, b_A, qs + 1);
    if (ztest0 > 0.0F) {
      apply_transform = true;
      if (b_A[qs] < 0.0F) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      if (fabsf(s[q]) >= 9.86076132E-32F) {
        ztest0 = 1.0F / s[q];
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
      s[q] = 0.0F;
    }

    for (kase = q + 1; kase + 1 < 7; kase++) {
      ixstart = q + 6 * kase;
      if (apply_transform) {
        xaxpy(6 - q, -(xdotc(6 - q, b_A, qs + 1, b_A, ixstart + 1) / b_A[q + 6 *
                       q]), qs + 1, b_A, ixstart + 1);
      }

      e[kase] = b_A[ixstart];
    }

    if (q + 1 <= 4) {
      ztest0 = f_xnrm2(5 - q, e, q + 2);
      if (ztest0 == 0.0F) {
        e[q] = 0.0F;
      } else {
        if (e[q + 1] < 0.0F) {
          e[q] = -ztest0;
        } else {
          e[q] = ztest0;
        }

        ztest0 = e[q];
        if (fabsf(e[q]) >= 9.86076132E-32F) {
          ztest0 = 1.0F / e[q];
          for (ixstart = q + 1; ixstart + 1 < 7; ixstart++) {
            e[ixstart] *= ztest0;
          }
        } else {
          for (ixstart = q + 1; ixstart + 1 < 7; ixstart++) {
            e[ixstart] /= ztest0;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (kase = q + 1; kase + 1 < 7; kase++) {
          work[kase] = 0.0F;
        }

        for (kase = q + 1; kase + 1 < 7; kase++) {
          b_xaxpy(5 - q, e[kase], b_A, (q + 6 * kase) + 2, work, q + 2);
        }

        for (kase = q + 1; kase + 1 < 7; kase++) {
          c_xaxpy(5 - q, -e[kase] / e[q + 1], work, q + 2, b_A, (q + 6 * kase) +
                  2);
        }
      }
    }
  }

  b_m = 4;
  s[5] = b_A[35];
  e[4] = b_A[34];
  e[5] = 0.0F;
  iter = 0;
  snorm = 0.0F;
  for (q = 0; q < 6; q++) {
    b = e[q];
    if (s[q] != 0.0F) {
      ztest = fabsf(s[q]);
      ztest0 = s[q] / ztest;
      s[q] = ztest;
      if (q + 1 < 6) {
        b = e[q] / ztest0;
      }
    }

    if ((q + 1 < 6) && (b != 0.0F)) {
      ztest = fabsf(b);
      ztest0 = b;
      b = ztest;
      s[q + 1] *= ztest / ztest0;
    }

    snorm = fmaxf(snorm, fmaxf(fabsf(s[q]), fabsf(b)));
    e[q] = b;
  }

  while ((b_m + 2 > 0) && (!(iter >= 75))) {
    kase = b_m;
    do {
      exitg3 = 0;
      q = kase + 1;
      if (kase + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = fabsf(e[kase]);
        if ((ztest0 <= 1.1920929E-7F * (fabsf(s[kase]) + fabsf(s[kase + 1]))) ||
            (ztest0 <= 9.86076132E-32F) || ((iter > 20) && (ztest0 <=
              1.1920929E-7F * snorm))) {
          e[kase] = 0.0F;
          exitg3 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg3 == 0);

    if (kase + 1 == b_m + 1) {
      kase = 4;
    } else {
      qs = b_m + 2;
      ixstart = b_m + 2;
      exitg2 = false;
      while ((!exitg2) && (ixstart >= kase + 1)) {
        qs = ixstart;
        if (ixstart == kase + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0F;
          if (ixstart < b_m + 2) {
            ztest0 = fabsf(e[ixstart - 1]);
          }

          if (ixstart > kase + 2) {
            ztest0 += fabsf(e[ixstart - 2]);
          }

          ztest = fabsf(s[ixstart - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            s[ixstart - 1] = 0.0F;
            exitg2 = true;
          } else {
            ixstart--;
          }
        }
      }

      if (qs == kase + 1) {
        kase = 3;
      } else if (qs == b_m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[b_m];
      e[b_m] = 0.0F;
      for (ixstart = b_m; ixstart + 1 >= q + 1; ixstart--) {
        ztest0 = s[ixstart];
        xrotg(&ztest0, &f, &ztest, &b);
        s[ixstart] = ztest0;
        if (ixstart + 1 > q + 1) {
          f = -b * e[ixstart - 1];
          e[ixstart - 1] *= ztest;
        }
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      while (q + 1 <= b_m + 2) {
        xrotg(&s[q], &f, &ztest, &b);
        f = -b * e[q];
        e[q] *= ztest;
        q++;
      }
      break;

     case 3:
      varargin_1[0] = fabsf(s[b_m + 1]);
      varargin_1[1] = fabsf(s[b_m]);
      varargin_1[2] = fabsf(e[b_m]);
      varargin_1[3] = fabsf(s[q]);
      varargin_1[4] = fabsf(e[q]);
      ixstart = 1;
      mtmp = varargin_1[0];
      if (rtIsNaNF(varargin_1[0])) {
        kase = 2;
        exitg1 = false;
        while ((!exitg1) && (kase < 6)) {
          ixstart = kase;
          if (!rtIsNaNF(varargin_1[kase - 1])) {
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

      f = s[b_m + 1] / mtmp;
      ztest0 = s[b_m] / mtmp;
      ztest = e[b_m] / mtmp;
      sqds = s[q] / mtmp;
      b = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0F;
      ztest0 = f * ztest;
      ztest0 *= ztest0;
      if ((b != 0.0F) || (ztest0 != 0.0F)) {
        ztest = sqrtf(b * b + ztest0);
        if (b < 0.0F) {
          ztest = -ztest;
        }

        ztest = ztest0 / (b + ztest);
      } else {
        ztest = 0.0F;
      }

      f = (sqds + f) * (sqds - f) + ztest;
      ztest0 = sqds * (e[q] / mtmp);
      for (ixstart = q + 1; ixstart <= b_m + 1; ixstart++) {
        xrotg(&f, &ztest0, &ztest, &b);
        if (ixstart > q + 1) {
          e[ixstart - 2] = f;
        }

        f = ztest * s[ixstart - 1] + b * e[ixstart - 1];
        e[ixstart - 1] = ztest * e[ixstart - 1] - b * s[ixstart - 1];
        ztest0 = b * s[ixstart];
        s[ixstart] *= ztest;
        s[ixstart - 1] = f;
        xrotg(&s[ixstart - 1], &ztest0, &ztest, &b);
        f = ztest * e[ixstart - 1] + b * s[ixstart];
        s[ixstart] = -b * e[ixstart - 1] + ztest * s[ixstart];
        ztest0 = b * e[ixstart];
        e[ixstart] *= ztest;
      }

      e[b_m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
      }

      kase = q + 1;
      while ((q + 1 < 6) && (s[q] < s[kase])) {
        ztest = s[q];
        s[q] = s[kase];
        s[kase] = ztest;
        q = kase;
        kase++;
      }

      iter = 0;
      b_m--;
      break;
    }
  }

  for (ixstart = 0; ixstart < 6; ixstart++) {
    U[ixstart] = s[ixstart];
  }
}

//
// UNDISTORTPOINT Undistort a point (or points) that are from a camera with
// the calibration cameraparams
//    Undistort a point or set of points from one camera. Depending on the
//    camera model used to calibrate the camera, the appropriate undistortion
//    is applied
// Arguments    : const float pt_d_data[]
//                const int pt_d_size[1]
//                int cameraparams_ATAN
//                const float cameraparams_FocalLength[2]
//                const float cameraparams_PrincipalPoint[2]
//                const float cameraparams_RadialDistortion[3]
//                int cameraparams_DistortionModel
//                float pt_u_data[]
//                int pt_u_size[1]
// Return Type  : void
//
static void undistortPoint(const float pt_d_data[], const int pt_d_size[1], int
  cameraparams_ATAN, const float cameraparams_FocalLength[2], const float
  cameraparams_PrincipalPoint[2], const float cameraparams_RadialDistortion[3],
  int cameraparams_DistortionModel, float pt_u_data[], int pt_u_size[1])
{
  int rankR;
  int i9;
  float fx;
  float fy;
  float Cx;
  float Cy;
  float c;
  int i;
  float pt_d_n[2];
  float x;
  float b_x[3];
  float c_x[3];
  float A[3];
  float xnorm;
  float atmp;
  float tau;
  int knt;
  float r_u;
  int b_i;
  float k1;
  float k2;
  float k3;
  float r_d_sq;
  float r_u_sq;
  int c_i;
  boolean_T exitg1;
  float diff;
  char cv6[48];
  static const char cv7[48] = { 'P', 'i', 'x', 'e', 'l', ' ', 'r', 'a', 'd', 'i',
    'u', 's', ' ', 'i', 'n', 'v', 'e', 'r', 's', 'i', 'o', 'n', ':', ' ', 'R',
    'e', 'a', 'c', 'h', 'e', 'd', ' ', 'i', 't', 'e', 'r', 'a', 't', 'i', 'o',
    'n', ' ', 'l', 'i', 'm', 'i', 't', '\x00' };

  char cv8[52];
  static const char cv9[52] = { 'n', 'e', 'g', 'a', 't', 'i', 'v', 'e', ' ', 'u',
    'n', 'd', 'i', 's', 't', 'o', 'r', 't', 'e', 'd', ' ', 'r', 'a', 'd', 'i',
    'u', 's', '.', ' ', 'r', 'd', '_', 's', 'q', ' ', '=', ' ', '%', 'f', ',',
    ' ', 'r', 'u', '_', 's', 'q', ' ', '=', ' ', '%', 'f', '\x00' };

  float coeff;

  // 'undistortPoint:8' if cameraparams.DistortionModel == cameraparams.ATAN
  if (cameraparams_DistortionModel == cameraparams_ATAN) {
    // 'undistortPoint:9' pt_u = undistortPointAtan(pt_d, cameraparams);
    // % Atan
    // 'undistortPoint:20' if mod(length(pt_d), 2)
    // 'undistortPoint:24' pt_u = pt_d;
    pt_u_size[0] = pt_d_size[0];
    rankR = pt_d_size[0];
    for (i9 = 0; i9 < rankR; i9++) {
      pt_u_data[i9] = pt_d_data[i9];
    }

    // 'undistortPoint:26' fx = cameraparams.FocalLength(1);
    fx = cameraparams_FocalLength[0];

    // 'undistortPoint:27' fy = cameraparams.FocalLength(2);
    fy = cameraparams_FocalLength[1];

    // 'undistortPoint:28' Cx = cameraparams.PrincipalPoint(1);
    Cx = cameraparams_PrincipalPoint[0];

    // 'undistortPoint:29' Cy = cameraparams.PrincipalPoint(2);
    Cy = cameraparams_PrincipalPoint[1];

    // 'undistortPoint:30' w  = cameraparams.RadialDistortion;
    // 'undistortPoint:32' for i = 1:length(pt_d)/2
    c = (float)pt_d_size[0] / 2.0F;
    for (i = 0; i < (int)c; i++) {
      // 'undistortPoint:34' pt_d_n = [(pt_d((i-1)*2 + 1)-Cx)/fx;
      // 'undistortPoint:35'     (pt_d((i-1)*2 + 2)-Cy)/fy];
      pt_d_n[0] = (pt_d_data[i << 1] - Cx) / fx;
      pt_d_n[1] = (pt_d_data[(i << 1) + 1] - Cy) / fy;

      // 'undistortPoint:37' r_d = sqrt(pt_d_n(1)^2 + pt_d_n(2)^2);
      x = sqrtf(pt_d_n[0] * pt_d_n[0] + pt_d_n[1] * pt_d_n[1]);

      // 'undistortPoint:39' if r_d > 0.001
      if (x > 0.001F) {
        // 'undistortPoint:40' r_u = tan(r_d * w)/(2*tan(w/2));
        for (rankR = 0; rankR < 3; rankR++) {
          xnorm = 2.0F * tanf(cameraparams_RadialDistortion[rankR] / 2.0F);
          A[rankR] = xnorm;
          b_x[rankR] = tanf(x * cameraparams_RadialDistortion[rankR]);
          c_x[rankR] = xnorm;
        }

        atmp = c_x[0];
        tau = 0.0F;
        xnorm = xnrm2(c_x);
        if (xnorm != 0.0F) {
          xnorm = hypotf(c_x[0], xnorm);
          if (c_x[0] >= 0.0F) {
            xnorm = -xnorm;
          }

          if (fabsf(xnorm) < 9.86076132E-32F) {
            knt = 0;
            do {
              knt++;
              for (rankR = 0; rankR < 2; rankR++) {
                A[rankR + 1] *= 1.01412048E+31F;
              }

              xnorm *= 1.01412048E+31F;
              atmp *= 1.01412048E+31F;
            } while (!(fabsf(xnorm) >= 9.86076132E-32F));

            xnorm = hypotf(atmp, xnrm2(A));
            if (atmp >= 0.0F) {
              xnorm = -xnorm;
            }

            tau = (xnorm - atmp) / xnorm;
            atmp = 1.0F / (atmp - xnorm);
            for (rankR = 0; rankR < 2; rankR++) {
              A[rankR + 1] *= atmp;
            }

            for (rankR = 1; rankR <= knt; rankR++) {
              xnorm *= 9.86076132E-32F;
            }

            atmp = xnorm;
          } else {
            tau = (xnorm - c_x[0]) / xnorm;
            atmp = 1.0F / (c_x[0] - xnorm);
            for (rankR = 0; rankR < 2; rankR++) {
              A[rankR + 1] *= atmp;
            }

            atmp = xnorm;
          }
        }

        A[0] = atmp;
        rankR = 0;
        xnorm = 3.0F * fabsf(atmp) * 1.1920929E-7F;
        while ((rankR < 1) && (fabsf(atmp) >= xnorm)) {
          rankR = 1;
        }

        r_u = 0.0F;
        for (i9 = 0; i9 < 3; i9++) {
          c_x[i9] = b_x[i9];
        }

        if (tau != 0.0F) {
          xnorm = b_x[0];
          for (b_i = 0; b_i < 2; b_i++) {
            xnorm += A[b_i + 1] * b_x[b_i + 1];
          }

          xnorm *= tau;
          if (xnorm != 0.0F) {
            c_x[0] = b_x[0] - xnorm;
            for (b_i = 0; b_i < 2; b_i++) {
              c_x[b_i + 1] -= A[b_i + 1] * xnorm;
            }
          }
        }

        b_i = 1;
        while (b_i <= rankR) {
          r_u = c_x[0];
          b_i = 2;
        }

        while (rankR > 0) {
          r_u /= atmp;
          rankR = 0;
        }

        // 'undistortPoint:42' pt_u_n = pt_d_n*r_u/r_d;
        for (i9 = 0; i9 < 2; i9++) {
          pt_d_n[i9] = pt_d_n[i9] * r_u / x;
        }

        // 'undistortPoint:43' pt_u((i-1)*2 + (1:2)) = [pt_u_n(1)*fx + Cx; pt_u_n(2)*fy + Cy]; 
        rankR = (i << 1) - 1;
        pt_u_data[rankR + 1] = pt_d_n[0] * fx + Cx;
        pt_u_data[rankR + 2] = pt_d_n[1] * fy + Cy;
      } else {
        // 'undistortPoint:45' else
        // 'undistortPoint:46' pt_u((i-1)*2 + (1:2)) = pt_d((i-1)*2 + (1:2));
        rankR = (i << 1) - 1;
        knt = (i << 1) - 1;
        for (i9 = 0; i9 < 2; i9++) {
          pt_u_data[(i9 + rankR) + 1] = pt_d_data[(i9 + knt) + 1];
        }
      }
    }
  } else {
    // 'undistortPoint:10' else
    // 'undistortPoint:11' pt_u = undistortPointPB(pt_d, cameraparams);
    // % Plumb Bob
    // 'undistortPoint:58' if mod(length(pt_d), 2)
    // 'undistortPoint:62' pt_u = pt_d;
    pt_u_size[0] = pt_d_size[0];
    rankR = pt_d_size[0];
    for (i9 = 0; i9 < rankR; i9++) {
      pt_u_data[i9] = pt_d_data[i9];
    }

    // 'undistortPoint:64' fx = cameraParameters.FocalLength(1);
    fx = cameraparams_FocalLength[0];

    // 'undistortPoint:65' fy = cameraParameters.FocalLength(2);
    fy = cameraparams_FocalLength[1];

    // 'undistortPoint:66' Cx = cameraParameters.PrincipalPoint(1);
    Cx = cameraparams_PrincipalPoint[0];

    // 'undistortPoint:67' Cy = cameraParameters.PrincipalPoint(2);
    Cy = cameraparams_PrincipalPoint[1];

    // 'undistortPoint:68' k1 = cameraParameters.RadialDistortion(1);
    k1 = cameraparams_RadialDistortion[0];

    // 'undistortPoint:69' k2 = cameraParameters.RadialDistortion(2);
    k2 = cameraparams_RadialDistortion[1];

    // 'undistortPoint:70' k3 = cameraParameters.RadialDistortion(3);
    k3 = cameraparams_RadialDistortion[2];

    // 'undistortPoint:72' for i = 1:length(pt_d)/2
    c = (float)pt_d_size[0] / 2.0F;
    for (i = 0; i < (int)c; i++) {
      // 'undistortPoint:74' pt_d_n = [(pt_d((i-1)*2 + 1)-Cx)/fx;
      // 'undistortPoint:75'     (pt_d((i-1)*2 + 2)-Cy)/fy];
      pt_d_n[0] = (pt_d_data[i << 1] - Cx) / fx;
      pt_d_n[1] = (pt_d_data[(i << 1) + 1] - Cy) / fy;

      // 'undistortPoint:77' r_d_sq = pt_d_n(1)^2 + pt_d_n(2)^2;
      r_d_sq = pt_d_n[0] * pt_d_n[0] + pt_d_n[1] * pt_d_n[1];

      // 'undistortPoint:78' r_u_sq = get_r_u(k1, k2, k3, r_d_sq);
      // get_r_u Get undistorted radius from distorted radius
      //    Get the pixel radius of the undistorted pixels from a distorted pixel 
      //    radius and distortion parameters
      // 'get_r_u:5' x = rd_sq;
      r_u_sq = r_d_sq;

      // 'get_r_u:6' thresh = 1e-6;
      // 'get_r_u:8' for i = 1:100
      b_i = 1;
      c_i = 0;
      exitg1 = false;
      while ((!exitg1) && (c_i < 100)) {
        b_i = c_i + 1;

        // 'get_r_u:9' f_x = x * (1 + k1*x + k2*x^2 + k3*x^3)^2 - rd_sq;
        xnorm = ((1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * powf
          (r_u_sq, 3.0F);

        // 'get_r_u:10' f_prime_x = (1 + k1*x + k2*x^2 + k3*x^3)^2 + 2*x*(1 + k1*x + k2*x^2 + k3*x^3)^2*(k1 + 2*k2*x + 2*k3*x^2); 
        atmp = ((1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * powf
          (r_u_sq, 3.0F);
        tau = ((1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * powf(r_u_sq,
          3.0F);

        // 'get_r_u:13' diff = f_x / f_prime_x;
        diff = (r_u_sq * (xnorm * xnorm) - r_d_sq) / (atmp * atmp + 2.0F *
          r_u_sq * (tau * tau) * ((k1 + 2.0F * k2 * r_u_sq) + 2.0F * k3 *
          (r_u_sq * r_u_sq)));

        // 'get_r_u:14' x = x - diff;
        r_u_sq -= diff;

        // 'get_r_u:15' if diff < thresh && diff > -thresh
        if ((diff < 1.0E-6F) && (diff > -1.0E-6F)) {
          exitg1 = true;
        } else {
          c_i++;
        }
      }

      // 'get_r_u:20' if i == 100
      if ((b_i == 100) && (debug_level >= 2.0F)) {
        // 'get_r_u:21' log_warn('Pixel radius inversion: Reached iteration limit') 
        // log_warn Print to console in Matlab
        //  in C++, vio_logging.h needs to be created to define what LOG_WARN does, 
        //  e.g. redefine ROS_WARN
        //  debug_level == 0: print errors, == 1: print warnings, == 2: print info 
        // 'log_warn:8' if coder.target('MATLAB')
        // 'log_warn:12' elseif ~coder.target('MEX')
        // 'log_warn:13' coder.cinclude('<vio_logging.h>')
        // 'log_warn:14' if debug_level >= 2
        // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
        for (i9 = 0; i9 < 48; i9++) {
          cv6[i9] = cv7[i9];
        }

        LOG_WARN(cv6);
      }

      // 'get_r_u:24' if x < 0
      if ((r_u_sq < 0.0F) && (debug_level >= 2.0F)) {
        // 'get_r_u:25' log_warn('negative undistorted radius. rd_sq = %f, ru_sq = %f', rd_sq, x); 
        // log_warn Print to console in Matlab
        //  in C++, vio_logging.h needs to be created to define what LOG_WARN does, 
        //  e.g. redefine ROS_WARN
        //  debug_level == 0: print errors, == 1: print warnings, == 2: print info 
        // 'log_warn:8' if coder.target('MATLAB')
        // 'log_warn:12' elseif ~coder.target('MEX')
        // 'log_warn:13' coder.cinclude('<vio_logging.h>')
        // 'log_warn:14' if debug_level >= 2
        // 'log_warn:15' coder.ceval('LOG_WARN', [str, 0], varargin{:});
        for (i9 = 0; i9 < 52; i9++) {
          cv8[i9] = cv9[i9];
        }

        LOG_WARN(cv8, r_d_sq, r_u_sq);
      }

      // 'undistortPoint:80' coeff = (1 + k1*r_u_sq + k2*r_u_sq^2 + k3*r_u_sq^3); 
      coeff = ((1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * powf(r_u_sq,
        3.0F);

      // 'undistortPoint:82' pt_u_n = pt_d_n / coeff;
      for (i9 = 0; i9 < 2; i9++) {
        pt_d_n[i9] /= coeff;
      }

      // 'undistortPoint:84' pt_u((i-1)*2 + (1:2)) = [pt_u_n(1)*fx + Cx; pt_u_n(2)*fy + Cy]; 
      rankR = (i << 1) - 1;
      pt_u_data[rankR + 1] = pt_d_n[0] * fx + Cx;
      pt_u_data[rankR + 2] = pt_d_n[1] * fy + Cy;
    }
  }
}

//
// Arguments    : int n
//                float a
//                int ix0
//                float y[36]
//                int iy0
// Return Type  : void
//
static void xaxpy(int n, float a, int ix0, float y[36], int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0F) {
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
//                const float x[36]
//                int ix0
//                const float y[36]
//                int iy0
// Return Type  : float
//
static float xdotc(int n, const float x[36], int ix0, const float y[36], int iy0)
{
  float d;
  int ix;
  int iy;
  int k;
  d = 0.0F;
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
// Arguments    : emxArray_real32_T *A
//                emxArray_real32_T *tau
//                emxArray_int32_T *jpvt
// Return Type  : void
//
static void xgeqp3(emxArray_real32_T *A, emxArray_real32_T *tau,
                   emxArray_int32_T *jpvt)
{
  int b_m;
  int n;
  int mn;
  int i41;
  emxArray_real32_T *work;
  int itemp;
  emxArray_real32_T *vn1;
  emxArray_real32_T *vn2;
  int k;
  int nmi;
  int i;
  int i_i;
  int mmi;
  int ix;
  float smax;
  float s;
  int pvt;
  int iy;
  float absxk;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  float t;
  b_m = A->size[0];
  n = A->size[1];
  if (A->size[0] <= A->size[1]) {
    mn = A->size[0];
  } else {
    mn = A->size[1];
  }

  i41 = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity((emxArray__common *)tau, i41, (int)sizeof(float));
  eml_signed_integer_colon(A->size[1], jpvt);
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
  } else {
    emxInit_real32_T(&work, 1);
    itemp = A->size[1];
    i41 = work->size[0];
    work->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)work, i41, (int)sizeof(float));
    for (i41 = 0; i41 < itemp; i41++) {
      work->data[i41] = 0.0F;
    }

    emxInit_real32_T(&vn1, 1);
    emxInit_real32_T(&vn2, 1);
    itemp = A->size[1];
    i41 = vn1->size[0];
    vn1->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)vn1, i41, (int)sizeof(float));
    i41 = vn2->size[0];
    vn2->size[0] = vn1->size[0];
    emxEnsureCapacity((emxArray__common *)vn2, i41, (int)sizeof(float));
    k = 1;
    for (nmi = 0; nmi + 1 <= n; nmi++) {
      vn1->data[nmi] = b_xnrm2(b_m, A, k);
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
          smax = fabsf(vn1->data[i]);
          for (k = 0; k + 2 <= nmi; k++) {
            ix++;
            s = fabsf(vn1->data[ix]);
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
        s = 0.0F;
        if (1 + mmi <= 0) {
        } else {
          smax = c_xnrm2(mmi, A, i_i + 2);
          if (smax != 0.0F) {
            smax = hypotf(A->data[i_i], smax);
            if (A->data[i_i] >= 0.0F) {
              smax = -smax;
            }

            if (fabsf(smax) < 9.86076132E-32F) {
              pvt = 0;
              do {
                pvt++;
                xscal(mmi, 1.01412048E+31F, A, i_i + 2);
                smax *= 1.01412048E+31F;
                absxk *= 1.01412048E+31F;
              } while (!(fabsf(smax) >= 9.86076132E-32F));

              smax = c_xnrm2(mmi, A, i_i + 2);
              smax = hypotf(absxk, smax);
              if (absxk >= 0.0F) {
                smax = -smax;
              }

              s = (smax - absxk) / smax;
              xscal(mmi, 1.0F / (absxk - smax), A, i_i + 2);
              for (k = 1; k <= pvt; k++) {
                smax *= 9.86076132E-32F;
              }

              absxk = smax;
            } else {
              s = (smax - A->data[i_i]) / smax;
              xscal(mmi, 1.0F / (A->data[i_i] - smax), A, i_i + 2);
              absxk = smax;
            }
          }
        }

        tau->data[i] = s;
        A->data[i_i] = absxk;
      } else {
        tau->data[i] = 0.0F;
      }

      if (i + 1 < n) {
        absxk = A->data[i_i];
        A->data[i_i] = 1.0F;
        i_ip1 = (i + (i + 1) * b_m) + 1;
        if (tau->data[i] != 0.0F) {
          lastv = mmi;
          itemp = i_i + mmi;
          while ((lastv + 1 > 0) && (A->data[itemp] == 0.0F)) {
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
                if (A->data[k - 1] != 0.0F) {
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
              work->data[iy - 1] = 0.0F;
            }

            iy = 0;
            i41 = i_ip1 + b_m * (lastc - 1);
            itemp = i_ip1;
            while ((b_m > 0) && (itemp <= i41)) {
              ix = i_i;
              smax = 0.0F;
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

          if (-tau->data[i] == 0.0F) {
          } else {
            itemp = 0;
            for (nmi = 1; nmi <= lastc; nmi++) {
              if (work->data[itemp] != 0.0F) {
                smax = work->data[itemp] * -tau->data[i];
                ix = i_i;
                i41 = lastv + i_ip1;
                for (pvt = i_ip1; pvt <= i41; pvt++) {
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
        if (vn1->data[nmi] != 0.0F) {
          smax = fabsf(A->data[i + A->size[0] * nmi]) / vn1->data[nmi];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1->data[nmi] / vn2->data[nmi];
          s = smax * (s * s);
          if (s <= 0.000345266977F) {
            if (i + 1 < b_m) {
              smax = 0.0F;
              if (mmi < 1) {
              } else if (mmi == 1) {
                smax = fabsf(A->data[itemp]);
              } else {
                s = 1.17549435E-38F;
                pvt = itemp + mmi;
                while (itemp + 1 <= pvt) {
                  absxk = fabsf(A->data[itemp]);
                  if (absxk > s) {
                    t = s / absxk;
                    smax = 1.0F + smax * t * t;
                    s = absxk;
                  } else {
                    t = absxk / s;
                    smax += t * t;
                  }

                  itemp++;
                }

                smax = s * sqrtf(smax);
              }

              vn1->data[nmi] = smax;
              vn2->data[nmi] = vn1->data[nmi];
            } else {
              vn1->data[nmi] = 0.0F;
              vn2->data[nmi] = 0.0F;
            }
          } else {
            vn1->data[nmi] *= sqrtf(smax);
          }
        }
      }
    }

    emxFree_real32_T(&vn2);
    emxFree_real32_T(&vn1);
    emxFree_real32_T(&work);
  }
}

//
// Arguments    : int b_m
//                int n
//                emxArray_real32_T *A
//                int lda
//                emxArray_int32_T *ipiv
//                int *info
// Return Type  : void
//
static void xgetrf(int b_m, int n, emxArray_real32_T *A, int lda,
                   emxArray_int32_T *ipiv, int *info)
{
  int c_m;
  int b_info;
  int i39;
  int j;
  int mmj;
  int c;
  int iy;
  int ix;
  float smax;
  int jA;
  float s;
  int i40;
  int jy;
  int b_j;
  int ijA;
  if (b_m <= n) {
    c_m = b_m;
  } else {
    c_m = n;
  }

  eml_signed_integer_colon(c_m, ipiv);
  b_info = 0;
  if ((b_m < 1) || (n < 1)) {
  } else {
    if (b_m - 1 <= n) {
      i39 = b_m - 1;
    } else {
      i39 = n;
    }

    for (j = 0; j + 1 <= i39; j++) {
      mmj = b_m - j;
      c = j * (lda + 1);
      if (mmj < 1) {
        iy = -1;
      } else {
        iy = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabsf(A->data[c]);
          for (jA = 1; jA + 1 <= mmj; jA++) {
            ix++;
            s = fabsf(A->data[ix]);
            if (s > smax) {
              iy = jA;
              smax = s;
            }
          }
        }
      }

      if (A->data[c + iy] != 0.0F) {
        if (iy != 0) {
          ipiv->data[j] = (j + iy) + 1;
          ix = j;
          iy += j;
          for (jA = 1; jA <= n; jA++) {
            smax = A->data[ix];
            A->data[ix] = A->data[iy];
            A->data[iy] = smax;
            ix += lda;
            iy += lda;
          }
        }

        i40 = c + mmj;
        for (iy = c + 1; iy + 1 <= i40; iy++) {
          A->data[iy] /= A->data[c];
        }
      } else {
        b_info = j + 1;
      }

      iy = (n - j) - 1;
      jA = c + lda;
      jy = c + lda;
      for (b_j = 1; b_j <= iy; b_j++) {
        smax = A->data[jy];
        if (A->data[jy] != 0.0F) {
          ix = c + 1;
          i40 = mmj + jA;
          for (ijA = 1 + jA; ijA + 1 <= i40; ijA++) {
            A->data[ijA] += A->data[ix] * -smax;
            ix++;
          }
        }

        jy += lda;
        jA += lda;
      }
    }

    if ((b_info == 0) && (b_m <= n) && (!(A->data[(b_m + A->size[0] * (b_m - 1))
          - 1] != 0.0F))) {
      b_info = b_m;
    }
  }

  *info = b_info;
}

//
// Arguments    : const float x[3]
// Return Type  : float
//
static float xnrm2(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 2; k++) {
    absxk = fabsf(x[k + 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
}

//
// Arguments    : float *a
//                float *b
//                float *c
//                float *s
// Return Type  : void
//
static void xrotg(float *a, float *b, float *c, float *s)
{
  float roe;
  float absa;
  float absb;
  float scale;
  float ads;
  float bds;
  roe = *b;
  absa = fabsf(*a);
  absb = fabsf(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    *s = 0.0F;
    *c = 1.0F;
    scale = 0.0F;
    *b = 0.0F;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrtf(ads * ads + bds * bds);
    if (roe < 0.0F) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0F) {
      *b = 1.0F / *c;
    } else {
      *b = 1.0F;
    }
  }

  *a = scale;
}

//
// Arguments    : int n
//                float a
//                emxArray_real32_T *x
//                int ix0
// Return Type  : void
//
static void xscal(int n, float a, emxArray_real32_T *x, int ix0)
{
  int i42;
  int k;
  i42 = (ix0 + n) - 1;
  for (k = ix0; k <= i42; k++) {
    x->data[k - 1] *= a;
  }
}

//
// input
//  coder.cstructname(measurements, 'VIOMeasurements');
//  coder.cstructname(noiseParameters, 'NoiseParameters');
//  coder.cstructname(noiseParameters.process_noise, 'ProcessNoise');
//  coder.cstructname(cameraParameters, 'DUOParameters');
//  coder.cstructname(cameraParameters.CameraParameters1, 'CameraParameters');
//  coder.cstructname(cameraParameters.CameraParameters2, 'CameraParameters');
//  coder.cstructname(VIOParameters, 'VIOParameters');
// Arguments    : int updateVect[48]
//                const float z_all_l[96]
//                const float z_all_r[96]
//                float dt
//                const VIOMeasurements *measurements
//                const DUOParameters *cameraParameters
//                const NoiseParameters *noiseParameters
//                const VIOParameters *b_VIOParameters
//                boolean_T vision
//                boolean_T reset
//                RobotState *xt_out
//                float map_out[144]
//                AnchorPose anchor_poses_out[6]
//                float delayedStatus_out[48]
// Return Type  : void
//
void SLAM(int updateVect[48], const float z_all_l[96], const float z_all_r[96],
          float dt, const VIOMeasurements *measurements, const DUOParameters
          *cameraParameters, const NoiseParameters *noiseParameters, const
          VIOParameters *b_VIOParameters, boolean_T vision, boolean_T reset,
          RobotState *xt_out, float map_out[144], AnchorPose anchor_poses_out[6],
          float delayedStatus_out[48])
{
  int ixstart;
  float varargin_1[4];
  float mtmp;
  int itmp;
  int ix;
  boolean_T exitg2;
  static const signed char iv2[4] = { 0, 0, 0, 1 };

  static const f_struct_T rv0[6] = { { { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F,
        1.0F }, { 0, 0, 0, 0, 0, 0 }, { { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F,
            0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F,
            0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0,
          0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, {
          0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 } } }, { { 0.0F, 0.0F,
        0.0F }, { 0.0F, 0.0F, 0.0F, 1.0F }, { 0, 0, 0, 0, 0, 0 }, { { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F,
            0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F
          }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, {
            0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F,
            0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F,
            0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 } } }, { { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F, 1.0F },
        { 0, 0, 0, 0, 0, 0 }, { { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0,
          0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, {
          0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F,
            0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F
          }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 } } }, { { 0.0F, 0.0F, 0.0F }, { 0.0F,
        0.0F, 0.0F, 1.0F }, { 0, 0, 0, 0, 0, 0 }, { { 0.0F, { 0.0F, 0.0F, 0.0F },
          { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F,
            0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F,
            0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0,
          0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, {
          0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 } } }, { { 0.0F,
        0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F, 1.0F }, { 0, 0, 0, 0, 0, 0 }, { { 0.0F,
          { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F,
            0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F
          }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, {
            0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F,
            0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F,
            0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 } } }, { { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F, 1.0F },
        { 0, 0, 0, 0, 0, 0 }, { { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0,
          0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, {
          0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F,
            0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F
          }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 } } } };

  int anchorIdx;
  float t_ci[3];
  float w_imu[3];
  static const float fv15[3] = { 0.0F, 0.0F, 1.0F };

  float w_c[3];
  float a_c[3];
  __attribute__((aligned(16))) float fv16[9];
  float R_ci[9];
  float R_cw[9];
  boolean_T exitg1;
  __attribute__((aligned(16))) float b_R_cw[9];
  __attribute__((aligned(16))) float fv17[9];
  static const signed char iv3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  float b_R_ci[9];
  float R[9];
  __attribute__((aligned(16))) float t0_pos[3];
  __attribute__((aligned(16))) float t0_vel[3];
  float b_R[9];
  __attribute__((aligned(16))) float t0_IMU_gyro_bias[3];
  float v[15];
  float d[225];
  __attribute__((aligned(16))) float grav_origin[3];
  float c[3];
  static const float b_b[3] = { 0.0F, 0.0F, 9.81F };

  float G[270];
  static const signed char iv4[45] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0 };

  static const signed char iv5[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv6[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  __attribute__((aligned(16))) float fv18[324];
  static const signed char iv7[324] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  float fv19[9];
  float fv20[9];
  float fv21[9];
  float fv22[9];
  float fv23[9];
  __attribute__((aligned(16))) float fv24[324];
  static const signed char iv8[54] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv9[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  __attribute__((aligned(16))) float Phi[324];
  __attribute__((aligned(16))) float b_Phi[324];
  float b_G[270];
  float c_G[324];
  __attribute__((aligned(16))) float P_xx_apr[324];
  float P_xs_apr[1512];
  float theta;
  float dq[4];
  float b_z_all_l[96];
  float b_z_all_r[96];
  float t0_IMU_att[4];
  float t0_IMU_acc_bias[3];
  struct_T rv1[6];

  // 'SLAM:11' coder.cstructname(measurements, 'VIOMeasurements', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:12' coder.cstructname(noiseParameters, 'NoiseParameters', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:13' coder.cstructname(noiseParameters.process_noise, 'ProcessNoise', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:14' coder.cstructname(cameraParameters, 'DUOParameters', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:15' coder.cstructname(cameraParameters.CameraParameters1, 'CameraParameters', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:16' coder.cstructname(cameraParameters.CameraParameters2, 'CameraParameters', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:17' coder.cstructname(VIOParameters, 'VIOParameters', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:19' assert ( all ( size (updateVect) == [numTrackFeatures 1] ) );
  // 'SLAM:19' assert(isa(updateVect,'int32'));
  // 'SLAM:20' assert ( all ( size (z_all_l) == [numTrackFeatures*2 1] ) )
  // 'SLAM:21' assert ( all ( size (z_all_r) == [numTrackFeatures*2 1] ) )
  // 'SLAM:22' assert ( all ( size (dt) == [1] ) )
  // 'SLAM:23' assert(isa(vision,'logical'));
  // 'SLAM:24' assert(isa(reset,'logical'));
  // 'SLAM:28' if isempty(initialized) || reset
  if ((!initialized_not_empty) || reset) {
    // 'SLAM:29' initialized = [];
    initialized.size[0] = 0;
    initialized.size[1] = 0;
    initialized_not_empty = false;

    //  make sure it is also empty when resetting
    // 'SLAM:31' updateVect(:) = 0;
    for (ixstart = 0; ixstart < 48; ixstart++) {
      updateVect[ixstart] = 0;
    }

    // 'SLAM:33' xt.robot_state.IMU.pos = cameraParameters.t_ci;
    for (ixstart = 0; ixstart < 3; ixstart++) {
      xt.robot_state.IMU.pos[ixstart] = cameraParameters->t_ci[ixstart];
    }

    // 'SLAM:34' xt.robot_state.IMU.att = QuatFromRotJ(cameraParameters.R_ci);
    //  THIS IS OK, It is according to the NASA memo found
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
    // % speed optimization
    // 'QuatFromRotJ:50' [~, index] = max([1+R(1,1)-(R(2,2)+R(3,3)), 1+R(2,2)-(R(1,1)+R(3,3)), 1+R(3,3)-(R(1,1)+R(2,2)), 1+(R(1,1)+R(2,2)+R(3,3))]); 
    varargin_1[0] = (1.0F + cameraParameters->R_ci[0]) - (cameraParameters->
      R_ci[4] + cameraParameters->R_ci[8]);
    varargin_1[1] = (1.0F + cameraParameters->R_ci[4]) - (cameraParameters->
      R_ci[0] + cameraParameters->R_ci[8]);
    varargin_1[2] = (1.0F + cameraParameters->R_ci[8]) - (cameraParameters->
      R_ci[0] + cameraParameters->R_ci[4]);
    varargin_1[3] = 1.0F + ((cameraParameters->R_ci[0] + cameraParameters->R_ci
      [4]) + cameraParameters->R_ci[8]);
    ixstart = 1;
    mtmp = varargin_1[0];
    itmp = 1;
    if (rtIsNaNF(varargin_1[0])) {
      ix = 2;
      exitg2 = false;
      while ((!exitg2) && (ix < 5)) {
        ixstart = ix;
        if (!rtIsNaNF(varargin_1[ix - 1])) {
          mtmp = varargin_1[ix - 1];
          itmp = ix;
          exitg2 = true;
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

    // 'QuatFromRotJ:50' ~
    // 'QuatFromRotJ:52' if(index==1)
    if (itmp == 1) {
      // 'QuatFromRotJ:53' Q = [sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2;
      // 'QuatFromRotJ:54'         (R(1,2)+R(2,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:55'         (R(1,3)+R(3,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:56'         (R(2,3)-R(3,2))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
      xt.robot_state.IMU.att[0] = sqrtf((1.0F + 2.0F * cameraParameters->R_ci[0])
        - ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
           cameraParameters->R_ci[8])) / 2.0F;
      xt.robot_state.IMU.att[1] = (cameraParameters->R_ci[3] +
        cameraParameters->R_ci[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[0]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[2] = (cameraParameters->R_ci[6] +
        cameraParameters->R_ci[2]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[0]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[3] = (cameraParameters->R_ci[7] -
        cameraParameters->R_ci[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[0]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
    } else if (itmp == 2) {
      // 'QuatFromRotJ:57' elseif(index==2)
      // 'QuatFromRotJ:58' Q = [(R(1,2)+R(2,1))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:59'         (sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2);
      // 'QuatFromRotJ:60'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:61'         (R(3,1)-R(1,3))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
      xt.robot_state.IMU.att[0] = (cameraParameters->R_ci[3] +
        cameraParameters->R_ci[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[4]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[1] = sqrtf((1.0F + 2.0F * cameraParameters->R_ci[4])
        - ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
           cameraParameters->R_ci[8])) / 2.0F;
      xt.robot_state.IMU.att[2] = (cameraParameters->R_ci[7] +
        cameraParameters->R_ci[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[4]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[3] = (cameraParameters->R_ci[2] -
        cameraParameters->R_ci[6]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[4]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
    } else if (itmp == 3) {
      // 'QuatFromRotJ:62' elseif(index==3)
      // 'QuatFromRotJ:63' Q = [(R(1,3)+R(3,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:64'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:65'         (sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2);
      // 'QuatFromRotJ:66'         (R(1,2)-R(2,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
      xt.robot_state.IMU.att[0] = (cameraParameters->R_ci[6] +
        cameraParameters->R_ci[2]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[8]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[1] = (cameraParameters->R_ci[7] +
        cameraParameters->R_ci[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[8]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[2] = sqrtf((1.0F + 2.0F * cameraParameters->R_ci[8])
        - ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
           cameraParameters->R_ci[8])) / 2.0F;
      xt.robot_state.IMU.att[3] = (cameraParameters->R_ci[3] -
        cameraParameters->R_ci[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
        cameraParameters->R_ci[8]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F));
    } else {
      // 'QuatFromRotJ:67' else
      // 'QuatFromRotJ:68' Q = [(R(2,3)-R(3,2))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:69'         (R(3,1)-R(1,3))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:70'         (R(1,2)-R(2,1))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
      // 'QuatFromRotJ:71'         (sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2);];
      xt.robot_state.IMU.att[0] = (cameraParameters->R_ci[7] -
        cameraParameters->R_ci[5]) / (4.0F * (sqrtf(1.0F +
        ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
         cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[1] = (cameraParameters->R_ci[2] -
        cameraParameters->R_ci[6]) / (4.0F * (sqrtf(1.0F +
        ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
         cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[2] = (cameraParameters->R_ci[3] -
        cameraParameters->R_ci[1]) / (4.0F * (sqrtf(1.0F +
        ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
         cameraParameters->R_ci[8])) / 2.0F));
      xt.robot_state.IMU.att[3] = sqrtf(1.0F + ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8])) / 2.0F;
    }

    // 'SLAM:36' xt.robot_state.IMU.gyro_bias = cameraParameters.gyro_bias;
    // 'SLAM:37' xt.robot_state.IMU.acc_bias = cameraParameters.acc_bias;
    // 'SLAM:39' xt.robot_state.pos = [0; 0; 0];
    for (ixstart = 0; ixstart < 3; ixstart++) {
      xt.robot_state.IMU.gyro_bias[ixstart] = cameraParameters->
        gyro_bias[ixstart];
      xt.robot_state.IMU.acc_bias[ixstart] = cameraParameters->acc_bias[ixstart];
      xt.robot_state.pos[ixstart] = 0.0F;
    }

    //  position relative to the origin frame
    // 'SLAM:40' xt.robot_state.att = [0; 0; 0; 1];
    for (ixstart = 0; ixstart < 4; ixstart++) {
      xt.robot_state.att[ixstart] = iv2[ixstart];
    }

    //  orientation relative to the origin frame
    // 'SLAM:41' xt.robot_state.vel = [0; 0; 0];
    //  velocity in the origin frame
    // 'SLAM:42' xt.fixed_feature = int32(0);
    xt.fixed_feature = 0;

    // 'SLAM:43' xt.origin.anchor_idx = int32(0);
    xt.origin.anchor_idx = 0;

    //  idx of the anchor that is at the origin
    // 'SLAM:44' xt.origin.pos = [0; 0; 0];
    for (ixstart = 0; ixstart < 3; ixstart++) {
      xt.robot_state.vel[ixstart] = 0.0F;
      xt.origin.pos[ixstart] = 0.0F;
    }

    //  position of the origin in the world frame
    // 'SLAM:45' xt.origin.att = [0; 0; 0; 1];
    for (ixstart = 0; ixstart < 4; ixstart++) {
      xt.origin.att[ixstart] = iv2[ixstart];
    }

    //  orientation of the origin in the world frame
    // 'SLAM:47' P = zeros(numStates + numAnchors*(6+numPointsPerAnchor));
    memset(&P[0], 0, 10404U * sizeof(float));

    //  initial error state covariance
    // 'SLAM:49' anchor_state.pos = [0; 0; 0];
    // 'SLAM:50' anchor_state.att = [0; 0; 0; 1];
    // 'SLAM:51' anchor_state.P_idx = int32(zeros(1, 6));
    // 'SLAM:53' feature_state.inverse_depth = 0;
    // 'SLAM:54' feature_state.m = zeros(3,1);
    // 'SLAM:55' feature_state.scaled_map_point = zeros(3,1);
    // 'SLAM:56' feature_state.status = int32(0);
    // 'SLAM:57' feature_state.status_idx = int32(0);
    // 'SLAM:58' feature_state.P_idx = int32(0);
    // 'SLAM:60' anchor_state.feature_states = repmat(feature_state, numPointsPerAnchor,1); 
    // 'SLAM:62' xt.anchor_states = repmat(anchor_state, numAnchors, 1);
    // 'SLAM:64' for anchorIdx = 1:numAnchors
    memcpy(&xt.anchor_states[0], &rv0[0], 6U * sizeof(f_struct_T));
    for (anchorIdx = 0; anchorIdx < 6; anchorIdx++) {
      // 'SLAM:65' xt.anchor_states(anchorIdx).P_idx = numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6); 
      ixstart = anchorIdx * 14 + 18;
      for (ix = 0; ix < 6; ix++) {
        xt.anchor_states[anchorIdx].P_idx[ix] = (ix + ixstart) + 1;
      }
    }

    // 'SLAM:68' if vision
    if (vision) {
      // 'SLAM:69' map = zeros(numTrackFeatures*3, 1);
      memset(&map[0], 0, 144U * sizeof(float));

      // 'SLAM:70' delayedStatus = zeros(numTrackFeatures, 1);
      memset(&delayedStatus[0], 0, 48U * sizeof(float));
    } else {
      // 'SLAM:71' else
      // 'SLAM:72' z_b = measurements.acc - xt.robot_state.IMU.acc_bias;
      for (ixstart = 0; ixstart < 3; ixstart++) {
        t_ci[ixstart] = measurements->acc[ixstart] -
          xt.robot_state.IMU.acc_bias[ixstart];
      }

      // 'SLAM:73' z_n_b = z_b/norm(z_b);
      mtmp = norm(t_ci);
      for (ix = 0; ix < 3; ix++) {
        t_ci[ix] /= mtmp;
      }

      // 'SLAM:74' m_n_b = [0;0;1];
      // 'SLAM:75' y_n_b = cross(z_n_b,m_n_b);
      cross(t_ci, fv15, w_imu);

      // 'SLAM:76' y_n_b = y_n_b./norm(y_n_b);
      rdivide(w_imu, norm(w_imu), w_c);

      // 'SLAM:77' x_n_b = (cross(y_n_b,z_n_b));
      cross(w_c, t_ci, w_imu);

      // 'SLAM:78' x_n_b = x_n_b./norm(x_n_b);
      // 'SLAM:80' R_iw_init = [x_n_b,y_n_b,z_n_b];
      // 'SLAM:81' R_cw_init = RotFromQuatJ(xt.robot_state.IMU.att) * R_iw_init; 
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
      // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
      // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
      rdivide(w_imu, norm(w_imu), a_c);
      fv16[0] = ((xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0] -
                  xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                 xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      fv16[3] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] +
                        xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      fv16[6] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] -
                        xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      fv16[1] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] -
                        xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      fv16[4] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) +
                  xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                 xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      fv16[7] = 2.0F * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] +
                        xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      fv16[2] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] +
                        xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      fv16[5] = 2.0F * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] -
                        xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      fv16[8] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) -
                  xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) +
                 xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      for (ix = 0; ix < 3; ix++) {
        R_ci[ix] = a_c[ix];
        R_ci[3 + ix] = w_c[ix];
        R_ci[6 + ix] = t_ci[ix];
      }

      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          R_cw[ix + 3 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            R_cw[ix + 3 * itmp] += fv16[ix + 3 * ixstart] * R_ci[ixstart + 3 *
              itmp];
          }
        }
      }

      // 'SLAM:83' xt.origin.att = QuatFromRotJ(R_cw_init);
      //  THIS IS OK, It is according to the NASA memo found
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
      // % speed optimization
      // 'QuatFromRotJ:50' [~, index] = max([1+R(1,1)-(R(2,2)+R(3,3)), 1+R(2,2)-(R(1,1)+R(3,3)), 1+R(3,3)-(R(1,1)+R(2,2)), 1+(R(1,1)+R(2,2)+R(3,3))]); 
      varargin_1[0] = (1.0F + R_cw[0]) - (R_cw[4] + R_cw[8]);
      varargin_1[1] = (1.0F + R_cw[4]) - (R_cw[0] + R_cw[8]);
      varargin_1[2] = (1.0F + R_cw[8]) - (R_cw[0] + R_cw[4]);
      varargin_1[3] = 1.0F + ((R_cw[0] + R_cw[4]) + R_cw[8]);
      ixstart = 1;
      mtmp = varargin_1[0];
      itmp = 1;
      if (rtIsNaNF(varargin_1[0])) {
        ix = 2;
        exitg1 = false;
        while ((!exitg1) && (ix < 5)) {
          ixstart = ix;
          if (!rtIsNaNF(varargin_1[ix - 1])) {
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

      // 'QuatFromRotJ:50' ~
      // 'QuatFromRotJ:52' if(index==1)
      if (itmp == 1) {
        // 'QuatFromRotJ:53' Q = [sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2;
        // 'QuatFromRotJ:54'         (R(1,2)+R(2,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:55'         (R(1,3)+R(3,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:56'         (R(2,3)-R(3,2))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
        xt.origin.att[0] = sqrtf((1.0F + 2.0F * R_cw[0]) - ((R_cw[0] + R_cw[4])
          + R_cw[8])) / 2.0F;
        xt.origin.att[1] = (R_cw[3] + R_cw[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[0]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[2] = (R_cw[6] + R_cw[2]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[0]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[3] = (R_cw[7] - R_cw[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[0]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
      } else if (itmp == 2) {
        // 'QuatFromRotJ:57' elseif(index==2)
        // 'QuatFromRotJ:58' Q = [(R(1,2)+R(2,1))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:59'         (sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2); 
        // 'QuatFromRotJ:60'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:61'         (R(3,1)-R(1,3))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
        xt.origin.att[0] = (R_cw[3] + R_cw[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[4]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[1] = sqrtf((1.0F + 2.0F * R_cw[4]) - ((R_cw[0] + R_cw[4])
          + R_cw[8])) / 2.0F;
        xt.origin.att[2] = (R_cw[7] + R_cw[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[4]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[3] = (R_cw[2] - R_cw[6]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[4]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
      } else if (itmp == 3) {
        // 'QuatFromRotJ:62' elseif(index==3)
        // 'QuatFromRotJ:63' Q = [(R(1,3)+R(3,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:64'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:65'         (sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2); 
        // 'QuatFromRotJ:66'         (R(1,2)-R(2,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
        xt.origin.att[0] = (R_cw[6] + R_cw[2]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[8]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[1] = (R_cw[7] + R_cw[5]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[8]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[2] = sqrtf((1.0F + 2.0F * R_cw[8]) - ((R_cw[0] + R_cw[4])
          + R_cw[8])) / 2.0F;
        xt.origin.att[3] = (R_cw[3] - R_cw[1]) / (4.0F * (sqrtf((1.0F + 2.0F *
          R_cw[8]) - ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F));
      } else {
        // 'QuatFromRotJ:67' else
        // 'QuatFromRotJ:68' Q = [(R(2,3)-R(3,2))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:69'         (R(3,1)-R(1,3))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:70'         (R(1,2)-R(2,1))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
        // 'QuatFromRotJ:71'         (sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2);];
        xt.origin.att[0] = (R_cw[7] - R_cw[5]) / (4.0F * (sqrtf(1.0F + ((R_cw[0]
          + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[1] = (R_cw[2] - R_cw[6]) / (4.0F * (sqrtf(1.0F + ((R_cw[0]
          + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[2] = (R_cw[3] - R_cw[1]) / (4.0F * (sqrtf(1.0F + ((R_cw[0]
          + R_cw[4]) + R_cw[8])) / 2.0F));
        xt.origin.att[3] = sqrtf(1.0F + ((R_cw[0] + R_cw[4]) + R_cw[8])) / 2.0F;
      }

      //  orientation of the origin in the world frame
      // 'SLAM:85' P(  1:3,   1:3) = zeros(3);
      //  position
      // 'SLAM:86' P(  4:6,   4:6) = zeros(3);
      //  orientation of camera in origin frame
      // 'SLAM:87' P(  7:9,   7:9) = 1*eye(3);
      //  velocity
      // 'SLAM:88' P(10:12, 10:12) = diag(noiseParameters.gyro_bias_initial_unc); 
      diag(noiseParameters->gyro_bias_initial_unc, fv16);

      //  gyro bias
      // 'SLAM:89' P(13:15, 13:15) = diag(noiseParameters.acc_bias_initial_unc); 
      diag(noiseParameters->acc_bias_initial_unc, b_R_cw);

      //  acc bias
      // 'SLAM:90' P(16:18, 16:18) = 0.1*R_cw_init * diag([1 1 0]) * R_cw_init'; 
      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          P[itmp + 102 * ix] = 0.0F;
          P[(itmp + 102 * (3 + ix)) + 3] = 0.0F;
          P[(itmp + 102 * (6 + ix)) + 6] = iv3[itmp + 3 * ix];
          P[(itmp + 102 * (9 + ix)) + 9] = fv16[itmp + 3 * ix];
          P[(itmp + 102 * (12 + ix)) + 12] = b_R_cw[itmp + 3 * ix];
          fv17[ix + 3 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            fv17[ix + 3 * itmp] += 0.1F * R_cw[ix + 3 * ixstart] * (float)
              b[ixstart + 3 * itmp];
          }
        }
      }

      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          P[(ix + 102 * (15 + itmp)) + 15] = 0.0F;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            P[(ix + 102 * (15 + itmp)) + 15] += fv17[ix + 3 * ixstart] *
              R_cw[itmp + 3 * ixstart];
          }
        }
      }

      //  origin orientation
      //          P(19:21, 19:21) = 0*0.01*eye(3); % R_ci
      // 'SLAM:93' map = getMap(xt);
      getMap(xt.origin.pos, xt.origin.att, xt.anchor_states, map);

      // 'SLAM:94' delayedStatus = zeros(size(updateVect));
      memset(&delayedStatus[0], 0, 48U * sizeof(float));

      // 'SLAM:96' printParams(noiseParameters, VIOParameters)
      printParams(noiseParameters->process_noise.qv,
                  noiseParameters->process_noise.qw,
                  noiseParameters->process_noise.qao,
                  noiseParameters->process_noise.qwo,
                  noiseParameters->process_noise.qR_ci,
                  noiseParameters->gyro_bias_initial_unc,
                  noiseParameters->acc_bias_initial_unc,
                  noiseParameters->image_noise,
                  noiseParameters->inv_depth_initial_unc,
                  b_VIOParameters->max_ekf_iterations,
                  b_VIOParameters->fixed_feature,
                  b_VIOParameters->delayed_initialization, b_VIOParameters->mono,
                  b_VIOParameters->full_stereo, b_VIOParameters->RANSAC);

      // 'SLAM:97' initialized = 1;
      initialized.size[0] = 1;
      initialized.size[1] = 1;
      initialized.data[0] = 1.0F;
      initialized_not_empty = true;
    }
  } else {
    // 'SLAM:100' else
    // 'SLAM:102' if ~vision
    if (!vision) {
      //      [xt,P] =  SLAM_pred(P, xt, dt, noiseParameters.process_noise, measurements, numStates); 
      // 'SLAM:104' [xt,P] =  SLAM_pred_euler(P, xt, dt, noiseParameters.process_noise, measurements); 
      // 'SLAM_pred_euler:3' R_cw = RotFromQuatJ(x.robot_state.att);
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
      // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
      // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
      R_cw[0] = ((xt.robot_state.att[0] * xt.robot_state.att[0] -
                  xt.robot_state.att[1] * xt.robot_state.att[1]) -
                 xt.robot_state.att[2] * xt.robot_state.att[2]) +
        xt.robot_state.att[3] * xt.robot_state.att[3];
      R_cw[3] = 2.0F * (xt.robot_state.att[0] * xt.robot_state.att[1] +
                        xt.robot_state.att[2] * xt.robot_state.att[3]);
      R_cw[6] = 2.0F * (xt.robot_state.att[0] * xt.robot_state.att[2] -
                        xt.robot_state.att[1] * xt.robot_state.att[3]);
      R_cw[1] = 2.0F * (xt.robot_state.att[0] * xt.robot_state.att[1] -
                        xt.robot_state.att[2] * xt.robot_state.att[3]);
      R_cw[4] = ((-(xt.robot_state.att[0] * xt.robot_state.att[0]) +
                  xt.robot_state.att[1] * xt.robot_state.att[1]) -
                 xt.robot_state.att[2] * xt.robot_state.att[2]) +
        xt.robot_state.att[3] * xt.robot_state.att[3];
      R_cw[7] = 2.0F * (xt.robot_state.att[1] * xt.robot_state.att[2] +
                        xt.robot_state.att[0] * xt.robot_state.att[3]);
      R_cw[2] = 2.0F * (xt.robot_state.att[0] * xt.robot_state.att[2] +
                        xt.robot_state.att[1] * xt.robot_state.att[3]);
      R_cw[5] = 2.0F * (xt.robot_state.att[1] * xt.robot_state.att[2] -
                        xt.robot_state.att[0] * xt.robot_state.att[3]);
      R_cw[8] = ((-(xt.robot_state.att[0] * xt.robot_state.att[0]) -
                  xt.robot_state.att[1] * xt.robot_state.att[1]) +
                 xt.robot_state.att[2] * xt.robot_state.att[2]) +
        xt.robot_state.att[3] * xt.robot_state.att[3];

      //  rotation in origin frame
      // 'SLAM_pred_euler:5' R_ci = RotFromQuatJ(x.robot_state.IMU.att);
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
      // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
      // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
      b_R_ci[0] = ((xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0] -
                    xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                   xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      b_R_ci[3] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1]
                          + xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      b_R_ci[6] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2]
                          - xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      b_R_ci[1] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1]
                          - xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      b_R_ci[4] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) +
                    xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                   xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      b_R_ci[7] = 2.0F * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2]
                          + xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      b_R_ci[2] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2]
                          + xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      b_R_ci[5] = 2.0F * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2]
                          - xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      b_R_ci[8] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) -
                    xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) +
                   xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];

      // 'SLAM_pred_euler:6' t_ci = x.robot_state.IMU.pos;
      //  in camera frame
      // 'SLAM_pred_euler:7' t_ci = -R_ci' * t_ci;
      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          R_ci[itmp + 3 * ix] = -b_R_ci[ix + 3 * itmp];
        }
      }

      //  in imu frame
      // 'SLAM_pred_euler:8' w_imu = measurements.gyr - x.robot_state.IMU.gyro_bias; 
      for (ixstart = 0; ixstart < 3; ixstart++) {
        t_ci[ixstart] = 0.0F;
        for (ix = 0; ix < 3; ix++) {
          t_ci[ixstart] += R_ci[ixstart + 3 * ix] * xt.robot_state.IMU.pos[ix];
        }

        w_imu[ixstart] = measurements->gyr[ixstart] -
          xt.robot_state.IMU.gyro_bias[ixstart];
      }

      //  gyro in IMU frame
      // 'SLAM_pred_euler:9' w_c = R_ci * w_imu;
      //  gyro in camera frame
      //  w = 0*w;
      // 'SLAM_pred_euler:11' a_imu = measurements.acc - x.robot_state.IMU.acc_bias; 
      //  acceleration in IMU frame
      // 'SLAM_pred_euler:12' a_c = R_ci * (a_imu + skew(w_imu)^2 * t_ci);
      // 'skew:2' R=[0,-w(3),w(2);
      // 'skew:3'     w(3),0,-w(1);
      // 'skew:4'     -w(2),w(1),0];
      R[0] = 0.0F;
      R[3] = -w_imu[2];
      R[6] = w_imu[1];
      R[1] = w_imu[2];
      R[4] = 0.0F;
      R[7] = -w_imu[0];
      R[2] = -w_imu[1];
      R[5] = w_imu[0];
      R[8] = 0.0F;
      for (ixstart = 0; ixstart < 3; ixstart++) {
        w_c[ixstart] = 0.0F;
        t0_pos[ixstart] = measurements->acc[ixstart] -
          xt.robot_state.IMU.acc_bias[ixstart];
        t0_vel[ixstart] = 0.0F;
        for (ix = 0; ix < 3; ix++) {
          w_c[ixstart] += b_R_ci[ixstart + 3 * ix] * w_imu[ix];
          b_R[ixstart + 3 * ix] = 0.0F;
          for (itmp = 0; itmp < 3; itmp++) {
            b_R[ixstart + 3 * ix] += R[ixstart + 3 * itmp] * R[itmp + 3 * ix];
          }

          t0_vel[ixstart] += b_R[ixstart + 3 * ix] * t_ci[ix];
        }
      }

      mw_neon_mm_add_f32x4(t0_pos, 3, 1, t0_vel, &t0_IMU_gyro_bias[0]);
      for (ix = 0; ix < 3; ix++) {
        a_c[ix] = 0.0F;
        for (itmp = 0; itmp < 3; itmp++) {
          a_c[ix] += b_R_ci[ix + 3 * itmp] * t0_IMU_gyro_bias[itmp];
        }
      }

      //  a = 0*a;
      // % compute the linearization F of the non linear model f
      // 'SLAM_pred_euler:17' qv    = processNoise.qv;
      // 'SLAM_pred_euler:18' qw    = processNoise.qw;
      // 'SLAM_pred_euler:19' qwo   = processNoise.qwo;
      // 'SLAM_pred_euler:20' qao   = processNoise.qao;
      // 'SLAM_pred_euler:21' qR_ci = processNoise.qR_ci;
      // 'SLAM_pred_euler:24' Q = diag([qw,qw,qw, qv,qv,qv, qwo,qwo,qwo, 0*qao,qao,0*qao, qR_ci,qR_ci,qR_ci]); 
      v[0] = noiseParameters->process_noise.qw;
      v[1] = noiseParameters->process_noise.qw;
      v[2] = noiseParameters->process_noise.qw;
      v[3] = noiseParameters->process_noise.qv;
      v[4] = noiseParameters->process_noise.qv;
      v[5] = noiseParameters->process_noise.qv;
      v[6] = noiseParameters->process_noise.qwo;
      v[7] = noiseParameters->process_noise.qwo;
      v[8] = noiseParameters->process_noise.qwo;
      v[9] = 0.0F * noiseParameters->process_noise.qao;
      v[10] = noiseParameters->process_noise.qao;
      v[11] = 0.0F * noiseParameters->process_noise.qao;
      v[12] = noiseParameters->process_noise.qR_ci;
      v[13] = noiseParameters->process_noise.qR_ci;
      v[14] = noiseParameters->process_noise.qR_ci;
      memset(&d[0], 0, 225U * sizeof(float));
      for (ixstart = 0; ixstart < 15; ixstart++) {
        d[ixstart + 15 * ixstart] = v[ixstart];
      }

      // 'SLAM_pred_euler:26' I = eye(3);
      // 'SLAM_pred_euler:27' O = zeros(3);
      // 'SLAM_pred_euler:29' grav_origin = RotFromQuatJ(x.origin.att) * [0; 0; 9.81]; 
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
      // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
      // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
      fv16[0] = ((xt.origin.att[0] * xt.origin.att[0] - xt.origin.att[1] *
                  xt.origin.att[1]) - xt.origin.att[2] * xt.origin.att[2]) +
        xt.origin.att[3] * xt.origin.att[3];
      fv16[3] = 2.0F * (xt.origin.att[0] * xt.origin.att[1] + xt.origin.att[2] *
                        xt.origin.att[3]);
      fv16[6] = 2.0F * (xt.origin.att[0] * xt.origin.att[2] - xt.origin.att[1] *
                        xt.origin.att[3]);
      fv16[1] = 2.0F * (xt.origin.att[0] * xt.origin.att[1] - xt.origin.att[2] *
                        xt.origin.att[3]);
      fv16[4] = ((-(xt.origin.att[0] * xt.origin.att[0]) + xt.origin.att[1] *
                  xt.origin.att[1]) - xt.origin.att[2] * xt.origin.att[2]) +
        xt.origin.att[3] * xt.origin.att[3];
      fv16[7] = 2.0F * (xt.origin.att[1] * xt.origin.att[2] + xt.origin.att[0] *
                        xt.origin.att[3]);
      fv16[2] = 2.0F * (xt.origin.att[0] * xt.origin.att[2] + xt.origin.att[1] *
                        xt.origin.att[3]);
      fv16[5] = 2.0F * (xt.origin.att[1] * xt.origin.att[2] - xt.origin.att[0] *
                        xt.origin.att[3]);
      fv16[8] = ((-(xt.origin.att[0] * xt.origin.att[0]) - xt.origin.att[1] *
                  xt.origin.att[1]) + xt.origin.att[2] * xt.origin.att[2]) +
        xt.origin.att[3] * xt.origin.att[3];

      //  gravity transformed into the origin frame
      //    pos,            rot, vel,                                         gyr_bias,   acc_bias,          origin_att 
      // 'SLAM_pred_euler:32' F=[ O,                O,   I,                                                O,           O,                  O; 
      // 'SLAM_pred_euler:33'     O,       -skew(w_c),   O,                                               -I,           O,                  O; 
      // 'SLAM_pred_euler:34'     O, -R_cw'*skew(a_c),   O, -skew(skew(w_imu)*t_ci) - skew(w_imu)*skew(t_ci), -R_cw'*R_ci, -skew(grav_origin); 
      // 'SLAM_pred_euler:35'     O,                O,   O,                                                O,           O,                  O; 
      // 'SLAM_pred_euler:36'     O,                O,   O,                                                O,           O,                  O; 
      // 'SLAM_pred_euler:37'     O,                O,   O,                                                O,           O,                  O]; 
      // 'skew:2' R=[0,-w(3),w(2);
      // 'skew:3'     w(3),0,-w(1);
      // 'skew:4'     -w(2),w(1),0];
      // 'skew:2' R=[0,-w(3),w(2);
      // 'skew:3'     w(3),0,-w(1);
      // 'skew:4'     -w(2),w(1),0];
      // 'skew:2' R=[0,-w(3),w(2);
      // 'skew:3'     w(3),0,-w(1);
      // 'skew:4'     -w(2),w(1),0];
      b_R_cw[0] = 0.0F;
      b_R_cw[3] = -w_imu[2];
      b_R_cw[6] = w_imu[1];
      b_R_cw[1] = w_imu[2];
      b_R_cw[4] = 0.0F;
      b_R_cw[7] = -w_imu[0];
      b_R_cw[2] = -w_imu[1];
      b_R_cw[5] = w_imu[0];
      b_R_cw[8] = 0.0F;
      for (ix = 0; ix < 3; ix++) {
        grav_origin[ix] = 0.0F;
        c[ix] = 0.0F;
        for (itmp = 0; itmp < 3; itmp++) {
          grav_origin[ix] += fv16[ix + 3 * itmp] * b_b[itmp];
          c[ix] += b_R_cw[ix + 3 * itmp] * t_ci[itmp];
        }
      }

      // 'skew:2' R=[0,-w(3),w(2);
      // 'skew:3'     w(3),0,-w(1);
      // 'skew:4'     -w(2),w(1),0];
      // 'skew:2' R=[0,-w(3),w(2);
      // 'skew:3'     w(3),0,-w(1);
      // 'skew:4'     -w(2),w(1),0];
      // 'skew:2' R=[0,-w(3),w(2);
      // 'skew:3'     w(3),0,-w(1);
      // 'skew:4'     -w(2),w(1),0];
      // 'skew:2' R=[0,-w(3),w(2);
      // 'skew:3'     w(3),0,-w(1);
      // 'skew:4'     -w(2),w(1),0];
      //  pos
      //  att
      //  vel
      //  gyro bias
      //  acc bias
      // 'SLAM_pred_euler:39' G=[  O,     O, O, O, O;  % pos
      // 'SLAM_pred_euler:40'     -I,     O, O, O, O;  % att
      // 'SLAM_pred_euler:41'      O,-R_cw', O, O, O;  % vel
      // 'SLAM_pred_euler:42'      O,     O, I, O, O;  % gyro bias
      // 'SLAM_pred_euler:43'      O,     O, O, I, O;  % acc bias
      // 'SLAM_pred_euler:44'      O,     O, O, O, O];
      for (ix = 0; ix < 15; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          G[itmp + 18 * ix] = 0.0F;
          G[(itmp + 18 * ix) + 3] = iv4[itmp + 3 * ix];
        }
      }

      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          G[(itmp + 18 * ix) + 6] = 0.0F;
          G[(itmp + 18 * (ix + 3)) + 6] = -R_cw[ix + 3 * itmp];
          G[(itmp + 18 * (ix + 6)) + 6] = 0.0F;
          G[(itmp + 18 * (ix + 9)) + 6] = 0.0F;
          G[(itmp + 18 * (ix + 12)) + 6] = 0.0F;
        }
      }

      for (ix = 0; ix < 15; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          G[(itmp + 18 * ix) + 9] = iv5[itmp + 3 * ix];
          G[(itmp + 18 * ix) + 12] = iv6[itmp + 3 * ix];
          G[(itmp + 18 * ix) + 15] = 0.0F;
        }
      }

      //  origin att
      // 'SLAM_pred_euler:46' Phi = eye(numStates) + F * dt;
      for (ix = 0; ix < 324; ix++) {
        fv18[ix] = iv7[ix];
      }

      fv17[0] = 0.0F;
      fv17[3] = -c[2];
      fv17[6] = c[1];
      fv17[1] = c[2];
      fv17[4] = 0.0F;
      fv17[7] = -c[0];
      fv17[2] = -c[1];
      fv17[5] = c[0];
      fv17[8] = 0.0F;
      fv19[0] = 0.0F;
      fv19[3] = -w_imu[2];
      fv19[6] = w_imu[1];
      fv19[1] = w_imu[2];
      fv19[4] = 0.0F;
      fv19[7] = -w_imu[0];
      fv19[2] = -w_imu[1];
      fv19[5] = w_imu[0];
      fv19[8] = 0.0F;
      fv20[0] = 0.0F;
      fv20[3] = -t_ci[2];
      fv20[6] = t_ci[1];
      fv20[1] = t_ci[2];
      fv20[4] = 0.0F;
      fv20[7] = -t_ci[0];
      fv20[2] = -t_ci[1];
      fv20[5] = t_ci[0];
      fv20[8] = 0.0F;
      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          b_R_cw[itmp + 3 * ix] = -fv17[itmp + 3 * ix];
          fv16[ix + 3 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            fv16[ix + 3 * itmp] += fv19[ix + 3 * ixstart] * fv20[ixstart + 3 *
              itmp];
          }
        }
      }

      mw_neon_mm_sub_f32x4(b_R_cw, 3, 3, fv16, &fv17[0]);
      fv21[0] = 0.0F;
      fv21[3] = -w_c[2];
      fv21[6] = w_c[1];
      fv21[1] = w_c[2];
      fv21[4] = 0.0F;
      fv21[7] = -w_c[0];
      fv21[2] = -w_c[1];
      fv21[5] = w_c[0];
      fv21[8] = 0.0F;
      fv22[0] = 0.0F;
      fv22[3] = -a_c[2];
      fv22[6] = a_c[1];
      fv22[1] = a_c[2];
      fv22[4] = 0.0F;
      fv22[7] = -a_c[0];
      fv22[2] = -a_c[1];
      fv22[5] = a_c[0];
      fv22[8] = 0.0F;
      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          b_R[itmp + 3 * ix] = -R_cw[ix + 3 * itmp];
          R_ci[itmp + 3 * ix] = -R_cw[ix + 3 * itmp];
        }
      }

      fv23[0] = 0.0F;
      fv23[3] = -grav_origin[2];
      fv23[6] = grav_origin[1];
      fv23[1] = grav_origin[2];
      fv23[4] = 0.0F;
      fv23[7] = -grav_origin[0];
      fv23[2] = -grav_origin[1];
      fv23[5] = grav_origin[0];
      fv23[8] = 0.0F;
      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          R[ix + 3 * itmp] = 0.0F;
          b_R_cw[ix + 3 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 3; ixstart++) {
            R[ix + 3 * itmp] += b_R[ix + 3 * ixstart] * fv22[ixstart + 3 * itmp];
            b_R_cw[ix + 3 * itmp] += R_ci[ix + 3 * ixstart] * b_R_ci[ixstart + 3
              * itmp];
          }
        }
      }

      for (ix = 0; ix < 18; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          fv24[itmp + 18 * ix] = iv8[itmp + 3 * ix];
        }
      }

      for (ix = 0; ix < 3; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          fv24[(itmp + 18 * ix) + 3] = 0.0F;
          fv24[(itmp + 18 * (ix + 3)) + 3] = -fv21[itmp + 3 * ix];
          fv24[(itmp + 18 * (ix + 6)) + 3] = 0.0F;
          fv24[(itmp + 18 * (ix + 9)) + 3] = iv9[itmp + 3 * ix];
          fv24[(itmp + 18 * (ix + 12)) + 3] = 0.0F;
          fv24[(itmp + 18 * (ix + 15)) + 3] = 0.0F;
          fv24[(itmp + 18 * ix) + 6] = 0.0F;
          fv24[(itmp + 18 * (ix + 3)) + 6] = R[itmp + 3 * ix];
          fv24[(itmp + 18 * (ix + 6)) + 6] = 0.0F;
          fv24[(itmp + 18 * (ix + 9)) + 6] = fv17[itmp + 3 * ix];
          fv24[(itmp + 18 * (ix + 12)) + 6] = b_R_cw[itmp + 3 * ix];
          fv24[(itmp + 18 * (ix + 15)) + 6] = -fv23[itmp + 3 * ix];
        }
      }

      for (ix = 0; ix < 18; ix++) {
        for (itmp = 0; itmp < 3; itmp++) {
          fv24[(itmp + 18 * ix) + 9] = 0.0F;
          fv24[(itmp + 18 * ix) + 12] = 0.0F;
          fv24[(itmp + 18 * ix) + 15] = 0.0F;
        }

        for (itmp = 0; itmp < 18; itmp++) {
          Phi[itmp + 18 * ix] = fv24[itmp + 18 * ix] * dt;
        }
      }

      mw_neon_mm_add_f32x4(fv18, 18, 18, Phi, &b_Phi[0]);

      // 'SLAM_pred_euler:48' states_idx = int32(1:numStates);
      // 'SLAM_pred_euler:50' P_xx_apr = Phi*P_apo(states_idx, states_idx)*Phi'  + G*Q*G'*dt; 
      for (ix = 0; ix < 18; ix++) {
        for (itmp = 0; itmp < 18; itmp++) {
          Phi[ix + 18 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 18; ixstart++) {
            Phi[ix + 18 * itmp] += b_Phi[ix + 18 * ixstart] * P[ixstart + 102 *
              itmp];
          }
        }

        for (itmp = 0; itmp < 18; itmp++) {
          fv18[ix + 18 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 18; ixstart++) {
            fv18[ix + 18 * itmp] += Phi[ix + 18 * ixstart] * b_Phi[itmp + 18 *
              ixstart];
          }
        }

        for (itmp = 0; itmp < 15; itmp++) {
          b_G[ix + 18 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 15; ixstart++) {
            b_G[ix + 18 * itmp] += G[ix + 18 * ixstart] * d[ixstart + 15 * itmp];
          }
        }

        for (itmp = 0; itmp < 18; itmp++) {
          c_G[ix + 18 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 15; ixstart++) {
            c_G[ix + 18 * itmp] += b_G[ix + 18 * ixstart] * G[itmp + 18 *
              ixstart];
          }
        }
      }

      for (ix = 0; ix < 18; ix++) {
        for (itmp = 0; itmp < 18; itmp++) {
          fv24[itmp + 18 * ix] = c_G[itmp + 18 * ix] * dt;
        }
      }

      mw_neon_mm_add_f32x4(fv18, 18, 18, fv24, &P_xx_apr[0]);

      //  covariance of the state
      // 'SLAM_pred_euler:51' P_xx_apr = (P_xx_apr + P_xx_apr')/2;
      // 'SLAM_pred_euler:52' P_xs_apr = Phi*P_apo(states_idx, int32(numStates+1:int32(end))); 
      //  covariance between current state and trails
      // 'SLAM_pred_euler:54' P_apr = P_apo;
      // 'SLAM_pred_euler:55' P_apr(states_idx, states_idx) = P_xx_apr;
      for (ix = 0; ix < 18; ix++) {
        for (itmp = 0; itmp < 84; itmp++) {
          P_xs_apr[ix + 18 * itmp] = 0.0F;
          for (ixstart = 0; ixstart < 18; ixstart++) {
            P_xs_apr[ix + 18 * itmp] += b_Phi[ix + 18 * ixstart] * P[ixstart +
              102 * (18 + itmp)];
          }
        }

        for (itmp = 0; itmp < 18; itmp++) {
          fv18[itmp + 18 * ix] = P_xx_apr[ix + 18 * itmp];
        }
      }

      mw_neon_mm_add_f32x4(P_xx_apr, 18, 18, fv18, &fv24[0]);
      for (ix = 0; ix < 18; ix++) {
        for (itmp = 0; itmp < 18; itmp++) {
          P[itmp + 102 * ix] = fv24[itmp + 18 * ix] / 2.0F;
        }
      }

      // 'SLAM_pred_euler:56' P_apr(states_idx, int32(numStates+1:int32(end))) = P_xs_apr; 
      for (ix = 0; ix < 84; ix++) {
        memcpy(&P[ix * 102 + 1836], &P_xs_apr[ix * 18], 18U * sizeof(float));
      }

      // 'SLAM_pred_euler:57' P_apr(int32(numStates+1:int32(end)), states_idx) = P_xs_apr'; 
      for (ix = 0; ix < 18; ix++) {
        for (itmp = 0; itmp < 84; itmp++) {
          P[(itmp + 102 * ix) + 18] = P_xs_apr[ix + 18 * itmp];
        }
      }

      // 'SLAM_pred_euler:59' x.robot_state.pos = x.robot_state.pos + x.robot_state.vel*dt; 
      for (ixstart = 0; ixstart < 3; ixstart++) {
        t0_pos[ixstart] = xt.robot_state.vel[ixstart] * dt;
      }

      mw_neon_mm_add_f32x4(xt.robot_state.pos, 3, 1, t0_pos, &t0_vel[0]);

      // 'SLAM_pred_euler:60' dq = quatPlusThetaJ(w_c*dt);
      for (ixstart = 0; ixstart < 3; ixstart++) {
        xt.robot_state.pos[ixstart] = t0_vel[ixstart];
        w_c[ixstart] *= dt;
      }

      // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
      theta = norm(w_c) * 0.5F;

      // 'quatPlusThetaJ:3' if theta < 0.244
      if (theta < 0.244F) {
        // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
        for (ixstart = 0; ixstart < 3; ixstart++) {
          dq[ixstart] = 0.5F * w_c[ixstart];
        }

        dq[3] = 1.0F;
      } else {
        // 'quatPlusThetaJ:5' else
        // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
        // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
        // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
        // 'quatPlusThetaJ:9'          cos(theta)];
        dq[0] = 0.5F * w_c[0] * sinf(theta) / theta;
        dq[1] = 0.5F * w_c[1] * sinf(theta) / theta;
        dq[2] = 0.5F * w_c[2] * sinf(theta) / theta;
        dq[3] = cosf(theta);
      }

      // 'quatPlusThetaJ:11' dq = dq/norm(dq);
      mtmp = b_norm(dq);

      // 'SLAM_pred_euler:61' x.robot_state.att = quatmultJ(dq, x.robot_state.att); 
      for (ixstart = 0; ixstart < 4; ixstart++) {
        varargin_1[ixstart] = xt.robot_state.att[ixstart];
        dq[ixstart] /= mtmp;
      }

      quatmultJ(dq, varargin_1, xt.robot_state.att);

      // 'SLAM_pred_euler:62' x.robot_state.vel = x.robot_state.vel + (R_cw'*a_c - grav_origin)*dt; 
      for (ix = 0; ix < 3; ix++) {
        t0_pos[ix] = 0.0F;
        for (itmp = 0; itmp < 3; itmp++) {
          t0_pos[ix] += R_cw[itmp + 3 * ix] * a_c[itmp];
        }
      }

      mw_neon_mm_sub_f32x4(t0_pos, 3, 1, grav_origin, &t0_vel[0]);
      for (ixstart = 0; ixstart < 3; ixstart++) {
        t0_pos[ixstart] = t0_vel[ixstart] * dt;
      }

      mw_neon_mm_add_f32x4(xt.robot_state.vel, 3, 1, t0_pos, &t0_vel[0]);
      for (ixstart = 0; ixstart < 3; ixstart++) {
        xt.robot_state.vel[ixstart] = t0_vel[ixstart];
      }

      //  velocity
      //  P_apr = (P_apr+P_apr')/2;
    } else {
      // 'SLAM:105' else
      // 'SLAM:106' [xt, P, updateVect, map, delayedStatus] = SLAM_upd(P, xt, cameraParameters, updateVect, z_all_l, z_all_r, noiseParameters, VIOParameters); 
      memcpy(&b_z_all_l[0], &z_all_l[0], 96U * sizeof(float));
      memcpy(&b_z_all_r[0], &z_all_r[0], 96U * sizeof(float));
      SLAM_upd(P, &xt, cameraParameters->CameraParameters1.ATAN,
               cameraParameters->CameraParameters1.FocalLength,
               cameraParameters->CameraParameters1.PrincipalPoint,
               cameraParameters->CameraParameters1.RadialDistortion,
               cameraParameters->CameraParameters1.DistortionModel,
               cameraParameters->CameraParameters2.ATAN,
               cameraParameters->CameraParameters2.FocalLength,
               cameraParameters->CameraParameters2.PrincipalPoint,
               cameraParameters->CameraParameters2.RadialDistortion,
               cameraParameters->CameraParameters2.DistortionModel,
               cameraParameters->r_lr, cameraParameters->R_lr,
               cameraParameters->R_rl, updateVect, b_z_all_l, b_z_all_r,
               noiseParameters->image_noise,
               noiseParameters->inv_depth_initial_unc, *b_VIOParameters, map,
               delayedStatus);
    }
  }

  // 'SLAM:109' map_out = map;
  memcpy(&map_out[0], &map[0], 144U * sizeof(float));

  // 'SLAM:110' xt_out = getWorldState(xt);
  getWorldState(xt.robot_state.IMU.pos, xt.robot_state.IMU.att,
                xt.robot_state.IMU.gyro_bias, xt.robot_state.IMU.acc_bias,
                xt.robot_state.pos, xt.robot_state.att, xt.robot_state.vel,
                xt.origin.pos, xt.origin.att, t0_pos, varargin_1, t0_vel,
                t0_IMU_gyro_bias, t0_IMU_acc_bias, c, t0_IMU_att);
  for (ixstart = 0; ixstart < 3; ixstart++) {
    xt_out->pos[ixstart] = t0_pos[ixstart];
  }

  for (ixstart = 0; ixstart < 4; ixstart++) {
    xt_out->att[ixstart] = varargin_1[ixstart];
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    xt_out->vel[ixstart] = t0_vel[ixstart];
    xt_out->IMU.gyro_bias[ixstart] = t0_IMU_gyro_bias[ixstart];
    xt_out->IMU.acc_bias[ixstart] = t0_IMU_acc_bias[ixstart];
    xt_out->IMU.pos[ixstart] = c[ixstart];
  }

  for (ixstart = 0; ixstart < 4; ixstart++) {
    xt_out->IMU.att[ixstart] = t0_IMU_att[ixstart];
  }

  // 'SLAM:111' anchor_poses_out = getAnchorPoses(xt);
  getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, rv1);
  cast(rv1, anchor_poses_out);

  // 'SLAM:112' delayedStatus_out = delayedStatus;
  memcpy(&delayedStatus_out[0], &delayedStatus[0], 48U * sizeof(float));

  //  output
  //  coder.cstructname(xt_out, 'RobotState');
  //  coder.cstructname(xt_out.IMU, 'IMUState');
  //  coder.cstructname(anchor_poses_out(1), 'AnchorPose');
  // 'SLAM:119' coder.cstructname(xt_out, 'RobotState', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:120' coder.cstructname(xt_out.IMU, 'IMUState', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:121' coder.cstructname(anchor_poses_out(1), 'AnchorPose', 'extern', 'HeaderFile', 'InterfaceStructs.h'); 
  // 'SLAM:123' assert ( all ( size (map_out) == [numTrackFeatures*3 1] ) )
  // 'SLAM:124' assert ( all ( size (anchor_poses_out) == [numAnchors 1] ) )
  // 'SLAM:125' assert ( all ( size (updateVect) == [numTrackFeatures 1] ) )
  // 'SLAM:126' assert ( all ( size (delayedStatus_out) == [numTrackFeatures 1] ) ) 
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  rt_InitInfAndNaN(8U);
  debug_level = b_debug_level;
  SLAM_init();
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_terminate()
{
  SLAM_free();
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
