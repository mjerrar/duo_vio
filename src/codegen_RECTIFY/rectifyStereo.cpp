//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rectifyStereo.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Jun-2015 16:07:13
//

// Include Files
#include "rt_nonfinite.h"
#include "rectifyStereo.h"
#include "libmwremaptbb.h"

// Type Definitions
#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T

struct emxArray_char_T
{
  char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_char_T

typedef struct {
  double T[9];
} projective2d;

typedef struct {
  projective2d H1;
  projective2d H2;
  double XBounds[2];
  double YBounds[2];
  double OriginalImageSize[2];
  emxArray_char_T *OutputView;
} c_vision_internal_calibration_R;

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
  emxArray_real32_T *XmapSingle;
  emxArray_real32_T *YmapSingle;
  emxArray_real_T *SizeOfImage;
  emxArray_char_T *ClassOfImage;
  emxArray_char_T *OutputView;
  double XBounds[2];
  double YBounds[2];
} c_vision_internal_calibration_I;

typedef struct {
  double RadialDistortion[3];
  double TangentialDistortion[2];
  char WorldUnits[2];
  double NumRadialDistortionCoefficients;
  c_vision_internal_calibration_I UndistortMap;
  double IntrinsicMatrixInternal[9];
} d_vision_internal_calibration_C;

typedef struct {
  d_vision_internal_calibration_C *CameraParameters1;
  d_vision_internal_calibration_C *CameraParameters2;
  double TranslationOfCamera2[3];
  c_vision_internal_calibration_R RectificationParams;
} c_vision_internal_calibration_S;

typedef struct {
  double RadialDistortion[3];
  double TangentialDistortion[2];
  char WorldUnits[2];
  double NumRadialDistortionCoefficients;
  double IntrinsicMatrixInternal[9];
} c_vision_internal_calibration_C;

typedef struct {
  c_vision_internal_calibration_C *CameraParameters1;
  c_vision_internal_calibration_C *CameraParameters2;
  double RotationOfCamera2[9];
  double TranslationOfCamera2[3];
  c_vision_internal_calibration_I RectifyMap1;
  c_vision_internal_calibration_I RectifyMap2;
  c_vision_internal_calibration_R RectificationParams;
} d_vision_internal_calibration_S;

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

// Variable Definitions
static c_vision_internal_calibration_C gobj_1;
static c_vision_internal_calibration_C gobj_0;
static boolean_T initialized_not_empty;
static double cameraparams[4];
static d_vision_internal_calibration_C cameraParams_l;
static d_vision_internal_calibration_C cameraParams_r;
static c_vision_internal_calibration_S stereoParams;
static d_vision_internal_calibration_S stereoParamsSecond;

// Function Declarations
static void ImageTransformer_computeMap(c_vision_internal_calibration_I *b_this,
  const double intrinsicMatrix[9], const double radialDist[3], const double
  tangentialDist[2]);
static boolean_T ImageTransformer_needToUpdate(const
  c_vision_internal_calibration_I *b_this);
static void ImageTransformer_transformImage(const
  c_vision_internal_calibration_I *b_this, const unsigned char I[360960],
  emxArray_uint8_T *J, double newOrigin[2]);
static void b_ImageTransformer_computeMap(c_vision_internal_calibration_I
  *b_this, const double intrinsicMatrix[9], const double radialDist[3], const
  double tangentialDist[2]);
static boolean_T b_ImageTransformer_needToUpdate(const
  c_vision_internal_calibration_I *b_this);
static void b_ImageTransformer_transformIma(const
  c_vision_internal_calibration_I *b_this, const unsigned char I[360960],
  emxArray_uint8_T *J);
static void b_abs(const emxArray_real_T *x, emxArray_real_T *y);
static void b_eml_sort(double x[8]);
static void b_eml_xaxpy(int n, double a, const double x[9], int ix0, double y[3],
  int iy0);
static double b_eml_xnrm2(const double x[3], int ix0);
static void b_emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
  numDimensions);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void bwtraceboundary(const emxArray_uint8_T *varargin_1, const double
  varargin_2[2], emxArray_real_T *B);
static d_vision_internal_calibration_C *c_CameraParametersImpl_CameraPa
  (d_vision_internal_calibration_C *b_this, const double
   varargin_1_RadialDistortion[3], const double varargin_1_TangentialDistortion
   [2], const char varargin_1_WorldUnits[2], double
   c_varargin_1_NumRadialDistortio, const double varargin_1_IntrinsicMatrix[9]);
static void c_CameraParametersImpl_computeU(c_vision_internal_calibration_C
  *b_this, double xBounds[2], double yBounds[2]);
static void c_CameraParametersImpl_createUn(c_vision_internal_calibration_C
  *b_this, emxArray_uint8_T *undistortedMask, double xBoundsBig[2], double
  yBoundsBig[2]);
static void c_ImageTransformer_computeMap(c_vision_internal_calibration_I
  *b_this, const double intrinsicMatrix[9], const double radialDist[3], const
  double tangentialDist[2]);
static c_vision_internal_calibration_R *c_RectificationParameters_Recti
  (c_vision_internal_calibration_R *b_this);
static c_vision_internal_calibration_S *c_StereoParametersImpl_StereoPa
  (c_vision_internal_calibration_S *b_this, const double
   c_varargin_1_CameraParameters1_[3], const double
   d_varargin_1_CameraParameters1_[2], const char
   e_varargin_1_CameraParameters1_[2], double f_varargin_1_CameraParameters1_,
   const double g_varargin_1_CameraParameters1_[9], const double
   c_varargin_1_CameraParameters2_[3], const double
   d_varargin_1_CameraParameters2_[2], const char
   e_varargin_1_CameraParameters2_[2], double f_varargin_1_CameraParameters2_,
   const double g_varargin_1_CameraParameters2_[9], const double
   varargin_1_TranslationOfCamera2[3], const struct3_T
   *varargin_1_RectificationParams, d_vision_internal_calibration_C *iobj_0,
   d_vision_internal_calibration_C *iobj_1);
static void c_StereoParametersImpl_computeH(const
  d_vision_internal_calibration_S *b_this, double Rl[9], double Rr[9]);
static void c_StereoParametersImpl_computeO(const
  d_vision_internal_calibration_S *b_this, const double Hleft_T[9], const double
  Hright_T[9], double xBounds[2], double yBounds[2]);
static void c_StereoParametersImpl_computeR(d_vision_internal_calibration_S
  *b_this, double Hleft_T[9], double Hright_T[9], double Q[16], double xBounds[2],
  double yBounds[2]);
static void c_eml_xaxpy(int n, double a, const double x[3], int ix0, double y[9],
  int iy0);
static void c_emxFreeStruct_vision_internal(c_vision_internal_calibration_I
  *pStruct);
static void c_emxInitStruct_vision_internal(d_vision_internal_calibration_S
  *pStruct);
static void computeRowAlignmentRotation(const double t[3], double RrowAlign[9]);
static c_vision_internal_calibration_C *d_CameraParametersImpl_CameraPa
  (c_vision_internal_calibration_C *b_this, const double
   varargin_1_RadialDistortion[3], const double varargin_1_TangentialDistortion
   [2], const char varargin_1_WorldUnits[2], double
   c_varargin_1_NumRadialDistortio, const double varargin_1_IntrinsicMatrix[9]);
static void d_ImageTransformer_computeMap(c_vision_internal_calibration_I
  *b_this, const double intrinsicMatrix[9], const double radialDist[3], const
  double tangentialDist[2], const double H_T[9]);
static d_vision_internal_calibration_S *d_StereoParametersImpl_StereoPa
  (d_vision_internal_calibration_S *b_this, const double
   c_varargin_1_CameraParameters1_[3], const double
   d_varargin_1_CameraParameters1_[2], const char
   e_varargin_1_CameraParameters1_[2], double f_varargin_1_CameraParameters1_,
   const double g_varargin_1_CameraParameters1_[9], const double
   c_varargin_1_CameraParameters2_[3], const double
   d_varargin_1_CameraParameters2_[2], const char
   e_varargin_1_CameraParameters2_[2], double f_varargin_1_CameraParameters2_,
   const double g_varargin_1_CameraParameters2_[9], const double
   varargin_1_RotationOfCamera2[9], const double
   varargin_1_TranslationOfCamera2[3], const struct3_T
   *varargin_1_RectificationParams, c_vision_internal_calibration_C *iobj_0,
   c_vision_internal_calibration_C *iobj_1);
static void d_emxFreeStruct_vision_internal(d_vision_internal_calibration_S
  *pStruct);
static void d_emxInitStruct_vision_internal(c_vision_internal_calibration_I
  *pStruct);
static void distortPoints(const emxArray_real_T *points, const double
  intrinsicMatrix[9], const double radialDistortion[3], const double
  tangentialDistortion[2], emxArray_real_T *distortedPoints);
static void e_ImageTransformer_computeMap(c_vision_internal_calibration_I
  *b_this, const double intrinsicMatrix[9], const double radialDist[3], const
  double tangentialDist[2], const double H_T[9]);
static void e_emxFreeStruct_vision_internal(c_vision_internal_calibration_R
  *pStruct);
static void e_emxInitStruct_vision_internal(c_vision_internal_calibration_R
  *pStruct);
static void eml_sort(double x[2]);
static void eml_xaxpy(int n, double a, int ix0, double y[9], int iy0);
static double eml_xdotc(int n, const double x[9], int ix0, const double y[9],
  int iy0);
static void eml_xgesvd(const double A[9], double U[9], double S[3], double V[9]);
static double eml_xnrm2(int n, const double x[9], int ix0);
static void eml_xrot(double x[9], int ix0, int iy0, double c, double s);
static void eml_xrotg(double *a, double *b, double *c, double *s);
static void eml_xscal(double a, double x[9], int ix0);
static void eml_xswap(double x[9], int ix0, int iy0);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
static void emxFree_char_T(emxArray_char_T **pEmxArray);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_real32_T(emxArray_real32_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxFree_uint8_T(emxArray_uint8_T **pEmxArray);
static void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions);
static void emxInit_char_T(emxArray_char_T **pEmxArray, int numDimensions);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void emxInit_uint8_T(emxArray_uint8_T **pEmxArray, int numDimensions);
static void f_emxFreeStruct_vision_internal(c_vision_internal_calibration_S
  *pStruct);
static void f_emxInitStruct_vision_internal(c_vision_internal_calibration_S
  *pStruct);
static void g_emxFreeStruct_vision_internal(d_vision_internal_calibration_C
  *pStruct);
static void g_emxInitStruct_vision_internal(d_vision_internal_calibration_C
  *pStruct);
static boolean_T isequal(const emxArray_real_T *varargin_1, const double
  varargin_2[2]);
static void merge(int idx[8], double x[8], int offset, int np, int nq);
static void meshgrid(const emxArray_real_T *x, const emxArray_real_T *y,
                     emxArray_real_T *xx, emxArray_real_T *yy);
static void mrdivide(const double A[9], const double B[9], double y[9]);
static double norm(const double x[3]);
static void padarray(const emxArray_real_T *varargin_1, emxArray_real_T *b);
static void power(const emxArray_real_T *a, emxArray_real_T *y);
static void rdivide(const emxArray_real_T *x, double y, emxArray_real_T *z);
static void rectifyStereo_free();
static void rectifyStereo_init();
static void rodriguesVectorToMatrix(const double rotationVector[3], double
  rotationMatrix[9]);
static double rt_roundd_snf(double u);

// Function Definitions

//
// Arguments    : c_vision_internal_calibration_I *b_this
//                const double intrinsicMatrix[9]
//                const double radialDist[3]
//                const double tangentialDist[2]
// Return Type  : void
//
static void ImageTransformer_computeMap(c_vision_internal_calibration_I *b_this,
  const double intrinsicMatrix[9], const double radialDist[3], const double
  tangentialDist[2])
{
  double n;
  double m;
  int b_n;
  double apnd;
  double ndbl;
  double cdiff;
  double absa;
  double absb;
  emxArray_real_T *y;
  int i10;
  int nm1d2;
  int k;
  emxArray_real_T *b_y;
  emxArray_real_T *X;
  emxArray_real_T *Y;
  emxArray_real_T *b_X;
  emxArray_real_T *c_X;
  emxArray_real_T *b_Y;
  emxArray_real_T *ptsOut;
  emxArray_char_T *a;
  boolean_T b_bool;
  int32_T exitg1;
  static const char cv12[6] = { 'd', 'o', 'u', 'b', 'l', 'e' };

  double varargin_1[2];
  int iv0[2];
  n = b_this->XBounds[0];
  m = b_this->XBounds[1];
  if (rtIsNaN(n) || rtIsNaN(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else if (m < n) {
    b_n = -1;
    apnd = m;
  } else if (rtIsInf(n) || rtIsInf(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else {
    ndbl = floor((m - n) + 0.5);
    apnd = n + ndbl;
    cdiff = apnd - m;
    absa = fabs(n);
    absb = fabs(m);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = m;
    } else if (cdiff > 0.0) {
      apnd = n + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      b_n = (int)ndbl - 1;
    } else {
      b_n = -1;
    }
  }

  emxInit_real_T(&y, 2);
  i10 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = b_n + 1;
  emxEnsureCapacity((emxArray__common *)y, i10, (int)sizeof(double));
  if (b_n + 1 > 0) {
    y->data[0] = n;
    if (b_n + 1 > 1) {
      y->data[b_n] = apnd;
      nm1d2 = b_n / 2;
      for (k = 1; k < nm1d2; k++) {
        y->data[k] = n + (double)k;
        y->data[b_n - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == b_n) {
        y->data[nm1d2] = (n + apnd) / 2.0;
      } else {
        y->data[nm1d2] = n + (double)nm1d2;
        y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  n = b_this->YBounds[0];
  m = b_this->YBounds[1];
  if (rtIsNaN(n) || rtIsNaN(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else if (m < n) {
    b_n = -1;
    apnd = m;
  } else if (rtIsInf(n) || rtIsInf(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else {
    ndbl = floor((m - n) + 0.5);
    apnd = n + ndbl;
    cdiff = apnd - m;
    absa = fabs(n);
    absb = fabs(m);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = m;
    } else if (cdiff > 0.0) {
      apnd = n + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      b_n = (int)ndbl - 1;
    } else {
      b_n = -1;
    }
  }

  emxInit_real_T(&b_y, 2);
  i10 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = b_n + 1;
  emxEnsureCapacity((emxArray__common *)b_y, i10, (int)sizeof(double));
  if (b_n + 1 > 0) {
    b_y->data[0] = n;
    if (b_n + 1 > 1) {
      b_y->data[b_n] = apnd;
      nm1d2 = b_n / 2;
      for (k = 1; k < nm1d2; k++) {
        b_y->data[k] = n + (double)k;
        b_y->data[b_n - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == b_n) {
        b_y->data[nm1d2] = (n + apnd) / 2.0;
      } else {
        b_y->data[nm1d2] = n + (double)nm1d2;
        b_y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  emxInit_real_T(&X, 2);
  emxInit_real_T(&Y, 2);
  emxInit_real_T(&b_X, 2);
  b_emxInit_real_T(&c_X, 1);
  meshgrid(y, b_y, X, Y);
  nm1d2 = X->size[0] * X->size[1];
  k = X->size[0] * X->size[1];
  i10 = c_X->size[0];
  c_X->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)c_X, i10, (int)sizeof(double));
  emxFree_real_T(&b_y);
  emxFree_real_T(&y);
  for (i10 = 0; i10 < nm1d2; i10++) {
    c_X->data[i10] = X->data[i10];
  }

  b_emxInit_real_T(&b_Y, 1);
  nm1d2 = Y->size[0] * Y->size[1];
  b_n = Y->size[0] * Y->size[1];
  i10 = b_Y->size[0];
  b_Y->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)b_Y, i10, (int)sizeof(double));
  for (i10 = 0; i10 < nm1d2; i10++) {
    b_Y->data[i10] = Y->data[i10];
  }

  emxFree_real_T(&Y);
  i10 = b_X->size[0] * b_X->size[1];
  b_X->size[0] = k;
  b_X->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b_X, i10, (int)sizeof(double));
  for (i10 = 0; i10 < k; i10++) {
    b_X->data[i10] = c_X->data[i10];
  }

  emxFree_real_T(&c_X);
  for (i10 = 0; i10 < b_n; i10++) {
    b_X->data[i10 + b_X->size[0]] = b_Y->data[i10];
  }

  emxFree_real_T(&b_Y);
  emxInit_real_T(&ptsOut, 2);
  emxInit_char_T(&a, 2);
  distortPoints(b_X, intrinsicMatrix, radialDist, tangentialDist, ptsOut);
  m = (b_this->YBounds[1] - b_this->YBounds[0]) + 1.0;
  n = (b_this->XBounds[1] - b_this->XBounds[0]) + 1.0;
  i10 = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = b_this->ClassOfImage->size[1];
  emxEnsureCapacity((emxArray__common *)a, i10, (int)sizeof(char));
  nm1d2 = b_this->ClassOfImage->size[0] * b_this->ClassOfImage->size[1];
  emxFree_real_T(&b_X);
  for (i10 = 0; i10 < nm1d2; i10++) {
    a->data[i10] = b_this->ClassOfImage->data[i10];
  }

  b_bool = false;
  if (a->size[1] != 6) {
  } else {
    k = 0;
    do {
      exitg1 = 0;
      if (k <= 5) {
        if (a->data[k] != cv12[k]) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  emxFree_char_T(&a);
  if (b_bool) {
  } else {
    varargin_1[0] = m;
    varargin_1[1] = n;
    i10 = ptsOut->size[0];
    for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
      iv0[nm1d2] = (int)varargin_1[nm1d2];
    }

    nm1d2 = X->size[0] * X->size[1];
    X->size[0] = iv0[0];
    X->size[1] = iv0[1];
    emxEnsureCapacity((emxArray__common *)X, nm1d2, (int)sizeof(double));
    for (k = 0; k + 1 <= i10; k++) {
      X->data[k] = ptsOut->data[k];
    }

    i10 = b_this->XmapSingle->size[0] * b_this->XmapSingle->size[1];
    b_this->XmapSingle->size[0] = X->size[0];
    b_this->XmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->XmapSingle, i10, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_this->XmapSingle->data[i10] = (float)X->data[i10];
    }

    varargin_1[0] = m;
    varargin_1[1] = n;
    i10 = ptsOut->size[0];
    for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
      iv0[nm1d2] = (int)varargin_1[nm1d2];
    }

    nm1d2 = X->size[0] * X->size[1];
    X->size[0] = iv0[0];
    X->size[1] = iv0[1];
    emxEnsureCapacity((emxArray__common *)X, nm1d2, (int)sizeof(double));
    for (k = 0; k + 1 <= i10; k++) {
      X->data[k] = ptsOut->data[k + ptsOut->size[0]];
    }

    i10 = b_this->YmapSingle->size[0] * b_this->YmapSingle->size[1];
    b_this->YmapSingle->size[0] = X->size[0];
    b_this->YmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->YmapSingle, i10, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_this->YmapSingle->data[i10] = (float)X->data[i10];
    }
  }

  emxFree_real_T(&X);
  emxFree_real_T(&ptsOut);
}

//
// Arguments    : const c_vision_internal_calibration_I *b_this
// Return Type  : boolean_T
//
static boolean_T ImageTransformer_needToUpdate(const
  c_vision_internal_calibration_I *b_this)
{
  double dv0[2];
  int k;
  emxArray_char_T *b;
  boolean_T sameSize;
  int loop_ub;
  boolean_T sameClass;
  int32_T exitg3;
  static const char cv10[5] = { 'u', 'i', 'n', 't', '8' };

  boolean_T sameOutputView;
  boolean_T p;
  int32_T exitg2;
  boolean_T exitg1;
  static const char cv11[4] = { 's', 'a', 'm', 'e' };

  for (k = 0; k < 2; k++) {
    dv0[k] = 480.0 + 272.0 * (double)k;
  }

  emxInit_char_T(&b, 2);
  sameSize = isequal(b_this->SizeOfImage, dv0);
  k = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = b_this->ClassOfImage->size[1];
  emxEnsureCapacity((emxArray__common *)b, k, (int)sizeof(char));
  loop_ub = b_this->ClassOfImage->size[0] * b_this->ClassOfImage->size[1];
  for (k = 0; k < loop_ub; k++) {
    b->data[k] = b_this->ClassOfImage->data[k];
  }

  sameClass = false;
  if (5 != b->size[1]) {
  } else {
    k = 0;
    do {
      exitg3 = 0;
      if (k < 5) {
        if (cv10[k] != b->data[k]) {
          exitg3 = 1;
        } else {
          k++;
        }
      } else {
        sameClass = true;
        exitg3 = 1;
      }
    } while (exitg3 == 0);
  }

  k = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = b_this->OutputView->size[1];
  emxEnsureCapacity((emxArray__common *)b, k, (int)sizeof(char));
  loop_ub = b_this->OutputView->size[0] * b_this->OutputView->size[1];
  for (k = 0; k < loop_ub; k++) {
    b->data[k] = b_this->OutputView->data[k];
  }

  sameOutputView = false;
  p = false;
  k = 0;
  do {
    exitg2 = 0;
    if (k < 2) {
      if (b->size[k] != 1 + 3 * k) {
        exitg2 = 1;
      } else {
        k++;
      }
    } else {
      p = true;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  if (p && (!(b->size[1] == 0))) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!(b->data[k] == cv11[k])) {
        p = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  emxFree_char_T(&b);
  if (!p) {
  } else {
    sameOutputView = true;
  }

  if (sameSize && sameClass && sameOutputView) {
    sameSize = true;
  } else {
    sameSize = false;
  }

  return !sameSize;
}

//
// Arguments    : const c_vision_internal_calibration_I *b_this
//                const unsigned char I[360960]
//                emxArray_uint8_T *J
//                double newOrigin[2]
// Return Type  : void
//
static void ImageTransformer_transformImage(const
  c_vision_internal_calibration_I *b_this, const unsigned char I[360960],
  emxArray_uint8_T *J, double newOrigin[2])
{
  emxArray_real32_T *X;
  int j;
  int loop_ub;
  emxArray_real32_T *Yin;
  static unsigned char inputImage[368388];
  unsigned char fillValues;
  double dv1[2];
  double b_J[2];
  double c_this[2];
  emxInit_real32_T(&X, 2);
  j = X->size[0] * X->size[1];
  X->size[0] = b_this->XmapSingle->size[0];
  X->size[1] = b_this->XmapSingle->size[1];
  emxEnsureCapacity((emxArray__common *)X, j, (int)sizeof(float));
  loop_ub = b_this->XmapSingle->size[0] * b_this->XmapSingle->size[1];
  for (j = 0; j < loop_ub; j++) {
    X->data[j] = b_this->XmapSingle->data[j];
  }

  emxInit_real32_T(&Yin, 2);
  j = Yin->size[0] * Yin->size[1];
  Yin->size[0] = b_this->YmapSingle->size[0];
  Yin->size[1] = b_this->YmapSingle->size[1];
  emxEnsureCapacity((emxArray__common *)Yin, j, (int)sizeof(float));
  loop_ub = b_this->YmapSingle->size[0] * b_this->YmapSingle->size[1];
  for (j = 0; j < loop_ub; j++) {
    Yin->data[j] = b_this->YmapSingle->data[j];
  }

  for (j = 0; j < 3; j++) {
    memset(&inputImage[486 * j], 0, 486U * sizeof(unsigned char));
  }

  for (j = 0; j < 3; j++) {
    memset(&inputImage[486 * (j + 755)], 0, 486U * sizeof(unsigned char));
  }

  for (j = 0; j < 752; j++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      inputImage[loop_ub + 486 * (j + 3)] = 0;
    }

    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      inputImage[(loop_ub + 486 * (j + 3)) + 483] = 0;
    }

    memcpy(&inputImage[3 + 486 * (j + 3)], &I[480 * j], 480U * sizeof(unsigned
            char));
  }

  j = X->size[0] * X->size[1];
  emxEnsureCapacity((emxArray__common *)X, j, (int)sizeof(float));
  loop_ub = X->size[0];
  j = X->size[1];
  loop_ub *= j;
  for (j = 0; j < loop_ub; j++) {
    X->data[j] = (X->data[j] + 3.0F) - 1.0F;
  }

  j = J->size[0] * J->size[1];
  J->size[0] = X->size[0];
  J->size[1] = X->size[1];
  emxEnsureCapacity((emxArray__common *)J, j, (int)sizeof(unsigned char));
  j = Yin->size[0] * Yin->size[1];
  emxEnsureCapacity((emxArray__common *)Yin, j, (int)sizeof(float));
  loop_ub = Yin->size[0];
  j = Yin->size[1];
  loop_ub *= j;
  for (j = 0; j < loop_ub; j++) {
    Yin->data[j] = (Yin->data[j] + 3.0F) - 1.0F;
  }

  fillValues = 0;
  for (j = 0; j < 2; j++) {
    dv1[j] = 486.0 + 272.0 * (double)j;
  }

  for (j = 0; j < 2; j++) {
    b_J[j] = J->size[j];
  }

  remaptbb_uint8(inputImage, dv1, 2.0, &Yin->data[0], &X->data[0], 2,
                 &fillValues, &J->data[0], b_J, (double)(J->size[0] * J->size[1]));
  c_this[0] = b_this->XBounds[0];
  c_this[1] = b_this->YBounds[0];
  emxFree_real32_T(&X);
  emxFree_real32_T(&Yin);
  for (j = 0; j < 2; j++) {
    newOrigin[j] = c_this[j] - 1.0;
  }
}

//
// Arguments    : c_vision_internal_calibration_I *b_this
//                const double intrinsicMatrix[9]
//                const double radialDist[3]
//                const double tangentialDist[2]
// Return Type  : void
//
static void b_ImageTransformer_computeMap(c_vision_internal_calibration_I
  *b_this, const double intrinsicMatrix[9], const double radialDist[3], const
  double tangentialDist[2])
{
  double n;
  double m;
  int b_n;
  double apnd;
  double ndbl;
  double cdiff;
  double absa;
  double absb;
  emxArray_real_T *y;
  int i13;
  int nm1d2;
  int k;
  emxArray_real_T *b_y;
  emxArray_real_T *X;
  emxArray_real_T *Y;
  emxArray_real_T *b_X;
  emxArray_real_T *c_X;
  emxArray_real_T *b_Y;
  emxArray_real_T *ptsOut;
  emxArray_char_T *a;
  boolean_T b_bool;
  int32_T exitg1;
  static const char cv13[6] = { 'd', 'o', 'u', 'b', 'l', 'e' };

  double varargin_1[2];
  int iv1[2];
  n = b_this->XBounds[0];
  m = b_this->XBounds[1];
  if (rtIsNaN(n) || rtIsNaN(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else if (m < n) {
    b_n = -1;
    apnd = m;
  } else if (rtIsInf(n) || rtIsInf(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else {
    ndbl = floor((m - n) + 0.5);
    apnd = n + ndbl;
    cdiff = apnd - m;
    absa = fabs(n);
    absb = fabs(m);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = m;
    } else if (cdiff > 0.0) {
      apnd = n + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      b_n = (int)ndbl - 1;
    } else {
      b_n = -1;
    }
  }

  emxInit_real_T(&y, 2);
  i13 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = b_n + 1;
  emxEnsureCapacity((emxArray__common *)y, i13, (int)sizeof(double));
  if (b_n + 1 > 0) {
    y->data[0] = n;
    if (b_n + 1 > 1) {
      y->data[b_n] = apnd;
      nm1d2 = b_n / 2;
      for (k = 1; k < nm1d2; k++) {
        y->data[k] = n + (double)k;
        y->data[b_n - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == b_n) {
        y->data[nm1d2] = (n + apnd) / 2.0;
      } else {
        y->data[nm1d2] = n + (double)nm1d2;
        y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  n = b_this->YBounds[0];
  m = b_this->YBounds[1];
  if (rtIsNaN(n) || rtIsNaN(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else if (m < n) {
    b_n = -1;
    apnd = m;
  } else if (rtIsInf(n) || rtIsInf(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else {
    ndbl = floor((m - n) + 0.5);
    apnd = n + ndbl;
    cdiff = apnd - m;
    absa = fabs(n);
    absb = fabs(m);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = m;
    } else if (cdiff > 0.0) {
      apnd = n + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      b_n = (int)ndbl - 1;
    } else {
      b_n = -1;
    }
  }

  emxInit_real_T(&b_y, 2);
  i13 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = b_n + 1;
  emxEnsureCapacity((emxArray__common *)b_y, i13, (int)sizeof(double));
  if (b_n + 1 > 0) {
    b_y->data[0] = n;
    if (b_n + 1 > 1) {
      b_y->data[b_n] = apnd;
      nm1d2 = b_n / 2;
      for (k = 1; k < nm1d2; k++) {
        b_y->data[k] = n + (double)k;
        b_y->data[b_n - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == b_n) {
        b_y->data[nm1d2] = (n + apnd) / 2.0;
      } else {
        b_y->data[nm1d2] = n + (double)nm1d2;
        b_y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  emxInit_real_T(&X, 2);
  emxInit_real_T(&Y, 2);
  emxInit_real_T(&b_X, 2);
  b_emxInit_real_T(&c_X, 1);
  meshgrid(y, b_y, X, Y);
  nm1d2 = X->size[0] * X->size[1];
  k = X->size[0] * X->size[1];
  i13 = c_X->size[0];
  c_X->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)c_X, i13, (int)sizeof(double));
  emxFree_real_T(&b_y);
  emxFree_real_T(&y);
  for (i13 = 0; i13 < nm1d2; i13++) {
    c_X->data[i13] = X->data[i13];
  }

  b_emxInit_real_T(&b_Y, 1);
  nm1d2 = Y->size[0] * Y->size[1];
  b_n = Y->size[0] * Y->size[1];
  i13 = b_Y->size[0];
  b_Y->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)b_Y, i13, (int)sizeof(double));
  for (i13 = 0; i13 < nm1d2; i13++) {
    b_Y->data[i13] = Y->data[i13];
  }

  emxFree_real_T(&Y);
  i13 = b_X->size[0] * b_X->size[1];
  b_X->size[0] = k;
  b_X->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b_X, i13, (int)sizeof(double));
  for (i13 = 0; i13 < k; i13++) {
    b_X->data[i13] = c_X->data[i13];
  }

  emxFree_real_T(&c_X);
  for (i13 = 0; i13 < b_n; i13++) {
    b_X->data[i13 + b_X->size[0]] = b_Y->data[i13];
  }

  emxFree_real_T(&b_Y);
  emxInit_real_T(&ptsOut, 2);
  emxInit_char_T(&a, 2);
  distortPoints(b_X, intrinsicMatrix, radialDist, tangentialDist, ptsOut);
  m = (b_this->YBounds[1] - b_this->YBounds[0]) + 1.0;
  n = (b_this->XBounds[1] - b_this->XBounds[0]) + 1.0;
  i13 = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = b_this->ClassOfImage->size[1];
  emxEnsureCapacity((emxArray__common *)a, i13, (int)sizeof(char));
  nm1d2 = b_this->ClassOfImage->size[0] * b_this->ClassOfImage->size[1];
  emxFree_real_T(&b_X);
  for (i13 = 0; i13 < nm1d2; i13++) {
    a->data[i13] = b_this->ClassOfImage->data[i13];
  }

  b_bool = false;
  if (a->size[1] != 6) {
  } else {
    k = 0;
    do {
      exitg1 = 0;
      if (k <= 5) {
        if (a->data[k] != cv13[k]) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  emxFree_char_T(&a);
  if (b_bool) {
  } else {
    varargin_1[0] = m;
    varargin_1[1] = n;
    i13 = ptsOut->size[0];
    for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
      iv1[nm1d2] = (int)varargin_1[nm1d2];
    }

    nm1d2 = X->size[0] * X->size[1];
    X->size[0] = iv1[0];
    X->size[1] = iv1[1];
    emxEnsureCapacity((emxArray__common *)X, nm1d2, (int)sizeof(double));
    for (k = 0; k + 1 <= i13; k++) {
      X->data[k] = ptsOut->data[k];
    }

    i13 = b_this->XmapSingle->size[0] * b_this->XmapSingle->size[1];
    b_this->XmapSingle->size[0] = X->size[0];
    b_this->XmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->XmapSingle, i13, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i13 = 0; i13 < nm1d2; i13++) {
      b_this->XmapSingle->data[i13] = (float)X->data[i13];
    }

    varargin_1[0] = m;
    varargin_1[1] = n;
    i13 = ptsOut->size[0];
    for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
      iv1[nm1d2] = (int)varargin_1[nm1d2];
    }

    nm1d2 = X->size[0] * X->size[1];
    X->size[0] = iv1[0];
    X->size[1] = iv1[1];
    emxEnsureCapacity((emxArray__common *)X, nm1d2, (int)sizeof(double));
    for (k = 0; k + 1 <= i13; k++) {
      X->data[k] = ptsOut->data[k + ptsOut->size[0]];
    }

    i13 = b_this->YmapSingle->size[0] * b_this->YmapSingle->size[1];
    b_this->YmapSingle->size[0] = X->size[0];
    b_this->YmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->YmapSingle, i13, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i13 = 0; i13 < nm1d2; i13++) {
      b_this->YmapSingle->data[i13] = (float)X->data[i13];
    }
  }

  emxFree_real_T(&X);
  emxFree_real_T(&ptsOut);
}

//
// Arguments    : const c_vision_internal_calibration_I *b_this
// Return Type  : boolean_T
//
static boolean_T b_ImageTransformer_needToUpdate(const
  c_vision_internal_calibration_I *b_this)
{
  double dv2[2];
  int k;
  emxArray_char_T *b;
  boolean_T sameSize;
  int loop_ub;
  boolean_T sameClass;
  int32_T exitg3;
  static const char cv14[5] = { 'u', 'i', 'n', 't', '8' };

  boolean_T sameOutputView;
  boolean_T p;
  int32_T exitg2;
  boolean_T exitg1;
  static const char cv15[5] = { 'v', 'a', 'l', 'i', 'd' };

  for (k = 0; k < 2; k++) {
    dv2[k] = 480.0 + 272.0 * (double)k;
  }

  emxInit_char_T(&b, 2);
  sameSize = isequal(b_this->SizeOfImage, dv2);
  k = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = b_this->ClassOfImage->size[1];
  emxEnsureCapacity((emxArray__common *)b, k, (int)sizeof(char));
  loop_ub = b_this->ClassOfImage->size[0] * b_this->ClassOfImage->size[1];
  for (k = 0; k < loop_ub; k++) {
    b->data[k] = b_this->ClassOfImage->data[k];
  }

  sameClass = false;
  if (5 != b->size[1]) {
  } else {
    k = 0;
    do {
      exitg3 = 0;
      if (k < 5) {
        if (cv14[k] != b->data[k]) {
          exitg3 = 1;
        } else {
          k++;
        }
      } else {
        sameClass = true;
        exitg3 = 1;
      }
    } while (exitg3 == 0);
  }

  k = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = b_this->OutputView->size[1];
  emxEnsureCapacity((emxArray__common *)b, k, (int)sizeof(char));
  loop_ub = b_this->OutputView->size[0] * b_this->OutputView->size[1];
  for (k = 0; k < loop_ub; k++) {
    b->data[k] = b_this->OutputView->data[k];
  }

  sameOutputView = false;
  p = false;
  k = 0;
  do {
    exitg2 = 0;
    if (k < 2) {
      if (b->size[k] != 1 + (k << 2)) {
        exitg2 = 1;
      } else {
        k++;
      }
    } else {
      p = true;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  if (p && (!(b->size[1] == 0))) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!(b->data[k] == cv15[k])) {
        p = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  emxFree_char_T(&b);
  if (!p) {
  } else {
    sameOutputView = true;
  }

  if (sameSize && sameClass && sameOutputView) {
    sameSize = true;
  } else {
    sameSize = false;
  }

  return !sameSize;
}

//
// Arguments    : const c_vision_internal_calibration_I *b_this
//                const unsigned char I[360960]
//                emxArray_uint8_T *J
// Return Type  : void
//
static void b_ImageTransformer_transformIma(const
  c_vision_internal_calibration_I *b_this, const unsigned char I[360960],
  emxArray_uint8_T *J)
{
  emxArray_real32_T *X;
  int j;
  int loop_ub;
  emxArray_real32_T *Yin;
  static unsigned char inputImage[368388];
  unsigned char fillValues;
  double dv4[2];
  double b_J[2];
  emxInit_real32_T(&X, 2);
  j = X->size[0] * X->size[1];
  X->size[0] = b_this->XmapSingle->size[0];
  X->size[1] = b_this->XmapSingle->size[1];
  emxEnsureCapacity((emxArray__common *)X, j, (int)sizeof(float));
  loop_ub = b_this->XmapSingle->size[0] * b_this->XmapSingle->size[1];
  for (j = 0; j < loop_ub; j++) {
    X->data[j] = b_this->XmapSingle->data[j];
  }

  emxInit_real32_T(&Yin, 2);
  j = Yin->size[0] * Yin->size[1];
  Yin->size[0] = b_this->YmapSingle->size[0];
  Yin->size[1] = b_this->YmapSingle->size[1];
  emxEnsureCapacity((emxArray__common *)Yin, j, (int)sizeof(float));
  loop_ub = b_this->YmapSingle->size[0] * b_this->YmapSingle->size[1];
  for (j = 0; j < loop_ub; j++) {
    Yin->data[j] = b_this->YmapSingle->data[j];
  }

  for (j = 0; j < 3; j++) {
    memset(&inputImage[486 * j], 0, 486U * sizeof(unsigned char));
  }

  for (j = 0; j < 3; j++) {
    memset(&inputImage[486 * (j + 755)], 0, 486U * sizeof(unsigned char));
  }

  for (j = 0; j < 752; j++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      inputImage[loop_ub + 486 * (j + 3)] = 0;
    }

    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      inputImage[(loop_ub + 486 * (j + 3)) + 483] = 0;
    }

    memcpy(&inputImage[3 + 486 * (j + 3)], &I[480 * j], 480U * sizeof(unsigned
            char));
  }

  j = X->size[0] * X->size[1];
  emxEnsureCapacity((emxArray__common *)X, j, (int)sizeof(float));
  loop_ub = X->size[0];
  j = X->size[1];
  loop_ub *= j;
  for (j = 0; j < loop_ub; j++) {
    X->data[j] = (X->data[j] + 3.0F) - 1.0F;
  }

  j = J->size[0] * J->size[1];
  J->size[0] = X->size[0];
  J->size[1] = X->size[1];
  emxEnsureCapacity((emxArray__common *)J, j, (int)sizeof(unsigned char));
  j = Yin->size[0] * Yin->size[1];
  emxEnsureCapacity((emxArray__common *)Yin, j, (int)sizeof(float));
  loop_ub = Yin->size[0];
  j = Yin->size[1];
  loop_ub *= j;
  for (j = 0; j < loop_ub; j++) {
    Yin->data[j] = (Yin->data[j] + 3.0F) - 1.0F;
  }

  fillValues = 0;
  for (j = 0; j < 2; j++) {
    dv4[j] = 486.0 + 272.0 * (double)j;
  }

  for (j = 0; j < 2; j++) {
    b_J[j] = J->size[j];
  }

  remaptbb_uint8(inputImage, dv4, 2.0, &Yin->data[0], &X->data[0], 2,
                 &fillValues, &J->data[0], b_J, (double)(J->size[0] * J->size[1]));
  emxFree_real32_T(&X);
  emxFree_real32_T(&Yin);
}

//
// Arguments    : const emxArray_real_T *x
//                emxArray_real_T *y
// Return Type  : void
//
static void b_abs(const emxArray_real_T *x, emxArray_real_T *y)
{
  unsigned int unnamed_idx_0;
  int k;
  unnamed_idx_0 = (unsigned int)x->size[0];
  k = y->size[0];
  y->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)y, k, (int)sizeof(double));
  for (k = 0; k < x->size[0]; k++) {
    y->data[k] = fabs(x->data[k]);
  }
}

//
// Arguments    : double x[8]
// Return Type  : void
//
static void b_eml_sort(double x[8])
{
  int idx[8];
  int m;
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
  for (m = 0; m < 8; m++) {
    idx[m] = 0;
  }

  for (m = 0; m < 4; m++) {
    x4[m] = 0.0;
    idx4[m] = 0;
  }

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
          m = 1;
          bLen = 2;
        } else {
          m = 2;
          bLen = 1;
        }

        if (x4[2] <= x4[3]) {
          nPairs = 3;
          i4 = 4;
        } else {
          nPairs = 4;
          i4 = 3;
        }

        if (x4[m - 1] <= x4[nPairs - 1]) {
          if (x4[bLen - 1] <= x4[nPairs - 1]) {
            perm[0] = (signed char)m;
            perm[1] = (signed char)bLen;
            perm[2] = (signed char)nPairs;
            perm[3] = (signed char)i4;
          } else if (x4[bLen - 1] <= x4[i4 - 1]) {
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
        } else if (x4[m - 1] <= x4[i4 - 1]) {
          if (x4[bLen - 1] <= x4[i4 - 1]) {
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
    for (m = 0; m < 4; m++) {
      perm[m] = 0;
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

  m = (nNaNs + 7) >> 1;
  for (k = 1; k <= m; k++) {
    ib = idx[k - nNaNs];
    idx[k - nNaNs] = idx[8 - k];
    idx[8 - k] = ib;
    x[k - nNaNs] = xwork[8 - k];
    x[8 - k] = xwork[k - nNaNs];
  }

  if (((nNaNs + 7) & 1) != 0) {
    x[(m - nNaNs) + 1] = xwork[(m - nNaNs) + 1];
  }

  if (1 - nNaNs > 1) {
    nPairs = (1 - nNaNs) >> 2;
    bLen = 4;
    while (nPairs > 1) {
      if ((nPairs & 1) != 0) {
        nPairs--;
        ib = bLen * nPairs;
        m = 1 - (nNaNs + ib);
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

    if (1 - nNaNs > bLen) {
      merge(idx, x, 0, bLen, 1 - (nNaNs + bLen));
    }
  }
}

//
// Arguments    : int n
//                double a
//                const double x[9]
//                int ix0
//                double y[3]
//                int iy0
// Return Type  : void
//
static void b_eml_xaxpy(int n, double a, const double x[9], int ix0, double y[3],
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
// Arguments    : const double x[3]
//                int ix0
// Return Type  : double
//
static double b_eml_xnrm2(const double x[3], int ix0)
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = ix0; k <= ix0 + 1; k++) {
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
// Arguments    : emxArray_boolean_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
static void b_emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
  numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
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
// Arguments    : const emxArray_uint8_T *varargin_1
//                const double varargin_2[2]
//                emxArray_real_T *B
// Return Type  : void
//
static void bwtraceboundary(const emxArray_uint8_T *varargin_1, const double
  varargin_2[2], emxArray_real_T *B)
{
  emxArray_boolean_T *BW;
  int i19;
  int loop_ub;
  int numRows;
  double maxNumPoints;
  emxArray_boolean_T *b;
  unsigned int sizeB[2];
  int currentCircIdx;
  int checkDir;
  emxArray_uint8_T *bwPadImage;
  double idx;
  double fVOffsets[8];
  double fOffsets[8];
  boolean_T tf;
  boolean_T exitg3;
  int fNextSearchDir;
  boolean_T exitg2;
  emxArray_real_T *fScratch;
  double numPixels;
  int currentPixel;
  int initDepartureDir;
  emxArray_real_T *b_fScratch;
  boolean_T foundNextPixel;
  boolean_T guard1 = false;
  boolean_T exitg1;
  static const signed char iv4[8] = { 2, 3, 4, 5, 6, 7, 8, 1 };

  static const signed char iv5[8] = { 8, 8, 2, 2, 4, 4, 6, 6 };

  emxArray_real_T *boundary;
  b_emxInit_boolean_T(&BW, 2);
  i19 = BW->size[0] * BW->size[1];
  BW->size[0] = varargin_1->size[0];
  BW->size[1] = varargin_1->size[1];
  emxEnsureCapacity((emxArray__common *)BW, i19, (int)sizeof(boolean_T));
  loop_ub = varargin_1->size[0] * varargin_1->size[1];
  for (i19 = 0; i19 < loop_ub; i19++) {
    BW->data[i19] = (varargin_1->data[i19] != 0);
  }

  numRows = BW->size[0];
  maxNumPoints = 2.0 * (double)BW->size[0] * (double)BW->size[1] + 2.0;
  b_emxInit_boolean_T(&b, 2);
  if ((BW->size[0] == 0) || (BW->size[1] == 0)) {
    for (i19 = 0; i19 < 2; i19++) {
      sizeB[i19] = BW->size[i19] + 2U;
    }

    i19 = b->size[0] * b->size[1];
    b->size[0] = (int)sizeB[0];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(boolean_T));
    i19 = b->size[0] * b->size[1];
    b->size[1] = (int)sizeB[1];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(boolean_T));
    loop_ub = (int)sizeB[0] * (int)sizeB[1];
    for (i19 = 0; i19 < loop_ub; i19++) {
      b->data[i19] = false;
    }
  } else {
    for (i19 = 0; i19 < 2; i19++) {
      sizeB[i19] = BW->size[i19] + 2U;
    }

    i19 = b->size[0] * b->size[1];
    b->size[0] = (int)sizeB[0];
    b->size[1] = (int)sizeB[1];
    emxEnsureCapacity((emxArray__common *)b, i19, (int)sizeof(boolean_T));
    for (loop_ub = 0; loop_ub < (int)sizeB[0]; loop_ub++) {
      b->data[loop_ub] = false;
    }

    i19 = b->size[1];
    for (currentCircIdx = BW->size[1] + 1; currentCircIdx + 1 <= i19;
         currentCircIdx++) {
      checkDir = b->size[0];
      for (loop_ub = 0; loop_ub < checkDir; loop_ub++) {
        b->data[loop_ub + b->size[0] * currentCircIdx] = false;
      }
    }

    for (currentCircIdx = 0; currentCircIdx < BW->size[1]; currentCircIdx++) {
      b->data[b->size[0] * (currentCircIdx + 1)] = false;
    }

    for (currentCircIdx = 0; currentCircIdx < BW->size[1]; currentCircIdx++) {
      i19 = b->size[0];
      for (loop_ub = BW->size[0] + 1; loop_ub + 1 <= i19; loop_ub++) {
        b->data[loop_ub + b->size[0] * (currentCircIdx + 1)] = false;
      }
    }

    for (currentCircIdx = 0; currentCircIdx < BW->size[1]; currentCircIdx++) {
      for (loop_ub = 0; loop_ub < BW->size[0]; loop_ub++) {
        b->data[(loop_ub + b->size[0] * (currentCircIdx + 1)) + 1] = BW->
          data[loop_ub + BW->size[0] * currentCircIdx];
      }
    }
  }

  emxInit_uint8_T(&bwPadImage, 2);
  i19 = bwPadImage->size[0] * bwPadImage->size[1];
  bwPadImage->size[0] = b->size[0];
  bwPadImage->size[1] = b->size[1];
  emxEnsureCapacity((emxArray__common *)bwPadImage, i19, (int)sizeof(unsigned
    char));
  loop_ub = b->size[0] * b->size[1];
  for (i19 = 0; i19 < loop_ub; i19++) {
    bwPadImage->data[i19] = b->data[i19];
  }

  emxFree_boolean_T(&b);
  idx = (varargin_2[1] * ((double)BW->size[0] + 2.0) + varargin_2[0]) + 1.0;
  fOffsets[0] = -1.0;
  fOffsets[1] = ((double)BW->size[0] + 2.0) - 1.0;
  fOffsets[2] = (double)BW->size[0] + 2.0;
  fOffsets[3] = ((double)BW->size[0] + 2.0) + 1.0;
  fOffsets[4] = 1.0;
  fOffsets[5] = -((double)BW->size[0] + 2.0) + 1.0;
  fOffsets[6] = -((double)BW->size[0] + 2.0);
  fOffsets[7] = -((double)BW->size[0] + 2.0) - 1.0;
  fVOffsets[0] = -1.0;
  fVOffsets[1] = (double)BW->size[0] + 2.0;
  fVOffsets[2] = 1.0;
  fVOffsets[3] = -((double)BW->size[0] + 2.0);
  tf = false;
  if (bwPadImage->data[(int)idx - 1] != 0) {
    loop_ub = 0;
    exitg3 = false;
    while ((!exitg3) && (loop_ub <= 3)) {
      if (!(bwPadImage->data[(int)(idx + fVOffsets[loop_ub]) - 1] != 0)) {
        tf = true;
        exitg3 = true;
      } else {
        loop_ub++;
      }
    }
  }

  if (tf) {
    fNextSearchDir = 0;
    loop_ub = 0;
    exitg2 = false;
    while ((!exitg2) && (loop_ub <= 7)) {
      currentCircIdx = (loop_ub - ((int)floor((6.0 + (double)loop_ub) / 8.0) <<
        3)) + 5;
      checkDir = currentCircIdx;
      if (currentCircIdx < 0) {
        checkDir = currentCircIdx + 8;
      }

      checkDir -= (int)floor((double)checkDir / 8.0) << 3;
      if (bwPadImage->data[(int)(idx + fOffsets[checkDir]) - 1] == 0) {
        fNextSearchDir = currentCircIdx + 2;
        exitg2 = true;
      } else {
        loop_ub++;
      }
    }

    b_emxInit_real_T(&fScratch, 1);
    numPixels = 1.0;
    i19 = fScratch->size[0];
    fScratch->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)fScratch, i19, (int)sizeof(double));
    fScratch->data[0] = idx;
    bwPadImage->data[(int)idx - 1] = 2;
    tf = false;
    currentPixel = (int)idx;
    initDepartureDir = -1;
    b_emxInit_real_T(&b_fScratch, 1);
    while (!tf) {
      currentCircIdx = fNextSearchDir;
      foundNextPixel = false;
      checkDir = 0;
      guard1 = false;
      exitg1 = false;
      while ((!exitg1) && (checkDir < 8)) {
        idx = (double)currentPixel + fOffsets[currentCircIdx - 1];
        if (bwPadImage->data[(int)idx - 1] != 0) {
          if ((bwPadImage->data[currentPixel - 1] == 2) && (initDepartureDir ==
               -1)) {
            initDepartureDir = currentCircIdx;
            guard1 = true;
          } else if ((bwPadImage->data[currentPixel - 1] == 2) &&
                     (initDepartureDir == currentCircIdx)) {
            tf = true;
            foundNextPixel = true;
          } else {
            guard1 = true;
          }

          exitg1 = true;
        } else {
          currentCircIdx = iv4[currentCircIdx - 1];
          checkDir++;
          guard1 = false;
        }
      }

      if (guard1) {
        fNextSearchDir = iv5[currentCircIdx - 1];
        foundNextPixel = true;
        numPixels++;
        currentCircIdx = fScratch->size[0];
        i19 = fScratch->size[0];
        fScratch->size[0] = currentCircIdx + 1;
        emxEnsureCapacity((emxArray__common *)fScratch, i19, (int)sizeof(double));
        fScratch->data[currentCircIdx] = idx;
        if (numPixels == maxNumPoints) {
          tf = true;
        } else {
          if (bwPadImage->data[(int)idx - 1] != 2) {
            bwPadImage->data[(int)idx - 1] = 3;
          }

          currentPixel = (int)idx;
        }
      }

      if (!foundNextPixel) {
        numPixels = 2.0;
        i19 = b_fScratch->size[0];
        b_fScratch->size[0] = fScratch->size[0] + fScratch->size[0];
        emxEnsureCapacity((emxArray__common *)b_fScratch, i19, (int)sizeof
                          (double));
        loop_ub = fScratch->size[0];
        for (i19 = 0; i19 < loop_ub; i19++) {
          b_fScratch->data[i19] = fScratch->data[i19];
        }

        loop_ub = fScratch->size[0];
        for (i19 = 0; i19 < loop_ub; i19++) {
          b_fScratch->data[i19 + fScratch->size[0]] = fScratch->data[i19];
        }

        i19 = fScratch->size[0];
        fScratch->size[0] = b_fScratch->size[0];
        emxEnsureCapacity((emxArray__common *)fScratch, i19, (int)sizeof(double));
        loop_ub = b_fScratch->size[0];
        for (i19 = 0; i19 < loop_ub; i19++) {
          fScratch->data[i19] = b_fScratch->data[i19];
        }

        tf = true;
      }
    }

    emxFree_real_T(&b_fScratch);
    emxInit_real_T(&boundary, 2);
    i19 = boundary->size[0] * boundary->size[1];
    boundary->size[0] = (int)numPixels;
    boundary->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)boundary, i19, (int)sizeof(double));
    for (loop_ub = 0; loop_ub < (int)numPixels; loop_ub++) {
      if (numRows + 2U == BW->size[0] + 2U) {
        idx = (fScratch->data[loop_ub] - 1.0) - floor((fScratch->data[loop_ub] -
          1.0) / ((double)numRows + 2.0)) * ((double)numRows + 2.0);
      } else {
        idx = (fScratch->data[loop_ub] - 1.0) / ((double)numRows + 2.0);
        if (fabs(idx - rt_roundd_snf(idx)) <= 2.2204460492503131E-16 * fabs(idx))
        {
          idx = 0.0;
        } else {
          idx = (idx - floor(idx)) * ((double)numRows + 2.0);
        }
      }

      boundary->data[loop_ub] = idx;
      boundary->data[(int)(numPixels + (1.0 + (double)loop_ub)) - 1] = floor
        ((fScratch->data[loop_ub] - 1.0) / ((double)numRows + 2.0));
    }

    emxFree_real_T(&fScratch);
    i19 = B->size[0] * B->size[1];
    B->size[0] = boundary->size[0];
    B->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)B, i19, (int)sizeof(double));
    loop_ub = boundary->size[0] * boundary->size[1];
    for (i19 = 0; i19 < loop_ub; i19++) {
      B->data[i19] = boundary->data[i19];
    }

    emxFree_real_T(&boundary);
  } else {
    i19 = B->size[0] * B->size[1];
    B->size[0] = 0;
    B->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)B, i19, (int)sizeof(double));
  }

  emxFree_uint8_T(&bwPadImage);
  emxFree_boolean_T(&BW);
}

//
// Arguments    : d_vision_internal_calibration_C *b_this
//                const double varargin_1_RadialDistortion[3]
//                const double varargin_1_TangentialDistortion[2]
//                const char varargin_1_WorldUnits[2]
//                double c_varargin_1_NumRadialDistortio
//                const double varargin_1_IntrinsicMatrix[9]
// Return Type  : d_vision_internal_calibration_C *
//
static d_vision_internal_calibration_C *c_CameraParametersImpl_CameraPa
  (d_vision_internal_calibration_C *b_this, const double
   varargin_1_RadialDistortion[3], const double varargin_1_TangentialDistortion
   [2], const char varargin_1_WorldUnits[2], double
   c_varargin_1_NumRadialDistortio, const double varargin_1_IntrinsicMatrix[9])
{
  d_vision_internal_calibration_C *c_this;
  d_vision_internal_calibration_C *d_this;
  int i4;
  int i5;
  c_vision_internal_calibration_I *e_this;
  static const char cv6[5] = { 'u', 'i', 'n', 't', '8' };

  static const char cv7[4] = { 's', 'a', 'm', 'e' };

  c_this = b_this;
  d_this = c_this;
  for (i4 = 0; i4 < 3; i4++) {
    for (i5 = 0; i5 < 3; i5++) {
      d_this->IntrinsicMatrixInternal[i5 + 3 * i4] =
        varargin_1_IntrinsicMatrix[i4 + 3 * i5];
    }
  }

  for (i4 = 0; i4 < 3; i4++) {
    d_this->RadialDistortion[i4] = varargin_1_RadialDistortion[i4];
  }

  for (i4 = 0; i4 < 2; i4++) {
    d_this->TangentialDistortion[i4] = varargin_1_TangentialDistortion[i4];
  }

  for (i4 = 0; i4 < 2; i4++) {
    d_this->WorldUnits[i4] = varargin_1_WorldUnits[i4];
  }

  d_this->NumRadialDistortionCoefficients = c_varargin_1_NumRadialDistortio;
  e_this = &c_this->UndistortMap;
  for (i4 = 0; i4 < 2; i4++) {
    e_this->XBounds[i4] = -1.0;
  }

  for (i4 = 0; i4 < 2; i4++) {
    e_this->YBounds[i4] = -1.0;
  }

  i4 = e_this->SizeOfImage->size[0] * e_this->SizeOfImage->size[1];
  e_this->SizeOfImage->size[0] = 1;
  e_this->SizeOfImage->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)e_this->SizeOfImage, i4, (int)sizeof
                    (double));
  for (i4 = 0; i4 < 2; i4++) {
    e_this->SizeOfImage->data[i4] = 0.0;
  }

  i4 = e_this->SizeOfImage->size[0] * e_this->SizeOfImage->size[1];
  e_this->SizeOfImage->size[0] = 1;
  e_this->SizeOfImage->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)e_this->SizeOfImage, i4, (int)sizeof
                    (double));
  for (i4 = 0; i4 < 3; i4++) {
    e_this->SizeOfImage->data[i4] = 0.0;
  }

  i4 = e_this->ClassOfImage->size[0] * e_this->ClassOfImage->size[1];
  e_this->ClassOfImage->size[0] = 1;
  e_this->ClassOfImage->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)e_this->ClassOfImage, i4, (int)sizeof
                    (char));
  e_this->ClassOfImage->data[0] = 'a';
  i4 = e_this->ClassOfImage->size[0] * e_this->ClassOfImage->size[1];
  e_this->ClassOfImage->size[0] = 1;
  e_this->ClassOfImage->size[1] = 5;
  emxEnsureCapacity((emxArray__common *)e_this->ClassOfImage, i4, (int)sizeof
                    (char));
  for (i4 = 0; i4 < 5; i4++) {
    e_this->ClassOfImage->data[i4] = cv6[i4];
  }

  i4 = e_this->OutputView->size[0] * e_this->OutputView->size[1];
  e_this->OutputView->size[0] = 1;
  e_this->OutputView->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)e_this->OutputView, i4, (int)sizeof(char));
  e_this->OutputView->data[0] = 'a';
  i4 = e_this->OutputView->size[0] * e_this->OutputView->size[1];
  e_this->OutputView->size[0] = 1;
  e_this->OutputView->size[1] = 4;
  emxEnsureCapacity((emxArray__common *)e_this->OutputView, i4, (int)sizeof(char));
  for (i4 = 0; i4 < 4; i4++) {
    e_this->OutputView->data[i4] = cv7[i4];
  }

  i4 = e_this->XmapSingle->size[0] * e_this->XmapSingle->size[1];
  e_this->XmapSingle->size[0] = 2;
  e_this->XmapSingle->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)e_this->XmapSingle, i4, (int)sizeof
                    (float));
  for (i4 = 0; i4 < 4; i4++) {
    e_this->XmapSingle->data[i4] = 0.0F;
  }

  i4 = e_this->YmapSingle->size[0] * e_this->YmapSingle->size[1];
  e_this->YmapSingle->size[0] = 2;
  e_this->YmapSingle->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)e_this->YmapSingle, i4, (int)sizeof
                    (float));
  for (i4 = 0; i4 < 4; i4++) {
    e_this->YmapSingle->data[i4] = 0.0F;
  }

  return c_this;
}

//
// Arguments    : c_vision_internal_calibration_C *b_this
//                double xBounds[2]
//                double yBounds[2]
// Return Type  : void
//
static void c_CameraParametersImpl_computeU(c_vision_internal_calibration_C
  *b_this, double xBounds[2], double yBounds[2])
{
  emxArray_uint8_T *undistortedMask;
  double yBoundsBig[2];
  double xBoundsBig[2];
  c_vision_internal_calibration_C *c_this;
  int sRow;
  int sCol;
  int cx;
  int i16;
  int i17;
  int i;
  boolean_T exitg5;
  double b_i;
  emxArray_real_T *boundaryPixelsUndistorted;
  emxArray_real_T *b_boundaryPixelsUndistorted;
  double b_sRow[2];
  emxArray_real_T *c_boundaryPixelsUndistorted;
  emxArray_real_T *d_boundaryPixelsUndistorted;
  double intrinsicMatrix[9];
  emxArray_real_T *boundaryPixelsDistorted;
  emxArray_real_T *b_boundaryPixelsDistorted;
  emxArray_boolean_T *topIdx;
  emxArray_real_T *r0;
  emxArray_real_T *c_boundaryPixelsDistorted;
  emxArray_boolean_T *botIdx;
  emxArray_real_T *d_boundaryPixelsDistorted;
  emxArray_boolean_T *leftIdx;
  emxArray_real_T *e_boundaryPixelsDistorted;
  emxArray_boolean_T *rightIdx;
  emxArray_int32_T *r1;
  boolean_T exitg4;
  double mtmp;
  boolean_T exitg3;
  double b_mtmp;
  boolean_T exitg2;
  double c_mtmp;
  boolean_T exitg1;
  emxInit_uint8_T(&undistortedMask, 2);
  c_CameraParametersImpl_createUn(b_this, undistortedMask, xBoundsBig,
    yBoundsBig);
  c_this = b_this;
  sRow = -1;
  sCol = -1;
  cx = (int)floor((double)undistortedMask->size[1] / 2.0);
  i16 = (int)floor((double)undistortedMask->size[0] / 2.0);
  i17 = (int)((double)undistortedMask->size[0] + (1.0 - (double)i16));
  i = 0;
  exitg5 = false;
  while ((!exitg5) && (i <= i17 - 1)) {
    b_i = (double)i16 + (double)i;
    if (undistortedMask->data[((int)b_i + undistortedMask->size[0] * (cx - 1)) -
        1] == 0) {
      sRow = (int)b_i - 1;
      sCol = cx;
      exitg5 = true;
    } else {
      i++;
    }
  }

  emxInit_real_T(&boundaryPixelsUndistorted, 2);
  emxInit_real_T(&b_boundaryPixelsUndistorted, 2);
  b_sRow[0] = sRow;
  b_sRow[1] = sCol;
  bwtraceboundary(undistortedMask, b_sRow, boundaryPixelsUndistorted);
  sRow = boundaryPixelsUndistorted->size[0];
  i16 = b_boundaryPixelsUndistorted->size[0] * b_boundaryPixelsUndistorted->
    size[1];
  b_boundaryPixelsUndistorted->size[0] = sRow;
  b_boundaryPixelsUndistorted->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b_boundaryPixelsUndistorted, i16, (int)
                    sizeof(double));
  emxFree_uint8_T(&undistortedMask);
  for (i16 = 0; i16 < 2; i16++) {
    for (i17 = 0; i17 < sRow; i17++) {
      b_boundaryPixelsUndistorted->data[i17 + b_boundaryPixelsUndistorted->size
        [0] * i16] = boundaryPixelsUndistorted->data[i17 +
        boundaryPixelsUndistorted->size[0] * (1 - i16)];
    }
  }

  b_emxInit_real_T(&c_boundaryPixelsUndistorted, 1);
  sRow = boundaryPixelsUndistorted->size[0];
  i16 = c_boundaryPixelsUndistorted->size[0];
  c_boundaryPixelsUndistorted->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)c_boundaryPixelsUndistorted, i16, (int)
                    sizeof(double));
  emxFree_real_T(&boundaryPixelsUndistorted);
  for (i16 = 0; i16 < sRow; i16++) {
    c_boundaryPixelsUndistorted->data[i16] = b_boundaryPixelsUndistorted->
      data[i16] + -750.0;
  }

  sRow = c_boundaryPixelsUndistorted->size[0];
  for (i16 = 0; i16 < sRow; i16++) {
    b_boundaryPixelsUndistorted->data[i16] = c_boundaryPixelsUndistorted->
      data[i16];
  }

  emxFree_real_T(&c_boundaryPixelsUndistorted);
  b_emxInit_real_T(&d_boundaryPixelsUndistorted, 1);
  sRow = b_boundaryPixelsUndistorted->size[0];
  i16 = d_boundaryPixelsUndistorted->size[0];
  d_boundaryPixelsUndistorted->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)d_boundaryPixelsUndistorted, i16, (int)
                    sizeof(double));
  for (i16 = 0; i16 < sRow; i16++) {
    d_boundaryPixelsUndistorted->data[i16] = b_boundaryPixelsUndistorted->
      data[i16 + b_boundaryPixelsUndistorted->size[0]] + -478.0;
  }

  sRow = d_boundaryPixelsUndistorted->size[0];
  for (i16 = 0; i16 < sRow; i16++) {
    b_boundaryPixelsUndistorted->data[i16 + b_boundaryPixelsUndistorted->size[0]]
      = d_boundaryPixelsUndistorted->data[i16];
  }

  emxFree_real_T(&d_boundaryPixelsUndistorted);
  for (i16 = 0; i16 < 3; i16++) {
    for (i17 = 0; i17 < 3; i17++) {
      intrinsicMatrix[i17 + 3 * i16] = c_this->IntrinsicMatrixInternal[i16 + 3 *
        i17];
    }
  }

  emxInit_real_T(&boundaryPixelsDistorted, 2);
  b_emxInit_real_T(&b_boundaryPixelsDistorted, 1);
  distortPoints(b_boundaryPixelsUndistorted, intrinsicMatrix,
                c_this->RadialDistortion, c_this->TangentialDistortion,
                boundaryPixelsDistorted);
  sRow = boundaryPixelsDistorted->size[0];
  i16 = b_boundaryPixelsDistorted->size[0];
  b_boundaryPixelsDistorted->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)b_boundaryPixelsDistorted, i16, (int)
                    sizeof(double));
  for (i16 = 0; i16 < sRow; i16++) {
    b_boundaryPixelsDistorted->data[i16] = boundaryPixelsDistorted->data[i16 +
      boundaryPixelsDistorted->size[0]] - 1.0;
  }

  emxInit_boolean_T(&topIdx, 1);
  b_emxInit_real_T(&r0, 1);
  b_abs(b_boundaryPixelsDistorted, r0);
  i16 = topIdx->size[0];
  topIdx->size[0] = r0->size[0];
  emxEnsureCapacity((emxArray__common *)topIdx, i16, (int)sizeof(boolean_T));
  sRow = r0->size[0];
  emxFree_real_T(&b_boundaryPixelsDistorted);
  for (i16 = 0; i16 < sRow; i16++) {
    topIdx->data[i16] = (r0->data[i16] < 7.0);
  }

  b_emxInit_real_T(&c_boundaryPixelsDistorted, 1);
  sRow = boundaryPixelsDistorted->size[0];
  i16 = c_boundaryPixelsDistorted->size[0];
  c_boundaryPixelsDistorted->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)c_boundaryPixelsDistorted, i16, (int)
                    sizeof(double));
  for (i16 = 0; i16 < sRow; i16++) {
    c_boundaryPixelsDistorted->data[i16] = boundaryPixelsDistorted->data[i16 +
      boundaryPixelsDistorted->size[0]] - 480.0;
  }

  emxInit_boolean_T(&botIdx, 1);
  b_abs(c_boundaryPixelsDistorted, r0);
  i16 = botIdx->size[0];
  botIdx->size[0] = r0->size[0];
  emxEnsureCapacity((emxArray__common *)botIdx, i16, (int)sizeof(boolean_T));
  sRow = r0->size[0];
  emxFree_real_T(&c_boundaryPixelsDistorted);
  for (i16 = 0; i16 < sRow; i16++) {
    botIdx->data[i16] = (r0->data[i16] < 7.0);
  }

  b_emxInit_real_T(&d_boundaryPixelsDistorted, 1);
  sRow = boundaryPixelsDistorted->size[0];
  i16 = d_boundaryPixelsDistorted->size[0];
  d_boundaryPixelsDistorted->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)d_boundaryPixelsDistorted, i16, (int)
                    sizeof(double));
  for (i16 = 0; i16 < sRow; i16++) {
    d_boundaryPixelsDistorted->data[i16] = boundaryPixelsDistorted->data[i16] -
      1.0;
  }

  emxInit_boolean_T(&leftIdx, 1);
  b_abs(d_boundaryPixelsDistorted, r0);
  i16 = leftIdx->size[0];
  leftIdx->size[0] = r0->size[0];
  emxEnsureCapacity((emxArray__common *)leftIdx, i16, (int)sizeof(boolean_T));
  sRow = r0->size[0];
  emxFree_real_T(&d_boundaryPixelsDistorted);
  for (i16 = 0; i16 < sRow; i16++) {
    leftIdx->data[i16] = (r0->data[i16] < 7.0);
  }

  b_emxInit_real_T(&e_boundaryPixelsDistorted, 1);
  sRow = boundaryPixelsDistorted->size[0];
  i16 = e_boundaryPixelsDistorted->size[0];
  e_boundaryPixelsDistorted->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)e_boundaryPixelsDistorted, i16, (int)
                    sizeof(double));
  for (i16 = 0; i16 < sRow; i16++) {
    e_boundaryPixelsDistorted->data[i16] = boundaryPixelsDistorted->data[i16] -
      752.0;
  }

  emxFree_real_T(&boundaryPixelsDistorted);
  emxInit_boolean_T(&rightIdx, 1);
  b_abs(e_boundaryPixelsDistorted, r0);
  i16 = rightIdx->size[0];
  rightIdx->size[0] = r0->size[0];
  emxEnsureCapacity((emxArray__common *)rightIdx, i16, (int)sizeof(boolean_T));
  sRow = r0->size[0];
  emxFree_real_T(&e_boundaryPixelsDistorted);
  for (i16 = 0; i16 < sRow; i16++) {
    rightIdx->data[i16] = (r0->data[i16] < 7.0);
  }

  emxFree_real_T(&r0);
  emxInit_int32_T(&r1, 1);
  sCol = topIdx->size[0] - 1;
  sRow = 0;
  for (i = 0; i <= sCol; i++) {
    if (topIdx->data[i]) {
      sRow++;
    }
  }

  i16 = r1->size[0];
  r1->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)r1, i16, (int)sizeof(int));
  sRow = 0;
  for (i = 0; i <= sCol; i++) {
    if (topIdx->data[i]) {
      r1->data[sRow] = i + 1;
      sRow++;
    }
  }

  emxFree_boolean_T(&topIdx);
  sRow = 1;
  sCol = r1->size[0];
  b_i = b_boundaryPixelsUndistorted->data[(r1->data[0] +
    b_boundaryPixelsUndistorted->size[0]) - 1];
  if (r1->size[0] > 1) {
    if (rtIsNaN(b_i)) {
      cx = 2;
      exitg4 = false;
      while ((!exitg4) && (cx <= sCol)) {
        sRow = cx;
        if (!rtIsNaN(b_boundaryPixelsUndistorted->data[(r1->data[cx - 1] +
              b_boundaryPixelsUndistorted->size[0]) - 1])) {
          b_i = b_boundaryPixelsUndistorted->data[(r1->data[cx - 1] +
            b_boundaryPixelsUndistorted->size[0]) - 1];
          exitg4 = true;
        } else {
          cx++;
        }
      }
    }

    if (sRow < r1->size[0]) {
      while (sRow + 1 <= sCol) {
        if (b_boundaryPixelsUndistorted->data[(r1->data[sRow] +
             b_boundaryPixelsUndistorted->size[0]) - 1] > b_i) {
          b_i = b_boundaryPixelsUndistorted->data[(r1->data[sRow] +
            b_boundaryPixelsUndistorted->size[0]) - 1];
        }

        sRow++;
      }
    }
  }

  sCol = botIdx->size[0] - 1;
  sRow = 0;
  for (i = 0; i <= sCol; i++) {
    if (botIdx->data[i]) {
      sRow++;
    }
  }

  i16 = r1->size[0];
  r1->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)r1, i16, (int)sizeof(int));
  sRow = 0;
  for (i = 0; i <= sCol; i++) {
    if (botIdx->data[i]) {
      r1->data[sRow] = i + 1;
      sRow++;
    }
  }

  emxFree_boolean_T(&botIdx);
  sRow = 1;
  sCol = r1->size[0];
  mtmp = b_boundaryPixelsUndistorted->data[(r1->data[0] +
    b_boundaryPixelsUndistorted->size[0]) - 1];
  if (r1->size[0] > 1) {
    if (rtIsNaN(mtmp)) {
      cx = 2;
      exitg3 = false;
      while ((!exitg3) && (cx <= sCol)) {
        sRow = cx;
        if (!rtIsNaN(b_boundaryPixelsUndistorted->data[(r1->data[cx - 1] +
              b_boundaryPixelsUndistorted->size[0]) - 1])) {
          mtmp = b_boundaryPixelsUndistorted->data[(r1->data[cx - 1] +
            b_boundaryPixelsUndistorted->size[0]) - 1];
          exitg3 = true;
        } else {
          cx++;
        }
      }
    }

    if (sRow < r1->size[0]) {
      while (sRow + 1 <= sCol) {
        if (b_boundaryPixelsUndistorted->data[(r1->data[sRow] +
             b_boundaryPixelsUndistorted->size[0]) - 1] < mtmp) {
          mtmp = b_boundaryPixelsUndistorted->data[(r1->data[sRow] +
            b_boundaryPixelsUndistorted->size[0]) - 1];
        }

        sRow++;
      }
    }
  }

  sCol = leftIdx->size[0] - 1;
  sRow = 0;
  for (i = 0; i <= sCol; i++) {
    if (leftIdx->data[i]) {
      sRow++;
    }
  }

  i16 = r1->size[0];
  r1->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)r1, i16, (int)sizeof(int));
  sRow = 0;
  for (i = 0; i <= sCol; i++) {
    if (leftIdx->data[i]) {
      r1->data[sRow] = i + 1;
      sRow++;
    }
  }

  emxFree_boolean_T(&leftIdx);
  sRow = 1;
  sCol = r1->size[0];
  b_mtmp = b_boundaryPixelsUndistorted->data[r1->data[0] - 1];
  if (r1->size[0] > 1) {
    if (rtIsNaN(b_mtmp)) {
      cx = 2;
      exitg2 = false;
      while ((!exitg2) && (cx <= sCol)) {
        sRow = cx;
        if (!rtIsNaN(b_boundaryPixelsUndistorted->data[r1->data[cx - 1] - 1])) {
          b_mtmp = b_boundaryPixelsUndistorted->data[r1->data[cx - 1] - 1];
          exitg2 = true;
        } else {
          cx++;
        }
      }
    }

    if (sRow < r1->size[0]) {
      while (sRow + 1 <= sCol) {
        if (b_boundaryPixelsUndistorted->data[r1->data[sRow] - 1] > b_mtmp) {
          b_mtmp = b_boundaryPixelsUndistorted->data[r1->data[sRow] - 1];
        }

        sRow++;
      }
    }
  }

  sCol = rightIdx->size[0] - 1;
  sRow = 0;
  for (i = 0; i <= sCol; i++) {
    if (rightIdx->data[i]) {
      sRow++;
    }
  }

  i16 = r1->size[0];
  r1->size[0] = sRow;
  emxEnsureCapacity((emxArray__common *)r1, i16, (int)sizeof(int));
  sRow = 0;
  for (i = 0; i <= sCol; i++) {
    if (rightIdx->data[i]) {
      r1->data[sRow] = i + 1;
      sRow++;
    }
  }

  emxFree_boolean_T(&rightIdx);
  sRow = 1;
  sCol = r1->size[0];
  c_mtmp = b_boundaryPixelsUndistorted->data[r1->data[0] - 1];
  if (r1->size[0] > 1) {
    if (rtIsNaN(c_mtmp)) {
      cx = 2;
      exitg1 = false;
      while ((!exitg1) && (cx <= sCol)) {
        sRow = cx;
        if (!rtIsNaN(b_boundaryPixelsUndistorted->data[r1->data[cx - 1] - 1])) {
          c_mtmp = b_boundaryPixelsUndistorted->data[r1->data[cx - 1] - 1];
          exitg1 = true;
        } else {
          cx++;
        }
      }
    }

    if (sRow < r1->size[0]) {
      while (sRow + 1 <= sCol) {
        if (b_boundaryPixelsUndistorted->data[r1->data[sRow] - 1] < c_mtmp) {
          c_mtmp = b_boundaryPixelsUndistorted->data[r1->data[sRow] - 1];
        }

        sRow++;
      }
    }
  }

  emxFree_int32_T(&r1);
  emxFree_real_T(&b_boundaryPixelsUndistorted);
  xBounds[0] = ceil(b_mtmp);
  xBounds[1] = floor(c_mtmp);
  eml_sort(xBounds);
  yBounds[0] = ceil(b_i);
  yBounds[1] = floor(mtmp);
  eml_sort(yBounds);
}

//
// Arguments    : c_vision_internal_calibration_C *b_this
//                emxArray_uint8_T *undistortedMask
//                double xBoundsBig[2]
//                double yBoundsBig[2]
// Return Type  : void
//
static void c_CameraParametersImpl_createUn(c_vision_internal_calibration_C
  *b_this, emxArray_uint8_T *undistortedMask, double xBoundsBig[2], double
  yBoundsBig[2])
{
  c_vision_internal_calibration_I myMap;
  int i;
  c_vision_internal_calibration_C *c_this;
  double intrinsicMatrix[9];
  int j;
  double radialDist[3];
  double tangentialDist[2];
  static const char cv16[5] = { 'u', 'i', 'n', 't', '8' };

  static const char outputView[5] = { 'v', 'a', 'l', 'i', 'd' };

  static unsigned char inputImage[368388];
  emxArray_real32_T *X;
  emxArray_real32_T *Y;
  unsigned char fillValues;
  double b_undistortedMask[2];
  d_emxInitStruct_vision_internal(&myMap);
  xBoundsBig[0] = -750.0;
  xBoundsBig[1] = 1503.0;
  yBoundsBig[0] = -478.0;
  yBoundsBig[1] = 959.0;
  i = myMap.XmapSingle->size[0] * myMap.XmapSingle->size[1];
  myMap.XmapSingle->size[0] = 2;
  myMap.XmapSingle->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)myMap.XmapSingle, i, (int)sizeof(float));
  for (i = 0; i < 4; i++) {
    myMap.XmapSingle->data[i] = 0.0F;
  }

  i = myMap.YmapSingle->size[0] * myMap.YmapSingle->size[1];
  myMap.YmapSingle->size[0] = 2;
  myMap.YmapSingle->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)myMap.YmapSingle, i, (int)sizeof(float));
  for (i = 0; i < 4; i++) {
    myMap.YmapSingle->data[i] = 0.0F;
  }

  c_this = b_this;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      intrinsicMatrix[j + 3 * i] = c_this->IntrinsicMatrixInternal[i + 3 * j];
    }
  }

  for (i = 0; i < 3; i++) {
    radialDist[i] = b_this->RadialDistortion[i];
  }

  for (i = 0; i < 2; i++) {
    tangentialDist[i] = b_this->TangentialDistortion[i];
  }

  i = myMap.SizeOfImage->size[0] * myMap.SizeOfImage->size[1];
  myMap.SizeOfImage->size[0] = 1;
  myMap.SizeOfImage->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)myMap.SizeOfImage, i, (int)sizeof(double));
  for (i = 0; i < 2; i++) {
    myMap.SizeOfImage->data[i] = 480.0 + 272.0 * (double)i;
  }

  i = myMap.ClassOfImage->size[0] * myMap.ClassOfImage->size[1];
  myMap.ClassOfImage->size[0] = 1;
  myMap.ClassOfImage->size[1] = 5;
  emxEnsureCapacity((emxArray__common *)myMap.ClassOfImage, i, (int)sizeof(char));
  for (i = 0; i < 5; i++) {
    myMap.ClassOfImage->data[i] = cv16[i];
  }

  i = myMap.OutputView->size[0] * myMap.OutputView->size[1];
  myMap.OutputView->size[0] = 1;
  myMap.OutputView->size[1] = 5;
  emxEnsureCapacity((emxArray__common *)myMap.OutputView, i, (int)sizeof(char));
  for (i = 0; i < 5; i++) {
    myMap.OutputView->data[i] = outputView[i];
  }

  for (i = 0; i < 2; i++) {
    myMap.XBounds[i] = -750.0 + 2253.0 * (double)i;
  }

  for (i = 0; i < 2; i++) {
    myMap.YBounds[i] = -478.0 + 1437.0 * (double)i;
  }

  c_ImageTransformer_computeMap(&myMap, intrinsicMatrix, radialDist,
    tangentialDist);
  for (j = 0; j < 3; j++) {
    memset(&inputImage[486 * j], 0, 486U * sizeof(unsigned char));
  }

  for (j = 0; j < 3; j++) {
    memset(&inputImage[486 * (j + 755)], 0, 486U * sizeof(unsigned char));
  }

  for (j = 0; j < 752; j++) {
    for (i = 0; i < 3; i++) {
      inputImage[i + 486 * (j + 3)] = 0;
    }

    for (i = 0; i < 3; i++) {
      inputImage[(i + 486 * (j + 3)) + 483] = 0;
    }

    memset(&inputImage[3 + 486 * (j + 3)], 1, 480U * sizeof(unsigned char));
  }

  emxInit_real32_T(&X, 2);
  i = X->size[0] * X->size[1];
  X->size[0] = myMap.XmapSingle->size[0];
  X->size[1] = myMap.XmapSingle->size[1];
  emxEnsureCapacity((emxArray__common *)X, i, (int)sizeof(float));
  j = myMap.XmapSingle->size[0] * myMap.XmapSingle->size[1];
  for (i = 0; i < j; i++) {
    X->data[i] = (myMap.XmapSingle->data[i] + 3.0F) - 1.0F;
  }

  emxInit_real32_T(&Y, 2);
  i = undistortedMask->size[0] * undistortedMask->size[1];
  undistortedMask->size[0] = X->size[0];
  undistortedMask->size[1] = X->size[1];
  emxEnsureCapacity((emxArray__common *)undistortedMask, i, (int)sizeof(unsigned
    char));
  i = Y->size[0] * Y->size[1];
  Y->size[0] = myMap.YmapSingle->size[0];
  Y->size[1] = myMap.YmapSingle->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i, (int)sizeof(float));
  j = myMap.YmapSingle->size[0] * myMap.YmapSingle->size[1];
  for (i = 0; i < j; i++) {
    Y->data[i] = (myMap.YmapSingle->data[i] + 3.0F) - 1.0F;
  }

  c_emxFreeStruct_vision_internal(&myMap);
  fillValues = 0;
  for (i = 0; i < 2; i++) {
    tangentialDist[i] = 486.0 + 272.0 * (double)i;
  }

  for (i = 0; i < 2; i++) {
    b_undistortedMask[i] = undistortedMask->size[i];
  }

  remaptbb_uint8(inputImage, tangentialDist, 2.0, &Y->data[0], &X->data[0], 1,
                 &fillValues, &undistortedMask->data[0], b_undistortedMask,
                 (double)(undistortedMask->size[0] * undistortedMask->size[1]));
  emxFree_real32_T(&Y);
  emxFree_real32_T(&X);
}

//
// Arguments    : c_vision_internal_calibration_I *b_this
//                const double intrinsicMatrix[9]
//                const double radialDist[3]
//                const double tangentialDist[2]
// Return Type  : void
//
static void c_ImageTransformer_computeMap(c_vision_internal_calibration_I
  *b_this, const double intrinsicMatrix[9], const double radialDist[3], const
  double tangentialDist[2])
{
  double n;
  double m;
  int b_n;
  double apnd;
  double ndbl;
  double cdiff;
  double absa;
  double absb;
  emxArray_real_T *y;
  int i18;
  int nm1d2;
  int k;
  emxArray_real_T *b_y;
  emxArray_real_T *X;
  emxArray_real_T *Y;
  emxArray_real_T *b_X;
  emxArray_real_T *c_X;
  emxArray_real_T *b_Y;
  emxArray_real_T *ptsOut;
  emxArray_char_T *a;
  boolean_T b_bool;
  int32_T exitg1;
  static const char cv17[6] = { 'd', 'o', 'u', 'b', 'l', 'e' };

  double varargin_1[2];
  int iv3[2];
  n = b_this->XBounds[0];
  m = b_this->XBounds[1];
  if (rtIsNaN(n) || rtIsNaN(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else if (m < n) {
    b_n = -1;
    apnd = m;
  } else if (rtIsInf(n) || rtIsInf(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else {
    ndbl = floor((m - n) + 0.5);
    apnd = n + ndbl;
    cdiff = apnd - m;
    absa = fabs(n);
    absb = fabs(m);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = m;
    } else if (cdiff > 0.0) {
      apnd = n + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      b_n = (int)ndbl - 1;
    } else {
      b_n = -1;
    }
  }

  emxInit_real_T(&y, 2);
  i18 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = b_n + 1;
  emxEnsureCapacity((emxArray__common *)y, i18, (int)sizeof(double));
  if (b_n + 1 > 0) {
    y->data[0] = n;
    if (b_n + 1 > 1) {
      y->data[b_n] = apnd;
      nm1d2 = b_n / 2;
      for (k = 1; k < nm1d2; k++) {
        y->data[k] = n + (double)k;
        y->data[b_n - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == b_n) {
        y->data[nm1d2] = (n + apnd) / 2.0;
      } else {
        y->data[nm1d2] = n + (double)nm1d2;
        y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  n = b_this->YBounds[0];
  m = b_this->YBounds[1];
  if (rtIsNaN(n) || rtIsNaN(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else if (m < n) {
    b_n = -1;
    apnd = m;
  } else if (rtIsInf(n) || rtIsInf(m)) {
    b_n = 0;
    n = rtNaN;
    apnd = m;
  } else {
    ndbl = floor((m - n) + 0.5);
    apnd = n + ndbl;
    cdiff = apnd - m;
    absa = fabs(n);
    absb = fabs(m);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = m;
    } else if (cdiff > 0.0) {
      apnd = n + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      b_n = (int)ndbl - 1;
    } else {
      b_n = -1;
    }
  }

  emxInit_real_T(&b_y, 2);
  i18 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = b_n + 1;
  emxEnsureCapacity((emxArray__common *)b_y, i18, (int)sizeof(double));
  if (b_n + 1 > 0) {
    b_y->data[0] = n;
    if (b_n + 1 > 1) {
      b_y->data[b_n] = apnd;
      nm1d2 = b_n / 2;
      for (k = 1; k < nm1d2; k++) {
        b_y->data[k] = n + (double)k;
        b_y->data[b_n - k] = apnd - (double)k;
      }

      if (nm1d2 << 1 == b_n) {
        b_y->data[nm1d2] = (n + apnd) / 2.0;
      } else {
        b_y->data[nm1d2] = n + (double)nm1d2;
        b_y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  emxInit_real_T(&X, 2);
  emxInit_real_T(&Y, 2);
  emxInit_real_T(&b_X, 2);
  b_emxInit_real_T(&c_X, 1);
  meshgrid(y, b_y, X, Y);
  nm1d2 = X->size[0] * X->size[1];
  k = X->size[0] * X->size[1];
  i18 = c_X->size[0];
  c_X->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)c_X, i18, (int)sizeof(double));
  emxFree_real_T(&b_y);
  emxFree_real_T(&y);
  for (i18 = 0; i18 < nm1d2; i18++) {
    c_X->data[i18] = X->data[i18];
  }

  b_emxInit_real_T(&b_Y, 1);
  nm1d2 = Y->size[0] * Y->size[1];
  b_n = Y->size[0] * Y->size[1];
  i18 = b_Y->size[0];
  b_Y->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)b_Y, i18, (int)sizeof(double));
  for (i18 = 0; i18 < nm1d2; i18++) {
    b_Y->data[i18] = Y->data[i18];
  }

  emxFree_real_T(&Y);
  i18 = b_X->size[0] * b_X->size[1];
  b_X->size[0] = k;
  b_X->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b_X, i18, (int)sizeof(double));
  for (i18 = 0; i18 < k; i18++) {
    b_X->data[i18] = c_X->data[i18];
  }

  emxFree_real_T(&c_X);
  for (i18 = 0; i18 < b_n; i18++) {
    b_X->data[i18 + b_X->size[0]] = b_Y->data[i18];
  }

  emxFree_real_T(&b_Y);
  emxInit_real_T(&ptsOut, 2);
  emxInit_char_T(&a, 2);
  distortPoints(b_X, intrinsicMatrix, radialDist, tangentialDist, ptsOut);
  m = (b_this->YBounds[1] - b_this->YBounds[0]) + 1.0;
  n = (b_this->XBounds[1] - b_this->XBounds[0]) + 1.0;
  i18 = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = b_this->ClassOfImage->size[1];
  emxEnsureCapacity((emxArray__common *)a, i18, (int)sizeof(char));
  nm1d2 = b_this->ClassOfImage->size[0] * b_this->ClassOfImage->size[1];
  emxFree_real_T(&b_X);
  for (i18 = 0; i18 < nm1d2; i18++) {
    a->data[i18] = b_this->ClassOfImage->data[i18];
  }

  b_bool = false;
  if (a->size[1] != 6) {
  } else {
    k = 0;
    do {
      exitg1 = 0;
      if (k <= 5) {
        if (a->data[k] != cv17[k]) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  emxFree_char_T(&a);
  if (b_bool) {
  } else {
    varargin_1[0] = m;
    varargin_1[1] = n;
    i18 = ptsOut->size[0];
    for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
      iv3[nm1d2] = (int)varargin_1[nm1d2];
    }

    nm1d2 = X->size[0] * X->size[1];
    X->size[0] = iv3[0];
    X->size[1] = iv3[1];
    emxEnsureCapacity((emxArray__common *)X, nm1d2, (int)sizeof(double));
    for (k = 0; k + 1 <= i18; k++) {
      X->data[k] = ptsOut->data[k];
    }

    i18 = b_this->XmapSingle->size[0] * b_this->XmapSingle->size[1];
    b_this->XmapSingle->size[0] = X->size[0];
    b_this->XmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->XmapSingle, i18, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i18 = 0; i18 < nm1d2; i18++) {
      b_this->XmapSingle->data[i18] = (float)X->data[i18];
    }

    varargin_1[0] = m;
    varargin_1[1] = n;
    i18 = ptsOut->size[0];
    for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
      iv3[nm1d2] = (int)varargin_1[nm1d2];
    }

    nm1d2 = X->size[0] * X->size[1];
    X->size[0] = iv3[0];
    X->size[1] = iv3[1];
    emxEnsureCapacity((emxArray__common *)X, nm1d2, (int)sizeof(double));
    for (k = 0; k + 1 <= i18; k++) {
      X->data[k] = ptsOut->data[k + ptsOut->size[0]];
    }

    i18 = b_this->YmapSingle->size[0] * b_this->YmapSingle->size[1];
    b_this->YmapSingle->size[0] = X->size[0];
    b_this->YmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->YmapSingle, i18, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i18 = 0; i18 < nm1d2; i18++) {
      b_this->YmapSingle->data[i18] = (float)X->data[i18];
    }
  }

  emxFree_real_T(&X);
  emxFree_real_T(&ptsOut);
}

//
// Arguments    : c_vision_internal_calibration_R *b_this
// Return Type  : c_vision_internal_calibration_R *
//
static c_vision_internal_calibration_R *c_RectificationParameters_Recti
  (c_vision_internal_calibration_R *b_this)
{
  c_vision_internal_calibration_R *c_this;
  int i3;
  static const char cv4[4] = { 'f', 'u', 'l', 'l' };

  static const char cv5[5] = { 'v', 'a', 'l', 'i', 'd' };

  signed char self_T[9];
  static const signed char T[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  c_this = b_this;
  for (i3 = 0; i3 < 2; i3++) {
    c_this->OriginalImageSize[i3] = 0.0;
  }

  i3 = c_this->OutputView->size[0] * c_this->OutputView->size[1];
  c_this->OutputView->size[0] = 1;
  c_this->OutputView->size[1] = 4;
  emxEnsureCapacity((emxArray__common *)c_this->OutputView, i3, (int)sizeof(char));
  for (i3 = 0; i3 < 4; i3++) {
    c_this->OutputView->data[i3] = cv4[i3];
  }

  i3 = c_this->OutputView->size[0] * c_this->OutputView->size[1];
  c_this->OutputView->size[0] = 1;
  c_this->OutputView->size[1] = 5;
  emxEnsureCapacity((emxArray__common *)c_this->OutputView, i3, (int)sizeof(char));
  for (i3 = 0; i3 < 5; i3++) {
    c_this->OutputView->data[i3] = cv5[i3];
  }

  for (i3 = 0; i3 < 9; i3++) {
    self_T[i3] = T[i3];
  }

  for (i3 = 0; i3 < 9; i3++) {
    c_this->H1.T[i3] = self_T[i3];
  }

  for (i3 = 0; i3 < 9; i3++) {
    self_T[i3] = T[i3];
  }

  for (i3 = 0; i3 < 9; i3++) {
    c_this->H2.T[i3] = self_T[i3];
  }

  return c_this;
}

//
// Arguments    : c_vision_internal_calibration_S *b_this
//                const double c_varargin_1_CameraParameters1_[3]
//                const double d_varargin_1_CameraParameters1_[2]
//                const char e_varargin_1_CameraParameters1_[2]
//                double f_varargin_1_CameraParameters1_
//                const double g_varargin_1_CameraParameters1_[9]
//                const double c_varargin_1_CameraParameters2_[3]
//                const double d_varargin_1_CameraParameters2_[2]
//                const char e_varargin_1_CameraParameters2_[2]
//                double f_varargin_1_CameraParameters2_
//                const double g_varargin_1_CameraParameters2_[9]
//                const double varargin_1_TranslationOfCamera2[3]
//                const struct3_T *varargin_1_RectificationParams
//                d_vision_internal_calibration_C *iobj_0
//                d_vision_internal_calibration_C *iobj_1
// Return Type  : c_vision_internal_calibration_S *
//
static c_vision_internal_calibration_S *c_StereoParametersImpl_StereoPa
  (c_vision_internal_calibration_S *b_this, const double
   c_varargin_1_CameraParameters1_[3], const double
   d_varargin_1_CameraParameters1_[2], const char
   e_varargin_1_CameraParameters1_[2], double f_varargin_1_CameraParameters1_,
   const double g_varargin_1_CameraParameters1_[9], const double
   c_varargin_1_CameraParameters2_[3], const double
   d_varargin_1_CameraParameters2_[2], const char
   e_varargin_1_CameraParameters2_[2], double f_varargin_1_CameraParameters2_,
   const double g_varargin_1_CameraParameters2_[9], const double
   varargin_1_TranslationOfCamera2[3], const struct3_T
   *varargin_1_RectificationParams, d_vision_internal_calibration_C *iobj_0,
   d_vision_internal_calibration_C *iobj_1)
{
  c_vision_internal_calibration_S *c_this;
  c_vision_internal_calibration_R *d_this;
  int i2;
  static const char cv2[4] = { 'f', 'u', 'l', 'l' };

  static const char cv3[5] = { 'v', 'a', 'l', 'i', 'd' };

  signed char self_T[9];
  static const signed char T[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  d_vision_internal_calibration_C *camParams1;
  d_vision_internal_calibration_C *camParams2;
  double b_self_T[9];
  double c_self_T[9];
  c_this = b_this;
  d_this = &c_this->RectificationParams;
  for (i2 = 0; i2 < 2; i2++) {
    d_this->OriginalImageSize[i2] = 0.0;
  }

  i2 = d_this->OutputView->size[0] * d_this->OutputView->size[1];
  d_this->OutputView->size[0] = 1;
  d_this->OutputView->size[1] = 4;
  emxEnsureCapacity((emxArray__common *)d_this->OutputView, i2, (int)sizeof(char));
  for (i2 = 0; i2 < 4; i2++) {
    d_this->OutputView->data[i2] = cv2[i2];
  }

  i2 = d_this->OutputView->size[0] * d_this->OutputView->size[1];
  d_this->OutputView->size[0] = 1;
  d_this->OutputView->size[1] = 5;
  emxEnsureCapacity((emxArray__common *)d_this->OutputView, i2, (int)sizeof(char));
  for (i2 = 0; i2 < 5; i2++) {
    d_this->OutputView->data[i2] = cv3[i2];
  }

  for (i2 = 0; i2 < 9; i2++) {
    self_T[i2] = T[i2];
  }

  for (i2 = 0; i2 < 9; i2++) {
    d_this->H1.T[i2] = self_T[i2];
  }

  for (i2 = 0; i2 < 9; i2++) {
    self_T[i2] = T[i2];
  }

  for (i2 = 0; i2 < 9; i2++) {
    d_this->H2.T[i2] = self_T[i2];
  }

  camParams1 = c_CameraParametersImpl_CameraPa(iobj_0,
    c_varargin_1_CameraParameters1_, d_varargin_1_CameraParameters1_,
    e_varargin_1_CameraParameters1_, f_varargin_1_CameraParameters1_,
    g_varargin_1_CameraParameters1_);
  camParams2 = c_CameraParametersImpl_CameraPa(iobj_1,
    c_varargin_1_CameraParameters2_, d_varargin_1_CameraParameters2_,
    e_varargin_1_CameraParameters2_, f_varargin_1_CameraParameters2_,
    g_varargin_1_CameraParameters2_);
  if (varargin_1_RectificationParams->Initialized) {
    for (i2 = 0; i2 < 9; i2++) {
      b_self_T[i2] = varargin_1_RectificationParams->H1[i2];
      c_self_T[i2] = varargin_1_RectificationParams->H2[i2];
    }

    d_this = &c_this->RectificationParams;
    for (i2 = 0; i2 < 2; i2++) {
      d_this->OriginalImageSize[i2] =
        varargin_1_RectificationParams->OriginalImageSize[i2];
    }

    for (i2 = 0; i2 < 9; i2++) {
      d_this->H1.T[i2] = b_self_T[i2];
    }

    for (i2 = 0; i2 < 9; i2++) {
      d_this->H2.T[i2] = c_self_T[i2];
    }

    i2 = d_this->OutputView->size[0] * d_this->OutputView->size[1];
    d_this->OutputView->size[0] = 1;
    d_this->OutputView->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)d_this->OutputView, i2, (int)sizeof
                      (char));
    for (i2 = 0; i2 < 5; i2++) {
      d_this->OutputView->data[i2] = varargin_1_RectificationParams->
        OutputView[i2];
    }

    for (i2 = 0; i2 < 2; i2++) {
      d_this->XBounds[i2] = varargin_1_RectificationParams->XBounds[i2];
    }

    for (i2 = 0; i2 < 2; i2++) {
      d_this->YBounds[i2] = varargin_1_RectificationParams->YBounds[i2];
    }
  }

  c_this->CameraParameters1 = camParams1;
  c_this->CameraParameters2 = camParams2;
  for (i2 = 0; i2 < 3; i2++) {
    c_this->TranslationOfCamera2[i2] = varargin_1_TranslationOfCamera2[i2];
  }

  return c_this;
}

//
// Arguments    : const d_vision_internal_calibration_S *b_this
//                double Rl[9]
//                double Rr[9]
// Return Type  : void
//
static void c_StereoParametersImpl_computeH(const
  d_vision_internal_calibration_S *b_this, double Rl[9], double Rr[9])
{
  double rotationMatrix[9];
  int ixstart;
  int ix;
  double V[9];
  double r[3];
  double U[9];
  double t;
  int i;
  double theta;
  int itmp;
  boolean_T exitg1;
  double b_r[3];
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      rotationMatrix[ix + 3 * ixstart] = b_this->RotationOfCamera2[ixstart + 3 *
        ix];
    }
  }

  eml_xgesvd(rotationMatrix, U, r, V);
  t = 0.0;
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      rotationMatrix[ixstart + 3 * ix] = 0.0;
      for (i = 0; i < 3; i++) {
        rotationMatrix[ixstart + 3 * ix] += U[ixstart + 3 * i] * V[ix + 3 * i];
      }
    }

    t += rotationMatrix[ixstart + 3 * ixstart];
  }

  theta = acos((t - 1.0) / 2.0);
  r[0] = rotationMatrix[5] - rotationMatrix[7];
  r[1] = rotationMatrix[6] - rotationMatrix[2];
  r[2] = rotationMatrix[1] - rotationMatrix[3];
  if (sin(theta) >= 0.0001) {
    t = theta / (2.0 * sin(theta));
    for (ixstart = 0; ixstart < 3; ixstart++) {
      r[ixstart] *= t;
    }
  } else if (t - 1.0 > 0.0) {
    t = (t - 3.0) / 12.0;
    for (ixstart = 0; ixstart < 3; ixstart++) {
      r[ixstart] *= 0.5 - t;
    }
  } else {
    for (ixstart = 0; ixstart < 3; ixstart++) {
      r[ixstart] = rotationMatrix[ixstart << 2];
    }

    ixstart = 1;
    t = r[0];
    itmp = 0;
    if (rtIsNaN(r[0])) {
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix + 1 < 4)) {
        ixstart = ix + 1;
        if (!rtIsNaN(r[ix])) {
          t = r[ix];
          itmp = ix;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < 3) {
      while (ixstart + 1 < 4) {
        if (r[ixstart] > t) {
          t = r[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }

    ixstart = (itmp - (int)floor((double)(itmp + 1) / 3.0) * 3) + 1;
    ix = (itmp - (int)floor(((double)(itmp + 1) + 1.0) / 3.0) * 3) + 2;
    t = sqrt(((rotationMatrix[itmp + 3 * itmp] - rotationMatrix[ixstart + 3 *
               ixstart]) - rotationMatrix[ix + 3 * ix]) + 1.0);
    for (i = 0; i < 3; i++) {
      r[i] = 0.0;
    }

    r[itmp] = t / 2.0;
    r[ixstart] = (rotationMatrix[ixstart + 3 * itmp] + rotationMatrix[itmp + 3 *
                  ixstart]) / (2.0 * t);
    r[ix] = (rotationMatrix[ix + 3 * itmp] + rotationMatrix[itmp + 3 * ix]) /
      (2.0 * t);
    t = norm(r);
    for (ixstart = 0; ixstart < 3; ixstart++) {
      r[ixstart] = theta * r[ixstart] / t;
    }
  }

  for (i = 0; i < 3; i++) {
    b_r[i] = r[i] / -2.0;
  }

  rodriguesVectorToMatrix(b_r, Rr);
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      Rl[ix + 3 * ixstart] = Rr[ixstart + 3 * ix];
    }
  }
}

//
// Arguments    : const d_vision_internal_calibration_S *b_this
//                const double Hleft_T[9]
//                const double Hright_T[9]
//                double xBounds[2]
//                double yBounds[2]
// Return Type  : void
//
static void c_StereoParametersImpl_computeO(const
  d_vision_internal_calibration_S *b_this, const double Hleft_T[9], const double
  Hright_T[9], double xBounds[2], double yBounds[2])
{
  double b_yBounds[2];
  double b_xBounds[2];
  double undistortBounds1[8];
  double undistortBounds2[8];
  double U[12];
  int jtilecol;
  int ibtile;
  double X[12];
  int k;
  double b_X[12];
  double outPts[16];
  double xSort[8];
  double ySort[8];
  c_CameraParametersImpl_computeU(b_this->CameraParameters1, b_xBounds,
    b_yBounds);
  undistortBounds1[0] = b_xBounds[0];
  undistortBounds1[4] = b_yBounds[0];
  undistortBounds1[1] = b_xBounds[1];
  undistortBounds1[5] = b_yBounds[0];
  undistortBounds1[2] = b_xBounds[1];
  undistortBounds1[6] = b_yBounds[1];
  undistortBounds1[3] = b_xBounds[0];
  undistortBounds1[7] = b_yBounds[1];
  c_CameraParametersImpl_computeU(b_this->CameraParameters2, b_xBounds,
    b_yBounds);
  undistortBounds2[0] = b_xBounds[0];
  undistortBounds2[4] = b_yBounds[0];
  undistortBounds2[1] = b_xBounds[1];
  undistortBounds2[5] = b_yBounds[0];
  undistortBounds2[2] = b_xBounds[1];
  undistortBounds2[6] = b_yBounds[1];
  undistortBounds2[3] = b_xBounds[0];
  undistortBounds2[7] = b_yBounds[1];
  for (jtilecol = 0; jtilecol < 4; jtilecol++) {
    U[8 + jtilecol] = 1.0;
  }

  for (ibtile = 0; ibtile < 2; ibtile++) {
    for (jtilecol = 0; jtilecol < 4; jtilecol++) {
      U[jtilecol + (ibtile << 2)] = undistortBounds1[jtilecol + (ibtile << 2)];
    }
  }

  for (jtilecol = 0; jtilecol < 4; jtilecol++) {
    for (ibtile = 0; ibtile < 3; ibtile++) {
      X[jtilecol + (ibtile << 2)] = 0.0;
      for (k = 0; k < 3; k++) {
        X[jtilecol + (ibtile << 2)] += U[jtilecol + (k << 2)] * Hleft_T[k + 3 *
          ibtile];
      }
    }
  }

  for (jtilecol = 0; jtilecol < 2; jtilecol++) {
    ibtile = jtilecol << 2;
    for (k = 0; k < 4; k++) {
      undistortBounds1[ibtile + k] = X[8 + k];
    }
  }

  for (jtilecol = 0; jtilecol < 2; jtilecol++) {
    for (ibtile = 0; ibtile < 4; ibtile++) {
      X[ibtile + (jtilecol << 2)] /= undistortBounds1[ibtile + (jtilecol << 2)];
    }
  }

  for (jtilecol = 0; jtilecol < 4; jtilecol++) {
    U[8 + jtilecol] = 1.0;
  }

  for (ibtile = 0; ibtile < 2; ibtile++) {
    for (jtilecol = 0; jtilecol < 4; jtilecol++) {
      U[jtilecol + (ibtile << 2)] = undistortBounds2[jtilecol + (ibtile << 2)];
    }
  }

  for (jtilecol = 0; jtilecol < 4; jtilecol++) {
    for (ibtile = 0; ibtile < 3; ibtile++) {
      b_X[jtilecol + (ibtile << 2)] = 0.0;
      for (k = 0; k < 3; k++) {
        b_X[jtilecol + (ibtile << 2)] += U[jtilecol + (k << 2)] * Hright_T[k + 3
          * ibtile];
      }
    }
  }

  for (jtilecol = 0; jtilecol < 2; jtilecol++) {
    ibtile = jtilecol << 2;
    for (k = 0; k < 4; k++) {
      undistortBounds1[ibtile + k] = b_X[8 + k];
    }
  }

  for (jtilecol = 0; jtilecol < 2; jtilecol++) {
    for (ibtile = 0; ibtile < 4; ibtile++) {
      b_X[ibtile + (jtilecol << 2)] /= undistortBounds1[ibtile + (jtilecol << 2)];
      outPts[ibtile + (jtilecol << 3)] = X[ibtile + (jtilecol << 2)];
    }

    for (ibtile = 0; ibtile < 4; ibtile++) {
      outPts[(ibtile + (jtilecol << 3)) + 4] = b_X[ibtile + (jtilecol << 2)];
    }
  }

  memcpy(&xSort[0], &outPts[0], sizeof(double) << 3);
  b_eml_sort(xSort);
  memcpy(&ySort[0], &outPts[8], sizeof(double) << 3);
  b_eml_sort(ySort);
  xBounds[0] = rt_roundd_snf(xSort[3]);
  xBounds[1] = rt_roundd_snf(xSort[4]);
  yBounds[0] = rt_roundd_snf(ySort[3]);
  yBounds[1] = rt_roundd_snf(ySort[4]);
}

//
// Arguments    : d_vision_internal_calibration_S *b_this
//                double Hleft_T[9]
//                double Hright_T[9]
//                double Q[16]
//                double xBounds[2]
//                double yBounds[2]
// Return Type  : void
//
static void c_StereoParametersImpl_computeR(d_vision_internal_calibration_S
  *b_this, double Hleft_T[9], double Hright_T[9], double Q[16], double xBounds[2],
  double yBounds[2])
{
  double Rr[9];
  double Rl[9];
  double b[3];
  int ixstart;
  double t[3];
  int ix;
  double RrowAlign[9];
  c_vision_internal_calibration_C *c_this;
  double intrinsicMatrix[9];
  double b_intrinsicMatrix[9];
  d_vision_internal_calibration_S *d_this;
  double c_intrinsicMatrix[9];
  double Kl[9];
  double Kr[9];
  double K_new[9];
  double varargin_1_idx_1;
  double mtmp;
  boolean_T exitg1;
  double b_K_new[9];
  int i15;
  double d_intrinsicMatrix[9];
  c_StereoParametersImpl_computeH(b_this, Rl, Rr);
  for (ixstart = 0; ixstart < 3; ixstart++) {
    b[ixstart] = b_this->TranslationOfCamera2[ixstart];
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    t[ixstart] = 0.0;
    for (ix = 0; ix < 3; ix++) {
      t[ixstart] += Rr[ixstart + 3 * ix] * b[ix];
    }
  }

  computeRowAlignmentRotation(t, RrowAlign);
  c_this = b_this->CameraParameters1;
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      intrinsicMatrix[ix + 3 * ixstart] = c_this->
        IntrinsicMatrixInternal[ixstart + 3 * ix];
    }
  }

  c_this = b_this->CameraParameters2;
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      b_intrinsicMatrix[ix + 3 * ixstart] = c_this->
        IntrinsicMatrixInternal[ixstart + 3 * ix];
    }
  }

  d_this = b_this;
  c_this = d_this->CameraParameters1;
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      c_intrinsicMatrix[ix + 3 * ixstart] = c_this->
        IntrinsicMatrixInternal[ixstart + 3 * ix];
    }
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      Kl[ix + 3 * ixstart] = c_intrinsicMatrix[ixstart + 3 * ix];
    }
  }

  c_this = d_this->CameraParameters2;
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      c_intrinsicMatrix[ix + 3 * ixstart] = c_this->
        IntrinsicMatrixInternal[ixstart + 3 * ix];
    }
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      Kr[ix + 3 * ixstart] = c_intrinsicMatrix[ixstart + 3 * ix];
    }
  }

  memcpy(&K_new[0], &Kl[0], 9U * sizeof(double));
  varargin_1_idx_1 = Kl[0];
  ixstart = 1;
  mtmp = Kr[0];
  if (rtIsNaN(Kr[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 3)) {
      ixstart = 2;
      if (!rtIsNaN(varargin_1_idx_1)) {
        mtmp = varargin_1_idx_1;
        exitg1 = true;
      } else {
        ix = 3;
      }
    }
  }

  if ((ixstart < 2) && (Kl[0] < mtmp)) {
    mtmp = Kl[0];
  }

  K_new[0] = mtmp;
  K_new[4] = mtmp;
  K_new[7] = (Kr[7] + Kl[7]) / 2.0;
  K_new[3] = 0.0;
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      Kr[ixstart + 3 * ix] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        Kr[ixstart + 3 * ix] += RrowAlign[ixstart + 3 * i15] * Rl[i15 + 3 * ix];
      }
    }
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      b_K_new[ixstart + 3 * ix] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        b_K_new[ixstart + 3 * ix] += K_new[ixstart + 3 * i15] * Kr[i15 + 3 * ix];
      }

      d_intrinsicMatrix[ix + 3 * ixstart] = intrinsicMatrix[ixstart + 3 * ix];
    }
  }

  mrdivide(b_K_new, d_intrinsicMatrix, c_intrinsicMatrix);
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      Rl[ix + 3 * ixstart] = c_intrinsicMatrix[ixstart + 3 * ix];
    }
  }

  memcpy(&Hleft_T[0], &Rl[0], 9U * sizeof(double));
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      Kr[ixstart + 3 * ix] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        Kr[ixstart + 3 * ix] += RrowAlign[ixstart + 3 * i15] * Rr[i15 + 3 * ix];
      }
    }
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      b_K_new[ixstart + 3 * ix] = 0.0;
      for (i15 = 0; i15 < 3; i15++) {
        b_K_new[ixstart + 3 * ix] += K_new[ixstart + 3 * i15] * Kr[i15 + 3 * ix];
      }

      intrinsicMatrix[ix + 3 * ixstart] = b_intrinsicMatrix[ixstart + 3 * ix];
    }
  }

  mrdivide(b_K_new, intrinsicMatrix, c_intrinsicMatrix);
  for (ixstart = 0; ixstart < 3; ixstart++) {
    for (ix = 0; ix < 3; ix++) {
      Rl[ix + 3 * ixstart] = c_intrinsicMatrix[ixstart + 3 * ix];
    }
  }

  memcpy(&Hright_T[0], &Rl[0], 9U * sizeof(double));
  for (ixstart = 0; ixstart < 3; ixstart++) {
    b[ixstart] = 0.0;
    for (ix = 0; ix < 3; ix++) {
      b[ixstart] += RrowAlign[ixstart + 3 * ix] * t[ix];
    }
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    t[ixstart] = b[ixstart];
  }

  c_StereoParametersImpl_computeO(b_this, Hleft_T, Hright_T, xBounds, yBounds);
  Q[0] = 1.0;
  Q[1] = 0.0;
  Q[2] = 0.0;
  Q[3] = -(Kl[6] - xBounds[0]);
  Q[4] = 0.0;
  Q[5] = 1.0;
  Q[6] = 0.0;
  Q[7] = -(K_new[7] - yBounds[0]);
  Q[8] = 0.0;
  Q[9] = 0.0;
  Q[10] = 0.0;
  Q[11] = mtmp;
  Q[12] = 0.0;
  Q[13] = 0.0;
  Q[14] = -1.0 / t[0];
  Q[15] = 0.0;
}

//
// Arguments    : int n
//                double a
//                const double x[3]
//                int ix0
//                double y[9]
//                int iy0
// Return Type  : void
//
static void c_eml_xaxpy(int n, double a, const double x[3], int ix0, double y[9],
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
// Arguments    : c_vision_internal_calibration_I *pStruct
// Return Type  : void
//
static void c_emxFreeStruct_vision_internal(c_vision_internal_calibration_I
  *pStruct)
{
  emxFree_real32_T(&pStruct->XmapSingle);
  emxFree_real32_T(&pStruct->YmapSingle);
  emxFree_real_T(&pStruct->SizeOfImage);
  emxFree_char_T(&pStruct->ClassOfImage);
  emxFree_char_T(&pStruct->OutputView);
}

//
// Arguments    : d_vision_internal_calibration_S *pStruct
// Return Type  : void
//
static void c_emxInitStruct_vision_internal(d_vision_internal_calibration_S
  *pStruct)
{
  d_emxInitStruct_vision_internal(&pStruct->RectifyMap1);
  d_emxInitStruct_vision_internal(&pStruct->RectifyMap2);
  e_emxInitStruct_vision_internal(&pStruct->RectificationParams);
}

//
// Arguments    : const double t[3]
//                double RrowAlign[9]
// Return Type  : void
//
static void computeRowAlignmentRotation(const double t[3], double RrowAlign[9])
{
  double angle;
  double xUnitVector[3];
  int k;
  static const signed char b_xUnitVector[3] = { 1, 0, 0 };

  static const signed char iv2[3] = { -1, 0, 0 };

  double rotationAxis[3];
  double B;
  angle = 0.0;
  for (k = 0; k < 3; k++) {
    xUnitVector[k] = b_xUnitVector[k];
    angle += (double)b_xUnitVector[k] * t[k];
  }

  if (angle < 0.0) {
    for (k = 0; k < 3; k++) {
      xUnitVector[k] = iv2[k];
    }
  }

  rotationAxis[0] = t[1] * xUnitVector[2] - t[2] * xUnitVector[1];
  rotationAxis[1] = t[2] * xUnitVector[0] - t[0] * xUnitVector[2];
  rotationAxis[2] = t[0] * xUnitVector[1] - t[1] * xUnitVector[0];
  if (norm(rotationAxis) == 0.0) {
    memset(&RrowAlign[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      RrowAlign[k + 3 * k] = 1.0;
    }
  } else {
    B = norm(rotationAxis);
    angle = 0.0;
    for (k = 0; k < 3; k++) {
      angle += t[k] * xUnitVector[k];
      rotationAxis[k] /= B;
    }

    angle = acos((double)(fabs(angle) / (norm(t) * norm(xUnitVector))));
    for (k = 0; k < 3; k++) {
      rotationAxis[k] *= angle;
    }

    rodriguesVectorToMatrix(rotationAxis, RrowAlign);
  }
}

//
// Arguments    : c_vision_internal_calibration_C *b_this
//                const double varargin_1_RadialDistortion[3]
//                const double varargin_1_TangentialDistortion[2]
//                const char varargin_1_WorldUnits[2]
//                double c_varargin_1_NumRadialDistortio
//                const double varargin_1_IntrinsicMatrix[9]
// Return Type  : c_vision_internal_calibration_C *
//
static c_vision_internal_calibration_C *d_CameraParametersImpl_CameraPa
  (c_vision_internal_calibration_C *b_this, const double
   varargin_1_RadialDistortion[3], const double varargin_1_TangentialDistortion
   [2], const char varargin_1_WorldUnits[2], double
   c_varargin_1_NumRadialDistortio, const double varargin_1_IntrinsicMatrix[9])
{
  c_vision_internal_calibration_C *c_this;
  c_vision_internal_calibration_C *d_this;
  int i8;
  int i9;
  c_this = b_this;
  d_this = c_this;
  for (i8 = 0; i8 < 3; i8++) {
    for (i9 = 0; i9 < 3; i9++) {
      d_this->IntrinsicMatrixInternal[i9 + 3 * i8] =
        varargin_1_IntrinsicMatrix[i8 + 3 * i9];
    }
  }

  for (i8 = 0; i8 < 3; i8++) {
    d_this->RadialDistortion[i8] = varargin_1_RadialDistortion[i8];
  }

  for (i8 = 0; i8 < 2; i8++) {
    d_this->TangentialDistortion[i8] = varargin_1_TangentialDistortion[i8];
  }

  for (i8 = 0; i8 < 2; i8++) {
    d_this->WorldUnits[i8] = varargin_1_WorldUnits[i8];
  }

  d_this->NumRadialDistortionCoefficients = c_varargin_1_NumRadialDistortio;
  return c_this;
}

//
// Arguments    : c_vision_internal_calibration_I *b_this
//                const double intrinsicMatrix[9]
//                const double radialDist[3]
//                const double tangentialDist[2]
//                const double H_T[9]
// Return Type  : void
//
static void d_ImageTransformer_computeMap(c_vision_internal_calibration_I
  *b_this, const double intrinsicMatrix[9], const double radialDist[3], const
  double tangentialDist[2], const double H_T[9])
{
  double absx21;
  double absx11;
  int p2;
  double apnd;
  double ndbl;
  double cdiff;
  double absa;
  double absb;
  emxArray_real_T *y;
  int i20;
  int nm1d2;
  int p3;
  emxArray_real_T *b_y;
  emxArray_real_T *X;
  emxArray_real_T *Y;
  emxArray_real_T *b_X;
  emxArray_real_T *c_X;
  emxArray_real_T *b_Y;
  emxArray_real_T *d_X;
  double x[9];
  int itmp;
  emxArray_real_T *U;
  double Tinv[9];
  double varargin_1[2];
  int m;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *ptsIn;
  emxArray_real_T *b_U;
  emxArray_real_T *ptsOut;
  emxArray_char_T *a;
  boolean_T b_bool;
  int32_T exitg1;
  static const char cv18[6] = { 'd', 'o', 'u', 'b', 'l', 'e' };

  int outsize[2];
  absx21 = b_this->XBounds[0];
  absx11 = b_this->XBounds[1];
  if (rtIsNaN(absx21) || rtIsNaN(absx11)) {
    p2 = 0;
    absx21 = rtNaN;
    apnd = absx11;
  } else if (absx11 < absx21) {
    p2 = -1;
    apnd = absx11;
  } else if (rtIsInf(absx21) || rtIsInf(absx11)) {
    p2 = 0;
    absx21 = rtNaN;
    apnd = absx11;
  } else {
    ndbl = floor((absx11 - absx21) + 0.5);
    apnd = absx21 + ndbl;
    cdiff = apnd - absx11;
    absa = fabs(absx21);
    absb = fabs(absx11);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = absx11;
    } else if (cdiff > 0.0) {
      apnd = absx21 + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      p2 = (int)ndbl - 1;
    } else {
      p2 = -1;
    }
  }

  emxInit_real_T(&y, 2);
  i20 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = p2 + 1;
  emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
  if (p2 + 1 > 0) {
    y->data[0] = absx21;
    if (p2 + 1 > 1) {
      y->data[p2] = apnd;
      nm1d2 = p2 / 2;
      for (p3 = 1; p3 < nm1d2; p3++) {
        y->data[p3] = absx21 + (double)p3;
        y->data[p2 - p3] = apnd - (double)p3;
      }

      if (nm1d2 << 1 == p2) {
        y->data[nm1d2] = (absx21 + apnd) / 2.0;
      } else {
        y->data[nm1d2] = absx21 + (double)nm1d2;
        y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  absx21 = b_this->YBounds[0];
  absx11 = b_this->YBounds[1];
  if (rtIsNaN(absx21) || rtIsNaN(absx11)) {
    p2 = 0;
    absx21 = rtNaN;
    apnd = absx11;
  } else if (absx11 < absx21) {
    p2 = -1;
    apnd = absx11;
  } else if (rtIsInf(absx21) || rtIsInf(absx11)) {
    p2 = 0;
    absx21 = rtNaN;
    apnd = absx11;
  } else {
    ndbl = floor((absx11 - absx21) + 0.5);
    apnd = absx21 + ndbl;
    cdiff = apnd - absx11;
    absa = fabs(absx21);
    absb = fabs(absx11);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = absx11;
    } else if (cdiff > 0.0) {
      apnd = absx21 + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      p2 = (int)ndbl - 1;
    } else {
      p2 = -1;
    }
  }

  emxInit_real_T(&b_y, 2);
  i20 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = p2 + 1;
  emxEnsureCapacity((emxArray__common *)b_y, i20, (int)sizeof(double));
  if (p2 + 1 > 0) {
    b_y->data[0] = absx21;
    if (p2 + 1 > 1) {
      b_y->data[p2] = apnd;
      nm1d2 = p2 / 2;
      for (p3 = 1; p3 < nm1d2; p3++) {
        b_y->data[p3] = absx21 + (double)p3;
        b_y->data[p2 - p3] = apnd - (double)p3;
      }

      if (nm1d2 << 1 == p2) {
        b_y->data[nm1d2] = (absx21 + apnd) / 2.0;
      } else {
        b_y->data[nm1d2] = absx21 + (double)nm1d2;
        b_y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  emxInit_real_T(&X, 2);
  emxInit_real_T(&Y, 2);
  emxInit_real_T(&b_X, 2);
  b_emxInit_real_T(&c_X, 1);
  meshgrid(y, b_y, X, Y);
  nm1d2 = X->size[0] * X->size[1];
  p3 = X->size[0] * X->size[1];
  i20 = c_X->size[0];
  c_X->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)c_X, i20, (int)sizeof(double));
  emxFree_real_T(&b_y);
  emxFree_real_T(&y);
  for (i20 = 0; i20 < nm1d2; i20++) {
    c_X->data[i20] = X->data[i20];
  }

  b_emxInit_real_T(&b_Y, 1);
  nm1d2 = Y->size[0] * Y->size[1];
  p2 = Y->size[0] * Y->size[1];
  i20 = b_Y->size[0];
  b_Y->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)b_Y, i20, (int)sizeof(double));
  for (i20 = 0; i20 < nm1d2; i20++) {
    b_Y->data[i20] = Y->data[i20];
  }

  emxFree_real_T(&Y);
  i20 = b_X->size[0] * b_X->size[1];
  b_X->size[0] = p3;
  b_X->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b_X, i20, (int)sizeof(double));
  for (i20 = 0; i20 < p3; i20++) {
    b_X->data[i20] = c_X->data[i20];
  }

  emxFree_real_T(&c_X);
  for (i20 = 0; i20 < p2; i20++) {
    b_X->data[i20 + b_X->size[0]] = b_Y->data[i20];
  }

  emxFree_real_T(&b_Y);
  emxInit_real_T(&d_X, 2);
  padarray(b_X, d_X);
  emxFree_real_T(&b_X);
  memcpy(&x[0], &H_T[0], 9U * sizeof(double));
  nm1d2 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(H_T[0]);
  absx21 = fabs(H_T[1]);
  ndbl = fabs(H_T[2]);
  if ((absx21 > absx11) && (absx21 > ndbl)) {
    nm1d2 = 3;
    p2 = 0;
    x[0] = H_T[1];
    x[1] = H_T[0];
    x[3] = H_T[4];
    x[4] = H_T[3];
    x[6] = H_T[7];
    x[7] = H_T[6];
  } else {
    if (ndbl > absx11) {
      nm1d2 = 6;
      p3 = 0;
      x[0] = H_T[2];
      x[2] = H_T[0];
      x[3] = H_T[5];
      x[5] = H_T[3];
      x[6] = H_T[8];
      x[8] = H_T[6];
    }
  }

  absx21 = x[1] / x[0];
  x[1] /= x[0];
  absx11 = x[2] / x[0];
  x[2] /= x[0];
  x[4] -= absx21 * x[3];
  x[5] -= absx11 * x[3];
  x[7] -= absx21 * x[6];
  x[8] -= absx11 * x[6];
  if (fabs(x[5]) > fabs(x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    x[1] = absx11;
    x[2] = absx21;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }

  emxInit_real_T(&U, 2);
  ndbl = x[5];
  apnd = x[4];
  absx21 = x[5] / x[4];
  x[8] -= absx21 * x[7];
  absx11 = (absx21 * x[1] - x[2]) / x[8];
  absx21 = -(x[1] + x[7] * absx11) / x[4];
  Tinv[nm1d2] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  Tinv[nm1d2 + 1] = absx21;
  Tinv[nm1d2 + 2] = absx11;
  absx11 = -(ndbl / apnd) / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  Tinv[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  Tinv[p2 + 1] = absx21;
  Tinv[p2 + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  Tinv[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  Tinv[p3 + 1] = absx21;
  Tinv[p3 + 2] = absx11;
  varargin_1[0] = d_X->size[0];
  m = d_X->size[0];
  i20 = U->size[0] * U->size[1];
  U->size[0] = (int)varargin_1[0];
  U->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)U, i20, (int)sizeof(double));
  nm1d2 = (int)varargin_1[0] * 3;
  for (i20 = 0; i20 < nm1d2; i20++) {
    U->data[i20] = 0.0;
  }

  if (d_X->size[0] == 0) {
  } else {
    nm1d2 = d_X->size[0] << 1;
    p2 = 0;
    while ((m > 0) && (p2 <= nm1d2)) {
      i20 = p2 + m;
      for (ic = p2; ic + 1 <= i20; ic++) {
        U->data[ic] = 0.0;
      }

      p2 += m;
    }

    p3 = 0;
    p2 = 0;
    while ((m > 0) && (p2 <= nm1d2)) {
      itmp = 0;
      for (ib = p3; ib + 1 <= p3 + 3; ib++) {
        if (Tinv[ib] != 0.0) {
          ia = itmp;
          i20 = p2 + m;
          for (ic = p2; ic + 1 <= i20; ic++) {
            ia++;
            U->data[ic] += Tinv[ib] * d_X->data[ia - 1];
          }
        }

        itmp += m;
      }

      p3 += 3;
      p2 += m;
    }
  }

  emxFree_real_T(&d_X);
  emxInit_real_T(&ptsIn, 2);
  if (U->size[0] == 0) {
    i20 = ptsIn->size[0] * ptsIn->size[1];
    ptsIn->size[0] = 0;
    ptsIn->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)ptsIn, i20, (int)sizeof(double));
  } else {
    i20 = U->size[0];
    p2 = ptsIn->size[0] * ptsIn->size[1];
    ptsIn->size[0] = i20;
    ptsIn->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)ptsIn, p2, (int)sizeof(double));
    i20 = U->size[0];
    for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
      p2 = nm1d2 * i20;
      for (p3 = 1; p3 <= i20; p3++) {
        ptsIn->data[(p2 + p3) - 1] = U->data[(p3 + (U->size[0] << 1)) - 1];
      }
    }

    emxInit_real_T(&b_U, 2);
    nm1d2 = U->size[0];
    i20 = b_U->size[0] * b_U->size[1];
    b_U->size[0] = nm1d2;
    b_U->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)b_U, i20, (int)sizeof(double));
    for (i20 = 0; i20 < 2; i20++) {
      for (p2 = 0; p2 < nm1d2; p2++) {
        b_U->data[p2 + b_U->size[0] * i20] = U->data[p2 + U->size[0] * i20] /
          ptsIn->data[p2 + ptsIn->size[0] * i20];
      }
    }

    for (i20 = 0; i20 < 2; i20++) {
      nm1d2 = b_U->size[0];
      for (p2 = 0; p2 < nm1d2; p2++) {
        U->data[p2 + U->size[0] * i20] = b_U->data[p2 + b_U->size[0] * i20];
      }
    }

    emxFree_real_T(&b_U);
    nm1d2 = U->size[0];
    i20 = ptsIn->size[0] * ptsIn->size[1];
    ptsIn->size[0] = nm1d2;
    ptsIn->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)ptsIn, i20, (int)sizeof(double));
    for (i20 = 0; i20 < 2; i20++) {
      for (p2 = 0; p2 < nm1d2; p2++) {
        ptsIn->data[p2 + ptsIn->size[0] * i20] = U->data[p2 + U->size[0] * i20];
      }
    }
  }

  emxFree_real_T(&U);
  emxInit_real_T(&ptsOut, 2);
  emxInit_char_T(&a, 2);
  distortPoints(ptsIn, intrinsicMatrix, radialDist, tangentialDist, ptsOut);
  absx11 = (b_this->YBounds[1] - b_this->YBounds[0]) + 1.0;
  absx21 = (b_this->XBounds[1] - b_this->XBounds[0]) + 1.0;
  i20 = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = b_this->ClassOfImage->size[1];
  emxEnsureCapacity((emxArray__common *)a, i20, (int)sizeof(char));
  nm1d2 = b_this->ClassOfImage->size[0] * b_this->ClassOfImage->size[1];
  emxFree_real_T(&ptsIn);
  for (i20 = 0; i20 < nm1d2; i20++) {
    a->data[i20] = b_this->ClassOfImage->data[i20];
  }

  b_bool = false;
  if (a->size[1] != 6) {
  } else {
    p3 = 0;
    do {
      exitg1 = 0;
      if (p3 <= 5) {
        if (a->data[p3] != cv18[p3]) {
          exitg1 = 1;
        } else {
          p3++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  emxFree_char_T(&a);
  if (b_bool) {
  } else {
    varargin_1[0] = absx11;
    varargin_1[1] = absx21;
    i20 = ptsOut->size[0];
    for (p2 = 0; p2 < 2; p2++) {
      outsize[p2] = (int)varargin_1[p2];
    }

    p2 = X->size[0] * X->size[1];
    X->size[0] = outsize[0];
    X->size[1] = outsize[1];
    emxEnsureCapacity((emxArray__common *)X, p2, (int)sizeof(double));
    for (p3 = 0; p3 + 1 <= i20; p3++) {
      X->data[p3] = ptsOut->data[p3];
    }

    i20 = b_this->XmapSingle->size[0] * b_this->XmapSingle->size[1];
    b_this->XmapSingle->size[0] = X->size[0];
    b_this->XmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->XmapSingle, i20, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i20 = 0; i20 < nm1d2; i20++) {
      b_this->XmapSingle->data[i20] = (float)X->data[i20];
    }

    varargin_1[0] = absx11;
    varargin_1[1] = absx21;
    i20 = ptsOut->size[0];
    for (p2 = 0; p2 < 2; p2++) {
      outsize[p2] = (int)varargin_1[p2];
    }

    p2 = X->size[0] * X->size[1];
    X->size[0] = outsize[0];
    X->size[1] = outsize[1];
    emxEnsureCapacity((emxArray__common *)X, p2, (int)sizeof(double));
    for (p3 = 0; p3 + 1 <= i20; p3++) {
      X->data[p3] = ptsOut->data[p3 + ptsOut->size[0]];
    }

    i20 = b_this->YmapSingle->size[0] * b_this->YmapSingle->size[1];
    b_this->YmapSingle->size[0] = X->size[0];
    b_this->YmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->YmapSingle, i20, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i20 = 0; i20 < nm1d2; i20++) {
      b_this->YmapSingle->data[i20] = (float)X->data[i20];
    }
  }

  emxFree_real_T(&X);
  emxFree_real_T(&ptsOut);
}

//
// Arguments    : d_vision_internal_calibration_S *b_this
//                const double c_varargin_1_CameraParameters1_[3]
//                const double d_varargin_1_CameraParameters1_[2]
//                const char e_varargin_1_CameraParameters1_[2]
//                double f_varargin_1_CameraParameters1_
//                const double g_varargin_1_CameraParameters1_[9]
//                const double c_varargin_1_CameraParameters2_[3]
//                const double d_varargin_1_CameraParameters2_[2]
//                const char e_varargin_1_CameraParameters2_[2]
//                double f_varargin_1_CameraParameters2_
//                const double g_varargin_1_CameraParameters2_[9]
//                const double varargin_1_RotationOfCamera2[9]
//                const double varargin_1_TranslationOfCamera2[3]
//                const struct3_T *varargin_1_RectificationParams
//                c_vision_internal_calibration_C *iobj_0
//                c_vision_internal_calibration_C *iobj_1
// Return Type  : d_vision_internal_calibration_S *
//
static d_vision_internal_calibration_S *d_StereoParametersImpl_StereoPa
  (d_vision_internal_calibration_S *b_this, const double
   c_varargin_1_CameraParameters1_[3], const double
   d_varargin_1_CameraParameters1_[2], const char
   e_varargin_1_CameraParameters1_[2], double f_varargin_1_CameraParameters1_,
   const double g_varargin_1_CameraParameters1_[9], const double
   c_varargin_1_CameraParameters2_[3], const double
   d_varargin_1_CameraParameters2_[2], const char
   e_varargin_1_CameraParameters2_[2], double f_varargin_1_CameraParameters2_,
   const double g_varargin_1_CameraParameters2_[9], const double
   varargin_1_RotationOfCamera2[9], const double
   varargin_1_TranslationOfCamera2[3], const struct3_T
   *varargin_1_RectificationParams, c_vision_internal_calibration_C *iobj_0,
   c_vision_internal_calibration_C *iobj_1)
{
  d_vision_internal_calibration_S *c_this;
  c_vision_internal_calibration_C *camParams1;
  c_vision_internal_calibration_C *d_this;
  int i6;
  int i7;
  double self_T[9];
  double b_self_T[9];
  c_vision_internal_calibration_R *e_this;
  c_vision_internal_calibration_I *f_this;
  static const char cv8[5] = { 'u', 'i', 'n', 't', '8' };

  static const char cv9[4] = { 's', 'a', 'm', 'e' };

  c_this = b_this;
  c_RectificationParameters_Recti(&c_this->RectificationParams);
  camParams1 = d_CameraParametersImpl_CameraPa(iobj_0,
    c_varargin_1_CameraParameters1_, d_varargin_1_CameraParameters1_,
    e_varargin_1_CameraParameters1_, f_varargin_1_CameraParameters1_,
    g_varargin_1_CameraParameters1_);
  d_this = iobj_1;
  for (i6 = 0; i6 < 3; i6++) {
    for (i7 = 0; i7 < 3; i7++) {
      d_this->IntrinsicMatrixInternal[i7 + 3 * i6] =
        g_varargin_1_CameraParameters2_[i6 + 3 * i7];
    }
  }

  for (i6 = 0; i6 < 3; i6++) {
    d_this->RadialDistortion[i6] = c_varargin_1_CameraParameters2_[i6];
  }

  for (i6 = 0; i6 < 2; i6++) {
    d_this->TangentialDistortion[i6] = d_varargin_1_CameraParameters2_[i6];
  }

  for (i6 = 0; i6 < 2; i6++) {
    d_this->WorldUnits[i6] = e_varargin_1_CameraParameters2_[i6];
  }

  d_this->NumRadialDistortionCoefficients = f_varargin_1_CameraParameters2_;
  if (varargin_1_RectificationParams->Initialized) {
    for (i6 = 0; i6 < 9; i6++) {
      self_T[i6] = varargin_1_RectificationParams->H1[i6];
      b_self_T[i6] = varargin_1_RectificationParams->H2[i6];
    }

    e_this = &c_this->RectificationParams;
    for (i6 = 0; i6 < 2; i6++) {
      e_this->OriginalImageSize[i6] =
        varargin_1_RectificationParams->OriginalImageSize[i6];
    }

    for (i6 = 0; i6 < 9; i6++) {
      e_this->H1.T[i6] = self_T[i6];
    }

    for (i6 = 0; i6 < 9; i6++) {
      e_this->H2.T[i6] = b_self_T[i6];
    }

    i6 = e_this->OutputView->size[0] * e_this->OutputView->size[1];
    e_this->OutputView->size[0] = 1;
    e_this->OutputView->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)e_this->OutputView, i6, (int)sizeof
                      (char));
    for (i6 = 0; i6 < 5; i6++) {
      e_this->OutputView->data[i6] = varargin_1_RectificationParams->
        OutputView[i6];
    }

    for (i6 = 0; i6 < 2; i6++) {
      e_this->XBounds[i6] = varargin_1_RectificationParams->XBounds[i6];
    }

    for (i6 = 0; i6 < 2; i6++) {
      e_this->YBounds[i6] = varargin_1_RectificationParams->YBounds[i6];
    }
  }

  c_this->CameraParameters1 = camParams1;
  c_this->CameraParameters2 = d_this;
  for (i6 = 0; i6 < 9; i6++) {
    c_this->RotationOfCamera2[i6] = varargin_1_RotationOfCamera2[i6];
  }

  for (i6 = 0; i6 < 3; i6++) {
    c_this->TranslationOfCamera2[i6] = varargin_1_TranslationOfCamera2[i6];
  }

  f_this = &c_this->RectifyMap1;
  for (i6 = 0; i6 < 2; i6++) {
    f_this->XBounds[i6] = -1.0;
  }

  for (i6 = 0; i6 < 2; i6++) {
    f_this->YBounds[i6] = -1.0;
  }

  i6 = f_this->SizeOfImage->size[0] * f_this->SizeOfImage->size[1];
  f_this->SizeOfImage->size[0] = 1;
  f_this->SizeOfImage->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)f_this->SizeOfImage, i6, (int)sizeof
                    (double));
  for (i6 = 0; i6 < 2; i6++) {
    f_this->SizeOfImage->data[i6] = 0.0;
  }

  i6 = f_this->SizeOfImage->size[0] * f_this->SizeOfImage->size[1];
  f_this->SizeOfImage->size[0] = 1;
  f_this->SizeOfImage->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)f_this->SizeOfImage, i6, (int)sizeof
                    (double));
  for (i6 = 0; i6 < 3; i6++) {
    f_this->SizeOfImage->data[i6] = 0.0;
  }

  i6 = f_this->ClassOfImage->size[0] * f_this->ClassOfImage->size[1];
  f_this->ClassOfImage->size[0] = 1;
  f_this->ClassOfImage->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)f_this->ClassOfImage, i6, (int)sizeof
                    (char));
  f_this->ClassOfImage->data[0] = 'a';
  i6 = f_this->ClassOfImage->size[0] * f_this->ClassOfImage->size[1];
  f_this->ClassOfImage->size[0] = 1;
  f_this->ClassOfImage->size[1] = 5;
  emxEnsureCapacity((emxArray__common *)f_this->ClassOfImage, i6, (int)sizeof
                    (char));
  for (i6 = 0; i6 < 5; i6++) {
    f_this->ClassOfImage->data[i6] = cv8[i6];
  }

  i6 = f_this->OutputView->size[0] * f_this->OutputView->size[1];
  f_this->OutputView->size[0] = 1;
  f_this->OutputView->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)f_this->OutputView, i6, (int)sizeof(char));
  f_this->OutputView->data[0] = 'a';
  i6 = f_this->OutputView->size[0] * f_this->OutputView->size[1];
  f_this->OutputView->size[0] = 1;
  f_this->OutputView->size[1] = 4;
  emxEnsureCapacity((emxArray__common *)f_this->OutputView, i6, (int)sizeof(char));
  for (i6 = 0; i6 < 4; i6++) {
    f_this->OutputView->data[i6] = cv9[i6];
  }

  i6 = f_this->XmapSingle->size[0] * f_this->XmapSingle->size[1];
  f_this->XmapSingle->size[0] = 2;
  f_this->XmapSingle->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)f_this->XmapSingle, i6, (int)sizeof
                    (float));
  for (i6 = 0; i6 < 4; i6++) {
    f_this->XmapSingle->data[i6] = 0.0F;
  }

  i6 = f_this->YmapSingle->size[0] * f_this->YmapSingle->size[1];
  f_this->YmapSingle->size[0] = 2;
  f_this->YmapSingle->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)f_this->YmapSingle, i6, (int)sizeof
                    (float));
  for (i6 = 0; i6 < 4; i6++) {
    f_this->YmapSingle->data[i6] = 0.0F;
  }

  f_this = &c_this->RectifyMap2;
  for (i6 = 0; i6 < 2; i6++) {
    f_this->XBounds[i6] = -1.0;
  }

  for (i6 = 0; i6 < 2; i6++) {
    f_this->YBounds[i6] = -1.0;
  }

  i6 = f_this->SizeOfImage->size[0] * f_this->SizeOfImage->size[1];
  f_this->SizeOfImage->size[0] = 1;
  f_this->SizeOfImage->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)f_this->SizeOfImage, i6, (int)sizeof
                    (double));
  for (i6 = 0; i6 < 2; i6++) {
    f_this->SizeOfImage->data[i6] = 0.0;
  }

  i6 = f_this->SizeOfImage->size[0] * f_this->SizeOfImage->size[1];
  f_this->SizeOfImage->size[0] = 1;
  f_this->SizeOfImage->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)f_this->SizeOfImage, i6, (int)sizeof
                    (double));
  for (i6 = 0; i6 < 3; i6++) {
    f_this->SizeOfImage->data[i6] = 0.0;
  }

  i6 = f_this->ClassOfImage->size[0] * f_this->ClassOfImage->size[1];
  f_this->ClassOfImage->size[0] = 1;
  f_this->ClassOfImage->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)f_this->ClassOfImage, i6, (int)sizeof
                    (char));
  f_this->ClassOfImage->data[0] = 'a';
  i6 = f_this->ClassOfImage->size[0] * f_this->ClassOfImage->size[1];
  f_this->ClassOfImage->size[0] = 1;
  f_this->ClassOfImage->size[1] = 5;
  emxEnsureCapacity((emxArray__common *)f_this->ClassOfImage, i6, (int)sizeof
                    (char));
  for (i6 = 0; i6 < 5; i6++) {
    f_this->ClassOfImage->data[i6] = cv8[i6];
  }

  i6 = f_this->OutputView->size[0] * f_this->OutputView->size[1];
  f_this->OutputView->size[0] = 1;
  f_this->OutputView->size[1] = 1;
  emxEnsureCapacity((emxArray__common *)f_this->OutputView, i6, (int)sizeof(char));
  f_this->OutputView->data[0] = 'a';
  i6 = f_this->OutputView->size[0] * f_this->OutputView->size[1];
  f_this->OutputView->size[0] = 1;
  f_this->OutputView->size[1] = 4;
  emxEnsureCapacity((emxArray__common *)f_this->OutputView, i6, (int)sizeof(char));
  for (i6 = 0; i6 < 4; i6++) {
    f_this->OutputView->data[i6] = cv9[i6];
  }

  i6 = f_this->XmapSingle->size[0] * f_this->XmapSingle->size[1];
  f_this->XmapSingle->size[0] = 2;
  f_this->XmapSingle->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)f_this->XmapSingle, i6, (int)sizeof
                    (float));
  for (i6 = 0; i6 < 4; i6++) {
    f_this->XmapSingle->data[i6] = 0.0F;
  }

  i6 = f_this->YmapSingle->size[0] * f_this->YmapSingle->size[1];
  f_this->YmapSingle->size[0] = 2;
  f_this->YmapSingle->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)f_this->YmapSingle, i6, (int)sizeof
                    (float));
  for (i6 = 0; i6 < 4; i6++) {
    f_this->YmapSingle->data[i6] = 0.0F;
  }

  return c_this;
}

//
// Arguments    : d_vision_internal_calibration_S *pStruct
// Return Type  : void
//
static void d_emxFreeStruct_vision_internal(d_vision_internal_calibration_S
  *pStruct)
{
  c_emxFreeStruct_vision_internal(&pStruct->RectifyMap1);
  c_emxFreeStruct_vision_internal(&pStruct->RectifyMap2);
  e_emxFreeStruct_vision_internal(&pStruct->RectificationParams);
}

//
// Arguments    : c_vision_internal_calibration_I *pStruct
// Return Type  : void
//
static void d_emxInitStruct_vision_internal(c_vision_internal_calibration_I
  *pStruct)
{
  emxInit_real32_T(&pStruct->XmapSingle, 2);
  emxInit_real32_T(&pStruct->YmapSingle, 2);
  emxInit_real_T(&pStruct->SizeOfImage, 2);
  emxInit_char_T(&pStruct->ClassOfImage, 2);
  emxInit_char_T(&pStruct->OutputView, 2);
}

//
// Arguments    : const emxArray_real_T *points
//                const double intrinsicMatrix[9]
//                const double radialDistortion[3]
//                const double tangentialDistortion[2]
//                emxArray_real_T *distortedPoints
// Return Type  : void
//
static void distortPoints(const emxArray_real_T *points, const double
  intrinsicMatrix[9], const double radialDistortion[3], const double
  tangentialDistortion[2], emxArray_real_T *distortedPoints)
{
  emxArray_real_T *centeredPoints;
  double center[2];
  int na1;
  int i11;
  int points_idx_0;
  emxArray_real_T *av;
  emxArray_real_T *cv;
  int k;
  int ak;
  int bk;
  int b_points_idx_0;
  int ck;
  int loop_ub;
  emxArray_real_T *b_centeredPoints;
  emxArray_real_T *yNorm;
  emxArray_real_T *c_centeredPoints;
  emxArray_real_T *r2;
  emxArray_real_T *r4;
  double b_k[3];
  emxArray_real_T *xyProduct;
  emxArray_real_T *b;
  double a;
  double b_a;
  emxArray_real_T *b_r4;
  emxArray_real_T *b_xyProduct;
  emxInit_real_T(&centeredPoints, 2);
  center[0] = intrinsicMatrix[2];
  center[1] = intrinsicMatrix[5];
  na1 = points->size[0];
  i11 = centeredPoints->size[0] * centeredPoints->size[1];
  centeredPoints->size[0] = points->size[0];
  centeredPoints->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)centeredPoints, i11, (int)sizeof(double));
  points_idx_0 = points->size[0];
  b_emxInit_real_T(&av, 1);
  b_emxInit_real_T(&cv, 1);
  if (points_idx_0 == 0) {
  } else {
    k = points->size[0];
    i11 = av->size[0];
    av->size[0] = k;
    emxEnsureCapacity((emxArray__common *)av, i11, (int)sizeof(double));
    ak = -1;
    bk = 0;
    points_idx_0 = points->size[0];
    k = points->size[0];
    b_points_idx_0 = points->size[0];
    i11 = (k << 1) - b_points_idx_0;
    ck = 0;
    while ((points_idx_0 > 0) && (ck <= i11)) {
      for (k = 1; k <= na1; k++) {
        av->data[k - 1] = points->data[ak + k];
      }

      b_points_idx_0 = cv->size[0];
      cv->size[0] = av->size[0];
      emxEnsureCapacity((emxArray__common *)cv, b_points_idx_0, (int)sizeof
                        (double));
      loop_ub = av->size[0];
      for (b_points_idx_0 = 0; b_points_idx_0 < loop_ub; b_points_idx_0++) {
        cv->data[b_points_idx_0] = av->data[b_points_idx_0] - center[bk];
      }

      for (k = 1; k <= points_idx_0; k++) {
        centeredPoints->data[(ck + k) - 1] = cv->data[k - 1];
      }

      ak += na1;
      bk++;
      ck += points_idx_0;
    }
  }

  b_emxInit_real_T(&b_centeredPoints, 1);
  k = centeredPoints->size[0];
  loop_ub = centeredPoints->size[0];
  i11 = b_centeredPoints->size[0];
  b_centeredPoints->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)b_centeredPoints, i11, (int)sizeof
                    (double));
  for (i11 = 0; i11 < loop_ub; i11++) {
    b_centeredPoints->data[i11] = centeredPoints->data[i11 + k];
  }

  b_emxInit_real_T(&yNorm, 1);
  b_emxInit_real_T(&c_centeredPoints, 1);
  rdivide(b_centeredPoints, intrinsicMatrix[4], yNorm);
  loop_ub = centeredPoints->size[0];
  i11 = c_centeredPoints->size[0];
  c_centeredPoints->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)c_centeredPoints, i11, (int)sizeof
                    (double));
  emxFree_real_T(&b_centeredPoints);
  for (i11 = 0; i11 < loop_ub; i11++) {
    c_centeredPoints->data[i11] = centeredPoints->data[i11] - intrinsicMatrix[1]
      * yNorm->data[i11];
  }

  b_emxInit_real_T(&r2, 1);
  rdivide(c_centeredPoints, intrinsicMatrix[0], cv);
  power(cv, r2);
  power(yNorm, av);
  i11 = r2->size[0];
  emxEnsureCapacity((emxArray__common *)r2, i11, (int)sizeof(double));
  loop_ub = r2->size[0];
  emxFree_real_T(&c_centeredPoints);
  for (i11 = 0; i11 < loop_ub; i11++) {
    r2->data[i11] += av->data[i11];
  }

  b_emxInit_real_T(&r4, 1);
  i11 = r4->size[0];
  r4->size[0] = r2->size[0];
  emxEnsureCapacity((emxArray__common *)r4, i11, (int)sizeof(double));
  loop_ub = r2->size[0];
  for (i11 = 0; i11 < loop_ub; i11++) {
    r4->data[i11] = r2->data[i11] * r2->data[i11];
  }

  for (i11 = 0; i11 < 3; i11++) {
    b_k[i11] = 0.0;
  }

  for (i11 = 0; i11 < 2; i11++) {
    b_k[i11] = radialDistortion[i11];
  }

  i11 = r4->size[0];
  r4->size[0] = r2->size[0];
  emxEnsureCapacity((emxArray__common *)r4, i11, (int)sizeof(double));
  loop_ub = r2->size[0];
  for (i11 = 0; i11 < loop_ub; i11++) {
    r4->data[i11] = (b_k[0] * r2->data[i11] + b_k[1] * r4->data[i11]) +
      radialDistortion[2] * (r2->data[i11] * r4->data[i11]);
  }

  b_emxInit_real_T(&xyProduct, 1);
  i11 = xyProduct->size[0];
  xyProduct->size[0] = cv->size[0];
  emxEnsureCapacity((emxArray__common *)xyProduct, i11, (int)sizeof(double));
  loop_ub = cv->size[0];
  for (i11 = 0; i11 < loop_ub; i11++) {
    xyProduct->data[i11] = cv->data[i11] * yNorm->data[i11];
  }

  b_emxInit_real_T(&b, 1);
  power(cv, b);
  a = 2.0 * tangentialDistortion[0];
  power(yNorm, av);
  b_a = 2.0 * tangentialDistortion[1];
  i11 = av->size[0];
  av->size[0] = r2->size[0];
  emxEnsureCapacity((emxArray__common *)av, i11, (int)sizeof(double));
  loop_ub = r2->size[0];
  emxFree_real_T(&cv);
  emxFree_real_T(&yNorm);
  for (i11 = 0; i11 < loop_ub; i11++) {
    av->data[i11] = tangentialDistortion[0] * (r2->data[i11] + 2.0 * av->
      data[i11]) + b_a * xyProduct->data[i11];
  }

  i11 = xyProduct->size[0];
  emxEnsureCapacity((emxArray__common *)xyProduct, i11, (int)sizeof(double));
  loop_ub = xyProduct->size[0];
  for (i11 = 0; i11 < loop_ub; i11++) {
    xyProduct->data[i11] = (a * xyProduct->data[i11] + tangentialDistortion[1] *
      (r2->data[i11] + 2.0 * b->data[i11])) * intrinsicMatrix[0] +
      intrinsicMatrix[1] * av->data[i11];
  }

  emxFree_real_T(&b);
  emxFree_real_T(&r2);
  i11 = av->size[0];
  emxEnsureCapacity((emxArray__common *)av, i11, (int)sizeof(double));
  loop_ub = av->size[0];
  for (i11 = 0; i11 < loop_ub; i11++) {
    av->data[i11] *= intrinsicMatrix[4];
  }

  emxInit_real_T(&b_r4, 2);
  k = r4->size[0];
  b_points_idx_0 = r4->size[0];
  i11 = b_r4->size[0] * b_r4->size[1];
  b_r4->size[0] = k;
  b_r4->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b_r4, i11, (int)sizeof(double));
  for (i11 = 0; i11 < k; i11++) {
    b_r4->data[i11] = r4->data[i11];
  }

  for (i11 = 0; i11 < b_points_idx_0; i11++) {
    b_r4->data[i11 + b_r4->size[0]] = r4->data[i11];
  }

  emxFree_real_T(&r4);
  emxInit_real_T(&b_xyProduct, 2);
  k = xyProduct->size[0];
  b_points_idx_0 = av->size[0];
  i11 = b_xyProduct->size[0] * b_xyProduct->size[1];
  b_xyProduct->size[0] = k;
  b_xyProduct->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b_xyProduct, i11, (int)sizeof(double));
  for (i11 = 0; i11 < k; i11++) {
    b_xyProduct->data[i11] = xyProduct->data[i11];
  }

  emxFree_real_T(&xyProduct);
  for (i11 = 0; i11 < b_points_idx_0; i11++) {
    b_xyProduct->data[i11 + b_xyProduct->size[0]] = av->data[i11];
  }

  emxFree_real_T(&av);
  i11 = distortedPoints->size[0] * distortedPoints->size[1];
  distortedPoints->size[0] = points->size[0];
  distortedPoints->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)distortedPoints, i11, (int)sizeof(double));
  for (i11 = 0; i11 < 2; i11++) {
    loop_ub = points->size[0];
    for (b_points_idx_0 = 0; b_points_idx_0 < loop_ub; b_points_idx_0++) {
      distortedPoints->data[b_points_idx_0 + distortedPoints->size[0] * i11] =
        (points->data[b_points_idx_0 + points->size[0] * i11] +
         centeredPoints->data[b_points_idx_0 + centeredPoints->size[0] * i11] *
         b_r4->data[b_points_idx_0 + b_r4->size[0] * i11]) + b_xyProduct->
        data[b_points_idx_0 + b_xyProduct->size[0] * i11];
    }
  }

  emxFree_real_T(&b_xyProduct);
  emxFree_real_T(&b_r4);
  emxFree_real_T(&centeredPoints);
}

//
// Arguments    : c_vision_internal_calibration_I *b_this
//                const double intrinsicMatrix[9]
//                const double radialDist[3]
//                const double tangentialDist[2]
//                const double H_T[9]
// Return Type  : void
//
static void e_ImageTransformer_computeMap(c_vision_internal_calibration_I
  *b_this, const double intrinsicMatrix[9], const double radialDist[3], const
  double tangentialDist[2], const double H_T[9])
{
  double absx21;
  double absx11;
  int p2;
  double apnd;
  double ndbl;
  double cdiff;
  double absa;
  double absb;
  emxArray_real_T *y;
  int i22;
  int nm1d2;
  int p3;
  emxArray_real_T *b_y;
  emxArray_real_T *X;
  emxArray_real_T *Y;
  emxArray_real_T *b_X;
  emxArray_real_T *c_X;
  emxArray_real_T *b_Y;
  emxArray_real_T *d_X;
  double x[9];
  int itmp;
  emxArray_real_T *U;
  double Tinv[9];
  double varargin_1[2];
  int m;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *ptsIn;
  emxArray_real_T *b_U;
  emxArray_real_T *ptsOut;
  emxArray_char_T *a;
  boolean_T b_bool;
  int32_T exitg1;
  static const char cv19[6] = { 'd', 'o', 'u', 'b', 'l', 'e' };

  int outsize[2];
  absx21 = b_this->XBounds[0];
  absx11 = b_this->XBounds[1];
  if (rtIsNaN(absx21) || rtIsNaN(absx11)) {
    p2 = 0;
    absx21 = rtNaN;
    apnd = absx11;
  } else if (absx11 < absx21) {
    p2 = -1;
    apnd = absx11;
  } else if (rtIsInf(absx21) || rtIsInf(absx11)) {
    p2 = 0;
    absx21 = rtNaN;
    apnd = absx11;
  } else {
    ndbl = floor((absx11 - absx21) + 0.5);
    apnd = absx21 + ndbl;
    cdiff = apnd - absx11;
    absa = fabs(absx21);
    absb = fabs(absx11);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = absx11;
    } else if (cdiff > 0.0) {
      apnd = absx21 + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      p2 = (int)ndbl - 1;
    } else {
      p2 = -1;
    }
  }

  emxInit_real_T(&y, 2);
  i22 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = p2 + 1;
  emxEnsureCapacity((emxArray__common *)y, i22, (int)sizeof(double));
  if (p2 + 1 > 0) {
    y->data[0] = absx21;
    if (p2 + 1 > 1) {
      y->data[p2] = apnd;
      nm1d2 = p2 / 2;
      for (p3 = 1; p3 < nm1d2; p3++) {
        y->data[p3] = absx21 + (double)p3;
        y->data[p2 - p3] = apnd - (double)p3;
      }

      if (nm1d2 << 1 == p2) {
        y->data[nm1d2] = (absx21 + apnd) / 2.0;
      } else {
        y->data[nm1d2] = absx21 + (double)nm1d2;
        y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  absx21 = b_this->YBounds[0];
  absx11 = b_this->YBounds[1];
  if (rtIsNaN(absx21) || rtIsNaN(absx11)) {
    p2 = 0;
    absx21 = rtNaN;
    apnd = absx11;
  } else if (absx11 < absx21) {
    p2 = -1;
    apnd = absx11;
  } else if (rtIsInf(absx21) || rtIsInf(absx11)) {
    p2 = 0;
    absx21 = rtNaN;
    apnd = absx11;
  } else {
    ndbl = floor((absx11 - absx21) + 0.5);
    apnd = absx21 + ndbl;
    cdiff = apnd - absx11;
    absa = fabs(absx21);
    absb = fabs(absx11);
    if ((absa >= absb) || rtIsNaN(absb)) {
      absb = absa;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
      ndbl++;
      apnd = absx11;
    } else if (cdiff > 0.0) {
      apnd = absx21 + (ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      p2 = (int)ndbl - 1;
    } else {
      p2 = -1;
    }
  }

  emxInit_real_T(&b_y, 2);
  i22 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = p2 + 1;
  emxEnsureCapacity((emxArray__common *)b_y, i22, (int)sizeof(double));
  if (p2 + 1 > 0) {
    b_y->data[0] = absx21;
    if (p2 + 1 > 1) {
      b_y->data[p2] = apnd;
      nm1d2 = p2 / 2;
      for (p3 = 1; p3 < nm1d2; p3++) {
        b_y->data[p3] = absx21 + (double)p3;
        b_y->data[p2 - p3] = apnd - (double)p3;
      }

      if (nm1d2 << 1 == p2) {
        b_y->data[nm1d2] = (absx21 + apnd) / 2.0;
      } else {
        b_y->data[nm1d2] = absx21 + (double)nm1d2;
        b_y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }

  emxInit_real_T(&X, 2);
  emxInit_real_T(&Y, 2);
  emxInit_real_T(&b_X, 2);
  b_emxInit_real_T(&c_X, 1);
  meshgrid(y, b_y, X, Y);
  nm1d2 = X->size[0] * X->size[1];
  p3 = X->size[0] * X->size[1];
  i22 = c_X->size[0];
  c_X->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)c_X, i22, (int)sizeof(double));
  emxFree_real_T(&b_y);
  emxFree_real_T(&y);
  for (i22 = 0; i22 < nm1d2; i22++) {
    c_X->data[i22] = X->data[i22];
  }

  b_emxInit_real_T(&b_Y, 1);
  nm1d2 = Y->size[0] * Y->size[1];
  p2 = Y->size[0] * Y->size[1];
  i22 = b_Y->size[0];
  b_Y->size[0] = nm1d2;
  emxEnsureCapacity((emxArray__common *)b_Y, i22, (int)sizeof(double));
  for (i22 = 0; i22 < nm1d2; i22++) {
    b_Y->data[i22] = Y->data[i22];
  }

  emxFree_real_T(&Y);
  i22 = b_X->size[0] * b_X->size[1];
  b_X->size[0] = p3;
  b_X->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b_X, i22, (int)sizeof(double));
  for (i22 = 0; i22 < p3; i22++) {
    b_X->data[i22] = c_X->data[i22];
  }

  emxFree_real_T(&c_X);
  for (i22 = 0; i22 < p2; i22++) {
    b_X->data[i22 + b_X->size[0]] = b_Y->data[i22];
  }

  emxFree_real_T(&b_Y);
  emxInit_real_T(&d_X, 2);
  padarray(b_X, d_X);
  emxFree_real_T(&b_X);
  memcpy(&x[0], &H_T[0], 9U * sizeof(double));
  nm1d2 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(H_T[0]);
  absx21 = fabs(H_T[1]);
  ndbl = fabs(H_T[2]);
  if ((absx21 > absx11) && (absx21 > ndbl)) {
    nm1d2 = 3;
    p2 = 0;
    x[0] = H_T[1];
    x[1] = H_T[0];
    x[3] = H_T[4];
    x[4] = H_T[3];
    x[6] = H_T[7];
    x[7] = H_T[6];
  } else {
    if (ndbl > absx11) {
      nm1d2 = 6;
      p3 = 0;
      x[0] = H_T[2];
      x[2] = H_T[0];
      x[3] = H_T[5];
      x[5] = H_T[3];
      x[6] = H_T[8];
      x[8] = H_T[6];
    }
  }

  absx21 = x[1] / x[0];
  x[1] /= x[0];
  absx11 = x[2] / x[0];
  x[2] /= x[0];
  x[4] -= absx21 * x[3];
  x[5] -= absx11 * x[3];
  x[7] -= absx21 * x[6];
  x[8] -= absx11 * x[6];
  if (fabs(x[5]) > fabs(x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    x[1] = absx11;
    x[2] = absx21;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }

  emxInit_real_T(&U, 2);
  ndbl = x[5];
  apnd = x[4];
  absx21 = x[5] / x[4];
  x[8] -= absx21 * x[7];
  absx11 = (absx21 * x[1] - x[2]) / x[8];
  absx21 = -(x[1] + x[7] * absx11) / x[4];
  Tinv[nm1d2] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  Tinv[nm1d2 + 1] = absx21;
  Tinv[nm1d2 + 2] = absx11;
  absx11 = -(ndbl / apnd) / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  Tinv[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  Tinv[p2 + 1] = absx21;
  Tinv[p2 + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  Tinv[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  Tinv[p3 + 1] = absx21;
  Tinv[p3 + 2] = absx11;
  varargin_1[0] = d_X->size[0];
  m = d_X->size[0];
  i22 = U->size[0] * U->size[1];
  U->size[0] = (int)varargin_1[0];
  U->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)U, i22, (int)sizeof(double));
  nm1d2 = (int)varargin_1[0] * 3;
  for (i22 = 0; i22 < nm1d2; i22++) {
    U->data[i22] = 0.0;
  }

  if (d_X->size[0] == 0) {
  } else {
    nm1d2 = d_X->size[0] << 1;
    p2 = 0;
    while ((m > 0) && (p2 <= nm1d2)) {
      i22 = p2 + m;
      for (ic = p2; ic + 1 <= i22; ic++) {
        U->data[ic] = 0.0;
      }

      p2 += m;
    }

    p3 = 0;
    p2 = 0;
    while ((m > 0) && (p2 <= nm1d2)) {
      itmp = 0;
      for (ib = p3; ib + 1 <= p3 + 3; ib++) {
        if (Tinv[ib] != 0.0) {
          ia = itmp;
          i22 = p2 + m;
          for (ic = p2; ic + 1 <= i22; ic++) {
            ia++;
            U->data[ic] += Tinv[ib] * d_X->data[ia - 1];
          }
        }

        itmp += m;
      }

      p3 += 3;
      p2 += m;
    }
  }

  emxFree_real_T(&d_X);
  emxInit_real_T(&ptsIn, 2);
  if (U->size[0] == 0) {
    i22 = ptsIn->size[0] * ptsIn->size[1];
    ptsIn->size[0] = 0;
    ptsIn->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)ptsIn, i22, (int)sizeof(double));
  } else {
    i22 = U->size[0];
    p2 = ptsIn->size[0] * ptsIn->size[1];
    ptsIn->size[0] = i22;
    ptsIn->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)ptsIn, p2, (int)sizeof(double));
    i22 = U->size[0];
    for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
      p2 = nm1d2 * i22;
      for (p3 = 1; p3 <= i22; p3++) {
        ptsIn->data[(p2 + p3) - 1] = U->data[(p3 + (U->size[0] << 1)) - 1];
      }
    }

    emxInit_real_T(&b_U, 2);
    nm1d2 = U->size[0];
    i22 = b_U->size[0] * b_U->size[1];
    b_U->size[0] = nm1d2;
    b_U->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)b_U, i22, (int)sizeof(double));
    for (i22 = 0; i22 < 2; i22++) {
      for (p2 = 0; p2 < nm1d2; p2++) {
        b_U->data[p2 + b_U->size[0] * i22] = U->data[p2 + U->size[0] * i22] /
          ptsIn->data[p2 + ptsIn->size[0] * i22];
      }
    }

    for (i22 = 0; i22 < 2; i22++) {
      nm1d2 = b_U->size[0];
      for (p2 = 0; p2 < nm1d2; p2++) {
        U->data[p2 + U->size[0] * i22] = b_U->data[p2 + b_U->size[0] * i22];
      }
    }

    emxFree_real_T(&b_U);
    nm1d2 = U->size[0];
    i22 = ptsIn->size[0] * ptsIn->size[1];
    ptsIn->size[0] = nm1d2;
    ptsIn->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)ptsIn, i22, (int)sizeof(double));
    for (i22 = 0; i22 < 2; i22++) {
      for (p2 = 0; p2 < nm1d2; p2++) {
        ptsIn->data[p2 + ptsIn->size[0] * i22] = U->data[p2 + U->size[0] * i22];
      }
    }
  }

  emxFree_real_T(&U);
  emxInit_real_T(&ptsOut, 2);
  emxInit_char_T(&a, 2);
  distortPoints(ptsIn, intrinsicMatrix, radialDist, tangentialDist, ptsOut);
  absx11 = (b_this->YBounds[1] - b_this->YBounds[0]) + 1.0;
  absx21 = (b_this->XBounds[1] - b_this->XBounds[0]) + 1.0;
  i22 = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = b_this->ClassOfImage->size[1];
  emxEnsureCapacity((emxArray__common *)a, i22, (int)sizeof(char));
  nm1d2 = b_this->ClassOfImage->size[0] * b_this->ClassOfImage->size[1];
  emxFree_real_T(&ptsIn);
  for (i22 = 0; i22 < nm1d2; i22++) {
    a->data[i22] = b_this->ClassOfImage->data[i22];
  }

  b_bool = false;
  if (a->size[1] != 6) {
  } else {
    p3 = 0;
    do {
      exitg1 = 0;
      if (p3 <= 5) {
        if (a->data[p3] != cv19[p3]) {
          exitg1 = 1;
        } else {
          p3++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  emxFree_char_T(&a);
  if (b_bool) {
  } else {
    varargin_1[0] = absx11;
    varargin_1[1] = absx21;
    i22 = ptsOut->size[0];
    for (p2 = 0; p2 < 2; p2++) {
      outsize[p2] = (int)varargin_1[p2];
    }

    p2 = X->size[0] * X->size[1];
    X->size[0] = outsize[0];
    X->size[1] = outsize[1];
    emxEnsureCapacity((emxArray__common *)X, p2, (int)sizeof(double));
    for (p3 = 0; p3 + 1 <= i22; p3++) {
      X->data[p3] = ptsOut->data[p3];
    }

    i22 = b_this->XmapSingle->size[0] * b_this->XmapSingle->size[1];
    b_this->XmapSingle->size[0] = X->size[0];
    b_this->XmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->XmapSingle, i22, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i22 = 0; i22 < nm1d2; i22++) {
      b_this->XmapSingle->data[i22] = (float)X->data[i22];
    }

    varargin_1[0] = absx11;
    varargin_1[1] = absx21;
    i22 = ptsOut->size[0];
    for (p2 = 0; p2 < 2; p2++) {
      outsize[p2] = (int)varargin_1[p2];
    }

    p2 = X->size[0] * X->size[1];
    X->size[0] = outsize[0];
    X->size[1] = outsize[1];
    emxEnsureCapacity((emxArray__common *)X, p2, (int)sizeof(double));
    for (p3 = 0; p3 + 1 <= i22; p3++) {
      X->data[p3] = ptsOut->data[p3 + ptsOut->size[0]];
    }

    i22 = b_this->YmapSingle->size[0] * b_this->YmapSingle->size[1];
    b_this->YmapSingle->size[0] = X->size[0];
    b_this->YmapSingle->size[1] = X->size[1];
    emxEnsureCapacity((emxArray__common *)b_this->YmapSingle, i22, (int)sizeof
                      (float));
    nm1d2 = X->size[0] * X->size[1];
    for (i22 = 0; i22 < nm1d2; i22++) {
      b_this->YmapSingle->data[i22] = (float)X->data[i22];
    }
  }

  emxFree_real_T(&X);
  emxFree_real_T(&ptsOut);
}

//
// Arguments    : c_vision_internal_calibration_R *pStruct
// Return Type  : void
//
static void e_emxFreeStruct_vision_internal(c_vision_internal_calibration_R
  *pStruct)
{
  emxFree_char_T(&pStruct->OutputView);
}

//
// Arguments    : c_vision_internal_calibration_R *pStruct
// Return Type  : void
//
static void e_emxInitStruct_vision_internal(c_vision_internal_calibration_R
  *pStruct)
{
  emxInit_char_T(&pStruct->OutputView, 2);
}

//
// Arguments    : double x[2]
// Return Type  : void
//
static void eml_sort(double x[2])
{
  double b_x[2];
  int i23;
  boolean_T p;
  for (i23 = 0; i23 < 2; i23++) {
    b_x[i23] = x[i23];
  }

  if ((x[0] <= x[1]) || rtIsNaN(x[1])) {
    p = true;
  } else {
    p = false;
  }

  if (p) {
  } else {
    b_x[0] = x[1];
    b_x[1] = x[0];
  }

  for (i23 = 0; i23 < 2; i23++) {
    x[i23] = b_x[i23];
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[9]
//                int iy0
// Return Type  : void
//
static void eml_xaxpy(int n, double a, int ix0, double y[9], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
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
//                const double x[9]
//                int ix0
//                const double y[9]
//                int iy0
// Return Type  : double
//
static double eml_xdotc(int n, const double x[9], int ix0, const double y[9],
  int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

//
// Arguments    : const double A[9]
//                double U[9]
//                double S[3]
//                double V[9]
// Return Type  : void
//
static void eml_xgesvd(const double A[9], double U[9], double S[3], double V[9])
{
  double b_A[9];
  double s[3];
  double e[3];
  double work[3];
  int kase;
  int q;
  int qs;
  boolean_T apply_transform;
  double ztest0;
  int ii;
  int m;
  double rt;
  double ztest;
  int iter;
  double snorm;
  int32_T exitg3;
  boolean_T exitg2;
  double f;
  double varargin_1[5];
  double mtmp;
  boolean_T exitg1;
  double sqds;
  memcpy(&b_A[0], &A[0], 9U * sizeof(double));
  for (kase = 0; kase < 3; kase++) {
    s[kase] = 0.0;
    e[kase] = 0.0;
    work[kase] = 0.0;
  }

  for (kase = 0; kase < 9; kase++) {
    U[kase] = 0.0;
    V[kase] = 0.0;
  }

  for (q = 0; q < 2; q++) {
    qs = q + 3 * q;
    apply_transform = false;
    ztest0 = eml_xnrm2(3 - q, b_A, qs + 1);
    if (ztest0 > 0.0) {
      apply_transform = true;
      if (b_A[qs] < 0.0) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      if (fabs(s[q]) >= 1.0020841800044864E-292) {
        ztest0 = 1.0 / s[q];
        kase = (qs - q) + 3;
        for (ii = qs; ii + 1 <= kase; ii++) {
          b_A[ii] *= ztest0;
        }
      } else {
        kase = (qs - q) + 3;
        for (ii = qs; ii + 1 <= kase; ii++) {
          b_A[ii] /= s[q];
        }
      }

      b_A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }

    for (ii = q + 1; ii + 1 < 4; ii++) {
      kase = q + 3 * ii;
      if (apply_transform) {
        eml_xaxpy(3 - q, -(eml_xdotc(3 - q, b_A, qs + 1, b_A, kase + 1) / b_A[q
                           + 3 * q]), qs + 1, b_A, kase + 1);
      }

      e[ii] = b_A[kase];
    }

    for (ii = q; ii + 1 < 4; ii++) {
      U[ii + 3 * q] = b_A[ii + 3 * q];
    }

    if (q + 1 <= 1) {
      ztest0 = b_eml_xnrm2(e, 2);
      if (ztest0 == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          ztest0 = -ztest0;
        }

        e[0] = ztest0;
        if (fabs(ztest0) >= 1.0020841800044864E-292) {
          ztest0 = 1.0 / ztest0;
          for (ii = 1; ii + 1 < 4; ii++) {
            e[ii] *= ztest0;
          }
        } else {
          for (ii = 1; ii + 1 < 4; ii++) {
            e[ii] /= ztest0;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (ii = 2; ii < 4; ii++) {
          work[ii - 1] = 0.0;
        }

        for (ii = 1; ii + 1 < 4; ii++) {
          b_eml_xaxpy(2, e[ii], b_A, 3 * ii + 2, work, 2);
        }

        for (ii = 1; ii + 1 < 4; ii++) {
          c_eml_xaxpy(2, -e[ii] / e[1], work, 2, b_A, 3 * ii + 2);
        }
      }

      for (ii = 1; ii + 1 < 4; ii++) {
        V[ii] = e[ii];
      }
    }
  }

  m = 1;
  s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  for (ii = 0; ii < 3; ii++) {
    U[6 + ii] = 0.0;
  }

  U[8] = 1.0;
  for (q = 1; q > -1; q += -1) {
    qs = q + 3 * q;
    if (s[q] != 0.0) {
      for (ii = q + 1; ii + 1 < 4; ii++) {
        kase = (q + 3 * ii) + 1;
        eml_xaxpy(3 - q, -(eml_xdotc(3 - q, U, qs + 1, U, kase) / U[qs]), qs + 1,
                  U, kase);
      }

      for (ii = q; ii + 1 < 4; ii++) {
        U[ii + 3 * q] = -U[ii + 3 * q];
      }

      U[qs]++;
      ii = 1;
      while (ii <= q) {
        U[3] = 0.0;
        ii = 2;
      }
    } else {
      for (ii = 0; ii < 3; ii++) {
        U[ii + 3 * q] = 0.0;
      }

      U[qs] = 1.0;
    }
  }

  for (q = 2; q > -1; q += -1) {
    if ((q + 1 <= 1) && (e[0] != 0.0)) {
      for (ii = 0; ii < 2; ii++) {
        kase = 2 + 3 * (ii + 1);
        eml_xaxpy(2, -(eml_xdotc(2, V, 2, V, kase) / V[1]), 2, V, kase);
      }
    }

    for (ii = 0; ii < 3; ii++) {
      V[ii + 3 * q] = 0.0;
    }

    V[q + 3 * q] = 1.0;
  }

  for (q = 0; q < 3; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0) {
      rt = fabs(s[q]);
      ztest = s[q] / rt;
      s[q] = rt;
      if (q + 1 < 3) {
        ztest0 = e[q] / ztest;
      }

      eml_xscal(ztest, U, 1 + 3 * q);
    }

    if ((q + 1 < 3) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      ztest = rt / ztest0;
      ztest0 = rt;
      s[q + 1] *= ztest;
      eml_xscal(ztest, V, 1 + 3 * (q + 1));
    }

    e[q] = ztest0;
  }

  iter = 0;
  snorm = 0.0;
  for (ii = 0; ii < 3; ii++) {
    ztest0 = fabs(s[ii]);
    ztest = fabs(e[ii]);
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
    ii = m;
    do {
      exitg3 = 0;
      q = ii + 1;
      if (ii + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = fabs(e[ii]);
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s[ii]) + fabs(s[ii + 1])))
            || (ztest0 <= 1.0020841800044864E-292) || ((iter > 20) && (ztest0 <=
              2.2204460492503131E-16 * snorm))) {
          e[ii] = 0.0;
          exitg3 = 1;
        } else {
          ii--;
        }
      }
    } while (exitg3 == 0);

    if (ii + 1 == m + 1) {
      kase = 4;
    } else {
      qs = m + 2;
      kase = m + 2;
      exitg2 = false;
      while ((!exitg2) && (kase >= ii + 1)) {
        qs = kase;
        if (kase == ii + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0;
          if (kase < m + 2) {
            ztest0 = fabs(e[kase - 1]);
          }

          if (kase > ii + 2) {
            ztest0 += fabs(e[kase - 2]);
          }

          ztest = fabs(s[kase - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s[kase - 1] = 0.0;
            exitg2 = true;
          } else {
            kase--;
          }
        }
      }

      if (qs == ii + 1) {
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
      for (ii = m; ii + 1 >= q + 1; ii--) {
        ztest0 = s[ii];
        eml_xrotg(&ztest0, &f, &ztest, &rt);
        s[ii] = ztest0;
        if (ii + 1 > q + 1) {
          f = -rt * e[0];
          e[0] *= ztest;
        }

        eml_xrot(V, 1 + 3 * ii, 1 + 3 * (m + 1), ztest, rt);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      for (ii = q; ii + 1 <= m + 2; ii++) {
        eml_xrotg(&s[ii], &f, &ztest, &rt);
        f = -rt * e[ii];
        e[ii] *= ztest;
        eml_xrot(U, 1 + 3 * ii, 1 + 3 * (q - 1), ztest, rt);
      }
      break;

     case 3:
      varargin_1[0] = fabs(s[m + 1]);
      varargin_1[1] = fabs(s[m]);
      varargin_1[2] = fabs(e[m]);
      varargin_1[3] = fabs(s[q]);
      varargin_1[4] = fabs(e[q]);
      kase = 1;
      mtmp = varargin_1[0];
      if (rtIsNaN(varargin_1[0])) {
        ii = 2;
        exitg1 = false;
        while ((!exitg1) && (ii < 6)) {
          kase = ii;
          if (!rtIsNaN(varargin_1[ii - 1])) {
            mtmp = varargin_1[ii - 1];
            exitg1 = true;
          } else {
            ii++;
          }
        }
      }

      if (kase < 5) {
        while (kase + 1 < 6) {
          if (varargin_1[kase] > mtmp) {
            mtmp = varargin_1[kase];
          }

          kase++;
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
      for (ii = q + 1; ii <= m + 1; ii++) {
        eml_xrotg(&f, &ztest0, &ztest, &rt);
        if (ii > q + 1) {
          e[0] = f;
        }

        f = ztest * s[ii - 1] + rt * e[ii - 1];
        e[ii - 1] = ztest * e[ii - 1] - rt * s[ii - 1];
        ztest0 = rt * s[ii];
        s[ii] *= ztest;
        eml_xrot(V, 1 + 3 * (ii - 1), 1 + 3 * ii, ztest, rt);
        s[ii - 1] = f;
        eml_xrotg(&s[ii - 1], &ztest0, &ztest, &rt);
        f = ztest * e[ii - 1] + rt * s[ii];
        s[ii] = -rt * e[ii - 1] + ztest * s[ii];
        ztest0 = rt * e[ii];
        e[ii] *= ztest;
        eml_xrot(U, 1 + 3 * (ii - 1), 1 + 3 * ii, ztest, rt);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
        eml_xscal(-1.0, V, 1 + 3 * q);
      }

      kase = q + 1;
      while ((q + 1 < 3) && (s[q] < s[kase])) {
        rt = s[q];
        s[q] = s[kase];
        s[kase] = rt;
        eml_xswap(V, 1 + 3 * q, 1 + 3 * (q + 1));
        eml_xswap(U, 1 + 3 * q, 1 + 3 * (q + 1));
        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (ii = 0; ii < 3; ii++) {
    S[ii] = s[ii];
  }
}

//
// Arguments    : int n
//                const double x[9]
//                int ix0
// Return Type  : double
//
static double eml_xnrm2(int n, const double x[9], int ix0)
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
// Arguments    : double x[9]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
static void eml_xrot(double x[9], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 3; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
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
// Arguments    : double a
//                double x[9]
//                int ix0
// Return Type  : void
//
static void eml_xscal(double a, double x[9], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 2; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : double x[9]
//                int ix0
//                int iy0
// Return Type  : void
//
static void eml_xswap(double x[9], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 3; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
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
// Arguments    : emxArray_char_T **pEmxArray
// Return Type  : void
//
static void emxFree_char_T(emxArray_char_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T *)NULL) {
    if (((*pEmxArray)->data != (char *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_char_T *)NULL;
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
// Arguments    : emxArray_uint8_T **pEmxArray
// Return Type  : void
//
static void emxFree_uint8_T(emxArray_uint8_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint8_T *)NULL) {
    if (((*pEmxArray)->data != (unsigned char *)NULL) && (*pEmxArray)
        ->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_uint8_T *)NULL;
  }
}

//
// Arguments    : emxArray_boolean_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
static void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_char_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
static void emxInit_char_T(emxArray_char_T **pEmxArray, int numDimensions)
{
  emxArray_char_T *emxArray;
  int i;
  *pEmxArray = (emxArray_char_T *)malloc(sizeof(emxArray_char_T));
  emxArray = *pEmxArray;
  emxArray->data = (char *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
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
// Arguments    : emxArray_real32_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions)
{
  emxArray_real32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (float *)NULL;
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
// Arguments    : emxArray_uint8_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
static void emxInit_uint8_T(emxArray_uint8_T **pEmxArray, int numDimensions)
{
  emxArray_uint8_T *emxArray;
  int i;
  *pEmxArray = (emxArray_uint8_T *)malloc(sizeof(emxArray_uint8_T));
  emxArray = *pEmxArray;
  emxArray->data = (unsigned char *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : c_vision_internal_calibration_S *pStruct
// Return Type  : void
//
static void f_emxFreeStruct_vision_internal(c_vision_internal_calibration_S
  *pStruct)
{
  e_emxFreeStruct_vision_internal(&pStruct->RectificationParams);
}

//
// Arguments    : c_vision_internal_calibration_S *pStruct
// Return Type  : void
//
static void f_emxInitStruct_vision_internal(c_vision_internal_calibration_S
  *pStruct)
{
  e_emxInitStruct_vision_internal(&pStruct->RectificationParams);
}

//
// Arguments    : d_vision_internal_calibration_C *pStruct
// Return Type  : void
//
static void g_emxFreeStruct_vision_internal(d_vision_internal_calibration_C
  *pStruct)
{
  c_emxFreeStruct_vision_internal(&pStruct->UndistortMap);
}

//
// Arguments    : d_vision_internal_calibration_C *pStruct
// Return Type  : void
//
static void g_emxInitStruct_vision_internal(d_vision_internal_calibration_C
  *pStruct)
{
  d_emxInitStruct_vision_internal(&pStruct->UndistortMap);
}

//
// Arguments    : const emxArray_real_T *varargin_1
//                const double varargin_2[2]
// Return Type  : boolean_T
//
static boolean_T isequal(const emxArray_real_T *varargin_1, const double
  varargin_2[2])
{
  boolean_T p;
  boolean_T b_p;
  int k;
  int32_T exitg2;
  boolean_T exitg1;
  p = false;
  b_p = false;
  k = 0;
  do {
    exitg2 = 0;
    if (k < 2) {
      if (varargin_1->size[k] != 1 + k) {
        exitg2 = 1;
      } else {
        k++;
      }
    } else {
      b_p = true;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  if (b_p && (!(varargin_1->size[1] == 0))) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (!(varargin_1->data[k] == varargin_2[k])) {
        b_p = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (!b_p) {
  } else {
    p = true;
  }

  return p;
}

//
// Arguments    : int idx[8]
//                double x[8]
//                int offset
//                int np
//                int nq
// Return Type  : void
//
static void merge(int idx[8], double x[8], int offset, int np, int nq)
{
  int iwork[8];
  double xwork[8];
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
// Arguments    : const emxArray_real_T *x
//                const emxArray_real_T *y
//                emxArray_real_T *xx
//                emxArray_real_T *yy
// Return Type  : void
//
static void meshgrid(const emxArray_real_T *x, const emxArray_real_T *y,
                     emxArray_real_T *xx, emxArray_real_T *yy)
{
  emxArray_real_T *a;
  int ibtile;
  int jcol;
  int varargin_1_idx_0;
  int k;
  int y_idx_0;
  emxInit_real_T(&a, 2);
  if ((x->size[1] == 0) || (y->size[1] == 0)) {
    ibtile = xx->size[0] * xx->size[1];
    xx->size[0] = 0;
    xx->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)xx, ibtile, (int)sizeof(double));
    ibtile = yy->size[0] * yy->size[1];
    yy->size[0] = 0;
    yy->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)yy, ibtile, (int)sizeof(double));
  } else {
    jcol = x->size[1];
    ibtile = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = jcol;
    emxEnsureCapacity((emxArray__common *)a, ibtile, (int)sizeof(double));
    for (ibtile = 0; ibtile < jcol; ibtile++) {
      a->data[a->size[0] * ibtile] = x->data[ibtile];
    }

    varargin_1_idx_0 = y->size[1];
    ibtile = xx->size[0] * xx->size[1];
    xx->size[0] = varargin_1_idx_0;
    xx->size[1] = a->size[1];
    emxEnsureCapacity((emxArray__common *)xx, ibtile, (int)sizeof(double));
    for (jcol = 0; jcol + 1 <= a->size[1]; jcol++) {
      ibtile = jcol * varargin_1_idx_0;
      for (k = 1; k <= varargin_1_idx_0; k++) {
        xx->data[(ibtile + k) - 1] = a->data[jcol];
      }
    }

    varargin_1_idx_0 = x->size[1];
    y_idx_0 = y->size[1];
    ibtile = yy->size[0] * yy->size[1];
    yy->size[0] = y_idx_0;
    yy->size[1] = varargin_1_idx_0;
    emxEnsureCapacity((emxArray__common *)yy, ibtile, (int)sizeof(double));
    y_idx_0 = y->size[1];
    for (jcol = 1; jcol <= varargin_1_idx_0; jcol++) {
      ibtile = (jcol - 1) * y_idx_0;
      for (k = 1; k <= y_idx_0; k++) {
        yy->data[(ibtile + k) - 1] = y->data[k - 1];
      }
    }
  }

  emxFree_real_T(&a);
}

//
// Arguments    : const double A[9]
//                const double B[9]
//                double y[9]
// Return Type  : void
//
static void mrdivide(const double A[9], const double B[9], double y[9])
{
  double b_A[9];
  int r1;
  int r2;
  int r3;
  double maxval;
  double a21;
  int rtemp;
  memcpy(&b_A[0], &B[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(B[0]);
  a21 = fabs(B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = B[r2] / B[r1];
  b_A[r3] /= b_A[r1];
  b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
  b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
  b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
  b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
  if (fabs(b_A[3 + r3]) > fabs(b_A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[3 + r3] /= b_A[3 + r2];
  b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
  for (rtemp = 0; rtemp < 3; rtemp++) {
    y[rtemp + 3 * r1] = A[rtemp] / b_A[r1];
    y[rtemp + 3 * r2] = A[3 + rtemp] - y[rtemp + 3 * r1] * b_A[3 + r1];
    y[rtemp + 3 * r3] = A[6 + rtemp] - y[rtemp + 3 * r1] * b_A[6 + r1];
    y[rtemp + 3 * r2] /= b_A[3 + r2];
    y[rtemp + 3 * r3] -= y[rtemp + 3 * r2] * b_A[6 + r2];
    y[rtemp + 3 * r3] /= b_A[6 + r3];
    y[rtemp + 3 * r2] -= y[rtemp + 3 * r3] * b_A[3 + r3];
    y[rtemp + 3 * r1] -= y[rtemp + 3 * r3] * b_A[r3];
    y[rtemp + 3 * r1] -= y[rtemp + 3 * r2] * b_A[r2];
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
// Arguments    : const emxArray_real_T *varargin_1
//                emxArray_real_T *b
// Return Type  : void
//
static void padarray(const emxArray_real_T *varargin_1, emxArray_real_T *b)
{
  double b_varargin_1[2];
  int i21;
  int i;
  int j;
  if (varargin_1->size[0] == 0) {
    b_varargin_1[0] = (signed char)varargin_1->size[0];
    i21 = b->size[0] * b->size[1];
    b->size[0] = (int)b_varargin_1[0];
    b->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)b, i21, (int)sizeof(double));
    i = (int)b_varargin_1[0] * 3;
    for (i21 = 0; i21 < i; i21++) {
      b->data[i21] = 1.0;
    }
  } else {
    for (i21 = 0; i21 < 2; i21++) {
      b_varargin_1[i21] = (double)varargin_1->size[i21] + (double)i21;
    }

    i21 = b->size[0] * b->size[1];
    b->size[0] = (int)b_varargin_1[0];
    b->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)b, i21, (int)sizeof(double));
    i21 = b->size[0];
    for (i = 0; i < i21; i++) {
      b->data[i + (b->size[0] << 1)] = 1.0;
    }

    for (j = 0; j < 2; j++) {
      i21 = b->size[0];
      for (i = varargin_1->size[0]; i + 1 <= i21; i++) {
        b->data[i + b->size[0] * j] = 1.0;
      }
    }

    for (j = 0; j < 2; j++) {
      for (i = 0; i < varargin_1->size[0]; i++) {
        b->data[i + b->size[0] * j] = varargin_1->data[i + varargin_1->size[0] *
          j];
      }
    }
  }
}

//
// Arguments    : const emxArray_real_T *a
//                emxArray_real_T *y
// Return Type  : void
//
static void power(const emxArray_real_T *a, emxArray_real_T *y)
{
  unsigned int unnamed_idx_0;
  int k;
  unnamed_idx_0 = (unsigned int)a->size[0];
  k = y->size[0];
  y->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)y, k, (int)sizeof(double));
  for (k = 0; k < (int)unnamed_idx_0; k++) {
    y->data[k] = a->data[k] * a->data[k];
  }
}

//
// Arguments    : const emxArray_real_T *x
//                double y
//                emxArray_real_T *z
// Return Type  : void
//
static void rdivide(const emxArray_real_T *x, double y, emxArray_real_T *z)
{
  int i12;
  int loop_ub;
  i12 = z->size[0];
  z->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)z, i12, (int)sizeof(double));
  loop_ub = x->size[0];
  for (i12 = 0; i12 < loop_ub; i12++) {
    z->data[i12] = x->data[i12] / y;
  }
}

//
// Arguments    : void
// Return Type  : void
//
static void rectifyStereo_free()
{
  d_emxFreeStruct_vision_internal(&stereoParamsSecond);
  f_emxFreeStruct_vision_internal(&stereoParams);
  g_emxFreeStruct_vision_internal(&cameraParams_r);
  g_emxFreeStruct_vision_internal(&cameraParams_l);
}

//
// Arguments    : void
// Return Type  : void
//
static void rectifyStereo_init()
{
  c_emxInitStruct_vision_internal(&stereoParamsSecond);
  f_emxInitStruct_vision_internal(&stereoParams);
  g_emxInitStruct_vision_internal(&cameraParams_r);
  g_emxInitStruct_vision_internal(&cameraParams_l);
}

//
// Arguments    : const double rotationVector[3]
//                double rotationMatrix[9]
// Return Type  : void
//
static void rodriguesVectorToMatrix(const double rotationVector[3], double
  rotationMatrix[9])
{
  double theta;
  int k;
  double n[3];
  double x;
  double a;
  double b_n[9];
  double dv3[9];
  int i14;
  static const signed char b_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  theta = norm(rotationVector);
  if (theta < 2.2204460492503131E-16) {
    memset(&rotationMatrix[0], 0, 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      rotationMatrix[k + 3 * k] = 1.0;
    }
  } else {
    for (k = 0; k < 3; k++) {
      n[k] = rotationVector[k] / theta;
    }

    x = cos(theta);
    a = 1.0 - cos(theta);
    theta = sin(theta);
    b_n[0] = n[0] * n[0];
    b_n[3] = n[0] * n[1];
    b_n[6] = n[0] * n[2];
    b_n[1] = n[1] * n[0];
    b_n[4] = n[1] * n[1];
    b_n[7] = n[1] * n[2];
    b_n[2] = n[2] * n[0];
    b_n[5] = n[2] * n[1];
    b_n[8] = n[2] * n[2];
    dv3[0] = 0.0;
    dv3[3] = -n[2];
    dv3[6] = n[1];
    dv3[1] = n[2];
    dv3[4] = 0.0;
    dv3[7] = -n[0];
    dv3[2] = -n[1];
    dv3[5] = n[0];
    dv3[8] = 0.0;
    for (k = 0; k < 3; k++) {
      for (i14 = 0; i14 < 3; i14++) {
        rotationMatrix[i14 + 3 * k] = ((double)b_a[i14 + 3 * k] * x + a *
          b_n[i14 + 3 * k]) + theta * dv3[i14 + 3 * k];
      }
    }
  }
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
// Arguments    : int numDimensions
//                int *size
// Return Type  : emxArray_uint8_T *
//
emxArray_uint8_T *emxCreateND_uint8_T(int numDimensions, int *size)
{
  emxArray_uint8_T *emx;
  int numEl;
  int i;
  emxInit_uint8_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (unsigned char *)calloc((unsigned int)numEl, sizeof(unsigned char));
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : unsigned char *data
//                int numDimensions
//                int *size
// Return Type  : emxArray_uint8_T *
//
emxArray_uint8_T *emxCreateWrapperND_uint8_T(unsigned char *data, int
  numDimensions, int *size)
{
  emxArray_uint8_T *emx;
  int numEl;
  int i;
  emxInit_uint8_T(&emx, numDimensions);
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
// Arguments    : unsigned char *data
//                int rows
//                int cols
// Return Type  : emxArray_uint8_T *
//
emxArray_uint8_T *emxCreateWrapper_uint8_T(unsigned char *data, int rows, int
  cols)
{
  emxArray_uint8_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  emxInit_uint8_T(&emx, 2);
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
// Return Type  : emxArray_uint8_T *
//
emxArray_uint8_T *emxCreate_uint8_T(int rows, int cols)
{
  emxArray_uint8_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  emxInit_uint8_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (unsigned char *)calloc((unsigned int)numEl, sizeof(unsigned char));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : emxArray_uint8_T *emxArray
// Return Type  : void
//
void emxDestroyArray_uint8_T(emxArray_uint8_T *emxArray)
{
  emxFree_uint8_T(&emxArray);
}

//
// Arguments    : emxArray_uint8_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInitArray_uint8_T(emxArray_uint8_T **pEmxArray, int numDimensions)
{
  emxInit_uint8_T(pEmxArray, numDimensions);
}

//
// RECTIFYSTEREO Summary of this function goes here
//    Detailed explanation goes here
// Arguments    : const unsigned char ImGrayR_lr[360960]
//                const unsigned char ImGrayR_rr[360960]
//                const struct0_T *stereoParamStruct
//                const struct4_T *stereoParamsSecondStruct
//                emxArray_uint8_T *ImGrayR_l
//                emxArray_uint8_T *ImGrayR_r
//                double cameraparams_out[4]
// Return Type  : void
//
void rectifyStereo(const unsigned char ImGrayR_lr[360960], const unsigned char
                   ImGrayR_rr[360960], const struct0_T *stereoParamStruct, const
                   struct4_T *stereoParamsSecondStruct, emxArray_uint8_T
                   *ImGrayR_l, emxArray_uint8_T *ImGrayR_r, double
                   cameraparams_out[4])
{
  c_vision_internal_calibration_C *params;
  double H2_T[9];
  int i0;
  int i1;
  double cameraMatrix_second_l[9];
  double cameraMatrix_second_r[9];
  double c_hoistedGlobal_TranslationOfCa[3];
  double x[4];
  double y;
  int k;
  double b_y;
  double c_y;
  d_vision_internal_calibration_C *cameraParams;
  double tangentialDist[2];
  static const char cv0[5] = { 'u', 'i', 'n', 't', '8' };

  static const char outputView[4] = { 's', 'a', 'm', 'e' };

  emxArray_uint8_T *J;
  emxArray_uint8_T *b_J;
  d_vision_internal_calibration_S *b_stereoParams;
  boolean_T p;
  boolean_T b_p;
  boolean_T exitg2;
  emxArray_char_T *b;
  boolean_T guard1 = false;
  int32_T exitg1;
  static const char cv1[5] = { 'v', 'a', 'l', 'i', 'd' };

  double yBounds[2];
  double xBounds[2];
  double Q[16];
  projective2d H1;
  static const char a[5] = { 'v', 'a', 'l', 'i', 'd' };

  static unsigned char c_J[360960];
  if (!initialized_not_empty) {
    initialized_not_empty = true;

    //    %% camera model
    //   stereoParams = stereoParameters(stereoParamStruct);
    //  stereoParamsSecond = stereoParameters(stereoParamStruct);
    c_StereoParametersImpl_StereoPa(&stereoParams,
      stereoParamStruct->CameraParameters1.RadialDistortion,
      stereoParamStruct->CameraParameters1.TangentialDistortion,
      stereoParamStruct->CameraParameters1.WorldUnits,
      stereoParamStruct->CameraParameters1.NumRadialDistortionCoefficients,
      stereoParamStruct->CameraParameters1.IntrinsicMatrix,
      stereoParamStruct->CameraParameters2.RadialDistortion,
      stereoParamStruct->CameraParameters2.TangentialDistortion,
      stereoParamStruct->CameraParameters2.WorldUnits,
      stereoParamStruct->CameraParameters2.NumRadialDistortionCoefficients,
      stereoParamStruct->CameraParameters2.IntrinsicMatrix,
      stereoParamStruct->TranslationOfCamera2,
      &stereoParamStruct->RectificationParams, &cameraParams_l, &cameraParams_r);
    d_StereoParametersImpl_StereoPa(&stereoParamsSecond,
      stereoParamsSecondStruct->CameraParameters1.RadialDistortion,
      stereoParamsSecondStruct->CameraParameters1.TangentialDistortion,
      stereoParamsSecondStruct->CameraParameters1.WorldUnits,
      stereoParamsSecondStruct->CameraParameters1.NumRadialDistortionCoefficients,
      stereoParamsSecondStruct->CameraParameters1.IntrinsicMatrix,
      stereoParamsSecondStruct->CameraParameters2.RadialDistortion,
      stereoParamsSecondStruct->CameraParameters2.TangentialDistortion,
      stereoParamsSecondStruct->CameraParameters2.WorldUnits,
      stereoParamsSecondStruct->CameraParameters2.NumRadialDistortionCoefficients,
      stereoParamsSecondStruct->CameraParameters2.IntrinsicMatrix,
      stereoParamsSecondStruct->RotationOfCamera2,
      stereoParamsSecondStruct->TranslationOfCamera2,
      &stereoParamsSecondStruct->RectificationParams, &gobj_0, &gobj_1);

    //  left camera
    //  constants
    //  right camera
    //  load second callibration
    params = stereoParamsSecond.CameraParameters1;
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        H2_T[i1 + 3 * i0] = params->IntrinsicMatrixInternal[i0 + 3 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        cameraMatrix_second_l[i1 + 3 * i0] = H2_T[i0 + 3 * i1];
      }
    }

    params = stereoParamsSecond.CameraParameters2;
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        H2_T[i1 + 3 * i0] = params->IntrinsicMatrixInternal[i0 + 3 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        cameraMatrix_second_r[i1 + 3 * i0] = H2_T[i0 + 3 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      c_hoistedGlobal_TranslationOfCa[i0] = stereoParams.TranslationOfCamera2[i0];
    }

    x[0] = cameraMatrix_second_l[0];
    x[1] = cameraMatrix_second_l[4];
    x[2] = cameraMatrix_second_r[0];
    x[3] = cameraMatrix_second_r[4];
    y = cameraMatrix_second_l[0];
    for (k = 0; k < 3; k++) {
      y += x[k + 1];
    }

    x[0] = cameraMatrix_second_l[6];
    x[1] = cameraMatrix_second_l[7];
    x[2] = cameraMatrix_second_r[6];
    x[3] = cameraMatrix_second_r[7];
    b_y = cameraMatrix_second_l[6];
    for (k = 0; k < 3; k++) {
      b_y += x[k + 1];
    }

    x[0] = cameraMatrix_second_l[6];
    x[1] = cameraMatrix_second_l[7];
    x[2] = cameraMatrix_second_r[6];
    x[3] = cameraMatrix_second_r[7];
    c_y = cameraMatrix_second_l[6];
    for (k = 0; k < 3; k++) {
      c_y += x[k + 1];
    }

    cameraparams[0] = y / 4.0;
    cameraparams[1] = b_y / 4.0;
    cameraparams[2] = c_y / 4.0;
    cameraparams[3] = norm(c_hoistedGlobal_TranslationOfCa) / 1000.0;
  }

  // % Re-create the Stereo Parameters
  cameraParams = &cameraParams_l;
  if (ImageTransformer_needToUpdate(&cameraParams_l.UndistortMap)) {
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        H2_T[i1 + 3 * i0] = cameraParams->IntrinsicMatrixInternal[i0 + 3 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      c_hoistedGlobal_TranslationOfCa[i0] = cameraParams->RadialDistortion[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      tangentialDist[i0] = cameraParams->TangentialDistortion[i0];
    }

    i0 = cameraParams->UndistortMap.SizeOfImage->size[0] *
      cameraParams->UndistortMap.SizeOfImage->size[1];
    cameraParams->UndistortMap.SizeOfImage->size[0] = 1;
    cameraParams->UndistortMap.SizeOfImage->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)cameraParams->UndistortMap.SizeOfImage,
                      i0, (int)sizeof(double));
    for (i0 = 0; i0 < 2; i0++) {
      cameraParams->UndistortMap.SizeOfImage->data[i0] = 480.0 + 272.0 * (double)
        i0;
    }

    i0 = cameraParams->UndistortMap.ClassOfImage->size[0] *
      cameraParams->UndistortMap.ClassOfImage->size[1];
    cameraParams->UndistortMap.ClassOfImage->size[0] = 1;
    cameraParams->UndistortMap.ClassOfImage->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)
                      cameraParams->UndistortMap.ClassOfImage, i0, (int)sizeof
                      (char));
    for (i0 = 0; i0 < 5; i0++) {
      cameraParams->UndistortMap.ClassOfImage->data[i0] = cv0[i0];
    }

    i0 = cameraParams->UndistortMap.OutputView->size[0] *
      cameraParams->UndistortMap.OutputView->size[1];
    cameraParams->UndistortMap.OutputView->size[0] = 1;
    cameraParams->UndistortMap.OutputView->size[1] = 4;
    emxEnsureCapacity((emxArray__common *)cameraParams->UndistortMap.OutputView,
                      i0, (int)sizeof(char));
    for (i0 = 0; i0 < 4; i0++) {
      cameraParams->UndistortMap.OutputView->data[i0] = outputView[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      cameraParams->UndistortMap.XBounds[i0] = 1.0 + 751.0 * (double)i0;
    }

    for (i0 = 0; i0 < 2; i0++) {
      cameraParams->UndistortMap.YBounds[i0] = 1.0 + 479.0 * (double)i0;
    }

    ImageTransformer_computeMap(&cameraParams->UndistortMap, H2_T,
      c_hoistedGlobal_TranslationOfCa, tangentialDist);
  }

  emxInit_uint8_T(&J, 2);
  ImageTransformer_transformImage(&cameraParams->UndistortMap, ImGrayR_lr, J,
    tangentialDist);
  cameraParams = &cameraParams_r;
  if (ImageTransformer_needToUpdate(&cameraParams_r.UndistortMap)) {
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        H2_T[i1 + 3 * i0] = cameraParams->IntrinsicMatrixInternal[i0 + 3 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      c_hoistedGlobal_TranslationOfCa[i0] = cameraParams->RadialDistortion[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      tangentialDist[i0] = cameraParams->TangentialDistortion[i0];
    }

    i0 = cameraParams->UndistortMap.SizeOfImage->size[0] *
      cameraParams->UndistortMap.SizeOfImage->size[1];
    cameraParams->UndistortMap.SizeOfImage->size[0] = 1;
    cameraParams->UndistortMap.SizeOfImage->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)cameraParams->UndistortMap.SizeOfImage,
                      i0, (int)sizeof(double));
    for (i0 = 0; i0 < 2; i0++) {
      cameraParams->UndistortMap.SizeOfImage->data[i0] = 480.0 + 272.0 * (double)
        i0;
    }

    i0 = cameraParams->UndistortMap.ClassOfImage->size[0] *
      cameraParams->UndistortMap.ClassOfImage->size[1];
    cameraParams->UndistortMap.ClassOfImage->size[0] = 1;
    cameraParams->UndistortMap.ClassOfImage->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)
                      cameraParams->UndistortMap.ClassOfImage, i0, (int)sizeof
                      (char));
    for (i0 = 0; i0 < 5; i0++) {
      cameraParams->UndistortMap.ClassOfImage->data[i0] = cv0[i0];
    }

    i0 = cameraParams->UndistortMap.OutputView->size[0] *
      cameraParams->UndistortMap.OutputView->size[1];
    cameraParams->UndistortMap.OutputView->size[0] = 1;
    cameraParams->UndistortMap.OutputView->size[1] = 4;
    emxEnsureCapacity((emxArray__common *)cameraParams->UndistortMap.OutputView,
                      i0, (int)sizeof(char));
    for (i0 = 0; i0 < 4; i0++) {
      cameraParams->UndistortMap.OutputView->data[i0] = outputView[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      cameraParams->UndistortMap.XBounds[i0] = 1.0 + 751.0 * (double)i0;
    }

    for (i0 = 0; i0 < 2; i0++) {
      cameraParams->UndistortMap.YBounds[i0] = 1.0 + 479.0 * (double)i0;
    }

    b_ImageTransformer_computeMap(&cameraParams->UndistortMap, H2_T,
      c_hoistedGlobal_TranslationOfCa, tangentialDist);
  }

  emxInit_uint8_T(&b_J, 2);
  ImageTransformer_transformImage(&cameraParams->UndistortMap, ImGrayR_rr, b_J,
    tangentialDist);
  b_stereoParams = &stereoParamsSecond;
  for (i0 = 0; i0 < 2; i0++) {
    tangentialDist[i0] =
      stereoParamsSecond.RectificationParams.OriginalImageSize[i0];
  }

  p = false;
  b_p = true;
  k = 0;
  exitg2 = false;
  while ((!exitg2) && (k < 2)) {
    if (!(480.0 + 272.0 * (double)k == tangentialDist[k])) {
      b_p = false;
      exitg2 = true;
    } else {
      k++;
    }
  }

  if (!b_p) {
  } else {
    p = true;
  }

  emxInit_char_T(&b, 2);
  guard1 = false;
  if (p) {
    i0 = b->size[0] * b->size[1];
    b->size[0] = 1;
    b->size[1] = stereoParamsSecond.RectificationParams.OutputView->size[1];
    emxEnsureCapacity((emxArray__common *)b, i0, (int)sizeof(char));
    k = stereoParamsSecond.RectificationParams.OutputView->size[0] *
      stereoParamsSecond.RectificationParams.OutputView->size[1];
    for (i0 = 0; i0 < k; i0++) {
      b->data[i0] = stereoParamsSecond.RectificationParams.OutputView->data[i0];
    }

    p = false;
    if (5 != b->size[1]) {
    } else {
      k = 0;
      do {
        exitg1 = 0;
        if (k < 5) {
          if (cv1[k] != b->data[k]) {
            exitg1 = 1;
          } else {
            k++;
          }
        } else {
          p = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (p) {
      p = true;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    p = false;
  }

  emxFree_char_T(&b);
  if ((!p) || b_ImageTransformer_needToUpdate(&stereoParamsSecond.RectifyMap1) ||
      b_ImageTransformer_needToUpdate(&b_stereoParams->RectifyMap2)) {
    p = true;
  } else {
    p = false;
  }

  if (p) {
    c_StereoParametersImpl_computeR(b_stereoParams, H1.T, H2_T, Q, xBounds,
      yBounds);
    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectificationParams.OriginalImageSize[i0] = 480.0 + 272.0 *
        (double)i0;
    }

    b_stereoParams->RectificationParams.H1 = H1;
    for (i0 = 0; i0 < 9; i0++) {
      b_stereoParams->RectificationParams.H2.T[i0] = H2_T[i0];
    }

    i0 = b_stereoParams->RectificationParams.OutputView->size[0] *
      b_stereoParams->RectificationParams.OutputView->size[1];
    b_stereoParams->RectificationParams.OutputView->size[0] = 1;
    b_stereoParams->RectificationParams.OutputView->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)
                      b_stereoParams->RectificationParams.OutputView, i0, (int)
                      sizeof(char));
    for (i0 = 0; i0 < 5; i0++) {
      b_stereoParams->RectificationParams.OutputView->data[i0] = a[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectificationParams.XBounds[i0] = xBounds[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectificationParams.YBounds[i0] = yBounds[i0];
    }

    params = b_stereoParams->CameraParameters1;
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        H2_T[i1 + 3 * i0] = params->IntrinsicMatrixInternal[i0 + 3 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      c_hoistedGlobal_TranslationOfCa[i0] = params->RadialDistortion[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      tangentialDist[i0] = params->TangentialDistortion[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      xBounds[i0] = b_stereoParams->RectificationParams.XBounds[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      yBounds[i0] = b_stereoParams->RectificationParams.YBounds[i0];
    }

    H1 = b_stereoParams->RectificationParams.H1;
    i0 = b_stereoParams->RectifyMap1.SizeOfImage->size[0] *
      b_stereoParams->RectifyMap1.SizeOfImage->size[1];
    b_stereoParams->RectifyMap1.SizeOfImage->size[0] = 1;
    b_stereoParams->RectifyMap1.SizeOfImage->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)
                      b_stereoParams->RectifyMap1.SizeOfImage, i0, (int)sizeof
                      (double));
    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectifyMap1.SizeOfImage->data[i0] = 480.0 + 272.0 *
        (double)i0;
    }

    i0 = b_stereoParams->RectifyMap1.ClassOfImage->size[0] *
      b_stereoParams->RectifyMap1.ClassOfImage->size[1];
    b_stereoParams->RectifyMap1.ClassOfImage->size[0] = 1;
    b_stereoParams->RectifyMap1.ClassOfImage->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)
                      b_stereoParams->RectifyMap1.ClassOfImage, i0, (int)sizeof
                      (char));
    for (i0 = 0; i0 < 5; i0++) {
      b_stereoParams->RectifyMap1.ClassOfImage->data[i0] = cv0[i0];
    }

    i0 = b_stereoParams->RectifyMap1.OutputView->size[0] *
      b_stereoParams->RectifyMap1.OutputView->size[1];
    b_stereoParams->RectifyMap1.OutputView->size[0] = 1;
    b_stereoParams->RectifyMap1.OutputView->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)b_stereoParams->RectifyMap1.OutputView,
                      i0, (int)sizeof(char));
    for (i0 = 0; i0 < 5; i0++) {
      b_stereoParams->RectifyMap1.OutputView->data[i0] = a[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectifyMap1.XBounds[i0] = xBounds[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectifyMap1.YBounds[i0] = yBounds[i0];
    }

    d_ImageTransformer_computeMap(&b_stereoParams->RectifyMap1, H2_T,
      c_hoistedGlobal_TranslationOfCa, tangentialDist, H1.T);
    params = b_stereoParams->CameraParameters2;
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        H2_T[i1 + 3 * i0] = params->IntrinsicMatrixInternal[i0 + 3 * i1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      c_hoistedGlobal_TranslationOfCa[i0] = params->RadialDistortion[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      tangentialDist[i0] = params->TangentialDistortion[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      xBounds[i0] = b_stereoParams->RectificationParams.XBounds[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      yBounds[i0] = b_stereoParams->RectificationParams.YBounds[i0];
    }

    H1 = b_stereoParams->RectificationParams.H2;
    i0 = b_stereoParams->RectifyMap2.SizeOfImage->size[0] *
      b_stereoParams->RectifyMap2.SizeOfImage->size[1];
    b_stereoParams->RectifyMap2.SizeOfImage->size[0] = 1;
    b_stereoParams->RectifyMap2.SizeOfImage->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)
                      b_stereoParams->RectifyMap2.SizeOfImage, i0, (int)sizeof
                      (double));
    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectifyMap2.SizeOfImage->data[i0] = 480.0 + 272.0 *
        (double)i0;
    }

    i0 = b_stereoParams->RectifyMap2.ClassOfImage->size[0] *
      b_stereoParams->RectifyMap2.ClassOfImage->size[1];
    b_stereoParams->RectifyMap2.ClassOfImage->size[0] = 1;
    b_stereoParams->RectifyMap2.ClassOfImage->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)
                      b_stereoParams->RectifyMap2.ClassOfImage, i0, (int)sizeof
                      (char));
    for (i0 = 0; i0 < 5; i0++) {
      b_stereoParams->RectifyMap2.ClassOfImage->data[i0] = cv0[i0];
    }

    i0 = b_stereoParams->RectifyMap2.OutputView->size[0] *
      b_stereoParams->RectifyMap2.OutputView->size[1];
    b_stereoParams->RectifyMap2.OutputView->size[0] = 1;
    b_stereoParams->RectifyMap2.OutputView->size[1] = 5;
    emxEnsureCapacity((emxArray__common *)b_stereoParams->RectifyMap2.OutputView,
                      i0, (int)sizeof(char));
    for (i0 = 0; i0 < 5; i0++) {
      b_stereoParams->RectifyMap2.OutputView->data[i0] = a[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectifyMap2.XBounds[i0] = xBounds[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      b_stereoParams->RectifyMap2.YBounds[i0] = yBounds[i0];
    }

    e_ImageTransformer_computeMap(&b_stereoParams->RectifyMap2, H2_T,
      c_hoistedGlobal_TranslationOfCa, tangentialDist, H1.T);
  }

  k = J->size[0];
  for (i0 = 0; i0 < 752; i0++) {
    for (i1 = 0; i1 < 480; i1++) {
      c_J[i1 + 480 * i0] = J->data[i1 + k * i0];
    }
  }

  emxFree_uint8_T(&J);
  b_ImageTransformer_transformIma(&b_stereoParams->RectifyMap1, c_J, ImGrayR_l);
  k = b_J->size[0];
  for (i0 = 0; i0 < 752; i0++) {
    for (i1 = 0; i1 < 480; i1++) {
      c_J[i1 + 480 * i0] = b_J->data[i1 + k * i0];
    }
  }

  emxFree_uint8_T(&b_J);
  b_ImageTransformer_transformIma(&b_stereoParams->RectifyMap2, c_J, ImGrayR_r);
  for (i0 = 0; i0 < 4; i0++) {
    cameraparams_out[i0] = cameraparams[i0];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rectifyStereo_initialize()
{
  rt_InitInfAndNaN(8U);
  initialized_not_empty = false;
  rectifyStereo_init();
}

//
// Arguments    : void
// Return Type  : void
//
void rectifyStereo_terminate()
{
  rectifyStereo_free();
}

//
// File trailer for rectifyStereo.cpp
//
// [EOF]
//
