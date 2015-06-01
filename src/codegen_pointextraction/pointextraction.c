/*
 * File: pointextraction.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 01-Jun-2015 17:14:17
 */

/* Include files */
#include "rt_nonfinite.h"
#include "pointextraction.h"
#include "cvstCG_detectFAST.h"
#include "cvstCG_pointTracker.h"
#include "libmwmorphop_flat_tbb.h"
#include "tmwtypes.h"

/* Type Definitions */
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

#endif                                 /*struct_emxArray__common*/

#ifndef typedef_emxArray__common
#define typedef_emxArray__common

typedef struct emxArray__common emxArray__common;

#endif                                 /*typedef_emxArray__common*/

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T

struct emxArray_int8_T
{
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_int8_T*/

#ifndef typedef_emxArray_int8_T
#define typedef_emxArray_int8_T

typedef struct emxArray_int8_T emxArray_int8_T;

#endif                                 /*typedef_emxArray_int8_T*/

#ifndef typedef_b_struct_T
#define typedef_b_struct_T

typedef struct {
  double offset[2];
  double count;
  emxArray_int8_T *grid;
} b_struct_T;

#endif                                 /*typedef_b_struct_T*/

#ifndef struct_emxArray_b_struct_T
#define struct_emxArray_b_struct_T

struct emxArray_b_struct_T
{
  b_struct_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_b_struct_T*/

#ifndef typedef_emxArray_b_struct_T
#define typedef_emxArray_b_struct_T

typedef struct emxArray_b_struct_T emxArray_b_struct_T;

#endif                                 /*typedef_emxArray_b_struct_T*/

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

#endif                                 /*struct_emxArray_boolean_T*/

#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T

typedef struct emxArray_boolean_T emxArray_boolean_T;

#endif                                 /*typedef_emxArray_boolean_T*/

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

#endif                                 /*struct_emxArray_int32_T*/

#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T

typedef struct emxArray_int32_T emxArray_int32_T;

#endif                                 /*typedef_emxArray_int32_T*/

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

#endif                                 /*struct_emxArray_real32_T*/

#ifndef typedef_emxArray_real32_T
#define typedef_emxArray_real32_T

typedef struct emxArray_real32_T emxArray_real32_T;

#endif                                 /*typedef_emxArray_real32_T*/

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

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
  double offset[2];
  double featureCandidates[64];
} struct_T;

#endif                                 /*typedef_struct_T*/

#ifndef struct_emxArray_struct_T
#define struct_emxArray_struct_T

struct emxArray_struct_T
{
  struct_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_struct_T*/

#ifndef typedef_emxArray_struct_T
#define typedef_emxArray_struct_T

typedef struct emxArray_struct_T emxArray_struct_T;

#endif                                 /*typedef_emxArray_struct_T*/

#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T

struct emxArray_uint8_T
{
  unsigned char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_uint8_T*/

#ifndef typedef_emxArray_uint8_T
#define typedef_emxArray_uint8_T

typedef struct emxArray_uint8_T emxArray_uint8_T;

#endif                                 /*typedef_emxArray_uint8_T*/

#ifndef struct_vision_ImageDataTypeConverter_0
#define struct_vision_ImageDataTypeConverter_0

struct vision_ImageDataTypeConverter_0
{
  boolean_T S0_isInitialized;
  boolean_T S1_isReleased;
};

#endif                                 /*struct_vision_ImageDataTypeConverter_0*/

#ifndef typedef_vision_ImageDataTypeConverter_0
#define typedef_vision_ImageDataTypeConverter_0

typedef struct vision_ImageDataTypeConverter_0 vision_ImageDataTypeConverter_0;

#endif                                 /*typedef_vision_ImageDataTypeConverter_0*/

#ifndef struct_vision_ColorSpaceConverter_1
#define struct_vision_ColorSpaceConverter_1

struct vision_ColorSpaceConverter_1
{
  boolean_T SystemObjectStructType;
};

#endif                                 /*struct_vision_ColorSpaceConverter_1*/

#ifndef typedef_vision_ColorSpaceConverter_1
#define typedef_vision_ColorSpaceConverter_1

typedef struct vision_ColorSpaceConverter_1 vision_ColorSpaceConverter_1;

#endif                                 /*typedef_vision_ColorSpaceConverter_1*/

#ifndef typedef_c_visioncodegen_ColorSpaceConve
#define typedef_c_visioncodegen_ColorSpaceConve

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  boolean_T isInitialized;
  boolean_T isReleased;
  vision_ColorSpaceConverter_1 cSFunObject;
} c_visioncodegen_ColorSpaceConve;

#endif                                 /*typedef_c_visioncodegen_ColorSpaceConve*/

#ifndef struct_vision_ImageDataTypeConverter_2
#define struct_vision_ImageDataTypeConverter_2

struct vision_ImageDataTypeConverter_2
{
  boolean_T SystemObjectStructType;
};

#endif                                 /*struct_vision_ImageDataTypeConverter_2*/

#ifndef typedef_vision_ImageDataTypeConverter_2
#define typedef_vision_ImageDataTypeConverter_2

typedef struct vision_ImageDataTypeConverter_2 vision_ImageDataTypeConverter_2;

#endif                                 /*typedef_vision_ImageDataTypeConverter_2*/

#ifndef typedef_vision_PointTracker_4
#define typedef_vision_PointTracker_4

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  boolean_T isInitialized;
  boolean_T isReleased;
  unsigned int inputVarSize1[8];
  void * pTracker;
  double FrameSize[2];
  double NumPoints;
  boolean_T IsRGB;
  char FrameClass[5];
  boolean_T IsInitialized;
  c_visioncodegen_ColorSpaceConve ColorConverter;
  vision_ImageDataTypeConverter_2 ImageDataTypeConverterCG;
} vision_PointTracker_4;

#endif                                 /*typedef_vision_PointTracker_4*/

/* Variable Definitions */
static boolean_T initialized_not_empty;
static vision_PointTracker_4 pointTracker;
static double prevPoints[64];
static double prevDisp[32];
static vision_ImageDataTypeConverter_0 h2;
static boolean_T h2_not_empty;

/* Function Declarations */
static double FeaturePointsImpl_get_Count(const emxArray_real32_T
  *this_pLocation);
static vision_PointTracker_4 *PointTracker_PointTracker(vision_PointTracker_4
  *obj);
static void PointTracker_initialize(vision_PointTracker_4 *obj, const double
  points_data[], const int points_size[2], const unsigned char I[332220]);
static void all(const double x[64], boolean_T y[32]);
static boolean_T any(const boolean_T x[2]);
static boolean_T b_any(const emxArray_boolean_T *x);
static void b_ceil(double x[2]);
static void b_detectFASTFeatures(const emxArray_uint8_T *I, emxArray_real32_T
  *pts_pLocation, emxArray_real32_T *pts_pMetric);
static void b_eml_li_find(const boolean_T x[32], int y_data[], int y_size[1]);
static void b_eml_sort(const emxArray_real32_T *x, int dim, emxArray_real32_T *y,
  emxArray_int32_T *idx);
static void b_emxFree_struct_T(emxArray_b_struct_T **pEmxArray);
static void b_emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
  numDimensions);
static void b_emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
static void b_emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void b_emxInit_struct_T(emxArray_b_struct_T **pEmxArray, int
  numDimensions);
static void b_repmat(const double a[2], double varargin_1, emxArray_real_T *b);
static void b_round(emxArray_real32_T *x);
static boolean_T c_any(const emxArray_int8_T *x);
static void c_eml_sort(const emxArray_real_T *x, emxArray_real_T *y,
  emxArray_int32_T *idx);
static void cornerPoints_cg_cornerPoints_cg(const emxArray_real32_T *varargin_1,
  const emxArray_real32_T *varargin_3, emxArray_real32_T *this_pLocation,
  emxArray_real32_T *this_pMetric);
static void detectFASTFeatures(const emxArray_uint8_T *I, emxArray_real32_T
  *pts_pLocation, emxArray_real32_T *pts_pMetric);
static int div_s32(int numerator, int denominator);
static void eml_li_find(const emxArray_boolean_T *x, emxArray_int32_T *y);
static void eml_sort(const emxArray_real32_T *x, emxArray_real32_T *y,
                     emxArray_int32_T *idx);
static void emxCopyMatrix_real_T(double dst[2], const double src[2]);
static void emxCopyStruct_struct_T(b_struct_T *dst, const b_struct_T *src);
static void emxCopy_int8_T(emxArray_int8_T **dst, emxArray_int8_T * const *src);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxEnsureCapacity_struct_T(emxArray_b_struct_T *emxArray, int
  oldNumel);
static void emxExpand_struct_T(emxArray_b_struct_T *emxArray, int fromIndex, int
  toIndex);
static void emxFreeStruct_struct_T(b_struct_T *pStruct);
static void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_int8_T(emxArray_int8_T **pEmxArray);
static void emxFree_real32_T(emxArray_real32_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxFree_struct_T(emxArray_struct_T **pEmxArray);
static void emxFree_uint8_T(emxArray_uint8_T **pEmxArray);
static void emxInitStruct_struct_T(b_struct_T *pStruct);
static void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
static void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void emxInit_struct_T(emxArray_struct_T **pEmxArray, int numDimensions);
static void emxInit_uint8_T(emxArray_uint8_T **pEmxArray, int numDimensions);
static void emxTrim_struct_T(emxArray_b_struct_T *emxArray, int fromIndex, int
  toIndex);
static void im2col(emxArray_real_T *a, const double block[2], emxArray_real_T *b);
static void imerode(const emxArray_int8_T *A, emxArray_int8_T *B);
static void initFeatures(const unsigned char greyImg[332220], double
  borderLength, double numBinsX, double numBinsY, double minDistance, double
  features[64], double featStatus[32], double *threshold, double *flag);
static void kron(const emxArray_int8_T *A, const emxArray_int8_T *B,
                 emxArray_int8_T *K);
static void pointextraction_init(void);
static void rdivide(const double x[2], const double y[2], double z[2]);
static boolean_T refillFeature(const unsigned char greyImg[332220], double
  features[64], double featStatus[32], double numAnchors, double numBinsX,
  double numBinsY, double borderLength, double minDistance);
static void repmat(const b_struct_T a, double varargin_1, emxArray_b_struct_T *b);
static double rt_roundd_snf(double u);
static void stereoRight2Left(const unsigned char imgR[332220], const unsigned
  char imgL[332220], const double pointsR_data[], const int pointsR_size[2],
  double minDisp_data[], int minDisp_size[1], double maxDisp_data[], int
  maxDisp_size[1], double pointsL_data[], int pointsL_size[2], boolean_T
  status_data[], int status_size[1]);

/* Function Definitions */

/*
 * Arguments    : const emxArray_real32_T *this_pLocation
 * Return Type  : double
 */
static double FeaturePointsImpl_get_Count(const emxArray_real32_T
  *this_pLocation)
{
  return this_pLocation->size[0];
}

/*
 * Arguments    : vision_PointTracker_4 *obj
 * Return Type  : vision_PointTracker_4 *
 */
static vision_PointTracker_4 *PointTracker_PointTracker(vision_PointTracker_4
  *obj)
{
  vision_PointTracker_4 *b_obj;
  vision_PointTracker_4 *c_obj;
  c_visioncodegen_ColorSpaceConve *d_obj;
  void * ptrObj;
  b_obj = obj;
  b_obj->IsRGB = false;
  c_obj = b_obj;
  c_obj->matlabCodegenIsDeleted = false;
  c_obj->isInitialized = false;
  c_obj->isReleased = false;
  c_obj->matlabCodegenIsDeleted = false;
  d_obj = &b_obj->ColorConverter;
  d_obj->matlabCodegenIsDeleted = false;
  d_obj->isInitialized = false;
  d_obj->isReleased = false;
  d_obj->matlabCodegenIsDeleted = false;
  d_obj->matlabCodegenIsDeleted = false;
  ptrObj = NULL;
  pointTracker_construct(&ptrObj);
  c_obj = b_obj;
  c_obj->pTracker = ptrObj;
  return b_obj;
}

/*
 * Arguments    : vision_PointTracker_4 *obj
 *                const double points_data[]
 *                const int points_size[2]
 *                const unsigned char I[332220]
 * Return Type  : void
 */
static void PointTracker_initialize(vision_PointTracker_4 *obj, const double
  points_data[], const int points_size[2], const unsigned char I[332220])
{
  int ixstart;
  int blockW;
  float b_points_data[64];
  vision_PointTracker_4 *b_obj;
  static const char value[5] = { 'u', 'i', 'n', 't', '8' };

  double varargin_1[2];
  double fdbl;
  boolean_T exitg1;
  int eint;
  void * ptrObj;
  static unsigned char Iu8_gray[332220];
  float c_points_data[64];
  cvstPTStruct_T paramStruct;
  ixstart = points_size[0] * points_size[1];
  for (blockW = 0; blockW < ixstart; blockW++) {
    b_points_data[blockW] = (float)points_data[blockW];
  }

  b_obj = obj;
  for (blockW = 0; blockW < 5; blockW++) {
    b_obj->FrameClass[blockW] = value[blockW];
  }

  b_obj = obj;
  for (blockW = 0; blockW < 2; blockW++) {
    b_obj->FrameSize[blockW] = 452.0 + 283.0 * (double)blockW;
  }

  b_obj = obj;
  b_obj->NumPoints = points_size[0];
  b_obj = obj;
  for (blockW = 0; blockW < 2; blockW++) {
    varargin_1[blockW] = b_obj->FrameSize[blockW];
  }

  ixstart = 1;
  fdbl = varargin_1[0];
  if (rtIsNaN(varargin_1[0])) {
    blockW = 2;
    exitg1 = false;
    while ((!exitg1) && (blockW < 3)) {
      ixstart = 2;
      if (!rtIsNaN(varargin_1[1])) {
        fdbl = varargin_1[1];
        exitg1 = true;
      } else {
        blockW = 3;
      }
    }
  }

  if ((ixstart < 2) && (varargin_1[1] < fdbl)) {
    fdbl = varargin_1[1];
  }

  if (fdbl == 0.0) {
    fdbl = rtMinusInf;
  } else if (fdbl < 0.0) {
    fdbl = rtNaN;
  } else {
    if ((!rtIsInf(fdbl)) && (!rtIsNaN(fdbl))) {
      if ((!rtIsInf(fdbl)) && (!rtIsNaN(fdbl))) {
        fdbl = frexp(fdbl, &eint);
      } else {
        eint = 0;
      }

      if (fdbl == 0.5) {
        fdbl = (double)eint - 1.0;
      } else {
        fdbl = log(fdbl) / 0.69314718055994529 + (double)eint;
      }
    }
  }

  ptrObj = obj->pTracker;
  for (blockW = 0; blockW < 452; blockW++) {
    for (ixstart = 0; ixstart < 735; ixstart++) {
      Iu8_gray[ixstart + 735 * blockW] = I[blockW + 452 * ixstart];
    }
  }

  ixstart = points_size[0] << 1;
  for (blockW = 0; blockW < ixstart; blockW++) {
    c_points_data[blockW] = b_points_data[blockW];
  }

  ixstart = (int32_T)(21.0);
  blockW = (int32_T)(21.0);
  paramStruct.blockSize[0] = ixstart;
  paramStruct.blockSize[1] = blockW;
  ixstart = (int32_T)(fmax(0.0, fmin(floor(fdbl - 2.0), 4.0)));
  paramStruct.numPyramidLevels = ixstart;
  fdbl = (double)(30.0);
  paramStruct.maxIterations = fdbl;
  paramStruct.epsilon = 0.01;
  paramStruct.maxBidirectionalError = -1.0;
  pointTracker_initialize(ptrObj, Iu8_gray, 452, 735, &c_points_data[0],
    points_size[0], &paramStruct);
  b_obj = obj;
  b_obj->IsInitialized = true;
}

/*
 * Arguments    : const double x[64]
 *                boolean_T y[32]
 * Return Type  : void
 */
static void all(const double x[64], boolean_T y[32])
{
  int i;
  int i1;
  int i2;
  int j;
  int ix;
  boolean_T exitg1;
  for (i = 0; i < 32; i++) {
    y[i] = true;
  }

  i = -1;
  i1 = 0;
  i2 = 32;
  for (j = 0; j < 32; j++) {
    i1++;
    i2++;
    i++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2)) {
      if (x[ix - 1] == 0.0) {
        y[i] = false;
        exitg1 = true;
      } else {
        ix += 32;
      }
    }
  }
}

/*
 * Arguments    : const boolean_T x[2]
 * Return Type  : boolean_T
 */
static boolean_T any(const boolean_T x[2])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!(x[k] == 0)) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/*
 * Arguments    : const emxArray_boolean_T *x
 * Return Type  : boolean_T
 */
static boolean_T b_any(const emxArray_boolean_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b0;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    b0 = (x->data[ix - 1] == 0);
    if (!b0) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

/*
 * Arguments    : double x[2]
 * Return Type  : void
 */
static void b_ceil(double x[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    x[k] = ceil(x[k]);
  }
}

/*
 * Arguments    : const emxArray_uint8_T *I
 *                emxArray_real32_T *pts_pLocation
 *                emxArray_real32_T *pts_pMetric
 * Return Type  : void
 */
static void b_detectFASTFeatures(const emxArray_uint8_T *I, emxArray_real32_T
  *pts_pLocation, emxArray_real32_T *pts_pMetric)
{
  vision_ImageDataTypeConverter_0 *obj;
  emxArray_uint8_T *Iu8;
  int i12;
  int out_numel;
  emxArray_real32_T *rawPts_metric;
  emxArray_real32_T *rawPts_loc;
  void * ptrKeypoints;
  emxArray_real32_T *locations;
  emxArray_real32_T *metricValues;
  float threshold;
  int ix;
  boolean_T exitg1;
  emxArray_boolean_T *validIndex;
  emxArray_int32_T *r4;
  if (!h2_not_empty) {
    obj = &h2;

    /* System object Constructor function: vision.ImageDataTypeConverter */
    obj->S0_isInitialized = false;
    obj->S1_isReleased = false;
    h2_not_empty = true;
  }

  emxInit_uint8_T(&Iu8, 2);

  /* System object Outputs function: vision.ImageDataTypeConverter */
  i12 = Iu8->size[0] * Iu8->size[1];
  Iu8->size[0] = I->size[0];
  Iu8->size[1] = I->size[1];
  emxEnsureCapacity((emxArray__common *)Iu8, i12, (int)sizeof(unsigned char));
  out_numel = I->size[0] * I->size[1];
  for (i12 = 0; i12 < out_numel; i12++) {
    Iu8->data[i12] = I->data[i12];
  }

  emxInit_real32_T(&rawPts_metric, 1);
  b_emxInit_real32_T(&rawPts_loc, 2);
  ptrKeypoints = NULL;
  out_numel = detectFAST_compute(&Iu8->data[0], I->size[0], I->size[1], false,
    51, &ptrKeypoints);
  i12 = rawPts_loc->size[0] * rawPts_loc->size[1];
  rawPts_loc->size[0] = out_numel;
  rawPts_loc->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)rawPts_loc, i12, (int)sizeof(float));
  i12 = rawPts_metric->size[0];
  rawPts_metric->size[0] = out_numel;
  emxEnsureCapacity((emxArray__common *)rawPts_metric, i12, (int)sizeof(float));
  detectFAST_assignOutput(ptrKeypoints, &rawPts_loc->data[0],
    &rawPts_metric->data[0]);
  emxFree_uint8_T(&Iu8);
  b_emxInit_real32_T(&locations, 2);
  emxInit_real32_T(&metricValues, 1);
  if (!(rawPts_metric->size[0] == 0)) {
    out_numel = 1;
    threshold = rawPts_metric->data[0];
    if (rawPts_metric->size[0] > 1) {
      if (rtIsNaNF(rawPts_metric->data[0])) {
        ix = 2;
        exitg1 = false;
        while ((!exitg1) && (ix <= rawPts_metric->size[0])) {
          out_numel = ix;
          if (!rtIsNaNF(rawPts_metric->data[ix - 1])) {
            threshold = rawPts_metric->data[ix - 1];
            exitg1 = true;
          } else {
            ix++;
          }
        }
      }

      if (out_numel < rawPts_metric->size[0]) {
        while (out_numel + 1 <= rawPts_metric->size[0]) {
          if (rawPts_metric->data[out_numel] > threshold) {
            threshold = rawPts_metric->data[out_numel];
          }

          out_numel++;
        }
      }
    }

    b_emxInit_boolean_T(&validIndex, 1);
    threshold *= 0.1F;
    i12 = validIndex->size[0];
    validIndex->size[0] = rawPts_metric->size[0];
    emxEnsureCapacity((emxArray__common *)validIndex, i12, (int)sizeof(boolean_T));
    out_numel = rawPts_metric->size[0];
    for (i12 = 0; i12 < out_numel; i12++) {
      validIndex->data[i12] = (rawPts_metric->data[i12] >= threshold);
    }

    emxInit_int32_T(&r4, 1);
    eml_li_find(validIndex, r4);
    i12 = locations->size[0] * locations->size[1];
    locations->size[0] = r4->size[0];
    locations->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)locations, i12, (int)sizeof(float));
    for (i12 = 0; i12 < 2; i12++) {
      out_numel = r4->size[0];
      for (ix = 0; ix < out_numel; ix++) {
        locations->data[ix + locations->size[0] * i12] = rawPts_loc->data
          [(r4->data[ix] + rawPts_loc->size[0] * i12) - 1];
      }
    }

    eml_li_find(validIndex, r4);
    i12 = metricValues->size[0];
    metricValues->size[0] = r4->size[0];
    emxEnsureCapacity((emxArray__common *)metricValues, i12, (int)sizeof(float));
    out_numel = r4->size[0];
    emxFree_boolean_T(&validIndex);
    for (i12 = 0; i12 < out_numel; i12++) {
      metricValues->data[i12] = rawPts_metric->data[r4->data[i12] - 1];
    }

    emxFree_int32_T(&r4);
  } else {
    i12 = locations->size[0] * locations->size[1];
    locations->size[0] = 0;
    locations->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)locations, i12, (int)sizeof(float));
    i12 = metricValues->size[0];
    metricValues->size[0] = 0;
    emxEnsureCapacity((emxArray__common *)metricValues, i12, (int)sizeof(float));
  }

  emxFree_real32_T(&rawPts_loc);
  emxFree_real32_T(&rawPts_metric);
  cornerPoints_cg_cornerPoints_cg(locations, metricValues, pts_pLocation,
    pts_pMetric);
  emxFree_real32_T(&metricValues);
  emxFree_real32_T(&locations);
}

/*
 * Arguments    : const boolean_T x[32]
 *                int y_data[]
 *                int y_size[1]
 * Return Type  : void
 */
static void b_eml_li_find(const boolean_T x[32], int y_data[], int y_size[1])
{
  int k;
  int i;
  k = 0;
  for (i = 0; i < 32; i++) {
    if (x[i]) {
      k++;
    }
  }

  y_size[0] = k;
  k = 0;
  for (i = 0; i < 32; i++) {
    if (x[i]) {
      y_data[k] = i + 1;
      k++;
    }
  }
}

/*
 * Arguments    : const emxArray_real32_T *x
 *                int dim
 *                emxArray_real32_T *y
 *                emxArray_int32_T *idx
 * Return Type  : void
 */
static void b_eml_sort(const emxArray_real32_T *x, int dim, emxArray_real32_T *y,
  emxArray_int32_T *idx)
{
  int vlen;
  emxArray_real32_T *vwork;
  int ix;
  unsigned int unnamed_idx_0;
  int vstride;
  int k;
  int i1;
  int j;
  emxArray_int32_T *iidx;
  emxArray_int32_T *idx0;
  boolean_T p;
  int i2;
  int b_j;
  int pEnd;
  int b_p;
  int q;
  int qEnd;
  int kEnd;
  if (dim <= 1) {
    vlen = x->size[0];
  } else {
    vlen = 1;
  }

  emxInit_real32_T(&vwork, 1);
  ix = vwork->size[0];
  vwork->size[0] = vlen;
  emxEnsureCapacity((emxArray__common *)vwork, ix, (int)sizeof(float));
  ix = y->size[0];
  y->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)y, ix, (int)sizeof(float));
  unnamed_idx_0 = (unsigned int)x->size[0];
  ix = idx->size[0];
  idx->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)idx, ix, (int)sizeof(int));
  vstride = 1;
  k = 1;
  while (k <= dim - 1) {
    vstride *= x->size[0];
    k = 2;
  }

  i1 = -1;
  j = 1;
  emxInit_int32_T(&iidx, 1);
  emxInit_int32_T(&idx0, 1);
  while (j <= vstride) {
    i1++;
    ix = i1 + 1;
    for (k = 0; k < vlen; k++) {
      vwork->data[k] = x->data[ix - 1];
      ix += vstride;
    }

    unnamed_idx_0 = (unsigned int)vwork->size[0];
    ix = iidx->size[0];
    iidx->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)iidx, ix, (int)sizeof(int));
    if (vwork->size[0] == 0) {
    } else {
      for (k = 1; k <= vwork->size[0]; k++) {
        iidx->data[k - 1] = k;
      }

      for (k = 1; k <= vwork->size[0] - 1; k += 2) {
        if ((vwork->data[k - 1] >= vwork->data[k]) || rtIsNaNF(vwork->data[k - 1]))
        {
          p = true;
        } else {
          p = false;
        }

        if (p) {
        } else {
          iidx->data[k - 1] = k + 1;
          iidx->data[k] = k;
        }
      }

      ix = idx0->size[0];
      idx0->size[0] = vwork->size[0];
      emxEnsureCapacity((emxArray__common *)idx0, ix, (int)sizeof(int));
      i2 = vwork->size[0];
      for (ix = 0; ix < i2; ix++) {
        idx0->data[ix] = 1;
      }

      ix = 2;
      while (ix < vwork->size[0]) {
        i2 = ix << 1;
        b_j = 1;
        for (pEnd = 1 + ix; pEnd < vwork->size[0] + 1; pEnd = qEnd + ix) {
          b_p = b_j - 1;
          q = pEnd;
          qEnd = b_j + i2;
          if (qEnd > vwork->size[0] + 1) {
            qEnd = vwork->size[0] + 1;
          }

          k = 0;
          kEnd = qEnd - b_j;
          while (k + 1 <= kEnd) {
            if ((vwork->data[iidx->data[b_p] - 1] >= vwork->data[iidx->data[q -
                 1] - 1]) || rtIsNaNF(vwork->data[iidx->data[b_p] - 1])) {
              p = true;
            } else {
              p = false;
            }

            if (p) {
              idx0->data[k] = iidx->data[b_p];
              b_p++;
              if (b_p + 1 == pEnd) {
                while (q < qEnd) {
                  k++;
                  idx0->data[k] = iidx->data[q - 1];
                  q++;
                }
              }
            } else {
              idx0->data[k] = iidx->data[q - 1];
              q++;
              if (q == qEnd) {
                while (b_p + 1 < pEnd) {
                  k++;
                  idx0->data[k] = iidx->data[b_p];
                  b_p++;
                }
              }
            }

            k++;
          }

          for (k = 0; k + 1 <= kEnd; k++) {
            iidx->data[(b_j + k) - 1] = idx0->data[k];
          }

          b_j = qEnd;
        }

        ix = i2;
      }
    }

    ix = i1;
    for (k = 0; k < vlen; k++) {
      y->data[ix] = vwork->data[iidx->data[k] - 1];
      idx->data[ix] = iidx->data[k];
      ix += vstride;
    }

    j++;
  }

  emxFree_int32_T(&idx0);
  emxFree_int32_T(&iidx);
  emxFree_real32_T(&vwork);
}

/*
 * Arguments    : emxArray_b_struct_T **pEmxArray
 * Return Type  : void
 */
static void b_emxFree_struct_T(emxArray_b_struct_T **pEmxArray)
{
  int numEl;
  int i;
  if (*pEmxArray != (emxArray_b_struct_T *)NULL) {
    if ((*pEmxArray)->data != (b_struct_T *)NULL) {
      numEl = 1;
      for (i = 0; i < (*pEmxArray)->numDimensions; i++) {
        numEl *= (*pEmxArray)->size[i];
      }

      for (i = 0; i < numEl; i++) {
        emxFreeStruct_struct_T(&(*pEmxArray)->data[i]);
      }

      if ((*pEmxArray)->canFreeData) {
        free((void *)(*pEmxArray)->data);
      }
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_b_struct_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_boolean_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_int8_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void b_emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions)
{
  emxArray_int8_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int8_T *)malloc(sizeof(emxArray_int8_T));
  emxArray = *pEmxArray;
  emxArray->data = (signed char *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_real32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void b_emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions)
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

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_b_struct_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void b_emxInit_struct_T(emxArray_b_struct_T **pEmxArray, int
  numDimensions)
{
  emxArray_b_struct_T *emxArray;
  int i;
  *pEmxArray = (emxArray_b_struct_T *)malloc(sizeof(emxArray_b_struct_T));
  emxArray = *pEmxArray;
  emxArray->data = (b_struct_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : const double a[2]
 *                double varargin_1
 *                emxArray_real_T *b
 * Return Type  : void
 */
static void b_repmat(const double a[2], double varargin_1, emxArray_real_T *b)
{
  int jcol;
  int ibmat;
  int itilerow;
  jcol = b->size[0] * b->size[1];
  b->size[0] = (int)varargin_1;
  b->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b, jcol, (int)sizeof(double));
  if ((int)varargin_1 == 0) {
  } else {
    for (jcol = 0; jcol < 2; jcol++) {
      ibmat = jcol * (int)varargin_1;
      for (itilerow = 1; itilerow <= (int)varargin_1; itilerow++) {
        b->data[(ibmat + itilerow) - 1] = a[jcol];
      }
    }
  }
}

/*
 * Arguments    : emxArray_real32_T *x
 * Return Type  : void
 */
static void b_round(emxArray_real32_T *x)
{
  int i18;
  int k;
  i18 = x->size[0] << 1;
  for (k = 0; k < i18; k++) {
    x->data[k] = roundf(x->data[k]);
  }
}

/*
 * Arguments    : const emxArray_int8_T *x
 * Return Type  : boolean_T
 */
static boolean_T c_any(const emxArray_int8_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b1;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    b1 = (x->data[ix - 1] == 0);
    if (!b1) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_real_T *y
 *                emxArray_int32_T *idx
 * Return Type  : void
 */
static void c_eml_sort(const emxArray_real_T *x, emxArray_real_T *y,
  emxArray_int32_T *idx)
{
  int dim;
  int vlen;
  emxArray_real_T *vwork;
  int i2;
  unsigned int unnamed_idx_0;
  int vstride;
  int k;
  int i1;
  int j;
  emxArray_int32_T *iidx;
  emxArray_int32_T *idx0;
  boolean_T p;
  int b_j;
  int pEnd;
  int b_p;
  int q;
  int qEnd;
  int kEnd;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  if (dim <= 1) {
    vlen = x->size[0];
  } else {
    vlen = 1;
  }

  emxInit_real_T(&vwork, 1);
  i2 = vwork->size[0];
  vwork->size[0] = vlen;
  emxEnsureCapacity((emxArray__common *)vwork, i2, (int)sizeof(double));
  i2 = y->size[0];
  y->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)y, i2, (int)sizeof(double));
  unnamed_idx_0 = (unsigned int)x->size[0];
  i2 = idx->size[0];
  idx->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)idx, i2, (int)sizeof(int));
  vstride = 1;
  k = 1;
  while (k <= dim - 1) {
    vstride *= x->size[0];
    k = 2;
  }

  i1 = -1;
  j = 1;
  emxInit_int32_T(&iidx, 1);
  emxInit_int32_T(&idx0, 1);
  while (j <= vstride) {
    i1++;
    dim = i1 + 1;
    for (k = 0; k < vlen; k++) {
      vwork->data[k] = x->data[dim - 1];
      dim += vstride;
    }

    unnamed_idx_0 = (unsigned int)vwork->size[0];
    i2 = iidx->size[0];
    iidx->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)iidx, i2, (int)sizeof(int));
    if (vwork->size[0] == 0) {
    } else {
      for (k = 1; k <= vwork->size[0]; k++) {
        iidx->data[k - 1] = k;
      }

      for (k = 1; k <= vwork->size[0] - 1; k += 2) {
        if ((vwork->data[k - 1] <= vwork->data[k]) || rtIsNaN(vwork->data[k])) {
          p = true;
        } else {
          p = false;
        }

        if (p) {
        } else {
          iidx->data[k - 1] = k + 1;
          iidx->data[k] = k;
        }
      }

      i2 = idx0->size[0];
      idx0->size[0] = vwork->size[0];
      emxEnsureCapacity((emxArray__common *)idx0, i2, (int)sizeof(int));
      dim = vwork->size[0];
      for (i2 = 0; i2 < dim; i2++) {
        idx0->data[i2] = 1;
      }

      dim = 2;
      while (dim < vwork->size[0]) {
        i2 = dim << 1;
        b_j = 1;
        for (pEnd = 1 + dim; pEnd < vwork->size[0] + 1; pEnd = qEnd + dim) {
          b_p = b_j;
          q = pEnd - 1;
          qEnd = b_j + i2;
          if (qEnd > vwork->size[0] + 1) {
            qEnd = vwork->size[0] + 1;
          }

          k = 0;
          kEnd = qEnd - b_j;
          while (k + 1 <= kEnd) {
            if ((vwork->data[iidx->data[b_p - 1] - 1] <= vwork->data[iidx->
                 data[q] - 1]) || rtIsNaN(vwork->data[iidx->data[q] - 1])) {
              p = true;
            } else {
              p = false;
            }

            if (p) {
              idx0->data[k] = iidx->data[b_p - 1];
              b_p++;
              if (b_p == pEnd) {
                while (q + 1 < qEnd) {
                  k++;
                  idx0->data[k] = iidx->data[q];
                  q++;
                }
              }
            } else {
              idx0->data[k] = iidx->data[q];
              q++;
              if (q + 1 == qEnd) {
                while (b_p < pEnd) {
                  k++;
                  idx0->data[k] = iidx->data[b_p - 1];
                  b_p++;
                }
              }
            }

            k++;
          }

          for (k = 0; k + 1 <= kEnd; k++) {
            iidx->data[(b_j + k) - 1] = idx0->data[k];
          }

          b_j = qEnd;
        }

        dim = i2;
      }
    }

    dim = i1;
    for (k = 0; k < vlen; k++) {
      y->data[dim] = vwork->data[iidx->data[k] - 1];
      idx->data[dim] = iidx->data[k];
      dim += vstride;
    }

    j++;
  }

  emxFree_int32_T(&idx0);
  emxFree_int32_T(&iidx);
  emxFree_real_T(&vwork);
}

/*
 * Arguments    : const emxArray_real32_T *varargin_1
 *                const emxArray_real32_T *varargin_3
 *                emxArray_real32_T *this_pLocation
 *                emxArray_real32_T *this_pMetric
 * Return Type  : void
 */
static void cornerPoints_cg_cornerPoints_cg(const emxArray_real32_T *varargin_1,
  const emxArray_real32_T *varargin_3, emxArray_real32_T *this_pLocation,
  emxArray_real32_T *this_pMetric)
{
  int i5;
  int loop_ub;
  emxArray_real32_T *r2;
  int varargin_1_idx_0;
  int32_T exitg1;
  i5 = this_pLocation->size[0] * this_pLocation->size[1];
  this_pLocation->size[0] = varargin_1->size[0];
  this_pLocation->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)this_pLocation, i5, (int)sizeof(float));
  loop_ub = varargin_1->size[0] * varargin_1->size[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    this_pLocation->data[i5] = varargin_1->data[i5];
  }

  emxInit_real32_T(&r2, 1);
  if (varargin_3->size[0] == 1) {
    varargin_1_idx_0 = varargin_1->size[0];
    i5 = r2->size[0];
    r2->size[0] = varargin_1_idx_0;
    emxEnsureCapacity((emxArray__common *)r2, i5, (int)sizeof(float));
    varargin_1_idx_0 = varargin_1->size[0];
    if (varargin_1_idx_0 == 0) {
    } else {
      loop_ub = 1;
      do {
        exitg1 = 0;
        varargin_1_idx_0 = varargin_1->size[0];
        if (loop_ub <= varargin_1_idx_0) {
          r2->data[loop_ub - 1] = varargin_3->data[0];
          loop_ub++;
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
  } else {
    i5 = r2->size[0];
    r2->size[0] = varargin_3->size[0];
    emxEnsureCapacity((emxArray__common *)r2, i5, (int)sizeof(float));
    loop_ub = varargin_3->size[0];
    for (i5 = 0; i5 < loop_ub; i5++) {
      r2->data[i5] = varargin_3->data[i5];
    }
  }

  i5 = this_pMetric->size[0];
  this_pMetric->size[0] = r2->size[0];
  emxEnsureCapacity((emxArray__common *)this_pMetric, i5, (int)sizeof(float));
  loop_ub = r2->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    this_pMetric->data[i5] = r2->data[i5];
  }

  emxFree_real32_T(&r2);
}

/*
 * Arguments    : const emxArray_uint8_T *I
 *                emxArray_real32_T *pts_pLocation
 *                emxArray_real32_T *pts_pMetric
 * Return Type  : void
 */
static void detectFASTFeatures(const emxArray_uint8_T *I, emxArray_real32_T
  *pts_pLocation, emxArray_real32_T *pts_pMetric)
{
  vision_ImageDataTypeConverter_0 *obj;
  emxArray_uint8_T *Iu8;
  int i4;
  int out_numel;
  emxArray_real32_T *rawPts_metric;
  emxArray_real32_T *rawPts_loc;
  void * ptrKeypoints;
  emxArray_real32_T *locations;
  emxArray_real32_T *metricValues;
  float threshold;
  int ix;
  boolean_T exitg1;
  emxArray_boolean_T *validIndex;
  emxArray_int32_T *r1;
  if (!h2_not_empty) {
    obj = &h2;

    /* System object Constructor function: vision.ImageDataTypeConverter */
    obj->S0_isInitialized = false;
    obj->S1_isReleased = false;
    h2_not_empty = true;
  }

  emxInit_uint8_T(&Iu8, 2);

  /* System object Outputs function: vision.ImageDataTypeConverter */
  i4 = Iu8->size[0] * Iu8->size[1];
  Iu8->size[0] = I->size[0];
  Iu8->size[1] = I->size[1];
  emxEnsureCapacity((emxArray__common *)Iu8, i4, (int)sizeof(unsigned char));
  out_numel = I->size[0] * I->size[1];
  for (i4 = 0; i4 < out_numel; i4++) {
    Iu8->data[i4] = I->data[i4];
  }

  emxInit_real32_T(&rawPts_metric, 1);
  b_emxInit_real32_T(&rawPts_loc, 2);
  ptrKeypoints = NULL;
  out_numel = detectFAST_compute(&Iu8->data[0], I->size[0], I->size[1], false,
    51, &ptrKeypoints);
  i4 = rawPts_loc->size[0] * rawPts_loc->size[1];
  rawPts_loc->size[0] = out_numel;
  rawPts_loc->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)rawPts_loc, i4, (int)sizeof(float));
  i4 = rawPts_metric->size[0];
  rawPts_metric->size[0] = out_numel;
  emxEnsureCapacity((emxArray__common *)rawPts_metric, i4, (int)sizeof(float));
  detectFAST_assignOutput(ptrKeypoints, &rawPts_loc->data[0],
    &rawPts_metric->data[0]);
  emxFree_uint8_T(&Iu8);
  b_emxInit_real32_T(&locations, 2);
  emxInit_real32_T(&metricValues, 1);
  if (!(rawPts_metric->size[0] == 0)) {
    out_numel = 1;
    threshold = rawPts_metric->data[0];
    if (rawPts_metric->size[0] > 1) {
      if (rtIsNaNF(rawPts_metric->data[0])) {
        ix = 2;
        exitg1 = false;
        while ((!exitg1) && (ix <= rawPts_metric->size[0])) {
          out_numel = ix;
          if (!rtIsNaNF(rawPts_metric->data[ix - 1])) {
            threshold = rawPts_metric->data[ix - 1];
            exitg1 = true;
          } else {
            ix++;
          }
        }
      }

      if (out_numel < rawPts_metric->size[0]) {
        while (out_numel + 1 <= rawPts_metric->size[0]) {
          if (rawPts_metric->data[out_numel] > threshold) {
            threshold = rawPts_metric->data[out_numel];
          }

          out_numel++;
        }
      }
    }

    b_emxInit_boolean_T(&validIndex, 1);
    threshold *= 0.5F;
    i4 = validIndex->size[0];
    validIndex->size[0] = rawPts_metric->size[0];
    emxEnsureCapacity((emxArray__common *)validIndex, i4, (int)sizeof(boolean_T));
    out_numel = rawPts_metric->size[0];
    for (i4 = 0; i4 < out_numel; i4++) {
      validIndex->data[i4] = (rawPts_metric->data[i4] >= threshold);
    }

    emxInit_int32_T(&r1, 1);
    eml_li_find(validIndex, r1);
    i4 = locations->size[0] * locations->size[1];
    locations->size[0] = r1->size[0];
    locations->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)locations, i4, (int)sizeof(float));
    for (i4 = 0; i4 < 2; i4++) {
      out_numel = r1->size[0];
      for (ix = 0; ix < out_numel; ix++) {
        locations->data[ix + locations->size[0] * i4] = rawPts_loc->data
          [(r1->data[ix] + rawPts_loc->size[0] * i4) - 1];
      }
    }

    eml_li_find(validIndex, r1);
    i4 = metricValues->size[0];
    metricValues->size[0] = r1->size[0];
    emxEnsureCapacity((emxArray__common *)metricValues, i4, (int)sizeof(float));
    out_numel = r1->size[0];
    emxFree_boolean_T(&validIndex);
    for (i4 = 0; i4 < out_numel; i4++) {
      metricValues->data[i4] = rawPts_metric->data[r1->data[i4] - 1];
    }

    emxFree_int32_T(&r1);
  } else {
    i4 = locations->size[0] * locations->size[1];
    locations->size[0] = 0;
    locations->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)locations, i4, (int)sizeof(float));
    i4 = metricValues->size[0];
    metricValues->size[0] = 0;
    emxEnsureCapacity((emxArray__common *)metricValues, i4, (int)sizeof(float));
  }

  emxFree_real32_T(&rawPts_loc);
  emxFree_real32_T(&rawPts_metric);
  cornerPoints_cg_cornerPoints_cg(locations, metricValues, pts_pLocation,
    pts_pMetric);
  emxFree_real32_T(&metricValues);
  emxFree_real32_T(&locations);
}

/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : int
 */
static int div_s32(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
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
    absNumerator /= absDenominator;
    if (quotientNeedsNegation) {
      quotient = -(int)absNumerator;
    } else {
      quotient = (int)absNumerator;
    }
  }

  return quotient;
}

/*
 * Arguments    : const emxArray_boolean_T *x
 *                emxArray_int32_T *y
 * Return Type  : void
 */
static void eml_li_find(const emxArray_boolean_T *x, emxArray_int32_T *y)
{
  int k;
  int i;
  int j;
  k = 0;
  for (i = 1; i <= x->size[0]; i++) {
    if (x->data[i - 1]) {
      k++;
    }
  }

  j = y->size[0];
  y->size[0] = k;
  emxEnsureCapacity((emxArray__common *)y, j, (int)sizeof(int));
  j = 0;
  for (i = 1; i <= x->size[0]; i++) {
    if (x->data[i - 1]) {
      y->data[j] = i;
      j++;
    }
  }
}

/*
 * Arguments    : const emxArray_real32_T *x
 *                emxArray_real32_T *y
 *                emxArray_int32_T *idx
 * Return Type  : void
 */
static void eml_sort(const emxArray_real32_T *x, emxArray_real32_T *y,
                     emxArray_int32_T *idx)
{
  int dim;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  b_eml_sort(x, dim, y, idx);
}

/*
 * Arguments    : double dst[2]
 *                const double src[2]
 * Return Type  : void
 */
static void emxCopyMatrix_real_T(double dst[2], const double src[2])
{
  int i;
  for (i = 0; i < 2; i++) {
    dst[i] = src[i];
  }
}

/*
 * Arguments    : b_struct_T *dst
 *                const b_struct_T *src
 * Return Type  : void
 */
static void emxCopyStruct_struct_T(b_struct_T *dst, const b_struct_T *src)
{
  emxCopyMatrix_real_T(dst->offset, src->offset);
  dst->count = src->count;
  emxCopy_int8_T(&dst->grid, &src->grid);
}

/*
 * Arguments    : emxArray_int8_T **dst
 *                emxArray_int8_T * const *src
 * Return Type  : void
 */
static void emxCopy_int8_T(emxArray_int8_T **dst, emxArray_int8_T * const *src)
{
  int numElDst;
  int numElSrc;
  int i;
  numElDst = 1;
  numElSrc = 1;
  for (i = 0; i < (*dst)->numDimensions; i++) {
    numElDst *= (*dst)->size[i];
    numElSrc *= (*src)->size[i];
  }

  for (i = 0; i < (*dst)->numDimensions; i++) {
    (*dst)->size[i] = (*src)->size[i];
  }

  emxEnsureCapacity((emxArray__common *)*dst, numElDst, (int)sizeof(signed char));
  for (i = 0; i < numElSrc; i++) {
    (*dst)->data[i] = (*src)->data[i];
  }
}

/*
 * Arguments    : emxArray__common *emxArray
 *                int oldNumel
 *                int elementSize
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_b_struct_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
static void emxEnsureCapacity_struct_T(emxArray_b_struct_T *emxArray, int
  oldNumel)
{
  int elementSize;
  int newNumel;
  int i;
  void *newData;
  elementSize = (int)sizeof(b_struct_T);
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
      memcpy(newData, (void *)emxArray->data, (unsigned int)(elementSize *
              oldNumel));
      if (emxArray->canFreeData) {
        free((void *)emxArray->data);
      }
    }

    emxArray->data = (b_struct_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }

  if (oldNumel > newNumel) {
    emxTrim_struct_T(emxArray, newNumel, oldNumel);
  } else {
    if (oldNumel < newNumel) {
      emxExpand_struct_T(emxArray, oldNumel, newNumel);
    }
  }
}

/*
 * Arguments    : emxArray_b_struct_T *emxArray
 *                int fromIndex
 *                int toIndex
 * Return Type  : void
 */
static void emxExpand_struct_T(emxArray_b_struct_T *emxArray, int fromIndex, int
  toIndex)
{
  int i;
  for (i = fromIndex; i < toIndex; i++) {
    emxInitStruct_struct_T(&emxArray->data[i]);
  }
}

/*
 * Arguments    : b_struct_T *pStruct
 * Return Type  : void
 */
static void emxFreeStruct_struct_T(b_struct_T *pStruct)
{
  emxFree_int8_T(&pStruct->grid);
}

/*
 * Arguments    : emxArray_boolean_T **pEmxArray
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_int32_T **pEmxArray
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_int8_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_int8_T(emxArray_int8_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int8_T *)NULL) {
    if (((*pEmxArray)->data != (signed char *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_int8_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_real32_T **pEmxArray
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_struct_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_struct_T(emxArray_struct_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_struct_T *)NULL) {
    if (((*pEmxArray)->data != (struct_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_struct_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_uint8_T **pEmxArray
 * Return Type  : void
 */
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

/*
 * Arguments    : b_struct_T *pStruct
 * Return Type  : void
 */
static void emxInitStruct_struct_T(b_struct_T *pStruct)
{
  emxInit_int8_T(&pStruct->grid, 2);
}

/*
 * Arguments    : emxArray_boolean_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_int32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_int8_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions)
{
  emxArray_int8_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int8_T *)malloc(sizeof(emxArray_int8_T));
  emxArray = *pEmxArray;
  emxArray->data = (signed char *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_real32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_struct_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void emxInit_struct_T(emxArray_struct_T **pEmxArray, int numDimensions)
{
  emxArray_struct_T *emxArray;
  int i;
  *pEmxArray = (emxArray_struct_T *)malloc(sizeof(emxArray_struct_T));
  emxArray = *pEmxArray;
  emxArray->data = (struct_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_uint8_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
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

/*
 * Arguments    : emxArray_b_struct_T *emxArray
 *                int fromIndex
 *                int toIndex
 * Return Type  : void
 */
static void emxTrim_struct_T(emxArray_b_struct_T *emxArray, int fromIndex, int
  toIndex)
{
  int i;
  for (i = fromIndex; i < toIndex; i++) {
    emxFreeStruct_struct_T(&emxArray->data[i]);
  }
}

/*
 * Arguments    : emxArray_real_T *a
 *                const double block[2]
 *                emxArray_real_T *b
 * Return Type  : void
 */
static void im2col(emxArray_real_T *a, const double block[2], emxArray_real_T *b)
{
  unsigned int b_a[2];
  double b_block[2];
  boolean_T c_a[2];
  int i7;
  double nc;
  double nn;
  int n;
  double anew;
  double apnd;
  double ndbl;
  double cdiff;
  double y_data[453];
  int nm1d2;
  int k;
  int loop_ub;
  double cidx_data[453];
  int b_n;
  emxArray_real_T *ridx;
  emxArray_real_T *t;
  emxArray_real_T *tt;
  int tmp_data[453];
  emxArray_real_T *ttt;
  emxArray_int32_T *r3;
  emxArray_real_T *d_a;

  /*  matlab im2col function put here for codegen */
  b_a[0] = (unsigned int)a->size[0];
  b_a[1] = (unsigned int)a->size[1];
  b_block[0] = block[0];
  b_block[1] = block[1];
  for (i7 = 0; i7 < 2; i7++) {
    c_a[i7] = (b_a[i7] < b_block[i7]);
  }

  if (any(c_a)) {
    /*  if neighborhood is larger than image */
    i7 = b->size[0] * b->size[1];
    b->size[0] = (int)(block[0] * block[1]);
    b->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)b, i7, (int)sizeof(double));
  } else {
    /*  Create Hankel-like indexing sub matrix. */
    nc = ((double)a->size[0] - block[0]) + 1.0;
    nn = ((double)a->size[1] - block[1]) + 1.0;
    if (rtIsNaN(block[0] - 1.0)) {
      n = 0;
      anew = rtNaN;
      apnd = block[0] - 1.0;
    } else if (block[0] - 1.0 < 0.0) {
      n = -1;
      anew = 0.0;
      apnd = block[0] - 1.0;
    } else if (rtIsInf(block[0] - 1.0)) {
      n = 0;
      anew = rtNaN;
      apnd = block[0] - 1.0;
    } else {
      anew = 0.0;
      ndbl = floor((block[0] - 1.0) + 0.5);
      apnd = ndbl;
      cdiff = ndbl - (block[0] - 1.0);
      if (fabs(cdiff) < 4.4408920985006262E-16 * fabs(block[0] - 1.0)) {
        ndbl++;
        apnd = block[0] - 1.0;
      } else if (cdiff > 0.0) {
        apnd = ndbl - 1.0;
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = (int)ndbl - 1;
      } else {
        n = -1;
      }
    }

    if (n + 1 > 0) {
      y_data[0] = anew;
      if (n + 1 > 1) {
        y_data[n] = apnd;
        nm1d2 = n / 2;
        for (k = 1; k < nm1d2; k++) {
          y_data[k] = anew + (double)k;
          y_data[n - k] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n) {
          y_data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          y_data[nm1d2] = anew + (double)nm1d2;
          y_data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    loop_ub = n + 1;
    for (i7 = 0; i7 < loop_ub; i7++) {
      cidx_data[i7] = y_data[i7];
    }

    if (rtIsNaN(nc)) {
      b_n = 0;
      anew = rtNaN;
      apnd = nc;
    } else if (nc < 1.0) {
      b_n = -1;
      anew = 1.0;
      apnd = nc;
    } else if (rtIsInf(nc)) {
      b_n = 0;
      anew = rtNaN;
      apnd = nc;
    } else {
      anew = 1.0;
      ndbl = floor((nc - 1.0) + 0.5);
      apnd = 1.0 + ndbl;
      cdiff = (1.0 + ndbl) - nc;
      if (fabs(cdiff) < 4.4408920985006262E-16 * nc) {
        ndbl++;
        apnd = nc;
      } else if (cdiff > 0.0) {
        apnd = 1.0 + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        b_n = (int)ndbl - 1;
      } else {
        b_n = -1;
      }
    }

    b_emxInit_real_T(&ridx, 2);
    i7 = ridx->size[0] * ridx->size[1];
    ridx->size[0] = 1;
    ridx->size[1] = b_n + 1;
    emxEnsureCapacity((emxArray__common *)ridx, i7, (int)sizeof(double));
    if (b_n + 1 > 0) {
      ridx->data[0] = anew;
      if (b_n + 1 > 1) {
        ridx->data[b_n] = apnd;
        nm1d2 = b_n / 2;
        for (k = 1; k < nm1d2; k++) {
          ridx->data[k] = anew + (double)k;
          ridx->data[b_n - k] = apnd - (double)k;
        }

        if (nm1d2 << 1 == b_n) {
          ridx->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          ridx->data[nm1d2] = anew + (double)nm1d2;
          ridx->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    b_emxInit_real_T(&t, 2);
    loop_ub = n + 1;
    i7 = t->size[0] * t->size[1];
    t->size[0] = n + 1;
    t->size[1] = (int)nc;
    emxEnsureCapacity((emxArray__common *)t, i7, (int)sizeof(double));
    k = (int)nc;
    for (i7 = 0; i7 < k; i7++) {
      for (b_n = 0; b_n < loop_ub; b_n++) {
        t->data[b_n + t->size[0] * i7] = cidx_data[b_n] + ridx->data[ridx->size
          [0] * i7];
      }
    }

    b_emxInit_real_T(&tt, 2);

    /*  Hankel Subscripts */
    i7 = tt->size[0] * tt->size[1];
    tt->size[0] = (int)(block[0] * block[1]);
    emxEnsureCapacity((emxArray__common *)tt, i7, (int)sizeof(double));
    i7 = tt->size[0] * tt->size[1];
    tt->size[1] = (int)nc;
    emxEnsureCapacity((emxArray__common *)tt, i7, (int)sizeof(double));
    loop_ub = (int)(block[0] * block[1]) * (int)nc;
    for (i7 = 0; i7 < loop_ub; i7++) {
      tt->data[i7] = 0.0;
    }

    if (rtIsNaN(block[0])) {
      n = 0;
      anew = rtNaN;
      apnd = block[0];
    } else if (block[0] < 1.0) {
      n = -1;
      anew = 1.0;
      apnd = block[0];
    } else if (rtIsInf(block[0])) {
      n = 0;
      anew = rtNaN;
      apnd = block[0];
    } else {
      anew = 1.0;
      ndbl = floor((block[0] - 1.0) + 0.5);
      apnd = 1.0 + ndbl;
      cdiff = (1.0 + ndbl) - block[0];
      if (fabs(cdiff) < 4.4408920985006262E-16 * block[0]) {
        ndbl++;
        apnd = block[0];
      } else if (cdiff > 0.0) {
        apnd = 1.0 + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = (int)ndbl - 1;
      } else {
        n = -1;
      }
    }

    i7 = ridx->size[0] * ridx->size[1];
    ridx->size[0] = 1;
    ridx->size[1] = n + 1;
    emxEnsureCapacity((emxArray__common *)ridx, i7, (int)sizeof(double));
    if (n + 1 > 0) {
      ridx->data[0] = anew;
      if (n + 1 > 1) {
        ridx->data[n] = apnd;
        nm1d2 = n / 2;
        for (k = 1; k < nm1d2; k++) {
          ridx->data[k] = anew + (double)k;
          ridx->data[n - k] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n) {
          ridx->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          ridx->data[nm1d2] = anew + (double)nm1d2;
          ridx->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    for (nm1d2 = 0; nm1d2 < (int)((block[1] - 1.0) + 1.0); nm1d2++) {
      anew = (double)nm1d2 * block[0];
      loop_ub = ridx->size[1];
      for (i7 = 0; i7 < loop_ub; i7++) {
        tmp_data[i7] = (int)(anew + ridx->data[ridx->size[0] * i7]) - 1;
      }

      anew = (double)a->size[0] * (double)nm1d2;
      loop_ub = t->size[1];
      for (i7 = 0; i7 < loop_ub; i7++) {
        k = t->size[0];
        for (b_n = 0; b_n < k; b_n++) {
          tt->data[tmp_data[b_n] + tt->size[0] * i7] = t->data[b_n + t->size[0] *
            i7] + anew;
        }
      }
    }

    emxFree_real_T(&t);
    b_emxInit_real_T(&ttt, 2);
    i7 = ttt->size[0] * ttt->size[1];
    ttt->size[0] = (int)(block[0] * block[1]);
    emxEnsureCapacity((emxArray__common *)ttt, i7, (int)sizeof(double));
    i7 = ttt->size[0] * ttt->size[1];
    ttt->size[1] = (int)(nc * nn);
    emxEnsureCapacity((emxArray__common *)ttt, i7, (int)sizeof(double));
    loop_ub = (int)(block[0] * block[1]) * (int)(nc * nn);
    for (i7 = 0; i7 < loop_ub; i7++) {
      ttt->data[i7] = 0.0;
    }

    if (nc < 1.0) {
      n = -1;
      anew = 1.0;
      apnd = 0.0;
    } else if (rtIsInf(nc)) {
      n = 0;
      anew = rtNaN;
      apnd = nc;
    } else {
      anew = 1.0;
      ndbl = floor((nc - 1.0) + 0.5);
      apnd = 1.0 + ndbl;
      cdiff = (1.0 + ndbl) - nc;
      if (fabs(cdiff) < 4.4408920985006262E-16 * nc) {
        ndbl++;
        apnd = nc;
      } else if (cdiff > 0.0) {
        apnd = 1.0 + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      n = (int)ndbl - 1;
    }

    i7 = ridx->size[0] * ridx->size[1];
    ridx->size[0] = 1;
    ridx->size[1] = n + 1;
    emxEnsureCapacity((emxArray__common *)ridx, i7, (int)sizeof(double));
    if (n + 1 > 0) {
      ridx->data[0] = anew;
      if (n + 1 > 1) {
        ridx->data[n] = apnd;
        nm1d2 = n / 2;
        for (k = 1; k < nm1d2; k++) {
          ridx->data[k] = anew + (double)k;
          ridx->data[n - k] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n) {
          ridx->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          ridx->data[nm1d2] = anew + (double)nm1d2;
          ridx->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    nm1d2 = 0;
    emxInit_int32_T(&r3, 1);
    while (nm1d2 <= (int)((nn - 1.0) + 1.0) - 1) {
      anew = (double)nm1d2 * nc;
      i7 = r3->size[0];
      r3->size[0] = ridx->size[1];
      emxEnsureCapacity((emxArray__common *)r3, i7, (int)sizeof(int));
      loop_ub = ridx->size[1];
      for (i7 = 0; i7 < loop_ub; i7++) {
        r3->data[i7] = (int)(anew + ridx->data[ridx->size[0] * i7]) - 1;
      }

      anew = (double)a->size[0] * (double)nm1d2;
      loop_ub = tt->size[1];
      for (i7 = 0; i7 < loop_ub; i7++) {
        k = tt->size[0];
        for (b_n = 0; b_n < k; b_n++) {
          ttt->data[b_n + ttt->size[0] * r3->data[i7]] = tt->data[b_n + tt->
            size[0] * i7] + anew;
        }
      }

      nm1d2++;
    }

    emxFree_int32_T(&r3);
    emxFree_real_T(&tt);
    emxFree_real_T(&ridx);

    /*  If a is a row vector, change it to a column vector. This change is */
    /*  necessary when A is a row vector and [M N] = size(A). */
    emxInit_real_T(&d_a, 1);
    if ((a->size[1] > 1) && (a->size[0] == 1)) {
      nm1d2 = a->size[1];
      k = a->size[1];
      i7 = d_a->size[0];
      d_a->size[0] = nm1d2;
      emxEnsureCapacity((emxArray__common *)d_a, i7, (int)sizeof(double));
      for (i7 = 0; i7 < nm1d2; i7++) {
        d_a->data[i7] = a->data[i7];
      }

      i7 = a->size[0] * a->size[1];
      a->size[0] = k;
      a->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)a, i7, (int)sizeof(double));
      i7 = 0;
      while (i7 <= 0) {
        for (i7 = 0; i7 < k; i7++) {
          a->data[i7] = d_a->data[i7];
        }

        i7 = 1;
      }
    }

    emxFree_real_T(&d_a);
    i7 = b->size[0] * b->size[1];
    b->size[0] = ttt->size[0];
    b->size[1] = ttt->size[1];
    emxEnsureCapacity((emxArray__common *)b, i7, (int)sizeof(double));
    loop_ub = ttt->size[0] * ttt->size[1];
    for (i7 = 0; i7 < loop_ub; i7++) {
      b->data[i7] = a->data[(int)ttt->data[i7] - 1];
    }

    emxFree_real_T(&ttt);
  }
}

/*
 * Arguments    : const emxArray_int8_T *A
 *                emxArray_int8_T *B
 * Return Type  : void
 */
static void imerode(const emxArray_int8_T *A, emxArray_int8_T *B)
{
  int i10;
  int i11;
  double asize[2];
  boolean_T nhood[9];
  double nsize[2];
  for (i10 = 0; i10 < 2; i10++) {
    i11 = B->size[0] * B->size[1];
    B->size[i10] = A->size[i10];
    emxEnsureCapacity((emxArray__common *)B, i11, (int)sizeof(signed char));
  }

  for (i10 = 0; i10 < 2; i10++) {
    asize[i10] = A->size[i10];
  }

  for (i10 = 0; i10 < 9; i10++) {
    nhood[i10] = true;
  }

  for (i10 = 0; i10 < 2; i10++) {
    nsize[i10] = 3.0;
  }

  erode_flat_int8_tbb(&A->data[0], asize, 2.0, nhood, nsize, 2.0, &B->data[0]);
}

/*
 * INITFEATURES initialize features for all anchors at the beginning
 *  features numFeatures x 2 matrix
 *  threshold suggested threshold for FASt detector (dynamic adjusted
 *  initially set to default 30)
 *  flag shows the status of initialization:
 *  {0,1,-1} = {successful but not balanced,successful,unsuccessful not
 *  enough features found}
 * Arguments    : const unsigned char greyImg[332220]
 *                double borderLength
 *                double numBinsX
 *                double numBinsY
 *                double minDistance
 *                double features[64]
 *                double featStatus[32]
 *                double *threshold
 *                double *flag
 * Return Type  : void
 */
static void initFeatures(const unsigned char greyImg[332220], double
  borderLength, double numBinsX, double numBinsY, double minDistance, double
  features[64], double featStatus[32], double *threshold, double *flag)
{
  emxArray_struct_T *structBin;
  double binWidth;
  double binHeight;
  double numBins;
  double numAvgFeaturePerBin;
  double tooMany;
  int i0;
  int loop_ub;
  static const struct_T r0 = { { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  emxArray_real_T *featureCounts;
  unsigned int cntBin;
  int i;
  emxArray_real_T *arrBinFeatures;
  emxArray_real_T *unusedU1;
  emxArray_real32_T *unusedU0;
  emxArray_real32_T *arrBinCandidates_pLocation;
  emxArray_real32_T *arrBinCandidates_pMetric;
  emxArray_int32_T *iidx;
  emxArray_uint8_T *b_greyImg;
  int j;
  double numGridsY;
  double numLeftOverFeature;
  double numFeatureInit;
  int cntChecked;
  int i1;
  int i2;
  int i3;
  int b_loop_ub;
  double numGridsX;
  emxArray_boolean_T *binGrids;
  double numFeatureInit_old;
  double arrBinCandidates_data[64];
  double grid[2];
  int32_T exitg1;
  double left[2];
  boolean_T guard1 = false;
  *flag = 1.0;
  memset(&features[0], 0, sizeof(double) << 6);
  memset(&featStatus[0], 0, sizeof(double) << 5);
  emxInit_struct_T(&structBin, 1);

  /* % divide bins and find fast feature in each bin */
  /*  set initial output */
  binWidth = rt_roundd_snf((735.0 - 2.0 * borderLength) / numBinsX);
  binHeight = rt_roundd_snf((452.0 - 2.0 * borderLength) / numBinsY);
  numBins = numBinsX * numBinsY;
  numAvgFeaturePerBin = 32.0 / numBins;

  /*  parameter for detector */
  *threshold = 0.2;

  /*  default value */
  /*  minimum threshold allowed (used in threshold adaption) */
  /*  flag to adjust threshold. it is incremented in each bin, if there are too */
  /*  many features found in this bin */
  tooMany = 0.0;

  /*  initialize structBin */
  i0 = structBin->size[0];
  structBin->size[0] = (int)numBins;
  emxEnsureCapacity((emxArray__common *)structBin, i0, (int)sizeof(struct_T));
  loop_ub = (int)numBins;
  for (i0 = 0; i0 < loop_ub; i0++) {
    structBin->data[i0] = r0;
  }

  emxInit_real_T(&featureCounts, 1);
  cntBin = 1U;
  i0 = featureCounts->size[0];
  featureCounts->size[0] = (int)numBins;
  emxEnsureCapacity((emxArray__common *)featureCounts, i0, (int)sizeof(double));
  loop_ub = (int)numBins;
  for (i0 = 0; i0 < loop_ub; i0++) {
    featureCounts->data[i0] = 0.0;
  }

  i = 0;
  b_emxInit_real_T(&arrBinFeatures, 2);
  emxInit_real_T(&unusedU1, 1);
  emxInit_real32_T(&unusedU0, 1);
  b_emxInit_real32_T(&arrBinCandidates_pLocation, 2);
  emxInit_real32_T(&arrBinCandidates_pMetric, 1);
  emxInit_int32_T(&iidx, 1);
  emxInit_uint8_T(&b_greyImg, 2);
  while (i <= (int)numBinsX - 1) {
    for (j = 0; j < (int)numBinsY; j++) {
      numGridsY = borderLength + binWidth * ((1.0 + (double)i) - 1.0);
      numLeftOverFeature = borderLength + binHeight * ((1.0 + (double)j) - 1.0);
      structBin->data[(int)cntBin - 1].offset[0] = numGridsY;
      structBin->data[(int)cntBin - 1].offset[1] = numLeftOverFeature;
      numFeatureInit = (numLeftOverFeature + 1.0) + binHeight;
      if (numLeftOverFeature + 1.0 > numFeatureInit) {
        i0 = 0;
        cntChecked = 0;
      } else {
        i0 = (int)(numLeftOverFeature + 1.0) - 1;
        cntChecked = (int)numFeatureInit;
      }

      numFeatureInit = (numGridsY + 1.0) + binWidth;
      if (numGridsY + 1.0 > numFeatureInit) {
        i1 = 0;
        i2 = 0;
      } else {
        i1 = (int)(numGridsY + 1.0) - 1;
        i2 = (int)numFeatureInit;
      }

      /*  feature detection */
      /*  arrBinCandidates = detectHarrisFeatures(subImg,'MinQuality',0.5,'FilterSize',5); */
      i3 = b_greyImg->size[0] * b_greyImg->size[1];
      b_greyImg->size[0] = cntChecked - i0;
      b_greyImg->size[1] = i2 - i1;
      emxEnsureCapacity((emxArray__common *)b_greyImg, i3, (int)sizeof(unsigned
        char));
      loop_ub = i2 - i1;
      for (i2 = 0; i2 < loop_ub; i2++) {
        b_loop_ub = cntChecked - i0;
        for (i3 = 0; i3 < b_loop_ub; i3++) {
          b_greyImg->data[i3 + b_greyImg->size[0] * i2] = greyImg[(i0 + i3) +
            452 * (i1 + i2)];
        }
      }

      detectFASTFeatures(b_greyImg, arrBinCandidates_pLocation,
                         arrBinCandidates_pMetric);

      /*  rank the feature wrt the detector response and keep maximal */
      /*  numFeatures features in each bin */
      eml_sort(arrBinCandidates_pMetric, unusedU0, iidx);
      i0 = unusedU1->size[0];
      unusedU1->size[0] = iidx->size[0];
      emxEnsureCapacity((emxArray__common *)unusedU1, i0, (int)sizeof(double));
      loop_ub = iidx->size[0];
      for (i0 = 0; i0 < loop_ub; i0++) {
        unusedU1->data[i0] = iidx->data[i0];
      }

      numFeatureInit = FeaturePointsImpl_get_Count(arrBinCandidates_pLocation);
      numGridsY = 32.0;
      if (numFeatureInit < 32.0) {
        numGridsY = numFeatureInit;
      }

      featureCounts->data[(int)cntBin - 1] = numGridsY;

      /*  save candidates in bin structures */
      if (1.0 > numGridsY) {
        loop_ub = 0;
      } else {
        loop_ub = (int)numGridsY;
      }

      i0 = arrBinFeatures->size[0] * arrBinFeatures->size[1];
      arrBinFeatures->size[0] = loop_ub;
      arrBinFeatures->size[1] = 2;
      emxEnsureCapacity((emxArray__common *)arrBinFeatures, i0, (int)sizeof
                        (double));
      for (i0 = 0; i0 < 2; i0++) {
        for (cntChecked = 0; cntChecked < loop_ub; cntChecked++) {
          arrBinFeatures->data[cntChecked + arrBinFeatures->size[0] * i0] =
            arrBinCandidates_pLocation->data[((int)unusedU1->data[cntChecked] +
            arrBinCandidates_pLocation->size[0] * i0) - 1];
        }
      }

      for (i0 = 0; i0 < 2; i0++) {
        loop_ub = arrBinFeatures->size[0];
        for (cntChecked = 0; cntChecked < loop_ub; cntChecked++) {
          structBin->data[(int)cntBin - 1].featureCandidates[cntChecked + (i0 <<
            5)] = arrBinFeatures->data[cntChecked + arrBinFeatures->size[0] * i0];
        }
      }

      cntBin++;
    }

    i++;
  }

  emxFree_uint8_T(&b_greyImg);
  emxFree_real32_T(&arrBinCandidates_pMetric);
  emxFree_real32_T(&arrBinCandidates_pLocation);
  emxFree_real32_T(&unusedU0);

  /* % fine placement in each bin, a better way to keep minimum distance */
  /*  check distance to existing feature by constructing a fine grid with */
  /*  gridwidth = minDistance */
  /*  proceed from the highest ranking feature, */
  /*  check in candidate's neighboring grids whether a features has already */
  /*  been chosen, if yes discard this candidate */
  /*  a grid is flagged 1 if a feature in its area is found and 0 otherwise */
  numGridsX = ceil(binWidth / minDistance);
  numGridsY = ceil(binHeight / minDistance);

  /*  start from the bin with least number of candidates, because it's most */
  /*  possible to be not filled. if one bin is not filled, we'll be filled in */
  /*  the next bin */
  c_eml_sort(featureCounts, unusedU1, iidx);
  i0 = unusedU1->size[0];
  unusedU1->size[0] = iidx->size[0];
  emxEnsureCapacity((emxArray__common *)unusedU1, i0, (int)sizeof(double));
  loop_ub = iidx->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    unusedU1->data[i0] = iidx->data[i0];
  }

  emxFree_int32_T(&iidx);

  /*  number of features that cannot be filled in the processed bins */
  numLeftOverFeature = 0.0;
  binWidth = 0.0;

  /*  number of initialized features in all bin (in order to make sure total */
  /*  number of features does not exceed required) */
  numFeatureInit = 0.0;
  binHeight = 0.0;
  i = 0;
  emxInit_boolean_T(&binGrids, 3);
  while (i <= unusedU1->size[0] - 1) {
    binWidth++;
    numFeatureInit_old = numFeatureInit;

    /*  get number of found features in this bin. */
    /*  if found nothing, then increase the number of features to be found in */
    /*  other bins */
    if (featureCounts->data[(int)unusedU1->data[i] - 1] == 0.0) {
      numLeftOverFeature += numAvgFeaturePerBin;
      *flag = 0.0;
    } else {
      if (1.0 > featureCounts->data[(int)unusedU1->data[i] - 1]) {
        loop_ub = 0;
      } else {
        loop_ub = (int)featureCounts->data[(int)unusedU1->data[i] - 1];
      }

      for (i0 = 0; i0 < 2; i0++) {
        for (cntChecked = 0; cntChecked < loop_ub; cntChecked++) {
          arrBinCandidates_data[cntChecked + loop_ub * i0] = structBin->data
            [(int)unusedU1->data[i] - 1].featureCandidates[cntChecked + (i0 << 5)];
        }
      }

      /*  initialize arrBinFeatures */
      i0 = arrBinFeatures->size[0] * arrBinFeatures->size[1];
      arrBinFeatures->size[0] = (int)(numAvgFeaturePerBin + binHeight);
      arrBinFeatures->size[1] = 2;
      emxEnsureCapacity((emxArray__common *)arrBinFeatures, i0, (int)sizeof
                        (double));
      b_loop_ub = (int)(numAvgFeaturePerBin + binHeight) << 1;
      for (i0 = 0; i0 < b_loop_ub; i0++) {
        arrBinFeatures->data[i0] = 0.0;
      }

      /*  a finer grids to represent location of keypoints */
      /*  first and last row and column are dummy row and column */
      /*  hence grid = grid + [1,1] */
      /*  allocate memory for matrix, in each bin maximum numFeatures */
      /*  features are found */
      i0 = binGrids->size[0] * binGrids->size[1] * binGrids->size[2];
      binGrids->size[0] = (int)(numGridsX + 2.0);
      binGrids->size[1] = (int)(numGridsY + 2.0);
      binGrids->size[2] = 32;
      emxEnsureCapacity((emxArray__common *)binGrids, i0, (int)sizeof(boolean_T));
      b_loop_ub = (int)(numGridsX + 2.0) * (int)(numGridsY + 2.0) << 5;
      for (i0 = 0; i0 < b_loop_ub; i0++) {
        binGrids->data[i0] = false;
      }

      /*  always keep the highest ranking candidate */
      for (i0 = 0; i0 < 2; i0++) {
        grid[i0] = ceil(arrBinCandidates_data[loop_ub * i0] / minDistance);
      }

      /*  save highest rating feature */
      for (i0 = 0; i0 < 2; i0++) {
        arrBinFeatures->data[arrBinFeatures->size[0] * i0] =
          arrBinCandidates_data[loop_ub * i0] + structBin->data[(int)
          unusedU1->data[i] - 1].offset[i0];
      }

      binGrids->data[((int)grid[0] + (int)(numGridsX + 2.0) * ((int)grid[1] - 1))
        - 1] = true;
      numFeatureInit++;

      /*  start checking candidates */
      cntBin = 1U;
      cntChecked = 0;
      do {
        exitg1 = 0;
        if (cntChecked + 1 < featureCounts->data[(int)unusedU1->data[i] - 1]) {
          cntChecked++;
          for (i0 = 0; i0 < 2; i0++) {
            grid[i0] = ceil(arrBinCandidates_data[cntChecked + loop_ub * i0] /
                            minDistance) + 1.0;
          }

          b_loop_ub = binGrids->size[0];
          if (binGrids->data[((int)grid[0] + b_loop_ub * ((int)grid[1] - 1)) - 1]
              == 1) {
          } else {
            for (i0 = 0; i0 < 2; i0++) {
              left[i0] = grid[i0] + (-1.0 + (double)i0);
            }

            b_loop_ub = binGrids->size[0];
            if (binGrids->data[((int)left[0] + b_loop_ub * ((int)left[1] - 1)) -
                1] == 1) {
            } else {
              for (i0 = 0; i0 < 2; i0++) {
                left[i0] = grid[i0] + (1.0 - (double)i0);
              }

              b_loop_ub = binGrids->size[0];
              if (binGrids->data[((int)left[0] + b_loop_ub * ((int)left[1] - 1))
                  - 1] == 1) {
              } else {
                for (i0 = 0; i0 < 2; i0++) {
                  left[i0] = grid[i0] + (0.0 - (double)i0);
                }

                b_loop_ub = binGrids->size[0];
                if (binGrids->data[((int)left[0] + b_loop_ub * ((int)left[1] - 1))
                    - 1] == 1) {
                } else {
                  for (i0 = 0; i0 < 2; i0++) {
                    left[i0] = grid[i0] + (double)i0;
                  }

                  b_loop_ub = binGrids->size[0];
                  if (binGrids->data[((int)left[0] + b_loop_ub * ((int)left[1] -
                        1)) - 1] == 1) {
                  } else {
                    /*  passed check mark grid as occupied */
                    b_loop_ub = binGrids->size[0];
                    binGrids->data[((int)grid[0] + b_loop_ub * ((int)grid[1] - 1))
                      - 1] = true;
                    cntBin++;
                    numFeatureInit++;
                    for (i0 = 0; i0 < 2; i0++) {
                      arrBinFeatures->data[((int)cntBin + arrBinFeatures->size[0]
                                            * i0) - 1] =
                        arrBinCandidates_data[cntChecked + loop_ub * i0] +
                        structBin->data[(int)unusedU1->data[i] - 1].offset[i0];
                    }

                    if ((cntBin >= numAvgFeaturePerBin + binHeight) ||
                        (numFeatureInit >= 32.0)) {
                      /*  double check */
                      /*  also an early break means we found too many features */
                      tooMany++;
                      exitg1 = 1;
                    }
                  }
                }
              }
            }
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);

      numLeftOverFeature = (numLeftOverFeature + numAvgFeaturePerBin) - (double)
        cntBin;

      /*  stop adding features to this bin if this feature is full and the */
      /*  left over features from earlier bins are also found in this bin */
      binHeight = rt_roundd_snf(numLeftOverFeature / ((numBins - binWidth) + 1.0));

      /*  if there's leftover slots in this bin, flag 0 (unbalanced) */
      if (numLeftOverFeature > 0.0) {
        *flag = 0.0;
      }

      /*  if this bin is not filled */
      /*  store results in a cell for the full image */
      if (numFeatureInit_old + 1.0 > numFeatureInit) {
        i0 = 0;
      } else {
        i0 = (int)(numFeatureInit_old + 1.0) - 1;
      }

      for (cntChecked = 0; cntChecked < 2; cntChecked++) {
        loop_ub = (int)cntBin;
        for (i1 = 0; i1 < loop_ub; i1++) {
          features[(i0 + i1) + (cntChecked << 5)] = arrBinFeatures->data[i1 +
            arrBinFeatures->size[0] * cntChecked];
        }
      }
    }

    i++;
  }

  emxFree_real_T(&unusedU1);
  emxFree_boolean_T(&binGrids);
  emxFree_real_T(&arrBinFeatures);
  emxFree_real_T(&featureCounts);
  emxFree_struct_T(&structBin);

  /* % set flag */
  guard1 = false;
  if (numFeatureInit < 32.0) {
    *flag = -1.0;
    if (numFeatureInit == 0.0) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    /* % dynamically adjust threshold */
    /*  dynamically adjust parameter to keep number of found features */
    /*  around a certain value. This step reduces future computation */
    /*  if features found in one bin are more than number of required */
    /*  features the whole image, increase threshold */
    if (tooMany == numBins) {
      *threshold = 0.21000000000000002;
    } else {
      if (*flag == -1.0) {
        /*  decrease threshold slowlier, because we stil want good valid features */
        *threshold = 0.19;
      }
    }

    /* % return output */
    loop_ub = (int)numFeatureInit;
    for (i0 = 0; i0 < loop_ub; i0++) {
      featStatus[i0] = 2.0;
    }
  }
}

/*
 * Arguments    : const emxArray_int8_T *A
 *                const emxArray_int8_T *B
 *                emxArray_int8_T *K
 * Return Type  : void
 */
static void kron(const emxArray_int8_T *A, const emxArray_int8_T *B,
                 emxArray_int8_T *K)
{
  int kidx;
  int unnamed_idx_1;
  int j2;
  int i1;
  int i2;
  kidx = A->size[0] * B->size[0];
  unnamed_idx_1 = A->size[1] * B->size[1];
  j2 = K->size[0] * K->size[1];
  K->size[0] = kidx;
  K->size[1] = unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)K, j2, (int)sizeof(signed char));
  kidx = -1;
  for (unnamed_idx_1 = 1; unnamed_idx_1 <= A->size[1]; unnamed_idx_1++) {
    for (j2 = 1; j2 <= B->size[1]; j2++) {
      for (i1 = 1; i1 <= A->size[0]; i1++) {
        for (i2 = 1; i2 <= B->size[0]; i2++) {
          kidx++;
          K->data[kidx] = A->data[(i1 + A->size[0] * (unnamed_idx_1 - 1)) - 1];
        }
      }
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void pointextraction_init(void)
{
  initialized_not_empty = false;
  memset(&prevPoints[0], 0, sizeof(double) << 6);
  memset(&prevDisp[0], 0, sizeof(double) << 5);
}

/*
 * Arguments    : const double x[2]
 *                const double y[2]
 *                double z[2]
 * Return Type  : void
 */
static void rdivide(const double x[2], const double y[2], double z[2])
{
  int i9;
  for (i9 = 0; i9 < 2; i9++) {
    z[i9] = x[i9] / y[i9];
  }
}

/*
 * REFILLFEATURES finds new features to fill in feature slots
 *  features numFeatures x 2 matrix
 *  threshold suggested threshold for FASt detector (dynamic adjusted
 *  initially set to default 30)
 *  flag shows whether enough features are found
 * % parameters
 *  detector parameters
 *  TODO dynamically adjusted
 * Arguments    : const unsigned char greyImg[332220]
 *                double features[64]
 *                double featStatus[32]
 *                double numAnchors
 *                double numBinsX
 *                double numBinsY
 *                double borderLength
 *                double minDistance
 * Return Type  : boolean_T
 */
static boolean_T refillFeature(const unsigned char greyImg[332220], double
  features[64], double featStatus[32], double numAnchors, double numBinsX,
  double numBinsY, double borderLength, double minDistance)
{
  boolean_T notEnough;
  double featStatus_old[32];
  int i;
  emxArray_boolean_T *deactivatedAnchor;
  double numBins;
  double numPointsPerAnchor;
  int i14;
  int loop_ub;
  int ca;
  double d0;
  double numGridsX;
  int i15;
  int ii;
  signed char activeAnchorFeature_data[32];
  int idx;
  int i16;
  int ii_data[32];
  boolean_T exitg2;
  boolean_T guard3 = false;
  signed char idxDeadFeatures_data[32];
  boolean_T b_featStatus[32];
  int ii_size[1];
  int activeFeatures_size_idx_0;
  double activeFeatures_data[64];
  double binWidth;
  double binHeight;
  emxArray_int8_T *t0_grid;
  b_struct_T expl_temp;
  emxArray_b_struct_T *structBin;
  emxArray_real_T *featureCounts;
  unsigned int b_i;
  double activeFeatures[2];
  double b_binWidth[2];
  double bin[2];
  double features_tmp[64];
  emxArray_real_T *b_featureCounts;
  emxArray_int32_T *iidx;
  double numCheckedBin;
  double numFeatureFound;
  emxArray_int8_T *erodedGrid;
  emxArray_real32_T *arrBinCandidates_pLocation;
  emxArray_real32_T *arrBinCandidates_pMetric;
  emxArray_real32_T *roundedCandLocation;
  emxArray_int8_T *validCandidates;
  emxArray_real32_T *arrMetric;
  emxArray_real32_T *binFeatures;
  emxArray_real_T *ranking;
  emxArray_real_T *r10;
  emxArray_real32_T *varargin_2;
  emxArray_boolean_T *b_validCandidates;
  emxArray_boolean_T *c_validCandidates;
  emxArray_uint8_T *b_greyImg;
  emxArray_int8_T *b_erodedGrid;
  emxArray_int8_T *r11;
  emxArray_int8_T *c_erodedGrid;
  emxArray_int8_T *d_erodedGrid;
  boolean_T exitg1;
  double numFeatureFound_old;
  int i17;
  int b_loop_ub;
  int c_loop_ub;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  int siz[2];
  double varargin_1[3];
  boolean_T bv0[32];
  double features_tmp_data[64];
  signed char tmp_data[31];

  /*  copy old featStatus */
  for (i = 0; i < 32; i++) {
    featStatus_old[i] = featStatus[i];
  }

  b_emxInit_boolean_T(&deactivatedAnchor, 1);

  /*  erosion neighborhood size */
  numBins = numBinsX * numBinsY;

  /* % deactivate anchor when half of its features are not tracked */
  numPointsPerAnchor = 32.0 / numAnchors;
  i14 = deactivatedAnchor->size[0];
  deactivatedAnchor->size[0] = (int)numAnchors;
  emxEnsureCapacity((emxArray__common *)deactivatedAnchor, i14, (int)sizeof
                    (boolean_T));
  loop_ub = (int)numAnchors;
  for (i14 = 0; i14 < loop_ub; i14++) {
    deactivatedAnchor->data[i14] = false;
  }

  for (ca = 0; ca < (int)numAnchors; ca++) {
    d0 = ((1.0 + (double)ca) - 1.0) * numPointsPerAnchor + 1.0;
    numGridsX = (1.0 + (double)ca) * numPointsPerAnchor;
    if (d0 > numGridsX) {
      i14 = 0;
      i15 = 0;
    } else {
      i14 = (int)d0 - 1;
      i15 = (int)numGridsX;
    }

    ii = i15 - i14;
    loop_ub = i15 - i14;
    for (i15 = 0; i15 < loop_ub; i15++) {
      activeAnchorFeature_data[i15] = (signed char)(featStatus[i14 + i15] > 0.0);
    }

    if (ii == 0) {
      numGridsX = 0.0;
    } else {
      numGridsX = activeAnchorFeature_data[0];
      for (idx = 2; idx <= ii; idx++) {
        numGridsX += (double)activeAnchorFeature_data[idx - 1];
      }
    }

    if (numGridsX < ceil(numPointsPerAnchor / 2.0)) {
      d0 = ((1.0 + (double)ca) - 1.0) * numPointsPerAnchor + 1.0;
      numGridsX = (1.0 + (double)ca) * numPointsPerAnchor;
      if (d0 > numGridsX) {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)d0 - 1;
        i15 = (int)numGridsX;
      }

      loop_ub = i15 - i14;
      for (i16 = 0; i16 < loop_ub; i16++) {
        ii_data[i16] = i14 + i16;
      }

      loop_ub = i15 - i14;
      for (i14 = 0; i14 < loop_ub; i14++) {
        featStatus[ii_data[i14]] = 0.0;
      }

      deactivatedAnchor->data[ca] = true;
    }
  }

  /*  if no anchor is deactivated, return input feature */
  if (!b_any(deactivatedAnchor)) {
    notEnough = false;
  } else {
    idx = 0;
    ii = 1;
    exitg2 = false;
    while ((!exitg2) && (ii < 33)) {
      guard3 = false;
      if (featStatus[ii - 1] == 0.0) {
        idx++;
        ii_data[idx - 1] = ii;
        if (idx >= 32) {
          exitg2 = true;
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }

      if (guard3) {
        ii++;
      }
    }

    if (1 > idx) {
      loop_ub = 0;
    } else {
      loop_ub = idx;
    }

    for (i14 = 0; i14 < loop_ub; i14++) {
      activeAnchorFeature_data[i14] = (signed char)ii_data[i14];
    }

    for (i14 = 0; i14 < loop_ub; i14++) {
      ii_data[i14] = activeAnchorFeature_data[i14];
    }

    for (i14 = 0; i14 < loop_ub; i14++) {
      idxDeadFeatures_data[i14] = (signed char)ii_data[i14];
    }

    for (i = 0; i < 32; i++) {
      b_featStatus[i] = (featStatus[i] > 0.0);
    }

    b_eml_li_find(b_featStatus, ii_data, ii_size);
    activeFeatures_size_idx_0 = ii_size[0];
    for (i14 = 0; i14 < 2; i14++) {
      idx = ii_size[0];
      for (i15 = 0; i15 < idx; i15++) {
        activeFeatures_data[i15 + ii_size[0] * i14] = features[(ii_data[i15] +
          (i14 << 5)) - 1];
      }
    }

    /* % go through each bin and fill free grid with width minDistance */
    /*  create structure to store feature locations */
    /*  field name idx, xidx, yidx, offset, count, grid */
    binWidth = rt_roundd_snf((735.0 - 2.0 * borderLength) / numBinsX);
    binHeight = rt_roundd_snf((452.0 - 2.0 * borderLength) / numBinsY);
    numGridsX = ceil(binWidth / minDistance);
    numPointsPerAnchor = ceil(binHeight / minDistance);

    /*  initialize structBin */
    emxInit_int8_T(&t0_grid, 2);
    i14 = t0_grid->size[0] * t0_grid->size[1];
    t0_grid->size[0] = (int)numGridsX;
    t0_grid->size[1] = (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)t0_grid, i14, (int)sizeof(signed char));
    idx = (int)numGridsX * (int)numPointsPerAnchor;
    for (i14 = 0; i14 < idx; i14++) {
      t0_grid->data[i14] = 1;
    }

    emxInitStruct_struct_T(&expl_temp);
    i14 = expl_temp.grid->size[0] * expl_temp.grid->size[1];
    expl_temp.grid->size[0] = t0_grid->size[0];
    expl_temp.grid->size[1] = t0_grid->size[1];
    emxEnsureCapacity((emxArray__common *)expl_temp.grid, i14, (int)sizeof
                      (signed char));
    idx = t0_grid->size[0] * t0_grid->size[1];
    for (i14 = 0; i14 < idx; i14++) {
      expl_temp.grid->data[i14] = t0_grid->data[i14];
    }

    emxFree_int8_T(&t0_grid);
    expl_temp.count = 0.0;
    for (i14 = 0; i14 < 2; i14++) {
      expl_temp.offset[i14] = 0.0;
    }

    b_emxInit_struct_T(&structBin, 1);
    emxInit_real_T(&featureCounts, 1);
    repmat(expl_temp, numBins, structBin);
    i14 = featureCounts->size[0];
    featureCounts->size[0] = (int)numBins;
    emxEnsureCapacity((emxArray__common *)featureCounts, i14, (int)sizeof(double));
    idx = (int)numBins;
    emxFreeStruct_struct_T(&expl_temp);
    for (i14 = 0; i14 < idx; i14++) {
      featureCounts->data[i14] = 0.0;
    }

    /*  [1 numBinsY+1 ...; */
    /*   2 numBinsY+2 ...] */
    b_i = 1U;
    for (idx = 0; idx < (int)numBinsX; idx++) {
      for (ii = 0; ii < (int)numBinsY; ii++) {
        structBin->data[(int)b_i - 1].offset[0] = borderLength + binWidth *
          ((1.0 + (double)idx) - 1.0);
        structBin->data[(int)b_i - 1].offset[1] = borderLength + binHeight *
          ((1.0 + (double)ii) - 1.0);
        b_i++;
      }
    }

    /*  count number of features/bin and their location in grid */
    for (i = 0; i < ii_size[0]; i++) {
      for (i14 = 0; i14 < 2; i14++) {
        activeFeatures[i14] = activeFeatures_data[i + ii_size[0] * i14] -
          borderLength;
      }

      b_binWidth[0] = binWidth;
      b_binWidth[1] = binHeight;
      rdivide(activeFeatures, b_binWidth, bin);
      b_ceil(bin);

      /*  get which bin this feature belongs to */
      /*  TODO perhaps delete these features */
      if (bin[0] > numBinsX) {
        bin[0] = numBinsX;
      }

      if (bin[1] > numBinsY) {
        bin[1] = numBinsY;
      }

      /*  idx of the bin where this feature belongs to */
      numGridsX = (bin[0] - 1.0) * numBinsY + bin[1];

      /*  increment number of features in this bin by 1 */
      featureCounts->data[(int)numGridsX - 1]++;

      /*  relative location in bin */
      /*  get grid coordinates of this  */
      for (i14 = 0; i14 < 2; i14++) {
        bin[i14] = (activeFeatures_data[i + ii_size[0] * i14] - structBin->data
                    [(int)numGridsX - 1].offset[i14]) / minDistance;
      }

      b_ceil(bin);
      structBin->data[(int)numGridsX - 1].grid->data[((int)bin[0] +
        structBin->data[(int)numGridsX - 1].grid->size[0] * ((int)bin[1] - 1)) -
        1] = 0;
    }

    /* % go through each bin, compute mask from grid and find new features under mask */
    /*  use erosion to create mask from free grid */
    /*  erosion set target point to 0 if any point in its neighborhood is 0 */
    /*  in our case, a grid is marked as 0 (deactivated) if it's neighborhood has */
    /*  0 (occupied) */
    memset(&features_tmp[0], 0, sizeof(double) << 6);
    emxInit_real_T(&b_featureCounts, 1);

    /*  create erosion neighbourhood */
    /*  sort bins with ascending occupancy */
    i14 = b_featureCounts->size[0];
    b_featureCounts->size[0] = featureCounts->size[0];
    emxEnsureCapacity((emxArray__common *)b_featureCounts, i14, (int)sizeof
                      (double));
    idx = featureCounts->size[0];
    for (i14 = 0; i14 < idx; i14++) {
      b_featureCounts->data[i14] = featureCounts->data[i14];
    }

    emxInit_int32_T(&iidx, 1);
    c_eml_sort(b_featureCounts, featureCounts, iidx);
    i14 = featureCounts->size[0];
    featureCounts->size[0] = iidx->size[0];
    emxEnsureCapacity((emxArray__common *)featureCounts, i14, (int)sizeof(double));
    idx = iidx->size[0];
    emxFree_real_T(&b_featureCounts);
    for (i14 = 0; i14 < idx; i14++) {
      featureCounts->data[i14] = iidx->data[i14];
    }

    /*  number of features still to be found */
    numPointsPerAnchor = 32.0 - (double)ii_size[0];

    /*  average number of features to be found in each bin */
    numGridsX = (32.0 - (double)ii_size[0]) / numBins;

    /*  TODO merge with mask from motion */
    numCheckedBin = 0.0;
    numFeatureFound = 0.0;
    i = 0;
    emxInit_int8_T(&erodedGrid, 2);
    b_emxInit_real32_T(&arrBinCandidates_pLocation, 2);
    emxInit_real32_T(&arrBinCandidates_pMetric, 1);
    b_emxInit_real32_T(&roundedCandLocation, 2);
    b_emxInit_int8_T(&validCandidates, 1);
    emxInit_real32_T(&arrMetric, 1);
    b_emxInit_real32_T(&binFeatures, 2);
    emxInit_real_T(&ranking, 1);
    b_emxInit_real_T(&r10, 2);
    emxInit_real32_T(&varargin_2, 1);
    b_emxInit_boolean_T(&b_validCandidates, 1);
    b_emxInit_boolean_T(&c_validCandidates, 1);
    emxInit_uint8_T(&b_greyImg, 2);
    emxInit_int8_T(&b_erodedGrid, 2);
    emxInit_int8_T(&r11, 2);
    emxInit_int8_T(&c_erodedGrid, 2);
    emxInit_int8_T(&d_erodedGrid, 2);
    exitg1 = false;
    while ((!exitg1) && (i <= featureCounts->size[0] - 1)) {
      numFeatureFound_old = numFeatureFound;
      numCheckedBin++;

      /*  create a mask in this bin so that only features whose neighborhood */
      /*  is not occupied will be saved */
      imerode(structBin->data[(int)featureCounts->data[i] - 1].grid, erodedGrid);
      i14 = b_erodedGrid->size[0] * b_erodedGrid->size[1];
      b_erodedGrid->size[0] = erodedGrid->size[1];
      b_erodedGrid->size[1] = erodedGrid->size[0];
      emxEnsureCapacity((emxArray__common *)b_erodedGrid, i14, (int)sizeof
                        (signed char));
      idx = erodedGrid->size[0];
      for (i14 = 0; i14 < idx; i14++) {
        ii = erodedGrid->size[1];
        for (i15 = 0; i15 < ii; i15++) {
          b_erodedGrid->data[i15 + b_erodedGrid->size[0] * i14] =
            erodedGrid->data[i14 + erodedGrid->size[0] * i15];
        }
      }

      i14 = r11->size[0] * r11->size[1];
      r11->size[0] = (int)minDistance;
      r11->size[1] = (int)minDistance;
      emxEnsureCapacity((emxArray__common *)r11, i14, (int)sizeof(signed char));
      idx = (int)minDistance * (int)minDistance;
      for (i14 = 0; i14 < idx; i14++) {
        r11->data[i14] = 1;
      }

      kron(b_erodedGrid, r11, erodedGrid);

      /*  crop right and down side to correct size mistake due to rounding */
      if (1.0 > binHeight) {
        idx = 0;
      } else {
        idx = (int)binHeight;
      }

      if (1.0 > binWidth) {
        ii = 0;
      } else {
        ii = (int)binWidth;
      }

      /*  detect fast feature in current bin */
      d0 = ((structBin->data[(int)featureCounts->data[i] - 1].offset[1] + 1.0) +
            binHeight) - 1.0;
      if (structBin->data[(int)featureCounts->data[i] - 1].offset[1] + 1.0 > d0)
      {
        i14 = 0;
        i15 = 0;
      } else {
        i14 = (int)(structBin->data[(int)featureCounts->data[i] - 1].offset[1] +
                    1.0) - 1;
        i15 = (int)d0;
      }

      d0 = ((structBin->data[(int)featureCounts->data[i] - 1].offset[0] + 1.0) +
            binWidth) - 1.0;
      if (structBin->data[(int)featureCounts->data[i] - 1].offset[0] + 1.0 > d0)
      {
        i16 = 0;
        ca = 0;
      } else {
        i16 = (int)(structBin->data[(int)featureCounts->data[i] - 1].offset[0] +
                    1.0) - 1;
        ca = (int)d0;
      }

      i17 = b_greyImg->size[0] * b_greyImg->size[1];
      b_greyImg->size[0] = i15 - i14;
      b_greyImg->size[1] = ca - i16;
      emxEnsureCapacity((emxArray__common *)b_greyImg, i17, (int)sizeof(unsigned
        char));
      b_loop_ub = ca - i16;
      for (ca = 0; ca < b_loop_ub; ca++) {
        c_loop_ub = i15 - i14;
        for (i17 = 0; i17 < c_loop_ub; i17++) {
          b_greyImg->data[i17 + b_greyImg->size[0] * ca] = greyImg[(i14 + i17) +
            452 * (i16 + ca)];
        }
      }

      b_detectFASTFeatures(b_greyImg, arrBinCandidates_pLocation,
                           arrBinCandidates_pMetric);

      /*      arrBinCandidates = detectHarrisFeatures(subImg,'MinQuality',0.5,'FilterSize',5); */
      /*  if found features in this bin */
      guard1 = false;
      guard2 = false;
      if (FeaturePointsImpl_get_Count(arrBinCandidates_pLocation) > 0.0) {
        /*  get the mask value of found feature candidates */
        i14 = roundedCandLocation->size[0] * roundedCandLocation->size[1];
        roundedCandLocation->size[0] = arrBinCandidates_pLocation->size[0];
        roundedCandLocation->size[1] = 2;
        emxEnsureCapacity((emxArray__common *)roundedCandLocation, i14, (int)
                          sizeof(float));
        b_loop_ub = arrBinCandidates_pLocation->size[0] *
          arrBinCandidates_pLocation->size[1];
        for (i14 = 0; i14 < b_loop_ub; i14++) {
          roundedCandLocation->data[i14] = arrBinCandidates_pLocation->data[i14];
        }

        b_round(roundedCandLocation);
        b_loop_ub = roundedCandLocation->size[0];
        i14 = arrMetric->size[0];
        arrMetric->size[0] = b_loop_ub;
        emxEnsureCapacity((emxArray__common *)arrMetric, i14, (int)sizeof(float));
        for (i14 = 0; i14 < b_loop_ub; i14++) {
          arrMetric->data[i14] = roundedCandLocation->data[i14 +
            roundedCandLocation->size[0]];
        }

        b_loop_ub = roundedCandLocation->size[0];
        i14 = varargin_2->size[0];
        varargin_2->size[0] = b_loop_ub;
        emxEnsureCapacity((emxArray__common *)varargin_2, i14, (int)sizeof(float));
        for (i14 = 0; i14 < b_loop_ub; i14++) {
          varargin_2->data[i14] = roundedCandLocation->data[i14];
        }

        i14 = c_erodedGrid->size[0] * c_erodedGrid->size[1];
        c_erodedGrid->size[0] = idx;
        c_erodedGrid->size[1] = ii;
        emxEnsureCapacity((emxArray__common *)c_erodedGrid, i14, (int)sizeof
                          (signed char));
        for (i14 = 0; i14 < ii; i14++) {
          for (i15 = 0; i15 < idx; i15++) {
            c_erodedGrid->data[i15 + c_erodedGrid->size[0] * i14] =
              erodedGrid->data[i15 + erodedGrid->size[0] * i14];
          }
        }

        for (i14 = 0; i14 < 2; i14++) {
          siz[i14] = c_erodedGrid->size[i14];
        }

        i14 = d_erodedGrid->size[0] * d_erodedGrid->size[1];
        d_erodedGrid->size[0] = idx;
        d_erodedGrid->size[1] = ii;
        emxEnsureCapacity((emxArray__common *)d_erodedGrid, i14, (int)sizeof
                          (signed char));
        for (i14 = 0; i14 < ii; i14++) {
          for (i15 = 0; i15 < idx; i15++) {
            d_erodedGrid->data[i15 + d_erodedGrid->size[0] * i14] =
              erodedGrid->data[i15 + erodedGrid->size[0] * i14];
          }
        }

        i14 = validCandidates->size[0];
        validCandidates->size[0] = arrMetric->size[0];
        emxEnsureCapacity((emxArray__common *)validCandidates, i14, (int)sizeof
                          (signed char));
        idx = arrMetric->size[0];
        for (i14 = 0; i14 < idx; i14++) {
          validCandidates->data[i14] = d_erodedGrid->data[((int)arrMetric->
            data[i14] + siz[0] * ((int)varargin_2->data[i14] - 1)) - 1];
        }

        if (!c_any(validCandidates)) {
          guard1 = true;
        } else {
          /*  filter candidates with mask value */
          i14 = c_validCandidates->size[0];
          c_validCandidates->size[0] = validCandidates->size[0];
          emxEnsureCapacity((emxArray__common *)c_validCandidates, i14, (int)
                            sizeof(boolean_T));
          idx = validCandidates->size[0];
          for (i14 = 0; i14 < idx; i14++) {
            c_validCandidates->data[i14] = (validCandidates->data[i14] == 1);
          }

          eml_li_find(c_validCandidates, iidx);
          i14 = roundedCandLocation->size[0] * roundedCandLocation->size[1];
          roundedCandLocation->size[0] = iidx->size[0];
          roundedCandLocation->size[1] = 2;
          emxEnsureCapacity((emxArray__common *)roundedCandLocation, i14, (int)
                            sizeof(float));
          for (i14 = 0; i14 < 2; i14++) {
            idx = iidx->size[0];
            for (i15 = 0; i15 < idx; i15++) {
              roundedCandLocation->data[i15 + roundedCandLocation->size[0] * i14]
                = arrBinCandidates_pLocation->data[(iidx->data[i15] +
                arrBinCandidates_pLocation->size[0] * i14) - 1];
            }
          }

          i14 = b_validCandidates->size[0];
          b_validCandidates->size[0] = validCandidates->size[0];
          emxEnsureCapacity((emxArray__common *)b_validCandidates, i14, (int)
                            sizeof(boolean_T));
          idx = validCandidates->size[0];
          for (i14 = 0; i14 < idx; i14++) {
            b_validCandidates->data[i14] = (validCandidates->data[i14] == 1);
          }

          eml_li_find(b_validCandidates, iidx);
          i14 = arrMetric->size[0];
          arrMetric->size[0] = iidx->size[0];
          emxEnsureCapacity((emxArray__common *)arrMetric, i14, (int)sizeof
                            (float));
          idx = iidx->size[0];
          for (i14 = 0; i14 < idx; i14++) {
            arrMetric->data[i14] = arrBinCandidates_pMetric->data[iidx->data[i14]
              - 1];
          }

          /*  select x strongest features in this bin. */
          /*  x should be smaller than numFeatureToFindBin and numFeatureToFind */
          eml_sort(arrMetric, varargin_2, iidx);
          i14 = ranking->size[0];
          ranking->size[0] = iidx->size[0];
          emxEnsureCapacity((emxArray__common *)ranking, i14, (int)sizeof(double));
          idx = iidx->size[0];
          for (i14 = 0; i14 < idx; i14++) {
            ranking->data[i14] = iidx->data[i14];
          }

          varargin_1[0] = ceil(numGridsX);
          varargin_1[1] = numPointsPerAnchor;
          varargin_1[2] = arrMetric->size[0];
          idx = 1;
          numPointsPerAnchor = varargin_1[0];
          if (rtIsNaN(varargin_1[0])) {
            idx = 2;
            numPointsPerAnchor = varargin_1[1];
          }

          while (idx + 1 < 4) {
            if (varargin_1[idx] < numPointsPerAnchor) {
              numPointsPerAnchor = varargin_1[idx];
            }

            idx++;
          }

          if (1.0 > numPointsPerAnchor) {
            idx = 0;
          } else {
            idx = (int)numPointsPerAnchor;
          }

          i14 = binFeatures->size[0] * binFeatures->size[1];
          binFeatures->size[0] = idx;
          binFeatures->size[1] = 2;
          emxEnsureCapacity((emxArray__common *)binFeatures, i14, (int)sizeof
                            (float));
          for (i14 = 0; i14 < 2; i14++) {
            for (i15 = 0; i15 < idx; i15++) {
              binFeatures->data[i15 + binFeatures->size[0] * i14] =
                roundedCandLocation->data[((int)ranking->data[i15] +
                roundedCandLocation->size[0] * i14) - 1];
            }
          }

          /*  add offset due to subimg */
          b_repmat(structBin->data[(int)featureCounts->data[i] - 1].offset, idx,
                   r10);
          i14 = binFeatures->size[0] * binFeatures->size[1];
          binFeatures->size[1] = 2;
          emxEnsureCapacity((emxArray__common *)binFeatures, i14, (int)sizeof
                            (float));
          idx = binFeatures->size[0];
          ii = binFeatures->size[1];
          idx *= ii;
          for (i14 = 0; i14 < idx; i14++) {
            binFeatures->data[i14] += (float)r10->data[i14];
          }

          /*  update temporary counters */
          numFeatureFound += (double)binFeatures->size[0];
          numPointsPerAnchor = (32.0 - (double)ii_size[0]) - numFeatureFound;
          if (numFeatureFound_old + 1.0 > numFeatureFound) {
            i14 = 0;
          } else {
            i14 = (int)(numFeatureFound_old + 1.0) - 1;
          }

          for (i15 = 0; i15 < 2; i15++) {
            idx = binFeatures->size[0];
            for (i16 = 0; i16 < idx; i16++) {
              features_tmp[(i14 + i16) + (i15 << 5)] = binFeatures->data[i16 +
                binFeatures->size[0] * i15];
            }
          }

          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        if (numPointsPerAnchor <= 0.0) {
          exitg1 = true;
        } else {
          numGridsX = numPointsPerAnchor / (numBins - numCheckedBin);
          guard1 = true;
        }
      }

      if (guard1) {
        i++;
      }
    }

    emxFree_int8_T(&d_erodedGrid);
    emxFree_int8_T(&c_erodedGrid);
    emxFree_int8_T(&r11);
    emxFree_int8_T(&b_erodedGrid);
    emxFree_uint8_T(&b_greyImg);
    emxFree_boolean_T(&c_validCandidates);
    emxFree_boolean_T(&b_validCandidates);
    emxFree_int32_T(&iidx);
    emxFree_real32_T(&varargin_2);
    emxFree_real_T(&r10);
    emxFree_real_T(&ranking);
    emxFree_real32_T(&binFeatures);
    emxFree_real32_T(&arrMetric);
    emxFree_int8_T(&validCandidates);
    emxFree_real32_T(&roundedCandLocation);
    emxFree_real32_T(&arrBinCandidates_pMetric);
    emxFree_real32_T(&arrBinCandidates_pLocation);
    emxFree_int8_T(&erodedGrid);
    emxFree_real_T(&featureCounts);
    b_emxFree_struct_T(&structBin);

    /* % output */
    if (numFeatureFound > 0.0) {
      all(features_tmp, bv0);
      b_eml_li_find(bv0, ii_data, ii_size);
      for (i14 = 0; i14 < 2; i14++) {
        idx = ii_size[0];
        for (i15 = 0; i15 < idx; i15++) {
          features_tmp_data[i15 + ii_size[0] * i14] = features_tmp[(ii_data[i15]
            + (i14 << 5)) - 1];
        }
      }

      /*  set flag and save features */
      if (numFeatureFound < 32.0 - (double)activeFeatures_size_idx_0) {
        notEnough = true;
        for (i14 = 0; i14 < 2; i14++) {
          loop_ub = ii_size[0];
          for (i15 = 0; i15 < loop_ub; i15++) {
            features[(idxDeadFeatures_data[i15] + (i14 << 5)) - 1] =
              features_tmp_data[i15 + ii_size[0] * i14];
          }
        }

        loop_ub = (int)numFeatureFound;
        for (i14 = 0; i14 < loop_ub; i14++) {
          tmp_data[i14] = idxDeadFeatures_data[i14];
        }

        loop_ub = (int)numFeatureFound;
        for (i14 = 0; i14 < loop_ub; i14++) {
          featStatus[tmp_data[i14] - 1] = 2.0;
        }
      } else {
        notEnough = false;
        for (i14 = 0; i14 < 2; i14++) {
          idx = ii_size[0];
          for (i15 = 0; i15 < idx; i15++) {
            features[(idxDeadFeatures_data[i15] + (i14 << 5)) - 1] =
              features_tmp_data[i15 + ii_size[0] * i14];
          }
        }

        for (i14 = 0; i14 < loop_ub; i14++) {
          ii_data[i14] = idxDeadFeatures_data[i14];
        }

        for (i14 = 0; i14 < loop_ub; i14++) {
          featStatus[ii_data[i14] - 1] = 2.0;
        }
      }
    } else {
      /*  if haven't found feature at all, return original */
      notEnough = true;
      memcpy(&featStatus[0], &featStatus_old[0], sizeof(double) << 5);
    }
  }

  emxFree_boolean_T(&deactivatedAnchor);
  return notEnough;
}

/*
 * Arguments    : const b_struct_T a
 *                double varargin_1
 *                emxArray_b_struct_T *b
 * Return Type  : void
 */
static void repmat(const b_struct_T a, double varargin_1, emxArray_b_struct_T *b)
{
  int i8;
  int loop_ub;
  i8 = b->size[0];
  b->size[0] = (int)varargin_1;
  emxEnsureCapacity_struct_T(b, i8);
  loop_ub = (int)varargin_1;
  for (i8 = 0; i8 < loop_ub; i8++) {
    emxCopyStruct_struct_T(&b->data[i8], &a);
  }
}

/*
 * Arguments    : double u
 * Return Type  : double
 */
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

/*
 * STEREOMATCHING finds correspondences in the left image given points in the
 * right image. Steoro images should be rectified first.
 *  pointsR Nx2 Array
 *  minDisp Nx1 Array
 *  maxDisp Nx1 Array
 *  shift scalar
 * % parse parameters
 * Arguments    : const unsigned char imgR[332220]
 *                const unsigned char imgL[332220]
 *                const double pointsR_data[]
 *                const int pointsR_size[2]
 *                double minDisp_data[]
 *                int minDisp_size[1]
 *                double maxDisp_data[]
 *                int maxDisp_size[1]
 *                double pointsL_data[]
 *                int pointsL_size[2]
 *                boolean_T status_data[]
 *                int status_size[1]
 * Return Type  : void
 */
static void stereoRight2Left(const unsigned char imgR[332220], const unsigned
  char imgL[332220], const double pointsR_data[], const int pointsR_size[2],
  double minDisp_data[], int minDisp_size[1], double maxDisp_data[], int
  maxDisp_size[1], double pointsL_data[], int pointsL_size[2], boolean_T
  status_data[], int status_size[1])
{
  static double b_imgR[332220];
  static double b_imgL[332220];
  int i6;
  int k;
  int loop_ub;
  int counter;
  emxArray_real_T *template;
  emxArray_real_T *roi;
  emxArray_real_T *ssd;
  emxArray_real_T *a;
  emxArray_real_T *c_imgL;
  double posx;
  double posy;
  double b[4];
  double boundT[4];
  int ibtile;
  int unnamed_idx_1;
  int ixstart;
  int ix;
  double boundR[4];
  double m;
  double b_m[2];
  int numPix;
  int32_T exitg2;
  double sz[2];
  double s;
  boolean_T exitg1;
  for (i6 = 0; i6 < 332220; i6++) {
    b_imgR[i6] = (double)imgR[i6] / 255.0;
    b_imgL[i6] = (double)imgL[i6] / 255.0;
  }

  /*  default values */
  for (k = 0; k < maxDisp_size[0]; k++) {
    maxDisp_data[k] = rt_roundd_snf(maxDisp_data[k]);
  }

  for (k = 0; k < minDisp_size[0]; k++) {
    minDisp_data[k] = rt_roundd_snf(minDisp_data[k]);
  }

  /* % preset output */
  status_size[0] = pointsR_size[0];
  loop_ub = pointsR_size[0];
  for (i6 = 0; i6 < loop_ub; i6++) {
    status_data[i6] = true;
  }

  pointsL_size[0] = pointsR_size[0];
  pointsL_size[1] = 2;
  loop_ub = pointsR_size[0] * pointsR_size[1];
  for (i6 = 0; i6 < loop_ub; i6++) {
    pointsL_data[i6] = pointsR_data[i6];
  }

  /* % Matching by convolving along a stripe around epipolar line */
  /*  extract template around points then extract stripe in other image around */
  /*  epipolar line */
  /*  template = zeros(winSize*2+1,winSize*2+1); */
  /*  roi = zeros(2*(shift+winSize)+1,2*winSize+maxDisp-minDisp+1); */
  /*  match = repmat(struct('template',template,'roi',roi),numPoints,1); */
  counter = 0;
  b_emxInit_real_T(&template, 2);
  b_emxInit_real_T(&roi, 2);
  b_emxInit_real_T(&ssd, 2);
  emxInit_real_T(&a, 1);
  b_emxInit_real_T(&c_imgL, 2);
  while (counter <= pointsR_size[0] - 1) {
    posx = rt_roundd_snf(pointsR_data[counter]);
    posy = rt_roundd_snf(pointsR_data[counter + pointsR_size[0]]);
    b[0] = posy - 2.0;
    b[1] = posy + 2.0;
    b[2] = posx - 2.0;
    b[3] = posx + 2.0;
    for (i6 = 0; i6 < 4; i6++) {
      boundT[i6] = b[i6];
    }

    if (posy - 2.0 <= 0.0) {
      /* top */
      boundT[0] = 1.0;
    }

    if (posy + 2.0 > 452.0) {
      /* bottom */
      boundT[1] = 452.0;
    }

    if (posx - 2.0 <= 0.0) {
      /* left */
      boundT[2] = 1.0;
    }

    if (posx + 2.0 > 735.0) {
      /* right */
      boundT[3] = 735.0;
    }

    if (boundT[0] > boundT[1]) {
      i6 = 0;
      ibtile = 0;
    } else {
      i6 = (int)boundT[0] - 1;
      ibtile = (int)boundT[1];
    }

    if (boundT[2] > boundT[3]) {
      unnamed_idx_1 = 0;
      ixstart = 0;
    } else {
      unnamed_idx_1 = (int)boundT[2] - 1;
      ixstart = (int)boundT[3];
    }

    k = template->size[0] * template->size[1];
    template->size[0] = ibtile - i6;
    template->size[1] = ixstart - unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)template, k, (int)sizeof(double));
    loop_ub = ixstart - unnamed_idx_1;
    for (ixstart = 0; ixstart < loop_ub; ixstart++) {
      ix = ibtile - i6;
      for (k = 0; k < ix; k++) {
        template->data[k + template->size[0] * ixstart] = b_imgR[(i6 + k) + 452 *
          (unnamed_idx_1 + ixstart)];
      }
    }

    /* mxn */
    b[0] = (posy - 1.0) - 2.0;
    b[1] = (posy + 1.0) + 2.0;
    b[2] = (posx + minDisp_data[counter]) - 2.0;
    b[3] = (posx + maxDisp_data[counter]) + 2.0;
    for (i6 = 0; i6 < 4; i6++) {
      boundR[i6] = b[i6];
    }

    if ((posy - 1.0) - 2.0 <= 0.0) {
      /* top */
      boundR[0] = 1.0;
    }

    if ((posy + 1.0) + 2.0 > 452.0) {
      /* bottom */
      boundR[1] = 452.0;
    }

    if (b[2] <= 0.0) {
      /* left */
      boundR[2] = 1.0;
    }

    if (b[3] > 735.0) {
      /* right */
      boundR[3] = 735.0;
    }

    if (boundR[0] > boundR[1]) {
      i6 = 0;
      ibtile = 0;
    } else {
      i6 = (int)boundR[0] - 1;
      ibtile = (int)boundR[1];
    }

    if (boundR[2] > boundR[3]) {
      unnamed_idx_1 = 0;
      ixstart = 0;
    } else {
      unnamed_idx_1 = (int)boundR[2] - 1;
      ixstart = (int)boundR[3];
    }

    /* mmxnn */
    m = (boundT[1] - boundT[0]) + 1.0;

    /*  vectorize */
    k = c_imgL->size[0] * c_imgL->size[1];
    c_imgL->size[0] = ibtile - i6;
    c_imgL->size[1] = ixstart - unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)c_imgL, k, (int)sizeof(double));
    loop_ub = ixstart - unnamed_idx_1;
    for (ixstart = 0; ixstart < loop_ub; ixstart++) {
      ix = ibtile - i6;
      for (k = 0; k < ix; k++) {
        c_imgL->data[k + c_imgL->size[0] * ixstart] = b_imgL[(i6 + k) + 452 *
          (unnamed_idx_1 + ixstart)];
      }
    }

    b_m[0] = m;
    b_m[1] = (boundT[3] - boundT[2]) + 1.0;
    im2col(c_imgL, b_m, roi);
    numPix = template->size[0] * template->size[1];
    i6 = a->size[0];
    a->size[0] = template->size[0] * template->size[1];
    emxEnsureCapacity((emxArray__common *)a, i6, (int)sizeof(double));
    loop_ub = template->size[0] * template->size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      a->data[i6] = template->data[i6];
    }

    ibtile = template->size[0] * template->size[1];
    unnamed_idx_1 = roi->size[1];
    i6 = template->size[0] * template->size[1];
    template->size[0] = ibtile;
    template->size[1] = unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)template, i6, (int)sizeof(double));
    unnamed_idx_1 = roi->size[1];
    if ((ibtile == 0) || (unnamed_idx_1 == 0)) {
    } else {
      ixstart = 1;
      do {
        exitg2 = 0;
        unnamed_idx_1 = roi->size[1];
        if (ixstart <= unnamed_idx_1) {
          ibtile = (ixstart - 1) * a->size[0];
          for (k = 1; k <= a->size[0]; k++) {
            template->data[(ibtile + k) - 1] = a->data[k - 1];
          }

          ixstart++;
        } else {
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }

    /*  if roi is smaller than template block set track failure */
    if ((roi->size[0] == 0) || (roi->size[1] == 0)) {
      status_data[counter] = false;
    } else {
      /*  SSD */
      i6 = template->size[0] * template->size[1];
      emxEnsureCapacity((emxArray__common *)template, i6, (int)sizeof(double));
      ibtile = template->size[0];
      unnamed_idx_1 = template->size[1];
      loop_ub = ibtile * unnamed_idx_1;
      for (i6 = 0; i6 < loop_ub; i6++) {
        template->data[i6] = (template->data[i6] - roi->data[i6]) *
          (template->data[i6] - roi->data[i6]);
      }

      for (i6 = 0; i6 < 2; i6++) {
        sz[i6] = template->size[i6];
      }

      i6 = ssd->size[0] * ssd->size[1];
      ssd->size[0] = 1;
      ssd->size[1] = (int)sz[1];
      emxEnsureCapacity((emxArray__common *)ssd, i6, (int)sizeof(double));
      ix = -1;
      ibtile = -1;
      for (unnamed_idx_1 = 1; unnamed_idx_1 <= template->size[1]; unnamed_idx_1
           ++) {
        ixstart = ix + 1;
        ix++;
        s = template->data[ixstart];
        for (k = 2; k <= template->size[0]; k++) {
          ix++;
          s += template->data[ix];
        }

        ibtile++;
        ssd->data[ibtile] = s;
      }

      ixstart = 1;
      s = ssd->data[0];
      unnamed_idx_1 = 0;
      if (ssd->size[1] > 1) {
        if (rtIsNaN(ssd->data[0])) {
          ix = 2;
          exitg1 = false;
          while ((!exitg1) && (ix <= ssd->size[1])) {
            ixstart = ix;
            if (!rtIsNaN(ssd->data[ix - 1])) {
              s = ssd->data[ix - 1];
              unnamed_idx_1 = ix - 1;
              exitg1 = true;
            } else {
              ix++;
            }
          }
        }

        if (ixstart < ssd->size[1]) {
          while (ixstart + 1 <= ssd->size[1]) {
            if (ssd->data[ixstart] < s) {
              s = ssd->data[ixstart];
              unnamed_idx_1 = ixstart;
            }

            ixstart++;
          }
        }
      }

      /*  failure if error larger than threshold */
      if (s / (double)numPix > 0.1) {
        status_data[counter] = false;
      } else {
        sz[0] = (((boundR[1] - boundR[0]) + 1.0) - m) + 1.0;
        ibtile = div_s32(unnamed_idx_1, (int)sz[0]);
        pointsL_data[counter] = (((double)(ibtile + 1) + boundR[2]) - 1.0) +
          (posx - boundT[2]);
        pointsL_data[counter + pointsR_size[0]] = (((double)((unnamed_idx_1 -
          ibtile * (int)sz[0]) + 1) + boundR[0]) - 1.0) + (posy - boundT[0]);
        status_data[counter] = true;
      }
    }

    counter++;
  }

  emxFree_real_T(&c_imgL);
  emxFree_real_T(&a);
  emxFree_real_T(&ssd);
  emxFree_real_T(&roi);
  emxFree_real_T(&template);
}

/*
 * POINTEXTRACTION extracts feature points from stereo images
 *  ImGrayR_r rectified right stereo image
 *  ImGrayR_l rectified left stereo image
 *  updateVect Nx1 array showing the status of points
 *  numAnchors scalar number of anchors
 *  binx number of bins in x direction (horizontal)
 *  binx number of bins in y direction (vertical)
 *  border width of border where a feature is considered invalid
 *  minDistance quaosi distance of between features in one grid
 *  h_u_apo estimated point location in the right image
 * Arguments    : const unsigned char ImGrayR_r[332220]
 *                const unsigned char ImGrayR_l[332220]
 *                double updateVect[32]
 *                double numAnchors
 *                double binx
 *                double biny
 *                double border
 *                double minDistance
 *                boolean_T useInitGuess
 *                const double h_u_apo_data[]
 *                const int h_u_apo_size[1]
 *                double pts_r_arr[64]
 *                double pts_l_arr[64]
 *                double useDisparity[32]
 *                double z_all[96]
 * Return Type  : void
 */
void pointextraction(const unsigned char ImGrayR_r[332220], const unsigned char
                     ImGrayR_l[332220], double updateVect[32], double numAnchors,
                     double binx, double biny, double border, double minDistance,
                     boolean_T useInitGuess, const double h_u_apo_data[], const
                     int h_u_apo_size[1], double pts_r_arr[64], double
                     pts_l_arr[64], double useDisparity[32], double z_all[96])
{
  emxArray_int32_T *r5;
  double flag;
  double num_points;
  double minDisp_data[32];
  int i;
  int ii_data[32];
  int ii;
  boolean_T exitg5;
  boolean_T guard4 = false;
  int loop_ub;
  signed char b_ii_data[32];
  int i13;
  signed char active_idx_data[32];
  double b_pts_l_arr[64];
  double dispVect[64];
  int tmp_size[2];
  int maxDisp_size[1];
  emxArray_real_T *r6;
  int new_active_idx_size[1];
  boolean_T new_active_idx_data[32];
  int points_size[2];
  double points_data[64];
  boolean_T b_new_active_idx_data[32];
  int b_new_active_idx_size[1];
  emxArray_boolean_T c_new_active_idx_data;
  boolean_T d_new_active_idx_data[32];
  int c_new_active_idx_size[1];
  emxArray_boolean_T e_new_active_idx_data;
  double tmp_data[64];
  boolean_T f_new_active_idx_data[32];
  int d_new_active_idx_size[1];
  emxArray_boolean_T g_new_active_idx_data;
  int ii_size[1];
  boolean_T h_new_active_idx_data[32];
  int e_new_active_idx_size[1];
  emxArray_boolean_T i_new_active_idx_data;
  boolean_T j_new_active_idx_data[32];
  int f_new_active_idx_size[1];
  emxArray_boolean_T k_new_active_idx_data;
  int b_tmp_size[2];
  vision_PointTracker_4 *obj;
  boolean_T b_updateVect[32];
  float b_points_data[64];
  void * ptrObj;
  float c_points_data[64];
  boolean_T exitg4;
  boolean_T guard3 = false;
  static const short value[8] = { 452, 735, 1, 1, 1, 1, 1, 1 };

  boolean_T exitg3;
  static const short iv0[8] = { 452, 735, 1, 1, 1, 1, 1, 1 };

  static unsigned char Iu8_gray[332220];
  emxArray_boolean_T *trackStatus;
  emxArray_real_T *scores;
  emxArray_real32_T *points;
  emxArray_real32_T *x;
  emxArray_real32_T *y;
  emxArray_boolean_T *r7;
  emxArray_boolean_T *r8;
  emxArray_boolean_T *b_x;
  emxArray_boolean_T *b_points;
  emxArray_boolean_T *b_trackStatus;
  emxArray_int32_T *r9;
  emxArray_boolean_T *c_trackStatus;
  emxArray_boolean_T *d_trackStatus;
  boolean_T exitg2;
  boolean_T guard2 = false;
  double maxDisp_data[32];
  int minDisp_size[1];
  boolean_T l_new_active_idx_data[32];
  int g_new_active_idx_size[1];
  boolean_T m_new_active_idx_data[32];
  int h_new_active_idx_size[1];
  boolean_T n_new_active_idx_data[32];
  int i_new_active_idx_size[1];
  boolean_T exitg1;
  boolean_T guard1 = false;
  (void)useInitGuess;
  (void)h_u_apo_data;
  (void)h_u_apo_size;

  /* % compute initial feature */
  /*  get anchor pose and estimated feature depth to compute initial feature */
  /*  location */
  /* % initialization */
  emxInit_int32_T(&r5, 1);
  if (!initialized_not_empty) {
    initialized_not_empty = true;

    /*  initialize features in the right image */
    initFeatures(ImGrayR_r, border, binx, biny, minDistance, prevPoints,
                 minDisp_data, &num_points, &flag);
    i = 0;
    ii = 1;
    exitg5 = false;
    while ((!exitg5) && (ii < 33)) {
      guard4 = false;
      if (minDisp_data[ii - 1] != 0.0) {
        i++;
        ii_data[i - 1] = ii;
        if (i >= 32) {
          exitg5 = true;
        } else {
          guard4 = true;
        }
      } else {
        guard4 = true;
      }

      if (guard4) {
        ii++;
      }
    }

    if (1 > i) {
      loop_ub = 0;
    } else {
      loop_ub = i;
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      b_ii_data[i13] = (signed char)ii_data[i13];
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      ii_data[i13] = b_ii_data[i13];
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      active_idx_data[i13] = (signed char)ii_data[i13];
    }

    /*  find correspondences in the left image */
    memset(&b_pts_l_arr[0], 0, sizeof(double) << 6);
    tmp_size[0] = loop_ub;
    tmp_size[1] = 2;
    for (i13 = 0; i13 < 2; i13++) {
      for (ii = 0; ii < loop_ub; ii++) {
        dispVect[ii + loop_ub * i13] = prevPoints[(active_idx_data[ii] + (i13 <<
          5)) - 1];
      }
    }

    maxDisp_size[0] = loop_ub;
    for (i13 = 0; i13 < loop_ub; i13++) {
      minDisp_data[i13] = 0.0;
    }

    emxInit_real_T(&r6, 1);
    i13 = r6->size[0];
    r6->size[0] = loop_ub;
    emxEnsureCapacity((emxArray__common *)r6, i13, (int)sizeof(double));
    for (i13 = 0; i13 < loop_ub; i13++) {
      r6->data[i13] = 40.0;
    }

    stereoRight2Left(ImGrayR_r, ImGrayR_l, dispVect, tmp_size, minDisp_data,
                     maxDisp_size, r6->data, r6->size, points_data, points_size,
                     new_active_idx_data, new_active_idx_size);
    b_new_active_idx_size[0] = new_active_idx_size[0];
    loop_ub = new_active_idx_size[0];
    emxFree_real_T(&r6);
    for (i13 = 0; i13 < loop_ub; i13++) {
      b_new_active_idx_data[i13] = ((int)new_active_idx_data[i13] > 0);
    }

    c_new_active_idx_data.data = (boolean_T *)&b_new_active_idx_data;
    c_new_active_idx_data.size = (int *)&b_new_active_idx_size;
    c_new_active_idx_data.allocatedSize = 32;
    c_new_active_idx_data.numDimensions = 1;
    c_new_active_idx_data.canFreeData = false;
    eml_li_find(&c_new_active_idx_data, r5);
    loop_ub = r5->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      ii_data[i13] = active_idx_data[r5->data[i13] - 1] - 1;
    }

    c_new_active_idx_size[0] = new_active_idx_size[0];
    loop_ub = new_active_idx_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      d_new_active_idx_data[i13] = ((int)new_active_idx_data[i13] > 0);
    }

    e_new_active_idx_data.data = (boolean_T *)&d_new_active_idx_data;
    e_new_active_idx_data.size = (int *)&c_new_active_idx_size;
    e_new_active_idx_data.allocatedSize = 32;
    e_new_active_idx_data.numDimensions = 1;
    e_new_active_idx_data.canFreeData = false;
    eml_li_find(&e_new_active_idx_data, r5);
    i = r5->size[0];
    for (i13 = 0; i13 < 2; i13++) {
      loop_ub = r5->size[0];
      for (ii = 0; ii < loop_ub; ii++) {
        tmp_data[ii + i * i13] = points_data[(r5->data[ii] + points_size[0] *
          i13) - 1];
      }
    }

    for (i13 = 0; i13 < 2; i13++) {
      for (ii = 0; ii < i; ii++) {
        b_pts_l_arr[ii_data[ii] + (i13 << 5)] = tmp_data[ii + i * i13];
      }
    }

    /*  reset features in right image if no correspondances are found in the */
    /*  left image */
    d_new_active_idx_size[0] = new_active_idx_size[0];
    loop_ub = new_active_idx_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      f_new_active_idx_data[i13] = (new_active_idx_data[i13] == 0);
    }

    g_new_active_idx_data.data = (boolean_T *)&f_new_active_idx_data;
    g_new_active_idx_data.size = (int *)&d_new_active_idx_size;
    g_new_active_idx_data.allocatedSize = 32;
    g_new_active_idx_data.numDimensions = 1;
    g_new_active_idx_data.canFreeData = false;
    eml_li_find(&g_new_active_idx_data, r5);
    ii_size[0] = r5->size[0];
    loop_ub = r5->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      ii_data[i13] = active_idx_data[r5->data[i13] - 1];
    }

    for (i13 = 0; i13 < 2; i13++) {
      loop_ub = ii_size[0];
      for (ii = 0; ii < loop_ub; ii++) {
        prevPoints[(ii_data[ii] + (i13 << 5)) - 1] = 0.0;
      }
    }

    e_new_active_idx_size[0] = new_active_idx_size[0];
    loop_ub = new_active_idx_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      h_new_active_idx_data[i13] = ((int)new_active_idx_data[i13] > 0);
    }

    i_new_active_idx_data.data = (boolean_T *)&h_new_active_idx_data;
    i_new_active_idx_data.size = (int *)&e_new_active_idx_size;
    i_new_active_idx_data.allocatedSize = 32;
    i_new_active_idx_data.numDimensions = 1;
    i_new_active_idx_data.canFreeData = false;
    eml_li_find(&i_new_active_idx_data, r5);
    ii_size[0] = r5->size[0];
    loop_ub = r5->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      ii_data[i13] = active_idx_data[r5->data[i13] - 1];
    }

    loop_ub = ii_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      updateVect[ii_data[i13] - 1] = 2.0;
    }

    /*  initialized KLT tracker */
    PointTracker_PointTracker(&pointTracker);
    f_new_active_idx_size[0] = new_active_idx_size[0];
    loop_ub = new_active_idx_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      j_new_active_idx_data[i13] = ((int)new_active_idx_data[i13] > 0);
    }

    k_new_active_idx_data.data = (boolean_T *)&j_new_active_idx_data;
    k_new_active_idx_data.size = (int *)&f_new_active_idx_size;
    k_new_active_idx_data.allocatedSize = 32;
    k_new_active_idx_data.numDimensions = 1;
    k_new_active_idx_data.canFreeData = false;
    eml_li_find(&k_new_active_idx_data, r5);
    b_tmp_size[0] = r5->size[0];
    b_tmp_size[1] = 2;
    for (i13 = 0; i13 < 2; i13++) {
      loop_ub = r5->size[0];
      for (ii = 0; ii < loop_ub; ii++) {
        dispVect[ii + b_tmp_size[0] * i13] = prevPoints[(active_idx_data
          [r5->data[ii] - 1] + (i13 << 5)) - 1];
      }
    }

    PointTracker_initialize(&pointTracker, dispVect, b_tmp_size, ImGrayR_r);

    /* % tracking and refill */
  } else {
    /*  set to track current visible points in right image for next iteration */
    obj = &pointTracker;
    for (i = 0; i < 32; i++) {
      b_updateVect[i] = (updateVect[i] > 0.0);
    }

    b_eml_li_find(b_updateVect, ii_data, ii_size);
    for (i13 = 0; i13 < 2; i13++) {
      loop_ub = ii_size[0];
      for (ii = 0; ii < loop_ub; ii++) {
        points_data[ii + ii_size[0] * i13] = prevPoints[(ii_data[ii] + (i13 << 5))
          - 1];
      }
    }

    loop_ub = ii_size[0] << 1;
    for (i13 = 0; i13 < loop_ub; i13++) {
      b_points_data[i13] = (float)points_data[i13];
    }

    obj->NumPoints = ii_size[0];
    ptrObj = obj->pTracker;
    loop_ub = ii_size[0] << 1;
    for (i13 = 0; i13 < loop_ub; i13++) {
      c_points_data[i13] = b_points_data[i13];
    }

    new_active_idx_size[0] = ii_size[0];
    loop_ub = ii_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      new_active_idx_data[i13] = true;
    }

    pointTracker_setPoints(ptrObj, &c_points_data[0], ii_size[0],
      &new_active_idx_data[0]);

    /*  indices of points that are to be tracked */
    i = 0;
    ii = 1;
    exitg4 = false;
    while ((!exitg4) && (ii < 33)) {
      guard3 = false;
      if (updateVect[ii - 1] != 0.0) {
        i++;
        ii_data[i - 1] = ii;
        if (i >= 32) {
          exitg4 = true;
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }

      if (guard3) {
        ii++;
      }
    }

    if (1 > i) {
      loop_ub = 0;
    } else {
      loop_ub = i;
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      b_ii_data[i13] = (signed char)ii_data[i13];
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      ii_data[i13] = b_ii_data[i13];
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      active_idx_data[i13] = (signed char)ii_data[i13];
    }

    /*  KLT Tracker tracks active points */
    obj = &pointTracker;
    if (!obj->isInitialized) {
      obj->isInitialized = true;
      for (i13 = 0; i13 < 8; i13++) {
        obj->inputVarSize1[i13] = (unsigned int)value[i13];
      }
    }

    i = 0;
    exitg3 = false;
    while ((!exitg3) && (i < 8)) {
      if (obj->inputVarSize1[i] != (unsigned int)iv0[i]) {
        for (i13 = 0; i13 < 8; i13++) {
          obj->inputVarSize1[i13] = (unsigned int)value[i13];
        }

        exitg3 = true;
      } else {
        i++;
      }
    }

    ptrObj = obj->pTracker;
    for (i13 = 0; i13 < 452; i13++) {
      for (ii = 0; ii < 735; ii++) {
        Iu8_gray[ii + 735 * i13] = ImGrayR_r[i13 + 452 * ii];
      }
    }

    b_emxInit_boolean_T(&trackStatus, 1);
    emxInit_real_T(&scores, 1);
    b_emxInit_real32_T(&points, 2);
    emxInit_real32_T(&x, 1);
    num_points = obj->NumPoints;
    num_points = rt_roundd_snf(num_points);
    if (num_points < 2.147483648E+9) {
      if (num_points >= -2.147483648E+9) {
        i = (int)num_points;
      } else {
        i = MIN_int32_T;
      }
    } else if (num_points >= 2.147483648E+9) {
      i = MAX_int32_T;
    } else {
      i = 0;
    }

    i13 = points->size[0] * points->size[1];
    points->size[0] = i;
    points->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)points, i13, (int)sizeof(float));
    i13 = trackStatus->size[0];
    trackStatus->size[0] = i;
    emxEnsureCapacity((emxArray__common *)trackStatus, i13, (int)sizeof
                      (boolean_T));
    i13 = scores->size[0];
    scores->size[0] = i;
    emxEnsureCapacity((emxArray__common *)scores, i13, (int)sizeof(double));
    pointTracker_step(ptrObj, Iu8_gray, 452, 735, &points->data[0],
                      &trackStatus->data[0], &scores->data[0]);
    loop_ub = points->size[0];
    i13 = x->size[0];
    x->size[0] = loop_ub;
    emxEnsureCapacity((emxArray__common *)x, i13, (int)sizeof(float));
    emxFree_real_T(&scores);
    for (i13 = 0; i13 < loop_ub; i13++) {
      x->data[i13] = points->data[i13];
    }

    emxInit_real32_T(&y, 1);
    loop_ub = points->size[0];
    i13 = y->size[0];
    y->size[0] = loop_ub;
    emxEnsureCapacity((emxArray__common *)y, i13, (int)sizeof(float));
    for (i13 = 0; i13 < loop_ub; i13++) {
      y->data[i13] = points->data[i13 + points->size[0]];
    }

    b_emxInit_boolean_T(&r7, 1);
    num_points = obj->FrameSize[1];
    i13 = r7->size[0];
    r7->size[0] = x->size[0];
    emxEnsureCapacity((emxArray__common *)r7, i13, (int)sizeof(boolean_T));
    loop_ub = x->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      r7->data[i13] = (x->data[i13] > num_points);
    }

    b_emxInit_boolean_T(&r8, 1);
    num_points = obj->FrameSize[0];
    i13 = r8->size[0];
    r8->size[0] = y->size[0];
    emxEnsureCapacity((emxArray__common *)r8, i13, (int)sizeof(boolean_T));
    loop_ub = y->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      r8->data[i13] = (y->data[i13] > num_points);
    }

    b_emxInit_boolean_T(&b_x, 1);
    i13 = b_x->size[0];
    b_x->size[0] = x->size[0];
    emxEnsureCapacity((emxArray__common *)b_x, i13, (int)sizeof(boolean_T));
    loop_ub = x->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      b_x->data[i13] = ((x->data[i13] < 1.0F) || (y->data[i13] < 1.0F) ||
                        r7->data[i13] || r8->data[i13]);
    }

    emxFree_boolean_T(&r8);
    emxFree_boolean_T(&r7);
    emxFree_real32_T(&y);
    emxFree_real32_T(&x);
    eml_li_find(b_x, r5);
    loop_ub = r5->size[0];
    emxFree_boolean_T(&b_x);
    for (i13 = 0; i13 < loop_ub; i13++) {
      trackStatus->data[r5->data[i13] - 1] = false;
    }

    b_emxInit_boolean_T(&b_points, 1);

    /*  deactivate points too close to the border */
    loop_ub = points->size[0];
    i13 = b_points->size[0];
    b_points->size[0] = loop_ub;
    emxEnsureCapacity((emxArray__common *)b_points, i13, (int)sizeof(boolean_T));
    for (i13 = 0; i13 < loop_ub; i13++) {
      b_points->data[i13] = ((points->data[i13] < border) || (points->data[i13] >
        735.0 - border) || (points->data[i13 + points->size[0]] < border) ||
        (points->data[i13 + points->size[0]] > 452.0 - border));
    }

    eml_li_find(b_points, r5);
    loop_ub = r5->size[0];
    emxFree_boolean_T(&b_points);
    for (i13 = 0; i13 < loop_ub; i13++) {
      trackStatus->data[r5->data[i13] - 1] = false;
    }

    b_emxInit_boolean_T(&b_trackStatus, 1);

    /*  save to pts_r_arr and updateVect */
    i13 = b_trackStatus->size[0];
    b_trackStatus->size[0] = trackStatus->size[0];
    emxEnsureCapacity((emxArray__common *)b_trackStatus, i13, (int)sizeof
                      (boolean_T));
    loop_ub = trackStatus->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      b_trackStatus->data[i13] = (trackStatus->data[i13] == 0);
    }

    emxInit_int32_T(&r9, 1);
    eml_li_find(b_trackStatus, r5);
    i13 = r9->size[0];
    r9->size[0] = r5->size[0];
    emxEnsureCapacity((emxArray__common *)r9, i13, (int)sizeof(int));
    loop_ub = r5->size[0];
    emxFree_boolean_T(&b_trackStatus);
    for (i13 = 0; i13 < loop_ub; i13++) {
      r9->data[i13] = active_idx_data[r5->data[i13] - 1];
    }

    loop_ub = r9->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      updateVect[r9->data[i13] - 1] = 0.0;
    }

    memset(&prevPoints[0], 0, sizeof(double) << 6);
    b_emxInit_boolean_T(&c_trackStatus, 1);
    i13 = c_trackStatus->size[0];
    c_trackStatus->size[0] = trackStatus->size[0];
    emxEnsureCapacity((emxArray__common *)c_trackStatus, i13, (int)sizeof
                      (boolean_T));
    loop_ub = trackStatus->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      c_trackStatus->data[i13] = ((int)trackStatus->data[i13] > 0);
    }

    eml_li_find(c_trackStatus, r5);
    i13 = r9->size[0];
    r9->size[0] = r5->size[0];
    emxEnsureCapacity((emxArray__common *)r9, i13, (int)sizeof(int));
    loop_ub = r5->size[0];
    emxFree_boolean_T(&c_trackStatus);
    for (i13 = 0; i13 < loop_ub; i13++) {
      r9->data[i13] = active_idx_data[r5->data[i13] - 1] - 1;
    }

    b_emxInit_boolean_T(&d_trackStatus, 1);
    i13 = d_trackStatus->size[0];
    d_trackStatus->size[0] = trackStatus->size[0];
    emxEnsureCapacity((emxArray__common *)d_trackStatus, i13, (int)sizeof
                      (boolean_T));
    loop_ub = trackStatus->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      d_trackStatus->data[i13] = ((int)trackStatus->data[i13] > 0);
    }

    emxFree_boolean_T(&trackStatus);
    b_emxInit_real_T(&r6, 2);
    eml_li_find(d_trackStatus, r5);
    i13 = r6->size[0] * r6->size[1];
    r6->size[0] = r5->size[0];
    r6->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)r6, i13, (int)sizeof(double));
    emxFree_boolean_T(&d_trackStatus);
    for (i13 = 0; i13 < 2; i13++) {
      loop_ub = r5->size[0];
      for (ii = 0; ii < loop_ub; ii++) {
        r6->data[ii + r6->size[0] * i13] = points->data[(r5->data[ii] +
          points->size[0] * i13) - 1];
      }
    }

    emxFree_real32_T(&points);
    i = r9->size[0];
    for (i13 = 0; i13 < 2; i13++) {
      for (ii = 0; ii < i; ii++) {
        prevPoints[r9->data[ii] + (i13 << 5)] = r6->data[ii + i * i13];
      }
    }

    emxFree_real_T(&r6);
    emxFree_int32_T(&r9);

    /*  fill feature vector */
    refillFeature(ImGrayR_r, prevPoints, updateVect, numAnchors, binx, biny,
                  border, minDistance);
    i = 0;
    ii = 1;
    exitg2 = false;
    while ((!exitg2) && (ii < 33)) {
      guard2 = false;
      if (updateVect[ii - 1] != 0.0) {
        i++;
        ii_data[i - 1] = ii;
        if (i >= 32) {
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

    if (1 > i) {
      loop_ub = 0;
    } else {
      loop_ub = i;
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      b_ii_data[i13] = (signed char)ii_data[i13];
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      ii_data[i13] = b_ii_data[i13];
    }

    for (i13 = 0; i13 < loop_ub; i13++) {
      active_idx_data[i13] = (signed char)ii_data[i13];
    }

    /*  indices of newly initialized features relative in active_idx */
    new_active_idx_size[0] = loop_ub;
    for (i13 = 0; i13 < loop_ub; i13++) {
      new_active_idx_data[i13] = (updateVect[active_idx_data[i13] - 1] == 2.0);
    }

    /*  use last disparity to limit search window (but do not use newly */
    /*  initialized ones) */
    maxDisp_size[0] = loop_ub;
    for (i13 = 0; i13 < loop_ub; i13++) {
      maxDisp_data[i13] = prevDisp[active_idx_data[i13] - 1] + 6.0;
    }

    minDisp_size[0] = loop_ub;
    for (i13 = 0; i13 < loop_ub; i13++) {
      minDisp_data[i13] = prevDisp[active_idx_data[i13] - 1] - 6.0;
    }

    c_new_active_idx_data.data = (boolean_T *)&new_active_idx_data;
    c_new_active_idx_data.size = (int *)&new_active_idx_size;
    c_new_active_idx_data.allocatedSize = 32;
    c_new_active_idx_data.numDimensions = 1;
    c_new_active_idx_data.canFreeData = false;
    eml_li_find(&c_new_active_idx_data, r5);
    ii_size[0] = r5->size[0];
    i = r5->size[0];
    for (i13 = 0; i13 < i; i13++) {
      ii_data[i13] = r5->data[i13];
    }

    i = ii_size[0];
    for (i13 = 0; i13 < i; i13++) {
      maxDisp_data[ii_data[i13] - 1] = 40.0;
    }

    e_new_active_idx_data.data = (boolean_T *)&new_active_idx_data;
    e_new_active_idx_data.size = (int *)&new_active_idx_size;
    e_new_active_idx_data.allocatedSize = 32;
    e_new_active_idx_data.numDimensions = 1;
    e_new_active_idx_data.canFreeData = false;
    eml_li_find(&e_new_active_idx_data, r5);
    ii_size[0] = r5->size[0];
    i = r5->size[0];
    for (i13 = 0; i13 < i; i13++) {
      ii_data[i13] = r5->data[i13];
    }

    i = ii_size[0];
    for (i13 = 0; i13 < i; i13++) {
      minDisp_data[ii_data[i13] - 1] = 0.0;
    }

    /*  find correspondences in the left image */
    memset(&b_pts_l_arr[0], 0, sizeof(double) << 6);
    tmp_size[0] = loop_ub;
    tmp_size[1] = 2;
    for (i13 = 0; i13 < 2; i13++) {
      for (ii = 0; ii < loop_ub; ii++) {
        dispVect[ii + loop_ub * i13] = prevPoints[(active_idx_data[ii] + (i13 <<
          5)) - 1];
      }
    }

    stereoRight2Left(ImGrayR_r, ImGrayR_l, dispVect, tmp_size, minDisp_data,
                     minDisp_size, maxDisp_data, maxDisp_size, points_data,
                     points_size, new_active_idx_data, new_active_idx_size);
    g_new_active_idx_size[0] = new_active_idx_size[0];
    loop_ub = new_active_idx_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      l_new_active_idx_data[i13] = ((int)new_active_idx_data[i13] > 0);
    }

    g_new_active_idx_data.data = (boolean_T *)&l_new_active_idx_data;
    g_new_active_idx_data.size = (int *)&g_new_active_idx_size;
    g_new_active_idx_data.allocatedSize = 32;
    g_new_active_idx_data.numDimensions = 1;
    g_new_active_idx_data.canFreeData = false;
    eml_li_find(&g_new_active_idx_data, r5);
    loop_ub = r5->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      ii_data[i13] = active_idx_data[r5->data[i13] - 1] - 1;
    }

    h_new_active_idx_size[0] = new_active_idx_size[0];
    loop_ub = new_active_idx_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      m_new_active_idx_data[i13] = ((int)new_active_idx_data[i13] > 0);
    }

    i_new_active_idx_data.data = (boolean_T *)&m_new_active_idx_data;
    i_new_active_idx_data.size = (int *)&h_new_active_idx_size;
    i_new_active_idx_data.allocatedSize = 32;
    i_new_active_idx_data.numDimensions = 1;
    i_new_active_idx_data.canFreeData = false;
    eml_li_find(&i_new_active_idx_data, r5);
    i = r5->size[0];
    for (i13 = 0; i13 < 2; i13++) {
      loop_ub = r5->size[0];
      for (ii = 0; ii < loop_ub; ii++) {
        tmp_data[ii + i * i13] = points_data[(r5->data[ii] + points_size[0] *
          i13) - 1];
      }
    }

    for (i13 = 0; i13 < 2; i13++) {
      for (ii = 0; ii < i; ii++) {
        b_pts_l_arr[ii_data[ii] + (i13 << 5)] = tmp_data[ii + i * i13];
      }
    }

    i_new_active_idx_size[0] = new_active_idx_size[0];
    loop_ub = new_active_idx_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      n_new_active_idx_data[i13] = ((int)new_active_idx_data[i13] <= 0);
    }

    k_new_active_idx_data.data = (boolean_T *)&n_new_active_idx_data;
    k_new_active_idx_data.size = (int *)&i_new_active_idx_size;
    k_new_active_idx_data.allocatedSize = 32;
    k_new_active_idx_data.numDimensions = 1;
    k_new_active_idx_data.canFreeData = false;
    eml_li_find(&k_new_active_idx_data, r5);
    ii_size[0] = r5->size[0];
    loop_ub = r5->size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      ii_data[i13] = active_idx_data[r5->data[i13] - 1];
    }

    loop_ub = ii_size[0];
    for (i13 = 0; i13 < loop_ub; i13++) {
      updateVect[ii_data[i13] - 1] = 0.0;
    }

    /*  reset features in right image if no correspondances are found in the */
    /*  left image */
    for (i = 0; i < 32; i++) {
      b_updateVect[i] = (updateVect[i] == 0.0);
    }

    b_eml_li_find(b_updateVect, ii_data, ii_size);
    for (i13 = 0; i13 < 2; i13++) {
      loop_ub = ii_size[0];
      for (ii = 0; ii < loop_ub; ii++) {
        prevPoints[(ii_data[ii] + (i13 << 5)) - 1] = 0.0;
      }
    }
  }

  emxFree_int32_T(&r5);

  /* % Disparity check and output */
  /*  compute the disparity Vector (should be only x component because images are rectified) */
  for (i13 = 0; i13 < 64; i13++) {
    dispVect[i13] = b_pts_l_arr[i13] - prevPoints[i13];
  }

  i = 0;
  ii = 1;
  exitg1 = false;
  while ((!exitg1) && (ii < 33)) {
    guard1 = false;
    if (updateVect[ii - 1] != 0.0) {
      i++;
      ii_data[i - 1] = ii;
      if (i >= 32) {
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

  if (1 > i) {
    loop_ub = 0;
  } else {
    loop_ub = i;
  }

  for (i13 = 0; i13 < loop_ub; i13++) {
    b_ii_data[i13] = (signed char)ii_data[i13];
  }

  for (i13 = 0; i13 < loop_ub; i13++) {
    ii_data[i13] = b_ii_data[i13];
  }

  for (i13 = 0; i13 < loop_ub; i13++) {
    active_idx_data[i13] = (signed char)ii_data[i13];
  }

  memset(&useDisparity[0], 0, sizeof(double) << 5);
  memset(&z_all[0], 0, 96U * sizeof(double));
  for (i = 0; i < loop_ub; i++) {
    /*  x coordinates in pixels */
    /*  y coordinates in pixels */
    z_all[3 * active_idx_data[i] - 3] = prevPoints[active_idx_data[i] - 1];

    /*  fill in the pixel measurements from the left? */
    z_all[3 * active_idx_data[i] - 2] = prevPoints[active_idx_data[i] + 31];

    /*  make a second outlier rejecton based on the dispairty (disparity */
    /*  should be disp=[d,0] because images are rectified */
    if ((dispVect[active_idx_data[i] - 1] > 1.7759) && (fabs
         (dispVect[active_idx_data[i] + 31]) < 5.0) &&
        (dispVect[active_idx_data[i] - 1] < 100.0)) {
      z_all[3 * active_idx_data[i] - 1] = dispVect[active_idx_data[i] - 1];

      /*  if valid use disparity */
      useDisparity[active_idx_data[i] - 1] = 1.0;

      /*  set use disparity vector to 1 */
    } else {
      z_all[3 * active_idx_data[i] - 1] = -100.0;

      /*  else use dummy value */
      useDisparity[active_idx_data[i] - 1] = 0.0;

      /*  set vect. to 0 */
    }
  }

  /*  save for next loop */
  for (i13 = 0; i13 < 32; i13++) {
    prevDisp[i13] = dispVect[i13];
    for (ii = 0; ii < 2; ii++) {
      pts_l_arr[ii + (i13 << 1)] = b_pts_l_arr[i13 + (ii << 5)];
      pts_r_arr[ii + (i13 << 1)] = prevPoints[i13 + (ii << 5)];
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void pointextraction_initialize(void)
{
  rt_InitInfAndNaN(8U);
  h2_not_empty = false;
  pointextraction_init();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void pointextraction_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for pointextraction.c
 *
 * [EOF]
 */
