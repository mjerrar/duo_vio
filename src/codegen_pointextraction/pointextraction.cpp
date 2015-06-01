//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pointextraction.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Jun-2015 13:49:37
//

// Include Files
#include "rt_nonfinite.h"
#include "pointextraction.h"
#include "cvstCG_detectFAST.h"
#include "cvstCG_pointTracker.h"
#include "libmwgrayto8.h"
#include "libmwmorphop_flat_tbb.h"

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

#endif                                 //struct_emxArray_int8_T

typedef struct {
  double offset[2];
  double count;
  emxArray_int8_T *grid;
} b_struct_T;

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

#endif                                 //struct_emxArray_b_struct_T

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
  double offset[2];
  double featureCandidates[64];
} struct_T;

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

#endif                                 //struct_emxArray_struct_T

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

#endif                                 //struct_emxArray_uint8_T

#ifndef struct_vision_ColorSpaceConverter_0
#define struct_vision_ColorSpaceConverter_0

struct vision_ColorSpaceConverter_0
{
  boolean_T SystemObjectStructType;
};

#endif                                 //struct_vision_ColorSpaceConverter_0

typedef struct {
  vision_ColorSpaceConverter_0 cSFunObject;
} c_visioncodegen_ColorSpaceConve;

typedef struct {
  int isInitialized;
  unsigned int inputVarSize1[8];
  void * pTracker;
  double FrameSize[2];
  double NumPoints;
  boolean_T IsRGB;
  char FrameClass[5];
  boolean_T IsInitialized;
  c_visioncodegen_ColorSpaceConve ColorConverter;
} vision_PointTracker_3;

// Variable Definitions
static boolean_T initialized_not_empty;
static vision_PointTracker_3 pointTracker;
static double prevPoints[64];
static double prevDisp[32];

// Function Declarations
static double FeaturePointsImpl_get_Count(const emxArray_real32_T
  *this_pLocation);
static vision_PointTracker_3 *PointTracker_PointTracker(vision_PointTracker_3
  *obj);
static void PointTracker_getKLTParams(vision_PointTracker_3 *obj, double
  kltParams_BlockSize[2], double *kltParams_NumPyramidLevels, double
  *kltParams_MaxIterations, double *kltParams_Epsilon, double
  *kltParams_MaxBidirectionalError);
static void PointTracker_initialize(vision_PointTracker_3 *obj, const double
  points_data[], const int points_size[2], const unsigned char I[332672]);
static void PointTracker_setPoints(vision_PointTracker_3 *obj, const double
  points_data[], const int points_size[2]);
static void SystemCore_step(vision_PointTracker_3 *obj, const unsigned char
  varargin_1[332672], emxArray_real32_T *varargout_1, emxArray_boolean_T
  *varargout_2);
static void all(const double x[64], boolean_T y[32]);
static boolean_T any(const boolean_T x[2]);
static void applyMinQuality(const emxArray_real32_T *points_Location, const
  emxArray_real32_T *points_Metric, double params_MinQuality, emxArray_real32_T *
  locations, emxArray_real32_T *metricValues);
static boolean_T b_any(const emxArray_boolean_T *x);
static void b_ceil(double x[2]);
static void b_eml_sort(emxArray_real32_T *x, int dim, emxArray_int32_T *idx);
static void b_eml_sort_idx(emxArray_real_T *x, emxArray_int32_T *idx);
static void b_emxFree_struct_T(emxArray_b_struct_T **pEmxArray);
static void b_emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
  numDimensions);
static void b_emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
static void b_emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void b_emxInit_struct_T(emxArray_b_struct_T **pEmxArray, int
  numDimensions);
static void b_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int
                    np, int nq);
static void b_merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel);
static void b_repmat(const double a[2], double varargin_1, emxArray_real_T *b);
static void b_round(double x_data[], int x_size[1]);
static void boundWindow(double b[4]);
static boolean_T c_any(const emxArray_int8_T *x);
static void c_eml_sort(emxArray_real_T *x, emxArray_int32_T *idx);
static void c_round(emxArray_real32_T *x);
static void cornerPoints_cg_cornerPoints_cg(const emxArray_real32_T *varargin_1,
  const emxArray_real32_T *varargin_3, emxArray_real32_T *this_pLocation,
  emxArray_real32_T *this_pMetric);
static void detectFASTFeatures(const emxArray_uint8_T *I, emxArray_real32_T
  *pts_pLocation, emxArray_real32_T *pts_pMetric);
static int div_s32(int numerator, int denominator);
static void eml_sort(emxArray_real32_T *x, emxArray_int32_T *idx);
static void eml_sort_idx(emxArray_real32_T *x, emxArray_int32_T *idx);
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
static void initFeatures(const unsigned char greyImg[332672], double
  borderLength, double numBinsX, double numBinsY, double minDistance, double
  features[64], double featStatus[32], double *threshold, double *flag);
static void kron(const emxArray_int8_T *A, const emxArray_int8_T *B,
                 emxArray_int8_T *K);
static void merge(emxArray_int32_T *idx, emxArray_real32_T *x, int offset, int
                  np, int nq);
static void merge_block(emxArray_int32_T *idx, emxArray_real32_T *x, int offset,
  int n, int preSortLevel);
static void rdivide(const double x[2], const double y[2], double z[2]);
static boolean_T refillFeature(const unsigned char greyImg[332672], double
  features[64], double featStatus[32], double numAnchors, double numBinsX,
  double numBinsY, double borderLength, double minDistance);
static void repmat(const b_struct_T a, double varargin_1, emxArray_b_struct_T *b);
static double rt_roundd_snf(double u);
static float rt_roundf_snf(float u);
static void stereoRight2Left(const unsigned char imgR[332672], const unsigned
  char imgL[332672], const double pointsR_data[], const int pointsR_size[2],
  double minDisp_data[], int minDisp_size[1], double maxDisp_data[], int
  maxDisp_size[1], double pointsL_data[], int pointsL_size[2], boolean_T
  status_data[], int status_size[1]);
static double sum(const signed char x_data[], const int x_size[1]);

// Function Definitions

//
// Arguments    : const emxArray_real32_T *this_pLocation
// Return Type  : double
//
static double FeaturePointsImpl_get_Count(const emxArray_real32_T
  *this_pLocation)
{
  return this_pLocation->size[0];
}

//
// Arguments    : vision_PointTracker_3 *obj
// Return Type  : vision_PointTracker_3 *
//
static vision_PointTracker_3 *PointTracker_PointTracker(vision_PointTracker_3
  *obj)
{
  vision_PointTracker_3 *b_obj;
  vision_PointTracker_3 *c_obj;
  void * ptrObj;
  b_obj = obj;
  b_obj->IsRGB = false;
  c_obj = b_obj;
  c_obj->isInitialized = 0;
  ptrObj = NULL;
  pointTracker_construct(&ptrObj);
  b_obj->pTracker = ptrObj;
  return b_obj;
}

//
// Arguments    : vision_PointTracker_3 *obj
//                double kltParams_BlockSize[2]
//                double *kltParams_NumPyramidLevels
//                double *kltParams_MaxIterations
//                double *kltParams_Epsilon
//                double *kltParams_MaxBidirectionalError
// Return Type  : void
//
static void PointTracker_getKLTParams(vision_PointTracker_3 *obj, double
  kltParams_BlockSize[2], double *kltParams_NumPyramidLevels, double
  *kltParams_MaxIterations, double *kltParams_Epsilon, double
  *kltParams_MaxBidirectionalError)
{
  int ixstart;
  vision_PointTracker_3 *b_obj;
  double varargin_1[2];
  double topOfPyramid;
  int ix;
  boolean_T exitg1;
  int eint;
  for (ixstart = 0; ixstart < 2; ixstart++) {
    kltParams_BlockSize[ixstart] = 21.0;
  }

  b_obj = obj;
  for (ixstart = 0; ixstart < 2; ixstart++) {
    varargin_1[ixstart] = b_obj->FrameSize[ixstart];
  }

  ixstart = 1;
  topOfPyramid = varargin_1[0];
  if (rtIsNaN(varargin_1[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 3)) {
      ixstart = 2;
      if (!rtIsNaN(varargin_1[1])) {
        topOfPyramid = varargin_1[1];
        exitg1 = true;
      } else {
        ix = 3;
      }
    }
  }

  if ((ixstart < 2) && (varargin_1[1] < topOfPyramid)) {
    topOfPyramid = varargin_1[1];
  }

  if (topOfPyramid == 0.0) {
    topOfPyramid = rtMinusInf;
  } else if (topOfPyramid < 0.0) {
    topOfPyramid = rtNaN;
  } else {
    if ((!rtIsInf(topOfPyramid)) && (!rtIsNaN(topOfPyramid))) {
      if ((!rtIsInf(topOfPyramid)) && (!rtIsNaN(topOfPyramid))) {
        topOfPyramid = frexp(topOfPyramid, &eint);
      } else {
        eint = 0;
      }

      if (topOfPyramid == 0.5) {
        topOfPyramid = (double)eint - 1.0;
      } else {
        topOfPyramid = log(topOfPyramid) / 0.69314718055994529 + (double)eint;
      }
    }
  }

  topOfPyramid = floor(topOfPyramid - 2.0);
  if (topOfPyramid <= 4.0) {
  } else {
    topOfPyramid = 4.0;
  }

  if (0.0 >= topOfPyramid) {
    *kltParams_NumPyramidLevels = 0.0;
  } else {
    *kltParams_NumPyramidLevels = (int)topOfPyramid;
  }

  *kltParams_MaxIterations = 30.0;
  *kltParams_Epsilon = 0.01;
  *kltParams_MaxBidirectionalError = -1.0;
}

//
// Arguments    : vision_PointTracker_3 *obj
//                const double points_data[]
//                const int points_size[2]
//                const unsigned char I[332672]
// Return Type  : void
//
static void PointTracker_initialize(vision_PointTracker_3 *obj, const double
  points_data[], const int points_size[2], const unsigned char I[332672])
{
  vision_PointTracker_3 *b_obj;
  int blockW;
  static const short iv0[8] = { 452, 736, 1, 1, 1, 1, 1, 1 };

  int blockH;
  float b_points_data[64];
  static const char cv0[5] = { 'u', 'i', 'n', 't', '8' };

  void * ptrObj;
  double expl_temp;
  double b_expl_temp;
  double c_expl_temp;
  double params_NumPyramidLevels;
  double d_expl_temp[2];
  static unsigned char Iu8_gray[332672];
  float c_points_data[64];
  cvstPTStruct_T paramStruct;
  b_obj = obj;
  b_obj->isInitialized = 1;
  for (blockW = 0; blockW < 8; blockW++) {
    b_obj->inputVarSize1[blockW] = (unsigned int)iv0[blockW];
  }

  blockH = points_size[0] * points_size[1];
  for (blockW = 0; blockW < blockH; blockW++) {
    b_points_data[blockW] = (float)points_data[blockW];
  }

  for (blockW = 0; blockW < 5; blockW++) {
    obj->FrameClass[blockW] = cv0[blockW];
  }

  for (blockW = 0; blockW < 2; blockW++) {
    obj->FrameSize[blockW] = 452.0 + 284.0 * (double)blockW;
  }

  obj->NumPoints = points_size[0];
  ptrObj = obj->pTracker;
  PointTracker_getKLTParams(obj, d_expl_temp, &params_NumPyramidLevels,
    &c_expl_temp, &b_expl_temp, &expl_temp);
  for (blockW = 0; blockW < 452; blockW++) {
    for (blockH = 0; blockH < 736; blockH++) {
      Iu8_gray[blockH + 736 * blockW] = I[blockW + 452 * blockH];
    }
  }

  blockH = points_size[0] << 1;
  for (blockW = 0; blockW < blockH; blockW++) {
    c_points_data[blockW] = b_points_data[blockW];
  }

  blockH = (int32_T)(21.0);
  blockW = (int32_T)(21.0);
  paramStruct.blockSize[0] = blockH;
  paramStruct.blockSize[1] = blockW;
  blockH = (int32_T)(params_NumPyramidLevels);
  paramStruct.numPyramidLevels = blockH;
  params_NumPyramidLevels = (double)(30.0);
  paramStruct.maxIterations = params_NumPyramidLevels;
  paramStruct.epsilon = 0.01;
  paramStruct.maxBidirectionalError = -1.0;
  pointTracker_initialize(ptrObj, Iu8_gray, 452, 736, &c_points_data[0],
    points_size[0], &paramStruct);
  obj->IsInitialized = true;
}

//
// Arguments    : vision_PointTracker_3 *obj
//                const double points_data[]
//                const int points_size[2]
// Return Type  : void
//
static void PointTracker_setPoints(vision_PointTracker_3 *obj, const double
  points_data[], const int points_size[2])
{
  int loop_ub;
  int i12;
  float b_points_data[64];
  void * ptrObj;
  float c_points_data[64];
  boolean_T pointValidity_data[32];
  loop_ub = points_size[0] * points_size[1];
  for (i12 = 0; i12 < loop_ub; i12++) {
    b_points_data[i12] = (float)points_data[i12];
  }

  obj->NumPoints = points_size[0];
  ptrObj = obj->pTracker;
  loop_ub = points_size[0] << 1;
  for (i12 = 0; i12 < loop_ub; i12++) {
    c_points_data[i12] = b_points_data[i12];
  }

  loop_ub = points_size[0];
  for (i12 = 0; i12 < loop_ub; i12++) {
    pointValidity_data[i12] = true;
  }

  pointTracker_setPoints(ptrObj, &c_points_data[0], points_size[0],
    &pointValidity_data[0]);
}

//
// Arguments    : vision_PointTracker_3 *obj
//                const unsigned char varargin_1[332672]
//                emxArray_real32_T *varargout_1
//                emxArray_boolean_T *varargout_2
// Return Type  : void
//
static void SystemCore_step(vision_PointTracker_3 *obj, const unsigned char
  varargin_1[332672], emxArray_real32_T *varargout_1, emxArray_boolean_T
  *varargout_2)
{
  vision_PointTracker_3 *b_obj;
  int i;
  static const short inputSize[8] = { 452, 736, 1, 1, 1, 1, 1, 1 };

  int numPoints;
  boolean_T exitg1;
  static const short iv1[8] = { 452, 736, 1, 1, 1, 1, 1, 1 };

  void * ptrObj;
  static unsigned char Iu8_gray[332672];
  emxArray_real32_T *points;
  emxArray_real32_T *x;
  emxArray_real_T *scores;
  double num_points;
  emxArray_real32_T *y;
  emxArray_boolean_T *badPoints;
  emxArray_boolean_T *r4;
  if (obj->isInitialized != 1) {
    b_obj = obj;
    b_obj->isInitialized = 1;
    for (i = 0; i < 8; i++) {
      b_obj->inputVarSize1[i] = (unsigned int)inputSize[i];
    }
  }

  b_obj = obj;
  numPoints = 0;
  exitg1 = false;
  while ((!exitg1) && (numPoints < 8)) {
    if (b_obj->inputVarSize1[numPoints] != (unsigned int)iv1[numPoints]) {
      for (i = 0; i < 8; i++) {
        b_obj->inputVarSize1[i] = (unsigned int)inputSize[i];
      }

      exitg1 = true;
    } else {
      numPoints++;
    }
  }

  b_obj = obj;
  ptrObj = b_obj->pTracker;
  for (i = 0; i < 452; i++) {
    for (numPoints = 0; numPoints < 736; numPoints++) {
      Iu8_gray[numPoints + 736 * i] = varargin_1[i + 452 * numPoints];
    }
  }

  emxInit_real32_T(&points, 2);
  b_emxInit_real32_T(&x, 1);
  emxInit_real_T(&scores, 1);
  num_points = b_obj->NumPoints;
  num_points = rt_roundd_snf(num_points);
  if (num_points < 2.147483648E+9) {
    if (num_points >= -2.147483648E+9) {
      numPoints = (int)num_points;
    } else {
      numPoints = MIN_int32_T;
    }
  } else if (num_points >= 2.147483648E+9) {
    numPoints = MAX_int32_T;
  } else {
    numPoints = 0;
  }

  i = points->size[0] * points->size[1];
  points->size[0] = numPoints;
  points->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)points, i, (int)sizeof(float));
  i = varargout_2->size[0];
  varargout_2->size[0] = numPoints;
  emxEnsureCapacity((emxArray__common *)varargout_2, i, (int)sizeof(boolean_T));
  i = scores->size[0];
  scores->size[0] = numPoints;
  emxEnsureCapacity((emxArray__common *)scores, i, (int)sizeof(double));
  pointTracker_step(ptrObj, Iu8_gray, 452, 736, &points->data[0],
                    &varargout_2->data[0], &scores->data[0]);
  numPoints = points->size[0];
  i = x->size[0];
  x->size[0] = numPoints;
  emxEnsureCapacity((emxArray__common *)x, i, (int)sizeof(float));
  emxFree_real_T(&scores);
  for (i = 0; i < numPoints; i++) {
    x->data[i] = points->data[i];
  }

  b_emxInit_real32_T(&y, 1);
  numPoints = points->size[0];
  i = y->size[0];
  y->size[0] = numPoints;
  emxEnsureCapacity((emxArray__common *)y, i, (int)sizeof(float));
  for (i = 0; i < numPoints; i++) {
    y->data[i] = points->data[i + points->size[0]];
  }

  b_emxInit_boolean_T(&badPoints, 1);
  num_points = b_obj->FrameSize[1];
  i = badPoints->size[0];
  badPoints->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)badPoints, i, (int)sizeof(boolean_T));
  numPoints = x->size[0];
  for (i = 0; i < numPoints; i++) {
    badPoints->data[i] = (x->data[i] > num_points);
  }

  b_emxInit_boolean_T(&r4, 1);
  num_points = b_obj->FrameSize[0];
  i = r4->size[0];
  r4->size[0] = y->size[0];
  emxEnsureCapacity((emxArray__common *)r4, i, (int)sizeof(boolean_T));
  numPoints = y->size[0];
  for (i = 0; i < numPoints; i++) {
    r4->data[i] = (y->data[i] > num_points);
  }

  i = badPoints->size[0];
  badPoints->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)badPoints, i, (int)sizeof(boolean_T));
  numPoints = x->size[0];
  for (i = 0; i < numPoints; i++) {
    badPoints->data[i] = ((x->data[i] < 1.0F) || (y->data[i] < 1.0F) ||
                          badPoints->data[i] || r4->data[i]);
  }

  emxFree_boolean_T(&r4);
  emxFree_real32_T(&y);
  emxFree_real32_T(&x);
  numPoints = badPoints->size[0];
  for (i = 0; i < numPoints; i++) {
    if (badPoints->data[i]) {
      varargout_2->data[i] = false;
    }
  }

  emxFree_boolean_T(&badPoints);
  i = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = points->size[0];
  varargout_1->size[1] = points->size[1];
  emxEnsureCapacity((emxArray__common *)varargout_1, i, (int)sizeof(float));
  numPoints = points->size[0] * points->size[1];
  for (i = 0; i < numPoints; i++) {
    varargout_1->data[i] = points->data[i];
  }

  emxFree_real32_T(&points);
}

//
// Arguments    : const double x[64]
//                boolean_T y[32]
// Return Type  : void
//
static void all(const double x[64], boolean_T y[32])
{
  int iy;
  int i1;
  int i2;
  int j;
  int ix;
  boolean_T exitg1;
  for (iy = 0; iy < 32; iy++) {
    y[iy] = true;
  }

  iy = -1;
  i1 = 0;
  i2 = 32;
  for (j = 0; j < 32; j++) {
    i1++;
    i2++;
    iy++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2)) {
      if (x[ix - 1] == 0.0) {
        y[iy] = false;
        exitg1 = true;
      } else {
        ix += 32;
      }
    }
  }
}

//
// Arguments    : const boolean_T x[2]
// Return Type  : boolean_T
//
static boolean_T any(const boolean_T x[2])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
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

  return y;
}

//
// Arguments    : const emxArray_real32_T *points_Location
//                const emxArray_real32_T *points_Metric
//                double params_MinQuality
//                emxArray_real32_T *locations
//                emxArray_real32_T *metricValues
// Return Type  : void
//
static void applyMinQuality(const emxArray_real32_T *points_Location, const
  emxArray_real32_T *points_Metric, double params_MinQuality, emxArray_real32_T *
  locations, emxArray_real32_T *metricValues)
{
  int ixstart;
  int n;
  float threshold;
  int ix;
  boolean_T exitg1;
  emxArray_boolean_T *validIndex;
  emxArray_int32_T *r1;
  int end;
  if (!(points_Metric->size[0] == 0)) {
    ixstart = 1;
    n = points_Metric->size[0];
    threshold = points_Metric->data[0];
    if (points_Metric->size[0] > 1) {
      if (rtIsNaNF(points_Metric->data[0])) {
        ix = 2;
        exitg1 = false;
        while ((!exitg1) && (ix <= n)) {
          ixstart = ix;
          if (!rtIsNaNF(points_Metric->data[ix - 1])) {
            threshold = points_Metric->data[ix - 1];
            exitg1 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < points_Metric->size[0]) {
        while (ixstart + 1 <= n) {
          if (points_Metric->data[ixstart] > threshold) {
            threshold = points_Metric->data[ixstart];
          }

          ixstart++;
        }
      }
    }

    b_emxInit_boolean_T(&validIndex, 1);
    threshold *= (float)params_MinQuality;
    ix = validIndex->size[0];
    validIndex->size[0] = points_Metric->size[0];
    emxEnsureCapacity((emxArray__common *)validIndex, ix, (int)sizeof(boolean_T));
    ixstart = points_Metric->size[0];
    for (ix = 0; ix < ixstart; ix++) {
      validIndex->data[ix] = (points_Metric->data[ix] >= threshold);
    }

    emxInit_int32_T(&r1, 1);
    end = validIndex->size[0] - 1;
    ixstart = 0;
    for (n = 0; n <= end; n++) {
      if (validIndex->data[n]) {
        ixstart++;
      }
    }

    ix = r1->size[0];
    r1->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)r1, ix, (int)sizeof(int));
    ixstart = 0;
    for (n = 0; n <= end; n++) {
      if (validIndex->data[n]) {
        r1->data[ixstart] = n + 1;
        ixstart++;
      }
    }

    ix = locations->size[0] * locations->size[1];
    locations->size[0] = r1->size[0];
    locations->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)locations, ix, (int)sizeof(float));
    for (ix = 0; ix < 2; ix++) {
      ixstart = r1->size[0];
      for (n = 0; n < ixstart; n++) {
        locations->data[n + locations->size[0] * ix] = points_Location->data
          [(r1->data[n] + points_Location->size[0] * ix) - 1];
      }
    }

    end = validIndex->size[0] - 1;
    ixstart = 0;
    for (n = 0; n <= end; n++) {
      if (validIndex->data[n]) {
        ixstart++;
      }
    }

    ix = r1->size[0];
    r1->size[0] = ixstart;
    emxEnsureCapacity((emxArray__common *)r1, ix, (int)sizeof(int));
    ixstart = 0;
    for (n = 0; n <= end; n++) {
      if (validIndex->data[n]) {
        r1->data[ixstart] = n + 1;
        ixstart++;
      }
    }

    emxFree_boolean_T(&validIndex);
    ix = metricValues->size[0];
    metricValues->size[0] = r1->size[0];
    emxEnsureCapacity((emxArray__common *)metricValues, ix, (int)sizeof(float));
    ixstart = r1->size[0];
    for (ix = 0; ix < ixstart; ix++) {
      metricValues->data[ix] = points_Metric->data[r1->data[ix] - 1];
    }

    emxFree_int32_T(&r1);
  } else {
    ix = locations->size[0] * locations->size[1];
    locations->size[0] = 0;
    locations->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)locations, ix, (int)sizeof(float));
    ix = metricValues->size[0];
    metricValues->size[0] = 0;
    emxEnsureCapacity((emxArray__common *)metricValues, ix, (int)sizeof(float));
  }
}

//
// Arguments    : const emxArray_boolean_T *x
// Return Type  : boolean_T
//
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
    b0 = !x->data[ix - 1];
    if (!b0) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

//
// Arguments    : double x[2]
// Return Type  : void
//
static void b_ceil(double x[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    x[k] = ceil(x[k]);
  }
}

//
// Arguments    : emxArray_real32_T *x
//                int dim
//                emxArray_int32_T *idx
// Return Type  : void
//
static void b_eml_sort(emxArray_real32_T *x, int dim, emxArray_int32_T *idx)
{
  int i18;
  emxArray_real32_T *vwork;
  int vstride;
  int unnamed_idx_0;
  int j;
  emxArray_int32_T *iidx;
  if (dim <= 1) {
    i18 = x->size[0];
  } else {
    i18 = 1;
  }

  b_emxInit_real32_T(&vwork, 1);
  vstride = vwork->size[0];
  vwork->size[0] = i18;
  emxEnsureCapacity((emxArray__common *)vwork, vstride, (int)sizeof(float));
  unnamed_idx_0 = x->size[0];
  vstride = idx->size[0];
  idx->size[0] = unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)idx, vstride, (int)sizeof(int));
  vstride = 1;
  unnamed_idx_0 = 1;
  while (unnamed_idx_0 <= dim - 1) {
    vstride *= x->size[0];
    unnamed_idx_0 = 2;
  }

  j = 0;
  emxInit_int32_T(&iidx, 1);
  while (j + 1 <= vstride) {
    for (unnamed_idx_0 = 0; unnamed_idx_0 + 1 <= i18; unnamed_idx_0++) {
      vwork->data[unnamed_idx_0] = x->data[j + unnamed_idx_0 * vstride];
    }

    eml_sort_idx(vwork, iidx);
    for (unnamed_idx_0 = 0; unnamed_idx_0 + 1 <= i18; unnamed_idx_0++) {
      x->data[j + unnamed_idx_0 * vstride] = vwork->data[unnamed_idx_0];
      idx->data[j + unnamed_idx_0 * vstride] = iidx->data[unnamed_idx_0];
    }

    j++;
  }

  emxFree_int32_T(&iidx);
  emxFree_real32_T(&vwork);
}

//
// Arguments    : emxArray_real_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void b_eml_sort_idx(emxArray_real_T *x, emxArray_int32_T *idx)
{
  emxArray_real_T *b_x;
  int ib;
  int p;
  int m;
  int n;
  double x4[4];
  int idx4[4];
  emxArray_real_T *xwork;
  int nNaNs;
  int k;
  int wOffset;
  int i4;
  signed char perm[4];
  int nNonNaN;
  int nBlocks;
  int iwork[256];
  double b_xwork[256];
  int bLen2;
  int nPairs;
  int32_T exitg1;
  emxInit_real_T(&b_x, 1);
  ib = x->size[0];
  p = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, p, (int)sizeof(double));
  m = x->size[0];
  for (p = 0; p < m; p++) {
    b_x->data[p] = x->data[p];
  }

  p = idx->size[0];
  idx->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)idx, p, (int)sizeof(int));
  for (p = 0; p < ib; p++) {
    idx->data[p] = 0;
  }

  n = x->size[0];
  for (m = 0; m < 4; m++) {
    x4[m] = 0.0;
    idx4[m] = 0;
  }

  emxInit_real_T(&xwork, 1);
  ib = x->size[0];
  p = xwork->size[0];
  xwork->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)xwork, p, (int)sizeof(double));
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
          m = 1;
          p = 2;
        } else {
          m = 2;
          p = 1;
        }

        if (x4[2] <= x4[3]) {
          wOffset = 3;
          i4 = 4;
        } else {
          wOffset = 4;
          i4 = 3;
        }

        if (x4[m - 1] <= x4[wOffset - 1]) {
          if (x4[p - 1] <= x4[wOffset - 1]) {
            perm[0] = (signed char)m;
            perm[1] = (signed char)p;
            perm[2] = (signed char)wOffset;
            perm[3] = (signed char)i4;
          } else if (x4[p - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)m;
            perm[1] = (signed char)wOffset;
            perm[2] = (signed char)p;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)m;
            perm[1] = (signed char)wOffset;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)p;
          }
        } else if (x4[m - 1] <= x4[i4 - 1]) {
          if (x4[p - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)wOffset;
            perm[1] = (signed char)m;
            perm[2] = (signed char)p;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)wOffset;
            perm[1] = (signed char)m;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)p;
          }
        } else {
          perm[0] = (signed char)wOffset;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)m;
          perm[3] = (signed char)p;
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
      idx->data[(wOffset - ib) + k] = idx4[perm[k - 1] - 1];
      b_x->data[(wOffset - ib) + k] = x4[perm[k - 1] - 1];
    }
  }

  m = (nNaNs - 1) >> 1;
  for (k = 1; k <= m; k++) {
    p = idx->data[wOffset + k];
    idx->data[wOffset + k] = idx->data[n - k];
    idx->data[n - k] = p;
    b_x->data[wOffset + k] = xwork->data[n - k];
    b_x->data[n - k] = xwork->data[wOffset + k];
  }

  if (((nNaNs - 1) & 1) != 0) {
    b_x->data[(wOffset + m) + 1] = xwork->data[(wOffset + m) + 1];
  }

  emxFree_real_T(&xwork);
  nNonNaN = (x->size[0] - nNaNs) + 1;
  m = 2;
  if (nNonNaN > 1) {
    if (x->size[0] >= 256) {
      nBlocks = nNonNaN >> 8;
      if (nBlocks > 0) {
        for (wOffset = 1; wOffset <= nBlocks; wOffset++) {
          i4 = ((wOffset - 1) << 8) - 1;
          for (nNaNs = 0; nNaNs < 6; nNaNs++) {
            n = 1 << (nNaNs + 2);
            bLen2 = n << 1;
            nPairs = 256 >> (nNaNs + 3);
            for (k = 1; k <= nPairs; k++) {
              m = i4 + (k - 1) * bLen2;
              for (ib = 1; ib <= bLen2; ib++) {
                iwork[ib - 1] = idx->data[m + ib];
                b_xwork[ib - 1] = b_x->data[m + ib];
              }

              p = 0;
              ib = n;
              do {
                exitg1 = 0;
                m++;
                if (b_xwork[p] <= b_xwork[ib]) {
                  idx->data[m] = iwork[p];
                  b_x->data[m] = b_xwork[p];
                  if (p + 1 < n) {
                    p++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  idx->data[m] = iwork[ib];
                  b_x->data[m] = b_xwork[ib];
                  if (ib + 1 < bLen2) {
                    ib++;
                  } else {
                    ib = m - p;
                    while (p + 1 <= n) {
                      idx->data[(ib + p) + 1] = iwork[p];
                      b_x->data[(ib + p) + 1] = b_xwork[p];
                      p++;
                    }

                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
        }

        m = nBlocks << 8;
        ib = nNonNaN - m;
        if (ib > 0) {
          b_merge_block(idx, b_x, m, ib, 2);
        }

        m = 8;
      }
    }

    b_merge_block(idx, b_x, 0, nNonNaN, m);
  }

  p = x->size[0];
  x->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)x, p, (int)sizeof(double));
  m = b_x->size[0];
  for (p = 0; p < m; p++) {
    x->data[p] = b_x->data[p];
  }

  emxFree_real_T(&b_x);
}

//
// Arguments    : emxArray_b_struct_T **pEmxArray
// Return Type  : void
//
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
// Arguments    : emxArray_int8_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
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

//
// Arguments    : emxArray_real32_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
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
// Arguments    : emxArray_b_struct_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
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

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real_T *x
//                int offset
//                int np
//                int nq
// Return Type  : void
//
static void b_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int
                    np, int nq)
{
  emxArray_int32_T *iwork;
  emxArray_real_T *xwork;
  int n;
  int qend;
  int p;
  int iout;
  int32_T exitg1;
  emxInit_int32_T(&iwork, 1);
  emxInit_real_T(&xwork, 1);
  n = iwork->size[0];
  iwork->size[0] = idx->size[0];
  emxEnsureCapacity((emxArray__common *)iwork, n, (int)sizeof(int));
  qend = x->size[0];
  n = xwork->size[0];
  xwork->size[0] = qend;
  emxEnsureCapacity((emxArray__common *)xwork, n, (int)sizeof(double));
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

  emxFree_real_T(&xwork);
  emxFree_int32_T(&iwork);
}

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real_T *x
//                int offset
//                int n
//                int preSortLevel
// Return Type  : void
//
static void b_merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel)
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
        b_merge(idx, x, offset + tailOffset, bLen, nTail - bLen);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 1; nTail <= nPairs; nTail++) {
      b_merge(idx, x, offset + (nTail - 1) * tailOffset, bLen, bLen);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    b_merge(idx, x, offset, bLen, n - bLen);
  }
}

//
// Arguments    : const double a[2]
//                double varargin_1
//                emxArray_real_T *b
// Return Type  : void
//
static void b_repmat(const double a[2], double varargin_1, emxArray_real_T *b)
{
  int jcol;
  int ibmat;
  int itilerow;
  jcol = b->size[0] * b->size[1];
  b->size[0] = (int)varargin_1;
  b->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)b, jcol, (int)sizeof(double));
  if (!((int)varargin_1 == 0)) {
    for (jcol = 0; jcol < 2; jcol++) {
      ibmat = jcol * (int)varargin_1;
      for (itilerow = 1; itilerow <= (int)varargin_1; itilerow++) {
        b->data[(ibmat + itilerow) - 1] = a[jcol];
      }
    }
  }
}

//
// Arguments    : double x_data[]
//                int x_size[1]
// Return Type  : void
//
static void b_round(double x_data[], int x_size[1])
{
  int i20;
  int k;
  i20 = x_size[0];
  for (k = 0; k < i20; k++) {
    x_data[k] = rt_roundd_snf(x_data[k]);
  }
}

//
// Arguments    : double b[4]
// Return Type  : void
//
static void boundWindow(double b[4])
{
  if (b[0] <= 0.0) {
    // top
    b[0] = 1.0;
  }

  if (b[1] > 452.0) {
    // bottom
    b[1] = 452.0;
  }

  if (b[2] <= 0.0) {
    // left
    b[2] = 1.0;
  }

  if (b[3] > 736.0) {
    // right
    b[3] = 736.0;
  }
}

//
// Arguments    : const emxArray_int8_T *x
// Return Type  : boolean_T
//
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

//
// Arguments    : emxArray_real_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void c_eml_sort(emxArray_real_T *x, emxArray_int32_T *idx)
{
  int dim;
  int i19;
  emxArray_real_T *vwork;
  int j;
  int vstride;
  int k;
  emxArray_int32_T *iidx;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  if (dim <= 1) {
    i19 = x->size[0];
  } else {
    i19 = 1;
  }

  emxInit_real_T(&vwork, 1);
  j = vwork->size[0];
  vwork->size[0] = i19;
  emxEnsureCapacity((emxArray__common *)vwork, j, (int)sizeof(double));
  vstride = x->size[0];
  j = idx->size[0];
  idx->size[0] = vstride;
  emxEnsureCapacity((emxArray__common *)idx, j, (int)sizeof(int));
  vstride = 1;
  k = 1;
  while (k <= dim - 1) {
    vstride *= x->size[0];
    k = 2;
  }

  j = 0;
  emxInit_int32_T(&iidx, 1);
  while (j + 1 <= vstride) {
    for (k = 0; k + 1 <= i19; k++) {
      vwork->data[k] = x->data[j + k * vstride];
    }

    b_eml_sort_idx(vwork, iidx);
    for (k = 0; k + 1 <= i19; k++) {
      x->data[j + k * vstride] = vwork->data[k];
      idx->data[j + k * vstride] = iidx->data[k];
    }

    j++;
  }

  emxFree_int32_T(&iidx);
  emxFree_real_T(&vwork);
}

//
// Arguments    : emxArray_real32_T *x
// Return Type  : void
//
static void c_round(emxArray_real32_T *x)
{
  int i24;
  int k;
  i24 = x->size[0] << 1;
  for (k = 0; k < i24; k++) {
    x->data[k] = rt_roundf_snf(x->data[k]);
  }
}

//
// Arguments    : const emxArray_real32_T *varargin_1
//                const emxArray_real32_T *varargin_3
//                emxArray_real32_T *this_pLocation
//                emxArray_real32_T *this_pMetric
// Return Type  : void
//
static void cornerPoints_cg_cornerPoints_cg(const emxArray_real32_T *varargin_1,
  const emxArray_real32_T *varargin_3, emxArray_real32_T *this_pLocation,
  emxArray_real32_T *this_pMetric)
{
  int i5;
  int itilerow;
  emxArray_real32_T *r2;
  int varargin_1_idx_0;
  int32_T exitg1;
  i5 = this_pLocation->size[0] * this_pLocation->size[1];
  this_pLocation->size[0] = varargin_1->size[0];
  this_pLocation->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)this_pLocation, i5, (int)sizeof(float));
  itilerow = varargin_1->size[0] * varargin_1->size[1];
  for (i5 = 0; i5 < itilerow; i5++) {
    this_pLocation->data[i5] = varargin_1->data[i5];
  }

  b_emxInit_real32_T(&r2, 1);
  if (varargin_3->size[0] == 1) {
    varargin_1_idx_0 = varargin_1->size[0];
    itilerow = varargin_1->size[0];
    i5 = r2->size[0];
    r2->size[0] = itilerow;
    emxEnsureCapacity((emxArray__common *)r2, i5, (int)sizeof(float));
    if (!(varargin_1_idx_0 == 0)) {
      itilerow = 1;
      do {
        exitg1 = 0;
        varargin_1_idx_0 = varargin_1->size[0];
        if (itilerow <= varargin_1_idx_0) {
          r2->data[itilerow - 1] = varargin_3->data[0];
          itilerow++;
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
  } else {
    i5 = r2->size[0];
    r2->size[0] = varargin_3->size[0];
    emxEnsureCapacity((emxArray__common *)r2, i5, (int)sizeof(float));
    itilerow = varargin_3->size[0];
    for (i5 = 0; i5 < itilerow; i5++) {
      r2->data[i5] = varargin_3->data[i5];
    }
  }

  i5 = this_pMetric->size[0];
  this_pMetric->size[0] = r2->size[0];
  emxEnsureCapacity((emxArray__common *)this_pMetric, i5, (int)sizeof(float));
  itilerow = r2->size[0];
  for (i5 = 0; i5 < itilerow; i5++) {
    this_pMetric->data[i5] = r2->data[i5];
  }

  emxFree_real32_T(&r2);
}

//
// Arguments    : const emxArray_uint8_T *I
//                emxArray_real32_T *pts_pLocation
//                emxArray_real32_T *pts_pMetric
// Return Type  : void
//
static void detectFASTFeatures(const emxArray_uint8_T *I, emxArray_real32_T
  *pts_pLocation, emxArray_real32_T *pts_pMetric)
{
  emxArray_uint8_T *Iu8;
  unsigned char minContrast;
  double IM1;
  int i17;
  int out_numel;
  emxArray_real32_T *rawPts_Location;
  emxArray_real32_T *rawPts_metric;
  emxArray_real32_T *rawPts_loc;
  void * ptrKeypoints;
  emxArray_real32_T *rawPts_Metric;
  emxArray_real32_T *locations;
  emxInit_uint8_T(&Iu8, 2);
  IM1 = 0.2;
  grayto8_real64(&IM1, &minContrast, 1.0);
  i17 = Iu8->size[0] * Iu8->size[1];
  Iu8->size[0] = I->size[0];
  Iu8->size[1] = I->size[1];
  emxEnsureCapacity((emxArray__common *)Iu8, i17, (int)sizeof(unsigned char));
  out_numel = I->size[0] * I->size[1];
  for (i17 = 0; i17 < out_numel; i17++) {
    Iu8->data[i17] = I->data[i17];
  }

  emxInit_real32_T(&rawPts_Location, 2);
  b_emxInit_real32_T(&rawPts_metric, 1);
  emxInit_real32_T(&rawPts_loc, 2);
  ptrKeypoints = NULL;
  out_numel = detectFAST_compute(&Iu8->data[0], I->size[0], I->size[1], false,
    minContrast, &ptrKeypoints);
  i17 = rawPts_loc->size[0] * rawPts_loc->size[1];
  rawPts_loc->size[0] = out_numel;
  rawPts_loc->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)rawPts_loc, i17, (int)sizeof(float));
  i17 = rawPts_metric->size[0];
  rawPts_metric->size[0] = out_numel;
  emxEnsureCapacity((emxArray__common *)rawPts_metric, i17, (int)sizeof(float));
  detectFAST_assignOutput(ptrKeypoints, &rawPts_loc->data[0],
    &rawPts_metric->data[0]);
  i17 = rawPts_Location->size[0] * rawPts_Location->size[1];
  rawPts_Location->size[0] = rawPts_loc->size[0];
  rawPts_Location->size[1] = rawPts_loc->size[1];
  emxEnsureCapacity((emxArray__common *)rawPts_Location, i17, (int)sizeof(float));
  out_numel = rawPts_loc->size[0] * rawPts_loc->size[1];
  emxFree_uint8_T(&Iu8);
  for (i17 = 0; i17 < out_numel; i17++) {
    rawPts_Location->data[i17] = rawPts_loc->data[i17];
  }

  emxFree_real32_T(&rawPts_loc);
  b_emxInit_real32_T(&rawPts_Metric, 1);
  i17 = rawPts_Metric->size[0];
  rawPts_Metric->size[0] = rawPts_metric->size[0];
  emxEnsureCapacity((emxArray__common *)rawPts_Metric, i17, (int)sizeof(float));
  out_numel = rawPts_metric->size[0];
  for (i17 = 0; i17 < out_numel; i17++) {
    rawPts_Metric->data[i17] = rawPts_metric->data[i17];
  }

  emxInit_real32_T(&locations, 2);
  applyMinQuality(rawPts_Location, rawPts_Metric, 0.1, locations, rawPts_metric);
  cornerPoints_cg_cornerPoints_cg(locations, rawPts_metric, pts_pLocation,
    pts_pMetric);
  emxFree_real32_T(&locations);
  emxFree_real32_T(&rawPts_metric);
  emxFree_real32_T(&rawPts_Metric);
  emxFree_real32_T(&rawPts_Location);
}

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
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

//
// Arguments    : emxArray_real32_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void eml_sort(emxArray_real32_T *x, emxArray_int32_T *idx)
{
  int dim;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  b_eml_sort(x, dim, idx);
}

//
// Arguments    : emxArray_real32_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void eml_sort_idx(emxArray_real32_T *x, emxArray_int32_T *idx)
{
  emxArray_real32_T *b_x;
  int ib;
  int p;
  int m;
  int n;
  float x4[4];
  int idx4[4];
  emxArray_real32_T *xwork;
  int nNaNs;
  int k;
  int wOffset;
  int i4;
  signed char perm[4];
  int nNonNaN;
  int nBlocks;
  int iwork[256];
  float b_xwork[256];
  int bLen;
  int bLen2;
  int nPairs;
  int32_T exitg1;
  emxArray_int32_T *b_iwork;
  b_emxInit_real32_T(&b_x, 1);
  ib = x->size[0];
  p = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, p, (int)sizeof(float));
  m = x->size[0];
  for (p = 0; p < m; p++) {
    b_x->data[p] = x->data[p];
  }

  p = idx->size[0];
  idx->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)idx, p, (int)sizeof(int));
  for (p = 0; p < ib; p++) {
    idx->data[p] = 0;
  }

  n = x->size[0];
  for (m = 0; m < 4; m++) {
    x4[m] = 0.0F;
    idx4[m] = 0;
  }

  b_emxInit_real32_T(&xwork, 1);
  ib = x->size[0];
  p = xwork->size[0];
  xwork->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)xwork, p, (int)sizeof(float));
  nNaNs = 0;
  ib = 0;
  for (k = 0; k + 1 <= n; k++) {
    if (rtIsNaNF(b_x->data[k])) {
      idx->data[(n - nNaNs) - 1] = k + 1;
      xwork->data[(n - nNaNs) - 1] = b_x->data[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = k + 1;
      x4[ib - 1] = b_x->data[k];
      if (ib == 4) {
        ib = k - nNaNs;
        if (x4[0] >= x4[1]) {
          m = 1;
          p = 2;
        } else {
          m = 2;
          p = 1;
        }

        if (x4[2] >= x4[3]) {
          wOffset = 3;
          i4 = 4;
        } else {
          wOffset = 4;
          i4 = 3;
        }

        if (x4[m - 1] >= x4[wOffset - 1]) {
          if (x4[p - 1] >= x4[wOffset - 1]) {
            perm[0] = (signed char)m;
            perm[1] = (signed char)p;
            perm[2] = (signed char)wOffset;
            perm[3] = (signed char)i4;
          } else if (x4[p - 1] >= x4[i4 - 1]) {
            perm[0] = (signed char)m;
            perm[1] = (signed char)wOffset;
            perm[2] = (signed char)p;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)m;
            perm[1] = (signed char)wOffset;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)p;
          }
        } else if (x4[m - 1] >= x4[i4 - 1]) {
          if (x4[p - 1] >= x4[i4 - 1]) {
            perm[0] = (signed char)wOffset;
            perm[1] = (signed char)m;
            perm[2] = (signed char)p;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)wOffset;
            perm[1] = (signed char)m;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)p;
          }
        } else {
          perm[0] = (signed char)wOffset;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)m;
          perm[3] = (signed char)p;
        }

        idx->data[ib - 3] = idx4[perm[0] - 1];
        idx->data[ib - 2] = idx4[perm[1] - 1];
        idx->data[ib - 1] = idx4[perm[2] - 1];
        idx->data[ib] = idx4[perm[3] - 1];
        b_x->data[ib - 3] = x4[perm[0] - 1];
        b_x->data[ib - 2] = x4[perm[1] - 1];
        b_x->data[ib - 1] = x4[perm[2] - 1];
        b_x->data[ib] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  wOffset = (x->size[0] - nNaNs) - 1;
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

    for (k = 1; k <= ib; k++) {
      idx->data[(wOffset - ib) + k] = idx4[perm[k - 1] - 1];
      b_x->data[(wOffset - ib) + k] = x4[perm[k - 1] - 1];
    }
  }

  m = nNaNs >> 1;
  for (k = 1; k <= m; k++) {
    p = idx->data[wOffset + k];
    idx->data[wOffset + k] = idx->data[n - k];
    idx->data[n - k] = p;
    b_x->data[wOffset + k] = xwork->data[n - k];
    b_x->data[n - k] = xwork->data[wOffset + k];
  }

  if ((nNaNs & 1) != 0) {
    b_x->data[(wOffset + m) + 1] = xwork->data[(wOffset + m) + 1];
  }

  nNonNaN = x->size[0] - nNaNs;
  m = 2;
  if (nNonNaN > 1) {
    if (x->size[0] >= 256) {
      nBlocks = nNonNaN >> 8;
      if (nBlocks > 0) {
        for (wOffset = 1; wOffset <= nBlocks; wOffset++) {
          i4 = ((wOffset - 1) << 8) - 1;
          for (n = 0; n < 6; n++) {
            bLen = 1 << (n + 2);
            bLen2 = bLen << 1;
            nPairs = 256 >> (n + 3);
            for (k = 1; k <= nPairs; k++) {
              m = i4 + (k - 1) * bLen2;
              for (ib = 1; ib <= bLen2; ib++) {
                iwork[ib - 1] = idx->data[m + ib];
                b_xwork[ib - 1] = b_x->data[m + ib];
              }

              p = 0;
              ib = bLen;
              do {
                exitg1 = 0;
                m++;
                if (b_xwork[p] >= b_xwork[ib]) {
                  idx->data[m] = iwork[p];
                  b_x->data[m] = b_xwork[p];
                  if (p + 1 < bLen) {
                    p++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  idx->data[m] = iwork[ib];
                  b_x->data[m] = b_xwork[ib];
                  if (ib + 1 < bLen2) {
                    ib++;
                  } else {
                    ib = m - p;
                    while (p + 1 <= bLen) {
                      idx->data[(ib + p) + 1] = iwork[p];
                      b_x->data[(ib + p) + 1] = b_xwork[p];
                      p++;
                    }

                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
        }

        m = nBlocks << 8;
        ib = nNonNaN - m;
        if (ib > 0) {
          merge_block(idx, b_x, m, ib, 2);
        }

        m = 8;
      }
    }

    merge_block(idx, b_x, 0, nNonNaN, m);
  }

  emxInit_int32_T(&b_iwork, 1);
  if ((nNaNs > 0) && (nNonNaN > 0)) {
    p = b_iwork->size[0];
    b_iwork->size[0] = idx->size[0];
    emxEnsureCapacity((emxArray__common *)b_iwork, p, (int)sizeof(int));
    ib = b_x->size[0];
    p = xwork->size[0];
    xwork->size[0] = ib;
    emxEnsureCapacity((emxArray__common *)xwork, p, (int)sizeof(float));
    for (k = 0; k + 1 <= nNaNs; k++) {
      xwork->data[k] = b_x->data[nNonNaN + k];
      b_iwork->data[k] = idx->data[nNonNaN + k];
    }

    for (k = nNonNaN - 1; k + 1 > 0; k--) {
      b_x->data[nNaNs + k] = b_x->data[k];
      idx->data[nNaNs + k] = idx->data[k];
    }

    for (k = 0; k + 1 <= nNaNs; k++) {
      b_x->data[k] = xwork->data[k];
      idx->data[k] = b_iwork->data[k];
    }
  }

  emxFree_int32_T(&b_iwork);
  emxFree_real32_T(&xwork);
  p = x->size[0];
  x->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)x, p, (int)sizeof(float));
  m = b_x->size[0];
  for (p = 0; p < m; p++) {
    x->data[p] = b_x->data[p];
  }

  emxFree_real32_T(&b_x);
}

//
// Arguments    : double dst[2]
//                const double src[2]
// Return Type  : void
//
static void emxCopyMatrix_real_T(double dst[2], const double src[2])
{
  int i;
  for (i = 0; i < 2; i++) {
    dst[i] = src[i];
  }
}

//
// Arguments    : b_struct_T *dst
//                const b_struct_T *src
// Return Type  : void
//
static void emxCopyStruct_struct_T(b_struct_T *dst, const b_struct_T *src)
{
  emxCopyMatrix_real_T(dst->offset, src->offset);
  dst->count = src->count;
  emxCopy_int8_T(&dst->grid, &src->grid);
}

//
// Arguments    : emxArray_int8_T **dst
//                emxArray_int8_T * const *src
// Return Type  : void
//
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
// Arguments    : emxArray_b_struct_T *emxArray
//                int oldNumel
// Return Type  : void
//
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

//
// Arguments    : emxArray_b_struct_T *emxArray
//                int fromIndex
//                int toIndex
// Return Type  : void
//
static void emxExpand_struct_T(emxArray_b_struct_T *emxArray, int fromIndex, int
  toIndex)
{
  int i;
  for (i = fromIndex; i < toIndex; i++) {
    emxInitStruct_struct_T(&emxArray->data[i]);
  }
}

//
// Arguments    : b_struct_T *pStruct
// Return Type  : void
//
static void emxFreeStruct_struct_T(b_struct_T *pStruct)
{
  emxFree_int8_T(&pStruct->grid);
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
// Arguments    : emxArray_int8_T **pEmxArray
// Return Type  : void
//
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
// Arguments    : emxArray_struct_T **pEmxArray
// Return Type  : void
//
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
// Arguments    : b_struct_T *pStruct
// Return Type  : void
//
static void emxInitStruct_struct_T(b_struct_T *pStruct)
{
  emxInit_int8_T(&pStruct->grid, 2);
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
// Arguments    : emxArray_int8_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
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
// Arguments    : emxArray_struct_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
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
// Arguments    : emxArray_b_struct_T *emxArray
//                int fromIndex
//                int toIndex
// Return Type  : void
//
static void emxTrim_struct_T(emxArray_b_struct_T *emxArray, int fromIndex, int
  toIndex)
{
  int i;
  for (i = fromIndex; i < toIndex; i++) {
    emxFreeStruct_struct_T(&emxArray->data[i]);
  }
}

//
// Arguments    : emxArray_real_T *a
//                const double block[2]
//                emxArray_real_T *b
// Return Type  : void
//
static void im2col(emxArray_real_T *a, const double block[2], emxArray_real_T *b)
{
  int ma;
  unsigned int b_a[2];
  double b_block[2];
  boolean_T c_a[2];
  int i11;
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

  //  matlab im2col function put here for codegen
  ma = a->size[0];
  b_a[0] = (unsigned int)a->size[0];
  b_a[1] = (unsigned int)a->size[1];
  b_block[0] = block[0];
  b_block[1] = block[1];
  for (i11 = 0; i11 < 2; i11++) {
    c_a[i11] = (b_a[i11] < b_block[i11]);
  }

  if (any(c_a)) {
    //  if neighborhood is larger than image
    i11 = b->size[0] * b->size[1];
    b->size[0] = (int)(block[0] * block[1]);
    b->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)b, i11, (int)sizeof(double));
  } else {
    //  Create Hankel-like indexing sub matrix.
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
        nm1d2 = (n + (n < 0)) >> 1;
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
    for (i11 = 0; i11 < loop_ub; i11++) {
      cidx_data[i11] = y_data[i11];
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
    i11 = ridx->size[0] * ridx->size[1];
    ridx->size[0] = 1;
    ridx->size[1] = b_n + 1;
    emxEnsureCapacity((emxArray__common *)ridx, i11, (int)sizeof(double));
    if (b_n + 1 > 0) {
      ridx->data[0] = anew;
      if (b_n + 1 > 1) {
        ridx->data[b_n] = apnd;
        nm1d2 = (b_n + (b_n < 0)) >> 1;
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
    i11 = t->size[0] * t->size[1];
    t->size[0] = n + 1;
    t->size[1] = (int)nc;
    emxEnsureCapacity((emxArray__common *)t, i11, (int)sizeof(double));
    k = (int)nc;
    for (i11 = 0; i11 < k; i11++) {
      for (b_n = 0; b_n < loop_ub; b_n++) {
        t->data[b_n + t->size[0] * i11] = cidx_data[b_n] + ridx->data[ridx->
          size[0] * i11];
      }
    }

    b_emxInit_real_T(&tt, 2);

    //  Hankel Subscripts
    i11 = tt->size[0] * tt->size[1];
    tt->size[0] = (int)(block[0] * block[1]);
    emxEnsureCapacity((emxArray__common *)tt, i11, (int)sizeof(double));
    i11 = tt->size[0] * tt->size[1];
    tt->size[1] = (int)nc;
    emxEnsureCapacity((emxArray__common *)tt, i11, (int)sizeof(double));
    loop_ub = (int)(block[0] * block[1]) * (int)nc;
    for (i11 = 0; i11 < loop_ub; i11++) {
      tt->data[i11] = 0.0;
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

    i11 = ridx->size[0] * ridx->size[1];
    ridx->size[0] = 1;
    ridx->size[1] = n + 1;
    emxEnsureCapacity((emxArray__common *)ridx, i11, (int)sizeof(double));
    if (n + 1 > 0) {
      ridx->data[0] = anew;
      if (n + 1 > 1) {
        ridx->data[n] = apnd;
        nm1d2 = (n + (n < 0)) >> 1;
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
      for (i11 = 0; i11 < loop_ub; i11++) {
        tmp_data[i11] = (int)(anew + ridx->data[ridx->size[0] * i11]) - 1;
      }

      anew = (double)ma * (double)nm1d2;
      loop_ub = t->size[1];
      for (i11 = 0; i11 < loop_ub; i11++) {
        k = t->size[0];
        for (b_n = 0; b_n < k; b_n++) {
          tt->data[tmp_data[b_n] + tt->size[0] * i11] = t->data[b_n + t->size[0]
            * i11] + anew;
        }
      }
    }

    emxFree_real_T(&t);
    b_emxInit_real_T(&ttt, 2);
    i11 = ttt->size[0] * ttt->size[1];
    ttt->size[0] = (int)(block[0] * block[1]);
    emxEnsureCapacity((emxArray__common *)ttt, i11, (int)sizeof(double));
    i11 = ttt->size[0] * ttt->size[1];
    ttt->size[1] = (int)(nc * nn);
    emxEnsureCapacity((emxArray__common *)ttt, i11, (int)sizeof(double));
    loop_ub = (int)(block[0] * block[1]) * (int)(nc * nn);
    for (i11 = 0; i11 < loop_ub; i11++) {
      ttt->data[i11] = 0.0;
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

    i11 = ridx->size[0] * ridx->size[1];
    ridx->size[0] = 1;
    ridx->size[1] = n + 1;
    emxEnsureCapacity((emxArray__common *)ridx, i11, (int)sizeof(double));
    if (n + 1 > 0) {
      ridx->data[0] = anew;
      if (n + 1 > 1) {
        ridx->data[n] = apnd;
        nm1d2 = (n + (n < 0)) >> 1;
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
      i11 = r3->size[0];
      r3->size[0] = ridx->size[1];
      emxEnsureCapacity((emxArray__common *)r3, i11, (int)sizeof(int));
      loop_ub = ridx->size[1];
      for (i11 = 0; i11 < loop_ub; i11++) {
        r3->data[i11] = (int)(anew + ridx->data[ridx->size[0] * i11]) - 1;
      }

      anew = (double)ma * (double)nm1d2;
      loop_ub = tt->size[1];
      for (i11 = 0; i11 < loop_ub; i11++) {
        k = tt->size[0];
        for (b_n = 0; b_n < k; b_n++) {
          ttt->data[b_n + ttt->size[0] * r3->data[i11]] = tt->data[b_n +
            tt->size[0] * i11] + anew;
        }
      }

      nm1d2++;
    }

    emxFree_int32_T(&r3);
    emxFree_real_T(&tt);
    emxFree_real_T(&ridx);

    //  If a is a row vector, change it to a column vector. This change is
    //  necessary when A is a row vector and [M N] = size(A).
    emxInit_real_T(&d_a, 1);
    if ((a->size[1] > 1) && (a->size[0] == 1)) {
      nm1d2 = a->size[1];
      k = a->size[1];
      i11 = d_a->size[0];
      d_a->size[0] = nm1d2;
      emxEnsureCapacity((emxArray__common *)d_a, i11, (int)sizeof(double));
      for (i11 = 0; i11 < nm1d2; i11++) {
        d_a->data[i11] = a->data[i11];
      }

      i11 = a->size[0] * a->size[1];
      a->size[0] = k;
      a->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)a, i11, (int)sizeof(double));
      i11 = 0;
      while (i11 <= 0) {
        for (i11 = 0; i11 < k; i11++) {
          a->data[i11] = d_a->data[i11];
        }

        i11 = 1;
      }
    }

    emxFree_real_T(&d_a);
    i11 = b->size[0] * b->size[1];
    b->size[0] = ttt->size[0];
    b->size[1] = ttt->size[1];
    emxEnsureCapacity((emxArray__common *)b, i11, (int)sizeof(double));
    loop_ub = ttt->size[0] * ttt->size[1];
    for (i11 = 0; i11 < loop_ub; i11++) {
      b->data[i11] = a->data[(int)ttt->data[i11] - 1];
    }

    emxFree_real_T(&ttt);
  }
}

//
// Arguments    : const emxArray_int8_T *A
//                emxArray_int8_T *B
// Return Type  : void
//
static void imerode(const emxArray_int8_T *A, emxArray_int8_T *B)
{
  int i15;
  int i16;
  double asize[2];
  boolean_T nhood[9];
  double nsize[2];
  for (i15 = 0; i15 < 2; i15++) {
    i16 = B->size[0] * B->size[1];
    B->size[i15] = A->size[i15];
    emxEnsureCapacity((emxArray__common *)B, i16, (int)sizeof(signed char));
  }

  for (i15 = 0; i15 < 2; i15++) {
    asize[i15] = A->size[i15];
  }

  for (i15 = 0; i15 < 9; i15++) {
    nhood[i15] = true;
  }

  for (i15 = 0; i15 < 2; i15++) {
    nsize[i15] = 3.0;
  }

  erode_flat_int8_tbb(&A->data[0], asize, 2.0, nhood, nsize, 2.0, &B->data[0]);
}

//
// INITFEATURES initialize features for all anchors at the beginning
//  features numFeatures x 2 matrix
//  threshold suggested threshold for FASt detector (dynamic adjusted
//  initially set to default 30)
//  flag shows the status of initialization:
//  {0,1,-1} = {successful but not balanced,successful,unsuccessful not
//  enough features found}
// Arguments    : const unsigned char greyImg[332672]
//                double borderLength
//                double numBinsX
//                double numBinsY
//                double minDistance
//                double features[64]
//                double featStatus[32]
//                double *threshold
//                double *flag
// Return Type  : void
//
static void initFeatures(const unsigned char greyImg[332672], double
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
  emxArray_real32_T *arrBinCandidates_pLocation;
  emxArray_real32_T *arrBinCandidates_pMetric;
  emxArray_int32_T *iidx;
  emxArray_real32_T *rawPts_Location;
  emxArray_real32_T *rawPts_Metric;
  emxArray_real32_T *rawPts_metric;
  emxArray_real32_T *locations;
  emxArray_real32_T *rawPts_loc;
  emxArray_uint8_T *Iu8;
  int j;
  double offsetX;
  double offsetY;
  double d0;
  int i1;
  int out_numel;
  int i2;
  unsigned char minContrast;
  double mtmp;
  int i3;
  int b_loop_ub;
  int i4;
  void * ptrKeypoints;
  double numGridsX;
  double numGridsY;
  double numLeftOverFeature;
  double numCheckedBins;
  double numFeatureInit;
  double numExtraToCheckBin;
  emxArray_boolean_T *binGrids;
  int bin;
  double numFeatureInit_old;
  double arrBinCandidates_data[64];
  double grid[2];
  unsigned int cntFeatureInitBin;
  int cntChecked;
  int32_T exitg1;
  double left[2];
  boolean_T guard1 = false;
  *flag = 1.0;
  memset(&features[0], 0, sizeof(double) << 6);
  memset(&featStatus[0], 0, sizeof(double) << 5);
  emxInit_struct_T(&structBin, 1);

  // % divide bins and find fast feature in each bin
  //  set initial output
  binWidth = rt_roundd_snf((736.0 - 2.0 * borderLength) / numBinsX);
  binHeight = rt_roundd_snf((452.0 - 2.0 * borderLength) / numBinsY);
  numBins = numBinsX * numBinsY;
  numAvgFeaturePerBin = 32.0 / numBins;

  //  parameter for detector
  *threshold = 0.2;

  //  default value
  //  minimum threshold allowed (used in threshold adaption)
  //  flag to adjust threshold. it is incremented in each bin, if there are too
  //  many features found in this bin
  tooMany = 0.0;

  //  initialize structBin
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
  emxInit_real32_T(&arrBinCandidates_pLocation, 2);
  b_emxInit_real32_T(&arrBinCandidates_pMetric, 1);
  emxInit_int32_T(&iidx, 1);
  emxInit_real32_T(&rawPts_Location, 2);
  b_emxInit_real32_T(&rawPts_Metric, 1);
  b_emxInit_real32_T(&rawPts_metric, 1);
  emxInit_real32_T(&locations, 2);
  emxInit_real32_T(&rawPts_loc, 2);
  emxInit_uint8_T(&Iu8, 2);
  while (i <= (int)numBinsX - 1) {
    for (j = 0; j < (int)numBinsY; j++) {
      offsetX = borderLength + binWidth * ((1.0 + (double)i) - 1.0);
      offsetY = borderLength + binHeight * ((1.0 + (double)j) - 1.0);
      structBin->data[(int)cntBin - 1].offset[0] = offsetX;
      structBin->data[(int)cntBin - 1].offset[1] = offsetY;
      d0 = (offsetY + 1.0) + binHeight;
      if (offsetY + 1.0 > d0) {
        i0 = 0;
        i1 = 0;
      } else {
        i0 = (int)(offsetY + 1.0) - 1;
        i1 = (int)d0;
      }

      d0 = (offsetX + 1.0) + binWidth;
      if (offsetX + 1.0 > d0) {
        out_numel = 0;
        i2 = 0;
      } else {
        out_numel = (int)(offsetX + 1.0) - 1;
        i2 = (int)d0;
      }

      //  feature detection
      //  arrBinCandidates = detectHarrisFeatures(subImg,'MinQuality',0.5,'FilterSize',5); 
      mtmp = 0.2;
      grayto8_real64(&mtmp, &minContrast, 1.0);
      i3 = Iu8->size[0] * Iu8->size[1];
      Iu8->size[0] = i1 - i0;
      Iu8->size[1] = i2 - out_numel;
      emxEnsureCapacity((emxArray__common *)Iu8, i3, (int)sizeof(unsigned char));
      loop_ub = i2 - out_numel;
      for (i3 = 0; i3 < loop_ub; i3++) {
        b_loop_ub = i1 - i0;
        for (i4 = 0; i4 < b_loop_ub; i4++) {
          Iu8->data[i4 + Iu8->size[0] * i3] = greyImg[(i0 + i4) + 452 *
            (out_numel + i3)];
        }
      }

      ptrKeypoints = NULL;
      out_numel = detectFAST_compute(&Iu8->data[0], i1 - i0, i2 - out_numel,
        false, minContrast, &ptrKeypoints);
      i0 = rawPts_loc->size[0] * rawPts_loc->size[1];
      rawPts_loc->size[0] = out_numel;
      rawPts_loc->size[1] = 2;
      emxEnsureCapacity((emxArray__common *)rawPts_loc, i0, (int)sizeof(float));
      i0 = rawPts_metric->size[0];
      rawPts_metric->size[0] = out_numel;
      emxEnsureCapacity((emxArray__common *)rawPts_metric, i0, (int)sizeof(float));
      detectFAST_assignOutput(ptrKeypoints, &rawPts_loc->data[0],
        &rawPts_metric->data[0]);
      i0 = rawPts_Location->size[0] * rawPts_Location->size[1];
      rawPts_Location->size[0] = rawPts_loc->size[0];
      rawPts_Location->size[1] = rawPts_loc->size[1];
      emxEnsureCapacity((emxArray__common *)rawPts_Location, i0, (int)sizeof
                        (float));
      loop_ub = rawPts_loc->size[0] * rawPts_loc->size[1];
      for (i0 = 0; i0 < loop_ub; i0++) {
        rawPts_Location->data[i0] = rawPts_loc->data[i0];
      }

      i0 = rawPts_Metric->size[0];
      rawPts_Metric->size[0] = rawPts_metric->size[0];
      emxEnsureCapacity((emxArray__common *)rawPts_Metric, i0, (int)sizeof(float));
      loop_ub = rawPts_metric->size[0];
      for (i0 = 0; i0 < loop_ub; i0++) {
        rawPts_Metric->data[i0] = rawPts_metric->data[i0];
      }

      applyMinQuality(rawPts_Location, rawPts_Metric, 0.5, locations,
                      rawPts_metric);
      cornerPoints_cg_cornerPoints_cg(locations, rawPts_metric,
        arrBinCandidates_pLocation, arrBinCandidates_pMetric);

      //  rank the feature wrt the detector response and keep maximal
      //  numFeatures features in each bin
      i0 = rawPts_metric->size[0];
      rawPts_metric->size[0] = arrBinCandidates_pMetric->size[0];
      emxEnsureCapacity((emxArray__common *)rawPts_metric, i0, (int)sizeof(float));
      loop_ub = arrBinCandidates_pMetric->size[0];
      for (i0 = 0; i0 < loop_ub; i0++) {
        rawPts_metric->data[i0] = arrBinCandidates_pMetric->data[i0];
      }

      eml_sort(rawPts_metric, iidx);
      i0 = unusedU1->size[0];
      unusedU1->size[0] = iidx->size[0];
      emxEnsureCapacity((emxArray__common *)unusedU1, i0, (int)sizeof(double));
      loop_ub = iidx->size[0];
      for (i0 = 0; i0 < loop_ub; i0++) {
        unusedU1->data[i0] = iidx->data[i0];
      }

      d0 = FeaturePointsImpl_get_Count(arrBinCandidates_pLocation);
      mtmp = 32.0;
      if (d0 < 32.0) {
        mtmp = d0;
      }

      featureCounts->data[(int)cntBin - 1] = mtmp;

      //  save candidates in bin structures
      if (1.0 > mtmp) {
        loop_ub = 0;
      } else {
        loop_ub = (int)mtmp;
      }

      i0 = arrBinFeatures->size[0] * arrBinFeatures->size[1];
      arrBinFeatures->size[0] = loop_ub;
      arrBinFeatures->size[1] = 2;
      emxEnsureCapacity((emxArray__common *)arrBinFeatures, i0, (int)sizeof
                        (double));
      for (i0 = 0; i0 < 2; i0++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          arrBinFeatures->data[i1 + arrBinFeatures->size[0] * i0] =
            arrBinCandidates_pLocation->data[((int)unusedU1->data[i1] +
            arrBinCandidates_pLocation->size[0] * i0) - 1];
        }
      }

      for (i0 = 0; i0 < 2; i0++) {
        loop_ub = arrBinFeatures->size[0];
        for (i1 = 0; i1 < loop_ub; i1++) {
          structBin->data[(int)cntBin - 1].featureCandidates[i1 + (i0 << 5)] =
            arrBinFeatures->data[i1 + arrBinFeatures->size[0] * i0];
        }
      }

      cntBin++;
    }

    i++;
  }

  emxFree_uint8_T(&Iu8);
  emxFree_real32_T(&rawPts_loc);
  emxFree_real32_T(&locations);
  emxFree_real32_T(&rawPts_metric);
  emxFree_real32_T(&rawPts_Metric);
  emxFree_real32_T(&rawPts_Location);
  emxFree_real32_T(&arrBinCandidates_pMetric);
  emxFree_real32_T(&arrBinCandidates_pLocation);

  // % fine placement in each bin, a better way to keep minimum distance
  //  check distance to existing feature by constructing a fine grid with
  //  gridwidth = minDistance
  //  proceed from the highest ranking feature,
  //  check in candidate's neighboring grids whether a features has already
  //  been chosen, if yes discard this candidate
  //  a grid is flagged 1 if a feature in its area is found and 0 otherwise
  numGridsX = ceil(binWidth / minDistance);
  numGridsY = ceil(binHeight / minDistance);

  //  start from the bin with least number of candidates, because it's most
  //  possible to be not filled. if one bin is not filled, we'll be filled in
  //  the next bin
  i0 = unusedU1->size[0];
  unusedU1->size[0] = featureCounts->size[0];
  emxEnsureCapacity((emxArray__common *)unusedU1, i0, (int)sizeof(double));
  loop_ub = featureCounts->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    unusedU1->data[i0] = featureCounts->data[i0];
  }

  c_eml_sort(unusedU1, iidx);
  i0 = unusedU1->size[0];
  unusedU1->size[0] = iidx->size[0];
  emxEnsureCapacity((emxArray__common *)unusedU1, i0, (int)sizeof(double));
  loop_ub = iidx->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    unusedU1->data[i0] = iidx->data[i0];
  }

  emxFree_int32_T(&iidx);

  //  number of features that cannot be filled in the processed bins
  numLeftOverFeature = 0.0;
  numCheckedBins = 0.0;

  //  number of initialized features in all bin (in order to make sure total
  //  number of features does not exceed required)
  numFeatureInit = 0.0;
  numExtraToCheckBin = 0.0;
  i = 0;
  emxInit_boolean_T(&binGrids, 3);
  while (i <= unusedU1->size[0] - 1) {
    bin = (int)unusedU1->data[i] - 1;
    numCheckedBins++;
    numFeatureInit_old = numFeatureInit;

    //  get number of found features in this bin.
    //  if found nothing, then increase the number of features to be found in
    //  other bins
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
        for (i1 = 0; i1 < loop_ub; i1++) {
          arrBinCandidates_data[i1 + loop_ub * i0] = structBin->data[(int)
            unusedU1->data[i] - 1].featureCandidates[i1 + (i0 << 5)];
        }
      }

      //  initialize arrBinFeatures
      i0 = arrBinFeatures->size[0] * arrBinFeatures->size[1];
      arrBinFeatures->size[0] = (int)(numAvgFeaturePerBin + numExtraToCheckBin);
      arrBinFeatures->size[1] = 2;
      emxEnsureCapacity((emxArray__common *)arrBinFeatures, i0, (int)sizeof
                        (double));
      b_loop_ub = (int)(numAvgFeaturePerBin + numExtraToCheckBin) << 1;
      for (i0 = 0; i0 < b_loop_ub; i0++) {
        arrBinFeatures->data[i0] = 0.0;
      }

      //  a finer grids to represent location of keypoints
      //  first and last row and column are dummy row and column
      //  hence grid = grid + [1,1]
      //  allocate memory for matrix, in each bin maximum numFeatures
      //  features are found
      i0 = binGrids->size[0] * binGrids->size[1] * binGrids->size[2];
      binGrids->size[0] = (int)(numGridsX + 2.0);
      binGrids->size[1] = (int)(numGridsY + 2.0);
      binGrids->size[2] = 32;
      emxEnsureCapacity((emxArray__common *)binGrids, i0, (int)sizeof(boolean_T));
      b_loop_ub = (int)(numGridsX + 2.0) * (int)(numGridsY + 2.0) << 5;
      for (i0 = 0; i0 < b_loop_ub; i0++) {
        binGrids->data[i0] = false;
      }

      //  always keep the highest ranking candidate
      for (i0 = 0; i0 < 2; i0++) {
        grid[i0] = arrBinCandidates_data[loop_ub * i0] / minDistance;
      }

      b_ceil(grid);

      //  save highest rating feature
      for (i0 = 0; i0 < 2; i0++) {
        arrBinFeatures->data[arrBinFeatures->size[0] * i0] =
          arrBinCandidates_data[loop_ub * i0] + structBin->data[(int)
          unusedU1->data[i] - 1].offset[i0];
      }

      binGrids->data[((int)grid[0] + (int)(numGridsX + 2.0) * ((int)grid[1] - 1))
        - 1] = true;
      numFeatureInit++;

      //  start checking candidates
      cntFeatureInitBin = 1U;
      cntChecked = 0;
      do {
        exitg1 = 0;
        if (cntChecked + 1 < featureCounts->data[bin]) {
          cntChecked++;
          for (i0 = 0; i0 < 2; i0++) {
            grid[i0] = arrBinCandidates_data[cntChecked + loop_ub * i0] /
              minDistance;
          }

          b_ceil(grid);
          for (i0 = 0; i0 < 2; i0++) {
            grid[i0]++;
          }

          out_numel = binGrids->size[0];
          if (binGrids->data[((int)grid[0] + out_numel * ((int)grid[1] - 1)) - 1])
          {
          } else {
            for (i0 = 0; i0 < 2; i0++) {
              left[i0] = grid[i0] + (-1.0 + (double)i0);
            }

            out_numel = binGrids->size[0];
            if (binGrids->data[((int)left[0] + out_numel * ((int)left[1] - 1)) -
                1]) {
            } else {
              for (i0 = 0; i0 < 2; i0++) {
                left[i0] = grid[i0] + (1.0 - (double)i0);
              }

              out_numel = binGrids->size[0];
              if (binGrids->data[((int)left[0] + out_numel * ((int)left[1] - 1))
                  - 1]) {
              } else {
                for (i0 = 0; i0 < 2; i0++) {
                  left[i0] = grid[i0] + (0.0 - (double)i0);
                }

                out_numel = binGrids->size[0];
                if (binGrids->data[((int)left[0] + out_numel * ((int)left[1] - 1))
                    - 1]) {
                } else {
                  for (i0 = 0; i0 < 2; i0++) {
                    left[i0] = grid[i0] + (double)i0;
                  }

                  out_numel = binGrids->size[0];
                  if (binGrids->data[((int)left[0] + out_numel * ((int)left[1] -
                        1)) - 1]) {
                  } else {
                    //  passed check mark grid as occupied
                    out_numel = binGrids->size[0];
                    binGrids->data[((int)grid[0] + out_numel * ((int)grid[1] - 1))
                      - 1] = true;
                    cntFeatureInitBin++;
                    numFeatureInit++;
                    for (i0 = 0; i0 < 2; i0++) {
                      arrBinFeatures->data[((int)cntFeatureInitBin +
                                            arrBinFeatures->size[0] * i0) - 1] =
                        arrBinCandidates_data[cntChecked + loop_ub * i0] +
                        structBin->data[bin].offset[i0];
                    }

                    if ((cntFeatureInitBin >= numAvgFeaturePerBin +
                         numExtraToCheckBin) || (numFeatureInit >= 32.0)) {
                      //  double check
                      //  also an early break means we found too many features
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
        cntFeatureInitBin;

      //  stop adding features to this bin if this feature is full and the
      //  left over features from earlier bins are also found in this bin
      numExtraToCheckBin = rt_roundd_snf(numLeftOverFeature / ((numBins -
        numCheckedBins) + 1.0));

      //  if there's leftover slots in this bin, flag 0 (unbalanced)
      if (numLeftOverFeature > 0.0) {
        *flag = 0.0;
      }

      //  if this bin is not filled
      //  store results in a cell for the full image
      if (numFeatureInit_old + 1.0 > numFeatureInit) {
        i0 = 0;
      } else {
        i0 = (int)(numFeatureInit_old + 1.0) - 1;
      }

      for (i1 = 0; i1 < 2; i1++) {
        loop_ub = (int)cntFeatureInitBin;
        for (out_numel = 0; out_numel < loop_ub; out_numel++) {
          features[(i0 + out_numel) + (i1 << 5)] = arrBinFeatures->
            data[out_numel + arrBinFeatures->size[0] * i1];
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

  // % set flag
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
    // % dynamically adjust threshold
    //  dynamically adjust parameter to keep number of found features
    //  around a certain value. This step reduces future computation
    //  if features found in one bin are more than number of required
    //  features the whole image, increase threshold
    if (tooMany == numBins) {
      *threshold = 0.21000000000000002;
    } else {
      if (*flag == -1.0) {
        //  decrease threshold slowlier, because we stil want good valid features 
        *threshold = 0.19;
      }
    }

    // % return output
    loop_ub = (int)numFeatureInit;
    for (i0 = 0; i0 < loop_ub; i0++) {
      featStatus[i0] = 2.0;
    }
  }
}

//
// Arguments    : const emxArray_int8_T *A
//                const emxArray_int8_T *B
//                emxArray_int8_T *K
// Return Type  : void
//
static void kron(const emxArray_int8_T *A, const emxArray_int8_T *B,
                 emxArray_int8_T *K)
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  kidx = A->size[0] * B->size[0];
  b_j1 = A->size[1] * B->size[1];
  j2 = K->size[0] * K->size[1];
  K->size[0] = kidx;
  K->size[1] = b_j1;
  emxEnsureCapacity((emxArray__common *)K, j2, (int)sizeof(signed char));
  kidx = -1;
  for (b_j1 = 1; b_j1 <= A->size[1]; b_j1++) {
    for (j2 = 1; j2 <= B->size[1]; j2++) {
      for (i1 = 1; i1 <= A->size[0]; i1++) {
        for (i2 = 1; i2 <= B->size[0]; i2++) {
          kidx++;
          K->data[kidx] = A->data[(i1 + A->size[0] * (b_j1 - 1)) - 1];
        }
      }
    }
  }
}

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real32_T *x
//                int offset
//                int np
//                int nq
// Return Type  : void
//
static void merge(emxArray_int32_T *idx, emxArray_real32_T *x, int offset, int
                  np, int nq)
{
  emxArray_int32_T *iwork;
  emxArray_real32_T *xwork;
  int n;
  int qend;
  int p;
  int iout;
  int32_T exitg1;
  emxInit_int32_T(&iwork, 1);
  b_emxInit_real32_T(&xwork, 1);
  n = iwork->size[0];
  iwork->size[0] = idx->size[0];
  emxEnsureCapacity((emxArray__common *)iwork, n, (int)sizeof(int));
  qend = x->size[0];
  n = xwork->size[0];
  xwork->size[0] = qend;
  emxEnsureCapacity((emxArray__common *)xwork, n, (int)sizeof(float));
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
      if (xwork->data[p] >= xwork->data[n]) {
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

  emxFree_real32_T(&xwork);
  emxFree_int32_T(&iwork);
}

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real32_T *x
//                int offset
//                int n
//                int preSortLevel
// Return Type  : void
//
static void merge_block(emxArray_int32_T *idx, emxArray_real32_T *x, int offset,
  int n, int preSortLevel)
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
        merge(idx, x, offset + tailOffset, bLen, nTail - bLen);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 1; nTail <= nPairs; nTail++) {
      merge(idx, x, offset + (nTail - 1) * tailOffset, bLen, bLen);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    merge(idx, x, offset, bLen, n - bLen);
  }
}

//
// Arguments    : const double x[2]
//                const double y[2]
//                double z[2]
// Return Type  : void
//
static void rdivide(const double x[2], const double y[2], double z[2])
{
  int i14;
  for (i14 = 0; i14 < 2; i14++) {
    z[i14] = x[i14] / y[i14];
  }
}

//
// REFILLFEATURES finds new features to fill in feature slots
//  features numFeatures x 2 matrix
//  threshold suggested threshold for FASt detector (dynamic adjusted
//  initially set to default 30)
//  flag shows whether enough features are found
// % parameters
//  detector parameters
//  TODO dynamically adjusted
// Arguments    : const unsigned char greyImg[332672]
//                double features[64]
//                double featStatus[32]
//                double numAnchors
//                double numBinsX
//                double numBinsY
//                double borderLength
//                double minDistance
// Return Type  : boolean_T
//
static boolean_T refillFeature(const unsigned char greyImg[332672], double
  features[64], double featStatus[32], double numAnchors, double numBinsX,
  double numBinsY, double borderLength, double minDistance)
{
  boolean_T notEnough;
  double featStatus_old[32];
  int i;
  emxArray_boolean_T *deactivatedAnchor;
  double numBins;
  double numPointsPerAnchor;
  int i21;
  int loop_ub;
  int idx;
  double d1;
  double d2;
  int i22;
  signed char ii_data[32];
  int featStatus_size[1];
  double numActiveAnchorFeatures;
  int i23;
  int tmp_data[32];
  int ii;
  boolean_T exitg2;
  boolean_T guard3 = false;
  signed char idxDeadFeatures_data[32];
  double binWidth;
  double binHeight;
  double numGridsX;
  double numGridsY;
  emxArray_int8_T *t0_grid;
  int b_loop_ub;
  b_struct_T expl_temp;
  emxArray_b_struct_T *structBin;
  emxArray_real_T *featureCounts;
  unsigned int b_i;
  int trueCount;
  int partialTrueCount;
  double features_data[64];
  double b_features[2];
  double b_binWidth[2];
  double bin[2];
  double binIdx;
  double features_tmp[64];
  emxArray_int32_T *iidx;
  int numDeadFeature;
  double numFeatureToFind;
  double numFeatureToFindBin;
  double numCheckedBin;
  double numFeatureFound;
  emxArray_int8_T *erodedGrid;
  emxArray_int8_T *mask;
  emxArray_real32_T *arrBinCandidates_pLocation;
  emxArray_real32_T *arrBinCandidates_pMetric;
  emxArray_real32_T *roundedCandLocation;
  emxArray_int8_T *validCandidates;
  emxArray_real_T *ranking;
  emxArray_real_T *r8;
  emxArray_real_T *r9;
  emxArray_real32_T *varargin_1;
  emxArray_real32_T *varargin_2;
  emxArray_uint8_T *b_greyImg;
  emxArray_int8_T *b_erodedGrid;
  emxArray_int8_T *r10;
  emxArray_int8_T *c_erodedGrid;
  emxArray_real32_T *b_arrBinCandidates_pLocation;
  boolean_T exitg1;
  double numFeatureFound_old;
  int c_loop_ub;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  int siz[2];
  double b_varargin_1[3];
  double mtmp;
  boolean_T bv0[32];
  double features_tmp_data[64];
  signed char b_tmp_data[31];

  //  copy old featStatus
  for (i = 0; i < 32; i++) {
    featStatus_old[i] = featStatus[i];
  }

  b_emxInit_boolean_T(&deactivatedAnchor, 1);

  //  erosion neighborhood size
  numBins = numBinsX * numBinsY;

  // % deactivate anchor when half of its features are not tracked
  numPointsPerAnchor = 32.0 / numAnchors;
  i21 = deactivatedAnchor->size[0];
  deactivatedAnchor->size[0] = (int)numAnchors;
  emxEnsureCapacity((emxArray__common *)deactivatedAnchor, i21, (int)sizeof
                    (boolean_T));
  loop_ub = (int)numAnchors;
  for (i21 = 0; i21 < loop_ub; i21++) {
    deactivatedAnchor->data[i21] = false;
  }

  for (idx = 0; idx < (int)numAnchors; idx++) {
    d1 = ((1.0 + (double)idx) - 1.0) * numPointsPerAnchor + 1.0;
    d2 = (1.0 + (double)idx) * numPointsPerAnchor;
    if (d1 > d2) {
      i21 = 0;
      i22 = 0;
    } else {
      i21 = (int)d1 - 1;
      i22 = (int)d2;
    }

    featStatus_size[0] = i22 - i21;
    loop_ub = i22 - i21;
    for (i22 = 0; i22 < loop_ub; i22++) {
      ii_data[i22] = (signed char)(featStatus[i21 + i22] > 0.0);
    }

    numActiveAnchorFeatures = sum(ii_data, featStatus_size);
    if (numActiveAnchorFeatures < ceil(numPointsPerAnchor / 2.0)) {
      d1 = ((1.0 + (double)idx) - 1.0) * numPointsPerAnchor + 1.0;
      d2 = (1.0 + (double)idx) * numPointsPerAnchor;
      if (d1 > d2) {
        i21 = 0;
        i22 = 0;
      } else {
        i21 = (int)d1 - 1;
        i22 = (int)d2;
      }

      loop_ub = i22 - i21;
      for (i23 = 0; i23 < loop_ub; i23++) {
        tmp_data[i23] = i21 + i23;
      }

      loop_ub = i22 - i21;
      for (i21 = 0; i21 < loop_ub; i21++) {
        featStatus[tmp_data[i21]] = 0.0;
      }

      deactivatedAnchor->data[idx] = true;
    }
  }

  //  if no anchor is deactivated, return input feature
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
        ii_data[idx - 1] = (signed char)ii;
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

    for (i21 = 0; i21 < loop_ub; i21++) {
      idxDeadFeatures_data[i21] = ii_data[i21];
    }

    // % go through each bin and fill free grid with width minDistance
    //  create structure to store feature locations
    //  field name idx, xidx, yidx, offset, count, grid
    binWidth = rt_roundd_snf((736.0 - 2.0 * borderLength) / numBinsX);
    binHeight = rt_roundd_snf((452.0 - 2.0 * borderLength) / numBinsY);
    numGridsX = ceil(binWidth / minDistance);
    numGridsY = ceil(binHeight / minDistance);

    //  initialize structBin
    emxInit_int8_T(&t0_grid, 2);
    i21 = t0_grid->size[0] * t0_grid->size[1];
    t0_grid->size[0] = (int)numGridsX;
    t0_grid->size[1] = (int)numGridsY;
    emxEnsureCapacity((emxArray__common *)t0_grid, i21, (int)sizeof(signed char));
    b_loop_ub = (int)numGridsX * (int)numGridsY;
    for (i21 = 0; i21 < b_loop_ub; i21++) {
      t0_grid->data[i21] = 1;
    }

    emxInitStruct_struct_T(&expl_temp);
    i21 = expl_temp.grid->size[0] * expl_temp.grid->size[1];
    expl_temp.grid->size[0] = t0_grid->size[0];
    expl_temp.grid->size[1] = t0_grid->size[1];
    emxEnsureCapacity((emxArray__common *)expl_temp.grid, i21, (int)sizeof
                      (signed char));
    b_loop_ub = t0_grid->size[0] * t0_grid->size[1];
    for (i21 = 0; i21 < b_loop_ub; i21++) {
      expl_temp.grid->data[i21] = t0_grid->data[i21];
    }

    emxFree_int8_T(&t0_grid);
    expl_temp.count = 0.0;
    for (i21 = 0; i21 < 2; i21++) {
      expl_temp.offset[i21] = 0.0;
    }

    b_emxInit_struct_T(&structBin, 1);
    emxInit_real_T(&featureCounts, 1);
    repmat(expl_temp, numBins, structBin);
    i21 = featureCounts->size[0];
    featureCounts->size[0] = (int)numBins;
    emxEnsureCapacity((emxArray__common *)featureCounts, i21, (int)sizeof(double));
    b_loop_ub = (int)numBins;
    emxFreeStruct_struct_T(&expl_temp);
    for (i21 = 0; i21 < b_loop_ub; i21++) {
      featureCounts->data[i21] = 0.0;
    }

    //  [1 numBinsY+1 ...;
    //   2 numBinsY+2 ...]
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

    //  count number of features/bin and their location in grid
    trueCount = 0;
    for (i = 0; i < 32; i++) {
      if (featStatus[i] > 0.0) {
        trueCount++;
      }
    }

    for (i = 0; i < trueCount; i++) {
      idx = 0;
      for (ii = 0; ii < 32; ii++) {
        if (featStatus[ii] > 0.0) {
          idx++;
        }
      }

      partialTrueCount = 0;
      for (ii = 0; ii < 32; ii++) {
        if (featStatus[ii] > 0.0) {
          ii_data[partialTrueCount] = (signed char)(ii + 1);
          partialTrueCount++;
        }
      }

      for (i21 = 0; i21 < 2; i21++) {
        for (i22 = 0; i22 < idx; i22++) {
          features_data[i22 + idx * i21] = features[(ii_data[i22] + (i21 << 5))
            - 1];
        }
      }

      for (i21 = 0; i21 < 2; i21++) {
        b_features[i21] = features_data[i + idx * i21] - borderLength;
      }

      b_binWidth[0] = binWidth;
      b_binWidth[1] = binHeight;
      rdivide(b_features, b_binWidth, bin);
      b_ceil(bin);

      //  get which bin this feature belongs to
      //  TODO perhaps delete these features
      if (bin[0] > numBinsX) {
        bin[0] = numBinsX;
      }

      if (bin[1] > numBinsY) {
        bin[1] = numBinsY;
      }

      //  idx of the bin where this feature belongs to
      binIdx = (bin[0] - 1.0) * numBinsY + bin[1];

      //  increment number of features in this bin by 1
      featureCounts->data[(int)binIdx - 1]++;

      //  relative location in bin
      idx = 0;
      for (ii = 0; ii < 32; ii++) {
        if (featStatus[ii] > 0.0) {
          idx++;
        }
      }

      partialTrueCount = 0;
      for (ii = 0; ii < 32; ii++) {
        if (featStatus[ii] > 0.0) {
          ii_data[partialTrueCount] = (signed char)(ii + 1);
          partialTrueCount++;
        }
      }

      //  get grid coordinates of this
      for (i21 = 0; i21 < 2; i21++) {
        for (i22 = 0; i22 < idx; i22++) {
          features_data[i22 + idx * i21] = features[(ii_data[i22] + (i21 << 5))
            - 1];
        }
      }

      for (i21 = 0; i21 < 2; i21++) {
        bin[i21] = (features_data[i + idx * i21] - structBin->data[(int)binIdx -
                    1].offset[i21]) / minDistance;
      }

      b_ceil(bin);
      structBin->data[(int)binIdx - 1].grid->data[((int)bin[0] + structBin->
        data[(int)binIdx - 1].grid->size[0] * ((int)bin[1] - 1)) - 1] = 0;
    }

    // % go through each bin, compute mask from grid and find new features under mask 
    //  use erosion to create mask from free grid
    //  erosion set target point to 0 if any point in its neighborhood is 0
    //  in our case, a grid is marked as 0 (deactivated) if it's neighborhood has 
    //  0 (occupied)
    memset(&features_tmp[0], 0, sizeof(double) << 6);
    emxInit_int32_T(&iidx, 1);

    //  create erosion neighbourhood
    //  sort bins with ascending occupancy
    c_eml_sort(featureCounts, iidx);
    i21 = featureCounts->size[0];
    featureCounts->size[0] = iidx->size[0];
    emxEnsureCapacity((emxArray__common *)featureCounts, i21, (int)sizeof(double));
    b_loop_ub = iidx->size[0];
    for (i21 = 0; i21 < b_loop_ub; i21++) {
      featureCounts->data[i21] = iidx->data[i21];
    }

    //  number of features still to be found
    trueCount = 0;
    for (i = 0; i < 32; i++) {
      if (featStatus[i] > 0.0) {
        trueCount++;
      }
    }

    numDeadFeature = 32 - trueCount;
    numFeatureToFind = numDeadFeature;

    //  average number of features to be found in each bin
    numFeatureToFindBin = (double)numDeadFeature / numBins;

    //  TODO merge with mask from motion
    numCheckedBin = 0.0;
    numFeatureFound = 0.0;
    i = 0;
    emxInit_int8_T(&erodedGrid, 2);
    emxInit_int8_T(&mask, 2);
    emxInit_real32_T(&arrBinCandidates_pLocation, 2);
    b_emxInit_real32_T(&arrBinCandidates_pMetric, 1);
    emxInit_real32_T(&roundedCandLocation, 2);
    b_emxInit_int8_T(&validCandidates, 1);
    emxInit_real_T(&ranking, 1);
    emxInit_real_T(&r8, 1);
    b_emxInit_real_T(&r9, 2);
    b_emxInit_real32_T(&varargin_1, 1);
    b_emxInit_real32_T(&varargin_2, 1);
    emxInit_uint8_T(&b_greyImg, 2);
    emxInit_int8_T(&b_erodedGrid, 2);
    emxInit_int8_T(&r10, 2);
    emxInit_int8_T(&c_erodedGrid, 2);
    emxInit_real32_T(&b_arrBinCandidates_pLocation, 2);
    exitg1 = false;
    while ((!exitg1) && (i <= featureCounts->size[0] - 1)) {
      numFeatureFound_old = numFeatureFound;
      numCheckedBin++;

      //  create a mask in this bin so that only features whose neighborhood
      //  is not occupied will be saved
      imerode(structBin->data[(int)featureCounts->data[i] - 1].grid, erodedGrid);
      i21 = b_erodedGrid->size[0] * b_erodedGrid->size[1];
      b_erodedGrid->size[0] = erodedGrid->size[1];
      b_erodedGrid->size[1] = erodedGrid->size[0];
      emxEnsureCapacity((emxArray__common *)b_erodedGrid, i21, (int)sizeof
                        (signed char));
      b_loop_ub = erodedGrid->size[0];
      for (i21 = 0; i21 < b_loop_ub; i21++) {
        idx = erodedGrid->size[1];
        for (i22 = 0; i22 < idx; i22++) {
          b_erodedGrid->data[i22 + b_erodedGrid->size[0] * i21] =
            erodedGrid->data[i21 + erodedGrid->size[0] * i22];
        }
      }

      i21 = r10->size[0] * r10->size[1];
      r10->size[0] = (int)minDistance;
      r10->size[1] = (int)minDistance;
      emxEnsureCapacity((emxArray__common *)r10, i21, (int)sizeof(signed char));
      b_loop_ub = (int)minDistance * (int)minDistance;
      for (i21 = 0; i21 < b_loop_ub; i21++) {
        r10->data[i21] = 1;
      }

      kron(b_erodedGrid, r10, erodedGrid);

      //  crop right and down side to correct size mistake due to rounding
      if (1.0 > binHeight) {
        b_loop_ub = 0;
      } else {
        b_loop_ub = (int)binHeight;
      }

      if (1.0 > binWidth) {
        idx = 0;
      } else {
        idx = (int)binWidth;
      }

      i21 = mask->size[0] * mask->size[1];
      mask->size[0] = b_loop_ub;
      mask->size[1] = idx;
      emxEnsureCapacity((emxArray__common *)mask, i21, (int)sizeof(signed char));
      for (i21 = 0; i21 < idx; i21++) {
        for (i22 = 0; i22 < b_loop_ub; i22++) {
          mask->data[i22 + mask->size[0] * i21] = erodedGrid->data[i22 +
            erodedGrid->size[0] * i21];
        }
      }

      //  detect fast feature in current bin
      d1 = ((structBin->data[(int)featureCounts->data[i] - 1].offset[1] + 1.0) +
            binHeight) - 1.0;
      if (structBin->data[(int)featureCounts->data[i] - 1].offset[1] + 1.0 > d1)
      {
        i21 = 0;
        i22 = 0;
      } else {
        i21 = (int)(structBin->data[(int)featureCounts->data[i] - 1].offset[1] +
                    1.0) - 1;
        i22 = (int)d1;
      }

      d1 = ((structBin->data[(int)featureCounts->data[i] - 1].offset[0] + 1.0) +
            binWidth) - 1.0;
      if (structBin->data[(int)featureCounts->data[i] - 1].offset[0] + 1.0 > d1)
      {
        i23 = 0;
        ii = 0;
      } else {
        i23 = (int)(structBin->data[(int)featureCounts->data[i] - 1].offset[0] +
                    1.0) - 1;
        ii = (int)d1;
      }

      partialTrueCount = b_greyImg->size[0] * b_greyImg->size[1];
      b_greyImg->size[0] = i22 - i21;
      b_greyImg->size[1] = ii - i23;
      emxEnsureCapacity((emxArray__common *)b_greyImg, partialTrueCount, (int)
                        sizeof(unsigned char));
      trueCount = ii - i23;
      for (ii = 0; ii < trueCount; ii++) {
        c_loop_ub = i22 - i21;
        for (partialTrueCount = 0; partialTrueCount < c_loop_ub;
             partialTrueCount++) {
          b_greyImg->data[partialTrueCount + b_greyImg->size[0] * ii] = greyImg
            [(i21 + partialTrueCount) + 452 * (i23 + ii)];
        }
      }

      detectFASTFeatures(b_greyImg, arrBinCandidates_pLocation,
                         arrBinCandidates_pMetric);

      //      arrBinCandidates = detectHarrisFeatures(subImg,'MinQuality',0.5,'FilterSize',5); 
      //  if found features in this bin
      guard1 = false;
      guard2 = false;
      if (FeaturePointsImpl_get_Count(arrBinCandidates_pLocation) > 0.0) {
        //  get the mask value of found feature candidates
        i21 = roundedCandLocation->size[0] * roundedCandLocation->size[1];
        roundedCandLocation->size[0] = arrBinCandidates_pLocation->size[0];
        roundedCandLocation->size[1] = 2;
        emxEnsureCapacity((emxArray__common *)roundedCandLocation, i21, (int)
                          sizeof(float));
        trueCount = arrBinCandidates_pLocation->size[0] *
          arrBinCandidates_pLocation->size[1];
        for (i21 = 0; i21 < trueCount; i21++) {
          roundedCandLocation->data[i21] = arrBinCandidates_pLocation->data[i21];
        }

        c_round(roundedCandLocation);
        trueCount = roundedCandLocation->size[0];
        i21 = varargin_1->size[0];
        varargin_1->size[0] = trueCount;
        emxEnsureCapacity((emxArray__common *)varargin_1, i21, (int)sizeof(float));
        for (i21 = 0; i21 < trueCount; i21++) {
          varargin_1->data[i21] = roundedCandLocation->data[i21 +
            roundedCandLocation->size[0]];
        }

        trueCount = roundedCandLocation->size[0];
        i21 = varargin_2->size[0];
        varargin_2->size[0] = trueCount;
        emxEnsureCapacity((emxArray__common *)varargin_2, i21, (int)sizeof(float));
        for (i21 = 0; i21 < trueCount; i21++) {
          varargin_2->data[i21] = roundedCandLocation->data[i21];
        }

        i21 = c_erodedGrid->size[0] * c_erodedGrid->size[1];
        c_erodedGrid->size[0] = b_loop_ub;
        c_erodedGrid->size[1] = idx;
        emxEnsureCapacity((emxArray__common *)c_erodedGrid, i21, (int)sizeof
                          (signed char));
        for (i21 = 0; i21 < idx; i21++) {
          for (i22 = 0; i22 < b_loop_ub; i22++) {
            c_erodedGrid->data[i22 + c_erodedGrid->size[0] * i21] =
              erodedGrid->data[i22 + erodedGrid->size[0] * i21];
          }
        }

        for (i21 = 0; i21 < 2; i21++) {
          siz[i21] = c_erodedGrid->size[i21];
        }

        i21 = iidx->size[0];
        iidx->size[0] = varargin_1->size[0];
        emxEnsureCapacity((emxArray__common *)iidx, i21, (int)sizeof(int));
        b_loop_ub = varargin_1->size[0];
        for (i21 = 0; i21 < b_loop_ub; i21++) {
          iidx->data[i21] = (int)varargin_1->data[i21] + siz[0] * ((int)
            varargin_2->data[i21] - 1);
        }

        i21 = r8->size[0];
        r8->size[0] = iidx->size[0];
        emxEnsureCapacity((emxArray__common *)r8, i21, (int)sizeof(double));
        b_loop_ub = iidx->size[0];
        for (i21 = 0; i21 < b_loop_ub; i21++) {
          r8->data[i21] = iidx->data[i21];
        }

        i21 = validCandidates->size[0];
        validCandidates->size[0] = iidx->size[0];
        emxEnsureCapacity((emxArray__common *)validCandidates, i21, (int)sizeof
                          (signed char));
        b_loop_ub = iidx->size[0];
        for (i21 = 0; i21 < b_loop_ub; i21++) {
          validCandidates->data[i21] = mask->data[iidx->data[i21] - 1];
        }

        if (!c_any(validCandidates)) {
          guard1 = true;
        } else {
          //  filter candidates with mask value
          //  select x strongest features in this bin.
          //  x should be smaller than numFeatureToFindBin and numFeatureToFind
          idx = r8->size[0] - 1;
          trueCount = 0;
          for (ii = 0; ii <= idx; ii++) {
            if (mask->data[(int)r8->data[ii] - 1] == 1) {
              trueCount++;
            }
          }

          i21 = iidx->size[0];
          iidx->size[0] = trueCount;
          emxEnsureCapacity((emxArray__common *)iidx, i21, (int)sizeof(int));
          partialTrueCount = 0;
          for (ii = 0; ii <= idx; ii++) {
            if (mask->data[(int)r8->data[ii] - 1] == 1) {
              iidx->data[partialTrueCount] = ii + 1;
              partialTrueCount++;
            }
          }

          i21 = varargin_1->size[0];
          varargin_1->size[0] = iidx->size[0];
          emxEnsureCapacity((emxArray__common *)varargin_1, i21, (int)sizeof
                            (float));
          b_loop_ub = iidx->size[0];
          for (i21 = 0; i21 < b_loop_ub; i21++) {
            varargin_1->data[i21] = arrBinCandidates_pMetric->data[iidx->
              data[i21] - 1];
          }

          eml_sort(varargin_1, iidx);
          i21 = ranking->size[0];
          ranking->size[0] = iidx->size[0];
          emxEnsureCapacity((emxArray__common *)ranking, i21, (int)sizeof(double));
          b_loop_ub = iidx->size[0];
          for (i21 = 0; i21 < b_loop_ub; i21++) {
            ranking->data[i21] = iidx->data[i21];
          }

          idx = r8->size[0] - 1;
          trueCount = 0;
          for (ii = 0; ii <= idx; ii++) {
            if (mask->data[(int)r8->data[ii] - 1] == 1) {
              trueCount++;
            }
          }

          i21 = iidx->size[0];
          iidx->size[0] = trueCount;
          emxEnsureCapacity((emxArray__common *)iidx, i21, (int)sizeof(int));
          partialTrueCount = 0;
          for (ii = 0; ii <= idx; ii++) {
            if (mask->data[(int)r8->data[ii] - 1] == 1) {
              iidx->data[partialTrueCount] = ii + 1;
              partialTrueCount++;
            }
          }

          b_varargin_1[0] = ceil(numFeatureToFindBin);
          b_varargin_1[1] = numFeatureToFind;
          b_varargin_1[2] = iidx->size[0];
          idx = 1;
          mtmp = b_varargin_1[0];
          if (rtIsNaN(b_varargin_1[0])) {
            idx = 2;
            mtmp = b_varargin_1[1];
          }

          while (idx + 1 < 4) {
            if (b_varargin_1[idx] < mtmp) {
              mtmp = b_varargin_1[idx];
            }

            idx++;
          }

          if (1.0 > mtmp) {
            b_loop_ub = 0;
          } else {
            b_loop_ub = (int)mtmp;
          }

          idx = r8->size[0] - 1;
          trueCount = 0;
          for (ii = 0; ii <= idx; ii++) {
            if (mask->data[(int)r8->data[ii] - 1] == 1) {
              trueCount++;
            }
          }

          i21 = iidx->size[0];
          iidx->size[0] = trueCount;
          emxEnsureCapacity((emxArray__common *)iidx, i21, (int)sizeof(int));
          partialTrueCount = 0;
          for (ii = 0; ii <= idx; ii++) {
            if (mask->data[(int)r8->data[ii] - 1] == 1) {
              iidx->data[partialTrueCount] = ii + 1;
              partialTrueCount++;
            }
          }

          i21 = b_arrBinCandidates_pLocation->size[0] *
            b_arrBinCandidates_pLocation->size[1];
          b_arrBinCandidates_pLocation->size[0] = iidx->size[0];
          b_arrBinCandidates_pLocation->size[1] = 2;
          emxEnsureCapacity((emxArray__common *)b_arrBinCandidates_pLocation,
                            i21, (int)sizeof(float));
          for (i21 = 0; i21 < 2; i21++) {
            idx = iidx->size[0];
            for (i22 = 0; i22 < idx; i22++) {
              b_arrBinCandidates_pLocation->data[i22 +
                b_arrBinCandidates_pLocation->size[0] * i21] =
                arrBinCandidates_pLocation->data[(iidx->data[i22] +
                arrBinCandidates_pLocation->size[0] * i21) - 1];
            }
          }

          i21 = roundedCandLocation->size[0] * roundedCandLocation->size[1];
          roundedCandLocation->size[0] = b_loop_ub;
          roundedCandLocation->size[1] = 2;
          emxEnsureCapacity((emxArray__common *)roundedCandLocation, i21, (int)
                            sizeof(float));
          for (i21 = 0; i21 < 2; i21++) {
            for (i22 = 0; i22 < b_loop_ub; i22++) {
              roundedCandLocation->data[i22 + roundedCandLocation->size[0] * i21]
                = b_arrBinCandidates_pLocation->data[((int)ranking->data[i22] +
                b_arrBinCandidates_pLocation->size[0] * i21) - 1];
            }
          }

          //  add offset due to subimg
          b_repmat(structBin->data[(int)featureCounts->data[i] - 1].offset,
                   (double)b_loop_ub, r9);
          i21 = roundedCandLocation->size[0] * roundedCandLocation->size[1];
          roundedCandLocation->size[1] = 2;
          emxEnsureCapacity((emxArray__common *)roundedCandLocation, i21, (int)
                            sizeof(float));
          idx = roundedCandLocation->size[0];
          ii = roundedCandLocation->size[1];
          b_loop_ub = idx * ii;
          for (i21 = 0; i21 < b_loop_ub; i21++) {
            roundedCandLocation->data[i21] += (float)r9->data[i21];
          }

          //  update temporary counters
          numFeatureFound += (double)roundedCandLocation->size[0];
          numFeatureToFind = (double)numDeadFeature - numFeatureFound;
          if (numFeatureFound_old + 1.0 > numFeatureFound) {
            i21 = 0;
          } else {
            i21 = (int)(numFeatureFound_old + 1.0) - 1;
          }

          for (i22 = 0; i22 < 2; i22++) {
            b_loop_ub = roundedCandLocation->size[0];
            for (i23 = 0; i23 < b_loop_ub; i23++) {
              features_tmp[(i21 + i23) + (i22 << 5)] = roundedCandLocation->
                data[i23 + roundedCandLocation->size[0] * i22];
            }
          }

          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        if (numFeatureToFind <= 0.0) {
          exitg1 = true;
        } else {
          numFeatureToFindBin = numFeatureToFind / (numBins - numCheckedBin);
          guard1 = true;
        }
      }

      if (guard1) {
        i++;
      }
    }

    emxFree_real32_T(&b_arrBinCandidates_pLocation);
    emxFree_int8_T(&c_erodedGrid);
    emxFree_int8_T(&r10);
    emxFree_int8_T(&b_erodedGrid);
    emxFree_uint8_T(&b_greyImg);
    emxFree_real32_T(&varargin_2);
    emxFree_real32_T(&varargin_1);
    emxFree_int32_T(&iidx);
    emxFree_real_T(&r9);
    emxFree_real_T(&r8);
    emxFree_real_T(&ranking);
    emxFree_int8_T(&validCandidates);
    emxFree_real32_T(&roundedCandLocation);
    emxFree_real32_T(&arrBinCandidates_pMetric);
    emxFree_real32_T(&arrBinCandidates_pLocation);
    emxFree_int8_T(&mask);
    emxFree_int8_T(&erodedGrid);
    emxFree_real_T(&featureCounts);
    b_emxFree_struct_T(&structBin);

    // % output
    if (numFeatureFound > 0.0) {
      all(features_tmp, bv0);
      trueCount = 0;
      for (i = 0; i < 32; i++) {
        if (bv0[i]) {
          trueCount++;
        }
      }

      partialTrueCount = 0;
      for (i = 0; i < 32; i++) {
        if (bv0[i]) {
          ii_data[partialTrueCount] = (signed char)(i + 1);
          partialTrueCount++;
        }
      }

      for (i21 = 0; i21 < 2; i21++) {
        for (i22 = 0; i22 < trueCount; i22++) {
          features_tmp_data[i22 + trueCount * i21] = features_tmp[(ii_data[i22]
            + (i21 << 5)) - 1];
        }
      }

      //  set flag and save features
      if (numFeatureFound < numDeadFeature) {
        notEnough = true;
        for (i21 = 0; i21 < 2; i21++) {
          for (i22 = 0; i22 < trueCount; i22++) {
            features[(idxDeadFeatures_data[i22] + (i21 << 5)) - 1] =
              features_tmp_data[i22 + trueCount * i21];
          }
        }

        loop_ub = (int)numFeatureFound;
        for (i21 = 0; i21 < loop_ub; i21++) {
          b_tmp_data[i21] = idxDeadFeatures_data[i21];
        }

        loop_ub = (int)numFeatureFound;
        for (i21 = 0; i21 < loop_ub; i21++) {
          featStatus[b_tmp_data[i21] - 1] = 2.0;
        }
      } else {
        notEnough = false;
        for (i21 = 0; i21 < 2; i21++) {
          for (i22 = 0; i22 < trueCount; i22++) {
            features[(idxDeadFeatures_data[i22] + (i21 << 5)) - 1] =
              features_tmp_data[i22 + trueCount * i21];
          }
        }

        for (i21 = 0; i21 < loop_ub; i21++) {
          ii_data[i21] = idxDeadFeatures_data[i21];
        }

        for (i21 = 0; i21 < loop_ub; i21++) {
          featStatus[ii_data[i21] - 1] = 2.0;
        }
      }
    } else {
      //  if haven't found feature at all, return original
      notEnough = true;
      memcpy(&featStatus[0], &featStatus_old[0], sizeof(double) << 5);
    }
  }

  emxFree_boolean_T(&deactivatedAnchor);
  return notEnough;
}

//
// Arguments    : const b_struct_T a
//                double varargin_1
//                emxArray_b_struct_T *b
// Return Type  : void
//
static void repmat(const b_struct_T a, double varargin_1, emxArray_b_struct_T *b)
{
  int i13;
  int loop_ub;
  i13 = b->size[0];
  b->size[0] = (int)varargin_1;
  emxEnsureCapacity_struct_T(b, i13);
  loop_ub = (int)varargin_1;
  for (i13 = 0; i13 < loop_ub; i13++) {
    emxCopyStruct_struct_T(&b->data[i13], &a);
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
// Arguments    : float u
// Return Type  : float
//
static float rt_roundf_snf(float u)
{
  float y;
  if ((real32_T)fabs((real_T)u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floor((real_T)(u + 0.5F));
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (real32_T)ceil((real_T)(u - 0.5F));
    }
  } else {
    y = u;
  }

  return y;
}

//
// STEREOMATCHING finds correspondences in the left image given points in the
// right image. Steoro images should be rectified first.
//  pointsR Nx2 Array
//  minDisp Nx1 Array
//  maxDisp Nx1 Array
//  shift scalar
// % parse parameters
// Arguments    : const unsigned char imgR[332672]
//                const unsigned char imgL[332672]
//                const double pointsR_data[]
//                const int pointsR_size[2]
//                double minDisp_data[]
//                int minDisp_size[1]
//                double maxDisp_data[]
//                int maxDisp_size[1]
//                double pointsL_data[]
//                int pointsL_size[2]
//                boolean_T status_data[]
//                int status_size[1]
// Return Type  : void
//
static void stereoRight2Left(const unsigned char imgR[332672], const unsigned
  char imgL[332672], const double pointsR_data[], const int pointsR_size[2],
  double minDisp_data[], int minDisp_size[1], double maxDisp_data[], int
  maxDisp_size[1], double pointsL_data[], int pointsL_size[2], boolean_T
  status_data[], int status_size[1])
{
  static double b_imgR[332672];
  static double b_imgL[332672];
  int i6;
  int outsize_idx_0;
  int counter;
  emxArray_real_T *b_template;
  emxArray_real_T *roi;
  emxArray_real_T *ssd;
  emxArray_real_T *a;
  emxArray_real_T *c_imgL;
  emxArray_real_T *c_imgR;
  double posx;
  double posy;
  double boundT[4];
  int i7;
  int i8;
  int i9;
  double b[4];
  double boundR[4];
  int i10;
  int nrows;
  int ix;
  int ixstart;
  double m;
  int k;
  int loop_ub;
  double b_m[2];
  boolean_T p;
  int32_T exitg2;
  double sz[2];
  double s;
  boolean_T exitg1;
  for (i6 = 0; i6 < 332672; i6++) {
    b_imgR[i6] = (double)imgR[i6] / 255.0;
    b_imgL[i6] = (double)imgL[i6] / 255.0;
  }

  //  default values
  b_round(maxDisp_data, maxDisp_size);
  b_round(minDisp_data, minDisp_size);

  // % preset output
  status_size[0] = pointsR_size[0];
  outsize_idx_0 = pointsR_size[0];
  for (i6 = 0; i6 < outsize_idx_0; i6++) {
    status_data[i6] = true;
  }

  pointsL_size[0] = pointsR_size[0];
  pointsL_size[1] = 2;
  outsize_idx_0 = pointsR_size[0] * pointsR_size[1];
  for (i6 = 0; i6 < outsize_idx_0; i6++) {
    pointsL_data[i6] = pointsR_data[i6];
  }

  // % Matching by convolving along a stripe around epipolar line
  //  extract template around points then extract stripe in other image around
  //  epipolar line
  //  template = zeros(winSize*2+1,winSize*2+1);
  //  roi = zeros(2*(shift+winSize)+1,2*winSize+maxDisp-minDisp+1);
  //  match = repmat(struct('template',template,'roi',roi),numPoints,1);
  counter = 0;
  b_emxInit_real_T(&b_template, 2);
  b_emxInit_real_T(&roi, 2);
  b_emxInit_real_T(&ssd, 2);
  emxInit_real_T(&a, 1);
  b_emxInit_real_T(&c_imgL, 2);
  b_emxInit_real_T(&c_imgR, 2);
  while (counter <= pointsR_size[0] - 1) {
    posx = rt_roundd_snf(pointsR_data[counter]);
    posy = rt_roundd_snf(pointsR_data[counter + pointsR_size[0]]);
    boundT[0] = posy - 2.0;
    boundT[1] = posy + 2.0;
    boundT[2] = posx - 2.0;
    boundT[3] = posx + 2.0;
    boundWindow(boundT);
    if (boundT[0] > boundT[1]) {
      i6 = 1;
      i7 = 1;
    } else {
      i6 = (int)boundT[0];
      i7 = (int)boundT[1] + 1;
    }

    if (boundT[2] > boundT[3]) {
      i8 = 1;
      i9 = 1;
    } else {
      i8 = (int)boundT[2];
      i9 = (int)boundT[3] + 1;
    }

    // mxn
    b[0] = (posy - 1.0) - 2.0;
    b[1] = (posy + 1.0) + 2.0;
    b[2] = (posx + minDisp_data[counter]) - 2.0;
    b[3] = (posx + maxDisp_data[counter]) + 2.0;
    for (i10 = 0; i10 < 4; i10++) {
      boundR[i10] = b[i10];
    }

    if ((posy - 1.0) - 2.0 <= 0.0) {
      // top
      boundR[0] = 1.0;
    }

    if ((posy + 1.0) + 2.0 > 452.0) {
      // bottom
      boundR[1] = 452.0;
    }

    if (b[2] <= 0.0) {
      // left
      boundR[2] = 1.0;
    }

    if (b[3] > 736.0) {
      // right
      boundR[3] = 736.0;
    }

    if (boundR[0] > boundR[1]) {
      i10 = 0;
      nrows = 0;
    } else {
      i10 = (int)boundR[0] - 1;
      nrows = (int)boundR[1];
    }

    if (boundR[2] > boundR[3]) {
      ix = 0;
      ixstart = 0;
    } else {
      ix = (int)boundR[2] - 1;
      ixstart = (int)boundR[3];
    }

    // mmxnn
    m = (boundT[1] - boundT[0]) + 1.0;

    //  vectorize
    k = c_imgL->size[0] * c_imgL->size[1];
    c_imgL->size[0] = nrows - i10;
    c_imgL->size[1] = ixstart - ix;
    emxEnsureCapacity((emxArray__common *)c_imgL, k, (int)sizeof(double));
    outsize_idx_0 = ixstart - ix;
    for (ixstart = 0; ixstart < outsize_idx_0; ixstart++) {
      loop_ub = nrows - i10;
      for (k = 0; k < loop_ub; k++) {
        c_imgL->data[k + c_imgL->size[0] * ixstart] = b_imgL[(i10 + k) + 452 *
          (ix + ixstart)];
      }
    }

    b_m[0] = m;
    b_m[1] = (boundT[3] - boundT[2]) + 1.0;
    im2col(c_imgL, b_m, roi);
    i10 = c_imgR->size[0] * c_imgR->size[1];
    c_imgR->size[0] = i7 - i6;
    c_imgR->size[1] = i9 - i8;
    emxEnsureCapacity((emxArray__common *)c_imgR, i10, (int)sizeof(double));
    outsize_idx_0 = i9 - i8;
    for (i10 = 0; i10 < outsize_idx_0; i10++) {
      loop_ub = i7 - i6;
      for (nrows = 0; nrows < loop_ub; nrows++) {
        c_imgR->data[nrows + c_imgR->size[0] * i10] = b_imgR[((i6 + nrows) + 452
          * ((i8 + i10) - 1)) - 1];
      }
    }

    outsize_idx_0 = (i7 - i6) * (i9 - i8);
    i10 = a->size[0];
    a->size[0] = outsize_idx_0;
    emxEnsureCapacity((emxArray__common *)a, i10, (int)sizeof(double));
    for (i10 = 0; i10 < outsize_idx_0; i10++) {
      a->data[i10] = c_imgR->data[i10];
    }

    outsize_idx_0 = (i7 - i6) * (i9 - i8);
    ix = roi->size[1];
    nrows = roi->size[1];
    i10 = b_template->size[0] * b_template->size[1];
    b_template->size[0] = outsize_idx_0;
    b_template->size[1] = nrows;
    emxEnsureCapacity((emxArray__common *)b_template, i10, (int)sizeof(double));
    if (!(a->size[0] == 0)) {
      if (outsize_idx_0 == 0) {
        p = true;
      } else if (ix == 0) {
        p = true;
      } else {
        p = false;
      }

      if (!p) {
        nrows = a->size[0];
        ixstart = 1;
        do {
          exitg2 = 0;
          ix = roi->size[1];
          if (ixstart <= ix) {
            outsize_idx_0 = (ixstart - 1) * nrows;
            for (k = 1; k <= nrows; k++) {
              b_template->data[(outsize_idx_0 + k) - 1] = a->data[k - 1];
            }

            ixstart++;
          } else {
            exitg2 = 1;
          }
        } while (exitg2 == 0);
      }
    }

    //  if roi is smaller than template block set track failure
    if ((roi->size[0] == 0) || (roi->size[1] == 0)) {
      status_data[counter] = false;
    } else {
      //  SSD
      i10 = b_template->size[0] * b_template->size[1];
      emxEnsureCapacity((emxArray__common *)b_template, i10, (int)sizeof(double));
      outsize_idx_0 = b_template->size[0];
      nrows = b_template->size[1];
      outsize_idx_0 *= nrows;
      for (i10 = 0; i10 < outsize_idx_0; i10++) {
        b_template->data[i10] = (b_template->data[i10] - roi->data[i10]) *
          (b_template->data[i10] - roi->data[i10]);
      }

      for (i10 = 0; i10 < 2; i10++) {
        sz[i10] = b_template->size[i10];
      }

      i10 = ssd->size[0] * ssd->size[1];
      ssd->size[0] = 1;
      ssd->size[1] = (int)sz[1];
      emxEnsureCapacity((emxArray__common *)ssd, i10, (int)sizeof(double));
      ix = -1;
      outsize_idx_0 = -1;
      for (nrows = 1; nrows <= b_template->size[1]; nrows++) {
        ixstart = ix + 1;
        ix++;
        s = b_template->data[ixstart];
        for (k = 2; k <= b_template->size[0]; k++) {
          ix++;
          s += b_template->data[ix];
        }

        outsize_idx_0++;
        ssd->data[outsize_idx_0] = s;
      }

      ixstart = 1;
      outsize_idx_0 = ssd->size[1];
      s = ssd->data[0];
      nrows = 0;
      if (ssd->size[1] > 1) {
        if (rtIsNaN(ssd->data[0])) {
          ix = 2;
          exitg1 = false;
          while ((!exitg1) && (ix <= outsize_idx_0)) {
            ixstart = ix;
            if (!rtIsNaN(ssd->data[ix - 1])) {
              s = ssd->data[ix - 1];
              nrows = ix - 1;
              exitg1 = true;
            } else {
              ix++;
            }
          }
        }

        if (ixstart < ssd->size[1]) {
          while (ixstart + 1 <= outsize_idx_0) {
            if (ssd->data[ixstart] < s) {
              s = ssd->data[ixstart];
              nrows = ixstart;
            }

            ixstart++;
          }
        }
      }

      //  failure if error larger than threshold
      if (s / (double)((i7 - i6) * (i9 - i8)) > 0.1) {
        status_data[counter] = false;
      } else {
        sz[0] = (((boundR[1] - boundR[0]) + 1.0) - m) + 1.0;
        outsize_idx_0 = div_s32(nrows, (int)sz[0]);
        pointsL_data[counter] = (((double)(outsize_idx_0 + 1) + boundR[2]) - 1.0)
          + (posx - boundT[2]);
        pointsL_data[counter + pointsL_size[0]] = (((double)((nrows -
          outsize_idx_0 * (int)sz[0]) + 1) + boundR[0]) - 1.0) + (posy - boundT
          [0]);
        status_data[counter] = true;
      }
    }

    counter++;
  }

  emxFree_real_T(&c_imgR);
  emxFree_real_T(&c_imgL);
  emxFree_real_T(&a);
  emxFree_real_T(&ssd);
  emxFree_real_T(&roi);
  emxFree_real_T(&b_template);
}

//
// Arguments    : const signed char x_data[]
//                const int x_size[1]
// Return Type  : double
//
static double sum(const signed char x_data[], const int x_size[1])
{
  double y;
  int k;
  if (x_size[0] == 0) {
    y = 0.0;
  } else {
    y = x_data[0];
    for (k = 2; k <= x_size[0]; k++) {
      y += (double)x_data[k - 1];
    }
  }

  return y;
}

//
// POINTEXTRACTION extracts feature points from stereo images
//  ImGrayR_r rectified right stereo image
//  ImGrayR_l rectified left stereo image
//  updateVect Nx1 array showing the status of points
//  numAnchors scalar number of anchors
//  binx number of bins in x direction (horizontal)
//  binx number of bins in y direction (vertical)
//  border width of border where a feature is considered invalid
//  minDistance quaosi distance of between features in one grid
//  h_u_apo estimated point location in the right image
// Arguments    : const unsigned char ImGrayR_r[332672]
//                const unsigned char ImGrayR_l[332672]
//                double updateVect[32]
//                double numAnchors
//                double binx
//                double biny
//                double border
//                double minDistance
//                boolean_T useInitGuess
//                const double h_u_apo_data[]
//                const int h_u_apo_size[1]
//                double pts_r_arr[64]
//                double pts_l_arr[64]
//                double useDisparity[32]
//                double z_all[96]
// Return Type  : void
//
void pointextraction(const unsigned char ImGrayR_r[332672], const unsigned char
                     ImGrayR_l[332672], double updateVect[32], double numAnchors,
                     double binx, double biny, double border, double minDistance,
                     boolean_T, const double [], const int [1], double
                     pts_r_arr[64], double pts_l_arr[64], double useDisparity[32],
                     double z_all[96])
{
  double flag;
  double threshold;
  double updateVect_r[32];
  int idx;
  int ii_data[32];
  int ii;
  boolean_T exitg4;
  boolean_T guard4 = false;
  int trueCount;
  int i;
  signed char active_idx_data[32];
  double b_pts_l_arr[64];
  double tmp_data[64];
  int tmp_size[2];
  double b_tmp_data[32];
  int maxDisp_size[1];
  emxArray_real_T *pts_r_arr_tmp;
  int status_r_l_size[1];
  boolean_T status_r_l_data[32];
  int pts_l_arr_tmp_size[2];
  double pts_l_arr_tmp_data[64];
  int b_tmp_size[2];
  boolean_T exitg3;
  boolean_T guard3 = false;
  emxArray_boolean_T *trackStatus;
  emxArray_real32_T *b_pts_r_arr_tmp;
  emxArray_boolean_T *b_trackStatus;
  emxArray_boolean_T *r5;
  emxArray_boolean_T *r6;
  emxArray_int32_T *r7;
  boolean_T exitg2;
  boolean_T guard2 = false;
  double maxDisp_data[32];
  int minDisp_size[1];
  double minDisp_data[32];
  double dispVect[64];
  boolean_T exitg1;
  boolean_T guard1 = false;

  // % compute initial feature
  //  get anchor pose and estimated feature depth to compute initial feature
  //  location
  // % initialization
  if (!initialized_not_empty) {
    initialized_not_empty = true;
    memset(&prevPoints[0], 0, sizeof(double) << 6);
    memset(&prevDisp[0], 0, sizeof(double) << 5);

    //  initialize features in the right image
    initFeatures(ImGrayR_r, border, binx, biny, minDistance, prevPoints,
                 updateVect_r, &threshold, &flag);
    idx = 0;
    ii = 1;
    exitg4 = false;
    while ((!exitg4) && (ii < 33)) {
      guard4 = false;
      if (updateVect_r[ii - 1] != 0.0) {
        idx++;
        ii_data[idx - 1] = ii;
        if (idx >= 32) {
          exitg4 = true;
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

    if (1 > idx) {
      ii = 0;
    } else {
      ii = idx;
    }

    if (1 > idx) {
      trueCount = 0;
    } else {
      trueCount = idx;
    }

    for (i = 0; i < ii; i++) {
      active_idx_data[i] = (signed char)ii_data[i];
    }

    //  find correspondences in the left image
    memset(&b_pts_l_arr[0], 0, sizeof(double) << 6);
    tmp_size[0] = trueCount;
    tmp_size[1] = 2;
    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < ii; idx++) {
        tmp_data[idx + trueCount * i] = prevPoints[(active_idx_data[idx] + (i <<
          5)) - 1];
      }
    }

    maxDisp_size[0] = trueCount;
    for (i = 0; i < trueCount; i++) {
      b_tmp_data[i] = 0.0;
    }

    emxInit_real_T(&pts_r_arr_tmp, 1);
    i = pts_r_arr_tmp->size[0];
    pts_r_arr_tmp->size[0] = trueCount;
    emxEnsureCapacity((emxArray__common *)pts_r_arr_tmp, i, (int)sizeof(double));
    for (i = 0; i < trueCount; i++) {
      pts_r_arr_tmp->data[i] = 40.0;
    }

    stereoRight2Left(ImGrayR_r, ImGrayR_l, tmp_data, tmp_size, b_tmp_data,
                     maxDisp_size, pts_r_arr_tmp->data, pts_r_arr_tmp->size,
                     pts_l_arr_tmp_data, pts_l_arr_tmp_size, status_r_l_data,
                     status_r_l_size);
    idx = status_r_l_size[0] - 1;
    trueCount = 0;
    emxFree_real_T(&pts_r_arr_tmp);
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] > 0) {
        trueCount++;
      }
    }

    ii = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] > 0) {
        ii_data[ii] = i + 1;
        ii++;
      }
    }

    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < trueCount; idx++) {
        b_pts_l_arr[(active_idx_data[ii_data[idx] - 1] + (i << 5)) - 1] =
          pts_l_arr_tmp_data[(ii_data[idx] + pts_l_arr_tmp_size[0] * i) - 1];
      }
    }

    //  reset features in right image if no correspondances are found in the
    //  left image
    idx = status_r_l_size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (!status_r_l_data[i]) {
        trueCount++;
      }
    }

    ii = 0;
    for (i = 0; i <= idx; i++) {
      if (!status_r_l_data[i]) {
        ii_data[ii] = i + 1;
        ii++;
      }
    }

    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < trueCount; idx++) {
        prevPoints[(active_idx_data[ii_data[idx] - 1] + (i << 5)) - 1] = 0.0;
      }
    }

    idx = status_r_l_size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] > 0) {
        trueCount++;
      }
    }

    ii = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] > 0) {
        ii_data[ii] = i + 1;
        ii++;
      }
    }

    for (i = 0; i < trueCount; i++) {
      updateVect[active_idx_data[ii_data[i] - 1] - 1] = 2.0;
    }

    //  initialized KLT tracker
    PointTracker_PointTracker(&pointTracker);
    idx = status_r_l_size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] > 0) {
        trueCount++;
      }
    }

    ii = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] > 0) {
        ii_data[ii] = i + 1;
        ii++;
      }
    }

    b_tmp_size[0] = trueCount;
    b_tmp_size[1] = 2;
    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < trueCount; idx++) {
        tmp_data[idx + trueCount * i] = prevPoints[(active_idx_data[ii_data[idx]
          - 1] + (i << 5)) - 1];
      }
    }

    PointTracker_initialize(&pointTracker, tmp_data, b_tmp_size, ImGrayR_r);

    // % tracking and refill
  } else {
    //  set to track current visible points in right image for next iteration
    trueCount = 0;
    for (i = 0; i < 32; i++) {
      if (updateVect[i] > 0.0) {
        trueCount++;
      }
    }

    ii = 0;
    for (i = 0; i < 32; i++) {
      if (updateVect[i] > 0.0) {
        ii_data[ii] = i + 1;
        ii++;
      }
    }

    tmp_size[0] = trueCount;
    tmp_size[1] = 2;
    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < trueCount; idx++) {
        tmp_data[idx + trueCount * i] = prevPoints[(ii_data[idx] + (i << 5)) - 1];
      }
    }

    PointTracker_setPoints(&pointTracker, tmp_data, tmp_size);

    //  indices of points that are to be tracked
    idx = 0;
    ii = 1;
    exitg3 = false;
    while ((!exitg3) && (ii < 33)) {
      guard3 = false;
      if (updateVect[ii - 1] != 0.0) {
        idx++;
        ii_data[idx - 1] = ii;
        if (idx >= 32) {
          exitg3 = true;
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
      ii = 0;
    } else {
      ii = idx;
    }

    for (i = 0; i < ii; i++) {
      active_idx_data[i] = (signed char)ii_data[i];
    }

    b_emxInit_boolean_T(&trackStatus, 1);
    emxInit_real32_T(&b_pts_r_arr_tmp, 2);
    b_emxInit_boolean_T(&b_trackStatus, 1);

    //  KLT Tracker tracks active points
    SystemCore_step(&pointTracker, ImGrayR_r, b_pts_r_arr_tmp, b_trackStatus);
    i = trackStatus->size[0];
    trackStatus->size[0] = b_trackStatus->size[0];
    emxEnsureCapacity((emxArray__common *)trackStatus, i, (int)sizeof(boolean_T));
    ii = b_trackStatus->size[0];
    for (i = 0; i < ii; i++) {
      trackStatus->data[i] = b_trackStatus->data[i];
    }

    emxFree_boolean_T(&b_trackStatus);
    b_emxInit_boolean_T(&r5, 1);

    //  deactivate points too close to the border
    ii = b_pts_r_arr_tmp->size[0];
    i = r5->size[0];
    r5->size[0] = ii;
    emxEnsureCapacity((emxArray__common *)r5, i, (int)sizeof(boolean_T));
    for (i = 0; i < ii; i++) {
      r5->data[i] = ((b_pts_r_arr_tmp->data[i] < border) ||
                     (b_pts_r_arr_tmp->data[i] > 736.0 - border) ||
                     (b_pts_r_arr_tmp->data[i + b_pts_r_arr_tmp->size[0]] <
                      border));
    }

    b_emxInit_boolean_T(&r6, 1);
    ii = b_pts_r_arr_tmp->size[0];
    i = r6->size[0];
    r6->size[0] = ii;
    emxEnsureCapacity((emxArray__common *)r6, i, (int)sizeof(boolean_T));
    for (i = 0; i < ii; i++) {
      r6->data[i] = (b_pts_r_arr_tmp->data[i + b_pts_r_arr_tmp->size[0]] > 452.0
                     - border);
    }

    idx = r5->size[0];
    for (i = 0; i < idx; i++) {
      if (r5->data[i] || r6->data[i]) {
        trackStatus->data[i] = false;
      }
    }

    emxFree_boolean_T(&r6);
    emxFree_boolean_T(&r5);
    emxInit_int32_T(&r7, 1);

    //  save to pts_r_arr and updateVect
    idx = trackStatus->size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (!trackStatus->data[i]) {
        trueCount++;
      }
    }

    i = r7->size[0];
    r7->size[0] = trueCount;
    emxEnsureCapacity((emxArray__common *)r7, i, (int)sizeof(int));
    ii = 0;
    for (i = 0; i <= idx; i++) {
      if (!trackStatus->data[i]) {
        r7->data[ii] = i + 1;
        ii++;
      }
    }

    ii = r7->size[0];
    for (i = 0; i < ii; i++) {
      updateVect[active_idx_data[r7->data[i] - 1] - 1] = 0.0;
    }

    memset(&prevPoints[0], 0, sizeof(double) << 6);
    idx = trackStatus->size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)trackStatus->data[i] > 0) {
        trueCount++;
      }
    }

    i = r7->size[0];
    r7->size[0] = trueCount;
    emxEnsureCapacity((emxArray__common *)r7, i, (int)sizeof(int));
    ii = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)trackStatus->data[i] > 0) {
        r7->data[ii] = i + 1;
        ii++;
      }
    }

    emxFree_boolean_T(&trackStatus);
    b_emxInit_real_T(&pts_r_arr_tmp, 2);
    i = pts_r_arr_tmp->size[0] * pts_r_arr_tmp->size[1];
    pts_r_arr_tmp->size[0] = r7->size[0];
    pts_r_arr_tmp->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)pts_r_arr_tmp, i, (int)sizeof(double));
    for (i = 0; i < 2; i++) {
      ii = r7->size[0];
      for (idx = 0; idx < ii; idx++) {
        pts_r_arr_tmp->data[idx + pts_r_arr_tmp->size[0] * i] =
          b_pts_r_arr_tmp->data[(r7->data[idx] + b_pts_r_arr_tmp->size[0] * i) -
          1];
      }
    }

    emxFree_real32_T(&b_pts_r_arr_tmp);
    ii = r7->size[0];
    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < ii; idx++) {
        prevPoints[(active_idx_data[r7->data[idx] - 1] + (i << 5)) - 1] =
          pts_r_arr_tmp->data[idx + ii * i];
      }
    }

    emxFree_real_T(&pts_r_arr_tmp);
    emxFree_int32_T(&r7);

    //  fill feature vector
    refillFeature(ImGrayR_r, prevPoints, updateVect, numAnchors, binx, biny,
                  border, minDistance);
    idx = 0;
    ii = 1;
    exitg2 = false;
    while ((!exitg2) && (ii < 33)) {
      guard2 = false;
      if (updateVect[ii - 1] != 0.0) {
        idx++;
        ii_data[idx - 1] = ii;
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
      ii = 0;
    } else {
      ii = idx;
    }

    if (1 > idx) {
      trueCount = 0;
    } else {
      trueCount = idx;
    }

    for (i = 0; i < ii; i++) {
      active_idx_data[i] = (signed char)ii_data[i];
    }

    //  indices of newly initialized features relative in active_idx
    //  use last disparity to limit search window (but do not use newly
    //  initialized ones)
    maxDisp_size[0] = trueCount;
    for (i = 0; i < trueCount; i++) {
      maxDisp_data[i] = prevDisp[active_idx_data[i] - 1] + 6.0;
    }

    minDisp_size[0] = trueCount;
    for (i = 0; i < trueCount; i++) {
      minDisp_data[i] = prevDisp[active_idx_data[i] - 1] - 6.0;
    }

    for (i = 0; i < trueCount; i++) {
      if (updateVect[active_idx_data[i] - 1] == 2.0) {
        maxDisp_data[i] = 40.0;
      }
    }

    for (i = 0; i < trueCount; i++) {
      if (updateVect[active_idx_data[i] - 1] == 2.0) {
        minDisp_data[i] = 0.0;
      }
    }

    //  find correspondences in the left image
    memset(&b_pts_l_arr[0], 0, sizeof(double) << 6);
    b_tmp_size[0] = trueCount;
    b_tmp_size[1] = 2;
    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < ii; idx++) {
        tmp_data[idx + trueCount * i] = prevPoints[(active_idx_data[idx] + (i <<
          5)) - 1];
      }
    }

    stereoRight2Left(ImGrayR_r, ImGrayR_l, tmp_data, b_tmp_size, minDisp_data,
                     minDisp_size, maxDisp_data, maxDisp_size,
                     pts_l_arr_tmp_data, pts_l_arr_tmp_size, status_r_l_data,
                     status_r_l_size);
    idx = status_r_l_size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] > 0) {
        trueCount++;
      }
    }

    ii = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] > 0) {
        ii_data[ii] = i + 1;
        ii++;
      }
    }

    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < trueCount; idx++) {
        b_pts_l_arr[(active_idx_data[ii_data[idx] - 1] + (i << 5)) - 1] =
          pts_l_arr_tmp_data[(ii_data[idx] + pts_l_arr_tmp_size[0] * i) - 1];
      }
    }

    idx = status_r_l_size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] <= 0) {
        trueCount++;
      }
    }

    ii = 0;
    for (i = 0; i <= idx; i++) {
      if ((int)status_r_l_data[i] <= 0) {
        ii_data[ii] = i + 1;
        ii++;
      }
    }

    for (i = 0; i < trueCount; i++) {
      updateVect[active_idx_data[ii_data[i] - 1] - 1] = 0.0;
    }

    //  reset features in right image if no correspondances are found in the
    //  left image
    trueCount = 0;
    for (i = 0; i < 32; i++) {
      if (updateVect[i] == 0.0) {
        trueCount++;
      }
    }

    ii = 0;
    for (i = 0; i < 32; i++) {
      if (updateVect[i] == 0.0) {
        ii_data[ii] = i + 1;
        ii++;
      }
    }

    for (i = 0; i < 2; i++) {
      for (idx = 0; idx < trueCount; idx++) {
        prevPoints[(ii_data[idx] + (i << 5)) - 1] = 0.0;
      }
    }
  }

  // % Disparity check and output
  //  compute the disparity Vector (should be only x component because images are rectified) 
  for (i = 0; i < 64; i++) {
    dispVect[i] = b_pts_l_arr[i] - prevPoints[i];
  }

  idx = 0;
  ii = 1;
  exitg1 = false;
  while ((!exitg1) && (ii < 33)) {
    guard1 = false;
    if (updateVect[ii - 1] != 0.0) {
      idx++;
      ii_data[idx - 1] = ii;
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
    trueCount = 0;
  } else {
    trueCount = idx;
  }

  if (1 > idx) {
    ii = 0;
  } else {
    ii = idx;
  }

  for (i = 0; i < ii; i++) {
    active_idx_data[i] = (signed char)ii_data[i];
  }

  memset(&useDisparity[0], 0, sizeof(double) << 5);
  memset(&z_all[0], 0, 96U * sizeof(double));
  for (ii = 0; ii < trueCount; ii++) {
    //  x coordinates in pixels
    //  y coordinates in pixels
    z_all[3 * active_idx_data[ii] - 3] = prevPoints[active_idx_data[ii] - 1];

    //  fill in the pixel measurements from the left?
    z_all[3 * active_idx_data[ii] - 2] = prevPoints[active_idx_data[ii] + 31];

    //  make a second outlier rejecton based on the dispairty (disparity
    //  should be disp=[d,0] because images are rectified
    if ((dispVect[active_idx_data[ii] - 1] > 1.7759) && (fabs
         (dispVect[active_idx_data[ii] + 31]) < 5.0) &&
        (dispVect[active_idx_data[ii] - 1] < 100.0)) {
      z_all[3 * active_idx_data[ii] - 1] = dispVect[active_idx_data[ii] - 1];

      //  if valid use disparity
      useDisparity[active_idx_data[ii] - 1] = 1.0;

      //  set use disparity vector to 1
    } else {
      z_all[3 * active_idx_data[ii] - 1] = -100.0;

      //  else use dummy value
      useDisparity[active_idx_data[ii] - 1] = 0.0;

      //  set vect. to 0
    }
  }

  //  save for next loop
  for (i = 0; i < 32; i++) {
    prevDisp[i] = dispVect[i];
    for (idx = 0; idx < 2; idx++) {
      pts_l_arr[idx + (i << 1)] = b_pts_l_arr[i + (idx << 5)];
      pts_r_arr[idx + (i << 1)] = prevPoints[i + (idx << 5)];
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void pointextraction_initialize()
{
  rt_InitInfAndNaN(8U);
  initialized_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void pointextraction_terminate()
{
  // (no terminate code required)
}

//
// File trailer for pointextraction.cpp
//
// [EOF]
//
