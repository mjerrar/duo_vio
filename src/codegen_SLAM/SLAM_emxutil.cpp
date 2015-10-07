//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_emxutil.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Declarations
static void b_emxCopyMatrix_real_T(double dst[4], const double src[4]);
static void b_emxFree_struct_T(emxArray_struct_T **pEmxArray);
static void b_emxInit_struct_T(emxArray_struct_T **pEmxArray, int
  b_numDimensions);
static void c_emxCopyMatrix_real_T(double dst[6], const double src[6]);
static void emxCopyMatrix_real_T(double dst[3], const double src[3]);
static void emxCopy_struct_T(emxArray_struct_T **dst, emxArray_struct_T * const *
  src);
static void emxExpand_struct_T(emxArray_b_struct_T *emxArray, int fromIndex, int
  toIndex);
static void emxFree_struct_T(emxArray_b_struct_T **pEmxArray);
static void emxInit_struct_T(emxArray_b_struct_T **pEmxArray, int
  b_numDimensions);
static void emxTrim_struct_T(emxArray_b_struct_T *emxArray, int fromIndex, int
  toIndex);

// Function Definitions

//
// Arguments    : double dst[4]
//                const double src[4]
// Return Type  : void
//
static void b_emxCopyMatrix_real_T(double dst[4], const double src[4])
{
  int i;
  for (i = 0; i < 4; i++) {
    dst[i] = src[i];
  }
}

//
// Arguments    : emxArray_struct_T **pEmxArray
// Return Type  : void
//
static void b_emxFree_struct_T(emxArray_struct_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_struct_T *)NULL) {
    if (((*pEmxArray)->data != (d_struct_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_struct_T *)NULL;
  }
}

//
// Arguments    : emxArray_struct_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void b_emxInit_struct_T(emxArray_struct_T **pEmxArray, int
  b_numDimensions)
{
  emxArray_struct_T *emxArray;
  int i;
  *pEmxArray = (emxArray_struct_T *)malloc(sizeof(emxArray_struct_T));
  emxArray = *pEmxArray;
  emxArray->data = (d_struct_T *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : double dst[6]
//                const double src[6]
// Return Type  : void
//
static void c_emxCopyMatrix_real_T(double dst[6], const double src[6])
{
  int i;
  for (i = 0; i < 6; i++) {
    dst[i] = src[i];
  }
}

//
// Arguments    : double dst[3]
//                const double src[3]
// Return Type  : void
//
static void emxCopyMatrix_real_T(double dst[3], const double src[3])
{
  int i;
  for (i = 0; i < 3; i++) {
    dst[i] = src[i];
  }
}

//
// Arguments    : emxArray_struct_T **dst
//                emxArray_struct_T * const *src
// Return Type  : void
//
static void emxCopy_struct_T(emxArray_struct_T **dst, emxArray_struct_T * const *
  src)
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

  emxEnsureCapacity((emxArray__common *)*dst, numElDst, (int)sizeof(d_struct_T));
  for (i = 0; i < numElSrc; i++) {
    (*dst)->data[i] = (*src)->data[i];
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
    b_emxInitStruct_struct_T(&emxArray->data[i]);
  }
}

//
// Arguments    : emxArray_b_struct_T **pEmxArray
// Return Type  : void
//
static void emxFree_struct_T(emxArray_b_struct_T **pEmxArray)
{
  int numEl;
  int i;
  if (*pEmxArray != (emxArray_b_struct_T *)NULL) {
    if ((*pEmxArray)->data != (e_struct_T *)NULL) {
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
// Arguments    : emxArray_b_struct_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_struct_T(emxArray_b_struct_T **pEmxArray, int
  b_numDimensions)
{
  emxArray_b_struct_T *emxArray;
  int i;
  *pEmxArray = (emxArray_b_struct_T *)malloc(sizeof(emxArray_b_struct_T));
  emxArray = *pEmxArray;
  emxArray->data = (e_struct_T *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
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
// Arguments    : f_struct_T *pStruct
// Return Type  : void
//
void b_emxFreeStruct_struct_T(f_struct_T *pStruct)
{
  emxFree_struct_T(&pStruct->anchor_states);
}

//
// Arguments    : e_struct_T *pStruct
// Return Type  : void
//
void b_emxInitStruct_struct_T(e_struct_T *pStruct)
{
  b_emxInit_struct_T(&pStruct->feature_states, 1);
}

//
// Arguments    : emxArray_int32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
void b_emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions)
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
void b_emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions)
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
// Arguments    : emxArray_c_struct_T **pEmxArray
// Return Type  : void
//
void c_emxFree_struct_T(emxArray_c_struct_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_c_struct_T *)NULL) {
    if (((*pEmxArray)->data != (g_struct_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_c_struct_T *)NULL;
  }
}

//
// Arguments    : emxArray_c_struct_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
void c_emxInit_struct_T(emxArray_c_struct_T **pEmxArray, int b_numDimensions)
{
  emxArray_c_struct_T *emxArray;
  int i;
  *pEmxArray = (emxArray_c_struct_T *)malloc(sizeof(emxArray_c_struct_T));
  emxArray = *pEmxArray;
  emxArray->data = (g_struct_T *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : e_struct_T *dst
//                const e_struct_T *src
// Return Type  : void
//
void emxCopyStruct_struct_T(e_struct_T *dst, const e_struct_T *src)
{
  emxCopyMatrix_real_T(dst->pos, src->pos);
  b_emxCopyMatrix_real_T(dst->att, src->att);
  c_emxCopyMatrix_real_T(dst->P_idx, src->P_idx);
  emxCopy_struct_T(&dst->feature_states, &src->feature_states);
}

//
// Arguments    : emxArray__common *emxArray
//                int oldNumel
//                int elementSize
// Return Type  : void
//
void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int elementSize)
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
void emxEnsureCapacity_struct_T(emxArray_b_struct_T *emxArray, int oldNumel)
{
  int elementSize;
  int newNumel;
  int i;
  void *newData;
  elementSize = (int)sizeof(e_struct_T);
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

    emxArray->data = (e_struct_T *)newData;
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
// Arguments    : e_struct_T *pStruct
// Return Type  : void
//
void emxFreeStruct_struct_T(e_struct_T *pStruct)
{
  b_emxFree_struct_T(&pStruct->feature_states);
}

//
// Arguments    : emxArray_AnchorPose **pEmxArray
// Return Type  : void
//
void emxFree_AnchorPose(emxArray_AnchorPose **pEmxArray)
{
  if (*pEmxArray != (emxArray_AnchorPose *)NULL) {
    if (((*pEmxArray)->data != (AnchorPose *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_AnchorPose *)NULL;
  }
}

//
// Arguments    : emxArray_boolean_T **pEmxArray
// Return Type  : void
//
void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
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
void emxFree_int32_T(emxArray_int32_T **pEmxArray)
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
void emxFree_real_T(emxArray_real_T **pEmxArray)
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
// Arguments    : f_struct_T *pStruct
// Return Type  : void
//
void emxInitStruct_struct_T(f_struct_T *pStruct)
{
  emxInit_struct_T(&pStruct->anchor_states, 1);
}

//
// Arguments    : emxArray_AnchorPose **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
void emxInit_AnchorPose(emxArray_AnchorPose **pEmxArray, int b_numDimensions)
{
  emxArray_AnchorPose *emxArray;
  int i;
  *pEmxArray = (emxArray_AnchorPose *)malloc(sizeof(emxArray_AnchorPose));
  emxArray = *pEmxArray;
  emxArray->data = (AnchorPose *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_boolean_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int b_numDimensions)
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
void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions)
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
void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions)
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
// File trailer for SLAM_emxutil.cpp
//
// [EOF]
//
