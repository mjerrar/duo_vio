//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getMap.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getMap.h"
#include "SLAM_emxutil.h"
#include <stdio.h>

// Function Definitions

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
//                double b_numStatesPerAnchorxt
//                emxArray_real_T *map
//                emxArray_real_T *anchorInd
//                emxArray_real_T *featureAnchorInd
// Return Type  : void
//
void getMap(const emxArray_real_T *x, const emxArray_real_T *b_anchorFeatures,
            const emxArray_real_T *b_m_vect, double c_numTrackFeatures, double
            stateSize, double b_numStatesPerAnchorxt, emxArray_real_T *map,
            emxArray_real_T *anchorInd, emxArray_real_T *featureAnchorInd)
{
  int i5;
  int ii;
  int anchorIdx;
  emxArray_real_T *featureIdxVect;
  emxArray_int32_T *b_ii;
  int idx;
  int i6;
  boolean_T exitg1;
  boolean_T guard1 = false;
  double b_stateSize;
  double anchorPos[3];
  int b_featureIdxVect;
  double b_x[9];
  double d0;
  i5 = map->size[0] * map->size[1];
  map->size[0] = 3;
  map->size[1] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)map, i5, (int)sizeof(double));
  ii = 3 * (int)c_numTrackFeatures;
  for (i5 = 0; i5 < ii; i5++) {
    map->data[i5] = rtNaN;
  }

  i5 = anchorInd->size[0];
  anchorInd->size[0] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)anchorInd, i5, (int)sizeof(double));
  ii = (int)c_numTrackFeatures;
  for (i5 = 0; i5 < ii; i5++) {
    anchorInd->data[i5] = 0.0;
  }

  i5 = featureAnchorInd->size[0];
  featureAnchorInd->size[0] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)featureAnchorInd, i5, (int)sizeof(double));
  ii = (int)c_numTrackFeatures;
  for (i5 = 0; i5 < ii; i5++) {
    featureAnchorInd->data[i5] = 0.0;
  }

  anchorIdx = 0;
  b_emxInit_real_T(&featureIdxVect, 2);
  emxInit_int32_T(&b_ii, 1);
  while (anchorIdx <= b_anchorFeatures->size[1] - 1) {
    i5 = b_anchorFeatures->size[0];
    idx = 0;
    i6 = b_anchorFeatures->size[0];
    ii = b_ii->size[0];
    b_ii->size[0] = i6;
    emxEnsureCapacity((emxArray__common *)b_ii, ii, (int)sizeof(int));
    ii = 1;
    exitg1 = false;
    while ((!exitg1) && (ii <= i5)) {
      guard1 = false;
      if (b_anchorFeatures->data[(ii + b_anchorFeatures->size[0] * anchorIdx) -
          1] != 0.0) {
        idx++;
        b_ii->data[idx - 1] = ii;
        if (idx >= i5) {
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

    i5 = b_anchorFeatures->size[0];
    if (i5 == 1) {
      if (idx == 0) {
        i5 = b_ii->size[0];
        b_ii->size[0] = 0;
        emxEnsureCapacity((emxArray__common *)b_ii, i5, (int)sizeof(int));
      }
    } else {
      i5 = b_ii->size[0];
      if (1 > idx) {
        b_ii->size[0] = 0;
      } else {
        b_ii->size[0] = idx;
      }

      emxEnsureCapacity((emxArray__common *)b_ii, i5, (int)sizeof(int));
    }

    i5 = featureIdxVect->size[0] * featureIdxVect->size[1];
    featureIdxVect->size[0] = 1;
    featureIdxVect->size[1] = b_ii->size[0];
    emxEnsureCapacity((emxArray__common *)featureIdxVect, i5, (int)sizeof(double));
    ii = b_ii->size[0];
    for (i5 = 0; i5 < ii; i5++) {
      featureIdxVect->data[featureIdxVect->size[0] * i5] = b_ii->data[i5];
    }

    //  the transpose prevents going into the loop if find returns empty
    for (ii = 0; ii < featureIdxVect->size[1]; ii++) {
      if (b_anchorFeatures->data[((int)featureIdxVect->data[ii] +
           b_anchorFeatures->size[0] * anchorIdx) - 1] == 1.0) {
        //  if this is not a lost feature
        b_stateSize = stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt;
        for (i5 = 0; i5 < 3; i5++) {
          anchorPos[i5] = x->data[(int)(b_stateSize + (1.0 + (double)i5)) - 1];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        idx = (int)featureIdxVect->data[ii];
        b_featureIdxVect = (int)featureIdxVect->data[ii];
        b_x[0] = ((x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize + ((1.0
          + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] -
                   x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          b_numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize + ((1.0
          + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1]) -
                  x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                    b_numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)
                  ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                    b_numStatesPerAnchorxt) + 6.0) - 1]) + x->data[(int)
          ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 7.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        b_x[1] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) -
                        1] + x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)
                        ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[2] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) -
                        1] - x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)
                        ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[3] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) -
                        1] - x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)
                        ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[4] = ((-(x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0)
          * b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1])
                   + x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0)
          * b_numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1])
                  - x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0)
                    * b_numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)
                  ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                    b_numStatesPerAnchorxt) + 6.0) - 1]) + x->data[(int)
          ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 7.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        b_x[5] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) -
                        1] + x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)
                        ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[6] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) -
                        1] + x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)
                        ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[7] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize
          + ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 6.0) -
                        1] - x->data[(int)((stateSize + ((1.0 + (double)
          anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)
                        ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                          b_numStatesPerAnchorxt) + 7.0) - 1]);
        b_x[8] = ((-(x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0)
          * b_numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 4.0) - 1])
                   - x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0)
          * b_numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 5.0) - 1])
                  + x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0)
                    * b_numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)
                  ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                    b_numStatesPerAnchorxt) + 6.0) - 1]) + x->data[(int)
          ((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
            b_numStatesPerAnchorxt) + 7.0) - 1] * x->data[(int)((stateSize +
          ((1.0 + (double)anchorIdx) - 1.0) * b_numStatesPerAnchorxt) + 7.0) - 1];
        b_stateSize = x->data[(int)(((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * b_numStatesPerAnchorxt) + 7.0) + (1.0 + (double)ii)) - 1];
        for (i5 = 0; i5 < 3; i5++) {
          d0 = 0.0;
          for (i6 = 0; i6 < 3; i6++) {
            d0 += b_x[i5 + 3 * i6] * b_m_vect->data[i6 + b_m_vect->size[0] *
              (b_featureIdxVect - 1)];
          }

          map->data[i5 + map->size[0] * (idx - 1)] = anchorPos[i5] + d0 /
            b_stateSize;
        }

        anchorInd->data[(int)featureIdxVect->data[ii] - 1] = 1.0 + (double)
          anchorIdx;
        featureAnchorInd->data[(int)featureIdxVect->data[ii] - 1] = 1.0 +
          (double)ii;
      }
    }

    anchorIdx++;
  }

  emxFree_int32_T(&b_ii);
  emxFree_real_T(&featureIdxVect);
}

//
// File trailer for getMap.cpp
//
// [EOF]
//
