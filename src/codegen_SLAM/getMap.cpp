//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getMap.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 25-Aug-2015 17:43:12
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
//                double numStatesPerAnchorxt
//                emxArray_real_T *map
//                emxArray_real_T *anchorInd
//                emxArray_real_T *featureAnchorInd
// Return Type  : void
//
void getMap(const emxArray_real_T *x, const emxArray_real_T *b_anchorFeatures,
            const emxArray_real_T *b_m_vect, double c_numTrackFeatures, double
            stateSize, double numStatesPerAnchorxt, emxArray_real_T *map,
            emxArray_real_T *anchorInd, emxArray_real_T *featureAnchorInd)
{
  int i0;
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
  i0 = map->size[0] * map->size[1];
  map->size[0] = 3;
  map->size[1] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)map, i0, (int)sizeof(double));
  ii = 3 * (int)c_numTrackFeatures;
  for (i0 = 0; i0 < ii; i0++) {
    map->data[i0] = rtNaN;
  }

  i0 = anchorInd->size[0];
  anchorInd->size[0] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)anchorInd, i0, (int)sizeof(double));
  ii = (int)c_numTrackFeatures;
  for (i0 = 0; i0 < ii; i0++) {
    anchorInd->data[i0] = 0.0;
  }

  i0 = featureAnchorInd->size[0];
  featureAnchorInd->size[0] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)featureAnchorInd, i0, (int)sizeof(double));
  ii = (int)c_numTrackFeatures;
  for (i0 = 0; i0 < ii; i0++) {
    featureAnchorInd->data[i0] = 0.0;
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

    for (i0 = 0; i0 < ii; i0++) {
      featureIdxVect_data[i0] = ii_data[i0];
    }

    //  the transpose prevents going into the loop if find returns empty
    for (ii = 0; ii < ii_size_idx_0; ii++) {
      if (b_anchorFeatures->data[(featureIdxVect_data[ii] +
           b_anchorFeatures->size[0] * anchorIdx) - 1] == 1.0) {
        //  if this is not a lost feature
        b_stateSize = stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
          numStatesPerAnchorxt;
        for (i0 = 0; i0 < 3; i0++) {
          anchorPos[i0] = x->data[(int)(b_stateSize + (1.0 + (double)i0)) - 1];
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
        for (i0 = 0; i0 < 3; i0++) {
          d0 = 0.0;
          for (idx = 0; idx < 3; idx++) {
            d0 += b_x[i0 + 3 * idx] * b_m_vect->data[idx + b_m_vect->size[0] *
              (featureIdxVect_data[ii] - 1)];
          }

          map->data[i0 + map->size[0] * (featureIdxVect_data[ii] - 1)] =
            anchorPos[i0] + d0 / b_stateSize;
        }

        anchorInd->data[featureIdxVect_data[ii] - 1] = 1.0 + (double)anchorIdx;
        featureAnchorInd->data[featureIdxVect_data[ii] - 1] = 1.0 + (double)ii;
      }
    }
  }
}

//
// File trailer for getMap.cpp
//
// [EOF]
//
