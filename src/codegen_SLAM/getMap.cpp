//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getMap.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Aug-2015 20:50:24
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getMap.h"
#include "SLAM_emxutil.h"
#include <ros/console.h>
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
  int i4;
  int ii;
  int anchorIdx;
  int idx;
  signed char ii_data[24];
  boolean_T exitg1;
  boolean_T guard1 = false;
  int ii_size_idx_0;
  signed char featureIdxVect_data[24];
  double b_stateSize;
  double anchorPos[3];
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
  double anchorRot[9];
  i4 = map->size[0] * map->size[1];
  map->size[0] = 3;
  map->size[1] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)map, i4, (int)sizeof(double));
  ii = 3 * (int)c_numTrackFeatures;
  for (i4 = 0; i4 < ii; i4++) {
    map->data[i4] = rtNaN;
  }

  i4 = anchorInd->size[0];
  anchorInd->size[0] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)anchorInd, i4, (int)sizeof(double));
  ii = (int)c_numTrackFeatures;
  for (i4 = 0; i4 < ii; i4++) {
    anchorInd->data[i4] = 0.0;
  }

  i4 = featureAnchorInd->size[0];
  featureAnchorInd->size[0] = (int)c_numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)featureAnchorInd, i4, (int)sizeof(double));
  ii = (int)c_numTrackFeatures;
  for (i4 = 0; i4 < ii; i4++) {
    featureAnchorInd->data[i4] = 0.0;
  }

  for (anchorIdx = 0; anchorIdx < b_anchorFeatures->size[1]; anchorIdx++) {
    idx = 0;
    ii = 1;
    exitg1 = false;
    while ((!exitg1) && (ii < 25)) {
      guard1 = false;
      if (b_anchorFeatures->data[(ii + b_anchorFeatures->size[0] * anchorIdx) -
          1] != 0.0) {
        idx++;
        ii_data[idx - 1] = (signed char)ii;
        if (idx >= 24) {
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

    for (i4 = 0; i4 < ii; i4++) {
      featureIdxVect_data[i4] = ii_data[i4];
    }

    //  the transpose prevents going into the loop if find returns empty
    b_stateSize = stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
      numStatesPerAnchorxt;
    for (i4 = 0; i4 < 3; i4++) {
      anchorPos[i4] = x->data[(int)(b_stateSize + (1.0 + (double)i4)) - 1];
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_stateSize = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
      numStatesPerAnchorxt) + 4.0) - 1];
    a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                       numStatesPerAnchorxt) + 5.0) - 1];
    b_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 6.0) - 1];
    c_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 7.0) - 1];
    d_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 4.0) - 1];
    e_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 5.0) - 1];
    f_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 6.0) - 1];
    g_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 7.0) - 1];
    h_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 4.0) - 1];
    i_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 5.0) - 1];
    j_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 6.0) - 1];
    k_a = x->data[(int)((stateSize + ((1.0 + (double)anchorIdx) - 1.0) *
                         numStatesPerAnchorxt) + 7.0) - 1];
    anchorRot[0] = ((b_stateSize * b_stateSize - a * a) - b_a * b_a) + c_a * c_a;
    anchorRot[3] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 5.0) - 1] +
                          x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
    anchorRot[6] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 6.0) - 1] -
                          x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
    anchorRot[1] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 5.0) - 1] -
                          x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 6.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
    anchorRot[4] = ((-(d_a * d_a) + e_a * e_a) - f_a * f_a) + g_a * g_a;
    anchorRot[7] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 6.0) - 1] +
                          x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
    anchorRot[2] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 6.0) - 1] +
                          x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
    anchorRot[5] = 2.0 * (x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 5.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 6.0) - 1] -
                          x->data[(int)((stateSize + ((1.0 + (double)anchorIdx)
      - 1.0) * numStatesPerAnchorxt) + 4.0) - 1] * x->data[(int)((stateSize +
      ((1.0 + (double)anchorIdx) - 1.0) * numStatesPerAnchorxt) + 7.0) - 1]);
    anchorRot[8] = ((-(h_a * h_a) - i_a * i_a) + j_a * j_a) + k_a * k_a;
    for (ii = 0; ii < ii_size_idx_0; ii++) {
      if (b_anchorFeatures->data[(featureIdxVect_data[ii] +
           b_anchorFeatures->size[0] * anchorIdx) - 1] == 1.0) {
        //  if this is not a lost feature
        b_stateSize = x->data[(int)(((stateSize + ((1.0 + (double)anchorIdx) -
          1.0) * numStatesPerAnchorxt) + 7.0) + (1.0 + (double)ii)) - 1];
        for (i4 = 0; i4 < 3; i4++) {
          a = 0.0;
          for (idx = 0; idx < 3; idx++) {
            a += anchorRot[idx + 3 * i4] * b_m_vect->data[idx + b_m_vect->size[0]
              * (featureIdxVect_data[ii] - 1)];
          }

          map->data[i4 + map->size[0] * (featureIdxVect_data[ii] - 1)] =
            anchorPos[i4] + a / b_stateSize;
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
