//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getMap.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getMap.h"
#include "SLAM_emxutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

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
//                const emxArray_b_struct_T *xt_anchor_states
//                emxArray_real_T *b_map
// Return Type  : void
//
void getMap(const double xt_origin_pos[3], const double xt_origin_att[4], const
            emxArray_b_struct_T *xt_anchor_states, emxArray_real_T *b_map)
{
  double R_ow[9];
  int i4;
  int loop_ub;
  int anchorIdx;
  double d0;
  int i5;
  double anchorPos[3];
  double b_xt_anchor_states[9];
  double anchorRot[9];
  int c_xt_anchor_states;

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
  i4 = b_map->size[0] * b_map->size[1];
  b_map->size[0] = 3;
  b_map->size[1] = (int)numTrackFeatures;
  emxEnsureCapacity((emxArray__common *)b_map, i4, (int)sizeof(double));
  loop_ub = 3 * (int)numTrackFeatures;
  for (i4 = 0; i4 < loop_ub; i4++) {
    b_map->data[i4] = rtNaN;
  }

  for (anchorIdx = 0; anchorIdx < (int)numAnchors; anchorIdx++) {
    for (i4 = 0; i4 < 3; i4++) {
      d0 = 0.0;
      for (i5 = 0; i5 < 3; i5++) {
        d0 += R_ow[i5 + 3 * i4] * xt_anchor_states->data[anchorIdx].pos[i5];
      }

      anchorPos[i4] = xt_origin_pos[i4] + d0;
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_xt_anchor_states[0] = ((xt_anchor_states->data[anchorIdx].att[0] *
      xt_anchor_states->data[anchorIdx].att[0] - xt_anchor_states->
      data[anchorIdx].att[1] * xt_anchor_states->data[anchorIdx].att[1]) -
      xt_anchor_states->data[anchorIdx].att[2] * xt_anchor_states->
      data[anchorIdx].att[2]) + xt_anchor_states->data[anchorIdx].att[3] *
      xt_anchor_states->data[anchorIdx].att[3];
    b_xt_anchor_states[3] = 2.0 * (xt_anchor_states->data[anchorIdx].att[0] *
      xt_anchor_states->data[anchorIdx].att[1] + xt_anchor_states->
      data[anchorIdx].att[2] * xt_anchor_states->data[anchorIdx].att[3]);
    b_xt_anchor_states[6] = 2.0 * (xt_anchor_states->data[anchorIdx].att[0] *
      xt_anchor_states->data[anchorIdx].att[2] - xt_anchor_states->
      data[anchorIdx].att[1] * xt_anchor_states->data[anchorIdx].att[3]);
    b_xt_anchor_states[1] = 2.0 * (xt_anchor_states->data[anchorIdx].att[0] *
      xt_anchor_states->data[anchorIdx].att[1] - xt_anchor_states->
      data[anchorIdx].att[2] * xt_anchor_states->data[anchorIdx].att[3]);
    b_xt_anchor_states[4] = ((-(xt_anchor_states->data[anchorIdx].att[0] *
      xt_anchor_states->data[anchorIdx].att[0]) + xt_anchor_states->
      data[anchorIdx].att[1] * xt_anchor_states->data[anchorIdx].att[1]) -
      xt_anchor_states->data[anchorIdx].att[2] * xt_anchor_states->
      data[anchorIdx].att[2]) + xt_anchor_states->data[anchorIdx].att[3] *
      xt_anchor_states->data[anchorIdx].att[3];
    b_xt_anchor_states[7] = 2.0 * (xt_anchor_states->data[anchorIdx].att[1] *
      xt_anchor_states->data[anchorIdx].att[2] + xt_anchor_states->
      data[anchorIdx].att[0] * xt_anchor_states->data[anchorIdx].att[3]);
    b_xt_anchor_states[2] = 2.0 * (xt_anchor_states->data[anchorIdx].att[0] *
      xt_anchor_states->data[anchorIdx].att[2] + xt_anchor_states->
      data[anchorIdx].att[1] * xt_anchor_states->data[anchorIdx].att[3]);
    b_xt_anchor_states[5] = 2.0 * (xt_anchor_states->data[anchorIdx].att[1] *
      xt_anchor_states->data[anchorIdx].att[2] - xt_anchor_states->
      data[anchorIdx].att[0] * xt_anchor_states->data[anchorIdx].att[3]);
    b_xt_anchor_states[8] = ((-(xt_anchor_states->data[anchorIdx].att[0] *
      xt_anchor_states->data[anchorIdx].att[0]) - xt_anchor_states->
      data[anchorIdx].att[1] * xt_anchor_states->data[anchorIdx].att[1]) +
      xt_anchor_states->data[anchorIdx].att[2] * xt_anchor_states->
      data[anchorIdx].att[2]) + xt_anchor_states->data[anchorIdx].att[3] *
      xt_anchor_states->data[anchorIdx].att[3];
    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        anchorRot[i4 + 3 * i5] = 0.0;
        for (loop_ub = 0; loop_ub < 3; loop_ub++) {
          anchorRot[i4 + 3 * i5] += b_xt_anchor_states[i4 + 3 * loop_ub] *
            R_ow[loop_ub + 3 * i5];
        }
      }
    }

    for (loop_ub = 0; loop_ub < (int)numPointsPerAnchor; loop_ub++) {
      if (xt_anchor_states->data[anchorIdx].feature_states->data[loop_ub].status
          != 0.0) {
        c_xt_anchor_states = (int)xt_anchor_states->data[anchorIdx].
          feature_states->data[loop_ub].status_idx;
        for (i4 = 0; i4 < 3; i4++) {
          d0 = 0.0;
          for (i5 = 0; i5 < 3; i5++) {
            d0 += anchorRot[i5 + 3 * i4] * xt_anchor_states->data[anchorIdx].
              feature_states->data[loop_ub].m[i5];
          }

          b_map->data[i4 + b_map->size[0] * (c_xt_anchor_states - 1)] =
            anchorPos[i4] + d0 / xt_anchor_states->data[anchorIdx].
            feature_states->data[loop_ub].inverse_depth;
        }
      }
    }
  }
}

//
// File trailer for getMap.cpp
//
// [EOF]
//
