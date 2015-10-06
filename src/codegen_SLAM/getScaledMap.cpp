//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getScaledMap.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getScaledMap.h"
#include "SLAM_emxutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

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
// Arguments    : f_struct_T *b_xt
// Return Type  : void
//
void getScaledMap(f_struct_T *b_xt)
{
  double R_cw[9];
  int anchorIdx;
  e_struct_T c_xt;
  e_struct_T d_xt;
  e_struct_T e_xt;
  double anchorPos[3];
  int i34;
  double anchorRot[9];
  int featureIdx;
  double f_xt[3];
  double d8;
  int i35;

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
  anchorIdx = 0;
  b_emxInitStruct_struct_T(&c_xt);
  b_emxInitStruct_struct_T(&d_xt);
  b_emxInitStruct_struct_T(&e_xt);
  while (anchorIdx <= (int)numAnchors - 1) {
    for (i34 = 0; i34 < 3; i34++) {
      anchorPos[i34] = b_xt->anchor_states->data[anchorIdx].pos[i34];
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    anchorRot[0] = ((b_xt->anchor_states->data[anchorIdx].att[0] *
                     b_xt->anchor_states->data[anchorIdx].att[0] -
                     b_xt->anchor_states->data[anchorIdx].att[1] *
                     b_xt->anchor_states->data[anchorIdx].att[1]) -
                    b_xt->anchor_states->data[anchorIdx].att[2] *
                    b_xt->anchor_states->data[anchorIdx].att[2]) +
      b_xt->anchor_states->data[anchorIdx].att[3] * b_xt->anchor_states->
      data[anchorIdx].att[3];
    anchorRot[3] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[0] *
                          b_xt->anchor_states->data[anchorIdx].att[1] +
                          b_xt->anchor_states->data[anchorIdx].att[2] *
                          b_xt->anchor_states->data[anchorIdx].att[3]);
    anchorRot[6] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[0] *
                          b_xt->anchor_states->data[anchorIdx].att[2] -
                          b_xt->anchor_states->data[anchorIdx].att[1] *
                          b_xt->anchor_states->data[anchorIdx].att[3]);
    anchorRot[1] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[0] *
                          b_xt->anchor_states->data[anchorIdx].att[1] -
                          b_xt->anchor_states->data[anchorIdx].att[2] *
                          b_xt->anchor_states->data[anchorIdx].att[3]);
    anchorRot[4] = ((-(b_xt->anchor_states->data[anchorIdx].att[0] *
                       b_xt->anchor_states->data[anchorIdx].att[0]) +
                     b_xt->anchor_states->data[anchorIdx].att[1] *
                     b_xt->anchor_states->data[anchorIdx].att[1]) -
                    b_xt->anchor_states->data[anchorIdx].att[2] *
                    b_xt->anchor_states->data[anchorIdx].att[2]) +
      b_xt->anchor_states->data[anchorIdx].att[3] * b_xt->anchor_states->
      data[anchorIdx].att[3];
    anchorRot[7] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[1] *
                          b_xt->anchor_states->data[anchorIdx].att[2] +
                          b_xt->anchor_states->data[anchorIdx].att[0] *
                          b_xt->anchor_states->data[anchorIdx].att[3]);
    anchorRot[2] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[0] *
                          b_xt->anchor_states->data[anchorIdx].att[2] +
                          b_xt->anchor_states->data[anchorIdx].att[1] *
                          b_xt->anchor_states->data[anchorIdx].att[3]);
    anchorRot[5] = 2.0 * (b_xt->anchor_states->data[anchorIdx].att[1] *
                          b_xt->anchor_states->data[anchorIdx].att[2] -
                          b_xt->anchor_states->data[anchorIdx].att[0] *
                          b_xt->anchor_states->data[anchorIdx].att[3]);
    anchorRot[8] = ((-(b_xt->anchor_states->data[anchorIdx].att[0] *
                       b_xt->anchor_states->data[anchorIdx].att[0]) -
                     b_xt->anchor_states->data[anchorIdx].att[1] *
                     b_xt->anchor_states->data[anchorIdx].att[1]) +
                    b_xt->anchor_states->data[anchorIdx].att[2] *
                    b_xt->anchor_states->data[anchorIdx].att[2]) +
      b_xt->anchor_states->data[anchorIdx].att[3] * b_xt->anchor_states->
      data[anchorIdx].att[3];
    for (featureIdx = 0; featureIdx < (int)numPointsPerAnchor; featureIdx++) {
      if (b_xt->anchor_states->data[anchorIdx].feature_states->data[featureIdx].
          status != 0.0) {
        emxCopyStruct_struct_T(&c_xt, &b_xt->anchor_states->data[anchorIdx]);
        emxCopyStruct_struct_T(&d_xt, &b_xt->anchor_states->data[anchorIdx]);
        emxCopyStruct_struct_T(&e_xt, &b_xt->anchor_states->data[anchorIdx]);
        for (i34 = 0; i34 < 3; i34++) {
          d8 = 0.0;
          for (i35 = 0; i35 < 3; i35++) {
            d8 += anchorRot[i35 + 3 * i34] * d_xt.feature_states->
              data[featureIdx].m[i35];
          }

          f_xt[i34] = (c_xt.feature_states->data[featureIdx].inverse_depth *
                       anchorPos[i34] + d8) - b_xt->robot_state.pos[i34] *
            e_xt.feature_states->data[featureIdx].inverse_depth;
        }

        for (i34 = 0; i34 < 3; i34++) {
          b_xt->anchor_states->data[anchorIdx].feature_states->data[featureIdx].
            scaled_map_point[i34] = 0.0;
          for (i35 = 0; i35 < 3; i35++) {
            b_xt->anchor_states->data[anchorIdx].feature_states->data[featureIdx]
              .scaled_map_point[i34] += R_cw[i34 + 3 * i35] * f_xt[i35];
          }
        }
      }
    }

    anchorIdx++;
  }

  emxFreeStruct_struct_T(&e_xt);
  emxFreeStruct_struct_T(&d_xt);
  emxFreeStruct_struct_T(&c_xt);
}

//
// File trailer for getScaledMap.cpp
//
// [EOF]
//
