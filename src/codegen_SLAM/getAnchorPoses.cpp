//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getAnchorPoses.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getAnchorPoses.h"
#include "QuatFromRotJ.h"
#include "SLAM_emxutil.h"
#include "SLAM_data.h"
#include <ros/console.h>

// Function Definitions

//
// getAnchorPoses Get the anchor poses in the world frame
// Arguments    : const double xt_origin_pos[3]
//                const double xt_origin_att[4]
//                const emxArray_b_struct_T *xt_anchor_states
//                emxArray_c_struct_T *anchor_poses
// Return Type  : void
//
void getAnchorPoses(const double xt_origin_pos[3], const double xt_origin_att[4],
                    const emxArray_b_struct_T *xt_anchor_states,
                    emxArray_c_struct_T *anchor_poses)
{
  double R_ow[9];
  double varargin_1;
  int i1;
  int loop_ub;
  static const g_struct_T r0 = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 } };

  int i2;
  double b_xt_anchor_states[9];
  double c_xt_anchor_states[9];
  int i3;

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
  varargin_1 = numAnchors;
  i1 = anchor_poses->size[0] * anchor_poses->size[1];
  anchor_poses->size[0] = (int)numAnchors;
  anchor_poses->size[1] = (int)numAnchors;
  emxEnsureCapacity((emxArray__common *)anchor_poses, i1, (int)sizeof(g_struct_T));
  loop_ub = (int)varargin_1 * (int)varargin_1;
  for (i1 = 0; i1 < loop_ub; i1++) {
    anchor_poses->data[i1] = r0;
  }

  for (loop_ub = 0; loop_ub < (int)numAnchors; loop_ub++) {
    for (i1 = 0; i1 < 3; i1++) {
      varargin_1 = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        varargin_1 += R_ow[i2 + 3 * i1] * xt_anchor_states->data[loop_ub].pos[i2];
      }

      anchor_poses->data[loop_ub].pos[i1] = xt_origin_pos[i1] + varargin_1;
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    c_xt_anchor_states[0] = ((xt_anchor_states->data[loop_ub].att[0] *
      xt_anchor_states->data[loop_ub].att[0] - xt_anchor_states->data[loop_ub].
      att[1] * xt_anchor_states->data[loop_ub].att[1]) - xt_anchor_states->
      data[loop_ub].att[2] * xt_anchor_states->data[loop_ub].att[2]) +
      xt_anchor_states->data[loop_ub].att[3] * xt_anchor_states->data[loop_ub].
      att[3];
    c_xt_anchor_states[3] = 2.0 * (xt_anchor_states->data[loop_ub].att[0] *
      xt_anchor_states->data[loop_ub].att[1] + xt_anchor_states->data[loop_ub].
      att[2] * xt_anchor_states->data[loop_ub].att[3]);
    c_xt_anchor_states[6] = 2.0 * (xt_anchor_states->data[loop_ub].att[0] *
      xt_anchor_states->data[loop_ub].att[2] - xt_anchor_states->data[loop_ub].
      att[1] * xt_anchor_states->data[loop_ub].att[3]);
    c_xt_anchor_states[1] = 2.0 * (xt_anchor_states->data[loop_ub].att[0] *
      xt_anchor_states->data[loop_ub].att[1] - xt_anchor_states->data[loop_ub].
      att[2] * xt_anchor_states->data[loop_ub].att[3]);
    c_xt_anchor_states[4] = ((-(xt_anchor_states->data[loop_ub].att[0] *
      xt_anchor_states->data[loop_ub].att[0]) + xt_anchor_states->data[loop_ub].
      att[1] * xt_anchor_states->data[loop_ub].att[1]) - xt_anchor_states->
      data[loop_ub].att[2] * xt_anchor_states->data[loop_ub].att[2]) +
      xt_anchor_states->data[loop_ub].att[3] * xt_anchor_states->data[loop_ub].
      att[3];
    c_xt_anchor_states[7] = 2.0 * (xt_anchor_states->data[loop_ub].att[1] *
      xt_anchor_states->data[loop_ub].att[2] + xt_anchor_states->data[loop_ub].
      att[0] * xt_anchor_states->data[loop_ub].att[3]);
    c_xt_anchor_states[2] = 2.0 * (xt_anchor_states->data[loop_ub].att[0] *
      xt_anchor_states->data[loop_ub].att[2] + xt_anchor_states->data[loop_ub].
      att[1] * xt_anchor_states->data[loop_ub].att[3]);
    c_xt_anchor_states[5] = 2.0 * (xt_anchor_states->data[loop_ub].att[1] *
      xt_anchor_states->data[loop_ub].att[2] - xt_anchor_states->data[loop_ub].
      att[0] * xt_anchor_states->data[loop_ub].att[3]);
    c_xt_anchor_states[8] = ((-(xt_anchor_states->data[loop_ub].att[0] *
      xt_anchor_states->data[loop_ub].att[0]) - xt_anchor_states->data[loop_ub].
      att[1] * xt_anchor_states->data[loop_ub].att[1]) + xt_anchor_states->
      data[loop_ub].att[2] * xt_anchor_states->data[loop_ub].att[2]) +
      xt_anchor_states->data[loop_ub].att[3] * xt_anchor_states->data[loop_ub].
      att[3];
    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        b_xt_anchor_states[i1 + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          b_xt_anchor_states[i1 + 3 * i2] += c_xt_anchor_states[i1 + 3 * i3] *
            R_ow[i3 + 3 * i2];
        }
      }
    }

    QuatFromRotJ(b_xt_anchor_states, anchor_poses->data[loop_ub].att);
  }
}

//
// File trailer for getAnchorPoses.cpp
//
// [EOF]
//
