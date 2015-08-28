//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QuatFromRotJ.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 14:06:46
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "QuatFromRotJ.h"
#include <stdio.h>

// Function Definitions

//
// THIS IS OK, It is according to the NASA memo found
//  https://claraty.jpl.nasa.gov/man/software/development/conventions/standards_docs/unadopted/JPL_Quaternions_Breckenridge.pdf
// QUATFROMROTJ Get equivalent quaternion of the rotation matrix R
//    Returns a quaternion in JPL notation
//  The implementation is copied from qGetQ(R), but we are careful about the
//  ordering of the output vector
//  Q = [0;0;0;1];
//  % [r,c] = size( R );
//  % if( r ~= 3 || c ~= 3 )
//  %     error( 'R must be a 3x3 matrix\n\r' );
//  % end
//
//  if abs(det(R) - 1) > 1e-2
//      error('The provided matrix is not a valid rotation matrix. It must have det(R) = 1, but is %i.', det(R))
//  end
//  T = R(1,1)+R(2,2)+R(3,3);
//  pivot = zeros(4,1);
//  pivot(1) = sqrt(max(1+2*R(1,1)-T, eps))/2;
//  pivot(2) = sqrt(max(1+2*R(2,2)-T, eps))/2;
//  pivot(3) = sqrt(max(1+2*R(3,3)-T, eps))/2;
//  pivot(4) = sqrt(max(1+T, eps))/2;
//  if ~all(isreal(pivot))
//      error('complex')
//  end
//  index = round(find(pivot==max(pivot)));
//  if(index(1)==1)
//      Q(1) = sqrt(1+2*R(1,1)-T)/2;
//      Q(2) = (R(1,2)+R(2,1))/(4*(Q(1)));
//      Q(3) = (R(1,3)+R(3,1))/(4*(Q(1)));
//      Q(4) = (R(2,3)-R(3,2))/(4*(Q(1)));
//  elseif(index(1)==2)
//      Q(2) = sqrt(1+2*R(2,2)-T)/2;
//      Q(1) = (R(1,2)+R(2,1))/(4*Q(2));
//      Q(3) = (R(2,3)+R(3,2))/(4*Q(2));
//      Q(4) = (R(3,1)-R(1,3))/(4*Q(2));
//  elseif(index(1)==3)
//      Q(3) = sqrt(1+2*R(3,3)-T)/2;
//      Q(1) = (R(1,3)+R(3,1))/(4*Q(3));
//      Q(2) = (R(2,3)+R(3,2))/(4*Q(3));
//      Q(4) = (R(1,2)-R(2,1))/(4*Q(3));
//  elseif(index(1)==4)
//      Q(4) = sqrt(1+T)/2;
//      Q(1) = (R(2,3)-R(3,2))/(4*Q(4));
//      Q(2) = (R(3,1)-R(1,3))/(4*Q(4));
//      Q(3) = (R(1,2)-R(2,1))/(4*Q(4));
//  end
// Arguments    : const double R[9]
//                double Q[4]
// Return Type  : void
//
void QuatFromRotJ(const double R[9], double Q[4])
{
  double varargin_1[4];
  int ixstart;
  double mtmp;
  int itmp;
  int ix;
  boolean_T exitg1;

  // % speed optimization
  varargin_1[0] = (1.0 + R[0]) - (R[4] + R[8]);
  varargin_1[1] = (1.0 + R[4]) - (R[0] + R[8]);
  varargin_1[2] = (1.0 + R[8]) - (R[0] + R[4]);
  varargin_1[3] = 1.0 + ((R[0] + R[4]) + R[8]);
  ixstart = 1;
  mtmp = varargin_1[0];
  itmp = 1;
  if (rtIsNaN(varargin_1[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 5)) {
      ixstart = ix;
      if (!rtIsNaN(varargin_1[ix - 1])) {
        mtmp = varargin_1[ix - 1];
        itmp = ix;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 4) {
    while (ixstart + 1 < 5) {
      if (varargin_1[ixstart] > mtmp) {
        mtmp = varargin_1[ixstart];
        itmp = ixstart + 1;
      }

      ixstart++;
    }
  }

  if (itmp == 1) {
    Q[0] = sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[1] = (R[3] + R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[2] = (R[6] + R[2]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[3] = (R[7] - R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else if (itmp == 2) {
    Q[0] = (R[3] + R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[1] = sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[2] = (R[7] + R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[3] = (R[2] - R[6]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else if (itmp == 3) {
    Q[0] = (R[6] + R[2]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[1] = (R[7] + R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[2] = sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[3] = (R[3] - R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else {
    Q[0] = (R[7] - R[5]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[1] = (R[2] - R[6]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[2] = (R[3] - R[1]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[3] = sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0;
  }
}

//
// File trailer for QuatFromRotJ.cpp
//
// [EOF]
//
