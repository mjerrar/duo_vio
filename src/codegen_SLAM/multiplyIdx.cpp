//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: multiplyIdx.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "multiplyIdx.h"
#include <ros/console.h>

// Function Definitions

//
// # coder
// multiplyIdx Create a longer index array from idx
//    for example:
//    idx_mult = multiplyIdx([1 4 5], 2)
//    idx_mult = [1 2 7 8 9 10]
// Arguments    : const double idx_data[]
//                const int idx_size[1]
//                double idx_mult_data[]
//                int idx_mult_size[1]
// Return Type  : void
//
void multiplyIdx(const double idx_data[], const int idx_size[1], double
                 idx_mult_data[], int idx_mult_size[1])
{
  int loop_ub;
  int j;
  idx_mult_size[0] = idx_size[0] << 1;
  loop_ub = idx_size[0] << 1;
  for (j = 0; j < loop_ub; j++) {
    idx_mult_data[j] = 0.0;
  }

  for (loop_ub = 0; loop_ub < idx_size[0]; loop_ub++) {
    for (j = 0; j < 2; j++) {
      idx_mult_data[(loop_ub << 1) + j] = (idx_data[loop_ub] - 1.0) * 2.0 + (1.0
        + (double)j);
    }
  }
}

//
// File trailer for multiplyIdx.cpp
//
// [EOF]
//
