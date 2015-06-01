/*
 * File: pointextraction.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 01-Jun-2015 17:14:17
 */

#ifndef __POINTEXTRACTION_H__
#define __POINTEXTRACTION_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "pointextraction_types.h"

/* Function Declarations */
extern void pointextraction(const unsigned char ImGrayR_r[332220], const
  unsigned char ImGrayR_l[332220], double updateVect[32], double numAnchors,
  double binx, double biny, double border, double minDistance, boolean_T
  useInitGuess, const double h_u_apo_data[], const int h_u_apo_size[1], double
  pts_r_arr[64], double pts_l_arr[64], double useDisparity[32], double z_all[96]);
extern void pointextraction_initialize(void);
extern void pointextraction_terminate(void);

#endif

/*
 * File trailer for pointextraction.h
 *
 * [EOF]
 */
