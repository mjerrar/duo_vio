//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 10-Mar-2016 13:25:23
//
#ifndef __SLAM_H__
#define __SLAM_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Custom Header Code
//***************************************************************************
//
//    Copyright (c) 2015-2016 AIT, ETH Zurich. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  1. Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//  3. Neither the name AIT nor the names of its contributors may be
//     used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//
// **************************************************************************

// Type Definitions
#include <stdio.h>

// Function Declarations
extern void SLAM(int updateVect[48], const double z_all_l[96], const double
                 z_all_r[96], double dt, const VIOMeasurements *measurements,
                 const DUOParameters *cameraParameters, const NoiseParameters
                 *noiseParameters, const VIOParameters *b_VIOParameters,
                 boolean_T vision, boolean_T reset, RobotState *xt_out, double
                 map_out[144], AnchorPose anchor_poses_out[6], double
                 delayedStatus_out[48]);
extern void SLAM_initialize();
extern void SLAM_terminate();

#endif

//
// File trailer for SLAM.h
//
// [EOF]
//
