/* 
 * File: pointextraction_types.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 01-Jun-2015 17:14:17 
 */

#ifndef __POINTEXTRACTION_TYPES_H__
#define __POINTEXTRACTION_TYPES_H__

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_cvstPTStruct_T
#define typedef_cvstPTStruct_T
typedef struct
{
    int blockSize[2];
    int numPyramidLevels;
    double maxIterations;
    double epsilon;
    double maxBidirectionalError;
} cvstPTStruct_T;
#endif /*typedef_cvstPTStruct_T*/
#ifndef struct_emxArray_boolean_T_32
#define struct_emxArray_boolean_T_32
struct emxArray_boolean_T_32
{
    boolean_T data[32];
    int size[1];
};
#endif /*struct_emxArray_boolean_T_32*/
#ifndef typedef_emxArray_boolean_T_32
#define typedef_emxArray_boolean_T_32
typedef struct emxArray_boolean_T_32 emxArray_boolean_T_32;
#endif /*typedef_emxArray_boolean_T_32*/
#ifndef struct_emxArray_int32_T_32
#define struct_emxArray_int32_T_32
struct emxArray_int32_T_32
{
    int data[32];
    int size[1];
};
#endif /*struct_emxArray_int32_T_32*/
#ifndef typedef_emxArray_int32_T_32
#define typedef_emxArray_int32_T_32
typedef struct emxArray_int32_T_32 emxArray_int32_T_32;
#endif /*typedef_emxArray_int32_T_32*/
#ifndef struct_emxArray_real_T_1x453
#define struct_emxArray_real_T_1x453
struct emxArray_real_T_1x453
{
    double data[453];
    int size[2];
};
#endif /*struct_emxArray_real_T_1x453*/
#ifndef typedef_emxArray_real_T_1x453
#define typedef_emxArray_real_T_1x453
typedef struct emxArray_real_T_1x453 emxArray_real_T_1x453;
#endif /*typedef_emxArray_real_T_1x453*/
#ifndef struct_emxArray_real_T_32
#define struct_emxArray_real_T_32
struct emxArray_real_T_32
{
    double data[32];
    int size[1];
};
#endif /*struct_emxArray_real_T_32*/
#ifndef typedef_emxArray_real_T_32
#define typedef_emxArray_real_T_32
typedef struct emxArray_real_T_32 emxArray_real_T_32;
#endif /*typedef_emxArray_real_T_32*/
#ifndef struct_emxArray_real_T_32x2
#define struct_emxArray_real_T_32x2
struct emxArray_real_T_32x2
{
    double data[64];
    int size[2];
};
#endif /*struct_emxArray_real_T_32x2*/
#ifndef typedef_emxArray_real_T_32x2
#define typedef_emxArray_real_T_32x2
typedef struct emxArray_real_T_32x2 emxArray_real_T_32x2;
#endif /*typedef_emxArray_real_T_32x2*/

#endif
/* 
 * File trailer for pointextraction_types.h 
 *  
 * [EOF] 
 */
