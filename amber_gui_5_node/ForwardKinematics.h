/*
 * File: ForwardKinematics.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 25-Jan-2022 17:39:36
 */

#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void ForwardKinematics(const double joints[7], double pose[16],
                              double *nsparam);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ForwardKinematics.h
 *
 * [EOF]
 */
