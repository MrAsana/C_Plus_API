/*
 * File: InverseKinematics.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 25-Jan-2022 17:36:15
 */

#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void InverseKinematics(const double pose[16], double nsparam,
                              double rconf, double joints[7]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for InverseKinematics.h
 *
 * [EOF]
 */
