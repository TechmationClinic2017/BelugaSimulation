/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: beluga_dynamic_model.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 15-Jan-2017 12:59:41
 */

#ifndef BELUGA_DYNAMIC_MODEL_H
#define BELUGA_DYNAMIC_MODEL_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "beluga_dynamic_model_types.h"

/* Function Declarations */
extern void beluga_dynamic_model(const double state[16], const double u[4],
  const double disturbance[3], double Ts, double newstate[16]);

#endif

/*
 * File trailer for beluga_dynamic_model.h
 *
 * [EOF]
 */