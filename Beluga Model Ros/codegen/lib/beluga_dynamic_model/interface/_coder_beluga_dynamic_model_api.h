/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_beluga_dynamic_model_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 15-Jan-2017 12:59:41
 */

#ifndef _CODER_BELUGA_DYNAMIC_MODEL_API_H
#define _CODER_BELUGA_DYNAMIC_MODEL_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_beluga_dynamic_model_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void beluga_dynamic_model(real_T state[16], real_T u[4], real_T
  disturbance[3], real_T Ts, real_T newstate[16]);
extern void beluga_dynamic_model_api(const mxArray *prhs[4], const mxArray *
  plhs[1]);
extern void beluga_dynamic_model_atexit(void);
extern void beluga_dynamic_model_initialize(void);
extern void beluga_dynamic_model_terminate(void);
extern void beluga_dynamic_model_xil_terminate(void);

#endif

/*
 * File trailer for _coder_beluga_dynamic_model_api.h
 *
 * [EOF]
 */
