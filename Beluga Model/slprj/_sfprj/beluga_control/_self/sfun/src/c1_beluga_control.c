/* Include files */

#include "beluga_control_sfun.h"
#include "c1_beluga_control.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "beluga_control_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[40] = { "thruster_pwm",
  "thruster_force", "DEAD_POS", "DEAD_NEG", "PERIOD", "LEAK_THRESH", "i", "u",
  "v", "w", "p", "q", "r", "x", "y", "z", "phi", "theta", "psi", "d", "rho", "n",
  "Kt", "X_c", "Y_c", "Z_c", "K_c", "M_c", "N_c", "X", "Y", "Z", "K", "M", "N",
  "nargin", "nargout", "state", "inputs", "F" };

/* Function Declarations */
static void initialize_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void initialize_params_c1_beluga_control
  (SFc1_beluga_controlInstanceStruct *chartInstance);
static void enable_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void disable_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void c1_update_debugger_state_c1_beluga_control
  (SFc1_beluga_controlInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_beluga_control
  (SFc1_beluga_controlInstanceStruct *chartInstance);
static void set_sim_state_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void sf_gateway_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void mdl_start_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void c1_chartstep_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void initSimStructsc1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct *chartInstance,
  const mxArray *c1_b_F, const char_T *c1_identifier, real_T c1_y[6]);
static void c1_b_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[6]);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_c_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static creal_T c1_d_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_interp1(SFc1_beluga_controlInstanceStruct *chartInstance,
  real_T c1_varargin_1[81], real_T c1_varargin_2[81], real_T c1_varargin_3);
static void c1_error(SFc1_beluga_controlInstanceStruct *chartInstance);
static void c1_b_error(SFc1_beluga_controlInstanceStruct *chartInstance);
static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_e_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_f_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_beluga_control, const char_T *
  c1_identifier);
static uint8_T c1_g_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_beluga_controlInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc1_beluga_controlInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_beluga_control(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_is_active_c1_beluga_control = 0U;
}

static void initialize_params_c1_beluga_control
  (SFc1_beluga_controlInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_beluga_control
  (SFc1_beluga_controlInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_beluga_control
  (SFc1_beluga_controlInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  const mxArray *c1_b_y = NULL;
  uint8_T c1_hoistedGlobal;
  const mxArray *c1_c_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(2, 1), false);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", *chartInstance->c1_F, 0, 0U, 1U, 0U,
    1, 6), false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_hoistedGlobal = chartInstance->c1_is_active_c1_beluga_control;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_hoistedGlobal, 3, 0U, 0U, 0U, 0),
                false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T c1_dv0[6];
  int32_T c1_i0;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("F", c1_u, 0)),
                      "F", c1_dv0);
  for (c1_i0 = 0; c1_i0 < 6; c1_i0++) {
    (*chartInstance->c1_F)[c1_i0] = c1_dv0[c1_i0];
  }

  chartInstance->c1_is_active_c1_beluga_control = c1_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c1_beluga_control",
       c1_u, 1)), "is_active_c1_beluga_control");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_beluga_control(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  int32_T c1_i1;
  int32_T c1_i2;
  int32_T c1_i3;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i1 = 0; c1_i1 < 4; c1_i1++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_inputs)[c1_i1], 1U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  for (c1_i2 = 0; c1_i2 < 12; c1_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_state)[c1_i2], 0U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_beluga_control(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_beluga_controlMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c1_i3 = 0; c1_i3 < 6; c1_i3++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_F)[c1_i3], 2U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }
}

static void mdl_start_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c1_chartstep_c1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  int32_T c1_i4;
  int32_T c1_i5;
  real_T c1_b_state[12];
  uint32_T c1_debug_family_var_map[40];
  real_T c1_b_inputs[4];
  real_T c1_thruster_pwm[81];
  real_T c1_thruster_force[81];
  real_T c1_DEAD_POS;
  real_T c1_DEAD_NEG;
  real_T c1_PERIOD;
  real_T c1_LEAK_THRESH;
  real_T c1_i;
  real_T c1_u;
  real_T c1_v;
  real_T c1_w;
  real_T c1_p;
  real_T c1_q;
  real_T c1_r;
  real_T c1_x;
  real_T c1_y;
  real_T c1_z;
  real_T c1_phi;
  real_T c1_theta;
  real_T c1_psi;
  real_T c1_d;
  real_T c1_rho;
  creal_T c1_n;
  creal_T c1_Kt;
  real_T c1_X_c;
  real_T c1_Y_c;
  real_T c1_Z_c;
  real_T c1_K_c;
  real_T c1_M_c;
  real_T c1_N_c;
  real_T c1_X;
  real_T c1_Y;
  real_T c1_Z;
  real_T c1_K;
  real_T c1_M;
  real_T c1_N;
  real_T c1_nargin = 2.0;
  real_T c1_nargout = 1.0;
  real_T c1_b_F[6];
  int32_T c1_i6;
  int32_T c1_i7;
  static real_T c1_dv1[81] = { -40.047363000000004, -40.047363000000004,
    -40.092489, -37.555623000000004, -36.621711, -35.509257, -33.773868,
    -32.394582, -31.237002000000004, -29.991132, -28.522575000000003,
    -27.632808000000004, -26.564499, -24.784965, -23.182992000000002,
    -22.159809000000003, -21.358332, -19.534653000000002, -18.466344000000003,
    -17.35389, -16.10802, -14.639463000000001, -13.082616, -12.191868,
    -10.991124000000001, -10.056231, -8.899632, -7.7430330000000005, -6.80814,
    -6.051789, -5.116896, -4.271274, -3.470778, -2.714427, -1.868805, -1.112454,
    -0.533664, -0.266832, 0.0, 0.0, 0.0, 0.0, 0.0, 0.57879, 1.112454,
    1.8246600000000002, 2.581011, 3.4266330000000003, 4.093713, 5.116896,
    6.22935, 7.030827, 8.276697, 9.2557350000000014, 10.812582,
    12.236994000000001, 13.660425000000002, 14.995566, 16.464123, 17.709993,
    19.222695, 20.557836, 21.759561, 23.672511, 24.74082, 26.431083, 27.722079,
    29.635029000000003, 31.326273, 32.884101, 33.818013, 34.263387, 37.066104,
    38.668077000000004, 39.291012, 41.159817000000004, 42.940332000000005,
    44.809137000000007, 47.122335, 48.057228, 49.970178000000004 };

  int32_T c1_b_i;
  real_T c1_d0;
  real_T c1_d1;
  int32_T c1_i8;
  int32_T c1_i9;
  real_T c1_dv2[81];
  real_T c1_dv3[81];
  int32_T c1_c_i;
  int32_T c1_i10;
  real_T c1_A;
  real_T c1_b_x;
  real_T c1_c_x[4];
  real_T c1_d_x;
  int32_T c1_k;
  real_T c1_b_y;
  creal_T c1_e_x;
  real_T c1_yr;
  boolean_T c1_b0;
  real_T c1_f_x;
  real_T c1_g_x;
  real_T c1_yi;
  boolean_T c1_b1;
  real_T c1_h_x;
  real_T c1_i_x;
  boolean_T c1_b;
  real_T c1_b_z;
  real_T c1_c_z;
  real_T c1_j_x;
  boolean_T c1_b_b;
  real_T c1_k_x;
  creal_T c1_c_y;
  boolean_T c1_c_b;
  real_T c1_b_A;
  real_T c1_absxr;
  real_T c1_l_x;
  real_T c1_absxi;
  real_T c1_m_x;
  creal_T c1_d_y;
  real_T c1_b_X[6];
  real_T c1_ar;
  real_T c1_n_x;
  real_T c1_ai;
  real_T c1_e_y;
  real_T c1_br;
  real_T c1_x1;
  real_T c1_o_x;
  real_T c1_bi;
  real_T c1_x2;
  real_T c1_f_y;
  real_T c1_d_z;
  real_T c1_b_x1;
  int32_T c1_i11;
  real_T c1_b_x2;
  real_T c1_brm;
  real_T c1_absxd2;
  real_T c1_bim;
  real_T c1_p_x;
  real_T c1_q_x;
  real_T c1_s;
  real_T c1_g_y;
  real_T c1_h_y;
  real_T c1_b_d;
  static creal_T c1_dc0 = { 1.0,       /* re */
    0.0                                /* im */
  };

  int32_T c1_i12;
  real_T c1_r_x;
  real_T c1_s_x;
  real_T c1_sgnbr;
  real_T c1_nr;
  real_T c1_t_x;
  real_T c1_i_y;
  real_T c1_j_y;
  real_T c1_ni;
  real_T c1_k_y;
  real_T c1_e_z;
  real_T c1_f_z;
  real_T c1_u_x;
  real_T c1_sgnbi;
  real_T c1_l_y;
  real_T c1_g_z;
  creal_T c1_v_x;
  creal_T c1_w_x;
  creal_T c1_x_x;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i4 = 0; c1_i4 < 12; c1_i4++) {
    c1_b_state[c1_i4] = (*chartInstance->c1_state)[c1_i4];
  }

  for (c1_i5 = 0; c1_i5 < 4; c1_i5++) {
    c1_b_inputs[c1_i5] = (*chartInstance->c1_inputs)[c1_i5];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 40U, 40U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_thruster_pwm, 0U, c1_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_thruster_force, 1U, c1_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_DEAD_POS, 2U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_DEAD_NEG, 3U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_PERIOD, 4U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_LEAK_THRESH, 5U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i, 6U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_u, 7U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_v, 8U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_w, 9U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_p, 10U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_q, 11U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_r, 12U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x, 13U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y, 14U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_z, 15U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_phi, 16U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_theta, 17U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_psi, 18U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_d, 19U, c1_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_rho, 20U, c1_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_n, 21U, c1_e_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Kt, 22U, c1_e_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_X_c, 23U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Y_c, 24U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Z_c, 25U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_K_c, 26U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_M_c, 27U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_N_c, 28U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_X, 29U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Y, 30U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Z, 31U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_K, 32U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_M, 33U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_N, 34U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 35U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 36U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_state, 37U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_inputs, 38U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_F, 39U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  for (c1_i6 = 0; c1_i6 < 81; c1_i6++) {
    c1_thruster_pwm[c1_i6] = 1100.0 + 10.0 * (real_T)c1_i6;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 8);
  for (c1_i7 = 0; c1_i7 < 81; c1_i7++) {
    c1_thruster_force[c1_i7] = c1_dv1[c1_i7];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
  c1_DEAD_POS = 16.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 13);
  c1_DEAD_NEG = -35.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 14);
  c1_PERIOD = 500.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 15);
  c1_LEAK_THRESH = 150.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 18);
  c1_i = 1.0;
  c1_b_i = 0;
  while (c1_b_i < 4) {
    c1_i = 1.0 + (real_T)c1_b_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 19);
    c1_d0 = c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 2092, 9, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 2092U, 9U, c1_i), 1, 4) - 1];
    if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c1_d0, 0.0, -1, 4U,
          c1_d0 > 0.0))) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 20);
      c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
        chartInstance->S, 1U, 2118, 9, MAX_uint32_T, (int32_T)sf_integer_check
        (chartInstance->S, 1U, 2118U, 9U, c1_i), 1, 4) - 1] =
        c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
        chartInstance->S, 1U, 2130, 9, MAX_uint32_T, (int32_T)sf_integer_check
        (chartInstance->S, 1U, 2130U, 9U, c1_i), 1, 4) - 1] * 50.0 + c1_DEAD_POS;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
      c1_d1 = c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
        chartInstance->S, 1U, 2168, 9, MAX_uint32_T, (int32_T)sf_integer_check
        (chartInstance->S, 1U, 2168U, 9U, c1_i), 1, 4) - 1];
      if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c1_d1, 0.0, -1, 2U,
            c1_d1 < 0.0))) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
        c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 1U, 2194, 9, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 1U, 2194U, 9U, c1_i), 1, 4) - 1] =
          c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 1U, 2206, 9, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 1U, 2206U, 9U, c1_i), 1, 4) - 1] * 50.0 +
          c1_DEAD_NEG;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 25);
    c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 2270, 9, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 2270U, 9U, c1_i), 1, 4) - 1] =
      c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 2282, 9, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 2282U, 9U, c1_i), 1, 4) - 1] + 1500.0;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 26);
    for (c1_i8 = 0; c1_i8 < 81; c1_i8++) {
      c1_dv2[c1_i8] = 1100.0 + 10.0 * (real_T)c1_i8;
    }

    for (c1_i9 = 0; c1_i9 < 81; c1_i9++) {
      c1_dv3[c1_i9] = c1_dv1[c1_i9];
    }

    c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 2308, 9, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 2308U, 9U, c1_i), 1, 4) - 1] = c1_interp1
      (chartInstance, c1_dv2, c1_dv3, c1_b_inputs[sf_eml_array_bounds_check
       (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 2358, 9, MAX_uint32_T,
        (int32_T)sf_integer_check(chartInstance->S, 1U, 2358U, 9U, c1_i), 1, 4)
       - 1]);
    c1_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
  c1_u = c1_b_state[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 33);
  c1_v = c1_b_state[1];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 34);
  c1_w = c1_b_state[2];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 35);
  c1_p = c1_b_state[3];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 36);
  c1_q = c1_b_state[4];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 37);
  c1_r = c1_b_state[5];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
  c1_x = c1_b_state[6];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 40);
  c1_y = c1_b_state[7];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 41);
  c1_z = c1_b_state[8];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 42);
  c1_phi = c1_b_state[9];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
  c1_theta = c1_b_state[10];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 44);
  c1_psi = c1_b_state[11];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 47);
  c1_d = 0.076;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 48);
  c1_rho = 1000.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 49);
  c1_i = 1.0;
  c1_c_i = 0;
  while (c1_c_i < 4) {
    c1_i = 1.0 + (real_T)c1_c_i;
    CV_EML_FOR(0, 1, 1, 1);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 50);
    c1_A = c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 2822, 9, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 2822U, 9U, c1_i), 1, 4) - 1];
    c1_b_x = c1_A;
    c1_d_x = c1_b_x;
    c1_b_y = c1_d_x / 0.0061986923008;
    c1_e_x.re = c1_b_y;
    c1_e_x.im = 0.0;
    c1_n = c1_e_x;
    c1_e_x = c1_n;
    c1_n = c1_e_x;
    c1_e_x = c1_n;
    c1_n = c1_e_x;
    c1_e_x = c1_n;
    c1_n = c1_e_x;
    if (c1_n.im == 0.0) {
      if (c1_n.re < 0.0) {
        c1_yr = 0.0;
        c1_yi = muDoubleScalarSqrt(muDoubleScalarAbs(c1_n.re));
      } else {
        c1_yr = muDoubleScalarSqrt(c1_n.re);
        c1_yi = 0.0;
      }
    } else if (c1_n.re == 0.0) {
      if (c1_n.im < 0.0) {
        c1_g_x = -c1_n.im;
        c1_i_x = c1_g_x;
        c1_c_z = c1_i_x / 2.0;
        c1_yr = muDoubleScalarSqrt(c1_c_z);
        c1_yi = -c1_yr;
      } else {
        c1_f_x = c1_n.im;
        c1_h_x = c1_f_x;
        c1_b_z = c1_h_x / 2.0;
        c1_yr = muDoubleScalarSqrt(c1_b_z);
        c1_yi = c1_yr;
      }
    } else {
      c1_e_x = c1_n;
      c1_b0 = muDoubleScalarIsNaN(c1_e_x.re);
      c1_b1 = muDoubleScalarIsNaN(c1_e_x.im);
      c1_b = (c1_b0 || c1_b1);
      if (c1_b) {
        c1_yr = rtNaN;
        c1_yi = rtNaN;
      } else {
        c1_j_x = c1_n.im;
        c1_b_b = muDoubleScalarIsInf(c1_j_x);
        if (c1_b_b) {
          c1_yr = rtInf;
          c1_yi = c1_n.im;
        } else {
          c1_k_x = c1_n.re;
          c1_c_b = muDoubleScalarIsInf(c1_k_x);
          if (c1_c_b) {
            if (c1_n.re < 0.0) {
              c1_yr = 0.0;
              c1_yi = rtInf;
            } else {
              c1_yr = rtInf;
              c1_yi = 0.0;
            }
          } else {
            c1_absxr = muDoubleScalarAbs(c1_n.re);
            c1_absxi = muDoubleScalarAbs(c1_n.im);
            if ((c1_absxr > 4.4942328371557893E+307) || (c1_absxi >
                 4.4942328371557893E+307)) {
              c1_absxr *= 0.5;
              c1_absxi *= 0.5;
              c1_o_x = c1_absxr;
              c1_f_y = c1_absxi;
              c1_b_x1 = c1_o_x;
              c1_b_x2 = c1_f_y;
              c1_absxd2 = muDoubleScalarHypot(c1_b_x1, c1_b_x2);
              if (c1_absxd2 > c1_absxr) {
                c1_q_x = c1_absxr;
                c1_h_y = c1_absxd2;
                c1_s_x = c1_q_x;
                c1_j_y = c1_h_y;
                c1_f_z = c1_s_x / c1_j_y;
                c1_yr = muDoubleScalarSqrt(c1_absxd2) * muDoubleScalarSqrt(1.0 +
                  c1_f_z);
              } else {
                c1_yr = muDoubleScalarSqrt(c1_absxd2) * 1.4142135623730951;
              }
            } else {
              c1_n_x = c1_absxr;
              c1_e_y = c1_absxi;
              c1_x1 = c1_n_x;
              c1_x2 = c1_e_y;
              c1_d_z = muDoubleScalarHypot(c1_x1, c1_x2);
              c1_yr = muDoubleScalarSqrt((c1_d_z + c1_absxr) * 0.5);
            }

            if (c1_n.re > 0.0) {
              c1_p_x = c1_n.im;
              c1_g_y = c1_yr;
              c1_r_x = c1_p_x;
              c1_i_y = c1_g_y;
              c1_e_z = c1_r_x / c1_i_y;
              c1_yi = 0.5 * c1_e_z;
            } else {
              if (c1_n.im < 0.0) {
                c1_yi = -c1_yr;
              } else {
                c1_yi = c1_yr;
              }

              c1_t_x = c1_n.im;
              c1_k_y = c1_yi;
              c1_u_x = c1_t_x;
              c1_l_y = c1_k_y;
              c1_g_z = c1_u_x / c1_l_y;
              c1_yr = 0.5 * c1_g_z;
            }
          }
        }
      }
    }

    c1_n.re = c1_yr;
    c1_n.im = c1_yi;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 51);
    c1_e_x = c1_n;
    c1_c_y.re = 0.076 * c1_e_x.re;
    c1_c_y.im = 0.076 * c1_e_x.im;
    c1_b_A = c1_u;
    c1_l_x = c1_b_A;
    c1_m_x = c1_l_x;
    c1_d_y = c1_c_y;
    c1_ar = c1_m_x;
    c1_ai = 0.0;
    c1_br = c1_d_y.re;
    c1_bi = c1_d_y.im;
    if (c1_bi == 0.0) {
      if (c1_ai == 0.0) {
        c1_c_y.re = c1_ar / c1_br;
        c1_c_y.im = 0.0;
      } else if (c1_ar == 0.0) {
        c1_c_y.re = 0.0;
        c1_c_y.im = c1_ai / c1_br;
      } else {
        c1_c_y.re = c1_ar / c1_br;
        c1_c_y.im = c1_ai / c1_br;
      }
    } else if (c1_br == 0.0) {
      if (c1_ar == 0.0) {
        c1_c_y.re = c1_ai / c1_bi;
        c1_c_y.im = 0.0;
      } else if (c1_ai == 0.0) {
        c1_c_y.re = 0.0;
        c1_c_y.im = -(c1_ar / c1_bi);
      } else {
        c1_c_y.re = c1_ai / c1_bi;
        c1_c_y.im = -(c1_ar / c1_bi);
      }
    } else {
      c1_brm = muDoubleScalarAbs(c1_br);
      c1_bim = muDoubleScalarAbs(c1_bi);
      if (c1_brm > c1_bim) {
        c1_s = c1_bi / c1_br;
        c1_b_d = c1_br + c1_s * c1_bi;
        c1_nr = c1_ar + c1_s * c1_ai;
        c1_ni = c1_ai - c1_s * c1_ar;
        c1_c_y.re = c1_nr / c1_b_d;
        c1_c_y.im = c1_ni / c1_b_d;
      } else if (c1_bim == c1_brm) {
        if (c1_br > 0.0) {
          c1_sgnbr = 0.5;
        } else {
          c1_sgnbr = -0.5;
        }

        if (c1_bi > 0.0) {
          c1_sgnbi = 0.5;
        } else {
          c1_sgnbi = -0.5;
        }

        c1_nr = c1_ar * c1_sgnbr + c1_ai * c1_sgnbi;
        c1_ni = c1_ai * c1_sgnbr - c1_ar * c1_sgnbi;
        c1_c_y.re = c1_nr / c1_brm;
        c1_c_y.im = c1_ni / c1_brm;
      } else {
        c1_s = c1_br / c1_bi;
        c1_b_d = c1_bi + c1_s * c1_br;
        c1_nr = c1_s * c1_ar + c1_ai;
        c1_ni = c1_s * c1_ai - c1_ar;
        c1_c_y.re = c1_nr / c1_b_d;
        c1_c_y.im = c1_ni / c1_b_d;
      }
    }

    c1_e_x.re = c1_dc0.re - c1_c_y.re;
    c1_e_x.im = c1_dc0.im - c1_c_y.im;
    c1_Kt.re = 0.1858 * c1_e_x.re;
    c1_Kt.im = 0.1858 * c1_e_x.im;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 52);
    c1_e_x = c1_Kt;
    c1_c_y.re = 1000.0 * c1_e_x.re;
    c1_c_y.im = 1000.0 * c1_e_x.im;
    c1_e_x = c1_n;
    c1_v_x = c1_e_x;
    c1_w_x = c1_e_x;
    c1_e_x.re = c1_v_x.re * c1_w_x.re - c1_v_x.im * c1_w_x.im;
    c1_e_x.im = c1_v_x.re * c1_w_x.im + c1_v_x.im * c1_w_x.re;
    c1_x_x = c1_e_x;
    c1_e_x.re = c1_c_y.re * c1_x_x.re - c1_c_y.im * c1_x_x.im;
    c1_e_x.im = c1_c_y.re * c1_x_x.im + c1_c_y.im * c1_x_x.re;
    c1_c_y.re = 3.3362176E-5 * c1_e_x.re;
    c1_c_y.im = 3.3362176E-5 * c1_e_x.im;
    c1_b_inputs[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 2895, 9, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 2895U, 9U, c1_i), 1, 4) - 1] = c1_c_y.re;
    c1_c_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 1, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 56);
  for (c1_i10 = 0; c1_i10 < 4; c1_i10++) {
    c1_c_x[c1_i10] = c1_b_inputs[c1_i10];
  }

  c1_X_c = c1_c_x[0];
  for (c1_k = 1; c1_k + 1 < 5; c1_k++) {
    c1_X_c += c1_c_x[c1_k];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 57);
  c1_Y_c = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 58);
  c1_Z_c = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 59);
  c1_K_c = ((c1_b_inputs[0] - c1_b_inputs[1]) + c1_b_inputs[2]) - c1_b_inputs[3];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 60);
  c1_M_c = c1_b_inputs[1] - c1_b_inputs[3];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 61);
  c1_N_c = c1_b_inputs[0] - c1_b_inputs[2];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 64);
  c1_X = c1_X_c;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 65);
  c1_Y = c1_Y_c;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 66);
  c1_Z = c1_Z_c;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 67);
  c1_K = c1_K_c;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 68);
  c1_M = c1_M_c;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 69);
  c1_N = c1_N_c;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 70);
  c1_b_X[0] = c1_X;
  c1_b_X[1] = c1_Y;
  c1_b_X[2] = c1_Z;
  c1_b_X[3] = c1_K;
  c1_b_X[4] = c1_M;
  c1_b_X[5] = c1_N;
  for (c1_i11 = 0; c1_i11 < 6; c1_i11++) {
    c1_b_F[c1_i11] = c1_b_X[c1_i11];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -70);
  _SFD_SYMBOL_SCOPE_POP();
  for (c1_i12 = 0; c1_i12 < 6; c1_i12++) {
    (*chartInstance->c1_F)[c1_i12] = c1_b_F[c1_i12];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_beluga_control(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_i13;
  const mxArray *c1_y = NULL;
  real_T c1_u[6];
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  for (c1_i13 = 0; c1_i13 < 6; c1_i13++) {
    c1_u[c1_i13] = (*(real_T (*)[6])c1_inData)[c1_i13];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct *chartInstance,
  const mxArray *c1_b_F, const char_T *c1_identifier, real_T c1_y[6])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_F), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_F);
}

static void c1_b_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[6])
{
  real_T c1_dv4[6];
  int32_T c1_i14;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv4, 1, 0, 0U, 1, 0U, 1, 6);
  for (c1_i14 = 0; c1_i14 < 6; c1_i14++) {
    c1_y[c1_i14] = c1_dv4[c1_i14];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_F;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[6];
  int32_T c1_i15;
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_b_F = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_F), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_F);
  for (c1_i15 = 0; c1_i15 < 6; c1_i15++) {
    (*(real_T (*)[6])c1_outData)[c1_i15] = c1_y[c1_i15];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_i16;
  const mxArray *c1_y = NULL;
  real_T c1_u[4];
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  for (c1_i16 = 0; c1_i16 < 4; c1_i16++) {
    c1_u[c1_i16] = (*(real_T (*)[4])c1_inData)[c1_i16];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 4, 1), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_i17;
  const mxArray *c1_y = NULL;
  real_T c1_u[12];
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  for (c1_i17 = 0; c1_i17 < 12; c1_i17++) {
    c1_u[c1_i17] = (*(real_T (*)[12])c1_inData)[c1_i17];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 12), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_c_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d2;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d2, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d2;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout), &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  creal_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(creal_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 1U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static creal_T c1_d_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  creal_T c1_y;
  creal_T c1_dc1;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_dc1, 1, 0, 1U, 0, 0U, 0);
  c1_y = c1_dc1;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_Kt;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  creal_T c1_y;
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_Kt = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_Kt), &c1_thisId);
  sf_mex_destroy(&c1_Kt);
  *(creal_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_i18;
  const mxArray *c1_y = NULL;
  real_T c1_u[81];
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  for (c1_i18 = 0; c1_i18 < 81; c1_i18++) {
    c1_u[c1_i18] = (*(real_T (*)[81])c1_inData)[c1_i18];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 1, 81), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

const mxArray *sf_c1_beluga_control_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c1_nameCaptureInfo;
}

static real_T c1_interp1(SFc1_beluga_controlInstanceStruct *chartInstance,
  real_T c1_varargin_1[81], real_T c1_varargin_2[81], real_T c1_varargin_3)
{
  real_T c1_Vq;
  int32_T c1_i19;
  real_T c1_xi;
  real_T c1_y[81];
  int32_T c1_i20;
  int32_T c1_k;
  real_T c1_x[81];
  real_T c1_b_x;
  int32_T c1_j1;
  boolean_T c1_b;
  int32_T c1_b_k;
  int32_T c1_b_j1;
  real_T c1_xtmp;
  real_T c1_b_xi;
  real_T c1_minx;
  int32_T c1_j2;
  real_T c1_maxx;
  real_T c1_b_xtmp;
  real_T c1_c_xi;
  real_T c1_b_minx;
  real_T c1_b_maxx;
  real_T c1_c_x;
  boolean_T c1_b_b;
  real_T c1_d_xi;
  int32_T c1_low_i;
  int32_T c1_low_ip1;
  int32_T c1_high_i;
  int32_T c1_n;
  int32_T c1_b_low_i;
  real_T c1_xn;
  int32_T c1_b_high_i;
  real_T c1_xnp1;
  int32_T c1_mid_i;
  real_T c1_A;
  real_T c1_B;
  real_T c1_d_x;
  real_T c1_b_y;
  real_T c1_e_x;
  real_T c1_c_y;
  real_T c1_r;
  real_T c1_onemr;
  real_T c1_y1;
  real_T c1_y2;
  int32_T exitg1;
  for (c1_i19 = 0; c1_i19 < 81; c1_i19++) {
    c1_y[c1_i19] = c1_varargin_2[c1_i19];
  }

  c1_xi = c1_varargin_3;
  for (c1_i20 = 0; c1_i20 < 81; c1_i20++) {
    c1_x[c1_i20] = c1_varargin_1[c1_i20];
  }

  c1_k = 0;
  do {
    exitg1 = 0;
    if (c1_k + 1 < 82) {
      c1_b_x = c1_x[c1_k];
      c1_b = muDoubleScalarIsNaN(c1_b_x);
      if (c1_b) {
        c1_error(chartInstance);
        exitg1 = 1;
      } else {
        c1_k++;
      }
    } else {
      if (c1_x[1] < c1_x[0]) {
        for (c1_j1 = 1; c1_j1 < 41; c1_j1++) {
          c1_xtmp = c1_x[c1_j1 - 1];
          c1_x[c1_j1 - 1] = c1_x[81 - c1_j1];
          c1_x[81 - c1_j1] = c1_xtmp;
        }

        for (c1_b_j1 = 0; c1_b_j1 + 1 < 41; c1_b_j1++) {
          c1_j2 = 80 - c1_b_j1;
          c1_b_xtmp = c1_y[c1_b_j1];
          c1_y[c1_b_j1] = c1_y[c1_j2];
          c1_y[c1_j2] = c1_b_xtmp;
        }
      }

      for (c1_b_k = 0; c1_b_k + 2 < 82; c1_b_k++) {
        if (c1_x[c1_b_k + 1] <= c1_x[c1_b_k]) {
          c1_b_error(chartInstance);
        }
      }

      c1_b_xi = c1_xi;
      c1_Vq = rtNaN;
      c1_minx = c1_x[0];
      c1_maxx = c1_x[80];
      c1_c_xi = c1_b_xi;
      c1_b_minx = c1_minx;
      c1_b_maxx = c1_maxx;
      c1_c_x = c1_c_xi;
      c1_b_b = muDoubleScalarIsNaN(c1_c_x);
      if (c1_b_b) {
        c1_Vq = rtNaN;
      } else if ((c1_c_xi > c1_b_maxx) || (c1_c_xi < c1_b_minx)) {
      } else {
        c1_d_xi = c1_c_xi;
        c1_low_i = 1;
        c1_low_ip1 = 1;
        c1_high_i = 81;
        while (c1_high_i > c1_low_ip1 + 1) {
          c1_b_low_i = c1_low_i;
          c1_b_high_i = c1_high_i;
          c1_mid_i = (c1_b_low_i + c1_b_high_i) >> 1;
          if (c1_d_xi >= c1_x[c1_mid_i - 1]) {
            c1_low_i = c1_mid_i;
            c1_low_ip1 = c1_mid_i;
          } else {
            c1_high_i = c1_mid_i;
          }
        }

        c1_n = c1_low_i;
        c1_xn = c1_x[c1_n - 1];
        c1_xnp1 = c1_x[c1_n];
        c1_A = c1_c_xi - c1_xn;
        c1_B = c1_xnp1 - c1_xn;
        c1_d_x = c1_A;
        c1_b_y = c1_B;
        c1_e_x = c1_d_x;
        c1_c_y = c1_b_y;
        c1_r = c1_e_x / c1_c_y;
        c1_onemr = 1.0 - c1_r;
        if (c1_r == 0.0) {
          c1_y1 = c1_y[c1_n - 1];
          c1_Vq = c1_y1;
        } else if (c1_r == 1.0) {
          c1_y2 = c1_y[c1_n];
          c1_Vq = c1_y2;
        } else {
          c1_y1 = c1_y[c1_n - 1];
          c1_y2 = c1_y[c1_n];
          if (c1_y1 == c1_y2) {
            c1_Vq = c1_y1;
          } else {
            c1_Vq = c1_onemr * c1_y1 + c1_r * c1_y2;
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c1_Vq;
}

static void c1_error(SFc1_beluga_controlInstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_cv0[21] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'i', 'n', 't',
    'e', 'r', 'p', '1', ':', 'N', 'a', 'N', 'i', 'n', 'X' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_cv0, 10, 0U, 1U, 0U, 2, 1, 21),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c1_y));
}

static void c1_b_error(SFc1_beluga_controlInstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_cv1[35] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', 'e', 'r', 'p', '1', '_', 'n', 'o', 'n',
    'M', 'o', 'n', 'o', 't', 'o', 'n', 'i', 'c', 'X' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_cv1, 10, 0U, 1U, 0U, 2, 1, 35),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c1_y));
}

static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_e_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i21;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i21, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i21;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_beluga_control, const char_T *
  c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_beluga_control), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_beluga_control);
  return c1_y;
}

static uint8_T c1_g_emlrt_marshallIn(SFc1_beluga_controlInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_beluga_controlInstanceStruct
  *chartInstance)
{
  chartInstance->c1_state = (real_T (*)[12])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c1_F = (real_T (*)[6])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_inputs = (real_T (*)[4])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_beluga_control_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(185410163U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(876445866U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(986321858U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(15573207U);
}

mxArray* sf_c1_beluga_control_get_post_codegen_info(void);
mxArray *sf_c1_beluga_control_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("KfbaocZkgTT4mRrjWLfHhG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c1_beluga_control_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_beluga_control_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_beluga_control_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_beluga_control_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_beluga_control_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c1_beluga_control(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"F\",},{M[8],M[0],T\"is_active_c1_beluga_control\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_beluga_control_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_beluga_controlInstanceStruct *chartInstance =
      (SFc1_beluga_controlInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _beluga_controlMachineNumber_,
           1,
           1,
           1,
           0,
           3,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_beluga_controlMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_beluga_controlMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _beluga_controlMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"state");
          _SFD_SET_DATA_PROPS(1,1,1,0,"inputs");
          _SFD_SET_DATA_PROPS(2,2,0,1,"F");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,2,0,0,0,2,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,3357);
        _SFD_CV_INIT_EML_IF(0,1,0,2089,2105,2161,2181);
        _SFD_CV_INIT_EML_IF(0,1,1,2161,2181,-1,2181);
        _SFD_CV_INIT_EML_FOR(0,1,0,2057,2081,2402);
        _SFD_CV_INIT_EML_FOR(0,1,1,2773,2797,2937);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,2092,2105,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,2168,2181,-1,2);

        {
          unsigned int dimVector[1];
          dimVector[0]= 12U;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4U;
          dimVector[1]= 1U;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6U;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)
            c1_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _beluga_controlMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_beluga_controlInstanceStruct *chartInstance =
      (SFc1_beluga_controlInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c1_state);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c1_F);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c1_inputs);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sL1O5WucWlbbsTewwjAPo3G";
}

static void sf_opaque_initialize_c1_beluga_control(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_beluga_controlInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_beluga_control((SFc1_beluga_controlInstanceStruct*)
    chartInstanceVar);
  initialize_c1_beluga_control((SFc1_beluga_controlInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_beluga_control(void *chartInstanceVar)
{
  enable_c1_beluga_control((SFc1_beluga_controlInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_beluga_control(void *chartInstanceVar)
{
  disable_c1_beluga_control((SFc1_beluga_controlInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_beluga_control(void *chartInstanceVar)
{
  sf_gateway_c1_beluga_control((SFc1_beluga_controlInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_beluga_control(SimStruct* S)
{
  return get_sim_state_c1_beluga_control((SFc1_beluga_controlInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_beluga_control(SimStruct* S, const
  mxArray *st)
{
  set_sim_state_c1_beluga_control((SFc1_beluga_controlInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c1_beluga_control(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_beluga_controlInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_beluga_control_optimization_info();
    }

    finalize_c1_beluga_control((SFc1_beluga_controlInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_beluga_control((SFc1_beluga_controlInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_beluga_control(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_beluga_control((SFc1_beluga_controlInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c1_beluga_control(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssSetStatesModifiedOnlyInUpdate(S, 1);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_beluga_control_optimization_info
      (sim_mode_is_rtw_gen(S), sim_mode_is_modelref_sim(S), sim_mode_is_external
       (S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 1);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1596362444U));
  ssSetChecksum1(S,(3064767714U));
  ssSetChecksum2(S,(3032379666U));
  ssSetChecksum3(S,(1912213817U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_beluga_control(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_beluga_control(SimStruct *S)
{
  SFc1_beluga_controlInstanceStruct *chartInstance;
  chartInstance = (SFc1_beluga_controlInstanceStruct *)utMalloc(sizeof
    (SFc1_beluga_controlInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc1_beluga_controlInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_beluga_control;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_beluga_control;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_beluga_control;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_beluga_control;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_beluga_control;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_beluga_control;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_beluga_control;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_beluga_control;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_beluga_control;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_beluga_control;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_beluga_control;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c1_beluga_control(chartInstance);
}

void c1_beluga_control_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_beluga_control(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_beluga_control(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_beluga_control(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_beluga_control_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
