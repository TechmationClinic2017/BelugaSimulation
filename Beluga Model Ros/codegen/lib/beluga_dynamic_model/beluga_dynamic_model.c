/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: beluga_dynamic_model.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 15-Jan-2017 12:59:41
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "beluga_dynamic_model.h"
#include "mldivide.h"

/* Function Definitions */

/*
 * SURFACE_VEHICLE_DYNAMIC_MODEL
 *  Author: Vaibhav Viswanathan (vviswanathan@hmc.edu)
 *              (email me for any questions!)
 *                     ___
 *   current state -> |   |
 *                    | H | -> new state
 *  control inputs -> |___|
 * Arguments    : const double state[16]
 *                const double u[4]
 *                const double disturbance[3]
 *                double Ts
 *                double newstate[16]
 * Return Type  : void
 */
void beluga_dynamic_model(const double state[16], const double u[4], const
  double disturbance[3], double Ts, double newstate[16])
{
  double accel[6];
  int i;
  double dv0[36];
  double state_d[12];
  double b_newstate[12];
  int j;
  double Ts_int;
  (void)disturbance;

  /* 'beluga_dynamic_model:10' integration_steps = 5; */
  /* increase this number if you find the  */
  /* simulator go into numerical instability */
  /*  CONSTANTS TODO UPDATE */
  /* 'beluga_dynamic_model:14' m = 1; */
  /*  mass 1kg */
  /* 'beluga_dynamic_model:15' I = 1; */
  /* 'beluga_dynamic_model:16' k1 = 10; */
  /*  1/2*p*Cd*A; drag coefficient */
  /* 'beluga_dynamic_model:17' k2 = 5; */
  /*  rotational drag */
  /* 'beluga_dynamic_model:18' r = 0.050; */
  /*  thrusters r meters from the center of mass axis */
  /* 'beluga_dynamic_model:19' a = 0.050; */
  /*  center of drag a meters from center of mass along axis */
  /* 'beluga_dynamic_model:20' max_thrust = 500; */
  /*  Newtons */
  /* 'beluga_dynamic_model:22' mass_matrix = ones(6); */
  /*  STATE */
  /* 'beluga_dynamic_model:25' x       = state(1); */
  /* 'beluga_dynamic_model:26' y       = state(2); */
  /* 'beluga_dynamic_model:27' z       = state(3); */
  /* 'beluga_dynamic_model:28' phi     = state(4); */
  /* 'beluga_dynamic_model:29' theta   = state(5); */
  /* 'beluga_dynamic_model:30' psi     = state(6); */
  /* 'beluga_dynamic_model:32' x_d   = state(1); */
  /* 'beluga_dynamic_model:33' y_d   = state(2); */
  /* 'beluga_dynamic_model:34' z_d   = state(3); */
  /* 'beluga_dynamic_model:35' phi_d = state(4); */
  /* 'beluga_dynamic_model:36' theta_d = state(5); */
  /* 'beluga_dynamic_model:37' psi_d = state(6); */
  /* 'beluga_dynamic_model:39' x_disturbance     = disturbance(1); */
  /* 'beluga_dynamic_model:40' y_disturbance     = disturbance(2); */
  /* 'beluga_dynamic_model:41' theta_disturbance = disturbance(3); */
  /*  CALCULATE FORCES TODO */
  /* 'beluga_dynamic_model:45' Fl = max_thrust*u(1); */
  /* 'beluga_dynamic_model:46' Fr = max_thrust*u(2); */
  /* 'beluga_dynamic_model:48' F = ones(6,1); */
  /*  COMPUTE NEW STATE TODO */
  /*  Convert forces into accelerations */
  /* 'beluga_dynamic_model:53' accel = (mass_matrix)\F; */
  for (i = 0; i < 6; i++) {
    accel[i] = 1.0;
  }

  for (i = 0; i < 36; i++) {
    dv0[i] = 1.0;
  }

  mldivide(dv0, accel);

  /* 'beluga_dynamic_model:54' state_d = [state(1:6), accel']; */
  for (i = 0; i < 6; i++) {
    state_d[i] = state[i];
    state_d[i + 6] = accel[i];
  }

  /* 'beluga_dynamic_model:56' newstate = state(1:12); */
  memcpy(&b_newstate[0], &state[0], 12U * sizeof(double));

  /* 'beluga_dynamic_model:58' for j=1:integration_steps */
  for (j = 0; j < 5; j++) {
    /* 'beluga_dynamic_model:60' Ts_int = Ts/integration_steps; */
    Ts_int = Ts / 5.0;

    /*  update state variables by integrating */
    /* 'beluga_dynamic_model:63' newstate = newstate + state_d*Ts_int; */
    for (i = 0; i < 12; i++) {
      b_newstate[i] += state_d[i] * Ts_int;
    }
  }

  /*  convert to global frame */
  /* 'beluga_dynamic_model:68' newstate = [newstate, u(1), u(2), u(3), u(4)]; */
  memcpy(&newstate[0], &b_newstate[0], 12U * sizeof(double));
  newstate[12] = u[0];
  newstate[13] = u[1];
  newstate[14] = u[2];
  newstate[15] = u[3];
}

/*
 * File trailer for beluga_dynamic_model.c
 *
 * [EOF]
 */
