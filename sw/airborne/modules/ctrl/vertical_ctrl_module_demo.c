/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/vertical_control_module.h
 * @brief example vertical controller
 *
 */

// variables for in message:
float divergence;
float normalized_thrust;
float cov;
float pstate;

#include "subsystems/datalink/telemetry.h"
#include <time.h>

long previous_time;

 static void send_divergence(void) {
  DOWNLINK_SEND_FAKE_DIVERGENCE (DefaultChannel, DefaultDevice, &divergence, &normalized_thrust, &cov);
 }

#include "modules/ctrl/vertical_ctrl_module_demo.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"

/* Default sonar/agl to use */
#ifndef VERTICAL_CTRL_MODULE_AGL_ID
#define VERTICAL_CTRL_MODULE_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VERTICAL_CTRL_MODULE_AGL_ID)

#ifndef VERTICAL_CTRL_MODULE_PGAIN
#define VERTICAL_CTRL_MODULE_PGAIN 1.0
#endif

#ifndef VERTICAL_CTRL_MODULE_IGAIN
#define VERTICAL_CTRL_MODULE_IGAIN 0.0
#endif

#ifndef VERTICAL_CTRL_MODULE_VISION_METHOD
#define VERTICAL_CTRL_MODULE_VISION_METHOD 0
#endif

#ifndef VERTICAL_CTRL_MODULE_CONTROL_METHOD
#define VERTICAL_CTRL_MODULE_CONTROL_METHOD 0
#endif

static abi_event agl_ev; ///< The altitude ABI event

/// Callback function of the ground altitude
static void vertical_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);

struct VerticalCtrlDemo v_ctrl;


void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool_t in_flight);

void vertical_ctrl_module_init(void)
{
  unsigned int i;

  v_ctrl.agl = 0.0f;
  v_ctrl.agl_lp = 0.0f;
  v_ctrl.vel = 0.0f;
  v_ctrl.setpoint = 0.0f;
  v_ctrl.cov_set_point = -0.25f;
  v_ctrl.cov_limit = 1.0f;
  v_ctrl.lp_factor = 0.95f;
  v_ctrl.pgain = VERTICAL_CTRL_MODULE_PGAIN;
  v_ctrl.igain = VERTICAL_CTRL_MODULE_IGAIN;
  v_ctrl.sum_err = 0.0f;
  v_ctrl.nominal_thrust = 0.640f;
  v_ctrl.VISION_METHOD = VERTICAL_CTRL_MODULE_VISION_METHOD;
  v_ctrl.CONTROL_METHOD = VERTICAL_CTRL_MODULE_CONTROL_METHOD;
  v_ctrl.pgain_adaptive = 0.01;
  v_ctrl.igain_adaptive = 0.001;

  previous_time = time(NULL);

  // clear histories:
  ind_hist = 0;
  for(i = 0; i < COV_WINDOW_SIZE; i++) {
	  thrust_history[i] = 0;
	  divergence_history[i] = 0;
  }

  normalized_thrust = 0.0f;
  divergence = 0.0f;
  cov = 0.0f;
  pstate = v_ctrl.pgain;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(VERTICAL_CTRL_MODULE_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);

  register_periodic_telemetry(DefaultPeriodic, "FAKE_DIVERGENCE", send_divergence);
}

void vertical_ctrl_module_run(bool_t in_flight)
{
  float new_lp;

  // get delta time, dt, to scale the divergence measurements:
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  long new_time = spec.tv_nsec / 1.0E6;
  long delta_t = new_time - previous_time;
  float dt = ((float)delta_t) / 1000.0f;
  previous_time = new_time;

  if (!in_flight) {
	  // When not flying:
	  // Reset integrators
	  v_ctrl.sum_err = 0;
	  stabilization_cmd[COMMAND_THRUST] = 0;
  } else {

	  if(v_ctrl.VISION_METHOD == 0) {

		  // USE OPTITRACK HEIGHT:
		  // printf("z = %f\n", (float) gps.lla_pos.alt);
		  v_ctrl.agl = (float) gps.lla_pos.alt / 1000.0f;
		  // calculate the new low-pass height and the velocity
		  new_lp = v_ctrl.agl_lp * v_ctrl.lp_factor + v_ctrl.agl * (1.0f - v_ctrl.lp_factor);

		  if(dt > 0.0001f)
		  {
			  v_ctrl.vel = (new_lp - v_ctrl.agl_lp) / dt; // should still be divided by dt!
			  v_ctrl.agl_lp = new_lp;

			  // calculate the fake divergence:
			  if(v_ctrl.agl_lp > 0.0001f) {
				  divergence = v_ctrl.vel / v_ctrl.agl_lp;
			  }
			  else
			  {
				  divergence = 1000.0f;
				  printf("agl = %f, agl_lp = %f, vel = %f, divergence = %f, dt = %f.\n",  v_ctrl.agl, v_ctrl.agl_lp, v_ctrl.vel, divergence, (float) dt);
				  // perform no control with this value (keeping thrust the same)
				  return;
			  }
		  }
	  }

	  if(v_ctrl.CONTROL_METHOD == 0) {
		  // fixed gain control, cov_limit for landing:

		  // use the divergence for control:
		  int32_t nominal_throttle = v_ctrl.nominal_thrust * MAX_PPRZ;
		  float err = v_ctrl.setpoint - divergence;
		  int32_t thrust = nominal_throttle + v_ctrl.pgain * err * MAX_PPRZ;// + v_ctrl.igain * v_ctrl.sum_err * MAX_PPRZ; // still with i-gain (should be determined with 0-divergence maneuver)

		  // histories and cov detection:
		  normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
		  thrust_history[ind_hist%COV_WINDOW_SIZE] = normalized_thrust;
		  divergence_history[ind_hist%COV_WINDOW_SIZE] = divergence;
		  ind_hist++;
		  if(ind_hist >= COV_WINDOW_SIZE) ind_hist = 0; // prevent overflow
		  cov = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
		  if(abs(cov) > v_ctrl.cov_limit) {
			  // set to NAV and give land command:

		  }
		  // bound thrust:
		  Bound(thrust, 0, MAX_PPRZ);
		  stabilization_cmd[COMMAND_THRUST] = thrust;
		  v_ctrl.sum_err += err;
		  printf("Err = %f, div = %f, cov = %f\n", err, divergence, cov);
	  }
	  else {
		  // ADAPTIVE GAIN CONTROL:

	  }

		//printf("agl = %f, agl_lp = %f, vel = %f, divergence = %f, dt = %f.\n",  v_ctrl.agl, v_ctrl.agl_lp, v_ctrl.vel, divergence, (float) dt);

  }
}

float get_cov(float* a, float* b, int n_elements)
{
	// determine means for each vector:
	float mean_a = get_mean(a, n_elements);
	float mean_b = get_mean(b, n_elements);
	float cov = 0;
	for(unsigned int i = 0; i < n_elements; i++)
	{
		cov += (a[i] - mean_a) * (b[i] - mean_b);
	}

	cov /= n_elements;

	return cov;
}

float get_mean(float *a, int n_elements)
{
	// determine the mean for the vector:
	float mean = 0;
	for(unsigned int i = 0; i < n_elements; i++)
	{
		mean += a[i];
	}
	mean /= n_elements;

	return mean;
}

static void vertical_ctrl_agl_cb(uint8_t sender_id, float distance)
{
  //printf("distance = %f\n", distance);
  v_ctrl.agl = distance;
}


////////////////////////////////////////////////////////////////////
// Call our controller
void guidance_v_module_init(void)
{
  vertical_ctrl_module_init();
}

void guidance_v_module_enter(void)
{
  // reset integrator
  v_ctrl.sum_err = 0.0f;
}

void guidance_v_module_run(bool_t in_flight)
{
  vertical_ctrl_module_run(in_flight);
}
