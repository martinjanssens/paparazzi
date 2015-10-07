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

float divergence;

#include "subsystems/datalink/telemetry.h"
#include <time.h>

long previous_time;

 static void send_divergence(void) {
  DOWNLINK_SEND_FAKE_DIVERGENCE (DefaultChannel, DefaultDevice, &divergence);
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
#define VERTICAL_CTRL_MODULE_IGAIN 0.01
#endif

static abi_event agl_ev; ///< The altitude ABI event

/// Callback function of the ground altitude
static void vertical_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);

struct VerticalCtrlDemo v_ctrl;


void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool_t in_flight);

void vertical_ctrl_module_init(void)
{

  v_ctrl.agl = 0.0f;
  v_ctrl.agl_lp = 0.0f;
  v_ctrl.vel = 0.0f;
  v_ctrl.setpoint = 0.0f;
  v_ctrl.lp_factor = 0.9f;
  v_ctrl.pgain = VERTICAL_CTRL_MODULE_PGAIN;
  v_ctrl.igain = VERTICAL_CTRL_MODULE_IGAIN;
  v_ctrl.sum_err = 0.0f;
  v_ctrl.nominal_thrust = 0.55f;
  previous_time = time(NULL);

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(VERTICAL_CTRL_MODULE_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);

  register_periodic_telemetry(DefaultPeriodic, "FAKE_DIVERGENCE", send_divergence);
}

void vertical_ctrl_module_run(bool_t in_flight)
{
  float new_lp;

  // get dt:
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  long new_time = spec.tv_nsec / 1.0E6;
  long delta_t = new_time - previous_time;
  float dt = ((float)delta_t) / 1000.0f;
  previous_time = new_time;
  //printf("dt = %f\n", dt);


  if (!in_flight) {
    // Reset integrators
    v_ctrl.sum_err = 0;
    stabilization_cmd[COMMAND_THRUST] = 0;
  } else {
	// calculate the new low-pass height and the velocity
	new_lp = v_ctrl.agl_lp * v_ctrl.lp_factor + v_ctrl.agl * (1.0f - v_ctrl.lp_factor);
	v_ctrl.vel = (new_lp - v_ctrl.agl_lp) / dt; // should still be divided by dt!
	v_ctrl.agl_lp = new_lp;
	// calculate the fake divergence:
	if(abs(v_ctrl.agl_lp) > 0.001f) {
		divergence = v_ctrl.vel / v_ctrl.agl_lp;
	}
	else divergence = 1000.0f;
	printf("agl_lp = %f, vel = %f, divergence = %f, dt = %f.\n", v_ctrl.agl_lp, v_ctrl.vel, divergence, (float) dt);
	// use the divergence for control:
	int32_t nominal_throttle = v_ctrl.nominal_thrust * MAX_PPRZ;
	float err = v_ctrl.setpoint - divergence;
	int32_t thrust = nominal_throttle + v_ctrl.pgain * err; // + v_ctrl.igain * v_ctrl.sum_err; // still with i-gain (should be determined with 0-divergence maneuver)
	Bound(thrust, 0, MAX_PPRZ);
	stabilization_cmd[COMMAND_THRUST] = thrust;
	v_ctrl.sum_err += err;



	// old control:
	/*int32_t nominal_throttle = 0.5 * MAX_PPRZ;
    float err = v_ctrl.setpoint - v_ctrl.agl;
    int32_t thrust = nominal_throttle + v_ctrl.pgain * err + v_ctrl.igain * v_ctrl.sum_err;
    Bound(thrust, 0, MAX_PPRZ);
    stabilization_cmd[COMMAND_THRUST] = thrust;
    v_ctrl.sum_err += err;*/

  }
}

static void vertical_ctrl_agl_cb(uint8_t sender_id, float distance)
{
  printf("distance = %f\n", distance);
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
