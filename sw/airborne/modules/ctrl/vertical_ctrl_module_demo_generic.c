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
float opticalflow;
float opticalflow_vision;
float opticalflow_vision_dt;
float normalized_thrust;
float cov_opticalflow;
float pstate;
float pused;
float dt;
int vision_message_nr;
int previous_message_nr;
int landing;
float previous_err;
float previous_cov_err;
float x_pos;

#define MINIMUM_GAIN 0.1

// used for automated landing:
//#include "subsystems/navigation/common_flight_plan.h"
//#include "generated/flight_plan.h"
#include "firmwares/rotorcraft/autopilot.h"
//#include "subsystems/nav.h"
#include "subsystems/navigation/common_flight_plan.h"

#include "subsystems/datalink/telemetry.h"
#include <time.h>
#include "firmwares/rotorcraft/navigation.h"

long previous_time;

// This is the message including the variables I get to see in the plotter. Despite the crappy name, this still works with translational flow, but you need to include the x-distance in it once you have made it available in this program!
static void send_opticalflow(void) {
  DOWNLINK_SEND_FAKE_OPTICAL_FLOW (DefaultChannel, DefaultDevice, &opticalflow, &opticalflow_vision_dt, &normalized_thrust, &cov_opticalflow, &pstate, &pused, &(v_ctrl.agl), &x_pos);
 }

#include "modules/ctrl/vertical_ctrl_module_demo.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
//#include "subsystems/gps/gps_udp.h"

/* Default sonar/agl to use */
#ifndef VERTICAL_CTRL_MODULE_AGL_ID
#define VERTICAL_CTRL_MODULE_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VERTICAL_CTRL_MODULE_AGL_ID)

/* Use optical flow estimates */
#ifndef VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID
#define VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID)

/* Same for gps coordinates for x-estimation */
#ifndef VERTICAL_CTRL_MODULE_GPS_ID
#define VERTICAL_CTRL_MODULE_GPS_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID)


#ifndef VERTICAL_CTRL_MODULE_PGAIN
#define VERTICAL_CTRL_MODULE_PGAIN 1.0
#endif

#ifndef VERTICAL_CTRL_MODULE_IGAIN
#define VERTICAL_CTRL_MODULE_IGAIN 1.0
#endif

#ifndef VERTICAL_CTRL_MODULE_DGAIN
#define VERTICAL_CTRL_MODULE_DGAIN 1.0
#endif

#ifndef VERTICAL_CTRL_MODULE_VISION_METHOD
#define VERTICAL_CTRL_MODULE_VISION_METHOD 0
#endif

#ifndef VERTICAL_CTRL_MODULE_CONTROL_METHOD
#define VERTICAL_CTRL_MODULE_CONTROL_METHOD 1
#endif

#ifndef VERTICAL_CTRL_MODULE_COV_METHOD
#define VERTICAL_CTRL_MODULE_COV_METHOD 0
#endif

#ifndef VERTICAL_CTRL_MODULE_OPTICAL_FLOW_TYPE
#define VERTICAL_CTRL_MODULE_OPTICAL_FLOW_TYPE 0
#endif

static abi_event agl_ev; ///< The altitude ABI event
static abi_event optical_flow_ev;
//static abi_event x_ev; //x-position ABI event

/// Callback function of the ground altitude
static void vertical_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);
// Callback function of the optical flow estimate:
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, uint8_t quality, float size_divergence, float dist);
// Callback function of the position:
//static void vertical_ctrl_gps_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, struct GpsState gps_s);

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
  v_ctrl.cov_set_point = -0.025f;
  v_ctrl.cov_limit = 0.0010f; //1.0f; // for cov(uz,OF)
  v_ctrl.lp_factor = 0.95f;
  v_ctrl.pgain = VERTICAL_CTRL_MODULE_PGAIN;
  v_ctrl.igain = VERTICAL_CTRL_MODULE_IGAIN;
  v_ctrl.dgain = VERTICAL_CTRL_MODULE_DGAIN;
  v_ctrl.sum_err = 0.0f;
  v_ctrl.nominal_thrust = 0.710f; //0.666f; // 0.640 with small battery
  v_ctrl.VISION_METHOD = VERTICAL_CTRL_MODULE_VISION_METHOD;
  v_ctrl.CONTROL_METHOD = VERTICAL_CTRL_MODULE_CONTROL_METHOD;
  v_ctrl.COV_METHOD = VERTICAL_CTRL_MODULE_COV_METHOD;
  v_ctrl.OPTICAL_FLOW_TYPE = VERTICAL_CTRL_MODULE_OPTICAL_FLOW_TYPE;
  v_ctrl.delay_steps = 40;
  v_ctrl.pgain_adaptive = 10.0;
  v_ctrl.igain_adaptive = 0.25;
  v_ctrl.dgain_adaptive = 0.00;

  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  previous_time = spec.tv_nsec / 1.0E6;
  //previous_time = time(NULL);

  // clear histories:
  ind_hist = 0;
  for(i = 0; i < COV_WINDOW_SIZE; i++) {
	  thrust_history[i] = 0;
	  opticalflow_history[i] = 0;
  }

  previous_err = 0.0f;
  previous_cov_err = 0.0f;
  normalized_thrust = 0.0f;
  opticalflow = 0.0f;
  opticalflow_vision = 0.0f;
  opticalflow_vision_dt = 0.0f;
  cov_opticalflow = 0.0f;
  dt = 0.0f;
  pstate = v_ctrl.pgain;
  pused = pstate;
  vision_message_nr = 1;
  previous_message_nr = 0;
  v_ctrl.agl_lp = 0.0f;

  landing = 0;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(VERTICAL_CTRL_MODULE_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);
  // Subscribe to the optical flow estimator:
  AbiBindMsgOPTICAL_FLOW(VERTICAL_CTRL_MODULE_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);
  // Subscribe to the x-position ABI message
  //AbiBindMsgGPS(VERTICAL_CTRL_MODULE_GPS_ID, &x_ev, vertical_ctrl_gps_cb);

  register_periodic_telemetry(DefaultPeriodic, "FAKE_OPTICAL_FLOW", send_opticalflow);
}

void reset_all_vars()
{
	int i;
	v_ctrl.sum_err = 0;
	stabilization_cmd[COMMAND_THRUST] = 0;
	ind_hist = 0;
	v_ctrl.agl_lp = 0;
	cov_opticalflow = v_ctrl.cov_set_point;
	normalized_thrust = 0.0f;
	dt = 0.0f;
	previous_err = 0.0f;
	previous_cov_err = 0.0f;
	opticalflow = v_ctrl.setpoint;
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	previous_time = spec.tv_nsec / 1.0E6;
	vision_message_nr = 1;
	previous_message_nr = 0;
	for(i = 0; i < COV_WINDOW_SIZE; i++) {
		thrust_history[i] = 0;
		opticalflow_history[i] = 0;
	}
	landing = 0;
}

void vertical_ctrl_module_run(bool_t in_flight)
{
	int i;
	float new_lp;
	float div_factor;
	if(dt < 0) dt = 0.0f;
	// get delta time, dt, to scale the divergence measurements:
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	long new_time = spec.tv_nsec / 1.0E6;
	long delta_t = new_time - previous_time;
	dt += ((float)delta_t) / 1000.0f;
	if(dt > 10.0f) {
		printf("WAITED LONGER than 10 seconds\n");
		dt = 0.0f;
		return;
	}
	//printf("dt = %f vs %f\n", dt, ((float)delta_t) / 1000.0f); // we have negative times...
	previous_time = new_time;

	if (!in_flight) {
		// When not flying and in mode module:
		// Reset integrators
		printf("NOT FLYING!\n");
		reset_all_vars();
	} else {

		/***********
		 * VISION
		 ***********/
		if(v_ctrl.VISION_METHOD == 0) {
			
			// USE OPTITRACK HEIGHT if you are in divergence mode:
			v_ctrl.agl = (float) gps.lla_pos.alt / 1000.0f; //How is it possible to define this?
			// else we get an immediate jump in divergence when switching on.
			if(v_ctrl.agl_lp < 1E-5 || ind_hist == 0) {
				v_ctrl.agl_lp = v_ctrl.agl;
			}
			if(fabs(v_ctrl.agl-v_ctrl.agl_lp) > 1.0f)
			{
				printf("Outlier: agl = %f, agl_lp = %f\n", v_ctrl.agl, v_ctrl.agl_lp);
				// ignore outliers:
				v_ctrl.agl = v_ctrl.agl_lp;
			}
			// calculate the new low-pass height and the velocity
			new_lp = v_ctrl.agl_lp * v_ctrl.lp_factor + v_ctrl.agl * (1.0f - v_ctrl.lp_factor);

			//PROBABLY NEED TO DO SOME PROCESSING OF THE GPS X-POSITION! (make it relative to the object it will look at, remove outliers... No idea how to start doing this!)


			if(dt > 0.0001f)
			{
				v_ctrl.vel = (new_lp - v_ctrl.agl_lp) / dt; // should still be divided by dt!
				v_ctrl.agl_lp = new_lp;
				x_pos = GetPosX() + 4.0f; //So x cannot become negative

				// calculate the fake divergence: ONLY IN DIVERGENCE MODE!
				if(v_ctrl.agl_lp > 0.0001f) {
					if(v_ctrl.OPTICAL_FLOW_TYPE == 0) {
					        opticalflow = v_ctrl.vel / v_ctrl.agl_lp;
					}
					else {
						opticalflow = v_ctrl.vel / x_pos;
					}
					printf("x_position: %f, opticalflow: %f\n", x_pos, opticalflow);
					opticalflow_vision_dt = (opticalflow_vision/dt);
					if(fabs(opticalflow_vision_dt) > 1E-5) {
						div_factor = opticalflow / opticalflow_vision_dt;
					}
					//printf("optical flow optitrack: %f, optical flow vision: %f, factor = %f, dt = %f\n", opticalflow, opticalflow_vision/dt, div_factor, dt);
					//printf("div = %f, vel = %f, agl_lp = %f\n", divergence, v_ctrl.vel, v_ctrl.agl_lp);
				}
				else
				{
					opticalflow = 1000.0f;
					//printf("agl = %f, agl_lp = %f, vel = %f, divergence = %f, dt = %f.\n",  v_ctrl.agl, v_ctrl.agl_lp, v_ctrl.vel, divergence, (float) dt);
					// perform no control with this value (keeping thrust the same)
					return;
				}
				// reset dt:
				dt = 0.0f;
			}
		}
		else 
		{
			if(vision_message_nr != previous_message_nr && dt > 1E-5 && ind_hist > 1) {
				div_factor = -1.28f; // magic number comprising field of view etc. NEED TO ADAPT THIS!
				float new_opticalflow = (opticalflow_vision * div_factor) / dt;
				// deal with outliers: THIS ASSUMES OUTLIERS WILL BE EQUAL FOR BOTH OFs, ADAPT? Make generic?
				if(fabs(new_opticalflow - opticalflow) > 0.20) {
					printf("OUTLIER: div = %f", new_opticalflow);
					if(new_opticalflow < opticalflow) new_opticalflow = opticalflow - 0.10f;
					else new_opticalflow = opticalflow + 0.10f;
					printf("corrected to div = %f\n", new_opticalflow);
				}
				opticalflow = opticalflow * v_ctrl.lp_factor + (new_opticalflow * (1.0f - v_ctrl.lp_factor));
				printf("Vision optical flow = %f, dt = %f\n", opticalflow_vision, dt);
				previous_message_nr = vision_message_nr;
				dt = 0.0f;
			}
			else {
				// after a re-enter, divergence should be equal to the set point:
				if(ind_hist <= 1) {
					opticalflow = v_ctrl.setpoint;
					for(i = 0; i < COV_WINDOW_SIZE; i++) {
						thrust_history[i] = 0;
						opticalflow_history[i] = 0;
					}
					ind_hist++;
					dt = 0.0f;
					int32_t nominal_throttle = v_ctrl.nominal_thrust * MAX_PPRZ;
					stabilization_cmd[COMMAND_THRUST] = nominal_throttle;

				}
				// else: do nothing, let dt increment
				//printf("Skipping, no new vision input: dt = %f\n", dt);
				return;
			}
		}

		/***********
		* CONTROL
		***********/

		int32_t nominal_throttle = v_ctrl.nominal_thrust * MAX_PPRZ;
		if(!landing) {
			if(v_ctrl.CONTROL_METHOD == 0) {
				// fixed gain control, cov_limit for landing:

				// use the divergence for control:
				float err = v_ctrl.setpoint - opticalflow;
				int32_t thrust = nominal_throttle + v_ctrl.pgain * err * MAX_PPRZ + v_ctrl.igain * v_ctrl.sum_err * MAX_PPRZ; // still with i-gain (should be determined with 0-divergence maneuver)
				// make sure the p gain is logged:
				pstate = v_ctrl.pgain;
				pused = pstate;
				// bound thrust:
				Bound(thrust, 0.8*nominal_throttle, 0.75*MAX_PPRZ); // used to be 0.6 / 0.9

				// histories and cov detection:
				normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
				thrust_history[ind_hist%COV_WINDOW_SIZE] = normalized_thrust;
				opticalflow_history[ind_hist%COV_WINDOW_SIZE] = opticalflow;
				int ind_past = (ind_hist%COV_WINDOW_SIZE) - v_ctrl.delay_steps;
				while(ind_past < 0) ind_past += COV_WINDOW_SIZE;
				float past_opticalflow = opticalflow_history[ind_past];
				past_opticalflow_history[ind_hist%COV_WINDOW_SIZE] = past_opticalflow;
				ind_hist++;
				//if(ind_hist >= COV_WINDOW_SIZE) ind_hist = 0; // prevent overflow
				if(v_ctrl.COV_METHOD == 0) {
					cov_opticalflow = get_cov(thrust_history, opticalflow_history, COV_WINDOW_SIZE);
				}
				else {
					cov_opticalflow = get_cov(past_opticalflow_history, opticalflow_history, COV_WINDOW_SIZE);
					printf("ind_past = %d, past_opticalflow = %f, cov_opticalflow = %f\n", ind_past, past_opticalflow, cov_opticalflow);
				}

				if(ind_hist >= COV_WINDOW_SIZE && fabs(cov_opticalflow) > v_ctrl.cov_limit) {
					// set to NAV and give land command:
					// autopilot_set_mode(AP_MODE_NAV);
					// nav_goto_block( 9 );
					landing = 1;
					printf("START LANDING!\n");
					thrust = 0.80 * nominal_throttle;
				}

				stabilization_cmd[COMMAND_THRUST] = thrust;
				v_ctrl.sum_err += err;
				printf("Err = %f, thrust = %f, opticalflow = %f, cov = %f, ind_hist = %d\n", err, normalized_thrust, opticalflow, cov_opticalflow, (int) ind_hist);
			}
			else {
				// ADAPTIVE GAIN CONTROL: For both the divergence and translational flow modes!

				// adapt the gains according to the error in covariance:
				float error_cov = v_ctrl.cov_set_point - cov_opticalflow;
				// limit the error_cov, which could else become very large:
				if(error_cov > fabs(v_ctrl.cov_set_point)) error_cov = fabs(v_ctrl.cov_set_point);
				pstate -= (v_ctrl.igain_adaptive * pstate) * error_cov; //v_ctrl.igain_adaptive * error_cov;//
				if(pstate < MINIMUM_GAIN) pstate = MINIMUM_GAIN;
				// regulate the optical flow:
				float err = v_ctrl.setpoint - opticalflow;
				pused = pstate - (v_ctrl.pgain_adaptive * pstate) * error_cov; // v_ctrl.pgain_adaptive * error_cov;//// pused instead of v_ctrl.pgain to avoid problems with interface
				if(pused < MINIMUM_GAIN) pused = MINIMUM_GAIN;
				if(v_ctrl.COV_METHOD == 1 && error_cov > 0.001) {
					// Emergency measure:
					pused = 0.5 * pused;
				}
				int32_t thrust = nominal_throttle + pused * err * MAX_PPRZ + v_ctrl.igain * v_ctrl.sum_err * MAX_PPRZ; // still with i-gain (should be determined with 0-divergence maneuver)

				// histories and cov detection:
				normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
				thrust_history[ind_hist%COV_WINDOW_SIZE] = normalized_thrust;
				opticalflow_history[ind_hist%COV_WINDOW_SIZE] = opticalflow;
				int ind_past = (ind_hist%COV_WINDOW_SIZE) - v_ctrl.delay_steps;
				while(ind_past < 0) ind_past += COV_WINDOW_SIZE;
				float past_opticalflow = opticalflow_history[ind_past];
				past_opticalflow_history[ind_hist%COV_WINDOW_SIZE] = 100.0f * past_opticalflow;
				ind_hist++; // TODO: prevent overflow?
				// only take covariance into account if there are enough samples in the histories:
				if(ind_hist >= COV_WINDOW_SIZE) {
					if(v_ctrl.COV_METHOD == 0) {
						cov_opticalflow = get_cov(thrust_history, opticalflow_history, COV_WINDOW_SIZE);
					}
					else {
						cov_opticalflow = get_cov(past_opticalflow_history, opticalflow_history, COV_WINDOW_SIZE);
						printf("Cov_opticalflow = %f\n", cov_opticalflow);
					}
				}
				else {
					printf("cov_opticalflow not enough history\n");
					cov_opticalflow = v_ctrl.cov_set_point;
				}

				// landing condition based on pstate (if too low)
				/*if(abs(cov_div) > v_ctrl.cov_limit) {
  			  // set to NAV and give land command:
  			  autopilot_set_mode(AP_MODE_NAV);
  			  nav_goto_block( 9 );
  		  }*/

				// bound thrust:
				Bound(thrust, 0.8*nominal_throttle, 0.75*MAX_PPRZ); // was 0.6 0.9
				stabilization_cmd[COMMAND_THRUST] = thrust;
				v_ctrl.sum_err += err;
				printf("Err cov = %f, cov = %f, thrust = %f, err OF = %f, pstate = %f, pused = %f\n", error_cov, cov_opticalflow, normalized_thrust, err, pstate, pused);
			}
		}
		else
		{
			printf("Landing!!\n");
			int32_t thrust = 0.90 * nominal_throttle;
			Bound(thrust, 0.6*nominal_throttle, 0.9*MAX_PPRZ);
			stabilization_cmd[COMMAND_THRUST] = thrust;
		}
		//printf("agl = %f, agl_lp = %f, vel = %f, divergence = %f, dt = %f.\n",  v_ctrl.agl, v_ctrl.agl_lp, v_ctrl.vel, divergence, (float) dt);

	}
}

float get_mean_array(float *a, int n_elements)
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

float get_cov(float* a, float* b, int n_elements)
{
	// determine means for each vector:
	float mean_a = get_mean_array(a, n_elements);
	float mean_b = get_mean_array(b, n_elements);
	float cov = 0;
	for(unsigned int i = 0; i < n_elements; i++)
	{
		cov += (a[i] - mean_a) * (b[i] - mean_b);
	}

	cov /= n_elements;

	return cov;
}



// Reading from "sensors":
static void vertical_ctrl_agl_cb(uint8_t sender_id, float distance)
{
  //printf("distance = %f\n", distance);
  v_ctrl.agl = distance;
}

static void vertical_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, uint8_t quality, float size_divergence, float dist)
{ 
  if (v_ctrl.OPTICAL_FLOW_TYPE == 0) opticalflow_vision = size_divergence;
  else opticalflow_vision = flow_der_y;
  vision_message_nr++;
  if(vision_message_nr > 10) vision_message_nr = 0;
  //printf("Received divergence: %f\n", divergence_vision);
}

//???
/*static void vertical_ctrl_gps_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, struct GpsState gps_s)
{
  x_pos = gps_s.ecef_pos.x;
}*/


////////////////////////////////////////////////////////////////////
// Call our controller
void guidance_v_module_init(void)
{
  vertical_ctrl_module_init();
}

void guidance_v_module_enter(void)
{
	int i;
  printf("ENTERING!");
  // reset integrator
  v_ctrl.sum_err = 0.0f;
  landing = 0;
  ind_hist = 0;
  previous_err = 0.0f;
  previous_cov_err = 0.0f;
  v_ctrl.agl_lp = 0.0f;
  cov_opticalflow = v_ctrl.cov_set_point;
  normalized_thrust = 0.0f;
  opticalflow = v_ctrl.setpoint;
  dt = 0.0f;
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  previous_time = spec.tv_nsec / 1.0E6;
  vision_message_nr = 1;
  previous_message_nr = 0;
  for(i = 0; i < COV_WINDOW_SIZE; i++) {
	  thrust_history[i] = 0;
	  opticalflow_history[i] = 0;
  }
}

void guidance_v_module_run(bool_t in_flight)
{
  vertical_ctrl_module_run(in_flight);
}
