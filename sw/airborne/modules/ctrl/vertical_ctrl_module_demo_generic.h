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
 * @file modules/ctrl/vertical_ctrl_module_demo.c
 * @brief example vertical
 *
 * Implements an example vertical controller in a module.
 */

#ifndef VERTICAL_CTRL_MODULE_DEMO_H_
#define VERTICAL_CTRL_MODULE_DEMO_H_

#define COV_WINDOW_SIZE 60
// 30
// 15

#include "std.h"

struct VerticalCtrlDemo {
  float agl;
  float agl_lp;
  float lp_factor;
  float vel;
  float setpoint;
  float pgain;
  float igain;
  float dgain;
  float sum_err;
  float nominal_thrust;
  int VISION_METHOD;
  int CONTROL_METHOD;
  float cov_set_point;
  float cov_limit;
  float pgain_adaptive;
  float igain_adaptive;
  float dgain_adaptive;
  int COV_METHOD;
  int delay_steps;
  int OPTICAL_FLOW_TYPE;
};

extern struct VerticalCtrlDemo v_ctrl;
// This is all used in the vertical_ctrl_module_demo.xml as the dl_settings (the settings that can be changed in flight)

unsigned long ind_hist;
float thrust_history[COV_WINDOW_SIZE];
float opticalflow_history[COV_WINDOW_SIZE];
float past_opticalflow_history[COV_WINDOW_SIZE];

// for example use the standard horizontal (hover) mode // GUIDANCE_H_MODE_ATTITUDE // GUIDANCE_H_MODE_HOVER
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_HOVER

// and own guidance_v, I guess using the extension MODULE means that it's using the guidance_v_mode defined in this module
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

float get_cov(float* a, float* b, int n_elements);
float get_mean_array(float *a, int n_elements);
void reset_all_vars(void);

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool_t in_flight);

#endif /* VERTICAL_CTRL_MODULE_DEMO_H_ */
