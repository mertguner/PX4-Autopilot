/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file FlightTaskManualAccelerationSlow.cpp
 */

#include "FlightTaskManualAccelerationSlow.hpp"
#include <px4_platform_common/events.h>
#include <mathlib/mathlib.h>

using namespace time_literals;
using namespace matrix;

bool FlightTaskManualAccelerationSlow::update()
{
	// Used to apply a configured default slowdown if neither MAVLink nor remote knob commands limits
	bool velocity_horizontal_limited = false;
	bool velocity_vertical_limited = false;
	bool yaw_rate_limited = false;

	// Limits which can only slow down from the nominal configuration we initialize with here
	// This is ensured by the executing classes
	float velocity_horizontal = _param_mpc_vel_manual.get();
	float velocity_up = _param_mpc_z_vel_max_up.get();
	float velocity_down = _param_mpc_z_vel_max_dn.get();
	float yaw_rate = math::radians(_param_mpc_man_y_max.get());

	// MAVLink commanded limits
	if (_velocity_limits_sub.update(&_velocity_limits)) {
		_velocity_limits_received_before = true;
	}

	if (_velocity_limits_received_before) {
		// message received once since mode was started
		if (PX4_ISFINITE(_velocity_limits.horizontal_velocity)) {
			velocity_horizontal = fmaxf(_velocity_limits.horizontal_velocity, _param_mc_slow_min_hvel.get());
			velocity_horizontal_limited = true;
		}

		if (PX4_ISFINITE(_velocity_limits.vertical_velocity)) {
			velocity_up = velocity_down = fmaxf(_velocity_limits.vertical_velocity, _param_mc_slow_min_vvel.get());
			velocity_vertical_limited = true;
		}

		if (PX4_ISFINITE(_velocity_limits.yaw_rate)) {
			yaw_rate = fmaxf(_velocity_limits.yaw_rate, math::radians(_param_mc_slow_min_yawr.get()));
			yaw_rate_limited = true;
		}
	}

	// Remote knob commanded limits
	if (_param_mc_slow_map_hvel.get() != 0) {
		const float min_horizontal_velocity_scale = _param_mc_slow_min_hvel.get() / fmaxf(velocity_horizontal, FLT_EPSILON);
		const float aux_input = getInputFromSanitizedAuxParameterIndex(_param_mc_slow_map_hvel.get());
		const float aux_based_scale =
			math::interpolate(aux_input, -1.f, 1.f, min_horizontal_velocity_scale, 1.f);
		velocity_horizontal *= aux_based_scale;
		velocity_horizontal_limited = true;
	}

	if (_param_mc_slow_map_vvel.get() != 0) {
		const float min_up_speed_scale = _param_mc_slow_min_vvel.get() / fmaxf(velocity_up, FLT_EPSILON);
		const float min_down_speed_scale = _param_mc_slow_min_vvel.get() / fmaxf(velocity_down, FLT_EPSILON);
		const float aux_input = getInputFromSanitizedAuxParameterIndex(_param_mc_slow_map_vvel.get());
		const float up_aux_based_scale =
			math::interpolate(aux_input, -1.f, 1.f, min_up_speed_scale, 1.f);
		const float down_aux_based_scale =
			math::interpolate(aux_input, -1.f, 1.f, min_down_speed_scale, 1.f);
		velocity_up *= up_aux_based_scale;
		velocity_down *= down_aux_based_scale;
		velocity_vertical_limited = true;
	}

	if (_param_mc_slow_map_yawr.get() != 0) {
		const float min_yaw_rate_scale = math::radians(_param_mc_slow_min_yawr.get()) / fmaxf(yaw_rate, FLT_EPSILON);
		const float aux_input = getInputFromSanitizedAuxParameterIndex(_param_mc_slow_map_yawr.get());
		const float aux_based_scale =
			math::interpolate(aux_input, -1.f, 1.f, min_yaw_rate_scale, 1.f);
		yaw_rate *= aux_based_scale;
		yaw_rate_limited = true;
	}

	// No input from remote and MAVLink -> use default slow mode limits
	if (!velocity_horizontal_limited) {
		velocity_horizontal = _param_mc_slow_def_hvel.get();
	}

	if (!velocity_vertical_limited) {
		velocity_up = velocity_down = _param_mc_slow_def_vvel.get();
	}

	if (!yaw_rate_limited) {
		yaw_rate = math::radians(_param_mc_slow_def_yawr.get());
	}

	// Interface to set resulting velocity limits
	FlightTaskManualAcceleration::_stick_acceleration_xy.setVelocityConstraint(velocity_horizontal);
	FlightTaskManualAltitude::_velocity_constraint_up = velocity_up;
	FlightTaskManualAltitude::_velocity_constraint_down = velocity_down;
	FlightTaskManualAcceleration::_stick_yaw.setYawspeedConstraint(yaw_rate);

	bool ret = FlightTaskManualAcceleration::update();

	if (_rc_sub < 0) {
		_rc_sub = orb_subscribe(ORB_ID(input_rc));
	}
	if (_rc_sub >= 0) {
    		input_rc_s rc_data;
    		if (orb_copy(ORB_ID(input_rc), _rc_sub, &rc_data) == PX4_OK) {
			// RC channel 5 ve 6 değerleri
			float tf_enable_input = math::constrain(((rc_data.values[4] - 1000.0f) / 1000.0f), 0.0f, 1.0f);
			bool terrain_active = tf_enable_input > 0.5f;

			if (fabsf(tf_enable_input - last_tf_enable_input) > 0.01f) {
            			PX4_INFO("RC Channel 5: %f", (double)tf_enable_input);
            			last_tf_enable_input = tf_enable_input;
        		}

			if (terrain_active && PX4_ISFINITE(_dist_to_bottom)) {
				float alt_input = math::constrain(((rc_data.values[5] - 1000.0f) / 1000.0f), 0.0f, 1.0f);
				float desired_hagl = 0.05f + (0.25f * alt_input);

				// actual clearance measured from the landing gear
				const float dist_from_feet = math::max(_dist_to_bottom - sensor_to_foot_offset, 0.f);

				if (fabsf(alt_input - last_alt_input) > 0.01f) {
					PX4_INFO("RC Channel 6: %.2f : %.2f", (double)alt_input, (double)dist_from_feet);
					last_alt_input = alt_input;
				}

				if (_local_pos_sub < 0) {
				_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
				}

				vehicle_local_position_s local_pos;
				if (orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos) == PX4_OK) {
					float current_x = local_pos.x;
					float current_y = local_pos.y;
					float current_z = _position(2) - (desired_hagl - dist_from_feet);

					if (fabsf(current_x - last_current_x) > 0.01f || fabsf(current_y - last_current_y) > 0.01f) {
						PX4_INFO("Center Koordinate : %.2f : %.2f : %.2f", (double)current_x, (double)current_y, (double)current_z);
						last_current_x = current_x;
						last_current_y = current_y;
					}

					_position_setpoint(2) = _position(2) - (desired_hagl - dist_from_feet);
					_velocity_setpoint(2) = 0.f;
					_acceleration_setpoint(2) = 0.f;
				}
			}
		}
	}

	// Optimize input-to-video latency gimbal control
	if (_gimbal.checkForTelemetry(_time_stamp_current) && haveTakenOff()) {
		_gimbal.acquireGimbalControlIfNeeded();

		// the exact same _yawspeed_setpoint is setpoint for the gimbal and vehicle feed-forward
		const float pitchrate_setpoint = getInputFromSanitizedAuxParameterIndex(_param_mc_slow_map_pitch.get()) * yaw_rate;
		_yawspeed_setpoint = _sticks.getYaw() * yaw_rate;

		_gimbal.publishGimbalManagerSetAttitude(Gimbal::FLAGS_ALL_AXES_LOCKED, Quatf(NAN, NAN, NAN, NAN),
							Vector3f(NAN, pitchrate_setpoint, _yawspeed_setpoint));

		if (_gimbal.allAxesLockedConfirmed()) {
			// but the vehicle makes sure it stays alligned with the gimbal absolute yaw
			_yaw_setpoint = _gimbal.getTelemetryYaw();
		}

	} else {
		_gimbal.releaseGimbalControlIfNeeded();
	}

	return ret;
}

float FlightTaskManualAccelerationSlow::getInputFromSanitizedAuxParameterIndex(int parameter_value)
{
	const int sanitized_index = math::constrain(parameter_value - 1, 0, 5);
	return _sticks.getAux()(sanitized_index);
}

bool FlightTaskManualAccelerationSlow::haveTakenOff()
{
	takeoff_status_s takeoff_status{};
	_takeoff_status_sub.copy(&takeoff_status);

	return takeoff_status.takeoff_state == takeoff_status_s::TAKEOFF_STATE_FLIGHT;
}
