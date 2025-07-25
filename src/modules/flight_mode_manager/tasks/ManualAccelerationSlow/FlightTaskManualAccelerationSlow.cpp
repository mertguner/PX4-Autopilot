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
#include <geo/geo.h>

using namespace time_literals;
using namespace matrix;

void set_max_tilt_angle(float degrees) {
    param_t tiltmax_air_handle = param_find("MPC_TILTMAX_AIR");
    if (tiltmax_air_handle != PARAM_INVALID) {
        param_set(tiltmax_air_handle, &degrees);
        // Değişikliklerin etkili olması için gerekirse param_save_default() fonksiyonu çağırabilirsin.
    }
}

void FlightTaskManualAccelerationSlow::update_gps_history(const LocalXYZ &current_gps)
{
    // 1) Mevcut tüm elemanlarla mesafeyi kontrol et
    bool is_far_enough = true;
    for (size_t i = 0; i < _gps_history_len; ++i) {
        float dx = _gps_history[i].x - current_gps.x;
        float dy = _gps_history[i].y - current_gps.y;
        float distance = sqrtf(dx * dx + dy * dy);
        if (distance < MIN_DISTANCE_M) {
            is_far_enough = false;
            break;
        }
    }
    if (!is_far_enough)
        return; // Eklenmeyecek

    // 2) Doluysa en uzak noktayı bul ve çıkar
    if (_gps_history_len >= MAX_HISTORY_SIZE) {
        float max_dist = -1.0f;
        size_t farthest_idx = 0;
        for (size_t i = 0; i < _gps_history_len; ++i) {
            float dx = _gps_history[i].x - current_gps.x;
            float dy = _gps_history[i].y - current_gps.y;
            float distance = sqrtf(dx * dx + dy * dy);
            if (distance > max_dist) {
                max_dist = distance;
                farthest_idx = i;
            }
        }
        // En uzaktakini çıkarmak için tüm elemanları bir sola kaydır
        for (size_t i = farthest_idx; i + 1 < _gps_history_len; ++i) {
            _gps_history[i] = _gps_history[i + 1];
        }
        --_gps_history_len;
    }

    // 3) Yeni koordinatı sona ekle
    if (_gps_history_len < MAX_HISTORY_SIZE) {
        _gps_history[_gps_history_len++] = current_gps;
    }
}

bool FlightTaskManualAccelerationSlow::is_point_in_rotated_square(float x, float y, float cx, float cy, float yaw_rad)
{
    float dx = x - cx;
    float dy = y - cy;

    float cos_yaw = cosf(-yaw_rad);
    float sin_yaw = sinf(-yaw_rad);

    float local_x = cos_yaw * dx - sin_yaw * dy;
    float local_y = sin_yaw * dx + cos_yaw * dy;

    return (fabsf(local_x) <= AREA_HALF && fabsf(local_y) <= AREA_HALF);
}

float FlightTaskManualAccelerationSlow::find_points_in_area(const LocalXYZ &center, float yaw_rad)
{
    float result = center.z;
    float min = center.z;

    for (size_t i = 0; i < _gps_history_len; ++i) {
        const LocalXYZ &pt = _gps_history[i];
        if (is_point_in_rotated_square(pt.x, pt.y, center.x, center.y, yaw_rad) && min > pt.z) {
            result = pt.z;
            min = pt.z;
        }
    }
    return result;
}

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

					if (_att_sub < 0) {
				 		_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
					}

					vehicle_attitude_s att = {};
					if (orb_copy(ORB_ID(vehicle_attitude), _att_sub, &att) == PX4_OK) {
						matrix::Quatf q(att.q);
						float drone_yaw = matrix::Eulerf(q).psi();
						// Yaw değeriniz burada!

						LocalXYZ localCenterXYZ;
						localCenterXYZ.x = local_pos.x + _param_ekf2_of_pos_x.get();
						localCenterXYZ.y = local_pos.y + _param_ekf2_of_pos_y.get();
						localCenterXYZ.z = _position(2) - (desired_hagl - dist_from_feet);
						LocalXYZ localXYZ;
						localXYZ.x = local_pos.x + _param_ekf2_of_pos_x.get();
						localXYZ.y = local_pos.y + _param_ekf2_of_pos_y.get();
						localXYZ.z = localCenterXYZ.z;
						update_gps_history(localXYZ);

						if (fabsf(localCenterXYZ.x  - last_current_x) > 0.01f || fabsf(localCenterXYZ.y - last_current_y) > 0.01f) {
							PX4_INFO("Center Koordinate : %.2f : %.2f : %.2f", (double)localCenterXYZ.x, (double)localCenterXYZ.y, (double)localCenterXYZ.z);
							last_current_x = localCenterXYZ.x;
							last_current_y = localCenterXYZ.y;
						}

						_position_setpoint(2) = find_points_in_area(localCenterXYZ, drone_yaw);
					}
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
