/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file gnss_rejoin_control.cpp
 * GNSS control in strict mode (EKF2_GPS_MODE = Strict).
 *
 * GNSS is fused only after it has passed the rejoin: a window of EKF2_REQ_GPS_H seconds of
 * samples passing the strict checks in which the heading, position and altitude filters are
 * all consistent on the same sample. The join (heading -> position -> altitude -> start of
 * fusion) is done in a single call. While in use, any sample failing the strict checks, no data
 * or no fusion for EKF2_GPS_RJ_TOUT stops all GNSS fusion, and GNSS can only come back through
 * the rejoin. There are no GNSS resets while in use, no fault latches and no EKF-GSF.
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGnssStrictMode(const imuSample &imu_delayed, const bool shadow)
{
	GnssRejoinStatus &st = _gnss_rj_status;
	st.shadow = shadow;

	if (!_gnss_strict_mode_active) {
		// entering strict mode: drop the latches of the other modes and start collecting from scratch
		_gnss_strict_mode_active = true;
		_control_status.flags.gnss_fault = false;
		_control_status.flags.gnss_hgt_fault = false;
		_yawEstimator.reset();
		clearGnssRejoinWindow(false);
		_gnss_rj_reset_count_seen = _state_reset_status.reset_count;
	}

	// shadow: evaluate all three filters as if GNSS velocity, position and height were enabled
	const bool vel_enabled = shadow || (_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::VEL));
	const bool pos_enabled = shadow || (_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::HPOS));
	const bool hgt_enabled = shadow || (_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::VPOS));

	st.heading_required = vel_enabled || pos_enabled;
	st.pos_required = pos_enabled;
	st.alt_required = hgt_enabled;

	const uint64_t timeout_us = static_cast<uint64_t>(math::max(_params.ekf2_gps_rj_tout, 0.1f) * 1e6f);

	_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_gps_data_ready = _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed);

	if (_gps_data_ready) {
		const gnssSample &gnss_sample = _gps_sample_delayed;

		const bool initial_checks_passed_prev = _gnss_checks.initialChecksPassed();
		_gnss_checks.run(gnss_sample, _time_delayed_us);

		if (_gnss_checks.initialChecksPassed() && !initial_checks_passed_prev) {
			_information_events.flags.gps_checks_passed = true;
		}

		updateGnssPos(gnss_sample, _aid_src_gnss_pos);
		updateGnssVel(imu_delayed, gnss_sample, _aid_src_gnss_vel);

		st.time_us = gnss_sample.time_us;
		st.sample_failed_checks = _gnss_checks.lastSampleFailed();
		st.checks_passed = _gnss_checks.passed();
		st.time_since_check_fail = (_gnss_checks.getLastFailUs() != 0)
					   ? static_cast<float>(_time_delayed_us - _gnss_checks.getLastFailUs()) * 1e-6f : NAN;
		st.preconditions_ok = isGnssRejoinPreconditionPassing(shadow);

		const bool in_use = _control_status.flags.gnss_vel || _control_status.flags.gnss_pos
				    || _control_status.flags.gps_hgt;

		if (st.sample_failed_checks) {
			// do not use this sample anywhere (height fusion runs later in this cycle)
			_gps_data_ready = false;

			if (in_use) {
				ECL_WARN("GNSS failed strict checks, stopping use");
				stopGnssStrict(GnssRejoinStopReason::kChecks);

			} else {
				// the window must contain EKF2_REQ_GPS_H of clean samples
				clearGnssRejoinWindow();
			}

		} else if (in_use) {
			if (_control_status.flags.gnss_vel) {
				if (vel_enabled) {
					fuseVelocity(_aid_src_gnss_vel);

				} else {
					stopGnssVelFusion();
				}
			}

			if (_control_status.flags.gnss_pos) {
				if (pos_enabled) {
					fuseHorizontalPosition(_aid_src_gnss_pos);

				} else {
					stopGnssPosFusion();
				}
			}

			// height is fused by controlGnssHeightFusion()

		} else if (st.preconditions_ok) {
			pushGnssRejoinSample(gnss_sample);

			MapProjection proj{};
			GnssRejoinWindow::Result res{};
			evaluateGnssRejoin(proj, res);
			st.window = res;
			st.pos_offset = (_local_origin_lat_lon.isInitialized() && PX4_ISFINITE(res.pos_scatter))
					? Vector2f(res.pos_mean - getLocalHorizontalPosition()) : Vector2f(NAN, NAN);
			st.alt_offset = res.alt_mean - _gpos.altitude();

			const float window_s = math::max(static_cast<float>(_min_gps_health_time_us) * 1e-6f, 1.f);

			const bool window_full = _gnss_checks.passed()
						 && (res.span_s >= 0.9f * window_s)
						 && (res.n >= GnssRejoinWindow::kMinSamples);

			const bool all_ready = (st.heading_required || st.pos_required || st.alt_required)
					       && (!st.heading_required || res.heading_ready)
					       && (!st.pos_required || res.pos_ready)
					       && (!st.alt_required || res.alt_ready);

			if (shadow) {
				// never fuse, keep the window running and log when GNSS would have joined
				if (window_full && all_ready && !st.shadow_would_join) {
					st.shadow_join_count++;
				}

				st.shadow_would_join = window_full && all_ready;

			} else if (window_full && all_ready) {
				st.shadow_would_join = false;
				joinGnss(res, proj);
			}

		} else {
			// rejoin not possible (e.g. no other horizontal aiding to carry samples forward)
			clearGnssRejoinWindow();
			st.shadow_would_join = false;
		}

		if (st.sample_failed_checks) {
			st.shadow_would_join = false;
		}

		if (shadow) {
			// the sample must not be used by any other GNSS fusion
			_gps_data_ready = false;
		}

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_gps_data_ready) {
			controlGnssYawFusion(gnss_sample);
		}

#endif // CONFIG_EKF2_GNSS_YAW
	}

	// monitor GNSS while in use, also between samples
	const bool in_use = _control_status.flags.gnss_vel || _control_status.flags.gnss_pos
			    || _control_status.flags.gps_hgt;

	if (in_use) {
		const bool alignment_ok = _control_status.flags.tilt_align
					  && (_control_status.flags.yaw_align
					      || !(_control_status.flags.gnss_vel || _control_status.flags.gnss_pos));

		const bool fusion_timeout = (_control_status.flags.gnss_vel && isTimedOut(_aid_src_gnss_vel.time_last_fuse, timeout_us))
					    || (_control_status.flags.gnss_pos && isTimedOut(_aid_src_gnss_pos.time_last_fuse, timeout_us))
					    || (_control_status.flags.gps_hgt && isTimedOut(_aid_src_gnss_hgt.time_last_fuse, timeout_us));

		if (!alignment_ok) {
			ECL_WARN("GNSS stopped, alignment lost");
			stopGnssStrict(GnssRejoinStopReason::kPrerequisites);

		} else if (!isNewestSampleRecent(_time_last_gps_buffer_push, timeout_us)) {
			ECL_WARN("GNSS data stopped");
			stopGnssStrict(GnssRejoinStopReason::kNoData);

		} else if (fusion_timeout) {
			ECL_WARN("GNSS fusion rejected for %.1fs, stopping use", (double)_params.ekf2_gps_rj_tout);
			stopGnssStrict(GnssRejoinStopReason::kRejected);
		}
	}

	st.in_use = _control_status.flags.gnss_vel || _control_status.flags.gnss_pos || _control_status.flags.gps_hgt;

	const auto time_since = [this](bool active, uint64_t t) {
		return (active && (t != 0) && (_time_delayed_us >= t)) ? static_cast<float>(_time_delayed_us - t) * 1e-6f : NAN;
	};

	st.time_since_vel_fuse = time_since(_control_status.flags.gnss_vel, _aid_src_gnss_vel.time_last_fuse);
	st.time_since_pos_fuse = time_since(_control_status.flags.gnss_pos, _aid_src_gnss_pos.time_last_fuse);
	st.time_since_hgt_fuse = time_since(_control_status.flags.gps_hgt, _aid_src_gnss_hgt.time_last_fuse);

	updateGnssRejoinPilotState();
}

bool Ekf::isGnssRejoinPreconditionPassing(const bool shadow) const
{
	// The window carries GNSS samples forward with the EKF displacement, which must come
	// from another aiding source (radar/vision velocity or position).
	return _control_status.flags.tilt_align
	       && isHorizontalAidingActive()
	       && isLocalVerticalPositionValid()
	       && (shadow || (_params.ekf2_gps_ctrl & (static_cast<int32_t>(GnssCtrl::VEL)
			       | static_cast<int32_t>(GnssCtrl::HPOS)
			       | static_cast<int32_t>(GnssCtrl::VPOS))));
}

void Ekf::pushGnssRejoinSample(const gnssSample &gnss_sample)
{
	// The sample is up to one IMU step older than the EKF state: move the EKF state back to the sample time
	const float dt = math::constrain(static_cast<float>(static_cast<int64_t>(_time_delayed_us)
					 - static_cast<int64_t>(gnss_sample.time_us)) * 1e-6f, 0.f, 0.1f);

	const Vector3f pos_offset_body = gnss_sample.pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	GnssRejoinWindow::Sample s{};
	s.time_us = gnss_sample.time_us;

	// antenna offset corrected observations computed by updateGnssPos() and updateGnssVel()
	s.lat = _aid_src_gnss_pos.observation[0];
	s.lon = _aid_src_gnss_pos.observation[1];
	s.alt = gnss_sample.alt + pos_offset_earth(2);
	s.gnss_vel = Vector2f(_aid_src_gnss_vel.observation[0], _aid_src_gnss_vel.observation[1]);

	// same accuracies as used for fusion
	s.vel_acc = math::max(gnss_sample.sacc, _params.ekf2_gps_v_noise, 0.01f);
	s.pos_acc = math::max(gnss_sample.hacc, _params.ekf2_gps_p_noise, 0.01f);
	s.hgt_acc = math::max(gnss_sample.vacc, 1.5f * _params.ekf2_gps_p_noise, 0.01f);

	s.ekf_pos = getLocalHorizontalPosition() - Vector2f(_state.vel.xy()) * dt - _gnss_rj_pos_accum;
	s.ekf_alt = _gpos.altitude() + _state.vel(2) * dt - _gnss_rj_alt_accum;
	s.ekf_vel = Vector2f(_state.vel.xy());

	if (!(s.gnss_vel.isAllFinite() && s.ekf_pos.isAllFinite() && s.ekf_vel.isAllFinite()
	      && PX4_ISFINITE(s.alt) && PX4_ISFINITE(s.ekf_alt))) {
		return;
	}

	const uint64_t window_us = math::max(static_cast<uint64_t>(_min_gps_health_time_us), static_cast<uint64_t>(1e6));
	_gnss_rj_window.push(s, window_us);
}

void Ekf::evaluateGnssRejoin(MapProjection &proj, GnssRejoinWindow::Result &res) const
{
	if (_gnss_rj_window.size() == 0) {
		res = GnssRejoinWindow::Result{};
		return;
	}

	if (_local_origin_lat_lon.isInitialized()) {
		proj = _local_origin_lat_lon;

	} else {
		// no origin yet: temporary reference, only the carried-forward mean is used (to set the origin)
		proj.initReference(_gnss_rj_window.newest().lat, _gnss_rj_window.newest().lon, _time_delayed_us);
	}

	GnssRejoinWindow::EkfNow now{};
	now.pos = getLocalHorizontalPosition() - _gnss_rj_pos_accum;
	now.alt = _gpos.altitude() - _gnss_rj_alt_accum;
	now.yaw_align = _control_status.flags.yaw_align;
	now.yaw_std = sqrtf(math::max(getYawVar(), 0.f));

	GnssRejoinWindow::Params params{};
	params.k = _params.ekf2_gps_rj_k;
	params.heading_std_max = math::radians(_params.ekf2_gps_rj_ysd);
	params.speed_min = _params.ekf2_gps_rj_vmin;

	res = _gnss_rj_window.evaluate(proj, now, params);
}

void Ekf::joinGnss(const GnssRejoinWindow::Result &res, const MapProjection &proj)
{
	GnssRejoinStatus &st = _gnss_rj_status;

	// Everything below uses values evaluated on the same window snapshot. A heading reset does not
	// change the position or altitude states and a position reset does not change the altitude, so
	// the order heading -> position -> altitude keeps all offsets valid.

	// 1. heading
	if (st.heading_required) {
		const float dpsi = res.heading_correction;

		if ((res.heading_source == GnssRejoinWindow::HeadingSource::kFit)
		    && (!_control_status.flags.yaw_align || (fabsf(dpsi) > math::radians(_params.ekf2_gps_rj_yrst)))) {

			const float yaw_new = wrap_pi(getEulerYaw(_R_to_earth) + dpsi);
			ECL_INFO("GNSS rejoin: heading reset %.1f deg", (double)math::degrees(dpsi));
			resetQuatStateYaw(yaw_new, sq(res.heading_std));
			_information_events.flags.yaw_aligned_to_imu_gps = true;
			st.heading_reset_count++;
		}

		_control_status.flags.yaw_align = true;
		st.join_heading_correction = dpsi;
		st.join_heading_source = res.heading_source;
	}

	// 2. horizontal position
	if (st.pos_required) {
		double lat = 0.0;
		double lon = 0.0;
		proj.reproject(res.pos_mean(0), res.pos_mean(1), lat, lon);

		if (!_local_origin_lat_lon.isInitialized()) {
			// first GNSS position: place the origin so that the local position does not move
			resetLatLonTo(lat, lon);
			st.join_pos_offset.setZero();

		} else {
			const Vector2f offset = res.pos_mean - getLocalHorizontalPosition();
			st.join_pos_offset = offset;

			if (offset.longerThan(_params.ekf2_gps_rj_prst)) {
				ECL_INFO("GNSS rejoin: position reset (%.2f, %.2f) m", (double)offset(0), (double)offset(1));
				_information_events.flags.reset_pos_to_gps = true;
				const float var = sq(res.pos_acc_rms);
				resetHorizontalPositionTo(lat, lon, Vector2f(var, var));
				st.pos_reset_count++;
			}
		}

		_aid_src_gnss_pos.time_last_fuse = _time_delayed_us;
		_control_status.flags.gnss_pos = true;
	}

	// velocity: never reset, the heading reset already rotated it
	if (_params.ekf2_gps_ctrl & static_cast<int32_t>(GnssCtrl::VEL)) {
		_aid_src_gnss_vel.time_last_fuse = _time_delayed_us;
		_control_status.flags.gnss_vel = true;
	}

	// 3. altitude
	if (st.alt_required) {
		const bool gnss_is_hgt_ref = (_params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::GNSS));
		const float offset = res.alt_mean - _gpos.altitude();
		st.join_alt_offset = offset;

		if (!PX4_ISFINITE(_local_origin_alt)) {
			// first GNSS altitude: move the datum only, local z does not change and no reset is counted
			ECL_INFO("GNSS rejoin: altitude origin %.1f m", (double)offset);
			shiftAltitudeDatum(offset);

		} else if (gnss_is_hgt_ref && (fabsf(offset) > _params.ekf2_gps_rj_hrst)) {
			ECL_INFO("GNSS rejoin: altitude reset %.2f m", (double)offset);
			_information_events.flags.reset_hgt_to_gps = true;
			resetAltitudeTo(res.alt_mean, sq(res.alt_acc_rms));
			st.alt_reset_count++;
		}

		if (gnss_is_hgt_ref) {
			_height_sensor_ref = HeightSensor::GNSS;
			_gps_hgt_b_est.reset();

		} else {
			_gps_hgt_b_est.setBias(res.alt_mean - _gpos.altitude());
		}

		_gps_hgt_b_est.setFusionActive();
		_aid_src_gnss_hgt.time_last_fuse = _time_delayed_us;
		_control_status.flags.gps_hgt = true;
		_control_status.flags.gnss_hgt_fault = false;
	}

	ECL_INFO("GNSS rejoin: starting GNSS fusion");
	_information_events.flags.starting_gps_fusion = true;
	_control_status.flags.gnss_fault = false;
	st.join_count++;
	st.last_stop_reason = GnssRejoinStopReason::kNone;

	clearGnssRejoinWindow(false);
}

void Ekf::stopGnssStrict(GnssRejoinStopReason reason)
{
	stopGnssFusion();

	// GNSS must pass the strict checks for the full health time again before it can rejoin
	_gnss_checks.reset();

	_gnss_rj_status.stop_count++;
	_gnss_rj_status.last_stop_reason = reason;

	clearGnssRejoinWindow(false);
}

void Ekf::clearGnssRejoinWindow(bool count)
{
	if (count && (_gnss_rj_window.size() > 0)) {
		_gnss_rj_status.window_clear_count++;
	}

	_gnss_rj_window.clear();
	_gnss_rj_pos_accum.setZero();
	_gnss_rj_alt_accum = 0.f;
	_gnss_rj_status.window = GnssRejoinWindow::Result{};
	_gnss_rj_status.pos_offset = Vector2f(NAN, NAN);
	_gnss_rj_status.alt_offset = NAN;
}

void Ekf::updateGnssRejoinResetTracking()
{
	if (!_gnss_strict_mode_active) {
		return;
	}

	const StateResetCounts &rc = _state_reset_status.reset_count;
	bool clear = false;

	// origin changed (external command or our own join): stored local positions are not comparable anymore
	const bool origin_initialized = _local_origin_lat_lon.isInitialized();
	const uint64_t origin_time = origin_initialized ? _local_origin_lat_lon.getProjectionReferenceTimestamp() : 0;
	const bool origin_alt_changed = (PX4_ISFINITE(_local_origin_alt) != PX4_ISFINITE(_gnss_rj_origin_alt))
					|| (PX4_ISFINITE(_local_origin_alt) && (fabsf(_local_origin_alt - _gnss_rj_origin_alt) > 1e-3f));

	if ((origin_initialized != _gnss_rj_origin_initialized)
	    || (origin_time != _gnss_rj_origin_time)
	    || origin_alt_changed) {
		clear = true;
	}

	// heading reset not done by the rejoin: the stored EKF velocities and displacements are in another frame
	if (rc.quat != _gnss_rj_reset_count_seen.quat) {
		clear = true;
	}

	// position and height resets: keep the stored displacements continuous. The reset deltas are only
	// available for resets that happened during this cycle, otherwise start over.
	if (rc.posNE != _gnss_rj_reset_count_seen.posNE) {
		if (static_cast<uint8_t>(rc.posNE - _gnss_rj_reset_count_seen.posNE)
		    == static_cast<uint8_t>(rc.posNE - _state_reset_count_prev.posNE)) {
			_gnss_rj_pos_accum += _state_reset_status.posNE_change;

		} else {
			clear = true;
		}
	}

	if (rc.posD != _gnss_rj_reset_count_seen.posD) {
		if (static_cast<uint8_t>(rc.posD - _gnss_rj_reset_count_seen.posD)
		    == static_cast<uint8_t>(rc.posD - _state_reset_count_prev.posD)) {
			// posD_change is the change of the down position, altitude changes by the opposite
			_gnss_rj_alt_accum -= _state_reset_status.posD_change;

		} else {
			clear = true;
		}
	}

	_gnss_rj_reset_count_seen = rc;
	_gnss_rj_origin_initialized = origin_initialized;
	_gnss_rj_origin_time = origin_time;
	_gnss_rj_origin_alt = _local_origin_alt;

	if (clear) {
		clearGnssRejoinWindow();
	}
}

void Ekf::updateGnssRejoinPilotState()
{
	GnssRejoinStatus &st = _gnss_rj_status;

	if (st.shadow) {
		// GNSS is not used
		st.pilot_state = 0;

	} else if (st.in_use) {
		st.pilot_state = 2;

	} else if (st.checks_passed && st.preconditions_ok
		   && isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL)) {
		st.pilot_state = 1;

	} else {
		st.pilot_state = 0;
	}
}

void Ekf::shiftAltitudeDatum(const float delta_alt)
{
	if (!PX4_ISFINITE(delta_alt)) {
		return;
	}

	_local_origin_alt = (PX4_ISFINITE(_local_origin_alt) ? _local_origin_alt : 0.f) + delta_alt;

	_gpos.setAltitude(_gpos.altitude() + delta_alt);
	_state.pos(2) = -_gpos.altitude();
	_output_predictor.resetAltitudeTo(_gpos.altitude(), 0.f);

	_last_known_gpos.setAltitude(_last_known_gpos.altitude() + delta_alt);
	_last_on_ground_posD -= delta_alt;

	// bias = measurement - altitude
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() - delta_alt);

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_b_est.setBias(_baro_b_est.getBias() - delta_alt);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// bias = measurement (down) + altitude
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() + delta_alt);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_TERRAIN)
	// terrain is stored as minus the terrain altitude, height above ground is unchanged
	_state.terrain -= delta_alt;
#endif // CONFIG_EKF2_TERRAIN
}

void Ekf::resetNavigationFrameAfterFailstate()
{
	if (_local_origin_lat_lon.isInitialized()) {
		// keep the local position, store it in the zero reference convention used without origin
		const Vector2f pos = getLocalHorizontalPosition();
		_local_origin_lat_lon = MapProjection{};

		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		double lat = 0.0;
		double lon = 0.0;
		zero_ref.reproject(pos(0), pos(1), lat, lon);

		_gpos.setLatLonDeg(lat, lon);
		_output_predictor.resetLatLonTo(lat, lon);
		_last_known_gpos.setLatLonDeg(lat, lon);

		ECL_WARN("failstate restart: global origin cleared");
	}

	if (PX4_ISFINITE(_local_origin_alt)) {
		// altitude becomes local altitude again, local z unchanged
		shiftAltitudeDatum(-_local_origin_alt);
		_local_origin_alt = NAN;
	}

	if (_control_status.flags.yaw_align) {
		ECL_WARN("failstate restart: heading alignment cleared");
		_control_status.flags.yaw_align = false;
	}
}
