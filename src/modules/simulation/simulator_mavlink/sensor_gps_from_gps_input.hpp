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
 * @file sensor_gps_from_gps_input.hpp
 *
 * Mapping of the MAVLink GPS_INPUT (#232) message into a sensor_gps uORB
 * message, shared by the simulation input drivers (simulator_mavlink for SITL
 * and the VOXL2 dsp_hitl driver for HITL).
 *
 * Unlike HIL_GPS, GPS_INPUT carries the receiver's claimed quality (DOPs,
 * horizontal/vertical/speed accuracy) so a simulator can control the whole
 * GPS quality contract seen by the estimator.
 *
 * Field mapping (GPS_INPUT -> sensor_gps):
 *   lat, lon [degE7]                 -> latitude_deg, longitude_deg [deg]
 *   alt [m, MSL]                     -> altitude_msl_m and altitude_ellipsoid_m [m]
 *                                       (same value, as in the HIL_GPS path)
 *   fix_type                         -> fix_type (passed through)
 *   satellites_visible               -> satellites_used
 *   hdop, vdop [-]                   -> hdop, vdop [-]
 *   horiz_accuracy, vert_accuracy [m]-> eph, epv [m]
 *   speed_accuracy [m/s]             -> s_variance_m_s [m/s]
 *   vn, ve, vd [m/s]                 -> vel_n_m_s, vel_e_m_s, vel_d_m_s [m/s]
 *                                       vel_m_s = |(vn, ve)|, cog_rad = atan2(ve, vn)
 *                                       vel_ned_valid = true
 *   time_week, time_week_ms [GPS]    -> time_utc_usec [us, UTC]
 *                                       (GPS epoch 1980-01-06 minus 18 leap seconds;
 *                                        0 if both fields are 0)
 *   yaw [cdeg, 0 = not available]    -> heading [rad, -pi..pi], NaN if yaw == 0
 *                                       (36000 = north = 0 rad)
 *   (not in GPS_INPUT)               -> c_variance_rad = 0.5, heading_offset = NaN,
 *                                       heading_accuracy = NaN (EKF2 then uses
 *                                       EKF2_GPS_YAW_NOISE)
 *   time_usec, gps_id                -> not mapped here; device_id and timestamps are
 *                                       set by the caller, as for HIL_GPS.
 *
 * GPS_INPUT_IGNORE_FLAG_* handling (an ignored field gets the value the HIL_GPS
 * path produces when that information is not provided):
 *   ALT                 -> altitudes NaN and fix_type capped at 2D, so no consumer
 *                          treats the missing altitude as a 3D solution
 *   HDOP / VDOP         -> 0 (HIL_GPS hard-codes 0); a DOP of UINT16_MAX or a
 *                          non-finite DOP (MAVLink "unknown") is treated the same
 *   SPEED_ACCURACY      -> 0.25 m/s (HIL_GPS hard-codes 0.25)
 *   HORIZONTAL_ACCURACY -> 655.35 m (HIL_GPS maps an unknown eph of UINT16_MAX cm
 *   VERTICAL_ACCURACY      to this value)
 *   VEL_HORIZ           -> vel_n/vel_e/vel_m_s = 0, cog_rad = NaN, vel_ned_valid = false
 *   VEL_VERT            -> vel_d = 0, vel_ned_valid = false
 *   (EKF2 discards samples with vel_ned_valid == false.)
 */

#pragma once

#include <math.h>
#include <stdint.h>

#include <mavlink.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <uORB/topics/sensor_gps.h>

namespace simulation
{

// Defaults used by the HIL_GPS path for information that message does not carry.
static constexpr float kGpsInputDefaultSpeedAccuracy = 0.25f;              // [m/s]
static constexpr float kGpsInputDefaultCourseAccuracy = 0.5f;              // [rad]
static constexpr float kGpsInputUnknownPosAccuracy = UINT16_MAX * 1e-2f;   // [m]

static constexpr uint64_t kGpsEpochUnixSec = 315964800ULL;   // 1980-01-06T00:00:00Z
static constexpr uint64_t kGpsUtcLeapSec = 18ULL;            // GPS - UTC since 2017-01-01
static constexpr uint64_t kSecondsPerWeek = 7ULL * 24ULL * 3600ULL;

// Plain integer test: some includers define a templated enum operator&.
static inline bool gps_input_ignored(uint16_t ignore_flags, int flag)
{
	return (static_cast<unsigned>(ignore_flags) & static_cast<unsigned>(flag)) != 0u;
}

static inline float gps_input_dop(float dop, bool ignored)
{
	if (ignored || !PX4_ISFINITE(dop) || dop >= static_cast<float>(UINT16_MAX)) {
		return 0.f;
	}

	return dop;
}

/**
 * Fill gps from a decoded GPS_INPUT message. device_id, timestamp and
 * timestamp_sample are left untouched for the caller to set.
 */
static inline void sensor_gps_from_gps_input(const mavlink_gps_input_t &in, sensor_gps_s &gps)
{
	const uint16_t ignore = in.ignore_flags;

	gps.latitude_deg = in.lat * 1e-7;
	gps.longitude_deg = in.lon * 1e-7;

	gps.fix_type = in.fix_type;
	gps.satellites_used = in.satellites_visible;

	if (gps_input_ignored(ignore, GPS_INPUT_IGNORE_FLAG_ALT)) {
		gps.altitude_msl_m = static_cast<double>(NAN);
		gps.altitude_ellipsoid_m = static_cast<double>(NAN);

		if (gps.fix_type > sensor_gps_s::FIX_TYPE_2D) {
			gps.fix_type = sensor_gps_s::FIX_TYPE_2D;
		}

	} else {
		gps.altitude_msl_m = static_cast<double>(in.alt);
		gps.altitude_ellipsoid_m = static_cast<double>(in.alt);
	}

	gps.hdop = gps_input_dop(in.hdop, gps_input_ignored(ignore, GPS_INPUT_IGNORE_FLAG_HDOP));
	gps.vdop = gps_input_dop(in.vdop, gps_input_ignored(ignore, GPS_INPUT_IGNORE_FLAG_VDOP));

	gps.eph = gps_input_ignored(ignore, GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY)
		  ? kGpsInputUnknownPosAccuracy : in.horiz_accuracy;
	gps.epv = gps_input_ignored(ignore, GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)
		  ? kGpsInputUnknownPosAccuracy : in.vert_accuracy;
	gps.s_variance_m_s = gps_input_ignored(ignore, GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY)
			     ? kGpsInputDefaultSpeedAccuracy : in.speed_accuracy;
	gps.c_variance_rad = kGpsInputDefaultCourseAccuracy;

	const bool vel_horiz_valid = !gps_input_ignored(ignore, GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
	const bool vel_vert_valid = !gps_input_ignored(ignore, GPS_INPUT_IGNORE_FLAG_VEL_VERT);

	if (vel_horiz_valid) {
		gps.vel_n_m_s = in.vn;
		gps.vel_e_m_s = in.ve;
		gps.vel_m_s = sqrtf(in.vn * in.vn + in.ve * in.ve);
		gps.cog_rad = atan2f(in.ve, in.vn);

	} else {
		gps.vel_n_m_s = 0.f;
		gps.vel_e_m_s = 0.f;
		gps.vel_m_s = 0.f;
		gps.cog_rad = NAN;
	}

	gps.vel_d_m_s = vel_vert_valid ? in.vd : 0.f;
	gps.vel_ned_valid = vel_horiz_valid && vel_vert_valid;

	if (in.time_week == 0 && in.time_week_ms == 0) {
		gps.time_utc_usec = 0;

	} else {
		const uint64_t gps_sec = kGpsEpochUnixSec + static_cast<uint64_t>(in.time_week) * kSecondsPerWeek;
		gps.time_utc_usec = (gps_sec - kGpsUtcLeapSec) * 1000000ULL
				    + static_cast<uint64_t>(in.time_week_ms) * 1000ULL;
	}

	gps.timestamp_time_relative = 0;

	gps.heading = (in.yaw == 0) ? NAN : matrix::wrap_pi(math::radians(in.yaw * 1e-2f));
	gps.heading_offset = NAN;
	gps.heading_accuracy = NAN;

	gps.noise_per_ms = 0;
	gps.automatic_gain_control = 0;
	gps.jamming_indicator = 0;
	gps.jamming_state = 0;
	gps.spoofing_state = 0;
}

} // namespace simulation
