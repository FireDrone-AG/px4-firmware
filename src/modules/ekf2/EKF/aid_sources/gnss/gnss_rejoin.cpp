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
 * @file gnss_rejoin.cpp
 */

#include "aid_sources/gnss/gnss_rejoin.hpp"

#include <mathlib/mathlib.h>

namespace estimator
{

void GnssRejoinWindow::push(const Sample &sample, uint64_t window_us)
{
	if (_count > 0) {
		const uint64_t newest_time = newest().time_us;

		if (sample.time_us <= newest_time) {
			// out of order or duplicate, ignore
			return;
		}

		const uint64_t min_spacing_us = window_us / (kCapacity - 8);

		if (sample.time_us - newest_time < min_spacing_us) {
			return;
		}
	}

	if (_count == kCapacity) {
		// drop the oldest
		_head = (_head + 1) % kCapacity;
		_count--;
	}

	_buf[(_head + _count) % kCapacity] = sample;
	_count++;

	// age out samples older than the window
	while (_count > 1 && (sample.time_us - oldest().time_us) > window_us) {
		_head = (_head + 1) % kCapacity;
		_count--;
	}
}

float GnssRejoinWindow::spanSeconds() const
{
	if (_count < 2) {
		return 0.f;
	}

	return static_cast<float>(newest().time_us - oldest().time_us) * 1e-6f;
}

GnssRejoinWindow::Result GnssRejoinWindow::evaluate(const MapProjection &proj, const EkfNow &ekf,
		const Params &params) const
{
	Result res{};
	res.n = static_cast<uint8_t>(_count);
	res.span_s = spanSeconds();

	if (_count < 2 || !proj.isInitialized()) {
		return res;
	}

	const float n = static_cast<float>(_count);
	const float k = math::max(params.k, 0.f);

	// ---------------------------------------------------------------
	// Heading filter: find dpsi minimising sum |b - R(dpsi) a|^2 with a = EKF velocity, b = GNSS velocity.
	// R(dpsi) is a rotation about the down axis in the NE plane (positive = clockwise seen from above).
	// ---------------------------------------------------------------
	float sum_cross = 0.f;
	float sum_dot = 0.f;
	float sum_aa = 0.f;
	float sum_bb = 0.f;
	float sum_vel_acc_sq = 0.f;

	for (int i = 0; i < _count; i++) {
		const Sample &s = at(i);
		const Vector2f &a = s.ekf_vel;
		const Vector2f &b = s.gnss_vel;
		sum_cross += a(0) * b(1) - a(1) * b(0);
		sum_dot += a.dot(b);
		sum_aa += a.dot(a);
		sum_bb += b.dot(b);
		sum_vel_acc_sq += math::sq(s.vel_acc);
	}

	res.vel_acc_rms = sqrtf(sum_vel_acc_sq / n);
	res.speed_rms = sqrtf(sum_aa / n);

	// residual of a rotation fit: sum |b - R a|^2 = sum|a|^2 + sum|b|^2 - 2 (cos * sum_dot + sin * sum_cross)
	const auto residual_sum = [&](float dpsi) {
		return math::max(sum_aa + sum_bb - 2.f * (cosf(dpsi) * sum_dot + sinf(dpsi) * sum_cross), 0.f);
	};

	const float resid_zero_sum = residual_sum(0.f);
	res.vel_resid_zero = sqrtf(resid_zero_sum / (2.f * n));

	if (sum_aa > FLT_EPSILON) {
		const float dpsi = atan2f(sum_cross, sum_dot);
		const float resid_fit_sum = residual_sum(dpsi);
		res.heading_fit = dpsi;
		res.vel_resid_fit = sqrtf(resid_fit_sum / (2.f * n));

		// variance of the fitted angle: per-axis residual variance / sum |a|^2,
		// inflated for the time correlation of the velocity errors
		const float dt_avg = res.span_s / (n - 1.f);
		const float corr_factor = (dt_avg > FLT_EPSILON) ? math::max(1.f, kVelErrCorrTime / dt_avg) : 1.f;
		const float resid_var = resid_fit_sum / (2.f * (n - 1.f));
		res.heading_fit_std = sqrtf(resid_var / sum_aa * corr_factor);
	}

	const bool fit_ready = PX4_ISFINITE(res.heading_fit_std)
			       && (res.speed_rms >= params.speed_min)
			       && (res.heading_fit_std <= params.heading_std_max)
			       && (res.vel_resid_fit <= k * res.vel_acc_rms);

	const bool ekf_heading_ready = ekf.yaw_align
				       && PX4_ISFINITE(ekf.yaw_std)
				       && (ekf.yaw_std <= params.heading_std_max)
				       && (res.vel_resid_zero <= k * res.vel_acc_rms);

	if (fit_ready) {
		res.heading_source = HeadingSource::kFit;
		res.heading_correction = res.heading_fit;
		res.heading_std = res.heading_fit_std;
		res.heading_ready = true;

	} else if (ekf_heading_ready) {
		res.heading_source = HeadingSource::kEkf;
		res.heading_correction = 0.f;
		res.heading_std = ekf.yaw_std;
		res.heading_ready = true;
	}

	// ---------------------------------------------------------------
	// Altitude filter: carry each GNSS altitude forward with the EKF altitude change
	// ---------------------------------------------------------------
	{
		float sum = 0.f;
		float sum_acc_sq = 0.f;

		// accumulate relative to the first value to keep float precision
		const float ref = at(0).alt + (ekf.alt - at(0).ekf_alt);

		for (int i = 0; i < _count; i++) {
			const Sample &s = at(i);
			sum += (s.alt + (ekf.alt - s.ekf_alt)) - ref;
			sum_acc_sq += math::sq(s.hgt_acc);
		}

		const float mean_rel = sum / n;
		float sum_dev_sq = 0.f;

		for (int i = 0; i < _count; i++) {
			const Sample &s = at(i);
			sum_dev_sq += math::sq((s.alt + (ekf.alt - s.ekf_alt)) - ref - mean_rel);
		}

		res.alt_mean = ref + mean_rel;
		res.alt_scatter = sqrtf(sum_dev_sq / n);
		res.alt_acc_rms = sqrtf(sum_acc_sq / n);
		res.alt_ready = (_count >= kMinSamples) && (res.alt_scatter <= k * res.alt_acc_rms);
	}

	// ---------------------------------------------------------------
	// Position filter: carry each GNSS position forward with the heading corrected EKF displacement
	// ---------------------------------------------------------------
	if (res.heading_ready) {
		const float c = cosf(res.heading_correction);
		const float s_ = sinf(res.heading_correction);

		// GNSS position of sample i carried forward to now (in the projection frame)
		const auto carried = [&](int i) {
			const Sample &s = at(i);
			const Vector2f d = ekf.pos - s.ekf_pos;
			const Vector2f d_rot(c * d(0) - s_ * d(1), s_ * d(0) + c * d(1));
			return Vector2f(proj.project(s.lat, s.lon) + d_rot);
		};

		Vector2f sum{};
		float sum_acc_sq = 0.f;

		for (int i = 0; i < _count; i++) {
			sum += carried(i);
			sum_acc_sq += math::sq(at(i).pos_acc);
		}

		const Vector2f mean = sum / n;
		float sum_dev_sq = 0.f;

		for (int i = 0; i < _count; i++) {
			sum_dev_sq += (carried(i) - mean).norm_squared();
		}

		res.pos_mean = mean;
		res.pos_scatter = sqrtf(sum_dev_sq / (2.f * n));
		res.pos_acc_rms = sqrtf(sum_acc_sq / n);
		res.pos_ready = (_count >= kMinSamples) && (res.pos_scatter <= k * res.pos_acc_rms);
	}

	if (_count < kMinSamples) {
		res.heading_ready = false;
		res.heading_source = HeadingSource::kNone;
	}

	return res;
}

} // namespace estimator
