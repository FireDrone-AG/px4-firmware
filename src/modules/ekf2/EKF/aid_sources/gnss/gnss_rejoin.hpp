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
 * @file gnss_rejoin.hpp
 * Sliding window and consistency filters used to (re)join GNSS in strict mode.
 *
 * GNSS samples are stored together with the EKF state at the same time while GNSS is
 * not fused. Three filters are evaluated over the window:
 *  - heading: rotation between EKF (non-GNSS aided) velocity and GNSS velocity
 *  - position: GNSS positions carried forward to now with the rotated EKF displacement
 *  - altitude: GNSS altitudes carried forward to now with the EKF altitude change
 * Each filter is consistent if its RMS scatter is at most K times the RMS reported accuracy.
 */

#ifndef EKF_GNSS_REJOIN_HPP
#define EKF_GNSS_REJOIN_HPP

#include <lib/geo/geo.h>
#include <matrix/math.hpp>

namespace estimator
{

using matrix::Vector2f;

class GnssRejoinWindow final
{
public:
	struct Sample {
		uint64_t time_us{0};
		double lat{0.0};        ///< GNSS latitude, antenna offset corrected (deg)
		double lon{0.0};        ///< GNSS longitude, antenna offset corrected (deg)
		float alt{0.f};         ///< GNSS altitude AMSL, antenna offset corrected (m)
		Vector2f gnss_vel{};    ///< GNSS NE velocity, antenna offset corrected (m/s)
		float vel_acc{0.f};     ///< 1-sigma GNSS velocity accuracy used for fusion (m/s)
		float pos_acc{0.f};     ///< 1-sigma GNSS horizontal position accuracy used for fusion (m)
		float hgt_acc{0.f};     ///< 1-sigma GNSS altitude accuracy used for fusion (m)
		Vector2f ekf_pos{};     ///< EKF local NE position at the sample time, minus reset accumulator (m)
		float ekf_alt{0.f};     ///< EKF altitude at the sample time, minus reset accumulator (m)
		Vector2f ekf_vel{};     ///< EKF NE velocity at the sample time (m/s)
	};

	enum class HeadingSource : uint8_t {
		kNone = 0,
		kFit  = 1, ///< heading correction fitted from GNSS vs EKF velocity
		kEkf  = 2  ///< EKF heading still good, zero correction
	};

	struct Params {
		float k;               ///< consistency factor
		float heading_std_max; ///< (rad)
		float speed_min;       ///< (m/s)
	};

	struct EkfNow {
		Vector2f pos;          ///< current EKF local NE position, minus reset accumulator (m)
		float alt;             ///< current EKF altitude, minus reset accumulator (m)
		bool yaw_align;
		float yaw_std;         ///< (rad)
	};

	struct Result {
		uint8_t n{0};
		float span_s{0.f};

		// heading
		HeadingSource heading_source{HeadingSource::kNone};
		float heading_correction{0.f};  ///< chosen correction (rad)
		float heading_std{NAN};         ///< std of the chosen correction (rad)
		float heading_fit{NAN};         ///< fitted correction (rad)
		float heading_fit_std{NAN};     ///< (rad)
		float speed_rms{NAN};           ///< RMS EKF horizontal speed (m/s)
		float vel_resid_fit{NAN};       ///< RMS velocity residual per axis using the fit (m/s)
		float vel_resid_zero{NAN};      ///< RMS velocity residual per axis using zero correction (m/s)
		float vel_acc_rms{NAN};         ///< (m/s)
		bool heading_ready{false};

		// position (valid if heading_ready)
		Vector2f pos_mean{NAN, NAN};    ///< mean carried-forward GNSS position in the projection frame (m)
		float pos_scatter{NAN};         ///< RMS scatter per axis (m)
		float pos_acc_rms{NAN};         ///< (m)
		bool pos_ready{false};

		// altitude
		float alt_mean{NAN};            ///< mean carried-forward GNSS altitude AMSL (m)
		float alt_scatter{NAN};         ///< (m)
		float alt_acc_rms{NAN};         ///< (m)
		bool alt_ready{false};
	};

	static constexpr int kCapacity = 128;
	static constexpr int kMinSamples = 5;

	void clear() { _count = 0; _head = 0; }

	/*
	 * Add a sample and drop samples older than window_us relative to it.
	 * Samples closer than window_us / (kCapacity - 8) to the previous one are skipped
	 * so that the buffer always covers the whole window.
	 */
	void push(const Sample &sample, uint64_t window_us);

	int size() const { return _count; }
	float spanSeconds() const;

	/*
	 * Evaluate the three filters. proj is the projection used for the GNSS positions;
	 * ekf_pos values are only used as displacements so any local frame aligned with NED works.
	 */
	Result evaluate(const MapProjection &proj, const EkfNow &ekf, const Params &params) const;

	const Sample &oldest() const { return at(0); }
	const Sample &newest() const { return at(_count - 1); }

private:
	const Sample &at(int i) const { return _buf[(_head + i) % kCapacity]; }

	Sample _buf[kCapacity] {};
	int _head{0};   ///< index of the oldest sample
	int _count{0};

	static constexpr float kVelErrCorrTime = 1.f; ///< assumed correlation time of the velocity errors (s)
};

enum class GnssRejoinStopReason : uint8_t {
	kNone          = 0,
	kChecks        = 1, ///< a sample failed the strict GNSS checks
	kNoData        = 2, ///< no GNSS data for the timeout
	kRejected      = 3, ///< velocity, position or height not fused for the timeout (innovations rejected)
	kPrerequisites = 4, ///< tilt or yaw alignment lost
	kNotIntended   = 5, ///< GNSS fusion disabled
	kModeChange    = 6  ///< EKF2_GPS_MODE changed
};

/*
 * Values that determine the rejoin state, for logging.
 */
struct GnssRejoinStatus {
	uint64_t time_us{0};                ///< time of the GNSS sample of the last update (us)

	uint8_t pilot_state{0};             ///< 0: GPS bad, 1: waiting for agreement, 2: in use
	bool shadow{false};                 ///< GNSS fusion disabled (EKF2_GPS_CTRL = 0 or sensor disabled): evaluated only
	bool shadow_would_join{false};      ///< shadow: all filters ready on the last sample, GNSS would have joined
	uint16_t shadow_join_count{0};      ///< shadow: number of times shadow_would_join became true
	bool in_use{false};
	bool preconditions_ok{false};       ///< tilt aligned, non-GNSS horizontal aiding active, vertical position valid
	bool checks_passed{false};          ///< strict checks including the health time
	bool sample_failed_checks{false};   ///< the last sample failed the strict checks
	float time_since_check_fail{NAN};   ///< (s)

	GnssRejoinWindow::Result window{};  ///< filter values of the last evaluation (while not in use)
	Vector2f pos_offset{NAN, NAN};      ///< carried-forward GNSS position - EKF position at the fusion horizon (m), NaN without origin
	float alt_offset{NAN};              ///< carried-forward GNSS altitude - EKF altitude at the fusion horizon (m)
	bool heading_required{false};
	bool pos_required{false};
	bool alt_required{false};

	float time_since_vel_fuse{NAN};     ///< (s) while in use
	float time_since_pos_fuse{NAN};     ///< (s) while in use
	float time_since_hgt_fuse{NAN};     ///< (s) while in use

	float join_heading_correction{NAN}; ///< correction applied (or found) at the last join (rad)
	GnssRejoinWindow::HeadingSource join_heading_source{GnssRejoinWindow::HeadingSource::kNone};
	Vector2f join_pos_offset{NAN, NAN}; ///< position offset found at the last join (m)
	float join_alt_offset{NAN};         ///< altitude offset found at the last join (m)

	uint16_t join_count{0};
	uint16_t stop_count{0};
	uint16_t heading_reset_count{0};
	uint16_t pos_reset_count{0};
	uint16_t alt_reset_count{0};
	uint16_t window_clear_count{0};      ///< window cleared for any reason other than a join
	GnssRejoinStopReason last_stop_reason{GnssRejoinStopReason::kNone};
};

} // namespace estimator

#endif // !EKF_GNSS_REJOIN_HPP
