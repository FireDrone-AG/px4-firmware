/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * Tests of the GNSS rejoin in strict mode (EKF2_GPS_MODE = Strict).
 *
 * The vehicle is aided by vision (body frame velocity, height and optionally FRD position with its
 * own heading) and has no magnetometer and no barometer, like a radar odometry vehicle.
 */

#include <gtest/gtest.h>
#include <deque>
#include <random>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

using estimator::GnssRejoinStopReason;

class EkfGnssRejoinTest : public ::testing::Test
{
public:
	EkfGnssRejoinTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	struct Truth {
		uint64_t time_us;
		Vector3f pos;
		Vector3f vel;
	};

	// true heading of the vehicle (the EKF starts at 0 and has no heading reference)
	float _yaw_true{math::radians(60.f)};
	// heading of the vision frame relative to north and its origin (changes when the radar restarts)
	float _ev_yaw_offset{math::radians(25.f)};
	Vector3f _ev_origin{};
	bool _ev_running{true};

	Vector3f _pos{};
	Vector3f _vel{};
	Vector3f _vel_target{};
	std::deque<Truth> _history{};

	// GNSS errors added on top of the truth
	Vector2f _gnss_pos_offset{};
	float _gnss_alt_offset{0.f};
	int _gnss_nsats{16};
	bool _gnss_running{true};

	MapProjection _ref{};
	static constexpr double kLat0 = 47.3566094;
	static constexpr double kLon0 = 8.5190237;
	static constexpr float kAlt0 = 422.056f;

	std::mt19937 _rng{42};
	std::normal_distribution<float> _noise{0.f, 1.f};

	void SetUp() override
	{
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_ref.initReference(kLat0, kLon0, 0);

		// no magnetometer, no barometer
		_sensor_simulator._mag.stop();
		_ekf_wrapper.setMagFuseTypeNone();
		_sensor_simulator.stopBaro();
		_ekf_wrapper.disableBaroHeightFusion();

		// vision: body frame velocity and height. FRD position is only enabled in one test because its
		// rotation into the EKF frame follows the EKF heading and degrades the heading while GNSS is in use.
		_ekf_wrapper.enableExternalVisionVelocityFusion();
		_ekf_wrapper.enableExternalVisionHeightFusion();
		_sensor_simulator._vio.setVelocityFrameToBody();
		_sensor_simulator._vio.setPositionFrameToLocalFRD();
		_sensor_simulator._vio.setVelocityVariance(Vector3f(0.01f, 0.01f, 0.01f));
		_sensor_simulator._vio.setPositionVariance(Vector3f(0.01f, 0.01f, 0.01f));
		_sensor_simulator._vio.setAngularVariance(1e-4f);

		// GNSS in strict mode, GNSS is the height reference
		_ekf_wrapper.setGnssStrictMode();
		_ekf_wrapper.enableGpsFusion();
		_ekf_wrapper.enableGpsHeightFusion();
		_ekf_wrapper.setGpsHeightRef();

		updateSensorData();
		_sensor_simulator.startExternalVision();
		_sensor_simulator.startGps();
	}

	const estimator::GnssRejoinStatus &status() const { return _ekf->gnss_rejoin_status(); }

	bool gnssInUse() const
	{
		return _ekf->control_status_flags().gnss_pos && _ekf->control_status_flags().gnss_vel
		       && _ekf->control_status_flags().gps_hgt;
	}

	bool gnssAnyInUse() const
	{
		return _ekf->control_status_flags().gnss_pos || _ekf->control_status_flags().gnss_vel
		       || _ekf->control_status_flags().gps_hgt;
	}

	Truth delayedTruth(float delay_s) const
	{
		const uint64_t t = _sensor_simulator.getTime() - static_cast<uint64_t>(delay_s * 1e6f);

		for (auto it = _history.rbegin(); it != _history.rend(); ++it) {
			if (it->time_us <= t) {
				return *it;
			}
		}

		return _history.empty() ? Truth{0, _pos, _vel} : _history.front();
	}

	void updateSensorData()
	{
		const Dcmf R_body_to_world{Eulerf(0.f, 0.f, _yaw_true)};
		const Dcmf R_ev{Eulerf(0.f, 0.f, -_ev_yaw_offset)};

		// vision, 175 ms delay (EKF2_EV_DELAY)
		const Truth ev = delayedTruth(0.175f);
		_sensor_simulator._vio.setVelocity(Vector3f(R_body_to_world.transpose() * ev.vel));
		_sensor_simulator._vio.setPosition(Vector3f(R_ev * (ev.pos - _ev_origin)));
		_sensor_simulator._vio.setOrientation(Quatf(Dcmf(R_ev * R_body_to_world)));

		// GNSS, 110 ms delay (simulator time stamping)
		const Truth gnss = delayedTruth(0.110f);
		const Vector2f pos_ne = Vector2f(gnss.pos.xy()) + _gnss_pos_offset
					+ Vector2f(_noise(_rng), _noise(_rng)) * 0.1f;
		double lat = 0.0;
		double lon = 0.0;
		_ref.reproject(pos_ne(0), pos_ne(1), lat, lon);
		_sensor_simulator._gps.setLatitude(lat);
		_sensor_simulator._gps.setLongitude(lon);
		_sensor_simulator._gps.setAltitude(kAlt0 - gnss.pos(2) + _gnss_alt_offset + 0.1f * _noise(_rng));
		_sensor_simulator._gps.setVelocity(gnss.vel + Vector3f(_noise(_rng), _noise(_rng), _noise(_rng)) * 0.05f);
		_sensor_simulator._gps.setNumberOfSatellites(_gnss_nsats);
	}

	// simulate the vehicle following _vel_target with limited acceleration
	void run(float duration_s)
	{
		const Dcmf R_body_to_world{Eulerf(0.f, 0.f, _yaw_true)};
		const int steps = static_cast<int>(duration_s * 1000.f);

		for (int i = 0; i < steps; i++) {
			constexpr float dt = 1e-3f;
			constexpr float acc_max = 2.f;
			const Vector3f dv = _vel_target - _vel;
			const Vector3f acc = dv.longerThan(acc_max * dt) ? Vector3f(dv.normalized() * acc_max) : Vector3f(dv / dt);

			_vel += acc * dt;
			_pos += _vel * dt;

			_history.push_back(Truth{_sensor_simulator.getTime(), _pos, _vel});

			while (_history.size() > 400) {
				_history.pop_front();
			}

			const Vector3f specific_force = R_body_to_world.transpose() * (acc + Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
			_sensor_simulator._imu.setData(specific_force, Vector3f());

			updateSensorData();

			if (_gnss_running != _sensor_simulator._gps.isRunning()) {
				_gnss_running ? _sensor_simulator.startGps() : _sensor_simulator.stopGps();
			}

			if (_ev_running != _sensor_simulator._vio.isRunning()) {
				_ev_running ? _sensor_simulator.startExternalVision() : _sensor_simulator.stopExternalVision();
			}

			_sensor_simulator.runMicroseconds(1000);
		}
	}

	// ground phase with failing GNSS checks so that the rejoin window only starts in flight
	void groundPhaseWithoutGnss()
	{
		_gnss_nsats = 3;
		run(10.f);
		_gnss_nsats = 16;
	}

	void takeOff()
	{
		_ekf->set_in_air_status(true);
		_ekf->set_vehicle_at_rest(false);
	}

	float ekfYaw() const { return matrix::Eulerf(_ekf->getQuaternion()).psi(); }

	// GNSS-derived (true) position of the vehicle in the reference frame used by the test
	Vector2f ekfPositionInRefFrame() const
	{
		const LatLonAlt lla = _ekf->getLatLonAlt();
		return _ref.project(lla.latitude_deg(), lla.longitude_deg());
	}

	// fly until GNSS joins, return false on timeout
	bool runUntilJoined(float timeout_s)
	{
		for (float t = 0.f; t < timeout_s; t += 0.1f) {
			run(0.1f);

			if (gnssInUse()) {
				return true;
			}
		}

		return false;
	}
};

TEST_F(EkfGnssRejoinTest, firstJoinWithHeadingError)
{
	// GIVEN: the EKF aligned on ground with vision aiding, no heading reference
	run(10.f);
	EXPECT_TRUE(_ekf->control_status_flags().ev_vel);
	EXPECT_FALSE(_ekf->control_status_flags().yaw_align);

	// THEN: GNSS passes the checks but cannot join while the vehicle does not move
	run(10.f);
	EXPECT_TRUE(_ekf->gps_checks_passed());
	EXPECT_FALSE(gnssAnyInUse());
	EXPECT_EQ(status().pilot_state, 1);
	EXPECT_FALSE(_ekf->global_origin().isInitialized());

	const uint8_t posD_reset_count = _ekf->state_reset_status().reset_count.posD;
	const Vector3f local_pos_before = _ekf->getPosition();

	// WHEN: flying at 2 m/s
	takeOff();
	_vel_target = Vector3f(2.f, 0.f, 0.f);

	// THEN: GNSS joins after the window is full of consistent samples
	ASSERT_TRUE(runUntilJoined(20.f));
	EXPECT_EQ(status().join_count, 1);
	EXPECT_EQ(status().pilot_state, 2);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);

	// AND: the heading was reset by the 60 deg error
	EXPECT_EQ(status().heading_reset_count, 1);
	EXPECT_EQ(status().join_heading_source, estimator::GnssRejoinWindow::HeadingSource::kFit);
	EXPECT_NEAR(math::degrees(status().join_heading_correction), 60.f, 3.f);
	EXPECT_NEAR(math::degrees(wrap_pi(ekfYaw() - _yaw_true)), 0.f, 3.f);

	// AND: the origin was set without moving the local position and without a vertical reset
	EXPECT_TRUE(_ekf->global_origin().isInitialized());
	EXPECT_TRUE(PX4_ISFINITE(_ekf->getEkfGlobalOriginAltitude()));
	EXPECT_EQ(_ekf->state_reset_status().reset_count.posD, posD_reset_count);
	EXPECT_EQ(status().pos_reset_count, 0);
	EXPECT_EQ(status().alt_reset_count, 0);
	EXPECT_NEAR(_ekf->getPosition()(2), local_pos_before(2), 0.5f);

	// AND: the global position and altitude match the truth
	run(5.f);
	EXPECT_TRUE(gnssInUse());
	const Vector2f pos_err = ekfPositionInRefFrame() - Vector2f(_pos.xy());
	EXPECT_LT(pos_err.norm(), 1.f);
	EXPECT_NEAR(_ekf->getLatLonAlt().altitude(), kAlt0 - _pos(2), 1.f);
}

TEST_F(EkfGnssRejoinTest, hoverWithoutHeadingNeverJoins)
{
	run(10.f);
	takeOff();

	// WHEN: hovering for a long time without a heading reference
	run(40.f);

	// THEN: GNSS waits for agreement and never joins
	EXPECT_TRUE(_ekf->gps_checks_passed());
	EXPECT_FALSE(gnssAnyInUse());
	EXPECT_EQ(status().pilot_state, 1);
	EXPECT_EQ(status().join_count, 0);
	EXPECT_FALSE(status().window.heading_ready);
}

TEST_F(EkfGnssRejoinTest, checkFailureDuringCollectionRestartsWindow)
{
	groundPhaseWithoutGnss();
	takeOff();
	_vel_target = Vector3f(0.f, 2.f, 0.f);
	run(8.f);
	EXPECT_FALSE(_ekf->gps_checks_passed());
	EXPECT_FALSE(gnssAnyInUse());

	// WHEN: the strict checks fail for 1 s, 8 s into the window
	_gnss_nsats = 3;
	run(1.f);
	_gnss_nsats = 16;

	// THEN: no join before the checks have passed again for the full health time
	run(9.f);
	EXPECT_FALSE(gnssAnyInUse());
	EXPECT_GE(status().window_clear_count, 1);

	// AND: GNSS joins afterwards
	EXPECT_TRUE(runUntilJoined(5.f));
}

TEST_F(EkfGnssRejoinTest, checkFailureInUseStopsImmediatelyAndRejoins)
{
	run(10.f);
	takeOff();
	_vel_target = Vector3f(2.f, 0.f, 0.f);
	ASSERT_TRUE(runUntilJoined(20.f));

	// WHEN: a single sample fails the strict checks
	_gnss_nsats = 3;
	run(0.2f);
	_gnss_nsats = 16;
	run(0.4f); // fusion horizon delay

	// THEN: all GNSS fusion is stopped at once
	EXPECT_FALSE(gnssAnyInUse());
	EXPECT_EQ(status().last_stop_reason, GnssRejoinStopReason::kChecks);
	EXPECT_EQ(status().stop_count, 1);
	EXPECT_EQ(status().pilot_state, 0);

	// AND: it rejoins only after the full health time, without a heading reset (heading is good)
	run(9.f);
	EXPECT_FALSE(gnssAnyInUse());
	ASSERT_TRUE(runUntilJoined(5.f));
	EXPECT_EQ(status().join_count, 2);
	EXPECT_EQ(status().heading_reset_count, 1);
	EXPECT_LT(status().join_pos_offset.norm(), 0.5f); // only the vision drift during the outage
}

TEST_F(EkfGnssRejoinTest, offsetAfterOutageResetsPosition)
{
	run(10.f);
	takeOff();
	_vel_target = Vector3f(2.f, 1.f, 0.f);
	ASSERT_TRUE(runUntilJoined(20.f));
	run(2.f);

	// WHEN: GNSS data stops
	_gnss_running = false;
	run(6.f);

	// THEN: GNSS is stopped after the timeout
	EXPECT_FALSE(gnssAnyInUse());
	EXPECT_EQ(status().last_stop_reason, GnssRejoinStopReason::kNoData);

	// WHEN: GNSS comes back 3 m east and 1 m higher of where the vision aided EKF is
	_gnss_pos_offset = Vector2f(0.f, 3.f);
	_gnss_alt_offset = 1.f;
	_gnss_running = true;

	// THEN: it rejoins and resets position and altitude once
	ASSERT_TRUE(runUntilJoined(20.f));
	EXPECT_EQ(status().join_count, 2);
	EXPECT_EQ(status().pos_reset_count, 1);
	EXPECT_EQ(status().alt_reset_count, 1);
	EXPECT_NEAR(status().join_pos_offset(1), 3.f, 0.5f);
	EXPECT_NEAR(status().join_alt_offset, 1.f, 0.4f);
}

TEST_F(EkfGnssRejoinTest, jumpingGnssDoesNotJoin)
{
	groundPhaseWithoutGnss();
	takeOff();
	_vel_target = Vector3f(2.f, 0.f, 0.f);

	// WHEN: the GNSS position jumps by 4 m every 2 s (multipath-like) while reporting 0.5 m accuracy
	for (int i = 0; i < 15; i++) {
		_gnss_pos_offset = (i % 2) ? Vector2f(4.f, 0.f) : Vector2f(0.f, 0.f);
		run(2.f);
		EXPECT_FALSE(_ekf->control_status_flags().gnss_pos);
	}

	EXPECT_TRUE(_ekf->gps_checks_passed());
	EXPECT_FALSE(status().window.pos_ready);

	// THEN: once clean, it joins after the inconsistent samples aged out
	_gnss_pos_offset.setZero();
	EXPECT_TRUE(runUntilJoined(15.f));
}

TEST_F(EkfGnssRejoinTest, rejectedInUseStopsAndRejoinsAtNewPosition)
{
	run(10.f);
	takeOff();
	_vel_target = Vector3f(2.f, 0.f, 0.f);
	ASSERT_TRUE(runUntilJoined(20.f));
	run(2.f);

	// WHEN: the GNSS position steps by 20 m (passes the checks, fails the innovation gate)
	_gnss_pos_offset = Vector2f(20.f, 0.f);
	run(5.5f);

	// THEN: GNSS is stopped after EKF2_GPS_RJ_TOUT without being fused
	EXPECT_FALSE(gnssAnyInUse());
	EXPECT_EQ(status().last_stop_reason, GnssRejoinStopReason::kRejected);
	EXPECT_FALSE(_ekf->control_status_flags().gnss_fault);

	// AND: it rejoins with a reset to the new (consistent) position
	ASSERT_TRUE(runUntilJoined(20.f));
	EXPECT_EQ(status().pos_reset_count, 1);
	EXPECT_NEAR(status().join_pos_offset(0), 20.f, 1.f);
}

TEST_F(EkfGnssRejoinTest, hoverRejoinWithGoodEkfHeading)
{
	run(10.f);
	takeOff();
	_vel_target = Vector3f(2.f, 0.f, 0.f);
	ASSERT_TRUE(runUntilJoined(20.f));
	run(5.f);

	// WHEN: GNSS is lost briefly and the vehicle hovers
	_vel_target.setZero();
	_gnss_running = false;
	run(6.f);
	EXPECT_FALSE(gnssAnyInUse());
	_gnss_running = true;

	// THEN: it rejoins with the EKF heading, which is still good, without a heading reset
	ASSERT_TRUE(runUntilJoined(15.f));
	EXPECT_EQ(status().heading_reset_count, 1);
	EXPECT_EQ(status().join_heading_source, estimator::GnssRejoinWindow::HeadingSource::kEkf);
}


TEST_F(EkfGnssRejoinTest, firstJoinWithEvPosition)
{
	// GIVEN: vision FRD position fused as well
	_ekf_wrapper.enableExternalVisionPositionFusion();
	run(10.f);
	EXPECT_TRUE(_ekf->control_status_flags().ev_pos);

	takeOff();
	_vel_target = Vector3f(2.f, 0.f, 0.f);
	const Vector3f local_pos_before = _ekf->getPosition();

	// THEN: GNSS joins with a heading reset and the local position does not jump
	ASSERT_TRUE(runUntilJoined(20.f));
	EXPECT_EQ(status().heading_reset_count, 1);
	EXPECT_NEAR(math::degrees(status().join_heading_correction), 60.f, 3.f);
	EXPECT_EQ(status().pos_reset_count, 0);
	EXPECT_LT(Vector2f(_ekf->getPosition().xy() - local_pos_before.xy()).norm(), 5.f);

	// AND: the vision position keeps being fused (its bias absorbed the frame change)
	run(3.f);
	EXPECT_TRUE(gnssInUse());
	EXPECT_TRUE(_ekf->control_status_flags().ev_pos);
	EXPECT_LT(Vector2f(ekfPositionInRefFrame() - Vector2f(_pos.xy())).norm(), 2.f);
}

TEST_F(EkfGnssRejoinTest, shadowWithGnssDisabled)
{
	// GIVEN: GNSS fusion disabled (EKF2_GPS_CTRL = 0), strict mode
	_ekf_wrapper.disableGpsFusion();
	_ekf_wrapper.disableGpsHeightFusion();
	run(10.f);
	takeOff();
	_vel_target = Vector3f(2.f, 0.f, 0.f);

	// THEN: the rejoin is evaluated and would join, but GNSS is never fused
	run(20.f);
	EXPECT_FALSE(gnssAnyInUse());
	EXPECT_FALSE(_ekf->global_origin().isInitialized());
	EXPECT_TRUE(status().shadow);
	EXPECT_TRUE(status().shadow_would_join);
	EXPECT_GE(status().shadow_join_count, 1);
	EXPECT_EQ(status().pilot_state, 0);
	EXPECT_TRUE(status().window.heading_ready);
	EXPECT_NEAR(math::degrees(status().window.heading_fit), 60.f, 3.f);

	// WHEN: GNSS fusion is enabled in flight
	_ekf_wrapper.enableGpsFusion();
	_ekf_wrapper.enableGpsHeightFusion();

	// THEN: it joins on the next samples, the shadow window is already full
	EXPECT_TRUE(runUntilJoined(1.f));
	EXPECT_FALSE(status().shadow);
}

TEST_F(EkfGnssRejoinTest, radarRestartAfterFailstateStartsInRadarFrame)
{
	// GIVEN: radar position fused and GNSS in use (learned GNSS-radar offsets)
	_ekf_wrapper.enableExternalVisionPositionFusion();
	run(10.f);
	takeOff();
	_vel_target = Vector3f(2.f, 0.f, 0.f);
	ASSERT_TRUE(runUntilJoined(20.f));
	run(5.f);
	ASSERT_TRUE(PX4_ISFINITE(_ekf->getEkfGlobalOriginAltitude()));

	// WHEN: radar and GNSS are both lost (failstate, GNSS stopped after EKF2_GPS_RJ_TOUT)
	_ev_running = false;
	_gnss_running = false;
	_vel_target.setZero();
	run(6.f);
	EXPECT_FALSE(gnssAnyInUse());
	EXPECT_FALSE(_ekf->control_status_flags().ev_pos);
	EXPECT_FALSE(_ekf->control_status_flags().ev_hgt);

	// AND: the radar comes back with a new frame: origin at the current position, other heading
	_ev_origin = _pos;
	_ev_yaw_offset = math::radians(-40.f);
	_sensor_simulator._vio.setResetCounter(1);
	const uint8_t posNE_resets = _ekf->state_reset_status().reset_count.posNE;
	_ev_running = true;
	run(2.f);

	// THEN: the EKF restarts in the radar frame: offsets discarded, local position and height = radar
	EXPECT_TRUE(_ekf->control_status_flags().ev_pos);
	EXPECT_TRUE(_ekf->control_status_flags().ev_hgt);
	EXPECT_NE(_ekf->state_reset_status().reset_count.posNE, posNE_resets);
	EXPECT_LT(Vector2f(_ekf->getPosition().xy()).norm(), 0.5f);
	EXPECT_NEAR(_ekf->getPosition()(2), 0.f, 0.3f);
	EXPECT_FALSE(gnssAnyInUse());

	// AND: like a boot without GNSS and magnetometer: no global origin, no heading alignment (O31)
	EXPECT_FALSE(_ekf->global_origin().isInitialized());
	EXPECT_FALSE(_ekf->control_status_flags().yaw_align);
	EXPECT_FALSE(_ekf->isGlobalHorizontalPositionValid());
	EXPECT_FALSE(_ekf->isGlobalVerticalPositionValid());

	// WHEN: GNSS is back and the vehicle flies again
	_gnss_running = true;
	_vel_target = Vector3f(0.f, 2.f, 0.f);

	// THEN: GNSS rejoins with a heading reset and a new origin, and the global position is right again
	ASSERT_TRUE(runUntilJoined(25.f));
	EXPECT_EQ(status().heading_reset_count, 2);
	EXPECT_TRUE(_ekf->global_origin().isInitialized());
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);
	run(3.f);
	EXPECT_LT(Vector2f(ekfPositionInRefFrame() - Vector2f(_pos.xy())).norm(), 2.f);
	EXPECT_NEAR(_ekf->getLatLonAlt().altitude(), kAlt0 - _pos(2), 1.f);
}

TEST_F(EkfGnssRejoinTest, radarResetAsFallbackHeightReferenceKeepsLocalZ)
{
	// GIVEN: GNSS joined (altitude origin set), then lost: radar is the fallback height reference
	run(10.f);
	takeOff();
	_vel_target = Vector3f(2.f, 0.f, -0.5f);
	ASSERT_TRUE(runUntilJoined(20.f));
	run(4.f);
	_vel_target = Vector3f(2.f, 0.f, 0.f);
	_gnss_running = false;
	run(7.f);
	ASSERT_FALSE(gnssAnyInUse());
	ASSERT_TRUE(PX4_ISFINITE(_ekf->getEkfGlobalOriginAltitude()));
	const float z_before = _ekf->getPosition()(2);

	// WHEN: the radar resets its frame (reset counter, height restarts at 0)
	_ev_origin = _pos;
	_sensor_simulator._vio.setResetCounter(1);
	run(1.f);

	// THEN: local z follows the radar height (0), and does not jump by the origin altitude
	EXPECT_NEAR(_ekf->getPosition()(2), 0.f, 0.5f);
	EXPECT_LT(fabsf(_ekf->getPosition()(2) - z_before), 10.f);
}

class EkfGnssRejoinHeightBiasTest : public EkfGnssRejoinTest
{
public:
	// GNSS height wanders by 2 m over 60 s while the radar height is exact.
	// Returns how much of the wander reached the altitude estimate (m).
	float runGnssHeightWander()
	{
		run(10.f);
		takeOff();
		_vel_target = Vector3f(2.f, 0.f, 0.f);
		EXPECT_TRUE(runUntilJoined(20.f));
		_vel_target.setZero();
		run(20.f);

		for (int i = 0; i < 120; i++) {
			_gnss_alt_offset = 2.f * (i + 1) / 120.f;
			run(0.5f);
		}

		EXPECT_TRUE(gnssInUse());
		// estimated height above the truth (local z is down)
		return _pos(2) - _ekf->getPosition()(2);
	}
};

TEST_F(EkfGnssRejoinHeightBiasTest, gnssHeightWanderStock)
{
	const float leak = runGnssHeightWander();
	printf("GNSS height wander reaching the altitude, stock (0.13, boost on): %.2f m of 2.00 m\n", (double)leak);
	EXPECT_GT(leak, 0.5f);
}

TEST_F(EkfGnssRejoinHeightBiasTest, gnssHeightWanderLowBiasNoiseNoBoost)
{
	estimator::parameters *params = _ekf->getParamHandle();
	params->ev_hgt_bias_nsd = 0.02f;
	params->ev_bias_boost = 0;

	const float leak = runGnssHeightWander();
	printf("GNSS height wander reaching the altitude, 0.02 without boost: %.2f m of 2.00 m\n", (double)leak);
	EXPECT_LT(leak, 0.5f);
}

TEST_F(EkfGnssRejoinHeightBiasTest, gnssHeightWanderLowBiasNoiseLowFloor)
{
	estimator::parameters *params = _ekf->getParamHandle();
	params->ev_hgt_bias_nsd = 0.02f;
	params->ev_bias_boost = 0;
	params->ekf2_gps_p_noise = 0.1f; // radar height noise floor while GNSS height is fused

	const float leak = runGnssHeightWander();
	printf("GNSS height wander reaching the altitude, 0.02 without boost, floor 0.1 m: %.2f m of 2.00 m\n", (double)leak);
	EXPECT_LT(leak, 0.2f);
}

TEST_F(EkfGnssRejoinHeightBiasTest, gnssHeightWanderStockLowFloor)
{
	estimator::parameters *params = _ekf->getParamHandle();
	params->ekf2_gps_p_noise = 0.1f;

	const float leak = runGnssHeightWander();
	printf("GNSS height wander reaching the altitude, stock, floor 0.1 m: %.2f m of 2.00 m\n", (double)leak);
	EXPECT_LT(leak, 0.3f);
}

TEST_F(EkfGnssRejoinTest, headingWithEvPosition)
{
	// GIVEN: radar FRD position fused together with GNSS (O32)
	_ekf_wrapper.enableExternalVisionPositionFusion();
	run(10.f);
	takeOff();
	_vel_target = Vector3f(2.f, 1.f, 0.f);
	ASSERT_TRUE(runUntilJoined(20.f));

	// THEN: GNSS keeps the heading right while in use
	float max_err = 0.f;

	for (int i = 0; i < 30; i++) {
		run(1.f);
		max_err = math::max(max_err, fabsf(math::degrees(wrap_pi(ekfYaw() - _yaw_true))));
	}

	EXPECT_LT(max_err, 1.f);

	// WHEN: GNSS is lost for a minute
	_gnss_running = false;
	run(60.f);

	// THEN: the heading does not drift away and its uncertainty covers the error
	const float err = fabsf(math::degrees(wrap_pi(ekfYaw() - _yaw_true)));
	EXPECT_LT(err, 1.f);
	EXPECT_GT(math::degrees(sqrtf(_ekf->getYawVar())), err);
	EXPECT_LT(Vector2f(ekfPositionInRefFrame() - Vector2f(_pos.xy())).norm(), 2.f);
}

