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
 * GNSS rejoin state for the pilot (EKF2_GPS_MODE = Strict), sent as NAMED_VALUE_INT
 * with name "GPS_STATE" and value estimator_gnss_rejoin.pilot_state:
 *   0: GPS bad (no data, failing checks, health time running or rejoin not possible)
 *   1: GPS passing checks, waiting for the filters to agree
 *   2: GPS in use
 * Sent at 1 Hz and immediately when the value changes. Not sent if EKF2 is not in strict mode.
 */

#ifndef GNSS_REJOIN_STATE_HPP
#define GNSS_REJOIN_STATE_HPP

#include <uORB/topics/estimator_gnss_rejoin.h>
#include <uORB/topics/estimator_selector_status.h>

class MavlinkStreamGnssRejoinState : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGnssRejoinState(mavlink); }

	static constexpr const char *get_name_static() { return "GNSS_REJOIN_STATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NAMED_VALUE_INT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _estimator_gnss_rejoin_sub.advertised() ? MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamGnssRejoinState(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_gnss_rejoin_sub{ORB_ID(estimator_gnss_rejoin)};

	int32_t _last_value{-1};
	hrt_abstime _last_send{0};

	bool send() override
	{
		// follow the primary estimator
		if (_estimator_selector_status_sub.updated()) {
			estimator_selector_status_s estimator_selector_status;

			if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
				if (estimator_selector_status.primary_instance != _estimator_gnss_rejoin_sub.get_instance()) {
					_estimator_gnss_rejoin_sub.ChangeInstance(estimator_selector_status.primary_instance);
				}
			}
		}

		estimator_gnss_rejoin_s rejoin;

		if (!_estimator_gnss_rejoin_sub.copy(&rejoin) || (rejoin.timestamp == 0)) {
			return false;
		}

		const int32_t value = rejoin.pilot_state;

		// 1 Hz, and immediately on change (the stream itself is configured faster)
		if ((value == _last_value) && (hrt_elapsed_time(&_last_send) < 1_s)) {
			return false;
		}

		mavlink_named_value_int_t msg{};
		msg.time_boot_ms = hrt_absolute_time() / 1000;
		strncpy(msg.name, "GPS_STATE", sizeof(msg.name));
		msg.value = value;

		mavlink_msg_named_value_int_send_struct(_mavlink->get_channel(), &msg);

		_last_value = value;
		_last_send = hrt_absolute_time();
		return true;
	}
};

#endif // GNSS_REJOIN_STATE_HPP
