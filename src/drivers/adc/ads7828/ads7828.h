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

#pragma once

#include <math.h>
#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>

using namespace time_literals;

class ADS7828 : public device::I2C, public I2CSPIDriver<ADS7828>
{
public:
	ADS7828(const I2CSPIDriverConfig &config);
	~ADS7828() override;

	static void print_usage();

	int init() override;
	int probe() override;
	void RunImpl();
	void print_status() override;

private:
	static constexpr uint8_t kAddressMin = 0x48;
	static constexpr uint8_t kAddressMax = 0x4B;
	static constexpr uint8_t kTh1Channel = 0;
	static constexpr uint8_t kTh2Channel = 1;
	static constexpr uint8_t kPublishedChannelCount = 2;
	static constexpr uint8_t kSamplesPerReading = 4;
	static constexpr uint16_t kAdsMaxCode = 4095;
	static constexpr uint16_t kAdsClipThreshold = 4090;
	static constexpr float kAdsReferenceVolts = 2.5f;
	static constexpr float kDividerSupplyVolts = 3.3f;
	static constexpr float kPullupResistorOhms = 10000.0f;
	static constexpr float kThermistorNominalOhms = 10000.0f;
	static constexpr float kThermistorNominalTempC = 25.0f;
	static constexpr float kThermistorBeta = 3950.0f;
	static const hrt_abstime kSampleInterval;
	static const uint8_t kAdsSingleEndedCommand[8];

	struct ThermistorReading {
		bool sample_ok{false};
		bool conversion_ok{false};
		bool saturated{false};
		uint16_t raw{0};
		float voltage{0.0f};
		float resistance_ohms{0.0f};
		float temp_c{NAN};
	};

	uORB::Publication<adc_report_s> _adc_report_pub{ORB_ID(adc_report)};

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;

	adc_report_s _adc_report{};
	ThermistorReading _last_th1{};
	ThermistorReading _last_th2{};
	bool _has_published{false};
	uint32_t _error_count{0};

	bool probe_address(uint8_t address);
	void clear_adc_report_channels();
	void prime_reference();
	int read_raw_once(uint8_t channel, uint16_t &raw_out);
	int read_raw_filtered(uint8_t channel, uint16_t &raw_out);
	ThermistorReading read_thermistor(uint8_t channel);
	void publish_adc_report(const ThermistorReading &th1, const ThermistorReading &th2, hrt_abstime timestamp);
};
