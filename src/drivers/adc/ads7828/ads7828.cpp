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

#include "ads7828.h"

#include <cmath>

#include <drivers/drv_adc.h>
#include <px4_platform_common/time.h>

const hrt_abstime ADS7828::kSampleInterval{500_ms};

const uint8_t ADS7828::kAdsSingleEndedCommand[8] = {
	0x8C, 0xCC, 0x9C, 0xDC, 0xAC, 0xEC, 0xBC, 0xFC
};

ADS7828::ADS7828(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	static_assert(arraySize(adc_report_s::channel_id) >= kAdsChannelCount,
		      "ADS7828 requires adc_report to publish all 8 channels");
	configure_channels_from_mask(static_cast<uint8_t>(config.custom1));
}

ADS7828::~ADS7828()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

int ADS7828::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	_adc_report.device_id = get_device_id();
	_adc_report.v_ref = kAdsReferenceVolts;
	_adc_report.resolution = kAdsMaxCode + 1;
	clear_adc_report_channels();
	prime_reference();

	ScheduleOnInterval(kSampleInterval, kSampleInterval);
	return PX4_OK;
}

int ADS7828::probe()
{
	const uint8_t starting_address = get_device_address();

	if (probe_address(starting_address)) {
		return PX4_OK;
	}

	for (uint8_t address = kAddressMin; address <= kAddressMax; ++address) {
		if (address == starting_address) {
			continue;
		}

		if (probe_address(address)) {
			return PX4_OK;
		}
	}

	set_device_address(starting_address);
	PX4_DEBUG("ADS7828 not found on 0x%02X..0x%02X", kAddressMin, kAddressMax);
	return PX4_ERROR;
}

bool ADS7828::probe_address(uint8_t address)
{
	set_device_address(address);

	const uint8_t probe_channel = (_configured_channel_count > 0) ? _configured_channels[0] : 0;
	const uint8_t command = kAdsSingleEndedCommand[probe_channel];
	uint8_t response[2] {};
	const int ret = transfer(&command, 1, response, sizeof(response));

	if (ret != PX4_OK) {
		return false;
	}

	const uint16_t raw = static_cast<uint16_t>(((response[0] & 0x0F) << 8) | response[1]);
	return raw <= kAdsMaxCode;
}

void ADS7828::configure_channels_from_mask(uint8_t channel_mask)
{
	_channel_mask = (channel_mask != 0) ? channel_mask : kDefaultChannelMask;
	_configured_channel_count = 0;

	for (uint8_t channel = 0; channel < kAdsChannelCount; ++channel) {
		if ((_channel_mask & (1u << channel)) != 0) {
			_configured_channels[_configured_channel_count++] = channel;
		}
	}
}

void ADS7828::clear_adc_report_channels()
{
	for (unsigned i = 0; i < arraySize(_adc_report.channel_id); ++i) {
		_adc_report.channel_id[i] = -1;
		_adc_report.raw_data[i] = 0;
	}
}

void ADS7828::prime_reference()
{
	if (_configured_channel_count == 0) {
		return;
	}

	uint16_t throwaway = 0;
	(void)read_raw_once(_configured_channels[0], throwaway);
	px4_usleep(5000);
	(void)read_raw_once(_configured_channels[0], throwaway);
}

int ADS7828::read_raw_once(uint8_t channel, uint16_t &raw_out)
{
	if (channel >= kAdsChannelCount) {
		return PX4_ERROR;
	}

	const uint8_t command = kAdsSingleEndedCommand[channel];
	uint8_t response[2] {};
	const int ret = transfer(&command, 1, response, sizeof(response));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		++_error_count;
		return ret;
	}

	raw_out = static_cast<uint16_t>(((response[0] & 0x0F) << 8) | response[1]);
	return PX4_OK;
}

int ADS7828::read_raw_filtered(uint8_t channel, uint16_t &raw_out)
{
	uint16_t throwaway = 0;

	if (read_raw_once(channel, throwaway) != PX4_OK) {
		return PX4_ERROR;
	}

	uint32_t sum = 0;

	for (uint8_t i = 0; i < kSamplesPerReading; ++i) {
		uint16_t sample = 0;

		if (read_raw_once(channel, sample) != PX4_OK) {
			return PX4_ERROR;
		}

		sum += sample;
		px4_usleep(60);
	}

	raw_out = static_cast<uint16_t>((sum + (kSamplesPerReading / 2)) / kSamplesPerReading);
	return PX4_OK;
}

ADS7828::ThermistorReading ADS7828::read_thermistor(uint8_t channel)
{
	ThermistorReading reading{};

	if (read_raw_filtered(channel, reading.raw) != PX4_OK) {
		return reading;
	}

	reading.sample_ok = true;

	reading.voltage = (static_cast<float>(reading.raw) * kAdsReferenceVolts) / static_cast<float>(kAdsMaxCode);

	if (reading.raw >= kAdsClipThreshold) {
		reading.saturated = true;
		return reading;
	}

	if (reading.voltage <= 0.001f || reading.voltage >= (kDividerSupplyVolts - 0.001f)) {
		return reading;
	}

	reading.resistance_ohms =
		(reading.voltage * kPullupResistorOhms) / (kDividerSupplyVolts - reading.voltage);

	if (!std::isfinite(reading.resistance_ohms) || reading.resistance_ohms <= 0.0f) {
		return reading;
	}

	const float nominal_temp_k = kThermistorNominalTempC + 273.15f;
	const float inverse_temp_k =
		(1.0f / nominal_temp_k) +
		(std::log(reading.resistance_ohms / kThermistorNominalOhms) / kThermistorBeta);

	if (!std::isfinite(inverse_temp_k) || inverse_temp_k <= 0.0f) {
		return reading;
	}

	reading.temp_c = (1.0f / inverse_temp_k) - 273.15f;
	reading.conversion_ok = std::isfinite(reading.temp_c);
	return reading;
}

void ADS7828::publish_adc_report(const ThermistorReading readings[kAdsChannelCount], hrt_abstime timestamp)
{
	clear_adc_report_channels();

	_adc_report.timestamp = timestamp;
	_adc_report.device_id = get_device_id();
	_adc_report.v_ref = kAdsReferenceVolts;
	_adc_report.resolution = kAdsMaxCode + 1;

	for (uint8_t i = 0; i < _configured_channel_count; ++i) {
		if (readings[i].sample_ok) {
			_adc_report.channel_id[i] = _configured_channels[i];
			_adc_report.raw_data[i] = readings[i].raw;
		}
	}

	_adc_report_pub.publish(_adc_report);
}

void ADS7828::RunImpl()
{
	if (should_exit()) {
		return;
	}

	perf_begin(_cycle_perf);

	ThermistorReading readings[kAdsChannelCount] {};

	for (uint8_t i = 0; i < _configured_channel_count; ++i) {
		readings[i] = read_thermistor(_configured_channels[i]);
		_last_readings[i] = readings[i];
	}

	const hrt_abstime now = hrt_absolute_time();

	publish_adc_report(readings, now);
	_has_published = true;

	perf_end(_cycle_perf);
}

void ADS7828::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("errors=%lu", static_cast<unsigned long>(_error_count));
	PX4_INFO("configured channels=%u mask=0x%02X",
		 static_cast<unsigned>(_configured_channel_count),
		 static_cast<unsigned>(_channel_mask));

	if (_has_published) {
		for (uint8_t i = 0; i < _configured_channel_count; ++i) {
			const ThermistorReading &reading = _last_readings[i];
			PX4_INFO("TH%u channel=%u valid=%d raw=%u temp=%.2f C",
				 static_cast<unsigned>(i + 1),
				 static_cast<unsigned>(_configured_channels[i]),
				 reading.conversion_ok,
				 reading.sample_ok ? reading.raw : 0,
				 static_cast<double>(reading.conversion_ok ? reading.temp_c : NAN));
		}

	} else {
		PX4_INFO("No thermistor samples published yet");
	}
}
