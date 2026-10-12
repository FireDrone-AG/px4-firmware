/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include <px4_log.h>
#include <cmath>
#include "PCA9685.h"

#include <px4_platform_common/sem.hpp>

using namespace drv_pca9685_pwm;

PCA9685::PCA9685(int bus, int addr):
	I2C(DRV_PWM_DEVTYPE_PCA9685, MODULE_NAME, bus, addr, 100000)
{

}

int PCA9685::Stop()
{
	disableAllOutput();
	stopOscillator();
	return PX4_OK;
}

int PCA9685::updatePWM(const uint16_t *outputs, unsigned num_outputs)
{
	if (num_outputs > PCA9685_PWM_CHANNEL_COUNT) {
		num_outputs = PCA9685_PWM_CHANNEL_COUNT;
		PX4_DEBUG("PCA9685 can only drive up to 16 channels");
	}

	uint16_t out[PCA9685_PWM_CHANNEL_COUNT];
	memcpy(out, outputs, sizeof(uint16_t) * num_outputs);

	for (unsigned i = 0; i < num_outputs; ++i) {
		out[i] = static_cast<uint16_t>(roundf((out[i] * _Freq * PCA9685_PWM_RES) / 1e6f)); // convert us to 12 bit resolution
	}

	return setPWM(num_outputs, out);
}

int PCA9685::setPWMus(uint8_t channel, uint16_t pulse_width_us)
{
	if (channel >= PCA9685_PWM_CHANNEL_COUNT) {
		return -EINVAL;
	}

	const uint16_t output = static_cast<uint16_t>(roundf((pulse_width_us * _Freq * PCA9685_PWM_RES) / 1e6f));
	return setPWM(channel, output);
}

int PCA9685::setFreq(float freq)
{
	uint16_t realResolution = static_cast<uint16_t>(floorf((float)PCA9685_CLOCK_FREQ / freq));

	if (realResolution < PCA9685_PWM_RES) { // unable to provide enough resolution
		PX4_DEBUG("frequency too high");
		return -EINVAL;
	}

	uint16_t divider = static_cast<uint16_t>(roundf((float)PCA9685_CLOCK_FREQ / freq / PCA9685_PWM_RES)) - 1;

	if (divider > 0x00FF) { // out of divider
		PX4_DEBUG("frequency too low");
		return -EINVAL;
	}

	float freq_err = ((float)PCA9685_CLOCK_FREQ / (float)(divider + (uint16_t)1)
			  - (float)(freq * PCA9685_PWM_RES))
			 / (float)(freq * PCA9685_PWM_RES); // actually asked for (freq * PCA9685_PWM_RES)

	if (fabsf(freq_err) > 0.01f) { // TODO decide threshold
		PX4_WARN("frequency error too large: %.4f", (double)freq_err);
		// should we return an error?
	}

	_Freq = (float)PCA9685_CLOCK_FREQ / (float)(divider + (uint16_t)1) / PCA9685_PWM_RES; // use actual pwm freq instead.

	setDivider(divider);

	return PX4_OK;

}

int PCA9685::initReg()
{
	uint8_t buf[2] = {};

	buf[0] = PCA9685_REG_MODE1;
	buf[1] = DEFAULT_MODE1_CFG;
	int ret = transfer(buf, 2, nullptr, 0); // make sure oscillator is disabled

	if (OK != ret) {
		PX4_ERR("init: i2c::transfer returned %d", ret);
		return ret;
	}

	ret = transfer(buf, 2, nullptr, 0); // enable EXTCLK if possible

	if (OK != ret) {
		PX4_ERR("init: i2c::transfer returned %d", ret);
		return ret;
	}

	buf[0] = PCA9685_REG_MODE2;
	buf[1] = DEFAULT_MODE2_CFG;
	ret = transfer(buf, 2, nullptr, 0);

	if (OK != ret) {
		PX4_ERR("init: i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

int PCA9685::probe()
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
	PX4_DEBUG("PCA9685 not found on 0x%02X..0x%02X", kAddressMin, kAddressMax);
	return PX4_ERROR;
}

bool PCA9685::probe_address(uint8_t address)
{
	set_device_address(address);
	uint8_t mode1 = 0;
	const int ret = readRegister(PCA9685_REG_MODE1, mode1);

	if (ret != PX4_OK) {
		return false;
	}

	PX4_DEBUG("PCA9685 found at 0x%02X MODE1=0x%02X", address, mode1);
	return true;
}

int PCA9685::readRegister(uint8_t reg, uint8_t &value)
{
	return transfer(&reg, 1, &value, 1);
}

int PCA9685::recordTransferResult(const char *context, int ret)
{
	if (ret != PX4_OK) {
		if (_last_transfer_result == PX4_OK || _last_transfer_result != ret) {
			PX4_ERR("%s: i2c::transfer returned %d", context, ret);
		}

	} else if (_last_transfer_result != PX4_OK) {
		PX4_INFO("%s: i2c transfer recovered", context);
	}

	_last_transfer_result = ret;
	return ret;
}

int PCA9685::setPWM(uint8_t channel, const uint16_t &value)
{
	if (value >= 4096) {
		PX4_ERR("invalid pwm value");
		return -EINVAL;
	}

	uint8_t buf[5] = {};
	buf[0] = PCA9685_REG_LED0 + channel * PCA9685_REG_LED_INCREMENT;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = (uint8_t)(value & (uint8_t)0xFF);
	buf[4] = value != 0 ? ((uint8_t)(value >> (uint8_t)8)) : PCA9685_LED_ON_FULL_ON_OFF_MASK;

	return recordTransferResult("setPWM", transfer(buf, 5, nullptr, 0));
}

int PCA9685::setPWM(uint8_t channel_count, const uint16_t *value)
{
	uint8_t buf[PCA9685_PWM_CHANNEL_COUNT * PCA9685_REG_LED_INCREMENT + 1] = {};
	buf[0] = PCA9685_REG_LED0;

	for (int i = 0; i < channel_count; ++i) {
		if (value[i] >= 4096) {
			PX4_ERR("invalid pwm value");
			return -EINVAL;
		}

		buf[1 + i * PCA9685_REG_LED_INCREMENT] = 0x00;
		buf[2 + i * PCA9685_REG_LED_INCREMENT] = 0x00;
		buf[3 + i * PCA9685_REG_LED_INCREMENT] = (uint8_t)(value[i] & (uint8_t)0xFF);
		buf[4 + i * PCA9685_REG_LED_INCREMENT] = value[i] != 0 ? ((uint8_t)(value[i] >> (uint8_t)8)) :
				PCA9685_LED_ON_FULL_ON_OFF_MASK;
	}

	return recordTransferResult("setPWM", transfer(buf, channel_count * PCA9685_REG_LED_INCREMENT + 1, nullptr, 0));
}

void PCA9685::disableAllOutput()
{
	uint8_t buf[5] = {};
	buf[0] = PCA9685_REG_ALLLED_ON_L;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = PCA9685_LED_ON_FULL_ON_OFF_MASK;

	(void)recordTransferResult("disableAllOutput", transfer(buf, 5, nullptr, 0));
}

void PCA9685::setDivider(uint8_t value)
{
	uint8_t buf[2] = {};
	buf[0] = PCA9685_REG_PRE_SCALE;
	buf[1] = value;
	(void)recordTransferResult("setDivider", transfer(buf, 2, nullptr, 0));
}

void PCA9685::stopOscillator()
{
	uint8_t buf[2] = {PCA9685_REG_MODE1};

	// set to sleep
	buf[1] = DEFAULT_MODE1_CFG;
	(void)recordTransferResult("stopOscillator", transfer(buf, 2, nullptr, 0));
}

void PCA9685::startOscillator()
{
	uint8_t buf[2] = {PCA9685_REG_MODE1};

	// clear sleep bit, with restart bit = 0
	buf[1] = DEFAULT_MODE1_CFG & (~PCA9685_MODE1_SLEEP_MASK);
	(void)recordTransferResult("startOscillator", transfer(buf, 2, nullptr, 0));
}

void PCA9685::triggerRestart()
{
	uint8_t buf[2] = {PCA9685_REG_MODE1};

	// clear sleep bit, with restart bit = 0
	buf[1] = DEFAULT_MODE1_CFG & (~PCA9685_MODE1_SLEEP_MASK);
	buf[1] |= PCA9685_MODE1_RESTART_MASK;
	(void)recordTransferResult("triggerRestart", transfer(buf, 2, nullptr, 0));
}
