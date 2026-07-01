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

#include <cstdlib>
#include <cstring>

#include <drivers/drv_adc.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace
{
constexpr uint8_t ADS7828_CHANNEL_COUNT = 8;
constexpr uint8_t ADS7828_DEFAULT_CHANNEL_MASK = (1u << 0) | (1u << 1);

uint8_t channel_mask_from_count(int channel_count)
{
	if (channel_count <= 0) {
		return 0;
	}

	if (channel_count >= ADS7828_CHANNEL_COUNT) {
		return 0xFF;
	}

	return static_cast<uint8_t>((1u << channel_count) - 1u);
}

bool parse_channel_list(const char *arg, uint8_t &channel_mask)
{
	if (arg == nullptr || *arg == '\0') {
		return false;
	}

	uint8_t parsed_mask = 0;
	const char *cursor = arg;

	while (*cursor != '\0') {
		char *endptr = nullptr;
		const long channel = strtol(cursor, &endptr, 10);

		if (endptr == cursor || channel < 0 || channel >= ADS7828_CHANNEL_COUNT) {
			return false;
		}

		parsed_mask |= static_cast<uint8_t>(1u << channel);

		if (*endptr == '\0') {
			break;
		}

		if (*endptr != ',') {
			return false;
		}

		cursor = endptr + 1;
	}

	channel_mask = parsed_mask;
	return parsed_mask != 0;
}
} // namespace

void ADS7828::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ads7828", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("adc");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x48);
	PRINT_MODULE_USAGE_PARAM_INT('n', 2, 1, 8, "Number of thermistor channels starting at ADS7828 channel 0", true);
	PRINT_MODULE_USAGE_PARAM_STRING('c', "0,1", "0,1,...,7",
				       "Comma-separated ADS7828 channel list to sample. Overrides -n.", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_PARAM_COMMENT(
		"Publishes raw ADS7828 samples on adc_report. By default channels 0 and 1 are sampled.");
}

extern "C" int ads7828_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = ADS7828;
	BusCLIArguments cli{true, false};
	cli.custom1 = ADS7828_DEFAULT_CHANNEL_MASK;
	bool channel_count_set = false;
	uint8_t channel_count_mask = ADS7828_DEFAULT_CHANNEL_MASK;
	bool channel_list_set = false;
	uint8_t channel_list_mask = ADS7828_DEFAULT_CHANNEL_MASK;

	cli.default_i2c_frequency = 100000;
	cli.i2c_address = 0x48;

	while ((ch = cli.getOpt(argc, argv, "n:c:")) != EOF) {
		switch (ch) {
		case 'n': {
				const long channel_count = strtol(cli.optArg(), nullptr, 0);

				if (channel_count < 1 || channel_count > ADS7828_CHANNEL_COUNT) {
					PX4_ERR("invalid thermistor channel count: %ld", channel_count);
					ThisDriver::print_usage();
					return -1;
				}

				channel_count_mask = channel_mask_from_count(channel_count);
				channel_count_set = true;
				break;
			}

		case 'c': {
				uint8_t channel_mask = 0;

				if (!parse_channel_list(cli.optArg(), channel_mask)) {
					PX4_ERR("invalid thermistor channel list: %s", cli.optArg());
					ThisDriver::print_usage();
					return -1;
				}

				channel_list_mask = channel_mask;
				channel_list_set = true;
				break;
			}
		}
	}

	if (channel_list_set) {
		cli.custom1 = channel_list_mask;

	} else if (channel_count_set) {
		cli.custom1 = channel_count_mask;
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ADC_DEVTYPE_ADS7828);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
