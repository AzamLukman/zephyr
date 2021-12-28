/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for MH-Z19B CO2 Sensor
 *
 * Some capabilities and operational requirements for this sensor
 * cannot be expressed within the sensor driver abstraction.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MHZ19B_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MHZ19B_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/sensor.h>

enum sensor_channel_vmszs {
	/** Automatic Baseline Correction Self Calibration Function. */
	SENSOR_CHAN_NOISE = SENSOR_CHAN_COMMON_COUNT,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MHZ19B_H_ */
