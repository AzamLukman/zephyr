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
#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_JSYMK163_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_JSYMK163_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <device.h>
#include <drivers/sensor.h>

enum sensor_channel_jsymk163 {
	/** Automatic Baseline Correction Self Calibration Function. */
	SENSOR_CHAN_CURRENT_JSYMK163_1= SENSOR_CHAN_COMMON_COUNT,
	SENSOR_CHAN_CURRENT_JSYMK163_2,
	SENSOR_CHAN_CURRENT_JSYMK163_3,

};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_JSYMK163_H_ */
