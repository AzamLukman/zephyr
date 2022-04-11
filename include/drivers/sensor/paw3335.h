/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for PixArt PA3335 mouse sensor
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_PAW3335_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_PAW3335_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/sensor.h>
#include <device.h>

enum sensor_config_paw3335 {

	SENSOR_CFG_AXIS_SWAP_XY,

	SENSOR_CFG_AXIS_INV_Y,

	SENSOR_CFG_AXIS_INV_X,

	SENSOR_CFG_RESOLUTION
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_PAW3335_H_ */









