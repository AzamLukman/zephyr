/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <device.h>
#include <sys/printk.h>
#include <drivers/sensor.h>
#include <sys/reboot.h>
#include <drivers/gpio.h>
#include <init.h>
#include <sys/util.h>
#include <inttypes.h>


static struct sensor_value val[3];

int main(void)
{
	uint8_t rc;

	const struct device *dev =device_get_binding(DT_LABEL(DT_INST(0, vemsee_bgt3001)));

		if (!device_is_ready(dev)) {
		printk("sensor: device not found.\n");
		// sys_reboot(1);
		return 0;
	}
	while (1) {
	sensor_sample_fetch(dev);
	if (sensor_channel_get(dev, SENSOR_CHAN_SOIL_NPK, val) != 0) {
			printk("sensor: channel get fail.\n");
			// return;
		}
	printk("N %.1f mg/kg\t P:%.1f mg/kg\tK:%.1f mg/kg\r\n",sensor_value_to_double(&val[0]),
						sensor_value_to_double(&val[1]),
						sensor_value_to_double(&val[2]));

	k_msleep(1000);
	}
}

