/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/sensor.h>
#include <drivers/sensor/jsymk163.h>

void main(void)
{
	const struct device *dev = device_get_binding("MK-163");
	struct sensor_value val[4];
	// int ret;

	printk(" MK-163 1-Channel current  sensor application %s\n", dev->name);

	if (!dev) {
		printk("sensor: device not found.\n");
		return;
	}
	printk("sensor: device found %s.\n",dev->name);
	while (1) {
	// sensor_sample_fetch(dev);

		if (sensor_channel_get(dev, SENSOR_CHAN_CURRENT_JSYMK163_1, val) != 0) {
			printk("sensor: channel get fail.\n");
			return;
		}
		// sensor_channel_get(dev, SENSOR_CHAN_CURRENT_1, val);
		printk("Unit 1 Current reading:A:%.2f\tB:%.2f\tC:%.2f\tD:%.2f\n",
		       sensor_value_to_double(&val[0]), sensor_value_to_double(&val[1]),
		       sensor_value_to_double(&val[2]), sensor_value_to_double(&val[3]));
		k_msleep(500);

	}
}
