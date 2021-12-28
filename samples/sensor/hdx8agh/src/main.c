/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/sensor.h>
#include <drivers/sensor/hdx8agh.h>

void main(void)
{
	const struct device *dev = device_get_binding("HD8AGH");
	struct sensor_value val[8];
	// int ret;

	printk(" HDX8AGH 8-Channel current  sensor application %s\n", dev->name);

	if (!dev) {
		printk("sensor: device not found.\n");
		return;
	}
	// sensor_sample_fetch(dev);
	while (1) {
		if (sensor_channel_get(dev, SENSOR_CHAN_CURRENT_HDX8AGH_2, val) != 0) {
			printk("sensor: channel get fail.\n");
			return;
		}
		printk("Unit 2 Current reading:A:%.2f\tB:%.2f\tC:%.2f\tD:%.2f\tE:%.2f\tF:%.2f\tG:%.2f\tH:%.2f\n",
		       sensor_value_to_double(&val[0]), sensor_value_to_double(&val[1]),
		       sensor_value_to_double(&val[2]), sensor_value_to_double(&val[3]),
		       sensor_value_to_double(&val[4]), sensor_value_to_double(&val[5]),
		       sensor_value_to_double(&val[6]), sensor_value_to_double(&val[7]));

		k_msleep(2000);
	}
}
