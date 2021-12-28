/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/sensor.h>
#include <drivers/sensor/vmszs.h>
int accel_test(void)
{
	const struct device *sensor = device_get_binding(DT_LABEL(DT_INST(0, kionix_kx022)));

	fetch_and_display(sensor);
}
void main(void)
{
	const struct device *dev =device_get_binding(DT_LABEL(DT_INST(0, vemsee_vmszs)));
	struct sensor_value val[1];
	// int ret;


	if (!dev) {
		printk("sensor: device not found.\n");
		return 0;
	}
	while (1) {
	sensor_sample_fetch(dev);
	if (sensor_channel_get(dev, SENSOR_CHAN_NOISE, val) != 0) {
			printk("sensor: channel get fail.\n");
			return;
		}
	printk("Sound %.2f dbA\r\n",sensor_value_to_double(&val[0]));
	}

}
