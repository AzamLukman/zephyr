/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <sys/printk.h>
#include <drivers/sensor.h>
#include <drivers/sensor/vmszs.h>

struct sensor_value val[1];
struct sensor_value temp, hum;

static int toff = 0;
static int hoff = 0;
static int soff = 0;

void sound(void)
{
	const struct device *dev =device_get_binding(DT_LABEL(DT_INST(0, vemsee_vmszs)));

	// int ret;


	if (!dev) {
		printk("sensor: device not found.\n");
		// return 0;
	}
	// while (1) {
	sensor_sample_fetch(dev);
	if (sensor_channel_get(dev, SENSOR_CHAN_NOISE, val) != 0) {
			printk("sensor: channel get fail.\n");
			// return;
		}
	// printk("Sound %.2f dbA\r\n",sensor_value_to_double(&val[0]));
	// }

}

void sht3x(void)
{
	const struct device *dev = device_get_binding("SHT3XD");
	struct sensor_value last_temp, last_hum;

	int rc;


	if (dev == NULL) {
		printk("Could not get SHT3XD device\n");
		// return;
	}

	// printk(" get SHT3XD device\n");

	rc = sensor_sample_fetch(dev);
		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
						&temp);
			last_temp = temp;
		}
		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY,
						&hum);
			last_hum = hum;
		}
		if (rc != 0) {
			// printf("SHT3XD: failed: %d\n", rc);
			// hum = last_hum;
			// temp =last_temp;

		}
	// printk("/*SHT3XD,%.2f,%0.2f*/",
	// 	       sensor_value_to_double(&temp),
	// 	       sensor_value_to_double(&hum));


}
int main(void)
{
	while(1)
	{
		sound();
		// k_sleep(K_MSEC(2000));

		sht3x();
		printk("/*Sensor test,%.2f,%0.2f,%0.2f*/",
			sensor_value_to_double(&temp)+(toff),
		       sensor_value_to_double(&hum)+(hoff),
		       sensor_value_to_double(&val[0])+(soff));
		k_sleep(K_MSEC(500));
	}
}
