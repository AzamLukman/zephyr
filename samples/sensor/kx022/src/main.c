/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <math.h>
// #include <drivers/esp_at.h>

#define MAX_TEST_TIME 1500
#define SLEEPTIME 300
#define KX022_TILT_POS_FU 0x01
#define KX022_TILT_POS_FD 0x02
#define KX022_TILT_POS_UP 0x04
#define KX022_TILT_POS_DO 0x08
#define KX022_TILT_POS_RI 0x010
#define KX022_TILT_POS_LE 0x20
#define KX022_ZPWU 0x01
#define KX022_ZNWU 0x02
#define KX022_YPMU 0x04
#define KX022_YNWU 0x08
#define KX022_XPWU 0x010
#define KX022_XNWU 0x20
static uint8_t slope_test_done = 0;
static uint8_t motion_test_done = 0;

static void tilt_position(struct sensor_value *value)
{
	uint32_t data;

	data = (uint8_t)value->val1;

	if (data == KX022_TILT_POS_LE) {
		printk(" Tilt Position: Face-Up(Z+)\r\n");
	} else if (data == KX022_TILT_POS_RI) {
		printk(" Tilt Position: Face-Down(Z-)\r\n");
	} else if (data == KX022_TILT_POS_DO) {
		printk(" Tilt Position: Up(Y+)\r\n");
	} else if (data == KX022_TILT_POS_UP) {
		printk(" Tilt Position: Down(Y-)\r\n");
	} else if (data == KX022_TILT_POS_FD) {
		printk(" Tilt Position: Right(X+)\r\n");
	} else if (data == KX022_TILT_POS_FU) {
		printk(" Tilt Position: Left (X-)\r\n");
	} else {
		printk("Not support for multiple axis\r\n");
	}
}

static void motion_direction(struct sensor_value *value)
{
	uint32_t data;

	data = (uint8_t)value->val1;

	if (data == KX022_ZPWU) {
		printk("Z+\r\n");
	} else if (data == KX022_ZNWU) {
		printk("Z-\r\n");
	} else if (data == KX022_YPMU) {
		printk("Y+\r\n");
	} else if (data == KX022_YNWU) {
		printk("Y-\r\n");
	} else if (data == KX022_XPWU) {
		printk("X+\r\n");
	} else if (data == KX022_XNWU) {
		printk("X-\r\n");
	} else {
		printk("Not support for multiple axis\r\n");
	}
}
int accel_magnitude_cal(struct sensor_value *val)
{
	static float tmp =0 ,tx,ty,tz;
	float Mag,cal,x,y,z;
	float x2,y2,z2;
	x = sensor_value_to_double(&val[0]);
	y = sensor_value_to_double(&val[1]);
	z = sensor_value_to_double(&val[2]);

	x2 = pow(sensor_value_to_double(&val[0]),2);
	y2 = pow(sensor_value_to_double(&val[1]),2);
	z2 = pow(sensor_value_to_double(&val[2]),2);


	Mag = sqrt(x2+y2+z2);
	cal =Mag - tmp;
	// printk("Magnitude :%.4f\t tmp:%.4f\t cal :%.4f\r\n",Mag,tmp,cal);
	// printk("x:%.4f\ty:%.4f\tz:%.4f\r\n",x-tx,y-ty,z-tz);
	printk("Mag:%.4f\tcal:%.4f\r\n",Mag,cal);

	tmp = Mag;
	tx =x;
	ty=y;
	tz =z;

}
static void fetch_and_display(const struct device *sensor)
{
	static unsigned int count;
	struct sensor_value accel[3];
	const char *overrun = "";

	int rc = sensor_sample_fetch(sensor);

	++count;
	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_KX022_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);
	}

	if (rc < 0) {
		printk("ERROR: Update failed: %d\n", rc);
	} else {
		// printk("#%u @ %u ms: %sx %f , y %f , z %f\n", count, k_uptime_get_32(), overrun,
		//        sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
		//        sensor_value_to_double(&accel[2]));
		       accel_magnitude_cal(&accel);
	}
}
static void motion_display(const struct device *sensor)
{
	struct sensor_value rd_data[1];
	const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);

	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_KX022_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor, SENSOR_CHAN_FREE_FALL, rd_data);
	}
	if (rc < 0) {
		printk("ERROR: Update failed: %d\n", rc);
	} else {
		printk("Motion Direction :\t");
		motion_direction(&rd_data[0]);
	}
}
static void tilt_position_display(const struct device *sensor)
{
	struct sensor_value rd_data[2];
	const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);

	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_KX022_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor, SENSOR_CHAN_NEAR_FAR, rd_data);
	}
	if (rc < 0) {
		printk("ERROR: Update failed: %d\n", rc);
	} else {
		printk("Previous Position :\t");
		tilt_position(&rd_data[0]);
		printk("Current Position :\t");
		tilt_position(&rd_data[1]);
	}
}

#ifdef CONFIG_KX022_TRIGGER
static void trigger_handler(const struct device *dev, struct sensor_trigger *trig)
{
	fetch_and_display(dev);
	motion_display(dev);
	tilt_position_display(dev);
}

static void motion_handler(const struct device *dev, struct sensor_trigger *trig)
{
	static unsigned int motion_cnt;
	printk ("motion trig\r\n");
	fetch_and_display(dev);
	// motion_display(dev);
	if (++motion_cnt > 5) {
		motion_test_done = 1;
		motion_cnt = 0;
	}
}
static void slope_handler(const struct device *dev, struct sensor_trigger *trig)
{
	static unsigned int slope_cont ;

	fetch_and_display(dev);
	tilt_position_display(dev);
	if (++slope_cont > 5) {
		slope_test_done = 1;
		slope_cont = 0;
	}
}
#endif

static void test_polling_mode(const struct device *dev)
{
	int32_t remaining_test_time = MAX_TEST_TIME;

	do {
		fetch_and_display(dev);
		/* wait a while */
		k_msleep(20);

		remaining_test_time -= SLEEPTIME;
	} while (remaining_test_time > 0);
}
static void test_trigger_mode(const struct device *dev)
{
	struct sensor_trigger trig;
	uint8_t rc;

	trig.type = SENSOR_TRIG_NEAR_FAR;
	trig.chan = SENSOR_CHAN_ALL;

	// rc = sensor_trigger_set(dev, &trig, slope_handler);
	// printk("\n");
	// printk("\t\tAccelerometer: Tilt Position trigger test Start\r\n");

	// while (slope_test_done == 0) {
	// 	k_sleep(K_MSEC(200));
	// }
	// slope_test_done = 0;
	// printk("\n");
	// printk("\t\tAccelerometer: Tilt Position trigger test finished\r\n");

	trig.type = SENSOR_TRIG_DATA_READY;//SENSOR_TRIG_FREEFALL;
	trig.chan = SENSOR_CHAN_ALL;

	rc = sensor_trigger_set(dev, &trig, motion_handler);
	// printk("\n");
	// printk("\t\tAccelerometer: Motion  trigger test Start\r\n");
	while (motion_test_done == 0) {
		k_sleep(K_MSEC(2000));
	}
	motion_test_done = 0;
	// printk("\n");
	// printk("\t\tAccelerometer: Motion trigger test finished\r\n");
	rc = sensor_attr_set(dev, SENSOR_CHAN_FREE_FALL, SENSOR_ATTR_WUFF_TH, 0);
	k_sleep(K_MSEC(500));
}

void main(void)
{
	const struct device *sensor = device_get_binding(DT_LABEL(DT_INST(0, kionix_kx022)));
	struct sensor_trigger trig;
	uint8_t rc;

	if (sensor == NULL) {
		printk("Could not get %s device\n", DT_LABEL(DT_INST(0, kionix_kx022)));
		return;
	}

	trig.type = SENSOR_TRIG_FREEFALL;
	trig.chan = SENSOR_CHAN_ALL;
	sensor_trigger_set(sensor, &trig, motion_handler);

	while (true) {
		test_polling_mode(sensor);
		// k_msleep(50);
		// test_trigger_mode(sensor);
	}
}
