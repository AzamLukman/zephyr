/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <sys/reboot.h>
#include <drivers/sensor/kx022.h>

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
#define SENSOR_WAIT 200

static uint8_t slope_test_done;
static uint8_t motion_test_done;


static void tilt_position(struct sensor_value *value)
{
	uint32_t data;

	data = (uint8_t)value->val1;

	if (data == KX022_TILT_POS_LE) {
		printf(" Tilt Position: Face-Up(Z+)\r\n");
	} else if (data == KX022_TILT_POS_RI) {
		printf(" Tilt Position: Face-Down(Z-)\r\n");
	} else if (data == KX022_TILT_POS_DO) {
		printf(" Tilt Position: Up(Y+)\r\n");
	} else if (data == KX022_TILT_POS_UP) {
		printf(" Tilt Position: Down(Y-)\r\n");
	} else if (data == KX022_TILT_POS_FD) {
		printf(" Tilt Position: Right(X+)\r\n");
	} else if (data == KX022_TILT_POS_FU) {
		printf(" Tilt Position: Left (X-)\r\n");
	} else {
		printf("Not support for multiple axis\r\n");
	}
}

static void motion_direction(struct sensor_value *value)
{
	uint32_t data;

	data = (uint8_t)value->val1;

	if (data == KX022_ZPWU) {
		printf("Z+\r\n");
	} else if (data == KX022_ZNWU) {
		printf("Z-\r\n");
	} else if (data == KX022_YPMU) {
		printf("Y+\r\n");
	} else if (data == KX022_YNWU) {
		printf("Y-\r\n");
	} else if (data == KX022_XPWU) {
		printf("X+\r\n");
	} else if (data == KX022_XNWU) {
		printf("X-\r\n");
	} else {
		printf("Not support for multiple axis\r\n");
	}
}

static void fetch_and_display(const struct device *sensor)
{
	static unsigned int count;
	struct sensor_value accel[3];
	const char *overrun = "";
	int rc = sensor_sample_fetch_chan(sensor, SENSOR_CHAN_ACCEL_XYZ);

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
		printf("ERROR: Update failed: %d\n", rc);
	} else {
		printf("#%u @ %u ms: %sx %f , y %f , z %f\n", count, k_uptime_get_32(), overrun,
		       sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));
	}
}

static void motion_display(const struct device *sensor)
{
	struct sensor_value rd_data[1];
	const char *overrun = "";
	int rc = sensor_sample_fetch_chan(sensor, SENSOR_CHAN_KX022_MOTION);

	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_KX022_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor, SENSOR_CHAN_KX022_MOTION, rd_data);
	}
	if (rc < 0) {
		printf("ERROR: Update failed: %d\n", rc);
	} else {
		printf("Motion Direction :\t");
		motion_direction(&rd_data[0]);
	}
}

static void tilt_position_display(const struct device *sensor)
{
	struct sensor_value rd_data[2];
	const char *overrun = "";
	int rc = sensor_sample_fetch_chan(sensor, SENSOR_CHAN_KX022_TILT);

	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_KX022_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor, SENSOR_CHAN_KX022_TILT, rd_data);
	}
	if (rc < 0) {
		printf("ERROR: Update failed: %d\n", rc);
	} else {
		printf("Previous Position :\t");
		tilt_position(&rd_data[0]);
		printf("Current Position :\t");
		tilt_position(&rd_data[1]);
	}
}

#ifdef CONFIG_KX022_TRIGGER
static void motion_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	static unsigned int motion_cnt;

	fetch_and_display(dev);
	motion_display(dev);
	if (++motion_cnt > 5) {
		motion_test_done = 1;
		motion_cnt = 0;
	}
}
static void tilt_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	static unsigned int slope_cont;

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

	printf("\n");
	printf("\t\tAccelerometer: Poling test Start %s\r\n",dev->name);
	do {
		fetch_and_display(dev);
		/* wait a while */
		k_msleep(SLEEPTIME);

		remaining_test_time -= SLEEPTIME;
	} while (remaining_test_time > 0);
}

static void test_trigger_mode(const struct device *dev)
{
	struct sensor_trigger trig;
	uint8_t rc;

	trig.type = SENSOR_TRIG_KX022_MOTION;

	rc = sensor_trigger_set(dev, &trig, motion_handler);
	printf("\n");
	printf("\t\tAccelerometer: Motion  trigger test Start %s\r\n",dev->name);

	while (motion_test_done == 0) {
		k_sleep(K_MSEC(SENSOR_WAIT));
	}
	motion_test_done = 0;
	printf("\n");
	printf("\t\tAccelerometer: Motion trigger test finished\r\n");

	rc = kx022_restore_default_trigger_setup(dev, &trig);

	k_sleep(K_MSEC(SENSOR_WAIT));

	trig.type = SENSOR_TRIG_KX022_TILT;

	rc = sensor_trigger_set(dev, &trig, &tilt_handler);
	printf("\n");
	printf("\t\tAccelerometer: Tilt Position trigger test Start\r\n");

	while (slope_test_done == 0) {
		k_sleep(K_MSEC(SENSOR_WAIT));
	}
	slope_test_done = 0;
	printf("\n");
	printf("\t\tAccelerometer: Tilt Position trigger test finished\r\n");

	rc = kx022_restore_default_trigger_setup(dev, &trig);
}

static void test_runtime_cfg(const struct device *dev)
{
#if CONFIG_KX022_FS_RUNTIME
	val.val1 = 1;
	rc = sensor_attr_set(dev, SENSOR_CHAN_KX022_CFG,
			     SENSOR_ATTR_FULL_SCALE, &val);
#endif

#if CONFIG_KX022_ODR_RUNTIME
	val.val1 = 3;
	rc = sensor_attr_set(dev, SENSOR_CHAN_KX022_CFG,
			     SENSOR_ATTR_KX022_ODR, &val);
#endif

#if CONFIG_KX022_RES_RUNTIME
	val.val1 = 0;
	rc = sensor_attr_set(dev, SENSOR_CHAN_KX022_CFG,
			     SENSOR_ATTR_KX022_RESOLUTION, &val);
#endif

#if CONFIG_KX022_MOTION_DETECTION_TIMER_RUNTIME
	val.val1 = 2;
	rc = sensor_attr_set(dev, SENSOR_CHAN_KX022_CFG,
			     SENSOR_ATTR_KX022_MOTION_DETECTION_TIMER,
			     &val);
#endif

#if CONFIG_KX022_TILT_TIMER_RUNTIME
	val.val1 = 3;
	rc = sensor_attr_set(dev, SENSOR_CHAN_KX022_CFG,
			     SENSOR_ATTR_KX022_TILT_TIMER, &val);
#endif

#if CONFIG_KX022_MOTION_DETECT_THRESHOLD_RUNTIME
	val.val1 = 4;
	rc = sensor_attr_set(dev, SENSOR_CHAN_KX022_CFG,
			     SENSOR_ATTR_KX022_MOTION_DETECT_THRESHOLD,
			     &val);
#endif

#if CONFIG_KX022_TILT_ANGLE_LL_RUNTIME
	val.val1 = 16;
	rc = sensor_attr_set(dev, SENSOR_CHAN_KX022_CFG,
			     SENSOR_ATTR_KX022_TILT_ANGLE_LL, &val);
#endif
}

void main(void)
{
	const struct device *sensor = device_get_binding("KX022");
	const struct device *sensor2 = device_get_binding("KX022_2");

	if (!device_is_ready(sensor)) {
		printf("Device kx022 device 1 is not ready\n");
		sys_reboot(SYS_REBOOT_COLD);
		return;
	}

	if (!device_is_ready(sensor2)) {
		printf("Device kx022 device 2 is not ready\n");
		sys_reboot(SYS_REBOOT_COLD);
		return;
	}

	test_runtime_cfg(sensor);

	while (true) {
		test_polling_mode(sensor);
#ifdef CONFIG_KX022_TRIGGER
		test_trigger_mode(sensor);
#endif
		k_msleep(SENSOR_WAIT);
		test_polling_mode(sensor2);
#ifdef CONFIG_KX022_TRIGGER
		test_trigger_mode(sensor2);
#endif
	}
}
