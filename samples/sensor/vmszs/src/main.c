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
#include <drivers/sensor/vmszs.h>
#include <settings/settings.h>
#include <shell/shell.h>

struct sensor_value val[1];
struct sensor_value temp, hum;

 int toff = 0;
 int hoff = 0;
 int soff = 0;

struct settings_read_callback_params
{
	bool value_found;
	void *value;
};

static int settings_read_callback(const char *key,
								  size_t len,
								  settings_read_cb read_cb,
								  void *cb_arg,
								  void *param)
{
	ssize_t num_read_bytes = MIN(len, 32);
	struct settings_read_callback_params *params = param;

	/* Process only the exact match and ignore descendants of the searched name
	 */
	if (settings_name_next(key, NULL) != 0)
	{
		return 0;
	}

	params->value_found = true;
	num_read_bytes = read_cb(cb_arg, params->value, num_read_bytes);

	return 0;
}

int config_save(char *settings_name, void *value, size_t len)
{
	return settings_save_one(settings_name, value, len);
}

int config_apply(char *settings_name, void *value)
{
	int err;
	struct settings_read_callback_params params = { .value_found = false,
													.value = value };

	err = settings_load_subtree_direct(settings_name,
									   settings_read_callback,
									   &params);

	return 0;
}

int config_init(void)
{
	int rc;

	rc = settings_subsys_init();
	if (rc)
	{
		return rc;
	}
		config_apply("sensor/temp", &toff);
		config_apply("sensor/humid", &hoff);
		config_apply("sensor/sound", &soff);

	config_save("sensor/temp", &toff, sizeof(toff));
	config_save("sensor/humid", &hoff, sizeof(hoff));
	config_save("sensor/sound", &soff, sizeof(soff));


	return 0;
}


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
	// printk("/SHT3XD,%.2f,%0.2f/",
	// 	       sensor_value_to_double(&temp),
	// 	       sensor_value_to_double(&hum));


}
int main(void)
{
			config_init();

	// while(1)
	// {
		sound();
		// k_sleep(K_MSEC(2000));

		sht3x();
		printk("/*Sensor test,%.2f,%0.2f,%0.2f*/",
			sensor_value_to_double(&temp)+(toff),
		       sensor_value_to_double(&hum)+(hoff),
		       sensor_value_to_double(&val[0])+(soff));
		k_sleep(K_MSEC(500));
	// }
}

static int cmd_temp_set(const struct shell *shell, size_t argc, char *argv[])
{

	int temp;
	char *ptr;

	temp = (int)strtol(argv[2], &ptr, 10);

	shell_print(shell, "old temp = %d", toff);
	settings_save_one(argv[1], &temp, sizeof(temp));

	config_apply(argv[1], &toff);
		shell_print(shell, "new temp = %d", toff);

	return 0;
}

static int cmd_hum_set(const struct shell *shell, size_t argc, char *argv[])
{

	int temp;
	char *ptr;

	temp = (int)strtol(argv[2], &ptr, 10);

	shell_print(shell, "old humidity = %d", hoff);
	settings_save_one(argv[1], &temp, sizeof(temp));

	config_apply(argv[1], &hoff);
		shell_print(shell, "new humidity = %d", hoff);

	return 0;
}
static int cmd_sound_set(const struct shell *shell, size_t argc, char *argv[])
{

	int temp;
	char *ptr;

	temp = (int)strtol(argv[2], &ptr, 10);

	shell_print(shell, "old sound = %d", soff);
	settings_save_one(argv[1], &temp, sizeof(temp));

	config_apply(argv[1], &soff);
		shell_print(shell, "new sound = %d", toff);

	return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(smp_cmds,
	SHELL_CMD_ARG(tem, NULL, "Set temp offset", cmd_temp_set, 2, 2),
	SHELL_CMD_ARG(hmd, NULL, "Set humidity offset", cmd_hum_set, 2, 2),
	SHELL_CMD_ARG(snd, NULL, "Set sound offset", cmd_sound_set, 2, 2),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(smp, &smp_cmds, "SMP shell commands", NULL);
