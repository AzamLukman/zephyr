/* Kionix KX022 3-axis accelerometer driver
 *
 * Copyright (c) 2021-2022 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "kx022.h"

#define DT_DRV_COMPAT kionix_kx022

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
static int kx022_i2c_read_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
			       uint8_t len)
{
	const struct kx022_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->bus_cfg, reg_addr, value, len);
}

static int kx022_i2c_write_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
				uint8_t len)
{
	const struct kx022_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->bus_cfg, reg_addr, value, len);
}

static int kx022_i2c_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
	const struct kx022_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->bus_cfg, reg_addr, value);
}

static int kx022_i2c_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t value)
{
	const struct kx022_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->bus_cfg, reg_addr, value);
}

static int kx022_i2c_update_reg(const struct device *dev, uint8_t reg_addr, uint8_t mask,
				uint8_t value)
{
	const struct kx022_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->bus_cfg, reg_addr, mask, value);
}

static const struct kx022_transfer_function kx022_i2c_transfer_fn = {
	.read_data = kx022_i2c_read_data,
	.write_data = kx022_i2c_write_data,
	.read_reg = kx022_i2c_read_reg,
	.write_reg = kx022_i2c_write_reg,
	.update_reg = kx022_i2c_update_reg,
};

int kx022_i2c_init(const struct device *dev)
{
	struct kx022_data *data = dev->data;
	const struct kx022_config *cfg = dev->config;

	data->hw_tf = &kx022_i2c_transfer_fn;

	if (!device_is_ready(cfg->bus_cfg.bus)) {
		return -ENODEV;
	}

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
