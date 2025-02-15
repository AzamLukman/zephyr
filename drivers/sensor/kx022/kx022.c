/* Kionix KX022 3-axis accelerometer driver
 *
 * Copyright (c) 2021-2022 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kionix_kx022

#include "kx022.h"

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error "KX022 driver enabled without any devices"
#endif

LOG_MODULE_REGISTER(KX022, CONFIG_SENSOR_LOG_LEVEL);

/************************************************************
 * info: kx022_mode use to set pc1 in standby or operating mode
 * input: 0 - standby
 *        1 - operating
 * **********************************************************/
static int kx022_mode(const struct device *dev, bool mode)
{
	struct kx022_data *data = dev->data;
	uint8_t chip_id;
	uint8_t val = mode << KX022_CNTL1_PC1_SHIFT;
	int ret;

	if (mode == KX022_STANDY_MODE) {
		ret = data->hw_tf->read_reg(dev, KX022_REG_WHO_AM_I, &chip_id);
		if (ret) {
			LOG_DBG("%s: Failed to read %s: %d", dev->name, "chip id", ret);
			return ret;
		}

		if (chip_id != KX022_VAL_WHO_AM_I) {
			LOG_DBG("%s: Invalid chip id 0x%x", dev->name, chip_id);
			return -EIO;
		}

		ret = data->hw_tf->update_reg(dev, KX022_REG_CNTL1, KX022_MASK_CNTL1_PC1, val);
		if (ret) {
			LOG_DBG("%s: Failed to update %s: %d", dev->name, "standby", ret);
		}

		return ret;
	}

	ret = data->hw_tf->update_reg(dev, KX022_REG_CNTL1, KX022_MASK_CNTL1_PC1, val);
	if (ret) {
		LOG_DBG("%s: Failed to update %s: %d", dev->name, "operating mode", ret);
	}

	return ret;
}

int kx022_operating_mode(const struct device *dev)
{
	return kx022_mode(dev, KX022_OPERATING_MODE);
}

int kx022_standby_mode(const struct device *dev)
{
	return kx022_mode(dev, KX022_STANDY_MODE);
}

#ifdef CONFIG_KX022_ODR_RUNTIME
static int kx022_accel_odr_set(const struct device *dev, uint16_t freq)
{
	struct kx022_data *data = dev->data;
	int ret;

	if (freq > KX022_ODR_RANGE_MAX) {
		return -EINVAL;
	}

	ret = data->hw_tf->update_reg(dev, KX022_REG_ODCNTL, KX022_MASK_ODCNTL_OSA, (uint8_t)freq);
	if (ret) {
		LOG_DBG("%s: Failed to update %s: %d", dev->name, "ODR", ret);
	}

	return ret;
}
#endif /* defined(KX022_ODR_RUNTIME) */

#ifdef CONFIG_KX022_FS_RUNTIME
static int kx022_accel_range_set(const struct device *dev, int32_t range)
{
	struct kx022_data *data = dev->data;
	int ret;

	if (range > KX022_FS_RANGE_MAX) {
		return -EINVAL;
	}

	ret = data->hw_tf->update_reg(dev, KX022_REG_CNTL1, KX022_MASK_CNTL1_GSEL,
				      ((uint8_t)range << KX022_CNTL1_GSEL_SHIFT));
	if (ret) {
		LOG_DBG("%s: Failed to update %s: %d", dev->name, "full-scale range", ret);
		return ret;
	}

	if (range == KX022_FS_2G) {
		data->gain = (float)(range * GAIN_XL);
	} else {
		data->gain = (float)(2 * range * GAIN_XL);
	}

	return 0;
}
#endif /* KX022_FS_RUNTIME */

#ifdef CONFIG_KX022_RES_RUNTIME
static int kx022_accel_res_set(const struct device *dev, uint16_t res)
{
	struct kx022_data *data = dev->data;
	int ret;

	if (res > KX022_RES_RANGE_MAX) {
		return -EINVAL;
	}

	ret = data->hw_tf->update_reg(dev, KX022_REG_CNTL1, KX022_MASK_CNTL1_RES,
				      ((uint8_t)res << KX022_CNTL1_RES_SHIFT));
	if (ret) {
		LOG_DBG("%s: Failed to update %s: %d", dev->name, "RES", ret);
	}

	return ret;
}
#endif /* KX022_RES_RUNTIME */

#ifdef CONFIG_KX022_MOTION_DETECTION_TIMER_RUNTIME
static int kx022_accel_motion_detection_timer_set(const struct device *dev, uint16_t delay)
{
	struct kx022_data *data = dev->data;
	int ret;

	if (delay > KX022_WUFC_RANGE_MAX) {
		return -EINVAL;
	}

	ret = data->hw_tf->write_reg(dev, KX022_REG_WUFC, (uint8_t)delay);
	if (ret) {
		LOG_DBG("%s: Failed to write %s: %d", dev->name, "WUFC", ret);
	}

	return ret;
}
#endif /* KX022_MOTION_DETECTION_TIMER_RUNTIME */

#if CONFIG_KX022_TILT_TIMER_RUNTIME
static int kx022_accel_tilt_timer_set(const struct device *dev, uint16_t delay)
{
	struct kx022_data *data = dev->data;
	int ret;

	if (delay > KX022_TILT_TIMER_RANGE_MAX) {
		return -EINVAL;
	}

	ret = data->hw_tf->write_reg(dev, KX022_REG_TILT_TIMER, (uint8_t)delay);
	if (ret) {
		LOG_DBG("%s: Failed to write %s: %d", dev->name, "tilt timer", ret);
	}

	return ret;
}
#endif /* KX022_TILT_TIMER_RUNTIME */

#ifdef CONFIG_KX022_TILT_ANGLE_LL_RUNTIME
static int kx022_accel_tilt_angle_set(const struct device *dev, uint16_t angle)
{
	struct kx022_data *data = dev->data;
	int ret;

	if (angle > KX022_TILT_ANGLE_LL_RANGE_MAX) {
		return -EINVAL;
	}

	ret = data->hw_tf->write_reg(dev, KX022_REG_TILT_ANGLE_LL, (uint8_t)angle);
	if (ret) {
		LOG_DBG("%s: Failed to write %s: %d", dev->name, "tilt angle ll", ret);
	}

	return ret;
}
#endif /* CONFIG_KX022_TILT_ANGLE_LL_RUNTIME */

#ifdef CONFIG_KX022_MOTION_DETECT_THRESHOLD_RUNTIME
static int kx022_accel_motion_detect_threshold_set(const struct device *dev, uint16_t ath)
{
	struct kx022_data *data = dev->data;
	int ret;

	if (ath > KX022_ATH_RANGE_MAX) {
		return -EINVAL;
	}

	ret = data->hw_tf->write_reg(dev, KX022_REG_ATH, (uint8_t)ath) if (ret)
	{
		LOG_DBG("%s: Failed to write %s: %d", dev->name, "ATH", ret);
	}

	return ret;
}
#endif /* CONFIG_KX022_MOTION_DETECT_THRESHOLD_RUNTIME */

static int kx022_accel_config(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	switch ((int)attr) {
#ifdef CONFIG_KX022_FS_RUNTIME
	case SENSOR_ATTR_FULL_SCALE:
		return kx022_accel_range_set(dev, val->val1);
#endif /* CONFIG_KX022_FS_RUNTIME */

#ifdef CONFIG_KX022_ODR_RUNTIME
	case SENSOR_ATTR_KX022_ODR:
		return kx022_accel_odr_set(dev, val->val1);
#endif /* CONFIG_KX022_ODR_RUNTIME */

#ifdef CONFIG_KX022_RES_RUNTIME
	case SENSOR_ATTR_KX022_RESOLUTION:
		return kx022_accel_res_set(dev, val->val1);
#endif /* CONFIG_KX022_RES_RUNTIME */

#ifdef CONFIG_KX022_MOTION_DETECTION_TIMER_RUNTIME
	case SENSOR_ATTR_KX022_MOTION_DETECTION_TIMER:
		return kx022_accel_motion_detection_timer_set(dev, val->val1);
#endif /* CONFIG_KX022_MOTION_DETECTION_TIMER_RUNTIME */

#ifdef CONFIG_KX022_MOTION_DETECT_THRESHOLD_RUNTIME
	case SENSOR_ATTR_KX022_MOTION_DETECT_THRESHOLD:
		return kx022_accel_motion_detect_threshold_set(dev, val->val1);
#endif /* CONFIG_KX022_MOTION_DETECT_THRESHOLD_RUNTIME */

#ifdef CONFIG_KX022_TILT_TIMER_RUNTIME
	case SENSOR_ATTR_KX022_TILT_TIMER:
		return kx022_accel_tilt_timer_set(dev, val->val1);
#endif /* CONFIG_KX022_TILT_TIMER_RUNTIME */

#ifdef CONFIG_KX022_TILT_ANGLE_LL_RUNTIME
	case SENSOR_ATTR_KX022_TILT_ANGLE_LL:
		return kx022_accel_tilt_angle_set(dev, val->val1);
#endif /* CONFIG_KX022_TILT_ANGLE_LL_RUNTIME */
	default:
		LOG_DBG("attribute %d not supported.", attr);
		return -ENOTSUP;
	}

	return 0;
}

static int kx022_attr_set(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret;

	ret = kx022_standby_mode(dev);
	if (ret) {
		return ret;
	}

	switch ((int)chan) {
	case SENSOR_CHAN_KX022_CFG:
		ret = kx022_accel_config(dev, chan, attr, val);
		break;
	default:
		LOG_WRN("%d channel not supported.", chan);
		ret = -ENOTSUP;
	}

	(void)kx022_operating_mode(dev);
	return ret;
}

#ifdef CONFIG_KX022_DIAGNOSTIC_MODE
int kx022_read_register_value(const struct device *dev, uint8_t reg, uint8_t *val)
{
	struct kx022_data *data = dev->data;
	int ret;

	ret = data->hw_tf->read_reg(dev, reg, &(*val));
	if (ret) {
		LOG_DBG("%s: Failed to read register 0x%X: %d", dev->name, reg, ret);
	}

	return ret;
}
#endif /*CONFIG_KX022_DIAGNOSTIC_MODE*/

static int kx022_sample_fetch_accel(const struct device *dev, uint8_t reg_addr)
{
	struct kx022_data *data = dev->data;
	uint8_t buf[2];
	uint16_t val;
	int ret;

	ret = data->hw_tf->read_data(dev, reg_addr, buf, sizeof(buf));
	if (ret) {
		LOG_DBG("%s: Failed to read %s: %d", dev->name, "sample", ret);
		return ret;
	}

	val = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));

	switch (reg_addr) {
	case KX022_REG_XOUT_L:
		data->sample_x = val;
		break;
	case KX022_REG_YOUT_L:
		data->sample_y = val;
		break;
	case KX022_REG_ZOUT_L:
		data->sample_z = val;
		break;
	default:
		LOG_DBG("%s: Invalid register address", dev->name);
		return -EIO;
	}

	return 0;
}

#define kx022_sample_fetch_accel_x(dev) kx022_sample_fetch_accel(dev, KX022_REG_XOUT_L)
#define kx022_sample_fetch_accel_y(dev) kx022_sample_fetch_accel(dev, KX022_REG_YOUT_L)
#define kx022_sample_fetch_accel_z(dev) kx022_sample_fetch_accel(dev, KX022_REG_ZOUT_L)

static int kx022_sample_fetch_accel_xyz(const struct device *dev)
{
	struct kx022_data *data = dev->data;
	uint8_t buf[6];
	uint8_t buf1[6];
	int ret;

	ret = data->hw_tf->read_data(dev, KX022_REG_XOUT_L, buf, sizeof(buf));
	if (ret) {
		LOG_DBG("%s: Failed to read %s: %d", dev->name, "sample", ret);
		return ret;
	}
	data->sample_x = (int16_t)sys_get_le16(&buf[0]);
	data->sample_y = (int16_t)sys_get_le16(&buf[2]);
	data->sample_z = (int16_t)sys_get_le16(&buf[4]);

	// printk("high filter x %d \t%d\t%d\r\n",(int16_t)sys_get_le16(&buf1[0]),
	// 				(int16_t)sys_get_le16(&buf1[2]),
	// 				(int16_t)sys_get_le16(&buf1[4]));
	return 0;
}

static int kx022_tilt_pos(const struct device *dev)
{
	struct kx022_data *data = dev->data;
	int ret;

	ret = data->hw_tf->read_reg(dev, KX022_REG_TSCP, &data->sample_tscp);
	if (ret) {
		return ret;
	}

	return data->hw_tf->read_reg(dev, KX022_REG_TSPP, &data->sample_tspp);
}

static int kx022_motion_direction(const struct device *dev)
{
	struct kx022_data *data = dev->data;

	return data->hw_tf->read_reg(dev, KX022_REG_INS3, &data->sample_motion_dir);
}

static int kx022_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret = 0;

	switch ((int)chan) {
	case SENSOR_CHAN_ACCEL_X:
		ret = kx022_sample_fetch_accel_x(dev);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ret = kx022_sample_fetch_accel_y(dev);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		ret = kx022_sample_fetch_accel_z(dev);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = kx022_sample_fetch_accel_xyz(dev);
		break;
	case SENSOR_CHAN_ALL:
		ret = kx022_sample_fetch_accel_xyz(dev);
		if (ret) {
			return ret;
		}

		ret = kx022_tilt_pos(dev);
		if (ret) {
			return ret;
		}

		ret = kx022_motion_direction(dev);
		break;
	case SENSOR_CHAN_KX022_MOTION:
		ret = kx022_motion_direction(dev);
		break;
	case SENSOR_CHAN_KX022_TILT:
		ret = kx022_tilt_pos(dev);
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

static inline void kx022_convert(struct sensor_value *val, int raw_val, float gain)
{
	int64_t dval;

	/* Gain is in mg/LSB */
	/* Convert to m/s^2 */
	dval = ((int64_t)raw_val * gain * SENSOR_G) / 1000;
	val->val1 = dval / 1000000LL;
	val->val2 = dval % 1000000LL;
}
static inline void kx022_tilt_pos_get(struct sensor_value *val, int raw_val)
{
	val->val1 = (int16_t)raw_val;
}

static inline void kx022_motion_dir_get(struct sensor_value *val, int raw_val)
{
	val->val1 = (int16_t)raw_val;
}
static inline int kx022_get_channel(enum sensor_channel chan, struct sensor_value *val,
				    struct kx022_data *data, float gain)
{
	switch ((int)chan) {
	case SENSOR_CHAN_ACCEL_X:
		kx022_convert(val, data->sample_x, gain);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		kx022_convert(val, data->sample_y, gain);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		kx022_convert(val, data->sample_z, gain);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		kx022_convert(val, data->sample_x, gain);
		kx022_convert(val + 1, data->sample_y, gain);
		kx022_convert(val + 2, data->sample_z, gain);
		break;
	case SENSOR_CHAN_ALL:
		kx022_convert(val, data->sample_x, gain);
		kx022_convert(val + 1, data->sample_y, gain);
		kx022_convert(val + 2, data->sample_z, gain);
		kx022_tilt_pos_get(val + 3, data->sample_tspp);
		kx022_tilt_pos_get(val + 4, data->sample_tscp);
		kx022_motion_dir_get(val + 5, data->sample_motion_dir);
		break;
	case SENSOR_CHAN_KX022_TILT:
		kx022_tilt_pos_get(val, data->sample_tspp);
		kx022_tilt_pos_get(val + 1, data->sample_tscp);
		break;
	case SENSOR_CHAN_KX022_MOTION:
		kx022_motion_dir_get(val, data->sample_motion_dir);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int kx022_channel_get(const struct device *dev, enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct kx022_data *data = dev->data;

	return kx022_get_channel(chan, val, data, data->gain);
}

static const struct sensor_driver_api kx022_api_funcs = {
	.attr_set = kx022_attr_set,
#ifdef CONFIG_KX022_TRIGGER
	.trigger_set = kx022_trigger_set,
#endif
	.sample_fetch = kx022_sample_fetch,
	.channel_get = kx022_channel_get,
};

static int kx022_init(const struct device *dev)
{
	const struct kx022_config *const cfg = dev->config;
	struct kx022_data *data = dev->data;
	uint8_t chip_id;
	uint8_t val;
	int ret;

	ret = cfg->bus_init(dev);
	if (ret) {
		return ret;
	}

	ret = data->hw_tf->read_reg(dev, KX022_REG_WHO_AM_I, &chip_id);
	if (ret) {
		LOG_DBG("%s: Failed to read %s: %d", dev->name, "chip id", ret);
		return ret;
	}

	if (chip_id != KX022_VAL_WHO_AM_I) {
		LOG_DBG("%s: Invalid chip id 0x%x", dev->name, chip_id);
		return -EIO;
	}

	/* s/w reset the sensor */
	ret = data->hw_tf->update_reg(dev, KX022_REG_CNTL2, KX022_MASK_CNTL2_SRST,
				      KX022_MASK_CNTL2_SRST);
	if (ret) {
		LOG_DBG("%s: s/w reset fail: %d", dev->name, ret);
		return ret;
	}

	/* Arbitrary delay for the device to be ready after reset */
	(void)k_msleep(KX022_RESET_DELAY);

	ret = data->hw_tf->read_reg(dev, KX022_REG_WHO_AM_I, &chip_id);
	if (ret) {
		LOG_DBG("%s: Failed to read %s: %d", dev->name, "chip id", ret);
		return ret;
	}

	/* Make sure the KX022 is stopped before we configure */
	val = (cfg->resolution << KX022_CNTL1_RES_SHIFT) |
	      (cfg->full_scale << KX022_CNTL1_GSEL_SHIFT);

	ret = data->hw_tf->write_reg(dev, KX022_REG_CNTL1, val);
	if (ret) {
		LOG_DBG("%s: Failed to write %s: %d", dev->name, "CNTL1", ret);
		return ret;
	}

	/* Set KX022 default odr */
	ret = data->hw_tf->update_reg(dev, KX022_REG_ODCNTL, KX022_MASK_ODCNTL_OSA, cfg->odr);
	if (ret) {
		LOG_DBG("%s: Failed to update %s: %d", dev->name, "ODR", ret);
		return ret;
	}

	ret = data->hw_tf->update_reg(dev, KX022_REG_CNTL3, KX022_MASK_CNTL3_OWUF,
				0x06);

#ifdef CONFIG_KX022_TRIGGER
	ret = kx022_trigger_init(dev);
	if (ret) {
		LOG_DBG("%s: Failed to initialize triggers: %d", dev->name, ret);
		return ret;
	}
#endif /* CONFIG_KX022_TRIGGER */

	/* Set Kx022 to Operating Mode */
	ret = kx022_operating_mode(dev);
	if (ret) {
		return ret;
	}

	if (cfg->full_scale == KX022_FS_2G) {
		data->gain = GAIN_XL;
	} else if (cfg->full_scale == KX022_FS_4G) {
		data->gain = 2 * GAIN_XL;
	} else if (cfg->full_scale == KX022_FS_8G) {
		data->gain = 4 * GAIN_XL;
	}

	/* After setting, need some delay, otherwise first polling data is wrong */;
	(void)k_msleep(KX022_INIT_DELAY);

	return 0;
}

#ifdef CONFIG_KX022_TRIGGER
#define KX022_CFG_IRQ(inst)                                                                        \
	.gpio_int = GPIO_DT_SPEC_INST_GET(inst, int_gpios), .int_pin = DT_INST_PROP(inst, int_pin),
#else
#define KX022_CFG_IRQ(inst)
#endif /* CONFIG_KX022_TRIGGER */

#define KX022_INIT(inst)                                                                           \
	static struct kx022_data dev_data_##inst;                                                  \
	static const struct kx022_config dev_config_##inst = {                                     \
		.bus_init = kx022_i2c_init,                                                        \
		.bus_cfg = I2C_DT_SPEC_INST_GET(inst),                                             \
		.int_pin_1_polarity = DT_INST_PROP(inst, int_pin_1_polarity),                      \
		.int_pin_1_response = DT_INST_PROP(inst, int_pin_1_response),                      \
		.full_scale = DT_INST_PROP(inst, full_scale),                                      \
		.odr = DT_INST_PROP(inst, odr),                                                    \
		.resolution = DT_INST_PROP(inst, resolution),                                      \
		.motion_odr = DT_INST_PROP(inst, motion_odr),                                      \
		.motion_threshold = DT_INST_PROP(inst, motion_threshold),                          \
		.motion_detection_timer = DT_INST_PROP(inst, motion_detection_timer),              \
		.tilt_odr = DT_INST_PROP(inst, tilt_odr),                                          \
		.tilt_timer = DT_INST_PROP(inst, tilt_timer),                                      \
		.tilt_angle_ll = DT_INST_PROP(inst, tilt_angle_ll),                                \
		.tilt_angle_hl = DT_INST_PROP(inst, tilt_angle_hl),                                \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int_gpios), (KX022_CFG_IRQ(inst)), ())     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, kx022_init, NULL, &dev_data_##inst, &dev_config_##inst,        \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &kx022_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(KX022_INIT)
