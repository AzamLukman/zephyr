/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2020 Innoseis BV
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <device.h>
#include <devicetree.h>
#include <drivers/adc.h>
#include <logging/log.h>
#include <drivers/i2c.h>
#include <zephyr.h>
#include <sys/byteorder.h>
#include <sys/util.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER 1
#include "adc_context.h"

#define DT_DRV_COMPAT ti_ads1119

LOG_MODULE_REGISTER(ADS1119, CONFIG_ADC_LOG_LEVEL);

#define ADS1119_GEN_MASK(shift, size)  (BIT_MASK(size) << shift)

#define ADS1119_CONFIG_SHIFT_VREF 0
#define ADS1119_CONFIG_SIZE_VREF 1
#define ADS1119_CONFIG_MASK_VREF ADS1119_GEN_MASK(ADS1119_CONFIG_SHIFT_VREF, \
						  ADS1119_CONFIG_SIZE_VREF)
#define ADS1119_CONFIG_SHIFT_CM 1
#define ADS1119_CONFIG_SIZE_CM 1
#define ADS1119_CONFIG_MASK_CM ADS1119_GEN_MASK(ADS1119_CONFIG_SHIFT_CM, \
						ADS1119_CONFIG_SIZE_CM)
#define ADS1119_CONFIG_SHIFT_DR 2
#define ADS1119_CONFIG_SIZE_DR 2
#define ADS1119_CONFIG_MASK_DR ADS1119_GEN_MASK(ADS1119_CONFIG_SHIFT_DR, \
						ADS1119_CONFIG_SIZE_DR)
#define ADS1119_CONFIG_SHIFT_GAIN 4
#define ADS1119_CONFIG_SIZE_GAIN 1
#define ADS1119_CONFIG_MASK_GAIN ADS1119_GEN_MASK(ADS1119_CONFIG_SHIFT_GAIN, \
						  ADS1119_CONFIG_SIZE_GAIN)
#define ADS1119_CONFIG_SHIFT_MUX 5
#define ADS1119_CONFIG_SIZE_MUX 3
#define ADS1119_CONFIG_MASK_MUX ADS1119_GEN_MASK(ADS1119_CONFIG_SHIFT_MUX, \
						 ADS1119_CONFIG_SIZE_MUX)

#define ADS1119_CONFIG(flag, n) (ADS1119_CONFIG_MASK_##flag & (n << ADS1119_CONFIG_SHIFT_##flag))

#define ADS1119_STATUS_SHIFT_ID 0
#define ADS1119_STATUS_SIZE_ID 7
#define ADS1119_CONFIG_MASK_ID ADS1119_GEN_MASK(ADS1119_STATUS_SHIFT_ID, \
						ADS1119_STATUS_SIZE_ID)

#define ADS1119_STATUS_SHIFT_READY 7
#define ADS1119_STATUS_SIZE_READY 1
#define ADS1119_STATUS_MASK_READY ADS1119_GEN_MASK(ADS1119_STATUS_SHIFT_READY, \
						   ADS1119_STATUS_SIZE_READY)

#define ADS1119_REG_SHIFT 2

#define ADS1119_RESOLUTION 15
#define ADS1119_REF_INTERNAL 2048

enum ads1119_cmd {
	ADS1119_CMD_RESET       = 0x06,
	ADS1119_CMD_START_SYNC  = 0x08,
	ADS1119_CMD_POWER_DOWN  = 0x02,
	ADS1119_CMD_READ_DATA   = 0x10,
	ADS1119_CMD_READ_REG    = 0x20,
	ADS1119_CMD_WRITE_REG   = 0x40,
};

enum ads1119_reg {
	ADS1119_REG_CONFIG      = 0 << ADS1119_REG_SHIFT,
	ADS1119_REG_STATUS      = 1 << ADS1119_REG_SHIFT,
};

enum {
	ADS1119_CONFIG_VREF_INTERNAL    = 0,
	ADS1119_CONFIG_VREF_EXTERNAL    = 1,
};

enum {
	ADS1119_CONFIG_MUX_DIFF_0_1     = 0,
	ADS1119_CONFIG_MUX_DIFF_2_3     = 1,
	ADS1119_CONFIG_MUX_DIFF_1_2     = 2,
	ADS1119_CONFIG_MUX_SINGLE_0     = 3,
	ADS1119_CONFIG_MUX_SINGLE_1     = 4,
	ADS1119_CONFIG_MUX_SINGLE_2     = 5,
	ADS1119_CONFIG_MUX_SINGLE_3     = 6,
	ADS1119_CONFIG_MUX_SHORTED      = 7,
};

enum {
	ADS1119_CONFIG_DR_20            = 0,
	ADS1119_CONFIG_DR_90            = 1,
	ADS1119_CONFIG_DR_330           = 2,
	ADS1119_CONFIG_DR_1000          = 3,
	ADS1119_CONFIG_DR_DEFAULT       = ADS1119_CONFIG_DR_20,
};

enum {
	ADS1119_CONFIG_GAIN_1   = 0,
	ADS1119_CONFIG_GAIN_4   = 1,
};

enum {
	ADS1119_CONFIG_CM_SINGLE        = 0,
	ADS1119_CONFIG_CM_CONTINUOUS    = 1,
};

struct ads1119_config {
	const struct device *i2c_dev;
	const uint16_t i2c_dt_device_addr;
	k_thread_stack_t *stack;
};

struct ads1119_data {
	struct adc_context ctx;
	k_timeout_t ready_time;
	struct k_sem acq_sem;
	int16_t *buffer;
	int16_t *buffer_ptr;
#if CONFIG_ADC_ASYNC
	struct k_thread thread;
#endif
};

static int ads1119_read_reg(const struct device *dev, enum ads1119_reg reg_addr, uint8_t *reg_val)
{
	const struct ads1119_config *config = dev->config;

	return i2c_reg_read_byte(config->i2c_dev, config->i2c_dt_device_addr,
				 ADS1119_CMD_READ_REG | reg_addr, reg_val);
}

static int ads1119_write_reg(const struct device *dev, uint8_t reg)
{
	const struct ads1119_config *config = dev->config;

	return i2c_reg_write_byte(config->i2c_dev, config->i2c_dt_device_addr,
				  ADS1119_CMD_WRITE_REG, reg);
}

static inline int ads1119_acq_time_to_dr(const struct device *dev,
					 uint16_t acq_time)
{
	struct ads1119_data *data = dev->data;
	int odr = -EINVAL;
	uint16_t acq_value = ADC_ACQ_TIME_VALUE(acq_time);

	if (acq_time != ADC_ACQ_TIME_DEFAULT &&
	    ADC_ACQ_TIME_UNIT(acq_time) != ADC_ACQ_TIME_MILLISECONDS) {
		return -EINVAL;
	}

	switch (acq_value) {
	case ADC_ACQ_TIME_VALUE(ADC_ACQ_TIME_DEFAULT):
	case 1000 / 20:
		odr = ADS1119_CONFIG_DR_20;
		data->ready_time = K_MSEC(1000 / 20);
		break;
	case 1000 / 90:
		odr = ADS1119_CONFIG_DR_90;
		data->ready_time = K_MSEC(1000 / 90);
		break;
	case 1000 / 330:
		odr = ADS1119_CONFIG_DR_330;
		data->ready_time = K_MSEC(1000 / 330);
		break;
	case 1000 / 1000:
		odr = ADS1119_CONFIG_DR_1000;
		data->ready_time = K_MSEC(1000 / 1000);
		break;
	default:
		break;
	}

	/* As per datasheet acquisition time is a bit longer wait a bit more
	 * to ensure data ready at first try
	 */
	data->ready_time.ticks += K_USEC(10).ticks;

	return odr;
}

static int ads1119_send_start_read(const struct device *dev)
{
	const struct ads1119_config *config = dev->config;
	const uint8_t cmd = ADS1119_CMD_START_SYNC;

	return i2c_write(config->i2c_dev, &cmd, sizeof(cmd), config->i2c_dt_device_addr);
}

static int ads1119_wait_data_ready(const struct device *dev)
{
	int rc;
	struct ads1119_data *data = dev->data;

	k_sleep(data->ready_time);
	uint8_t status = 0;

	rc = ads1119_read_reg(dev, ADS1119_REG_STATUS, &status);
	if (rc != 0) {
		return rc;
	}

	while ((status & ADS1119_STATUS_MASK_READY)
	       >> ADS1119_STATUS_SHIFT_READY == 0) {

		k_sleep(K_USEC(100));
		rc = ads1119_read_reg(dev, ADS1119_REG_STATUS, &status);
		if (rc != 0) {
			return rc;
		}
	}

	if ((status & ADS1119_STATUS_MASK_READY)
	    >> ADS1119_STATUS_SHIFT_READY == 1) {
		rc = 0;
	} else {
		rc = -ETIMEDOUT;
	}

	return rc;
}

static int ads1119_read_sample(const struct device *dev, uint16_t *buff)
{
	int res;
	uint8_t rx_bytes[2];
	const struct ads1119_config *config = dev->config;
	const uint8_t cmd = ADS1119_CMD_READ_DATA;

	res = i2c_write_read(config->i2c_dev, config->i2c_dt_device_addr,
			     &cmd, sizeof(cmd),
			     rx_bytes, sizeof(rx_bytes));

	*buff = sys_get_be16(rx_bytes);
	return res;
}

static int ads1119_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	uint8_t config = 0;
	int dr = 0;

	if (channel_cfg->channel_id != 0) {
		return -ENOTSUP;
	}

	switch (channel_cfg->reference) {
	case ADC_REF_EXTERNAL0:
		config |= ADS1119_CONFIG(VREF, ADS1119_CONFIG_VREF_EXTERNAL);
		break;
	case ADC_REF_INTERNAL:
		config |= ADS1119_CONFIG(VREF, ADS1119_CONFIG_VREF_INTERNAL);
		break;
	default:
		return -ENOTSUP;
	}

	if (channel_cfg->differential) {
		if (channel_cfg->input_positive == 0 && channel_cfg->input_negative == 1) {
			config |= ADS1119_CONFIG(MUX, ADS1119_CONFIG_MUX_DIFF_0_1);
		} else if (channel_cfg->input_positive == 1 && channel_cfg->input_negative == 2) {
			config |= ADS1119_CONFIG(MUX, ADS1119_CONFIG_MUX_DIFF_1_2);
		} else if (channel_cfg->input_positive == 2 && channel_cfg->input_negative == 3) {
			config |= ADS1119_CONFIG(MUX, ADS1119_CONFIG_MUX_DIFF_2_3);
		} else {
			return -ENOTSUP;
		}
	} else {
		if (channel_cfg->input_positive == 0) {
			config |= ADS1119_CONFIG(MUX, ADS1119_CONFIG_MUX_SINGLE_0);
		} else if (channel_cfg->input_positive == 1) {
			config |= ADS1119_CONFIG(MUX, ADS1119_CONFIG_MUX_SINGLE_1);
		} else if (channel_cfg->input_positive == 2) {
			config |= ADS1119_CONFIG(MUX, ADS1119_CONFIG_MUX_SINGLE_2);
		} else if (channel_cfg->input_positive == 3) {
			config |= ADS1119_CONFIG(MUX, ADS1119_CONFIG_MUX_SINGLE_3);
		} else {
			return -ENOTSUP;
		}
	}

	dr = ads1119_acq_time_to_dr(dev, channel_cfg->acquisition_time);
	if (dr < 0) {
		return -ENOTSUP;
	}

	config |= ADS1119_CONFIG(DR, dr);

	switch (channel_cfg->gain) {
	case ADC_GAIN_1:
		config |= ADS1119_CONFIG(GAIN, ADS1119_CONFIG_GAIN_1);
		break;
	case ADC_GAIN_4:
		config |= ADS1119_CONFIG(GAIN, ADS1119_CONFIG_GAIN_4);
		break;
	default:
		return -ENOTSUP;
	}

	config |= ADS1119_CONFIG(CM, ADS1119_CONFIG_CM_SINGLE); /* Only single shot supported */

	return ads1119_write_reg(dev, config);
}


static int ads1119_validate_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed = sizeof(int16_t);

	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int ads1119_validate_sequence(const struct adc_sequence *sequence)
{
	if (sequence->resolution != ADS1119_RESOLUTION) {
		return -ENOTSUP;
	}

	if (sequence->channels != BIT(0)) {
		return -ENOTSUP;
	}

	if (sequence->oversampling) {
		return -ENOTSUP;
	}

	return ads1119_validate_buffer_size(sequence);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct ads1119_data *data = CONTAINER_OF(ctx,
						 struct ads1119_data,
						 ctx);

	if (repeat_sampling) {
		data->buffer = data->buffer_ptr;
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads1119_data *data = CONTAINER_OF(ctx,
						 struct ads1119_data, ctx);

	data->buffer_ptr = data->buffer;
	k_sem_give(&data->acq_sem);
}

static int ads1119_adc_start_read(const struct device *dev,
				  const struct adc_sequence *sequence,
				  bool wait)
{
	int rc;
	struct ads1119_data *data = dev->data;

	rc = ads1119_validate_sequence(sequence);
	if (rc != 0) {
		return rc;
	}

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	if (wait) {
		rc = adc_context_wait_for_completion(&data->ctx);
	}
	return rc;
}

static int ads1119_adc_perform_read(const struct device *dev)
{
	int rc;
	struct ads1119_data *data = dev->data;

	k_sem_take(&data->acq_sem, K_FOREVER);

	rc = ads1119_send_start_read(dev);
	if (rc) {
		adc_context_complete(&data->ctx, rc);
		return rc;
	}

	rc = ads1119_wait_data_ready(dev);
	if (rc != 0) {
		adc_context_complete(&data->ctx, rc);
		return rc;
	}

	rc = ads1119_read_sample(dev, data->buffer);
	if (rc != 0) {
		adc_context_complete(&data->ctx, rc);
		return rc;
	}
	data->buffer++;

	adc_context_on_sampling_done(&data->ctx, dev);

	return rc;
}

#if CONFIG_ADC_ASYNC
static int ads1119_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	int rc;
	struct ads1119_data *data = dev->data;

	adc_context_lock(&data->ctx, false, NULL);
	rc = ads1119_adc_start_read(dev, sequence, true);
	adc_context_release(&data->ctx, rc);

	return rc;
}
#else
static int ads1119_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	int rc;
	struct ads1119_data *data = dev->data;

	adc_context_lock(&data->ctx, false, NULL);
	rc = ads1119_adc_start_read(dev, sequence, false);


	while (rc == 0 && k_sem_count_get(&data->ctx.sync) == 0) {
		rc = ads1119_adc_perform_read(dev);
	}
	k_sem_take(&data->ctx.sync, K_FOREVER);
	adc_context_release(&data->ctx, rc);
	return rc;
}
#endif

#if CONFIG_ADC_ASYNC
static void ads1119_acquisition_thread(struct device *dev)
{
	while (true) {
		ads1119_adc_perform_read(dev);
	}
}
#endif

static int ads1119_init(const struct device *dev)
{
	int rc;
	uint8_t status;
	const struct ads1119_config *config = dev->config;
	struct ads1119_data *data = dev->data;

	adc_context_init(&data->ctx);

	k_sem_init(&data->acq_sem, 0, 1);

	if (!device_is_ready(config->i2c_dev)) {
		return -ENODEV;
	}

	rc = ads1119_read_reg(dev, ADS1119_REG_STATUS, &status);
	if (rc) {
		LOG_ERR("Could not get %s ID", dev->name);
		return rc;
	}

#if CONFIG_ADC_ASYNC
	const k_tid_t tid =
		k_thread_create(&data->thread, config->stack,
				CONFIG_ADC_ADS1119_ACQUISITION_THREAD_STACK_SIZE,
				(k_thread_entry_t)ads1119_acquisition_thread,
				(void *)dev, NULL, NULL,
				CONFIG_ADC_ADS1119_ASYNC_THREAD_INIT_PRIO,
				0, K_NO_WAIT);
	k_thread_name_set(tid, "adc_ads1119");
#endif
	adc_context_unlock_unconditionally(&data->ctx);

	return rc;
}

static const struct adc_driver_api api = {
	.channel_setup = ads1119_channel_setup,
	.read = ads1119_read,
	.ref_internal = ADS1119_REF_INTERNAL,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads1119_adc_read_async,
#endif
};
#define ADC_ADS1119_INST_DEFINE(n)								 \
	COND_CODE_1(CONFIG_ADC_ASYNC,								 \
		    (static									 \
		     K_KERNEL_STACK_DEFINE(thread_stack_##n,					 \
					   CONFIG_ADC_ADS1119_ACQUISITION_THREAD_STACK_SIZE);),	 \
		    ())										 \
	static const struct ads1119_config config_##n = {					 \
		.i2c_dev = DEVICE_DT_GET(DT_INST_BUS(n)),					 \
		.i2c_dt_device_addr = DT_INST_REG_ADDR(n),					 \
		COND_CODE_1(CONFIG_ADC_ASYNC,							 \
			    (.stack = thread_stack_##n),					 \
			    ())									 \
	};											 \
	static struct ads1119_data data_##n;							 \
	DEVICE_DEFINE(ads1119_##n, DT_INST_LABEL(n), ads1119_init,				 \
		      NULL, &data_##n, &config_##n,						 \
		      POST_KERNEL, CONFIG_ADC_ADS1119_INIT_PRIORITY,				 \
		      &api);

DT_INST_FOREACH_STATUS_OKAY(ADC_ADS1119_INST_DEFINE);
