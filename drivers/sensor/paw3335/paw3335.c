#define DT_DRV_COMPAT pixart_paw3335d

#include <kernel.h>
#include <string.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <drivers/spi.h>
#include <logging/log.h>

#include <drivers/sensor/paw3335.h>

#define PAW3335_READ	0x7F
#define PAW3335_WRITE	0x80

LOG_MODULE_REGISTER(paw3335, CONFIG_SENSOR_LOG_LEVEL);

struct paw3335_config {
	const char *spi_label;
	struct spi_config spi_config;
};

struct paw3335_data {
	const struct device *spi;
	uint8_t motion;
	uint8_t data_xl;
	uint8_t data_xh;
	uint8_t data_yl;
	uint8_t data_yh;
	int16_t sample_x;
	int16_t sample_y;
};

static int paw3335_read(const struct device *dev,
			      uint8_t reg_addr, uint8_t *sample, uint8_t length)
{
	struct paw3335_data *data = dev->data;
	const struct paw3335_config *config = dev->config;

	uint8_t access[1] = { PAW3335_READ & reg_addr };

	const struct spi_buf tx_buf = {
		.buf = access,
		.len = 1,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = sample,
			.len = length,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2,
	};

	return spi_transceive(data->spi, &config->spi_config, &tx, &rx);
}

static int paw3335_write(const struct device *dev, uint8_t reg_addr, uint8_t sample)
{
	struct paw3335_data *data = dev->data;
	const struct paw3335_config *config = dev->config;

	uint8_t access[2] = { PAW3335_WRITE | reg_addr, sample};

	const struct spi_buf tx_buf = {
		.buf = access,
		.len = 2,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	return spi_write(data->spi, &config->spi_config, &tx);
}


static int paw3335_rawdata_grab(const struct device *dev, uint8_t frame[])
{
	int i;
	uint8_t data;
	uint8_t op_mode;
	uint8_t pg_status;

	paw3335_write(dev, 0x7F, 0x00);
	paw3335_write(dev, 0x40, 0x80);
	paw3335_write(dev, 0x7F, 0x13);
	paw3335_write(dev, 0x47, 0x30);
	paw3335_write(dev, 0x7F, 0x00);
	paw3335_write(dev, 0x55, 0x04);

	do {
		paw3335_read(dev, 0x02, &data, 1);
		op_mode = data << 6;
	} while (op_mode != 0x00);

	paw3335_write(dev, 0x58, 0xFF);

	do {
		paw3335_read(dev, 0x59, &data, 1);
		pg_status = data & 0x40;
	} while (pg_status != 0x40);

	for (i = 0; i < 900; i++){
		do {
			paw3335_read(dev, 0x59, &data, 1);
			pg_status = data & 0x80;
		} while (pg_status != 0x80);

		paw3335_read(dev, 0x58, &data, 1);
		frame[i] = data & 0x7F;
	}

	paw3335_write(dev, 0x55, 0x00);
	paw3335_write(dev, 0x40, 0x00);
	paw3335_write(dev, 0x7F, 0x13);
	paw3335_write(dev, 0x47, 0x20);
	paw3335_write(dev, 0x7F, 0x00);

	return 0;
}

static int paw3335_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct paw3335_data *data = dev->data;
	uint8_t buffer[12];

	paw3335_read(dev, 0x16, buffer, 12);
	k_usleep(100);
	k_usleep(1);
	data->motion = buffer[0];
	data->data_xl = buffer[2];
	data->data_xh = buffer[3];
	data->data_yl = buffer[4];
	data->data_yh = buffer[5];

	return 0;
}

static int paw3335_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct paw3335_data *data = dev->data;

	data->sample_x = (data->data_xh<<8) | (data->data_xl);
	data->sample_y = (data->data_yh<<8) | (data->data_yl);

	val->val1 = data->motion;
	(val+1)->val1 = data->sample_x;
	(val+2)->val1 = data->sample_y;

	return 0;
}

static int paw3335_set_resolution(const struct device *dev, uint8_t reso)
{
	//add error checking of reso
	return paw3335_write(dev, 0x4E, reso);
}

static int paw3335_set_axis(const struct device *dev, uint8_t mask)
{
	uint8_t axis;

	paw3335_read(dev, 0x5B, &axis, 1);
	axis = axis | mask;

	return paw3335_write(dev, 0x5B, axis);
}

static int paw3335_set_config(const struct device *dev, enum sensor_attribute cfg,
				const struct sensor_value *val)
{
	switch ((enum sensor_config_paw3335) cfg){
		case SENSOR_CFG_AXIS_SWAP_XY:
		return paw3335_set_axis(dev, 0x80);

		case SENSOR_CFG_AXIS_INV_Y:
		return paw3335_set_axis(dev, 0x40);

		case SENSOR_CFG_AXIS_INV_X:
		return paw3335_set_axis(dev, 0x20);

		case SENSOR_CFG_RESOLUTION:
		return paw3335_set_resolution(dev, val->val1);

		default:
		LOG_DBG("Config not supported.");
		return -ENOTSUP;
	}
}

static int paw3335_power_up_init(const struct device *dev)
{
	struct paw3335_data *data = dev->data;

	uint8_t var_a;
	uint8_t var_b;
	uint8_t reg;
	uint8_t sample;

	k_msleep(50);

	paw3335_write(dev, 0x3A, 0x5A);
	k_msleep(5);

	paw3335_write(dev, 0x40, 0x80);
	paw3335_write(dev, 0x55, 0x01);

	k_msleep(1);

	paw3335_write(dev, 0x7F, 0x0E);
	paw3335_write(dev, 0x43, 0x1D);
	paw3335_read(dev, 0x46, &var_a, 1);
	paw3335_write(dev, 0x43, 0x1E);
	paw3335_read(dev, 0x46, &var_b, 1);
	paw3335_write(dev, 0x7F, 0x14);
	paw3335_write(dev, 0x6A, var_a);
	paw3335_write(dev, 0x6C, var_b);
	paw3335_write(dev, 0x7F, 0x00);
	paw3335_write(dev, 0x55, 0x00);
	paw3335_write(dev, 0x4E, 0x23);
	paw3335_write(dev, 0x77, 0x18);
	paw3335_write(dev, 0x7F, 0x05);
	paw3335_write(dev, 0x53, 0x0C);
	paw3335_write(dev, 0x5B, 0xEA);
	paw3335_write(dev, 0x61, 0x13);
	paw3335_write(dev, 0x62, 0x0B);
	paw3335_write(dev, 0x64, 0xD8);
	paw3335_write(dev, 0x6D, 0x86);
	paw3335_write(dev, 0x7D, 0x84);
	paw3335_write(dev, 0x7E, 0x00);
	paw3335_write(dev, 0x7F, 0x06);
	paw3335_write(dev, 0x60, 0xB0);
	paw3335_write(dev, 0x61, 0x00);
	paw3335_write(dev, 0x7E, 0x40);
	paw3335_write(dev, 0x7F, 0x0A);
	paw3335_write(dev, 0x4A, 0x23);
	paw3335_write(dev, 0x4C, 0x28);
	paw3335_write(dev, 0x49, 0x00);
	paw3335_write(dev, 0x4F, 0x02);
	paw3335_write(dev, 0x7F, 0x07);
	paw3335_write(dev, 0x42, 0x16);
	paw3335_write(dev, 0x7F, 0x09);
	paw3335_write(dev, 0x40, 0x03);
	paw3335_write(dev, 0x7F, 0x0C);
	paw3335_write(dev, 0x54, 0x00);
	paw3335_write(dev, 0x44, 0x44);
	paw3335_write(dev, 0x56, 0x40);
	paw3335_write(dev, 0x42, 0x0C);
	paw3335_write(dev, 0x43, 0xA8);
	paw3335_write(dev, 0x4E, 0x8B);
	paw3335_write(dev, 0x59, 0x63);
	paw3335_write(dev, 0x7F, 0x0D);
	paw3335_write(dev, 0x5E, 0xC3);
	paw3335_write(dev, 0x4F, 0x02);
	paw3335_write(dev, 0x7F, 0x14);
	paw3335_write(dev, 0x4A, 0x67);
	paw3335_write(dev, 0x6D, 0x82);
	paw3335_write(dev, 0x73, 0x83);
	paw3335_write(dev, 0x74, 0x00);
	paw3335_write(dev, 0x7A, 0x16);
	paw3335_write(dev, 0x63, 0x14);
	paw3335_write(dev, 0x62, 0x14);
	paw3335_write(dev, 0x7F, 0x10);
	paw3335_write(dev, 0x48, 0x0F);
	paw3335_write(dev, 0x49, 0x88);
	paw3335_write(dev, 0x4C, 0x1D);
	paw3335_write(dev, 0x4F, 0x08);
	paw3335_write(dev, 0x51, 0x6F);
	paw3335_write(dev, 0x52, 0x90);
	paw3335_write(dev, 0x54, 0x64);
	paw3335_write(dev, 0x55, 0xF0);
	paw3335_write(dev, 0x5C, 0x40);
	paw3335_write(dev, 0x61, 0xEE);
	paw3335_write(dev, 0x62, 0xE5);
	paw3335_write(dev, 0x7F, 0x00);
	paw3335_write(dev, 0x5B, 0x40);
	paw3335_write(dev, 0x61, 0xAD);
	paw3335_write(dev, 0x51, 0xEA);
	paw3335_write(dev, 0x19, 0x9F);
	paw3335_write(dev, 0x5B, 0x60);

	for (int i = 0; i < 55; i++)
	{
		paw3335_read(dev, 0x20, &reg, 1);
		if (reg == 0x0F){
			break;
		}
		k_msleep(1);
	}

	k_msleep(1);

	paw3335_write(dev, 0x19, 0x10);
	paw3335_write(dev, 0x61, 0xD5);
	paw3335_write(dev, 0x40, 0x00);
	paw3335_write(dev, 0x7F, 0x00);
	paw3335_write(dev, 0x77, 0x24);
	paw3335_write(dev, 0x7F, 0x0D);
	paw3335_write(dev, 0x4E, 0x6B);
	paw3335_write(dev, 0x7F, 0x05);
	paw3335_write(dev, 0x44, 0xA8);
	paw3335_write(dev, 0x4A, 0x14);
	paw3335_write(dev, 0x7F, 0x00);
	paw3335_write(dev, 0x4F, 0x46);
	paw3335_write(dev, 0x4D, 0xD0);


	paw3335_read(dev, 0x02, &sample, 1);
	paw3335_read(dev, 0x03, &data->data_xl, 1);
	paw3335_read(dev, 0x04, &data->data_xh, 1);
	paw3335_read(dev, 0x05, &data->data_yl, 1);
	paw3335_read(dev, 0x06, &data->data_yh, 1);

		paw3335_read(dev, 0x00, &sample, 1);
	printk("dev id = %x\n", sample);

	return 0;
}
static int paw3335_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute cfg,
				const struct sensor_value *val)
{
	return paw3335_set_config(dev, cfg, val);

}
static const struct sensor_driver_api paw3335_api = {
	.attr_set = &paw3335_attr_set,
	.sample_fetch = &paw3335_sample_fetch,
	.channel_get = &paw3335_channel_get,
};

static int paw3335_init(const struct device *dev)
{
	struct paw3335_data *data = dev->data;
	const struct paw3335_config *config = dev->config;

	data->spi = device_get_binding(config->spi_label);
		if (data->spi == NULL) {
		LOG_ERR("Could not get SPI device %s", config->spi_label);
		return -ENODEV;
	}
		uint8_t sample;






	return paw3335_power_up_init(dev);
}

#define PAW3335_INIT(inst)																							\
																													\
	static struct paw3335_data paw3335_data_##inst;																	\
																													\
	static const struct paw3335_config paw3335_config_##inst = {													\
		.spi_label = DT_INST_BUS_LABEL(inst),																		\
		.spi_config = {																								\
			.frequency = DT_INST_PROP(inst, spi_max_frequency),														\
			.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB,	\
		}																											\
	};																												\
																													\
	DEVICE_DT_INST_DEFINE(inst, &paw3335_init, NULL,																\
						&paw3335_data_##inst, &paw3335_config_##inst,												\
						POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,													\
						&paw3335_api);																				\

DT_INST_FOREACH_STATUS_OKAY(PAW3335_INIT)
