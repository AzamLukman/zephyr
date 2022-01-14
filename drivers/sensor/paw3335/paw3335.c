
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

LOG_MODULE_REGISTER(paw3335, CONFIG_SENSOR_LOG_LEVEL);

#define PAW33335_READ_REG           0
#define  PAW33335_WRITE_REG           1

struct paw33335_data{
	int16_t temp;
	uint8_t selected_range;
};
struct paw33335_config{
	struct spi_dt_spec bus;
};

static struct paw33335_data paw33335_data;

static int paw33335_reg_access(const struct device *dev, uint8_t cmd,
			      uint8_t reg_addr, void *data, size_t length)
{
	const struct paw33335_config *cfg = dev->config;
	uint8_t access[2] = { cmd, reg_addr };
	const struct spi_buf buf[2] = {
		{
			.buf = access,
			.len = 2
		}
		// ,
		// {
		// 	.buf = data,
		// 	.len = length
		// }
	};
	struct spi_buf_set tx = {
		.buffers = buf,
	};

	if (cmd == PAW33335_READ_REG) {
		printk("paw33335_reg_access\r\n");
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		tx.count = 1;

		return spi_transceive_dt(&cfg->bus, &tx, &rx);
	}

	tx.count = 2;

	return spi_write_dt(&cfg->bus, &tx);
}

static inline int paw33335_get_reg(const struct device *dev, uint8_t *read_buf,
				  uint8_t register_address, uint8_t count)
{

	return paw33335_reg_access(dev, PAW33335_READ_REG,
				  register_address, read_buf, count);
}

static int paw33335_init (const struct device *dev)
{
	const struct paw33335_config *cfg = dev->config;
	uint8_t value;

	if (!spi_is_ready(&cfg->bus)) {
		LOG_DBG("spi device not ready: %s %s", cfg->bus.bus->name,cfg->bus.bus->data);
		printk("paw33335_init \r\n");
		return -EINVAL;
	}

	printk("spi device  ready:  %s %p\r\n", cfg->bus.bus->name,cfg->bus.bus->data);
	while(1)
	{
	paw33335_get_reg(dev,&value,0x02, 1);
	printk("device id: %x\r\n",value);
	k_msleep(100);
	}

}

static const struct paw33335_config paw33335_config =
{
	.bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),
};

DEVICE_DT_INST_DEFINE(0, paw33335_init, NULL,
		    &paw33335_data, &paw33335_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, NULL);
