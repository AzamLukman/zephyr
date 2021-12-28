#include <logging/log.h>
// LOG_MODULE_REGISTER(hdx8agh, CONFIG_SENSOR_LOG_LEVEL);

#include <drivers/sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <stdio.h>
#include <drivers/uart.h>
#include "hdx8agh-priv.h"


#define SENSOR_UART_NODE DT_INST_BUS(0)
#define SELECTION ;
#define DATA_VAL = 0x00;
#define CHECKSUM ;
// static struct hdx8agh_data hdx8agh;
static struct hdx8agh_data hdx8agh;
static uint8_t rd_data[HDX8AGH_RD_BUF_LEN] = {0};
struct k_sem tx_sem;


static const uint8_t measure_cmd[HDX8AGH_BUF_LEN] = {0x01, 0x03,0x00, 0x00, 0x00, 0x20, 0x44, 0x12};
// static const uint8_t self_cal_on_cmd[HDX8AGH_BUF_LEN] = {0xff, 0x01, 0x79, 0xA0, 0x00,
// 														0x00, 0x00, 0x00, 0xE6};

// static const uint8_t self_cal_off_cmd[HDX8AGH_BUF_LEN] = {0xff, 0x01, 0x79, 0x00, 0x00,
// 														 0x00, 0x00, 0x00, 0x86};

// static const uint8_t range_cmd1[HDX8AGH_BUF_LEN] = {0xff, 0x01, 0x99, 0x00, 0x00,
// 												   0x00, 0x07, 0xD0, 0x8F};

// static const uint8_t range_cmd2[HDX8AGH_BUF_LEN] = {0xff, 0x01, 0x99, 0x00, 0x00,
// 												   0x00, 0x13, 0x88, 0xCB};

// static const uint8_t range_cmd3[HDX8AGH_BUF_LEN] = {0xff, 0x01, 0x99, 0x00, 0x00,
// 												   0x00, 0x027, 0x10, 0x2F};

enum
{
	READ_CMD = 0,
	SELF_CALIBRATION_ON,
	SELF_CALIBRATION_OFF,
	RANGE2000,
	RANGE5000,
	RANGE10000,
};
static uint8_t cmd = READ_CMD;
uint8_t range[HDX8AGH_BUF_LEN] = {};
static int hdx8agh_channel_get(const struct device *dev, enum sensor_channel chan,
							  struct sensor_value *val)
{
	uint16_t tmp;

	// if (chan == SENSOR_CHAN_CO2)
	// {
	// 	tmp = (dev_data.CO2_HH * 256) + dev_data.CO2_LL;

	// 	val->val1 = (uint16_t)(tmp);
	// 	val->val2 = 0;
	// 	return 0;
	// }
	// else
	// {
	// 	return -ENOTSUP;
	// }
}
static int hdx8agh_checksum(uint8_t *data, size_t size)
{
	int i, chksum;
	for (i = 0; i < size; i++)
	{
		chksum += data[i];
	}
	chksum = 0xff - chksum;
	return chksum += 1;
}
static void hdx8agh_uart_flush(const struct device *dev)
{
	uint8_t c;

	while (uart_fifo_read(dev, &c, 1) > 0)
	{
		continue;
	}
}

static void hdx8agh_uart_isr(const struct device *dev, void *user_data)
{
	int rx = 0;
	static int offset = 0, rd_size = HDX8AGH_RD_BUF_LEN;
	static int i = 0;
	int j = 0;


	ARG_UNUSED(user_data);

	if (dev == NULL)
	{
		return;
	}

	/* Verify uart_irq_update() */
	if (!uart_irq_update(dev))
	{
		return;
	}

	if (uart_irq_rx_ready(dev))
	{
		rx = uart_fifo_read(dev, &rd_data[offset], rd_size);
		// printk("%drd_data :%x\r\n",offset,rd_data[offset]);
		offset += rx;

		if (rx < rd_size)
		{
			rd_size -= rx;
		}

		if (offset == HDX8AGH_RD_BUF_LEN -2)
		{
			LOG_HEXDUMP_DBG(rd_data, HDX8AGH_RD_BUF_LEN, "data");
			offset = 0;
			printk("\r\n");
			for(j=0;j<HDX8AGH_RD_BUF_LEN;j++)
			{

				printk("%x ",rd_data[j]);
			}

			printk("==========================================\r\n");
			// rd_size = HDX8AGH_BUF_LEN;
			// dev_data.CO2_HH = rd_data[2];
			// dev_data.CO2_LL = rd_data[3];
			k_sem_give(&tx_sem);
		}
	}

	if (uart_irq_tx_ready(dev))
	{
		switch (cmd)
		{
		case READ_CMD:
			uart_fifo_fill(dev, &measure_cmd[i], 1);
			break;

		// case SELF_CALIBRATION_ON:
		// 	uart_fifo_fill(dev, &self_cal_on_cmd[i], 1);
		// 	break;
		// case SELF_CALIBRATION_OFF:
		// 	uart_fifo_fill(dev, &self_cal_off_cmd[i], 1);
		// 	break;
		// case RANGE2000:
		// 	uart_fifo_fill(dev, &range_cmd1[i], 1);
		// 	break;
		// case RANGE5000:
		// 	uart_fifo_fill(dev, &range_cmd2[i], 1);
		// 	break;
		// case RANGE10000:
		// 	uart_fifo_fill(dev, &range_cmd3[i], 1);
		// 	break;
		default:
			break;
		}
		i++;
	}

	if (i == 8)
	{
		uart_irq_tx_disable(dev);
		cmd = READ_CMD;
		i = 0;
	}
}

static int hdx8agh_uart_write(void)
{
	const struct device *uart_dev = DEVICE_DT_GET(SENSOR_UART_NODE);

	uart_irq_tx_enable(uart_dev);

	return k_sem_take(&tx_sem, K_FOREVER);
}

static int hdx8agh_attr_set(const struct device *dev, enum sensor_channel chan,
						   enum sensor_attribute attr, const struct sensor_value *val)
{
	switch (attr)
	{
	case SENSOR_ATTR_FULL_SCALE:
		if (val == 2000)
		{
			cmd = RANGE2000;
		}
		else if (val == 5000)
		{
			cmd = RANGE5000;
		}
		else if (val == 10000)
		{
			cmd = RANGE10000;
		}
		else
		{
			return -ENOTSUP;
		}
		hdx8agh_uart_write();
		break;
	case SENSOR_ATTR_OFFSET:
		if (val == 0)
		{
			cmd = SELF_CALIBRATION_OFF;
		}
		else if (val == 1)
		{
			cmd = SELF_CALIBRATION_ON;
		}
		else
		{
			return -ENOTSUP;
		}
		hdx8agh_uart_write();
		break;
	default:
		return -ENOTSUP;
	}
}

static int hdx8agh_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
	uint8_t ret;

	cmd = READ_CMD;
	return hdx8agh_uart_write();
}

static const struct sensor_driver_api hdx8agh_api_funcs = {
	.attr_set = hdx8agh_attr_set,
	.sample_fetch = hdx8agh_sample_fetch,
	.channel_get = hdx8agh_channel_get,
};

static int hdx8agh_init(const struct device *dev)
{
	const struct device *uart_dev = DEVICE_DT_GET(SENSOR_UART_NODE);

	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	hdx8agh_uart_flush(uart_dev);

	uart_irq_callback_set(uart_dev, hdx8agh_uart_isr);

	uart_irq_rx_enable(uart_dev);

	return k_sem_init(&tx_sem, 0, 1);
}

DEVICE_DT_INST_DEFINE(0, hdx8agh_init, NULL, &hdx8agh, NULL, POST_KERNEL,
					  CONFIG_SENSOR_INIT_PRIORITY, &hdx8agh_api_funcs);
