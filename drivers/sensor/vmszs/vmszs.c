/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "vmszs.h"

static struct vmszs_data vmszs;
struct k_sem tx_sem;
static uint8_t rd_data[VMSZS_RD_BUF_LEN];

static void vmszs_uart_flush(const struct device *uart_dev)
{
	uint8_t c;

	while (uart_fifo_read(uart_dev, &c, 1) > 0) {
		continue;
	}
}

static uint8_t vmszs_checksum(const uint8_t *data, uint8_t rd_size)
{
	static uint16_t table[256] = {
		0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0,
		0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41,
		0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0,
		0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
		0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1,
		0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341,
		0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1,
		0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
		0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0,
		0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940,
		0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1,
		0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
		0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0,
		0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740,
		0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0,
		0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
		0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0,
		0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
		0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0,
		0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
		0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0,
		0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40,
		0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1,
		0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
		0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0,
		0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
	};

	uint8_t xor = 0;
	uint16_t crc = 0xFFFF;

	while (rd_size--) {
		xor = (*data++) ^ crc;
		crc >>= 8;
		crc ^= table[xor];
	}

	return crc;
}

static void vmszs_uart_isr(const struct device *uart_dev, void *user_data)
{
	static int xfer_bytes;
	int i = 0;
	static int j = 0;
	ARG_UNUSED(user_data);

	if (uart_dev == NULL) {
		return;
	}

	/* Verify uart_irq_update() */
	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {
		xfer_bytes += uart_fifo_read(uart_dev, &vmszs.rd_data[vmszs.cmd][xfer_bytes],
					     VMSZS_RD_BUF_LEN - xfer_bytes);

		if (xfer_bytes == VMSZS_RD_BUF_LEN) {
			xfer_bytes = 0;

			// for (i = 0; i < VMSZS_RD_BUF_LEN; i++) {
			// 	printk(" read buff:%i:%x\n", i, vmszs.rd_data[vmszs.cmd][i]);
			// }

			uart_irq_rx_disable(uart_dev);
			k_sem_give(&vmszs.rx_sem);
			if (vmszs.has_rsp) {
				k_sem_give(&vmszs.tx_sem);
			}
		}
		j++;
	}

	if (uart_irq_tx_ready(uart_dev)) {

		// printk("xfer_byte : %d \tdata: %x\r\n",xfer_bytes,vmszs_cmds[vmszs.cmd][xfer_bytes]);
		xfer_bytes += uart_fifo_fill(uart_dev,&vmszs_cmds[vmszs.cmd][xfer_bytes],
					     VMSZS_BUF_LEN - xfer_bytes);

		if (xfer_bytes == VMSZS_BUF_LEN) {
			xfer_bytes = 0;
			j = 0;
			uart_irq_tx_disable(uart_dev);
			if (!vmszs.has_rsp) {
				k_sem_give(&vmszs.tx_sem);
			}
		}
	}
}

static int vmszs_send_cmd(const struct device *dev, const enum vmszs_cmd cmd,
			     const bool has_rsp)
{
	struct vmszs_data *d = dev->data;
	const struct device *uart_dev = DEVICE_DT_GET(SENSOR_UART_NODE);
	int ret;
	/* Make sure last command has been transferred */
	ret = k_sem_take(&d->tx_sem, VMSZS_WAIT);
	if (ret) {
		return ret;
	}

	d->cmd = cmd;

	d->has_rsp = has_rsp;
	k_sem_reset(&d->rx_sem);

#ifdef CONFIG_VMSZS_KEEP_MCU_AWAKE
	if (has_rsp) {
		pm_constraint_set(PM_STATE_SUSPEND_TO_IDLE);
	}
#endif

	uart_irq_tx_enable(uart_dev);

	if (has_rsp) {
		uart_irq_rx_enable(uart_dev);
		ret = k_sem_take(&d->rx_sem, VMSZS_WAIT);
#ifdef CONFIG_VMSZS_KEEP_MCU_AWAKE
		pm_constraint_release(PM_STATE_SUSPEND_TO_IDLE);
#endif
	}

	return ret;
}

static inline int vmszs_send_config(const struct device *dev, const enum vmszs_cmd cmd)
{
	struct vmszs_data *d = dev->data;
	int ret;

	return ret;
}


static inline int vmszs_poll_data(const struct device *dev, const enum vmszs_cmd cmd)
{
	struct vmszs_data *d = dev->data;
	int ret;

	ret = vmszs_send_cmd(dev, cmd, true);

	return ret;
}

static inline void vmszs_convert(struct sensor_value *val, uint8_t i,
				    const enum vmszs_cmd cmd)
{	uint16_t dt;
	uint8_t d[2];
	d[0] = vmszs.rd_data[cmd][i];
	d[1] =vmszs.rd_data[cmd][i + 1];
	dt =sys_get_be16(&d);

	val->val1 = dt/10;
	val->val2 = dt%10 *100000;
}
static inline int vmszs_rx_check(const enum vmszs_cmd dat)
{
	if ((vmszs.rd_data[dat][0] != vmszs_cmds[vmszs.cmd][0]) &&
	    (vmszs.rd_data[dat][1] != vmszs_cmds[vmszs.cmd][1])) {
		return -EINVAL;
	} else {
		return 0;
	}
}
static int vmszs_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct vmszs_data *d = dev->data;
	int ret;
	uint8_t rd[2];
	// if (chan != SENSOR_CHAN_CURRENT_VMSZS_1 && chan != SENSOR_CHAN_CURRENT_VMSZS_2 &&
	//     chan != SENSOR_CHAN_CURRENT_VMSZS_3) {
	// 	return -ENOTSUP;
	// }

	// if (!d->data_valid) {
	// 	LOG_DBG("Checksum mismatch");
	// 	return -EINVAL;
	// }
	switch (chan) {
	case SENSOR_CHAN_NOISE:
		vmszs_convert(val,3,0);
	break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int vmszs_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_CURRENT) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		break;
	default:
		return -ENOTSUP;
	}
}

static int vmszs_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	struct vmszs_data *d = dev->data;
	int ret;
	uint8_t rd[2];

	if (chan != SENSOR_CHAN_CURRENT) {
		return -ENOTSUP;
	}
}

static int vmszs_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
				sensor_trigger_handler_t handler)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trig);
	ARG_UNUSED(handler);

	return -ENOTSUP;
}

static int vmszs_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;
	 ret =vmszs_poll_data(dev,VMSZS_CMD_GET_SAMPLE_DATA);

	return ret;
}

static const struct sensor_driver_api vmszs_api_funcs = {
	.attr_set = vmszs_attr_set,
	.attr_get = vmszs_attr_get,
	.trigger_set = vmszs_trigger_set,
	.sample_fetch = vmszs_sample_fetch,
	.channel_get = vmszs_channel_get,
};

static int vmszs_init(const struct device *dev)
{
	const struct device *uart_dev = DEVICE_DT_GET(SENSOR_UART_NODE);
	struct vmszs_data *d = dev->data;
	struct sensor_value cfg;
	int ret;

	memset(d, 0, sizeof(struct vmszs_data));

	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	vmszs_uart_flush(uart_dev);

	uart_irq_callback_set(uart_dev, vmszs_uart_isr);

	k_sem_init(&d->rx_sem, 0, 1);
	k_sem_init(&d->tx_sem, 1, 1);

	return ret;
}

DEVICE_DT_INST_DEFINE(0, vmszs_init, NULL, &vmszs, NULL, POST_KERNEL,
		      CONFIG_SENSOR_INIT_PRIORITY, &vmszs_api_funcs);
