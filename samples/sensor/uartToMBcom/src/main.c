#include <zephyr.h>
#include <sys/util.h>

static uint8_t tx_buff[256];
static uint8_t rx_buff[256];
int uart_send_cmd(uint8_t *arr)
{
	memcpy
}
static void uart_isr(const struct device *uart_dev, void *user_data)
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

		printk("xfer_byte : %d \tdata: %x\r\n",xfer_bytes,vmszs_cmds[vmszs.cmd][xfer_bytes]);
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
static uart_device_init(const struct device *device)
{
	const struct device *uart_dev = DEVICE_DT_GET(SENSOR_UART_NODE);
	int ret;

	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	vmszs_uart_flush(uart_dev);

	uart_irq_callback_set(uart_dev, uart_isr);

	k_sem_init(&d->rx_sem, 0, 1);
	k_sem_init(&d->tx_sem, 1, 1);

	return ret;
}
