/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <logging/log.h>
#include <drivers/sensor.h>

// struct uart_data{
// 	/* Max data length is 16 bits */
// 	uint32_t data : 16;
// 	/* Command buf length is 9, fits inside 4 bits */
// 	uint32_t xfer_bytes : 4;
// 	/* Boolean */
// 	uint32_t data_valid : 1;
// 	/* Boolean */
// 	uint32_t has_rsp : 1;
// 	/* Unused bits */
// 	uint32_t unused : 10;

// 	// uint8_t rd_data[MHZ19B_BUF_LEN];
#define UART_WAIT K_SECONDS(1)
static	struct k_sem tx_sem;
static	struct k_sem rx_sem;
// };

static void uart_flush(const struct device *uart_dev)
{
	uint8_t c;

	while (uart_fifo_read(uart_dev, &c, 1) > 0) {
		continue;
	}
}

int uart_send_cmd(uint8_t *tx_buff)
{
	struct uart_data *data ;
	int ret;

	ret = k_sem_take(&tx_sem,UART_WAIT);
	if (ret) {
		return ret;
	}

	k_sem_reset(&rx_sem);

	uart_irq_tx_enable


}
