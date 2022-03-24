/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <modbus/modbus.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(mbc_sample, LOG_LEVEL_INF);

static int client_iface;

const static struct modbus_iface_param client_param = {
	.mode = MODBUS_MODE_RTU,
	.rx_timeout = 100000,
	.serial = {
		.baud = 9600,
		.parity = UART_CFG_PARITY_NONE,
	},
};

static int init_modbus_client(void)
{
	const char iface_name[] = {DT_PROP(DT_INST(0, zephyr_modbus_serial), label)};

	client_iface = modbus_iface_get_by_name(iface_name);

	return modbus_init_client(client_iface, client_param);
}

void main(void)
{
	uint16_t holding_reg[10];//= {'H', 'e', 'l', 'l', 'o'};
	const uint8_t coil_qty = 10;
	uint8_t coil[1] = {0};
	const int32_t sleep = 250;
	static uint8_t node = 0x01;
	int err;


	if (init_modbus_client()) {
		LOG_ERR("Modbus RTU client initialization failed");
		return;
	}

	// err = modbus_write_holding_regs(client_iface, node, 0, holding_reg,
	// 				ARRAY_SIZE(holding_reg));
	// if (err != 0) {
	// 	LOG_ERR("FC16 failed");
	// 	return;
	// }
	// while(1){
	k_msleep(1000);
	// err = modbus_read_holding_regs(client_iface, node, 0x00, holding_reg,
	// 			       ARRAY_SIZE(holding_reg));
	// if (err != 0) {
	// 	LOG_ERR("FC03 failed with %d", err);
	// 	// return 0;
	// }
	// err = modbus_read_coils(client_iface, node, 0x48, coil, coil_qty);
	// 	if (err != 0) {
	// 		LOG_ERR("FC01 failed with %d", err);
	// 		return;
	// 	}
	// }
	LOG_HEXDUMP_INF(holding_reg, sizeof(holding_reg),
			"WR|RD holding register:");
	printk("holding reh :%x\r\n",holding_reg[1]);

	err = modbus_read_holding_regs(client_iface, 0x03, 0x48, holding_reg,
				       ARRAY_SIZE(holding_reg));
	if (err != 0) {
		LOG_ERR("FC03 failed with %d", err);
		// return 0;
	}
	LOG_HEXDUMP_INF(holding_reg, sizeof(holding_reg),
			"WR|RD holding register:");
	// while (true) {
	// 	uint16_t addr = 0;

	// 	err = modbus_read_coils(client_iface, node, 0, coil, coil_qty);
	// 	if (err != 0) {
	// 		LOG_ERR("FC01 failed with %d", err);
	// 		return;
	// 	}

	// 	LOG_INF("Coils state 0x%02x", coil[0]);

	// 	err = modbus_write_coil(client_iface, node, addr++, true);
	// 	if (err != 0) {
	// 		LOG_ERR("FC05 failed with %d", err);
	// 		return;
	// 	}

	// 	k_msleep(sleep);
	// 	err = modbus_write_coil(client_iface, node, addr++, true);
	// 	if (err != 0) {
	// 		LOG_ERR("FC05 failed with %d", err);
	// 		return;
	// 	}

	// 	k_msleep(sleep);
	// 	err = modbus_write_coil(client_iface, node, addr++, true);
	// 	if (err != 0) {
	// 		LOG_ERR("FC05 failed with %d", err);
	// 		return;
	// 	}

	// 	k_msleep(sleep);
	// 	err = modbus_read_coils(client_iface, node, 0, coil, coil_qty);
	// 	if (err != 0) {
	// 		LOG_ERR("FC01 failed with %d", err);
	// 		return;
	// 	}

	// 	LOG_INF("Coils state 0x%02x", coil[0]);

	// 	coil[0] = 0;
	// 	err = modbus_write_coils(client_iface, node, 0, coil, coil_qty);
	// 	if (err != 0) {
	// 		LOG_ERR("FC15 failed with %d", err);
	// 		return;
	// 	}

	// 	k_msleep(sleep);
	// }
}
