/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BGT3001_BGT3001
#define ZEPHYR_DRIVERS_SENSOR_BGT3001_BGT3001

#include <logging/log.h>

LOG_MODULE_REGISTER(bgt3001, CONFIG_SENSOR_LOG_LEVEL);
#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <stdio.h>
#include <drivers/uart.h>
#include <drivers/sensor.h>

#include <sys/byteorder.h>

#define DT_DRV_COMPAT vemsee_bgt3001
#define SENSOR_UART_NODE DT_INST_BUS(0)

#define BGT3001_BUF_LEN 8
#define BGT3001_RD_BUF_LEN 11

#define BGT3001_START_IDX 0


/* Arbitrary max duration to wait for the response */
#define BGT3001_WAIT K_SECONDS(1)

enum bgt3001_cmd {
	BGT3001_CMD_GET_SAMPLE_DATA,
	BGT3001_CMD_MAX,
};

struct bgt3001_data {
	/* Max data length is 16 bits */
	// uint32_t data : 16;
	bool data_valid;
	bool has_rsp;

	int sample_n;
	int sample_p;
	int sample_k;

	uint8_t rd_data[BGT3001_CMD_MAX][BGT3001_RD_BUF_LEN];

	struct k_sem tx_sem_bgt;
	struct k_sem rx_sem_bgt;

	enum bgt3001_cmd cmd;
};





/* Table of supported MH-Z19B commands with precomputed checksum */
static const uint8_t bgt3001_cmds[BGT3001_CMD_MAX][BGT3001_BUF_LEN] = {
		{0x02, 0x03, 0x00, 0x1E, 0x00, 0x03, 0x65, 0xFE},
		//  { 0x01, 0x03, 0x00, 0x1E, 0x00, 0x03, 0x65, 0xCD },
	};


#endif /* ZEPHYR_DRIVERS_SENSOR_BGT3001_BGT3001 */
