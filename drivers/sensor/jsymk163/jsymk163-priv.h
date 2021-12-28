/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __JSYMK163_PRIV_H__
#define __JSYMK163_PRIV_H__

#include <logging/log.h>

LOG_MODULE_REGISTER(jsymk163, CONFIG_SENSOR_LOG_LEVEL);
#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <stdio.h>
#include <drivers/uart.h>
#include <drivers/sensor.h>

#include <sys/byteorder.h>
#include <drivers/sensor/jsymk163.h>

#ifdef CONFIG_JSYMK163_KEEP_MCU_AWAKE
#include <pm/pm.h>
#endif

#define DT_DRV_COMPAT jsy_mk163
#define SENSOR_UART_NODE DT_INST_BUS(0)

#define JSYMK163_BUF_LEN 8
#define JSYMK163_RD_BUF_LEN 25

#define JSYMK163_START_IDX 0


/* Arbitrary max duration to wait for the response */
#define JSYMK163_WAIT K_SECONDS(1)

enum jsymk163_cmd {
	JSYMK163_CMD_GET_SAMPLE_DATA_CHAN_1 ,

	JSYMK163_CMD_GET_SAMPLE_DATA_CHAN_2,

	JSYMK163_CMD_GET_SAMPLE_DATA_CHAN_3,

	JSYMK163_CMD_MAX,
};
enum jsymk163_channel{
	JSYMK163_CHANNEL_VOLTAGE_RANGE =3,
	JSYMK163_CHANNEL_CURRENT =5 ,
	JSYMK163_CHANNEL_WATT =7,
	JSYMK163_CHANNEL_KWATT =9,
};

struct jsymk163_data {
	// const struct device *comm_master;

	uint16_t data;
	bool data_valid;
	bool has_rsp;

	uint8_t rd_data[JSYMK163_CMD_MAX][JSYMK163_RD_BUF_LEN];

	struct k_sem tx_sem;
	struct k_sem rx_sem;

	enum jsymk163_cmd cmd;
};

static const uint8_t jsymk163_cmds[JSYMK163_CMD_MAX][JSYMK163_BUF_LEN] ={
	/*JSYMK163_CMD_GET_SAMPLE_DATA_CHAN_1*/
	{0x01, 0x03,0x00, 0x48, 0x00, 0x0A, 0x45, 0xDB},
	// {0x01,0x03,0x00,0x48,0x00,0x0A,0x44,0x39},
	/*JSYMK163_CMD_GET_SAMPLE_DATA_CHAN_2*/
	{0x02, 0x03,0x00, 0x48, 0x00, 0x0A, 0x45, 0x21},
	/*JSYMK163_CMD_GET_SAMPLE_DATA_CHAN_3*/
	{0x03,0x03,0x00,0x48,0x00,0x0A,0x44,0x39},
};
// static const uint8_t jsymk163_cmds[JSYMK163_BUF_LEN] =
// 	/*JSYMK163_CMD_GET_SAMPLE_DATA_CHAN_1*/
// 	// {0x01, 0x03,0x00, 0x48, 0x00, 0x0A, 0x45, 0xDB},
// 	{0x03,0x03,0x00,0x48,0x00,0x0A,0x44,0x39};



#endif /* __JSYMK163_PRIV_H__*/
