/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __HDX8AGH_PRIV_H__
#define __HDX8AGH_PRIV_H__

#include <logging/log.h>

LOG_MODULE_REGISTER(hdx8agh, CONFIG_SENSOR_LOG_LEVEL);
#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <stdio.h>
#include <drivers/uart.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include <drivers/sensor/hdx8agh.h>

#ifdef CONFIG_HDX8AGH_KEEP_MCU_AWAKE
#include <pm/pm.h>
#endif

#define DT_DRV_COMPAT huidkj_hdx8agh
#define SENSOR_UART_NODE DT_INST_BUS(0)

#define HDX8AGH_BUF_LEN 8
#define HDX8AGH_RD_BUF_LEN 69
#define JSYMK163_RD_BUF_LEN 25
#define HDX8AGH_START_IDX 0


/* Arbitrary max duration to wait for the response */
#define HDX8AGH_WAIT K_SECONDS(2)

enum hdx8agh_cmd {
	HDX8AGH_CMD_GET_SAMPLE_DATA_CHAN_1 ,

	HDX8AGH_CMD_GET_SAMPLE_DATA_CHAN_2,

	HDX8AGH_CMD_GET_SAMPLE_DATA_CHAN_3,

	HDX8AGH_CMD_MAX,
};
enum hdx8agh_channel{
	HDX8AGH_CHANNEL_A =19,
	HDX8AGH_CHANNEL_B =21 ,
	HDX8AGH_CHANNEL_C =23,
	HDX8AGH_CHANNEL_D =25,
	HDX8AGH_CHANNEL_E =27,
	HDX8AGH_CHANNEL_F =29,
	HDX8AGH_CHANNEL_G =31,
	HDX8AGH_CHANNEL_H =33,
};

enum jsymk163_channel{
	JSYMK163_CHANNEL_VOLTAGE_RANGE =3,
	JSYMK163_CHANNEL_CURRENT =5 ,
	JSYMK163_CHANNEL_WATT =7,
	JSYMK163_CHANNEL_KWATT =9,
};

struct hdx8agh_data {
	// const struct device *comm_master;

	uint16_t data;
	bool data_valid;
	bool has_rsp;

	uint8_t rd_data[HDX8AGH_CMD_MAX][HDX8AGH_RD_BUF_LEN];

	struct k_sem tx_sem;
	struct k_sem rx_sem;

	enum hdx8agh_cmd cmd;
};

static const uint8_t hdx8agh_cmds[HDX8AGH_CMD_MAX][HDX8AGH_BUF_LEN] ={
	/*HDX8AGH_CMD_GET_SAMPLE_DATA_CHAN_1*/
	{0x01, 0x03,0x00, 0x00, 0x00, 0x20, 0x44, 0x12},
	/*HDX8AGH_CMD_GET_SAMPLE_DATA_CHAN_2*/
	{0x02, 0x03,0x00, 0x00, 0x00, 0x20, 0x44, 0x21},
	/*HDX8AGH_CMD_GET_SAMPLE_DATA_CHAN_3*/
	{0x03, 0x03,0x00, 0x00, 0x00, 0x20, 0x44, 0x31},
};



#endif /* __HDX8AGH_PRIV_H__*/
