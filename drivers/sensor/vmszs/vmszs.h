/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __VMSZS_PRIV_H__
#define __VMSZS_PRIV_H__

#include <logging/log.h>

LOG_MODULE_REGISTER(vmszs, CONFIG_SENSOR_LOG_LEVEL);
#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <stdio.h>
#include <drivers/uart.h>
#include <drivers/sensor.h>

#include <sys/byteorder.h>
#include <drivers/sensor/vmszs.h>

#ifdef CONFIG_VMSZS_KEEP_MCU_AWAKE
#include <pm/pm.h>
#endif

#define DT_DRV_COMPAT vemsee_vmszs
#define SENSOR_UART_NODE DT_INST_BUS(0)

#define VMSZS_BUF_LEN 8
#define VMSZS_RD_BUF_LEN 7

#define VMSZS_START_IDX 0


/* Arbitrary max duration to wait for the response */
#define VMSZS_WAIT K_SECONDS(1)

enum vmszs_cmd {
	VMSZS_CMD_GET_SAMPLE_DATA,
	VMSZS_CMD_MAX,
};
enum VMSZS_channel{
	VMSZS_CHANNEL_VOLTAGE_RANGE =3,
	VMSZS_CHANNEL_CURRENT =5 ,
	VMSZS_CHANNEL_WATT =7,
	VMSZS_CHANNEL_KWATT =9,
};

struct vmszs_data {
	// const struct device *comm_master;

	uint16_t data;
	bool data_valid;
	bool has_rsp;

	uint8_t rd_data[VMSZS_CMD_MAX][VMSZS_RD_BUF_LEN];

	struct k_sem tx_sem;
	struct k_sem rx_sem;

	enum vmszs_cmd cmd;
};

static const uint8_t vmszs_cmds[VMSZS_CMD_MAX][VMSZS_BUF_LEN] ={
	/*VMSZS_CMD_GET_SAMPLE_DATA_CHAN_1*/
	{0x01, 0x03,0x00, 0x00, 0x00, 0x01, 0x84, 0x0A},
	// {0x01,0x03,0x00,0x48,0x00,0x0A,0x44,0x39},
};
// static const uint8_t VMSZS_cmds[VMSZS_BUF_LEN] =
// 	/*VMSZS_CMD_GET_SAMPLE_DATA_CHAN_1*/
// 	// {0x01, 0x03,0x00, 0x48, 0x00, 0x0A, 0x45, 0xDB},
// 	{0x03,0x03,0x00,0x48,0x00,0x0A,0x44,0x39};



#endif /* __VMSZS_PRIV_H__*/
