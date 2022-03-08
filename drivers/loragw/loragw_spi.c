#define DT_DRV_COMPAT ebyte_loragw
#include <zephyr.h>
#include <device.h>

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* malloc free */
#include <string.h>     /* memset */
#include <inttypes.h>
#include <math.h>

#include <drivers/spi.h>

#include "loragw_spi.h"
#include "loragw_aux.h"



#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80

#define LGW_BURST_CHUNK     1024

#define BUFF_SIZE_SPI       1024

#define SX1302_AGC_MCU_MEM  0x0000
#define SX1302_REG_COMMON   0x5600
#define SX1302_REG_AGC_MCU  0x5780

/* SPI LAYER */

struct sx1302_config {
	const char *spi_label;
	struct spi_config spi_config;
};

struct sx1302_data {
	const struct device *spi;
};

/* Simple write */
int lgw_spi_w(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t data)
{
    struct sx1302_data *devdata = dev->data;
    const struct sx1302_config *config = dev->config;

    uint8_t out_buf[4];
    uint8_t command_size;

    /* prepare frame to be sent */
  //  out_buf[0] = spi_mux_target;
	out_buf[0] = spi_mux_target;
    out_buf[1] = WRITE_ACCESS | ((address >> 8) & 0x7F);
    out_buf[2] =                (address & 0xFF);
    out_buf[3] = data;
    command_size = 4;

	const struct spi_buf tx_buf = {
		.buf = out_buf,
		.len = 4,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	spi_write(devdata->spi, &config->spi_config, &tx);


//     SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
//     digitalWrite(cs,LOW);
//     SPI.transfer(out_buf, command_size);
//     SPI.endTransaction();
//     digitalWrite(cs,HIGH);
    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_spi_r(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data)
{
	struct sx1302_data *devdata = dev->data;
	const struct sx1302_config *config = dev->config;

	uint8_t out_buf[4];
	uint8_t outer;
	uint8_t in_buf[5];
	uint8_t command_size;

    /* prepare frame to be sent */
   // out_buf[0] = spi_mux_target;
	out_buf[0] = spi_mux_target;
    out_buf[1] = READ_ACCESS | ((address >> 8) & 0x7F);
    out_buf[2] =               (address & 0xFF);
    out_buf[3] = 0x00;
//     out_buf[4] = 0x00;
    command_size = 5;

	const struct spi_buf tx_buf= {
		.buf = out_buf,
		.len = 4,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf = {
			.buf = in_buf,
			.len = 5,
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

//     SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
//     digitalWrite(cs,LOW);
//     SPI.transfer(out_buf, command_size);
//     SPI.endTransaction();
//     digitalWrite(cs,HIGH);

	spi_transceive(devdata->spi, &config->spi_config, &tx, &rx);
	// for (int i = 0; i < 1; i++){
	// 	printk("RX buf = %X\n", in_buf[i]);
	// }

	*data = in_buf[4];
	return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Single Byte Read-Modify-Write */
int lgw_spi_rmw(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t length, uint8_t data)
{
    int spi_stat = LGW_SPI_SUCCESS;
    uint8_t buf[4] = {0};

    /* Read */
    spi_stat += lgw_spi_r(dev, spi_mux_target, address, &buf[0]);

    /* Modify */
    buf[1] = ((1 << length) - 1) << offs; /* bit mask */
    buf[2] = ((uint8_t)data) << offs; /* new data offsetted */
    buf[3] = (~buf[1] & buf[0]) | (buf[1] & buf[2]); /* mixing old & new data */

    /* Write */
    spi_stat += lgw_spi_w(dev, spi_mux_target, address, buf[3]);

    return spi_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_spi_wb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size)
{
	struct sx1302_data *devdata = dev->data;
	const struct sx1302_config *config = dev->config;
	uint8_t command[3];
	uint8_t command_buffer[3];
	uint8_t command_size;
    	int size_to_do, chunk_size, offset;
	int byte_transfered = 0;
	int i;

    /* check input parameters */
//     if (size == 0) {
//         DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
//         return LGW_SPI_ERROR;
//     }

    /* prepare command byte */
    command[0] = spi_mux_target;
    command[1] = WRITE_ACCESS | ((address >> 8) & 0x7F);
    command[2] =                ((address >> 0) & 0xFF);
    command_size = 3;
    size_to_do = size;

	for (i=0; size_to_do > 0; ++i) {
        chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;
        offset = i * LGW_BURST_CHUNK;

	memcpy(command_buffer, command, command_size);

	const struct spi_buf tx_buf[2] = {
		{
		.buf = command_buffer,
		.len = command_size,
		},
		{
		.buf = data + offset,
		.len = chunk_size,
		}
	};

	struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2,
	};

	spi_write(devdata->spi, &config->spi_config, &tx);
	size_to_do -= chunk_size;
	}

	 return LGW_SPI_SUCCESS;

//     SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
//     digitalWrite(cs,LOW);
//     for (i=0; size_to_do > 0; ++i) {
//         chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;
//         offset = i * LGW_BURST_CHUNK;

//         memcpy(command_buffer,command,command_size);
//         SPI.transfer(command_buffer, command_size);
//         SPI.transfer(data + offset, chunk_size);
//         //byte_transfered += (ioctl(spi_device, SPI_IOC_MESSAGE(2), &k) - k[0].len );
//         DEBUG_PRINTF("BURST WRITE: to trans %d # chunk %d # transferred %d \n", size_to_do, chunk_size, byte_transfered);
//         size_to_do -= chunk_size; /* subtract the quantity of data already transferred */
//     }

//     SPI.endTransaction();
//     digitalWrite(cs,HIGH);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_spi_rb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size)
{
	struct sx1302_data *devdata = dev->data;
	const struct sx1302_config *config = dev->config;

    uint8_t command[4];
    uint8_t command_size;
    uint8_t command_buffer[4];
    int size_to_do, chunk_size, offset;
    int byte_transfered = 0;
    int i;

    /* check input parameters */
//     if (size == 0) {
//         DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
//         return LGW_SPI_ERROR;
//     }

    /* prepare command byte */
    command[0] = spi_mux_target;
    command[1] = READ_ACCESS | ((address >> 8) & 0x7F);
    command[2] =               ((address >> 0) & 0xFF);
    command[3] = 0x00;
    command_size = 4;
    size_to_do = size;

    for (i=0; size_to_do > 0; ++i) {
	chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;
        offset = i * LGW_BURST_CHUNK;

        memcpy(command_buffer,command,command_size);
	    	const struct spi_buf tx_buf = {
		.buf = command_buffer,
		.len = command_size,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf[2] = {
		{
		.buf = NULL,
		.len = 1,
		},
		{
		.buf = data + offset,
		.len = chunk_size,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2,
	};

	spi_transceive(devdata->spi, &config->spi_config, &tx, &rx);
        size_to_do -= chunk_size; /* subtract the quantity of data already transferred */
    }

 	return LGW_SPI_SUCCESS;
}

/* COM LAYER */
/* Simple write */
// int lgw_com_w(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t data) {
//     int com_stat;
//     com_stat = lgw_spi_w(dev, spi_mux_target, address, data);
//     return com_stat;
// }

// /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// /* Simple read */
// int lgw_com_r(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
//     int com_stat;
//     com_stat = lgw_spi_r(dev, spi_mux_target, address, data);
//     return com_stat;
// }

// /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// int lgw_com_rmw(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data) {
//     int com_stat;
//     com_stat = lgw_spi_rmw(dev, spi_mux_target, address, offs, leng, data);
//     return com_stat;
// }

// /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// /* Burst (multiple-byte) write */
// int lgw_com_wb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
//     int com_stat;
//     com_stat = lgw_spi_wb(dev, spi_mux_target, address, data, size);
//     return com_stat;
// }

// /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// /* Burst (multiple-byte) read */
// int lgw_com_rb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
//     int com_stat;
//     com_stat = lgw_spi_rb(dev, spi_mux_target, address, data, size);
//     return com_stat;
// }

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// int lgw_com_set_write_mode(lgw_com_write_mode_t write_mode) {
//     return LGW_COM_SUCCESS;
// }

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// int lgw_com_flush(void) {
//     return LGW_COM_SUCCESS;
// }

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// uint16_t lgw_com_chunk_size(void) {
//     return lgw_spi_chunk_size();
// }

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// int lgw_com_get_temperature(float * temperature) {
//     *temperature = 25.0;
//     return LGW_COM_SUCCESS;
// }

/* REG LAYER */

/* SX1302 LAYER */

// int reg_w(const struct device *dev, uint8_t spi_mux_target, struct lgw_reg_s r, int32_t reg_value) {
//     int com_stat = 0;

//     if ((r.leng == 8) && (r.offs == 0))
//     {
//         /* direct write */
//         com_stat = lgw_com_w(dev, spi_mux_target, r.addr, (uint8_t)reg_value);
//     }
// else if ((r.offs + r.leng) <= 8)
// 	{
//         /* read-modify-write */
//         com_stat = lgw_com_rmw(dev, spi_mux_target, r.addr, r.offs, r.leng, (uint8_t)reg_value);
//     }
//      else
//      {
//         /* register spanning multiple memory bytes but with an offset */
//         return -1;
//     }

//     return com_stat;
// }

// /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// int reg_r(const struct device *dev, uint8_t spi_mux_target, struct lgw_reg_s r, int32_t *reg_value)
// {
//     int com_stat = 0;
//     uint8_t bufu[4] = "\x00\x00\x00\x00";
//     int8_t *bufs = (int8_t *)bufu;

//     if ((r.offs + r.leng) <= 8) {
//         /* read one byte, then shift and mask bits to get reg value with sign extension if needed */
//         com_stat = lgw_com_r(dev, spi_mux_target, r.addr, &bufu[0]);
//         bufu[1] = bufu[0] << (8 - r.leng - r.offs); /* left-align the data */
//         if (r.sign == true) {
//             bufs[2] = bufs[1] >> (8 - r.leng); /* right align the data with sign extension (ARITHMETIC right shift) */
//             *reg_value = (int32_t)bufs[2]; /* signed pointer -> 32b sign extension */
//         } else {
//             bufu[2] = bufu[1] >> (8 - r.leng); /* right align the data, no sign extension */
//             *reg_value = (int32_t)bufu[2]; /* unsigned pointer -> no sign extension */
//         }
//     } else {
//         /* register spanning multiple memory bytes but with an offset */
//         return -1;
//     }

//     return com_stat;
// }

// int lgw_reg_w(const struct device *dev, uint16_t register_id, int32_t reg_value)
// {
//     int com_stat = LGW_COM_SUCCESS;
//     struct lgw_reg_s r;

//     /* get register struct from the struct array */
//     r = loregs[register_id];

//     com_stat = reg_w(dev, LGW_SPI_MUX_TARGET_SX1302, r, reg_value);

//     return com_stat;
// }

// int lgw_reg_r(const struct device *dev, uint16_t register_id, int32_t *reg_value)
// {
//     int com_stat = LGW_COM_SUCCESS;
//     struct lgw_reg_s r;

//     /* get register struct from the struct array */
//     r = loregs[register_id];

//     com_stat = reg_r(dev, LGW_SPI_MUX_TARGET_SX1302, r, reg_value);

// return 0;
// }

// static int lgw_check_ver(const struct device *dev)
// {
// 	int com_stat;
// 	uint8_t u =0;

// 	com_stat = lgw_com_r(dev, LGW_SPI_MUX_TARGET_SX1302, loregs[16].addr, &u);

// 	printk("Note: chip version is 0x%02X (v%u.%u)\n", u, (u >> 4) & 0x0F, u & 0x0F);

// 	return 0;
// }

// static int loragw_write_single_byte(const struct device *dev)
// {
// 	int
// }

static int loragw_init(const struct device *dev)
{
	struct sx1302_data *data = dev->data;
	const struct sx1302_config *config = dev->config;

	int x;
	uint8_t devver;
	uint64_t eui = 0;
	uint8_t test_buff = 0x11;
	uint8_t read_buff;

	data->spi = device_get_binding(config->spi_label);
		if (data->spi == NULL) {
		printk("Could not get SPI device %s", config->spi_label);
		return -ENODEV;
	}

	// x = lgw_com_r(dev, LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_COMMON + 6, &devver);
	// if (x !=0){
	// 	printk("ERROR: failed to read register, x=%d\n", x);
	// }

	// printk("SX1302 version: %X\n", devver);

	// x = sx1302_get_eui(dev, &eui);
	//   if (x != LGW_COM_SUCCESS) {
        // printk("ERROR: failed to get concentrator EUI\n");
    	// }
	// printk("EUI: %llu\n", eui);


	// x = lgw_check_ver(dev);
	// if (x != LGW_COM_SUCCESS) {
        // printk("ERROR: failed to check version\n");
    	// }


	// x = lgw_com_w(dev, LGW_SPI_MUX_TARGET_SX1302, 0x0000, 2);
	// printk("\ntest_buff = %X\n", test_buff);
	// if (x != 0){
	// 	printk("ERROR: failed to write data\n");
	// 	return -1;
	// }


	// x = lgw_com_r(dev, LGW_SPI_MUX_TARGET_SX1302, 0x0000, &read_buff);
	// printk("read_buff = %X\n", read_buff);
	// if (x != 0){
	// 	printk("ERROR: f1ailed to read data\n");
	// 	return -1;
	// }

	// if (test_buff != read_buff){
	// 	printk("read back data not same as written data, read_data = %X\n", read_buff);
	// }

	//printk("did a 1 byte R/W with no error\n");


	return 0;
}

static struct sx1302_data sx1302_data;

static const struct sx1302_config sx1302_config = {
	.spi_label = DT_INST_BUS_LABEL(0),
	.spi_config = {
		.frequency = DT_INST_PROP(0, spi_max_frequency),
		.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	}
};


DEVICE_DT_INST_DEFINE(0, loragw_init, NULL,
			&sx1302_data, &sx1302_config,
			POST_KERNEL, CONFIG_LORAGW_INIT_PRIORITY, NULL);
