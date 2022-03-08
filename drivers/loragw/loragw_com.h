#ifndef _LORAGW_COM_H
#define _LORAGW_COM_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */
#include <zephyr.h>
#include <device.h>
#include <stdint.h>   /* C99 types*/

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_COM_SUCCESS     0
#define LGW_COM_ERROR       -1

#define LGW_SPI_MUX_TARGET_SX1302   0x00
#define LGW_SPI_MUX_TARGET_RADIOA   0x01
#define LGW_SPI_MUX_TARGET_RADIOB   0x02

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum com_type_e {
    LGW_COM_SPI,
    LGW_COM_USB,
    LGW_COM_UNKNOWN
} lgw_com_type_t;

typedef enum com_write_mode_e {
    LGW_COM_WRITE_MODE_SINGLE,
    LGW_COM_WRITE_MODE_BULK,
    LGW_COM_WRITE_MODE_UNKNOWN
} lgw_com_write_mode_t;

int lgw_com_w(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t data);

int lgw_com_r(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data);

int lgw_com_rmw(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data);

int lgw_com_wb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);

int lgw_com_rb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);

int lgw_com_set_write_mode(lgw_com_write_mode_t write_mode);

int lgw_com_flush(void);

uint16_t lgw_com_chunk_size(void);

int lgw_com_get_temperature(float * temperature);

#endif
