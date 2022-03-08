#ifndef _LORAGW_SPI_H
#define _LORAGW_SPI_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>        /* C99 types*/
/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_SPI_SUCCESS     0
#define LGW_SPI_ERROR       -1

int lgw_spi_w(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t data);

int lgw_spi_r(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data);

int lgw_spi_rmw(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t length, uint8_t data);

int lgw_spi_wb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);

int lgw_spi_rb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size);

uint16_t lgw_spi_chunk_size(void);

#endif
