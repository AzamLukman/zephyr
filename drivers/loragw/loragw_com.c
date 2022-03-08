
#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */

#include "loragw_com.h"
#include "loragw_spi.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

// #define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_COM == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_COM_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_COM_ERROR;}
#endif

/* Simple write */
int lgw_com_w(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t data) {
    int com_stat;
    com_stat = lgw_spi_w(dev, spi_mux_target, address, data);
    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_com_r(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
    int com_stat;
    com_stat = lgw_spi_r(dev, spi_mux_target, address, data);
    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_rmw(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data) {
    int com_stat;
    com_stat = lgw_spi_rmw(dev, spi_mux_target, address, offs, leng, data);
    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_com_wb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    int com_stat;
    com_stat = lgw_spi_wb(dev, spi_mux_target, address, data, size);
    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_com_rb(const struct device *dev, uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    int com_stat;
    com_stat = lgw_spi_rb(dev, spi_mux_target, address, data, size);
    return com_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_set_write_mode(lgw_com_write_mode_t write_mode) {
    return LGW_COM_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_flush(void) {
    return LGW_COM_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t lgw_com_chunk_size(void) {
    return lgw_spi_chunk_size();
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_get_temperature(float * temperature) {
    *temperature = 25.0;
    return LGW_COM_SUCCESS;
}
