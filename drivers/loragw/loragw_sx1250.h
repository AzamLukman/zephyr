

#ifndef _LORAGW_SX1250_H
#define _LORAGW_SX1250_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */
#include <zephyr.h>
#include <device.h>
#include <stdint.h>     /* C99 types*/
#include <stdbool.h>    /* bool type */

#include "sx1250_defs.h"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1250_calibrate(const struct device *dev, uint8_t rf_chain, uint32_t freq_hz);
int sx1250_setup(const struct device *dev, uint8_t rf_chain, uint32_t freq_hz, bool single_input_mode);

int sx1250_reg_w(const struct device *dev, sx1250_op_code_t op_code, uint8_t *data, uint16_t size, uint8_t rf_chain);
int sx1250_reg_r(const struct device *dev, sx1250_op_code_t op_code, uint8_t *data, uint16_t size, uint8_t rf_chain);

#endif

/* --- EOF ------------------------------------------------------------------ */
