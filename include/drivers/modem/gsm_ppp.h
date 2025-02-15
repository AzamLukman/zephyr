/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_GSM_PPP_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_GSM_PPP_H_

#define GSM_PPP_MDM_MANUFACTURER_LENGTH  10
#define GSM_PPP_MDM_MODEL_LENGTH         16
#define GSM_PPP_MDM_REVISION_LENGTH      64
#define GSM_PPP_MDM_IMEI_LENGTH          16
#define GSM_PPP_MDM_IMSI_LENGTH          16
#define GSM_PPP_MDM_ICCID_LENGTH         32

#include <zephyr.h>
#include <time.h>

struct gsm_ppp_modem_info {
	char mdm_manufacturer[GSM_PPP_MDM_MANUFACTURER_LENGTH];
	char mdm_model[GSM_PPP_MDM_MODEL_LENGTH];
	char mdm_revision[GSM_PPP_MDM_REVISION_LENGTH];
	char mdm_imei[GSM_PPP_MDM_IMEI_LENGTH];
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	char mdm_imsi[GSM_PPP_MDM_IMSI_LENGTH];
	char mdm_iccid[GSM_PPP_MDM_ICCID_LENGTH];
#endif
	int  mdm_rssi;
};

/** @cond INTERNAL_HIDDEN */
struct device;
typedef void (*gsm_modem_power_cb)(const struct device *, void *);

void gsm_ppp_start(const struct device *dev);
void gsm_ppp_stop(const struct device *dev);
/** @endcond */

/**
 * @brief Register functions callbacks for power modem on/off.
 *
 * @param dev: gsm modem device
 * @param modem_on: callback function to
 *		execute during gsm ppp configuring.
 * @param modem_off: callback function to
 *		execute during gsm ppp stopping.
 * @param user_data: user specified data
 *
 * @retval None.
 */
void gsm_ppp_register_modem_power_callback(const struct device *dev,
					   gsm_modem_power_cb modem_on,
					   gsm_modem_power_cb modem_off,
					   void *user_data);

/**
 * @brief Get GSM modem information.
 *
 * @param dev: GSM modem device.
 *
 * @retval struct gsm_ppp_modem_info * pointer to modem information structure.
 */
const struct gsm_ppp_modem_info *gsm_ppp_modem_info(const struct device *dev);

/**
 * @brief Get the local time from the modem's real time clock.
 *
 * @param dev Pointer to the GSM modem
 * @param[inout] tm time structure
 * @param[inout] offset The amount the local time is offset from GMT/UTC in seconds.
 *
 * @retval 0 if successful
 * @retval -EIO if RTC time invalid
 */
int32_t gsm_ppp_get_local_time(const struct device *dev, struct tm *tm, int32_t *offset);

/**
 * @brief Get the modem's IMEI
 *
 * @retval Pointer to the buffer holding the IMEI
 */
char *gsm_ppp_get_imei(const struct device *dev);

/**
 * @brief Get the modem's firmware revision
 *
 * @retval Pointer to the buffer holding the firmware revision
 */
char *gsm_ppp_get_revision(const struct device *dev);

/**
 * @brief Get the modem's manufacturer
 *
 * @retval Pointer to the buffer holding the manufacturer
 */
char *gsm_ppp_get_manufacturer(const struct device *dev);

/**
 * @brief Get the modem's model
 *
 * @retval Pointer to the buffer holding the model
 */
char *gsm_ppp_get_model(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_GSM_PPP_H_ */
