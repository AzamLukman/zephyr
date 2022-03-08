# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(loragw_spi.c)
zephyr_library_sources(loragw_com.c)
zephyr_library_sources(loragw_reg.c)
zephyr_library_sources(loragw_sx1302.c)
zephyr_library_sources(loragw_hal.c)
zephyr_library_sources(loragw_sx1302_rx.c)
zephyr_library_sources(loragw_sx1302_timestamp.c)
zephyr_library_sources(loragw_cal.c)
zephyr_library_sources(loragw_sx1250.c)
zephyr_library_sources(sx1250_com.c)
zephyr_library_sources(sx1250_spi.c)







