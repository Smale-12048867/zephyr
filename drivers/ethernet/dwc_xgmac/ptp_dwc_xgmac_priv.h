/*
 * Intel Hard Processor System 10 Giga bit TSN Ethernet PTP driver
 *
 * Driver private data declarations
 *
 * Copyright (c) 2023 Intel Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ZEPHYR_DRIVERS_ETHERNET_PTP_DWC_XGMAC_PRIV_H_
#define _ZEPHYR_DRIVERS_ETHERNET_PTP_DWC_XGMAC_PRIV_H_

#include <zephyr/device.h>
#include <zephyr/drivers/ptp_clock.h>
#include "ptp_dwc_xgmac_priv.h"

#define SYSTEM_TIME_SECONDS_UPDATE_TSS_SET(value)      (((value) << 0) & 0xffffffff)
#define SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_SET(value) (((value) << 0) & 0x7fffffff)
#define TIMESTAMP_CONTROL_TSINIT_SET_MSK               0x00000004
#define SYSTEM_TIME_NANOSECONDS_UPDATE_ADDSUB_SET_MSK  0x80000000
#define TIMESTAMP_CONTROL_TSUPDT_SET_MSK               0x00000008
#define TIMESTAMP_CONTROL_TSENALL_SET(value)           (((value) << 8) & 0x00000100)
#define TIMESTAMP_CONTROL_TSCTRLSSR_SET(value)         (((value) << 9) & 0x00000200)
#define TIMESTAMP_CONTROL_TSCFUPDT_SET(value)          (((value) << 1) & 0x00000002)
#define TIMESTAMP_CONTROL_TSENA_SET(value)             (((value) << 0) & 0x00000001)
#define SUB_SECOND_INCREMENT_SNSINC_SET(value)         (((value) << 8) & 0x0000ff00)
#define SUB_SECOND_INCREMENT_SSINC_SET(value)          (((value) << 16) & 0x00ff0000)
#define TIMESTAMP_ADDEND_TSAR_SET(value)               (((value) << 0) & 0xffffffff)
#define SYSTEM_TIME_SECONDS_UPDATE_OFST                0xd10
#define SYSTEM_TIME_NANOSECONDS_UPDATE_OFST            0xd14
#define TIMESTAMP_CONTROL_OFST                         0xd00
#define SYSTEM_TIME_NANOSECONDS_OFST                   0xd0c
#define SYSTEM_TIME_SECONDS_OFST                       0xd08
#define SUB_SECOND_INCREMENT_OFST                      0xd04
#define TIMESTAMP_ADDEND_OFST                          0xd18

/**
 * @brief Constant device configuration data structure.
 *
 */
struct phc_dwc_xgmac_config {
	const struct device *xgmac_dev;
	uint32_t phc_ref_frq;
	uint32_t phc_req_frq;
	/*
	 * Timestamp rollover type
	 * 0: Digital -> Resolution 1ns
	 * 1: Binary -> Resolution ~0.465ns
	 */
	bool ts_rolovr_type;
	/*
	 * PTP clock sub second resolution in nano seconds
	 */
	float clk_res_ns;
	/**/
};

#endif /* _ZEPHYR_DRIVERS_ETHERNET_PTP_DWC_XGMAC_PRIV_H_ */