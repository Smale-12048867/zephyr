/*
 * Copyright (c) 2023 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dwcxgmac_phc

#include "ptp_dwc_xgmac_priv.h"

#define LOG_MODULE_NAME xgmac_ptp_clk
#define LOG_LEVEL       CONFIG_ETHERNET_LOG_LEVEL
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL);

/**
 * @brief Set the time of the PTP clock.
 *
 * @param dev PTP clock device
 * @param tm Time to set
 *
 * @return 0 if ok, <0 if error
 */
static int dwc_xgmac_ptp_clock_set(const struct device *ptp_clk_dev, struct net_ptp_time *tm)
{
	const struct phc_dwc_xgmac_config *cfg = (struct phc_dwc_xgmac_config *)ptp_clk_dev->config;
	mem_addr_t ioaddr = 0;
	uint32_t reg_val;
	int timeout = 10; /*mSec*/

	ioaddr = (mem_addr_t)DEVICE_MMIO_GET(cfg->xgmac_dev);

	reg_val = SYSTEM_TIME_SECONDS_UPDATE_TSS_SET(tm->second);
	sys_write32(reg_val, ioaddr + SYSTEM_TIME_SECONDS_UPDATE_OFST);
	reg_val = SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_SET(tm->nanosecond);
	sys_write32(reg_val, ioaddr + SYSTEM_TIME_NANOSECONDS_UPDATE_OFST);

	/* Issue command to initialize the system time value */
	reg_val = sys_read32(ioaddr + TIMESTAMP_CONTROL_OFST);
	reg_val |= TIMESTAMP_CONTROL_TSINIT_SET_MSK;
	sys_write32(reg_val, ioaddr + TIMESTAMP_CONTROL_OFST);

	/* wait for present system time initialize to complete */
	while (timeout--) {
		if (!(sys_read32(ioaddr + TIMESTAMP_CONTROL_OFST) &
		      TIMESTAMP_CONTROL_TSINIT_SET_MSK)) {
			break;
		}
		k_sleep(K_MSEC(1));
	}
	if (timeout < 0) {
		return -EBUSY;
	}
	return 0;
}

/**
 * @brief Get the time of the PTP clock.
 *
 * @param dev PTP clock device
 * @param tm Where to store the current time.
 *
 * @return 0 if ok, <0 if error
 */
static int dwc_xgmac_ptp_clock_get(const struct device *ptp_clk_dev, struct net_ptp_time *tm)
{
	const struct phc_dwc_xgmac_config *cfg = (struct phc_dwc_xgmac_config *)ptp_clk_dev->config;
	mem_addr_t ioaddr = 0;

	ioaddr = (mem_addr_t)DEVICE_MMIO_GET(cfg->xgmac_dev);

	tm->second = sys_read32(ioaddr + SYSTEM_TIME_NANOSECONDS_OFST);
	tm->nanosecond = sys_read32(ioaddr + SYSTEM_TIME_SECONDS_OFST);

	return 0;
}

/**
 * @brief Adjust the PTP clock time.
 *
 * @param dev PTP clock device
 * @param increment Increment of the clock in nanoseconds
 *
 * @return 0 if ok, <0 if error
 */
static int dwc_xgmac_ptp_clock_adjust(const struct device *ptp_clk_dev, int increment)
{
	const struct phc_dwc_xgmac_config *cfg = (struct phc_dwc_xgmac_config *)ptp_clk_dev->config;
	mem_addr_t ioaddr = 0;
	uint32_t reg_val;
	uint32_t sec;
	uint32_t nsec;
	int timeout = 10; /*timeout 10msec*/

	ioaddr = (mem_addr_t)DEVICE_MMIO_GET(cfg->xgmac_dev);
	sec = (uint32_t)(increment / 1000000000); /*1 sec = 1000000000 nsec*/
	nsec = (uint32_t)(increment % 1000000000);

	if (increment < 0) { /*-ve adjustment*/
		nsec |= SYSTEM_TIME_NANOSECONDS_UPDATE_ADDSUB_SET_MSK;
	}

	reg_val = sys_read32(ioaddr + TIMESTAMP_CONTROL_OFST);
	if (!(reg_val & TIMESTAMP_CONTROL_TSUPDT_SET_MSK)) {
		sys_write32(sec, ioaddr + SYSTEM_TIME_SECONDS_UPDATE_OFST);
		sys_write32(nsec, ioaddr + SYSTEM_TIME_NANOSECONDS_UPDATE_OFST);
	} else {
		return -EBUSY;
	}

	/* Issue command to increment the system time value */
	reg_val = sys_read32(ioaddr + TIMESTAMP_CONTROL_OFST);
	reg_val |= TIMESTAMP_CONTROL_TSUPDT_SET_MSK;
	sys_write32(reg_val, ioaddr + TIMESTAMP_CONTROL_OFST);

	while (timeout--) {
		if (!(sys_read32(ioaddr + TIMESTAMP_CONTROL_OFST) &
		      TIMESTAMP_CONTROL_TSUPDT_SET_MSK)) {
			return 0;
		}
		k_sleep(K_MSEC(1));
	}
	return -EBUSY;
}

/**
 * @brief Adjust the PTP clock time change rate when compared to its neighbor.
 *
 * @param dev PTP clock device
 * @param rate Rate of the clock time change
 *
 * @return 0 if ok, <0 if error
 */
static int dwc_xgmac_ptp_rate_adjust(const struct device *ptp_clk_dev, double rate)
{
	const struct phc_dwc_xgmac_config *cfg = (struct phc_dwc_xgmac_config *)ptp_clk_dev->config;
	mem_addr_t ioaddr = 0;
	//uint32_t reg_val;

	ioaddr = (mem_addr_t)DEVICE_MMIO_GET(cfg->xgmac_dev);

	return 0;
}

/**
 * @brief PTP HW clock device initialization function
 * @param dev Pointer to the device data
 * @retval 0 if the device initialization completed successfully
 */
static int dwc_xgmac_ptp_clock_init(const struct device *ptp_clk_dev)
{
	const struct phc_dwc_xgmac_config *cfg = (struct phc_dwc_xgmac_config *)ptp_clk_dev->config;
	mem_addr_t ioaddr = 0;
	uint32_t reg_val;
	float clk_res;
	uint8_t ssinc;  /* Sub-second Increment Value */
	uint8_t snsinc; /* Sub-nanosecond Increment Value */
	uint32_t ts_addend;
	struct net_ptp_time *tm;
	int ret = 0;

	ioaddr = (mem_addr_t)DEVICE_MMIO_GET(cfg->xgmac_dev);

	reg_val = TIMESTAMP_CONTROL_TSENALL_SET(1u) |   /*Enable timestamp for all packets*/
		  TIMESTAMP_CONTROL_TSCTRLSSR_SET(1u) | /*sub second accuracy is 1ns*/
		  TIMESTAMP_CONTROL_TSCFUPDT_SET(1u) |  /*Timestamp fine update method*/
		  TIMESTAMP_CONTROL_TSENA_SET(1u);      /*Enable timestamp*/

	sys_write32(reg_val, ioaddr + TIMESTAMP_CONTROL_OFST);

	if (cfg->ts_rolovr_type) {
		/*DIGITAL_ROLLOVER*/
		clk_res = (cfg->clk_res_ns / 0.465);
		ssinc = (uint8_t)(clk_res);
		snsinc = (uint8_t)((clk_res - ssinc) *
				   28u); /* 28 is multiplication factor given in databook*/
	} else {
		/*BINARY_ROLLOVER*/
		ssinc = (uint8_t)(cfg->clk_res_ns);
		snsinc = (uint8_t)((cfg->clk_res_ns - ssinc) *
				   28u); /* 28 is multiplication factor given in databook*/
	}

	reg_val = SUB_SECOND_INCREMENT_SNSINC_SET(snsinc) | SUB_SECOND_INCREMENT_SSINC_SET(ssinc);
	sys_write32(reg_val, ioaddr + SUB_SECOND_INCREMENT_OFST);

	/* The frequency division is the ratio of the reference clock frequency
	 * to the required clock frequency. For example, if the reference clock
	 * (clk_ptp_ref_i) is 62.5 MHz, this ratio is calculated as
	 * 62.5 MHz / 50 MHz = 1.25. Therefore, the default addend value to
	 * be set in the register is 2^32/ 1.25, 0xCCCCCCCC
	 */
	ts_addend = (2 << 32u) * (cfg->phc_req_frq / cfg->phc_ref_frq);
	reg_val = TIMESTAMP_ADDEND_TSAR_SET(ts_addend);
	sys_write32(reg_val, ioaddr + TIMESTAMP_ADDEND_OFST);

	tm->second = 0;
	tm->nanosecond = 0;
	ret = dwc_xgmac_ptp_clock_set(ptp_clk_dev, tm);

	return ret;
}

static const struct ptp_clock_driver_api dwc_xgmac_ptp_clock_api = {
	.set = dwc_xgmac_ptp_clock_set,
	.get = dwc_xgmac_ptp_clock_get,
	.adjust = dwc_xgmac_ptp_clock_adjust,
	.rate_adjust = dwc_xgmac_ptp_rate_adjust,
};

#define PTP_CLK_DWC_XGMAC_DEV_CONFIG(port)                                                         \
	static const struct phc_dwc_xgmac_config phc_dwc_xgmac##port##_dev_cfg = {                 \
		.xgmac_dev = DEVICE_DT_GET(DT_INST_PARENT(port)),                                  \
		.phc_ref_frq = DT_INST_PROP(port, ref_frq),                                    \
		.phc_req_frq = DT_INST_PROP(port, req_frq),                                    \
		.ts_rolovr_type = DT_INST_PROP(port, ts_rolovr_type),                          \
		.clk_res_ns = DT_INST_PROP(port, clk_res),                                     \
	};

/* Top-level device initialization macro - bundles all of the above */
#define PTP_CLK_DWC_XGMAC_INITIALIZE(port)                                                         \
	PTP_CLK_DWC_XGMAC_DEV_CONFIG(port)                                                         \
	DEVICE_DT_INST_DEFINE(port, &dwc_xgmac_ptp_clock_init, NULL, NULL,                         \
			      &phc_dwc_xgmac##port##_dev_cfg, POST_KERNEL,                         \
			      CONFIG_APPLICATION_INIT_PRIORITY, &dwc_xgmac_ptp_clock_api);

DT_INST_FOREACH_STATUS_OKAY(PTP_CLK_DWC_XGMAC_INITIALIZE)
