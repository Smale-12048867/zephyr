/*
 * Copyright (c) 2025 Altera Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT mrvl_88e151x

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mrvl_88e151x, CONFIG_PHY_LOG_LEVEL);

/* Page switch register */
#define MRVL_88E151x_MII_PAGE_SWITCH	22
/* Page select mask */
#define PAGE_ADDRESS(x)					(x & 0xFF)
/* MAC_REG_PAGE_0*/
#define MAC_REG_PAGE_0					PAGE_ADDRESS(0)
/* MAC_REG_PAGE_2*/
#define MAC_REG_PAGE_2					PAGE_ADDRESS(2)
/* MAC Specific Control Register 2 */
#define MRVL_88E151x_MII_MAC_CTRL2      21
/* Transmit clock internally delayed */
#define RGMII_TX_INTERNAL_DELAY_DIS_MSK ~(1 << 4)
/* Transmit clock internally delayed */
#define RGMII_RX_INTERNAL_DELAY_DIS_MSK ~(1 << 5)

struct mrvl_88e151x_phy_config {
	bool tx_delay_en;
	bool rx_delay_en;
	const struct device *const phy;
};

static int disable_tx_delay(const struct device *dev)
{
	const struct mrvl_88e151x_phy_config *const cfg = (const struct mrvl_88e151x_phy_config *const)dev->config;
	uint32_t mac_ctrl2_reg;

	if (!cfg->phy) {
		return -ENODEV;
	}

	if(phy_write(cfg->phy, MRVL_88E151x_MII_PAGE_SWITCH, MAC_REG_PAGE_2)) {
		return -EIO;
	}

	if (phy_read(cfg->phy, MRVL_88E151x_MII_MAC_CTRL2, &mac_ctrl2_reg) < 0) {
		return -EIO;
	}

	mac_ctrl2_reg &= RGMII_TX_INTERNAL_DELAY_DIS_MSK;

	if(phy_write(cfg->phy, MRVL_88E151x_MII_MAC_CTRL2, mac_ctrl2_reg)) {
		return -EIO;
	}

	if(phy_write(cfg->phy, MRVL_88E151x_MII_PAGE_SWITCH, MAC_REG_PAGE_0)) {
		return -EIO;
	}

	return 0;
}

static int disable_rx_delay(const struct device *dev)
{
	const struct mrvl_88e151x_phy_config *const cfg = dev->config;
	uint32_t mac_ctrl2_reg;

	if (!cfg->phy) {
		return -ENODEV;
	}

	if(phy_write(cfg->phy, MRVL_88E151x_MII_PAGE_SWITCH, MAC_REG_PAGE_2)) {
		return -EIO;
	}

	if (phy_read(cfg->phy, RGMII_RX_INTERNAL_DELAY_DIS_MSK, &mac_ctrl2_reg) < 0) {
		return -EIO;
	}

	mac_ctrl2_reg &= RGMII_TX_INTERNAL_DELAY_DIS_MSK;

	if(phy_write(cfg->phy, MRVL_88E151x_MII_MAC_CTRL2, mac_ctrl2_reg)) {
		return -EIO;
	}

	if(phy_write(cfg->phy, MRVL_88E151x_MII_PAGE_SWITCH, MAC_REG_PAGE_0)) {
		return -EIO;
	}

	return 0;
}

static int mrvl_88e151x_phy_vendor_init(const struct device *dev)
{
	const struct mrvl_88e151x_phy_config *const cfg = dev->config;
	int ret = 0;

	if (!cfg->tx_delay_en)
		ret = disable_tx_delay(dev);

	if (!cfg->rx_delay_en)
		ret = disable_rx_delay(dev);

	return ret;
}

#define PHY_MII_VENDOR_CONFIG(n)                                                                   \
	static const struct mrvl_88e151x_phy_config mrvl_88e151x_phy_config_##n = {                \
		.tx_delay_en = DT_INST_PROP(n, tx_delay_en),                                       \
		.rx_delay_en = DT_INST_PROP(n, rx_delay_en),                                       \
		.phy = UTIL_AND(UTIL_NOT(IS_FIXED_LINK(n)), DEVICE_DT_GET(DT_INST_BUS(n)))};

#define PHY_MII_VENDOR_DEVICE(n)                                                                   \
	PHY_MII_VENDOR_CONFIG(n);                                                                  \
	DEVICE_DT_INST_DEFINE(n, &mrvl_88e151x_phy_vendor_init, NULL, NULL,                        \
			      &mrvl_88e151x_phy_config_##n, POST_KERNEL, CONFIG_PHY_INIT_PRIORITY, \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(PHY_MII_VENDOR_DEVICE)
