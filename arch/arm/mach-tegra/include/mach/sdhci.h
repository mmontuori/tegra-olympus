/*
 * include/asm-arm/arch-tegra/include/mach/sdhci.h
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __ASM_ARM_ARCH_TEGRA_SDHCI_H
#define __ASM_ARM_ARCH_TEGRA_SDHCI_H

#include <linux/mmc/host.h>
#include <asm/mach/mmc.h>

struct tegra_sdhci_platform_data {
	int cd_gpio;
	int wp_gpio;
	int power_gpio;
	int is_8bit;
	struct mmc_platform_data mmc_data;
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	unsigned long offset;	/* offset in blocks to MBR */
#endif
	char *regulator_str;	/* Voltage regulator used to control the
							   the card. */
#ifdef CONFIG_MACH_OLYMPUS
	unsigned int ocr_mask;	/* available voltages */
	int (*register_status_notify)\
		(void (*callback)(void *dev_id), void *dev_id);
	struct embedded_sdio_data *embedded_sdio;
#endif
};

#endif
