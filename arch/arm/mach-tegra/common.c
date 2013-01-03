/*
 * arch/arm/mach-tegra/common.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/memblock.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/system.h>

#include <mach/gpio.h>
#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <mach/powergate.h>
#include <mach/system.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/ctype.h>

#include <mach/kbc.h>
#include <mach/nand.h>
#include <mach/sdhci.h>

#include "apbio.h"
#include "board.h"
#include "clock.h"
#include "fuse.h"
#include "pm.h"

#define MC_SECURITY_CFG2 0x7c

unsigned long tegra_bootloader_fb_start;
unsigned long tegra_bootloader_fb_size;
unsigned long tegra_fb_start;
unsigned long tegra_fb_size;
unsigned long tegra_fb2_start;
unsigned long tegra_fb2_size;
unsigned long tegra_carveout_start;
unsigned long tegra_carveout_size;
unsigned long tegra_lp0_vec_start;
unsigned long tegra_lp0_vec_size;
unsigned long tegra_vpr_start;
unsigned long tegra_vpr_size;
unsigned long tegra_grhost_aperture;

void (*arch_reset)(char mode, const char *cmd) = tegra_assert_system_reset;

void tegra_assert_system_reset(char mode, const char *cmd)
{
	void __iomem *reset = IO_ADDRESS(TEGRA_CLK_RESET_BASE + 0x04);
	u32 reg;

	/* use *_related to avoid spinlock since caches are off */
	reg = readl_relaxed(reset);
	reg |= 0x04;
	writel_relaxed(reg, reset);
}

#if defined(CONFIG_MTD_NAND_TEGRA) || defined(CONFIG_EMBEDDED_MMC_START_OFFSET)
#define MAX_MTD_PARTNR 16
static struct mtd_partition tegra_mtd_partitions[MAX_MTD_PARTNR];

struct tegra_nand_platform tegra_nand_plat = {
	.parts = tegra_mtd_partitions,
	.nr_parts = 0,
};

static int __init tegrapart_setup(char *options)
{
	char *str = options;

	if (!options || !*options)
		return 0;

	while (tegra_nand_plat.nr_parts < ARRAY_SIZE(tegra_mtd_partitions)) {
		struct mtd_partition *part;
		unsigned long long start, length, sector_sz;
		char *tmp = str;

		part = &tegra_nand_plat.parts[tegra_nand_plat.nr_parts];

		while (*tmp && !isspace(*tmp) && *tmp!=':')
			tmp++;

		if (tmp==str || *tmp!=':') {
			pr_err("%s: improperly formatted string %s\n",
			       __func__, options);
			break;
		}

		part->name = str;
		*tmp = 0;

		str = tmp+1;
		start = simple_strtoull(str, &tmp, 16);
		if (*tmp!=':')
			break;
		str = tmp+1;
		length = simple_strtoull(str, &tmp, 16);
		if (*tmp!=':')
			break;
		str = tmp+1;
		sector_sz = simple_strtoull(str, &tmp, 16);

		start *= sector_sz;
		length *= sector_sz;
		part->offset = start;
		part->size = length;

		pr_info("%s: %s at offset 0x%llx %llukB\n", __func__, part->name,
			part->offset, part->size / 1024);

		tegra_nand_plat.nr_parts++;
		str = tmp+1;

		if (*tmp!=',')
			break;
	}

	/* clean up if the last partition was parsed incorrectly */
	if (tegra_nand_plat.nr_parts < ARRAY_SIZE(tegra_mtd_partitions) &&
	    tegra_mtd_partitions[tegra_nand_plat.nr_parts].name) {
		kfree(tegra_mtd_partitions[tegra_nand_plat.nr_parts].name);
		tegra_mtd_partitions[tegra_nand_plat.nr_parts].name = NULL;
	}

	return 0;
}
__setup("tegrapart=", tegrapart_setup);
#endif

static __initdata struct tegra_clk_init_table common_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "clk_m",	NULL,		0,		true },
	{ "pll_p",	"clk_m",	216000000,	true },
	{ "pll_p_out1",	"pll_p",	28800000,	true },
	{ "pll_p_out2",	"pll_p",	48000000,	true },
	{ "pll_p_out3",	"pll_p",	72000000,	true },
	{ "pll_p_out4",	"pll_p",	108000000,	true },
	{ "pll_m_out1",	"pll_m",	120000000,	true },
	{ "sclk",	"pll_m_out1",	120000000,	true },
	{ "hclk",	"sclk",		120000000,	true },
	{ "pclk",	"hclk",		60000000,	true },
	{ "csite",	NULL,		0,		true },
	{ "emc",	NULL,		0,		true },
	{ "cpu",	NULL,		0,		true },
	{ "kfuse",	NULL,		0,		true },
	{ "pll_u",	"clk_m",	480000000,	false },
	{ "sdmmc1",	"pll_p",	48000000,	false},
	{ "sdmmc2",	"pll_p",	48000000,	false},
	{ "sdmmc3",	"pll_p",	48000000,	false},
	{ "sdmmc4",	"pll_p",	48000000,	false},
	{ NULL,		NULL,		0,		0},
};

void tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;

	writel_relaxed(0x331, p + L2X0_TAG_LATENCY_CTRL);
	writel_relaxed(0x441, p + L2X0_DATA_LATENCY_CTRL);

	l2x0_init(p, 0x6C480001, 0x8200c3fe);
#endif

}

static void __init tegra_init_power(void)
{
	tegra_powergate_power_off(TEGRA_POWERGATE_MPE);
	tegra_powergate_power_off(TEGRA_POWERGATE_3D);
}

static bool console_flushed;

static void tegra_pm_flush_console(void)
{
	if (console_flushed)
		return;
	console_flushed = true;

	printk("\n");
	pr_emerg("Restarting %s\n", linux_banner);
	if (console_trylock()) {
		console_unlock();
		return;
	}

	mdelay(50);

	local_irq_disable();
	if (!console_trylock())
		pr_emerg("tegra_restart: Console was locked! Busting\n");
	else
		pr_emerg("tegra_restart: Console was locked!\n");
	console_unlock();
}

static void tegra_pm_restart(char mode, const char *cmd)
{
	tegra_pm_flush_console();
	arm_machine_restart(mode, cmd);
}

void __init tegra_init_early(void)
{
	arm_pm_restart = tegra_pm_restart;
	tegra_init_fuse();
	tegra_gpio_resume_init();
	tegra_init_clock();
	tegra_init_pinmux();
	tegra_clk_init_from_table(common_clk_init_table);
	tegra_init_power();
	tegra_init_cache();
}

static int __init tegra_lp0_vec_arg(char *options)
{
	char *p = options;

	tegra_lp0_vec_size = memparse(p, &p);
	if (*p == '@')
		tegra_lp0_vec_start = memparse(p+1, &p);

	return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);

static int __init tegra_bootloader_fb_arg(char *options)
{
	char *p = options;

	tegra_bootloader_fb_size = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_fb_start = memparse(p+1, &p);

	pr_info("Found tegra_fbmem: %08lx@%08lx\n",
		tegra_bootloader_fb_size, tegra_bootloader_fb_start);

	return 0;
}
early_param("tegra_fbmem", tegra_bootloader_fb_arg);

static int __init tegra_vpr_arg(char *options)
{
	char *p = options;

	tegra_vpr_size = memparse(p, &p);
	if (*p == '@')
		tegra_vpr_start = memparse(p+1, &p);
	pr_info("Found vpr, start=0x%lx size=%lx",
		tegra_vpr_start, tegra_vpr_size);
	return 0;
}
early_param("vpr", tegra_vpr_arg);

/*
 * Tegra has a protected aperture that prevents access by most non-CPU
 * memory masters to addresses above the aperture value.  Enabling it
 * secures the CPU's memory from the GPU, except through the GART.
 */
void __init tegra_protected_aperture_init(unsigned long aperture)
{
#ifndef CONFIG_NVMAP_ALLOW_SYSMEM
	void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);
	pr_info("Enabling Tegra protected aperture at 0x%08lx\n", aperture);
	writel(aperture, mc_base + MC_SECURITY_CFG2);
#else
	pr_err("Tegra protected aperture disabled because nvmap is using "
		"system memory\n");
#endif
}

/*
 * Due to conflicting restrictions on the placement of the framebuffer,
 * the bootloader is likely to leave the framebuffer pointed at a location
 * in memory that is outside the grhost aperture.  This function will move
 * the framebuffer contents from a physical address that is anywher (lowmem,
 * highmem, or outside the memory map) to a physical address that is outside
 * the memory map.
 */
void tegra_move_framebuffer(unsigned long to, unsigned long from,
	unsigned long size)
{
	struct page *page;
	void __iomem *to_io;
	void *from_virt;
	unsigned long i;

	BUG_ON(PAGE_ALIGN((unsigned long)to) != (unsigned long)to);
	BUG_ON(PAGE_ALIGN(from) != from);
	BUG_ON(PAGE_ALIGN(size) != size);

	to_io = ioremap(to, size);
	if (!to_io) {
		pr_err("%s: Failed to map target framebuffer\n", __func__);
		return;
	}

	if (pfn_valid(page_to_pfn(phys_to_page(from)))) {
		for (i = 0 ; i < size; i += PAGE_SIZE) {
			page = phys_to_page(from + i);
			from_virt = kmap(page);
			memcpy_toio(to_io + i, from_virt, PAGE_SIZE);
			kunmap(page);
		}
	} else {
		void __iomem *from_io = ioremap(from, size);
		if (!from_io) {
			pr_err("%s: Failed to map source framebuffer\n",
				__func__);
			goto out;
		}

		for (i = 0; i < size; i+= 4)
			writel(readl(from_io + i), to_io + i);

		iounmap(from_io);
	}
out:
	iounmap(to_io);
}

void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
	unsigned long fb2_size)
{
	if (tegra_lp0_vec_size)
		if (memblock_reserve(tegra_lp0_vec_start, tegra_lp0_vec_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);


	tegra_carveout_start = memblock_end_of_DRAM() - carveout_size;
	if (memblock_remove(tegra_carveout_start, carveout_size))
		pr_err("Failed to remove carveout %08lx@%08lx from memory "
			"map\n",
			tegra_carveout_start, carveout_size);
	else
		tegra_carveout_size = carveout_size;

	tegra_fb2_start = memblock_end_of_DRAM() - fb2_size;
	if (memblock_remove(tegra_fb2_start, fb2_size))
		pr_err("Failed to remove second framebuffer %08lx@%08lx from "
			"memory map\n",
			tegra_fb2_start, fb2_size);
	else
		tegra_fb2_size = fb2_size;

	tegra_fb_start = memblock_end_of_DRAM() - fb_size;
	if (memblock_remove(tegra_fb_start, fb_size))
		pr_err("Failed to remove framebuffer %08lx@%08lx from memory "
			"map\n",
			tegra_fb_start, fb_size);
	else
		tegra_fb_size = fb_size;

	if (tegra_fb_size)
		tegra_grhost_aperture = tegra_fb_start;

	if (tegra_fb2_size && tegra_fb2_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_fb2_start;

	if (tegra_carveout_size && tegra_carveout_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_carveout_start;

	/*
	 * TODO: We should copy the bootloader's framebuffer to the framebuffer
	 * allocated above, and then free this one.
	 */
	if (tegra_bootloader_fb_size)
		if (memblock_reserve(tegra_bootloader_fb_start,
				tegra_bootloader_fb_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);

	pr_info("Tegra reserved memory:\n"
		"LP0:                    %08lx - %08lx\n"
		"Bootloader framebuffer: %08lx - %08lx\n"
		"Framebuffer:            %08lx - %08lx\n"
		"2nd Framebuffer:         %08lx - %08lx\n"
		"Carveout:               %08lx - %08lx\n",
		"Vpr:                    %08lx - %08lx\n",
		tegra_lp0_vec_start,
		tegra_lp0_vec_start + tegra_lp0_vec_size - 1,
		tegra_bootloader_fb_start,
		tegra_bootloader_fb_start + tegra_bootloader_fb_size - 1,
		tegra_fb_start,
		tegra_fb_start + tegra_fb_size - 1,
		tegra_fb2_start,
		tegra_fb2_start + tegra_fb2_size - 1,
		tegra_carveout_start,
		tegra_carveout_start + tegra_carveout_size - 1,
		tegra_vpr_start,
		tegra_vpr_size ?
			tegra_vpr_start + tegra_vpr_size - 1 : 0);
}
