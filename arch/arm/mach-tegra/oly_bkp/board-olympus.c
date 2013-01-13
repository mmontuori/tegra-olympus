/*
 * arch/arm/mach-tegra/board-olympus.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h>
#include <linux/pda_power.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>

#include <asm/bootinfo.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/nand.h>
#include <asm/setup.h>

#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/gpio.h>
#include <mach/clk.h>
#include <mach/spi.h>

#include <linux/usb/android_composite.h>

#include "board.h"
#include "board-olympus.h"
#include "clock.h"
#include "gpio-names.h"
#include "devices.h"

/*
 * Debugging
 */

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTB_BASE),
		.mapbase	= TEGRA_UARTB_BASE,
		.irq		= INT_UARTB,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 0, /* filled in by tegra_olympus_init */
	}, {
		.flags		= 0
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

static struct resource ram_console_resource[] = {
	{
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};

/* OTG gadget device */
static u64 tegra_otg_dmamask = DMA_BIT_MASK(32);


static struct resource tegra_otg_resources[] = {
	[0] = {
		.start  = TEGRA_USB_BASE,
		.end    = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = INT_USB,
		.end    = INT_USB,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct fsl_usb2_platform_data tegra_otg_pdata = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_UTMI,
};

static struct platform_device tegra_otg = {
	.name = "fsl-tegra-udc",
	.id   = -1,
	.dev  = {
		.dma_mask		= &tegra_otg_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data = &tegra_otg_pdata,
	},
	.resource = tegra_otg_resources,
	.num_resources = ARRAY_SIZE(tegra_otg_resources),
};

static char *usb_functions[] = { "usb_mass_storage" };
static char *usb_functions_adb[] = { "usb_mass_storage", "adb" };

static struct android_usb_product usb_products[] = {
	{
		.product_id     = 0xDEAD,
		.num_functions  = ARRAY_SIZE(usb_functions),
		.functions      = usb_functions,
	},
	{
		.product_id     = 0xBEEF,
		.num_functions  = ARRAY_SIZE(usb_functions_adb),
		.functions      = usb_functions_adb,
	},
};

/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id                      = 0x18d1,
	.product_id                     = 0x0002,
	.manufacturer_name      = "Google",
	.product_name           = "Olympus!",
	.serial_number          = "0000",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_adb),
	.functions = usb_functions_adb,
};


static struct platform_device androidusb_device = {
	.name   = "android_usb",
	.id     = -1,
	.dev    = {
		.platform_data  = &andusb_plat,
	},
};

/* PDA power */
static struct pda_power_pdata pda_power_pdata = {
};

static struct platform_device pda_power_device = {
	.name   = "pda_power",
	.id     = -1,
	.dev    = {
		.platform_data  = &pda_power_pdata,
	},
};


static struct resource tegra_gart_resources[] = {
    {
        .name = "mc",
        .flags = IORESOURCE_MEM,
	.start = TEGRA_MC_BASE,
	.end = TEGRA_MC_BASE + TEGRA_MC_SIZE - 1,
    },
    {
        .name = "gart",
        .flags = IORESOURCE_MEM,
	.start = 0x58000000,
	.end = 0x58000000 - 1 + 32 * 1024 * 1024,
    }
};


static struct platform_device tegra_gart_dev = {
    .name = "tegra_gart",
    .id = -1,
    .num_resources = ARRAY_SIZE(tegra_gart_resources),
    .resource = tegra_gart_resources
};

static struct resource tegra_grhost_resources[] = {
	{
		.start = TEGRA_HOST1X_BASE,
		.end = TEGRA_HOST1X_BASE + TEGRA_HOST1X_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = TEGRA_DISPLAY_BASE,
		.end = TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = TEGRA_DISPLAY2_BASE,
		.end = TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = TEGRA_VI_BASE,
		.end = TEGRA_VI_BASE + TEGRA_VI_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = TEGRA_ISP_BASE,
		.end = TEGRA_ISP_BASE + TEGRA_ISP_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = TEGRA_MPE_BASE,
		.end = TEGRA_MPE_BASE + TEGRA_MPE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_SYNCPT_THRESH_BASE,
 		.end = INT_SYNCPT_THRESH_BASE + INT_SYNCPT_THRESH_NR - 1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_HOST1X_MPCORE_GENERAL,
		.end = INT_HOST1X_MPCORE_GENERAL,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_grhost_dev = {
	.name = "tegra_grhost",
	.id = -1,
	.resource = tegra_grhost_resources,
	.num_resources = ARRAY_SIZE(tegra_grhost_resources),
};

static struct platform_device *olympus_devices[] __initdata = {
	&debug_uart,
	&tegra_otg,
	&androidusb_device,
	&pda_power_device,
	&tegra_uartd_device,
	&tegra_i2c_device1,
	&tegra_i2c_device2,
	&tegra_i2c_device3,
	&tegra_i2c_device4,
	&tegra_spi_device1,
	&tegra_spi_device2,
	&tegra_spi_device3,
	&tegra_spi_device4,
	&tegra_gart_dev,
	&tegra_grhost_dev,
	&ram_console_device,
};
/* 
 * SDHCI init
 */

static u64 tegra_dma_mask = DMA_BIT_MASK(32);

extern struct tegra_nand_platform tegra_nand_plat;

static struct tegra_sdhci_platform_data olympus_wifi_data = { /* SDHCI 1 */
		.mmc_data = {
			.built_in = 1,
		},
		.wp_gpio = -1,
		.cd_gpio = -1,
		.power_gpio = -1,
/*		.max_clk_limit = 50000000,*/
};

/*
static struct tegra_sdhci_platform_data olympus_sdhci_platform_data4 = {
	.cd_gpio = TEGRA_GPIO_PH2,
	.wp_gpio = TEGRA_GPIO_PH3,
	.power_gpio = TEGRA_GPIO_PI6,
};
*/

static struct tegra_sdhci_platform_data olympus_sdhci_platform_data3 = {
		.mmc_data = {
			.built_in = 0,
			.card_present = 0,
		},
		.wp_gpio = -1,
		.cd_gpio = 69,
		.power_gpio = -1,
/*		.max_clk_limit = 50000000,*/
};

static struct tegra_sdhci_platform_data olympus_sdhci_platform_data4 = {
		.mmc_data = {
			.built_in = 1,
		},
		.wp_gpio = -1,
		.cd_gpio = -1,
		.power_gpio = -1,
		.is_8bit = 1,
/*		.max_clk_limit = 50000000,*/
};

static struct resource tegra_sdhci_resources[][2] = {
	[0] = {
		[0] = {
			.start = TEGRA_SDMMC1_BASE,
			.end = TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC1,
			.end = INT_SDMMC1,
			.flags = IORESOURCE_IRQ,
		},
	},
	[1] = {
		[0] = {
			.start = TEGRA_SDMMC2_BASE,
			.end = TEGRA_SDMMC2_BASE + TEGRA_SDMMC2_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC2,
			.end = INT_SDMMC2,
			.flags = IORESOURCE_IRQ,
		},
	},
	[2] = {
		[0] = {
			.start = TEGRA_SDMMC3_BASE,
			.end = TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC3,
			.end = INT_SDMMC3,
			.flags = IORESOURCE_IRQ,
		},
	},
	[3] = {
		[0] = {
			.start = TEGRA_SDMMC4_BASE,
			.end = TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = INT_SDMMC4,
			.end = INT_SDMMC4,
			.flags = IORESOURCE_IRQ,
		},
	},
};

struct platform_device tegra_sdhci_devices[] = {
	[0] = {
		.id = 0,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[0],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[0]),
		.dev = {
			.platform_data = &olympus_wifi_data,
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	[1] = {},
	[2] = {
		.id = 2,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[2],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[2]),
		.dev = {
			.platform_data = &olympus_sdhci_platform_data3,
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
	[3] = {
		.id = 3,
		.name = "tegra-sdhci",
		.resource = tegra_sdhci_resources[3],
		.num_resources = ARRAY_SIZE(tegra_sdhci_resources[3]),
		.dev = {
			.platform_data = &olympus_sdhci_platform_data4,
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &tegra_dma_mask,
		},
	},
};

static const char tegra_sdio_ext_reg_str[] = "vsdio_ext";
int tegra_sdhci_boot_device = -1;

static void olympus_sdhci_init(void)
{
		int i;
#if 0
	/* TODO: setup GPIOs for cd, wd, and power */
	tegra_sdhci_device1.dev.platform_data = &olympus_wifi_data;
	tegra_sdhci_device3.dev.platform_data = &olympus_sdhci_platform_data3;
	tegra_sdhci_device4.dev.platform_data = &olympus_sdhci_platform_data4;
#endif
	printk(KERN_INFO "pICS_%s: Starting...",__func__);
	/* check if an "MBR" partition was parsed from the tegra partition
		 * command line, and store it in sdhci.3's offset field */
	for (i=0; i<tegra_nand_plat.nr_parts; i++) {
		if (strcmp("mbr", tegra_nand_plat.parts[i].name))
			continue;
		olympus_sdhci_platform_data4.offset = tegra_nand_plat.parts[i].offset;
		printk(KERN_INFO "pICS_%s: tegra_sdhci_boot_device plat->offset = 0x%llx ",__func__, tegra_nand_plat.parts[i].offset);	
		}

	platform_device_register(&tegra_sdhci_devices[3]);
	platform_device_register(&tegra_sdhci_devices[0]);
	platform_device_register(&tegra_sdhci_devices[2]);

/*	platform_device_register(&tegra_sdhci_device1);
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device4); */
}

static struct tegra_spi_platform_data tegra_spi_platform[] = {
	[0] = {
	},
	[1] = {
	},
	[2] = {
	},
	[3] = {
	},
	[4] = {
	},
};
static struct platform_device tegra_spi_devices[] = {
	[0] = {
		.name = "tegra_spi_slave",
		.id = 0,
		.dev = {
			.platform_data = &tegra_spi_platform[0],
		},
	},
	[1] = {
		.name = "tegra_spi",
		.id = 1,
		.dev = {
			.platform_data = &tegra_spi_platform[1],
		},
	},
	[2] = {
		.name = "tegra_spi",
		.id = 2,
		.dev = {
			.platform_data = &tegra_spi_platform[2],
		},
	},
	[3] = {
		.name = "tegra_spi",
		.id = 3,
		.dev = {
			.platform_data = &tegra_spi_platform[3],
		},
	},
	[4] = {
		.name = "tegra_spi",
		.id = 4,
		.dev = {
			.platform_data = &tegra_spi_platform[4],
		},
	},
};

static void olympus_spi_init(void)
{


	int rc;

	rc = platform_device_register(&tegra_spi_devices[0]);
	if (rc) {
		pr_err("%s: registration of %s.%d failed\n",
		       __func__, tegra_spi_devices[0].name, tegra_spi_devices[0].id);
	}

	rc = platform_device_register(&tegra_spi_devices[1]);
	if (rc) {
		pr_err("%s: registration of %s.%d failed\n",
		       __func__, tegra_spi_devices[1].name, tegra_spi_devices[1].id);
	}

	rc = platform_device_register(&tegra_spi_devices[2]);
	if (rc) {
		pr_err("%s: registration of %s.%d failed\n",
		       __func__, tegra_spi_devices[2].name, tegra_spi_devices[2].id);
	}

	printk(KERN_INFO "pICS_%s: Ending...",__func__);
}

static __initdata struct tegra_clk_init_table olympus_clk_init_table[] = {
	/* name		parent		rate		enabled */
/*	{ "uartb",	"clk_m",	26000000,	true},
	{ "host1x",	"pll_p",	108000000,	true},
	{ "2d",		"pll_m",	50000000,	true},
	{ "epp",	"pll_m",	50000000,	true},
	{ "vi",		"pll_m",	50000000,	true},
	{ NULL,		NULL,		0,		0},*/
	{ "uartb",	"pll_p",	216000000,	true},
	{ "uartc",	"pll_m",	600000000,	true},
	{ "uartd",	"pll_p",	216000000,	true},
	{ "emc",	"pll_m",	600000000,	true},
	{ "pll_m",	NULL,		600000000,	true},
	{ "mpe",	"pll_c",	300000000,	false},
	{ "pll_a",	"pll_p_out1",	56448000,	false},
	{ "pll_a_out0",	"pll_a",	11289600,	false},
	{ "i2s1",	"pll_a_out0",	2822400,	false},
	{ "i2s2",	"clk_m",	26000000,	false},
	{ "sdmmc4",	"pll_p",	48000000,	true},
	{ "sdmmc2",	"pll_p",	48000000,	false},
	{ "spdif_out",	"pll_a_out0",	11289600,	false},
	{ NULL,		NULL,		0,		0},
};

static void __init tegra_olympus_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	int i;

	/*
	 * Dump some key ATAGs
	 */
	for (t=tags; t->hdr.size; t = tag_next(t)) {
		switch (t->hdr.tag) {
		case ATAG_WLAN_MAC:        // 57464d41 parsed in board-mot-wlan.c
		case ATAG_BLDEBUG:         // 41000811 same, in board-mot-misc.c
		case ATAG_POWERUP_REASON:  // F1000401 ex: 0x4000, parsed after... ignore
			break;
		case ATAG_CORE:     // 54410001
			printk("%s: atag_core hdr.size=%d\n", __func__, t->hdr.size);
			break;
		case ATAG_CMDLINE:
			printk("%s: atag_cmdline=\"%s\"\n", __func__, t->u.cmdline.cmdline);
			break;
		case ATAG_REVISION: // 54410007
			printk("%s: atag_revision=0x%x\n", __func__, t->u.revision.rev);
			break;
		case ATAG_SERIAL:   // 54410006
			printk("%s: atag_serial=%x%x\n", __func__, t->u.serialnr.low, t->u.serialnr.high);
			break;
		case ATAG_INITRD2:  // 54420005
			printk("%s: atag_initrd2=0x%x size=0x%x\n", __func__, t->u.initrd.start, t->u.initrd.size);
			break;
		case ATAG_MEM:
			printk("%s: atag_mem.start=0x%x, mem.size=0x%x\n", __func__, t->u.mem.start, t->u.mem.size);
			break;
#ifdef CONFIG_MACH_OLYMPUS
		case ATAG_MOTOROLA: // 41000810
			printk("%s: atag_moto allow_fb=%d\n", __func__, t->u.motorola.allow_fb_open);
			break;
#endif
		case ATAG_NVIDIA_TEGRA: // 41000801
			printk("%s: atag_tegra=0x%X\n", __func__, t->u.tegra.bootarg_key);
			break;
		default:
			printk("%s: ATAG %X\n", __func__, t->hdr.tag);
		}
	}

	/*
	 * Dump memory nodes
	 */
	for (i=0; i<mi->nr_banks; i++) {
		printk("%s: bank[%d]=%lx@%lx\n", __func__, i, mi->bank[i].size, mi->bank[i].start);
	}
#if 0
	mi->nr_banks = 2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = 448 * SZ_1M;
	mi->bank[1].start = SZ_512M;
	mi->bank[1].size = SZ_512M;
#endif
}

static void __init tegra_olympus_init(void)
{
	struct clk *clk;

	/* Olympus has a USB switch that disconnects the usb port from the AP20
	   unless a factory cable is used, the factory jumper is set, or the
	   usb_data_en gpio is set.
	 */
	tegra_clk_init_from_table(olympus_clk_init_table);
	olympus_emc_init();

	tegra_gpio_enable(TEGRA_GPIO_PV6);
	gpio_request(TEGRA_GPIO_PV6, "usb_data_en");
	gpio_direction_output(TEGRA_GPIO_PV6, 1);

	olympus_pinmux_init();

	clk = tegra_get_clock_by_name("uartb");
	debug_uart_platform_data[0].uartclk = clk_get_rate(clk);

	clk = clk_get_sys("dsi", NULL);
	clk_enable(clk);
	clk_put(clk);

	clk = clk_get_sys("3d", NULL);
	tegra_periph_reset_assert(clk);
	writel(0x101, IO_ADDRESS(TEGRA_PMC_BASE) + 0x30);
	clk_enable(clk);
	udelay(10);
	writel(1 << 1, IO_ADDRESS(TEGRA_PMC_BASE) + 0x34);
	tegra_periph_reset_deassert(clk);
	clk_put(clk);


	platform_add_devices(olympus_devices, ARRAY_SIZE(olympus_devices));

	olympus_keypad_init();
	olympus_i2c_init();
	olympus_sdhci_init();
	olympus_spi_init();
	olympus_power_init();
	olympus_panel_init();
	olympus_wlan_init();
}

int __init tegra_olympus_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(tegra_olympus_protected_aperture_init);

void __init tegra_olympus_reserve(void)
{
	u64 ram_console_start;
	int ret;

	if (memblock_reserve(0x0, 4096) < 0) {
			pr_warn("Cannot reserve first 4K of memory for safety\n");
	}

	tegra_reserve(SZ_256M, SZ_8M, SZ_16M);

	/*!!!!!!!!!!!!! Reserve memory for the ram console. !!!!!!!!!!!!!!!!!!!!!!*/
	ram_console_start = memblock_end_of_DRAM() - SZ_1M;

	ret = memblock_remove(ram_console_start, SZ_1M);
	if (ret < 0) {
		pr_err("Failed to reserve 0x%x bytes for ram_console at "
				"0x%llx, err = %d.\n",
				SZ_1M, ram_console_start, ret);
	} else {
		ram_console_resource[0].start = ram_console_start;
		ram_console_resource[0].end = ram_console_start + SZ_1M - 1;
	}

}

MACHINE_START(OLYMPUS, "Olympus")
	.boot_params  = 0x00000100,
	.fixup		= tegra_olympus_fixup,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_olympus_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_olympus_init,
MACHINE_END
