/*
 * arch/arm/mach-tegra/board-nvodm.c
 *
 * Converts data from ODM query library into platform data
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h>
#include <linux/regulator/machine.h>
//#include <linux/lbee9qmb-rfkill.h>
#include <linux/pda_power.h>
#include <linux/gpio.h>
#include <linux/console.h>
#include <linux/reboot.h>
#include <linux/spi-tegra.h>
#include <linux/i2c-tegra.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_data/tegra_usb.h>

#include <linux/usb/android_composite.h>

#include <asm/mach/time.h>

#include <mach/clk.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/kbc.h>
#include <mach/nand.h>
#include <mach/pinmux.h>
#include <mach/sdhci.h>
#include <mach/spi.h>
#include <mach/w1.h>
#include <mach/usb_phy.h>

#include <asm/mach-types.h>

#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "board.h"
#include "hwrev.h"
#include "board-olympus.h"
#include <linux/mmc/host.h>

#define BT_RESET 0
#define BT_SHUTDOWN 1

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTB_BASE),
		.mapbase	= TEGRA_UARTB_BASE,
		.irq		= INT_UARTB,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 0, /* filled in by init */
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
#if 0
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
#endif
/* 
 * SDHCI init
 */

extern struct tegra_nand_platform tegra_nand_plat;

static struct tegra_sdhci_platform_data olympus_sdhci_platform[] = {
	[0] = { /* SDHCI 1 - WIFI*/
		.mmc_data = {
			.built_in = 1,
		},
		.wp_gpio = -1,
		.cd_gpio = -1,
		.power_gpio = -1,
//,		.max_clk_limit = 50000000,
	},
	[1] = {
		
	},
	[2] = {
		.mmc_data = {
			.built_in = 0,
			.card_present = 0,
		},
		.wp_gpio = -1,
		.cd_gpio = 69,
		.power_gpio = -1,
//		.max_clk_limit = 50000000,
	},
	[3] = {
		.mmc_data = {
			.built_in = 1,
		},
		.wp_gpio = -1,
		.cd_gpio = -1,
		.power_gpio = -1,
		.is_8bit = 1,
//		.max_clk_limit = 50000000,
	},
};

static const char tegra_sdio_ext_reg_str[] = "vsdio_ext";
int tegra_sdhci_boot_device = -1;

static void __init olympus_sdhci_init(void)
{
	int i;

	printk(KERN_INFO "pICS_%s: Starting...",__func__);
	
	tegra_sdhci_device1.dev.platform_data = &olympus_sdhci_platform[0];
	tegra_sdhci_device3.dev.platform_data = &olympus_sdhci_platform[2];
	tegra_sdhci_device4.dev.platform_data = &olympus_sdhci_platform[3];

	/* Olympus P3+, Etna P2+, Etna S3+, Daytona and Sunfire
	   can handle shutting down the external SD card. */
	if ( (HWREV_TYPE_IS_FINAL(system_rev) || (HWREV_TYPE_IS_PORTABLE(system_rev) && (HWREV_REV(system_rev) >= HWREV_REV_3)))) 			{
		olympus_sdhci_platform[2].regulator_str = (char *)tegra_sdio_ext_reg_str;
		}

		/* check if an "MBR" partition was parsed from the tegra partition
		 * command line, and store it in sdhci.3's offset field */
	for (i=0; i<tegra_nand_plat.nr_parts; i++) {
		if (strcmp("mbr", tegra_nand_plat.parts[i].name))
			continue;
		olympus_sdhci_platform[3].offset = tegra_nand_plat.parts[i].offset;
		printk(KERN_INFO "pICS_%s: tegra_sdhci_boot_device plat->offset = 0x%llx ",__func__, tegra_nand_plat.parts[i].offset);		
		}

	platform_device_register(&tegra_sdhci_device4); 
	platform_device_register(&tegra_sdhci_device1);
	platform_device_register(&tegra_sdhci_device3);

	printk(KERN_INFO "pICS_%s: Ending...",__func__);
}
#if 0
#ifdef CONFIG_LBEE9QMB_RFKILL
static struct lbee9qmb_platform_data lbee9qmb_platform;
static struct platform_device lbee9qmb_device = {
	.name = "lbee9qmb-rfkill",
	.dev = {
		.platform_data = &lbee9qmb_platform,
	},
};
static noinline void __init olympus_rfkill_init(void)
{

	lbee9qmb_platform.de1lay=5;
	lbee9qmb_platform.gpio_pwr=-1;
	lbee9qmb_platform.gpio_reset = 160;

	if (platform_device_register(&lbee9qmb_device))
					pr_err("%s: registration failed\n", __func__);
				return;
}
#else
static void olympus_rfkill_init(void) { }
#endif
#endif
static void tegra_system_power_off(void)
{
	struct regulator *regulator = regulator_get(NULL, "soc_main");

	if (!IS_ERR(regulator)) {
		int rc;
		regulator_enable(regulator);
		rc = regulator_disable(regulator);
		pr_err("%s: regulator_disable returned %d\n", __func__, rc);
	} else {
		pr_err("%s: regulator_get returned %ld\n", __func__,
		       PTR_ERR(regulator));
	}
	local_irq_disable();
	while (1) {
		dsb();
		__asm__ ("wfi");
	}
}
#if 0
static struct resource tegra_grhost_resources[] = {
	[0] = {
		.name = "host1x",
		.start = TEGRA_HOST1X_BASE,
		.end = TEGRA_HOST1X_BASE + TEGRA_HOST1X_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "display",
		.start = TEGRA_DISPLAY_BASE,
		.end = TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name = "display2",
		.start = TEGRA_DISPLAY2_BASE,
		.end = TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.name = "vi",
		.start = TEGRA_VI_BASE,
		.end = TEGRA_VI_BASE + TEGRA_VI_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		.name = "isp",
		.start = TEGRA_ISP_BASE,
		.end = TEGRA_ISP_BASE + TEGRA_ISP_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[5] = {
		.name = "mpe",
		.start = TEGRA_MPE_BASE,
		.end = TEGRA_MPE_BASE + TEGRA_MPE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[6] = {
		.name = "syncpt_thresh",
		.start = INT_SYNCPT_THRESH_BASE,
		.end = INT_SYNCPT_THRESH_BASE + INT_SYNCPT_THRESH_NR - 1,
		.flags = IORESOURCE_IRQ,
	},
	[7] = {
		.name = "host1x_mpcore_general",
		.start = INT_HOST1X_MPCORE_GENERAL,
		.end = INT_HOST1X_MPCORE_GENERAL,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device tegra_grhost_device = {
	.name = "tegra_grhost",
	.id = -1,
	.resource = tegra_grhost_resources,
	.num_resources = ARRAY_SIZE(tegra_grhost_resources),
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

/*
static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = false,
	.has_hostpc = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};*/

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = TEGRA_GPIO_PD3,
		.vbus_reg = NULL,
		.hot_plug = true,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 9,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
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

static u64 tegra_udc_dma_mask = DMA_BIT_MASK(32);

static struct fsl_usb2_platform_data tegra_udc_platform = {
	.phy_mode = FSL_USB2_PHY_UTMI,
	.operating_mode = FSL_USB2_DR_DEVICE,
};
static struct resource tegra_udc_resources[] = {
	[0] = {
		.start = TEGRA_USB_BASE,
		.end = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_USB,
		.end = INT_USB,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device tegra_udc_dev = {
	.name = "tegra-udc",
	.id = 0,
	.dev = {
		.platform_data = &tegra_udc_platform,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &tegra_udc_dma_mask,
	},
	.resource = tegra_udc_resources,
	.num_resources = ARRAY_SIZE(tegra_udc_resources),
};

static const u32 olympus_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
	KEY(0, 2, KEY_AUX),
	KEY(1, 0, KEY_CAMERA_FOCUS),
	KEY(1, 1, KEY_CAMERA),
	KEY(1, 2, KEY_SEARCH),
	KEY(2, 0, KEY_MENU),
	KEY(2, 1, KEY_HOME),
	KEY(2, 2, KEY_BACK),
};

static const struct matrix_keymap_data olympus_keymap_data = {
	.keymap = olympus_keymap,
	.keymap_size = ARRAY_SIZE(olympus_keymap),
};

struct tegra_kbc_platform_data tegra_kbc_platform;

static struct resource tegra_kbc_resources[] = {
	[0] = {
		.start = TEGRA_KBC_BASE,
		.end = TEGRA_KBC_BASE + TEGRA_KBC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_KBC,
		.end = INT_KBC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device tegra_kbc_dev = {
	.name = "tegra-kbc",
	.id = -1,
	.dev = {
		.platform_data = &tegra_kbc_platform,
	},
	.resource = tegra_kbc_resources,
	.num_resources = ARRAY_SIZE(tegra_kbc_resources),
};
#endif
static noinline void __init olympus_kbc_init(void)
{
#if 0
	tegra_kbc_platform.wake_cnt = 5; /* 0:wake on any key >1:wake on wake_cfg */

	/* debounce time is reported from ODM in terms of clock ticks. */
	tegra_kbc_platform.debounce_cnt = 10;

	/* repeat cycle is reported from ODM in milliseconds,
	* but needs to be specified in 32KHz ticks */
	tegra_kbc_platform.repeat_cnt = 1024;

	tegra_kbc_platform.pin_cfg[0].num = 0;
	tegra_kbc_platform.pin_cfg[0].is_row = true;
	tegra_kbc_platform.pin_cfg[1].num = 1;
	tegra_kbc_platform.pin_cfg[1].is_row = true;
	tegra_kbc_platform.pin_cfg[2].num = 2;
	tegra_kbc_platform.pin_cfg[2].is_row = true;
	tegra_kbc_platform.pin_cfg[16].num = 0;
	tegra_kbc_platform.pin_cfg[16].en = true; /*ICS check if is_col = en*/
	tegra_kbc_platform.pin_cfg[17].num = 1;
	tegra_kbc_platform.pin_cfg[17].en = true;
	tegra_kbc_platform.pin_cfg[18].num = 2;
	tegra_kbc_platform.pin_cfg[18].en = true;

	tegra_kbc_platform.keymap_data = &olympus_keymap_data;

/*	tegra_kbc_platform.keymap[0]=115;
	tegra_kbc_platform.keymap[1]=114;
	tegra_kbc_platform.keymap[2]=152;
	tegra_kbc_platform.keymap[16]=211;
	tegra_kbc_platform.keymap[17]=212;
	tegra_kbc_platform.keymap[18]=217;
	tegra_kbc_platform.keymap[32]=139;
	tegra_kbc_platform.keymap[33]=102;
	tegra_kbc_platform.keymap[34]=158;*/
#endif
}

static struct tegra_i2c_platform_data olympus_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },

};

static struct tegra_i2c_platform_data olympus_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 400000, 400000 },
};

static struct tegra_i2c_platform_data olympus_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
};

static struct tegra_i2c_platform_data olympus_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
};

void olympus_i2c_reg(void)
{
	tegra_i2c_device1.dev.platform_data = &olympus_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &olympus_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &olympus_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &olympus_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);
}

static struct platform_device *olympus_devices[] __initdata = {
	&tegra_gart_device,
	&tegra_grhost_device,
	&debug_uart,
/*	&tegra_otg,
	&androidusb_device,
	&pda_power_device,*/
	&tegra_uarta_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
	&tegra_spi_device2,
	&tegra_spi_device3,
	&tegra_spi_device4,
	&tegra_w1_device,
/*	&tegra_udc_dev,*/
/*	&tegra_kbc_dev,*/
};

static int tegra_reboot_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	printk(KERN_INFO "pICS_%s: event = [%lu]",__func__, event);
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		/* USB power rail must be enabled during boot */
		/*NvOdmEnableUsbPhyPowerRail(1);*/
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tegra_reboot_nb = {
	.notifier_call = tegra_reboot_notify,
	.next = NULL,
	.priority = 0
};

static void olympus_reboot_init(void)
{
	int rc = register_reboot_notifier(&tegra_reboot_nb);
	if (rc)
		pr_err("%s: failed to regsiter platform reboot notifier\n",
			__func__);
}

void __init olympus_devices_init()
{
	struct clk *clk;

	olympus_i2c_reg();
	printk(KERN_INFO "pICS_%s: olympus_sdhci_init();\n",__func__);
	olympus_sdhci_init();
	printk(KERN_INFO "pICS_%s: olympus_rfkill_init();\n",__func__);
/*	olympus_rfkill_init();*/
	olympus_kbc_init();

	clk = tegra_get_clock_by_name("uartb");
	debug_uart_platform_data[0].uartclk = clk_get_rate(clk);

	platform_add_devices(olympus_devices, ARRAY_SIZE(olympus_devices));

	pm_power_off = tegra_system_power_off;
	
	olympus_reboot_init();
}

