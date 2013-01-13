/*
 * arch/arm/mach-tegra/board-mot.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 NVIDIA Corporation
 * Copyright (C) 2010 Motorola, Inc.
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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pda_power.h>
<<<<<<< HEAD
#include <linux/io.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/memblock.h>
#include <linux/console.h>

#include <asm/bootinfo.h>
=======
#include <linux/gpio.h>
#include <linux/delay.h>

>>>>>>> a43e12f... initial commit for bootable olympus kernel
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
<<<<<<< HEAD
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/cpcap-accy.h>
#include <linux/reboot.h>
=======
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/gpio.h>
#include <mach/clk.h>

#include <linux/usb/android_composite.h>
>>>>>>> a43e12f... initial commit for bootable olympus kernel

#include "clock.h"
#include "gpio-names.h"
#include "pm.h"

#include <linux/qtouch_obp_ts.h>
#include <linux/isl29030.h>

#include <linux/leds-lm3530.h>
#include <linux/leds-lm3532.h>

#include "board.h"
#include "hwrev.h"

#include "board-olympus.h"

#define PWRUP_FACTORY_CABLE         0x00000020 /* Bit 5  */
#define PWRUP_INVALID               0xFFFFFFFF
#define PWRUP_BAREBOARD             0x00100000 /* Bit 20 */


static char oly_unused_pins_p3[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,
        TEGRA_GPIO_PC1,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PE5,
        TEGRA_GPIO_PE6,
        TEGRA_GPIO_PE7,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PT4,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS0,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PS2,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        /* TEGRA_GPIO_PK5, AKM8975_RESET_GPIO */
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
};

static char oly_unused_pins_p2[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,
        TEGRA_GPIO_PC1,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PE7,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PT4,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS0,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PS2,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        TEGRA_GPIO_PK5,
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
        TEGRA_GPIO_PD1,
};

<<<<<<< HEAD
static char oly_unused_pins_p1[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        TEGRA_GPIO_PK5,
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
        TEGRA_GPIO_PV7,
        TEGRA_GPIO_PD1,
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

static __initdata struct tegra_clk_init_table olympus_clk_init_table[] = {
	/* name		parent		rate		enabled */  
	{ "twd",        NULL,           62500000,	true}, 
	{ "usb3",	"clk_m",	26000000,	false},
	{ "usbd",	"clk_m",	26000000,	true},
	{ "dvc",	"clk_m",	2888888,	true},
	{ "i2c3",	"clk_m",	2888888,	true},
	{ "i2c2",	"clk_m",	787878,		true},
	{ "i2c1",	"clk_m",	2888888,	true},
	{ "owr",	"clk_m",	1000000,	false},
	{ "kfuse",	"clk_m",	26000000,	true},
	{ "fuse_burn",	"clk_m",	26000000,	false},
	{ "fuse",	"clk_m",	26000000,	true},
	{ "i2s2",	"clk_m",	26000000,	false},
	{ "timer",	"clk_m",	26000000,	true},
/*	{ "cdev1",	"clk_m",	26000000,	true},*/
	{ "pll_u",	"clk_m",	480000000,	true},
	{ "pll_d",	"clk_m",	459000000,	true},
	{ "pll_d_out0",	"pll_d",	229500000,	true},
	{ "dsia",	"pll_d_out0",	229500000,	true},	
	{ "disp1",	"pll_d_out0",	229500000,	true},
	{ "pll_p",	"clk_m",	216000000,	true},
	{ "host1x",	"pll_p",	108000000,	true},
	{ "uartb",	"pll_p",	216000000,	true},
	{ "uartd",	"pll_p",	216000000,	true},
	{ "csite",	"pll_p",	144000000,	true},
	{ "sdmmc4",	"pll_p",	48000000,	true},
	{ "sdmmc3",	"pll_p",	48000000,	false},
	{ "sdmmc2",	"pll_p",	24000000,	false},
	{ "sdmmc1",	"pll_p",	48000000,	false},
	{ "pll_p_out3",	"pll_p",	72000000,	true},
	{ "pll_p_out1",	"pll_p",	28800000,	true},
	{ "pll_a",	"pll_p_out1",	56448000,	false},
	{ "pll_a_out0",	"pll_a",	11289600,	false},
	{ "spdif_out",	"pll_a_out0",	11289600,	false},
	{ "pll_c",	"clk_m",	600000000,	true},
	{ "mpe",	"pll_c",	300000000,	true},
	{ "epp",	"pll_c",	300000000,	true},
	{ "vi",		"pll_c",	100000000,	true},  
	{ "2d",		"pll_c",	300000000,	true},
	{ "3d",		"pll_c",	300000000,	true},
	{ "sbc2",	"pll_c",	31578947,	true},
	{ "pll_c_out1",	"pll_c",	80000000,	true},
	{ "sclk",	"pll_c_out1",	80000000,	true},
	{ "cop",	"sclk",		80000000,	true},
	{ "virt_sclk",	"sclk",		80000000,	true},
	{ "usbd.sclk",	"virt_sclk",	80000000,	true},
	{ "hclk",	"pll_c_out1",	80000000,	true},
	{ "i2s1",	"pll_a_out0",	2822400,	false},
	{ "pll_m",	"clk_m",	600000000,	true},
	{ "emc",	"pll_m",	600000000,	true},
	{ "uartc",	"pll_m",	600000000,	true},
	{ "sbc1",	"pll_m",	100000000,	true},
	{ "emc",	"pll_m",	100000000,	true},
	{ "pwm",	"clk_32k",	32768,		false},
	{ "kbc",	"clk_32k",	32768,		true},
	{ "rtc",	"clk_32k",	32768,		true},
	{ NULL,		NULL,		0,		0},
=======
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
>>>>>>> a43e12f... initial commit for bootable olympus kernel
};

#if 1
static noinline void __init tegra_setup_bluesleep(void)
{
       struct platform_device *pDev = NULL;
       struct resource *res;

       pDev = platform_device_alloc("bluesleep", 0);
       if (!pDev) {
               pr_err("unable to allocate platform device for bluesleep");
               goto fail;
       }

       res = kzalloc(sizeof(struct resource)*3, GFP_KERNEL);
       if (!res) {
               pr_err("unable to allocate resource for bluesleep\n");
               goto fail;
       }

       res[0].name   = "gpio_host_wake";
       res[0].start  = TEGRA_GPIO_PU6;
       res[0].end    = TEGRA_GPIO_PU6;
       res[0].flags  = IORESOURCE_IO;

       res[1].name   = "gpio_ext_wake";
       res[1].start  = TEGRA_GPIO_PU1;
       res[1].end    = TEGRA_GPIO_PU1;
       res[1].flags  = IORESOURCE_IO;

       res[2].name   = "host_wake";
       res[2].start  = gpio_to_irq(TEGRA_GPIO_PU6);
       res[2].end    = gpio_to_irq(TEGRA_GPIO_PU6);
       res[2].flags  = IORESOURCE_IRQ;

       if (platform_device_add_resources(pDev, res, 3)) {
               pr_err("unable to add resources to bluesleep device\n");
               goto fail;
       }

       if (platform_device_add(pDev)) {
               pr_err("unable to add bluesleep device\n");
               goto fail;
       }

fail:
       if (pDev)
               return;
}
#else
static inline void tegra_setup_bluesleep(void) { }
#endif

static int config_unused_pins(char *pins, int num)
{
        int i, ret = 0;
        
        pr_info("%s: ENTRY\n", __func__);

        for (i = 0; i < num; i++) {
                ret = gpio_request(pins[i], "unused");
                if (ret) {
                        printk(KERN_ERR "%s: Error (%d) - gpio_reqest failed for unused GPIO %d\n", __func__,ret, pins[i]);
                } else {                
                        ret = gpio_direction_output(pins[i], 1);
                        if (ret) {
                                printk(KERN_ERR "%s: Error (%d)- gpio_direction failed for unused GPIO %d\n", __func__,ret, pins[i]);
                        }
                }
        }
        
        pr_info("%s: EXIT\n", __func__);

        return ret;
}

void __init config_gpios(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PF2);
	gpio_request(TEGRA_GPIO_PF2, "spi_gpio_srdy");
	gpio_direction_output(TEGRA_GPIO_PF2, 1);
	tegra_gpio_enable(TEGRA_GPIO_PF3);
	gpio_request(TEGRA_GPIO_PF3, "sdio_en");
	gpio_direction_output(TEGRA_GPIO_PF3, 1);
	tegra_gpio_enable(TEGRA_GPIO_PF7);
	gpio_request(TEGRA_GPIO_PF3, "sdio_en");
	gpio_direction_output(TEGRA_GPIO_PF3, 1);
	tegra_gpio_enable(TEGRA_GPIO_PI5);
	gpio_request(TEGRA_GPIO_PI5, "sdhci_cd");
	gpio_direction_input(TEGRA_GPIO_PI5);
	tegra_gpio_enable(TEGRA_GPIO_PL1);
	gpio_request(TEGRA_GPIO_PL1, "spi_gpio_mrdy");
	gpio_direction_input(TEGRA_GPIO_PL1);
	tegra_gpio_enable(TEGRA_GPIO_PM2);
	gpio_request(TEGRA_GPIO_PM2, "hs_detect_enable");
	gpio_direction_output(TEGRA_GPIO_PM2,1);
	tegra_gpio_enable(TEGRA_GPIO_PT2);
	gpio_request(TEGRA_GPIO_PT2, "usb_host_pwr_en");
	gpio_direction_output(TEGRA_GPIO_PT2,0);
}

static void __init tegra_mot_init(void)
{
/*	struct clk *clk;*/

	tegra_clk_init_from_table(olympus_clk_init_table);

	olympus_emc_init();

	/* Olympus has a USB switch that disconnects the usb port from the AP20
	   unless a factory cable is used, the factory jumper is set, or the
	   usb_data_en gpio is set.
	 */
/*
	tegra_gpio_enable(TEGRA_GPIO_PV6);
	gpio_request(TEGRA_GPIO_PV6, "usb_data_en");
	gpio_direction_output(TEGRA_GPIO_PV6, 1);*/

	olympus_pinmux_init();
/*
	clk = clk_get_sys("3d", NULL);
	tegra_periph_reset_assert(clk);
	writel(0x101, IO_ADDRESS(TEGRA_PMC_BASE) + 0x30);
	clk_enable(clk);

	udelay(10);
	writel(1 << 1, IO_ADDRESS(TEGRA_PMC_BASE) + 0x34);
	tegra_periph_reset_deassert(clk);
	clk_put(clk);
*/
	olympus_devices_init();

<<<<<<< HEAD
	olympus_keypad_init();

	olympus_i2c_init();

	platform_device_register(&ram_console_device);

	if (0==1) config_gpios();
=======

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
};

extern struct tegra_sdhci_platform_data olympus_wifi_data; /* sdhci1 */

static struct tegra_sdhci_platform_data olympus_sdhci_platform_data3 = {
};

static struct tegra_sdhci_platform_data olympus_sdhci_platform_data4 = {
	.cd_gpio = TEGRA_GPIO_PH2,
	.wp_gpio = TEGRA_GPIO_PH3,
	.power_gpio = TEGRA_GPIO_PI6,
};

static __initdata struct tegra_clk_init_table olympus_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "uartb",	"clk_m",	26000000,	true},
	{ "host1x",	"pll_p",	108000000,	true},
	{ "2d",		"pll_m",	50000000,	true},
	{ "epp",	"pll_m",	50000000,	true},
	{ "vi",		"pll_m",	50000000,	true},
	{ NULL,		NULL,		0,		0},
};
>>>>>>> a43e12f... initial commit for bootable olympus kernel

/*	mot_setup_lights(&tegra_i2c_bus0_board_info[BACKLIGHT_DEV]);
	mot_setup_touch(&tegra_i2c_bus0_board_info[TOUCHSCREEN_DEV]);*/

<<<<<<< HEAD
	olympus_panel_init();

/*	mot_setup_gadget();*/

	if( (bi_powerup_reason() & PWRUP_FACTORY_CABLE) &&
	    (bi_powerup_reason() != PWRUP_INVALID) ){
#ifdef NEED_FACT_BUSY_HINT
		FactoryBusyHint(); //factory workaround no longer needed
#endif
	}

/*	mot_modem_init();*/

/*	mot_wlan_init();
	mot_sensors_init();*/

	pm_power_off = mot_system_power_off;
	if (0==1) tegra_setup_bluesleep();

	/* Configure SPDIF_OUT as GPIO by default, it can be later controlled
	   as needed. When SPDIF_OUT is enabled and if HDMI is connected, it
	   can interefere with CPCAP ID pin, as SPDIF_OUT and ID are coupled.
	*/

	tegra_gpio_enable(TEGRA_GPIO_PD4);
	gpio_request(TEGRA_GPIO_PD4, "spdif_enable");
	gpio_direction_output(TEGRA_GPIO_PD4, 0);
	gpio_export(TEGRA_GPIO_PD4, false);

	olympus_power_init();

	if ((HWREV_TYPE_IS_PORTABLE(system_rev) || HWREV_TYPE_IS_FINAL(system_rev)))
	{
		if (HWREV_REV(system_rev) >= HWREV_REV_1 && HWREV_REV(system_rev) < HWREV_REV_2)
		{
			// Olympus P1
			config_unused_pins(oly_unused_pins_p1, ARRAY_SIZE(oly_unused_pins_p1));
		}
		else if (HWREV_REV(system_rev) >= HWREV_REV_2 && HWREV_REV(system_rev) < HWREV_REV_3)
		{
			// Olympus P2
			config_unused_pins(oly_unused_pins_p2, ARRAY_SIZE(oly_unused_pins_p2));
		}
		else if (HWREV_REV(system_rev) >= HWREV_REV_3 || HWREV_TYPE_IS_FINAL(system_rev))
		{
			// Olympus P3 and newer
			config_unused_pins(oly_unused_pins_p3, ARRAY_SIZE(oly_unused_pins_p3));
		}
	}

=======
static void olympus_sdhci_init(void)
{
	/* TODO: setup GPIOs for cd, wd, and power */
	tegra_sdhci_device1.dev.platform_data = &olympus_wifi_data;
	tegra_sdhci_device3.dev.platform_data = &olympus_sdhci_platform_data3;
	tegra_sdhci_device4.dev.platform_data = &olympus_sdhci_platform_data4;

	platform_device_register(&tegra_sdhci_device1);
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device4); 
}

static void __init tegra_olympus_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = 448 * SZ_1M;
	mi->bank[1].start = SZ_512M;
	mi->bank[1].size = SZ_512M;
>>>>>>> a43e12f... initial commit for bootable olympus kernel
}

static void __init mot_fixup(struct machine_desc *desc, struct tag *tags,
                 char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	int i;

<<<<<<< HEAD
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
		printk("%s: bank[%d]=%lx@%lx\n", __func__, i, mi->bank[i].size, (long unsigned int)(mi->bank[i].start));
	}
}
=======
	/* Olympus has a USB switch that disconnects the usb port from the AP20
	   unless a factory cable is used, the factory jumper is set, or the
	   usb_data_en gpio is set.
	 */
	tegra_clk_init_from_table(olympus_clk_init_table);

	tegra_gpio_enable(TEGRA_GPIO_PV6);
	gpio_request(TEGRA_GPIO_PV6, "usb_data_en");
	gpio_direction_output(TEGRA_GPIO_PV6, 1);
>>>>>>> a43e12f... initial commit for bootable olympus kernel

int __init olympus_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(olympus_protected_aperture_init);

<<<<<<< HEAD
void __init tegra_olympus_reserve(void)
{
	u64 ram_console_start;
	int ret;

	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_256M, SZ_8M, SZ_16M);

	/* Reserve memory for the ram console. */
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
    .fixup        = mot_fixup,
    .map_io       = tegra_map_common_io,
    .reserve	  = tegra_olympus_reserve,
    .init_early	  = tegra_init_early,
    .init_irq     = tegra_init_irq,
    .timer        = &tegra_timer,
    .init_machine = tegra_mot_init,

=======
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
	olympus_panel_init();
	olympus_sdhci_init();
	olympus_wlan_init();
}

MACHINE_START(OLYMPUS, "Olympus")
	.boot_params  = 0x00000100,
	.fixup		= tegra_olympus_fixup,
	.map_io         = tegra_map_common_io,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_olympus_init,
>>>>>>> a43e12f... initial commit for bootable olympus kernel
MACHINE_END

