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


#include <asm/bootinfo.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/gpio.h>
#include <mach/clk.h>

#include <linux/usb/android_composite.h>

#include "board.h"
#include "board-olympus.h"
#include "clock.h"
#include "gpio-names.h"
#include "devices.h"

/* NVidia bootloader tags and parsing routines */
#define ATAG_NVIDIA		0x41000801

#define ATAG_NVIDIA_RM			0x1
#define ATAG_NVIDIA_DISPLAY		0x2
#define ATAG_NVIDIA_FRAMEBUFFER		0x3
#define ATAG_NVIDIA_CHIPSHMOO		0x4
#define ATAG_NVIDIA_CHIPSHMOOPHYS	0x5
#define ATAG_NVIDIA_CARVEOUT		0x6
#define ATAG_NVIDIA_WARMBOOT		0x7

#define ATAG_NVIDIA_PRESERVED_MEM_0	0x10000
#define ATAG_NVIDIA_PRESERVED_MEM_N	3
#define ATAG_NVIDIA_FORCE_32		0x7fffffff

/**
 * Resource Manager boot args.
 *
 * Nothing here yet.
 */
struct NVBOOTARGS_Rm
{
	u32 	reserved;
};

/**
 * Carveout boot args, which define the physical memory location of the GPU
 * carved-out memory region(s).
 */
struct NVBOOTARGS_Carveout
{
	void* 	base;
	u32 	size;
};

/**
 * Warmbootloader boot args. This structure only contains
 * a mem handle key to preserve the warm bootloader
 * across the bootloader->os transition
 */
struct NVBOOTARGS_Warmboot
{
	/* The key used for accessing the preserved memory handle */
	u32 	MemHandleKey;
};

/**
 * PreservedMemHandle boot args, indexed by ATAG_NVIDIA_PRESERVED_MEM_0 + n.
 * This allows physical memory allocations (e.g., for framebuffers) to persist
 * between the bootloader and operating system.  Only carveout and IRAM
 * allocations may be preserved with this interface.
 */
struct NVBOOTARGS_PreservedMemHandle
{
	u32 	Address;
	u32   	Size;
};

/**
 * Display boot args.
 *
 * The bootloader may have a splash screen. This will flag which controller
 * and device was used for the splash screen so the device will not be
 * reinitialized (which causes visual artifacts).
 */
struct NVBOOTARGS_Display
{
	/* which controller is initialized */
	u32 	Controller;

	/* index into the ODM device list of the boot display device */
	u32 	DisplayDeviceIndex;

	/* set to != 0 if the display has been initialized */
	u8 		bEnabled;
};

/**
 * Framebuffer boot args
 *
 * A framebuffer may be shared between the bootloader and the
 * operating system display driver.  When this key is present,
 * a preserved memory handle for the framebuffer must also
 * be present, to ensure that no display corruption occurs
 * during the transition.
 */
struct NVBOOTARGS_Framebuffer
{
	/*  The key used for accessing the preserved memory handle */
	u32 	MemHandleKey;
	/*  Total memory size of the framebuffer */
	u32 	Size;
	/*  Color format of the framebuffer, cast to a U32  */
	u32 	ColorFormat;
	/*  Width of the framebuffer, in pixels  */
	u16 	Width;
	/*  Height of each surface in the framebuffer, in pixels  */
	u16 	Height;
	/*  Pitch of a framebuffer scanline, in bytes  */
	u16 	Pitch;
	/*  Surface layout of the framebuffer, cast to a U8 */
	u8  	SurfaceLayout;
	/*  Number of contiguous surfaces of the same height in the
	    framebuffer, if multi-buffering.  Each surface is
	    assumed to begin at Pitch * Height bytes from the
	    previous surface.  */
	u8  	NumSurfaces;
	/* Flags for future expandability.
	   Current allowable flags are:
	   zero - default
	   NV_BOOT_ARGS_FB_FLAG_TEARING_EFFECT - use a tearing effect signal in
	   combination with a trigger from the display software to generate
	   a frame of pixels for the display device. */
	u32 	Flags;
#define NVBOOTARG_FB_FLAG_TEARING_EFFECT (0x1)

};

/**
 * Chip characterization shmoo data
 */
struct NVBOOTARGS_ChipShmoo
{
	/* The key used for accessing the preserved memory handle of packed
	   characterization tables  */
	u32 	MemHandleKey;

	/* Offset and size of each unit in the packed buffer */
	u32 	CoreShmooVoltagesListOffset;
	u32 	CoreShmooVoltagesListSize;

	u32 	CoreScaledLimitsListOffset;
	u32 	CoreScaledLimitsListSize;

	u32 	OscDoublerListOffset;
	u32 	OscDoublerListSize;

	u32 	SKUedLimitsOffset;
	u32 	SKUedLimitsSize;

	u32 	CpuShmooVoltagesListOffset;
	u32 	CpuShmooVoltagesListSize;

	u32 	CpuScaledLimitsOffset;
	u32 	CpuScaledLimitsSize;

	/* Misc characterization settings */
	u16 	CoreCorner;
	u16 	CpuCorner;
	u32 	Dqsib;
	u32 	SvopLowVoltage;
	u32 	SvopLowSetting;
	u32 	SvopHighSetting;
};

/**
 * Chip characterization shmoo data indexed by NvBootArgKey_ChipShmooPhys
 */
struct NVBOOTARGS_ChipShmooPhys
{
	u32 	PhysShmooPtr;
	u32 	Size;
};


/**
 * OS-agnostic bootarg structure.
 */
struct NVBOOTARGS
{
	struct NVBOOTARGS_Rm 					RmArgs;
	struct NVBOOTARGS_Display 				DisplayArgs;
	struct NVBOOTARGS_Framebuffer 			FramebufferArgs;
	struct NVBOOTARGS_ChipShmoo 			ChipShmooArgs;
	struct NVBOOTARGS_ChipShmooPhys			ChipShmooPhysArgs;
	struct NVBOOTARGS_Warmboot 				WarmbootArgs;
	struct NVBOOTARGS_PreservedMemHandle 	MemHandleArgs[ATAG_NVIDIA_PRESERVED_MEM_N];
};

static __initdata struct NVBOOTARGS NvBootArgs = { {0}, {0}, {0}, {0}, {0}, {0}, {{0}} }; 

static int __init get_cfg_from_tags(void)
{
	/* If the bootloader framebuffer is found, use it */
	if (tegra_bootloader_fb_start == 0 && tegra_bootloader_fb_size == 0 &&
			NvBootArgs.FramebufferArgs.MemHandleKey >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			NvBootArgs.FramebufferArgs.MemHandleKey <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) &&
			NvBootArgs.FramebufferArgs.Size != 0 &&
			NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size != 0) 
	{
		/* Got the bootloader framebuffer address and size. Store it */
		tegra_bootloader_fb_start = NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Address;
		tegra_bootloader_fb_size  = NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size;

		pr_info("Nvidia ATAG: framebuffer: %lu @ 0x%08lx\n",tegra_bootloader_fb_size,tegra_bootloader_fb_start);
	}

	/* If the LP0 vector is found, use it */
	if (tegra_lp0_vec_start == 0 && tegra_lp0_vec_size == 0 &&
			NvBootArgs.WarmbootArgs.MemHandleKey >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			NvBootArgs.WarmbootArgs.MemHandleKey <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) &&
			NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size != 0) 
	{
		/* Got the Warmboot block address and size. Store it */
		tegra_lp0_vec_start = NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Address;
		tegra_lp0_vec_size  = NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size;

		pr_info("Nvidia ATAG: LP0: %lu @ 0x%08lx\n",tegra_lp0_vec_size,tegra_lp0_vec_start);

	}

	return 0;
}

static int __init parse_tag_nvidia(const struct tag *tag)
{
	const char *addr = (const char *)&tag->hdr + sizeof(struct tag_header);
	const struct tag_nvidia_tegra *nvtag = (const struct tag_nvidia_tegra*)addr;

	if (nvtag->bootarg_key >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			nvtag->bootarg_key <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) )
	{
		int Index = nvtag->bootarg_key - ATAG_NVIDIA_PRESERVED_MEM_0;

		struct NVBOOTARGS_PreservedMemHandle *dst = 
			&NvBootArgs.MemHandleArgs[Index];
		const struct NVBOOTARGS_PreservedMemHandle *src = 
			(const struct NVBOOTARGS_PreservedMemHandle *) nvtag->bootarg;

		if (nvtag->bootarg_len != sizeof(*dst)) {
			pr_err("Unexpected preserved memory handle tag length (expected: %d, got: %d!\n",
					sizeof(*dst), nvtag->bootarg_len);
		} else {

			pr_debug("Preserved memhandle: 0x%08x, address: 0x%08x, size: %d\n",
					nvtag->bootarg_key, src->Address, src->Size);

			memcpy(dst,src,sizeof(*dst));
		}
		return get_cfg_from_tags();
	}

	switch (nvtag->bootarg_key) {
		case ATAG_NVIDIA_FRAMEBUFFER:
			{
				struct NVBOOTARGS_Framebuffer *dst = 
					&NvBootArgs.FramebufferArgs;
				const struct NVBOOTARGS_Framebuffer *src = 
					(const struct NVBOOTARGS_Framebuffer *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected framebuffer tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					pr_debug("Framebuffer tag with 0x%08x handle, size: %d\n",
							src->MemHandleKey,src->Size);
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}
		case ATAG_NVIDIA_WARMBOOT:
			{
				struct NVBOOTARGS_Warmboot *dst = 
					&NvBootArgs.WarmbootArgs;
				const struct NVBOOTARGS_Warmboot *src =
					(const struct NVBOOTARGS_Warmboot *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected Warnboot tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					pr_debug("Found a warmboot tag with handle 0x%08x!\n", src->MemHandleKey);
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}

		default:
			pr_info("Ignoring irrelevant nvidia tag 0x%04x!\n", nvtag->bootarg_key);
			break;
	} 
	return 0;
}
__tagtable(ATAG_NVIDIA, parse_tag_nvidia);

/*
 * Some global queries for the framebuffer, display, and backlight drivers.
 */
static unsigned int s_MotorolaDispInfo = 0;
static unsigned int s_MotorolaFBInfo = 1;

unsigned short bootloader_ver_major = 0;
unsigned short bootloader_ver_minor = 0;
unsigned short uboot_ver_major = 0;
unsigned short uboot_ver_minor = 0;

unsigned char lpddr2_mr[12];

int MotorolaBootFBArgGet(unsigned int *arg)
{
    *arg = s_MotorolaFBInfo;
    return 0;
}

int MotorolaBootDispArgGet(unsigned int *arg)
{
    if(s_MotorolaDispInfo)
    {
        *arg = s_MotorolaDispInfo;
        return 0;
    }

    return -1;
}

/*
 * Parse the Motorola-specific ATAG
 */
static int __init parse_tag_motorola(const struct tag *tag)
{
    const struct tag_motorola *moto_tag = &tag->u.motorola;
    int i = 0;

    s_MotorolaDispInfo = moto_tag->panel_size;
	
    s_MotorolaFBInfo = moto_tag->allow_fb_open;
 
/*   mot_sec_platform_data.fl_factory = moto_tag->in_factory;*/

    bootloader_ver_major = moto_tag->bl_ver_major;
    bootloader_ver_minor = moto_tag->bl_ver_minor;
    uboot_ver_major = moto_tag->uboot_ver_major;
    uboot_ver_minor = moto_tag->uboot_ver_minor;
#ifdef CONFIG_BOOTINFO
    bi_set_cid_recover_boot(moto_tag->cid_suspend_boot);
#endif
    pr_info("%s: bootloader v%d.%d\n", __func__, bootloader_ver_major, bootloader_ver_minor);
    pr_info("%s: uboot v%d.%d\n", __func__, uboot_ver_major, uboot_ver_minor);
    pr_info("%s: panel_size: 0x%x\n", __func__, s_MotorolaDispInfo);
    pr_info("%s: allow_fb_open: %x\n", __func__, s_MotorolaFBInfo);
/*    pr_info("%s: factory: %d\n", __func__, mot_sec_platform_data.fl_factory);*/
    pr_info("%s: cid_suspend_boot: %u\n", __func__, (unsigned)moto_tag->cid_suspend_boot);

    /*
     *	Dump memory information
     */
     /* FIXME:  Add eMMC support */
	for (i = 0; i < 12; i++) {
		lpddr2_mr[i] = moto_tag->at_lpddr2_mr[i];
		pr_info("%s: LPDDR2 MR%d:     0x%04X (0x%04X)\n", __func__, i,
			lpddr2_mr[i],
			moto_tag->at_lpddr2_mr[i]);
	}

    return 0;
}
__tagtable(ATAG_MOTOROLA, parse_tag_motorola);

/*
 * Parse the Motorola boot loader ATAG 41000811
 */
static int __init parse_tag_bldebug(const struct tag *t)
{
	pr_info("%s: powerup reason regs: INTS1=0x%4.4x INT2=0x%4.4x INTS2=0x%4.4x INT3=0x%4.4x "
		"PC2=0x%4.4x MEMA=0x%4.4x ACCY=%d UBOOT=%d\n", __func__, t->u.bldebug.ints1,
		t->u.bldebug.int2, t->u.bldebug.ints2, t->u.bldebug.int3, t->u.bldebug.pc2,
		t->u.bldebug.mema, t->u.bldebug.accy, t->u.bldebug.uboot);
	return 0;
}
__tagtable(ATAG_BLDEBUG, parse_tag_bldebug);

/*
  * Parse the WLAN MAC ATAG
  */

char mot_wlan_mac[6] = {0x00, 0x90, 0xC3, 0x00, 0x00, 0x00};

 static int __init parse_tag_wlan_mac(const struct tag *tag)
 {
	const struct tag_wlan_mac *wlan_mac_tag = &tag->u.wlan_mac;

	pr_info("%s: WLAN MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", __func__,
	        wlan_mac_tag->addr[0], wlan_mac_tag->addr[1],
		wlan_mac_tag->addr[2], wlan_mac_tag->addr[3],
		wlan_mac_tag->addr[4], wlan_mac_tag->addr[5]);

	memcpy(mot_wlan_mac, wlan_mac_tag->addr, sizeof(mot_wlan_mac));

	return 0;
 }
 __tagtable(ATAG_WLAN_MAC, parse_tag_wlan_mac);

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
	olympus_panel_init();
	olympus_sdhci_init();
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
