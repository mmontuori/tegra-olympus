/*
 * Copyright (C) 2007-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/pm.h>
#ifdef CONFIG_BOOTINFO
#include <asm/bootinfo.h>
#endif

struct cpcap_driver_info {
	struct list_head list;
	struct platform_device *pdev;
};

static long ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int __devinit cpcap_probe(struct spi_device *spi);
static int __devexit cpcap_remove(struct spi_device *spi);

#ifdef CONFIG_PM
static int cpcap_suspend(struct spi_device *spi, pm_message_t mesg);
static int cpcap_resume(struct spi_device *spi);
#endif

const static struct file_operations cpcap_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ioctl,
};

static struct miscdevice cpcap_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= CPCAP_DEV_NAME,
	.fops	= &cpcap_fops,
};

static struct spi_driver cpcap_driver = {
	.driver = {
		   .name = "cpcap",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = cpcap_probe,
	.remove = __devexit_p(cpcap_remove),
#ifdef CONFIG_PM
	.suspend = cpcap_suspend,
	.resume = cpcap_resume,
#endif
};

static struct platform_device cpcap_adc_device = {
	.name           = "cpcap_adc",
	.id             = -1,
	.dev.platform_data = NULL,
};


static struct platform_device cpcap_key_device = {
	.name           = "cpcap_key",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_uc_device = {
	.name           = "cpcap_uc",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_rtc_device = {
	.name           = "cpcap_rtc",
	.id             = -1,
	.dev.platform_data = NULL,
};



/* List of required CPCAP devices that will ALWAYS be present.
 *
 * DO NOT ADD NEW DEVICES TO THIS LIST! You must use cpcap_driver_register()
 * for any new drivers for non-core functionality of CPCAP.
 */
static struct platform_device *cpcap_devices[] = {
	&cpcap_uc_device,
	&cpcap_adc_device,
	&cpcap_key_device,
	&cpcap_rtc_device,
};

static struct cpcap_device *misc_cpcap;

static LIST_HEAD(cpcap_device_list);
static DEFINE_MUTEX(cpcap_driver_lock);

static int cpcap_reboot(struct notifier_block *this, unsigned long code,
			void *cmd)
{
	int ret = -1;
	int result = NOTIFY_DONE;

	/* Disable the USB transceiver */
	ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_USBC2, 0,
				 CPCAP_BIT_USBXCVREN);

	if (ret) {
		dev_err(&(misc_cpcap->spi->dev),
			"Disable Transciever failure.\n");
		result = NOTIFY_BAD;
	}

	if (code == SYS_RESTART)
		cpcap_regacc_write(misc_cpcap, CPCAP_REG_MI2, 0, 0xFFFF);

	/* Always clear the power cut bit on SW Shutdown*/
	ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_PC1,
		0, CPCAP_BIT_PC1_PCEN);
	if (ret) {
		dev_err(&(misc_cpcap->spi->dev),
			"Clear Power Cut bit failure.\n");
		result = NOTIFY_BAD;
	}

	/* Clear the charger and charge path settings to avoid a false turn on
	 * event in caused by CPCAP. After clearing these settings, 100ms is
	 * needed to before SYSRSTRTB is pulled low to avoid the false turn on
	 * event.
	 */
	cpcap_regacc_write(misc_cpcap, CPCAP_REG_CRM, 0, 0x3FFF);
	mdelay(100);

	return result;
}
static struct notifier_block cpcap_reboot_notifier = {
	.notifier_call = cpcap_reboot,
};

static int __init cpcap_init(void)
{
	return spi_register_driver(&cpcap_driver);
}

static void cpcap_vendor_read(struct cpcap_device *cpcap)
{
	unsigned short value;

	(void)cpcap_regacc_read(cpcap, CPCAP_REG_VERSC1, &value);

	cpcap->vendor = (enum cpcap_vendor)((value >> 6) & 0x0007);
	cpcap->revision = (enum cpcap_revision)(((value >> 3) & 0x0007) |
						((value << 3) & 0x0038));
}


int cpcap_device_unregister(struct platform_device *pdev)
{
	struct cpcap_driver_info *info;
	struct cpcap_driver_info *tmp;
	int found;


	found = 0;
	mutex_lock(&cpcap_driver_lock);

	list_for_each_entry_safe(info, tmp, &cpcap_device_list, list) {
		if (info->pdev == pdev) {
			list_del(&info->list);

			/*
			 * misc_cpcap != NULL suggests pdev
			 * already registered
			 */
			if (misc_cpcap) {
				printk(KERN_INFO "CPCAP: unregister %s\n",
					pdev->name);
				platform_device_unregister(pdev);
			}
			info->pdev = NULL;
			kfree(info);
			found = 1;
		}
	}

	mutex_unlock(&cpcap_driver_lock);

	BUG_ON(!found);
	return 0;
}

int cpcap_device_register(struct platform_device *pdev)
{
	int retval;
	struct cpcap_driver_info *info;

	retval = 0;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		printk(KERN_ERR	"Cannot save device %s\n", pdev->name);
		return -ENOMEM;
	}

	mutex_lock(&cpcap_driver_lock);

	info->pdev = pdev;
	list_add_tail(&info->list, &cpcap_device_list);

	/* If misc_cpcap is valid, the CPCAP driver has already been probed.
	 * Therefore, call platform_device_register() to probe the device.
	 */
	if (misc_cpcap) {
		dev_info(&(misc_cpcap->spi->dev),
			 "Probing CPCAP device %s\n", pdev->name);

		/*
		 * platform_data is non-empty indicates
		 * CPCAP client devices need to pass their own data
		 * In that case we put cpcap data in driver_data
		 */
		if (pdev->dev.platform_data != NULL)
			platform_set_drvdata(pdev, misc_cpcap);
		else
			pdev->dev.platform_data = misc_cpcap;
		retval = platform_device_register(pdev);
	} else
		printk(KERN_INFO "CPCAP: delaying %s probe\n",
				pdev->name);
	mutex_unlock(&cpcap_driver_lock);

	return retval;
}

static void cpcap_standby_macro_init(void)
{
	/* enable standby macros */
	cpcap_uc_start(misc_cpcap, CPCAP_BANK_SECONDARY, CPCAP_MACRO_4);
	cpcap_uc_start(misc_cpcap, CPCAP_BANK_SECONDARY, CPCAP_MACRO_5);
}

static int __devinit cpcap_probe(struct spi_device *spi)
{
	int retval = -EINVAL;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *data;
	int i;
	struct cpcap_driver_info *info;

	// printk(KERN_INFO "pICS_%s: step 1...\n",__func__);

	cpcap = kzalloc(sizeof(*cpcap), GFP_KERNEL);
	if (cpcap == NULL)
		return -ENOMEM;
	
	// printk(KERN_INFO "pICS_%s: step 1a...\n",__func__);

	cpcap->spi = spi;

	// printk(KERN_INFO "pICS_%s: step 1b...\n",__func__);

	data = spi->controller_data;

	// printk(KERN_INFO "pICS_%s: step 1c...\n",__func__);
	misc_cpcap = cpcap;  /* kept for misc device */

	// printk(KERN_INFO "pICS_%s: step 1d...\n",__func__);

	spi_set_drvdata(spi, cpcap);
	// printk(KERN_INFO "pICS_%s: step 1e...\n",__func__);

	cpcap->spdif_gpio = data->spdif_gpio;
	// printk(KERN_INFO "pICS_%s: step 1f...\n",__func__);

	retval = cpcap_regacc_init(cpcap);
	// printk(KERN_INFO "pICS_%s: step 1g...\n",__func__);

	if (retval < 0)
		goto free_mem;
	retval = cpcap_irq_init(cpcap);
	if (retval < 0)
		goto free_cpcap_irq;

	// printk(KERN_INFO "pICS_%s: step 2...\n",__func__);

#ifdef CONFIG_BOOTINFO
	if (bi_powerup_reason() != PU_REASON_CHARGER) {
		/* Set Kpanic bit, which will be cleared at normal reboot */
		cpcap_regacc_write(cpcap, CPCAP_REG_VAL1,
			CPCAP_BIT_AP_KERNEL_PANIC, CPCAP_BIT_AP_KERNEL_PANIC);
	}
#endif

	cpcap_vendor_read(cpcap);

	// printk(KERN_INFO "pICS_%s: step 3...\n",__func__);

	for (i = 0; i < ARRAY_SIZE(cpcap_devices); i++)
		cpcap_devices[i]->dev.platform_data = cpcap;

	retval = misc_register(&cpcap_dev);
	if (retval < 0)
		goto free_cpcap_irq;

	// printk(KERN_INFO "pICS_%s: step 4...\n",__func__);
	/* the cpcap usb_detection device is a consumer of the
	 * vusb regulator */
/*	data->regulator_init[CPCAP_VUSB].num_consumer_supplies = 1;
	data->regulator_init[CPCAP_VUSB].consumer_supplies =
		&cpcap_vusb_consumers;
*/
	/* loop twice becuase cpcap_regulator_probe may refer to other devices
	 * in this list to handle dependencies between regulators.  Create them
	 * all and then add them */
	for (i = 0; i < CPCAP_NUM_REGULATORS; i++) {
		struct platform_device *pdev;

		pdev = platform_device_alloc("cpcap-regltr", i);
		if (!pdev) {
			dev_err(&(spi->dev), "Cannot create regulator\n");
			continue;
		}

		pdev->dev.parent = &(spi->dev);
		pdev->dev.platform_data = &data->regulator_init[i];
		platform_set_drvdata(pdev, cpcap);
		cpcap->regulator_pdev[i] = pdev;
	}

	// printk(KERN_INFO "pICS_%s: step 5...\n",__func__);
	for (i = 0; i < CPCAP_NUM_REGULATORS; i++) {
		/* vusb has to be added after sw5 so skip it for now,
		 * it will be added from probe of sw5 */
		if (i == CPCAP_VUSB)
			continue;
		platform_device_add(cpcap->regulator_pdev[i]);
	}

	// printk(KERN_INFO "pICS_%s: step 6...\n",__func__);
	platform_add_devices(cpcap_devices, ARRAY_SIZE(cpcap_devices));

	mutex_lock(&cpcap_driver_lock);
	misc_cpcap = cpcap;  /* kept for misc device */

	list_for_each_entry(info, &cpcap_device_list, list) {
		dev_info(&(spi->dev), "Probing CPCAP device %s\n",
			 info->pdev->name);
		if (info->pdev->dev.platform_data != NULL)
			platform_set_drvdata(info->pdev, cpcap);
		else
			info->pdev->dev.platform_data = cpcap;
		platform_device_register(info->pdev);
	}
	mutex_unlock(&cpcap_driver_lock);

	// printk(KERN_INFO "pICS_%s: step 7...\n",__func__);

	register_reboot_notifier(&cpcap_reboot_notifier);

	cpcap_standby_macro_init();

	return 0;

free_cpcap_irq:
	cpcap_irq_shutdown(cpcap);
free_mem:
	kfree(cpcap);
	return retval;
}

static int __devexit cpcap_remove(struct spi_device *spi)
{
	struct cpcap_device *cpcap = spi_get_drvdata(spi);
	struct cpcap_driver_info *info;
	int i;

	unregister_reboot_notifier(&cpcap_reboot_notifier);

	mutex_lock(&cpcap_driver_lock);
	list_for_each_entry(info, &cpcap_device_list, list) {
		dev_info(&(spi->dev), "Removing CPCAP device %s\n",
			 info->pdev->name);
		platform_device_unregister(info->pdev);
	}
	misc_cpcap = NULL;
	mutex_unlock(&cpcap_driver_lock);

	for (i = ARRAY_SIZE(cpcap_devices); i > 0; i--)
		platform_device_unregister(cpcap_devices[i-1]);

	for (i = 0; i < CPCAP_NUM_REGULATORS; i++)
		platform_device_unregister(cpcap->regulator_pdev[i]);

	misc_deregister(&cpcap_dev);
	cpcap_irq_shutdown(cpcap);
	kfree(cpcap);
	return 0;
}


static long test_ioctl(unsigned int cmd, unsigned long arg)
{
	int retval = -EINVAL;
	struct cpcap_regacc read_data;
	struct cpcap_regacc write_data;

	switch (cmd) {
	case CPCAP_IOCTL_TEST_READ_REG:
		if (copy_from_user((void *)&read_data, (void *)arg,
				   sizeof(read_data)))
			return -EFAULT;
		retval = cpcap_regacc_read(misc_cpcap, read_data.reg,
					   &read_data.value);
		if (retval < 0)
			return retval;
		if (copy_to_user((void *)arg, (void *)&read_data,
				 sizeof(read_data)))
			return -EFAULT;
		return 0;
	break;

	case CPCAP_IOCTL_TEST_WRITE_REG:
		if (copy_from_user((void *) &write_data,
				   (void *) arg,
				   sizeof(write_data)))
			return -EFAULT;
		retval = cpcap_regacc_write(misc_cpcap, write_data.reg,
					    write_data.value, write_data.mask);
	break;

	default:
		retval = -ENOTTY;
	break;
	}

	return retval;
}

static long adc_ioctl(unsigned int cmd, unsigned long arg)
{
	int retval = -EINVAL;
	struct cpcap_adc_phase phase;

	switch (cmd) {
	case CPCAP_IOCTL_ADC_PHASE:
		if (copy_from_user((void *) &phase, (void *) arg,
				   sizeof(phase)))
			return -EFAULT;

		cpcap_adc_phase(misc_cpcap, &phase);
		retval = 0;
	break;

	default:
		retval = -ENOTTY;
	break;
	}

	return retval;
}

void cpcap_misc_clear_power_handoff_info(void)
{
	/* Clear the kpanic bit, and any other bits that may be present. */
	/* This forces Bootloader to evaluate any power-up reason as new */
	cpcap_regacc_write(misc_cpcap, CPCAP_REG_VAL1, 0, 0xFFFF);
}

bool cpcap_misc_is_ext_power(void)
{
	unsigned short value;

	/* Return true if there is valid external power. */
	cpcap_regacc_read(misc_cpcap, CPCAP_REG_INTS2, &value);
	return (value&CPCAP_BIT_VBUSVLD_S) && (value&CPCAP_BIT_CHRGCURR1_S);
}

extern void whisper_toggle_audio_switch_for_spdif(struct cpcap_device *cpcap, bool state);

void cpcap_accy_whisper_audio_switch_spdif_state(bool state)
{
	whisper_toggle_audio_switch_for_spdif(misc_cpcap, state);
}
EXPORT_SYMBOL_GPL(cpcap_accy_whisper_audio_switch_spdif_state);

extern void cpcap_accy_set_dock_switch(struct cpcap_device *cpcap, int state, bool is_hall_effect);

void cpcap_set_dock_switch(int state)
{
	cpcap_accy_set_dock_switch(misc_cpcap, state, true);
}
EXPORT_SYMBOL_GPL(cpcap_set_dock_switch);
/*
 * GPIO4 on CPCAP is used as a gate for WDI
 */
int cpcap_set_wdigate(short value)
{
	int ret = -1;
	short gpio_val = value?CPCAP_BIT_GPIO4DRV:0;

	ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_GPIO4,
				 CPCAP_BIT_GPIO4MACROML|CPCAP_BIT_GPIO4MACROMH|
				 CPCAP_BIT_GPIO4DIR, 0xFFFF);
	if (!ret)
		ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_GPIO4, gpio_val,
					 CPCAP_BIT_GPIO4DRV);

	return ret;
}
EXPORT_SYMBOL_GPL(cpcap_set_wdigate);

int cpcap_disable_powercut(void){

	int ret;

	ret = cpcap_regacc_write(misc_cpcap, CPCAP_REG_PC1,
		0, CPCAP_BIT_PC1_PCEN);
	if (ret) {
		dev_err(&(misc_cpcap->spi->dev),
			"Clear Power Cut bit failure.\n");
	}

	return ret;
}
EXPORT_SYMBOL_GPL(cpcap_disable_powercut);

static long accy_ioctl(unsigned int cmd, unsigned long arg)
{
	int retval = -EINVAL;
	struct cpcap_whisper_request read_data;

	switch (cmd) {
	case CPCAP_IOCTL_ACCY_WHISPER:
		if (copy_from_user((void *) &read_data, (void *) arg,
				   sizeof(read_data)))
			return -EFAULT;
		read_data.dock_id[CPCAP_WHISPER_ID_SIZE - 1] = '\0';
		read_data.dock_prop[CPCAP_WHISPER_PROP_SIZE - 1] = '\0';
		retval = cpcap_accy_whisper(misc_cpcap, &read_data);
	break;

	default:
		retval = -ENOTTY;
	break;
	}

	return retval;
}

static long ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int retval = -ENOTTY;
	unsigned int cmd_num;

	cmd_num = _IOC_NR(cmd);

	if ((cmd_num > CPCAP_IOCTL_NUM_TEST__START) &&
	    (cmd_num < CPCAP_IOCTL_NUM_TEST__END)) {
		retval = test_ioctl(cmd, arg);
	}
	if ((cmd_num > CPCAP_IOCTL_NUM_ADC__START) &&
	    (cmd_num < CPCAP_IOCTL_NUM_ADC__END)) {
		retval = adc_ioctl(cmd, arg);
	}
	if ((cmd_num > CPCAP_IOCTL_NUM_ACCY__START) &&
	    (cmd_num < CPCAP_IOCTL_NUM_ACCY__END)) {
		retval = accy_ioctl(cmd, arg);
	}

	return retval;
}

static void cpcap_shutdown(void)
{
	spi_unregister_driver(&cpcap_driver);
}

#ifdef CONFIG_PM
static int cpcap_suspend(struct spi_device *spi, pm_message_t mesg)
{

	struct cpcap_device *cpcap = spi_get_drvdata(spi);

	return cpcap_irq_suspend(cpcap);
}

static int cpcap_resume(struct spi_device *spi)
{
	struct cpcap_device *cpcap = spi_get_drvdata(spi);

	return cpcap_irq_resume(cpcap);
}
#endif

subsys_initcall(cpcap_init);
module_exit(cpcap_shutdown);

MODULE_ALIAS("platform:cpcap");
MODULE_DESCRIPTION("CPCAP driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
