/*
 * leds-asus-kpd.c - Asus LED driver
 *
 * Copyright (C) 2014 Asus Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/early_suspend_sysfs.h>

#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_pwm.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>

/*ASUS_BSP Deeo : add P73L +++ */
#if defined(CONFIG_EEPROM_PADSTATION)
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
extern int AX_MicroP_ControlLED(uint8_t, uint8_t);
static bool pad_in=false;
#endif
/*ASUS_BSP Deeo : add P73L ---*/

#define CHRLEDPWM 0x194
#define CHRLEDCTRL 0x195
#define PWM1_DUT_CYC_ER 0x68
#define PWM2_DUT_CYC_ER 0x69

#define LED_RED_GPIO 38
#define LED_GREEN_GPIO 39
/*static bool led_update = false; /*allow led update */
/*static DEFINE_MUTEX(led_mutex); */

/*ASUS_BSP Deeo : setting priority +++ */
#ifdef ASUS_FACTORY_BUILD
#define LED_RW_ATTR (S_IRUGO | S_IWUGO)
#define LED_WO_ATTR (S_IWUGO)
#define LED_RO_ATTR (S_IRUGO)
#else
#define LED_RW_ATTR (S_IRUGO | S_IWUSR | S_IWGRP)
#define LED_WO_ATTR (S_IWUSR | S_IWGRP)
#define LED_RO_ATTR (S_IRUGO)
#endif
/*ASUS_BSP Deeo : setting priority --- */

/*ASUS_BSP Deeo : add for charger mode +++*/
static int g_blink=0;
extern bool g_Main_mode;
/*ASUS_BSP Deeo : add for charger mode ---*/

/*ASUS_BSP Deeo : add for MicroP notify +++ */
void led_clean(void)
{
	printk("[LED] led_clean\n");
	g_blink = 0;
	intel_scu_ipc_iowrite8(CHRLEDPWM, 0x00);
	intel_scu_ipc_iowrite8(CHRLEDCTRL, 0x36);
	gpio_set_value(LED_RED_GPIO, 0);
	gpio_set_value(LED_GREEN_GPIO, 0);
}

#ifdef CONFIG_EEPROM_PADSTATION
static int microp_notify=0;
static int led_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	printk("[LED] Mircop event %d\n", (unsigned int)event);
	switch (event)
	{
		case P01_ADD:
			led_clean();
			pad_in = true;
			return NOTIFY_DONE;

		case P01_REMOVE:
			pad_in = false;
		default:
			return NOTIFY_DONE;
	}
}

static struct notifier_block led_mp_notifier = {
        .notifier_call = led_mp_event,
        .priority = TOUCH_MP_NOTIFY,
};
#endif
/*ASUS_BSP Deeo : add for MicroP notify --- */

/*ASUS_BSP Deeo : PAD LED brightness +++ */
void led_pad_brigthness(uint8_t value)
{
	uint8_t brightness = 0;

	brightness = (value) ? 1 : 0;

	printk("[LED] PAD LED value %d\n", value);
	printk("[LED] PAD LED blink %d\n", g_blink);

	if (g_blink)
		brightness = brightness * g_blink * 2;

	printk("[LED] PAD LED brightness %d\n", brightness);

#ifdef CONFIG_EEPROM_PADSTATION
	switch (value)
	{
		case 0:
			AX_MicroP_ControlLED(0x0, 0x0);
		break;

		case 1 :
			AX_MicroP_ControlLED(brightness, 0x0);
		break;

		case 2 :
			AX_MicroP_ControlLED(0x0, brightness);
		break;

		case 3 :
			AX_MicroP_ControlLED(brightness, brightness);
		break;

		default :
			printk("[LED] Pad LED no support!!\n");
	}
#endif
}
/*ASUS_BSP Deeo : PAD LED brightness --- */

/*ASUS_BSP Deeo : Charger mode LED +++ */
static bool default_ch_led = true;
static int pad_cable_status = 1;

void set_charger_mode_led(uint8_t led_type)
{
	if (g_Main_mode)
		return;

#ifdef CONFIG_EEPROM_PADSTATION
	printk("[LED] Charger mode type(%d) pad(%d)\n", led_type, pad_in);
#else
	printk("[LED] Charger mode type(%d) \n", led_type);
#endif

	if (led_type == 1) {	/* cable in */
#ifdef CONFIG_EEPROM_PADSTATION
		if (pad_in) {
			gpio_set_value(LED_RED_GPIO, 0);
			gpio_set_value(LED_GREEN_GPIO, 0);
			AX_MicroP_ControlLED(0x1, 0x1);
			pad_cable_status = 0;
			return;
		}
#endif
		gpio_set_value(LED_RED_GPIO, 1);
		gpio_set_value(LED_GREEN_GPIO, 1);

		default_ch_led = false;

	} else if (led_type == 2) {	/* battery full */
#ifdef CONFIG_EEPROM_PADSTATION
		if (pad_in) {
			if (pad_cable_status) {
				printk("[LED] PAD cable not pulg in\n");
				return;
			}
			printk("[LED] PAD cable pulg in\n");
			gpio_set_value(LED_RED_GPIO, 0);
			gpio_set_value(LED_GREEN_GPIO, 0);
			AX_MicroP_ControlLED(0x0, 0x1);
			return;
		}
#endif

		gpio_set_value(LED_RED_GPIO, 0);
		gpio_set_value(LED_GREEN_GPIO, 1);

		default_ch_led = false;

	} else {			/* cable out */
#ifdef CONFIG_EEPROM_PADSTATION
		if (pad_in) {
			AX_MicroP_ControlLED(0x0, 0x0);
			pad_cable_status = true;
		}
#endif
		led_clean();
		default_ch_led = true;

		return;
	}
}
EXPORT_SYMBOL(set_charger_mode_led);
/*ASUS_BSP Deeo : Charger mode LED --- */

static void asus_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret = 0;

	printk("[LED] asus_led_set\n");
/*
	if(led_update){
		printk("[LED] updating ....\n");
		return ;
	}
	led_update = true ;
	mutex_lock(&led_mutex);
*/

#ifdef CONFIG_EEPROM_PADSTATION
	if (pad_in){
		led_pad_brigthness(value);
		return;
	}
#endif

	switch (value) {
	case 0: // off
		printk("[LED] set led brightness : 0\n");

		ret = intel_scu_ipc_iowrite8(CHRLEDPWM, 0x00); 		/* duty_cycle = 0 */
		if (ret)
			goto fail;

		ret = intel_scu_ipc_iowrite8(CHRLEDCTRL, 0x36);		/* led disable */
		if (ret)
			goto fail;

		gpio_set_value(LED_RED_GPIO, 0);
		gpio_set_value(LED_GREEN_GPIO, 0);
		break;

	case 1: //red led
	case 2: //green led
	case 3: //orange led

		if(value == 1){
			printk("[LED] set led brightness : 1\n");
			gpio_set_value(LED_RED_GPIO, 1);
			gpio_set_value(LED_GREEN_GPIO, 0);
		}else if(value == 2){
			printk("[LED] set led brightness : 2\n");
			gpio_set_value(LED_RED_GPIO, 0);
			gpio_set_value(LED_GREEN_GPIO, 1);
		}else if (value ==3){
			printk("[LED] set led brightness : 3\n");
			gpio_set_value(LED_RED_GPIO, 1);
			gpio_set_value(LED_GREEN_GPIO, 1);
		}
		break;

	default :
		printk("[LED] no support CMD \n");
		break;
	}

	return ;

/*
	mutex_unlock(&led_mutex);
	led_update = false ;
*/
fail:
	printk("[LED] intel_scu_ipc_iowrite8 fail ...\n");
/*
	mutex_unlock(&led_mutex);
	led_update = false ;
*/
	return ;
}

/* ASUS_BSP Deeo : Avoid CTS fail */
static void dummy_led_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	return;
}

/* ASUS_BSP Deeo : Add for PF450CL CHRLED setting +++ */
static ssize_t ledPWM_store(struct led_classdev *led_cdev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	u32 pwm;

	ret = kstrtou32(buf, 10, &pwm);
	if (ret){
		printk("[LED] kstrtou32 fail...\n");
		return ret;
	}

	printk("[LED] ledPWM_store 0x%x\n",pwm);

	ret = intel_scu_ipc_iowrite8(CHRLEDPWM, pwm);
	if (ret){
		printk("[LED] intel_scu_ipc_iowrite8 fail ...\n");
		return ret;
	}

	return count;
}

static ssize_t ledPWM_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 tmp = 1;
	printk("[LED] ledPWM_show\n");

	ret = intel_scu_ipc_ioread8(CHRLEDPWM, &tmp);
	if (ret){
		printk("[LED] intel_scu_ipc_ioread8 fail ...\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "ledPWM_show: 0x%x\n", tmp);
}

static ssize_t ledCTRL_store(struct led_classdev *led_cdev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	u32 ctrl;

	ret = kstrtou32(buf, 10, &ctrl);
	if (ret){
		printk("[LED] kstrtou32 fail...\n");
		return ret;
	}

	printk("[LED] ledCTRL_store 0x%x\n", ctrl);

	ret = intel_scu_ipc_iowrite8(CHRLEDCTRL, ctrl);
	if (ret){
		printk("[LED] intel_scu_ipc_iowrite8 fail ...\n");
		return ret;
	}

	return count;
}

static ssize_t ledCTRL_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 tmp = 1;
	printk("[LED] ledCTRL_show\n");

	ret = intel_scu_ipc_ioread8(CHRLEDCTRL, &tmp);
	if (ret){
		printk("[LED] intel_scu_ipc_ioread8 fail ...\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "ledCTRL_show: 0x%x\n", tmp);
}

static ssize_t blink_store(struct led_classdev *led_cdev,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	u32 blink;

	ret = kstrtou32(buf, 10, &blink);
	if (ret){
		printk("[LED] kstrtou32 fail...\n");
		return ret;
	}

	printk("[LED] blink_store 0x%x\n", blink);

#ifdef CONFIG_EEPROM_PADSTATION
	if (pad_in){
		g_blink = ( blink < 0 || blink > 2 ) ? 0 : blink;
		return count;
	}
#endif

	gpio_set_value(LED_RED_GPIO, 0);
	gpio_set_value(LED_GREEN_GPIO, 0);

	switch (blink) {
		case 0:
			g_blink = 0;
			intel_scu_ipc_iowrite8(CHRLEDPWM, 0x00);
			intel_scu_ipc_iowrite8(CHRLEDCTRL, 0x36);
		break;
		case 1:
			//printk("[LED] blink 4s\n");
			g_blink = 1;
			/*intel_scu_ipc_iowrite8(CHRLEDPWM, 0xEF); 	//duty_cycle = 6.25%	0.25/4 (s) */
			intel_scu_ipc_iowrite8(CHRLEDPWM, 0xF7); 	//duty_cycle = 3.125%	0.125/4 (s)
			intel_scu_ipc_iowrite8(CHRLEDCTRL, 0x27);	// led enable
		break;

		case 2:
			//printk("[LED] blink 2s\n");
			g_blink = 2;
			/* intel_scu_ipc_iowrite8(CHRLEDPWM, 0xDF); 	//duty_cycle = 12.5%	0.25/2 (s) */
			intel_scu_ipc_iowrite8(CHRLEDPWM, 0xEF); 	//duty_cycle = 6.25%	0.125/2 (s)
			intel_scu_ipc_iowrite8(CHRLEDCTRL, 0x2F);	// led enable
		break;

		default:
			//printk("[LED] Not support\n");
			g_blink = 0;
			intel_scu_ipc_iowrite8(CHRLEDPWM, 0x00); 	//duty_cycle = 0
			intel_scu_ipc_iowrite8(CHRLEDCTRL, 0x36);	// led disable
		break;
	}

	return count;
}

static ssize_t blink_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0, value = 0;
	
	printk("[LED] blink_show %d ,Main mode %d\n", g_blink, g_Main_mode);

	value = gpio_get_value(LED_RED_GPIO);
	printk("[LED] RED %d\n", value);

	value = gpio_get_value(LED_GREEN_GPIO);
	printk("[LED] Green %d\n", value);

	return snprintf(buf, PAGE_SIZE, "blink_show: %d\n", g_blink);
}
/* ASUS_BSP Deeo : Add for PF450CL CHRLED setting --- */

static struct led_classdev asus_led = {
	.name			= "asus_led",
	.brightness_set		= asus_led_set,
	.brightness		= LED_OFF,
	.max_brightness		= 15,
};

static struct led_classdev dummy_led = {
	.name			= "red",
	.brightness_set		= dummy_led_set,
	.brightness		= LED_OFF,
	.max_brightness		= 15,
};

/* ASUS_BSP Deeo : Make LED attribute +++ */
static DEVICE_ATTR(ledPWM, LED_RW_ATTR, ledPWM_show, ledPWM_store);
static DEVICE_ATTR(ledCTRL, LED_RW_ATTR, ledCTRL_show, ledCTRL_store);
static DEVICE_ATTR(blink, LED_RW_ATTR, blink_show, blink_store);

static struct attribute *leds_attrs[] = {
	&dev_attr_ledPWM.attr,
	&dev_attr_ledCTRL.attr,
	&dev_attr_blink.attr,
	NULL
};

static const struct attribute_group leds_attr_group = {
	.attrs = leds_attrs,
};
/* ASUS_BSP Deeo : Make LED attribute ---*/

/*
static void intel_kpd_led_early_suspend(struct early_suspend *h)
{
	led_classdev_suspend(&intel_kpd_led);
}

static void intel_kpd_led_late_resume(struct early_suspend *h)
{
	led_classdev_resume(&intel_kpd_led);
}

static ssize_t early_suspend_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (!strncmp(buf, EARLY_SUSPEND_ON, EARLY_SUSPEND_STATUS_LEN))
		led_classdev_suspend(&intel_kpd_led);
	else if (!strncmp(buf, EARLY_SUSPEND_OFF, EARLY_SUSPEND_STATUS_LEN))
		led_classdev_resume(&intel_kpd_led);

	return count;
}

static DEVICE_EARLY_SUSPEND_ATTR(early_suspend_store);

static struct early_suspend intel_kpd_led_suspend_desc = {
	.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = intel_kpd_led_early_suspend,
	.resume  = intel_kpd_led_late_resume,
};
*/

static int asus_led_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret;

	dev_info(&rpdev->dev, "Probed asus_led rpmsg device\n");

	printk("[Progress][LED] starts\n");
	// Add class
	printk("[LED] Create /sys/class\n");
	ret = led_classdev_register(&rpdev->dev, &asus_led);
	if (ret) {
		dev_err(&rpdev->dev, "register asus_led failed");
		goto fail_id_check;
	}
	ret = led_classdev_register(&rpdev->dev, &dummy_led);
	if (ret) {
		dev_err(&rpdev->dev, "register asus_led failed");
		goto fail_id_check;
	}

	// Add Attribute
	printk("[LED] Create attribute\n");
	ret = sysfs_create_group(&asus_led.dev->kobj,&leds_attr_group);
	if (ret)
		goto fail_id_check;

	// Request LED control GPIO
	gpio_request(LED_RED_GPIO, "LED_RED");
	gpio_request(LED_GREEN_GPIO, "LED_GREEN");

	/* Initial PMIC register */
	intel_scu_ipc_iowrite8(CHRLEDPWM, 0x00);
	intel_scu_ipc_iowrite8(CHRLEDCTRL, 0x36);

	// Default close LED
	if (default_ch_led)
		asus_led_set(&asus_led, LED_OFF);
	else
		asus_led_set(&asus_led, 3);

	/*
	device_create_file(&rpdev->dev, &dev_attr_early_suspend);
	register_early_suspend(&intel_kpd_led_suspend_desc);
	register_early_suspend_device(&rpdev->dev);
	*/

	//ASUS BSP Deeo : MircoP notifer +++
	#ifdef CONFIG_EEPROM_PADSTATION
	if( !microp_notify )
	{
		printk("[LED] register MicroP notifier\n");
		microp_notify = 1;
		register_microp_notifier(&led_mp_notifier);
		//notify_register_microp_notifier(&led_mp_notifier, "asus_led");
	}
	#endif
	//ASUS BSP Deeo : MircoP notifer  ---

	printk("[Progress][LED] ends\n");
	return 0;

fail_id_check:
	return ret;
}

static void asus_led_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	asus_led_set(&asus_led, LED_OFF);
/*
	device_remove_file(&rpdev->dev, &dev_attr_early_suspend);
	unregister_early_suspend(&intel_kpd_led_suspend_desc);
	unregister_early_suspend_device(&rpdev->dev);
*/
	led_classdev_unregister(&asus_led);
	led_classdev_unregister(&dummy_led);

	/*ASUS BSP Deeo : MircoP notifer +++ */
	#ifdef CONFIG_EEPROM_NUVOTON
	if( !microp_notify )
	{
		printk("[LED] unregister MicroP notifier\n");
		microp_notify = 0;
		unregister_microp_notifier(&led_mp_notifier);
		//notify_unregister_microp_notifier(&led_mp_notifier, "asus_led");
	}
	#endif
	/*ASUS BSP Deeo : MircoP notifer Deeo ---*/

}

static void asus_led_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}
static struct rpmsg_device_id asus_led_id_table[] = {
	{ .name	= "rpmsg_asus_led" },
	{ },
};

static struct rpmsg_driver asus_led_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= asus_led_id_table,
	.probe		= asus_led_rpmsg_probe,
	.callback	= asus_led_rpmsg_cb,
	.remove		= asus_led_rpmsg_remove,
};

static int __init asus_led_rpmsg_init(void)
{
	return register_rpmsg_driver(&asus_led_rpmsg);
}

module_init(asus_led_rpmsg_init);

static void __exit asus_led_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&asus_led_rpmsg);
}
module_exit(asus_led_rpmsg_exit);

MODULE_DESCRIPTION("ASUS PF450CL LED Driver");
MODULE_LICENSE("GPL v2");
