/*
 * Power button driver for Medfield.
 *
 * Copyright (C) 2010 Intel Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/rpmsg.h>
#include <linux/async.h>
#include <asm/intel_mid_powerbtn.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <linux/PMUtil.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/workqueue.h>



#define DRIVER_NAME "msic_power_btn"

static bool bPowerKeyState;  /*ASUS BSP Austin_T +++ add debug msg */

unsigned int pwr_keycode ;
struct mid_pb_priv {
	struct input_dev *input;
	int irq;
	void __iomem *pb_stat;
	u16 pb_level;
	u16 irq_lvl1_mask;
	bool irq_ack;
};

static inline int pb_clear_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0, mask);
}


/* ASUS BSP Austin_T : time of long pressing power key +++ */
#define TIMEOUT_COUNT 55
static struct work_struct __power_key_pressing_work;
void power_key_pressing_work(struct work_struct *work)
{
	static int power_key_pressing;
	int i;

	if (!power_key_pressing) {
		if (!bPowerKeyState) {
			return;
		}
		power_key_pressing = 1;
		for (i = 1 ; i <= TIMEOUT_COUNT && bPowerKeyState; i++) {
			if (i%10 == 0) {
				printk("side_key:powerkey press time = %d s\n", i/10); }
			if (bPowerKeyState) {
				msleep(100);
			}
		}
		power_key_pressing = 0;
	}
}
/* ASUS BSP Austin_T : time of long pressing power key --- */

int g_keycheck_abort;


static irqreturn_t mid_pb_isr(int irq, void *dev_id)
{
	struct mid_pb_priv *priv = dev_id;
	u8 pbstat;
	/*bool bPowerKeyState;    ASUS BSP Austin_T +++ add debug msg */

	g_keycheck_abort = 1;

	pbstat = readb(priv->pb_stat);
	dev_dbg(&priv->input->dev, "pbstat: 0x%x\n", pbstat);

	/* ASUS BSP freddy ++ add debug msg */
	bPowerKeyState = !(pbstat & priv->pb_level);
	printk("[%s] side_key:keyCode=%d  state=%s \n", priv->input->name, pwr_keycode, bPowerKeyState ? "press" : "release");
	/* ASUS BSP freddy -- add debug msg */

	input_event(priv->input, EV_KEY, pwr_keycode, bPowerKeyState);
	input_sync(priv->input);

	/* ASUS BSP Austin_T +++ add debug msg */
	if (bPowerKeyState) {
		schedule_work(&__power_key_pressing_work);
	}
	/* ASUS BSP Austin_T --- add debug msg */

	/* [ASUS BSP] Cheryl Chen - PF450CL001S - Add Power On/Off & Suspend/Resume Log */
	if (bPowerKeyState) {
		PMTestStart(EPMTestTypePowerKeyResume);
		SetPowerKeyResumeStarted(true);
	}
	/* [ASUS BSP] Cheryl Chen - PF450CL001E */

	return IRQ_WAKE_THREAD;
}

static irqreturn_t mid_pb_threaded_isr(int irq, void *dev_id)
{
	struct mid_pb_priv *priv = dev_id;

	if (priv->irq_ack)
		pb_clear_bits(priv->irq_lvl1_mask, MSIC_PWRBTNM);

	return IRQ_HANDLED;
}

/* ASUS BSP freddy++ add node for side_key SMMI_Test */
/* node: sys/module/intel_mid_powerbtn/parameter/pwrkey_mode */
static int pwrkey_mode ;

static int pwrkeyMode_function(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	int old_val = pwrkey_mode;

	if (ret)
		return ret;

	if (pwrkey_mode > 0xf) {
		pwrkey_mode = old_val;
		return -EINVAL;
	}

	ret = param_set_int(val, kp);

	if (pwrkey_mode == 0) {
		pwr_keycode = KEY_POWER;
		printk("[mid_powerbtn] Normal_Mode! \n");
		printk("[mid_powerbtn] PwrKeyCode = %d\n", pwr_keycode);

	} else if (pwrkey_mode == 1) {
		pwr_keycode = KEY_A;
		printk("[mid_powerbtn] Debug_Mode! \n");
		printk("[mid_powerbtn] PwrKeyCode = %d\n", pwr_keycode);

	}

	return 0;
}

module_param_call(pwrkey_mode, pwrkeyMode_function, param_get_int, &pwrkey_mode, 0644);
/* ASUS BSP freddy-- add node for side_key SMMI_Test */
static int mid_pb_probe(struct platform_device *pdev)
{
	struct input_dev *input;
	struct mid_pb_priv *priv;
	int irq;
	int ret;
	struct intel_msic_power_btn_platform_data *pdata;
	printk("[Progress][mid_powerbtn] Probe starts\n");

	if (pdev == NULL)
		return -ENODEV;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "No power button platform data\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "Probed mid powerbutton devivce\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -EINVAL;

	priv = kzalloc(sizeof(struct mid_pb_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	input = input_allocate_device();
	if (!input) {
		kfree(priv);
		return -ENOMEM;
	}

	priv->input = input;
	priv->irq = irq;
	platform_set_drvdata(pdev, priv);

	input->name = pdev->name;
	input->phys = "power-button/input0";
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

	input_set_capability(input, EV_KEY, KEY_POWER);
	input_set_capability(input, EV_KEY, KEY_A);
	pwr_keycode = KEY_POWER;

	priv->pb_stat = ioremap(pdata->pbstat, MSIC_PB_LEN);
	if (!priv->pb_stat) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register input dev, error %d\n", ret);
		goto out_iounmap;
	}

	priv->pb_level = pdata->pb_level;
	priv->irq_lvl1_mask = pdata->irq_lvl1_mask;

	/* Unmask the PBIRQ and MPBIRQ on Tangier */
	if (pdata->irq_ack) {
		pdata->irq_ack(pdata);
		priv->irq_ack = true;
	}

	ret = request_threaded_irq(priv->irq, mid_pb_isr, mid_pb_threaded_isr,
		IRQF_NO_SUSPEND, DRIVER_NAME, priv);

	if (ret) {
		dev_err(&pdev->dev,
			"unable to request irq %d for power button\n", irq);
		goto out_unregister_input;
	}
	/* SCU firmware might send power button interrupts to IA core before
	 * kernel boots and doesn't get EOI from IA core. The first bit of
	 * MSIC lvl1 mask reg is kept masked, and SCU firmware doesn't send new
	 * power interrupt to Android kernel. Unmask the bit when probing
	 * power button in kernel.
	 */
	pb_clear_bits(priv->irq_lvl1_mask, MSIC_PWRBTNM);
	printk("[Progress][mid_powerbtn] Probe ends\n");

	return 0;

out_unregister_input:
	input_unregister_device(input);
	input = NULL;
out_iounmap:
	iounmap(priv->pb_stat);
fail:
	platform_set_drvdata(pdev, NULL);
	input_free_device(input);
	kfree(priv);
	return ret;
}

static int mid_pb_remove(struct platform_device *pdev)
{
	struct mid_pb_priv *priv = platform_get_drvdata(pdev);

	iounmap(priv->pb_stat);
	free_irq(priv->irq, priv);
	platform_set_drvdata(pdev, NULL);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
static int mid_pb_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	printk("[mid_powerbtn] mid_pb_suspend.\n");

	return 0;
}

static int mid_pb_suspend_noirq(struct device *dev)
{
	if (g_keycheck_abort) {
		printk("[mid_powerbtn] noirq_check: suspend_abort\n");
		return -EBUSY;
	}
	printk("[mid_powerbtn] mid_pb_suspend_noirq done.\n");

       return 0;
}

static const struct dev_pm_ops mid_pb_pm_ops = {
	.suspend	= mid_pb_suspend,
	.suspend_noirq  = mid_pb_suspend_noirq,
};
#endif

static const struct platform_device_id mid_pb_table[] = {
	{"mid_powerbtn", 1},
};

static struct platform_driver mid_pb_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &mid_pb_pm_ops,
#endif
	},
	.probe	= mid_pb_probe,
	.remove	= mid_pb_remove,
	.id_table = mid_pb_table,
};

static int __init mid_pb_module_init(void)
{
	return platform_driver_register(&mid_pb_driver);
}

static void  mid_pb_module_exit(void)
{
	platform_driver_unregister(&mid_pb_driver);
}

/* RPMSG related functionality */

static int mid_pb_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;
	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed mid_pb rpmsg device\n");

	ret = mid_pb_module_init();
out:
	return ret;
}


static void mid_pb_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	mid_pb_module_exit();
	dev_info(&rpdev->dev, "Removed mid_pb rpmsg device\n");
}

static void mid_pb_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id mid_pb_id_table[] = {
	{ .name	= "rpmsg_mid_powerbtn" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, mid_pb_id_table);


static struct rpmsg_driver mid_pb_rpmsg_driver = {
	.drv.name	= DRIVER_NAME,
	.drv.owner	= THIS_MODULE,
	.probe		= mid_pb_rpmsg_probe,
	.callback	= mid_pb_rpmsg_cb,
	.remove		= mid_pb_rpmsg_remove,
	.id_table	= mid_pb_id_table,
};

static int __init mid_pb_rpmsg_init(void)
{
	/* ASUS BSP Austin_T +++ add power key timer workqueue */
	INIT_WORK(&__power_key_pressing_work, power_key_pressing_work);
	/* ASUS BSP Austin_T --- add power key timer workqueue */

	return register_rpmsg_driver(&mid_pb_rpmsg_driver);
}

static void __exit mid_pb_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&mid_pb_rpmsg_driver);
}

module_init(mid_pb_rpmsg_init);

module_exit(mid_pb_rpmsg_exit);

MODULE_AUTHOR("Hong Liu <hong.liu@intel.com>");
MODULE_DESCRIPTION("Intel Medfield Power Button Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
