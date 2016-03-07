/*
 * platform_t4k35.c: t4k35 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_t4k35.h"

extern int Read_HW_ID(void);
static unsigned int HW_ID;
static bool first_open;

static int camera_reset;
static int camera_power;
static int camera_vcm_pd;
static int camera_sensor_2_8v;
static int camera_vemmc1_on;  
static struct regulator *vemmc1_reg;
#define VEMMC1_VAL 2850000  

static int camera_vprog1_on;
static int camera_vprog2_on;
static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
#define VPROG1_VAL 1800000
#define VPROG2_VAL 1200000

#define GP_CORE_012 108
#define GP_CAMERA_1_2_8v "V_2P80_SEN_EN"

static int is_ctp(void)
{
	return INTEL_MID_BOARD(1, PHONE, CLVTP)
	       || INTEL_MID_BOARD(1, TABLET, CLVT);
}

static int is_victoriabay(void)
{
	return INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO)
	       || INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG)
	       || INTEL_MID_BOARD(3, PHONE, CLVTP, RHB, PRO, VVLITE)
	       || INTEL_MID_BOARD(3, PHONE, CLVTP, RHB, ENG, VVLITE)
	       || ((INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, PRO)
		    || INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, ENG))
		   && (SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR1A)   
		       || SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR1B)
		       || SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR20)));
}

static int is_moorefield(void)
{
	return INTEL_MID_BOARD(1, PHONE, MOFD) ||
	       INTEL_MID_BOARD(1, TABLET, MOFD);
}

/*
 * MRFLD VV primary camera sensor - T4K35 platform data
 */

static int t4k35_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (flag) {
		gpio_set_value(camera_reset, 1);
		gpio_set_value(camera_vcm_pd, 1);
		pr_info("%s camera_vcm_pd is high\n", __func__);
		pr_info("%s camera_reset is high\n", __func__);

	} else {
		gpio_set_value(camera_reset, 0);
		pr_info("%s camera_reset is low\n", __func__);
		usleep_range(200, 300);
		gpio_free(camera_reset);
		gpio_set_value(camera_vcm_pd, 0);
		pr_info("%s camera_vcm_pd is low\n", __func__);
		usleep_range(200, 300);
		gpio_free(camera_vcm_pd);
	}

	return 0;
}

static int t4k35_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
#else
	pr_err("t4k35 clock is not set.\n");
	return 0;
#endif
}

static int t4k35_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	int reset_gpio_pin, vcm_pd_gpio_pin;

	if (first_open) {
		camera_reset = -1;
		camera_power = -1;
		camera_vcm_pd = -1;
		camera_sensor_2_8v = -1;
		HW_ID = Read_HW_ID();
		first_open = false;
	}

	reset_gpio_pin = 177;
	vcm_pd_gpio_pin = 175;


	if (camera_reset < 0) {
		gpio_free(reset_gpio_pin);
		ret = camera_sensor_gpio(reset_gpio_pin, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s error\n", GP_CAMERA_0_RESET);
			return ret;
		}
		camera_reset = reset_gpio_pin;
	}
	
	if (camera_vcm_pd < 0) {
		ret = camera_sensor_gpio(vcm_pd_gpio_pin, GP_CAMERA_0_POWER_DOWN,
					GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s error\n", GP_CAMERA_0_POWER_DOWN);
			return ret;
		}
		camera_vcm_pd = reset_gpio_pin;
	}

	pr_info("[t4k35_platform] HW_ID = %d\n", HW_ID);
	if (camera_sensor_2_8v < 0 && HW_ID >= 3) { /*for PR HW change 2.8V*/
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_2_8v,
				 GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s not available.\n", GP_CAMERA_1_2_8v);
			return ret;
		}
		camera_sensor_2_8v = GP_CORE_012;
	}
#if 0
	if (is_moorefield()) {
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
		ret = intel_scu_ipc_msic_vprog1(flag);
		if (ret) {
			pr_err("t4k35 power failed\n");
			return ret;
		}
		ret = intel_scu_ipc_msic_vprog3(flag);
#else
		ret = -ENODEV;
#endif
		if (ret)
			pr_err("t4k35 power failed\n");
		if (flag)
			usleep_range(1000, 1200);

		return ret;
	}
#endif
	if (flag) {
		gpio_set_value(camera_reset, 0);
		usleep_range(200, 300);
		pr_info("%s camera_reset is low\n", __func__);
		/* turn on VCM power 2.85V */
		if (!camera_vemmc1_on) {
			camera_vemmc1_on = 1;
			ret = regulator_enable(vemmc1_reg);
			/* ret = regulator_enable(vemmc1_reg); */
			if (!ret) {
				pr_info("%s enable regulator vemmc1 2.85V\n", __func__);
			} else {
				pr_err("%s Failed to enable regulator vemmc1\n", __func__);
			return ret;
			}
		}
		if (!camera_vprog2_on) {
			camera_vprog2_on = 1;
			ret = regulator_enable(vprog2_reg);
			if (!ret) {
				pr_info("%s enable regulator vprog2 1.2V\n", __func__);
			} else {
				pr_err("%s Failed to enable regulator vprog2\n", __func__);
			return ret;
			}
		}
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			ret = regulator_enable(vprog1_reg);
			if (!ret) {
				pr_info("%s enable regulator vprog1 1.8V\n", __func__);
			} else {
				pr_err("%s Failed to enable regulator vprog1\n", __func__);
				return ret;
			}
		}
		if (camera_sensor_2_8v >= 0) {
			gpio_set_value(camera_sensor_2_8v, 1);
			pr_err("<<< 2.8v = 1\n");
		}
	} else {
		if (camera_sensor_2_8v >= 0) {
			gpio_set_value(camera_sensor_2_8v, 0);
			pr_err("<<< 2.8v = 0\n");
			gpio_free(camera_sensor_2_8v);
			camera_sensor_2_8v = -1;
		}

		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				pr_info("%s disable regulator vprog1 1.8V\n", __func__);
			} else {
				pr_err("%s Failed to disable regulator vprog1\n", __func__);
			return ret;
			}
		}
		if (camera_vprog2_on) {
			camera_vprog2_on = 0;
			ret = regulator_disable(vprog2_reg);
			if (!ret) {
				pr_info("%s disable regulator vprog2 1.2V\n", __func__);
			} else {
				pr_err("%s Failed to disable regulator vprog2\n", __func__);
			return ret;
			}
		}
		/* turn off VCM power 2.85V */
		if (camera_vemmc1_on) {
			camera_vemmc1_on = 0;
			ret = regulator_disable(vemmc1_reg);
			if (!ret) {
				pr_info("%s disable regulator vemmc1 2.85V\n", __func__);
			} else {
				pr_err("%s Failed to disable regulator vemmc1\n", __func__);
			return ret;
			}
		}
	}
	return 0;
}
static int t4k35_platform_init(struct i2c_client *client)
{
	int ret;
	/*if (is_ctp()) {
		vemmc1_reg = regulator_get(&client->dev, "vemmc1");
		if (IS_ERR(vemmc1_reg)) {
			dev_err(&client->dev, "regulator_get failed\n");
			return PTR_ERR(vemmc1_reg);
		}
	}*/
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	}
	vprog2_reg = regulator_get(&client->dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog2_reg);
	}
	ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog2_reg);
	}
	vemmc1_reg = regulator_get(&client->dev, "vemmc1");
	if (IS_ERR(vemmc1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vemmc1_reg);
	}
	ret = regulator_set_voltage(vemmc1_reg, VEMMC1_VAL, VEMMC1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vemmc1_reg);
	}

	return 0;
}

static int t4k35_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);
	regulator_put(vemmc1_reg);

	/*if (is_ctp())
		regulator_put(vemmc1_reg);*/

	return 0;
}

static int t4k35_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_gbrg, flag);
}


static struct camera_sensor_platform_data t4k35_sensor_platform_data = {
	.gpio_ctrl      = t4k35_gpio_ctrl,
	.flisclk_ctrl   = t4k35_flisclk_ctrl,
	.power_ctrl     = t4k35_power_ctrl,
	.csi_cfg        = t4k35_csi_configure,
	.platform_init = t4k35_platform_init,
	.platform_deinit = t4k35_platform_deinit,
};

void *t4k35_platform_data(void *info)
{
	first_open = true;
	return &t4k35_sensor_platform_data;
}
