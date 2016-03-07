/*
 * platform_gc2155.c: gc2155 platform data initilization file
 *
 * (C) Copyright 2013 ASUSTeK COMPUTER INC
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include "platform_camera.h"
#include "platform_gc2155.h"
#include "platform_mt9e013.h"

extern int Read_HW_ID(void);
static unsigned int HW_ID;
static bool first_open;

static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_vprog2_on;
static int camera_vemmc1_on;
static int camera_sensor_1_8v;
static int camera_sensor_2_8v;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
#define VPROG1_VAL 1800000
#define VPROG2_VAL 1200000
#define GP_CORE_077 173
#define GP_CORE_080 176
#define GP_CORE_012 108

#define GP_CAMERA_1_RESET "SUB_CAM_RST#"
#define GP_CAMERA_1_PDWN "SUB_CAM_PWDN"
#define GP_CAMERA_1_2_8v "V_2P80_SEN_EN"

/*
 * MFLD PR2 secondary camera sensor - gc2155 platform data
 */
static int gc2155_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (first_open) {
		HW_ID = Read_HW_ID();
	}
	pr_info("[gc2155_platform]gc2155_gpio_ctrl HW_ID = %d\n", HW_ID);
	if (camera_reset < 0 && HW_ID >= 3) {
		ret = camera_sensor_gpio(GP_CORE_077, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s not available.\n", GP_CAMERA_1_RESET);
			return ret;
		}
		camera_reset = GP_CORE_077;
	}

	if (camera_power_down < 0 && HW_ID >= 3) {
		ret = camera_sensor_gpio(GP_CORE_080, GP_CAMERA_1_PDWN,
				 GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s not available.\n", GP_CAMERA_1_PDWN);
			return ret;
		}
		camera_power_down = GP_CORE_080;
	}

	if (flag) {
#if 0
		gpio_set_value(camera_reset, 0);
		pr_err("<<< RESETPN = 0\n");
		usleep_range(1000, 1100);
#endif
		if (camera_power_down >= 0) {
			gpio_set_value(camera_power_down, 0);
			pr_err("<<< PWDN = 0\n");
		}
		if (camera_reset >= 0) {
			gpio_set_value(camera_reset, 1);
			pr_err("<<< RESETPN = 1\n");
		}
	} else {
		if (camera_power_down >= 0) {
			gpio_set_value(camera_power_down, 1);
			pr_err("<<< PWDN = 1\n");
			if (HW_ID >= 3) {  /*only free gpio after pr*/
				gpio_free(camera_power_down);
				camera_power_down = -1;
			}
		}
		if (camera_reset >= 0) {
			gpio_set_value(camera_reset, 0);
			pr_err("<<< RESETPN = 0\n");
			if (HW_ID >= 3) {   /*only free gpio after pr*/
				gpio_free(camera_reset);
				camera_reset = -1;
			}
		}

#if 0
		if (camera_reset >= 0) {
			gpio_set_value(camera_reset, 0);
			pr_err("<<< RESETPN = 0\n");
			gpio_free(camera_reset);
			camera_reset = -1;
			usleep_range(10000, 11000);
		}
#endif
	}

	return 0;
}

static int gc2155_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int gc2155_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err, ret;

#if 0
	if (camera_sensor_1_8v < 0) {
		ret = camera_sensor_gpio(GP_AON_052, GP_CAMERA_CAM2_1V8_EN,
				 GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s not available.\n", GP_CAMERA_CAM2_1V8_EN);
			return ret;
		}
		camera_sensor_1_8v = GP_AON_052;
	}

	if (camera_sensor_2_8v < 0) {
		ret = camera_sensor_gpio(GP_CORE_014, GP_CAMERA_2_8V ,
					 GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s not available.\n", GP_CAMERA_2_8V);
			return ret;
		}
		camera_sensor_2_8v = GP_CORE_014;
	}
#endif
	if (first_open) {
		HW_ID = Read_HW_ID();
		pr_info("[gc2155_platform]gc2155_power_ctrl HW_ID = %d\n", HW_ID);
		if (HW_ID >= 3) {
			camera_reset = -1;
			camera_power_down = -1;
			camera_sensor_2_8v = -1;
		} else {
			camera_reset = GP_CORE_077;
			camera_power_down = GP_CORE_080;
			camera_sensor_2_8v = -1;  /*before PR change no 2.8V GPIO*/
		}
		first_open = false;
	}

	if (camera_reset < 0 && HW_ID >= 3) {
		ret = camera_sensor_gpio(GP_CORE_077, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s not available.\n", GP_CAMERA_1_RESET);
			return ret;
		}
		camera_reset = GP_CORE_077;
	}

	if (camera_power_down < 0 && HW_ID >= 3) {
		ret = camera_sensor_gpio(GP_CORE_080, GP_CAMERA_1_PDWN,
				 GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s not available.\n", GP_CAMERA_1_PDWN);
			return ret;
		}
		camera_power_down = GP_CORE_080;
	}

	if (camera_sensor_2_8v < 0 && HW_ID >= 3) { /*for PR HW change 2.8V*/
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_2_8v,
					GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_err("%s not available.\n", GP_CAMERA_1_2_8v);
			return ret;
		}
		camera_sensor_2_8v = GP_CORE_012;
	}

	if (flag) {
#if 0
	    if (!camera_vprog1_on) {
		    ret = regulator_enable(vprog1_reg);
			if (!ret) {
				pr_info("enable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to enable regulator vprog1\n");
				return ret;
			}
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				pr_info("disable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to disable regulator vprog1\n");
				return ret;
			}
		}

		if (!camera_vprog2_on) {
			ret = regulator_enable(vprog2_reg);
			if (!ret) {
				pr_info("enable regulator vprog2 2.8V\n");
			} else {
				pr_err("Failed to enable regulator vprog2\n");
				return ret;
			}
			ret = regulator_disable(vprog2_reg);
			if (!ret) {
				pr_info("disable regulator vprog2 2.8V\n");
			} else {
				pr_err("Failed to disable regulator vprog1\n");
				return ret;
			}
		}
		msleep(100);
#endif

		/* force turn off 1.8V power */
		if (!camera_vprog1_on) {
			ret = regulator_force_disable(vprog1_reg);
			if (!ret) {
				pr_info("disable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to enable regulator vprog1\n");
				return ret;
			}
		}

		/* turn off 2.8V power */
		if (camera_sensor_2_8v >= 0) {
			gpio_set_value(camera_sensor_2_8v, 0);
			pr_err("<<< 2.8v = 0\n");
		}

		if (camera_reset >= 0) {
			gpio_set_value(camera_reset, 0);
			pr_err("<<< RESETPN = 0\n");
		}

		if (camera_power_down >= 0) {
			gpio_set_value(camera_power_down, 1);
			pr_err("<<< PWDN = 1\n");
		}

		usleep_range(1000, 1100);  /* make sure to turn off power

		/* turn on 1.8V power */
		if (!camera_vprog1_on) {
			ret = regulator_enable(vprog1_reg);
			if (!ret) {
				camera_vprog1_on = 1;
				pr_info("enable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to enable regulator vprog1\n");
				return ret;
			}
		}

		usleep_range(1000, 1100);  /*make sure to turn on 2.8V after 1.8V*/
		/* turn on 2.8V power */
		if (camera_sensor_2_8v >= 0) {
			gpio_set_value(camera_sensor_2_8v, 1);
			pr_err("<<< 2.8v = 1\n");
		}

#if 0
		usleep_range(1000, 1100);

		/* PJ turn on 2.8V power */
		if (!camera_vprog2_on) {
			ret = regulator_enable(vprog2_reg);
/*			usleep_range(1000, 1100); */
			if (!ret) {
				camera_vprog2_on = 1;
				pr_info("enable regulator vprog2 2.8V\n");
			} else {
				pr_err("Failed to enable regulator vprog2\n");
				return ret;
			}
			usleep_range(1000, 1100);
		}

		/* PJ turn on 1.8V GPIO */
		if (camera_sensor_1_8v >= 0) {
			gpio_set_value(camera_sensor_1_8v, 1);
			pr_err("<<< 1.8V = 1\n");
			usleep_range(1000, 1100);
		}
		/* PJ turn on 2.8V GPIO */
		if (camera_sensor_2_8v >= 0) {
			gpio_set_value(camera_sensor_2_8v, 1);
			pr_err("<<< AVDD_SENSOR 2.8V = 1\n");
			usleep_range(1000, 1100);
		}
#endif        
	} else {

#if 0
		/* turn off 2.8V power */
		if (camera_vprog2_on) {
			ret = regulator_disable(vprog2_reg);
			if (!ret) {
				camera_vprog2_on = 0;
				pr_info("disable regulator vprog2 2.8V\n");
			} else {
				pr_err("Failed to disable regulator vprog1\n");
				return ret;
			}
		}
#endif

		/* turn off 2.8V power */
		if (camera_sensor_2_8v >= 0) {
			gpio_set_value(camera_sensor_2_8v, 0);
			pr_err("<<< 2.8v = 0\n");
			gpio_free(camera_sensor_2_8v);
			camera_sensor_2_8v = -1;
		}

		usleep_range(1000, 1100);  /*make sure to turn off 1.8V after 2.8V*/
		/* turn off 1.8V power */
		if (camera_vprog1_on) {
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				camera_vprog1_on = 0;
				pr_info("disable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to disable regulator vprog1\n");
				return ret;
			}
		}
#if 0
		/* turn off 2.8V GPIO */
		if (camera_sensor_2_8v >= 0) {
			gpio_set_value(camera_sensor_2_8v, 0);
			pr_err("<<< AVDD_SENSOR 2.8V = 0\n");
			gpio_free(camera_sensor_2_8v);
			camera_sensor_2_8v = -1;
			usleep_range(10000, 11000);
		}
		/* turn off 1.8V GPIO */
		if (camera_sensor_1_8v >= 0) {
			gpio_set_value(camera_sensor_1_8v, 0);
			pr_err("<<< 1.8V = 0\n");
			gpio_free(camera_sensor_1_8v);
			camera_sensor_1_8v = -1;
			usleep_range(10000, 11000);
		}
#endif
	}

	return 0;
}

static int gc2155_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);

/*
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		-1, 0, flag);
*/

/*
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
			ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
*/
}

static int gc2155_platform_init(struct i2c_client *client)
{
	int ret = 0;

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

#if 0
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
#endif
	return ret;
}

static int gc2155_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);

	return 0;
}

static struct camera_sensor_platform_data gc2155_sensor_platform_data = {
	.gpio_ctrl	= gc2155_gpio_ctrl,
	.flisclk_ctrl	= gc2155_flisclk_ctrl,
	.power_ctrl	= gc2155_power_ctrl,
	.csi_cfg	= gc2155_csi_configure,
	.platform_init = gc2155_platform_init,
	.platform_deinit = gc2155_platform_deinit,
};

void *gc2155_platform_data(void *info)
{
	first_open = true;
	return &gc2155_sensor_platform_data;
}

