/*
 * platform_hx8528.c: hx8528 platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/ft3x17.h>
#include "platform_ft3x17.h"

void *ft3x17_platform_data(void *info)
{
	static struct focal_i2c_platform_data focal_pdata;

	focal_pdata.abs_x_max			= 720;
	focal_pdata.abs_y_max			= 1280;
	focal_pdata.rst_gpio			= 96 + 66;
	focal_pdata.intr_gpio			= 62;

	return &focal_pdata;
}
