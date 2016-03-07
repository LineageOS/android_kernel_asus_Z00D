/* 
 * Copyright (C) 2014 ASUSTek Inc.
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

 /*********************************/
/* IR Sensor Calibration Module */
/*******************************/
#define MODULE_NAME	"calibration"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include "calibration.h"
#include "../IRsensor.h"

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef IR_CAL_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif

#define log(fmt, args...) printk(KERN_DEBUG "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)

static DEVICE_ATTR(proxm_calibration_hi, S_IRWXU | S_IRWXG | S_IRWXO,
		   proximity_show_calibration_hi, proximity_store_calibration_hi);

static DEVICE_ATTR(proxm_calibration_lo, S_IRWXU | S_IRWXG | S_IRWXO,
		   proximity_show_calibration_lo, proximity_store_calibration_lo);

static DEVICE_ATTR(light_calibration_200lux, S_IRWXU | S_IRWXG | S_IRWXO,
		   light_show_calibration_200lux, light_store_calibration_200lux);

static DEVICE_ATTR(light_calibration_1000lux, S_IRWXU | S_IRWXG | S_IRWXO,
		   light_show_calibration_1000lux, light_store_calibration_1000lux);

static DEVICE_ATTR(adc, S_IRWXU | S_IRWXG | S_IROTH, light_show_adc, NULL);

static DEVICE_ATTR(proxm, S_IRWXU | S_IRWXG  | S_IROTH, proximity_show_adc, NULL);

static struct attribute *IRsensor_attributes[] = {
#ifdef ASUS_FACTORY_BUILD
	&dev_attr_light_calibration_200lux.attr,
	&dev_attr_light_calibration_1000lux.attr,
	&dev_attr_proxm_calibration_lo.attr,
	&dev_attr_proxm_calibration_hi.attr,
#endif
	&dev_attr_proxm.attr,
	&dev_attr_adc.attr,
	NULL
};

static const struct attribute_group IRsensor_attr_group = {
    	.name = DRIVER_NAME,
	.attrs = IRsensor_attributes,
};

int IRsensor_calibration_register(struct i2c_client *client)
{
	int ret = 0;
	
	ret = sysfs_create_group(&client->dev.kobj, &IRsensor_attr_group);
	if (ret) {
		err("IR sensor sysfs_create_group ERROR.\n");
		return -1;
	}
	
	return 0;
}
int IRsensor_calibration_unregister(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &IRsensor_attr_group);
	return 0;
}
