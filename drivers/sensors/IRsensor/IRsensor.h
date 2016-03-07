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
 
#ifndef __LINUX_IRSENSOR_H
#define __LINUX_IRSENSOR_H

#define DRIVER_NAME	"IRsensor"

/****************************/
/* IR Sensor Configuration */
/**************************/
#define IRSENSOR_INT_PS_CLOSE 				(1)
#define IRSENSOR_INT_PS_AWAY     			(2) 
#define IRSENSOR_INT_PS_MASK				(3<< 0)
#define IRSENSOR_INT_ALS           				(4)
#define IRSENSOR_INT_ALS_MASK				(1<< 2)

#define LIGHT_CALVALUE_200LUX_DEFAULT	(200)
#define LIGHT_CALVALUE_1000LUX_DEFAULT	(1000)
#define LIGHT_CALVALUE_SHIFT_DEFAULT	(40)
#define LIGHT_GAIN_ACCURACY_CALVALUE	(100000)
#define LIGHT_CHANGE_LOW_SENSITIVITY 	(10)
#define LIGHT_CHANGE_MID_SENSITIVITY 	(5)
#define LIGHT_CHANGE_HI_SENSITIVITY 		(2)
#define LIGHT_CHANGE_FACTORY_SENSITIVITY 		(0)
#define LIGHT_MAX_THRESHOLD				(65535)
#define LIGHT_MAX_LUX							(20000)
#define LIGHT_TURNON_DELAY_TIME			(250)

#define PROXIMITY_THDL_DEFAULT				(30)
#define PROXIMITY_THDH_DEFAULT			(50)

#define PROXIMITY_GARBAGE_ACCURACY		(10000)
#define PROXIMITY_GARBAGEA_DEFAULT		(10000)
#define PROXIMITY_GARBAGEB_DEFAULT		(0)

/*******************************/
/* IR Sensor Driver Interface */
/*****************************/
/*i2c*/
#include <linux/i2c.h>
extern int IRsensor_probe(struct i2c_client *client, const struct i2c_device_id *id);
extern int IRsensor_remove(struct i2c_client *client);
extern void IRsensor_shutdown(struct i2c_client *client);
extern int IRsensor_suspend(struct i2c_client *client, pm_message_t mesg);
extern int IRsensor_resume(struct i2c_client *client);
extern bool IRsensor_check_probe(void);

/*gpio*/
#include <linux/irq.h>
extern irqreturn_t IRsensor_irq_handler(int irq, void *dev_id);

/*Calibration*/
#include <linux/device.h>
extern ssize_t proximity_show_calibration_hi(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t proximity_store_calibration_hi(struct device *dev, struct device_attribute *attr, 
	const char *buf, size_t count);
extern ssize_t proximity_show_calibration_lo(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t proximity_store_calibration_lo(struct device *dev, struct device_attribute *attr, 
	const char *buf, size_t count);
extern ssize_t light_show_calibration_200lux(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t light_store_calibration_200lux(struct device *dev, struct device_attribute *attr, 
	const char *buf, size_t count);
extern ssize_t light_show_calibration_1000lux(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t light_store_calibration_1000lux(struct device *dev, struct device_attribute *attr, 
	const char *buf, size_t count);
extern ssize_t light_show_adc(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t proximity_show_adc (struct device *dev, struct device_attribute *attr, char *buf);

/*Property*/
extern int 		IRsensor_show_allreg(void);

extern int 		proximity_switch_onoff(bool bOn);
extern int 		proximity_get_adc(void);
extern bool 	proximity_get_switch(void);
extern void 	proximity_set_polling_mode(bool bOn);
extern int 		proximity_atd_test(void);

extern int 		light_switch_onoff(bool bOn);
extern int 		light_get_adc(void);
extern bool 	light_get_switch(void);
extern int 		light_atd_test(void);

#endif
