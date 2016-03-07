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
 
 /******************************/
/* IR Sensor Property Module */
/*****************************/
#define MODULE_NAME	"property"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/input/proximity_class.h>
#include "property.h"
#include "../IRsensor.h"

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef IR_PROPERTY_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif

#define log(fmt, args...) printk(KERN_DEBUG "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)

/*****************************/
/*Proximity Sensor Property */
/****************************/
static enum proximity_property proxmdev_properties[] = {
    /* int */
    SENSORS_PROP_INTERVAL,
    SENSORS_PROP_HI_THRESHOLD,
    SENSORS_PROP_LO_THRESHOLD,
    SENSORS_PROP_MAXRANGE,      		/* read only */
    SENSORS_PROP_RESOLUTION,    		/* read only */
    SENSORS_PROP_VERSION,       			/* read only */
    SENSORS_PROP_CURRENT,       			/* read only */
    SENSORS_PROP_DBG, 					/* Add for debug only */
    /* char */
    SENSORS_PROP_SWITCH,
    SENSORS_PROP_VENDOR,        			/* read only */
    SENSORS_PROP_ADC,           				/* adc raw data */
    SENSORS_PROP_ATD_STATUS     		/* for atd mode only */
};

static int proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	int ret=0;
	switch (property)	{
	case SENSORS_PROP_SWITCH:		
		val->intval = proximity_get_switch();
		dbg("Proximity get switch = %d.\n", val->intval);
		break;	
		
	case SENSORS_PROP_ADC:
		val->intval = proximity_get_adc();
		dbg("Proximity get adc property: %d\n", val->intval);
		break;
		
	case SENSORS_PROP_VENDOR:
		dbg("SENSORS_PROP_VENDOR.\n");
		sprintf(val->strval, "Capella");
		break;

	case SENSORS_PROP_VERSION:
		printk("SENSORS_PROP_VERSION.\n");
		val->intval = 1;
		break;
		
	case SENSORS_PROP_DBG:		
		IRsensor_show_allreg();
		val->intval = 0;
		break;

	case SENSORS_PROP_ATD_STATUS:
		ret = proximity_atd_test();
		if(0 == ret)
			val->intval = 1;
		else
			val->intval = 0;
		dbg("Proximity get ATD status: %d\n", val->intval);
		break;
		
	default:
		err("Proximity get property ERROR.\n");
		return -EINVAL;
	}
	return 0;
}

static int proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{		
	switch (property)	{

	/* Turn on/off */
	case SENSORS_PROP_SWITCH:
		proximity_switch_onoff(val->intval);
		break;	
	/* Debug Mode */
	case SENSORS_PROP_DBG:
		if (val->intval == 1) {
			proximity_set_polling_mode(true);						
		}else if (val->intval == 2) {
			proximity_set_polling_mode(false);						
		}

		break;
		
	default:
		err("Proximity get property ERROR.\n");
		return -EINVAL;
	}
	return 0;
}

static int proxmdev_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	dbg("Proximity device open.\n");

	if (file->f_flags & O_NONBLOCK)
		dbg("Proximity device open (O_NONBLOCK)\n");

	ret = nonseekable_open(inode, file);
	return ret;
}

static struct file_operations proxmdev_fops = {
    .owner = THIS_MODULE,
    .open = proxmdev_open,
};

struct proximity_class_dev proxmdev_property = {
	.id = SENSORS_PROXIMITY,
	.name = "psensor",
	.num_properties = ARRAY_SIZE(proxmdev_properties),
	.properties = proxmdev_properties,
	.get_property = proxmdev_get_property,
	.put_property = proxmdev_put_property,
	.fops = &proxmdev_fops
};

/*************************/
/* Light Sensor Property */
/************************/
static enum proximity_property lightdev_properties[] = {
	/* int */
	SENSORS_PROP_INTERVAL,
	SENSORS_PROP_HI_THRESHOLD,
	SENSORS_PROP_LO_THRESHOLD,
	SENSORS_PROP_MAXRANGE,      		/* read only */
	SENSORS_PROP_RESOLUTION,    		/* read only */
	SENSORS_PROP_VERSION,       		/* read only */
	SENSORS_PROP_CURRENT,       		/* read only */
	SENSORS_PROP_DBG,           			/* Add for debug only */
	/* char */
	SENSORS_PROP_SWITCH,
	SENSORS_PROP_VENDOR,        		/* read only */
	SENSORS_PROP_CALIBRATION,   	/* Old_calibration value */
	SENSORS_PROP_ADC,           			/* adc raw data */
	SENSORS_PROP_K_ADC,         			/* adc raw data w/ calibrated */
	SENSORS_PROP_LUX,            			/* lux data (calibrated) */
	SENSORS_PROP_ATD_STATUS,    	/* for atd mode only */
	SENSORS_PROP_ATD_ADC,        		/* for atd mode only */
};

static int lightdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	int ret = 0;
	
	switch (property)	{
	case SENSORS_PROP_SWITCH:		
		val->intval = light_get_switch();
		dbg("Light Sensor get switch = %d.\n", val->intval);
		break;	
		
	case SENSORS_PROP_ADC:
		val->intval = light_get_adc();
		dbg("Light Sensor get adc property: %d\n", val->intval);
		break;
		
	case SENSORS_PROP_VENDOR:
		dbg("SENSORS_PROP_VENDOR.\n");
		sprintf(val->strval, "Capella");
		break;

	case SENSORS_PROP_VERSION:
		printk("SENSORS_PROP_VERSION.\n");
		val->intval = 1;
		break;
	
	case SENSORS_PROP_DBG:		
		IRsensor_show_allreg();
		val->intval = 0;
		break;

	case SENSORS_PROP_ATD_STATUS:
		ret = light_atd_test();
		if(0 == ret)
			val->intval = 1;
		else
			val->intval = 0;
		dbg("Light Sensor get ATD status: %d\n", val->intval);
		break;
		
	default:
		err("Light Sensor get property ERROR.\n");
		return -EINVAL;
	}
	return 0;
}

static int lightdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	switch (property)	{

	/* Turn on/off */
	case SENSORS_PROP_SWITCH:
		light_switch_onoff(val->intval);
		break;
		
	default:
		err("Light Sensor get property ERROR.\n");
		return -EINVAL;
	}
	return 0;
}
static int lightdev_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	dbg("Light Sensor device open.\n");

	if (file->f_flags & O_NONBLOCK)
		dbg("Light Sensor device open (O_NONBLOCK)\n");	

	ret = nonseekable_open(inode, file);
	return ret;
}

static struct file_operations lightdev_fops = {
	.owner = THIS_MODULE,
	.open = lightdev_open,
};

struct proximity_class_dev lightdev_property = {
	.id = SENSORS_LIGHT,
	.name = "lsensor",
	.num_properties = ARRAY_SIZE(lightdev_properties),
	.properties = lightdev_properties,
	.get_property = lightdev_get_property,
	.put_property = lightdev_put_property,
	.fops = &lightdev_fops
};

int IRsensor_property_register(void)
{
	int ret = 0;
	ret = proximity_dev_register(&proxmdev_property);
	if (ret) {
		err("Proxmity dev create sysfile ERROR.\n");
		return -1;
	}
	
	ret = proximity_dev_register(&lightdev_property);
	if (ret) {
		err("Light Sensor dev create sysfile ERROR.\n");
		return -1;
	}
	return 0;
}

int IRsensor_property_unregister(void)
{
	proximity_dev_unregister(&proxmdev_property);
	proximity_dev_unregister(&lightdev_property);
	return 0;
}

