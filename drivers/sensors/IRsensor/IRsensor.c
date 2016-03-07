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

  /*
   * Interrupt Design
   * ================
   * There are 3 situations which will change the PS_INT and ALS_INT_EN.
   * 		1.  init_hw() : disable proximity and light sensor interrupt.
   *			proximity_turn_onoff(0)
   *			light_turn_onoff(0)
   * 		2. proximity_turn_onoff()
   * 		3. light_turn_onoff()
   */

  /*
   * Enable/Disable IRQ Design
   * ========================
   * There are 2 situations which will enable/disable IRQ.
   * 		1.  proximity_turn_onoff() and light_turn_onoff().
   * 		2. Interrupt Service Routine.
   */

   /*
    * Mutex Design
    * =============
    * We use BIG mutex to lock the two critical sections,
    * 	1. switch ON/OFF.
    *		2. IR sensor ISR.
    */

  /*
   * Wake lock Design (in ISR)
   * ================
   *	wake_lock : after disable_irq
   * wake_unlock : before enable_irq
   */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
//#include <asm/intel-mid.h>

#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include "IRsensor.h"
#include "sysfs/sysfs.h"
#include "hardware/hardware.h"
#include "report/report.h"
#include "calibration/calibration.h"
#include "property/property.h"
#include "i2c/i2c.h"
#include "gpio/gpio.h"

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef IR_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s] "fmt,DRIVER_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif

#define log(fmt, args...) printk(KERN_DEBUG "[%s] "fmt,DRIVER_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] "fmt,DRIVER_NAME,##args)

/*******************************/
/* ALS and PS data structure */
/******************************/
struct ASUS_light_sensor_data 
{	
	int g_als_threshold_lo;					/* Lightsensor setting low threshold(adc) */
	int g_als_threshold_hi;					/* Lightsensor setting high threshold(adc) */
	int g_als_200lux_calvalue;				/* Lightsensor 200lux calibration value(adc) */
	int g_als_1000lux_calvalue;				/* Lightsensor 1000lux calibration value(adc) */
	int g_als_shift_calvalue;					/* Lightsensor Shift calibration value */	
	int g_als_accuracy_calvalue;				/* Lightsensor Accuracy calibration value */

	bool HAL_switch_on;						/* this var. means if HAL is turning on als or not */
	bool Device_switch_on;					/* this var. means if als hw is turn on or not */
	bool pad_mode;							/* For phone plug in pad issue */		
	
};

struct ASUS_proximity_sensor_data 
{	
	int g_ps_threshold_lo;					/* Proximitysensor setting low threshold(adc) */
	int g_ps_threshold_hi;					/* Proximitysensor setting high threshold(adc) */	

	bool HAL_switch_on;						/* this var. means if HAL is turning on ps or not */
	bool Device_switch_on;					/* this var. means is turning on ps or not */
	bool pad_mode;							/* For phone plug in pad issue */
	bool polling_mode;							/* Polling for adc of proximity */

	int garbageA;
	int garbageB;
};

/******************************/
/* IR Sensor Global Variables */
/*****************************/
static int ASUS_IR_SENSOR_IRQ;
static struct ASUS_light_sensor_data			*g_als_data;
static struct ASUS_proximity_sensor_data	*g_ps_data;
static struct IRsensor_hw						*IRsensor_hw_client;
static struct workqueue_struct 					*IRsensor_workqueue;
static struct workqueue_struct 					*IRsensor_delay_workqueue;
static struct mutex 								g_ir_lock;
static struct wake_lock 							g_ir_wake_lock;

static bool ASUS_IR_SENSOR_PROBE = false;
static int ASUS_IR_SECOND_SOURCE = 0;

/***********************/
/* IR Sensor Functions*/
/**********************/
extern void ftxxxx_disable_touch(bool flag);
extern int get_audiomode(void);
/*Device Layer Part*/
static int 	proximity_turn_onoff(bool bOn);
static int 	proximity_set_threshold(void);
static void proximity_polling_adc(struct work_struct *work);
static int 	light_turn_onoff(bool bOn);
static int 	light_get_lux(int adc);
static int 	light_get_shift(void);
static int 	light_get_accuracy_gain(void);

/*Interrupt Service Routine Part*/
static void proximity_work(int state);
static void light_work(void);
static void IRsensor_ist(struct work_struct *work);

/*Initialization Part*/
static int init_data(void);

/*Work Queue*/
static 		DECLARE_WORK(IRsensor_ist_work, IRsensor_ist);
static 		DECLARE_DELAYED_WORK(proximity_polling_adc_work, proximity_polling_adc);

/*=======================
 *|| I2c Stress Test Part ||
 *=======================
 */
#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int IRsensor_I2C_stress_test(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;	
	int ret = 0;
	int adc = 0;
	int low_threshold = 0;
	int high_threshold = 0;

	i2c_log_in_test_case("TestIRSensorI2C ++\n");

	/* Turn on Proximity and Light Sensor */
	if(!g_ps_data->Device_switch_on) {
		proximity_turn_onoff(true);
	}
	if(!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}

	/* Proximity i2c read test */
	ret = IRsensor_hw_client->proximity_hw_get_adc();
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to get adc\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Proximity i2c write test */
	ret = IRsensor_hw_client->proximity_hw_set_hi_threshold(g_ps_data->g_ps_threshold_hi);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set high threshold.\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}
	ret = IRsensor_hw_client->proximity_hw_set_lo_threshold(g_ps_data->g_ps_threshold_lo);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set low threshold. \n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Light Sensor i2c read test */
	adc = IRsensor_hw_client->light_hw_get_adc();
	if(adc < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get adc\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	/* Light Sensor Low Threshold */	
	low_threshold = adc * (100 - LIGHT_CHANGE_MID_SENSITIVITY) / 100;

	/* Light Sensor High Threshold */
	high_threshold = adc * (100 + LIGHT_CHANGE_MID_SENSITIVITY) / 100;	
	if (high_threshold > LIGHT_MAX_THRESHOLD)	
		high_threshold = LIGHT_MAX_THRESHOLD;	
	
	ret = IRsensor_hw_client->light_hw_set_hi_threshold(high_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set high threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	
	ret = IRsensor_hw_client->light_hw_set_lo_threshold(low_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set low threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	
	i2c_log_in_test_case("TestLSensorI2C --\n");
	return lnResult;
}

static struct i2c_test_case_info IRSensorTestCaseInfo[] =	{
	__I2C_STRESS_TEST_CASE_ATTR(IRsensor_I2C_stress_test),
};
#endif

/*========================
 *|| Driver Interface Part ||
 *========================
 */ 
 /*Calibration*/
ssize_t proximity_show_calibration_hi(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int calvalue;	
	calvalue = proximity_sysfs_read_high();	
	dbg("Proximity show High Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}

ssize_t proximity_store_calibration_hi(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;

	if ((strict_strtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;	
	if(calvalue < 0) {
		err("Proximity store High Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;	
	}

	log("Proximity store High Calibration: %lu\n", calvalue);
	proximity_sysfs_write_high(calvalue);
	proximity_set_threshold();
	
	return calvalue;
}

ssize_t proximity_show_calibration_lo(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int calvalue;	
	calvalue = proximity_sysfs_read_low();	
	dbg("Proximity show Low Calibration: %d\n", calvalue);
	return sprintf(buf, "%d\n", calvalue);
}

ssize_t proximity_store_calibration_lo(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;

	if ((strict_strtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;	
	if(calvalue < 0) {
		err("Proximity store Low Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;	
	}

	log("Proximity store Low Calibration: %lu\n", calvalue);
	proximity_sysfs_write_low(calvalue);
	proximity_set_threshold();
	
	return calvalue;
}

ssize_t light_show_calibration_200lux(struct device *dev, struct device_attribute *attr, char *buf)
{
	int gainvalue;
	gainvalue = light_get_accuracy_gain();
	dbg("Light Sensor show Gain Calibration: %d.%d\n", 
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);
	return sprintf(buf, "%d.%d\n", 
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);
}

ssize_t light_store_calibration_200lux(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;
	int adc = 0;
	int lux = 0;

	if ((strict_strtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;	
	if(calvalue < 0) {
		err("Light Sensor store 200 lux Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;	
	}
	
	log("Light Sensor store 200 lux Calibration: %lu\n", calvalue);
	light_sysfs_write_200lux(calvalue);
	g_als_data->g_als_200lux_calvalue = calvalue;		
	light_get_shift();

	/* Light Sensor Report input event*/
	adc = IRsensor_hw_client->light_hw_get_adc();
	lux = light_get_lux(adc);
	log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	light_report_lux(lux);	
	
	return calvalue;
}

ssize_t light_show_calibration_1000lux(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int shiftvalue;
	shiftvalue = light_get_shift();
	dbg("Light Sensor show Shift Calibration: %d\n", shiftvalue);
	return sprintf(buf, "%d\n", shiftvalue);
}	

ssize_t light_store_calibration_1000lux(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long calvalue;
	int adc = 0;
	int lux = 0;

	if ((strict_strtoul(buf, 10, &calvalue) < 0))
		return -EINVAL;	
	if(calvalue < 0) {
		err("Light Sensor store 1000 lux Calibration with NEGATIVE value. (%lu) \n", calvalue);
		return -EINVAL;	
	}

	log("Light Sensor store 1000 lux Calibration: %lu\n", calvalue);
	light_sysfs_write_1000lux(calvalue);
	g_als_data->g_als_1000lux_calvalue = calvalue;
	light_get_shift();

	/* Light Sensor Report input event*/
	adc = IRsensor_hw_client->light_hw_get_adc();
	lux = light_get_lux(adc);
	log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	light_report_lux(lux);	
			
	return calvalue;
}

ssize_t light_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	adc = IRsensor_hw_client->light_hw_get_adc();
	return sprintf(buf, "%d\n", adc);
}

ssize_t proximity_show_adc (struct device *dev, struct device_attribute *attr, char *buf)
{
	int adc = 0;
	adc = IRsensor_hw_client->proximity_hw_get_adc();
	return sprintf(buf, "%d\n", adc);
}

/*Property*/
int IRsensor_show_allreg(void)
{	
	return IRsensor_hw_client->IRsensor_hw_show_allreg();	
}

int proximity_switch_onoff(bool bOn)
{
	mutex_lock(&g_ir_lock);
	dbg("Proximity switch = %d.\n", bOn);		
	if ((g_ps_data->Device_switch_on != bOn))	{						
		if (bOn == true)	{
			/* Turn on Proxomity */
			g_ps_data->HAL_switch_on = true;
			proximity_turn_onoff(true);
			/* send the init value */
			proximity_report_onoff(true);
		} else	{
			/* Turn off Proxomity */
			g_ps_data->HAL_switch_on = false;				
			proximity_turn_onoff(false);	
			ftxxxx_disable_touch(false);
		}			
	}
	mutex_unlock(&g_ir_lock);
	return 0;
}

int 	proximity_get_adc(void)
{
	int adc = 0;
	adc = IRsensor_hw_client->proximity_hw_get_adc();
	return adc;
}

bool proximity_get_switch(void)
{
	return g_ps_data->Device_switch_on;
}
int proximity_atd_test(void)
{
	int ret=0;
	int round=0;
	
	ret = proximity_turn_onoff(true);
	if(ret < 0){
		err("Proximity ATD test turn on ERROR\n");
		return ret;
	}	
	
	for(;round<5; round++){
		ret = IRsensor_hw_client->proximity_hw_get_adc();
		if(ret < 0){
			err("Proximity ATD test get adc ERROR\n");
			return ret;
		}
		msleep(100);
	}	

	ret = proximity_turn_onoff(false);
	if(ret < 0){
		err("Proximity ATD test turn off ERROR\n");
		return ret;
	}
	return 0;
}

void proximity_set_polling_mode(bool bOn)
{
	g_ps_data->polling_mode = bOn;
}

int light_switch_onoff(bool bOn)
{
	int adc = 0;
	int lux = 0;
	
	mutex_lock(&g_ir_lock);
	dbg("Light Sensor switch = %d.\n", bOn);		
	if ((g_als_data->Device_switch_on != bOn)) {					
		if (bOn == true)	{
			/* Turn on Light Sensor */
			g_als_data->HAL_switch_on = true;
			light_turn_onoff(true);

			/* wait for light sensor being ready */
			msleep(LIGHT_TURNON_DELAY_TIME);
			
			/* Light Sensor Report input event*/
			adc = IRsensor_hw_client->light_hw_get_adc();
			lux = light_get_lux(adc);
			log("Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
			light_report_lux(lux);	
		} else	{
			/* Turn off Light Sensor */
			g_als_data->HAL_switch_on = false;				
			light_turn_onoff(false);
			/* Report lux=-1 when turn off */
			light_report_onoff(false);
		}			
	}
	mutex_unlock(&g_ir_lock);
	return 0;
}

int light_get_adc(void)
{
	return IRsensor_hw_client->light_hw_get_adc();	
}

bool light_get_switch(void)
{
	return g_als_data->Device_switch_on;
}

int light_atd_test(void)
{
	int ret=0;
	int round=0;
	
	ret =light_turn_onoff(true);
	if(ret < 0){
		err("Light Sensor ATD test turn on ERROR\n");
		return ret;
	}	
	
	for(; round<5; round++){
		ret = IRsensor_hw_client->light_hw_get_adc();
		if(ret < 0){
			err("Light Sensor ATD test get adc ERROR\n");
			return ret;
		}
		msleep(100);
	}	
	
	ret = light_turn_onoff(false);
	if(ret < 0){
		err("Light Sensor ATD test turn off ERROR\n");
		return ret;
	}
	return 0;
}

/*====================
 *|| Device Layer Part ||
 *====================
 */ 
static int proximity_turn_onoff(bool bOn)
{
	int ret = 0;	
	
	if (bOn == 1)	{	/* power on */

		/*Set Proximity Threshold*/
		ret = proximity_set_threshold();
		if (ret < 0) {		
			return ret;
		}
		if(g_ps_data->Device_switch_on == false)
			IRsensor_hw_client->proximity_hw_turn_onoff(true);		
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ASUS_IR_SENSOR_IRQ);
		}
		g_ps_data->Device_switch_on = true;

		/*check the poiing mode*/
		if(g_ps_data->polling_mode == true) {
			queue_delayed_work(IRsensor_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
			log("[Polling] Proximity polling adc START. \n");
		}		
	} else	{	/* power off */	
		if(g_ps_data->Device_switch_on == true)
			IRsensor_hw_client->proximity_hw_turn_onoff(false);
		/*disable IRQ only when proximity is on and light sensor is off*/
		if (g_ps_data->Device_switch_on == true && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
		}
		g_ps_data->Device_switch_on = false;		
	}	
	return 0;
}

static int proximity_set_threshold(void)
{
	int ret = 0;	

	/* Check mapping Garbage A */
	ret = proximity_sysfs_read_garbageA();
	if(ret >= 0) 	    	
	    	g_ps_data->garbageA= ret;

	/* Check mapping Garbage B */
	ret = proximity_sysfs_read_garbageB();
	if(ret >= 0) 	    	
	    	g_ps_data->garbageB= ret;
	
	/*Set Proximity High Threshold*/
	ret = proximity_sysfs_read_high();
	if(ret >= 0) 	    	
	    	g_ps_data->g_ps_threshold_hi = ret;
	
	g_ps_data->g_ps_threshold_hi = 
		(g_ps_data->g_ps_threshold_hi * g_ps_data->garbageA) / PROXIMITY_GARBAGE_ACCURACY + 
		g_ps_data->garbageB;
	
	log("After Garbage Mapping High Threshold : %d\n", g_ps_data->g_ps_threshold_hi);
	IRsensor_hw_client->proximity_hw_set_hi_threshold(g_ps_data->g_ps_threshold_hi);
	
	/*Set Proximity Low Threshold*/
	ret = proximity_sysfs_read_low();	
	if(ret >= 0)  	
	    	g_ps_data->g_ps_threshold_lo = ret;	
	
	g_ps_data->g_ps_threshold_lo = 
		(g_ps_data->g_ps_threshold_lo * g_ps_data->garbageA) / PROXIMITY_GARBAGE_ACCURACY + 
		g_ps_data->garbageB;
	
	log("After Garbage Mapping Low Threshold : %d\n", g_ps_data->g_ps_threshold_lo);
	IRsensor_hw_client->proximity_hw_set_lo_threshold(g_ps_data->g_ps_threshold_lo);
	
	return 0;
}

static void proximity_polling_adc(struct work_struct *work)
{
	int adc = 0;
		
	adc= IRsensor_hw_client->proximity_hw_get_adc();
	log("[Polling] Proximity get adc = %d\n", adc);
	
	if(g_ps_data->Device_switch_on == true)
		queue_delayed_work(IRsensor_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
	else
		log("[Polling] Proximity polling adc STOP. \n");
}

static int light_turn_onoff(bool bOn)
{
	int ret = 0;	
	
	if (bOn == 1)	{	/* power on */	
		
		/* Light Sensor Read Calibration and Shift */
		ret = light_sysfs_read_200lux();
		if(ret >= 0 )
			g_als_data->g_als_200lux_calvalue = ret;
		ret = light_sysfs_read_1000lux();
		if(ret >= 0 )
			g_als_data->g_als_1000lux_calvalue = ret;
		
		light_get_shift();
		log("[Cal] Light Sensor Set Shift calibration value : %d\n", g_als_data->g_als_shift_calvalue);

		if(g_als_data->Device_switch_on == false) {
			IRsensor_hw_client->light_hw_set_hi_threshold(0);
			IRsensor_hw_client->light_hw_set_lo_threshold(0);
			IRsensor_hw_client->light_hw_turn_onoff(true);
		}
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ASUS_IR_SENSOR_IRQ);
		}
		g_als_data->Device_switch_on = true;		
	} else	{	/* power off */	
		if(g_als_data->Device_switch_on == true)
			IRsensor_hw_client->light_hw_turn_onoff(false);
		/*disable IRQ only when proximity is on and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == true) {
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
		}
		g_als_data->Device_switch_on = false;			
	}
	
	return 0;
}

int 	light_get_shift(void)
{
	g_als_data->g_als_shift_calvalue = (1000 - g_als_data->g_als_1000lux_calvalue*800/
				(g_als_data->g_als_1000lux_calvalue-g_als_data->g_als_200lux_calvalue));
	return g_als_data->g_als_shift_calvalue;
}

int 	light_get_accuracy_gain(void)
{
	int gainvalue = 0;
	gainvalue = (800*LIGHT_GAIN_ACCURACY_CALVALUE)/
				(g_als_data->g_als_1000lux_calvalue-g_als_data->g_als_200lux_calvalue);
	
	return gainvalue;
}

/*================================
 *|| Interrupt Service Routine Part ||
 *================================
 */
static void proximity_work(int state)
{
	int adc = 0;

	/* Get Proximity adc value */
	adc= IRsensor_hw_client->proximity_hw_get_adc();
	if(adc < 0){
		err("[ISR] Proximity get adc ERROR\n");		
	}

	if(g_ps_data->HAL_switch_on == true) 
	{
		/* Check proximity close or away. */
		if(IRSENSOR_INT_PS_AWAY == state) {
			log("[ISR] Proximity Detect Object Away. (adc = %d)\n", adc);		
			proximity_report_abs(IRSENSOR_INT_PS_AWAY);
			if(2 == get_audiomode()) {
				ftxxxx_disable_touch(false);
			}
		} else if (IRSENSOR_INT_PS_CLOSE == state) {
			log("[ISR] Proximity Detect Object Close. (adc = %d)\n", adc);		
			proximity_report_abs(IRSENSOR_INT_PS_CLOSE);
			if(2 == get_audiomode()) {
				ftxxxx_disable_touch(true);
			}
		} else {
			err("[ISR] Proximity Detect Object ERROR. (adc = %d)\n", adc);
		}
	}

	
}

static int light_get_lux(int adc)
{
	int lux = 0;

	if(adc < 0) {
		err("Light Sensor get Lux ERROR. (adc < 0)\n");
		return 0;
	}	
	
	lux = (adc * 800/(g_als_data->g_als_1000lux_calvalue-g_als_data->g_als_200lux_calvalue)
			+ g_als_data->g_als_shift_calvalue);	
	
	if(lux > LIGHT_MAX_LUX)
		lux = LIGHT_MAX_LUX;
	if(adc < 10 || lux < 0)
		lux = 0;
	
	return lux;
}

static void light_work(void)
{	
	int low_threshold = 0;
	int high_threshold = 0;	
	int adc = 0;
	int lux = 0;
	int ret = 0;
	int light_change_sensitivity = 0;

	adc = light_get_adc();
	dbg("[ISR] Light Sensor Get adc : %d\n", adc);

	if(adc >= g_als_data->g_als_1000lux_calvalue) {
		light_change_sensitivity = LIGHT_CHANGE_LOW_SENSITIVITY;
	} else if (adc <= g_als_data->g_als_200lux_calvalue) {
		light_change_sensitivity = LIGHT_CHANGE_HI_SENSITIVITY;
	} else {
		light_change_sensitivity = LIGHT_CHANGE_MID_SENSITIVITY;
	}
#ifdef ASUS_FACTORY_BUILD
	light_change_sensitivity = LIGHT_CHANGE_FACTORY_SENSITIVITY;
#endif
	dbg("[ISR] Light Sensor Set Sensitivity. (light_change_sensitivity:%d)\n", light_change_sensitivity);	
	
	/* Light Sensor Low Threshold */	
	low_threshold = adc * (100 - light_change_sensitivity) / 100;

	/* Light Sensor High Threshold */
	high_threshold = adc * (100 + light_change_sensitivity) / 100;	
	if (high_threshold > LIGHT_MAX_THRESHOLD)	
		high_threshold = LIGHT_MAX_THRESHOLD;	
	
	ret = IRsensor_hw_client->light_hw_set_hi_threshold(high_threshold);
	if(ret < 0) {
		err("[ISR] Light Sensor Set High Threshold ERROR. (High:%d)\n", high_threshold);
	}
	dbg("[ISR] Light Sensor Set High Threshold. (High:%d)\n", high_threshold);	

	ret = IRsensor_hw_client->light_hw_set_lo_threshold(low_threshold);
	if(ret < 0) {
		err("[ISR] Light Sensor Set Low Threshold ERROR. (Low:%d)\n", low_threshold);
	}
	dbg("[ISR] Light Sensor Set Low Threshold. (Low:%d)\n", low_threshold);	
	
	/* Light Sensor Report input event*/
	lux = light_get_lux(adc);
	log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	light_report_lux(lux);	
	
}

static void IRsensor_ist(struct work_struct *work)
{
	int ret = 0;	
	int irsensor_int_ps, irsensor_int_als;
	
mutex_lock(&g_ir_lock);

	dbg("IRsensor ist +++ \n");
	/* Read INT_FLAG will clean the interrupt */
	ret = IRsensor_hw_client->IRsensor_hw_get_interrupt();

	/* Check Proximity Interrupt */
	irsensor_int_ps = ret&IRSENSOR_INT_PS_MASK;	
	if(irsensor_int_ps == IRSENSOR_INT_PS_CLOSE || irsensor_int_ps == IRSENSOR_INT_PS_AWAY) 
	{
		dbg("Proximity ist \n");
		if (irsensor_int_ps == IRSENSOR_INT_PS_AWAY) 
			proximity_work(IRSENSOR_INT_PS_AWAY);
		
		if (irsensor_int_ps == IRSENSOR_INT_PS_CLOSE) 
			proximity_work(IRSENSOR_INT_PS_CLOSE);						
	}

	/* Check Light Sensor Interrupt */
	irsensor_int_als = ret&IRSENSOR_INT_ALS_MASK;
	if (irsensor_int_als == IRSENSOR_INT_ALS) {
		dbg("Light Sensor ist \n");
		light_work();
	}
	dbg("IRsensor ist --- \n");
	
	wake_unlock(&g_ir_wake_lock);
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ASUS_IR_SENSOR_IRQ);
mutex_unlock(&g_ir_lock);

}

irqreturn_t IRsensor_irq_handler(int irq, void *dev_id)
{
	dbg("[IRQ] Disable irq !! \n");
	disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
	queue_work(IRsensor_workqueue, &IRsensor_ist_work);
	wake_lock(&g_ir_wake_lock);
	return IRQ_HANDLED;
}

/* || For Proximity check status || */

bool proximity_check_status(void)
{
	int adc_value = 0;
	bool status = false;	

	/* check probe status */
	if(ASUS_IR_SENSOR_PROBE == false)
		return status;

	mutex_lock(&g_ir_lock);
	
	if (!g_ps_data->HAL_switch_on) {
		proximity_turn_onoff(true);
	}
	
	msleep(50);

	adc_value = IRsensor_hw_client->proximity_hw_get_adc();

	if (adc_value >= g_ps_data->g_ps_threshold_hi) {
		status = true;
	}else{ 
		status = false;
	}
	log("proximity_check_status : %s", status?"Close":"Away");
	
	if (!g_ps_data->HAL_switch_on) {
		proximity_turn_onoff(false);
	}
	mutex_unlock(&g_ir_lock);

	return status;
}

EXPORT_SYMBOL(proximity_check_status);

/*====================
 *|| Initialization Part ||
 *====================
 */
static int init_data(void)
{
	int ret = 0;
	/* Reset ASUS_light_sensor_data */
	g_als_data = kmalloc(sizeof(struct ASUS_light_sensor_data), GFP_KERNEL);
	if (!g_als_data)	{
		err("g_als_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_als_data, 0, sizeof(struct ASUS_light_sensor_data));
	g_als_data->Device_switch_on = false;
	g_als_data->HAL_switch_on = false;	
	
	g_als_data->g_als_threshold_hi = 0;
	g_als_data->g_als_threshold_lo = 0;
	g_als_data->g_als_200lux_calvalue = LIGHT_CALVALUE_200LUX_DEFAULT;
	g_als_data->g_als_1000lux_calvalue = LIGHT_CALVALUE_1000LUX_DEFAULT;	
	g_als_data->g_als_shift_calvalue = LIGHT_CALVALUE_SHIFT_DEFAULT;
	
	/* Reset ASUS_proximity_sensor_data */
	g_ps_data = kmalloc(sizeof(struct ASUS_proximity_sensor_data), GFP_KERNEL);
	if (!g_ps_data) {
		err("g_ps_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_ps_data, 0, sizeof(struct ASUS_proximity_sensor_data));
	g_ps_data->Device_switch_on = false;
	g_ps_data->HAL_switch_on = false;	
	
	g_ps_data->g_ps_threshold_hi = PROXIMITY_THDH_DEFAULT;
	g_ps_data->g_ps_threshold_lo = PROXIMITY_THDL_DEFAULT;	
	g_ps_data->garbageA = PROXIMITY_GARBAGEA_DEFAULT;
	g_ps_data->garbageB = PROXIMITY_GARBAGEB_DEFAULT;

	g_ps_data->polling_mode = false;

	/* Initialize the Mutex */
	mutex_init(&g_ir_lock);	

	/* Initialize the wake lock */
	wake_lock_init(&g_ir_wake_lock, WAKE_LOCK_SUSPEND, "IRsensor_wake_lock");
	
	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}

int IRsensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;	

	printk(KERN_INFO "[Progress][%s] Probe starts \n", client->driver->id_table->name);
log("%s Probe +++\n", client->driver->id_table->name);

	/*check i2c client*/
	if (client == NULL) {
		err("i2c Client is NUll\n");
		ret =  -EFAULT;
		goto probe_err;
	}	

	/* Hardware Register Initialization */
	IRsensor_hw_client = IRsensor_hw_getHardware(ASUS_IR_SECOND_SOURCE);
	if(IRsensor_hw_client == NULL) {
		err("Hardware Client is NUll\n");
		ret =  -EFAULT;
		goto probe_err;
	}
	ret = IRsensor_hw_client->IRsensor_hw_init(client);
	if (ret < 0)
		goto probe_err;	

	
	/*link driver data to i2c client*/
	strlcpy(client->name, DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, g_als_data);
	i2c_set_clientdata(client, g_ps_data);	

	/* GPIO */
	ASUS_IR_SENSOR_IRQ = IRsensor_gpio_register(client);
	if (ret < 0)
		goto probe_err;	
	
	/* Calibration */
	ret = IRsensor_calibration_register(client);
	if (ret) {
		err("Calibration Create ERROR.\n");
		goto probe_err;
	}

	/* I2c stress test */
#ifdef CONFIG_I2C_STRESS_TEST	
	i2c_add_test_case(client, "IRSensorTest", ARRAY_AND_SIZE(IRSensorTestCaseInfo));	
#endif
	
log("%s Probe ---\n", client->driver->id_table->name);
	printk(KERN_INFO "[Progress][%s] Probe ends \n", client->driver->id_table->name);
	ASUS_IR_SENSOR_PROBE = true;
	return 0;

probe_err:
	err("%s Probe ERROR ---\n", client->driver->id_table->name);
	printk(KERN_INFO "[Progress][%s] Probe ERROR\n", client->driver->id_table->name);
	ASUS_IR_SENSOR_PROBE = false;
	return ret;
}

bool IRsensor_check_probe(void)
{
	return ASUS_IR_SENSOR_PROBE;
}

int IRsensor_remove(struct i2c_client *client)
{
	log("Driver REMOVE +++\n");

	IRsensor_report_event_unregister();
	destroy_workqueue(IRsensor_workqueue);
	destroy_workqueue(IRsensor_delay_workqueue);
	IRsensor_calibration_unregister(client);
	IRsensor_property_unregister();
	IRsensor_gpio_unregister(ASUS_IR_SENSOR_IRQ);
	wake_lock_destroy(&g_ir_wake_lock);
	mutex_destroy(&g_ir_lock);
	kfree(g_ps_data);
	kfree(g_als_data);
	
	log("Driver REMOVE ---\n");
	
	return 0;
}

void IRsensor_shutdown(struct i2c_client *client)
{
	log("Driver SHUTDOWN +++\n");
	
	/* Disable sensor */
	if (g_als_data->Device_switch_on)
		light_switch_onoff(0);
	if (g_ps_data->Device_switch_on)
		proximity_switch_onoff(0);	
	log("Driver SHUTDOWN ---\n");
	
}

int IRsensor_suspend(struct i2c_client *client, pm_message_t mesg)
{
	log("Driver SUSPEND +++\n");

	/* For keep Proximity can wake_up system */
	if (g_ps_data->Device_switch_on)
		enable_irq_wake(ASUS_IR_SENSOR_IRQ);

	/* For make sure Light sensor mush be switch off when system suspend */
	if (g_als_data->Device_switch_on)				
		light_switch_onoff(0);
	 
	log("Driver SUSPEND ---\n");
	
	return 0;
}

int IRsensor_resume(struct i2c_client *client)
{
	log("Driver RESUME +++\n");
	
	if (g_als_data->Device_switch_on == 0 && g_als_data->HAL_switch_on == 1)
		light_switch_onoff(1);
	
	log("Driver RESUME ---\n");
	
	return 0;
}

static int __init IRsensor_init(void)
{
	int ret = 0;
log("Driver INIT +++\n");
	
	/*driver data structure initialize*/
	ret = init_data();
	if (ret < 0)
		goto init_err;

	/* Work Queue */
	IRsensor_workqueue = create_singlethread_workqueue(DRIVER_NAME"_wq");	
	IRsensor_delay_workqueue = create_singlethread_workqueue(DRIVER_NAME"_delay_wq");	

	/* Property */
	IRsensor_property_register();	
	if (ret < 0)
		goto init_err;
	
	/* Input Device */
	ret = IRsensor_report_event_register();
	if (ret < 0)
		goto init_err;	
	
	/* i2c Registration */	
	for (;ASUS_IR_SECOND_SOURCE < IRsensor_hw_source_max; 
			ASUS_IR_SECOND_SOURCE++) {
				
		ret = IRsensor_i2c_register(ASUS_IR_SECOND_SOURCE);
		if(0 == ret) 
			break;
	}
	if(ASUS_IR_SECOND_SOURCE == IRsensor_hw_source_max) {
		err("There is NO source can Probe.\n");
		goto init_err;
	}
	
	/*Turn off Proximity and Disable Proximity Interrupt */
	ret = proximity_turn_onoff(0);
	if (ret < 0) {		
		goto init_err;
	}	
	
	/*Turn off Light Sensor and Disable Light Sensor Interrupt*/
	ret = light_turn_onoff(0);
	if (ret < 0) {		
		goto init_err;
	}	
		
	log("Driver INIT ---\n");
	return 0;

init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit IRsensor_exit(void)
{
	log("Driver EXIT +++\n");

	/* i2c Unregistration */	
	IRsensor_i2c_unregister();
	
	log("Driver EXIT ---\n");
}

module_init(IRsensor_init);
module_exit(IRsensor_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Proximity and Ambient Light Sensor");
MODULE_LICENSE("GPL");

