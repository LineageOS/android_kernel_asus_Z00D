/*
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input/kxtj9.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
// 2012.11.30 cheng_kao early_suspend ++
#include <linux/earlysuspend.h>
// 2012.11.30 cheng_kao early_suspend --

// added by cheng_kao for  HWVersion 2013.11.11 ++
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
// added by cheng_kao for  HWVersion 2013.11.11 --



//#ifdef CONFIG_ME372CG
//#define NAME			"kxtj9"
//#elif defined(CONFIG_ME372CL)
//#ifdef CONFIG_ME372CL
//#define NAME			"kxtj9"
//#else
#define NAME			"kxtj2"
//#endif


#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define CTRL_REG2		0x1D
#define INT_CTRL1		0x1E
#define INT_CTRL2		0x1F
#define DATA_CTRL		0x21
// for enable motion detect , added by cheng_kao 2014.02.12 ++
#define WAKEUP_TIMER	0x29
#define WAKEUP_THRES	0x6A

#define MOTION_DETECT_PERCENT	10
#define MOTION_DETECT_COUNT	25

#define WUFE_ON			(1 << 1)
#define WUFE_OFF		0xFD
#define DRDYE_OFF		0xDF

#define INT_MODE_DRDY	0
#define INT_MODE_WUFE	1

#define WUFE12_5F		4
#define WUFE25F			5
#define WUFE50F			6
#define WUFE100F		7
// for enable motion detect , added by cheng_kao 2014.02.12 --


/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
#define RES_12bit		(1 << 6)
#define RES_8bit			0xBF

/* DATA CONTROL REGISTER BITS */
#define ODR3_125F		0x0A
#define ODR6_25F		0x0B
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F			3
#define ODR200F			4
#define ODR400F			5
#define ODR800F			6
#define ODR1600F		7
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL		(1 << 3)
#define KXTJ9_IEA		(1 << 4)
#define KXTJ9_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3

//use to define raw data by the chip location
#define KXTJ9_CHIP_LOCATION_EVB_ME372CL		0
#define KXTJ9_CHIP_LOCATION_SR_ME372CG		1	/*	X(-1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , -1) */
#define KXTJ9_CHIP_LOCATION_SR_ME372CL		2	/*	X(-1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_ME175CG		3	/*	X(0 , 1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_PF400CG		4	/*	X(-1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_A400CG		5	/*	X(-1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_ER_ME372CL		6	/*	X(0 , 0 , 0 );	Y(0 , -1 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_FE380CG		7	/*	X(0 , 1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_A450CG		8	/*	X(0 , 1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_PF450CL		9	/*	X(1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , 1) */
#define KXTJ2_CHIP_LOCATION_SR_ZC500CL		10	/*	X(0, -1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , 1) */
int g_ilocation=0;

#define WHOAMI_VALUE_FOR_KXTJ9	8
#define WHOAMI_VALUE_FOR_KXTJ2	9

#define RAW_LIMIT_PERCENT			35

#define KXTJ9_VALUE_FOR_NOT_NEED_RESET	0
#define KXTJ9_RESET_FOR_SAME_RAWDATA	1
#define KXTJ9_RESET_FOR_ZERO_RAWDATA	2

// for calibration 2013.11.25 ++
unsigned char  g_kxtj_for_calibration[6]={0};
// for calibration 2013.11.25 --

//define for Camera added by cheng_Kao 2013.10.31 ++
int g_kxtj_for_camera_x =0;
int g_kxtj_for_camera_y =0;
int g_kxtj_for_camera_z =0;
void gsensor_info_for_camera(int *info_x, int *info_y, int *info_z);
//define for Camera added by cheng_Kao 2013.10.31 --

// added by cheng_kao 2013.06.01  for sensors calibration ++
#define GSENSOR_CALIBRATION_FILE_PATH	"/data/sensors/accel_cal_data.ini"
// added by cheng_kao 2013.06.01  for sensors calibration --

extern int build_version;
//extern int g_i2c5_reset_by_gsensor;
bool bypass_for_eng_mode = true ;

// define the GPIO PIN for Intel x86
int gpio_line =  60;
//int i2c_bus =    5;
//module_param(gpio_line, int, S_IRUGO);
//module_param(i2c_bus, int, S_IRUGO);
//Use to open/close the debugmessage
int KXTJ9_DEBUG_MESSAGE=0;
int KXTJ9_REG_MESSAGE=0;
int KXTJ9_CALIBRATED_MESSAGE=1;

#define KXTJ9_RESUME_DISABLE		0
#define KXTJ9_RESUME_ENABLE		1
#define KXTJ9_RESUME_MISSDISABLE	2
#define KXTJ9_RESUME_MISSENABLE	3

//calibration algorithm : used for 6 planes
int g_KXTJ9_CLIBRATION_6_PLANES=0;

// ASUS_BSP +++ Jiunhau_Wang "[PF450CL][Sensor][NA][Spec] Porting 9-axis sensor"
static int KXTJ9_status = 0;
// ASUS_BSP --- Jiunhau_Wang "[PF450CL][Sensor][NA][Spec] Porting 9-axis sensor"

/******
 * 	The following table lists the maximum appropriate poll interval for each available output data rate.
 *	Default Setting :
 *	  kxtj9_odr_table[] = {
 *		{ 1,		ODR1600F },
 *		{ 3,		ODR800F },
 *		{ 5,		ODR400F },
 *		{ 10,	ODR200F },
 *		{ 20,	ODR100F },
 *		{ 40,	ODR50F  },
 *		{ 80,	ODR25F  },
 *		{ 0xFFFFFFFF,	ODR12_5F},
 *	//	{ 0xFFFFFFFF,		ODR3_125F,	KXTJ9_RES_8BIT},				// 320,	251~max	NO POLL
 *	};
 */
#define KXTJ9_RES_8BIT	0
#define KXTJ9_RES_12BIT	1

struct kionix_odr_table {
	unsigned int cutoff;
	u8 mask;
	int RES;
};
static const struct kionix_odr_table kxtj9_odr_table[] = {
															/*  ms,	range,	mode */
	{ 15, 			ODR200F,	KXTJ9_RES_12BIT},			/*  2.5,	6~ 10	FASTEST MODE , full power mode */
	{ 35, 			ODR50F,		KXTJ9_RES_12BIT},			/*  20,	21~30	GAME MODE */
	{ 70, 			ODR25F,		KXTJ9_RES_12BIT},			/*  70,	31~70	UI MODE */
	{ 250,			ODR12_5F,	KXTJ9_RES_12BIT},			/*  160,	71~250	NORMAL MODE */
	{ 0xFFFFFFFF,		ODR12_5F,	KXTJ9_RES_12BIT},			/*  160,	251~max	NO POLL */
};

/******
 * 	kionix kernel driver data struture
 */
struct kxtj9_data {
	struct i2c_client *client;
	struct kxtj9_platform_data pdata;
	struct input_dev *input_dev;
#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
	struct input_polled_dev *poll_dev;
#endif
	unsigned int last_poll_interval;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
// for enable motion detect , added by cheng_kao 2014.02.12 ++
	u8 wufe_rate;
	u8 wufe_timer;
	u8 wufe_thres;
// for enable motion detect , added by cheng_kao 2014.02.12 --
	atomic_t enabled;
// added by cheng_kao 2013.06.01  for sensors calibration ++
	int accel_cal_data[6];
	int accel_cal_offset[3];
	int accel_cal_sensitivity[3];
// added by cheng_kao 2013.06.01  for sensors calibration --
	int suspend_resume_state;
	int resume_enable;
	int irq_status;
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend kxtj9_early_suspendresume;
#endif
	int irq;
// for enable motion detect , added by cheng_kao 2014.02.12 ++
	int motion_detect_threshold_x;
	int motion_detect_threshold_y;
	int motion_detect_threshold_z;
	int motion_detect_timer;
	int chip_interrupt_mode;
// for enable motion detect , added by cheng_kao 2014.02.12 --
};

static int kxtj9_into_drdy_function(struct kxtj9_data *tj9)
{
	int result = 0;
	// turn off power
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (result < 0){
		printk("alp : drdy motion_detect_power_off_fail !\n");
		return result;
	}

	// turn on power
	tj9->ctrl_reg1 |= DRDYE;
	tj9->ctrl_reg1 &= WUFE_OFF;
	if (KXTJ9_DEBUG_MESSAGE)
		printk("alp : kxtj9_into_drdy_function (%d)\n", tj9->ctrl_reg1);
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1); // enable motion detect
	if (result < 0){
		printk("alp : kxtj9_into_drdy_function : into fail !\n");
		return result;
	}
	mdelay(50); // 50 ms

	return result;
}


static int kxtj9_into_motion_detect_function(struct kxtj9_data *tj9)
{
	int result = 0;
	// turn off power
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (result < 0){
		printk("alp : motion_detect_power_off_fail !\n");
		return result;
	}

	// set the chip timer
	result = i2c_smbus_write_byte_data(tj9->client, WAKEUP_TIMER, tj9->wufe_timer );
	if (result < 0){
		printk("alp : set_timer_fail !\n");
		return result;
	}

	// set the chip threshold
	result = i2c_smbus_write_byte_data(tj9->client, WAKEUP_THRES, tj9->wufe_thres);
	if (result < 0){
		printk("alp : set_chip_threshold_fail !\n");
		return result;
	}

	// set the chip wufe rate
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG2, tj9->wufe_rate);
	if (result < 0){
		printk("alp : set_chip_wufe_rate_fail !\n");
		return result;
	}

	// turn on power
	tj9->ctrl_reg1 |= WUFE_ON;
	tj9->ctrl_reg1 &= DRDYE_OFF;
	if (KXTJ9_DEBUG_MESSAGE)
		printk("alp : kxtj9_into_motion_detect_function (%d)\n", tj9->ctrl_reg1);
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1); // enable motion detect
	if (result < 0){
		printk("alp : kxtj9_into_motion_detect_function : into fail !\n");
		return result;
	}
	mdelay(50); // 50 ms

	return result;
}


static int kxtj9_reset_function(struct kxtj9_data *tj9)
{
	int result = 0;
	tj9->ctrl_reg1 &= PC1_OFF;
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG2, 0x84); // SRST set to 1
	if (result < 0){
		printk("alp : kxtj9_reset_function : reset fail (%d)\n",result);
		result = 1;
		return result;
	}
	mdelay(50); // 50 ms

	/* ensure that PC1 is cleared before updating control registers */
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (result < 0){
		result = 3;
		return result;
	}
	/* only write INT_CTRL_REG1 if in irq mode */
	if (tj9->irq) {
		result = i2c_smbus_write_byte_data(tj9->client,INT_CTRL1, tj9->int_ctrl);
		if (result < 0){
			result = 4;
			return result;
		}
	}

	/* turn on outputs */
	tj9->ctrl_reg1 |= PC1_ON;
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (result < 0){
		result = 5;
		return result;
	}

	/* clear initial interrupt if in irq mode */
	if (tj9->irq) {
		result = i2c_smbus_read_byte_data(tj9->client, INT_REL);
		if (result < 0){
			result = 6;
			return result;
		}
	}
	atomic_set(&tj9->enabled, 1);
		
	return result;
}

static int kxtj9_i2c_read(struct kxtj9_data *tj9, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tj9->client->adapter, msgs, 2);
}

static void kxtj9_report_acceleration_data(struct kxtj9_data *tj9)
{
	unsigned char acc_data[6];
	int rawdata_x=0,rawdata_y=0,rawdata_z=0,err=0, ireset=0;
	int reportdata_x=0,reportdata_y=0,reportdata_z=0;
	int thres_x_low=0,thres_x_high=0,thres_y_low=0,thres_y_high=0,thres_z_low=0,thres_z_high=0;
	err = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
	if (err < 0){
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");
		return;
	}
	rawdata_x = ( (acc_data[0]>>4) | (acc_data[1]<<4) );
	rawdata_y = ( (acc_data[2]>>4) | (acc_data[3]<<4) );
	rawdata_z = ( (acc_data[4]>>4) | (acc_data[5]<<4) );

	if( (g_kxtj_for_camera_x==rawdata_x)&&(g_kxtj_for_camera_y==rawdata_y)&&(g_kxtj_for_camera_z==rawdata_z) ){
		printk("alp : the same rawdata!!!\n");
		ireset = KXTJ9_RESET_FOR_SAME_RAWDATA;
	}

	if( (rawdata_x==0)&&(rawdata_y==0)&&(rawdata_z==0) ){
		printk("alp : the zero rawdata!!!\n");
		ireset = KXTJ9_RESET_FOR_ZERO_RAWDATA;
	}

	if(ireset!=KXTJ9_VALUE_FOR_NOT_NEED_RESET){
		err=kxtj9_reset_function(tj9);
		printk("alp :  kxtj9_reset_function : %d\n",err);		
	}

// for calibration 2013.11.25 ++
	g_kxtj_for_calibration[0]=acc_data[0];
	g_kxtj_for_calibration[1]=acc_data[1];
	g_kxtj_for_calibration[2]=acc_data[2];
	g_kxtj_for_calibration[3]=acc_data[3];
	g_kxtj_for_calibration[4]=acc_data[4];
	g_kxtj_for_calibration[5]=acc_data[5];
// for calibration 2013.11.25 --

	if(rawdata_x>2047)
		rawdata_x = rawdata_x-4096;
	if(rawdata_y>2047)
		rawdata_y = rawdata_y-4096;
	if(rawdata_z>2047)
		rawdata_z = rawdata_z-4096;

//	reportdata_x = rawdata_x;
//	reportdata_y = rawdata_y;
//	reportdata_z = rawdata_z;
	
	// transfromed by chip location
	switch(g_ilocation){
		case KXTJ2_CHIP_LOCATION_SR_ZC500CL:
			if ((tj9->accel_cal_sensitivity[0] == 0) || (tj9->accel_cal_sensitivity[1] == 0) || (tj9->accel_cal_sensitivity[2] == 0)) {
				reportdata_x = rawdata_y*(-1);
				reportdata_y = rawdata_x;
				reportdata_z = rawdata_z;
				if (KXTJ9_DEBUG_MESSAGE) {
					printk("alp : LOCATION_PF450CL_SR default\n");
				}
			} else {
				reportdata_x = -1024 * (rawdata_y - tj9->accel_cal_offset[1]) / tj9->accel_cal_sensitivity[1];
				reportdata_y = 1024 * (rawdata_x - tj9->accel_cal_offset[0]) / tj9->accel_cal_sensitivity[0];
				reportdata_z = 1024 * (rawdata_z - tj9->accel_cal_offset[2]) / tj9->accel_cal_sensitivity[2];
				if (KXTJ9_DEBUG_MESSAGE) {
					printk("alp : LOCATION_PF450CL_SR calibration\n");
				}
			}
		break;
	}

	if(KXTJ9_DEBUG_MESSAGE) printk("report_acceleration data : (%d), (%d), (%d)\n",reportdata_x, reportdata_y, reportdata_z);
	g_kxtj_for_camera_x = reportdata_x;
	g_kxtj_for_camera_y = reportdata_y;
	g_kxtj_for_camera_z = reportdata_z;
	input_report_abs(tj9->input_dev, ABS_X, reportdata_x);
	input_report_abs(tj9->input_dev, ABS_Y, reportdata_y);
	input_report_abs(tj9->input_dev, ABS_Z, reportdata_z);
	input_sync(tj9->input_dev);

// for enable motion detect , added by cheng_kao 2014.02.12 ++
	if(!bypass_for_eng_mode){
		if(tj9->motion_detect_timer==-1){
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : motion_detect_timer init!!\n");
			tj9->motion_detect_timer=0;
			tj9->motion_detect_threshold_x = reportdata_x;
			tj9->motion_detect_threshold_y = reportdata_y;
			tj9->motion_detect_threshold_z = reportdata_z;
		}

		thres_x_low = tj9->motion_detect_threshold_x-1024*MOTION_DETECT_PERCENT/100;
		thres_x_high = tj9->motion_detect_threshold_x+1024*MOTION_DETECT_PERCENT/100;
		thres_y_low = tj9->motion_detect_threshold_y-1024*MOTION_DETECT_PERCENT/100;
		thres_y_high = tj9->motion_detect_threshold_y+1024*MOTION_DETECT_PERCENT/100;
		thres_z_low = tj9->motion_detect_threshold_z-1024*MOTION_DETECT_PERCENT/100;;
		thres_z_high = tj9->motion_detect_threshold_z+1024*MOTION_DETECT_PERCENT/100;;
		if(KXTJ9_DEBUG_MESSAGE) printk("alp : thres_x : (%d),(%d) ; thres_y : (%d),(%d) ; thres_z : (%d),(%d)\n",thres_x_low,thres_x_high,thres_y_low,thres_y_high,thres_z_low,thres_z_high);

		if( (reportdata_x>thres_x_low)&&(reportdata_x<thres_x_high) && (reportdata_y>thres_y_low)&&(reportdata_y<thres_y_high) && (reportdata_z>thres_z_low)&&(reportdata_z<thres_z_high) ) {
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : motion_detect_timer count=%d !!\n",tj9->motion_detect_timer);
			tj9->motion_detect_timer++;
		}else{
			tj9->motion_detect_timer=0;
			tj9->motion_detect_threshold_x = reportdata_x;
			tj9->motion_detect_threshold_y = reportdata_y;
			tj9->motion_detect_threshold_z = reportdata_z;
		}
		if(tj9->chip_interrupt_mode==INT_MODE_WUFE){
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : into the drdy method!!\n");
			err=kxtj9_into_drdy_function(tj9);
			if(err<0){
				printk("alp :  kxtj9_into_drdy_function : %d\n",err);	
			}else{
				tj9->chip_interrupt_mode=INT_MODE_DRDY;
				if(KXTJ9_DEBUG_MESSAGE) printk("alp : start drdy mode!!\n");
			}
		}

		if(tj9->motion_detect_timer > MOTION_DETECT_COUNT){
			if(tj9->chip_interrupt_mode==INT_MODE_DRDY){
				if(KXTJ9_DEBUG_MESSAGE) printk("alp : into the motion detect method!!\n");
				err=kxtj9_into_motion_detect_function(tj9);
				if(err<0){
					printk("alp :  kxtj9_into_motion_detect_function : %d\n",err);	
				}else{
					tj9->motion_detect_timer=0;
					tj9->chip_interrupt_mode=INT_MODE_WUFE;
					if(KXTJ9_DEBUG_MESSAGE) printk("alp : start motion detect mode!!\n");
				}	
			}
		}
	}
// for enable motion detect , added by cheng_kao 2014.02.12 --	

}

void gsensor_info_for_camera(int *info_x, int *info_y, int *info_z)
{
	printk("alp : gsensor_info_for_camera \n");
	if(g_kxtj_for_camera_x>1024){
		*info_x = 98;
	}else if(g_kxtj_for_camera_x<-1024){
		*info_x = -98;
	}else{
		*info_x = (g_kxtj_for_camera_x*98)/1024 ;
	}

	if(g_kxtj_for_camera_y>1024){
		*info_y = 98;
	}else if(g_kxtj_for_camera_y<-1024){
		*info_y = -98;
	}else{
		*info_y = (g_kxtj_for_camera_y*98)/1024 ;
	}

	if(g_kxtj_for_camera_z>1024){
		*info_z = 98;
	}else if(g_kxtj_for_camera_z<-1024){
		*info_z = -98;
	}else{
		*info_z = (g_kxtj_for_camera_z*98)/1024 ;
	}

	printk("alp : gsensor_info_for_camera x(%d), y(%d), z(%d) \n",*info_x,*info_y,*info_z);
}
EXPORT_SYMBOL_GPL(gsensor_info_for_camera);

static irqreturn_t kxtj9_isr(int irq, void *dev)
{
	struct kxtj9_data *tj9 = dev;
	int err;

	/* data ready is the only possible interrupt type */
	kxtj9_report_acceleration_data(tj9);
	err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
	if (err < 0){
		printk("alp : kxtj9_isr err(%x)\n",err);
	}

	return IRQ_HANDLED;
}

/*
static int kxtj9_update_g_range(struct kxtj9_data *tj9, u8 new_g_range)
{

	switch (new_g_range) {
	case KXTJ9_G_2G:
		tj9->shift = 4;
		break;
	case KXTJ9_G_4G:
		tj9->shift = 3;
		break;
	case KXTJ9_G_8G:
		tj9->shift = 2;
		break;
	default:
		tj9->shift = 4;
		new_g_range = KXTJ9_G_2G;
//		return -EINVAL;
	}

	tj9->ctrl_reg1 &= 0xe7;
	tj9->ctrl_reg1 |= new_g_range;

	return 0;
}
*/
static int kxtj9_update_odr(struct kxtj9_data *tj9, unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtj9_odr_table); i++) {
		if (poll_interval < kxtj9_odr_table[i].cutoff)
			break;
	}
	tj9->data_ctrl = kxtj9_odr_table[i].mask;
	if (KXTJ9_REG_MESSAGE)	{
		printk("[Gsensor] alp : kxtj9_update_odr  i(%d), cutoff(%d), mask(%d), poll(%d)\n", i, kxtj9_odr_table[i].cutoff, kxtj9_odr_table[i].mask, poll_interval);
		printk("[Gsensor] alp : data_ctrl(%x), ctrl_reg(%x)\n", tj9->data_ctrl, tj9->ctrl_reg1);
	}

	tj9->ctrl_reg1 |=RES_12bit;

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, DATA_CTRL, tj9->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

/*
static int kxtj9_device_power_on(struct kxtj9_data *tj9)
{
	if (tj9->pdata.power_on)
		return tj9->pdata.power_on();
	return 0;
}
*/

static void kxtj9_device_power_off(struct kxtj9_data *tj9)
{
	int err;
	if (KXTJ9_DEBUG_MESSAGE)
		printk("alp : kxtj9_device_power_off ++\n");
	tj9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		dev_err(&tj9->client->dev, "soft power off failed\n");

	if (KXTJ9_DEBUG_MESSAGE)
		printk("alp : kxtj9_device_power_off --\n");
}

static int kxtj9_enable(struct kxtj9_data *tj9)
{
	int err;
	if (KXTJ9_DEBUG_MESSAGE)
		printk("[Gsensor] alp:kxtj9_enable ++\n");

	if (tj9->suspend_resume_state == 1)	{
		printk("[Gsensor] alp : kxtj9_enable  already suspend return !\n");
		tj9->resume_enable=KXTJ9_RESUME_MISSENABLE;
		return 0;
	}

	if (atomic_read(&tj9->enabled) == 1)	{
		printk("[Gsensor] alp : kxtj9_enable  already enable , return 0\n");
		return 0;
	}
	
	if(tj9->irq_status==0){
		enable_irq(tj9->irq);
		tj9->irq_status = 1;
	}

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (tj9->irq) {
		err = i2c_smbus_write_byte_data(tj9->client,INT_CTRL1, tj9->int_ctrl);
		if (err < 0){
			printk("[Gsensor] alp : kxtj9_enable fail 1 irq = %d\n", tj9->irq);
			return err;
		}
		if (KXTJ9_DEBUG_MESSAGE)
			printk("[Gsensor] alp : kxtj9_enable  irq = %d\n", tj9->irq);
	}
//	err = kxtj9_update_g_range(tj9, tj9->pdata.g_range);

	/* turn on outputs */
	if (KXTJ9_DEBUG_MESSAGE)
		printk("[Gsensor] alp : turn on ++\n");
	tj9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (KXTJ9_DEBUG_MESSAGE)
		printk("[Gsensor] alp : turn on -- (%d)\n", err);
	if (err < 0)	{
		printk("[Gsensor] alp : turn on fail -- (%d)\n", err);
		return err;
	}

	err = kxtj9_update_odr(tj9, tj9->last_poll_interval);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
	if (tj9->irq) {
		err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
		if (err < 0) {
			if (KXTJ9_DEBUG_MESSAGE)
				printk("[Gsensor] alp : error clearing interrupt: %d\n", err)	;
			goto fail;
		}
	}
	atomic_set(&tj9->enabled, 1);

	return 0;

fail:
	kxtj9_device_power_off(tj9);
	return err;
}

static void kxtj9_disable(struct kxtj9_data *tj9)
{
	if(atomic_read(&tj9->enabled)==0){
		printk("alp : kxtj9_disable  already disable , return !!\n");
		return ;
	}

	if(tj9->irq_status==1){
		disable_irq(tj9->irq);
		tj9->irq_status = 0;
	}

	kxtj9_device_power_off(tj9);
	atomic_set(&tj9->enabled, 0);
}

static int kxtj9_enable_by_orientation(struct kxtj9_data *tj9)
{
	int err;
	if(KXTJ9_DEBUG_MESSAGE) printk("alp:kxtj9_enable_by_orientation ++\n");
	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (tj9->irq) {
		err = i2c_smbus_write_byte_data(tj9->client,INT_CTRL1, tj9->int_ctrl);
		if (err < 0){
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_enable fail 1 irq = %d\n",tj9->irq);
			return err;
		}
		if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_enable  irq = %d\n",tj9->irq);
	}

	/* turn on outputs */
	tj9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
	if (tj9->irq) {
		err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
		if (err < 0) {
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : error clearing interrupt: %d\n",err)	;
			goto fail;
		}
	}
	return 0;

fail:
	kxtj9_device_power_off(tj9);
	return err;
}

static int kxtj9_input_open(struct input_dev *input)
{
	/*
	struct kxtj9_data *tj9 = input_get_drvdata(input);
	return kxtj9_enable(tj9);
	*/
	return 0;
}

static void kxtj9_input_close(struct input_dev *dev)
{
	struct kxtj9_data *tj9 = input_get_drvdata(dev);

	kxtj9_disable(tj9);
}

//static void __devinit kxtj9_init_input_device(struct kxtj9_data *tj9,
static void kxtj9_init_input_device(struct kxtj9_data *tj9,
					      struct input_dev *input_dev)
{
	if(KXTJ9_DEBUG_MESSAGE) printk("alp :  kxtj9_init_input_device ++\n");
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_SW, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = "kxtj9_accel";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tj9->client->dev;
	if(KXTJ9_DEBUG_MESSAGE) printk("alp :  kxtj9_init_input_device --\n");
}

//static int __devinit kxtj9_setup_input_device(struct kxtj9_data *tj9)
static int kxtj9_setup_input_device(struct kxtj9_data *tj9)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tj9->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tj9->input_dev = input_dev;

	input_dev->open = kxtj9_input_open;
	input_dev->close = kxtj9_input_close;
	input_set_drvdata(input_dev, tj9);

	kxtj9_init_input_device(tj9, input_dev);

	err = input_register_device(tj9->input_dev);
	if (err) {
		dev_err(&tj9->client->dev,
			"unable to register input polled device %s: %d\n",
			tj9->input_dev->name, err);
		input_free_device(tj9->input_dev);
		return err;
	}

	return 0;
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */

// ASUS_BSP +++ Jiunhau_Wang "[PF450][Sensor][NA][Spec] Porting 9-axis sensor"
static ssize_t kxtj9_chip_id_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int retval=0;
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	
	retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
	
	printk("alp : whoami = %d\n", retval);

	if(retval == 0x09)
		printk("alp : use kxtj2!!\n");
	else if(retval == 0x08)
		printk("alp : use kxtj9!!\n");
	else if(retval < 0)
	    printk("alp : read fail!!\n");
			
	return sprintf(buf, "%d\n", retval);
}

static ssize_t kxtj9_status_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", KXTJ9_status);
}

static ssize_t kxtj9_dump_reg(
	struct device *dev, struct device_attribute *attr, char *buf)	
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int i = 0, val = 0;

	for(i = 0; i <0x3b; i++)
	{
		val = i2c_smbus_read_byte_data(tj9->client, i);
		printk("cmd =%02x value = %02x\n", i, val);
	}
	val = i2c_smbus_read_byte_data(tj9->client, 0x6A);
	printk("cmd = 6A value = %02x\n", val);
	
	return sprintf(buf, "%d\n", KXTJ9_status);
}

static ssize_t kxtj9_read_raw(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int retval = 0,rawdata_x=0,rawdata_y=0,rawdata_z=0 ;
	unsigned char acc_data[6];
	static bool user_enable_sensor = false;
	
	retval = atomic_read(&tj9->enabled);

	if (KXTJ9_DEBUG_MESSAGE)
		printk("alp : get_rawdata enable state=(%d)\n", retval);
	if(retval==0){
		retval = kxtj9_enable(tj9);
		mdelay(100);
		if (retval < 0){
			printk("ATTR power on fail\n");
			return retval;
		}
		user_enable_sensor = true;
	}

	retval = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
	if (retval < 0){
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");
		return sprintf(buf, "get no data\n");
	}

	/* PF450 Project */
	/********************
	 * rawdata_x = ( (acc_data[0]>>4) | (acc_data[1]<<4) );
	 * rawdata_y = ( (acc_data[2]>>4) | (acc_data[3]<<4) );
	 * rawdata_z = ( (acc_data[4]>>4) | (acc_data[5]<<4) );
	 */

	/* ZE500CL Project */
	rawdata_x = ((acc_data[2]>>4) | (acc_data[3]<<4));
	rawdata_y = ((acc_data[0]>>4) | (acc_data[1]<<4));
	rawdata_z = ((acc_data[4]>>4) | (acc_data[5]<<4));

	if (rawdata_x > 2047)
		rawdata_x = rawdata_x - 4096;
	if (rawdata_y > 2047)
		rawdata_y = rawdata_y - 4096;
	if (rawdata_z > 2047)
		rawdata_z = rawdata_z - 4096;

	/* ZE500CL Project */
	rawdata_x = rawdata_x*(-1);

	if (user_enable_sensor == true)
	{
		kxtj9_disable(tj9);
		user_enable_sensor = false;
	}

	return sprintf(buf, "%d %d %d\n", rawdata_x, rawdata_y, rawdata_z);
}

static ssize_t kxtj9_r_en_mt(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = (bypass_for_eng_mode == true) ? (0) : (1);

	return sprintf(buf, "en(%d)  bypass(%d) \n", ret, bypass_for_eng_mode);
}

static ssize_t kxtj9_w_en_mt(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);

	if (val == 1) { /* enable mt => disable bypass mode */
		bypass_for_eng_mode = false;
	} else {
		bypass_for_eng_mode = true;
		kxtj9_into_drdy_function(tj9);
	}
	printk(KERN_INFO "[KXTJ9] en(%d)  bypass(%d) \n", val, bypass_for_eng_mode);

	return count;
}

// ASUS_BSP --- Jiunhau_Wang "[PF450][Sensor][NA][Spec] Porting 9-axis sensor"

static ssize_t kxtj9_show_message(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	printk("[Gsensor] alp : kxtj9_message DEBUG(%d), REG(%d), CALI(%d)\n", KXTJ9_DEBUG_MESSAGE, KXTJ9_REG_MESSAGE, KXTJ9_CALIBRATED_MESSAGE);
	return sprintf(buf, "\n[Gsensor] alp : kxtj9_message DEBUG(%d), REG(%d), CALI(%d)\n", KXTJ9_DEBUG_MESSAGE, KXTJ9_REG_MESSAGE, KXTJ9_CALIBRATED_MESSAGE);
}

static ssize_t kxtj9_set_message(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	int val = simple_strtoul(buf, NULL, 10);
	switch(val){
		case 0:
			KXTJ9_DEBUG_MESSAGE=0;
			KXTJ9_REG_MESSAGE=0;
			KXTJ9_CALIBRATED_MESSAGE=0;
			printk("[Gsensor] alp : disable all message !!!\n");
		break;

		case 1:
			KXTJ9_DEBUG_MESSAGE=1;
			KXTJ9_REG_MESSAGE=1;
			KXTJ9_CALIBRATED_MESSAGE=1;
			printk("[Gsensor] alp : enable all message !!!\n");
		break;

		case 2:
			KXTJ9_REG_MESSAGE=1;
			printk("[Gsensor] alp : enable  REG_MESSAGE !!!\n");
		break;

		case 3:
			KXTJ9_CALIBRATED_MESSAGE=1;
			printk("[Gsensor] alp : enable  CALIBRATED_MESSAG !!!\n");
		break;

		case 4:
			KXTJ9_DEBUG_MESSAGE=1;
			printk("[Gsensor] alp : enable  DEBUG_MESSAGE !!!\n");
		break;

		case 5 :
//			g_i2c5_reset_by_gsensor=1;
			printk("[Gsensor] alp : reset  i2c-5 !!!\n");
		break;

		default:
			printk("[Gsensor] alp : error input !!!\n");
		break;
	}
	return count;
}




/* Returns currently selected poll interval (in ms) */
static ssize_t kxtj9_get_poll(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int kxtj9_ctrl = 0;
	kxtj9_ctrl = i2c_smbus_read_byte_data(client, DATA_CTRL);
	printk("alp : kxtj9_get_poll (%d) (%d)\n",tj9->last_poll_interval,kxtj9_ctrl);
	return sprintf(buf, "%d , %d\n", tj9->last_poll_interval,kxtj9_ctrl);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtj9_set_poll(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	unsigned int interval=0;

	interval = simple_strtoul(buf, NULL, 10);
	if (KXTJ9_DEBUG_MESSAGE)
		printk("[Gsensor] alp : interval (%u)\n", interval);
	if (KXTJ9_DEBUG_MESSAGE)
		printk("[Gsensor] alp : kxtj9_set_poll buf (%s)\n", buf);
	if (KXTJ9_DEBUG_MESSAGE)
		printk("[Gsensor] alp : last_poll_interval (%u), New_poll(%d)\n", tj9->last_poll_interval, interval);

	// Lock the device to prevent races with open/close (and itself)
	mutex_lock(&input_dev->mutex);
	 //Set current interval to the greater of the minimum interval or the requested interval
	kxtj9_update_odr(tj9, interval);
	tj9->last_poll_interval = interval;
	mutex_unlock(&input_dev->mutex);

	return count;
}

static ssize_t kxtj9_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	printk("[Gsensor] alp : kxtj9_enable_show (%d)\n", atomic_read(&tj9->enabled));
	return sprintf(buf, "%d\n", atomic_read(&tj9->enabled));
}

static ssize_t kxtj9_enable_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	switch(val){
		case 0:
			printk("[Gsensor] alp : kxtj9_disable_by_acceleation !!!\n");
			kxtj9_disable(tj9);
		break;
		case 1:
			printk("[Gsensor] alp : kxtj9_enable_by_acceleation !!!\n");
			kxtj9_enable(tj9);
		break;
		// handled by orientation added by cheng_kao 2012.12.05 ++
		case 2:
			printk("[Gsensor] alp : kxtj9_disable_by_orientation !!!\n");
			kxtj9_device_power_off(tj9);
		break;
		case 3:
			printk("[Gsensor] alp : kxtj9_enable_by_orientation !!!\n");
			kxtj9_enable_by_orientation(tj9);
		break;
		// handled by orientation added by cheng_kao 2012.12.05 --
	}
	return count;
}

static ssize_t get_kxtj9_state(struct device *dev, struct device_attribute *devattr, char *buf)
{	

	struct i2c_client *client = to_i2c_client(dev);
	int kxtj9_wia = 0 , ret=0;
	kxtj9_wia = i2c_smbus_read_byte_data(client, WHO_AM_I);
	printk("get_kxtj9_state wia : %d\n",kxtj9_wia);

	if( (kxtj9_wia==WHOAMI_VALUE_FOR_KXTJ9) || (kxtj9_wia==WHOAMI_VALUE_FOR_KXTJ2) ) // 8 for kxtj9 ; 9 for kxtj2
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n",ret);

}

static ssize_t get_cal_rawdata(struct device *dev, struct device_attribute *devattr, char *buf)
{
	printk("calibration data : (%d), (%d), (%d)\n",g_kxtj_for_camera_x, g_kxtj_for_camera_y, g_kxtj_for_camera_z);
	return sprintf(buf, "%d %d %d\n",g_kxtj_for_camera_x, g_kxtj_for_camera_y, g_kxtj_for_camera_z);
}

static ssize_t get_rawdata(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	unsigned char acc_data[6];
	int retval = 0, rawdata_x = 0, rawdata_y = 0, rawdata_z = 0;
	unsigned char data[6];
	
	retval = atomic_read(&tj9->enabled);
	printk("alp : get_rawdata enable state=(%d)\n",retval);
	if(retval==0){
		retval = kxtj9_enable(tj9);
		mdelay(100);
		if (retval < 0){
			printk("ATTR power on fail\n");
			return retval;
		}
	}
	printk("get_rawdata rawdata : (%d), (%d), (%d), (%d), (%d), (%d)\n",g_kxtj_for_calibration[0], g_kxtj_for_calibration[1], g_kxtj_for_calibration[2],g_kxtj_for_calibration[3],g_kxtj_for_calibration[4],g_kxtj_for_calibration[5]);

// for calibration 2013.11.25 ++
	data[0]=g_kxtj_for_calibration[0];
	data[1]=g_kxtj_for_calibration[1];
	data[2]=g_kxtj_for_calibration[2];
	data[3]=g_kxtj_for_calibration[3];
	data[4]=g_kxtj_for_calibration[4];
	data[5]=g_kxtj_for_calibration[5];
// for calibration 2013.11.25 --

	retval = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
	if (retval < 0)	{
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");
		return sprintf(buf, "get no data\n");
	}
	printk("get_rawdata new rawdata : (%d), (%d), (%d), (%d), (%d), (%d)\n",
				acc_data[0], acc_data[1], acc_data[2], acc_data[3], acc_data[4], acc_data[5]);

	/* PF450 Project */
	/********************
	 * rawdata_x = ( (acc_data[0]>>4) | (acc_data[1]<<4) );
	 * rawdata_y = ( (acc_data[2]>>4) | (acc_data[3]<<4) );
	 * rawdata_z = ( (acc_data[4]>>4) | (acc_data[5]<<4) );
	 */

	/* ZE500CL Project */
	rawdata_x = ((acc_data[2] >> 4) | (acc_data[3] << 4));
	rawdata_y = ((acc_data[0] >> 4) | (acc_data[1] << 4));
	rawdata_z = ((acc_data[4] >> 4) | (acc_data[5] << 4));

	if (rawdata_x > 2047)
		rawdata_x = rawdata_x - 4096;
	if (rawdata_y > 2047)
		rawdata_y = rawdata_y - 4096;
	if (rawdata_z > 2047)
		rawdata_z = rawdata_z - 4096;

	/* ZE500CL Project */
	rawdata_x = rawdata_x*(-1);

	return sprintf(buf, "%d %d %d\n", rawdata_x, rawdata_y, rawdata_z);

}

static ssize_t reset_kxtj9_calibration(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	char tmp_data[128]={0};
	int retval = 0,index=0,iplane=0,ilen=0, icount=0;
	int up_calibration_limit=0,down_calibration_limit=0;
	int neg_up_calibration_limit=0,neg_down_calibration_limit=0;
	bool bNegative=false;
//	read file data++
	struct file *fp=NULL;
	mm_segment_t old_fs;
//	read file data--

	up_calibration_limit=1024*(100+RAW_LIMIT_PERCENT)/100;
	down_calibration_limit=1024*(100-RAW_LIMIT_PERCENT)/100;
	neg_up_calibration_limit = (-1)*up_calibration_limit;
	neg_down_calibration_limit = (-1)*down_calibration_limit;

//	init acc config 2013.11.25 ++
	for(index=0;index<6;index++){
		tj9->accel_cal_data[index]=0;
	}
//	init acc config 2013.11.25 --

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp=filp_open(GSENSOR_CALIBRATION_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		printk(KERN_INFO "filp_open fail\n");
		retval = -1;
		return retval;
	}
	ilen = fp->f_op->read(fp,tmp_data,128,&fp->f_pos);
	printk(KERN_INFO "KXTJ9_CONFIG_FILE %d\n",ilen);
	printk(KERN_INFO "%s \n",tmp_data);
	set_fs(old_fs);
	filp_close(fp,NULL);

	if(ilen>0){
//	for transfor form file to int ++
		do{
			if(tmp_data[icount]==38){	//	38="&"
				iplane = (tmp_data[icount-2]-48)-1;	// 1: start from 0, 2:start from 1 ; 0:start from -1 -> error!!!
				index =  tmp_data[icount-1]-48;
				if(KXTJ9_DEBUG_MESSAGE) printk(KERN_INFO "iplane(%d), index(%d) \n",iplane ,index );
				if(tmp_data[icount+1]==45){	//	45="-"
					bNegative = true;
					icount++;
				}
				switch(index){
					case 1:
						tj9->accel_cal_data[iplane]= (tmp_data[icount+1]-48);
					break;

					case 2:
						tj9->accel_cal_data[iplane]= ( (tmp_data[icount+1]-48)*10+(tmp_data[icount+2]-48) );
					break;

					case 3:
						tj9->accel_cal_data[iplane]= ( (tmp_data[icount+1]-48)*100+(tmp_data[icount+2]-48)*10+(tmp_data[icount+3]-48) );
					break;

					case 4:
						tj9->accel_cal_data[iplane]= ( (tmp_data[icount+1]-48)*1000+(tmp_data[icount+2]-48)*100+(tmp_data[icount+3]-48)*10+(tmp_data[icount+4]-48) );
					break;

					default:
						printk(KERN_INFO "error index %d \n",index );
					break;
				}
				if(bNegative) tj9->accel_cal_data[iplane] = tj9->accel_cal_data[iplane]*(-1);
			}
			bNegative = false;
			icount++;
		}while(icount<ilen);
//	for transfor form file to int --
		if(KXTJ9_CALIBRATED_MESSAGE) {
			for(iplane=0;iplane<6;iplane++){
				printk(KERN_INFO "iplane[%d]  %d \n",iplane ,tj9->accel_cal_data[iplane] );
			}
		}
		retval=1;
	}

	if( (tj9->accel_cal_data[0]==0) && (tj9->accel_cal_data[2]==0) && (tj9->accel_cal_data[4]==0)  ){
		g_KXTJ9_CLIBRATION_6_PLANES = 0;
		printk(KERN_INFO "alp : use the calibration one plane\n" );
	}else{
		g_KXTJ9_CLIBRATION_6_PLANES = 1;
		printk(KERN_INFO "alp : use the calibration six plane\n" );
	}
	
	if(g_KXTJ9_CLIBRATION_6_PLANES){
		//for X axis
		tj9->accel_cal_sensitivity[0] = (tj9->accel_cal_data[2]-tj9->accel_cal_data[3])/2;
		tj9->accel_cal_offset[0] = tj9->accel_cal_data[3]+tj9->accel_cal_sensitivity[0];
		if( (tj9->accel_cal_sensitivity[0]==0) ||(tj9->accel_cal_sensitivity[0]>up_calibration_limit) || (tj9->accel_cal_sensitivity[0]<down_calibration_limit))
			tj9->accel_cal_sensitivity[0]=1024;

		//for Y axis
		tj9->accel_cal_sensitivity[1] = (tj9->accel_cal_data[4]-tj9->accel_cal_data[5])/2;
		tj9->accel_cal_offset[1] = tj9->accel_cal_data[5]+tj9->accel_cal_sensitivity[1];
		if( (tj9->accel_cal_sensitivity[1]==0) ||(tj9->accel_cal_sensitivity[1]>up_calibration_limit) || (tj9->accel_cal_sensitivity[1]<down_calibration_limit))
			tj9->accel_cal_sensitivity[1]=1024;

		//for Z axis
		tj9->accel_cal_sensitivity[2] = (tj9->accel_cal_data[0]-tj9->accel_cal_data[1])/2;
		tj9->accel_cal_offset[2] = tj9->accel_cal_data[1]+tj9->accel_cal_sensitivity[2];
		if( (tj9->accel_cal_sensitivity[2]==0) ||(tj9->accel_cal_sensitivity[2]>up_calibration_limit) || (tj9->accel_cal_sensitivity[2]<down_calibration_limit))
			tj9->accel_cal_sensitivity[2]=1024;
	}else{
		// for PF400CG, A400CG, A450CG, ME372CL
		if( (g_ilocation == KXTJ2_CHIP_LOCATION_SR_PF400CG) || (g_ilocation == KXTJ2_CHIP_LOCATION_SR_A400CG) ||(g_ilocation == KXTJ2_CHIP_LOCATION_SR_A450CG) ||(g_ilocation == KXTJ2_CHIP_LOCATION_ER_ME372CL) ){
			//for X axis
			tj9->accel_cal_offset[0]=tj9->accel_cal_data[1];
			if( (tj9->accel_cal_offset[0]<up_calibration_limit) && (tj9->accel_cal_offset[0]>down_calibration_limit) )
				tj9->accel_cal_offset[0]=tj9->accel_cal_offset[0]-1024;
			else
				tj9->accel_cal_offset[0] = 0;
			//for Y axis
			tj9->accel_cal_offset[1]=tj9->accel_cal_data[3]; 
			if( (tj9->accel_cal_offset[1]<up_calibration_limit) && (tj9->accel_cal_offset[1]>down_calibration_limit) )
				tj9->accel_cal_offset[1]=tj9->accel_cal_offset[1]-1024;
			else
				tj9->accel_cal_offset[1] = 0;

			//for Z axis
			tj9->accel_cal_offset[2]=tj9->accel_cal_data[5];
			if(tj9->accel_cal_offset[2]>0){
				if( (tj9->accel_cal_offset[2]>up_calibration_limit) ||(tj9->accel_cal_offset[2]<down_calibration_limit) )
					tj9->accel_cal_offset[2]=0;
				else
					tj9->accel_cal_offset[2]=tj9->accel_cal_offset[2]-1024;
			}else{
				if( (tj9->accel_cal_offset[2]<neg_up_calibration_limit) || (tj9->accel_cal_offset[2]>neg_down_calibration_limit) )
					tj9->accel_cal_offset[2]=0;
				else
					tj9->accel_cal_offset[2]=tj9->accel_cal_offset[2]+1024;
			}
		}
		else{
			printk(KERN_INFO "alp : kxtj2 use the default setting!!\n" );
		}
	}

	if(KXTJ9_CALIBRATED_MESSAGE) {
		printk("accel_cal_sensitivity : (%d), (%d), (%d)\n",tj9->accel_cal_sensitivity[0],tj9->accel_cal_sensitivity[1],tj9->accel_cal_sensitivity[2] );
		printk("accel_cal_offset : (%d), (%d), (%d)\n",tj9->accel_cal_offset[0],tj9->accel_cal_offset[1],tj9->accel_cal_offset[2] );
	}
	return retval;
}

static ssize_t read_kxtj9_resolution(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data;
	data = i2c_smbus_read_byte_data(client, CTRL_REG1);
	printk("alp : read_kxtj9_resolution (%d)\n",data);
	return sprintf(buf, "%d\n", data);
}

static ssize_t write_kxtj9_resolution(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	switch(val){
		case 0:
			printk("alp : 8-bit !!!\n");
			tj9->ctrl_reg1 &=RES_8bit;
			i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
		break;
		case 1:
			printk("alp : 12-bit !!!\n");
			tj9->ctrl_reg1 |=RES_12bit;
			i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
		break;

		default:
			printk("alp : error input !!!\n");
		break;
	}
	return count;
}

static ssize_t read_kxtj9_wufe(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[2]={0};
	data[0] = i2c_smbus_read_byte_data(client, WAKEUP_TIMER);
	data[1] = i2c_smbus_read_byte_data(client, WAKEUP_THRES);
	printk("alp : read_kxtj9_wufe time(%d) , thres(%d)\n",data[0],data[1]);
	return sprintf(buf, "%d & %d\n",data[0],data[1]);
}

static ssize_t write_kxtj9_wufe(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	u8 wufe_data[0]={0};
	int result = 0;
	int val = simple_strtoul(buf, NULL, 10);

	// turn off power
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (result < 0){
		printk("alp : motion_detect_power_off_fail !\n");
		return result;
	}
	wufe_data[0] = 0x01; 
	switch(val){
		case 0:
			wufe_data[0] = 0x00; wufe_data[1] = 0x01;
		break;

		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			wufe_data[1] = val;
		break;

		default:
			printk("alp : not support !!!\n");
		break;
	}
	tj9->wufe_timer = wufe_data[0];
	tj9->wufe_thres = wufe_data[1];

	// set the chip timer
	result = i2c_smbus_write_byte_data(tj9->client, WAKEUP_TIMER, wufe_data[0]);
	if (result < 0){
		printk("alp : ATTR set_timer_fail !\n");
		return result;
	}

	// set the chip threshold
	result = i2c_smbus_write_byte_data(tj9->client, WAKEUP_THRES, wufe_data[1]);
	if (result < 0){
		printk("alp : ATTR set_chip_threshold_fail !\n");
		return result;
	}

	// turn on power
	tj9->ctrl_reg1 |= DRDYE;
	tj9->ctrl_reg1 &= WUFE_OFF;
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1); // enable motion detect
	if (result < 0){
		printk("alp : write_kxtj9_wufe into DRDY fail (%d)\n",result);
		return result;
	}
	mdelay(50); // 50 ms
	
	return count;
}

static ssize_t read_kxtj9_reg2_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data;
	data = i2c_smbus_read_byte_data(client, CTRL_REG2);
	printk("alp : read_kxtj9_reg2_rate reg2 = %d\n",data);
	return sprintf(buf, "%d\n",data);
}

static ssize_t write_kxtj9_reg2_rate(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int result = 0;
	int val = simple_strtoul(buf, NULL, 10);
	unsigned char data;

	// turn off power
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (result < 0){
		printk("alp : motion_detect_power_off_fail !\n");
		return result;
	}
	switch(val){
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			data = val;
		break;

		default:
			printk("alp : not support !!!\n");
		break;
	}
	tj9->wufe_rate = data;
	// set the chip wufe rate
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG2, data);
	if (result < 0){
		printk("alp : ATTR write_kxtj9_reg2_rate !\n");
		return result;
	}

	// turn on power
	tj9->ctrl_reg1 |= DRDYE;
	tj9->ctrl_reg1 &= WUFE_OFF;
	result = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1); // enable motion detect
	if (result < 0){
		printk("alp : write_kxtj9_reg2_rate power on fail ! (%d)\n",result);
		return result;
	}
	mdelay(50); // 50 ms
	
	return count;
}

static ssize_t reset_kxtj9(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int res=0;
	kxtj9_reset_function(tj9);
	return sprintf(buf, "%d\n", res);
}


static DEVICE_ATTR(message, 0660, kxtj9_show_message, kxtj9_set_message);
static DEVICE_ATTR(delay, 0660, kxtj9_get_poll, kxtj9_set_poll);
static DEVICE_ATTR(enable, 0660,kxtj9_enable_show,kxtj9_enable_store);
static DEVICE_ATTR(rawdata, S_IRUGO, get_rawdata, NULL);
static DEVICE_ATTR(state, S_IRUGO, get_kxtj9_state, NULL);
static DEVICE_ATTR(cal_rawdata, S_IRUGO, get_cal_rawdata, NULL);
static DEVICE_ATTR(calibration, S_IRUGO, reset_kxtj9_calibration, NULL);
static DEVICE_ATTR(resolution, 0660, read_kxtj9_resolution, write_kxtj9_resolution);
static DEVICE_ATTR(wufe, 0660, read_kxtj9_wufe, write_kxtj9_wufe);
static DEVICE_ATTR(reg2_rate, 0660, read_kxtj9_reg2_rate, write_kxtj9_reg2_rate);
static DEVICE_ATTR(reset, 0660, reset_kxtj9, NULL);
// ASUS_BSP +++ Jiunhau_Wang "[PF450][Sensor][NA][Spec] Porting 9-axis sensor"
static DEVICE_ATTR(kxtj9_chip_id, 0440, kxtj9_chip_id_show, NULL);
static DEVICE_ATTR(kxtj9_status, 0440, kxtj9_status_show, NULL);
static DEVICE_ATTR(kxtj9_raw, 0440, kxtj9_read_raw, NULL);
static DEVICE_ATTR(kxtj9_dump_reg, 0440, kxtj9_dump_reg, NULL);
static DEVICE_ATTR(kxtj9_en_mt, 0660, kxtj9_r_en_mt, kxtj9_w_en_mt);
// ASUS_BSP --- Jiunhau_Wang "[PF450][Sensor][NA][Spec] Porting 9-axis sensor"

static struct attribute *kxtj9_attributes[] = {
	&dev_attr_message.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_state.attr,
	&dev_attr_cal_rawdata.attr,
	&dev_attr_calibration.attr,
	&dev_attr_resolution.attr,
	&dev_attr_wufe.attr,
	&dev_attr_reg2_rate.attr,
	&dev_attr_reset.attr,
// ASUS_BSP +++ Jiunhau_Wang "[PF450][Sensor][NA][Spec] Porting 9-axis sensor"
	&dev_attr_kxtj9_chip_id.attr,
	&dev_attr_kxtj9_status.attr,
	&dev_attr_kxtj9_raw.attr,
	&dev_attr_kxtj9_dump_reg,
	&dev_attr_kxtj9_en_mt,
// ASUS_BSP --- Jiunhau_Wang "[PF450][Sensor][NA][Spec] Porting 9-axis sensor"
	NULL
};

static struct attribute_group kxtj9_attribute_group = {
	.attrs = kxtj9_attributes
};


#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
static void kxtj9_poll(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;
	unsigned int poll_interval = dev->poll_interval;

	kxtj9_report_acceleration_data(tj9);

	if (poll_interval != tj9->last_poll_interval) {
		kxtj9_update_odr(tj9, poll_interval);
		tj9->last_poll_interval = poll_interval;
	}
//	if(KXTJ9_DEBUG_MESSAGE) 
		printk("alp : kxtj9_poll poll_interval(%d)\n",tj9->last_poll_interval);
}

static void kxtj9_polled_input_open(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;
	/*
	kxtj9_enable(tj9);
	*/
}

static void kxtj9_polled_input_close(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;

	kxtj9_disable(tj9);
}

//static int __devinit kxtj9_setup_polled_device(struct kxtj9_data *tj9)
static int kxtj9_setup_polled_device(struct kxtj9_data *tj9)
{
	int err;
	struct input_polled_dev *poll_dev;
	poll_dev = input_allocate_polled_device();

	if (!poll_dev) {
		dev_err(&tj9->client->dev,
			"Failed to allocate polled device\n");
		return -ENOMEM;
	}

	tj9->poll_dev = poll_dev;
	tj9->input_dev = poll_dev->input;

	poll_dev->private = tj9;
	poll_dev->poll = kxtj9_poll;
	poll_dev->open = kxtj9_polled_input_open;
	poll_dev->close = kxtj9_polled_input_close;

	kxtj9_init_input_device(tj9, poll_dev->input);

	err = input_register_polled_device(poll_dev);
	if (err) {
		dev_err(&tj9->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return err;
	}

	return 0;
}

static void __devexit kxtj9_teardown_polled_device(struct kxtj9_data *tj9)
{
	input_unregister_polled_device(tj9->poll_dev);
	input_free_polled_device(tj9->poll_dev);
}

#else

static inline int kxtj9_setup_polled_device(struct kxtj9_data *tj9)
{
	return -ENOSYS;
}

static inline void kxtj9_teardown_polled_device(struct kxtj9_data *tj9)
{
}

#endif

//static int __devinit kxtj9_vertify(struct kxtj9_data *tj9)
static int kxtj9_vertify(struct kxtj9_data *tj9)
{
	int retval=0;
	if(KXTJ9_DEBUG_MESSAGE) printk("[Gsensor] alp : kxtj9_verify ++\n");

//	retval = kxtj9_device_power_on(tj9);
//	if (retval < 0)
//		return retval;

	retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
	if(KXTJ9_DEBUG_MESSAGE) printk("[Gsensor] alp : kxtj9_vertify ret = %d\n",retval);
	if (retval < 0) {
		if(KXTJ9_DEBUG_MESSAGE) printk("[Gsensor] alp : read fail ret = %d\n", retval);
		dev_err(&tj9->client->dev, "read err int source\n");
		goto out;
	}
	if(KXTJ9_DEBUG_MESSAGE) printk("[Gsensor] alp : read retval = %d\n", retval);

	if(retval == 0x09)
		printk("[Gsensor] alp : use kxtj2!!\n");
	else if(retval == 0x08)
		printk("[Gsensor] alp : use kxtj9!!\n");
	

	if ((retval != 0x04) && (retval != 0x07) && (retval != 0x08) && (retval != 0x09) && (retval != 0x0f)) {
		if(KXTJ9_DEBUG_MESSAGE) printk("[Gsensor] alp : whoami = %d\n", retval);
		pr_warn("[Gsensor] kxtj9 whoami = %d\n", retval);
		retval = -EIO;
	} else
		retval = 0;
out:
	kxtj9_device_power_off(tj9);
	if(KXTJ9_DEBUG_MESSAGE) printk("[Gsensor] alp : kxtj9_vertify --\n");
	return retval;
}

/*
static struct file_operations kxtj9_fops = {
	.owner			= 	THIS_MODULE,
//	.poll 			= 	kxtj9_poll,
//	.read 			= 	kxtj9_read,
	.unlocked_ioctl	=	kxtj9_ioctl,
//	.compat_ioctl		=	kxtj9_ioctl,
	.open			=	kxtj9_open,
	.release			=	kxtj9_release,
};
*/

static struct miscdevice kxtj9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kxtj9_dev",
//	.fops = &kxtj9_fops,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kxtj9_early_suspend(struct early_suspend *h)
{
	struct kxtj9_data *tj9 = container_of(h,struct kxtj9_data,kxtj9_early_suspendresume);
	struct input_dev *input_dev = tj9->input_dev;
	int status=0;

	mutex_lock(&input_dev->mutex);
	tj9->suspend_resume_state = 1;
	// update enable status
	status = 	tj9->ctrl_reg1 >> 7;
	printk("alp : kxtj9_early_suspend enable(%d)\n",status);
	if(status==KXTJ9_RESUME_ENABLE){
		printk("alp : kxtj, need to enable after resume!\n");
		tj9->resume_enable = KXTJ9_RESUME_ENABLE;
	}
	else{
		tj9->resume_enable = KXTJ9_RESUME_DISABLE;
	}
//	if(tj9->irq_status==1){
//		disable_irq(tj9->irq);
//		tj9->irq_status=0;
//	}
	kxtj9_disable(tj9);
	mutex_unlock(&input_dev->mutex);
	printk("alp : kxtj9_early_suspend irq(%d)\n",tj9->irq);
}

static void kxtj9_late_resume(struct early_suspend *h)
{
	struct kxtj9_data *tj9 = container_of(h,struct kxtj9_data,kxtj9_early_suspendresume);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
	tj9->suspend_resume_state = 0;
	if( (tj9->resume_enable==KXTJ9_RESUME_ENABLE)||(tj9->resume_enable==KXTJ9_RESUME_MISSENABLE) ){
		kxtj9_enable(tj9);
//		if(tj9->irq_status==0){
//			enable_irq(tj9->irq);
//			tj9->irq_status = 1;
//		}
		printk("alp : kxtj9_late_resume enable\n");
	}else
		printk("alp : kxtj9_late_resume pass enable\n");
	mutex_unlock(&input_dev->mutex);
	printk("alp : kxtj9_late_resume irq(%d)\n",tj9->irq);
}
#endif


static int kxtj9_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct kxtj9_platform_data *pdata = client->dev.platform_data;
	struct kxtj9_data *tj9;
	int gpio=0, iloop=0;
	int err = 0;
	printk(KERN_INFO "[Progress][%s] Probe starts \n", NAME);
	printk("[Gsensor] Probe KXTJ9 Gsensor i2c driver\n");

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	tj9 = kzalloc(sizeof(*tj9), GFP_KERNEL);
	if (!tj9) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	tj9->client = client;
	tj9->pdata = *pdata;
	//init calibration data
	for(iloop=0;iloop<6;iloop++){
		tj9->accel_cal_data[iloop]=0;
	}
	for(iloop=0;iloop<3;iloop++){
		tj9->accel_cal_offset[iloop]=0;
		tj9->accel_cal_sensitivity[iloop]=1024;
	}

	gpio = pdata->gpio;
	printk("[Gsensor] Get Gsensor interrupt : %d\n", gpio);
	err = gpio_request(gpio,"accel_kxtj9_interrupt");
	err = gpio_direction_input(gpio);
	tj9->irq = gpio_to_irq(gpio);
	printk("[Gsensor] Setting Final irq=%d\n", tj9->irq);

	i2c_set_clientdata(client, tj9);
	err = kxtj9_vertify(tj9);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
	}
	tj9->ctrl_reg1 |= RES_12bit;
	tj9->last_poll_interval = tj9->pdata.init_interval;
	printk("[Gsensor] alp : init reg1=%d \n",tj9->ctrl_reg1);

	if (tj9->irq) {
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		tj9->int_ctrl |= KXTJ9_IEN | KXTJ9_IEA ;//| KXTJ9_IEL;

		tj9->ctrl_reg1 |= DRDYE;
		tj9->ctrl_reg1 &= 0xe7;
		err = kxtj9_setup_input_device(tj9);

		err = request_threaded_irq(tj9->irq, NULL, kxtj9_isr,IRQF_TRIGGER_RISING | IRQF_ONESHOT, "kxtj9-irq", tj9);
		if (err) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
		}

		err = sysfs_create_group(&client->dev.kobj,
				&kxtj9_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
		}
		err = misc_register(&kxtj9_device);
		if (err) {
				printk("[Gsensor] alp :  kxtj9_misc_register failed\n");
		}
	} else {
		err = kxtj9_setup_polled_device(tj9);
	}
	printk("[Gsensor] alp : final reg1=%d \n",tj9->ctrl_reg1);
	atomic_set(&tj9->enabled, 0);

	g_ilocation = KXTJ2_CHIP_LOCATION_SR_ZC500CL;
	
// for enable motion detect , added by cheng_kao 2014.02.12 ++
	if(build_version==1){	// 1:eng ; 2:user ; 3:userdebug 
		bypass_for_eng_mode = true;
		printk("[Gsensor] alp.D. : kxtj9 G-sensor for eng mode !!\n");
	}
	tj9->motion_detect_threshold_x=0;
	tj9->motion_detect_threshold_y=0;
	tj9->motion_detect_threshold_z=0;
	tj9->motion_detect_timer=-1;	// init counter
	tj9->chip_interrupt_mode=INT_MODE_DRDY;
	tj9->wufe_rate=WUFE25F;		// 	rate
	tj9->wufe_timer=0x01;			//	1 / rate = delay (sec)
	tj9->wufe_thres=0x01;			//	counts 16 = 1g
// for enable motion detect , added by cheng_kao 2014.02.12 --

#ifdef CONFIG_HAS_EARLYSUSPEND
	tj9->kxtj9_early_suspendresume.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	tj9->kxtj9_early_suspendresume.suspend = kxtj9_early_suspend;
	tj9->kxtj9_early_suspendresume.resume = kxtj9_late_resume;
	register_early_suspend(&tj9->kxtj9_early_suspendresume);
#endif

	tj9->suspend_resume_state = 0;
	tj9->resume_enable = 0;
	tj9->irq_status=1;
	if (err == 0)
		KXTJ9_status = 1;
	printk("[Gsensor] alp : kxtj9_probe (%d) --\n", g_ilocation);
	printk(KERN_INFO "[Progress][%s] Probe ends \n", NAME);

	return err;
}

//static int __devexit kxtj9_remove(struct i2c_client *client)
static int kxtj9_remove(struct i2c_client *client)
{
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);

	if (tj9->irq) {
		sysfs_remove_group(&client->dev.kobj, &kxtj9_attribute_group);
		free_irq(tj9->irq, tj9);
		input_unregister_device(tj9->input_dev);
	} else {
		kxtj9_teardown_polled_device(tj9);
	}

	if (tj9->pdata.exit)
		tj9->pdata.exit();

	kfree(tj9);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kxtj9_suspend(struct device *dev)
{
/*
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
//	if (input_dev->users){
//		disable_irq(tj9->irq);
		kxtj9_disable(tj9);
//	}
	mutex_unlock(&input_dev->mutex);
*/
	printk("alp : kxtj9_suspend\n");
	return 0;
}

static int kxtj9_resume(struct device *dev)
{
/*
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
//	if (input_dev->users){
//		enable_irq(tj9->irq);
		kxtj9_enable(tj9);
//	}
	mutex_unlock(&input_dev->mutex);
*/
	printk("alp : kxtj9_resume\n");
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(kxtj9_pm_ops, kxtj9_suspend, kxtj9_resume);

static const struct i2c_device_id kxtj9_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kxtj9_id);

static struct i2c_driver kxtj9_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.pm	= &kxtj9_pm_ops,
	},
	.probe		= kxtj9_probe,
	//.remove		= __devexit_p(kxtj9_remove),
	.remove     = kxtj9_remove,
	.id_table	= kxtj9_id,
};

static int __init kxtj9_init(void)
{
	int res = 0;
	pr_info("[Gsensor] Gsensor driver: initialize.\n");
	res = i2c_add_driver(&kxtj9_driver);
	if (res != 0)
		printk("[Gsensor] I2c_add_driver fail, Error : %d\n", res);
	return res;
}
module_init(kxtj9_init);

static void __exit kxtj9_exit(void)
{
	gpio_free(gpio_line);
	i2c_del_driver(&kxtj9_driver);
}
module_exit(kxtj9_exit);

MODULE_DESCRIPTION("KXTJ9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
