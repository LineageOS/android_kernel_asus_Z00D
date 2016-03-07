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
/* IR Sensor module test Module */
/********************************/
#define MODULE_NAME	"mtest"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/random.h>
#include "mtest.h"
#include "../IRsensor.h"
#include "../sysfs/sysfs.h"
#include "../hardware/hardware.h"
#include "../hardware/cm36686/cm36686.h"

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef IR_MTEST_DEBUG
	#define dbg(fmt, args...) printk("[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif

#define log(fmt, args...) printk("[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)

/*********************/
/* sysfs module test */
/********************/
int mtest_sysfs(void)
{
	bool bret;
	int ret;
	int round;
	int random;

	log("sysfs module test START\n");
	/*Proximity R/W High sysfs*/
	for(round =0; round < TEST_RUN; round++) {
		random = get_random_int() % 100;
		bret = proximity_sysfs_write_high(random);
		if(bret == false) {
			err("proximity_sysfs_write_high ERROR:(%d)\n", random);
			return -SYSFS_ERROR;
		}
		ret = proximity_sysfs_read_high();
		if(ret != random) {			
			err("proximity_sysfs_read_high ERROR:(%d)\n", ret);
			return -SYSFS_ERROR;
		}
	}

	/*Proximity R/W Low sysfs*/
	for(round =0; round < TEST_RUN; round++) {
		random = get_random_int() % 100;
		bret = proximity_sysfs_write_low(random);
		if(bret == false) {
			err("proximity_sysfs_write_low ERROR:(%d)\n", random);
			return -SYSFS_ERROR;
		}
		ret = proximity_sysfs_read_low();
		if(ret != random) {			
			err("proximity_sysfs_read_low ERROR:(%d)\n", ret);
			return -SYSFS_ERROR;
		}
	}

	/*Light Sensor R/W 200Lux sysfs*/
	for(round =0; round < TEST_RUN; round++) {
		random = get_random_int() % 100;
		bret = light_sysfs_write_200lux(random);
		if(bret == false) {
			err("light_sysfs_write_200lux ERROR:(%d)\n", random);
			return -SYSFS_ERROR;
		}
		ret = light_sysfs_read_200lux();
		if(ret != random) {			
			err("light_sysfs_read_200lux ERROR:(%d)\n", ret);
			return -SYSFS_ERROR;
		}
	}

	/*Light Sensor R/W 1000Lux sysfs*/
	for(round =0; round < TEST_RUN; round++) {
		random = get_random_int() % 100;
		bret = light_sysfs_write_1000lux(random);
		if(bret == false) {
			err("light_sysfs_write_1000lux ERROR:(%d)\n", random);
			return -SYSFS_ERROR;
		}
		ret = light_sysfs_read_1000lux();
		if(ret != random) {			
			err("light_sysfs_read_1000lux ERROR:(%d)\n", ret);
			return -SYSFS_ERROR;
		}
	}

	log("sysfs module test done : PASS\n");
	return 0;
}

/*************************/
/* hardware module test */
/************************/
int mtest_hardware(void)
{
	uint8_t power_state_data_buf[2] = {0, 0};
	int ret = 0;

	/* read register test */
	ret = IRsensor_hw_read_reg(PS_CONF1, &power_state_data_buf);
	if (ret < 0) {
		err("IRsensor_hw_read_reg ERROR:(0x%02X)\n", PS_CONF1);
		return -HARDWARE_ERROR;
	}
	dbg("Proximity read PS_CONF1 (0x%02X%02X) \n", power_state_data_buf[1], power_state_data_buf[0]);

	/*proximity set threshold*/
	ret = proximity_set_threshold();
	if (ret < 0) {		
		return -HARDWARE_ERROR;
	}
	
	/*proximity turn ON/OFF test*/
	proximity_hw_turn_onoff(true);
	
	IRsensor_hw_read_reg(PS_CONF1, &power_state_data_buf);	
	if((power_state_data_buf[1] != PS_TURN_ON_H) || 
		(power_state_data_buf[0] != PS_TURN_ON_L)) {
		err("proximity_hw_turn_onoff ERROR:(0x%02X%02X)\n", power_state_data_buf[1], power_state_data_buf[0]);
		return -HARDWARE_ERROR;
	}

	ret = proximity_hw_get_adc();
	if (ret < 0) {
		err("proximity_hw_get_adc ERROR:(adc = %d)\n", ret);
		return -HARDWARE_ERROR;
	}
	
	proximity_hw_turn_onoff(false);
	
	IRsensor_hw_read_reg(PS_CONF1, &power_state_data_buf);	
	if((power_state_data_buf[1] != PS_TURN_OFF_H) || 
		(power_state_data_buf[0] != PS_TURN_OFF_L)) {
		err("proximity_hw_turn_onoff ERROR:(0x%02X%02X)\n", power_state_data_buf[1], power_state_data_buf[0]);
		return -HARDWARE_ERROR;
	}
	
	log("hardware module test done : PASS\n");
	return 0;
}

