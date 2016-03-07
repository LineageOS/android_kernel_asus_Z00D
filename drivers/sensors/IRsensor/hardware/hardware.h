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

/*******************************/
/* IR Sensor Hardware Module */
/******************************/
#ifndef __LINUX_IRSENSOR_HARDWARE_H
#define __LINUX_IRSENSOR_HARDWARE_H

enum hardware_source {
	IRsensor_hw_source_cm36686=0,	
	IRsensor_hw_source_ap3425,
	IRsensor_hw_source_max
};

typedef struct IRsensor_hw {	
	int (*IRsensor_hw_init)(struct i2c_client* client);
	int (*IRsensor_hw_show_allreg)(void);
	int (*IRsensor_hw_get_interrupt)(void);	
	
	int (*proximity_hw_turn_onoff)(bool bOn);
	int (*proximity_hw_get_adc)(void);
	int (*proximity_hw_set_hi_threshold)(int hi_threshold);
	int (*proximity_hw_set_lo_threshold)(int low_threshold);

	int (*light_hw_turn_onoff)(bool bOn);
	int (*light_hw_get_adc)(void);
	int (*light_hw_set_hi_threshold)(int hi_threshold);
	int (*light_hw_set_lo_threshold)(int low_threshold);
}IRsensor_hw;

typedef struct IRsensor_hw_i2c {
	
	
}IRsensor_hw_i2c;

extern IRsensor_hw* IRsensor_hw_getHardware(int hardware_source);
extern struct i2c_driver* IRsensor_hw_setI2cDriver(int hardware_source, 
	struct i2c_driver* IRsensor_i2c_driver_client);

/*read/write i2c for 1 Byte (8bits)*/
extern uint8_t i2c_read_reg_u8(struct i2c_client* client, u8 reg);
extern int i2c_write_reg_u8(struct i2c_client* client, u8 reg, uint8_t data);

/*read/write i2c for 2 Byte (16bits)*/
extern int i2c_read_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data);
extern int i2c_write_reg_u16(struct i2c_client* client, u8 reg, uint8_t* data);

#endif

