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

/********************************/
/* IR Sensor CM36686 Module */
/******************************/
#define MODULE_NAME	"ap3425"
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include "../hardware.h"
#include "../../IRsensor.h"
#include "ap3425.h"

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef IR_AP3425_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_DEBUG "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] [%s] "fmt,DRIVER_NAME, MODULE_NAME,##args)

/************************************/
/*Global static variable and function*/
/***********************************/
static struct i2c_client	*g_i2c_client = NULL;
static int ap3425_IRsensor_hw_check(void);
static int ap3425_IRsensor_hw_set_LED(void);
static int ap3425_IRsensor_hw_set_interrupt(void);
static int ap3425_proximity_hw_set_integration(void);

static int ap3425_IRsensor_hw_init(struct i2c_client* client);
static int ap3425_IRsensor_hw_show_allreg(void);
static int ap3425_IRsensor_hw_get_interrupt(void);

static int ap3425_proximity_hw_turn_onoff(bool bOn);
static int ap3425_proximity_hw_get_adc(void);
static int ap3425_proximity_hw_set_hi_threshold(int hi_threshold);
static int ap3425_proximity_hw_set_lo_threshold(int low_threshold);

static int ap3425_light_hw_turn_onoff(bool bOn);
static int ap3425_light_hw_get_adc(void);
static int ap3425_light_hw_set_hi_threshold(int hi_threshold);
static int ap3425_light_hw_set_lo_threshold(int low_threshold);

static u8 ap3425_reg[AP3425_NUM_CACHABLE_REGS] = 
{0x00,0x01,0x02,0x06,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x14,0x1A,0x1B,0x1C,0x1D,
    0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x28,0x29,0x2A,0x2B,0x2C,0x2D};

/**************************/
/* i2c read/write register */
/*************************/
static int ap3425_IRsensor_hw_check(void)
{	
	uint8_t data_buf = 0;	

	/* Check the Device ID */
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3425_REG_SYS_INTCTRL);
	if (data_buf != 0x88) {
		err("ap3425_IRsensor_hw_check ERROR. (ID_REG : 0x%02X)\n", AP3425_REG_SYS_INTCTRL);
		return -ENOMEM;
	}
	log("ap3425_IRsensor_hw_check (0x%X) \n", data_buf);
	return 0;
}

static int ap3425_IRsensor_hw_set_LED(void)
{
	int ret = 0;	
	uint8_t data_origin = 0;
	uint8_t data_buf = 0;	

	/* read LED current */
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3425_REG_PS_LEDD);
	if (data_buf < 0) {
		err("ap3425_IRsensor_hw_set_LED read ERROR\n");
		return data_buf;
	}
	dbg("ap3425_IRsensor_hw_set_LED read (0x%02X) \n", data_buf);
	
	data_origin = data_buf;		
	data_buf = AP3425_SYS_LED_667;
	
	/* write LED current */
	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_PS_LEDD, data_buf);
	if (ret < 0) {
		err("ap3425_IRsensor_hw_set_LED write ERROR \n");
		return ret;
	} else {
		log("ap3425_IRsensor_hw_set_LED write (0x%X -> 0x%X) \n", data_origin, data_buf);
	}

	return 0;
}

static int ap3425_IRsensor_hw_set_interrupt(void)
{
	int ret = 0;
	uint8_t data_origin= 0;
	uint8_t data_buf = 0;	

	/* read IRsensor Interrupt state */
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3425_REG_SYS_INTCTRL);
	if (data_buf < 0) {
		err("ap3425_IRsensor_hw_set_interrupt read ERROR\n");
		return data_buf;
	}
	dbg("ap3425_IRsensor_hw_set_interrupt read (0x%02X) \n", data_buf);

	data_origin = data_buf;
	data_buf |= AP3425_SYS_ICLEAN_MANUAL;

	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_SYS_INTCTRL, data_buf);
	if (ret < 0) {
		err("ap3425_IRsensor_hw_set_interrupt write ERROR \n");
		return ret;
	} else {
		log("ap3425_IRsensor_hw_set_interrupt write (0x%X -> 0x%X) \n", data_origin, data_buf);
	}

	return 0;	
}

static int ap3425_proximity_hw_set_integration(void)
{
	int ret = 0;	
	uint8_t data_origin= 0;
	uint8_t data_buf = 0;	

	/* read Proximity Integration state */
	data_buf = i2c_read_reg_u8(g_i2c_client, AP3425_REG_PS_INTEGR);
	if (data_buf < 0) {
		err("ap3425_proximity_hw_set_integration read ERROR\n");
		return data_buf;
	}
	dbg("ap3425_proximity_hw_set_integration read (0x%02X) \n", data_buf);

	data_origin = data_buf;
	data_buf = AP3425_PS_INTEG;

	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_PS_INTEGR, data_buf);
	if (ret < 0) {
		err("ap3425_proximity_hw_set_integration write ERROR \n");
		return ret;
	} else {
		log("ap3425_proximity_hw_set_integration write (0x%X -> 0x%X) \n", data_origin, data_buf);
	}

	return 0;	
}

/*****************/
/*IR Sensor Part*/
/****************/
static int ap3425_IRsensor_hw_init(struct i2c_client* client)
{
	int ret = 0;

	g_i2c_client = client;
	
	/* Check the Hardware Device */
	ret = ap3425_IRsensor_hw_check();
	if (ret < 0) {		
		return ret;
	}	
	
	/*Set LED */
	ret = ap3425_IRsensor_hw_set_LED();
	if (ret < 0) {		
		return ret;
	}

	/*Set Interrupt Control*/
	ret = ap3425_IRsensor_hw_set_interrupt();
	if (ret < 0) {		
		return ret;
	}

	 /*Set proximity Integration */
	ret = ap3425_proximity_hw_set_integration();
	if (ret < 0) {		
		return ret;
	}

	return 0;
}

static int ap3425_IRsensor_hw_show_allreg(void)
{	
	int reg = 0;
	uint8_t buf = 0;
	for(;reg < AP3425_NUM_CACHABLE_REGS; reg++)
	{
		buf = i2c_read_reg_u8(g_i2c_client, ap3425_reg[reg]);
		log("Show All Register (0x%X) = 0x%02X\n", ap3425_reg[reg], buf);
		if(buf < 0) 
		{
			err("IR Sensor show all Register ERROR. (REG:0x%X)\n", ap3425_reg[reg]);
			return buf;
		}
	}
	
	return 0;
}

static int ap3425_IRsensor_hw_get_interrupt(void)
{
	uint8_t buf = 0;
	uint8_t data_buf = 0;
	bool check_flag = false;
	int irsensor_int = 0;
	int ret = 0;
	
	/* Read INT_FLAG and clean the interrupt */
	buf = i2c_read_reg_u8(g_i2c_client, AP3425_REG_SYS_INTSTATUS);
	data_buf = buf;
	data_buf &= ~AP3425_REG_SYS_INT_PS_CLR;
	data_buf &= ~AP3425_REG_SYS_INT_LS_CLR;
	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_SYS_INTSTATUS, data_buf);
	if (ret < 0) {
		err("ap3425_IRsensor_hw_get_interrupt clear ERROR \n");
		return ret;
	}
	log("ap3425_IRsensor_hw_get_interrupt clear (0x%02X) \n", data_buf);

	/*Proximity Sensor work */
	if (buf&AP3425_REG_SYS_INT_PMASK) 
	{
		check_flag =true;
		if (buf&AP3426_OBJ_MASK) {
			irsensor_int |= IRSENSOR_INT_PS_CLOSE;
		}else{
			irsensor_int |= IRSENSOR_INT_PS_AWAY;
		}
	}

	/* Light Sensor work */
	if(buf&AP3425_REG_SYS_INT_AMASK) 
	{	
		check_flag =true;
		irsensor_int |= IRSENSOR_INT_ALS;
	} 

	/* Interrupt Error */
	if(check_flag == false){		
		err("Can NOT recognize the INT_FLAG (0x%02X)\n", buf);
	}
	
	return irsensor_int;
}

/****************/
/*Proximity Part*/
/****************/
static int ap3425_proximity_hw_turn_onoff(bool bOn)
{	
	int ret = 0;	
	uint8_t power_state_data_buf= 0;
	uint8_t power_state_data_origin = 0;

	/* read power status */
	power_state_data_buf = i2c_read_reg_u8(g_i2c_client, AP3425_REG_SYS_CONF);
	if (power_state_data_buf < 0) {
		err("Proximity read AP3425_REG_SYS_CONF ERROR\n");
		return power_state_data_buf;
	}
	dbg("Proximity read AP3425_REG_SYS_CONF (0x%02X) \n", power_state_data_buf);	
	power_state_data_origin = power_state_data_buf;
	
	if (bOn == 1)	{	/* power on */		
		power_state_data_buf |= AP3425_SYS_PS_ENABLE;
		
		ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_SYS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Proximity power on ERROR (AP3425_REG_SYS_CONF) \n");
			return ret;
		} else {
			log("Proximity power on (AP3425_REG_SYS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin, power_state_data_buf);
		}
	} else	{	/* power off */
		power_state_data_buf &= ~AP3425_SYS_PS_ENABLE;      	

		ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_SYS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Proximity power off ERROR (AP3425_REG_SYS_CONF) \n");
			return ret;
		} else {
			log("Proximity power off (AP3425_REG_SYS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin, power_state_data_buf);
		}
	}
	
	return 0;
}

static int ap3425_proximity_hw_get_adc(void)
{
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	

	adc_buf[0] = i2c_read_reg_u8(g_i2c_client, AP3425_REG_PS_DATA_LOW);
	if (ret < 0) {
		err("Proximity get adc ERROR. (AP3425_REG_PS_DATA_LOW)\n");
		return ret;
	}

	adc_buf[1] = i2c_read_reg_u8(g_i2c_client, AP3425_REG_PS_DATA_HIGH);
	if (ret < 0) {
		err("Proximity get adc ERROR. (AP3425_REG_PS_DATA_HIGH)\n");
		return ret;
	}
	
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Proximity get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;
}

static int ap3425_proximity_hw_set_hi_threshold(int hi_threshold)
{	
	int ret = 0;
	uint8_t data_buf[2] = {0,0};	
	
	/*Set Proximity High Threshold*/
	data_buf[0] = hi_threshold % (1<<8);
	data_buf[1] = hi_threshold /  (1<<8);
	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_PS_THDH_L, data_buf[0]);
	if(ret < 0) {
		err("Proximity write High Threshold lsb ERROR. (AP3425_REG_PS_THDH_L : 0x%02X)\n",  data_buf[0]);
	    	return ret;
	} else {
	    	log("Proximity write High Threshold lsb (AP3425_REG_PS_THDH_L : 0x%02X)\n", data_buf[0]);
	}		

	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_PS_THDH_H, data_buf[1]);
	if(ret < 0) {
		err("Proximity write High Threshold msb ERROR. (AP3425_REG_PS_THDH_H : 0x%02X)\n",  data_buf[1]);
	    	return ret;
	} else {
	    	log("Proximity write High Threshold msb (AP3425_REG_PS_THDH_H : 0x%02X)\n", data_buf[1]);
	}	
	
	return 0;
}

static int ap3425_proximity_hw_set_lo_threshold(int low_threshold)
{
	int ret = 0;
	uint8_t data_buf[2] = {0, 0};	

	/*Set Proximity Low Threshold*/
	data_buf[0] = low_threshold % (1<<8);
	data_buf[1] = low_threshold /  (1<<8);
	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_PS_THDL_L, data_buf[0]);
	if(ret < 0) {
		err("Proximity write Low Threshold lsb ERROR. (AP3425_REG_PS_THDL_L : 0x%02X)\n", data_buf[0]);
	    	return ret;
	} else {
	    	log("Proximity write Low Threshold lsb (AP3425_REG_PS_THDL_L : 0x%02X)\n", data_buf[0]);
	}

	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_PS_THDL_H, data_buf[1]);
	if(ret < 0) {
		err("Proximity write High Threshold msb ERROR. (AP3425_REG_PS_THDL_H : 0x%02X)\n",  data_buf[1]);
	    	return ret;
	} else {
	    	log("Proximity write High Threshold msb (AP3425_REG_PS_THDL_H : 0x%02X)\n", data_buf[1]);
	}	
	
	return 0;
}

/*********************/
/* Light Sensor Part */
/********************/
static int ap3425_light_hw_turn_onoff(bool bOn)
{
	int ret = 0;	
	uint8_t power_state_data_buf = 0;
	uint8_t power_state_data_origin = 0;

	/* read power status */
	power_state_data_buf = i2c_read_reg_u8(g_i2c_client, AP3425_REG_SYS_CONF);	
	if (power_state_data_buf < 0) {
		err("Light Sensor read AP3425_REG_SYS_CONF ERROR\n");
		return power_state_data_buf;
	}
	dbg("Light Sensor read AP3425_REG_SYS_CONF (0x%02X) \n", power_state_data_buf);
	power_state_data_origin = power_state_data_buf;	

	if (bOn == 1)	{	/* power on */
		power_state_data_buf |= AP3425_SYS_ALS_ENABLE;

		ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_SYS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Light Sensor power on ERROR (AP3425_REG_SYS_CONF) \n");
			return ret;
		} else {
			log("Light Sensor power on (AP3425_REG_SYS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin, power_state_data_buf);
		}		
	} else	{	/* power off */
		power_state_data_buf &= ~AP3425_SYS_ALS_ENABLE;
		
		ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_SYS_CONF, power_state_data_buf);
		if (ret < 0) {
			err("Light Sensor power off ERROR (AP3425_REG_SYS_CONF) \n");
			return ret;
		} else {
			log("Light Sensor power off (AP3425_REG_SYS_CONF : 0x%X -> 0x%X) \n", 
				power_state_data_origin, power_state_data_buf);
		}
	}	

	return 0;
}

static int ap3425_light_hw_get_adc(void)
{
	int ret = 0;
	int adc = 0;
	uint8_t adc_buf[2] = {0, 0};	
	
	adc_buf[0] = i2c_read_reg_u8(g_i2c_client, AP3425_REG_ALS_DATA_LOW);
	if (ret < 0) {
		err("Light Sensor Get adc ERROR (AP3425_REG_ALS_DATA_LOW)\n");
		return ret;
	}

	adc_buf[1] = i2c_read_reg_u8(g_i2c_client, AP3425_REG_ALS_DATA_HIGH);
	if (ret < 0) {
		err("Light Sensor Get adc ERROR (AP3425_REG_ALS_DATA_HIGH)\n");
		return ret;
	}
	
	adc = (adc_buf[1] << 8) + adc_buf[0];
	dbg("Light Sensor Get adc : 0x%02X%02X\n", adc_buf[1], adc_buf[0]); 
	
	return adc;	
}

static int ap3425_light_hw_set_hi_threshold(int hi_threshold)
{
	int ret = 0;	
	uint8_t data_buf[2] = {0, 0};	

	/*Set Light Sensor High Threshold*/
	data_buf[0] = hi_threshold % (1<<8);
	data_buf[1] = hi_threshold /  (1<<8);
	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_ALS_THDH_L, data_buf[0]);
	if(ret < 0) {
		err("Light Sensor write High Threshold lsb ERROR. (AP3425_REG_ALS_THDH_L : 0x%02X)\n", data_buf[0]);
	    	return ret;
	} else {
	    	dbg("Light Sensor write High Threshold lsb (ALS_AP3425_REG_ALS_THDH_LTHDH : 0x%02X)\n", data_buf[0]);
	}		

	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_ALS_THDH_H, data_buf[1]);
	if(ret < 0) {
		err("Light Sensor write High Threshold msb ERROR. (AP3425_REG_ALS_THDH_H : 0x%02X)\n", data_buf[1]);
	    	return ret;
	} else {
	    	dbg("Light Sensor write High Threshold msb (AP3425_REG_ALS_THDH_H : 0x%02X)\n", data_buf[1]);
	}	
	
	return 0;
}

static int ap3425_light_hw_set_lo_threshold(int low_threshold)
{
	int ret = 0;	
	uint8_t data_buf[2] = {0, 0};	

	/*Set Light Sensor Low Threshold */
	data_buf[0] = low_threshold % (1<<8);
	data_buf[1] = low_threshold /  (1<<8);
	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_ALS_THDL_L, data_buf[0]);
	if(ret < 0) {
		err("Light Sensor write Low Threshold lsb ERROR. (AP3425_REG_ALS_THDL_L : 0x%02X)\n", data_buf[0]);
	    	return ret;
	} else {
	    	dbg("Light Sensor write Low Threshold lsb (AP3425_REG_ALS_THDL_L : 0x%02X)\n", data_buf[0]);
	}

	ret = i2c_write_reg_u8(g_i2c_client, AP3425_REG_ALS_THDL_H, data_buf[1]);
	if(ret < 0) {
		err("Light Sensor write Low Threshold msb ERROR. (AP3425_REG_ALS_THDL_H : 0x%02X)\n", data_buf[1]);
	    	return ret;
	} else {
	    	dbg("Light Sensor write Low Threshold msb (AP3425_REG_ALS_THDL_H : 0x%02X)\n", data_buf[1]);
	}
	
	return 0;
}

static struct IRsensor_hw IRsensor_hw_ap3425 = {	
	.IRsensor_hw_init = ap3425_IRsensor_hw_init,
	.IRsensor_hw_show_allreg = ap3425_IRsensor_hw_show_allreg,
	.IRsensor_hw_get_interrupt = ap3425_IRsensor_hw_get_interrupt,
	
	.proximity_hw_turn_onoff = ap3425_proximity_hw_turn_onoff,
	.proximity_hw_get_adc = ap3425_proximity_hw_get_adc,
	.proximity_hw_set_hi_threshold = ap3425_proximity_hw_set_hi_threshold,
	.proximity_hw_set_lo_threshold = ap3425_proximity_hw_set_lo_threshold,

	.light_hw_turn_onoff = ap3425_light_hw_turn_onoff,
	.light_hw_get_adc = ap3425_light_hw_get_adc,
	.light_hw_set_hi_threshold = ap3425_light_hw_set_hi_threshold,
	.light_hw_set_lo_threshold = ap3425_light_hw_set_lo_threshold,
};

IRsensor_hw* IRsensor_hw_ap3425_getHardware(void)
{
	IRsensor_hw* IRsensor_hw_client = NULL;
	IRsensor_hw_client = &IRsensor_hw_ap3425;
	return IRsensor_hw_client;
}

