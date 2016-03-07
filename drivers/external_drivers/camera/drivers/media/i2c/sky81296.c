/*
 * sky81296.c	SKY81296 flash LED driver
 *
 * Copyright 2014 Asus Corporation.
 * Author : Chung-Yi Chou <chung-yi_chou@asus.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <media/sky81296.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/atomisp.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>

#define FLED_DRIVER_ENT "FLED_DRIVER_EN"
#define FLED_DRIVER_ENF "FLASH_FLEN"

static int camera_vprog1_on;
static struct regulator *vprog1_reg;
#define VPROG1_VAL 1800000

static int flash_ent;
static int flash_enf;
/* Registers */
#define SKY81296_FLASH1_CURRENT		0x00
#define SKY81296_FLASH2_CURRENT		0x01
#define SKY81296_FLASH_TIMER		0x02
#define SKY81296_MOVIE_MODE_CURRENT	0x03
#define SKY81296_CONTROL1		0x04
#define SKY81296_CONTROL2		0x05
#define SKY81296_CONTROL3		0x06
#define SKY81296_BLINKING_MODE1_TIME	0x07
#define SKY81296_BLINKING_MODE2_TIME	0x08
#define SKY81296_FAULT			0x09
#define SKY81296_INPUT_MONITOR1		0x0A
#define SKY81296_INPUT_MONITOR2		0x0B

/* bit mask */
#define SKY81296_IFL1			0x1F
#define SKY81296_IFL2			0x1F
#define SKY81296_FLTM2			0xF0
#define SKY81296_FLTM1			0x0F
#define SKY81296_IMM2			0xF0
#define SKY81296_IMM1			0x0F
#define SKY81296_BLK_EN2		0x40
#define SKY81296_FL_EN2			0x20
#define SKY81296_MM_EN2			0x10
#define SKY81296_BLK_EN1		0x04
#define SKY81296_FL_EN1			0x02
#define SKY81296_MM_EN1			0x01
#define SKY81296_RESET			0x80
#define SKY81296_FLINHM			0x10
#define SKY81296_FLT_INH		0x08
#define SKY81296_ILIM			0x06
#define SKY81296_VINM			0x01
#define SKY81296_VINHYS			0xF0
#define SKY81296_VINTH			0x0F
#define SKY81296_TOFF1			0xF0
#define SKY81296_TON1			0x0F
#define SKY81296_TOFF2			0xF0
#define SKY81296_TON2			0x0F
#define SKY81296_VINMONEX		0x80
#define SKY81296_SC			0x40
#define SKY81296_OC			0x20
#define SKY81296_OTMP			0x10
#define SKY81296_FLED2			0x0C
#define SKY81296_FLED1			0x03
#define SKY81296_IFL_MON1		0x1F
#define SKY81296_IFL_MON2		0x1F

#define CTZ(b) __builtin_ctz(b)

#define SKY81296_NAME			"sky81296"
#define SKY81296_FLASH_MAX_BRIGHTNESS	21
#define SKY81296_MOVIE_MAX_BRIGHTNESS	10
#define SKY81296_BLINK_MAX_BRIGHTNESS	SKY81296_MOVIE_MAX_BRIGHTNESS

#define sky81296_suspend NULL
#define sky81296_resume  NULL

static bool first_open;

struct sky81296 {
	struct v4l2_subdev sd;
	struct mutex power_lock;
	int power_count;
	struct timer_list flash_off_delay;
	struct regmap *map;
	int enable_by_pin;
	unsigned char fault;
	enum sky81296_flash_current flash_current;
	enum sky81296_torch_current torch_current;
	s32 indicator_current;
	enum sky81296_flash_timeout timeout;
	enum sky81296_flash_mode mode;
	struct sky81296_platform_data *pdata;
};
#define to_sky81296(p_sd)	container_of(p_sd, struct sky81296, sd)

static struct sky81296 *inner_sky81296;

struct sky81296_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl) (struct v4l2_subdev *sd, __u32 val);
	int (*g_ctrl) (struct v4l2_subdev *sd, __s32 *val);
};


static int sky81296_set_mode(struct sky81296 *flash, unsigned int new_mode)
{
	int ret;

	switch (new_mode) {
		case SKY81296_MODE_SHUTDOWN:
			ret = regmap_write(flash->map,SKY81296_CONTROL1,0x0);
			break;
		case SKY81296_MODE_FLASH:
			ret = regmap_write(flash->map, SKY81296_CONTROL1, 0x02);
			break;
		case SKY81296_MODE_INDICATOR:
			/*ret = regmap_write(flash->map, SKY81296_CONTROL1, 0x0f);*/
			break;
		case SKY81296_MODE_TORCH:
			ret = regmap_write(flash->map, SKY81296_CONTROL1, 0x01);
			break;
		default:
			return -EINVAL;
	}
	if (ret == 0)
		flash->mode = new_mode;
	return ret;
}

static int sky81296_set_flash(struct sky81296 *flash)
{
	int val1,val2;
	int ret;
	/*val2 = flash->flash_current % 100;
	val1 = (flash->flash_current - val2 ) / 100;*/
	val1 = flash->flash_current;

	/*if(val2 > SKY81296_FLASH_CURRENT_1000MA || val2 < SKY81296_FLASH_CURRENT_250MA ){
		val2 = SKY81296_FLASH_CURRENT_1000MA;
	}*/

	if(val1 > SKY81296_FLASH_CURRENT_1500MA || val1 < SKY81296_FLASH_CURRENT_250MA ){
		val1 = SKY81296_FLASH_CURRENT_1000MA;
	}

	ret = regmap_write(flash->map,SKY81296_FLASH1_CURRENT,val1);
	/*ret = regmap_write(flash->map,SKY81296_FLASH2_CURRENT,val2);*/
	/*ret = regmap_write(flash->map,SKY81296_CONTROL1, 0x02 );*/

	flash->mode = SKY81296_MODE_FLASH;
	return ret;
}

static int sky81296_set_torch(struct sky81296 *flash)
{
	u8 val;
	int ret;
	val = flash->torch_current;
	if(val > SKY81296_TORCH_CURRENT_250MA || val < SKY81296_TORCH_CURRENT_25MA){
		val = SKY81296_TORCH_CURRENT_125MA;
	}
	printk("%s : intensity = %d\n", __func__, val);
	ret = regmap_write(flash->map,SKY81296_MOVIE_MODE_CURRENT, (val << 4 | val) );
	/*ret = regmap_write(flash->map,SKY81296_CONTROL1, 0x01 );*/

	flash->mode = SKY81296_MODE_TORCH;

	return ret;
}

static int sky81296_set_indicator(struct sky81296 *flash)
{
	u8 val;
	int ret;
	val = flash->indicator_current;
	if (flash->mode == SKY81296_MODE_TORCH && flash->torch_current >= SKY81296_TORCH_CURRENT_25MA) {
		return 0;
	}
	if (val < 5000) {
		flash->mode = SKY81296_MODE_SHUTDOWN;
		return 0;
	}

	ret = regmap_write(flash->map, SKY81296_MOVIE_MODE_CURRENT, ((val-5000) / 5000) << CTZ(0x0f));
	if (IS_ERR_VALUE(ret))
		return ret;

	/*ret = regmap_write(flash->map,SKY81296_CONTROL1, 0x01 );*/

	flash->mode = SKY81296_MODE_INDICATOR;

	return ret;
}

static void sky81296_flash_off_delay(long unsigned int arg)
{
	int ret;
	struct v4l2_subdev *sd = i2c_get_clientdata((struct i2c_client *)arg);
	struct sky81296 *flash = to_sky81296(sd);

	ret = regmap_write(flash->map,SKY81296_CONTROL1, 0x0);

	flash->mode = SKY81296_MODE_SHUTDOWN;
}

static const struct regmap_config sky81296_config =
{
	.reg_bits = 8,
	.val_bits = 8,
};


/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static int sky81296_s_flash_timeout(struct v4l2_subdev *sd, u32 val)
{
	struct sky81296 *flash = to_sky81296(sd);
	int ret;
// Protect: Setting this to 1425ms
	if( val > SKY81296_FLASHTIMEOUT_1425MS || val < SKY81296_FLASHTIMEOUT_OFF){
		val = SKY81296_FLASHTIMEOUT_1425MS;
	}
	ret = regmap_write(flash->map,SKY81296_FLASH_TIMER, (val << 4 | val) );
	flash->timeout = val;
	return 0;
}

static int sky81296_g_flash_timeout(struct v4l2_subdev *sd, s32 *val)
{
	struct sky81296 *flash = to_sky81296(sd);
	*val = flash->timeout;
	return 0;
}

static int sky81296_s_flash_intensity(struct v4l2_subdev *sd, u32 intensity)
{

	struct sky81296 *flash = to_sky81296(sd);

	flash->flash_current = intensity;

	return sky81296_set_flash(flash);
}

static int sky81296_g_flash_intensity(struct v4l2_subdev *sd, s32 *val)
{

	struct sky81296 *flash = to_sky81296(sd);
	int value1,value2;
	int ret;
	ret = regmap_read(flash->map,SKY81296_FLASH1_CURRENT,&value1);
	ret = regmap_read(flash->map,SKY81296_FLASH2_CURRENT,&value2);
	*val = value1 * 100 + value2;

	return 0;
}

static int sky81296_s_torch_intensity(struct v4l2_subdev *sd, u32 intensity)
{

	struct sky81296 *flash = to_sky81296(sd);

/*
	Todo : Mapping minimum and maximum current for torch mode.
*/
	flash->torch_current = intensity;

	return sky81296_set_torch(flash);
}

static int sky81296_g_torch_intensity(struct v4l2_subdev *sd, s32 *val)
{

	struct sky81296 *flash = to_sky81296(sd);
	*val = flash->torch_current;
	return 0;
}

static int sky81296_s_indicator_intensity(struct v4l2_subdev *sd, u32 intensity)
{

	struct sky81296 *flash = to_sky81296(sd);

/*
	Todo : Mapping minimum and maximum current for torch mode.
*/
	flash->indicator_current = intensity;

	/*return sky81296_set_indicator(flash);*/
	return 0;
}

static int sky81296_g_indicator_intensity(struct v4l2_subdev *sd, s32 *val)
{
	struct sky81296 *flash = to_sky81296(sd);

	*val = (u32)flash->torch_current;

	return 0;
}

static int sky81296_s_flash_strobe(struct v4l2_subdev *sd, u32 val)
{
	struct sky81296 *flash = to_sky81296(sd);
	int ret;
	ret = regmap_write(flash->map,SKY81296_CONTROL1,0x44);
	return ret;
}

static int sky81296_s_flash_mode(struct v4l2_subdev *sd, u32 new_mode)
{
	struct sky81296 *flash = to_sky81296(sd);
	unsigned int mode;
	switch (new_mode) {
	case ATOMISP_FLASH_MODE_OFF:
		mode = SKY81296_MODE_SHUTDOWN;
		break;
	case ATOMISP_FLASH_MODE_FLASH:
		mode = SKY81296_MODE_FLASH;
		break;
	case ATOMISP_FLASH_MODE_INDICATOR:
		mode = SKY81296_MODE_INDICATOR;
		break;
	case ATOMISP_FLASH_MODE_TORCH:
		mode = SKY81296_MODE_TORCH;
		break;
	default:
		return -EINVAL;
	}

	return sky81296_set_mode(flash, mode);
}

static int sky81296_g_flash_mode(struct v4l2_subdev *sd, s32 * val)
{
	struct sky81296 *flash = to_sky81296(sd);
	*val = flash->mode;
	return 0;
}

static int sky81296_g_flash_status(struct v4l2_subdev *sd, s32 *val)
{
	struct sky81296 *flash = to_sky81296(sd);
	int ret;

	ret = regmap_read(flash->map, SKY81296_CONTROL1,val);
	if (ret < 0)
		return ret;
/*
	It should be mapped to enum atomisp_flash_status
	for
	ATOMISP_FLASH_STATUS_OK,
	ATOMISP_FLASH_STATUS_HW_ERROR,
	ATOMISP_FLASH_STATUS_INTERRUPTED,
	ATOMISP_FLASH_STATUS_TIMEOUT,
*/
	return 0;
}

#ifndef CSS15
static int sky81296_g_flash_status_register(struct v4l2_subdev *sd, s32 *val)
{

	struct sky81296 *flash = to_sky81296(sd);
	int ret;

	ret = regmap_read(flash->map, SKY81296_CONTROL1,val);

	if (ret < 0)
		return ret;

	return 0;
}
#endif

static const struct sky81296_ctrl_id sky81296_ctrls[] = {
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_TIMEOUT,
				"Flash Timeout",
				0,
				1445,
				1,
				1045,
				0,
				sky81296_s_flash_timeout,
				sky81296_g_flash_timeout),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_INTENSITY,
				"Flash Intensity",
				250,
				1500,
				1,
				750,
				0,
				sky81296_s_flash_intensity,
				sky81296_g_flash_intensity),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_TORCH_INTENSITY,
				"Torch Intensity",
				25,
				250,
				1,
				200,
				0,
				sky81296_s_torch_intensity,
				sky81296_g_torch_intensity),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_INDICATOR_INTENSITY,
				"Indicator Intensity",
				5000,
				50000,
				5000,
				40000,
				0,
				sky81296_s_indicator_intensity,
				sky81296_g_indicator_intensity),
	s_ctrl_id_entry_boolean(V4L2_CID_FLASH_STROBE,
				"Flash Strobe",
				0,
				0,
				sky81296_s_flash_strobe,
				NULL),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_MODE,
				"Flash Mode",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				ATOMISP_FLASH_MODE_OFF,
				0,
				sky81296_s_flash_mode,
				sky81296_g_flash_mode),
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_STATUS,
				"Flash Status",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				ATOMISP_FLASH_STATUS_OK,
				0,
				NULL,
				sky81296_g_flash_status),
#ifndef CSS15
	s_ctrl_id_entry_integer(V4L2_CID_FLASH_STATUS_REGISTER,
				"Flash Status Register",
				0,   /* don't assume any enum ID is first */
				100, /* enum value, may get extended */
				1,
				0,
				0,
				NULL,
				sky81296_g_flash_status_register),
#endif
};

static const struct sky81296_ctrl_id *find_ctrl_id(unsigned int id)
{
	int i;
	int num;

	num = ARRAY_SIZE(sky81296_ctrls);
	for (i = 0; i < num; i++) {
		if (sky81296_ctrls[i].qc.id == id)
			return &sky81296_ctrls[i];
	}

	return NULL;
}

static int sky81296_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int num;

	if (!qc)
		return -EINVAL;

	num = ARRAY_SIZE(sky81296_ctrls);
	if (qc->id >= num)
		return -EINVAL;

	*qc = sky81296_ctrls[qc->id].qc;

	return 0;
}

static int sky81296_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	const struct sky81296_ctrl_id *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = find_ctrl_id(ctrl->id);
	if (!s_ctrl)
		return -EINVAL;

	return s_ctrl->s_ctrl(sd, ctrl->value);
}

static int sky81296_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	const struct sky81296_ctrl_id *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = find_ctrl_id(ctrl->id);
	if (s_ctrl == NULL)
		return -EINVAL;

	return s_ctrl->g_ctrl(sd, &ctrl->value);
}

static int sky81296_setup(struct sky81296 *flash)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->sd);
	struct regmap *map;
	unsigned char value;
	int ret = 0;

	map = devm_regmap_init_i2c(client, &sky81296_config);
	if (IS_ERR(map)){
		return PTR_ERR(map);
	}



	switch(flash->pdata->current_limit) {
		case 2420: /* 2420mA to 3080mA */
			value = 0 << CTZ(SKY81296_ILIM);
			break;
		case 2600: /* 2600mA to 3300mA */
			value = 1 << CTZ(SKY81296_ILIM);
			break;
		case 2800: /* 2800mA to 3600mA */
			value = 2 << CTZ(SKY81296_ILIM);
			break;
		case 3000: /* 3000mA to 3950mA */
			value = 3 << CTZ(SKY81296_ILIM);
			break;
		default:
			return -EINVAL;
	}

	if (flash->pdata->disable_short_led_report)
		value |= SKY81296_FLT_INH;

	if (flash->pdata->shutoff_on_inhibit_mode)
		value |= SKY81296_FLINHM;

	if (flash->pdata->enable_voltage_monitor)
		value |= SKY81296_VINM;

	ret = regmap_write(map, SKY81296_CONTROL2, value);
	if (IS_ERR_VALUE(ret))
		return ret;


	if (!flash->pdata->enable_voltage_monitor){
		flash->map = map;
		return 0;
	}

	if (flash->pdata->input_voltage_threshold < 2800 ||
			flash->pdata->input_voltage_threshold > 3900)
		return -EINVAL;

	value = ((flash->pdata->input_voltage_threshold - 2800) / 100)
			<< CTZ(SKY81296_VINTH);
	if (flash->pdata->input_voltage_hysteresis < 2900 ||
			flash->pdata->input_voltage_hysteresis > 4000)
		return -EINVAL;

	value |= ((flash->pdata->input_voltage_hysteresis - 2900) / 100)
			<< CTZ(SKY81296_VINHYS);

	ret = regmap_write(map, SKY81296_CONTROL3, value);


	flash->map = map;

	return ret;
}

static int __sky81296_s_power(struct sky81296 *flash, int power)
{
	int ret;

	printk("%s:start\n",__func__);

	if (first_open) {
		if (power) {
			if (!camera_vprog1_on) {
				camera_vprog1_on = 1;
				ret = regulator_enable(vprog1_reg);
				if (!ret) {
					pr_info("%s enable regulator vprog1 1.8V\n", __func__);
				} else {
					pr_err("%s Failed to enable regulator vprog1\n", __func__);
					return ret;
				}
			}
		} else {
			if (camera_vprog1_on) {
				camera_vprog1_on = 0;
				first_open = false;
				ret = regulator_disable(vprog1_reg);
				if (!ret) {
					pr_info("disable regulator vprog1 1.8V\n");
				} else {
					pr_err("Failed to disable regulator vprog1\n");
				return ret;
				}
			}
		}
	}

	printk("%s:end\n",__func__);

	return 0;
}

static int sky81296_s_power(struct v4l2_subdev *sd, int power)
{
	struct sky81296 *flash = to_sky81296(sd);
	int ret = 0;

	printk("%s:start\n",__func__);
	mutex_lock(&flash->power_lock);

	if (flash->power_count == !power) {
		ret = __sky81296_s_power(flash, !!power);
		if (ret < 0)
			goto done;
	}

	flash->power_count += power ? 1 : -1;
	WARN_ON(flash->power_count < 0);

	printk("%s:end\n",__func__);
done:
	mutex_unlock(&flash->power_lock);
	return ret;
}

static void sky81296_torch_on(struct sky81296 *flash , int intensity){
	int ret;
	u16 flash_val;
	
	if(intensity < SKY81296_TORCH_CURRENT_25MA || intensity > SKY81296_TORCH_CURRENT_250MA ){
		intensity = SKY81296_TORCH_CURRENT_250MA;
	}
	printk("[AsusFlash] Set Torch on %u \n", ( intensity << 4 | intensity ));
	ret = regmap_write(flash->map,SKY81296_MOVIE_MODE_CURRENT,( intensity << 4 | intensity ) );
	ret = regmap_write(flash->map,SKY81296_CONTROL1,0x01);
}

static void sky81296_flash_on(struct sky81296 *flash){
	int ret;
	printk("[AsusFlash] Set Flash on \n");
	ret = regmap_write(flash->map, SKY81296_CONTROL1, 0x02);
}

static void sky81296_flash_off(struct sky81296 *flash){
	int ret;
	printk("[AsusFlash] Set Flash off \n");
	ret = regmap_write(flash->map, SKY81296_CONTROL1, 0x0);
}

static long sky81296_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{

	struct sky81296 *flash = to_sky81296(sd);
	int input_arg = 0;

	switch (cmd) {
	case ATOMISP_FLASH_MODE_TORCH:
		input_arg = *(int *)arg;
		if(input_arg == 0){
			sky81296_flash_off(flash);
		}else{
			sky81296_torch_on(flash , 4);
		}
		return 0;
	case ATOMISP_FLASH_MODE_FLASH:
		input_arg = *(int *)arg;
		if(input_arg == 0){
			sky81296_flash_off(flash);
		}else{
			sky81296_flash_on(flash);
		}
		return 0;
	default:
		return -EINVAL;
	}

	return 0;
}

/*For ASUS FLASH BRIGHTNESS+++*/
#define	FLASH_BRIGHTNESS_PROC_FILE	"driver/asus_flash_brightness"
static struct proc_dir_entry *flash_brightness_proc_file;
static int last_flash_brightness_value;

static int flash_brightness_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", last_flash_brightness_value);
    return 0;
}

static int flash_brightness_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, flash_brightness_proc_read, NULL);
}

static ssize_t flash_brightness_proc_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	int set_val = -1, now_flash_brightness_value = -1;
	int torch_map_offset = 25;
	int map_num = -1, ret;


	sscanf(buff, "%d", &now_flash_brightness_value);
	set_val = now_flash_brightness_value * 125 / 99;
	pr_info("[AsusFlashBrightness]flash brightness value=%d\n", set_val);

	if (last_flash_brightness_value == now_flash_brightness_value) {
		printk(KERN_INFO "[AsusFlashBrightness] now_flash_brightness_value = last_flash_brightness_value so donothing\n");
		return len;
	}

	if (!camera_vprog1_on) {
		camera_vprog1_on = 1;
		ret = regulator_enable(vprog1_reg);
		if (!ret) {
			pr_info("%s enable regulator vprog1 1.8V\n", __func__);
		} else {
			pr_err("%s Failed to enable regulator vprog1\n", __func__);
			return ret;
		}
	}

	last_flash_brightness_value = now_flash_brightness_value;
	if (set_val > 125) {
		/*sky81296_flash_off(inner_sky81296);*/
		sky81296_torch_on(inner_sky81296, SKY81296_TORCH_CURRENT_125MA);
		map_num = SKY81296_TORCH_CURRENT_125MA;
	} else if (set_val <= 0) {
		sky81296_flash_off(inner_sky81296);
		last_flash_brightness_value = 0;
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				pr_info("disable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to disable regulator vprog1\n");
			return ret;
			}
		}
	} else if (0 < set_val && set_val < 126) {
		printk(KERN_INFO "[AsusFlashBrightness] current now in 1~125");
		/*sky81296_flash_off(inner_sky81296);*/
		/*map_num = set_val - (set_val % torch_map_offset);
		map_num = (map_num / (25 * torch_map_offset))-3;*/
		map_num = (set_val / torch_map_offset)-1;
		if (map_num < 0)
			map_num = 0;
		sky81296_torch_on(inner_sky81296, map_num);
	} else {
		sky81296_flash_off(inner_sky81296);
		last_flash_brightness_value = 0;
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				pr_info("disable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to disable regulator vprog1\n");
			return ret;
			}
		}
		return -1;
	}
	printk(KERN_INFO "[AsusFlashBrightness] Real set torch current  to %d\n", map_num);
	return len;
}

static const struct file_operations flash_brightness_fops = {
	.owner = THIS_MODULE,
	.open = flash_brightness_proc_open,
	.read = seq_read,
	.write = flash_brightness_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_flash_brightness_proc_file(void)
{
	last_flash_brightness_value = 0;
	flash_brightness_proc_file = proc_create(FLASH_BRIGHTNESS_PROC_FILE, 0666, NULL, &flash_brightness_fops);
	if (flash_brightness_proc_file) {
		printk("%s sucessed!\n", __func__);
	} else {
		printk("%s failed!\n", __func__);
	}
}
/*For ASUS FLASH BRIGHTNESS---*/

int ATD_sky81296_status;
/*For FLASH STATUS Controll+++*/
#define	STATUS_PROC_FILE	"driver/flash_status"
static struct proc_dir_entry *status_proc_file;

static int status_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", ATD_sky81296_status);
	ATD_sky81296_status = 0;
    return 0;
}

static int status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, status_proc_read, NULL);
}

static ssize_t status_proc_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	pr_info("command not support\n");
	return len;
}

static const struct file_operations status_fops = {
	.owner = THIS_MODULE,
	.open = status_proc_open,
	.read = seq_read,
	.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_status_proc_file(void)
{
    status_proc_file = proc_create(STATUS_PROC_FILE, 0666, NULL, &status_fops);
    if (status_proc_file) {
	printk("%s sucessed!\n", __func__);
    } else {
	printk("%s failed!\n", __func__);
    }
}
/*For FLASH STATUS Controll---*/

static ssize_t sky81296_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
#if 0
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	printk(KERN_INFO "[AsusFlash] Read Flash %d\n", light_record);
	buff = kmalloc(100,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

	len += sprintf(buff+len, "%d\n", light_record);
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
	kfree(buff);
#endif
	return 0;
}

static ssize_t sky81296_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	int mode = -1, set_val = -1, set_val2 = -1;
	int torch_map_offset = 25;
	int flash_current_map_offset = 50, flash_current_base = 250, flash_current_default = 1000;
	int flash_timeout_map_offset = 95;
	int map_num = -1, ret;

	sscanf(buf, "%d %d %d", &mode, &set_val, &set_val2);
	pr_info("[AsusFlash]flash mode=%d value=%d value=%d\n", mode, set_val, set_val2);

	if (!camera_vprog1_on) {
		camera_vprog1_on = 1;
		ret = regulator_enable(vprog1_reg);
		if (!ret) {
			pr_info("%s enable regulator vprog1 1.8V\n", __func__);
		} else {
			pr_err("%s Failed to enable regulator vprog1\n", __func__);
			return ret;
		}
	}

	if(mode == 0) {
		if (set_val < 0 || set_val > 250 || set_val == 1) {
			sky81296_flash_off(inner_sky81296);
			sky81296_torch_on(inner_sky81296, SKY81296_TORCH_CURRENT_200MA);
			ATD_sky81296_status = 1;
			map_num = SKY81296_TORCH_CURRENT_200MA;
		} else if (set_val == 0 ) {
			sky81296_flash_off(inner_sky81296);
			ATD_sky81296_status = 1;
			if (camera_vprog1_on) {
				camera_vprog1_on = 0;
				ret = regulator_disable(vprog1_reg);
				if (!ret) {
					pr_info("disable regulator vprog1 1.8V\n");
				} else {
					pr_err("Failed to disable regulator vprog1\n");
				return ret;
				}
			}
		} else if(0 < set_val && set_val < 251) {
			printk(KERN_INFO "[AsusFlash] current now in 1~250");
			sky81296_flash_off(inner_sky81296);
			/*map_num = set_val - (set_val % torch_map_offset);
			map_num = (map_num / (25 * torch_map_offset))-3;*/
			map_num = (set_val / torch_map_offset)-1;
			if (map_num < 0)
				map_num = 0;
			sky81296_torch_on(inner_sky81296,map_num);
		} else {
			sky81296_flash_off(inner_sky81296);
			if (camera_vprog1_on) {
				camera_vprog1_on = 0;
				ret = regulator_disable(vprog1_reg);
				if (!ret) {
					pr_info("disable regulator vprog1 1.8V\n");
				} else {
					pr_err("Failed to disable regulator vprog1\n");
				return ret;
				}
			}
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set torch current  to %d\n", map_num);
	} else if(mode == 1) {
		if (set_val == 1 || set_val < 0 || set_val > 1500) {
			sky81296_s_flash_intensity(inner_sky81296, SKY81296_FLASH_CURRENT_1000MA);
			map_num = SKY81296_FLASH_CURRENT_1000MA;
			ATD_sky81296_status = 1;
		} else if (set_val == 0) {
			set_val2 = 0;
		} else if (0 < set_val && set_val < 1501) {
			printk(KERN_INFO "[AsusFlash] Flash current now in 1~1500");
			sky81296_flash_off(inner_sky81296);
			if (set_val < 1001)
				map_num = ((set_val - flash_current_base) / flash_current_map_offset);
			else
				map_num = SKY81296_FLASH_CURRENT_1000MA + (set_val - flash_current_default) / 100;
			if (map_num < 0)
				map_num = 0;
			sky81296_s_flash_intensity(inner_sky81296, map_num);
		} else {
			sky81296_flash_off(inner_sky81296);
			if (camera_vprog1_on) {
				camera_vprog1_on = 0;
				ret = regulator_disable(vprog1_reg);
				if (!ret) {
					pr_info("disable regulator vprog1 1.8V\n");
				} else {
					pr_err("Failed to disable regulator vprog1\n");
				return ret;
				}
			}
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set flash current to %d\n", map_num);
		if (set_val2 == -1) {
			set_val2 = 1;
		}
		if (set_val2 < 0 || set_val2 > 1425 || set_val2 == 1) {
			sky81296_flash_off(inner_sky81296);
			sky81296_s_flash_timeout(inner_sky81296, SKY81296_FLASHTIMEOUT_95MS);
			sky81296_flash_on(inner_sky81296);
			/*sky81296_flash_off(inner_sky81296);*/
			map_num = SKY81296_FLASHTIMEOUT_95MS;
		} else if (set_val2 == 0) {
			sky81296_flash_off(inner_sky81296);
		} else if (0 < set_val2 && set_val2 < 1426) {
			printk(KERN_INFO "[AsusFlash] Flash time out now in 1~1425");
			sky81296_flash_off(inner_sky81296);
			/*map_num = set_val - (set_val % flash_map_offset);
			map_num = (map_num / (5 * flash_map_offset))-2;*/
			map_num = (set_val2 / flash_timeout_map_offset);
			if (map_num < 0)
				map_num = 0;	
			sky81296_s_flash_timeout(inner_sky81296, map_num);
			sky81296_flash_on(inner_sky81296);
			/*sky81296_flash_off(inner_sky81296);*/
		} else {
			sky81296_flash_off(inner_sky81296);
			if (camera_vprog1_on) {
				camera_vprog1_on = 0;
				ret = regulator_disable(vprog1_reg);
				if (!ret) {
					pr_info("disable regulator vprog1 1.8V\n");
				} else {
					pr_err("Failed to disable regulator vprog1\n");
				return ret;
				}
			}
			return -1;
		}
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				pr_info("disable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to disable regulator vprog1\n");
			return ret;
			}
		}
		printk(KERN_INFO "[AsusFlash] Real set flash time out to %d\n", map_num);
	} else {
		sky81296_flash_off(inner_sky81296);
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				pr_info("disable regulator vprog1 1.8V\n");
			} else {
				pr_err("Failed to disable regulator vprog1\n");
			return ret;
			}
		}
		return -1;
	}

	return count;
}

static const struct file_operations flash_proc_fops = {
	.read = sky81296_show,
	.write = sky81296_store,
};

static const struct v4l2_subdev_core_ops sky81296_core_ops = {
	.queryctrl = sky81296_queryctrl,
	.g_ctrl = sky81296_g_ctrl,
	.s_ctrl = sky81296_s_ctrl,
	.s_power = sky81296_s_power,
	.ioctl = sky81296_ioctl,
};

static const struct v4l2_subdev_ops sky81296_ops = {
	.core = &sky81296_core_ops,
};


static int sky81296_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_adapter *adapter = client->adapter;
	struct sky81296 *flash = to_sky81296(sd);
	/*struct sky81296_platform_data *pdata = flash->pdata;*/
	int ret;
	printk("%s:start\n",__func__);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "sky81296_detect i2c error\n");
		return -ENODEV;
	}

	/* Power up the flash driver and reset it */
	ret = sky81296_s_power(&flash->sd, 1);
	if (ret < 0)
		return ret;

	/* Setup default values. This makes sure that the chip is in a known
	 * state.
	 */
	ret = sky81296_setup(flash); //Now Not-ready
	if (ret < 0)
		goto fail;

	dev_dbg(&client->dev, "Successfully detected sky81296 LED flash\n");
	sky81296_s_power(&flash->sd, 0);
#if 0
	ret = gpio_direction_output(pdata->i2c_control_enable, 0);
	if (ret < 0) {
		printk("%s:enable gpio i2c_control_enable fail\n", __func__);
		gpio_free(pdata->i2c_control_enable);
	}
#endif
	printk("%s:endn",__func__);
	return 0;

fail:
	sky81296_s_power(&flash->sd, 0);
	return ret;
}

static int sky81296_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	printk("%s:start\n",__func__);
	return sky81296_s_power(sd, 1);
}

static int sky81296_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	printk("%s:start\n",__func__);
	return sky81296_s_power(sd, 0);
}

static const struct v4l2_subdev_internal_ops sky81296_internal_ops = {
	.registered = sky81296_detect,
	.open = sky81296_open,
	.close = sky81296_close,
};

static int sky81296_gpio_init(struct i2c_client *client)
{
	int ret;

	printk("%s:start\n",__func__);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sky81296 *flash = to_sky81296(sd);
	struct sky81296_platform_data *pdata = flash->pdata;

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
	gpio_free(pdata->i2c_control_enable);
	ret = gpio_request(pdata->i2c_control_enable, FLED_DRIVER_ENT);
	if (ret < 0) {
		dev_err(&client->dev, "FLED_DRIVER_ENT1 failed\n");
		return ret;
	}
	gpio_free(pdata->flash_enable);
	ret = gpio_request(pdata->flash_enable, FLED_DRIVER_ENF);
	if (ret < 0) {
		dev_err(&client->dev, "FLED_DRIVER_ENF1 failed\n");
		return ret;
	}
	ret = gpio_direction_output(pdata->i2c_control_enable, 1);
	if (ret < 0) {
		dev_err(&client->dev, "gpio_direction_output failed\n");
		goto err_init;
	}
#if 0
	ret = gpio_direction_output(pdata->flash_enable, 0);
	if (ret < 0)
		goto err_init;
#endif
	printk("%s:end\n",__func__);
	return 0;

err_init:
	regulator_put(vprog1_reg);
	gpio_free(pdata->i2c_control_enable);
	gpio_free(pdata->flash_enable);
	return ret;
}

static int sky81296_gpio_uninit(struct i2c_client *client)
{
	int ret;

	printk("%s:start\n",__func__);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sky81296 *flash = to_sky81296(sd);
	struct sky81296_platform_data *pdata = flash->pdata;
	ret = gpio_direction_output(pdata->i2c_control_enable, 0);
	if (ret < 0)
		return ret;
#if 0
	ret = gpio_direction_output(pdata->flash_enable, 0);
	if (ret < 0)
		return ret;
	regulator_put(vprog1_reg);
	gpio_free(pdata->flash_enable);
#endif
	gpio_free(pdata->i2c_control_enable);
	printk("%s:end\n",__func__);
	
	return 0;
}


static int sky81296_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	struct sky81296 *flash;
	struct proc_dir_entry* proc_entry_flash;
	void* dummy = NULL;

	printk("%s:start\n",__func__);
	first_open = true;
	ATD_sky81296_status = 0;
	create_status_proc_file();
	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "no platform data\n");
		return -ENODEV;
	}

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (!flash) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	flash->pdata = client->dev.platform_data;

	v4l2_i2c_subdev_init(&flash->sd, client, &sky81296_ops);//Now
	flash->sd.internal_ops = &sky81296_internal_ops;
	flash->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->mode = SKY81296_MODE_SHUTDOWN;


	err = media_entity_init(&flash->sd.entity, 0, NULL, 0);
	if (err) {
		dev_err(&client->dev, "error initialize a media entity.\n");
		goto fail1;
	}

	flash->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_FLASH;

	mutex_init(&flash->power_lock);

	setup_timer(&flash->flash_off_delay, sky81296_flash_off_delay,(unsigned long)client);

	err = sky81296_gpio_init(client);
	if (err) {
		dev_err(&client->dev, "gpio request/direction_output fail");
		goto fail2;
	}

//Add node for flash control
	proc_entry_flash = proc_create_data("driver/asus_flash", 0666, NULL, &flash_proc_fops, dummy);
	proc_set_user(proc_entry_flash, 1000, 1000);
	inner_sky81296 = flash;

	create_flash_brightness_proc_file();/*For ASUS FLASH BRIGHTNESS*/

	printk("%s:end\n",__func__);
	return 0;
fail2:
	media_entity_cleanup(&flash->sd.entity);
fail1:
	v4l2_device_unregister_subdev(&flash->sd);
	kfree(flash);

	return err;
}

static int sky81296_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sky81296 *flash = to_sky81296(sd);
	int ret;

	media_entity_cleanup(&flash->sd.entity);
	v4l2_device_unregister_subdev(sd);


	ret = sky81296_gpio_uninit(client);
	if (ret < 0)
		goto fail;

	kfree(flash);

	return 0;
fail:
	dev_err(&client->dev, "gpio request/direction_output fail");
	return ret;
}

static const struct i2c_device_id sky81296_ids[] = {
	{SKY81296_NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, sky81296_ids);

static const struct dev_pm_ops sky81296_pm_ops = {
	.suspend = sky81296_suspend,
	.resume = sky81296_resume,
};

static struct i2c_driver sky81296_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SKY81296_NAME,
		.pm   = &sky81296_pm_ops,
	},
	.probe = sky81296_probe,
	.remove = sky81296_remove,
	.id_table = sky81296_ids,
};

static __init int init_sky81296(void)
{
	return i2c_add_driver(&sky81296_driver);
}

static __exit void exit_sky81296(void)
{
	i2c_del_driver(&sky81296_driver);
}

module_init(init_sky81296);
module_exit(exit_sky81296);
MODULE_AUTHOR("Chung-Yi Chou <chung-yi_chou@asus.com>");
MODULE_DESCRIPTION("SKY81296 LED Flash Driver");
MODULE_LICENSE("GPL");

