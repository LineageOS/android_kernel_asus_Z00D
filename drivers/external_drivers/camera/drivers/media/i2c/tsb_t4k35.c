/*
 * Support for Toshiba T4K35(8MP) camera sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <asm/intel-mid.h>
#include <linux/atomisp_platform.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/libmsrlisthelper.h>
#include <linux/mm.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include "tsb_t4k35.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

struct v4l2_subdev *main_sd;
static char camera_module_otp[120];

static enum atomisp_bayer_order t4k35_bayer_order_mapping[] = {
#if 0	// debug bayer 20140609
	atomisp_bayer_order_rggb,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_gbrg,
	atomisp_bayer_order_bggr
#else	// debug bayer 20140609
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg
#endif	// debug bayer 20140609
};
#if 0
/*For LED FLASH Controll+++*/
#if 0
#define FLED_DRIVER_ENT "FLED_DRIVER_FNT#"
#define FLED_DRIVER_ENF "MOD_TXPWRTH"
#endif
#define FLED_DRIVER_ENT "FLED_DRIVER_EN"
#define FLED_DRIVER_ENF "FLASH_FLEN"

static int flash_ent;
static int flash_enf;
/*For LED FLASH Controll---*/
#endif
static int lowlight_status;/*ASUS_BSP PJ "add lowlight mode control binning weight iterface"*/
int ATD_t4k35_status;

static int t4k35_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[T4K35_SHORT_MAX];
	int err, i;

	//printk("[DEBUG] %s, start\n", __func__);

	if (len > T4K35_BYTE_MAX) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));
	memset(data, 0 , sizeof(data));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	/* high byte comes first */
	if (len == T4K35_8BIT) {
		*val = (u8)data[0];
	} else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int t4k35_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

	//printk("[DEBUG] %s, start\n", __func__);
#if 0
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);
#endif
again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying i2c write transfer... %d",
			retry);
		retry++;
		msleep(20);
		goto again;
	}

	return ret == num_msg ? 0 : -EIO;
}

int t4k35_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	//printk("[DEBUG] %s, start\n", __func__);

	if (data_length != T4K35_8BIT && data_length != T4K35_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == T4K35_8BIT)
		data[2] = (u8)(val);
	else {
		/* T4K35_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = t4k35_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

/*
 * t4k35_write_reg_array - Initializes a list of t4k35 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __t4k35_flush_reg_array, __t4k35_buf_reg_array() and
 * __t4k35_write_reg_is_consecutive() are internal functions to
 * t4k35_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __t4k35_flush_reg_array(struct i2c_client *client,
				     struct t4k35_write_ctrl *ctrl)
{
	u16 size;

	//printk("[DEBUG] %s, start\n", __func__);

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return t4k35_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __t4k35_buf_reg_array(struct i2c_client *client,
				   struct t4k35_write_ctrl *ctrl,
				   const struct t4k35_reg *next)
{
	int size;
	u16 *data16;

	//printk("[DEBUG] %s, start\n", __func__);

	switch (next->type) {
	case T4K35_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case T4K35_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= T4K35_MAX_WRITE_BUF_SIZE)
		return __t4k35_flush_reg_array(client, ctrl);

	return 0;
}

static int
__t4k35_write_reg_is_consecutive(struct i2c_client *client,
				   struct t4k35_write_ctrl *ctrl,
				   const struct t4k35_reg *next)
{
	//printk("[DEBUG] %s, start\n", __func__);

	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->sreg;
}

static int t4k35_write_reg_array(struct i2c_client *client,
				   const struct t4k35_reg *reglist)
{
	const struct t4k35_reg *next = reglist;
	struct t4k35_write_ctrl ctrl;
	int err;

	//printk("[DEBUG] %s, start\n", __func__);

	ctrl.index = 0;
	for (; next->type != T4K35_TOK_TERM; next++) {
		switch (next->type & T4K35_TOK_MASK) {
		case T4K35_TOK_DELAY:
			err = __t4k35_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;

		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__t4k35_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __t4k35_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __t4k35_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __t4k35_flush_reg_array(client, &ctrl);
}

static int __t4k35_min_fps_diff(int fps, const struct t4k35_fps_setting *fps_list)
{
	int diff = INT_MAX;
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	if (fps == 0)
		return 0;

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (abs(fps_list[i].fps - fps) < diff)
			diff = abs(fps_list[i].fps - fps);
	}

	return diff;
}

static int __t4k35_nearest_fps_index(int fps,
					const struct t4k35_fps_setting *fps_list)
{
	int fps_index = 0;
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (abs(fps_list[i].fps - fps)
		    < abs(fps_list[fps_index].fps - fps))
			fps_index = i;
	}
	return fps_index;
}

/*
 * This is to choose the nearest fps setting above the requested fps
 * fps_list should be in ascendant order.
 */
static int __t4k35_above_nearest_fps_index(int fps,
					const struct t4k35_fps_setting *fps_list)
{
	int fps_index = 0;
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (fps <= fps_list[i].fps) {
			fps_index = i;
			break;
		}
	}

	return fps_index;
}

static int __t4k35_get_max_fps_index(
				const struct t4k35_fps_setting *fps_settings)
{
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (fps_settings[i].fps == 0)
			break;
	}

	return i - 1;
}

static int __t4k35_update_exposure_timing(struct i2c_client *client, u16 exposure,
			u16 llp, u16 fll)
{
	int ret = 0;

	//printk("[DEBUG] %s, start\n", __func__);

	/* Increase the VTS to match exposure + margin */
	if (exposure > fll - T4K35_INTEGRATION_TIME_MARGIN)
		fll = exposure + T4K35_INTEGRATION_TIME_MARGIN;

	ret = t4k35_write_reg(client, T4K35_16BIT, T4K35_LINE_LENGTH_PIXELS, llp);
	if (ret)
		return ret;

	ret = t4k35_write_reg(client, T4K35_16BIT, T4K35_FRAME_LENGTH_LINES, fll);
	if (ret)
		return ret;

	if (exposure)
		ret = t4k35_write_reg(client, T4K35_16BIT,
			T4K35_COARSE_INTEGRATION_TIME, exposure);
	return ret;
}

static int __t4k35_update_gain(struct v4l2_subdev *sd, u16 gain)
{
	//struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

	/* set global gain */
	//ret = t4k35_write_reg(client, T4K35_8BIT, T4K35_GLOBAL_GAIN, gain);
	ret = t4k35_write_reg(client, T4K35_16BIT, T4K35_GLOBAL_GAIN, gain);
	if (ret)
		return ret;

#if 0	// T4K35
	/* set short analog gain */
	if (dev->sensor_id == IMX135_ID)
		ret = imx_write_reg(client, IMX_8BIT, IMX_SHORT_AGC_GAIN, gain);
#endif	// T4K35

	return ret;
}

static int __t4k35_update_digital_gain(struct i2c_client *client, u16 digitgain)
{
	struct t4k35_write_buffer digit_gain;

	//printk("[DEBUG] %s, start\n", __func__);

	digit_gain.addr = cpu_to_be16(T4K35_DGC_ADJ);
	digit_gain.data[0] = (digitgain >> 8) & 0xFF;
	digit_gain.data[1] = digitgain & 0xFF;
	digit_gain.data[2] = (digitgain >> 8) & 0xFF;
	digit_gain.data[3] = digitgain & 0xFF;
	digit_gain.data[4] = (digitgain >> 8) & 0xFF;
	digit_gain.data[5] = digitgain & 0xFF;
	digit_gain.data[6] = (digitgain >> 8) & 0xFF;
	digit_gain.data[7] = digitgain & 0xFF;

	return t4k35_i2c_write(client, T4K35_DGC_LEN, (u8 *)&digit_gain);
}
#if 0
static int t4k35_real_to_register_gain(u16 gain, u16 *real_gain)
{

	u16 reg_gain;
	u16 cg, asc1;
	u16 gain_value;
	u16 calculated_gain;
#if 0
	 if (_gain > 255)
		gain = 255 * 256; /*Cap max gain to use only analog portion.*/
	else
		gain = _gain * 256;
#endif
		if (gain < 22) {/*Use 2nd stage as soon as possible*/
			calculated_gain = gain * 2;
			cg = 0x0;
			asc1 = 0x0;
		} else if (gain < 32) {
			calculated_gain = gain * 6 / 4;
			cg = 0x0;
			asc1 = 0x100;
		} else if (gain < 43) {
			calculated_gain = gain;
			cg = 0x800;
			asc1 = 0x0;
		} else if (gain < 48) {
			calculated_gain = gain * 768 / 0x400;
			cg = 0x800;
			asc1 = 0x100;
		} else if (gain < 64) {
			calculated_gain = gain * 682 / 0x400;
			cg = 0x400;
			asc1 = 0x0;
		} else if (gain < 85) {
			calculated_gain = gain * 2 / 4;
			cg = 0xc00;
			asc1 = 0x0;
		} else if (gain < 128) {
			calculated_gain = gain * 384 / 0x400;
			cg = 0xc00;
			asc1 = 0x100;
		} else {
			calculated_gain = gain / 4;
			cg = 0xc00;
			asc1 = 0x200;
		}
		reg_gain = calculated_gain;
		/*printk("%s gain %d calculated_gain %x\n", __func__, gain, reg_gain);*/
		reg_gain |= cg;
		reg_gain |= asc1;
		*real_gain = reg_gain;

	return 0;
}

static long t4k35_set_exposure(struct v4l2_subdev *sd, u16 coarse_itg,
				 u16 fine_itg, u16 a_gain, u16 d_gain)

{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 frame_length;
	u16 real_gain;
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	mutex_lock(&dev->input_lock);

	/* enable group hold */
	ret = t4k35_write_reg_array(client, t4k35_param_hold);
	if (ret)
		goto out;

	frame_length = dev->curr_res_table[dev->fmt_idx].lines_per_frame;
	if (frame_length < coarse_itg)
		frame_length = coarse_itg + 5;

	/* set frame length lines */
	ret = t4k35_write_reg(client, T4K35_16BIT,
		T4K35_FRAME_LENGTH_LINES, frame_length);
	if (ret)
		goto out_disable;

	/* set coarse integration time */
	ret = t4k35_write_reg(client, T4K35_16BIT,
			T4K35_COARSE_INTEGRATION_TIME, coarse_itg);
	if (ret)
		goto out_disable;

	/* set fine integration time */
	/*ret = t4k35_write_reg(client, T4K35_16BIT,
			T4K35_FINE_INTEGRATION_TIME, fine_itg);
	if (ret)
		goto out_disable;*/

	ret = t4k35_real_to_register_gain(a_gain, &real_gain);
	real_gain |= 0x1000;

/*	printk("%s write gain %x fine %x coarse %x frame_length %x\n", __func__,
		real_gain, fine_itg, coarse_itg, frame_length);*/

	/* set global gain */
	ret = t4k35_write_reg(client, T4K35_16BIT,
			T4K35_GLOBAL_GAIN, real_gain);

	if (ret)
		goto out_disable;


	dev->gain       = real_gain;
	dev->coarse_itg = coarse_itg;
	dev->fine_itg   = fine_itg;

out_disable:
	/* disable group hold */
	t4k35_write_reg_array(client, t4k35_param_update);
out:
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long t4k35_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	u16 coarse_itg, fine_itg, analog_gain, digital_gain;

	coarse_itg = exposure->integration_time[0];
	fine_itg = exposure->integration_time[1];
	analog_gain = exposure->gain[0];
	digital_gain = exposure->gain[1];

	/* we should not accept the invalid value below */
	if (fine_itg == 0 || analog_gain == 0) {
		struct i2c_client *client = v4l2_get_subdevdata(sd);
		v4l2_err(client, "%s: invalid value\n", __func__);
		return -EINVAL;
	}

	/*printk("%s analog %x digital %x\n", __func__, analog_gain, digital_gain);*/
	return t4k35_set_exposure(sd, coarse_itg, fine_itg, analog_gain, digital_gain);
}
#endif
static int t4k35_set_exposure_gain(struct v4l2_subdev *sd, u16 coarse_itg,
	u16 gain, u16 digitgain)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	/*printk("[DEBUG] %s, coarse %d gain %d d_gain %d start\n", __func__, coarse_itg, gain, digitgain);*/

	/* Validate exposure:  cannot exceed VTS-6 where VTS is 16bit */
	coarse_itg = clamp_t(u16, coarse_itg, 0, T4K35_MAX_EXPOSURE_SUPPORTED);

	/* Validate gain: must not exceed maximum 12bit value */
	gain = clamp_t(u16, gain, 40, T4K35_MAX_GLOBAL_GAIN_SUPPORTED);

	/* Validate digital gain: must not exceed 10 bit value*/
	digitgain = clamp_t(u16, digitgain, 256, T4K35_MAX_DIGITAL_GAIN_SUPPORTED);

	mutex_lock(&dev->input_lock);

	ret = t4k35_write_reg_array(client, t4k35_param_hold);

	ret = __t4k35_update_exposure_timing(client, coarse_itg,
			dev->pixels_per_line, dev->lines_per_frame);
	if (ret)
		goto out;
	dev->coarse_itg = coarse_itg;

#if 0	// T4K35
	if (dev->sensor_id == IMX175_ID)
		ret = __imx_update_gain(sd, dev->gain);
	else
#endif	// T4K35
		ret = __t4k35_update_gain(sd, gain);
	if (ret)
		goto out;
	dev->gain = gain;

#if 0	// T4K35
	if ((dev->sensor_id == IMX175_ID) && dev->digital_gain)
		ret = __imx_update_digital_gain(client, dev->digital_gain);
	else
#endif	// T4K35
		ret = __t4k35_update_digital_gain(client, digitgain);
	if (ret)
		goto out;
	dev->digital_gain = digitgain;

out:
	ret = t4k35_write_reg_array(client, t4k35_param_update);
	mutex_unlock(&dev->input_lock);
	return ret;
}

static long t4k35_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	/*printk("[DEBUG] %s, start\n", __func__);*/

	return t4k35_set_exposure_gain(sd, exposure->integration_time[0],
				exposure->gain[0], exposure->gain[1]);
}

static int __t4k35_otp_read(struct v4l2_subdev *sd, struct t4k35_af_data *buf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 i, check_status, check_data1, check_data2, check_count;
	int page_num;
	u32 read_value[24];

	page_num = 2;
	ret = t4k35_write_reg(client, T4K35_8BIT, 0x3500, 0x01);
	if (ret) {
		v4l2_err(client, "%s: failed to write 0x3500 = 0x01\n",
			 __func__);
		return ret;
	}

	do {
		check_count = 0;
		ret = t4k35_write_reg(client, T4K35_8BIT, 0x3502, page_num);
		usleep_range(300, 500);
		pr_info("PJotp now read bank %d = 0x3502 + %d \n", page_num+1, page_num);
		if (ret) {
			v4l2_err(client, "%s: failed to write 0x3502 = %d\n",
				 __func__, page_num);
			return ret;
		}
		page_num--;
		ret = t4k35_write_reg(client, T4K35_8BIT, 0x3500, 0x81);
		if (ret) {
			v4l2_err(client, "%s: failed to write 0x3500 = 0x81\n",
				 __func__);
			return ret;
	}
		do {
			printk("%s : check access status ...\n", __func__);
			t4k35_read_reg(client, T4K35_8BIT, 0x3500, &check_status);
			usleep_range(300, 500);
			check_count++;
		} while (((check_status & 0x80) != 0) && (check_count <= 10));

		t4k35_read_reg(client, T4K35_8BIT, 0x3504, &check_data1);
		usleep_range(300, 500);
		t4k35_read_reg(client, T4K35_8BIT, 0x3505, &check_data2);
		usleep_range(300, 500);
	} while ((page_num >= 0) && ((check_data1 == 0x0 && check_data2 == 0x0) || (check_data1 == 0xff && check_data2 == 0xff)));

	t4k35_read_reg(client, T4K35_8BIT, 0x3546, &check_status);
	if ((check_status & 0x08) == 1) {
		printk("%s : check error status fail\n", __func__);
		ret = t4k35_write_reg(client, T4K35_8BIT, 0x3500, 0x00);
		if (ret) {
			v4l2_err(client, "%s: failed to write 0x3500 = 0x00\n",
				 __func__);
			return ret;
		}
		return -1;
	}

	//Read OTP value
	for (i = 0; i < 24; i++) {
		t4k35_read_reg(client, T4K35_8BIT, 0x3504+i, &read_value[i]);
		/*camera_module_otp[i] = read_value[i];*/
	}

	//Return OTP value
	snprintf(camera_module_otp, sizeof(camera_module_otp), "0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n"
		, read_value[0]&0xFF, read_value[1]&0xFF, read_value[2]&0xFF, read_value[3]&0xFF, read_value[4]&0xFF
		, read_value[5]&0xFF, read_value[6]&0xFF, read_value[7]&0xFF, read_value[8]&0xFF, read_value[9]&0xFF
		, read_value[10]&0xFF, read_value[11]&0xFF, read_value[12]&0xFF, read_value[13]&0xFF, read_value[14]&0xFF
		, read_value[15]&0xFF, read_value[16]&0xFF, read_value[17]&0xFF, read_value[18]&0xFF, read_value[19]&0xFF
		, read_value[20]&0xFF, read_value[21]&0xFF, read_value[22]&0xFF, read_value[23]&0xFF);

	pr_info("%s OTP value: %s\n", __func__, camera_module_otp);
/*
	buf->af_inf_pos = read_value[0] | read_value[1];
	buf->af_30cm_pos = read_value[2] | read_value[3];
	buf->af_10cm_pos = read_value[4] | read_value[5];
	buf->af_start_curr = read_value[6] | read_value[7];
	buf->module_id = read_value[8];
	buf->vendor_id = read_value[9];
	buf->date_code = read_value[10] | read_value[11] | read_value[12] | read_value[13];
	buf->sn_number = read_value[14] | read_value[15] | read_value[16] | read_value[17];
	buf->pn_number = read_value[18] | read_value[19] | read_value[20] | read_value[21];
	buf->cks = read_value[22] | read_value[23];
*/
	buf->af_inf_pos = read_value[0]<<8 | read_value[1];
	buf->af_30cm_pos = read_value[2]<<8 | read_value[3];
	buf->af_10cm_pos = read_value[4]<<8 | read_value[5];
	buf->af_start_curr = read_value[6]<<8 | read_value[7];
	buf->module_id = read_value[8];
	buf->vendor_id = read_value[9];
	buf->date_code = read_value[10]<<24 | read_value[11]<<16 | read_value[12]<<8 | read_value[13];
	buf->sn_number = read_value[14]<<24 | read_value[15]<<16 | read_value[16]<<8 | read_value[17];
	buf->pn_number = read_value[18]<<24 | read_value[19]<<16 | read_value[20]<<8 | read_value[21];
	buf->cks = read_value[22]<<8 | read_value[23];

	ret = t4k35_write_reg(client, T4K35_8BIT, 0x3500, 0x00);
	if (ret) {
		v4l2_err(client, "%s: failed to write 0x3500 = 0x00\n",
			 __func__);
		return ret;
	}

	return 0;
}

static void *t4k35_otp_read(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	void *buf;
	int ret;

	buf = kmalloc(T4K35_OTP_DATA_SIZE, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	/*
	 * Try all banks in reverse order and return after first success.
	 * Last used bank has most up-to-date data.
	 */
	ret = __t4k35_otp_read(sd, buf);
	/* Driver has failed to find valid data */
	if (ret) {
		v4l2_err(client, "%s: sensor found no valid OTP data\n",
			  __func__);
		kfree(buf);
		return ERR_PTR(ret);
	}

	return buf;
}

/* FIXME -To be updated with real OTP reading */
static int t4k35_g_priv_int_data(struct v4l2_subdev *sd,
				   struct v4l2_private_int_data *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	u8 __user *to = priv->data;
	u32 read_size = priv->size;
	int ret;

	printk("[DEBUG] %s, start\n", __func__);

	/* No need to copy data if size is 0 */
	if (!read_size)
		goto out;

	if (IS_ERR(dev->otp_data)) {
		dev_err(&client->dev, "OTP data not available");
		return PTR_ERR(dev->otp_data);
	}
	/* Correct read_size value only if bigger than maximum */
	if (read_size > T4K35_OTP_DATA_SIZE)
		read_size = T4K35_OTP_DATA_SIZE;

	ret = copy_to_user(to, dev->otp_data, read_size);
	if (ret) {
		dev_err(&client->dev, "%s: failed to copy OTP data to user\n",
			 __func__);
		return -EFAULT;
	}
out:
	/* Return correct size */
	priv->size = T4K35_OTP_DATA_SIZE;

	return 0;
}

static int __t4k35_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->sensor_id == T4K35_ID_DEFAULT)
		return 0;

	/* Sets the default FPS */
	dev->fps_index = 0;
	dev->curr_res_table = dev->mode_tables->res_preview;
	dev->entries_curr_table = dev->mode_tables->n_res_preview;

	return t4k35_write_reg_array(client,
			dev->mode_tables->init_settings);
}

static int t4k35_init(struct v4l2_subdev *sd, u32 val)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	int ret = 0;

	//printk("[DEBUG] %s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	ret = __t4k35_init(sd, val);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long t4k35_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{

	/*printk("[DEBUG] %s, start\n", __func__);*/

	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return t4k35_s_exposure(sd, arg);
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return t4k35_g_priv_int_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	int ret;

	printk("[DEBUG] %s, start\n", __func__);

       /* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;
	usleep_range(1000+2000, 1100+2000);/*original:1000us+extra:2000us*/

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "gpio failed\n");
		goto fail_gpio;
	}
	usleep_range(11500, 11600);
	lowlight_status = 0;
#if 0
/*For LED FLASH Controll+++*/
	/*FLED_DRIVER_ENT 159*/
	if (flash_ent < 0) {
		ret = gpio_request(54, FLED_DRIVER_ENT);
		if (ret < 0) {
			printk("%s not available.\n", FLED_DRIVER_ENT);
			return ret;
		}
		gpio_direction_output(54, 0);
		flash_ent = 54;
	}

	/*FLED_DRIVER_ENF 161*/
	if (flash_enf < 0) {
		ret = gpio_request(53, FLED_DRIVER_ENF);
		if (ret < 0) {
			printk("%s not available.\n", FLED_DRIVER_ENF);
			return ret;
		}
		gpio_direction_output(53, 0);
		flash_enf = 53;
	}
/*For LED FLASH Controll---*/
#endif

	return 0;
fail_gpio:
	dev->platform_data->gpio_ctrl(sd, 0);
fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	printk("[DEBUG] %s, start\n", __func__);

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed\n");

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");
#if 0
/*For LED FLASH Controll+++*/
	if (flash_ent >= 0) {
		gpio_set_value(flash_ent, 0);
		printk("<<< FLED_DRIVER_ENT = 0\n");
		gpio_free(flash_ent);
		flash_ent = -1;
		/*msleep(10);*/
	}

	if (flash_enf >= 0) {
		gpio_set_value(flash_enf, 0);
		printk("<<< FLED_DRIVER_ENF = 0\n");
		gpio_free(flash_enf);
		flash_enf = -1;
		/*msleep(10);*/
	}
/*For LED FLASH Controll---*/
#endif
	return ret;
}

static int __t4k35_s_power(struct v4l2_subdev *sd, int on)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	int ret = 0;
	int r = 0;

	printk("[DEBUG] %s, start\n", __func__);

	if (on == 0) {
		ret = power_down(sd);
		/*if (dev->vcm_driver && dev->vcm_driver->power_down)
			r = dev->vcm_driver->power_down(sd);*/
		if (ret == 0)
			ret = r;
		dev->power = 0;
	} else {
#if 0	// 0704
		if (dev->vcm_driver && dev->vcm_driver->power_up)
			ret = dev->vcm_driver->power_up(sd);
		if (ret)
			return ret;
		ret = power_up(sd);
		if (!ret) {
			dev->power = 1;
			return __t4k35_init(sd, 0);
		}
#else	// 0704
		// current VCMVDD is jumpered to AVDD on conv.board,
		// so, VCM power_up() (includes AD5816 ID checking)
		// should NOT be done before sensor init (__t4k35_init)
		ret = power_up(sd);
		if (!ret) {
			dev->power = 1;
			__t4k35_init(sd, 0);
		}
		/*if (dev->vcm_driver && dev->vcm_driver->power_up)
			ret = dev->vcm_driver->power_up(sd);
		if (ret)
			return ret*/;
#endif	// 0704
	}

	return ret;
}

static int t4k35_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	printk("[DEBUG] %s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	ret = __t4k35_s_power(sd, on);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int t4k35_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	if (!chip)
		return -EINVAL;

	v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_T4K35, 0);

	return 0;
}

static int t4k35_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct t4k35_reg *reglist)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	u32 vt_pix_clk_div;
	u32 vt_sys_clk_div;
	u32 pre_pll_clk_div;
	u32 pll_multiplier;

	const int ext_clk_freq_hz = 19200000;
	struct atomisp_sensor_mode_data *buf = &info->data;
	int ret;
	u16 data[T4K35_INTG_BUF_COUNT];

	u32 vt_pix_clk_freq_mhz;
	u32 coarse_integration_time_min;
	u32 coarse_integration_time_max_margin;
	u32 read_mode;
	u32 div;

	printk("[DEBUG] %s, start\n", __func__);

	if (info == NULL)
		return -EINVAL;

	memset(data, 0, T4K35_INTG_BUF_COUNT * sizeof(u16));
	ret = t4k35_read_reg(client, 1, T4K35_VT_PIX_CLK_DIV, data);
	/*printk("[DEBUG] ret3a=%d\n", ret);*/
	if (ret)
		return ret;
#if 0	// T4K35
	vt_pix_clk_div = data[0] & IMX_MASK_5BIT;
#else	// T4K35
	vt_pix_clk_div = data[0];
	if(vt_pix_clk_div==0)
		vt_pix_clk_div = 1;
#endif	// T4K35

#if 0	// T4K35
	if (dev->sensor_id == IMX132_ID)
		ret = imx_read_reg(client, 1, IMX132_VT_RGPLTD, data);
	else
#endif	// T4K35
		ret = t4k35_read_reg(client, 1, T4K35_VT_SYS_CLK_DIV, data);
	/*printk("[DEBUG] ret3b=%d\n", ret);*/
	if (ret)
		return ret;
#if 0	// T4K35
	vt_sys_clk_div = data[0] & IMX_MASK_2BIT;
#else	// T4K35
	vt_sys_clk_div = data[0];
	if(vt_sys_clk_div==0)
		vt_sys_clk_div = 1;
#endif	// T4K35
	ret = t4k35_read_reg(client, 1, T4K35_PRE_PLL_CLK_DIV, data);
	/*printk("[DEBUG] ret3c=%d\n", ret);*/
	if (ret)
		return ret;
#if 0	// T4K35
	pre_pll_clk_div = data[0] & T4K35_MASK_4BIT;
#else	// T4K35
	pre_pll_clk_div = data[0] + 1;
#endif	// T4K35
#if 0	// T4K35
	ret = imx_read_reg(client, 2,
		(dev->sensor_id == IMX132_ID) ?
		IMX132_PLL_MULTIPLIER : IMX_PLL_MULTIPLIER, data);
#else	// T4K35
	ret = t4k35_read_reg(client, 2, T4K35_PLL_MULTIPLIER, data);
#endif	// T4K35
	/*printk("[DEBUG] ret3d=%d\n", ret);*/
	if (ret)
		return ret;
	//pll_multiplier = data[0] & IMX_MASK_11BIT;
	pll_multiplier = data[0];

	memset(data, 0, T4K35_INTG_BUF_COUNT * sizeof(u16));
#if 0	// T4K35
	ret = t4k35_read_reg(client, 4, T4K35_COARSE_INTG_TIME_MIN, data);
	if (ret)
		return ret;
	coarse_integration_time_min = data[0];
	coarse_integration_time_max_margin = data[1];
#else	// T4K35
	coarse_integration_time_min = 1;
	coarse_integration_time_max_margin = 6;
#endif	// T4K35

	/* Get the cropping and output resolution to ISP for this mode. */
#if 0	// T4K35
	ret =  imx_read_reg(client, 2, IMX_HORIZONTAL_START_H, data);
	if (ret)
		return ret;
	buf->crop_horizontal_start = data[0];
#else	// T4K35
	// temp. fixed to full-size (no-crop) for K37,K35
	buf->crop_horizontal_start = 0;
#endif	// T4K35

	ret = t4k35_read_reg(client, 2, T4K35_VERTICAL_START_H, data);
	/*printk("[DEBUG] ret3e=%d\n", ret);*/
	if (ret)
		return ret;
	buf->crop_vertical_start = data[0];
	/*printk("%s PJshading buf->crop_vertical_start = %d\n", __func__, buf->crop_vertical_start);*/

#if 0	// T4K35
	ret = imx_read_reg(client, 2, IMX_HORIZONTAL_END_H, data);
	if (ret)
		return ret;
	buf->crop_horizontal_end = data[0];
#endif	// T4K35

	ret = t4k35_read_reg(client, 2, T4K35_VERTICAL_END_H, data);
	/*printk("[DEBUG] ret3f=%d\n", ret);*/
	if (ret)
		return ret;
	buf->crop_vertical_end = data[0];
	/*printk("%s PJshading buf->crop_vertical_end = %d\n", __func__, buf->crop_vertical_end);*/

	ret = t4k35_read_reg(client, 2, T4K35_HORIZONTAL_OUTPUT_SIZE_H, data);
	/*printk("[DEBUG] ret3g=%d\n", ret);*/
	if (ret)
		return ret;
	buf->output_width = data[0];
	/*printk("%s PJ output_width = %d\n", __func__, buf->output_width);*/
#if 1	// T4K35
	// temp. fixed to full-size (no-crop) for K37,K35
	buf->crop_horizontal_end = 3279;
	/*printk("%s PJshading buf->crop_horizontal_end = %d\n", __func__, buf->crop_horizontal_end);*/
#endif	// T4K35

	ret = t4k35_read_reg(client, 2, T4K35_VERTICAL_OUTPUT_SIZE_H, data);
	/*printk("[DEBUG] ret3h=%d\n", ret);*/
	if (ret)
		return ret;
	buf->output_height = data[0];
	/*printk("%s PJ output_height = %d\n", __func__, buf->output_height);*/
	memset(data, 0, T4K35_INTG_BUF_COUNT * sizeof(u16));
#if 0	// T4K35
	if (dev->sensor_id == IMX132_ID)
		read_mode = 0;
	else {
#endif	// T4K35
		ret = t4k35_read_reg(client, 1, T4K35_H_BINNING_ENABLE, data);
	/*printk("[DEBUG] ret3i=%d\n", ret);*/
		if (ret)
			return ret;
		read_mode = data[0] & T4K35_MASK_2BIT;
//	}	// T4K35

	div = pre_pll_clk_div*vt_sys_clk_div*vt_pix_clk_div;
	/*printk("[DEBUG] ret3j=%d\n", div);*/
	if (div == 0)
		return -EINVAL;

#if 0	// T4K35
	vt_pix_clk_freq_mhz = 2 * ext_clk_freq_hz / div;
	vt_pix_clk_freq_mhz *= pll_multiplier;
#else	// T4K35
	vt_pix_clk_freq_mhz = 2 * ext_clk_freq_hz / div;
	vt_pix_clk_freq_mhz *= pll_multiplier;
#endif	// T4K35

	dev->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;

	buf->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	/*printk("[DEBUG] vt_pix_clk_freq_mhz=%d\n", buf->vt_pix_clk_freq_mhz);*/
	buf->coarse_integration_time_min = coarse_integration_time_min;
	buf->coarse_integration_time_max_margin =
				coarse_integration_time_max_margin;
	/*printk("[DEBUG] coarse_intg_min=%d\n", buf->coarse_integration_time_min);*/
	/*printk("[DEBUG] coarse_intg_max_margin=%d\n", buf->coarse_integration_time_max_margin);*/

	buf->fine_integration_time_min = T4K35_FINE_INTG_TIME;
	buf->fine_integration_time_max_margin = T4K35_FINE_INTG_TIME;
	buf->fine_integration_time_def = T4K35_FINE_INTG_TIME;
	buf->frame_length_lines = dev->lines_per_frame;
	buf->line_length_pck = dev->pixels_per_line;
	buf->read_mode = read_mode;

#if 0	// T4K35
	if (dev->sensor_id == IMX132_ID) {
		buf->binning_factor_x = 1;
		buf->binning_factor_y = 1;
	} else {
		ret = imx_read_reg(client, 1, IMX_BINNING_ENABLE, data);
		if (ret)
			return ret;
		/* 1:binning enabled, 0:disabled */
		if (data[0] == 1) {
			ret = imx_read_reg(client, 1, IMX_BINNING_TYPE, data);
			if (ret)
				return ret;
			buf->binning_factor_x = data[0] >> 4 & 0x0f;
			if (!buf->binning_factor_x)
				buf->binning_factor_x = 1;
			buf->binning_factor_y = data[0] & 0xf;
			if (!buf->binning_factor_y)
				buf->binning_factor_y = 1;
		} else {
			buf->binning_factor_x = 1;
			buf->binning_factor_y = 1;
		}
	}
#else	// T4K35
	ret = t4k35_read_reg(client, 1, T4K35_H_BINNING_ENABLE, data);
	/*printk("[DEBUG] ret3k=%d\n", ret);*/
	if (ret)
		return ret;
	/* 2:1/4binning enabled, 1:1/2binning enabled, 0:disabled */
	buf->binning_factor_x = data[0]&0x03;
	buf->binning_factor_x += 1;
	if(buf->binning_factor_x==3) buf->binning_factor_x += 1;	// bin_x : 3->4
	ret = t4k35_read_reg(client, 1, T4K35_V_BINNING_ENABLE, data);
	/*printk("[DEBUG] ret3l=%d\n", ret);*/
	printk("[DEBUG] bin_x=%d\n", buf->binning_factor_x);
	if (ret)
		return ret;
	/* 2:1/4binning enabled, 1:1/2binning enabled, 0:disabled */
	buf->binning_factor_y = data[0]&0x03;
	buf->binning_factor_y += 1;
	if(buf->binning_factor_y==3) buf->binning_factor_y += 1;	// bin_x : 3->4
#endif	// T4K35

	/*printk("[DEBUG] end of ret3\n");*/
	printk("[DEBUG] bin_y=%d\n", buf->binning_factor_y);
	return 0;
}

static int t4k35_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = t4k35_read_reg(client, T4K35_16BIT,
			       T4K35_COARSE_INTEGRATION_TIME, &coarse);
	*value = coarse;

	return ret;
}

#if 0
/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int t4k35_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

	/* the fine integration time is currently not calculated */
	ret = t4k35_read_reg(client, T4K35_16BIT,
			       T4K35_COARSE_INTEGRATION_TIME, &coarse);
	*value = coarse;

	return ret;
}
#endif

static int t4k35_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	return t4k35_write_reg(client, T4K35_16BIT, T4K35_TEST_PATTERN_MODE, value);
}

static enum v4l2_mbus_pixelcode
t4k35_translate_bayer_order(enum atomisp_bayer_order code)
{
	printk("[DEBUG] %s, start\n", __func__);

#if 0	// debug bayer 20140609
	switch (code) {
	case atomisp_bayer_order_rggb:
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SRGGB10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SRGGB10_1X10;
	case atomisp_bayer_order_grbg:
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SGRBG10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SGRBG10_1X10;
	case atomisp_bayer_order_bggr:
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SBGGR10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SBGGR10_1X10;
	case atomisp_bayer_order_gbrg:
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SGBRG10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SGBRG10_1X10;
	}
#else	// debug bayer 20140609
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SGRBG10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SGRBG10_1X10;
#endif	// debug bayer 20140609
	return 0;
}

static int t4k35_v_flip(struct v4l2_subdev *sd, s32 value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct camera_mipi_info *t4k35_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val;

	//printk("[DEBUG] %s, start\n", __func__);

	ret = t4k35_write_reg_array(client, t4k35_param_hold);
	if (ret)
		return ret;
	ret = t4k35_read_reg(client, T4K35_8BIT, T4K35_IMG_ORIENTATION, &val);
	if (ret)
		return ret;
	if (value)
		val |= T4K35_VFLIP_BIT;
	else
		val &= ~T4K35_VFLIP_BIT;
	ret = t4k35_write_reg(client, T4K35_8BIT,
			T4K35_IMG_ORIENTATION, val);
	if (ret)
		return ret;

	t4k35_info = v4l2_get_subdev_hostdata(sd);
	if (t4k35_info) {
		val &= (T4K35_VFLIP_BIT|T4K35_HFLIP_BIT);
		t4k35_info->raw_bayer_order = t4k35_bayer_order_mapping[val];
		dev->format.code = t4k35_translate_bayer_order(
			t4k35_info->raw_bayer_order);
	}

	return t4k35_write_reg_array(client, t4k35_param_update);
}

static int t4k35_h_flip(struct v4l2_subdev *sd, s32 value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct camera_mipi_info *t4k35_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val;

	//printk("[DEBUG] %s, start\n", __func__);

	ret = t4k35_write_reg_array(client, t4k35_param_hold);
	if (ret)
		return ret;
	ret = t4k35_read_reg(client, T4K35_8BIT, T4K35_IMG_ORIENTATION, &val);
	if (ret)
		return ret;
	if (value)
		val |= T4K35_HFLIP_BIT;
	else
		val &= ~T4K35_HFLIP_BIT;
	ret = t4k35_write_reg(client, T4K35_8BIT,
			T4K35_IMG_ORIENTATION, val);
	if (ret)
		return ret;

	t4k35_info = v4l2_get_subdev_hostdata(sd);
	if (t4k35_info) {
		val &= (T4K35_VFLIP_BIT|T4K35_HFLIP_BIT);
		t4k35_info->raw_bayer_order = t4k35_bayer_order_mapping[val];
		dev->format.code = t4k35_translate_bayer_order(
		t4k35_info->raw_bayer_order);
	}

	return t4k35_write_reg_array(client, t4k35_param_update);
}

static int t4k35_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	//printk("[DEBUG] %s, start\n", __func__);

	*val = (T4K35_FOCAL_LENGTH_NUM << 16) | T4K35_FOCAL_LENGTH_DEM;
	return 0;
}

static int t4k35_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	//printk("[DEBUG] %s, start\n", __func__);

	/*const f number for t4k35*/
	*val = (T4K35_F_NUMBER_DEFAULT_NUM << 16) | T4K35_F_NUMBER_DEM;
	return 0;
}

static int t4k35_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	//printk("[DEBUG] %s, start\n", __func__);

	*val = (T4K35_F_NUMBER_DEFAULT_NUM << 24) |
		(T4K35_F_NUMBER_DEM << 16) |
		(T4K35_F_NUMBER_DEFAULT_NUM << 8) | T4K35_F_NUMBER_DEM;
	return 0;
}

static int t4k35_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	*val = dev->curr_res_table[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int t4k35_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	*val = dev->curr_res_table[dev->fmt_idx].bin_factor_y;

	return 0;
}

int t4k35_vcm_power_up(struct v4l2_subdev *sd)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->power_up)
		return dev->vcm_driver->power_up(sd);
	return 0;
}

int t4k35_vcm_power_down(struct v4l2_subdev *sd)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->power_down)
		return dev->vcm_driver->power_down(sd);
	return 0;
}

int t4k35_vcm_init(struct v4l2_subdev *sd)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->init)
		return dev->vcm_driver->init(sd);
	return 0;
}

int t4k35_t_focus_vcm(struct v4l2_subdev *sd, u16 val)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->t_focus_vcm)
		return dev->vcm_driver->t_focus_vcm(sd, val);
	return 0;
}

int t4k35_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	/*printk("[DEBUG] %s, start\n", __func__);*/

	if (dev->vcm_driver && dev->vcm_driver->t_focus_abs)
		return dev->vcm_driver->t_focus_abs(sd, value);
	return 0;
}
int t4k35_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->t_focus_rel)
		return dev->vcm_driver->t_focus_rel(sd, value);
	return 0;
}

int t4k35_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->q_focus_status)
		return dev->vcm_driver->q_focus_status(sd, value);
	return 0;
}

int t4k35_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->q_focus_abs)
		return dev->vcm_driver->q_focus_abs(sd, value);
	return 0;
}

int t4k35_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->t_vcm_slew)
		return dev->vcm_driver->t_vcm_slew(sd, value);
	return 0;
}

int t4k35_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->t_vcm_timing)
		return dev->vcm_driver->t_vcm_timing(sd, value);
	return 0;
}

int t4k35_s_hilightmode(struct v4l2_subdev *sd, u16 value)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	printk("[DEBUG] %s value = %d, start\n", __func__, value);

	lowlight_status = value;

	return 0;
}

int t4k35_q_hilightmode(struct v4l2_subdev *sd, u16 value)
{
	return 0;
}

int t4k35_s_controlDPC(struct v4l2_subdev *sd, bool DPCstatus)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	printk("[DEBUG] %s DPCstatus = %d, start\n", __func__, DPCstatus);
	t4k35_write_reg_array(client, t4k35_param_hold);
	if (DPCstatus) {
		t4k35_write_reg(client, T4K35_8BIT, 0x0A0B, 0x00FF);
		t4k35_write_reg(client, T4K35_8BIT, 0x3286, 0x0001);
		t4k35_write_reg(client, T4K35_8BIT, 0x3290, 0x0030);
		t4k35_write_reg(client, T4K35_8BIT, 0x3294, 0x0011);
		t4k35_write_reg(client, T4K35_8BIT, 0x3295, 0x0020);
	} else {
		t4k35_write_reg(client, T4K35_8BIT, 0x0A0B, 0x0098);
		t4k35_write_reg(client, T4K35_8BIT, 0x3286, 0x0002);
		t4k35_write_reg(client, T4K35_8BIT, 0x3290, 0x0020);
		t4k35_write_reg(client, T4K35_8BIT, 0x3294, 0x0010);
		t4k35_write_reg(client, T4K35_8BIT, 0x3295, 0x0060);
	}
	t4k35_write_reg_array(client, t4k35_param_update);

	return 0;
}

int t4k35_q_controlDPC(struct v4l2_subdev *sd, u16 value)
{
	return 0;
}

#if 0
/*For LED FLASH Controll+++*/
#define	LED_PROC_FILE	"driver/camera_flash"
static struct proc_dir_entry *led_proc_file;
static u16 sky81296_read_value;

static int led_proc_read(struct seq_file *buf, void *v)
{
	return sprintf(buf, "Not support this command\n");
}

static int led_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, led_proc_read, NULL);
}

static int set_torch_enable(int enable)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(main_sd);

	if (enable) {
		printk("set torch on\n");
		gpio_set_value(flash_ent, 1);
		msleep(1);
		gpio_set_value(flash_enf, 1);
		msleep(1);
		gpio_set_value(flash_enf, 0);
		msleep(1);
		gpio_set_value(flash_ent, 0);
		msleep(1);
	} else {
		printk("set torch off\n");
		gpio_set_value(flash_enf, 0);
		msleep(1);
		gpio_set_value(flash_ent, 0);
		msleep(1);
	}
}

static int set_flash_enable(int enable)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(main_sd);
	
	if (enable) {
		printk("set flash on\n");
		gpio_set_value(flash_ent, 1);
		msleep(1);
		gpio_set_value(flash_enf, 1);
		msleep(1);
		gpio_set_value(flash_enf, 0);
		msleep(1);
		gpio_set_value(flash_ent, 0);
		msleep(1);
	} else {
		printk("set flash off\n");
		gpio_set_value(flash_enf, 0);
		msleep(1);
		gpio_set_value(flash_ent, 0);
		msleep(1);
	}
}

static ssize_t led_proc_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	char buffer[256];
	s32 value;
	int ret;

	if (len > 256)
	len = 256;

	pr_info("led_proc_write %s\n", buff);
	if (copy_from_user(buffer, buff, len)) {
		printk(KERN_INFO "%s: proc write to buffer failed.\n", __func__);
	return -EFAULT;
	}

	/*FLED_DRIVER_ENT 159*/
	if (flash_ent < 0) {
		ret = gpio_request(54, FLED_DRIVER_ENT);
		if (ret < 0) {
			printk("%s not available.\n", FLED_DRIVER_ENT);
			return ret;
		}
		gpio_direction_output(54, 0);
		flash_ent = 54;
	}

	/*FLED_DRIVER_ENF 161*/
	if (flash_enf < 0) {
		ret = gpio_request(53, FLED_DRIVER_ENF);
		if (ret < 0) {
			printk("%s not available.\n", FLED_DRIVER_ENF);
			return ret;
		}
		gpio_direction_output(53, 0);
		flash_enf = 53;
	}

	if (!strncmp("set_torch_on", buffer, 12)) {
		set_torch_enable(1);
	} else if (!strncmp("set_torch_off", buffer, 13)) {
		set_torch_enable(0);
	} else if (!strncmp("set_flash_on", buffer, 12)) {
		set_flash_enable(1);
	} else if (!strncmp("set_flash_off", buffer, 13)) {
		set_flash_enable(0);
	} else {
		pr_info("command not support\n");
	}

	return len;
}

static const struct file_operations led_fops = {
	.owner = THIS_MODULE,
	.open = led_proc_open,
	.read = seq_read,
	.write = led_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void create_led_proc_file(void)
{
	led_proc_file = proc_create(LED_PROC_FILE, 0666, NULL, &led_fops);
	if (led_proc_file) {
		printk("led proc file create sucessed!\n");
	} else{
		printk("led proc file create failed!\n");
	}
}
/*For LED FLASH Controll---*/
#endif

/*For CAMERA STATUS Controll+++*/
#define	STATUS_PROC_FILE	"driver/camera_status"
static struct proc_dir_entry *status_proc_file;

static int status_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", ATD_t4k35_status);
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
/*For CAMERA STATUS Controll---*/

/*For CAMERA OTP Controll+++*/
#define	OTP_PROC_FILE	"driver/camera_otp"
static struct proc_dir_entry *otp_proc_file;

static int otp_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", camera_module_otp);
    return 0;
}

static int otp_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, otp_proc_read, NULL);
}

static ssize_t otp_proc_write(struct file *filp, const char __user *buff,
		unsigned long len, void *data)
{
    pr_info("command not support\n");
    return len;
}

static const struct file_operations otp_fops = {
	.owner = THIS_MODULE,
	.open = otp_proc_open,
	.read = seq_read,
	.write = otp_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_otp_proc_file(void)
{
    otp_proc_file = proc_create(OTP_PROC_FILE, 0666, NULL, &otp_fops);
    if (otp_proc_file) {
	printk("%s sucessed!\n", __func__);
    } else {
	printk("%s failed!\n", __func__);
    }
}
/*For CAMERA OTP Controll---*/
#if 0
/* ASUS_BSP +++ For LED FLASH Controll */
static int t4k35_s_flash_timeout(struct v4l2_subdev *sd, u32 val)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_g_flash_timeout(struct v4l2_subdev *sd, s32 *val)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_s_flash_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_g_flash_intensity(struct v4l2_subdev *sd, s32 *val)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_s_torch_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_g_torch_intensity(struct v4l2_subdev *sd, s32 *val)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_s_indicator_intensity(struct v4l2_subdev *sd, u32 intensity)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_g_indicator_intensity(struct v4l2_subdev *sd, s32 *val)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_s_flash_strobe(struct v4l2_subdev *sd, u32 val)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_s_flash_mode(struct v4l2_subdev *sd, u32 new_mode)
{
       pr_info("%s mode(%d)\n", __func__, new_mode);
	switch (new_mode) {
	case ATOMISP_FLASH_MODE_OFF:
		set_flash_enable(0);
		break;
	case ATOMISP_FLASH_MODE_FLASH:
		set_flash_enable(1);
		break;
	case ATOMISP_FLASH_MODE_TORCH:
		set_torch_enable(1);
		break;
	case ATOMISP_FLASH_MODE_INDICATOR:
	default:
		pr_err("%s not support mode(%d)\n", __func__, new_mode);
		return -EINVAL;
	}

	return 0;
}

static int t4k35_g_flash_mode(struct v4l2_subdev *sd, s32 *val)
{
	pr_info("%s \n", __func__);

	return 0;
}

static int t4k35_g_flash_status(struct v4l2_subdev *sd, s32 *val)
{
	pr_info("%s \n", __func__);

	return 0;
}
/* ASUS_BSP --- For LED FLASH Controll */
#endif

struct t4k35_control t4k35_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = t4k35_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Test pattern",
			.minimum = 0,
			.maximum = 0xffff,
			.step = 1,
			.default_value = 0,
		},
		.tweak = t4k35_test_pattern,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = t4k35_v_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Mirror",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = t4k35_h_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.minimum = 0,
			.maximum = T4K35_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = t4k35_t_focus_abs,
		.query = t4k35_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = T4K35_MAX_FOCUS_NEG,
			.maximum = T4K35_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = t4k35_t_focus_rel,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus status",
			.minimum = 0,
			.maximum = 100, /* allow enum to grow in the future */
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = t4k35_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_SLEW,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm slew",
			.minimum = 0,
			.maximum = T4K35_VCM_SLEW_STEP_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = t4k35_t_vcm_slew,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_TIMEING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm step time",
			.minimum = 0,
			.maximum = T4K35_VCM_SLEW_TIME_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = t4k35_t_vcm_timing,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = T4K35_FOCAL_LENGTH_DEFAULT,
			.maximum = T4K35_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = T4K35_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = t4k35_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = T4K35_F_NUMBER_DEFAULT,
			.maximum = T4K35_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = T4K35_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = t4k35_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = T4K35_F_NUMBER_RANGE,
			.maximum =  T4K35_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = T4K35_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = t4k35_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = T4K35_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = t4k35_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = T4K35_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = t4k35_g_bin_factor_y,
	},
	/*ASUS_BSP+++ PJ "add lowlight mode control binning weight interface"*/
	{
		.qc = {
			.id = V4L2_CID_HILIGHT_MODE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "hilight mode",
			.minimum = 0x0,
			.maximum = 0xf,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = t4k35_s_hilightmode,
		.query = t4k35_q_hilightmode,
	},
	/*ASUS_BSP--- PJ "add lowlight mode control binning weight interface"*/
	/*ASUS_BSP+++ PJ "add control DPC interface"*/
	{
		.qc = {
			.id = V4L2_CID_SET_DPC,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "control DPC",
			.minimum = 0x0,
			.maximum = 0xf,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = t4k35_s_controlDPC,
		.query = t4k35_q_controlDPC,
	},
	/*ASUS_BSP--- PJ "add control DPC interface"*/
#if 0
/* ASUS_BSP +++ For LED FLASH Controll */
	{
		.qc = {
			.id = V4L2_CID_FLASH_TIMEOUT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Flash Timeout",
			.minimum = 0,
			.maximum = 1024U,
			.step = 1,
			.default_value = 512U,
			.flags = 0,
		},
		.tweak = t4k35_s_flash_timeout,
		.query = t4k35_g_flash_timeout,
	},
	{
		.qc = {
			.id = V4L2_CID_FLASH_INTENSITY,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Flash Intensity",
			.minimum = 0,
			.maximum = 100U,
			.step = 1,
			.default_value = 80U,
			.flags = 0,
		},
		.tweak = t4k35_s_flash_intensity,
		.query = t4k35_g_flash_intensity,
	},
	{
		.qc = {
			.id = V4L2_CID_FLASH_TORCH_INTENSITY,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Torch Intensity",
			.minimum = 0,
			.maximum = 100U,
			.step = 1,
			.default_value = 80U,
			.flags = 0,
		},
		.tweak = t4k35_s_torch_intensity,
		.query = t4k35_g_torch_intensity,
	},
	{
		.qc = {
			.id = V4L2_CID_FLASH_INDICATOR_INTENSITY,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Indicator Intensity",
			.minimum = 0,
			.maximum = 100U,
			.step = 1,
			.default_value = 80U,
			.flags = 0,
		},
		.tweak = t4k35_s_indicator_intensity,
		.query = t4k35_g_indicator_intensity,
	},
	{
		.qc = {
			.id = V4L2_CID_FLASH_STROBE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Flash Strobe",
			.minimum = 0,
			.maximum = 0,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = t4k35_s_flash_strobe,
	},
	{
		.qc = {
			.id = V4L2_CID_FLASH_MODE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Flash Mode",
			.minimum = 0,
			.maximum = 100,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = t4k35_s_flash_mode,
		.query = t4k35_g_flash_mode,
	},
	{
		.qc = {
			.id = V4L2_CID_FLASH_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Flash Status",
			.minimum = 0,
			.maximum = 100,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = t4k35_g_flash_status,
	},
/* ASUS_BSP --- For LED FLASH Controll */
#endif
};
#define N_CONTROLS (ARRAY_SIZE(t4k35_controls))

static struct t4k35_control *t4k35_find_control(u32 id)
{
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < N_CONTROLS; i++)
		if (t4k35_controls[i].qc.id == id)
			return &t4k35_controls[i];
	return NULL;
}

static int t4k35_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct t4k35_control *ctrl = t4k35_find_control(qc->id);
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* t4k35 control set/get */
static int t4k35_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct t4k35_control *s_ctrl;
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

	if (!ctrl)
		return -EINVAL;

	s_ctrl = t4k35_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int t4k35_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct t4k35_control *octrl = t4k35_find_control(ctrl->id);
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 600
static int distance(struct t4k35_resolution const *res, u32 w, u32 h)
{
	unsigned int w_ratio;
	unsigned int h_ratio;
	int match;

	//printk("[DEBUG] %s, start\n", __func__);

	if (w == 0)
		return -1;
	w_ratio = (res->width << 13) / w;
	if (h == 0)
		return -1;
	h_ratio = (res->height << 13) / h;
	if (h_ratio == 0)
		return -1;
	match   = abs(((w_ratio << 13) / h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)  ||
		(match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(struct v4l2_subdev *sd, int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int fps_diff;
	int min_fps_diff = INT_MAX;
	int min_dist = INT_MAX;
	const struct t4k35_resolution *tmp_res = NULL;
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < dev->entries_curr_table; i++) {
		tmp_res = &dev->curr_res_table[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
		if (dist == min_dist) {
			fps_diff = __t4k35_min_fps_diff(dev->fps,
						tmp_res->fps_options);
			if (fps_diff < min_fps_diff) {
				min_fps_diff = fps_diff;
				idx = i;
			}
		}
	}

	return idx;
}

static int t4k35_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	int idx = 0;

	printk("[DEBUG] %s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	printk("[DEBUG] now width=%d, height=%d\n", fmt->width, fmt->height);

	if ((fmt->width > t4k35_max_res[dev->sensor_id].res_max_width)
		|| (fmt->height > t4k35_max_res[dev->sensor_id].res_max_height)) {
		fmt->width =  t4k35_max_res[dev->sensor_id].res_max_width;
		fmt->height = t4k35_max_res[dev->sensor_id].res_max_height;
	} else {
		idx = nearest_resolution_index(sd, fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller
		 *  resolutions. If it fails, it means the requested
		 *  resolution is higher than wecan support. Fallback
		 *  to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = dev->entries_curr_table - 1;

		fmt->width = dev->curr_res_table[idx].width;
		fmt->height = dev->curr_res_table[idx].height;
	}

	fmt->code = dev->format.code;

	mutex_unlock(&dev->input_lock);
	return 0;
}

/* Call with ctrl_handler.lock hold */
static int __adjust_hvblank(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	u16 new_frame_length_lines, new_line_length_pck;
	int ret;

	/*printk("[DEBUG] %s, start\n", __func__);*/

	/*
	 * No need to adjust h/v blank if not set dbg value
	 * Note that there is no other checking on the h/v blank value,
	 * as h/v blank can be set to any value above zero for debug purpose
	 */
	if (!dev->v_blank->val || !dev->h_blank->val)
		return 0;

	new_frame_length_lines = dev->curr_res_table[dev->fmt_idx].height +
		dev->v_blank->val;
	new_line_length_pck = dev->curr_res_table[dev->fmt_idx].width +
		dev->h_blank->val;

	ret = t4k35_write_reg(client, T4K35_16BIT, T4K35_LINE_LENGTH_PIXELS,
			    new_line_length_pck);
	if (ret)
		return ret;
	ret = t4k35_write_reg(client, T4K35_16BIT, T4K35_FRAME_LENGTH_LINES,
			    new_frame_length_lines);
	if (ret)
		return ret;

	dev->lines_per_frame = new_frame_length_lines;
	dev->pixels_per_line = new_line_length_pck;

	return 0;
}

static int t4k35_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct camera_mipi_info *t4k35_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct t4k35_resolution *res;
	int ret;
	u16 val;

	printk("[DEBUG] %s, start\n", __func__);

	t4k35_info = v4l2_get_subdev_hostdata(sd);
	if (t4k35_info == NULL)
		return -EINVAL;
	ret = t4k35_try_mbus_fmt(sd, fmt);
	if (ret)
		return ret;

	mutex_lock(&dev->input_lock);

	printk("[DEBUG] width=%d, height=%d\n", fmt->width,fmt->height);
	dev->fmt_idx = nearest_resolution_index(sd, fmt->width, fmt->height);
	if (dev->fmt_idx == -1) {
		ret = -EINVAL;
		goto out;
	}
	res = &dev->curr_res_table[dev->fmt_idx];

	/* Adjust the FPS selection based on the resolution selected */
	dev->fps_index = __t4k35_nearest_fps_index(dev->fps, res->fps_options);
	dev->fps = res->fps_options[dev->fps_index].fps;
	dev->regs = res->fps_options[dev->fps_index].regs;
	if (!dev->regs)
		dev->regs = res->regs;

	ret = t4k35_write_reg_array(client, dev->regs);
	if (ret)
		goto out;

	dev->pixels_per_line = res->fps_options[dev->fps_index].pixels_per_line;
	dev->lines_per_frame = res->fps_options[dev->fps_index].lines_per_frame;
	printk("[DEBUG] desc=%s\n", res->desc);

	/* dbg h/v blank time */
	mutex_lock(dev->ctrl_handler.lock);
	__adjust_hvblank(sd);
	mutex_unlock(dev->ctrl_handler.lock);

	ret = __t4k35_update_exposure_timing(client, dev->coarse_itg,
		dev->pixels_per_line, dev->lines_per_frame);
	//printk("[DEBUG] ret1=%d\n", ret);
	if (ret)
		goto out;

	ret = t4k35_write_reg_array(client, t4k35_param_update);
	//printk("[DEBUG] ret2=%d\n", ret);
	if (ret)
		goto out;

	ret = t4k35_get_intg_factor(client, t4k35_info, dev->regs);
	/*printk("[DEBUG] ret3=%d\n", ret);*/
	if (ret)
		goto out;

	ret = t4k35_read_reg(client, T4K35_8BIT, T4K35_IMG_ORIENTATION, &val);
	/*printk("[DEBUG] ret4=%d\n", ret);*/
	if (ret)
		goto out;
	val &= (T4K35_VFLIP_BIT|T4K35_HFLIP_BIT);
	t4k35_info->raw_bayer_order = t4k35_bayer_order_mapping[val];
	/*printk("[DEBUG] bayer_order=%d\n", t4k35_info->raw_bayer_order);*/
	dev->format.code = t4k35_translate_bayer_order(
		t4k35_info->raw_bayer_order);
out:
	/*printk("[DEBUG] ret5=%d\n", ret);*/
	mutex_unlock(&dev->input_lock);
	return ret;
}


static int t4k35_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	printk("[DEBUG] %s, start\n", __func__);

	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fmt->width = dev->curr_res_table[dev->fmt_idx].width;
	fmt->height = dev->curr_res_table[dev->fmt_idx].height;
	fmt->code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int t4k35_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;

	ATD_t4k35_status = 0;
	printk("[DEBUG] %s, start\n", __func__);

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
#if 0	// T4K35
	if (imx_read_reg(client, IMX_16BIT, IMX132_175_CHIP_ID, id)) {
		v4l2_err(client, "sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}
	if (*id == IMX132_ID || *id == IMX175_ID)
		goto found;

	if (imx_read_reg(client, IMX_16BIT, IMX134_135_CHIP_ID, id)) {
		v4l2_err(client, "sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}
	if (*id != IMX134_ID && *id != IMX135_ID) {
		v4l2_err(client, "no imx sensor found\n");
		return -ENODEV;
	}
#else	// T4K35
	if (t4k35_read_reg(client, T4K35_16BIT, T4K35_CHIP_ID, id)) {
		v4l2_err(client, "sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}
	if (*id != T4K35_ID) {
		v4l2_err(client, "no toshiba sensor found\n");
		return -ENODEV;
	}
#endif	// T4K35
//found:	// T4K35
	v4l2_info(client, "sensor_id = 0x%x\n", *id);

	/* TODO - need to be updated */
	*revision = 0;

	ATD_t4k35_status = 1;
	return 0;
}

static void __t4k35_print_timing(struct v4l2_subdev *sd)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 width = dev->curr_res_table[dev->fmt_idx].width;
	u16 height = dev->curr_res_table[dev->fmt_idx].height;

	printk("[DEBUG] %s, start\n", __func__);
#if 0
	dev_dbg(&client->dev, "Dump t4k35 timing in stream on:\n");
	dev_dbg(&client->dev, "width: %d:\n", width);
	dev_dbg(&client->dev, "height: %d:\n", height);
	dev_dbg(&client->dev, "pixels_per_line: %d:\n", dev->pixels_per_line);
	dev_dbg(&client->dev, "line per frame: %d:\n", dev->lines_per_frame);
	dev_dbg(&client->dev, "pix freq: %d:\n", dev->vt_pix_clk_freq_mhz);
	dev_dbg(&client->dev, "init fps: %d:\n", dev->vt_pix_clk_freq_mhz /
			dev->pixels_per_line / dev->lines_per_frame);
	dev_dbg(&client->dev, "HBlank: %d nS:\n",
			1000 * (dev->pixels_per_line - width) /
			(dev->vt_pix_clk_freq_mhz / 1000000));
	dev_dbg(&client->dev, "VBlank: %d uS:\n",
			(dev->lines_per_frame - height) * dev->pixels_per_line /
			(dev->vt_pix_clk_freq_mhz / 1000000));
#endif
	pr_info("Dump t4k35 timing in stream on:\n");
	pr_info("width: %d:\n", width);
	pr_info("height: %d:\n", height);
	pr_info("pixels_per_line: %d:\n", dev->pixels_per_line);
	pr_info("line per frame: %d:\n", dev->lines_per_frame);
	pr_info("pix freq: %d:\n", dev->vt_pix_clk_freq_mhz);
	pr_info("init fps: %d:\n", dev->vt_pix_clk_freq_mhz /
			dev->pixels_per_line / dev->lines_per_frame);
	pr_info("HBlank: %d nS:\n",
			1000 * (dev->pixels_per_line - width) /
			(dev->vt_pix_clk_freq_mhz / 1000000));
	pr_info("VBlank: %d uS:\n",
			(dev->lines_per_frame - height) * dev->pixels_per_line /
			(dev->vt_pix_clk_freq_mhz / 1000000));
}

/*
 * t4k35 stream on/off
 */
static int t4k35_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	printk("[DEBUG] %s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	if (enable) {
#if 0
		/* Noise reduction & dead pixel applied before streaming */
		if (dev->fw == NULL) {
			dev_warn(&client->dev, "No MSR loaded from library");
		} else {
			ret = apply_msr_data(client, dev->fw);
			if (ret) {
				mutex_unlock(&dev->input_lock);
				return ret;
			}
		}
#endif
		/*__t4k35_print_timing(sd);*/
		if (lowlight_status) {
			ret = t4k35_write_reg(client, T4K35_8BIT, T4K35_BINNING_TYPE,
					    1);
			if (ret)
				return ret;
		}
		ret = t4k35_write_reg_array(client, t4k35_streaming);
		if (ret != 0) {
			v4l2_err(client, "write_reg_array err\n");
			mutex_unlock(&dev->input_lock);
			return ret;
		}
		printk("[DEBUG] %s, stream on\n", __func__);
		dev->streaming = 1;
	} else {
		ret = t4k35_write_reg_array(client, t4k35_soft_standby);
		if (ret != 0) {
			v4l2_err(client, "write_reg_array err\n");
			mutex_unlock(&dev->input_lock);
			return ret;
		}
		printk("[DEBUG] %s, stream off\n", __func__);
		dev->streaming = 0;
		dev->fps_index = 0;
		dev->fps = 0;
		msleep(10);
	}
	mutex_unlock(&dev->input_lock);
	printk("[DEBUG] %s, end\n", __func__);

	return 0;
}

/*
 * t4k35 enum frame size, frame intervals
 */
static int t4k35_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	/*printk("[DEBUG] %s, start\n", __func__);*/
	/*printk("[DEBUG] index in t4k35_enum_framesizes is %d\n", index);*/

	mutex_lock(&dev->input_lock);
	if (index >= dev->entries_curr_table) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->curr_res_table[index].width;
	fsize->discrete.height = dev->curr_res_table[index].height;
	fsize->reserved[0] = dev->curr_res_table[index].used;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int t4k35_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	/*printk("[DEBUG] %s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	/* since the isp will donwscale the resolution to the right size,
	  * find the nearest one that will allow the isp to do so
	  * important to ensure that the resolution requested is padded
	  * correctly by the requester, which is the atomisp driver in
	  * this case.
	  */
	i = nearest_resolution_index(sd, fival->width, fival->height);

	if (i == -1)
		goto out;

	/* Check if this index is supported */
	if (index > __t4k35_get_max_fps_index(dev->curr_res_table[i].fps_options))
		goto out;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = dev->curr_res_table[i].width;
	fival->height = dev->curr_res_table[i].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = dev->curr_res_table[i].fps_options[index].fps;
	mutex_unlock(&dev->input_lock);
	return 0;
out:
	mutex_unlock(&dev->input_lock);
	return -EINVAL;
}

static int t4k35_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	printk("[DEBUG] %s, start\n", __func__);

	if (index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}


static int __update_t4k35_device_settings(struct t4k35_device *dev, u16 sensor_id)
{
	printk("[DEBUG] %s, start\n", __func__);

	switch (sensor_id) {
#if 0	// T4K35
	case IMX175_ID:
		if (INTEL_MID_BOARD(1, PHONE, BYT) ||
			INTEL_MID_BOARD(1, TABLET, BYT)) {
			dev->mode_tables = &imx_sets[IMX175_VALLEYVIEW];
			dev->vcm_driver = &imx_vcms[IMX175_VALLEYVIEW];
			dev->otp_driver = &imx_otps[IMX175_VALLEYVIEW];
		} else {
			dev->mode_tables = &imx_sets[IMX175_MERRFLD];
			dev->vcm_driver = &imx_vcms[IMX175_MERRFLD];
			dev->otp_driver = &imx_otps[IMX175_MERRFLD];
		}
		break;
	case IMX135_ID:
		if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW ||
		    dev->i2c_id == IMX135_FUJI_ID) {
			dev->mode_tables = &imx_sets[IMX135_VICTORIABAY];
			dev->vcm_driver = &imx_vcms[IMX135_VICTORIABAY];
			dev->otp_driver = &imx_otps[IMX135_VICTORIABAY];
		} else {
			dev->mode_tables = &imx_sets[IMX135_SALTBAY];
			dev->vcm_driver = &imx_vcms[IMX135_SALTBAY];
			dev->otp_driver = &imx_otps[IMX135_SALTBAY];
		}
		break;
	case IMX134_ID:
		dev->mode_tables = &imx_sets[IMX134_VALLEYVIEW];
		dev->vcm_driver = &imx_vcms[IMX134_VALLEYVIEW];
		dev->otp_driver = &imx_otps[IMX134_VALLEYVIEW];
		break;
	case IMX132_ID:
		dev->mode_tables = &imx_sets[IMX132_SALTBAY];
		dev->otp_driver = &imx_otps[IMX132_SALTBAY];
		dev->vcm_driver = NULL;
		return 0;
#else	// T4K35
	case T4K35_ID:
		dev->mode_tables = &t4k35_sets[T4K35_MERRFLD];
		dev->vcm_driver = &t4k35_vcms[T4K35_MERRFLD];
		break;
#endif	// T4K35
	default:
		return -EINVAL;
	}

	return dev->vcm_driver->init(&dev->sd);
	/*dev->vcm_driver->init(&dev->sd);
	t4k35_t_focus_abs(&dev->sd, 1023);
	msleep(100);
	t4k35_t_focus_abs(&dev->sd, 0);
	return 0;*/
}

static int t4k35_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;
	void *otp_data;
	printk("[DEBUG] %s, start\n", __func__);

	if (pdata == NULL)
		return -ENODEV;

	dev->platform_data = pdata;

	mutex_lock(&dev->input_lock);

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			dev_err(&client->dev, "t4k35 platform init err\n");
			return ret;
		}
	}

	ret = __t4k35_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "t4k35 power-up err.\n");
		mutex_unlock(&dev->input_lock);
		return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = t4k35_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "t4k35_detect err s_config.\n");
		goto fail_detect;
	}

	dev->sensor_id = sensor_id;
	printk("[DEBUG] sensor_id=%x\n", dev->sensor_id);
	dev->sensor_revision = sensor_revision;

	/* Resolution settings depend on sensor type and platform */
	ret = __update_t4k35_device_settings(dev, dev->sensor_id);
	if (ret)
		goto fail_detect;

	/* Read sensor's OTP data */
	otp_data = t4k35_otp_read(sd);
	if (!IS_ERR(otp_data))
		dev->otp_data = otp_data;
	/* power off sensor */
	ret = __t4k35_s_power(sd, 0);

	mutex_unlock(&dev->input_lock);
	if (ret)
		v4l2_err(client, "t4k35 power-down err.\n");

	return ret;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	__t4k35_s_power(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
t4k35_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	/*printk("[DEBUG] %s, start\n", __func__);*/

	if (code->index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	code->code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int
t4k35_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	/*printk("[DEBUG] %s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	if (index >= dev->entries_curr_table) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fse->min_width = dev->curr_res_table[index].width;
	fse->min_height = dev->curr_res_table[index].height;
	fse->max_width = dev->curr_res_table[index].width;
	fse->max_height = dev->curr_res_table[index].height;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static struct v4l2_mbus_framefmt *
__t4k35_get_pad_format(struct t4k35_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	/*printk("[DEBUG] %s, start\n", __func__);*/

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
t4k35_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__t4k35_get_pad_format(dev, fh, fmt->pad, fmt->which);

	/*printk("[DEBUG] %s, start\n", __func__);*/

	fmt->format = *format;

	return 0;
}

static int
t4k35_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	/*printk("[DEBUG] %s, start\n", __func__);*/

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}

static int
t4k35_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;

	printk("[DEBUG] %s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		dev->curr_res_table = dev->mode_tables->res_video;
		dev->entries_curr_table = dev->mode_tables->n_res_video;
		break;
	case CI_MODE_STILL_CAPTURE:
		dev->curr_res_table = dev->mode_tables->res_still;
		dev->entries_curr_table = dev->mode_tables->n_res_still;
		break;
	default:
		dev->curr_res_table = dev->mode_tables->res_preview;
		dev->entries_curr_table = dev->mode_tables->n_res_preview;
	}
	mutex_unlock(&dev->input_lock);
	return 0;
}

int
t4k35_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	const struct t4k35_resolution *res =
				&dev->curr_res_table[dev->fmt_idx];

	/*printk("[DEBUG] %s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	interval->interval.denominator = res->fps_options[dev->fps_index].fps;
	interval->interval.numerator = 1;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int __t4k35_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct t4k35_resolution *res =
				&dev->curr_res_table[dev->fmt_idx];
	struct camera_mipi_info *t4k35_info = NULL;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	unsigned int fps_index;
	int fps;
	int ret = 0;


	/*printk("[DEBUG] %s, start\n", __func__);*/

	t4k35_info = v4l2_get_subdev_hostdata(sd);
	if (t4k35_info == NULL)
		return -EINVAL;

	if (!interval->interval.numerator)
		interval->interval.numerator = 1;

	fps = interval->interval.denominator / interval->interval.numerator;

	if (!fps)
		/* currently does not support fps format like 1/2, 1/3 */
		fps = 1;

	/* No need to proceed further if we are not streaming */
	if (!dev->streaming) {
		/* Save the new FPS and use it while selecting setting */
		dev->fps = fps;
		return 0;
	}

	 /* Ignore if we are already using the required FPS. */
	if (fps == dev->fps)
		return 0;

	/*
	 * Start here, sensor is already streaming, so adjust fps dynamically
	 */
	fps_index = __t4k35_above_nearest_fps_index(fps, res->fps_options);

	if (res->fps_options[fps_index].regs &&
	    res->fps_options[fps_index].regs != dev->regs) {
		/*
		 * if need a new setting, but the new setting has difference
		 * with current setting, not use this one, as may have
		 * unexpected result, e.g. PLL, IQ.
		 */
		dev_dbg(&client->dev, "Sensor is streaming, not apply new sensor setting\n");
		if (fps > res->fps_options[dev->fps_index].fps) {
			/*
			 * Does not support increase fps based on low fps
			 * setting, as the high fps setting could not be used,
			 * and fps requested is above current setting fps.
			 */
			dev_warn(&client->dev, "Could not support fps: %d, keep current: %d.\n",
					fps, dev->fps);
			goto done;
		}
	} else {
		dev->fps_index = fps_index;
		dev->fps = res->fps_options[dev->fps_index].fps;
	}

	/* Update the new frametimings based on FPS */
	pixels_per_line = res->fps_options[dev->fps_index].pixels_per_line;
	lines_per_frame = res->fps_options[dev->fps_index].lines_per_frame;

	if (fps > res->fps_options[fps_index].fps) {
		/*
		 * if does not have high fps setting, not support increase fps
		 * by adjust lines per frame.
		 */
		dev_warn(&client->dev, "Could not support fps: %d. Use:%d.\n",
				fps, res->fps_options[fps_index].fps);
		goto update;
	}

	/* if the new setting does not match exactly */
	if (dev->fps != fps) {
#define MAX_LINES_PER_FRAME	0xffff
		dev_dbg(&client->dev, "adjusting fps using lines_per_frame\n");
		/*
		 * FIXME!
		 * 1: check DS on max value of lines_per_frame
		 * 2: consider use pixel per line for more range?
		 */
		if (dev->lines_per_frame * dev->fps / fps >
				MAX_LINES_PER_FRAME) {
			dev_warn(&client->dev,
					"adjust lines_per_frame out of range, try to use max value.\n");
			lines_per_frame = MAX_LINES_PER_FRAME;
		} else {
			lines_per_frame = lines_per_frame * dev->fps / fps;
		}
	}
update:
	/* Update the new frametimings based on FPS */
	dev->pixels_per_line = pixels_per_line;
	dev->lines_per_frame = lines_per_frame;

	/* Update the new values so that user side knows the current settings */
	ret = __t4k35_update_exposure_timing(client,
		dev->coarse_itg, dev->pixels_per_line, dev->lines_per_frame);
	if (ret)
		return ret;

	dev->fps = fps;

	ret = t4k35_get_intg_factor(client, t4k35_info, dev->regs);
	if (ret)
		return ret;
done:
	interval->interval.denominator = res->fps_options[dev->fps_index].fps;
	interval->interval.numerator = 1;
	__t4k35_print_timing(sd);

	return ret;
}

static int t4k35_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);
	int ret;

	/*printk("[DEBUG] %s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	ret = __t4k35_s_frame_interval(sd, interval);
	mutex_unlock(&dev->input_lock);

	return ret;
}
static int t4k35_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	printk("[DEBUG] %s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	*frames = dev->curr_res_table[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);
	printk("[DEBUG] skip_frames=%d\n", dev->curr_res_table[dev->fmt_idx].skip_frames);

	return 0;
}

static const struct v4l2_subdev_sensor_ops t4k35_sensor_ops = {
	.g_skip_frames	= t4k35_g_skip_frames,
};

static int t4k35_set_ctrl(struct v4l2_ctrl *ctrl)
{
	printk("[DEBUG] %s, start\n", __func__);

	return 0;
}

static int t4k35_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct t4k35_device *dev = container_of(ctrl->handler, struct t4k35_device,
			ctrl_handler);

	/*printk("[DEBUG] %s, start\n", __func__);*/

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ctrl->val = dev->lines_per_frame -
			dev->curr_res_table[dev->fmt_idx].height;
		break;
	case V4L2_CID_HBLANK:
		ctrl->val = dev->pixels_per_line -
			dev->curr_res_table[dev->fmt_idx].width;
		break;
	case V4L2_CID_PIXEL_RATE:
		ctrl->val = dev->vt_pix_clk_freq_mhz;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct v4l2_ctrl_ops t4k35_ctrl_ops = {
	.s_ctrl = t4k35_set_ctrl,
	.g_volatile_ctrl = t4k35_g_volatile_ctrl,
};

static const struct v4l2_subdev_video_ops t4k35_video_ops = {
	.s_stream = t4k35_s_stream,
	.enum_framesizes = t4k35_enum_framesizes,
	.enum_frameintervals = t4k35_enum_frameintervals,
	.enum_mbus_fmt = t4k35_enum_mbus_fmt,
	.try_mbus_fmt = t4k35_try_mbus_fmt,
	.g_mbus_fmt = t4k35_g_mbus_fmt,
	.s_mbus_fmt = t4k35_s_mbus_fmt,
	.s_parm = t4k35_s_parm,
	.g_frame_interval = t4k35_g_frame_interval,
	.s_frame_interval = t4k35_s_frame_interval,
};

static const struct v4l2_subdev_core_ops t4k35_core_ops = {
	.g_chip_ident = t4k35_g_chip_ident,
	.queryctrl = t4k35_queryctrl,
	.g_ctrl = t4k35_g_ctrl,
	.s_ctrl = t4k35_s_ctrl,
	.s_power = t4k35_s_power,
	.ioctl = t4k35_ioctl,
	.init = t4k35_init,
};

static const struct v4l2_subdev_pad_ops t4k35_pad_ops = {
	.enum_mbus_code = t4k35_enum_mbus_code,
	.enum_frame_size = t4k35_enum_frame_size,
	.get_fmt = t4k35_get_pad_format,
	.set_fmt = t4k35_set_pad_format,
};

static const struct v4l2_subdev_ops t4k35_ops = {
	.core = &t4k35_core_ops,
	.video = &t4k35_video_ops,
	.pad = &t4k35_pad_ops,
	.sensor = &t4k35_sensor_ops,
};

static const struct media_entity_operations t4k35_entity_ops = {
	.link_setup = NULL,
};

static int t4k35_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct t4k35_device *dev = to_t4k35_sensor(sd);

	printk("[DEBUG] %s, start\n", __func__);

	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();

	media_entity_cleanup(&dev->sd.entity);
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	/*release_msr_list(client, dev->fw);*/
	kfree(dev);

	return 0;
}

static int __t4k35_init_ctrl_handler(struct t4k35_device *dev)
{
	struct v4l2_ctrl_handler *hdl;

	/*printk("[DEBUG] %s, start\n", __func__);*/

	hdl = &dev->ctrl_handler;

	v4l2_ctrl_handler_init(&dev->ctrl_handler, 3);

	dev->pixel_rate = v4l2_ctrl_new_std(&dev->ctrl_handler,
					    &t4k35_ctrl_ops,
					    V4L2_CID_PIXEL_RATE,
					    0, UINT_MAX, 1, 0);

	dev->h_blank = v4l2_ctrl_new_std(&dev->ctrl_handler,
					  &t4k35_ctrl_ops,
					  V4L2_CID_HBLANK, 0, SHRT_MAX, 1, 0);

	dev->v_blank = v4l2_ctrl_new_std(&dev->ctrl_handler,
					  &t4k35_ctrl_ops,
					  V4L2_CID_VBLANK, 0, SHRT_MAX, 1, 0);

	if (dev->ctrl_handler.error || dev->pixel_rate == NULL
		|| dev->h_blank == NULL || dev->v_blank == NULL) {
		return dev->ctrl_handler.error;
	}

	dev->sd.ctrl_handler = hdl;

	dev->pixel_rate->flags |= V4L2_CTRL_FLAG_VOLATILE;
	dev->h_blank->flags |= V4L2_CTRL_FLAG_VOLATILE;
	dev->v_blank->flags |= V4L2_CTRL_FLAG_VOLATILE;

	return 0;
}

/*For CAMERA i2c Interface+++*/
static u16 i2c_read_value;
#define	I2C_CONTROL_PROC_FILE	"driver/i2c_control"
static struct proc_dir_entry *i2c_control_proc_file;

static int i2c_control_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%x\n", i2c_read_value);
	return 0;
}

static int
i2c_control_proc_open(struct inode *inode, struct  file *file) {
    return single_open(file, i2c_control_proc_read, NULL);
}

static ssize_t i2c_control_proc_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	struct i2c_client *client = v4l2_get_subdevdata(main_sd);
	int ret;
       char buffer[256];
       u16	arg[2];
	int index, tmp;

	arg[0] = 0;

	if (len > 256)
	len = 256;

	if (copy_from_user(buffer, buff, len)) {
		printk(KERN_INFO "%s:i2c proc write to buffer failed.\n", __func__);
		return -EFAULT;
       }

	buffer[len] = 0; /*add string end*/

	sscanf(buffer, "%x %x", &arg[0], &arg[1]);
	if (arg[0] == NULL) {
		printk(KERN_INFO "%s: input format error.\n", __func__);
		return -EFAULT;
	}
#if 0
	if (arg[0] == 0x1632) {
		for (index = 0; t4k35_Bin_1632x1228_30fps_2lane[index].type != T4K35_TOK_TERM; index++) {
			ret = t4k35_read_reg(client, T4K35_8BIT,
				       t4k35_Bin_1632x1228_30fps_2lane[index].sreg, &i2c_read_value);
			if (ret) {
				v4l2_err(client, "%s: failed to read 0x%x reg\n", __func__, arg[0]);
				return ret;
			}
			pr_info("PJ 1632 %x %x\n", t4k35_Bin_1632x1228_30fps_2lane[index].sreg, i2c_read_value);
		}
		return len;
	}
	if (arg[0] == 0x192) {
		for (index = 0; t4k35_Bin_192x160_30fps_2lane[index].type != T4K35_TOK_TERM; index++) {
			ret = t4k35_read_reg(client, T4K35_8BIT,
				       t4k35_Bin_192x160_30fps_2lane[index].sreg, &i2c_read_value);
			if (ret) {
				v4l2_err(client, "%s: failed to read 0x%x reg\n", __func__, arg[0]);
				return ret;
			}
			pr_info("PJ 192 %x %x\n", t4k35_Bin_192x160_30fps_2lane[index].sreg, i2c_read_value);
		}
		return len;
	}
	if (arg[0] == 0x656) {
		for (index = 0; t4k35_Bin_656x496_30fps_2lane[index].type != T4K35_TOK_TERM; index++) {
			ret = t4k35_read_reg(client, T4K35_8BIT,
				       t4k35_Bin_656x496_30fps_2lane[index].sreg, &i2c_read_value);
			if (ret) {
				v4l2_err(client, "%s: failed to read 0x%x reg\n", __func__, arg[0]);
				return ret;
			}
			pr_info("PJ 656 %x %x\n", t4k35_Bin_656x496_30fps_2lane[index].sreg, i2c_read_value);
		}
		return len;
	}

#endif
	if (arg[1] == NULL) {
		ret = t4k35_read_reg(client, T4K35_8BIT, arg[0], &i2c_read_value);
		if (ret) {
			v4l2_err(client, "%s: failed to read 0x%x reg\n", __func__, arg[0]);
			return ret;
		}
	return len;
	}
	printk(KERN_INFO "%s:argument is arg1=0x%x arg2=0x%x\n", __func__, arg[0], arg[1]);
	ret = t4k35_write_reg(client, T4K35_16BIT, arg[0], arg[1]);
	if (ret) {
		v4l2_err(client, "%s: failed to write 0x%x reg\n", __func__, arg[0]);
		return ret;
	}
	return len;
}

static const struct file_operations i2c_control_fops = {
	.owner = THIS_MODULE,
	.open = i2c_control_proc_open,
	.read = seq_read,
	.write = i2c_control_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_i2c_control_proc_file(void)
{
    i2c_control_proc_file = proc_create(I2C_CONTROL_PROC_FILE, 0666, NULL, &i2c_control_fops);
    if (i2c_control_proc_file) {
	printk("%s sucessed!\n", __func__);
    } else {
	printk("%s failed!\n", __func__);
    }
}
/*For CAMERA i2c Interface---*/

static int t4k35_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct t4k35_device *dev;
	struct camera_mipi_info *t4k35_info = NULL;
	int ret;
	char *msr_file_name = NULL;

	/*printk("[DEBUG] %s, start\n", __func__);*/
	pr_info("[Progress][%s] Probe starts\n", T4K35_NAME_K35);

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		pr_info("[Progress][%s] Probe ends\n", T4K35_NAME_K35);
		return -ENOMEM;
	}

	create_status_proc_file();/*For CAMERA STATUS Controll+++*/
	create_otp_proc_file();/*For CAMERA OTP Read+++*/
#if 0
	create_led_proc_file();/*For CAMERA FLASH Controll+++*/
	create_i2c_control_proc_file();/*For CAMERA i2c interface+++*/
#endif


	mutex_init(&dev->input_lock);

	dev->i2c_id = id->driver_data;
	dev->fmt_idx = 0;
	dev->sensor_id = T4K35_ID_DEFAULT;
	/*dev->vcm_driver = &t4k35_vcms[T4K35_ID_DEFAULT];*/

	v4l2_i2c_subdev_init(&(dev->sd), client, &t4k35_ops);

	if (client->dev.platform_data) {
		ret = t4k35_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret)
			goto out_free;
	}
	t4k35_info = v4l2_get_subdev_hostdata(&dev->sd);

	/*
	 * sd->name is updated with sensor driver name by the v4l2.
	 * change it to sensor name in this case.
	 */
	snprintf(dev->sd.name, sizeof(dev->sd.name), "%s%x %d-%04x",
		T4K35_SUBDEV_PREFIX, dev->sensor_id,
		i2c_adapter_id(client->adapter), client->addr);

	ret = __t4k35_init_ctrl_handler(dev);
	if (ret)
		goto out_ctrl_handler_free;

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = t4k35_translate_bayer_order(
		t4k35_info->raw_bayer_order);
	dev->sd.entity.ops = &t4k35_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		t4k35_remove(client);
		pr_info("[Progress][%s] Probe ends\n", T4K35_NAME_K35);
		return ret;
	}
#if 0
	/* Load the Noise reduction, Dead pixel registers from cpf file*/
	if (dev->platform_data->msr_file_name != NULL)
		msr_file_name = dev->platform_data->msr_file_name();
	if (msr_file_name) {
		ret = load_msr_list(client, msr_file_name, &dev->fw);
		if (ret) {
			t4k35_remove(client);
			pr_info("[Progress][%s] Probe end3\n", T4K35_NAME_K35);
			return ret;
		}
	} else {
		dev_warn(&client->dev, "Drvb file not present");
	}
#endif

	main_sd = &dev->sd;
	pr_info("[Progress][%s] Probe ends\n", T4K35_NAME_K35);
	return ret;

out_ctrl_handler_free:
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	pr_info("[Progress][%s] Probe ends\n", T4K35_NAME_K35);

out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	pr_info("[Progress][%s] Probe ends\n", T4K35_NAME_K35);
	return ret;
}

static const struct i2c_device_id t4k35_ids[] = {
#if 0	// T4K35
	{IMX_NAME_175, IMX175_ID},
	{IMX_NAME_135, IMX135_ID},
	{IMX_NAME_135_FUJI, IMX135_FUJI_ID},
	{IMX_NAME_134, IMX134_ID},
	{IMX_NAME_132, IMX132_ID},
#else	// T4K35
	{T4K35_NAME_K35, T4K35_ID},
#endif	// T4K35
	{}
};

MODULE_DEVICE_TABLE(i2c, t4k35_ids);

static struct i2c_driver t4k35_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = T4K35_DRIVER,
	},
	.probe = t4k35_probe,
	.remove = t4k35_remove,
	.id_table = t4k35_ids,
};


static __init int init_t4k35(void)
{
	printk("[DEBUG] %s, start\n", __func__);

	return i2c_add_driver(&t4k35_driver);
}

static __exit void exit_t4k35(void)
{
	printk("[DEBUG] %s, start\n", __func__);

	i2c_del_driver(&t4k35_driver);
}

module_init(init_t4k35);
module_exit(exit_t4k35);

MODULE_DESCRIPTION("A low-level driver for Toshiba sensors");
MODULE_AUTHOR("Shenbo Huang <shenbo.huang@intel.com>");
MODULE_LICENSE("GPL");

