/*
 * Support for hm2051 Camera Sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "hm2051.h"

#define to_hm2051_sensor(sd) container_of(sd, struct hm2051_device, sd)

//Add for ATD command+++
extern int build_version; //Add build version -> user:3, userdebug:2, eng:1
struct v4l2_subdev *main_sdx;

int ATD_hm2051_status = 0;
EXPORT_SYMBOL(ATD_hm2051_status);
extern int Read_FCAM_ID(void);
static char camera_module_otp[60];

static void *hm2051_otp_read(struct v4l2_subdev *sd);

static ssize_t hm2051_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	printk("%s: get hm2051 status (%d) !!\n", __func__, ATD_hm2051_status);
	//Check sensor connect status, just do it  in begining for ATD camera status

	return sprintf(buf,"%d\n", ATD_hm2051_status);
}

static ssize_t hm2051_read_otp(struct device *dev,struct device_attribute *attr,char *buf)
{
	printk("%s: get hm2051 module OTP %s !!\n", __func__, camera_module_otp);
	//Check sensor OTP value, just do it in begining for ATD camera status
/*
	if(build_version != 1){ //not eng, need to read otp first
		hm2051_otp_read(main_sd);
	}
*/
	return sprintf(buf,"%s", camera_module_otp);
}

static DEVICE_ATTR(hm2051_status, S_IRUGO,hm2051_show_status,NULL);
static DEVICE_ATTR(hm2051_read_otp, S_IRUGO,hm2051_read_otp,NULL);

static struct attribute *hm2051_attributes[] = {
	&dev_attr_hm2051_status.attr,
	&dev_attr_hm2051_read_otp.attr,
	NULL
};
//Add for ATD command---

static int
hm2051_read_reg(struct i2c_client *client, u16 data_length, u32 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != HM2051_8BIT) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = MSG_LEN_OFFSET;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u16) (reg >> 8);
	data[1] = (u16) (reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err >= 0) {
		*val = data[0];
		return 0;
	}

	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int
hm2051_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 val)
{
	int num_msg;
	struct i2c_msg msg;
	unsigned char data[6] = {0};
	u16 *wreg;
	int retry = 0;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != HM2051_8BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(u16) + data_length;
	msg.buf = data;

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	data[2] = (u8)(val);

	num_msg = i2c_transfer(client->adapter, &msg, 1);
	if (num_msg >= 0)
		return 0;

	dev_err(&client->dev, "write error: wrote 0x%x to offset 0x%x error %d",
		val, reg, num_msg);
	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying... %d", retry);
		retry++;
		msleep(20);
		goto again;
	}

	return num_msg;
}

/*
 * hm2051_write_reg_array - write a list of hm2051 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function write a list of registers.
 */
static int hm2051_write_reg_array(struct i2c_client *client,
				const struct hm2051_reg *reglist)
{
	const struct hm2051_reg *next = reglist;
	struct hm2051_write_ctrl ctrl;
	int ret;

	ctrl.index = 0;
	for (; next->type != HM2051_TOK_TERM; next++) {
		switch (next->type & HM2051_TOK_MASK) {
		case HM2051_TOK_DELAY:
			msleep(next->val);
			break;
		default:
			ret = hm2051_write_reg(client, next->type, next->reg, next->val);
			if (ret) {
				v4l2_err(client, "%s: write %x error, aborted\n",
					 __func__, next->reg);
				return ret;
			}
			break;
		}
	}
	return 0;
}

static int hm2051_set_suspend(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	printk("%s\n", __func__);

	ret = hm2051_write_reg_array(client, hm2051_stream_off);
	if (ret)
		return -EINVAL;
	printk("%s X\n", __func__);
	return 0;
}

/*For CAMERA STATUS Controll+++*/
#define	STATUS_PROC_FILE	"driver/vga_status"
static struct proc_dir_entry *status_proc_file;

static int status_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", ATD_hm2051_status);
    return 0;
}

static int status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, status_proc_read, NULL);
}

static ssize_t status_proc_write(struct file *filp, const char __user *buff,
			unsigned long len, void *data)
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

/*For CAMERA NAME Controll+++*/
#define	NAME_PROC_FILE	"driver/vga_name"
static struct proc_dir_entry *name_proc_file;

static int name_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%s\n", HM2051_NAME);
    return 0;
}

static int name_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, name_proc_read, NULL);
}

static ssize_t name_proc_write(struct file *filp, const char __user *buff,
			unsigned long len, void *data)
{
	pr_info("command not support\n");
    return len;
}

static const struct file_operations name_fops = {
	.owner = THIS_MODULE,
	.open = name_proc_open,
	.read = seq_read,
	.write = name_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_name_proc_file(void)
{
    name_proc_file = proc_create(NAME_PROC_FILE, 0666, NULL, &name_fops);
    if (name_proc_file) {
		printk("%s sucessed!\n", __func__);
    } else {
		printk("%s failed!\n", __func__);
    }
}
/*For CAMERA NAME Controll---*/

static int hm2051_set_streaming(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	printk("%s\n", __func__);

	ret = hm2051_write_reg_array(client, hm2051_stream_on);
	if (ret)
		return -EINVAL;
	printk("%s X\n", __func__);
	return 0;
}

static int hm2051_detect(struct hm2051_device *dev, struct i2c_client *client, bool flag)
{
	struct i2c_adapter *adapter = client->adapter;
	u32 reg_h, reg_l;
	u32 chip_id;
	int ret;
	printk("%s\n", __func__);
	if (flag)
		ATD_hm2051_status = 0;	/*Add for ATD command+++*/

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	ret = hm2051_read_reg(client, HM2051_8BIT, HM2051_REG_CHIP_ID_H, &reg_h);
	if (ret) {
		v4l2_err(client, "%s: fail to read HM2051_REG_CHIP_ID_H\n", __func__);
		return -EINVAL;
	}

	ret = hm2051_read_reg(client, HM2051_8BIT, HM2051_REG_CHIP_ID_L, &reg_l);
	if (ret) {
		v4l2_err(client, "%s: fail to read HM2051_REG_CHIP_ID_L\n", __func__);
		return -EINVAL;
	}

	chip_id = reg_l | (reg_h << 8);

	if (chip_id != HM2051_MOD_ID) {
		dev_err(&client->dev, "%s: failed: client->addr = %x, id read %x\n",
			__func__, client->addr, chip_id);
		return -ENODEV;
	} else {
		dev_info(&client->dev, "%s sensor ID is 0x%x\n", __func__, chip_id);
	}
	printk("%s X\n", __func__);
	if (flag)
		ATD_hm2051_status = 1;	/*Add for ATD command+++*/

	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	printk("%s\n", __func__);

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "power_ctrl failed\n");
		goto fail_power;
	}
	usleep_range(1000, 1100); /*make sure mclock after AVDD(2.8V)*/
	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "flisclk_ctrl failed\n");
		goto fail_clk;
	}
	/*usleep_range(1000, 1100);*/
	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "gpio_ctrl failed\n");
	}
	usleep_range(100, 110);
	printk("%s X\n", __func__);
	return 0;

fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}
/*Change*/
static int power_down(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
    /*return 0;*/
	printk("%s\n", __func__);

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data\n");
		return -ENODEV;
	}

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	/*according to DS, 20ms is needed after power down*/
	msleep(20);

	printk("%s X\n", __func__);
	return ret;
}

static int hm2051_s_power(struct v4l2_subdev *sd, int power)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;
	printk("%s on/off %d\n", __func__, power);

	mutex_lock(&dev->input_lock);
	if (power == 0) {
		ret = power_down(sd);
	} else {
		ret = power_up(sd);
		if (ret)
			v4l2_err(client, "hm2051 power-up err");

		ret = hm2051_detect(dev, client, 0);
		if (ret)
			v4l2_err(client, "hm2051_detect err s_config\n");
#if 0
		if (!ret) {
			ret = hm2051_write_reg_array(client, hm2051_init);
		}
#endif
	}
	mutex_unlock(&dev->input_lock);
	printk("%s X\n", __func__);
	return ret;
}

static int hm2051_to_res(struct v4l2_subdev *sd, u32 w, u32 h)
{
	int idx;
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	printk("%s\n", __func__);
	for (idx = 0; idx < dev->n_res; idx++) {
		if ((dev->hm2051_res[idx].width == w) &&
		    (dev->hm2051_res[idx].height == h))
			break;
	}

	/* No mode found */
	if (idx >= dev->n_res)
		return -1;
	printk("%s X\n", __func__);
	return idx;


}
/*Change*/
static int hm2051_try_res(struct v4l2_subdev *sd, u32 *w, u32 *h)
{
	int idx, idx_ratio;
	struct hm2051_device *dev = to_hm2051_sensor(sd);
    int set_ratio, sup_ratio;
	int ratio_correct = 0;
	printk("%s wxh=%dx%d\n", __func__, *w, *h);
	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (idx = 0; idx < dev->n_res; idx++) {
		if ((dev->hm2051_res[idx].width >= *w) &&
		    (dev->hm2051_res[idx].height >= *h)) {
				/*printk("ZZ idx=%d width =%d height=%d ", idx, dev->hm2051_res[idx].width, dev->hm2051_res[idx].height);*/
				break;
		}
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (idx == dev->n_res)
		idx--;

	/*check ratio*/
	set_ratio = (*w * 10)/(*h);
	for (ratio_correct = 0, idx_ratio = idx ; idx_ratio < dev->n_res ; idx_ratio++) {
		sup_ratio = (dev->hm2051_res[idx_ratio].width * 10)/(dev->hm2051_res[idx_ratio].height);
		/*printk("ZZ idx_ratio=%d sup_ratio=%d  set_ratio=%d ", idx_ratio, sup_ratio, set_ratio);*/
		if ((set_ratio > 15 && sup_ratio > 15) ||
		(set_ratio < 13 && sup_ratio < 13) ||
		((set_ratio >= 13 && set_ratio <= 15) && (sup_ratio >= 13 && sup_ratio <= 15))
			) {
					printk("ZZ %s Match res:  idx_ratio=%d sup_ratio=%d  for set_ratio=%d ",  __func__, idx_ratio, sup_ratio, set_ratio);
		ratio_correct = 1;
		break;
		}
	}
	if (ratio_correct)
		idx = idx_ratio;

	*w = dev->hm2051_res[idx].width;
	*h = dev->hm2051_res[idx].height;
	printk("ZZ %s idx[%d]=%s X\n", __func__, idx, dev->hm2051_res[idx].desc);
	return 0;
}

static int hm2051_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;
	printk("%s\n", __func__);
	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = hm2051_try_res(sd, &fmt->width, &fmt->height);
	mutex_unlock(&dev->input_lock);
	printk("%s X\n", __func__);
	return ret;
}

static int hm2051_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct hm2051_res_struct *res)
{
	struct atomisp_sensor_mode_data *buf = &info->data;
	u32 reg_val_l, reg_val_h;
	int ret;

	printk("%s\n", __func__);
	if (info == NULL || res == NULL)
		return -EINVAL;

	buf->vt_pix_clk_freq_mhz = res->pixel_clk;
	buf->frame_length_lines = res->frame_length_lines;
	buf->line_length_pck = res->line_length_pck;

	/* get integration time */
	buf->coarse_integration_time_min = HM2051_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
					HM2051_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min = HM2051_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
					HM2051_FINE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_def = HM2051_FINE_INTG_TIME_MIN;

	buf->output_width = res->width;
	buf->output_height = res->height;
	buf->read_mode = res->bin_mode;
	buf->binning_factor_x = res->bin_factor_x;
	buf->binning_factor_y = res->bin_factor_y;
	buf->crop_horizontal_start = res->horizontal_start;
	buf->crop_horizontal_end = res->horizontal_end;
	buf->crop_vertical_start = res->vertical_start;
	buf->crop_vertical_end = res->vertical_end;

	dev_info(&client->dev, "%s h start %d end %d v start %d end %d "
		"frame_length_line %d line_length_pck %d "
		" read mode %d binning_x %d binning_y %d\n", __func__,
		buf->crop_horizontal_start, buf->crop_horizontal_end,
		buf->crop_vertical_start, buf->crop_vertical_end,
		buf->frame_length_lines, buf->line_length_pck,
		buf->read_mode,	buf->binning_factor_x, buf->binning_factor_y);
	printk("%s X\n", __func__);
	return 0;
}

static int hm2051_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	printk("%s\n", __func__);
	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fmt->width = dev->hm2051_res[dev->fmt_idx].width;
	fmt->height = dev->hm2051_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;
	mutex_unlock(&dev->input_lock);
	printk("%s X\n", __func__);
	return 0;
}

static int hm2051_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	u32 width = fmt->width;
	u32 height = fmt->height;
	struct camera_mipi_info *hm2051_info = NULL;
	int ret;
	int index;
	printk("%s\n", __func__);
	if (!fmt)
		return -EINVAL;

	hm2051_info = v4l2_get_subdev_hostdata(sd);

	if (hm2051_info == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	hm2051_try_res(sd, &width, &height);

	dev_info(&client->dev, "%s %d x %d => %d x %d\n", __func__, fmt->width, fmt->height, width, height);

	dev->fmt_idx = hm2051_to_res(sd, width, height);
	if (dev->fmt_idx == -1) {
		dev->fmt_idx = 0;
	  	mutex_unlock(&dev->input_lock);
		dev_err(&client->dev, "%s no resolution found\n", __func__);
		return -EINVAL;
	}

	printk("%s RES %s selected\n", __func__, dev->hm2051_res[dev->fmt_idx].desc);
	ret = hm2051_write_reg_array(client, dev->hm2051_res[dev->fmt_idx].regs);

	if (ret) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	if (hm2051_set_suspend(sd)) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	/*
	 * Marked current sensor res as being "used"
	 *
	 * REVISIT: We don't need to use an "used" field on each mode
	 * list entry to know which mode is selected. If this
	 * information is really necessary, how about to use a single
	 * variable on sensor dev struct?
	 */
	for (index = 0; index < dev->n_res; index++) {
		if ((width == dev->hm2051_res[index].width) &&
		    (height == dev->hm2051_res[index].height)) {
			dev->hm2051_res[index].used = 1;
			continue;
		}
		dev->hm2051_res[index].used = 0;
	}

	ret = hm2051_get_intg_factor(client, hm2051_info,
					&dev->hm2051_res[dev->fmt_idx]);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		dev_err(&client->dev, "failed to get integration_factor\n");
		return -EINVAL;
	}

	fmt->width = width;
	fmt->height = height;
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

	mutex_unlock(&dev->input_lock);
	printk("%s X\n", __func__);
	return 0;
}

static int hm2051_q_exposure(struct v4l2_subdev *sd, s32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u32 integration_time_h = 0;
	u32 integration_time_l = 0;
	int ret = 0;
	printk("%s\n", __func__);
	ret = hm2051_read_reg(client, HM2051_8BIT, HM2051_REG_INTEGRATION_TIME_H, &integration_time_h);
	if (ret) {
		v4l2_err(client, "%s: read HM2051_REG_INTEGRATION_TIME_H error %d\n", __func__, ret);
		return ret;
	}

	ret = hm2051_read_reg(client, HM2051_8BIT, HM2051_REG_INTEGRATION_TIME_L, &integration_time_l);
	if (ret) {
		v4l2_err(client, "%s: read HM2051_REG_INTEGRATION_TIME_L error %d\n", __func__, ret);
		return ret;
	}
	printk("%s X\n", __func__);
	*val = integration_time_l | (integration_time_h << 8);
	return 0;
}

/*Change*/
static long __hm2051_set_adgain(struct v4l2_subdev *sd, int gain, int digitgain) {
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  //struct hm2051_device *dev = to_hm2051_sensor(sd);
  int ret= 0, iGainBase=32, iMaxGainX100=6169;  //max AgainX100 (3100) * max Dgain (iMaxDgainX100) 
  int iGainX100 = (gain*100)/iGainBase;  
  int iAgainReg = 0 , iFineAgainReg = 0, iFineAgainBase=16;
  int iAgainRegPowX100 = 100, iMaxAgainReg=3, iMaxAgainRegPowX100 = 800; //iMaxAgainReg and iMaxAgainRegPowX10   
  int iDgainReg = 0;
  int iMaxDgainX100=398, iMinDgainX100=100, iDGainBase=0x40;
  int iTemp;
	printk("%s\n", __func__);
  if(gain < 0)
      return -EINVAL;


  /* gain = Analog gain * Digitgain*/
  /* [8*(1+15/16)]*[(3.98)] = [31/2]*[3.98] */  
  /* Analog gain */
  //get the approach Analog gain base on gain 
  //Adgain = 2^iAgainReg*(1+iFineAgainReg/iFineAgainBase)
  //pr_info("PJ: %s analog gain 0x%x, iGainX100 %d\n", __func__, gain, iGainX100);
  iGainX100 = iGainX100 > iMaxGainX100 ? iMaxGainX100:(iGainX100 < 100  ? 100: iGainX100); 
      
  iTemp = iGainX100/200;
  while(iTemp > 0) {
      iAgainReg += 1;
      iTemp >>= 1;
      iAgainRegPowX100 *= 2;  
  }

  /*
  if (iAgainReg > iMaxAgainReg) {
      pr_info("PJ: %s cal coarse gain error -> gain=%d, iAgainReg=%d, iAgainRegPowX100=%d\n", __func__, gain, iAgainReg, iAgainRegPowX100);
  }
  */
  
  iAgainReg = iAgainReg > iMaxAgainReg ? iMaxAgainReg : iAgainReg;
  iAgainRegPowX100 =  iAgainRegPowX100 > iMaxAgainRegPowX100? iMaxAgainRegPowX100 : iAgainRegPowX100;     
  iTemp = (iFineAgainBase * iGainX100)/iAgainRegPowX100 - iFineAgainBase;
  
  /*
  if (iTemp  < 0 || iTemp >= iFineAgainBase) {
      pr_info("PJ: %s cal final gain error -> gain=%d, iAgainReg=%d, iFineAgainReg=%d, iAgainRegPowX100=%d\n", __func__, gain, iAgainReg, iTemp, iAgainRegPowX100);
  }
  */
  
  iFineAgainReg = iTemp  < 0 ? 0 : (iTemp >= iFineAgainBase ? iFineAgainBase -1 : iTemp); 
  gain = (iAgainReg ) + (iFineAgainReg<<4);
  //pr_info("PJ: set Again -> %s Again=0x%X, iAgainReg=%d, iFineAgainReg=%d\n", __func__, gain, iAgainReg, iFineAgainReg);
  
  
  ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_AGAIN, gain); /*gain & 0xff*/
  if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_AGAIN);
		return ret;
	}
  
  /* Digitgain*/
  /*Digitgain = gain / Analog gain*/
  //Digitgain = iDgainReg*(1+iFineDGainReg/iFineDGainBase)                         
  iGainX100 = (iGainX100*100)/(iAgainRegPowX100 + (iAgainRegPowX100 * iFineAgainReg)/ iFineAgainBase);
  //pr_info("PJ: %s Cal Dgain : iAgainReg=0x%X, iFineAgainReg=%d, DgainX100: %d\n", __func__,  iAgainReg, iFineAgainReg, iGainX100); 
  
  /*
  if ( iGainX100 < iMinDgainX100 || iGainX100 > iMaxDgainX100)
      pr_info("PJ: %s Dgain Error 0x%x, iGainX100 %d\n", __func__, gain, iGainX100);
  */
  
  iGainX100 = iGainX100 < iMinDgainX100 ? iMinDgainX100 : (iGainX100 > iMaxDgainX100 ? iMaxDgainX100:iGainX100);  
  
  //Digital gain = DGAIN[7:0] / 0x40
  iDgainReg = (iGainX100 * iDGainBase)/100;
  gain = iDgainReg = (iDgainReg < iDGainBase ? iDGainBase : iDgainReg);  
  //pr_info("PJ:  %s set Dgain -> iGainX100=%d,  iDGainBase=0x%X,  iDgainReg=%0xX\n", __func__, iGainX100, iDGainBase, iDgainReg); 

  ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_DGAIN, gain);
	
  if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_DGAIN);
		return ret;
  }  
	printk("%s X\n", __func__);
  return ret;
} 
/*Change*/
static long __hm2051_set_exposure(struct v4l2_subdev *sd, int coarse_itg,
				 int gain, int digitgain)

{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	u16 vts, hts;
	int ret= 0;
 
	hts = dev->hm2051_res[dev->fmt_idx].line_length_pck;
	vts = dev->hm2051_res[dev->fmt_idx].frame_length_lines;


	pr_info("PJ: %s in\n", __func__);
  //return ret; //dor debug 
	/* Increase the VTS to match exposure + MARGIN */
  /* data blanking*/
	if (vts < coarse_itg )
		vts = (u16) coarse_itg + HM2051_INTEGRATION_TIME_MARGIN;
 
  //pr_info("PJ: %s vts = %d , lines_per_frame = %d , coarse_itg = %d\n", __func__, vts , hm2051_res[dev->fmt_idx].lines_per_frame, coarse_itg); 

  #ifdef FIXED_FRAME_RATE
	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_TIMING_VTS_H, ((vts - hm2051_res[dev->fmt_idx].height) & 0xFF00) >> 8);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_TIMING_VTS_H);
		return ret;
	}

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_TIMING_VTS_L, (vts - hm2051_res[dev->fmt_idx].height) & 0x00FF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_TIMING_VTS_L);
		return ret;
	}
  #endif
	/* set exposure */
   
	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_EXPOSURE_H, (coarse_itg >> 8) & 0xFF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_EXPOSURE_H);
		return ret;
	}  
  
	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_EXPOSURE_L, coarse_itg & 0xFF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_EXPOSURE_L);
		return ret;
	}
  
  /* set AD gain */
  ret = __hm2051_set_adgain(sd, gain, digitgain);
	if (ret) {
		dev_err(&client->dev, "%s: Set adgain error, aborted\n",
			__func__);
		return ret;
	}
  
   
	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_COMMAND_UPDATE, 1);
	if (ret) {
		dev_err(&client->dev, "%s: Write HM2051_COMMAND_UPDATE %x error, aborted\n",
			__func__, HM2051_COMMAND_UPDATE);
		return ret;
	}
	pr_info("PJ: %s out\n", __func__);
	return ret;
  
}

/*Change*/
static long hm2051_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	/*printk("%s\n", __func__);*/
#if 0
  u16 coarse_itg = exposure->integration_time[0];
	u16 analog_gain = exposure->gain[0];
	u16 digital_gain = exposure->gain[1];

  mutex_lock(&dev->input_lock);
  ret = __hm2051_set_exposure(sd, coarse_itg, analog_gain/32, digital_gain);
	mutex_unlock(&dev->input_lock);
#endif

	/*u8 analog_gain = 0;
	u8 digital_gain = 0;*/
	u32 reg_val_l, reg_val_h;
	u32 dit_dig_gain = 0;
	unsigned int frame_length_lines = 0;
	unsigned int analog_gain = exposure->gain[0];  /*real_gain = exposure->gain[0];*/
	unsigned int digital_gain = exposure->gain[1];
	mutex_lock(&dev->input_lock);

/*    frame_length_lines = dev->hm2056_raw_res[dev->fmt_idx].frame_length_lines;

	if (frame_length_lines < exposure->integration_time[0]) {
//		dev_info(&client->dev, "%s update frame_length_line %d "
//			"integration_time %d\n", __func__, frame_length_lines,
//			exposure->integration_time[0]);
		frame_length_lines = exposure->integration_time[0] + 5;
	}

	reg_val_l = (frame_length_lines - dev->hm2056_raw_res[dev->fmt_idx].height) & 0xFF;
	reg_val_h = ((frame_length_lines - dev->hm2056_raw_res[dev->fmt_idx].height) & 0xFF00) >> 8;

	ret = hm2056_raw_write_reg(client, HM2056_8BIT, HM2056_REG_BLANKING_ROW_H, reg_val_h);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2056_REG_BLANKING_ROW_H error %x\n", __func__, reg_val_h);
		return ret;
	}

	ret = hm2056_raw_write_reg(client, HM2056_8BIT, HM2056_REG_BLANKING_ROW_L, reg_val_l);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2056_REG_BLANKING_ROW_L error %x\n", __func__, reg_val_l);
		return ret;
	}*/

	reg_val_h = (exposure->integration_time[0] & 0xFF00) >> 8;
	reg_val_l = exposure->integration_time[0] & 0xFF;

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_REG_INTEGRATION_TIME_H, reg_val_h);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2051_REG_INTEGRATION_TIME_H error %x\n", __func__, reg_val_h);
		return ret;
	}

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_REG_INTEGRATION_TIME_L, reg_val_l);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2051_REG_INTEGRATION_TIME_L error %x\n", __func__, reg_val_l);
		return ret;
	}

#if 0
	/* DIT request the AG and DG can't more than 4x, so the max gain value will be 4xAG*4xDG=16x*/
	if (16 <= real_gain && real_gain < 32) {
		analog_gain = 0x0;
		digital_gain = real_gain * 4;
	} else if (32 <= real_gain && real_gain < 64) {
		analog_gain = 0x1;
		dit_dig_gain = real_gain * 2 * 111 / 100;
		digital_gain = dit_dig_gain & 0xff;
	} else if (64 <= real_gain && real_gain <= 256) {
		analog_gain = 0x2;
		if (real_gain < 233)
			dit_dig_gain = real_gain * 109 / 100;
		else
			dit_dig_gain = 253;
		digital_gain = dit_dig_gain & 0xff;
	} else
		v4l2_err(client, "unsupported gain value\n");
#endif
	/*v4l2_err(client, "%s: write again = %x, dgain = %x\n", __func__, analog_gain, digital_gain);*/
	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_REG_AGAIN, analog_gain);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2051_REG_AGAIN error %x\n", __func__, analog_gain);
		return ret;
	}

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_REG_DGAIN, digital_gain);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2056_REG_DGAIN error %x\n", __func__, digital_gain);
		return ret;
	}

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_REG_COMMAND_UPDATE, 1);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2056_REG_COMMAND_UPDATE error\n", __func__);
		return ret;
	}
	mutex_unlock(&dev->input_lock);

	/*printk("%s X\n", __func__);*/
	return ret;

}


static long hm2051_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return hm2051_s_exposure(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int hm2051_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	*val = dev->hm2051_res[dev->fmt_idx].bin_factor_x - 1;
	return 0;
}

static int hm2051_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	*val = dev->hm2051_res[dev->fmt_idx].bin_factor_y - 1;
	return 0;
}

static int hm2051_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	/*printk("[DEBUG] %s, start\n", __func__);*/

	*val = (HM2051_FOCAL_LENGTH_NUM << 16) | HM2051_FOCAL_LENGTH_DEM;
	return 0;
}

static int hm2051_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*printk("[DEBUG] %s, start\n", __func__);*/

	*val = (HM2051_F_NUMBER_DEFAULT_NUM << 16) | HM2051_F_NUMBER_DEM;
	return 0;
}

static int hm2051_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	/*printk("[DEBUG] %s, start\n", __func__);*/

	*val = (HM2051_F_NUMBER_DEFAULT_NUM << 24) |
		(HM2051_F_NUMBER_DEM << 16) |
		(HM2051_F_NUMBER_DEFAULT_NUM << 8) | HM2051_F_NUMBER_DEM;
	return 0;
}

static struct hm2051_control hm2051_controls[] = {
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
		.query = hm2051_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = 2,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = hm2051_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = 2,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = hm2051_g_bin_factor_y,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = HM2051_FOCAL_LENGTH_DEFAULT,
			.maximum = HM2051_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = HM2051_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = hm2051_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = HM2051_F_NUMBER_DEFAULT,
			.maximum = HM2051_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = HM2051_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = hm2051_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = HM2051_F_NUMBER_RANGE,
			.maximum =  HM2051_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = HM2051_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = hm2051_g_fnumber_range,
	},
};
#define N_CONTROLS (ARRAY_SIZE(hm2051_controls))

static struct hm2051_control *hm2051_find_control(__u32 id)
{
	int i;
	for (i = 0; i < N_CONTROLS; i++) {
		if (hm2051_controls[i].qc.id == id) {
			return &hm2051_controls[i];
		}
	}
	return NULL;
}

static int
hm2051_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	printk("%s\n", __func__);

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
	    (struct camera_sensor_platform_data *)platform_data;

	mutex_lock(&dev->input_lock);
	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "hm2051 platform init err\n");
			return ret;
		}
	}
	ret = power_up(sd);
	if (ret) {
		v4l2_err(client, "hm2051 power-up err");
		goto fail_detect;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret) {
		v4l2_err(client, "hm2051 config csi err");
		goto fail_csi_cfg;
	}

	/*ret = hm2051_write_reg_array(client, hm2051_init);*/

	/* config & detect sensor */
	ret = hm2051_detect(dev, client, 1);
	if (ret) {
		v4l2_err(client, "hm2051_detect err s_config.\n");
		goto fail_detect;
	}

	ret = power_down(sd);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "hm2051 power down err");
		return ret;
	}

	mutex_unlock(&dev->input_lock);
	printk("%s X\n", __func__);
	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
fail_detect:
	power_down(sd);
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int hm2051_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct hm2051_control *ctrl = hm2051_find_control(qc->id);
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int hm2051_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	if (!param)
		return -EINVAL;

	dev->run_mode = param->parm.capture.capturemode;

	mutex_lock(&dev->input_lock);

	switch (dev->run_mode) {
		case CI_MODE_VIDEO:
			dev->hm2051_res = hm2051_res_video;
			dev->n_res = N_RES_VIDEO;
			break;
		case CI_MODE_STILL_CAPTURE:
			dev->hm2051_res = hm2051_res_still;
			dev->n_res = N_RES_STILL;
			break;
		default:
			dev->hm2051_res = hm2051_res_preview;
			dev->n_res = N_RES_PREVIEW;
	}

	dev->fmt_idx = 0;

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int hm2051_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct hm2051_control *octrl = hm2051_find_control(ctrl->id);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2051_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct hm2051_control *octrl = hm2051_find_control(ctrl->id);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2051_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	dev_info(&client->dev, "%s enable %d\n", __func__, enable);

	mutex_lock(&dev->input_lock);
	if (enable)
		ret = hm2051_set_streaming(sd);
	else
		ret = hm2051_set_suspend(sd);

	mutex_unlock(&dev->input_lock);
	pr_info("PJ: %s ret ->%d\n", __func__, ret);

	return ret;
}

static int
hm2051_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	if (index >= dev->n_res)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->hm2051_res[index].width;
	fsize->discrete.height = dev->hm2051_res[index].height;
	fsize->reserved[0] = dev->hm2051_res[index].used;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int hm2051_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	printk("%s\n", __func__);
	if (index >= dev->n_res)
		return -EINVAL;

	mutex_lock(&dev->input_lock);

	/* find out the first equal or bigger size */
	for (i = 0; i < dev->n_res; i++) {
		if ((dev->hm2051_res[i].width >= fival->width) &&
		    (dev->hm2051_res[i].height >= fival->height))
			break;
	}
	if (i == dev->n_res)
		i--;

	index = i;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = dev->hm2051_res[index].fps;

	mutex_unlock(&dev->input_lock);
	printk("%s X\n", __func__);
	return 0;
}

static int
hm2051_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_HM2051, 0);
}

static int hm2051_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	/*printk("%s\n", __func__);*/
	if (!code || code->index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	code->code = V4L2_MBUS_FMT_SBGGR10_1X10;
	mutex_unlock(&dev->input_lock);
	/*printk("%s X\n", __func__);*/
	return code->code < 0 ? code->code : 0;
}

static int hm2051_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int index;
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	printk("%s\n", __func__);
	if (!fse)
		return -EINVAL;

	index = fse->index;

	if (index >= dev->n_res)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fse->min_width = dev->hm2051_res[index].width;
	fse->min_height = dev->hm2051_res[index].height;
	fse->max_width = dev->hm2051_res[index].width;
	fse->max_height = dev->hm2051_res[index].height;
	mutex_unlock(&dev->input_lock);
	printk("%s X\n", __func__);
	return 0;
}

static struct v4l2_mbus_framefmt *
__hm2051_get_pad_format(struct hm2051_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	printk("%s\n", __func__);
	if (pad != 0) {
		dev_err(&client->dev,  "%s err. pad %x\n", __func__, pad);
		return NULL;
	}

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
	printk("%s V4L2_SUBDEV_FORMAT_TRY X\n", __func__);
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
	printk("%s V4L2_SUBDEV_FORMAT_ACTIVE X\n", __func__);
		return &sensor->format;
	default:
	printk("%s X\n", __func__);
		return NULL;
	}
}

static int
hm2051_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct hm2051_device *snr = to_hm2051_sensor(sd);
	struct v4l2_mbus_framefmt *format;
	printk("%s\n", __func__);
	if (!fmt)
		return -EINVAL;

	format =__hm2051_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	printk("%s X\n", __func__);
	return 0;
}

static int
hm2051_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct hm2051_device *snr = to_hm2051_sensor(sd);
	struct v4l2_mbus_framefmt *format;
	printk("%s\n", __func__);
	if (!fmt)
		return -EINVAL;

	format =__hm2051_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;
	printk("%s X\n", __func__);
	return 0;
}

static int hm2051_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	printk("%s\n", __func__);
	if (frames == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*frames = dev->hm2051_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);
	printk("%s X\n", __func__);
	return 0;
}
static const struct v4l2_subdev_video_ops hm2051_video_ops = {
	.s_parm = hm2051_s_parm,
	.try_mbus_fmt = hm2051_try_mbus_fmt,
	.s_mbus_fmt = hm2051_set_mbus_fmt,
	.g_mbus_fmt = hm2051_get_mbus_fmt,
	.s_stream = hm2051_s_stream,
	.enum_framesizes = hm2051_enum_framesizes,
	.enum_frameintervals = hm2051_enum_frameintervals,
};

static struct v4l2_subdev_sensor_ops hm2051_sensor_ops = {
	.g_skip_frames	= hm2051_g_skip_frames,
};

static const struct v4l2_subdev_core_ops hm2051_core_ops = {
	.g_chip_ident = hm2051_g_chip_ident,
	.queryctrl = hm2051_queryctrl,
	.g_ctrl = hm2051_g_ctrl,
	.s_ctrl = hm2051_s_ctrl,
	.s_power = hm2051_s_power,
	.ioctl = hm2051_ioctl,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops hm2051_pad_ops = {
	.enum_mbus_code = hm2051_enum_mbus_code,
	.enum_frame_size = hm2051_enum_frame_size,
	.get_fmt = hm2051_get_pad_format,
	.set_fmt = hm2051_set_pad_format,
};

static const struct v4l2_subdev_ops hm2051_ops = {
	.core = &hm2051_core_ops,
	.video = &hm2051_video_ops,
	.pad = &hm2051_pad_ops,
	.sensor = &hm2051_sensor_ops,
};

static int hm2051_remove(struct i2c_client *client)
{
	struct hm2051_device *dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	dev = container_of(sd, struct hm2051_device, sd);

	dev->platform_data->csi_cfg(sd, 0);

	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);
	return 0;
}

static int hm2051_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct hm2051_device *dev;
	int ret;

	HW_ID = Read_HW_ID();
	pr_info("[Progress][%s] HW_ID = %d\n", HM2051_NAME, HW_ID);
	pr_info("[Progress][%s] PCB_ID0 = %d\n", HM2051_NAME, Read_FCAM_ID());
	pr_info("[Progress][%s] PCB_ID2 = %d\n", HM2051_NAME, gpio_get_value(114));

	if (HW_ID >= 3) {  /*for PR HW change PCB_ID2*/
		int pcb_value = gpio_get_value(114);  /* GP_CORE_018 */
		if (!pcb_value) {
			pr_info("[Progress][%s] PCB_ID2(%d) not match\n", HM2051_NAME, pcb_value);
			return 1;
		}
	}

	pr_info("[Progress][%s] Probe starts\n",HM2051_NAME);

	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		pr_info("[Progress][%s] Probe ends\n",HM2051_NAME);
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);
	dev->fmt_idx = 0;
	dev->hm2051_res = hm2051_res_preview;
	dev->n_res = N_RES_STILL;

	//Add for ATD command+++
	dev->sensor_i2c_attribute.attrs = hm2051_attributes;

	// Register sysfs hooks
	ret = sysfs_create_group(&client->dev.kobj, &dev->sensor_i2c_attribute);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		pr_info("[Progress][%s] Probe ends\n",HM2051_NAME);
		return ret;
	}
	//Add for ATD command---

	v4l2_i2c_subdev_init(&dev->sd, client, &hm2051_ops);

	if (client->dev.platform_data) {
		ret = hm2051_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			pr_info("[Progress][%s] Probe ends\n",HM2051_NAME);
			return ret;
		} else {
			create_status_proc_file();  /*For CAMERA STATUS Controll+++*/
			create_name_proc_file();  /*For CAMERA NAME Controll+++*/
		}
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		hm2051_remove(client);
	}

	main_sdx = &dev->sd;	//Add for ATD command+++

	pr_info("[Progress][%s] Probe ends\n",HM2051_NAME);
       
	return ret;
}

MODULE_DEVICE_TABLE(i2c, hm2051_id);

static struct i2c_driver hm2051_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = HM2051_NAME,
	},
	.probe = hm2051_probe,
	.remove = hm2051_remove,
	.id_table = hm2051_id,
};

static __init int init_hm2051(void)
{
	return i2c_add_driver(&hm2051_driver);
}

static __exit void exit_hm2051(void)
{
	i2c_del_driver(&hm2051_driver);
}

module_init(init_hm2051);
module_exit(exit_hm2051);

MODULE_AUTHOR("Hayden Huang <hayden.huang@intel.com>");
MODULE_DESCRIPTION("A low-level driver for Himax HM2051 sensors");
MODULE_LICENSE("GPL");
