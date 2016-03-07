/*
 * Support for HM2051 Camera Sensor.
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

#ifndef __HM2051_H__
#define __HM2051_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <linux/HWVersion.h>

extern int Read_HW_ID(void);
static unsigned int HW_ID = 0xFF;

#ifndef V4L2_IDENT_HM2051
#define V4L2_IDENT_HM2051			8245
#endif
#define HM2051_NAME				"hm2051"

#define I2C_RETRY_COUNT				5
#define MSG_LEN_OFFSET				2
#define MAX_FMTS 				1

//registers
#define HM2051_REG_CHIP_ID_H			0x0001
#define HM2051_REG_CHIP_ID_L			0x0002

/*
#define HM2051_REG_BLANKING_ROW_H			0x0010
#define HM2051_REG_BLANKING_ROW_L			0x0011
#define HM2051_REG_BLANKING_COLUMN_CLK		0x0012
#define HM2051_REG_BLANKING_COLUMN			0x0013
*/

#define HM2051_REG_INTEGRATION_TIME_H		0x0015
#define HM2051_REG_INTEGRATION_TIME_L		0x0016

#define HM2051_REG_AGAIN			0x0018
#define HM2051_REG_DGAIN			0x001D

#define HM2051_REG_COMMAND_UPDATE		0x0100

/*
#define HM2051_REG_H_START_L			0x0661
#define HM2051_REG_H_START_H			0x0660
#define HM2051_REG_H_END_L			0x0663
#define HM2051_REG_H_END_H			0x0662

#define HM2051_REG_V_START_L			0x0665
#define HM2051_REG_V_START_H			0x0664
#define HM2051_REG_V_END_L			0x0667
#define HM2051_REG_V_END_H			0x0666
*/

/* Add by marky */
/*
//#define HM2051_SW_RESET					0x0022
//#define HM2051_SW_STREAM				0x0005

//#define HM2051_SC_CMMN_CHIP_ID_H		0x0001  //by orig
//#define HM2051_SC_CMMN_CHIP_ID_L		0x0002  //by orig
*/

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define HM2051_FOCAL_LENGTH_DEFAULT 0x1B70064
#define HM2051_FOCAL_LENGTH_NUM	293	/*2.93mm*/
#define HM2051_FOCAL_LENGTH_DEM	100

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define HM2051_F_NUMBER_DEFAULT 0x1C000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define HM2051_F_NUMBER_RANGE 0x1C0a1C0a
#define HM2051_F_NUMBER_DEFAULT_NUM	28
#define HM2051_F_NUMBER_DEM	10


#define HM2051_COMMAND_UPDATE   HM2051_REG_COMMAND_UPDATE /*0x0100  //by orig*/

#define HM2051_EXPOSURE_H				HM2051_REG_INTEGRATION_TIME_H /*0x0015  //by orig*/
#define HM2051_EXPOSURE_L				HM2051_REG_INTEGRATION_TIME_L /*0x0016  //by orig*/
#define HM2051_AGAIN            HM2051_REG_AGAIN /*0x0018  //by orig*/
#define HM2051_DGAIN            HM2051_REG_DGAIN /*0x001D  //by orig*/

#define HM2051_INTEGRATION_TIME_MARGIN	5

#ifdef FIXED_FRAME_RATE
#define HM2051_TIMING_VTS_H				0x0010
#define HM2051_TIMING_VTS_L				0x0011
#endif

#define HM2051_START_STREAMING			0x03
#define HM2051_STOP_STREAMING		    0x02


#define FULL_SPEED_HM2051
#ifdef FULL_SPEED_HM2051
#define HM2051_PCLK             88533333
#define HM2051_CLK_DIV          0x00
#else
#define HM2051_PCLK             35200000
#define HM2051_CLK_DIV          0x05
#endif
/* DEVICE_ID */
#define HM2051_MOD_ID				0x2051

#define HM2051_FINE_INTG_TIME_MIN		0
#define HM2051_FINE_INTG_TIME_MAX_MARGIN	65535
#define HM2051_COARSE_INTG_TIME_MIN		0
#define HM2051_COARSE_INTG_TIME_MAX_MARGIN	65535

enum hm2051_tok_type {
	HM2051_8BIT  = 0x0001,
	HM2051_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	HM2051_TOK_DELAY  = 0xfe00,	/* delay token for reg list */
	HM2051_TOK_MASK = 0xfff0
};

struct hm2051_reg {
	enum hm2051_tok_type type;
	u16 reg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

struct hm2051_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct mutex input_lock;

	struct hm2051_res_struct *hm2051_res;
	int n_res;
	int fmt_idx;
	int run_mode;

	struct camera_sensor_platform_data *platform_data;
	char name[32];

	struct attribute_group sensor_i2c_attribute; //Add for ATD read camera status+++
};

struct hm2051_res_struct {
	u8 *desc;
	int width;
	int height;
	int fps;
	int skip_frames;
	bool used;
	struct hm2051_reg const *regs;
	int horizontal_start;
	int horizontal_end;
	int vertical_start;
	int vertical_end;
	int pixel_clk;
	int line_length_pck;
	int frame_length_lines;
	u8 bin_factor_x;
	u8 bin_factor_y;
	u8 bin_mode;
};

struct hm2051_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

/* 2 bytes used for address: 256 bytes total */
#define HM2051_MAX_WRITE_BUF_SIZE	1024
struct hm2051_write_buffer {
	u16 addr;
	u8 data[HM2051_MAX_WRITE_BUF_SIZE];
};

struct hm2051_write_ctrl {
	int index;
	struct hm2051_write_buffer buffer;
};

static const struct i2c_device_id hm2051_id[] = {
	{HM2051_NAME, 0},
	{}
};

static struct hm2051_reg const hm2051_stream_on[] = {
	{HM2051_8BIT, 0x0000, 0x01},
	{HM2051_8BIT, 0x0100, 0x01},
	{HM2051_8BIT, 0x0101, 0x01},
	{HM2051_8BIT, 0x0005, 0x03},
	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_stream_off[] = {
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_TOK_TERM, 0, 0}
};

/*
 * initial settings
 */
#define hm2051_init hm2051_1616x1216_30fps
/*
static struct hm2051_reg const hm2051_init[] = {
    //---------------------------------------------------
    // Initial
    //---------------------------------------------------

	  {HM2051_TOK_TERM, 0, 0}
};
*/
/*
static struct hm2051_reg const hm2051_176x144_91fps[] = {
	{HM2051_8BIT, 0x0022, 0x00},
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_TOK_DELAY, 0, 100},
	{HM2051_8BIT, 0x0026, 0x08},
	{HM2051_8BIT, 0x002A, 0x52},
	{HM2051_8BIT, 0x0006, 0x04},
	{HM2051_8BIT, 0x000F, 0x00},
	{HM2051_8BIT, 0x000D, 0x01},
	{HM2051_8BIT, 0x000E, 0x01},
	{HM2051_8BIT, 0x0024, 0x40},
	{HM2051_8BIT, 0x0027, 0x23},
	{HM2051_8BIT, 0x0074, 0x13},
	{HM2051_8BIT, 0x002B, HM2051_CLK_DIV},
	{HM2051_8BIT, 0x002C, 0x06},
	{HM2051_8BIT, 0x0040, 0x00},
	{HM2051_8BIT, 0x0044, 0x03},
	{HM2051_8BIT, 0x0045, 0x63},
	{HM2051_8BIT, 0x0046, 0x5F},
	{HM2051_8BIT, 0x0049, 0xC0},
	{HM2051_8BIT, 0x004B, 0x03},
	{HM2051_8BIT, 0x0070, 0x2F},
	{HM2051_8BIT, 0x0072, 0xFB},
	{HM2051_8BIT, 0x0073, 0x77},
	{HM2051_8BIT, 0x0075, 0x40},
	{HM2051_8BIT, 0x0078, 0x65},
	{HM2051_8BIT, 0x0080, 0x98},
	{HM2051_8BIT, 0x0082, 0x09},
	{HM2051_8BIT, 0x0083, 0x3C},
	{HM2051_8BIT, 0x0087, 0x41},
	{HM2051_8BIT, 0x008D, 0x20},
	{HM2051_8BIT, 0x008E, 0x30},
	{HM2051_8BIT, 0x009D, 0x11},
	{HM2051_8BIT, 0x009E, 0x12},
	{HM2051_8BIT, 0x0090, 0x00},
	{HM2051_8BIT, 0x0091, 0x01},
	{HM2051_8BIT, 0x0092, 0x02},
	{HM2051_8BIT, 0x0093, 0x03},
	{HM2051_8BIT, 0x00C0, 0x64},
	{HM2051_8BIT, 0x00C1, 0x15},
	{HM2051_8BIT, 0x00C2, 0x00},
	{HM2051_8BIT, 0x00C3, 0x02},
	{HM2051_8BIT, 0x00C4, 0x0B},
	{HM2051_8BIT, 0x00C6, 0x83},
	{HM2051_8BIT, 0x00C7, 0x02},
	{HM2051_8BIT, 0x00CC, 0x00},
	{HM2051_8BIT, 0x4B3B, 0x12},
	{HM2051_8BIT, 0x4B41, 0x10},
	{HM2051_8BIT, 0x0165, 0x03},
	{HM2051_8BIT, 0x018C, 0x00},
	{HM2051_8BIT, 0x0195, 0x06},
	{HM2051_8BIT, 0x0196, 0x4F},
	{HM2051_8BIT, 0x0197, 0x04},
	{HM2051_8BIT, 0x0198, 0xBF},
	{HM2051_8BIT, 0x0144, 0x10},
	{HM2051_8BIT, 0x0140, 0x20},
	{HM2051_8BIT, 0x015A, 0x30},
	{HM2051_8BIT, 0x015D, 0xFF},
	{HM2051_8BIT, 0x0160, 0x21},
	{HM2051_8BIT, 0x0123, 0xD5},
	{HM2051_8BIT, 0x0660, 0x01},
	{HM2051_8BIT, 0x0661, 0x3c},
	{HM2051_8BIT, 0x0662, 0x01},
	{HM2051_8BIT, 0x0663, 0xEB},
	{HM2051_8BIT, 0x0664, 0x00},
	{HM2051_8BIT, 0x0665, 0x70},
	{HM2051_8BIT, 0x0666, 0x00},
	{HM2051_8BIT, 0x0667, 0xFF},
	{HM2051_8BIT, 0x4B50, 0x01},
	{HM2051_8BIT, 0x4B51, 0x46},
	{HM2051_8BIT, 0x4B0A, 0x00},
	{HM2051_8BIT, 0x4B0B, 0xB0},
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x4B07, 0xBD},
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_TOK_TERM, 0, 0}
};
*/
static struct hm2051_reg const hm2051_1488x1216_30fps[] = {
	{HM2051_8BIT, 0x0005, 0x00},
	{HM2051_TOK_DELAY, 0, 100},
    /*---------------------------------------------------
     Initial
    ---------------------------------------------------*/
	{HM2051_8BIT, 0x0022, 0x00},/* RESET*/
    /* ---------------------------------------------------
     CMU update
     ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},
	{HM2051_8BIT, 0x0005, 0x02},/* power up*/
	{HM2051_TOK_DELAY, 0, 100},

	{HM2051_8BIT, 0x0026, 0x08},/* PLL1, mipi pll_pre_div*/
	{HM2051_8BIT, 0x002A, 0x52},/*PLL1, mipi pll_div (pclk=pktclk= 002A + 1)  0x52 */
    /*{HM2051_8BIT,0x0013,0xFF},  jack
    ---------------------------------------------------
     Digital function
    ---------------------------------------------------*/
	{HM2051_8BIT, 0x0006, 0x00},/* [1] hmirror, [0] vflip*/
	{HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode, [0] non fixed frame rate mode */
	{HM2051_8BIT, 0x0011, 0x3A},/* ASUS_BSP ZZ++ Add for 0x200*/
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x13},/* disable output black rows*/
    /*---------------------------------------------------
     Analog
    ---------------------------------------------------*/
	{HM2051_8BIT, 0x002B, HM2051_CLK_DIV}, /*  clk divider */
	{HM2051_8BIT, 0x002C, 0x06},/* PLL cfg: CP, LPF, use PLL clk*/

	{HM2051_8BIT, 0x0040, 0x00}, /* BLC  //0A->00*/
	{HM2051_8BIT, 0x0044, 0x03},/* enable BLC, enable BLC IIR*/
	{HM2051_8BIT, 0x0045, 0x63},/*CFPN cfg, 0x65=repeat 32 times, 0x55=repeat 16 times*/
	{HM2051_8BIT, 0x0046, 0x5F},/* CFPN cfg, enable IIR, weight=1/4 new, CFPN applied to BLC*/
	{HM2051_8BIT, 0x0049, 0xC0},/*improve BLC_hunting*/
	{HM2051_8BIT, 0x004B, 0x03},/*improve BLC_huntingv*/

	{HM2051_8BIT, 0x0070, 0x2F}, /* ADD  0923*/
	{HM2051_8BIT, 0x0072, 0xFB}, /*for CFPN*/
	{HM2051_8BIT, 0x0073, 0x77}, /*for 30 fps*/
	{HM2051_8BIT, 0x0075, 0x40},/*Negative CP is controlled by INT*/
	{HM2051_8BIT, 0x0078, 0x65}, /*for 30 fps */

	{HM2051_8BIT, 0x0080, 0x98}, /* fd_clamp_en_d=1, tg_boost_en_d=1*/
	{HM2051_8BIT, 0x0082, 0x09},/* fd_clamp_en_d=1, tg_boost_en_d=1*/
	{HM2051_8BIT, 0x0083, 0x3C},/*VRPP=avdd+1.36V, VRNP=-1V, w/ lowest pump clk freq.*/
	{HM2051_8BIT, 0x0087, 0x41}, /* disable dsun clamp first*/
	{HM2051_8BIT, 0x008D, 0x20},/* pix_disc_diff_en_d=1*/
	{HM2051_8BIT, 0x008E, 0x30},

	{HM2051_8BIT, 0x009D, 0x11},/*Nramp_rst1,2*/
	{HM2051_8BIT, 0x009E, 0x12},/* Nramp_rst3,4*/

	{HM2051_8BIT, 0x0090, 0x00},/*gain table*/
	{HM2051_8BIT, 0x0091, 0x01},/* gain table*/
	{HM2051_8BIT, 0x0092, 0x02},/* gain table*/
	{HM2051_8BIT, 0x0093, 0x03},/* gain table*/

	{HM2051_8BIT, 0x00C0, 0x64},/*col_ldo setting*/
	{HM2051_8BIT, 0x00C1, 0x15},/* pvdd_refsel=5h(for power noise), pvdd_lorefsel*/
	{HM2051_8BIT, 0x00C2, 0x00},/* ramp power consumption*/

	{HM2051_8BIT, 0x00C3, 0x02},/* comp1,2,3_bias_sel*/
	{HM2051_8BIT, 0x00C4, 0x0B},/* column ADC cfg*/
	{HM2051_8BIT, 0x00C6, 0x83}, /* sf_srcdr_shrt_en_right_d=1, sf_always_on_d=0(improve noise)*/
	{HM2051_8BIT, 0x00C7, 0x02}, /* sr_sel_sh_d (reduce CFPN @ high AVDD) ADD 0923*/
	{HM2051_8BIT, 0x00CC, 0x00},/* mipi skew[5:0]*/

	{HM2051_8BIT, 0x4B3B, 0x12}, /* MIPI analog setting*/
	{HM2051_8BIT, 0x4B41, 0x10},/* HS0_D=1, prevent enter test mode(clk lane=0)*/

     /*Star of BPC setting*/
	{HM2051_8BIT, 0x0165, 0x03},/*[1:0]0:24 1:32, 2:48, 3:80*/
	{HM2051_8BIT, 0x0183, 0xF0},/*ASUS_BSP ZZ++ Add for 0x200*/
	{HM2051_8BIT, 0x018C, 0x00},/*[7:6]Dark_raw_enable*/

	{HM2051_8BIT, 0x0195, 0x06},/*X_offset[2:0]*/
	{HM2051_8BIT, 0x0196, 0x4F},/*X_offset[7:0]*/
	{HM2051_8BIT, 0x0197, 0x04},/*Y_offset[2:0]*/
	{HM2051_8BIT, 0x0198, 0xBF},/*Y_offset[7:0]*/

	{HM2051_8BIT, 0x0144, 0x12},/*/BPC_HOT_TH[8],[1]Median Filter with current pixel*/
	{HM2051_8BIT, 0x0140, 0x20},/*BPC_HOT_TH[7:0]*/
	{HM2051_8BIT, 0x015A, 0x80},/*BPC_HOT_2*/
	{HM2051_8BIT, 0x015D, 0x20},/*BPC_HOT_3*/
	{HM2051_8BIT, 0x0160, 0x65},/*[0]hot_replace[1]cold_replace[3:2]Max1_Max2[4]correct_all[5]Dynamic[6]Static[7]no write back */

    /* ---------------------------------------------------
     mipi-tx settings
    ---------------------------------------------------
     window off (1616x1216)*/
	{HM2051_8BIT, 0x0123, 0xD5},/* [4] digital window on*/
	{HM2051_8BIT, 0x0660, 0x00},/*win x_st Hb*/
	{HM2051_8BIT, 0x0661, 0x40},/*win x_st Lb*/
	{HM2051_8BIT, 0x0662, 0x06},/*win x_ed Hb*/
	{HM2051_8BIT, 0x0663, 0x0F},/*win x_ed Lb*/
	{HM2051_8BIT, 0x0664, 0x00},/* win y_st Hb*/
	{HM2051_8BIT, 0x0665, 0x00},/* win y_st Lb*/
	{HM2051_8BIT, 0x0666, 0x04},/*win y_ed Hb*/
	{HM2051_8BIT, 0x0667, 0xBF},/* win y_ed Lb*/
	{HM2051_8BIT, 0x4B50, 0x08},/* pre_h Hb*/
	{HM2051_8BIT, 0x4B51, 0xEA},/* pre_h Lb*/
	{HM2051_8BIT, 0x4B0A, 0x05},/* HSIZE (1488)*/
	{HM2051_8BIT, 0x4B0B, 0xD0},
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x4B07, 0xBD},/* MARK1 width*/
	{HM2051_8BIT, 0x4B0E, 0x14},/*clk front porch*/

    /* ---------------------------------------------------/
     mipi-tx Timming settings
     ---------------------------------------------------*/

#ifdef FULL_SPEED_HM2051
	{HM2051_8BIT, 0x4B02, 0x04},
	{HM2051_8BIT, 0x4B03, 0x09},
	{HM2051_8BIT, 0x4B04, 0x05},
	{HM2051_8BIT, 0x4B05, 0x0C},
	{HM2051_8BIT, 0x4B06, 0x06},
	{HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/
	{HM2051_8BIT, 0x4B3F, 0x12},
	{HM2051_8BIT, 0x4B42, 0x05},
	{HM2051_8BIT, 0x4B43, 0x00},
#else
	{HM2051_8BIT, 0x4B02, 0x02},
	{HM2051_8BIT, 0x4B03, 0x07},
	{HM2051_8BIT, 0x4B04, 0x02},
	{HM2051_8BIT, 0x4B05, 0x0C},
	{HM2051_8BIT, 0x4B06, 0x04},
	{HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/
	{HM2051_8BIT, 0x4B3F, 0x0C},
	{HM2051_8BIT, 0x4B42, 0x02},
	{HM2051_8BIT, 0x4B43, 0x00},
#endif

    /* window on (default 1600x1200)*/
	{HM2051_8BIT, 0x4B20, 0x9E},
    /*{HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/


    /* ---------------------------------------------------
     CMU update
     ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},

    /*---------------------------------------------------
     Turn on rolling shutter
    ---------------------------------------------------*/
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_TOK_TERM, 0, 0}
   };

static struct hm2051_reg const hm2051_1096x736_48fps[] = {
    
    //---------------------------------------------------
    // Initial
    //---------------------------------------------------
    {HM2051_8BIT,0x0022,0x00},// RESET
    {HM2051_8BIT, 0x0000, 0x00},
    {HM2051_8BIT, 0x0100, 0x00},
    {HM2051_8BIT, 0x0101, 0x00},
    {HM2051_8BIT,0x0005,0x02},// power up
    {HM2051_TOK_DELAY,0,100},
    
    {HM2051_8BIT,0x0026,0x08},// PLL1, mipi pll_pre_div
    {HM2051_8BIT,0x002A,0x52},// PLL1, mipi pll_div (pclk=pktclk= 002A + 1)
    /*{HM2051_8BIT,0x0013,0xFF},  //jack*/
    
    //---------------------------------------------------
    // Digital function
    //---------------------------------------------------
    {HM2051_8BIT,0x0006,0x04},// [1] hmirror, [0] vflip
    {HM2051_8BIT,0x000F,0x00},// [1] fixed frame rate mode, [0] non fixed frame rate mode
    {HM2051_8BIT,0x0024,0x40},// mipi enable
    {HM2051_8BIT,0x0027,0x23},// OUTFMT, after BPC, [7] pre_vsync_en
    {HM2051_8BIT,0x0065,0x01},// Hscan_delay (default=1)
    {HM2051_8BIT,0x0074,0x1B},// disable output black rows
    //---------------------------------------------------
    // Analog
    //---------------------------------------------------   
    {HM2051_8BIT, 0x002B, HM2051_CLK_DIV}, /*  clk divider*/
    {HM2051_8BIT,0x002C,0x06},// PLL cfg: CP, LPF, use PLL clk
    
    {HM2051_8BIT,0x0040,0x00}, // BLC  //0A->00
    {HM2051_8BIT,0x0044,0x03},// enable BLC, enable BLC IIR
    {HM2051_8BIT,0x0045,0x63},// CFPN cfg, 0x65=repeat 32 times, 0x55=repeat 16 times
    {HM2051_8BIT,0x0046,0x5F},// CFPN cfg, enable IIR, weight=1/4 new, CFPN applied to BLC
    {HM2051_8BIT,0x0049,0xC0},//improve BLC_hunting
    {HM2051_8BIT,0x004B,0x03},//improve BLC_hunting
    
    {HM2051_8BIT, 0x0070, 0x2F}, /* ADD  0923 */
    {HM2051_8BIT, 0x0072, 0xFB}, /*for CFPN */
    {HM2051_8BIT,0x0073,0x77}, // for 30 fps    
    {HM2051_8BIT,0x0075,0x40},// Negative CP is controlled by INT
    {HM2051_8BIT,0x0078,0x65}, // for 30 fps 
    
    {HM2051_8BIT, 0x0080, 0x98}, /* fd_clamp_en_d=1, tg_boost_en_d=1*/
    {HM2051_8BIT,0x0082,0x09},// fd_clamp_en_d=1, tg_boost_en_d=1
    {HM2051_8BIT,0x0083,0x3C},// VRPP=avdd+1.36V, VRNP=-1V, w/ lowest pump clk freq.
    {HM2051_8BIT, 0x0087, 0x41}, /* disable dsun clamp first*/
    {HM2051_8BIT,0x008D,0x20},// pix_disc_diff_en_d=1
    {HM2051_8BIT,0x008E,0x30},//
    
    {HM2051_8BIT,0x009D,0x11},// Nramp_rst1,2
    {HM2051_8BIT,0x009E,0x12},// Nramp_rst3,4
    
    {HM2051_8BIT,0x0090,0x00},// gain table
    {HM2051_8BIT,0x0091,0x01},// gain table
    {HM2051_8BIT,0x0092,0x02},// gain table
    {HM2051_8BIT,0x0093,0x03},// gain table
    
     
    {HM2051_8BIT,0x00C0,0x64},// col_ldo setting
    {HM2051_8BIT,0x00C1,0x15},// pvdd_refsel=5h(for power noise), pvdd_lorefsel
    {HM2051_8BIT,0x00C2,0x00},// ramp power consumption
    
    {HM2051_8BIT,0x00C3,0x02},// comp1,2,3_bias_sel
    {HM2051_8BIT,0x00C4,0x0B},// column ADC cfg
    {HM2051_8BIT, 0x00C6, 0x83}, /* sf_srcdr_shrt_en_right_d=1, sf_always_on_d=0(improve noise)*/
    {HM2051_8BIT, 0x00C7, 0x02}, /* sr_sel_sh_d (reduce CFPN @ high AVDD) ADD 0923*/
    {HM2051_8BIT,0x00CC,0x00},// mipi skew[5:0]
    
    {HM2051_8BIT, 0x4B3B, 0x12}, /* MIPI analog setting*/
    {HM2051_8BIT,0x4B41,0x10},// HS0_D=1, prevent enter test mode(clk lane=0)
    
    //Star of BPC setting
    {HM2051_8BIT,0x0165,0x03},//[1:0]0:24 1:32, 2:48, 3:80
	{HM2051_8BIT, 0x0183, 0xF0},/*ASUS_BSP ZZ++ Add for 0x200*/
    {HM2051_8BIT,0x018C,0x00},//[7:6]Dark_raw_enable
    
    {HM2051_8BIT,0x0195,0x06},//X_offset[2:0]
    {HM2051_8BIT,0x0196,0x4F},//X_offset[7:0]
    {HM2051_8BIT,0x0197,0x04},//Y_offset[2:0]
    {HM2051_8BIT,0x0198,0xBF},//Y_offset[7:0]
    
    {HM2051_8BIT,0x0144,0x12},//BPC_HOT_TH[8],[1]Median Filter with current pixel
    {HM2051_8BIT,0x0140,0x20},//BPC_HOT_TH[7:0]
    {HM2051_8BIT,0x015A,0x80},//BPC_HOT_2
    {HM2051_8BIT,0x015D,0x20},//BPC_HOT_3
    {HM2051_8BIT,0x0160,0x65},//[0]hot_replace[1]cold_replace[3:2]Max1_Max2[4]correct_all[5]Dynamic[6]Static[7]no write back 
   
    
    
    // ---------------------------------------------------/
    // mipi-tx settings
    // ---------------------------------------------------
    // window off (1616x1216)
    //{HM2051_8BIT,0x0123,0xC5},// [4] digital window off
        
    {HM2051_8BIT,0x0123,0xD5},  
    {HM2051_8BIT,0x0660,0x01},  //win x_st Hb
    {HM2051_8BIT,0x0661,0x04},  //win x_st Lb
    {HM2051_8BIT,0x0662,0x05},  //win x_ed Hb
    {HM2051_8BIT,0x0663,0x4B},  //win x_ed Lb
    {HM2051_8BIT,0x0664,0x00},  //win y_st Hb
    {HM2051_8BIT,0x0665,0x2E},  //win y_st Lb
    {HM2051_8BIT,0x0666,0x03},  //win y_ed Hb
    {HM2051_8BIT,0x0667,0x0D},  //win y_ed Lb
    
    {HM2051_8BIT,0x4B50,0x00},// pre_h Hb
    {HM2051_8BIT,0x4B51,0xE0},// pre_h Lb  CD->AF  => EO -> C2   //keep E0 workaround
    {HM2051_8BIT,0x4B0A,0x04},
    {HM2051_8BIT,0x4B0B,0x48},
    
    
    
    // window on (default 1600x1200)
    {HM2051_8BIT,0x4B20,0x9E},//
    {HM2051_8BIT,0x4B07,0xBD},// MARK1 width
    
    
    // ---------------------------------------------------
    // CMU update
    // ---------------------------------------------------
    {HM2051_8BIT,0x0000,0x00},//
    {HM2051_8BIT,0x0100,0x00},//
    {HM2051_8BIT,0x0101,0x00},//
    
    //---------------------------------------------------
    // Turn on rolling shutter
    //---------------------------------------------------
    {HM2051_8BIT, 0x0005, 0x02},
    {HM2051_8BIT,0x0025,0x00},//
    {HM2051_TOK_TERM, 0, 0}
};


static struct hm2051_reg const hm2051_1296x736_48fps[] = {
    
    //---------------------------------------------------
    // Initial
    //---------------------------------------------------
    {HM2051_8BIT,0x0022,0x00},// RESET
    {HM2051_8BIT, 0x0000, 0x00},
    {HM2051_8BIT, 0x0100, 0x00},
    {HM2051_8BIT, 0x0101, 0x00},
    {HM2051_8BIT,0x0005,0x02},// power up
    {HM2051_TOK_DELAY,0,100},
    
    {HM2051_8BIT,0x0026,0x08},// PLL1, mipi pll_pre_div
    {HM2051_8BIT,0x002A,0x52},// PLL1, mipi pll_div (pclk=pktclk= 002A + 1)
    /*{HM2051_8BIT,0x0013,0xFF},  //jack*/
    
    //---------------------------------------------------
    // Digital function
    //---------------------------------------------------
    {HM2051_8BIT,0x0006,0x04},// [1] hmirror, [0] vflip
    {HM2051_8BIT,0x000F,0x00},// [1] fixed frame rate mode, [0] non fixed frame rate mode    
    {HM2051_8BIT,0x0024,0x40},// mipi enable
    {HM2051_8BIT,0x0027,0x23},// OUTFMT, after BPC, [7] pre_vsync_en
    {HM2051_8BIT,0x0065,0x01},// Hscan_delay (default=1)
    {HM2051_8BIT,0x0074,0x1B},// disable output black rows
    //---------------------------------------------------
    // Analog
    //---------------------------------------------------
           
    
    {HM2051_8BIT, 0x002B, HM2051_CLK_DIV}, /*  clk divider*/
    {HM2051_8BIT,0x002C,0x06},// PLL cfg: CP, LPF, use PLL clk
    
    {HM2051_8BIT,0x0040,0x00}, // BLC  //0A->00
    {HM2051_8BIT,0x0044,0x03},// enable BLC, enable BLC IIR
    {HM2051_8BIT,0x0045,0x63},// CFPN cfg, 0x65=repeat 32 times, 0x55=repeat 16 times
    {HM2051_8BIT,0x0046,0x5F},// CFPN cfg, enable IIR, weight=1/4 new, CFPN applied to BLC
    {HM2051_8BIT,0x0049,0xC0},//improve BLC_hunting
    {HM2051_8BIT,0x004B,0x03},//improve BLC_hunting

    {HM2051_8BIT, 0x0070, 0x2F}, /*ADD  0923*/
    {HM2051_8BIT, 0x0072, 0xFB}, /*for CFPN*/
    {HM2051_8BIT, 0x0073, 0x77}, /* for 30 fps */
    {HM2051_8BIT,0x0075,0x40},// Negative CP is controlled by INT
    {HM2051_8BIT, 0x0078, 0x65}, /* for 30 fps*/
    
    {HM2051_8BIT, 0x0080, 0x98}, /* fd_clamp_en_d=1, tg_boost_en_d=1*/
    {HM2051_8BIT,0x0082,0x09},// fd_clamp_en_d=1, tg_boost_en_d=1
    {HM2051_8BIT,0x0083,0x3C},// VRPP=avdd+1.36V, VRNP=-1V, w/ lowest pump clk freq.
    {HM2051_8BIT, 0x0087, 0x41}, /* disable dsun clamp first */
    {HM2051_8BIT,0x008D,0x20},// pix_disc_diff_en_d=1
    {HM2051_8BIT,0x008E,0x30},//
    
    {HM2051_8BIT,0x009D,0x11},// Nramp_rst1,2
    {HM2051_8BIT,0x009E,0x12},// Nramp_rst3,4
    
    {HM2051_8BIT,0x0090,0x00},// gain table
    {HM2051_8BIT,0x0091,0x01},// gain table
    {HM2051_8BIT,0x0092,0x02},// gain table
    {HM2051_8BIT,0x0093,0x03},// gain table
    
     
    {HM2051_8BIT,0x00C0,0x64},// col_ldo setting
    {HM2051_8BIT,0x00C1,0x15},// pvdd_refsel=5h(for power noise), pvdd_lorefsel
    {HM2051_8BIT,0x00C2,0x00},// ramp power consumption
    
    {HM2051_8BIT,0x00C3,0x02},// comp1,2,3_bias_sel
    {HM2051_8BIT,0x00C4,0x0B},// column ADC cfg
    {HM2051_8BIT, 0x00C6, 0x83}, /* sf_srcdr_shrt_en_right_d=1, sf_always_on_d=0(improve noise)*/
    {HM2051_8BIT, 0x00C7, 0x02}, /* sr_sel_sh_d (reduce CFPN @ high AVDD) ADD 0923*/
    {HM2051_8BIT,0x00CC,0x00},// mipi skew[5:0]
    
    {HM2051_8BIT, 0x4B3B, 0x12}, /* MIPI analog setting*/
    {HM2051_8BIT,0x4B41,0x10},// HS0_D=1, prevent enter test mode(clk lane=0)
    
    //Star of BPC setting
    {HM2051_8BIT,0x0165,0x03},//[1:0]0:24 1:32, 2:48, 3:80
	{HM2051_8BIT, 0x0183, 0xF0},/*ASUS_BSP ZZ++ Add for 0x200*/
    {HM2051_8BIT,0x018C,0x00},//[7:6]Dark_raw_enable
    
    {HM2051_8BIT,0x0195,0x06},//X_offset[2:0]
    {HM2051_8BIT,0x0196,0x4F},//X_offset[7:0]
    {HM2051_8BIT,0x0197,0x04},//Y_offset[2:0]
    {HM2051_8BIT,0x0198,0xBF},//Y_offset[7:0]
    
    {HM2051_8BIT,0x0144,0x12},//BPC_HOT_TH[8],[1]Median Filter with current pixel
    {HM2051_8BIT,0x0140,0x20},//BPC_HOT_TH[7:0]
    {HM2051_8BIT,0x015A,0x80},//BPC_HOT_2
    {HM2051_8BIT,0x015D,0x20},//BPC_HOT_3
    {HM2051_8BIT,0x0160,0x65},//[0]hot_replace[1]cold_replace[3:2]Max1_Max2[4]correct_all[5]Dynamic[6]Static[7]no write back 
    
    
    // ---------------------------------------------------/
    // mipi-tx settings
    // ---------------------------------------------------
    // window off (1616x1216)
    //{HM2051_8BIT,0x0123,0xC5},// [4] digital window off
    
    

    {HM2051_8BIT,0x0123,0xD5},  
    {HM2051_8BIT,0x0660,0x00},  //win x_st Hb
    {HM2051_8BIT,0x0661,0xA0},  //win x_st Lb
    {HM2051_8BIT,0x0662,0x05},  //win x_ed Hb
    {HM2051_8BIT,0x0663,0xAF},  //win x_ed Lb
    {HM2051_8BIT,0x0664,0x00},  //win y_st Hb
    {HM2051_8BIT,0x0665,0x2E},  //win y_st Lb
    {HM2051_8BIT,0x0666,0x03},  //win y_ed Hb
    {HM2051_8BIT,0x0667,0x0D},  //win y_ed Lb
    
    {HM2051_8BIT,0x4B50,0x00},// pre_h Hb
    {HM2051_8BIT,0x4B51,0xAF},// pre_h Lb  CD->AF
    {HM2051_8BIT,0x4B0A,0x05},
    {HM2051_8BIT,0x4B0B,0x10},
    
    
    
    // window on (default 1600x1200)
    {HM2051_8BIT,0x4B20,0x9E},//
    {HM2051_8BIT,0x4B07,0xBD},// MARK1 width
    
    
    // ---------------------------------------------------
    // CMU update
    // ---------------------------------------------------
    {HM2051_8BIT,0x0000,0x00},//
    {HM2051_8BIT,0x0100,0x00},//
    {HM2051_8BIT,0x0101,0x00},//
    
    //---------------------------------------------------
    // Turn on rolling shutter
    //---------------------------------------------------
    {HM2051_8BIT, 0x0005, 0x02},
    {HM2051_8BIT,0x0025,0x00},//
    {HM2051_TOK_TERM, 0, 0}
};


static struct hm2051_reg const hm2051_1616x916_30fps[] = {
    
//---------------------------------------------------
// Initial
//---------------------------------------------------
    {HM2051_8BIT,0x0022,0x00},// RESET
    {HM2051_8BIT, 0x0000, 0x00},
    {HM2051_8BIT, 0x0100, 0x00},
    {HM2051_8BIT, 0x0101, 0x00},
    {HM2051_8BIT,0x0005,0x02},// power up
    {HM2051_TOK_DELAY,0,100},

    {HM2051_8BIT,0x0026,0x08},// PLL1, mipi pll_pre_div
    {HM2051_8BIT, 0x002A, 0x52},/* PLL1, mipi pll_div (pclk=pktclk= 002A + 1)*/
    /*{HM2051_8BIT,0x0013,0xFF},  //jack*/

    //---------------------------------------------------
    // Digital function
    //---------------------------------------------------
    {HM2051_8BIT,0x0006,0x00},// [1] hmirror, [0] vflip
    {HM2051_8BIT, 0x000F, 0x00}, /* [1] fixed frame rate mode, [0] non fixed frame rate mode */
	{HM2051_8BIT, 0x0011, 0x3A},/*ASUS_BSP ZZ++ Add for CIS0x200 original=0x02 */
    {HM2051_8BIT,0x0024,0x40},// mipi enable
    {HM2051_8BIT,0x0027,0x23},// OUTFMT, after BPC, [7] pre_vsync_en
    {HM2051_8BIT,0x0065,0x01},// Hscan_delay (default=1)
    {HM2051_8BIT,0x0074,0x13},// disable output black rows

    //---------------------------------------------------
    // Analog
    //---------------------------------------------------
       

    {HM2051_8BIT, 0x002B, HM2051_CLK_DIV}, /*  clk divider*/
    {HM2051_8BIT,0x002C,0x06},// PLL cfg: CP, LPF, use PLL clk
    
    {HM2051_8BIT,0x0040,0x00}, // BLC  //0A->00
    {HM2051_8BIT,0x0044,0x03},// enable BLC, enable BLC IIR
    {HM2051_8BIT,0x0045,0x63},// CFPN cfg, 0x65=repeat 32 times, 0x55=repeat 16 times
    {HM2051_8BIT,0x0046,0x5F},// CFPN cfg, enable IIR, weight=1/4 new, CFPN applied to BLC
    {HM2051_8BIT,0x0049,0xC0},//improve BLC_hunting
    {HM2051_8BIT,0x004B,0x03},//improve BLC_hunting
    
    {HM2051_8BIT, 0x0070, 0x2F}, /*ADD  0923*/
    {HM2051_8BIT, 0x0072, 0xFB}, /*for CFPN*/
    {HM2051_8BIT, 0x0073, 0x77}, /* for 30 fps*/
    {HM2051_8BIT,0x0075,0x40},// Negative CP is controlled by INT
    {HM2051_8BIT,0x0078,0x65}, // for 30 fps 
    
    {HM2051_8BIT, 0x0080, 0x98}, /* fd_clamp_en_d=1, tg_boost_en_d=1*/
    {HM2051_8BIT,0x0082,0x09},// fd_clamp_en_d=1, tg_boost_en_d=1
    {HM2051_8BIT,0x0083,0x3C},// VRPP=avdd+1.36V, VRNP=-1V, w/ lowest pump clk freq.
    {HM2051_8BIT, 0x0087, 0x41}, /* disable dsun clamp first */
    {HM2051_8BIT,0x008D,0x20},// pix_disc_diff_en_d=1
    {HM2051_8BIT,0x008E,0x30},//
    
    {HM2051_8BIT,0x009D,0x11},// Nramp_rst1,2
    {HM2051_8BIT,0x009E,0x12},// Nramp_rst3,4
    
    {HM2051_8BIT,0x0090,0x00},// gain table
    {HM2051_8BIT,0x0091,0x01},// gain table
    {HM2051_8BIT,0x0092,0x02},// gain table
    {HM2051_8BIT,0x0093,0x03},// gain table
    
     
    {HM2051_8BIT,0x00C0,0x64},// col_ldo setting
    {HM2051_8BIT,0x00C1,0x15},// pvdd_refsel=5h(for power noise), pvdd_lorefsel
    {HM2051_8BIT,0x00C2,0x00},// ramp power consumption
    
    {HM2051_8BIT,0x00C3,0x02},// comp1,2,3_bias_sel
    {HM2051_8BIT,0x00C4,0x0B},// column ADC cfg
    {HM2051_8BIT, 0x00C6, 0x83}, /* sf_srcdr_shrt_en_right_d=1, sf_always_on_d=0(improve noise)*/
    {HM2051_8BIT, 0x00C7, 0x02}, /* sr_sel_sh_d (reduce CFPN @ high AVDD) ADD 0923*/
    {HM2051_8BIT,0x00CC,0x00},// mipi skew[5:0]
    
    {HM2051_8BIT, 0x4B3B, 0x12}, /* MIPI analog setting*/
    {HM2051_8BIT,0x4B41,0x10},// HS0_D=1, prevent enter test mode(clk lane=0)
    
    //Star of BPC setting
    {HM2051_8BIT,0x0165,0x03},//[1:0]0:24 1:32, 2:48, 3:80
	{HM2051_8BIT, 0x0183, 0xF0},/*ASUS_BSP ZZ++ Add for 0x200*/
    {HM2051_8BIT,0x018C,0x00},//[7:6]Dark_raw_enable
    
    {HM2051_8BIT,0x0195,0x06},//X_offset[2:0]
    {HM2051_8BIT,0x0196,0x4F},//X_offset[7:0]
    {HM2051_8BIT,0x0197,0x04},//Y_offset[2:0]
    {HM2051_8BIT,0x0198,0xBF},//Y_offset[7:0]
    
    {HM2051_8BIT, 0x0144, 0x10},/*BPC_HOT_TH[8],[1]Median Filter with current pixel*/
    {HM2051_8BIT, 0x0140, 0x20},/*BPC_HOT_TH[7:0]*/
    {HM2051_8BIT, 0x015A, 0x30},/*BPC_HOT_2*/
    {HM2051_8BIT, 0x015D, 0xFF},/*BPC_HOT_3*/
    {HM2051_8BIT,0x0160,0x65},//[0]hot_replace[1]cold_replace[3:2]Max1_Max2[4]correct_all[5]Dynamic[6]Static[7]no write back 
    
    
    // ---------------------------------------------------/
    // mipi-tx settings
    // ---------------------------------------------------
    // window off (1616x1216)
    //{HM2051_8BIT,0x0123,0xC5},// [4] digital window off


    {HM2051_8BIT,0x0123,0xD5},  
    {HM2051_8BIT,0x0660,0x00},  //win x_st Hb
    {HM2051_8BIT,0x0661,0x00},  //win x_st Lb
    {HM2051_8BIT,0x0662,0x06},  //win x_ed Hb
    {HM2051_8BIT,0x0663,0x4F},  //win x_ed Lb
    {HM2051_8BIT,0x0664,0x00},  //win y_st Hb
    {HM2051_8BIT,0x0665,0x96},  //win y_st Lb
    {HM2051_8BIT,0x0666,0x04},  //win y_ed Hb
    {HM2051_8BIT,0x0667,0x29},  //win y_ed Lb

    {HM2051_8BIT,0x4B50,0x08},// pre_h Hb 09->08
    {HM2051_8BIT, 0x4B51, 0xE2}, /* pre_h Lb 22->B2*/
    {HM2051_8BIT,0x4B0A,0x06},
    {HM2051_8BIT,0x4B0B,0x50},



    // window on (default 1600x1200)
    {HM2051_8BIT,0x4B20,0x9E},//
    {HM2051_8BIT,0x4B07,0xBD},// MARK1 width


    // ---------------------------------------------------
    // CMU update
    // ---------------------------------------------------
    {HM2051_8BIT,0x0000,0x00},//
    {HM2051_8BIT,0x0100,0x00},//
    {HM2051_8BIT,0x0101,0x00},//

    //---------------------------------------------------
    // Turn on rolling shutter
    //---------------------------------------------------
    {HM2051_8BIT, 0x0005, 0x02},
    {HM2051_8BIT,0x0025,0x00},//
    {HM2051_TOK_TERM, 0, 0}
};



static struct hm2051_reg const hm2051_1616x1216_30fps[] = {
    {HM2051_8BIT, 0x0005, 0x00},
    {HM2051_TOK_DELAY, 0, 100},
    //---------------------------------------------------
    // Initial
    //---------------------------------------------------
    {HM2051_8BIT,0x0022,0x00},// RESET
    /* ---------------------------------------------------
    // CMU update
    // ---------------------------------------------------*/
    {HM2051_8BIT, 0x0000, 0x00},
    {HM2051_8BIT, 0x0100, 0x00},
    {HM2051_8BIT, 0x0101, 0x00},
    {HM2051_8BIT,0x0005,0x02},// power up
    {HM2051_TOK_DELAY,0,100},

    {HM2051_8BIT,0x0026,0x08},// PLL1, mipi pll_pre_div
    {HM2051_8BIT, 0x002A, 0x52},/*PLL1, mipi pll_div (pclk=pktclk= 002A + 1)  0x52 */
    /*{HM2051_8BIT,0x0013,0xFF},  //jack*/
    //---------------------------------------------------
    // Digital function
    //---------------------------------------------------
    {HM2051_8BIT,0x0006,0x00},// [1] hmirror, [0] vflip
	{HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode, [0] non fixed frame rate mode */
	{HM2051_8BIT, 0x0011, 0x58},/* ASUS_BSP ZZ++ Add for 0x200*/
    {HM2051_8BIT,0x0024,0x40},// mipi enable
    {HM2051_8BIT,0x0027,0x23},// OUTFMT, after BPC, [7] pre_vsync_en
    {HM2051_8BIT,0x0065,0x01},// Hscan_delay (default=1)
    {HM2051_8BIT,0x0074,0x13},// disable output black rows
    //---------------------------------------------------
    // Analog
    //---------------------------------------------------
    {HM2051_8BIT, 0x002B, HM2051_CLK_DIV}, /*  clk divider */
    {HM2051_8BIT,0x002C,0x06},// PLL cfg: CP, LPF, use PLL clk
    
    {HM2051_8BIT,0x0040,0x00}, // BLC  //0A->00
    {HM2051_8BIT,0x0044,0x03},// enable BLC, enable BLC IIR
    {HM2051_8BIT,0x0045,0x63},// CFPN cfg, 0x65=repeat 32 times, 0x55=repeat 16 times
    {HM2051_8BIT,0x0046,0x5F},// CFPN cfg, enable IIR, weight=1/4 new, CFPN applied to BLC
    {HM2051_8BIT,0x0049,0xC0},//improve BLC_hunting
    {HM2051_8BIT,0x004B,0x03},//improve BLC_hunting
    
    {HM2051_8BIT, 0x0070, 0x2F}, /* ADD  0923*/
    {HM2051_8BIT, 0x0072, 0xFB}, /*for CFPN*/
    {HM2051_8BIT, 0x0073, 0x77}, /*for 30 fps*/
    {HM2051_8BIT,0x0075,0x40},// Negative CP is controlled by INT
    {HM2051_8BIT,0x0078,0x65}, // for 30 fps 
    
    {HM2051_8BIT, 0x0080, 0x98}, /* fd_clamp_en_d=1, tg_boost_en_d=1*/
    {HM2051_8BIT,0x0082,0x09},// fd_clamp_en_d=1, tg_boost_en_d=1
    {HM2051_8BIT,0x0083,0x3C},// VRPP=avdd+1.36V, VRNP=-1V, w/ lowest pump clk freq.
    {HM2051_8BIT, 0x0087, 0x41}, /* disable dsun clamp first*/
    {HM2051_8BIT,0x008D,0x20},// pix_disc_diff_en_d=1
    {HM2051_8BIT,0x008E,0x30},//
    
    {HM2051_8BIT,0x009D,0x11},// Nramp_rst1,2
    {HM2051_8BIT,0x009E,0x12},// Nramp_rst3,4
    
    {HM2051_8BIT,0x0090,0x00},// gain table
    {HM2051_8BIT,0x0091,0x01},// gain table
    {HM2051_8BIT,0x0092,0x02},// gain table
    {HM2051_8BIT,0x0093,0x03},// gain table
    
     
    {HM2051_8BIT,0x00C0,0x64},// col_ldo setting
    {HM2051_8BIT,0x00C1,0x15},// pvdd_refsel=5h(for power noise), pvdd_lorefsel
    {HM2051_8BIT,0x00C2,0x00},// ramp power consumption
    
    {HM2051_8BIT,0x00C3,0x02},// comp1,2,3_bias_sel
    {HM2051_8BIT,0x00C4,0x0B},// column ADC cfg
    {HM2051_8BIT, 0x00C6, 0x83}, /* sf_srcdr_shrt_en_right_d=1, sf_always_on_d=0(improve noise)*/
    {HM2051_8BIT, 0x00C7, 0x02}, /* sr_sel_sh_d (reduce CFPN @ high AVDD) ADD 0923*/
    {HM2051_8BIT,0x00CC,0x00},// mipi skew[5:0]
    
    {HM2051_8BIT, 0x4B3B, 0x12}, /* MIPI analog setting*/
    {HM2051_8BIT,0x4B41,0x10},// HS0_D=1, prevent enter test mode(clk lane=0)
    
    //Star of BPC setting
    {HM2051_8BIT,0x0165,0x03},//[1:0]0:24 1:32, 2:48, 3:80
	{HM2051_8BIT, 0x0183, 0xF0},/*ASUS_BSP ZZ++ Add for 0x200*/
    {HM2051_8BIT,0x018C,0x00},//[7:6]Dark_raw_enable
    
    {HM2051_8BIT,0x0195,0x06},//X_offset[2:0]
    {HM2051_8BIT,0x0196,0x4F},//X_offset[7:0]
    {HM2051_8BIT,0x0197,0x04},//Y_offset[2:0]
    {HM2051_8BIT,0x0198,0xBF},//Y_offset[7:0]
    
    {HM2051_8BIT, 0x0144, 0x10},/*BPC_HOT_TH[8],[1]Median Filter with current pixel*/
    {HM2051_8BIT, 0x0140, 0x20},/*BPC_HOT_TH[7:0]*/
    {HM2051_8BIT, 0x015A, 0x30},/*BPC_HOT_2*/
    {HM2051_8BIT, 0x015D, 0xFF},/*BPC_HOT_3*/
    {HM2051_8BIT,0x0160,0x65},//[0]hot_replace[1]cold_replace[3:2]Max1_Max2[4]correct_all[5]Dynamic[6]Static[7]no write back 
    
    

    // ---------------------------------------------------/
    // mipi-tx settings
    // ---------------------------------------------------
    // window off (1616x1216)
    {HM2051_8BIT,0x0123,0xC5},// [4] digital window off
    {HM2051_8BIT,0x4B50,0x08},// pre_h Hb 09->08
    {HM2051_8BIT, 0x4B51, 0xE2}, /* pre_h Lb 22->B2*/

    /* ---------------------------------------------------/
    // mipi-tx Timming settings
    // ---------------------------------------------------*/

#ifdef FULL_SPEED_HM2051
    {HM2051_8BIT, 0x4B02, 0x04},
    {HM2051_8BIT, 0x4B03, 0x09},
    {HM2051_8BIT, 0x4B04, 0x05},
    {HM2051_8BIT, 0x4B05, 0x0C},
    {HM2051_8BIT, 0x4B06, 0x06},
    {HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/
    {HM2051_8BIT, 0x4B3F, 0x12},
    {HM2051_8BIT, 0x4B42, 0x05},
    {HM2051_8BIT, 0x4B43, 0x00},
#else
    {HM2051_8BIT, 0x4B02, 0x02},
    {HM2051_8BIT, 0x4B03, 0x07},
    {HM2051_8BIT, 0x4B04, 0x02},
    {HM2051_8BIT, 0x4B05, 0x0C},
    {HM2051_8BIT, 0x4B06, 0x04},
    {HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/
    {HM2051_8BIT, 0x4B3F, 0x0C},
    {HM2051_8BIT, 0x4B42, 0x02},
    {HM2051_8BIT, 0x4B43, 0x00},
#endif


    // window on (default 1600x1200)
    {HM2051_8BIT,0x4B20,0x9E},//
    /*{HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/


    // ---------------------------------------------------
    // CMU update
    // ---------------------------------------------------
    {HM2051_8BIT,0x0000,0x00},//
    {HM2051_8BIT,0x0100,0x00},//
    {HM2051_8BIT,0x0101,0x00},//

    //---------------------------------------------------
    // Turn on rolling shutter
    //---------------------------------------------------
    {HM2051_8BIT, 0x0005, 0x02},
    {HM2051_8BIT,0x0025,0x00},//
    {HM2051_TOK_TERM, 0, 0}
};



static struct hm2051_reg const hm2051_808x608_57fps[] = {
    {HM2051_8BIT, 0x0005, 0x00},
    {HM2051_TOK_DELAY, 0, 100},
    /*---------------------------------------------------
    // Initial
    //---------------------------------------------------*/
    {HM2051_8BIT, 0x0022, 0x00},/* RESET*/
    /* ---------------------------------------------------
    // CMU update
    // ---------------------------------------------------*/
    {HM2051_8BIT, 0x0000, 0x00},
    {HM2051_8BIT, 0x0100, 0x00},
    {HM2051_8BIT, 0x0101, 0x00},
    {HM2051_8BIT, 0x0005, 0x02},/* power up*/
    {HM2051_TOK_DELAY, 0, 100},

    {HM2051_8BIT, 0x0026, 0x08},/* PLL1, mipi pll_pre_div*/
    {HM2051_8BIT, 0x002A, 0x5f},/*PLL1, mipi pll_div (pclk=pktclk= 002A + 1)  0x52 */
    /*{HM2051_8BIT,0x0013,0xFF}, /*jack*/
    /*---------------------------------------------------
    // Digital function
    //---------------------------------------------------*/
    {HM2051_8BIT, 0x0006, 0x00},/* [1] hmirror, [0] vflip*/
    {HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode, [0] non fixed frame rate mode*/
    {HM2051_8BIT, 0x000D, 0x01},
    {HM2051_8BIT, 0x000E, 0x01},
    {HM2051_8BIT, 0x0011, 0xC6},
    {HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
    {HM2051_8BIT, 0x0027, 0x23},/* OUTFMT, after BPC, [7] pre_vsync_en*/
    {HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
    {HM2051_8BIT, 0x0074, 0x13},/* disable output black rows*/
    /*---------------------------------------------------
    // Analog
    //---------------------------------------------------*/
    {HM2051_8BIT, 0x002B, HM2051_CLK_DIV}, /*  clk divider */
    {HM2051_8BIT, 0x002C, 0x06},/*PLL cfg: CP, LPF, use PLL clk*/

    {HM2051_8BIT, 0x0040, 0x00}, /*BLC  //0A->00*/
    {HM2051_8BIT, 0x0044, 0x03},/*enable BLC, enable BLC IIR*/
    {HM2051_8BIT, 0x0045, 0x63},/* CFPN cfg, 0x65=repeat 32 times, 0x55=repeat 16 times*/
    {HM2051_8BIT, 0x0046, 0x5F},/* CFPN cfg, enable IIR, weight=1/4 new, CFPN applied to BLC*/
    {HM2051_8BIT, 0x0049, 0xC0},/*improve BLC_hunting*/
    {HM2051_8BIT, 0x004B, 0x03},/*improve BLC_hunting*/

    {HM2051_8BIT, 0x0070, 0x2F}, /* ADD  0923*/
    {HM2051_8BIT, 0x0072, 0xFB}, /*for CFPN*/
    {HM2051_8BIT, 0x0073, 0x77}, /*for 30 fps*/
    {HM2051_8BIT, 0x0075, 0x40},/* Negative CP is controlled by INT*/
    {HM2051_8BIT, 0x0078, 0x65}, /* for 30 fps */

    {HM2051_8BIT, 0x0080, 0x98}, /* fd_clamp_en_d=1, tg_boost_en_d=1*/
    {HM2051_8BIT, 0x0082, 0x09},/* fd_clamp_en_d=1, tg_boost_en_d=1*/
    {HM2051_8BIT, 0x0083, 0x3C},/*VRPP=avdd+1.36V, VRNP=-1V, w/ lowest pump clk freq.*/
    {HM2051_8BIT, 0x0087, 0x41}, /* disable dsun clamp first*/
    {HM2051_8BIT, 0x008D, 0x20},/*pix_disc_diff_en_d=1*/
    {HM2051_8BIT, 0x008E, 0x30},

    {HM2051_8BIT, 0x009D, 0x11},/* Nramp_rst1,2*/
    {HM2051_8BIT, 0x009E, 0x12},/* Nramp_rst3,4*/

    {HM2051_8BIT, 0x0090, 0x00},/* gain table*/
    {HM2051_8BIT, 0x0091, 0x01},/* gain table*/
    {HM2051_8BIT, 0x0092, 0x02},/* gain table*/
    {HM2051_8BIT, 0x0093, 0x03},/* gain table*/


    {HM2051_8BIT, 0x00C0, 0x64},/* col_ldo setting*/
    {HM2051_8BIT, 0x00C1, 0x15},/* pvdd_refsel=5h(for power noise), pvdd_lorefsel*/
    {HM2051_8BIT, 0x00C2, 0x00},/* ramp power consumption*/

    {HM2051_8BIT, 0x00C3, 0x02},/* comp1,2,3_bias_sel*/
    {HM2051_8BIT, 0x00C4, 0x0B},/*column ADC cfg*/
    {HM2051_8BIT, 0x00C6, 0x83}, /* sf_srcdr_shrt_en_right_d=1, sf_always_on_d=0(improve noise)*/
    {HM2051_8BIT, 0x00C7, 0x02}, /* sr_sel_sh_d (reduce CFPN @ high AVDD) ADD 0923*/
    {HM2051_8BIT, 0x00CC, 0x00},/* mipi skew[5:0]*/

    {HM2051_8BIT, 0x4B3B, 0x12}, /* MIPI analog setting*/
    {HM2051_8BIT, 0x4B41, 0x10},/* HS0_D=1, prevent enter test mode(clk lane=0)*/

    /*Star of BPC setting*/
    {HM2051_8BIT, 0x0165, 0x03},/*[1:0]0:24 1:32, 2:48, 3:80*/
	{HM2051_8BIT, 0x0183, 0xF0},/*ASUS_BSP ZZ++ Add for 0x200*/
    {HM2051_8BIT, 0x018C, 0x00},/*[7:6]Dark_raw_enable*/

    {HM2051_8BIT, 0x0195, 0x06},/*X_offset[2:0]*/
    {HM2051_8BIT, 0x0196, 0x4F},/*X_offset[7:0]*/
    {HM2051_8BIT, 0x0197, 0x04},/*Y_offset[2:0]*/
    {HM2051_8BIT, 0x0198, 0xBF},/*Y_offset[7:0]*/

    {HM2051_8BIT, 0x0144, 0x12},/*BPC_HOT_TH[8],[1]Median Filter with current pixel*/
    {HM2051_8BIT, 0x0140, 0x20},/*BPC_HOT_TH[7:0]*/
    {HM2051_8BIT, 0x015A, 0x80},/*BPC_HOT_2*/
    {HM2051_8BIT, 0x015D, 0x20},/*BPC_HOT_3*/
    {HM2051_8BIT, 0x0160, 0x65},/*[0]hot_replace[1]cold_replace[3:2]Max1_Max2[4]correct_all[5]Dynamic[6]Static[7]no write back*/



    /* ---------------------------------------------------/
    // mipi-tx settings
    // ---------------------------------------------------
    // window off (1616x1216)*/
    {HM2051_8BIT, 0x0123, 0xC5},/*[4] digital window off*/
    {HM2051_8BIT, 0x4B50, 0x08},/* pre_h Hb 09->08*/
    {HM2051_8BIT, 0x4B51, 0xE2}, /* pre_h Lb 22->B2*/
    {HM2051_8BIT, 0x4B0A, 0x03},
    {HM2051_8BIT, 0x4B0B, 0x28},
    /* ---------------------------------------------------/
    // mipi-tx Timming settings
    // ---------------------------------------------------*/

#ifdef FULL_SPEED_HM2051
    {HM2051_8BIT, 0x4B02, 0x04},
    {HM2051_8BIT, 0x4B03, 0x09},
    {HM2051_8BIT, 0x4B04, 0x05},
    {HM2051_8BIT, 0x4B05, 0x0C},
    {HM2051_8BIT, 0x4B06, 0x06},
    {HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/
    {HM2051_8BIT, 0x4B3F, 0x12},
    {HM2051_8BIT, 0x4B42, 0x05},
    {HM2051_8BIT, 0x4B43, 0x00},
#else
    {HM2051_8BIT, 0x4B02, 0x02},
    {HM2051_8BIT, 0x4B03, 0x07},
    {HM2051_8BIT, 0x4B04, 0x02},
    {HM2051_8BIT, 0x4B05, 0x0C},
    {HM2051_8BIT, 0x4B06, 0x04},
    {HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/
    {HM2051_8BIT, 0x4B3F, 0x0C},
    {HM2051_8BIT, 0x4B42, 0x02},
    {HM2051_8BIT, 0x4B43, 0x00},
#endif


    /* window on (default 1600x1200)*/
    {HM2051_8BIT, 0x4B20, 0x9E},
    /*{HM2051_8BIT, 0x4B07, 0xBD}, /* MARK1 width*/


    /* ---------------------------------------------------
    // CMU update
    // ---------------------------------------------------*/
    {HM2051_8BIT, 0x0000, 0x00},
    {HM2051_8BIT, 0x0100, 0x00},
    {HM2051_8BIT, 0x0101, 0x00},

    /*---------------------------------------------------
    // Turn on rolling shutter
    //---------------------------------------------------*/
    {HM2051_8BIT, 0x0005, 0x02},
    {HM2051_8BIT, 0x0025, 0x00},
    {HM2051_TOK_TERM, 0, 0}
};




/*
 * Modes supported by the hm2051 driver.
 * Please, keep them in ascending order.
 */

static struct hm2051_res_struct hm2051_res_preview[] = {
/*	{
	.desc	= "THIS IS PREVIEW:hm2051_176x144_91fps",
	.width	= 176,
	.height	= 144,
	.fps	= 91,
	.used	= 0,
	.regs	= hm2051_176x144_91fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 175,
	.vertical_start = 0,
	.vertical_end = 143,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 418,
	.bin_factor_x = 2,
	.bin_factor_y = 2,
	.bin_mode = 1,
	},
*/
	{
	.desc	= "THIS IS PREVIEW:hm2051_1488x1216_30fps",
	.width	= 1488,
	.height	= 1216,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2051_1488x1216_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1487,
	.vertical_start = 0,
	.vertical_end = 1215,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 1324,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},

	{
	.desc	= "THIS IS PREVIEW:hm2051_1616x916_30fps",
	.width	= 1616,
	.height	= 916,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2051_1616x916_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1615,
	.vertical_start = 0,
	.vertical_end = 915,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 1324,/* ASUS_BSP ZZ++ Add for 0x200 original=1266*/
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},

	{
	.desc	= "THIS IS PREVIEW:hm2051_1616x1216_30fps",
	.width	= 1616,
	.height	= 1216,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2051_1616x1216_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1615,
	.vertical_start = 0,
	.vertical_end = 1215,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 1354,/* ASUS_BSP ZZ++ Add for 0x200 original=1266*/
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
};

#define N_RES_PREVIEW (ARRAY_SIZE(hm2051_res_preview))

static struct hm2051_res_struct hm2051_res_still[] = {
	{
	.desc	= "THIS IS STILL:hm2051_1616x916_30fps",
	.width	= 1616,
	.height	= 916,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2051_1616x916_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1615,
	.vertical_start = 0,
	.vertical_end = 915,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 1324,/* ASUS_BSP ZZ++ Add for 0x200 original=1266*/
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},

	{
	.desc	= "THIS IS STILL:hm2051_1616x1216_30fps",
	.width	= 1616,
	.height	= 1216,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2051_1616x1216_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1615,
	.vertical_start = 0,
	.vertical_end = 1215,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 1354,/* ASUS_BSP ZZ++ Add for 0x200 original=1266*/
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
};

#define N_RES_STILL (ARRAY_SIZE(hm2051_res_still))

static struct hm2051_res_struct hm2051_res_video[] = {
	{
	.desc	= "THIS IS VIDEO:hm2051_808x608_57fps",
	.width	= 808,
	.height	= 608,
	.pixel_clk = HM2051_PCLK,
	.fps	= 30,
	.used	= 0,
	.line_length_pck = 1913,
	.frame_length_lines = 658,
	.bin_factor_x = 2,
	.bin_factor_y = 2,
	.bin_mode = 1,
	.regs	= hm2051_808x608_57fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 807,
	.vertical_start = 0,
	.vertical_end = 607,
	},
	{
	.desc	= "THIS IS VIDEO:hm2051_1488x1216_30fps",
	.width	= 1488,
	.height	= 1216,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2051_1488x1216_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1487,
	.vertical_start = 0,
	.vertical_end = 1215,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 1324,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},

	{
	.desc	= "THIS IS VIDEO:hm2051_1616x916_30fps",
	.width	= 1616,
	.height	= 916,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2051_1616x916_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1615,
	.vertical_start = 0,
	.vertical_end = 915,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 1324,/* ASUS_BSP ZZ++ Add for 0x200 original=1266*/
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},

	{
	.desc	= "THIS IS VIDEO:hm2051_1616x1216_30fps",
	.width	= 1616,
	.height	= 1216,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2051_1616x1216_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1615,
	.vertical_start = 0,
	.vertical_end = 1215,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 2310,
	.frame_length_lines = 1354,/* ASUS_BSP ZZ++ Add for 0x200 original=1266*/
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	}
};

/*
static struct hm2051_res_struct hm2051_res_video[] = {
	{
	.desc	= "THIS IS VIDEO:hm2051_808x608_57fps",
	.width	= 808,
	.height	= 608,
	.pixel_clk = HM2051_PCLK,
	.fps	= 30,
	.used	= 0,
	.line_length_pck = 1913,
	.frame_length_lines = 658,
	.bin_factor_x = 2,
	.bin_factor_y = 2,
	.bin_mode = 1,
	.regs	= hm2051_808x608_57fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 807,
	.vertical_start = 0,
	.vertical_end = 607,
	},
	{
	.desc	= "THIS IS VIDEO:hm2051_1296x736_48fps",
	.width	= 1296,
	.height	= 736,
	.fps	= 48,
	.used	= 0,
	.regs	= hm2051_1296x736_48fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1295,
	.vertical_start = 0,
	.vertical_end = 735,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 1913,
	.frame_length_lines = 786,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
	{
	.desc	= "THIS IS VIDEO:hm2051_1096x736_48fps",
	.width	= 1096,
	.height	= 736,
	.fps	= 48,
	.used	= 0,
	.regs	= hm2051_1096x736_48fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1095,
	.vertical_start = 0,
	.vertical_end = 735,
	.pixel_clk = HM2051_PCLK,
	.line_length_pck = 1913,
	.frame_length_lines = 786,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
};
*/

#define N_RES_VIDEO (ARRAY_SIZE(hm2051_res_video))
#endif
