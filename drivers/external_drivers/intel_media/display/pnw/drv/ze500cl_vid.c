/*
 * Copyright (c)  2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "displays/ze500cl_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
#include "psb_fb.h"
#include "psb_drv.h"
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/unistd.h>

#define OTM1284A_PANEL_NAME	"OTM8018B"
#define OTM1284A_DEBUG 1
#define SENDING_MCS 1
#define SLEEP_MODE_ENABLE 1
#define AUO_STANDBY_MODE_ENABLE 1

static bool splendid_init;
static bool panel_pwr_on;
static int cabc_status; /*(-1)=shutdown, 0=off, 1=on*/
static int backlight_val;
static int default_raw_brightness;
static int default_expected_brightness;
/*
 * GPIO pin definition
 */
#define LCD_BL_EN 0x7E           /*PMIC:BACKLIGHT_EN*/
#define LCD_ANALONG_PWR_EN 0xda    /*PMIC:VEMMC2CNT*/
#define LCD_ANALONG_PWR_EN_OFF 0x4
#define LCD_ANALONG_PWR_EN_AUTO 0x6
#define LCD_RST_N 57        /*GP_AON_057*/

#define PCB_ID3 112 /*GP_CORE_016 = 96 + 16 = 112*/
#define PCB_ID12 111 /*GP_CORE_015 = 96 + 15 = 111*/

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
#define PWM_ENABLE_GPIO 49        /*LED_BL_PWM*/
#define PWM_BASE_UNIT 0x1555 /*25,000Hz*/
/*ASUS jacob add for double check backlight value*/
#define BRIGHTNESS_MIN_LEVEL 0
#define BRIGHTNESS_INIT_LEVEL	61
#define BRIGHTNESS_MAX_LEVEL 255
#define MAX_BRIGHTNESS_NIT 400
/*ASUS jacob add for double check backlight value*/

/* ZE500CL001S */
#include <linux/PMUtil.h>
/* ZE500CL001E */

extern int Read_HW_ID(void);
extern int Read_TP_ID(void);
extern u32 lcd_unique_id;
extern unsigned int raw_luminance;
extern unsigned int cali_luminance;
static int board_hw_id;
static int board_tp_id;
static void __iomem *pwmctrl_mmio;
static int panel_reset_gpio;
static int auo_standby_mode;
extern bool dpms_bl_flag;

static struct panel_list supp_panels[] = {
	{"NO_PANEL", NO_PANEL},
	{"AUO_720P_VID", AUO_VID_PANEL},
	{"TM_720P_VID", TM_VID_PANEL},
	{"CPT_720P_VID", CPT_VID_PANEL},
};

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

struct mipi_dsi_cmd{
	int gamma_enable;
	/*int delay;*/
	int long_cmd;
	u32 len1;
	u8 *commands1;
	u32 len2;
	u8 *commands2;
};

struct panel_config{
	int width;
	int height;
	int hfp;
	int hpulse;
	int hbp;
	int vfp;
	int vpulse;
	int vbp;
};

static struct mdfld_dsi_config *otm1284a_dsi_config;

#define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))

/*initial code start*/

static u8 otm1284a_cm1[] = {0x00, 0x00};
static u8 otm1284a_cm2[] = {0xFF, 0x12, 0x84, 0x01};/*EXTC = 1*/
static u8 otm1284a_cm3[] = {0x00, 0x80};
static u8 otm1284a_cm4[] = {0xFF, 0x12, 0x84};/*Orise_mode_enable*/
static u8 otm1284a_cm5[] = {0x00, 0x92};
static u8 otm1284a_cm6[] = {0xFF, 0x10, 0x02}; /*0x10 = 2-Lane 0x11 = 4-Lane*/
static u8 otm1284a_cm7[] = {0x00, 0x00};
static u8 otm1284a_cm8[] = {0xFF, 0xFF, 0xFF, 0xFF}; /*Orise_mode_disable*/

/*CPT_SR start*/
/*---------------mipi setting -----------*/
static u8 cpt_sr_cm1[] = {0x00, 0xB9};
static u8 cpt_sr_cm2[] = {0xB0, 0x11};

/*--------------PANEL setting -----------*/
static u8 cpt_sr_cm3[] = {0x00, 0x80};
static u8 cpt_sr_cm4[] = {0xC0, 0x00, 0x64, 0x00, 0x10, 0x10, 0x00, 0x64, 0x10, 0x10};

static u8 cpt_sr_cm5[] = {0x00, 0x90};
static u8 cpt_sr_cm6[] = {0xC0, 0x00, 0x5C, 0x00, 0x01, 0x00, 0x04};

static u8 cpt_sr_cm7[] = {0x00, 0xA4};
static u8 cpt_sr_cm8[] = {0xC0, 0x22};

static u8 cpt_sr_cm9[] = {0x00, 0xB3};
static u8 cpt_sr_cm10[] = {0xC0, 0x00, 0x55};

static u8 cpt_sr_cm11[] = {0x00, 0x81};
static u8 cpt_sr_cm12[] = {0xC1, 0x55};

static u8 cpt_sr_cm13[] = {0x00, 0x90};
static u8 cpt_sr_cm14[] = {0xC4, 0x49};

/*----------------POWER SETTING----------------*/
static u8 cpt_sr_cm15[] = {0x00, 0xA0};
static u8 cpt_sr_cm16[] = {0xC4, 0x05, 0x10, 0x06, 0x02, 0x05, 0x15, 0x10, 0x05, 0x10, 0x07, 0x02, 0x05, 0x15, 0x10};

static u8 cpt_sr_cm17[] = {0x00, 0xB0};
static u8 cpt_sr_cm18[] = {0xC4, 0x00, 0x00};

static u8 cpt_sr_cm19[] = {0x00, 0x91};
static u8 cpt_sr_cm20[] = {0xC5, 0x46, 0x42}; /*0xC590*/

static u8 cpt_sr_cm21[] = {0x00, 0x00};
static u8 cpt_sr_cm22[] = {0xD8, 0xB6, 0xB6};

static u8 cpt_sr_cm23[] = {0x00, 0x00};
static u8 cpt_sr_cm24[] = {0xD9, 0x89};

static u8 cpt_sr_cm25[] = {0x00, 0x80};
static u8 cpt_sr_cm26[] = {0xC4, 0x00};

static u8 cpt_sr_cm27[] = {0x00, 0x81};
static u8 cpt_sr_cm28[] = {0xC4, 0x84};

static u8 cpt_sr_cm29[] = {0x00, 0x00};
static u8 cpt_sr_cm30[] = {0x36, 0x00};

static u8 cpt_sr_cm31[] = {0x00, 0xB2};
static u8 cpt_sr_cm32[] = {0xC5, 0x00, 0x84};

static u8 cpt_sr_cm33[] = {0x00, 0xBB};
static u8 cpt_sr_cm34[] = {0xC5, 0x8A};

static u8 cpt_sr_cm35[] = {0x00, 0x82};
static u8 cpt_sr_cm36[] = {0xC4, 0x0A};

static u8 cpt_sr_cm37[] = {0x00, 0xC6};
static u8 cpt_sr_cm38[] = {0xB0, 0x03};

static u8 cpt_sr_cm39[] = {0x00, 0xC2};
static u8 cpt_sr_cm40[] = {0xF5, 0x40};

static u8 cpt_sr_cm41[] = {0x00, 0xC3};
static u8 cpt_sr_cm42[] = {0xF5, 0x85};

static u8 cpt_sr_cm43[] = {0x00, 0x87};
static u8 cpt_sr_cm44[] = {0xC4, 0x18};

/*-------------CONTROL SETTING---------------*/
static u8 cpt_sr_cm45[] = {0x00, 0x00};
static u8 cpt_sr_cm46[] = {0xD0, 0x40};

static u8 cpt_sr_cm47[] = {0x00, 0x00};
static u8 cpt_sr_cm48[] = {0xD1, 0x00, 0x00};

/*-----------------POWER IC-----------------*/
static u8 cpt_sr_cm49[] = {0x00, 0x90};
static u8 cpt_sr_cm50[] = {0xF5, 0x02, 0x11, 0x02, 0x15};

static u8 cpt_sr_cm51[] = {0x00, 0x90};
static u8 cpt_sr_cm52[] = {0xC5, 0x50};

static u8 cpt_sr_cm53[] = {0x00, 0x94};
static u8 cpt_sr_cm54[] = {0xC5, 0x66};

/*------------------VGLO1/O2 disable----------------*/
static u8 cpt_sr_cm55[] = {0x00, 0xB2};
static u8 cpt_sr_cm56[] = {0xF5, 0x00, 0x00};

static u8 cpt_sr_cm57[] = {0x00, 0xB4};
static u8 cpt_sr_cm58[] = {0xF5, 0x00, 0x00};

static u8 cpt_sr_cm59[] = {0x00, 0xB6};
static u8 cpt_sr_cm60[] = {0xF5, 0x00, 0x00};

static u8 cpt_sr_cm61[] = {0x00, 0xB8};
static u8 cpt_sr_cm62[] = {0xF5, 0x00, 0x00};

static u8 cpt_sr_cm63[] = {0x00, 0x94};
static u8 cpt_sr_cm64[] = {0xF5, 0x00, 0x00};

static u8 cpt_sr_cm65[] = {0x00, 0xD2};
static u8 cpt_sr_cm66[] = {0xF5, 0x06, 0x15};

static u8 cpt_sr_cm67[] = {0x00, 0xB4};
static u8 cpt_sr_cm68[] = {0xC5, 0xCC};

/*-----------------PANEL TIMING STATE-------------------*/
static u8 cpt_sr_cm69[] = {0x00, 0x80};
static u8 cpt_sr_cm70[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm71[] = {0x00, 0x90};
static u8 cpt_sr_cm72[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm73[] = {0x00, 0xA0};
static u8 cpt_sr_cm74[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm75[] = {0x00, 0xB0};
static u8 cpt_sr_cm76[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm77[] = {0x00, 0xC0};
static u8 cpt_sr_cm78[] = {0xCB, 0x05, 0x05, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm79[] = {0x00, 0xD0};
static u8 cpt_sr_cm80[] = {0xCB, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm81[] = {0x00, 0xE0};
static u8 cpt_sr_cm82[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0x05, 0x05};

static u8 cpt_sr_cm83[] = {0x00, 0xF0};
static u8 cpt_sr_cm84[] = {0xCB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/*--------------PANEL PAD MAPPING-------------*/
static u8 cpt_sr_cm85[] = {0x00, 0x80};
static u8 cpt_sr_cm86[] = {0xCC, 0x02, 0x0A, 0x0C, 0x0E, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm87[] = {0x00, 0x90};
static u8 cpt_sr_cm88[] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x2E, 0x2D, 0x06, 0x01, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm89[] = {0x00, 0xA0};
static u8 cpt_sr_cm90[] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2E, 0x2D, 0x05};

static u8 cpt_sr_cm91[] = {0x00, 0xB0};
static u8 cpt_sr_cm92[] = {0xCC, 0x05, 0x0F, 0x0D, 0x0B, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm93[] = {0x00, 0xC0};
static u8 cpt_sr_cm94[] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x2E, 0x01, 0x06, 0x10, 0x0E, 0x0C, 0x0A, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm95[] = {0x00, 0xD0};
static u8 cpt_sr_cm96[] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x2E, 0x02};

/*-----------PANEL TIMING SETTING---------------------*/
static u8 cpt_sr_cm97[] = {0x00, 0x80};
static u8 cpt_sr_cm98[] = {0xCE, 0x87, 0x03, 0x10, 0x86, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm99[] = {0x00, 0x90};
static u8 cpt_sr_cm100[] = {0xCE, 0x34, 0xff, 0x3E, 0x35, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm101[] = {0x00, 0xA0};
static u8 cpt_sr_cm102[] = {0xCE, 0x38, 0x03, 0x84, 0xF8, 0x89, 0x1C, 0x00, 0x38, 0x02, 0x84, 0xF9, 0x89, 0x1C, 0x00};

static u8 cpt_sr_cm103[] = {0x00, 0xB0};
static u8 cpt_sr_cm104[] = {0xCE, 0x38, 0x01, 0x84, 0xFA, 0x89, 0x1C, 0x00, 0x38, 0x00, 0x84, 0xFB, 0x89, 0x1C, 0x00};

static u8 cpt_sr_cm105[] = {0x00, 0xC0};
static u8 cpt_sr_cm106[] = {0xCE, 0x30, 0x00, 0x84, 0xFC, 0x89, 0x1C, 0x00, 0x30, 0x01, 0x84, 0xFD, 0x89, 0x1C, 0x00};

static u8 cpt_sr_cm107[] = {0x00, 0xD0};
static u8 cpt_sr_cm108[] = {0xCE, 0x30, 0x02, 0x84, 0xFE, 0x89, 0x1C, 0x00, 0x30, 0x03, 0x84, 0xFF, 0x89, 0x1C, 0x00};

static u8 cpt_sr_cm109[] = {0x00, 0x80};
static u8 cpt_sr_cm110[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm111[] = {0x00, 0x90};
static u8 cpt_sr_cm112[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm113[] = {0x00, 0xA0};
static u8 cpt_sr_cm114[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm115[] = {0x00, 0xB0};
static u8 cpt_sr_cm116[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm117[] = {0x00, 0xC0};
static u8 cpt_sr_cm118[] = {0xCF, 0x01, 0x01, 0x20, 0x20, 0x00, 0x00, 0x01, 0x81, 0x00, 0x00, 0x00};

static u8 cpt_sr_cm119[] = {0x00, 0xB5};
static u8 cpt_sr_cm120[] = {0xC5, 0x3F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF};

/*============= C0505DT Gamma code==============*/
static u8 cpt_sr_cm121[] = {0x00, 0x00};
static u8 cpt_sr_cm122[] = {0xE1, 0x00, 0x16, 0x23, 0x31, 0x40, 0x4E, 0x50, 0x7C, 0x6F, 0x8B, 0x75, 0x5D, 0x6D, 0x47, 0x45, 0x37, 0x28, 0x19, 0x0A, 0x03};

static u8 cpt_sr_cm123[] = {0x00, 0x00};
static u8 cpt_sr_cm124[] = {0xE2, 0x00, 0x16, 0x23, 0x31, 0x40, 0x4E, 0x50, 0x7C, 0x6F, 0x8B, 0x75, 0x5D, 0x6D, 0x47, 0x45, 0x37, 0x28, 0x19, 0x0A, 0x03};

static u8 cpt_sr_cm125[] = {0x00, 0xA4};
static u8 cpt_sr_cm126[] = {0xC1, 0xF0};
/*CPT_SR end*/

/*CPT_REF start*/
/*================= Panel Setting =================*/
static u8 cpt_cm1[] = {0x00, 0x90};
static u8 cpt_cm2[] = {0xC0, 0x00, 0x55, 0x00, 0x01, 0x00, 0x04};
static u8 cpt_cm3[] = {0x00, 0x81};
static u8 cpt_cm4[] = {0xC1, 0x55};
/*================= Power Setting =================*/
static u8 cpt_cm5[] = {0x00, 0xB0};
static u8 cpt_cm6[] = {0xC4, 0x00, 0x00};
static u8 cpt_cm7[] = {0x00, 0x91};
static u8 cpt_cm8[] = {0xC5, 0x46, 0x42};
/*================= Gamma 2.2 =================*/
static u8 cpt_cm9[] = {0x00, 0x00};
static u8 cpt_cm10[] = {0xE1, 0x00, 0x16, 0x23, 0x31, 0x40, 0x4E, 0x50, 0x7C, 0x6F, 0x8B, 0x75, 0x5D, 0x6D, 0x47,
	0x45, 0x37, 0x28, 0x19, 0x0A, 0x03};
static u8 cpt_cm11[] = {0x00, 0x00};
static u8 cpt_cm12[] = {0xE2, 0x00, 0x16, 0x23, 0x31, 0x40, 0x4E, 0x50, 0x7C, 0x6F, 0x8B, 0x75, 0x5D, 0x6D, 0x47,
	0x45, 0x37, 0x28, 0x19, 0x0A, 0x03};

static u8 cpt_cm13[] = {0x00, 0xA4};
static u8 cpt_cm14[] = {0xC1, 0xF0};
/*CPT_REF end*/


static char otp_cm1[] = {0x51, 0x64};
static char otp_cm2[] = {0x53, 0x2C};
#ifdef ASUS_FACTORY_BUILD
static char otp_cm3[] = {0x55, 0x0};
#else
static char otp_cm3[] = {0x55, 0x03}; /* CABC Moving mode */
#endif
/*initial code end*/
static char auo_cm1[] = {0xFE, 0x04};
static char auo_cm2[] = {0x3F, 0x04};
static char auo_cm3[] = {0x40, 0x02};
static char auo_cabc_cm1[] = {0x22, 0x00};
static char auo_cabc_cm2[] = {0x23, 0x11};
static char auo_cabc_cm3[] = {0x24, 0x22};
static char auo_cabc_cm4[] = {0x25, 0x33};
static char auo_cabc_cm5[] = {0x26, 0x44};
static char auo_cabc_cm6[] = {0x27, 0x5B};
static char auo_cabc_cm7[] = {0x15, 0xFE};
static char auo_cabc_cm8[] = {0x0A, 0x62};
static char auo_cm4[] = {0xFE, 0x00};

static char tm_cm1[] = {0x00, 0xB1};
static char tm_cm2[] = {0xC6, 0x05};

static struct mipi_dsi_cmd cpt_sr_cmd_table[] = {
	{0, 0, sizeof(otm1284a_cm5), otm1284a_cm5, sizeof(otm1284a_cm6), otm1284a_cm6},
	{0, 0, sizeof(cpt_sr_cm1), cpt_sr_cm1, sizeof(cpt_sr_cm2), cpt_sr_cm2},
	{0, 0, sizeof(cpt_sr_cm3), cpt_sr_cm3, sizeof(cpt_sr_cm4), cpt_sr_cm4},
	{0, 0, sizeof(cpt_sr_cm5), cpt_sr_cm5, sizeof(cpt_sr_cm6), cpt_sr_cm6},
	{0, 0, sizeof(cpt_sr_cm7), cpt_sr_cm7, sizeof(cpt_sr_cm8), cpt_sr_cm8},
	{0, 0, sizeof(cpt_sr_cm9), cpt_sr_cm9, sizeof(cpt_sr_cm10), cpt_sr_cm10},
	{0, 0, sizeof(cpt_sr_cm11), cpt_sr_cm11, sizeof(cpt_sr_cm12), cpt_sr_cm12},
	{0, 0, sizeof(cpt_sr_cm13), cpt_sr_cm13, sizeof(cpt_sr_cm14), cpt_sr_cm14},
	{0, 0, sizeof(cpt_sr_cm15), cpt_sr_cm15, sizeof(cpt_sr_cm16), cpt_sr_cm16},
	{0, 0, sizeof(cpt_sr_cm17), cpt_sr_cm17, sizeof(cpt_sr_cm18), cpt_sr_cm18},
	{0, 0, sizeof(cpt_sr_cm19), cpt_sr_cm19, sizeof(cpt_sr_cm20), cpt_sr_cm20},
	{0, 0, sizeof(cpt_sr_cm21), cpt_sr_cm21, sizeof(cpt_sr_cm22), cpt_sr_cm22},
	{0, 0, sizeof(cpt_sr_cm23), cpt_sr_cm23, sizeof(cpt_sr_cm24), cpt_sr_cm24},
	{0, 0, sizeof(cpt_sr_cm25), cpt_sr_cm25, sizeof(cpt_sr_cm26), cpt_sr_cm26},
	{0, 0, sizeof(cpt_sr_cm27), cpt_sr_cm27, sizeof(cpt_sr_cm28), cpt_sr_cm28},
	{0, 0, sizeof(cpt_sr_cm29), cpt_sr_cm29, sizeof(cpt_sr_cm30), cpt_sr_cm30},
	{0, 0, sizeof(cpt_sr_cm31), cpt_sr_cm31, sizeof(cpt_sr_cm32), cpt_sr_cm32},
	{0, 0, sizeof(cpt_sr_cm33), cpt_sr_cm33, sizeof(cpt_sr_cm34), cpt_sr_cm34},
	{0, 0, sizeof(cpt_sr_cm35), cpt_sr_cm35, sizeof(cpt_sr_cm36), cpt_sr_cm36},
	{0, 0, sizeof(cpt_sr_cm37), cpt_sr_cm37, sizeof(cpt_sr_cm38), cpt_sr_cm38},
	{0, 0, sizeof(cpt_sr_cm39), cpt_sr_cm39, sizeof(cpt_sr_cm40), cpt_sr_cm40},
	{0, 0, sizeof(cpt_sr_cm41), cpt_sr_cm41, sizeof(cpt_sr_cm42), cpt_sr_cm42},
	{0, 0, sizeof(cpt_sr_cm43), cpt_sr_cm43, sizeof(cpt_sr_cm44), cpt_sr_cm44},
	{0, 0, sizeof(cpt_sr_cm45), cpt_sr_cm45, sizeof(cpt_sr_cm46), cpt_sr_cm46},
	{0, 0, sizeof(cpt_sr_cm47), cpt_sr_cm47, sizeof(cpt_sr_cm48), cpt_sr_cm48},
	{0, 0, sizeof(cpt_sr_cm49), cpt_sr_cm49, sizeof(cpt_sr_cm50), cpt_sr_cm50},
	{0, 0, sizeof(cpt_sr_cm51), cpt_sr_cm51, sizeof(cpt_sr_cm52), cpt_sr_cm52},
	{0, 0, sizeof(cpt_sr_cm53), cpt_sr_cm53, sizeof(cpt_sr_cm54), cpt_sr_cm54},
	{0, 0, sizeof(cpt_sr_cm55), cpt_sr_cm55, sizeof(cpt_sr_cm56), cpt_sr_cm56},
	{0, 0, sizeof(cpt_sr_cm57), cpt_sr_cm57, sizeof(cpt_sr_cm58), cpt_sr_cm58},
	{0, 0, sizeof(cpt_sr_cm59), cpt_sr_cm59, sizeof(cpt_sr_cm60), cpt_sr_cm60},
	{0, 0, sizeof(cpt_sr_cm61), cpt_sr_cm61, sizeof(cpt_sr_cm62), cpt_sr_cm62},
	{0, 0, sizeof(cpt_sr_cm63), cpt_sr_cm63, sizeof(cpt_sr_cm64), cpt_sr_cm64},
	{0, 0, sizeof(cpt_sr_cm65), cpt_sr_cm65, sizeof(cpt_sr_cm66), cpt_sr_cm66},
	{0, 0, sizeof(cpt_sr_cm67), cpt_sr_cm67, sizeof(cpt_sr_cm68), cpt_sr_cm68},
	{0, 0, sizeof(cpt_sr_cm69), cpt_sr_cm69, sizeof(cpt_sr_cm70), cpt_sr_cm70},
	{0, 0, sizeof(cpt_sr_cm71), cpt_sr_cm71, sizeof(cpt_sr_cm72), cpt_sr_cm72},
	{0, 0, sizeof(cpt_sr_cm73), cpt_sr_cm73, sizeof(cpt_sr_cm74), cpt_sr_cm74},
	{0, 0, sizeof(cpt_sr_cm75), cpt_sr_cm75, sizeof(cpt_sr_cm76), cpt_sr_cm76},
	{0, 0, sizeof(cpt_sr_cm77), cpt_sr_cm77, sizeof(cpt_sr_cm78), cpt_sr_cm78},
	{0, 0, sizeof(cpt_sr_cm79), cpt_sr_cm79, sizeof(cpt_sr_cm80), cpt_sr_cm80},
	{0, 0, sizeof(cpt_sr_cm81), cpt_sr_cm81, sizeof(cpt_sr_cm82), cpt_sr_cm82},
	{0, 0, sizeof(cpt_sr_cm83), cpt_sr_cm83, sizeof(cpt_sr_cm84), cpt_sr_cm84},
	{0, 0, sizeof(cpt_sr_cm85), cpt_sr_cm85, sizeof(cpt_sr_cm86), cpt_sr_cm86},
	{0, 0, sizeof(cpt_sr_cm87), cpt_sr_cm87, sizeof(cpt_sr_cm88), cpt_sr_cm88},
	{0, 0, sizeof(cpt_sr_cm89), cpt_sr_cm89, sizeof(cpt_sr_cm90), cpt_sr_cm90},
	{0, 0, sizeof(cpt_sr_cm91), cpt_sr_cm91, sizeof(cpt_sr_cm92), cpt_sr_cm92},
	{0, 0, sizeof(cpt_sr_cm93), cpt_sr_cm93, sizeof(cpt_sr_cm94), cpt_sr_cm94},
	{0, 0, sizeof(cpt_sr_cm95), cpt_sr_cm95, sizeof(cpt_sr_cm96), cpt_sr_cm96},
	{0, 0, sizeof(cpt_sr_cm97), cpt_sr_cm97, sizeof(cpt_sr_cm98), cpt_sr_cm98},
	{0, 0, sizeof(cpt_sr_cm99), cpt_sr_cm99, sizeof(cpt_sr_cm100), cpt_sr_cm100},
	{0, 0, sizeof(cpt_sr_cm101), cpt_sr_cm101, sizeof(cpt_sr_cm102), cpt_sr_cm102},
	{0, 0, sizeof(cpt_sr_cm103), cpt_sr_cm103, sizeof(cpt_sr_cm104), cpt_sr_cm104},
	{0, 0, sizeof(cpt_sr_cm105), cpt_sr_cm105, sizeof(cpt_sr_cm106), cpt_sr_cm106},
	{0, 0, sizeof(cpt_sr_cm107), cpt_sr_cm107, sizeof(cpt_sr_cm108), cpt_sr_cm108},
	{0, 0, sizeof(cpt_sr_cm109), cpt_sr_cm109, sizeof(cpt_sr_cm110), cpt_sr_cm110},
	{0, 0, sizeof(cpt_sr_cm111), cpt_sr_cm111, sizeof(cpt_sr_cm112), cpt_sr_cm112},
	{0, 0, sizeof(cpt_sr_cm113), cpt_sr_cm113, sizeof(cpt_sr_cm114), cpt_sr_cm114},
	{0, 0, sizeof(cpt_sr_cm115), cpt_sr_cm115, sizeof(cpt_sr_cm116), cpt_sr_cm116},
	{0, 0, sizeof(cpt_sr_cm117), cpt_sr_cm117, sizeof(cpt_sr_cm118), cpt_sr_cm118},
	{0, 0, sizeof(cpt_sr_cm119), cpt_sr_cm119, sizeof(cpt_sr_cm120), cpt_sr_cm120},
	{0, 0, sizeof(cpt_sr_cm121), cpt_sr_cm121, sizeof(cpt_sr_cm122), cpt_sr_cm122},
	{0, 0, sizeof(cpt_sr_cm123), cpt_sr_cm123, sizeof(cpt_sr_cm124), cpt_sr_cm124},
	{0, 0, sizeof(cpt_sr_cm125), cpt_sr_cm125, sizeof(cpt_sr_cm126), cpt_sr_cm126},
};

static struct mipi_dsi_cmd cpt_cmd_table[] = {
	{0, 0, sizeof(otm1284a_cm5), otm1284a_cm5, sizeof(otm1284a_cm6), otm1284a_cm6},
	{0, 0, sizeof(cpt_cm1), cpt_cm1, sizeof(cpt_cm2), cpt_cm2},
	{0, 0, sizeof(cpt_cm3), cpt_cm3, sizeof(cpt_cm4), cpt_cm4},
	{0, 0, sizeof(cpt_cm5), cpt_cm5, sizeof(cpt_cm6), cpt_cm6},
	{0, 0, sizeof(cpt_cm7), cpt_cm7, sizeof(cpt_cm8), cpt_cm8},
	{1, 0, sizeof(cpt_cm9), cpt_cm9, sizeof(cpt_cm10), cpt_cm10},
	{1, 0, sizeof(cpt_cm11), cpt_cm11, sizeof(cpt_cm12), cpt_cm12},
	{0, 0, sizeof(cpt_cm13), cpt_cm13, sizeof(cpt_cm14), cpt_cm14},
};

static struct mipi_dsi_cmd tm_cmd_table[] = {
	{0, 1, sizeof(otm1284a_cm1), otm1284a_cm1, sizeof(otm1284a_cm2), otm1284a_cm2},
	{0, 1, sizeof(otm1284a_cm3), otm1284a_cm3, sizeof(otm1284a_cm4), otm1284a_cm4},
	{0, 0, sizeof(tm_cm1), tm_cm1, sizeof(tm_cm2), tm_cm2},
	{0, 0, sizeof(otp_cm1), otp_cm1, sizeof(otp_cm2), otp_cm2},
	{0, 0, sizeof(otp_cm3), otp_cm3, 0, 0},
};

static struct mipi_dsi_cmd auo_cmd_table[] = {
	{0, 0, sizeof(auo_cm1), auo_cm1, sizeof(auo_cm2), auo_cm2},
	{0, 0, sizeof(auo_cm3), auo_cm3, sizeof(auo_cabc_cm1), auo_cabc_cm1},
	{0, 0, sizeof(auo_cabc_cm2), auo_cabc_cm2, sizeof(auo_cabc_cm3), auo_cabc_cm3},
	{0, 0, sizeof(auo_cabc_cm4), auo_cabc_cm4, sizeof(auo_cabc_cm5), auo_cabc_cm5},
	{0, 0, sizeof(auo_cabc_cm6), auo_cabc_cm6, sizeof(auo_cabc_cm7), auo_cabc_cm7},
	{0, 0, sizeof(auo_cabc_cm8), auo_cabc_cm8, sizeof(auo_cm4), auo_cm4},
	{0, 0, sizeof(otp_cm1), otp_cm1, sizeof(otp_cm2), otp_cm2},
	{0, 0, sizeof(otp_cm3), otp_cm3, 0, 0},
};

static struct mipi_dsi_cmd otp_cmd_table[] = {
	{0, 0, sizeof(otp_cm1), otp_cm1, sizeof(otp_cm2), otp_cm2},
	{0, 0, sizeof(otp_cm3), otp_cm3, 0, 0},
};

static struct panel_config cpt_panel_conf = {720, 1280, 24, 2, 32, 16, 2, 14};
static struct panel_config tm_panel_conf = {720, 1280, 52, 10, 52, 20, 2, 26};
static struct panel_config auo_panel_conf = {720, 1280, 40, 10, 50, 16, 2, 18};
static struct panel_config auo_panel_conf_4lane = {720, 1280, 52, 10, 52, 20, 2, 26};

static struct panel_config otm1284a_panel_conf;
static struct mipi_dsi_cmd *otm1284a_cmd_table;
static int otm1284a_cmd_table_size;


static int pwm_configure(int duty)
{
	union sst_pwmctrl_reg pwmctrl;

	/*Read the PWM register to make sure there is no pending
	*update.
	*/
	pwmctrl.full = readl(pwmctrl_mmio);

	/*check pwnswupdate bit */
	if (pwmctrl.part.pwmswupdate)
		return -EBUSY;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = duty;
	writel(pwmctrl.full,  pwmctrl_mmio);

	return 0;
}


static void pwm_enable()
{
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable()
{
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full,  pwmctrl_mmio);

	gpio_set_value(PWM_ENABLE_GPIO, 0);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, 0);
}

static int otm1284a_vid_brightness_init(void)
{
	int ret = 0;

	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG, PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return ret;
}

/*normal*/
static int otm1284a_send_mipi_cmd(
				struct mdfld_dsi_pkg_sender *sender,
				u8 *data,
				u32 len)
{

	int r = 0, i;

	DRM_DEBUG("%s: reg=0x%2x, data=0x%2x, len=%d\n", __func__, data[0], data[1], len);

	switch (len) {
#if !SENDING_MCS
	case 1:
			r = mdfld_dsi_send_gen_short_lp(sender, data[0], 0, 1, 0);
			break;
	case 2:
			r = mdfld_dsi_send_gen_short_lp(sender, data[0], data[1], 2, 0);
			break;
#else
	case 1:
			r = mdfld_dsi_send_mcs_short_lp(sender, data[0], 0, 0, MDFLD_DSI_SEND_PACKAGE);
			break;
	case 2:
			r = mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, MDFLD_DSI_SEND_PACKAGE);
			break;
#endif
	default:
			r = mdfld_dsi_send_gen_long_lp(sender, data, len, 0);
			break;
	}
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n", __func__);
		return -EIO;
	} else {
		return 0;
	}
}

static int otm1284a_send_mipi_cmd_long2short(
				struct mdfld_dsi_pkg_sender *sender,
				u8 *data1, u8 *data2,
				u32 len, u32 len2)
{
	int ret = 0;
	u8 data3[20] = {0};
	int i, r = 0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	if (len2 <= 2) {
		otm1284a_send_mipi_cmd(sender, data1, len);
		if (len2 != 0)
			otm1284a_send_mipi_cmd(sender, data2, len2);
	} else {

#if !SENDING_MCS
		for (i = 0 ; i < (len2-1); i++) {
			r = mdfld_dsi_send_gen_short_lp(sender, 0x0, data1[1]+i, 2, 0);
			r = mdfld_dsi_send_gen_short_lp(sender, data2[0], data2[i+1], 2, 0);
		}
#else
		for (i = 0 ; i < (len2-1); i++) {
			r = mdfld_dsi_send_mcs_short_lp(sender, 0x0, data1[1]+i, 1, MDFLD_DSI_SEND_PACKAGE);
			r = mdfld_dsi_send_mcs_short_lp(sender, data2[0], data2[i+1], 1, MDFLD_DSI_SEND_PACKAGE);
		}
#endif
	}

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n", __func__);
		return -EIO;
	} else {
		return 0;
	}
}

static int otm1284a_send_mipi_cmd_long2short_gamma(
				struct mdfld_dsi_pkg_sender *sender,
				u8 *data1, u8 *data2,
				u32 len)
{

	int ret = 0;
	u8 data3[20] = {0};
	int i, r = 0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

#if !SENDING_MCS
	r = mdfld_dsi_send_gen_short_lp(sender, 0x0 , 0x0, 2, 0);

	for (i = 0 ; i < (len-1) ; i++) {
		r = mdfld_dsi_send_gen_short_lp(sender, data2[0], data2[i+1]  , 2, 0);
	}
#else
	r = mdfld_dsi_send_mcs_short_lp(sender, 0x0 , 0x0, 1, MDFLD_DSI_SEND_PACKAGE);

	for (i = 0 ; i < (len-1) ; i++) {
		r = mdfld_dsi_send_mcs_short_lp(sender, data2[0], data2[i+1]  , 1, MDFLD_DSI_SEND_PACKAGE);
	}
#endif

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n", __func__);
		return -EIO;
	} else {
		return 0;
	}
}

static int otm1284a_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	int i;
	int r = 0;
	u8 data[10] = {0};
	int retry_count;

	DRM_INFO("[DISPLAY] %s: Enter +++\n", __func__);

	retry_count = 5;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	mdfld_dsi_read_gen_lp(sender, 0x0A, 0x00, 1, data, 1); /*trigger MIPI lane to LP11 state*/

#if SLEEP_MODE_ENABLE
	if (!panel_pwr_on || auo_standby_mode) {
#endif

		if (gpio_direction_output(panel_reset_gpio, 1))
		    gpio_set_value_cansleep(panel_reset_gpio, 1);

		usleep_range(10000, 11000);

		if (gpio_direction_output(panel_reset_gpio, 0))
		    gpio_set_value_cansleep(panel_reset_gpio, 0);

		usleep_range(10000, 11000);

		if (gpio_direction_output(panel_reset_gpio, 1))
		    gpio_set_value_cansleep(panel_reset_gpio, 1);

		usleep_range(120000, 130000);

		/*Orise mode enable*/
		/*while (data[0] != 0x55 && retry_count > 0) {
		    otm1284a_send_mipi_cmd(sender, otm1284a_cm1, ARRAY_SIZE(otm1284a_cm1));
		    otm1284a_send_mipi_cmd(sender, otm1284a_cm2, ARRAY_SIZE(otm1284a_cm2));
		    otm1284a_send_mipi_cmd(sender, otm1284a_cm3, ARRAY_SIZE(otm1284a_cm3));
		    otm1284a_send_mipi_cmd(sender, otm1284a_cm4, ARRAY_SIZE(otm1284a_cm4));

		    mdfld_dsi_send_gen_short_lp(sender, 0x0 , 0x81, 2, 0);
		    r = mdfld_dsi_read_gen_lp(sender, 0xc1, 0, 1, data, 1);
			if (retry_count == 1 && data[0] != 0x55)
				printk("Orise mode does not enable: C181h=0x%02x\n", data[0]);
		    retry_count--;
		}*/

		if (asus_panel_id == CPT_VID_PANEL) {
			/*sending initial commands*/
			for (i = 0; i < otm1284a_cmd_table_size; i++) {
			    if (otm1284a_cmd_table[i].gamma_enable) {
					otm1284a_send_mipi_cmd_long2short_gamma(sender, otm1284a_cmd_table[i].commands1,
						otm1284a_cmd_table[i].commands2, otm1284a_cmd_table[i].len2);
				} else {
					otm1284a_send_mipi_cmd_long2short(sender, otm1284a_cmd_table[i].commands1,
						otm1284a_cmd_table[i].commands2, otm1284a_cmd_table[i].len1, otm1284a_cmd_table[i].len2);
				}
			}
			/*Orise mode disable*/
			otm1284a_send_mipi_cmd(sender, otm1284a_cm7, ARRAY_SIZE(otm1284a_cm7));
			otm1284a_send_mipi_cmd(sender, otm1284a_cm8, ARRAY_SIZE(otm1284a_cm8));
		} else {
			for (i = 0; i < (otm1284a_cmd_table_size - (panel_pwr_on ? 2 : 0)); i++) {
				if (otm1284a_cmd_table[i].long_cmd) {
					 otm1284a_send_mipi_cmd(sender, otm1284a_cmd_table[i].commands1, otm1284a_cmd_table[i].len1);
					 otm1284a_send_mipi_cmd(sender, otm1284a_cmd_table[i].commands2, otm1284a_cmd_table[i].len2);
				} else if (otm1284a_cmd_table[i].gamma_enable) {
					otm1284a_send_mipi_cmd_long2short_gamma(sender, otm1284a_cmd_table[i].commands1,
						otm1284a_cmd_table[i].commands2, otm1284a_cmd_table[i].len2);
				} else {
					otm1284a_send_mipi_cmd_long2short(sender, otm1284a_cmd_table[i].commands1,
						otm1284a_cmd_table[i].commands2, otm1284a_cmd_table[i].len1, otm1284a_cmd_table[i].len2);
				}
			}

			if (panel_pwr_on) {
				mdfld_dsi_send_mcs_short_lp(sender, 0x51, backlight_val, 1, 0);
				mdfld_dsi_send_mcs_short_lp(sender, 0x53, 0x2C, 1, 0);
				mdfld_dsi_send_mcs_short_lp(sender, 0x55, (cabc_status > 0 ? 0x3 : 0x0), 1, 0);
			} else {
				if (otp_cm3[1] == 0x0)
					cabc_status = -1;
				else
					cabc_status = 1;
			}
		}

#if SLEEP_MODE_ENABLE
	}
#endif

	DRM_INFO("[DISPLAY] %s: End ---\n", __func__);

	return 0;
}

static void
otm1284a_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	struct drm_device *dev = dsi_config->dev;

	struct csc_setting csc = {	.pipe = 0,
								.type = CSC_REG_SETTING,
								.enable_state = true,
								.data_len = CSC_REG_COUNT,
								.data.csc_reg_data = {
									0x400, 0x0, 0x4000000, 0x0, 0x0, 0x400}
							 };
	struct gamma_setting gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x020202, 0x040404, 0x060606,
										0x080808, 0x0A0A0A, 0x0C0C0C, 0x0E0E0E,
										0x101010, 0x121212, 0x141414, 0x161616,
										0x181818, 0x1A1A1A, 0x1C1C1C, 0x1E1E1E,
										0x202020, 0x222222, 0x242424, 0x262626,
										0x282828, 0x2A2A2A, 0x2C2C2C, 0x2E2E2E,
										0x303030, 0x323232, 0x343434, 0x363636,
										0x383838, 0x3A3A3A, 0x3C3C3C, 0x3E3E3E,
										0x404040, 0x424242, 0x444444, 0x464646,
										0x484848, 0x4A4A4A, 0x4C4C4C, 0x4E4E4E,
										0x505050, 0x525252, 0x545454, 0x565656,
										0x585858, 0x5A5A5A, 0x5C5C5C, 0x5E5E5E,
										0x606060, 0x626262, 0x646464, 0x666666,
										0x686868, 0x6A6A6A, 0x6C6C6C, 0x6E6E6E,
										0x707070, 0x727272, 0x747474, 0x767676,
										0x787878, 0x7A7A7A, 0x7C7C7C, 0x7E7E7E,
										0x808080, 0x828282, 0x848484, 0x868686,
										0x888888, 0x8A8A8A, 0x8C8C8C, 0x8E8E8E,
										0x909090, 0x929292, 0x949494, 0x969696,
										0x989898, 0x9A9A9A, 0x9C9C9C, 0x9E9E9E,
										0xA0A0A0, 0xA2A2A2, 0xA4A4A4, 0xA6A6A6,
										0xA8A8A8, 0xAAAAAA, 0xACACAC, 0xAEAEAE,
										0xB0B0B0, 0xB2B2B2, 0xB4B4B4, 0xB6B6B6,
										0xB8B8B8, 0xBABABA, 0xBCBCBC, 0xBEBEBE,
										0xC0C0C0, 0xC2C2C2, 0xC4C4C4, 0xC6C6C6,
										0xC8C8C8, 0xCACACA, 0xCCCCCC, 0xCECECE,
										0xD0D0D0, 0xD2D2D2, 0xD4D4D4, 0xD6D6D6,
										0xD8D8D8, 0xDADADA, 0xDCDCDC, 0xDEDEDE,
										0xE0E0E0, 0xE2E2E2, 0xE4E4E4, 0xE6E6E6,
										0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
										0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
										0xF8F8F8, 0xFAFAFA, 0xFCFCFC, 0xFEFEFE,
										0x010000, 0x010000, 0x010000}
								 };

	DRM_INFO("[DISPLAY] %s: \n", __func__);

	if (board_hw_id >= HW_ID_ER) {
		dsi_config->lane_count = 4;
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	} else {
		dsi_config->lane_count = 2;
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
	}
	dsi_config->enable_gamma_csc = ENABLE_CSC | ENABLE_GAMMA;

	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 0;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x3;

	if (board_hw_id >= HW_ID_ER) {
		hw_ctx->high_low_switch_count = 0x17;
		hw_ctx->clk_lane_switch_time_cnt = 0x1D000C;
		hw_ctx->lp_byteclk = 0x3;
		hw_ctx->dphy_param = 0x150C340F;
	} else {
		hw_ctx->lp_byteclk = 0x6;
		hw_ctx->high_low_switch_count = 0x28;
		hw_ctx->clk_lane_switch_time_cnt = 0x00280013;
		hw_ctx->dphy_param = 0x25155B1E;
	}

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xe;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	otm1284a_dsi_config = dsi_config;

	if (!splendid_init) {
		if (dsi_config->enable_gamma_csc & ENABLE_CSC) {
			/* setting the tuned csc setting */
			drm_psb_enable_color_conversion = 1;
			mdfld_intel_crtc_set_color_conversion(dev, &csc);
		}

		if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
			/* setting the tuned gamma setting */
			drm_psb_enable_gamma = 1;
			mdfld_intel_crtc_set_gamma(dev, &gamma);
		}
	}
	splendid_init = true;
}

static int otm1284a_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val, dphy_val;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			dphy_val = REG_READ(regs->dphy_param_reg);
			panel_pwr_on = true;
			cabc_status = 1;
			DRM_INFO("%s: dphy_val = 0x%x\n", __func__, dphy_val);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected in ifwi!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int otm1284a_vid_gpio_init(void)
{
	int ret;

	if (!panel_reset_gpio) {
		panel_reset_gpio = LCD_RST_N;
		if (gpio_request(panel_reset_gpio, "panel_reset")) {
			DRM_ERROR("Faild to request panel reset gpio\n");
			return -EINVAL;
		}
	}

	intel_scu_ipc_iowrite8(LCD_BL_EN, 0x1);

	return 0;
}

static int otm1284a_vid_gpio_control(bool on)
{
	if (on) {
#if SLEEP_MODE_ENABLE
		if (!panel_pwr_on) {
#endif
			intel_scu_ipc_iowrite8(LCD_ANALONG_PWR_EN, LCD_ANALONG_PWR_EN_OFF);
			usleep_range(1000, 1500);

			intel_scu_ipc_iowrite8(LCD_ANALONG_PWR_EN, LCD_ANALONG_PWR_EN_AUTO);
			usleep_range(5000, 5500);
#if SLEEP_MODE_ENABLE
		}
#endif
	} else {
#if SLEEP_MODE_ENABLE
		if (!panel_pwr_on) {
#endif
			usleep_range(50000, 55000);
			if (gpio_direction_output(panel_reset_gpio, 0))
				gpio_set_value_cansleep(panel_reset_gpio, 0);
			usleep_range(120000, 121000);
			intel_scu_ipc_iowrite8(LCD_ANALONG_PWR_EN, LCD_ANALONG_PWR_EN_OFF);
#if SLEEP_MODE_ENABLE
		}
#endif
	}

	return 0;
}

static int otm1284a_vid_shutdown_gpio_control(void)
{
	usleep_range(50000, 55000);
	if (gpio_direction_output(panel_reset_gpio, 0))
		gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(120000, 121000);
	intel_scu_ipc_iowrite8(LCD_ANALONG_PWR_EN, LCD_ANALONG_PWR_EN_OFF);

	DRM_INFO("[DISPLAY] %s\n", __func__);
	return 0;
}

static void otm1284a_vid_get_panel_id(struct mdfld_dsi_pkg_sender *sender)
{
	u8 panel_uniq_id[4] = {0};
	u8 addr = 0x00; /*Panel_Wafer_ID address*/
	u8 data[4];
	int i;

	lcd_unique_id = 0xFFFFFFFF;

	switch (asus_panel_id) {
	case AUO_VID_PANEL:
		/*Reading panel unique ID*/
		mdfld_dsi_send_mcs_short_lp(sender, 0xFE, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE); /*Chip-ID page*/

		/*Vendor ID byte*/
		mdfld_dsi_read_mcs_lp(sender, addr, data, 1);
		panel_uniq_id[3] = data[0];

		for (i = 0; i < 7; i++) {
			mdfld_dsi_read_mcs_lp(sender, addr+i+1, data, 1);
			panel_uniq_id[i % 3] ^= data[0];
		}

		mdfld_dsi_send_mcs_short_lp(sender, 0xFE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE); /*User command set*/

		if (panel_uniq_id[3] != 0x0)
			lcd_unique_id = panel_uniq_id[3] << 24 | panel_uniq_id[1] << 16 | panel_uniq_id[2] << 8 | panel_uniq_id[0];
		break;
	case TM_VID_PANEL:
		/*otm1284a_send_mipi_cmd(sender, otm1284a_cm1, ARRAY_SIZE(otm1284a_cm1));
		otm1284a_send_mipi_cmd(sender, otm1284a_cm2, ARRAY_SIZE(otm1284a_cm2));
		otm1284a_send_mipi_cmd(sender, otm1284a_cm3, ARRAY_SIZE(otm1284a_cm3));
		otm1284a_send_mipi_cmd(sender, otm1284a_cm4, ARRAY_SIZE(otm1284a_cm4));*/

		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_read_gen_lp(sender, 0xF4, 0, 1, data, 4);

		if (data[0] != 0x0)
			lcd_unique_id = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
		break;
	default:
		break;
	}

	printk("[DISPLAY] %s: LCD_UNIQUE_ID = 0x%x\n", __func__, lcd_unique_id);
}

#define MANTISSA_NUM 23

static int raw_luminance_4byte_converter(u8 *byte1, u8 *byte2, u8 *byte3, u8 *byte4)
{
	int i, exponent;
	unsigned long mantissa, mantissa_float = 0;

	exponent = ((*byte4 << 1) | (*byte3 >> 7)) - 127;

	mantissa = *byte1 | (*byte2 << 8) | ((*byte3 & 0x7F) << 16);

	/*for (i = 0; i < MANTISSA_NUM; i++) {
		if ((mantissa >> (MANTISSA_NUM - 1 - i)) & 0x01)
			mantissa_float += 1000000 / (0x01 << (i + 1));
	}*/

	mantissa_float = mantissa * 512 >> MANTISSA_NUM;

	return (mantissa_float << exponent) / 512 + (0x01 << exponent);
}

#define LCD_CALIBRATION_INI_FILE  "/factory/lcd_calibration.ini"
#define LCD_BL_CALIBRATION_SWITCH  "/data/data/bl_cali_enable"
bool read_lcd_calibration_data()
{
	struct file *fp1, *fp2 = NULL;
	unsigned char *readstr;
	unsigned char *readstr2;
	int readlen, readlen2 = 0;
	mm_segment_t old_fs;
	int i;
	unsigned char lcd_uniq_id_crc;
	bool bl_cali_enable = false;
	unsigned char EDID;
	int tmp_luminance, tmp_expected_luminance;

	fp1 = filp_open(LCD_CALIBRATION_INI_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	fp2 = filp_open(LCD_BL_CALIBRATION_SWITCH, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);

	if (IS_ERR_OR_NULL(fp1)) {
		printk("[DISPLAY]no lcd calibration data\n");
		return false;
	}

	readstr = (unsigned char *)kmalloc(256, GFP_KERNEL);
	readstr2 = (unsigned char *)kmalloc(256, GFP_KERNEL);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp1->f_op != NULL && fp1->f_op->read != NULL) {
		readlen = fp1->f_op->read(fp1, readstr, 256, &fp1->f_pos);
		readstr[readlen] = '\0';
	} else
		printk("[DISPLAY]LCD_CALIBRATION_INI_FILE read fail\n");

	if (IS_ERR_OR_NULL(fp2))
		printk("[DISPLAY]Dynamic Backlight Discount disable\n");
	else {
		if (fp2->f_op != NULL && fp2->f_op->read != NULL) {
			readlen2 = fp2->f_op->read(fp2, readstr2, 256, &fp2->f_pos);
			readstr2[readlen2] = '\0';
		}
		kstrtol(readstr2, 0, &tmp_expected_luminance);
		if (0 < tmp_expected_luminance && 1000 >= tmp_expected_luminance)
			bl_cali_enable = true;
			filp_close(fp2, NULL);
	}

	lcd_uniq_id_crc = (lcd_unique_id >> 24) ^ 0x55 ^ (lcd_unique_id >> 16) ^ (lcd_unique_id >> 8) ^ (lcd_unique_id);

	EDID = *(readstr+5);

	if (lcd_uniq_id_crc == EDID) {
		tmp_luminance = ((*(readstr+60) << 8) | *(readstr+61)) * BRIGHTNESS_MAX_LEVEL / 175 ; /*roll back to original brightness before calibration*/
		if (tmp_luminance >= 400 && tmp_luminance <= 1000) /*restrict brighness 400~1000*/
			cali_luminance = tmp_luminance;
		else {
			printk("[DISPLAY]%s:: Calibration Luminance is out of spec \n");
			set_fs(old_fs);
			filp_close(fp1, NULL);
			kfree(readstr);
			kfree(readstr2);
			return false;
		}

		tmp_luminance = raw_luminance_4byte_converter(readstr+52, readstr+53, readstr+54, readstr+55) * BRIGHTNESS_MAX_LEVEL / 175; /*roll back to original brightness after calibration*/
		if (tmp_luminance >= 400 && tmp_luminance <= 1000)
			raw_luminance = tmp_luminance;
		else {
			printk("[DISPLAY]%s:: Original Luminance is out of spec\n");
			set_fs(old_fs);
			filp_close(fp1, NULL);
			kfree(readstr);
			kfree(readstr2);
			return false;
		}

		printk("[DISPLAY] R_LUMI(%d) LCD_UNIQ_ID_CRC(%x) C_LUMI(%d)\n", raw_luminance, EDID, cali_luminance);

		if (bl_cali_enable) {
			default_raw_brightness = raw_luminance;
			default_expected_brightness = tmp_expected_luminance;
			printk("[DISPLAY]%s::BACKLIGHT DISCOUNT MODIFY TO %d / %d\n", __func__, default_expected_brightness, default_raw_brightness);
		}
	} else {
		printk("[DISPLAY]%s:: LCD_UNIQ_ID_CRC Compare Fail\n", __func__);
		cali_luminance = 0;
		raw_luminance = 0;
		set_fs(old_fs);
		filp_close(fp1, NULL);
		kfree(readstr);
		kfree(readstr2);
		return false;
	}

	set_fs(old_fs);
	filp_close(fp1, NULL);

	kfree(readstr);
	kfree(readstr2);

	return true;
}
EXPORT_SYMBOL(read_lcd_calibration_data);

static int otm1284a_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	DRM_INFO("[DISPLAY] %s: Enter +++\n", __func__);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	usleep_range(1000, 1200);
	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(120);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);

	if (!panel_pwr_on) {
		otm1284a_vid_get_panel_id(sender);
		panel_pwr_on = true;
	}

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	if (board_hw_id >= HW_ID_EVB && board_hw_id < HW_ID_ER)
		pwm_enable();

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN, 0x1);
	if (dpms_bl_flag)
		dpms_bl_flag = false;
	DRM_INFO("[DISPLAY] %s: End ---\n", __func__);
	/* ZE500CL001S */
	printk("[PM] Display power on\n");
	SetDisplayPowerOnWithoutFrame(true);
	/* ZE500CL001E */

	return 0;
}


static int otm1284a_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	DRM_INFO("[DISPLAY] %s: Enter+++\n", __func__);

	if (board_hw_id >= HW_ID_EVB && board_hw_id < HW_ID_ER)
		pwm_disable();

	usleep_range(1000, 1500);

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN, 0x0);
	if (dpms_bl_flag)
		dpms_bl_flag = false;

	/* Send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
	if (auo_standby_mode) {
		mdfld_dsi_send_mcs_short_lp(sender, 0xFE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x4F, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
	} else {
		mdfld_dsi_send_mcs_short_lp(sender, 0x28, 0x00, 0, 0);
		mdfld_dsi_send_mcs_short_lp(sender, 0x10, 0x00, 0, 0);
	}
	mdelay(200); /*Make sure MIPI lane entering ulps*/

	otm1284a_vid_gpio_control(0);

	DRM_INFO("[DISPLAY] %s: End---\n", __func__);
	/* ZE500CL001S */
	printk("[PM][LCD ---]\n");
	printk("[PM] Display power off\n");
	/* ZE500CL001E */

	return 0;
}

static int otm1284a_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	DRM_INFO("[DISPLAY] %s: Enter+++\n", __func__);

	otm1284a_vid_gpio_control(1);

	pr_debug("[DISPLAY] %s: End---\n", __func__);

	return 0;
}

static int otm1284a_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;
	int bl_en_value = 0;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (dpms_bl_flag && level != BRIGHTNESS_MIN_LEVEL) {
		DRM_INFO("[DISPLAY] display blank dpms skip set bl expect 0\n");
		return 0;
	}

	duty_val = (level == 1 ? 0 : level) * 100/100;

	if (level >= 1) {

		duty_val = (level < 5 ? 5 : level) * 255 / BRIGHTNESS_MAX_LEVEL;
		
		if (level > 1) {
			if (cabc_status == 1 && duty_val <= 20) {
				mdfld_dsi_send_mcs_short_lp(sender, 0x55, 0x0, 1, 0);
				cabc_status = 0;
			} else if (cabc_status == 0 && duty_val > 20) {
				mdfld_dsi_send_mcs_short_lp(sender, 0x55, 0x3, 1, 0);
				cabc_status = 1;
			}
			backlight_val = duty_val;
		}
	}

	if (board_hw_id >= HW_ID_EVB && board_hw_id < HW_ID_ER) {	/*	workaround for wrong hw_id	*/
		if (duty_val > BRIGHTNESS_MAX_LEVEL)
			duty_val = BRIGHTNESS_MAX_LEVEL;
		else if (duty_val < BRIGHTNESS_MIN_LEVEL)
			duty_val = BRIGHTNESS_INIT_LEVEL;
		pwm_configure(duty_val);
	} else {
		mdfld_dsi_send_mcs_short_lp(sender, 0x51, duty_val, 1, 0);
	}

	DRM_INFO("[DISPLAY] brightness level = %d , duty_val = %d\n", level, duty_val);
	return 0;
}

struct drm_display_mode *otm1284a_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
	DRM_INFO("[DISPLAY] %s:\n", __func__);

	mode->hdisplay = otm1284a_panel_conf.width;
	mode->vdisplay = otm1284a_panel_conf.height;

	mode->hsync_start = mode->hdisplay + otm1284a_panel_conf.hfp;
	mode->hsync_end = mode->hsync_start + otm1284a_panel_conf.hpulse;
	mode->htotal = mode->hsync_end + otm1284a_panel_conf.hbp;
	mode->vsync_start = mode->vdisplay + otm1284a_panel_conf.vfp;
	mode->vsync_end = mode->vsync_start + otm1284a_panel_conf.vpulse;
	mode->vtotal = mode->vsync_end + otm1284a_panel_conf.vbp;

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void otm1284a_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 65;
	pi->height_mm = 114;
}

#ifdef OTM1284A_DEBUG
static int psb_fb_blank = -1;
static int mipi_read_data;
static int mipi_send_ret = -1;

static ssize_t blank_fb_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", psb_fb_blank);
}

static ssize_t blank_fb_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct drm_device *drmdev = otm1284a_dsi_config->dev;
    struct drm_psb_private *dev_priv = (struct drm_psb_private *) drmdev->dev_private;

    sscanf(buf, "%x", &psb_fb_blank);

	DRM_INFO("[DISPLAY] %s: blank(%d)\n", __func__, psb_fb_blank);

    psbfb_panel_blank(dev_priv, psb_fb_blank);

    return count;
}

static ssize_t mipi_send_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mipi_send_ret);
}

static ssize_t mipi_send_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdfld_dsi_pkg_sender *sender = mdfld_dsi_get_pkg_sender(otm1284a_dsi_config);
	u8 mipi_x0 = 0, mipi_x1 = 0;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	sscanf(buf, "%x,%x", &mipi_x0, &mipi_x1);

	mipi_send_ret = mdfld_dsi_send_mcs_short_lp(sender, mipi_x0, mipi_x1, 1, 0);
	DRM_INFO("[DISPLAY] %s: 0x%x = 0x%x, ret =%d\n", __func__, mipi_x0, mipi_x1, mipi_send_ret);

    return count;
}

static ssize_t mipi_read_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mipi_read_data);
}

static ssize_t mipi_read_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdfld_dsi_pkg_sender *sender = mdfld_dsi_get_pkg_sender(otm1284a_dsi_config);
	static int mipi_read_ret = -1;
	int data = 0;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	sscanf(buf, "%x", &mipi_read_data);

	mipi_read_ret = mdfld_dsi_read_mcs_lp(sender, mipi_read_data, &data, 1);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		mipi_read_ret = -EIO;

	DRM_INFO("[DISPLAY] %s: read 0x%x = %x, ret =%d\n", __func__, mipi_read_data, data, mipi_read_ret);

    return count;
}

DEVICE_ATTR(psbfb_on_off, S_IRUGO | S_IWUSR, blank_fb_show, blank_fb_store);
DEVICE_ATTR(psbfb_mipi_send, S_IRUGO | S_IWUSR, mipi_send_show, mipi_send_store);
DEVICE_ATTR(psbfb_mipi_read, S_IRUGO | S_IWUSR, mipi_read_show, mipi_read_store);

static struct attribute *otm1284a_attrs[] = {
	&dev_attr_psbfb_on_off.attr,
	&dev_attr_psbfb_mipi_send.attr,
	&dev_attr_psbfb_mipi_read.attr,
	NULL
};

static struct attribute_group otm1284a_attr_group = {
	.attrs = otm1284a_attrs,
	.name = "otm1284a",
};
#endif

static int asus_panel_init()
{
	board_tp_id = Read_TP_ID();

	switch (board_tp_id) {
	case 0:
	case 4:
		asus_panel_id = AUO_VID_PANEL;
		/*switch AUO to sleepmode due to fifo error during resuming*/
		/*auo_standby_mode = AUO_STANDBY_MODE_ENABLE;*/
		otm1284a_cmd_table = auo_cmd_table;
		otm1284a_cmd_table_size = ARRAY_SIZE(auo_cmd_table);
		if (board_hw_id >= HW_ID_ER)
			otm1284a_panel_conf = auo_panel_conf_4lane;
		else
			otm1284a_panel_conf = auo_panel_conf;
		break;
	case 2:
	case 6:
		asus_panel_id = TM_VID_PANEL;
		otm1284a_cmd_table = tm_cmd_table;
		otm1284a_cmd_table_size = ARRAY_SIZE(tm_cmd_table);
		otm1284a_panel_conf = tm_panel_conf;
		break;
	case 3:
	case 7:
		asus_panel_id = CPT_VID_PANEL;
		otm1284a_cmd_table = cpt_cmd_table;
		otm1284a_cmd_table_size = ARRAY_SIZE(cpt_cmd_table);
		otm1284a_panel_conf = cpt_panel_conf;
		break;
	default:
		asus_panel_id = NO_PANEL;
		otm1284a_cmd_table = otp_cmd_table;
		otm1284a_cmd_table_size = ARRAY_SIZE(otp_cmd_table);
		if (board_hw_id >= HW_ID_ER)
			otm1284a_panel_conf = auo_panel_conf_4lane;
		else
			otm1284a_panel_conf = auo_panel_conf;
		break;
	}

	DRM_INFO("[DISPLAY] %s: board_tp_id = %d, %s is connected\n", __func__, board_tp_id, supp_panels[asus_panel_id].name);

	return 0;
}

void otm1284a_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];

	int ret = 0;

	p_funcs->get_config_mode = otm1284a_vid_get_config_mode;
	p_funcs->get_panel_info = otm1284a_vid_get_panel_info;
	p_funcs->dsi_controller_init = otm1284a_vid_dsi_controller_init;
	p_funcs->detect = otm1284a_vid_detect;
	p_funcs->power_on = otm1284a_vid_power_on;
	p_funcs->drv_ic_init = otm1284a_drv_ic_init;
	p_funcs->power_off = otm1284a_vid_power_off;
	p_funcs->reset = otm1284a_vid_reset;
	p_funcs->set_brightness = otm1284a_vid_set_brightness;

	ret = otm1284a_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for otm1284a panel\n");

	board_hw_id = Read_HW_ID();

	if (board_hw_id >= HW_ID_EVB && board_hw_id < HW_ID_ER)
		ret = otm1284a_vid_brightness_init();

	asus_panel_init();

	DRM_INFO("%s: board_hw_id = %d\n", __func__, board_hw_id);

#ifdef OTM1284A_DEBUG
	sysfs_create_group(&dev->dev->kobj, &otm1284a_attr_group);
#endif

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

}

static int otm1284a_vid_lcd_probe(struct platform_device *pdev)
{
	int ret = 0;
	printk("[Progress][OTM1284A_vid] Probe starts\n");
	default_raw_brightness = 582;
	default_expected_brightness = MAX_BRIGHTNESS_NIT;
	DRM_INFO("%s: otm1284a panel detected\n", __func__);
	intel_mid_panel_register(otm1284a_vid_init);
	printk("[Progress][OTM1284A_vid] Probe ends\n");

	return 0;
}

static int otm1284a_vid_lcd_shutdown(struct platform_device *pdev)
{

	DRM_INFO("%s: Enter +++\n", __func__);
	struct drm_device *drmdev = otm1284a_dsi_config->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) drmdev->dev_private;

	mdfld_dsi_dpi_set_power(encoder_shutdown, 0);
	/*otm1284a_vid_power_off(otm1284a_dsi_config);*/
	/*otm1284a_vid_shutdown_gpio_control();*/
	mdelay(10);

	DRM_INFO("%s: End ---\n", __func__);

	return 0;
}

struct platform_driver ze500cl_vid_lcd_driver = {
	.probe	= otm1284a_vid_lcd_probe,
	.driver	= {
		.name	= OTM1284A_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
	.shutdown = otm1284a_vid_lcd_shutdown,
};


