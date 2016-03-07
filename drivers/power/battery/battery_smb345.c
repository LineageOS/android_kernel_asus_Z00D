/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/power/smb345-me372cg-charger.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>

#include "smb345_external_include.h"
#include "asus_battery.h"
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/usb/penwell_otg.h>
/*#include "../intel_mdf_charger.h"*/
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>
#include <linux/microp_api.h>
#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#define I2C_TEST_SMB345_FAIL (-1)
#endif
static int HW_ID;
extern int Read_HW_ID(void);
extern bool g_Main_mode;
static bool is_pad_present();
static void smb345_config_max_current(int usb_state);
static bool g_disable_700 = true;
static bool g_init_disable_aicl = true;

/* I2C communication related */
#define I2C_RETRY_COUNT 3
#define I2C_RETRY_DELAY 5

#define CFG_CHARGE_CURRENT            0x00
#define CFG_CHARGE_CURRENT_FCC_MASK        0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT        5
#define CFG_CHARGE_CURRENT_PCC_MASK        0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT        3
#define CFG_CHARGE_CURRENT_TC_MASK        0x07
#define CFG_CHARGE_CURRENT_ALL        0x41

#define CFG_CURRENT_LIMIT            0x01
#define CFG_CURRENT_LIMIT_DC_MASK        0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT        4
#define CFG_CURRENT_LIMIT_USB_MASK        0x0f
#define CFG_CURRENT_LIMIT_SMB346_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB346_VALUE_700 0x20
#define CFG_CURRENT_LIMIT_SMB346_VALUE_500 0x10
#define CFG_VARIOUS_FUNCS            0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB        BIT(2)
#define CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE    BIT(4)
#define CFG_VARIOUS_FUNCS_BATTERY_OV    BIT(1)
#define CFG_FLOAT_VOLTAGE            0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK    0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT    6
#define CFG_STAT                0x05
#define CFG_STAT_DISABLED            BIT(5)
#define CFG_STAT_ACTIVE_HIGH            BIT(7)
#define CFG_PIN                    0x06
#define CFG_PIN_EN_CTRL_MASK            0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH        0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW        0x60
#define CFG_PIN_EN_APSD_IRQ            BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR        BIT(2)
#define CFG_THERM                0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK    0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT    0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK    0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT    2
#define CFG_THERM_MONITOR_DISABLED        BIT(4)
#define CFG_SYSOK                0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED    BIT(2)
#define CFG_OTHER                0x09
#define CFG_OTHER_RID_MASK            0xc0
#define CFG_OTHER_RID_DISABLED_OTG_PIN        0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C        0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG        0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW        BIT(5)
#define CFG_OTG                    0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK        0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT        4
#define CFG_OTG_CC_COMPENSATION_MASK        0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT        6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK    0x03
#define CFG_TEMP_LIMIT                0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK        0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT        0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK        0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT        2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK        0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT        4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK        0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT        6
#define CFG_FAULT_IRQ                0x0c
#define CFG_FAULT_IRQ_DCIN_UV            BIT(2)
#define CFG_FAULT_IRQ_OTG_UV            BIT(5)
#define CFG_STATUS_IRQ                0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT        BIT(7)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER    BIT(4)
#define CFG_ADDRESS                0x0e

/* Command registers */
#define CMD_A                    0x30
#define CMD_A_CHG_ENABLED            BIT(1)
#define CMD_A_SUSPEND_ENABLED            BIT(2)
#define CMD_A_OTG_ENABLED            BIT(4)
#define CMD_A_ALLOW_WRITE            BIT(7)
#define CMD_B                    0x31
#define CMD_B_USB9_AND_HC_MODE    0x03
#define CMD_C                    0x33

/* Interrupt Status registers */
#define IRQSTAT_A                0x35
#define IRQSTAT_C                0x37
#define IRQSTAT_C_TERMINATION_STAT        BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ        BIT(1)
#define IRQSTAT_C_TAPER_IRQ            BIT(3)
#define IRQSTAT_D                0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT        BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ        BIT(3)
#define IRQSTAT_E                0x39
#define IRQSTAT_E_USBIN_UV_STAT            BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ            BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT            BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ            BIT(5)
#define IRQSTAT_F                0x3a
#define IRQSTAT_F_OTG_UV_IRQ            BIT(5)
#define IRQSTAT_F_OTG_UV_STAT            BIT(4)

/* Status registers */
#define STAT_A                    0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK        0x3f
#define STAT_B                    0x3c
#define STAT_C                    0x3d
#define STAT_C_CHG_ENABLED            BIT(0)
#define STAT_C_HOLDOFF_STAT            BIT(3)
#define STAT_C_CHG_MASK                0x06
#define STAT_C_CHG_SHIFT            1
#define STAT_C_CHG_TERM                BIT(5)
#define STAT_C_CHARGER_ERROR            BIT(6)
#define STAT_E                    0x3f

#define STATUS_UPDATE_INTERVAL            (HZ * 60)

struct smb345_otg_event {
	struct list_head    node;
	bool            param;
};

struct smb345_charger {
	struct mutex        lock;
	struct i2c_client    *client;
	struct power_supply    mains;
	struct power_supply    usb;
	struct power_supply    battery;
	bool            mains_online;
	bool            usb_online;
	bool            charging_enabled;
	bool            running;
	struct dentry        *dentry;
	struct dentry        *dentry2;
	struct otg_transceiver    *otg;
	struct notifier_block    otg_nb;
	struct work_struct    otg_work;
	struct delayed_work aicl_dete_work;
	struct delayed_work smb_probe_work;
	struct workqueue_struct *chrgr_work_queue;
	struct list_head    otg_queue;
	spinlock_t        otg_queue_lock;
	bool            otg_enabled;
	bool            otg_battery_uv;
	const struct smb345_charger_platform_data    *pdata;
	/* wake lock to prevent S3 during charging */
	struct wake_lock wakelock;
};

static bool pad_pwr_supply();
#ifdef CONFIG_UPI_BATTERY
struct wake_lock wlock;
struct wake_lock wlock_t;
#endif

/* global charger type variable lock */
DEFINE_MUTEX(g_usb_state_lock);
static int g_usb_state = CABLE_OUT;
static int g_pad_cable_state = CABLE_OUT;
/* global software charging toggle lock */
DEFINE_MUTEX(g_charging_toggle_lock);
static bool g_charging_toggle = true;

static struct smb345_charger *smb345_dev;
static struct chgr_dev_func smb345_tbl;

/* Input current limit in mA */
static const unsigned int icl_tbl[] = {
	300,
	500,
	700,
	1000,
	1200,
	1300,
	1800,
	2000,
};

#define EXPORT_CHARGER_OTG

#define DEBUG 1
#define DRIVER_VERSION            "1.1.0"

#define SMB345_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

/* Register definitions */
#define CHG_CURRENT_REG            0x00
#define INPUT_CURRENT_LIMIT_REG    0x01
#define VAR_FUNC_REG            0x02
#define FLOAT_VOLTAGE_REG        0x03
#define CHG_CTRL_REG            0x04
#define STAT_TIMER_REG            0x05
#define PIN_ENABLE_CTRL_REG        0x06
#define THERM_CTRL_A_REG        0x07
#define SYSOK_USB3_SELECT_REG    0x08
#define OTHER_CTRL_A_REG        0x09
#define OTG_TLIM_THERM_CNTRL_REG                0x0A

#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG    0x0B
#define SOFT_LIMIT_HOT_CELL_TEMP_MASK            SMB345_MASK(2, 0)
#define SOFT_LIMIT_COLD_CELL_TEMP_MASK            SMB345_MASK(2, 2)
#define HARD_LIMIT_HOT_CELL_TEMP_MASK            SMB345_MASK(2, 4)
#define HARD_LIMIT_COLD_CELL_TEMP_MASK            SMB345_MASK(2, 6)

#define FAULT_INTERRUPT_REG        0x0C
#define STATUS_INTERRUPT_REG    0x0D
#define I2C_BUS_SLAVE_REG        0x0E
#define CMD_A_REG        0x30
#define CMD_B_REG        0x31
#define CMD_C_REG        0x33
#define INTERRUPT_A_REG        0x35
#define INTERRUPT_B_REG        0x36
#define INTERRUPT_C_REG        0x37
#define INTERRUPT_D_REG        0x38
#define INTERRUPT_E_REG        0x39
#define INTERRUPT_F_REG        0x3A
#define STATUS_A_REG    0x3B
#define STATUS_B_REG    0x3C
#define STATUS_C_REG    0x3D
#define STATUS_D_REG    0x3E
#define STATUS_E_REG    0x3F

/* Status bits and masks */
#define CHG_STATUS_MASK        SMB345_MASK(2, 1)
#define CHG_ENABLE_STATUS_BIT        BIT(0)

/* Control bits and masks */
#define RECHARGE_VOLTAGE_MASK    SMB345_MASK(2, 2)
#define FAST_CHG_CURRENT_MASK            SMB345_MASK(4, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK        SMB345_MASK(4, 0)
#define PRE_CHG_CURRENT_MASK            SMB345_MASK(3, 5)
#define TERMINATION_CURRENT_MASK        SMB345_MASK(3, 2)
#define PRE_CHG_TO_FAST_CHG_THRESH_MASK    SMB345_MASK(2, 6)
#define FLOAT_VOLTAGE_MASK                SMB345_MASK(6, 0)
#define CHG_ENABLE_BIT            BIT(1)
#define VOLATILE_W_PERM_BIT        BIT(7)
#define USB_SELECTION_BIT        BIT(1)
#define SYSTEM_FET_ENABLE_BIT    BIT(7)
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT            BIT(4)
#define AUTOMATIC_POWER_SOURCE_DETECTION_BIT    BIT(2)
#define BATT_OV_END_CHG_BIT        BIT(1)
#define VCHG_FUNCTION            BIT(0)
#define CURR_TERM_END_CHG_BIT    BIT(6)

#define OTGID_PIN_CONTROL_MASK    SMB345_MASK(2, 6)
#define OTGID_PIN_CONTROL_BITS    BIT(6)

#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    SMB345_MASK(2, 2)
#define OTG_CURRENT_LIMIT_750mA    (BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_500mA    BIT(3)
#define OTG_CURRENT_LIMIT_250mA    BIT(2)
#define OTG_BATTERY_UVLO_THRESHOLD_MASK    SMB345_MASK(2, 0)

#define CHARGE_CURRENT_COMPENSATION         SMB345_MASK(2, 6)
#define CHARGE_CURRENT_COMPENSATION_VALUE   0x00

#define CREATE_DEBUGFS_INTERRUPT_STATUS_REGISTERS
#define SMB358_FAST_CHG_CURRENT_MASK            SMB345_MASK(3, 5)
#define SMB358_TERMINATION_CURRENT_MASK         SMB345_MASK(3, 0)
#define SMB358_TERMINATION_CURRENT_VALUE_100mA    BIT(2)
#define SMB358_TERMINATION_CURRENT_VALUE_200mA    (BIT(0)|BIT(1)|BIT(2))
#define SMB358_FAST_CHG_CURRENT_VALUE_2000mA    (BIT(5)|BIT(6)|BIT(7))
#define SMB358_FAST_CHG_CURRENT_VALUE_1300mA    BIT(7)
#define SMB358_FAST_CHG_CURRENT_VALUE_900mA    (BIT(5)|BIT(6))
#define SMB358_FAST_CHG_CURRENT_VALUE_600mA    BIT(6)
#define SMB358_COLD_SOFT_LIMIT_CURRENT_VALUE_600mA    BIT(7)
#define SMB358_FLOAT_VOLTAGE_VALUE_4340mV    (BIT(1)|BIT(3)|BIT(5))
#define SMB358_FLOAT_VOLTAGE_VALUE_4100mV    (BIT(1)|BIT(2)|BIT(3)|BIT(4))
#define SMB358_VFLT_50mA    0x00
#define SMB358_VFLT_200mA    BIT(3)
#define SWITCHING_FREQUENCY_1_5MHZ    BIT(7)
#define IC_RESET        BIT(7)
static int smb346_soc_detect_batt_tempr(int usb_state);
static inline int get_battery_rsoc(int *rsoc);
static inline int get_battery_temperature(int *tempr);
static inline int get_battery_voltage(int *volt);
static inline int get_battery_status(int *status);

#ifdef CONFIG_TOUCHSCREEN_FT3X17
extern void focal_usb_detection(bool);
#endif


#if defined(ASUS_FACTORY_BUILD)
bool eng_charging_limit;
bool g_charging_toggle_for_charging_limit;
#endif
static char *supply_list[] = {
	"battery",
};

static enum power_supply_property asus_power_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb345_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val);

static struct power_supply smb345_power_supplies[] = {
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
};

static int smb345_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	int usb_state;
	int chrg_status;

	usb_state = g_usb_state;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:

		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			if (is_pad_present()) {
				val->intval = 0;
			} else{
				val->intval = (usb_state == USB_IN) ? 1 : 0;
			}

		} else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (is_pad_present()) {
				val->intval = (AX_MicroP_get_USBDetectStatus(Batt_P01) == P01_CABLE_CHARGER) ? 1 : 0;
			} else {
				val->intval = (usb_state == AC_IN || usb_state == UNKNOWN_IN) ? 1 : 0;
			}

		} else {

			ret = -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:

		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			/* for ATD test to acquire the status about charger ic */
			if (!smb345_has_charger_error()) {
				val->intval = 1;
				return 0;
			}
			chrg_status = smb345_get_charging_status();
			if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
				val->intval = 1;
				return 0;
			} else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
				val->intval = 1;
				return 0;
			}
			val->intval = 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (!smb345_has_charger_error()) {
				val->intval = 1;
				return 0;
			}

			chrg_status = smb345_get_charging_status();

			if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
				val->intval = 1;
				return 0;
			} else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
				val->intval = 1;
				return 0;
			}
			val->intval = 0;
		} else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

extern bool get_sw_charging_toggle()
{
	bool ret;

	mutex_lock(&g_charging_toggle_lock);
	ret = g_charging_toggle;
	mutex_unlock(&g_charging_toggle_lock);

	return ret;
}

extern int get_charger_type()
{
	int ret;

	if (!smb345_dev) {
		pr_err("Warning: smb345_dev is null due to probe function has error\n");
		return -1;
	}

	if (is_pad_present())
		ret = PAD_SUPPLY;
	else
		ret = g_usb_state;

	return ret;
}

int request_power_supply_changed()
{
	int ret = -1;

	if (!smb345_dev) {
		pr_err("Warning: smb345_dev is null due to probe function has error\n");
		return -1;
	}

	power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
	power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);

	return ret;
}

static int smb345_register_power_supply(struct device *dev)
{
	int ret;

	ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_USB-1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply USB\n");
		goto batt_err_reg_fail_usb;
	}

	ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_AC-1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply AC\n");
		goto batt_err_reg_fail_ac;
	}

	return 0;

batt_err_reg_fail_ac:
	power_supply_unregister(&smb345_power_supplies[CHARGER_USB-1]);
batt_err_reg_fail_usb:
	return ret;
}

static int smb345_read_reg(struct i2c_client *client, int reg,
	u8 *val, int ifDebug)
{
	s32 ret;
	struct smb345_charger *smb345_chg;

	smb345_chg = i2c_get_clientdata(client);
	ret = i2c_smbus_read_byte_data(smb345_chg->client, reg);
	if (ret < 0) {
		dev_err(&smb345_chg->client->dev,
			"i2c read fail: can't read from Reg%02Xh: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	if (ifDebug)
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(*val));

	return 0;
}

static int smb345_write_reg(struct i2c_client *client, int reg, u8 val)
{

	s32 ret;
	struct smb345_charger *smb345_chg;

	smb345_chg = i2c_get_clientdata(client);

	ret = i2c_smbus_write_byte_data(smb345_chg->client, reg, val);
	if (ret < 0) {
		dev_err(&smb345_chg->client->dev,
			"i2c write fail: can't write %02X to %02X: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb345_masked_write(struct i2c_client *client, int reg,
		u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = smb345_read_reg(client, reg, &temp, 0);
	if (rc) {
		pr_err("smb345_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb345_write_reg(client, reg, temp);
	if (rc) {
		pr_err("smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int smb345_read(struct smb345_charger *smb, u8 reg)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		ret = i2c_smbus_read_byte_data(smb->client, reg);
		if (ret < 0) {
			retry_count--;
			dev_warn(&smb->client->dev, "fail to read reg %02xh: %d\n",
				reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);


	return ret;
}

static int smb345_write(struct smb345_charger *smb, u8 reg, u8 val)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		ret = i2c_smbus_write_byte_data(smb->client, reg, val);
		if (ret < 0) {
			retry_count--;
			dev_warn(&smb->client->dev, "fail to write reg %02xh: %d\n",
				reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}

static int smb345_set_writable(struct smb345_charger *smb, bool writable)
{
	int ret;

	ret = smb345_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb345_write(smb, CMD_A, ret);
}

static int cancel_soft_hot_temp_limit(bool cancel_it)
{
	int ret = -1;

	/* ME371MG EVB/SR1 meet this problem that smb345 stop charging
	    when IC temperature is high up to Soft Hot Limit. But Battery
	    is not full charging. We disable this limitation.
	*/

	ret = smb345_read(smb345_dev, CFG_THERM);
	if (ret < 0)
		return ret;

	ret &= ~CFG_THERM_SOFT_HOT_COMPENSATION_MASK;
	if (!cancel_it)
		ret &= 0x02;

	ret = smb345_write(smb345_dev, CFG_THERM, ret);

	return ret;
}
#if defined(ASUS_FACTORY_BUILD)
static bool asus_battery_charging_limit()
{
	int recharging_soc = 54;
	int discharging_soc = 55;
	int percentage;

	if (get_battery_rsoc(&percentage)) {
		BAT_DBG_E(" %s: * fail to get battery rsoc *\n", __func__);
		g_charging_toggle_for_charging_limit = true;
	} else{
		if (eng_charging_limit) {
			/*BSP david: enable charging when soc <= recharging soc*/
			if (percentage <= recharging_soc) {
				BAT_DBG("%s, soc: %d <= recharging soc: %d , enable charging\n", __FUNCTION__, percentage, recharging_soc);
				g_charging_toggle_for_charging_limit = true;
			/*BSP david: disable charging when soc >= discharging soc*/
			} else if (percentage >= discharging_soc) {
				BAT_DBG("%s, soc: %d >= discharging soc: %d , disable charging\n", __FUNCTION__, percentage, discharging_soc);
				g_charging_toggle_for_charging_limit = false;
			} else{
				BAT_DBG("%s, soc: %d, between %d and %d, maintain original charging toggle:%d\n", __FUNCTION__, percentage, recharging_soc, discharging_soc, g_charging_toggle_for_charging_limit);
			}
		} else{
			BAT_DBG("%s, charging limit disable, enable charging!\n", __FUNCTION__);
			g_charging_toggle_for_charging_limit = true;
		}
	}
	return g_charging_toggle_for_charging_limit;
}
#endif
/*----------------------------------------------------------------------------*/
/* JEITA function for cell temperature control by SoC
 */
 int smb345_set_recharge_voltage()
{
	int ret;
	int vflt_value = 0;
	int voltage;

	/* acquire battery voltage here */
	ret = get_battery_voltage(&voltage);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __func__);
		return;
	} else{
	}

	if (voltage < 4250) {
		BAT_DBG("%s: VBAT < 4.25, set VFLT 50mA\n", __FUNCTION__);
		vflt_value = SMB358_VFLT_50mA;
	} else{
		BAT_DBG("%s: VBAT >= 4.25, set VFLT 200mA\n", __FUNCTION__);
		vflt_value = SMB358_VFLT_200mA;
	}
	/*Set recharge voltage*/
	/* write 01h[3:2]="01" */
	ret = smb345_masked_write(smb345_dev->client,
			CFG_CURRENT_LIMIT,
			RECHARGE_VOLTAGE_MASK,
			vflt_value);
	if (ret) {
		pr_err("fail to set RECHARGE_VOLTAGE_MASK ret=%d\n", ret);
	}


}
 int smb345_soc_control_jeita(void)
{
	int ret;

	if (!smb345_dev) {
		pr_err("Warning: smb345_dev is null due to probe function has error\n");
		return 1;
	}

	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	smb345_set_recharge_voltage();
	/* write 0bh[5:4]="11" */
	ret = smb345_masked_write(smb345_dev->client,
			HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
			HARD_LIMIT_HOT_CELL_TEMP_MASK,
			0xff);
	if (ret) {
		pr_err("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}

	/* write 07h[1:0]="00" */
	ret = smb345_masked_write(smb345_dev->client,
			CFG_THERM,
			CFG_THERM_SOFT_HOT_COMPENSATION_MASK,
			0);
	if (ret) {
		pr_err("fail to set CFG_THERM_SOFT_HOT_COMPENSATION_MASK ret=%d\n", ret);
		return ret;
	}
	/* write 07h[3:2]="00" */
	ret = smb345_masked_write(smb345_dev->client,
			CFG_THERM,
			SOFT_LIMIT_COLD_CELL_TEMP_MASK,
			0);
	if (ret) {
		pr_err("fail to set SOFT_LIMIT_COLD_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}

	return ret;
}
int smb345_charger_control_jeita(void)
{
	int ret;
	bool charging_enable = true;
	BAT_DBG("%s:+++\n", __func__);

	if (!smb345_dev) {
		BAT_DBG_E("Warning: smb345_dev is null due to probe function has error\n");
		return 1;
	}
	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;
	smb345_set_recharge_voltage();

	/* write 0bh[5:4]= "00" */
	ret = smb345_masked_write(smb345_dev->client,
			HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
			HARD_LIMIT_HOT_CELL_TEMP_MASK,
			0x00);
	if (ret) {
		BAT_DBG_E("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}

	/* write 07h[1:0]="10" */
	ret = smb345_masked_write(smb345_dev->client,
			CFG_THERM,
			CFG_THERM_SOFT_HOT_COMPENSATION_MASK,
			0x02);
	if (ret) {
		BAT_DBG_E("fail to set Soft Hot Limit Behavior to Float Voltage Compensation ret=%d\n", ret);
		return ret;
	}

	/* write 07h[3:2]="01" */
	ret = smb345_masked_write(smb345_dev->client,
			CFG_THERM,
			SOFT_LIMIT_COLD_CELL_TEMP_MASK,
			0x04);
	if (ret) {
		BAT_DBG_E("fail to set Soft Cold Temp Limit to Charge Current Compensation ret=%d\n", ret);
		return ret;
	}
	/*BSP david: do 5060 when FACTORY BUILD defined*/
#if defined(ASUS_FACTORY_BUILD)
	charging_enable = asus_battery_charging_limit();
#endif

	ret = smb345_charging_toggle(JEITA, charging_enable);
	if (ret) {
		pr_err("fail to set charging enable ret=%d\n", ret);
		return ret;
	}

	BAT_DBG("%s:---\n", __func__);

	return ret;
};
/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl,
		size_t size, unsigned int val)
{
	if (val >= size)
		return tbl[size-1];
	return tbl[val];
}
/* Acquire the value of AICL Results in Status Register E (3Fh)
   return the current value (unit: mA)
*/
static int get_aicl_results(void)
{
	int ret;
	ret = smb345_read(smb345_dev, STAT_E);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
		return ret;
	}

	ret &= 0x0F;
	return hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret);
}

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: ERROR: smb345_dev is null due to probe function has error\n",
			__func__);

		return sprintf(buf, "%d\n", -EINVAL);
	}

	ret = smb345_read(smb345_dev, STAT_E);
	if (ret < 0) {
		pr_info("%s: ERROR: i2c read error\n", __func__);
		return sprintf(buf, "%d\n", -EIO);
	}

	ret &= 0x0F;

	return sprintf(buf, "%d\n", hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret));
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;

	ret = smb345_get_charging_status();
	if (ret == POWER_SUPPLY_STATUS_CHARGING || ret == POWER_SUPPLY_STATUS_FULL)
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n", ret);
}

static char gbuffer[64];
/* Generate UUID by invoking kernel library */
static void generate_key(void)
{
	char sysctl_bootid[16];

	generate_random_uuid(sysctl_bootid);
	sprintf(gbuffer, "%pU", sysctl_bootid);
}

/* Acquire the UUID */
static ssize_t get_charge_keys(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	generate_key();
	return sprintf(buf, "%s\n", gbuffer);
}

static DEVICE_ATTR(charge_keys, S_IRUGO, get_charge_keys, NULL);
static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_input_current.attr,
	&dev_attr_charge_status.attr,
	&dev_attr_charge_keys.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

#define SMB_DUMP(...) \
do { \
	local_len = sprintf(page, __VA_ARGS__); \
	len += local_len; \
	page += local_len; \
} while (0)

/*----------------------------------------------------------------------------*/

static int config_otg_regs(int toggle)
{
	int ret;

	if (toggle) {
		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(smb345_dev->client,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}
		/* Toggle to enable OTG function: output High */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 1);

		/* Set OTG current limit to 500mA: 0Ah[3:2]="01" */
		ret = smb345_masked_write(smb345_dev->client,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			BIT(2));
		if (ret) {
			pr_err("fail to set OTG current limit 500mA ret=%d\n", ret);
			return ret;
		}
	} else{

		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(smb345_dev->client,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Toggle to disable OTG function: output Low */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 0);

	}
    return ret;
}
/*----------------------------------------------------------------------------*/
static int otg(int toggle)
{
	int ret;

	if (!smb345_dev) {
		pr_info("Warning: smb345_dev is null due to probe function has error\n");
		return 1;
	}


	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;


	ret = config_otg_regs(toggle);
	if (ret < 0)
		return ret;

	smb345_dev->otg_enabled = (toggle > 0 ? true : false);
	return 0;
}

/* enable/disable AICL function */
static int smb345_OptiCharge_Toggle(bool on)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return -1;
	}

	ret = smb345_read(smb345_dev, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;

	if (on)
		ret |= CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;
	else
		ret &= ~CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;

	ret = smb345_write(smb345_dev, CFG_VARIOUS_FUNCS, ret);
	if (ret < 0)
		goto fail;

fail:
	return ret;
}

/* print the value in Various Functions Register */
static int smb345_get_AICL(void)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
		__func__);

		return -1;
	}

	ret = smb345_read(smb345_dev, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_VARIOUS_FUNCS, BYTETOBINARY(ret));
fail:
	return ret;
}

/* print the value in CMD_B */
static int smb345_get_USB9_HC_Toggle(void)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);
		return -1;

	}

	ret = smb345_read(smb345_dev, CMD_B);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
fail:
	return ret;
}


/* print the value in CFG_PIN */
static int smb345_get_USB9_HC_PIN_Control(void)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return -1;
	}

	ret = smb345_read(smb345_dev, CFG_PIN);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_PIN, BYTETOBINARY(ret));
fail:
	return ret;
}

/* Convert current to register value using lookup table */
static int current_to_hw(const unsigned int *tbl,
		size_t size, unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

/* print the value in CFG_CURRENT_LIMIT */
static int smb345_get_current_limits()
{
	int ret, index = -1;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);
		return -1;

	}

	ret = smb345_read(smb345_dev, CFG_CURRENT_LIMIT);
	if (ret < 0)
		return ret;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_CURRENT_LIMIT, BYTETOBINARY(ret));
	return ret;
}

/* config DC_IN current limits according to BAT RSOC */
static int smb346_set_pad_supply_dc_in_limits(int rsoc)
{
	int ret;

	if (rsoc >= 20)
		g_disable_700 = true;
	else if (rsoc <= 15)
		g_disable_700 = false;


	if ((rsoc > 15) && (g_disable_700)) {

		pr_info("%s: CFG_CURRENT_LIMIT_SMB346_VALUE_500(%d)\n", __func__, g_disable_700);

		/* Disable AICL - Write 02h[4]="0" */
		if (smb345_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			return;
		}

		ret = smb345_masked_write(smb345_dev->client,
				CFG_CURRENT_LIMIT,
				CFG_CURRENT_LIMIT_SMB346_MASK,
				CFG_CURRENT_LIMIT_SMB346_VALUE_500);

		/* Enable AICL - Write 02h[4]="1" */
		if (smb345_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}
	}  else if (rsoc > -1) {

		pr_info("%s: CFG_CURRENT_LIMIT_SMB346_VALUE_700(%d)\n", __func__, g_disable_700);


		/* Disable AICL - Write 02h[4]="0" */
		if (smb345_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			return;
		}

		ret = smb345_masked_write(smb345_dev->client,
				CFG_CURRENT_LIMIT,
				CFG_CURRENT_LIMIT_SMB346_MASK,
				CFG_CURRENT_LIMIT_SMB346_VALUE_700);

		/* Enable AICL - Write 02h[4]="1" */
		if (smb345_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}
	} else{

		pr_info("%s: smb346_set_pad_supply_dc_in_limits FAIL, rsoc = %d, g_disable_700 = %d !!!\n", __func__, rsoc, g_disable_700);
		ret = -1;
	}


	if (ret < 0)
		return ret;

	return ret;
}

static int smb345_set_current_limits(int usb_state, bool is_twinsheaded)
{
	int ret;

	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	if (usb_state == AC_IN) {
		return smb345_masked_write(smb345_dev->client,
				CFG_CURRENT_LIMIT,
				CFG_CURRENT_LIMIT_SMB346_MASK,
				CFG_CURRENT_LIMIT_SMB346_VALUE_1200);
	} else if (usb_state == USB_IN) {
	} else if (usb_state == UNKNOWN_IN) {
	}

	return ret;
}

void aicl_dete_worker(struct work_struct *dat)
{
	int aicl_result;
	int usb_state;
	int i;
	int ret;
	int percentage;
#ifdef CONFIG_UPI_BATTERY
	if (smb345_dev) {
		if (!g_Main_mode) {
			if (wake_lock_active(&wlock)) {
				if (!get_battery_rsoc(&percentage)) {
					if (percentage == 100) {
						BAT_DBG(" %s: asus_battery_power_wakelock -> wake unlock\n",
							__func__);
						wake_unlock(&wlock);
					}
				}
			}
		}
	}
#endif
	if (!smb345_dev) {
		pr_err("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return;
	}

	usb_state = g_usb_state;
	/*BSP david: don't need to do any config if no cable!*/
	if (usb_state != AC_IN && usb_state != USB_IN && usb_state != PAD_SUPPLY && usb_state != UNKNOWN_IN)
		return;

	/*BSP David -if AC_IN & AICL result <= 500, do initial config and skip JEITA*/
	if (usb_state == AC_IN) {
		/*BSP David: get AICL results for 10 times since AICL result takes some times to ready*/
		for (i = 0; i < 10; i++) {
			/*get AICL result*/
			aicl_result = get_aicl_results();
			if (aicl_result > 500) {
				break;
			}
			msleep(100);
		}
		/*check if AICL completed here, already delay in config input current phase*/
		ret = smb345_read(smb345_dev, STAT_E);
		if (ret < 0) {
			BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
		} else{
			ret = ret & AUTOMATIC_INPUT_CURR_LIMIT_BIT;
			if (ret > 0) {
				ret = 1;
			} else{
				ret = 0;
			}
		}
		BAT_DBG("%s: aicl result(%dmA) with AICL status:%d\n", __func__, aicl_result, ret);
		/*if AICL result <= 500, then config input current but don't do JEITA*/
		if (aicl_result <= 500) {
			BAT_DBG("%s: don't do JEITA when aicl result(%dmA) <= 500mA.\n", __func__, aicl_result);
			smb345_config_max_current(usb_state);
			goto finish;
		}
	}
	smb346_soc_detect_batt_tempr(usb_state);
finish:
	queue_delayed_work(smb345_dev->chrgr_work_queue,
		&smb345_dev->aicl_dete_work,
		60*HZ);
}
EXPORT_SYMBOL(smb345_update_aicl_work);

static void verifyFW()
{
	if (!smb345_dev) {
		pr_err("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return;
	}

	/* USB Mode Detection (by SOC) */

	/* Get USB to HC mode */
	if (smb345_get_USB9_HC_Toggle() < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to get USB9 and HC mode!\n", __func__);
		return;
	}

	/* Get USB5/1/HC to register control */
	if (smb345_get_USB9_HC_PIN_Control() < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to get USB9 and HC mode pin control!\n", __func__);
		return;
	}


	/* Get I_USB_IN value */
	if (smb345_get_current_limits() < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to get max current limits!\n", __func__);
		return;
	}
}

static int smb358_pre_config()
{
	int ret;

	/* set fast charge current: 900mA */
	ret = smb345_masked_write(smb345_dev->client,
			CHG_CURRENT_REG,
			SMB358_FAST_CHG_CURRENT_MASK,
			SMB358_FAST_CHG_CURRENT_VALUE_1300mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set fast charge current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set termination current: 100mA */
	ret = smb345_masked_write(smb345_dev->client,
			CHG_CURRENT_REG,
			SMB358_TERMINATION_CURRENT_MASK,
			SMB358_TERMINATION_CURRENT_VALUE_100mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set termination current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set cold soft limit current: 600mA */
	ret = smb345_masked_write(smb345_dev->client,
			OTG_TLIM_THERM_CNTRL_REG,
			CHARGE_CURRENT_COMPENSATION,
			SMB358_COLD_SOFT_LIMIT_CURRENT_VALUE_600mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set cold soft limit current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	ret = smb345_charging_toggle(JEITA, false);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to stop charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	ret = smb345_masked_write(smb345_dev->client,
		FLOAT_VOLTAGE_REG,
		FLOAT_VOLTAGE_MASK,
		SMB358_FLOAT_VOLTAGE_VALUE_4340mV);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set charger voltage, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	ret = smb345_charging_toggle(JEITA, true);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to start charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

fail:
	return ret;
}

static void smb3xx_config_max_current(int usb_state)
{
	int ret;
	int i;
	int aicl_result;

	BAT_DBG("%s: usb_state = %d\n", __func__, usb_state);
	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {
		/*BSP david: if I_USB_IN is already 1200mA, don't config
					prevent low battery capacity shutdown when booting*/
		ret = smb345_read(smb345_dev, CFG_CURRENT_LIMIT);
		if (ret < 0) {
			BAT_DBG_E(" %s: fail to read max current limits for USB_IN\n", __func__);
			return;
		}
		ret = ret & CFG_CURRENT_LIMIT_SMB346_MASK;
		if (ret == CFG_CURRENT_LIMIT_SMB346_VALUE_1200) {
			BAT_DBG(" %s: USB_IN is already set to 1200 mA, don't config USB_IN!\n", __func__);
			return;
		}
		/* Disable AICL - Write 02h[4]="0" */
		if (smb345_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			return;
		}

		/* Set I_USB_IN=1200mA - Write 01h[7:4]="0100" */
		if (smb345_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__func__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb345_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}
	} else if (usb_state == USB_IN) {
		/*do nothing when USB_IN, using default setting 500mA*/
	} else if (usb_state == UNKNOWN_IN) {
		/*do nothing when UNKNOWN_IN, using default setting 500mA*/
	}
}

static void smb345_config_max_current(int usb_state)
{
	if (usb_state != AC_IN && usb_state != USB_IN && usb_state != PAD_SUPPLY && usb_state != UNKNOWN_IN)
		return;

	if (!smb345_dev) {
		pr_err("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return;
	}

	/* Allow violate register can be written - Write 30h[7]="1" */
	if (smb345_set_writable(smb345_dev, true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: smb345_set_writable failed!\n", __func__);
		return;
	}
	smb358_pre_config();
	smb3xx_config_max_current(usb_state);
}
extern void update_battery_alarm(void);
int setSMB345Charger(int usb_state)
{
	int ret = -1;
	int result = 0;
	int status = 0;
	int percentage = -1;
#ifdef CONFIG_EC_POWER
	g_pad_cable_state = usb_state;
	if (is_pad_present()) {

		usb_state = PAD_SUPPLY;
		g_usb_state = usb_state;
		BAT_DBG(" %s: Ignore usb_state: %d as PAD_SUPPLY\n",
			__func__, usb_state);
	} else if (g_usb_state != PAD_SUPPLY)
		g_usb_state = usb_state;
#else
	g_usb_state = usb_state;
#endif

	/*BSP david: update alarm interval*/
	update_battery_alarm();
	if (smb345_dev) {
		cancel_delayed_work_sync(&smb345_dev->smb_probe_work);
		switch (usb_state) {
		case USB_IN:
			power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);
			break;
		case AC_IN:
			power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			break;
		case CABLE_OUT:
			power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);
			break;
		case PAD_SUPPLY:
			if (g_pad_cable_state == AC_IN || g_pad_cable_state == CABLE_OUT)
				power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			break;
		case UNKNOWN_IN:
			power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			break;
		}
	}
	ret = get_battery_status(&status);
	if (ret) {
		printk("%s: ret = %d\n", __FUNCTION__, ret);
	} else{
		printk("%s: status = %d\n", __FUNCTION__, status);
	}
	switch (usb_state) {

	case USB_IN:
		BAT_DBG("%s:USB_IN\n", __func__);


		if (smb345_dev) {
			mutex_lock(&g_usb_state_lock);
			/*BSP david : do initial setting & config current*/
			smb345_config_max_current(USB_IN);
			mutex_unlock(&g_usb_state_lock);
			/*BSP david : do JEITA*/
			smb345_update_aicl_work();
		}
		break;
	case AC_IN:
		BAT_DBG("%s: AC_IN\n", __func__);


		if (smb345_dev) {
			mutex_lock(&g_usb_state_lock);
			/*BSP david : do initial setting & config current*/
			smb345_config_max_current(AC_IN);
			mutex_unlock(&g_usb_state_lock);
			/*BSP david : do JEITA*/
			smb345_update_aicl_work();
		}

#ifdef CONFIG_UPI_BATTERY
		if (smb345_dev) {
			if (!g_Main_mode) {
				if (!wake_lock_active(&wlock)) {
					get_battery_rsoc(&percentage);
					if (percentage != 100) {
						BAT_DBG(" %s: asus_battery_power_wakelock -> wake lock\n",
							__func__);

						wake_lock(&wlock);
					}
				}
			}
		}
#endif

#ifdef CONFIG_TOUCHSCREEN_FT3X17
		focal_usb_detection(true);
#endif
		break;

	case CABLE_OUT:
		BAT_DBG("%s: CABLE_OUT\n", __func__);


		if (smb345_dev) {
#ifdef CONFIG_UPI_BATTERY
			if (!g_Main_mode) {
				if (wake_lock_active(&wlock)) {
					BAT_DBG(" %s: asus_battery_power_wakelock -> wake unlock\n",
						__func__);

					/* timeout value as same as the
					<charger.exe>\asus_global.h
					#define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
					*/
					wake_lock_timeout(&wlock_t, 3*HZ);
					wake_unlock(&wlock);
				} else {
					wake_lock_timeout(&wlock_t, 3*HZ);
				}
			}
#endif
		}

#ifdef CONFIG_TOUCHSCREEN_FT3X17
		focal_usb_detection(false);
#endif
		break;

	case ENABLE_5V:
		BAT_DBG("%s: ENABLE_5V\n", __func__);
		result = otg(1);
		break;
	case DISABLE_5V:
		BAT_DBG("%s: DISABLE_5V\n", __func__);
		result = otg(0);
		break;
	case UNKNOWN_IN:
		BAT_DBG("%s: UNKNOWN_IN\n", __func__);

		if (smb345_dev) {
			mutex_lock(&g_usb_state_lock);
			/*BSP david : do initial setting & config current*/
			smb345_config_max_current(UNKNOWN_IN);
			mutex_unlock(&g_usb_state_lock);
			/*BSP david : do JEITA*/
			smb345_update_aicl_work();
		}
#ifdef CONFIG_UPI_BATTERY
		if (smb345_dev) {
			if (!g_Main_mode) {
				if (!wake_lock_active(&wlock)) {
					get_battery_rsoc(&percentage);
					if (percentage != 100) {
						BAT_DBG(" %s: asus_battery_power_wakelock -> wake lock\n",
							__func__);

						wake_lock(&wlock);
					}
				}
			}
		}
#endif
		break;
	default:
		BAT_DBG(" ERROR: wrong usb state value = %d\n", usb_state);
		result = 1;
	}

	return result;
}
EXPORT_SYMBOL(setSMB345Charger);


/* write 06h[6:5]="00" or "11" */
int smb345_charging_toggle(charging_toggle_level_t level, bool on)
{
	int ret = 0;
	int result_toggle;
	static int disabled;
	int reason;
	char *level_str[] = {
		"BALANCE",
		"JEITA",
		"FLAGS",
	};

	if (!smb345_dev) {
		pr_info("Warning: smb345_dev is null "
			"due to probe function has error\n");
		return 1;
	}
	reason = BIT(level);
	if (on == false) {
		disabled |= reason;
	} else{
		disabled &= ~reason;
	}
	result_toggle = !disabled;

	BAT_DBG("%s: current_flag: %d request_lv:%s, request_toggle:%s, result:%s\n",
		__func__,
		disabled,
		level_str[level],
		on ? "ON" : "OFF",
		result_toggle ? "ON" : "OFF");

	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	/* Config CFG_PIN register */

	ret = smb345_read(smb345_dev, CFG_PIN);
	if (ret < 0)
		goto out;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= ~CFG_PIN_EN_CTRL_MASK;
	if (result_toggle) {
		/* set Pin Controls - active low (ME371MG connect EN to GROUND) */
		ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
	} else {
		/* Do nothing, 0 means i2c control
		    . I2C Control - "0" in Command Register disables charger */
	}

	ret = smb345_write(smb345_dev, CFG_PIN, ret);
	if (ret < 0)
		goto out;

	mutex_lock(&g_charging_toggle_lock);
	g_charging_toggle = result_toggle;
	mutex_unlock(&g_charging_toggle_lock);

out:
	return ret;
}

bool external_power_source_present()
{
	return !gpio_get_value(get_gpio_by_name("CHG_INOK#"));
}
#if defined(CONFIG_EC_POWER)
static bool is_pad_present()
{
	bool ret;

	ret = !gpio_get_value(get_gpio_by_name("PAD_PLUG_IN_N"));

	return ret;
}
static bool pad_pwr_supply()
{
	bool ret;

	/* return TRUE if gpio for pad insertion detected
	   and charger received the valid external power
	*/
	ret = !gpio_get_value(get_gpio_by_name("PAD_PLUG_IN_N")) &&
		!gpio_get_value(get_gpio_by_name("CHG_INOK#"));
	return ret;
}
#else
static bool pad_pwr_supply()
{
	return false;
}
static bool is_pad_present()
{
	return false;
}
#endif
static irqreturn_t smb345_inok_interrupt(int irq, void *data)
{
	struct smb345_charger *smb = data;
	int stat_c, irqstat_c, irqstat_d, irqstat_e, irqstat_f;
	irqreturn_t ret = IRQ_NONE;
	int charger_type;

	/* wake lock to prevent system instantly
	   enter S3 while it's in resuming flow */
	wake_lock_timeout(&smb->wakelock, HZ);
	BAT_DBG("%s: wake up system!\n", __FUNCTION__);
	pm_runtime_get_sync(&smb->client->dev);
	if (gpio_get_value(smb->pdata->inok_gpio)) {
		dev_warn(&smb->client->dev,
			"%s: >>> INOK pin (HIGH) <<<\n", __func__);
	} else {
		dev_warn(&smb->client->dev,
			"%s: >>> INOK pin (LOW) <<<\n", __func__);
	}
	pm_runtime_put_sync(&smb->client->dev);
	return IRQ_HANDLED;
}

static int smb346_otg_gpio_init(struct smb345_charger *smb)
{
	const struct smb345_charger_platform_data *pdata = smb->pdata;
	int ret;

	if (pdata->gp_sdio_2_clk < 0) {
		BAT_DBG_E("%s: fail to request CHG_OTG gpio number\n", __func__);
		return -1;
	}

	ret = gpio_request_one(pdata->gp_sdio_2_clk,
			GPIOF_OUT_INIT_LOW,
			"smb346_otg");
	if (ret < 0)
		BAT_DBG_E("%s: request CHG_OTG gpio fail!\n", __func__);

	return ret;
}
static int smb345_inok_gpio_init(struct smb345_charger *smb)
{
	const struct smb345_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->inok_gpio);
	int gpio_usb0_id;

	if (gpio_get_value(pdata->inok_gpio))
		BAT_DBG(">>> INOK (HIGH) <<<\n");
	else
		BAT_DBG(">>> INOK (LOW) <<<\n");

	ret = gpio_request_one(pdata->inok_gpio, GPIOF_IN, "smb345_inok");
	if (ret < 0) {
		BAT_DBG_E("%s: request INOK gpio fail!\n", __func__);
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL, smb345_inok_interrupt,
			IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
			IRQF_ONESHOT |
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			smb->client->name,
			smb);

	if (ret < 0) {
		BAT_DBG_E("%s: config INOK gpio as IRQ fail!\n", __func__);
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	gpio_free(pdata->inok_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}


bool smb345_has_charger_error(void)
{
	int ret, status;

	if (!smb345_dev)
		return -EINVAL;

	ret = smb345_read(smb345_dev, STAT_C);
	if (ret < 0)
		return true;

	if (ret & STAT_C_CHARGER_ERROR)
		return true;

	return false;
}

int smb345_get_charging_status(void)
{
	int ret, status;
	int irqstat_c;

	if (!smb345_dev)
		return -EINVAL;

	ret = smb345_read(smb345_dev, STAT_C);
	if (ret < 0)
		return ret;

	irqstat_c = smb345_read(smb345_dev, IRQSTAT_C);
	if (irqstat_c < 0)
		return irqstat_c;





	if ((ret & STAT_C_CHARGER_ERROR) ||
	(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_C_CHG_TERM) {
			/* set the status to FULL if battery is not in pre
			 * charge, fast charge or taper charging mode AND
			 * charging is terminated at least once.
			 */
			status = POWER_SUPPLY_STATUS_FULL;
		} else {
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!!
			 */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

		if (irqstat_c & IRQSTAT_C_TERMINATION_STAT)
			status = POWER_SUPPLY_STATUS_FULL;
	}

	return status;
}

int smb345_dump_registers(struct seq_file *s)
{
	struct smb345_charger *smb;
	int ret;
	u8 reg;

	if (s) {
		smb = s->private;
	} else {

		if (!smb345_dev) {
			BAT_DBG(" %s: smb345_dev is null!\n",
			__func__);
			return -1;
		} else {

			smb = smb345_dev;
		}
	}

	BAT_DBG(" %s:\n", __func__);
	BAT_DBG(" Control registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");

	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Command registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Command registers:\n");
		seq_printf(s, "==================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}

	ret = smb345_read(smb, CMD_A);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_A, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", CMD_A, BYTETOBINARY(ret));
	ret = smb345_read(smb, CMD_B);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
	ret = smb345_read(smb, CMD_C);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_C, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", CMD_C, BYTETOBINARY(ret));
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Interrupt status registers:\n");
	BAT_DBG(" ===========================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Interrupt status registers:\n");
		seq_printf(s, "===========================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Status registers:\n");
	BAT_DBG(" =================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Status registers:\n");
		seq_printf(s, "=================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}

	return 0;
}

static int smb345_debugfs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");

	smb345_dump_registers(s);

	return 0;
}

static int smb345_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb345_debugfs_show, inode->i_private);
}

static const struct file_operations smb345_debugfs_fops = {
	.open	= smb345_debugfs_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}

static inline int get_battery_temperature(int *tempr)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP_AMBIENT, &val);
	if (!ret)
		*tempr = val.intval;

	return ret;
}

static inline int get_battery_voltage(int *volt)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		*volt = val.intval / 1000;

	return ret;
}
static inline int get_battery_status(int *status)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret)
		*status = val.intval;

	return ret;
}

static inline int get_battery_rsoc(int *rsoc)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret)
		*rsoc = val.intval;

	return ret;
}
enum JEITA_state {
	JEITA_STATE_INITIAL,
	JEITA_STATE_LESS_THAN_15,
	JEITA_STATE_RANGE_15_to_100,
	JEITA_STATE_RANGE_100_to_450,
	JEITA_STATE_RANGE_450_to_600,
	JEITA_STATE_LARGER_THAN_600,
};
int smb346_JEITA_judge_state(int old_State, int batt_tempr)
{
	int ret;
	int result_State;

	/*decide value to set each reg (Vchg, Charging enable, Fast charge current)*/
	/*batt_tempr < 1.5*/
	if (batt_tempr < 15) {
		result_State = JEITA_STATE_LESS_THAN_15;
	/*1.5 <= batt_tempr < 10*/
	} else if (batt_tempr < 100) {
		result_State = JEITA_STATE_RANGE_15_to_100;
	/*10 <= batt_tempr < 45*/
	} else if (batt_tempr < 450) {
		result_State = JEITA_STATE_RANGE_100_to_450;
	/*45 <= batt_tempr < 60*/
	} else if (batt_tempr < 600) {
		result_State = JEITA_STATE_RANGE_450_to_600;
	/*60 <= batt_tempr*/
	} else{
		result_State = JEITA_STATE_LARGER_THAN_600;
	}

	/*BSP david: do 3 degree hysteresis*/
	if (old_State == JEITA_STATE_LESS_THAN_15 && result_State == JEITA_STATE_RANGE_15_to_100) {
		if (batt_tempr <= 45) {
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_15_to_100 && result_State == JEITA_STATE_RANGE_100_to_450) {
		if (batt_tempr <= 130) {
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_450_to_600 && result_State == JEITA_STATE_RANGE_100_to_450) {
		if (batt_tempr >= 420) {
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_LARGER_THAN_600 && result_State == JEITA_STATE_RANGE_450_to_600) {
		if (batt_tempr >= 570) {
			result_State = old_State;
		}
	}
	return result_State;
}
static int smb346_soc_detect_batt_tempr(int usb_state)
{
	int ret;
	static int state = JEITA_STATE_INITIAL;
	int fast_chg_reg_value = 0;/*set fast chg reg value*/
	int Vchg_reg_value = 0;/*set Vchg reg value*/
	bool charging_enable = false;
	drv_status_t drv_sts;
	int batt_tempr = 250;/* unit: C  */
	int batt_volt = 4000;/* unit: mV */
	int chg_volt_reg = 0;/*read reg value*/
	if (usb_state != AC_IN && usb_state != USB_IN && usb_state != PAD_SUPPLY && usb_state != UNKNOWN_IN)
		return 0;
	/*BSP david: do soc control jeita setting*/
	smb345_soc_control_jeita();
	/* acquire battery temperature here */
	ret = get_battery_temperature(&batt_tempr);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __func__);
		return ret;
	} else{
	}

	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __func__);
		return ret;
	} else{
	}
	/* Vchg here */
	ret = smb345_read(smb345_dev, FLOAT_VOLTAGE_REG);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to read charger voltage reg\n", __func__);
		return ret;
	}

	chg_volt_reg = ret & FLOAT_VOLTAGE_MASK;
	BAT_DBG("%s: battery temperature: %d, battery voltage: %d, Vchg: %x\n", __func__, batt_tempr, batt_volt, chg_volt_reg);

	/*BSP david: judge current jeita state, do 3 degree hysteresis*/
	state = smb346_JEITA_judge_state(state, batt_tempr);

	/*decide value to set each reg (Vchg, Charging enable, Fast charge current)*/
	switch (state) {
	case JEITA_STATE_LESS_THAN_15:
		charging_enable = false;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_600mA;
		Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4340mV;
		BAT_DBG("%s: temperature < 1.5\n", __FUNCTION__);
		break;
	case JEITA_STATE_RANGE_15_to_100:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_600mA;
		Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4340mV;
		BAT_DBG("%s: 1.5 <= temperature < 10\n", __FUNCTION__);
		break;
	case JEITA_STATE_RANGE_100_to_450:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4340mV;
		BAT_DBG("%s: 10 <= temperature < 45\n", __FUNCTION__);
		break;
	case JEITA_STATE_RANGE_450_to_600:
		if (chg_volt_reg == SMB358_FLOAT_VOLTAGE_VALUE_4340mV && batt_volt >= 4100) {
			charging_enable = false;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4340mV;
			BAT_DBG("%s: 45 <= temperature < 60, Vchg == 4.34 && Vbat >= 4.1\n", __FUNCTION__);
		} else{
			charging_enable = true;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4100mV;
			BAT_DBG("%s: 45 <= temperature < 60, Vchg != 4.34 || Vbat < 4.1\n", __FUNCTION__);
		}
		break;
	case JEITA_STATE_LARGER_THAN_600:
		charging_enable = false;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4340mV;
		BAT_DBG("%s: 60 <= temperature\n", __FUNCTION__);
		break;
	default:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4340mV;
		BAT_DBG("%s: wrong state!\n", __FUNCTION__);
		break;
	}
	/*BSP david: do 5060 when FACTORY BUILD defined*/
#if defined(ASUS_FACTORY_BUILD)
	if (charging_enable) {
		charging_enable = asus_battery_charging_limit();
	} else{
		BAT_DBG_E("%s: charging disabled by JEITA!\n", __FUNCTION__);
	}
#endif

	/*do set reg value after depend on above decision*/
	/*set enable charging reg*/
	ret = smb345_charging_toggle(JEITA, charging_enable);
	if (ret) {
		pr_err("fail to set charging enable ret=%d\n", ret);
		goto finish;
	}
	/*Set Vchg*/
	ret = smb345_masked_write(smb345_dev->client,
		FLOAT_VOLTAGE_REG,
		FLOAT_VOLTAGE_MASK,
		Vchg_reg_value);
	if (ret) {
		pr_err("fail to set Vchg ret=%d\n", ret);
		goto finish;
	}
	/*Set Fast charge current*/
	ret = smb345_masked_write(smb345_dev->client,
		CHG_CURRENT_REG,
		SMB358_FAST_CHG_CURRENT_MASK,
		fast_chg_reg_value);
	if (ret) {
		pr_err("fail to set Vchg ret=%d\n", ret);
		goto finish;
	}
finish:
	request_power_supply_changed();
	return ret;
}


/*+++BSP David Charger Proc Data Structure+++*/
struct ChargerReg_command{
	u8 addr;
	enum readwrite{
		E_READ = 0,
		E_WRITE = 1,
		E_READWRITE = 2,
		E_NOUSE = 3,
	} rw;
};

struct ChargerReg_command ChargerReg_CMD_Table[] = {
	{0x00,	E_READWRITE},
	{0x01,	E_READWRITE},
	{0x02,	E_READWRITE},
	{0x03,	E_READWRITE},
	{0x06,	E_READWRITE},
	{0x07,	E_READWRITE},
	{0x0a,	E_READWRITE},
	{0x0b,	E_READWRITE},
	{0x30,	E_READWRITE},
	{0x31,	E_READWRITE},
	{0x35,	E_READ},
	{0x36,	E_READ},
	{0x37,	E_READ},
	{0x38,	E_READ},
	{0x39,	E_READ},
	{0x3a,	E_READ},
	{0x3b,	E_READ},
	{0x3c,	E_READ},
	{0x3d,	E_READ},
	{0x3e,	E_READ},
	{0x3f,	E_READ},
};
/*---BSP David Charger Proc Data Structure---*/
/*+++BSP david: charger register dump proc+++*/

static int ChargerReg_proc_read_By_Table(int cmd)
{
	int Result = -1;
	if (cmd >= 0 && cmd < ARRAY_SIZE(ChargerReg_CMD_Table)) {
		if (E_READ == ChargerReg_CMD_Table[cmd].rw || E_READWRITE == ChargerReg_CMD_Table[cmd].rw) {
			if (smb345_dev) {
				Result = smb345_read(smb345_dev, ChargerReg_CMD_Table[cmd].addr);
			} else{
				/*do nothing*/
			}
		} else{
			/*do nothing*/
		}
	} else{
		/*do nothing*/
	}
	return Result;
}
static ssize_t ChargerRegDump_read_proc(char *page, char **start, off_t off, int count,
		int *eof, void *data)
{
	int i = 0;
	int Charger_reg_value = -1;
	char tempString[512] = "";
	for (i = 0; i < ARRAY_SIZE(ChargerReg_CMD_Table); i++) {
		Charger_reg_value = ChargerReg_proc_read_By_Table(i);
		snprintf(tempString, sizeof(tempString), "%s0x%02x=%d%d%d%d%d%d%d%d\n", tempString, ChargerReg_CMD_Table[i].addr, BYTETOBINARY(Charger_reg_value));
	}
	return seq_printf(page, "%s", tempString);
}

static ssize_t ChargerRegDump_write_proc(struct file *filp, const char __user *buff,
		unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	printk("[BAT][CHG][SMB][Proc]%s: %d\n", __FUNCTION__, val);

	return len;
}

static int ChargerRegDump_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ChargerRegDump_read_proc, NULL);
}

static const struct file_operations ChargerRegDump_fops = {
	.owner = THIS_MODULE,
	.open =  ChargerRegDump_proc_open,
	.write = ChargerRegDump_write_proc,
	.read = seq_read,
};

static void create_ChargerRegDump_proc_file(void)
{
	struct proc_dir_entry *ChargerReg_proc_file = proc_create("driver/Charger_Reg_Dump", 0666, NULL, &ChargerRegDump_fops);

	if (ChargerReg_proc_file) {
		BAT_DBG("[Proc]%s sucessed!\n", __FUNCTION__);
	} else{
		BAT_DBG("[Proc]%s failed!\n", __FUNCTION__);
	}

	return;
}
/*---BSP david: charger register dump proc---*/
#if defined(ASUS_FACTORY_BUILD)
/*+++BSP David proc charger_limit_enable Interface+++*/
static ssize_t charger_limit_enable_proc_read(struct seq_file *buf, void *v)
{
	if (eng_charging_limit) {
		seq_printf(buf, "charging limit enable\n");
	} else{
		seq_printf(buf, "charging limit disable\n");
	}
	return 0;
}

static ssize_t charger_limit_enable_proc_write(struct file *filp, const char __user *buff,
		unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	if (buff[0] == '1') {
		eng_charging_limit = true;
		g_charging_toggle_for_charging_limit = true;
		/* turn on charging limit in eng mode */
		printk("[BAT][CHG][SMB][Proc]charger_limit_enable:%d\n", 1);
	} else if (buff[0] == '0') {
		eng_charging_limit = false;
		g_charging_toggle_for_charging_limit = true;
		/* turn off charging limit in eng mode */
		printk("[BAT][CHG][SMB][Proc]charger_limit_enable:%d\n", 0);
	}
	/*BSP david: update charging status*/
	smb345_update_aicl_work();

	return len;
}

static int charger_limit_enable_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, charger_limit_enable_proc_read, NULL);
}

static const struct file_operations charger_limit_enable_fops = {
	.owner = THIS_MODULE,
	.open =  charger_limit_enable_proc_open,
	.write = charger_limit_enable_proc_write,
	.read = seq_read,
};

static void create_charger_limit_enable_proc_file(void)
{
	struct proc_dir_entry *charger_limit_enable_proc_file = proc_create("driver/charger_limit_enable", 0666, NULL, &charger_limit_enable_fops);

	if (charger_limit_enable_proc_file) {
		printk("[BAT][CHG][SMB][Proc]charger_limit_enable create ok!\n");
	} else{
		printk("[BAT][CHG][SMB][Proc]charger_limit_enable create failed!\n");
	}
	return;
}
#endif
/*---BSP David proc charger_limit_enable Interface---*/
/*+++BSP David BMMI Adb Interface+++*/
#define	chargerIC_status_PROC_FILE	"driver/chargerIC_status"
static struct proc_dir_entry *chargerIC_status_proc_file;
static int chargerIC_status_proc_read(struct seq_file *buf, void *v)
{
	int ret = -1;
	ret = smb345_read(smb345_dev, 0x01);
	if (ret > 0) {
		ret = 1;
	} else{
		ret = 0;
	}
	seq_printf(buf, "%d\n", ret);
	return 0;
}
static int chargerIC_status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, chargerIC_status_proc_read, NULL);
}
static ssize_t chargerIC_status_proc_write(struct file *filp, const char __user *buff,
	unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	printk("[BAT][CHG][SMB][Proc]ChargerIC Proc File: %d\n", val);

	return len;
}

static const struct file_operations chargerIC_status_fops = {
	.owner = THIS_MODULE,
	.open = chargerIC_status_proc_open,
	.write = chargerIC_status_proc_write,
	.read = seq_read,
};
void static create_chargerIC_status_proc_file(void)
{
	chargerIC_status_proc_file = proc_create(chargerIC_status_PROC_FILE, 0644, NULL, &chargerIC_status_fops);

	if (chargerIC_status_proc_file) {
		BAT_DBG("[Proc]%s sucessed!\n", __FUNCTION__);
	} else{
		BAT_DBG("[Proc]%s failed!\n", __FUNCTION__);
	}
}
/*---BSP David BMMI Adb Interface---*/

#ifdef CONFIG_I2C_STRESS_TEST
static int TestSmb345ChargerReadWrite(struct i2c_client *client)
{

	int status;
	u8 reg;
	i2c_log_in_test_case("[BAT][CHG][SMB][Test]Test Smb345Charger start\n");
	 
	status = smb345_read_reg(smb345_dev->client, CMD_A, &reg, 0);
	if (status < 0) {
		i2c_log_in_test_case("[BAT][CHG][SMB][Test]Test Smb345Charger end: status = %d\n", status);
		return I2C_TEST_SMB345_FAIL;
	}
	reg |= CMD_A_ALLOW_WRITE;
	status = smb345_write_reg(smb345_dev->client, CMD_A, reg);
	if (status < 0) {
		i2c_log_in_test_case("[BAT][CHG][SMB][Test]Test Smb345Charger end: status = %d\n", status);
		return I2C_TEST_SMB345_FAIL;
	}

	i2c_log_in_test_case("[BAT][CHG][SMB][Test]Test Smb345Charger end: data = %x\n", reg);
	return I2C_TEST_PASS;
};
static struct i2c_test_case_info gChargerTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestSmb345ChargerReadWrite),
};
static void smb345_i2c_stress_test()
{
	BAT_DBG("%s\n", __FUNCTION__);
	i2c_add_test_case(smb345_dev->client, "Smb345Charger", ARRAY_AND_SIZE(gChargerTestCaseInfo));
}
#endif
void smb345_update_aicl_work()
{
	if (smb345_dev) {
		cancel_delayed_work(&smb345_dev->aicl_dete_work);
		queue_delayed_work(smb345_dev->chrgr_work_queue,
			&smb345_dev->aicl_dete_work,
			0);
	}
}
static int smb345_routine_aicl_control(struct smb345_charger *smb)
{
	BAT_DBG(" %s\n", __FUNCTION__);

	INIT_DELAYED_WORK(&smb->aicl_dete_work, aicl_dete_worker);
	smb->chrgr_work_queue = create_singlethread_workqueue("smb358_wq");
	if (!(smb->chrgr_work_queue)) {
		BAT_DBG_E(" fail to create smb358_wq\n");
		return -ENOMEM;
	}
	queue_delayed_work(smb->chrgr_work_queue,
			&smb->aicl_dete_work,
			60*HZ);

	return 0;
}
extern bool g_upi_probe_done;
void smb_probe_worker(struct work_struct *dat)
{
	int charger_type;
	BAT_DBG("%s++\n", __func__);
	if (smb345_dev) {
		if (g_upi_probe_done) {
			mutex_lock(&g_usb_state_lock);
			charger_type = g_usb_state;
			dev_warn(&smb345_dev->client->dev,
					"%s: config current when probe\n", __func__);
			smb345_config_max_current(charger_type);
			mutex_unlock(&g_usb_state_lock);
			smb345_update_aicl_work();
		} else{
			schedule_delayed_work(&smb345_dev->smb_probe_work, HZ);
		}
	} else{
		BAT_DBG_E("%s: charger initial fail!\n", __func__);
	}
}
static int smb345_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct smb345_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb345_charger *smb;
	int ret;
	int charger_type;

	BAT_DBG_E(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);
	printk("[Progress][smb358] Probe starts\n");

	pdata = dev->platform_data;
	if (!pdata)
		return -EINVAL;

	if (!pdata->use_mains && !pdata->use_usb)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);

	smb->client = client;
	smb->pdata = pdata;

#ifdef CONFIG_UPI_BATTERY
	/* init wake lock in COS */
	if (!g_Main_mode) {
		BAT_DBG(" %s: wake lock init: asus_battery_power_wakelock\n",
			__func__);
		wake_lock_init(&wlock,
			WAKE_LOCK_SUSPEND,
			"asus_battery_power_wakelock");
		wake_lock_init(&wlock_t,
			WAKE_LOCK_SUSPEND,
			"asus_battery_power_wakelock_timeout");

		/* prevent system from entering s3 in COS
		    while AC charger is connected
		*/
		charger_type = g_usb_state;

		if (charger_type == AC_IN || charger_type == UNKNOWN_IN) {
			if (!wake_lock_active(&wlock)) {
				BAT_DBG(" %s: asus_battery_power_wakelock "
					"-> wake lock\n", __func__);
				wake_lock(&wlock);
			}
		}
	}
#endif
	smb345_routine_aicl_control(smb);
	INIT_DELAYED_WORK(&smb->smb_probe_work, smb_probe_worker);
	smb345_dev = smb;
	smb345_dump_registers(NULL);

	/* enable register writing - chris */
	ret = smb345_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/*BSP David create ATD intterface proc*/
	create_chargerIC_status_proc_file();
	create_ChargerRegDump_proc_file();
#if defined(ASUS_FACTORY_BUILD)
	eng_charging_limit = false;
	g_charging_toggle_for_charging_limit = true;
	create_charger_limit_enable_proc_file();
#endif
#ifdef CONFIG_I2C_STRESS_TEST
	smb345_i2c_stress_test();
#endif
	/* Refer to SMB345 Application Note 72 to solve serious problems */
	ret = smb345_masked_write(smb->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK,
				OTG_CURRENT_LIMIT_250mA);
	if (ret < 0)
		return ret;

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&smb->client->dev);
	pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);

	/* INOK pin configuration */
	if (pdata->inok_gpio >= 0) {
		ret = smb345_inok_gpio_init(smb);
		if (ret < 0) {
			dev_warn(dev,
				"fail to initialize INOK gpio: %d\n",
				ret);
		}
	}
	/* OTG pin configuration */
	ret = smb346_otg_gpio_init(smb);
	if (ret < 0)
		return ret;

	smb->running = true;
	smb->dentry = debugfs_create_file("smb", S_IRUGO, NULL, smb,
			&smb345_debugfs_fops);
	sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

	ret = smb345_register_power_supply(&client->dev);
	if (ret < 0)
		return ret;

	BAT_DBG_E(" ++++++++++++++++ %s done ++++++++++++++++\n", __func__);

	/* init wake lock */
	wake_lock_init(&smb->wakelock,
			WAKE_LOCK_SUSPEND, "smb345_wakelock");

	/* ME372CG charge current control algorithm:
	   config the charge current only when
	   Vbus is legal (a valid input voltage
	   is present)
	*/

	verifyFW();
	mutex_lock(&g_usb_state_lock);
	charger_type = g_usb_state;
	mutex_unlock(&g_usb_state_lock);
	if (charger_type == AC_IN || charger_type == USB_IN || charger_type == PAD_SUPPLY || charger_type == UNKNOWN_IN) {
		dev_warn(&smb345_dev->client->dev,
				"%s: start probe work: charger type = %d\n", __func__, charger_type);
		schedule_delayed_work(&smb->smb_probe_work, 0);
	}
	generate_key();
	printk("[Progress][smb358] Probe ends\n");
	return 0;
}

static int smb345_remove(struct i2c_client *client)
{
	struct smb345_charger *smb = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	smb->running = false;

	wake_lock_destroy(&smb->wakelock);
#ifdef CONFIG_UPI_BATTERY
	if (!g_Main_mode) {
		wake_lock_destroy(&wlock);
		wake_lock_destroy(&wlock_t);
	}
#endif
	pm_runtime_get_noresume(&smb->client->dev);

	return 0;
}

static int smb345_shutdown(struct i2c_client *client)
{
	dev_info(&client->dev, "%s\n", __func__);





	/* JEITA function by charger protection */
	if (g_usb_state == AC_IN || g_usb_state == USB_IN || g_usb_state == PAD_SUPPLY || g_usb_state == UNKNOWN_IN)
		smb345_charger_control_jeita();

	/* Disable OTG during shutdown */
	otg(0);

	/* registers dump */
	smb345_dump_registers(NULL);

	return 0;
}

#ifdef CONFIG_PM
static int smb345_prepare(struct device *dev)
{
	struct smb345_charger *smb = dev_get_drvdata(dev);

	dev_info(&smb->client->dev, "smb345 suspend\n");
	/*
	 * disable irq here doesn't mean smb345 interrupt
	 * can't wake up system. smb345 interrupt is triggered
	 * by GPIO pin, which is always active.
	 * When resume callback calls enable_irq, kernel
	 * would deliver the buffered interrupt (if it has) to
	 * driver.
	 */
	if (smb->client->irq > 0)
		disable_irq(smb->client->irq);

	/* JEITA function by charger protection */
	if (g_usb_state == AC_IN || g_usb_state == USB_IN || g_usb_state == PAD_SUPPLY || g_usb_state == UNKNOWN_IN)
		smb345_charger_control_jeita();

	return 0;
}

static void smb345_complete(struct device *dev)
{
	struct smb345_charger *smb = dev_get_drvdata(dev);

	dev_info(&smb->client->dev, "smb345 resume\n");
}
#else
#define smb345_prepare NULL
#define smb345_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb345_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb345_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb345_runtime_idle(struct device *dev)
{

	dev_info(dev, "%s called\n", __func__);
	return 0;
}
#else
#define smb345_runtime_suspend    NULL
#define smb345_runtime_resume    NULL
#define smb345_runtime_idle    NULL
#endif

static const struct i2c_device_id smb345_id[] = {
	{"smb358", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb345_id);

static const struct dev_pm_ops smb345_pm_ops = {
	.prepare        = smb345_prepare,
	.complete        = smb345_complete,
	.runtime_suspend    = smb345_runtime_suspend,
	.runtime_resume        = smb345_runtime_resume,
	.runtime_idle        = smb345_runtime_idle,
};

static struct i2c_driver smb345_driver = {
	.driver = {
		.name    = "smb358",
		.owner    = THIS_MODULE,
		.pm    = &smb345_pm_ops,
	},
	.probe        = smb345_probe,
	.remove        = smb345_remove,
	.shutdown    = smb345_shutdown,
	.id_table    = smb345_id,
};

static int __init smb345_init(void)
{

	BAT_DBG_E(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);

	HW_ID = Read_HW_ID();
	return i2c_add_driver(&smb345_driver);
}
module_init(smb345_init);

static void __exit smb345_exit(void)
{
	i2c_del_driver(&smb345_driver);
}
module_exit(smb345_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_AUTHOR("Chris Chang <chris1_chang@asus.com>");
MODULE_DESCRIPTION("SMB345 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb345");

