#ifndef _SMB345_EXTERNAL_INCLUDE_H
#define _SMB345_EXTERNAL_INCLUDE_H 1

#include <linux/seq_file.h>
enum {
	USB_IN,
	AC_IN,
	CABLE_OUT,
	ENABLE_5V,
	DISABLE_5V,
	PAD_SUPPLY,
/* ASUS_BSP+++ LandiceFu "[ZE500CL][USBH][NA][Spec] Support unknown charger" */
	UNKNOWN_IN,
/* ASUS_BSP--- LandiceFu "[ZE500CL][USBH][NA][Spec] Support unknown charger" */
};

typedef enum {
	BALANCE = 0,
	JEITA,
	FLAGS,
} charging_toggle_level_t;

/* This function is exported to external.
 * usb_state should be one of the above
 *
 * return 0 means success */
extern int setSMB345Charger(int usb_state);
extern int request_power_supply_changed(void);
extern int get_charger_type(void);
extern bool get_sw_charging_toggle(void);
extern void smb345_update_aicl_work(void);
extern bool external_power_source_present(void);

/* To know the charging status
 *
 * return true when charger is charging */
extern int smb345_get_charging_status(void);

/* To enable/disable charging
 *
 * return 0 means success */
extern int smb345_charging_toggle(charging_toggle_level_t level, bool on);

/* To know if charger has an error
 *
 * return true means charger has an error */
bool smb345_has_charger_error(void);

/* To acquire the value from charger ic registers
 *
 * return nothing */
int smb345_dump_registers(struct seq_file *s);

int pmic_get_batt_id(void);

/* To config hard/soft limit cell temperature
 * monitor for soc control JEITA function
 *
 * return 0 means config completely */
int smb345_soc_control_jeita(void);

/* To config hard/soft limit cell temperature
 * monitor for charger control JEITA function
 *
 * return 0 means config completely */
int smb345_charger_control_jeita(void);

#define LG "1"
#define COSLIGHT "0"
#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
#define SMB345_DEV_NAME "smb346"
#else
#define SMB345_DEV_NAME "smb345"
#endif
#define MSIC_CHRG_REG_DUMP_OTHERS	(1 << 3)

/* battery temperature unit: 0.1C */
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD         500
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD_PF400CG 450

#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  600
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 450
#elif defined(CONFIG_ME175CG)
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  550
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 500
#else
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  550
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 500
#endif

#endif /* _SMB345_EXTERNAL_INCLUDE_H */

