#ifndef _LINUX_PM_UTIL_H
#define _LINUX_PM_UTIL_H

/* [ASUS BSP] Cheryl Chen - PF450CL001S - Add Power On/Off & Suspend/Resume Log */
typedef enum {
	EPMTestTypeSuspend,
	EPMTestTypeSuspendDevices,
	EPMTestTypeResume,
	EPMTestTypePowerKeyResume,
	EPMTestTypeShutdown,
	EPMTestTypeTotal
} EPMTestType;

typedef enum {
	EResumeTypeDisplayOn,
	EResumeTypeLateResume,
	EResumeTypeTotal
} EResumeType;

typedef enum {
	WAKE_BATT_INSERT = 0,
	WAKE_PWR_BUTTON_PRESS,
	WAKE_RTC_TIMER,
	WAKE_USB_CHRG_INSERT,
	WAKE_LOWBAT_THRESHOLD,
	WAKE_REAL_RESET,
	WAKE_COLD_BOOT,
	WAKE_UNKNOWN,
	WAKE_WDT_RESET,
	WAKE_SEC_WDT_RESET,
	WAKE_WATCHDOG_COUNTER_EXCEEDED,
	WAKE_POWER_SUPPLY_DETECTED,
	WAKE_FASTBOOT_BUTTONS_COMBO,
	WAKE_NO_MATCHING_OSIP_ENTRY,
	WAKE_CRITICAL_BATTERY,
	WAKE_INVALID_CHECKSUM,
	WAKE_EXT_RESET,
	WAKE_ADAPTER_INSERT,
	WAKE_TOTAL
} WakeSource;

extern const char DBootUpReason[WAKE_TOTAL][50];

extern void PMTestStart(EPMTestType eType);
extern void PMTestFinish(EPMTestType eType, const char *szLabel);
extern void PMTestResumeFinish(char *szNote);

extern void SetResumeStarted(bool bStarted);
extern bool GetResumeStarted();
extern bool SetResumeState(EResumeType eType, bool bDone);
extern bool GetResumeState(EResumeType eType);
extern void SetFakeResume(bool bFakeResume);
extern bool GetFakeResume();

extern void SetSuspendStarted(bool bStarted);
extern bool GetSuspendStarted();
extern void SetPowerKeyResumeStarted(bool bStarted);
extern bool GetPowerKeyResumeStarted();

extern bool GetFakeResume();

extern void SetLastIRQ(int nIRQ);
extern int GetWakeUpIRQ();
extern void SetGPIOInfo(int nGPIO);
extern int GetGPIOInfo();
/* [ASUS BSP] Cheryl Chen - PF450CL001E */

/* [ASUS BSP] Cheryl Chen - PF450CL013S - Constrain the Log Number of Active Wake Lock in ASUSEvtLog */
#define DASUSEvtLogWakelockPrintTimesLimit	2
extern unsigned int g_nASUSEvtLogWakelockPrintTimes;
/* [ASUS BSP] Cheryl Chen - PF450CL013E */
extern char evtlog_bootup_reason[50];

extern bool GetDisplayPowerOnWithoutFrame();
extern void SetDisplayPowerOnWithoutFrame(bool bWithoutFrame);

#endif /* _LINUX_PM_UTIL_H */
