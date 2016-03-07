/* [ASUS BSP] Cheryl Chen - PF450CL001S - Add Power On/Off & Suspend/Resume Log */
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/PMUtil.h>

#define TEST_SUSPEND_SECONDS   10
static unsigned long g_ulPMTestStartTime[EPMTestTypeTotal];

const char DBootUpReason[WAKE_TOTAL][50] = {
	"WAKE_BATT_INSERT",
	"WAKE_PWR_BUTTON_PRESS",
	"WAKE_RTC_TIMER",
	"WAKE_USB_CHRG_INSERT",
	"WAKE_LOWBAT_THRESHOLD",
	"WAKE_REAL_RESET",
	"WAKE_COLD_BOOT",
	"WAKE_UNKNOWN",
	"WAKE_WDT_RESET",
	"WAKE_SEC_WDT_RESET",
	"WAKE_WATCHDOG_COUNTER_EXCEEDED",
	"WAKE_POWER_SUPPLY_DETECTED",
	"WAKE_FASTBOOT_BUTTONS_COMBO",
	"WAKE_NO_MATCHING_OSIP_ENTRY",
	"WAKE_CRITICAL_BATTERY",
	"WAKE_INVALID_CHECKSUM",
	"WAKE_EXT_RESET",
	"WAKE_ADAPTER_INSERT"
};

void PMTestStart(EPMTestType eType)
{
	/* FIXME Use better timebase than "jiffies", ideally a clocksource.
	 * 	 * What we want is a hardware counter that will work correctly even
	 * 	 	 * during the irqs-are-off stages of the suspend/resume cycle...
	 * 	 	 	 */
	g_ulPMTestStartTime[eType] = jiffies;
}
EXPORT_SYMBOL(PMTestStart);

void PMTestFinish(EPMTestType eType, const char *szLabel)
{
	long nj = jiffies - g_ulPMTestStartTime[eType];
	unsigned msec;

	msec = jiffies_to_msecs(abs(nj));
	pr_info("[PM] %s took %d.%03d seconds\n", szLabel, msec / 1000, msec % 1000);
}
EXPORT_SYMBOL(PMTestFinish);

void PMTestResumeFinish(char *szNote)
{
	char szLog[100] = "Resume";
	EPMTestType eType = EPMTestTypeResume;

	if (GetFakeResume())
		snprintf(szLog, sizeof(szLog), "Fake Resume");
	if (GetPowerKeyResumeStarted()) {
		eType = EPMTestTypePowerKeyResume;
		snprintf(szLog, sizeof(szLog), "%s - Power Key", szLog);
	}
	snprintf(szLog, sizeof(szLog), "%s (%s)", szLog, szNote);
	PMTestFinish(eType, szLog);
}
EXPORT_SYMBOL(PMTestResumeFinish);

static int g_bResumeStarted;
void SetResumeStarted(bool bStarted)
{
	g_bResumeStarted = bStarted;
	if (bStarted) {
		SetResumeState(EResumeTypeDisplayOn, false);
		SetResumeState(EResumeTypeLateResume, false);
	} else {
		SetResumeState(EResumeTypeDisplayOn, true);
		SetResumeState(EResumeTypeLateResume, true);
	}
}
EXPORT_SYMBOL(SetResumeStarted);
bool GetResumeStarted()
{
	return g_bResumeStarted;
}
EXPORT_SYMBOL(GetResumeStarted);

static int g_bResumeState[EResumeTypeTotal];
bool SetResumeState(EResumeType eType, bool bDone)
{
	g_bResumeState[eType] = bDone;
}
EXPORT_SYMBOL(SetResumeState);

bool GetResumeState(EResumeType eType)
{
	return g_bResumeState[eType];
}
EXPORT_SYMBOL(GetResumeState);

bool g_bFakeResume;
void SetFakeResume(bool bFakeResume)
{
	g_bFakeResume = bFakeResume;
	if (bFakeResume) {
		SetResumeState(EResumeTypeDisplayOn, false);
		SetResumeState(EResumeTypeLateResume, false);
	}
}
bool GetFakeResume()
{
	return g_bFakeResume;
}
EXPORT_SYMBOL(GetFakeResume);

bool g_bSuspendStarted;
void SetSuspendStarted(bool bStarted)
{
	g_bSuspendStarted = bStarted;
}
EXPORT_SYMBOL(SetSuspendStarted);
bool GetSuspendStarted()
{
	return g_bSuspendStarted;
}
EXPORT_SYMBOL(GetSuspendStarted);

bool g_bPowerKeyResumeStarted;
void SetPowerKeyResumeStarted(bool bStarted)
{
	g_bPowerKeyResumeStarted = bStarted;
}
EXPORT_SYMBOL(SetPowerKeyResumeStarted);
bool GetPowerKeyResumeStarted()
{
	return g_bPowerKeyResumeStarted;
}
EXPORT_SYMBOL(GetPowerKeyResumeStarted);

static int g_nWakeUpIRQ[2];
void SetLastIRQ(int nIRQ)
{
	g_nWakeUpIRQ[1] = g_nWakeUpIRQ[0];
	g_nWakeUpIRQ[0] = nIRQ;
}
EXPORT_SYMBOL(SetLastIRQ);
int GetWakeUpIRQ()
{
	int nResult = g_nWakeUpIRQ[1];
	g_nWakeUpIRQ[1] = -1;
	return nResult;
}
EXPORT_SYMBOL(GetWakeUpIRQ);

static int g_nGPIO;
void SetGPIOInfo(int nGPIO)
{
	g_nGPIO = nGPIO;
}
EXPORT_SYMBOL(SetGPIOInfo);
int GetGPIOInfo()
{
	int nResult = g_nGPIO;
	g_nGPIO = -1;
	return nResult;
}
EXPORT_SYMBOL(GetGPIOInfo);
/* [ASUS BSP] Cheryl Chen - PF450CL001E */
/* ZE500CL001S */
static bool g_bDisplayPowerOnWithoutFrame;
void SetDisplayPowerOnWithoutFrame(bool bWithoutFrame)
{
	g_bDisplayPowerOnWithoutFrame = bWithoutFrame;
}
bool GetDisplayPowerOnWithoutFrame()
{
	return g_bDisplayPowerOnWithoutFrame;
}
/* ZE500CL001E */