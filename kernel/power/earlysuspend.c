/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
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

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/writeback.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>

#include "power.h"

/* ZE500CL001S */
#include <linux/PMUtil.h>
#include <linux/switch.h>

extern bool g_resume_status;
extern struct timer_list check_wakeup_source_timer;
extern unsigned long print_period;

/* Add a timer to trigger wakelock debug */
extern struct timer_list unattended_timer;

struct work_struct pms_printer;
struct work_struct pm_wakeup_source_printer;
static struct switch_dev pmsp_dev;
static struct switch_dev pm_wakeup_source_dev;

void pmsp_print(void)
{
    schedule_work(&pms_printer);
    return;
}

EXPORT_SYMBOL(pmsp_print);

void print_pm_wakeup_source(void)
{
    schedule_work(&pm_wakeup_source_printer);
    return;
}
EXPORT_SYMBOL(print_pm_wakeup_source);
void pms_printer_func(struct work_struct *work)
{
	static int pmsp_counter;

	if (pmsp_counter % 2) {
		printk("%s: Enter pmsprinter ready to send uevent 0 \n", __func__);
		switch_set_state(&pmsp_dev, 0);
		pmsp_counter++;
	} else {
		printk("%s: Enter pmsprinter ready to send uevent 1 \n", __func__);
		switch_set_state(&pmsp_dev, 1);
		pmsp_counter++;
	}
}

void pm_wakeup_source_func(struct work_struct *work)
{
	static bool toggle;

	toggle = !toggle;
	printk("%s: Dump PowerManagerService wakelocks, toggle %d\n", __func__, toggle ? 1 : 0);
	switch_set_state(&pm_wakeup_source_dev, toggle);
}
/* ZE500CL001E */

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static DEFINE_MUTEX(suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static void try_to_suspend(struct work_struct *work);
static struct workqueue_struct *early_suspend_wq;
static struct workqueue_struct *suspend_wq;
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DECLARE_WORK(suspend_work, try_to_suspend);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;
static int suspend_state;
static struct wakeup_source *early_suspend_ws;

void queue_up_early_suspend_work(struct work_struct *work)
{
	queue_work(early_suspend_wq, work);
}

static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;

	if (!pm_get_wakeup_count(&initial_count, true))
		goto queue_again;

	mutex_lock(&suspend_lock);

	if (!pm_save_wakeup_count(initial_count) ||
		system_state != SYSTEM_RUNNING) {
		mutex_unlock(&suspend_lock);
		goto queue_again;
	}

	if (suspend_state == PM_SUSPEND_ON) {
		mutex_unlock(&suspend_lock);
		return;
	}

	/* ZE500CL001S */
	printk("[PM][Suspend +++]\n");
	/* ZE500CL001E */

	if (suspend_state >= PM_SUSPEND_MAX)
		hibernate();
	else
		pm_suspend(suspend_state);

	/* ZE500CL001S */
	printk("[PM][Resume ---]\n");
	/* ZE500CL001E */

	mutex_unlock(&suspend_lock);

	if (!pm_get_wakeup_count(&final_count, false))
		goto queue_again;

	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count)
		schedule_timeout_uninterruptible(HZ / 2);

queue_again:
	queue_work(suspend_wq, &suspend_work);
}

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL)
			pos->suspend(pos);
	}
	mutex_unlock(&early_suspend_lock);

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: after call handlers\n");
	/* just wake up flusher to start write back and don't wait it finished*/
	wakeup_flusher_threads(0, WB_REASON_SYNC);
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		__pm_relax(early_suspend_ws);
	spin_unlock_irqrestore(&state_lock, irqflags);
	queue_work(suspend_wq, &suspend_work);
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link)
		if (pos->resume != NULL)
			pos->resume(pos);
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
}

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;
	suspend_state_t prev_state;

	mutex_lock(&suspend_lock);
	prev_state = suspend_state;
	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("[PM] request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			prev_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
		/* +++ Fixed format for log parser, DO NOT MODIFY +++ */
		ASUSEvtlog("[PM]request_suspend_state: (%d->%d)\n", prev_state, new_state);
		/* --- Fixed format for log parser, DO NOT MODIFY --- */
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		/* ZE500CL001S */
		PMTestStart(EPMTestTypeSuspend);
		SetSuspendStarted(true);
		SetResumeStarted(false);
		g_resume_status = false;
		/* Add a timer to trigger wakelock debug */
		printk("[PM] unattended_timer: mod_timer (early_suspend)\n");

		mod_timer(&unattended_timer, jiffies + msecs_to_jiffies(PM_UNATTENDED_TIMEOUT));

		if (!timer_pending(&check_wakeup_source_timer)) {
			mod_timer(&check_wakeup_source_timer, jiffies + msecs_to_jiffies(WAIT_FOR_PRINT_WAKEUP_SOURCE));
			printk("[PM] check_wakeup_source_timer: mod_timer (early_suspend)\n");
		}
		/* ZE500CL001E */
		state |= SUSPEND_REQUESTED;
		queue_up_early_suspend_work(&early_suspend_work);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		/* ZE500CL001S */
		if (!GetResumeStarted()) {
			printk("[PM] Resume from fake suspend\n");
			PMTestStart(EPMTestTypeResume);
			SetFakeResume(true);
		} else
			SetFakeResume(false);
		printk("[PM] unattended_timer: del_timer (late_resume)\n");
		del_timer(&unattended_timer);
		/* Cheryl Chen - PF450CL013S - Constrain the Log Number of Active Wake Lock in ASUSEvtLog */
		g_nASUSEvtLogWakelockPrintTimes = 0;
		/* Cheryl Chen - PF450CL013E */

		if (timer_pending(&check_wakeup_source_timer)) {
			del_timer(&check_wakeup_source_timer);
			print_period = WAIT_FOR_PRINT_WAKEUP_SOURCE;
			printk("[PM] check_wakeup_source_timer: del_timer (late_resume)\n");
		}
		/* ZE500CL001E */
		state &= ~SUSPEND_REQUESTED;
		__pm_stay_awake(early_suspend_ws);
		queue_up_early_suspend_work(&late_resume_work);
	}
	suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
	mutex_unlock(&suspend_lock);
}

int __init early_suspend_init(void)
{
	int ret = 0;

	/* ZE500CL001S */
	pmsp_dev.name = "PowerManagerServicePrinter";
	pmsp_dev.index = 0;
	INIT_WORK(&pms_printer, pms_printer_func);
	ret = switch_dev_register(&pmsp_dev);
	if (ret < 0)
		printk("%s: Fail to register switch power_manager_printer \n", __func__);
	else
		printk("%s: Success to register pmsp switch \n", __func__);


	pm_wakeup_source_dev.name = "PowerManagerWakelocks";
	pm_wakeup_source_dev.index = 0;
	INIT_WORK(&pm_wakeup_source_printer, pm_wakeup_source_func);
	ret = switch_dev_register(&pm_wakeup_source_dev);
	if (ret < 0)
		printk("%s: Fail to register switch device pm_wakeup_source_dev\n", __func__);
	else
		printk("%s: Success to register switch device pm_wakeup_source_dev\n", __func__);
	/* ZE500CL001E */

	early_suspend_ws = wakeup_source_register("early_suspend");

	if (!early_suspend_ws) {
		ret = -ENOMEM;
		goto out;
	}

	early_suspend_wq = alloc_ordered_workqueue("early_suspend", 0);

	if (!early_suspend_wq) {
		ret = -ENOMEM;
		goto ws_err;
	}

	suspend_wq = alloc_ordered_workqueue("auto_suspend", 0);

	if (!suspend_wq) {
		ret = -ENOMEM;
		goto es_wq_err;
	}

	goto out;

es_wq_err:
	destroy_workqueue(early_suspend_wq);
ws_err:
	wakeup_source_unregister(early_suspend_ws);
out:
	return ret;
}
