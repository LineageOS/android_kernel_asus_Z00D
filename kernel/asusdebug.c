/* //20100930 jack_wong for asus debug mechanisms +++++
 *  asusdebug.c
 * //20100930 jack_wong for asus debug mechanisms -----
 *
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/export.h>

#include <linux/sizes.h>
#include <linux/stacktrace.h>
#include <linux/slab.h>
#include "rtmutex_common.h"

int entering_suspend = 0;
char messages[256];
int first = 0;

char evtlog_bootup_reason[50];
char evtlog_target_os[32];

extern int nSuspendInProgress;
static struct workqueue_struct *ASUSEvtlog_workQueue;
static int g_hfileEvtlog = -MAX_ERRNO;
static int g_bEventlogEnable = 1;
static char g_Asus_Eventlog[ASUS_EVTLOG_MAX_ITEM][ASUS_EVTLOG_STR_MAXLEN];
static int g_Asus_Eventlog_read = 0;
static int g_Asus_Eventlog_write = 0;

static void do_write_event_worker(struct work_struct *work);
static DECLARE_WORK(eventLog_Work, do_write_event_worker);

static struct mutex mA;
#define AID_SDCARD_RW 1015
static void do_write_event_worker(struct work_struct *work)
{
	char buffer[256];
	memset(buffer, 0, sizeof(char) * 256);

	if (IS_ERR((const void *)g_hfileEvtlog)) {
		long size;

		g_hfileEvtlog =
		    sys_open(ASUS_EVTLOG_PATH ".txt", O_CREAT | O_RDWR | O_SYNC,
			     0444);
		sys_chown(ASUS_EVTLOG_PATH ".txt", AID_SDCARD_RW,
			  AID_SDCARD_RW);

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_2M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH "_old.txt");
			sys_rename(ASUS_EVTLOG_PATH ".txt",
				   ASUS_EVTLOG_PATH "_old.txt");
			g_hfileEvtlog =
			    sys_open(ASUS_EVTLOG_PATH ".txt",
				     O_CREAT | O_RDWR | O_SYNC, 0444);
		}
		sprintf(buffer,
			"\n\n---------------System Boot----%s---------\n"
			"###### Bootup Reason: %s ######\n"
			"###### Target OS: %s     ######\n",
			ASUS_SW_VER,
			evtlog_bootup_reason,
			evtlog_target_os);

		sys_write(g_hfileEvtlog, buffer, strlen(buffer));
		sys_close(g_hfileEvtlog);
	}
	if (!IS_ERR((const void *)g_hfileEvtlog)) {
		int str_len;
		char *pchar;
		long size;

		g_hfileEvtlog =
		    sys_open(ASUS_EVTLOG_PATH ".txt", O_CREAT | O_RDWR | O_SYNC,
			     0444);
		sys_chown(ASUS_EVTLOG_PATH ".txt", AID_SDCARD_RW,
			  AID_SDCARD_RW);

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_2M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH "_old.txt");
			sys_rename(ASUS_EVTLOG_PATH ".txt",
				   ASUS_EVTLOG_PATH "_old.txt");
			g_hfileEvtlog =
			    sys_open(ASUS_EVTLOG_PATH ".txt",
				     O_CREAT | O_RDWR | O_SYNC, 0444);
		}

		while (g_Asus_Eventlog_read != g_Asus_Eventlog_write) {
			mutex_lock(&mA);
			str_len = strlen(g_Asus_Eventlog[g_Asus_Eventlog_read]);
			pchar = g_Asus_Eventlog[g_Asus_Eventlog_read];
			g_Asus_Eventlog_read++;
			g_Asus_Eventlog_read %= ASUS_EVTLOG_MAX_ITEM;
			mutex_unlock(&mA);

			if (pchar[str_len - 1] != '\n') {
				pchar[str_len] = '\n';
				pchar[str_len + 1] = '\0';
			}

			sys_write(g_hfileEvtlog, pchar, strlen(pchar));
			sys_fsync(g_hfileEvtlog);
		}
		sys_close(g_hfileEvtlog);
	}
}

extern struct timezone sys_tz;

void ASUSEvtlog(const char *fmt, ...)
{

	va_list args;
	char *buffer;

	if (g_bEventlogEnable == 0)
		return;
	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_lock(&mA);

	buffer = g_Asus_Eventlog[g_Asus_Eventlog_write];
	g_Asus_Eventlog_write++;
	g_Asus_Eventlog_write %= ASUS_EVTLOG_MAX_ITEM;

	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_unlock(&mA);

	memset(buffer, 0, ASUS_EVTLOG_STR_MAXLEN);
	if (buffer) {
		struct rtc_time tm;
		struct timespec ts;

		getnstimeofday(&ts);
		ts.tv_sec -= sys_tz.tz_minuteswest * 60;
		rtc_time_to_tm(ts.tv_sec, &tm);
		getrawmonotonic(&ts);
		sprintf(buffer, "(%ld)%04d-%02d-%02d %02d:%02d:%02d :",
			ts.tv_sec, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec);
		/*printk(buffer); */
		va_start(args, fmt);
		vscnprintf(buffer + strlen(buffer),
			   ASUS_EVTLOG_STR_MAXLEN - strlen(buffer), fmt, args);
		va_end(args);
		/*printk(buffer); */
		queue_work(ASUSEvtlog_workQueue, &eventLog_Work);
	} else {
		printk("ASUSEvtlog buffer cannot be allocated\n");
	}
}

EXPORT_SYMBOL(ASUSEvtlog);

struct mutex fake_mutex;
struct completion fake_completion;
struct rt_mutex fake_rtmutex;

int asus_rtc_read_time(struct rtc_time *tm)
{
	struct timespec ts;
	getnstimeofday(&ts);
	ts.tv_sec -= sys_tz.tz_minuteswest * 60;
	rtc_time_to_tm(ts.tv_sec, tm);
	printk("now %04d%02d%02d-%02d%02d%02d, tz=%d\r\n", tm->tm_year + 1900,
	       tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
	       sys_tz.tz_minuteswest);
	return 0;
}

EXPORT_SYMBOL(asus_rtc_read_time);

static char *g_phonehang_log;
static int g_iPtr = 0;
int save_log(const char *f, ...)
{
	va_list args;
	int len;

	if (g_iPtr < PHONE_HANG_LOG_SIZE) {
		va_start(args, f);
		len =
		    vsnprintf(g_phonehang_log + g_iPtr,
			      PHONE_HANG_LOG_SIZE - g_iPtr, f, args);
		va_end(args);
		if (g_iPtr < PHONE_HANG_LOG_SIZE) {
			g_iPtr += len;
			return 0;
		}
	}
	g_iPtr = PHONE_HANG_LOG_SIZE;
	return -1;
}

static char *task_state_array[] = {
	"RUNNING",		/*  0 */
	"INTERRUPTIBLE",	/*  1 */
	"UNINTERRUPTIB",	/*  2 */
	"STOPPED",		/*  4 */
	"TRACED",		/*  8 */
	"EXIT ZOMBIE",		/* 16 */
	"EXIT DEAD",		/* 32 */
	"DEAD",			/* 64 */
	"WAKEKILL",		/* 128 */
	"WAKING"		/* 256 */
};

struct thread_info_save;
struct thread_info_save {
	struct task_struct *pts;
	pid_t pid;
	u64 sum_exec_runtime;
	u64 vruntime;
	struct thread_info_save *pnext;
};
static char *print_state(long state)
{
	int i;
	if (state == 0)
		return task_state_array[0];
	for (i = 1; i <= 16; i++) {
		if (1 << (i - 1) & state)
			return task_state_array[i];
	}
	return "NOTFOUND";

}

/*
 * Ease the printing of nsec fields:
 */
static long long nsec_high(unsigned long long nsec)
{
	if ((long long)nsec < 0) {
		nsec = -nsec;
		do_div(nsec, 1000000);
		return -nsec;
	}
	do_div(nsec, 1000000);

	return nsec;
}

static unsigned long nsec_low(unsigned long long nsec)
{
	unsigned long long nsec1;
	if ((long long)nsec < 0)
		nsec = -nsec;

	nsec1 = do_div(nsec, 1000000);
	return do_div(nsec1, 1000000);
}

#define MAX_STACK_TRACE_DEPTH   64
struct stack_trace_data {
	struct stack_trace *trace;
	unsigned int no_sched_functions;
	unsigned int skip;
};

struct stackframe {
	unsigned long fp;
	unsigned long sp;
	unsigned long lr;
	unsigned long pc;
};

void show_stack1(struct task_struct *p1, void *p2)
{
	struct stack_trace trace;
	unsigned long *entries;
	int i;

	entries = kmalloc(MAX_STACK_TRACE_DEPTH * sizeof(*entries), GFP_KERNEL);
	if (!entries) {
		printk("entries malloc failure\n");
		return;
	}
	trace.nr_entries = 0;
	trace.max_entries = MAX_STACK_TRACE_DEPTH;
	trace.entries = entries;
	trace.skip = 0;
	save_stack_trace_tsk(p1, &trace);

	for (i = 0; i < trace.nr_entries; i++) {
		save_log("[<%p>] %pS\n", (void *)entries[i],
			 (void *)entries[i]);
	}
	kfree(entries);
}

#define SPLIT_NS(x) nsec_high(x), nsec_low(x)
void print_all_thread_info(void)
{
	struct task_struct *pts;
	struct thread_info *pti;
	struct rtc_time tm;
	asus_rtc_read_time(&tm);

	g_iPtr = 0;
	memset(g_phonehang_log, 0, PHONE_HANG_LOG_SIZE);

	save_log
	    ("PhoneHang-%04d%02d%02d-%02d%02d%02d.txt  ---  ASUS_SW_VER : %s----------------------------------------------\r\n",
	     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
	     tm.tm_min, tm.tm_sec, ASUS_SW_VER);
	save_log
	    (" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt-Binder----Waiting\r\n");

	for_each_process(pts) {
		pti = task_thread_info(pts);
		save_log
		    ("-----------------------------------------------------\r\n");
		save_log(" %-7d", pts->pid);

		if (pts->parent)
			save_log("%-8d", pts->parent->pid);
		else
			save_log("%-8d", 0);

		save_log("%-20s", pts->comm);
		save_log("%lld.%06ld", SPLIT_NS(pts->se.sum_exec_runtime));
		save_log("     %lld.%06ld     ", SPLIT_NS(pts->se.vruntime));
		save_log("%-5d", pts->static_prio);
		save_log("%-5d", pts->normal_prio);
		save_log("%-15s",
			 print_state((pts->state & TASK_REPORT) | pts->
				     exit_state));
		save_log("%-6d", pti->preempt_count);

		if (pti->pWaitingMutex != &fake_mutex
		    && pti->pWaitingMutex != NULL) {
			if (pti->pWaitingMutex->name) {
				save_log("    Mutex:%s,",
					 pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,",
				       pti->pWaitingMutex->name + 1);
			} else
				printk("pti->pWaitingMutex->name == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug) {
				save_log(" Owned by pID(%d)",
					 pti->pWaitingMutex->
					 mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)",
				       pti->pWaitingMutex->
				       mutex_owner_asusdebug->pid);
			} else
				printk
				    ("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm) {
				save_log(" %s",
					 pti->pWaitingMutex->
					 mutex_owner_asusdebug->comm);
				printk(" %s",
				       pti->pWaitingMutex->
				       mutex_owner_asusdebug->comm);
			} else
				printk
				    ("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
		}

		if (pti->pWaitingCompletion != &fake_completion
		    && pti->pWaitingCompletion != NULL) {
			if (pti->pWaitingCompletion->name)
				save_log
				    ("    Completion:wait_for_completion %s",
				     pti->pWaitingCompletion->name);
			else
				printk
				    ("pti->pWaitingCompletion->name == NULL\r\n");
		}

		if (pti->pWaitingRTMutex != &fake_rtmutex
		    && pti->pWaitingRTMutex != NULL) {
			struct task_struct *temp =
			    rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)",
					 temp->pid);
			else
				printk
				    ("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk
				    ("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}

		save_log("\r\n");
		show_stack1(pts, NULL);

		save_log("\r\n");

		if (!thread_group_empty(pts)) {
			struct task_struct *p1 = next_thread(pts);
			do {
				pti = task_thread_info(p1);
				save_log(" %-7d", p1->pid);

				if (pts->parent)
					save_log("%-8d", p1->parent->pid);
				else
					save_log("%-8d", 0);

				save_log("%-20s", p1->comm);
				save_log("%lld.%06ld",
					 SPLIT_NS(p1->se.sum_exec_runtime));
				save_log("     %lld.%06ld     ",
					 SPLIT_NS(p1->se.vruntime));
				save_log("%-5d", p1->static_prio);
				save_log("%-5d", p1->normal_prio);
				save_log("%-15s",
					 print_state((p1->
						      state & TASK_REPORT) |
						     p1->exit_state));
				save_log("%-6d", pti->preempt_count);

				if (pti->pWaitingMutex != &fake_mutex
				    && pti->pWaitingMutex != NULL) {
					if (pti->pWaitingMutex->name) {
						save_log("    Mutex:%s,",
							 pti->pWaitingMutex->
							 name + 1);
						printk("    Mutex:%s,",
						       pti->pWaitingMutex->
						       name + 1);
					} else
						printk
						    ("pti->pWaitingMutex->name == NULL\r\n");

					if (pti->pWaitingMutex->
					    mutex_owner_asusdebug) {
						save_log(" Owned by pID(%d)",
							 pti->pWaitingMutex->
							 mutex_owner_asusdebug->
							 pid);
						printk(" Owned by pID(%d)",
						       pti->pWaitingMutex->
						       mutex_owner_asusdebug->
						       pid);
					} else
						printk
						    ("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

					if (pti->pWaitingMutex->
					    mutex_owner_asusdebug->comm) {
						save_log(" %s",
							 pti->pWaitingMutex->
							 mutex_owner_asusdebug->
							 comm);
						printk(" %s",
						       pti->pWaitingMutex->
						       mutex_owner_asusdebug->
						       comm);
					} else
						printk
						    ("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
				}

				if (pti->pWaitingCompletion != &fake_completion
				    && pti->pWaitingCompletion != NULL) {
					if (pti->pWaitingCompletion->name)
						save_log
						    ("    Completion:wait_for_completion %s",
						     pti->pWaitingCompletion->
						     name);
					else
						printk
						    ("pti->pWaitingCompletion->name == NULL\r\n");
				}

				if (pti->pWaitingRTMutex != &fake_rtmutex
				    && pti->pWaitingRTMutex != NULL) {
					struct task_struct *temp =
					    rt_mutex_owner(pti->
							   pWaitingRTMutex);
					if (temp)
						save_log
						    ("    RTMutex: Owned by pID(%d)",
						     temp->pid);
					else
						printk
						    ("pti->pWaitingRTMutex->temp == NULL\r\n");
					if (temp->comm)
						save_log(" %s", temp->comm);
					else
						printk
						    ("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
				}
				save_log("\r\n");
				show_stack1(p1, NULL);

				save_log("\r\n");
				p1 = next_thread(p1);
			} while (p1 != pts);
		}
		save_log
		    ("-----------------------------------------------------\r\n\r\n\r\n");

	}
	save_log("\r\n\r\n\r\n\r\n");
}

struct thread_info_save *ptis_head;
int find_thread_info(struct task_struct *pts, int force)
{
	struct thread_info *pti;
	struct thread_info_save *ptis, *ptis_ptr;
	u64 vruntime = 0, sum_exec_runtime;

	if (ptis_head != NULL) {
		ptis = ptis_head->pnext;
		ptis_ptr = NULL;
		while (ptis) {
			if (ptis->pid == pts->pid && ptis->pts == pts) {
				ptis_ptr = ptis;
				break;
			}
			ptis = ptis->pnext;
		}
		if (ptis_ptr) {
			sum_exec_runtime =
			    pts->se.sum_exec_runtime - ptis->sum_exec_runtime;
		} else {
			sum_exec_runtime = pts->se.sum_exec_runtime;
		}
		if (sum_exec_runtime > 0 || force) {
			pti = task_thread_info(pts);
			save_log(" %-7d", pts->pid);

			if (pts->parent)
				save_log("%-8d", pts->parent->pid);
			else
				save_log("%-8d", 0);

			save_log("%-20s", pts->comm);
			save_log("%lld.%06ld", SPLIT_NS(sum_exec_runtime));
			if (nsec_high(sum_exec_runtime) > 1000)
				save_log(" ******");
			save_log("     %lld.%06ld     ", SPLIT_NS(vruntime));
			save_log("%-5d", pts->static_prio);
			save_log("%-5d", pts->normal_prio);
			save_log("%-15s", print_state(pts->state));
			save_log("%-6d", pti->preempt_count);

			if (pti->pWaitingMutex != &fake_mutex
			    && pti->pWaitingMutex != NULL) {
				if (pti->pWaitingMutex->name) {
					save_log("    Mutex:%s,",
						 pti->pWaitingMutex->name + 1);
					printk("    Mutex:%s,",
					       pti->pWaitingMutex->name + 1);
				} else
					printk
					    ("pti->pWaitingMutex->name == NULL\r\n");

				if (pti->pWaitingMutex->mutex_owner_asusdebug) {
					save_log(" Owned by pID(%d)",
						 pti->pWaitingMutex->
						 mutex_owner_asusdebug->pid);
					printk(" Owned by pID(%d)",
					       pti->pWaitingMutex->
					       mutex_owner_asusdebug->pid);
				} else
					printk
					    ("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

				if (pti->pWaitingMutex->mutex_owner_asusdebug->
				    comm) {
					save_log(" %s",
						 pti->pWaitingMutex->
						 mutex_owner_asusdebug->comm);
					printk(" %s",
					       pti->pWaitingMutex->
					       mutex_owner_asusdebug->comm);
				} else
					printk
					    ("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
			}

			if (pti->pWaitingCompletion != &fake_completion
			    && pti->pWaitingCompletion != NULL) {
				if (pti->pWaitingCompletion->name)
					save_log
					    ("    Completion:wait_for_completion %s",
					     pti->pWaitingCompletion->name);
				else
					printk
					    ("pti->pWaitingCompletion->name == NULL\r\n");
			}

			if (pti->pWaitingRTMutex != &fake_rtmutex
			    && pti->pWaitingRTMutex != NULL) {
				struct task_struct *temp =
				    rt_mutex_owner(pti->pWaitingRTMutex);
				if (temp)
					save_log
					    ("    RTMutex: Owned by pID(%d)",
					     temp->pid);
				else
					printk
					    ("pti->pWaitingRTMutex->temp == NULL\r\n");
				if (temp->comm)
					save_log(" %s", temp->comm);
				else
					printk
					    ("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
			}

			save_log("\r\n");

			show_stack1(pts, NULL);
			save_log("\r\n");
		} else
			return 0;
	}
	return 1;

}

void save_all_thread_info(void)
{
	struct task_struct *pts;
	struct thread_info *pti;
	struct thread_info_save *ptis = NULL, *ptis_ptr = NULL;

	struct rtc_time tm;
	asus_rtc_read_time(&tm);
	g_iPtr = 0;
	memset(g_phonehang_log, 0, PHONE_HANG_LOG_SIZE);

	save_log
	    ("ASUSSlowg-%04d%02d%02d-%02d%02d%02d.txt  ---  ASUS_SW_VER : %s----------------------------------------------\r\n",
	     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
	     tm.tm_min, tm.tm_sec, ASUS_SW_VER);
	save_log
	    (" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt-binder----Waiting\r\n");

	if (ptis_head != NULL) {
		struct thread_info_save *ptis_next = ptis_head->pnext;
		struct thread_info_save *ptis_next_next;
		while (ptis_next) {
			ptis_next_next = ptis_next->pnext;
			kfree(ptis_next);
			ptis_next = ptis_next_next;
		}
		kfree(ptis_head);
		ptis_head = NULL;
	}

	if (ptis_head == NULL) {
		ptis_ptr = ptis_head =
		    kmalloc(sizeof(struct thread_info_save), GFP_KERNEL);
		if (!ptis_head) {
			printk("kmalloc ptis_head failure\n");
			return;
		}
		memset(ptis_head, 0, sizeof(struct thread_info_save));
	}

	for_each_process(pts) {
		pti = task_thread_info(pts);
		ptis = kmalloc(sizeof(struct thread_info_save), GFP_KERNEL);
		if (!ptis) {
			printk("kmalloc ptis failure\n");
			return;
		}
		memset(ptis, 0, sizeof(struct thread_info_save));

		save_log
		    ("-----------------------------------------------------\r\n");
		save_log(" %-7d", pts->pid);
		if (pts->parent)
			save_log("%-8d", pts->parent->pid);
		else
			save_log("%-8d", 0);

		save_log("%-20s", pts->comm);
		save_log("%lld.%06ld", SPLIT_NS(pts->se.sum_exec_runtime));
		save_log("     %lld.%06ld     ", SPLIT_NS(pts->se.vruntime));
		save_log("%-5d", pts->static_prio);
		save_log("%-5d", pts->normal_prio);
		save_log("%-15s",
			 print_state((pts->state & TASK_REPORT) | pts->
				     exit_state));
		save_log("%-6d", pti->preempt_count);

		if (pti->pWaitingMutex != &fake_mutex
		    && pti->pWaitingMutex != NULL) {
			if (pti->pWaitingMutex->name) {
				save_log("    Mutex:%s,",
					 pti->pWaitingMutex->name + 1);
				printk("    Mutex:%s,",
				       pti->pWaitingMutex->name + 1);
			} else
				printk("pti->pWaitingMutex->name == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug) {
				save_log(" Owned by pID(%d)",
					 pti->pWaitingMutex->
					 mutex_owner_asusdebug->pid);
				printk(" Owned by pID(%d)",
				       pti->pWaitingMutex->
				       mutex_owner_asusdebug->pid);
			} else
				printk
				    ("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

			if (pti->pWaitingMutex->mutex_owner_asusdebug->comm) {
				save_log(" %s",
					 pti->pWaitingMutex->
					 mutex_owner_asusdebug->comm);
				printk(" %s",
				       pti->pWaitingMutex->
				       mutex_owner_asusdebug->comm);
			} else
				printk
				    ("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
		}
		if (pti->pWaitingCompletion != &fake_completion
		    && pti->pWaitingCompletion != NULL) {
			if (pti->pWaitingCompletion->name)
				save_log
				    ("    Completion:wait_for_completion %s",
				     pti->pWaitingCompletion->name);
			else
				printk
				    ("pti->pWaitingCompletion->name == NULL\r\n");
		}

		if (pti->pWaitingRTMutex != &fake_rtmutex
		    && pti->pWaitingRTMutex != NULL) {
			struct task_struct *temp =
			    rt_mutex_owner(pti->pWaitingRTMutex);
			if (temp)
				save_log("    RTMutex: Owned by pID(%d)",
					 temp->pid);
			else
				printk
				    ("pti->pWaitingRTMutex->temp == NULL\r\n");
			if (temp->comm)
				save_log(" %s", temp->comm);
			else
				printk
				    ("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
		}

		save_log("\r\n");
		show_stack1(pts, NULL);

		save_log("\r\n");

		ptis->pid = pts->pid;
		ptis->pts = pts;
		ptis->sum_exec_runtime = pts->se.sum_exec_runtime;
		ptis->vruntime = pts->se.vruntime;

		ptis_ptr->pnext = ptis;
		ptis_ptr = ptis;

		if (!thread_group_empty(pts)) {
			struct task_struct *p1 = next_thread(pts);
			do {
				pti = task_thread_info(p1);
				ptis =
				    kmalloc(sizeof(struct thread_info_save),
					    GFP_KERNEL);
				if (!ptis) {
					printk("kmalloc ptis 2 failure\n");
					return;
				}
				memset(ptis, 0,
				       sizeof(struct thread_info_save));

				ptis->pid = p1->pid;
				ptis->pts = p1;
				ptis->sum_exec_runtime =
				    p1->se.sum_exec_runtime;
				ptis->vruntime = p1->se.vruntime;

				ptis_ptr->pnext = ptis;
				ptis_ptr = ptis;
				save_log(" %-7d", p1->pid);

				if (pts->parent)
					save_log("%-8d", p1->parent->pid);
				else
					save_log("%-8d", 0);

				save_log("%-20s", p1->comm);
				save_log("%lld.%06ld",
					 SPLIT_NS(p1->se.sum_exec_runtime));
				save_log("     %lld.%06ld     ",
					 SPLIT_NS(p1->se.vruntime));
				save_log("%-5d", p1->static_prio);
				save_log("%-5d", p1->normal_prio);
				save_log("%-15s",
					 print_state((p1->
						      state & TASK_REPORT) |
						     p1->exit_state));
				save_log("%-6d", pti->preempt_count);

				if (pti->pWaitingMutex != &fake_mutex
				    && pti->pWaitingMutex != NULL) {
					if (pti->pWaitingMutex->name) {
						save_log("    Mutex:%s,",
							 pti->pWaitingMutex->
							 name + 1);
						printk("    Mutex:%s,",
						       pti->pWaitingMutex->
						       name + 1);
					} else
						printk
						    ("pti->pWaitingMutex->name == NULL\r\n");

					if (pti->pWaitingMutex->
					    mutex_owner_asusdebug) {
						save_log(" Owned by pID(%d)",
							 pti->pWaitingMutex->
							 mutex_owner_asusdebug->
							 pid);
						printk(" Owned by pID(%d)",
						       pti->pWaitingMutex->
						       mutex_owner_asusdebug->
						       pid);
					} else
						printk
						    ("pti->pWaitingMutex->mutex_owner_asusdebug == NULL\r\n");

					if (pti->pWaitingMutex->
					    mutex_owner_asusdebug->comm) {
						save_log(" %s",
							 pti->pWaitingMutex->
							 mutex_owner_asusdebug->
							 comm);
						printk(" %s",
						       pti->pWaitingMutex->
						       mutex_owner_asusdebug->
						       comm);
					} else
						printk
						    ("pti->pWaitingMutex->mutex_owner_asusdebug->comm == NULL\r\n");
				}

				if (pti->pWaitingCompletion != &fake_completion
				    && pti->pWaitingCompletion != NULL) {
					if (pti->pWaitingCompletion->name)
						save_log
						    ("    Completion:wait_for_completion %s",
						     pti->pWaitingCompletion->
						     name);
					else
						printk
						    ("pti->pWaitingCompletion->name == NULL\r\n");
				}

				if (pti->pWaitingRTMutex != &fake_rtmutex
				    && pti->pWaitingRTMutex != NULL) {
					struct task_struct *temp =
					    rt_mutex_owner(pti->
							   pWaitingRTMutex);
					if (temp)
						save_log
						    ("    RTMutex: Owned by pID(%d)",
						     temp->pid);
					else
						printk
						    ("pti->pWaitingRTMutex->temp == NULL\r\n");
					if (temp->comm)
						save_log(" %s", temp->comm);
					else
						printk
						    ("pti->pWaitingRTMutex->temp->comm == NULL\r\n");
				}
				save_log("\r\n");
				show_stack1(p1, NULL);

				save_log("\r\n");

				p1 = next_thread(p1);
			} while (p1 != pts);
		}

	}

}

EXPORT_SYMBOL(save_all_thread_info);

void delta_all_thread_info(void)
{
	struct task_struct *pts;
	int ret = 0, ret2 = 0;

	save_log
	    ("\r\nDELTA INFO----------------------------------------------------------------------------------------------\r\n");
	save_log
	    (" pID----ppID----NAME----------------SumTime---vruntime--SPri-NPri-State----------PmpCnt----Waiting\r\n");
	for_each_process(pts) {
		ret = find_thread_info(pts, 0);
		if (!thread_group_empty(pts)) {
			struct task_struct *p1 = next_thread(pts);
			ret2 = 0;
			do {
				ret2 += find_thread_info(p1, 0);
				p1 = next_thread(p1);
			} while (p1 != pts);
			if (ret2 && !ret)
				find_thread_info(pts, 1);
		}
		if (ret || ret2)
			save_log
			    ("-----------------------------------------------------\r\n\r\n-----------------------------------------------------\r\n");
	}
	save_log("\r\n\r\n\r\n\r\n");
}

EXPORT_SYMBOL(delta_all_thread_info);

static mm_segment_t oldfs;
static void initKernelEnv(void)
{
	oldfs = get_fs();
	set_fs(KERNEL_DS);
}

static void deinitKernelEnv(void)
{
	set_fs(oldfs);
}

char messages[256];
void save_phone_hang_log(void)
{
	int file_handle = -1;
	int ret = 0;
	if (g_phonehang_log
	    && ((strncmp(g_phonehang_log, "PhoneHang", 9) == 0)
		|| (strncmp(g_phonehang_log, "ASUSSlowg", 9) == 0))) {
		printk("save_phone_hang_log-1\n");
		initKernelEnv();
		memset(messages, 0, sizeof(messages));
		strcpy(messages, "/data/logs/");
		strncat(messages, g_phonehang_log, 29);
		file_handle =
		    sys_open(messages, O_CREAT | O_WRONLY | O_SYNC, 0444);
		printk("save_phone_hang_log-2 file_handle %d, name=%s\n",
		       file_handle, messages);
		if (!IS_ERR((const void *)(ulong) file_handle)) {
			ret =
			    sys_write(file_handle,
				      (unsigned char *)g_phonehang_log,
				      strlen(g_phonehang_log));
			sys_close(file_handle);
		}
		deinitKernelEnv();

	}
	if (g_phonehang_log && file_handle > 0 && ret > 0) {
		g_phonehang_log[0] = 0;
	}
}

EXPORT_SYMBOL(save_phone_hang_log);

static ssize_t evtlogswitch_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	if (strncmp(buf, "0", 1) == 0) {
		ASUSEvtlog("ASUSEvtlog disable !!");
		printk("ASUSEvtlog disable !!\n");
		flush_work_sync(&eventLog_Work);
		g_bEventlogEnable = 0;
	}
	if (strncmp(buf, "1", 1) == 0) {
		g_bEventlogEnable = 1;
		ASUSEvtlog("ASUSEvtlog enable !!");
		printk("ASUSEvtlog enable !!\n");
	}

	return count;
}

static ssize_t asusevtlog_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	if (count > 256)
		count = 256;

	memset(messages, 0, sizeof(messages));
	if (copy_from_user(messages, buf, count))
		return -EFAULT;
	ASUSEvtlog(messages);

	return count;
}

static int asusdebug_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int asusdebug_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t asusdebug_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	return 0;
}

extern int rtc_ready;
int asus_asdf_set = 0;
static ssize_t asusdebug_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	u8 messages[256] = { 0 };

	if (count > 256)
		count = 256;
	if (copy_from_user(messages, buf, count))
		return -EFAULT;

	if (strncmp(messages, "panic", strlen("panic")) == 0) {
		panic("panic test");
	} else if (strncmp(messages, "gichk", 5) == 0) {
		save_all_thread_info();
		return count;
	} else if (strncmp(messages, "gidelta", 7) == 0) {
		delta_all_thread_info();
		save_phone_hang_log();
		return count;
	} else if (strncmp(messages, "gi", 2) == 0) {
		print_all_thread_info();
		return count;
	} else if (strncmp(messages, "slowlog", strlen("slowlog")) == 0) {
		printk("start to gi chk\n");
		save_all_thread_info();

		msleep(5 * 1000);

		printk("start to gi delta\n");
		delta_all_thread_info();
		save_phone_hang_log();
		return count;
	}

	return count;
}

static int gJBStatus = 3;
static int JBStatus_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", gJBStatus);
	return 0;
}

static int JBStatus_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, JBStatus_proc_show, NULL);
}

static ssize_t JBStatus_proc_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	char num[10];
	memset(num, 0, sizeof(num));

	/* no data be written */
	if (!count) {
		return 0;
	}

	/* Input size is too large to write our buffer(num) */
	if (count > (sizeof(num) - 1)) {
		return -EINVAL;
	}

	if (copy_from_user(num, buf, count)) {
		return -EFAULT;
	}

	if (strncmp(num, "0", 1) == 0) {
		gJBStatus = 0;
	} else if (strncmp(num, "1", 1) == 0) {
		gJBStatus = 1;
	} else {
		printk("JBStatus unknown data!!\n");
	}

	return count;
}

int get_JB_status(void)
{
	return gJBStatus;
}
EXPORT_SYMBOL(get_JB_status);


static int gKmsgconfig = 1;
static int Kmsgconfig_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d", gKmsgconfig);
	return 0;
}

static int Kmsgconfig_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Kmsgconfig_proc_show, NULL);
}

static ssize_t Kmsgconfig_proc_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	char num[10];
	memset(num, 0, sizeof(num));

	/* no data be written */
	if (!count) {
		return 0;
	}

	/* Input size is too large to write our buffer(num) */
	if (count > (sizeof(num) - 1)) {
		return -EINVAL;
	}

	if (copy_from_user(num, buf, count)) {
		return -EFAULT;
	}

	if (strncmp(num, "0", 1) == 0) {
		gKmsgconfig = 0;
	} else if (strncmp(num, "1", 1) == 0) {
		gKmsgconfig = 1;
	} else {
		printk("gKmsgconfig unknown data!!\n");
	}

	return count;
}

int get_Kmsgconfig(void)
{
	return gKmsgconfig;
}
EXPORT_SYMBOL(get_Kmsgconfig);



static const struct file_operations proc_evtlogswitch_operations = {
	.write = evtlogswitch_write,
};

static const struct file_operations proc_asusevtlog_operations = {
	.write = asusevtlog_write,
};

/*
 * Remove for AT&T
 *
static const struct file_operations proc_asusdebug_operations = {
	.read = asusdebug_read,
	.write = asusdebug_write,
	.open = asusdebug_open,
	.release = asusdebug_release,
};
*/

static const struct file_operations proc_JBStatus_operations = {
	.open = JBStatus_proc_open,
	.read = seq_read,
	.write = JBStatus_proc_write,
};

static const struct file_operations proc_kmsgconfig_operations = {
	.open = Kmsgconfig_proc_open,
	.read = seq_read,
	.write = Kmsgconfig_proc_write,
};

static int __init proc_asusdebug_init(void)
{
/*
 *  Remove for AT&T
 *
	proc_create("asusdebug", S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP, NULL, &proc_asusdebug_operations);
 *
*/
	proc_create("asusevtlog", S_IRWXUGO, NULL, &proc_asusevtlog_operations);
	proc_create("asusevtlog-switch", S_IRWXUGO, NULL,
		    &proc_evtlogswitch_operations);
	proc_create("JBStatus", S_IRUGO | S_IWUSR, NULL,
		    &proc_JBStatus_operations);
	proc_create("kmsgconfig", S_IRUGO | S_IWUSR, NULL,
		    &proc_kmsgconfig_operations);
	mutex_init(&mA);

	fake_mutex.owner = current;
	fake_mutex.mutex_owner_asusdebug = current;
	fake_mutex.name = " fake_mutex";
	strcpy(fake_completion.name, " fake_completion");
	fake_rtmutex.owner = current;

	ASUSEvtlog_workQueue =
	    create_singlethread_workqueue("ASUSEVTLOG_WORKQUEUE");

	g_phonehang_log = kmalloc(SZ_512K, GFP_KERNEL);
	if (g_phonehang_log == NULL) {
		printk("%s: failed to allocate memory for g_phonehang_log\n",
		       __func__);
	} else {
		printk("%s: g_phonehang_log=%p\n", __func__, g_phonehang_log);
	}

	return 0;
}

module_init(proc_asusdebug_init);
