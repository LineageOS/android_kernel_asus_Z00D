/* 
 * Copyright (C) 2014 ASUSTek Inc.
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

 /*********************************/
/* IR Sensor module test Module */
/********************************/
 
#ifndef __LINUX_IRSENSOR_MTEST_H
#define __LINUX_IRSENSOR_MTEST_H

#define TEST_RUN		(10)
#define SYSFS_ERROR	(1)
#define HARDWARE_ERROR	(2)

#define PS_TURN_ON_H	(0x03)
#define PS_TURN_ON_L	(0x06)
#define PS_TURN_OFF_H	(0x00)
#define PS_TURN_OFF_L	(0x07)

extern int mtest_sysfs(void);
extern int mtest_hardware(void);
extern int mtest_report(void);
extern int mtest_property(void);
extern int mtest_calibration(void);

#endif
