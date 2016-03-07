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
/* IR Sensor Calibration Module */
/*******************************/
#ifndef __LINUX_IRSENSOR_CALIBRATION_H
#define __LINUX_IRSENSOR_CALIBRATION_H

extern int IRsensor_calibration_register(struct i2c_client *client);
extern int IRsensor_calibration_unregister(struct i2c_client *client);

#endif

