/*
 * kxtj9_platform_data.c: kxtj9 accel platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_kxtj9.h"
#include <linux/input/kxtj9.h>

void *kxtj9_platform_data(void *info)
{
	static struct kxtj9_platform_data accel_data_kxtj9;
	//accel_data_kxtj9.gpio = get_gpio_by_name("ACC_INT1");
	accel_data_kxtj9.gpio = get_gpio_by_name("ACC_INT");
	return &accel_data_kxtj9;
}
