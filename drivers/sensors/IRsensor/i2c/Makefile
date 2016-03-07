obj-y += i2c.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DIR_I2C_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

