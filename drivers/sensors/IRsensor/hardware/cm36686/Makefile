obj-y += cm36686.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DIR_CM36686_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)