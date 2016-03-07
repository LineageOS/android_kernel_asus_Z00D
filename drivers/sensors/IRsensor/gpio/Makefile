obj-y += gpio.o

INTEL = y
ifeq ($(INTEL),y)
	LOCAL_CFALG = -DIR_GPIO_INTEL
else
	LOCAL_CFALG = -DIR_GPIO_QCOM
endif

DEBUG = n
ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DIR_GPIO_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
ccflags-y += $(LOCAL_CFALG)


