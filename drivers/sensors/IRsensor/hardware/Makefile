obj-y += cm36686/
obj-y += ap3425/
obj-y += hardware.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DIR_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

