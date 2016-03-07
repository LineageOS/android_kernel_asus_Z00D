obj-y += property.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DIR_PROPERTY_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

