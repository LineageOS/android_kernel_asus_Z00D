obj-y += ap3425.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DIR_AP3425_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)