#
#	FILE NAME: Makefile
#
#	FILE DESCRIPTION: Make description file for building driver
#
#	PROJECT NAME: Linux DM6430 Driver, Library, and Example Programs
#
#	PROJECT VERSION: (Defined in README.TXT)
#
#	Copyright 2004 RTD Embedded Technologies, Inc.  All Rights Reserved.
#



#
# $(src) points to the relative pather where this Makefile exists.  This must
# be used instead of getting the current working directory via pwd when telling
# the prepocessor where to find the driver header files.
#

EXTRA_CFLAGS := -I$(shell pwd)/../include 

#
# Kernel build environment directory. Supposedly it is safer to use this method
# of referring to it than usign /usr/src.
#

KERNEL_DIR := /lib/modules/$(shell uname -r)/build

obj-m := rtd-dm6430.o

DRIVER_MODULE=rtd-dm6430.ko

FILES_TO_CLEAN = \
	*.o \
	.*.cmd \
	*.ko \
	.tmp_versions \
	rtd-dm6430.ko \
	rtd-dm6430.mod.c \
	.*.d \
	*.symvers \
	*.markers \
	*.order \
	*~

driver:	rtd-dm6430.c
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(EXTRA_CFLAGS)" modules

driver_debug: rtd-dm6430.c
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(EXTRA_CFLAGS) -DDEBUG" modules


#=============================================================================
# Rules applicable to either the 2.4 or 2.6 kernel
#=============================================================================
clean:
	rm -rf $(FILES_TO_CLEAN)


load:
	/sbin/insmod ./$(DRIVER_MODULE) io=0x300 irq1=10 irq2=5 dma1=5 dma2=6 buflength=131072
	chmod 0666 /dev/rtd-dm6430*

unload:
	/sbin/rmmod rtd-dm6430

#
# insmod-test is a rule for RTD internal testing.  It allocates the largest
# allowable DMA buffer, which is required by the dm6430-throughput example
# program.
#

insmod-test:
	insmod ./$(DRIVER_MODULE) io=0x300 irq1=3 irq2=5 dma1=5 dma2=6 \
		buflength=131072
