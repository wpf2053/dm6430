/*
	FILE NAME: rtd-dm6430.c

	FILE DESCRIPTION: Driver code for DM6430

	PROJECT NAME: Linux DM6430 Driver, Library, and Example Programs

	PROJECT VERSION: (Defined in README.TXT)

 */

//----------------------------------------------------------------------------
//  COPYRIGHT (C) RTD EMBEDDED TECHNOLOGIES, INC.  ALL RIGHTS RESERVED.
//
//  This software package is dual-licensed.  Source code that is compiled for
//  kernel mode execution is licensed under the GNU General Public License
//  version 2.  For a copy of this license, refer to the file
//  LICENSE_GPLv2.TXT (which should be included with this software) or contact
//  the Free Software Foundation.  Source code that is compiled for user mode
//  execution is licensed under the RTD End-User Software License Agreement.
//  For a copy of this license, refer to LICENSE.TXT or contact RTD Embedded
//  Technologies, Inc.  Using this software indicates agreement with the
//  license terms listed above.
//----------------------------------------------------------------------------


#include <linux/spinlock.h>
#include <linux/spinlock_types.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include <asm/io.h>
#include <asm/dma.h>
#include <asm/uaccess.h>

#include "dm6430ioctl.h"

#include "dm6430driver.h"
#include "dm6430_version.h"


#define DRIVER_NAME		"rtd-dm6430"
#define PROJECT_RELEASE		DRIVER_RELEASE
#define DRIVER_DESCRIPTION	"DM6430 device driver"
#define DRIVER_COPYRIGHT RTD_COPYRIGHT_STRING


static const char name[] = DRIVER_NAME;
static const char description[] __initdata = DRIVER_DESCRIPTION;
static const char version[] __initdata = PROJECT_RELEASE;
static const char copyright[] __initdata = DRIVER_COPYRIGHT;
static unsigned int dm6430_major = 0;
static unsigned int buflength = 0x10000; /* default value 64K */
#ifdef DEBUG
static int debug = 0x0;
#endif
static struct Dm6430hrDevice devices[DM6430HR_MAX_DEVS];
static int dm6430_device_count = 0;

/**
 * Character device descriptor for 2.6+ kernels
 */
static struct cdev dm6430_cdev;

/** Device class pointer */
static struct class *dev_class = NULL;


#ifndef DEBUG

#define dm_outb_p(x, y)		(outb_p(y, x))
#define dm_inb_p(x)		(inb_p(x))
#define dm_outw_p(x, y)		(outw_p(y, x))
#define dm_inw_p(x)		(inw_p(x))

#else

unsigned char
dm_inb_p(unsigned port)
{
	unsigned char byte_read;

	byte_read = inb_p(port);
	//printk(KERN_INFO "%X = inb_p(%X);\n", byte_read, port);

	return byte_read;
}

unsigned short
dm_inw_p(unsigned port)
{
	unsigned short word_read;

	word_read = inw_p(port);
	//printk(KERN_INFO "%X = inw_p(%X);\n", word_read, port);

	return word_read;
}

void
dm_outb_p(unsigned port, unsigned char byte_to_write)
{
	//printk(KERN_INFO "outb_p(%X,%X);\n", port, byte_to_write);
	outb_p(byte_to_write, port);
}

void
dm_outw_p(unsigned port, unsigned short word_to_write)
{
	// printk(KERN_INFO "outw_p(%X,%X);\n", port, word_to_write);
	outw_p(word_to_write, port);
}

#endif

#define dm_insb(port, addr, count)	(insb((port), (addr), (count)))
#define dm_insw(port, addr, count)	(insw((port), (addr), (count)))

#define dm_dma_mem_alloc(size) 		__get_dma_pages(GFP_KERNEL, get_order(size))
#define dm_dma_mem_free(addr, size) 	free_pages(addr, get_order(size))

static int dm6430hr_register_device(struct Dm6430hrDevice * dev);
static void dm6430hr_unregister_device(struct Dm6430hrDevice *dev);

static void
dm_stop_dma(struct Dm6430hrDevice *device_p, int dma)
{
	unsigned long flags;

	flags = claim_dma_lock();
	disable_dma(device_p->dma[dma]);
	clear_dma_ff(device_p->dma[dma]);
	release_dma_lock(flags);
}

static inline void
DM6430HRClear(struct Dm6430hrDevice *device_p, unsigned short Mask)
{

#ifdef DEBUG

	if (debug & DBG_DEV)
		printk(KERN_INFO "DM6430HRClear(0x%04x)\n", Mask);

#endif

	/* setup the clear mask */
	dm_outw_p(device_p->io + r_CLEAR_6430, Mask);
	/* clear */
	dm_inw_p(device_p->io + r_CLEAR_6430);
}

static void
DM6430HR_Initdev(struct Dm6430hrDevice *device_p)
{
	int i;

#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "DM6430HR_Initdev()\n");

#endif

	device_p->Control_Register = 0;

	for (i = 0; i < DM6430HR_IRQS; i++) {
		device_p->int_count[i] = 0;
		device_p->int_queue_in = 0;
		device_p->int_queue_out = 0;
		device_p->int_queue_missed = 0;
	}

	for (i = 0; i < DM6430HR_DMAS; i++) {
		device_p->dmabuf[i].addr = 0;
		device_p->dmabuf[i].length = 0;
		device_p->Control_Register |=
			(device_p->dma[i] & 0x03) << (i ? 14 : 12);
	}

	/* Default board settings */
	dm_outw_p(device_p->io + r_CONTROL_6430, device_p->Control_Register);
	dm_outw_p(device_p->io + r_TRIGGER_6430, device_p->Trigger_Register = 0);
	dm_outw_p(device_p->io + r_IRQ_6430, device_p->IRQ_Register = 0);
	dm_outw_p(device_p->io + r_DIN_CONFIG_6430, device_p->DIN_Register = 0);

	DM6430HRClear(device_p, 0x00FF);
}

static void
DM6430HRIRQEnable(struct Dm6430hrDevice *device_p, int IRQChannel, int Enable)
{
	static const unsigned short IRQs[16] ={0, 0, 0, 1, 0, 2, 0, 0, 0, 3, 4, 5, 6, 0, 0, 7};

#ifdef DEBUG

	if (debug & DBG_DEV)
		printk(KERN_INFO "DM6430HRIRQEnable(%d, %d)\n", IRQChannel, Enable);

#endif

	device_p->IRQ_Register &= (IRQChannel ? 0x1FFF : 0xFF1F);
	if (Enable)
		device_p->IRQ_Register |=
		((IRQs[device_p->irq[IRQChannel]]) << (IRQChannel ? 13 : 5));
	dm_outw_p(device_p->io + r_IRQ_6430, device_p->IRQ_Register);

	/* clear irq */
	DM6430HRClear(device_p, IRQChannel ? DM6430_CL_IRQ2 : DM6430_CL_IRQ1);
}

static inline struct Dm6430hrDevice *
lookup_dev(int minor)
{
	if (minor >= DM6430HR_MAX_DEVS)
		return 0;

	return &devices[minor];
}

/******************************************************************************
validate_dma_circuit()

    Purpose:
	Determine if a DMA circuit number given on certain ioctl() calls is
	valid.

    Parameters:
	dma_circuit => DMA circuit number to validate.

    Return Value:
	0
	    dma_circuit is valid.

	EINVAL
	    dma_circuit is not valid.
 *******************************************************************************/

static int
validate_dma_circuit(int dma_circuit)
{
	switch (dma_circuit) {
	case DM6430HR_DMA1:
	case DM6430HR_DMA2:
		return 0;
		break;

	default:
		return -EINVAL;
		break;
	}
}

/******************************************************************************
validate_interrupt_circuit()

    Purpose:
	Determine if an interrupt circuit number given on certain ioctl()
	calls is valid.

    Parameters:
	interrupt_circuit => Interrupt circuit number to validate.

    Return Value:
	0
	    interrupt_circuit is valid.

	EINVAL
	    interrupt_circuit is not valid.
 *******************************************************************************/

static int
validate_interrupt_circuit(int interrupt_circuit)
{
	switch (interrupt_circuit) {
	case DM6430HR_INT1:
	case DM6430HR_INT2:
		return 0;
		break;

	default:
		return -EINVAL;
		break;
	}
}

static int
dm6430hr_open(struct inode *inode_p, struct file *file_p)
{
	struct Dm6430hrDevice *device_p;
	int rc;
#ifdef DEBUG

	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "dm6430hr_open()\n");

#endif

	device_p = lookup_dev(MINOR((inode_p)->i_rdev));

	if (!device_p)
		return -ENXIO;

	if (!device_p->io)
		return -ENXIO;

    if (!(device_p->flags & INITIALIZED) &&
		(rc = dm6430hr_register_device(device_p)))
		return rc;

	if (file_p->private_data)
		return -EBUSY;

	file_p->private_data = device_p;
	atomic_inc(&device_p->counter);


	return 0;
}

static int
dm6430hr_release(struct inode *inode_p, struct file *file_p)
{
	struct Dm6430hrDevice *device_p;

	unsigned long flags;

#ifdef DEBUG

	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "dm6430hr_release()\n");

#endif

	device_p = (struct Dm6430hrDevice *) file_p->private_data;

	if (device_p) {
		int i;

		spin_lock_irqsave(&device_p->lock, flags);

		for (i = 0; i < DM6430HR_IRQS; i++) {
			DM6430HRIRQEnable(device_p, i, 0);
		}

		for (i = 0; i < DM6430HR_DMAS; i++) {
			device_p->Control_Register = device_p->Control_Register & (0xCFFF << (i * 2));
			dm_outw_p(device_p->io + r_CONTROL_6430, device_p->Control_Register);

			dm_stop_dma(device_p, i);

			if (device_p->dmabuf[i].addr)
				dm_dma_mem_free((unsigned long) device_p->dmabuf[i].addr, device_p->dmabuf[i].length);
			device_p->dmabuf[i].length = 0;
			device_p->dmabuf[i].addr = 0;

		}

		spin_unlock_irqrestore(&device_p->lock, flags);

		if (atomic_dec_and_test(&device_p->counter)) {
			dm6430hr_unregister_device(device_p);
		}
	}

	file_p->private_data = 0;

	return 0;
}

static int
DM6430HR_IOCTL_INB_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	struct DM6430HR_IO8 io_rq;

	unsigned long flags;

	if (copy_from_user(&io_rq, (struct DM6430HR_IO8 *) arg, sizeof(io_rq)))
		return -EFAULT;

	if (io_rq.reg >= DM6430HR_IO_EXTENT)
		return -EINVAL;

	spin_lock_irqsave(&device_p->lock, flags);
	io_rq.value = dm_inb_p(device_p->io + io_rq.reg);
	spin_unlock_irqrestore(&device_p->lock, flags);

#ifdef DEBUG

	if (debug & DBG_DEV)
		printk(
		KERN_INFO "INB io + 0x%04x, value 0x%02x\n",
		io_rq.reg,
		io_rq.value
		);

#endif

	if (copy_to_user((struct DM6430HR_IO8 *) arg, &io_rq, sizeof(io_rq)))
		return -EFAULT;

	return 0;
}

static int
DM6430HR_IOCTL_OUTB_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	struct DM6430HR_IO8 io_rq;

	unsigned long flags;

	if (copy_from_user(&io_rq, (struct DM6430HR_IO8 *) arg, sizeof(io_rq)))
		return -EFAULT;

	if (io_rq.reg >= DM6430HR_IO_EXTENT)
		return -EINVAL;

	spin_lock_irqsave(&device_p->lock, flags);
	dm_outb_p(device_p->io + io_rq.reg, io_rq.value);
	spin_unlock_irqrestore(&device_p->lock, flags);

#ifdef DEBUG

	if (debug & DBG_DEV)
		printk(
		KERN_INFO "OUTB io + 0x%04x = 0x%02x\n",
		io_rq.reg,
		io_rq.value
		);

#endif

	return 0;
}

static int
DM6430HR_IOCTL_MOUTB_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	u_int8_t value;
	struct DM6430HR_MIO8 io_rq;

	unsigned long flags;

	if (copy_from_user(&io_rq, (struct DM6430HR_IO8 *) arg, sizeof(io_rq)))
		return -EFAULT;

	if (io_rq.reg >= DM6430HR_IO_EXTENT)
		return -EINVAL;

	spin_lock_irqsave(&device_p->lock, flags);
	value = dm_inb_p(device_p->io + io_rq.reg);

	value = (value & io_rq.mask) | (io_rq.value & ~io_rq.mask);
	dm_outb_p(device_p->io + io_rq.reg, value);

	spin_unlock_irqrestore(&device_p->lock, flags);

#ifdef DEBUG

	if (debug & DBG_DEV)
		printk(
		KERN_INFO "MOUTB io + 0x%04x = 0x%04x(0x%04x/0x%04x)\n",
		io_rq.reg,
		value,
		io_rq.value,
		io_rq.mask
		);

#endif

	return 0;
}

static int
DM6430HR_IOCTL_INW_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	struct DM6430HR_IO16 io_rq;

	unsigned long flags;

	if (copy_from_user(&io_rq, (struct DM6430HR_IO16 *) arg, sizeof(io_rq)))
		return -EFAULT;

	if ((io_rq.reg > (DM6430HR_IO_EXTENT - 2)) || (io_rq.reg & 1))
		return -EINVAL;

	spin_lock_irqsave(&device_p->lock, flags);
	io_rq.value = dm_inw_p(device_p->io + io_rq.reg);
	spin_unlock_irqrestore(&device_p->lock, flags);

#ifdef DEBUG

	if (debug & DBG_DEV)
		printk(
		KERN_INFO "INW io + 0x%04x, value 0x%04x\n",
		io_rq.reg,
		io_rq.value
		);

#endif

	if (copy_to_user((struct DM6430HR_IO16 *) arg, &io_rq, sizeof(io_rq)))
		return -EFAULT;

	return 0;
}

static int
DM6430HR_IOCTL_OUTW_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	struct DM6430HR_IO16 io_rq;

	unsigned long flags;

	if (copy_from_user(&io_rq, (struct DM6430HR_IO16 *) arg, sizeof(io_rq)))
		return -EFAULT;

	if ((io_rq.reg > (DM6430HR_IO_EXTENT - 2)) || (io_rq.reg & 1))
		return -EINVAL;

	spin_lock_irqsave(&device_p->lock, flags);

	switch (io_rq.reg) {
	case r_CONTROL_6430:
		device_p->Control_Register = io_rq.value;
		break;

	case r_DIN_CONFIG_6430:
		device_p->DIN_Register = io_rq.value;
		break;

	case r_IRQ_6430:
		device_p->IRQ_Register = io_rq.value;
		break;

	case r_TRIGGER_6430:
		device_p->Trigger_Register = io_rq.value;
		break;

	default:
		;
	}

	dm_outw_p(device_p->io + io_rq.reg, io_rq.value);
	spin_unlock_irqrestore(&device_p->lock, flags);

#ifdef DEBUG

	if (debug & DBG_DEV)
		printk(
		KERN_INFO "OUTW io + 0x%04x = 0x%04x\n",
		io_rq.reg,
		io_rq.value
		);

#endif

	return 0;
}

static int
DM6430HR_IOCTL_MOUTW_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	u_int16_t value, *value_p;
	struct DM6430HR_MIO16 io_rq;

	unsigned long flags;

	if (copy_from_user(&io_rq, (struct DM6430HR_IO16 *) arg, sizeof(io_rq)))
		return -EFAULT;

	if ((io_rq.reg > (DM6430HR_IO_EXTENT - 2)) || (io_rq.reg & 1))
		return -EINVAL;

	spin_lock_irqsave(&device_p->lock, flags);

	switch (io_rq.reg) {
	case r_CONTROL_6430:
		value_p = &device_p->Control_Register;
		break;

	case r_DIN_CONFIG_6430:
		value_p = &device_p->DIN_Register;
		break;

	case r_IRQ_6430:
		value_p = &device_p->IRQ_Register;
		break;

	case r_TRIGGER_6430:
		value_p = &device_p->Trigger_Register;
		break;

	default:
		value = dm_inw_p(device_p->io + io_rq.reg);
		value_p = &value;
	}

	*value_p = (*value_p & io_rq.mask) | (io_rq.value & (~io_rq.mask));
	dm_outw_p(device_p->io + io_rq.reg, *value_p);

	spin_unlock_irqrestore(&device_p->lock, flags);

#ifdef DEBUG

	if (debug & DBG_DEV)
		printk(
		KERN_INFO "MOUTW io + 0x%04x = 0x%04x(0x%04x/0x%04x)\n",
		io_rq.reg,
		*value_p,
		io_rq.value,
		io_rq.mask
		);

#endif

	return 0;
}

static int
DM6430HR_IOCTL_CLEAR_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{

	unsigned long flags;

	spin_lock_irqsave(&device_p->lock, flags);
	DM6430HRClear(device_p, arg);
	spin_unlock_irqrestore(&device_p->lock, flags);
	return 0;
}

static int
DM6430HR_IOCTL_IRQ_ENABLE_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	int status;
	struct DM6430HR_IE io_rq;

	unsigned long flags;

#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "DM6430HR_IRQ_ENABLE()\n");

#endif

	if (copy_from_user(&io_rq, (struct DM6430HR_IE *) arg, sizeof(io_rq)))
		return -EFAULT;

	status = validate_interrupt_circuit(io_rq.intr);
	if (status != 0) {
		return status;
	}

	if (!device_p->irq[io_rq.intr])
		return -EINVAL;

	spin_lock_irqsave(&device_p->lock, flags);
	DM6430HRIRQEnable(device_p, io_rq.intr, io_rq.action ? 1 : 0);
	spin_unlock_irqrestore(&device_p->lock, flags);

#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "DM6430HR_IRQ_ENABLE() end\n");

#endif

	return 0;
}

static int
DM6430HR_IOCTL_DMA_INSTALL_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	int status;
	struct dm_dma_buf *buf_p;
	struct DM6430HR_DI io_rq;

	unsigned long flags;

#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "DM6430HR_DMA_INSTALL()\n");

#endif

	if (copy_from_user(&io_rq, (struct DM6430HR_DI *) arg, sizeof(io_rq)))
		return -EFAULT;

	status = validate_dma_circuit(io_rq.dma);
	if (status != 0) {
		return status;
	}

	if (!device_p->dma[io_rq.dma])
		return -EINVAL;

	buf_p = &device_p->dmabuf[io_rq.dma];

	if (io_rq.action) {
		if (buf_p->addr)
			return 0;

		buf_p->addr = (char *) dm_dma_mem_alloc(buflength);
		spin_lock_irqsave(&device_p->lock, flags);
		buf_p->length = buflength;
		spin_unlock_irqrestore(&device_p->lock, flags);

		if (!buf_p->addr)
			return -ENOMEM;

		memset(buf_p->addr, 0, buflength);
	} else {
		dm_stop_dma(device_p, io_rq.dma);

		spin_lock_irqsave(&device_p->lock, flags);
		if (buf_p->addr)
			dm_dma_mem_free((unsigned long) buf_p->addr, buf_p->length);
		buf_p->length = 0;
		buf_p->addr = 0;
		spin_unlock_irqrestore(&device_p->lock, flags);
	}

	return 0;
}

static int
DM6430HR_IOCTL_DMA_GETDATA_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	int status;
	struct DM6430HR_GDD io_rq;

#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "DM6430HR_DMA_GETDATA()\n");

#endif

	if (copy_from_user(&io_rq, (struct DM6430HR_GDD *) arg, sizeof(io_rq)))
		return -EFAULT;

	status = validate_dma_circuit(io_rq.dma);
	if (status != 0) {
		return status;
	}

	if (
		!device_p->dma[io_rq.dma]
		||
		!device_p->dmabuf[io_rq.dma].addr
		||
		((io_rq.offset + io_rq.length) > device_p->dmabuf[io_rq.dma].length)
		)
		return -EINVAL;

	if (
		copy_to_user(
		io_rq.buf,
		(device_p->dmabuf[io_rq.dma].addr + io_rq.offset),
		io_rq.length
		)
		)
		return -EFAULT;

	return 0;
}

static int
DM6430HR_IOCTL_DMA_GETINC_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	struct DM6430HR_GID io_rq;

	unsigned long flags;

#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "DM6430HR_DMA_GETINC()\n");

#endif

	if (copy_from_user(&io_rq, (struct DM6430HR_GID *) arg, sizeof(io_rq)))
		return -EFAULT;

	switch (io_rq.port) {
	case rSTR_AD_6430:
	case rSTR_DIN_FIFO_6430:
		break;

	default:
		return -EINVAL;
		break;
	}

	switch (io_rq.type) {
	case DM6430HR_STR_TYPE_BYTE:
	case DM6430HR_STR_TYPE_WORD:
		break;

	default:
		return -EINVAL;
		break;
	}

	/*
	 * Do not allow byte transfers from A/D FIFO
	 */

	if (
		(io_rq.port == rSTR_AD_6430)
		&&
		(io_rq.type != DM6430HR_STR_TYPE_WORD)
		) {
		return -EOPNOTSUPP;
	}

	/*
	 * Do not allow word transfers from digital input FIFO
	 */

	if (
		(io_rq.port == rSTR_DIN_FIFO_6430)
		&&
		(io_rq.type != DM6430HR_STR_TYPE_BYTE)
		) {
		return -EOPNOTSUPP;
	}

	/*
	 * Each FIFO has 1024 elements, don't allow more in one read
	 */

	if (io_rq.times > 1024) {
		return -EINVAL;
	}

	if (io_rq.times) {
		io_rq.port += device_p->io;

		spin_lock_irqsave(&device_p->lock, flags);

		/*
		 * Do I/O into kernel stream read buffer, not into user space buffer.
		 * In order to do I/O into a user space buffer, it must be remapped
		 * into kernel space using the kiobuf interface.  It's a stupid idea
		 * to do I/O directly into a user buffer without remapping because
		 * 1) the kernel is not set up do so, 2) it is a user address, 3) the
		 * 3) address can refer to anything or anywhere, 4) the address may
		 * not even be valid or in memory, 5) access_ok()/verify_area() can
		 * "approve" a user address which can later cause a fault, and 6) the
		 * I/O occurs into a kernel area that probably should not be mucked
		 * with.  Just say no to I/O directly into user space.
		 */

		switch (io_rq.type) {
		case DM6430HR_STR_TYPE_BYTE:
			dm_insb(io_rq.port, device_p->stream_buff_p, io_rq.times);
			break;

		case DM6430HR_STR_TYPE_WORD:
			dm_insw(io_rq.port, device_p->stream_buff_p, io_rq.times);
			break;
		}

		spin_unlock_irqrestore(&device_p->lock, flags);

		if (
			copy_to_user(
			io_rq.buf,
			device_p->stream_buff_p,
			(io_rq.times * io_rq.type)
			)
			) {
			return -EFAULT;
		}
	}

	return 0;
}


unsigned long flags;

static int
DM6430HR_IOCTL_DMA_START_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	int channel;
	int status;
	struct dm_dma_buf *buf_p;
	struct DM6430HR_DST io_rq;

#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "DM6430HR_DMA_START()\n");

#endif

	if (copy_from_user(&io_rq, (struct DM6430HR_DST *) arg, sizeof(io_rq)))
		return -EFAULT;

	status = validate_dma_circuit(io_rq.dma);
	if (status != 0) {
		return status;
	}

	if (
		!device_p->dma[io_rq.dma]
		||
		!device_p->dmabuf[io_rq.dma].addr
		||
		(io_rq.length & 1)
		||
		(io_rq.length > device_p->dmabuf[io_rq.dma].length)
		)
		return -EINVAL;

	channel = device_p->dma[io_rq.dma];
	buf_p = &device_p->dmabuf[io_rq.dma];

	//m_Dma[Channel].Initiate(&I, this, Offset, xFerLength, bDemand);
	flags = claim_dma_lock();
	//fixme FIRST DMA FLAG bug test only
	//memset(bufp->addr, 0, bufp->length);

	disable_dma(channel);
	clear_dma_ff(channel);
	set_dma_mode(channel, DMA_MODE_READ);
	set_dma_addr(channel, virt_to_bus(buf_p->addr));
	set_dma_count(channel, io_rq.length);
	enable_dma(channel);
	release_dma_lock(flags);

	return 0;
}

static int
DM6430HR_IOCTL_DMA_STOP_Handler(struct Dm6430hrDevice *device_p, ulong arg)
{
	int status;

#ifdef DEBUG

	if (debug & DBG_IOCTLS)
		printk(KERN_INFO "DM6430HR_DMA_STOP(%ld)\n", arg);

#endif

	status = validate_dma_circuit(arg);
	if (status != 0) {
		return status;
	}

	if (!device_p->dma[arg])
		return -EINVAL;

	flags = claim_dma_lock();
	disable_dma(device_p->dma[arg]);
	clear_dma_ff(device_p->dma[arg]);
	release_dma_lock(flags);

	return 0;
}

static int
DM6430HR_IOCTL_GET_IRQ_COUNTER_Handler(
	struct Dm6430hrDevice *device_p,
	ulong arg
	)
{
	struct DM6430HR_GIC interrupt_status;
	unsigned long interrupts_in_queue;
	unsigned long irq_flags;

	if (copy_from_user(&interrupt_status,
		(struct DM6430HR_GIC *) arg,
		sizeof(struct DM6430HR_GIC))) {
		return -EFAULT;
	}

	spin_lock_irqsave(&(device_p->lock), irq_flags);

	if (device_p->int_queue_out <= device_p->int_queue_out) {

		interrupts_in_queue =
			device_p->int_queue_in - device_p->int_queue_out;

	} else {

		interrupts_in_queue = (DM6430HR_INT_QUEUE_SIZE + 1) -
			(device_p->int_queue_out -
			device_p->int_queue_in);

	}

	/*
	 * If there is an interrupt in the queue then retreive the data.
	 */

	if (interrupts_in_queue > 0) {

		/*
		 * Cache local copies of the interrupt status
		 */

		interrupt_status.status =
			device_p->int_status[device_p->int_queue_out];

		/*
		 * Make copy of the calculated number of interrupts in the queue and
		 * return this value -1 to signifiy how many more interrupts the
		 * reading device needs to receive
		 */

		interrupt_status.int_remaining = interrupts_in_queue - 1;

		/*
		 * Increment the number of interrupt statuses we have sent to the user
		 */

		device_p->int_queue_out++;

		if (device_p->int_queue_out == (DM6430HR_INT_QUEUE_SIZE)) {

			/*
			 * wrap around if we have to
			 */

			device_p->int_queue_out = 0;

		}

	} else {

		/*
		 * Indicate that there are no interrupts in the queue
		 */

		interrupt_status.int_remaining = -1;
	}

	/*
	 * Pass back the number of missed interrupts regardless of its value
	 */

	interrupt_status.int_missed = device_p->int_queue_missed;

	if (interrupt_status.intr == DM6430HR_INT1) {
		interrupt_status.counter = device_p->int_count[DM6430HR_INT1];
	} else {
		interrupt_status.counter = device_p->int_count[DM6430HR_INT2];
	}

	spin_unlock_irqrestore(&(device_p->lock), irq_flags);

	/*
	 * Copy the interrupt status back to user space
	 */
	if (copy_to_user((struct DM6430HR_GIC *) arg,
		&interrupt_status, sizeof(struct DM6430HR_GIC))) {
		return -EFAULT;
	}

	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))

static int
dm6430hr_ioctl(
	struct inode *inode_p,
	struct file *file_p,
	uint cmd,
	ulong arg)

#else

static long
dm6430hr_ioctl(
	struct file *file_p,
	uint cmd,
	ulong arg)

#endif

{
	int rc = 0;
	struct Dm6430hrDevice *device_p;
	unsigned long irq_flags;

#ifdef DEBUG

	if (debug & DBG_FILEOPS)
		printk(KERN_INFO "dm6430hr_ioctl()\n");

#endif

	device_p = (struct Dm6430hrDevice *) file_p->private_data;

	if (!device_p)
		return -EINVAL;

	switch (cmd) {
	case DM6430HR_IOCTL_OUTW:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_OUTW_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_MOUTW:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_MOUTW_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_OUTB:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_OUTB_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_MOUTB:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_MOUTB_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_INW:
		if (!(file_p->f_mode & FMODE_READ))
			return -EACCES;

		rc = DM6430HR_IOCTL_INW_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_INB:
		if (!(file_p->f_mode & FMODE_READ))
			return -EACCES;

		rc = DM6430HR_IOCTL_INB_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_CLEAR:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_CLEAR_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_IRQ_ENABLE:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_IRQ_ENABLE_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_DMA_INSTALL:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_DMA_INSTALL_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_DMA_START:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_DMA_START_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_DMA_STOP:
		if (!(file_p->f_mode & FMODE_WRITE))
			return -EACCES;

		rc = DM6430HR_IOCTL_DMA_STOP_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_DMA_GETDATA:
		if (!(file_p->f_mode & FMODE_READ))
			return -EACCES;

		rc = DM6430HR_IOCTL_DMA_GETDATA_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_IRQ_REMOVE:
		spin_lock_irqsave(&device_p->lock, irq_flags);
		device_p->flags |= REMOVE_ISR;
		spin_unlock_irqrestore(&device_p->lock, irq_flags);
		wake_up_interruptible(&(device_p->int_wait_queue));
		rc = 0;
		break;
	case DM6430HR_IOCTL_DMA_GETINC:
		if (!(file_p->f_mode & FMODE_READ))
			return -EACCES;

		rc = DM6430HR_IOCTL_DMA_GETINC_Handler(device_p, arg);
		break;

	case DM6430HR_IOCTL_GET_IRQ_COUNTER:
		if (!(file_p->f_mode & FMODE_READ))
			return -EACCES;

		rc = DM6430HR_IOCTL_GET_IRQ_COUNTER_Handler(device_p, arg);
		break;

	default:
		rc = -EINVAL;
	}

	return rc;
}

static unsigned int
dm6430hr_poll(struct file *file, struct poll_table_struct *poll_table)
{
	struct Dm6430hrDevice *dev;
	unsigned int status_mask = 0;
	unsigned long irq_flags;

	dev = (struct Dm6430hrDevice *) file->private_data;

	/*
	 * Register with the file system layer so that it can wait on and check for
	 * DM6430 events
	 */

	poll_wait(file, &(dev->int_wait_queue), poll_table);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Waiting is done interruptibly, which means that a signal could have been
	   delivered.  Thus we might have been woken up by a signal before an
	   interrupt occurred.  Therefore, the process needs to examine the device's
	   interrupt flag.
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Prevent a race condition with the interrupt handler and make a local copy
	 * of the interrupt flag
	 */

	spin_lock_irqsave(&(dev->lock), irq_flags);

	if (dev->flags & REMOVE_ISR) {
		dev->flags &= ~REMOVE_ISR;
		status_mask |= (POLLIN | POLLRDNORM);
	}

	if (dev->flags & NOTIFY_IRQ) {
		dev->flags &= ~NOTIFY_IRQ;
		status_mask |= (POLLIN | POLLRDNORM);
	}

	spin_unlock_irqrestore(&(dev->lock), irq_flags);

	return status_mask;
}

static struct file_operations driver_fops = {
	.owner = THIS_MODULE,
	.open = dm6430hr_open,

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl = dm6430hr_ioctl,

#else
	.unlocked_ioctl = dm6430hr_ioctl,

#endif

	.release = dm6430hr_release,
	.poll = dm6430hr_poll
};


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))

INTERRUPT_HANDLER_TYPE dm6430hr_interrupt(
	int irq, struct Dm6430hrDevice *device_p, struct pt_regs *regs)
{
#else

INTERRUPT_HANDLER_TYPE dm6430hr_interrupt(
	int irq,
	struct Dm6430hrDevice *device_p)
{
#endif
	static short mask[] = {DM6430_CL_IRQ1, DM6430_CL_IRQ2};

	if (device_p) {
		int channel;
		unsigned long flags;
		unsigned int ints_received = 0;
		unsigned long interrupts_in_queue;
		uint16_t status;

#ifdef DEBUG

		if (debug & DBG_INT)
			printk(KERN_INFO "dm6430: hardware int\n");

#endif

		status = dm_inw_p(device_p->io + r_STATUS_6430);

		if ((device_p->irq[channel = 0] == irq) ||
			(device_p->irq[channel = 1] == irq)) {
			spin_lock_irqsave(&device_p->lock, flags);

#ifdef DEBUG

			if (debug & DBG_INT)
				printk(KERN_INFO "dm6430: int ch=%d\n", channel);

#endif


			ints_received++;
			device_p->flags |= NOTIFY_IRQ;

			/* clear interrupts */
			DM6430HRClear(device_p, mask[channel]);

			/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			   Log this interrupt in our interrupt status queue.
			   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
			/*
			 * Increment interrupt count
			 */
			device_p->int_count[channel]++;
			/*
			 * Make sure we have room in the interrupt queue
			 */
			if (device_p->int_queue_out <= device_p->int_queue_in) {

				interrupts_in_queue =
					device_p->int_queue_in - device_p->int_queue_out;

			} else {

				interrupts_in_queue = (DM6430HR_INT_QUEUE_SIZE + 1) -
					(device_p->int_queue_out -
					device_p->int_queue_in);

			}
			/*
			 * This is where the information is added to the queue if there is
			 * room otherwise we indicate queue overflow and log a missed
			 * interrupt
			 */

			if (interrupts_in_queue < DM6430HR_INT_QUEUE_SIZE) {
				/*
				 * Collect interrupt data and store in the device structure
				 */
				device_p->int_status[device_p->int_queue_in] = channel;

				device_p->int_queue_in++;

				if (device_p->int_queue_in ==
					(DM6430HR_INT_QUEUE_SIZE)) {
					/*
					 * Wrap around to the front of the queue
					 */
					device_p->int_queue_in = 0;

				}

			} else {
				/*
				 * Indicate interrupt status queue overflow
				 */
				printk(KERN_WARNING
					"dm6430: Missed interrupt info because queue is full\n");

				device_p->int_queue_missed++;

			}

			spin_unlock_irqrestore(&device_p->lock, flags);

		}

		if (ints_received > 10) {
			printk(KERN_WARNING
				"Interrupts occured faster than they could be handled!\n");
			DM6430HRClear(device_p, DM6430_CL_IRQ1);
			DM6430HRClear(device_p, DM6430_CL_IRQ2);
			return INTERRUPT_HANDLED;
		}

		if (ints_received != 0) {
			wake_up_interruptible(&(device_p->int_wait_queue));
			return INTERRUPT_HANDLED;
		} else {
			printk(KERN_EMERG "dm6430: not our interrupt\n");
			return INTERRUPT_NOT_HANDLED;
		}
	} else {
		return INTERRUPT_NOT_HANDLED;
	}
}

static void
dm6430hr_unregister_device(struct Dm6430hrDevice *device_p)
{
	int i;

	unsigned long flags;

#ifdef DEBUG

	if (debug & DBG_REG_DEVS)
		printk(KERN_INFO "dm6430hr_unregister_device()\n");

#endif

	if (device_p && (device_p->flags & INITIALIZED)) {
		spin_lock_irqsave(&device_p->lock, flags);

		for (i = 0; i < DM6430HR_DMAS; i++) {
			if (device_p->dma[i])
				free_dma(device_p->dma[i]);

			if (device_p->dmabuf[i].addr) {
				dm_dma_mem_free(
					(unsigned long) device_p->dmabuf[i].addr,
					device_p->dmabuf[i].length
					);
				device_p->dmabuf[i].length = 0;
				device_p->dmabuf[i].addr = 0;
			}
		}

		for (i = 0; i < DM6430HR_IRQS; i++) {
			if (device_p->irq[i]) {
				free_irq(device_p->irq[i], device_p);
			}
		}

		if (device_p->io)
			release_region(device_p->io, DM6430HR_IO_EXTENT);

		atomic_set(&device_p->counter, 0);

		device_p->flags &= ~INITIALIZED;

		/*
		 * Free streaming read buffer memory.
		 */

		if (device_p->stream_buff_p != NULL) {
			kfree(device_p->stream_buff_p);
		}

		spin_unlock_irqrestore(&device_p->lock, flags);
	}
}

static int
dm6430hr_register_device(struct Dm6430hrDevice *device_p)
{
	int rc = 0, i;
	void *stream_p;
	unsigned long flags;

#ifdef DEBUG

	if (debug & DBG_REG_DEVS)
		printk(
		KERN_INFO
		"dm6430hr_register_device(io=%#x irq=%d,%d dma=%d,%d)\n",
		device_p->io,
		device_p->irq[0],
		device_p->irq[1],
		device_p->dma[0],
		device_p->dma[1]
		);

#endif

	if ((device_p->flags & INITIALIZED) || !device_p->io)
		return 0;

	/*
	 * Allocate memory for streaming read buffer.  Maximum size is 2048 bytes
	 * because it can hold 1024 8-bit samples or 1024 16-bit samples.  Get the
	 * memory while no spinlock is held.
	 */

	stream_p = kmalloc(2048, GFP_KERNEL);
	if (stream_p == NULL) {
		return -ENOMEM;
	}


	if (request_region(device_p->io, DM6430HR_IO_EXTENT, name) == NULL) {
		printk(
			KERN_ERR "Unable get IO port range %#x-%#x: resource busy\n",
			device_p->io,
			(device_p->io + DM6430HR_IO_EXTENT - 1)
			);

		kfree(stream_p);
		return -EBUSY;
	}

	for (i = 0; i < DM6430HR_IRQS; i++) {
		if (
			device_p->irq[i]
			&&
			(
			rc
			=
			request_irq(
			device_p->irq[i],
			(isr_handler_t) dm6430hr_interrupt,
			0,
			name,
			device_p
			)
			)
			) {
			printk(
				KERN_ERR
				"dm6430hr: unable to get IRQ %#x (error = %d)\n",
				device_p->irq[i],
				-rc
				);

			while (--i >= 0) {
				if (device_p->irq[i]) {
					free_irq(device_p->irq[i], device_p);
				}
			}

			release_region(device_p->io, DM6430HR_IO_EXTENT);
			kfree(stream_p);
			return rc;
		}
	}

	for (i = 0; i < DM6430HR_DMAS; i++) {
		if (device_p->dma[i] && (rc = request_dma(device_p->dma[i], name))) {
			printk(
				KERN_ERR
				"dm6430hr: unable to get DMA channel %#x (error = %d)\n",
				device_p->dma[i],
				-rc
				);

			while (--i >= 0) {
				if (device_p->dma[i]) {
					free_dma(device_p->dma[i]);
				}
			}

			i = 2;
			while (--i >= 0) {
				if (device_p->irq[i]) {
					free_irq(device_p->irq[i], device_p);
				}
			}

			release_region(device_p->io, DM6430HR_IO_EXTENT);
			kfree(stream_p);
			return -EBUSY;
		}
	}
	spin_lock_irqsave(&device_p->lock, flags);
	printk(
		KERN_INFO
		"dm6430hr device successfully registered at io=%#x, "
		"irq=%d,%d dma=%d,%d\n",
		device_p->io,
		device_p->irq[0],
		device_p->irq[1],
		device_p->dma[0],
		device_p->dma[1]
		);

	DM6430HR_Initdev(device_p);
	device_p->flags |= INITIALIZED;
	device_p->flags &= ~(REMOVE_ISR & NOTIFY_IRQ);
	device_p->stream_buff_p = stream_p;
	init_waitqueue_head(&(device_p->int_wait_queue));
	spin_unlock_irqrestore(&device_p->lock, flags);
	return rc;
}

#ifdef MODULE

static unsigned short io[DM6430HR_MAX_DEVS];
static int irq1[DM6430HR_MAX_DEVS];
static int irq2[DM6430HR_MAX_DEVS];
static int dma1[DM6430HR_MAX_DEVS];
static int dma2[DM6430HR_MAX_DEVS];

MODULE_AUTHOR(DRIVER_COPYRIGHT);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_SUPPORTED_DEVICE(DRIVER_NAME);
MODULE_LICENSE("GPL");


module_param(buflength, uint, S_IRUSR | S_IRGRP | S_IROTH);
module_param_array(io, ushort, NULL, S_IRUSR | S_IRGRP | S_IROTH);
module_param_array(irq1, int, NULL, S_IRUSR | S_IRGRP | S_IROTH);
module_param_array(irq2, int, NULL, S_IRUSR | S_IRGRP | S_IROTH);
module_param_array(dma1, int, NULL, S_IRUSR | S_IRGRP | S_IROTH);
module_param_array(dma2, int, NULL, S_IRUSR | S_IRGRP | S_IROTH);


MODULE_PARM_DESC(buflength, "DMA buffer length");
MODULE_PARM_DESC(io, "I/O port base address");
MODULE_PARM_DESC(irq1, "IRQ line #1");
MODULE_PARM_DESC(irq2, "IRQ line #2");
MODULE_PARM_DESC(dma1, "DMA channel #1");
MODULE_PARM_DESC(dma2, "DMA channel #2");

#ifdef DEBUG

module_param(debug, int, S_IRUSR | S_IRGRP | S_IROTH);

MODULE_PARM_DESC(debug, "Debug level, bits field");

#endif

/******************************************************************************
validate_irq()

    Purpose:
	Determine if an IRQ number given on the insmod command line is valid
	for our purposes.  Called from init_module() only.

    Parameters:
	irq_num => IRQ number to validate.

    Return Value:
	0
	    Success or irq_num is 0.

	EINVAL
	    irq_num is not valid.
 *******************************************************************************/

static int
validate_irq(int irq_num)
{
	/* Acceptable irq, first is IRQ3 */
	static int irqs[] = {3, 0, 5, 0, 0, 0, 9, 10, 11, 12, 0, 0, 15};

	if (irq_num == 0) {
		return 0;
	}

	if ((irq_num < 3) || (irq_num > 15) || !irqs[irq_num - 3]) {
		printk(KERN_ERR "Invalid dm6430 device IRQ %u\n", irq_num);
		return -EINVAL;
	}

	return 0;
}

/******************************************************************************
validate_dma()

    Purpose:
	Determine if a DMA channel number given on the insmod command line is
	valid for our purposes.  Called from init_module() only.

    Parameters:
	dma_channel => DMA channel number to validate.

    Return Value:
	0
	    Success or dma_channel is 0.

	EINVAL
	    dma_channel is not valid.
 *******************************************************************************/

static int
validate_dma_channel(int dma_channel)
{
	if (dma_channel == 0) {
		return 0;
	}

	if ((dma_channel < 5) || (dma_channel > 7)) {
		printk(KERN_ERR "Invalid dm6430 device DMA channel %u\n", dma_channel);
		return -EINVAL;
	}

	return 0;
}

static int dm6430_register_char_device(unsigned int *major)
{

	dev_t device, devno;
	int status;
	struct device *dev = NULL;
	char dev_file_name[30];
	int minor = 0;

	status =
		alloc_chrdev_region(&device, 0, dm6430_device_count, DRIVER_NAME);
	if (status < 0) {
		return status;
	}

	cdev_init(&dm6430_cdev, &driver_fops);
	dm6430_cdev.owner = THIS_MODULE;

	status = cdev_add(&dm6430_cdev, device, dm6430_device_count);
	if (status < 0) {
		unregister_chrdev_region(device, dm6430_device_count);
		return status;
	}

	*major = MAJOR(device);

	dev_class = class_create(THIS_MODULE, DRIVER_NAME);

	if (dev_class == NULL) {
		unregister_chrdev_region(device, dm6430_device_count);
		return -ENODEV;
	}

	for (minor = 0; minor < dm6430_device_count; minor++) {
		sprintf(dev_file_name, "%s-%u", DRIVER_NAME, minor);
		devno = MKDEV(*major, minor);
		dev = device_create(dev_class,
			NULL, devno, NULL, dev_file_name, 0);

		if (dev == NULL) {
			return -ENODEV;
		}
	}

	return 0;
}

static void dm6430_unregister_char_device(void)
{

	unsigned int minor;

	cdev_del(&dm6430_cdev);

	for (minor = 0; minor < dm6430_device_count; minor++) {
		device_destroy(dev_class, MKDEV(dm6430_major, minor));

	}

	class_unregister(dev_class);

	class_destroy(dev_class);

	unregister_chrdev_region(MKDEV(dm6430_major, 0), dm6430_device_count);


}

int __init
DM6430_init_module(void)
{
	int device_num;
	int rc;
	int device_present = 0;

#ifdef DEBUG

	if (debug & DBG_LOAD)
		printk(KERN_INFO "dm6430hr init_module()\n");

#endif

	/*
	 * Validate base I/O addresses, IRQs, and DMA channels before making any
	 * changes.  This makes it easier to keep the driver in a consistent state
	 * in case something goes wrong.
	 */

	for (device_num = 0; device_num < DM6430HR_MAX_DEVS; device_num++) {
		unsigned short io_addr;

		/*
		 * No base I/O address means no device present
		 */

		io_addr = io[device_num];

		if (!io_addr)
			continue;

		device_present = 1;

		/*
		 * Validate base I/O address
		 */

		if ((io_addr < 0x200) || (io_addr > 0x3E0) || (io_addr & 0xF)) {
			printk(
				KERN_ERR "Invalid dm6430 device base I/O address 0x%04x\n",
				io_addr
				);
			return -EINVAL;
		}

		/*
		 * Validate IRQ numbers
		 */

		rc = validate_irq(irq1[device_num]);
		if (rc != 0) {
			return rc;
		}

		rc = validate_irq(irq2[device_num]);
		if (rc != 0) {
			return rc;
		}

		/*
		 * Validate DMA channels
		 */

		rc = validate_dma_channel(dma1[device_num]);
		if (rc != 0) {
			return rc;
		}

		rc = validate_dma_channel(dma2[device_num]);
		if (rc != 0) {
			return rc;
		}
		dm6430_device_count++;
	}

	if (device_present == 0) {
		printk(KERN_ERR "No DM6430 devices configured.\n");
		return -ENODEV;
	}

	/*
	 * We've done as much validation as possible, now reserve a character major
	 * number
	 */

	rc = dm6430_register_char_device(&dm6430_major);

	if (rc < 0) {
		printk(
			KERN_ERR
			"Major %u character device registration failed, errno = %d\n",
			dm6430_major,
			-rc
			);

		return rc;
	}

	printk(KERN_INFO "Registered dm6430 device with major %d\n", dm6430_major);



	/*
	 * Set up the device structure for each possible board and register each
	 * device
	 */

	for (device_num = 0; device_num < DM6430HR_MAX_DEVS; device_num++) {
		struct Dm6430hrDevice *device_p = &devices[device_num];

		atomic_set(&device_p->counter, 0);
		spin_lock_init(&device_p->lock);
		device_p->io = io[device_num];
		device_p->irq[0] = irq1[device_num];
		device_p->irq[1] = irq2[device_num];
		device_p->dma[0] = dma1[device_num];
		device_p->dma[1] = dma2[device_num];
		device_p->stream_buff_p = NULL;

		rc = dm6430hr_register_device(device_p);
		if (rc != 0) {

			/*
			 * Registration of current device failed, so completely
			 * clean up by unregistering any device successfully registered
			 * so far and unreserving the character major number
			 */

			printk(
				KERN_ERR
				"dm6430 device #%d registration failed.\n",
				device_num
				);

			while (--device_num > 0) {
				(void) dm6430hr_unregister_device(&devices[device_num]);
			}
			dm6430_unregister_char_device();
			return rc;
		}
	}


#ifdef DEBUG

	if (debug & DBG_LOAD)
		printk(KERN_INFO "dm6430hr init_module() done\n");

#endif

	return 0;
}

void __exit
DM6430_cleanup_module(void)
{
	int devno;

#ifdef DEBUG

	if (debug & DBG_LOAD)
		printk(KERN_INFO "dm6430hr cleanup_module()\n");

#endif

	dm6430_unregister_char_device();
	for (devno = 0; devno < DM6430HR_MAX_DEVS; devno++) {
		dm6430hr_unregister_device(&devices[devno]);
	}


#ifdef DEBUG

	if (debug & DBG_LOAD)
		printk(KERN_INFO "dm6430hr cleanup_module() done\n");

#endif

}

module_init(DM6430_init_module);
module_exit(DM6430_cleanup_module);

#endif /* MODULE */
