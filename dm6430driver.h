/*
	FILE NAME: dm6430driver.h

	FILE DESCRIPTION: This header file contains driver definitions

	PROJECT NAME: Linux DM6430 Driver, Library, and Example Programs

	PROJECT VERSION: (Defined in README.TXT)

	Copyright 2004 RTD Embedded Technologies, Inc.  All Rights Reserved.
*/

#ifndef __dm6430driver_h__
#define __dm6430driver_h__

#include <linux/module.h>
#include <linux/spinlock.h>

#include <asm/byteorder.h>
#include <asm/atomic.h>


/*
 * Number of interrupt circuits on the board
 */

#define DM6430HR_IRQS		2


/*
 * Number of DMA circuits on the board
 */

#define DM6430HR_DMAS		2

/*
 * Max number of entries allowed in the interrupt status queue
 */

#define DM6430HR_INT_QUEUE_SIZE   15


/*
 * Define a type to be used for kernel driver interrupt handler
 */


#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
	#define INTERRUPT_HANDLER_TYPE static irqreturn_t
	#define INTERRUPT_HANDLED   IRQ_HANDLED
	#define INTERRUPT_NOT_HANDLED   IRQ_NONE
	typedef irqreturn_t (*isr_handler_t) (int, void *, struct pt_regs *);
#else
	#define INTERRUPT_HANDLER_TYPE static irqreturn_t
	#define INTERRUPT_HANDLED   IRQ_HANDLED
	#define INTERRUPT_NOT_HANDLED   IRQ_NONE
	typedef irqreturn_t (*isr_handler_t) (int, void *);
#endif



/*******************************************************************************
Flags which indicate what debugging information is printed when kernel driver
is built with -DDEBUG.  The "debug=" argument on the insmod command controls
the information output.
*******************************************************************************/

enum DBG_FLAGS {

    /*
     * Print debugging information about driver module load/unload
     */

    DBG_LOAD		=	0x00001,

    /*
     * Print debugging information about device registration/unregistration
     */

    DBG_REG_DEVS	=	0x00002,

    /*
     * Print debugging information about close(), ioctl(), and open()
     * operations on device files
     */

    DBG_FILEOPS		=	0x00004,

    /*
     * Print debugging information about ioctl() operations on device files
     */

    DBG_IOCTLS		=	0x00008,

    /*
     * Print debugging information about reads/writes to board I/O space
     */

    DBG_DEV		=	0x00010,

    /*
     * Print debugging information about interrupts
     */

    DBG_INT		=	0x00020
};


/*
 * Length of board's I/O space in bytes
 */

#define DM6430HR_IO_EXTENT	32


/*******************************************************************************
Device flags
*******************************************************************************/

enum FLAGS {

    /*
     * Indicate that a particular device has been initialized
     */

    INITIALIZED = 0x01,

    /*
     * Indicate that an interrupt has occurred and user space notification
     * must take place
     */

    NOTIFY_IRQ = 0x02,

    /*
     * Indicate a user space request to uinstall the IRQ handler
     */

    REMOVE_ISR = 0x04
};

/*******************************************************************************
DMA buffer descriptor structure
*******************************************************************************/

struct dm_dma_buf {

    /*
     * Address of DMA buffer
     */

    char *		addr;

    /*
     * Length of DMA buffer in bytes
     */

    size_t		length;
};


/*******************************************************************************
Driver DM6430 device structure
*******************************************************************************/

struct Dm6430hrDevice{

    /*
     * Base I/O address of device
     */

    int 		io;

    /*
     * IRQ number assigned to each interrupt circuit
     */

    int 		irq[DM6430HR_IRQS];

    /*
     * DMA channel number assigned to each DMA circuit
     */

    int			dma[DM6430HR_DMAS];

    /*
     * DMA buffer descriptor for each DMA circuit
     */

    struct dm_dma_buf	dmabuf[DM6430HR_DMAS];

    /*
     * Interrupt status queue
     */

	uint32_t int_status[DM6430HR_INT_QUEUE_SIZE];

    /*
     * Number of entries in the interrupt status queue
     */

	unsigned int int_queue_in;

    /*
     * Number of entries read from the interrupt status queue
     */

	unsigned int int_queue_out;

    /*
     * Number of interrupts missed because of a full queue
     */

	unsigned int int_queue_missed;

    /*
     * Number of interrupts that have occurred on the board
     */

	unsigned int int_count[DM6430HR_IRQS];

    /*
     * Number of entities having a device file open
     */

    atomic_t		counter;

    /*
     * Device access serialization spin lock
     */

    spinlock_t		lock;

    /*
     * Device flags
     */

    long		flags;

    /*
     * Temporary register storage
     */

    unsigned short 	Control_Register,
			Trigger_Register,
			IRQ_Register,
			DIN_Register;

    /*
     * Pointer to read buffer used in streaming input from A/D FIFO or
     * digital FIFO.  This is allocated in dm6430hr_register_device() and
     * freed in dm6430hr_unregister_device().
     */

    void		*stream_buff_p;

    /*
     * Wait queue for select() interrupt notification.
     */

    wait_queue_head_t int_wait_queue;
};

#endif /* __dm6430driver_h__ */
