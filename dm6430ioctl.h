/*
	FILE NAME: dm6430ioctl.h

	FILE DESCRIPTION: This header file defines the driver ioctl interface

	PROJECT NAME: Linux DM6430 Driver, Library, and Example Programs

	PROJECT VERSION: (Defined in README.TXT)

	Copyright 2004 RTD Embedded Technologies, Inc.  All Rights Reserved.
*/

#ifndef __dm6430ioctl__h_
#define __dm6430ioctl__h_

#include <linux/ioctl.h>
#include <linux/types.h>

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Signal sender code
 */

#define DM6430HR_SI_CODE		SI_MESGQ


/*
 * 8 megahertz clock frequency
 */

#define DM6430HR_8MHZ			8000000


/*******************************************************************************
Board registers
*******************************************************************************/

enum DM6430HR_Regs {

    r_CLEAR_6430		= 0,	// Clear Register (Read/Write)
    r_STATUS_6430		= 2,	// Status Register (Read)
    r_CONTROL_6430		= 2,	// Control Register (Write)
    r_AD_6430			= 4,	// AD Data (Read)
    r_CHANNEL_GAIN_6430		= 4,	// Channel/Gain Register (Write)
    r_AD_TABLE_6430		= 4,	// AD Table (Write)
    r_DIGITAL_TABLE_6430	= 4,	// Digital Table (Write)
    r_START_CONVERSION_6430	= 6,	// Start Conversion (Read)
    r_TRIGGER_6430		= 6,	// Trigger Register (Write)
    r_IRQ_6430			= 8,	// IRQ Register (Write)
    r_DIN_FIFO_6430		= 10,	// Digital Input FIFO Data (Read)
    r_DIN_CONFIG_6430		= 10,	// Config Digital Input FIFO (Write)
    r_DAC1_6430			= 12,	// DAC 1 Data (Write)
    r_LOAD_AD_SAMPLE_COUNT_6430	= 14,	// Load A/D Sample Counter (Read)
    r_DAC2_6430			= 14,	// DAC 2 Data (Write)
    r_TIMER_CLCK0_6430		= 16,	// Timer/Counter 0 (Read/Write)
    r_TIMER_CLCK1_6430		= 18,	// Timer/Counter 1 (Read/Write)
    r_TIMER_CLCK2_6430		= 20,	// Timer/Counter 2 (Read/Write)
    r_TIMER_CTRL_6430		= 22,	// Timer/Counter Control Word (Write)

    r_DIO_PORT_0_6430		= 24,	// Digital I/O Port 0 Register
    r_DIO_PORT_1_6430		= 26,	// Digital I/O Port 1 Register

    r_DIO_PORT_DIR_6430		= 28,	// Digital I/O Port 0 Direction Register
    r_DIO_PORT_MASK_6430	= 28,	// Digital I/O Port 0 Mask Register
    r_DIO_PORT_COMP_6430	= 28,	// Digital I/O Port 0 Compare Register
    r_DIO_CLEAR_6430		= 28,	// Digital I/O Clear Register
    r_DIO_STATUS_6430		= 30,	// Digital I/O Status Register
    r_DIO_MODE_6430		= 30 	// Digital I/O Mode Register
};


/*******************************************************************************
Registers which support streaming read
*******************************************************************************/

enum DM6430HR_STR_Regs {
    rSTR_AD_6430		= r_AD_6430,		// AD Data (Read)
    rSTR_DIN_FIFO_6430		= r_DIN_FIFO_6430	// Digital Input FIFO Data (Read)    
};


/*******************************************************************************
Streaming read transfer types
*******************************************************************************/

enum DM6430HR_STR_TYPE {
    DM6430HR_STR_TYPE_BYTE = 1,		/* 8 bits */
    DM6430HR_STR_TYPE_WORD		/* 16 bits */
};


/* clear flags codes for r_CLEAR_6430 reg */
enum DM6430HR_CF {
    DM6430_CL_BOARD       = 0x0001,
    DM6430_CL_AD_FIFO	  = 0x0002,
    DM6430_CL_AD_DMA_DONE = 0x0004,
    DM6430_CL_CLEAR_GAIN  = 0x0008,
    DM6430_CL_RESET_GAIN  = 0x0010,    
    DM6430_CL_DIO_FIFO 	  = 0x0020,
    DM6430_CL_IRQ1 	  = 0x0040,
    DM6430_CL_IRQ2 	  = 0x0080,
};


/*******************************************************************************
Bit masks used to access individual bits in Digital I/O Status Register
*******************************************************************************/

/*
 * Mask to extract digital port 1 direction bit
 */

#define DIO_STATUS_PORT1_DIRECTION 0x04

/*
 * Mask to extract digital interrupt mode bit
 */

#define DIO_STATUS_IRQ_MODE 0x08

/*
 * Mask to extract digital interrupt enabled status bit
 */

#define DIO_STATUS_IRQ_ENABLE 0x10

/*
 * Mask to extract digital input sample clock source bit
 */

#define DIO_STATUS_SAMPLE_CLOCK 0x20

/*
 * Mask to extract digital interrupt status bit
 */

#define DIO_STATUS_IRQ_STATUS 0x40

/*
 * Mask to extract
 */

#define DIO_STATUS_STROBE_STATUS 0x80


/* struct for DM6430HR_IOCTL_INB/DM6430HR_IOCTL_OUTB */
struct DM6430HR_IO8 {

   /*
    * Target register
    */

    enum DM6430HR_Regs	reg;

    /*
     * Value to write to register or value read from register
     */

    u_int8_t		value;
};


/* struct for DM6430HR_IOCTL_MOUTW */
struct DM6430HR_MIO8 {

   /*
    * Target register
    */

    enum DM6430HR_Regs	reg;

    /*
     * Mask which controls which register bits are changeable
     */

    u_int8_t		mask;

    /*
     * Value to write to register
     */

    u_int8_t		value;
};


/* struct for DM6430HR_IOCTL_INW/DM6430HR_IOCTL_OUTW */
struct DM6430HR_IO16 {

   /*
    * Target register
    */

    enum DM6430HR_Regs	reg;

    /*
     * Value to write to register or value read from register
     */

    u_int16_t		value;
};


/* struct for DM6430HR_IOCTL_MOUTW */
struct DM6430HR_MIO16 {

   /*
    * Target register
    */

    enum DM6430HR_Regs	reg;

    /*
     * Mask which controls which register bits are changeable
     */

    u_int16_t		mask;

    /*
     * Value to write to register
     */

    u_int16_t		value;
};


/* timer counters of DM6430HR */
enum DM6430HR_CLK {

    /*
     * Timer/Counter 0
     */

    DM6430HR_CLK0,

    /*
     * Timer/Counter 1
     */

    DM6430HR_CLK1,

    /*
     * Timer/Counter 2
     */

    DM6430HR_CLK2
};


/* DMA channels of DM6430HR */
enum DM6430HR_DMA {

    /*
     * First DMA circuit on board
     */

    DM6430HR_DMA1,

    /*
     * Second DMA circuit on board
     */

    DM6430HR_DMA2
};


/* IRQ channels of DM6430HR */
enum DM6430HR_INT {

    /*
     * First interrupt circuit on board
     */

    DM6430HR_INT1,

    /*
     * Second interrupt circuit on board
     */

    DM6430HR_INT2
};


/* Clock modes, r_TIMER_CTRL_6430 = BA + 22 */
enum DM6430HR_CLK_MODE  {

    /*
     * Event count
     */

    DM6430HR_CLK_MODE0, 

    /*
     * Programmable 1-shot
     */

    DM6430HR_CLK_MODE1, 

    /*
     * Rate generator
     */

    DM6430HR_CLK_MODE2, 

    /*
     * Square wave rate generator
     */

    DM6430HR_CLK_MODE3, 

    /*
     * Software triggered strobe
     */

    DM6430HR_CLK_MODE4, 

    /*
     * Hardware triggered strobe
     */

    DM6430HR_CLK_MODE5
};


/* Timer/Clock Select r_CONTROL_6430 = BA+2 */
enum DM6430HR_CLK_SEL {

    /*
     * Pacer or burst clock
     */

    DM6430HR_CLOCK_TC, 

    /*
     * A/D sample counter or user timer/counter
     */

    DM6430HR_USER_TC
};


/*******************************************************************************
Advanced digital interrupt modes
*******************************************************************************/

enum DM6430HR_DIO_IRQ {

    /*
     * Event mode - any port 0 bit changes state
     */

    DM6430HR_DIO_IRQ_EVENT,

    /*
     * Match mode - port 0 bit pattern matches Compare Register value
     */

    DM6430HR_DIO_IRQ_MATCH
};


/*******************************************************************************
Control flags for operation of Read/Program Port Direction/Mask/Compare
Register at base I/O address offset 28
*******************************************************************************/

enum DM6430HR_REG_SEL {

    /*
     * Put register at offset 28 into Clear Mode
     */

    DM6430HR_REG_CLEAR,

    /*
     * Configure register at offset 28 to be Direction Register
     */

    DM6430HR_REG_DIR,

    /*
     * Configure register at offset 28 to be Mask Register
     */

    DM6430HR_REG_MASK,

    /*
     * Configure register at offset 28 to be Compare Register
     */

    DM6430HR_REG_CMP
};


/*******************************************************************************
Digital I/O port selection
*******************************************************************************/

enum DM6430HR_DIO {

    /*
     * Digital I/O port 0
     */

    DM6430HR_DIO0,

    /*
     * Digital I/O port 1
     */

    DM6430HR_DIO1
};


/* Analog inputs Channel, r_CHANNEL_GAIN_6430 - BA+4 */
enum DM6430HR_AIN {
    DM6430HR_AIN1 = 0,		// Channel 1
    DM6430HR_AIN2,		// Channel 2
    DM6430HR_AIN3,		// Channel 3
    DM6430HR_AIN4,		// Channel 4
    DM6430HR_AIN5,		// Channel 5
    DM6430HR_AIN6,		// Channel 6
    DM6430HR_AIN7,		// Channel 7
    DM6430HR_AIN8,		// Channel 8
    DM6430HR_AIN9,		// Channel 9
    DM6430HR_AIN10,		// Channel 10
    DM6430HR_AIN11,		// Channel 11
    DM6430HR_AIN12,		// Channel 12
    DM6430HR_AIN13,		// Channel 13
    DM6430HR_AIN14,		// Channel 14
    DM6430HR_AIN15,		// Channel 15
    DM6430HR_AIN16		// Channel 16
};


/* Gain select, r_CHANNEL_GAIN_6430 - BA+4 */
enum DM6430HR_GAIN {
    DM6430HR_GAINx1 = 0,	// Gain multiplier of 1
    DM6430HR_GAINx2,		// Gain multiplier of 2
    DM6430HR_GAINx4,		// Gain multiplier of 4
    DM6430HR_GAINx8		// Gain multiplier of 8
};


/* A/D SE/DIFF , r_CHANNEL_GAIN_6430 - BA+4 */
enum DM6430HR_SE {
    DM6430HR_SE_SE = 0,		// Set single ended analog inputs
    DM6430HR_SE_DIFF		// Set differential analog inputs
};


/* Conversion Select, r_TRIGGER_6430 = BA+6 */
enum DM6430HR_CONV {
    DM6430HR_CONV_SOFT_TRIGGER = 0,	// Software conversion control
    DM6430HR_CONV_PACER_CLOCK,		// Pacer clock conversion control
    DM6430HR_CONV_BURST_CLOCK,		// Burst clock conversion control
    DM6430HR_CONV_DIGITAL_INT		// Digital interrupt conversion control
};


/* Start Trigger Select,  r_TRIGGER_6430 = BA+6 */
enum DM6430HR_START_TRIG {
    DM6430HR_START_TRIG_SOFTWARE = 0,	// Software trigger start
    DM6430HR_START_TRIG_EXTERNAL,	// External trigger start
    DM6430HR_START_TRIG_DIGITAL_INT,	// Digital interrupt trigger start
    DM6430HR_START_TRIG_USER_TC1,	// User Timer/Counter 1 countdown trigger start
    DM6430HR_START_TRIG_RES1,		// Not supported
    DM6430HR_START_TRIG_RES2,		// Not supported
    DM6430HR_START_TRIG_RES3,		// Not supported
    DM6430HR_START_TRIG_GATE		// TRIGGER IN line trigger start
};


/* Stop Trigger Select,  r_TRIGGER_6430 = BA+6 */
enum DM6430HR_STOP_TRIG {
    DM6430HR_STOP_TRIG_SOFTWARE = 0,	// Software trigger stop
    DM6430HR_STOP_TRIG_EXTERNAL,	// External trigger stop
    DM6430HR_STOP_TRIG_DIGITAL_INT,	// Digital interrupt trigger stop
    DM6430HR_STOP_TRIG_SAMPLE_CNT,	// Sample counter countdown trigger stop
    DM6430HR_STOP_TRIG_ABOUT_SOFTWARE,	// Extra samples after software conversion
    DM6430HR_STOP_TRIG_ABOUT_EXTERNAL,	// Extra samples after external trigger
    DM6430HR_STOP_TRIG_ABOUT_DIGITAL,	// Extra samples after digital interrupt
    DM6430HR_STOP_TRIG_ABOUT_USER_TC1	// Extra samples after User Timer/Counter countdown
};


/* Pacer Clock Select, r_TRIGGER_6430 = BA+6 */
enum DM6430HR_PACER_CLK {
    DM6430HR_PACER_CLK_INTERNAL,	// Internal pacer clock
    DM6430HR_PACER_CLK_EXTERNAL		// External pacer clock
};


/* Burst Trigger Select, r_TRIGGER_6430 = BA+6 */
enum DM6430HR_BURST_TRIG {
    DM6430HR_BURST_TRIG_SOFTWARE = 0,	// Start Conversion trigger
    DM6430HR_BURST_TRIG_PACER,		// Pacer clock trigger
    DM6430HR_BURST_TRIG_EXTERNAL,	// External trigger
    DM6430HR_BURST_TRIG_DIGITAL		// Digital interrupt trigger
};


/* Trigger polarity, r_TRIGGER_6430 = BA+6 */
enum DM6430HR_POLAR {
    DM6430HR_POLAR_POSITIVE = 0,	// Positive edge external trigger
    DM6430HR_POLAR_NEGATIVE		// Negative edge external trigger
};


/* Trigger repeat, r_TRIGGER_6430 = BA+6 */
enum DM6430HR_REPEAT {
    DM6430HR_REPEAT_SINGLE = 0,		// Perform one conversion cycle
    DM6430HR_REPEAT_REPEAT		// Start conversion cycle on each trigger
};


/* Digital Input FIFO Clock, r_DIN_CONFIG_6430 = BA+10 */
enum DM6430HR_DI_FIFO_CLK {
    DM6430HR_DI_FIFO_CLK_USER_TC0 = 0,		// User Timer/Counter 0 output
    DM6430HR_DI_FIFO_CLK_USER_TC1,		// User Timer/Counter 1 output
    DM6430HR_DI_FIFO_CLK_AD_WRITE_FIFO,		// Write pulse to A/D FIFO
    DM6430HR_DI_FIFO_CLK_EXTERNAL_PACER,	// External pacer clock
    DM6430HR_DI_FIFO_CLK_EXTERNAL_TRIG		// External trigger
};


/* struct for DM6430HR_IOCTL_IRQ_ENABLE */
struct DM6430HR_IE {
    enum DM6430HR_INT	intr;		// Interrupt circuit
    int			action;		// Enable/disable flag
};


/* struct for DM6430HR_IOCTL_GET_IRQ_COUNTER */
struct DM6430HR_GIC {
    enum DM6430HR_INT	intr;		// Interrupt circuit
    unsigned long	counter;	// Interrupt count
    unsigned long int_remaining;
    unsigned long int_missed;
    unsigned long status;
};


/* IRQ sources of DM6430HR */
enum DM6430HR_INTSRC {
    IRQS_AD_SAMPLE_CNT_6430	= 0,	// A/D sample counter
    IRQS_AD_START_CONVERT_6430,		// A/D start convert
    IRQS_AD_END_CONVERT_6430,		// A/D end-of-convert
    IRQS_AD_WRITE_FIFO_6430,		// A/D write FIFO
    IRQS_AD_FIFO_HALF_6430,		// A/D FIFO half-full
    IRQS_AD_DMA_DONE_6430,		// A/D DMA done
    IRQS_RESET_GAIN_TABLE_6430,		// reset channel-gain table
    IRQS_PAUSE_GAIN_TABLE_6430,		// pause channel-gain table
    IRQS_EXT_PACER_CLOCK_6430,		// external pacer clock
    IRQS_EXT_TRIGGER_6430,		// external trigger
    IRQS_DIGITAL_6430,			// Digital interrupt
    IRQS_TC_COUNTER0_6430,		// User TC counter 0 out
    IRQS_TC_COUNTER0_INVERTED_6430,	// User TC counter 0 out inverted
    IRQS_TC_COUNTER1_6430,		// User TC counter 1 out
    IRQS_DIO_FIFO_HALF_6430,		// Digital input FIFO half-full
    IRQS_DIO_WRITE_FIFO_6430,		// Digital input write FIFO
};


/* struct for DM6430HR_IOCTL_DMA_INSTALL */
struct DM6430HR_DI {
    enum DM6430HR_DMA	dma;		// DMA circuit
    int			action;		// Install/uninstall flag
};


/* struct for DM6430HR_IOCTL_DMA_START */
struct DM6430HR_DST {
    enum DM6430HR_DMA		dma;	// DMA circuit
    size_t			length;	// Number of bytes to transfer
};


/* struct for DM6430HR_IOCTL_DMA_GETDATA */
struct DM6430HR_GDD {
    enum DM6430HR_DMA           dma;	// DMA circuit
    void *			buf;	// Address of buffer to store data in
    size_t			length; // Number of bytes to transfer
    size_t			offset;	// Offset in bytes from DMA buffer start
};


/* struct for DM6430HR_IOCTL_DMA_GETINC */
struct DM6430HR_GID {
    enum DM6430HR_STR_Regs	port;	// Register to read from
    enum DM6430HR_STR_TYPE	type;	// Type of transfer
    void *			buf;	// Address of buffer to store data in
    size_t			times;	// Number of entities to read
};


/*******************************************************************************
Callback routine descriptor structure.  This is used internally by the library
to set up callback notification of interrupt occurrence.
*******************************************************************************/

/* ioctl codes definition */
#define  __DM6430HR_IOCTL_ID_LETTER 	0x26

#define DM6430HR_IOCTL_INB			_IOWR(__DM6430HR_IOCTL_ID_LETTER, 0xC00, struct DM6430HR_IO8 )
#define DM6430HR_IOCTL_OUTB			_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC01, struct DM6430HR_IO8 )
#define DM6430HR_IOCTL_MOUTB			_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC02, struct DM6430HR_MIO8 )
#define DM6430HR_IOCTL_INW			_IOWR(__DM6430HR_IOCTL_ID_LETTER, 0xC03, struct DM6430HR_IO16 )
#define DM6430HR_IOCTL_OUTW			_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC04, struct DM6430HR_IO16 )
#define DM6430HR_IOCTL_MOUTW			_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC05, struct DM6430HR_MIO16)
#define DM6430HR_IOCTL_CLEAR			_IO  (__DM6430HR_IOCTL_ID_LETTER, 0xC06 )
#define DM6430HR_IOCTL_IRQ_ENABLE		_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC07, struct DM6430HR_IE )
#define DM6430HR_IOCTL_GET_IRQ_COUNTER		_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC08, struct DM6430HR_GIC)
#define DM6430HR_IOCTL_DMA_INSTALL		_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC09, struct DM6430HR_DI )
#define DM6430HR_IOCTL_DMA_GETDATA		_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC0A, struct DM6430HR_GDD)
#define DM6430HR_IOCTL_DMA_START		_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC0B, struct DM6430HR_DST)
#define DM6430HR_IOCTL_DMA_STOP			_IO  (__DM6430HR_IOCTL_ID_LETTER, 0xC0C )
#define DM6430HR_IOCTL_IRQ_REMOVE       _IO  (__DM6430HR_IOCTL_ID_LETTER, 0xC0D )
#define DM6430HR_IOCTL_DMA_GETINC		_IOW (__DM6430HR_IOCTL_ID_LETTER, 0xC0E, struct DM6430HR_GID)


/*
 * DM6430 device file name wthout the minor number suffix
 */

#define DM6430HR_DEVICE_BASENAME	"/dev/rtd-dm6430"


/*
 * Maximum number of DM6430 devices
 */

#define DM6430HR_MAX_DEVS		4

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __dm6430ioctl__h_ */
