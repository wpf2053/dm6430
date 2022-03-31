/*
	FILE NAME: dm6430lib.h

	FILE DESCRIPTION:	This file contains structure definitions and
				function prototypes for the user level library
				and application programs.

	PROJECT NAME: Linux DM6430 Driver, Library, and Example Programs

	PROJECT VERSION: (Defined in README.TXT)

	Copyright 2004 RTD Embedded Technologies, Inc.  All Rights Reserved.
*/

#ifndef __dm6430lib__h
#define __dm6430lib__h

#include <sys/types.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sys/wait.h>

#include <dm6430ioctl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Structure for DM6430 device */
typedef struct Dm6430HR_Device_Descriptor {
    int descriptor;
    void (*isr) (unsigned int status);
    pthread_t pid;
    int thread_status;
} Dm6430HR_Device_Descriptor;

/* Structure for A/D Table */
typedef struct _ADTableRow {
    enum DM6430HR_AIN	Channel;
    enum DM6430HR_GAIN	Gain;
    enum DM6430HR_SE	Se_Diff;
    int			Pause;
    int			Skip;
} ADTableRow;


/******************************************************************************
OpenBoard6430()

    Purpose:
        Open a DM6430 device file.

    Parameters:
        DeviceNumber => Minor number of board device file.

    Return Value:
        >=0
            Success.  The integer returned is the file descriptor from open()
            system call.

        -1
            Failure.  Please see the open(2) man page for information on
            possible values errno may have in this case.
 ******************************************************************************/

int OpenBoard6430(int DeviceNumber, struct Dm6430HR_Device_Descriptor **board);


/******************************************************************************
CloseBoard6430()

    Purpose:
        Close a DM6430 device file opened previously with OpenBoard6430().

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure.  Please see the close(2) man page for information on
            possible values errno may have in this case.
 ******************************************************************************/

int CloseBoard6430(struct Dm6430HR_Device_Descriptor **board);


/******************************************************************************
InstallCallbackIRQHandler6430()

    Purpose:
        Install a function which will be called whenever an interrupt occurs
        and the driver sends a signal to the process to indicate that the
        interrupt happened.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	callback ===> Address of callback function.
	IRQChannel => Board interrupt circuit that signal should be attached to.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    callback is NULL.

                EINVAL
                    IRQChannel is not valid.

                ENOMEM
                    Library callback descriptor memory could not be allocated.

            Please see the sigaction(2) man page, the sigprocmask(2) man page,
            or the ioctl(2) man page for information on other possible values
            errno may have in this case.
 ******************************************************************************/

int InstallCallbackIRQHandler6430(
    struct Dm6430HR_Device_Descriptor *board,
    void (*callback)(void),
    enum DM6430HR_INT IRQChannel
);


/******************************************************************************
RemoveIRQHandler6430()

    Purpose:
        Uninstall the function which was previously registered as an interrupt
        callback by InstallCallbackIRQHandler6430().

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	IRQChannel => Board interrupt circuit that signal should be detached
                      from.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    IRQChannel is not valid.

                EINVAL
                    No IRQ was ever allocated to IRQChannel.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int RemoveIRQHandler6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_INT IRQChannel);


/******************************************************************************
ClearRegister6430()

    Purpose:
        Write a bit mask into a board's Program Clear Register and then read
        from the Clear Register to clear some part(s) of the board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	ClearValue => Bit mask to write into Program Clear Register before
                      reading Clear Register.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClearRegister6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t ClearValue);


/******************************************************************************
ClearBoard6430()

    Purpose:
        Reset a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClearBoard6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
ClearADFIFO6430()

    Purpose:
        Clear a board's A/D sample FIFO.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClearADFIFO6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
ClearADDMADone6430()

    Purpose:
        Clear a board's A/D DMA done flag.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClearADDMADone6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
ClearChannelGainTable6430()

    Purpose:
        Clear the contents of a board's channel/gain table.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClearChannelGainTable6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
ResetChannelGainTable6430()

    Purpose:
        Reset a board's channel/gain table starting point to the beginning of
        the table.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ResetChannelGainTable6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
ClearDINFIFO6430()

    Purpose:
        Clear a board's digital input FIFO.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClearDINFIFO6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
ClearIRQ06430()

    Purpose:
        Clear the first interrupt circuit on a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClearIRQ06430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
ClearIRQ16430()

    Purpose:
        Clear the second interrupt circuit on a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClearIRQ16430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
InitBoard6430()

    Purpose:
        Initialize a board.  This will 1) clear the board, 2) clear the A/D
        DMA done flag, 3) clear the channel/gain table, and 4) clear the A/D
        input FIFO.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int InitBoard6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
EnableIRQ6430()

    Purpose:
        Enable the specified interrupt circuit on a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        IRQChannel => Interrupt circuit to enable.  Valid values are
                      DM6430HR_INT1 and DM6430HR_INT2.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    IRQChannel is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int EnableIRQ6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_INT IRQChannel);


/******************************************************************************
DisableIRQ6430()

    Purpose:
        Disable the specified interrupt circuit on a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        IRQChannel => Interrupt circuit to disable.  Valid values are
                      DM6430HR_INT1 and DM6430HR_INT2.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    IRQChannel is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DisableIRQ6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_INT IRQChannel);


/******************************************************************************
GetIRQCounter6430()

    Purpose:
        Get the number of interrupts that have occurred on a board's specified
        interrupt circuit.

    Parameters:
        descriptor ======> File descriptor from OpenBoard6430() call.
        IRQChannel ======> Interrupt circuit to read counter value from.
                           Valid values are DM6430HR_INT1 and DM6430HR_INT2.
        counter_value_p => Address where counter value should be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

                EINVAL
                    IRQChannel is not valid.

                EINVAL
                    No IRQ was ever allocated to IRQChannel.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int GetIRQCounter6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_INT IRQChannel,
    unsigned long *counter_value_p
);


/******************************************************************************
InstallDMA6430()

    Purpose:
        Configure the specified DMA circuit on a board to be able to perform
        DMA.

    Parameters:
        descriptor ======> File descriptor from OpenBoard6430() call.
        DMAChannel ======> DMA circuit to configure.  Valid values are
                           DM6430HR_DMA1 and DM6430HR_DMA2.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    DMAChannel is not valid.

                EINVAL
                    No DMA channel was ever allocated to DMAChannel.

                ENOMEM
                    DMA buffer memory allocation failed.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int InstallDMA6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_DMA DMAChannel);


/******************************************************************************
DeInstallDMA6430()

    Purpose:
        Configure the specified DMA circuit on a board so that DMA can no
        longer be performed.

    Parameters:
        descriptor ======> File descriptor from OpenBoard6430() call.
        DMAChannel ======> DMA circuit to configure.  Valid values are
                           DM6430HR_DMA1 and DM6430HR_DMA2.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    DMAChannel is not valid.

                EINVAL
                    No DMA channel was ever allocated to DMAChannel.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DeInstallDMA6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_DMA DMAChannel);


/******************************************************************************
GetDmaData6430()

    Purpose:
        Copy data from specified DMA circuit's DMA buffer to user buffer.

    Parameters:
        descriptor ==========> File descriptor from OpenBoard6430() call.
        dma_buffer_p ========> Address of user buffer.
        DMAChannel ==========> DMA circuit to operate on.  Valid values are
                               DM6430HR_DMA1 and DM6430HR_DMA2.
        length ==============> Number of bytes to transfer.
        offset ==============> Offset in bytes from beginning of DMA buffer.
        bytes_transferred_p => Address where actual number of bytes transferred
                               will be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

                EFAULT
                    dma_buffer_p is not a valid user address.

                EFAULT
                    dma_buffer_p is not large enough to hold the data.

                EINVAL
                    DMAChannel is not valid.

                EINVAL
                    No DMA channel was ever allocated to DMAChannel.

                EINVAL
                    No DMA buffer was ever allocated to DMAChannel.

                EINVAL
                    (length + offset) lies beyond the end of the DMA buffer.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int GetDmaData6430(
    struct Dm6430HR_Device_Descriptor *board,
    void *dma_buffer_p,
    enum DM6430HR_DMA DMAChannel,
    size_t length,
    size_t offset,
    size_t *bytes_transferred_p
);


/******************************************************************************
StartDMA6430()

    Purpose:
        Start DMA on a board's specified DMA circuit.

    Parameters:
        descriptor ====> File descriptor from OpenBoard6430() call.
        DMAChannel ====> DMA circuit to operate on.  Valid values are
                         DM6430HR_DMA1 and DM6430HR_DMA2.
        TransferBytes => Number of bytes to transfer in a single DMA.  This
                         value must be even.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    DMAChannel is not valid.

                EINVAL
                    No DMA channel was ever allocated to DMAChannel.

                EINVAL
                    No DMA buffer was ever allocated to DMAChannel.

                EINVAL
                    TransferBytes is odd.

                EINVAL
                    TransferBytes is greater than the DMA buffer size.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int StartDMA6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_DMA DMAChannel,
    size_t TransferBytes
);


/******************************************************************************
StopDMA6430()

    Purpose:
        Stop DMA on a board's specified DMA circuit.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        DMAChannel => DMA circuit to operate on.  Valid values are
                      DM6430HR_DMA1 and DM6430HR_DMA2.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    DMAChannel is not valid.

                EINVAL
                    No DMA channel was ever allocated to DMAChannel.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int StopDMA6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_DMA DMAChannel);


/******************************************************************************
GetAutoincData6430()

    Purpose:
        Initiate a streaming read from a board.  Once can specify the board
        register to read from, what size data is to be transferred, and how
        many data elements to transfer.

    Parameters:
        descriptor ====> File descriptor from OpenBoard6430() call.
        from_register => Register from which the data should be read.  Valid
                         values are rSTR_AD_6430 and rSTR_DIN_FIFO_6430.
        type ==========> Type/size of element to be transferred.  Valid values
                         are DM6430HR_STR_TYPE_BYTE and DM6430HR_STR_TYPE_WORD.
        buffer_p ======> Address of buffer in which to place the data read.
	element_num ===> How many data elements of type "type" should be read.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

                EFAULT
                    buffer_p is not a valid user address.

                EINVAL
                    from_register is not valid.

                EINVAL
                    type is not valid.

                EOPNOTSUPP
                    from_register is rSTR_AD_6430 and type is
                    DM6430HR_STR_TYPE_BYTE.

                EOPNOTSUPP
                    from_register is rSTR_DIN_FIFO_6430 and type is
                    DM6430HR_STR_TYPE_WORD.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int GetAutoincData6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_STR_Regs from_register,
    enum DM6430HR_STR_TYPE type,
    void *buffer_p,
    size_t element_num
);


/******************************************************************************
ReadStatus6430()

    Purpose:
        Read a board's Status Register.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        status_p ===> Address where status should be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ReadStatus6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t *status_p);


/******************************************************************************
IsADFIFOEmpty6430()

    Purpose:
        Determine whether or not a board's A/D FIFO is empty.

    Parameters:
        descriptor ======> File descriptor from OpenBoard6430() call.
        ad_fifo_empty_p => Address where FIFO empty flag should be stored.  If
                           the A/D FIFO is empty, a nonzero value will be
                           stored here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsADFIFOEmpty6430(struct Dm6430HR_Device_Descriptor *board, int *ad_fifo_empty_p);


/******************************************************************************
IsADFIFOFull6430()

    Purpose:
        Determine whether or not a board's A/D FIFO is full.

    Parameters:
        descriptor ======> File descriptor from OpenBoard6430() call.
        ad_fifo_empty_p => Address where FIFO full flag should be stored.  If
                           the A/D FIFO is full, a nonzero value will be
                           stored here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsADFIFOFull6430(struct Dm6430HR_Device_Descriptor *board, int *ad_fifo_full_p);


/******************************************************************************
IsADHalted6430()

    Purpose:
        Determine whether or not a board's A/D conversion has been stopped
        because the sample buffer is full.

    Parameters:
        descriptor ==> File descriptor from OpenBoard6430() call.
        ad_halted_p => Address where A/D halted flag should be stored.  If
                       A/D conversion has been stopped, a nonzero value will
                       be stored here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsADHalted6430(struct Dm6430HR_Device_Descriptor *board, int *ad_halted_p);


/******************************************************************************
IsADConverting6430()

    Purpose:
        Determine whether or not a board's A/D converter is converting.

    Parameters:
        descriptor ======> File descriptor from OpenBoard6430() call.
        ad_converting_p => Address where A/D converting flag should be stored.
                           If A/D conversion is occurring, a nonzero value will
                           be stored here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsADConverting6430(struct Dm6430HR_Device_Descriptor *board, int *ad_converting_p);


/******************************************************************************
IsADDMADone6430()

    Purpose:
        Determine whether or not a board's A/D DMA transfer is complete.

    Parameters:
        descriptor ====> File descriptor from OpenBoard6430() call.
        ad_dma_done_p => Address where A/D DMA done flag should be stored.  If
                         A/D DMA is finished, a nonzero value will be stored
                         here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsADDMADone6430(struct Dm6430HR_Device_Descriptor *board, int *ad_dma_done_p);


/******************************************************************************
IsFirstADDMADone6430()

    Purpose:
        Determine whether or not a board's A/D first DMA transfer is complete
        when in dual channel DMA mode.

    Parameters:
        descriptor ==========> File descriptor from OpenBoard6430() call.
        ad_first dma_done_p => Address where A/D first DMA done flag should be
                               stored.  If A/D first DMA is finished, a zero
                               value will be stored here.  Otherwise, a non-zero
                               will be stored here.  NOTE:  This is inverted
                               from expected.  Please see the Known Limitations
                               section of the README.TXT

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsFirstADDMADone6430(struct Dm6430HR_Device_Descriptor *board, int *ad_first_dma_done_p);


/******************************************************************************
IsBurstClockOn6430()

    Purpose:
        Determine whether or not a board's A/D burst clock gate is on.

    Parameters:
        descriptor ==========> File descriptor from OpenBoard6430() call.
        ad_burst_clock_on_p => Address where A/D burst clock flag should be
                               stored.  If A/D burst clock gate is on, a
                               nonzero value will be stored here.  Otherwise,
                               0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsBurstClockOn6430(struct Dm6430HR_Device_Descriptor *board, int *ad_burst_clock_on_p);


/******************************************************************************
IsPacerClockOn6430()

    Purpose:
        Determine whether or not a board's A/D pacer clock gate is on.

    Parameters:
        descriptor ==========> File descriptor from OpenBoard6430() call.
        ad_pacer_clock_on_p => Address where A/D pacer clock flag should be
                               stored.  If A/D pacer clock gate is on, a
                               nonzero value will be stored here.  Otherwise,
                               0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsPacerClockOn6430(struct Dm6430HR_Device_Descriptor *board, int *ad_pacer_clock_on_p);


/******************************************************************************
IsAboutTrigger6430()

    Purpose:
        Determine whether or not a board's A/D about trigger has occurred.

    Parameters:
        descriptor =========> File descriptor from OpenBoard6430() call.
        ad_about_trigger_p => Address where A/D about trigger flag should be
                              stored.  If A/D about trigger has occurred, a
                              nonzero value will be stored here.  Otherwise,
                              0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsAboutTrigger6430(struct Dm6430HR_Device_Descriptor *board, int *ad_about_trigger_p);


/******************************************************************************
IsDigitalIRQ6430()

    Purpose:
        Determine whether or not an advanced digital mode interrupt has
        occurred on a board.

    Parameters:
        descriptor ==========> File descriptor from OpenBoard6430() call.
        digital_interrupt_p => Address where digital interrupt flag should be
                               stored.  If an advanced digital interrupt has
                               occurred, a nonzero value will be stored here.
                               Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsDigitalIRQ6430(struct Dm6430HR_Device_Descriptor *board, int *digital_interrupt_p);


/******************************************************************************
IsDINFIFOEmpty6430()

    Purpose:
        Determine whether or not a board's digital input FIFO is empty.

    Parameters:
        descriptor ===========> File descriptor from OpenBoard6430() call.
        digital_fifo_empty_p => Address where digital fifo empty flag should be
                                stored.  If the digital fifo is empty, a
                                nonzero value will be stored here.  Otherwise,
                                0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsDINFIFOEmpty6430(struct Dm6430HR_Device_Descriptor *board, int *digital_fifo_empty_p);


/******************************************************************************
IsDINFIFOHalf6430()

    Purpose:
        Determine whether or not a board's digital input FIFO is half full.

    Parameters:
        descriptor ===============> File descriptor from OpenBoard6430() call.
        digital_fifo_half_full_p => Address where digital fifo half full flag
                                    should be stored.  If the digital fifo is
                                    half full, a nonzero value will be stored
                                    here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsDINFIFOHalf6430(struct Dm6430HR_Device_Descriptor *board, int *digital_fifo_half_full_p);


/******************************************************************************
IsDINFIFOFull6430()

    Purpose:
        Determine whether or not a board's digital input FIFO is full.

    Parameters:
        descriptor ==========> File descriptor from OpenBoard6430() call.
        digital_fifo_full_p => Address where digital fifo full flag should be
                               stored.  If the digital fifo is full, a nonzero
                               value will be stored here.  Otherwise, 0 will
                               be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.
 ******************************************************************************/

int IsDINFIFOFull6430(struct Dm6430HR_Device_Descriptor *board, int *digital_fifo_full_p);


/******************************************************************************
LoadControlRegister6430()

    Purpose:
        Load a value into a board's Control Register.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        value ======> Data to write into Control Register.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadControlRegister6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t value);


/******************************************************************************
EnableTables6430()

    Purpose:
        Enable or disable the A/D and digital tables on a board.

    Parameters:
        descriptor ===========> File descriptor from OpenBoard6430() call.
        Enable_AD_Table ======> Flag to indicate whether the A/D table should
                                be enabled.  A value of 0 means disable the A/D
                                table.  A nonzero value means enable the A/D
                                table.
        Enable_Digital_Table => Flag to indicate whether the digital table
                                should be enabled.  A value of 0 means disable
                                the digital table.  A nonzero value means
                                enable the digital table.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EOPNOTSUPP
                    The digital table is to be enabled but the A/D table is to
                    be disabled.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int EnableTables6430(
    struct Dm6430HR_Device_Descriptor *board,
    int Enable_AD_Table,
    int Enable_Digital_Table
);


/******************************************************************************
ChannelGainDataStore6430()

    Purpose:
        Enable or disable a board's channel/gain data store.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Enable =====> Flag to indicate whether the channel/gain data store
                      should be enabled.  A value of 0 means disable the data
                      store.  A nonzero value means enable the data store.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ChannelGainDataStore6430(struct Dm6430HR_Device_Descriptor *board, int Enable);


/******************************************************************************
SelectTimerCounter6430()

    Purpose:
        Select which timer/counter on a board will be accessed when a
        subsequent operation is performed on the registers located at base I/O
        address + 16 through base I/O address + 22.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Select =====> Indicate which timer/counter will be accessed.  Valid
                      values are DM6430HR_CLOCK_TC and DM6430HR_USER_TC.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Select is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SelectTimerCounter6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_CLK_SEL Select);


/******************************************************************************
SetSampleCounterStop6430()

    Purpose:
        Enable or disable a board's A/D Sample Counter Stop.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Disable ====> Flag to indicate whether the A/D sample counter should
                      stop the pacer clock.  A value of 0 means enable sample
                      counter stop.  A nonzero value means disable sample
                      counter stop.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetSampleCounterStop6430(struct Dm6430HR_Device_Descriptor *board, int Disable);


/******************************************************************************
SetPauseEnable6430()

    Purpose:
        Enable or disable a board's A/D table pause bit.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Enable =====> Flag to indicate whether the A/D table pause bit should
                      set.  A value of 0 means enable the pause bit.  A
                      nonzero value means disable the pause bit.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetPauseEnable6430(struct Dm6430HR_Device_Descriptor *board, int Enable);


/******************************************************************************
ReadADData6430()

    Purpose:
        Read 16 bits from a board's analog to digital FIFO.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        ad_data_p ==> Address where data read should be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ReadADData6430(struct Dm6430HR_Device_Descriptor *board, int16_t *ad_data_p);


/******************************************************************************
ReadChannelGainDataStore6430()

    Purpose:
        Read a board's channel gain data word.  This function assumes the
        caller knows whether or not the channel gain data store has been
        enabled since there is no way to query the board for this status.

    Parameters:
        descriptor ===> File descriptor from OpenBoard6430() call.
        cgds_data_p ==> Address where data read should be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ReadChannelGainDataStore6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t *cgds_data_p);


/******************************************************************************
SetChannelGain6430()

    Purpose:
        Load a board's channel/gain latch.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Channel ====> A/D channel.  Valid values are DM6430HR_AIN1,
                      DM6430HR_AIN2, DM6430HR_AIN3, DM6430HR_AIN4,
                      DM6430HR_AIN5, DM6430HR_AIN6, DM6430HR_AIN7,
                      DM6430HR_AIN8, DM6430HR_AIN9, DM6430HR_AIN10,
                      DM6430HR_AIN11, DM6430HR_AIN12, DM6430HR_AIN13,
                      DM6430HR_AIN14, DM6430HR_AIN15, and DM6430HR_AIN16.
        Gain =======> A/D gain.  Valid values are DM6430HR_GAINx1,
                      DM6430HR_GAINx2, DM6430HR_GAINx4, and
                      DM6430HR_GAINx8.
        Se_Diff ====> Select single-ended or differential mode.  Valid values
                      are DM6430HR_SE_SE and DM6430HR_SE_DIFF.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Channel is not valid.

                EINVAL
                    Gain is not valid.

                EINVAL
                    Se_Diff is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetChannelGain6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_AIN Channel,
    enum DM6430HR_GAIN Gain,
    enum DM6430HR_SE Se_Diff
);


/******************************************************************************
LoadADTable6430()

    Purpose:
        Load a board's A/D table with the given number of entries.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        ADEntries ==> Number of entries in A/D table.
	ADTable_p ==> Address of memory containing A/D table to send to board.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    ADEntries is 0.

                EINVAL
                    ADEntries is greater than 1024.

	    Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadADTable6430(
    struct Dm6430HR_Device_Descriptor *board,
    u_int16_t ADEntries,
    ADTableRow *ADTable_p
);


/******************************************************************************
LoadDigitalTable6430()

    Purpose:
        Load a board's digital table with the given number of entries.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        entries ====> Number of entries in digital table.
	table_p ====> Address of memory containing digital table to send to
                      board.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    entries is 0.

                EINVAL
                    entries is greater than 1024.

	    Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadDigitalTable6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t entries, u_int8_t *table_p);


/******************************************************************************
StartConversion6430()

    Purpose:
        Issue a Start Convert (software trigger) command to a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int StartConversion6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
LoadTriggerRegister6430()

    Purpose:
        Load a 16 bit value into a board's Trigger Mode Register.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	value ======> Value to write into Trigger Mode Register.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadTriggerRegister6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t value);


/******************************************************************************
SetConversionSelect6430()

    Purpose:
        Configure how a board's A/D conversion is done.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	Select =====> Indicates how A/D conversions are controlled.  Valid
                      values are DM6430HR_CONV_SOFT_TRIGGER,
                      DM6430HR_CONV_PACER_CLOCK,
                      DM6430HR_CONV_BURST_CLOCK, and
                      DM6430HR_CONV_DIGITAL_INT.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Select is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetConversionSelect6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_CONV Select);


/******************************************************************************
SetStartTrigger6430()

    Purpose:
        Configure how a board's pacer clock is started during A/D conversion.

    Parameters:
        descriptor ====> File descriptor from OpenBoard6430() call.
        Start_Trigger => What starts the pacer clock.  Valid values are
                         DM6430HR_START_TRIG_SOFTWARE,
                         DM6430HR_START_TRIG_EXTERNAL,
                         DM6430HR_START_TRIG_DIGITAL_INT,
                         DM6430HR_START_TRIG_USER_TC1, and
                         DM6430HR_START_TRIG_GATE.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Start_Trigger is not valid.

                EOPNOTSUPP
                    Start_Trigger is one of the three reserved bit patterns
                    as defined in the hardware manual.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetStartTrigger6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_START_TRIG Start_Trigger);


/******************************************************************************
SetStopTrigger6430()

    Purpose:
        Configure how a board's pacer clock is stopped during A/D conversion.

    Parameters:
        descriptor ===> File descriptor from OpenBoard6430() call.
        Stop_Trigger => What stops the pacer clock.  Valid values are
                        DM6430HR_STOP_TRIG_SOFTWARE,
                        DM6430HR_STOP_TRIG_EXTERNAL,
                        DM6430HR_STOP_TRIG_DIGITAL_INT,
                        DM6430HR_STOP_TRIG_SAMPLE_CNT,
                        DM6430HR_STOP_TRIG_ABOUT_SOFTWARE,
                        DM6430HR_STOP_TRIG_ABOUT_EXTERNAL,
                        DM6430HR_STOP_TRIG_ABOUT_DIGITAL, and
                        DM6430HR_STOP_TRIG_ABOUT_USER_TC1.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Stop_Trigger is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetStopTrigger6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_STOP_TRIG Stop_Trigger);


/******************************************************************************
SetPacerClockSource6430()

    Purpose:
        Select the source of a board's pacer clock.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Source =====> Pacer clock source.  Valid values are
                      DM6430HR_PACER_CLK_INTERNAL and
                      DM6430HR_PACER_CLK_EXTERNAL.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Source is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetPacerClockSource6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_PACER_CLK Source);


/******************************************************************************
SetBurstTrigger6430()

    Purpose:
        Select a board's burst mode trigger.

    Parameters:
        descriptor ====> File descriptor from OpenBoard6430() call.
        Burst_Trigger => What triggers a burst.  Valid values are
                         DM6430HR_BURST_TRIG_SOFTWARE,
                         DM6430HR_BURST_TRIG_PACER,
                         DM6430HR_BURST_TRIG_EXTERNAL, and
                         DM6430HR_BURST_TRIG_DIGITAL.


    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Burst_Trigger is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetBurstTrigger6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_BURST_TRIG Burst_Trigger);


/******************************************************************************
SetTriggerPolarity6430()

    Purpose:
        Select which edge of an external pacer clock triggers a board's burst
        mode.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Polarity ===> Which external pacer clock edge triggers burst mode.
                      Valid values are DM6430HR_POLAR_POSITIVE and
                      DM6430HR_POLAR_NEGATIVE.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Polarity is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetTriggerPolarity6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_POLAR Polarity);


/******************************************************************************
SetTriggerRepeat6430()

    Purpose:
        Select whether or not a trigger initiates multiple A/D conversion
        cycles.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Repeat =====> Indicates whether or not A/D conversion cycles repeat.
                      Valid values are DM6430HR_REPEAT_SINGLE and
                      DM6430HR_REPEAT_REPEAT.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Repeat is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetTriggerRepeat6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_REPEAT Repeat);


/******************************************************************************
LoadIRQRegister6430()

    Purpose:
        Load a 16 bit value into a board's Interrupt Register.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	value ======> Value to load into Interrupt Register.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadIRQRegister6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t value);


/******************************************************************************
SetIRQ0Source6430()

    Purpose:
        Select the interrupt source of the first interrupt circuit on a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	IRQSource ==> Source of interrupt on first circuit.  Valid values are
                      IRQS_AD_SAMPLE_CNT_6430,
                      IRQS_AD_START_CONVERT_6430,
                      IRQS_AD_END_CONVERT_6430,
                      IRQS_AD_WRITE_FIFO_6430,
                      IRQS_AD_FIFO_HALF_6430,
                      IRQS_AD_DMA_DONE_6430,
                      IRQS_RESET_GAIN_TABLE_6430,
                      IRQS_PAUSE_GAIN_TABLE_6430,
                      IRQS_EXT_PACER_CLOCK_6430,
                      IRQS_EXT_TRIGGER_6430,
                      IRQS_DIGITAL_6430,
                      IRQS_TC_COUNTER0_6430,
                      IRQS_TC_COUNTER0_INVERTED_6430,
                      IRQS_TC_COUNTER1_6430,
                      IRQS_DIO_FIFO_HALF_6430, and
                      IRQS_DIO_WRITE_FIFO_6430.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    IRQSource is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetIRQ0Source6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_INTSRC IRQSource);


/******************************************************************************
SetIRQ1Source6430()

    Purpose:
        Select the interrupt source of the second interrupt circuit on a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	IRQSource ==> Source of interrupt on second circuit.  Valid values are
                      IRQS_AD_SAMPLE_CNT_6430,
                      IRQS_AD_START_CONVERT_6430,
                      IRQS_AD_END_CONVERT_6430,
                      IRQS_AD_WRITE_FIFO_6430,
                      IRQS_AD_FIFO_HALF_6430,
                      IRQS_AD_DMA_DONE_6430,
                      IRQS_RESET_GAIN_TABLE_6430,
                      IRQS_PAUSE_GAIN_TABLE_6430,
                      IRQS_EXT_PACER_CLOCK_6430,
                      IRQS_EXT_TRIGGER_6430,
                      IRQS_DIGITAL_6430,
                      IRQS_TC_COUNTER0_6430,
                      IRQS_TC_COUNTER0_INVERTED_6430,
                      IRQS_TC_COUNTER1_6430,
                      IRQS_DIO_FIFO_HALF_6430, and
                      IRQS_DIO_WRITE_FIFO_6430.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    IRQSource is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetIRQ1Source6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_INTSRC IRQSource);


/******************************************************************************
ReadDINFIFO6430()

    Purpose:
        Read 8 bits of data from the digital input FIFO.

    Parameters:
        descriptor =====> File descriptor from OpenBoard6430() call.
        digital_data_p => Address where data read should be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ReadDINFIFO6430(struct Dm6430HR_Device_Descriptor *board, u_int8_t *digital_data_p);


/******************************************************************************
LoadDINConfigRegister6430()

    Purpose:
        Load a 16 bit value into a board's Digital Input FIFO Configuration
        Register.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        value ======> Value to write into register.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadDINConfigRegister6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t value);


/******************************************************************************
ConfigDINClock6430()

    Purpose:
        Set the source for a board's digital input FIFO clock.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        DIN_Clock ==> Source of digital input FIFO clock.  Valid values are
                      DM6430HR_DI_FIFO_CLK_USER_TC0,
                      DM6430HR_DI_FIFO_CLK_USER_TC1,
                      DM6430HR_DI_FIFO_CLK_AD_WRITE_FIFO,
                      DM6430HR_DI_FIFO_CLK_EXTERNAL_PACER, and
                      DM6430HR_DI_FIFO_CLK_EXTERNAL_TRIG.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    DIN_Clock is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ConfigDINClock6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_DI_FIFO_CLK DIN_Clock);


/******************************************************************************
DINClockEnable6430()

    Purpose:
        Enable or disable a board's digital input FIFO clock.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Enable =====> Indicates whether or not to enable the digital input FIFO
                      clock.  A value of 0 means disable the clock.  A nonzero
                      value means enable the clock.
    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DINClockEnable6430(struct Dm6430HR_Device_Descriptor *board, int Enable);


/******************************************************************************
LoadDAC6430()

    Purpose:
        Load a 16 bit two's complement value into a board's DAC1 Output
        Register.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Data =======> Value to write into register.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadDAC6430(struct Dm6430HR_Device_Descriptor *board, int16_t Data);


/******************************************************************************
LoadDAC26430()

    Purpose:
        Load a 16 bit two's complement value into a board's DAC2 Output
        Register.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Data =======> Value to write into register.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadDAC26430(struct Dm6430HR_Device_Descriptor *board, int16_t Data);


/******************************************************************************
ClockMode6430()

    Purpose:
        Set the mode of the specified counter on the 8254 chip.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Timer ======> Indicates which counter to use.  Valid values are
                      DM6430HR_CLK0, DM6430HR_CLK1, and DM6430HR_CLK2.
        Mode =======> Indicates which clock mode to set.  Valid values are
                      DM6430HR_CLK_MODE0, DM6430HR_CLK_MODE1,
                      DM6430HR_CLK_MODE2, DM6430HR_CLK_MODE3,
                      DM6430HR_CLK_MODE4, and DM6430HR_CLK_MODE5.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Timer is not valid.

                EINVAL
                    Mode is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClockMode6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_CLK Timer,
    enum DM6430HR_CLK_MODE Mode
);


/******************************************************************************
ClockDivisor6430()

    Purpose:
        Set the divisor for the specified counter on the 8254 chip.  Before
        calling this function, the counter must have been set to receive the
        divisor least significant byte first then most significant byte.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Timer ======> Indicates which counter to use.  Valid values are
                      DM6430HR_CLK0, DM6430HR_CLK1, and DM6430HR_CLK2.
        Divisor ====> Counter divisor value.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Timer is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ClockDivisor6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_CLK Timer,
    u_int16_t Divisor
);


/******************************************************************************
SetPacerClock6430()

    Purpose:
        Set a board's pacer clock rate.  This function decides whether to use
        a 16 bit or 32 bit clock depending upon the clock rate.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        clock ======> Clock rate desired.
        actual_p ===> Address where the actual programmed frequency should be
                      stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    clock is greater than 8 MHz.

                EINVAL
                    clock is 0.0.

                EINVAL
                    clock is less than 0.0.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetPacerClock6430(struct Dm6430HR_Device_Descriptor *board, double clock, double *actual_p);


/******************************************************************************
SetBurstClock6430()

    Purpose:
        Set a board's burst clock rate.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        BurstRate ==> Burst clock rate desired.
        actual_p ===> Address where the actual programmed frequency should be
                      stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetBurstClock6430(struct Dm6430HR_Device_Descriptor *board, double BurstRate, double *actual_p);


/******************************************************************************
SetUserClock6430()

    Purpose:
        Set up a board's user clock.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
	Timer ======> Indicates which counter to use.  Valid values are
                      DM6430HR_CLK0, DM6430HR_CLK1, and DM6430HR_CLK2.
	InputRate ==> Input frequency to specified counter.
	OutputRate => Desired output rate from specified counter.
        actual_p ===> Address where the actual programmed frequency should be
                      stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Timer is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int SetUserClock6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_CLK Timer,
    double InputRate,
    double OutputRate,
    double *actual_p
);


/******************************************************************************
ReadTimerCounter6430()

    Purpose:
        Read the 16 bit contents of the desired timer/counter.  The read is
	done as two 8 bit reads: least significant byte then most significant
	byte.

    Parameters:
        descriptor ======> File descriptor from OpenBoard6430() call.
        Timer ===========> Indicates which timer/counter to use.  Valid values
                           are DM6430HR_CLOCK_TC and DM6430HR_USER_TC.
        Clock ===========> Indicates which counter to use.  Valid values are
                           DM6430HR_CLK0, DM6430HR_CLK1, and DM6430HR_CLK2.
        counter_value_p => Address where timer contents should be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Timer is not valid.

                EINVAL
                    Clock is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int ReadTimerCounter6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_CLK_SEL Timer,
    enum DM6430HR_CLK Clock,
    u_int16_t *counter_value_p
);


/******************************************************************************
DoneTimer6430()

    Purpose:
        Initialize a board's user timer/counter counter 0 and counter 1 for
	high speed to ensure immediate load.  This function puts these two
	counters into mode 2.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure.  Please see the ioctl(2) man page for information on
            possible values errno may have in this case.
 ******************************************************************************/

int DoneTimer6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
LoadADSampleCounter6430()

    Purpose:
        Load a board's analog to digital sample counter.

    Parameters:
        descriptor ===> File descriptor from OpenBoard6430() call.
        NumOfSamples => Number of samples to take.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int LoadADSampleCounter6430(struct Dm6430HR_Device_Descriptor *board, u_int16_t NumOfSamples);


/******************************************************************************
DIOSelectRegister6430()

    Purpose:
        Set the mode of a board's digital I/O port 0 Direction/Mask/Compare
        Register located at base I/O address + 28.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Select =====> Which mode to configure.  Valid values are
                      DM6430HR_REG_CLEAR, DM6430HR_REG_DIR,
                      DM6430HR_REG_MASK, and DM6430HR_REG_CMP.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Select is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOSelectRegister6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_REG_SEL Select);


/******************************************************************************
DIOClearChip6430()

    Purpose:
        Clear a board's digital I/O chip.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOClearChip6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
DIOClearIrq6430()

    Purpose:
        Clear a board's digital I/O IRQ status flag.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOClearIrq6430(struct Dm6430HR_Device_Descriptor *board);


/******************************************************************************
DIOEnableIrq6430()

    Purpose:
        Enable or disable a board's digital interrupts.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Enable =====> Flag indicating how digital interrupts should be set.  A
                      value of 0 means disable digital interrupts.  A nonzero
                      value means enable digital interrupts.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOEnableIrq6430(struct Dm6430HR_Device_Descriptor *board, int Enable);


/******************************************************************************
DIOSetPort1Direction6430()

    Purpose:
        Set the digital I/O port 1 bit direction (input or output) on a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Direction ==> Flag indicating bit direction. A value of 0 means set
                      port 1 bits to input.  A nonzero value means set port 1
		      bits to output.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOSetPort1Direction6430(struct Dm6430HR_Device_Descriptor *board, int Direction);


/******************************************************************************
DIOSetPort0Direction6430()

    Purpose:
        Set the digital I/O port 0 bit direction (input or output) on a board.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Direction ==> Bit mask indicating bit direction for port 0 bits.  A 1
                      in the mask means set that bit to output.  A 0 in the
                      mask means set that bit to input.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOSetPort0Direction6430(struct Dm6430HR_Device_Descriptor *board, u_int8_t Direction);


/******************************************************************************
DIOLoadMask6430()

    Purpose:
        Load the Mask Register for a board's digital I/O port 0.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Mask =======> The bit mask to be loaded.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOLoadMask6430(struct Dm6430HR_Device_Descriptor *board, u_int8_t Mask);


/******************************************************************************
DIOLoadCompare6430()

    Purpose:
        Load the Compare Register for a board's digital I/O port 0.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Compare ====> The bit pattern to be matched.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOLoadCompare6430(struct Dm6430HR_Device_Descriptor *board, u_int8_t Compare);


/******************************************************************************
DIORead6430()

    Purpose:
        Read 8 bits of data from board's specified digital I/O port.

    Parameters:
        descriptor =====> File descriptor from OpenBoard6430() call.
        Port ===========> Indicates which digital I/O port to read.  Valid
                          values are DM6430HR_DIO0 and DM6430HR_DIO1.
        digital_data_p => Address where data read should be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

                EINVAL
                    Port is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIORead6430(
    struct Dm6430HR_Device_Descriptor *board,
    enum DM6430HR_DIO Port,
    u_int8_t *digital_data_p
);


/******************************************************************************
DIOReadCompareRegister6430()

    Purpose:
        Read the contents of a board's digital I/O Compare Register.

    Parameters:
        descriptor =======> File descriptor from OpenBoard6430() call.
        register_value_p => Address where register contents should be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOReadCompareRegister6430(struct Dm6430HR_Device_Descriptor *board, u_int8_t *register_value_p);


/******************************************************************************
DIOSelectClock6430()

    Purpose:
        Select the sample clock source for a board's digital I/O chip.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Clock ======> Indicates the clock source.  Valid values are
                      DM6430HR_CLOCK_TC and DM6430HR_USER_TC.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Clock is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOSelectClock6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_CLK_SEL Clock);


/******************************************************************************
DIOSelectIrqMode6430()

    Purpose:
        Set the advanced digital interrupt mode for a board's digital I/O chip
        port 0

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        IrqMode ====> Indicates the digital interrupt mode.  Valid values are
                      DM6430HR_DIO_IRQ_EVENT and DM6430HR_DIO_IRQ_MATCH .

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    IrqMode is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOSelectIrqMode6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_DIO_IRQ IrqMode);


/******************************************************************************
DIOWrite6430()

    Purpose:
        Write data to a board's selected digital I/O port.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        Port =======> The port to write to.  Valid values are DM6430HR_DIO0
                      and DM6430HR_DIO1.
        Data =======> The data to write.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for write
                    access.

                EINVAL
                    Port is not valid.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOWrite6430(struct Dm6430HR_Device_Descriptor *board, enum DM6430HR_DIO Port, u_int8_t Data);


/******************************************************************************
DIOReadStatus6430()

    Purpose:
        Read the contents of a board's Digital IRQ/Strobe Status register.

    Parameters:
        descriptor => File descriptor from OpenBoard6430() call.
        status_p ===> Address where digital I/O status register contents should
                      be stored.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOReadStatus6430(struct Dm6430HR_Device_Descriptor *board, u_int8_t *status_p);


/******************************************************************************
DIOIsChipIrq6430()

    Purpose:
        Determine whether or not a board has generated a digital I/O interrupt.

    Parameters:
        descriptor ============> File descriptor from OpenBoard6430() call.
        interrupt_generated_p => Address where interrupt generated flag should
                                 be stored.  If the board has generated a
                                 digital I/O interrupt, a nonzero value will be
                                 stored here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOIsChipIrq6430(struct Dm6430HR_Device_Descriptor *board, int *interrupt_generated_p);


/******************************************************************************
DIOIsChipStrobe6430()

    Purpose:
        Determine whether or not data has been strobed into port 0.

    Parameters:
        descriptor ========> File descriptor from OpenBoard6430() call.
        strobe_occurred_p => Address where strobe event flag should be stored.
                             If data was strobed into port 0, a nonzero value
                             will be stored here.  Otherwise, 0 will be stored
                             here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOIsChipStrobe6430(struct Dm6430HR_Device_Descriptor *board, int *strobe_occurred_p);


/******************************************************************************
DIOIsChipSystemClock6430()

    Purpose:
        Determine whether or not the 8 MHz system clock is driving digital
        I/O sampling.

    Parameters:
        descriptor =====> File descriptor from OpenBoard6430() call.
        system_clock_p => Address where system clock flag should be stored.
                          If the 8 MHz system clock is driving digital
                          sampling, a nonzero value will be stored here.
                          Otherwise, 0 will be stored here (meaning that the
                          User Timer/Counter is driving sampling).

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOIsChipSystemClock6430(struct Dm6430HR_Device_Descriptor *board, int *system_clock_p);


/******************************************************************************
DIOIsChipIRQEnabled6430()

    Purpose:
        Determine whether or not digital interrupts are enabled for a board.

    Parameters:
        descriptor ====> File descriptor from OpenBoard6430() call.
        irq_enabled_p => Address where IRQ enabled flag should be stored.  If
                         digital interrupts are enabled, a nonzero value will
                         be stored here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOIsChipIRQEnabled6430(struct Dm6430HR_Device_Descriptor *board, int *irq_enabled_p);


/******************************************************************************
DIOIsChipIRQEventMode6430()

    Purpose:
        Determine whether or not a board's digital interrupt mode is set to
        event.

    Parameters:
        descriptor ==> File descriptor from OpenBoard6430() call.
        irq_event_p => Address where IRQ event mode flag should be stored.  If
                       digital interrupts are in event mode, a nonzero value
                       will be stored here.  Otherwise, 0 will be stored here
                       (meaning digital interrupts are in match mode).

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOIsChipIRQEventMode6430(struct Dm6430HR_Device_Descriptor *board, int *irq_event_p);


/******************************************************************************
DIOIsChipPort1Output6430()

    Purpose:
        Determine whether or not a board's digital port 1 is set to output.

    Parameters:
        descriptor =====> File descriptor from OpenBoard6430() call.
        port1_output_p => Address where port 1 output flag should be stored.
                          If port 1 is set to output, a nonzero value will be
                          stored here.  Otherwise, 0 will be stored here.

    Return Value:
        0
            Success.

        -1
            Failure with errno set as follows:
                EACCES
                    descriptor refers to a file that is open but not for read
                    access.

            Please see the ioctl(2) man page for information on other possible
            values errno may have in this case.
 ******************************************************************************/

int DIOIsChipPort1Output6430(struct Dm6430HR_Device_Descriptor *board, int *port1_output_p);


int
InstallISR6430(struct Dm6430HR_Device_Descriptor * board,
		  void (*isr_fnct) (unsigned int status),
		  int policy, int priority);
int RemoveISR6430(struct Dm6430HR_Device_Descriptor * board);
void *WaitForInterrupt6430(void *ptr);
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __dm6430lib__h */
