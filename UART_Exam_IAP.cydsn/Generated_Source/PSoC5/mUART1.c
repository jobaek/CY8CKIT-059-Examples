/*******************************************************************************
* File Name: mUART1.c
* Version 2.50
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "mUART1.h"
#if (mUART1_INTERNAL_CLOCK_USED)
    #include "mUART1_IntClock.h"
#endif /* End mUART1_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 mUART1_initVar = 0u;

#if (mUART1_TX_INTERRUPT_ENABLED && mUART1_TX_ENABLED)
    volatile uint8 mUART1_txBuffer[mUART1_TX_BUFFER_SIZE];
    volatile uint8 mUART1_txBufferRead = 0u;
    uint8 mUART1_txBufferWrite = 0u;
#endif /* (mUART1_TX_INTERRUPT_ENABLED && mUART1_TX_ENABLED) */

#if (mUART1_RX_INTERRUPT_ENABLED && (mUART1_RX_ENABLED || mUART1_HD_ENABLED))
    uint8 mUART1_errorStatus = 0u;
    volatile uint8 mUART1_rxBuffer[mUART1_RX_BUFFER_SIZE];
    volatile uint8 mUART1_rxBufferRead  = 0u;
    volatile uint8 mUART1_rxBufferWrite = 0u;
    volatile uint8 mUART1_rxBufferLoopDetect = 0u;
    volatile uint8 mUART1_rxBufferOverflow   = 0u;
    #if (mUART1_RXHW_ADDRESS_ENABLED)
        volatile uint8 mUART1_rxAddressMode = mUART1_RX_ADDRESS_MODE;
        volatile uint8 mUART1_rxAddressDetected = 0u;
    #endif /* (mUART1_RXHW_ADDRESS_ENABLED) */
#endif /* (mUART1_RX_INTERRUPT_ENABLED && (mUART1_RX_ENABLED || mUART1_HD_ENABLED)) */


/*******************************************************************************
* Function Name: mUART1_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  mUART1_Start() sets the initVar variable, calls the
*  mUART1_Init() function, and then calls the
*  mUART1_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The mUART1_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time mUART1_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the mUART1_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void mUART1_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(mUART1_initVar == 0u)
    {
        mUART1_Init();
        mUART1_initVar = 1u;
    }

    mUART1_Enable();
}


/*******************************************************************************
* Function Name: mUART1_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call mUART1_Init() because
*  the mUART1_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void mUART1_Init(void) 
{
    #if(mUART1_RX_ENABLED || mUART1_HD_ENABLED)

        #if (mUART1_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(mUART1_RX_VECT_NUM, &mUART1_RXISR);
            CyIntSetPriority(mUART1_RX_VECT_NUM, mUART1_RX_PRIOR_NUM);
            mUART1_errorStatus = 0u;
        #endif /* (mUART1_RX_INTERRUPT_ENABLED) */

        #if (mUART1_RXHW_ADDRESS_ENABLED)
            mUART1_SetRxAddressMode(mUART1_RX_ADDRESS_MODE);
            mUART1_SetRxAddress1(mUART1_RX_HW_ADDRESS1);
            mUART1_SetRxAddress2(mUART1_RX_HW_ADDRESS2);
        #endif /* End mUART1_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        mUART1_RXBITCTR_PERIOD_REG = mUART1_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        mUART1_RXSTATUS_MASK_REG  = mUART1_INIT_RX_INTERRUPTS_MASK;
    #endif /* End mUART1_RX_ENABLED || mUART1_HD_ENABLED*/

    #if(mUART1_TX_ENABLED)
        #if (mUART1_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(mUART1_TX_VECT_NUM, &mUART1_TXISR);
            CyIntSetPriority(mUART1_TX_VECT_NUM, mUART1_TX_PRIOR_NUM);
        #endif /* (mUART1_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (mUART1_TXCLKGEN_DP)
            mUART1_TXBITCLKGEN_CTR_REG = mUART1_BIT_CENTER;
            mUART1_TXBITCLKTX_COMPLETE_REG = ((mUART1_NUMBER_OF_DATA_BITS +
                        mUART1_NUMBER_OF_START_BIT) * mUART1_OVER_SAMPLE_COUNT) - 1u;
        #else
            mUART1_TXBITCTR_PERIOD_REG = ((mUART1_NUMBER_OF_DATA_BITS +
                        mUART1_NUMBER_OF_START_BIT) * mUART1_OVER_SAMPLE_8) - 1u;
        #endif /* End mUART1_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (mUART1_TX_INTERRUPT_ENABLED)
            mUART1_TXSTATUS_MASK_REG = mUART1_TX_STS_FIFO_EMPTY;
        #else
            mUART1_TXSTATUS_MASK_REG = mUART1_INIT_TX_INTERRUPTS_MASK;
        #endif /*End mUART1_TX_INTERRUPT_ENABLED*/

    #endif /* End mUART1_TX_ENABLED */

    #if(mUART1_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        mUART1_WriteControlRegister( \
            (mUART1_ReadControlRegister() & (uint8)~mUART1_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(mUART1_PARITY_TYPE << mUART1_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End mUART1_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: mUART1_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call mUART1_Enable() because the mUART1_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  mUART1_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void mUART1_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (mUART1_RX_ENABLED || mUART1_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        mUART1_RXBITCTR_CONTROL_REG |= mUART1_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        mUART1_RXSTATUS_ACTL_REG  |= mUART1_INT_ENABLE;

        #if (mUART1_RX_INTERRUPT_ENABLED)
            mUART1_EnableRxInt();

            #if (mUART1_RXHW_ADDRESS_ENABLED)
                mUART1_rxAddressDetected = 0u;
            #endif /* (mUART1_RXHW_ADDRESS_ENABLED) */
        #endif /* (mUART1_RX_INTERRUPT_ENABLED) */
    #endif /* (mUART1_RX_ENABLED || mUART1_HD_ENABLED) */

    #if(mUART1_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!mUART1_TXCLKGEN_DP)
            mUART1_TXBITCTR_CONTROL_REG |= mUART1_CNTR_ENABLE;
        #endif /* End mUART1_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        mUART1_TXSTATUS_ACTL_REG |= mUART1_INT_ENABLE;
        #if (mUART1_TX_INTERRUPT_ENABLED)
            mUART1_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            mUART1_EnableTxInt();
        #endif /* (mUART1_TX_INTERRUPT_ENABLED) */
     #endif /* (mUART1_TX_INTERRUPT_ENABLED) */

    #if (mUART1_INTERNAL_CLOCK_USED)
        mUART1_IntClock_Start();  /* Enable the clock */
    #endif /* (mUART1_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: mUART1_Stop
********************************************************************************
*
* Summary:
*  Disables the UART operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void mUART1_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (mUART1_RX_ENABLED || mUART1_HD_ENABLED)
        mUART1_RXBITCTR_CONTROL_REG &= (uint8) ~mUART1_CNTR_ENABLE;
    #endif /* (mUART1_RX_ENABLED || mUART1_HD_ENABLED) */

    #if (mUART1_TX_ENABLED)
        #if(!mUART1_TXCLKGEN_DP)
            mUART1_TXBITCTR_CONTROL_REG &= (uint8) ~mUART1_CNTR_ENABLE;
        #endif /* (!mUART1_TXCLKGEN_DP) */
    #endif /* (mUART1_TX_ENABLED) */

    #if (mUART1_INTERNAL_CLOCK_USED)
        mUART1_IntClock_Stop();   /* Disable the clock */
    #endif /* (mUART1_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (mUART1_RX_ENABLED || mUART1_HD_ENABLED)
        mUART1_RXSTATUS_ACTL_REG  &= (uint8) ~mUART1_INT_ENABLE;

        #if (mUART1_RX_INTERRUPT_ENABLED)
            mUART1_DisableRxInt();
        #endif /* (mUART1_RX_INTERRUPT_ENABLED) */
    #endif /* (mUART1_RX_ENABLED || mUART1_HD_ENABLED) */

    #if (mUART1_TX_ENABLED)
        mUART1_TXSTATUS_ACTL_REG &= (uint8) ~mUART1_INT_ENABLE;

        #if (mUART1_TX_INTERRUPT_ENABLED)
            mUART1_DisableTxInt();
        #endif /* (mUART1_TX_INTERRUPT_ENABLED) */
    #endif /* (mUART1_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: mUART1_ReadControlRegister
********************************************************************************
*
* Summary:
*  Returns the current value of the control register.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the control register.
*
*******************************************************************************/
uint8 mUART1_ReadControlRegister(void) 
{
    #if (mUART1_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(mUART1_CONTROL_REG);
    #endif /* (mUART1_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: mUART1_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  mUART1_WriteControlRegister(uint8 control) 
{
    #if (mUART1_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       mUART1_CONTROL_REG = control;
    #endif /* (mUART1_CONTROL_REG_REMOVED) */
}


#if(mUART1_RX_ENABLED || mUART1_HD_ENABLED)
    /*******************************************************************************
    * Function Name: mUART1_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      mUART1_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      mUART1_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      mUART1_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      mUART1_RX_STS_BREAK            Interrupt on break.
    *      mUART1_RX_STS_OVERRUN          Interrupt on overrun error.
    *      mUART1_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      mUART1_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void mUART1_SetRxInterruptMode(uint8 intSrc) 
    {
        mUART1_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: mUART1_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns the next byte of received data. This function returns data without
    *  checking the status. You must check the status separately.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  mUART1_rxBuffer - RAM buffer pointer for save received data.
    *  mUART1_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  mUART1_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  mUART1_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 mUART1_ReadRxData(void) 
    {
        uint8 rxData;

    #if (mUART1_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        mUART1_DisableRxInt();

        locRxBufferRead  = mUART1_rxBufferRead;
        locRxBufferWrite = mUART1_rxBufferWrite;

        if( (mUART1_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = mUART1_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= mUART1_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            mUART1_rxBufferRead = locRxBufferRead;

            if(mUART1_rxBufferLoopDetect != 0u)
            {
                mUART1_rxBufferLoopDetect = 0u;
                #if ((mUART1_RX_INTERRUPT_ENABLED) && (mUART1_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( mUART1_HD_ENABLED )
                        if((mUART1_CONTROL_REG & mUART1_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            mUART1_RXSTATUS_MASK_REG  |= mUART1_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        mUART1_RXSTATUS_MASK_REG  |= mUART1_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end mUART1_HD_ENABLED */
                #endif /* ((mUART1_RX_INTERRUPT_ENABLED) && (mUART1_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = mUART1_RXDATA_REG;
        }

        mUART1_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = mUART1_RXDATA_REG;

    #endif /* (mUART1_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: mUART1_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns the current state of the receiver status register and the software
    *  buffer overflow status.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Side Effect:
    *  All status register bits are clear-on-read except
    *  mUART1_RX_STS_FIFO_NOTEMPTY.
    *  mUART1_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  mUART1_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   mUART1_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   mUART1_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 mUART1_ReadRxStatus(void) 
    {
        uint8 status;

        status = mUART1_RXSTATUS_REG & mUART1_RX_HW_MASK;

    #if (mUART1_RX_INTERRUPT_ENABLED)
        if(mUART1_rxBufferOverflow != 0u)
        {
            status |= mUART1_RX_STS_SOFT_BUFF_OVER;
            mUART1_rxBufferOverflow = 0u;
        }
    #endif /* (mUART1_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: mUART1_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. mUART1_GetChar() is
    *  designed for ASCII characters and returns a uint8 where 1 to 255 are values
    *  for valid characters and 0 indicates an error occurred or no data is present.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  mUART1_rxBuffer - RAM buffer pointer for save received data.
    *  mUART1_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  mUART1_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  mUART1_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 mUART1_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (mUART1_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        mUART1_DisableRxInt();

        locRxBufferRead  = mUART1_rxBufferRead;
        locRxBufferWrite = mUART1_rxBufferWrite;

        if( (mUART1_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = mUART1_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= mUART1_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            mUART1_rxBufferRead = locRxBufferRead;

            if(mUART1_rxBufferLoopDetect != 0u)
            {
                mUART1_rxBufferLoopDetect = 0u;
                #if( (mUART1_RX_INTERRUPT_ENABLED) && (mUART1_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( mUART1_HD_ENABLED )
                        if((mUART1_CONTROL_REG & mUART1_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            mUART1_RXSTATUS_MASK_REG |= mUART1_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        mUART1_RXSTATUS_MASK_REG |= mUART1_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end mUART1_HD_ENABLED */
                #endif /* mUART1_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = mUART1_RXSTATUS_REG;
            if((rxStatus & mUART1_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = mUART1_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (mUART1_RX_STS_BREAK | mUART1_RX_STS_PAR_ERROR |
                                mUART1_RX_STS_STOP_ERROR | mUART1_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        mUART1_EnableRxInt();

    #else

        rxStatus =mUART1_RXSTATUS_REG;
        if((rxStatus & mUART1_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = mUART1_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (mUART1_RX_STS_BREAK | mUART1_RX_STS_PAR_ERROR |
                            mUART1_RX_STS_STOP_ERROR | mUART1_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (mUART1_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: mUART1_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, returns received character and error
    *  condition.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains status and LSB contains UART RX data. If the MSB is nonzero,
    *  an error has occurred.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 mUART1_GetByte(void) 
    {
        
    #if (mUART1_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        mUART1_DisableRxInt();
        locErrorStatus = (uint16)mUART1_errorStatus;
        mUART1_errorStatus = 0u;
        mUART1_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | mUART1_ReadRxData() );
    #else
        return ( ((uint16)mUART1_ReadRxStatus() << 8u) | mUART1_ReadRxData() );
    #endif /* mUART1_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: mUART1_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received bytes available in the RX buffer.
    *  * RX software buffer is disabled (RX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty RX FIFO or 1 for not empty RX FIFO.
    *  * RX software buffer is enabled: returns the number of bytes available in 
    *    the RX software buffer. Bytes available in the RX FIFO do not take to 
    *    account.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Number of bytes in the RX buffer. 
    *    Return value type depends on RX Buffer Size parameter.
    *
    * Global Variables:
    *  mUART1_rxBufferWrite - used to calculate left bytes.
    *  mUART1_rxBufferRead - used to calculate left bytes.
    *  mUART1_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 mUART1_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (mUART1_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        mUART1_DisableRxInt();

        if(mUART1_rxBufferRead == mUART1_rxBufferWrite)
        {
            if(mUART1_rxBufferLoopDetect != 0u)
            {
                size = mUART1_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(mUART1_rxBufferRead < mUART1_rxBufferWrite)
        {
            size = (mUART1_rxBufferWrite - mUART1_rxBufferRead);
        }
        else
        {
            size = (mUART1_RX_BUFFER_SIZE - mUART1_rxBufferRead) + mUART1_rxBufferWrite;
        }

        mUART1_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((mUART1_RXSTATUS_REG & mUART1_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (mUART1_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: mUART1_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receiver memory buffer and hardware RX FIFO of all received data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_rxBufferWrite - cleared to zero.
    *  mUART1_rxBufferRead - cleared to zero.
    *  mUART1_rxBufferLoopDetect - cleared to zero.
    *  mUART1_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *
    *******************************************************************************/
    void mUART1_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        mUART1_RXDATA_AUX_CTL_REG |= (uint8)  mUART1_RX_FIFO_CLR;
        mUART1_RXDATA_AUX_CTL_REG &= (uint8) ~mUART1_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (mUART1_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        mUART1_DisableRxInt();

        mUART1_rxBufferRead = 0u;
        mUART1_rxBufferWrite = 0u;
        mUART1_rxBufferLoopDetect = 0u;
        mUART1_rxBufferOverflow = 0u;

        mUART1_EnableRxInt();

    #endif /* (mUART1_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: mUART1_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  mUART1__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  mUART1__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  mUART1__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  mUART1__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  mUART1__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  mUART1_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void mUART1_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(mUART1_RXHW_ADDRESS_ENABLED)
            #if(mUART1_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* mUART1_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = mUART1_CONTROL_REG & (uint8)~mUART1_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << mUART1_CTRL_RXADDR_MODE0_SHIFT);
                mUART1_CONTROL_REG = tmpCtrl;

                #if(mUART1_RX_INTERRUPT_ENABLED && \
                   (mUART1_RXBUFFERSIZE > mUART1_FIFO_LENGTH) )
                    mUART1_rxAddressMode = addressMode;
                    mUART1_rxAddressDetected = 0u;
                #endif /* End mUART1_RXBUFFERSIZE > mUART1_FIFO_LENGTH*/
            #endif /* End mUART1_CONTROL_REG_REMOVED */
        #else /* mUART1_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End mUART1_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: mUART1_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Sets the first of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #1 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void mUART1_SetRxAddress1(uint8 address) 
    {
        mUART1_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: mUART1_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Sets the second of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #2 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void mUART1_SetRxAddress2(uint8 address) 
    {
        mUART1_RXADDRESS2_REG = address;
    }

#endif  /* mUART1_RX_ENABLED || mUART1_HD_ENABLED*/


#if( (mUART1_TX_ENABLED) || (mUART1_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: mUART1_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   mUART1_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   mUART1_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   mUART1_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   mUART1_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void mUART1_SetTxInterruptMode(uint8 intSrc) 
    {
        mUART1_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: mUART1_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data into the transmit buffer to be sent when the bus is
    *  available without checking the TX status register. You must check status
    *  separately.
    *
    * Parameters:
    *  txDataByte: data byte
    *
    * Return:
    * None.
    *
    * Global Variables:
    *  mUART1_txBuffer - RAM buffer pointer for save data for transmission
    *  mUART1_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  mUART1_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  mUART1_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void mUART1_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(mUART1_initVar != 0u)
        {
        #if (mUART1_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            mUART1_DisableTxInt();

            if( (mUART1_txBufferRead == mUART1_txBufferWrite) &&
                ((mUART1_TXSTATUS_REG & mUART1_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                mUART1_TXDATA_REG = txDataByte;
            }
            else
            {
                if(mUART1_txBufferWrite >= mUART1_TX_BUFFER_SIZE)
                {
                    mUART1_txBufferWrite = 0u;
                }

                mUART1_txBuffer[mUART1_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                mUART1_txBufferWrite++;
            }

            mUART1_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            mUART1_TXDATA_REG = txDataByte;

        #endif /*(mUART1_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: mUART1_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Reads the status register for the TX portion of the UART.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the TX status register, which is cleared on read.
    *  It is up to the user to handle all bits in this return value accordingly,
    *  even if the bit was not enabled as an interrupt source the event happened
    *  and must be handled accordingly.
    *
    *******************************************************************************/
    uint8 mUART1_ReadTxStatus(void) 
    {
        return(mUART1_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: mUART1_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Puts a byte of data into the transmit buffer to be sent when the bus is
    *  available. This is a blocking API that waits until the TX buffer has room to
    *  hold the data.
    *
    * Parameters:
    *  txDataByte: Byte containing the data to transmit
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_txBuffer - RAM buffer pointer for save data for transmission
    *  mUART1_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  mUART1_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  mUART1_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void mUART1_PutChar(uint8 txDataByte) 
    {
    #if (mUART1_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((mUART1_TX_BUFFER_SIZE > mUART1_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            mUART1_DisableTxInt();
        #endif /* (mUART1_TX_BUFFER_SIZE > mUART1_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = mUART1_txBufferWrite;
            locTxBufferRead  = mUART1_txBufferRead;

        #if ((mUART1_TX_BUFFER_SIZE > mUART1_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            mUART1_EnableTxInt();
        #endif /* (mUART1_TX_BUFFER_SIZE > mUART1_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(mUART1_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((mUART1_TXSTATUS_REG & mUART1_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            mUART1_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= mUART1_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            mUART1_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((mUART1_TX_BUFFER_SIZE > mUART1_MAX_BYTE_VALUE) && (CY_PSOC3))
            mUART1_DisableTxInt();
        #endif /* (mUART1_TX_BUFFER_SIZE > mUART1_MAX_BYTE_VALUE) && (CY_PSOC3) */

            mUART1_txBufferWrite = locTxBufferWrite;

        #if ((mUART1_TX_BUFFER_SIZE > mUART1_MAX_BYTE_VALUE) && (CY_PSOC3))
            mUART1_EnableTxInt();
        #endif /* (mUART1_TX_BUFFER_SIZE > mUART1_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (mUART1_TXSTATUS_REG & mUART1_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                mUART1_SetPendingTxInt();
            }
        }

    #else

        while((mUART1_TXSTATUS_REG & mUART1_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        mUART1_TXDATA_REG = txDataByte;

    #endif /* mUART1_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: mUART1_PutString
    ********************************************************************************
    *
    * Summary:
    *  Sends a NULL terminated string to the TX buffer for transmission.
    *
    * Parameters:
    *  string[]: Pointer to the null terminated string array residing in RAM or ROM
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void mUART1_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(mUART1_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                mUART1_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: mUART1_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Places N bytes of data from a memory array into the TX buffer for
    *  transmission.
    *
    * Parameters:
    *  string[]: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of bytes to be transmitted. The type depends on TX Buffer
    *             Size parameter.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void mUART1_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(mUART1_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                mUART1_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: mUART1_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Writes a byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) to the transmit buffer.
    *
    * Parameters:
    *  txDataByte: Data byte to transmit before the carriage return and line feed.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void mUART1_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(mUART1_initVar != 0u)
        {
            mUART1_PutChar(txDataByte);
            mUART1_PutChar(0x0Du);
            mUART1_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: mUART1_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes in the TX buffer which are waiting to be 
    *  transmitted.
    *  * TX software buffer is disabled (TX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty TX FIFO, 1 for not full TX FIFO or 4 for full TX FIFO.
    *  * TX software buffer is enabled: returns the number of bytes in the TX 
    *    software buffer which are waiting to be transmitted. Bytes available in the
    *    TX FIFO do not count.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Number of bytes used in the TX buffer. Return value type depends on the TX 
    *  Buffer Size parameter.
    *
    * Global Variables:
    *  mUART1_txBufferWrite - used to calculate left space.
    *  mUART1_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 mUART1_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (mUART1_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        mUART1_DisableTxInt();

        if(mUART1_txBufferRead == mUART1_txBufferWrite)
        {
            size = 0u;
        }
        else if(mUART1_txBufferRead < mUART1_txBufferWrite)
        {
            size = (mUART1_txBufferWrite - mUART1_txBufferRead);
        }
        else
        {
            size = (mUART1_TX_BUFFER_SIZE - mUART1_txBufferRead) +
                    mUART1_txBufferWrite;
        }

        mUART1_EnableTxInt();

    #else

        size = mUART1_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & mUART1_TX_STS_FIFO_FULL) != 0u)
        {
            size = mUART1_FIFO_LENGTH;
        }
        else if((size & mUART1_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (mUART1_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: mUART1_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears all data from the TX buffer and hardware TX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_txBufferWrite - cleared to zero.
    *  mUART1_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Data waiting in the transmit buffer is not sent; a byte that is currently
    *  transmitting finishes transmitting.
    *
    *******************************************************************************/
    void mUART1_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        mUART1_TXDATA_AUX_CTL_REG |= (uint8)  mUART1_TX_FIFO_CLR;
        mUART1_TXDATA_AUX_CTL_REG &= (uint8) ~mUART1_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (mUART1_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        mUART1_DisableTxInt();

        mUART1_txBufferRead = 0u;
        mUART1_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        mUART1_EnableTxInt();

    #endif /* (mUART1_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: mUART1_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   mUART1_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   mUART1_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   mUART1_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   mUART1_SEND_WAIT_REINIT - Performs both options: 
    *      mUART1_SEND_BREAK and mUART1_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_initVar - checked to identify that the component has been
    *     initialized.
    *  txPeriod - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  There are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Function will block CPU until transmission
    *     complete.
    *  2) User may want to use blocking time if UART configured to the low speed
    *     operation
    *     Example for this case:
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to initialize and use the interrupt to
    *     complete break operation.
    *     Example for this case:
    *     Initialize TX interrupt with "TX - On TX Complete" parameter
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     When interrupt appear with mUART1_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The mUART1_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void mUART1_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(mUART1_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(mUART1_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == mUART1_SEND_BREAK) ||
                (retMode == mUART1_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                mUART1_WriteControlRegister(mUART1_ReadControlRegister() |
                                                      mUART1_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                mUART1_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = mUART1_TXSTATUS_REG;
                }
                while((tmpStat & mUART1_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == mUART1_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == mUART1_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = mUART1_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & mUART1_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == mUART1_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == mUART1_REINIT) ||
                (retMode == mUART1_SEND_WAIT_REINIT) )
            {
                mUART1_WriteControlRegister(mUART1_ReadControlRegister() &
                                              (uint8)~mUART1_CTRL_HD_SEND_BREAK);
            }

        #else /* mUART1_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == mUART1_SEND_BREAK) ||
                (retMode == mUART1_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (mUART1_PARITY_TYPE != mUART1__B_UART__NONE_REVB) || \
                                    (mUART1_PARITY_TYPE_SW != 0u) )
                    mUART1_WriteControlRegister(mUART1_ReadControlRegister() |
                                                          mUART1_CTRL_HD_SEND_BREAK);
                #endif /* End mUART1_PARITY_TYPE != mUART1__B_UART__NONE_REVB  */

                #if(mUART1_TXCLKGEN_DP)
                    txPeriod = mUART1_TXBITCLKTX_COMPLETE_REG;
                    mUART1_TXBITCLKTX_COMPLETE_REG = mUART1_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = mUART1_TXBITCTR_PERIOD_REG;
                    mUART1_TXBITCTR_PERIOD_REG = mUART1_TXBITCTR_BREAKBITS8X;
                #endif /* End mUART1_TXCLKGEN_DP */

                /* Send zeros */
                mUART1_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = mUART1_TXSTATUS_REG;
                }
                while((tmpStat & mUART1_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == mUART1_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == mUART1_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = mUART1_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & mUART1_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == mUART1_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == mUART1_REINIT) ||
                (retMode == mUART1_SEND_WAIT_REINIT) )
            {

            #if(mUART1_TXCLKGEN_DP)
                mUART1_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                mUART1_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End mUART1_TXCLKGEN_DP */

            #if( (mUART1_PARITY_TYPE != mUART1__B_UART__NONE_REVB) || \
                 (mUART1_PARITY_TYPE_SW != 0u) )
                mUART1_WriteControlRegister(mUART1_ReadControlRegister() &
                                                      (uint8) ~mUART1_CTRL_HD_SEND_BREAK);
            #endif /* End mUART1_PARITY_TYPE != NONE */
            }
        #endif    /* End mUART1_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: mUART1_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       mUART1_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       mUART1_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears mUART1_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void mUART1_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( mUART1_CONTROL_REG_REMOVED == 0u )
            mUART1_WriteControlRegister(mUART1_ReadControlRegister() |
                                                  mUART1_CTRL_MARK);
        #endif /* End mUART1_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( mUART1_CONTROL_REG_REMOVED == 0u )
            mUART1_WriteControlRegister(mUART1_ReadControlRegister() &
                                                  (uint8) ~mUART1_CTRL_MARK);
        #endif /* End mUART1_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndmUART1_TX_ENABLED */

#if(mUART1_HD_ENABLED)


    /*******************************************************************************
    * Function Name: mUART1_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the receiver configuration in half duplex mode. After calling this
    *  function, the UART is ready to receive data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the transmitter
    *  configuration.
    *
    *******************************************************************************/
    void mUART1_LoadRxConfig(void) 
    {
        mUART1_WriteControlRegister(mUART1_ReadControlRegister() &
                                                (uint8)~mUART1_CTRL_HD_SEND);
        mUART1_RXBITCTR_PERIOD_REG = mUART1_HD_RXBITCTR_INIT;

    #if (mUART1_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        mUART1_SetRxInterruptMode(mUART1_INIT_RX_INTERRUPTS_MASK);
    #endif /* (mUART1_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: mUART1_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the transmitter configuration in half duplex mode. After calling this
    *  function, the UART is ready to transmit data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the receiver configuration.
    *
    *******************************************************************************/
    void mUART1_LoadTxConfig(void) 
    {
    #if (mUART1_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        mUART1_SetRxInterruptMode(0u);
    #endif /* (mUART1_RX_INTERRUPT_ENABLED) */

        mUART1_WriteControlRegister(mUART1_ReadControlRegister() | mUART1_CTRL_HD_SEND);
        mUART1_RXBITCTR_PERIOD_REG = mUART1_HD_TXBITCTR_INIT;
    }

#endif  /* mUART1_HD_ENABLED */


/* [] END OF FILE */
