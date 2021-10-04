/*******************************************************************************
* File Name: mUART1INT.c
* Version 2.50
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "mUART1.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (mUART1_RX_INTERRUPT_ENABLED && (mUART1_RX_ENABLED || mUART1_HD_ENABLED))
    /*******************************************************************************
    * Function Name: mUART1_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_rxBuffer - RAM buffer pointer for save received data.
    *  mUART1_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  mUART1_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  mUART1_rxBufferOverflow - software overflow flag. Set to one
    *     when mUART1_rxBufferWrite index overtakes
    *     mUART1_rxBufferRead index.
    *  mUART1_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when mUART1_rxBufferWrite is equal to
    *    mUART1_rxBufferRead
    *  mUART1_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  mUART1_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(mUART1_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef mUART1_RXISR_ENTRY_CALLBACK
        mUART1_RXISR_EntryCallback();
    #endif /* mUART1_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START mUART1_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = mUART1_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in mUART1_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (mUART1_RX_STS_BREAK | 
                            mUART1_RX_STS_PAR_ERROR |
                            mUART1_RX_STS_STOP_ERROR | 
                            mUART1_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                mUART1_errorStatus |= readStatus & ( mUART1_RX_STS_BREAK | 
                                                            mUART1_RX_STS_PAR_ERROR | 
                                                            mUART1_RX_STS_STOP_ERROR | 
                                                            mUART1_RX_STS_OVERRUN);
                /* `#START mUART1_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef mUART1_RXISR_ERROR_CALLBACK
                mUART1_RXISR_ERROR_Callback();
            #endif /* mUART1_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & mUART1_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = mUART1_RXDATA_REG;
            #if (mUART1_RXHW_ADDRESS_ENABLED)
                if(mUART1_rxAddressMode == (uint8)mUART1__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & mUART1_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & mUART1_RX_STS_ADDR_MATCH) != 0u)
                        {
                            mUART1_rxAddressDetected = 1u;
                        }
                        else
                        {
                            mUART1_rxAddressDetected = 0u;
                        }
                    }
                    if(mUART1_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        mUART1_rxBuffer[mUART1_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    mUART1_rxBuffer[mUART1_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                mUART1_rxBuffer[mUART1_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (mUART1_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(mUART1_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        mUART1_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    mUART1_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(mUART1_rxBufferWrite >= mUART1_RX_BUFFER_SIZE)
                    {
                        mUART1_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(mUART1_rxBufferWrite == mUART1_rxBufferRead)
                    {
                        mUART1_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (mUART1_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            mUART1_RXSTATUS_MASK_REG  &= (uint8)~mUART1_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(mUART1_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (mUART1_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & mUART1_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START mUART1_RXISR_END` */

        /* `#END` */

    #ifdef mUART1_RXISR_EXIT_CALLBACK
        mUART1_RXISR_ExitCallback();
    #endif /* mUART1_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (mUART1_RX_INTERRUPT_ENABLED && (mUART1_RX_ENABLED || mUART1_HD_ENABLED)) */


#if (mUART1_TX_INTERRUPT_ENABLED && mUART1_TX_ENABLED)
    /*******************************************************************************
    * Function Name: mUART1_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  mUART1_txBuffer - RAM buffer pointer for transmit data from.
    *  mUART1_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  mUART1_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(mUART1_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef mUART1_TXISR_ENTRY_CALLBACK
        mUART1_TXISR_EntryCallback();
    #endif /* mUART1_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START mUART1_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((mUART1_txBufferRead != mUART1_txBufferWrite) &&
             ((mUART1_TXSTATUS_REG & mUART1_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(mUART1_txBufferRead >= mUART1_TX_BUFFER_SIZE)
            {
                mUART1_txBufferRead = 0u;
            }

            mUART1_TXDATA_REG = mUART1_txBuffer[mUART1_txBufferRead];

            /* Set next pointer */
            mUART1_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START mUART1_TXISR_END` */

        /* `#END` */

    #ifdef mUART1_TXISR_EXIT_CALLBACK
        mUART1_TXISR_ExitCallback();
    #endif /* mUART1_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (mUART1_TX_INTERRUPT_ENABLED && mUART1_TX_ENABLED) */


/* [] END OF FILE */
