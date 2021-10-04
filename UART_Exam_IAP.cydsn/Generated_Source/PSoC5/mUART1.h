/*******************************************************************************
* File Name: mUART1.h
* Version 2.50
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_mUART1_H)
#define CY_UART_mUART1_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
* Conditional Compilation Parameters
***************************************/

#define mUART1_RX_ENABLED                     (1u)
#define mUART1_TX_ENABLED                     (1u)
#define mUART1_HD_ENABLED                     (0u)
#define mUART1_RX_INTERRUPT_ENABLED           (0u)
#define mUART1_TX_INTERRUPT_ENABLED           (0u)
#define mUART1_INTERNAL_CLOCK_USED            (1u)
#define mUART1_RXHW_ADDRESS_ENABLED           (0u)
#define mUART1_OVER_SAMPLE_COUNT              (8u)
#define mUART1_PARITY_TYPE                    (0u)
#define mUART1_PARITY_TYPE_SW                 (0u)
#define mUART1_BREAK_DETECT                   (0u)
#define mUART1_BREAK_BITS_TX                  (13u)
#define mUART1_BREAK_BITS_RX                  (13u)
#define mUART1_TXCLKGEN_DP                    (1u)
#define mUART1_USE23POLLING                   (1u)
#define mUART1_FLOW_CONTROL                   (0u)
#define mUART1_CLK_FREQ                       (0u)
#define mUART1_TX_BUFFER_SIZE                 (4u)
#define mUART1_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(mUART1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define mUART1_CONTROL_REG_REMOVED            (0u)
#else
    #define mUART1_CONTROL_REG_REMOVED            (1u)
#endif /* End mUART1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct mUART1_backupStruct_
{
    uint8 enableState;

    #if(mUART1_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End mUART1_CONTROL_REG_REMOVED */

} mUART1_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void mUART1_Start(void) ;
void mUART1_Stop(void) ;
uint8 mUART1_ReadControlRegister(void) ;
void mUART1_WriteControlRegister(uint8 control) ;

void mUART1_Init(void) ;
void mUART1_Enable(void) ;
void mUART1_SaveConfig(void) ;
void mUART1_RestoreConfig(void) ;
void mUART1_Sleep(void) ;
void mUART1_Wakeup(void) ;

/* Only if RX is enabled */
#if( (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) )

    #if (mUART1_RX_INTERRUPT_ENABLED)
        #define mUART1_EnableRxInt()  CyIntEnable (mUART1_RX_VECT_NUM)
        #define mUART1_DisableRxInt() CyIntDisable(mUART1_RX_VECT_NUM)
        CY_ISR_PROTO(mUART1_RXISR);
    #endif /* mUART1_RX_INTERRUPT_ENABLED */

    void mUART1_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void mUART1_SetRxAddress1(uint8 address) ;
    void mUART1_SetRxAddress2(uint8 address) ;

    void  mUART1_SetRxInterruptMode(uint8 intSrc) ;
    uint8 mUART1_ReadRxData(void) ;
    uint8 mUART1_ReadRxStatus(void) ;
    uint8 mUART1_GetChar(void) ;
    uint16 mUART1_GetByte(void) ;
    uint8 mUART1_GetRxBufferSize(void)
                                                            ;
    void mUART1_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define mUART1_GetRxInterruptSource   mUART1_ReadRxStatus

#endif /* End (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) */

/* Only if TX is enabled */
#if(mUART1_TX_ENABLED || mUART1_HD_ENABLED)

    #if(mUART1_TX_INTERRUPT_ENABLED)
        #define mUART1_EnableTxInt()  CyIntEnable (mUART1_TX_VECT_NUM)
        #define mUART1_DisableTxInt() CyIntDisable(mUART1_TX_VECT_NUM)
        #define mUART1_SetPendingTxInt() CyIntSetPending(mUART1_TX_VECT_NUM)
        #define mUART1_ClearPendingTxInt() CyIntClearPending(mUART1_TX_VECT_NUM)
        CY_ISR_PROTO(mUART1_TXISR);
    #endif /* mUART1_TX_INTERRUPT_ENABLED */

    void mUART1_SetTxInterruptMode(uint8 intSrc) ;
    void mUART1_WriteTxData(uint8 txDataByte) ;
    uint8 mUART1_ReadTxStatus(void) ;
    void mUART1_PutChar(uint8 txDataByte) ;
    void mUART1_PutString(const char8 string[]) ;
    void mUART1_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void mUART1_PutCRLF(uint8 txDataByte) ;
    void mUART1_ClearTxBuffer(void) ;
    void mUART1_SetTxAddressMode(uint8 addressMode) ;
    void mUART1_SendBreak(uint8 retMode) ;
    uint8 mUART1_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define mUART1_PutStringConst         mUART1_PutString
    #define mUART1_PutArrayConst          mUART1_PutArray
    #define mUART1_GetTxInterruptSource   mUART1_ReadTxStatus

#endif /* End mUART1_TX_ENABLED || mUART1_HD_ENABLED */

#if(mUART1_HD_ENABLED)
    void mUART1_LoadRxConfig(void) ;
    void mUART1_LoadTxConfig(void) ;
#endif /* End mUART1_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mUART1) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    mUART1_CyBtldrCommStart(void) CYSMALL ;
    void    mUART1_CyBtldrCommStop(void) CYSMALL ;
    void    mUART1_CyBtldrCommReset(void) CYSMALL ;
    cystatus mUART1_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus mUART1_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mUART1)
        #define CyBtldrCommStart    mUART1_CyBtldrCommStart
        #define CyBtldrCommStop     mUART1_CyBtldrCommStop
        #define CyBtldrCommReset    mUART1_CyBtldrCommReset
        #define CyBtldrCommWrite    mUART1_CyBtldrCommWrite
        #define CyBtldrCommRead     mUART1_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_mUART1) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define mUART1_BYTE2BYTE_TIME_OUT (25u)
    #define mUART1_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define mUART1_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define mUART1_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define mUART1_SET_SPACE      (0x00u)
#define mUART1_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (mUART1_TX_ENABLED) || (mUART1_HD_ENABLED) )
    #if(mUART1_TX_INTERRUPT_ENABLED)
        #define mUART1_TX_VECT_NUM            (uint8)mUART1_TXInternalInterrupt__INTC_NUMBER
        #define mUART1_TX_PRIOR_NUM           (uint8)mUART1_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* mUART1_TX_INTERRUPT_ENABLED */

    #define mUART1_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define mUART1_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define mUART1_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(mUART1_TX_ENABLED)
        #define mUART1_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (mUART1_HD_ENABLED) */
        #define mUART1_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (mUART1_TX_ENABLED) */

    #define mUART1_TX_STS_COMPLETE            (uint8)(0x01u << mUART1_TX_STS_COMPLETE_SHIFT)
    #define mUART1_TX_STS_FIFO_EMPTY          (uint8)(0x01u << mUART1_TX_STS_FIFO_EMPTY_SHIFT)
    #define mUART1_TX_STS_FIFO_FULL           (uint8)(0x01u << mUART1_TX_STS_FIFO_FULL_SHIFT)
    #define mUART1_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << mUART1_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (mUART1_TX_ENABLED) || (mUART1_HD_ENABLED)*/

#if( (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) )
    #if(mUART1_RX_INTERRUPT_ENABLED)
        #define mUART1_RX_VECT_NUM            (uint8)mUART1_RXInternalInterrupt__INTC_NUMBER
        #define mUART1_RX_PRIOR_NUM           (uint8)mUART1_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* mUART1_RX_INTERRUPT_ENABLED */
    #define mUART1_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define mUART1_RX_STS_BREAK_SHIFT             (0x01u)
    #define mUART1_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define mUART1_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define mUART1_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define mUART1_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define mUART1_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define mUART1_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define mUART1_RX_STS_MRKSPC           (uint8)(0x01u << mUART1_RX_STS_MRKSPC_SHIFT)
    #define mUART1_RX_STS_BREAK            (uint8)(0x01u << mUART1_RX_STS_BREAK_SHIFT)
    #define mUART1_RX_STS_PAR_ERROR        (uint8)(0x01u << mUART1_RX_STS_PAR_ERROR_SHIFT)
    #define mUART1_RX_STS_STOP_ERROR       (uint8)(0x01u << mUART1_RX_STS_STOP_ERROR_SHIFT)
    #define mUART1_RX_STS_OVERRUN          (uint8)(0x01u << mUART1_RX_STS_OVERRUN_SHIFT)
    #define mUART1_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << mUART1_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define mUART1_RX_STS_ADDR_MATCH       (uint8)(0x01u << mUART1_RX_STS_ADDR_MATCH_SHIFT)
    #define mUART1_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << mUART1_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define mUART1_RX_HW_MASK                     (0x7Fu)
#endif /* End (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) */

/* Control Register definitions */
#define mUART1_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define mUART1_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define mUART1_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define mUART1_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define mUART1_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define mUART1_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define mUART1_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define mUART1_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define mUART1_CTRL_HD_SEND               (uint8)(0x01u << mUART1_CTRL_HD_SEND_SHIFT)
#define mUART1_CTRL_HD_SEND_BREAK         (uint8)(0x01u << mUART1_CTRL_HD_SEND_BREAK_SHIFT)
#define mUART1_CTRL_MARK                  (uint8)(0x01u << mUART1_CTRL_MARK_SHIFT)
#define mUART1_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << mUART1_CTRL_PARITY_TYPE0_SHIFT)
#define mUART1_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << mUART1_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define mUART1_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define mUART1_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define mUART1_SEND_BREAK                         (0x00u)
#define mUART1_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define mUART1_REINIT                             (0x02u)
#define mUART1_SEND_WAIT_REINIT                   (0x03u)

#define mUART1_OVER_SAMPLE_8                      (8u)
#define mUART1_OVER_SAMPLE_16                     (16u)

#define mUART1_BIT_CENTER                         (mUART1_OVER_SAMPLE_COUNT - 2u)

#define mUART1_FIFO_LENGTH                        (4u)
#define mUART1_NUMBER_OF_START_BIT                (1u)
#define mUART1_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define mUART1_TXBITCTR_BREAKBITS8X   ((mUART1_BREAK_BITS_TX * mUART1_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define mUART1_TXBITCTR_BREAKBITS ((mUART1_BREAK_BITS_TX * mUART1_OVER_SAMPLE_COUNT) - 1u)

#define mUART1_HALF_BIT_COUNT   \
                            (((mUART1_OVER_SAMPLE_COUNT / 2u) + (mUART1_USE23POLLING * 1u)) - 2u)
#if (mUART1_OVER_SAMPLE_COUNT == mUART1_OVER_SAMPLE_8)
    #define mUART1_HD_TXBITCTR_INIT   (((mUART1_BREAK_BITS_TX + \
                            mUART1_NUMBER_OF_START_BIT) * mUART1_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define mUART1_RXBITCTR_INIT  ((((mUART1_BREAK_BITS_RX + mUART1_NUMBER_OF_START_BIT) \
                            * mUART1_OVER_SAMPLE_COUNT) + mUART1_HALF_BIT_COUNT) - 1u)

#else /* mUART1_OVER_SAMPLE_COUNT == mUART1_OVER_SAMPLE_16 */
    #define mUART1_HD_TXBITCTR_INIT   ((8u * mUART1_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define mUART1_RXBITCTR_INIT      (((7u * mUART1_OVER_SAMPLE_COUNT) - 1u) + \
                                                      mUART1_HALF_BIT_COUNT)
#endif /* End mUART1_OVER_SAMPLE_COUNT */

#define mUART1_HD_RXBITCTR_INIT                   mUART1_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 mUART1_initVar;
#if (mUART1_TX_INTERRUPT_ENABLED && mUART1_TX_ENABLED)
    extern volatile uint8 mUART1_txBuffer[mUART1_TX_BUFFER_SIZE];
    extern volatile uint8 mUART1_txBufferRead;
    extern uint8 mUART1_txBufferWrite;
#endif /* (mUART1_TX_INTERRUPT_ENABLED && mUART1_TX_ENABLED) */
#if (mUART1_RX_INTERRUPT_ENABLED && (mUART1_RX_ENABLED || mUART1_HD_ENABLED))
    extern uint8 mUART1_errorStatus;
    extern volatile uint8 mUART1_rxBuffer[mUART1_RX_BUFFER_SIZE];
    extern volatile uint8 mUART1_rxBufferRead;
    extern volatile uint8 mUART1_rxBufferWrite;
    extern volatile uint8 mUART1_rxBufferLoopDetect;
    extern volatile uint8 mUART1_rxBufferOverflow;
    #if (mUART1_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 mUART1_rxAddressMode;
        extern volatile uint8 mUART1_rxAddressDetected;
    #endif /* (mUART1_RXHW_ADDRESS_ENABLED) */
#endif /* (mUART1_RX_INTERRUPT_ENABLED && (mUART1_RX_ENABLED || mUART1_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define mUART1__B_UART__AM_SW_BYTE_BYTE 1
#define mUART1__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define mUART1__B_UART__AM_HW_BYTE_BY_BYTE 3
#define mUART1__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define mUART1__B_UART__AM_NONE 0

#define mUART1__B_UART__NONE_REVB 0
#define mUART1__B_UART__EVEN_REVB 1
#define mUART1__B_UART__ODD_REVB 2
#define mUART1__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define mUART1_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define mUART1_NUMBER_OF_STOP_BITS    (1u)

#if (mUART1_RXHW_ADDRESS_ENABLED)
    #define mUART1_RX_ADDRESS_MODE    (0u)
    #define mUART1_RX_HW_ADDRESS1     (0u)
    #define mUART1_RX_HW_ADDRESS2     (0u)
#endif /* (mUART1_RXHW_ADDRESS_ENABLED) */

#define mUART1_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << mUART1_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << mUART1_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << mUART1_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << mUART1_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << mUART1_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << mUART1_RX_STS_BREAK_SHIFT) \
                                        | (0 << mUART1_RX_STS_OVERRUN_SHIFT))

#define mUART1_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << mUART1_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << mUART1_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << mUART1_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << mUART1_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef mUART1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define mUART1_CONTROL_REG \
                            (* (reg8 *) mUART1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define mUART1_CONTROL_PTR \
                            (  (reg8 *) mUART1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End mUART1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(mUART1_TX_ENABLED)
    #define mUART1_TXDATA_REG          (* (reg8 *) mUART1_BUART_sTX_TxShifter_u0__F0_REG)
    #define mUART1_TXDATA_PTR          (  (reg8 *) mUART1_BUART_sTX_TxShifter_u0__F0_REG)
    #define mUART1_TXDATA_AUX_CTL_REG  (* (reg8 *) mUART1_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define mUART1_TXDATA_AUX_CTL_PTR  (  (reg8 *) mUART1_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define mUART1_TXSTATUS_REG        (* (reg8 *) mUART1_BUART_sTX_TxSts__STATUS_REG)
    #define mUART1_TXSTATUS_PTR        (  (reg8 *) mUART1_BUART_sTX_TxSts__STATUS_REG)
    #define mUART1_TXSTATUS_MASK_REG   (* (reg8 *) mUART1_BUART_sTX_TxSts__MASK_REG)
    #define mUART1_TXSTATUS_MASK_PTR   (  (reg8 *) mUART1_BUART_sTX_TxSts__MASK_REG)
    #define mUART1_TXSTATUS_ACTL_REG   (* (reg8 *) mUART1_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define mUART1_TXSTATUS_ACTL_PTR   (  (reg8 *) mUART1_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(mUART1_TXCLKGEN_DP)
        #define mUART1_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define mUART1_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define mUART1_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define mUART1_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define mUART1_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define mUART1_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define mUART1_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define mUART1_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define mUART1_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define mUART1_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) mUART1_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* mUART1_TXCLKGEN_DP */

#endif /* End mUART1_TX_ENABLED */

#if(mUART1_HD_ENABLED)

    #define mUART1_TXDATA_REG             (* (reg8 *) mUART1_BUART_sRX_RxShifter_u0__F1_REG )
    #define mUART1_TXDATA_PTR             (  (reg8 *) mUART1_BUART_sRX_RxShifter_u0__F1_REG )
    #define mUART1_TXDATA_AUX_CTL_REG     (* (reg8 *) mUART1_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define mUART1_TXDATA_AUX_CTL_PTR     (  (reg8 *) mUART1_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define mUART1_TXSTATUS_REG           (* (reg8 *) mUART1_BUART_sRX_RxSts__STATUS_REG )
    #define mUART1_TXSTATUS_PTR           (  (reg8 *) mUART1_BUART_sRX_RxSts__STATUS_REG )
    #define mUART1_TXSTATUS_MASK_REG      (* (reg8 *) mUART1_BUART_sRX_RxSts__MASK_REG )
    #define mUART1_TXSTATUS_MASK_PTR      (  (reg8 *) mUART1_BUART_sRX_RxSts__MASK_REG )
    #define mUART1_TXSTATUS_ACTL_REG      (* (reg8 *) mUART1_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define mUART1_TXSTATUS_ACTL_PTR      (  (reg8 *) mUART1_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End mUART1_HD_ENABLED */

#if( (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) )
    #define mUART1_RXDATA_REG             (* (reg8 *) mUART1_BUART_sRX_RxShifter_u0__F0_REG )
    #define mUART1_RXDATA_PTR             (  (reg8 *) mUART1_BUART_sRX_RxShifter_u0__F0_REG )
    #define mUART1_RXADDRESS1_REG         (* (reg8 *) mUART1_BUART_sRX_RxShifter_u0__D0_REG )
    #define mUART1_RXADDRESS1_PTR         (  (reg8 *) mUART1_BUART_sRX_RxShifter_u0__D0_REG )
    #define mUART1_RXADDRESS2_REG         (* (reg8 *) mUART1_BUART_sRX_RxShifter_u0__D1_REG )
    #define mUART1_RXADDRESS2_PTR         (  (reg8 *) mUART1_BUART_sRX_RxShifter_u0__D1_REG )
    #define mUART1_RXDATA_AUX_CTL_REG     (* (reg8 *) mUART1_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define mUART1_RXBITCTR_PERIOD_REG    (* (reg8 *) mUART1_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define mUART1_RXBITCTR_PERIOD_PTR    (  (reg8 *) mUART1_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define mUART1_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) mUART1_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define mUART1_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) mUART1_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define mUART1_RXBITCTR_COUNTER_REG   (* (reg8 *) mUART1_BUART_sRX_RxBitCounter__COUNT_REG )
    #define mUART1_RXBITCTR_COUNTER_PTR   (  (reg8 *) mUART1_BUART_sRX_RxBitCounter__COUNT_REG )

    #define mUART1_RXSTATUS_REG           (* (reg8 *) mUART1_BUART_sRX_RxSts__STATUS_REG )
    #define mUART1_RXSTATUS_PTR           (  (reg8 *) mUART1_BUART_sRX_RxSts__STATUS_REG )
    #define mUART1_RXSTATUS_MASK_REG      (* (reg8 *) mUART1_BUART_sRX_RxSts__MASK_REG )
    #define mUART1_RXSTATUS_MASK_PTR      (  (reg8 *) mUART1_BUART_sRX_RxSts__MASK_REG )
    #define mUART1_RXSTATUS_ACTL_REG      (* (reg8 *) mUART1_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define mUART1_RXSTATUS_ACTL_PTR      (  (reg8 *) mUART1_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) */

#if(mUART1_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define mUART1_INTCLOCK_CLKEN_REG     (* (reg8 *) mUART1_IntClock__PM_ACT_CFG)
    #define mUART1_INTCLOCK_CLKEN_PTR     (  (reg8 *) mUART1_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define mUART1_INTCLOCK_CLKEN_MASK    mUART1_IntClock__PM_ACT_MSK
#endif /* End mUART1_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(mUART1_TX_ENABLED)
    #define mUART1_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End mUART1_TX_ENABLED */

#if(mUART1_HD_ENABLED)
    #define mUART1_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End mUART1_HD_ENABLED */

#if( (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) )
    #define mUART1_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define mUART1_WAIT_1_MS      mUART1_BL_CHK_DELAY_MS   

#define mUART1_TXBUFFERSIZE   mUART1_TX_BUFFER_SIZE
#define mUART1_RXBUFFERSIZE   mUART1_RX_BUFFER_SIZE

#if (mUART1_RXHW_ADDRESS_ENABLED)
    #define mUART1_RXADDRESSMODE  mUART1_RX_ADDRESS_MODE
    #define mUART1_RXHWADDRESS1   mUART1_RX_HW_ADDRESS1
    #define mUART1_RXHWADDRESS2   mUART1_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define mUART1_RXAddressMode  mUART1_RXADDRESSMODE
#endif /* (mUART1_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define mUART1_initvar                    mUART1_initVar

#define mUART1_RX_Enabled                 mUART1_RX_ENABLED
#define mUART1_TX_Enabled                 mUART1_TX_ENABLED
#define mUART1_HD_Enabled                 mUART1_HD_ENABLED
#define mUART1_RX_IntInterruptEnabled     mUART1_RX_INTERRUPT_ENABLED
#define mUART1_TX_IntInterruptEnabled     mUART1_TX_INTERRUPT_ENABLED
#define mUART1_InternalClockUsed          mUART1_INTERNAL_CLOCK_USED
#define mUART1_RXHW_Address_Enabled       mUART1_RXHW_ADDRESS_ENABLED
#define mUART1_OverSampleCount            mUART1_OVER_SAMPLE_COUNT
#define mUART1_ParityType                 mUART1_PARITY_TYPE

#if( mUART1_TX_ENABLED && (mUART1_TXBUFFERSIZE > mUART1_FIFO_LENGTH))
    #define mUART1_TXBUFFER               mUART1_txBuffer
    #define mUART1_TXBUFFERREAD           mUART1_txBufferRead
    #define mUART1_TXBUFFERWRITE          mUART1_txBufferWrite
#endif /* End mUART1_TX_ENABLED */
#if( ( mUART1_RX_ENABLED || mUART1_HD_ENABLED ) && \
     (mUART1_RXBUFFERSIZE > mUART1_FIFO_LENGTH) )
    #define mUART1_RXBUFFER               mUART1_rxBuffer
    #define mUART1_RXBUFFERREAD           mUART1_rxBufferRead
    #define mUART1_RXBUFFERWRITE          mUART1_rxBufferWrite
    #define mUART1_RXBUFFERLOOPDETECT     mUART1_rxBufferLoopDetect
    #define mUART1_RXBUFFER_OVERFLOW      mUART1_rxBufferOverflow
#endif /* End mUART1_RX_ENABLED */

#ifdef mUART1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define mUART1_CONTROL                mUART1_CONTROL_REG
#endif /* End mUART1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(mUART1_TX_ENABLED)
    #define mUART1_TXDATA                 mUART1_TXDATA_REG
    #define mUART1_TXSTATUS               mUART1_TXSTATUS_REG
    #define mUART1_TXSTATUS_MASK          mUART1_TXSTATUS_MASK_REG
    #define mUART1_TXSTATUS_ACTL          mUART1_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(mUART1_TXCLKGEN_DP)
        #define mUART1_TXBITCLKGEN_CTR        mUART1_TXBITCLKGEN_CTR_REG
        #define mUART1_TXBITCLKTX_COMPLETE    mUART1_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define mUART1_TXBITCTR_PERIOD        mUART1_TXBITCTR_PERIOD_REG
        #define mUART1_TXBITCTR_CONTROL       mUART1_TXBITCTR_CONTROL_REG
        #define mUART1_TXBITCTR_COUNTER       mUART1_TXBITCTR_COUNTER_REG
    #endif /* mUART1_TXCLKGEN_DP */
#endif /* End mUART1_TX_ENABLED */

#if(mUART1_HD_ENABLED)
    #define mUART1_TXDATA                 mUART1_TXDATA_REG
    #define mUART1_TXSTATUS               mUART1_TXSTATUS_REG
    #define mUART1_TXSTATUS_MASK          mUART1_TXSTATUS_MASK_REG
    #define mUART1_TXSTATUS_ACTL          mUART1_TXSTATUS_ACTL_REG
#endif /* End mUART1_HD_ENABLED */

#if( (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) )
    #define mUART1_RXDATA                 mUART1_RXDATA_REG
    #define mUART1_RXADDRESS1             mUART1_RXADDRESS1_REG
    #define mUART1_RXADDRESS2             mUART1_RXADDRESS2_REG
    #define mUART1_RXBITCTR_PERIOD        mUART1_RXBITCTR_PERIOD_REG
    #define mUART1_RXBITCTR_CONTROL       mUART1_RXBITCTR_CONTROL_REG
    #define mUART1_RXBITCTR_COUNTER       mUART1_RXBITCTR_COUNTER_REG
    #define mUART1_RXSTATUS               mUART1_RXSTATUS_REG
    #define mUART1_RXSTATUS_MASK          mUART1_RXSTATUS_MASK_REG
    #define mUART1_RXSTATUS_ACTL          mUART1_RXSTATUS_ACTL_REG
#endif /* End  (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) */

#if(mUART1_INTERNAL_CLOCK_USED)
    #define mUART1_INTCLOCK_CLKEN         mUART1_INTCLOCK_CLKEN_REG
#endif /* End mUART1_INTERNAL_CLOCK_USED */

#define mUART1_WAIT_FOR_COMLETE_REINIT    mUART1_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_mUART1_H */


/* [] END OF FILE */
