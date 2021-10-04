/*******************************************************************************
* File Name: mUART1_PM.c
* Version 2.50
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
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


/***************************************
* Local data allocation
***************************************/

static mUART1_BACKUP_STRUCT  mUART1_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: mUART1_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the mUART1_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  mUART1_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void mUART1_SaveConfig(void)
{
    #if(mUART1_CONTROL_REG_REMOVED == 0u)
        mUART1_backup.cr = mUART1_CONTROL_REG;
    #endif /* End mUART1_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: mUART1_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the nonretention control register except FIFO.
*  Does not restore the FIFO which is a set of nonretention registers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  mUART1_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling mUART1_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void mUART1_RestoreConfig(void)
{
    #if(mUART1_CONTROL_REG_REMOVED == 0u)
        mUART1_CONTROL_REG = mUART1_backup.cr;
    #endif /* End mUART1_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: mUART1_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The mUART1_Sleep() API saves the current component state. Then it
*  calls the mUART1_Stop() function and calls 
*  mUART1_SaveConfig() to save the hardware configuration.
*  Call the mUART1_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  mUART1_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void mUART1_Sleep(void)
{
    #if(mUART1_RX_ENABLED || mUART1_HD_ENABLED)
        if((mUART1_RXSTATUS_ACTL_REG  & mUART1_INT_ENABLE) != 0u)
        {
            mUART1_backup.enableState = 1u;
        }
        else
        {
            mUART1_backup.enableState = 0u;
        }
    #else
        if((mUART1_TXSTATUS_ACTL_REG  & mUART1_INT_ENABLE) !=0u)
        {
            mUART1_backup.enableState = 1u;
        }
        else
        {
            mUART1_backup.enableState = 0u;
        }
    #endif /* End mUART1_RX_ENABLED || mUART1_HD_ENABLED*/

    mUART1_Stop();
    mUART1_SaveConfig();
}


/*******************************************************************************
* Function Name: mUART1_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  mUART1_Sleep() was called. The mUART1_Wakeup() function
*  calls the mUART1_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  mUART1_Sleep() function was called, the mUART1_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  mUART1_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void mUART1_Wakeup(void)
{
    mUART1_RestoreConfig();
    #if( (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) )
        mUART1_ClearRxBuffer();
    #endif /* End (mUART1_RX_ENABLED) || (mUART1_HD_ENABLED) */
    #if(mUART1_TX_ENABLED || mUART1_HD_ENABLED)
        mUART1_ClearTxBuffer();
    #endif /* End mUART1_TX_ENABLED || mUART1_HD_ENABLED */

    if(mUART1_backup.enableState != 0u)
    {
        mUART1_Enable();
    }
}


/* [] END OF FILE */
