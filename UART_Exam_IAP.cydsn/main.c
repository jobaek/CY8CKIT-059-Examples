/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    mUART1_Start();

    for(;;)
    {
        /* Place your application code here. */
        LED_P21_Write(1);
        CyDelay(500);
        
        LED_P21_Write(0);
        CyDelay(500);
        
        mUART1_PutString("Test Message !!! \r\n");
    }
}

/* [] END OF FILE */
