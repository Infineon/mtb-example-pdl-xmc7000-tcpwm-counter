/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the <CE234965>
*              Example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
********************************************************************************/

/*******************************************************************************
* Function Prototypes
********************************************************************************/

/*******************************************************************************
* Global Variables
********************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This counter will be triggered to start by PWM output signal, and counter period is
* 60000 with 200Khz clock frequency. Enable the CC0 match interrupt, CC0 match value
* is 50000. It will generate the interrupt when counter up to CC0 match value, then
* toggle user LED with interrupt generated.
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t intrMask;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /*Initialize the User LED*/
    Cy_GPIO_Pin_Init(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, &CYBSP_USER_LED1_config );

    /*TCPWM PWM Mode initial*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(TCPWM_PWM_HW, TCPWM_PWM_NUM, &TCPWM_PWM_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized PWM */
    Cy_TCPWM_PWM_Enable(TCPWM_PWM_HW, TCPWM_PWM_NUM);

    /* Then start the PWM */
    Cy_TCPWM_TriggerReloadOrIndex_Single(TCPWM_PWM_HW, TCPWM_PWM_NUM);

    /*TCPWM Counter Mode initial*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM, &TCPWM_COUNTER_config))
    {
        CY_ASSERT(0);
    }
    /* Enable the initialized counter */
    Cy_TCPWM_Counter_Enable(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);

    /* Then start the counter, the counter will be trigger by PWM signal */

    for (;;)
    {
        /*get the counter CC0 compare interrupt mask*/
        intrMask = Cy_TCPWM_GetInterruptStatusMasked(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);
        if (intrMask == CY_TCPWM_INT_ON_CC0)
        {
            /*toggle user LED1*/
            Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN);

            /*clear the counter CC0 compare interrupt*/
            Cy_TCPWM_ClearInterrupt(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM, CY_TCPWM_INT_ON_CC0);
        }
    }
}

/* [] END OF FILE */
