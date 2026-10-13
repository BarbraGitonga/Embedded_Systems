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
#include <project.h>
#include <stdio.h>
#include <math.h>

int main(void)
{
    CyGlobalIntEnable;
    ADC_SAR_1_Start();
    ADC_SAR_1_StartConvert();

    USBUART_1_Start(0, USBUART_1_5V_OPERATION);

    // Wait for connection
    while(!USBUART_1_GetConfiguration()) { }
    USBUART_1_CDC_Init();

    for(;;)
    {
        // Read LDR voltage
        ADC_SAR_1_IsEndConversion(ADC_SAR_1_WAIT_FOR_RESULT);
        int16 adcResult = ADC_SAR_1_GetResult16();

        int16 mv = ADC_SAR_1_CountsTo_mVolts(adcResult);
        float voltage = mv / 1000.0f;


        // Prepare message
        char msg[60];
        sprintf(msg, "ADC counts: %d, Voltage: %.2f V\r\n", adcResult, voltage);


        // Send via USB
        if(USBUART_1_CDCIsReady())
        {
            USBUART_1_PutData((uint8*)msg, strlen(msg));
        }

        CyDelay(500);
    }
}

/* [] END OF FILE */
