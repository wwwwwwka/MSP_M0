#include "ADC.h"
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"

bool getValue = false;

volatile uint16_t gAdcResult0;  //速度1
volatile uint16_t gAdcResult1;  //速度2


// void ADC_Value_Get()
// {
//     DL_ADC12_startConversion(ADC_IN_INST);
//     while (getValue == false);

//     gAdcResult0 = DL_ADC12_getMemResult(ADC_IN_INST, ADC_IN_ADCMEM_0);
//     gAdcResult1 = DL_ADC12_getMemResult(ADC_IN_INST, ADC_IN_ADCMEM_1);

//     getValue = false;
//     DL_ADC12_enableConversions(ADC_IN_INST);

// }

// void ADC_IN_INST_IRQHandler()
// {
//     switch (DL_ADC12_getPendingInterrupt(ADC_IN_INST)) 
//     {
//         case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
//             getValue = true;
//             break;
//         default:
//             break;
//     }

// }