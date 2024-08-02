#ifndef	__ADC_H__
#define __ADC_H__

#include "ti_msp_dl_config.h"

extern bool getValue;

extern volatile uint16_t gAdcResult0;
extern volatile uint16_t gAdcResult1;

void ADC_Value_Get();

#endif
