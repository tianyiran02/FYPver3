/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_adc10.c$
******************************************************************************/

/**
*  Describe: This driver provide essential function for ADC10
*
**/

#include <msp430g2553.h>
#include "msp430_adc10.h"

/******************************************************************************
Name：void initADC10(void)
Function：initial ADC, setting register
Parameter：non
******************************************************************************/
void initADC10(void)
{
  P1SEL |= BIT1;
  P1SEL2 |= BIT1;
  ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE; // 16 SH time, On ADC, enable IE
  ADC10CTL1 = INCH_0;  // Input P1.0
  ADC10_START; // Start first converting
}
