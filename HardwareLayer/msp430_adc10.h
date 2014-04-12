/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_adc10.h$
******************************************************************************/


#ifndef _MSP430_ADC10_H_
#define _MSP430_ADC10_H_


/*Port Declaration*/
#ifndef _BIT
#define BIT(x)          (1<<(x))
#endif

/* Marcos Declaration */
#define ADC10_START       ADC10CTL0 |= ENC + ADC10SC

/* Function Declaration */
void initADC10(void);

#endif
