/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_pwm.c$
******************************************************************************/

/**
*
*Describe:      Output two PWM through BIT(PWM1BIT) and BIT(PWM2BIT) using 
*               Timer_A. Clock is SMCLK at 1Mhz. Setting process are as 
*               follow:
*
*               1. initPWM(DutyCycles);
*               2. PWMSTAT(START);
*               3. PWMSTAT(STOP);
*
*Attention:     DOC should be setted before PWM be used.
*
**/

#include<msp430g2553.h>
#include "..\FYPver3.h"
#include "msp430_pwm.h"


/******************************************************************************
Name:           initPWM(uchar Dutycycles)
Function:       Initial MSP430G2553 PWM, set port, write register
Parameter:      uchar Dutycycles--Aim dutycycles
Attention:      Only use for MSP430 G2553
                Select SMCLK as clock,SMCLK is 12MHZ, t = 0.0833us
                Set PWM frequency at 20Khz, period = 50us
******************************************************************************/
void initPWM(unsigned char Dutycycles)
{
  /*Port control*/
  PWMPDIR |= BIT(PWMBIT);                       //Set PWMBIT output
  PWMPSEL |= BIT(PWMBIT);                       //Set as Timer output        
  
  /*Initial Timer*/
  CCR0 = (uint16)(25*((PWMCLOCK)/1000000));     //Period/2
  CCTL1 = OUTMOD_6;                             //CCR1 toggle/set mode
  CCR1 = (uint16)(CCR0 * (100-Dutycycles) / 100); //Duty cycle according to param
  TACTL |= TASSEL_2 + MC_0;                     //SMCLK, Stop Timer
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/******************************************************************************
Name:           PWMSTAT(uchar STAT)
Function:       Change PWM status, start or stop
Parameter:      uchar STAT--Aim status
Attention:      Only use for MSP430 G2553
******************************************************************************/
void PWMSTAT(unsigned STAT)
{
  switch (STAT)
  {
  case STOP:
    TACTL &= ~MC_3;                             //Stop Timer
    PWMPSEL &= ~BIT(PWMBIT);
    PWMPOUT &= ~BIT(PWMBIT);                    //Stop device
    break;
  
  case START:
    PWMPSEL |= BIT(PWMBIT);
    TACTL |= MC_3;                              //Start Timer
    break;
    
  default:
    break;
  }
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
