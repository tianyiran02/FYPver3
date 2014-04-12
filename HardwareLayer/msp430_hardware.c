/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_hardware.c$
******************************************************************************/

/**
*  Describe: This driver provide essentil macro and function for msp430
*            hardware. Including LED and L_Motor, R_Motor driving program.
*
**/

#include <msp430g2553.h>
#include "msp430_hardware.h"

/******************************************************************************
Name:           void msp430_initHW()
Function:       Initial Function
Parameter:      return status
Attention:
******************************************************************************/
void msp430_initHW(void)
{
  LED0_ON;
  HWPSEL &= ~(BIT(LED0) + BIT(R_MOTOR) + BIT(L_MOTOR) + BIT(KEY0));
  HWPSEL2 &= ~(BIT(LED0) + BIT(R_MOTOR) + BIT(L_MOTOR) +BIT(KEY0));
  HWPOUT &= ~(BIT(R_MOTOR) + BIT(L_MOTOR));
  HWPOUT |= BIT(KEY0);
  HWPREN |= BIT(KEY0);
  HWPDIR |= BIT(LED0) + BIT(R_MOTOR) + BIT(L_MOTOR);
  HWPDIR &= ~BIT(KEY0);
  
  HWPIES |= BIT(KEY0); // HI-LO edge interrupt
  HWPIFG &= ~BIT(KEY0);
  HWPIE |= BIT(KEY0);
}
