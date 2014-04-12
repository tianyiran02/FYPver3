/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_hardware.h$
******************************************************************************/

#ifndef _MSP430_HARDWARE_H_
#define _MSP430_HARDWARE_H_

/* Port Declaration */
#ifndef _BIT
#define BIT(x)          (1<<(x))
#endif

#define HWPOUT          P2OUT
#define HWPIN           P2IN
#define HWPSEL          P2SEL
#define HWPSEL2         P2SEL
#define HWPDIR          P2DIR
#define HWPIE           P2IE
#define HWPREN          P2REN  
#define HWPIES          P2IES
#define HWPIFG          P2IFG

#define LED0            0
#define L_MOTOR         1
#define R_MOTOR         2
#define KEY0            5       


/* Function Marcos */

/* LED */
#define LED0_OFF        HWPOUT |= BIT(LED0)
#define LED0_ON         HWPOUT &= ~BIT(LED0)

/* MOTOR */
#define MOTOR_ON        HWPOUT |= (BIT(R_MOTOR) + BIT(L_MOTOR))
#define MOTOR_OFF       HWPOUT &= ~(BIT(R_MOTOR) + BIT(L_MOTOR))        

#define R_MOTOR_ON      HWPOUT |= BIT(R_MOTOR)
#define R_MOTOR_OFF     HWPOUT &= ~BIT(R_MOTOR)

#define L_MOTOR_ON      HWPOUT |= BIT(L_MOTOR)
#define L_MOTOR_OFF     HWPOUT &= ~BIT(L_MOTOR)

/* KEY */


/* Function Declaration */
void msp430_initHW(void);
#endif
