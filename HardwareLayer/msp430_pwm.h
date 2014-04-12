/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_pwm.h$
******************************************************************************/

#ifndef MSP430_PWM_H_
#define MSP430_PWM_H_

/*Macro*/
#define START 1
#define STOP 0

#ifdef SYS_CLOCK
#define PWMCLOCK        SYS_CLOCK
#else
#define SYS_CLOCK       1000000
#define PWMCLOCK        SYS_CLOCK
#endif

#ifndef _TYPE_UINT_
#define _TYPE_UINT_
typedef unsigned char   uint8;          
typedef unsigned int    uint16;           
#endif

/*Port Declaration*/
#ifndef _BIT
#define BIT(x)          (1<<(x))
#endif

#define PWMPDIR         P1DIR
#define PWMPSEL         P1SEL
#define PWMPOUT         P1OUT
#define PWMBIT          2

/*Parameter Declaration*/


/*Function Declaration*/
void initPWM(unsigned char Dutycycles);
void PWMSTAT(unsigned STAT);

#endif
