/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: FYPver3.c$
******************************************************************************/

/**
*  Describe:    This program is the Final Year Project "Blind swimming device"
*               program. Through gyroscope to solve direction problem, infrared                
*               sensor to solve edge detect problem. Using timer1 to drive
*               gyroscope, timer0 provide PWM to drive motor. Timer1 also 
*               using to provide measure signal for infrared sensor. 
*
*               Task job is been used to design the system. To save energy,
*               normally the MC is working at low-power-mode-0. Task causing         
*               intterupt.
*
*
*  Versions:    Ver1:           Original vertion.
*
*               Ver1-Ver2:      1. More suitable for blind user.  
*                               2. Vdd Check function added.
*                               3. Bug fixed.
*
*               Ver2-Ver3:      1. Bug fixed
*                               2. ReCalibrationg function added.       
*               
*  
*  PINS:        The I/O usage condition are as follow:
*                       P1^0    -       ADC10
*                       P1^2    -       PWM
*                       P1^4    -       VDD Check
*                       P1^6    -       I2C (SCL)
*                       P1^7    -       I2C (SDL)
*                       P2^0    -       LED0                       
*                       P2^1    -       LEFT_MOTOR
*                       P2^2    -       RIGHT_MOTOR
*                       P2^5    -       KEY0        
*
*                       P2^3    -       TEST PIN
**/

/* Include */
#include <msp430g2553.h>
#include <string.h>
#include <math.h>

#include "FYPver3.h"

#include "I2CDriver\msp430_i2c.h"
#include "MPU6050Driver\msp430_mpu6050.h"
#include "HardwareDriver\msp430_hardware.h"
#include "PWMDriver\msp430_pwm.h"
#include "ADCDriver\msp430_adc10.h"

/* Parameter */
uint8 DistanceTaskTimeCount = 0;
uint8 VDDCHECK = 10;
uint16 ReCalibrateTimer = 0;
uint16 ReCalibrateCounter = 0;

/* Gyroscope */
uint8 rx_buf[20] = {0};
uint8 tx_buf[20] = {0};

float yangle_reference = 0;
float GYRO_XANGLEdeg = 0;
float GYRO_YANGLEdeg = 0;
float GYRO_ZANGLEdeg = 0;

/* Infrared Sensor */
volatile uint8 ADC10_8BITBUF = 0;

/* System Task */
volatile short AngleTask = OFF;
volatile short DistanceTask = OFF;
volatile short ReCalibrateTask = OFF;

/* System Status */
short Distance_HighPrio = 0;
volatile short POWERKEY = 0;
volatile short KEYSTATUS = 1;
volatile short ReCalibrateStatus = 0;

#define KEYFREE         1
#define KEYPRESS        0

/* Function Integration */
#include "Function.h"


/* Main */
void main(void)
{
  /* Initial System */
  init();
  
  /* Operating System, no return */
  OS();
  
  WDTCTL = 0xff00;    //Unreachable, Reset device
}


/* ISR 
 *
 * Not include I2C Interrupt
 *
 **/
// Timer1 interrupt
#pragma vector=TIMER1_A0_VECTOR
__interrupt void ctimer_A0(void)
{
  /* Calculate Orientation Every 10ms */
  if(!ReCalibrateTask)
    AngleTask = ON;
  
  /* ReCalibration Timer */
  if(ReCalibrateStatus)
  {
    ReCalibrateTimer ++;
    if(ReCalibrateTimer >= (ReCalibrateTime + 50))
    {
      MOTOR_ON;
      __delay_cycles(3000000); // Delay 0.25s
      MOTOR_OFF;
      ReCalibrateTimer = ReCalibrateTime;
    }
  }
  
  /* Active Mode */
  LPM0_EXIT;
}

// ADC10 interrupt
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  if(!VDDCHECK)
    DistanceTask = ON;
  else
    VDDCHECK--;
  
  ADC10_8BITBUF = (ADC10MEM >> 2);
  LPM0_EXIT;
}


// P2 interrupt
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
/* Press Key:
 *
 * 1. Reset Direction;
 * 2. Press 2s to Recalibrate the gyroscope
 *
 **/  
  if(HWPIFG & BIT(KEY0))
  {
    switch(KEYSTATUS)
    {
    case KEYFREE:
      __delay_cycles(120000); // Delay for 10ms
      if((HWPIN & BIT(KEY0)) == 0)
      {
        /* Reset Direction */
        GYRO_XANGLE = 0;
        GYRO_YANGLE = 0;
        GYRO_ZANGLE = 0;
        
        /* Start 2s Timer */
        ReCalibrateStatus = ON;
        
        /* Prepare for release button interrupt */
        HWPIES &= ~BIT(KEY0); // lo-hi interrupt
        KEYSTATUS = KEYPRESS;
        
        /* Active Mode */
        LPM0_EXIT;
      }
      break;
        
    case KEYPRESS:
      __delay_cycles(120000); // Delay for 10ms
      if((HWPIN & BIT(KEY0)) != 0)
      {
        /* Determin whether recalibrate */
        ReCalibrateStatus = OFF;
        if(ReCalibrateTimer >= ReCalibrateTime)
        {
          ReCalibrateTask = ON;
          ReCalibrateCounter = 0;
        }
        ReCalibrateTimer = 0;
        
        /* Prepare for press button interrupt */
        HWPIES |= BIT(KEY0); // hi-lo interrupt
        KEYSTATUS = KEYFREE;   
    
        /* Bypass Powerup function */
        POWERKEY = ON;
        
        /* Active Mode */
        LPM0_EXIT;    
      }
      break;
        
    default:
      break; 
    }
  }
  HWPIFG &= ~BIT(KEY0);  
}
