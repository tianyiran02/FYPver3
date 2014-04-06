/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: Function.h$
******************************************************************************/

/**
*  Describe: This .h file is a integrating of initilise and loop function
*
**/

void init(void);
void AngleCaculate(void);
void DistanceCaculate(void);
void VDDCheck(void);
void Powerup(void);
void OS(void);

/******************************************************************************
Name:           init(void)
Function:       initialise the system
Parameter:      none
Attention:      none
******************************************************************************/
void init(void)
{
  /* WDT and System Clodk */
  WDTCTL = WDTPW + WDTHOLD; // WDT close
  
  /* Hardware Initilise */
  msp430_initHW(); // Initialise LED, Motor I/O and KEY0  
  
  /* Change DCO to 12MHz */
  if((CALBC1_12MHZ==0Xff) || (CALDCO_12MHZ==0xff))      // Select DCO=>12MHZ
     while(1);
  else
  {
    BCSCTL1 = CALBC1_12MHZ;                    
    DCOCTL = CALDCO_12MHZ;
  }  
     
  /* VDD Check */
  VDDCheck();
   
  /* Wait for Power up */
  Powerup();
  
  /* Initialize I2C */
  startI2C(); 
  closeI2C();
  
  /* PWM Initialize */
  initPWM(65); // Initilise PWM, 65% dutycycle
  PWMSTAT(START);  
  
  /* MPU6050 initialize */
  __delay_cycles(300000); // Delay for 25ms
  testMPU6050I2C();
  __delay_cycles(300000); // Delay for 25ms
  initMPU6050();
  __delay_cycles(3000000); // Delay for 250ms
  
  Calibrate_Gyros();
  
  /* Timer and ADC Initialize */
  initADC10();  
  TA1CTL = TASSEL_2;// SMCLK, up-down mode
  TA1CCR0 = 60000; 
  TA1CCTL0 = CCIE;

  /* Indicate system ready for use */
  MOTOR_ON;
  __delay_cycles(12000000); // Delay for 1s to indicate calibrate finish
  
  /* Re-initialize MPU6050 */
  initMPU6050();  
  
  MOTOR_OFF;
  LED0_OFF;    
  
  /* Enable Interrupt, Power Save Mode, Start Timer */
  TA1CTL |= MC_3;
  _EINT();
  LPM0;
}


/******************************************************************************
Name:           AngleCaculate(void)
Function:       Caculate the direction and reactaccording to the gyroscope data
Parameter:      none
Attention:      none
******************************************************************************/
void AngleCaculate(void)
{
  /* Caculate the yaw angle (degree) according to gyro */
  Get_Gyro_Rates();
  
  GYRO_XANGLEdeg = (GYRO_XANGLE * PI)/180;
  GYRO_YANGLEdeg = (GYRO_YANGLE * PI)/180;
  GYRO_ZANGLEdeg = (GYRO_ZANGLE * PI)/180;
  
  yangle_reference = 
     atan(((sin(GYRO_XANGLEdeg)*sin(GYRO_YANGLEdeg)*cos(GYRO_ZANGLEdeg)) - cos(GYRO_XANGLEdeg)*sin(GYRO_ZANGLEdeg))
          /((sin(GYRO_XANGLEdeg)*sin(GYRO_YANGLEdeg)*sin(GYRO_ZANGLEdeg)) + cos(GYRO_XANGLEdeg)*cos(GYRO_ZANGLEdeg)));
  yangle_reference = (yangle_reference * 180)/PI;
  
  
  /* React according to the angle and phase */  
  if(!Distance_HighPrio)
    // Only change direction when not close to edge
  {
  if((yangle_reference >= -7) && (yangle_reference <= 7))
  {
    LED0_OFF;
    MOTOR_OFF;
  }
  else
  {
    LED0_ON;
    
    if(yangle_reference < 0)
    {
      R_MOTOR_ON;
      L_MOTOR_OFF;
    }
    else if(yangle_reference > 0)
    {
      L_MOTOR_ON;
      R_MOTOR_OFF;
    }
  }
  }
}


/******************************************************************************
Name:           DistanceCaculate(void)
Function:       Caculate the distanse and react according to the gyroscope data
Parameter:      none
Attention:      none
******************************************************************************/
void DistanceCaculate(void)
{
  if(ADC10_8BITBUF > 0x2f)
  {
    MOTOR_ON;
    Distance_HighPrio = 1;
  }
  else
    Distance_HighPrio = 0;
}


/******************************************************************************
Name:           VDDCheck(void)
Function:       Check VDD
Parameter:      none
Attention:      none
******************************************************************************/
void VDDCheck(void)
{
  /* Setup ADC, Port 1_4 */
  P1SEL |= BIT4;
  P1SEL2 |= BIT4;
  P1DIR &= ~BIT4;
  P1OUT &= ~BIT4;
  ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE; // 16 SH time, On ADC, Enable IE
  ADC10CTL1 = INCH_4;  // Input P1.4
  ADC10_START; // Start first converting
  
  _EINT();
  LPM0;
  
  /* Wait for ADC Result */
  while(VDDCHECK)
  {
    if(ADC10_8BITBUF < 170)
    {
      VDDCHECK = 0;
      break;
    }
    else 
      ADC10_START; 
  } 
 
  /* React According to VDD Check Result */  
  if(ADC10_8BITBUF < 170) 
  {
    /* Setup Vibration Motor */
    P1DIR |= BIT2;
    P1SEL &= ~BIT2;
    P1SEL2 &= ~BIT2;      
    P1OUT |= BIT2;
    
    MOTOR_ON;
    __delay_cycles(6000000); // Delay for 500ms
    MOTOR_OFF;
    __delay_cycles(6000000); // Delay for 500ms
    MOTOR_ON;
    __delay_cycles(6000000); // Delay for 500ms
    MOTOR_OFF;
  }
  
  __delay_cycles(6000000); // Delay for 500ms
  
  _DINT();
}


/******************************************************************************
Name:           Powerup(void)
Function:       Wait for key to power up
Parameter:      none
Attention:      none
******************************************************************************/
void Powerup(void)
{
  _EINT();
  LPM0;
  while(!POWERKEY);
  _DINT();
}

/******************************************************************************
Name:           OS(void)
Function:       Integrating function
Parameter:      none
Attention:      none
******************************************************************************/
void OS(void)
{
while(1){
while(AngleTask || DistanceTask || ReCalibrateTask)
{
  if(AngleTask || ReCalibrateTask)
  {
    if(ReCalibrateTask)
    {
      ReCalibrate();
      AngleTask = OFF;
    }
    else if(AngleTask)
    {
      AngleCaculate();
      AngleTask = OFF;
      
      DistanceTaskTimeCount ++;
      if(DistanceTaskTimeCount == 10)
      { 
        DistanceTaskTimeCount = 0;
        ADC10_START;
      }
    }
  }
  else if(DistanceTask)
  {
    DistanceCaculate();
    DistanceTask = OFF;
  }
}
LPM0;
}
}
