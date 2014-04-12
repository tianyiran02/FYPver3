/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_mpu6050.c$
******************************************************************************/

#include <msp430g2553.h>
#include <string.h>
#include <math.h>
#include "msp430_mpu6050.h"
#include "..\I2CDriver\msp430_i2c.h"
#include "..\HardwareDriver\msp430_hardware.h"
#include "..\FYPver3.h"

short MPUSTATUS = 0;

extern uint16 ReCalibrateCounter;
extern volatile short ReCalibrateTask;

float dt=0;

/*Gyroscope Data Out*/
unsigned char GYRO_XOUT_H = 0;
unsigned char GYRO_XOUT_L = 0;
unsigned char GYRO_YOUT_H = 0;
unsigned char GYRO_YOUT_L = 0;
unsigned char GYRO_ZOUT_H = 0;
unsigned char GYRO_ZOUT_L = 0;

signed long int GYRO_XOUT_OFFSET_1000SUM = 0;
signed long int GYRO_YOUT_OFFSET_1000SUM = 0;
signed long int GYRO_ZOUT_OFFSET_1000SUM = 0;

signed int GYRO_XOUT_OFFSET = 0;
signed int GYRO_YOUT_OFFSET = 0;
signed int GYRO_ZOUT_OFFSET = 0;

signed int GYRO_XOUT = 0;
signed int GYRO_YOUT = 0;
signed int GYRO_ZOUT = 0;

float GYRO_XRATE = 0;
float GYRO_ZRATE = 0;
float GYRO_YRATE = 0;

float GYRO_XANGLE = 0;
float GYRO_YANGLE = 0;
float GYRO_ZANGLE = 0;

/*Accelerometer Data Out*/
unsigned char ACCEL_XOUT_H = 0;
unsigned char ACCEL_XOUT_L = 0;
unsigned char ACCEL_YOUT_H = 0;
unsigned char ACCEL_YOUT_L = 0;
unsigned char ACCEL_ZOUT_H = 0;
unsigned char ACCEL_ZOUT_L = 0;

signed int ACCEL_XOUT = 0;
signed int ACCEL_YOUT = 0;
signed int ACCEL_ZOUT = 0;

float ACCEL_XACCOUT = 0;
float ACCEL_YACCOUT = 0;
float ACCEL_ZACCOUT = 0;

float ACCEL_XANGLE = 0;
float ACCEL_YANGLE = 0;

extern unsigned char rx_buf[20];

/******************************************************************************
Name:           void testMPU6050I2C()
Function:       Initial Function
Parameter:      return status
Attention:
******************************************************************************/
void testMPU6050I2C(void)
{
  oneByteReadI2C(MPU6000, MPU6050_RA_WHO_AM_I,&rx_buf[0]);
  if(rx_buf[0] != 0x68)
  {
    __delay_cycles(1200000);     //delay 100ms
    WDTCTL = 0xff00;    //PUR
  }
  
  memset(rx_buf,0,sizeof(rx_buf));
}

/******************************************************************************
Name:           short initMPU6050()
Function:       Initial Function
Parameter:      return status
Attention:
******************************************************************************/
short initMPU6050(void)
{
  //unsigned char writecheck;
  if(MPUSTATUS)
    return 1;
  
  //Sets sample rate to 8000/1+7 = 1000Hz
  oneByteWriteI2C(MPU6000, MPU6050_RA_SMPLRT_DIV, 0x07);
  __delay_cycles(300000); // Delay for 25ms
  
  //readI2C(MPU6000,MPU6050_RA_SMPLRT_DIV,rx_buf,1);
  //if(!(rx_buf[0] & 0x07))
  //  return 0;
 
  //Disable FSync, 256Hz DLPF
  oneByteWriteI2C(MPU6000, MPU6050_RA_CONFIG, 0x00);
  //Disable gyro self tests, scale of 1000 degrees/s
  oneByteWriteI2C(MPU6000, MPU6050_RA_GYRO_CONFIG, 0x18);
  __delay_cycles(300000); // Delay for 25ms
  //oneByteReadI2C(MPU6000, MPU6050_RA_GYRO_CONFIG,&writecheck);
  //if(!(writecheck &= 0x18))
  //  return 0;  

  //Disable accel self tests, scale of +-2g, no DHPF
  oneByteWriteI2C(MPU6000, MPU6050_RA_ACCEL_CONFIG, 0x00);  
    __delay_cycles(300000); // Delay for 25ms
  //Freefall threshold of |0mg|
  oneByteWriteI2C(MPU6000, MPU6050_RA_FF_THR, 0x00);
  //Freefall duration limit of 0
  oneByteWriteI2C(MPU6000, MPU6050_RA_FF_DUR, 0x00);
  //Motion threshold of 0mg
  oneByteWriteI2C(MPU6000, MPU6050_RA_MOT_THR, 0x00);
  //Motion duration of 0s
  oneByteWriteI2C(MPU6000, MPU6050_RA_MOT_DUR, 0x00);
  //Zero motion threshold
  oneByteWriteI2C(MPU6000, MPU6050_RA_ZRMOT_THR, 0x00);
  //Zero motion duration threshold
  oneByteWriteI2C(MPU6000, MPU6050_RA_ZRMOT_DUR, 0x00);
  //Disable sensor output to FIFO buffer
  oneByteWriteI2C(MPU6000, MPU6050_RA_FIFO_EN, 0x00);

  //AUX I2C setup
  //Sets AUX I2C to single master control, plus other config
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_MST_CTRL, 0x00);
  //Setup AUX I2C slaves
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV0_REG, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV1_REG, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV2_REG, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV3_REG, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV4_REG, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV4_DO, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV4_DI, 0x00);

  //MPU6050_RA_I2C_MST_STATUS //Read-only
  //Setup INT pin and AUX I2C pass through
  oneByteWriteI2C(MPU6000, MPU6050_RA_INT_PIN_CFG, 0x00);
  //Enable data ready interrupt
  oneByteWriteI2C(MPU6000, MPU6050_RA_INT_ENABLE, 0x00);

  //MPU6050_RA_DMP_INT_STATUS        //Read-only
  //MPU6050_RA_INT_STATUS 3A        //Read-only
  //MPU6050_RA_ACCEL_XOUT_H         //Read-only
  //MPU6050_RA_ACCEL_XOUT_L         //Read-only
  //MPU6050_RA_ACCEL_YOUT_H         //Read-only
  //MPU6050_RA_ACCEL_YOUT_L         //Read-only
  //MPU6050_RA_ACCEL_ZOUT_H         //Read-only
  //MPU6050_RA_ACCEL_ZOUT_L         //Read-only
  //MPU6050_RA_TEMP_OUT_H         //Read-only
  //MPU6050_RA_TEMP_OUT_L         //Read-only
  //MPU6050_RA_GYRO_XOUT_H         //Read-only
  //MPU6050_RA_GYRO_XOUT_L         //Read-only
  //MPU6050_RA_GYRO_YOUT_H         //Read-only
  //MPU6050_RA_GYRO_YOUT_L         //Read-only
  //MPU6050_RA_GYRO_ZOUT_H         //Read-only
  //MPU6050_RA_GYRO_ZOUT_L         //Read-only
  //MPU6050_RA_EXT_SENS_DATA_00     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_01     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_02     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_03     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_04     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_05     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_06     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_07     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_08     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_09     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_10     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_11     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_12     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_13     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_14     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_15     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_16     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_17     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_18     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_19     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_20     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_21     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_22     //Read-only
  //MPU6050_RA_EXT_SENS_DATA_23     //Read-only
  //MPU6050_RA_MOT_DETECT_STATUS     //Read-only

  //Slave out, dont care
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV0_DO, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV1_DO, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV2_DO, 0x00);
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_SLV3_DO, 0x00);
  //More slave config
  oneByteWriteI2C(MPU6000, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
  //Reset sensor signal paths
  oneByteWriteI2C(MPU6000, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
  //Motion detection control
  oneByteWriteI2C(MPU6000, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
  //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
  oneByteWriteI2C(MPU6000, MPU6050_RA_USER_CTRL, 0x00);
  //Sets clock source to gyro reference w/ PLL
    __delay_cycles(300000); // Delay for 25ms
  oneByteWriteI2C(MPU6000, MPU6050_RA_PWR_MGMT_1, 0x02);
    __delay_cycles(300000); // Delay for 25ms
  //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
  oneByteWriteI2C(MPU6000, MPU6050_RA_PWR_MGMT_2, 0x00);
  //MPU6050_RA_BANK_SEL            //Not in datasheet
  //MPU6050_RA_MEM_START_ADDR        //Not in datasheet
  //MPU6050_RA_MEM_R_W            //Not in datasheet
  //MPU6050_RA_DMP_CFG_1            //Not in datasheet
  //MPU6050_RA_DMP_CFG_2            //Not in datasheet
  //MPU6050_RA_FIFO_COUNTH        //Read-only
  //MPU6050_RA_FIFO_COUNTL        //Read-only
  //Data transfer to and from the FIFO buffer
  oneByteWriteI2C(MPU6000, MPU6050_RA_FIFO_R_W, 0x00);
  //MPU6050_RA_WHO_AM_I             //Read-only, I2C address

  MPUSTATUS = 0;
  
  return MPUSTATUS;
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



/******************************************************************************
Name:           void Calibrate_Gyros(void)
Function:       Calibrate Gyroscope data
Parameter:      return status
Attention:
******************************************************************************/
void Calibrate_Gyros(void)
{
  GYRO_XOUT_OFFSET = (-26);
  GYRO_YOUT_OFFSET = 14;
  GYRO_ZOUT_OFFSET = (-6);
  
}	
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/******************************************************************************
Name:           void ReCalibrate(void)
Function:       Re_Calibrate Gyroscope offset
Parameter:      
Attention:
******************************************************************************/
void ReCalibrate(void)
{
  if(ReCalibrateCounter < 500)
  {
    oneByteReadI2C(MPU6000,MPU6050_RA_GYRO_XOUT_H,&GYRO_XOUT_H);
    oneByteReadI2C(MPU6000,MPU6050_RA_GYRO_XOUT_L,&GYRO_XOUT_L);
    oneByteReadI2C(MPU6000,MPU6050_RA_GYRO_YOUT_H,&GYRO_YOUT_H);
    oneByteReadI2C(MPU6000,MPU6050_RA_GYRO_YOUT_L,&GYRO_YOUT_L);
    oneByteReadI2C(MPU6000,MPU6050_RA_GYRO_ZOUT_H,&GYRO_ZOUT_H);
    oneByteReadI2C(MPU6000,MPU6050_RA_GYRO_ZOUT_L,&GYRO_ZOUT_L);

    GYRO_XOUT_OFFSET_1000SUM += ((GYRO_XOUT_H<<8)|GYRO_XOUT_L);
    GYRO_YOUT_OFFSET_1000SUM += ((GYRO_YOUT_H<<8)|GYRO_YOUT_L);
    GYRO_ZOUT_OFFSET_1000SUM += ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L);
    
    ReCalibrateCounter ++;
    
    LPM0;
  }
  else
  {
    GYRO_XOUT_OFFSET = (signed int)(GYRO_XOUT_OFFSET_1000SUM/500);
    GYRO_YOUT_OFFSET = (signed int)(GYRO_YOUT_OFFSET_1000SUM/500);
    GYRO_ZOUT_OFFSET = (signed int)(GYRO_ZOUT_OFFSET_1000SUM/500);
    
    ReCalibrateCounter = 0;
    
    /* Reset Direction */
    GYRO_XANGLE = 0;
    GYRO_YANGLE = 0;
    GYRO_ZANGLE = 0;
    
    ReCalibrateTask = OFF;
    
    MOTOR_ON;
    __delay_cycles(12000000); // Delay for 1s
    MOTOR_OFF;
  }
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
/******************************************************************************
Name:           void Get_Gyro_Rates(void)
Function:       Caculate the gyroscope rate convert it into degrees/s
Parameter:      
Attention:      
******************************************************************************/
void Get_Gyro_Rates(void)
{
  oneByteReadI2C(MPU6000, MPU6050_RA_GYRO_XOUT_H, &GYRO_XOUT_H);
  oneByteReadI2C(MPU6000, MPU6050_RA_GYRO_XOUT_L, &GYRO_XOUT_L);
  oneByteReadI2C(MPU6000, MPU6050_RA_GYRO_YOUT_H, &GYRO_YOUT_H);
  oneByteReadI2C(MPU6000, MPU6050_RA_GYRO_YOUT_L, &GYRO_YOUT_L);
  oneByteReadI2C(MPU6000, MPU6050_RA_GYRO_ZOUT_H, &GYRO_ZOUT_H);
  oneByteReadI2C(MPU6000, MPU6050_RA_GYRO_ZOUT_L, &GYRO_ZOUT_L);

  GYRO_XOUT = ((GYRO_XOUT_H<<8)|GYRO_XOUT_L) - GYRO_XOUT_OFFSET;
  GYRO_YOUT = ((GYRO_YOUT_H<<8)|GYRO_YOUT_L) - GYRO_YOUT_OFFSET;
  GYRO_ZOUT = ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L) - GYRO_ZOUT_OFFSET;


  GYRO_XRATE = (float)GYRO_XOUT/gyro_xsensitivity;
  GYRO_YRATE = (float)GYRO_YOUT/gyro_ysensitivity;
  GYRO_ZRATE = (float)GYRO_ZOUT/gyro_zsensitivity;
  
  GYRO_XANGLE += (GYRO_XRATE * 0.01);
  GYRO_YANGLE += (GYRO_YRATE * 0.01);
  GYRO_ZANGLE += (GYRO_ZRATE * 0.01);
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



/******************************************************************************
Name:           void Get_Accel_Values(void)
Function:       Get Accelerate data
Parameter:      return status
Attention:
******************************************************************************/
void Get_Accel_Values(void)
{
  oneByteReadI2C(MPU6000, MPU6050_RA_ACCEL_XOUT_H, &ACCEL_XOUT_H);
  oneByteReadI2C(MPU6000, MPU6050_RA_ACCEL_XOUT_L, &ACCEL_XOUT_L);
  oneByteReadI2C(MPU6000, MPU6050_RA_ACCEL_YOUT_H, &ACCEL_YOUT_H);
  oneByteReadI2C(MPU6000, MPU6050_RA_ACCEL_YOUT_L, &ACCEL_YOUT_L);
  oneByteReadI2C(MPU6000, MPU6050_RA_ACCEL_ZOUT_H, &ACCEL_ZOUT_H);
  oneByteReadI2C(MPU6000, MPU6050_RA_ACCEL_ZOUT_L, &ACCEL_ZOUT_L);

  ACCEL_XOUT = ((ACCEL_XOUT_H<<8)|ACCEL_XOUT_L);
  ACCEL_YOUT = ((ACCEL_YOUT_H<<8)|ACCEL_YOUT_L);
  ACCEL_ZOUT = ((ACCEL_ZOUT_H<<8)|ACCEL_ZOUT_L);

  ACCEL_XACCOUT = (((float)ACCEL_XOUT) / ACCEL_SENSITIVITY) * 9.81;
  ACCEL_YACCOUT = (((float)ACCEL_YOUT) / ACCEL_SENSITIVITY) * 9.81;
  ACCEL_ZACCOUT = (((float)ACCEL_ZOUT) / ACCEL_SENSITIVITY) * 9.81;
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/******************************************************************************
Name:           void Get_Accel_Angles(void)
Function:       Caculate the Accelerometer data, converte into 3D euler 
                angles.
Parameter:      
Attention:      
******************************************************************************/
void Get_Accel_Angles(void)
{
  ACCEL_XANGLE = 57.295*atan((float)ACCEL_YOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_XOUT,2)));
  ACCEL_YANGLE = 57.295*atan((float)-ACCEL_XOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_YOUT,2)));	
}	
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
