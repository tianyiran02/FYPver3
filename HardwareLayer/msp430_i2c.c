/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_i2c.c$
******************************************************************************/

/**
*  Describe: This driver provide essential function for I2C communication 
*            between TI msp430 G2553 Launchpad and PC. Communication frequency
*            is setted at 250KHz, 8Bit address mode. The process of using are
*            as follow:
*               1. #define I2CSlaveAddress in main          
*               1. startI2C();
*               2. writeI2C(Daddr,Raddr,*buf,Dlength)
*                  or readI2C(Daddr,Raddr,*buf,Dlength);
*
**/

#include <msp430g2553.h>
#include "msp430_i2c.h"

#define WRITE_I2C 1
#define READ_I2C 2
#define UNWRITE_REGISTER 0
#define WRITE_REGISTER 1

short enableI2C = 0;
short I2CSTAT = 0;      //set 1 -- Write Mode; 2 -- Read Mode
short writeReg = 0;     //Register Written flag. 0 -- Non; 1 -- Write

unsigned char RegisterAddress; //Register address
unsigned char DLength; //Data length

unsigned char *I2CTX_BUF; //I2C TX data pointer
unsigned char *I2CRX_BUF; //I2C RX data pointer

/******************************************************************************
Name:           short startI2C()
Function:       Initial I2C I/0 and Function
Parameter:      return status
Attention:
******************************************************************************/
short startI2C(void)
{
  if(enableI2C)
     return 0;
  
  I2CPSEL &= ~BIT(SCL); //p1.6 I2C SCL
  I2CPDIR |= BIT(SCL);
  I2CPOUT |= BIT(SCL);
  // Output 9 clock to recover I2C Status
  for(int i = 0;i < 9;i++)
  {
    I2CPOUT |= BIT(SCL);
    __delay_cycles(10);
    I2CPOUT &= ~BIT(SCL);
    __delay_cycles(10);
  }
  
  I2CPSEL |= BIT(SCL) + BIT(SDL); //Function I/0
  I2CPSEL2 |= BIT(SCL) + BIT(SDL);
  
  /* P1OUT |= BIT6 + BIT7; //Enable internal pull up register
  P1REN |= BIT6 + BIT7;*/
  /*MSP430 IO is too weak to drive I2C*/
  
  /*Set I2C Register*/
  UCB0CTL1 |= UCSWRST;  //Reset USCI
  UCB0CTL0 = UCMST + UCSYNC + UCMODE_3; //I2C Master, Synchronus
  UCB0CTL1 |= UCSWRST + UCSSEL_2;       //Keep reset, Select SMCLK as clock
  UCB0BR0 = 40;   //fCLK = SMCLK(12MHz)/40 = 300KHz
  UCB0BR1 = 0;
  UCB0I2CSA = I2CSlaveAddress; //Slave Device Address(might be 0x69)
  UCB0CTL1 &= ~UCSWRST; //Clear Reset Flag

  enableI2C = 1;        //Set Flag
  
  return 0;
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/******************************************************************************
Name:           void writeI2Cmode(void)
Function:       Set Controller into I2C write mode
Parameter:      none
Attention:      none
******************************************************************************/
void writeI2Cmode(void)
{
  UCB0CTL1 &= ~UCSWRST; //Clear Reset Flag
  UCB0CTL1 |= UCTR; //Set as Transmitter
  IFG2 &= ~UCB0TXIFG; //Clear IFG2 - TXIFG
  IE2 &= ~UCB0RXIE; //Clear I2C RXIE 
  IE2 |= UCB0TXIE; //Set I2C TXIE
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/******************************************************************************
Name:           void readI2Cmode(void)
Function:       Set Controller into I2C read mode
Parameter:      none
Attention:      none
******************************************************************************/
void readI2Cmode(void)
{
  UCB0CTL1 &= ~UCSWRST; //Clear Reset Flag
  UCB0CTL1 &= ~UCTR; //Set as Receiver
  IFG2 &= ~UCB0RXIFG; //Clear IFG2 - TXIFG
  IE2 &= ~UCB0TXIE; //Clear I2C TXIE 
  IE2 |= UCB0RXIE; //Set I2C RXIE
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/******************************************************************************
Name:           void closeI2C(void)
Function:       Close I2C function
Parameter:      none
Attention:      none
******************************************************************************/
void closeI2C(void)
{
  UCB0CTL1 |= UCSWRST;  //Reset USCI
  IFG2 &= (~UCB0RXIFG & ~UCB0TXIFG); //Clear IFG2 - TXIFG, RXIFG
  IE2 &= (~UCB0TXIE & ~UCB0RXIE); //Clear I2C
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/******************************************************************************
Name:           void writeI2C(uchar,uchar,uchar *,uchar)
Function:       Write LByte byte data to Device Daddr, register Raddr+
Parameter:      unsigned char Daddr -- Device address
                unsigned char Raddr -- Register address 
                unsigned char *p_buf -- TX array address
                unsigned char LByte -- Data length 
Attention:      none
******************************************************************************/
void writeI2C(unsigned char Daddr,unsigned char Raddr,unsigned char *p_buf,unsigned char LByte)
{ 
  /*Initial I2C, transfer parameter, prepare to send*/
  while(UCB0STAT & UCBBUSY); //Wait untill bus idel

  UCB0I2CSA = Daddr; //Set slave device address
  RegisterAddress = Raddr; //Load Register address
  writeReg = UNWRITE_REGISTER;
  DLength = LByte; //Load data length  
  I2CSTAT = WRITE_I2C; //Set I2C status
  I2CTX_BUF = p_buf; //Load data
  
  writeI2Cmode(); //Set into write mode

  UCB0CTL1 |= UCTXSTT; //Generate START condition
  //while(UCB0CTL1 & UCTXSTT); //Wait for Slave address ACK, STT clear
  /*Don't know why, last sentence doesn't work*/
  
  /*Send the Data in Interrupt sevice function*/ 
  _EINT(); 
  LPM0; 
  
  /*Send complete, Send STOP condition*/
  while (UCB0CTL1 & UCTXSTP);
  /*Might have problem*/
  closeI2C();
}


/******************************************************************************
Name:           unsigned char readI2C(uchar Daddr,uchar Raddr,uchar LByte)
Function:       Read LByte Byte data from Device Daddr, register Raddr
Parameter:      unsigned char Daddr -- Device address
                unsigned char Raddr -- Register address
                unsigned char *buf -- Receive data
                unsigned char LByte -- Length of data                              
Attention:      none
******************************************************************************/
void readI2C(unsigned char Daddr,unsigned char Raddr,unsigned char *buf,unsigned char LByte)
{  
  /*Initial I2C, transfer parameter, prepare to send and read*/

  /*Initial I2C, transfer parameter, prepare to send*/
  while(UCB0STAT & UCBBUSY); //Wait untill bus idel

  UCB0I2CSA = Daddr; //Set slave device address
  RegisterAddress = Raddr; //Load Register address
  writeReg = UNWRITE_REGISTER;
  DLength = LByte; //Load data length  
  I2CSTAT = READ_I2C; //Set I2C status
  I2CRX_BUF = buf; //Load data
  
  writeI2Cmode(); //Set into write mode
  
  UCB0CTL1 |= UCTXSTT; //Generate START condition
  
  /*Enable interrupt and entry LPM0, complete send in interrupt service*/
  _EINT();
  LPM0;
  
  while (UCB0CTL1 & UCTXSTP);
  closeI2C();
}


/******************************************************************************
Name:           void oneByteReadI2C(unsigned char,unsigned char,unsigned char*)
Function:       Read LByte Byte data from Device Daddr, register Raddr
Parameter:      unsigned char Daddr -- Device address
                unsigned char Raddr -- Register address
                unsigned char *buf -- Receive data                             
Attention:      none
******************************************************************************/
void oneByteReadI2C(unsigned char Daddr,unsigned char Raddr,unsigned char *buf)
{  
  /*Initial I2C, transfer parameter, prepare to send and read*/

  /*Initial I2C, transfer parameter, prepare to send*/
  while(UCB0STAT & UCBBUSY); //Wait untill bus idel

  UCB0I2CSA = Daddr; //Set slave device address
  RegisterAddress = Raddr; //Load Register address
  writeReg = UNWRITE_REGISTER;
  DLength = 1; //Load data length  
  I2CSTAT = READ_I2C; //Set I2C status
  I2CRX_BUF = buf; //Load data
  
  writeI2Cmode(); //Set into write mode
  
  UCB0CTL1 |= UCTXSTT; //Generate START condition
  
  /*Enable interrupt and entry LPM0, complete send in interrupt service*/
  _EINT();
  LPM0;
  
  while (UCB0CTL1 & UCTXSTP);
  closeI2C();
}


/******************************************************************************
Name:           void oneByteWriteI2C(uchar,uchar,uchar)
Function:       Write one byte data to Device Daddr, register Raddr+
Parameter:      unsigned char Daddr -- Device address
                unsigned char Raddr -- Register address 
                unsigned char data -- data send to register
Attention:      none
******************************************************************************/
void oneByteWriteI2C(unsigned char Daddr, unsigned char Raddr, unsigned char data)
{
  /*Initial I2C, transfer parameter, prepare to send*/
  while(UCB0STAT & UCBBUSY); //Wait untill bus idel

  UCB0I2CSA = Daddr; //Set slave device address
  RegisterAddress = Raddr; //Load Register address
  writeReg = UNWRITE_REGISTER;
  DLength = 1; //Load data length  
  I2CSTAT = WRITE_I2C; //Set I2C status
  I2CTX_BUF = &data; //Load data
  
  writeI2Cmode(); //Set into write mode

  UCB0CTL1 |= UCTXSTT; //Generate START condition
  //while(UCB0CTL1 & UCTXSTT); //Wait for Slave address ACK, STT clear
  /*Don't know why, last sentence doesn't work*/
  
  /*Send the Data in Interrupt sevice function*/ 
  _EINT(); 
  LPM0; 
  
  /*Send complete, Send STOP condition*/
  while (UCB0CTL1 & UCTXSTP);
  /*Might have problem*/
  closeI2C();
}


/******************************************************************************
Name:           __interuupt void I2C_ISR(void)
Function:       I2C interrupt service function
Parameter:      none                              
Attention:      none
******************************************************************************/
#pragma vector = USCIAB0TX_VECTOR
__interrupt void UCB0TXINT(void)
{
  
  /*TX Interrupt Service*/
  if(IFG2 & UCB0TXIFG)
  {
    IFG2 &= ~UCB0TXIFG; //Clear TXIFG
    
    /*Different function according to Service needed*/
    switch(I2CSTAT)
    {
      
      /*Write data to device*/    
    case WRITE_I2C:
      
      /*1. Send register address;
       *2. Send next data;
       *3. Send STOP condition.
      */
      if(!writeReg) //1.
      {
        writeReg = WRITE_REGISTER;
        UCB0TXBUF = RegisterAddress;      
      }
      else if(DLength) //2.
      {
        unsigned char next = *I2CTX_BUF;
        DLength--;
        I2CTX_BUF++;
        UCB0TXBUF = next; //Last write data to UCB0TXBUF
      }
      else //3.
      {
        UCB0CTL1 |= UCTXSTP; //Send STOP condition
        LPM0_EXIT; //Exit LPM0, CPU on
      }
      break;
     
      
      /*Read data from device*/
    case READ_I2C: 
      
      /*1. Send register address
       *2. Repeat Start, change to RX mode (if single byte, STOP immediately)
            
      */
      if(!writeReg) //1.
      {
        writeReg = WRITE_REGISTER;
        UCB0TXBUF = RegisterAddress;
      }
      else //2.
      {
        readI2Cmode();
        UCB0CTL1 |= UCTXSTT;
        
        if(DLength == 1) //One byte left, prepare STOP
        {
          while(UCB0CTL1 & UCTXSTT); //Wait for START condition been sent.
          UCB0CTL1 |= UCTXSTP; //Send STOP condition
        }
      }
      break;
            
    default:
     break; 
    } 
  }
  
  
  /*RX Interrupt Service*/
  else if(IFG2 & UCB0RXIFG)
  {
    IFG2 &= ~UCB0RXIFG; //Clear RX IFG
    
    DLength--;
    
    if(DLength)
    {
      if(DLength == 1) //One byte left, prepare STOP and NACK
        UCB0CTL1 |= UCTXSTP + UCTXNACK;
    }
    else //Non byte left, Exit LPM0
      LPM0_EXIT;
    
    *I2CRX_BUF++ = UCB0RXBUF; //Read buffer at last
  }
  
  else
    return;
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
