/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: msp430_i2c.h$
******************************************************************************/

#ifndef MSP430_I2C_H_
#define MSP430_I2C_H_

#ifndef I2CSlaveAddress
#define I2CSlaveAddress 0x68
#endif

/*Port Declaratoin*/
#ifndef _BIT
#define BIT(x)          (1<<(x))
#endif

#define I2CPSEL         P1SEL
#define I2CPSEL2        P1SEL2
#define I2CPDIR         P1DIR
#define I2CPOUT         P1OUT
#define SCL             6
#define SDL             7


/*Parameter Declaration*/
extern short enableI2C;
extern short writeReg;
extern short I2CSTAT;

extern unsigned char RegisterAddress;
extern unsigned char DLength;

extern unsigned char *I2CTX_BUF;
extern unsigned char *I2CRX_BUF;


/*Function Declaration*/
short startI2C(void);
void closeI2C(void);
void writeI2C(unsigned char Daddr,unsigned char Raddr,unsigned char *p_buf,unsigned char LByte);
void readI2C(unsigned char Daddr,unsigned char Raddr,unsigned char *buf,unsigned char LByte);
void oneByteWriteI2C(unsigned char Daddr, unsigned char Raddr, unsigned char data);
void oneByteReadI2C(unsigned char Daddr,unsigned char Raddr,unsigned char *buf);
//void UCB0TXINT(void);

#endif
