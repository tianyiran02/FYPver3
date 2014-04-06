/*
$License:
  Copy right    Yiran Tian, All Rights Reserved.
$
*/    

/******************************************************************************
*$Id: FYPver3.h$
******************************************************************************/

#ifndef _FYPVER1_H_
#define _FYPVER1_H_

/* Global Macros */
#define PI              3.14159265358979
#define SYS_CLOCK       12000000

#define ReCalibrateTime 250     // Press button for 3s then recalibrete

#define uchar   unsigned char
#define uint    unsigned int

#ifndef _ON_OFF_STATUS_
#define _ON_OFF_STATUS_
#define ON      1
#define OFF     0
#endif

#ifndef _TYPE_UINT_
#define _TYPE_UINT_
typedef unsigned char   uint8;          
typedef unsigned int    uint16;           
#endif

#endif
