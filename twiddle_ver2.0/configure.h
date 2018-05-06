#include "Arduino.h"
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//****************  convert   **************
#define deg2pi    PI/180
#define pi2deg    180/PI
//******************************************

//************  HC11  **********************
#define CA1       2       //Serial1
#define CB1       3
#define PWM1      4       //Serial2
#define DIR1      5

#define CA2       18       //Serial3
#define CB2       19
#define PWM2      6
#define DIR2      7

#define CA3       20
#define CB3       21
#define PWM3      8
#define DIR3      9  

//*****************************************
//#define OUT1     10
//#define OUT2     11
//#define OUT3     12
//*****************************************

void init_config()
{
  //  config for ISP
  pinMode(CA1, INPUT_PULLUP);
  pinMode(CB1, INPUT_PULLUP);
  pinMode(CA2, INPUT_PULLUP);
  pinMode(CB2, INPUT_PULLUP);
  pinMode(CA3, INPUT_PULLUP);
  pinMode(CB3, INPUT_PULLUP);
  
//  pinMode(PWM1, OUTPUT);
//  pinMode(PWM2, OUTPUT);
//  pinMode(PWM3, OUTPUT);
//  pinMode(DIR1, OUTPUT);
//  pinMode(DIR2, OUTPUT);
//  pinMode(DIR3, OUTPUT);
  
  }
#endif

