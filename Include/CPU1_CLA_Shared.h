/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#ifndef _F2837xD_CLA_DEFINES_H
#define _F2837xD_CLA_DEFINES_H

// Project includes

#include "F28x_Project.h"
#include "F2837xD_Cla_defines.h"
#include "CC.h"
//#include "FC.h"

extern __interrupt void cla_ISR1(void);
extern __interrupt void cla_ISR2(void);
extern __interrupt void cla_ISR3(void);
extern __interrupt void cla_ISR4(void);
extern __interrupt void cla_ISR5(void);
extern __interrupt void cla_ISR6(void);
extern __interrupt void cla_ISR7(void);
extern __interrupt void cla_Init(void);

void CLA_configClaMemory(void);
void CLA_initCpu1Cla1(void);
void InitCla(void);

/* CLA to CPU variable */

extern Uint16 cla1_cnt,cla2_cnt;
extern Uint16 cla_Init_cnt;
extern float Offset_ADC_INV1_Ibs, Offset_ADC_INV1_Ics, Offset_ADC_INV1_Ias,Offset_ADC_ESS_Iqe;

/* CPU to CLA variable */

/* CLA Data */
//extern CC_3PH INV3_CC;
extern Uint16 Flag_SinInit;
//extern FLOOR Floor ;
extern float da_cla[8], da_scale[8], da_mid[8];
extern long da_value[8];
#endif
