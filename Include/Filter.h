/***************************************************************
	filter.h
	generic 1st- and 2nd-order filter

	programmed by Yo-chan Son, Oct 2000.
	copyright (c) 1991, 2000 by EEPEL, SNU, SEOUL, KOREA
	All Rights Reserved.

	modified by T.S.Kwon, 2008

****************************************************************/

#ifndef FILTER_H
#define FILTER_H

#define K_ALLPASS	0
#define K_LPF		1
#define K_HPF		2
#define K_BPF		3
#define K_NOTCH		4
#define PREWARP_BPF	5
#define PREWARP_LPF	6
#define PREWARP_NOTCH	7

#define IIR1DEFINE(type, w0, Ts)		{(int)(type), (float)(w0), (float)(Ts), 0, 0, 0, 0}
#define IIR2DEFINE(type, w0, zeta,Ts)	{(int)(type), (float)(w0), (float)(zeta), (float)(Ts), 0, 0, 0, 0, 0, 0, 0}

typedef struct	{
	int type;
	float w0;
	float delT;
	float coeff[3], reg;
}	IIR1;

typedef struct	{
	int type;
	float w0, zeta;
	float delT;
	float coeff[5], reg[2];
}	IIR2;

void IIR1CoeffInit(IIR1 *p_gIIR, float w0);
void IIR1Init(IIR1 *p_gIIR, float w0);
float IIR1Update(IIR1 *p_gIIR, const float input);

void IIR2CoeffInit(IIR2 *p_gIIR, float w0, float zeta );
void IIR2Init(IIR2 *p_gIIR, float w0, float zeta );
float IIR2Update(IIR2 *p_gIIR, const float input);

extern IIR2 Filter_Vdc;
extern IIR1 Filter_Align;
extern IIR1 Filter_ACC;
extern IIR2 Filter_ACC2;
extern IIR2 Filter_P_emul;
extern IIR1 Filter_Idss_HPF;
extern IIR1 Filter_Iqss_HPF;
//extern IIR2	Filter_Enc;
extern IIR2	Filter_Sup_Vdc;
extern IIR1	Filter_SinData;
extern IIR1	Filter_CosData;
extern IIR1	Filter_Enc;

//for High frequency injection
extern IIR2	Filter_Idm;
extern IIR2	Filter_Iqm;
extern IIR2	Filter_Idqmcos;
extern IIR2	Filter_Idqmsin;
extern IIR2	Filter_Idqmcos2;
extern IIR2	Filter_Idqmsin2;
extern IIR2	Filter_Idqse;

extern IIR2 Filter_Ias;
extern IIR2 Filter_Ibs;
extern IIR2 Filter_Ics;

extern IIR2 Filter_Wrm;
extern IIR2 Filter_Obs_Iq_ref;

extern IIR2 Filter_Active_Power;
extern IIR2 Filter_Reactive_Power;

extern IIR2 Filter_Hiedenhain_A;
extern IIR2 Filter_Hiedenhain_B;
extern IIR2 Filter_Hiedenhain_Spd_A;
extern IIR2 Filter_Hiedenhain_Spd_B;


//extern IIR2	Filter_SinData;
//extern IIR2	Filter_CosData;

//extern IIR1 All_pass_6d;
//extern IIR1 All_pass_6q;
//extern IIR1 All_pass_12d;
//extern IIR1 All_pass_12q;
//extern IIR1 Filter_Vdc;

#endif /* FILTER_H */

