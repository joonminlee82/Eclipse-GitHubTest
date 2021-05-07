/***************************************************************
	FILTER.c
	generic 1st- and 2nd-order filter
	
	1st-order filter
	void IIR1Init(IIR1 *p_gIIR, const float sample_time);
	void IIR1CoeffInit(IIR1 *p_gIIR, const float sample_time); 
	float IIR1Update(IIR1 *p_gIIR, const float input);

	2nd-order filter
	void IIR2Init(IIR2 *p_gIIR, const float sample_time);
	void IIR2CoeffInit(IIR2 *p_gIIR, const float sample_time); 
	float IIR2Update(IIR2 *p_gIIR, const float input);

	programmed by Yo-chan Son, Oct 2000.
	copyright (c) 1991, 2000 by EEPEL, SNU, SEOUL, KOREA
	All Rights Reserved.
	modified by T.S.Kwon, 2008
	1st-order filter : 
	void IIR1CoeffInit(IIR1 *p_gIIR, float w0 );
	void IIR1Init(IIR1 *p_gIIR, float w0 );
	float IIR1Update(IIR1 *p_gIIR, const float input );

	2nd-order filter : 
	void IIR2CoeffInit(IIR1 *p_gIIR, float w0, float zeta );
	void IIR2Init(IIR2 *p_gIIR, float w0, float zeta );
	float IIR2Update(IIR2 *p_gIIR, const float input);
****************************************************************/
#include "Filter.h"
#include "Variable.h"

//#pragma CODE_SECTION(IIR1Update, "ramfuncs");
//#pragma CODE_SECTION(IIR2Update, "ramfuncs");

IIR2 Filter_Vdc = IIR2DEFINE(PREWARP_NOTCH, TWOPI*60., 0.1, 1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR1 Filter_Align  = IIR1DEFINE(K_LPF, 2.*PI*5., 1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR1 Filter_ACC  = IIR1DEFINE(K_LPF, 2.*PI*5., 1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR1 Filter_SinData  = IIR1DEFINE(K_LPF, 2.*PI*300., 1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR1 Filter_CosData  = IIR1DEFINE(K_LPF, 2.*PI*300., 1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR1 Filter_Enc  = IIR1DEFINE(K_LPF, 2.*PI*100., 1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
//IIR2 Filter_SinData  = IIR2DEFINE(K_LPF, 2.*PI*300., 0.707, 1./(SWITCHING_FRQ*SAMPING_METHOD));
//IIR2 Filter_CosData  = IIR2DEFINE(K_LPF, 2.*PI*300., 0.707, 1./(SWITCHING_FRQ*SAMPING_METHOD));
IIR2 Filter_ACC2 = IIR2DEFINE(K_LPF, 2*PI*5, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));

IIR2 Filter_P_emul = IIR2DEFINE(K_LPF, 2*PI*2.5, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
//IIR2 Filter_Enc = IIR2DEFINE(K_LPF, 2*PI*100, 0.707,  1./(SWITCHING_FRQ*SAMPING_METHOD));
IIR2 Filter_Sup_Vdc = IIR2DEFINE(K_LPF, 2*PI*100, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
//IIR1 All_pass_6d = IIR1DEFINE(K_ALLPASS, TWOPI*60.*6., 1./(SWITCHING_FRQ*SAMPING_METHOD));
//IIR1 All_pass_6q = IIR1DEFINE(K_ALLPASS, TWOPI*60.*6., 1./(SWITCHING_FRQ*SAMPING_METHOD));
//IIR1 All_pass_12d = IIR1DEFINE(K_ALLPASS, TWOPI*60.*12., 1./(SWITCHING_FRQ*SAMPING_METHOD));
//IIR1 All_pass_12q = IIR1DEFINE(K_ALLPASS, TWOPI*60.*12., 1./(SWITCHING_FRQ*SAMPING_METHOD));

//IIR1 Filter_Vdc = IIR1DEFINE(K_LPF, TWOPI*10., 1./(SWITCHING_FRQ*SAMPING_METHOD));

//IIR1 Filter_Vdcf= IIR1DEFINE(K_LPF, 2.*PI*10., SAMPLE_TIME);
//IIR1 Filter_Wh  = IIR1DEFINE(K_LPF, 2.*PI*5., SAMPLE_TIME);
//IIR1 Filter_Pfd = IIR1DEFINE(K_LPF, 2.*PI*5., SAMPLE_TIME);

//IIR2 Filter_Lamdss = IIR2DEFINE(K_LPF, 2*PI*200, 0.707, SAMPLE_TIME);
//IIR2 Filter_Lamqss = IIR2DEFINE(K_LPF, 2*PI*200, 0.707, SAMPLE_TIME);
//IIR2 Filter_Err = IIR2DEFINE(PREWARP_BPF, TWOPI*500., 0.001, SAMPLE_TIME);
//IIR2 Filter_qErr = IIR2DEFINE(PREWARP_LPF, TWOPI*500., 0.001, SAMPLE_TIME);
/*
IIR2 Filter_Vdreh = IIR2DEFINE(PREWARP_LPF, TWOPI*500., 0.001, SAMPLE_TIME);
IIR2 Filter_Vqreh = IIR2DEFINE(PREWARP_LPF, TWOPI*500., 0.001, SAMPLE_TIME);

IIR1 Filter_Vdrehh_pos = IIR1DEFINE(K_LPF, 2.*PI*5., SAMPLE_TIME);
IIR1 Filter_Vqrehh_pos = IIR1DEFINE(K_LPF, 2.*PI*5., SAMPLE_TIME);
IIR1 Filter_Vdrehh_neg = IIR1DEFINE(K_LPF, 2.*PI*5., SAMPLE_TIME);
IIR1 Filter_Vqrehh_neg = IIR1DEFINE(K_LPF, 2.*PI*5., SAMPLE_TIME);
IIR1 Filter_Vqsh = IIR1DEFINE(K_LPF, 2.*PI*5., SAMPLE_TIME);
IIR2 Filter_Idre = IIR2DEFINE(PREWARP_LPF, TWOPI*500., 0.001, SAMPLE_TIME);
IIR1 Filter_Vdcr = IIR1DEFINE(K_LPF, 2.*PI*5000., SAMPLE_TIME);
IIR1 Filter_Pout = IIR1DEFINE(K_LPF, 2.*PI*5000., SAMPLE_TIME);
*/
IIR2 Filter_Idm = IIR2DEFINE(PREWARP_BPF, 2*PI*500., 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Iqm = IIR2DEFINE(PREWARP_BPF, 2*PI*500., 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Idqmcos = IIR2DEFINE(K_LPF, 1000., 1.,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Idqmsin = IIR2DEFINE(K_LPF, 1000., 1.,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Idqmcos2 = IIR2DEFINE(K_LPF, 80., 1.,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Idqmsin2 = IIR2DEFINE(K_LPF, 80., 1.,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Idqse = IIR2DEFINE(K_LPF, 1500., 1.,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));

IIR2 Filter_Ias = IIR2DEFINE(K_LPF, TWOPI*2500, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Ibs = IIR2DEFINE(K_LPF, TWOPI*2500, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Ics = IIR2DEFINE(K_LPF, TWOPI*2500, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));

IIR2 Filter_Wrm = IIR2DEFINE(K_LPF, TWOPI*200, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Obs_Iq_ref = IIR2DEFINE(K_LPF, TWOPI*100, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));

IIR2 Filter_Active_Power = IIR2DEFINE(K_LPF, TWOPI*100, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Reactive_Power = IIR2DEFINE(K_LPF, TWOPI*100, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));

IIR2 Filter_Hiedenhain_A = IIR2DEFINE(K_LPF, TWOPI*1000, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Hiedenhain_B = IIR2DEFINE(K_LPF, TWOPI*1000, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Hiedenhain_Spd_A = IIR2DEFINE(K_LPF, TWOPI*10, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
IIR2 Filter_Hiedenhain_Spd_B = IIR2DEFINE(K_LPF, TWOPI*10, 0.707,  1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));

void IIR1CoeffInit(IIR1 *p_gIIR, float w0)
{
	float a0, b0, b1;
	float INV_alpha, dt;
	int type;
	
	// Continuous-time Filter Coefficients
	p_gIIR->w0 = w0 ;
	type = p_gIIR->type;
	dt = p_gIIR->delT;

	a0 = w0;
	switch(type)
	{
		case K_LPF:
			b0 = w0;
			b1 = 0;
			break;
		case K_HPF: 
			b0 = 0;
			b1 = (float)1;
			break;
		default:
		case K_ALLPASS: 
			b0 = -w0;
			b1 = (float)1;
	}

	// Discrete-time Filter Coefficients
	INV_alpha = (float)1./((float)2 + dt*a0);
	p_gIIR->coeff[0] = ((float)2*b1 + dt*b0)*INV_alpha;
	p_gIIR->coeff[1] = (-(float)2*b1 + dt*b0)*INV_alpha;
	p_gIIR->coeff[2] = -(-(float)2 + dt*a0)*INV_alpha;
	
	return;
}

void IIR1Init(IIR1 *p_gIIR, float w0 )
{
	// Initialize Filter Coefficients
	IIR1CoeffInit(p_gIIR, w0);

	// Initialize Storage Elements
	p_gIIR->reg = 0;
	
	return;
}
float IIR1Update(IIR1 *p_gIIR, const float x)
{
	float y;

	y = p_gIIR->reg + p_gIIR->coeff[0]*x;
	p_gIIR->reg = p_gIIR->coeff[1]*x + p_gIIR->coeff[2]*y;

	return(y);
}

void IIR2CoeffInit(IIR2 *p_gIIR, float w0, float zeta )	// TF = (b2*s2 + b1*s + b0)/(s2 + a1*s + a0)
{
	float a0, a1, b0, b1, b2;
	float INV_alpha, dt;
	int type;

	// Continuous-time Filter Coefficients
	p_gIIR->w0 = w0;
	p_gIIR->zeta = zeta;
	dt = p_gIIR->delT ;
	type = p_gIIR->type;

	a0 = w0*w0;
	a1 = 2*zeta*w0;
	switch(type)
	{
		case K_LPF:
			b0 = w0*w0;
			b1 = 0;
			b2 = 0;
			break;
		case K_HPF: 
			b0 = 0;
			b1 = 0;
			b2 = (float)1;
			break;
		case K_BPF:
			b0 = 0;
			b1 = (float)2*zeta*w0;
			b2 = 0;
			break;
		case K_NOTCH:
			b0 = w0*w0;
			b1 = 0;
			b2 = (float)1;
			break;
		case PREWARP_LPF:
			a0 = 2./dt*tan(w0*dt*0.5);
			a1 = 2.*zeta*a0;
			b0 = a0*a0;
			b1 = 0;
			b2 = 0;
			a0 = a0*a0;
			break;
		case PREWARP_BPF:
			a0 = 2./dt*tan(w0*dt*0.5);
			a1 = 2.*zeta*a0;
			b0 = 0;
			b1 = 2.*zeta*a0;
			b2 = 0;
			a0 = a0*a0;
			break;
		case PREWARP_NOTCH:
			a0 = 2./dt*tan(w0*dt*0.5);
			a1 = 2.*zeta*a0;
			b0 = a0*a0;
			b1 = 0;
			b2 = 1;
			a0 = a0*a0;
			break;
		case K_ALLPASS:
		default:
			b0 = w0*w0;
			b1 = -(float)2*zeta*w0;
			b2 = (float)1;
	}

	// Discrete-time Filter Coefficients
	INV_alpha = (float)1./((float)4 + (float)2*dt*a1 + dt*dt*a0);
	p_gIIR->coeff[0] = ((float)4*b2 + (float)2*dt*b1 + dt*dt*b0)*INV_alpha;
	p_gIIR->coeff[1] = ((float)2*dt*dt*b0 - (float)8*b2)*INV_alpha;
	p_gIIR->coeff[2] = -((float)2*dt*dt*a0 - (float)8)*INV_alpha;
	p_gIIR->coeff[3] = ((float)4*b2 - (float)2*dt*b1 + dt*dt*b0)*INV_alpha;
	p_gIIR->coeff[4] = -((float)4 - (float)2*dt*a1 + dt*dt*a0)*INV_alpha;
	
	return;
}

void IIR2Init(IIR2 *p_gIIR, float w0, float zeta )
{   
	// Initialize Filter Coefficients
	IIR2CoeffInit(p_gIIR, w0, zeta);
	
	// Initialize Storage Elements
	p_gIIR->reg[0] = 0;
	p_gIIR->reg[1] = 0;
	
	return;
}

float IIR2Update(IIR2 *p_gIIR, const float x)
{
	float y;

	y = p_gIIR->reg[0] + p_gIIR->coeff[0]*x;
	p_gIIR->reg[0] = p_gIIR->reg[1] + p_gIIR->coeff[1]*x + p_gIIR->coeff[2]*y;
	p_gIIR->reg[1] = p_gIIR->coeff[3]*x + p_gIIR->coeff[4]*y;

	return(y);
}
