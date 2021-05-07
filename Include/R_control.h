/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#ifndef _R_CONTROL_H__
#define _R_CONTROL_H__

//R contoller //
extern int Flag_Resonant;
extern int Flag_Resonant_reset;

typedef struct	{
	float input[2];
	float output[2];
	float Ki, Kp;
	float a,b;
	float w0;
	float wcc;
}	R_control;

extern R_control R_6d, R_12d;
extern R_control R_6q, R_12q;

extern float ERR_Idss,ERR_Iqss;

void Init_R_control_var(R_control *CON);
void Updata_R_control_var(R_control *CON,float w0,float wcc);
float Resonant_controller(R_control *CON,float input);


#endif	//__R_CONTROL_H__

