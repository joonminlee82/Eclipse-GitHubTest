/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#ifndef _MECH_PARA_ID_H_
#define _MECH_PARA_ID_H_

#include "comp.h"
#include "Variable.h"

typedef struct	{
	float time;
	float Tsamp;

	float T_ID, f_ID;
	float N_ID;
	float N_cnt;
	float Arpm_ID;
	float Wrpm_ref;
	float Wrm_ref;

	float Te, Wrm;
	float Wrm2;
	float TeWrm_Integ;
	float Wrm2_Integ;
	float Te_Integ;

	float ITE_Wrm_Integ;

	float J_eq, B_eq;
	float J_eq_samp, B_eq_samp;

	Uint16 Flag_Mech_ID;
}Mech_para_ID;
extern Mech_para_ID HY_Motor_JB_ID;
void Init_MechID(Mech_para_ID *M_ID);
void Mech_ID_Cal(Mech_para_ID *M_ID, float Wrm, float Te);


#endif	//__MECH_PARA_ID_H__
