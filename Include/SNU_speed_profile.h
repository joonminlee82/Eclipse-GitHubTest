/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#ifndef SPEED_PROFILE_H
#define SPEED_PROFILE_H

#include "CPU1_CLA_Shared.h"

typedef struct 	_PROFILE_STR_
{
	float V_max;
	float Acc_max;
	float T_init;
	float H;
	float Third_Hmin;
	float Second_Hmin;
	float Time_Max;

	float jerk_time_mid;
	float jerk_time_total;
	float Acc_time_mid;
	float Acc_time_total;
	float V_time_mid;
	float V_time_total;

	float V_max_lower;
	float jerk_max;
	float Djerk_max;

	const float Djerk_base[15];
	float Djerk[15];
	float Djerk_Time[15];
	Uint16 DJerk_index;

	const float jerk_base[7];
	float jerk[7];
	float jerk_Time[7];
	Uint16 Jerk_index;

	float Jerk;
	float Acc;
	float V;
	float P;
	float Direction;
	float Trans_rad;
	float Acc_rad;
	float V_rad;
	float P_rad;
	float V_rpm;

	float time;

	Uint16 Flag_Third;
	Uint16 Flag_Second;
	Uint16 Flag_Done;

	float r_sheave;
	float load_ratio;
	float mass_total;
	float mech_eff;
	float J_emul;
	float B_emul;
	float T_load;
	float F_load;
	float P_emul;

} Profile_str;
extern Profile_str Elevator_Profile;

extern void Make_SpdProfile(Profile_str *prof, float H);
extern float jerk_generation(Profile_str *prof, float time);
extern float Djerk_generation(Profile_str *prof, float time);
extern void Profile_generation(Profile_str *prof, float time, float Ts);
extern void InitProfile(Profile_str *prof);
extern void ResetProfile(Profile_str *prof);

#endif  // end of SPEED_PROFILE_H definition
