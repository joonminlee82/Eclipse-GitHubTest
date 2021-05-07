/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#include "SNU_speed_profile.h"
#include "comp.h"

Profile_str Elevator_Profile={.Djerk_base={1.,0.,-1.,0.,-1.,0.,1.,0.,-1.,0.,1.,0.,1.,0.,-1.},
							  .jerk_base ={1.,0.,-1.,0.,-1.,0.,1.}
							  };
void Make_SpdProfile(Profile_str *prof, float H){

	prof->H = H;
	//Third ptn time calculation
	//jerk_time_mid = 0.8;
	prof->jerk_time_total = 1.8;
	prof->Acc_time_mid = (prof->V_max / prof->Acc_max) - prof->jerk_time_total;
	prof->Acc_time_total = 2.*prof->jerk_time_total + prof->Acc_time_mid;
	//minimum Distance
	prof->Third_Hmin =  prof->Acc_time_total*prof->V_max;

	if(H >prof->Third_Hmin){
		prof->Flag_Third = 1;
		prof->Flag_Second = 0;

		prof->jerk_time_mid = 0.8;
		prof->jerk_time_total = 1.8;
		prof->Acc_time_mid = prof->V_max/prof->Acc_max-prof->jerk_time_total;
		prof->Acc_time_total = 2.*prof->jerk_time_total + prof->Acc_time_mid;
		prof->V_time_mid = prof->H/prof->V_max-prof->Acc_time_total;
		prof->V_time_total = 2.*prof->Acc_time_total+ prof->V_time_mid;

		// Max jerk calculation
		prof->jerk_max = prof->Acc_max/(prof->jerk_time_total/2.+prof->jerk_time_mid/2.);
		prof->Djerk_max = prof->jerk_max/(prof->jerk_time_total/2.-prof->jerk_time_mid/2.);

		// Djerk Generation
		Uint16 i;
		for(i =0; i<15; i++){
			prof->Djerk[i] = prof->Djerk_base[i]*prof->Djerk_max;
		}

		prof->Djerk_Time[0] = (prof->jerk_time_total - prof->jerk_time_mid)/2.;
		prof->Djerk_Time[1] = prof->Djerk_Time[0]+ prof->jerk_time_mid;
		prof->Djerk_Time[2] = prof->jerk_time_total;

		prof->Djerk_Time[3] = prof->Djerk_Time[2] + prof->Acc_time_mid;

		prof->Djerk_Time[4] = prof->Djerk_Time[3]+prof->Djerk_Time[0];
		prof->Djerk_Time[5] = prof->Djerk_Time[3]+prof->Djerk_Time[1];
		prof->Djerk_Time[6] = prof->Djerk_Time[3]+prof->Djerk_Time[2];

		prof->Djerk_Time[7] = prof->Djerk_Time[6] + prof->V_time_mid;

		prof->Djerk_Time[8]  = prof->Djerk_Time[7]+prof->Djerk_Time[0];
		prof->Djerk_Time[9]  = prof->Djerk_Time[7]+prof->Djerk_Time[1];
		prof->Djerk_Time[10] = prof->Djerk_Time[7]+prof->Djerk_Time[2];
		prof->Djerk_Time[11] = prof->Djerk_Time[7]+prof->Djerk_Time[3];
		prof->Djerk_Time[12] = prof->Djerk_Time[7]+prof->Djerk_Time[4];
		prof->Djerk_Time[13] = prof->Djerk_Time[7]+prof->Djerk_Time[5];
		prof->Djerk_Time[14] = prof->Djerk_Time[7]+prof->Djerk_Time[6];

		prof->Time_Max = prof->Djerk_Time[14];
	}
	else{
	    //Second_spd_profile();//25m
		prof->Flag_Third = 0;
		prof->Flag_Second = 1;

		prof->jerk_time_total = prof->T_init; //1.s
		prof->Acc_time_mid = prof->V_max/prof->Acc_max-prof->jerk_time_total;
		prof->Acc_time_total = 2.*prof->jerk_time_total + prof->Acc_time_mid;
		//minimum Distance
		prof->Second_Hmin =  prof->Acc_time_total*prof->V_max;

		//clear jerk_time_total Acc_time_mid Acc_time_total;
		if (H > prof->Second_Hmin){
			// time calculation
			prof->jerk_time_total = prof->T_init;
			prof->Acc_time_mid = prof->V_max/prof->Acc_max-prof->jerk_time_total;
			prof->Acc_time_total = 2.*prof->jerk_time_total + prof->Acc_time_mid;
			prof->V_time_mid = H/prof->V_max - prof->Acc_time_total;
			prof->V_time_total = 2.*prof->Acc_time_total+ prof->V_time_mid;

			// Max jerk calculation
			prof->jerk_max = prof->Acc_max/prof->jerk_time_total;

			// jerk Generation
			Uint16 i;
			for(i =0; i<15; i++){
				prof->jerk[i] = prof->jerk_base[i]*prof->jerk_max;
			}

			prof->jerk_Time[0] = prof->jerk_time_total;
			prof->jerk_Time[1] = prof->jerk_Time[0]+ prof->Acc_time_mid;
			prof->jerk_Time[2] = prof->Acc_time_total;

			prof->jerk_Time[3] = prof->jerk_Time[2] + prof->V_time_mid;

			prof->jerk_Time[4] = prof->jerk_Time[3]+prof->jerk_Time[0];
			prof->jerk_Time[5] = prof->jerk_Time[3]+prof->jerk_Time[1];
			prof->jerk_Time[6] = prof->jerk_Time[3]+prof->jerk_Time[2];

			prof->Time_Max = prof->jerk_Time[6];
		}
		else if (H >1.6){
			// time calculation
			prof->Acc_time_total = 0.5+__sqrt(1+4.*H/prof->Acc_max)/2.;	//roots([1 -1 -H/prof->Acc_max])
			prof->jerk_time_total = prof->T_init;
			prof->Acc_time_mid = prof->Acc_time_total - 2.*prof->jerk_time_total;
			prof->V_time_total = 2.*prof->Acc_time_total;

			// Max jerk calculation
			prof->jerk_max = prof->Acc_max/prof->jerk_time_total;
			// max V calculation
			prof->V_max_lower = H/(prof->V_time_total/2.);


			prof->V_time_mid =0.;
			// jerk Generation
			Uint16 i;
			for(i =0; i<15; i++){
				prof->jerk[i] = prof->jerk_base[i]*prof->jerk_max;
			}

			prof->jerk_Time[0] = prof->jerk_time_total;
			prof->jerk_Time[1] = prof->jerk_Time[0]+ prof->Acc_time_mid;
			prof->jerk_Time[2] = prof->Acc_time_total;

			prof->jerk_Time[3] = prof->jerk_Time[2] + prof->V_time_mid;

			prof->jerk_Time[4] = prof->jerk_Time[3]+prof->jerk_Time[0];
			prof->jerk_Time[5] = prof->jerk_Time[3]+prof->jerk_Time[1];
			prof->jerk_Time[6] = prof->jerk_Time[3]+prof->jerk_Time[2];

			prof->Time_Max = prof->jerk_Time[6];
		}
		else{
			// error('Impassable');
		}

	}

}

float jerk_generation(Profile_str *prof, float time){
	Uint16 i;

	for(i=prof->Jerk_index; i<7; i++){
		if(time < prof->jerk_Time[i]){
			prof->Jerk_index = i;
			return prof->jerk[i];
		}
		else{;}
	}
	return 0.;
}

float Djerk_generation(Profile_str *prof, float time){
	Uint16 i;

	for(i=prof->DJerk_index; i<15; i++){
		if(time < prof->Djerk_Time[i]){
			prof->DJerk_index = i;
			return prof->Djerk[i];
		}
		else{;}
	}
	return 0.;
}

float eff_motor = 0.89;
float eff_conv =0.95;
void Profile_generation(Profile_str *prof, float time, float Ts){
	if(prof->Flag_Third){
		if(time > prof->Time_Max)	prof->Jerk =0.;
		else		     			prof->Jerk += prof->Direction*Ts*Djerk_generation(prof,time);
	}
	else if(prof->Flag_Second){
		prof->Jerk = prof->Direction*jerk_generation(prof,time);
	}

	if(time > prof->Time_Max){
		prof->Acc =0.;
		prof->V =0.;
	}
	else{
		prof->Acc += Ts*prof->Jerk;
		prof->V   += Ts*prof->Acc;
	}
	prof->P   += Ts*prof->V;

	prof->Acc_rad =prof->Acc*prof->Trans_rad;
	prof->V_rad   =prof->V*prof->Trans_rad;
	prof->P_rad   =prof->P*prof->Trans_rad;

	prof->V_rpm  =prof->V_rad *Rm2Rpm;

	prof->P_emul = ((prof->J_emul*prof->Acc_rad + prof->T_load)*prof->V_rad);

	if(prof->P_emul >= 0.) prof->P_emul = prof->P_emul/(0.8*eff_motor*eff_conv);
	else				   prof->P_emul = prof->P_emul*(0.8*eff_motor*eff_conv);

}

void ResetProfile(Profile_str *prof){
	prof->time =0.;
	//prof->Flag_Third=0;
	//prof->Flag_Second=0;
	prof->DJerk_index =0;
	prof->Jerk_index =0;
	prof->Acc_time_total =0.;
	//prof->Time_Max =0.;
	prof->Jerk =0.;
	prof->Acc =0.;
	prof->V   =0.;
	prof->P   =0.;

	prof->Acc_rad =0.;
	prof->V_rad   =0.;
	prof->P_rad   =0.;

	prof->V_rpm =0.;
}



void InitProfile(Profile_str *prof){

	prof->V_max 			= 4.; //m/s
	prof->Acc_max 			= 0.8;//m/s2
	prof->T_init 			= 1.; //s for 2nd profile
	prof->H 				= 0.; //hight
	prof->Third_Hmin		= 0.;
	prof->Second_Hmin		= 0.;

	prof->jerk_time_mid		= 0.;
	prof->jerk_time_total	= 0.;
	prof->Acc_time_mid		= 0.;
	prof->Acc_time_total	= 0.;
	prof->V_time_mid		= 0.;
	prof->V_time_total		= 0.;

	prof->V_max_lower		= 0.;
	prof->jerk_max			= 0.;
	prof->Djerk_max			= 0.;

//	prof->Djerk_Time[15];
//	prof->DJerk_index;

//	prof->jerk[7];
//	prof->jerk_Time[7];
	prof->Jerk_index	= 0;

	prof->Jerk			= 0.;
	prof->Acc			= 0.;
	prof->V				= 0.;
	prof->P				= 0.;
	prof->Direction 	= 1.;
	prof->Trans_rad 	= 2./(0.52/2.);
	prof->time			= 0.;

	prof->Flag_Third	= 0;
	prof->Flag_Second	= 0;
	prof->Flag_Done		= 1;


//for JB_emulation and power calculation
	prof->r_sheave 		= 0.26;
	prof->load_ratio 	= 0.5;
	prof->mass_total 	= 3203+1600*0.5+4031+2880.8;
	prof->mech_eff 		= 0.8;
	prof->J_emul 		= 230.5752;
	prof->B_emul 		= 0.;
	prof->T_load 		= 0.;
	prof->F_load 		= 0.;
	prof->P_emul 		= 0.;
}
