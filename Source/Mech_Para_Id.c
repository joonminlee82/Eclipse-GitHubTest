/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#include "Mech_Para_Id.h"

Mech_para_ID HY_Motor_JB_ID ={.N_ID =10.,
							  .f_ID =TWOPI*2.,
							  .Arpm_ID = 50.,
							  .Flag_Mech_ID =0
							  };

void Init_MechID(Mech_para_ID *M_ID){
	M_ID->time =0.;
	M_ID->Tsamp =Tsamp*10.;

	M_ID->N_cnt =1.;
	//M_ID->N_ID =10.;
	//M_ID->f_ID =1./TWOPI;
	//M_ID->Arpm_ID =100.;
	M_ID->T_ID =1./M_ID->f_ID;

	M_ID->Wrpm_ref =0.;
	M_ID->Wrm_ref =0.;

	M_ID->Te =0.;
	M_ID->Wrm =0.;
	M_ID->Wrm2 =0.;
	M_ID->TeWrm_Integ =0.;
	M_ID->Wrm2_Integ =0.;
	M_ID->Te_Integ =0.;

	//M_ID->Flag_Mech_ID=0;

}


#if 1
void Mech_ID_Cal(Mech_para_ID *M_ID, float Wrm, float Te){
	M_ID->time += M_ID->Tsamp;

	M_ID->Wrm = Wrm;
	M_ID->Te = Te;

	M_ID->Wrpm_ref = M_ID->Arpm_ID*__sin(TWOPI*M_ID->f_ID*M_ID->time);
	M_ID->Wrm_ref  = M_ID->Wrpm_ref*Rm2Rpm;

	M_ID->TeWrm_Integ += M_ID->Wrm*M_ID->Te*M_ID->Tsamp;

	M_ID->Wrm2 = M_ID->Wrm*M_ID->Wrm;
	M_ID->Wrm2_Integ += M_ID->Wrm2*M_ID->Tsamp;

	M_ID->Te_Integ += M_ID->Te*M_ID->Tsamp;

	M_ID->ITE_Wrm_Integ += M_ID->Te_Integ*M_ID->Wrm;

	M_ID->B_eq = M_ID->TeWrm_Integ/M_ID->Wrm2_Integ;
	M_ID->J_eq = M_ID->ITE_Wrm_Integ/M_ID->Wrm2_Integ;


	if(M_ID->time > M_ID->T_ID*M_ID->N_cnt ){
		M_ID->N_cnt++;

		M_ID->B_eq_samp = M_ID->B_eq;
		M_ID->J_eq_samp = M_ID->J_eq;
	}

	if(M_ID->time > (M_ID->N_ID*M_ID->T_ID)){
		M_ID->Flag_Mech_ID = 0;
		M_ID->Wrpm_ref = 0.;
	}

}

/* 사용 예시
 * 		// JB Identification
		if(HY_Motor_JB_ID.Flag_Mech_ID==1 && Flags.Speed_control_1==1){
			HY_Motor_JB_ID.Tsamp =Tsamp*Spd_Con_st_Set;
			Mech_ID_Cal(&HY_Motor_JB_ID, INV1_SC.Wrm, INV1.Iqse*INV1.Kt);
			INV1_SC.Wrpm_ref = HY_Motor_JB_ID.Wrpm_ref;
		}
		else{
			Init_MechID(&HY_Motor_JB_ID);
		}
		//End of JB Identification
*/
#endif
