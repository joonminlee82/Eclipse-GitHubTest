/*****************************************************************************
-------------------------------Organization------------------------------------
    Project             : FREEWAY
    Compiler            : TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
    Author              : Inverter part, Advanced Controls Research Team in Hyudai elevator(with KERI)
    Version             : v1.0
    Last Rev.           : 2019.07.22
    History             : Linear Motor Selection Menu is added into the HHT Menu (20180206 by GWON)
                        : Motor Side R Controller is added in order to suppress 3th, 6th 9th harmonics, but this controllers are disable. (20180206 by GWON)
                        : Mark to need to modify for FREEWAY --> Search command "// Need to modify for FREEWAY" (20180209 by GWON)
                        : Modified something for Linear Motor. ex) pole_Pitch, polepair, ... etc., (20180417 by GWON)
                        : Modified Position and Mechinical/Electrical Angle Calculation Function (20180418 by GWON)
                        : Inserted Rotor Position Estimation Function (20180430 by GWON)
                        : Profile Generation Function (20180508 by GWON)
                        : Fault Definition & Setting (20180509 by GWON)
                        : Added Profile Error Compansator (20180510 by GWON)
                        : Added Control Code using HHT (20180629 by GWON)
                        : Added Mechanical_Observer for FRLSM (20180718 by GWON)
                        : Load Mass Estimator using Te for FRLSM (20180718 by GWON)
                        : Modified MC Operation Condition. Over 540V --> Over UVDC_Set (20180718 by GWON)
                        : STOP Mode Speed Max Value 0.1 --> 0.05 (20180718 by GWON)
                        : TX_INDEX 20 --> 40 (20180806 by GWON)
                        : Added SCI Protocol Version Response Part (20181107 by GWON)
                        : Added SCI Protocol Information Response Part (20181107 by GWON)
                        : Rearrangement of F/W for FREEWAY (20181112 by GWON)
                        : Added SPI Init Setting for between ARM and DSP communication (20181127 by GWON)
                        : Added SPI Control Communication Part (20181130 by GWON)
                        : Ver2 Board GPIO Setting (20181207 by GWON)
                        : Verified SPI Protocol (20181224 by GWON & Shin)
                        : Removed RMB, RAMP, MSTEP, FC (20190108 by GWON)
                        : Heidenhain Encoder F/W Merge using SPIC & DMA (20190702 by GWON)
                        : DAC Selection Data Insert at Inverter Data on SCI (20190714 by GWON, 20190722 Modified by GWON)
                        : DAC Selection F/W is added on InitDa() (20190714 by GWON, 20190722 Modified by GWON)
******************************************************************************/
#ifndef _CC_H
#define _CC_H

#include "F28x_Project.h"
#include "DAC.h"
#include "Variable.h"
#include "eQEP_Encoder.h"
#include "F2837xD_sdfm_drivers.h"
#include "R_control.h"
#include "Filter.h"
#include "SPI.h"

#define __LINEARSCALEMODE       0x30
#define __HEIDENHAINMODE        0x31
#define __MAGNETICSENSORMODE    0x32

#if defined(__TMS320C28XX_CLA__)

#else
	#include <math.h>
#endif

__interrupt void cpu1_cc(void);

void Wait_EOC_ADC_SOC0(void);
void Wait_EOC_ADC_SOC1(void);
void Wait_EOC_ADC_SOC2(void);

typedef struct 	_CC_3PH_
{
	// Inputs
	//float	RefSlope;
	float	Err_Idse, Err_Iqse;
	float	Idse_Ref, Iqse_Ref;//, Te_Ref_Set, Te_Ref;
	float	Idse_Ref_set, Iqse_Ref_set ;  //<-내가 넣은거

	// PI controller
	float 	Wc;
	float	Kid, Kid_T, Kpd, Kad;
	float	Kiq, Kiq_T, Kpq, Kaq;
	float 	Ra;
	float	Vdse_Ref_Integ, Vdse_Ref_FB, Vdse_Ref_FF, Vdse_Ref;
	float	Vqse_Ref_Integ, Vqse_Ref_FB, Vqse_Ref_FF, Vqse_Ref;
	float	Vdss_Ref, Vqss_Ref;
	float	Vas_Ref, Vbs_Ref, Vcs_Ref;
	float	Vdse_Anti, Vqse_Anti;//<- 내가 넣은거

	float	Vmax, Vmin, Vsn, Vsn_max, Vsn_min;
	float	Van_ref, Vbn_ref, Vcn_ref;
	float	pwmTa, pwmTb, pwmTc;
	float 	Tdead, Icomp0, INV_Icomp0 ;
	float 	TaComp, TbComp, TcComp, TaZcc, TbZcc, TcZcc;

	Uint16 	Cnt_VaRef;
	Uint16 	Cnt_VbRef;
	Uint16 	Cnt_VcRef;

	//JS
	float Idss_ref, Iqss_ref;
	float Ias_ref, Ibs_ref, Ics_ref;
	float Ias_DComp, Ibs_DComp, Ics_DComp;

	float Van_Comp,Vbn_Comp,Vcn_Comp;
	float Vsat;
	float Isat;
	float Ktraps;
	Uint16 Flag_InvComp;
	Uint16 Dead_Time_Comp;
	Uint16 Ref_Dead_Comp;
	Uint16 Flag_Rcontrol;

	//R_control R_6d, R_12d;
	//R_control R_6q, R_12q;
    //
	//IIR1 All_pass_6d;
	//IIR1 All_pass_6q;
	//IIR1 All_pass_12d;
	//IIR1 All_pass_12q;
    //
	//float All_filter_output_6d;
	//float All_filter_output_6q;
	//float All_filter_output_12d;
	//float All_filter_output_12q;
	//Uint16 Flags_All_pass_6dq;
	//Uint16 Flags_All_pass_12dq;
	
	R_control R_1d, R_1q;
	R_control R_2d, R_2q;
	R_control R_3d, R_3q;
	R_control R_6d, R_6q;
	R_control R_8d, R_8q;
	R_control R_9d, R_9q;
	R_control R_12d, R_12q;

	IIR1 All_pass_1d;
	IIR1 All_pass_1q;
	IIR1 All_pass_2d;
	IIR1 All_pass_2q;
	IIR1 All_pass_3d;
	IIR1 All_pass_3q;
	IIR1 All_pass_6d;
	IIR1 All_pass_6q;
	IIR1 All_pass_8d;
	IIR1 All_pass_8q;
	IIR1 All_pass_9d;
	IIR1 All_pass_9q;
	IIR1 All_pass_12d;
	IIR1 All_pass_12q;

	float All_filter_output_1d;
	float All_filter_output_1q;
	float All_filter_output_2d;
	float All_filter_output_2q;
	float All_filter_output_3d;
	float All_filter_output_3q;
	float All_filter_output_6d;
	float All_filter_output_6q;
	float All_filter_output_8d;
	float All_filter_output_8q;
	float All_filter_output_9d;
	float All_filter_output_9q;	
	float All_filter_output_12d;
	float All_filter_output_12q;

	Uint16 Flags_All_pass_1dq;
	Uint16 Flags_All_pass_2dq;
	Uint16 Flags_All_pass_3dq;
	Uint16 Flags_All_pass_6dq;
	Uint16 Flags_All_pass_8dq;
	Uint16 Flags_All_pass_9dq;
	Uint16 Flags_All_pass_12dq; 
	
	float Iqse_Ref_L;
	float Wr_L, Wrm_L, Wrpm_L;
	
	//For high frequency injection
	float Sin_Theta_hf, Cos_Theta_hf;
	float Err_Im, Dth_Hat, Dth_set;
	float thetar_hat, Theta_hf, Theta_hf2;
	float Sin_Theta_hf2, Cos_Theta_hf2;
	float Idmcos2, Idmsin2, Idmcos2_L, Idmsin2_L;
	float Theta_dir, Wc_i;
	float VD_INJ, Vdse_i, Vdss_i, Vqss_i;
	
	// For FREEWAY
	float CCtime;
	
} CC_3PH;

extern CC_3PH INV1_CC, INV2_CC;

extern void UpdateCCGains(Motor *tmpM, CC_3PH *tmpC);
extern void ResetCC(CC_3PH *tmpC) ;
extern void InitCCVars(CC_3PH *tmpC, float Wc);

// mechanical observer
typedef struct 	_Para_Mechenical_Observer_
{
  	float Thetar_error;
  	float Te;
  	float TL, Wrm, Thetam;
  	float Thetam_unb;
  	float Wr, Wrpm, Thetar;
  	float Vr;
  	float L1, L2, L3;
  	float diff_Wrm_observer;
  	float diff_Wrm_observer_model;

  	float Wbw;
	float inv_Jm;
	Uint16 Thetam_dir;
}Mechanical_Observer_st;

extern Mechanical_Observer_st INV1_Spdobs;
extern Mechanical_Observer_st INV1_Spdobs_for_Acc;
void Updata_MachaObse_Gains(Motor *tmpM, Mechanical_Observer_st *OBS);
void Mechanical_Observer(Motor *tmpM, float thetar_est, Mechanical_Observer_st *OBS);
void Reset_Mecha_Obs(Mechanical_Observer_st *OBS);

#define		LPF		0
#define		BPF		1

void CurrCtrl(Motor *tmpM, CC_3PH *tmpC, Uint16 mode);
void PwmUpdate(Motor *tmpM, CC_3PH *tmpC , volatile struct EPWM_REGS *phA, volatile struct EPWM_REGS *phB, volatile struct EPWM_REGS *phC);
void InvNonComp(CC_3PH *tmpC, Motor *tmpM);
void Current_Transform(Motor *MOT, Uint16 mode);
void CC_VariUpdate(void);
void Init_ADScale(void);
void ADC_Read(void);																						// Added for FREEWAY 20180209 by GWON
void Rotor_InitPos_Estimation(Motor *tmpM, CC_3PH *tmpC, Uint16 mode);
void Init_Inv(void);
void DC_MC_OP(void);																						// Added for FREEWAY 20181113 by GWON
void SW_Fault_Check(void);																					// Added for FREEWAY 20181113 by GWON
void Applied_Speed_Variable(u8 mode, Motor *tmpM, CC_3PH *tmpC, Encoder_str *p, ENC_DATA_OBJ *pp);	        // Added for FREEWAY 20181113 by GWON
void Applied_Speed_Observer(u8 mode, Motor *tmpM, CC_3PH *tmpC, Mechanical_Observer_st *tmpObs, Encoder_str *p, ENC_DATA_OBJ *pp);		// Added for FREEWAY 20181113 by GWON
void Calc_APS_POS_SPD();
void Calc_Hiedenhain_POS_SPD(ENC_DATA_OBJ *p);
#endif  // end of SNU_CPU1_CC_H definition
