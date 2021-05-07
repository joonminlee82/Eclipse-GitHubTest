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
#ifndef	_SC_H_
#define	_SC_H_

#include "CC.h"

__interrupt void cpu1_sc(void);
__interrupt void cpu1_enc_Z(void);

typedef struct 	_SC_
{
	float32	Wrm, Wr, Wrpm;
	float32	Wrm_ref, Wr_ref, Wrpm_ref, Wrpm_ref_set ;
	float32	Te_ref_fb, Te_ref_ff, Te_ref, Te_ref_integ, Te_real, Te_Anti;
	float32	Err_Wrm, Err_Wr;
	float32	Ki_sc, Ki_scT, Kp_sc, Ka_sc, alpha_sc ;
	float32 RefSlope ;
	float32 Te_ref_max , Iqse_ref_max ;
	float32 Wc_sc ;
	float32 Te_ref1;
	float32 Wrpm_ref_old, Acc_ref;
	float32 SCFF_Gain;
	
	float32	Spd, Spd_d1; //??

	float32 APSWrm1, APSWrm2;
} SC;

typedef struct _PC_
{
	float32	PCtime;
	float32	Err_Pos;
	float32	PosRefFinal, Pos, Pos_d1;
	float32	Kp_pc, Ki_pc;
	float32	SignProfile;
	float32 POS_CMD;
	float32 POS_CMD_d1;
	float32	Spd_ref;
	float32 Spd_ref_max;

	float32 TimeStep_period; // unit: [sec] -> Step mode일때 Step period
	float32 TimePC;			// unit: [sec]
	float32 time_period; 	// unit: [sec]
	float32 Goal; 			// unit: [m]
	float32 Init; 			// unit: [m]
	float32 TimeStep; 		// unit: [sec]
	float32 PosStep; 		// unit: [m]

	float32 APSPos;			// unit: [m]
	float32 APSTheta;

	float32 LajerPosUp;
	float32 LajerPosDown;

	float32 Pos_Offset;
	float32 Remain_Distance;

	u32 u32MaxFloor;
	float32 MaxLength;
} PC;

typedef struct _PROFILE_
{
	// In order to solve second order equation
	float32 a;
	float32 b;
	float32 c;

	// PosProfileGeneration
	float32 POS_Ref;
	float32 AccMax;      		// 최대 가속도 설정
	float32 SpeedMax;    		// 최대 속도 설정
	float32 SpeedMax_short;		// Short Profile 최대 속도 계산
	float32 Sratio;      		// 가속 구간중 가가속 구간 비율 설정  t1/(2*t1 + tp)
	float32 PosDistance;
	float32 PosDistance_old;
	float32 PosRefc;
	float32 Jerk;
	float32 JerkMax;			// 최대 저크 설정
	float32 JerkTime;			// 저크 시간 계산: AccMax/JerkMax
	float32 Acc_time_total;
	float32 Jerk_time_total;
	float32 Acc_time_mid;
	float32 V_time_mid;
	float32 V_time_total;
	float32 Second_Hmin;

	Uint32 iAcc_time_total;
	Uint32 iJerk_time_total;
	Uint32 iAcc_time_mid;
	Uint32 iV_time_mid;
	Uint32 iV_time_total;

	float32 t1;   				// 가가속 구간 시간 계산
	float32 tp;   				// 정가속 구간 시간 계산
	float32 tc;   				// 0 가속구간 시간 계산

	float32 A1;
	float32 A2;
	float32 A3;
	float32 A4;
	float32 A5;
	float32 A6;
	float32 A7;
	float32 A8;
	float32 A9;
	float32 A10;

    Uint32  iA1;
    Uint32  iA2;
    Uint32  iA3;
    Uint32  iA4;
    Uint32  iA5;
    Uint32  iA6;
    Uint32  iA7;
    Uint32  iA8;
    Uint32  iA9;
    Uint32  iA10;

	float32 SpeedRef_d1;
	float32 PosRef_d1;
	float32 AccRef_d1;
	float32 Jerk_d1;

	float32 AccRef;
	float32 PosRef;
	float32 SpeedRef;
	float32 SignProfile;

	float32 POS_CMD;
	float32 POS_CMD_old;
	float32 POS_CMD_PRESENT;

	float32 temp_cal;

	float32 TCH;
	float32 TCH_d1;
	float32 PROFILE;
	float32 test_profile;

	float32 PosRefFinal;

	Uint32 ProfileGen_Flag;

	float32 Tprofile;

	Uint32 sc_cnt;

	float32 Stop_Time;
	/* For Profile Compansation */
	float32 Err_Profile;
	float32 Comp_Profile;
	float32 Chk_Err;

	float32 CreepSpeed;
	float32 InspectSpeed;
	float32 RelevelSpeed;

	float32 Minimized_distance;

	Uint32 PH[7];
	u8 Profile_Status;
} PROFILE;

extern SC	INV1_SC,INV1_SC_FF;
extern PC	INV1_PC;
extern PROFILE  INV1_PROFILE;

void Init_SC(void);
void Position_control(PC *tmpP);
void Speed_control(SC *tmpS);
void Init_PROFILE(void);
void Controller_Gain_Kt_Tuning(void);
void PosProfileGeneration(PROFILE *tmpPRO, PC *tmpPC);
void COMP_PROFILE(PROFILE *tempPRO);
void Pos_StepMode(void);

extern void SC_VariUpdate(void);
extern void PC_VariUpdate(void);

extern void UpdateSCGains(Motor *tmpM, SC *tmpS);
extern void InitSCVars(SC *tmpS, float slope, float TeMax, float IqMax, float Wc);
extern void ResetSC(SC *tmpSC);

extern void UpdatePCGains(void);
extern void InitPCVars(PC *tmpP, float SpdMax);
extern void ResetPC(PC *tmpPC);

float Cal_Kt_Func(float x, float mode, float mode2);
float Est_Load_Func(float x, float mode);
#endif
