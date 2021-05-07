/*****************************************************************************
-------------------------------Organization------------------------------------
    Project             : FREEWAY
    Compiler            : TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
    Author              : Inverter part, Advanced Controls Research Team in Hyudai elevator(with KERI)
    Version             : v1.0
    Last Rev.           : 2018.12.28
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
                        : Added SCI Protocol Infomation Response Part (20181107 by GWON)
                        : Rearrangement of F/W for FREEWAY (20181112 by GWON)
                        : Added SPI Init Setting for between ARM and DSP communication (20181127 by GWON)
                        : Added SPI Control Communication Part (20181130 by GWON)
                        : Ver2 Board GPIO Setting (20181207 by GWON)
                        : Verified SPI Protocol (20181224 by GWON & Shin)
                        : Removed RMB, RAMP, MSTEP, FC (20190108 by GWON)
                        : Heidenhain Encoder F/W Merge using SPIC & DMA (20190702 by GWON)
                        : DAC Selection Data Insert at Inverter Data on SCI (20190714 by GWON)
                        : DAC Selection F/W is added on InitDa() (20190714 by GWON)
******************************************************************************/
#ifndef VARIABLE_H
#define VARIABLE_H

#include "F28x_Project.h"
#include "comp.h"
#include "gnl.h"
#include "KeyMenu.h"

#define	LAB_TEST	0
#define HHT_ENABLE	0			// 0: Disable, 1: Enable

//system setting
#define SYS_CLK			((float)200e6)
#define SYS_CLK_PRD		((float)5.0e-9)
#define OFFSET_MAX_CNT 	14 		// 2^13 = 8192?
#define sc_cnt_max 		5.		// 10kHz: 10, 15kHz: 15.
#define drv_cnt_max 	25.

#define SPI_RING_BUF_SIZE 8

// If you want to change the swithcing frequency, you only change blew line.
#define __5KHZ
//#define __10KHZ
//#define __15KHZ
//#define __30KHZ

// Encoder Resolution Setting
#define __ENC_RESOLUTION	1e-6;	// Linear Scale

#define LCDLEN		20
#define EDECEL_SW_ERR_LMT	300
/* Port Macro defines */
#define MCU0			GpioDataRegs.GPDDAT.bit.GPIO113
#define MCU3			GpioDataRegs.GPDDAT.bit.GPIO114
#define ME_UP			GpioDataRegs.GPCDAT.bit.GPIO83		//Added item at 20160118
#define ME_VP			GpioDataRegs.GPCDAT.bit.GPIO91
#define ME_WP			GpioDataRegs.GPCDAT.bit.GPIO93
#define HHT_MODE		GpioDataRegs.GPADAT.bit.GPIO12
#define MCU2			GpioDataRegs.GPADAT.bit.GPIO23 		// Signal of CP RX ready, active low
#define MCU1			GpioDataRegs.GPADAT.bit.GPIO22
#define INIC  			GpioDataRegs.GPEDAT.bit.GPIO131
#define INV_WD2			GpioDataRegs.GPDDAT.bit.GPIO96
#define nPFO_15V		(GpioDataRegs.GPFDAT.bit.GPIO164)
#define GATE_CUT_3V		(GpioDataRegs.GPBDAT.bit.GPIO40) 	// Fault signal of CP Watchdog I/O //Added item at 20160118
#define BKO             GpioDataRegs.GPEDAT.bit.GPIO134

//Monitor Port 
//To FRAM
#define nCS_FRAM_ON 	GpioDataRegs.GPDCLEAR.bit.GPIO109 	= 1
#define nCS_FRAM_OFF 	GpioDataRegs.GPDSET.bit.GPIO109 	= 1
#define nCS_DAC			GpioDataRegs.GPDDAT.bit.GPIO109
	
//Output Port 
//To MCU
#define MCU0_ON			GpioDataRegs.GPDSET.bit.GPIO113 	= 1
#define MCU0_OFF		GpioDataRegs.GPDCLEAR.bit.GPIO113 	= 1

#define MCU1_ON			GpioDataRegs.GPACLEAR.bit.GPIO22 	= 1
#define MCU1_OFF		GpioDataRegs.GPASET.bit.GPIO22 		= 1

#define MCU2_ON			GpioDataRegs.GPACLEAR.bit.GPIO23 	= 1
#define MCU2_OFF		GpioDataRegs.GPASET.bit.GPIO23 		= 1

#define MCU3_ON			GpioDataRegs.GPDCLEAR.bit.GPIO114 	= 1
#define MCU3_OFF		GpioDataRegs.GPDSET.bit.GPIO114 	= 1

#define nINIC_ON		GpioDataRegs.GPECLEAR.bit.GPIO131 	= 1
#define nINIC_OFF		GpioDataRegs.GPESET.bit.GPIO131 	= 1

#define nINV_ZSP_ON		GpioDataRegs.GPECLEAR.bit.GPIO135 	= 1
#define nINV_ZSP_OFF	GpioDataRegs.GPESET.bit.GPIO135 	= 1

#define nBKO_ON			GpioDataRegs.GPECLEAR.bit.GPIO134 	= 1
#define nBKO_OFF		GpioDataRegs.GPESET.bit.GPIO134 	= 1

#define nINV_RUN_ON		GpioDataRegs.GPECLEAR.bit.GPIO133 	= 1
#define nINV_RUN_OFF	GpioDataRegs.GPESET.bit.GPIO133 	= 1

#define nFAULT_ON		GpioDataRegs.GPESET.bit.GPIO132 	= 1
#define nFAULT_OFF		GpioDataRegs.GPECLEAR.bit.GPIO132 	= 1

#define INV_WD_ON		GpioDataRegs.GPECLEAR.bit.GPIO138 	= 1
#define INV_WD_OFF		GpioDataRegs.GPESET.bit.GPIO138 	= 1
#define INV_WD_TOG		GpioDataRegs.GPETOGGLE.bit.GPIO138	= 1

//To Output
#define nMC_ON			GpioDataRegs.GPECLEAR.bit.GPIO137 	= 1 
#define nMC_OFF			GpioDataRegs.GPESET.bit.GPIO137 	= 1

#define nINV_FAN_ON		GpioDataRegs.GPECLEAR.bit.GPIO136 	= 1 
#define nINV_FAN_OFF	GpioDataRegs.GPESET.bit.GPIO136 	= 1

#define nUCC_FAN_ON		GpioDataRegs.GPBCLEAR.bit.GPIO56 	= 1 
#define nUCC_FAN_OFF	GpioDataRegs.GPBSET.bit.GPIO56 		= 1

//To Inveter control
#define INV_WD1_ON		GpioDataRegs.GPCSET.bit.GPIO95 		= 1
#define INV_WD1_OFF		GpioDataRegs.GPCCLEAR.bit.GPIO95 	= 1
#define INV_WD1_TOG		GpioDataRegs.GPCTOGGLE.bit.GPIO95 	= 1

#define INV_WD2_ON		GpioDataRegs.GPDSET.bit.GPIO96 		= 1
#define INV_WD2_OFF		GpioDataRegs.GPDCLEAR.bit.GPIO96 	= 1 
#define INV_WD2_TOG		GpioDataRegs.GPDTOGGLE.bit.GPIO96 	= 1 

#define INV_TRIP_ON		GpioDataRegs.GPBSET.bit.GPIO34 		= 1
#define INV_TRIP_OFF	GpioDataRegs.GPBCLEAR.bit.GPIO34 	= 1

//To DAC
#define nCS_DAC0_ON		GpioDataRegs.GPDSET.bit.GPIO109 	= 1
#define nCS_DAC0_OFF	GpioDataRegs.GPDCLEAR.bit.GPIO109 	= 1

#define nDAC0_LD_ON		GpioDataRegs.GPDCLEAR.bit.GPIO118 	= 1
#define nDAC0_LD_OFF	GpioDataRegs.GPDSET.bit.GPIO118 	= 1


//To TEST
#define TEST1_ON		GpioDataRegs.GPBSET.bit.GPIO35 		= 1
#define TEST1_OFF		GpioDataRegs.GPBCLEAR.bit.GPIO35 	= 1

// 20170320 Added GWON
#define GATE_RST_OFF()	GpioDataRegs.GPCSET.bit.GPIO76 		= 0x1	// Software Reset OFF (GPIO76)
#define GATE_RST_ON()	GpioDataRegs.GPCCLEAR.bit.GPIO76	= 0x1	// Software Reset ON (GPIO76)
//3k

//-----------sampling & switching-------//
#ifdef __5KHZ
#define SWITCHING_FRQ       5.0e3  //���⼭ ����Ī ���ļ� �����ϸ�, EPWM������ �ڵ����� ������ (F2837xD_EPwm.c ����)
#define SAMPING_METHOD      1.0     //���⼭ �����ϸ�, variable.c���� ������ Tsamp �� �ڵ� ������ (F2837xD_EPwm.c ����)
#define PWM_INT_NUM         1       // 2 for 15kHz Switching, 3 for 10kHz Switching
#define CC_FREQ             5.0e3  // ���� ����� ���ļ�
#endif

#ifdef __10KHZ
#define SWITCHING_FRQ	    30.0e3	//���⼭ ����Ī ���ļ� �����ϸ�, EPWM������ �ڵ����� ������ (F2837xD_EPwm.c ����)
#define SAMPING_METHOD  	1.0 	//���⼭ �����ϸ�, variable.c���� ������ Tsamp �� �ڵ� ������	(F2837xD_EPwm.c ����)
#define PWM_INT_NUM			3		// 2 for 15kHz Switching, 3 for 10kHz Switching
#define CC_FREQ				10.0e3	// ���� ����� ���ļ�
#endif

#ifdef __15KHZ
#define SWITCHING_FRQ	    30.0e3	//���⼭ ����Ī ���ļ� �����ϸ�, EPWM������ �ڵ����� ������ (F2837xD_EPwm.c ����)
#define SAMPING_METHOD  	1.0 	//���⼭ �����ϸ�, variable.c���� ������ Tsamp �� �ڵ� ������	(F2837xD_EPwm.c ����)
#define PWM_INT_NUM			2		// 2 for 15kHz Switching, 3 for 10kHz Switching
#define CC_FREQ				15.0e3	// ���� ����� ���ļ�
#endif

#ifdef __30KHZ
#define SWITCHING_FRQ	    30.0e3	//���⼭ ����Ī ���ļ� �����ϸ�, EPWM������ �ڵ����� ������ (F2837xD_EPwm.c ����)
#define SAMPING_METHOD  	1.0 	//���⼭ �����ϸ�, variable.c���� ������ Tsamp �� �ڵ� ������	(F2837xD_EPwm.c ����)
#define PWM_INT_NUM			1		// 2 for 15kHz Switching, 3 for 10kHz Switching
#define CC_FREQ				30.0e3	// ���� ����� ���ļ�
#endif
//ET_CTR_PRD	    1.0 : single sampling
//ET_CTR_PRDZERO    2.0 : double sampling
//----------ADC Values-----------------//
//ADC �ἱ ����
#define Rm_Ain_Iac_1_5kW        10000.  // Iac Sensing Resistor --> 10000(LEM HX 05)
#define Rm_Ain_ESS_Idc          41.     // Idc Sensing Resistor
#define Rm_Ain_Vac_1_5kW        201.    // Vac Sensing Resistor
//#define Rm_Ain_Vdc_1_5kW      150.    // Vdc Sensing Resistor
#define Rm_Ain_Vdc_1_5kW        430.    // Vdc Sensing Resistor
#define Rm_Ain_ESS_Vdc          201.    // ESS_Vdc Sensing Resistor

//#define LV25_Rm_11kW          65.3e3  //LV25-P Sensing Resistor
#define LV25_Rm_1_5kW           100.0e3 //LV25-P Sensing Resistor
#define LV25_Rm_ESS             66.0e3  //LV25-P Sensing Resistor
#define SenScale_Iac_1_5kW      12500.  //HX 05
#define SenScale_ESS_Idc        2000.   //LA200-P Scale
#define SenScale_Vac_1_5kW      (LV25_Rm_1_5kW/2.5) //LV25-P Scale
#define SenScale_Vdc_1_5kW      (LV25_Rm_1_5kW/2.5) //LV25-P Scale
#define SenScale_ESS_Vdc        (LV25_Rm_ESS/2.5)   //LV25-P Scale

extern float ScaleAin_SOC0[4], ScaleAin_SOC1[4], ScaleAin_SOC2[4];
extern float OffsetAin_SOC0[4], OffsetAin_SOC1[4], OffsetAin_SOC2[4];

extern float ScaleADin_Vac_1_5kW, ScaleADin_Vdc_1_5kW, ScaleADin_Iac_1_5kW,
        ScaleADin_ESS_Idc, ScaleADin_ESS_Vdc;

extern float CONV_Ibs_Flt_HW_Set;	//CONV -Ibs
extern float CONV_Ics_Flt_HW_Set;	//CONV -Ics
extern float CONV_Ias_Flt_HW_Set;	//CONV -Ias
extern float Vdc_Flt_HW_Set;	//Vdc

extern float INV_Ibs_Flt_HW_Set;	//INV Ibs
extern float INV_Ics_Flt_HW_Set;	//INV Ics
extern float INV_Ias_Flt_HW_Set;	//INV Ias
extern float ESS_I_Flt_HW_Set;	// sup I

extern float GRID_Vbc_HW_Set;	// grid B-C
extern float GRID_Vab_HW_Set;	// grid A-B
extern float ESS_V_Flt_HW_Set;	// super V (2.8*240=672V rated, 2.7*240= 648V max)

extern Uint16 maxCount_ePWM;
extern Uint16 halfCount_ePWM;
extern float halfTsw_PWM;
extern float Tsamp;
extern float INV_Tsamp;
extern float Freq_i;

typedef union{
	float *ptr;
	Uint32 pad;
}CLA_FPTR;

typedef union{
	Uint16 *ptr;
	Uint32 pad;
}CLA_UIPTR;

typedef union{
	Uint16 *intptr;
	float *fptr;
	Uint32 pad;
}CLA_PTR;

//define PWM Buffer Control
#define PWM1_BUFF_ON()	GpioDataRegs.GPCCLEAR.bit.GPIO94 	= 0x1 //nINV1ENA (GPIO94)
#define PWM1_BUFF_OFF()	GpioDataRegs.GPCSET.bit.GPIO94 		= 0x1
//#define PWM2_BUFF_ON()	GpioDataRegs.GPCCLEAR.bit.GPIO90 	= 0x1 //nINV2ENA (GPIO90)
//#define PWM2_BUFF_OFF()	GpioDataRegs.GPCSET.bit.GPIO90 		= 0x1
//#define PWM3_BUFF_ON()	GpioDataRegs.GPCCLEAR.bit.GPIO92 	= 0x1 //nINV3ENA (GPIO92)
//#define PWM3_BUFF_OFF()	GpioDataRegs.GPCSET.bit.GPIO92 		= 0x1

#define PWM1_BUFF_TRIP()	GpioDataRegs.GPBSET.bit.GPIO34 		= 0x1 // INV1TRIP (GPIO34)
//#define PWM2_BUFF_TRIP()	GpioDataRegs.GPBSET.bit.GPIO37 		= 0x1 // INV2TRIP (GPIO37)
//#define PWM3_BUFF_TRIP()	GpioDataRegs.GPBSET.bit.GPIO48 		= 0x1 // INV3TRIP (GPIO48)

// Motor Parameter
typedef struct 	_Para_Motor_
{
	Uint16	Pole, PolePair;
	float	InvPolePair;
	float	Rs, Ls, Ke, Lds, Lqs, LAMpm, InvLAMpm;
	float	Te_rated, Is_rated;
	float	Te_max, Iq_max, Id_max;
	float	Pout_rated;
	float	Jm, Bm, InvJm;
	float 	Kt, InvKt;
	float 	Ias, Ibs, Ics ;
	float 	Idss, Iqss, Idse, Iqse  ;
	float 	Inss;
	float	Thetar, Thetam, ThetaAdv, ThetaOffset ;
	float	Thetasl, Thetae;
	float	Cos_Theta, Sin_Theta, Cos_ThetaAdv, Sin_ThetaAdv ;
	float	Cos_Thetasf, Sin_Thetasf;
	float 	Wr, Wrm, Wrpm, We, Wsl ;
	float 	Vas, Vbs, Vcs ;
	float 	Vdss, Vqss, Vdse, Vqse ;
	//For Induction Machine
	float	Rr, Lr, InvLr, Llr, Lls, Lm, InvLm, sigmaLs, InvLdiff;
	float	Lamdss, Lamqss;
	float	Lamdss_flt, Lamqss_flt;
	float	Lamdrs, Lamqrs;
	float	Theta_est, Cos_Theta_est, Sin_Theta_est;
	float	Lamdre, Lamqre;
	float	Lamdre_ref, InvLamdre_ref;
	float 	INV_Rs;

	float	Thetasf;
	float	Lamdas_mag;
	float	Te;
	float	Lamdas_d;
	float	Lamdas_q;
	float	Idssf, Iqssf;
	float	Theta_init,Del_thetar_init, INV_CNT_thetar_set;
	float	Thetar_old, Thetar_init_new;
	
	//For high frequency injection
	float   Theta_dm, Theta_i;
	float 	Sin_Theta_dm, Cos_Theta_dm;
	float   Idm, Iqm, Idm_B, Iqm_B;
	float   Idmcos, Idmsin, Idmcos_L, Idmsin_L;
	float   Idm_amp;
	float   Iqmcos, Iqmsin, Iqmcos_L, Iqmsin_L;
	float   Iqm_amp;
	float   Sin_Theta_i, Cos_Theta_i;
	float   Idse_L, Iqse_L;
	
	//For Freeway
	float	PolePitch;
	float	InvPolePitch;
	float 	MaxSpeed;
	float 	Mass;
	float 	FL;
	float	FL_gain;
	float	CreepSpeed;
	float	InspectSpeed;

	float 	Ias_avr;
	float 	Ibs_avr;
	float 	Ics_avr;

	float 	Te_Limit;

    float   Active_Power;
    float   Reactive_Power;
    float   Active_Power_f;
    float   Reactive_Power_f;
    float   Apparent_Power;
    float   Apparent_Power_f;
    float   Power_Factor;
} Motor;
extern Motor INV1, INV2;//, INV3;
extern void InitParameters(void);  // Motor Parameter

// Flags
typedef struct 	_FLAGS_
{
	Uint16 Start1, Start2;
	Uint16 Run1, Run2, Run3, Run4;
	Uint16 Vdc;//, Vsc;
	Uint16 Vdc_filter;
	Uint16 Fault, Fault_Warn, Fault_Warn_RunCnt ;
	Uint16 Ready1,Ready2;//, Ready;
	Uint16 AngleAdv;
//	int Zcc ;

	Uint16 Align1;
	Uint16 Align2;
	Uint16 Speed_control_1;
	Uint16 Speed_control_2;

	Uint16 Vdc_control;
	Uint16 inertia_emulation;

	Uint16 Run_PLL;
	Uint16 Profile_Control;

	Uint16 Vsup_control;
	Uint16 Vdc_control_using_SuperCap;
	
	/////////////// HD Variable ///////////////////////////////
	Uint16	Conv_Run, Inv_Run, Conv_Run_old, Inv_Run_old;
	Uint16	Ready_INV;
	Uint16	Check_Vdc_charge, MC_Relay;
	Uint16	GRID_Volt_Seq, GRID_Volt_SeqChk;
	Uint16	Vdc_Control_Ok;
	Uint16  Fault_old, Fault_Warn_old;
	Uint16  Not_Control_SuperCap;
	Uint16	RotorPos_est;
	Uint16	HF_Inj;
	///////////////////////////////////////////////////////////
	///////////////// FREEWAY Variable ////////////////////////
	Uint16	Op_Mode;
	Uint16	ProfileMode;
	Uint16	RTN2ZERO;
	Uint16 	Position_control;
	Uint16  Kt_mode;
	Uint16	Pos_Con_Kt_mode;
	Uint16	CompProfile;
	Uint16  CompProfileEnd;
	Uint16	PreLoad;
	///////////////////////////////////////////////////////////
	Uint16	PosCon_with_APS;
	Uint16  PosCon;
	Uint16  Decel;
	//////////////////////////////////////////////////////////
	Uint16  Bootup;
} FLAG;
extern FLAG Flags;
extern void InitFlags(void);

extern float Tsc;
extern float INV_Tsc;
extern float Tdrv;
//DC link ���� ����
extern float Vdc, Vdc_rd, Vdc_set, INV_Vdc;
extern float Vdc_filter;

//Super Cap Vdc
extern float Vsup_dc;
extern float Flag_power_control_mode;

/////////////////////////////////////////////////////////////////////////
/*Fault*/
extern u8 FLT_backup[50];
extern u8 Fault_RealYear[10];
extern u8 Fault_RealMonth[10];
extern u8 Fault_RealDate[10];
extern u8 Fault_RealHour[10];
extern u8 Fault_RealMinute[10];
extern u8 Fault_RealSec[10];

extern Uint16	EncType, KNOW_ANGLE;
/*CC*/
extern float E_mag_set, Eld_In_Volt, InVoltage, Vdc_relay_on, UVDC_set, Vdc_relay_off;
extern float ELD_Vdc_relay_on, ELD_UV_set, ELD_Vdc_relay_off ;
extern float Is_rate, OVDC_Set;
extern float Theta_U, TqBiasLmt;
extern float OC_Set_I, OC_Set_C, OC_Set_Sup, OVDC_Sup;
extern Uint16 Flag_MecaObs, Flag_EncFilter;
extern float Wcc;
extern float sum_current1;
extern float Normal_VDC;
/*SC*/
extern float Wsc;
/*PC*/
extern float Wpc;
extern float test_postion;
/* Auto Tqbias vari */
extern Uint16 Flag_Plus, Flag_Plus_old, Flag_Exit, Flag_READY, Tqbias_Step_Cnt, TQSELECT;
extern Uint16 Enc_Dir,FWD_Dir;
extern float SC_fTqBias_pre, SC_fTqBias, SC_fTqBias_off_didt, SC_fTqBias_off;
extern float SC_fTqBias_didt;
extern Uint16 uEL_Start, Flag_RUN, Flag_slope;
extern Uint16	Flag_offset;
extern float OVER_SP_set, OVER_SP_set_K;
extern float SP_Limit;
extern float TqBias_Step, Iqse_Pos_ff, Kpp;
extern float Pos_real, EncPPR;

/*SinCos Encoder*/
extern float	SinMax, SinMin, SinMax_old, SinMin_old, SinOffset, CosMax, CosMin, CosMax_old, CosMin_old, CosOffset ;
extern float 	thetar_H, thetar_L ;
extern int 		Flag_thetar_H, Flag_thetar_L, Flag_thetar_False ;

extern unsigned int Flag_SINCOSTH, Flag_Sin, Flag_U, Flag_Atan, Flag_Atan_old, AtanCnt, Z_Cnt, Z_Cnt_old ;

/*SCI*/
extern Uint16 Scid_Tx_Buf[100], Scic_Tx_Cnt, Scic_Tx_End, Scid_Tx_Cnt, Scid_Tx_End ;
extern Uint16 scic_tx_cnt ;
extern Uint16 crc16_table[256] ;
extern Uint16 Key_In, Key_In_old, Key_Value ;

/* Non-Stop Vari */
extern Uint16 NumOfnonStop, nonStop_Cnt, nonStop_LoopCnt;//non-stop
extern Uint16 nonStop_Flr[64];

/*EEPROM*/
extern long FRAM_value, E_Check ;

/*BDIB vari*/
extern int BDIB_X3, BDIB_X2, BDIB_X1,	BDIB_SUF,	BDIB_PLUH,	BDIB_RST,	BDIB_PLDH,	BDIB_PLDM, BDIB_CP_AT,	BDIB_SDF/*,	BDIB_INI*/,	BDIB_HOU,	
		BDIB_HOD,	BDIB_PLUM, BDIB_ENABLE, BDIB_DZ, BDIB_RDY, BDIB_ULS, BDIB_DLS, BDIB_MCC, BDIB_UP, BDIB_DOWN;

extern int BDIB_PLDH2, BDIB_PLUH2, BDIB_XMCS, BDIB_X29, BDIB_XAUTO;
extern int BDIB_XMC2, BDIB_XBKAS, BDIB_XBKBS, BDIB_FUSC;
extern int BDIB_INP1, BDIB_INP2, BDIB_INP3;
extern int BDIB_SUS4, BDIB_SUS5, BDIB_SUS6, BDIB_SDS4, BDIB_SDS5, BDIB_SDS6;
		
extern float fSUF_stPos, fPLUM_stPos, fPLUH_stPos, fPLUH2_stPos, fSUS4_stPos, fSUS5_stPos, fSUS6_stPos;
extern float fSDF_stPos, fPLDM_stPos, fPLDH_stPos, fPLDH2_stPos, fSDS4_stPos, fSDS5_stPos, fSDS6_stPos;
extern float fSUF_RealPos, fPLUM_RealPos, fPLUH_RealPos, fPLUH2_RealPos, fSUS4_RealPos, fSUS5_RealPos, fSUS6_RealPos;
extern float fSDF_RealPos, fPLDM_RealPos , fPLDH_RealPos, fPLDH2_RealPos, fSDS4_RealPos, fSDS5_RealPos, fSDS6_RealPos;
extern float fSUF_Diff, fPLUM_Diff, fPLUH_Diff, fPLUH2_Diff, fSUS4_Diff, fSUS5_Diff, fSUS6_Diff;
extern float fSDF_Diff, fPLDM_Diff, fPLDH_Diff, fPLDH2_Diff, fSDS4_Diff, fSDS5_Diff, fSDS6_Diff;
extern float fRunnigOpenLength, fRunOpenLength, fChime_Point ;
extern int	ChimeFlgStart ;
extern int 	Inverter_Sel;
extern int RMB_ELSpeed, RMB_Decel_Flg;
extern float fRMB_Decel_Flg, RMB_fVmax_K;
extern int Van_ErrFlg;
extern float fVariOffset;
extern Uint16 ELD_SpeedMode_Flg;

extern float fmpm_Spd_fbk;
extern int BDIB_HOD_Old, BDIB_HOU_Old;
extern int Rsc_Vane;
extern unsigned long ulCurrent_Mileage, ulCurrent_Mileage_temp;
extern long lCurrent_Mileage, lCurrent_Mileage_temp;
extern Uint16 LimitOver_Mileage_Flg;
extern Uint16 DoorZone_Flg;
extern int CALL_Data, RMB_DIV9_Flg, CALL_Data_Old;
extern Uint16 uPloCreepFlag;
extern int	SaveFault_Flg;

extern int RMB_chgStimeAcc_Flg;
extern float fRMB_Stop_Pos, fRMB_reCalDecelLength;

extern int CP_Lab_Test;
extern float fTqBiasValue;
extern Uint16 RMB_SpdPtnSel, DecelModeSelect, HHT_Use_Flg, Man_Cmd, Man_SpdSel;
extern Uint16 Inv_Control_Mode, Motor_Type, Mth_Use;
extern float fRMB_DblSpd_Pos_Start, fRMB_DblSpdDiffPos;
extern float fDecFloorOffset, fDecFloorOffset_delta, Commu_delay ;

/*Forced decel switch control*/
extern int   SUDS_SW_NUM , SUDS_POS_Err, Flag_mode, EL_Speed, SUDSOK, SUDS_ONOFF ;
extern float RMB_fVref_Lmt ;
extern float SUDS_SW_Pos[7], SUS_Vref[7], SDS_Vref[7] ;
extern float Acc2_SUDS, RMB_fPosLmt, SUDSPos, RMB_fPosLmt_SW;
extern float RMB_fVrefOld_Lmt, Acc_SUDS ;
extern Uint16 	SUDS_SW_BIT, LimitSw_PosFlg ;
extern float 	qSUDSRegs, qRMBfVref, qRMBfVref18, qSUDSMode, qfPosLmt, qfPos, qSUDSPos, qfpos, qPosDiff ;
extern int Flag_SUSD;
extern float SUDS_fPositionLength, SUDF_fPositionToLength, SUDS_Vmax;
extern int SUDS_UPDN;
extern Uint16 SUDS_Ptn_Use;
extern float SUDS_LENGTH_OFFSET, SUDS_VANE_LENGTH;

/*Floor Control*/
extern float fTqunbal, fTqunbal_tmp, Sign_TqCom;
extern float TqUnbal_Bot[11], TqUnbal_Top[11];
extern Uint16 INIT_DRV, fFlagAfterInit ;
extern Uint16 ELD_Speed_Mode;

/*Drive*/
extern int  u20msFlag, Drive_Cnt ;
extern int 	Flag_ZSP;
extern int 	DriveStatPsudo, ANGLE_cnt, HwFltCnt, HwFltWarnCnt, SwFltCnt, DclFlag ;
extern int Flag_Tans, Flag_mode, Flag_Err, Bko;
extern float I_phase_rms, V_ll_rms, C_phase_rms ;

extern float DRV_fLS_LoadAD, DRV_fLS_LoadAD_L,DRV_fLS_LoadAD_Old;
extern float DRV_fLS_Load_AD50, DRV_fTqBias, DRV_fLS_Load_Gain , DRV_fLS_LoadtoTq;
extern float DRV_fLS_Load_AD0, DRV_fLoadData, DRV_INV_perLoad;

extern int	encU, encV, encW, encZ, encUVW, encUVW_old, Flag_thetar_reset, CNT_thetar, CNT_thetar_set, Reset_Thetar_Flg;
extern int	encUVW_Chk_old, Flag_reset, encU_old;
extern int 	ENC_AB, GENC_AB, ENC_ABCnt, GENC_ABCnt, ENCnt, ENCHK;

/*Fault*/
extern int ErrGetFlag;
/*Test*/
extern Uns Power_Test, Fwd_Rev_Test;
extern float Power_Test_Gain, Power_Test_Idse;
/*AD Offset*/
extern int64 Offset_ADC_SOC0[4];
extern int64 Offset_ADC_SOC1[4];
extern int64 Offset_ADC_SOC2[4];
extern int32 offsetLoopCnt, offsetMaxCnt;

/*Grid PLL*/
extern Uint16 	sequence_count;

extern float SC_F_load, SC_T_load;
extern float SC_mass_real;
extern float SC_mass_total;	
extern float SC_P_emul;
extern float SC_Iqse_init;
extern float Vcap_L;
extern float P_elec_grid;
extern float P_conv_loss;
extern float P_mech_real;
extern float Power_Limit_Set;

extern float temp_cal;
extern float VC_Iqse_Ref;
extern float VC_P_sup;

extern float Vcap, RMB_INV1_Iqse_50L;

extern float Thermal;

extern int SYNorLIN_Motor;
extern int SYNorLIN;
extern float Test_Key_Menu;

/* CC */
extern int CC_ExTimeChk_Flg;
extern float cc_V_ref;
extern int v_mode;
extern int Static_Force_Measurement;
extern float V_ref_mag;
extern float cc_Hz_f;
extern u8 TxCNT;
extern u16 u16TXsize;
extern u16 u16SpiTXsize;
extern u8 u8ScicRx[256];
extern int TestSpi[256];
extern u8 u8SpibTxCnt;
extern u8 u8SpibRxCnt;

/* Inverter Data Transformation (SPI) */
extern u32 u32Wrm;
extern u32 u32Wrm_Ref;
extern u32 u32Ide;
extern u32 u32Ide_Ref;
extern u32 u32Iqe;
extern u32 u32Iqe_Ref;
extern u32 u32V_ref_mag;
extern u32 u32Vdc;
extern u32 u32Te;
extern u32 u32CurrentPos;
extern u8 u8SpdFaultCnt;
extern u8 u8OvAngleCnt;

/* Spi Ring Buffer */
//typedef struct __CallQueue
//{
//    u32 spi_ring_buf[SPI_RING_BUF_SIZE];  // Call buffer
//    u8 u8RunStatus[SPI_RING_BUF_SIZE];    // Direction buffer
//}CALLQUEUE;
extern u32 spi_ring_buf[SPI_RING_BUF_SIZE];     // ring buffer
extern u32 u32spi_ring_buf_len;                 // ring buffer length
extern u32 u32spi_ring_buf_lp;                  // load pointer
extern u32 u32spi_ring_buf_cp;                  // consume pointer

extern u8 u8TestTypeCasing[6];


extern u8 Flag_DA_Test;
extern float32 Theta_grid;
extern float32 GRID_Vas, GRID_Vbs, GRID_Vcs;
extern float32 GRID_Vab, GRID_Vbc, GRID_Vca;

extern u32 g_u32MAX_FLOOR;
extern float SetPosition;
extern u8 u8Spic_Comm_Chk;
extern u8 u8Spic_SC_Comm_Chk;
extern u8 DMA_Start_CNT;
#endif  // end of SNU_CPU1_VARIABLE_H definition
