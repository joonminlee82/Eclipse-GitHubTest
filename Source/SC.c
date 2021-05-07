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
#include "SC.h"
#include "CC.h"
#include "eQEP_Encoder.h"
#include "bdib.h"
#include "drive.h"
#include "Fault.h"
#include "Variable.h"
#include "SCI.h"
#include "vane.h"
#include "FRAM.h"
#include "comp.h"
#include "KeyMenu.h"
//#include "fc.h"

SC 		INV1_SC, INV2_SC, INV1_SC_FF;
PC 		INV1_PC;
PROFILE INV1_PROFILE;

extern BDIB g_BDIB;
extern STATUS_ARM_OBJ  g_SpiArmStatus;
extern STATUS_DSP_OBJ  g_SpiDspStatus;
extern Flt 	FLT;
extern Uint16 uDrvST;
//extern ENC_DATA_OBJ ENCA_Data_SC;
extern ENC_DATA_OBJ ENCA_Data;

Uint16 SC_ExTimeChk_Flg = 0;
Uint16 DLA_ErrCnt1 = 0, DLA_ErrCnt2 = 0;	// FC에서 사용 중
float delta_DecI = 0.;	// drive에서 사용 중

//******************For Freeway**********************//
float Time_PC = 0.;
float wsc1 = 2.0*3.141592*50.0;
float wsc2 = 2.0*3.141592*50.0;
float wpi  = 10;
float test_Kt 			= 0.;
float Start_Gain_Cnt 	= 0.;
float TimePC_Step		= 0.;
//***************************************************//
u8 u8PosChCnt = 0;
u8 u8Profile_Deleay = 0;

float Test_SC_Flag = 0;
u8 Spic_SC_Err_Cnt = 0;
u8 u8ProfileStatus = 0;

#pragma CODE_SECTION(cpu1_sc, "ramfuncs");
__interrupt void cpu1_sc(void)
{
	IER &= 0x0000;
	IER |=  M_INT2 | M_INT3 | M_INT5 ;				// Fault, CC, Vdc_Control int Enable
	EINT;
	//TEST3_ON;
	/* USER CODE START */
	/* Position & Speed Calculation */
//	if(DMA_Start_CNT >= 15000 && DmaRegs.CH4.CONTROL.bit.RUNSTS == 0){
//	    StartDMACH3();                          // Start SPI RX DMA channel
//	    asm(" NOP");
//        StartDMACH4();                          // Start SPI TX DMA channel
//
//        /* If first rdata is not STX then Encoder A will be done Watchdog reset */
//        /* And position values are not exceeded 593.75mm */
//        if(ENCA_Data_SC.rdata[0] == STX && ENCA_Data_SC.rdata[9] == ETX && (ENCA_Data_SC.positionfA < 600 || ENCA_Data_SC.positionfB < 600))
//        {
//            u8Spic_SC_Comm_Chk = 1;
//            ENCA_Data_SC.sdata[9] = 0x06;           // Communication of between DSP and Encoder BD is Okay...!!
//        }
//        else
//        {
//            u8Spic_SC_Comm_Chk = 0;
//            ENCA_Data_SC.sdata[9] = 0x04;           // Communication Fault Check
//            Spic_SC_Err_Cnt++;
//            if(g_Id43Data.u8RxData.u8EncoderType == __HEIDENHAINMODE && Spic_SC_Err_Cnt >= 65000){
//                Spic_SC_Err_Cnt = 65001;
//                ENCA_Data_SC.sdata[9] = 0x07;       // Communication Fault
//                FLT_Raise(FLT_UVW);
//            }
//        }
//	}
//    else ENCA_Data_SC.sdata[9] = 0x05;              // Communication Start
//
//    if(u8Spic_SC_Comm_Chk == 1)
//    {
//        Calc_Hiedenhain_POS_SPD(&ENCA_Data_SC);
//    }

	Calc_Enc_Pos_Spd(&Encoder_HD);
	/* Kt & Gain tuning*/
	Controller_Gain_Kt_Tuning();

	if((KNOW_ANGLE == 1)&&(Flags.Align1 == 0)){
		INV1_CC.Idse_Ref = 0.;
		if((uDrvST&DRV_SC_FLG)&&(Flags.Ready1 == 1)){
			Flags.Speed_control_1 = 1;
		}
		else{
			Flags.Speed_control_1 = 0;
		}
	}
	else{
		if(Flags.Ready1 == 1){
			if((uDrvST&DRV_SC_FLG)) Flags.Speed_control_1 = 1;
			else Flags.Speed_control_1 = 0;
		}
	}

	/* Check the Profile use or not */
	if(Flags.ProfileMode)	// Auto, Manual Mode
	{
//	    BK_RELEASE();
	    if((KNOW_ANGLE == 1)&&(g_BDIB.regStatusBit.XBKAS == 1)&&(g_BDIB.regStatusBit.XBKBS == 1)){
	        u8Profile_Deleay++;
	        if(u8Profile_Deleay >= 1000){       // 1.0sec delay
	            u8Profile_Deleay = 1001;
	            Flags.PosCon = 1;
	            PosProfileGeneration(&INV1_PROFILE, &INV1_PC);
	            INV1_PC.PosRefFinal = __fsat(INV1_PC.PosRefFinal, INV1_PC.MaxLength, -INV1_PC.MaxLength);       // Position limit 2.3m
	        }
	    }
	    else{
	        u8Profile_Deleay = 0;
	        Flags.PosCon = 0;
	    }
	}
	else					// Stop, Step Mode
	{
//	    BK_RELEASE();
	    if((KNOW_ANGLE == 1)&&(g_BDIB.regStatusBit.XBKAS == 1)&&(g_BDIB.regStatusBit.XBKBS == 1)){
	        u8Profile_Deleay++;
            if(u8Profile_Deleay >= 1000){       // 1.0sec delay
                u8Profile_Deleay = 1001;
                Flags.PosCon = 1;
                Pos_StepMode();
                INV1_PC.PosRefFinal = INV1_PROFILE.POS_CMD;
                INV1_PC.PosRefFinal = __fsat(INV1_PC.PosRefFinal, INV1_PC.MaxLength, -INV1_PC.MaxLength);       // Position limit 2.3m
            }
	    }
	    else{
	        u8Profile_Deleay = 0;
	        Flags.PosCon = 0;
	    }
	}

	//----------------------------------------------------------INV1----------------------------------------------------------//
		if((Flags.Run1 == 1)||(SC_ExTimeChk_Flg == 1)){
			if((Flags.Ready1 == 1)||(SC_ExTimeChk_Flg == 1)){
				if((Flags.Speed_control_1==1)||(SC_ExTimeChk_Flg == 1)){
					Test_SC_Flag = 1;
				/* Position Controller */
					Position_control(&INV1_PC);

					if(Flags.Position_control)	INV1_PC.Spd_ref = INV1_PC.Spd_ref;
					else						INV1_PC.Spd_ref = INV1_PROFILE.SpeedRef;

					INV1_SC.Wrm_ref = __fsat(INV1_PC.Spd_ref,INV1_PC.Spd_ref_max, -INV1_PC.Spd_ref_max);

					/* Speed Controller */
					Speed_control(&INV1_SC);
					INV1_CC.Idse_Ref = 0.;
				}
				else{
					ResetSC(&INV1_SC);
					ResetPC(&INV1_PC);
				}

				if(Flags.Pos_Con_Kt_mode == 0) 	INV1_SC.Te_ref1 = INV1_SC.Te_ref*INV1.Kt;
				else							INV1_SC.Te_ref1 = INV1_SC.Te_ref*INV1.Kt +  INV1_SC.SCFF_Gain*(INV1_PROFILE.AccRef * INV1.Mass);

				INV1_SC.Te_real  = __fsat(INV1_SC.Te_ref1,INV1_SC.Te_ref_max,-INV1_SC.Te_ref_max);
				INV1_CC.Iqse_Ref = (INV1_SC.Te_real)*INV1.InvKt;
				INV1_CC.Iqse_Ref = __fsat(INV1_CC.Iqse_Ref,INV1_SC.Iqse_ref_max,-INV1_SC.Iqse_ref_max);

			}
			else{
				INV1_CC.Iqse_Ref = 0.;
				INV1_SC.Te_ref_integ = 0.;
				INV1_SC.Te_ref =0.;
				INV1_PC.Spd_ref = 0.;
			}
		}
		else {
			INV1_CC.Idse_Ref = 0.;
			INV1_CC.Iqse_Ref = 0.;
			INV1_SC.Wrpm_ref = 0.;
			ResetSC(&INV1_SC);
			ResetPC(&INV1_PC);
			Test_SC_Flag = 0;
		} // End of Flags.Run1
	/* Speed Fault Check */
	if(fabs(Encoder_HD.speed_elec) > OVER_SP_set)
	{
		FLT_Raise(FLT_OS);			// Over Speed Fault
	}
	/* Speed Control Fault Check */
	if(fabs(INV1_SC.Err_Wrm) > INV1_PROFILE.SpeedMax*0.5)
	{
	    u8SpdFaultCnt++;
	    if(u8SpdFaultCnt >= 1000){
//            FLT_Raise(FLT_SPDCTL);      // Speed Control Fault
	        u8SpdFaultCnt = 0;
	    }
	}
	/* USER CODE END   */
	//TEST3_OFF;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
	asm(" NOP");
	PieCtrlRegs.PIEIER8.bit.INTx1 = 0x01;//I2CA (drive)
	PieCtrlRegs.PIEIER12.bit.INTx1 = 0x01;//vane
}

void SC_VariUpdate(void)
{
	FWD_Dir 		= (Uint16)(KEY_GetControlCode(FWD_DIRECTION) + 0.5);
	OVER_SP_set 	= KEY_GetFactoryCode(OS_SET);
	
	EncPPR 		= 4294967296;

	InitEnc(&Encoder_HD, (Uint32)EncPPR,(float)INV1.PolePair, 0, &EQep2Regs);	// Encoder Variables Initialization

	/* Position Controller & Speed Controller Gain Calculation */
	InitSCVars(&INV1_SC,0.,INV1.Te_max,INV1.Iq_max,TWOPI*Wsc);
//	UpdateSCGains(&INV1,&INV1_SC);
	/* Kt & Gain tuning*/
	Controller_Gain_Kt_Tuning();
	INV1_SC.SCFF_Gain = KEY_GetControlCode(SC_FF_GAIN);

	g_u32MAX_FLOOR	= (u32)KEY_GetControlCode(MAX_FLOOR);
	g_SpiArmStatus.u8MaxFloor[0] = ((g_u32MAX_FLOOR >> 8) & 0x0F) | 0x30;
	g_SpiArmStatus.u8MaxFloor[1] = ((g_u32MAX_FLOOR >> 4) & 0x0F) | 0x30;
	g_SpiArmStatus.u8MaxFloor[2] = ((g_u32MAX_FLOOR >> 0) & 0x0F) | 0x30;
}

void ResetSC(SC *tmpSC)
{
	tmpSC->Te_ref_integ = 0.;
	tmpSC->Te_ref =0.;
	//tmpSC->Wrpm_ref = tmpSC->Wrpm ;//??
	//tmpSC->Wr_ref = tmpSC->Wr;//??
	tmpSC->Wrpm_ref = 0. ;
	tmpSC->Wr_ref = 0.;
}
void InitSCVars(SC *tmpS, float slope, float TeMax, float IqMax, float Wc)
{
	tmpS->Wrm = 0; tmpS->Wr= 0; tmpS->Wrpm= 0;
	tmpS->Wrm_ref=0 ; tmpS->Wr_ref=0 ; tmpS->Wrpm_ref=0 ; tmpS->Wrpm_ref_set=0;
	tmpS->Te_ref_fb = 0 ; tmpS->Te_ref_ff = 0;
	tmpS->Te_ref= 0; tmpS->Te_ref_integ = 0; tmpS->Te_real=0; tmpS->Te_Anti=0; tmpS->Err_Wrm = 0;
	tmpS->Ki_sc = 0; tmpS->Ki_scT=0 ; tmpS->Kp_sc=0; tmpS->Ka_sc=0; tmpS->alpha_sc=1. ;
	tmpS->RefSlope = slope;// rpm/sec
	tmpS->Te_ref_max = TeMax; tmpS->Iqse_ref_max = IqMax;
	tmpS->Wc_sc = Wc;	tmpS->Te_ref1= 0.;	tmpS->Wrpm_ref_old = 0.;	tmpS->Acc_ref = 0.;
	tmpS->Spd = 0.; tmpS->Spd_d1 = 0.;
}
void UpdateSCGains(Motor *tmpM, SC *tmpS)
{
	tmpS->Kp_sc = tmpM->Jm * tmpS->Wc_sc; //500.0
	//tmpS->Ki_sc = 0.5*tmpS->Kp_sc * tmpS->Wc_sc;
	tmpS->Ki_sc = 0.2*tmpS->Kp_sc * tmpS->Wc_sc;//500.0
	tmpS->Ka_sc = 1./ tmpS->Kp_sc;
	tmpS->Ki_scT = tmpS->Ki_sc*Tsc;
}
void Speed_control(SC *tmpS)
{
	//----------Speed control--------------------------------//
	//tmpS->Wrm_ref = Rpm2Rm * tmpS->Wrpm_ref;
	tmpS->Err_Wrm = tmpS->Wrm_ref - tmpS->Wrm;
	tmpS->Te_Anti = tmpS->Te_ref + tmpS->Te_ref_ff - tmpS->Te_real;
	tmpS->Te_ref_integ += tmpS->Ki_scT * (tmpS->Err_Wrm - tmpS->Ka_sc * tmpS->Te_Anti);

	tmpS->Te_ref = tmpS->Te_ref_integ + (tmpS->alpha_sc * tmpS->Kp_sc) * tmpS->Err_Wrm; //PI controller
	tmpS->Te_ref -= ((1.-tmpS->alpha_sc) *tmpS->Kp_sc) * tmpS->Wrm; //IP controller
	//----------Speed control--------------------------------//
}
void Init_SC(void)
{
	// Speed Controller Variables Initialization
	INV1_SC.Acc_ref = 0.;
	INV1_SC.Err_Wr = 0.;
	INV1_SC.Err_Wrm = 0.;
	INV1_SC.Iqse_ref_max = INV1.Iq_max;
	INV1_SC.Spd = 0.;
	INV1_SC.Spd_d1 = 0.;
	INV1_SC.Te_Anti = 0.;
	INV1_SC.Te_real = 0.;
	INV1_SC.Te_ref = 0.;
	INV1_SC.Te_ref1 = 0.;
	INV1_SC.Te_ref_fb = 0.;
	INV1_SC.Te_ref_ff = 0.;
	INV1_SC.Te_ref_integ = 0.;
	INV1_SC.Te_ref_max = INV1.Te_max;
	INV1_SC.Wc_sc = 0.;
	INV1_SC.Wr = 0.;
	INV1_SC.Wr_ref = 0.;
	INV1_SC.Wrm = 0.;
	INV1_SC.Wrm_ref = 0.;
	INV1_SC.alpha_sc = 0.;
	INV1_SC.Ka_sc = 0.;
	INV1_SC.Ki_sc = 0.;
	INV1_SC.Kp_sc = 0.;
	INV1_SC.RefSlope = 0.;
	// Position Controller Variables Initialization
	INV1_PC.Err_Pos = 0.;
	INV1_PC.Ki_pc = 0.;
	INV1_PC.Kp_pc = 0.;
	INV1_PC.PCtime = 0.;
	INV1_PC.POS_CMD = 0.;
	INV1_PC.POS_CMD_d1 = 0.;
	INV1_PC.Pos = 0.;
	INV1_PC.PosRefFinal = 0.;
	INV1_PC.Pos_d1 = 0.;
	INV1_PC.SignProfile = 0;
	INV1_PC.Remain_Distance = 0;
}
///////////////////////////////////////
void PC_VariUpdate(void)
{
	/* Position Controller & Speed Controller Gain Calculation */
	INV1_PC.TimeStep_period 	= 9.0;			// unit: [sec] -> Step mode일때 Step period
	INV1_PC.time_period 		= 9.0;			// unit: [sec]
	INV1_PC.Goal 				= 2.3;			// unit: [m]
	INV1_PC.Init 				= 0.0;			// unit: [m]
	INV1_PC.TimeStep 			= 5.0;			// unit: [sec]
	INV1_PC.PosStep 			= 0.1;			// unit: [m]

	INV1_PC.u32MaxFloor         = g_u32MAX_FLOOR;   // unit: [floor]
	INV1_PC.MaxLength           = KEY_GetControlCode(MAX_LENGTH)*0.001;         // [mm] --> [m]
	INV1_PC.Pos_Offset          = KEY_GetControlCode(CURRENT_POS);

	InitPCVars(&INV1_PC, INV1.MaxSpeed);
	UpdatePCGains();
}

void ResetPC(PC *tmpPC)
{
	tmpPC->Err_Pos = 0.;
	tmpPC->Spd_ref = 0.;
}
void InitPCVars(PC *tmpP, float SpdMax)
{
	tmpP->Err_Pos = 0; tmpP->PCtime = 0; tmpP->POS_CMD = 0;
	tmpP->POS_CMD_d1 = 0; tmpP->PosRefFinal = 0; tmpP->Pos = 0; tmpP->Spd_ref = 0.;
	tmpP->Pos_d1 = 0; tmpP->SignProfile = 0; tmpP->Kp_pc = 0; tmpP->Ki_pc = 0;
	tmpP->Spd_ref_max = SpdMax;
}
void UpdatePCGains(void)
{
	INV1_PC.Kp_pc = TWOPI*KEY_GetFactoryCode(WPC)*0.111111111;
	INV1_PC.Ki_pc = 0.;
}
void Position_control(PC *tmpP)
{
	u8PosChCnt++;
	if(Flags.PosCon_with_APS && u8PosChCnt >= 100){
		u8PosChCnt = 101;
		tmpP->Pos = tmpP->APSPos;
	}
	else tmpP->Pos = tmpP->Pos;

	//----------Position control--------------------------------//
	tmpP->Err_Pos = tmpP->PosRefFinal - tmpP->Pos;
	tmpP->Spd_ref = tmpP->Kp_pc * tmpP->Err_Pos + INV1_PROFILE.SpeedRef;
	//----------Position control--------------------------------//
}
////////////////////////////////////////

void Init_PROFILE(void)
{
	// PosProfileGenerationc
	INV1_PROFILE.a 					= 0.;
	INV1_PROFILE.b 					= 0.;
	INV1_PROFILE.c 					= 0.;
	INV1_PROFILE.POS_Ref			= 0.;
	INV1_PROFILE.AccMax 			= KEY_GetControlCode(MAX_ACC);     		// 최대 가속도 설정
	INV1_PROFILE.SpeedMax 			= KEY_GetControlCode(MAX_SPEED);    	// 최대 속도 설정
	INV1_PROFILE.SpeedMax_short 	= 0.;									// Short Profile 최대 속도 계산
	INV1_PROFILE.Sratio 			= 0.15;      							// 가속 구간중 가가속 구간 비율 설정  t1/(2*t1 + tp)
	INV1_PROFILE.PosDistance 		= 0.;
	INV1_PROFILE.PosRefc 			= 0.;
	INV1_PROFILE.Jerk 				= 0.;
	INV1_PROFILE.JerkMax 			= KEY_GetControlCode(MAX_JERK);			// 최대 저크 설정
	INV1_PROFILE.JerkTime 			= 0.;									// 저크 시간 계산: AccMax/JerkMax
	INV1_PROFILE.Acc_time_total 	= 0.;
	INV1_PROFILE.Jerk_time_total 	= 0.;
	INV1_PROFILE.Acc_time_mid 		= 0.;
	INV1_PROFILE.V_time_mid			= 0.;
	INV1_PROFILE.V_time_total 		= 0.;
	INV1_PROFILE.Second_Hmin 		= 0.;

	INV1_PROFILE.iAcc_time_total	= 0;
	INV1_PROFILE.iJerk_time_total	= 0;
	INV1_PROFILE.iAcc_time_mid		= 0;
	INV1_PROFILE.iV_time_mid		= 0;
	INV1_PROFILE.iV_time_total		= 0;

    INV1_PROFILE.t1                 = INV1_PROFILE.AccMax/INV1_PROFILE.JerkMax;             // 가가속 구간 시간 계산
    INV1_PROFILE.tp                 = INV1_PROFILE.SpeedMax/INV1_PROFILE.AccMax-INV1_PROFILE.JerkTime;              // 정가속 구간 시간 계산


    INV1_PROFILE.A1                 = INV1_PROFILE.AccMax/INV1_PROFILE.t1;
    INV1_PROFILE.A2                 = INV1_PROFILE.AccMax + INV1_PROFILE.A1*(INV1_PROFILE.t1+INV1_PROFILE.tp);
    INV1_PROFILE.A3                 = -0.5*INV1_PROFILE.A1*INV1_PROFILE.t1*INV1_PROFILE.t1;
    INV1_PROFILE.A4                 = INV1_PROFILE.A1*INV1_PROFILE.t1*INV1_PROFILE.t1 + INV1_PROFILE.AccMax*INV1_PROFILE.tp + INV1_PROFILE.A3 + 0.5*INV1_PROFILE.A1*(INV1_PROFILE.t1+INV1_PROFILE.tp)*(INV1_PROFILE.t1+INV1_PROFILE.tp) - INV1_PROFILE.A2*(INV1_PROFILE.t1+INV1_PROFILE.tp);
    INV1_PROFILE.A5                 = -INV1_PROFILE.A1/3.0*INV1_PROFILE.t1*INV1_PROFILE.t1*INV1_PROFILE.t1 - INV1_PROFILE.A3*INV1_PROFILE.t1;
    INV1_PROFILE.A6                 =  0.5*INV1_PROFILE.AccMax*(INV1_PROFILE.t1+INV1_PROFILE.tp)*(INV1_PROFILE.t1+INV1_PROFILE.tp) + INV1_PROFILE.A3*(INV1_PROFILE.t1+INV1_PROFILE.tp) + INV1_PROFILE.A5 + INV1_PROFILE.A1/6.0*(INV1_PROFILE.t1+INV1_PROFILE.tp)*(INV1_PROFILE.t1+INV1_PROFILE.tp)*(INV1_PROFILE.t1+INV1_PROFILE.tp) - 0.5*INV1_PROFILE.A2*(INV1_PROFILE.t1+INV1_PROFILE.tp)*(INV1_PROFILE.t1+INV1_PROFILE.tp) - INV1_PROFILE.A4*(INV1_PROFILE.t1+INV1_PROFILE.tp);;
	INV1_PROFILE.A7					= 0.;
	INV1_PROFILE.A8					= 0.;
	INV1_PROFILE.A9					= 0.;
	INV1_PROFILE.A10				= 0.;
	INV1_PROFILE.SpeedRef_d1 		= 0.;
	INV1_PROFILE.PosRef_d1 			= 0.;
	INV1_PROFILE.AccRef_d1 			= 0.;
	INV1_PROFILE.Jerk_d1			= 0.;

	INV1_PROFILE.tc                 = 0.;               // 0 가속구간 시간 계산

    INV1_PROFILE.iA1                = 0;
    INV1_PROFILE.iA2                = 0;
    INV1_PROFILE.iA3                = 0;
    INV1_PROFILE.iA4                = 0;
    INV1_PROFILE.iA5                = 0;
    INV1_PROFILE.iA6                = 0;
    INV1_PROFILE.iA7                = 0;
    INV1_PROFILE.iA8                = 0;
    INV1_PROFILE.iA9                = 0;
    INV1_PROFILE.iA10               = 0;

	INV1_PROFILE.Tprofile 			= 0.;

	INV1_PROFILE.AccRef				= 0.;
	INV1_PROFILE.SpeedRef			= 0.;
	INV1_PROFILE.SignProfile 		= 1.;

	INV1_PROFILE.POS_CMD 			= INV1_PC.Pos;
	INV1_PROFILE.POS_CMD_old 		= INV1_PC.Pos;
	INV1_PROFILE.POS_CMD_PRESENT 	= 0.;

	INV1_PROFILE.temp_cal			= 0.;

	INV1_PROFILE.TCH 				= 1.;
	INV1_PROFILE.TCH_d1				= 1.;
	INV1_PROFILE.PROFILE 			= 0.;
	INV1_PROFILE.test_profile 		= 0.;

	INV1_PROFILE.PosRefFinal		= INV1_PC.Pos;

	INV1_PROFILE.ProfileGen_Flag	= 0;

	INV1_PROFILE.sc_cnt				= 0;
	INV1_PROFILE.PosRef				= INV1_PC.Pos;
	
	INV1_PROFILE.Stop_Time			= 3.;		// Stop Ready Time: 2sec;

	INV1_PROFILE.Err_Profile		= 0.;
	INV1_PROFILE.Comp_Profile		= INV1_PC.Pos;

	INV1_PROFILE.CreepSpeed			= KEY_GetControlCode(CREEP_SPEED);
	INV1_PROFILE.InspectSpeed		= KEY_GetControlCode(INSPECT_SPEED);
	INV1_PROFILE.RelevelSpeed       = KEY_GetControlCode(RELEVEL_SPEED);

	INV1_PROFILE.PH[0]              = 0;
	INV1_PROFILE.PH[1]              = 0;
	INV1_PROFILE.PH[2]              = 0;
	INV1_PROFILE.PH[3]              = 0;
	INV1_PROFILE.PH[4]              = 0;
	INV1_PROFILE.PH[5]              = 0;
	INV1_PROFILE.PH[6]              = 0;

	INV1_PROFILE.Profile_Status     = 0;        // PH1: 1, PH2: 2, PH3: 3, PH4: 4, PH5: 5; PH6: 6, PH7: 7

	Flags.CompProfile               = 0;

//	KEY_SetInterfaceCode(MAN_POS_CMD,0.0);		// Initial Setting of Position Command with HHT
}

/********************************************************/
/*	 Profile Generation Function	 					*/
/* Author: Gogume & KERI								*/
/* History: 20180322 init								*/
/*        : 20180426 Modified 							*/
/********************************************************/
void PosProfileGeneration(PROFILE *tmpPRO, PC *tmpPC)
{
    if(tmpPRO->POS_CMD != tmpPRO->POS_CMD_old)
    {
    	tmpPRO->PosDistance = tmpPRO->POS_CMD - tmpPRO->POS_CMD_old;			// For Direction and Moving Distance
    	tmpPRO->JerkTime = tmpPRO->AccMax/tmpPRO->JerkMax;					    // Added for FREEWAY 20180213 by GWON

		/* for Long Stroke */
//	    t1 = Sratio*SpeedMax/((1.0 - Sratio)*AccMax);
//		tp = (1.0-2.0*Sratio)*t1/Sratio;
    	tmpPRO->t1      = tmpPRO->AccMax/tmpPRO->JerkMax;				        // Modified for FREEWAY 20180213 by GWON
    	tmpPRO->tp      = tmpPRO->SpeedMax/tmpPRO->AccMax-tmpPRO->JerkTime;	    // Modified for FREEWAY 20180213 by GWON

    	tmpPRO->Sratio  = tmpPRO->t1/(2*tmpPRO->t1+tmpPRO->tp);			        // Modified for FREEWAY 20180213 by GWON --> S-curve Ratio Calculate

    	tmpPRO->A1 = tmpPRO->AccMax/tmpPRO->t1;
    	tmpPRO->A2 = tmpPRO->AccMax + tmpPRO->A1*(tmpPRO->t1+tmpPRO->tp);
    	tmpPRO->A3 = -0.5*tmpPRO->A1*tmpPRO->t1*tmpPRO->t1;
    	tmpPRO->A4 = tmpPRO->A1*tmpPRO->t1*tmpPRO->t1 + tmpPRO->AccMax*tmpPRO->tp + tmpPRO->A3 + 0.5*tmpPRO->A1*(tmpPRO->t1+tmpPRO->tp)*(tmpPRO->t1+tmpPRO->tp) - tmpPRO->A2*(tmpPRO->t1+tmpPRO->tp);
    	tmpPRO->A5 = -tmpPRO->A1/3.0*tmpPRO->t1*tmpPRO->t1*tmpPRO->t1 - tmpPRO->A3*tmpPRO->t1;
    	tmpPRO->A6 = 0.5*tmpPRO->AccMax*(tmpPRO->t1+tmpPRO->tp)*(tmpPRO->t1+tmpPRO->tp) + tmpPRO->A3*(tmpPRO->t1+tmpPRO->tp) + tmpPRO->A5 + tmpPRO->A1/6.0*(tmpPRO->t1+tmpPRO->tp)*(tmpPRO->t1+tmpPRO->tp)*(tmpPRO->t1+tmpPRO->tp) - 0.5*tmpPRO->A2*(tmpPRO->t1+tmpPRO->tp)*(tmpPRO->t1+tmpPRO->tp) - tmpPRO->A4*(tmpPRO->t1+tmpPRO->tp);
//    	tmpPRO->A6 = -2/3*tmpPRO->A1*(tmpPRO->t1+tmpPRO->tp)*(tmpPRO->t1+tmpPRO->tp)*(tmpPRO->t1+tmpPRO->tp) + tmpPRO->A3*(tmpPRO->t1+tmpPRO->tp) - tmpPRO->A4*(tmpPRO->t1+tmpPRO->tp) + tmpPRO->A5;

    	tmpPRO->PosRefc = -tmpPRO->A1/6.0*(2.0*tmpPRO->t1+tmpPRO->tp)*(2.0*tmpPRO->t1+tmpPRO->tp)*(2.0*tmpPRO->t1+tmpPRO->tp) + 0.5*tmpPRO->A2*(2.0*tmpPRO->t1+tmpPRO->tp)*(2.0*tmpPRO->t1+tmpPRO->tp) + tmpPRO->A4*(2*tmpPRO->t1+tmpPRO->tp) + tmpPRO->A6;

        if(tmpPRO->PosDistance > 0.0)   tmpPRO->SignProfile = 1.0;
        else if(tmpPRO->PosDistance < 0.0) tmpPRO->SignProfile = -1.0;

        tmpPRO->Second_Hmin = 2*tmpPRO->PosRefc;

        tmpPRO->PosRefc = tmpPRO->SignProfile*tmpPRO->PosRefc;

        tmpPRO->temp_cal = tmpPRO->PosDistance - 2.0*tmpPRO->PosRefc;
        tmpPRO->tc = fabs(tmpPRO->temp_cal)/tmpPRO->SpeedMax;

        if((fabs(tmpPRO->PosDistance) > tmpPRO->Second_Hmin)) tmpPRO->ProfileGen_Flag = 1;	// Modified for FREEWAY 20180222 by GWON
        else tmpPRO->ProfileGen_Flag = 0;
        tmpPRO->PROFILE = tmpPRO->ProfileGen_Flag;

        tmpPRO->A8  = tmpPRO->A1*(2*tmpPRO->t1+tmpPRO->tp+tmpPRO->tc);
        //A9        = SpeedMax + A1/2*(2*t1+tp+tc)^2 - A8*(2*t1+tp+tc);
        tmpPRO->A10 = -tmpPRO->AccMax - tmpPRO->A1*(3*tmpPRO->t1+2*tmpPRO->tp+tmpPRO->tc);

        tmpPRO->sc_cnt              = 0;
        tmpPRO->Tprofile            = 0.0;
        tmpPRO->POS_CMD_old         = tmpPRO->POS_CMD;
        tmpPRO->PosDistance_old     = tmpPRO->PosDistance;
        tmpPRO->TCH                 = 0.0;
        g_SpiArmStatus.u8VoiceFlg   = 0x00;
        tmpPRO->SpeedMax_short      = 0.0;

        tmpPRO->Minimized_distance  = 0.9e-6;     // Minimized Moving Distance that is one floor hight(1um)         /// modified 20190225 need to verify
//        tmpPRO->Minimized_distance = 1.6;       // Minimized Moving Distance that is one floor hight(1.6m)        /// modified 20190225 need to verify

		/* for Short Stroke */
        if(tmpPRO->ProfileGen_Flag == 0 && (tmpPRO->SignProfile*tmpPRO->PosDistance > tmpPRO->Minimized_distance))		                // Minimized Moving Distance that is one floor hight(1um)   /// modified 20190225 need to verify
        {
         	tmpPRO->a                  = 1;
        	tmpPRO->b                  = -tmpPRO->JerkTime;
        	tmpPRO->c                  = __divf32(-(tmpPRO->SignProfile*tmpPRO->PosDistance), tmpPRO->AccMax);
        	tmpPRO->Acc_time_total     = (__divf32((-tmpPRO->b+__sqrt(tmpPRO->b*tmpPRO->b-4*tmpPRO->a*tmpPRO->c)), (2*tmpPRO->a)));	    // Time Compansation
        	tmpPRO->Jerk_time_total    = tmpPRO->JerkTime;
        	tmpPRO->Acc_time_mid       = (tmpPRO->Acc_time_total - 2*tmpPRO->Jerk_time_total);
        	tmpPRO->V_time_mid         = 0.;
        	tmpPRO->V_time_total       = 2*tmpPRO->Acc_time_total;
        	tmpPRO->SpeedMax_short     = __divf32((tmpPRO->SignProfile*tmpPRO->PosDistance), (tmpPRO->V_time_total*0.5));

        	tmpPRO->iAcc_time_total    = (Uint32)(tmpPRO->Acc_time_total*1000);
        	tmpPRO->iJerk_time_total   = (Uint32)(tmpPRO->Jerk_time_total*1000);
        	tmpPRO->iAcc_time_mid      = (Uint32)(tmpPRO->Acc_time_mid*1000);
        	tmpPRO->iV_time_mid        = (Uint32)(tmpPRO->V_time_mid*1000);
        	tmpPRO->iV_time_total      = (Uint32)(tmpPRO->V_time_total*1000);

        	tmpPRO->PH[0]               = tmpPRO->iJerk_time_total;
        	tmpPRO->PH[1]               = tmpPRO->iAcc_time_total-tmpPRO->iJerk_time_total;
        	tmpPRO->PH[2]               = tmpPRO->iAcc_time_total;
        	tmpPRO->PH[3]               = tmpPRO->iAcc_time_total;
        	tmpPRO->PH[4]               = tmpPRO->iAcc_time_total+tmpPRO->iJerk_time_total;
        	tmpPRO->PH[5]               = 2*tmpPRO->iAcc_time_total - tmpPRO->iJerk_time_total;
        	tmpPRO->PH[6]               = 2*tmpPRO->iAcc_time_total;
		}
        else
        {
            tmpPRO->PH[0]              = (Uint32)(tmpPRO->t1*1000);
            tmpPRO->PH[1]              = tmpPRO->PH[0] + (Uint32)(tmpPRO->tp*1000);
            tmpPRO->PH[2]              = tmpPRO->PH[1] + tmpPRO->PH[0];
            tmpPRO->PH[3]              = tmpPRO->PH[2] + (Uint32)(tmpPRO->tc*1000);
            tmpPRO->PH[4]              = tmpPRO->PH[3] + tmpPRO->PH[0];
            tmpPRO->PH[5]              = tmpPRO->PH[4] + (Uint32)(tmpPRO->tp*1000);
            tmpPRO->PH[6]              = tmpPRO->PH[5] + tmpPRO->PH[0];
        }
    }

    if(tmpPRO->ProfileGen_Flag == 1 && tmpPRO->SignProfile*tmpPRO->PosDistance > tmpPRO->Second_Hmin) 				                                    // Modified for FREEWAY 20180213 by GWON
    {
    	// Total Profile Time Calculation
    	tmpPC->time_period = tmpPRO->Stop_Time + 4.0*tmpPRO->t1+2.0*tmpPRO->tp+tmpPRO->tc;

        if(tmpPRO->sc_cnt == 0)
        {
            u8ProfileStatus = 0;
            tmpPRO->Jerk      = 0;
            tmpPRO->AccRef    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
//            tmpPRO->AccRef      = 0;
            tmpPRO->SpeedRef    = Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
            tmpPRO->PosRef      = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1; // + POS_CMD_PRESENT;
//            tmpPRO->Jerk        = (tmpPRO->AccRef - tmpPRO->AccRef_d1)*INV_Tsc;
        }
    	else if(tmpPRO->sc_cnt <= tmpPRO->PH[0])
        {
    	    u8ProfileStatus = 1;
            tmpPRO->Jerk      = tmpPRO->JerkMax*tmpPRO->SignProfile;
            tmpPRO->AccRef    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
//        	tmpPRO->AccRef      = tmpPRO->SignProfile*tmpPRO->A1*tmpPRO->Tprofile;
        	tmpPRO->SpeedRef    = Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef      = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1; // + POS_CMD_PRESENT;
//        	tmpPRO->Jerk        = (tmpPRO->AccRef - tmpPRO->AccRef_d1)*INV_Tsc;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[0])&&(tmpPRO->sc_cnt <= tmpPRO->PH[1]))
        {
            u8ProfileStatus = 2;
            tmpPRO->Jerk      = 0;
            tmpPRO->AccRef    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
//            tmpPRO->AccRef      = tmpPRO->SignProfile*tmpPRO->AccMax;
        	tmpPRO->SpeedRef    = Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef      = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;  // + POS_CMD_PRESENT;
//        	tmpPRO->Jerk        = (tmpPRO->AccRef - tmpPRO->AccRef_d1)*INV_Tsc;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[1])&&(tmpPRO->sc_cnt <= tmpPRO->PH[2]))
        {
            u8ProfileStatus = 3;
            tmpPRO->Jerk      = -tmpPRO->JerkMax*tmpPRO->SignProfile;
            tmpPRO->AccRef    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
//        	tmpPRO->AccRef      = tmpPRO->SignProfile*(tmpPRO->A2 - tmpPRO->A1*tmpPRO->Tprofile);
        	tmpPRO->SpeedRef    = Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef      = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;  // + POS_CMD_PRESENT;
//        	tmpPRO->Jerk        = (tmpPRO->AccRef - tmpPRO->AccRef_d1)*INV_Tsc;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[2])&&(tmpPRO->sc_cnt <= tmpPRO->PH[3]))
        {
            u8ProfileStatus = 4;
            tmpPRO->Jerk      = 0;
            tmpPRO->AccRef    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
//        	tmpPRO->AccRef      = 0.0;
        	tmpPRO->SpeedRef    = Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef      = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;  // + POS_CMD_PRESENT;
//        	tmpPRO->Jerk        = (tmpPRO->AccRef - tmpPRO->AccRef_d1)*INV_Tsc;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[3])&&(tmpPRO->sc_cnt <= tmpPRO->PH[4]))
        {
            u8ProfileStatus = 5;
            tmpPRO->Jerk      = -tmpPRO->JerkMax*tmpPRO->SignProfile;
            tmpPRO->AccRef    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
//        	tmpPRO->AccRef      = tmpPRO->SignProfile*(tmpPRO->A8 - tmpPRO->A1*tmpPRO->Tprofile);
        	tmpPRO->SpeedRef    = Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef      = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;  // + POS_CMD_PRESENT;
//        	tmpPRO->Jerk        = (tmpPRO->AccRef - tmpPRO->AccRef_d1)*INV_Tsc;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[4])&&(tmpPRO->sc_cnt <= tmpPRO->PH[5]))
        {
            u8ProfileStatus = 6;
            tmpPRO->Jerk      = 0;
            tmpPRO->AccRef    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
//        	tmpPRO->AccRef      = -tmpPRO->SignProfile*tmpPRO->AccMax;
        	tmpPRO->SpeedRef    = Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef      = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;   // + POS_CMD_PRESENT;
//        	tmpPRO->Jerk        = (tmpPRO->AccRef - tmpPRO->AccRef_d1)*INV_Tsc;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[5])&&(tmpPRO->sc_cnt <= tmpPRO->PH[6]))
        {
            u8ProfileStatus = 7;
            tmpPRO->Jerk      = tmpPRO->JerkMax*tmpPRO->SignProfile;
            tmpPRO->AccRef    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
//        	tmpPRO->AccRef      = tmpPRO->SignProfile*(tmpPRO->A1*tmpPRO->Tprofile+tmpPRO->A10);
        	tmpPRO->SpeedRef    = Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef      = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;  // + POS_CMD_PRESENT;
//        	tmpPRO->Jerk        = (tmpPRO->AccRef - tmpPRO->AccRef_d1)*INV_Tsc;
        }
        else
        {
            u8ProfileStatus = 8;
            tmpPRO->Jerk                = 0;
        	tmpPRO->ProfileGen_Flag     = 0;
        	tmpPRO->POS_CMD_PRESENT     = tmpPRO->POS_CMD;
        	tmpPRO->TCH                 = 1.0;
        	g_SpiArmStatus.u8VoiceFlg   = 0x01;
        }
        /* Force Deceleration Sequence */
        if(Flags.Decel == 1)                        // FORCE_DECEL_OP
        {
            if(u8ProfileStatus == 4)
            {
                tmpPRO->sc_cnt = tmpPRO->PH[3];
                tmpPRO->sc_cnt++;
            }
            else tmpPRO->sc_cnt++;
        }
        else tmpPRO->sc_cnt++;

        tmpPRO->Tprofile    = tmpPRO->sc_cnt * Tsc;
        tmpPRO->SpeedRef_d1 = tmpPRO->SpeedRef;
        tmpPRO->PosRef_d1   = tmpPRO->PosRef;
        tmpPRO->AccRef_d1   = tmpPRO->AccRef;
        tmpPRO->Jerk_d1     = tmpPRO->Jerk;
    }
    else if(tmpPRO->ProfileGen_Flag == 0 && tmpPRO->SignProfile*tmpPRO->PosDistance <= tmpPRO->Second_Hmin && (tmpPRO->SignProfile*tmpPRO->PosDistance > tmpPRO->Minimized_distance ))		// Minimized Moving Distance that is one floor hight(1.6m)
    {
    	// Total Profile Time Calculation
    	tmpPC->time_period = tmpPRO->Stop_Time + tmpPRO->V_time_total;

    	if(tmpPRO->sc_cnt == 0)
    	{
    	    u8ProfileStatus = 0;
    		tmpPRO->Jerk 		= 0;
    		tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
    		tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
    		tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
        }
        else if(tmpPRO->sc_cnt <= tmpPRO->PH[0])
        {
            u8ProfileStatus = 1;
        	tmpPRO->Jerk 		= tmpPRO->JerkMax*tmpPRO->SignProfile;
        	tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
        	tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[0])&&(tmpPRO->sc_cnt < tmpPRO->PH[1]))
        {
            u8ProfileStatus = 2;
        	tmpPRO->Jerk 		= 0;
        	tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
        	tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
        }
        else if((tmpPRO->sc_cnt >= tmpPRO->PH[1])&&(tmpPRO->sc_cnt < tmpPRO->PH[2]))
        {
            u8ProfileStatus = 3;
        	tmpPRO->Jerk 		= -tmpPRO->JerkMax*tmpPRO->SignProfile;
        	tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
        	tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
        }
        else if((tmpPRO->sc_cnt == tmpPRO->PH[3]))
        {
            u8ProfileStatus = 4;
        	tmpPRO->Jerk 		= 0;
        	tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
        	tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[3])&&(tmpPRO->sc_cnt <= tmpPRO->PH[4]))
        {
            u8ProfileStatus = 5;
        	tmpPRO->Jerk 		= -tmpPRO->JerkMax*tmpPRO->SignProfile;
        	tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
        	tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
            tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
        }
        else if((tmpPRO->sc_cnt > tmpPRO->PH[4])&&(tmpPRO->sc_cnt < tmpPRO->PH[5]))
        {
            u8ProfileStatus = 6;
        	tmpPRO->Jerk 		= 0;
        	tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
        	tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
        }
        else if((tmpPRO->sc_cnt >= tmpPRO->PH[5])&&(tmpPRO->sc_cnt < tmpPRO->PH[6]))
        {
            u8ProfileStatus = 7;
        	tmpPRO->Jerk 		= tmpPRO->JerkMax*tmpPRO->SignProfile;
        	tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
        	tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
        }
        else if((tmpPRO->sc_cnt == tmpPRO->PH[6]))
    	{
        	tmpPRO->Jerk 		= 0;
        	tmpPRO->AccRef 	    = Tsc*tmpPRO->Jerk_d1 + tmpPRO->AccRef_d1;
        	tmpPRO->SpeedRef 	= Tsc*tmpPRO->AccRef + tmpPRO->SpeedRef_d1;
        	tmpPRO->PosRef 	    = Tsc*tmpPRO->SpeedRef + tmpPRO->PosRef_d1;
//        	tmpPRO->test_profile = tmpPRO->PosRef;
    	}
        else
        {
            u8ProfileStatus = 8;
        	tmpPRO->ProfileGen_Flag     = 0;
        	tmpPRO->Jerk                = 0;
        	tmpPRO->POS_CMD_PRESENT     = tmpPRO->POS_CMD;
        	tmpPRO->TCH                 = 1.0;
        	g_SpiArmStatus.u8VoiceFlg   = 0x01;
        }
        /* Force Deceleration Sequence */
    	if(Flags.Decel == 1)                        // FORCE_DECEL_OP
        {
            if(u8ProfileStatus == 4)
            {
                tmpPRO->sc_cnt = tmpPRO->PH[3];
                tmpPRO->sc_cnt++;
            }
            else tmpPRO->sc_cnt++;
        }
    	else tmpPRO->sc_cnt++;

    	tmpPRO->Tprofile    = tmpPRO->sc_cnt * Tsc;
    	tmpPRO->Jerk_d1     = tmpPRO->Jerk;
    	tmpPRO->AccRef_d1   = tmpPRO->AccRef;
    	tmpPRO->SpeedRef_d1 = tmpPRO->SpeedRef;
    	tmpPRO->PosRef_d1   = tmpPRO->PosRef;
    }
    else
    {
    	tmpPRO->PosRef = tmpPRO->PosRef;
    }
    
    if(Flags.CompProfile)
    {
		/* Compensation of Profile Calculate Error */
		if(tmpPRO->TCH != tmpPRO->TCH_d1)
		{
			tmpPRO->Err_Profile     = tmpPRO->POS_CMD - tmpPRO->PosRef;
			tmpPRO->Comp_Profile    = tmpPRO->PosRef;
			tmpPRO->TCH_d1          = tmpPRO->TCH;
		}
		if(tmpPRO->TCH)
		{
			COMP_PROFILE(tmpPRO);
			tmpPRO->PosRef = tmpPRO->Comp_Profile;
		}
	}
    /* Applied to Position Command */
    tmpPRO->PosRefFinal = tmpPRO->PosRef;
    tmpPC->PosRefFinal  = tmpPRO->PosRefFinal;
}

void Controller_Gain_Kt_Tuning(void)
{
	switch(Flags.Pos_Con_Kt_mode){
	case 0:		// With out Speed Controller Feedforward term
		INV1.Kt = 1;
		INV1.InvKt = 1/INV1.Kt;
//		INV1.Jm = 30;
		wsc1 = TWOPI*KEY_GetFactoryCode(WSC);
		wsc2 = TWOPI*KEY_GetFactoryCode(WSC);
		wpi = 5;
		break;
	case 1:		// Calculated Kt refered by Paper [1]
		INV1.Kt = 92.48956698;
		INV1.InvKt =0.01081203;
//		INV1.Jm = 100;
		wsc1 = TWOPI*KEY_GetFactoryCode(WSC);
		wsc2 = TWOPI*KEY_GetFactoryCode(WSC);
		wpi = 5;
		break;
	case 2:		// Measurement & Average Kt
		INV1.Kt = 79.52;
		INV1.InvKt = 0.0126;
//		INV1.Jm = 100;
		wsc1 = TWOPI*KEY_GetFactoryCode(WSC);
		wsc2 = TWOPI*KEY_GetFactoryCode(WSC);
		wpi = 5;
		break;
	case 3:		// Measurement Kt at Rated Current
		INV1.Kt = 63.27;
		INV1.InvKt = 0.0158;
//		INV1.Jm = 100;
		wsc1 = TWOPI*KEY_GetFactoryCode(WSC);
		wsc2 = TWOPI*KEY_GetFactoryCode(WSC);
		wpi = 5;
		break;
	case 4:		// Adaptive Kt using Measurement
//		INV1.Jm = 100;
		wsc1 = TWOPI*KEY_GetFactoryCode(WSC);
		wsc2 = TWOPI*KEY_GetFactoryCode(WSC);
		wpi = 5;

		if(Flags.Kt_mode == 0){		// Based on KERI Data
			// Considering FRLSM Nonlinear Chariteristic of thrust.
			if(INV1_CC.Iqse_Ref < 60){		// Nonlinear Region
				test_Kt = Cal_Kt_Func(fabs(INV1_CC.Iqse_Ref), 1, 0);
				INV1.Kt = test_Kt;
				INV1.InvKt = 1/INV1.Kt;
			}
			else{				// Linear Region
				test_Kt = Cal_Kt_Func(fabs(INV1_CC.Iqse_Ref), 0, 0);
				INV1.Kt = test_Kt;
				INV1.InvKt = 1/INV1.Kt;
			}
		}
		else{					// Based on HDEL Data
			test_Kt = Cal_Kt_Func(fabs(INV1_CC.Iqse_Ref), 1, 1);
			INV1.Kt = test_Kt;
			INV1.InvKt = 1/INV1.Kt;
		}
		break;
	default:
		INV1.Kt = 1;
		INV1.InvKt = 1/INV1.Kt;
//		INV1.Jm = 30;
		wsc1 = TWOPI*KEY_GetFactoryCode(WSC);
		wsc2 = TWOPI*KEY_GetFactoryCode(WSC);
		wpi = 5;
		break;
	}

	// Speed Gain Scheduling 20180228 by GWON
	if(INV1_PC.SignProfile > 0)
	{
		if(Start_Gain_Cnt > 10000)							// 1sec
		{
			INV1_SC.Kp_sc  = INV1.Jm * wsc1 *INV1.InvKt;					// Speed Controller P Gain
			INV1_SC.Ki_scT = INV1_SC.Kp_sc*wpi*Tsc;							// Speed Controller I Gain
//			INV1_SC.Ka_sc  = __divf32(1.,INV1_SC.Kp_sc);					// Speed Controller Anti-windup Gain
			Start_Gain_Cnt = 100000;
		}
		else
		{
			INV1_SC.Kp_sc  = INV1.Jm * wsc2 *INV1.InvKt;					// Speed Controller P Gain
			INV1_SC.Ki_scT = INV1_SC.Kp_sc*wpi*Tsc;							// Speed Controller I Gain
//			INV1_SC.Ka_sc  = __divf32(1.,INV1_SC.Kp_sc);					// Speed Controller Anti-windup Gain
			Start_Gain_Cnt++;
		}
	}
	else
	{
		INV1_SC.Kp_sc  = INV1.Jm * wsc1 *INV1.InvKt;						// Speed Controller P Gain
		INV1_SC.Ki_scT = INV1_SC.Kp_sc*wpi*Tsc;								// Speed Controller I Gain
//		INV1_SC.Ka_sc  = __divf32(1.,INV1_SC.Kp_sc);						// Speed Controller Anti-windup Gain
		Start_Gain_Cnt = 0.0;
	}
}


/********************************************************/
/*	Thrust Force Constant Calculation 					*/
/* Author: Gogume										*/
/* History: 20180322 init								*/
/*        : 20180405 Added HDEL data version function 	*/
/********************************************************/
float Cal_Kt_Func(float x, float mode, float mode2)
{

    float p1    = 0.099972;
    float p2    = -1.1539;
    float p3    = 3.8971;
    float p4    = -4.5087;
    float p5    = 3.7818;
    float p6    = 232.98;
    float y     = 0;

    y = p1*x*x*x*x*x + p2*x*x*x*x + p3*x*x*x + p4*x*x + p5*x + p6;

    return  y;
}

/********************************************************/
/*	Load Estimation Function based on Iq				*/
/* Author: Gogume										*/
/* History: 20180406 init								*/
/********************************************************/
float Est_Load_Func(float x, float mode){
	float p1 = -0.057417;
	float p2 = 10.48;
	float p3 = 0.66667;
	float y = 0.;

 	float a 	= 1.152941176;
 	float b 	= 378.4235294;

 	if(mode){	// Nonlinear Region
	 	y = p1*x*x + p2*x + p3;
	}
	else{		// Linear Region
		y = a*x+b;
	}

	return y;
}

/********************************************************/
/*	 Profile Error Compensation Function				*/
/* Author: Gogume										*/
/* History: 20180010 init								*/
/********************************************************/
void COMP_PROFILE(PROFILE *tempPRO)
{
	/* Compensation of Profile Calculate Error */
	if(INV1_PROFILE.Err_Profile > 1e-6)
	{
	    Flags.CompProfileEnd = 0;
		INV1_PROFILE.Comp_Profile += (INV1_PROFILE.Err_Profile - INV1_PROFILE.Err_Profile*0.01)*Tsc;
		if(INV1_PROFILE.Comp_Profile >= INV1_PROFILE.POS_CMD){
		   INV1_PROFILE.Comp_Profile = INV1_PROFILE.POS_CMD;
		   Flags.CompProfileEnd = 1;
		}
	}
	else if(INV1_PROFILE.Err_Profile < -1e-6)
	{
	    Flags.CompProfileEnd = 0;
		INV1_PROFILE.Comp_Profile += (INV1_PROFILE.Err_Profile - INV1_PROFILE.Err_Profile*0.01)*Tsc;
		if(INV1_PROFILE.Comp_Profile <= INV1_PROFILE.POS_CMD){
		    INV1_PROFILE.Comp_Profile = INV1_PROFILE.POS_CMD;
		    Flags.CompProfileEnd = 1;
		}
	}
	else
	{
	    Flags.CompProfileEnd = 1;
		INV1_PROFILE.Comp_Profile = INV1_PROFILE.Comp_Profile;
	}
}


void Pos_StepMode(void)
{
	switch(Flags.Op_Mode)
	{
	case 1:
		//if(Flags.Op_Mode == 1){		// Repetition Mode
			TimePC_Step = TimePC_Step + Tsc;
			if(TimePC_Step<=INV1_PC.TimeStep_period)
			{
//				INV1_PROFILE.POS_CMD = KEY_GetInterfaceCode(MAN_POS_CMD);  //-2.9;			// Need to modify from spi data very important!!!!!!!!!!!!!
			}
			else if((TimePC_Step>INV1_PC.TimeStep_period)&&(TimePC_Step<=2.0*INV1_PC.TimeStep_period))
			{
				INV1_PROFILE.POS_CMD = INV1_PC.Init+0.1;
			}
			else
			{
				TimePC_Step = 0.0;
			}
		//}
		break;
	case 2:
		//else if(Flags.Op_Mode==2) {    //레이저 측정용, 반복능
			TimePC_Step = TimePC_Step + Tsc;
			if(TimePC_Step<=-10.0)																INV1_PROFILE.POS_CMD = -0.01;				 // unit: [m]
			else if((TimePC_Step>-10.0)&&(TimePC_Step<=0.0))									INV1_PROFILE.POS_CMD = 0.0;      			 // unit: [m]
			else if((TimePC_Step>  0.0)&&(TimePC_Step<=INV1_PC.TimeStep))						INV1_PROFILE.POS_CMD = INV1_PC.PosStep;  	 // unit: [m]
			else if((TimePC_Step>INV1_PC.TimeStep)&&(TimePC_Step<=2.0*INV1_PC.TimeStep))		INV1_PROFILE.POS_CMD = 2.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>2.0*INV1_PC.TimeStep)&&(TimePC_Step<=3.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 3.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>3.0*INV1_PC.TimeStep)&&(TimePC_Step<=4.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 4.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>4.0*INV1_PC.TimeStep)&&(TimePC_Step<=5.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 5.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>5.0*INV1_PC.TimeStep)&&(TimePC_Step<=6.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 6.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>6.0*INV1_PC.TimeStep)&&(TimePC_Step<=7.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 7.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>7.0*INV1_PC.TimeStep)&&(TimePC_Step<=8.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 8.0*INV1_PC.PosStep;	 // unit: [m]
			else if((TimePC_Step>8.0*INV1_PC.TimeStep)&&(TimePC_Step<=9.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 9.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>9.0*INV1_PC.TimeStep)&&(TimePC_Step<=10.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 10.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>10.0*INV1_PC.TimeStep)&&(TimePC_Step<=11.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 11.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>11.0*INV1_PC.TimeStep)&&(TimePC_Step<=12.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 12.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>12.0*INV1_PC.TimeStep)&&(TimePC_Step<=13.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 13.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>13.0*INV1_PC.TimeStep)&&(TimePC_Step<=14.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 14.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>14.0*INV1_PC.TimeStep)&&(TimePC_Step<=15.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 15.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>15.0*INV1_PC.TimeStep)&&(TimePC_Step<=16.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 16.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>16.0*INV1_PC.TimeStep)&&(TimePC_Step<=17.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 17.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>17.0*INV1_PC.TimeStep)&&(TimePC_Step<=18.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 18.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>18.0*INV1_PC.TimeStep)&&(TimePC_Step<=19.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 19.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>19.0*INV1_PC.TimeStep)&&(TimePC_Step<=20.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 20.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>20.0*INV1_PC.TimeStep)&&(TimePC_Step<=21.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 21.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>21.0*INV1_PC.TimeStep)&&(TimePC_Step<=22.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 22.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>22.0*INV1_PC.TimeStep)&&(TimePC_Step<=23.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 23.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>23.0*INV1_PC.TimeStep)&&(TimePC_Step<=24.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 24.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>24.0*INV1_PC.TimeStep)&&(TimePC_Step<=25.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 25.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>25.0*INV1_PC.TimeStep)&&(TimePC_Step<=26.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 26.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>26.0*INV1_PC.TimeStep)&&(TimePC_Step<=27.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 27.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>27.0*INV1_PC.TimeStep)&&(TimePC_Step<=28.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 28.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>28.0*INV1_PC.TimeStep)&&(TimePC_Step<=29.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 29.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>29.0*INV1_PC.TimeStep)&&(TimePC_Step<=30.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 2.91; 				 // unit: [m]
			else if((TimePC_Step>30.0*INV1_PC.TimeStep)&&(TimePC_Step<=31.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 29.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>31.0*INV1_PC.TimeStep)&&(TimePC_Step<=32.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 28.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>32.0*INV1_PC.TimeStep)&&(TimePC_Step<=33.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 27.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>33.0*INV1_PC.TimeStep)&&(TimePC_Step<=34.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 26.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>34.0*INV1_PC.TimeStep)&&(TimePC_Step<=35.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 25.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>35.0*INV1_PC.TimeStep)&&(TimePC_Step<=36.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 24.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>36.0*INV1_PC.TimeStep)&&(TimePC_Step<=37.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 23.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>37.0*INV1_PC.TimeStep)&&(TimePC_Step<=38.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 22.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>38.0*INV1_PC.TimeStep)&&(TimePC_Step<=39.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 21.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>39.0*INV1_PC.TimeStep)&&(TimePC_Step<=40.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 20.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>40.0*INV1_PC.TimeStep)&&(TimePC_Step<=41.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 19.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>41.0*INV1_PC.TimeStep)&&(TimePC_Step<=42.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 18.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>42.0*INV1_PC.TimeStep)&&(TimePC_Step<=43.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 17.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>43.0*INV1_PC.TimeStep)&&(TimePC_Step<=44.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 16.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>44.0*INV1_PC.TimeStep)&&(TimePC_Step<=45.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 15.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>45.0*INV1_PC.TimeStep)&&(TimePC_Step<=46.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 14.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>46.0*INV1_PC.TimeStep)&&(TimePC_Step<=47.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 13.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>47.0*INV1_PC.TimeStep)&&(TimePC_Step<=48.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 12.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>48.0*INV1_PC.TimeStep)&&(TimePC_Step<=49.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 11.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>49.0*INV1_PC.TimeStep)&&(TimePC_Step<=50.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 10.0*INV1_PC.PosStep; // unit: [m]
			else if((TimePC_Step>50.0*INV1_PC.TimeStep)&&(TimePC_Step<=51.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 9.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>51.0*INV1_PC.TimeStep)&&(TimePC_Step<=52.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 8.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>52.0*INV1_PC.TimeStep)&&(TimePC_Step<=53.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 7.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>53.0*INV1_PC.TimeStep)&&(TimePC_Step<=54.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 6.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>54.0*INV1_PC.TimeStep)&&(TimePC_Step<=55.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 5.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>55.0*INV1_PC.TimeStep)&&(TimePC_Step<=56.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 4.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>56.0*INV1_PC.TimeStep)&&(TimePC_Step<=57.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 3.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>57.0*INV1_PC.TimeStep)&&(TimePC_Step<=58.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 2.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>58.0*INV1_PC.TimeStep)&&(TimePC_Step<=59.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 1.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>59.0*INV1_PC.TimeStep)&&(TimePC_Step<=60.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 0.0*INV1_PC.PosStep;  // unit: [m]
			else TimePC_Step = -20.0;
			//INV1_PC.PosRefFinal = INV1_PROFILE.POS_CMD;
			INV1_PROFILE.POS_CMD_PRESENT = INV1_PROFILE.POS_CMD;
			INV1_PROFILE.POS_CMD_old 	 = INV1_PROFILE.POS_CMD;
		//}
		break;
	case 3:
		//else if(Flags.Op_Mode==3){		//um 측정
			TimePC_Step = TimePC_Step + Tsc;
			if(TimePC_Step<=INV1_PC.TimeStep)													INV1_PROFILE.POS_CMD = 0.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>INV1_PC.TimeStep)&&(TimePC_Step<=2.0*INV1_PC.TimeStep))		INV1_PROFILE.POS_CMD = 1.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>2.0*INV1_PC.TimeStep)&&(TimePC_Step<=3.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 2.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>3.0*INV1_PC.TimeStep)&&(TimePC_Step<=4.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 3.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>4.0*INV1_PC.TimeStep)&&(TimePC_Step<=5.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 4.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>5.0*INV1_PC.TimeStep)&&(TimePC_Step<=6.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 5.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>6.0*INV1_PC.TimeStep)&&(TimePC_Step<=7.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 4.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>7.0*INV1_PC.TimeStep)&&(TimePC_Step<=8.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 3.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>8.0*INV1_PC.TimeStep)&&(TimePC_Step<=9.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 2.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>9.0*INV1_PC.TimeStep)&&(TimePC_Step<=10.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 1.0*INV1_PC.PosStep;  // unit: [m]
			else if((TimePC_Step>10.0*INV1_PC.TimeStep)&&(TimePC_Step<=11.0*INV1_PC.TimeStep))	INV1_PROFILE.POS_CMD = 0.0*INV1_PC.PosStep;  // unit: [m]
			else TimePC_Step = 0.0;
			//INV1_PC.PosRefFinal = INV1_PROFILE.POS_CMD;
			INV1_PROFILE.POS_CMD_PRESENT = INV1_PROFILE.POS_CMD;
			INV1_PROFILE.POS_CMD_old 	 = INV1_PROFILE.POS_CMD;
		//}
		break;
	default:
		//else TimePC_Step = 0.0;
		TimePC_Step = 0.0;
		//INV1_PC.PosRefFinal = INV1_PROFILE.POS_CMD;
//		INV1_PROFILE.POS_CMD_PRESENT = INV1_PROFILE.POS_CMD;    //
//		INV1_PROFILE.POS_CMD_old 	 = INV1_PROFILE.POS_CMD;    //
		break;
	}
}

#pragma CODE_SECTION(cpu1_enc_Z, "ramfuncs");
__interrupt void cpu1_enc_Z(void)
{
	IER &= 0x0000;
 	IER |= M_INT2 | M_INT3 | M_INT5 ;			// Fault || CC || SC 
	EINT;
	
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
	asm(" NOP");
}
