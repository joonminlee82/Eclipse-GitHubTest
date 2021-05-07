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
#include "Variable.h"
#include "KeyMenu.h"
#include "gnl.h"

#pragma DATA_SECTION(maxCount_ePWM, "CLADataLS0");
#pragma DATA_SECTION(halfTsw_PWM, "CLADataLS0");
Uint16 maxCount_ePWM =	(Uint16)(0.5 * SYS_CLK/(SWITCHING_FRQ));
Uint16 halfCount_ePWM = (Uint16)(0.25 * SYS_CLK/(SWITCHING_FRQ));
float halfTsw_PWM = 1./(2.*SWITCHING_FRQ);
float Tsamp = 1./((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD);
float INV_Tsamp = (SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD;
float Tsc = sc_cnt_max/((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD);
float INV_Tsc = 1/(sc_cnt_max/((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD));
float Tdrv = drv_cnt_max/((SWITCHING_FRQ/PWM_INT_NUM)*SAMPING_METHOD);
float Freq_i = 500.;
//#pragma DATA_SECTION(INV3, "CLADataLS0");
#pragma DATA_SECTION(INV1, "CLADataLS0");
Motor INV1, INV2;
void InitParameters(void)  // Motor Parameter
{

	INV1.Pole 			= 2;
	INV1.PolePitch		= KEY_GetLinMotorCode(LIN_MOTOR_POLE_PITCH)*0.001;
	INV1.Rs 			= KEY_GetLinMotorCode(LIN_MOTOR_RS);
	INV1.Lds 			= KEY_GetLinMotorCode(LIN_MOTOR_LS)*0.001;
	INV1.Lqs 			= KEY_GetLinMotorCode(LIN_MOTOR_LS)*0.001;
	INV1.LAMpm 			= KEY_GetLinMotorCode(LIN_MOTOR_KF);
	INV1.Is_rated 		= KEY_GetLinMotorCode(LIN_MOTOR_IQSE_RATE);

	// SMPM (INV1)
//	INV1.Pole 			= KEY_GetSynMotorCode(SYN_MOTOR_POLES);
	INV1.PolePair 		= INV1.Pole*0.5;
	INV1.InvPolePair 	= 1./(float)INV1.PolePair;
	//INV1.Rs 			= 0.123574;//실측 : 0.123574 현대 : 0.048;
//	INV1.Rs 			= KEY_GetSynMotorCode(SYN_MOTOR_RS);
	INV1.INV_Rs 		= 1/INV1.Rs;
	//INV1.Lds 			= 1.55e-3; //실측 : 1.55mH // data 1.3mH
	//INV1.Lqs 			= 1.55e-3;
//	INV1.Lds 			= KEY_GetSynMotorCode(SYN_MOTOR_LS)*0.001;
//	INV1.Lqs 			= KEY_GetSynMotorCode(SYN_MOTOR_LS)*0.001;
	//INV1.LAMpm 			= 0.373678;//측정 : 0.379039(pk-pk)~0.373678(AMP) // 현대 : 0.373195
//	INV1.LAMpm 			= KEY_GetSynMotorCode(SYN_MOTOR_KE);
	INV1.InvLAMpm 		= 1/INV1.LAMpm;
	INV1.Kt				= 1.5*INV1.PolePair*INV1.LAMpm;
	INV1.InvKt			= 1/INV1.Kt;

	//INV1.Jm 			= 48.99632;//현대 1짝 : 108.513, 측정 2짝 : 48.99632
	INV1.Jm 			= KEY_GetControlCode(SC_JM);
	INV1.InvJm 			= 1/INV1.Jm;
	///INV1.Bm 			= 1.747409;//850e-6+850e-6;
	//INV1.Bm 			= 0.;
	INV1.Bm 			= 1.747409;	// 속도 관측기
	INV1.Wr 			= 0.;
	INV1.Wrm 			= 0;
	INV1.Wrpm 			= 0;
	INV1.ThetaOffset 	= 0;    //KEY_GetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET);
	INV1.Thetar			= 0.;
	INV1.Thetar_init_new = 0.;

	//INV1.Is_rated 		= 99.;//2.85 * SQRT2; // peak value
//	INV1.Is_rated 		= KEY_GetSynMotorCode(SYN_MOTOR_IQSE_RATE);
	INV1.Te_rated 		= INV1.Kt * INV1.Is_rated;
	//INV1.Pout_rated 	= 1.8e3;
	INV1.Iq_max 		= INV1.Is_rated*KEY_GetFactoryCode(THRUST_FORCE_LIMIT)*0.01;
	INV1.Te_max 		= INV1.Kt*INV1.Iq_max;
	INV1.Theta_init 	= 0.;
	INV1.Del_thetar_init = 0.;
	INV1.INV_CNT_thetar_set = 0.0004;	//0.001
	INV1.Thetar_old 	= 0.;
	INV1.MaxSpeed 		= 1.0;
	INV1.Mass			= 100.;
	INV1.FL				= 0.;

    INV1.Active_Power   = 0.;
    INV1.Reactive_Power = 0.;
    INV1.Active_Power_f = 0.;
    INV1.Reactive_Power_f = 0.;
    INV1.Apparent_Power = 0.;
    INV1.Apparent_Power_f = 0.;
    INV1.Power_Factor = 0;

	// GRID (INV2)
//	INV2.Pole 			= 2;
//	INV2.PolePair 		= INV2.Pole>>1;
//	INV2.InvPolePair 	= 1./(float)INV2.PolePair;
//	INV2.Rs 			= 0.1;
//	INV2.INV_Rs 		= 1/INV2.Rs;
//	//INV2.Lds 			= 0.9e-3;
//	//INV2.Lqs 			= 0.9e-3;
//	INV2.Lds 			= KEY_GetFactoryCode(INPUT_L)*0.001;
//	INV2.Lqs 			= KEY_GetFactoryCode(INPUT_L)*0.001;
//	INV2.LAMpm 			= 1.;
//	INV2.InvLAMpm 		= 1/INV2.LAMpm;
//	INV2.Kt				= 1.5*INV2.PolePair*INV2.LAMpm;
//	INV2.InvKt			= 1/INV2.Kt;
//
//	//INV2.Jm 			= 375e-6+305e-6;//INV1 : 305e-6
//	//INV2.InvJm 			= 1/INV2.Jm;
//	//INV2.Bm 			= 850e-6+850e-6;//INV1 : 850e-6
//	INV2.Wr 			= 0.;
//	//INV2.Wrm 			= 0;
//	//INV2.Wrpm 			= 0;
//	//INV2.ThetaOffset 	= 0.;
//	INV2.Thetar			= 0.;
//
//	//INV2.Is_rated 		= 107.14; // peak value 50kW 기준
//	INV2.Is_rated 		= KEY_GetSynMotorCode(SYN_MOTOR_IQSE_RATE);
//	INV2.Te_rated 		= INV2.Kt * INV2.Is_rated;
//	//INV1.Pout_rated 	= 1.8e3;
//	INV2.Iq_max 		= INV2.Is_rated*KEY_GetFactoryCode(TORQUE_LIMIT)*0.01*1.1;
//	INV2.Te_max 		= INV2.Kt*INV2.Iq_max;

	// Super Cap (INV3)
	/*INV3.Pole 			= 2;
	INV3.PolePair 		= INV3.Pole>>1;
	INV3.InvPolePair 	= 1./(float)INV3.PolePair;
	INV3.Rs 			= 0.60;//0.78;//슈퍼캪 전체 ESR
	INV3.INV_Rs 		= 1/INV3.Rs;
	INV3.Lds 			= 0.3e-3;
	INV3.Lqs 			= 0.3e-3;
	INV3.LAMpm 			= 1.;
	INV3.InvLAMpm 		= 1/INV3.LAMpm;
	INV3.Kt				= 1.5*INV3.PolePair*INV3.LAMpm;
	INV3.InvKt			= 1/INV3.Kt;

	//INV2.Jm 			= 375e-6+305e-6;//INV1 : 305e-6
	//INV2.InvJm 			= 1/INV2.Jm;
	//INV2.Bm 			= 850e-6+850e-6;//INV1 : 850e-6
	INV3.Wr 			= 0.;
	//INV2.Wrm 			= 0;
	//INV2.Wrpm 			= 0;
	//INV2.ThetaOffset 	= 0.;
	INV3.Thetar			= 0.;

	INV3.Is_rated 		= 115.; // peak value 50kW 기준
	//INV3.Te_rated 		= INV2.Kt * INV2.Is_rated;
	//INV1.Pout_rated 	= 1.8e3;
	INV3.Iq_max 		= INV3.Is_rated;
	//INV3.Te_max 		= INV3.Kt*INV2.Iq_max;*/

}

FLAG Flags;
#pragma DATA_SECTION(Flags, "CLADataLS0");
void InitFlags(void)
{
	Flags.Start1			= 0;
	Flags.Start2 			= 0;
	Flags.Run1 				= 0;
	Flags.Run2 				= 0;
	Flags.Run3 				= 0;
	Flags.Run4				= 0;
	Flags.Vdc 				= 0; 		//0: Vdc_rd, 1:Vdc_set
	Flags.Vdc_filter 		= 1;
	Flags.Fault 			= 0;
	Flags.Fault_Warn 		= 0;
	Flags.Fault_Warn_RunCnt = 0;
	Flags.Ready1 			= 0;
	Flags.Ready2 			= 0;
	Flags.AngleAdv 			= 1;
//	Flags.Zcc 				= 0;

	Flags.Align1 			= 0;
	Flags.Align2 			= 0;
	Flags.Speed_control_1	= 0;
	Flags.Speed_control_2	= 0;
	Flags.Run_PLL 			= 0;

	Flags.Vdc_control 		= 0;
	Flags.inertia_emulation = 0;

	Flags.Profile_Control	= 0;

	Flags.Vsup_control 		= 0;
	Flags.Vdc_control_using_SuperCap =0;
	
	/////////////// HD Variable ///////////////////////////////
	Flags.Conv_Run 			= 1;
	Flags.Inv_Run 			= 1;
	Flags.Conv_Run_old 		= 0;
	Flags.Inv_Run_old 		= 0;
	Flags.Ready_INV 		= 0;
	Flags.Check_Vdc_charge 	= 0;
	Flags.MC_Relay 			= 0;
	Flags.GRID_Volt_Seq 	= 0;
	Flags.GRID_Volt_SeqChk 	= 0;
	Flags.Vdc_Control_Ok 	= 0;
	Flags.Fault_old 		= 0;
	Flags.Fault_Warn_old 	= 0;
	Flags.Not_Control_SuperCap = 0;
	Flags.RotorPos_est 		= 1;
	Flags.HF_Inj 			= 0;
	///////////////////////////////////////////////////////////
	///////////////// FREEWAY Variable ////////////////////////

	Flags.Op_Mode			= 0;    
	Flags.ProfileMode		= 0;
	Flags.RTN2ZERO			= 0;   
	Flags.Kt_mode 			= 0;
	Flags.Pos_Con_Kt_mode 	= 4;
	Flags.Position_control	= 1;
	Flags.CompProfile		= 0;
	Flags.PreLoad			= 1;
	///////////////////////////////////////////////////////////
	Flags.PosCon_with_APS	= 0;
	Flags.Decel             = 0;
}

//for ADC Scale
float ScaleADin_Vac_1_5kW   = (3.0*SenScale_Vac_1_5kW/Rm_Ain_Vac_1_5kW/0.2/65536.);
float ScaleADin_Vdc_1_5kW   = (3.0*SenScale_Vdc_1_5kW/Rm_Ain_Vdc_1_5kW/0.2/65536.);
float ScaleADin_ESS_Vdc     = (3.0*SenScale_ESS_Vdc/Rm_Ain_ESS_Vdc/0.2/65536.);
float ScaleADin_Iac_1_5kW   = (3.0*SenScale_Iac_1_5kW/Rm_Ain_Iac_1_5kW/0.2/65536.);
float ScaleADin_ESS_Idc     = (3.0*SenScale_ESS_Idc/Rm_Ain_ESS_Idc/0.2/65536.);

//float ScaleAin_SOC0[4]  = {0.011165, 0.011165, 0.011165, 0.03105};
//float OffsetAin_SOC0[4] = {365.85366, 365.85366, 365.85366, 830.33};
float ScaleAin_SOC0[4]  = {1.34215, 1.34215, 1.34215, 0.025431315};
float OffsetAin_SOC0[4] = {9.375, 9.375, 9.375, 830.33};
float ScaleAin_SOC2[4]  = {0.029892, 0.029892, 0.011165, 0.029892};
float OffsetAin_SOC2[4] = {979.5, 979.5, 365.85366,979.5};

float CONV_Ibs_Flt_HW_Set 	= 20.;	//CONV -Ibs
float CONV_Ics_Flt_HW_Set	= 20.;	//CONV -Ics
float CONV_Ias_Flt_HW_Set	= 20.;	//CONV -Ias
float Vdc_Flt_HW_Set 		= 750.;	//Vdc

float INV_Ibs_Flt_HW_Set 	= 20.;	//INV Ibs
float INV_Ics_Flt_HW_Set 	= 20.;	//INV Ics
float INV_Ias_Flt_HW_Set 	= 20.;	//INV Ias
float ESS_I_Flt_HW_Set 		= 145.;	//sup I

float GRID_Vbc_HW_Set 	= 650.;	// grid B-C
float GRID_Vab_HW_Set 	= 650.;	// grid A-B
float ESS_V_Flt_HW_Set 	= 670.;	// super V (2.8*240=672V rated, 2.7*240= 648V max)

//DC link 전압 변수
float Vdc=0., Vdc_rd=0., Vdc_set=540., INV_Vdc=1./540.;
float Vdc_filter =0.;
#pragma DATA_SECTION(Vdc_rd, "CLADataLS0");
#pragma DATA_SECTION(Vdc, "CLADataLS0");
#pragma DATA_SECTION(INV_Vdc, "CLADataLS0");

//Super Cap Vdc
float Vsup_dc =0.;
float Flag_power_control_mode =0.;
#pragma DATA_SECTION(Vsup_dc, "CLADataLS0");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*Fault*/
u8 FLT_backup[ERRORBACKUPDATANUM];
u8 Fault_RealYear[ERRORSTORENUM];
u8 Fault_RealMonth[ERRORSTORENUM];
u8 Fault_RealDate[ERRORSTORENUM];
u8 Fault_RealHour[ERRORSTORENUM];
u8 Fault_RealMinute[ERRORSTORENUM];
u8 Fault_RealSec[ERRORSTORENUM];
void Fault_Vari_Init(void){
	int i;
	for(i=0; i< ERRORBACKUPDATANUM; i++){
		FLT_backup[i] = 0;
	}

	for(i=0; i< ERRORSTORENUM; i++){
		Fault_RealYear[i] = 0;
		Fault_RealMonth[i] = 0;
		Fault_RealDate[i] = 0;
		Fault_RealHour[i] = 0;
		Fault_RealMinute[i] = 0;
		Fault_RealSec[i] = 0;
	}
}
#pragma DATA_SECTION(EncType, "CLADataLS0");
#pragma DATA_SECTION(KNOW_ANGLE, "CLADataLS0");
Uint16	EncType = 0, KNOW_ANGLE = 0;
/*CC*/
float E_mag_set = 380., Eld_In_Volt = 310., InVoltage = 310., Vdc_relay_on = 490., UVDC_set = 540., Vdc_relay_off = 390.;	// UVDC 400-> 540 for FREEWAY
float ELD_Vdc_relay_on = 490., ELD_UV_set = 400., ELD_Vdc_relay_off = 390.;
float Is_rate = 0., OVDC_Set = 850;
float Theta_U = 0., TqBiasLmt = 0.;
float OC_Set_I = 20., OC_Set_C = 20., OC_Set_Sup = 20., OVDC_Sup = 780.;
Uint16 Flag_MecaObs = 1, Flag_EncFilter = 1;
float Wcc = 200.;
float sum_current1 = 0.;
float Normal_VDC = 0.;
/*SC*/
float Wsc = 2.;
/* PC */
float Wpc = 2.;
float test_postion = 0.;
/* Auto Tqbias vari */
#pragma DATA_SECTION(Enc_Dir, "CLADataLS0");
Uint16 Flag_Plus = 0, Flag_Plus_old = 0, Flag_Exit = 0, Flag_READY = 0, Tqbias_Step_Cnt = 0, TQSELECT = 0 ;
Uint16 Enc_Dir = 1, FWD_Dir = 0;
float SC_fTqBias_pre = 0., SC_fTqBias = 0., SC_fTqBias_off_didt = 0., SC_fTqBias_off = 0. ;
float SC_fTqBias_didt = 0.;
Uint16 uEL_Start = 0, Flag_RUN = 0, Flag_slope = 0;

float OVER_SP_set = 0., OVER_SP_set_K = 0.;
float SP_Limit = 0.;
float TqBias_Step = 0., Iqse_Pos_ff = 0., Kpp = 0.;
float Pos_real = 0., EncPPR = 0.;

/*SinCos Encoder*/
float	SinMax = 0, SinMin = 1000, SinMax_old = 1000, SinMin_old = 1000, SinOffset = 0, CosMax = 0, CosMin = 1000, CosMax_old = 1000, CosMin_old = 1000, CosOffset = 0 ;
float thetar_H = 0, thetar_L = 0 ;
int 	Flag_thetar_H = 0, Flag_thetar_L = 0, Flag_thetar_False = 0 ;
unsigned int Flag_SINCOSTH = 0, Flag_Sin = 0, Flag_U = 0, Flag_Atan = 0, Flag_Atan_old = 0, AtanCnt = 0, Z_Cnt = 0, Z_Cnt_old = 0 ;
#pragma DATA_SECTION(Flag_SINCOSTH, "CLADataLS0");
#pragma DATA_SECTION(Flag_Sin, "CLADataLS0");
/*SCI*/
extern Uint16 Scid_Tx_Buf[100] = {0}, Scic_Tx_Cnt = 0, Scic_Tx_End = 0, Scid_Tx_Cnt = 0, Scid_Tx_End = 0 ;
extern Uint16 scic_tx_cnt = 0 ;
extern Uint16 Key_In = 0, Key_In_old = 0, Key_Value = 0 ;
extern Uint16 crc16_table[256] =
{ 0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/*EEPROM*/
long FRAM_value = 0, E_Check = 0xFFAA5500;

/* Non-Stop Vari */
Uint16 NumOfnonStop = 0, nonStop_Cnt = 0, nonStop_LoopCnt = 0;//non-stop
Uint16 nonStop_Flr[MAXNONSTOPNUM] = {0,0,0,0,0,0,0,0,0,0,
									0,0,0,0,0,0,0,0,0,0,
									0,0,0,0,0,0,0,0,0,0,
									0,0,0,0,0,0,0,0,0,0,
									0,0,0,0,0,0,0,0,0,0,
									0,0,0,0,0,0,0,0,0,0,
									0,0,0,0};
int 	BDIB_X3 = 0,		BDIB_X2 = 0,		BDIB_X1 = 0,		BDIB_SUF = 0,		
		BDIB_PLUH = 0,		BDIB_RST = 0,		BDIB_PLDH = 0,		BDIB_PLDM = 0,
		BDIB_CP_AT = 0,		BDIB_SDF = 0,		/*	BDIB_INI = 0,*/	BDIB_HOU = 0,	
		BDIB_HOD = 0,		BDIB_PLUM = 0,		BDIB_ENABLE = 0,	BDIB_DZ = 0,
		BDIB_RDY = 0,		BDIB_ULS = 0,		BDIB_DLS = 0,
		BDIB_MCC = 0,		BDIB_UP = 0, 		BDIB_DOWN = 0;
int 	BDIB_PLDH2 = 0, 	BDIB_PLUH2 = 0, 	BDIB_XMCS = 0, 		BDIB_X29 = 0, 		BDIB_XAUTO = 0;
int 	BDIB_XMC2 = 0, 		BDIB_XBKAS = 0, 	BDIB_XBKBS = 0, 	BDIB_FUSC = 0;
int 	BDIB_INP1 = 0, 		BDIB_INP2 = 0, 		BDIB_INP3 = 0;
int 	BDIB_SUS4 = 0, 		BDIB_SUS5 = 0, 		BDIB_SUS6 = 0, 		BDIB_SDS4 = 0, 		BDIB_SDS5 = 0, 		BDIB_SDS6 = 0;
float   fSUF_stPos = 0., 	fPLUM_stPos = 0., 	fPLUH_stPos = 0., 	fPLUH2_stPos = 0., 	fSUS4_stPos = 0., 	fSUS5_stPos = 0., 	fSUS6_stPos = 0.;
float   fSDF_stPos = 0., 	fPLDM_stPos = 0., 	fPLDH_stPos = 0., 	fPLDH2_stPos = 0., 	fSDS4_stPos = 0., 	fSDS5_stPos = 0., 	fSDS6_stPos = 0.;
float   fSUF_RealPos = 0., 	fPLUM_RealPos = 0., fPLUH_RealPos = 0., fPLUH2_RealPos = 0.,fSUS4_RealPos = 0., fSUS5_RealPos = 0., fSUS6_RealPos = 0.;
float	fSDF_RealPos = 0., 	fPLDM_RealPos = 0., fPLDH_RealPos = 0., fPLDH2_RealPos = 0.,fSDS4_RealPos = 0., fSDS5_RealPos = 0., fSDS6_RealPos = 0.;
float   fSUF_Diff = 0., 	fPLUM_Diff = 0., 	fPLUH_Diff = 0., 	fPLUH2_Diff = 0., 	fSUS4_Diff = 0., 	fSUS5_Diff = 0., 	fSUS6_Diff = 0.;
float	fSDF_Diff = 0., 	fPLDM_Diff = 0., 	fPLDH_Diff = 0., 	fPLDH2_Diff = 0., 	fSDS4_Diff = 0., 	fSDS5_Diff = 0., 	fSDS6_Diff = 0.;
float	fRunnigOpenLength = 0., fRunOpenLength = 0., fChime_Point = 0.;
Uint16 ELD_SpeedMode_Flg = 0;
int	ChimeFlgStart = 0;
int	Inverter_Sel = 0;
int RMB_ELSpeed = 0, RMB_Decel_Flg = 0;
float fRMB_Decel_Flg = 0., RMB_fVmax_K = 0.;
int Van_ErrFlg = 0;
float fVariOffset = 0.0001;

float fmpm_Spd_fbk = 0.;
int BDIB_HOD_Old = 0, BDIB_HOU_Old = 0;
int Rsc_Vane = 0;
unsigned long ulCurrent_Mileage = 0, ulCurrent_Mileage_temp = 0;
long lCurrent_Mileage = 0, lCurrent_Mileage_temp = 0;
Uint16 LimitOver_Mileage_Flg = 0;
Uint16 DoorZone_Flg = 0;
int CALL_Data = 0, RMB_DIV9_Flg = 0, CALL_Data_Old = 0;
Uint16 uPloCreepFlag = 0;

int	SaveFault_Flg = FALSE;

int RMB_chgStimeAcc_Flg = 0;
float fRMB_Stop_Pos = 0., fRMB_reCalDecelLength = 0.;

int CP_Lab_Test = 0;
float fTqBiasValue = 0.;
Uint16	RMB_SpdPtnSel = 0,DecelModeSelect = 0, HHT_Use_Flg = 0, Man_Cmd = 0, Man_SpdSel = 0; 
Uint16 Inv_Control_Mode = 0, Motor_Type = 0, Mth_Use = 0;
float fRMB_DblSpd_Pos_Start = 0., fRMB_DblSpdDiffPos = 0.;
float fDecFloorOffset = 0., fDecFloorOffset_delta = 0., Commu_delay = 0. ;

/*Forced decel switch control*/
int  SUDS_SW_NUM = 0, SUDS_POS_Err = 0, EL_Speed = 0, SUDSOK = 0, SUDS_ONOFF = 0 ;
float  RMB_fVref_Lmt = 0.;
float SUDS_SW_Pos[7] = {0.,0.,0.,0.,0.,0.,0.}, SUS_Vref[7] = {0.,0.,0.,0.,0.,0.,0.}, SDS_Vref[7] = {0.,0.,0.,0.,0.,0.,0.} ;
float Acc2_SUDS = 0., RMB_fPosLmt = 0. , SUDSPos = 0., RMB_fPosLmt_SW = 0. ;
float RMB_fVrefOld_Lmt = 0., Acc_SUDS = 0. ;
Uint16 	SUDS_SW_BIT = 0, LimitSw_PosFlg = 0 ;
int Flag_SUSD = 0  ;
float SUDS_fPositionLength = 0., SUDF_fPositionToLength = 0., SUDS_Vmax = 0. ;
int SUDS_UPDN = 0 ;	// SUDS
Uint16 SUDS_Ptn_Use = 1;
float SUDS_LENGTH_OFFSET = 0., SUDS_VANE_LENGTH = 0.;
/*Floor Control*/
float fTqunbal = 0., fTqunbal_tmp = 0., Sign_TqCom = 0.;
float TqUnbal_Bot[11] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};
float TqUnbal_Top[11] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};
Uint16 INIT_DRV = 0, fFlagAfterInit = 0 ;
Uint16 ELD_Speed_Mode = 0;
/*Drive*/
int u20msFlag = 0, Drive_Cnt = 0 ;
int Flag_ZSP = 0;
int DriveStatPsudo = 0, ANGLE_cnt = 0,   HwFltCnt = 0, HwFltWarnCnt = 0, SwFltCnt = 0, DclFlag = 0 ;
int Flag_Tans = 0, Flag_mode = 0, Flag_Err = 0, Bko = 0 ;
float	I_phase_rms = 0., V_ll_rms = 0., C_phase_rms = 0. ;
/* Load */
float DRV_fLS_LoadAD = 0., DRV_fLS_LoadAD_L = 0.,DRV_fLS_LoadAD_Old = 0.;
float DRV_fLS_Load_AD50 = 0., DRV_fTqBias = 0., DRV_fLS_Load_Gain = 0., DRV_fLS_LoadtoTq = 0.;
float DRV_fLS_Load_AD0 = 0., DRV_fLoadData = 0., DRV_INV_perLoad = 0.;
/* Initial Position */
#pragma DATA_SECTION(Flag_thetar_reset, "CLADataLS0");
#pragma DATA_SECTION(CNT_thetar, "CLADataLS0");
#pragma DATA_SECTION(CNT_thetar_set, "CLADataLS0");
int	  encU = 0, encV = 0, encW = 0, encZ = 0, encUVW = 0, encUVW_old = 0, Flag_thetar_reset = 0, CNT_thetar = 0, CNT_thetar_set = 2500, Reset_Thetar_Flg = 0 ;	// CNT_thetar_set = 1000
int	  encUVW_Chk_old = 0, Flag_reset = 0, encU_old = 0 ;
int 	ENC_AB = 0, GENC_AB = 0, ENC_ABCnt = 0, GENC_ABCnt = 0, ENCnt = 0, ENCHK = 0  ;
/*Fault*/
int ErrGetFlag = 0;
/*Test*/
Uns Power_Test = 0, Fwd_Rev_Test = 0;
float Power_Test_Gain = 0., Power_Test_Idse = 0.;
/*AD_Offset vari*/
int64 Offset_ADC_SOC0[4]={0,0,0,0};
int64 Offset_ADC_SOC1[4]={0,0,0,0};
int64 Offset_ADC_SOC2[4]={0,0,0,0};

float SC_F_load = 0., SC_T_load =0.;
float SC_mass_real = -800.;
float SC_mass_total = 0.;	
float SC_P_emul = 0.;
float SC_Iqse_init = 0.;
float Vcap_L = 0.;
float P_elec_grid =0.;
float P_conv_loss =0.;
float P_mech_real =0.;

float temp_cal = 0.;
float VC_Iqse_Ref = 0.;
float VC_P_sup = 0.;
float Power_Limit_Set = 50000.;
float Vcap =0., RMB_INV1_Iqse_50L = 10.;

/* CLA1 data */
#pragma DATA_SECTION(Flag_offset, "CLADataLS0");
#pragma DATA_SECTION(offsetLoopCnt, "CLADataLS0");
#pragma DATA_SECTION(offsetMaxCnt, "CLADataLS0");
#pragma DATA_SECTION(ScaleAin_SOC1, "CLADataLS0");
#pragma DATA_SECTION(OffsetAin_SOC1, "CLADataLS0");
Uint16	Flag_offset = 0;
/*AD_Offset vari*/
int32 offsetLoopCnt = 0, offsetMaxCnt = (0x1<<OFFSET_MAX_CNT);
float ScaleAin_SOC1[4] = {0.011165, 0.011165, 0.011165, 0.011165};
float OffsetAin_SOC1[4] = {365.85366, 365.85366, 365.85366,365.85366};

/*Grid PLL*/
Uint16 	sequence_count = 0;

float Thermal = 0;

int SYNorLIN_Motor = 2;
int SYNorLIN = 2;

float Test_Key_Menu = 0;

/* CC */
int CC_ExTimeChk_Flg = 0;
float cc_V_ref = 0.;
int v_mode = 0;
int Static_Force_Measurement = 0;
float V_ref_mag = 0.;
float cc_Hz_f = 0.;
u8 TxCNT = 0;
u16 u16TXsize = 0;
u16 u16SpiTXsize = 0;

u8 u8ScicRx[256] = {0,};
int TestSpi[256] = {0,};
u8 u8SpibTxCnt = 0;
u8 u8SpibRxCnt = 0;

/* Inverter Data Transformation (SPI) */
u32 u32Wrm = 0;
u32 u32Wrm_Ref = 0;
u32 u32Ide = 0;
u32 u32Ide_Ref = 0;
u32 u32Iqe = 0;
u32 u32Iqe_Ref = 0;
u32 u32V_ref_mag = 0;
u32 u32Vdc = 0;
u32 u32Te = 0;
u32 u32CurrentPos = 0;
u8 u8SpdFaultCnt = 0;
u8 u8OvAngleCnt = 0;

/* Spi Ring Buffer */
u32 spi_ring_buf[SPI_RING_BUF_SIZE] = {0,}; // ring buffer
u32 u32spi_ring_buf_len = 0; // ring buffer length
u32 u32spi_ring_buf_lp = 0; // load pointer
u32 u32spi_ring_buf_cp = 0; // consume pointer

u8 u8TestTypeCasing[6] = {0,};


u8 Flag_DA_Test = 0;
float32 Theta_grid = 0.;
float32 GRID_Vas = 0, GRID_Vbs = 0, GRID_Vcs = 0;
float32 GRID_Vab = 0, GRID_Vbc = 0, GRID_Vca = 0;
u32 g_u32MAX_FLOOR = 0;
float SetPosition = 0.;
u8 u8Spic_Comm_Chk = 0;
u8 u8Spic_SC_Comm_Chk = 0;
u8 DMA_Start_CNT = 0.;
