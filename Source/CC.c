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
                        : Heidenhain Encoder F/W Added (20190904 by GWON)
******************************************************************************/
#include "CC.h"
#include "SC.h"
#include "FRAM.h"
#include "drive.h"
#include "Fault.h"
#include "SCI.h"
#include "comp.h"
#include "bdib.h"
#include "AD_Offset.h"
#include "Variable.h"
#include "eQEP_Encoder.h"
#include "SPI.h"

#define __DA_TEST

//ADC Voltage
float ADC_A_RESULT_VOLTAGE[3]={0.,};
float ADC_B_RESULT_VOLTAGE[3]={0.,};
float ADC_C_RESULT_VOLTAGE[3]={0.,};
float ADC_D_RESULT_VOLTAGE[3]={0.,};

//cc counter
Uint16 cc_cnt =0;
int fault_cc=0;
extern Uint16 fault_cnt;
float Time_Chk = 0., execute_time = 0.;
Uint32 ccTimer0start;
Uint32 delta_ccTimer0;
Uint32 max_ccTimer0;

//CC variable
CC_3PH 	INV1_CC;
CC_3PH 	INV2_CC;

extern PROFILE INV1_PROFILE;
////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern Flt 		FLT;
extern Uint16 	Flag_RUN;
extern SCI_Out2	SCI_OUT2 ;
Uint16 uDrvST = 0;

extern	STATUS_DSP_OBJ 	g_SpiDspStatus;
extern  STATUS_ARM_OBJ  g_SpiArmStatus;
extern  IdCoData        g_IdSpiCoData;
extern  Id48Data        g_Id48Data;
extern  IdFAData        g_IdFAData;

extern ENC_DATA_OBJ ENC1_Data;
extern ENC_DATA_OBJ ENCB_Data;
extern ENC_DATA_OBJ ENCA_Data;

//extern  BDIB        g_BDIB;

Uint16 Flag_SinInit = 0;
Uint16 Scic_FIFO_OVF = 0;
Uint16 Spib_FIFO_OVF = 0;

float  RotorPos_EstTime = 0.;


#ifdef __5KHZ
Uint16 Vdc_Con_st       =0;
Uint16 Vdc_Con_st_Set   =2.5;     //10kHz: 5, 15kHz: 7;   0.5msec
Uint16 Spd_Con_st       =0;
Uint16 Spd_Con_st_Set   =5;    //10kHz:10, 15kHz: 15;  1msec
Uint16 Drive_Con_st     =0;
Uint16 Drive_Con_st_Set =25;    //10kHz: 50, 15kHz: 75; 5msec
#endif

#ifdef __10KHZ
Uint16 Vdc_Con_st 		=0;
Uint16 Vdc_Con_st_Set 	=5;		//10kHz: 5, 15kHz: 7;   0.5msec
Uint16 Spd_Con_st 		=0;
Uint16 Spd_Con_st_Set 	=10;	//10kHz:10, 15kHz: 15;  1msec
Uint16 Drive_Con_st 	=0;
Uint16 Drive_Con_st_Set =50;	//10kHz: 50, 15kHz: 75;	5msec
#endif

#ifdef __15KHZ
Uint16 Vdc_Con_st 		=0;
Uint16 Vdc_Con_st_Set 	=7;		//10kHz: 5, 15kHz: 7;   0.5msec(0.46666666msec)
Uint16 Spd_Con_st 		=0;
Uint16 Spd_Con_st_Set 	=15;	//10kHz:10, 15kHz: 15;  1msec
Uint16 Drive_Con_st 	=0;
Uint16 Drive_Con_st_Set =75;	//10kHz: 50, 15kHz: 75;	5msec
#endif

#ifdef __30KHZ
Uint16 Vdc_Con_st 		=0;
Uint16 Vdc_Con_st_Set 	=14;	//10kHz: 5, 15kHz: 7;   0.5msec(0.46666666msec)
Uint16 Spd_Con_st 		=0;
Uint16 Spd_Con_st_Set 	=30;	//10kHz:10, 15kHz: 15;  1msec
Uint16 Drive_Con_st 	=0;
Uint16 Drive_Con_st_Set =150;	//10kHz: 50, 15kHz: 75;	5msec
#endif

Uint16 da_flg = 1;
int AD_Cnt = 0, AD_Delay = 0;

Uint16 EarCcnt = 0;
float flag_sumcurrent_error =0.;

Uint16 Flags_avr_current = 1;
Uint16 Flags_PWM_Test = 0;

Uns CC_PWM_Data1 = 1000, CC_PWM_Data2 = 1600, CC_PWM_Data3 = 3000;

Uint16 INV1_SpdObs_OS_FltCnt = 0;
Uint16 MechObs_SetCnt = 0;
Uint16 PosCnt = 0;
float FL_d1 = 0.;

u8 u8BK_RELEASE_TIMER = 0;
u8 u8BK_CK_TIMER = 0;

u8 u8Spic_Test_Flag = 0;

u8 u8Caltimer = 0;

u8 u8Offset_Cal = 0;
u8 u8EncoderInfo_d1 = 0;

float Flag_ENC_Sel = 0;
u8 Spic_Err_Cnt = 0;

#pragma CODE_SECTION(cpu1_cc, "ramfuncs");
__interrupt void cpu1_cc(void)
{
	u32 u32TimerCNT_temp = 0;

	/* Enable Fault Interrupt */
	IER &=0x0000;
	IER |=M_INT2;		// Fault INT Enable
	EINT;
	TEST1_ON;
	//TEST3_ON;
	ccTimer0start = ReadCpuTimer0Counter();

	/* Compensated ADC Delay */
	while(AD_Cnt<AD_Delay)	AD_Cnt++;
	AD_Cnt = 0;
	
	/* External DAC out */
	if(da_flg == 1) daOut();

	cc_cnt++;
	Time_Chk = Time_Chk + Tsamp;


#ifdef __DA_TEST
    if(Flag_DA_Test)
    {
        Theta_grid += 60*TWOPI*Tsamp;
        Theta_grid = BOUND_PI(Theta_grid);

        GRID_Vas = 310*__sin(Theta_grid);
        GRID_Vbs = 310*__sin(Theta_grid - TWOPI/3);
        GRID_Vcs = 310*__sin(Theta_grid + TWOPI/3);

        GRID_Vab = GRID_Vas - GRID_Vbs;
        GRID_Vbc = GRID_Vbs - GRID_Vcs;
        GRID_Vca = GRID_Vcs - GRID_Vas;
    }
#endif

	/* Transmitted Hiedenhain Encoder BD */
    DMA_Start_CNT++;

    if(DMA_Start_CNT >= 15000)
    {
        DMA_Start_CNT = 15001;

        /* Received Hiedenhain Encoder Data */
        StartDMACH6();                          // Start SPI TX DMA channel
        asm(" NOP");
        StartDMACH5();                          // Start SPI RX DMA channel

        /* If first rdata is not STX then Encoder A will be done Watchdog reset */
        /* And position values are not exceeded 593.75mm */
        if(ENCA_Data.rdata[0] == STX && ENCA_Data.rdata[9] == ETX && (ENCA_Data.positionfA < 600 || ENCA_Data.positionfB < 600))
        {
            u8Spic_Comm_Chk = 1;
            ENCA_Data.sdata[9] = 0x06;           // Communication of between DSP and Encoder BD is Okay...!!
        }
        else
        {
            u8Spic_Comm_Chk = 0;
            ENCA_Data.sdata[9] = 0x04;           // Communication Fault Check
            Spic_Err_Cnt++;
            if(g_Id43Data.u8RxData.u8EncoderType == __HEIDENHAINMODE && Spic_Err_Cnt >= 65000){
                Spic_Err_Cnt = 65001;
                ENCA_Data.sdata[9] = 0x07;       // Communication Fault
                FLT_Raise(FLT_UVW);
            }
        }
    }
    else ENCA_Data.sdata[9] = 0x05;              // Communication Start

	/* USER CODE START */
	/* ADC Read */
	ADC_Read();									 // Modify for FREEWAY 20180209 by GWON. In order to improve the readability
	/* Main MC Operation Sequence     */
	/* For D C-Link Magnetic Contactor */
	DC_MC_OP();
	/* Software Level Fault Check */
	SW_Fault_Check();
	/* Hiedenhain Encoder */
	if(u8Spic_Comm_Chk == 1)
	{
	    Calc_Hiedenhain_POS_SPD(&ENCA_Data);
	}
	/* Position & Speed Calculation for Current Control */
	Calc_Enc_Pos(&Encoder_HD);
	/* Position Calculation from APS */
	Calc_APS_POS_SPD();
	/* Mechanical Observer for FRLSM */
	switch(g_Id43Data.u8RxData.u8EncoderType)
	{
	case __LINEARSCALEMODE:
//        Mechanical_Observer(&INV1, Encoder_HD.theta_elec, &INV1_Spdobs);
        Mechanical_Observer(&INV1, ENCA_Data.theta_elec, &INV1_Spdobs);
        Flag_EncFilter = 0;
        Flag_MecaObs = 0;
	    break;
	case __HEIDENHAINMODE:
	    Mechanical_Observer(&INV1, ENCA_Data.theta_elec, &INV1_Spdobs);
        Flag_EncFilter = 0;
        Flag_MecaObs = 0;
	    break;
	default:
        Mechanical_Observer(&INV1, Encoder_HD.theta_elec, &INV1_Spdobs);
        Flag_EncFilter = 1;
        Flag_MecaObs = 1;
	    break;
	}
	/* Insert to Control Variable from Encoder Variable */
	Applied_Speed_Variable(g_Id43Data.u8RxData.u8EncoderType, &INV1, &INV1_CC, &Encoder_HD, &ENCA_Data);
	/* Applied Speed Observer */
	Applied_Speed_Observer(g_Id43Data.u8RxData.u8EncoderType, &INV1, &INV1_CC, &INV1_Spdobs, &Encoder_HD, &ENCA_Data);

	/* Sequence check for Initial Rotor Postion Estimation */
	uDrvST = DRV_GetStatus();
	/* Initial Rotor Position Estimation Start Condition Check */
	if((Flags.Inv_Run == 0)||((uDrvST&DRV_CC_FLG)==0))
	{
		Init_Inv();									// Initialization Inverter Variables
	}
	else
	{
		if((Flags.Inv_Run_old == 0)&&(Flags.Inv_Run == 1)&&(uDrvST&DRV_CC_FLG))
		{
			Flags.Run1 = 1;
			if(KNOW_ANGLE == 1)
			{
				Flags.Ready1 = 1;					// A Flag of aling complete. ex) Iinit Position Flag
				Flags.Align1 = 0;
				g_SpiArmStatus.u8LMotAngleFind = 0x01;
				Flags.Inv_Run_old = Flags.Inv_Run;
			}
			else if(KNOW_ANGLE == 0)
			{
				Flags.Align1 = 1;					// A Flag of aling complete. ex) Iinit Position Flag
				Flags.Ready1 = 0;
				Flags.Speed_control_1 = 0;
				g_SpiArmStatus.u8LMotAngleFind = 0x00;
				g_IdFAData.u8Flag_RotorEstimation = g_SpiArmStatus.u8LMotAngleFind;
				Flags.Inv_Run_old = Flags.Inv_Run;
			}
		}
	}
	/* Initial Rotor Postion Estimation */
	if((Flags.Align1)&&(Flags.Ready1==0)) Rotor_InitPos_Estimation(&INV1, &INV1_CC, DC_INJECT);
	else Time_Chk = 0;

	/* Clark & Park Transformation */
	Current_Transform(&INV1,0);		// Current Transform function must be on next Rotor_InitPos_Estimation!!!!

	/* Current Control */
	if((Flags.Run1)||(CC_ExTimeChk_Flg == 1)){
		if(v_mode){	// V/F Control Mode
			INV1.Thetar += cc_Hz_f*TWOPI*Tsamp;
			INV1.Thetar = BOUND_PI(INV1.Thetar);
			INV1.Wr = cc_Hz_f*TWOPI;

			INV1_CC.Vqse_Ref =cc_V_ref;
			INV1_CC.Vdse_Ref =0.;
			// Coordinate change (synchronous  --> stationary)
			INV1_CC.Vdss_Ref = INV1.Cos_Theta*INV1_CC.Vdse_Ref - INV1.Sin_Theta*INV1_CC.Vqse_Ref;
			INV1_CC.Vqss_Ref = INV1.Sin_Theta*INV1_CC.Vdse_Ref + INV1.Cos_Theta*INV1_CC.Vqse_Ref;

			// Phase voltage
			INV1_CC.Vas_Ref = INV1_CC.Vdss_Ref;
			INV1_CC.Vbs_Ref = -0.5*(INV1_CC.Vdss_Ref - SQRT3*INV1_CC.Vqss_Ref);
			INV1_CC.Vcs_Ref = -(INV1_CC.Vas_Ref + INV1_CC.Vbs_Ref);
		}
		else if(Static_Force_Measurement){          // Static Force Measurement Mode
		    INV1_CC.Iqse_Ref = g_Id48Data.StaticForceCurrentRef;
		    INV1_CC.Idse_Ref = 0.;
            INV1_CC.Iqse_Ref = __fsat(INV1_CC.Iqse_Ref,INV1_SC.Iqse_ref_max,-INV1_SC.Iqse_ref_max);
		    if(KNOW_ANGLE == 1){
		        CurrCtrl(&INV1, &INV1_CC, 0);
		    }
		    else{
		        INV1_CC.Iqse_Ref_set = 0.;
		    }
		    if(fabs(INV1_CC.Err_Iqse) <= 0.001){
		        g_IdFAData.u8Flag_StaticForceCurrentCon = 0x31;
		    }
		    else if(fabs(INV1_CC.Err_Iqse) == 0.0 &&  INV1_CC.Iqse_Ref_set == 0){
		        g_IdFAData.u8Flag_StaticForceCurrentCon = 0x30;
		    }
		    else{
		        g_IdFAData.u8Flag_StaticForceCurrentCon = 0x32;
		    }
		}
		else{
			if(Flags.Speed_control_1){
				//INV1_CC.Idse_Ref_set = 0.;
				INV1_CC.Iqse_Ref_set = 0.;
			}
			else{
				INV1_CC.Idse_Ref = INV1_CC.Idse_Ref_set;
				if(Flag_slope == 0) INV1_CC.Iqse_Ref = INV1_CC.Iqse_Ref_set;
			}
			if(((Flags.Ready1 == 1)&&(Flags.Align1 == 0))||(CC_ExTimeChk_Flg == 1)){
				V_ref_mag=__sqrt(INV1.Vdss*INV1.Vdss +INV1.Vqss*INV1.Vqss);
			}
			else{
				;
			}
			CurrCtrl(&INV1, &INV1_CC, 0);
		}
		if(Flags.Fault){
			PWM1_BUFF_OFF();
		}
		else{
			GATE_RST_OFF();
			PWM1_BUFF_ON();
		}
	}
	else{//Flags.Run1
		ResetCC(&INV1_CC);
		PWM1_BUFF_OFF();
		Time_Chk = 0;
	}// End of Flags.Run1
	
	PwmUpdate(&INV1, &INV1_CC, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs);

    // Active Power & Reactive Power & Apparent Power & Power Factor Calculation
    INV1.Active_Power = 1.5*(INV1.Vdse * INV1.Idse + INV1.Vqse * INV1.Iqse);
    INV1.Reactive_Power = 1.5*(INV1.Vqse * INV1.Idse - INV1.Vdse * INV1.Iqse);

    INV1.Active_Power_f = IIR2Update(&Filter_Active_Power, INV1.Active_Power);
    INV1.Reactive_Power_f = IIR2Update(&Filter_Reactive_Power, INV1.Reactive_Power);

    INV1.Apparent_Power = __sqrt(INV1.Active_Power * INV1.Active_Power + INV1.Reactive_Power * INV1.Reactive_Power);
    INV1.Apparent_Power_f = __sqrt(INV1.Active_Power_f * INV1.Active_Power_f + INV1.Reactive_Power_f * INV1.Reactive_Power_f);

    INV1.Power_Factor = __divf32(INV1.Active_Power,  INV1.Apparent_Power);


    /* Test Code Here */
//    TypeCast_floattou8(3122.14151, u8TestTypeCasing, 1, 2);

    /* Test Code End */

#if 0
	//For PWM_Test
	if(Flags_PWM_Test){
		EPwm1Regs.CMPA.bit.CMPA = CC_PWM_Data1;
		EPwm2Regs.CMPA.bit.CMPA = CC_PWM_Data2;
		EPwm3Regs.CMPA.bit.CMPA = CC_PWM_Data3;
	}// End of PWM_Test
#endif

	/* USER CODE END */
	
	Spd_Con_st++;
	if (Spd_Con_st >= Spd_Con_st_Set){
		Spd_Con_st = 0;
		PieCtrlRegs.PIEIER8.bit.INTx1 = 0x00;//I2CA (drive)
		PieCtrlRegs.PIEIFR5.bit.INTx3 = 0x01;
	}
	Drive_Con_st++;
	if(Drive_Con_st >= Drive_Con_st_Set){
		Drive_Con_st = 0;
//		PieCtrlRegs.PIEIFR5.bit.INTx3 = 0x00;
		PieCtrlRegs.PIEIFR8.bit.INTx1 = 0x01;
	}
	if(fault_cnt)  fault_cc++;
		
	//Scic FIFO OVF monitoring	
	if(ScicRegs.SCIFFRX.bit.RXFFOVF == 1){
		Scic_FIFO_OVF++;
		ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
		ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;
		asm(" NOP");
		asm(" NOP");
		ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
	} 
	else Scic_FIFO_OVF = 0;

	if(SpibRegs.SPIFFRX.bit.RXFFOVF == 1){
		Spib_FIFO_OVF++;
		SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
		SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;
		asm(" NOP");
		asm(" NOP");
		SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;
	}
	SpibRegs.SPIFFRX.bit.RXFFIENA = 1;
	//TEST3_OFF;	
	///////////////////////////////////////////////////////////////////////
	//		CC_Interrupt out
	///////////////////////////////////////////////////////////////////////
	
	//trigger CLA(cla_ISR)
//	EALLOW;
//	//--- Enable use software to start a task (IACK)
//	Cla1Regs.MCTL.bit.IACKE = 1;        // Enable IACKE to start task using software
//	asm("  IACK  #0x007f");             // IACK - CLA task1 force instruction
//	EDIS;
	//trigger CLA(cla_ISR)
	
	u32TimerCNT_temp = ReadCpuTimer0Counter();
	delta_ccTimer0 = (ccTimer0start - u32TimerCNT_temp);
	if(delta_ccTimer0 > max_ccTimer0) max_ccTimer0 = delta_ccTimer0;
	execute_time = delta_ccTimer0*SYS_CLK_PRD;
	TEST1_OFF;							//execute time : about 65[us]
	// Clear INT flag for this timer
  	EPwm1Regs.ETCLR.bit.INT = 1;
  	// Acknowledge this interrupt to receive more interrupts from group 3
  	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
  	asm(" NOP");
}

void UpdateCCGains(Motor *tmpM, CC_3PH *tmpC)
{
  	tmpC->Kid = (tmpM->Rs + tmpC->Ra) * tmpC->Wc;
  	tmpC->Kiq = (tmpM->Rs + tmpC->Ra) * tmpC->Wc;

	tmpC->Kid_T = tmpC->Kid * Tsamp;
	tmpC->Kiq_T = tmpC->Kiq * Tsamp;

	tmpC->Kpd = tmpM->Lds * tmpC->Wc;
//	tmpC->Kad = __divf32(1.,tmpC->Kpd);		// Without Anti-Windup
	tmpC->Kpq = tmpM->Lqs * tmpC->Wc;
//	tmpC->Kaq = __divf32(1.,tmpC->Kpq);		// Without Anti-Windup
	//tmpC->Rv = 0. * tmpM->Rs;
}

void ResetCC(CC_3PH *tmpC)
{
	tmpC->Vdse_Ref_FB	= 0.; 	tmpC->Vdse_Ref_FF 		= 0.;
	tmpC->Vdse_Ref 		= 0.; 	tmpC->Vdse_Ref_Integ 	= 0.;
	tmpC->Vqse_Ref_FB 	= 0.; 	tmpC->Vqse_Ref_FF 		= 0.;
	tmpC->Vqse_Ref 		= 0.; 	tmpC->Vqse_Ref_Integ 	= 0.;
	tmpC->Vdse_Anti 	= 0.; 	tmpC->Vqse_Anti 		= 0.;
	tmpC->Vdss_Ref 		= 0.; 	tmpC->Vqss_Ref 			= 0.;
	tmpC->Vas_Ref = 0.;
	tmpC->Vbs_Ref = 0.;
	tmpC->Vcs_Ref = 0.;
}

Mechanical_Observer_st INV1_Spdobs;
Mechanical_Observer_st INV1_Spdobs_for_Acc;

/****************************************/
/*		Init CC Variables Function		*/
/* Author: Gogume						*/
/* History: Modified 20180314			*/
/****************************************/
void CC_VariUpdate(void)
{
	Init_ADScale();

	KNOW_ANGLE 			= (Uint16)(KEY_GetLinMotorCode(LIN_MOTOR_KNOW_RPOS) + 0.5);
	g_SpiArmStatus.u8LMotAngleFind = KNOW_ANGLE;
	INV1.ThetaOffset 	= KEY_GetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET);
	Flags.RotorPos_est 	= (Uint16)(KEY_GetLinMotorCode(LIN_MOTOR_RPOS_ESTIMAION) + 0.5);
	RotorPos_EstTime 	= (KEY_GetLinMotorCode(LIN_MOTOR_RPOS_ESTI_TIME));
	INV1.Pole 			= 2;
	INV1.PolePitch		= KEY_GetLinMotorCode(LIN_MOTOR_POLE_PITCH)*0.001;
	INV1.InvPolePitch	= 1/INV1.PolePitch;
	INV1.Rs 			= KEY_GetLinMotorCode(LIN_MOTOR_RS);
	INV1.Lds 			= KEY_GetLinMotorCode(LIN_MOTOR_LS)*0.001;
	INV1.Lqs 			= KEY_GetLinMotorCode(LIN_MOTOR_LS)*0.001;
	INV1.LAMpm 			= KEY_GetLinMotorCode(LIN_MOTOR_KF);					// FRLSM: 43.6*1.414/(2*PI*Speed_Max/(Pole_Pitch)) => 0.236
	INV1.Is_rated 		= KEY_GetLinMotorCode(LIN_MOTOR_IQSE_RATE);
	KNOW_ANGLE			= FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_KNOW_RPOS);

	if(KNOW_ANGLE == 0) g_SpiArmStatus.u8LMotAngleFind = 0x00;
	
	EncType 			= (Uint16)(KEY_GetFactoryCode(ENCODERTYPE) + 0.5);
	Wcc 				= KEY_GetFactoryCode(INV_WCC);
	Wsc 				= KEY_GetFactoryCode(WSC);
	INV1.PolePair 		= INV1.Pole*0.5;
	INV1.InvPolePair 	= 1./(float)INV1.PolePair;
	INV1.MaxSpeed		= KEY_GetControlCode(MAX_SPEED);
	INV1.CreepSpeed		= KEY_GetControlCode(CREEP_SPEED);
	INV1.CreepSpeed		= KEY_GetControlCode(INSPECT_SPEED);
	INV1.INV_Rs 		= 1/INV1.Rs;
	INV1.InvLAMpm 		= 1/INV1.LAMpm;
	INV1.Kt				= (1.5*INV1.MaxSpeed*TWOPI/INV1.PolePitch*INV1.LAMpm)*0.5;
	INV1.InvKt 			= 1/INV1.Kt;
	INV1.Jm 			= KEY_GetControlCode(SC_JM);
	//INV1.Jm = 48.99632;//�뜝�룞�삕�뜝�룞�삕 1吏� : 108.513, �뜝�룞�삕�뜝�룞�삕 2吏� : 48.99632
	INV1.InvJm 			= 1/INV1.Jm;
//	INV1.Te_rated 		= INV1.Kt * INV1.Is_rated;
	INV1.Te_rated 		= 390*(GRAVITY+INV1_PROFILE.AccMax);		// RATED LOAD CONDITION : 390kg
	INV1.Iq_max 		= INV1.Is_rated*KEY_GetFactoryCode(THRUST_FORCE_LIMIT)*0.01;
//	INV1.Te_max 		= INV1.Kt*INV1.Iq_max;
	INV1.Te_max 		= 390*1.2*(GRAVITY+INV1_PROFILE.AccMax);	// OVER LOAD CONDITION : 390kg * 120% = 468kg
	INV1.Mass			= 1000;				// MOVER MASS : 100kg
	INV1.FL				= 0.;				// LOAD MASS : 0~390kg
	INV1.FL_gain		= 0.6;				// LOAD MASS GAIN

	InitCCVars(&INV1_CC, TWOPI*Wcc);
	IIR1Init(&Filter_Align, 5.*TWOPI);
	
	/* SW Fault Level Set */
	OC_Set_I			= KEY_GetFactoryCode(INV_OC_SET);
	Normal_VDC			= KEY_GetFactoryCode(INPUT_VOLTAGE);
	OVDC_Set 			= Normal_VDC*1.5;
	UVDC_set			= Normal_VDC - Normal_VDC*0.25;

	/* Current Controller Gain Calculation */
	UpdateCCGains(&INV1, &INV1_CC);
	Reset_Mecha_Obs(&INV1_Spdobs);
	Updata_MachaObse_Gains(&INV1,&INV1_Spdobs);

	ENCA_Data.s1 =  0.000426;
	ENCA_Data.s2 = 1;
	ENCA_Data.b1 = 1;
	ENCA_Data.b2 = 2;
	ENCA_Data.b3 = 1;
	ENCA_Data.a1 = 1;
	ENCA_Data.a2 = -1.940778;
	ENCA_Data.a3 =  0.942482;
}

/****************************************/
/*		Current Transform Function		*/
/* Author: HDEL							*/
/* History: Modified 20180314			*/
/****************************************/
void Current_Transform(Motor *MOT, Uint16 mode){
	
	MOT->Idss = INV_2_3*MOT->Ias - INV_3*MOT->Ibs - INV_3*MOT->Ics;
	if(mode ==1){
		MOT->Iqss = INV_SQRT3*MOT->Ics - INV_SQRT3*MOT->Ibs;
	}
	else{
		MOT->Iqss = INV_SQRT3*MOT->Ibs - INV_SQRT3*MOT->Ics;
	}
	MOT->Inss = SQRT2*INV_3*(MOT->Ias+MOT->Ibs+MOT->Ics);

	if (Flags.AngleAdv) {
		MOT->Cos_ThetaAdv = __cos(MOT->Thetar+1.5*Tsamp*MOT->Wr);
		MOT->Sin_ThetaAdv = __sin(MOT->Thetar+1.5*Tsamp*MOT->Wr);
	}
	else{;}
	MOT->Sin_Theta = __sin(MOT->Thetar);
	MOT->Cos_Theta = __cos(MOT->Thetar);
	MOT->Idse =  MOT->Cos_Theta*MOT->Idss + MOT->Sin_Theta*MOT->Iqss;
	MOT->Iqse = -MOT->Sin_Theta*MOT->Idss + MOT->Cos_Theta*MOT->Iqss;
	if(KNOW_ANGLE == 0){
		MOT->Idse_L = IIR2Update(&Filter_Idqse,MOT->Idse);
		MOT->Iqse_L = IIR2Update(&Filter_Idqse,MOT->Iqse);
	}
}

/****************************************/
/*		Current Controller Function		*/
/* Author: HDEL							*/
/* History: Modified Release 20180314	*/
/****************************************/
void CurrCtrl(Motor *tmpM, CC_3PH *tmpC, Uint16 mode)
{
	float32 Cos, Sin;
	
	Cos = tmpM->Cos_Theta; 		
	Sin = tmpM->Sin_Theta;

	if(mode == 1){
		tmpC->Err_Idse = -(tmpC->Idse_Ref - tmpM->Idse);
		tmpC->Err_Iqse = -(tmpC->Iqse_Ref - tmpM->Iqse);
	}
	else{
		if((Flags.Align1 == 0)&&(Flags.HF_Inj == 1)&&(Flags.Ready1 == 0)){
			tmpC->Err_Idse = tmpC->Idse_Ref - tmpM->Idse_L;
			tmpC->Err_Iqse = tmpC->Iqse_Ref - tmpM->Iqse_L;
		}
		else{
			tmpC->Err_Idse = tmpC->Idse_Ref - tmpM->Idse;
			tmpC->Err_Iqse = tmpC->Iqse_Ref - tmpM->Iqse;
		}
	}
	// d-axis current control
	tmpC->Vdse_Anti = tmpC->Vdse_Ref - tmpM->Vdse;
	tmpC->Vdse_Ref_Integ += tmpC->Kid_T * (tmpC->Err_Idse - tmpC->Kad * tmpC->Vdse_Anti);
	tmpC->Vdse_Ref_FB = tmpC->Kpd * tmpC->Err_Idse + tmpC->Vdse_Ref_Integ;
	if((mode == 0)&&(Flags.Ready1 == 0)) tmpC->Vdse_Ref_FF = 0.;
	else tmpC->Vdse_Ref_FF = - tmpM->Wr * tmpM->Lqs * tmpM->Iqse;

	tmpC->Vdse_Ref = tmpC->Vdse_Ref_FB + tmpC->Vdse_Ref_FF - tmpC->Ra * tmpM->Idse;

	// q-axis current control
	tmpC->Vqse_Anti = tmpC->Vqse_Ref - tmpM->Vqse;
	tmpC->Vqse_Ref_Integ += tmpC->Kiq_T * (tmpC->Err_Iqse - tmpC->Kaq * tmpC->Vqse_Anti);
	tmpC->Vqse_Ref_FB = tmpC->Kpq * tmpC->Err_Iqse + tmpC->Vqse_Ref_Integ;
	if((mode == 0)&&(Flags.Ready1 == 0)) tmpC->Vqse_Ref_FF = 0.;
	else tmpC->Vqse_Ref_FF = tmpM->Wr * tmpM->Lds * tmpM->Idse + tmpM->Wr * tmpM->LAMpm;

	tmpC->Vqse_Ref = tmpC->Vqse_Ref_FB + tmpC->Vqse_Ref_FF - tmpC->Ra * tmpM->Iqse;
	
	// Coordinate change (synchronous  --> stationary)
	tmpC->Vdss_Ref = Cos*tmpC->Vdse_Ref - Sin*tmpC->Vqse_Ref;
	tmpC->Vqss_Ref = Sin*tmpC->Vdse_Ref + Cos*tmpC->Vqse_Ref;

	// Phase voltage
	if((Flags.Align1 == 0)&&(Flags.HF_Inj == 1)&&(Flags.Ready1 == 0)){
		tmpC->Vas_Ref = tmpC->Vdss_Ref + tmpC->Vdss_i;
		tmpC->Vbs_Ref = -0.5*((tmpC->Vdss_Ref + tmpC->Vdss_i) - SQRT3*(tmpC->Vqss_Ref + tmpC->Vqss_i));
		tmpC->Vcs_Ref = -(tmpC->Vas_Ref + tmpC->Vbs_Ref);
	}
	else{
		tmpC->Vas_Ref = tmpC->Vdss_Ref;
		tmpC->Vbs_Ref = -0.5*(tmpC->Vdss_Ref - SQRT3*tmpC->Vqss_Ref);
		tmpC->Vcs_Ref = -(tmpC->Vas_Ref + tmpC->Vbs_Ref);
	}
}

/****************************************/
/*  Init Current Controller Function	*/
/* Author: HDEL							*/
/* History:Initial Release 20180314		*/
/****************************************/
void InitCCVars(CC_3PH *tmpC, float Wc)
{
	tmpC->Idse_Ref 		= 0., 		tmpC->Iqse_Ref 			= 0.;
	tmpC->Idse_Ref_set 	= 0., 		tmpC->Iqse_Ref_set 		= 0.;
	tmpC->Vas_Ref 		= 0., 		tmpC->Vbs_Ref 			= 0., 	tmpC->Vcs_Ref = 0.;
	tmpC->Vdse_Ref_FB 	= 0., 		tmpC->Vdse_Ref_FF 		= 0.;
	tmpC->Vdse_Ref 		= 0., 		tmpC->Vdse_Ref_Integ 	= 0.;
	tmpC->Vqse_Ref_FB 	= 0., 		tmpC->Vqse_Ref_FF 		= 0.;
	tmpC->Vqse_Ref 		= 0., 		tmpC->Vqse_Ref_Integ 	= 0.;
	tmpC->Vdse_Anti 	= 0., 		tmpC->Vqse_Anti 		= 0.;
	tmpC->Vdss_Ref 		= 0., 		tmpC->Vqss_Ref 			= 0.;
	tmpC->Err_Idse 		= 0., 		tmpC->Err_Iqse 			= 0.;
	tmpC->Wc 			= Wc; 		//tmpC->alpha 			= 1.;
	tmpC->Tdead 		= 3.5e-6; 	tmpC->Icomp0 			= 0.;
	tmpC->Ra = 0;

	tmpC->Idss_ref =0., tmpC->Iqss_ref =0.;
	tmpC->Ias_ref =0., tmpC->Ibs_ref=0., tmpC->Ics_ref=0.;

	tmpC->Van_Comp=0., tmpC->Vbn_Comp=0.,tmpC->Vcn_Comp=0.;
	tmpC->Vsat=22.75;;//650V, 3.5us
	tmpC->Isat=2.5;
	tmpC->Ktraps =3.;
	tmpC->Flag_InvComp=0;
	tmpC->Dead_Time_Comp =0;
	tmpC->Ref_Dead_Comp=1;
	tmpC->Flag_Rcontrol =0;

	tmpC->R_6d.wcc = TWOPI*4.;
	tmpC->R_6q.wcc = TWOPI*4.;
	tmpC->R_12d.wcc = TWOPI*4.;
	tmpC->R_12q.wcc = TWOPI*4.;
	tmpC->Flags_All_pass_6dq =0;
	tmpC->Flags_All_pass_12dq =1;
	
	tmpC->Wr_L = 0.;
	tmpC->Wrm_L = 0.;
	tmpC->Wrpm_L = 0.;
}

/****************************************/
/*			ADC Scaling Function		*/
/* Author: Gogume						*/
/* History:Initial Release 20180314		*/
/****************************************/
void Init_ADScale(void)
{
	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
//    ScaleADin_Vdc_1_5kW = KEY_GetFactoryCode(SCALE_VDC)*0.001;
//    ScaleADin_Iac_1_5kW = KEY_GetFactoryCode(SCALE_IS)*0.00001;

          //No Filter
      ScaleAin_SOC0[0]    = ScaleADin_Iac_1_5kW;
      ScaleAin_SOC0[1]    = ScaleADin_Iac_1_5kW;
      ScaleAin_SOC0[2]    = ScaleADin_Iac_1_5kW;
      ScaleAin_SOC0[3]    = ScaleADin_Vdc_1_5kW;
      OffsetAin_SOC0[0]   = (1.5*SenScale_Iac_1_5kW/Rm_Ain_Iac_1_5kW/0.2);
      OffsetAin_SOC0[1]   = (1.5*SenScale_Iac_1_5kW/Rm_Ain_Iac_1_5kW/0.2);
      OffsetAin_SOC0[2]   = (1.5*SenScale_Iac_1_5kW/Rm_Ain_Iac_1_5kW/0.2);
      OffsetAin_SOC0[3]   = (1.5*LV25_Rm_1_5kW/2.5/Rm_Ain_Vdc_1_5kW/0.2);

      ScaleAin_SOC1[0]    = 1;
      ScaleAin_SOC1[1]    = 1;
      ScaleAin_SOC1[2]    = 0.00022888;       // 1/(65536/3*0.4/2)
      ScaleAin_SOC1[3]    = 1;
      OffsetAin_SOC1[0]   = 32768;
      OffsetAin_SOC1[1]   = 32768;
      OffsetAin_SOC1[2]   = 32768;
      OffsetAin_SOC1[3]   = 32768;
}

/****************************************/
/*			ADC Read Function			*/
/* Author: Gogume						*/
/* History:Initial Release 20180314		*/
/****************************************/
void ADC_Read(void)
{
	//forced SW ADC SOC -------------------------------------------
	AdcaRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1  forced start
	AdcbRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1  forced start
	AdccRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1  forced start
	AdcdRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1  forced start
	//forced SW ADC SOC ---------------------12 cyccle-------------

//	cc_cnt++;
//	Time_Chk = Time_Chk + Tsamp;
	///////////////////////////////////////////////////////////////////////
	//		Sensed Analog Value to Physical Quantity
	///////////////////////////////////////////////////////////////////////
	//Wait ADC EOC //SOC �뜝�룞�삕�샇�뜝�룞�삕�뜝�룞�삕 EOC�뜝�룞�삕�뜝�룞�삕 63+149cycle(1.065us, 200MHz �뜝�룞�삕�뜝�룞�삕) �뜝��釉앹삕 -> �뜝�럥�왂 200cycle �뜝�떛�씛�삕 �뜝�룞�삕�뜝�룞�삕�뜝�떦紐뚯삕 �뜝�룞�삕
	Wait_EOC_ADC_SOC0(); // FLAG�뜝�룞�삕 �솗�뜝�룞�삕�뜝�떦�뒗�벝�삕�뜝�룞�삕 66cycle, time out �뜝�뙓�벝�삕 �뜝�떕�꽔�뼲�삕�뜝�룞�삕.. �뜝�떗�슱�삕 �뜝��源띿삕??

	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	//Calculation sensor value(Conveter Current)
	INV1.Ias = (float)AdcaResultRegs.ADCRESULT0*ScaleAin_SOC0[0] - OffsetAin_SOC0[0]; //SOC0�뜝�룞�삕 �뜝�룞�삕�뜝占� CN6, AIN1P_1, ADCINA2/3
	INV1.Ibs = (float)AdcbResultRegs.ADCRESULT0*ScaleAin_SOC0[1] - OffsetAin_SOC0[1]; //SOC0�뜝�룞�삕 �뜝�룞�삕�뜝占� CN6, AIN2P_1, ADCINB2/3
	INV1.Ics = (float)AdccResultRegs.ADCRESULT0*ScaleAin_SOC0[2] - OffsetAin_SOC0[2]; //SOC0�뜝�룞�삕 �뜝�룞�삕�뜝占� CN6, AIN3P_1, ADCINC2/3
	Vdc_rd   = (float)AdcdResultRegs.ADCRESULT0*ScaleAin_SOC0[3] - OffsetAin_SOC0[3]; //SOC0�뜝�룞�삕 �뜝�룞�삕�뜝占� J1(SMA), ADCIND0/1

	//Wait ADC EOC
	Wait_EOC_ADC_SOC1();
	if(Flag_offset == 2) offsetLoopCnt = 0;
	Encoder_HD.fSin_Data = (float)AdcaResultRegs.ADCRESULT1*ScaleAin_SOC1[0] - OffsetAin_SOC1[0];	//SOC1�뜝�룞�삕 �뜝�룞�삕�뜝占� J8, ADCINA0/1
	Encoder_HD.fCos_Data = (float)AdcbResultRegs.ADCRESULT1*ScaleAin_SOC1[1] - OffsetAin_SOC1[1]; 	//SOC1�뜝�룞�삕 �뜝�룞�삕�뜝占� J8, ADCINB0/1
	Thermal = ((float)AdccResultRegs.ADCRESULT1 - OffsetAin_SOC1[2])*ScaleAin_SOC1[2];				//SOC1�뜝�룞�삕 �뜝�룞�삕�뜝占� CN7, ADCINC4/5
	//--------------------------------------44cycle-------------------

	sum_current1 = INV1.Ias+INV1.Ibs+INV1.Ics;
	if((fabs(sum_current1) > 10.)&&(Flag_offset==2)) 	EarCcnt++;
	else EarCcnt = 0;
	if(EarCcnt>=5){
		flag_sumcurrent_error =1.;
		FLT_Raise(FLT_EARTH_I) ;
	}

	// Nyquist Frequency Filter Active Damping for LC Filter
	INV1.Ias_avr = IIR2Update(&Filter_Ias, INV1.Ias);
	INV1.Ibs_avr = IIR2Update(&Filter_Ibs, INV1.Ibs);
	INV1.Ics_avr = IIR2Update(&Filter_Ics, INV1.Ics);

	if(Flags_avr_current){
		INV1.Ias = INV1.Ias_avr;
		INV1.Ibs = INV1.Ibs_avr;
		INV1.Ics = INV1.Ics_avr;
	}
	else{
		INV1.Ias = INV1.Ias;
		INV1.Ibs = INV1.Ibs;
		INV1.Ics = INV1.Ics;
	}

#if 1
	/* Pesudo Vdc Voltage for Test */
	if(Flags.Vdc)
	{
		Vdc_rd = 540;
	}
	else
	{
		Vdc_rd = Vdc_rd;
	}
#endif
}

/****************************************/
/*  	DC MC Operation Function		*/
/* Author: Gogume						*/
/* History:Initial Release 20181113		*/
/****************************************/
void DC_MC_OP(void)
{
	/* Main MC Operation Sequence     */
	/* For DC-Link Magnetic Contactor */
	/* Direct Current Pre-Charging    */
	if((Vdc_rd >= Vdc_relay_on)&&(Flags.Fault==0))
	{
		nMC_ON;					// MC Enable
		Flags.MC_Relay = 1;
	}
	else
	{
		nMC_OFF;				// MC Disable
		Flags.MC_Relay = 0;
	}
}

/****************************************/
/*  Software Level Fault Check Function	*/
/* Author: Gogume						*/
/* History:Initial Release 20181113		*/
/****************************************/
void SW_Fault_Check(void)
{
	/* Current Fault Check */
	if((fabs(INV1.Ibs) > OC_Set_I)||(fabs(INV1.Ias) > OC_Set_I)||(fabs(INV1.Ics) > OC_Set_I)) FLT_Raise(FLT_CC_OCS_I);
	/* DC-Link Voltage Fault Check */
	if(Vdc_rd > OVDC_Set) FLT_Raise(FLT_CC_OVDC);
	if((Vdc_rd < UVDC_set)&&(Flags.MC_Relay)) FLT_Raise(FLT_UVDCS);
}

/****************************************************/
/*  		Applied Speed Variable Function			*/
/* Author: Gogume									*/
/* History:Initial Release 20181113					*/
/* Insert to Control Variable from Encoder Variable */
/****************************************************/
void Applied_Speed_Variable(u8 mode, Motor *tmpM, CC_3PH *tmpC, Encoder_str *p, ENC_DATA_OBJ *pp)
{
	/* Insert Encoder Wr and theta_elect to INV1 */
    if(v_mode == 0){
        switch(mode)
        {
        case __LINEARSCALEMODE:
            tmpM->Wr = p->Wr;
            tmpM->Thetar = p->theta_elec;
            tmpC->Wr_L = IIR1Update(&Filter_Enc,tmpM->Wr);
            tmpC->Wrm_L = (tmpC->Wr_L * tmpM->InvPolePair);

            INV1_SC.Wrm = IIR2Update(&Filter_Wrm, p->speed_mech2);      // Cutoff Frequency 200Hz Lowpass Filter must use for speed measurement.
            INV1_PC.Pos = p->position_;
            break;
        case __HEIDENHAINMODE:
            tmpM->Wr = pp->Wr;
            tmpM->Thetar = pp->theta_elec;
            tmpC->Wr_L = IIR1Update(&Filter_Enc,tmpM->Wr);
            tmpC->Wrm_L = (tmpC->Wr_L * tmpM->InvPolePair);

            INV1_SC.Wrm = IIR2Update(&Filter_Wrm, pp->speed_mech2);      // Cutoff Frequency 200Hz Lowpass Filter must use for speed measurement.
//            INV1_SC.Wrm = pp->speed_mech2;
            INV1_PC.Pos = pp->position;
//            INV1_PC.Pos = pp->positionfA - pp->Zero_Cal;
            break;
        case __MAGNETICSENSORMODE:

            break;
        default:

            break;
        }
    }
}

void Applied_Speed_Observer(u8 mode, Motor *tmpM, CC_3PH *tmpC, Mechanical_Observer_st *tmpObs, Encoder_str *p, ENC_DATA_OBJ *pp)
{
    float32 theta_elec;
    float32 Wr;

	/* For Mechanical Observer */
    switch(mode)
    {
    case __LINEARSCALEMODE:
        theta_elec = p->theta_elec;
        Wr = p->Wr;
        break;
    case __HEIDENHAINMODE:
        theta_elec = pp->theta_elec;
        Wr = pp->Wr;
        break;
    case __MAGNETICSENSORMODE:

       break;
    default:

       break;
    }

	if(Flags.Ready1 && v_mode == 0){
		if(Flag_MecaObs == 1){
			MechObs_SetCnt++;
			if(MechObs_SetCnt < 500){									// Because of set time of observer after reset
			    tmpM->Thetar = theta_elec;
			   	tmpM->Wr = Wr;
				INV1_SC.Wrm = INV1_SC.Wrm;
			}
			else{
				MechObs_SetCnt = 501;
				if(fabs(tmpObs->Wrpm) > OVER_SP_set){
						INV1_SpdObs_OS_FltCnt++;
						if(INV1_SpdObs_OS_FltCnt > 10){
							Flags.Run1 =0;
							FLT_Raise(FLT_OS_OBS);
						}
					}else INV1_SpdObs_OS_FltCnt = 0;

					tmpM->Thetar = tmpObs->Thetar;
					tmpM->Wr = tmpObs->Wr;							// Have to check to compare with INV1.Wr and INV1_Spdobs.Wr;
					INV1_SC.Wrm = tmpObs->Vr;
					/* Load Mass Estimator */
					if(FL_d1 < (tmpObs->Te * INV_GRAVITY)) FL_d1 = (tmpObs->Te / (GRAVITY + INV1_PROFILE.AccRef));
					tmpM->FL = FL_d1 * INV1_SC.SCFF_Gain;				// Load Mass: FL
			}
		}
		else{
			tmpM->Thetar = theta_elec;
			if(Flag_EncFilter == 0){
				tmpM->Wr = Wr;
				INV1_SC.Wrm = INV1_SC.Wrm;

			}
			else{
				tmpM->Wr = tmpC->Wr_L;
				INV1_SC.Wrm = tmpC->Wrm_L;
			}
		}
	}
	else MechObs_SetCnt = 0;
}

void PwmUpdate(Motor *tmpM, CC_3PH *tmpC , volatile struct EPWM_REGS *phA, volatile struct EPWM_REGS *phB, volatile struct EPWM_REGS *phC)
{
	float b,Cos,Sin;
	int i ;
	//-----------------------------------------------------//
	tmpC->Vmax = __fmax(tmpC->Vas_Ref,tmpC->Vbs_Ref);
	tmpC->Vmax = __fmax(tmpC->Vmax	 ,tmpC->Vcs_Ref);
	tmpC->Vmin = __fmin(tmpC->Vas_Ref,tmpC->Vbs_Ref);
	tmpC->Vmin = __fmin(tmpC->Vmin   ,tmpC->Vcs_Ref);
	tmpC->Vsn = -0.5*(tmpC->Vmax + tmpC->Vmin);
	//---------------------46clock-----------------------------------//
	// Pole Voltage
	tmpC->Van_ref = tmpC->Vas_Ref + tmpC->Vsn;
	tmpC->Vbn_ref = tmpC->Vbs_Ref + tmpC->Vsn;
	tmpC->Vcn_ref = tmpC->Vcs_Ref + tmpC->Vsn;
	//--------------�겕�뜝�룞�삕 �뜝�룞�삕�뜝�룞�삕�뜝�룞�삕------------------------------------//
	b = 0.5*Vdc_rd;		// Vdc to Vdc_rd. Modified by GWON 2018.06.28
	tmpC->Van_ref = __fsat(tmpC->Van_ref,b,-b);
	tmpC->Vbn_ref = __fsat(tmpC->Vbn_ref,b,-b);
	tmpC->Vcn_ref = __fsat(tmpC->Vcn_ref,b,-b);

//	if(tmpC->Van_ref > b) 		tmpC->Van_ref = b;
//	else if(tmpC->Van_ref < -b) tmpC->Van_ref = -b;
//	if(tmpC->Vbn_ref > b) 		tmpC->Vbn_ref = b;
//	else if(tmpC->Vbn_ref < -b) tmpC->Vbn_ref = -b;
//	if(tmpC->Vcn_ref > b) 		tmpC->Vcn_ref = b;
//	else if(tmpC->Vcn_ref < -b) tmpC->Vcn_ref = -b;
	//---------------------51->31clock-----------------------------------//

	/*--------------------------*/
	/* Recalculation of Voltage	*/
	/*--------------------------*/
	tmpM->Vdss = (2.*tmpC->Van_ref - tmpC->Vbn_ref - tmpC->Vcn_ref)/3. ;
	tmpM->Vqss = INV_SQRT3*(tmpC->Vbn_ref - tmpC->Vcn_ref);

	// Limited Voltages due to the inverter saturation ( for anti-windup)
	if (Flags.AngleAdv) {
		Cos = tmpM->Cos_ThetaAdv;	Sin = tmpM->Sin_ThetaAdv;
	} else {
		Cos = tmpM->Cos_Theta; 		Sin = tmpM->Sin_Theta;
	}

	tmpM->Vdse = Cos * tmpM->Vdss + Sin * tmpM->Vqss;
	tmpM->Vqse = -Sin * tmpM->Vdss + Cos * tmpM->Vqss;

	// Dead Time Compensation routine is included
	//---------dead time comp------------------------------------------------------//
	InvNonComp(tmpC, tmpM);
	if(tmpC->Dead_Time_Comp){
		tmpC->Van_ref += tmpC->Van_Comp;
		tmpC->Vbn_ref += tmpC->Vbn_Comp;
		tmpC->Vcn_ref += tmpC->Vcn_Comp;
	}
	//---------dead time comp------------------------------------------------------//
	//--------------------------------------------------------//
#if 1
	b = 0.5*Vdc_rd;		// Vdc to Vdc_rd. Modified by GWON 2018.06.28
	tmpC->Van_ref = __fsat(tmpC->Van_ref,b,-b);
	tmpC->Vbn_ref = __fsat(tmpC->Vbn_ref,b,-b);
	tmpC->Vcn_ref = __fsat(tmpC->Vcn_ref,b,-b);

//	if(tmpC->Van_ref > b) 		tmpC->Van_ref = b;
//	else if(tmpC->Van_ref < -b) tmpC->Van_ref = -b;
//	if(tmpC->Vbn_ref > b) 		tmpC->Vbn_ref = b;
//	else if(tmpC->Vbn_ref < -b) tmpC->Vbn_ref = -b;
//	if(tmpC->Vcn_ref > b) 		tmpC->Vcn_ref = b;
//	else if(tmpC->Vcn_ref < -b) tmpC->Vcn_ref = -b;
	//---------------------51->31clock-----------------------------------//
	// Calc. Counter value
	tmpC->pwmTa = halfTsw_PWM*(tmpC->Van_ref*INV_Vdc + 0.5);
	tmpC->pwmTb = halfTsw_PWM*(tmpC->Vbn_ref*INV_Vdc + 0.5);
	tmpC->pwmTc = halfTsw_PWM*(tmpC->Vcn_ref*INV_Vdc + 0.5);
	tmpC->INV_Icomp0 = __divf32(1., tmpC->Icomp0);
	//--------------------------------------------------------//
	// FLOAT to INT conversion.
	i = (int)(SYS_CLK * tmpC->pwmTa + 0.5);
	i = (i > maxCount_ePWM) ? maxCount_ePWM : ((i < 0) ? 0 : i);
	phA->CMPA.bit.CMPA = i;

	i = (int)(SYS_CLK * tmpC->pwmTb + 0.5);
	i = (i > maxCount_ePWM) ? maxCount_ePWM : ((i < 0) ? 0 : i);
	phB->CMPA.bit.CMPA = i;

	i = (int)(SYS_CLK * tmpC->pwmTc + 0.5);
	i = (i > maxCount_ePWM) ? maxCount_ePWM : ((i < 0) ? 0 : i);
	phC->CMPA.bit.CMPA = i;
	//--------------------------------------------------------//
#endif
#if 0
	tmpC->Van_ref = tmpC->Van_ref*SQRT3/Vdc_rd;
	tmpC->Vbn_ref = tmpC->Vbn_ref*SQRT3/Vdc_rd;
	tmpC->Vcn_ref = tmpC->Vcn_ref*SQRT3/Vdc_rd;

	tmpC->Van_ref = __fsat(tmpC->Van_ref,1,-1);
	tmpC->Vbn_ref = __fsat(tmpC->Vbn_ref,1,-1);
	tmpC->Vcn_ref = __fsat(tmpC->Vcn_ref,1,-1);

	tmpC->Cnt_VaRef = halfCount_ePWM + (int)(tmpC->Van_ref*(float)halfCount_ePWM);
	tmpC->Cnt_VbRef = halfCount_ePWM + (int)(tmpC->Vbn_ref*(float)halfCount_ePWM);
	tmpC->Cnt_VcRef = halfCount_ePWM + (int)(tmpC->Vcn_ref*(float)halfCount_ePWM);

	phA->CMPA.bit.CMPA = tmpC->Cnt_VaRef;
	phB->CMPA.bit.CMPA = tmpC->Cnt_VbRef;
	phC->CMPA.bit.CMPA = tmpC->Cnt_VcRef;
#endif
}

void InvNonComp(CC_3PH *tmpC, Motor *tmpM)
{
	tmpC->Idss_ref = tmpM->Cos_Theta * tmpC->Idse_Ref - tmpM->Sin_Theta * tmpC->Iqse_Ref;
	tmpC->Iqss_ref = tmpM->Sin_Theta * tmpC->Idse_Ref + tmpM->Cos_Theta * tmpC->Iqse_Ref;

	tmpC->Ias_ref = tmpC->Idss_ref;
	tmpC->Ibs_ref = - 0.5 * (tmpC->Idss_ref - SQRT3 * tmpC->Iqss_ref);
	tmpC->Ics_ref = - (tmpC->Ias_ref + tmpC->Ibs_ref);

	if(tmpC->Ref_Dead_Comp){
		tmpC->Ias_DComp = tmpC->Ias_ref;
		tmpC->Ibs_DComp = tmpC->Ibs_ref;
		tmpC->Ics_DComp = tmpC->Ics_ref;
	}
	else{
		tmpC->Ias_DComp = tmpM->Ias;
		tmpC->Ibs_DComp = tmpM->Ibs;
		tmpC->Ics_DComp = tmpM->Ics;
	}
	if(tmpC->Flag_InvComp==1){  //trapzoidaL
		tmpC->Van_Comp = tmpC->Ktraps * tmpC->Ias_DComp;
		tmpC->Vbn_Comp = tmpC->Ktraps * tmpC->Ibs_DComp;
		tmpC->Vcn_Comp = tmpC->Ktraps * tmpC->Ics_DComp;

		tmpC->Van_Comp = (tmpC->Van_Comp > tmpC->Vsat)? tmpC->Vsat : ((tmpC->Van_Comp < -tmpC->Vsat)? -tmpC->Vsat : tmpC->Van_Comp);
		tmpC->Vbn_Comp = (tmpC->Vbn_Comp > tmpC->Vsat)? tmpC->Vsat : ((tmpC->Vbn_Comp < -tmpC->Vsat)? -tmpC->Vsat : tmpC->Vbn_Comp);
		tmpC->Vcn_Comp = (tmpC->Vcn_Comp > tmpC->Vsat)? tmpC->Vsat : ((tmpC->Vcn_Comp < -tmpC->Vsat)? -tmpC->Vsat : tmpC->Vcn_Comp);
	}
	else if(tmpC->Flag_InvComp==2){//ATAN
		tmpC->Van_Comp = 2 * tmpC->Vsat * INV_PI * atan(tmpC->Ias_DComp * tmpC->Isat);
		tmpC->Vbn_Comp = 2 * tmpC->Vsat * INV_PI * atan(tmpC->Ibs_DComp * tmpC->Isat);
		tmpC->Vcn_Comp = 2 * tmpC->Vsat * INV_PI * atan(tmpC->Ics_DComp * tmpC->Isat);
	}
	else if(tmpC->Flag_InvComp==3){//ATAN
		if(EPwm1Regs.TBSTS.bit.CTRDIR==1){
			if(tmpC->Ias_DComp >= 0) tmpC->Van_Comp = 2 * tmpC->Vsat;
			else if(tmpC->Ias_DComp < 0) tmpC->Van_Comp = 4 * tmpC->Vsat * INV_PI * atan(tmpC->Ias_DComp * tmpC->Isat) + 2 * tmpC->Vsat;

			if(tmpC->Ibs_DComp >= 0) tmpC->Vbn_Comp = 2 * tmpC->Vsat;
			else if(tmpC->Ibs_DComp < 0) tmpC->Vbn_Comp = 4 * tmpC->Vsat * INV_PI * atan(tmpC->Ibs_DComp * tmpC->Isat) + 2 * tmpC->Vsat;

			if(tmpC->Ics_DComp >= 0) tmpC->Vcn_Comp = 2 * tmpC->Vsat;
			else if(tmpC->Ics_DComp < 0) tmpC->Vcn_Comp = 4 * tmpC->Vsat * INV_PI * atan(tmpC->Ics_DComp * tmpC->Isat) + 2 * tmpC->Vsat;
		}
		else{
			if(tmpC->Ias_DComp <= 0) tmpC->Van_Comp = - 2 * tmpC->Vsat;
			else if(tmpC->Ias_DComp > 0) tmpC->Van_Comp = 4 * tmpC->Vsat * INV_PI * atan(tmpC->Ias_DComp * tmpC->Isat) - 2 * tmpC->Vsat;

			if(tmpC->Ibs_DComp <= 0) tmpC->Vbn_Comp = - 2 * tmpC->Vsat;
			else if(tmpC->Ibs_DComp > 0) tmpC->Vbn_Comp = 4 * tmpC->Vsat * INV_PI * atan(tmpC->Ibs_DComp * tmpC->Isat) - 2 * tmpC->Vsat;

			if(tmpC->Ics_DComp <= 0) tmpC->Vcn_Comp = - 2 * tmpC->Vsat;
			else if(tmpC->Ics_DComp > 0) tmpC->Vcn_Comp = 4 * tmpC->Vsat * INV_PI * atan(tmpC->Ics_DComp * tmpC->Isat) - 2 * tmpC->Vsat;
		}
	}
	else{;}
}

void Rotor_InitPos_Estimation(Motor *tmpM, CC_3PH *tmpC, Uint16 mode)
{
    u8BK_RELEASE_TIMER++;
    if(u8BK_RELEASE_TIMER >= 75){
        u8BK_RELEASE_TIMER = 76;
        BK_RELEASE();
    }

    if(mode == DC_INJECT){
        v_mode =0;
        tmpM->Thetar =0.;
        tmpM->Wr =0.;
        Flag_U = 0;
        Flag_Sin = 0;
        tmpM->Theta_init = 0.;
        tmpM->ThetaOffset = 0.;

        switch(g_IdSpiCoData.u8CarType)
        {
        case __VERTICAL:
            if(g_IdSpiCoData.u8CarNumber == 0x30){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x31){
                ENCA_Data.Zero_Cal = 158.8518677;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x32){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x33){
                ENCA_Data.Zero_Cal = 167.7305756;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x34){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x35){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x36){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x37){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x38){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x39){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else{
                ENCA_Data.Zero_Cal = 0.00;
            }

            break;
        case __HORIZONTAL:
            if(g_IdSpiCoData.u8CarNumber == 0x30){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x31){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x32){
                ENCA_Data.Zero_Cal = 457.0360107;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x33){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x34){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x35){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x36){
                ENCA_Data.Zero_Cal = 349.8845215;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x37){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x38){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else if(g_IdSpiCoData.u8CarNumber == 0x39){
                ENCA_Data.Zero_Cal = 0.00;
            }
            else{
                ENCA_Data.Zero_Cal = 0.00;
            }

            break;
        default:
            ENCA_Data.Zero_Cal = 0.00;
            break;
        }

        ENCA_Data.position = 0.;

        g_SpiArmStatus.u8LMotAngleFind = 0x02;
        g_IdFAData.u8Flag_RotorEstimation = g_SpiArmStatus.u8LMotAngleFind;
        if(Time_Chk > RotorPos_EstTime){
            tmpC->Idse_Ref_set = IIR1Update(&Filter_Align, 0.);
        }
        else{
            tmpC->Idse_Ref_set = IIR1Update(&Filter_Align, (KEY_GetLinMotorCode(LIN_MOTOR_RATING_A)*2.0));
        }
        if(Time_Chk > (RotorPos_EstTime - 1.0)){
            u8BK_RELEASE_TIMER = 0;
            BK_CLOSE();                                                                 // Brake CLOSE
            ENCA_Data.Zero_Cal = ENCA_Data.positionfA;
        }
        tmpC->Iqse_Ref_set = 0.;
        if(Time_Chk > (RotorPos_EstTime+0.1)){                                          // RotorPos_EstTime+1
            Flags.Align1 =0;
            tmpC->Idse_Ref_set = 0.;
            tmpC->Iqse_Ref_set = 0.;

            InitEnc(&Encoder_HD, (Uint32)EncPPR,(float)INV1.PolePair, 0, &EQep2Regs);   // Encoder Variables Initialization
            ENCA_Data.Zero_Cal = ENCA_Data.positionfA;                                  // For Heidenhain Encoder
            ENCA_Data.Taccum = 0;
            INV1_PROFILE.SignProfile = 0;
            ENCA_Data.position = 0.;

            switch(g_Id43Data.u8RxData.u8EncoderType)
            {
            case __LINEARSCALEMODE:
                tmpM->Thetar = Encoder_HD.theta_elec;
                tmpM->Wr = Encoder_HD.Wr;
                break;
            case __HEIDENHAINMODE:
                tmpM->Thetar = ENCA_Data.theta_elec;
                tmpM->Wr = ENCA_Data.Wr;
                break;
            default:
                tmpM->Thetar = Encoder_HD.theta_elec;
                tmpM->Wr = Encoder_HD.Wr;
                break;
            }

            Flags.Ready1 =1;                                                            // A Flag of aling complete. ex) Iinit Position Flag
            g_SpiArmStatus.u8LMotAngleFind = 0x01;
            g_IdFAData.u8Flag_RotorEstimation = g_SpiArmStatus.u8LMotAngleFind;
            Time_Chk = 0.;

            KNOW_ANGLE = 1;

            FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_KNOW_RPOS, KNOW_ANGLE);             // LIN_MOTOR_KNOW_RPOS value change after aling completed.
            KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);                // Storing the end of Electrical Theta to FRAM
            FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);
            KEY_SetControlCode(CURRENT_POS, 0);
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, 0);
            KEY_SetControlCode(ZERO_CAL, ENCA_Data.Zero_Cal);
            FRAM_Write(KEY_CONTROL_START+ZERO_CAL, ENCA_Data.Zero_Cal);
            KEY_SetControlCode(ACCUM, ENCA_Data.Taccum);
            FRAM_Write(KEY_CONTROL_START+ACCUM, ENCA_Data.Taccum);
        }
    }
    else{
        Time_Chk = 0;
        BK_CLOSE();                                                                     // Brake CLOSE
    }
}

void Init_Inv(void)
{
	Flags.Run1 = 0;
	Flags.Align1 = 0;
	Flags.Ready1 = 0;
	Flags.Speed_control_1 = 0;
	Flags.Inv_Run_old = 0;

	INV1_SC.Wrpm_ref = 0.;
	INV1_CC.Iqse_Ref_L = 0.;
	Flag_slope = 0;
	INV1_SC.Te_ref_ff = 0.;
	INV1_SC.Te_ref1 = 0.;
	Flag_Atan = 0 ;
	Flag_Atan_old = 0 ;
	Flag_Sin = 0;
	Flag_U = 0;
	Flag_SinInit = 0 ;
	Iqse_Pos_ff = 0.;
	Flag_Plus = 0;
	Flag_Plus_old = 0;
	Flag_Exit = 0 ;
	Flag_READY = 0 ;
	Pos_real = 0. ;
	ResetSC(&INV1_SC_FF);
	ResetSC(&INV1_SC);
//	RMB_SpdPtnSel = ((int)(KEY_GetFactoryCode(RMB_PTN_SELECT) + 0.5));
}

/********************************************/
/*	Thrust Force Observer for FRLSM			*/
/* Author: SNU & Gogume						*/
/* History: Modified 	       20180704 	*/
/*        : Modified for FRLSM 20180710     */
/*		  : Added Speed Estimation 20180713 */
/********************************************/
#pragma DATA_SECTION(INV1_Spdobs, "CLADataLS0");
Mechanical_Observer_st INV1_Spdobs;
Mechanical_Observer_st INV1_Spdobs_for_Acc;
void Mechanical_Observer(Motor *tmpM, float thetar_est, Mechanical_Observer_st *OBS){
    float Thetam_error_observer;

    OBS->Thetar_error = BOUND_PI(thetar_est - OBS->Thetar);
//	OBS->Te = 1.5 * tmpM->PolePair * (tmpM->LAMpm * tmpM->Iqse + (tmpM->Lds - tmpM->Lqs) * tmpM->Iqse * tmpM->Idse);
//	OBS->Te = 1.5 * tmpM->PolePair * PI_POLE_PITCH * (tmpM->LAMpm * INV1_CC.Iqse_Ref + (tmpM->Lds - tmpM->Lqs) * INV1_CC.Iqse_Ref * INV1_CC.Idse_Ref);
	OBS->Te = INV1.Kt * IIR2Update(&Filter_Obs_Iq_ref, INV1_CC.Iqse_Ref);			// Cutoff 100Hz Lowpass Filter is Applied at q-axis current.
    // Luenberger observer
    Thetam_error_observer = BOUND_PI(OBS->Thetar_error*tmpM->InvPolePair);			// mechanical angle error

    OBS->TL += Tsamp*OBS->L3*Thetam_error_observer;

    OBS->diff_Wrm_observer_model = (OBS->Te - OBS->TL - tmpM->Bm*INV_2PI*tmpM->PolePitch*OBS->Wrm)/tmpM->Jm;
    OBS->diff_Wrm_observer = OBS->diff_Wrm_observer_model + OBS->L2*Thetam_error_observer;
//  OBS->diff_Wrm_observer = (OBS->Te - OBS->TL - tmpM->Bm*OBS->Wrm)/tmpM->Jm + OBS->L2*Thetam_error_observer;
    OBS->Wrm += Tsamp*OBS->diff_Wrm_observer;

    OBS->Thetam_unb += (OBS->L1*Thetam_error_observer + OBS->Wrm)*Tsamp;

    OBS->Thetam += (OBS->L1*Thetam_error_observer + OBS->Wrm)*Tsamp;
    OBS->Thetam = BOUND_PI(OBS->Thetam);

    OBS->Thetar = OBS->Thetam*tmpM->PolePair + tmpM->Theta_init;
    OBS->Thetar = BOUND_PI(OBS->Thetar);
    OBS->Wr = OBS->Wrm*tmpM->PolePair;
    OBS->Vr = OBS->Wrm*INV_2PI*tmpM->PolePitch;
}

void Updata_MachaObse_Gains(Motor *tmpM, Mechanical_Observer_st *OBS){
	float wn;
	wn = OBS->Wbw;

	OBS->L1  = 2*wn - tmpM->Bm*INV_2PI*tmpM->PolePitch*tmpM->InvJm;
	OBS->L2  = -OBS->L1*tmpM->Bm*INV_2PI*tmpM->PolePitch*tmpM->InvJm+2.*wn*wn;
	OBS->L3  = -tmpM->Jm*wn*wn*wn;
}

void Reset_Mecha_Obs(Mechanical_Observer_st *OBS){
	OBS->Thetam_unb =0.;
	OBS->Thetar_error=0.;
	OBS->Te=0.;
	OBS->TL=0., OBS->Wrm=0., OBS->Thetam=0.;
	OBS->Wr=0., OBS->Wrpm=0., OBS->Thetar=0., OBS->Vr=0.;
	if(EncType == SIN_COS_ENC) OBS->Wbw =TWOPI*100.;
	else OBS->Wbw =TWOPI*30.;
	OBS->diff_Wrm_observer =0.;
	OBS->diff_Wrm_observer_model =0.;
	OBS->Thetam_dir = 1.;
}

void Calc_APS_POS_SPD(){
	if(g_SpiDspStatus.u8APS1PositionSign == 0x30){
		INV1_PC.APSPos = ((float32)g_SpiDspStatus.CurrentPosition1)*0.0005;
	}
	else if(g_SpiDspStatus.u8APS1PositionSign == 0x31){
		INV1_PC.APSPos = -1*((float32)g_SpiDspStatus.CurrentPosition1)*0.0005;
	}
	INV1_PC.APSTheta = BOUND_PI(INV1_PC.APSPos*TWOPI*INV1.InvPolePitch);

	INV1_PC.LajerPosUp = ((float32)g_SpiDspStatus.LaserDistanceUp);
	INV1_PC.LajerPosDown = ((float32)g_SpiDspStatus.LaserDistanceDown);

	INV1_SC.APSWrm1 = ((float32)g_SpiDspStatus.CurrentVelocity1)*0.001;
	INV1_SC.APSWrm2 = ((float32)g_SpiDspStatus.CurrentVelocity2)*0.001;
}

void Calc_Hiedenhain_POS_SPD(ENC_DATA_OBJ *p)
{
    /* Data Conversion from SPI Encoder BD */
    p->positionA  = (((u32)p->rdata[1]) << 24);
    p->positionA += (((u32)p->rdata[2]) << 16);
    p->positionA += (((u32)p->rdata[3]) << 8);
    p->positionA += (((u32)p->rdata[4]) << 0);

    p->positionB  = (((u32)p->rdata[5]) << 24);
    p->positionB += (((u32)p->rdata[6]) << 16);
    p->positionB += (((u32)p->rdata[7]) << 8);
    p->positionB += (((u32)p->rdata[8]) << 0);

    if(u8Caltimer >= 100)
    {
        p->positionfA = ((p->positionA * 593.75) / 262144);                   // 593.75mm/unit; output: 2^18 = 262144
        p->positionfB = ((p->positionB * 593.75) / 262144);                   // 593.75mm/unit; output: 2^18 = 262144

        u8Caltimer = 101;
    }
    else
    {
        p->positionfA = ((p->positionA * 593.75) / 262144);                   // 593.75mm/unit; output: 2^18 = 262144
        p->positionfB = ((p->positionB * 593.75) / 262144);                   // 593.75mm/unit; output: 2^18 = 262144
        p->Calibration = p->positionfB - p->positionfA;
        u8Caltimer++;
    }

    p->positionfA_f = IIR2Update(&Filter_Hiedenhain_A, p->positionfA);        // Position A
    p->positionfB_f = IIR2Update(&Filter_Hiedenhain_B, p->positionfB);        // Position B

    p->diff = p->positionfA_f - p->positionfB_f;

    /* Insert Encoder A/B Switching Algorithm Here */
    /* Data Switching Algorithm START */
//    g_SpiDspStatus.u8EncoderInfo = 0x31; // for test
    Flag_ENC_Sel = (int)(g_SpiDspStatus.u8EncoderInfo - 0x30);
    if(INV1_PROFILE.SignProfile == 1)   // Direction
    {
        if(g_SpiDspStatus.u8EncoderInfo == 0x31 && u8EncoderInfo_d1 == 0x00)
        {
            p->selPosition = p->positionfA;
            u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x32 &&  u8EncoderInfo_d1 == 0x00)
        {
            p->selPosition = p->positionfB;
            u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x32 && u8EncoderInfo_d1 == 0x31)
        {
            p->Offset_Cal = p->selPosition - p->positionfB;

            p->selPosition = p->positionfB + p->Offset_Cal;
            u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x31 && u8EncoderInfo_d1 == 0x32)
        {
            p->Offset_Cal = p->selPosition -p->positionfA;

            p->selPosition = p->positionfA + p->Offset_Cal;
            u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x32 && u8EncoderInfo_d1 == 0x32)
        {
            p->selPosition = p->positionfB + p->Offset_Cal;
           u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x31 && u8EncoderInfo_d1 == 0x31)
        {
            p->selPosition = p->positionfA + p->Offset_Cal;
           u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
    }
    else if(INV1_PROFILE.SignProfile == -1) // Direction
    {
        if(g_SpiDspStatus.u8EncoderInfo == 0x31 && u8EncoderInfo_d1 == 0x00)
        {
            p->selPosition = p->positionfA;
            u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x32 &&  u8EncoderInfo_d1 == 0x00)
        {
            p->selPosition = p->positionfB;
            u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x32 && u8EncoderInfo_d1 == 0x31)
        {
            p->Offset_Cal = p->selPosition - p->positionfB;

            p->selPosition = p->positionfB + p->Offset_Cal;
            u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x31 && u8EncoderInfo_d1 == 0x32)
        {
            p->Offset_Cal = p->selPosition - p->positionfA;

            p->selPosition = p->positionfA + p->Offset_Cal;
            u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x32 && u8EncoderInfo_d1 == 0x32)
        {
            p->selPosition = p->positionfB + p->Offset_Cal;
           u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
        else if(g_SpiDspStatus.u8EncoderInfo == 0x31 && u8EncoderInfo_d1 == 0x31)
        {
            p->selPosition = p->positionfA + p->Offset_Cal;
           u8EncoderInfo_d1 = g_SpiDspStatus.u8EncoderInfo;
        }
    }
    /* Data Switching Algorithm END */

    /* Position Accumulation Algorithm START */
    p->selPosition_diff = p->selPosition - p->selPosition_dd1;
    p->selPosition_dd1 = p->selPosition;

    if(fabs(p->selPosition_diff) > 500)
    {
        if(p->selPosition_diff > 0) p->Taccum--;
        else if(p->selPosition_diff < 0) p->Taccum++;
    }
    /* Position Accumulation Algorithm END */
    /* Real Position Calculation */
    p->realpositionA = p->selPosition + ((float32)p->Taccum * 593.75);      // [mm]
    /* Zero Position Setting */
    p->realpositionA = p->realpositionA - p->Zero_Cal;                      // [mm]
    /* Electrical Angle Calculation */
    p->thetaA = BOUND_PI(p->realpositionA * TWOPI*INV1.InvPolePitch * 0.001);
    /* Speed Calculation */
    p->floorPosition = (float32)((int32)(p->realpositionA * 100)) * 0.01;        // floor, 0.001
    p->spdA = __divf32((float)(p->floorPosition - p->positionfA_d1), Tsamp);     // [mm/sec]
    p->positionfA_d1 = p->floorPosition;
    p->spdA_f = IIR2Update(&Filter_Hiedenhain_Spd_A, p->spdA * 0.001);

//    p->LPF_spd = p->s1*p->s2*p->a1*(p->b1*p->spdA + p->b2*p->spd_d1 + p->b3*p->spd_d2)-p->a1*(p->a2*p->LPF_spd_d1+p->a3*p->LPF_spd_d2);
//    p->LPF_spd_d2 = p->LPF_spd_d1;
//    p->LPF_spd_d1 = p->LPF_spd;
//    p->spd_d2 = p->spd_d1;
//    p->spd_d1 = p->spdA;

    /* Variables Matching */
    p->theta_elec = p->thetaA;
    p->position = p->realpositionA* 0.001;                                  // [m]
    p->speed_mech2 = p->spdA_f;                                             // [m/s]
    /* Electrical Angular Frequency */
    p->Wr = __divf32(TWOPI * p->speed_mech2, INV1.InvPolePitch);            // [rad/sec]
}
