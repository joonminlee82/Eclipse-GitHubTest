/*****************************************************************************
-------------------------------Organization------------------------------------
	Project				: FREEWAY
	Compiler    		: TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
	Author				: Inverter part, Advanced Controls Research Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.12.28
	History				: Linear Motor Selection Menu is added into the HHT Menu (20180206 by GWON)
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
******************************************************************************/
#include <math.h>
#include "F28x_Project.h" 
#include "Variable.h"
#include "drive.h"
#include "Fault.h"
#include "bdib.h"
#include "CC.h"
#include "Device.h"
#include "KeyMenu.h"
#include "FRAM.h"
//#include "fc.h"
#include "SC.h"
#include "vane.h"
#include "SCI.h"
#include "SPI.h"
#include "comp.h"
#include "gnl.h"
#include "eQEP_Encoder.h"

#define ENC_CNT_OFFSET 1

Drive 	DRV ;
Olpd 	OLD;
extern	Flt 			FLT;
extern  STATUS_ARM_OBJ   g_ArmStatus;
extern 	BDIB 			g_BDIB;
extern	STATUS_DSP_OBJ 	g_SpiDspStatus;
extern  STATUS_ARM_OBJ  g_SpiArmStatus;
STATUS_DSP_OBJ 	g_PreSpiDspStatus;
extern  Id45Data        g_Id45Data;
extern  ENC_DATA_OBJ ENCA_Data;

extern u8 g_Mode_Check;

extern PC           INV1_PC;

//extern 	FLOOR 		Floor;
extern  float ELD_UV_set, delta_DecI;
extern int DLA_ErrCnt1, DLA_ErrCnt2;
extern Byte SCI_CRC_Err;
extern Uns DblSpd_Msw_Use;
extern Uint16 Scic_rxd_Cnt;

Uns	cntConv =0 ;
Uns DblCmd_cnt = 0, iCntDrive = 0, Drv_Bko_CNT = 0, RunCnt = 0, Drv_Bko_CNT_Lmt = 10, RST_Offset = 0;
Uns iCnt = 0,SCIC_RDY_Cnt = 0 ;
Uns	ReadyHoldTime = 60, READYCnt = 0, FluxON_Cnt = 0, FluxHoldTime = 4, DCBON_Cnt = 0, DCBHoldTime = 100,	flgRefOff =0 ;
Uns DRVCnt = 0, DrvStat_Drive = 0 ;
Uns Scic_rxd_Cnt_old = 0, MCP_COM_CNT = 0, MCP_COM_ERR_Flg = 0 ;
Uns PosTqCom_Flg = 0;
Uns FlagSINCOSErr_C = 0, FlagSINCOSErr_D = 0 ;
Uns Converter_TestFlg = 0;

float Vari_Tans_Level = 0.5,  SpdCtlErrLevel = 20. ;
extern float ZspLevel = 0.005;
float Wc_tq = 10.;		//10Hz*50(Tsmap) =  500Hz
float DRV_La_tq = 0., DRV_Lb_tq = 0.;
float fEepromRead1 = 0. ;
float Thetar_Rst_Vdse = 0., Thetar_Rst_Vdse_Insp = 0.;
float fTqunbal_Gain = 0., fINV_MaxRpm = 0.;
float LoadtoTq = 0., INV_per = 0. ;
float delta_DecI_D = 0.1 ;
Uint16 FRAM_Index = 0, FRAM_TestFlg = 0 ;
Uint16 CP_WD_Cnt = 0, CP_WD_ERR_Flg = 0;
long E_Check_tmp = 0;
float step_temp = 0.;
float step_temp_old = 0.;
Uns Bkop_Cnt = 0, Bkcloseop_Cnt = 0;

float TestJS = 0.;			// For Test

Uint16 Flag_BK_Test = 0;	// For BK Test

Uint16 First_Call = 1;

extern HW_TX_OBJ	Hw_TX_Data;
extern HW_RX_OBJ 	Hw_RX_Data;

u8 RunMode_Check = 0;
float32 distance_temp = 0;
u8 TX_Size = 0;
u8 u8INV_RUN_RDY = 0;
u8 u8SwResetCnt = 0;
u8 u8FlrInitErrCnt = 0;

int TestBKClose = 0;
int BKCloseStatusNum = 0;
u8 u8BkReleaseDelay = 250;      // 500msec
u8 u8OpMode = 0;
u8 u8FanOffTimer = 500;         // 1sec
u8 u8FanOffCnt = 0;
u8 u8Possible_Decel = 0;
u8 u8TestForceDecel = 0;

__interrupt void cpu1_drive(void)  
{
	IER &= 0x0000;
	IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT6 | M_INT8 | M_INT10 | M_INT12 ;				// Fault, CC, Vdc_Control, SC, SPIB, SCIC int Enable
	EINT;	
	
	Drive_Cnt++;

	if(Drive_Cnt>1000)		Drive_Cnt = 1001 ;	//占쏙옙占� ON 占쏙옙, SCIB 占십깍옙화 占쏙옙 MCU 占쏙옙占� 占쌉뤄옙 占시곤옙(3.2S)
	
	/* Block Device Input Block(Digital Input(GPIO) Status Update) */
	BDIB_UpdateAllStatus();	
	/* FRAM Status Check */	
	if(FRAM_TestFlg){
		if(FRAM_TestFlg == 1) FRAM_Write(FRAM_Index,E_Check_tmp);
		NOP4();
		fEepromRead1 = FRAM_Read(FRAM_Index);
		FRAM_TestFlg = 0;
	}

	/* CP Watchdog Check */
	if(GATE_CUT_3V == 1){
		CP_WD_Cnt++;
		if(CP_WD_Cnt > 200)
		{
			CP_WD_Cnt = 0;
			CP_WD_ERR_Flg = 1;
			FLT_Raise(FLT_CP_WD);
		}
	}
	else{
		CP_WD_Cnt = 0;
		CP_WD_ERR_Flg = 0;
	}

	/* FAN Control START */
	if((Flags.Run1 == 1) && (RunMode_Check != OUTPUT_DEBUG))
	{
	    nINV_FAN_ON;
	}
	else
	{
	    if(u8FanOffCnt >= u8FanOffTimer){
	        u8FanOffCnt = u8FanOffTimer + 1;
	        nINV_FAN_OFF;
	    }
	    u8FanOffCnt++;

	}
	/* FAN Control END */

	/* Drive Status Update */
	DRV_UpdateStatus();
	/* Bit operation */
	DrvStat_Drive = DRV_GetStatus();
	TestJS = DriveStatPsudo;			// For Test
	/* Operating Sequence by DrvStat_Drive  */
	/* If both BDIB_UP and BDIB_DOWN are set, it is command falut state. */
	if((g_BDIB.regStatusBit.UP == 1)&&(g_BDIB.regStatusBit.DOWN == 1)){
		DblCmd_cnt++;
		if(DblCmd_cnt > 6)	FLT_Raise(FLT_DBL_CMD);
	}
	else DblCmd_cnt =0;
	
	iCntDrive ++ ;
	
	/* Flag setting for Inverter Run */
	/* USER CODE HERE */
	/* For QT Test Program Operation */
	if(g_Mode_Check == __OUTPUT_DEBUG_MODE) u8OpMode = OUTPUT_DEBUG;
	else u8OpMode = g_PreSpiDspStatus.u8RunMode;

	switch(u8OpMode){
	case STOP_DRIVE:			// Normal Stop Mode
		RunMode_Check = STOP_DRIVE;

		INIT_DRV = 0;

		Flags.ProfileMode = 0;
		step_temp = INV1_PC.Pos;
		step_temp_old = step_temp;
		INV1_PC.Spd_ref_max = 0.05;
		INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;

		if(fabs(INV1_SC.Wrm) <= ZspLevel){		// Zero speed Level: 0.005m/s
			nINV_ZSP_ON;						// Notify Zero Speed to MCU
			Flag_ZSP = 1;
		}
		else{
			nINV_ZSP_OFF;
			Flag_ZSP = 0;
		}

		break;
	case INIT_DRIVE:			// Measuring Floor Height Mode
		RunMode_Check = INIT_DRIVE;

		INIT_DRV = 1;
		nINIC_ON;

		// Inserted by GWON 2090304. Need to verify. START
		KEY_SetControlCode(INIT_START, INIT_DRV);
        FRAM_Write(KEY_CONTROL_START+INIT_START, INIT_DRV);
		g_SpiArmStatus.u8FlInitOK = 0x02;       // Floor Height Operation status Check
		if(g_SpiArmStatus.u8FlInitOK =! g_SpiDspStatus.InitOpStatus){
		    u8FlrInitErrCnt++;
		    if(u8FlrInitErrCnt >= 100){              // Floor Init Error is occurred over the 100 time(500msec).
		        g_SpiArmStatus.u8FlInitOK = 0x00;
                g_SpiArmStatus.u8FlInitWng = 0x01;
		    }
		}
		else u8FlrInitErrCnt = 0;
		// Inserted by GWON 2090304. Need to verify. END
        Flags.Position_control = 1;
        Flags.ProfileMode = 1;
        Flags.CompProfile = 1;

        if(Flags.Ready1 == 1 && Flags.Run1 == 1){

            if((g_SpiDspStatus.u8RunStatus == RUN_UP)||(g_SpiDspStatus.u8RunStatus == RUN_RIGHT))
            {
                step_temp = ((float32)g_SpiDspStatus.SetPosition)*0.0001;
            }
            else if((g_SpiDspStatus.u8RunStatus == RUN_DOWN)||(g_SpiDspStatus.u8RunStatus == RUN_LEFT))
            {
                step_temp = (-1*(float32)g_SpiDspStatus.SetPosition)*0.0001;
            }

            if(step_temp != step_temp_old)
            {
                distance_temp = step_temp - step_temp_old;          // For Direction and Moving Distance
            }

            INV1_PROFILE.SpeedMax   = KEY_GetControlCode(INSPECT_SPEED);
            INV1_PC.Spd_ref_max     = INV1_PROFILE.SpeedMax * 1.2;
            INV1_PROFILE.AccMax     = 0.005;
            INV1_PROFILE.JerkMax    = INV1_PROFILE.AccMax;
            INV1_PROFILE.POS_CMD    = step_temp;
            g_SpiArmStatus.u8LowSpdOp = 0x01;

            step_temp_old = step_temp;

            INV1_PC.Remain_Distance = INV1_PROFILE.POS_CMD - INV1_PC.Pos;
            if((fabs(INV1_SC.Wrm) <= ZspLevel)&&(fabs(INV1_PC.Remain_Distance) <= 0.001) && g_SpiArmStatus.u8LMotAngleFind == 0x01){     // Zero speed Level: 0.005m/s
                nINV_ZSP_ON;                        // Notify Zero Speed to MCU
                Flag_ZSP = 1;
            }
            else{
                nINV_ZSP_OFF;
                Flag_ZSP = 0;
            }
        }
        else{
            step_temp = INV1_PC.Pos;
            step_temp_old = step_temp;
            /* Align Sequence Code */
            nINV_ZSP_ON;                        // Notify Zero Speed to MCU
            Flag_ZSP = 1;
            /* Align Sequence Code End */
        }

        if((Flag_ZSP == 1)&&(BKO == TRUE)&&(KNOW_ANGLE == 1))
        {
            KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);            // Storing the end of Electrical Theta to FRAM
            FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);
            KEY_SetControlCode(ACCUM, ENCA_Data.Taccum);
            FRAM_Write(KEY_CONTROL_START+ACCUM, ENCA_Data.Taccum);
            KEY_SetControlCode(SEL_POS, ENCA_Data.selPosition);
            FRAM_Write(KEY_CONTROL_START+SEL_POS, ENCA_Data.selPosition);
#ifdef ENC_CNT_OFFSET
            KEY_SetControlCode(CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
#else
            KEY_SetControlCode(CURRENT_POS, INV1_PC.Pos);
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, INV1_PC.Pos);
#endif

            Init_PROFILE();
            INV1_PROFILE.POS_CMD = INV1_PC.Pos;
            INV1_PROFILE.PosRef = INV1_PC.Pos;
            INV1_PROFILE.PosRefFinal = INV1_PC.Pos;
            INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;
            INV1_PC.PosRefFinal = INV1_PROFILE.PosRefFinal;
        }

		break;
	case TEST_DRIVE:			// Lab Test Mode : Static Force Test Mode
		RunMode_Check = TEST_DRIVE;

		INIT_DRV = 0;

		Flags.Position_control = 1;
		Flags.ProfileMode = 1;
		Flags.CompProfile = 1;

		if(Flags.Ready1 == 1 && Flags.Run1 == 1){
			if((g_SpiDspStatus.u8RunStatus == RUN_UP)||(g_SpiDspStatus.u8RunStatus == RUN_RIGHT))
			{
				step_temp = ((float32)g_SpiDspStatus.SetPosition)*0.0001;
			}
			else if((g_SpiDspStatus.u8RunStatus == RUN_DOWN)||(g_SpiDspStatus.u8RunStatus == RUN_LEFT))
			{
				step_temp = (-1*(float32)g_SpiDspStatus.SetPosition)*0.0001;
			}


			if(step_temp != step_temp_old)
			{
				distance_temp = step_temp - step_temp_old;			// For Direction and Moving Distance
			}

			if(fabs(distance_temp) > 1.6){
			    INV1_PC.Spd_ref_max     = KEY_GetFactoryCode(OS_SET);
			    INV1_PROFILE.SpeedMax   = KEY_GetControlCode(MAX_SPEED);
			    INV1_PROFILE.AccMax     = KEY_GetControlCode(MAX_ACC);
			    INV1_PROFILE.JerkMax    = KEY_GetControlCode(MAX_JERK);
			    INV1_PROFILE.POS_CMD    = step_temp;
			    g_SpiArmStatus.u8LowSpdOp = 0x00;
            }
			else if((fabs(distance_temp) >= 0.1)&&(fabs(distance_temp) <= 1.6)){
			    INV1_PROFILE.SpeedMax   = KEY_GetControlCode(CREEP_SPEED);
			    INV1_PC.Spd_ref_max     = INV1_PROFILE.SpeedMax * 1.2;
			    INV1_PROFILE.AccMax     = fabs(distance_temp) * 0.01;
                INV1_PROFILE.JerkMax    = INV1_PROFILE.AccMax;
                INV1_PROFILE.POS_CMD    = step_temp;
                g_SpiArmStatus.u8LowSpdOp = 0x01;
			}
			else if((fabs(distance_temp) < 0.1)){
                INV1_PROFILE.SpeedMax   = KEY_GetControlCode(CREEP_SPEED);
                INV1_PC.Spd_ref_max     = INV1_PROFILE.SpeedMax * 1.2;
                INV1_PROFILE.AccMax     = fabs(distance_temp) * 0.01;
                INV1_PROFILE.JerkMax    = INV1_PROFILE.AccMax;
                INV1_PROFILE.POS_CMD = step_temp;
                g_SpiArmStatus.u8LowSpdOp = 0x02;
			}
			step_temp_old = step_temp;

			/* Decelerelation Sequence */
//			g_SpiDspStatus.u8PreLoadStatus
			switch(u8TestForceDecel){
			case NOMAL_OP:
			    Flags.Decel = 0;
			    u8Possible_Decel = 0;
			    break;
			case FORCE_DECEL_OP:
			    if(fabs(INV1_PC.Remain_Distance) >= INV1_PROFILE.PosRefc)
			    {
			        u8Possible_Decel = 1;
			    }
                else                                                        // Emergency Stop
                {
                    BK_CLOSE();
                }

			    if(u8Possible_Decel == 1)
			    {
			        Flags.Decel = 1;
			    }

			    break;
			case FIRST_IN_POSITION_OP:
			    Flags.Decel = 2;
			    break;
			case SECOND_IN_POSITION_OP:
			    Flags.Decel = 3;
			    break;
			default:
			    Flags.Decel = 0;
			    break;
			}

			if(Flags.Decel == 1 && INV1_PROFILE.TCH == 1)
			{
			    INV1_PROFILE.POS_CMD = INV1_PC.Pos;
			    INV1_PROFILE.POS_CMD_old = INV1_PROFILE.POS_CMD;
			}

			INV1_PC.Remain_Distance = INV1_PROFILE.POS_CMD - INV1_PC.Pos;
			if((fabs(INV1_SC.Wrm) <= ZspLevel)&&(fabs(INV1_PC.Remain_Distance) <= 0.001) && g_SpiArmStatus.u8LMotAngleFind == 0x01){     // Zero speed Level: 0.005m/s
			    nINV_ZSP_ON;                        // Notify Zero Speed to MCU
                Flag_ZSP = 1;
            }
            else{
                nINV_ZSP_OFF;
                Flag_ZSP = 0;
            }
		}
		else{
		    step_temp = INV1_PC.Pos;
		    step_temp_old = step_temp;
			/* Align Sequence Code */
		    nINV_ZSP_ON;                        // Notify Zero Speed to MCU
            Flag_ZSP = 1;
			/* Align Sequence Code End */
		}

		if((Flag_ZSP == 1)&&(BKO == TRUE)&&(KNOW_ANGLE == 1))
		{
            KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);	        // Storing the end of Electrical Theta to FRAM
            FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);
            KEY_SetControlCode(ACCUM, ENCA_Data.Taccum);
            FRAM_Write(KEY_CONTROL_START+ACCUM, ENCA_Data.Taccum);
            KEY_SetControlCode(SEL_POS, ENCA_Data.selPosition);
            FRAM_Write(KEY_CONTROL_START+SEL_POS, ENCA_Data.selPosition);
#ifdef ENC_CNT_OFFSET
            KEY_SetControlCode(CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
#else
            KEY_SetControlCode(CURRENT_POS, INV1_PC.Pos);
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, INV1_PC.Pos);
#endif

            Init_PROFILE();
            INV1_PROFILE.POS_CMD = INV1_PC.Pos;
            INV1_PROFILE.PosRef = INV1_PC.Pos;
            INV1_PROFILE.PosRefFinal = INV1_PC.Pos;
            INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;
            INV1_PC.PosRefFinal = INV1_PROFILE.PosRefFinal;
		}

		break;
	case EMERGENCY_DRIVE:		// Emergency Stop Mode
	    RunMode_Check = EMERGENCY_DRIVE;

	    BKCloseStatusNum = 2;
	    BK_CLOSE();

	    if((fabs(INV1_SC.Wrm) <= ZspLevel)){        // Zero speed Level: 0.005m/s
            nINV_ZSP_ON;                            // Notify Zero Speed to MCU
            Flag_ZSP = 1;
        }
        else{
            nINV_ZSP_OFF;
            Flag_ZSP = 0;
        }

        if((Flag_ZSP == 1)&&(BKO == TRUE)&&(KNOW_ANGLE == 1))
        {
            KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);    // Storing the end of Electrical Theta to FRAM
            FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);
            KEY_SetControlCode(ACCUM, ENCA_Data.Taccum);
            FRAM_Write(KEY_CONTROL_START+ACCUM, ENCA_Data.Taccum);
            KEY_SetControlCode(SEL_POS, ENCA_Data.selPosition);
            FRAM_Write(KEY_CONTROL_START+SEL_POS, ENCA_Data.selPosition);
#ifdef ENC_CNT_OFFSET
            KEY_SetControlCode(CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
#else
            KEY_SetControlCode(CURRENT_POS, INV1_PC.Pos);
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, INV1_PC.Pos);
#endif
            Init_PROFILE();
            INV1_PROFILE.POS_CMD = INV1_PC.Pos;
            INV1_PROFILE.PosRef = INV1_PC.Pos;
            INV1_PROFILE.PosRefFinal = INV1_PC.Pos;
            INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;
            INV1_PC.PosRefFinal = INV1_PROFILE.PosRefFinal;
        }
		break;
	case RE_LEVEL_DRIVE:
	    RunMode_Check = RE_LEVEL_DRIVE;

	    Flags.Position_control = 1;
	    Flags.ProfileMode = 1;

	    if(Flags.Ready1 == 1 && Flags.Run1 == 1){
            if((g_SpiDspStatus.u8RunStatus == RUN_UP)||(g_SpiDspStatus.u8RunStatus == RUN_RIGHT))
            {
                step_temp = ((float32)g_SpiDspStatus.SetPosition)*0.0001;
            }
            else if((g_SpiDspStatus.u8RunStatus == RUN_DOWN)||(g_SpiDspStatus.u8RunStatus == RUN_LEFT))
            {
                step_temp = (-1*(float32)g_SpiDspStatus.SetPosition)*0.0001;
            }

            if(step_temp != step_temp_old)
            {
                distance_temp = step_temp - step_temp_old;          // For Direction and Moving Distance
            }

            INV1_PROFILE.SpeedMax   = KEY_GetControlCode(RELEVEL_SPEED);
            INV1_PC.Spd_ref_max     = INV1_PROFILE.SpeedMax * 1.2;
            INV1_PROFILE.AccMax     = fabs(distance_temp) * 0.04;
            INV1_PROFILE.JerkMax    = INV1_PROFILE.AccMax;
            INV1_PROFILE.POS_CMD = step_temp;

            step_temp_old = step_temp;

            if((fabs(INV1_SC.Wrm) <= ZspLevel)&&(fabs(INV1_PC.Err_Pos) <= 0.005)){      // Zero speed Level: 0.005m/s
                nINV_ZSP_ON;                                                            // Notify Zero Speed to MCU
                Flag_ZSP = 1;
            }
            else{
                nINV_ZSP_OFF;
                Flag_ZSP = 0;
            }

	    }
	    else{
            /* Align Sequence Code */
            nINV_ZSP_ON;                        // Notify Zero Speed to MCU
            Flag_ZSP = 1;
            /* Align Sequence Code End */
        }

        if((Flag_ZSP == 1)&&(BKO == TRUE)&&(KNOW_ANGLE == 1))
        {
            KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);    // Storing the end of Electrical Theta to FRAM
            FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);
            KEY_SetControlCode(ACCUM, ENCA_Data.Taccum);
            FRAM_Write(KEY_CONTROL_START+ACCUM, ENCA_Data.Taccum);
            KEY_SetControlCode(SEL_POS, ENCA_Data.selPosition);
            FRAM_Write(KEY_CONTROL_START+SEL_POS, ENCA_Data.selPosition);
#ifdef ENC_CNT_OFFSET
            KEY_SetControlCode(CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
#else
            KEY_SetControlCode(CURRENT_POS, INV1_PC.Pos);
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, INV1_PC.Pos);
#endif
            Init_PROFILE();
            INV1_PROFILE.POS_CMD = INV1_PC.Pos;
            INV1_PROFILE.PosRef = INV1_PC.Pos;
            INV1_PROFILE.PosRefFinal = INV1_PC.Pos;
            INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;
            INV1_PC.PosRefFinal = INV1_PROFILE.PosRefFinal;
        }
		break;
	case MANUAL_DRIVE:			// Manual Operation Mode
		RunMode_Check = MANUAL_DRIVE;

		if(Flags.Ready1 == 1 && Flags.Run1 == 1){
            Flags.Position_control = 0;
            Flags.ProfileMode = 0;
            INV1_PC.Spd_ref_max = INV1_PROFILE.InspectSpeed;

            if((g_SpiDspStatus.u8RunStatus == RUN_UP)||(g_SpiDspStatus.u8RunStatus == RUN_RIGHT))
            {
                if(KNOW_ANGLE == 1 && Flags.PosCon == 1){
                    step_temp+=(INV1_PC.Spd_ref_max-INV1_PC.Spd_ref_max*0.5)*Tdrv;
                    if(step_temp >= INV1_PC.Spd_ref_max) step_temp = INV1_PC.Spd_ref_max;
                }
                INV1_PROFILE.SpeedRef = step_temp;
            }
            else if((g_SpiDspStatus.u8RunStatus == RUN_DOWN)||(g_SpiDspStatus.u8RunStatus == RUN_LEFT))
            {
                if(KNOW_ANGLE == 1 && Flags.PosCon == 1){
                    step_temp+=(INV1_PC.Spd_ref_max-INV1_PC.Spd_ref_max*0.5)*Tdrv;
                    if(step_temp >= INV1_PC.Spd_ref_max) step_temp = INV1_PC.Spd_ref_max;
                }
                INV1_PROFILE.SpeedRef = -step_temp;
            }
            else
            {
                if(INV1_PROFILE.SpeedRef > 0)
                {

                    if(INV1_PROFILE.SpeedRef <= 0)
                    {
                        INV1_PROFILE.SpeedRef = 0;
                        step_temp = 0;
                        Init_PROFILE();
                    }
                    else
                    {
                        INV1_PROFILE.SpeedRef = INV1_PROFILE.SpeedRef - 0.001;
                    }
                }
                else  if(INV1_PROFILE.SpeedRef < 0)
                {
                    if(INV1_PROFILE.SpeedRef >= 0)
                    {
                        INV1_PROFILE.SpeedRef = 0;
                        step_temp = 0;
                        Init_PROFILE();
                    }
                    else
                    {
                        INV1_PROFILE.SpeedRef = INV1_PROFILE.SpeedRef + 0.001;
                    }
                }
                else
                {
                    INV1_PROFILE.SpeedRef = 0;
                    step_temp = 0;
                    Init_PROFILE();
                }

                INV1_PROFILE.POS_CMD = INV1_PC.Pos;
                INV1_PROFILE.PosRef = INV1_PC.Pos;
                INV1_PROFILE.PosRefFinal = INV1_PC.Pos;
                INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;
                INV1_PC.PosRefFinal = INV1_PROFILE.PosRefFinal;
            }

            if((fabs(INV1_SC.Wrm) <= ZspLevel)){        // Zero speed Level: 0.005m/s
                nINV_ZSP_ON;                            // Notify Zero Speed to MCU
                Flag_ZSP = 1;
            }
            else{
                nINV_ZSP_OFF;
                Flag_ZSP = 0;
            }
        }
        else{
            /* Align Sequence Code */
            nINV_ZSP_ON;                                // Notify Zero Speed to MCU
            Flag_ZSP = 1;
            /* Align Sequence Code End */
        }

        if((Flag_ZSP == 1)&&(BKO == TRUE)&&(KNOW_ANGLE == 1))
        {
            KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);    // Storing the end of Electrical Theta to FRAM
            FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);
            KEY_SetControlCode(ACCUM, ENCA_Data.Taccum);
            FRAM_Write(KEY_CONTROL_START+ACCUM, ENCA_Data.Taccum);
            KEY_SetControlCode(SEL_POS, ENCA_Data.selPosition);
            FRAM_Write(KEY_CONTROL_START+SEL_POS, ENCA_Data.selPosition);
#ifdef ENC_CNT_OFFSET
            KEY_SetControlCode(CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
#else
            KEY_SetControlCode(CURRENT_POS, INV1_PC.Pos);
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, INV1_PC.Pos);
#endif
            Init_PROFILE();
            INV1_PROFILE.POS_CMD = INV1_PC.Pos;
            INV1_PROFILE.PosRef = INV1_PC.Pos;
            INV1_PROFILE.PosRefFinal = INV1_PC.Pos;
            INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;
            INV1_PC.PosRefFinal = INV1_PROFILE.PosRefFinal;
        }


		break;
	case AUTO_DRIVE:			// Auto Operation Mode
	    RunMode_Check = AUTO_DRIVE;

        INIT_DRV = 0;

        Flags.Position_control = 1;
        Flags.ProfileMode = 1;
        Flags.CompProfile = 1;

        if(Flags.Ready1 == 1 && Flags.Run1 == 1){
            if((g_SpiDspStatus.u8RunStatus == RUN_UP)||(g_SpiDspStatus.u8RunStatus == RUN_RIGHT))
            {
                step_temp = ((float32)g_SpiDspStatus.SetPosition)*0.0001;
            }
            else if((g_SpiDspStatus.u8RunStatus == RUN_DOWN)||(g_SpiDspStatus.u8RunStatus == RUN_LEFT))
            {
                step_temp = (-1*(float32)g_SpiDspStatus.SetPosition)*0.0001;
            }

            if(step_temp != step_temp_old)
            {
                distance_temp = step_temp - step_temp_old;          // For Direction and Moving Distance
            }

            if(fabs(distance_temp) > 1.6){
                INV1_PC.Spd_ref_max     = KEY_GetFactoryCode(OS_SET);
                INV1_PROFILE.SpeedMax   = KEY_GetControlCode(MAX_SPEED);
                INV1_PROFILE.AccMax     = KEY_GetControlCode(MAX_ACC);
                INV1_PROFILE.JerkMax    = KEY_GetControlCode(MAX_JERK);
                INV1_PROFILE.POS_CMD    = step_temp;
                g_SpiArmStatus.u8LowSpdOp = 0x00;
            }
            else if((fabs(distance_temp) >= 0.1)&&(fabs(distance_temp) <= 1.6)){
                INV1_PROFILE.SpeedMax   = KEY_GetControlCode(CREEP_SPEED);
                INV1_PC.Spd_ref_max     = INV1_PROFILE.SpeedMax * 1.2;
                INV1_PROFILE.AccMax     = fabs(distance_temp) * 0.01;
                INV1_PROFILE.JerkMax    = INV1_PROFILE.AccMax;
                INV1_PROFILE.POS_CMD    = step_temp;
                g_SpiArmStatus.u8LowSpdOp = 0x01;
            }
            else if((fabs(distance_temp) < 0.1)){
                INV1_PROFILE.SpeedMax   = KEY_GetControlCode(CREEP_SPEED);
                INV1_PC.Spd_ref_max     = INV1_PROFILE.SpeedMax * 1.2;
                INV1_PROFILE.AccMax     = fabs(distance_temp) * 0.01;
                INV1_PROFILE.JerkMax    = INV1_PROFILE.AccMax;
                INV1_PROFILE.POS_CMD = step_temp;
                g_SpiArmStatus.u8LowSpdOp = 0x02;
            }
            step_temp_old = step_temp;

            INV1_PC.Remain_Distance = INV1_PROFILE.POS_CMD - INV1_PC.Pos;
            if((fabs(INV1_SC.Wrm) <= ZspLevel)&&(fabs(INV1_PC.Remain_Distance) <= 0.001) && g_SpiArmStatus.u8LMotAngleFind == 0x01){     // Zero speed Level: 0.005m/s
                nINV_ZSP_ON;                        // Notify Zero Speed to MCU
                Flag_ZSP = 1;
            }
            else{
                nINV_ZSP_OFF;
                Flag_ZSP = 0;
            }
        }
        else{
            step_temp = INV1_PC.Pos;
            step_temp_old = step_temp;
            /* Align Sequence Code */
            nINV_ZSP_ON;                        // Notify Zero Speed to MCU
            Flag_ZSP = 1;
            /* Align Sequence Code End */
        }

        if((Flag_ZSP == 1)&&(BKO == TRUE)&&(KNOW_ANGLE == 1))
        {
            KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);            // Storing the end of Electrical Theta to FRAM
            FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET, INV1.Thetar);
            KEY_SetControlCode(ACCUM, ENCA_Data.Taccum);
            FRAM_Write(KEY_CONTROL_START+ACCUM, ENCA_Data.Taccum);
            KEY_SetControlCode(SEL_POS, ENCA_Data.selPosition);
            FRAM_Write(KEY_CONTROL_START+SEL_POS, ENCA_Data.selPosition);
#ifdef ENC_CNT_OFFSET
            KEY_SetControlCode(CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, (Encoder_HD.Pos_cnt + Encoder_HD.PosCntOffset));
#else
            KEY_SetControlCode(CURRENT_POS, INV1_PC.Pos);
            FRAM_Write(KEY_CONTROL_START+CURRENT_POS, INV1_PC.Pos);
#endif

            Init_PROFILE();
            INV1_PROFILE.POS_CMD = INV1_PC.Pos;
            INV1_PROFILE.PosRef = INV1_PC.Pos;
            INV1_PROFILE.PosRefFinal = INV1_PC.Pos;
            INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;
            INV1_PC.PosRefFinal = INV1_PROFILE.PosRefFinal;
        }
		break;
	case OUTPUT_DEBUG:
	    RunMode_Check = OUTPUT_DEBUG;

	    if(g_Id45Data.u8nMC_A == 0x31) Flags.Vdc = 1;
	    else Flags.Vdc = 0;

	    if(g_Id45Data.u8nIFAN == 0x31) nINV_FAN_ON;
	    else nINV_FAN_OFF;

	    if(g_Id45Data.u8nDBKA == 0x31) nBKO_ON;
	    else nBKO_OFF;

	    break;
    case STATIC_MEASUREMENT:
        RunMode_Check = STATIC_MEASUREMENT;

        Static_Force_Measurement = 1;

        break;
	default:
//		INV1_PROFILE.POS_CMD = 0;
		Init_PROFILE();														// Speed & Postion Profile Vatiable Initialization
//		INV1_PC.PosRefFinal = 0.;
		INV1_PC.TimePC = 0;
		BKCloseStatusNum = 3;
        BK_CLOSE();
		break;
	}
	g_PreSpiDspStatus = g_SpiDspStatus;
	/* Operating Sequence by DrvStat_Drive END */


	/* USER CODE END  */

	if((((Flags.Fault == 1)||(FLT.uFlag == TRUE))&&(FLT.uFltNum != FLT_ENABLE))){	
		PWM1_BUFF_OFF();
		nFAULT_ON ;
		g_SpiArmStatus.u8InvErr = 1;
//		SCI_OUT2.regStatusBit.inver = 1 ;	// Need to modify 20181228
		nFAULT_ON ;
	}
	else{
	    g_SpiArmStatus.u8InvErr = 0;
//    	SCI_OUT2.regStatusBit.inver = 0 ;	// Need to modify 20181228
		nFAULT_OFF ;
	}
	I_phase_rms = __sqrt(INV1.Idse*INV1.Idse + INV1.Iqse*INV1.Iqse)*INV_SQRT2;
	
	// SW Reset from ARM
    if((g_BDIB.regStatusBit.RST == TRUE)){
        u8SwResetCnt++;
        if((u8SwResetCnt > 50)){
            u8SwResetCnt = 0;
            while(1){   // Watch-dog Reset
               FLT_Reset();
               EALLOW;
               WdRegs.WDKEY.bit.WDKEY = 0x0011;
               WdRegs.WDKEY.bit.WDKEY = 0x00AA;

               WdRegs.SCSR.bit.WDENINT = 0x0;
               WdRegs.WDCR.all = 0x10;
               EDIS;
           }
        }
    }
    else u8SwResetCnt = 0;

	if((DRVCnt%50) == 0){
		if(Flags.Fault == 0){
			if(g_SpiArmStatus.u8InvWatchdog == 1){
				INV_WD1_ON;
				INV_WD2_OFF;
			}
			else{
				INV_WD1_OFF;
				INV_WD2_ON;
			}
		}
		else{
			if(g_SpiArmStatus.u8InvWatchdog == 1){
				INV_WD1_ON;
				INV_WD2_ON;
			}
			else{
				INV_WD1_OFF;
				INV_WD2_OFF;
			}
		}
		INV_WD_TOG;
		g_SpiArmStatus.u8InvWatchdog ^= 1;
	}
	DRVCnt++;

	// Solved ZSP Problem on Profile Operation Mode. Need to modify ZSP Problem on Non-Profile Mode
    if(((Flags.Align1 == 0 && Flags.ProfileMode && ((INV1_PROFILE.TCH && Flags.CompProfileEnd == 1) || Flag_ZSP))||(Flags.Align1 == 0 && Flags.ProfileMode == 0 && Flag_ZSP))){
        BKCloseStatusNum = 4;
        if(KNOW_ANGLE == 1){
            nINV_ZSP_ON;                        // Notify Zero Speed to MCU
            Flag_ZSP = 1;
        }

        BK_CLOSE();
    }

    // Missing the Encoder theta Fault Condition
    if(fabs(INV1_CC.Vdse_Ref_Integ) > Thetar_Rst_Vdse) {
        u8OvAngleCnt++;
        if(u8OvAngleCnt >= 100){
//            FLT_Raise(FLT_OV_ANGLE);
            u8OvAngleCnt = 0;
        }
    }

//	// Brake GPIO Test
//#if 0
//	if(Flag_BK_Test)        // BK
//	{
////		nBKO_ON ;
////	    g_SpiArmStatus.u8BK_status1 = g_BDIB.regStatusBit.XBKAS;
////	    g_SpiArmStatus.u8BK_status2 = g_BDIB.regStatusBit.XBKBS;
//
//	    BK_RELEASE();
//	}
//	else
//	{
////		nBKO_OFF ;
////	    g_SpiArmStatus.u8BK_status1 = g_BDIB.regStatusBit.XBKAS;
////	    g_SpiArmStatus.u8BK_status2 = g_BDIB.regStatusBit.XBKBS;
//
//	    BK_CLOSE();
//	}
//#endif

//	if(DRVCnt % 100 == 0) 	SpibRegs.SPIFFTX.bit.TXFFIENA = 1;

	if(DRVCnt >= 400)	DRVCnt = 0 ;

#if HHT_ENABLE
	KEY_Scan();
#endif

	iCnt++; 
	if(iCnt > 4){
		iCnt = 0 ;
		u20msFlag = 1 ;
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
	asm(" NOP");
}	
	
/*******************************************************************/
/* 	DRV_Create - DRV 占쎈씈猷뉔뜮?占쎈쇊?占쎌룞?占썼껀占쎄퍏逾녔�ⓦ끉?占쏙옙醫묒삕   */
/*******************************************************************/
void DRV_Create( void )
{   
	int i;
	DRV.flgReset 			= ZERO;
	DRV.regMainStatus 		= ZERO;
	DRV.regPrevMainStatus 	= ZERO;
	DRV_Initialize();
	ZspLevel 				= KEY_GetInterfaceCode(ZSP_LEVEL);
//	SpdCtlErrLevel 			= KEY_GetInterfaceCode(SPDCTL_ERR_LVL);
//  Drv_Bko_CNT_Lmt 		= (unsigned int)(KEY_GetControlCode(BRK_OFF_DELAY)*200.);
	Thetar_Rst_Vdse 		= 200;      // Test and verified 20190227 by GWON
//	Floor.uFloorTop = (int)(KEY_GetControlCode(MAX_FLOOR) + 0.5) ;		// Using Variable at drive.c, FC.c, Rmb.c, VANE.c

	for (i=0;i<MAXNONSTOPNUM;i++) nonStop_Flr[i] = 0;
}

void DRV_Initialize(void)
{
	ANGLE_cnt = 0;
	TqBias_Step = 0.;
//	Drv_Bko_CNT = 0;
	FlagSINCOSErr_C = 0 ; 
	FlagSINCOSErr_D = 0 ;
}

/*******************************************************************/
/* DRV_CheckCmdOff - Cmd占쎌쥙猷욑옙?占쎌㉨縕ワℓ濡녹삕?占쎌뜴?占쏙옙?占쎌뵰占쎈벙??占쏙옙占�?占쏙옙占�?占쏙옙??		           */
/*******************************************************************/
Uint16 DRV_CheckCmdOff( void )
{
	if((g_BDIB.regStatusBit.UP == CLOSED)||(g_BDIB.regStatusBit.DOWN == CLOSED)||(g_BDIB.regStatusBit.RST == CLOSED)){
		return (FALSE) ;
	}
	else{ 
		return (TRUE) ;
	} 
}

/*******************************************************************/
/* DRV_GetStatus - Drive Staus 占쎌쥙?占썼짆袁ъ삕?占쏙옙                              */
/*******************************************************************/
Uint16	DRV_GetStatus( void )
{
	if ((DRV.flgReset == CLOSED)&&(Flags.Fault == 1)) return DRV_RESET;
	return ( DRV.regMainStatus & DRV_USEBIT );
}

/*******************************************************************/
/* DRV_UpdateStatus - Drive Staus Update                           */
/*******************************************************************/
void DRV_UpdateStatus( void )
{
	if (g_BDIB.regStatusBit.RST == 1){
		DRV.flgReset = CLOSED ;
		READYCnt = 0 ;
		FluxON_Cnt = 0 ;
		DCBON_Cnt = 0 ;
	}
	else{
		DRV.flgReset = OPEN; 
	}
	if ((Flags.Fault == 0)&&(Flags.MC_Relay == 1)&&(Flag_offset==2)/*&&((fabs(INV1.ThetaOffset)>=0.000001)||(KNOW_ANGLE==0))*/){
		DRV.regMainStatusBit.flgReady = ON ; 
	}
	else{
		DriveStatPsudo = 0;
		PWM1_BUFF_OFF();
		DRV.regMainStatusBit.flgReady = OFF;
		DRV.regMainStatus = ZERO;
		READYCnt = 0 ;
		FluxON_Cnt = 0 ;
		DCBON_Cnt = 0 ; 
		goto UpdateStatusExit;
	}

	switch (DRV.regPrevMainStatus){
		case DRV_READY:       
			DriveStatPsudo = 1;
			READYCnt++ ;
			DRV_Initialize();
//			BKCloseStatusNum = 5;
//			BK_CLOSE();
			nINV_RUN_OFF ;          // RUN disableb
			u8INV_RUN_RDY = 0 ;
			Bkop_Cnt = 0;
			u8SpdFaultCnt = 0;
			u8OvAngleCnt = 0;

			INV1_PROFILE.PosRef_d1 = INV1_PC.Pos;
			INV1_PROFILE.POS_CMD_old = INV1_PC.Pos;
			step_temp = INV1_PC.Pos;
			step_temp_old = step_temp;

			if(KNOW_ANGLE){
			    INV1.ThetaOffset = KEY_GetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET);
//			    INV1_PC.Pos_Offset = KEY_GetControlCode(CURRENT_POS);
//			    Encoder_HD.PosCntOffset = KEY_GetControlCode(CURRENT_POS);
			}

			Flag_thetar_False = 0 ;
			if ( READYCnt > ReadyHoldTime)	READYCnt = ReadyHoldTime+1 ;
			if ( READYCnt > ReadyHoldTime){
				if ((RunCnt == 0) || (Reset_Thetar_Flg == 1) || (FLT.uFlag == 1)){	
					Flag_thetar_reset = 0 ;
					Flag_reset = 0 ;
					Flag_Sin = 0 ;
					Reset_Thetar_Flg = 0 ;
			  	}
				ANGLE_cnt = 0;
				ENCnt = 0;
				ENC_ABCnt = 0 ;
				flgRefOff = DRV_CheckCmdOff();
				if ( flgRefOff == TRUE ) goto UpdateStatusExit;
//				SinCosDir();
//				uEL_Start = 1;
//				FC_StopFlr_Cal();
				/* Upadate to DRV_FLUXON */
				DRV.regMainStatusBit.flgCmd 	= ON ;
				DRV.regMainStatusBit.flgConv 	= OFF ;	// FOR FREEWAY 20180225
				DRV.regMainStatusBit.flgCC 		= ON ;	// FOR FREEWAY 20180225
				DRV.regMainStatusBit.flgSC 		= OFF ;
//				DLA_ErrCnt2 = 0;
//				DLA_ErrCnt1 = 0;
				READYCnt = 0 ;
			}
			Flag_mode = 0 ;//SUDS
			break;
		case DRV_CONVRUN :		// NEVER INTO THIS SEQUENCE AT FREEWAY PROJECT 20180205
			DriveStatPsudo = 2;
			cntConv ++ ;
			flgRefOff = DRV_CheckCmdOff();
			if(flgRefOff){
				DRV.regMainStatusBit.flgCmd 	= OFF ;
				DRV.regMainStatusBit.flgConv 	= OFF ;
				DRV.regMainStatusBit.flgCC 		= OFF ;
				DRV.regMainStatusBit.flgSC 		= OFF ;
				cntConv = 0 ;
			}
			else if(((Flags.Vdc_Control_Ok == 1)&&(cntConv >= 20 ))||((/*(Floor.uMode == FC_ELD)||*/(ELD_Speed_Mode == PLO_MODE))&&(Flags.MC_Relay == 1))){
				if(Converter_TestFlg == 1) goto UpdateStatusExit; //for Test
				DRV.regMainStatusBit.flgCC = ON ;
				RunCnt++ ;
				cntConv=0 ;
				if(Flags.Fault_Warn == 1){
					Flags.Fault_Warn_RunCnt++;
				}
			}
			break ;
		case DRV_INVINIT:   	
			DriveStatPsudo = 3;

			flgRefOff = DRV_CheckCmdOff();
			if(flgRefOff){
				DRV.regMainStatusBit.flgCmd 	= OFF ;
				DRV.regMainStatusBit.flgConv 	= OFF ;
				DRV.regMainStatusBit.flgCC 		= OFF ;
				DRV.regMainStatusBit.flgSC 		= OFF ;
				FluxON_Cnt = 0;				
			}
			FluxON_Cnt++;
			if(FluxON_Cnt > FluxHoldTime)	FluxON_Cnt = FluxHoldTime+1 ;
			if((FluxON_Cnt > FluxHoldTime)&&(Flags.Ready1==1)){
				DRV.regMainStatusBit.flgSC = ON ;
				FluxON_Cnt = 0;				
				RST_Offset++;
			}
			break;
		case DRV_RUN: 		
			DriveStatPsudo = 4 ;
			nINV_RUN_ON ;               // RUN disable

			if((Flags.Align1 == 0) && (Flags.Speed_control_1==1)){
			    u8INV_RUN_RDY++;
                if(u8INV_RUN_RDY >= u8BkReleaseDelay){    // 500msec delay
                    BKCloseStatusNum = 6;
                    u8INV_RUN_RDY = u8BkReleaseDelay + 1;
                    BK_RELEASE();
                }
                else BK_CLOSE();
			}

			flgRefOff = DRV_CheckCmdOff();
			if ( flgRefOff == TRUE )	
				DRV.regMainStatusBit.flgCmd = OFF ;
			break;
			
		case DRV_DECEL:			
			DriveStatPsudo = 5;
			flgRefOff = DRV_CheckCmdOff();
			if(flgRefOff == TRUE){	
				if((RMB_SpdPtnSel == NORMAL_V2TH)||(RMB_SpdPtnSel == DOUBLE_SPD_PTN)||(RMB_SpdPtnSel == NORMAL_V3TH)||(RMB_SpdPtnSel == OPTIMAL_SPD_PTN)){                     
					DRV.regMainStatusBit.flgSC = OFF;
				}
				else{
					DRV.regMainStatusBit.flgConv 	= OFF;		//go to Ready
					DRV.regMainStatusBit.flgCC 		= OFF;
					DRV.regMainStatusBit.flgSC 		= OFF;
				}
			}else DRV.regMainStatusBit.flgCmd = ON;
			break;
		
		case DRV_DCBON:             
			DriveStatPsudo = 6;
			flgRefOff = DRV_CheckCmdOff();
			DCBON_Cnt++;
			if(INIT_DRV){	// Floor Height Measurement Operating Mode
//				KEY_SetControlCode(INIT_DRIVE, 0);
//				INIT_DRV = (Uns)(KEY_GetControlCode(INIT_DRIVE)+0.5);
//				nINIC_OFF;
			}		
			if(DCBON_Cnt > DCBHoldTime)	DCBON_Cnt = DCBHoldTime+1 ;
			if(DCBON_Cnt > DCBHoldTime){    
				DRV.regMainStatusBit.flgConv 	= OFF;
				DRV.regMainStatusBit.flgCC 		= OFF;
				DRV.regMainStatusBit.flgSC 		= OFF;
				DCBON_Cnt = 0;
			}
			else{
				goto UpdateStatusExit;
			}
			break;
			
		case DRV_FAULT:  
			DriveStatPsudo = 7;           
			PWM1_BUFF_OFF();
			if(INIT_DRV){	
//				KEY_SetControlCode(INIT_DRIVE, 0);
//				INIT_DRV = (Uns)(KEY_GetControlCode(INIT_DRIVE)+0.5);
				nINIC_OFF;			
			}	
			if((Flags.Fault == 0)&&(Flag_offset==2)/*&&((Flags.GRID_Volt_SeqChk==1)||((Floor.uMode == FC_ELD)||(ELD_Speed_Mode == PLO_MODE)))//FOR FREEWAY 20180225*/){
				DRV.regMainStatusBit.flgReady 	= ON ;
				DRV.regMainStatusBit.flgCmd 	= OFF;
				DRV.regMainStatusBit.flgCC 		= OFF;
				DRV.regMainStatusBit.flgSC 		= OFF;
				DRV.regMainStatusBit.flgConv 	= OFF; 
			}
			else{
				DRV.regMainStatusBit.flgReady 	= OFF;
				DRV.regMainStatusBit.flgCmd 	= OFF;
				DRV.regMainStatusBit.flgCC 		= OFF;
				DRV.regMainStatusBit.flgSC 		= OFF;
				DRV.regMainStatusBit.flgConv 	= OFF;
				DRV.regMainStatus = ZERO; 
				READYCnt = 0 ;
				FluxON_Cnt = 0 ;
				DCBON_Cnt = 0 ;
			}
			if(((RunCnt==0)||(Reset_Thetar_Flg==1)||(FLT.uFlag == 1))&&(fabs(INV1_SC.Wrpm) <=0.1)/*&&((Floor.uFloorTop > 2)&&(Floor.uFloorReal<(Floor.uFloorTop-1)))*/){
				Flag_thetar_reset = 0 ;
				Flag_reset = 0 ;
				Flag_Sin = 0 ;
				Reset_Thetar_Flg = 0 ;
		  	}
  			ANGLE_cnt = 0;
      	break;
		default:	
			//; // V/F Test	
			DriveStatPsudo = 8;
			PWM1_BUFF_OFF();
			if(INIT_DRV){	
//				KEY_SetControlCode(INIT_DRIVE, 0);
//				INIT_DRV = (Uns)(KEY_GetControlCode(INIT_DRIVE)+0.5);
				nINIC_OFF;			
			}	
			if((Flags.Fault == 0)&&(Flag_offset==2)/*&&((Flags.GRID_Volt_SeqChk==1)||((Floor.uMode == FC_ELD)||(ELD_Speed_Mode == PLO_MODE)))//FOR FREEWAY 20180225*/){
				DRV.regMainStatusBit.flgReady 	= ON ;
				DRV.regMainStatusBit.flgCmd 	= OFF;
				DRV.regMainStatusBit.flgCC 		= OFF;
				DRV.regMainStatusBit.flgSC 		= OFF;
				DRV.regMainStatusBit.flgConv 	= OFF; 
			}
			else{
				DRV.regMainStatusBit.flgReady 	= OFF;
				DRV.regMainStatusBit.flgCmd 	= OFF;
				DRV.regMainStatusBit.flgCC 		= OFF;
				DRV.regMainStatusBit.flgSC 		= OFF;
				DRV.regMainStatusBit.flgConv 	= OFF;
				DRV.regMainStatus = ZERO; 
				READYCnt = 0 ;
				FluxON_Cnt = 0 ;
				DCBON_Cnt = 0 ;
			}
			DriveStatPsudo = 10;
			if((RunCnt==0)||(Reset_Thetar_Flg == 1)||(FLT.uFlag == 1)){	
				Flag_thetar_reset = 0 ;
				Flag_reset = 0 ;
				Flag_Sin = 0 ;
		 		Reset_Thetar_Flg = 0 ;
		  	}
			ANGLE_cnt = 0;
			break;
	}

UpdateStatusExit:
	DRV.regPrevMainStatus = DRV.regMainStatus;
	return;

}

void SinCosDir(void)
{
	if(Enc_Dir){
		if(FWD_Dir == 0){	
			if(BDIB_UP == 1)			XintRegs.XINT4CR.bit.POLARITY = 0;      // Falling edge interrupt
			else if(BDIB_DOWN == 1)		XintRegs.XINT4CR.bit.POLARITY = 1;      // Rising edge interrupt
		}
		else{
			if(BDIB_UP == 1)			XintRegs.XINT4CR.bit.POLARITY = 1;      // Rising edge interrupt
			else if(BDIB_DOWN == 1)		XintRegs.XINT4CR.bit.POLARITY = 0;      // Falling edge interrupt
		}
	}
	else{
		if(FWD_Dir == 0){	
			if(BDIB_UP == 1)			XintRegs.XINT4CR.bit.POLARITY = 1;      // Rising edge interrupt
			else if(BDIB_DOWN == 1)		XintRegs.XINT4CR.bit.POLARITY = 0;      // Falling edge interrupt
		}
		else{
			if(BDIB_UP == 1)			XintRegs.XINT4CR.bit.POLARITY = 0;      // Falling edge interrupt
			else if(BDIB_DOWN == 1)		XintRegs.XINT4CR.bit.POLARITY = 1;      // Rising edge interrupt
		}									
	}	
}

/*******************************************************************/
/* DRV_GetSCStatus - SC Staus 占쎌쥙?占썼짆袁ъ삕?占쏙옙                               */
/*******************************************************************/
Uint16 DRV_GetSCStatus( void )
{
	return DRV.regMainStatusBit.flgSC;
}

Uint16 DRV_GetCCStatus( void )
{
	return DRV.regMainStatusBit.flgCC;
}

void BK_CLOSE(void)
{
    TestBKClose++;
	if(Drv_Bko_CNT_Lmt > 1){
		Drv_Bko_CNT++;
		if(Drv_Bko_CNT > Drv_Bko_CNT_Lmt){
			nBKO_OFF;
			Bko = 0 ;
			Drv_Bko_CNT = Drv_Bko_CNT_Lmt + 1;
		}
	}
	else{
		nBKO_OFF;
		Bko = 0 ;
	}

	g_SpiArmStatus.u8BK_status1 = g_BDIB.regStatusBit.XBKAS;
	g_SpiArmStatus.u8BK_status2 = g_BDIB.regStatusBit.XBKBS;
//
//	if((g_BDIB.regStatusBit.XBKAS == 1)||(g_BDIB.regStatusBit.XBKBS == 1)){
//	    Bkcloseop_Cnt++;
//        if(Bkcloseop_Cnt > 600){
//            Bkcloseop_Cnt = 601;
//            FLT_Raise(FLT_BKOP);
//        }
//    }
//    else Bkcloseop_Cnt = 0;
}

void BK_RELEASE(void)
{
    if(Flag_ZSP == 0 || Flags.Align1 == 1){
        Drv_Bko_CNT = 0;
    }

	nBKO_ON ;
	Bko = 1 ;
//	DELAY_US(1);

	g_SpiArmStatus.u8BK_status1 = g_BDIB.regStatusBit.XBKAS;
	g_SpiArmStatus.u8BK_status2 = g_BDIB.regStatusBit.XBKBS;

	// Brake Fault Condition is changed from Bko to BKO.
	if((BKO == TRUE)&&((g_BDIB.regStatusBit.XBKAS == 0)||(g_BDIB.regStatusBit.XBKBS == 0))){
		Bkop_Cnt++;
		if(Bkop_Cnt > 2500){ // 5 sec
			Bkop_Cnt = 2501;
			FLT_Raise(FLT_BKOP);
		}
	}
	else Bkop_Cnt = 0;
}
