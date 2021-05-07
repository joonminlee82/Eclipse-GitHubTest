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
#include "DAC.h"
#include "Variable.h"
#include "CC.h"
#include "SC.h"
#include "SCI.h"
#include "SPI.h"
#include "drive.h"
#include "gnl.h"
#include "CPU1_CLA_Shared.h"

extern Uint16 Vdc_Con_st, Spd_Con_st, cc_cnt, Vcd_con_cnt;
extern int scic_Cnt,scic_Cnt_old;
extern Uns MCP_COM_CNT;

extern STATUS_ARM_OBJ   g_SpiArmStatus;
extern Id46Data         g_Id46Data;
extern ENC_DATA_OBJ     ENC1_Data;
extern ENC_DATA_OBJ     ENCB_Data;
extern ENC_DATA_OBJ     ENCA_Data;
extern CC_3PH           INV1_CC;
extern Motor            INV1;
extern SC               INV1_SC;
extern PC               INV1_PC;
extern PROFILE          INV1_PROFILE;

long *da[8] = {0,0,0,0,0,0,0,0};
#pragma DATA_SECTION(da_cla, "CLADataLS1");
#pragma DATA_SECTION(da_value, "CLADataLS1");
#pragma DATA_SECTION(da_scale, "CLADataLS1");
#pragma DATA_SECTION(da_mid, "CLADataLS1");
float da_cla[8] = {0.,0.,0.,0.,0.,0.,0.,0.};
long  da_value[8] = {0,0,0,0,0,0,0,0};
int   da_type[8] = {0,0,0,0,0,0,0,0};
float da_scale[8] = {0.,0.,0.,0.,0.,0.,0.,0.};
float da_mid[8] = {0.,0.,0.,0.,0.,0.,0.,0.};

Uint32 Spi_TxEmpty_cnt = 0;

u8 u8DacSelection = 0;

u8 u8Upper;
u8 u8Lower;
long *address;

void InitDa(u8 u8mode)
{
    int i;

	//DAC1
	DA_CH1 = 0x800;       // range of DA_CH1 : 0000 0000 0000 - 1111 1111 1111
	DA_CH2 = 0x800;
	DA_CH3 = 0x800;
	DA_CH4 = 0x800;

	nDAC0_LD_ON;

	DELAY_US(1L);
	
	nDAC0_LD_OFF;

	switch(u8mode){
	case 0:
	    for(i=0; i < 4; i++){
	        da[i]       = (long *)(getadd(g_Id46Data.NumVari[i]));
	        da_type[i]  = g_Id46Data.type[i];
	        da_scale[i] = 2048./g_Id46Data.Scale[i];
	        da_mid[i]   = g_Id46Data.Mid[i];
	    }
	    break;
	case 1:
        for(i=0; i < 4; i++){
            da[i]       = (long *)g_Id47Data.NumVariAdd[i];
            da_type[i]  = g_Id47Data.type[i];
            da_scale[i] = 2048./g_Id46Data.Scale[i];
            da_mid[i]   = g_Id47Data.Mid[i];
        }
	    break;
	default:
        for(i=0; i < 4; i++){
            da[i]       = (long *)(getadd(g_Id46Data.NumVari[i]));
            da_type[i]  = g_Id46Data.type[i];
            da_scale[i] = 2048./g_Id46Data.Scale[i];
            da_mid[i]   = g_Id46Data.Mid[i];
        }
	    break;
	}

#if 0
	da[0] = (long *)(&Vdc_rd);
	da_type[0] = 0;
	da_scale[0] = 2048./800;
	da_mid[0] = 0;

	da[1] = (long *)(&INV1.Idse);
	da_type[1] = 0;
	da_scale[1] = 2048./8.;
	da_mid[1] = 0;

	da[2] = (long *)(&INV1_CC.Iqse_Ref);
	da_type[2] = 0;
	da_scale[2] = 2048./160.;
	da_mid[2] = 0;

	da[3] = (long *)(&INV1.Iqse);
	da_type[3] = 0;
	da_scale[3] = 2048./160.;
	da_mid[3] = 0;
#endif
#if 0
	da[4] = (long *)(&Scic_Buf_From_Main[4]);
	da_type[4] = 0;
	da_scale[4] = 2048./100.;
	da_mid[4] = 0;
	
	da[5] = (long *)(&INV1_CC.Idse_Ref);
	da_type[5] = 0;
	da_scale[5] = 2048./100.;
	da_mid[5] = 0;

	da[6] = (long *)(&INV1.Iqse);
	da_type[6] = 0;
	da_scale[6] = 2048./100.;
	da_mid[6] = 0;

	da[7] = (long *)(&INV1_CC.Iqse_Ref);
	da_type[7] = 0;
	da_scale[7] = 2048./100.;
	da_mid[7] = 0;
#endif
}

#pragma CODE_SECTION(daOut, "ramfuncs");
void daOut(void)
{
	Uint32 i;
	float a;
	
	nCS_DAC0_ON;//EM2A11 should be 'high' for DA
	nDAC0_LD_ON;
	for(i=0;i<8;i++){
		if(da_type[i] == 0){
			a = *(float *)da[i];
		}
		else{ 
			a = (float)((((*da[i])<<16)>>16));
		}
		da_cla[i] = a; 
		da_value[i] = (((long)((a - da_mid[i])*da_scale[i]) + 0x800) & 0x0FFF);      // digital value equivalent to a 
	}
	DA_CH1 = da_value[0];//It is more 1 sample update delay than SPI 
	DA_CH2 = da_value[1];
	DA_CH3 = da_value[2];
	DA_CH4 = da_value[3];
	asm("  RPT #3 || NOP");
	nCS_DAC0_OFF;
	nDAC0_LD_OFF;//ex time is about 1.5[us]
#if 0
		
//DA5------------------------------------------------------------------------------------------------
	da_value[4] = ~(0x0000 | da_value[4]);
	SpicRegs.SPITXBUF = da_value[4];
//DA6------------------------------------------------------------------------------------------------
	da_value[5] = ~(0x4000 | da_value[5]);
	SpicRegs.SPITXBUF = da_value[5];
//DA7------------------------------------------------------------------------------------------------
	da_value[6] = ~(0x8000 | da_value[6]);
	SpicRegs.SPITXBUF = da_value[6];
//DA8------------------------------------------------------------------------------------------------
	da_value[7] = ~(0xc000 | da_value[7]);
	SpicRegs.SPITXBUF = da_value[7];
#endif
}

//#pragma CODE_SECTION(cpu1_Spic_Tx, "ramfuncs");
//__interrupt void cpu1_Spic_Tx(void)
//{
//	IER &= 0x0000;
// 	IER |= M_INT2|M_INT3|M_INT5|M_INT6|M_INT8|M_INT10|M_INT12;
//	EINT;
//
//	Spi_TxEmpty_cnt++;
//
//	SpicRegs.SPITXBUF = da_value[4];
//	SpicRegs.SPITXBUF = da_value[5];
//	SpicRegs.SPITXBUF = da_value[6];
//	SpicRegs.SPITXBUF = da_value[7];
//
//	SpicRegs.SPIFFTX.bit.TXFFINTCLR = 1;
//	// Acknowledge this interrupt to recieve more interrupts from group 6
//	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
//	asm(" NOP");
//}


/************************************************/
/*         Get Data Address Function            */
/* Author: Gogume                               */
/* History: First Released 20190722             */
/* subject: Get Variable Address                */
/************************************************/
long *getadd(int vari)
{

    u8Upper = ((vari >> 4) & 0x0F) | 0x30;
    u8Lower = ((vari >> 0) & 0x0F) | 0x30;

    switch(u8Upper){
    case DA_CC:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case Q_AXIS_I:                  // Q axis Current
            address = (long*)&INV1.Iqse;
            break;
        case Q_AXIS_I_REF:              // Q axis Current Reference
            address = (long*)&INV1_CC.Iqse_Ref;
            break;
        case Q_AXIS_I_ERR:              // Q axis Current Control Error
            address = (long*)&INV1_CC.Err_Iqse;
            break;
        case D_AXIS_I:                  // D axis Current
            address = (long*)&INV1.Idse;
            break;
        case D_AXIS_I_REF:              // D axis Current Reference
            address = (long*)&INV1_CC.Idse_Ref;
            break;
        case D_AXIS_I_ERR:              // D axis Current Control Error
            address = (long*)&INV1_CC.Err_Idse;
            break;
        default:
            address = (long*)&INV1.Iqse;
            break;
        }
        break;
    case DA_SC:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case SPD_SC:                    // Speed on Speed Controller
            address = (long*)&INV1_SC.Wrm;
            break;
        case SPD_REF:                   // Speed Reference
            address = (long*)&INV1_SC.Wrm_ref;
            break;
        case SPD_ERR:                   // Speed Control Error
            address = (long*)&INV1_SC.Err_Wrm;
            break;
        default:
            address = (long*)&INV1_SC.Wrm;
        break;
        }
        break;
    case DA_PC:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case POS_PC:                    // Position on Position Controller
            address = (long*)&INV1_PC.Pos;
            break;
        case POS_REF:                   // Position Reference
            address = (long*)&INV1_PC.PosRefFinal;
            break;
        case POS_ERR:                   // Position Control Error
            address = (long*)&INV1_PC.Err_Pos;
            break;
        default:
            address = (long*)&INV1_PC.Pos;
            break;
        }
        break;
    case DA_PROFILE:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case POS_PROFILE:               // Position Reference on PROFILE
            address = (long*)&INV1_PROFILE.PosRefFinal;
            break;
        case SPD_PROFILE:               // Speed Reference on PROFILE
            address = (long*)&INV1_PROFILE.SpeedRef;
            break;
        case ACC_PROFILE:               // Acceleration Reference on PROFILE
            address = (long*)&INV1_PROFILE.AccRef;
            break;
        case JERK_PROFILE:              // Jerk Reference on PROFILE
            address = (long*)&INV1_PROFILE.Jerk;
            break;
        case TCH_PROFILE:               // PROFILE End Status
            address = (long*)&INV1_PROFILE.TCH;
            break;
        default:
            address = (long*)&INV1_PROFILE.PosRefFinal;
            break;
        }
        break;
    case DA_ROTOR_EST:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case VDSE_INTEG:                // D axis Voltage Integral Term
            address = (long*)&INV1_CC.Vdse_Ref_Integ;
            break;
        default:

            break;
        }
        break;
    case DA_FLAGS:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case FLAGS_BKO:                 // Flag Bko
            address = (long*)&Bko;
            break;
        case FLAGS_ZSP:                 // Flag ZSP
            address = (long*)&Flag_ZSP;
            break;
        default:
            address = (long*)&Bko;
            break;
        }
        break;
    case DA_APS:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case POS_APS:                   // Position on APS
            address = (long*)&INV1_PC.APSPos;
            break;
        case SPD1_APS:                  // Speed1 on APS
            address = (long*)&INV1_SC.APSWrm1;
            break;
        case SPD2_APS:                  // Speed2 on APS
            address = (long*)&INV1_SC.APSWrm2;
            break;
        case THETA_APS:                 // Theta on APS
            address = (long*)&INV1_PC.APSTheta;
            break;
        default:
            address = (long*)&INV1_PC.APSPos;
            break;
        }
        break;
    case DA_CRASH:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case DIST_UP:                   // UP Distance on Crash Sensor(Laser Sensor)
            address = (long*)&INV1_PC.LajerPosUp;
            break;
        case DIST_DOWN:                 // DOWN Distance on Crash Sensor(Laser Sensor)
            address = (long*)&INV1_PC.LajerPosDown;
            break;
        default:
            address = (long*)&INV1_PC.LajerPosUp;
            break;
        }
        break;
    case DA_ENC:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case POSA_HEIDENHAIN:           // Position A on Heidenhain Encoder
            address = (long*)&ENCA_Data.positionA;
            break;
        case POSB_HEIDENHAIN:           // Position B on Heidenhain Encoder
            address = (long*)&ENCA_Data.positionB;
            break;
        case POS_F_A_HEIDENHAIN:        // Filtered Position A on Heidenhain Encoder
            address = (long*)&ENCA_Data.positionfA;
            break;
        case POS_F_B_HEIDENHAIN:        // Filtered Position B on Heidenhain Encoder
            address = (long*)&ENCA_Data.positionfB;
            break;
        case DIFFPOSAB_HEIDENHAIN:      // Difference of between Postion A and B on Heidenhain Encoder
            address = (long*)&ENCA_Data.diff;
            break;
        case SPDA_HEIDENHAIN:           // Speed A on Heidenhain Encoder
            address = (long*)&ENCA_Data.speed_mech2;
            break;
        case SPDB_HEIDENHAIN:           // Speed B on Heidenhain Encoder
            address = (long*)&ENCA_Data.spdB_f;
            break;
        case REALPOSA_HEIDENHAIN:       // Real Position A on Heidenhain Encoder
            address = (long*)&ENCA_Data.realpositionA;
            break;
        case REALPOSB_HEIDENHAIN:       // Real Position B on Heidenhain Encoder
            address = (long*)&ENCA_Data.realpositionB;
            break;
        case THETAA_HEIDENHAIN:         // Theat A on Heidenhain Encoder
            address = (long*)&ENCA_Data.thetaA;
            break;
        case THETAB_HEIDENHAIN:         // Theta B on Heidenhain Encoder
            address = (long*)&ENCA_Data.thetaB;
            break;
        default:
            address = (long*)&ENCA_Data.positionA;
            break;
        }
        break;
    case DA_PWM:
        Flag_DA_Test = 0;
        switch(u8Lower){
        case VA_REF:                    // Phase-A Voltage Reference on PWM
           address = (long*)&INV1_CC.Vas_Ref;
           break;
       case VB_REF:                    // Phase-B Voltage Reference on PWM
           address = (long*)&INV1_CC.Vbs_Ref;
           break;
       case VC_REF:                    // Phase-C Voltage Reference on PWM
           address = (long*)&INV1_CC.Vcs_Ref;
           break;
       case THETA_INV:                 // Inverter Theta
           address = (long*)&INV1.Thetar;
           break;
       default:
           address = (long*)&INV1_CC.Vas_Ref;
           break;
        }
        break;
    case DA_DAC:
        Flag_DA_Test = 1;
        switch(u8Lower){
        case GRIDA_REF:                 // GRID-A DA Test Signal
            address = (long*)&GRID_Vas;
            break;
        case GRIDB_REF:                 // GRID-B DA Test Signal
            address = (long*)&GRID_Vbs;
            break;
        case GRIDC_REF:                 // GRID-C DA Test Signal
            address = (long*)&GRID_Vcs;
            break;
        case GRID_THETA:                // GRID-Theta DA Test Signal
            address = (long*)&Theta_grid;
            break;
        default:
            address = (long*)&GRID_Vas;
        }
        break;
    default:

        break;
    }

    return address;
}
