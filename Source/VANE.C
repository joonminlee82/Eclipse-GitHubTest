/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler TI_v6.4.2
	Author				: Inverter part, E/C Department in Hyudai elevator
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#include <math.h>
#include "vane.h"
#include "F28x_Project.h" 
#include "Variable.h"     
#include "CC.h"              
#include "SC.h"                
#include "Fault.h"             
#include "drive.h"             
#include "device.h"                   
//#include "rmb.h"
#include "bdib.h"
//#include "fc.h"
#include "SCI.h"
#include "gnl.h"

//extern FLOOR Floor ;
//extern RMB_PTN Pattern    ;
//extern Rmb RMB;
extern STATUS_ARM_OBJ  g_SpiArmStatus;

int Van_mode = 0;
float fPositionDiff = 0.;
float fVc_MIN = 5. ;
int CNTvane = 1;

__interrupt void cpu1_vane(void)
{
	IER &= 0x0000;
 	IER |= M_INT2 | M_INT3 | M_INT5 ;			// Fault || CC || SC 
	EINT;
	BDIB_UpdateAllStatus();
//	BDIB_HOU = (int)DIB_XULA;
//	BDIB_HOD = (int)DIB_XDLA;
	CNTvane ++ ;	             
	vane_c();			
	
	if((BDIB_HOU == 1)&&(BDIB_HOD == 0)) Van_mode = 1;
	else if((BDIB_HOU == 0)&&(BDIB_HOD == 1)) Van_mode = 2;
	else if((BDIB_HOU == 1)&&(BDIB_HOD == 1)) Van_mode = 3;
	else Van_mode = 0;
		
	//DINT ;
	//GpioDataRegs.GPBCLEAR.bit.GPIOB3=1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;	     
	asm(" NOP");
}

void vane_c(void)
{
//	float a, temp;
//	Uns	b;
//	if ((Floor.uMode == FC_NORMAL)||(Floor.uMode==FC_60MPM))
//	{
//		Floor.udoorzone = 0;
//		if ((BDIB_HOD == 0)&&(BDIB_HOU == 1)&&(BDIB_UP == 1)&&(Floor.uFloorReal == Floor.uFloorStop))
//		{
//			temp = (6*RMB.fAccel*VANE_LENGTH - 3*RMB.fVref*RMB.fVref) ;
//			if (RMB.fVref <= __sqrt(3*RMB.fAccel*VANE_LENGTH/2)) 	RMB.fVc = RMB.fVref ;
//			else if (RMB.fVref >= __sqrt(2*RMB.fAccel*VANE_LENGTH)) 	RMB.fVc = fVc_MIN ;
//			else 	RMB.fVc = __sqrt(temp) ;
//
//			MCP.OUT_VoiceFlg = 1 ;
//		}
//		if ((BDIB_HOU == 0)&&(BDIB_HOD == 1)&&(BDIB_DOWN == 1)&&(Floor.uFloorReal == Floor.uFloorStop))
//		{
//			temp = (6*RMB.fAccel*VANE_LENGTH - 3*RMB.fVref*RMB.fVref) ;
//			if (RMB.fVref <= __sqrt(3*RMB.fAccel*VANE_LENGTH/2)) 	RMB.fVc = RMB.fVref ;
//			else if (RMB.fVref >= __sqrt(2*RMB.fAccel*VANE_LENGTH)) 	RMB.fVc = fVc_MIN ;
//			else 	RMB.fVc = __sqrt(temp) ;
//
//			MCP.OUT_VoiceFlg = 1 ;
//		}
//	}
//	if (Floor.uMode == FC_INITIAL)
//	{
//		if (BDIB_UP == 1)
//		{
//			if ((Floor.uFloorReal == (Floor.uFloorTop - 1)) && (BDIB_HOU == 1) && (BDIB_HOD == 0))
//			{
//				Floor.uFloorReal++ ;
//				return ;
//			}
//			if ((BDIB_HOU == 1)&&(BDIB_HOD == 1))
//			{
//				if(BDIB_DZ == 1) Floor.udoorzone++ ;
//				if (Floor.uFloorReal != Floor.uFloorTop)
//				{
//				 	Floor.uFloorReal++ ;
//				 	//Non-Stop floor level setting
//					if((Floor.uFloorReal > 1)&&(NumOfnonStop > nonStop_Cnt)&&(nonStop_Flr[nonStop_Cnt] > 1)&&(nonStop_Flr[nonStop_Cnt] < Floor.uFloorTop))//non-stop
//					{
//						while((nonStop_Flr[nonStop_Cnt] == Floor.uFloorReal)&&(Floor.uFloorReal < Floor.uFloorTop)&&(nonStop_LoopCnt <= NumOfnonStop))
//						{
//							nonStop_LoopCnt++;
//							b = 0;
//							while((b < nonStop_LoopCnt)&&(b <= NumOfnonStop)) //case consecutive non-stop floor level, previous floor is set again.
//							{
//								Floor.fLevel[Floor.uFloorReal-b] = (((Floor.fPosition + VANE_EDGE_LENGTH) - Floor.fLevel[Floor.uFloorReal-nonStop_LoopCnt])/(nonStop_LoopCnt+1))*(nonStop_LoopCnt-b) + Floor.fLevel[Floor.uFloorReal-nonStop_LoopCnt] ;
//								b++;
//							}
//							Floor.uFloorReal++;
//							nonStop_Cnt++;
//						}
//						nonStop_LoopCnt = 0;
//					}
//				}
//				if (Floor.uFloorReal == 1)
//				{
//					Floor.fLevel[1] = LEVEL1F ;
//					Floor.fPosition = LEVEL1F - VANE_EDGE_LENGTH ;
//					//MT_Dcnt_Sum = (long)(Floor.fPosition*INV_SM_fPositionToLength+0.5);
//					MCP.OUT_MaxFlUp = 0;
//					MCP.OUT_MaxFlDown = 0;
//				}
//				Floor.fLevel[Floor.uFloorReal] = Floor.fPosition + VANE_EDGE_LENGTH ;
//			}
//		}
//	}else
//	{
//		if(MCP.OUT_FlInitOK == 1)
//		{
//			if (((BDIB_UP == 1)&&(fabs(INV1_SC.Wrpm_ref) > fVariOffset))&&(BDIB_DZ == 1)&&((BDIB_HOU == 1)&&(BDIB_HOD == 1))&&(Van_mode == 1))
//			{
//				a = Floor.fLevel[Floor.uFloorReal] - VANE_EDGE_LENGTH;
//				if((fabs(a - Floor.fPosition) > 500.)&&(Pattern.DecelFlag != 2)) MCP.OUT_PosErr = 1 ;
//				fPositionDiff = a - Floor.fPosition;
//				Floor.fPosition = a ;
//				//MT_Dcnt_Sum = (long)(Floor.fPosition*INV_SM_fPositionToLength+0.5);
//				return ;
//			}
//			else if (((BDIB_DOWN == 1)&&(fabs(INV1_SC.Wrpm_ref) > fVariOffset))&&(BDIB_DZ == 1)&&((BDIB_HOU == 1)&&(BDIB_HOD == 1))&&(Van_mode == 2))
//			{
//				a = Floor.fLevel[Floor.uFloorReal] + VANE_EDGE_LENGTH;
//				if((fabs(a - Floor.fPosition) > 500.)&&(Pattern.DecelFlag != 2)) MCP.OUT_PosErr = 1 ;
//				fPositionDiff = a - Floor.fPosition;
//				Floor.fPosition = a ;
//				//MT_Dcnt_Sum = (long)(Floor.fPosition*INV_SM_fPositionToLength+0.5);
//			  return ;
//			}else return ;
//		}
//	}
}

