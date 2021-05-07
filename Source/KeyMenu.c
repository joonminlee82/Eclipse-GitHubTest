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
#include "CC.h"
#include "SC.h"
#include "Fault.h"
#include "drive.h"
#include "device.h"
#include "FRAM.h"
#include "keydata.h"
#include "keymenu.h"
#include "Variable.h"
#include "bdib.h"
#include "SCI.h"
#include "GNL.h"
#include "comp.h"
#include <math.h>
#include <string.h>

extern 	Flt 		FLT ;
extern 	ITEM 		ControlList[];
extern 	ITEM 		InterfaceList[];
extern 	ITEM 		FactoryList[];
extern 	ITEM		LinearMotorList[];	// Added for FREEWAY 20180206 by GWON

extern  STATUS_ARM_OBJ  g_SpiArmStatus;
extern  IdF5Data        g_IdF5Data[10];

//extern DA_List	DaList[4];

Bool	FaultFlag = FALSE, FaultWarnFlag = FALSE;
unsigned long gErr_Fault_Backup[ERRORSTORENUM][ERRORBACKUPDATANUM] = {0}, gOldFault_Backup[ERRORSTORENUM][ERRORBACKUPDATANUM] = {0};

void KEY_FLT_GetStatus(void)	/*Fault Update*/
{
	if((Flags.Fault == TRUE)&&(Flags.Fault_Warn == FALSE)) {	
		if(FaultFlag == FALSE){	
			SaveFault_Flg = TRUE;
			FaultFlag = TRUE;
		}
	}
	else if((Flags.Fault == FALSE)&&(Flags.Fault_Warn == TRUE)){
		if(FaultWarnFlag == FALSE){	
			SaveFault_Flg = TRUE;
			FaultWarnFlag = TRUE;
		}
	}
	else if((Flags.Fault == TRUE)&&(Flags.Fault_Warn == TRUE)){
		if((FaultFlag == FALSE)||(FaultWarnFlag == FALSE)){	
			SaveFault_Flg = TRUE;
			FaultFlag = TRUE;
			FaultWarnFlag = TRUE;
		}
	}
	else{	
		SaveFault_Flg = FALSE;
		FaultFlag = FALSE;
		FaultWarnFlag = FALSE;
	}
}

int ErrStr[ERRORSTORENUM] = {0};
int Err_Year[ERRORSTORENUM] = {0};
int Err_Month[ERRORSTORENUM] = {0};
int Err_Date[ERRORSTORENUM] = {0};
int Err_Hour[ERRORSTORENUM] = {0};
int Err_Minute[ERRORSTORENUM] = {0};
int Err_Sec[ERRORSTORENUM] = {0};
int ct = 0;

/*----------ERROR MENU----------*/
/*******************************************************************/
/* LCD_FaultIDConv - FLT 揶쏆빘猿쒙옙占썸�⑥쥙�삢ID�몴占폥CD占쏙옙�빊�뮆�젾占쎌꼵由� 占쎄쑵釉�占쏙옙    */
/*                Error[]占쏙옙占쎌꼷肉댐옙占썸�⑥쥙�삢占쎌뮇�뻻占쎌뮇苑뚳옙占�1:1 占쏙옙�벓占쎌꼶�뮉 占썩뫁�땾*/
/* Parameters : int => Fault ID                                    */
/* Returns : int => Error[]占쏙옙占쎌꼷肉댐옙占썸�⑥쥙�삢占쎌뮇�뻻占쎌뮇苑�                  */
/* Called :  anther objects                                        */
/* Related : FLT Class                                             */
/*******************************************************************/
int LCD_FaultIDConv( int FltID )
{	
	switch(FltID){
		/* H/W 野껓옙�뀱 Fault */
		case FLT_OCAH_C :       return 1;
		case FLT_OCAH_I:       	return 2;
		case FLT_OCBH_C:       	return 3;
		case FLT_OCBH_I :		return 4;
		case FLT_OCCH_C :    	return 5;
		case FLT_OCCH_I :    	return 6;
		case FLT_OVDCH :       	return 7;
//		case FLT_OCH_SUP:       return 8;
//		case FLT_OVH_SUP:       return 9;
		case FLT_MOSFET_C :		return 10;
		case FLT_MOSFET_I :    	return 11;
//		case FLT_IGBT_SUP :    	return 12;
	
		/* S/W 野껓옙�뀱 Fault */
		case FLT_15V:      		return 13;
		case FLT_5V:     		return 14;
		case FLT_DCC:    		return 15;
		case FLT_PGBK:    		return 16;
		case FLT_ITH:   		return 17;
//		case FLT_LIMITSW:      	return 18;
		case FLT_OS:			return 19;
		case FLT_DLA:			return 20;
		case FLT_MCC:    		return 21;
		case FLT_OL:			return 22;
		case FLT_VERSION:		return 23;
		case FLT_OCAS_C:       	return 24;
		case FLT_OCAS_I:       	return 25;
		case FLT_OCBS_C:       	return 26;
		case FLT_OCBS_I:		return 27;
		case FLT_OCCS_C:    	return 28;
		case FLT_OCCS_I:    	return 29;
//		case FLT_OVBCS_C:       return 30;
//		case FLT_OVABS_C:       return 31;
		case FLT_OVDCS:       	return 32;
//		case FLT_OCS_SUP:		return 33;
//		case FLT_OVS_SUP:		return 34;
//		case FLT_SEQ_C:			return 35;
		case FLT_UVDCS:			return 36;
		case FLT_BKOP:        	return 37;
		case FLT_ENC_UVW_SEQ:   return 38;
		case FLT_SPDCTL:      	return 39;
		case FLT_OV_ANGLE:     	return 40;
//		case FLT_ENC_UVW:    	return 41;
		case FLT_OUTPUT:    	return 42;
		case FLT_DBL_CMD:   	return 43;
		case FLT_CP_WD:   		return 44;
//		case FLT_MODE	:		return 45;
//		case FLT_OVHFUS:		return 46;
		case FLT_EARTH_I:		return 47;
		case FLT_EARTH_C:		return 48;
		case FLT_CMD_OFF:		return 49;
		case FLT_UVW: 			return 50;
//		case FLT_CMD_CFM:		return 51;
		case FLT_OFFSET : 		return 52;
		case FLT_ENABLE : 		return 53;
//		case FLT_GRID_PLL_FREQ:	return 54;
//		case FLT_GRID_PLL_OV:	return 55;
//		case FLT_GRID_PLL_UV:	return 56;
		case FLT_FRAM:			return 57;
		case FLT_CC_OCS_I:		return 58;
		case FLT_CC_OCS_C:		return 59;
//		case FLT_CC_OCS_SUP:	return 60;
		case FLT_CC_OVDC:		return 61;
//		case FLT_CC_OVDC_SUP:	return 62;
//		case FLT_OUTPUT_SUP:	return 63;
//		case FLT_VDCSEN_SUP:	return 64;
		case FLT_CP_COM:		return 65;
		case FLT_OS_OBS:		return 66;
		case FLT_RUN_PLL:		return 67;
//		case FLT_ESS_WRN:		return 68;
		case FLT_UNKNOWN:   	return 69;
		default :				return 0 ;
	}
}

/*****************************************************************************/
/*  ErrorGet(int*) :                                                         */
/*****************************************************************************/
void ErrorGet(Uns *NewErr, u8 *NewFault_RealYear, u8 *NewFault_RealMonth, u8 *NewFault_RealDate, u8 *NewFault_RealHour, u8 *NewFault_RealMinute, u8 *NewFault_RealSec, u8 *NewFault_Backup)
{
//	TEST3_ON;
	Bool		UvCheck = FALSE;
	static Bool PastUvCheck = FALSE;

	int i, j, Stack = 0;
	int OldErr[ERRORSTORENUM];
	int OldErr_Year[ERRORSTORENUM],OldErr_Month[ERRORSTORENUM], OldErr_Date[ERRORSTORENUM], OldErr_Hour[ERRORSTORENUM],OldErr_Minute[ERRORSTORENUM], OldErr_Sec[ERRORSTORENUM];	
	int Err_Backup_Index_Offset = ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM;
	int Err_Backup_Index_offset1;
	for(i = 0; i < ERRORSTORENUM; i++)
	{	
		Err_Backup_Index_offset1 = Err_Backup_Index_Offset*(i+1);
		OldErr[i] = FRAM_ErrRead(i); 
		OldErr_Year[i]= FRAM_ErrRead(i+ERRORSTORENUM);
		OldErr_Month[i]= FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM);
		OldErr_Date[i]= FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM);
		OldErr_Hour[i]= FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM);
		OldErr_Minute[i]= FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM);
		OldErr_Sec[i]= FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM); 
		for(j = 0; j < ERRORBACKUPDATANUM; j++)
		{
			gOldFault_Backup[i][j] = FRAM_ErrRead(j+Err_Backup_Index_offset1);
		}   
  	}
 	while(*NewErr)	
	{				
    	ErrStr[Stack] = *NewErr++;   
    	Err_Year[Stack] = *NewFault_RealYear++;
    	Err_Month[Stack] = *NewFault_RealMonth++;
    	Err_Date[Stack] = *NewFault_RealDate++;
    	Err_Hour[Stack] = *NewFault_RealHour++;
    	Err_Minute[Stack] = *NewFault_RealMinute++;
    	Err_Sec[Stack] =*NewFault_RealSec++;
    	for(j = 0; j < ERRORBACKUPDATANUM; j++)
    	{
    		gErr_Fault_Backup[Stack][j] = NewFault_Backup[j]; 
    	}
    	Stack++;
    	if((Flags.Fault_Warn == 1)||(Flags.Fault == 1)) break;
    	if(Stack > ERRORSTORENUM) break;  
	}
	
	if((Stack == 1)&&(ErrStr[0] == FLT_UVDCS)) 
	{	
		UvCheck = TRUE;
	}

	if ((PastUvCheck == TRUE)&&(UvCheck == TRUE)) return;
 	PastUvCheck = UvCheck;
    
  	if(Stack < ERRORSTORENUM)
	{					
		i = 0;		
		while(Stack < ERRORSTORENUM)
		{	
			ErrStr[Stack] = OldErr[i];
			Err_Year[Stack] = OldErr_Year[i];
    		Err_Month[Stack] = OldErr_Month[i];
    		Err_Date[Stack] =OldErr_Date[i];
    		Err_Hour[Stack] = OldErr_Hour[i];
    		Err_Minute[Stack] = OldErr_Minute[i];
    		Err_Sec[Stack] = OldErr_Sec[i];
    	for(j = 0; j < ERRORBACKUPDATANUM; j++)
    	{
    		gErr_Fault_Backup[Stack][j] = gOldFault_Backup[i][j]; 
    	}
      	i++;
      	Stack++; 
      //if((Flags.Fault_Warn == 1)||(Flags.Fault == 1)) break;
    	}	
	}
	for(i=0; i < ERRORSTORENUM; i++)
	{
		FRAM_ErrWrite(i, ErrStr[i]);
    	FRAM_ErrWrite(i+ERRORSTORENUM, Err_Year[i]); 
    	FRAM_ErrWrite(i+ERRORSTORENUM+ERRORTIMENUM, Err_Month[i]);
    	FRAM_ErrWrite(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM, Err_Date[i]);
    	FRAM_ErrWrite(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM, Err_Hour[i]);
    	FRAM_ErrWrite(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM, Err_Minute[i]);
    	FRAM_ErrWrite(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM, Err_Sec[i]);
    for(j = 0; j < ERRORBACKUPDATANUM; j++)
    {
    	FRAM_ErrWrite(j+Err_Backup_Index_Offset*(i+1), gErr_Fault_Backup[i][j]);
    }   	
  }
  ErrGetFlag++;
//  TEST3_OFF;
}

/****************************************/
/*    Error Store to SCI Function       */
/* Author: Gogume                       */
/* History: First Released 20190308     */
/****************************************/
void ErrorStoreToSci(Bool value)
{
    int i, j;
    int Err_Backup_Index_Offset = ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM;
    int Err_Backup_Index_offset1;

    for(i = 0; i < ERRORSTORENUM; i++)
    {
        Err_Backup_Index_offset1 = Err_Backup_Index_Offset*(i+1);
        g_IdF5Data[i].u8FaultID = FRAM_ErrRead(i);
        g_IdF5Data[i].rspDate.u8Year = FRAM_ErrRead(i+ERRORSTORENUM);
        g_IdF5Data[i].rspDate.u8Month = FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM);
        g_IdF5Data[i].rspDate.u8Date = FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM);
        g_IdF5Data[i].rspTime.u8Hour = FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM);;
        g_IdF5Data[i].rspTime.u8Minute = FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM);;
        g_IdF5Data[i].rspTime.u8Second = FRAM_ErrRead(i+ERRORSTORENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM+ERRORTIMENUM);;

        for(j = 0; j < ERRORBACKUPDATANUM; j++)
        {
            g_IdF5Data[i].u8ErrorData.u8ErrorData[j] = FRAM_ErrRead(j+Err_Backup_Index_offset1);
        }
    }
}

void FaultError(void)
{
	ErrorGet(FLT.ariTable, Fault_RealYear, Fault_RealMonth, Fault_RealDate, Fault_RealHour, Fault_RealMinute, Fault_RealSec, FLT_backup);
	SaveFault_Flg = FALSE;
}

float KEY_GetControlCode(int ControlID)
{	return	ControlList[ControlID].value;	}

float KEY_GetInterfaceCode(int InterfaceID)
{	return	InterfaceList[InterfaceID].value;	}

float KEY_GetLinMotorCode(int MotorID)
{	return	LinearMotorList[MotorID].value;	}

float KEY_GetFactoryCode(int FactoryID)
{	return	FactoryList[FactoryID].value;	}

void KEY_SetBasicCode(int BasicID, float value)
{	BasicList[BasicID].value = value;	}

void KEY_SetControlCode(int ControlID, float value)
{	ControlList[ControlID].value = value;	}

void KEY_SetInterfaceCode(int InterfaceID, float value)
{	InterfaceList[InterfaceID].value = value;	}

void KEY_SetLinMotorCode(int MotorID, float value)
{	LinearMotorList[MotorID].value = value;	}

void KEY_SetFactoryCode(int FactoryID, float value)
{	FactoryList[FactoryID].value = value;	}
