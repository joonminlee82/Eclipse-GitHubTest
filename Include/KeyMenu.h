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
#ifndef	_KEY_DATA_H_
#define	_KEY_DATA_H_

#include "variable.h"
#include "F28x_Project.h" 
#include "gnl.h"

//#define PASSWORD1				02 	
//#define PASSWORD2				03 
//#define PASSWORD3				12 	
//#define PASSWORD4				05 
//#define PASSWORD_DRIVE_SW1	34 	
//#define PASSWORD_DRIVE_SW2	52

/* Item Number */
#define MONITORNUM				4		
#define	PROGRAMNUM				4
#define	MOTORMENUNUM			2

#define BASICNUM				21
#define IONUM					8
#define ERRORNUM				70	

#define CONTROLNUM				19
#define INTERFACENUM			14
#define SYNMOTORNUM				15
#define LINEARMOTORNUM			15

#define FACTORYNUM				19	//46
#define INVERTERNUM				6
#define INVERTERSELECTNUM		18		
#define FLOORNUM				80	//138	
	
#define KEY_START				0
#define KEY_CONTROL_START		3	// This vaule is changed 0 to 3 in order to fix EL_SPEED and ROPING data. (20171214 by GWON)
#define KEY_INTERFACE_START		(KEY_CONTROL_START + CONTROLNUM)
#define KEY_LINMOTOR_START		(KEY_INTERFACE_START + INTERFACENUM)
#define KEY_FACTORY_START		(KEY_LINMOTOR_START + LINEARMOTORNUM)		// Modified for FREEWAY 20180206 by GWON
#define KEY_END					(KEY_FACTORY_START + FACTORYNUM)
#define KEY_EEPROM_VERSION		(KEY_END + 1)
#define DRV_CNT_SW				(KEY_EEPROM_VERSION + 1)
#define DRV_CNT_NUM				(DRV_CNT_SW + 1)
#define KEY_TOTAL				DRV_CNT_NUM

/* Parameter Number */
#define TQSELECTNUM				6
#define INDMOTORSELECTNUM		22	
#define ONOFFSELECTNUM			2
#define UPDNSELECTNUM			2  
#define DECELMODENUM			2
#define YESNOSELECTNUM			2
#define ROTORPOSESTISELNUM		2
#define ENCPPRNUM				8
#define DRIVEMODENUM			17
#define OKFAILSELECTNUM		 	2
#define SPDPTNNUM				6
#define OFFUPDNSELECTNUM  		3
#define MANSPDSELNUM			8
#define MANMODESELNUM			4		// Added for FREEWAY 20180212 by GWON
#define INVCTLMODENUM			5
#define MOTORTYPESELECTNUM 		2
#define THREESELECTNUM		 	3
#define AOSELECTNUM         	46
#define ENCODERTYPENUM		 	2
#define MAXNONSTOPNUM			64
#define FWDREVSELECTNUM			2
#define ELDSPDSELECTNUM			2
#define	TQRIPPLESELECTNUM		7

#define ERRORSTORENUM			10
#define ERRORTIMENUM			ERRORSTORENUM
#define ERRORBACKUPDATANUM		50
#define ERRORTOTALDATANUM		ERRORSTORENUM + 6*ERRORTIMENUM + ERRORBACKUPDATANUM*ERRORSTORENUM
/* Select Menu Mode */
#define	MONITORMENU				0
#define	PROGRAMMENU				1
#define	BASICMENU				2
#define	IOMENU					3
#define	ERRORMENU				4
#define FLOORMENU				5
#define	CONTROLMENU				6
#define	INTERFACEMENU			7
#define	MOTORMENU				8
#define	FACTORYMENU				9
#define SYNMOTOR				10		// Added for FREEWAY 20180206 by GWON
#define LINEARMOTOR				11		// Added for FREEWAY 20180206 by GWON

#define MONITOR					1
#define ERROR					0

#define TOTALMENUNUM		 	FACTORYMENU+1

#define MOTORSELCTIONMENU		2		// Added for FREEWAY 20180206 by GWON

typedef enum
{	BASIC_NAME,				BASIC_EL_SPEED,			BASIC_SPD_FBK,				BASIC_SPD_REF,				BASIC_CURRENT_INV,		
	BASIC_VOLTAGE_INV,		BASIC_CURRENT_CONV,		BASIC_VIN,					BASIC_DCLINK,				BASIC_VDSEINTEG,						
	BASIC_LOAD_PULSE,		BASIC_IA_INV,			BASIC_IB_INV,				BASIC_IC_INV,				BASIC_FLOOR,						
	BASIC_CALL,				BASIC_MODE,				BASIC_SCI_ST,				BASIC_DECEL_FLG,			BASIC_INIT, 						
	BASIC_SDS							
}	BASIC_ITEM_LIST;
	
typedef enum	
{	EL_SPEED,				MAX_SPEED,	  			MAX_ACC,					MAX_JERK,					INSPECT_SPEED,
	CREEP_SPEED,			RELEVEL_SPEED,			ANTISTALL_TIME,				MAX_FLOOR,					FWD_DIRECTION,
	SC_JM,					SC_FF_GAIN,				INIT_START,                 CURRENT_POS,                CAR_TYPE,
	MAX_LENGTH,             ZERO_CAL,               ACCUM,                      SEL_POS
}	CONTROL_ITEM_LIST;

typedef enum
{	
	ZSP_LEVEL,				SPDCTL_ERR_LVL,				AO1_SELECT,				AO2_SELECT,					AO3_SELECT,
	AO4_SELECT,				AO_1_CENTER,				AO_1_RANGE,				AO_2_CENTER,				AO_2_RANGE,
	AO_3_CENTER,			AO_3_RANGE,					AO_4_CENTER,			AO_4_RANGE
}	INTERFACE_ITEM_LIST;
	
typedef enum
{	SYN_MOTOR_SELECT,				SYN_MOTOR_CAPACIT, 				SYN_MOTOR_RATING_V,				SYN_MOTOR_RATING_A,			SYN_MOTOR_POLES,		
	SYN_MOTOR_ENC_PPR,				SYN_MOTOR_BASE_RPM, 			SYN_MOTOR_IQSE_RATE, 			SYN_MOTOR_KNOW_RPOS,	  	SYN_MOTOR_RPOS_ENC_OFFSET,
	SYN_MOTOR_RPOS_ESTIMAION,		SYN_MOTOR_RPOS_ESTI_TIME,		SYN_MOTOR_LS,					SYN_MOTOR_RS,				SYN_MOTOR_KE, 				
	SYN_MOTOR_SHEAVE_DIA,			SYN_OR_LIN_MOTOR_FLAG
}	SYN_MOTOR_ITEM_LIST;

typedef enum		// Added for FREEWAY 20182026 by GWON
{	LIN_MOTOR_SELECT,				LIN_MOTOR_CAPACIT, 				LIN_MOTOR_RATING_V,				LIN_MOTOR_RATING_A,			LIN_MOTOR_POLE_PITCH,		
	LIN_MOTOR_ENC_PPR,				LIN_MOTOR_BASE_SPEED, 			LIN_MOTOR_IQSE_RATE, 			LIN_MOTOR_KNOW_RPOS,	  	LIN_MOTOR_RPOS_ENC_OFFSET,
	LIN_MOTOR_RPOS_ESTIMAION,		LIN_MOTOR_RPOS_ESTI_TIME,		LIN_MOTOR_LS,					LIN_MOTOR_RS,				LIN_MOTOR_KF
}	LIN_MOTOR_ITEM_LIST;

typedef enum
{	INVERTER_SELECT,			MOTOR_TYPE,							INV_CTL_MODE,					INPUT_VOLTAGE,				THRUST_FORCE_LIMIT,
	SCALE_VDC,					SCALE_IS, 							INV_WCC,						WSC,						WPC,
	INV_OC_SET,					OS_SET,								ENCDIR,							DC_CAPACITOR,				ANGLE_RST_VTG,
	ENCODERTYPE,				ERROR_ERASE,						ROM_VERSION,					FRAM_VERSION
}	FACTORY_ITEM_LIST;

typedef enum	/* AOSELECT_ITEM_NUM = 25	 */
{	AO_SPEED_FBK,				AO_SPEED_REF,				AO_IQSE_FBK,				AO_IQSE_REF, 				AO_IDSE_FBK,
	AO_IDSE_REF,				AO_VOLTAGE,					AO_LINK_VDC,				AO_IU_CURRENT,				AO_IV_CURRENT,				
	AO_IW_CURRENT,				AO_DRIVE_STATE,				AO_VDSE_INT,				AO_REMAIN_LENGTH,			AO_CPCALL_DATA,			
	AO_REALFLOOR, 				AO_DECELFLOOR,				AO_STOPFLOOR,				AO_CPDRIVE_MODE,			AO_REAL_POSITION,	
	AO_UPDN_CMD,				AO_PTN_MODE,				AO_DECEL_FLAG,				AO_FAULT_FLAG,				AO_RMB_VREF	
}	AOSELECT_ITEM_LIST;

typedef enum	/* SPD_PATTERN_NUM = 5	 */
{	NORMAL_V2TH,				DOUBLE_SPD_PTN,					NORMAL_V3TH,					OPTIMAL_SPD_PTN,			ST12_SPD_PTN, 				
	ST12_ES_SPD_PTN
}	SPD_PATTERN_SELECT;

typedef enum	/* INV_CTL_MODE_NUM = 4	 */
{	SENSED_IV_MODE,				HYBRID_IV_MODE,				SENSELESS_VF_MODE,					SENSELESS_DV_MODE,					AUTO_TUNE_MODE
}	INV_CTL_MODE_SELECT;

typedef enum	/* MOTOR_TYPE_SELECT_NUM = 2	 */
{	SYNCHRONOUS_MOTOR,				INDUCTION_MOTOR
}	MOTOR_TYPE_SELECT;

typedef enum	/* ROTOR_POS_ESTIMATION_SELECT_NUM = 2	 */
{	HIGH_FREQ_INJECT,				DC_INJECT
}	ROTOR_POS_ESTIMATION_SELECT;

typedef enum	/* ENCODER_TYPE_SELECT_NUM = 2	 */
{	INCREMENTAL_ENC,				SIN_COS_ENC
}	ENCODER_TYPE_SELECT;

typedef enum	/* TQBIAS_SELECT_NUM = 6	 */
{	NO_USE,		AUTO_MODE,	AUTO_LS_DATA_MODE, LS_DATA_MODE, LV12_MODE, 	LMI_DATA_MODE
}	TQBIAS_SELECT;

typedef enum
{ CREEP_MODE,	PLO_MODE
} ELD_SPEED_SELECT;

typedef enum	/* INV_CAPACITY_SELECT_NUM = 2	 */
{	WB210_5P5KW,		WB210_7P5KW,	     	WB210_11KW, 	    WB210_15KW, 	     	WB150_22KW,
	WB150_30KW,        	WB150_40KW,			 	WB150_50KW,			WB150SSM_22KW,			WB150SSM_30KW,
	WB150SSM_40KW,		WB150SSM_HY_40KW,		WB150SSF_22KW,		WB150SSF_30KW,			UELCO_40KW,
	OTHER_INVERTER
}	INV_CAPA_SELECT;

typedef struct {
    double MIN;		
    double MAX;	
} DATA;

typedef struct {
    char *display;
    DATA limit;
    char *unit;
    double value;		
    double incdec;	
} ITEM;

typedef struct {
    char *display;
    char *unit;
    float value;		
    float incdec;	
} ITEMBASIC;

typedef struct {
	char *display;
  float value;
} ITEMDISPLAY;

typedef struct {
	char *display;
	DATA limit;
} ITEMMINMAX;

void KEY_FLT_GetStatus(void);
void FaultError(void);
void ErrorTimeDisplay(int index,int *ErrNum, int *Err_Year, int *Err_Mon, int *Err_Date, int *Err_h, int *Err_m, int *Err_s, unsigned long *Err_Backup);
void MotorEncPPRComputation(char key, float *value, double incdec, float min, float max);
void ErrorGet(Uns *NewErr, u8 *NewFault_RealYear, u8 *NewFault_RealMonth, u8 *NewFault_RealDate, u8 *NewFault_RealHour, u8 *NewFault_RealMinute, u8 *NewFault_RealSec, u8 *NewFault_Backup);
void ErrorDisplay(int index, int CurRow, int FaultID);
void ErrorBackupDisplay(int index,int ErrNum, char *Time, unsigned long *Err_Backup_Data);
void ErrorStoreToSci(Bool value);
float KEY_GetControlCode(int ControlID);
float KEY_GetInterfaceCode(int InterfaceID);
float KEY_GetLinMotorCode(int MotorID);
float KEY_GetFactoryCode(int FactoryID);

void KEY_SetBasicCode(int BasicID, float value);
void KEY_SetControlCode(int ControlID, float value);
void KEY_SetInterfaceCode(int InterfaceID, float value);
void KEY_SetLinMotorCode(int MotorID, float value);
void KEY_SetFactoryCode(int FactoryID, float value);
void GotoArrow(int CurRow);
#endif	/* __KEYMENU_H_ */
