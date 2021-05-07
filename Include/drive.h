/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: FREEWAY
	Compiler    		: TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.07.18
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
						: Added SCI Protocol Infmation Response Part (20181107 by GWON)
						: Rearrangement of F/W for FREEWAY (20181112 by GWON)
******************************************************************************/

#ifndef _DRIVE_
#define _DRIVE_

#include "Variable.h"

#define regMainStatus    _MAINSTAT._intval
#define regMainStatusBit _MAINSTAT._bitval
#define regPrevMainStatus    _PREVMAINSTAT._intval
#define regPrevMainStatusBit _PREVMAINSTAT._bitval
#define DRV_DEAD_TIMER		100

/* Drive Status  ID(Identification) */
#define DRV_FAULT			0x00
#define DRV_READY			0x01
#define DRV_CONVRUN			0X03	//0x13	FOR FREEWAY 20180205
#define DRV_INVINIT			0X07	//0x17	FOR FREEWAY 20180205
#define DRV_RUN				0X0F	//0x1F	FOR FREEWAY 20180205
#define DRV_DECEL			0X0D	//0x1D	FOR FREEWAY 20180205
#define DRV_DCBON			0X05	//0x15	FOR FREEWAY 20180205
#define DRV_RESET			0x80

/* Drive Status flag ID(Identification) */
#define	DRV_READY_FLG		0x01
#define	DRV_CMD_FLG			0x02
#define DRV_CC_FLG			0x04
#define DRV_SC_FLG			0x08
#define	DRV_CONV_FLG		0x10

#define DRV_USEBIT			0x1F

__interrupt void cpu1_drive(void) ;

typedef union
{
	Uint16 _intval;
	struct {
		Uint16	flgReady		:1;
		Uint16	flgCmd			:1;
		Uint16	flgCC		   	:1; /* CC of INV */
		Uint16	flgSC			:1; /* SC of INV */
		Uint16	flgConv			:1; /* Converter Run */
		Uint16 __reserved		:11;
	} _bitval;
} DRIVESTAT_REG;

/* The definition for Load Cell type interface board setting */
#define  LOAD_0SET                0xD3       //0% Load setting 
#define  LOAD_50SET               0xD5       //50% Load setting 
#define  LOAD_RESET               0xD9       //Load re-setting 
#define  LOAD_SETEND              0xD0       //setting end 
#define  LOAD_STORECALL           0xB0       //LS board store-data call 
#define  LOAD_DATASEND            0xA0       //Host data sent 
#define  LOAD_DATACALL            0x90       //LS board data call 

typedef struct _Drive {
	long		flgReset;
	DRIVESTAT_REG	_MAINSTAT;
	DRIVESTAT_REG	_PREVMAINSTAT;
} Drive;

typedef struct _OLPD {
	Uint16	iOLTime1;		
	Uint16	iOLTime2;		
	Uint16	iOLTime3;		
	float	fOL120;		
	float	fOL150;		
	float	fOL200;		
} Olpd;

typedef struct	_DA_LIST_
{
	int			Name;		
	int 		Value ;
	float		*Address;	
}	DA_List, *pDA_List;
	
Uint16	DRV_GetStatus( void );
Uint16 DRV_CheckCmdOff( void );
void DRV_UpdateStatus( void );
void DRV_Create(void);
Uint16 DRV_GetSCStatus( void );
Uint16 DRV_GetCCStatus( void );
void DRV_Initialize(void);
void SinCosDir(void);
void BK_CLOSE(void);
void BK_RELEASE(void);

#endif /* #ifndef _DRIVE_ */
