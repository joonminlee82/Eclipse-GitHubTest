/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: FREEWAY 
	Compiler    		: TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.04.18
	History				: Linear Motor Selection Menu is added into the HHT Menu (20180206 by GWON)
						: Motor Side R Controller is added in order to suppress 3th, 6th 9th harmonics, but this controllers are disable. (20180206 by GWON)
						: Mark to need to modify for FREEWAY --> Search command "// Need to modify for FREEWAY" (20180209 by GWON)
						: Modified something for Linear Motor. ex) pole_Pitch, polepair, ... etc., (20180417 by GWON)
						: Modified Position and Mechinical/Electrical Angle Calculation Function (20180418 by GWON)
******************************************************************************/
//#include "Device.h"
#include "FRAM.h"
#include "DAC.h"
#include "comp.h"
#include "Variable.h"
#include "KeyMenu.h"
#include "Fault.h"
//#include "fc.h"
#include "gnl.h"
#include "Device.h"

extern volatile float  *ROMTableStart	= (float *) 0x01A000;//GS14 RAM
volatile int	*FRAM_Start		= (int *) EMIF2_CS2;
volatile int	*FRAM_ErrStart		= (int *)(EMIF2_CS2 + 0x800);

extern	Flt 	FLT;
extern 	ITEM 	ControlList[];
extern 	ITEM 	InterfaceList[];
extern 	ITEM 	SynMotorList[];
extern 	ITEM 	LinearMotorList[];
//extern 	ITEM 	IndMotorList[];
extern 	ITEM 	FactoryList[];

void 	ItoaInt(int num, int blank, char *buf)
{
	int shift, unitnum;

	unitnum = 10;
	shift = 0;

	while ( num / unitnum )
	{
		unitnum *=10;
		shift++;
	}
	unitnum /= 10;

	while (shift+1)
	{
		buf[LCDLEN-(shift+blank+1)] = (num / unitnum) + 0x30;
		num %= unitnum;
		unitnum /= 10;
		shift--;
	}
}

void FRAM_Write(Uint16 E_Index, float E_Value) 
{ 
	nCS_FRAM_ON;//DAC와 충돌 Check!!
	GlobalData temp;
	
	temp.FloatValue = E_Value;
	E_Index = (E_Index+1) * 4;
	NOP4();
	*(FRAM_Start + E_Index - 4) = temp.ByteValue.Byte0 & 0xFF;
	NOP4();
	NOP();
 	*(FRAM_Start + E_Index - 3) = temp.ByteValue.Byte1 & 0xFF;
 	NOP4();
	NOP();
 	*(FRAM_Start + E_Index - 2) = temp.ByteValue.Byte2 & 0xFF;
 	NOP4();
	NOP();
 	*(FRAM_Start + E_Index - 1) = temp.ByteValue.Byte3 & 0xFF;
 	NOP4();
 	nCS_FRAM_OFF;
}


float	FRAM_Read(Uint16 E_Index)
{	
	nCS_FRAM_ON;
	GlobalData temp;

	E_Index = (E_Index+1) * 4;
	NOP4();
  	temp.ByteValue.Byte0 = *(FRAM_Start + E_Index - 4) & 0xFF;
  	NOP4();
  	NOP();
 	temp.ByteValue.Byte1 = *(FRAM_Start + E_Index - 3) & 0xFF;
 	NOP4();
 	NOP();
 	temp.ByteValue.Byte2 = *(FRAM_Start + E_Index - 2) & 0xFF;
 	NOP4();
 	NOP();
 	temp.ByteValue.Byte3 = *(FRAM_Start + E_Index - 1) & 0xFF; 
 	NOP4();
	nCS_FRAM_OFF;
	return temp.FloatValue;
}

void FRAM_ErrWrite(Uint16 E_Index, Uint16 E_Value)	
{	
	nCS_FRAM_ON;
	GlobalData temp; 

	temp.IntValue = E_Value;
	E_Index = (E_Index+1) * 4;
	
	NOP4();
	*(FRAM_ErrStart + E_Index - 4) = temp.ByteValue.Byte0 & 0xFF;
	NOP4();
	NOP(); 
 	*(FRAM_ErrStart + E_Index - 3) = temp.ByteValue.Byte1 & 0xFF;
 	NOP4();
	NOP(); 
	*(FRAM_ErrStart + E_Index - 2) = temp.ByteValue.Byte2 & 0xFF;
	NOP4();
	NOP(); 
 	*(FRAM_ErrStart + E_Index - 1) = temp.ByteValue.Byte3 & 0xFF;
 	NOP4();
 	nCS_FRAM_OFF;
}

Uint16	FRAM_ErrRead(Uint16 E_Index)	
{	
	nCS_FRAM_ON;
	GlobalData temp; 
	E_Index = (E_Index+1) * 4;

	NOP4();
	temp.ByteValue.Byte0 = *(FRAM_ErrStart + E_Index - 4) & 0xFF;
	NOP();
  	temp.ByteValue.Byte1 = *(FRAM_ErrStart + E_Index - 3) & 0xFF;
  	NOP();
  	temp.ByteValue.Byte2 = *(FRAM_ErrStart + E_Index - 2) & 0xFF;
  	NOP();
  	temp.ByteValue.Byte3 = *(FRAM_ErrStart + E_Index - 1) & 0xFF; 
  	NOP();
  	NOP4();
	nCS_FRAM_OFF;	
	return temp.IntValue;
}

void FRAM_Check(void)
{	
	long Addr ;
	
//	LCDClearScreen();
//	WriteStringXY(0, 0, "CHECKING FRAM ");

	//Addr = FRAM_MAXINDEX-5;
	Addr = FRAM_MAXINDEX-1;	// Modified by GJS because 28377D released version chip charateristic
	
	FRAM_Write(Addr,E_Check);
	NOP();
	NOP4();
	FRAM_value = FRAM_Read(Addr);
//	if ( FRAM_value != E_Check){
//		WriteStringXY(0, 1, " ERROR ");
//	}
//	else	WriteStringXY(0, 0," CHECKING GOOD  ");
}

void    FRAM_Erase(void)
{   	
	int k;
	GlobalData GValue;

//	LCDClearScreen();
//	WriteStringXY(0, 0, "    FRAM_ERASE    ");

	GValue.ByteValue.Byte0 = 0x00;
	GValue.ByteValue.Byte1 = 0x00;
	GValue.ByteValue.Byte2 = 0x00;
	GValue.ByteValue.Byte3 = 0x00;
	 
  for(k=0; k < FRAM_MAXINDEX; k++){
    FRAM_Write(k, GValue.FloatValue);
	}
}

void	FRAM_ErrErase(void)
{   
	int i;
	GlobalData GValue;
	int ErrorStoreNum = ERRORTOTALDATANUM;

//	LCDClearScreen();
//	WriteStringXY(0, 0, "    ERROR ERASE     ");

	GValue.ByteValue.Byte0 = 0x00;
	GValue.ByteValue.Byte1 = 0x00;
	GValue.ByteValue.Byte2 = 0x00;
	GValue.ByteValue.Byte3 = 0x00;
    	
  for(i=0; i < ErrorStoreNum; i++){
    FRAM_ErrWrite(i, GValue.FloatValue);
	}
}

void FRAM_Init()
{ 	
	int i;
	
//	LCDClearScreen();
//	WriteStringXY(0, 0, "  EEPROM INIT.  ");
	
  for(i=0; i < KEY_TOTAL; i++){
		if ( i < KEY_INTERFACE_START )
		   	FRAM_Write(i, ControlList[i-KEY_CONTROL_START].value);
		else if ( i < KEY_LINMOTOR_START )
			FRAM_Write(i, InterfaceList[i-KEY_INTERFACE_START].value);
		else if ( i < KEY_FACTORY_START )
			FRAM_Write(i, LinearMotorList[i-KEY_LINMOTOR_START].value);
		else if ( i < KEY_END )
			FRAM_Write(i, FactoryList[i-KEY_FACTORY_START].value);
	}

  FRAM_SetVersion(FactoryList[ROM_VERSION].value);
}

float   GetROMTable(Uint16 R_Index)
{	return *(ROMTableStart + R_Index);	}

void    GetROMTableAll(void)
{	
	int i;
	float temp;

  	for(i=0; i < CONTROLNUM; i++){
		temp = *(ROMTableStart+KEY_CONTROL_START+i);
		ControlList[i].value = temp;
	}
  	for(i=0; i < INTERFACENUM; i++){
		temp = *(ROMTableStart+KEY_INTERFACE_START+i);
		InterfaceList[i].value = temp;
	}
	for(i=0; i < LINEARMOTORNUM; i++){
		temp = *(ROMTableStart+KEY_LINMOTOR_START+i);
		LinearMotorList[i].value = temp;
	}
  	for(i=0; i < FACTORYNUM; i++){
		temp = *(ROMTableStart+KEY_FACTORY_START+i);
    	FactoryList[i].value = temp;
	}
	*(ROMTableStart+KEY_FACTORY_START+FRAM_VERSION) = FRAM_GetVersion();
}

void SetROMTable(void)
{	
	volatile int i;
	volatile float temp;

  	for(i=0; i < CONTROLNUM; i++){	
  		temp = ControlList[i].value;
		*(ROMTableStart+KEY_CONTROL_START+i) = temp;
	}
  	for(i=0; i < INTERFACENUM;	i++){	
  		temp = InterfaceList[i].value;
		*(ROMTableStart+KEY_INTERFACE_START+i) = temp;
	}
	for(i=0; i < LINEARMOTORNUM; i++){
  		temp = LinearMotorList[i].value;
		*(ROMTableStart+KEY_LINMOTOR_START+i) = temp;
	}
  	for(i=0; i < FACTORYNUM; i++){	
  		temp = FactoryList[i].value;
		*(ROMTableStart+KEY_FACTORY_START+i) = temp;
	}

	*(ROMTableStart+KEY_FACTORY_START+FRAM_VERSION) = FRAM_GetVersion();
}

void FRAM_Load(void)
{	
	int i=0;
	float temp=0. ;
	
	for(i=0; i<CONTROLNUM; i++){	
		temp = FRAM_Read(KEY_CONTROL_START+i);
		ControlList[i].value = temp;
	}
	for(i=0; i<INTERFACENUM; i++){	
		temp = FRAM_Read(KEY_INTERFACE_START+i);
		InterfaceList[i].value = temp;
	}
	for(i=0; i<LINEARMOTORNUM; i++){
		temp = FRAM_Read(KEY_LINMOTOR_START+i);
		LinearMotorList[i].value = temp;
	}
	for(i=0; i<FACTORYNUM; i++){	
		temp = FRAM_Read(KEY_FACTORY_START+i);
		FactoryList[i].value = temp;
	}
	FactoryList[FRAM_VERSION].value = FRAM_GetVersion();
	*(ROMTableStart+KEY_FACTORY_START+FRAM_VERSION) = FRAM_GetVersion();
}

void FRAM_VersionCheck(void)
{
	
	float RomVer=0, FRAM_Ver=0;
	
	FRAM_Ver = FRAM_GetVersion();
	RomVer = ROM_GetVersion();
	FactoryList[ROM_VERSION].value = RomVer;
	FactoryList[FRAM_VERSION].value = FRAM_Ver;
	
	if (((int)((RomVer+0.001)*100)) == ((int)((FRAM_Ver+0.001)*100)) )	FRAM_Load();	/*Eeprom에서 RAM으로 Data 이동*/
	else{		FLT_Raise( FLT_VERSION );}
}

float ROM_GetVersion(void)
{	return	FactoryList[ROM_VERSION].value;	}

float FRAM_GetVersion(void)
{	return FRAM_Read(KEY_FACTORY_START+FRAM_VERSION);	}

void FRAM_SetVersion(float ver)
{	FRAM_Write(KEY_FACTORY_START+FRAM_VERSION, ver);	}

void 	FRAM_ProgressBar(Uint16 persent)
{					/*   1234567890123456	*/
	char	Bar[LCDLEN]="                    ";
	int		i, BarEnd;

	BarEnd = (int) (persent / 10.);

	for (i=0; i<BarEnd; i++)
		Bar[i] = 0x0ff;
	for (i=BarEnd; i<10 ; i++)
		Bar[i] = '=';

	ItoaInt(persent, 1, Bar);//100자리 계산
	Bar[19] = '%';

	WriteStringXY(0, 0, Bar);
}

