/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: FREEWAY 
	Compiler    		: TMS320F28377D C-Compiler v6.1.2
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.02.06
	History				: Linear Motor Selection Menu is added into the HHT Menu (20180206 by GWON)
						: Motor Side R Controller is added in order to suppress 3th, 6th 9th harmonics, but this controllers are disable. (20180206 by GWON)
						: Mark to need to modify for FREEWAY --> Search command "// Need to modify for FREEWAY" (20180209 by GWON)
******************************************************************************/

#ifndef __FRAM_H_
#define __FRAM_H_

//#include "gnl.h"
#include "F28x_Project.h" 

//#define FRAM_MAXINDEX 	0x800	/*2 k * 32bit, 8 k * 8bit*/
#define FRAM_MAXINDEX 	0x400		/*2 k * 32bit, 8 k * 8bit*/ // Modified by GJS because 28377D released version chip charateristic

typedef union
{
	unsigned int UnsIntValue;
	unsigned long   IntValue;
	float FloatValue;
 
 	struct
 	{	 	 
 		unsigned int Word0	: 16;
		unsigned int Word1	: 16;
    	} WordValue;	/* 16 bit */
    	struct
    	{ 	 
    		unsigned int Byte0	: 8;
		unsigned int Byte1	: 8;
		unsigned int Byte2	: 8;
		unsigned int Byte3	: 8;
    	 } ByteValue;	/* 8 bit */
} GlobalData;

void  	FRAM_ErrWrite(Uint16 E_Index, Uint16 E_Value);
Uint16	FRAM_ErrRead(Uint16 E_Index);
void    FRAM_Write(Uint16 E_Index, float E_Value); 
float	FRAM_Read(Uint16 E_Index);

void    FRAM_IntWrite(Uint16 E_Index, Uint16 E_Value);
Uint16	FRAM_IntRead(Uint16 E_Index);
void    FRAM_ErrErase(void);
void    FRAM_Check(void);
void    FRAM_Erase(void);
void    FRAM_Init(void);
float   GetROMTable(Uint16 R_Index);
void    SetROMTable(void);
void	FRAM_Load(void);
void    GetROMTableAll(void);
void	FRAM_VersionCheck(void);
float	ROM_GetVersion(void);
float	FRAM_GetVersion(void);
void	FRAM_SetVersion(float ver);
void	FRAM_ProgressBar(Uint16 persent);

#endif	/* __EEPROM_H_ */




