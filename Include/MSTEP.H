/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler TI_v6.4.2
	Author				: Inverter part, E/C Department in Hyudai elevator
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/

#ifndef _MSTEP_
#define _MSTEP_

#include "F28x_Project.h"

/**********************************************************************/
/* Definition for MSTEP ID(Identification)                            */
/**********************************************************************/
#define MSTEP_STOP1			0
#define MSTEP_STOP2			1
#define MSTEP_INITIAL		2
#define MSTEP_CREEP			3
#define MSTEP_INSPECTION	4
#define MSTEP_SHORTRUN		5
#define MSTEP_RELEVEL		6
#define MSTEP_LONGRUN		7
#define MSTEP_REFOFF		0x10

/**********************************************************************/
/* Multistep Input Block                                              */
/**********************************************************************/
typedef struct _MStep {
	float	fRelevel;
	float	fInitial;
	float	fCreep;
	float	fInspection;
	float	fShortRun;
	float	fLongRun;
	Uint16		uCommand;
	float	fRef;
} MStep;

void MSTEP_Create( void );
void MSTEP_Update( void ); 
Uint16 MSTEP_GetCommandRef( void );
float  MSTEP_GetSpeedRef( void );
#endif /* #ifndef _MSTEP_ */ 
