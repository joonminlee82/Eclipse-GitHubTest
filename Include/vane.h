/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler TI_v6.4.2
	Author				: Inverter part, E/C Department in Hyudai elevator
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#ifndef _VANE_
#define _VANE_
 
#include "Variable.h"
#include "bdib.h"
//#include "fc.h"

//#define VANE_LENGTH			270
#define VANE_LENGTH			130
#define VANE_EDGE_LENGTH	10
#define VANE_DZ_EDGE_LENGTH			65
#define VANE_TOTAL_LENGTH ((float)(140))

__interrupt void cpu1_vane(void);
void vane_c(void) ;

#endif
