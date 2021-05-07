/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#ifndef _INIT_CMPSS_H
#define _INIT_CMPSS_H

#include "F28x_Project.h"

//definitions for selecting DACH reference
#define REFERENCE_VDDA     0
#define REFERENCE_VDAC     1

//definitions for COMPH input selection
#define NEGIN_DAC          0
#define NEGIN_PIN          1

//definitions for CTRIPH/CTRIPOUTH output selection
#define CTRIP_ASYNCH       0
#define CTRIP_SYNCH        1
#define CTRIP_FILTER       2
#define CTRIP_LATCH        3

//definitions for selecting output pin
#define GPIO_CTRIP_PIN_NUM      14 //XTRIPOUT3 is mux'd with GPIO14
#define GPIO_CTRIP_PER_NUM       6 //XTRIPOUT3 is peripheral option 6 for GPIO14
#define GPIO_CTRIPOUT_PIN_NUM   15 //EPWM8B is mux'd with GPIO15
#define GPIO_CTRIPOUT_PER_NUM    1 //EPWM8B is peripheral option 1 for GPIO15

#define NOT_INVERTED 	0
#define INVERTED 		1



extern void InitCMPSS(void);


#endif  // end of SNU_CPU1_CMPSS_H definition
