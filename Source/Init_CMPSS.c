/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: FREEWAY 
	Compiler    		: TMS320F28377D C-Compiler v6.1.2
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.02.02
******************************************************************************/

#include "Init_CMPSS.h"
#include "Init_Func_Prototypes.h"

void Config_CMPSS(volatile struct CMPSS_REGS *CmpssXRegs);

void InitCMPSS(void) {
	Config_CMPSS(&Cmpss1Regs);	// ADC A2 pin,(Inv1.Ias)
//	Config_CMPSS(&Cmpss2Regs);
	Config_CMPSS(&Cmpss3Regs);	// ADC B2 pin,(Inv1.Ibs) 
//	Config_CMPSS(&Cmpss4Regs);
//	Config_CMPSS(&Cmpss5Regs);
	Config_CMPSS(&Cmpss6Regs);	// ADC C2 pin,(Inv1.Ics) 
	Config_CMPSS(&Cmpss7Regs);	// ADC D0 pin,(Vdc)
//	Config_CMPSS(&Cmpss8Regs);		// Don't use because SIN and COS signals are not Triped by fault level.
}

void Config_CMPSS(volatile struct CMPSS_REGS *CmpssXRegs){
  	EALLOW;
  	//Enable CMPSS
	CmpssXRegs->COMPCTL.bit.COMPDACE            = 1;			//Comparator/DAC enabled
	//CmpssXRegs->COMPCTL.bit.COMPDACE 			= 0;			//Comparator/DAC enabled
  	//NEG signal comes from DAC
  	//CmpssXRegs->COMPCTL.bit.COMPHSOURCE     	= NEGIN_DAC;  	//default value, Inverting input of comparator driven by internal DAC
  	//CmpssXRegs->COMPCTL.bit.COMPLSOURCE  		= NEGIN_DAC;  	//default value, Inverting input of comparator driven by internal DAC
	CmpssXRegs->COMPCTL.bit.COMPHINV			= NOT_INVERTED;	//comparator output non-invert
	CmpssXRegs->COMPCTL.bit.COMPLINV 		   	= INVERTED;		//comparator output invert

 	//Use VDDA as the reference for DAC
 	//CmpssXRegs->COMPDACCTL.bit.SELREF  	 		= REFERENCE_VDDA;  //default value
	// Configure CTRIPOUT path
	// CmpssXRegs->CTRIPLFILCTL.bit.SAMPWIN 		= 1; //sample window is 1+1 = 2;
	// CmpssXRegs->CTRIPLFILCTL.bit.THRESH 			= 2;
	// CmpssXRegs->CTRIPLFILCLKCTL.bit.CLKPRESCALE 	=1;

	//Asynch|Latched output feeds CTRIPH and CTRIPOUTH
	CmpssXRegs->COMPCTL.bit.CTRIPHSEL           = 0;	//CTRIP_LATCH; Asynchronous comparator output drives CTRIPL //to EPWM X-BAR
	CmpssXRegs->COMPCTL.bit.CTRIPOUTHSEL        = 0;	//CTRIP_LATCH; Asynchronous comparator output drives CTRIPOUTH //to OUTPUT X-BAR
	CmpssXRegs->COMPCTL.bit.CTRIPLSEL           = 0;	//CTRIP_LATCH; Asynchronous comparator output drives CTRIPL //to EPWM X-BAR
	CmpssXRegs->COMPCTL.bit.CTRIPOUTLSEL        = 0;	//CTRIP_LATCH; Asynchronous comparator output drives CTRIPOUTL //to OUTPUT X-BAR
	CmpssXRegs->COMPCTL.bit.ASYNCHEN			= 1;  	// asynch enable :1, Asynchronous comparator output feeds into OR gate with latched digital filter output
	CmpssXRegs->COMPCTL.bit.ASYNCLEN	        = 1;  	// asynch enable :1, Asynchronous comparator output feeds into OR gate with latched digital filter output
    //Set DAC to midpoint for arbitrary reference
	CmpssXRegs->DACHVALS.bit.DACVAL             = 4095; //비교 값 12bit(max 4095)
	CmpssXRegs->DACLVALS.bit.DACVAL             = 0; 	//비교 값
	
	#if 1
	// Use Digital Filter setting
	CmpssXRegs->CTRIPLFILCTL.bit.SAMPWIN		= 0b11111;
	CmpssXRegs->CTRIPLFILCTL.bit.THRESH			= 0b11100;
	CmpssXRegs->CTRIPLFILCTL.bit.FILINIT		= 1;
	CmpssXRegs->CTRIPLFILCLKCTL.bit.CLKPRESCALE	= 2;

	CmpssXRegs->CTRIPHFILCTL.bit.SAMPWIN		= 0b11111;
	CmpssXRegs->CTRIPHFILCTL.bit.THRESH			= 0b11100;
	CmpssXRegs->CTRIPHFILCTL.bit.FILINIT		= 1;
	CmpssXRegs->CTRIPHFILCLKCTL.bit.CLKPRESCALE	= 2;

	CmpssXRegs->COMPSTSCLR.bit.HLATCHCLR		= 1;
	CmpssXRegs->COMPSTSCLR.bit.LLATCHCLR		= 1;
	//Asynch|Latched output feeds CTRIPH and CTRIPOUTH
	CmpssXRegs->COMPCTL.bit.CTRIPHSEL           = 2; 	// CTRIP_LATCH; Asynchronous comparator output drives CTRIPL //to EPWM X-BAR
	CmpssXRegs->COMPCTL.bit.CTRIPOUTHSEL        = 2; 	// CTRIP_LATCH; Asynchronous comparator output drives CTRIPOUTH //to OUTPUT X-BAR
	CmpssXRegs->COMPCTL.bit.CTRIPLSEL           = 2; 	// CTRIP_LATCH; Asynchronous comparator output drives CTRIPL //to EPWM X-BAR
	CmpssXRegs->COMPCTL.bit.CTRIPOUTLSEL        = 2; 	// CTRIP_LATCH; Asynchronous comparator output drives CTRIPOUTL //to OUTPUT X-BAR
	CmpssXRegs->COMPCTL.bit.ASYNCHEN			= 0;  	// asynch enable :1, Asynchronous comparator output feeds into OR gate with latched digital filter output
	CmpssXRegs->COMPCTL.bit.ASYNCLEN	        = 0;  	// asynch enable :1, Asynchronous comparator output feeds into OR gate with latched digital filter output
	// End Digital Filter setting
	#endif
	
    EDIS;
}

