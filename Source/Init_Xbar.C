/**********************************************************************
* File: Xbar.c -- Lab File
* Devices: TMS320F28x7x
* Author: Technical Training Organization (TTO), Texas Instruments
**********************************************************************/

#include "Init_Func_Prototypes.h"						// Main include file


/**********************************************************************
* Function: InitXbar()
*
* Description: Initializes the Input, Output & ePWM X-Bar on the F28x7x
**********************************************************************/
void InitXbar(void)
{
	asm(" EALLOW");								// Enable EALLOW protected register access

//---------------------------------------------------------------------
//--- Input X-Bar
//---------------------------------------------------------------------

//--- Input select registers
	//IPM1FAULTFLAG(GPIO69)
	InputXbarRegs.INPUT1SELECT = 69;  	// Feed GPIO69 to ePWM[TZ1, TRIP1], ePWM X-Bar, Output X-Bar via Input X-Bar
	//IPM2FAULTFLAG(GPIO77)           	
//	InputXbarRegs.INPUT2SELECT = 77;  	// Feed GPIO78 to ePWM[TZ2, TRIP2], ePWM X-Bar, Output X-Bar via Input X-Bar
	//IPM3FAULTFLAG(GPIO147)          	
//	InputXbarRegs.INPUT3SELECT = 147; 	// Feed GPIO147 to ePWM[TZ3, TRIP3], ePWM X-Bar, Output X-Bar via Input X-Bar
	//IPM4FAULTFLAG(GPIO79)           	
//	InputXbarRegs.INPUT4SELECT = 0;	// Feed GPIO79 to XINT1, ePWM X-Bar, Output X-Bar via Input X-Bar
//	InputXbarRegs.INPUT5SELECT = 0; 	// Feed GPIO0 to XINT2, ADCEXTSOC, EXTSYNCIN1, ePWM X-Bar, Output X-Bar via Input X-Bar
//	InputXbarRegs.INPUT6SELECT = 0; 	// Feed GPIO0 to XINT3, ePWM[TRIP6], EXTSYNCIN2, ePWM X-Bar, Output X-Bar via Input X-Bar


//--- INPUTxSELECT lock register control
  	InputXbarRegs.INPUTSELECTLOCK.all = 0x00000000;    // Write a 1 to lock (cannot be cleared once set)


  //---------------------------------------------------------------------
  //--- EPwm X-Bar
  //---------------------------------------------------------------------

  //--- Polarity of trip to EPwm modules (default : 0)
//	EPwmXbarRegs.TRIPOUTINV.bit.TRIP4 		= 0;		// TRIPIN4    0=active high output   1=active low output
	EPwmXbarRegs.TRIPOUTINV.bit.TRIP5 		= 0;		// TRIPIN5    0=active high output   1=active low output
	//EPwmXbarRegs.TRIPOUTINV.bit.TRIP7 	= 0;		// TRIPIN7    0=active high output   1=active low output
	//EPwmXbarRegs.TRIPOUTINV.bit.TRIP8 	= 0;		// TRIPIN8    0=active high output   1=active low output
	//EPwmXbarRegs.TRIPOUTINV.bit.TRIP9 	= 0;		// TRIPIN9    0=active high output   1=active low output
	//EPwmXbarRegs.TRIPOUTINV.bit.TRIP10 	= 0;		// TRIPIN10   0=active high output   1=active low output
	//EPwmXbarRegs.TRIPOUTINV.bit.TRIP11 	= 0;		// TRIPIN11   0=active high output   1=active low output
	//EPwmXbarRegs.TRIPOUTINV.bit.TRIP12 	= 0;		// TRIPIN12   0=active high output   1=active low output

  	//--- EPwm X-Bar Lock register control
  	EPwmXbarRegs.TRIPLOCK.all = 0x00000000;            	// Write a 1 to lock (cannot be cleared once set)

	
  	//--- Group TRIPIN4 (for CMPSS, Gate fault signal)
  	//default : 0
  	//CMPSS1, (Inv1.Ias)
  	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0 = 1;    	// 0=CMPSS1.CTRIPH    1=CMPSS1.CTRIPH_OR_CTRIPL   2=ADCAEVT1   3=ECAP1.OUT
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0 = 1;       // 0=disable trip    1=enable trip
	//IPM1FAULTFLAG(GPIO69)
  	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX1 = 1;     // 0=CMPSS1.CTRIPL   1=INPUTXBAR1 3=ADCCEVT1
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX1 = 1;       // 0=disable trip    1=enable trip
	//CMPSS2, 
// 	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2 = 1;     // 0=CMPSS2.CTRIPH   1=CMPSS2.CTRIPH_OR_CTRIPL   2=ADCAEVT2   3=ECAP2.OUT
// 	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2 = 1;       // 0=disable trip    1=enable trip
	//IPM2FAULTFLAG(GPIO78)                        	
// 	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3 = 1;     // 0=CMPSS2.CTRIPL   1=INPUTXBAR2 3=ADCCEVT2
// 	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX3 = 1;       // 0=disable trip    1=enable trip
	//CMPSS3, (Inv1.Ibs)                           	
  	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX4 = 1;     // 0=CMPSS3.CTRIPH   1=CMPSS3.CTRIPH_OR_CTRIPL   2=ADCAEVT3   3=ECAP3.OUT
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX4 = 1;       // 0=disable trip    1=enable trip
	//IPM3FAULTFLAG(GPIO147)
//  EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX5 = 1; 	// 0=CMPSS3.CTRIPL   1=INPUTXBAR3 3=ADCCEVT3
//  EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX5 = 1;   	// 0=disable trip    1=enable trip
	//CMPSS4, 
// 	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX6 = 1;     // 0=CMPSS4.CTRIPH   1=CMPSS4.CTRIPH_OR_CTRIPL   2=ADCAEVT4   3=ECAP4.OUT
// 	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX6 = 1;       // 0=disable trip    1=enable trip
	//IPM4FAULTFLAG(GPIO79)
//	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX7 = 1;    	// 0=CMPSS4.CTRIPL   1=INPUTXBAR4 3=ADCCEVT4
//	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX7 = 0;     	// 0=disable trip    1=enable trip
	//CMPSS5, (Inv1.Ias)
//  EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX8 = 1;     // 0=CMPSS5.CTRIPH   1=CMPSS5.CTRIPH_OR_CTRIPL   2=ADCBEVT1   3=ECAP5.OUT
//  EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX8 = 1;       // 0=disable trip    1=enable trip
// 	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX9 = 0;    	// 0=CMPSS5.CTRIPL   1=INPUTXBAR5 3=ADCDEVT1
//  EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX9 = 0;      	// 0=disable trip    1=enable trip
	//CMPSS6, (Inv1.Ics)
  	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX10 = 1;    // 0=CMPSS6.CTRIPH   1=CMPSS6.CTRIPH_OR_CTRIPL   2=ADCBEVT2   3=ECAP6.OUT
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX10 = 1;      // 0=disable trip    1=enable trip
//  EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX11 = 0;   	// 0=CMPSS6.CTRIPL   1=INPUTXBAR6 3=ADCDEVT2
//  EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX11 = 0;     	// 0=disable trip    1=enable trip
	//CMPSS7, (Vdc)
  	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX12 = 1;   	// 0=CMPSS7.CTRIPH   1=CMPSS7.CTRIPH_OR_CTRIPL   2=ADCBEVT3
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX12 = 1;     	// 0=disable trip    1=enable trip
//  EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX13 = 0;   	// 0=CMPSS7.CTRIPL   1=ADCSOCA 3=ADCDEVT3
//  EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX13 = 0;     	// 0=disable trip    1=enable trip
	//CMPSS8, (Inv3.Iqse)
//  EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX14 = 1;   	// 0=CMPSS8.CTRIPH   1=CMPSS8.CTRIPH_OR_CTRIPL   2=ADCBEVT4   3=EXTSYNCOUT
//  EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX14 = 1;     	// 0=disable trip    1=enable trip
                                                   	
//  EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX15 = 0;   	// 0=CMPSS8.CTRIPL   1=ADCSOCB 3=ADCDEVT4
//  EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX15 = 0;     	// 0=disable trip    1=enable trip
/*
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX16 = 0;    	// 0=SD1FLT1.COMPH   1=SD1FLT1.COMPH_OR_COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX16 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX17 = 0;    	// 0=SD1FLT1.COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX17 = 0;       	// 0=disable trip    1=enable trip
  	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX18 = 0;    	// 0=SD1FLT2.COMPH   1=SD1FLT2.COMPH_OR_COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX18 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX19 = 0;    	// 0=SD1FLT2.COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX19 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX20 = 0;    	// 0=SD1FLT3.COMPH   1=SD1FLT3.COMPH_OR_COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX20 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX21 = 0;    	// 0=SD1FLT3.COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX21 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX22 = 0;    	// 0=SD1FLT4.COMPH   1=SD1FLT4.COMPH_OR_COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX22 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX23 = 0;    	// 0=SD1FLT4.COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX23 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX24 = 0;    	// 0=SD2FLT1.COMPH   1=SD2FLT1.COMPH_OR_COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX24 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX25 = 0;    	// 0=SD2FLT1.COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX25 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX26 = 0;    	// 0=SD2FLT2.COMPH   1=SD2FLT2.COMPH_OR_COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX26 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX27 = 0;    	// 0=SD2FLT2.COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX27 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX28 = 0;    	// 0=SD2FLT3.COMPH   1=SD2FLT3.COMPH_OR_COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX28 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX29 = 0;    	// 0=SD2FLT3.COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX29 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX30 = 0;    	// 0=SD2FLT4.COMPH   1=SD2FLT4.COMPH_OR_COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX30 = 0;       	// 0=disable trip    1=enable trip
  	                                                 	
  	EPwmXbarRegs.TRIP4MUX16TO31CFG.bit.MUX31 = 0;    	// 0=SD2FLT4.COMPL
  	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX31 = 0;       	// 0=disable trip    1=enable trip
	*/
  //--- Group TRIPIN5 (for ADCEVT fault signal)
#if 1
	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	//(Inv1.Ias) 
  	EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX0 = 2;    	// 0=CMPSS1CTRIPH    1=CMPSS1.CTRIPH_OR_CTRIPL   2=ADCAEVT1   3=ECAP1.OUT
  	EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX0 = 1;      	// 0=disable trip    1=enable trip
	//(Inv1.Ics)                                   	
  	EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX1 = 3;    	// 0=CMPSS1.CTRIPL   1=INPUTXBAR1 3=ADCCEVT1
  	EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX1 = 1;      	// 0=disable trip    1=enable trip
	                                               	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX2 = 2;    	// 0=CMPSS2.CTRIPH   1=CMPSS2.CTRIPH_OR_CTRIPL   2=ADCAEVT2   3=ECAP2.OUT
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX2 = 1;      	// 0=disable trip    1=enable trip
//	                                               	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX3 = 3;    	// 0=CMPSS2.CTRIPL   1=INPUTXBAR2 3=ADCCEVT2
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX3 = 1;      	// 0=disable trip    1=enable trip
//	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX4 = 2;    	// 0=CMPSS3.CTRIPH   1=CMPSS3.CTRIPH_OR_CTRIPL   2=ADCAEVT3   3=ECAP3.OUT
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX4 = 1;      	// 0=disable trip    1=enable trip
//	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX5 = 0;    	// 0=CMPSS3.CTRIPL   1=INPUTXBAR3 3=ADCCEVT3
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX5 = 0;      	// 0=disable trip    1=enable trip
//                                                 	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX6 = 0;    	// 0=CMPSS4.CTRIPH   1=CMPSS4.CTRIPH_OR_CTRIPL   2=ADCAEVT4   3=ECAP4.OUT
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX6 = 0;      	// 0=disable trip    1=enable trip
//                                                 	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX7 = 0;    	// 0=CMPSS4.CTRIPL   1=INPUTXBAR4 3=ADCCEVT4
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX7 = 0;      	// 0=disable trip    1=enable trip
	//(Inv1.Ibs)
  	EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX8 = 2;     // 0=CMPSS5.CTRIPH   1=CMPSS5.CTRIPH_OR_CTRIPL   2=ADCBEVT1   3=ECAP5.OUT
  	EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX8 = 1;       // 0=disable trip    1=enable trip
	//(Vdc)
  	EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX9 = 3;     // 0=CMPSS5.CTRIPL   1=INPUTXBAR5 3=ADCDEVT1
  	EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX9 = 1;       // 0=disable trip    1=enable trip
	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX10 = 2;    // 0=CMPSS6.CTRIPH   1=CMPSS6.CTRIPH_OR_CTRIPL   2=ADCBEVT2   3=ECAP6.OUT
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX10 = 1;      // 0=disable trip    1=enable trip
	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX11 = 3;   	// 0=CMPSS6.CTRIPL   1=INPUTXBAR6 3=ADCDEVT2
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX11 = 1;     	// 0=disable trip    1=enable trip
//	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX12 = 2;   	// 0=CMPSS7.CTRIPH   1=CMPSS7.CTRIPH_OR_CTRIPL   2=ADCBEVT3
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX12 = 1;     	// 0=disable trip    1=enable trip
//	
//  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX13 = 3;   	// 0=CMPSS7.CTRIPL   1=ADCSOCA 3=ADCDEVT3
//  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX13 = 1;     	// 0=disable trip    1=enable trip
/*
  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX14 = 0;     	// 0=CMPSS8.CTRIPH   1=CMPSS8.CTRIPH_OR_CTRIPL   2=ADCBEVT4   3=EXTSYNCOUT
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX14 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX0TO15CFG.bit.MUX15 = 0;     	// 0=CMPSS8.CTRIPL   1=ADCSOCB 3=ADCDEVT4
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX15 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX16 = 0;    	// 0=SD1FLT1.COMPH   1=SD1FLT1.COMPH_OR_COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX16 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX17 = 0;    	// 0=SD1FLT1.COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX17 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX18 = 0;    	// 0=SD1FLT2.COMPH   1=SD1FLT2.COMPH_OR_COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX18 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX19 = 0;    	// 0=SD1FLT2.COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX19 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX20 = 0;    	// 0=SD1FLT3.COMPH   1=SD1FLT3.COMPH_OR_COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX20 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX21 = 0;    	// 0=SD1FLT3.COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX21 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX22 = 0;    	// 0=SD1FLT4.COMPH   1=SD1FLT4.COMPH_OR_COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX22 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX23 = 0;    	// 0=SD1FLT4.COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX23 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX24 = 0;    	// 0=SD2FLT1.COMPH   1=SD2FLT1.COMPH_OR_COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX24 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX25 = 0;    	// 0=SD2FLT1.COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX25 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX26 = 0;    	// 0=SD2FLT2.COMPH   1=SD2FLT2.COMPH_OR_COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX26 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX27 = 0;    	// 0=SD2FLT2.COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX27 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX28 = 0;    	// 0=SD2FLT3.COMPH   1=SD2FLT3.COMPH_OR_COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX28 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX29 = 0;    	// 0=SD2FLT3.COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX29 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX30 = 0;    	// 0=SD2FLT4.COMPH   1=SD2FLT4.COMPH_OR_COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX30 = 0;       	// 0=disable trip    1=enable trip
                                                   	
  EPwmXbarRegs.TRIP5MUX16TO31CFG.bit.MUX31 = 0;    	// 0=SD2FLT4.COMPL
  EPwmXbarRegs.TRIP5MUXENABLE.bit.MUX31 = 0;       	// 0=disable trip    1=enable trip
*/
#endif

//---------------------------------------------------------------------
//--- Output X-Bar
//---------------------------------------------------------------------

//--- Polarity of output  (default : 0)
	//OutputXbarRegs.OUTPUTINV.bit.OUTPUT1 = 0;           // OUTPUT1    0=active high output   1=active low output (default : 0)
	//OutputXbarRegs.OUTPUTINV.bit.OUTPUT2 = 0;           // OUTPUT2    0=active high output   1=active low output (default : 0)
	//OutputXbarRegs.OUTPUTINV.bit.OUTPUT3 = 0;           // OUTPUT3    0=active high output   1=active low output (default : 0)
	//OutputXbarRegs.OUTPUTINV.bit.OUTPUT4 = 0;           // OUTPUT4    0=active high output   1=active low output (default : 0)
	//OutputXbarRegs.OUTPUTINV.bit.OUTPUT5 = 0;           // OUTPUT5    0=active high output   1=active low output (default : 0)
	//OutputXbarRegs.OUTPUTINV.bit.OUTPUT6 = 0;           // OUTPUT6    0=active high output   1=active low output (default : 0)
	//OutputXbarRegs.OUTPUTINV.bit.OUTPUT7 = 0;           // OUTPUT7    0=active high output   1=active low output (default : 0)
	//OutputXbarRegs.OUTPUTINV.bit.OUTPUT8 = 0;           // OUTPUT8    0=active high output   1=active low output (default : 0)
/*
//--- Output X-Bar Lock register control
  OutputXbarRegs.OUTPUTLOCK.all = 0x00000000;         // Write a 1 to lock (cannot be cleared once set)

//--- Output latch registers
  OutputXbarRegs.OUTPUTLATCHENABLE.all = 0x00000000;  // Latch enable  -  0=latch not selected   1=latch selected
	OutputXbarRegs.OUTPUTLATCH.all = 0x00000000;        // Latch status  -  0=not triggered   1=triggered
  OutputXbarRegs.OUTPUTLATCHCLR.all = 0x00000000;     // Write 1 to clear bit
	OutputXbarRegs.OUTPUTLATCHFRC.all = 0x00000000;     // Write 1 to set bit

	//OutputXbarRegs.OUTPUTLATCHENABLE.bit.OUTPUT1=1;
	//OutputXbarRegs.OUTPUTLATCHCLR.bit.OUTPUT1 = 1;
//--- Group OUTPUT1

	OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX0 = 0;        // 0=CMPSS1CoutputH    1=CMPSS1.CoutputH_OR_CoutputL   2=ADCAEVT1   3=ECAP1.OUT
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX0 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX1 = 1;        // 0=CMPSS1.CoutputL   1=INPUTXBAR1 3=ADCCEVT1
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX1 = 1;          // 0=disable output    1=enable output

	OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX2 = 0;        // 0=CMPSS2.CoutputH   1=CMPSS2.CoutputH_OR_CoutputL   2=ADCAEVT2   3=ECAP2.OUT
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX2 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX3 = 0;        // 0=CMPSS2.CoutputL   1=INPUTXBAR2 3=ADCCEVT2
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX3 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX4 = 0;        // 0=CMPSS3.CoutputH   1=CMPSS3.CoutputH_OR_CoutputL   2=ADCAEVT3   3=ECAP3.OUT
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX4 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX5 = 0;        // 0=CMPSS3.CoutputL   1=INPUTXBAR3 3=ADCCEVT3
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX5 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX6 = 0;        // 0=CMPSS4.CoutputH   1=CMPSS4.CoutputH_OR_CoutputL   2=ADCAEVT4   3=ECAP4.OUT
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX6 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX7 = 0;        // 0=CMPSS4.CoutputL   1=INPUTXBAR4 3=ADCCEVT4
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX7 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX8 = 0;        // 0=CMPSS5.CoutputH   1=CMPSS5.CoutputH_OR_CoutputL   2=ADCBEVT1   3=ECAP5.OUT
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX8 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX9 = 0;        // 0=CMPSS5.CoutputL   1=INPUTXBAR5 3=ADCDEVT1
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX9 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX10 = 0;       // 0=CMPSS6.CoutputH   1=CMPSS6.CoutputH_OR_CoutputL   2=ADCBEVT2   3=ECAP6.OUT
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX10 = 0;         // 0=disable output    1=enable output
*/
  /*
  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX11 = 0;       // 0=CMPSS6.CoutputL   1=INPUTXBAR6 3=ADCDEVT2
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX11 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX12 = 0;       // 0=CMPSS7.CoutputH   1=CMPSS7.CoutputH_OR_CoutputL   2=ADCBEVT3
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX12 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX13 = 0;       // 0=CMPSS7.CoutputL   1=ADCSOCA 3=ADCDEVT3
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX13 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX14 = 0;       // 0=CMPSS8.CoutputH   1=CMPSS8.CoutputH_OR_CoutputL   2=ADCBEVT4   3=EXTSYNCOUT
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX14 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX15 = 0;       // 0=CMPSS8.CoutputL   1=ADCSOCB 3=ADCDEVT4
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX15 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX16 = 0;      // 0=SD1FLT1.COMPH     1=SD1FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX16 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX17 = 0;      // 0=SD1FLT1.COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX17 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX18 = 0;      // 0=SD1FLT2.COMPH     1=SD1FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX18 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX19 = 0;      // 0=SD1FLT2.COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX19 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX20 = 0;      // 0=SD1FLT3.COMPH     1=SD1FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX20 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX21 = 0;      // 0=SD1FLT3.COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX21 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX22 = 0;      // 0=SD1FLT4.COMPH     1=SD1FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX22 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX23 = 0;      // 0=SD1FLT4.COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX23 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX24 = 0;      // 0=SD2FLT1.COMPH     1=SD2FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX24 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX25 = 0;      // 0=SD2FLT1.COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX25 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX26 = 0;      // 0=SD2FLT2.COMPH     1=SD2FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX26 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX27 = 0;      // 0=SD2FLT2.COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX27 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX28 = 0;      // 0=SD2FLT3.COMPH     1=SD2FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX28 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX29 = 0;      // 0=SD2FLT3.COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX29 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX30 = 0;      // 0=SD2FLT4.COMPH     1=SD2FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX30 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT1MUX16TO31CFG.bit.MUX31 = 0;      // 0=SD2FLT4.COMPL
  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX31 = 0;         // 0=disable output    1=enable output

//---Group OUTPUT2
  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX0 = 0;        // 0=CMPSS1CoutputH    1=CMPSS1.CoutputH_OR_CoutputL   2=ADCAEVT1   3=ECAP1.OUT
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX0 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX1 = 0;        // 0=CMPSS1.CoutputL   1=INPUTXBAR1 3=ADCCEVT1
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX1 = 0;          // 0=disable output    1=enable output

	OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX2 = 0;        // 0=CMPSS2.CoutputH   1=CMPSS2.CoutputH_OR_CoutputL   2=ADCAEVT2   3=ECAP2.OUT
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX2 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX3 = 0;        // 0=CMPSS2.CoutputL   1=INPUTXBAR2 3=ADCCEVT2
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX3 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX4 = 0;        // 0=CMPSS3.CoutputH   1=CMPSS3.CoutputH_OR_CoutputL   2=ADCAEVT3   3=ECAP3.OUT
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX4 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX5 = 0;        // 0=CMPSS3.CoutputL   1=INPUTXBAR3 3=ADCCEVT3
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX5 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX6 = 0;        // 0=CMPSS4.CoutputH   1=CMPSS4.CoutputH_OR_CoutputL   2=ADCAEVT4   3=ECAP4.OUT
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX6 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX7 = 0;        // 0=CMPSS4.CoutputL   1=INPUTXBAR4 3=ADCCEVT4
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX7 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX8 = 0;        // 0=CMPSS5.CoutputH   1=CMPSS5.CoutputH_OR_CoutputL   2=ADCBEVT1   3=ECAP5.OUT
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX8 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX9 = 0;        // 0=CMPSS5.CoutputL   1=INPUTXBAR5 3=ADCDEVT1
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX9 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX10 = 0;       // 0=CMPSS6.CoutputH   1=CMPSS6.CoutputH_OR_CoutputL   2=ADCBEVT2   3=ECAP6.OUT
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX10 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX11 = 0;       // 0=CMPSS6.CoutputL   1=INPUTXBAR6 3=ADCDEVT2
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX11 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX12 = 0;       // 0=CMPSS7.CoutputH   1=CMPSS7.CoutputH_OR_CoutputL   2=ADCBEVT3
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX12 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX13 = 0;       // 0=CMPSS7.CoutputL   1=ADCSOCA 3=ADCDEVT3
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX13 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX14 = 0;       // 0=CMPSS8.CoutputH   1=CMPSS8.CoutputH_OR_CoutputL   2=ADCBEVT4   3=EXTSYNCOUT
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX14 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX15 = 0;       // 0=CMPSS8.CoutputL   1=ADCSOCB 3=ADCDEVT4
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX15 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX16 = 0;      // 0=SD1FLT1.COMPH     1=SD1FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX16 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX17 = 0;      // 0=SD1FLT1.COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX17 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX18 = 0;      // 0=SD1FLT2.COMPH     1=SD1FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX18 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX19 = 0;      // 0=SD1FLT2.COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX19 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX20 = 0;      // 0=SD1FLT3.COMPH     1=SD1FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX20 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX21 = 0;      // 0=SD1FLT3.COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX21 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX22 = 0;      // 0=SD1FLT4.COMPH     1=SD1FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX22 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX23 = 0;      // 0=SD1FLT4.COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX23 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX24 = 0;      // 0=SD2FLT1.COMPH     1=SD2FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX24 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX25 = 0;      // 0=SD2FLT1.COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX25 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX26 = 0;      // 0=SD2FLT2.COMPH     1=SD2FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX26 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX27 = 0;      // 0=SD2FLT2.COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX27 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX28 = 0;      // 0=SD2FLT3.COMPH     1=SD2FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX28 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX29 = 0;      // 0=SD2FLT3.COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX29 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX30 = 0;      // 0=SD2FLT4.COMPH     1=SD2FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX30 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT2MUX16TO31CFG.bit.MUX31 = 0;      // 0=SD2FLT4.COMPL
  OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX31 = 0;         // 0=disable output    1=enable output

//---Group OUTPUT3
  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX0 = 0;        // 0=CMPSS1CoutputH    1=CMPSS1.CoutputH_OR_CoutputL   2=ADCAEVT1   3=ECAP1.OUT
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX0 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX1 = 0;        // 0=CMPSS1.CoutputL   1=INPUTXBAR1 3=ADCCEVT1
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX1 = 0;          // 0=disable output    1=enable output

	OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX2 = 0;        // 0=CMPSS2.CoutputH   1=CMPSS2.CoutputH_OR_CoutputL   2=ADCAEVT2   3=ECAP2.OUT
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX2 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX3 = 0;        // 0=CMPSS2.CoutputL   1=INPUTXBAR2 3=ADCCEVT2
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX3 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX4 = 0;        // 0=CMPSS3.CoutputH   1=CMPSS3.CoutputH_OR_CoutputL   2=ADCAEVT3   3=ECAP3.OUT
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX4 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX5 = 0;        // 0=CMPSS3.CoutputL   1=INPUTXBAR3 3=ADCCEVT3
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX5 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX6 = 0;        // 0=CMPSS4.CoutputH   1=CMPSS4.CoutputH_OR_CoutputL   2=ADCAEVT4   3=ECAP4.OUT
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX6 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX7 = 0;        // 0=CMPSS4.CoutputL   1=INPUTXBAR4 3=ADCCEVT4
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX7 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX8 = 0;        // 0=CMPSS5.CoutputH   1=CMPSS5.CoutputH_OR_CoutputL   2=ADCBEVT1   3=ECAP5.OUT
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX8 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX9 = 0;        // 0=CMPSS5.CoutputL   1=INPUTXBAR5 3=ADCDEVT1
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX9 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX10 = 0;       // 0=CMPSS6.CoutputH   1=CMPSS6.CoutputH_OR_CoutputL   2=ADCBEVT2   3=ECAP6.OUT
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX10 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX11 = 0;       // 0=CMPSS6.CoutputL   1=INPUTXBAR6 3=ADCDEVT2
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX11 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX12 = 0;       // 0=CMPSS7.CoutputH   1=CMPSS7.CoutputH_OR_CoutputL   2=ADCBEVT3
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX12 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX13 = 0;       // 0=CMPSS7.CoutputL   1=ADCSOCA 3=ADCDEVT3
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX13 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX14 = 0;       // 0=CMPSS8.CoutputH   1=CMPSS8.CoutputH_OR_CoutputL   2=ADCBEVT4   3=EXTSYNCOUT
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX14 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX15 = 0;       // 0=CMPSS8.CoutputL   1=ADCSOCB 3=ADCDEVT4
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX15 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX16 = 0;      // 0=SD1FLT1.COMPH     1=SD1FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX16 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX17 = 0;      // 0=SD1FLT1.COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX17 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX18 = 0;      // 0=SD1FLT2.COMPH     1=SD1FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX18 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX19 = 0;      // 0=SD1FLT2.COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX19 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX20 = 0;      // 0=SD1FLT3.COMPH     1=SD1FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX20 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX21 = 0;      // 0=SD1FLT3.COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX21 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX22 = 0;      // 0=SD1FLT4.COMPH     1=SD1FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX22 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX23 = 0;      // 0=SD1FLT4.COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX23 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX24 = 0;      // 0=SD2FLT1.COMPH     1=SD2FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX24 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX25 = 0;      // 0=SD2FLT1.COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX25 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX26 = 0;      // 0=SD2FLT2.COMPH     1=SD2FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX26 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX27 = 0;      // 0=SD2FLT2.COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX27 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX28 = 0;      // 0=SD2FLT3.COMPH     1=SD2FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX28 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX29 = 0;      // 0=SD2FLT3.COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX29 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX30 = 0;      // 0=SD2FLT4.COMPH     1=SD2FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX30 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT3MUX16TO31CFG.bit.MUX31 = 0;      // 0=SD2FLT4.COMPL
  OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX31 = 0;         // 0=disable output    1=enable output

//---Group OUTPUT4
  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX0 = 0;        // 0=CMPSS1CoutputH    1=CMPSS1.CoutputH_OR_CoutputL   2=ADCAEVT1   3=ECAP1.OUT
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX0 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX1 = 0;        // 0=CMPSS1.CoutputL   1=INPUTXBAR1 3=ADCCEVT1
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX1 = 0;          // 0=disable output    1=enable output

	OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX2 = 0;        // 0=CMPSS2.CoutputH   1=CMPSS2.CoutputH_OR_CoutputL   2=ADCAEVT2   3=ECAP2.OUT
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX2 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX3 = 0;        // 0=CMPSS2.CoutputL   1=INPUTXBAR2 3=ADCCEVT2
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX3 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX4 = 0;        // 0=CMPSS3.CoutputH   1=CMPSS3.CoutputH_OR_CoutputL   2=ADCAEVT3   3=ECAP3.OUT
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX4 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX5 = 0;        // 0=CMPSS3.CoutputL   1=INPUTXBAR3 3=ADCCEVT3
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX5 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX6 = 0;        // 0=CMPSS4.CoutputH   1=CMPSS4.CoutputH_OR_CoutputL   2=ADCAEVT4   3=ECAP4.OUT
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX6 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX7 = 0;        // 0=CMPSS4.CoutputL   1=INPUTXBAR4 3=ADCCEVT4
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX7 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX8 = 0;        // 0=CMPSS5.CoutputH   1=CMPSS5.CoutputH_OR_CoutputL   2=ADCBEVT1   3=ECAP5.OUT
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX8 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX9 = 0;        // 0=CMPSS5.CoutputL   1=INPUTXBAR5 3=ADCDEVT1
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX9 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX10 = 0;       // 0=CMPSS6.CoutputH   1=CMPSS6.CoutputH_OR_CoutputL   2=ADCBEVT2   3=ECAP6.OUT
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX10 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX11 = 0;       // 0=CMPSS6.CoutputL   1=INPUTXBAR6 3=ADCDEVT2
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX11 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX12 = 0;       // 0=CMPSS7.CoutputH   1=CMPSS7.CoutputH_OR_CoutputL   2=ADCBEVT3
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX12 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX13 = 0;       // 0=CMPSS7.CoutputL   1=ADCSOCA 3=ADCDEVT3
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX13 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX14 = 0;       // 0=CMPSS8.CoutputH   1=CMPSS8.CoutputH_OR_CoutputL   2=ADCBEVT4   3=EXTSYNCOUT
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX14 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX15 = 0;       // 0=CMPSS8.CoutputL   1=ADCSOCB 3=ADCDEVT4
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX15 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX16 = 0;      // 0=SD1FLT1.COMPH     1=SD1FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX16 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX17 = 0;      // 0=SD1FLT1.COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX17 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX18 = 0;      // 0=SD1FLT2.COMPH     1=SD1FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX18 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX19 = 0;      // 0=SD1FLT2.COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX19 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX20 = 0;      // 0=SD1FLT3.COMPH     1=SD1FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX20 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX21 = 0;      // 0=SD1FLT3.COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX21 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX22 = 0;      // 0=SD1FLT4.COMPH     1=SD1FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX22 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX23 = 0;      // 0=SD1FLT4.COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX23 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX24 = 0;      // 0=SD2FLT1.COMPH     1=SD2FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX24 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX25 = 0;      // 0=SD2FLT1.COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX25 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX26 = 0;      // 0=SD2FLT2.COMPH     1=SD2FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX26 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX27 = 0;      // 0=SD2FLT2.COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX27 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX28 = 0;      // 0=SD2FLT3.COMPH     1=SD2FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX28 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX29 = 0;      // 0=SD2FLT3.COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX29 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX30 = 0;      // 0=SD2FLT4.COMPH     1=SD2FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX30 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT4MUX16TO31CFG.bit.MUX31 = 0;      // 0=SD2FLT4.COMPL
  OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX31 = 0;         // 0=disable output    1=enable output

//---Group OUTPUT5
  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX0 = 0;        // 0=CMPSS1CoutputH    1=CMPSS1.CoutputH_OR_CoutputL   2=ADCAEVT1   3=ECAP1.OUT
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX0 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX1 = 0;        // 0=CMPSS1.CoutputL   1=INPUTXBAR1 3=ADCCEVT1
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX1 = 0;          // 0=disable output    1=enable output

	OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX2 = 0;        // 0=CMPSS2.CoutputH   1=CMPSS2.CoutputH_OR_CoutputL   2=ADCAEVT2   3=ECAP2.OUT
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX2 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX3 = 0;        // 0=CMPSS2.CoutputL   1=INPUTXBAR2 3=ADCCEVT2
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX3 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX4 = 0;        // 0=CMPSS3.CoutputH   1=CMPSS3.CoutputH_OR_CoutputL   2=ADCAEVT3   3=ECAP3.OUT
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX4 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX5 = 0;        // 0=CMPSS3.CoutputL   1=INPUTXBAR3 3=ADCCEVT3
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX5 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX6 = 0;        // 0=CMPSS4.CoutputH   1=CMPSS4.CoutputH_OR_CoutputL   2=ADCAEVT4   3=ECAP4.OUT
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX6 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX7 = 0;        // 0=CMPSS4.CoutputL   1=INPUTXBAR4 3=ADCCEVT4
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX7 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX8 = 0;        // 0=CMPSS5.CoutputH   1=CMPSS5.CoutputH_OR_CoutputL   2=ADCBEVT1   3=ECAP5.OUT
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX8 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX9 = 0;        // 0=CMPSS5.CoutputL   1=INPUTXBAR5 3=ADCDEVT1
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX9 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX10 = 0;       // 0=CMPSS6.CoutputH   1=CMPSS6.CoutputH_OR_CoutputL   2=ADCBEVT2   3=ECAP6.OUT
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX10 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX11 = 0;       // 0=CMPSS6.CoutputL   1=INPUTXBAR6 3=ADCDEVT2
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX11 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX12 = 0;       // 0=CMPSS7.CoutputH   1=CMPSS7.CoutputH_OR_CoutputL   2=ADCBEVT3
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX12 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX13 = 0;       // 0=CMPSS7.CoutputL   1=ADCSOCA 3=ADCDEVT3
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX13 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX14 = 0;       // 0=CMPSS8.CoutputH   1=CMPSS8.CoutputH_OR_CoutputL   2=ADCBEVT4   3=EXTSYNCOUT
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX14 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX15 = 0;       // 0=CMPSS8.CoutputL   1=ADCSOCB 3=ADCDEVT4
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX15 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX16 = 0;      // 0=SD1FLT1.COMPH     1=SD1FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX16 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX17 = 0;      // 0=SD1FLT1.COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX17 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX18 = 0;      // 0=SD1FLT2.COMPH     1=SD1FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX18 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX19 = 0;      // 0=SD1FLT2.COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX19 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX20 = 0;      // 0=SD1FLT3.COMPH     1=SD1FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX20 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX21 = 0;      // 0=SD1FLT3.COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX21 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX22 = 0;      // 0=SD1FLT4.COMPH     1=SD1FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX22 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX23 = 0;      // 0=SD1FLT4.COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX23 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX24 = 0;      // 0=SD2FLT1.COMPH     1=SD2FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX24 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX25 = 0;      // 0=SD2FLT1.COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX25 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX26 = 0;      // 0=SD2FLT2.COMPH     1=SD2FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX26 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX27 = 0;      // 0=SD2FLT2.COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX27 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX28 = 0;      // 0=SD2FLT3.COMPH     1=SD2FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX28 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX29 = 0;      // 0=SD2FLT3.COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX29 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX30 = 0;      // 0=SD2FLT4.COMPH     1=SD2FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX30 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT5MUX16TO31CFG.bit.MUX31 = 0;      // 0=SD2FLT4.COMPL
  OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX31 = 0;         // 0=disable output    1=enable output

//---Group OUTPUT6
  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX0 = 0;        // 0=CMPSS1CoutputH    1=CMPSS1.CoutputH_OR_CoutputL   2=ADCAEVT1   3=ECAP1.OUT
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX0 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX1 = 0;        // 0=CMPSS1.CoutputL   1=INPUTXBAR1 3=ADCCEVT1
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX1 = 0;          // 0=disable output    1=enable output

	OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX2 = 0;        // 0=CMPSS2.CoutputH   1=CMPSS2.CoutputH_OR_CoutputL   2=ADCAEVT2   3=ECAP2.OUT
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX2 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX3 = 0;        // 0=CMPSS2.CoutputL   1=INPUTXBAR2 3=ADCCEVT2
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX3 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX4 = 0;        // 0=CMPSS3.CoutputH   1=CMPSS3.CoutputH_OR_CoutputL   2=ADCAEVT3   3=ECAP3.OUT
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX4 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX5 = 0;        // 0=CMPSS3.CoutputL   1=INPUTXBAR3 3=ADCCEVT3
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX5 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX6 = 0;        // 0=CMPSS4.CoutputH   1=CMPSS4.CoutputH_OR_CoutputL   2=ADCAEVT4   3=ECAP4.OUT
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX6 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX7 = 0;        // 0=CMPSS4.CoutputL   1=INPUTXBAR4 3=ADCCEVT4
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX7 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX8 = 0;        // 0=CMPSS5.CoutputH   1=CMPSS5.CoutputH_OR_CoutputL   2=ADCBEVT1   3=ECAP5.OUT
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX8 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX9 = 0;        // 0=CMPSS5.CoutputL   1=INPUTXBAR5 3=ADCDEVT1
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX9 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX10 = 0;       // 0=CMPSS6.CoutputH   1=CMPSS6.CoutputH_OR_CoutputL   2=ADCBEVT2   3=ECAP6.OUT
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX10 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX11 = 0;       // 0=CMPSS6.CoutputL   1=INPUTXBAR6 3=ADCDEVT2
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX11 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX12 = 0;       // 0=CMPSS7.CoutputH   1=CMPSS7.CoutputH_OR_CoutputL   2=ADCBEVT3
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX12 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX13 = 0;       // 0=CMPSS7.CoutputL   1=ADCSOCA 3=ADCDEVT3
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX13 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX14 = 0;       // 0=CMPSS8.CoutputH   1=CMPSS8.CoutputH_OR_CoutputL   2=ADCBEVT4   3=EXTSYNCOUT
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX14 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX15 = 0;       // 0=CMPSS8.CoutputL   1=ADCSOCB 3=ADCDEVT4
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX15 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX16 = 0;      // 0=SD1FLT1.COMPH     1=SD1FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX16 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX17 = 0;      // 0=SD1FLT1.COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX17 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX18 = 0;      // 0=SD1FLT2.COMPH     1=SD1FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX18 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX19 = 0;      // 0=SD1FLT2.COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX19 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX20 = 0;      // 0=SD1FLT3.COMPH     1=SD1FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX20 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX21 = 0;      // 0=SD1FLT3.COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX21 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX22 = 0;      // 0=SD1FLT4.COMPH     1=SD1FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX22 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX23 = 0;      // 0=SD1FLT4.COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX23 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX24 = 0;      // 0=SD2FLT1.COMPH     1=SD2FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX24 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX25 = 0;      // 0=SD2FLT1.COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX25 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX26 = 0;      // 0=SD2FLT2.COMPH     1=SD2FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX26 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX27 = 0;      // 0=SD2FLT2.COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX27 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX28 = 0;      // 0=SD2FLT3.COMPH     1=SD2FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX28 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX29 = 0;      // 0=SD2FLT3.COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX29 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX30 = 0;      // 0=SD2FLT4.COMPH     1=SD2FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX30 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT6MUX16TO31CFG.bit.MUX31 = 0;      // 0=SD2FLT4.COMPL
  OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX31 = 0;         // 0=disable output    1=enable output

//---Group OUTPUT7
  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX0 = 0;        // 0=CMPSS1CoutputH    1=CMPSS1.CoutputH_OR_CoutputL   2=ADCAEVT1   3=ECAP1.OUT
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX0 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX1 = 0;        // 0=CMPSS1.CoutputL   1=INPUTXBAR1 3=ADCCEVT1
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX1 = 0;          // 0=disable output    1=enable output

	OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX2 = 0;        // 0=CMPSS2.CoutputH   1=CMPSS2.CoutputH_OR_CoutputL   2=ADCAEVT2   3=ECAP2.OUT
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX2 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX3 = 0;        // 0=CMPSS2.CoutputL   1=INPUTXBAR2 3=ADCCEVT2
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX3 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX4 = 0;        // 0=CMPSS3.CoutputH   1=CMPSS3.CoutputH_OR_CoutputL   2=ADCAEVT3   3=ECAP3.OUT
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX4 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX5 = 0;        // 0=CMPSS3.CoutputL   1=INPUTXBAR3 3=ADCCEVT3
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX5 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX6 = 0;        // 0=CMPSS4.CoutputH   1=CMPSS4.CoutputH_OR_CoutputL   2=ADCAEVT4   3=ECAP4.OUT
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX6 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX7 = 0;        // 0=CMPSS4.CoutputL   1=INPUTXBAR4 3=ADCCEVT4
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX7 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX8 = 0;        // 0=CMPSS5.CoutputH   1=CMPSS5.CoutputH_OR_CoutputL   2=ADCBEVT1   3=ECAP5.OUT
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX8 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX9 = 0;        // 0=CMPSS5.CoutputL   1=INPUTXBAR5 3=ADCDEVT1
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX9 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX10 = 0;       // 0=CMPSS6.CoutputH   1=CMPSS6.CoutputH_OR_CoutputL   2=ADCBEVT2   3=ECAP6.OUT
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX10 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX11 = 0;       // 0=CMPSS6.CoutputL   1=INPUTXBAR6 3=ADCDEVT2
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX11 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX12 = 0;       // 0=CMPSS7.CoutputH   1=CMPSS7.CoutputH_OR_CoutputL   2=ADCBEVT3
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX12 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX13 = 0;       // 0=CMPSS7.CoutputL   1=ADCSOCA 3=ADCDEVT3
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX13 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX14 = 0;       // 0=CMPSS8.CoutputH   1=CMPSS8.CoutputH_OR_CoutputL   2=ADCBEVT4   3=EXTSYNCOUT
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX14 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX15 = 0;       // 0=CMPSS8.CoutputL   1=ADCSOCB 3=ADCDEVT4
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX15 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX16 = 0;      // 0=SD1FLT1.COMPH     1=SD1FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX16 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX17 = 0;      // 0=SD1FLT1.COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX17 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX18 = 0;      // 0=SD1FLT2.COMPH     1=SD1FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX18 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX19 = 0;      // 0=SD1FLT2.COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX19 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX20 = 0;      // 0=SD1FLT3.COMPH     1=SD1FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX20 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX21 = 0;      // 0=SD1FLT3.COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX21 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX22 = 0;      // 0=SD1FLT4.COMPH     1=SD1FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX22 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX23 = 0;      // 0=SD1FLT4.COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX23 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX24 = 0;      // 0=SD2FLT1.COMPH     1=SD2FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX24 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX25 = 0;      // 0=SD2FLT1.COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX25 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX26 = 0;      // 0=SD2FLT2.COMPH     1=SD2FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX26 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX27 = 0;      // 0=SD2FLT2.COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX27 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX28 = 0;      // 0=SD2FLT3.COMPH     1=SD2FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX28 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX29 = 0;      // 0=SD2FLT3.COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX29 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX30 = 0;      // 0=SD2FLT4.COMPH     1=SD2FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX30 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT7MUX16TO31CFG.bit.MUX31 = 0;      // 0=SD2FLT4.COMPL
  OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX31 = 0;         // 0=disable output    1=enable output

//---Group OUTPUT8
  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX0 = 0;        // 0=CMPSS1CoutputH    1=CMPSS1.CoutputH_OR_CoutputL   2=ADCAEVT1   3=ECAP1.OUT
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX0 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX1 = 0;        // 0=CMPSS1.CoutputL   1=INPUTXBAR1 3=ADCCEVT1
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX1 = 0;          // 0=disable output    1=enable output

	OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX2 = 0;        // 0=CMPSS2.CoutputH   1=CMPSS2.CoutputH_OR_CoutputL   2=ADCAEVT2   3=ECAP2.OUT
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX2 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX3 = 0;        // 0=CMPSS2.CoutputL   1=INPUTXBAR2 3=ADCCEVT2
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX3 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX4 = 0;        // 0=CMPSS3.CoutputH   1=CMPSS3.CoutputH_OR_CoutputL   2=ADCAEVT3   3=ECAP3.OUT
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX4 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX5 = 0;        // 0=CMPSS3.CoutputL   1=INPUTXBAR3 3=ADCCEVT3
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX5 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX6 = 0;        // 0=CMPSS4.CoutputH   1=CMPSS4.CoutputH_OR_CoutputL   2=ADCAEVT4   3=ECAP4.OUT
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX6 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX7 = 0;        // 0=CMPSS4.CoutputL   1=INPUTXBAR4 3=ADCCEVT4
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX7 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX8 = 0;        // 0=CMPSS5.CoutputH   1=CMPSS5.CoutputH_OR_CoutputL   2=ADCBEVT1   3=ECAP5.OUT
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX8 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX9 = 0;        // 0=CMPSS5.CoutputL   1=INPUTXBAR5 3=ADCDEVT1
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX9 = 0;          // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX10 = 0;       // 0=CMPSS6.CoutputH   1=CMPSS6.CoutputH_OR_CoutputL   2=ADCBEVT2   3=ECAP6.OUT
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX10 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX11 = 0;       // 0=CMPSS6.CoutputL   1=INPUTXBAR6 3=ADCDEVT2
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX11 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX12 = 0;       // 0=CMPSS7.CoutputH   1=CMPSS7.CoutputH_OR_CoutputL   2=ADCBEVT3
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX12 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX13 = 0;       // 0=CMPSS7.CoutputL   1=ADCSOCA 3=ADCDEVT3
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX13 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX14 = 0;       // 0=CMPSS8.CoutputH   1=CMPSS8.CoutputH_OR_CoutputL   2=ADCBEVT4   3=EXTSYNCOUT
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX14 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX15 = 0;       // 0=CMPSS8.CoutputL   1=ADCSOCB 3=ADCDEVT4
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX15 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX16 = 0;      // 0=SD1FLT1.COMPH     1=SD1FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX16 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX17 = 0;      // 0=SD1FLT1.COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX17 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX18 = 0;      // 0=SD1FLT2.COMPH     1=SD1FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX18 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX19 = 0;      // 0=SD1FLT2.COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX19 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX20 = 0;      // 0=SD1FLT3.COMPH     1=SD1FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX20 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX21 = 0;      // 0=SD1FLT3.COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX21 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX22 = 0;      // 0=SD1FLT4.COMPH     1=SD1FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX22 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX23 = 0;      // 0=SD1FLT4.COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX23 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX24 = 0;      // 0=SD2FLT1.COMPH     1=SD2FLT1.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX24 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX25 = 0;      // 0=SD2FLT1.COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX25 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX26 = 0;      // 0=SD2FLT2.COMPH     1=SD2FLT2.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX26 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX27 = 0;      // 0=SD2FLT2.COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX27 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX28 = 0;      // 0=SD2FLT3.COMPH     1=SD2FLT3.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX28 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX29 = 0;      // 0=SD2FLT3.COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX29 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX30 = 0;      // 0=SD2FLT4.COMPH     1=SD2FLT4.COMPH_OR_COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX30 = 0;         // 0=disable output    1=enable output

  OutputXbarRegs.OUTPUT8MUX16TO31CFG.bit.MUX31 = 0;      // 0=SD2FLT4.COMPL
  OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX31 = 0;         // 0=disable output    1=enable output
*/
//--- Finish up
		asm(" EDIS");								// Disable EALLOW protected register access

} // end InitXbar()

//--- end of file -----------------------------------------------------
