//###########################################################################
// FILE:   F2837xD_ECap.c
// TITLE:  F2837xD eCAP Initialization & Support Functions.
//###########################################################################
// $TI Release: F2837xD Support Library v160 $
// $Release Date: Mon Jun 15 13:36:23 CDT 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File

//---------------------------------------------------------------------------
// InitECap:
//---------------------------------------------------------------------------
// This function initializes the eCAP(s) to a known state.
void InitECap(void)
{
    // Initialize eCAP1/2/3/4/5/6
//-----ECap1-------------------------------(for SDFM1)
	ECap1Regs.ECCTL2.bit.CAP_APWM = 1;		// Enable APWM mode
	//ECap1Regs.CAP1 = 40;					// Set Period value
	//ECap1Regs.CAP2 = 20;					// Set Compare value
	//ECap1Regs.CAP3 = 40;					// Set Period value(shadow)
	//ECap1Regs.CAP4 = 20;					// Set Compare value(shadow)
	ECap1Regs.CAP1 		= 20;				// Set Period value
	ECap1Regs.CAP2 		= 10;				// Set Compare value
	ECap1Regs.CAP3 		= 20;				// Set Period value(shadow)
	ECap1Regs.CAP4 		= 10;				// Set Compare value(shadow)
	ECap1Regs.ECCLR.all = 0x0FF;			// Clear pending __interrupts
	//ECap1Regs.ECEINT.bit.CTR_EQ_CMP = 0; 	// enable Compare Equal Int

	EALLOW;
	OutputXbarRegs.OUTPUT5MUX0TO15CFG.bit.MUX0 	= 3;	// Select ECAP1.OUT on Mux0  //Gmux 먼저 설정해야 glitch 피할수 있음
	OutputXbarRegs.OUTPUT5MUXENABLE.bit.MUX0 	= 1;	// Enable MUX0 for ECAP1.OUT
	//GpioCtrlRegs.GPAGMUX2.bit.GPIO28 =1;
	//GpioCtrlRegs.GPAMUX2.bit.GPIO28	 =1;
	//----------> GPIO_setPinMuxConfig(void) 에서 이미 설정
	EDIS;
	// Start counters
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;

//-----ECap2-------------------------------(for SDFM1)
	ECap2Regs.ECCTL2.bit.CAP_APWM = 1;		// Enable APWM mode
	//ECap2Regs.CAP1 = 40;					// Set Period value
	//ECap2Regs.CAP2 = 20;					// Set Compare value
	//ECap2Regs.CAP3 = 40;					// Set Period value(shadow)
	//ECap2Regs.CAP4 = 20;					// Set Compare value(shadow)
	ECap2Regs.CAP1 		= 20;				// Set Period value
	ECap2Regs.CAP2 		= 10;				// Set Compare value
	ECap2Regs.CAP3 		= 20;				// Set Period value(shadow)
	ECap2Regs.CAP4 		= 10;				// Set Compare value(shadow)
	ECap2Regs.ECCLR.all = 0x0FF;			// Clear pending __interrupts
	//ECap1Regs.ECEINT.bit.CTR_EQ_CMP = 0; 	// enable Compare Equal Int

	EALLOW;
	OutputXbarRegs.OUTPUT6MUX0TO15CFG.bit.MUX2 = 3;	// Select ECAP2.OUT on Mux2  //Gmux 먼저 설정해야 glitch 피할수 있음
	OutputXbarRegs.OUTPUT6MUXENABLE.bit.MUX2 = 1;	// Enable MUX0 for ECAP1.OUT
	//GpioCtrlRegs.GPAGMUX2.bit.GPIO29 =1;
	//GpioCtrlRegs.GPAMUX2.bit.GPIO29	 =1;
	//----------> GPIO_setPinMuxConfig(void) 에서 이미 설정
	EDIS;
	// Start counters
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;
//-----ECap3-------------------------------(for CLA timer)
	ECap3Regs.ECCTL2.bit.CAP_APWM = 1;		// Enable APWM mode
	ECap3Regs.CAP1 		= 0xffffffff;		// Set Period value
	ECap3Regs.CAP2 		= 0;				// Set Compare value
	ECap3Regs.CAP3 		= 0xffffffff;		// Set Period value(shadow)
	ECap3Regs.CAP4 		= 0;				// Set Compare value(shadow)
	ECap3Regs.ECCLR.all = 0x0FF;			// Clear pending __interrupts
	//ECap1Regs.ECEINT.bit.CTR_EQ_CMP = 0; 	// enable Compare Equal Int

	// Start counters
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1; //1 : start
    //tbd...

}

//---------------------------------------------------------------------------
// Example: InitECapGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ECAP pins
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
// Caution:
// For each eCAP peripheral
// Only one GPIO pin should be enabled for ECAP operation.
// Comment out other unwanted lines.

void InitECapGpio()
{
}

void InitECap1Gpio(Uint16 pin)
{
	EALLOW;
	InputXbarRegs.INPUT7SELECT = pin; 		// Set eCAP1 source to GPIO-pin
	EDIS;
}

void InitECap2Gpio(Uint16 pin)
{
	EALLOW;
	InputXbarRegs.INPUT8SELECT = pin; 		// Set eCAP2 source to GPIO-pin
	EDIS;
}

void InitECap3Gpio(Uint16 pin)
{
	EALLOW;
	InputXbarRegs.INPUT9SELECT = pin; 		// Set eCAP3 source to GPIO-pin
	EDIS;
}

void InitECap4Gpio(Uint16 pin)
{
	EALLOW;
	InputXbarRegs.INPUT10SELECT = pin; 		// Set eCAP4 source to GPIO-pin
	EDIS;
}

void InitECap5Gpio(Uint16 pin)
{
	EALLOW;
	InputXbarRegs.INPUT11SELECT = pin; 		// Set eCAP5 source to GPIO-pin
	EDIS;
}

void InitECap6Gpio(Uint16 pin)
{
	EALLOW;
	InputXbarRegs.INPUT12SELECT = pin; 		// Set eCAP6 source to GPIO-pin
	EDIS;
}

void InitAPwm1Gpio()
{
	EALLOW;
	OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX0 = 3;	// Select ECAP1.OUT on Mux0
	OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX0 = 1;	// Enable MUX0 for ECAP1.OUT
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 3;	// Select OUTPUTXBAR3 on GPIO5
	EDIS;
}
//===========================================================================
// End of file.
//===========================================================================
