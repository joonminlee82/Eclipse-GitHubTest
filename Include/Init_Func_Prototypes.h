/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#ifndef _INIT_FUNC_PROTOTYPES_H
#define _INIT_FUNC_PROTOTYPES_H

//---------------------------------------------------------------------------
// Include any other Header Files
//
#include "F2837xD_Cla_typedefs.h"        // CLA type definitions
#include "F2837xD_Device.h"              // F2807x header file peripheral address definitions
#include "F2837xD_Adc_defines.h"         // ADC definitions
#include "F2837xD_DefaultIsr.h"          // ISR definitions
#include "F2837xD_Pie_defines.h"         // PIE definitions

#include "f2837xd_pinmux.h"
#include "CC.h"
#include "AD_Offset.h"
#include "DAC.h"
#include "Init_CMPSS.h"
#include "Fault.h"
#include "eQEP_Encoder.h"

//---------------------------------------------------------------------------
// Function Prototypes
//
extern void Gpio_setup(void);
extern void InitXbar(void);
extern void InitEPwm(void);
extern void InitAdc(void);
extern void Config_16bit_Adc(volatile struct ADC_REGS *AdcXRegs, int ADC_num, int SOC0_ch,  int SOC1_ch,  int SOC2_ch);
extern void Wait_EOC_ADC_SOC1(void);
extern void Wait_EOC_ADC_SOC2(void);
extern void Wait_EOC_ADC_SOC3(void);

extern void InitEmif2(void);
extern void Emif2Initialize(void);
extern void ASync_cs2_config(	Uint16 inst, Uint16 async_mem_data_width,
								Uint16 turn_around_time, Uint16 r_hold_time,
								Uint16 r_strobe_time, Uint16 r_setup, Uint16 w_hold,
								Uint16 w_strobe, Uint16 w_setup, Uint16 extend_wait,
								Uint16 strobe_sel) ;
extern void setup_emif2_pinmux_async_16bit(Uint16 cpu_sel);

extern void InitEQep(void);
extern void Init_SDFM(void);
extern void InitSPIC(void);

//---------------------------------------------------------------------------
#endif  // end of SNU_28377D_INIT_FUNC_PROTOTYPES_H definition


//--- end of file -----------------------------------------------------
