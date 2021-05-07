//###########################################################################
//
// FILE:   F2837xD_Emif.c
//
// TITLE:  F2837xD EMIF Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v160 $
// $Release Date: Mon Jun 15 13:36:23 CDT 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File
#include "Init_Func_Prototypes.h"

void InitEmif2(void)
{
	//int i_mux;

	Emif2Initialize();
	setup_emif2_pinmux_async_16bit(GPIO_MUX_CPU1);
	//EMIF clk 의 기본 설정은 sysclk/2 => 100MHz
	//DAC7724의 경우, 쓰기의 	t_strobe>50ns 가 유일한 조건  이다. 안전을 위해 t_strobe =60ns로 설정시, WSTROBE = 5이다.
	//TA 시간은 임의로 가장 긴 시간인 4cycle로 정했다.
	//Configure the access timing for CS2 space
	Emif2Regs.ASYNC_CS2_CR.all =  (EMIF_ASYNC_ASIZE_16  	| // 16Bit Memory Interface
			  	  	  	  	  	  	 EMIF_ASYNC_TA_4 		| // Turn Around time of 1 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_RHOLD_1 	| // Read Hold time of 1 Emif Clock
									 EMIF_ASYNC_RSTROBE_21 	| // Read Strobe time of 21 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_RSETUP_1 	| // Read Setup time of 1 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_WHOLD_1 	| // Write Hold time of 1 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_WSTROBE_6 	| // Write Strobe time of 6 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_WSETUP_1 	| // Write Setup time of 1 Emif Clock
			  	  	  	  	  	  	 EMIF_ASYNC_EW_DISABLE 	| // Extended Wait Disable.
									 EMIF_ASYNC_SS_ENABLE    // Strobe Select Mode Enable.
			  	  	  	  	  	  	);

	//setup async mode for Data pins

//	for (i_mux=57; i_mux<=68;i_mux++)
//	{
//		GPIO_SetupPinOptions(i_mux,0,0x31);
//	}

//	EALLOW;
	//	GpioCtrlRegs.GPDMUX2.bit.GPIO119 = 3; // EM2RNW
	//	GpioCtrlRegs.GPDMUX1.bit.GPIO98 = 3; //EM2A0
	//	GpioCtrlRegs.GPDMUX1.bit.GPIO99 = 3; //EM2A1
	//	GpioCtrlRegs.GPDMUX1.bit.GPIO100 = 3; //EM2A2
	//	GpioCtrlRegs.GPDMUX1.bit.GPIO101 = 3; //EM2A3
	//----------> GPIO_setPinMuxConfig(void) 에서 이미 설정

	//DAC1
	//GpioCtrlRegs.GPDMUX2.bit.GPIO115 = 0; 	//EM2CS0 =input (안씀)
	//GpioCtrlRegs.GPDDIR.bit.GPIO115 = 0;    //EM2CS0 =input (안씀)
	//GpioCtrlRegs.GPDDIR.bit.GPIO118 = 1;   // nDAC0LD = output
	//GpioDataRegs.GPDCLEAR.bit.GPIO118 = 1; //nDAC0LD
	//GpioDataRegs.GPDSET.bit.GPIO118 = 1; //nDAC0LD

	//DAC2
	//GpioCtrlRegs.GPDMUX2.bit.GPIO116 = 3; // EM2CS2
	//GpioCtrlRegs.GPDDIR.bit.GPIO117 = 1;   //nDAC1LD = output
	//GpioDataRegs.GPDCLEAR.bit.GPIO117 = 1; //nDAC1LD clear
	//EDIS;

	//setup_emif2_pinmux_async_16bit(GPIO_MUX_CPU1);
}
//------------------------------------------------by JS

//
// This function initializes the EMIF1 to a known state.
//
void Emif1Initialize(void)
{
    EALLOW;
    // Perform a Module soft reset on EMIF
#ifdef CPU1
    DevCfgRegs.SOFTPRES1.all = 0x1;
   __asm (" nop");
    DevCfgRegs.SOFTPRES1.all = 0x0;
#endif
    EDIS;
}

//
// This function initializes the EMIF2 to a known state.
//
void Emif2Initialize(void)
{
    EALLOW;
    // Perform a Module soft reset on EMIF
#ifdef CPU1
    DevCfgRegs.SOFTPRES1.all = 0x2;
   __asm (" nop");
    DevCfgRegs.SOFTPRES1.all = 0x0;
#endif
    EDIS;

}

//
//Async wait config function
//
void ASync_wait_config(Uint16 inst, Uint16 wait_count, Uint16 wait_polarity)
{
    if (inst == 0) 
    {
        Emif1Regs.ASYNC_WCCR.bit.MAX_EXT_WAIT = wait_count;       // 7:0 Maximum Extended Wait cycles.
        Emif1Regs.ASYNC_WCCR.bit.WP0 = wait_polarity;             // 28 Wait Polarity for pad_wait_i[0].
	}
	else 
    {
        Emif2Regs.ASYNC_WCCR.bit.MAX_EXT_WAIT = wait_count;       // 7:0 Maximum Extended Wait cycles.
        Emif2Regs.ASYNC_WCCR.bit.WP0 = wait_polarity;             // 28 Wait Polarity for pad_wait_i[0].
	}
}

void ASync_cs2_config(Uint16 inst, Uint16 async_mem_data_width, 
                      Uint16 turn_around_time, Uint16 r_hold_time, 
                      Uint16 r_strobe_time, Uint16 r_setup, Uint16 w_hold, 
                      Uint16 w_strobe, Uint16 w_setup, Uint16 extend_wait, 
                      Uint16 strobe_sel) 
{ 
    if (inst == 0) 
    {
        Emif1Regs.ASYNC_CS2_CR.bit.ASIZE = async_mem_data_width;  // 1:0 Asynchronous Memory Size.
        Emif1Regs.ASYNC_CS2_CR.bit.TA= turn_around_time;          // 3:2 Turn Around cycles.
        Emif1Regs.ASYNC_CS2_CR.bit.R_HOLD= r_hold_time;           // 6:4 Read Strobe Hold cycles.
        Emif1Regs.ASYNC_CS2_CR.bit.R_STROBE = r_strobe_time;      // 12:7 Read Strobe Duration cycles.
        Emif1Regs.ASYNC_CS2_CR.bit.R_SETUP = r_setup;             // 16:13 Read Strobe Setup cycles.
        Emif1Regs.ASYNC_CS2_CR.bit.W_HOLD = w_hold;               // 19:17 Write Strobe Hold cycles.
        Emif1Regs.ASYNC_CS2_CR.bit.W_STROBE = w_strobe;           // 25:20 Write Strobe Duration cycles.
        Emif1Regs.ASYNC_CS2_CR.bit.W_SETUP  = w_setup;            // 29:26 Write Strobe Setup cycles.
        Emif1Regs.ASYNC_CS2_CR.bit.EW = extend_wait;              // 30 Extend Wait mode.
        Emif1Regs.ASYNC_CS2_CR.bit.SS = strobe_sel;               // 31 Select Strobe mode.
	}
	else 
    {
        Emif2Regs.ASYNC_CS2_CR.bit.ASIZE = async_mem_data_width;  // 1:0 Asynchronous Memory Size.
        Emif2Regs.ASYNC_CS2_CR.bit.TA= turn_around_time;          // 3:2 Turn Around cycles.
        Emif2Regs.ASYNC_CS2_CR.bit.R_HOLD= r_hold_time;           // 6:4 Read Strobe Hold cycles.
        Emif2Regs.ASYNC_CS2_CR.bit.R_STROBE = r_strobe_time;      // 12:7 Read Strobe Duration cycles.
        Emif2Regs.ASYNC_CS2_CR.bit.R_SETUP = r_setup;             // 16:13 Read Strobe Setup cycles.
        Emif2Regs.ASYNC_CS2_CR.bit.W_HOLD = w_hold;               // 19:17 Write Strobe Hold cycles.
        Emif2Regs.ASYNC_CS2_CR.bit.W_STROBE = w_strobe;           // 25:20 Write Strobe Duration cycles.
        Emif2Regs.ASYNC_CS2_CR.bit.W_SETUP  = w_setup;            // 29:26 Write Strobe Setup cycles.
        Emif2Regs.ASYNC_CS2_CR.bit.EW = extend_wait;              // 30 Extend Wait mode.
        Emif2Regs.ASYNC_CS2_CR.bit.SS = strobe_sel;               // 31 Select Strobe mode.
	}
}

void ASync_cs3_config(Uint16 inst, Uint16 async_mem_data_width, 
                      Uint16 turn_around_time, Uint16 r_hold_time, 
                      Uint16 r_strobe_time, Uint16 r_setup, Uint16 w_hold, 
                      Uint16 w_strobe, Uint16 w_setup, Uint16 extend_wait, 
                      Uint16 strobe_sel) 
{
    Emif1Regs.ASYNC_CS3_CR.bit.ASIZE = async_mem_data_width;      // 1:0 Asynchronous Memory Size.
    Emif1Regs.ASYNC_CS3_CR.bit.TA= turn_around_time;              // 3:2 Turn Around cycles.
    Emif1Regs.ASYNC_CS3_CR.bit.R_HOLD= r_hold_time;               // 6:4 Read Strobe Hold cycles.
    Emif1Regs.ASYNC_CS3_CR.bit.R_STROBE = r_strobe_time;          // 12:7 Read Strobe Duration cycles.
    Emif1Regs.ASYNC_CS3_CR.bit.R_SETUP = r_setup;                 // 16:13 Read Strobe Setup cycles.
    Emif1Regs.ASYNC_CS3_CR.bit.W_HOLD = w_hold;                   // 19:17 Write Strobe Hold cycles.
    Emif1Regs.ASYNC_CS3_CR.bit.W_STROBE = w_strobe;               // 25:20 Write Strobe Duration cycles.
    Emif1Regs.ASYNC_CS3_CR.bit.W_SETUP  = w_setup;                // 29:26 Write Strobe Setup cycles.
    Emif1Regs.ASYNC_CS3_CR.bit.EW = extend_wait;                  // 30 Extend Wait mode.
    Emif1Regs.ASYNC_CS3_CR.bit.SS = strobe_sel;                   // 31 Select Strobe mode.
}

void ASync_cs4_config(Uint16 inst, Uint16 async_mem_data_width, 
                      Uint16 turn_around_time, Uint16 r_hold_time, 
                      Uint16 r_strobe_time, Uint16 r_setup, Uint16 w_hold, 
                      Uint16 w_strobe, Uint16 w_setup, Uint16 extend_wait, 
                      Uint16 strobe_sel) 
{
    Emif1Regs.ASYNC_CS4_CR.bit.ASIZE = async_mem_data_width;      // 1:0 Asynchronous Memory Size.
    Emif1Regs.ASYNC_CS4_CR.bit.TA= turn_around_time;              // 3:2 Turn Around cycles.
    Emif1Regs.ASYNC_CS4_CR.bit.R_HOLD= r_hold_time;               // 6:4 Read Strobe Hold cycles.
    Emif1Regs.ASYNC_CS4_CR.bit.R_STROBE = r_strobe_time;          // 12:7 Read Strobe Duration cycles.
    Emif1Regs.ASYNC_CS4_CR.bit.R_SETUP = r_setup;                 // 16:13 Read Strobe Setup cycles.
    Emif1Regs.ASYNC_CS4_CR.bit.W_HOLD = w_hold;                   // 19:17 Write Strobe Hold cycles.
    Emif1Regs.ASYNC_CS4_CR.bit.W_STROBE = w_strobe;               // 25:20 Write Strobe Duration cycles.
    Emif1Regs.ASYNC_CS4_CR.bit.W_SETUP  = w_setup;                // 29:26 Write Strobe Setup cycles.
    Emif1Regs.ASYNC_CS4_CR.bit.EW = extend_wait;                  // 30 Extend Wait mode.
    Emif1Regs.ASYNC_CS4_CR.bit.SS = strobe_sel;                   // 31 Select Strobe mode.
}

#ifdef CPU1
//function for EMIF1 GPIO pin setup
void setup_emif1_pinmux_async_16bit(Uint16 cpu_sel)
{
    Uint16 i;
    
    for (i=28; i<=52;i++) 
    {
        if (i != 42 || i != 43) 
        {
            GPIO_SetupPinMux(i,cpu_sel,2);
        }
    }
    for (i=63; i<=87;i++) 
    {
        if (i != 84)
        {        
            GPIO_SetupPinMux(i,cpu_sel,2);
        }
    }

    GPIO_SetupPinMux(88,cpu_sel,3);
    GPIO_SetupPinMux(89,cpu_sel,3);
    GPIO_SetupPinMux(90,cpu_sel,3);
    GPIO_SetupPinMux(91,cpu_sel,3);
    GPIO_SetupPinMux(92,cpu_sel,3);
    GPIO_SetupPinMux(93,cpu_sel,3);
    GPIO_SetupPinMux(94,cpu_sel,2);
	   
    //setup async mode and enable pull-ups for Data pins
    for (i=69; i<=85;i++) 
    {
        if (i != 84)
        {        
            GPIO_SetupPinOptions(i,0,0x31); // GPIO_ASYNC||GPIO_PULLUP
        }
    }   
 }   
 
void setup_emif1_pinmux_async_32bit(Uint16 cpu_sel)
{
    Uint16 i;
    
    for (i=28; i<=87;i++) 
    {
        if (i != 42 || i != 43 || i != 84 )
        {
            GPIO_SetupPinMux(i,cpu_sel,2);
        }
    }
    
    GPIO_SetupPinMux(88,cpu_sel,3);
    GPIO_SetupPinMux(89,cpu_sel,3);
    GPIO_SetupPinMux(90,cpu_sel,3);
    GPIO_SetupPinMux(91,cpu_sel,3);
    GPIO_SetupPinMux(92,cpu_sel,3);
    GPIO_SetupPinMux(93,cpu_sel,3);
    GPIO_SetupPinMux(94,cpu_sel,2);
	   
    //setup async mode for Data pins	   
    for (i=53; i<=85;i++) 
    {
        if (i != 84)
        {        
            GPIO_SetupPinOptions(i,0,0x31);
        }
    }	   
}
 
//function for EMIF1 GPIO pin setup
void setup_emif2_pinmux_async_16bit(Uint16 cpu_sel)
{
	Uint16 i;
    
	//for (i=96; i<=121;i++)
	for (i=98; i<=108;i++) 
  {
		GPIO_SetupPinMux(i,cpu_sel,3);
	}
    
	for (i=57; i<=68;i++) 
  {
		GPIO_SetupPinMux(i,cpu_sel,3);
	}

	//setup async mode for Data pins
	for (i=57; i<=68;i++) 
    {
		GPIO_SetupPinOptions(i,0,0x31);
	}
	GPIO_SetupPinMux(109, cpu_sel, 0);
  GPIO_SetupPinOptions(109, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(111,cpu_sel,3);
	GPIO_SetupPinMux(112,cpu_sel,3);
	GPIO_SetupPinMux(116,cpu_sel,3);
	//GPIO_SetupPinMux(119, cpu_sel, 0);
  //GPIO_SetupPinOptions(119, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(119,cpu_sel,3);
	GPIO_SetupPinMux(120,cpu_sel,3);
	GPIO_SetupPinMux(121,cpu_sel,3);
}
 
void setup_emif1_pinmux_sdram_16bit(Uint16 cpu_sel)
{
	int i;
    
	for (i=29; i<=52;i++) 
    {
        if (i != 42 || i != 43)
        {        
            GPIO_SetupPinMux(i,cpu_sel,2);
        }
	}
    
	for (i=69; i<=85;i++) 
    {
        if (i != 84)
        {
            GPIO_SetupPinMux(i,cpu_sel,2);
        }
	}
	
	for(i=86;i<=93;i++)
    {
	   GPIO_SetupPinMux(i,cpu_sel,3);
	}

	//configure Data pins for Async mode
	for (i = 69;i <= 85;i++)
    {
		if (i != 84)
        {
			GPIO_SetupPinOptions(i,0,0x31);
        }
	}

	GPIO_SetupPinOptions(88,0,0x31);
	GPIO_SetupPinOptions(89,0,0x31);
	GPIO_SetupPinOptions(90,0,0x31);
	GPIO_SetupPinOptions(91,0,0x31);
}

void setup_emif2_pinmux_sdram_16bit(Uint16 cpu_sel)
{
	int i;
    
	for (i=53; i<=68;i++) 
    {
        GPIO_SetupPinMux(i,cpu_sel,3);
	}
	for (i=96; i<=121;i++) 
    {
        GPIO_SetupPinMux(i,cpu_sel,3);
	}
	
	//configure Data pins for Async mode
	for (i = 53;i <= 68;i++)
    {
        GPIO_SetupPinOptions(i,0,0x31);
	}
}

void setup_emif1_pinmux_sdram_32bit(Uint16 cpu_sel)
{
	int i;
    
	for (i=28; i<=85;i++) 
    {
        if (i != 42 || i != 43 || i != 84 )
        {        
            GPIO_SetupPinMux(i,cpu_sel,2);
        }
	}

	for(i=86;i<=93;i++)
    {
		GPIO_SetupPinMux(i,cpu_sel,3);
	}

	GPIO_SetupPinMux(94,cpu_sel,2);

	//configure Data pins for Async mode
	for (i = 53;i <= 85;i++)
    {
        if (i != 84)
        {
            GPIO_SetupPinOptions(i,0,0x31);
        }
	}

	GPIO_SetupPinOptions(88,0,0x31);
	GPIO_SetupPinOptions(89,0,0x31);
	GPIO_SetupPinOptions(90,0,0x31);
	GPIO_SetupPinOptions(91,0,0x31);
 }

#endif // CPU1
