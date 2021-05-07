/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler v6.1
	Author				: Inverter part, E/C Department in Hyudai elevator(with SNU)
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/
#include "CPU1_CLA_Shared.h"
#include "F28x_Project.h"
#include "F2837xD_Cla_defines.h"
#include "F2837xD_Cla_typedefs.h"

void InitCla(void)
{
  CLA_configClaMemory();
  CLA_initCpu1Cla1();
	
  asm("  IACK  #0x0080");
	__asm(" RPT #3 || NOP");
	
}

//*****************************************************************************
// function definitions
//*****************************************************************************

void CLA_configClaMemory(void)
{
	extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
	//extern uint32_t Cla1mathTablesLoadStart, Cla1mathTablesRunStart, Cla1mathTablesLoadSize;

	EALLOW;

#ifdef _FLASH
	// Copy over code from FLASH to RAM
	memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
			(uint32_t)&Cla1funcsLoadSize);
	//memcpy((uint32_t *)&Cla1mathTablesRunStart, (uint32_t *)&Cla1mathTablesLoadStart,
			//(uint32_t)&Cla1mathTablesLoadSize);
#endif //_FLASH

	// Initialize and wait for CLA1ToCPUMsgRAM
	MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
	while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

	// Initialize and wait for CPUToCLA1MsgRAM
	MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
	while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

	
	//--- Memory Configuration - Master CPU and CLA Select
	MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;        // 0=CPU    1=CPU and CLA
	MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;        // 0=CPU    1=CPU and CLA
	MemCfgRegs.LSxMSEL.bit.MSEL_LS2 = 1;        // 0=CPU    1=CPU and CLA
	MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;        // 0=CPU    1=CPU and CLA
	MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;        // 0=CPU    1=CPU and CLA
	MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;        // 0=CPU    1=CPU and CLA

//--- Memory Configuration - CLA Data Memory and CLA Program Memory Select
	MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;    // 0=CLA data memory    1=CLA program memory
	MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;    // 0=CLA data memory    1=CLA program memory
	MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS2 = 1;    // 0=CLA data memory    1=CLA program memory
	MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 1;    // 0=CLA data memory    1=CLA program memory
	MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;    // 0=CLA data memory    1=CLA program memory
	MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1;    // 0=CLA data memory    1=CLA program memory
	
	EDIS;
}

void CLA_initCpu1Cla1(void)
{
	// Compute all CLA task vectors
	// On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
	// opposed to offsets used on older Type-0 CLAs
	EALLOW;
	Cla1Regs.MVECT1 = (uint16_t)(&cla_ISR1);
	Cla1Regs.MVECT2 = (uint16_t)(&cla_ISR2);
	Cla1Regs.MVECT3 = (uint16_t)(&cla_ISR3);
	Cla1Regs.MVECT4 = (uint16_t)(&cla_ISR4);
	Cla1Regs.MVECT5 = (uint16_t)(&cla_ISR5);
	Cla1Regs.MVECT6 = (uint16_t)(&cla_ISR6);
	Cla1Regs.MVECT7 = (uint16_t)(&cla_ISR7);
	Cla1Regs.MVECT8 = (uint16_t)(&cla_Init);

 //--- Enable use software to start a task (IACK)
	Cla1Regs.MCTL.bit.IACKE = 1;        // Enable IACKE to start task using software
//--- Force one-time initialization Task 8 - zero delay buffer
	Cla1Regs.MIER.all = 0x0080;            // Enable CLA task interrupt 8
	asm("  IACK  #0x0080");                // IACK - CLA task force instruction
	asm("  RPT #3 || NOP");                // Wait at least 4 cycles
	while(Cla1Regs.MIRUN.bit.INT8 == 1);   // Loop until task completes
//--- Enable CLA task interrupts
	Cla1Regs.MIER.all = 0x00FF;        // Enable CLA task interrupt 1 (and disable interrupt 8)
	
	EDIS;
}
