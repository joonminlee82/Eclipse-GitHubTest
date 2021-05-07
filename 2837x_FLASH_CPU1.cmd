// The user must define CLA_C in the project linker settings if using the
// CLA C compiler
// Project Properties -> C2000 Linker -> Advanced Options -> Command File
// Preprocessing -> --define
--define=CLA_C=1
_Cla1Prog_Start = _Cla1funcsRunStart;
-heap 0x200
-stack 0x200

#ifdef CLA_C
// Define a size for the CLA scratchpad area that will be used
// by the CLA compiler for local symbols and temps
// Also force references to the special symbols that mark the
// scratchpad are. 
CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start
#endif //CLA_C

MEMORY
{
PAGE 0 :  /* Program Memory */
          /* Memory (RAM/FLASH) blocks can be moved to PAGE1 for data allocation */
          /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN           	: origin = 0x0A0000, length = 0x000002
   RAMM0           	: origin = 0x000122, length = 0x0002DE /*M0 RAM*/
   RAMD0           	: origin = 0x00B000, length = 0x000800 /*D0 RAM*/
   
   RAMLS2_5      			: origin = 0x009000, length = 0x002000 /*CLA Program*/
   //RAMLS3      			: origin = 0x009800, length = 0x000800 /*CLA Program*/
   //RAMLS4      			: origin = 0x00A000, length = 0x000800 /*CLA Program*/
   //RAMLS5      			: origin = 0x00A800, length = 0x000800 /*CLA Program*/
   RAMGS0_5      		: origin = 0x00C000, length = 0x006000 /*for ramfunc*/
   RESET           	: origin = 0x3FFFC0, length = 0x000002
   
   /* Flash sectors */
   
   //FLASHA           : origin = 0x080002, length = 0x001FFE	/* on-chip Flash */
   //FLASHB           : origin = 0x082000, length = 0x002000	/* on-chip Flash */
   //FLASHC           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
   //FLASHD           : origin = 0x086000, length = 0x002000	/* on-chip Flash */
   //FLASHE           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
   //FLASHF           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
   //FLASHG           : origin = 0x098000, length = 0x008000	/* on-chip Flash */
   //FLASHH           : origin = 0x0A0000, length = 0x008000	/* on-chip Flash */
   FLASH_AG					: origin = 0x0A0002, length = 0x0017EFE
   FLASH_TAG				: origin = 0x0B7F00, length = 0x0000100
   

   /* Part of Z1 OTP.  LinkPointers/PSWD LOCK/CRC LOCK/JTAG lock/ Boot Ctrl */
   DCSM_OTP_Z1_P0	: origin = 0x78000, length = 0x000020		
   /* Part of Z2 OTP.  LinkPointers/PSWD LOCK/CRC LOCK/JTAG lock/ Boot Ctrl */
   DCSM_OTP_Z2_P0	: origin = 0x78200, length = 0x000020		
   
   /* DCSM Z1 Zone Select Contents (!!Movable!!) */
   /* Part of Z1 OTP.  Z1 password locations / Flash and RAM partitioning */
   DCSM_ZSEL_Z1_P0	: origin = 0x78020, length = 0x000010	
   
   /* DCSM Z1 Zone Select Contents (!!Movable!!) */
   /* Part of Z2 OTP.  Z2 password locations / Flash and RAM partitioning  */
   DCSM_ZSEL_Z2_P0	: origin = 0x78220, length = 0x000010	

PAGE 1 : /* Data Memory */
         /* Memory (RAM/FLASH) blocks can be moved to PAGE0 for program allocation */

   BOOT_RSVD       : origin = 0x000002, length = 0x000120 /* Part of M0, BOOT rom will use this for stack */
   RAMM1           : origin = 0x000400, length = 0x000400 /* on-chip RAM block M1 */
   RAMD1           : origin = 0x00B800, length = 0x000800	/*D1 RAM*/

   RAMLS0          : origin = 0x008000, length = 0x000800 /*CLA Data*/
   RAMLS1          : origin = 0x008800, length = 0x000800 /*CLA Data*/

   RAMGS6_13      	 : origin = 0x012000, length = 0x008000 /*GS6 ~ GS13 RAM, for ebss*/
   RAMGS14_0         : origin = 0x01A000, length = 0x000800 /*GS14 RAM, for ebss*/
   RAMGS14_1         : origin = 0x01A800, length = 0x000800 /*GS14 RAM, for Filter_RegsFile*/
   RAMGS15_0         : origin = 0x01B000, length = 0x000800 /*GS15 RAM, for SHARERAMGS0*/
   RAMGS15_1         : origin = 0x01B800, length = 0x000800 /*GS15 RAM, for SHARERAMGS1*/
	 
	 CLA1_MSGRAMLOW    : origin = 0x001480, length = 0x000080 /*CPU1_CLA1 to CPU MSG RAM*/
   CLA1_MSGRAMHIGH   : origin = 0x001500, length = 0x000080 /*CPU1 to CPU1_CLA1 MSG RAM*/
	 
   CPU2TOCPU1RAM     : origin = 0x03F800, length = 0x000400
   CPU1TOCPU2RAM     : origin = 0x03FC00, length = 0x000400
   
   ADCA_RESULT   : origin = 0x000B00, length = 0x000020
   ADCB_RESULT   : origin = 0x000B20, length = 0x000020
   ADCC_RESULT   : origin = 0x000B40, length = 0x000020
   ADCD_RESULT   : origin = 0x000B60, length = 0x000020

   ADCA          : origin = 0x007400, length = 0x000080
   ADCB          : origin = 0x007480, length = 0x000080
   ADCC          : origin = 0x007500, length = 0x000080
   ADCD          : origin = 0x007580, length = 0x000080

   ANALOG_SUBSYS : origin = 0x05D180, length = 0x000080
   
   CLA1          : origin = 0x001400, length = 0x000040     /* CLA registers */

   CLB_XBAR      : origin = 0x007A40, length = 0x000040

   CMPSS1        : origin = 0x005C80, length = 0x000020
   CMPSS2        : origin = 0x005CA0, length = 0x000020
   CMPSS3        : origin = 0x005CC0, length = 0x000020
   CMPSS4        : origin = 0x005CE0, length = 0x000020
   CMPSS5        : origin = 0x005D00, length = 0x000020
   CMPSS6        : origin = 0x005D20, length = 0x000020
   CMPSS7        : origin = 0x005D40, length = 0x000020
   CMPSS8        : origin = 0x005D60, length = 0x000020

   CPU_TIMER0    : origin = 0x000C00, length = 0x000008     /* CPU Timer0 registers */
   CPU_TIMER1    : origin = 0x000C08, length = 0x000008     /* CPU Timer1 registers */
   CPU_TIMER2    : origin = 0x000C10, length = 0x000008     /* CPU Timer2 registers */

   DACA          : origin = 0x005C00, length = 0x000010
   DACB          : origin = 0x005C10, length = 0x000010
   DACC          : origin = 0x005C20, length = 0x000010

   DMA          : origin = 0x001000, length = 0x000200
   DMACLASRCSEL : origin = 0x007980, length = 0x000040

   ECAP1        : origin = 0x005000, length = 0x000020     /* Enhanced Capture 1 registers */
   ECAP2        : origin = 0x005020, length = 0x000020     /* Enhanced Capture 2 registers */
   ECAP3        : origin = 0x005040, length = 0x000020     /* Enhanced Capture 3 registers */
   ECAP4        : origin = 0x005060, length = 0x000020     /* Enhanced Capture 4 registers */
   ECAP5        : origin = 0x005080, length = 0x000020     /* Enhanced Capture 5 registers */
   ECAP6        : origin = 0x0050A0, length = 0x000020     /* Enhanced Capture 6 registers */
   
   EMIF1        : origin = 0x047000, length = 0x000800
   EMIF2        : origin = 0x047800, length = 0x000800

   EQEP1        : origin = 0x005100, length = 0x000040     /* Enhanced QEP 1 registers */
   EQEP2        : origin = 0x005140, length = 0x000040     /* Enhanced QEP 2 registers */
   EQEP3        : origin = 0x005180, length = 0x000040     /* Enhanced QEP 3 registers */

   EPWM1        : origin = 0x004000, length = 0x000100     /* Enhanced PWM 1 registers */
   EPWM2        : origin = 0x004100, length = 0x000100     /* Enhanced PWM 2 registers */
   EPWM3        : origin = 0x004200, length = 0x000100     /* Enhanced PWM 3 registers */
   EPWM4        : origin = 0x004300, length = 0x000100     /* Enhanced PWM 4 registers */
   EPWM5        : origin = 0x004400, length = 0x000100     /* Enhanced PWM 5 registers */
   EPWM6        : origin = 0x004500, length = 0x000100     /* Enhanced PWM 6 registers */
   EPWM7        : origin = 0x004600, length = 0x000100     /* Enhanced PWM 7 registers */
   EPWM8        : origin = 0x004700, length = 0x000100     /* Enhanced PWM 8 registers */
   EPWM9        : origin = 0x004800, length = 0x000100     /* Enhanced PWM 9 registers */
   EPWM10       : origin = 0x004900, length = 0x000100     /* Enhanced PWM 10 registers */
   EPWM11       : origin = 0x004A00, length = 0x000100     /* Enhanced PWM 11 registers */
   EPWM12       : origin = 0x004B00, length = 0x000100     /* Enhanced PWM 12 registers */

   EPWM_XBAR  	: origin = 0x007A00, length = 0x000040

   FLASH0_CTRL  : origin = 0x05F800, length = 0x000300
   FLASH0_ECC   : origin = 0x05FB00, length = 0x000040

   GPIOCTRL     : origin = 0x007C00, length = 0x000180     /* GPIO control registers */
   GPIODAT      : origin = 0x007F00, length = 0x000030     /* GPIO data registers */

   OUTPUT_XBAR  : origin = 0x007A80, length = 0x000040
   I2CA         : origin = 0x007300, length = 0x000040     /* I2C-A registers */
   I2CB         : origin = 0x007340, length = 0x000040     /* I2C-B registers */

   /*IPC          : origin = 0x050000, length = 0x001000*/
   IPC          : origin = 0x050000, length = 0x000024

   FLASHPUMPSEMAPHORE   : origin = 0x050024, length = 0x000002

   ROMPREFETCH  : origin = 0x05E608, length = 0x000002

   MEMCFG       : origin = 0x05F400, length = 0x000080     /* Mem Config registers */
   EMIF1CONFIG  : origin = 0x05F480, length = 0x000020     /* Emif-1 Config registers */
   EMIF2CONFIG  : origin = 0x05F4A0, length = 0x000020     /* Emif-2 Config registers */
   ACCESSPROTECTION  : origin = 0x05F4C0, length = 0x000040     /* Access Protection registers */
   MEMORYERROR  : origin = 0x05F500, length = 0x000040     /* Access Protection registers */
   ROMWAITSTATE : origin = 0x05F540, length = 0x000002     /* ROM Config registers */
   

   MCBSPA       : origin = 0x006000, length = 0x000040     /* McBSP-A registers */
   MCBSPB       : origin = 0x006040, length = 0x000040     /* McBSP-A registers */

   NMIINTRUPT   : origin = 0x007060, length = 0x000010     /* NMI Watchdog Interrupt Registers */

   PIE_CTRL     : origin = 0x000CE0, length = 0x000020     /* PIE control registers */
   PIE_VECT     : origin = 0x000D00, length = 0x000200     /* PIE Vector Table */
   SCIA         : origin = 0x007200, length = 0x000010     /* SCI-A registers */
   SCIB         : origin = 0x007210, length = 0x000010     /* SCI-B registers */
   SCIC         : origin = 0x007220, length = 0x000010     /* SCI-C registers */
   SCID         : origin = 0x007230, length = 0x000010     /* SCI-D registers */
   
   SDFM1		: origin = 0x005E00, length = 0x000080	   /* Sigma delta 1 registers */
   SDFM2		: origin = 0x005E80, length = 0x000080	   /* Sigma delta 2 registers */

   SPIA         : origin = 0x006100, length = 0x000010
   SPIB         : origin = 0x006110, length = 0x000010
   SPIC         : origin = 0x006120, length = 0x000010
   SPID         : origin = 0x006130, length = 0x000010

   UPP          : origin = 0x006200, length = 0x000100     /* uPP registers */

   DEV_CFG     : origin = 0x05D000, length = 0x000180
   CLK_CFG     : origin = 0x05D200, length = 0x000100
   CPU_SYS     : origin = 0x05D300, length = 0x000100

   INPUT_XBAR   : origin = 0x007900, length = 0x000020
   XBAR         : origin = 0x007920, length = 0x000020
   /*TRIG         : origin = 0x007940, length = 0x000010*/
   SYNC_SOC     : origin = 0x007940, length = 0x000010
   WD           : origin = 0x007000, length = 0x000040

   XINT         : origin = 0x007070, length = 0x000010

   DCSM_Z1      : origin = 0x05F000, length = 0x000030     /* Zone 1 Dual code security module registers */
   DCSM_Z2      : origin = 0x05F040, length = 0x000030     /* Zone 2 Dual code security module registers */
   DCSM_COMMON  : origin = 0x05F070, length = 0x000010     /* Common Dual code security module registers */

   DCSM_Z1_OTP  : origin = 0x078000, length = 0x000020     /* Part of Z1 OTP.  LinkPointer/JTAG lock/ Boot Mode */
   DCSM_Z2_OTP  : origin = 0x078200, length = 0x000020     /* Part of Z2 OTP.  LinkPointer/JTAG lock */
}


SECTIONS
{
   /* Allocate program areas: */
   .cinit          : > FLASH_AG      PAGE = 0, ALIGN(4)
   .pinit          : > FLASH_AG,     PAGE = 0, ALIGN(4)
   .text           : > FLASH_AG      PAGE = 0, ALIGN(4)
   .text1          : > FLASH_AG,  		 PAGE = 0, ALIGN(4)
	 {
		 keymenu.obj(.text)
		 SCI.obj(.text)
	 }
   codestart       : > BEGIN       PAGE = 0, ALIGN(4)
   ramfuncs        : LOAD = FLASH_AG,
                     RUN = RAMGS0_5,
                     LOAD_START(_RamfuncsLoadStart),
                     LOAD_SIZE(_RamfuncsLoadSize),
                     LOAD_END(_RamfuncsLoadEnd),
                     RUN_START(_RamfuncsRunStart),
                     RUN_SIZE(_RamfuncsRunSize),
                     RUN_END(_RamfuncsRunEnd),
                     PAGE = 0, ALIGN(4)
						 
   /* Allocate uninitalized data sections: */
   .stack              : > RAMM1        PAGE = 1
   .ebss               : > RAMGS6_13   PAGE = 1
   usersector 		   : > RAMGS14_0	PAGE = 1
   .esysmem            : > RAMD1       	PAGE = 1

   /* Initalized sections go in Flash */
   .econst             : > FLASH_AG      PAGE = 0, ALIGN(4)
   .switch             : > FLASH_AG      PAGE = 0, ALIGN(4)
   
   .reset              : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   Filter_RegsFile     : > RAMGS15_0,	   PAGE = 1
   
   .sysmem          : > RAMGS6_13,    PAGE = 1                      
   .cio             : > RAMGS6_13,    PAGE = 1
   
   SHARERAMGS0		: > RAMGS15_0,		PAGE = 1
   SHARERAMGS1		: > RAMGS15_1,		PAGE = 1
   
   /* The following section definitions are required when using the IPC API Drivers */ 
		GROUP : > CPU1TOCPU2RAM, PAGE = 1 
		{
		    PUTBUFFER 
		    PUTWRITEIDX 
		    GETREADIDX 
		}
		
		GROUP : > CPU2TOCPU1RAM, PAGE = 1
		{
		    GETBUFFER :    TYPE = DSECT
		    GETWRITEIDX :  TYPE = DSECT
		    PUTREADIDX :   TYPE = DSECT
		}  
    /* CLA specific sections */
   Cla1Prog         	: LOAD = FLASH_AG, /* Note for running from RAM the load and RUN can be the same */
                      //RUN = RAMLS2 | RAMLS3 | RAMLS4 | RAMLS5,
                      RUN = RAMLS2_5,
                      LOAD_START(_Cla1funcsLoadStart),
                      LOAD_SIZE(_Cla1funcsLoadSize),
                      LOAD_END(_Cla1funcsLoadEnd),
                      RUN_START(_Cla1funcsRunStart),
                      PAGE = 0, ALIGN(4)   
   /*Cla1Prog     : >> RAMLS4 | RAMLS5, PAGE=0*/

   CLADataLS0		: > RAMLS0, PAGE=1
   CLADataLS1		: > RAMLS1, PAGE=1

   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW,   PAGE = 1
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH,  PAGE = 1
   
   /*FPUmathTables		: > RAMD1, PAGE = 1*/
   
   dcsm_otp_z1		: > DCSM_OTP_Z1_P0		PAGE = 0, type = DSECT
   dcsm_otp_z2		: > DCSM_OTP_Z2_P0		PAGE = 0, type = DSECT 
   
   dcsm_zsel_z1		: > DCSM_ZSEL_Z1_P0		PAGE = 0, type = DSECT
   dcsm_zsel_z2		: > DCSM_ZSEL_Z2_P0		PAGE = 0, type = DSECT
   
#ifdef CLA_C
   /* CLA C compiler sections */
   //
   // Must be allocated to memory the CLA has write access to
   //
   /*CLA1mathTables    : LOAD = FLASHN,
                     RUN = RAMLS1,
                     LOAD_START(_Cla1mathTablesLoadStart),
                     LOAD_END(_Cla1mathTablesLoadEnd),
                     RUN_START(_Cla1mathTablesRunStart),
                     LOAD_SIZE(_Cla1mathTablesLoadSize),
                     PAGE = 1*/
  /* CLA1mathTables		:  > RAMLS1,  PAGE = 1*/
   CLAscratch       :  > RAMLS1,  PAGE = 1
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) }

   .scratchpad      : > RAMLS1,       PAGE = 1                  
   .bss_cla		      : > RAMLS1,       PAGE = 1
   .const_cla	    :  LOAD = FLASH_AI,
                     RUN = RAMLS1,
                     RUN_START(_Cla1ConstRunStart),
                     LOAD_START(_Cla1ConstLoadStart),
                     LOAD_SIZE(_Cla1ConstLoadSize),
                     PAGE = 1, ALIGN(4)

#endif //CLA_C
}

SECTIONS
{
/*** PIE Vect Table and Boot ROM Variables Structures ***/
  UNION run = PIE_VECT, PAGE = 1
   {
      PieVectTableFile    : TYPE=DSECT
      GROUP
      {
         EmuKeyVar        : TYPE=DSECT
         EmuBModeVar      : TYPE=DSECT
         FlashCallbackVar : TYPE=DSECT
         FlashScalingVar  : TYPE=DSECT
      }
   }

   AdcaResultFile        : > ADCA_RESULT,  PAGE = 1
   AdcbResultFile        : > ADCB_RESULT,  PAGE = 1
   AdccResultFile        : > ADCC_RESULT,  PAGE = 1
   AdcdResultFile        : > ADCD_RESULT,  PAGE = 1

   AdcaRegsFile          : > ADCA,         PAGE = 1
   AdcbRegsFile          : > ADCB,         PAGE = 1
   AdccRegsFile          : > ADCC,         PAGE = 1
   AdcdRegsFile          : > ADCD,         PAGE = 1

   AnalogSubsysRegsFile : > ANALOG_SUBSYS, PAGE = 1

   Cla1RegsFile          : > CLA1,         PAGE = 1
   Cla1SoftIntRegsFile   : > PIE_CTRL,     PAGE = 1, type=DSECT

   ClbXbarRegsFile      : > CLB_XBAR     PAGE = 1

   Cmpss1RegsFile        : > CMPSS1,      PAGE = 1
   Cmpss2RegsFile        : > CMPSS2,      PAGE = 1
   Cmpss3RegsFile        : > CMPSS3,      PAGE = 1
   Cmpss4RegsFile        : > CMPSS4,      PAGE = 1
   Cmpss5RegsFile        : > CMPSS5,      PAGE = 1
   Cmpss6RegsFile        : > CMPSS6,      PAGE = 1
   Cmpss7RegsFile        : > CMPSS7,      PAGE = 1
   Cmpss8RegsFile        : > CMPSS8,      PAGE = 1

   CpuTimer0RegsFile     : > CPU_TIMER0,    PAGE = 1
   CpuTimer1RegsFile     : > CPU_TIMER1,    PAGE = 1
   CpuTimer2RegsFile     : > CPU_TIMER2,    PAGE = 1

   DacaRegsFile          : > DACA          PAGE = 1
   DacbRegsFile          : > DACB          PAGE = 1
   DaccRegsFile          : > DACC          PAGE = 1

   DcsmZ1RegsFile        : > DCSM_Z1,          PAGE = 1
   DcsmZ2RegsFile        : > DCSM_Z2,          PAGE = 1
   DcsmCommonRegsFile    : > DCSM_COMMON,      PAGE = 1

   /*** Warning:  Only remove "Type = NOLOAD" to program OTP Locations ***/
   DcsmZ1OtpFile         : > DCSM_Z1_OTP,      PAGE = 1, type = NOLOAD
   DcsmZ2OtpFile         : > DCSM_Z2_OTP,      PAGE = 1, type = NOLOAD

   DmaRegsFile           : > DMA           PAGE = 1
   DmaClaSrcSelRegsFile  : > DMACLASRCSEL  PAGE = 1

   ECap1RegsFile         : > ECAP1,        PAGE = 1
   ECap2RegsFile         : > ECAP2,        PAGE = 1
   ECap3RegsFile         : > ECAP3,        PAGE = 1
   ECap4RegsFile         : > ECAP4,        PAGE = 1
   ECap5RegsFile         : > ECAP5,        PAGE = 1
   ECap6RegsFile         : > ECAP6,        PAGE = 1

   Emif1RegsFile         : > EMIF1         PAGE = 1
   Emif2RegsFile         : > EMIF2         PAGE = 1

   EPwm1RegsFile         : > EPWM1,        PAGE = 1
   EPwm2RegsFile         : > EPWM2,        PAGE = 1
   EPwm3RegsFile         : > EPWM3,        PAGE = 1
   EPwm4RegsFile         : > EPWM4,        PAGE = 1
   EPwm5RegsFile         : > EPWM5,        PAGE = 1
   EPwm6RegsFile         : > EPWM6,        PAGE = 1
   EPwm7RegsFile         : > EPWM7,        PAGE = 1
   EPwm8RegsFile         : > EPWM8,        PAGE = 1
   EPwm9RegsFile         : > EPWM9,        PAGE = 1
   EPwm10RegsFile        : > EPWM10,       PAGE = 1
   EPwm11RegsFile        : > EPWM11,       PAGE = 1
   EPwm12RegsFile        : > EPWM12,       PAGE = 1

   EPwmXbarRegsFile      : > EPWM_XBAR     PAGE = 1

   EQep1RegsFile         : > EQEP1,        PAGE = 1
   EQep2RegsFile         : > EQEP2,        PAGE = 1
   EQep3RegsFile         : > EQEP3,        PAGE = 1

   Flash0CtrlRegsFile     : > FLASH0_CTRL    PAGE = 1
   Flash0EccRegsFile      : > FLASH0_ECC     PAGE = 1

   GpioCtrlRegsFile      : > GPIOCTRL,     PAGE = 1
   GpioDataRegsFile      : > GPIODAT,      PAGE = 1

   OutputXbarRegsFile    : > OUTPUT_XBAR    PAGE = 1
   I2caRegsFile          : > I2CA,          PAGE = 1
   I2cbRegsFile          : > I2CB,          PAGE = 1
   InputXbarRegsFile     : > INPUT_XBAR     PAGE = 1
   XbarRegsFile          : > XBAR           PAGE = 1
   IpcRegsFile           : > IPC,           PAGE = 1

   FlashPumpSemaphoreRegsFile   : > FLASHPUMPSEMAPHORE,    PAGE = 1

   RomPrefetchRegsFile       : > ROMPREFETCH,       PAGE = 1
   MemCfgRegsFile            : > MEMCFG,            PAGE = 1
   Emif1ConfigRegsFile       : > EMIF1CONFIG,       PAGE = 1
   Emif2ConfigRegsFile       : > EMIF2CONFIG,       PAGE = 1
   AccessProtectionRegsFile  : > ACCESSPROTECTION,  PAGE = 1
   MemoryErrorRegsFile       : > MEMORYERROR,       PAGE = 1
   RomWaitStateRegsFile      : > ROMWAITSTATE,      PAGE = 1

   McbspaRegsFile        : > MCBSPA,       PAGE = 1
   McbspbRegsFile        : > MCBSPB,       PAGE = 1

   UppRegsFile           : > UPP,          PAGE = 1

   NmiIntruptRegsFile    : > NMIINTRUPT,   PAGE = 1
   PieCtrlRegsFile       : > PIE_CTRL,     PAGE = 1

   SciaRegsFile          : > SCIA,         PAGE = 1
   ScibRegsFile          : > SCIB,         PAGE = 1
   ScicRegsFile          : > SCIC,         PAGE = 1
   ScidRegsFile          : > SCID,         PAGE = 1

   Sdfm1RegsFile         : > SDFM1,        PAGE = 1
   Sdfm2RegsFile         : > SDFM2,        PAGE = 1

   SpiaRegsFile          : > SPIA,        PAGE = 1
   SpibRegsFile          : > SPIB,        PAGE = 1
   SpicRegsFile          : > SPIC,        PAGE = 1
   SpidRegsFile          : > SPID,        PAGE = 1

   DevCfgRegsFile        : > DEV_CFG,     PAGE = 1
   ClkCfgRegsFile        : > CLK_CFG,     PAGE = 1
   CpuSysRegsFile        : > CPU_SYS,     PAGE = 1

   SyncSocRegsFile       : > SYNC_SOC,    PAGE = 1

   WdRegsFile            : > WD,           PAGE = 1


   XintRegsFile          : > XINT          PAGE = 1
   MemCfgRegs            : > MEMCFG        PAGE = 1
}


/*
//===========================================================================
// End of file.
//===========================================================================
*/
