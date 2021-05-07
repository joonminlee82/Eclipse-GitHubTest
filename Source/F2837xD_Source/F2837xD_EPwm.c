//###########################################################################
//
// FILE:   F2837xD_EPwm.c
//
// TITLE:  F2837xD EPwm Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v160 $
// $Release Date: Mon Jun 15 13:36:23 CDT 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File

//---------------------------------------------------------------------- by SNU(JS)
#include "Init_Func_Prototypes.h"
#include "Variable.h"
// TBCLK 의 클럭 입력이 28335나, 28345와 다른점이 있다.
// 기존에는 시스템 클럭(sysclk)가 바로 EPWM의 프리스케일러로 입력 되었는데, 28377에서는 ClkCfgRegs.PERCLKDIVSEL[EPWMCLKDIV]를 이용하여
// 먼저 한번 분주 시킨 클럭(여기서 EPWMCLK 라 정의)를 EPWM 프리스켈러 입력으로 들어가 최종적으로 카운터에 사용되는 클럭 TBCLK를 만든다.
// ClkCfgRegs.PERCLKDIVSEL[EPWMCLKDIV] 의 디폴트 값은 '1' 인 /2 가 기본적으로 설정이 되어 있다.
// 따라서 EPWM에 사용되는 클럭을 가장 빠른 클럭(시스템클럭)을 상용 하고자 한다면 ClkCfgRegs.PERCLKDIVSEL[EPWMCLKDIV] 의 값을 0으로 바꾸어야 한다.
// 이 프로젝트에서는 PWM의 분해능을 최대한 확보하기 위해 ClkCfgRegs.PERCLKDIVSEL[EPWMCLKDIV] 값을 InitPeripheralClocks() 함수 안에서 1로 설정해 준다.


// Configure the period for each timer (using UP Down mode)
//maxCount_ePWM 가 결정

// define Dead Band Values
//TBCLK = SYSTEMCLK = 200MHz
//Dead Band = 3us * 200MHz = 900
//Maximum EPWM_DB 	1023(10bit)

//#define EPWM_DB   400 // 2us
#define EPWM_DB	600 // 3us
//#define EPWM_DB	700 // 3.5us
//#define EPWM_DB		100 // 0.5us
//#define EPWM_DB	200 // 1us
//#define EPWM_DB	1000 // 5us
//#define EPWM_DB	900 // 4.5us
#define ACT_High     1
#define ACT_Low      0

void Config_EPwm(volatile struct EPWM_REGS *EPwmXRegs, Uint16 Timer_PRD, Uint16 Master, Uint16 INT_EN_Num, Uint16 INT_sel, Uint16 Act_level);
#define Sync_Master 1
#define Sync_Slave 0
#define INT_ENABLE 1
#define INT_DISABLE 0

void InitEPwm(void)
{
	Uint16 INT_sel_PWM = ET_CTR_PRD;

	halfCount_ePWM = maxCount_ePWM>>1;
	maxCount_ePWM =	halfCount_ePWM<<1;
	if(SAMPING_METHOD == 1){ //single sampling
		//INT_sel_PWM = ET_CTR_PRD;
		INT_sel_PWM = ET_CTR_ZERO;
		//Tsamp = __divf32( (2.*(float)maxCount_ePWM*PWM_INT_NUM), SYS_CLK);
		Tsamp = 1./((SWITCHING_FRQ/PWM_INT_NUM)*1);
	}
	else if(SAMPING_METHOD == 2){ //double sampling
		INT_sel_PWM = ET_CTR_PRDZERO;
		//Tsamp = __divf32( (float)maxCount_ePWM*PWM_INT_NUM, SYS_CLK);
		Tsamp = 1./((SWITCHING_FRQ/PWM_INT_NUM)*2);
	}
	else{;} //error LED 표시 할까????
	INV_Tsamp = __divf32( 1., Tsamp);

	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
	EDIS;

	//INV1
	//Config_EPwm(&EPwm1Regs, maxCount_ePWM, Sync_Master, PWM_INT_NUM , INT_sel_PWM, ACT_High); //INT_ENABLE
	//Config_EPwm(&EPwm2Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_High);
	//Config_EPwm(&EPwm3Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_High);

	//INV2
	//Config_EPwm(&EPwm4Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_High);
	//Config_EPwm(&EPwm5Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_High);
	//Config_EPwm(&EPwm6Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_High);

	//INV3
	//Config_EPwm(&EPwm7Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_High);
	
	//INV1
	Config_EPwm(&EPwm1Regs, maxCount_ePWM, Sync_Master, PWM_INT_NUM , INT_sel_PWM, ACT_Low); //INT_ENABLE
	Config_EPwm(&EPwm2Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_Low);
	Config_EPwm(&EPwm3Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_Low);

	//INV2
	Config_EPwm(&EPwm4Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_Low);
	Config_EPwm(&EPwm5Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_Low);
	Config_EPwm(&EPwm6Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_Low);

	//INV3
	Config_EPwm(&EPwm7Regs, maxCount_ePWM, Sync_Slave, INT_DISABLE, INT_sel_PWM, ACT_Low);
	
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
	EDIS;
}

void Config_EPwm(volatile struct EPWM_REGS *EPwmXRegs, Uint16 Timer_PRD, Uint16 Master, Uint16 INT_EN_Num, Uint16 INT_sel, Uint16 Act_level){
	EALLOW;
  //--Setup TBCLK
	EPwmXRegs->TBPRD = Timer_PRD;          // Set timer period (TBCLKs)
	EPwmXRegs->TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
	EPwmXRegs->TBCTR = 0x0000;                     // Clear counter

  //--Set Compare values
	EPwmXRegs->CMPA.bit.CMPA = Timer_PRD>>1;    // Set compare A value
	EPwmXRegs->CMPB.bit.CMPB = Timer_PRD>>1;    // Set Compare B value

  //--Setup counter mode
	if(Master){  //for master (carrier sync)
		EPwmXRegs->TBCTL.all = 0x2012;  //0b0010 0000 0001 0010
		//EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and douwn
		//EPwm1Regs.TBCTL.bit.FREE_SOFT = 0; //?
		//EPwm1Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1;     // Clock ratio to SYSCLKOUT
		//EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
		//EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;				//1: Count Up after the synchronization event.
		//EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;				//0: Do not load the time-base counter (TBCTR) from the time-base phase register (TBPHS).
		//EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
		//EPwm1Regs.TBCTL.bit.SWFSYNC	= 0; 			//0: Writing a 0 has no effect and reads always return a 0.
		//EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	}
	else{  //for slave (carrier sync)
		EPwmXRegs->TBCTL.all = 0x2006;  //0b0010 0000 0000 0110
		//EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and douwn
		//EPwm1Regs.TBCTL.bit.FREE_SOFT = 0; //?
		//EPwm1Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1;     // Clock ratio to SYSCLKOUT
		//EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
		//EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;				//0: Count down after the synchronization event.
		//EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;				//0: Do not load the time-base counter (TBCTR) from the time-base phase register (TBPHS).
		//EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
		//EPwm1Regs.TBCTL.bit.SWFSYNC	= 0; 			//0: Writing a 0 has no effect and reads always return a 0.
		//EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	}

  //--Setup shadowing
  //EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  //EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwmXRegs->CMPCTL.bit.LOADAMODE = CC_CTR_PRD; // Load on PRD
	EPwmXRegs->CMPCTL.bit.LOADBMODE = CC_CTR_PRD;

  	//--Set actions
	if(Act_level==ACT_High){
		EPwmXRegs->AQCTLA.bit.CAU = AQ_SET;            // Clear PWM1A on event A, up count
		EPwmXRegs->AQCTLA.bit.CAD = AQ_CLEAR;              // Set PWM1A on event A, down count
	}else if(Act_level==ACT_Low){
		EPwmXRegs->AQCTLA.bit.CAU = AQ_CLEAR;            // Clear PWM1A on event A, up count
		EPwmXRegs->AQCTLA.bit.CAD = AQ_SET;              // Set PWM1A on event A, down count
	}else{
		EPwmXRegs->AQCTLA.bit.CAU = AQ_SET;            // Clear PWM1A on event A, up count
		EPwmXRegs->AQCTLA.bit.CAD = AQ_CLEAR;              // Set PWM1A on event A, down count
	}
  //EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
  //EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  //EPwm1Regs.AQCTLA2.bit.T1D  특정 이벤트 에서 출력 결정 (폴트 설정일듯)

  //--Active low complemetary(ALC)
	EPwmXRegs->DBCTL.all = 0x0007; //0b 0000 0000 0000 0111
	//InMode = ePWMxA
	//DEDB_MODE = 0
	//OutMode = Full enable

	//------active high, low, 결정
	if(Act_level==ACT_High){
		EPwmXRegs->DBCTL.bit.POLSEL = DB_ACTV_LOC;
	}
	else if(Act_level==ACT_Low){
		EPwmXRegs->DBCTL.bit.POLSEL = DB_ACTV_HIC;
	}
	else{//defualt : active low
		EPwmXRegs->DBCTL.bit.POLSEL = DB_ACTV_HIC;
	}
	//EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;, 700
	//EPwm1Regs.DBCTL.bit.OUTSWAP	 = 0;
	EPwmXRegs->DBRED.bit.DBRED = EPWM_DB;
	EPwmXRegs->DBFED.bit.DBFED = EPWM_DB;

	//--Trip Zone Selection.
	//EPwmXRegs->TZSEL.all = 0xd000; //0b 1101 0000 0000 0000
	EPwmXRegs->TZSEL.bit.OSHT1 = TZ_DISABLE; 	//TZ1 one shot (default : 0)
	EPwmXRegs->TZSEL.bit.OSHT2 = TZ_DISABLE; 	//TZ2 one shot (default : 0)
	EPwmXRegs->TZSEL.bit.OSHT3 = TZ_DISABLE; 	//TZ3 one shot (default : 0)
	EPwmXRegs->TZSEL.bit.OSHT4 = TZ_DISABLE;	//TZ4(EQEPERR) one shot  (default : 0)
	EPwmXRegs->TZSEL.bit.OSHT5 = TZ_ENABLE; 	//TZ5(CLKFAIL) one shot
	EPwmXRegs->TZSEL.bit.OSHT6 = TZ_DISABLE;	//TZ6(EMUSTOP) one shot  (default : 0)
	EPwmXRegs->TZSEL.bit.DCAEVT1 = 0;			//TZ_ENABLE; //DCAEVT1 one shot  CMPSS, GATE (TRIP 4)
	EPwmXRegs->TZSEL.bit.DCBEVT1 = 1;			//TZ_ENABLE; //DCBEVT1 one shot  ADC (TRIP 5)

	//--Trip Zone Control
	if(Act_level==ACT_High){
		EPwmXRegs->TZCTL.bit.TZA =  TZ_FORCE_HI;//트립 발생시, EPWMA 상태 : high
		EPwmXRegs->TZCTL.bit.TZB =  TZ_FORCE_HI;//트립 발생시, EPWMB 상태 : high
	}
	else if(Act_level==ACT_Low){
		EPwmXRegs->TZCTL.bit.TZA =  TZ_FORCE_LO;//트립 발생시, EPWMA 상태 : low
		EPwmXRegs->TZCTL.bit.TZB =  TZ_FORCE_LO;//트립 발생시, EPWMB 상태 : low
	}
	else{//defualt : active low
		EPwmXRegs->TZCTL.bit.TZA =  TZ_FORCE_HI;//트립 발생시, EPWMA 상태 : high
		EPwmXRegs->TZCTL.bit.TZB =  TZ_FORCE_HI;//트립 발생시, EPWMB 상태 : high
	}
	EPwmXRegs->TZCTL.bit.DCAEVT1 = 3;
	EPwmXRegs->TZCTL.bit.DCBEVT1 = 3;
	EPwmXRegs->TZCTL.bit.DCAEVT2 = 3;
	EPwmXRegs->TZCTL.bit.DCBEVT2 = 3;

	//--Enable TZ interrupt
	EPwmXRegs->TZEINT.bit.OST = 1; // 1: Enable EPWMx_TZINT PIE interrupt.(at OST EVT)

	//--CLEAR FLAG
	EPwmXRegs->TZCLR.all = 0x0005;// 0b 0000 0000 0000 0101
	//EPwm1Regs.TZCLR.bit.OST = 1;
	//EPwm1Regs.TZCLR.bit.INT = 1;

#if 0
	//--Digital Compare AH Trip Select
	//EPwmXRegs->DCAHTRIPSEL.all = 0x0018; //0b 0000 0000 0001 1000;
	//EPwmXRegs->DCAHTRIPSEL.bit.TRIPINPUT4 = 1; //CMPSS(3번 bit) :1 사용
	//EPwmXRegs->DCAHTRIPSEL.bit.TRIPINPUT5 = 1; //ADCEVT(4번 bit) :1 사용
	EPwmXRegs->DCAHTRIPSEL.bit.TRIPINPUT9 = 1; //ADCEVT A :1 사용
	EPwmXRegs->DCAHTRIPSEL.bit.TRIPINPUT10 = 1; //ADCEVT B :1 사용
	EPwmXRegs->DCAHTRIPSEL.bit.TRIPINPUT11 = 1; //ADCEVT C :1 사용
	EPwmXRegs->DCAHTRIPSEL.bit.TRIPINPUT12 = 1; //ADCEVT D :1 사용

	//--Digital Compare BH Trip Select
	//EPwmXRegs->DCBHTRIPSEL.all = 0x4040; //0b 0100 0000 0100 0000;
	EPwmXRegs->DCBHTRIPSEL.bit.TRIPINPUT7 = 1; //INPUT Xbar(6번 bit) (gate fault)
	EPwmXRegs->DCBHTRIPSEL.bit.TRIPINPUT15 = 1; //clock fail(14번 bit)

	//--Digital Compare Trip Select
	//EPwmXRegs->DCTRIPSEL.all = 0x0f0f;//0b0000 1111 0000 1111;
	EPwmXRegs->DCTRIPSEL.bit.DCAHCOMPSEL= 0xf; // Trip combination input (all trip inputs selected by DCAHTRIPSEL register ORed together)
	EPwmXRegs->DCTRIPSEL.bit.DCBHCOMPSEL= 0xf; // Trip combination input (all trip inputs selected by DCBHTRIPSEL register ORed together)

#else

	//--Digital Compare Trip Select
	//EPwmXRegs->DCTRIPSEL.all = 0x0f0f;//0b0000 1111 0000 1111;
	EPwmXRegs->DCTRIPSEL.bit.DCAHCOMPSEL= 0x3; //CMPSS, GATE (TRIP 4)
	EPwmXRegs->DCTRIPSEL.bit.DCBHCOMPSEL= 0x4; //ADC		 (TRIP 5)

#endif
	EPwmXRegs->DCACTL.bit.EVT1SRCSEL = 0;//source is DCAEVT1 signal
	EPwmXRegs->DCBCTL.bit.EVT1SRCSEL = 0;//source is DCBEVT1 signal

	EPwmXRegs->DCACTL.bit.EVT1FRCSYNCSEL = 1; //Source Is Asynchronous Signal: 1
	EPwmXRegs->DCBCTL.bit.EVT1FRCSYNCSEL = 1; //Source Is Asynchronous Signal: 1


	//--Trip Zone Digital Comparator Select
	//EPwmXRegs->TZDCSEL.all = 0x0082; //0b 0000 0000 1000 0010;
	EPwmXRegs->TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;//TZ_EVT_DISABLE;//TZ_DCAH_HI //010: DCAH = high, DCAL = don't care
	EPwmXRegs->TZDCSEL.bit.DCBEVT1 = TZ_DCAH_HI;	//010: DCBH = high, DCBL = don't care
	EPwmXRegs->TZDCSEL.bit.DCAEVT2 = TZ_EVT_DISABLE;//TZ_EVT_DISABLE;//TZ_DCAH_HI //010: DCAH = high, DCAL = don't care
	EPwmXRegs->TZDCSEL.bit.DCBEVT2 = TZ_EVT_DISABLE;	//010: DCBH = high, DCBL = don't care

	// Interrupt where we will change the Compare Values
	EPwmXRegs->ETSEL.bit.INTSEL = INT_sel;//ET_CTR_PRD;     // Select INT on Zero event (for single sampling)
	//EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRDZERO;//for double sampling 확인해 봐야 함
	if(INT_EN_Num > 0){
		EPwmXRegs->ETSEL.bit.INTEN = 1;                // Enable INT
		EPwmXRegs->ETPS.bit.INTPRD = INT_EN_Num;     // Generate INT on # event
	}
	else{
		EPwmXRegs->ETSEL.bit.INTEN = ET_DISABLE;        // Disable INT
		EPwmXRegs->ETPS.bit.INTPRD = ET_1ST;    		// Generate INT on 1st event

	}
	EDIS;
}


void InitEPwmGpio(void)
{
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitEPwm4Gpio();
	InitEPwm5Gpio();
	InitEPwm6Gpio();
	InitEPwm7Gpio();
	InitEPwm8Gpio();
	InitEPwm9Gpio();
	InitEPwm10Gpio();
	InitEPwm11Gpio();
	InitEPwm12Gpio();

}

void InitEPwm1Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO145 = 1;    // Disable pull-up on GPIO145 (EPWM1A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO146 = 1;    // Disable pull-up on GPIO146 (EPWM1B)
	

/* Configure EPWM-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
//	GpioCtrlRegs.GPEMUX2.bit.GPIO145 = 1;   // Configure GPIO145 as EPWM1A
//	GpioCtrlRegs.GPEMUX2.bit.GPIO146 = 1;   // Configure GPIO0146 as EPWM1B

    EDIS;
}

void InitEPwm2Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO147 = 1;    // Disable pull-up on GPIO147 (EPWM2A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO148 = 1;    // Disable pull-up on GPIO148 (EPWM2B)

/* Configure EPwm-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
//	GpioCtrlRegs.GPEMUX2.bit.GPIO147 = 1;   // Configure GPIO147 as EPWM2A
//	GpioCtrlRegs.GPEMUX2.bit.GPIO148 = 1;   // Configure GPIO148 as EPWM2B

    EDIS;
}

void InitEPwm3Gpio(void)
{
   EALLOW;

/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO149 = 1;    // Disable pull-up on GPIO149 (EPWM3A)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO150 = 1;    // Disable pull-up on GPIO150 (EPWM3B)

/* Configure EPwm-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
//	GpioCtrlRegs.GPEMUX2.bit.GPIO149 = 1;   // Configure GPIO149 as EPWM3A
//	GpioCtrlRegs.GPEMUX2.bit.GPIO150 = 1;   // Configure GPIO150 as EPWM3B

    EDIS;
}

void InitEPwm4Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    //GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    //GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7 (EPWM4B)
 	GpioCtrlRegs.GPEPUD.bit.GPIO151 = 1;    // Disable pull-up on GPIO151 (EPWM4A)
 	GpioCtrlRegs.GPEPUD.bit.GPIO152 = 1;    // Disable pull-up on GPIO152 (EPWM4B)

/* Configure EPWM-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM4 functional pins.
// Comment out other unwanted lines.

    //GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
	GpioCtrlRegs.GPEMUX2.bit.GPIO151 = 1;   // Configure GPIO151 as EPWM4A
	GpioCtrlRegs.GPEMUX2.bit.GPIO152 = 1;   // Configure GPIO152 as EPWM4B

    EDIS;
}

void InitEPwm5Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO8 (EPWM5A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up on GPIO9 (EPWM5B)
 	GpioCtrlRegs.GPEPUD.bit.GPIO153 = 1;    // Disable pull-up on GPIO153 (EPWM5A)
 	GpioCtrlRegs.GPEPUD.bit.GPIO154 = 1;    // Disable pull-up on GPIO154 (EPWM5B)

/* Configure EPWM-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM5 functional pins.
// Comment out other unwanted lines.

//    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B
	GpioCtrlRegs.GPEMUX2.bit.GPIO153 = 1;   // Configure GPIO153 as EPWM5A
	GpioCtrlRegs.GPEMUX2.bit.GPIO154 = 1;   // Configure GPIO0154 as EPWM5B

    EDIS;
}

void InitEPwm6Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up on GPIO10 (EPWM6A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up on GPIO11 (EPWM6B)
 	GpioCtrlRegs.GPEPUD.bit.GPIO155 = 1;    // Disable pull-up on GPIO155 (EPWM6A)
 	GpioCtrlRegs.GPEPUD.bit.GPIO156 = 1;    // Disable pull-up on GPIO156 (EPWM6B)

/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

//    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
		GpioCtrlRegs.GPEMUX2.bit.GPIO155 = 1;   // Configure GPIO155 as EPWM6A
		GpioCtrlRegs.GPEMUX2.bit.GPIO156 = 1;   // Configure GPIO156 as EPWM6B

    EDIS;
}

void InitEPwm7Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

 //   GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;    // Disable pull-up on GPIO12 (EPWM7A)
 //   GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;    // Disable pull-up on GPIO13 (EPWM7B)
 	GpioCtrlRegs.GPEPUD.bit.GPIO157 = 1;    // Disable pull-up on GPIO157 (EPWM7A)
 	GpioCtrlRegs.GPEPUD.bit.GPIO158 = 1;    // Disable pull-up on GPIO158 (EPWM7B)

/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

 //   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;   // Configure GPIO12 as EPWM7A
 //   GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;   // Configure GPIO13 as EPWM7B
		GpioCtrlRegs.GPEMUX2.bit.GPIO157 = 1;   // Configure GPIO157 as EPWM7A
		GpioCtrlRegs.GPEMUX2.bit.GPIO158 = 1;   // Configure GPIO158 as EPWM7B

    EDIS;
}

void InitEPwm8Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;    // Disable pull-up on GPIO14 (EPWM8A)
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;    // Disable pull-up on GPIO15 (EPWM8B)
// 	GpioCtrlRegs.GPEPUD.bit.GPIO159 = 1;    // Disable pull-up on GPIO159 (EPWM8A)
//  GpioCtrlRegs.GPFPUD.bit.GPIO160 = 1;    // Disable pull-up on GPIO160 (EPWM8B)

/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;   // Configure GPIO14 as EPWM8A
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;   // Configure GPIO15 as EPWM8B
	//	GpioCtrlRegs.GPEMUX2.bit.GPIO159 = 1;   // Configure GPIO159 as EPWM8A
	//	GpioCtrlRegs.GPFMUX1.bit.GPIO160 = 1;   // Configure GPIO160 as EPWM8B

    EDIS;
}

void InitEPwm9Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPFPUD.bit.GPIO161 = 1;    // Disable pull-up on GPIO161 (EPWM9A)
    GpioCtrlRegs.GPFPUD.bit.GPIO162 = 1;    // Disable pull-up on GPIO162 (EPWM9B)

/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

	GpioCtrlRegs.GPFMUX1.bit.GPIO161 = 1;   // Configure GPIO161 as EPWM9A
	GpioCtrlRegs.GPFMUX1.bit.GPIO162 = 1;   // Configure GPIO162 as EPWM9B

    EDIS;
}

void InitEPwm10Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPFPUD.bit.GPIO163 = 1;    // Disable pull-up on GPIO163 (EPWM10A)
    GpioCtrlRegs.GPFPUD.bit.GPIO164 = 1;    // Disable pull-up on GPIO164 (EPWM10B)

/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

GpioCtrlRegs.GPFMUX1.bit.GPIO163 = 1;   // Configure GPIO163 as EPWM10A
GpioCtrlRegs.GPFMUX1.bit.GPIO164 = 1;   // Configure GPIO164 as EPWM10B

    EDIS;
}

void InitEPwm11Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPFPUD.bit.GPIO165 = 1;    // Disable pull-up on GPIO165 (EPWM11A)
    GpioCtrlRegs.GPFPUD.bit.GPIO166 = 1;    // Disable pull-up on GPIO166 (EPWM11B)

/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

GpioCtrlRegs.GPFMUX1.bit.GPIO165 = 1;   // Configure GPIO165 as EPWM11A
GpioCtrlRegs.GPFMUX1.bit.GPIO166 = 1;   // Configure GPIO166 as EPWM11B

    EDIS;
}

void InitEPwm12Gpio(void)
{
   EALLOW;
/* Disable internal pull-up for the selected output pins
   for reduced power consumption */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPFPUD.bit.GPIO167 = 1;    // Disable pull-up on GPIO167 (EPWM12A)
    GpioCtrlRegs.GPFPUD.bit.GPIO168 = 1;    // Disable pull-up on GPIO168 (EPWM12B)

/* Configure EPWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM6 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPFMUX1.bit.GPIO167 = 1;   // Configure GPIO167 as EPWM12A
    GpioCtrlRegs.GPFMUX1.bit.GPIO168 = 1;   // Configure GPIO168 as EPWM12B

    EDIS;
}
