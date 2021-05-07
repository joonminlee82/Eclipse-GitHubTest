//###########################################################################
// FILE:   F2837xD_Adc.c
// TITLE:  F2837xD Adc Support Functions.
//###########################################################################
// $TI Release: F2837xD Support Library v160 $
// $Release Date: Mon Jun 15 13:36:23 CDT 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File
#include "Variable.h"

//28377 revision 0,A,B 의 제기된 문제 (Erreta 참고)
//Random Conversion Errors의 확율을 낮추기 위해 해야 하는 것들
//1. S+H 시간 최소 : 320ns
//2. ADCCLK : 40MHz 이하 (전기적으로 최대 50MHz까지 쓸수 있지만, 확율을 낮추기 위해 40MHz로 사용 해야 함)
//3. ADCCLK의 prescale 을 정수로(ex : /1,/2,/3,/4,/5,/6,/7,/8)
//4. 메모리 0x0000743F, 0x000074BF, 0x0000753F, 0x000075BF 위치에 0X7000값을 쓴다
void Config_16bit_Adc(volatile struct ADC_REGS *AdcXRegs, int ADC_num, int SOC0_ch,  int SOC1_ch,  int SOC2_ch);

void InitAdc(void){

	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	Config_16bit_Adc(&AdcaRegs,ADC_ADCA,2,0,4);
	Config_16bit_Adc(&AdcbRegs,ADC_ADCB,2,0,4);
	Config_16bit_Adc(&AdccRegs,ADC_ADCC,2,4,0);
	Config_16bit_Adc(&AdcdRegs,ADC_ADCD,0,2,4);

}

void Config_16bit_Adc(volatile struct ADC_REGS *AdcXRegs, int ADC_num, int SOC0_ch,  int SOC1_ch,  int SOC2_ch){
	Uint16 acqps;

	EALLOW;																// Enable EALLOW protected register access
	//--- Reset the ADC.  This is good programming practice.
	DevCfgRegs.SOFTPRES13.all = (1<<ADC_num);							// ADC is reset
	DevCfgRegs.SOFTPRES13.all = 0;										// ADC is released from reset

	//--- Configure the ADC base registers
	AdcXRegs->ADCCTL1.bit.ADCPWDNZ = 0;    								// ADCPWDNZ, ADC power down, 0=powered down, 1=powered up
	AdcXRegs->ADCCTL1.bit.INTPULSEPOS = 1; 								// INTPULSEPOS, INT pulse generation, 0=start of conversion, 1=end of conversion

	//--- ADC clock configuration ( 16bit 기준,  149+64 sysclk -> SOC 부터 총 1.065us 걸림)
	AdcXRegs->ADCCTL2.bit.PRESCALE = 0x8;								// PRESCALE, ADC clock prescaler.  1000=CPUCLK/5 : 40MHz, 50MHz is maxium clk for ADC
	AdcXRegs->rsvd5[3] = 0x7000;  										// for silicon revision B

	AdcXRegs->ADCCTL2.bit.RESOLUTION =ADC_RESOLUTION_16BIT;     		//RESOLUTION, 0= 12BIT, 1=16 BIT, configured by AdcSetMode() below to get calibration correct
	AdcXRegs->ADCCTL2.bit.SIGNALMODE =ADC_SIGNALMODE_DIFFERENTIAL;		//SIGNALMODE, 0= SINGLE-END, 1=DIFFERENTIAL, configured by AdcSetMode() below to get calibration correct


	//--- Call AdcSetMode() to configure the resolution and signal mode.
	//    This also performs the correct ADC calibration for the configured mode.
    AdcSetMode(ADC_num, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_DIFFERENTIAL);

	//--- SOC0 configuration
    acqps = 63; //63 : 320ns (16bit mode minimun duration time), 14 : 75ns (12bit minimun duration time)

    AdcXRegs->ADCSOC0CTL.bit.TRIGSEL 	= 0;							// Trigger : Software only
    AdcXRegs->ADCSOC0CTL.bit.CHSEL 		= SOC0_ch;						// Convert channel (ADCIN0 (non-inverting) and ADCIN1 (inverting) )
    AdcXRegs->ADCSOC0CTL.bit.ACQPS 		= acqps;						// Acquisition window (셈플링 시간 관련)

    AdcXRegs->ADCSOC1CTL.bit.TRIGSEL 	= 0;							// Trigger : Software only
    AdcXRegs->ADCSOC1CTL.bit.CHSEL 		= SOC1_ch;						// Convert channel (ADCIN2 (non-inverting) and ADCIN3 (inverting) )
    AdcXRegs->ADCSOC1CTL.bit.ACQPS 		= acqps;						// Acquisition window (셈플링 시간 관련)

    AdcXRegs->ADCSOC2CTL.bit.TRIGSEL 	= 0;							// Trigger : Software only
    AdcXRegs->ADCSOC2CTL.bit.CHSEL 		= SOC2_ch;						// Convert channel (ADCIN4 (non-inverting) and ADCIN5 (inverting) )
    AdcXRegs->ADCSOC2CTL.bit.ACQPS 		= acqps;						// Acquisition window (셈플링 시간 관련)

	//AdcaRegs.ADCINTSOCSEL1.bit.SOC0 = ;								// ADC interrupt triggers 안씀
	//AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 0x10;						// SOC priority mode : default setting (SOC0)

	//--- ADCA interrupt configuration
    AdcXRegs->ADCINTSEL1N2.bit.INT1SEL 	= 0; 							// ADCINT1 -> SOC0
    AdcXRegs->ADCINTSEL1N2.bit.INT1E	= 1; 							// ADCINT1 Enable
    AdcXRegs->ADCINTFLGCLR.bit.ADCINT1 	= 1; 							// make sure INT1 flag is cleared

    AdcXRegs->ADCINTSEL1N2.bit.INT2SEL 	= 1; 							// ADCINT2 -> SOC1
    AdcXRegs->ADCINTSEL1N2.bit.INT2E	= 1; 							// ADCINT2 Enable
    AdcXRegs->ADCINTFLGCLR.bit.ADCINT2 	= 1; 							// make sure INT1 flag is cleared

    AdcXRegs->ADCINTSEL3N4.bit.INT3SEL 	= 2; 							// ADCINT3 -> SOC2
    AdcXRegs->ADCINTSEL3N4.bit.INT3E	= 1; 							// ADCINT3 Enable
    AdcXRegs->ADCINTFLGCLR.bit.ADCINT3 	= 1; 							// make sure INT1 flag is cleared

    //AdcXRegs->ADCINTSEL3N4.bit.INT4SEL 	= 5; 						// ADCINT4 -> SOC5
    //AdcXRegs->ADCINTSEL3N4.bit.INT4E		= 1; 						// ADCINT4 Enable
    //AdcXRegs->ADCINTFLGCLR.bit.ADCINT4 	= 1; 						//make sure INT1 flag is cleared

	//--- PPB
    AdcXRegs->ADCPPB1CONFIG.bit.CONFIG = 0; 							//SOC0
    AdcXRegs->ADCPPB2CONFIG.bit.CONFIG = 1; 							//SOC1
    AdcXRegs->ADCPPB3CONFIG.bit.CONFIG = 2; 							//SOC2
    AdcXRegs->ADCPPB4CONFIG.bit.CONFIG = 3; 							//초기값이 0(SOC0) 이기 때문에 안전을 위해 SOC3을 향하도록 한다. PPB4는 실제로 사용하지 않는다.
                                            							
    AdcXRegs->ADCPPB1OFFCAL.bit.OFFCAL = 0; 							//초기값이 0 이지만, 확실히 하기 위해 한번더 서술
    AdcXRegs->ADCPPB2OFFCAL.bit.OFFCAL = 0; 							//초기값이 0 이지만, 확실히 하기 위해 한번더 서술
    AdcXRegs->ADCPPB3OFFCAL.bit.OFFCAL = 0; 							//초기값이 0 이지만, 확실히 하기 위해 한번더 서술

	// SW fault Enable -> trip signal propagation
   	//AdcXRegs->ADCEVTSEL.all = 0x0333; //0b 0000 0011 0011 0011
    AdcXRegs->ADCEVTSEL.all = 0x0000; //0b 0000 0000 0000 0000
  	//AdcXRegs->ADCEVTSEL.bit.PPB1TRIPHI = 1;
  	//AdcXRegs->ADCEVTSEL.bit.PPB1TRIPLO = 1;
  	//AdcXRegs->ADCEVTSEL.bit.PPB2TRIPHI = 1;
  	//AdcXRegs->ADCEVTSEL.bit.PPB2TRIPLO = 1;
	//AdcXRegs->ADCEVTSEL.bit.PPB3TRIPHI = 1;
	//AdcXRegs->ADCEVTSEL.bit.PPB3TRIPLO = 1;
	//AdcXRegs->ADCEVTSEL.bit.PPB4TRIPHI = 0;
	//AdcXRegs->ADCEVTSEL.bit.PPB4TRIPLO = 0;

 	//AdcXRegs->ADCEVTCLR.bit.PPB1TRIPHI = 1;
	//AdcXRegs->ADCEVTCLR.bit.PPB1TRIPLO = 1;
	//AdcXRegs->ADCEVTCLR.bit.PPB2TRIPHI = 1;
	//AdcXRegs->ADCEVTCLR.bit.PPB2TRIPLO = 1;
	//AdcXRegs->ADCEVTCLR.bit.PPB3TRIPHI = 1;
	//AdcXRegs->ADCEVTCLR.bit.PPB3TRIPLO = 1;

	// SW fault Enable -> interrrupt signal propagation  ?? 확인
 	//AdcXRegs->ADCEVTINTSEL.all = 0x0333; //0b 0000 0011 0011 0011
	//AdcXRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
	//AdcXRegs.ADCEVTINTSEL.bit.PPB1TRIPLO = 1;
	//AdcXRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = 1;
	//AdcXRegs.ADCEVTINTSEL.bit.PPB2TRIPLO = 1;
	//AdcXRegs.ADCEVTINTSEL.bit.PPB3TRIPHI = 1;
	//AdcXRegs.ADCEVTINTSEL.bit.PPB3TRIPLO = 1;
	//--- Finish up
    AdcXRegs->ADCCTL1.bit.ADCPWDNZ = 1;			// Power up the ADC
	EDIS;										// Disable EALLOW protected register access

	DELAY_US(1000L);								// Wait 1 ms after power-up before using the ADC
}


// ADC End Of Convertion 기다리는 함수
void Wait_EOC_ADC_SOC0(void){
	while( (AdcaRegs.ADCINTFLG.bit.ADCINT1 & AdcbRegs.ADCINTFLG.bit.ADCINT1 & AdccRegs.ADCINTFLG.bit.ADCINT1 & AdcdRegs.ADCINTFLG.bit.ADCINT1)==0){;} //31cycle
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
	AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
}
void Wait_EOC_ADC_SOC1(void){
	while( (AdcaRegs.ADCINTFLG.bit.ADCINT2 & AdcbRegs.ADCINTFLG.bit.ADCINT2 & AdccRegs.ADCINTFLG.bit.ADCINT2 & AdcdRegs.ADCINTFLG.bit.ADCINT2)==0){;} //31cycle
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
	AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
}
void Wait_EOC_ADC_SOC2(void){
	while( (AdcaRegs.ADCINTFLG.bit.ADCINT3 & AdcbRegs.ADCINTFLG.bit.ADCINT3 & AdccRegs.ADCINTFLG.bit.ADCINT3 & AdcdRegs.ADCINTFLG.bit.ADCINT3)==0){;} //31cycle
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
	AdccRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
}
//---------------------------------------------------------------------- by SNU(JS)


/* 
* Set the resolution and signalmode for a given ADC. This will ensure that
* the correct trim is loaded. 
*/
void AdcSetMode(Uint16 adc, Uint16 resolution, Uint16 signalmode)
{
	Uint16 adcOffsetTrimOTPIndex; //index into OTP table of ADC offset trims
	Uint16 adcOffsetTrim; //temporary ADC offset trim
	
	//re-populate INL trim
	CalAdcINL(adc);	// Unconnected Pin 20170321 GWON
	
	if(0xFFFF != *((Uint16*)GetAdcOffsetTrimOTP)){
		//offset trim function is programmed into OTP, so call it

		//calculate the index into OTP table of offset trims and call
		//function to return the correct offset trim
		adcOffsetTrimOTPIndex = 4*adc + 2*resolution + 1*signalmode;
		adcOffsetTrim = (*GetAdcOffsetTrimOTP)(adcOffsetTrimOTPIndex);
	}
	else {
		//offset trim function is not populated, so set offset trim to 0
		adcOffsetTrim = 0;
	}

	//Apply the resolution and signalmode to the specified ADC.
	//Also apply the offset trim and, if needed, linearity trim correction.
	switch(adc){
		case ADC_ADCA:
			AdcaRegs.ADCCTL2.bit.RESOLUTION = resolution;
			AdcaRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
			AdcaRegs.ADCOFFTRIM.all = adcOffsetTrim;
			if(ADC_RESOLUTION_12BIT == resolution){

				//12-bit linearity trim workaround
				AdcaRegs.ADCINLTRIM1 &= 0xFFFF0000;
				AdcaRegs.ADCINLTRIM2 &= 0xFFFF0000;
				AdcaRegs.ADCINLTRIM4 &= 0xFFFF0000;
				AdcaRegs.ADCINLTRIM5 &= 0xFFFF0000;
			}
		break;
		case ADC_ADCB:
			AdcbRegs.ADCCTL2.bit.RESOLUTION = resolution;
			AdcbRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
			AdcbRegs.ADCOFFTRIM.all = adcOffsetTrim;
			if(ADC_RESOLUTION_12BIT == resolution){

				//12-bit linearity trim workaround
				AdcbRegs.ADCINLTRIM1 &= 0xFFFF0000;
				AdcbRegs.ADCINLTRIM2 &= 0xFFFF0000;
				AdcbRegs.ADCINLTRIM4 &= 0xFFFF0000;
				AdcbRegs.ADCINLTRIM5 &= 0xFFFF0000;
			}
		break;
		case ADC_ADCC:
			AdccRegs.ADCCTL2.bit.RESOLUTION = resolution;
			AdccRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
			AdccRegs.ADCOFFTRIM.all = adcOffsetTrim;
			if(ADC_RESOLUTION_12BIT == resolution){

				//12-bit linearity trim workaround
				AdccRegs.ADCINLTRIM1 &= 0xFFFF0000;
				AdccRegs.ADCINLTRIM2 &= 0xFFFF0000;
				AdccRegs.ADCINLTRIM4 &= 0xFFFF0000;
				AdccRegs.ADCINLTRIM5 &= 0xFFFF0000;
			}
		break;
		case ADC_ADCD:
			AdcdRegs.ADCCTL2.bit.RESOLUTION = resolution;
			AdcdRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
			AdcdRegs.ADCOFFTRIM.all = adcOffsetTrim;
			if(ADC_RESOLUTION_12BIT == resolution){

				//12-bit linearity trim workaround
				AdcdRegs.ADCINLTRIM1 &= 0xFFFF0000;
				AdcdRegs.ADCINLTRIM2 &= 0xFFFF0000;
				AdcdRegs.ADCINLTRIM4 &= 0xFFFF0000;
				AdcdRegs.ADCINLTRIM5 &= 0xFFFF0000;
			}
		break;
	}
}

/* 
* Loads INL trim values from OTP into the trim registers of the specified ADC.
* Use only as part of AdcSetMode function, since linearity trim correction
* is needed for some modes.
*/
void CalAdcINL(Uint16 adc)
{
	switch(adc){
		case ADC_ADCA:
			if(0xFFFF != *((Uint16*)CalAdcaINL)){
				//trim function is programmed into OTP, so call it
				(*CalAdcaINL)();
			}
			else {
				//do nothing, no INL trim function populated
			}
			break;
		case ADC_ADCB:
			if(0xFFFF != *((Uint16*)CalAdcbINL)){
				//trim function is programmed into OTP, so call it
				(*CalAdcbINL)();
			}
			else {
				//do nothing, no INL trim function populated
			}
			break;
		case ADC_ADCC:
			if(0xFFFF != *((Uint16*)CalAdccINL)){
				//trim function is programmed into OTP, so call it
				(*CalAdccINL)();
			}
			else {
				//do nothing, no INL trim function populated
			}
			break;
		case ADC_ADCD:
			if(0xFFFF != *((Uint16*)CalAdcdINL)){
				//trim function is programmed into OTP, so call it
				(*CalAdcdINL)();
			}
			else {
				//do nothing, no INL trim function populated
			}
			break;
	}
}
