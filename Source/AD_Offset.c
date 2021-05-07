/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: FREEWAY 
	Compiler    		: TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.07.18
	History				: Linear Motor Selection Menu is added into the HHT Menu (20180206 by GWON)
						: Motor Side R Controller is added in order to suppress 3th, 6th 9th harmonics, but this controllers are disable. (20180206 by GWON)
						: Mark to need to modify for FREEWAY --> Search command "// Need to modify for FREEWAY" (20180209 by GWON)
						: Modified something for Linear Motor. ex) pole_Pitch, polepair, ... etc., (20180417 by GWON)
						: Modified Position and Mechinical/Electrical Angle Calculation Function (20180418 by GWON)
						: Inserted Rotor Position Estimation Function (20180430 by GWON)
						: Profile Generation Function (20180508 by GWON)
						: Fault Definition & Setting (20180509 by GWON)
						: Added Profile Error Compansator (20180510 by GWON)
						: Added Control Code using HHT (20180629 by GWON)
						: Added Mechanical_Observer for FRLSM (20180718 by GWON)
						: Load Mass Estimator using Te for FRLSM (20180718 by GWON)
						: Modified MC Operation Condition. Over 540V --> Over UVDC_Set (20180718 by GWON)
						: STOP Mode Speed Max Value 0.1 --> 0.05 (20180718 by GWON)
******************************************************************************/
#include "AD_Offset.h"
#include "Variable.h"
#include "CC.h"
#include "Fault.h"

extern Flt	FLT ;

int offset_cnt =0;
__interrupt void cpu1_offset(void)
{
	/* Enable Fault Interrupt */
	IER &=0x0000;
	IER |=M_INT2;		// Fault INT Enable
	EINT;

	offset_cnt++;
	
	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)

	//forced SW ADC SOC -------------------------------------------
	AdcaRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1, SOC2 forced start
	AdcbRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1, SOC2 forced start
	AdccRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1, SOC2 forced start
	AdcdRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1, SOC2 forced start
	//forced SW ADC SOC ---------------------12 cycle-------------
	if(nPFO_15V == 1){
		//wait for EOC (SOC1)
		Wait_EOC_ADC_SOC1();
		if((Flag_offset == 0)&&(offsetLoopCnt >= 20000)){
			Flag_offset = 1;
			Offset_ADC_SOC1[0] = 0;
			Offset_ADC_SOC1[1] = 0;  
			Offset_ADC_SOC1[2] = 0;  
			Offset_ADC_SOC1[3] = 0;
			offsetLoopCnt = 0;
		}
		if(Flag_offset == 1){
			Offset_ADC_SOC0[0] += (int64)AdcaResultRegs.ADCRESULT0; //INV1.Ias  
			Offset_ADC_SOC0[1] += (int64)AdcbResultRegs.ADCRESULT0; //INV1.Ibs 
			Offset_ADC_SOC0[2] += (int64)AdccResultRegs.ADCRESULT0; //INV1.Ics
		}
		if((Flag_offset == 1)&&(offsetLoopCnt > offsetMaxCnt)){
			Offset_ADC_SOC0[0] = (int64)(((float)(Offset_ADC_SOC0[0]>>OFFSET_MAX_CNT))- OffsetAin_SOC0[0]/ScaleAin_SOC0[0]);
			Offset_ADC_SOC0[1] = (int64)(((float)(Offset_ADC_SOC0[1]>>OFFSET_MAX_CNT))- OffsetAin_SOC0[1]/ScaleAin_SOC0[1]);
			Offset_ADC_SOC0[2] = (int64)(((float)(Offset_ADC_SOC0[2]>>OFFSET_MAX_CNT))- OffsetAin_SOC0[2]/ScaleAin_SOC0[2]);

			if((labs(Offset_ADC_SOC0[0]) > 200)||(labs(Offset_ADC_SOC0[1]) > 200)||(labs(Offset_ADC_SOC0[2]) > 200)){
				FLT_Raise(FLT_OFFSET);
			}
			else
			{
				EALLOW;
				AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = (int)Offset_ADC_SOC0[0]&0x3ff;	//Inv.Ias
				AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = (int)Offset_ADC_SOC0[1]&0x3ff;	//Inv.Ibs
				AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = (int)Offset_ADC_SOC0[2]&0x3ff;	//Inv.Ics
				EDIS;
				Flag_offset = 2;
			}
			if ((FLT.uFlag == 1)&&(FLT.uFltNum == FLT_MOSFET_I)) FLT_Reset();	//check!!
			EALLOW;
			PieVectTable.EPWM1_INT = &cpu1_cc;
			EDIS;
		}
	}
	else{

		if(((labs(Offset_ADC_SOC0[0]) < 200)&&(labs(Offset_ADC_SOC0[0]) > 0))||((labs(Offset_ADC_SOC0[1]) < 200)&&(labs(Offset_ADC_SOC0[1]) > 0))
			||((labs(Offset_ADC_SOC0[2]) < 200)&&(labs(Offset_ADC_SOC0[2]) > 0))){
			EALLOW;
			AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = (int)Offset_ADC_SOC0[0]&0x3ff;	//Inv.Ibs
			AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = (int)Offset_ADC_SOC0[1]&0x3ff;	//Inv.Ics
			AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = (int)Offset_ADC_SOC0[2]&0x3ff;	//Inv.Ias
			EDIS;
			Flag_offset = 2;
		}
		if ((FLT.uFlag == 1)&&(FLT.uFltNum == FLT_MOSFET_I)) FLT_Reset();//check!!
			
		EALLOW;
		PieVectTable.EPWM1_INT = &cpu1_cc;
		EDIS;
	}
	offsetLoopCnt++;
		
  // Clear INT flag for this timer
  EPwm1Regs.ETCLR.bit.INT = 1;
  // Acknowledge this interrupt to receive more interrupts from group 3
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
  asm(" NOP");
}
