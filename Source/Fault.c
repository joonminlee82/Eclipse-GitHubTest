/*****************************************************************************
-------------------------------Organization------------------------------------
	Project				: FREEWAY
	Compiler    		: TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
	Author				: Inverter part, Advanced Controls Research Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.12.28
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
						: TX_INDEX 20 --> 40 (20180806 by GWON)
						: Added SCI Protocol Version Response Part (20181107 by GWON)
						: Added SCI Protocol Infomation Response Part (20181107 by GWON)
						: Rearrangement of F/W for FREEWAY (20181112 by GWON)
						: Added SPI Init Setting for between ARM and DSP communication (20181127 by GWON)
						: Added SPI Control Communication Part (20181130 by GWON)
						: Ver2 Board GPIO Setting (20181207 by GWON)
						: Verified SPI Protocol (20181224 by GWON & Shin)
******************************************************************************/
#include "Fault.h"
#include "CC.h"
#include "SC.h"
#include "Variable.h"
#include "drive.h"
#include "bdib.h"
#include "Device.h"
#include "KeyMenu.h"
#include "FRAM.h"
#include "SCI.h"
#include "SPI.h"
//#include "fc.h"
//#include "rmb.h"
#include "gnl.h"
#include "CPU1_CLA_Shared.h"

#pragma DATA_SECTION(Fault, "CLADataLS0");
#pragma DATA_SECTION(FLT, "CLADataLS0");
FAULT Fault;
Flt	FLT ;
struct FAULT_DSP_SIGNAL  Fault_DSP;

extern Suds	SUDS ;
extern STATUS_ARM_OBJ  g_SpiArmStatus;
extern BDIB            g_BDIB;
extern IdCoData        g_IdSpiCoData;

//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)

//Positive Trip
float Fault_Volt_A_SOC0_P = 0.;	// INV -Ias
float Fault_Volt_B_SOC0_P = 0.;	// INV -Ibs
float Fault_Volt_C_SOC0_P = 0.;	// INV -Ics
float Fault_Volt_D_SOC0_P = 0.; // Vdc
	                               
float Fault_Volt_A_SOC1_P = 0.;	// (no used)
float Fault_Volt_B_SOC1_P = 0.;	// (no used)
float Fault_Volt_C_SOC1_P = 0.;	// (no used)
float Fault_Volt_D_SOC1_P = 0.; // (no used)
	
float Fault_Volt_A_SOC2_P = 0.;	// (no used)
float Fault_Volt_B_SOC2_P = 0.;	// (no used)
float Fault_Volt_C_SOC2_P = 0.;	// (no used)
float Fault_Volt_D_SOC2_P = 0.; // (no used)
	
//Negative trip
float Fault_Volt_A_SOC0_N = 0.;	// INV -Ias
float Fault_Volt_B_SOC0_N = 0.;	// INV -Ibs
float Fault_Volt_C_SOC0_N = 0.;	// INV -Ics
float Fault_Volt_D_SOC0_N = 0.; // Vdc
	                               
float Fault_Volt_A_SOC1_N = 0.;	// (no used)
float Fault_Volt_B_SOC1_N = 0.;	// (no used)
float Fault_Volt_C_SOC1_N = 0.;	// (no used)
float Fault_Volt_D_SOC1_N = 0.; // (no used)
	
float Fault_Volt_A_SOC2_N = 0.;	// (no used)
float Fault_Volt_B_SOC2_N = 0.;	// (no used)
float Fault_Volt_C_SOC2_N = 0.;	// (no used)
float Fault_Volt_D_SOC2_N = 0.; // (no used) 
	
float Fault_Volt_SOC0_P[4] = {0., 0., 0., 0.};
float Fault_Volt_SOC1_P[4] = {0., 0., 0., 0.};
float Fault_Volt_SOC2_P[4] = {0., 0., 0., 0.};
	
float Fault_Volt_SOC0_N[4] = {0., 0., 0., 0.};
float Fault_Volt_SOC1_N[4] = {0., 0., 0., 0.};
float Fault_Volt_SOC2_N[4] = {0., 0., 0., 0.};

// Positive CMPSS MAP
// CMPIN1P : ADCINA2	(INV1.Ias)
// CMPIN3P : ADCINB2	(INV1.Ibs)
// CMPIN6P : ADCINC2	(INV1.Ics)
// CMPIN7P : ADCIND0	(Vdc)

//CMPSS Positive trip
float Fault_Volt_CMPSS1_P = 0.;   	// ADC A2 pin,(Inv1.Ias) 
//float Fault_Volt_CMPSS2_P = 0.;   // ADC A4 pin,
float Fault_Volt_CMPSS3_P = 0.;   	// ADC B2 pin,(Inv1.Ibs)
//float Fault_Volt_CMPSS4_P = 0.;   // ADC 14 pin,
//float Fault_Volt_CMPSS5_P = 0.;   // ADC C4 pin,
float Fault_Volt_CMPSS6_P = 0.;   	// ADC C2 pin,(Inv1.Ics)
float Fault_Volt_CMPSS7_P = 0.;   	// ADC D0 pin,(Vdc)
//float Fault_Volt_CMPSS8_P = 0.;   // ADC D2 pin,

// Negative CMPSS MAP 
// CMPIN1N : ADCINA3	(INV1.Ias)
// CMPIN3N : ADCINB3	(INV1.Ibs)
// CMPIN6N : ADCINC3	(INV1.Ics)
// CMPIN7N : ADCIND1	(Vdc)

//CMPSS Negative trip
float Fault_Volt_CMPSS1_N = 0.;   	// ADC A3 pin,(Inv1.Ias)
//float Fault_Volt_CMPSS2_N = 0.;   // ADC A5 pin,
float Fault_Volt_CMPSS3_N = 0.;   	// ADC B3 pin,(Inv1.Ibs)
//float Fault_Volt_CMPSS4_N = 0.;   // ADC 15 pin,
//float Fault_Volt_CMPSS5_N = 0.;   // ADC C5 pin,
float Fault_Volt_CMPSS6_N = 0.;   	// ADC C3 pin,(Inv1.Ics)
float Fault_Volt_CMPSS7_N = 0.;   	// ADC D1 pin,(Vdc)
//float Fault_Volt_CMPSS8_N = 0.;   // ADC D3 pin,
	
//ADC Fault Level (Positive)//16bit
Uint16 Fault_ADC_SOC0_P[4] = {0, 0, 0, 0};
Uint16 Fault_ADC_SOC1_P[4] = {0, 0, 0, 0};
Uint16 Fault_ADC_SOC2_P[4] = {0, 0, 0, 0};
	
//ADC Fault Level (Negative)
Uint16 Fault_ADC_SOC0_N[4] = {0, 0, 0, 0};
Uint16 Fault_ADC_SOC1_N[4] = {0, 0, 0, 0};
Uint16 Fault_ADC_SOC2_N[4] = {0, 0, 0, 0};
	
//CMPSS Fault Level (Positive)//12bit
Uint16 Fault_CMPSS_P[8] = {	0, 0, 0, 0, 0, 0, 0, 0};
Uint16 Fault_CMPSS_N[8] = {	0, 0, 0, 0, 0, 0, 0, 0};

Uint16 fault_cnt =0;
float HW_FaultGain = 1.1;
#pragma DATA_SECTION(fault_cnt, "CpuToCla1MsgRAM");

u8 g_ErrorCode = 0;

void InitFault(void)
{
//	CONV_Ibs_Flt_HW_Set = 45;		// KEY_GetFactoryCode(CON_OC_SET)*HW_FaultGain;	//CONV -Ibs
//	CONV_Ics_Flt_HW_Set	= 45;		// KEY_GetFactoryCode(CON_OC_SET)*HW_FaultGain;	//CONV -Ics
//	CONV_Ias_Flt_HW_Set	= 45;		// KEY_GetFactoryCode(CON_OC_SET)*HW_FaultGain;	//CONV -Ias
	Vdc_Flt_HW_Set 		= 840.;		// Vdc
	
	INV_Ibs_Flt_HW_Set 	= KEY_GetFactoryCode(INV_OC_SET)*HW_FaultGain;	//INV Ibs
	INV_Ics_Flt_HW_Set 	= KEY_GetFactoryCode(INV_OC_SET)*HW_FaultGain;	//INV Ics
	INV_Ias_Flt_HW_Set 	= KEY_GetFactoryCode(INV_OC_SET)*HW_FaultGain;	//INV Ias
		
//	GRID_Vbc_HW_Set 	= 680.;		// grid B-C
//	GRID_Vab_HW_Set 	= 680.;		// grid A-B

	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	//Positive Trip
	Fault_Volt_A_SOC0_P = (INV_Ias_Flt_HW_Set/SenScale_Iac_1_5kW)*Rm_Ain_Iac_1_5kW; // INV -Ias
	Fault_Volt_B_SOC0_P = (INV_Ibs_Flt_HW_Set/SenScale_Iac_1_5kW)*Rm_Ain_Iac_1_5kW; // INV -Ibs
	Fault_Volt_C_SOC0_P = (INV_Ics_Flt_HW_Set/SenScale_Iac_1_5kW)*Rm_Ain_Iac_1_5kW; // INV -Ics
	Fault_Volt_D_SOC0_P = (Vdc_Flt_HW_Set/SenScale_Vdc_1_5kW)*Rm_Ain_Vdc_1_5kW;     // Vdc
	                                                                                   
//	Fault_Volt_A_SOC1_P = (INV_Ibs_Flt_HW_Set/SenScale_Iac_30kVA)*Rm_Ain_Iac_30kVA;	
//	Fault_Volt_B_SOC1_P = (INV_Ics_Flt_HW_Set/SenScale_Iac_30kVA)*Rm_Ain_Iac_30kVA;	
//	Fault_Volt_C_SOC1_P = (INV_Ias_Flt_HW_Set/SenScale_Iac_30kVA)*Rm_Ain_Iac_30kVA;	
//	Fault_Volt_D_SOC1_P = (ESS_I_Flt_HW_Set/SenScale_ESS_Idc)*Rm_Ain_ESS_Idc; 		
	
//	Fault_Volt_A_SOC2_P = (GRID_Vbc_HW_Set/SenScale_Vac_40kW)*Rm_Ain_Vac_40kW;		
//	Fault_Volt_B_SOC2_P = (GRID_Vab_HW_Set/SenScale_Vac_40kW)*Rm_Ain_Vac_40kW;		
//	Fault_Volt_C_SOC2_P = (INV_Ias_Flt_HW_Set/SenScale_Iac_30kVA)*Rm_Ain_Iac_30kVA;	
//	Fault_Volt_D_SOC2_P = (ESS_V_Flt_HW_Set/SenScale_ESS_Vdc)*Rm_Ain_ESS_Vdc; 		
	
	//Negative trip
    Fault_Volt_A_SOC0_N = (-INV_Ias_Flt_HW_Set/SenScale_Iac_1_5kW)*Rm_Ain_Iac_1_5kW;    // INV -Ias
    Fault_Volt_B_SOC0_N = (-INV_Ibs_Flt_HW_Set/SenScale_Iac_1_5kW)*Rm_Ain_Iac_1_5kW;    // INV -Ibs
    Fault_Volt_C_SOC0_N = (-INV_Ics_Flt_HW_Set/SenScale_Iac_1_5kW)*Rm_Ain_Iac_1_5kW;    // INV -Ics
    Fault_Volt_D_SOC0_N = (-Vdc_Flt_HW_Set/SenScale_Vdc_1_5kW)*Rm_Ain_Vdc_1_5kW;        // Vdc
	                                                                                   
//	Fault_Volt_A_SOC1_N = (-INV_Ibs_Flt_HW_Set/SenScale_Iac_30kVA)*Rm_Ain_Iac_30kVA;	
//	Fault_Volt_B_SOC1_N = (-INV_Ics_Flt_HW_Set/SenScale_Iac_30kVA)*Rm_Ain_Iac_30kVA;	
//	Fault_Volt_C_SOC1_N = (-INV_Ias_Flt_HW_Set/SenScale_Iac_30kVA)*Rm_Ain_Iac_30kVA;	
//	Fault_Volt_D_SOC1_N = (-ESS_I_Flt_HW_Set/SenScale_ESS_Idc)*Rm_Ain_ESS_Idc; 		
	
//	Fault_Volt_A_SOC2_N = (-GRID_Vbc_HW_Set/SenScale_Vac_40kW)*Rm_Ain_Vac_40kW;		
//	Fault_Volt_B_SOC2_N = (-GRID_Vab_HW_Set/SenScale_Vac_40kW)*Rm_Ain_Vac_40kW;		
//	Fault_Volt_C_SOC2_N = (-INV_Ias_Flt_HW_Set/SenScale_Iac_30kVA)*Rm_Ain_Iac_30kVA;	
//	Fault_Volt_D_SOC2_N = (-ESS_V_Flt_HW_Set/SenScale_ESS_Vdc)*Rm_Ain_ESS_Vdc;

	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	Fault_Volt_SOC0_P[0] = Fault_Volt_A_SOC0_P;	// INV -Ias 
	Fault_Volt_SOC0_P[1] = Fault_Volt_B_SOC0_P;	// INV -Ibs 
	Fault_Volt_SOC0_P[2] = Fault_Volt_C_SOC0_P;	// INV -Ics 
	Fault_Volt_SOC0_P[3] = Fault_Volt_D_SOC0_P;	// Vdc       
//	Fault_Volt_SOC1_P[0] = Fault_Volt_A_SOC1_P; 
//	Fault_Volt_SOC1_P[1] = Fault_Volt_B_SOC1_P; 
//	Fault_Volt_SOC1_P[2] = Fault_Volt_C_SOC1_P; 
//	Fault_Volt_SOC1_P[3] = Fault_Volt_D_SOC1_P; 
//	Fault_Volt_SOC2_P[0] = Fault_Volt_A_SOC2_P;	
//	Fault_Volt_SOC2_P[1] = Fault_Volt_B_SOC2_P; 
//	Fault_Volt_SOC2_P[2] = Fault_Volt_C_SOC2_P; 
//	Fault_Volt_SOC2_P[3] = Fault_Volt_D_SOC2_P; 
	
	Fault_Volt_SOC0_N[0] = Fault_Volt_A_SOC0_N;	// INV -Ias        
	Fault_Volt_SOC0_N[1] = Fault_Volt_B_SOC0_N; // INV -Ibs        
	Fault_Volt_SOC0_N[2] = Fault_Volt_C_SOC0_N; // INV -Ics        
	Fault_Volt_SOC0_N[3] = Fault_Volt_D_SOC0_N; // Vdc              
//	Fault_Volt_SOC1_N[0] = Fault_Volt_A_SOC1_N;  
//	Fault_Volt_SOC1_N[1] = Fault_Volt_B_SOC1_N;  
//	Fault_Volt_SOC1_N[2] = Fault_Volt_C_SOC1_N;  
//	Fault_Volt_SOC1_N[3] = Fault_Volt_D_SOC1_N;  
//	Fault_Volt_SOC2_N[0] = Fault_Volt_A_SOC2_N;  
//	Fault_Volt_SOC2_N[1] = Fault_Volt_B_SOC2_N;  
//	Fault_Volt_SOC2_N[2] = Fault_Volt_C_SOC2_N;  
//	Fault_Volt_SOC2_N[3] = Fault_Volt_D_SOC2_N;  

	// Positive CMPSS MAP
	// CMPIN1P : ADCINA2	(INV1.Ias)
	// CMPIN3P : ADCINB2	(INV1.Ibs)
	// CMPIN6P : ADCINC2	(INV1.Ics)
	// CMPIN7P : ADCIND0	(Vdc)

	//CMPSS Positive trip
	Fault_Volt_CMPSS1_P = Fault_Volt_A_SOC0_P;   // ADC A2 pin,(Inv1.Ias) 
//	Fault_Volt_CMPSS2_P = Fault_Volt_A_SOC1_P;   // ADC A4 pin,
	Fault_Volt_CMPSS3_P = Fault_Volt_B_SOC0_P;   // ADC B2 pin,(Inv1.Ibs)
//	Fault_Volt_CMPSS4_P = Fault_Volt_B_SOC1_P;   // ADC 14 pin,
//	Fault_Volt_CMPSS5_P = Fault_Volt_C_SOC1_P;   // ADC C4 pin,
	Fault_Volt_CMPSS6_P = Fault_Volt_C_SOC0_P;   // ADC C2 pin,(Inv1.Ics)
	Fault_Volt_CMPSS7_P = Fault_Volt_D_SOC0_P;   // ADC D0 pin,(Vdc)
//	Fault_Volt_CMPSS8_P = Fault_Volt_D_SOC1_P;   // ADC D2 pin,
	
	// Negative CMPSS MAP 
	// CMPIN1N : ADCINA3	(INV1.Ias)
	// CMPIN3N : ADCINB3	(INV1.Ibs)
	// CMPIN6N : ADCINC3	(INV1.Ics)
	// CMPIN7N : ADCIND1	(Vdc)
	
	//CMPSS Negative trip
	Fault_Volt_CMPSS1_N = Fault_Volt_A_SOC0_N;   // ADC A3 pin,(Inv1.Ias)
//	Fault_Volt_CMPSS2_N = Fault_Volt_A_SOC1_N;   // ADC A5 pin,
	Fault_Volt_CMPSS3_N = Fault_Volt_B_SOC0_N;   // ADC B3 pin,(Inv1.Ibs)
//	Fault_Volt_CMPSS4_N = Fault_Volt_B_SOC1_N;   // ADC 15 pin,
//	Fault_Volt_CMPSS5_N = Fault_Volt_C_SOC1_N;   // ADC C5 pin,
	Fault_Volt_CMPSS6_N = Fault_Volt_C_SOC0_N;   // ADC C3 pin,(Inv1.Ics)
	Fault_Volt_CMPSS7_N = Fault_Volt_D_SOC0_N;   // ADC D1 pin,(Vdc)
//	Fault_Volt_CMPSS8_N = Fault_Volt_D_SOC1_N;   // ADC D3 pin,
	
	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	//ADC Fault Level (Positive)//16bit
	Fault_ADC_SOC0_P[0] = (Uint16)((Fault_Volt_A_SOC0_P/15.+0.5)*65536.+0.5);	// INV -Ias         
	Fault_ADC_SOC0_P[1] = (Uint16)((Fault_Volt_B_SOC0_P/15.+0.5)*65536.+0.5);   // INV -Ibs         
	Fault_ADC_SOC0_P[2] = (Uint16)((Fault_Volt_C_SOC0_P/15.+0.5)*65536.+0.5);   // INV -Ics         
	Fault_ADC_SOC0_P[3] = (Uint16)((Fault_Volt_D_SOC0_P/15.+0.5)*65536.+0.5);   // Vdc               
	                                                                            
//	Fault_ADC_SOC1_P[0] = (Uint16)((Fault_Volt_A_SOC1_P/15.+0.5)*65536.+0.5);     
//	Fault_ADC_SOC1_P[1] = (Uint16)((Fault_Volt_B_SOC1_P/15.+0.5)*65536.+0.5);     
//	Fault_ADC_SOC1_P[2] = (Uint16)((Fault_Volt_C_SOC1_P/15.+0.5)*65536.+0.5);     
//	Fault_ADC_SOC1_P[3] = (Uint16)((Fault_Volt_D_SOC1_P/15.+0.5)*65536.+0.5);     
	                                                                            
//	Fault_ADC_SOC2_P[0] = (Uint16)((Fault_Volt_A_SOC2_P/15.+0.5)*65536.+0.5);      
//	Fault_ADC_SOC2_P[1] = (Uint16)((Fault_Volt_B_SOC2_P/15.+0.5)*65536.+0.5);      
//	Fault_ADC_SOC2_P[2] = (Uint16)((Fault_Volt_C_SOC2_P/15.+0.5)*65536.+0.5);      
//	Fault_ADC_SOC2_P[3] = (Uint16)((Fault_Volt_D_SOC2_P/15.+0.5)*65536.+0.5);      
	
	//ADC Fault Level (Negative)
	Fault_ADC_SOC0_N[0] = (Uint16)((Fault_Volt_A_SOC0_N/15.+0.5)*65536.+0.5);   // INV -Ias           
	Fault_ADC_SOC0_N[1] = (Uint16)((Fault_Volt_B_SOC0_N/15.+0.5)*65536.+0.5);   // INV -Ibs           
	Fault_ADC_SOC0_N[2] = (Uint16)((Fault_Volt_C_SOC0_N/15.+0.5)*65536.+0.5);   // INV -Ics           
	Fault_ADC_SOC0_N[3] = (Uint16)((Fault_Volt_D_SOC0_N/15.+0.5)*65536.+0.5);   // Vdc                 
	                                                                                                   
//	Fault_ADC_SOC1_N[0] = (Uint16)((Fault_Volt_A_SOC1_N/15.+0.5)*65536.+0.5);    
//	Fault_ADC_SOC1_N[1] = (Uint16)((Fault_Volt_B_SOC1_N/15.+0.5)*65536.+0.5);    
//	Fault_ADC_SOC1_N[2] = (Uint16)((Fault_Volt_C_SOC1_N/15.+0.5)*65536.+0.5);    
//	Fault_ADC_SOC1_N[3] = (Uint16)((Fault_Volt_D_SOC1_N/15.+0.5)*65536.+0.5);    
	                                                                             
//	Fault_ADC_SOC2_N[0] = (Uint16)((Fault_Volt_A_SOC2_N/15.+0.5)*65536.+0.5);    
//	Fault_ADC_SOC2_N[1] = (Uint16)((Fault_Volt_B_SOC2_N/15.+0.5)*65536.+0.5);    
//	Fault_ADC_SOC2_N[2] = (Uint16)((Fault_Volt_C_SOC2_N/15.+0.5)*65536.+0.5);    
//	Fault_ADC_SOC2_N[3] = (Uint16)((Fault_Volt_D_SOC2_N/15.+0.5)*65536.+0.5);    	

	// Positive CMPSS MAP
	// CMPIN1P : ADCINA2	(INV1.Ias)
	// CMPIN3P : ADCINB2	(INV1.Ibs)
	// CMPIN6P : ADCINC2	(INV1.Ics)
	// CMPIN7P : ADCIND0	(Vdc)

	//CMPSS Fault Level (Positive)//12bit
	Fault_CMPSS_P[0] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS1_P)+0.5);	// ADC A2 pin,(Inv1.Ias)  
//	Fault_CMPSS_P[1] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS2_P)+0.5);  // ADC A4 pin,
	Fault_CMPSS_P[2] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS3_P)+0.5);  // ADC B2 pin,(Inv1.Ibs)  
//	Fault_CMPSS_P[3] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS4_P)+0.5);  // ADC 14 pin,
//	Fault_CMPSS_P[4] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS5_P)+0.5);  // ADC C4 pin,
	Fault_CMPSS_P[5] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS6_P)+0.5);  // ADC C2 pin,(Inv1.Ics)  
	Fault_CMPSS_P[6] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS7_P)+0.5);  // ADC D0 pin,(Vdc)     
//	Fault_CMPSS_P[7] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS8_P)+0.5);  // ADC D2 pin,

	// Negative CMPSS MAP 
	// CMPIN1N : ADCINA3	(INV1.Ias)
	// CMPIN3N : ADCINB3	(INV1.Ibs)
	// CMPIN6N : ADCINC3	(INV1.Ics)
	// CMPIN7N : ADCIND1	(Vdc)

	Fault_CMPSS_N[0] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS1_N)+0.5);	// ADC A3 pin,(Inv1.Ias) 
//	Fault_CMPSS_N[1] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS2_N)+0.5);  // ADC A5 pin,
	Fault_CMPSS_N[2] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS3_N)+0.5);  // ADC B3 pin,(Inv1.Ibs) 
//	Fault_CMPSS_N[3] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS4_N)+0.5);  // ADC 15 pin,
//	Fault_CMPSS_N[4] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS5_N)+0.5);  // ADC C5 pin,
	Fault_CMPSS_N[5] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS6_N)+0.5);  // ADC C3 pin,(Inv1.Ics) 
	Fault_CMPSS_N[6] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS7_N)+0.5);  // ADC D1 pin,(Vdc)      
//	Fault_CMPSS_N[7] = (Uint16)( 4095./3.3*(1.5+0.2*Fault_Volt_CMPSS8_N)+0.5);  // ADC D3 pin,

	Fault.HW_Prot.all=0x0000;
	Fault.GT_Prot.all=0x0000;
	Fault.SW_Prot.all=0x0000;

	//ADCRESULT 에 0이 들어가 있는데 0V를 의미하는 중간값을 넣어주기 위해
	//forced SW ADC SOC -------------------------------------------
	AdcaRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1, SOC2 forced start
	AdcbRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1, SOC2 forced start
	AdccRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1, SOC2 forced start
	AdcdRegs.ADCSOCFRC1.all = 0x007; //SOC0, SOC1, SOC2 forced start
	//forced SW ADC SOC ---------------------12 cycle-------------
	DELAY_US(10L);
	AdcaRegs.ADCINTFLGCLR.all = 0x7;
	AdcbRegs.ADCINTFLGCLR.all = 0x7;
	AdccRegs.ADCINTFLGCLR.all = 0x7;
	AdcdRegs.ADCINTFLGCLR.all = 0x7;

  	EALLOW;
  	
  	//소프트 웨어(디지털) 폴트 설정
	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	AdcaRegs.ADCPPB1TRIPHI.bit.LIMITHI = Fault_ADC_SOC0_P[0];//0xffff;//16bit unsigned int 형,	(Inv1.Ias) 
	AdcaRegs.ADCPPB1TRIPLO.bit.LIMITLO = Fault_ADC_SOC0_N[0];//0x0000;//16bit unsigned int 형
//	AdcaRegs.ADCPPB2TRIPHI.bit.LIMITHI = Fault_ADC_SOC1_P[0];//16bit unsigned int 형,			
//	AdcaRegs.ADCPPB2TRIPLO.bit.LIMITLO = Fault_ADC_SOC1_N[0];//0x0000;//16bit unsigned int 형
//	AdcaRegs.ADCPPB3TRIPHI.bit.LIMITHI = Fault_ADC_SOC2_P[0];//16bit unsigned int 형,			
//	AdcaRegs.ADCPPB3TRIPLO.bit.LIMITLO = Fault_ADC_SOC2_N[0];//0x0000;//16bit unsigned int 형

	AdcbRegs.ADCPPB1TRIPHI.bit.LIMITHI = Fault_ADC_SOC0_P[1];//0xffff;//16bit unsigned int 형   	(Inv1.Ibs)
	AdcbRegs.ADCPPB1TRIPLO.bit.LIMITLO = Fault_ADC_SOC0_N[1];//0x0000;//16bit unsigned int 형   
//	AdcbRegs.ADCPPB2TRIPHI.bit.LIMITHI = Fault_ADC_SOC1_P[1];//16bit unsigned int 형,			
//	AdcbRegs.ADCPPB2TRIPLO.bit.LIMITLO = Fault_ADC_SOC1_N[1];//0x0000;//16bit unsigned int 형   
//	AdcbRegs.ADCPPB3TRIPHI.bit.LIMITHI = Fault_ADC_SOC2_P[1];//16bit unsigned int 형,			
//	AdcbRegs.ADCPPB3TRIPLO.bit.LIMITLO = Fault_ADC_SOC2_N[1];//0x0000;//16bit unsigned int 형   

	AdccRegs.ADCPPB1TRIPHI.bit.LIMITHI = Fault_ADC_SOC0_P[2];//0xffff;//16bit unsigned int 형   	(Inv1.Ics)
	AdccRegs.ADCPPB1TRIPLO.bit.LIMITLO = Fault_ADC_SOC0_N[2];//0x0000;//16bit unsigned int 형   
//	AdccRegs.ADCPPB2TRIPHI.bit.LIMITHI = Fault_ADC_SOC1_P[2];//16bit unsigned int 형,			
//	AdccRegs.ADCPPB2TRIPLO.bit.LIMITLO = Fault_ADC_SOC1_N[2];//0x0000;//16bit unsigned int 형   
//	AdccRegs.ADCPPB3TRIPHI.bit.LIMITHI = Fault_ADC_SOC2_P[2];//16bit unsigned int 형,			
//	AdccRegs.ADCPPB3TRIPLO.bit.LIMITLO = Fault_ADC_SOC2_N[2];//0x0000;//16bit unsigned int 형   

	AdcdRegs.ADCPPB1TRIPHI.bit.LIMITHI = Fault_ADC_SOC0_P[3];//0xffff;//16bit unsigned int 형   	(Vdc)
	AdcdRegs.ADCPPB1TRIPLO.bit.LIMITLO = Fault_ADC_SOC0_N[3];//0x0000;//16bit unsigned int 형   
//	AdcdRegs.ADCPPB2TRIPHI.bit.LIMITHI = Fault_ADC_SOC1_P[3];//16bit unsigned int 형,			
//	AdcdRegs.ADCPPB2TRIPLO.bit.LIMITLO = Fault_ADC_SOC1_N[3];//0x0000;//16bit unsigned int 형   
//	AdcdRegs.ADCPPB3TRIPHI.bit.LIMITHI = Fault_ADC_SOC2_P[3];//16bit unsigned int 형,			
//	AdcdRegs.ADCPPB3TRIPLO.bit.LIMITLO = Fault_ADC_SOC2_N[3];//0x0000;//16bit unsigned int 형 
	EDIS;

  	ADCEVT_PPB_CLEAR();

	//enable ADEVT ISR (FAULT INT)
  	EALLOW;
  	AdcaRegs.ADCEVTSEL.all = 0x0003; 		//0b 0000 0000 0011 0011//for Trip with EPwmXbar, case INV1/2.Ibs
	AdcbRegs.ADCEVTSEL.all = 0x0003; 		//0b 0000 0000 0011 0011//case INV1/2.Ics
	AdccRegs.ADCEVTSEL.all = 0x0003; 		//0b 0000 0000 0011 0011//case INV1/2.Ias
	//AdcdRegs.ADCEVTSEL.all = 0x0131; 		//0b 0000 0001 0011 0001//case Vdc, INV3.Iqse, Vdc_sup
	AdcdRegs.ADCEVTSEL.all = 0x0001; 		//0b 0000 0000 0000 0001//only Vdc for running E/L
	
	AdcaRegs.ADCEVTINTSEL.all = 0x0003;	 	//0b 0000 0000 0011 0011//for Interrupt routine with software
	AdcbRegs.ADCEVTINTSEL.all = 0x0003; 	//0b 0000 0000 0011 0011
	AdccRegs.ADCEVTINTSEL.all = 0x0003; 	//0b 0000 0000 0011 0011
	//AdcdRegs.ADCEVTINTSEL.all = 0x0131; 	//0b 0000 0001 0011 0001
	AdcdRegs.ADCEVTINTSEL.all = 0x0001; 	//0b 0000 0000 0000 0001//only Vdc for running E/L
	
  //AdcaRegs.ADCEVTINTSEL.all = 0x0333; 	//0b 0000 0011 0011 0011
  //AdcbRegs.ADCEVTINTSEL.all = 0x0333; 	//0b 0000 0011 0011 0011
  //AdccRegs.ADCEVTINTSEL.all = 0x0333; 	//0b 0000 0011 0011 0011
  //AdccRegs.ADCEVTINTSEL.all = 0x0033; 	//0b 0000 0000 0011 0011
  //AdcdRegs.ADCEVTINTSEL.all = 0x0333; 	//0b 0000 0011 0011 0011
	EDIS;
	
	//소프트 웨어(디지털) 폴트 설정
	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)o used)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	// Positive CMPSS MAP
	// CMPIN1P : ADCINA2	(INV1.Ias)
	// CMPIN3P : ADCINB2	(INV1.Ibs)
	// CMPIN6P : ADCINC2	(INV1.Ics)
	// CMPIN7P : ADCIND0	(Vdc)	
	
	// Negative CMPSS MAP 
	// CMPIN1N : ADCINA3	(INV1.Ias)
	// CMPIN3N : ADCINB3	(INV1.Ibs)
	// CMPIN6N : ADCINC3	(INV1.Ics)
	// CMPIN7N : ADCIND1	(Vdc)
	
	//CMPSS 아날로그 폴트 설정
  	//Set DAC to midpoint for arbitrary reference (ADCINA2) : SOC1, A, 
  	Cmpss1Regs.DACHVALS.bit.DACVAL             = Fault_CMPSS_P[0]; //비교 값 12bit(max 4095),(Inv1.Ias), ADCINA2
  	Cmpss1Regs.DACLVALS.bit.DACVAL             = Fault_CMPSS_N[0]; //비교 값, , ADCINA3
  	//Set DAC to midpoint for arbitrary reference (ADCINA4) : SOC2, A
// 	Cmpss2Regs.DACHVALS.bit.DACVAL             = Fault_CMPSS_P[1]; //비교 값 12bit(max 4095),(no used), ADCINA4
// 	Cmpss2Regs.DACLVALS.bit.DACVAL             = Fault_CMPSS_N[1]; //비교 값, ADCINA5
  	//Set DAC to midpoint for arbitrary reference (ADCINB2) : SOC1, B
  	Cmpss3Regs.DACHVALS.bit.DACVAL             = Fault_CMPSS_P[2]; //비교 값 12bit(max 4095),(Inv1.Ibs), ADCINB2
  	Cmpss3Regs.DACLVALS.bit.DACVAL             = Fault_CMPSS_N[2]; //비교 값, ADCINB3
  	//Set DAC to midpoint for arbitrary reference (ADCIN14) : SOC2, C
// 	Cmpss4Regs.DACHVALS.bit.DACVAL             = Fault_CMPSS_P[3]; //비교 값 12bit(max 4095),(no used), ADCIN14
// 	Cmpss4Regs.DACLVALS.bit.DACVAL             = Fault_CMPSS_N[3]; //비교 값, ADCIN15
  	//Set DAC to midpoint for arbitrary reference (ADCINC4) : SOC1, C
// 	Cmpss5Regs.DACHVALS.bit.DACVAL             = Fault_CMPSS_P[4]; //비교 값 12bit(max 4095),(no used), ADCINC4
// 	Cmpss5Regs.DACLVALS.bit.DACVAL             = Fault_CMPSS_N[4]; //비교 값, ADCINC4
  	//Set DAC to midpoint for arbitrary reference (ADCINC2) : SOC0, C
  	Cmpss6Regs.DACHVALS.bit.DACVAL             = Fault_CMPSS_P[5]; //비교 값 12bit(max 4095),(Inv1.Ics), ADCINC2
  	Cmpss6Regs.DACLVALS.bit.DACVAL             = Fault_CMPSS_N[5]; //비교 값, ADCINC3
  	//Set DAC to midpoint for arbitrary reference (ADCIND0) : SOC0, D
  	Cmpss7Regs.DACHVALS.bit.DACVAL             = Fault_CMPSS_P[6]; //비교 값 12bit(max 4095),(Vdc), ADCIND0
  	Cmpss7Regs.DACLVALS.bit.DACVAL             = Fault_CMPSS_N[6]; //비교 값, ADCIND1
  	//Set DAC to midpoint for arbitrary reference (ADCIND2) : SOC1, D
// 	Cmpss8Regs.DACHVALS.bit.DACVAL             = Fault_CMPSS_P[7]; //비교 값 12bit(max 4095),(no used), ADCIND2
// 	Cmpss8Regs.DACLVALS.bit.DACVAL             = Fault_CMPSS_N[7]; //비교 값, ADCIND3
  	
  	CMPSS_STS_LATCH_CLEAR();
  	
  	nFAULT_OFF;
	Flags.Fault 			= 0;
//	Flags.Conv_Run 			= 1;
	Flags.Inv_Run 			= 1;
	Flags.Fault_old 		= 0;
	Flags.Fault_Warn_old 	= 0;
	HwFltCnt 				= 0;
	FLT.uFlag 				= 0 ;
	FLT.uFltNum 			= 0;
	g_SpiArmStatus.u8InvErr	= 0x00;
//	SCI_OUT2.regStatusBit.inver 				= 0 ;
	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.all 	= 0x00;
	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.all 	= 0x00;
	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.all 	= 0x00;
	Fault_DSP.FAULT_DCBEVT1.FAULT_ECT.all 		= 0x00;
}

void FLT_Reset(void)
{
	int i;
	InitFault();
	nFAULT_OFF;
	nINV_ERR = 1;
	g_ErrorCode = 0;
	g_SpiArmStatus.u8INV_ErrCode = 0;
//	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;	//EPWM1
	Flags.Fault 				= 0;
//	Flags.Conv_Run 				= 1;
	Flags.Inv_Run 				= 1;
	Flags.Fault_old 			= 0;
	Flags.Fault_Warn_old 		= 0;
	Flags.Fault_Warn_RunCnt 	= 0;
	Flags.Fault_Warn 			= 0;
	g_SpiArmStatus.u8InvErr     = 0x00;
//	SCI_OUT2.regStatusBit.inver = 0;
	 for( i=0; i<FLT_TABLE_NUM; ++i ) FLT.ariTable[i] = 0;  
	FLT.iTableIndex 	= 0;
	FLT.iTableIndex1 	= 0;
	HwFltCnt 			= 0;
	FLT.uFlag 			= 0 ;
	//FLT.uFltNum = 0;
	//Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.all 		= 0x00;
	//Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.all 		= 0x00;
	//Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.all 	= 0x00;
	//Fault_DSP.FAULT_DCBEVT1.FAULT_ECT.all 		= 0x00;
//	CONV_TRIP_OFF;
	INV_TRIP_OFF;
//	ESS_TRIP_OFF;
	GATE_RST_ON();
	EALLOW;
	EPwm1Regs.TZCLR.all = 0xFF;
	EPwm2Regs.TZCLR.all = 0xFF;
	EPwm3Regs.TZCLR.all = 0xFF;
	EPwm4Regs.TZCLR.all = 0xFF;
	EPwm5Regs.TZCLR.all = 0xFF;
	EPwm6Regs.TZCLR.all = 0xFF;
	EPwm7Regs.TZCLR.all = 0xFF;
	EDIS;
}
void FLT_WarnRaise(Uint16 FltNum){
    u8 u8FltBackIndex = 0;
	int i;
	
	EALLOW;
	EPwm7Regs.TZFRC.bit.OST = 1;
	EDIS;
	
	Flags.Fault_Warn = 1;
	Flags.Run3 = 0;
	HwFltWarnCnt++;
	
	if((Flags.Fault_Warn)&&(Flags.Fault_Warn_old == 0))
	{
	    FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[0];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[1];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[2];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[3];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[4];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[5];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[0];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[1];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[2];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[3];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[4];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[5];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[0];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[1];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[2];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[3];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[4];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[5];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[0];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[1];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[2];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[3];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[4];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[5];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[0];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[1];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[2];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[3];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[4];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[5];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[0];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[1];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[2];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[3];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[4];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[5];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Speed_Ref[0];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Speed_Ref[1];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Speed_Ref[2];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Speed_Ref[3];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[0];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[1];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[2];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[3];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[4];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[5];
        FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[6];

        FLT_backup[u8FltBackIndex++]    = ((g_BDIB.regStatusBit.UP&0x01<<8)|(g_BDIB.regStatusBit.DOWN&0x01<<7)|(g_BDIB.regStatusBit.RST&0x01<<6)|(g_BDIB.regStatusBit.RDY&0x01<<5)|(g_BDIB.regStatusBit.AT&0x01<<4)|(g_BDIB.regStatusBit.XSA&0x01<<3)|(g_BDIB.regStatusBit.XBKAS&0x01<<2)|(g_BDIB.regStatusBit.XBKBS&0x01<<1)|(BKO&0x01));
        FLT_backup[u8FltBackIndex++]    = ((g_BDIB.regStatusBit.XCLI&0x01<<8)|(g_BDIB.regStatusBit.XDLV&0x01<<7)|(g_BDIB.regStatusBit.XDZV&0x01<<6)|(g_BDIB.regStatusBit.XHCLI&0x01<<5)|(g_BDIB.regStatusBit.XULV&0x01<<4));
        FLT_backup[u8FltBackIndex++]    = (((u8)Flag_ZSP&0x01<<8)) ;//1110b
        u8FltBackIndex = 0;
 	}
	Flags.Fault_Warn_old = Flags.Fault_Warn;
	/* 고장이 FLT_TABLE_NUM갯수 보다 많이 발생 하였을 경우 */
	if ( FLT.iTableIndex1 >= FLT_TABLE_NUM ) return;

	/* 고장테이블에 FltNum고장이 이미 존재한다면 등록을 하지 않는다. */
	for ( i=0; i< FLT_TABLE_NUM; i++) 
		if ( FLT.ariTable[i] == FltNum ) return;

	FLT.ariTable[FLT.iTableIndex1] 		= FltNum;
	Fault_RealYear[FLT.iTableIndex]     = (g_IdSpiCoData.rspDate.u8Year&0xff);
    Fault_RealMonth[FLT.iTableIndex]    = (g_IdSpiCoData.rspDate.u8Month&0xff);
    Fault_RealDate[FLT.iTableIndex]     = (g_IdSpiCoData.rspDate.u8Date&0xff);
    Fault_RealHour[FLT.iTableIndex]     = (g_IdSpiCoData.rspTime.u8Hour&0xff);
    Fault_RealMinute[FLT.iTableIndex]   = (g_IdSpiCoData.rspTime.u8Minute&0xff);
    Fault_RealSec[FLT.iTableIndex]      = (g_IdSpiCoData.rspTime.u8Second&0xff);

	FLT.iTableIndex1++;
}

void FLT_Raise(Uint16 FltNum)
{
    u8 u8FltBackIndex = 0;

	g_ErrorCode = FltNum;	// Test Variable Added by GWON
	g_SpiArmStatus.u8INV_ErrCode = FltNum;
	nINV_ERR = 0;
	int i;
	PWM1_BUFF_TRIP();
	PWM1_BUFF_OFF();

	BK_CLOSE();

	EALLOW;
	EPwm1Regs.TZFRC.bit.OST = 1;
	EPwm2Regs.TZFRC.bit.OST = 1;
	EPwm3Regs.TZFRC.bit.OST = 1;
//	EPwm4Regs.TZFRC.bit.OST = 1;
//	EPwm5Regs.TZFRC.bit.OST = 1;
//	EPwm6Regs.TZFRC.bit.OST = 1;
//	EPwm7Regs.TZFRC.bit.OST = 1;
	EDIS;
	
	nFAULT_ON;
	Flags.Fault =1;
	Flags.Conv_Run = 0;
	Flags.Inv_Run = 0;
	FLT.uFlag = 1 ;
	
//	INV_WD2_ON;
	g_SpiArmStatus.u8InvErr = 1;
//	SCI_OUT2.regStatusBit.inver = 1 ;
	
	FLT.uFltNum = FltNum;
	if((((FltNum <= 112)&&(FltNum >= 100))||(FltNum == FLT_EARTH_C)||(FltNum == FLT_EARTH_I)||(FltNum == FLT_UVW)||(FltNum == FLT_OUTPUT)||(FltNum == FLT_OL))&&(Flags.Fault == 0))
	{
		HwFltCnt++;
	}	

	// Need to FLT_backup data type from unsigned int to u8
	if((Flags.Fault)&&(Flags.Fault_old == 0))
	{
		FLT_backup[u8FltBackIndex++] 	= g_SpiArmStatus.u8Inv_Iqse[0];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[1];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[2];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[3];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[4];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse[5];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[0];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[1];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[2];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[3];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[4];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse[5];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[0];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[1];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[2];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[3];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[4];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Iqse_Ref[5];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[0];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[1];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[2];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[3];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[4];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Idse_Ref[5];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[0];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[1];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[2];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[3];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[4];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_DC_Link[5];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[0];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[1];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[2];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[3];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[4];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Vll[5];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Speed_Ref[0];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Speed_Ref[1];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Speed_Ref[2];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Speed_Ref[3];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[0];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[1];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[2];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[3];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[4];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[5];
		FLT_backup[u8FltBackIndex++]    = g_SpiArmStatus.u8Inv_Thrust[6];

		FLT_backup[u8FltBackIndex++]    = ((g_BDIB.regStatusBit.UP&0x01<<8)|(g_BDIB.regStatusBit.DOWN&0x01<<7)|(g_BDIB.regStatusBit.RST&0x01<<6)|(g_BDIB.regStatusBit.RDY&0x01<<5)|(g_BDIB.regStatusBit.AT&0x01<<4)|(g_BDIB.regStatusBit.XSA&0x01<<3)|(g_BDIB.regStatusBit.XBKAS&0x01<<2)|(g_BDIB.regStatusBit.XBKBS&0x01<<1)|(BKO&0x01));
		FLT_backup[u8FltBackIndex++] 	= ((g_BDIB.regStatusBit.XCLI&0x01<<8)|(g_BDIB.regStatusBit.XDLV&0x01<<7)|(g_BDIB.regStatusBit.XDZV&0x01<<6)|(g_BDIB.regStatusBit.XHCLI&0x01<<5)|(g_BDIB.regStatusBit.XULV&0x01<<4));
		FLT_backup[u8FltBackIndex++] 	= (((u8)Flag_ZSP&0x01<<8)) ;//1110b
		u8FltBackIndex = 0;
	}
 	Flags.Fault_old = Flags.Fault;
 	/* 고장이 FLT_TABLE_NUM갯수 보다 많이 발생 하였을 경우 */
	if ( FLT.iTableIndex >= FLT_TABLE_NUM ) return;

	/* 고장테이블에 FltNum고장이 이미 존재한다면 등록을 하지 않는다. */
	for ( i=0; i< FLT_TABLE_NUM; i++) 
		if ( FLT.ariTable[i] == FLT.uFltNum ) return;

	FLT.ariTable[FLT.iTableIndex] 		= FLT.uFltNum;

	// Need to Fault_Real* data type from int to u8
	Fault_RealYear[FLT.iTableIndex] 	= (g_IdSpiCoData.rspDate.u8Year&0xff);
	Fault_RealMonth[FLT.iTableIndex] 	= (g_IdSpiCoData.rspDate.u8Month&0xff);
	Fault_RealDate[FLT.iTableIndex] 	= (g_IdSpiCoData.rspDate.u8Date&0xff);
	Fault_RealHour[FLT.iTableIndex] 	= (g_IdSpiCoData.rspTime.u8Hour&0xff);
	Fault_RealMinute[FLT.iTableIndex] 	= (g_IdSpiCoData.rspTime.u8Minute&0xff);
	Fault_RealSec[FLT.iTableIndex] 		= (g_IdSpiCoData.rspTime.u8Second&0xff);

	FLT.iTableIndex++;
}
__interrupt void epwm1_tzint_isr(void)
{

//	test_JS = 494949;

	PWM1_BUFF_TRIP();
//	PWM2_BUFF_TRIP();
//	PWM3_BUFF_TRIP();//if converter or inverter is fault, ESS control is disenable.

	PWM1_BUFF_OFF();
//	PWM2_BUFF_OFF();
//	PWM3_BUFF_OFF();//if converter or inverter is fault, ESS control is disenable.
	
	nFAULT_ON;
	INV_WD2_ON;
	g_SpiArmStatus.u8InvErr = 1;
//	SCI_OUT2.regStatusBit.inver = 1 ;
	
	fault_cnt++;

	EALLOW;
	EPwm1Regs.TZFRC.bit.OST = 1;
	EPwm2Regs.TZFRC.bit.OST = 1;
	EPwm3Regs.TZFRC.bit.OST = 1;
	EPwm4Regs.TZFRC.bit.OST = 1;
	EPwm5Regs.TZFRC.bit.OST = 1;
	EPwm6Regs.TZFRC.bit.OST = 1;
	EPwm7Regs.TZFRC.bit.OST = 1;
	EDIS;

	Flags.Fault =1;
	FLT.uFlag = 1 ;
	Flags.Conv_Run = 0;
	Flags.Inv_Run = 0;

	//소프트 웨어(디지털) 폴트 설정
	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)o used)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	// Positive CMPSS MAP
	// CMPIN1P : ADCINA2	(INV1.Ias)
	// CMPIN3P : ADCINB2	(INV1.Ibs)
	// CMPIN6P : ADCINC2	(INV1.Ics)
	// CMPIN7P : ADCIND0	(Vdc)	
	
	// Negative CMPSS MAP 
	// CMPIN1N : ADCINA3	(INV1.Ias)
	// CMPIN3N : ADCINB3	(INV1.Ibs)
	// CMPIN6N : ADCINC3	(INV1.Ics)
	// CMPIN7N : ADCIND1	(Vdc)
	
	Fault_DSP.FAULT_DCAEVT1.FAULT_DCAEVT1_FLG_LAT = EPwm1Regs.TZFLG.bit.DCAEVT1;

	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS1_A2_H = Cmpss1Regs.COMPSTS.bit.COMPHLATCH;  
  	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS1_A2_L = Cmpss1Regs.COMPSTS.bit.COMPLLATCH;  
  	if((Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS1_A2_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS1_A2_L)) FLT_Raise(FLT_OCAH_I);

//	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS2_A4_H = Cmpss2Regs.COMPSTS.bit.COMPHLATCH; 
// 	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS2_A4_L = Cmpss2Regs.COMPSTS.bit.COMPLLATCH;
// 	if((Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS2_A4_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS2_A4_L)) FLT_Raise(FLT_OCBH_I);

	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS3_B2_H = Cmpss3Regs.COMPSTS.bit.COMPHLATCH;  
  	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS3_B2_L = Cmpss3Regs.COMPSTS.bit.COMPLLATCH;  
  	if((Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS3_B2_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS3_B2_L)) FLT_Raise(FLT_OCBH_I);

//	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS4_C14_H = Cmpss4Regs.COMPSTS.bit.COMPHLATCH;  
// 	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS4_C14_L = Cmpss4Regs.COMPSTS.bit.COMPLLATCH;  
// 	if((Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS4_C14_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS4_C14_L)) FLT_Raise(FLT_OCCH_I);

//	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS5_C4_H = Cmpss5Regs.COMPSTS.bit.COMPHLATCH;  
// 	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS5_C4_L = Cmpss5Regs.COMPSTS.bit.COMPLLATCH;  
// 	if((Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS5_C4_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS5_C4_L)) FLT_Raise(FLT_OCAH_I);

	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS6_C2_H = Cmpss6Regs.COMPSTS.bit.COMPHLATCH;  
  	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS6_C2_L = Cmpss6Regs.COMPSTS.bit.COMPLLATCH;  
  	if((Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS6_C2_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS6_C2_L)) FLT_Raise(FLT_OCCH_I);

	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS7_D0_H = Cmpss7Regs.COMPSTS.bit.COMPHLATCH;  
 	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS7_D0_L = Cmpss7Regs.COMPSTS.bit.COMPLLATCH;  
 	if((Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS7_D0_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS7_D0_L)) FLT_Raise(FLT_OVDCH);

//	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS8_D2_H = Cmpss8Regs.COMPSTS.bit.COMPHLATCH;  
// 	Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS8_D2_L = Cmpss8Regs.COMPSTS.bit.COMPLLATCH;  
//  if((Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS8_D2_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_CMPSS.bit.CMPSS8_D2_L)) FLT_Raise(FLT_OCH_SUP);// (SIN) no used

	//Adca ch : SOC0=2(INV2.Ibs), SOC1=4(INV1.Ibs)		, SOC2=0(Vbc)
	//Adcb ch : SOC0=2(INV2.Ics), SOC1=14(INV1.Ics)		, SOC2=0(Vab)
	//Adcc ch : SOC0=2(INV2.Ias), SOC1=4(INV1.Ias)		, SOC2=4(INV1.Ias)(no used)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(SIN)			, SOC2=4(COS)
	
	//Adca ch : SOC0=2(INV1.Ias), SOC1=0(SIN)		, SOC2=4(no use)
	//Adcb ch : SOC0=2(INV1.Ibs), SOC1=0(COS)		, SOC2=4(no use)
	//Adcc ch : SOC0=2(INV1.Ics), SOC1=4(Thermal)	, SOC2=0(no use)o used)
	//Adcd ch : SOC0=0(Vdc)		, SOC1=2(no use)	, SOC2=4(no use)
	
	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB1_H = AdcaRegs.ADCEVTSTAT.bit.PPB1TRIPHI; 
	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB1_L = AdcaRegs.ADCEVTSTAT.bit.PPB1TRIPLO;
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB2_H = AdcaRegs.ADCEVTSTAT.bit.PPB2TRIPHI; 
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB2_L = AdcaRegs.ADCEVTSTAT.bit.PPB2TRIPLO;
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB3_H = AdcaRegs.ADCEVTSTAT.bit.PPB3TRIPHI; 
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB3_L = AdcaRegs.ADCEVTSTAT.bit.PPB3TRIPLO;
	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB1_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB1_L)) FLT_Raise(FLT_OCAS_I);
//	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB2_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB2_L)) FLT_Raise(FLT_OCBS_I);
//	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB3_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.A_PPB3_L)) FLT_Raise(FLT_OVBCS_C);// (grid bc) no used

	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB1_H = AdcbRegs.ADCEVTSTAT.bit.PPB1TRIPHI; 
	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB1_L = AdcbRegs.ADCEVTSTAT.bit.PPB1TRIPLO;
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB2_H = AdcbRegs.ADCEVTSTAT.bit.PPB2TRIPHI; 
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB2_L = AdcbRegs.ADCEVTSTAT.bit.PPB2TRIPLO;
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB3_H = AdcbRegs.ADCEVTSTAT.bit.PPB3TRIPHI; 
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB3_L = AdcbRegs.ADCEVTSTAT.bit.PPB3TRIPLO;
	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB1_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB1_L)) FLT_Raise(FLT_OCBS_I);
//	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB2_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB2_L)) FLT_Raise(FLT_OCCS_I);
//	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB3_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.B_PPB3_L)) FLT_Raise(FLT_OVABS_C);// (grid ab) no used

	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB1_H = AdccRegs.ADCEVTSTAT.bit.PPB1TRIPHI; 
	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB1_L = AdccRegs.ADCEVTSTAT.bit.PPB1TRIPLO;
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB2_H = AdccRegs.ADCEVTSTAT.bit.PPB2TRIPHI; 
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB2_L = AdccRegs.ADCEVTSTAT.bit.PPB2TRIPLO;
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB3_H = AdccRegs.ADCEVTSTAT.bit.PPB3TRIPHI; 
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB3_L = AdccRegs.ADCEVTSTAT.bit.PPB3TRIPLO;
	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB1_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB1_L)) FLT_Raise(FLT_OCCS_I);
//	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB2_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB2_L)) FLT_Raise(FLT_OCAS_I);
//	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB3_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.C_PPB3_L)) ;//FLT_Raise(FLT_OVDCS);	// no used

	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB1_H = AdcdRegs.ADCEVTSTAT.bit.PPB1TRIPHI; 
	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB1_L = AdcdRegs.ADCEVTSTAT.bit.PPB1TRIPLO;
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB2_H = AdcdRegs.ADCEVTSTAT.bit.PPB2TRIPHI; 
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB2_L = AdcdRegs.ADCEVTSTAT.bit.PPB2TRIPLO;
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB3_H = AdcdRegs.ADCEVTSTAT.bit.PPB3TRIPHI; 
//	Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB3_L = AdcdRegs.ADCEVTSTAT.bit.PPB3TRIPLO;
	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB1_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB1_L)) FLT_Raise(FLT_OVDCS);
//	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB2_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB2_L)) FLT_Raise(FLT_OCS_SUP);// (SIN) no used
//	if((Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB3_H)||(Fault_DSP.FAULT_DCAEVT1.FAULT_ADCEVT.bit.D_PPB3_L)) FLT_Raise(FLT_OVS_SUP);// (COS) no used
	
	Fault_DSP.FAULT_DCBEVT1.FAULT_DCBEVT1_FLG_LAT = EPwm1Regs.TZFLG.bit.DCBEVT1; 

	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFA_ALL =  GpioDataRegs.GPCDAT.bit.GPIO69;// IPM1FAULTFLAG(active high)
	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFA0 =  GpioDataRegs.GPCDAT.bit.GPIO71;// BGFA0(active high)
	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFA1 =  GpioDataRegs.GPCDAT.bit.GPIO70;// BGFA1(active high)
	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFA2 =  GpioDataRegs.GPCDAT.bit.GPIO73;// BGFA2(active high)
	//Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFA3 =  GpioDataRegs.GPCDAT.bit.GPIO78;// BGFA3(active high)
	if(Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFA_ALL) FLT_Raise(FLT_MOSFET_I);

//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFB_ALL =  GpioDataRegs.GPCDAT.bit.GPIO77;// IPM2FAULTFLAG(active high)
//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFB0 =  GpioDataRegs.GPCDAT.bit.GPIO82;// BGFB0(active high)
//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFB1 =  GpioDataRegs.GPCDAT.bit.GPIO81;// BGFB1(active high)
//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFB2 =  GpioDataRegs.GPEDAT.bit.GPIO150;// BGFB2(active high)
//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFB3 =  GpioDataRegs.GPEDAT.bit.GPIO149;// BGFB3(active high)
//	if(Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFB_ALL) FLT_Raise(FLT_IGBT_C);

//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFC_ALL =  GpioDataRegs.GPEDAT.bit.GPIO147;// IPM3FAULTFLAG(active high)
//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFC0 =  GpioDataRegs.GPCDAT.bit.GPIO74;// BGFC0(active high)
//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFC1 =  GpioDataRegs.GPCDAT.bit.GPIO75;// BGFC1(active high)
//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFC2 =  GpioDataRegs.GPCDAT.bit.GPIO76;// BGFC2(active high)
//	Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFC3 =  GpioDataRegs.GPCDAT.bit.GPIO80;// BGFC3(active high)
//	if(Fault_DSP.FAULT_DCBEVT1.FAULT_GPIO_STS.bit.GFC_ALL) FLT_Raise(FLT_IGBT_SUP);//no use

	Fault_DSP.FAULT_DCBEVT1.FAULT_ECT.bit.DSP_Clock_Fail =  NmiIntruptRegs.NMIFLG.bit.CLOCKFAIL;// clock fail(active high)

  // Acknowledge this interrupt to receive more interrupts from group 2
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
  asm(" NOP");
}

//__interrupt void cpu1_igbt3flt_isr(void){
//
//	PWM3_BUFF_TRIP();
//	PWM3_BUFF_OFF();
//
//	ESS_FLT_ON;
//	INV_WARN_ON;
//	ESS_TRIP_ON;
//
//	if(GpioDataRegs.GPEDAT.bit.GPIO147) FLT_WarnRaise(FLT_IGBT_SUP);
//
//	EALLOW;
//	EPwm7Regs.TZFRC.bit.OST = 1;
//	EDIS;
//	Flags.Fault_Warn = 1;
//}

void ADCEVT_PPB_CLEAR(void){

	AdcaRegs.ADCEVTCLR.all = 0x0333;
	AdcbRegs.ADCEVTCLR.all = 0x0333;
	AdccRegs.ADCEVTCLR.all = 0x0333;
	AdcdRegs.ADCEVTCLR.all = 0x0333;
}

void CMPSS_STS_LATCH_CLEAR(void){
	EALLOW;

	Cmpss1Regs.COMPSTSCLR.all = 0x0202;
	Cmpss2Regs.COMPSTSCLR.all = 0x0202;
	Cmpss3Regs.COMPSTSCLR.all = 0x0202;
	Cmpss4Regs.COMPSTSCLR.all = 0x0202;
	Cmpss5Regs.COMPSTSCLR.all = 0x0202;
	Cmpss6Regs.COMPSTSCLR.all = 0x0202;
	Cmpss7Regs.COMPSTSCLR.all = 0x0202;
	Cmpss8Regs.COMPSTSCLR.all = 0x0202;
	EDIS;
}
void PWM1_Enable(void)
{
	EALLOW;
	EPwm1Regs.TZCLR.all = 0xFF;
	EPwm2Regs.TZCLR.all = 0xFF;
	EPwm3Regs.TZCLR.all = 0xFF;
	EPwm1Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;	
	EPwm1Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;	
	EPwm2Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;	
	EPwm2Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;
	EPwm3Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;	
	EPwm3Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;
	EDIS;
}

void PWM1_Disable(void)
{
	EALLOW;
	EPwm1Regs.TZCTL.bit.TZA = TZ_HIZ;	
	EPwm1Regs.TZCTL.bit.TZB = TZ_HIZ;	
	EPwm2Regs.TZCTL.bit.TZA = TZ_HIZ;	
	EPwm2Regs.TZCTL.bit.TZB = TZ_HIZ;
	EPwm3Regs.TZCTL.bit.TZA = TZ_HIZ;	
	EPwm3Regs.TZCTL.bit.TZB = TZ_HIZ;
	EPwm1Regs.TZFRC.bit.OST = TZ_ENABLE;
	EPwm2Regs.TZFRC.bit.OST = TZ_ENABLE;
	EPwm3Regs.TZFRC.bit.OST = TZ_ENABLE;
	EDIS;
}

void PWM2_Enable(void)
{
	EALLOW;
	EPwm4Regs.TZCLR.all = 0xFF;
	EPwm5Regs.TZCLR.all = 0xFF;
	EPwm6Regs.TZCLR.all = 0xFF;
	EPwm4Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;	
	EPwm4Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;	
	EPwm5Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;	
	EPwm5Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;
	EPwm6Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;	
	EPwm6Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;
	EDIS;
}

void PWM2_Disable(void)
{
	EALLOW;
	EPwm4Regs.TZCTL.bit.TZA = TZ_HIZ;	
	EPwm4Regs.TZCTL.bit.TZB = TZ_HIZ;	
	EPwm5Regs.TZCTL.bit.TZA = TZ_HIZ;	
	EPwm5Regs.TZCTL.bit.TZB = TZ_HIZ;
	EPwm6Regs.TZCTL.bit.TZA = TZ_HIZ;	
	EPwm6Regs.TZCTL.bit.TZB = TZ_HIZ;
	EPwm4Regs.TZFRC.bit.OST = TZ_ENABLE;
	EPwm5Regs.TZFRC.bit.OST = TZ_ENABLE;
	EPwm6Regs.TZFRC.bit.OST = TZ_ENABLE;
	EDIS;
}
void PWM3_Enable(void)
{
	EALLOW;
	EPwm7Regs.TZCLR.all = 0xFF;
	EPwm7Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;	
	EPwm7Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;	
	
	EDIS;
}

void PWM3_Disable(void)
{
	EALLOW;
	EPwm7Regs.TZCTL.bit.TZA = TZ_HIZ;	
	EPwm7Regs.TZCTL.bit.TZB = TZ_HIZ;	
	EPwm7Regs.TZFRC.bit.OST = TZ_ENABLE;
	EDIS;
}
/*******************************************************************/
/* FLT_GetTableAddr - 고장테이블 Address                           */
/*******************************************************************/
Uns *FLT_GetTableAddr( void )
{
	return	FLT.ariTable;
}	
