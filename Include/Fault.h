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
						: TX_INDEX 20 --> 40 (20180806 by GWON)
						: Added SCI Protocol Version Response Part (20181107 by GWON)
						: Added SCI Protocol Infmation Response Part (20181107 by GWON)
						: Rearrangement of F/W for FREEWAY (20181112 by GWON)
******************************************************************************/
#ifndef _FAULT_H
#define _FAULT_H

#include "F28x_Project.h"
#include "gnl.h"

__interrupt void epwm1_tzint_isr(void);
__interrupt void epwm2_tzint_isr(void);
__interrupt void cpu1_igbt3flt_isr(void);

void InitFault(void);
void ADCEVT_PPB_CLEAR(void);
void CMPSS_STS_LATCH_CLEAR(void);

/////////////////////////////////////////////////////////////////////////////////////////
#define FLT_TABLE_NUM	10

#define FLT_ON		1
#define FLT_OFF		0

/**************************************************************************/
/* Definition for FLT ID(Identification) for HELCON                       */
/**************************************************************************/
/* Main Catagory: DSP (Vertical: 0x20; Horizontal: 0x21) */
#define FLT_15V				113
#define FLT_5V				114
#define FLT_DCC				115
#define FLT_PGBK			116
#define FLT_ITH				117
#define	FLT_VERSION			206
#define FLT_CP_WD			227
#define FLT_CP_COM			248
#define FLT_OS_OBS			249
/* Main Catagory: INVETER (Vertical: 0x40; Horizontal: 0x41) */
#define FLT_OCAH_C			101
#define FLT_OCAH_I			102
#define FLT_OCBH_C			103
#define FLT_OCBH_I			104
#define FLT_OCCH_C			105
#define FLT_OCCH_I			106
#define FLT_OVDCH			107
#define FLT_MOSFET_C 		110
#define FLT_MOSFET_I 		111
#define FLT_OS				202
#define FLT_DLA	      		203
#define FLT_MCC				204
#define FLT_OL       		205
#define FLT_OCBS_C			207
#define FLT_OCBS_I			208
#define FLT_OCAS_C			209
#define FLT_OCAS_I			210
#define FLT_OCCS_C			211
#define FLT_OCCS_I			212
#define FLT_OVDCS			215
#define FLT_UVDCS			219
#define FLT_SPDCTL			222
#define FLT_OUTPUT			225
#define FLT_DBL_CMD			226
#define FLT_EARTH_I			230
#define FLT_EARTH_C			231
#define FLT_CMD_OFF			232
#define FLT_OFFSET			235
#define FLT_ENABLE			236
#define FLT_FRAM			240
#define FLT_CC_OCS_I		241
#define FLT_CC_OCS_C		242
#define FLT_CC_OVDC			244
#define FLT_OS_OBS			249
#define FLT_RUN_PLL			250
#define FLT_UNKNOWN			255
/* Main Catagory: BRAKE (Vertical: 0x90; Horizontal: 0x91) */
#define FLT_BKOP			220
/* Main Catagory: ENCODER (Vertical: 0xA0; Horizontal: 0xA1) */
#define	FLT_ENC_UVW_SEQ		221
#define FLT_OV_ANGLE		223
#define FLT_UVW				233

/*
 H/W 검출 Fault ID H/W detection, H/W process
#define FLT_OCAH_C			101
#define FLT_OCAH_I			102
#define FLT_OCBH_C			103
#define FLT_OCBH_I			104
#define FLT_OCCH_C			105
#define FLT_OCCH_I			106
#define FLT_OVDCH			107
#define FLT_OCH_SUP			108
#define FLT_OVH_SUP			109
#define FLT_MOSFET_C 		110
#define FLT_MOSFET_I 		111
#define FLT_IGBT_SUP		112
 S/W 검출 Fault 1 ID -> H/W detection, S/W process
#define FLT_15V				113
#define FLT_5V				114
#define FLT_DCC				115
#define FLT_PGBK			116
#define FLT_ITH				117
 S/W 검출 Fault 2 ID S/W detection, S/W process
#define FLT_LIMITSW			201
#define FLT_OS				202
#define FLT_DLA	      		203
#define FLT_MCC				204
#define FLT_OL       		205
#define	FLT_VERSION			206

#define FLT_OCBS_C			207
#define FLT_OCBS_I			208
#define FLT_OCAS_C			209
#define FLT_OCAS_I			210
#define FLT_OCCS_C			211
#define FLT_OCCS_I			212
#define FLT_OVBCS_C			213
#define FLT_OVABS_C			214
#define FLT_OVDCS			215
#define FLT_OCS_SUP			216
#define FLT_OVS_SUP			217
			
#define FLT_SEQ_C			218
#define FLT_UVDCS			219
#define FLT_BKOP			220
#define	FLT_ENC_UVW_SEQ		221
#define FLT_SPDCTL			222
#define FLT_OV_ANGLE		223
#define FLT_ENC_UVW			224
#define FLT_OUTPUT			225
#define FLT_DBL_CMD			226
#define FLT_CP_WD			227
#define FLT_MODE			228
#define FLT_OVHFUS			229
#define FLT_EARTH_I			230
#define FLT_EARTH_C			231
#define FLT_CMD_OFF			232
#define FLT_UVW				233
#define FLT_CMD_CFM			234
#define FLT_OFFSET			235
#define FLT_ENABLE			236

#define FLT_GRID_PLL_FREQ	237
#define FLT_GRID_PLL_OV		238
#define FLT_GRID_PLL_UV		239
#define FLT_FRAM			240

#define FLT_CC_OCS_I		241
#define FLT_CC_OCS_C		242
#define FLT_CC_OCS_SUP		243
#define FLT_CC_OVDC			244
#define FLT_CC_OVDC_SUP		245
#define FLT_OUTPUT_SUP		246
#define FLT_VDCSEN_SUP		247

#define FLT_CP_COM			248
#define FLT_OS_OBS			249

#define FLT_RUN_PLL			250
#define FLT_ESS_WRN			251

#define FLT_UNKNOWN			999
*/

typedef struct
{
	Uns ariTable[FLT_TABLE_NUM];
	int iTableIndex;
	int iTableIndex1;
	Uns uFlag;
	Uns uFltNum;
} Flt;

/**************************************************************************/
/* FLT Methods                                                            */
/**************************************************************************/
void FLT_Create( void ); /* FLT 초기화 */
void FLT_CheckHW(void);
void FLT_Raise( Uint16 FltNum );
void FLT_WarnRaise(Uint16 FltNum);
void FLT_Reset( void );
Uns *FLT_GetTableAddr( void );	
void PWM1_Enable(void);
void PWM1_Disable(void);
void PWM2_Enable(void);
void PWM2_Disable(void);
void PWM3_Enable(void);
void PWM3_Disable(void);
///////////////////////////////////////////////////////////////////////////////////////

// FAULT 신호 1차 정리 하는 곳
struct FAULT_CMPSS_BITS {  		// bits description
    Uint16 CMPSS1_A2_H:1;  		//  0: CMPSS1 High(Inv2.Ibs)
    Uint16 CMPSS1_A2_L:1;  		//  1: CMPSS1 LOW
    Uint16 CMPSS2_A4_H:1;  		//  2: CMPSS2 High(Inv1.Ibs)
    Uint16 CMPSS2_A4_L:1;  		//  3: CMPSS2 LOW
    Uint16 CMPSS3_B2_H:1;  		//  4: CMPSS3 High(Inv2.Ics)
    Uint16 CMPSS3_B2_L:1;  		//  5: CMPSS3 LOW
    Uint16 CMPSS4_C14_H:1; 		//  6: CMPSS4 High(Inv1.Ics)
    Uint16 CMPSS4_C14_L:1; 		//  7: CMPSS4 LOW
    Uint16 CMPSS5_C4_H:1;  		//  8: CMPSS5 High(Inv1.Ias)
    Uint16 CMPSS5_C4_L:1;  		//  9: CMPSS5 LOW
    Uint16 CMPSS6_C2_H:1;  		// 10: CMPSS1 High(Inv2.Ias)
    Uint16 CMPSS6_C2_L:1;  		// 11: CMPSS1 LOW
    Uint16 CMPSS7_D0_H:1;  		// 12: CMPSS1 High(Vdc)
    Uint16 CMPSS7_D0_L:1;  		// 13: CMPSS1 LOW
    Uint16 CMPSS8_D2_H:1;  		// 14: CMPSS1 High(Inv3.Iqse)
    Uint16 CMPSS8_D2_L:1;  		// 15: CMPSS1 LOW
};
union FAULT_CMPSS_REG {
    Uint16  all;
    struct  FAULT_CMPSS_BITS  bit;
};

//Adcd ch : SOC0=0(Vdc), SOC1=2(Inv3.Iqse), SOC2=4(Vdc_sup)
struct FAULT_ADCEVT_BITS {		// bits description
	Uint32 A_PPB1_H:1;    		//  0: ADC A PPB1 CMP High(Inv2.Ibs)
	Uint32 A_PPB1_L:1;    		//  1: ADC A PPB1 CMP Low
	Uint32 A_PPB2_H:1;    		//  2: ADC A PPB2 CMP High(Inv1.Ibs)
	Uint32 A_PPB2_L:1;    		//  3: ADC A PPB2 CMP Low
	Uint32 A_PPB3_H:1;   		//  4: ADC A PPB3 CMP High(GRID1.Vbc)
	Uint32 A_PPB3_L:1;    		//  5: ADC A PPB3 CMP Low
	Uint32 B_PPB1_H:1;    		//  6: ADC B PPB1 CMP High(Inv2.Ics)
	Uint32 B_PPB1_L:1;    		//  7: ADC B PPB1 CMP Low
	Uint32 B_PPB2_H:1;    		//  8: ADC B PPB2 CMP High(Inv1.Ics)
	Uint32 B_PPB2_L:1;    		//  9: ADC B PPB2 CMP Low
	Uint32 B_PPB3_H:1;   		// 10: ADC B PPB3 CMP High(GRID1.Vab)
	Uint32 B_PPB3_L:1;    		// 11: ADC B PPB3 CMP Low
	Uint32 C_PPB1_H:1;    		// 12: ADC C PPB1 CMP High(Inv2.Ias)
	Uint32 C_PPB1_L:1;    		// 13: ADC C PPB1 CMP Low
	Uint32 C_PPB2_H:1;    		// 14: ADC C PPB2 CMP High(Inv1.Ias)
	Uint32 C_PPB2_L:1;    		// 15: ADC C PPB2 CMP Low
	Uint32 C_PPB3_H:1;    		// 16: ADC C PPB3 CMP High(no use)
	Uint32 C_PPB3_L:1;    		// 17: ADC C PPB3 CMP Low
	Uint32 D_PPB1_H:1;    		// 18: ADC D PPB1 CMP High(Vdc)
	Uint32 D_PPB1_L:1;    		// 19: ADC D PPB1 CMP Low
	Uint32 D_PPB2_H:1;    		// 20: ADC D PPB2 CMP High(Inv3.Iqse)
	Uint32 D_PPB2_L:1;    		// 21: ADC D PPB2 CMP Low
	Uint32 D_PPB3_H:1;    		// 22: ADC D PPB3 CMP High(Vdc_sup)
	Uint32 D_PPB3_L:1;    		// 23: ADC D PPB3 CMP Low
	Uint32 rsvd:7;        		// 31:24: Reserved
};
union FAULT_ADCEVT_REG {
	Uint32  all;
    struct  FAULT_ADCEVT_BITS  bit;
};

struct FAULT_DCAEVT1_REG {
	Uint16	FAULT_DCAEVT1_FLG_LAT;
    union   FAULT_CMPSS_REG		FAULT_CMPSS;	// 아날로그 비교기 latch 정보
    union   FAULT_ADCEVT_REG	FAULT_ADCEVT;	// ADC 출력 디지털 비교기 latch 정보
};

struct FAULT_GPIO_STS_BITS {	// bits description
    Uint32 GFA_ALL:1;       	//  0: GPIO_State - FAULT_OR A
    Uint32 GFA0:1;          	//  1: GPIO_State - FAULT_A0
    Uint32 GFA1:1;          	//  2: GPIO_State - FAULT_A1
    Uint32 GFA2:1;          	//  3: GPIO_State - FAULT_A2
    Uint32 GFA3:1;          	//  4: GPIO_State - FAULT_A3
    Uint32 GFB_ALL:1;       	//  5: GPIO_State - FAULT_OR B
    Uint32 GFB0:1;          	//  6: GPIO_State - FAULT_B0
    Uint32 GFB1:1;          	//  7: GPIO_State - FAULT_B1
    Uint32 GFB2:1;          	//  8: GPIO_State - FAULT_B2
    Uint32 GFB3:1;          	//  9: GPIO_State - FAULT_B3
    Uint32 GFC_ALL:1;       	// 10: GPIO_State - FAULT_OR C
    Uint32 GFC0:1;          	// 11: GPIO_State - FAULT_C0
    Uint32 GFC1:1;          	// 12: GPIO_State - FAULT_C1
    Uint32 GFC2:1;          	// 13: GPIO_State - FAULT_C2
    Uint32 GFC3:1;          	// 14: GPIO_State - FAULT_C3
    Uint32 GFD_ALL:1;       	// 15: GPIO_State - FAULT_OR D
    Uint32 GFD0:1;          	// 16: GPIO_State - FAULT_D0
    Uint32 GFD1:1;          	// 17: GPIO_State - FAULT_D1
    Uint32 GFD2:1;          	// 18: GPIO_State - FAULT_D2
    Uint32 GFD3:1;          	// 19: GPIO_State - FAULT_D3
	Uint32 rsvd:12;         	// 20:31: Reserved

};

union FAULT_GPIO_STS_REG {
    Uint32  all;
    struct  FAULT_GPIO_STS_BITS  bit;
};

struct FAULT_ECT_BITS {                   // bits description
	Uint16 DSP_Clock_Fail:1;              // 0: clock fail
	Uint16 rsvd:15;                       // 15:1: Reserved
};
union FAULT_ECT_REG {					  // ect FAULT signal
	Uint32  all;
    struct  FAULT_ECT_BITS  bit;
};

struct FAULT_DCBEVT1_REG {
	Uint16	FAULT_DCBEVT1_FLG_LAT;
    union   FAULT_GPIO_STS_REG		FAULT_GPIO_STS;		//Gate 폴트 GPIO 입력 신호 정보
    union   FAULT_ECT_REG			FAULT_ECT;			//기타 폴스 신호들
};

struct FAULT_DSP_SIGNAL{
	struct   FAULT_DCAEVT1_REG		FAULT_DCAEVT1;		//EPWM 내 폴트 그룹 DCAEV1 내 폴트 신호들
	struct   FAULT_DCBEVT1_REG		FAULT_DCBEVT1;		//EPWM 내 폴트 그룹 DCBEV1 내 폴트 신호들
};

/*
struct CMPSS {
    union   ADCCTL1_REG                      ADCCTL1;                      // ADC Control 1 Register
    Uint32                                   ADCINLTRIM1;                  // ADC Linearity Trim 1 Register
    Uint32                                   ADCINLTRIM6;                  // ADC Linearity Trim 6 Register
    Uint16                                   rsvd7[4];                     // Reserved
};

union ADCCTL1_REG {
    Uint16  all;
    struct  ADCCTL1_BITS  bit;
};
struct ADCCTL1_BITS {                   // bits description
    Uint16 rsvd1:2;                     // 1:0 Reserved
    Uint16 INTPULSEPOS:1;               // 2 ADC Interrupt Pulse Position
    Uint16 rsvd2:4;                     // 6:3 Reserved
    Uint16 ADCPWDNZ:1;                  // 7 ADC Power Down
    Uint16 ADCBSYCHN:4;                 // 11:8 ADC Busy Channel
    Uint16 rsvd3:1;                     // 12 Reserved
    Uint16 ADCBSY:1;                    // 13 ADC Busy
    Uint16 rsvd4:2;                     // 15:14 Reserved
};*/

typedef union
{
	Uint16 all;
	struct
	{
		unsigned int OV_bf			:1; // nPROT0
		unsigned int OV_cf			:1; // nPROT1
		unsigned int OV_af			:1; // nPROT2
		unsigned int OC_Ics			:1; // nPROT3
		unsigned int OC_Ibs			:1; // nPROT4
		unsigned int OC_Ias			:1; // nPROT5
		unsigned int OC_dcH			:1; // nPROT6
		unsigned int OC_dcL			:1; // nPROT7
		unsigned int OT_b			:1; // nPROT8
		unsigned int OT_c			:1; // nPROT9
		unsigned int OV_dcH			:1; // nPROT10
		unsigned int OV_dcL			:1; // nPROT11
		unsigned int OV_grid_b		:1; // nPROT12
		unsigned int OV_grid_c		:1; // nPROT13
		unsigned int OV_grid_a		:1; // nPROT14
		unsigned int OC_grid_c		:1; // nPROT15
	}bit;
}HW_Fault;

typedef union
{
	Uint16 all;
	struct
	{
		unsigned int BGFA0			:1;
		unsigned int BGFA1			:1;
		unsigned int BGFA2			:1;
		unsigned int BGFA3			:1;
		unsigned int BGFB0			:1;
		unsigned int BGFB1			:1;
		unsigned int BGFB2			:1;
		unsigned int BGFB3			:1;
		unsigned int BGFC0			:1;
		unsigned int BGFC1			:1;
		unsigned int BGFC2			:1;
		unsigned int BGFC3			:1;
		unsigned int BGFD0			:1;
		unsigned int BGFD1			:1;
		unsigned int BGFD2			:1;
		unsigned int BGFD3			:1;
	}bit;
}GATE_Fault;

typedef union
{
	Uint16 all;
	struct
	{
		unsigned int OC_as			:1;
		unsigned int OC_bs			:1;
		unsigned int OC_cs			:1;
		unsigned int OV_af			:1;
		unsigned int OV_bf			:1;
		unsigned int OV_cf			:1;
		unsigned int OC_grid_a		:1;
		unsigned int OC_grid_b		:1;
		unsigned int OC_grid_c		:1;
		unsigned int OV_grid_a		:1;
		unsigned int OV_grid_b		:1;
		unsigned int OV_grid_c		:1;
		unsigned int OC_dcH			:1;
		unsigned int OC_dcL			:1;
		unsigned int OV_dcH			:1;
		unsigned int OV_dcL			:1;
	}bit;
}SW_Fault;

typedef struct
{
	HW_Fault	HW_Prot;
	GATE_Fault	GT_Prot;
	SW_Fault	SW_Prot;
}FAULT;
extern FAULT Fault;
extern u8 g_ErrorCode;


#endif  // end of CPU1_FAULT_H definition
