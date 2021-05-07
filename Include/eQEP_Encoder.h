/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: FREEWAY
	Compiler    		: TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.04.18
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
******************************************************************************/
#ifndef _EQEP_ENCODER_H
#define _EQEP_ENCODER_H

#include "F28x_Project.h"
#include "comp.h"
#include "Variable.h"

typedef struct {
	Uint32  ppr;					// Pulse Per Revolution
	Uint32  mode;	    			// Multiplication Factor
	Uint32  maxCount;

	Uint32  Pos_cnt_spd;
	int32	Pos_cnt;
	Uint32 	Zsp_Pos_cnt;
	Uint32 	Zsp_Pos_cnt_max;
	Uint32 	Zsp_Pos_cnt_old;
	int32   oldPos_cnt;            // Input:
	int32	delPos_cnt;
	Uint32	Utimer_cnt;
	Uint32	Utimer_cnt_old;
	Uint32	QCtimer_cnt;
	Uint32 	QCtimer_cnt_old;
	Uint32	QCtimer_COEF_cnt;
	int32  	del_timer_cnt;
	Uint32	timer_cnt;
	Uint32 	timer_cnt_old;

	float32	theta_elec;         	// Output: Motor Electrical angle
	float32	theta_mech;         	// Output: Motor Mechanical Angle
	float32	theta_init;
	int 	DirectionQep;       	// Output: Motor rotation direction

	float32	mech_scale;        		// Parameter
	float32	pole_pairs;         	// Parameter: Number of pole pairs
	float32 pole_pitch;				// Parameter: Pole pitch(0.024)
	float32 inv_pole_pitch;

	float32	speed_mech;     		// Wrm, Parameter :  Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0) - independently with global Q
	float32	speed_elec;				// Wr
	float32	speed_rpm;
	float32 position;				// Position for Theta CCal.
	float32 position_;              // Position for Position Control
	float32 oldPosion;
	float32 oldPosion2;
	float32	speed_mech2;     		// Wrm, Parameter :  Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0) - independently with global Q
	float32	speed_elec2;			// Wr
	float32 Wr;

	int 	err_cnt;
	
	int32 	uSin_AD, uCos_AD;
	float32	fSin_Data, fCos_Data; 
	float32	fSin_Data_L, fCos_Data_L;
	float32	fSin_Max_Data_L, fCos_Max_Data_L,fSin_Max_Data_L_old, fCos_Max_Data_L_old;
	float32	fSin_Min_Data_L, fCos_Min_Data_L,fSin_Min_Data_L_old, fCos_Min_Data_L_old;
	
	float32	fixSinData, fixCosData, CosMag_K, SinVolt, CosVolt, SinCosThetarm, SinCosThetar; 
	float32	fSin_Max, fSin_Min, fCos_Max, fCos_Min, SinCosMag, SinCosMag_old;
	volatile struct EQEP_REGS *EQepxRegs;

	float32 PosCntOffset;

	/*  Real Acceleration & Real Jerk */
	float32 oldspeed_elec;
	float32 acceleration;
	float32 oldAcceleration;
	float32 jerk;

}Encoder_str;


extern Encoder_str Encoder_SNU;
extern Encoder_str Encoder_HD;

void InitEnc(Encoder_str *p, Uint32 ppr,float polepair, float theta_mech_present, volatile struct EQEP_REGS *EQepxRegs);
//void Calc_Enc_Pos_Spd(Encoder_str *p);
void Calc_Enc_Pos_Spd(Encoder_str *p);
void Calc_Enc_Pos(Encoder_str *p);
void Calc_Enc_Spd(Encoder_str *p);

#endif  // end of CPU1_EQEP_ENCODER_H definition


