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
#include "eQEP_Encoder.h"

Encoder_str Encoder_SNU;
Encoder_str Encoder_HD;
#pragma DATA_SECTION(Encoder_HD, "CLADataLS0");

void InitEnc(Encoder_str *p, Uint32 ppr,float polepair, float theta_mech_present, volatile struct EQEP_REGS *EQepxRegs)
{
	float temp_theta;

	p->EQepxRegs = EQepxRegs;

	p->ppr = ppr;
	p->mode = 4; 							//default X4

	if(SYNorLIN == 1){						// Synchronous Motor Selected
		p->maxCount = p->ppr*p->mode-1;
	}
	else if(SYNorLIN == 2){					// Linear Motor Selected
		p->maxCount = p->ppr;				// 32bit = 4294967296 --> Type Casting float to Uint32, It becomes to 4294967295
	}
	
	temp_theta = BOUND_PI(theta_mech_present);
	if(temp_theta<0.) temp_theta= temp_theta+TWOPI;

	p->EQepxRegs->QPOSINIT = (Uint32)( (float)(p->maxCount+1)*(temp_theta*INV_2PI) );
	p->EQepxRegs->QEPCTL.bit.SWI = 1;		// software initilization of position counter

	p->pole_pitch = KEY_GetLinMotorCode(LIN_MOTOR_POLE_PITCH)*0.001;
	p->inv_pole_pitch = 1/p->pole_pitch;
	
	if(SYNorLIN_Motor == 1){
		p->pole_pairs = polepair;
		p->mech_scale = __divf32(TWOPI, (float)(p->maxCount+1));
	}
	else if(SYNorLIN_Motor == 2){
		p->pole_pairs = polepair;			// Linear Motor's pole pair is 1.
		p->mech_scale = __divf32(TWOPI, (float)(p->pole_pitch));
	}

	p->Pos_cnt = p->EQepxRegs->QPOSINIT;
	p->Pos_cnt_spd = p->EQepxRegs->QPOSINIT;
	p->oldPos_cnt = p->EQepxRegs->QPOSINIT;
	p->delPos_cnt = 0;
	p->Zsp_Pos_cnt = 0;
	p->Zsp_Pos_cnt_max = 20;
	p->Zsp_Pos_cnt_old = 0;
	p->Utimer_cnt = 0;
	p->QCtimer_cnt = 0;
	p->QCtimer_cnt_old = 0;
	p->del_timer_cnt = 0;
	p->DirectionQep = 0;
	p->theta_elec = 0.;
	p->theta_mech = 0.;
	//p->theta_init = 0.;
	p->speed_mech = 0.;
	p->speed_elec = 0.;
	p->speed_rpm = 0.;
	p->position = 0.;
	p->oldPosion = 0.;
	p->oldPosion2 = 0.;
	p->speed_mech2 = 0.;
	p->speed_elec2 = 0.;
	p->Wr = 0.;

	p->EQepxRegs->QPOSMAX = p->maxCount; // This register contains the maximum position counter value. (0 ~ PPR*MODE - 1)
	p->EQepxRegs->QUPRD = 0xFFFFFFFF;
	p->EQepxRegs->QUTMR = 0;
	p->EQepxRegs->QCTMR = 0;

	p->err_cnt =0;
	
	p->uSin_AD = 0;
	p->fSin_Data = 0.;
	p->uCos_AD = 0;
	p->fCos_Data = 0.;
	p->fSin_Data_L = 0.;
	p->fCos_Data_L = 0.;
	p->fSin_Max_Data_L_old = 0.;
	p->fCos_Max_Data_L_old = 0.;
	p->fSin_Min_Data_L_old = 0.;
	p->fCos_Min_Data_L_old = 0.;
	p->fixSinData = 0;
	p->fixCosData = 0; 
	p->SinVolt = 0; 
	p->CosVolt = 0;
//	p->SinCosThetarm = 0;
//	p->SinCosThetar = 0;
	
	p->SinCosMag = 0; 
	p->SinCosMag_old = 0;

//	p->PosCntOffset = 0;
}

void Calc_Enc_Pos_Spd(Encoder_str *p)
{
	// QEP Unit Timer: This register acts as time base for unit time event generation. 
	// When this timer value matches with unit time period value, unit time event is generated.
	p->Utimer_cnt = p->EQepxRegs->QUTMR;				// read UTIMER
	if(p->EQepxRegs->QEPSTS.bit.UPEVNT==1)				// Unit position event
	{
		//**** Position calculation - mechanical and electrical motor angle  ****//
		// read position and time, direction
		p->Utimer_cnt = p->EQepxRegs->QUTMR;			// read UTIMER
		/// Position Counter: This 32-bit position counter register counts up/down on every eQEP pulse based on direction input.
		/// This counter acts as a position integrator whose count value is proportional to position from a give reference point.
		p->Pos_cnt = p->EQepxRegs->QPOSCNT;
		/// The eQEP capture timer value can be latched into this register on two events viz., until timeout event, reading the eQEP position counter.
		p->EQepxRegs->QUTMR = p->EQepxRegs->QCTMRLAT;	// write UTIMER count
		//p->EQepxRegs->QUTMR = 0;						// write UTIMER count
		p->QCtimer_cnt = p->EQepxRegs->QCTMRLAT;	
		if(p->EQepxRegs->QCTMRLAT < 50){				// 가끔 레치 타임과 poscnt 읽는 순간이 차이가 나서 펄스 하나를 놓치는 경우가 발생. 두번 읽어서 해결
			//read position and time, direction
			p->Utimer_cnt = p->EQepxRegs->QUTMR;		// read UTIMER
			p->Pos_cnt = p->EQepxRegs->QPOSCNT;
			//p->EQepxRegs->QUTMR = 0;					// write UTIMER count	
		 	p->QCtimer_cnt = p->EQepxRegs->QCTMRLAT;
		}
		p->DirectionQep = p->EQepxRegs->QEPSTS.bit.QDF;	// Motor direction: 0=CCW/reverse, 1=CW/forward

		p->EQepxRegs->QEPSTS.bit.UPEVNT =1; 			// for clear

		//**** Speed Calculation using M/T method****//
		p->delPos_cnt = p->Pos_cnt - p->oldPos_cnt;

		/// Position counter overflow interrupt flag --> 0: No interrupt generated, 1: Interrupt was generated.
		if(p->EQepxRegs->QFLG.bit.PCO){					// over flow  가 일어나면
			if(p->Pos_cnt == p->maxCount){
				p->err_cnt++;
			}
			else{
				p->delPos_cnt = p->delPos_cnt+(p->maxCount+1);
				p->EQepxRegs->QCLR.bit.PCO =1;
			}

		}
		else if(p->EQepxRegs->QFLG.bit.PCU){			//under flow 가 일어나면
			if(p->Pos_cnt == 0){
				p->err_cnt++;
			}
			else{
				p->delPos_cnt = p->delPos_cnt-(p->maxCount+1);
				p->EQepxRegs->QCLR.bit.PCU =1;
			}
		}
		p->oldPos_cnt = p->Pos_cnt;

		/// Capture overflow error flag --> 0: Capture direvtion error has not occurred. 1: Direction chage occurred between the capture position event. 
		/// This bit is cleared by writting a '1' to the corresponding bit in the QCLR register.
		if(p->EQepxRegs->QEPSTS.bit.COEF){ 				//time out
			p->speed_mech =0.;
			p->speed_elec =0.;
			p->speed_rpm =0.;
			p->EQepxRegs->QEPSTS.bit.COEF = 1;
		}
		else{;}
    }
    else{;}
    if(SYNorLIN_Motor == 1){
		// The following lines calculate p->theta_mech
		p->theta_mech = BOUND_PI( ((float)(p->Pos_cnt)) * p->mech_scale);
		// The following lines calculate p->theta_elec
		p->theta_elec = BOUND_PI(p->pole_pairs*p->theta_mech + p->theta_init);
		p->del_timer_cnt = p->Utimer_cnt - p->QCtimer_cnt;
		p->speed_mech = __divf32( SYS_CLK*((float)(p->delPos_cnt)*p->mech_scale), (float)(p->del_timer_cnt) );	// Wrm
		p->speed_elec = (p->pole_pairs*p->speed_mech);	// Wr
		p->speed_rpm = (p->speed_mech*Rm2Rpm);
		p->QCtimer_cnt_old = p->QCtimer_cnt;
	}
	else if(SYNorLIN_Motor == 2){
        p->position = ((float)p->Pos_cnt) * __ENC_RESOLUTION;                                         // Not Verifying ....
        p->position_ = ((float)(p->Pos_cnt + p->PosCntOffset)) * __ENC_RESOLUTION;
		// The following lines calculate p->theta_mech
		p->theta_mech = BOUND_PI( p->position * p->mech_scale);
		// The following lines calculate p->theta_elec
		p->theta_elec = BOUND_PI(p->pole_pairs*p->theta_mech + p->theta_init);
		// The following lines calculate p->theta_elec
		p->speed_mech = __divf32((float)(p->position - p->oldPosion), Tsc); 	// Vrm
		p->speed_elec = (p->pole_pairs*p->speed_mech);							// Vr
		p->acceleration = (p->speed_elec - p->oldspeed_elec)*INV_Tsc;           // Real Acceleration
		p->jerk = (p->acceleration - p->oldAcceleration)*INV_Tsc;               // Real Jerk
		p->oldPosion = p->position;
		p->oldspeed_elec = p->speed_elec;
		p->oldAcceleration = p->acceleration;
	}
}

void Calc_Enc_Spd(Encoder_str *p)
{
	//**** Position calculation - mechanical and electrical motor angle  ****//
	//read position and time, direction
	p->Utimer_cnt = p->EQepxRegs->QUTMR;			// read UTIMER
	p->Pos_cnt_spd = p->EQepxRegs->QPOSCNT;
	p->QCtimer_cnt = p->EQepxRegs->QCTMRLAT;
	if(p->EQepxRegs->QCTMRLAT < 50){				// 가끔 레치 타임과 poscnt 읽는 순간이 차이가 나서 펄스 하나를 놓치는 경우가 발생. 두번 읽어서 해결
	//read position and time, direction
		p->Utimer_cnt = p->EQepxRegs->QUTMR;		// read UTIMER
		p->Pos_cnt_spd = p->EQepxRegs->QPOSCNT;
		p->QCtimer_cnt = p->EQepxRegs->QCTMRLAT;
	}
	//**** Speed Calculation using M/T method****//
	p->delPos_cnt = p->Pos_cnt_spd - p->oldPos_cnt;
	if(p->EQepxRegs->QFLG.bit.PCO){					//over flow  가 일어나면
		if(p->Pos_cnt_spd == p->maxCount){
			p->err_cnt++;
		}
		else{
			p->delPos_cnt =p->delPos_cnt+(p->maxCount+1);
			p->EQepxRegs->QCLR.bit.PCO =1;
		}
	}
	else if(p->EQepxRegs->QFLG.bit.PCU){			//under flow 가 일어나면
		if(p->Pos_cnt_spd == 0){
			p->err_cnt++;
		}
		else{
			p->delPos_cnt =p->delPos_cnt-(p->maxCount+1);
			p->EQepxRegs->QCLR.bit.PCU =1;
		}
	}
	p->oldPos_cnt = p->Pos_cnt_spd;
	if(p->EQepxRegs->QEPSTS.bit.COEF){ 
		p->QCtimer_COEF_cnt++;
		p->EQepxRegs->QEPSTS.bit.COEF = 1;
	}
	p->del_timer_cnt = p->timer_cnt - p->timer_cnt_old;
	p->timer_cnt = p->Utimer_cnt - (p->QCtimer_COEF_cnt*0x10000 + p->QCtimer_cnt)*32;	//capclk is sysclkout/32
	p->timer_cnt_old = p->timer_cnt;
	if(p->delPos_cnt != 0){
		p->Zsp_Pos_cnt = 0;
		if(p->del_timer_cnt == 0) p->del_timer_cnt = 1;
		p->speed_mech = __divf32( SYS_CLK*((float)(p->delPos_cnt)*1e-6), (float)(p->del_timer_cnt) );
		p->speed_elec = (p->pole_pairs*p->speed_mech);
//		p->speed_rpm = (p->speed_mech*Rm2Rpm);
	}
	if(++p->Zsp_Pos_cnt > p->Zsp_Pos_cnt_max){
		p->Zsp_Pos_cnt = p->Zsp_Pos_cnt_max;
		p->speed_mech = 0.;
		p->speed_elec = 0.;
		p->speed_rpm = 0.;
		p->QCtimer_COEF_cnt = 0;
	}
	if((p->Zsp_Pos_cnt_old == p->Zsp_Pos_cnt_max)&&(p->Zsp_Pos_cnt == 1)){				// 1 pulse input but speed is zero.
		p->speed_mech = 0.;
		p->speed_elec = 0.;
		p->speed_rpm = 0.;
		p->QCtimer_COEF_cnt = 0;
	}
	p->Zsp_Pos_cnt_old = p->Zsp_Pos_cnt;
	p->QCtimer_COEF_cnt = 0;
}

/****************************************************************************/
/*		Position and Electrical/Mechinally Angle Calculation Function		*/
/* Author: Gogume															*/
/* History: Modified 20180418												*/
/****************************************************************************/
void Calc_Enc_Pos(Encoder_str *p)
{
	p->DirectionQep = p->EQepxRegs->QEPSTS.bit.QDF;    	// Motor direction: 0=CCW/reverse, 1=CW/forward
	if(p->EQepxRegs->QEPSTS.bit.UPEVNT==1){				// Unit position event
		//read position, direction
		p->Pos_cnt = p->EQepxRegs->QPOSCNT;
		if(p->EQepxRegs->QCTMRLAT <50){					// 가끔 레치 타임과 poscnt 읽는 순간이 차이가 나서 펄스 하나를 놓치는 경우가 발생. 두번 읽어서 해결
			//read position and time, direction
			p->Pos_cnt = p->EQepxRegs->QPOSCNT;
		}
		p->QCtimer_COEF_cnt = 0;
		p->EQepxRegs->QEPSTS.bit.UPEVNT =1; 			// for clear
	}
//	p->delPos_cnt = p->Pos_cnt - p->oldPos_cnt;
//	p->oldPos_cnt = p->Pos_cnt_spd;

	if(SYNorLIN_Motor == 1){
		// The following lines calculate p->theta_mech
		p->theta_mech = BOUND_PI( ((float)(p->Pos_cnt)) * p->mech_scale);
		// The following lines calculate p->theta_elec
		p->theta_elec = BOUND_PI(p->pole_pairs*p->theta_mech + p->theta_init);

	}
	else if(SYNorLIN_Motor == 2){
		p->position = ((float)p->Pos_cnt) * __ENC_RESOLUTION;
		p->position_ = ((float)(p->Pos_cnt + p->PosCntOffset)) * __ENC_RESOLUTION;

		// The following lines calculate p->theta_mech
		p->theta_mech = BOUND_PI( p->position * p->mech_scale);
		// The following lines calculate p->theta_elec
		p->theta_elec = BOUND_PI(p->pole_pairs*p->theta_mech + p->theta_init);
		p->speed_mech2 = __divf32((float)(p->position - p->oldPosion2), Tsamp); 	// Vrm
		p->speed_elec2 = (p->pole_pairs*p->speed_mech);								// Vr
		p->Wr = __divf32(TWOPI*(p->speed_elec2), p->pole_pitch);
		p->oldPosion2 = p->position;
	}
}
