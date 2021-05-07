/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: FREEWAY 
	Compiler    		: TMS320F28377D C-Compiler v6.1.2
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.02.06
	History				: Linear Motor Selection Menu is added into the HHT Menu (20180206 by GWON)
						: Motor Side R Controller is added in order to suppress 3th, 6th 9th harmonics, but this controllers are disable. (20180206 by GWON)
******************************************************************************/
#include "Variable.h"
#include <math.h>
#include "R_control.h"
//2014-01-02
//made by JS(SNU EEPEL)

// R controller //
int Flag_Resonant=0;
int Flag_Resonant_reset=0;
float ERR_Idss=0.,ERR_Iqss=0.;


void Init_R_control_var(R_control *CON){
	CON->input[0] 	= 0.;
	CON->input[1] 	= 0.;
	CON->output[0] 	= 0.;
	CON->output[1] 	= 0.;
//	CON->Ki 		= 0.;
//	CON->Kp 		= 0.;
//	CON->a 			= 0.;
//	CON->b 			= 0.;
//	CON->w0 		= 0.;
//	CON->wcc 		= wcc;
}

extern Motor INV1;
//void Updata_R_control_var(R_control *CON,float w0,float Ki,float Kp){
void Updata_R_control_var(R_control *CON,float w0,float wcc){ //w0 공진 주파수, wcc 가 bandwidth
	CON->wcc =wcc;
	CON->w0 = w0;
	CON->a =-2*__cos(w0*Tsamp);
	CON->b =__divf32(__sin(w0*Tsamp), (w0) );

	//CON->Ki = Ki;
	//CON->Kp = Kp;
	//CON->Kp = (INV1.Lg+INV1.Lc)*wcc;
	CON->Kp =0.;
	CON->Ki = INV2.Rs*wcc;
}

float Resonant_controller(R_control *CON,float input1){
	float output1 =0.;
	output1 = CON->Ki*CON->b*(input1-CON->input[1])-(CON->a*CON->output[0]+CON->output[1]);
	output1 = output1 + CON->Kp*input1;
	CON->input[1] = CON->input[0];
	CON->input[0] = input1;
	CON->output[1] = CON->output[0];
	CON->output[0] = output1;

	return output1; 
}


