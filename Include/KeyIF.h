/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: WBST MCU UPDATE HYUNDAI Elevator
	Compiler    	: TMS28335 C-Compiler v5.3
	Author				: Inverter part, E/C Department
	Version				: v1.0
	Last Rev.			: 2015.6.3
******************************************************************************/
 
#ifndef __KEYIF_H_
#define __KEYIF_H_

#include "gnl.h"


float KEY_GetControlCode(int ControlID);
float KEY_GetInterfaceCode(int InterfaceID);
float KEY_GetMotorCode(int MotorID);
float KEY_GetFactoryCode(int FactoryID);

void KEY_SetBasicCode(int BasicID, float value);
void KEY_SetControlCode(int ControlID, float value);
void KEY_SetInterfaceCode(int InterfaceID, float value);
void KEY_SetMotorCode(int MotorID, float value);
void KEY_SetFactoryCode(int FactoryID, float value);

#endif	/* __KEYIF_H_ */

