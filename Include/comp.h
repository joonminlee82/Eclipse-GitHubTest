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

#ifndef	_CUSTOMIZED_COMP
#define	_CUSTOMIZED_COMP


#if !defined(__TMS320C28XX_CLA__)
	#include <math.h>
#endif

/* Inverse Factorial */
#define		F2				(1./2.)
#define		F3				(F2/3.)
#define		F4				(F3/4.)
#define		F5				(F4/5.)
#define		F6				(F5/6.)
#define		F7				(F6/7.)
#define		F8				(F7/8.)
#define		F9				(F8/9.)
#define		F10				(F9/10.)
#define		F11				(F10/11.)
#define		F12				(F11/12.)
#define		F13				(F12/13.)
#define		F14				(F13/14.)
#define		F15				(F14/15.)

//#define TRUE			1
// Floating-point Constants
#define	PI			((float)3.141592653589793)
#define	PI_3		((float)1.047197551196598)
#define	TWOPI		((float)6.283185307179586)
#define	SQRT2		((float)1.414213562373095)
#define	SQRT3		((float)1.732050807568877)
#define	INV_SQRT3	((float)0.577350269189626)
#define	INV_SQRT2	((float)0.707106781186547)
#define	INV_3		((float)0.333333333333333)
#define INV_2PI		((float)0.159154943091895)
#define INV_PI		((float)0.318309886183791)
#define Rm2Rpm		((float)9.549296585513720)
#define Rpm2Rm		((float)0.104719755119659)
#define INV_2_3		((float)0.666666666666667)
#define	PI_INV_2	((float)(PI/2.))
#define	PI_INV_6	((float)(PI/6.))
#define PI5_INV_6 	((float)(5*PI/6.))
#define PI_INV_3  	((float)(PI/3.))
#define PI2_INV_3 	((float)(2*PI/3.))

#define GRAVITY		((float)9.80665)
#define INV_GRAVITY ((float)1/GRAVITY)
#define PI_POLE_PITCH PI/0.012

#define UNIT_ABS_POS    ((float)593.75)
#define THETA_SCALE     TWOPI / 593.75
#define REAL_POS_SCALE  593.75/TWOPI        // 593.75 is unit distance of scale.

#define		SQRT3_INV_SQRT2 	(SQRT3/SQRT2)
#define 	INV_SQRT3_80			0.4618802153517	//0.8 * 1/sqrt(3)

#define INV_6		0.16666666667
#define INV_24	0.04166666667
#define INV_4		0.25
#define INV_12	0.08333333333
#define INV_3_2	1.5
#define INV_3_4 0.75
#define INV_3_8 0.375
#define INV_4_3TH	0.015625
#define INV_4_3 1.33333333333
#define MPMTOMMPS 16.66666666667	// m/m --> mm/sec : 1000/60

#define	PI_L		((double)3.141592653589793)
#define	TWOPI_L		((double)6.283185307179586)
#define INV_2PI_L	((double)0.159154943091895)
#define INV_32768 ((float)3.051757813e-5)
#define INV_2_65536 ((float)32768.)
#define INV_4_65536 ((int)16384)
#define iZERO				((int)0)
#define fZERO				((float)0.)

// Macro Functions
#define BOUND_PI(x)		((x) - TWOPI*floor(((x) + PI) * INV_2PI))
#define		NOP()	asm("  RPT #3 || NOP");asm("  RPT #3 || NOP");asm("  RPT #3 || NOP");asm("  RPT #3 || NOP");asm("  RPT #3 || NOP");asm("  RPT #3 || NOP")
#define		NOP4()	NOP(); NOP(); NOP(); NOP()
//#define BOUND_PI_CLA(x)		((x>0)?((x)-2.*PI*(int)((x+PI)/(2.*PI))):((x)-2.*PI*(int)((x-PI)/(2.*PI))))
#define BOUND_PI_CLA(x)		((x>0)?((x)-2.*PI*(int)((x+PI)*INV_2PI)):((x)-2.*PI*(int)((x-PI)*INV_2PI)))
//#define BOUND_PI(x)		((x>0)?((x)-2.*PI*(int)((x+PI)*INV_2PI)):((x)-2.*PI*(int)((x-PI)*INV_2PI)))
//	#define BOUND_PI(x)		((x)>0?((x)-TWOPI*(long)(((x)+PI)*INV_2PI)):((x)-TWOPI*(long)(((x)-PI)*INV_2PI)))
//#define SIN(x,x2)		((x)*((float)1.-(x2)*(f3-(x2)*(f5-(x2)*(f7-(x2)*(f9-(x2)*(f11-(x2)*(f13-(x2)*f15))))))))
//#define COS(x2)			((float)1.-(x2)*(f2-(x2)*(f4-(x2)*(f6-(x2)*(f8-(x2)*(f10-(x2)*(f12-(x2)*f14)))))))
//#define SIN_INV_X(x2)   (((float)1.-(x2)*(f3-(x2)*(f5-(x2)*(f7-(x2)*(f9-(x2)*(f11-(x2)*(f13-(x2)*f15))))))))
//#define EXP(x)			((float)1.+(x)*((float)1.+(x)*(f2+(x)*(f3+(x)*(f4+(x)*(f5+(x)*(f6+(x)*f7)))))))

// 2-D Arctangent Approximation
//#define	ATAN_TABLE_SIZE	2048
//#define	ATAN_TABLE(f)	(*(float *)(p_atanTable + (int)(f*(float)atanSize + 0.5)))
//#define	atan1Table(f)	ATAN_TABLE((f))

//int atanTableInit(void);
//float atan2Table(const float y, const float x);

// for SCIC
#define WAIT_D			0
#define WAIT_R			1
#define WAIT_V			2
#define WAIT_LENGTH		3
#define WAIT_DATA		4
#define WAIT_STOP		5

#endif
