/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: WBST MCU UPDATE HYUNDAI Elevator
	Compiler    		: TMS28335 C-Compiler v5.3
	Author				: Inverter part, E/C Department
	Version				: v1.0
	Last Rev.			: 2015.6.3
******************************************************************************/

#ifndef _BDIB_
#define _BDIB_

#include "F28x_Project.h" 
#include "Variable.h"
#include "gnl.h"

/**********************************************************************/
/* Definition for BDIB ID(Identification)                             */
/**********************************************************************/
#define DIB_UP1						(GpioDataRegs.GPADAT.bit.GPIO6)		// nFWD(from MCU)
#define DIB_UP						!DIB_UP1							// UP
#define DIB_DOWN1					(GpioDataRegs.GPADAT.bit.GPIO7) 	// nREV(from MCU)
#define DIB_DOWN					!DIB_DOWN1							// DN
#define DIB_RDY1					(GpioDataRegs.GPADAT.bit.GPIO8)		// nREADY(from MCU)
#define DIB_RDY 					!DIB_RDY1							// RDY
#define DIB_RST1					(GpioDataRegs.GPADAT.bit.GPIO9)		// nCP_RST(from MCU)
#define DIB_RST						!DIB_RST1							// RST
#define DIB_AT1						(GpioDataRegs.GPADAT.bit.GPIO15)	// nAUTO(from MCU)
#define DIB_AT						!DIB_AT1							// AUTO
#define DIB_XATCTV					(GpioDataRegs.GPBDAT.bit.GPIO45)	// XATCTV (Vertical Car Top Auto)
#define DIB_XDLV					(GpioDataRegs.GPBDAT.bit.GPIO44)	// XDLV (Vertical Door Level D)
#define DIB_XSA						(GpioDataRegs.GPBDAT.bit.GPIO43)	// XSA (Safty Circuit Check)
#define DIB_XBKAS					(GpioDataRegs.GPBDAT.bit.GPIO42)	// XBKAS (Brake Open Check)
#define DIB_XBKBS					(GpioDataRegs.GPBDAT.bit.GPIO41)	// XBKBS (Brake Open Check)
#define DIB_XHCLI					(GpioDataRegs.GPBDAT.bit.GPIO33)	// XHCLI (Locking Device)
#define DIB_XCLI					(GpioDataRegs.GPBDAT.bit.GPIO32)	// XCLI (Locking Device)
#define nINV_ERR					(GpioDataRegs.GPBDAT.bit.GPIO55)	// INV_ERR(to MCU)
#define DIB_XULV					(GpioDataRegs.GPEDAT.bit.GPIO143)	// XULV	(Vertical Door Level U)
#define DIB_XDZV				  	(GpioDataRegs.GPEDAT.bit.GPIO144)	// XDZV (Vertical Door Zone)

#define BDIB_USEBIT_A   0x000083C0 // 00000000000000001000001111000000b
#define BDIB_USEBIT_B   0x00803E03 // 00000000100000000011111000000011b
#define BDIB_USEBIT_E   0x00018000 // 00000000000000011000000000000000b
#define BDIB_USEBIT_F   0x00000000 // 00000000000000000000000000000000b
/**********************************************************************/
/* Structure Definition for BDIB                                      */
/**********************************************************************/
typedef union
{
	Uns regStatus;
	struct
	{
		Uns UP			:1;			// nFWD
		Uns DOWN		:1;			// nREV
		Uns RDY			:1;			// nREADY
		Uns RST			:1;			// nCP_RST
		Uns AT			:1;			// AUTO
		Uns XATCTV		:1;			// Vertical Car Top Auto
		Uns XDLV		:1;			// Vertical Door Level D
		Uns XSA			:1;			// Safty Circuit Check
		Uns XBKAS		:1;			// Brake Open Check
		Uns XBKBS		:1;			// Brake Open Check
		Uns XHCLI		:1;			// Locking Device
		Uns XCLI		:1;			// Locking Device
		Uns XULV		:1;			// Vertical Door Level U
		Uns XDZV		:1;			// Verrical Door Zone
	} regStatusBit;
} BDIB;

typedef union
{
	Uns	regStatus;
	struct
	{
		Uns	flgSUS1		:1;	/*PLUL*/
		Uns	flgSUS2		:1;	/*PLUM*/
		Uns	flgSUS3 	:1; /*PLUH*/
		Uns	flgSUS4		:1; /*PLUH2*/
		Uns	flgSUS5		:1; /*SUS4*/
		Uns	flgSUS6		:1; /*SUS5*/
		Uns	flgSUS7		:1; /*SUS6*/

		Uns	flgSDS1		:1; /*PLDL*/
		Uns	flgSDS2 	:1; /*PLDM*/
		Uns	flgSDS3		:1; /*PLDH*/
		Uns	flgSDS4		:1; /*PLDH2*/
		Uns	flgSDS5		:1; /*SDS4*/
		Uns	flgSDS6		:1; /*SDS5*/
		Uns	flgSDS7		:1; /*SDS6*/
		Uns	__res		:2; /* reserved */
	} regStatusBit;
} Suds;

typedef union
{
	Uns	regStatus;
	struct
	{
		Uns	flgSUS1_old	:1;	/*SLOW UP1*/
		Uns	flgSUS2_old	:1;	/*SLOW UP2*/
		Uns	flgSUS3_old :1; /*SLOW UP3*/
		Uns	flgSUS4_old	:1; /*SLOW UP4*/
		Uns	flgSUS5_old	:1; /*SLOW UP5*/
		Uns	flgSUS6_old	:1; /*SLOW UP6*/
		Uns	flgSUS7_old	:1; /*SLOW UP7*/

		Uns	flgSDS1_old	:1; /*SLOW DN1*/
		Uns	flgSDS2_old :1; /*SLOW DN2*/
		Uns	flgSDS3_old	:1; /*SLOW DN3*/
		Uns	flgSDS4_old	:1; /*SLOW DN4*/
		Uns	flgSDS5_old	:1; /*SLOW DN5*/
		Uns	flgSDS6_old	:1; /*SLOW DN6*/
		Uns	flgSDS7_old	:1; /*SLOW DN7*/
		Uns	__res		:2; /* reserved */
	} regStatusBit;
} Suds_old;

typedef struct _SUDF_SW
{
		float		fSUF_Pos;
		float		fSDF_Pos;
		int			mode;
		int			mode_old;
		int 		init_OK;
} Sudf_sw ;

/**********************************************************************/
/*  BDIB Class's Methods                                              */
/**********************************************************************/
void	BDIB_UpdateAllStatus(void);

void BDIB_Create( void );
Uns	BDIB_GetAllStatus(void);
#endif /* #ifndef _BDIB_ */
