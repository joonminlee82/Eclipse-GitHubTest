/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: WBST MCU UPDATE HYUNDAI Elevator
	Compiler    	: TMS28335 C-Compiler v5.3
	Author				: Inverter part, E/C Department
	Version				: v1.0
	Last Rev.			: 2015.6.3
******************************************************************************/

#ifndef _MPLD_H__
#define _MPLD_H__


#define CARR_SHIFT		0
#define SIN_LUT_SIZE	8192.

#define XINTF_ZONE0		0x04000
#define XINTF_ZONE6		0x100000
#define XINTF_ZONE7		0x200000


#define	NVRAM_BASE		XINTF_ZONE7

#define	AD0_CS0			(*(volatile int *)(XINTF_ZONE0 + 0x0040))		//ADC 0x04040

//#define	AD0_CS0			(*(volatile int *)(XINTF_ZONE0 + 0x0010))
//#define	AD0_CS1			(*(volatile int *)(XINTF_ZONE0 + 0x0011))
//#define	AD0_CS2			(*(volatile int *)(XINTF_ZONE0 + 0x0012))
//#define	AD0_CS3			(*(volatile int *)(XINTF_ZONE0 + 0x0013))
//#define	AD0_CS4			(*(volatile int *)(XINTF_ZONE0 + 0x0014))
//#define	AD0_CS5			(*(volatile int *)(XINTF_ZONE0 + 0x0015))
//#define	AD0_CS6			(*(volatile int *)(XINTF_ZONE0 + 0x0016))
//#define	AD0_CS7			(*(volatile int *)(XINTF_ZONE0 + 0x0017))

//#define	AD_CVST			(*(volatile int *)(XINTF_ZONE0 + 0x0020))

//#define	PROT_REG		(*(volatile int *)(XINTF_ZONE0 + 0x0033))

//#define	WR_WDT			(*(volatile int *)(XINTF_ZONE0 + 0x0040))
#define	nPWMA				(*(volatile int *)(XINTF_ZONE0 + 0x0002))
#define	nPWMB				(*(volatile int *)(XINTF_ZONE0 + 0x0003))

#define	SW_RST			(*(volatile int *)(XINTF_ZONE0 + 0x0010))

#endif	// of _MPLD_H__

