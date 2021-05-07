/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler TI_v6.4.2
	Author				: Inverter part, E/C Department in Hyudai elevator
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/

#ifndef _GNL_
#define _GNL_

#include "F28x_Project.h"

/* type "names" of the fundamental types */
typedef long int Long;
typedef unsigned int  Uns;       
typedef unsigned long LgUns;

typedef Uns Bool;                                              /* boolean */

typedef unsigned char Byte;         /* smallest unit of addressable store */

/* Keypad, LCD Module에서 이용 for HHT 9242 */
typedef unsigned char TEXT;  
typedef unsigned char BOOL;
typedef unsigned char BYTE;       
typedef unsigned int  WORD;
typedef unsigned long DWORD;
typedef unsigned long LONG;
typedef char          *STRING;
//typedef enum { FALSE, TRUE } BOOL;

// SCI Protocol Test
typedef unsigned char 		u8;
typedef unsigned short 		u16;
typedef unsigned long 		u32;
typedef unsigned short 		DefErr;
//#ifndef NULL
#define NULL 0
#define ZERO 0
//#endif

#ifndef TRUE
#define FALSE ((Bool)0)
#define TRUE  ((Bool)1)
#endif

#ifndef CLOSED
#define OPEN    ((Bool)0)
#define CLOSED  ((Bool)1)
#endif

#ifndef ON
#define OFF ((Bool)0)
#define ON  ((Bool)1)
#endif

#define Void void

#endif /* _GNL_ */
