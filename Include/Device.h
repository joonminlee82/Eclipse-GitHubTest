/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: FREEWAY 
	Compiler    		: TMS320F28377D C-Compiler v6.1.2
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.02.06
	History				: Linear Motor Selection Menu is added into the HHT Menu (20180206 by GWON)
						: Motor Side R Controller is added in order to suppress 3th, 6th 9th harmonics, but this controllers are disable. (20180206 by GWON)
						: Mark to need to modify for FREEWAY --> Search command "// Need to modify for FREEWAY" (20180209 by GWON)
******************************************************************************/

#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "gnl.h"

/******************************************************************************/
/*  LCD COMMAND 정의                                                          */
/******************************************************************************/
#define CLEAR_SCREEN        0x01
#define CURSOR_HOME         0x02
#define CURSOR_LEFT         0x10
#define CURSOR_RIGHT        0x14
#define CURSOR1_ON          0x0e
#define CURSOR2_ON          0x0d
#define CURSOR_OFF          0x0c
#define SCROLL_LEFT         0x18
#define SCROLL_RIGHT        0x1c

/******************************************************************************/
/*  Key값 정의                                                                */
/******************************************************************************/
#define UP_KEY              'A'
#define DOWN_KEY            'B'
#define ESC_KEY             'C'
#define ENTER_KEY           'D'
#define UP_DOWN_KEY         'E'
#define LOCAL_KEY           'F'
#define RESET_KEY           'G'
#define DIR_KEY             'H'
#define JOG_KEY             'I'
#define STOP_KEY            'J'
#define RUN_KEY             'K'
#define UP_OFF_KEY          'L'
#define DOWN_OFF_KEY        'M'
#define JOG_OFF_KEY         'N'
#define ESC_ENTER_KEY				'O' 
#define DEF_R_ARROW         0x7E    /* "->" */

/******************************************************************************/
/*      Key Buffer 및 Pointer 변수 정의                                       */
/******************************************************************************/
#define KEYLEN              32              /* Key Buffer Size */
#define LCDLEN							20				/* LCD Length 16 */
#define LCD_DISPLAY					TRUE

/******************************************************************************/
/*      Function  Prototype                              		              */
/******************************************************************************/
TEXT GetCh(void);
TEXT GetKey(void);
void WriteChar(TEXT Ch);
void WriteString(STRING Str);
void WriteStringXY(TEXT X, TEXT Y, STRING Str);
void GotoXY(TEXT X, TEXT Y);
void KEY_Scan(void);
void DeviceDelayNOP(int32 cnt);
int  IndexLimit(int index, int MinIndex, int MaxIndex);
void DisplayIndex(int index);
void LCDClearScreen(void);

#endif	/* __Device_H_ */

