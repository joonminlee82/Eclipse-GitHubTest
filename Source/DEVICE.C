/*****************************************************************************
-------------------------------Organization------------------------------------	
	Project				: Hybrid Variable Speed System 
	Compiler    	: TMS320F28377D C-Compiler TI_v6.4.2
	Author				: Inverter part, E/C Department in Hyudai elevator
	Version				: v1.0
	Last Rev.			: 2015.8.28
******************************************************************************/

#include "Device.h"
#include "KeyMenu.h"
#include "SCI.h"
#include "Variable.h"
#include "gnl.h"
#include "comp.h"

/******************************************************************************/
/*  HardWare Key 값 정의                                                     */
/******************************************************************************/
#define HD_UP_KEY				2 /*0x00000010*/
#define HD_DOWN_KEY				4 /*0x00000100*/
#define HD_ESC_KEY				1 /*0x00000001*/
#define HD_ENTER_KEY				8 /*0x00001000*/

int	iPressKey;
int	Flag_KeyNo = 0;
extern int FaultFlag, SaveFault_Flg; 

/******************************************************************************/
/*      Key Buffer 및 Pointer 변수 정의                                       */
/******************************************************************************/
TEXT KeyBuf[KEYLEN];
TEXT KeyHead = 0;
TEXT KeyTail = 0;

void CheckRepatFlag(int KeyValue)
{	if( ((KeyHead+1) % KEYLEN) != KeyTail )
	{	KeyBuf[KeyHead] = KeyValue;
        KeyHead++;
        KeyHead %= KEYLEN;
    }
}

/******************************************************************************/
/*      KEY_Scan()                                                            */
/******************************************************************************/
/*      Parameters	: 없음                                                    */
/*      Returns		: 없음                                                    */
/*      Called		:                                                         */
/*      Related 	: GetKey(), InKey(), KeyEmpty(), PurgeKeyBuff()           */
/******************************************************************************/
void KEY_Scan(void) /* 5ms 루틴 속에 포함되어 있음 */
{
    static TEXT TmpKey     = 0;
    static TEXT TmpOldKey  = 0;
    static TEXT OldKey     = 0;
    static TEXT OnKey      = 0;
    static TEXT OffKey     = 0;
    static TEXT RepKey     = 0;
    static WORD RepCnt     = 0;
    static WORD TmpCnt     = 0;
    static WORD RepFlag    = FALSE;
    static WORD UpOn       = FALSE;
    static WORD DownOn     = FALSE;

		Key_In = (ScidRegs.SCIRXBUF.all&0xff);
		if((Key_In == HD_UP_KEY)||(Key_In == HD_DOWN_KEY)||(Key_In == HD_ESC_KEY)||(Key_In == HD_ENTER_KEY))
		{
			TmpKey = (Key_In) & 0x000f;
		}else TmpKey = 0;
	/* 현재 Key 값을 하위 4bit만masking하여 읽어옴. */
		iPressKey = TmpKey;
	if (TmpKey == TmpOldKey)
	{	TmpCnt++;
        if ( (OldKey != TmpKey) && (TmpCnt > 10) )
		{	TmpCnt = 0;
            OnKey  = (OldKey ^ TmpKey) & TmpKey;
            OffKey = (OldKey ^ TmpKey) & OldKey;
            OldKey = TmpKey;

            /* On Key Check */
            if (OnKey)
			{	if ( (OnKey & (HD_UP_KEY + HD_DOWN_KEY)) == (HD_UP_KEY + HD_DOWN_KEY) )
				{	CheckRepatFlag(UP_DOWN_KEY);
                    RepFlag = FALSE;
                }
                else if ( OnKey & (HD_UP_KEY + HD_DOWN_KEY) ) 
				{	if ( OnKey & (HD_UP_KEY) )
					{	if(((KeyHead+1)%KEYLEN) != KeyTail)
						{	KeyBuf[KeyHead] = UP_KEY;
                            KeyHead++;
                            KeyHead %= KEYLEN;
                            UpOn = TRUE;
                        }
	                    RepKey = UP_KEY;
	                }
	                else
					{   if ( ((KeyHead+1)%KEYLEN) != KeyTail )
						{	KeyBuf[KeyHead] = DOWN_KEY;
	                        KeyHead++;
    	                    KeyHead %= KEYLEN;
	                        DownOn = TRUE;
    	                }
	                    RepKey = DOWN_KEY;
    	            }
        	        RepCnt = 0;
	                RepFlag = TRUE;
    	        }
				if ( (OnKey & (HD_ESC_KEY + HD_ENTER_KEY)) == (HD_ESC_KEY + HD_ENTER_KEY) )
				{	CheckRepatFlag(ESC_ENTER_KEY);
                    RepFlag = FALSE;
                }
				else if ( OnKey & (HD_ESC_KEY + HD_ENTER_KEY) )
				{	if( OnKey & (HD_ESC_KEY) )/*0x0002)*/
					{	CheckRepatFlag(ESC_KEY);
		                RepFlag = FALSE;
    		        }
/*        		    if(OnKey & 0x0008)*/
					else
					{	CheckRepatFlag(ENTER_KEY);
                		RepFlag = FALSE;
					}
	            }
    	    } /* if(OnKey) */
        	if (OffKey)
			{	if (OffKey & HD_UP_KEY)
				{	if (UpOn)
					{	CheckRepatFlag(UP_OFF_KEY);
    	                UpOn = FALSE;
        		    }
	            }
    	        if (OffKey & HD_DOWN_KEY)
				{	if (DownOn)
					{	CheckRepatFlag(DOWN_OFF_KEY);
	                    DownOn = FALSE;
    	            }
        	    }
	            RepFlag = FALSE;
        	} /* if(OffKey) */
	    } /* if(OldKey != TmpKey) */
	} /* if(TmpKey==TmpOldKey) */
    else
		TmpCnt = 0;
    
    if(RepFlag)
	{	RepCnt++;
        if(RepCnt == 100)	
		{	RepCnt = 80;
         	CheckRepatFlag(RepKey);
        }
    } 
    
    TmpOldKey = TmpKey;
} 

/******************************************************************************/
/*      GetCh	: Key가 입력될 때까지 대기하다가 입력된 Key 값을 돌려준다.    */
/*					Fault를 Check 한다.										  */
/******************************************************************************/
/*      Parameters	: 없음                                                    */
/*      Returns 	: 입력된 Key 값을 return 한다.                            */
/*      Called		:                                                         */
/*      Related		: KeyEmpty(), GetKey(), PurgeKeyBuff()                    */
/*----------------------------------------------------------------------------*/
/*      Example		: switch(GetCh())         {                               */
/*                      case	UP_KEY:                                       */
/*                              break;                                        */
/*                      case	DOWN_KEY:                                     */
/*                              break;                                        */
/*                      case	ESC_KEY:                                      */
/*                              break;                                        */
/*                      case	ENTER_KEY:                                    */
/*                              break;                                        */
/*                      default:                                              */
/*                    }                                                       */
/******************************************************************************/
TEXT GetCh(void)
{   TEXT ch;

	while(KeyHead == KeyTail)
	{	
		KEY_FLT_GetStatus();
		if(SaveFault_Flg)	FaultError();	/*Fault발생시 fault data 저장 */
	}
    ch = KeyBuf[KeyTail++];
    KeyTail %= KEYLEN;
    
    return(ch);
} /* GetCh() */

/******************************************************************************/
/*      GetKey	: 입력된 Key 값을 돌려준다. Key 입력이 없을시 0을 리턴한다.   */
/*					Fault는 Check 하지 않는다.								  */
/******************************************************************************/
/*      Parameters	: 없음                                                    */
/*      Returns     : 입력된 Key가 있으면 Key 값을, 아니면 0을 return  한다.  */
/*      Called 		:		                                                  */
/*      Related		: KeyEmpty(), InKey(), PurgeKeyBuff()                     */
/*----------------------------------------------------------------------------*/
/*      Example 	: ch = GetKey();		                                  */
/*                    if(ch)	{                                             */
/*                      switch(ch)	{                                         */
/*                         case	UP_KEY:                                       */
/*                              break;                                        */
/*                         case	DOWN_KEY:                                     */
/*                              break;                                        */
/*                         case	ESC_KEY:                                      */
/*                              break;                                        */
/*                         case	ENTER_KEY:                                    */
/*                              break;                                        */
/*                         default:                                           */
/*                       }                                                    */
/*                     }                                                      */
/*                     else	{                                                 */
/*                                                                            */
/*                     }                                                      */
/******************************************************************************/
TEXT GetKey(void)
{   TEXT ch = 0;

    if(KeyHead != KeyTail)
	{	ch = KeyBuf[KeyTail++];
        KeyTail %= KEYLEN;
    } 
    return(ch);
} 

/******************************************************************************/
/*      WriteChar	: LCD 에 1 문자를 출력한다.                               */
/******************************************************************************/
/*      Parameters	: Ch (출력될 문자) 		                                  */
/*      Returns		: 없음                                                    */
/*      Called		:                          			                      */
/*      Related		: WriteString(), WriteStringXY()    		              */
/*----------------------------------------------------------------------------*/
/*      Example		: WriteChar('A');                                         */
/******************************************************************************/

void WriteChar(TEXT Ch)	//Edited-HHT
{  
	if((Ch >= 0x20)&&(Ch <= 0x7E))
	{
		Scid_Tx_Buf[Scid_Tx_End++] = Ch ;
		if(Scid_Tx_End>=100)	Scid_Tx_End = 0 ;
	
		if((ScidRegs.SCICTL2.bit.TXRDY)&&(HHT_MODE == 0))
		{	
			ScidRegs.SCICTL2.bit.TXINTENA=1;		//Tx Int
			ScidRegs.SCITXBUF.bit.TXDT = Scid_Tx_Buf[Scid_Tx_Cnt++]  ;
			if(Scid_Tx_Cnt>=100)	Scid_Tx_Cnt = 0 ;
		}
		else	ScidRegs.SCICTL2.bit.TXINTENA=1;		//Tx Int
	}
}
/******************************************************************************/
/*      WriteString	: LCD 에 문자열을 출력한다.                               */
/******************************************************************************/
/*      Parameters	: Str (출력될 문자열의 Pointer)                           */
/*      Returns		: 없음                                                    */
/*      Called		:                                                         */
/*      Related		: WriteChar(), WriteStringXY()                            */
/*----------------------------------------------------------------------------*/
/*      Example		: WriteString("HYUNDAI");                                 */
/******************************************************************************/

void WriteString(STRING Str)
{	
	while((*Str >= 0x20)&&(*Str <= 0x7E))
	{
		WriteChar(*Str++);
	}
	DeviceDelayNOP(60000) ;
}

/******************************************************************************/
/*      WriteStringXY()	: LCD 의 지정된 좌표에 문자열을 출력한다.             */
/******************************************************************************/
/*      Parameters		: X (LCD 의 X 촤표)                                   */
/*                        Y (LCD 의 Y 좌표)                                   */
/*                        Str (출력될 문자열의 Pointer)                       */
/*      Returns			: 없음                                                */
/*      Called			:                                                     */
/*      Related			: WriteChar(), WriteString()                          */
/*----------------------------------------------------------------------------*/
/*      Example    : WriteStringXY(0,0,"Hyundai");                            */
/******************************************************************************/

void WriteStringXY(TEXT X, TEXT Y, char* Str)
{   
	GotoXY(X,Y);
  while((*Str >= 0x20)&&(*Str <= 0x7E))
	{	
		WriteChar(*Str++);
	}
	DeviceDelayNOP(60000) ;
} /* WriteStringXY() */
/******************************************************************************/
/*      SetLCD		: LCD 를 설정한다.                                        */
/******************************************************************************/

void SetLCD(TEXT Cmd)	
{      
  if(Cmd != CURSOR2_ON && Cmd != CURSOR_OFF)
  {
		Scid_Tx_Buf[Scid_Tx_End++] = (Cmd & 0xff) | 0x80  ;
		if(Scid_Tx_End>=100)	Scid_Tx_End = 0 ;

		if((ScidRegs.SCICTL2.bit.TXRDY)&&(HHT_MODE == 0))
		{	
			ScidRegs.SCICTL2.bit.TXINTENA=1;		//Tx Int
			ScidRegs.SCITXBUF.bit.TXDT = Scid_Tx_Buf[Scid_Tx_Cnt++]  ;
			if(Scid_Tx_Cnt>=100)	Scid_Tx_Cnt = 0 ;
		}else	ScidRegs.SCICTL2.bit.TXINTENA=1;		//Tx Int
  }
}
/******************************************************************************/
/*      InitLCD		: LCD 를 초기화한다.                                      */
/******************************************************************************/
/*      Parameters	: 없음                                                    */
/*      Returns		: 없음                                                    */
/*      Called		: 		                                                  */
/*      Related		: WriteChar(), SetLCD()등 LCD 관련 함수                   */
/*----------------------------------------------------------------------------*/
/*      Example		: InitLCD();			                                  */
/******************************************************************************/
void LCD_Clear()
{
	WriteStringXY(0, 0, "                ");
	WriteStringXY(0, 1, "                ");
}
/******************************************************************************/
/*      GotoXY		: LCD 의 Cursor Position 을 설정한다.                     */
/******************************************************************************/
/*      Parameters	: X (LCD 의 X 좌표)                 	                  */
/*                    Y (LCD 의 Y 좌표)                                       */
/*      Returns		: 없음                          	                      */
/*      Called		:                       		                          */
/*      Related		: WriteStringXY()		                                  */
/*----------------------------------------------------------------------------*/
/*      Example		: GotoXY(0,2);                                            */
/*                    WriteChar('1');                                         */
/******************************************************************************/

void GotoXY(TEXT X, TEXT Y)
{	
	TEXT XY ;
	if((Y < 4)&&(X < 20))
	{
		if(Y==0)			XY = 0x14 +	X;		//if Line==0, Display 2
		else if(Y==1)	XY = 0x54 + X ;		//if Line==1, Display 3
		else if(Y==2)	XY = 0x14 +	X ;
		else if(Y==3) XY = 0x54 + X ;
		SetLCD(XY);
		DeviceDelayNOP(50000) ;
	}
} /* GotoXY() */

void DeviceDelayNOP(int32 cnt)
{
	if (cnt < 0)		cnt *= -1;
	if (cnt > 10000)	cnt = 10000;
	while(cnt--)
		asm("	NOP	");
}

int  IndexLimit(int index, int MinIndex, int MaxIndex)
{	if(index > MaxIndex)	index = MinIndex;
	if(index < MinIndex)	index = MaxIndex;
	return index;
}

void DisplayIndex(int index)
{	
	if(index<10)
	{
		WriteChar(index % 10 + 0x30);
		DeviceDelayNOP(1000) ;
		WriteChar('.');		
	}
	else if((index>=10)&&(index<100))
	{	
		WriteChar(index / 10 + 0x30);
		DeviceDelayNOP(1000) ;
		WriteChar(index % 10 + 0x30);
		DeviceDelayNOP(1000) ;
		WriteChar('.');	
	}
	else
	{
		WriteChar(index / 100 + 0x30);
		DeviceDelayNOP(1000) ;
		WriteChar((index / 10)%10 + 0x30);	
		DeviceDelayNOP(1000) ;
		WriteChar((index % 10)%10 + 0x30);		
		DeviceDelayNOP(1000) ;
		WriteChar('.');	
	}		
}
void LCDClearScreen (void)
{			/*   1234567890123456 */
	WriteStringXY(0, 0, "                    ");
	WriteStringXY(0, 1, "                    ");
}
