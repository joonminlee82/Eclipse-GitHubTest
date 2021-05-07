/*****************************************************************************
-------------------------------Organization------------------------------------
    Project             : FREEWAY
    Compiler            : TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
    Author              : Inverter part, Advanced Controls Research Team in Hyudai elevator(with KERI)
    Version             : v1.0
    Last Rev.           : 2019.07.22
    History             : Linear Motor Selection Menu is added into the HHT Menu (20180206 by GWON)
                        : Motor Side R Controller is added in order to suppress 3th, 6th 9th harmonics, but this controllers are disable. (20180206 by GWON)
                        : Mark to need to modify for FREEWAY --> Search command "// Need to modify for FREEWAY" (20180209 by GWON)
                        : Modified something for Linear Motor. ex) pole_Pitch, polepair, ... etc., (20180417 by GWON)
                        : Modified Position and Mechinical/Electrical Angle Calculation Function (20180418 by GWON)
                        : Inserted Rotor Position Estimation Function (20180430 by GWON)
                        : Profile Generation Function (20180508 by GWON)
                        : Fault Definition & Setting (20180509 by GWON)
                        : Added Profile Error Compensatory (20180510 by GWON)
                        : Added Control Code using HHT (20180629 by GWON)
                        : Added Mechanical_Observer for FRLSM (20180718 by GWON)
                        : Load Mass Estimator using Te for FRLSM (20180718 by GWON)
                        : Modified MC Operation Condition. Over 540V --> Over UVDC_Set (20180718 by GWON)
                        : STOP Mode Speed Max Value 0.1 --> 0.05 (20180718 by GWON)
                        : TX_INDEX 20 --> 40 (20180806 by GWON)
                        : Added SCI Protocol Version Response Part (20181107 by GWON)
                        : Added SCI Protocol Information Response Part (20181107 by GWON)
                        : Rearrangement of F/W for FREEWAY (20181112 by GWON)
                        : Added SPI Init Setting for between ARM and DSP communication (20181127 by GWON)
                        : Added SPI Control Communication Part (20181130 by GWON)
                        : Ver2 Board GPIO Setting (20181207 by GWON)
                        : Verified SPI Protocol (20181224 by GWON & Shin)
                        : Removed RMB, RAMP, MSTEP, FC (20190108 by GWON)
                        : Heidenhain Encoder F/W Merge using SPIC & DMA (20190702 by GWON)
                        : DAC Selection Data Insert at Inverter Data on SCI (20190714 by GWON, 20190722 Modified by GWON)
                        : DAC Selection F/W is added on InitDa() (20190714 by GWON, 20190722 Modified by GWON)
******************************************************************************/

#include <math.h>
#include <string.h>
#include "F28x_Project.h"
#include "Variable.h"
#include "drive.h"
#include "Fault.h"
#include "bdib.h"
#include "CC.h"
#include "Device.h"	
#include "KeyMenu.h"
#include "FRAM.h"
//#include "fc.h"
#include "SC.h"
#include "SCI.h"
#include "vane.h"
#include "gnl.h"

#pragma CODE_SECTION(cpu1_Scic_Rx, "ramfuncs");
#pragma CODE_SECTION(cpu1_Scic_Tx, "ramfuncs");
//#pragma CODE_SECTION(cpu1_Scid_Rx, "ramfuncs");
//#pragma CODE_SECTION(cpu1_Scid_Tx, "ramfuncs");
//#pragma CODE_SECTION(Main_To_INV_Msg_Pkt, "ramfuncs");

/* DSP VERSION INFORMATION */
char *DspVerStr	= "FAHD20052002";
//extern FLOOR 		Floor ;
extern Flt 			    FLT;
//extern Rmb			RMB;
extern STATUS_ARM_OBJ   g_SpiArmStatus;
extern u8               g_ErrorCode;
extern ENC_DATA_OBJ    ENCA_Data;

HW_RX_OBJ 		Hw_RX_Data;
HW_TX_OBJ		Hw_TX_Data;
u8				u8Rx_Data[128];
AP_DATE_OBJ 	*g_pDate;
STATUS_DSP_OBJ 	g_DspStatus;
STATUS_ARM_OBJ  g_ArmStatus;

IdCoData		g_IdCoData;
Id40Data		g_Id40Data;
Id41Data		g_Id41Data;
Id42Data		g_Id42Data;
Id43Data		g_Id43Data;
Id44Data        g_Id44Data;
Id45Data        g_Id45Data;
Id46Data        g_Id46Data;
Id47Data        g_Id47Data;
Id48Data        g_Id48Data;

IdF0Data 		g_IdF0Data;
IdF1Data 		g_IdF1Data;
IdF2Data 		g_IdF2Data;
IdF3Data 		g_IdF3Data;
IdF5Data        g_IdF5Data[10];
IdFAData        g_IdFAData;

u8 g_Mode_Check = 0;

Byte Scic_Buf_From_Main[FROM_MAIN_PACKET_LENGTH];
Byte Scic_Buf_To_Main[TO_MAIN_PACKET_LENGTH];

Uint16 scic_rxd = 0, scic_Cnt = 0, scic_ErrCnt = 0, scic_FltCnt = 0, scic_Cnt_old = 0;
Uint16 scid_rxd = 0, scid_Cnt = 0, scid_ErrCnt = 0, scid_FltCnt = 0, Scid_rxd_FailCnt = 0, Scid_rxd_Cnt = 0, Scic_rxd_Cnt = 0, Scic_rxd_ISR_Cnt = 0 ;
Uint16 scic_rx_crc_value = 0, scic_rx_crc = 0, scic_tx_crc_value = 0, scic_tx_crc = 0;

Byte KeyCnt = 0, SCI_CRC_Err = 0;
Uint16 SCIC_PE_Cnt = 0, SCIC_FE_Cnt = 0, SCID_PE_Cnt = 0, SCID_FE_Cnt = 0, SCI_CRC_Err_Cnt = 0 ;
Byte	test1 = 1,test2 = 2,test3 = 3,test4 = 4,test5 = 5,test6 = 6,test7 = 7,test8 = 8,test9 = 9,test10 = 10,test11 = 11,test12 = 12;

Uint16 Scic_Err_CNT = 0;

Byte TestPacket = 0;
Byte CheckCommand0 = 0;
Byte CheckCommand1 = 0;
//Uint16 TestCNT[128] = {0,};
int State = 0;
//u8 u8RecievedData[64];

u8 testTT[6];
float TestTTT = 0.;

u32 TestTXCNT = 0;
int Test_Sci_Tx = 0;
u16 u16BufferCNT = 0;
int Test_Scib_Rx = 0;

int TPTPTP = 0;

u8 u8DacModeChage_d1 = 0;

void SCI_Cmd_JumpApp(void);
void SCI_Cmd_JumpBoot(void);

extern float ZspLevel;

/*******************************************************************/
/* SCIC Initial()                              										 */
/*******************************************************************/
void InitSciB(void)
{
    ScibRegs.SCIFFTX.all = 0x8000;          // FIFO reset
    ScibRegs.SCIFFCT.all = 0x4000;          // Clear ABD(Auto baud bit)
    ScibRegs.SCICCR.all = 0x0007;           // 1 stop bit,  No loopback
                                            // No parity,8 char bits,
                                            // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0002;          // enable TX, RX, internal SCICLK, RX ERR
                                            // Disable SLEEP, TXWAKE

    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;    // RX/BK INT ENA=1,
    ScibRegs.SCICTL2.bit.TXINTENA = 0;      // TX INT ENA=1,

    ScibRegs.SCIHBAUD.bit.BAUD = BRR_VAL_B >> 8;
    ScibRegs.SCILBAUD.bit.BAUD = BRR_VAL_B & 0xff;

//  ScibRegs.SCIFFTX.all=0xE080;
    ScibRegs.SCIFFTX.all=0xE040;            // Interrupt
//  ScibRegs.SCIFFTX.all=0x8000;            // Polling
  //ScibRegs.SCIFFRX.all=0xB0AF;
//  ScibRegs.SCIFFRX.all=0xB0A8;
    ScibRegs.SCIFFRX.all=0xB0A1;

    ScibRegs.SCIFFCT.all=0x0;

    ScibRegs.SCIFFTX.bit.TXFIFORESET=1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET=1;

    ScibRegs.SCICTL1.all = 0x0023;          // enable TX, RX, internal SCICLK, RX ERR
                                            // Disable SLEEP, TXWAKE
}

void InitSciC(void)
{
	ScicRegs.SCIFFTX.all = 0x8000;			// FIFO reset
 	ScicRegs.SCIFFCT.all = 0x4000;			// Clear ABD(Auto baud bit)
 	ScicRegs.SCICCR.all = 0x0007;  			// 1 stop bit,  No loopback 
                                   			// No parity,8 char bits,
                                   			// async mode, idle-line protocol
	ScicRegs.SCICTL1.all = 0x0002; 			// enable TX, RX, internal SCICLK, RX ERR
                                   			// Disable SLEEP, TXWAKE

	ScicRegs.SCICTL2.bit.RXBKINTENA = 1;	// RX/BK INT ENA=1,
	ScicRegs.SCICTL2.bit.TXINTENA = 0;		// TX INT ENA=1,

	ScicRegs.SCIHBAUD.bit.BAUD = BRR_VAL_C >> 8;
	ScicRegs.SCILBAUD.bit.BAUD = BRR_VAL_C & 0xff;
	
//	ScicRegs.SCIFFTX.all=0xE080;
	ScicRegs.SCIFFTX.all=0xE040;			// Interrupt
//	ScicRegs.SCIFFTX.all=0x8000;            // Polling
  //ScicRegs.SCIFFRX.all=0xB0AF;
//	ScicRegs.SCIFFRX.all=0xB0A8;
	ScicRegs.SCIFFRX.all=0xB0A1;

	ScicRegs.SCIFFCT.all=0x0;
	
	ScicRegs.SCIFFTX.bit.TXFIFORESET=1;
	ScicRegs.SCIFFRX.bit.RXFIFORESET=1;
  
	ScicRegs.SCICTL1.all = 0x0023; 			// enable TX, RX, internal SCICLK, RX ERR
                                   			// Disable SLEEP, TXWAKE
	Drive_Cnt = 0 ;
}
/*******************************************************************/
/* SCID Initial()                              										 */
/*******************************************************************/
void InitSciD(void)
{
	ScidRegs.SCIFFTX.all = 0x8000;		// FIFO reset
	ScidRegs.SCIFFCT.all = 0x4000;		// Clear ABD(Auto baud bit)

	ScidRegs.SCICCR.all = 0x0007;  		// 1 stop bit,  No loopback 
                                 		// No parity, 8 char bits,
                                 		// async mode, idle-line protocol
	ScidRegs.SCICTL1.all = 0x0003; 		// enable TX, RX, internal SCICLK, 
                                 		// enable RX ERR, SLEEP, TXWAKE

	ScidRegs.SCICTL2.bit.RXBKINTENA = 1;// RX/BK INT ENA=1
	ScidRegs.SCICTL2.bit.TXINTENA = 0;	// TX INT ENA=0

	ScidRegs.SCIHBAUD.bit.BAUD = (int)BRR_VAL_D >> 8;	  // High Value
	ScidRegs.SCILBAUD.bit.BAUD = (int)BRR_VAL_D & 0xFF;	// Low Value

	ScidRegs.SCICTL1.all = 0x0023; 		// Relinquish SCI from Reset  
}
int scic_rxdata(void)
{
	Byte rxPE_flg = 0, rxFE_flg = 0;
	int i;
	Uint16 rxd_data = 0;
	// Check received character
	rxd_data = ScicRegs.SCIRXBUF.all;
	rxPE_flg = ((rxd_data>>14)&0x01);
	rxFE_flg = ((rxd_data>>15)&0x01);
	if((rxPE_flg == 1)||(rxFE_flg == 1))
	{
		if(rxPE_flg == 1) SCIC_PE_Cnt++;
		if(rxFE_flg == 1) SCIC_FE_Cnt++; 
		scic_ErrCnt++ ;
		if(scic_ErrCnt > FROM_MAIN_PACKET_LENGTH)
		{
    	for(i=0; i<FROM_MAIN_PACKET_LENGTH; i++)	Scic_Buf_From_Main[i] = 0;
    }
    Scic_rxd_Cnt = 0;
    ScicRegs.SCICTL1.bit.SWRESET = 0x01;
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;
		asm(" NOP");
		asm(" NOP");
		ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
		return -1;
	}else
	{
		scic_ErrCnt = 0;
		Scic_rxd_Cnt++;
		rxd_data = (rxd_data&0xff);
	}
	return (rxd_data&0xff);
}
// Transmit a character from the SCI
void scic_xmit(int a)
{
//  if(ScicRegs.SCIFFTX.bit.TXFFST != 0x00){
      ScicRegs.SCITXBUF.bit.TXDT = a;
//  }//TX FIFO에 데이터가 Full일 이 아닐 경우 Transmit.

}
void scic_msg(char * msg)
{
  int i;
  i = 0;
  while(msg[i] != '\0')
  {
      scic_xmit(msg[i]);
      i++;
  }
}
int scid_rxdata(void)
{
	Byte rxPE_flg = 0, rxFE_flg = 0;
	Uint16 rxd_data = 0;
	// Check received character
	rxd_data = ScidRegs.SCIRXBUF.all;
	rxPE_flg = ((rxd_data>>14)&0x01);
	rxFE_flg = ((rxd_data>>15)&0x01);
	if((rxPE_flg == 1)||(rxFE_flg == 1))
	{
		if(rxPE_flg == 1) SCID_PE_Cnt++;
		if(rxFE_flg == 1) SCID_FE_Cnt++; 
		scid_ErrCnt++ ;
    Scid_rxd_Cnt = 0;
    ScidRegs.SCICTL1.bit.SWRESET = 0x01;
		return -1;
	}else
	{
		scid_ErrCnt = 0;
		Scid_rxd_Cnt++;
		rxd_data = (rxd_data&0xff);
	}
	return (rxd_data&0xff);
}
// Transmit a character from the SCI
void scid_xmit(int a)
{
  if(ScidRegs.SCIFFTX.bit.TXFFST != 0x10)//TX FIFO에 데이터가 Full일 이 아닐 경우 Transmit.
  {
  	ScidRegs.SCITXBUF.bit.TXDT = a;
  }
}
void scid_msg(char * msg)
{
  int i;
  i = 0;
  while(msg[i] != '\0')
  {
      scid_xmit(msg[i]);
      i++;
  }
}

__interrupt void cpu1_Scib_Rx(void)
{
    IER &= 0x0000;
    IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT6 | M_INT8 | M_INT9 | M_INT10 | M_INT12 ;            // Fault || CC || SC || Drive || SPI ||Vane = Enc_Z
    EINT;

    ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    // Acknowledge this interrupt to receive more interrupts from group 9

    Test_Scib_Rx++;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
    asm(" NOP");
}

__interrupt void cpu1_Scic_Rx(void)
{
	Byte i = 0;
	int rx_data = 0;
	u8 u8Data = 0;
	u16 TimeoutCNT = 0;

	HW_RX_OBJ *pCmd;
	pCmd = &Hw_RX_Data;
	Scic_rxd_ISR_Cnt++;

	IER &= 0x0000;
    IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT6 | M_INT8 | M_INT9 | M_INT10 | M_INT12 ;            // Fault || CC || SC || Drive || SPI ||Vane = Enc_Z
	EINT;

	for(i=0;i<ScicRegs.SCIFFRX.bit.RXFFIL;i++)		// FIFO Size is One.
	{
		rx_data = scic_rxdata();					// Data Receive

		u8ScicRx[u16BufferCNT] = rx_data;
		u16BufferCNT++;
		if(rx_data == ETX) u16BufferCNT = 0;

		/* Request Timeout Check */
		TimeoutCNT = (ReadCpuTimer0Counter() - pCmd->u32Timeout_Count);

		if(TimeoutCNT > 20000000)					// Timeout is 100msec
		{
			pCmd->u8State = CMD_RXD_STATE_WAIT_STX;	// Goto Wait STX immediately
		}

		if(rx_data >= 0)
		{
			u8Data = (rx_data&0xff);
			switch(pCmd->u8State){
			case CMD_RXD_STATE_WAIT_STX:
				if(u8Data == STX)
				{
					pCmd->u8BCC_Calc = 0;
					pCmd->u32Timeout_Count = ReadCpuTimer0Counter();
					pCmd->u8IsCommand = FALSE;
					pCmd->u16RxDataCount = 0;
					pCmd->u8State = CMD_RXD_STATE_WAIT_BCC;
				}
			break;

			case CMD_RXD_STATE_WAIT_BCC:
				pCmd->u8BCC = u8Data;
				pCmd->u8State = CMD_RXD_STATE_WAIT_LEN0;
			break;

			case CMD_RXD_STATE_WAIT_LEN0:
				if (u8Data < 0x30 | u8Data > 0x3F)
				{
					pCmd->u16ErrorCode = ERROR_COMM_LENGTH;
				}
				else
				{
					pCmd->u8Length[0] = u8Data;
					pCmd->u8BCC_Calc ^= u8Data;
					pCmd->u8State = CMD_RXD_STATE_WAIT_LEN1;
				}
			break;
			case CMD_RXD_STATE_WAIT_LEN1:
				if (u8Data < 0x30 | u8Data > 0x3F)
				{
					pCmd->u16ErrorCode = ERROR_COMM_LENGTH;
				}
				else
				{
					pCmd->u8Length[1] = u8Data;
					pCmd->u8BCC_Calc ^= u8Data;
					pCmd->u8State = CMD_RXD_STATE_WAIT_LEN2;
				}
				break;

			case CMD_RXD_STATE_WAIT_LEN2:
				if (u8Data < 0x30 | u8Data > 0x3F)
				{
					pCmd->u16ErrorCode = ERROR_COMM_LENGTH;
				}
				else
				{
					pCmd->u8Length[2] = u8Data;
					pCmd->u8BCC_Calc ^= u8Data;
					pCmd->u8State = CMD_RXD_STATE_WAIT_LEN3;
				}
				break;

			case CMD_RXD_STATE_WAIT_LEN3:
				if (u8Data < 0x30 | u8Data > 0x3F)
				{
					pCmd->u16ErrorCode = ERROR_COMM_LENGTH;
				}
				else
				{
					pCmd->u8Length[3] = u8Data;
					pCmd->u8BCC_Calc ^= u8Data;
					pCmd->u16CommandLength  = (pCmd->u8Length[0] & 0x0F)<<12;
					pCmd->u16CommandLength += (pCmd->u8Length[1] & 0x0F)<<8;
					pCmd->u16CommandLength += (pCmd->u8Length[2] & 0x0F)<<4;
					pCmd->u16CommandLength += (pCmd->u8Length[3] & 0x0F);

					pCmd->u16RxDataCount = 0;

					pCmd->u8State = CMD_RXD_STATE_WAIT_COMMAND1;
				}
				break;

			case CMD_RXD_STATE_WAIT_COMMAND1:
				if (u8Data >= 0x30 )
				{
					pCmd->u8Command[0] = u8Data;
					pCmd->u8BCC_Calc ^= u8Data;
					pCmd->u8State = CMD_RXD_STATE_WAIT_COMMAND2;
				}
				else
				{
					pCmd->u16ErrorCode = ERROR_COMM_COMMAND;
				}
				break;

			case CMD_RXD_STATE_WAIT_COMMAND2:
				if (u8Data >= 0x30 )
				{
					pCmd->u8Command[1] = u8Data;
					pCmd->u8BCC_Calc ^= u8Data;
					pCmd->u8State = CMD_RXD_STATE_RECEIVE_DATA;
				}
				else
				{
					pCmd->u16ErrorCode = ERROR_COMM_COMMAND;
				}
				break;

			// status , error, data length, id, .... 기타 불필요 필드 삭제 예정
			case CMD_RXD_STATE_RECEIVE_DATA:
//				if (u8Data < 0x30 | u8Data > 0x3F)
//				{
//					pCmd->u16ErrorCode = ERROR_COMM_DATA;
//				}
//				else
//				{
					pCmd->RxData.u8Data[pCmd->u16RxDataCount] = u8Data;
					pCmd->u8BCC_Calc ^= u8Data;
					pCmd->u16RxDataCount++;
//				}

				if ( (pCmd->u16CommandLength - 9) == (pCmd->u16RxDataCount))
				{
					pCmd->u8State = CMD_RXD_STATE_WAIT_ETX;
				}

				break;

			case CMD_RXD_STATE_WAIT_ETX:
				if ((u8Data == ETX) && (pCmd->u8BCC == pCmd->u8BCC_Calc))			// Must check BCC(BCC Error is occurred)
				{
					pCmd->u8IsCommand = TRUE;
					pCmd->u8State = CMD_RXD_STATE_WAIT_STX;
				}
				else
				{
					pCmd->u16ErrorCode = ERROR_COMM_ETX;
				}
				break;

			default:
				pCmd->u8State = CMD_RXD_STATE_WAIT_STX;
				break;
			}

		}
	}

	ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	// Acknowledge this interrupt to recieve more interrupts from group 8

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;	
	asm(" NOP");
}
__interrupt void cpu1_Scic_Tx(void)
{
	IER &= 0x0000;
    IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT6 | M_INT8 | M_INT9 | M_INT10 | M_INT12 ;            // Fault || CC || SC || Drive || SPI ||Vane = Enc_Z
	EINT;

	ScicRegs.SCITXBUF.bit.TXDT = Scic_Buf_To_Main[scic_tx_cnt];
	scic_tx_cnt++;
	if(scic_tx_cnt >= (u16TXsize + TX_PACKET_FORMAT_LENGTH))
	{
		scic_tx_cnt = 0 ;
	}

	TestTXCNT++;

	ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
	// Acknowledge this interrupt to recieve more interrupts from group 8
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
	ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
	asm(" NOP");
}	
__interrupt void cpu1_Scid_Rx(void)
{
	IER &= 0x0000;
    IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT6 | M_INT8 | M_INT10 | M_INT12 ;            // Fault || CC || SC || Drive || SPI ||Vane = Enc_Z
	EINT;	
	
	if(ScidRegs.SCIRXST.bit.RXERROR == 1)
	{
		ScidRegs.SCICTL1.all = 0x0003 ;// Reset 
		ScidRegs.SCICTL1.all = 0x0023 ;// enable TX, RX, internal SCICLK, 
		scid_ErrCnt++ ;
	}else
	{	
		if((ScidRegs.SCIRXST.bit.RXRDY == 1)&&(HHT_MODE == 0))
		{	
			Scid_rxd_Cnt++;
			Key_In = ScidRegs.SCIRXBUF.all&0xff;
		}else
		{
			Scid_rxd_FailCnt++;
			// Acknowledge this interrupt to recieve more interrupts from group 9
			PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
			asm(" NOP");
			return;
		}
	}
	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;	
	asm(" NOP");
}	
__interrupt void cpu1_Scid_Tx(void)
{
	IER &= 0x0000;
    IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT6 | M_INT8 | M_INT9 | M_INT10 | M_INT12 ;            // Fault || CC || SC || Drive || SPI ||Vane = Enc_Z
	EINT;	
	
	if((Scid_Tx_Cnt != Scid_Tx_End)&&(HHT_MODE == 0))
	{
		ScidRegs.SCITXBUF.bit.TXDT = Scid_Tx_Buf[Scid_Tx_Cnt++] ;
		if(Scid_Tx_Cnt >= 100) Scid_Tx_Cnt = 0;
	}else	ScidRegs.SCICTL2.bit.TXINTENA=0 ;		//TX Stop
		
	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;	
	asm(" NOP");
}

/****************************************/
/*		Make Sending Data Packet 		*/
/* Author: Gogume						*/
/* History: First Released 20181107		*/
/****************************************/
void INV_To_Main_Msg_Pkt(HW_TX_OBJ *pCmd, u32 size, DefErr Error)
{
	u32 u32Length;
	u8 j;
	u8 sci_tx_ptr = 0 ;
	u8 bcc = 0;

	Scic_Buf_To_Main[sci_tx_ptr++] = STX;
	Scic_Buf_To_Main[sci_tx_ptr++] = 0x00;	// BCC
	Scic_Buf_To_Main[sci_tx_ptr++] = 0x00;	// LEN0
	Scic_Buf_To_Main[sci_tx_ptr++] = 0x00;	// LEN1
	Scic_Buf_To_Main[sci_tx_ptr++] = 0x00;	// LEN2
	Scic_Buf_To_Main[sci_tx_ptr++] = 0x00;	// LEN3

	Scic_Buf_To_Main[sci_tx_ptr++] = pCmd->u8Command[0];
	Scic_Buf_To_Main[sci_tx_ptr++] = pCmd->u8Command[1];

	if(Error == 0x00) Scic_Buf_To_Main[sci_tx_ptr++] = 'O';
	else Scic_Buf_To_Main[sci_tx_ptr++] = 'F';

	Scic_Buf_To_Main[sci_tx_ptr++] = ((Error >> 12) & 0x0F) | 0x30;		// Error 대분류
	Scic_Buf_To_Main[sci_tx_ptr++] = ((Error >>  8) & 0x0F) | 0x30;		// Error 중분류
	Scic_Buf_To_Main[sci_tx_ptr++] = ((Error >>  4) & 0x0F) | 0x30;		// Error 소분류 0
 	Scic_Buf_To_Main[sci_tx_ptr++] = ((Error >>  0) & 0x0F) | 0x30;		// Error 소분류 1

 	Scic_Buf_To_Main[sci_tx_ptr++] = pCmd->TxData.u8DataID;

    for(j = 0; j < (size - 1); j++)
    {
        Scic_Buf_To_Main[sci_tx_ptr + j] = pCmd->TxData.u8Data[j];
    }

    Scic_Buf_To_Main[sci_tx_ptr + j] = ETX;
    u32Length = sci_tx_ptr + j + 1;

 	Scic_Buf_To_Main[2] = ((u32Length >> 12) & 0x0F) | 0x30;			// LEN0
 	Scic_Buf_To_Main[3] = ((u32Length >>  8) & 0x0F) | 0x30;			// LEN1
 	Scic_Buf_To_Main[4] = ((u32Length >>  4) & 0x0F) | 0x30;			// LEN2
 	Scic_Buf_To_Main[5] = ((u32Length >>  0) & 0x0F) | 0x30;			// LEN3

 	for(j = 2, bcc = 0x00; j < u32Length -1 ; j++)
 	{
 		bcc ^= Scic_Buf_To_Main[j];
 	}

 	Scic_Buf_To_Main[1] = bcc;
}

void SCI_Create(void)
	{
	Byte i ;

	HW_RX_OBJ *pCmd;
	pCmd = &Hw_RX_Data;

	pCmd->u8BCC = 0;
	pCmd->u8Length[0] = 0;
	pCmd->u8Length[1] = 0;
	pCmd->u8Length[2] = 0;
	pCmd->u8Length[3] = 0;

	pCmd->u8Command[0] = 0;
	pCmd->u8Command[1] = 0;
	pCmd->u8BCC_Calc = 0;
	pCmd->u32Timeout_Count = 0;
	pCmd->u8IsCommand = FALSE;
	pCmd->u16RxDataCount = 0;
	pCmd->u8State = CMD_RXD_STATE_WAIT_STX;

	for(i=0; i < MAX_DATA_LENGTH ; i++){pCmd->RxData.u8Data[i] = 0;}

	pCmd->u8BCC_Calc = 0;
}

// SCI Protocol Test 20181031
/****************************************/
/*		Command Parsing Function		*/
/* Author: Gogume						*/
/* History: First Released 20181102		*/
/****************************************/
void DSP_COMMAND_TASK(HW_RX_OBJ *pCmd, HW_TX_OBJ *pCmd2)
{

	pCmd2->u8Command[0] = pCmd->u8Command[0];
	pCmd2->u8Command[1] = pCmd->u8Command[1];

	switch (pCmd->u8Command[0])
	{
	    case CMD_EXECUTE_JUMP: // 0x4B
	        if (pCmd->u8Command[1] == '0') {SCI_Cmd_JumpApp(); }
	        if (pCmd->u8Command[1] == '1') {SCI_Cmd_JumpBoot(); }
	        break;

		case CMD_EXECUTE_APPLICATION: 	//'A'
			// app jump
		break;
		case CMD_EXECUTE_DOWNLOAD: 		// 'D'
			// boot jump
		break;
		case CMD_GET_INFO: 				// 'G'
			if (pCmd->u8Command[1] == '0')						// Status Information
			{
				Get_Status(pCmd);
				u16TXsize = __LEN_G0;
				Send_IdF0(pCmd2, u16TXsize);
			}
			if (pCmd->u8Command[1] == '1')						// Version Information
			{
				u16TXsize = __LEN_G1;
				Send_Version(pCmd2, u16TXsize);					// DSP Version Information
			}
			if (pCmd->u8Command[1] == '2') { }					// Verification Information
			if (pCmd->u8Command[1] == '3') { }					// Device Information
			if (pCmd->u8Command[1] == '4') { }					// Inverter Information
		break;
		case CMD_INSPECT_DEVICE: 		// 'I'
			if (pCmd->u8Command[1] == '0') 						// Initialization of Motor Data
			{
				Get_MotorData(pCmd);
				u16TXsize = __LEN_I0;
                Send_Id42(pCmd2, u16TXsize);
			}
			if (pCmd->u8Command[1] == '1') 						// Initialization of Inverter Data
			{
				Get_InverterData(pCmd);
                u16TXsize = __LEN_I1;
                Send_Id43(pCmd2, u16TXsize);
			}
			if (pCmd->u8Command[1] == '2') { }					// Adjustment of Sensors
			if (pCmd->u8Command[1] == '3') 						// DSP Reset
			{
				DspSwReset(pCmd);
			}
			if (pCmd->u8Command[1] == '4') 						// FRAM Initialization
			{
				FRAM_Erase();
				GetROMTableAll();
				FRAM_Init();
				FRAM_Load();
				CC_VariUpdate();
				SC_VariUpdate();
				PC_VariUpdate();
				u16TXsize = __LEN_I4;
				AckSend(pCmd2, u16TXsize);
			}
			if (pCmd->u8Command[1] == '5')                      // Response of Motor Data
            {
			    Load_Motor_Data();
                u16TXsize = __LEN_I5;
                Send_Id42(pCmd2, u16TXsize);                    // [F6] = [42]
            }
			if (pCmd->u8Command[1] == '6')                      // Response of Inverter Data
            {
			    Load_Inverter_Data();
                u16TXsize = __LEN_I6;
                Send_Id43(pCmd2, u16TXsize);                    // [F7] = [43]
            }
			if (pCmd->u8Command[1] == '7')                      // Mode Change
			{
			    Get_Mode_Data(pCmd);
			    u16TXsize = __LEN_I7;
			    Send_Id44(pCmd2, u16TXsize);

			    g_Mode_Check = g_Id44Data.u8Mode;               // __NORMAL_MODE: 0x30, __OUTPUT_DEBUG_MODE: 0x31
			}
			if (pCmd->u8Command[1] == '8')                      // Digital Output Check.....!
            {
			    Get_Digital_Output_Check(pCmd);
			    u16TXsize = __LEN_I8;
//			    Send_Id45(pCmd2, u16TXsize);                    // [None]

            }
			if (pCmd->u8Command[1] == '9')                      // DAC Selection
			{
			    Get_Dac_Selection(pCmd);
			    u16TXsize = __LEN_I9;
                InitDa(0);
                Send_Id46(pCmd2, u16TXsize);
			}
			if (pCmd->u8Command[1] == ':')                      // DAC Selection Mode 2
            {
                Get_Dac_Selection_Mode2(pCmd);
                u16TXsize = __LEN_IA;
                InitDa(1);
                Send_Id47(pCmd2, u16TXsize);
            }
			if (pCmd->u8Command[1] == ';')                      // Static Force Measurement Mode
            {
			    Get_Static_Force_Measurement_Mode(pCmd);
			    u16TXsize = __LEN_IB;
			    SendIdFA(pCmd2, u16TXsize);
            }
		break;
		case CMD_SET_INFO: 				// 'S'
			if (pCmd->u8Command[1] == '0') { }					// Read LOG
			if (pCmd->u8Command[1] == '1')                      // Read Error LOG
			{
			    ErrorStoreToSci(TRUE);
			    u16TXsize = __LEN_S1;
			    SendIdF5(pCmd2, u16TXsize);
			}
			if (pCmd->u8Command[1] == '2') { }					// Write LOG
            if (pCmd->u8Command[1] == '3') {                    // Clear LOG
                FRAM_ErrErase();
            }
		break;
		case CMD_VERIFICATION: 			// 'V'
			if (pCmd->u8Command[1] == '1') { }					// Verification Ready
			if (pCmd->u8Command[1] == '2') { }					// Verification Start
			if (pCmd->u8Command[1] == '3') { }					// Verification Restart
			if (pCmd->u8Command[1] == '4') { }					// Verification Stop
			if (pCmd->u8Command[1] == '5') { }					// Verification End
		break;

		case CMD_FLASH_DOWNLOAD: // 'F'

	    break;

		case CMD_TEST:					// 'T' For ARM Test
			if(pCmd->u8Command[1] == '1')
			{
				State = 11;
//				nBKO_ON ;
			}
			if(pCmd->u8Command[1] == '2')
			{
				State = 22;
//				nBKO_OFF ;
			}

		break;
		default:

			break;
	}

	pCmd2->u8Length[0] = ((u16TXsize >> 12) & 0x0F) | 0x30;
	pCmd2->u8Length[1] = ((u16TXsize >>  8) & 0x0F) | 0x30;
	pCmd2->u8Length[2] = ((u16TXsize >>  4) & 0x0F) | 0x30;
	pCmd2->u8Length[3] = ((u16TXsize >>  0) & 0x0F) | 0x30;

	pCmd->u8IsCommand = FALSE;
}


#pragma DATA_SECTION (jumpflag, "usersector")
u8 jumpflag;

void SCI_Cmd_JumpApp()
{
    int (*pmain)(void);
    pmain = (int (*)(void))(ADDR_APP_MAIN);
    pmain();
}

void SCI_Cmd_JumpBoot()
{

    FRAM_Write(0x99, 'B');
    jumpflag = 'B';

    int (*pmain)(void);
    pmain = (int (*)(void))(ADDR_APP_BOOT);
    pmain();

//    while(1)
//    {
//        FLT_Reset();
//        EALLOW;
//        WdRegs.WDKEY.bit.WDKEY = 0x0011;
//        WdRegs.WDKEY.bit.WDKEY = 0x00AA;
//
//        WdRegs.SCSR.bit.WDENINT = 0x0;
//        WdRegs.WDCR.all = 0x10;
//        EDIS;
//    }
}
/****************************************/
/*			Get Status Function			*/
/* Author: Gogume						*/
/* History: First Released 20181102		*/
/****************************************/
void Get_Status(HW_RX_OBJ *pCmd)
{
	u8 i;
	u8 u8RecievedData[__LEN_G0];

	for(i = 0; i < sizeof(Id41Data); i++)
	{
		u8RecievedData[i] = 0x30;
	}
	Cmd_Parsing(pCmd, u8RecievedData, __LEN_G0);

	Id41_Data_Parsing(u8RecievedData);
}

/****************************************/
/*		Get Motor Data Function			*/
/* Author: Gogume						*/
/* History: First Released 20181120		*/
/****************************************/
void Get_MotorData(HW_RX_OBJ *pCmd)
{
	u8 i;
	u8 u8RecievedData[__LEN_I0];

	for(i = 0; i < sizeof(Id42Data); i++)
	{
		u8RecievedData[i] = 0x30;
	}
	Cmd_Parsing(pCmd, u8RecievedData, __LEN_I0);

	Id42_Data_Parsing(u8RecievedData);
}

/****************************************/
/*		Get Inverter Data Function		*/
/* Author: Gogume						*/
/* History: First Released 20181120		*/
/****************************************/
void Get_InverterData(HW_RX_OBJ *pCmd)
{
	u8 i;
	u8 u8RecievedData[__LEN_I1];

	for(i = 0; i < sizeof(Id43Data); i++)
	{
		u8RecievedData[i] = 0x30;
	}
	Cmd_Parsing(pCmd, u8RecievedData, __LEN_I1);

	Id43_Data_Parsing(u8RecievedData);
}

/************************************************/
/*   Mode Data Function                         */
/* Author: Gogume                               */
/* History: First Released 20190620             */
/* subject: Mode Change                         */
/************************************************/
void Get_Mode_Data(HW_RX_OBJ *pCmd)
{
    u8 i;
    u8 u8RecievedData[__LEN_I7];

    for(i = 0; i < sizeof(Id44Data); i++)
    {
        u8RecievedData[i] = 0x30;
    }
    Cmd_Parsing(pCmd, u8RecievedData, __LEN_I7);

    Id44_Data_Parsing(u8RecievedData);
}


/************************************************/
/*   Digital Output Check Data Function         */
/* Author: Gogume                               */
/* History: First Released 20190620             */
/* subject: Digital Output Check                */
/************************************************/
void Get_Digital_Output_Check(HW_RX_OBJ *pCmd)
{
    u8 i;
    u8 u8RecievedData[__LEN_I8];

    for(i = 0; i < sizeof(Id45Data); i++)
    {
        u8RecievedData[i] = 0x30;
    }
    Cmd_Parsing(pCmd, u8RecievedData, __LEN_I8);

    Id45_Data_Parsing(u8RecievedData);
}

/************************************************/
/*              DAC Selection Function          */
/* Author: Gogume                               */
/* History: First Released 20190722             */
/* subject: DAC Data Selection                  */
/************************************************/
void Get_Dac_Selection(HW_RX_OBJ *pCmd)
{
    u8 i;
    u8 u8RecievedData[__LEN_I9];

    for(i = 0; i < sizeof(Id46Data); i++)
    {
        u8RecievedData[i] = 0x30;
    }
    Cmd_Parsing(pCmd, u8RecievedData, __LEN_I9);

    Id46_Data_Parsing(u8RecievedData);

    /* Applying to InterfaceCode */
    KEY_SetInterfaceCode(AO1_SELECT, g_Id46Data.NumVari[0]);
    KEY_SetInterfaceCode(AO2_SELECT, g_Id46Data.NumVari[1]);
    KEY_SetInterfaceCode(AO3_SELECT, g_Id46Data.NumVari[2]);
    KEY_SetInterfaceCode(AO4_SELECT, g_Id46Data.NumVari[3]);

    KEY_SetInterfaceCode(AO_1_CENTER, g_Id46Data.Mid[0]);
    KEY_SetInterfaceCode(AO_2_CENTER, g_Id46Data.Mid[1]);
    KEY_SetInterfaceCode(AO_3_CENTER, g_Id46Data.Mid[2]);
    KEY_SetInterfaceCode(AO_4_CENTER, g_Id46Data.Mid[3]);

    KEY_SetInterfaceCode(AO_1_RANGE, g_Id46Data.Scale[0]);
    KEY_SetInterfaceCode(AO_2_RANGE, g_Id46Data.Scale[1]);
    KEY_SetInterfaceCode(AO_3_RANGE, g_Id46Data.Scale[2]);
    KEY_SetInterfaceCode(AO_4_RANGE, g_Id46Data.Scale[3]);
    /* FRAM Write */
    FRAM_Write(KEY_INTERFACE_START+AO1_SELECT, g_Id46Data.NumVari[0]);
    FRAM_Write(KEY_INTERFACE_START+AO2_SELECT, g_Id46Data.NumVari[1]);
    FRAM_Write(KEY_INTERFACE_START+AO3_SELECT, g_Id46Data.NumVari[2]);
    FRAM_Write(KEY_INTERFACE_START+AO4_SELECT, g_Id46Data.NumVari[3]);

    FRAM_Write(KEY_INTERFACE_START+AO_1_CENTER, g_Id46Data.Mid[0]);
    FRAM_Write(KEY_INTERFACE_START+AO_1_CENTER, g_Id46Data.Mid[1]);
    FRAM_Write(KEY_INTERFACE_START+AO_1_CENTER, g_Id46Data.Mid[2]);
    FRAM_Write(KEY_INTERFACE_START+AO_1_CENTER, g_Id46Data.Mid[3]);

    FRAM_Write(KEY_INTERFACE_START+AO_1_RANGE, g_Id46Data.Scale[0]);
    FRAM_Write(KEY_INTERFACE_START+AO_2_RANGE, g_Id46Data.Scale[1]);
    FRAM_Write(KEY_INTERFACE_START+AO_3_RANGE, g_Id46Data.Scale[2]);
    FRAM_Write(KEY_INTERFACE_START+AO_4_RANGE, g_Id46Data.Scale[3]);
}

/************************************************/
/*       DAC Selection Mode 2 Function          */
/* Author: Gogume                               */
/* History: First Released 20190724             */
/* subject: DAC Data Selection Mode 2           */
/************************************************/
void Get_Dac_Selection_Mode2(HW_RX_OBJ *pCmd)
{
    u8 i;
    u8 u8RecievedData[__LEN_IA];

    for(i = 0; i < sizeof(Id47Data); i++)
    {
        u8RecievedData[i] = 0x30;
    }
    Cmd_Parsing(pCmd, u8RecievedData, __LEN_IA);

    Id47_Data_Parsing(u8RecievedData);
}

/**********************************************************/
/*       Static Force Measurement  Mode Function          */
/* Author: Gogume                                         */
/* History: First Released 20191008                       */
/* subject: Static Force Measurement                      */
/**********************************************************/
void Get_Static_Force_Measurement_Mode(HW_RX_OBJ *pCmd)
{
    u8 i;
    u8 u8RecievedData[__LEN_IB];

    for(i = 0; i < sizeof(Id48Data); i++)
    {
        u8RecievedData[i] = 0x30;
    }
    Cmd_Parsing(pCmd, u8RecievedData, __LEN_IB);

    Id48_Data_Parsing(u8RecievedData);
}
/****************************************/
/*	Command Data Parsing Function		*/
/* Author: Gogume						*/
/* History: First Released 20181102		*/
/****************************************/
void Cmd_Parsing(HW_RX_OBJ *pCmd, u8 *pData, u8 size)
{
	u8 i;

	for(i = 0; i < size; i++)
	{
		pData[i] = pCmd->RxData.u8Data[i];
	}

	g_IdCoData.u8DataID 		= pData[0];
	g_IdCoData.u8DataLength[0] 	= pData[1];
	g_IdCoData.u8DataLength[1] 	= pData[2];
	g_IdCoData.u8DataLength[2] 	= pData[3];
	g_IdCoData.u8DataLength[3] 	= pData[4];
	g_IdCoData.rspDate.u8Year 	= pData[5];
	g_IdCoData.rspDate.u8Month 	= pData[6];
	g_IdCoData.rspDate.u8Date 	= pData[7];
	g_IdCoData.rspTime.u8Hour 	= pData[8];
	g_IdCoData.rspTime.u8Minute = pData[9];
	g_IdCoData.rspTime.u8Second = pData[10];
	g_IdCoData.u32TransNum 		 = (pData[11] & 0x0F) << 12;
	g_IdCoData.u32TransNum 		+= (pData[12] & 0x0F) << 8;
	g_IdCoData.u32TransNum 		+= (pData[13] & 0x0F) << 4;
	g_IdCoData.u32TransNum 		+= (pData[14] & 0x0F) << 0;
	g_IdCoData.u8CarSpeed 		= pData[15];
	g_IdCoData.u8CarType 		= pData[16];
	g_IdCoData.u8CarNumber 		= pData[17];
	g_IdCoData.u8CarPosition 	= pData[18];
	g_IdCoData.u8LimitSensorMode= pData[19];
	g_IdCoData.u8SPIMode 		= pData[20];
	g_IdCoData.u8APSMode 		= pData[21];
	g_IdCoData.u8LinearScaleMode= pData[22];
	g_IdCoData.u8Reserved 		= pData[23];

}

/****************************************/
/*		[41] Data Parsing Function		*/
/* Author: Gogume						*/
/* History: First Released 20181102		*/
/****************************************/
void Id41_Data_Parsing(u8 *pData)
{
	u8 i;
	i = 0;

	g_DspStatus.u8RunMode 			= pData[24+i++];
	g_DspStatus.u8RunStatus			= pData[24+i++];
	g_DspStatus.u8LoadStatus		= pData[24+i++];
	g_DspStatus.S_POS.x				= pData[24+i++];
	g_DspStatus.S_POS.y				= pData[24+i++];
	g_DspStatus.S_POS.z				= pData[24+i++];
	g_DspStatus.N_POS.x				= pData[24+i++];
	g_DspStatus.N_POS.y				= pData[24+i++];
	g_DspStatus.N_POS.z				= pData[24+i++];
	g_DspStatus.E_POS.x				= pData[24+i++];
	g_DspStatus.E_POS.y				= pData[24+i++];
	g_DspStatus.E_POS.z				= pData[24+i++];
	g_DspStatus.u8SetPosition[0]	= pData[24+i++];
	g_DspStatus.u8SetPosition[1]	= pData[24+i++];
	g_DspStatus.u8SetPosition[2]	= pData[24+i++];
	g_DspStatus.u8SetPosition[3]	= pData[24+i++];
	g_DspStatus.u8RemainPosition[0]	= pData[24+i++];
	g_DspStatus.u8RemainPosition[1]	= pData[24+i++];
	g_DspStatus.u8RemainPosition[2]	= pData[24+i++];
	g_DspStatus.u8RemainPosition[3]	= pData[24+i++];
	g_DspStatus.u8DoorStatus		= pData[24+i++];
	g_DspStatus.u8CommStatus		= pData[24+i++];
	g_DspStatus.u8StopAbleFloor		= pData[24+i++];
	g_DspStatus.u8LoadValue 		= pData[24+i++];
	g_DspStatus.u8EncoderInfo		= pData[24+i++];
	g_DspStatus.u8APSRegion 		= pData[24+i++];
	g_DspStatus.u8APS1PositionSign	= pData[24+i++];
	g_DspStatus.APS1Position[0]		= pData[24+i++];
	g_DspStatus.APS1Position[1]		= pData[24+i++];
	g_DspStatus.APS1Position[2]		= pData[24+i++];
	g_DspStatus.APS1Position[3]		= pData[24+i++];
	g_DspStatus.u8APS2PositionSign	= pData[24+i++];
	g_DspStatus.APS2Position[0]		= pData[24+i++];
	g_DspStatus.APS2Position[1]		= pData[24+i++];
	g_DspStatus.APS2Position[2]		= pData[24+i++];
	g_DspStatus.APS2Position[3]		= pData[24+i++];
	g_DspStatus.APS1Velocity[0] 	= pData[24+i++];
	g_DspStatus.APS1Velocity[1]	 	= pData[24+i++];
	g_DspStatus.APS2Velocity[0] 	= pData[24+i++];
	g_DspStatus.APS2Velocity[1] 	= pData[24+i++];
	g_DspStatus.LaserSensor1[0] 	= pData[24+i++];
	g_DspStatus.LaserSensor1[1] 	= pData[24+i++];
	g_DspStatus.LaserSensor2[0] 	= pData[24+i++];
	g_DspStatus.LaserSensor2[1]		= pData[24+i++];
	g_DspStatus.u8BrakeStatus		= pData[24+i++];
	g_DspStatus.InitOpStatus	    = pData[24+i++];
}

/****************************************/
/*		[42] Data Parsing Function		*/
/* Author: Gogume						*/
/* History: First Released 20181120		*/
/* Motor Data Parsing Part				*/
/****************************************/
void Id42_Data_Parsing(u8 *pData)
{
	u8 i = 0;
	u8 j;
	float temp = 0.;
	u8 u8firId42DataCall = 0;

	g_Id42Data.u8RxData.u8MotorType = pData[24+i++];					// Motor Type
	KEY_SetLinMotorCode(LIN_MOTOR_SELECT, g_Id42Data.u8RxData.u8MotorType - 0x30);
	FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_SELECT, g_Id42Data.u8RxData.u8MotorType - 0x30);
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8MotorCapacitor[j] = pData[24+i++];		// Motor Capacitor: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8MotorCapacitor[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_CAPACIT, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_CAPACIT, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8RatedVoltage[j] = pData[24+i++];			// Rated Voltage: Integer(4), Decimal(2)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8RatedVoltage[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_RATING_V, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RATING_V, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8RatedCurrent[j] = pData[24+i++];			// Rated Current: Integer(4), Decimal(2)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8RatedCurrent[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_RATING_A, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RATING_A, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8PolePitch[j] = pData[24+i++];				// Pole Pitch: Integer(4), Decimal(2)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8PolePitch[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_POLE_PITCH, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_POLE_PITCH, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8BaseSpeed[j] = pData[24+i++];				// Base Speed: Integer(4), Decimal(2)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8BaseSpeed[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_BASE_SPEED, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_BASE_SPEED, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8IqRate[j] = pData[24+i++];				// Q-axis Rated Current: Integer(4), Decimal(2)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8IqRate[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_IQSE_RATE, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_IQSE_RATE, temp);
	}

	g_Id42Data.u8RxData.u8KnowRpos = pData[24+i++];						// Do you Know Rotor Position?
	KNOW_ANGLE = (g_Id42Data.u8RxData.u8KnowRpos - 0x30);
	KEY_SetLinMotorCode(LIN_MOTOR_KNOW_RPOS, KNOW_ANGLE);
	FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_KNOW_RPOS, KNOW_ANGLE);

	if(KNOW_ANGLE == 0) {
	    Encoder_HD.theta_init = 0.;
	    g_SpiArmStatus.u8LMotAngleFind = 0x00;
//	    INV1_PC.Pos_Offset = 0.;
	}
	else g_SpiArmStatus.u8LMotAngleFind = 0x01;

	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8RposOffset[j] = pData[24+i++];			// Rotor Position Offset: Integer(4), Decimal(2)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8RposOffset[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ENC_OFFSET, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET, temp);
	}

	g_Id42Data.u8RxData.u8RposMethod = pData[24+i++];					// Rotor Position Estimation Method
	KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ESTIMAION, g_Id42Data.u8RxData.u8RposMethod - 0x30);
	FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ESTIMAION, g_Id42Data.u8RxData.u8RposMethod - 0x30);
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8RposEstimationTime[j] = pData[24+i++];	// DC Injection time: Integer(4), Decimal(2)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8RposEstimationTime[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_RPOS_ESTI_TIME, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ESTI_TIME, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8Ls[j] = pData[24+i++];					// Motor Inductance: Integer(4), Decimal(2)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8Ls[j-5], 4, 2);
		KEY_SetLinMotorCode(LIN_MOTOR_LS, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_LS, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8Rs[j] = pData[24+i++];					// Motor Resistance: Integer(2), Decimal(4)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8Rs[j-5], 2, 4);
		KEY_SetLinMotorCode(LIN_MOTOR_RS, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_RS, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id42Data.u8RxData.u8Kf[j] = pData[24+i++];					// Thrust Force Constant: Integer(2), Decimal(4)
		if(j == 5)temp = TypeCast_u8tofloat(&g_Id42Data.u8RxData.u8Kf[j-5], 2, 4);
		KEY_SetLinMotorCode(LIN_MOTOR_KF, temp);
		FRAM_Write(KEY_LINMOTOR_START+LIN_MOTOR_KF, temp);
	}
	for(j = 0; j < 12; j++)
	{
		g_Id42Data.u8RxData.u8Reserved[j] = pData[24+i++];				// Reserved Data
	}

    KEY_SetControlCode(CAR_TYPE, g_IdCoData.u8CarType);
    FRAM_Write(KEY_CONTROL_START+CAR_TYPE, g_IdCoData.u8CarType);

	if(u8firId42DataCall == 0)
	{
//		CC_VariUpdate();
//		SC_VariUpdate();
//		PC_VariUpdate();
//		Init_PROFILE();

		u8firId42DataCall = 1;
	}
}

/****************************************/
/*		[43] Data Parsing Function		*/
/* Author: Gogume						*/
/* History: First Released 20181120		*/
/* Inverter Data Parsing Part			*/
/****************************************/
void Id43_Data_Parsing(u8 *pData)
{
	u8 i = 0;
	u8 j;
	float temp = 0.;
	u8 u8firId43DataCall = 0;

	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8MaxSpeed[j] = pData[24+i++];				// Max Speed: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8MaxSpeed[j-5], 4, 2);
		KEY_SetControlCode(MAX_SPEED, temp);
		FRAM_Write(KEY_CONTROL_START+MAX_SPEED, temp);
		KEY_SetFactoryCode(OS_SET, temp*1.5);
		FRAM_Write(KEY_FACTORY_START+OS_SET, temp*1.5);
	}

	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8MaxAcc[j] = pData[24+i++];				// Max Accelation: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8MaxAcc[j-5], 4, 2);
		KEY_SetControlCode(MAX_ACC, temp);						// Need to change enum name from SHORT_RUNIRPM to MAX_ACC
		FRAM_Write(KEY_CONTROL_START+MAX_ACC, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8MaxJerk[j] = pData[24+i++];				// Max Jerk: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8MaxJerk[j-5], 4, 2);
		KEY_SetControlCode(MAX_JERK, temp);							// Need to change enum name from INSPECT_RPM to MAX_JERK
		FRAM_Write(KEY_CONTROL_START+MAX_JERK, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8Jm[j] = pData[24+i++];					// Inertia: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8Jm[j-5], 4, 2);
		KEY_SetControlCode(SC_JM, temp);
		FRAM_Write(KEY_CONTROL_START+SC_JM, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8SCFFGain[j] = pData[24+i++];				// Speed Controller FeedForward Gain: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8SCFFGain[j-5], 4, 2);
		KEY_SetControlCode(SC_FF_GAIN, temp);
		FRAM_Write(KEY_CONTROL_START+SC_FF_GAIN, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8TorqueLimit[j] = pData[24+i++];			// Torque Limit: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8TorqueLimit[j-5], 4, 2);
		KEY_SetFactoryCode(THRUST_FORCE_LIMIT, temp);
		FRAM_Write(KEY_FACTORY_START+THRUST_FORCE_LIMIT, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8ScaleVdc[j] = pData[24+i++];				// DC-Link Voltage Scale Factor: Integer(2), Decimal(4)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8ScaleVdc[j-5], 2, 4);
		KEY_SetFactoryCode(SCALE_VDC, temp);
		FRAM_Write(KEY_FACTORY_START+SCALE_VDC, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8ScaleCurrent[j] = pData[24+i++];			// Current Scale Factor: Integer(2), Decimal(4)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8ScaleCurrent[j-5], 2, 4);
		KEY_SetFactoryCode(SCALE_IS, temp);
		FRAM_Write(KEY_FACTORY_START+SCALE_IS, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8Wcc[j] = pData[24+i++];					// Current Controller Bandwidth: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8Wcc[j-5], 4, 2);
		KEY_SetFactoryCode(INV_WCC, temp);
		FRAM_Write(KEY_FACTORY_START+INV_WCC, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8Wsc[j] = pData[24+i++];					// Speed Controller Bandwidth: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8Wsc[j-5], 4, 2);
		KEY_SetFactoryCode(WSC, temp);
		FRAM_Write(KEY_FACTORY_START+WSC, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8Wpc[j] = pData[24+i++];					// Position Controller Bandwidth: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8Wpc[j-5], 4, 2);
		KEY_SetFactoryCode(WPC, temp);
		FRAM_Write(KEY_FACTORY_START+WPC, temp);
	}
	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8OverCurrent[j] = pData[24+i++];			// Over Current Setting: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8OverCurrent[j-5], 4, 2);
		KEY_SetFactoryCode(INV_OC_SET, temp);
		FRAM_Write(KEY_FACTORY_START+INV_OC_SET, temp);
	}

	g_Id43Data.u8RxData.u8FwdDirection = pData[24+i++];					// Forward Direction
	FWD_Dir = (g_Id43Data.u8RxData.u8FwdDirection - 0X30);
	KEY_SetControlCode(FWD_DIRECTION, FWD_Dir);
	FRAM_Write(KEY_CONTROL_START+FWD_DIRECTION, FWD_Dir);

	g_Id43Data.u8RxData.u8EncoderType = pData[24+i++];					// Encoder Type
	EncType = (g_Id43Data.u8RxData.u8EncoderType - 0x30);
	KEY_SetFactoryCode(ENCODERTYPE, EncType);
	FRAM_Write(KEY_FACTORY_START+ENCODERTYPE, EncType);

	for(j = 0; j < 3; j++)
	{
		g_Id43Data.u8RxData.u8MaxFloor[j] = pData[24+i++];				// Max Floor
		if(j == 2)
		{
			temp = (g_Id43Data.u8RxData.u8MaxFloor[0] - 0x30)*100;
			temp += (g_Id43Data.u8RxData.u8MaxFloor[1] - 0x30)*10;
			temp += (g_Id43Data.u8RxData.u8MaxFloor[2] - 0x30)*1;
//			Floor.uFloorTop = temp;
		}
		KEY_SetControlCode(MAX_FLOOR, temp);
		FRAM_Write(KEY_CONTROL_START+MAX_FLOOR, temp);
	}

	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8InpectSpeed[j] = pData[24+i++];			// Inspect Speed Setting: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8InpectSpeed[j-5], 4, 2);
		KEY_SetControlCode(INSPECT_SPEED, temp);
		FRAM_Write(KEY_CONTROL_START+INSPECT_SPEED, temp);
	}

	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8CreepSpeed[j] = pData[24+i++];			// Creep Speed Setting: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8CreepSpeed[j-5], 4, 2);
		KEY_SetControlCode(CREEP_SPEED, temp);
		FRAM_Write(KEY_CONTROL_START+CREEP_SPEED, temp);
	}

	for(j = 0; j < 6; j++)
	{
		g_Id43Data.u8RxData.u8RelevelSpeed[j] = pData[24+i++];			// Re-level Speed Setting: Integer(4), Decimal(2)
		if(j == 5) temp = TypeCast_u8tofloat(&g_Id43Data.u8RxData.u8RelevelSpeed[j-5], 4, 2);
		KEY_SetControlCode(RELEVEL_SPEED, temp);
		FRAM_Write(KEY_CONTROL_START+RELEVEL_SPEED, temp);
	}

	g_Id43Data.u8RxData.u8AntistallTime = pData[24+i++];				// Antistall Time
	g_Id43Data.u8RxData.u8AntistallTime = g_Id43Data.u8RxData.u8AntistallTime - 0x30;
	KEY_SetControlCode(ANTISTALL_TIME, g_Id43Data.u8RxData.u8AntistallTime);
	FRAM_Write(KEY_CONTROL_START+ANTISTALL_TIME, g_Id43Data.u8RxData.u8AntistallTime);

	for(j = 0; j <6; j++)                                               // Max Length: Integer(6) [mm]
	{
	    g_Id43Data.u8RxData.u8MaxLength[j] = pData[24+i++];
	    if(j == 5)
	    {
	        temp  =  (g_Id43Data.u8RxData.u8MaxLength[0] - 0x30) * 100000;
	        temp +=  (g_Id43Data.u8RxData.u8MaxLength[1] - 0x30) * 10000;
	        temp +=  (g_Id43Data.u8RxData.u8MaxLength[2] - 0x30) * 1000;
	        temp +=  (g_Id43Data.u8RxData.u8MaxLength[3] - 0x30) * 100;
	        temp +=  (g_Id43Data.u8RxData.u8MaxLength[4] - 0x30) * 10;
	        temp +=  (g_Id43Data.u8RxData.u8MaxLength[5] - 0x30) * 1;
	    }
	    KEY_SetControlCode(MAX_LENGTH, temp);
	    FRAM_Write(KEY_CONTROL_START+MAX_LENGTH, temp);
	}

	for(j = 0; j < 4; j++)
	{
	    temp = g_Id43Data.u8RxData.u8ZspLevel[j] = pData[24+i++];
	    if(j == 3)
	    {
	        temp  = (g_Id43Data.u8RxData.u8ZspLevel[0] - 0x30) * 1;
	        temp += (g_Id43Data.u8RxData.u8ZspLevel[1] - 0x30) * 0.1;
	        temp += (g_Id43Data.u8RxData.u8ZspLevel[2] - 0x30) * 0.01;
	        temp += (g_Id43Data.u8RxData.u8ZspLevel[3] - 0x30) * 0.001;
	    }
	    ZspLevel = temp;
	    KEY_SetInterfaceCode(ZSP_LEVEL, temp);
	    FRAM_Write(KEY_INTERFACE_START+ZSP_LEVEL, temp);
	}

	for(j = 0; j < 125; j++)
	{
		g_Id43Data.u8RxData.u8Reserved[j] = pData[24+i++];				// Reserved Data
	}

	KEY_SetControlCode(CAR_TYPE, g_IdCoData.u8CarType);
	FRAM_Write(KEY_CONTROL_START+CAR_TYPE, g_IdCoData.u8CarType);

	if(u8firId43DataCall == 0)
	{
//		CC_VariUpdate();
//		SC_VariUpdate();
//		PC_VariUpdate();
//		Init_PROFILE();

		u8firId43DataCall = 1;
	}
}

/******************************************/
/*      [44] Data Parsing Function        */
/* Author: Gogume                         */
/* History: First Released 20190620       */
/* Mode Data Parsing Part                 */
/******************************************/
void Id44_Data_Parsing(u8 *pData)
{
    u8 i = 0;
    g_Id44Data.u8Mode           = pData[24+i++];
}

/******************************************/
/*      [45] Data Parsing Function        */
/* Author: Gogume                         */
/* History: First Released 20190620       */
/* Digital Output Check Data Parsing Part */
/******************************************/
void Id45_Data_Parsing(u8 *pData)
{
    u8 i = 0;
    g_Id45Data.u8nDMC2      = pData[24+i++];
    g_Id45Data.u8nDBKA      = pData[24+i++];
    g_Id45Data.u8nDBKB      = pData[24+i++];
    g_Id45Data.u8nDBKP      = pData[24+i++];
    g_Id45Data.u8nDDS       = pData[24+i++];
    g_Id45Data.u8nDMS       = pData[24+i++];
    g_Id45Data.u8nIFAN      = pData[24+i++];
    g_Id45Data.u8nMC_A      = pData[24+i++];
    g_Id45Data.u8nDHCLD     = pData[24+i++];
    g_Id45Data.u8nDCLD      = pData[24+i++];
    g_Id45Data.u8nINV_SPO1  = pData[24+i++];
    g_Id45Data.u8nDHYDPRESS = pData[24+i++];
    g_Id45Data.u8nINV_SPO2  = pData[24+i++];
}

/******************************************/
/*      [46] Data Parsing Function        */
/* Author: Gogume                         */
/* History: First Released 20190723       */
/* Digital Output Check Data Parsing Part */
/******************************************/
void Id46_Data_Parsing(u8 *pData)
{
    u8 i = 0;
    g_Id46Data.u8Da0.u8DaVari[0]  = pData[24+i++];
    g_Id46Data.u8Da0.u8DaVari[1]  = pData[24+i++];
    g_Id46Data.u8Da0.u8DaType     = pData[24+i++];
    g_Id46Data.u8Da0.u8DaScale[0] = pData[24+i++];
    g_Id46Data.u8Da0.u8DaScale[1] = pData[24+i++];
    g_Id46Data.u8Da0.u8DaScale[2] = pData[24+i++];
    g_Id46Data.u8Da0.u8DaScale[3] = pData[24+i++];
    g_Id46Data.u8Da0.u8DaScale[4] = pData[24+i++];
    g_Id46Data.u8Da0.u8DaScale[5] = pData[24+i++];
    g_Id46Data.u8Da0.u8DaScale[6] = pData[24+i++];
    g_Id46Data.u8Da0.u8DaScale[7] = pData[24+i++];
    g_Id46Data.u8Da0.u8DaMid[0]   = pData[24+i++];
    g_Id46Data.u8Da0.u8DaMid[1]   = pData[24+i++];
    g_Id46Data.u8Da0.u8DaMid[2]   = pData[24+i++];
    g_Id46Data.u8Da0.u8DaMid[3]   = pData[24+i++];
    g_Id46Data.u8Da0.u8DaMid[4]   = pData[24+i++];
    g_Id46Data.u8Da0.u8DaMid[5]   = pData[24+i++];

    g_Id46Data.u8Da1.u8DaVari[0]  = pData[24+i++];
    g_Id46Data.u8Da1.u8DaVari[1]  = pData[24+i++];
    g_Id46Data.u8Da1.u8DaType     = pData[24+i++];
    g_Id46Data.u8Da1.u8DaScale[0] = pData[24+i++];
    g_Id46Data.u8Da1.u8DaScale[1] = pData[24+i++];
    g_Id46Data.u8Da1.u8DaScale[2] = pData[24+i++];
    g_Id46Data.u8Da1.u8DaScale[3] = pData[24+i++];
    g_Id46Data.u8Da1.u8DaScale[4] = pData[24+i++];
    g_Id46Data.u8Da1.u8DaScale[5] = pData[24+i++];
    g_Id46Data.u8Da1.u8DaScale[6] = pData[24+i++];
    g_Id46Data.u8Da1.u8DaScale[7] = pData[24+i++];
    g_Id46Data.u8Da1.u8DaMid[0]   = pData[24+i++];
    g_Id46Data.u8Da1.u8DaMid[1]   = pData[24+i++];
    g_Id46Data.u8Da1.u8DaMid[2]   = pData[24+i++];
    g_Id46Data.u8Da1.u8DaMid[3]   = pData[24+i++];
    g_Id46Data.u8Da1.u8DaMid[4]   = pData[24+i++];
    g_Id46Data.u8Da1.u8DaMid[5]   = pData[24+i++];

    g_Id46Data.u8Da2.u8DaVari[0]  = pData[24+i++];
    g_Id46Data.u8Da2.u8DaVari[1]  = pData[24+i++];
    g_Id46Data.u8Da2.u8DaType     = pData[24+i++];
    g_Id46Data.u8Da2.u8DaScale[0] = pData[24+i++];
    g_Id46Data.u8Da2.u8DaScale[1] = pData[24+i++];
    g_Id46Data.u8Da2.u8DaScale[2] = pData[24+i++];
    g_Id46Data.u8Da2.u8DaScale[3] = pData[24+i++];
    g_Id46Data.u8Da2.u8DaScale[4] = pData[24+i++];
    g_Id46Data.u8Da2.u8DaScale[5] = pData[24+i++];
    g_Id46Data.u8Da2.u8DaScale[6] = pData[24+i++];
    g_Id46Data.u8Da2.u8DaScale[7] = pData[24+i++];
    g_Id46Data.u8Da2.u8DaMid[0]   = pData[24+i++];
    g_Id46Data.u8Da2.u8DaMid[1]   = pData[24+i++];
    g_Id46Data.u8Da2.u8DaMid[2]   = pData[24+i++];
    g_Id46Data.u8Da2.u8DaMid[3]   = pData[24+i++];
    g_Id46Data.u8Da2.u8DaMid[4]   = pData[24+i++];
    g_Id46Data.u8Da2.u8DaMid[5]   = pData[24+i++];

    g_Id46Data.u8Da3.u8DaVari[0]  = pData[24+i++];
    g_Id46Data.u8Da3.u8DaVari[1]  = pData[24+i++];
    g_Id46Data.u8Da3.u8DaType     = pData[24+i++];
    g_Id46Data.u8Da3.u8DaScale[0] = pData[24+i++];
    g_Id46Data.u8Da3.u8DaScale[1] = pData[24+i++];
    g_Id46Data.u8Da3.u8DaScale[2] = pData[24+i++];
    g_Id46Data.u8Da3.u8DaScale[3] = pData[24+i++];
    g_Id46Data.u8Da3.u8DaScale[4] = pData[24+i++];
    g_Id46Data.u8Da3.u8DaScale[5] = pData[24+i++];
    g_Id46Data.u8Da3.u8DaScale[6] = pData[24+i++];
    g_Id46Data.u8Da3.u8DaScale[7] = pData[24+i++];
    g_Id46Data.u8Da3.u8DaMid[0]   = pData[24+i++];
    g_Id46Data.u8Da3.u8DaMid[1]   = pData[24+i++];
    g_Id46Data.u8Da3.u8DaMid[2]   = pData[24+i++];
    g_Id46Data.u8Da3.u8DaMid[3]   = pData[24+i++];
    g_Id46Data.u8Da3.u8DaMid[4]   = pData[24+i++];
    g_Id46Data.u8Da3.u8DaMid[5]   = pData[24+i++];

    /* Variable Selection */
    /* DA0 */
    g_Id46Data.NumVari[0]  = (g_Id46Data.u8Da0.u8DaVari[0] - 0x30) * 0x10;
    g_Id46Data.NumVari[0] += (g_Id46Data.u8Da0.u8DaVari[1] - 0x30) * 1;
    /* DA1 */
    g_Id46Data.NumVari[1]  = (g_Id46Data.u8Da1.u8DaVari[0] - 0x30) * 0x10;
    g_Id46Data.NumVari[1] += (g_Id46Data.u8Da1.u8DaVari[1] - 0x30) * 1;
    /* DA2 */
    g_Id46Data.NumVari[2]  = (g_Id46Data.u8Da2.u8DaVari[0] - 0x30) * 0x10;
    g_Id46Data.NumVari[2] += (g_Id46Data.u8Da2.u8DaVari[1] - 0x30) * 1;
    /* DA3 */
    g_Id46Data.NumVari[3]  = (g_Id46Data.u8Da3.u8DaVari[0] - 0x30) * 0x10;
    g_Id46Data.NumVari[3] += (g_Id46Data.u8Da3.u8DaVari[1] - 0x30) * 1;
    /* Variable Type */
    g_Id46Data.type[0] = g_Id46Data.u8Da0.u8DaType - 0x30;  /* DA0 */
    g_Id46Data.type[1] = g_Id46Data.u8Da1.u8DaType - 0x30;  /* DA1 */
    g_Id46Data.type[2] = g_Id46Data.u8Da2.u8DaType - 0x30;  /* DA2 */
    g_Id46Data.type[3] = g_Id46Data.u8Da3.u8DaType - 0x30;  /* DA3 */

    /* Scale Setting */
    /* DA0 */
    g_Id46Data.Scale[0]  = (g_Id46Data.u8Da0.u8DaScale[0] - 0x30) * 10000;
    g_Id46Data.Scale[0] += (g_Id46Data.u8Da0.u8DaScale[1] - 0x30) * 1000;
    g_Id46Data.Scale[0] += (g_Id46Data.u8Da0.u8DaScale[2] - 0x30) * 100;
    g_Id46Data.Scale[0] += (g_Id46Data.u8Da0.u8DaScale[3] - 0x30) * 10;
    g_Id46Data.Scale[0] += (g_Id46Data.u8Da0.u8DaScale[4] - 0x30) * 1;
    g_Id46Data.Scale[0] += (g_Id46Data.u8Da0.u8DaScale[5] - 0x30) * 0.1;
    g_Id46Data.Scale[0] += (g_Id46Data.u8Da0.u8DaScale[6] - 0x30) * 0.01;
    g_Id46Data.Scale[0] += (g_Id46Data.u8Da0.u8DaScale[7] - 0x30) * 0.001;
    /* DA1 */
    g_Id46Data.Scale[1]  = (g_Id46Data.u8Da1.u8DaScale[0] - 0x30) * 10000;
    g_Id46Data.Scale[1] += (g_Id46Data.u8Da1.u8DaScale[1] - 0x30) * 1000;
    g_Id46Data.Scale[1] += (g_Id46Data.u8Da1.u8DaScale[2] - 0x30) * 100;
    g_Id46Data.Scale[1] += (g_Id46Data.u8Da1.u8DaScale[3] - 0x30) * 10;
    g_Id46Data.Scale[1] += (g_Id46Data.u8Da1.u8DaScale[4] - 0x30) * 1;
    g_Id46Data.Scale[1] += (g_Id46Data.u8Da1.u8DaScale[5] - 0x30) * 0.1;
    g_Id46Data.Scale[1] += (g_Id46Data.u8Da1.u8DaScale[6] - 0x30) * 0.01;
    g_Id46Data.Scale[1] += (g_Id46Data.u8Da1.u8DaScale[7] - 0x30) * 0.001;
    /* DA2 */
    g_Id46Data.Scale[2]  = (g_Id46Data.u8Da2.u8DaScale[0] - 0x30) * 10000;
    g_Id46Data.Scale[2] += (g_Id46Data.u8Da2.u8DaScale[1] - 0x30) * 1000;
    g_Id46Data.Scale[2] += (g_Id46Data.u8Da2.u8DaScale[2] - 0x30) * 100;
    g_Id46Data.Scale[2] += (g_Id46Data.u8Da2.u8DaScale[3] - 0x30) * 10;
    g_Id46Data.Scale[2] += (g_Id46Data.u8Da2.u8DaScale[4] - 0x30) * 1;
    g_Id46Data.Scale[2] += (g_Id46Data.u8Da2.u8DaScale[5] - 0x30) * 0.1;
    g_Id46Data.Scale[2] += (g_Id46Data.u8Da2.u8DaScale[6] - 0x30) * 0.01;
    g_Id46Data.Scale[2] += (g_Id46Data.u8Da2.u8DaScale[7] - 0x30) * 0.001;
    /* DA3 */
    g_Id46Data.Scale[3]  = (g_Id46Data.u8Da3.u8DaScale[0] - 0x30) * 10000;
    g_Id46Data.Scale[3] += (g_Id46Data.u8Da3.u8DaScale[1] - 0x30) * 1000;
    g_Id46Data.Scale[3] += (g_Id46Data.u8Da3.u8DaScale[2] - 0x30) * 100;
    g_Id46Data.Scale[3] += (g_Id46Data.u8Da3.u8DaScale[3] - 0x30) * 10;
    g_Id46Data.Scale[3] += (g_Id46Data.u8Da3.u8DaScale[4] - 0x30) * 1;
    g_Id46Data.Scale[3] += (g_Id46Data.u8Da3.u8DaScale[5] - 0x30) * 0.1;
    g_Id46Data.Scale[3] += (g_Id46Data.u8Da3.u8DaScale[6] - 0x30) * 0.01;
    g_Id46Data.Scale[3] += (g_Id46Data.u8Da3.u8DaScale[7] - 0x30) * 0.001;

    /* Mid Point Setting */
    /* DA0 */
    g_Id46Data.Mid[0]  = (g_Id46Data.u8Da0.u8DaMid[0] - 0x30) * 100;
    g_Id46Data.Mid[0] += (g_Id46Data.u8Da0.u8DaMid[1] - 0x30) * 10;
    g_Id46Data.Mid[0] += (g_Id46Data.u8Da0.u8DaMid[2] - 0x30) * 1;
    g_Id46Data.Mid[0] += (g_Id46Data.u8Da0.u8DaMid[3] - 0x30) * 0.1;
    g_Id46Data.Mid[0] += (g_Id46Data.u8Da0.u8DaMid[4] - 0x30) * 0.01;
    /* DA1 */
    g_Id46Data.Mid[1]  = (g_Id46Data.u8Da1.u8DaMid[0] - 0x30) * 100;
    g_Id46Data.Mid[1] += (g_Id46Data.u8Da1.u8DaMid[1] - 0x30) * 10;
    g_Id46Data.Mid[1] += (g_Id46Data.u8Da1.u8DaMid[2] - 0x30) * 1;
    g_Id46Data.Mid[1] += (g_Id46Data.u8Da1.u8DaMid[3] - 0x30) * 0.1;
    g_Id46Data.Mid[1] += (g_Id46Data.u8Da1.u8DaMid[4] - 0x30) * 0.01;
    /* DA2 */
    g_Id46Data.Mid[2]  = (g_Id46Data.u8Da2.u8DaMid[0] - 0x30) * 100;
    g_Id46Data.Mid[2] += (g_Id46Data.u8Da2.u8DaMid[1] - 0x30) * 10;
    g_Id46Data.Mid[2] += (g_Id46Data.u8Da2.u8DaMid[2] - 0x30) * 1;
    g_Id46Data.Mid[2] += (g_Id46Data.u8Da2.u8DaMid[3] - 0x30) * 0.1;
    g_Id46Data.Mid[2] += (g_Id46Data.u8Da2.u8DaMid[4] - 0x30) * 0.01;
    /* DA3 */
    g_Id46Data.Mid[3]  = (g_Id46Data.u8Da3.u8DaMid[0] - 0x30) * 100;
    g_Id46Data.Mid[3] += (g_Id46Data.u8Da3.u8DaMid[1] - 0x30) * 10;
    g_Id46Data.Mid[3] += (g_Id46Data.u8Da3.u8DaMid[2] - 0x30) * 1;
    g_Id46Data.Mid[3] += (g_Id46Data.u8Da3.u8DaMid[3] - 0x30) * 0.1;
    g_Id46Data.Mid[3] += (g_Id46Data.u8Da3.u8DaMid[4] - 0x30) * 0.01;
}

/******************************************/
/*      [47] Data Parsing Function        */
/* Author: Gogume                         */
/* History: First Released 20190724       */
/* Digital Output Check Data Parsing Part */
/******************************************/
void Id47_Data_Parsing(u8 *pData)
{
    u8 i = 0;

    g_Id47Data.u8Da0.u8DaVariAdd[0]     = pData[24+i++];
    g_Id47Data.u8Da0.u8DaVariAdd[1]     = pData[24+i++];
    g_Id47Data.u8Da0.u8DaVariAdd[2]     = pData[24+i++];
    g_Id47Data.u8Da0.u8DaVariAdd[3]     = pData[24+i++];
    g_Id47Data.u8Da0.u8DaVariAdd[4]     = pData[24+i++];
    g_Id47Data.u8Da0.u8DaVariAdd[5]     = pData[24+i++];
    g_Id47Data.u8Da0.u8DaVariAdd[6]     = pData[24+i++];
    g_Id47Data.u8Da0.u8DaVariAdd[7]     = pData[24+i++];
    g_Id47Data.u8Da0.u8DaType           = pData[24+i++];
    g_Id47Data.u8Da0.u8DaScale[0]       = pData[24+i++];
    g_Id47Data.u8Da0.u8DaScale[1]       = pData[24+i++];
    g_Id47Data.u8Da0.u8DaScale[2]       = pData[24+i++];
    g_Id47Data.u8Da0.u8DaScale[3]       = pData[24+i++];
    g_Id47Data.u8Da0.u8DaScale[4]       = pData[24+i++];
    g_Id47Data.u8Da0.u8DaScale[5]       = pData[24+i++];
    g_Id47Data.u8Da0.u8DaScale[6]       = pData[24+i++];
    g_Id47Data.u8Da0.u8DaScale[7]       = pData[24+i++];
    g_Id47Data.u8Da0.u8DaMid[0]         = pData[24+i++];
    g_Id47Data.u8Da0.u8DaMid[1]         = pData[24+i++];
    g_Id47Data.u8Da0.u8DaMid[2]         = pData[24+i++];
    g_Id47Data.u8Da0.u8DaMid[3]         = pData[24+i++];
    g_Id47Data.u8Da0.u8DaMid[4]         = pData[24+i++];
    g_Id47Data.u8Da0.u8DaMid[5]         = pData[24+i++];

    g_Id47Data.u8Da1.u8DaVariAdd[0]     = pData[24+i++];
    g_Id47Data.u8Da1.u8DaVariAdd[1]     = pData[24+i++];
    g_Id47Data.u8Da1.u8DaVariAdd[2]     = pData[24+i++];
    g_Id47Data.u8Da1.u8DaVariAdd[3]     = pData[24+i++];
    g_Id47Data.u8Da1.u8DaVariAdd[4]     = pData[24+i++];
    g_Id47Data.u8Da1.u8DaVariAdd[5]     = pData[24+i++];
    g_Id47Data.u8Da1.u8DaVariAdd[6]     = pData[24+i++];
    g_Id47Data.u8Da1.u8DaVariAdd[7]     = pData[24+i++];
    g_Id47Data.u8Da1.u8DaType           = pData[24+i++];
    g_Id47Data.u8Da1.u8DaScale[0]       = pData[24+i++];
    g_Id47Data.u8Da1.u8DaScale[1]       = pData[24+i++];
    g_Id47Data.u8Da1.u8DaScale[2]       = pData[24+i++];
    g_Id47Data.u8Da1.u8DaScale[3]       = pData[24+i++];
    g_Id47Data.u8Da1.u8DaScale[4]       = pData[24+i++];
    g_Id47Data.u8Da1.u8DaScale[5]       = pData[24+i++];
    g_Id47Data.u8Da1.u8DaScale[6]       = pData[24+i++];
    g_Id47Data.u8Da1.u8DaScale[7]       = pData[24+i++];
    g_Id47Data.u8Da1.u8DaMid[0]         = pData[24+i++];
    g_Id47Data.u8Da1.u8DaMid[1]         = pData[24+i++];
    g_Id47Data.u8Da1.u8DaMid[2]         = pData[24+i++];
    g_Id47Data.u8Da1.u8DaMid[3]         = pData[24+i++];
    g_Id47Data.u8Da1.u8DaMid[4]         = pData[24+i++];
    g_Id47Data.u8Da1.u8DaMid[5]         = pData[24+i++];

    g_Id47Data.u8Da2.u8DaVariAdd[0]     = pData[24+i++];
    g_Id47Data.u8Da2.u8DaVariAdd[1]     = pData[24+i++];
    g_Id47Data.u8Da2.u8DaVariAdd[2]     = pData[24+i++];
    g_Id47Data.u8Da2.u8DaVariAdd[3]     = pData[24+i++];
    g_Id47Data.u8Da2.u8DaVariAdd[4]     = pData[24+i++];
    g_Id47Data.u8Da2.u8DaVariAdd[5]     = pData[24+i++];
    g_Id47Data.u8Da2.u8DaVariAdd[6]     = pData[24+i++];
    g_Id47Data.u8Da2.u8DaVariAdd[7]     = pData[24+i++];
    g_Id47Data.u8Da2.u8DaType           = pData[24+i++];
    g_Id47Data.u8Da2.u8DaScale[0]       = pData[24+i++];
    g_Id47Data.u8Da2.u8DaScale[1]       = pData[24+i++];
    g_Id47Data.u8Da2.u8DaScale[2]       = pData[24+i++];
    g_Id47Data.u8Da2.u8DaScale[3]       = pData[24+i++];
    g_Id47Data.u8Da2.u8DaScale[4]       = pData[24+i++];
    g_Id47Data.u8Da2.u8DaScale[5]       = pData[24+i++];
    g_Id47Data.u8Da2.u8DaScale[6]       = pData[24+i++];
    g_Id47Data.u8Da2.u8DaScale[7]       = pData[24+i++];
    g_Id47Data.u8Da2.u8DaMid[0]         = pData[24+i++];
    g_Id47Data.u8Da2.u8DaMid[1]         = pData[24+i++];
    g_Id47Data.u8Da2.u8DaMid[2]         = pData[24+i++];
    g_Id47Data.u8Da2.u8DaMid[3]         = pData[24+i++];
    g_Id47Data.u8Da2.u8DaMid[4]         = pData[24+i++];
    g_Id47Data.u8Da2.u8DaMid[5]         = pData[24+i++];

    g_Id47Data.u8Da3.u8DaVariAdd[0]     = pData[24+i++];
    g_Id47Data.u8Da3.u8DaVariAdd[1]     = pData[24+i++];
    g_Id47Data.u8Da3.u8DaVariAdd[2]     = pData[24+i++];
    g_Id47Data.u8Da3.u8DaVariAdd[3]     = pData[24+i++];
    g_Id47Data.u8Da3.u8DaVariAdd[4]     = pData[24+i++];
    g_Id47Data.u8Da3.u8DaVariAdd[5]     = pData[24+i++];
    g_Id47Data.u8Da3.u8DaVariAdd[6]     = pData[24+i++];
    g_Id47Data.u8Da3.u8DaVariAdd[7]     = pData[24+i++];
    g_Id47Data.u8Da3.u8DaType           = pData[24+i++];
    g_Id47Data.u8Da3.u8DaScale[0]       = pData[24+i++];
    g_Id47Data.u8Da3.u8DaScale[1]       = pData[24+i++];
    g_Id47Data.u8Da3.u8DaScale[2]       = pData[24+i++];
    g_Id47Data.u8Da3.u8DaScale[3]       = pData[24+i++];
    g_Id47Data.u8Da3.u8DaScale[4]       = pData[24+i++];
    g_Id47Data.u8Da3.u8DaScale[5]       = pData[24+i++];
    g_Id47Data.u8Da3.u8DaScale[6]       = pData[24+i++];
    g_Id47Data.u8Da3.u8DaScale[7]       = pData[24+i++];
    g_Id47Data.u8Da3.u8DaMid[0]         = pData[24+i++];
    g_Id47Data.u8Da3.u8DaMid[1]         = pData[24+i++];
    g_Id47Data.u8Da3.u8DaMid[2]         = pData[24+i++];
    g_Id47Data.u8Da3.u8DaMid[3]         = pData[24+i++];
    g_Id47Data.u8Da3.u8DaMid[4]         = pData[24+i++];
    g_Id47Data.u8Da3.u8DaMid[5]         = pData[24+i++];


    /* Address Transform */
    /* DA0 */
//    g_Id47Data.NumVariAdd[0]  = (long)(g_Id47Data.u8Da0.u8DaVariAdd[0] & 0x0F) << 28;
//    g_Id47Data.NumVariAdd[0] += (long)(g_Id47Data.u8Da0.u8DaVariAdd[1] & 0x0F) << 24;
//    g_Id47Data.NumVariAdd[0] += (long)(g_Id47Data.u8Da0.u8DaVariAdd[2] & 0x0F) << 20;
//    g_Id47Data.NumVariAdd[0] += (long)(g_Id47Data.u8Da0.u8DaVariAdd[3] & 0x0F) << 16;
//    g_Id47Data.NumVariAdd[0] += (long)(g_Id47Data.u8Da0.u8DaVariAdd[4] & 0x0F) << 12;
//    g_Id47Data.NumVariAdd[0] += (long)(g_Id47Data.u8Da0.u8DaVariAdd[5] & 0x0F) << 8;
//    g_Id47Data.NumVariAdd[0] += (long)(g_Id47Data.u8Da0.u8DaVariAdd[6] & 0x0F) << 4;
//    g_Id47Data.NumVariAdd[0] += (long)(g_Id47Data.u8Da0.u8DaVariAdd[7] & 0x0F) << 0;

    g_Id47Data.NumVariAdd[0]  = ((g_Id47Data.u8Da0.u8DaVariAdd[0] - 0x30) * 0x10000000);
    g_Id47Data.NumVariAdd[0] += ((g_Id47Data.u8Da0.u8DaVariAdd[1] - 0x30) * 0x1000000);
    g_Id47Data.NumVariAdd[0] += ((g_Id47Data.u8Da0.u8DaVariAdd[2] - 0x30) * 0x100000);
    g_Id47Data.NumVariAdd[0] += ((g_Id47Data.u8Da0.u8DaVariAdd[3] - 0x30) * 0x10000);
    g_Id47Data.NumVariAdd[0] += ((g_Id47Data.u8Da0.u8DaVariAdd[4] - 0x30) * 0x1000);
    g_Id47Data.NumVariAdd[0] += ((g_Id47Data.u8Da0.u8DaVariAdd[5] - 0x30) * 0x100);
    g_Id47Data.NumVariAdd[0] += ((g_Id47Data.u8Da0.u8DaVariAdd[6] - 0x30) * 0x10);
    g_Id47Data.NumVariAdd[0] += ((g_Id47Data.u8Da0.u8DaVariAdd[7] - 0x30) * 0x1);
    g_Id47Data.pNumVariAdd[0] = (long *)g_Id47Data.NumVariAdd[0];
    /* DA1 */
    g_Id47Data.NumVariAdd[1]  = ((g_Id47Data.u8Da1.u8DaVariAdd[0] - 0x30) * 0x10000000);
    g_Id47Data.NumVariAdd[1] += ((g_Id47Data.u8Da1.u8DaVariAdd[1] - 0x30) * 0x1000000);
    g_Id47Data.NumVariAdd[1] += ((g_Id47Data.u8Da1.u8DaVariAdd[2] - 0x30) * 0x100000);
    g_Id47Data.NumVariAdd[1] += ((g_Id47Data.u8Da1.u8DaVariAdd[3] - 0x30) * 0x10000);
    g_Id47Data.NumVariAdd[1] += ((g_Id47Data.u8Da1.u8DaVariAdd[4] - 0x30) * 0x1000);
    g_Id47Data.NumVariAdd[1] += ((g_Id47Data.u8Da1.u8DaVariAdd[5] - 0x30) * 0x100);
    g_Id47Data.NumVariAdd[1] += ((g_Id47Data.u8Da1.u8DaVariAdd[6] - 0x30) * 0x10);
    g_Id47Data.NumVariAdd[1] += ((g_Id47Data.u8Da1.u8DaVariAdd[7] - 0x30) * 0x1);
    g_Id47Data.pNumVariAdd[1] = (long *)g_Id47Data.NumVariAdd[1];
    /* DA2 */
    g_Id47Data.NumVariAdd[2]  = ((g_Id47Data.u8Da2.u8DaVariAdd[0] - 0x30) * 0x10000000);
    g_Id47Data.NumVariAdd[2] += ((g_Id47Data.u8Da2.u8DaVariAdd[1] - 0x30) * 0x1000000);
    g_Id47Data.NumVariAdd[2] += ((g_Id47Data.u8Da2.u8DaVariAdd[2] - 0x30) * 0x100000);
    g_Id47Data.NumVariAdd[2] += ((g_Id47Data.u8Da2.u8DaVariAdd[3] - 0x30) * 0x10000);
    g_Id47Data.NumVariAdd[2] += ((g_Id47Data.u8Da2.u8DaVariAdd[4] - 0x30) * 0x1000);
    g_Id47Data.NumVariAdd[2] += ((g_Id47Data.u8Da2.u8DaVariAdd[5] - 0x30) * 0x100);
    g_Id47Data.NumVariAdd[2] += ((g_Id47Data.u8Da2.u8DaVariAdd[6] - 0x30) * 0x10);
    g_Id47Data.NumVariAdd[2] += ((g_Id47Data.u8Da2.u8DaVariAdd[7] - 0x30) * 0x1);
    g_Id47Data.pNumVariAdd[2] = (long *)g_Id47Data.NumVariAdd[2];
    /* DA3 */
    g_Id47Data.NumVariAdd[3]  = ((g_Id47Data.u8Da3.u8DaVariAdd[0] - 0x30) * 0x10000000);
    g_Id47Data.NumVariAdd[3] += ((g_Id47Data.u8Da3.u8DaVariAdd[1] - 0x30) * 0x1000000);
    g_Id47Data.NumVariAdd[3] += ((g_Id47Data.u8Da3.u8DaVariAdd[2] - 0x30) * 0x100000);
    g_Id47Data.NumVariAdd[3] += ((g_Id47Data.u8Da3.u8DaVariAdd[3] - 0x30) * 0x10000);
    g_Id47Data.NumVariAdd[3] += ((g_Id47Data.u8Da3.u8DaVariAdd[4] - 0x30) * 0x1000);
    g_Id47Data.NumVariAdd[3] += ((g_Id47Data.u8Da3.u8DaVariAdd[5] - 0x30) * 0x100);
    g_Id47Data.NumVariAdd[3] += ((g_Id47Data.u8Da3.u8DaVariAdd[6] - 0x30) * 0x10);
    g_Id47Data.NumVariAdd[3] += ((g_Id47Data.u8Da3.u8DaVariAdd[7] - 0x30) * 0x1);
    g_Id47Data.pNumVariAdd[3] = (long *)g_Id47Data.NumVariAdd[3];

    /* Variable Type */
    g_Id47Data.type[0] = g_Id47Data.u8Da0.u8DaType - 0x30;  /* DA0 */
    g_Id47Data.type[1] = g_Id47Data.u8Da1.u8DaType - 0x30;  /* DA1 */
    g_Id47Data.type[2] = g_Id47Data.u8Da2.u8DaType - 0x30;  /* DA2 */
    g_Id47Data.type[3] = g_Id47Data.u8Da3.u8DaType - 0x30;  /* DA3 */

    /* Scale Setting */
    /* DA0 */
    g_Id47Data.Scale[0]  = (g_Id47Data.u8Da0.u8DaScale[0] - 0x30) * 10000;
    g_Id47Data.Scale[0] += (g_Id47Data.u8Da0.u8DaScale[1] - 0x30) * 1000;
    g_Id47Data.Scale[0] += (g_Id47Data.u8Da0.u8DaScale[2] - 0x30) * 100;
    g_Id47Data.Scale[0] += (g_Id47Data.u8Da0.u8DaScale[3] - 0x30) * 10;
    g_Id47Data.Scale[0] += (g_Id47Data.u8Da0.u8DaScale[4] - 0x30) * 1;
    g_Id47Data.Scale[0] += (g_Id47Data.u8Da0.u8DaScale[5] - 0x30) * 0.1;
    g_Id47Data.Scale[0] += (g_Id47Data.u8Da0.u8DaScale[6] - 0x30) * 0.01;
    g_Id47Data.Scale[0] += (g_Id47Data.u8Da0.u8DaScale[7] - 0x30) * 0.001;
    /* DA1 */
    g_Id47Data.Scale[1]  = (g_Id47Data.u8Da1.u8DaScale[0] - 0x30) * 10000;
    g_Id47Data.Scale[1] += (g_Id47Data.u8Da1.u8DaScale[1] - 0x30) * 1000;
    g_Id47Data.Scale[1] += (g_Id47Data.u8Da1.u8DaScale[2] - 0x30) * 100;
    g_Id47Data.Scale[1] += (g_Id47Data.u8Da1.u8DaScale[3] - 0x30) * 10;
    g_Id47Data.Scale[1] += (g_Id47Data.u8Da1.u8DaScale[4] - 0x30) * 1;
    g_Id47Data.Scale[1] += (g_Id47Data.u8Da1.u8DaScale[5] - 0x30) * 0.1;
    g_Id47Data.Scale[1] += (g_Id47Data.u8Da1.u8DaScale[6] - 0x30) * 0.01;
    g_Id47Data.Scale[1] += (g_Id47Data.u8Da1.u8DaScale[7] - 0x30) * 0.001;
    /* DA2 */
    g_Id47Data.Scale[2]  = (g_Id47Data.u8Da2.u8DaScale[0] - 0x30) * 10000;
    g_Id47Data.Scale[2] += (g_Id47Data.u8Da2.u8DaScale[1] - 0x30) * 1000;
    g_Id47Data.Scale[2] += (g_Id47Data.u8Da2.u8DaScale[2] - 0x30) * 100;
    g_Id47Data.Scale[2] += (g_Id47Data.u8Da2.u8DaScale[3] - 0x30) * 10;
    g_Id47Data.Scale[2] += (g_Id47Data.u8Da2.u8DaScale[4] - 0x30) * 1;
    g_Id47Data.Scale[2] += (g_Id47Data.u8Da2.u8DaScale[5] - 0x30) * 0.1;
    g_Id47Data.Scale[2] += (g_Id47Data.u8Da2.u8DaScale[6] - 0x30) * 0.01;
    g_Id47Data.Scale[2] += (g_Id47Data.u8Da2.u8DaScale[7] - 0x30) * 0.001;
    /* DA3 */
    g_Id47Data.Scale[3]  = (g_Id47Data.u8Da3.u8DaScale[0] - 0x30) * 10000;
    g_Id47Data.Scale[3] += (g_Id47Data.u8Da3.u8DaScale[1] - 0x30) * 1000;
    g_Id47Data.Scale[3] += (g_Id47Data.u8Da3.u8DaScale[2] - 0x30) * 100;
    g_Id47Data.Scale[3] += (g_Id47Data.u8Da3.u8DaScale[3] - 0x30) * 10;
    g_Id47Data.Scale[3] += (g_Id47Data.u8Da3.u8DaScale[4] - 0x30) * 1;
    g_Id47Data.Scale[3] += (g_Id47Data.u8Da3.u8DaScale[5] - 0x30) * 0.1;
    g_Id47Data.Scale[3] += (g_Id47Data.u8Da3.u8DaScale[6] - 0x30) * 0.01;
    g_Id47Data.Scale[3] += (g_Id47Data.u8Da3.u8DaScale[7] - 0x30) * 0.001;

    /* Mid Point Setting */
    /* DA0 */
    g_Id47Data.Mid[0]  = (g_Id47Data.u8Da0.u8DaMid[0] - 0x30) * 100;
    g_Id47Data.Mid[0] += (g_Id47Data.u8Da0.u8DaMid[1] - 0x30) * 10;
    g_Id47Data.Mid[0] += (g_Id47Data.u8Da0.u8DaMid[2] - 0x30) * 1;
    g_Id47Data.Mid[0] += (g_Id47Data.u8Da0.u8DaMid[3] - 0x30) * 0.1;
    g_Id47Data.Mid[0] += (g_Id47Data.u8Da0.u8DaMid[4] - 0x30) * 0.01;
    /* DA1 */
    g_Id47Data.Mid[1]  = (g_Id47Data.u8Da1.u8DaMid[0] - 0x30) * 100;
    g_Id47Data.Mid[1] += (g_Id47Data.u8Da1.u8DaMid[1] - 0x30) * 10;
    g_Id47Data.Mid[1] += (g_Id47Data.u8Da1.u8DaMid[2] - 0x30) * 1;
    g_Id47Data.Mid[1] += (g_Id47Data.u8Da1.u8DaMid[3] - 0x30) * 0.1;
    g_Id47Data.Mid[1] += (g_Id47Data.u8Da1.u8DaMid[4] - 0x30) * 0.01;
    /* DA2 */
    g_Id47Data.Mid[2]  = (g_Id47Data.u8Da2.u8DaMid[0] - 0x30) * 100;
    g_Id47Data.Mid[2] += (g_Id47Data.u8Da2.u8DaMid[1] - 0x30) * 10;
    g_Id47Data.Mid[2] += (g_Id47Data.u8Da2.u8DaMid[2] - 0x30) * 1;
    g_Id47Data.Mid[2] += (g_Id47Data.u8Da2.u8DaMid[3] - 0x30) * 0.1;
    g_Id47Data.Mid[2] += (g_Id47Data.u8Da2.u8DaMid[4] - 0x30) * 0.01;
    /* DA3 */
    g_Id47Data.Mid[3]  = (g_Id47Data.u8Da3.u8DaMid[0] - 0x30) * 100;
    g_Id47Data.Mid[3] += (g_Id47Data.u8Da3.u8DaMid[1] - 0x30) * 10;
    g_Id47Data.Mid[3] += (g_Id47Data.u8Da3.u8DaMid[2] - 0x30) * 1;
    g_Id47Data.Mid[3] += (g_Id47Data.u8Da3.u8DaMid[3] - 0x30) * 0.1;
    g_Id47Data.Mid[3] += (g_Id47Data.u8Da3.u8DaMid[4] - 0x30) * 0.01;
}

/******************************************/
/*      [48] Data Parsing Function        */
/* Author: Gogume                         */
/* History: First Released 20190724       */
/* Digital Output Check Data Parsing Part */
/******************************************/
void Id48_Data_Parsing(u8 *pData)
{
    u8 i = 0;

    g_Id48Data.u8StaticForceCurrentRef[0]= pData[24+i++];
    g_Id48Data.u8StaticForceCurrentRef[1]= pData[24+i++];
    g_Id48Data.u8StaticForceCurrentRef[2]= pData[24+i++];
    g_Id48Data.u8StaticForceCurrentRef[3]= pData[24+i++];
    g_Id48Data.u8StaticForceCurrentRef[4]= pData[24+i++];
    g_Id48Data.u8StaticForceCurrentRef[5]= pData[24+i++];

    g_Id48Data.StaticForceCurrentRef   = (g_Id48Data.u8StaticForceCurrentRef[0] - 0x30) * 100;
    g_Id48Data.StaticForceCurrentRef  += (g_Id48Data.u8StaticForceCurrentRef[1] - 0x30) * 10;
    g_Id48Data.StaticForceCurrentRef  += (g_Id48Data.u8StaticForceCurrentRef[2] - 0x30) * 1;
    g_Id48Data.StaticForceCurrentRef  += (g_Id48Data.u8StaticForceCurrentRef[3] - 0x30) * 0.1;
    g_Id48Data.StaticForceCurrentRef  += (g_Id48Data.u8StaticForceCurrentRef[4] - 0x30) * 0.01;
    g_Id48Data.StaticForceCurrentRef  += (g_Id48Data.u8StaticForceCurrentRef[5] - 0x30) * 0.001;
}


/****************************************/
/*	 Send IdF1 DSP Version Function		*/
/* Author: Gogume						*/
/* History: First Released 20181102		*/
/****************************************/
void Send_Version(HW_TX_OBJ *pCmd, u16 u16size)
{
	Byte i;
	DefErr ErrCode = 0;

	// FREEWAY VERTICAL DSP VERSTION 0.01
	g_IdF1Data.u8DataID 	= 0xF1;		// Data ID
	///////////// APP Information ////////////////
	for(i = 0; i < 12; i++)
	{
		g_IdF1Data.u8TxData[i] = DspVerStr[i];
	}
	/////////////// BOOT Information //////////////
	g_IdF1Data.u8TxData[12] = 0x30;
	g_IdF1Data.u8TxData[13] = 0x30;
	g_IdF1Data.u8TxData[14] = 0x30;
	g_IdF1Data.u8TxData[15] = 0x30;
	g_IdF1Data.u8TxData[16] = 0x30;
	g_IdF1Data.u8TxData[17] = 0x30;
	g_IdF1Data.u8TxData[18] = 0x30;
	g_IdF1Data.u8TxData[19] = 0x30;
	g_IdF1Data.u8TxData[20] = 0x30;
	g_IdF1Data.u8TxData[21] = 0x30;
	g_IdF1Data.u8TxData[22] = 0x30;
	g_IdF1Data.u8TxData[23] = 0x30;

	pCmd->TxData.u8DataID = g_IdF1Data.u8DataID;

	for(i = 0; i < u16size; i++)
	{
		pCmd->TxData.u8Data[i] = g_IdF1Data.u8TxData[i];
	}

	ErrCode = GenErrorCode(g_ErrorCode);

	INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

	for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
	{
#if SCIC_POLLING
		scic_xmit(Scic_Buf_To_Main[i]);
		while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
		ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
		asm(" NOP");
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;			// Scic TX Interrupt Enable
		while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
		Test_Sci_Tx = i;
	}
}

/****************************************/
/*		Send IdF0 Data  Function		*/
/* Author: Gogume						*/
/* History: First Released 20181107		*/
/****************************************/
void Send_IdF0(HW_TX_OBJ *pCmd, u16 u16size)
{
	u8 i;
	DefErr ErrCode = 0;
	u8 Index = 0;

//	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataID;
	pCmd->TxData.u8DataID = 0xF0;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
	pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
	pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
	pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
	pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
	pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;		// Need to insert Inverter Information
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;		// Need to insert Inverter Information
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;		// Need to insert Inverter Information
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;	// Need to insert Inverter Information
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
	pCmd->TxData.u8Data[Index++] = 0x30;

	for(i = 24; i < u16size; i++)
	{
		pCmd->TxData.u8Data[Index++] = 0x30;
	}

	Index = 24;

	memcpy(&g_ArmStatus, &g_SpiArmStatus, sizeof(g_SpiArmStatus));

	/* Insert Data to TX Buffer */
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8DecelFlg & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8ChimeFlg & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8VoiceFlg & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8RunOpenFlg & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8SpdPtnFlg & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8FlFindDir & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8FlInitOK & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8LMotAngleFind & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8InvErr & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8NearFlStop & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8NoCall & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8PosErr & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8FlInitWng & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8DoorZoneExceed & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8DoorZoneLack & 0xff);;
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8ElAntistall & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8InvWatchdog & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CallErr & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8LowSpdOp & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8ForceDecelOp & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8McucommuChk & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8MaxFloor[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8MaxFloor[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8MaxFloor[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8DecelFloor[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8DecelFloor[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8DecelFloor[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentFloor[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentFloor[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentFloor[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Dummy[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Dummy[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Dummy[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentSpeed[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentSpeed[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentSpeed[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentPos[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentPos[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentPos[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentPos[3] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentPos[4] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentPos[5] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentPos[6] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8CurrentPos[7] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8EL_Speed[0]  & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8EL_Speed[1]  & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8EL_Speed[2]  & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8INV_ErrCode & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8S_curveTime[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8S_curveTime[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8PTN_Accel[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8PTN_Accel[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8PTN_Accel[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse[3] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse[4] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse[5] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse[3] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse[4] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse[5] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse_Ref[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse_Ref[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse_Ref[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse_Ref[3] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse_Ref[4] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Iqse_Ref[5] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse_Ref[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse_Ref[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse_Ref[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse_Ref[3] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse_Ref[4] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Idse_Ref[5] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_DC_Link[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_DC_Link[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_DC_Link[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_DC_Link[3] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_DC_Link[4] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_DC_Link[5] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Vll[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Vll[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Vll[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Vll[3] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Vll[4] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Vll[5] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Speed_Ref[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Speed_Ref[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Speed_Ref[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Thrust[0] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Thrust[1] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Thrust[2] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Thrust[3] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Thrust[4] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8Inv_Thrust[5] & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8BK_status1 & 0xff);
    pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8BK_status2 & 0xff);


    for(i = 0; i < 27; i++)
    {
        pCmd->TxData.u8Data[Index++] = (g_ArmStatus.u8reserved[i] & 0xff);
    }

    /* End */

    ErrCode = GenErrorCode(g_ErrorCode);

	INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

	scic_tx_cnt = 0;
	TestTXCNT = 0;
	for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
	{
#if SCIC_POLLING
		scic_xmit(Scic_Buf_To_Main[i]);
		while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
		ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
		asm(" NOP");
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;			// Scic TX Interrupt Enable
		while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
		Test_Sci_Tx = i;
	}
}

/************************************************/
/*       Error Backup Data Send  Function       */
/* Author: Gogume                               */
/* History: First Released 20180305             */
/* subject: Error Backup Data Send              */
/************************************************/
void SendIdF5(HW_TX_OBJ *pCmd, u16 u16size)
{
    u32 i, j;
    DefErr ErrCode = 0;
    u8 Index = 0;

    /* Error Backup Data Read Code Here - START */
    pCmd->TxData.u8DataID = 0xF5;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;       // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;        // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;      // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;    // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
    pCmd->TxData.u8Data[Index++] = 0x30;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x30;
    }

    Index = 24;

    for(i = 0; i < ERRORSTORENUM; i++)
   {
       pCmd->TxData.u8Data[Index++] = g_IdF5Data[i].u8FaultID;
       pCmd->TxData.u8Data[Index++] = g_IdF5Data[i].rspDate.u8Year;
       pCmd->TxData.u8Data[Index++] = g_IdF5Data[i].rspDate.u8Month;
       pCmd->TxData.u8Data[Index++] = g_IdF5Data[i].rspDate.u8Date;
       pCmd->TxData.u8Data[Index++] = g_IdF5Data[i].rspTime.u8Hour;
       pCmd->TxData.u8Data[Index++] = g_IdF5Data[i].rspTime.u8Minute;
       pCmd->TxData.u8Data[Index++] = g_IdF5Data[i].rspTime.u8Second;

       for(j = 0; j < ERRORBACKUPDATANUM; j++)
       {
           pCmd->TxData.u8Data[Index++] = g_IdF5Data[i].u8ErrorData.u8ErrorData[j];
       }
   }
    /* Error Backup Data Read Code Here - END */

    ErrCode = GenErrorCode(g_ErrorCode);

    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
//      ScicRegs.SCITXBUF.bit.TXDT = Scic_Buf_To_Main[i];
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}

/*********************************************************************/
/*       Static Force Measurement Response Data Send  Function       */
/* Author: Gogume                                                    */
/* History: First Released 20191008                                  */
/* subject: Static Force Measurement Response Data Send              */
/*********************************************************************/
void SendIdFA(HW_TX_OBJ *pCmd, u16 u16size)
{
    u8 i;
    DefErr ErrCode = 0;
    u8 Index = 0;

    /* Error Backup Data Read Code Here - START */
    pCmd->TxData.u8DataID = 0xFA;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;       // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;        // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;      // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;    // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
    pCmd->TxData.u8Data[Index++] = 0x30;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x30;
    }

    Index = 24;

    pCmd->TxData.u8DataID = 0xFA;
    pCmd->TxData.u8Data[Index++] = g_Id48Data.u8StaticForceCurrentRef[0];
    pCmd->TxData.u8Data[Index++] = g_Id48Data.u8StaticForceCurrentRef[1];
    pCmd->TxData.u8Data[Index++] = g_Id48Data.u8StaticForceCurrentRef[2];
    pCmd->TxData.u8Data[Index++] = g_Id48Data.u8StaticForceCurrentRef[3];
    pCmd->TxData.u8Data[Index++] = g_Id48Data.u8StaticForceCurrentRef[4];
    pCmd->TxData.u8Data[Index++] = g_Id48Data.u8StaticForceCurrentRef[5];
    pCmd->TxData.u8Data[Index++] = g_IdFAData.u8Flag_RotorEstimation;
    pCmd->TxData.u8Data[Index++] = g_IdFAData.u8Flag_StaticForceCurrentCon;

    ErrCode = GenErrorCode(g_ErrorCode);

    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}
/************************************************/
/*      Motor Data Response Function            */
/* Author: Gogume                               */
/* History: First Released 20190304             */
/************************************************/
void Send_Id42(HW_TX_OBJ *pCmd, u16 u16size)
{
    u8 i;
    DefErr ErrCode = 0;
    u8 Index = 0;

    pCmd->TxData.u8DataID = 0x42;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;       // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;        // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;      // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;    // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
    pCmd->TxData.u8Data[Index++] = 0x30;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x30;
    }
    Index = 24;

    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8MotorType;
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8MotorCapacitor[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8MotorCapacitor[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8MotorCapacitor[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8MotorCapacitor[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8MotorCapacitor[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8MotorCapacitor[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedVoltage[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedVoltage[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedVoltage[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedVoltage[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedVoltage[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedVoltage[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedCurrent[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedCurrent[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedCurrent[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedCurrent[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedCurrent[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RatedCurrent[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8PolePitch[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8PolePitch[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8PolePitch[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8PolePitch[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8PolePitch[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8PolePitch[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8BaseSpeed[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8BaseSpeed[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8BaseSpeed[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8BaseSpeed[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8BaseSpeed[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8BaseSpeed[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8IqRate[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8IqRate[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8IqRate[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8IqRate[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8IqRate[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8IqRate[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8KnowRpos;
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposOffset[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposOffset[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposOffset[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposOffset[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposOffset[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposOffset[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposMethod;
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposEstimationTime[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposEstimationTime[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposEstimationTime[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposEstimationTime[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposEstimationTime[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8RposEstimationTime[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Ls[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Ls[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Ls[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Ls[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Ls[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Ls[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Rs[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Rs[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Rs[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Rs[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Rs[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Rs[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Kf[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Kf[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Kf[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Kf[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Kf[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Kf[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[0];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[1];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[2];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[3];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[4];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[5];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[6];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[7];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[8];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[9];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[10];
    pCmd->TxData.u8Data[Index++] = g_Id42Data.u8RxData.u8Reserved[11];

    ErrCode = GenErrorCode(g_ErrorCode);

    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}

/************************************************/
/*     Inverter Data Response Function          */
/* Author: Gogume                               */
/* History: First Released 20190304             */
/************************************************/

void Send_Id43(HW_TX_OBJ *pCmd, u16 u16size)
{
    u8 i;
    DefErr ErrCode = 0;
    u8 Index = 0;

    pCmd->TxData.u8DataID = 0x43;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;       // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;        // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;      // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;    // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
    pCmd->TxData.u8Data[Index++] = 0x30;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x30;
    }
    Index = 24;

    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxSpeed[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxSpeed[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxSpeed[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxSpeed[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxSpeed[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxSpeed[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxAcc[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxAcc[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxAcc[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxAcc[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxAcc[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxAcc[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxJerk[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxJerk[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxJerk[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxJerk[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxJerk[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxJerk[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Jm[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Jm[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Jm[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Jm[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Jm[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Jm[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8SCFFGain[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8SCFFGain[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8SCFFGain[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8SCFFGain[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8SCFFGain[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8SCFFGain[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8TorqueLimit[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8TorqueLimit[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8TorqueLimit[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8TorqueLimit[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8TorqueLimit[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8TorqueLimit[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleVdc[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleVdc[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleVdc[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleVdc[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleVdc[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleVdc[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleCurrent[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleCurrent[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleCurrent[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleCurrent[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleCurrent[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ScaleCurrent[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wcc[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wcc[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wcc[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wcc[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wcc[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wcc[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wsc[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wsc[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wsc[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wsc[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wsc[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wsc[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wpc[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wpc[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wpc[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wpc[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wpc[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Wpc[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8OverCurrent[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8OverCurrent[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8OverCurrent[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8OverCurrent[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8OverCurrent[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8OverCurrent[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8FwdDirection;
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8EncoderType;
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxFloor[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxFloor[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxFloor[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8InpectSpeed[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8InpectSpeed[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8InpectSpeed[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8InpectSpeed[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8InpectSpeed[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8InpectSpeed[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8CreepSpeed[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8CreepSpeed[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8CreepSpeed[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8CreepSpeed[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8CreepSpeed[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8CreepSpeed[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8RelevelSpeed[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8RelevelSpeed[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8RelevelSpeed[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8RelevelSpeed[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8RelevelSpeed[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8RelevelSpeed[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8AntistallTime;
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxLength[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxLength[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxLength[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxLength[3];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxLength[4];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8MaxLength[5];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ZspLevel[0];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ZspLevel[1];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ZspLevel[2];
    pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8ZspLevel[3];

    for(i = 0; i < 125; i++)
    {
        pCmd->TxData.u8Data[Index++] = g_Id43Data.u8RxData.u8Reserved[i];
    }

    ErrCode = GenErrorCode(g_ErrorCode);

    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}

/************************************************/
/*     Mode Data Response Function              */
/* Author: Gogume                               */
/* History: First Released 20190620             */
/************************************************/

void Send_Id44(HW_TX_OBJ *pCmd, u16 u16size)
{
    u8 i;
    DefErr ErrCode = 0;
    u8 Index = 0;

    pCmd->TxData.u8DataID = 0x43;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;       // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;        // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;      // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;    // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
    pCmd->TxData.u8Data[Index++] = 0x30;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x30;
    }
    Index = 24;

    pCmd->TxData.u8Data[Index++] = g_Id44Data.u8Mode;

    ErrCode = GenErrorCode(g_ErrorCode);

    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}

/************************************************/
/*  Digital Output Data Response Function       */
/* Author: Gogume                               */
/* History: First Released 20190620             */
/************************************************/

void Send_Id45(HW_TX_OBJ *pCmd, u16 u16size)
{
    u8 i;
    DefErr ErrCode = 0;
    u8 Index = 0;

    pCmd->TxData.u8DataID = 0x43;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;       // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;        // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;      // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;    // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
    pCmd->TxData.u8Data[Index++] = 0x30;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x30;
    }
    Index = 24;

    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDMC2;

    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDMC2;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDBKA;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDBKB;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDBKP;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDDS;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDMS;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nIFAN;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nMC_A;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDHCLD;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDCLD;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nINV_SPO1;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nDHYDPRESS;
    pCmd->TxData.u8Data[Index++] = g_Id45Data.u8nINV_SPO2;

    ErrCode = GenErrorCode(g_ErrorCode);

    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}

/************************************************/
/*     DAC Selection Response Function          */
/* Author: Gogume                               */
/* History: First Released 20190722             */
/************************************************/

void Send_Id46(HW_TX_OBJ *pCmd, u16 u16size)
{
    u8 i;
    DefErr ErrCode = 0;
    u8 Index = 0;

    pCmd->TxData.u8DataID = 0x46;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;       // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;        // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;      // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;    // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
    pCmd->TxData.u8Data[Index++] = 0x30;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x30;
    }
    Index = 24;
    /*DA0*/
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaVari[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaVari[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaType;
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaScale[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaScale[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaScale[2];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaScale[3];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaScale[4];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaScale[5];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaScale[6];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaScale[7];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaMid[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaMid[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaMid[2];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaMid[3];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da0.u8DaMid[4];
    /*DA1*/
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaVari[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaVari[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaType;
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaScale[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaScale[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaScale[2];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaScale[3];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaScale[4];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaScale[5];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaScale[6];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaScale[7];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaMid[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaMid[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaMid[2];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaMid[3];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da1.u8DaMid[4];
    /*DA2*/
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaVari[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaVari[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaType;
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaScale[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaScale[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaScale[2];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaScale[3];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaScale[4];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaScale[5];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaScale[6];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaScale[7];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaMid[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaMid[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaMid[2];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaMid[3];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da2.u8DaMid[4];
    /*DA3*/
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaVari[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaVari[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaType;
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaScale[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaScale[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaScale[2];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaScale[3];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaScale[4];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaScale[5];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaScale[6];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaScale[7];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaMid[0];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaMid[1];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaMid[2];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaMid[3];
    pCmd->TxData.u8Data[Index++] = g_Id46Data.u8Da3.u8DaMid[4];

    ErrCode = GenErrorCode(g_ErrorCode);

    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}

/************************************************/
/*  DAC Selection Mode 2 Response Function      */
/* Author: Gogume                               */
/* History: First Released 20190724             */
/************************************************/

void Send_Id47(HW_TX_OBJ *pCmd, u16 u16size)
{
    u8 i;
    DefErr ErrCode = 0;
    u8 Index = 0;

    pCmd->TxData.u8DataID = 0x47;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[0];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[1];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[2];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataLength[3];
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Year;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Month;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspDate.u8Date;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Hour;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Minute;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.rspTime.u8Second;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >> 12) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  8) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  4) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = ((g_IdCoData.u32TransNum >>  0) & 0x0F) | 0x30;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarSpeed;       // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarType;        // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarNumber;      // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8CarPosition;    // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LimitSensorMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8SPIMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8APSMode;
    pCmd->TxData.u8Data[Index++] = g_IdCoData.u8LinearScaleMode;
    pCmd->TxData.u8Data[Index++] = 0x30;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x30;
    }
    Index = 24;
    /*DA0*/
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaVariAdd[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaVariAdd[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaVariAdd[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaVariAdd[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaVariAdd[4];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaVariAdd[5];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaVariAdd[6];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaVariAdd[7];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaType;
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaScale[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaScale[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaScale[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaScale[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaScale[4];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaScale[5];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaScale[6];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaScale[7];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaMid[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaMid[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaMid[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaMid[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da0.u8DaMid[4];
    /*DA1*/
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaVariAdd[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaVariAdd[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaVariAdd[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaVariAdd[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaVariAdd[4];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaVariAdd[5];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaVariAdd[6];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaVariAdd[7];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaType;
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaScale[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaScale[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaScale[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaScale[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaScale[4];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaScale[5];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaScale[6];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaScale[7];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaMid[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaMid[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaMid[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaMid[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da1.u8DaMid[4];
    /*DA2*/
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaVariAdd[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaVariAdd[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaVariAdd[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaVariAdd[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaVariAdd[4];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaVariAdd[5];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaVariAdd[6];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaVariAdd[7];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaType;
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaScale[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaScale[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaScale[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaScale[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaScale[4];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaScale[5];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaScale[6];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaScale[7];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaMid[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaMid[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaMid[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaMid[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da2.u8DaMid[4];
    /*DA3*/
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaVariAdd[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaVariAdd[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaVariAdd[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaVariAdd[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaVariAdd[4];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaVariAdd[5];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaVariAdd[6];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaVariAdd[7];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaType;
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaScale[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaScale[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaScale[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaScale[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaScale[4];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaScale[5];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaScale[6];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaScale[7];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaMid[0];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaMid[1];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaMid[2];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaMid[3];
    pCmd->TxData.u8Data[Index++] = g_Id47Data.u8Da3.u8DaMid[4];

    ErrCode = GenErrorCode(g_ErrorCode);

    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}

/************************************************/
/*	u8 to float type casting  Function			*/
/* Author: Gogume								*/
/* History: First Released 20181121				*/
/* subject: u8 data before conversion 			*/
/* integer: The length of integer				*/
/* decimal: the length of decimal				*/
/* retern value: float data after conversion	*/
/************************************************/
float TypeCast_u8tofloat(u8 *subject, u8 integer, u8 decimal)
{
	u8 i;
	u8 u8CastingData[6];
	float fobject;

	for(i = 0; i < 6; i++)
	{
		u8CastingData[i] = subject[i] - 0x30;
		testTT[i] = subject[i] - 0x30;
	}

	fobject = 0.;

	if(integer == 4)
	{
		fobject  = u8CastingData[0] * 1000;
		fobject += u8CastingData[1] * 100;
		fobject += u8CastingData[2] * 10;
		fobject += u8CastingData[3] * 1;
	}
	else if(integer == 2)
	{
		fobject  = u8CastingData[0] * 10;
		fobject += u8CastingData[1] * 1;
	}

	if(decimal == 2)
	{
		fobject += u8CastingData[4] * 0.1;
		fobject += u8CastingData[5] * 0.01;
	}
	else if(decimal == 4)
	{
		fobject += u8CastingData[2] * 0.1;
		fobject += u8CastingData[3] * 0.01;
		fobject += u8CastingData[4] * 0.001;
		fobject += u8CastingData[5] * 0.0001;
	}

	return fobject;
}

/************************************************/
/*  float to u8 type casting  Function          */
/* Author: Gogume                               */
/* History: First Released 20190517             */
/* subject: float data before conversion        */
/* integer: The length of integer               */
/* decimal: the length of decimal               */
/* return value: u8 data after conversion       */
/************************************************/
void TypeCast_floattou8(float fsubject, u8 *object, u8 integer, u8 decimal)
{
    u32 temppower[6] = {0,};
    u32 temppower2[6] = {0,};
    u32 tempsubject = 0;

    tempsubject = (u32)(fsubject * powf(10,decimal));

    temppower[0] = (u32)(tempsubject * 0.00001) * 100000 ;
    temppower[1] = (u32)(tempsubject * 0.0001)  * 10000  ;
    temppower[2] = (u32)(tempsubject * 0.001)   * 1000   ;
    temppower[3] = (u32)(tempsubject * 0.01)    * 100    ;
    temppower[4] = (u32)(tempsubject * 0.1)     * 10     ;
    temppower[5] = (u32)(tempsubject * 1)       * 1      ;

    temppower2[0] = temppower[0]                    * 0.00001;
    temppower2[1] = (temppower[1] - temppower[0])   * 0.0001 ;
    temppower2[2] = (temppower[2] - temppower[1])   * 0.001  ;
    temppower2[3] = (temppower[3] - temppower[2])   * 0.01   ;
    temppower2[4] = (temppower[4] - temppower[3])   * 0.1    ;
    temppower2[5] = (temppower[5] - temppower[4])   * 1      ;


    if((integer+decimal) == 6){
        object[0] = (temppower2[0] & 0x0F)|0x30;
        object[1] = (temppower2[1] & 0x0F)|0x30;
        object[2] = (temppower2[2] & 0x0F)|0x30;
        object[3] = (temppower2[3] & 0x0F)|0x30;
        object[4] = (temppower2[4] & 0x0F)|0x30;
        object[5] = (temppower2[5] & 0x0F)|0x30;
    }
    else if((integer+decimal) == 3){
        object[0] = (temppower2[3] & 0x0F)|0x30;
        object[1] = (temppower2[4] & 0x0F)|0x30;
        object[2] = (temppower2[5] & 0x0F)|0x30;
    }

}

void DectoHex(u32 dec, u8 *hex){



}

/************************************************/
/*       CPU Watch-dog Reset  Function          */
/* Author: Gogume                               */
/* History: First Released 20180103             */
/* subject: CPU Reset from ARM                  */
/************************************************/
void DspSwReset(HW_RX_OBJ *pCmd)
{
    while(1){
        FLT_Reset();
        EALLOW;
        WdRegs.WDKEY.bit.WDKEY = 0x0011;
        WdRegs.WDKEY.bit.WDKEY = 0x00AA;

        WdRegs.SCSR.bit.WDENINT = 0x0;
        WdRegs.WDCR.all = 0x10;
        EDIS;
    }
}

/************************************************/
/*       SCI Normal Acknowledge  Function       */
/* Author: Gogume                               */
/* History: First Released 20180305             */
/* subject: Communication done Acknowledge      */
/************************************************/
void AckSend(HW_TX_OBJ *pCmd, u16 u16size)
{
    u8 i;
    u8 Index = 0;
    DefErr ErrCode;

    pCmd->TxData.u8DataID = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;   // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = 0x00;   // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = 0x00;   // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = 0x00;   // Need to insert Inverter Information
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;
    pCmd->TxData.u8Data[Index++] = 0x00;

    for(i = 24; i < u16size; i++)
    {
        pCmd->TxData.u8Data[Index++] = 0x00;
    }

    ErrCode = GenErrorCode(g_ErrorCode);
    INV_To_Main_Msg_Pkt(pCmd, u16size, ErrCode);

    scic_tx_cnt = 0;
    TestTXCNT = 0;
    for(i = 0; i < (u16size + TX_PACKET_FORMAT_LENGTH); i++)
    {
#if SCIC_POLLING
      scic_xmit(Scic_Buf_To_Main[i]);
      while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#else
        ScicRegs.SCIFFTX.bit.TXFFIENA = 0 ;
        asm(" NOP");
        ScicRegs.SCIFFTX.bit.TXFFIENA = 1 ;         // Scic TX Interrupt Enable
        while(ScicRegs.SCICTL2.bit.TXRDY == 0);
#endif
        Test_Sci_Tx = i;
    }
}

/************************************************/
/*       Error Code Generation  Function        */
/* Author: Gogume                               */
/* History: First Released 20180305             */
/* subject: Error Code Generation               */
/************************************************/
DefErr GenErrorCode(u8 ErrorCode)
{
    DefErr TempErrorCode = 0;

    if(ErrorCode == 0){
        TempErrorCode = 0x0000;
    }
    else
    {
        if(g_IdCoData.u8CarType == 0x30)        // Vertical
        {
            TempErrorCode = ErrorCode|0x4000;
        }
        else if(g_IdCoData.u8CarType == 0x31)   // Horizontal
        {
            TempErrorCode = ErrorCode|0x4100;
        }
        else TempErrorCode = ErrorCode|0x4000;
    }

    return TempErrorCode;
}

/************************************************/
/*       Motor Data Load Function               */
/* Author: Gogume                               */
/* History: First Released 20190517             */
/* subject: Load Motor Data                     */
/************************************************/
void Load_Motor_Data()
{
    g_Id42Data.u8RxData.u8MotorType = FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_SELECT) + 0x30;                                           // Motor Type

    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_CAPACIT), g_Id42Data.u8RxData.u8MotorCapacitor, 4, 2);            // Motor Capacitor: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_RATING_V), g_Id42Data.u8RxData.u8RatedVoltage, 4, 2);             // Rated Voltage: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_RATING_A), g_Id42Data.u8RxData.u8RatedCurrent, 4, 2);             // Rated Current: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_POLE_PITCH), g_Id42Data.u8RxData.u8PolePitch, 4, 2);              // Pole Pitch: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_BASE_SPEED), g_Id42Data.u8RxData.u8BaseSpeed, 4, 2);              // Base Speed: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_IQSE_RATE), g_Id42Data.u8RxData.u8IqRate, 4, 2);                  // Q-axis Rated Current: Integer(4), Decimal(2)

    g_Id42Data.u8RxData.u8KnowRpos = FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_KNOW_RPOS) + 0x30;                                         // Do you Know Rotor Position?

    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ENC_OFFSET), g_Id42Data.u8RxData.u8RposOffset, 4, 2);        // Rotor Position Offset: Integer(4), Decimal(2)

    g_Id42Data.u8RxData.u8RposMethod = FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ESTIMAION) + 0x30;                                  // Rotor Position Estimation Method

    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_RPOS_ESTI_TIME), g_Id42Data.u8RxData.u8RposEstimationTime, 4, 2); // DC Injection time: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_LS), g_Id42Data.u8RxData.u8Ls, 4, 2);                             // Motor Inductance: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_RS), g_Id42Data.u8RxData.u8Rs, 2, 4);                             // Motor Resistance: Integer(2), Decimal(4)
    TypeCast_floattou8(FRAM_Read(KEY_LINMOTOR_START+LIN_MOTOR_KF), g_Id42Data.u8RxData.u8Kf, 2, 4);                             // Thrust Force Constant: Integer(2), Decimal(4)

}

/************************************************/
/*       Inverter Data Load Function            */
/* Author: Gogume                               */
/* History: First Released 20190517             */
/* subject: Load Inverter Data                  */
/************************************************/
void Load_Inverter_Data()
{
    float32 temp;

    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+MAX_SPEED),  g_Id43Data.u8RxData.u8MaxSpeed, 4, 2);                          // Max Speed: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+MAX_ACC),  g_Id43Data.u8RxData.u8MaxAcc, 4, 2);                              // Max Accelation: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+MAX_JERK),  g_Id43Data.u8RxData.u8MaxJerk, 4, 2);                            // Max Jerk: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+SC_JM),  g_Id43Data.u8RxData.u8Jm, 4, 2);                                    // Inertia: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+SC_FF_GAIN),  g_Id43Data.u8RxData.u8SCFFGain, 4, 2);                         // Speed Controller FeedForward Gain: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_FACTORY_START+THRUST_FORCE_LIMIT),  g_Id43Data.u8RxData.u8TorqueLimit, 4, 2);              // Torque Limit: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_FACTORY_START+SCALE_VDC),  g_Id43Data.u8RxData.u8ScaleVdc, 2, 4);                          // DC-Link Voltage Scale Factor: Integer(2), Decimal(4)
    TypeCast_floattou8(FRAM_Read(KEY_FACTORY_START+SCALE_IS),  g_Id43Data.u8RxData.u8ScaleCurrent, 2, 4);                       // Current Scale Factor: Integer(2), Decimal(4)
    TypeCast_floattou8(FRAM_Read(KEY_FACTORY_START+INV_WCC),  g_Id43Data.u8RxData.u8Wcc, 4, 2);                                 // Current Controller Bandwidth: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_FACTORY_START+WSC),  g_Id43Data.u8RxData.u8Wsc, 4, 2);                                     // Speed Controller Bandwidth: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_FACTORY_START+WPC),  g_Id43Data.u8RxData.u8Wpc, 4, 2);                                     // Position Controller Bandwidth: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_FACTORY_START+INV_OC_SET),  g_Id43Data.u8RxData.u8OverCurrent, 4, 2);                      // Over Current Setting: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_FACTORY_START+INV_OC_SET),  g_Id43Data.u8RxData.u8OverCurrent, 4, 2);

    g_Id43Data.u8RxData.u8FwdDirection = FRAM_Read(KEY_CONTROL_START+FWD_DIRECTION) + 0x30;
    g_Id43Data.u8RxData.u8EncoderType = FRAM_Read(KEY_FACTORY_START+ENCODERTYPE) + 0x30;

    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+MAX_FLOOR),  g_Id43Data.u8RxData.u8MaxFloor, 3, 0);                          // Max Floor
    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+INSPECT_SPEED),  g_Id43Data.u8RxData.u8InpectSpeed, 4, 2);                   // Inspect Speed Setting: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+CREEP_SPEED),  g_Id43Data.u8RxData.u8CreepSpeed, 4, 2);                      // Creep Speed Setting: Integer(4), Decimal(2)
    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+RELEVEL_SPEED),  g_Id43Data.u8RxData.u8RelevelSpeed, 4, 2);                  // Re-level Speed Setting: Integer(4), Decimal(2)

    g_Id43Data.u8RxData.u8AntistallTime = FRAM_Read(KEY_CONTROL_START+ANTISTALL_TIME) + 0x30;

    TypeCast_floattou8(FRAM_Read(KEY_CONTROL_START+MAX_LENGTH), g_Id43Data.u8RxData.u8MaxLength, 6,0);                          // MAX Length: Integer(6), Decimal(0)
    TypeCast_floattou8(FRAM_Read(KEY_INTERFACE_START+ZSP_LEVEL), g_Id43Data.u8RxData.u8ZspLevel, 1, 3);                         // ZSP Level: Integer(1), Decimal(3)
//    temp = FRAM_Read(KEY_CONTROL_START+MAX_LENGTH);
//    g_Id43Data.u8RxData.u8MaxLength[0] = (((u32)(temp * 0.00001))   & 0x0F)|0x30;
//    g_Id43Data.u8RxData.u8MaxLength[1] = (((u32)(temp * 0.0001))    & 0x0F)|0x30;
//    g_Id43Data.u8RxData.u8MaxLength[2] = (((u32)(temp * 0.001))     & 0x0F)|0x30;
//    g_Id43Data.u8RxData.u8MaxLength[3] = (((u32)(temp * 0.01))      & 0x0F)|0x30;
//    g_Id43Data.u8RxData.u8MaxLength[4] = (((u32)(temp * 0.1))       & 0x0F)|0x30;
//    g_Id43Data.u8RxData.u8MaxLength[5] = (((u32)(temp * 1))         & 0x0F)|0x30;

    temp = FRAM_Read(KEY_INTERFACE_START+ZSP_LEVEL);
    g_Id43Data.u8RxData.u8ZspLevel[0] = (((u32)(temp * 1))      & 0x0F)|0x30;
    g_Id43Data.u8RxData.u8ZspLevel[1] = (((u32)(temp * 10))     & 0x0F)|0x30;
    g_Id43Data.u8RxData.u8ZspLevel[2] = (((u32)(temp * 100))    & 0x0F)|0x30;
    g_Id43Data.u8RxData.u8ZspLevel[3] = (((u32)(temp * 1000))   & 0x0F)|0x30;
}

/************************************************/
/*       DAC Setting Data Load Function         */
/* Author: Gogume                               */
/* History: First Released 20190722             */
/* subject: Load DAC Setting Data               */
/************************************************/
void Load_DAC_Setting()
{
    g_Id46Data.NumVari[0] = FRAM_Read(KEY_INTERFACE_START+AO1_SELECT);
    g_Id46Data.NumVari[1] = FRAM_Read(KEY_INTERFACE_START+AO2_SELECT);
    g_Id46Data.NumVari[2] = FRAM_Read(KEY_INTERFACE_START+AO3_SELECT);
    g_Id46Data.NumVari[3] = FRAM_Read(KEY_INTERFACE_START+AO4_SELECT);

    g_Id46Data.Mid[0] = FRAM_Read(KEY_INTERFACE_START+AO_1_CENTER);
    g_Id46Data.Mid[1] = FRAM_Read(KEY_INTERFACE_START+AO_2_CENTER);
    g_Id46Data.Mid[2] = FRAM_Read(KEY_INTERFACE_START+AO_3_CENTER);
    g_Id46Data.Mid[3] = FRAM_Read(KEY_INTERFACE_START+AO_4_CENTER);

    g_Id46Data.Scale[0] = FRAM_Read(KEY_INTERFACE_START+AO_1_RANGE);
    g_Id46Data.Scale[1] = FRAM_Read(KEY_INTERFACE_START+AO_2_RANGE);
    g_Id46Data.Scale[2] = FRAM_Read(KEY_INTERFACE_START+AO_3_RANGE);
    g_Id46Data.Scale[3] = FRAM_Read(KEY_INTERFACE_START+AO_4_RANGE);

    InitDa(0);
}
