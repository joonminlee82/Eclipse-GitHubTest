/*****************************************************************************
-------------------------------Organization------------------------------------
	Project				: FREEWAY
	Compiler    		: TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
	Author				: Inverter part, Advanced Controls Research Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.12.28
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
						: TX_INDEX 20 --> 40 (20180806 by GWON)
						: Added SCI Protocol Version Response Part (20181107 by GWON)
						: Added SCI Protocol Infomation Response Part (20181107 by GWON)
						: Rearrangement of F/W for FREEWAY (20181112 by GWON)
						: Added SPI Init Setting for between ARM and DSP communication (20181127 by GWON)
						: Added SPI Control Communication Part (20181130 by GWON)
						: Ver2 Board GPIO Setting (20181207 by GWON)
						: Verified SPI Protocol (20181224 by GWON & Shin)
						: SPI, SCI Status Data Variable Updated (20190212 by GWON)
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
#include "SPI.h"
#include "vane.h"
#include "gnl.h"

#pragma CODE_SECTION(cpu1_Spib_Rx, "ramfuncs");
#pragma CODE_SECTION(cpu1_Spib_Tx, "ramfuncs");
#pragma CODE_SECTION(cpu1_Spic_Rx, "ramfuncs");
#pragma CODE_SECTION(cpu1_Spic_Tx, "ramfuncs");

HW_RX_OBJ 		Hw_RX_Spi_Data;
HW_TX_OBJ		Hw_TX_Spi_Data;

IdCoData		g_IdSpiCoData;
Id41Data		g_IdSpi41Data;

STATUS_DSP_OBJ 	g_SpiDspStatus;
STATUS_ARM_OBJ  g_SpiArmStatus;

ENC_DATA_OBJ    ENC1_Data;
ENC_DATA_OBJ    ENCB_Data;
ENC_DATA_OBJ    ENCA_Data;
ENC_DATA_OBJ    ENCA_Data_SC;

Byte Spib_Buf_To_Main[TO_MAIN_PACKET_LENGTH];

u8 u8BuffCnt = 0;

u8 TestSpiPacket = 0;

extern u8 g_ErrorCode;

void InitSPIB(void)
{

// Initialize SPI FIFO registers
   SpibRegs.SPICCR.bit.SPISWRESET = 0; 		// Reset SPI

   SpibRegs.SPICCR.all = 0x000F;       		// 16-bit character,
   SpibRegs.SPICCR.bit.CLKPOLARITY = 1;		// Rising Edge, Falling Edge
//   SpibRegs.SPICCR.bit.SPILBK = 1;		// Loopback Disable: 0x0, Loopback Enable: 0x1 for SPI Test 20181127 by GWON
   SpibRegs.SPICTL.all = 0x0013;       		// Interrupt enabled, Master/Slave XMIT enabled, DSP is Slave.	-->> DSP is Master for Loopback Test
   SpibRegs.SPISTS.all = 0x0000;
   SpibRegs.SPIBRR.all = SPIB_BRR & 0x7f;	// Baud rate: 50MHz/(3+1) = 12.5MHz
// SpibRegs.SPIFFTX.all = 0xE040;      		// Enable FIFO's, set TX FIFO level to 1, SpicRegs.SPIFFTX.bit.TXFFIENA
   SpibRegs.SPIFFRX.all = 0x2041;      		// Enalbe RX FIFO, set RX FIFO level to 1
   SpibRegs.SPIFFCT.all = 0x0000;			// No FIFO Transmit Delay Bits
   SpibRegs.SPIPRI.all = 0x0010;			// Set so breakpoints don't disturb xmission
   SpibRegs.SPIPRI.bit.STEINV = 0;
//   SpicRegs.SPIFFCT.all = 0x01;

   SpibRegs.SPICCR.bit.SPISWRESET = 1;  	// Enable SPI

//   SpibRegs.SPIFFTX.bit.TXFIFO = 1;
   SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;

   SpibRegs.SPITXBUF = 0x02;
   u8SpibTxCnt = 0;
}

void InitSPIC(void)
{
//    SpicRegs.SPIFFTX.all = 0xE028;
//    SpicRegs.SPIFFRX.all = 0x2028;
//    SpicRegs.SPIFFCT.all = 0x00;
//
//    SpibRegs.SPIPRI.all = 0x0000;            // Set so breakpoints don't disturb xmission
//    SpibRegs.SPIPRI.bit.STEINV = 0;
//
//    SpicRegs.SPIFFTX.bit.TXFIFO = 1;            // TX FIFO Rest
//    SpicRegs.SPIFFRX.bit.RXFIFORESET = 1;       // RX FIFO Rest
//
//    SpicRegs.SPICCR.bit.SPISWRESET = 0;
//    SpicRegs.SPICCR.bit.CLKPOLARITY = 0;
//    SpicRegs.SPICCR.bit.SPICHAR = (16 -1);
//    SpicRegs.SPICCR.bit.SPILBK = 1;             // Loopback Test Enable
//
//    // Enable master (0 == slave, 1 == master)
//    // Enable transmission (Talk)
//    // Clock phase (0 == normal, 1 == delayed)
//    // SPI interrupts are disabled
//    SpicRegs.SPICTL.bit.MASTER_SLAVE = 1;       // Slave
//    SpicRegs.SPICTL.bit.TALK = 1;               // Enable Transmission
//    SpicRegs.SPICTL.bit.CLK_PHASE = 0;          // Normal
//    SpicRegs.SPICTL.bit.SPIINTENA = 1;          // Interrupt Enable
//
//    // Set the baud rate
//    SpicRegs.SPIBRR.all = SPIC_BRR & 0x7f;      // LSPCLK/3 = 12.5[MHz]
//
//    // Set FREE bit
//    // Halting on a breakpoint will not halt the SPI
//    SpicRegs.SPIPRI.bit.FREE = 1;
//
//    // Release the SPI from reset
//    SpicRegs.SPICCR.bit.SPISWRESET = 1;

    SpicRegs.SPIFFRX.all=0x2040;             // RX FIFO enabled, clear FIFO int
    SpicRegs.SPIFFRX.bit.RXFFIL = FIFO_LVL;  // Set RX FIFO level

    SpicRegs.SPIFFTX.all=0xE040;             // FIFOs enabled, TX FIFO released,
    SpicRegs.SPIFFTX.bit.TXFFIL = FIFO_LVL;  // Set TX FIFO level

//
// Initialize core SPI registers
//
// Initialize SPI-A

    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 16-bit character
    // Enable loop-back
    SpicRegs.SPICCR.bit.SPISWRESET = 0;
    SpicRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpicRegs.SPICCR.bit.SPICHAR = (16-1);
    SpicRegs.SPICCR.bit.SPILBK = 0;

    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpicRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpicRegs.SPICTL.bit.TALK = 1;
    SpicRegs.SPICTL.bit.CLK_PHASE = 0;
    SpicRegs.SPICTL.bit.SPIINTENA = 0;

    // Set the baud rate
    SpicRegs.SPIBRR.bit.SPI_BIT_RATE = SPIC_BRR;

    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpicRegs.SPIPRI.bit.FREE = 1;

    // Release the SPI from reset
    SpicRegs.SPICCR.bit.SPISWRESET = 1;

    ENCA_Data.spi = &SpicRegs;
}

__interrupt void cpu1_Spib_Rx(void)
{
	u8 u8MCP_COM_CNT = 0;
	u8SpibRxCnt++;			// SPIB RX Interrupt Counter

	IER &= 0x0000;
	IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT8 | M_INT10 | M_INT12 ;			// Fault || CC || SC || Drive || Vane = Enc_Z
	EINT;

	TestSpi[u8BuffCnt] = SpibRegs.SPIRXBUF;
	if(TestSpi[0] == STX){
	    g_SpiArmStatus.u8McucommuChk = 0x30;				// SPI Okay
		u8BuffCnt++;
	}
	else{
		u8MCP_COM_CNT++;
		if(u8MCP_COM_CNT > COM_FLT_LMT ){
			FLT_Raise(FLT_CP_COM);
			g_SpiArmStatus.u8McucommuChk = 0x31;			// SPI Fault
			InitSPIB();
		}
		u8BuffCnt = 0;
	}

	if (u8BuffCnt >= 142)
	{
		u8BuffCnt = 0;
		SpibRegs.SPITXBUF=0x02;
		MakeSendSPIData();
	}
	else
	{
		SpibRegs.SPITXBUF=Spib_Buf_To_Main[u8BuffCnt];
	}

	SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;	// Clear Overflow flag
	SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;	// Clear Interrupt flag
	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
	asm(" NOP");
}

__interrupt void cpu1_Spib_Tx(void)
{
	IER &= 0x0000;
	IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT8 | M_INT10 | M_INT12 ;			// Fault || CC || SC || Drive || Vane = Enc_Z
	EINT;

	SpibRegs.SPITXBUF=Spib_Buf_To_Main[u8SpibTxCnt];
	u8SpibTxCnt++;			// SPIB TX Interrupt Counter

	u16SpiTXsize = 128;
	if(u8SpibTxCnt > (u16SpiTXsize + TX_PACKET_FORMAT_LENGTH)-1)
	{
//		u8SpibTxCnt = 0 ;
		SpibRegs.SPIFFTX.bit.TXFIFO = 0;
		asm(" NOP");
		asm(" NOP");
		SpibRegs.SPIFFTX.bit.TXFIFO = 1;
	}

	SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1;
	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
	SpibRegs.SPIFFTX.bit.TXFFIENA = 0;
	asm(" NOP");
}

__interrupt void cpu1_Spic_Rx(void)
{
    int i;

//    TEST1_ON;

    IER &= 0x0000;
    IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT8 | M_INT10 | M_INT12 ;         // Fault || CC || SC || Drive || Vane = Enc_Z
    EINT;
    /* USER CODE START */
    for(i = 0; i < 8; i++)
    {
        ENC1_Data.rdata[i] = SpicRegs.SPIRXBUF;
        if((i == 0)){
            if(ENC1_Data.rdata[i] == STX) u8Spic_Comm_Chk = 1;
            else{
                u8Spic_Comm_Chk = 0;
            }
        }

        if(u8Spic_Comm_Chk == 0){
            SpicRegs.SPIFFRX.bit.RXFIFORESET = 1;
            SpicRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Clear Overflow flag
            SpicRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Clear Interrupt flag
        }
    }

    ENC1_Data.rxcount++;
    /* USER CODE END */
    SpicRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Clear Overflow flag
    SpicRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Clear Interrupt flag
    // Acknowledge this interrupt to recieve more interrupts from group 9
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
//    TEST1_OFF;
}

__interrupt void cpu1_Spic_Tx(void)
{
    int i;

    IER &= 0x0000;
    IER |=  M_INT2 | M_INT3 | M_INT5 | M_INT8 | M_INT10 | M_INT12 ;         // Fault || CC || SC || Drive || Vane = Enc_Z
    EINT;
    /* USER CODE START */
    ENC1_Data.txcount++;

    ENC1_Data.sdata[0] = 0x02;
    ENC1_Data.sdata[1] = 0x10;
    ENC1_Data.sdata[2] = 0x11;
    ENC1_Data.sdata[3] = 0x12;
    ENC1_Data.sdata[4] = 0x13;
    ENC1_Data.sdata[5] = 0x14;
    ENC1_Data.sdata[6] = 0x15;
    ENC1_Data.sdata[7] = 0x16;
    ENC1_Data.sdata[8] = 0x17;
    ENC1_Data.sdata[9] = 0x03;


    for(i = 0; i < 8; i++)
    {
        SpicRegs.SPITXBUF = ENC1_Data.sdata[i];
    }

    /* USER CODE END */
    SpicRegs.SPIFFTX.bit.TXFFINTCLR = 1;
    // Acknowledge this interrupt to recieve more interrupts from group 9
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    SpicRegs.SPIFFTX.bit.TXFFIENA = 0;
    asm(" NOP");
}

void MakeSendSPIData(void)
{
	int i;
	int j=0;
	u8 u8Data;

	HW_RX_OBJ *pCmd;
	pCmd = &Hw_RX_Spi_Data;


	for (i = 0 ; i < 142; i++)
	{
		if(TestSpi[0]==STX)
		{
			u8Data = TestSpi[i];
		}
		else
		{
			pCmd->u16ErrorCode = ERROR_COMM_STX;
		}

		j++;
		pCmd->u8BCC = (TestSpi[j]&0xFF);
		j++;
		pCmd->u8Length[0] = (TestSpi[j]&0xFF);
		j++;
		pCmd->u8Length[1] = (TestSpi[j]&0xFF);
		j++;
		pCmd->u8Length[2] = (TestSpi[j]&0xFF);
		j++;
		pCmd->u8Length[3] = (TestSpi[j]&0xFF);
		j++;
		pCmd->u8Command[0] = (TestSpi[j]&0xFF);
		j++;
		pCmd->RxData.u8DataID = 0x41;
		j = 0;

		switch(pCmd->u8State)
		{
			case CMD_RXD_STATE_WAIT_STX:
				if(u8Data == STX)
				{
					pCmd->u8BCC_Calc = 0;
					pCmd->u32Timeout_Count = ReadCpuTimer0Counter();
					pCmd->u8IsCommand = FALSE;
					pCmd->u16RxDataCount = 0;
					pCmd->u8State = CMD_RXD_STATE_WAIT_BCC;
					u8SpibTxCnt = 0;
				}
				else pCmd->u8State = CMD_RXD_STATE_WAIT_STX;
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
				if ((u8Data == ETX) && (pCmd->u8BCC == pCmd->u8BCC_Calc))
				{
					pCmd->u8IsCommand = TRUE;
					DSP_COMMAND_SPI_TASK(&Hw_RX_Spi_Data, &Hw_TX_Spi_Data);			// SPIB Commnad Check
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
		}//switch
	}// for

}

/****************************************/
/*		Make Sending Data Packet 		*/
/* Author: Gogume						*/
/* History: First Released 20181130		*/
/****************************************/
void INV_To_Main_Spi_Msg_Pkt(HW_TX_OBJ *pCmd, u32 size, DefErr Error)
{
	u32 u32Length;
	u8 j;
	u8 spi_tx_ptr = 0 ;
	u8 bcc = 0;

	Spib_Buf_To_Main[spi_tx_ptr++] = STX;
	Spib_Buf_To_Main[spi_tx_ptr++] = 0x00;	// BCC
	Spib_Buf_To_Main[spi_tx_ptr++] = 0x00;	// LEN0
	Spib_Buf_To_Main[spi_tx_ptr++] = 0x00;	// LEN1
	Spib_Buf_To_Main[spi_tx_ptr++] = 0x00;	// LEN2
	Spib_Buf_To_Main[spi_tx_ptr++] = 0x00;	// LEN3

	Spib_Buf_To_Main[spi_tx_ptr++] = pCmd->u8Command[0];
	Spib_Buf_To_Main[spi_tx_ptr++] = pCmd->u8Command[1];

	if(Error == 0x00) Scic_Buf_To_Main[spi_tx_ptr++] = 'O';
	else Scic_Buf_To_Main[spi_tx_ptr++] = 'F';

	Spib_Buf_To_Main[spi_tx_ptr++] = ((Error >> 12) & 0x0F) | 0x30;		// Error 대분류
	Spib_Buf_To_Main[spi_tx_ptr++] = ((Error >>  8) & 0x0F) | 0x30;		// Error 중분류
	Spib_Buf_To_Main[spi_tx_ptr++] = ((Error >>  4) & 0x0F) | 0x30;		// Error 소분류 0
 	Spib_Buf_To_Main[spi_tx_ptr++] = ((Error >>  0) & 0x0F) | 0x30;		// Error 소분류 1

 	Spib_Buf_To_Main[spi_tx_ptr++] = pCmd->TxData.u8DataID;

 	for(j = 0; j < (size - 1); j++)
 	{
 		Spib_Buf_To_Main[spi_tx_ptr + j] = pCmd->TxData.u8Data[j];
 	}

 	Spib_Buf_To_Main[spi_tx_ptr + j] = ETX;

 	u32Length = spi_tx_ptr + j + 1;
 	Spib_Buf_To_Main[2] = ((u32Length >> 12) & 0x0F) | 0x30;			// LEN0
 	Spib_Buf_To_Main[3] = ((u32Length >>  8) & 0x0F) | 0x30;			// LEN1
 	Spib_Buf_To_Main[4] = ((u32Length >>  4) & 0x0F) | 0x30;			// LEN2
 	Spib_Buf_To_Main[5] = ((u32Length >>  0) & 0x0F) | 0x30;			// LEN3

 	for(j = 2, bcc = 0x00; j < u32Length -1 ; j++)
 	{
 		bcc ^= Spib_Buf_To_Main[j];
 	}

 	pCmd->u8BCC = Spib_Buf_To_Main[1] = bcc;

}

// SPI Protocol Test 20181031
/****************************************/
/*		Command Parsing Function		*/
/* Author: Gogume						*/
/* History: First Released 20181203		*/
/****************************************/
void DSP_COMMAND_SPI_TASK(HW_RX_OBJ *pCmd, HW_TX_OBJ *pCmd2)
{
	pCmd2->u8Command[0] = pCmd->u8Command[0];
	pCmd2->u8Command[1] = pCmd->u8Command[1];

	switch (pCmd->u8Command[0])
	{
		case CMD_EXECUTE_APPLICATION: 	//'A'
			// app jump
		break;
		case CMD_EXECUTE_DOWNLOAD: 		// 'D'
			// boot jump
		break;
		case CMD_GET_INFO: 				// 'G'
			if (pCmd->u8Command[1] == '0')						// Status Information
			{
				Get_Spi_Status(pCmd);
				u16SpiTXsize = __LEN_G0;
				DSPStatusData(&g_SpiArmStatus);
				Send_Spi_IdF0(pCmd2, u16SpiTXsize);
			}
			if (pCmd->u8Command[1] == '1') { }					// Version Information
			if (pCmd->u8Command[1] == '2') { }					// Verification Information
			if (pCmd->u8Command[1] == '3') { }					// Device Information
			if (pCmd->u8Command[1] == '4') { }					// Inverter Information
		break;
		case CMD_INSPECT_DEVICE: 		// 'I'
			if (pCmd->u8Command[1] == '0') { }					// Initialization of Motor Data
			if (pCmd->u8Command[1] == '1') { }					// Initialization of Inverter Data
			if (pCmd->u8Command[1] == '2') { }					// Adjustment of Sensors
			if (pCmd->u8Command[1] == '3') { }					// DSP Reset
			if (pCmd->u8Command[1] == '4') { }					// FRAM Initialization
		break;
		case CMD_SET_INFO: 				// 'S'
			if (pCmd->u8Command[1] == '0') { }					// Read LOG
			if (pCmd->u8Command[1] == '1') { }					// Write LOG
			if (pCmd->u8Command[1] == '2') { }					// Clear LOG
		break;
		case CMD_VERIFICATION: 			// 'V'
			if (pCmd->u8Command[1] == '1') { }					// Verification Ready
			if (pCmd->u8Command[1] == '2') { }					// Verification Start
			if (pCmd->u8Command[1] == '3') { }					// Verification Restart
			if (pCmd->u8Command[1] == '4') { }					// Verification Stop
			if (pCmd->u8Command[1] == '5') { }					// Verification End
		break;
		case CMD_TEST:					// 'T' For ARM Test
			if(pCmd->u8Command[1] == '1') { }
			if(pCmd->u8Command[1] == '2') { }
		break;
		default:

			break;
	}

	pCmd2->u8Length[0] = ((u16SpiTXsize >> 12) & 0x0F) | 0x30;
	pCmd2->u8Length[1] = ((u16SpiTXsize >>  8) & 0x0F) | 0x30;
	pCmd2->u8Length[2] = ((u16SpiTXsize >>  4) & 0x0F) | 0x30;
	pCmd2->u8Length[3] = ((u16SpiTXsize >>  0) & 0x0F) | 0x30;

	pCmd->u8IsCommand = FALSE;
}

/****************************************/
/*		Send IdF0 Spi Data  Function	*/
/* Author: Gogume						*/
/* History: First Released 20181203		*/
/****************************************/
void Send_Spi_IdF0(HW_TX_OBJ *pCmd, u16 u16size)
{
	u8 i;
	DefErr ErrCode = 0;
	u8 Index = 0;

	TestSpiPacket++;

//	pCmd->TxData.u8Data[Index++] = g_IdCoData.u8DataID;
	pCmd->TxData.u8DataID = 0xF0;
//	pCmd->TxData.u8DataID = TestSpiPacket;
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

	/* Insert Data to TX Buffer */
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8DecelFlg & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8ChimeFlg & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8VoiceFlg & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8RunOpenFlg & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8SpdPtnFlg & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8FlFindDir & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8FlInitOK & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8LMotAngleFind & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8InvErr & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8NearFlStop & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8NoCall & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8PosErr & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8FlInitWng & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8DoorZoneExceed & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8DoorZoneLack & 0xff);;
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8ElAntistall & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8InvWatchdog & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CallErr & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8ALDOp & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8ForceDecelOp & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8LowSpdOp & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8McucommuChk & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8MaxFloor[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8MaxFloor[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8MaxFloor[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8DecelFloor[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8DecelFloor[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8DecelFloor[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentFloor[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentFloor[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentFloor[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Dummy[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Dummy[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Dummy[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentSpeed[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentSpeed[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentSpeed[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentPos[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentPos[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentPos[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentPos[3] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentPos[4] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentPos[5] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentPos[6] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8CurrentPos[7] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8EL_Speed[0]  & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8EL_Speed[1]  & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8EL_Speed[2]  & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8INV_ErrCode & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8S_curveTime[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8S_curveTime[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8PTN_Accel[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8PTN_Accel[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8PTN_Accel[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse[3] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse[4] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse[5] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse[3] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse[4] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse[5] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse_Ref[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse_Ref[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse_Ref[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse_Ref[3] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse_Ref[4] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Iqse_Ref[5] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse_Ref[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse_Ref[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse_Ref[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse_Ref[3] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse_Ref[4] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Idse_Ref[5] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_DC_Link[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_DC_Link[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_DC_Link[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_DC_Link[3] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_DC_Link[4] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_DC_Link[5] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Vll[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Vll[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Vll[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Vll[3] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Vll[4] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Vll[5] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Speed_Ref[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Speed_Ref[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Speed_Ref[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Thrust[0] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Thrust[1] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Thrust[2] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Thrust[3] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Thrust[4] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8Inv_Thrust[5] & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8BK_status1 & 0xff);
	pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8BK_status2 & 0xff);

	for(i = 0; i < 27; i++)
    {
        pCmd->TxData.u8Data[Index++] = (g_SpiArmStatus.u8reserved[i] & 0xff);
    }

	/* End */

	ErrCode = GenErrorCode(g_ErrorCode);

	INV_To_Main_Spi_Msg_Pkt(pCmd, u16size, ErrCode);
}

/****************************************/
/*		Get Spi Status Function			*/
/* Author: Gogume						*/
/* History: First Released 20181220		*/
/****************************************/
void Get_Spi_Status(HW_RX_OBJ *pCmd)
{
	u8 i;
	u8 u8RecievedData[72];

	for(i = 0; i < sizeof(g_IdSpi41Data); i++)
	{
		u8RecievedData[i] = 0x30;
	}
	Spi_Cmd_Parsing(pCmd, u8RecievedData, 72);

	Id41_Spi_Data_Parsing(u8RecievedData);
}

/****************************************/
/*		[41] Data Parsing Function		*/
/* Author: Gogume						*/
/* History: First Released 20181220		*/
/****************************************/
void Id41_Spi_Data_Parsing(u8 *pData)
{
	u8 i;
	i = 0;
	u32 SetPosition_d1 = 0;

	g_SpiDspStatus.u8RunMode 			= pData[24+i++];
	g_SpiDspStatus.u8RunStatus			= pData[24+i++];
	g_SpiDspStatus.u8LoadStatus			= pData[24+i++];
	g_SpiDspStatus.S_POS.x				= pData[24+i++];
	g_SpiDspStatus.S_POS.y				= pData[24+i++];
	g_SpiDspStatus.S_POS.z				= pData[24+i++];
	g_SpiDspStatus.N_POS.x				= pData[24+i++];
	g_SpiDspStatus.N_POS.y				= pData[24+i++];
	g_SpiDspStatus.N_POS.z				= pData[24+i++];
	g_SpiDspStatus.E_POS.x				= pData[24+i++];
	g_SpiDspStatus.E_POS.y				= pData[24+i++];
	g_SpiDspStatus.E_POS.z				= pData[24+i++];
	g_SpiDspStatus.u8SetPosition[0]		= pData[24+i++];
	g_SpiDspStatus.u8SetPosition[1]		= pData[24+i++];
	g_SpiDspStatus.u8SetPosition[2]		= pData[24+i++];
	g_SpiDspStatus.u8SetPosition[3]		= pData[24+i++];
	g_SpiDspStatus.u8RemainPosition[0]	= pData[24+i++];
	g_SpiDspStatus.u8RemainPosition[1]	= pData[24+i++];
	g_SpiDspStatus.u8RemainPosition[2]	= pData[24+i++];
	g_SpiDspStatus.u8RemainPosition[3]	= pData[24+i++];
	g_SpiDspStatus.u8DoorStatus			= pData[24+i++];
	g_SpiDspStatus.u8CommStatus			= pData[24+i++];
	g_SpiDspStatus.u8StopAbleFloor		= pData[24+i++];
	g_SpiDspStatus.u8LoadValue  		= pData[24+i++];
	g_SpiDspStatus.u8EncoderInfo		= pData[24+i++];
	g_SpiDspStatus.u8APSRegion  		= pData[24+i++];
	g_SpiDspStatus.u8APS1PositionSign	= pData[24+i++];
	g_SpiDspStatus.APS1Position[0]		= pData[24+i++];
	g_SpiDspStatus.APS1Position[1]		= pData[24+i++];
	g_SpiDspStatus.APS1Position[2]		= pData[24+i++];
	g_SpiDspStatus.APS1Position[3]		= pData[24+i++];
	g_SpiDspStatus.u8APS2PositionSign	= pData[24+i++];
	g_SpiDspStatus.APS2Position[0]		= pData[24+i++];
	g_SpiDspStatus.APS2Position[1]		= pData[24+i++];
	g_SpiDspStatus.APS2Position[2]		= pData[24+i++];
	g_SpiDspStatus.APS2Position[3]		= pData[24+i++];
	g_SpiDspStatus.APS1Velocity[0] 		= pData[24+i++];
	g_SpiDspStatus.APS1Velocity[1] 		= pData[24+i++];
	g_SpiDspStatus.APS2Velocity[0] 		= pData[24+i++];
	g_SpiDspStatus.APS2Velocity[1] 		= pData[24+i++];
	g_SpiDspStatus.LaserSensor1[0] 		= pData[24+i++];
	g_SpiDspStatus.LaserSensor1[1] 		= pData[24+i++];
	g_SpiDspStatus.LaserSensor1[2]      = pData[24+i++];
	g_SpiDspStatus.LaserSensor1[3]      = pData[24+i++];
	g_SpiDspStatus.LaserSensor2[0] 		= pData[24+i++];
	g_SpiDspStatus.LaserSensor2[1] 		= pData[24+i++];
	g_SpiDspStatus.LaserSensor2[2]      = pData[24+i++];
	g_SpiDspStatus.LaserSensor2[3]      = pData[24+i++];
	g_SpiDspStatus.u8BrakeStatus		= pData[24+i++];
	g_SpiDspStatus.InitOpStatus			= pData[24+i++];

	// Set Position Transformation
	g_SpiDspStatus.SetPosition			 = (u32)(g_SpiDspStatus.u8SetPosition[0]) << 24;
	g_SpiDspStatus.SetPosition			+= (u32)(g_SpiDspStatus.u8SetPosition[1]) << 16;
	g_SpiDspStatus.SetPosition			+= (u32)(g_SpiDspStatus.u8SetPosition[2]) << 8;
	g_SpiDspStatus.SetPosition			+= (u32)(g_SpiDspStatus.u8SetPosition[3]) << 0;

	if(g_SpiDspStatus.SetPosition != SetPosition_d1){
	    Spi_ring_buffer_push(g_SpiDspStatus.SetPosition);
	}
	SetPosition_d1 = g_SpiDspStatus.SetPosition;

	// Current Position Transformation(from APS1)
	g_SpiDspStatus.CurrentPosition1		 = (u32)(g_SpiDspStatus.APS1Position[0]) << 24;
	g_SpiDspStatus.CurrentPosition1		+= (u32)(g_SpiDspStatus.APS1Position[1]) << 16;
	g_SpiDspStatus.CurrentPosition1		+= (u32)(g_SpiDspStatus.APS1Position[2]) << 8;
	g_SpiDspStatus.CurrentPosition1		+= (u32)(g_SpiDspStatus.APS1Position[3]) << 0;

    // Current Position Transformation(from APS2)
    g_SpiDspStatus.CurrentPosition2      = (u32)(g_SpiDspStatus.APS2Position[0]) << 24;
    g_SpiDspStatus.CurrentPosition2     += (u32)(g_SpiDspStatus.APS2Position[1]) << 16;
    g_SpiDspStatus.CurrentPosition2     += (u32)(g_SpiDspStatus.APS2Position[2]) << 8;
    g_SpiDspStatus.CurrentPosition2     += (u32)(g_SpiDspStatus.APS2Position[3]) << 0;

    // Current Velocity Transformation(from APS1)
    g_SpiDspStatus.CurrentVelocity1      = (u32)(g_SpiDspStatus.APS1Velocity[0]) << 8;
    g_SpiDspStatus.CurrentVelocity1     += (u32)(g_SpiDspStatus.APS1Velocity[1]) << 0;

    // Current Velocity Transformation(from APS2)
    g_SpiDspStatus.CurrentVelocity2      = (u32)(g_SpiDspStatus.APS2Velocity[0]) << 8;
    g_SpiDspStatus.CurrentVelocity2     += (u32)(g_SpiDspStatus.APS2Velocity[1]) << 0;

	// Distance between a car and a car(from Laser Sensor1)
	g_SpiDspStatus.LaserDistanceUp       = (u32)(g_SpiDspStatus.LaserSensor1[0] - 0x30)  << 12;
	g_SpiDspStatus.LaserDistanceUp      += (u32)(g_SpiDspStatus.LaserSensor1[1] - 0x30)  << 8;
	g_SpiDspStatus.LaserDistanceUp      += (u32)(g_SpiDspStatus.LaserSensor1[2] - 0x30)  << 4;
	g_SpiDspStatus.LaserDistanceUp      += (u32)(g_SpiDspStatus.LaserSensor1[3] - 0x30)  << 0;

    // Distance between a car and a car(from Laser Sensor2)
    g_SpiDspStatus.LaserDistanceDown     = (u32)(g_SpiDspStatus.LaserSensor2[0] - 0x30)  << 12;
    g_SpiDspStatus.LaserDistanceDown    += (u32)(g_SpiDspStatus.LaserSensor2[1] - 0x30)  << 8;
    g_SpiDspStatus.LaserDistanceDown    += (u32)(g_SpiDspStatus.LaserSensor2[2] - 0x30)  << 4;
    g_SpiDspStatus.LaserDistanceDown    += (u32)(g_SpiDspStatus.LaserSensor2[3] - 0x30)  << 0;
}

/****************************************/
/*	Command Data Parsing Function		*/
/* Author: Gogume						*/
/* History: First Released 20181220		*/
/****************************************/
void Spi_Cmd_Parsing(HW_RX_OBJ *pCmd, u8 *pData, u8 size)
{
	u8 i;

	for(i = 0; i < size; i++)
	{
		pData[i] = pCmd->RxData.u8Data[i];
	}

	g_IdSpiCoData.u8DataID 			= pData[0];
	g_IdSpiCoData.u8DataLength[0] 	= pData[1];
	g_IdSpiCoData.u8DataLength[1] 	= pData[2];
	g_IdSpiCoData.u8DataLength[2] 	= pData[3];
	g_IdSpiCoData.u8DataLength[3] 	= pData[4];
	g_IdSpiCoData.rspDate.u8Year 	= pData[5];
	g_IdSpiCoData.rspDate.u8Month 	= pData[6];
	g_IdSpiCoData.rspDate.u8Date 	= pData[7];
	g_IdSpiCoData.rspTime.u8Hour 	= pData[8];
	g_IdSpiCoData.rspTime.u8Minute 	= pData[9];
	g_IdSpiCoData.rspTime.u8Second 	= pData[10];
	g_IdSpiCoData.u32TransNum 	    = (pData[11] & 0x0F) << 12;
	g_IdSpiCoData.u32TransNum 	   += (pData[12] & 0x0F) << 8;
	g_IdSpiCoData.u32TransNum 	   += (pData[13] & 0x0F) << 4;
	g_IdSpiCoData.u32TransNum 	   += (pData[14] & 0x0F) << 0;
	g_IdSpiCoData.u8CarSpeed 		= pData[15];
	g_IdSpiCoData.u8CarType 		= pData[16];
	g_IdSpiCoData.u8CarNumber 		= pData[17];
	g_IdSpiCoData.u8CarPosition 	= pData[18];
	g_IdSpiCoData.u8LimitSensorMode	= pData[19];
	g_IdSpiCoData.u8SPIMode 		= pData[20];
	g_IdSpiCoData.u8APSMode 		= pData[21];
	g_IdSpiCoData.u8LinearScaleMode	= pData[22];
	g_IdSpiCoData.u8Reserved 		= pData[23];
}

/****************************************/
/*	Initialization SPIB Packet Function	*/
/* Author: Gogume						*/
/* History: First Released 20181221		*/
/****************************************/
void Init_Spib_Pkt(HW_TX_OBJ *pCmd)
{
	Send_Spi_IdF0(pCmd, 128);

	SpibRegs.SPIFFTX.bit.TXFIFO = 0;
	asm(" NOP");
	asm(" NOP");
	SpibRegs.SPIFFTX.bit.TXFIFO = 1;
}

/****************************************/
/*Inverter Data Transformation Function */
/* Author: Gogume                       */
/* History: First Released 20190215     */
/****************************************/
void DSPStatusData(STATUS_ARM_OBJ *pArmObj)
{
    /* Current Speed SPI Data */
    u32Wrm = (u32)(fabs(INV1_SC.Wrm)*100);
    pArmObj->u8CurrentSpeed[0] = (u32Wrm / 100) | 0x30;
    pArmObj->u8CurrentSpeed[1] = ((u32Wrm % 100) /  10) | 0x30;
    pArmObj->u8CurrentSpeed[2] = ((u32Wrm % 10) /   1) | 0x30;

    /* Speed Reference SPI Data */
    u32Wrm_Ref = (u32)(fabs(INV1_SC.Wrm_ref*100));
    if(INV1_SC.Wrm_ref >= 0){                       // Sign bit: +(0x30), -(0x31)
        pArmObj->u8Inv_Speed_Ref[0] = 0x30;
    }
    else pArmObj->u8Inv_Speed_Ref[0] = 0x31;
    pArmObj->u8Inv_Speed_Ref[1] = (u32Wrm_Ref / 100) | 0x30;
    pArmObj->u8Inv_Speed_Ref[2] = ((u32Wrm_Ref % 100) /  10) | 0x30;
    pArmObj->u8Inv_Speed_Ref[3] = ((u32Wrm_Ref % 10) /   1) | 0x30;

    /* d, q axis Current SPI Data */
    u32Ide = (u32)(fabs(INV1.Idse*100));
    if(INV1.Idse >= 0){                         // Sign bit: +(0x30), -(0x31)
        pArmObj->u8Inv_Idse[0] = 0x30;
    }
    else pArmObj->u8Inv_Idse[0] = 0x31;
    pArmObj->u8Inv_Idse[1] = (u32Ide / 10000) | 0x30;
    pArmObj->u8Inv_Idse[2] = ((u32Ide % 10000) /  1000) | 0x30;
    pArmObj->u8Inv_Idse[3] = ((u32Ide % 1000) /   100) | 0x30;
    pArmObj->u8Inv_Idse[4] = ((u32Ide % 100) /    10) | 0x30;
    pArmObj->u8Inv_Idse[5] = ((u32Ide % 10) /     1) | 0x30;

    u32Iqe = (u32)(fabs(INV1.Iqse*100));
    if(INV1.Iqse >= 0){                         // Sign bit: +(0x30), -(0x31)
        pArmObj->u8Inv_Iqse[0] = 0x30;
    }
    else g_SpiArmStatus.u8Inv_Iqse[0] = 0x31;
    pArmObj->u8Inv_Iqse[1] = (u32Iqe / 10000) | 0x30;
    pArmObj->u8Inv_Iqse[2] = ((u32Iqe % 10000) /  1000) | 0x30;
    pArmObj->u8Inv_Iqse[3] = ((u32Iqe % 1000) /   100) | 0x30;
    pArmObj->u8Inv_Iqse[4] = ((u32Iqe % 100) /    10) | 0x30;
    pArmObj->u8Inv_Iqse[5] = ((u32Iqe % 10) /     1) | 0x30;

    /* d, q axis Current Reference SPI Data */
    u32Ide_Ref = (u32)(fabs(INV1_CC.Idse_Ref*100));
    if(INV1_CC.Idse_Ref >= 0){                 // Sign bit: +(0x30), -(0x31)
        pArmObj->u8Inv_Idse_Ref[0] = 0x30;
    }
    else pArmObj->u8Inv_Idse_Ref[0] = 0x31;
    pArmObj->u8Inv_Idse_Ref[1] = (u32Ide_Ref / 10000) | 0x30;
    pArmObj->u8Inv_Idse_Ref[2] = ((u32Ide_Ref % 10000) /  1000) | 0x30;
    pArmObj->u8Inv_Idse_Ref[3] = ((u32Ide_Ref % 1000) /   100) | 0x30;
    pArmObj->u8Inv_Idse_Ref[4] = ((u32Ide_Ref % 100) /    10) | 0x30;
    pArmObj->u8Inv_Idse_Ref[5] = ((u32Ide_Ref % 10) /     1) | 0x30;

    u32Iqe_Ref = (u32)(fabs(INV1_CC.Iqse_Ref*100));
    if(INV1_CC.Iqse_Ref >= 0){                  // Sign bit: +(0x30), -(0x31)
        pArmObj->u8Inv_Iqse_Ref[0] = 0x30;
    }
    else pArmObj->u8Inv_Iqse_Ref[0] = 0x31;
    pArmObj->u8Inv_Iqse_Ref[1] = (u32Iqe_Ref / 10000) | 0x30;
    pArmObj->u8Inv_Iqse_Ref[2] = ((u32Iqe_Ref % 10000) /  1000) | 0x30;
    pArmObj->u8Inv_Iqse_Ref[3] = ((u32Iqe_Ref % 1000) /   100) | 0x30;
    pArmObj->u8Inv_Iqse_Ref[4] = ((u32Iqe_Ref % 100) /    10) | 0x30;
    pArmObj->u8Inv_Iqse_Ref[5] = ((u32Iqe_Ref % 10) /     1) | 0x30;

    /* Voltage Reference magnitude SPI Data */
    u32V_ref_mag = (u32)(fabs(V_ref_mag)*100);
    if(V_ref_mag >= 0){                         // Sign bit: +(0x30), -(0x31)
        pArmObj->u8Inv_Vll[0] = 0x30;
    }
    else pArmObj->u8Inv_Vll[0] = 0x31;
    pArmObj->u8Inv_Vll[1] = (u32V_ref_mag / 10000) | 0x30;
    pArmObj->u8Inv_Vll[2] = ((u32V_ref_mag % 10000)/  1000) | 0x30;
    pArmObj->u8Inv_Vll[3] = ((u32V_ref_mag % 1000)/   100) | 0x30;
    pArmObj->u8Inv_Vll[4] = ((u32V_ref_mag % 100)/    10) | 0x30;
    pArmObj->u8Inv_Vll[5] = ((u32V_ref_mag % 10)/     1) | 0x30;

    /* Trust Force SPI Data */
    u32Te = (u32)(fabs(INV1_SC.Te_real)*10);
    if(INV1_SC.Te_real >= 0){                   // Sign bit: +(0x30), -(0x31)
        pArmObj->u8Inv_Thrust[0] = 0x30;
    }
    else pArmObj->u8Inv_Thrust[0] = 0x31;
    pArmObj->u8Inv_Thrust[1] = (u32Te / 100000) | 0x30;
    pArmObj->u8Inv_Thrust[2] = ((u32Te % 100000) /  10000) | 0x30;
    pArmObj->u8Inv_Thrust[3] = ((u32Te % 10000) /   1000) | 0x30;
    pArmObj->u8Inv_Thrust[4] = ((u32Te % 1000) /    100) | 0x30;
    pArmObj->u8Inv_Thrust[5] = ((u32Te % 100) /     10) | 0x30;
    pArmObj->u8Inv_Thrust[6] = ((u32Te % 10) /     1) | 0x30;

    /* Current Position SPI Data */
    u32CurrentPos = (u32)(fabs(INV1_PC.Pos*10000));     // m --> mm
    if(INV1_PC.Pos >= 0){                       // Sign bit: +(0x30), -(0x31)
        pArmObj->u8CurrentPos[0] = 0x30;
    }
    else pArmObj->u8CurrentPos[0] = 0x31;
    pArmObj->u8CurrentPos[1] = (u32CurrentPos / 1000000) | 0x30;
    pArmObj->u8CurrentPos[2] = ((u32CurrentPos % 1000000)/  100000) | 0x30;
    pArmObj->u8CurrentPos[3] = ((u32CurrentPos % 100000)/   10000) | 0x30;
    pArmObj->u8CurrentPos[4] = ((u32CurrentPos % 10000)/    1000) | 0x30;
    pArmObj->u8CurrentPos[5] = ((u32CurrentPos % 1000)/     100) | 0x30;
    pArmObj->u8CurrentPos[6] = ((u32CurrentPos % 100)/      10) | 0x30;
    pArmObj->u8CurrentPos[7] = ((u32CurrentPos % 10)/       1) | 0x30;

    /* DC-Link Voltage SPI Data */
    u32Vdc = (u32)(fabs(Vdc_rd)*100);
    pArmObj->u8Inv_DC_Link[0] = (u32Vdc / 100000) | 0x30;
    pArmObj->u8Inv_DC_Link[1] = ((u32Vdc % 100000) /  10000) | 0x30;
    pArmObj->u8Inv_DC_Link[2] = ((u32Vdc % 10000) /   1000) | 0x30;
    pArmObj->u8Inv_DC_Link[3] = ((u32Vdc % 1000) /    100) | 0x30;
    pArmObj->u8Inv_DC_Link[4] = ((u32Vdc % 100) /     10) | 0x30;
    pArmObj->u8Inv_DC_Link[5] = ((u32Vdc % 10) /      1) | 0x30;
}

/****************************************/
/*      SPI Ring Buffer Init Function   */
/* Author: Gogume                       */
/* History: First Released 20190227     */
/****************************************/
void Spi_ring_buffer_init(void)
{
    u32spi_ring_buf_lp = 0;
    u32spi_ring_buf_cp = 0;
    u32spi_ring_buf_len = 0;
}
/****************************************/
/*    SPI Ring Buffer Length Function   */
/* Author: Gogume                       */
/* History: First Released 20190227     */
/****************************************/
u32 Spi_ring_buffer_length(void)
{
    return u32spi_ring_buf_len;
}
/****************************************/
/*    SPI Ring Buffer Push Function     */
/* Author: Gogume                       */
/* History: First Released 20190227     */
/****************************************/
void Spi_ring_buffer_push(u32 data)
{
    spi_ring_buf[u32spi_ring_buf_lp++] = data;
    u32spi_ring_buf_len++;

    if (u32spi_ring_buf_len > SPI_RING_BUF_SIZE){
        u32spi_ring_buf_cp++;
        if (u32spi_ring_buf_cp == SPI_RING_BUF_SIZE){
            u32spi_ring_buf_cp = 0;
        }
        u32spi_ring_buf_len = SPI_RING_BUF_SIZE;
    }

    if (u32spi_ring_buf_lp == SPI_RING_BUF_SIZE){
        u32spi_ring_buf_lp = 0;
    }
}
/****************************************/
/*    SPI Ring Buffer Pop Function      */
/* Author: Gogume                       */
/* History: First Released 20190227     */
/****************************************/
u32 Spi_ring_buffer_pop(void)
{
    u32 ret = 0;

    if (u32spi_ring_buf_len > 0){
        if (u32spi_ring_buf_cp == SPI_RING_BUF_SIZE){
            u32spi_ring_buf_cp = 0;
        }
        ret = spi_ring_buf[u32spi_ring_buf_cp++];
        u32spi_ring_buf_len--;
    }
    return ret;
}
