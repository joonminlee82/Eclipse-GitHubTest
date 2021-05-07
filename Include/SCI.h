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
                        : Added Profile Error Compansator (20180510 by GWON)
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

#ifndef _SCI_
#define _SCI_

#include "variable.h"
#include "gnl.h"

#define SCIC_POLLING				1

#define	CPUCLK				200000000L
#define BAUDRATE_B          9600L
#define	BAUDRATE_C			115200L
#define	BAUDRATE_D			115200L
#define	LSP_CLOCK			(CPUCLK/4)
#define BRR_VAL_B           (LSP_CLOCK/(8*BAUDRATE_B)-1)    // BaudRate 설정 Register 값
#define	BRR_VAL_C			(LSP_CLOCK/(8*BAUDRATE_C)-1)    // BaudRate 설정 Register 값
#define	BRR_VAL_D			(LSP_CLOCK/(8*BAUDRATE_D)-1)    // BaudRate 설정 Register 값

//#define	SCIB_BUF_SIZE	100

/*
=========================================================================================================
*                                               UART interface for MAIN PACKET FORMAT
*                   (A)  4 1-byte start delimiters, forming the ASCII representation of "WBTX" or "WBRX".
*                   (B)  1 1-byte length, the length of the data segment
*                   (C)  n bytes of data
*                   (D)  2 2-byte CRC
*                   (E)  1 1-byte end delimiter, the character '/'.
*                                       +-----------------+
*                                       | 'I' | 'N' | 'V' |
*                                       +-----------------+
*                                       |Length:1bytes|
*                                       +----------------------------------------------------------+
*                                       |  Data ...................................................|
*                                       +----------------------------------------------------------+
*                                       +-------------------+---------+ 
*                                       |       16-CRC      |   '/'   |
*                                       +-------------------+---------+
=========================================================================================================
*/
/*----------------------------------[INVERTER -> MAIN]-----------------------------*/
#define     TX_PROTOCOL_SD0	'I'
#define     TX_PROTOCOL_SD1	'N'
#define     TX_PROTOCOL_SD2	'V'

/*----------------------------------[MAIN <- INVERTER]-----------------------------*/
#define     RX_PROTOCOL_SD0	'D'
#define     RX_PROTOCOL_SD1	'R'
#define     RX_PROTOCOL_SD2	'V'

#define     PROTOCOL_ED		0x2F            // '/'

#define 	MAX_DATA_LENGTH				1024
#define     TX_INDEX              		MAX_DATA_LENGTH
#define     RX_INDEX              		MAX_DATA_LENGTH
#define     RX_PACKET_FORMAT_LENGTH		(1+1+3+2+1)       // STX(1) + BCC(1) + LEN(3) + COMM(2) + ETX(1)
#define     TX_PACKET_FORMAT_LENGTH		(1+1+4+2+1+4+1)   // STX(1) + BCC(1) + LEN(4) + COMM(2)+ END_STATUS(1)+ ERROR_CODE(4) + ETX(1)

#define     TO_MAIN_PACKET_LENGTH       (TX_PACKET_FORMAT_LENGTH+TX_INDEX)//27
#define     FROM_MAIN_PACKET_LENGTH     (RX_PACKET_FORMAT_LENGTH+RX_INDEX)//9+128

#define		COM_FLT_LMT		(FROM_MAIN_PACKET_LENGTH*5)

#define     __LEN_G0    128
#define     __LEN_G1    25
#define     __LEN_I0    104
#define     __LEN_I1    256
#define     __LEN_I4    24
#define     __LEN_I5    104
#define     __LEN_I6    256
#define     __LEN_I7    25
#define     __LEN_I8    37
#define     __LEN_I9    92
#define     __LEN_IA    116
#define     __LEN_IB    36
#define     __LEN_S1    594

#define     __NORMAL_MODE       0x30
#define     __OUTPUT_DEBUG_MODE 0x31

#define     __VERTICAL      0x31
#define     __HORIZONTAL    0x32

extern __interrupt void cpu1_Scib_Rx(void);
extern __interrupt void cpu1_Scic_Rx(void);
extern __interrupt void cpu1_Scic_Tx(void);
extern __interrupt void cpu1_Scid_Rx(void);
extern __interrupt void cpu1_Scid_Tx(void);

extern Byte Scic_Buf_From_Main[FROM_MAIN_PACKET_LENGTH] ;
extern Byte Scic_Buf_To_Main[TO_MAIN_PACKET_LENGTH] ;

extern u8 g_Mode_Check;
/**************************************************************************/
/* STRUCTURE DEFINITION FOR INPUT FLAG			                  */
/**************************************************************************/
typedef union
{
	Byte	regStatus;
	struct
	{
		Byte autoop_flag			:1;		/* 자동운전 */
		Byte matnop_flag			:1;		/* 수동운전 */
		Byte retrnop_flag			:1;		/* 복귀운전 및 화재운전 */
		Byte flfndop_flag			:1;		/* 최기층운전, 층복귀운전*/
		Byte relvlop_flag			:1;		/* relevel 운전 */
		Byte initop_flag			:1;		/* 초기화 운전 */
		Byte eldop_flag				:1;		/* eld 운전 */
		Byte rescop_flag			:1;		/* 구출운전 */
	} regStatusBit;
} SCI_In1;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte rdycfm					:1;		/* 주행운전확인 */
		Byte upcfm					:1;		/* UP 주행확인 */
		Byte dncfm					:1;		/* DN 주행확인 */
		Byte hispd					:1;		/* 고속운전가능*/
		Byte matdecl				:1;		/* 수동운전감속*/
		Byte fwdrev					:1;		/* 주행준비(UP=1,DOWN=0)*/
		Byte mwtchdg				:1;		/*제어반 watchdog */
		Byte NonStopFlag			:1;		/*Non stop 정보 0:service, 1:Non stop */
	} regStatusBit;
} SCI_In2;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte LD30					:1;		/* 카내 30% 부하 스위치 데이터*/
		Byte LD70					:1;		/* 카내 70% 부하 스위치 데이터*/
		Byte LD100					:1;		/* 카내 100% 부하 스위치 데이터*/
		Byte LD110					:1;		/* 카내 110% 부하 스위치 데이터*/
		Byte LowSpeed				:1;		/* 최상,하층 확인 신호 찾기 운전*/
		Byte dooropen				:1;		/* 도어 상태 확인(1:open, 0:close) - current offset reset용*/
		Byte InvcommuChk			:1;		/* Inverter 통신 상태 확인용(1:OK, 0:Fail)*/
		Byte rsved					:1;
	} regStatusBit;
} SCI_In3;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte	CALLRD					:8;		/* 정지 가능 층*/
	} regStatusBit;
} SCI_In4;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte	YEAR					:8;		/* YEAR*/
	} regStatusBit;
} SCI_In5;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte	MONTH					:8;		/* MONTH*/
	} regStatusBit;
} SCI_In6;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte	DAY						:8;		/* DAY*/
	} regStatusBit;
} SCI_In7;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte	HOUR					:8;		/* HOUR*/
	} regStatusBit;
} SCI_In8;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte	MINUTE					:8;		/* MINUTE*/
	} regStatusBit;
} SCI_In9;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte	SECOND					:8;		/* SECOND*/
	} regStatusBit;
} SCI_In10;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte	Load					:8;		/* LS 보드 부하량*/
	} regStatusBit;
} SCI_In11;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte Load891011bit				:4;		/* LS 보드 부하량*/
		Byte BKwiring					:1;		/* Brake switch wiring(1:parallel, 0:serial) */
		Byte HHTMode					:1;		/* HHT attribute(1:Limit Menu, 0:Full Menu)  */
		Byte rsved						:2;
	} regStatusBit;
} SCI_In12;
/**************************************************************************/
/* STRUCTURE DEFINITION FOR OUTPUT FLAG			                  */
/**************************************************************************/
typedef union
{
	Byte	regStatus;
	struct
	{
		Byte decel1						:1;		/* 감속중 */
		Byte decel2						:1;		/* chime 신호 */
		Byte decel3						:1;		/* 음성 신호용 감속신호 */
		Byte runopn						:1;		/* Running Open 가능신호 */
		Byte ptnout						:1;		/* 속도 패턴유무 */
		Byte flrdir						:1;		/* 최기능 운전 방향 */
		Byte initok						:1;		/* initial 운전 확인 */
		Byte angldct					:1;		/* 동기모터 각 측정운전 */
	} regStatusBit;
} SCI_Out1 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte inver						:1;		/* 인버터 고장 */
		Byte nearflstop					:1;		/* 최기층 정지 요청 *///x
		Byte nocall						:1;		/* Call 무 */
		Byte posier						:1;		/* 위치이상*/
		Byte initwng					:1;		/* 이니셜 운전 이상 *///x
		Byte mxflrup					:1;		/* DZ 초과 */
		Byte mxflrdn					:1;		/* DZ 부족 */
		Byte sdsswer					:1;		/* 강제 감속 스위치 이상 */
	} regStatusBit;
} SCI_Out2 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte sdswng						:1;		/* 강제 감속 스위치 이상(200mm 초과) *///x
		Byte antistal					:1;		/* 안티스톨 *///x
		Byte iwatdg						:1;		/* 인버터 watchdog*/
		Byte dclfler					:1;		/* 감속가능층 입력오류(Call입력에 감속가능층이 아닌 이상입력)*/
		Byte FltDrive					:1;		/* 인버터 고장 운전*/
		Byte SudsDecel					:1;		/* 강제감속 스위치 동작 감속 운전*/
		Byte McucommuChk				:1;		/* MCU 통신 상태 확인용(1:OK, 0:Fail)*/
		Byte rsved						:1;		/* reserved */
	} regStatusBit;
} SCI_Out3 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte MaxFloor					:8;		/* 운전 최대 층수 */
	} regStatusBit;
} SCI_Out4 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte Decflr 					:8;		/*감속 가능 층 */
	} regStatusBit;
} SCI_Out5 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte Crtflr 					:8;		/*현재 층 */
	} regStatusBit;
} SCI_Out6 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte LoadrteL 					:8;		/*부하율 */
	} regStatusBit;
} SCI_Out7 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte CrtspdL 					:8;		/*현재 속도 */
	} regStatusBit;
} SCI_Out8 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte CrtspdH 					:8;		/*현재 속도 */
	} regStatusBit;
} SCI_Out9 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte CrtposiL 					:8;		/*현재 위치 */
	} regStatusBit;
} SCI_Out10 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte CrtposiM 					:8;		/*현재 위치 */
	} regStatusBit;
} SCI_Out11 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte CrtposiH 					:8;		/*현재 위치 */
	} regStatusBit;
} SCI_Out12 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte EL_SpeedL					:8;		/*엘리베이터 정격 최고 속도 */
	} regStatusBit;
} SCI_Out13 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte EL_SpeedH					:8;		/*엘리베이터 정격 최고 속도 */
	} regStatusBit;
} SCI_Out14 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte INV_ErrCode				:8;		/*Inverter Error Code */
	} regStatusBit;
} SCI_Out15 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte S_curveTime				:8;		/*S-curve Time */
	} regStatusBit;
} SCI_Out16 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte PTN_Accel					:8;		/*Pattern Accel Time */
	} regStatusBit;
} SCI_Out17 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte rsved						:8;		/* reserved */
	} regStatusBit;
} SCI_Out18 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte rsved						:8;		/* reserved */
	} regStatusBit;
} SCI_Out19 ;

typedef union
{
	Byte	regStatus;
	struct
	{
		Byte rsved						:8;		/* reserved */
	} regStatusBit;
} SCI_Out20 ;

typedef struct _MCP
{
	Uns			IN_AutoOp;
	Uns 		IN_ManualOp;
	Uns			IN_Return1FOp;
	Uns			IN_FlFindOp;
	Uns			IN_RelevelOp;
	Uns			IN_FlInitOp;
	Uns			IN_EldOp;
	Uns			IN_RescueOp;
	Uns			IN_ReadyCfm;
	Uns			IN_UpCfm;
	Uns			IN_DownCfm;
	Uns			IN_HighSpd;
	Uns			IN_ManualDecel;
	Uns			IN_FwdRev;
	Uns			IN_CpWatchdog;
	Uns     	IN_NonStop;
	Uns			IN_dooropen;
	Uns			IN_InvcommuChk;
	Uns			IN_Call;
	Uns			IN_RfidFloor;
	Uns			IN_RescuePosL;
	Uns			IN_RescuePosM;
	Uns			IN_RescuePosH;
	float   	IN_fRescuePos;
	Uns			IN_Year;
	Uns			IN_Month;
	Uns			IN_Date;
	Uns			IN_Hour;
	Uns			IN_Minute;
	Uns			IN_Second;
	Uns			IN_LS_LoadAD;
	Uns			IN_LowSpdOp;
	Uns			IN_ZeroSpdCtlOp;
	Uns			IN_SpecDrvOp;
	Uns			IN_GerTestOp;
	Uns			IN_LimitSW_TestOp;
	Uns			IN_Buffer_TestOp;
	Uns			IN_AutoPludl_TestOp;
	Uns			IN_En_TestOp;
	Uns			IN_BKwiring;
	Uns			IN_HHTMode;
	Uns			IN_LD30;
	Uns			IN_LD70;
	Uns			IN_LD100;
	Uns			IN_LD110;

	Uns			OUT_DecelFlg;
	Uns			OUT_ChimeFlg;
	Uns			OUT_VoiceFlg;
	Uns			OUT_RunOpenFlg;
	Uns			OUT_SpdPtnFlg;
	Uns			OUT_FlFindDir;
	Uns			OUT_FlInitOK;
	Uns			OUT_SMotAngleFind;
	Uns			OUT_InvErr;
	Uns			OUT_NearFlStop;
	Uns			OUT_NoCall;
	Uns			OUT_PosErr;
	Uns			OUT_FlInitWng;
	Uns			OUT_MaxFlUp;
	Uns			OUT_MaxFlDown;
	Uns			OUT_SdsErr;		// Check 해볼 것 ERR_DECEL_SW1
	Uns			OUT_SdsWng;		// Check 해볼 것 ERR_DECEL_SW1
	Uns			OUT_ElAntistall;
	Uns			OUT_InvWatchdog;
	Uns			OUT_CallErr;
	Uns			OUT_LowSpdOp;
	Uns			OUT_McucommuChk;
	Uns			OUT_MaxFloor;
	Uns			OUT_DecelFloor;
	Uns			OUT_CurrentFloor;
	Uns			OUT_Dummy;
	Uns			OUT_RescuePosCfmL;
	Uns			OUT_RescuePosCfmM;
	Uns			OUT_RescuePosCfmH;
	Uns			OUT_CurrentSpeedL;
	Uns			OUT_CurrentSpeedH;
	Uns			OUT_CurrentPosL;
	Uns			OUT_CurrentPosM;
	Uns			OUT_CurrentPosH;
	Uns			OUT_AutoOpOK;
	Uns 		OUT_ManualOpOK;
	Uns			OUT_Return1FOpOK;
	Uns			OUT_FlFindOpOK;
	Uns			OUT_RelevelOpOK;
	Uns			OUT_FlInitOpOK;
	Uns			OUT_EldOpOK;
	Uns			OUT_RescueOpOK;
	Uns			OUT_ReadyCfmOK;
	Uns			OUT_UpCfmOK;
	Uns			OUT_DownCfmOK;
	Uns			OUT_HighSpdOK;
	Uns			OUT_ManualDecelOK;
	Uns			OUT_GerTestOp_OK;
	Uns			OUT_LimitSW_TestOp_OK;
	Uns			OUT_Buffer_TestOp_OK;
	Uns			OUT_AutoPludl_TestOp_OK;
	Uns			OUT_En_TestOp_OK;
	Uns 		OUT_Inv_ElMode;
	Uns			OUT_TopPosL;
	Uns			OUT_TopPosM;
	Uns			OUT_TopPosH;
	Uns 		Pludl_Test_Flg;
	Uns			OUT_RMB_DecelFlg;
	Uns			OUT_EL_Speed;
	Uns			OUT_LS_perLoad;
	Uns			OUT_INV_ErrCode;
	Uns			OUT_S_curveTime;
	Uns			OUT_PTN_Accel;
	Uns			OUT_rsved18;
	Uns			OUT_rsved19;
	Uns			OUT_rsved20;
	/* for FREEWAY 20180806 */
	int			OUT_Inv_IqseL;
	int			OUT_Inv_IqseH;
	int			OUT_Inv_IdseL;
	int			OUT_Inv_IdseH;
	int			OUT_Inv_Iqse_RefL;
	int			OUT_Inv_Iqse_RefH;
	int			OUT_Inv_Idse_RefL;
	int			OUT_Inv_Idse_RefH;
	int			OUT_Inv_DC_LinkL;
	int			OUT_Inv_DC_LinkH;
	int			OUT_Inv_VllL;
	int			OUT_Inv_VllH;
	int			OUT_Inv_Speed_RefL;
	int			OUT_Inv_Speed_RefH;
	int			OUT_Inv_ThrustL;
	int			OUT_Inv_ThrustH;
	Uns			OUT_BK_status1;
	Uns			OUT_BK_status2;
	Uns			OUT_rsved37_2;
	Uns			OUT_rsved37_3;
	Uns			OUT_rsved37_4;
	Uns			OUT_rsved37_5;
	Uns			OUT_rsved37_6;
	Uns			OUT_rsved37_7;
	Uns			OUT_CAR_id;
	Uns			OUT_rsved39;
	Uns			OUT_rsved40;
}Mcp;

void InitSciB(void);
void InitSciC(void);
void InitSciD(void);
void SCI_Create(void);

int scic_rxdata(void);
void scic_xmit(int a);
void scic_msg(char * msg);
int scid_rxdata(void);
void scid_xmit(int a);
void scid_msg(char * msg);


// SCI Protocol
#define STX				0x02
#define ETX				0x03
#define ACK				0x06
#define NAK				0x15

#define ERROR_COMM_BASE			    0x3000
#define ERROR_COMM_STX              0x3001
#define ERROR_COMM_BCC              0x3002
#define ERROR_COMM_LENGTH           0x3003
#define ERROR_COMM_COMMAND          0x3004
#define ERROR_COMM_DATA             0x3005
#define ERROR_COMM_ETX              0x3006

#define CMD_RXD_STATE_WAIT_STX        0
#define CMD_RXD_STATE_WAIT_BCC        (CMD_RXD_STATE_WAIT_STX+1)
#define CMD_RXD_STATE_WAIT_LEN0       (CMD_RXD_STATE_WAIT_STX+2)
#define CMD_RXD_STATE_WAIT_LEN1       (CMD_RXD_STATE_WAIT_STX+3)
#define CMD_RXD_STATE_WAIT_LEN2       (CMD_RXD_STATE_WAIT_STX+4)
#define CMD_RXD_STATE_WAIT_LEN3       (CMD_RXD_STATE_WAIT_STX+5)
#define CMD_RXD_STATE_WAIT_COMMAND1	  (CMD_RXD_STATE_WAIT_STX+6)
#define CMD_RXD_STATE_WAIT_COMMAND2   (CMD_RXD_STATE_WAIT_STX+7)
#define CMD_RXD_STATE_WAIT_STATUS     (CMD_RXD_STATE_WAIT_STX+8)
#define CMD_RXD_STATE_WAIT_ERRORCODE  (CMD_RXD_STATE_WAIT_STX+9)
#define CMD_RXD_STATE_RECEIVE_DATA    (CMD_RXD_STATE_WAIT_STX+10)
#define CMD_RXD_STATE_WAIT_DATAID     (CMD_RXD_STATE_WAIT_STX+11)
#define CMD_RXD_STATE_WAIT_DATALENGTH (CMD_RXD_STATE_WAIT_STX+12)
#define CMD_RXD_STATE_WAIT_IDDATA     (CMD_RXD_STATE_WAIT_STX+13)
#define CMD_RXD_STATE_WAIT_ETX        (CMD_RXD_STATE_WAIT_STX+14)

#define CMD_EXECUTE_APPLICATION 	0x41  	// 'A'
#define CMD_EXECUTE_DOWNLOAD    	0x44  	// 'D'
#define CMD_EXECUTE_JUMP            0x4B
#define CMD_GET_INFO				0x47 	// 'G'
#define CMD_INSPECT_DEVICE  		0x49  	// 'I'
#define CMD_SET_INFO 				0x53  	// 'S'
#define CMD_VERIFICATION   			0X56  	// 'V'
#define CMD_FLASH_DOWNLOAD          0x65    // 'D'
#define CMD_TEST					0x54	// 'T'

#define RES_COMMON_STATUS			0xF0
#define RES_VERSION					0xF1
#define RES_VERFICATION_INFO		0xF2
#define RES_DEVICE_INFO				0xF3
#define RES_INVERTER_INFO			0xF4

#define IdCoData_SIZE				24

#define ADDR_APP_MAIN               0x000A0000
#define ADDR_APP_BOOT               0x00080000

typedef struct
{
	//u8 u8Century;
	u8 u8Year;
	u8 u8Month;
	u8 u8Day;
	u8 u8Weekday;
	u8 u8Hour;
	u8 u8Minutes;
	u8 u8Seconds;
} AP_DATE_OBJ;

typedef struct
{
	u8 u8DataID;
	u8 u8Data[MAX_DATA_LENGTH];
} AP_DATAFORMAT_OBJ;

typedef struct
{
	u8 u8BCC;
	u8 u8Length[4];
	u8 u8Command[2];
	AP_DATAFORMAT_OBJ RxData;
	u8 u8IsCommand;
	u16 u16RxDataCount;
	u16 u16CommandLength;
	u16 u16ErrorCode;
	u8 u8BCC_Calc;
	u8 u8State;
	u32 u32Timeout_Count;
} HW_RX_OBJ;

typedef struct
{
	u8 u8BCC;
	u8 u8Length[4];
	u8 u8Command[2];
	u8 u8EndStatus;
	AP_DATAFORMAT_OBJ TxData;
	u16 u16TxDataCount;
	u8 u16ErrorCode[4];
	u8 u8State;
} HW_TX_OBJ;

typedef struct
{
	u8 u8RunMode;
	u8 u8RunStatus;
	u8 u8LoadStatus;
	u8 u8DoorStatus;
	u8 u8CommStatus;
	u8 u8StopAbleFloor;
	u16 u16LoadValue;
	u8 u8BrakeCableSet;
	u8 u8HHTSet;
} CP_TX_OBJ;

typedef struct _StructDate
{
	u8 u8Year;
	u8 u8Month;
	u8 u8Date;
} TypeDate;

typedef struct _StructTime
{
	u8 u8Hour;
	u8 u8Minute;
	u8 u8Second;
} TypeTime;

typedef struct _StructIDCo
{
	u8 u8DataID;
	u8 u8DataLength[4];

	TypeDate rspDate;
	TypeTime rspTime;

	u32 u32TransNum;

	u8 u8CarSpeed;
	u8 u8CarType;
	u8 u8CarNumber;
	u8 u8CarPosition;
	u8 u8LimitSensorMode;
	u8 u8SPIMode;
	u8 u8APSMode;
	u8 u8LinearScaleMode;
	u8 u8Reserved;

} IdCoData;

typedef struct _StructIDMotorData
{
	/* FRLSM Motor Parameters */
	u8 u8MotorType;					// Motor Type, 0: Rotary Motor Type, 1: Linear Motor Type
	u8 u8MotorCapacitor[6];			// Motor Capacitor: 30[kVA] PF: 0.33
	u8 u8RatedVoltage[6];			// Rated Voltage: 380[V]
	u8 u8RatedCurrent[6];			// Rated Current: 60[A]
	u8 u8PolePitch[6];				// Pole Pitch: 24[mm]
	u8 u8BaseSpeed[6];				// Base Speed: 1[m/s](60[m/m])
	u8 u8IqRate[6];					// Q-axis Rated Current: 100[A]
	u8 u8KnowRpos;					// Do you know Rotor Position?: Unknow
	u8 u8RposOffset[6];				// Theta Offset: 0
	u8 u8RposMethod;				// Rotor Position Estimation Method: DC Injection
	u8 u8RposEstimationTime[6];		// DC Injection Time: 5[sec]
	u8 u8Ls[6];						// Motor Inductance: 0.015[H]
	u8 u8Rs[6];						// Motor Resistance: 0.145[ohm]
	u8 u8Kf[6];						// Thrust Force Constant: 0.235
	u8 u8Reserved[12];
}IdMotorData;

typedef struct _StructIDInverterData
{
	u8 u8MaxSpeed[6];				// Max Speed: 1.0[m/s]
	u8 u8MaxAcc[6];					// Max Accelator: 0.5[m/s^2]
	u8 u8MaxJerk[6];				// Max Jerk: 0.5[m/s^3]
	u8 u8Jm[6];						// Inertia: 50[kg/m^3]
	u8 u8SCFFGain[6];				// Speed Controller FeedForward Gain: 0.5
	u8 u8TorqueLimit[6];			// Torque Limit: 250[%]
	u8 u8ScaleVdc[6];				// DC-Link Voltage ADC Scale Factor
	u8 u8ScaleCurrent[6];			// Current ADC Scale Factor
	u8 u8Wcc[6];					// Current Controller Bandwidth: 100[Hz]
	u8 u8Wsc[6];					// Speed Controller Bandwidth: 2[Hz]
	u8 u8Wpc[6];					// Position Controller Bandwidth: 2[Hz]
	u8 u8OverCurrent[6];			// Over Current Setting: 150[A]
	u8 u8FwdDirection;				// Forward Direction
	u8 u8EncoderType;				// Encoder Type: Linear Encoder
	u8 u8MaxFloor[3];				// Max Floor
	u8 u8InpectSpeed[6];			// Inspect Speed: 0.15[m/s]
	u8 u8CreepSpeed[6];				// Creep Speed: 0.02[m/s]
	u8 u8RelevelSpeed[6];			// Re-level Speed: 0.03[m/s]
	u8 u8AntistallTime;				// AntistallTime
	u8 u8MaxLength[6];              // Max Length: 100,000[mm]
	u8 u8ZspLevel[4];               // ZSP Level: 0.005[m/s]
	u8 u8Reserved[125];
}IdInverterData;

typedef struct _StructDacSelectionData
{
    u8 u8DaVari[2];
    u8 u8DaType;
    u8 u8DaScale[8];
    u8 u8DaMid[6];
    u8 u8DaVariAdd[8];
}DacSelectionData;

typedef struct _StructID40
{
	IdCoData Data40;
} Id40Data;


typedef struct _StructID41
{
	IdCoData Data41;
	u8 u8RxData[40];
} Id41Data;

typedef struct _StructIDF0
{
	IdCoData DataF0;
	u8 u8TxData[104];
} IdF0Data;

typedef struct _StructIDF1
{
	u8 u8DataID;
	u8 u8TxData[24];
} IdF1Data;

typedef struct _StructIDF2
{
	u8 u8DataID;
	u8 u8DataLength[4];
	u8 u8Use[8];
	u8 u8RxData[51];
} IdF2Data;

typedef struct _StructIDF3
{
	u8 u8DataID;
	u8 u8DataLength[4];
	u8 u8RxData[64];
} IdF3Data;

typedef struct _StructIDF10
{
    u8 u8EcoCurrentRef[6];
    u8 u8Flag_RotorEstimation;
    u8 u8Flag_StaticForceCurrentCon;
} IdFAData;

typedef struct _StructID42
{
//	IdCoData 	Data42;
	IdMotorData u8RxData;
}Id42Data;

typedef struct _StructID43
{
//	IdCoData 		Data43;
	IdInverterData 	u8RxData;
}Id43Data;

typedef struct
{
    u8 u8Mode;                 // Mode Data
}Id44Data;

typedef struct
{
    u8 u8nDMC2;                // MC2
    u8 u8nDBKA;                // Brake Open/Close
    u8 u8nDBKB;                // Pre-charging MC B ON/OFF
    u8 u8nDBKP;                // Parking Brake Open/Close(Not Used)
    u8 u8nDDS;                 // Door Switch
    u8 u8nDMS;                 // Safety MC
    u8 u8nIFAN;                // Inverter FAN ON/OFF
    u8 u8nMC_A;                 // Pre-charging MC A ON/OFF
    u8 u8nDHCLD;               // Horizontal Close Locking Device
    u8 u8nDCLD;                // Close Locking Device
    u8 u8nINV_SPO1;            // Spare Output 1
    u8 u8nDHYDPRESS;           // Hyrdo-Pump ON/OFF
    u8 u8nINV_SPO2;            // Spare Output 2

}Id45Data;

typedef struct
{
    DacSelectionData u8Da0;
    DacSelectionData u8Da1;
    DacSelectionData u8Da2;
    DacSelectionData u8Da3;

    int     NumVari[4];
    int     type[4];
    float   Scale[4];
    float   Mid[4];
}Id46Data;

typedef struct
{
    DacSelectionData u8Da0;
    DacSelectionData u8Da1;
    DacSelectionData u8Da2;
    DacSelectionData u8Da3;

    long  NumVariAdd[4];
    long*   pNumVariAdd[4];
    int     type[4];
    float   Scale[4];
    float   Mid[4];
}Id47Data;

typedef struct
{
    u8 u8StaticForceCurrentRef[6];  // Static Force Measurement Mode Current Reference
    float32 StaticForceCurrentRef;
}Id48Data;

typedef union
{
    u8 u8ErrorData[50];

    struct{
        u8 u8Inv_Iqse[6];       // Fault Data - 1
        u8 u8Inv_Idse[6];       // Fault Data - 2
        u8 u8Inv_Iqse_Ref[6];   // Fault Data - 3
        u8 u8Inv_Idse_Ref[6];   // Fault Data - 4
        u8 u8Inv_DC_Link[6];    // Fault Data - 5
        u8 u8Inv_Vll[6];        // Fault Data - 6
        u8 u8Inv_Speed_Ref[4];  // Fault Data - 7
        u8 u8Inv_Thrust[7];     // Fault Data - 8
        u8 IO[3];               // [0]: UP/DOWN/RST/RDY/AT/XSA/XBKAS/XBKBS/BKO; [1]: XCLI/XDLV/XDZV/XHCLI/XULV; [2]: ZSP;
    }ErrorDataList;

}ErrorData;

typedef struct _SturctIDF5
{
    u8 u8FaultID;           // Fault ID
    TypeDate rspDate;       // Fault Date(Year, Month)
    TypeTime rspTime;       // Fault Time(Hour, Minute, Second)

    ErrorData u8ErrorData;  // Backup Error Data
} IdF5Data;

extern IdCoData	g_IdCoData;
extern Id40Data	g_Id40Data;
extern Id41Data	g_Id41Data;
extern Id42Data	g_Id42Data;
extern Id43Data	g_Id43Data;
extern Id44Data g_Id44Data;
extern Id45Data g_Id45Data;
extern Id46Data g_Id46Data;
extern Id47Data g_Id47Data;
extern Id48Data g_Id48Data;

extern IdF0Data g_IdF0Data;
extern IdF1Data g_IdF1Data;
extern IdF2Data g_IdF2Data;
extern IdF3Data g_IdF3Data;
extern IdF5Data g_IdF5Data[10];
extern IdFAData g_IdFAData;

typedef struct
{
	u8 x;
	u8 y;
	u8 z;

} POSITION_OBJ;

typedef struct
{
	u8 u8RunMode;
	u8 u8RunStatus;
	u8 u8LoadStatus;
    POSITION_OBJ S_POS;
	POSITION_OBJ N_POS;
	POSITION_OBJ E_POS;
    u8 u8SetPosition[4];
	u8 u8RemainPosition[4];
	u8 u8DoorStatus;
	u8 u8CommStatus;
	u8 u8StopAbleFloor;
	u8 u8LoadValue;
	u8 u8EncoderInfo;
	u8 u8APSRegion;
	u8 u8APS1PositionSign;
	u8 APS1Position[4];
	u8 u8APS2PositionSign;
	u8 APS2Position[4];
	u8 APS1Velocity[2];
	u8 APS2Velocity[2];
	u8 LaserSensor1[4];
	u8 LaserSensor2[4];
	u8 u8BrakeStatus;
    u8 InitOpStatus;

    u32 SetPosition;
    u32 CurrentPosition1;
    u32 CurrentPosition2;
    u32 CurrentVelocity1;
    u32 CurrentVelocity2;
    u32 LaserDistanceUp;
    u32 LaserDistanceDown;

    u8 u8PreLoadStatus;
} STATUS_DSP_OBJ;

typedef struct
{
    u8 u8DecelFlg;          // First Deceleration Signal
    u8 u8ChimeFlg;          // Second Deceleration Signal(Chime)
    u8 u8VoiceFlg;          // Third Deceleration Signal(Voice)
    u8 u8RunOpenFlg;        // Running Possible Signal
    u8 u8SpdPtnFlg;         // Speed Pattern
    u8 u8FlFindDir;         // Floor Find Direction
    u8 u8FlInitOK;          // Floor Measurement Operation Condition
    u8 u8LMotAngleFind;     // Initial Rotor Position Estimation Condition: Unknown(0x00), Know(0x01), Estimating(0x02), Otherwise(Unknown)
    u8 u8InvErr;            // Inverter Fault Signal: Normal(0x00), Error(0x01), Otherwise(Normal)
    u8 u8NearFlStop;        // The Nearest Floor Stop Request
    u8 u8NoCall;            // Don't have Call Information
    u8 u8PosErr;            // Position Error
    u8 u8FlInitWng;         // Floor Measurement Operation Error Signal: Not Success(0x00), Success(0x01), Otherwise(Not Success)
    u8 u8DoorZoneExceed;    // Exceeded Door Zone
    u8 u8DoorZoneLack;      // Lacked Door Zone
    u8 u8ElAntistall;       // Antistall Signal
    u8 u8InvWatchdog;       // Inverter Watch-dog Signal
    u8 u8CallErr;           // Doesn't be able to stop floor according to receive call information
    u8 u8ALDOp;             // Inverter Fault Operating(Auto Landing Device Operation Mode): Inverter Operation(0x00), ALD Operation(0x01), Otherwise(Inverter Operation)
    u8 u8ForceDecelOp;      // Force Deceleration Operation by Froce Decel SW
    u8 u8LowSpdOp;          // Low Speed Operation Signal: Normal(0x00), Low Speed Operation Mode(0x01), Otherwise(Normal)
    u8 u8McucommuChk;       // Communication Status Between DSP and ARM
    u8 u8MaxFloor[3];       // Max Floor(3 digit, 200 --> 0x32/0x30/0x30)
    u8 u8DecelFloor[3];     // Be able to deceleration Floor(3 digit, 200 --> 0x32/0x30/0x30)
    u8 u8CurrentFloor[3];   // Current Floor(3 digit, 200 --> 0x32/0x30/0x30)
    u8 u8Dummy[3];          // Load Ratio(0~125%)(3 digit, 125 --> 0x31/0x32/0x35)
    u8 u8CurrentSpeed[3];   // Current Speed(1.01[m/s], ASCII)
    u8 u8CurrentPos[8];     // Current Position(100,000.00 [mm], ASCII)
    u8 u8EL_Speed[3];       // Elevator Speed(060[m/m], ASCII)
    u8 u8INV_ErrCode;       // Inverter Error Code (Refer to VCSU_IFData_Spec.xlsx)
    u8 u8S_curveTime[2];    // S-Curve Time(1.0sec --> 0x31/0x30)
    u8 u8PTN_Accel[3];      // Acceleration(0.50m/s^2 --> 0x30/0x35/0x30)
    u8 u8Inv_Iqse[6];       // Control Data for monitoring(6 digit, 100.001 --> 0x31/0x30/0x30 ./ 0x30/0x30/0x31)
    u8 u8Inv_Idse[6];       // Control Data for monitoring(6 digit, 100.001 --> 0x31/0x30/0x30 ./ 0x30/0x30/0x31)
    u8 u8Inv_Iqse_Ref[6];   // Control Data for monitoring(6 digit, 100.001 --> 0x31/0x30/0x30 ./ 0x30/0x30/0x31)
    u8 u8Inv_Idse_Ref[6];   // Control Data for monitoring(6 digit, 100.001 --> 0x31/0x30/0x30 ./ 0x30/0x30/0x31)
    u8 u8Inv_DC_Link[6];    // Control Data for monitoring(6 digit, 100.001 --> 0x31/0x30/0x30 ./ 0x30/0x30/0x31)
    u8 u8Inv_Vll[6];        // Control Data for monitoring(6 digit, 100.001 --> 0x31/0x30/0x30 ./ 0x30/0x30/0x31)
    u8 u8Inv_Speed_Ref[4];  // Control Data for monitoring(3 digit, 1.001 --> 0x31 ./ 0x30/0x30/0x31)
    u8 u8Inv_Thrust[7];     // Control Data for monitoring(6 digit, 100.001 --> 0x31/0x30/0x30 ./ 0x30/0x30/0x31)
    u8 u8BK_status1;        // Right Brake Open Status(0x01: Opened, 0x00: Closed)
    u8 u8BK_status2;        // Left Brake Open Status(0x01: Opened, 0x00: Closed)
    u8 u8reserved[25];      // Reserved Data, Total Data Size is 128Byte.
} STATUS_ARM_OBJ;

extern STATUS_DSP_OBJ 	g_DspStatus;
extern STATUS_ARM_OBJ   g_ArmStatus;

extern void DSP_COMMAND_TASK(HW_RX_OBJ *pCmd, HW_TX_OBJ *pCmd2);
extern void Get_Status(HW_RX_OBJ *pCmd);
extern void Cmd_Parsing(HW_RX_OBJ *pCmd, u8 *pData, u8 size);
extern void Id41_Data_Parsing(u8 *pData);
extern void Id42_Data_Parsing(u8 *pData);
extern void Id43_Data_Parsing(u8 *pData);
extern void Id44_Data_Parsing(u8 *pData);
extern void Id45_Data_Parsing(u8 *pData);
extern void Id46_Data_Parsing(u8 *pData);
extern void Id47_Data_Parsing(u8 *pData);
extern void Id48_Data_Parsing(u8 *pData);
extern void Send_Version(HW_TX_OBJ *pCmd, u16 u16size);
extern void Send_IdF0(HW_TX_OBJ *pCmd, u16 u16size);
extern void SendIdF5(HW_TX_OBJ *pCmd, u16 u16size);
extern void SendIdFA(HW_TX_OBJ *pCmd, u16 u16size);
extern void Send_Id42(HW_TX_OBJ *pCmd, u16 u16size);
extern void Send_Id43(HW_TX_OBJ *pCmd, u16 u16size);
extern void Send_Id44(HW_TX_OBJ *pCmd, u16 u16size);
extern void Send_Id45(HW_TX_OBJ *pCmd, u16 u16size);
extern void Send_Id46(HW_TX_OBJ *pCmd, u16 u16size);
extern void Send_Id47(HW_TX_OBJ *pCmd, u16 u16size);
extern void INV_To_Main_Msg_Pkt(HW_TX_OBJ *pCmd, u32 size, DefErr Error);
extern void Get_MotorData(HW_RX_OBJ *pCmd);
extern void Get_InverterData(HW_RX_OBJ *pCmd);
extern void Get_Mode_Data(HW_RX_OBJ *pCmd);
extern void Get_Digital_Output_Check(HW_RX_OBJ *pCmd);
extern void Get_Dac_Selection(HW_RX_OBJ *pCmd);
extern void Get_Dac_Selection_Mode2(HW_RX_OBJ *pCmd);
extern void Get_Static_Force_Measurement_Mode(HW_RX_OBJ *pCmd);
extern float TypeCast_u8tofloat(u8 *subject, u8 integer, u8 decimal);
extern void TypeCast_floattou8(float fobject, u8 *subject, u8 integer, u8 decimal);
void DspSwReset(HW_RX_OBJ *pCmd);
extern void AckSend(HW_TX_OBJ *pCmd, u16 u16size);
extern DefErr GenErrorCode(u8 ErrorCode);
extern void Load_Motor_Data();
extern void Load_Inverter_Data();
extern void Load_DAC_Setting();
#endif
