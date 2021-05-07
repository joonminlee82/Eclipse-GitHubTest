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
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "CPU1_CLA_Shared.h"
#include "Init_CMPSS.h"
#include "Init_Func_Prototypes.h"
#include "easy2837xD_CPU1_sci_v8.5.h"
#include "CPU1_CLA_Shared.h"
#include "AD_Offset.h"
#include "R_control.h"
#include "Filter.h"
#include "SNU_speed_profile.h"
#include "CC.h"
#include "SC.h"
#include "drive.h"
#include "SCI.h"
#include "SPI.h"
#include "vane.h"
#include "Device.h"
#include "FRAM.h"
#include "DAC.h"
#include "keymenu.h"
#include "eQEP_Encoder.h"

#define USEWATCHDOG

__interrupt void wakeint_isr(void);
__interrupt void local_D_INTCH5_ISR(void);
__interrupt void local_D_INTCH6_ISR(void);
void dma_init(void);
void spi_fifo_init(void);



volatile Uint16 *DMADest;
volatile Uint16 *DMASource;
volatile Uint16 *DMADestSC;
volatile Uint16 *DMASourceSC;
volatile Uint16 done;

// Prototype statements for functions found within this file.
extern  int32 Delay_Cnt;
Uint32 main_cnt =0;
extern HW_RX_OBJ 	Hw_RX_Data;
extern HW_TX_OBJ	Hw_TX_Data;
extern AP_DATE_OBJ *g_pDate;

extern HW_RX_OBJ	Hw_RX_Spi_Data;
extern HW_TX_OBJ	Hw_TX_Spi_Data;

extern IdCoData     g_IdCoData;

extern ENC_DATA_OBJ ENC1_Data;
extern ENC_DATA_OBJ ENCB_Data;
extern ENC_DATA_OBJ ENCA_Data;
extern ENC_DATA_OBJ ENCA_Data_SC;

u8 TestSPI = 0;
u8 Flag_SPI_Test = 0;

//#ifdef USEWATCHDOG
Uint32 WakeCount;

int DMA_CH5_Cnt = 0;
int DMA_CH6_Cnt = 0;

//#endif

void main(void)
{
	PWM1_BUFF_OFF();
	nMC_OFF ;				// MC disable
	nFAULT_OFF ;			// 30B disable
  	nINV_FAN_ON ;			// Fan enable
	nINV_RUN_OFF ;			// RUN disable
	nBKO_OFF ;				// BKO disable
	nINV_ZSP_OFF ;			// ZSP disable
	MCU0_ON ;
	MCU1_OFF ;
	nINIC_OFF;
	GATE_RST_ON();

	Flags.Bootup = 0;       // For Boot-up Sequence

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
   	InitSysCtrl();
	
// Step 2. Initalize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//   InitGpio();

// Step 3. Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts
   	DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
   	InitPieCtrl();

// Step 4. Configure the CLA memory spaces first followed by
// the CLA task vectors
   	InitCla();

// Disable CPU __interrupts and clear all CPU __interrupt flags:
   	IER = 0x0000;
   	IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
   	InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   	EALLOW; // This is needed to write to EALLOW protected registers

   	PieVectTable.EPWM1_INT    	= &cpu1_offset;			//(3.1)
	PieVectTable.EQEP3_INT    	= &cpu1_sc;				//(5.3)
	PieVectTable.I2CA_INT     	= &cpu1_drive;			//(8.1)
	PieVectTable.XINT4_INT    	= &cpu1_enc_Z;			//(12.2)
//	PieVectTable.SCIB_RX_INT    = &cpu1_Scib_Rx;        //(9.3)
   	PieVectTable.SCIC_RX_INT  	= &cpu1_Scic_Rx;		//(8.5)
	PieVectTable.SCIC_TX_INT  	= &cpu1_Scic_Tx;		//(8.6)
//	PieVectTable.SCID_RX_INT  	= &cpu1_Scid_Rx;		//(8.7)
//	PieVectTable.SCID_TX_INT  	= &cpu1_Scid_Tx;		//(8.8)
//	PieVectTable.SPIC_TX_INT	= &cpu1_Spic_Tx;		//(6.10)
	PieVectTable.SPIB_RX_INT	= &cpu1_Spib_Rx;		//(6.3)
	PieVectTable.SPIB_TX_INT	= &cpu1_Spib_Tx;		//(6.4)
	PieVectTable.EPWM1_TZ_INT 	= &epwm1_tzint_isr;		//(2.1)
	PieVectTable.ADCA_EVT_INT 	= &epwm1_tzint_isr;		//(10.1)
	PieVectTable.ADCB_EVT_INT 	= &epwm1_tzint_isr;		//(10.5)
	PieVectTable.ADCC_EVT_INT 	= &epwm1_tzint_isr;		//(10.9)
	PieVectTable.ADCD_EVT_INT 	= &epwm1_tzint_isr;		//(10.13)
    PieVectTable.DMA_CH5_INT    = &local_D_INTCH5_ISR;
    PieVectTable.DMA_CH6_INT    = &local_D_INTCH6_ISR;
#ifdef USEWATCHDOG
	PieVectTable.WAKE_INT 		= &wakeint_isr;
#endif
   	EDIS;   // This is needed to disable write to EALLOW protected registers

// Step 5. Initialize the Device Peripheral.
	GPIO_setPinMuxConfig();
	Gpio_setup();
	
	DELAY_US(100000L);						// 100mSec

	PWM1_BUFF_OFF();
	nMC_OFF ;								// MC disable
	nFAULT_OFF ;							// 30B disable
  	nINV_FAN_ON ;							// Fan enable
	nINV_RUN_OFF ;							// RUN disable
	nBKO_OFF ;								// BKO disable
	nINV_ZSP_OFF ;							// ZSP disable
	MCU0_ON ;
	MCU1_OFF ;
	nINIC_OFF;
	INV_WD1_OFF;
	INV_WD2_ON;

	InitCMPSS();				 			// analog fault function
	InitAdc(); 								// ADC A,B,C,D
	InitFault(); 							// Fault Reconfiguration
	InitXbar();
	InitEPwm();
	InitEmif2(); 							// for external DAC
	InitEQep();								// for Linear Encoder
//	InitECap();								// APWM mode for SigmaDelta filter Module
//	Init_SDFM();							// for SDFM,
//    InitSciB();                             // for RFID Communication
	InitSciC();								// for DSP-ARM Communication
//	InitSciD();								// for HHT
	InitSPIB();								// for CP COM
	dma_init();                             // Set up DMA for SPI configuration
	InitSPIC();                             // for Encoder
	InitCpuTimers();
  	ConfigCpuTimer(&CpuTimer0, 200, 1000000);
  	CpuTimer0Regs.TCR.all = 0x4000; 		// Use write-only instruction to set TSS bit = 0, 1111111111111100001100000011b

    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;
    EDIS;

#ifdef USEWATCHDOG
  	EALLOW;
	WdRegs.SCSR.all = BIT1;
	EDIS;
#endif


	// Enable CPU INT3 which is connected to EPWM1-3 INT:
//	IER |= M_INT1|M_INT2|M_INT3|M_INT5|M_INT6|M_INT8|M_INT9|M_INT10;
    IER |= M_INT1|M_INT2|M_INT3|M_INT5|M_INT6|M_INT8|M_INT10;
#ifdef USEWATCHDOG
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
	PieCtrlRegs.PIEIER1.bit.INTx8 = 1;   // Enable PIE Group 1 INT8
#endif
	// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
	PieCtrlRegs.PIEIER3.bit.INTx1 	= 1;	//EPWM1 (cc)
	PieCtrlRegs.PIEIER5.bit.INTx3 	= 1;	//EQEP3 (sc)
	PieCtrlRegs.PIEIER8.bit.INTx1 	= 1;	//I2CA (drive)
	PieCtrlRegs.PIEIER2.bit.INTx1 	= 1;	//EPWM1_TZ
	PieCtrlRegs.PIEIER10.bit.INTx1 	= 1;	//ADCA_EVT_INT
	PieCtrlRegs.PIEIER10.bit.INTx5 	= 1;	//ADCB_EVT_INT
	PieCtrlRegs.PIEIER10.bit.INTx9 	= 1;	//ADCC_EVT_INT
	PieCtrlRegs.PIEIER10.bit.INTx13 = 1;	//ADCD_EVT_INT
	PieCtrlRegs.PIEIER12.bit.INTx2 	= 1;	//XINT4 (enc_z)
//	PieCtrlRegs.PIEIER9.bit.INTx3   = 1;    //SCIB_RX_INT (RFID.RX)
	PieCtrlRegs.PIEIER8.bit.INTx5 	= 1;	//SCIC_RX_INT (EnDat.RX)
	PieCtrlRegs.PIEIER8.bit.INTx6 	= 1;	//SCIC_TX_INT (EnDat.TX)
//	PieCtrlRegs.PIEIER8.bit.INTx7 	= 1;	//SCID_RX_INT (HHT.RX)
//	PieCtrlRegs.PIEIER8.bit.INTx8 	= 1;	//SCID_TX_INT (HHT.TX)
//	PieCtrlRegs.PIEIER6.bit.INTx10 	= 1;	//SPIC_TX_INT (DAC.TX)
	PieCtrlRegs.PIEIER6.bit.INTx3	= 1;	//SPIB_RX_INT (CP COM.RX)
	PieCtrlRegs.PIEIER6.bit.INTx4	= 1;	//SPIB_TX_INT (CP COM.TX)
    PieCtrlRegs.PIEIER7.bit.INTx5 = 1;   // Enable PIE Group 7, INT 1 (DMA CH1)
    PieCtrlRegs.PIEIER7.bit.INTx6 = 1;   // Enable PIE Group 7, INT 2 (DMA CH2)

	// 사용자의 인터럽트 관련 설정 이후 호출
//	easyDSP_SCIBootCPU2(); //CPU1만 사용시 사용 안함?
	easyDSP_SCI_Init();
// Enable global Interrupts and higher priority real-time debug events:
	EINT;  									// Enable Global interrupt INTM
	ERTM;  									// Enable Global realtime interrupt DBGM
// GPIO26 is XINT4
	GPIO_SetupXINT4Gpio(81);                // enc_z interrupt
// Configure XINT4
  	XintRegs.XINT4CR.bit.POLARITY = 0;      // Falling edge interrupt
// Enable XINT4
  	XintRegs.XINT4CR.bit.ENABLE = 1;        // Enable XINT4

#ifdef USEWATCHDOG
  	//
  	// Reset the watchdog counter
  	//
	ServiceDog();

	//
	// Enable the watchdog
	//
	EALLOW;
	WdRegs.WDCR.all = 0x0028;
	EDIS;
#endif
	nCS_FRAM_OFF;
	//-----------User program------------//
	FRAM_Check();																// FRAM Check
	SetROMTable();																// Created FRAM Table

	FRAM_VersionCheck();														// FRAM Version Check
	if(FRAM_value != E_Check)	FLT_Raise(FLT_FRAM);							// FRAM Check
	if(ROM_GetVersion() != FRAM_GetVersion())	FLT_Raise(FLT_VERSION);			// FRAM ROM Check

	InitFlags();
	InitParameters();
	Init_Inv();																	// Current Controller Variables Initialization
	Init_SC();																	// Speed Controller Variables Initialization
	Init_PROFILE();																// Speed & Position Profile Variables Initialization

//	CC_VariUpdate();															// Current Controller Variables Update
//	SC_VariUpdate();															// Speed Controller Variables Update
//	PC_VariUpdate();                                                            // Position Controller Variables Update
//	InitEnc(&Encoder_HD, (Uint32)EncPPR,(float)INV1.PolePair, 0, &EQep2Regs);	// Encoder Variables Initialization

	DRV_Create();																// For Drive.c
  	SCI_Create();																// For SCIC
	BDIB_Create();																// For Digital I/O Mapping
	FLT_Reset();
	Init_ADScale();																// For ADC Scale

	InitECap();																	// APWM mode for SigmaDelta filter Module

	IIR2Init(&Filter_Ias, 3900.*TWOPI, 0.707 );									// For Nyquist Frequency Active Damping of LC and LCL Filter (100uH + 4uF LC Filter)
	IIR2Init(&Filter_Ibs, 3900.*TWOPI, 0.707 );									// For Nyquist Frequency Active Damping of LC and LCL Filter (100uH + 4uF LC Filter)
	IIR2Init(&Filter_Ics, 3900.*TWOPI, 0.707 );									// For Nyquist Frequency Active Damping of LC and LCL Filter (100uH + 4uF LC Filter)

	IIR2Init(&Filter_Wrm, 200.*TWOPI, 0.707 );									// For Speed Measurement Noise Rejection(Real Speed Value)
	IIR2Init(&Filter_Obs_Iq_ref, 100.*TWOPI, 0.707 );							// For q-axis reference Noise Rejection(Observer Speed Value)
	IIR1CoeffInit(&Filter_Enc,100*TWOPI);

    IIR2Init(&Filter_Active_Power, 100.*TWOPI, 0.707 );                         // For Active Power
    IIR2Init(&Filter_Reactive_Power, 100.*TWOPI, 0.707 );                       // For Reactive Power

    IIR2Init(&Filter_Hiedenhain_A, 500.*TWOPI, 0.707 );                         // For Hiedenhain Position
    IIR2Init(&Filter_Hiedenhain_B, 500.*TWOPI, 0.707 );                         // For Hiedenhain Position
    IIR2Init(&Filter_Hiedenhain_Spd_A, 10.*TWOPI, 0.707 );                      // For Hiedenhain Speed
    IIR2Init(&Filter_Hiedenhain_Spd_B, 10.*TWOPI, 0.707 );                      // For Hiedenhain Speed

	Spi_ring_buffer_init();                                                     // SPI Ring Buffer Clear

	g_IdCoData.u8CarType = KEY_GetControlCode(CAR_TYPE);
	g_Id43Data.u8RxData.u8EncoderType = FRAM_Read(KEY_FACTORY_START+ENCODERTYPE) + 0x30;

	Init_Spib_Pkt(&Hw_TX_Spi_Data);

	Load_Motor_Data();
	Load_Inverter_Data();
	Load_DAC_Setting();

	FRAM_Load();
    CC_VariUpdate();                                                            // Current Controller Variables Update
    SC_VariUpdate();                                                            // Speed Controller Variables Update
    PC_VariUpdate();                                                            // Position Controller Variables Update

    //external DAC Initiate
    InitDa(0);

    if(KNOW_ANGLE) {
        InitEnc(&Encoder_HD, (Uint32)EncPPR,(float)INV1.PolePair, 0, &EQep2Regs);                   // Encoder Variables Initialization
        Encoder_HD.theta_init       = INV1.ThetaOffset;                    // The Angle is re-stored at power up after Rotor Position Estimation
        Encoder_HD.PosCntOffset = INV1_PC.Pos_Offset;
        ENCA_Data.Zero_Cal          = KEY_GetControlCode(ZERO_CAL);
        ENCA_Data.Taccum            = (int)KEY_GetControlCode(ACCUM);
        ENCA_Data.selPosition       = KEY_GetControlCode(SEL_POS);

        if(g_Id43Data.u8RxData.u8EncoderType == __LINEARSCALEMODE)
        {
            INV1_PC.Pos = (float)(Encoder_HD.PosCntOffset) * __ENC_RESOLUTION;
        }
        else if(g_Id43Data.u8RxData.u8EncoderType == __HEIDENHAINMODE)
        {
            INV1_PC.Pos = ((ENCA_Data.selPosition + ((float32)ENCA_Data.Taccum * UNIT_ABS_POS)) - ENCA_Data.Zero_Cal)*0.001;
        }
        INV1_PC.PosRefFinal = INV1_PC.Pos;
//        INV1_PROFILE.PosRef         = (float)(Encoder_HD.PosCntOffset) * __ENC_RESOLUTION;
//        INV1_PROFILE.PosRef_d1      = INV1_PROFILE.PosRef;
//        INV1_PROFILE.POS_CMD        = INV1_PROFILE.PosRef;
//        INV1_PROFILE.POS_CMD_old    = INV1_PROFILE.POS_CMD;
//        INV1_PROFILE.Comp_Profile   = INV1_PROFILE.PosRef;
//      INV1_PC.POS_CMD_d1 = INV1_PROFILE.PosRef;   ///// Not Verify
        Flags.CompProfile = 0;

    }
    else {
        INV1_PC.Pos             = 0.;
        Encoder_HD.theta_init   = 0.;
        Encoder_HD.PosCntOffset = 0;
        ENCA_Data.Zero_Cal      = 0.0;       // V-1 : 158.8518677 V-3 : 167.7305756;           // Init Position for Heidenhain Encoder
        ENCA_Data.Taccum        = 0;
        InitEnc(&Encoder_HD, (Uint32)EncPPR,(float)INV1.PolePair, 0, &EQep2Regs);                   // Encoder Variables Initialization

//        Flags.Bootup = 1;
    }

//    InitEnc(&Encoder_HD, (Uint32)EncPPR,(float)INV1.PolePair, 0, &EQep2Regs);   // Encoder Variables Initialization
    Init_PROFILE();                                                             // Speed & Position Profile Variables Initialization
	// Step 6. IDLE loop. Just sit and loop forever (optional):
   	while(1){
   		main_cnt++ ;
   		KEY_FLT_GetStatus();
   		if(SaveFault_Flg) FaultError();

   		if(Hw_RX_Data.u8IsCommand == TRUE) 		DSP_COMMAND_TASK(&Hw_RX_Data, &Hw_TX_Data);						// SCIC Commnad Check
   		DELAY_US(1000L);

   		/* Storing Error History */
#ifdef USEWATCHDOG
		ServiceDog();
#endif
   	}
}

void Gpio_setup(void)
{
	   EALLOW;
	   //GPIO Driection Setting
	   GpioCtrlRegs.GPADIR.all = GpioCtrlRegs.GPADIR.all | 0x8240043F; //10000000010000000000010000111111b //31-0
	   GpioCtrlRegs.GPBDIR.all = GpioCtrlRegs.GPBDIR.all | 0x0000000C; //00000000000000000000000000001100b //63-32
	   GpioCtrlRegs.GPCDIR.all = GpioCtrlRegs.GPCDIR.all | 0xC0100000; //11000000000100000000000000000000b //95-64
	   GpioCtrlRegs.GPDDIR.all = GpioCtrlRegs.GPDDIR.all | 0x03D7BFFD; //00000011110101111011111111111101b //127-96
	   GpioCtrlRegs.GPEDIR.all = GpioCtrlRegs.GPEDIR.all | 0x018057F8; //00000001100000000101011111111000b //159-128
	   GpioCtrlRegs.GPFDIR.all = GpioCtrlRegs.GPFDIR.all | 0x00000000; //000000000b //168-160

	   GpioDataRegs.GPCSET.bit.GPIO94 = 1;  //PWM buffer1 OFF
	   
	   GpioCtrlRegs.GPFPUD.bit.GPIO164 = 0; // 15V_nPFO pull-up enable
	   GpioCtrlRegs.GPEPUD.bit.GPIO132 = 0; // Fault pull-up enable

	   GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;	// SPISIMOB pull-up enable
	   GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;	// SPICLKB pull-up enable
	   GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;	// SPISTEB pull-up enable
	   
//	   GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;  // SPISIMOC pull-up enable
//       GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;  // SPICLKB pull-up enable
//       GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;  // SPISTEB pull-up enable


	   GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 1;
	   GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 1;
	   GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 1;

//	   GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 1;
//	   GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 1;
//       GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 1;

	   GpioCtrlRegs.GPCCTRL.bit.QUALPRD2 = 0xFF;	//enc_z : 734313.7255Hz
	   GpioCtrlRegs.GPCQSEL2.bit.GPIO81	 = 0x2; 	//enc_z : 6 smaples qualification = 130718.9542Hz = 7.65us , protectiont of z pulse chattering

	   EDIS;
}

//
// wakeint_isr - Wake up interrupt service routine
//
__interrupt void wakeint_isr(void)
{
    WakeCount++;
    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void dma_init()
{
    //
    // Initialize DMA
    //
    DMAInitialize();

    DMASource = (volatile Uint16 *)ENCA_Data.sdata;
    DMADest = (volatile Uint16 *)ENCA_Data.rdata;

    DMASourceSC = (volatile Uint16 *)ENCA_Data_SC.sdata;
    DMADestSC = (volatile Uint16 *)ENCA_Data_SC.rdata;

    //
    // configure DMACH3 for TX
    //
    DMACH3AddrConfig(&SpicRegs.SPITXBUF,DMASourceSC);
    DMACH3BurstConfig(BURST,1,0);         // Burst size, src step, dest step
    DMACH3TransferConfig(TRANSFER,1,0);   // transfer size, src step, dest step
    DMACH3ModeConfig(DMA_SPICTX,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,
                   SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,
                   CHINT_END,CHINT_ENABLE);

    //
    // configure DMA CH4 for RX
    //
    DMACH4AddrConfig(DMADestSC,&SpicRegs.SPIRXBUF);
    DMACH4BurstConfig(BURST,0,1);
    DMACH4TransferConfig(TRANSFER,0,1);
    DMACH4ModeConfig(DMA_SPICRX,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,
                   SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,
                   CHINT_END,CHINT_ENABLE);

    //
    // configure DMACH5 for TX
    //
    DMACH5AddrConfig(&SpicRegs.SPITXBUF,DMASource);
    DMACH5BurstConfig(BURST,1,0);         // Burst size, src step, dest step
    DMACH5TransferConfig(TRANSFER,1,0);   // transfer size, src step, dest step
    DMACH5ModeConfig(DMA_SPICTX,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,
                    SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,
                    CHINT_END,CHINT_ENABLE);

   //
   // configure DMA CH6 for RX
   //
    DMACH6AddrConfig(DMADest,&SpicRegs.SPIRXBUF);
    DMACH6BurstConfig(BURST,0,1);
    DMACH6TransferConfig(TRANSFER,0,1);
    DMACH6ModeConfig(DMA_SPICRX,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,
                    SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,
                    CHINT_END,CHINT_ENABLE);
}

//
// local_D_INTCH5_ISR - DMA Channel 5 ISR
//
__interrupt void local_D_INTCH5_ISR(void)
{
    DMA_CH5_Cnt++;
    EALLOW;  // NEED TO EXECUTE EALLOW INSIDE ISR !!!
    DmaRegs.CH5.CONTROL.bit.HALT=1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7; // ACK to receive more interrupts
                                            // from this PIE group
    EDIS;
    return;
}

//
// local_D_INTCH6_ISR - DMA Channel 6 ISR
//
__interrupt void local_D_INTCH6_ISR(void)
{
    DMA_CH6_Cnt++;
    EALLOW;  // NEED TO EXECUTE EALLOW INSIDE ISR !!!
    DmaRegs.CH6.CONTROL.bit.HALT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7; // ACK to receive more interrupts
                                            // from this PIE group
    EDIS;
    return;
}
//===========================================================================
// No more.
//===========================================================================

