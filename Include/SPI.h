/*****************************************************************************
-------------------------------Organization------------------------------------
    Project             : FREEWAY
    Compiler            : TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
    Author              : Inverter part, Advanced Controls Research Team in Hyudai elevator(with KERI)
    Version             : v1.0
    Last Rev.           : 2018.12.28
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
                        : Added SCI Protocol Infomation Response Part (20181107 by GWON)
                        : Rearrangement of F/W for FREEWAY (20181112 by GWON)
                        : Added SPI Init Setting for between ARM and DSP communication (20181127 by GWON)
                        : Added SPI Control Communication Part (20181130 by GWON)
                        : Ver2 Board GPIO Setting (20181207 by GWON)
                        : Verified SPI Protocol (20181224 by GWON & Shin)
                        : Removed RMB, RAMP, MSTEP, FC (20190108 by GWON)
                        : Heidenhain Encoder F/W Merge using SPIC & DMA (20190702 by GWON)
                        : DAC Selection Data Insert at Inverter Data on SCI (20190714 by GWON)
                        : DAC Selection F/W is added on InitDa() (20190714 by GWON)
******************************************************************************/
#ifndef _SPI_
#define _SPI_

#include "variable.h"
#include "gnl.h"
#include "SCI.h"

#define SPIB_POLLING					0

#define	CPUCLK							200000000L
#define	LSP_CLOCK						(CPUCLK/4)
#define SPIB_CLOCK						12500000L
//#define SPIB_CLOCK						1560000L
#define SPIB_BRR						(LSP_CLOCK/SPIB_CLOCK)-1

#define SPIC_CLOCK                      6250000L//12500000L
#define SPIC_BRR                        (LSP_CLOCK/SPIC_CLOCK)-1

#define BURST         (FIFO_LVL-1)      // burst size should be less than 8
#define TRANSFER      1                 // [(MEM_BUFFER_SIZE/FIFO_LVL)-1]
#define FIFO_LVL      5                 // FIFO Interrupt Level

// Run Mode Definition
#define STOP_DRIVE		    0x30
#define INIT_DRIVE		    0x31
#define TEST_DRIVE		    0x32
#define EMERGENCY_DRIVE	    0x33
#define RE_LEVEL_DRIVE	    0X34
#define MANUAL_DRIVE	    0X35
#define AUTO_DRIVE		    0X36
#define STATIC_MEASUREMENT  0x37
#define OUTPUT_DEBUG        0xFF

// Run Status
#define RUN_STOP		0x30
#define RUN_UP			0x31
#define RUN_DOWN		0x32
#define RUN_RIGHT		0x33
#define RUN_LEFT		0x34

// Deceleration Mode Definition
#define NOMAL_OP                0b00000000
#define FORCE_DECEL_OP          0b00000001
#define FIRST_IN_POSITION_OP    0b00000010
#define SECOND_IN_POSITION_OP   0b00000100

typedef struct {
    uint32_t begin;
    uint32_t elapsed;
    uint32_t pre;
} ENC_TIME_OBJ;

typedef struct  {                                 // bit descriptions
    u32  positionA;
    u32  positionB;
    float32   diff;
    u32  rxcount;
    u32  txcount;
    u16  error;
    volatile struct SPI_REGS *spi;
    ENC_TIME_OBJ time[4];
    u8 sdata[16];     // Send data buffer
    u8 rdata[16];     // Receive data buffer

    float32 positionfA;
    float32 positionfB;

    float32 positionfA_f;
    float32 positionfB_f;

    float32 spdA;
    float32 spdB;

    float32 positionfA_d1;
    float32 positionfB_d1;

    float32 spdA_f;
    float32 spdB_f;

    float32 thetaA;
    float32 thetaB;

    float32 temp_theta;
    float32 alpha;
    float32 beta;
    float32 d;
    float32 q;
    float32 Errq;
    float32 Errq_d1;
    float32 Kp_theta;
    float32 Ki_theta;
    float32 intq;
    float32 freq_d1;
    float32 freq;
    float32 theta;
    float32 theta_d1;
    float32 realpositionA;
    float32 realpositionB;
    float32 Wr;
    float32 theta_elec;
    float32 speed_mech2;
    float32 position;

    float32 Calibration;

    float32 selPosition;
    float32 Offset_Cal;

    float32 Zero_Cal;

    /* Highpass Filter */
    float32 s1;
    float32 s2;
    float32 a1;
    float32 a2;
    float32 a3;
    float32 b1;
    float32 b2;
    float32 b3;

    float32 LPF_spd;
    float32 LPF_spd_d1;
    float32 LPF_spd_d2;
    float32 spd_d1;
    float32 spd_d2;
    float32 LPF_spd_diff;
    int accum;
    u8 Flag_accum;

    /* Transient Count Algorithm */
    int Taccum;
    float32 selPosition_dd1;
    float32 selPosition_diff;

    float32 floorPosition;

    u16  dataReady;
} ENC_DATA_OBJ;

extern IdCoData	g_IdCoSpiData;
extern Id41Data	g_Id41SpiData;
extern IdF0Data g_IdF0SpiData;

__interrupt void cpu1_Spib_Rx(void);
__interrupt void cpu1_Spib_Tx(void);
__interrupt void cpu1_Spic_Tx(void);        // for Encoder
__interrupt void cpu1_Spic_Rx(void);        // for Encoder

void InitSPIC(void);
void InitSPIB(void);
void INV_To_Main_Spi_Msg_Pkt(HW_TX_OBJ *pCmd, u32 size, DefErr Error);
extern void DSP_COMMAND_SPI_TASK(HW_RX_OBJ *pCmd, HW_TX_OBJ *pCmd2);
void Send_Spi_IdF0(HW_TX_OBJ *pCmd, u16 u16TXsize);
void Get_Spi_Status(HW_RX_OBJ *pCmd);
void Spi_Cmd_Parsing(HW_RX_OBJ *pCmd, u8 *pData, u8 size);
void Id41_Spi_Data_Parsing(u8 *pData);
void MakeSendSPIData(void);
extern void Init_Spib_Pkt(HW_TX_OBJ *pCmd);
void DSPStatusData(STATUS_ARM_OBJ *pArmObj);

/* Ring Buffer */
extern void Spi_ring_buffer_init(void);
extern u32 Spi_ring_buffer_length(void);
extern void Spi_ring_buffer_push(u32 data);
extern u32 Spi_ring_buffer_pop(void);

#endif
