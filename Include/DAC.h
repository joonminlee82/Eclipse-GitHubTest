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
#ifndef _DAC_H
#define _DAC_H

#include "F2837xD_Cla_typedefs.h"
#include "F28x_Project.h"
#include "SCI.h"

extern void InitDa(u8 u8mode);
extern void daOut(void);

extern long *getadd(int vari);

//__interrupt void cpu1_Spic_Tx(void);

//#define EMIF_CS
#define EMIF2_CS0		0x90000000 //end ADD : 0x93FF FFFF
#define EMIF2_CS2		0x00002000 //end ADD : 0x0000 2FFF, 10000000000000b

// External DAC
#define	DA_CH1		(*(volatile Uint16 *)(EMIF2_CS2+0x0))
#define	DA_CH2		(*(volatile Uint16 *)(EMIF2_CS2+0x2))
#define	DA_CH3		(*(volatile Uint16 *)(EMIF2_CS2+0x4))
#define	DA_CH4		(*(volatile Uint16 *)(EMIF2_CS2+0x7))

//#define	DA_CH5		(*(volatile Uint16 *)(EMIF2_CS2+0x0))
//#define	DA_CH6		(*(volatile Uint16 *)(EMIF2_CS2+0x8))
//#define	DA_CH7		(*(volatile Uint16 *)(EMIF2_CS2+0x10))
//#define	DA_CH8		(*(volatile Uint16 *)(EMIF2_CS2+0x18))

/* DA Variables List */
/* Class A */
#define DA_CC                   0x30
#define DA_SC                   0x31
#define DA_PC                   0x32
#define DA_PROFILE              0x33
#define DA_ROTOR_EST            0x34
#define DA_FLAGS                0x35
#define DA_APS                  0x36
#define DA_CRASH                0x37
#define DA_ENC                  0x38
#define DA_PWM                  0x39
#define DA_DAC                  0x3a
/* Class B */
#define Q_AXIS_I                0x30      // Q axis Current
#define Q_AXIS_I_REF            0x31      // Q axis Current Reference
#define Q_AXIS_I_ERR            0x32      // Q axis Current Control Error
#define D_AXIS_I                0x33      // D axis Current
#define D_AXIS_I_REF            0x34      // D axis Current Reference
#define D_AXIS_I_ERR            0x35      // D axis Current Control Error

#define SPD_SC                  0x30      // Speed on Speed Controller
#define SPD_REF                 0x31      // Speed Reference
#define SPD_ERR                 0x32      // Speed Control Error

#define POS_PC                  0x30      // Position on Position Controller
#define POS_REF                 0x31      // Position Reference
#define POS_ERR                 0x32      // Position Control Error

#define POS_PROFILE             0x30      // Position Reference on PROFILE
#define SPD_PROFILE             0x31      // Speed Reference on PROFILE
#define ACC_PROFILE             0x32      // Acceleration Reference on PROFILE
#define JERK_PROFILE            0x33      // Jerk Reference on PROFILE
#define TCH_PROFILE             0x34      // PROFILE End Status

#define VDSE_INTEG              0x30      // D axis Voltage Integral Term

#define FLAGS_BKO               0x30      // Flag Bko
#define FLAGS_ZSP               0x31      // Flag ZSP

#define POS_APS                 0x30      // Position on APS
#define SPD1_APS                0x31      // Speed1 on APS
#define SPD2_APS                0x32      // Speed2 on APS
#define THETA_APS               0x33      // Theta on APS

#define DIST_UP                 0x30      // UP Distance on Crash Sensor(Laser Sensor)
#define DIST_DOWN               0x31      // DOWN Distance on Crash Sensor(Laser Sensor)

#define POSA_HEIDENHAIN         0x30      // Position A on Heidenhain Encoder
#define POSB_HEIDENHAIN         0x31      // Position B on Heidenhain Encoder
#define POS_F_A_HEIDENHAIN      0x32      // Filtered Position A on Heidenhain Encoder
#define POS_F_B_HEIDENHAIN      0x33      // Filtered Position B on Heidenhain Encoder
#define DIFFPOSAB_HEIDENHAIN    0x34      // Difference of between Postion A and B on Heidenhain Encoder
#define SPDA_HEIDENHAIN         0x35      // Speed A on Heidenhain Encoder
#define SPDB_HEIDENHAIN         0x36      // Speed B on Heidenhain Encoder
#define REALPOSA_HEIDENHAIN     0x37      // Real Position A on Heidenhain Encoder
#define REALPOSB_HEIDENHAIN     0x38      // Real Position B on Heidenhain Encoder
#define THETAA_HEIDENHAIN       0x39      // Theat A on Heidenhain Encoder
#define THETAB_HEIDENHAIN       0x3a      // Theta B on Heidenhain Encoder

#define VA_REF                  0x30      // Phase-A Voltage Reference on PWM
#define VB_REF                  0x31      // Phase-B Voltage Reference on PWM
#define VC_REF                  0x32      // Phase-C Voltage Reference on PWM
#define THETA_INV               0x33      // Inverter Theta

#define GRIDA_REF               0x30      // GRID-A DA Test Signal
#define GRIDB_REF               0x31      // GRID-B DA Test Signal
#define GRIDC_REF               0x32      // GRID-C DA Test Signal
#define GRID_THETA              0x33      // GRID-Theta DA Test Signal

#endif  // end of SNU_CPU1_EX_DAC_H definition


