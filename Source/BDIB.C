/*****************************************************************************
-------------------------------Organization------------------------------------
	Project				: FREEWAY
	Compiler    		: TMS320F28377D C-Compiler v6.1.2(TI v15.12.7.LTS)
	Author				: Inverter part, Future Electrocity Team in Hyudai elevator(with KERI)
	Version				: v1.0
	Last Rev.			: 2018.07.18
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
						: Added SCI Protocol Infmation Response Part (20181107 by GWON)
						: Rearrangement of F/W for FREEWAY (20181112 by GWON)
******************************************************************************/

#include "bdib.h"
#include "Variable.h"
//#include "mstep.h"
#include "SC.h"
//#include "rmb.h"
#include "keymenu.h"

//Bdib		BDIB;
BDIB 			g_BDIB;
Suds			SUDS;
Suds_old		SUDS_OLD;
//extern MStep  	MSTEP;
extern Uns 		DblSpd_Msw_Use;

long	BDIB_RD1[5] = {0,0,0,0,0}, BDIB_RD2[5] = {0,0,0,0,0}, BDIB_RD3[5] = {0,0,0,0,0}, BDIB_RD4[5] = {0,0,0,0,0},SUDS_RD[5] = {0,0,0,0,0} ;
Uint16 	BDIBCnt = 0;
/*******************************************************************/
/* 					BDIB_Create - BDIB °´Ã¼ ÃÊ±âÈ­                                        */
/*******************************************************************/
void BDIB_Create( void )
{
	g_BDIB.regStatus = 0;
//	SUDS.regStatus   = 0;

	g_BDIB.regStatusBit.UP   	= DIB_UP;			// nFWD
	g_BDIB.regStatusBit.DOWN 	= DIB_DOWN;			// nREV
	g_BDIB.regStatusBit.RST  	= DIB_RST;			// nCP_RST
	g_BDIB.regStatusBit.RDY     = DIB_RDY;          // nREADY
	g_BDIB.regStatusBit.AT	 	= DIB_AT;			// nAUTO
	g_BDIB.regStatusBit.XSA	 	= DIB_XSA;			// XSA
	g_BDIB.regStatusBit.XBKAS 	= DIB_XBKAS;		// XBKAS
	g_BDIB.regStatusBit.XBKBS	= DIB_XBKBS;		// XBKBS
	g_BDIB.regStatusBit.XCLI	= DIB_XCLI;			// XCLI
	g_BDIB.regStatusBit.XDLV	= DIB_XDLV;			// XDLV
	g_BDIB.regStatusBit.XDZV	= DIB_XDZV;			// XDZV
	g_BDIB.regStatusBit.XHCLI	= DIB_XHCLI;		// XHCLI
	g_BDIB.regStatusBit.XULV	= DIB_XULV;			// XULV

	/*if(HHT_Use_Flg == 0)
	{
		BDIB_UP = (int)DIB_UP;		BDIB_DOWN = (int)DIB_DOWN;
		if(RMB_SpdPtnSel == ST12_ES_SPD_PTN)
		{
			BDIB_RST	= (int)DIB_RST;		BDIB_CP_AT 	= 0;				BDIB_INI	= 0;		BDIB_RDY 	= 0;
			BDIB_SUF	= 0;				BDIB_X1 	= (int)(!DIB_PLUL);	BDIB_PLUH 	= 0;		BDIB_X3 	= (int)(!DIB_PLUH);
			BDIB_PLDH	= 0;				BDIB_PLDM 	= 0;				BDIB_SDF 	= 0;		BDIB_PLUM 	= 0;
			BDIB_ENABLE = (int)(!DIB_PLUM);	BDIB_PLUH2 	= 0;				BDIB_PLDH2 	= 0;
			BDIB_ULS	= 0;				BDIB_DLS 	= 0;				BDIB_XAUTO 	= 0;		BDIB_XMCS 	= 0;
			BDIB_X29 	= 0;				BDIB_XMC2 	= 0;				BDIB_XBKAS 	= 1;		BDIB_XBKBS 	= 1;
			BDIB_HOU 	= 0;				BDIB_HOD 	= 0;				BDIB_DZ 	= 0;		BDIB_X2 	= (int)DIB_XDZ;
			BDIB_MCC 	= 0;				BDIB_FUSC 	= 0;				BDIB_INP1 	= 0;		BDIB_INP2 	= 0;
			BDIB_INP3 	= 0;
			BDIB_SUS4 	= 0;				BDIB_SUS5 	= 0;				BDIB_SUS6 	= 0;		BDIB_SDS4 	= 0;
			BDIB_SDS5 	= 0;				BDIB_SDS6 	= 0;

		}else if(RMB_SpdPtnSel == ST12_SPD_PTN)
		{
			BDIB_RST 	= (int)DIB_RST;	BDIB_CP_AT 	= 0;				BDIB_INI 	= 0;	BDIB_RDY 	= 0;
			BDIB_SUF 	= 0;			BDIB_X1 	= (int)(!DIB_PLUL);	BDIB_PLUH 	= 0;	BDIB_X3 	= (int)(!DIB_PLUH);
			BDIB_PLDH 	= 0;			BDIB_PLDM 	= 0;				BDIB_SDF 	= 0;	BDIB_PLUM 	= 0;
			BDIB_ENABLE = 0;			BDIB_PLUH2 	= 0;				BDIB_PLDH2 	= 0;
			BDIB_ULS 	= 0;			BDIB_DLS 	= 0;				BDIB_XAUTO 	= 0;	BDIB_XMCS 	=	0;
			BDIB_X29 	= 0;			BDIB_XMC2 	= 0;				BDIB_XBKAS 	= 1;	BDIB_XBKBS 	= 1;
			BDIB_HOU 	= 0;			BDIB_HOD 	= 0;				BDIB_DZ 	= 0;	BDIB_X2 	= (int)DIB_XDZ;
			BDIB_MCC 	= 0;			BDIB_FUSC 	= 0;				BDIB_INP1 	= 0;	BDIB_INP2 	= 0;
			BDIB_INP3 	= 0;
			BDIB_SUS4 	= 0;			BDIB_SUS5 	= 0;				BDIB_SUS6 	= 0;	BDIB_SDS4 	= 0;
			BDIB_SDS5 	= 0;			BDIB_SDS6 	= 0;

		}else
		{
			BDIB_RST 	= (int)DIB_RST;		BDIB_RDY 	= (int)DIB_RDY;		BDIB_CP_AT 	= (int)DIB_AT;
			BDIB_X1 	= 0;				BDIB_X2 	= 0;				BDIB_X3 	= 0;				BDIB_ENABLE = 0;
			BDIB_SUF 	= (int)DIB_PLUL;	BDIB_PLUM = (int)DIB_PLUM;	BDIB_PLUH 	= (int)DIB_PLUH;	BDIB_PLUH2 	= (int)DIB_PLUH2;
			BDIB_SDF 	= (int)DIB_PLDL;	BDIB_PLDM = (int)DIB_PLDM;	BDIB_PLDH 	= (int)DIB_PLDH;	BDIB_PLDH2 	= (int)DIB_PLDH2;
			BDIB_XAUTO= (int)DIB_XAUTO;	BDIB_XMCS = (int)DIB_XMCS;	BDIB_X29 	= (int)DIB_X29;		BDIB_XMC2 	= 0;(int)DIB_XMC2;
			BDIB_XBKAS 	= (int)DIB_XBKAS;	BDIB_XBKBS 	= (int)DIB_XBKBS;	BDIB_ULS 	= (int)DIB_XULS;	BDIB_DLS 	= (int)DIB_XDLS;
			BDIB_HOU 	= (int)DIB_XULA;	BDIB_HOD 	= (int)DIB_XDLA;	BDIB_DZ 	= (int)DIB_XDZ;		BDIB_MCC 	= (int)DIB_XMCC;
			BDIB_FUSC= (int)DIB_XFUSC;	BDIB_INP1 = (int)DIB_XSPI1;	BDIB_INP2 = (int)DIB_XSPI2;	BDIB_INP3 = (int)DIB_XSPI3;
			BDIB_SUS4 = (int)DIB_XSUS4;	BDIB_SUS5 = (int)DIB_XSUS5;	BDIB_SUS6 = (int)DIB_XSUS6; 	BDIB_SDS4 = (int)DIB_XSDS4;
			BDIB_SDS5= (int)DIB_XSDS5;	BDIB_SDS6 = (int)DIB_XSDS6;

			if(RMB_SpdPtnSel == OPTIMAL_SPD_PTN){
				BDIB_PLUM = (int)BDIB_SUF;
				BDIB_PLDM = (int)BDIB_SDF;
			}
			else{
				BDIB_PLUM = (int)DIB_PLUM;
				BDIB_PLDM = (int)DIB_PLDM;
			}
		}
	}
	else
	{
		BDIB_RST 	= 0;	BDIB_CP_AT 	= 0;	BDIB_INI 	= 0;	BDIB_RDY 	= 0;
		BDIB_SUF 	= 0;	BDIB_X1 	= 0;	BDIB_PLUH 	= 0;	BDIB_X3 	= 0;
		BDIB_PLDH 	= 0;	BDIB_PLDM 	= 0;	BDIB_SDF 	= 0;	BDIB_PLUM 	= 0;
		BDIB_ENABLE = 0;	BDIB_PLUH2 	= 0;	BDIB_PLDH2 	= 0;
		BDIB_ULS 	= 0;	BDIB_DLS 	= 0;	BDIB_XAUTO 	= 0;	BDIB_XMCS 	= 0;
		BDIB_X29 	= 0;	BDIB_XMC2 	= 0;	BDIB_XBKAS 	= 1;	BDIB_XBKBS 	= 1;
		BDIB_HOU 	= 0;	BDIB_HOD 	= 0;	BDIB_DZ 	= 0;	BDIB_X2 	= 0;
		BDIB_MCC 	= 0;	BDIB_FUSC 	= 0;	BDIB_INP1 	= 0;	BDIB_INP2 	= 0;
		BDIB_INP3 	= 0;
		BDIB_SUS4 	= 0;	BDIB_SUS5 	= 0;	BDIB_SUS6 	= 0;	BDIB_SDS4 	= 0;
		BDIB_SDS5 	= 0;	BDIB_SDS6 	= 0;

		if(RMB_SpdPtnSel == ST12_ES_SPD_PTN) BDIB_ENABLE = 1;
		else BDIB_ENABLE = 0;
	}*/
}
/*******************************************************************/
/* BDIB_UpdateAllStatus - BDIB Register( ) Value Read		   */
/*******************************************************************/
void BDIB_UpdateAllStatus(void)
{
	g_BDIB.regStatus = 0;
//	SUDS.regStatus   = 0;

	g_BDIB.regStatusBit.UP   	= DIB_UP;			// nFWD
	g_BDIB.regStatusBit.DOWN 	= DIB_DOWN;			// nREV
	g_BDIB.regStatusBit.RST  	= DIB_RST;			// nCP_RST
	g_BDIB.regStatusBit.RDY     = DIB_RDY;          // nREADY
	g_BDIB.regStatusBit.AT	 	= DIB_AT;			// nAUTO
	g_BDIB.regStatusBit.XSA	 	= DIB_XSA;			// XSA
	g_BDIB.regStatusBit.XBKAS 	= DIB_XBKAS;		// XBKAS
	g_BDIB.regStatusBit.XBKBS	= DIB_XBKBS;		// XBKBS
	g_BDIB.regStatusBit.XCLI	= DIB_XCLI;			// XCLI
	g_BDIB.regStatusBit.XDLV	= DIB_XDLV;			// XDLV
	g_BDIB.regStatusBit.XDZV	= DIB_XDZV;			// XDZV
	g_BDIB.regStatusBit.XHCLI	= DIB_XHCLI;		// XHCLI
	g_BDIB.regStatusBit.XULV	= DIB_XULV;			// XULV

	/*long tmp1BDIB = 0, tmp2BDIB = 0;
	if(HHT_Use_Flg == 0)
	{
		tmp1BDIB = GpioDataRegs.GPADAT.all & BDIB_USEBIT_A;
		NOP();
		tmp2BDIB = GpioDataRegs.GPADAT.all & BDIB_USEBIT_A;

		if(tmp1BDIB == tmp2BDIB)			BDIB_RD1[BDIBCnt] = tmp1BDIB ;

		if(BDIB_RD1[0]==BDIB_RD1[1])
		{
			BDIB_UP = (int)DIB_UP;		BDIB_DOWN = (int)DIB_DOWN;
			if((RMB_SpdPtnSel == ST12_ES_SPD_PTN)||(RMB_SpdPtnSel == ST12_SPD_PTN))
			{
				BDIB_RST 	= (int)DIB_RST;	BDIB_CP_AT 	= 0;				BDIB_INI 	= 0;	BDIB_RDY 	= 0;
				BDIB_SUF 	= 0;			BDIB_X1 	= (int)(!DIB_PLUL);	BDIB_PLUH 	= 0;	BDIB_X3 	= (int)(!DIB_PLUH);
				BDIB_PLDH 	= 0;			BDIB_PLDM 	= 0;				BDIB_SDF 	= 0;	BDIB_PLUM 	= 0;

				if(RMB_SpdPtnSel == ST12_ES_SPD_PTN) BDIB_ENABLE = (int)(!DIB_PLUM);
				else BDIB_ENABLE = 0;
				BDIB_HOD = 0;
			}else
			{
				BDIB_RST	= (int)DIB_RST;		BDIB_RDY 	= (int)DIB_RDY;		BDIB_CP_AT 	= (int)DIB_AT;
				BDIB_X1 	= 0;				BDIB_X2 	= 0;				BDIB_X3 	= 0;				BDIB_ENABLE = 0;
				BDIB_SUF 	= (int)DIB_PLUL;	BDIB_PLUM = (int)DIB_PLUM;	BDIB_PLUH 	= (int)DIB_PLUH;
				BDIB_SDF 	= (int)DIB_PLDL;	BDIB_PLDM = (int)DIB_PLDM;	BDIB_PLDH 	= (int)DIB_PLDH;
				BDIB_HOD 	= (int)DIB_XDLA;

				if((RMB_SpdPtnSel == OPTIMAL_SPD_PTN)&&(SUDS_SW_NUM >= 5)){
					BDIB_PLUM = (int)DIB_PLUL;
					BDIB_PLDM = (int)DIB_PLDL;
				}
				else{
					BDIB_PLUM = (int)DIB_PLUM;
					BDIB_PLDM = (int)DIB_PLDM;
				}
			}
		}

		tmp1BDIB = GpioDataRegs.GPBDAT.all & BDIB_USEBIT_B;
		NOP();
		tmp2BDIB = GpioDataRegs.GPBDAT.all & BDIB_USEBIT_B;

		if(tmp1BDIB == tmp2BDIB)			BDIB_RD2[BDIBCnt] = tmp1BDIB ;

		if(BDIB_RD2[0]==BDIB_RD2[1])
		{
			if((RMB_SpdPtnSel == ST12_ES_SPD_PTN)||(RMB_SpdPtnSel == ST12_SPD_PTN))
			{
				BDIB_PLUH2	= 0;	BDIB_PLDH2 	= 0;
				BDIB_ULS 	= 0;	BDIB_DLS 	= 0;	BDIB_XAUTO 	= 0;	BDIB_XMCS 	= 0;
				BDIB_X29 	= 0;	BDIB_XMC2 	= 0;	BDIB_XBKAS 	= 1;	BDIB_XBKBS 	= 1;

			}else
			{
				BDIB_PLUH2 	= (int)DIB_PLUH2;	BDIB_PLDH2 	= (int)DIB_PLDH2;
				BDIB_XAUTO= (int)DIB_XAUTO;	BDIB_XMCS = (int)DIB_XMCS;	BDIB_X29	= (int)DIB_X29;		BDIB_XMC2 	= 0;(int)DIB_XMC2;
				BDIB_XBKAS 	= (int)DIB_XBKAS;	BDIB_XBKBS 	= (int)DIB_XBKBS;	BDIB_ULS 	= (int)DIB_XULS;	BDIB_DLS 	= (int)DIB_XDLS;
			}
		}

		tmp1BDIB = GpioDataRegs.GPEDAT.all & BDIB_USEBIT_E;
		NOP();
		tmp2BDIB = GpioDataRegs.GPEDAT.all & BDIB_USEBIT_E;

		if(tmp1BDIB == tmp2BDIB)			BDIB_RD3[BDIBCnt] = tmp1BDIB ;

		if(BDIB_RD3[0]==BDIB_RD3[1])
		{
			if((RMB_SpdPtnSel == ST12_ES_SPD_PTN)||(RMB_SpdPtnSel == ST12_SPD_PTN))
			{
				BDIB_HOU 	= 0;	BDIB_DZ 	= 0;	BDIB_X2 	= (int)DIB_XDZ;
				BDIB_MCC 	= 0;	BDIB_SUS4 	= 0;	BDIB_SUS5 	= 0;

			}else
			{
				BDIB_HOU 	= (int)DIB_XULA;	BDIB_DZ 	= (int)DIB_XDZ;		BDIB_MCC 	= (int)DIB_XMCC;	BDIB_X2 	= 0;
				BDIB_SUS4 = (int)DIB_XSUS4;	BDIB_SUS5 = (int)DIB_XSUS5;

			}
		}

		tmp1BDIB = GpioDataRegs.GPFDAT.all & BDIB_USEBIT_F;
		NOP();
		tmp2BDIB = GpioDataRegs.GPFDAT.all & BDIB_USEBIT_F;

		if(tmp1BDIB == tmp2BDIB)			BDIB_RD4[BDIBCnt] = tmp1BDIB ;

		if(BDIB_RD4[0]==BDIB_RD4[1])
		{
			if((RMB_SpdPtnSel == ST12_ES_SPD_PTN)||(RMB_SpdPtnSel == ST12_SPD_PTN))
			{
				BDIB_FUSC 	= 0;	BDIB_INP1 	= 0;	BDIB_INP2 	= 0;	BDIB_INP3 	= 0;
				BDIB_SUS6 	= 0;	BDIB_SDS4 	= 0;	BDIB_SDS5 	= 0;	BDIB_SDS6 	= 0;

			}else
			{
				BDIB_FUSC= (int)DIB_XFUSC;	BDIB_INP1 = (int)DIB_XSPI1;	BDIB_INP2 = (int)DIB_XSPI2;	BDIB_INP3 = (int)DIB_XSPI3;
				BDIB_SUS6 = (int)DIB_XSUS6;	BDIB_SDS4 = (int)DIB_XSDS4;	BDIB_SDS5= (int)DIB_XSDS5;	BDIB_SDS6 = (int)DIB_XSDS6;
			}
		}

		SUDS.regStatus = (((BDIB_SUF)|(BDIB_PLUM<<1)|(BDIB_PLUH<<2)|(BDIB_PLUH2<<3)|(BDIB_SUS4<<4)|(BDIB_SUS5<<5)|(BDIB_SUS6<<6)|(BDIB_SDF<<7)|(BDIB_PLDM<<8)|(BDIB_PLDH<<9)|(BDIB_PLDH2<<10)|(BDIB_SDS4<<11)|(BDIB_SDS5<<12)|(BDIB_SDS6<<13)) & SUDS_SW_BIT) ;	//SUDS
	}else
	{
		if(RMB_SpdPtnSel == ST12_ES_SPD_PTN) BDIB_ENABLE = 1;
		else BDIB_ENABLE = 0;
	}
	if((Fwd_Rev_Test == 1)&&((BDIB_UP == 1)||(BDIB_DOWN == 1)))
	{
		switch ( MSTEP.uCommand )
		{
			case MSTEP_INITIAL:
				if(((INV1_SC.Wrpm_ref) >= MSTEP.fInitial))
				{
					BDIB_UP 	= 0 ;
					BDIB_DOWN 	= 1 ;
				}
				if(((INV1_SC.Wrpm_ref) <= -MSTEP.fInitial))
				{
					BDIB_UP 	= 1 ;
					BDIB_DOWN 	= 0 ;
				}
				break;
			case MSTEP_CREEP:
				if(((INV1_SC.Wrpm_ref) >= MSTEP.fCreep))
				{
					BDIB_UP 	= 0 ;
					BDIB_DOWN 	= 1 ;
				}
				if(((INV1_SC.Wrpm_ref) <= -MSTEP.fCreep))
				{
					BDIB_UP 	= 1 ;
					BDIB_DOWN 	= 0 ;
				}
				break;
			case MSTEP_INSPECTION:
				if(((INV1_SC.Wrpm_ref) >= MSTEP.fInspection))
				{
					BDIB_UP 	= 0 ;
					BDIB_DOWN 	= 1 ;
				}
				if(((INV1_SC.Wrpm_ref) <= -MSTEP.fInspection))
				{
					BDIB_UP 	= 1 ;
					BDIB_DOWN 	= 0 ;
				}
				break;
			case MSTEP_SHORTRUN:
				if(((INV1_SC.Wrpm_ref) >= MSTEP.fShortRun))
				{
					BDIB_UP 	= 0 ;
					BDIB_DOWN 	= 1 ;
				}
				if(((INV1_SC.Wrpm_ref) <= -MSTEP.fShortRun))
				{
					BDIB_UP 	= 1 ;
					BDIB_DOWN 	= 0 ;
				}
				break;
			case MSTEP_RELEVEL:
				if(((INV1_SC.Wrpm_ref) >= MSTEP.fRelevel))
				{
					BDIB_UP 	= 0 ;
					BDIB_DOWN 	= 1 ;
				}
				if(((INV1_SC.Wrpm_ref) <= -MSTEP.fRelevel))
				{
					BDIB_UP 	= 1 ;
					BDIB_DOWN 	= 0 ;
				}
				break;
			case MSTEP_LONGRUN:
				if(((INV1_SC.Wrpm_ref) >= MSTEP.fLongRun))
				{
					BDIB_UP 	= 0 ;
					BDIB_DOWN 	= 1 ;
				}
				if(((INV1_SC.Wrpm_ref) <= -MSTEP.fLongRun))
				{
					BDIB_UP 	= 1 ;
					BDIB_DOWN 	= 0 ;
				}
				break;
			default:
				break;
		}
	}

	BDIBCnt ++ ;
	if(BDIBCnt>2)	BDIBCnt = 0 ;*/
}

