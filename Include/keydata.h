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
******************************************************************************/
#include "keymenu.h"
#include "variable.h"

#if CN_MOTOR_DATA
#define SYNMOTORSELECTNUM		43
#else
#define SYNMOTORSELECTNUM		56
#define	LINEARMOTORSELECTNUM	2
#endif

ITEMBASIC BasicList[BASICNUM] = {
#if CN_MOTOR_DATA
	{"FREEWAY Inverter 0.1CN", "",	0.0, 0.01},
#else	
	{"FREEWAY INV 0.2KR", "",	0.0, 0.01},
#endif
	{"SPEED:      ",	"m",	0.0, 0.1},
	{"SPEED FBK:  ",	"m",	0.0, 0.1},
	{"SPEED REF:  ",	"m",	0.0, 0.1},
	{"INV.CURRENT:",	"A",	0.0, 0.1},
	{"INV.VOLTAGE:",	"V",	0.0, 0.1},
	{"CON.CURRENT:",	"A",	0.0, 0.1},
	{"INPUT VOLT: ",	"V",	0.0, 0.1},
	{"DCLINK VOLT:",	"V",	0.0, 0.1},
	{"ROTOR POS:  ",	"V",	0.0, 0.1},
	{"LOAD PULSE: ",	" ",	0.0, 0.1},
	{"IU CURRENT: ",	"A",	0.0, 0.01},
	{"IV CURRENT: ",	"A",	0.0, 0.01},
	{"IW CURRENT: ",	"A",	0.0, 0.01},
	{"FLOOR:      ",	"F",	0.0, 1.0},
	{"CALL:       ",    "F",	0.0, 1.0},
	{"DRIVE MODE: ",    " ",	0.0, 1.0},
	{"COM STATUS: ",    " ",	0.0, 1.0},
	{"DECEL FLAG: ",    " ",	0.0, 1.0},
	{"INITIAL:    ",    " ",	0.0, 1.0},
	{"PLDL POS :  ",    " ",	0.0, 1.0}
};

ITEMDISPLAY IOList[IONUM] = {
	{"  INPUT         ",	0.0},
	{"  OUTPUT        ",	0.0},
	{"  SUS-SDS INPUT ",	0.0},
	{"  UART RX1      ",	0.0},
	{"  UART RX2      ",	0.0},
	{"  UART TX1      ",	0.0},
	{"  UART TX2      ",	0.0}
};

char *ErrorList[ERRORNUM]={		
	"NONE              ",	"CON OCA H/W      ",	"INV OCA H/W     ",	"CON OCB H/W      ",	"INV OCB H/W     ",
	"CON OCC H/W       ",	"INV OCC H/W      ",	"OVDC FAULT H/W  ",	"ESS OC H/W       ",	"ESS OV H/W      ",	
	"CON MOSFET FAULT  ",	"INV MOSFET FAULT ",	"ESS IGBT FAULT  ",	"15V POW FAULT    ",	"5V POW FAULT    ", 
	"DCC FAULT         ",	"ENCODER FAULT    ",	"INV THERM ERR   ",	"LMT SW FAULT     ",	"OVER SPEED FAULT", 
	"DLA FAULT         ",	"MC1 FAULT        ",	"INV OVER LOAD   ",	"VERSION ERR      ",	"CON OCA S/W     ", 	
	"INV OCA S/W       ",	"CON OCB S/W      ",	"INV OCB S/W     ",	"CON OCC S/W      ",	"INV OCC S/W     ",	
	"GRID OVBC S/W     ",	"GRID OVAB S/W    ",	"OVDC FAULT S/W  ",	"ESS OC S/W       ",	"ESS OV S/W      ",	
	"GRID SEQ ERR      ",	"UVDC FAULT S/W   ",	"BK SW FAULT     ",	"ENC UVW SEQ      ",	"SPD DISAGR      ",	
	"ANGLE OVER        ",	"PENC UVW ERR     ",	"OUTPUT ERR      ",	"FR ERR           ",	"CP WATCHDOG     ", 
	"CP MODE FAULT     ",	"OVH FUS FAULT    ",	"I EARTH ERR     ",	"C EARTH ERR      ",	"CMD OFF ERR     ",	
	"SENC UVW ERR      ",	"CMD CFM ERR      ",	"AD OFFSET ERR   ",	"ENABLE ERR       ",	"GRID PLL FREQ.  ", 
	"GRID PLL OV FLT   ",	"GRID PLL UV FLT  ",	"FRAM FAULT      ",	"CC INV OCS S/W   ",	"CC CON OCS S/W  ",
	"CC ESS OCS S/W    ",	"CC OVDC S/W      ",	"CC ESS OVDC S/W ",	"ESS OUTPUT ERR   ",	"ESS VOLT SENSE  ", 
	"CP COM. FAULT     ",	"OBS OS FAULT     ",	"GRID PLL RUN FLT",	"ESS WRN FAULT    ",	"UNKNOWN ERR      "
};		/* ERRORNUM = NONE + 58 + UNKNOWN */

ITEM ControlList[CONTROLNUM] = {			
	{"E/L SPEED       ",	{15.0,		240.0},				" m/m",			60.0,		1.0},
	{"MAX SPEED       ",	{0.1,		1.0},				" m/s",			1.0,		0.01},
	{"MAX ACCEL       ",	{0.1,		1.0},				" m/s2",		0.4,		0.01},
	{"MAX JERK        ",	{0.1,		1.0},				" m/s3",		0.4,		0.01},
	{"INSPECT SPEED   ",	{0.1,		1.0},				" m/s",			0.15,		0.01},
	{"CREEP SPEED     ",    {0.01,      1.0},               " m/s",         0.02,       0.01},
	{"RELEVEL SPEED   ",    {0.01,      1.0},               " m/s",         0.03,       0.01},
	{"ANTISTALL TIME  ",    {0.1,       60.0},              " sec",         1.0,        0.1},
	{"MAX  FLOOR      ",	{0.0,		200.0},		    	" flr",			24.0,		1.0},
	{"FWD DIRECTION   ",	{0.0,		UPDNSELECTNUM-1},	"   ",			0.0,		1.0},	
	{"SC JM           ",	{0.1,		1000.0},			" kg-m2",		50.0,		1.0},
	{"SC FEED FWD GAIN",	{0.0,		2000.0},			"   ",			0.5,		0.1},
	{"INIT START      ",	{ 0,		ONOFFSELECTNUM-1},	"  ",			0.0,	  	1.0},
	{"CURRENT POSTION ",    {-10000.0,  10000.0},           " um",          0.0,        0.1},
	{"CAR TYPE        ",    {0.0,       2.0},               "   ",          1.0,        1.0},
	{"MAX LENGTH      ",    {0,         100.0},             " m",           2.3,        0.1},
	{"ZERO CAL        ",    {0,         593.75},            " mm",          0.0,        0.01},
	{"ACCUM           ",    {0,         10000.0},           "   ",          0.0,        1.0},
	{"SEL POS         ",    {0,         593.75},            " mm",          0.0,        0.01}
};		

ITEM InterfaceList[INTERFACENUM] = {
	{"ZERO SPD LVL    ",	{0.0,		100.0},				" r/m",		0.5,	0.1},	
	{"SPD CTL LEVEL   ",	{1.0,		200.0},				" %",		30.0,	1.0},
	{"AO1 SELECT      ",	{ 0, 	AOSELECTNUM-1},		    " ",	 	0.0,	1.0},	
	{"AO2 SELECT      ",	{ 0, 	AOSELECTNUM-1},		    " ",	 	1.0,	1.0},	
	{"AO3 SELECT      ",	{ 0, 	AOSELECTNUM-1},		    " ",	 	2.0,	1.0},
	{"AO4 SELECT      ",	{ 0, 	AOSELECTNUM-1},		    " ",	 	3.0,	1.0},
	{"AO 1 CENTER     ",	{-10000.0, 	10000.0},	        " ",	 	0.0,	1.0},	
	{"AO 1 RANGE      ",	{-10000.0, 	10000.0},	        " ",	 	200.0,	1.0},	
	{"AO 2 CENTER     ",	{-10000.0, 	10000.0},	        " ",	 	0.0,	1.0},	
	{"AO 2 RANGE      ",	{-10000.0, 	10000.0},	        " ",	 	200.0,	1.0},
	{"AO 3 CENTER     ",	{-10000.0, 	10000.0},	        " ",	 	0.0,	1.0},
	{"AO 3 RANGE      ",	{-10000.0, 	10000.0},	        " ",	 	50.0,	1.0},
	{"AO 4 CENTER     ",	{-10000.0, 	10000.0},	        " ",	 	0.0,	1.0},	
	{"AO 4 RANGE      ",	{-10000.0, 	10000.0},	        " ",	 	50.0,	1.0}
};		

/***************************************************************************************** 
If you modify of inserte FACTORY LIST, You hava to change FACTORYNUM in "KeyMenu.h" and 
FACTORY_ITEM_LIST in "KeyMenu.h". 
After that, you should modify or insert blow list at proper positions.
(20171206 by GWON)
******************************************************************************************/
ITEM FactoryList[FACTORYNUM] = {	
	{"INVERTER CAPACIT",	{0.0, 		INVERTERSELECTNUM-1},	"   ",		6.0,		1.0},
	{"MOTOR TYPE?     ",	{0.0, 		MOTORTYPESELECTNUM-1},	"   ",		0.0,		0.0},
	{"INVERTER CTL MOD",	{0.0, 		INVCTLMODENUM-1},		"   ",		0.0,		1.0},
	{"INPUT VOLTAGE   ",	{200.0,		1000.0},				" Vdc",		600.0,		10.0},
	{"THRUST LIMIT    ",	{100.0,		350.0},					" %",		250.0,		1.0},
	{"SCALE DC VOLT   ",	{0.0, 		50.0},					"   ",		29.93,		0.01},
	{"SCALE CURRENT   ",	{0.0,		200.0},					"   ",		11.17,		0.01},
	{"INVERTER WCC    ",	{10.0,		800.0},					" Hz",		400.0,		1.0},
	{"SC WSC          ",	{1.0,		100.0},					" Hz",		2.0,		0.1},
	{"PC WPC          ",	{1.0,		100.0},					" Hz",		2.0,		0.1},
	{"INV.OC LEVEL    ",	{10.0,		1000.0},				" Apk",		270.0,		5.0},
	{"OVER SPD LEVEL  ",	{0.0,		2.0},					" m/s",		1.0,		0.1},
	{"ENCODER DIR     ",	{0.0,		1.0},					"   ",		0.0,		1.0},
	{"VDC CAPACITANCE ",	{0.0,		20000.0},				" uF",		200.0,		10.0},
	{"ANGLE RST VOLT  ", 	{0.0,		200.0},	 				" Vpk",		100.0,		1.0},
	{"ENCODER TYPE    ",	{0,			ENCODERTYPENUM-1},	    " ",		1.0,	 	1.0},	
	{"ERROR ERASE     ",	{0.0,		ONOFFSELECTNUM-1},	 	"   ",		0.0,		1.0},
	{"PROG VERSION    ",	{0.0,		100.0},					"   ",		1.1,		0.1},	
	{"FRAM VERSION    ",	{0.0,		100.0},					"   ",		1.0,		0.1}
};		

char* TQBIASSELECT[TQSELECTNUM] =	{
 "             No use ",	"       Auto Control ",	"    Auto + LS Board ",	"      LS Board type ",	"          LV12 type ",	"      LM Board type "
};
	
char* INVCTLMODESEL[INVCTLMODENUM] =	{
	"     Sensed_IV Mode ",	"     Hybrid_IV Mode ",	"   Senless_V/F Mode ",	"    Senless_DV Mode ",	"      AutoTune Mode "
};

char* DRIVEMODE[DRIVEMODENUM] =		
{"     AUTO",	"  INITIAL", 	"  INSPECT", 	"      PLO", "  RELEVEL", 
 "60M/M DRV", 	"LOW SPEED", 	" ELD DRIVE", 	"RES CREEP", "  RES PLO", 
 "  RESESUE", 	"    CREEP",  	"    STEP",	" AUTO", " ZERO SPD",
 " SPEC DRV",	"     STOP"};	 

char* OKFAILSELECT[OKFAILSELECTNUM] =		
{"     FAIL",					"       OK"	};

char* ELDSPDSELECT[ELDSPDSELECTNUM] =		
{"              CREEP ",					"            12[M/M] "	};
	
char* MANSPDSEL[MANSPDSELNUM] = {
	"               Stop","            Inspect","          Short Run","           Long Run","              Creep",		
	"        Flr Initial","        Flr arrange","           Zero Spd"			
};

char* MANMODESEL[MANMODESELNUM] = {	// Added for FREEWAY 20180212 by GWON
	"               Stop","            MANUAL","            AUTO","            STEP"			
};

char* THREESELECT[THREESELECTNUM] =		
{"                REV ","                FWD ","             NO USE "};	 

char* TQRIPPLESELECT[TQRIPPLESELECTNUM] =	{
 "             No use ",	"     6TH Q-x TQCOMP ",	"     6TH D-x TQCOMP ",	"    6TH DQ-x TQCOMP ",		"     360 Q-x TQCOMP ",	"     360 D-x TQCOMP ", "    360 DQ-x TQCOMP "
};

char* INVERTERSELECT[INVERTERSELECTNUM] =	{
	"        WB210 5.5kW","        WB210 7.5kW","         WB210 11kW","         WB210 15kW","         WB150 22kW",
	"         WB150 30kW","         WB150 40kW","         WB150 50kW","      WB150SSM 22kW","      WB150SSM 30kW",
  	"      WB150SSM 40kW","   WB150SSM_HY 40kW","      WB150SSF 22kW","      WB150SSF 30kW","         UELCO 40kW",
  	"         UELCO 11kW","      FREEWAY 30kVA","     Other Inverter"
};

#if CN_MOTOR_DATA
ITEM SynMotorList[SYNMOTORNUM] = {		
  {"MOTOR SELECT    ",	{0.0, SYNMOTORSELECTNUM-1},		"   ",		22.0,		1.0},	
  {"MOTOR CAPACIT   ",	{0.1, 	300.0},					" kW",		11.2,		0.1},
  {"MOT RATING V    ",	{80.0, 	480.0},		        	" Vrms",	335.0,		1.0},
  {"MOT RATING A    ",	{0.1, 	200.0},					" Arms",	25.5,		0.1},	
  {"MOTOR POLES     ",	{2.0,		50.0},				" pole",	24.0,		2.0},	
  {"ENCODER PPR     ",	{512.0, 32768.0},				" p/r",		32768.0,	1.0},
  {"WRPM BASE       ",	{0.0, 	3200.0},				" r/m",		95.0,		1.0},
  {"IQSE RATE       ",	{0.0, 	200.0},					" Apk",		36.0,		0.1},
  {"KNOW U ANGLE?   ",	{0.0,	YESNOSELECTNUM-1},		"   ",		1.0,	  	1.0},	
  {"MOTOR U ANGLE   ",	{-3.14,	3.14},		        	" rad",		0.0,	  	0.01},
  {"ANGLE METHOD    ",	{0.0,	ROTORPOSESTISELNUM-1},	"   ",		1.0,	  	1.0},	
  {"SIG INJ.TIME    ",	{50.0, 	50000.0},				" ms",   	5000.0,		10.0},
  {"MOTOR LS        ",	{0.0, 	500.0},					" mH",		29.760,		0.001},	
  {"MOTOR RS        ",	{0.0, 	200.0},					" Ohm",		0.682,		0.001},	
  {"MOTOR KE        ",	{0.0, 	5.0},					"   ",		1.62,	  	0.01},	
  {"M SHEAVE DIA    ",	{0.0, 	1000.0},		      	" mm",		406.0,		10.0}
};	

char* SYNMOTORSELECT[SYNMOTORSELECTNUM] =	{	
	"      WYJ250-10-630","      WYJ250-15-630","      WYJ250-10-800","      WYJ250-15-800","    WYJ250-17.5-800",
	"      WYJ250-20-800","      WYJ250-25-800","     WYJ250-10-1000","     WYJ250-15-1000","   WYJ250-17.5-1000",
	"     WYJ250-20-1000","     WYJ250-25-1000","     WYJ250-10-1250","     WYJ250-15-1250","   WYJ250-17.5-1250",
	"     WYJ250-20-1250","     WYJ250-25-1250","    WTYF250-10-1350","    WTYF250-16-1350","   WTYF250-17.5-135",
	"    WTYF250-20-1350","    WTYF250-25-1350","    WTYF250-10-1600","    WTYF250-16-1600","   WTYF250-17.5-160",
	"    WTYF250-20-1600","    WTYF250-25-1600","    WTYF250-10-2000","    WTYF250-20-2000","    WTYF250-10-2500",									
  	"    WTYF250-16-2500","    GE320-550-0.5/1","    GE630/800-0.5/1","     GE800-1.5/1.75","     GE1000-0.5/1.0",	
  	"    GE1000-1.5/1.75","     GE1150-0.5/1.0","    GE1150-1.5/1.75","     GE1350-0.5/1.0","    GE1350-1.5/1.75",
  	"     GE1600-0.5/1.0","    GE1600-1.5/1.75","        OTHER MOTOR"
};
#else
//ITEM SynMotorList[SYNMOTORNUM] = {
//	{"MOTOR SELECT    ",	{0.0, SYNMOTORSELECTNUM-1},		"   ",		52,			1.0},
//	{"MOTOR CAPACIT   ",	{0.1, 	300.0},					" kW",		39.2,		0.1},
//	{"MOT RATING V    ",	{80.0, 	480.0},		        	" Vrms",	373.0,		1.0},
//	{"MOT RATING A    ",	{0.1, 	200.0},					" Arms",	67.0,		0.1},
//	{"MOTOR POLES     ",	{2.0,	50.0},					" pole",	48.0,		2.0},	//5
//	{"ENCODER PPR     ",	{512.0, 32768.0},				" p/r",		32768.0,	1.0},
//	{"WRPM BASE       ",	{0.0, 	3200.0},				" r/m",		127.3,		1.0},
//	{"IQSE RATE       ",	{0.0, 	200.0},					" Apk",		94.8,		0.1},
//	{"KNOW U ANGLE?   ",	{0.0,	YESNOSELECTNUM-1},		"   ",		1.0,	  	1.0},
//	{"MOTOR U ANGLE   ",	{-3.14,	3.14},		        	" rad",		0.0,	  	0.01},	//10
//	{"ANGLE METHOD    ",	{0.0,	ROTORPOSESTISELNUM-1},	"   ",		1.0,	  	1.0},
//	{"SIG INJ.TIME    ",	{50.0, 	50000.0},				" ms",   	3000.0,		10.0},
//	{"MOTOR LS        ",	{0.0, 	500.0},					" mH",		2.79,		0.01},
//	{"MOTOR RS        ",	{0.0, 	200.0},					" Ohm",		0.080,		0.001},
//	{"MOTOR KE        ",	{0.0, 	5.0},					"   ",		0.883,	  	0.001},	//15
//	{"M SHEAVE DIA    ",	{0.0, 	1000.0},		      	" mm",		600.0,		10.0}
//};

ITEM LinearMotorList[LINEARMOTORNUM] = {		
	{"MOTOR SELECT    ",	{0.0, LINEARMOTORSELECTNUM-1},	"   ",		0,			1.0},	
	{"MOTOR CAPACIT   ",	{0.1, 	300.0},					" kVA",		30,			0.1},
	{"MOT RATING V    ",	{80.0, 	480.0},		        	" Vrms",	380.0,		1.0},
	{"MOT RATING A    ",	{0.1, 	200.0},					" Arms",	60.0,		0.1},	
	{"MOTOR POLE PITCH",	{1.0,	50.0},					" mm",		24.0,		1.0},	//5
	{"ENCODER PPR     ",	{512.0, 32768.0},				" p/r",		32768.0,	1.0},
	{"SPEED BASE      ",	{0.0, 	3200.0},				" m/s",		1.0,		0.1},
	{"IQSE RATE       ",	{0.0, 	200.0},					" Apk",		100.0,		0.1},
	{"KNOW U ANGLE?   ",	{0.0,	YESNOSELECTNUM-1},		"   ",		1.0,	  	1.0},	
	{"MOTOR U ANGLE   ",	{-3.14,	3.14},		        	" rad",		0.0,	  	0.01},	//10
	{"ANGLE METHOD    ",	{0.0,	ROTORPOSESTISELNUM-1},	"   ",		1.0,	  	1.0},	
	{"SIG INJ.TIME    ",	{50.0, 	50000.0},				" ms",   	3000.0,		10.0},
	{"MOTOR LS        ",	{0.0, 	500.0},					" mH",		15.0,		0.01},	
	{"MOTOR RS        ",	{0.0, 	200.0},					" Ohm",		0.29,		0.001},	
	{"MOTOR KF        ",	{0.0, 	5.0},					"   ",		0.235,	  	0.001}
};

char* SYNMOTORSELECT[SYNMOTORSELECTNUM] =	{	
	"           GN/GT17A","    GT25/GN25/GT50A","           GN/GT17B","    GT35/GN35/GT70A","    GT25/GN25/GT50B",
	"    GT25/50/GN25Br1","    GT35/GN35/GT70B","              GT71A","              GT71B","             GM16TA",
	"             GM10TB","             GM16TB","             GT100A","             GT100B","             GT100C",
	"             GT100D","             GT160A","             GT160S","              GX35A","              GX35B",
	"              GX50A","              GX50B","              GX60A","              GX60B","              GX70A",
	"              GX70B","             GX160A","              GN50A","              GN50B","              GN50C",
	"              GN60A","              GN60B","              GN60C","              GY17A","              GY17B",
	"              GY20A","              GY20B","              GY25A","              GY25B","              GY35A",
	"              GY35B","              GX30A","              GX30B","   SM225.60B(YONEZ)","             GA100A",
	"             GA100B","           GT100Cr1","             GX160C","             GX160D","              GT70C",
	"              GN50D","             GA160B","             GX300A","             GA100D","           UELCO11A",
	"        OTHER MOTOR"
};

char* LINEARMOTORSELECT[LINEARMOTORSELECTNUM] =	{	
	"           FRLSM",	"        OTHER MOTOR"
};
#endif

char* MOTORTYPESELECT[MOTORTYPESELECTNUM] =	{
	"     Synchro. Motor","    Induction Motor"	};
	
char* ONOFFSELECT[ONOFFSELECTNUM] =	{
	"                Off ","                On "	};

char* UPDNSELECT[UPDNSELECTNUM] =	{
	"                 Up ","               Down "	};
	
char* OFFUPDNSELECT[OFFUPDNSELECTNUM] =	{
	"                Off ","                 Up ","               Down "	};

char* DECELSELECT[DECELMODENUM] =	{
	"           Free Run ","      Pattern Decel "	};

char* YESNOSELECT[YESNOSELECTNUM] = {
	"                 No ","                Yes "	};

char* FWDREVSELECT[FWDREVSELECTNUM] = {
	"                Rev ","                Fwd "	};

char* ROTORPOSESTISEL[ROTORPOSESTISELNUM] = {
	"   High Freq Inject ","          DC Inject "};

char* ENCODERTYPSELECT[ENCODERTYPENUM] =		
 {"            INC ENC ","         SINCOS ENC "};

char* SPDPTNSELECT[SPDPTNNUM] = {
	"        Normal V2th","     Double Pattern","        Normal V3th","    Optimal Pattern","      SS/TVF1&2 E/L",
	"      SS/TVF1&2 E/S"};
	
float InverterValue[INVERTERSELECTNUM][INVERTERNUM]  = {	
  	{48.82,	0.390,	45.0,  45.0,  6.0,	2200},	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB300 5.5kw(30[Ohm])
	{48.82,	0.390,	70.0,  70.0,  4.0,	2200},	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB300 7.5kw(25[Ohm])
	{97.65,	0.390,	95.0,  95.0,  2.5,	2700},	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB300 11kw(24[Ohm])
	{97.65, 0.390,	100.0, 100.0, 2.0,	3300},	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB300 15kw(25[Ohm])
	{195.31,0.480,	130.0, 130.0, 0.9,	5600},	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150 22kw(15[Ohm])
	{195.31,0.480,	180.0, 180.0, 0.8,	6800}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150 30kw(15[Ohm]):
	{195.31,0.480,	260.0, 260.0, 0.5,	10200}, //SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150 40kw(15[Ohm]):
	{195.31,0.480,	400.0, 400.0, 0.9,	12300}, //SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150 50kw(15[Ohm]):
	{195.31,0.480,	180.0, 180.0, 1.5,	4400}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150SSM 22kw(15[Ohm]):
	{195.31,0.480,	180.0, 180.0, 1.2,	5400}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150SSM 30kw(15[Ohm]):
	{195.31,0.480,	260.0, 260.0, 0.9,	6600}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150SSM 40kw(15[Ohm]):
	{11.17,	29.93,	270.0, 270.0, 0.9,	7600}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150SSM_HY 40kw(15[Ohm]):
	{195.31,0.480,	130.0, 130.0, 1.5,	4700}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150SSF 22kw(15[Ohm]):
	{195.31,0.480,	180.0, 180.0, 1.2,	5600}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: WB150SSF 30kw(15[Ohm]):
	{11.16,	30.14,	270.0, 270.0, 0.1,	220}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: UELCO 40kw(15[Ohm]):
	{3.05,	29.93,	95.0, 	95.0, 0.4,	200}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: UELCO 11kw(75[Ohm]):
	{11.16,	29.93,	100.0, 100.0, 0.0,	200}, 	//SCALE Is , SCALE VIN, CON OC LEVEL, INV OC LEVEL, IQE_RATE,  Li, Cdc: FREEWAY 30kVA(15[Ohm]):
	{195.31,0.480,	130.0, 130.0, 0.9,	10200}
};

//#if CN_MOTOR_DATA
//float SynMotorValue[SYNMOTORSELECTNUM][SYNMOTORNUM-1+10] = {
//
//  {4.3,			325.0,		9.9,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.0,		14.0,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		59.80,		1.750,		1.57,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		93.2,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		107.0,		0.5,		0.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJA250-630F3*/
//
//  {6.8,			325.0,		15.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   150.0,		21.9,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		22.76,		0.648,		0.99,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   90.0,		2.0, 		139.7,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		160.7,		0.5,		0.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJB250-630F3*/
//
//  {5.5,			325.0,		12.3,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.0,		17.4,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		50.80,		1.390,		1.57,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		93.2,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		107.0,		0.5,		0.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJA250-800F4*/
//
//  {8.8,			325.0,		20.4,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   150.0,		28.8,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		19.80,		0.535,		0.99,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   90.0,		2.0, 		139.7,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		160.7,		0.5,		0.70}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJB250-800F4*/
//
//  {9.6,			325.0,		22.4,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   165.0,		31.6,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		16.08,		0.436,		0.90,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		163.0,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		187.5,		0.5,		0.80}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJF250-800F5*/
//
//  {11.0,		325.0,		25.7,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   190.0,		36.3,		1.0,		0.0,		1.0,	/*Wrpm BASE,     	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		12.70,		0.342,		0.78,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   120.0,		2.0, 		186.3,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		214.3,		0.5,		0.90}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJC250-800F5*/
//
//  {13.8,		325.0,		31.0,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   235.0,		43.8,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		8.640,		0.235,		0.63,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   150.0,		2.0, 		232.9,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		268.0,		0.5,		1.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJD250-800F5*/
//
//  {7.0,			325.0,		16.0,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.0,		22.6,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		43.60,		1.109,		1.57,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		93.2,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		107.0,		0.5,		0.70}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJA250-1000F5*/
//
//  {11.0,		325.0,		25.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   150.0,		36.0,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		16.84,		0.424,		0.99,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   90.0,		2.0, 		139.7,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		160.7,		0.5,		0.90}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJB250-1000F5*/
//
//  {12.2,		325.0,		28.2,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   165.0,		39.8,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		13.86,		0.353,		0.90,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		163.0,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		187.5,		0.5,		1.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJF250-1000F5*/
//
//  {14.0,		325.0,		32.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   190.0,		45.9,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		10.54,		0.268,		0.78,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   120.0,		2.0, 		186.3,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		214.3,		0.5,		1.30}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJC250-1000F5*/
//
//  {17.4,		325.0,		40.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   235.0,		57.2,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		7.140,		0.179,		0.63,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   150.0,		2.0, 		232.9,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		268.0,		0.5,		1.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJC250-1000F5*/
//
//  {8.8,			325.0,		20.1,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.0,		28.4,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		35.30,		0.848,		1.57,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		93.2,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		107.0,		0.5,		0.70}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJA250-1250F5*/
//
//  {13.9,		325.0,		32.8,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   150.0,		46.3,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		13.32,		0.323,		0.99,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   90.0,		2.0, 		139.7,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		160.7,		0.5,		1.10}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJB250-1250F6*/
//
//  {15.3,		325.0,		35.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   165.0,		50.2,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		11.30,		0.267,		0.90,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		163.0,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		187.5,		0.5,		1.30}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJF250-1250F6*/
//
//  {17.6,		325.0,		40.8,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   190.0,		57.6,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		9.14,		0.216,		0.78,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   120.0,		2.0, 		186.3,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		214.3,		0.5,		1.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJC250-1250F6*/
//
//  {21.8,		325.0,		50.0,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   235.0,		71.8,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		5.74,		0.137,		0.63,		410.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   150.0,		2.0, 		232.9,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		268.0,		0.5,		2.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WYJD250-1250F6*/
//
//  {9.5,			325.0,		21.6,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.0,		30.5,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		37.21,		0.890,		1.62,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		94.1,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		108.2,		0.5,		0.80}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-10-1350*/
//
//  {14.9,		335.0,		33.7,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   150.0,		47.6,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		14.03,		0.339,		1.02,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   90.0,		2.0, 		141.1,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		162.3,		0.5,		1.30}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-16-1350*/
//
//  {16.5,		335.0,		37.0,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   165.0,		52.3,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		12.47,		0.297,		0.93,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		164.6,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		189.3,		0.5,		1.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-17.5-1350*/
//
//  {18.9,		335.0,		42.6,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   190.0,		60.2,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		8.98,		0.212,		0.81,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   120.0,		2.0, 		188.2,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		216.4,		0.5,		1.60}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-20-1350*/
//
//  {23.4,		335.0,		52.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   235.0,		74.2,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		6.06,		0.142,		0.65,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   150.0,		2.0, 		235.0,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		270.0,		0.5,		2.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-25-1350*/
//
//  {11.2,		335.0,		25.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.0,		36.0,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		29.76,		0.682,		1.62,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		94.1,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		108.2,		0.5,		0.90}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-10-1600*/
//
//  {17.7,		335.0,		41.0,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   150.0,		57.9,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		11.60,		0.264,		1.02,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   90.0,		2.0, 		141.1,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		162.3,		0.5,		1.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-16-1600*/
//
//  {19.5,		335.0,		45.0,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   165.0,		63.6,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		10.12,		0.242,		0.93,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		164.6,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		189.3,		0.5,		1.70}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-17.5-1600*/
//
//  {22.4,		335.0,		51.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   190.0,		72.8,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		7.44,		0.169,		0.81,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   120.0,		2.0, 		188.2,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		216.4,		0.5,		2.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-20-1600*/
//
//  {27.8,		335.0,		64.2,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   235.0,		90.7,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		4.66,		0.107,		0.65,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   150.0,		2.0, 		235.0,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		270.0,		0.5,		2.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-25-1600*/
//
//  {14.0,		335.0,		31.9,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.0,		45.1,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		18.40,		0.587,		1.62,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		94.1,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		108.2,		0.5,		1.20}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-10-2000*/
//
//  {28.0,		335.0,		60.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   190.0,		85.5,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		4.60,		0.147,		0.81,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   120.0,		2.0, 		188.2,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		216.4,		0.5,		2.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-20-2000*/
//
//  {17.5,		335.0,		39.5,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.0,		45.1,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		15.90,		0.494,		1.62,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		94.1,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		108.2,		0.5,		1.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-10-2000*/
//
//  {27.6,		335.0,		60.0,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   150.0,		84.8,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		6.50,		0.203,		1.02,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   90.0,		2.0, 		141.1,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		162.3,		0.5,		2.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : WTYF250-16-2500*/
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  {3.6,			340.0,		7.9,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   119.0,		11.2,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		53.0,		2.40,		1.36,		320.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		119.0,		20.0,		2.8,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   3.4,			30.0,		137.0,		0.5,		0.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE320-550-0.5/1*/
//
//  {5.0,			340.0,		11.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.5,		15.56,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		33.76,		2.01,		1.70,		400.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		95.5,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			35.0,		110.0,		0.5,		0.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE630/800-0.5/1*/
//
//  {9.0,			340.0,		21.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   167.0,		29.70,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		11.41,		0.69,		1.03,		400.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		167.0,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			40.0,		192.0,		0.5,		0.70}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE800-1.5/1.75*/
//
//  {6.0,			340.0,		14.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.5,		19.80,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		28.62,		1.63,		1.75,		400.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		95.5,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			40.0,		110.0,		0.5,		0.60}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE1000-0.5/1.0*/
//
//  {11.7,		340.0,		26.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   167.0,		36.77,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		10.17,		0.59,		1.03,		400.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia*/
//   105.0,		2.0, 		167.0,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		192.0,		0.5,		0.90}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE1000-1.5/1.75*/
//
//  {7.4,			340.0,		16.9,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   95.5,		23.90,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		22.24,		1.26,		1.66,		400.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		95.5,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		110.0,		0.5,		0.70}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE1150-0.5/1.0*/
//
//  {13.0,		340.0,		30.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   167.0,		42.43,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		7.75,		0.45,		1.00,		400.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		167.0,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			50.0,		192.0,		0.5,		1.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE1150-1.5/1.75*/
//
//  {9.0,			340.0,		18.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   85.0,		25.46,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		25.70,		0.80,		2.00,		450.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		85.0,		25.0,		3.0,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.2,			50.0,		97.8,		0.5,		0.80}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE1350-0.5/1.0*/
//
//  {15.0,		290.0,		42.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   152.0,		59.40,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		5.94,		0.185,		0.94,		450.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		148.6,		25.0,		3.0,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.2,			60.0,		170.9,		0.5,		1.30}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE1350-1.5/1.75*/
//
//  {10.0,		340.0,		22.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   85.0,		31.20,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		20.90,		0.59,		1.95,		450.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   60.0,		2.0, 		85.0,		25.0,		3.0,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.2,			50.0,		97.8,		0.5,		0.90}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE1600-0.5/1.0*/
//
//  {17.5,		290.0,		45.0,		32.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   152.0,		63.64,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		4.99,		0.141,		0.97,		450.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   105.0,		2.0, 		148.6,		25.0,		3.0,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.2,			60.0,		170.9,		0.5,		1.50}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GE1600-1.5/1.75*/
//
//  {27.6,		335.0,		60.0,		24.0,		8192.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//   150.0,		84.80,		1.0,		0.0,		1.0,	/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//   3000.0,		6.50,		0.203,		1.02,		406.0,	/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//   90.0,		2.0, 		141.1,		28.0,		3.1,	/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//   4.7,			80.0,		162.3,		0.5,		2.00}, 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : OTHER MOTOR*/
//
//};
//#else
//float SynMotorValue[SYNMOTORSELECTNUM][SYNMOTORNUM-1+10] = {
//
//    {3.7,			325.0,			9.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     96.0,			12.7,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		53.6,			1.606,		1.5,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			95.6,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			35.0,			110.0,		0.20,		0.20}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GN17A*/
//
//    {6.2,			318.0,			15.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     96.0,			21.2,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		31.4,			0.724,		1.56,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			95.6,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			35.0,			110.0,		0.25,		0.50}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT25A*/
//
//    {6.5,			315.0,			15.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     168.0,			21.2,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		19.3,			0.548,		0.9,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			167.1,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			35.0,			190.0,		0.25,		0.50}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GN17B*/
//
//    {7.1,			324.0,			16.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     96.0,			23.3,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		27.8,			0.695,		1.59,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			95.6,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			35.0,			110.0,		0.25,		0.70}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT35A*/
//
//    {10.8,			325.0,			25.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     168.0,			35.4,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		11.3,			0.289,		0.94,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			167.1,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			50.0,			180.0,		0.30,		0.90}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT25B*/
//
//	{10.8,			285.0,			28.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     168.0,			39.6,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		10.35,			0.215,		0.8,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			167.1,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			50.0,			192.0,		0.30,		0.90}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT25Br1*/
//
//    {14.1,			322.0,			31.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     191.0,			43.8,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		7.6,			0.182,		0.83,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     120.0,			2.0, 			191.0,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			55.0,			220.0,		0.40,		1.20}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT35B*/
//
//    {10.5,			318.0,			23.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     96.0,			32.5,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		20.9,			0.440,		1.59,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			95.6,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			50.0,			111.0,		0.50,		1.0}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT71A*/
//
//    {18.3,			327.0,			39.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     168.0,			55.8,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		7.80,			0.180,		0.95,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			167.1,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.7,			80.0,			192.0,		0.50,		1.8}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT71B*/
//
//    {20.0,			257.0,			50.0,		24.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     137.0,			70.7,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		10.3,			0.220,		1.15,		560.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			136.4,		17.0,		2.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.9,			105.0,			156.9,		0.50,		1.50}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GM16TA*/
//
//    {22.0,			294.0,			47.0,		24.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     273.0,			66.5,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		6.1,			0.190,		0.68,		420.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     180.0,			2.0, 			272.8,		22.7,		3.0,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.5,			105.0,			313.8,		0.50,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GM10TB*/
//
//    {30.0,			253.0,			72.0,		24.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     205.0,			101.8,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		4.71,			0.110,		0.80,		560.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     180.0,			2.0, 			204.6,		17.0,		2.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.9,			200.0,			235.3,		0.60,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GM16TB*/
//
//    {11.0,			325.0,			26.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     91.0,			36.7,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		21.4,			0.513,		1.65,		420.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			91.0,		27.0,		3.0,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.5,			80.0,			107.0,		0.5,		0.90}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT100A*/
//
//    {22.1,			325.0,			50.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     182.0,			70.7,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		5.5,			0.132,		0.85,	  	420.0,		/*SEARCH TIME,		Ls,					Rs,			Ke,				Sheave Dia	*/
//     120.0,			2.0, 			182.0,		27.0,		3.0,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.5,			110.0,			200.0,		0.5,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT100B*/
//
//    {21.2,			309.0,			47.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     273.0,			67.2,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		2.8,			0.075,		0.59,	  	420.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     180.0,			2.0, 			273.0,		27.0,		3.0,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.5,			105.0,			313.0,		0.5,		1.70}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT100C*/
//
//    {24.5,			314.0,			55.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     227.0,			77.8,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		3.7,			0.086,		0.68,	 	420.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     150.0,			2.0, 			227.0,		27.0,		3.0,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.5,			110.0,			260.0,		0.7,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT100D*/
//
//	{30.0,			322.0,			52.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     204.0,			77.8,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		3.9,			0.160,		0.84,		560.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     180.0,			2.0, 			204.6,		17.0,		2.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.9,			150.0,			234.6,		1.0,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT160A*/
//
//    {30.0,			312.0,			59.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     119.0,			84.1,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		4.85,			0.155,		1.32,		560.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			119.4,		17.0,		2.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.9,			150.0,			137.3,		1.0,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GT160S*/
//
//	{3.7,			341.0,			8.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     48.0,			12.0,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		111.3,			2.775,		3.16,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			1.0, 			47.7,		12.0,		1.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     2.3,			25.0,			56.6,		0.4,		0.50}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX35A*/
//
//	{6.5,			293.0,			16.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     84.0,			22.6,			1.0,		0.0,		1.0,		/*Wrpm BASE,     	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		30.5,			0.727,		1.65,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			1.0, 			83.6,		12.0,		1.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     2.3,			50.0,			99.1,		0.5,		0.60}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX35B*/
//
//	{6.2,			290.0,			16.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     35.0,			23.3,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		69.9,			1.284,		3.61,		550.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			1.0, 			34.7,		8.7,		1.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     1.7,			70.0,			40.0,		1.0,		0.60}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX50A*/
//
//    {10.8,			302.0,			27.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     61.0,			38.9,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		30.7,			0.513,		2.17,		550.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			1.0, 			60.8,		8.7,		1.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     1.7,			110.0,			70.0,		1.0,		0.80}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX50B*/
//
//    {7.1,			308.0,			18.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     29.0,			25.5,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		44.8,			1.36,		4.52,		660.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			1.0, 			28.9,		7.2,		0.9,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     1.4,			70.0,			34.0,		1.0,		0.70}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX60A*/
//
//	{14.1,			304.0,			32.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     58.0,			45.2,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		12.5,			0.38,		2.57,		660.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     120.0,			1.0, 			57.9,		7.2,		0.9,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     1.4,			250.0,			68.0,		1.0,		1.20}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX60B*/
//
//    {9.8,			298.0,			23.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     29.0,			33.5,		 	 1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		29.5,			0.796,		4.87,		660.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			1.0, 			28.9,		7.2,		0.9,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     1.4,			80.0,			34.0,		1.2,		0.80}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX70A*/
//
//    {19.6,			306.0,			42.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     58.0,			61.0,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		8.85,			0.248,		2.67,		660.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     120.0,			1.0, 			57.9,		7.2,		0.9,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     1.4,			400.0,			66.7,		1.5,		1.50}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX70B*/
//
//    {29.5,			350.0,			56.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     174.0,			79.2,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		2.72,			0.086,		0.60,		660.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     180.0,			2.0, 			173.6,		14.5,		2.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.4,			450.0,			133.2,		1.5,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX160A*/
//
//    {12.3,			270.0,			33.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     70.0,			47.3,		 	1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		17.7,			0.32,		1.82,		550.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			69.4,		17.4,		2.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.4,			60.0,			76.0,		0.4,		1.20}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GN50A*/
//
//    {21.5,			317.0,			48.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     122.0,			67.9,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		8.49,			0.17,		1.25,		550.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			121.5,		17.4,		2.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.4,			150.0,			134.0,		0.5,		1.90}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GN50B*/
//
//	{24.5,			300.0,			55.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     174.0,			78.5,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		4.17,			0.096,		0.87,	  	550.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     150.0,			2.0, 			173.6,		17.4,		2.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.4,			200.0,			191.4,		0.8,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GN50C*/
//
//	{15.4,			317.0,			33.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     64.0,			46.7,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		11.62,			0.349,		2.46,		600.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			63.7,		15.9,		2.1,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.2,			250.0,			70.0,		0.8,		1.30}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GN60A*/
//
//	{26.8,			280.0,			62.5,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     112.0,			88.4,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		3.15,			0.095,		1.3,		600.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			111.4,		15.9,		2.1,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.2,			350.0,			122.0,		1.0,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GN60B*/
//
//    {30.0,			317.0,			63.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     128.0,			89.1,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		3.25,			0.085,		1.4,		600.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     120.0,			2.0, 			127.3,		15.9,		2.1,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.2,			400.0,			146.4,		1.0,		2.00},	 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GN60C*/
//
//    {3.7,			307.0,			8.5,		20.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     160.0,			12.0,		  	1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		23.75,			1.640,		1.33,		240.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			159.1,		39.7,		5.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     8.0,			8.0,			182.9,		0.14,		0.40},	 	/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GY17A*/
//
//    {6.2,			305.0,			15.2,		20.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     279.0,			21.50,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		7.85,			0.570,		0.57,		240.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			278.5,		39.7,		5.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     8.0,			8.0,			321.0,		0.14,		0.60}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GY17B*/
//
//    {6.2,			312.0,			14.6,		20.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     160.0,			20.6,		  	1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		35.24,			1.213,		0.90,	  	240.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			159.1,		39.7,		5.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     8.0,			8.0,			182.9,		0.14,		0.70}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GY20A*/
//
//    {11.5,			300.0,			25.6,		20.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     279.0,			32.2,		  	1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		5.65,			0.360,		0.560,		240.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			278.5,		39.7,		5.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     8.0,			10.0,			321.0,		0.14,		1.10}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GY20B*/
//
//    {7.5,			300.0,			17.0,		20.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     160.0,			24.04,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		12.90,			0.790,		0.950,		240.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			159.1,		39.7,		5.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     8.0,			8.0,			183.0,		0.14,		0.75}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GY25A*/
//
//    {13.2,			303.0,			29.0,		20.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     279.0,			41.0,		  	1.0,		0.0,		1.0,		/*Wrpm BASE,     	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		4.65,			0.280,		0.650,		240.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			278.5,		39.7,		5.3,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     8.0,			12.0,			321.0,		0.15,		1.50}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GY25B*/
//
//    {9.8,			297.0,			26.0,		24.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     142.0,			36.7,		 	1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		18.25,			0.640,		1.060,		270.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			141.5,		35.4,		4.7,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     7.1,			17.0,			163.0,		0.26,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GY35A*/
//
//    {17.2,			307.0,			43.0,		24.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     248.0,			60.80,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		5.55,			0.230,		0.580,		270.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			247.6,		35.4,		4.7,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     7.1,			17.0,			285.0,		0.26,		2.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GY35B*/
//
//    {3.6,			300.0,			9.8,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     48.0,			13.85,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		106.50,			2.120,		2.580,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			1.0, 			47.7,		12.0,		1.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     2.4,			52.0,			55.2,		0.90,		0.70}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX30A*/
//
//	{5.4,			315.0,			13.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     72.0,			19.0,		  	1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		57.0,			1.170,		1.89,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     90.0,			1.0, 			71.6,		12.0,		1.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     2.4,			52.0,			83.0,		0.95,		1.30}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GX30B*/
//
//    {11.0,			360.0,			35.0,		20.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     96.0,			49.45,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		19.60,			0.86,		0.900,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0, 			96.0,		24.0,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.8,			20.0,			110.4,		0.0,		1.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : SM225.60B(YONNETZ)*/
//
//    {26.2,			325.0,			57.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     239.0,			80.61,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		3.65,			0.079,		0.679,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     150.0,			2.0, 			238.7,		23.8,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.8,			200.0,			274.9,		0.5,		0.50}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GA100A*/
//
//    {21.2,			386.0,			37.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     287.0,			52.33,			1.0,		0.0,		1.0,		/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		4.35,			0.104,		0.713,		400.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     180.0,			2.0, 			286.5,		23.8,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.8,			150.0,			328.9,		0.5,		0.50}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : GA100B*/
//
//    {21.2,			294.0,			49.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     273.0,			69.0,			1,			0,			1,			/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000,			2.85,	  		0.08,		0.56,		420,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     180.0,			2.0,			272.8,		27.0,		3.0,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.5,			105.0,			314,		0.5,		2.0},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	: GT100Cr1*/
//
//    {13.1,			321.0,			27.2,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     68.2,			38.4,			1,			0,			1,			/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000,			12.85,	  		0.42,		2.30,		560,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0,			68.2,		17.0,		2.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.9,			60.0,			78.0,		0.5,		1.3},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	: GX160C*/
//
//    {26.2,			324.0,			53.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     136.4,			75.0,			1,			0,			1,			/*Wrpm BASE,     	 IQSE RATE, 	KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000,			3.45,	  		0.11,		1.20,		560,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     120.0,			2.0,			136.4,		17.0,		2.6,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.9,			300.0,			157.0,		0.5,		2.4},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	: GX160D*/
//
//    {18.8,			290.0,			44.0,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     239.0,			62.2,			1,			0,			1,			/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000,			3.62,	  		0.08,		0.59,		400,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     150.0,			2.0,			238.7,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.8,			100.0,			275.0,		0.5,		2.4},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	: GT70C*/
//
//    {10.5,			330.0,			23.5,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     95.5,			33.2,			1,			0,			1,			/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000,			21.8,	  		0.51,		1.65,		400,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			2.0,			95.5,		23.9,		3.2,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     4.8,			50.0,			115.0,		0.3,		1.0},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	C FF GAIN		TQBIAS_DELTA 	: GN50D*/
//
//    {39.2,			362.0,			71.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     294.0,			100.4,			1,			0,			1,			/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000,			1.55,	  		0.12,		0.37,		520,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     240.0,			2.0,			294.0,		18.4,		2.5,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     3.0,			220.0,			338.1,		1.0,		3.0},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	: GA160B*/
//
//    {39.2,			373.0,			67.0,		48.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     127.3,			94.8,			1,			0,			1,			/*Wrpm BASE,     	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000,			2.79,	  		0.08,		0.883,		600,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     240.0,			1.0,			127.3,		7.96,		1.1,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     1.3,			220.0,			146.4,		1.0,		3.0},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	: GX300A*/
//
//    {13.2,			300.0,			32.3,		32.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     104.0,			45.7,			1,			0,			1,			/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000,			14.85,	  		0.3,		1.125,		320,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			1.0,			104.0,		14.86,		2.0,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     2.3,			50.0,			120.0,		0.5,		1.0},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	: GA100D*/
//
//    {11.0,			307.0,			23.8,		8.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     836.0,			32.2,			1.0,		0.0,		1,			/*Wrpm BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		5.51,			0.25,		0.21,		80.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     105.0,			2.0, 			835.6,		119.35,		15.91,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     19.89,			2.0,			900.0,		0.2, 		1.0},		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	:UELCO*/
//
//    {0.0,			0.0,			0.0,		0.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     0.0,			0.0,			1.0,		0.0,		0.0,		/*Wrpm BASE,     	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		0.0,			0.0,		0.0,		0.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     0.0,			2.0, 			0.0,		0.0,		0.0,		/*EL_SPEED,			ROPING, 		MAX_RPM,		INSPECT_RPM,	CREEP_RPM	*/
//     0.0,			0.0,			0.0,		0.0,		0.0}		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA 	: OTHER MOTOR*/
//};
//#endif
//float LinMotorValue[LINEARMOTORSELECTNUM][LINEARMOTORNUM-1+10] = {
//	{30,			380.0,			60.0,		24.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     1.0,			100.0,			1.0,		0.0,		1.0,		/*Speed BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		15.0,			0.29,		0.235,		0.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			0.0, 			1.0,		0.3,		0.3,		/*EL_SPEED,			ROPING, 		MAX_SPEED,		INSPECT_RPM,	CREEP_RPM	*/
//     0.3,			15.0,			1.2,		0.00,		0.00}, 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : FRLSM*/
//    {30,			380.0,			60.0,		24.0,		32768.0,	/*MOTOR CAPACIT,	RATING V,		RATING A,		POLES,			PG PULSE	*/
//     1.0,			100.0,			1.0,		0.0,		1.0,		/*Speed BASE,      	IQSE RATE, 		KNOW_ANGLE, 	U_ANGLE,  		ANGLE METHOD*/
//     3000.0,		15.0,			0.29,		0.235,		0.0,		/*SEARCH TIME,		Ls,				Rs,				Ke,				Sheave Dia	*/
//     60.0,			0.0, 			1.0,		0.3,		0.3,		/*EL_SPEED,			ROPING, 		MAX_SPEED,		INSPECT_RPM,	CREEP_RPM	*/
//     0.3,			15.0,			1.2,		0.00,		0.00} 		/*RELEVEL RPM, 		SC JM,			OS LEVEL,   	SC FF GAIN		TQBIAS_DELTA : FRLSM*/
//};
char *LcdMenu[TOTALMENUNUM] = {
		/*Main Menu*/			"MONITOR         ",	/* 0*/
								"PROGRAM         ",	/* 1*/
		/*Monitor Menu*/		"BASIC           ",	/* 2*/
								"I/O             ",	/* 3*/
								"ERROR           ",	/* 4*/
								"FLOOR DATA      ",	/* 5*/
		/*Program Menu*/		"CONTROL         ",	/* 6*/
								"INTERFACE       ",	/* 7*/
								"MOTOR           ",	/* 8*/
								"FACTORY         ",	/* 9*/
};	/*total 9*/

char *MotorSelectionMenu[MOTORSELCTIONMENU] = {
		/* Syncronous Motor*/	"SYNC MOTOR		 ", /* 0*/
		/* Linear Motor*/		"LINEAR MOTOR    ", /* 1*/
	
}; /* total 1*/

ITEMDISPLAY FloorList[FLOORNUM] = {
	{"   CURRENT POS. ",	0.0},
	{"   1F      POS. ",	0.0},
	{"   2F      POS. ",	0.0}, 
	{"   3F      POS. ",	0.0},
	{"   4F      POS. ",	0.0},
	{"   5F      POS. ",	0.0},
	{"   6F      POS. ",	0.0},
	{"   7F      POS. ",	0.0},
	{"   8F      POS. ",	0.0},
	{"   9F      POS. ",	0.0},
	{"   10F     POS. ",	0.0},
	{"   11F     POS. ",	0.0},
	{"   12F     POS. ",	0.0},
	{"   13F     POS. ",	0.0},
	{"   14F     POS. ",	0.0},
	{"   15F     POS. ",	0.0},
	{"   16F     POS. ",	0.0},
	{"   17F     POS. ",	0.0},
	{"   18F     POS. ",	0.0},
	{"   19F     POS. ",	0.0},
	{"   20F     POS. ",	0.0},
	{"   21F     POS. ",	0.0},
	{"   22F     POS. ",	0.0},
	{"   23F     POS. ",	0.0},
	{"   24F     POS. ",	0.0},
	{"   25F     POS. ",	0.0},
	{"   26F     POS. ",	0.0},
	{"   27F     POS. ",	0.0},
	{"   28F     POS. ",	0.0},
	{"   29F     POS. ",	0.0},
	{"   30F     POS. ",	0.0},
	{"   31F     POS. ",	0.0},
	{"   32F     POS. ",	0.0},
	{"   33F     POS. ",	0.0},
	{"   34F     POS. ",	0.0},
	{"   35F     POS. ",	0.0},
	{"   36F     POS. ",	0.0},
	{"   37F     POS. ",	0.0},
	{"   38F     POS. ",	0.0},
	{"   39F     POS. ",	0.0},
	{"   40F     POS. ",	0.0},
	{"   41F     POS. ",	0.0},
	{"   42F     POS. ",	0.0}, 
	{"   43F     POS. ",	0.0},
	{"   44F     POS. ",	0.0},
	{"   45F     POS. ",	0.0},
	{"   46F     POS. ",	0.0},
	{"   47F     POS. ",	0.0},
	{"   48F     POS. ",	0.0},
	{"   49F     POS. ",	0.0},
	{"   50F     POS. ",	0.0},
	{"   51F     POS. ",	0.0},
	{"   52F     POS. ",	0.0}, 
	{"   53F     POS. ",	0.0},
	{"   54F     POS. ",	0.0},
	{"   55F     POS. ",	0.0},
	{"   56F     POS. ",	0.0},
	{"   57F     POS. ",	0.0},
	{"   58F     POS. ",	0.0},
	{"   59F     POS. ",	0.0},
	{"   60F     POS. ",	0.0},
	{"   61F     POS. ",	0.0},
	{"   62F     POS. ",	0.0},
	{"   63F     POS. ",	0.0},
//	{"   64F     POS. ",	0.0},
//	{"   65F     POS. ",	0.0},
//	{"   66F     POS. ",	0.0}, 
//	{"   67F     POS. ",	0.0},
//	{"   68F     POS. ",	0.0},
//	{"   69F     POS. ",	0.0},
//	{"   70F     POS. ",	0.0},
//	{"   71F     POS. ",	0.0},
//	{"   72F     POS. ",	0.0},
//	{"   73F     POS. ",	0.0},
//	{"   74F     POS. ",	0.0},
//	{"   75F     POS. ",	0.0},
//	{"   76F     POS. ",	0.0},
//	{"   77F     POS. ",	0.0},
//	{"   78F     POS. ",	0.0},
//	{"   79F     POS. ",	0.0},
//	{"   80F     POS. ",	0.0},
//	{"   81F     POS. ",	0.0},
//	{"   82F     POS. ",	0.0},
//	{"   83F     POS. ",	0.0},
//	{"   84F     POS. ",	0.0},
//	{"   85F     POS. ",	0.0},
//	{"   86F     POS. ",	0.0},
//	{"   87F     POS. ",	0.0},
//	{"   88F     POS. ",	0.0},
//	{"   89F     POS. ",	0.0},
//	{"   90F     POS. ",	0.0},
//	{"   91F     POS. ",	0.0},
//	{"   92F     POS. ",	0.0},
//	{"   93F     POS. ",	0.0},
//	{"   94F     POS. ",	0.0},
//	{"   95F     POS. ",	0.0},
//	{"   96F     POS. ",	0.0},
//	{"   97F     POS. ",	0.0},
//	{"   98F     POS. ",	0.0},
//	{"   99F     POS. ",	0.0},
//	{"   100F    POS. ",	0.0},
//	{"   101F    POS. ",	0.0},
//	{"   102F    POS. ",	0.0},
//	{"   103F    POS. ",	0.0},
//	{"   104F    POS. ",	0.0},
//	{"   105F    POS. ",	0.0},
//	{"   106F    POS. ",	0.0}, 
//	{"   107F    POS. ",	0.0},
//	{"   108F    POS. ",	0.0},
//	{"   109F    POS. ",	0.0},
//	{"   110F    POS. ",	0.0},
//	{"   111F    POS. ",	0.0},
//	{"   112F    POS. ",	0.0},
//	{"   113F    POS. ",	0.0},
//	{"   114F    POS. ",	0.0},
//	{"   115F    POS. ",	0.0},
//	{"   116F    POS. ",	0.0}, 
//	{"   117F    POS. ",	0.0},
//	{"   118F    POS. ",	0.0},
//	{"   119F    POS. ",	0.0},
//	{"   120F    POS. ",	0.0},
//	{"   121F    POS. ",	0.0},
//	{"   122F    POS. ",	0.0},
//	{"   123F    POS. ",	0.0},
//	{"   124F    POS. ",	0.0},
//	{"   125F    POS. ",	0.0},
//	{"   126F    POS. ",	0.0},
//	{"   127F    POS. ",	0.0},
	{"   SDS1    POS. ",	0.0},
	{"   SDS2    POS. ",	0.0},
	{"   SDS3    POS. ",	0.0},
	{"   SDS4    POS. ", 	0.0},
	{"   SDS5    POS. ", 	0.0},
	{"   SDS6    POS. ", 	0.0},
	{"   SDS7    POS. ", 	0.0},
	{"   SUS1    POS. ",	0.0},
	{"   SUS2    POS. ",	0.0},
	{"   SUS3    POS. ",	0.0},
	{"   SUS4    POS. ",	0.0},
	{"   SUS5    POS. ",	0.0},
	{"   SUS6    POS. ",	0.0},
	{"   SUS7    POS. ",	0.0},
	{"   SDF     POS. ",	0.0},
	{"   SUF     POS. ",	0.0}
};

ITEMMINMAX AOSELECT[AOSELECTNUM] =
{
	{"          SPEED REF ",			{0.0,	200.0}		}, 	//0
	{"          SPEED FBK ",			{0.0,	200.0}		},	//1
	{"         TORQUE REF ",			{0.0,	50.0}		},	//2
	{"         TORQUE FBK ",			{0.0,	50.0}		},	//3
	{"           FLUX REF ",			{0.0,	50.0}		},	//4
	{"           FLUX FBK ",			{0.0,	50.0}		},	//5 
	{"       DC LINK VOLT ",			{0.0,	800.0}		},	//6 
	{"        RMS CURRENT ",			{0.0,	50.0}		},	//7 
	{"            VOLTAGE ",			{0.0,	400.0}		},	//8
	{"         IU CURRENT ",			{0.0,	40.0}		},	//9
	{"         IV CURRENT ",			{0.0,	40.0}		},	//10
	{"         IW CURRENT ",			{0.0,	40.0}		},	//11
	{"            VAS REF ",			{0.0,	400.0}		},	//12
	{"            VBS REF ",			{0.0,	400.0}		},	//13
	{"            VCS REF ",			{0.0,	400.0}		},	//14
	{"           IDSS FBK ",			{0.0,	40.0}		},	//15
	{"           IQSS FBK ",			{0.0,	40.0}		},	//16
	{"           VDSE REF ",			{0.0,	400.0}		},	//17
	{"           VQSE REF ",			{0.0,	400.0}		},	//18
	{"       VDSE REF FBK ",			{0.0,	400.0}		},	//19
	{"       VQSE REF FBK ",			{0.0,	400.0}		},	//20
	{"     VDSE REF INTEG ",			{0.0,	400.0}		},	//21
	{"     VQSE REF INTEG ",			{0.0,	400.0}		},	//22
	{"             TE REF ",			{0.0,	400.0}		},	//23
	{"         TE REF FBK ",			{0.0,	400.0}		},	//24
	{"          TE REF FF ",			{0.0,	400.0}		},	//25
	{"        TQBIAS AUTO ",			{0.0,	40.0}		},	//26
	{"             TQBIAS ",			{0.0,	40.0}		},	//27
	{"              THETA ",			{0.0,	4.0}		},	//28
	{"              V_REF ",			{0.0,	2000.0}		},	//29
	{"        V_REF_LIMIT ",			{0.0,	2000.0}		},	//30
	{"         FAULT FLAG ",			{0.0,	4.0}		},	//31
	{"    DECEL FLAG MODE ",			{0.0,	40.0}		},	//32
	{"         DECEL FLAG ",			{0.0,	4.0}		},	//33
	{"    AUTO DRIVE MODE ",			{0.0,	20.0}		},	//34
	{"     PATTERN STATUS ",			{0.0,	40.0}		},	//35
	{"       DRIVE STATUS ",			{0.0,	10.0}		},	//36
	{"     DECEL FLAG OUT ",			{0.0,	4.0}		},	//37
	{"          CALL DATA ",			{0.0,	40.0}		},	//38
	{"         STOP FLOOR ",			{0.0,	40.0}		},	//39
	{"      DISPLAY FLOOR ",			{0.0,	40.0}		},	//40
	{"         REAL FLOOR ",			{0.0,	40.0}		},	//41
	{"        DECEL FLOOR ",			{0.0,	40.0}		},	//42
	{"        UP DOWN CMD ",			{0.0,	4.0}		},	//43
	{"             OUTPUT ",			{0.0,	20.0}		},	//44
	{"               SUDS ",			{0.0,	40.0}		}	//45
};
char *ErrBackupList[ERRORBACKUPDATANUM] = {
	"INV1_CC.Iqse_Ref ",
	"INV1.Iqse        ",
	"INV1_CC.Idse_Ref ",
	"INV1.Idse        ",
	"INV1.Ias         ",
	"INV1.Ibs         ",
	"INV1.Ics         ",
	"Vdc              ",
	"INV1_PC.Pos_Ref  ",
	"INV1_PC.Pos      ",
	"INV1_SC.Wrpm_ref ",
	"INV1_SC.Wrpm     ",
	"INV1_SC.Te_ref   ",
	"INV1.Kt          ",
	"GPIO INPUT1      ",
	"GPIO INPUT2      ",
	"GPIO OUTPUT      ",
};

