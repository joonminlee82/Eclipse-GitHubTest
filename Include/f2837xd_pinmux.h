//*****************************************************************************
//
//  f2837xd_pinmux.h - Created using TI Pinmux 4.0.1527  on 2018. 12. 6. at 오전 9:45:58.
//
//*****************************************************************************
//
// Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************
//
//  These values will provide the functionality requested when written into
//  the registers for which the #defines are named.  For example, using the
//  C2000Ware device support header files, use the defines like in this
//  sample function:
//
//  void samplePinMuxFxn(void)
//  {
//      EALLOW;
//      //
//      // Write generated values to mux registers
//      //
//      GpioCtrlRegs.GPAMUX1.all  = GPAMUX1_VALUE;
//      GpioCtrlRegs.GPAMUX2.all  = GPAMUX2_VALUE;
//      GpioCtrlRegs.GPBMUX1.all  = GPBMUX1_VALUE;
//        . . .
//      EDIS;
//  }
//
//  NOTE:  These GPIO control registers are only available on CPU1.
//
//*****************************************************************************

//
// Port A mux register values
//
// Pin T11 (GPIO30) to CANRXA (mode 1)
// Pin U11 (GPIO31) to CANTXA (mode 1)
// Pin C8 (GPIO0) to EPWM1A (mode 1)
// Pin D8 (GPIO1) to EPWM1B (mode 1)
// Pin A7 (GPIO2) to EPWM2A (mode 1)
// Pin B7 (GPIO3) to EPWM2B (mode 1)
// Pin C7 (GPIO4) to EPWM3A (mode 1)
// Pin D7 (GPIO5) to EPWM3B (mode 1)
// Pin A6 (GPIO6) to GPIO6 (mode 0)
// Pin B6 (GPIO7) to GPIO7 (mode 0)
// Pin G2 (GPIO8) to GPIO8 (mode 0)
// Pin G3 (GPIO9) to GPIO9 (mode 0)
// Pin C2 (GPIO12) to GPIO12 (mode 0)
// Pin D3 (GPIO15) to GPIO15 (mode 0)
// Pin J4 (GPIO22) to GPIO22 (mode 0)
// Pin K4 (GPIO23) to GPIO23 (mode 0)
// Pin C1 (GPIO11) to SCIRXDB (mode 2)
// Pin B2 (GPIO10) to SCITXDB (mode 6)
// Pin K3 (GPIO24) to SPISIMOB (mode 6)
// Pin K2 (GPIO25) to SPISOMIB (mode 6)
// Pin K1 (GPIO26) to SPICLKB (mode 6)
// Pin L1 (GPIO27) to SPISTEB (mode 6)
#define GPAMUX1_MASK		0xc3ffffff
#define GPAMUX2_MASK		0xf0fff000
#define GPAMUX1_VALUE		0x00a00555
#define GPAMUX2_VALUE		0x50aa0000
#define GPAGMUX1_VALUE		0x00100000
#define GPAGMUX2_VALUE		0x00550000

//
// Port B mux register values
//
// Pin J16 (GPIO63) to EM2D5 (mode 3)
// Pin J17 (GPIO62) to EM2D6 (mode 3)
// Pin L16 (GPIO61) to EM2D7 (mode 3)
// Pin M17 (GPIO60) to EM2D8 (mode 3)
// Pin M16 (GPIO59) to EM2D9 (mode 3)
// Pin N17 (GPIO58) to EM2D10 (mode 3)
// Pin N18 (GPIO57) to EM2D11 (mode 3)
// Pin U13 (GPIO32) to GPIO32 (mode 0)
// Pin T13 (GPIO33) to GPIO33 (mode 0)
// Pin U14 (GPIO34) to GPIO34 (mode 0)
// Pin T14 (GPIO35) to GPIO35 (mode 0)
// Pin V17 (GPIO40) to GPIO40 (mode 0)
// Pin U17 (GPIO41) to GPIO41 (mode 0)
// Pin D19 (GPIO42) to GPIO42 (mode 0)
// Pin C19 (GPIO43) to GPIO43 (mode 0)
// Pin K18 (GPIO44) to GPIO44 (mode 0)
// Pin K19 (GPIO45) to GPIO45 (mode 0)
// Pin P18 (GPIO54) to GPIO54 (mode 0)
// Pin P19 (GPIO55) to GPIO55 (mode 0)
// Pin R18 (GPIO50) to SPISIMOC (mode 6)
// Pin R19 (GPIO51) to SPISOMIC (mode 6)
// Pin P16 (GPIO52) to SPICLKC (mode 6)
// Pin P17 (GPIO53) to SPISTEC (mode 6)
#define GPBMUX1_MASK		0x0fff00ff
#define GPBMUX2_MASK		0xfffcfff0
#define GPBMUX1_VALUE		0x00000000
#define GPBMUX2_VALUE		0xfffc0aa0
#define GPBGMUX1_VALUE		0x00000000
#define GPBGMUX2_VALUE		0x00000550

//
// Port C mux register values
//
// Pin C18 (GPIO68) to EM2D0 (mode 3)
// Pin B19 (GPIO67) to EM2D1 (mode 3)
// Pin K17 (GPIO66) to EM2D2 (mode 3)
// Pin K16 (GPIO65) to EM2D3 (mode 3)
// Pin L17 (GPIO64) to EM2D4 (mode 3)
// Pin B15 (GPIO78) to EQEP2A (mode 6)
// Pin C15 (GPIO79) to EQEP2B (mode 6)
// Pin A14 (GPIO81) to EQEP2I (mode 6)
// Pin B16 (GPIO72) to GPIO72 (mode 0)
// Pin C14 (GPIO83) to GPIO83 (mode 0)
// Pin B5 (GPIO91) to GPIO91 (mode 0)
// Pin B4 (GPIO93) to GPIO93 (mode 0)
// Pin A3 (GPIO94) to GPIO94 (mode 0)
// Pin B3 (GPIO95) to GPIO95 (mode 0)
// Pin B11 (GPIO85) to SCIRXDA (mode 5)
// Pin A11 (GPIO84) to SCITXDA (mode 5)
#define GPCMUX1_MASK		0xf00303ff
#define GPCMUX2_MASK		0xfcc00fcc
#define GPCMUX1_VALUE		0xa00003ff
#define GPCMUX2_VALUE		0x00000508
#define GPCGMUX1_VALUE		0x50000000
#define GPCGMUX2_VALUE		0x00000504

//
// Port D mux register values
//
// Pin W10 (GPIO116) to EM2CS2n (mode 3)
// Pin U15 (GPIO120) to EM2WEn (mode 3)
// Pin T15 (GPIO119) to EM2RNW (mode 3)
// Pin W16 (GPIO121) to EM2OEn (mode 3)
// Pin M4 (GPIO111) to EM2BA0 (mode 3)
// Pin M3 (GPIO112) to EM2BA1 (mode 3)
// Pin F1 (GPIO98) to EM2A0 (mode 3)
// Pin G1 (GPIO99) to EM2A1 (mode 3)
// Pin H1 (GPIO100) to EM2A2 (mode 3)
// Pin H2 (GPIO101) to EM2A3 (mode 3)
// Pin H3 (GPIO102) to EM2A4 (mode 3)
// Pin J1 (GPIO103) to EM2A5 (mode 3)
// Pin J2 (GPIO104) to EM2A6 (mode 3)
// Pin J3 (GPIO105) to EM2A7 (mode 3)
// Pin L2 (GPIO106) to EM2A8 (mode 3)
// Pin L3 (GPIO107) to EM2A9 (mode 3)
// Pin L4 (GPIO108) to EM2A10 (mode 3)
// Pin C3 (GPIO96) to GPIO96 (mode 0)
// Pin N2 (GPIO109) to GPIO109 (mode 0)
// Pin N4 (GPIO113) to GPIO113 (mode 0)
// Pin N3 (GPIO114) to GPIO114 (mode 0)
// Pin T12 (GPIO118) to GPIO118 (mode 0)
#define GPDMUX1_MASK		0xcffffff3
#define GPDMUX2_MASK		0x000ff33f
#define GPDMUX1_VALUE		0xc3fffff0
#define GPDMUX2_VALUE		0x000fc303
#define GPDGMUX1_VALUE		0x00000000
#define GPDGMUX2_VALUE		0x00000000

//
// Port E mux register values
//
// Pin C13 (GPIO151) to EPWM4A (mode 1)
// Pin D13 (GPIO152) to EPWM4B (mode 1)
// Pin V10 (GPIO131) to GPIO131 (mode 0)
// Pin W18 (GPIO132) to GPIO132 (mode 0)
// Pin G18 (GPIO133) to GPIO133 (mode 0)
// Pin V18 (GPIO134) to GPIO134 (mode 0)
// Pin U18 (GPIO135) to GPIO135 (mode 0)
// Pin T17 (GPIO136) to GPIO136 (mode 0)
// Pin T18 (GPIO137) to GPIO137 (mode 0)
// Pin T19 (GPIO138) to GPIO138 (mode 0)
// Pin F18 (GPIO143) to GPIO143 (mode 0)
// Pin F17 (GPIO144) to GPIO144 (mode 0)
// Pin N19 (GPIO139) to SCIRXDC (mode 6)
// Pin M19 (GPIO140) to SCITXDC (mode 6)
// Pin M18 (GPIO141) to SCIRXDD (mode 6)
// Pin L19 (GPIO142) to SCITXDD (mode 6)
#define GPEMUX1_MASK		0xffffffc0
#define GPEMUX2_MASK		0x0003c003
#define GPEMUX1_VALUE		0x2a800000
#define GPEMUX2_VALUE		0x00014000
#define GPEGMUX1_VALUE		0x15400000
#define GPEGMUX2_VALUE		0x00000000

//
// Port F mux register values
//
// Pin B8 (GPIO164) to GPIO164 (mode 0)
#define GPFMUX1_MASK		0x00000300
#define GPFMUX1_VALUE		0x00000000
#define GPFGMUX1_VALUE		0x00000000

//
// Port B analog mode register values
//
// Pin D19 (GPIO42) to GPIO42 (mode 0)
// Pin C19 (GPIO43) to GPIO43 (mode 0)
#define GPBAMSEL_MASK		0x00000c00
#define GPBAMSEL_VALUE		0x00000000

//
// Input X-BAR register values
//
// Pin B18 (GPIO69) to INPUTXBAR1 (mode XBAR1)
#define INPUT1SELECT_VALUE	0x00000045

//*****************************************************************************
//
// Function prototype for function to write values above into their
// corresponding registers. This function is found in f2837xd_pinmux.c. Its use
// is completely optional.
//
//*****************************************************************************
extern void GPIO_setPinMuxConfig(void);
