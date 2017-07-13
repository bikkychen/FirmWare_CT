//
//      TMDX ALPHA RELEASE
//      Intended for product evaluation purposes
//
//###########################################################################
//
// FILE:	DSP28_Gpio.c
//
// TITLE:	DSP28 General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.55| 06 May 2002 | L.H. | EzDSP Alpha Release
//  0.56| 20 May 2002 | L.H. | No change
//  0.57| 27 May 2002 | L.H. | No change
//###########################################################################

#include "DSP28_Device.h"

//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known state.
//
void InitGpio(void)
{

// Set GPIO A port pins,AL(Bits 7:0)(input)-AH(Bits 15:8) (output) 8bits
// Input Qualifier =0, none
     EALLOW;


	 GpioMuxRegs.GPFMUX.bit.SCITXDA_GPIOF4 = 0;  //S1 
     GpioMuxRegs.GPFMUX.bit.SCIRXDA_GPIOF5 = 0;  //S2

     GpioMuxRegs.GPBMUX.bit.CAP6QI2_GPIOB10 = 0;   //CS1
	 GpioMuxRegs.GPBMUX.bit.C6TRIP_GPIOB15= 0;     //CS2

	 GpioMuxRegs.GPBMUX.bit.CAP5Q2_GPIOB9 = 0;    //UD1
	 GpioMuxRegs.GPBMUX.bit.C5TRIP_GPIOB14 = 0;   //UD2

	 GpioMuxRegs.GPBMUX.bit.CAP4Q1_GPIOB8 = 0;           //IN1	 
	 GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2 = 0;      //IN2

	 GpioMuxRegs.GPBMUX.bit.T3PWM_GPIOB6= 0;       //EN1
	 GpioMuxRegs.GPBMUX.bit.T4PWM_GPIOB7 = 0;       //EN2
	 GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1 = 0;        //EN3
	 GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0 = 0;        //EN4
	
     GpioMuxRegs.GPBMUX.bit.PWM11_GPIOB4 = 1;     //PWM11 CLK

     GpioMuxRegs.GPBMUX.bit.PWM7_GPIOB0 = 0;     //BEN
	 GpioMuxRegs.GPBMUX.bit.PWM8_GPIOB1 = 0;    //TEN
	 GpioMuxRegs.GPBMUX.bit.PWM9_GPIOB2 = 0;    //MUX
	 GpioMuxRegs.GPBMUX.bit.PWM10_GPIOB3 = 0;    //MISO
	 GpioMuxRegs.GPBMUX.bit.PWM12_GPIOB5 = 0; //SCLK

	 GpioMuxRegs.GPBMUX.bit.C4TRIP_GPIOB13=0; //EN

     S1=0;
     S2=0;
     CS1=1;
	 CS2=1;
     UD1=0;
	 UD2=0;
     IN1=0;
	 IN2=0;
     EN1=1;
	 EN2=1;
	 EN3=1;
	 EN4=1;
	 EN=1;

	 GpioMuxRegs.GPFDIR.all=0x0030;
     GpioMuxRegs.GPBDIR.all=0xe7f7;
	 GpioMuxRegs.GPEDIR.all=0x0007;

     S1=0;
     S2=0;
     CS1=1;
     CS2=1;
     UD1=0;
	 UD2=0;
     IN1=0;
	 IN2=0;
     EN1=1;
	 EN2=1;
	 EN3=1;
	 EN4=1;
	 EN=1;

     EDIS;

}	
	
//===========================================================================
// No more.
//===========================================================================
