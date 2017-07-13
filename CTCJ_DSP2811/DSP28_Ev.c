//
//      TMDX ALPHA RELEASE
//      Intended for product evaluation purposes
//
//###########################################################################
//
// FILE:	DSP28_Ev.c
//
// TITLE:	DSP28 Event Manager Initialization & Support Functions.
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
// InitEv: 
//---------------------------------------------------------------------------
// This function initializes to a known state.
//
void InitEv(void)
{
	EALLOW;
	GpioMuxRegs.GPBMUX.bit.PWM11_GPIOB4 = 1;  //PWM11 CLK
	EDIS;

    EvbRegs.EXTCONB.bit.INDCOE = 1;//单独使能比较输出模式
	EvbRegs.ACTRB.all = 0x0aaa;//空间矢量不动作
	EvbRegs.DBTCONB.all = 0;//0x08ec;//死区定时器启动
	EvbRegs.CMPR6 = 1;
	EvbRegs.COMCONB.all = 0xa4e0;

	EvbRegs.EXTCONB.bit.INDCOE = 1;
    EvbRegs.GPTCONB.all = 0x0012;
    EvbRegs.T3PR = 1;    //定时器1周期值
    EvbRegs.T3CMPR = 1;  //定时器1比较值
    EvbRegs.T3CNT = 0x0000;  //定时器1初值
	EvbRegs.T3CON.all = 0x1342;//连续增模式 ,HSPCLK不分频,


}	
	
//===========================================================================
// No more.
//===========================================================================
