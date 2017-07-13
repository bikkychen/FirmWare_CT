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

    EvbRegs.EXTCONB.bit.INDCOE = 1;//����ʹ�ܱȽ����ģʽ
	EvbRegs.ACTRB.all = 0x0aaa;//�ռ�ʸ��������
	EvbRegs.DBTCONB.all = 0;//0x08ec;//������ʱ������
	EvbRegs.CMPR6 = 1;
	EvbRegs.COMCONB.all = 0xa4e0;

	EvbRegs.EXTCONB.bit.INDCOE = 1;
    EvbRegs.GPTCONB.all = 0x0012;
    EvbRegs.T3PR = 1;    //��ʱ��1����ֵ
    EvbRegs.T3CMPR = 1;  //��ʱ��1�Ƚ�ֵ
    EvbRegs.T3CNT = 0x0000;  //��ʱ��1��ֵ
	EvbRegs.T3CON.all = 0x1342;//������ģʽ ,HSPCLK����Ƶ,


}	
	
//===========================================================================
// No more.
//===========================================================================
