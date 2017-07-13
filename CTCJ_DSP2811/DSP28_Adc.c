//
//      TMDX ALPHA RELEASE
//      Intended for product evaluation purposes
//
//###########################################################################
//
// FILE:	DSP28_Adc.c
//
// TITLE:	DSP28 ADC Initialization & Support Functions.
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
// InitAdc: 
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
void InitAdc(void)
{unsigned char i;
	AdcRegs.ADCTRL1.bit.RESET=1;//ADC模块复位
	NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;

	AdcRegs.ADCTRL1.bit.SUSMOD=0;//
	AdcRegs.ADCTRL1.bit.ACQ_PS=0;//采样窗口大小 = 1 ADCLK
	AdcRegs.ADCTRL1.bit.CPS=0;//对外设时钟HSPCLK不分频
	AdcRegs.ADCTRL1.bit.CONT_RUN=1;//连续运
	AdcRegs.ADCTRL1.bit.SEQ_CASC=0;//级联工作模式
	
  	AdcRegs.ADCTRL3.bit.ADCBGRFDN=3;//带隙和参考电路上电
 	for(i=0;i<50000;i++)	NOP;  //4.333ms 
	AdcRegs.ADCTRL3.bit.ADCPWDN=1;//ADC其它电路上电
	for(i=0;i<5000;i++)	NOP;
	AdcRegs.ADCTRL3.bit.ADCCLKPS=3; 
	AdcRegs.ADCTRL3.bit.SMODE_SEL=0;//顺序采样模式
	
	AdcRegs.MAX_CONV.bit.MAX_CONV=0;//最大转换通道数
	
//	AdcRegs.CHSELSEQ1.bit.CONV00=7;//ADC输入通道选择排序
	
	AdcRegs.ADC_ST_FLAG.bit.INT_SEQ1_CLR=1;//清除SEQ1中断标示
	AdcRegs.ADC_ST_FLAG.bit.INT_SEQ2_CLR=1;	//清除SEQ2中断标示
	
	AdcRegs.ADCTRL2.bit.EVB_SOC_SEQ=0;//级联排序器不使能EVB

	AdcRegs.ADCTRL2.bit.RST_SEQ1=1;//将排序器复位到CONV00
//	AdcRegs.ADCTRL2.bit.SOC_SEQ1=1;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=0;//使能SEQ1的中断申请
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;//每个SEQ1序列结束时产生中断
	AdcRegs.ADCTRL2.bit.EVA_SOC_SEQ1=0;//EVA不能触发SEQ1
	AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1=0;//外部信号不能触发SEQ1

}	

//===========================================================================
// No more.
//===========================================================================
