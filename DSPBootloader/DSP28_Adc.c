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
	AdcRegs.ADCTRL1.bit.RESET=1;//ADCģ�鸴λ
	NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;

	AdcRegs.ADCTRL1.bit.SUSMOD=0;//
	AdcRegs.ADCTRL1.bit.ACQ_PS=0;//�������ڴ�С = 1 ADCLK
	AdcRegs.ADCTRL1.bit.CPS=0;//������ʱ��HSPCLK����Ƶ
	AdcRegs.ADCTRL1.bit.CONT_RUN=1;//������
	AdcRegs.ADCTRL1.bit.SEQ_CASC=0;//��������ģʽ
	
  	AdcRegs.ADCTRL3.bit.ADCBGRFDN=3;//��϶�Ͳο���·�ϵ�
 	for(i=0;i<50000;i++)	NOP;  //4.333ms 
	AdcRegs.ADCTRL3.bit.ADCPWDN=1;//ADC������·�ϵ�
	for(i=0;i<5000;i++)	NOP;
	AdcRegs.ADCTRL3.bit.ADCCLKPS=3; 
	AdcRegs.ADCTRL3.bit.SMODE_SEL=0;//˳�����ģʽ
	
	AdcRegs.MAX_CONV.bit.MAX_CONV=0;//���ת��ͨ����
	
//	AdcRegs.CHSELSEQ1.bit.CONV00=7;//ADC����ͨ��ѡ������
	
	AdcRegs.ADC_ST_FLAG.bit.INT_SEQ1_CLR=1;//���SEQ1�жϱ�ʾ
	AdcRegs.ADC_ST_FLAG.bit.INT_SEQ2_CLR=1;	//���SEQ2�жϱ�ʾ
	
	AdcRegs.ADCTRL2.bit.EVB_SOC_SEQ=0;//������������ʹ��EVB

	AdcRegs.ADCTRL2.bit.RST_SEQ1=1;//����������λ��CONV00
//	AdcRegs.ADCTRL2.bit.SOC_SEQ1=1;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=0;//ʹ��SEQ1���ж�����
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;//ÿ��SEQ1���н���ʱ�����ж�
	AdcRegs.ADCTRL2.bit.EVA_SOC_SEQ1=0;//EVA���ܴ���SEQ1
	AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1=0;//�ⲿ�źŲ��ܴ���SEQ1

}	

//===========================================================================
// No more.
//===========================================================================
