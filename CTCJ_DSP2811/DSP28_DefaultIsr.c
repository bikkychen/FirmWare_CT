//###########################################################################
//
// FILE:	DSP28_DefaultIsr.c
//
// TITLE:	DSP28 Device Default Interrupt Service Routines.
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.55| 06 May 2002 | L.H. | EzDSP Alpha Release
//  0.56| 20 May 2002 | L.H. | No change
//  0.57| 27 May 2002 | L.H. | No change
//###########################################################################

//20151215:
//1���ɻ�������ֵʱ����ʱ����һ�ξ���ʱ���Ի�ü�ʱ��ֵ
//2��ȥ�����������ڷ�ֵ�·������Ṧ�ܣ������鿴����
//3�����ӻ������ײ���ֵ��У׼����

#include "DSP28_Device.h"
#include "Flash281x_API_Library.h"


/*
extern char SciCom;
extern char FlowBegin;
extern char StopFlag;
extern char FlowReq;
extern char DataReqFlag,RanReqFlag;
extern char HaltFlag;
extern char Fpar;
extern char ResetReqFlag;
extern char VerFlag,CalFlag;
extern float32 ApiVersion;
extern Uint16  Delv1,Delv2,Delv3,Delv4;

Uint16 Col=0,Col2=0,Colt,cn;
Uint16 ColBegin=0,ct;
Uint32 cl;
Uint16 tws;
char RxBuf[74];
char RxCn=0;
char RxFlag=0;
*/
extern Uint16 tp[8];//�������ײ��ж���ֵ + ���������ڱ궨��ֵ

extern long ADSData;
extern int16 td,pd,wd;

extern FLASH_ST ProgStatus;
extern FLASH_ST EraseStatus;
extern Uint16 Status;
extern Uint16 *Flash_Ptr;

unsigned long tl;
Uint16 tpbuf;

typedef union RR
{float f;
 char c[2];
 }b;

extern union RR RDa[4];
extern char RDDa[4];
extern void UARTPutByte(char b);
extern unsigned char AbsTime(unsigned char ch);
extern void ADS1222_sample(unsigned char Ch);
extern void ISRSciCom05(void);
extern void ISRSciCom0A(void);
extern unsigned char CalThr(void);
extern void CalP(void);//������������ֵ
 
 
extern void StreamDataInterpretation_B(unsigned char b);

//---------------------------------------------------------------------------
// INT13, INT14, NMI, XINT1, XINT2 Default ISRs:
//
interrupt void INT13_ISR(void)     // INT13 or CPU-Timer1
{
  // Insert ISR Code here
  
  // Next two lines for debug only - remove after inserting
  // ISR Code
     asm ("      ESTOP0");
     for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void INT14_ISR(void)     // CPU-Timer2
{
  // Insert ISR Code here
  
  // Next two lines for debug only - remove after inserting
  // ISR Code
     asm ("      ESTOP0");
     for(;;);
 
  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void NMI_ISR(void)       // Non-maskable interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void  XINT1_ISR(void)
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}     

interrupt void  XINT2_ISR(void)
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

//---------------------------------------------------------------------------
// DATALOG, RTOSINT, EMUINT, RTOS Default ISRs:
//
interrupt void DATALOG_ISR(void)   // Datalogging interrupt
{
  // Insert ISR Code here
 
  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;

}

interrupt void RTOSINT_ISR(void)   // RTOS interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void EMUINT_ISR(void)    // Emulation interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

//---------------------------------------------------------------------------
// ILLEGAL Instruction Trap ISR:
//
interrupt void ILLEGAL_ISR(void)   // Illegal operation TRAP
{
  // Insert ISR Code here
  asm("          ESTOP0");
  for(;;);

  // Uncomment this line after adding ISR Code 
  // return;

}

//---------------------------------------------------------------------------
// USER Traps Default ISRs:
//
interrupt void USER0_ISR(void)     // User Defined trap 0
{
 // Insert ISR Code here
 
  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void USER1_ISR(void)     // User Defined trap 1
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void USER2_ISR(void)     // User Defined trap 2
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;

}

interrupt void USER3_ISR(void)     // User Defined trap 3
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;

}

interrupt void USER4_ISR(void)     // User Defined trap 4
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void USER5_ISR(void)     // User Defined trap 5
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void USER6_ISR(void)     // User Defined trap 6
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void USER7_ISR(void)     // User Defined trap 7
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;

}

interrupt void USER8_ISR(void)     // User Defined trap 8
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void USER9_ISR(void)     // User Defined trap 9
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;

}

interrupt void USER10_ISR(void)    // User Defined trap 10
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;

}

interrupt void USER11_ISR(void)    // User Defined trap 11
{
  // Insert ISR Code here

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;

}

//---------------------------------------------------------------------------
// ADC Default ISR: 
//
interrupt void  ADCINT_ISR(void)     // ADC
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP1; 
  
  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

//---------------------------------------------------------------------------
// CPU Timer 0 Default ISR: 
//
interrupt void  TINT0_ISR(void)      // CPU-Timer 0
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP1; 

  // Next two lines for debug only - remove after inserting
  // ISR Code
     asm ("      ESTOP0");
     for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

//---------------------------------------------------------------------------
// Watchdog Default ISR: 
//
interrupt void  WAKEINT_ISR(void)    // WD
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP1; 
  
  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

//---------------------------------------------------------------------------
// EV-A Default ISRs: 
//
interrupt void PDPINTA_ISR( void )    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);
   
  // Uncomment this line after adding ISR Code 
  // return;

}

interrupt void CMP1INT_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CMP2INT_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);
   
  // Uncomment this line after adding ISR Code 
  // return;

}

interrupt void CMP3INT_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP2;
  
  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T1PINT_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP2;
  
  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T1CINT_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T1UFINT_ISR(void)   // EV-A
{
  // Insert ISR Code here

   
  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP2;
  
  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T1OFINT_ISR(void)   // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}
      
interrupt void T2PINT_ISR(void)     // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T2CINT_ISR(void)     // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);
   
  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T2UFINT_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T2OFINT_ISR(void)    // EV-A
{
  // Insert ISR Code here
  
  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP3;  

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CAPINT1_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP3;  

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CAPINT2_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only - remove after inserting ISR
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CAPINT3_ISR(void)    // EV-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}
      
//---------------------------------------------------------------------------
// EV-B Default ISRs: 
//
interrupt void  PDPINTB_ISR(void)   // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CMP4INT_ISR(void)    // EV-B
{
  // Insert ISR Code here
  
  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP4;  

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CMP5INT_ISR(void)    // EV-B
{
  // Insert ISR Code here
  
  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CMP6INT_ISR(void)    // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T3PINT_ISR(void)     // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T3CINT_ISR(void)     // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T3UFINT_ISR(void)    // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T3OFINT_ISR(void)    // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}
     
interrupt void T4PINT_ISR(void)    // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T4CINT_ISR(void)    // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP5;
 
  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T4UFINT_ISR(void)   // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void T4OFINT_ISR(void)   // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CAPINT4_ISR(void)   // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CAPINT5_ISR(void)   // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void CAPINT6_ISR(void)   // EV-B
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);
  
  // Uncomment this line after adding ISR Code 
  // return;
}

//---------------------------------------------------------------------------
// McBSP-A Default ISRs: 
//
interrupt void MRINTA_ISR(void)     // McBSP-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void MXINTA_ISR(void)     // McBSP-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

//---------------------------------------------------------------------------
// SPI-A Default ISRs: 
//
interrupt void SPIRXINTA_ISR(void)    // SPI-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void SPITXINTA_ISR(void)     // SPI-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}



//---------------------------------------------------------------------------
// SCI-A Default ISRs: 
//
interrupt void SCIRXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void SCITXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}


//---------------------------------------------------------------------------
// SCI-B Default ISRs: 
//
interrupt void SCIRXINTB_ISR(void)     // SCI-B
{
/*
   PieCtrl.PIEACK.bit.ACK9 = 1;

   if(ScibRx_Ready() == 1)
	{
		RxBuf[RxCn] = ScibRegs.SCIRXBUF.all;
		RxCn++;
		if(RxCn==3)
		{
		 SciCom = RxBuf[2];
		}

		if(RxCn==9)
		{
		  if(SciCom==0x88)// дϵ�������ǳ�����������Ҫ����һЩ�ֽ�
		  {
		    RxFlag=0;
		  }
		  else
		  {
		    RxFlag=1;
			RxCn=0;
		  }
		}

		if(RxCn==74)
		{
		   RxFlag=1;
		   RxCn=0;
		}
	}

if(RxFlag>0)
	{
		
    RxFlag=0;
	if(Col2==0xb0)//����ϵ������״̬, //FLASHI      : origin = 0x3DA000, length = 0x002000 
		{		
		    cn++;
			 if(cn&0x0001)
			  {ct=SciCom;
			   ct<<=8;}
			 else
			  { ct|=SciCom;
                Status = Flash_Program((Uint16*)(0x3DA000+(cn>>1)-1),&ct,1,&ProgStatus);//дһ��word��ʱ35us
			  }
		  if(cn>=216)
		    {Col2=0;
			cn=0;
			}			 
		}
	else
	   {
        switch(SciCom&0xf0)
		 {
		case 0x00:  
		  {
			if(SciCom == 0x00)  //��λ//
			{ 				 
		     FlowBegin = 0;
		     StopFlag=0;
		     FlowReq=0;
			 DataReqFlag=0;
             RanReqFlag=0;
			 ResetReqFlag=0;
			 ColBegin=0;	 			
			 UARTPutByte(0x55);
			 UARTPutByte(0x40);
			 UARTPutByte(0x88);
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 if(VerFlag==3)
			   {ResetReqFlag=1;
			   }
			  VerFlag=0;
		 	}		
			break;
		  }
		case 0x40:      //�������� 
		  	{FlowBegin=1;
		  	 FlowReq=1;
			 StopFlag=0;
			 Fpar=SciCom; 

			 RDa[0].f=123456.789;
			
			 UARTPutByte(RDa[0].c[1]); 
			 UARTPutByte((RDa[0].c[1])>>8);
			 UARTPutByte(RDa[0].c[0]);
			 UARTPutByte((RDa[0].c[0])>>8);
			 
			 
			 
			 RDa[1].f=987654.321;
			 UARTPutByte(RDa[1].c[1]); 
			 UARTPutByte((RDa[1].c[1])>>8);
			 UARTPutByte(RDa[1].c[0]);
			 UARTPutByte((RDa[1].c[0])>>8);
			 

			 break;
			 }
		case 0x50:       //�̼��汾��//
		   { 
		     UARTPutByte(0x55);
			 UARTPutByte(0x40);
			 UARTPutByte(0x88);
			 UARTPutByte((VM<<4)+(VS));
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 UARTPutByte(0x00);
			 VerFlag++;
		     break;
	      }

		case 0x60:	 
		 {
		  if(SciCom == 0x60)//����궨״̬
		  {
		    HaltFlag=0xff;

		   UARTPutByte(0x60);
		   }
		   else if(SciCom == 0x65)//���궨����֡��
		   {
		    ColBegin=0;
			cl=(Uint32)Flash_Ptr-(Uint32)0x003D8000;
			cl>>=1;                          //������(֡��)��Ϊ����
		    UARTPutByte(cl>>8);
		    UARTPutByte(cl);
		   }
		   else if(SciCom == 0x66)//��˳����ȡ�궨���ݣ�ÿ�η�4�ֽڣ��൱��2�֣�Ҳ�൱��һ��������
		   {
		    ct=(*(Uint16*)(0x003D8000+ColBegin));
		    UARTPutByte(ct>>8);
		    UARTPutByte(ct);
            ColBegin++;

			ct=(*(Uint16*)(0x003D8000+ColBegin));
		    UARTPutByte(ct>>8);
		    UARTPutByte(ct);
            ColBegin++;
		    }		  
		   break;
		  }
		case 0x20:	//ѹ������//
		 {  
		    // pd=1234;
			tws=pd+=32768;
		     UARTPutByte(tws);
			 UARTPutByte(tws>>8);
		   break;
		 }
		case 0x30:	//�¶Ȳ���//
		 { 
		    //td=4321;
			tws=td+=32768;
		     UARTPutByte(tws);
			 UARTPutByte(tws>>8);
		   break;
		 }

		 case 0xa0:
		 {
			if(SciCom == 0xa0)//�·�ѹ���궨ϵ��
		  	{
			     //FLASHI      : origin = 0x3DA000, length = 0x002000 
				  Status = Flash_Erase(SECTORI,&EraseStatus); //������ʾ�������Ժ�ʱԼ1.3s,datasheet��˵4Ƭ8K��FLASH������ʱ��10s����ÿƬ��ʱ2.5s		  		   
			     UARTPutByte(Status>>8);//����0x0000��ʾ�����ɹ�
				 UARTPutByte(Status);

				 if(Status==0x0000)
				 {
				 Col2=0xb0;
				 cn=0;
				 }
		     
			 }
		     else if(SciCom == 0xa1)//�ϴ�ѹ���궨ϵ����ʼ
			 {
			   Col=0;
			 }
			 else if(SciCom == 0xa2)//�ϴ�ѹ���¶ȱ궨����
			 {
			  Colt=*(Uint16 *)(0x3DA000+Col);//FLASHI      : origin = 0x3DA000, length = 0x002000 
			  UARTPutByte(Colt>>8);
			  UARTPutByte(Colt);
			  Col++;
			 }
			 else if(SciCom == 0xa3)//���ѹ���¶ȱ궨����
			 {
			  //FLASHJ      : origin = 0x3D8000, length = 0x002000
		     Status = Flash_Erase(SECTORJ,&EraseStatus); //������ʾ�������Ժ�ʱԼ1.3s,datasheet��˵4Ƭ8K��FLASH������ʱ��10s����ÿƬ��ʱ2.5s		  		   
		     UARTPutByte(Status>>8);//����0x0000��ʾ�����ɹ�
			 UARTPutByte(Status);
			 if(Status==STATUS_SUCCESS)
			   Flash_Ptr = (Uint16 *)0x003D8000;
			 }
			 break;
		 }
 
		case 0xb0:
		{
		  
		  break;
		}

		 case 0xe0:	//������������ֵ//       
		  {
			 AbsTime(1);
			 tl=Delv1;
			 tl=tl*3000/4096;//3000Ϊ�ڲ�AD�Ĳο���ѹ3V,4096Ϊ�ڲ�ADλ��12λ��2^12=4096
			 UARTPutByte(tl>>8);
			 UARTPutByte(tl);

			 AbsTime(2);
			 tl=Delv2;
			 tl=tl*3000/4096;//3000Ϊ�ڲ�AD�Ĳο���ѹ3V,4096Ϊ�ڲ�ADλ�12λ��2^12=4096
			 UARTPutByte(tl>>8);
			 UARTPutByte(tl);

			 AbsTime(3);
			 tl=Delv3;
			 tl=tl*3000/4096;//3000Ϊ�ڲ�AD�Ĳο���ѹ3V,4096Ϊ�ڲ�ADλ��12λ��2^12=4096
			 UARTPutByte(tl>>8);
			 UARTPutByte(tl);
	
	         AbsTime(4);
			 tl=Delv4;
			 tl=tl*3000/4096;//3000Ϊ�ڲ�AD�Ĳο���ѹ3V,4096Ϊ�ڲ�ADλ��12λ��2^12=4096
			 UARTPutByte(tl>>8);
			 UARTPutByte(tl);

			 CalP();//������������ֵ
		     break;
	      }
		case 0xf0:	//��ȡ��������//
		  {  FlowReq=0;
		     FlowBegin = 0;
		     StopFlag=0xffff;		     
			 DataReqFlag=SciCom;
		     break;
		  }
		default:
		     break;
		}
	   }
  	}*/
	//  DINT;//�ر����ж�	
   // if(ScibRegs.SCIRXST.bit.RXRDY == 1)
 

	StreamDataInterpretation_B(ScibRegs.SCIRXBUF.bit.RXDT);
    PieCtrl.PIEACK.all |= BIT9;   //��Ӧ�ж�
	EINT;   //�����ж�
}

interrupt void SCITXINTB_ISR(void)     // SCI-B
{
   asm ("      ESTOP0");
   for(;;);
}


//---------------------------------------------------------------------------
// CAN-A Default ISRs: 
//
interrupt void ECAN0INTA_ISR(void)  // eCAN-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}

interrupt void ECAN1INTA_ISR(void)  // eCAN-A
{
  // Insert ISR Code here

  // To recieve more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrl.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only - remove after inserting
  // ISR Code
   asm ("      ESTOP0");
   for(;;);

  // Uncomment this line after adding ISR Code 
  // return;
}


//---------------------------------------------------------------------------
// Catch All Default ISRs: 
//
interrupt void PIE_RESERVED(void)  // Reserved space.  For test.
{
  asm ("      ESTOP0");
  for(;;);
}

interrupt void rsvd_ISR(void)          // for test
{
  asm ("      ESTOP0");
  for(;;);
}

//===========================================================================
// No more.
//===========================================================================

