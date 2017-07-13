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
//1¡¢²É»»ÄÜÆ÷·ùÖµÊ±ÏÈÁÙÊ±²ÉÑùÒ»´Î¾ø¶ÔÊ±¼äÒÔ»ñµÃ¼´Ê±·ùÖµ
//2¡¢È¥µô»»ÄÜÆ÷³§ÄÚ·ùÖµÏÂ·¢ºÍÉÏÌá¹¦ÄÜ£¬±£Áô²é¿´¹¦ÄÜ
//3¡¢Ôö¼Ó»»ÄÜÆ÷Ê×²¨ãÐÖµ×ÔÐ£×¼¹¦ÄÜ

#include "DSP28_Device.h"
#include "Flash281x_API_Library.h"

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

extern Uint16 tp[8];//»»ÄÜÆ÷Ê×²¨ÅÐ¶¨ãÐÖµ + »»ÄÜÆ÷³§ÄÚ±ê¶¨·ùÖµ

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
extern void ISRSciCom06(void);
extern void ISRSciCom0A(void);
extern void ISRSciCom0B(void);
extern unsigned char CalThr(void);
extern void CalP(void);//µ÷Õû»»ÄÜÆ÷·ùÖµ
 

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
   PieCtrl.PIEACK.bit.ACK9 = 1;

if(ScibRx_Ready() == 1)
	{
		SciCom = ScibRegs.SCIRXBUF.all;

	if(Col2==0xb0)//½øÈëÏµÊý½ÓÊÕ×´Ì¬, //FLASHI      : origin = 0x3DA000, length = 0x002000 
		{		
		    cn++;
			 if(cn&0x0001)
			  {ct=SciCom;
			   ct<<=8;}
			 else
			  { ct|=SciCom;
                Status = Flash_Program((Uint16*)(0x3DA000+(cn>>1)-1),&ct,1,&ProgStatus);//Ð´Ò»¸öwordºÄÊ±35us
			  }


		  if(cn>=216)
		    {Col2=0;
			cn=0;
			}			 
		}
/*	else if(Col2==0xb1)//Èë³§ÄÚ·ùÖµ½ÓÊÕ×´Ì¬, //FLASHI      : origin = 0x3DA000, length = 0x002000 
		{		
          if((cn&0x0001)==0)//ÏÈÊÕµÍ×Ö½Ú
			  {
			   tpbuf=SciCom;
			   }
		  else  //ÔÙÊÕ¸ß×Ö½Ú
			  {		   
			   ct=SciCom;
			   ct<<=8;

			   tpbuf|=ct;

			   if(tpbuf>1000)
			     tp[4+cn/2]=tpbuf;
			  }
		  cn++;

		  if(cn>=8)
		    {Col2=0;
			cn=0;
			Status = Flash_Program((Uint16 *)(0x3F4000),tp,8,&ProgStatus);	
			}			 
		}*/
	else
	   {
        switch(SciCom&0xf0)
		 {
		case 0x00:  
		  {
			if(SciCom == 0x00)  //¸´Î»
			{ 				 
		     FlowBegin = 0;
		     StopFlag=0;
		     FlowReq=0;
			 DataReqFlag=0;
             RanReqFlag=0;
			 ResetReqFlag=0;
			 ColBegin=0;	 
			 ADSData=wd;
			 ADSData*=2727;
			 ADSData/=3000;
		     UARTPutByte(ADSData>>8);
			 UARTPutByte(ADSData);
			 if(VerFlag==3)
			   {ResetReqFlag=1;
			   }
			  VerFlag=0;
			 break;
		 	}
			else if(SciCom == 0x05)//µç×èµÝ¼õ£¬²»¶ÔÓÃ»§¿ª·¢´Ë¹¦ÄÜ£¬½ö³§ÄÚÁ÷Á¿±ê¼ìÓÃ
			{
			 ISRSciCom05();
			 break;
			}
			else if(SciCom == 0x0a)	//µç×èµÝÔö£¬²»¶ÔÓÃ»§¿ª·¢´Ë¹¦ÄÜ£¬½ö³§ÄÚÁ÷Á¿±ê¼ìÓÃ
			{			
			 ISRSciCom0A();
			 break;			 
			}
			else if(SciCom == 0x06)//µç×èµÝ¼õ£¬²»¶ÔÓÃ»§¿ª·¢´Ë¹¦ÄÜ£¬½ö³§ÄÚÁ÷Á¿±ê¼ìÓÃ
			{
			 ISRSciCom06();
			 break;
			}
			else if(SciCom == 0x0b)	//µç×èµÝÔö£¬²»¶ÔÓÃ»§¿ª·¢´Ë¹¦ÄÜ£¬½ö³§ÄÚÁ÷Á¿±ê¼ìÓÃ
			{			
			 ISRSciCom0B();
			 break;			 
			}
			else
				break;
		}
		case 0x40:      //Á÷Á¿²ÉÑù 
		  	{FlowBegin=1;
		  	 FlowReq=1;
			 StopFlag=0;
			 Fpar=SciCom; 
			if(SciCom == 0x4F)//SZTA-5×¨ÓÃ£¬¸¡µãÐÍ
			{

			// RDa[0].f=ApiVersion;
			 UARTPutByte((RDa[0].c[1])>>8);
			 UARTPutByte(RDa[0].c[1]);
			 UARTPutByte((RDa[0].c[0])>>8);
			 UARTPutByte(RDa[0].c[0]);
			 
			 UARTPutByte((RDa[1].c[1])>>8);
			 UARTPutByte(RDa[1].c[1]);
			 UARTPutByte((RDa[1].c[0])>>8);
			 UARTPutByte(RDa[1].c[0]);

			 UARTPutByte((RDa[2].c[1])>>8);
			 UARTPutByte(RDa[2].c[1]);
			 UARTPutByte((RDa[2].c[0])>>8);
			 UARTPutByte(RDa[2].c[0]);

			 UARTPutByte((RDa[3].c[1])>>8);
			 UARTPutByte(RDa[3].c[1]);
			 UARTPutByte((RDa[3].c[0])>>8);
			 UARTPutByte(RDa[3].c[0]);
			 }
			 else if(SciCom == 0x44)//ÕûÐÍ
			 {
			 UARTPutByte((RDDa[0])>>8);
			 UARTPutByte(RDDa[0]);
			 UARTPutByte((RDDa[1])>>8);
			 UARTPutByte(RDDa[1]);
			 UARTPutByte((RDDa[2])>>8);
			 UARTPutByte(RDDa[2]);
			 UARTPutByte((RDDa[3])>>8);
			 UARTPutByte(RDDa[3]);
			 }
			 else if(SciCom == 0x45)//¶Á»»ÄÜÆ÷ãÐÖµ
			 { 
				UARTPutByte(tp[0]);
				UARTPutByte(tp[1]);
				UARTPutByte(tp[2]);
				UARTPutByte(tp[3]);
			 }
			 
			 else if(SciCom == 0x46)//Ð´»»ÄÜÆ÷ãÐÖµ
			 { //FLASHB      : origin = 0x3F4000, length = 0x002000
			 	Status = Flash_Erase(SECTORB,&EraseStatus); //³£ÎÂÏÂÊ¾²¨Æ÷²âÊÔºÄÊ±Ô¼1.3s,datasheetÉÏËµ4Æ¬8KµÄFLASH²Á³ýºÄÊ±¹²10s£¬ÔòÃ¿Æ¬ºÄÊ±2.5s		  		   
		    	UARTPutByte(Status>>8);//·µ»Ø0x0000±íÊ¾²Ù×÷³É¹¦
			 	UARTPutByte(Status);
			 	Status = Flash_Program((Uint16 *)(0x3F4000),tp,8,&ProgStatus);			 
			 }
			 else if(SciCom == 0x47)//Ôö´óÉÏË³»»ÄÜÆ÷ãÐÖµ
			 {
			    if(tp[0]<255)
			  		tp[0]+=1;
				UARTPutByte(0); 
			 	UARTPutByte(tp[0]);
			 }
			 else if(SciCom == 0x48)//¼õÐ¡ÉÏË³»»ÄÜÆ÷ãÐÖµ
			 {
			    if(tp[0]>0)
				  tp[0]-=1;
				UARTPutByte(0); 
			 	UARTPutByte(tp[0]);
			 }
			 else if(SciCom == 0x49)//Ôö´óÉÏÄæ»»ÄÜÆ÷ãÐÖµ
			 {
			 if(tp[1]<255)
				tp[1]+=1;
				UARTPutByte(0); 
			 	UARTPutByte(tp[1]);
			 }
			 else if(SciCom == 0x4a)//¼õÐ¡ÉÏÄæ»»ÄÜÆ÷ãÐÖµ
			 {
			  if(tp[1]>0)
				tp[1]-=1;
				UARTPutByte(0); 
			 	UARTPutByte(tp[1]);
			 }
			 else if(SciCom == 0x4b)//Ôö´óÏÂË³»»ÄÜÆ÷ãÐÖµ
			 {
			  if(tp[2]<255)
				tp[2]+=1;
				UARTPutByte(tp[2]>>8); 
			 	UARTPutByte(tp[2]);
			 }
			 else if(SciCom == 0x4c)//¼õÐ¡ÏÂË³»»ÄÜÆ÷ãÐÖµ
			 {
			  if(tp[2]>0)
				tp[2]-=1;
				UARTPutByte(tp[2]>>8); 
			 	UARTPutByte(tp[2]);
			 }
			 else if(SciCom == 0x4d)//Ôö´óÏÂÄæ»»ÄÜÆ÷ãÐÖµ
			 {
			  if(tp[3]<255)
				tp[3]+=1;
				UARTPutByte(tp[3]>>8); 
			 	UARTPutByte(tp[3]);
			 }
			 else if(SciCom == 0x4e)//¼õÐ¡ÏÂÄæ»»ÄÜÆ÷ãÐÖµ
			 {
			 if(tp[3]>0)
				tp[3]-=1;
				UARTPutByte(tp[3]>>8); 
			 	UARTPutByte(tp[3]);	
			 }
			 break;
		   }
		case 0x50:       //¹Ì¼þ°æ±¾ºÅ
		   { 
	         while(ScibTx_Ready() != 1);
		     ScibRegs.SCITXBUF =(VM<<4)+(VS);
			 VerFlag++;
		     break;
	      }

		case 0x60:	 
		 {
		  if(SciCom == 0x60)//½øÈë±ê¶¨×´Ì¬
		  {
		    HaltFlag=0xff;

		   UARTPutByte(0x60);
		   }
		   else if(SciCom == 0x65)//¶Á±ê¶¨Êý¾ÝÖ¡Êý
		   {
		    ColBegin=0;
			cl=(Uint32)Flash_Ptr-(Uint32)0x003D8000;
			cl>>=1;                          //½«×ÖÊý(Ö¡Êý)±äÎªµãÊý
		    UARTPutByte(cl>>8);
		    UARTPutByte(cl);
		   }
		   else if(SciCom == 0x66)//°´Ë³ÐòÌáÈ¡±ê¶¨Êý¾Ý£¬Ã¿´Î·¢4×Ö½Ú£¬Ïàµ±ÓÚ2×Ö£¬Ò²Ïàµ±ÓÚÒ»¸ö²ÉÑùµã
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
		case 0x20:	//Ñ¹Á¦²ÉÑù
		 {  
		     UARTPutByte(pd>>8);
			 UARTPutByte(pd);
		   break;
		 }
		case 0x30:	//ÎÂ¶È²ÉÑù
		 { 
		     UARTPutByte(td>>8);
			 UARTPutByte(td);
		   break;
		 }
		 case 0xb0:
		 {
		 	if(SciCom == 0xb0)//ÏÂ·¢Ñ¹Á¦±ê¶¨ÏµÊý
		  	{
			     //FLASHI      : origin = 0x3DA000, length = 0x002000 
				  Status = Flash_Erase(SECTORI,&EraseStatus); //³£ÎÂÏÂÊ¾²¨Æ÷²âÊÔºÄÊ±Ô¼1.3s,datasheetÉÏËµ4Æ¬8KµÄFLASH²Á³ýºÄÊ±¹²10s£¬ÔòÃ¿Æ¬ºÄÊ±2.5s		  		   
			     UARTPutByte(Status>>8);//·µ»Ø0x0000±íÊ¾²Ù×÷³É¹¦
				 UARTPutByte(Status);

				 if(Status==0x0000)
				 {
				 Col2=0xb0;
				 cn=0;
				 }
		     
			 }
			 /*
			 else  if(SciCom == 0xb1)//ÏÂ·¢·ùÖµ
			 {
			   	//FLASHB      : origin = 0x3F4000, length = 0x002000
			  	 Status = Flash_Erase(SECTORB,&EraseStatus); //³£ÎÂÏÂÊ¾²¨Æ÷²âÊÔºÄÊ±Ô¼1.3s,datasheetÉÏËµ4Æ¬8KµÄFLASH²Á³ýºÄÊ±¹²10s£¬ÔòÃ¿Æ¬ºÄÊ±2.5s		  		   
		    	UARTPutByte(Status>>8);//·µ»Ø0x0000±íÊ¾²Ù×÷³É¹¦
			 	UARTPutByte(Status);

				if(Status==0x0000)
				{
			    Col2=0xb1;
			    cn=0;
				}			 
			 }
			 else  if(SciCom == 0xb2)//ÉÏ´«·ùÖµ 
			 { 
			    UARTPutByte(tp[4]>>8);
				UARTPutByte(tp[4]);
				UARTPutByte(tp[5]>>8);
				UARTPutByte(tp[5]);
				UARTPutByte(tp[6]>>8);
				UARTPutByte(tp[6]);
				UARTPutByte(tp[7]>>8);
				UARTPutByte(tp[7]);			 		  		  
			 }*/
			 else if(SciCom == 0xb4)//Ò»¼üÐ£×¼
			 {			 
			   CalFlag=CalThr();


				  //FLASHB      : origin = 0x3F4000, length = 0x002000
			 	Status = Flash_Erase(SECTORB,&EraseStatus); //³£ÎÂÏÂÊ¾²¨Æ÷²âÊÔºÄÊ±Ô¼1.3s,datasheetÉÏËµ4Æ¬8KµÄFLASH²Á³ýºÄÊ±¹²10s£¬ÔòÃ¿Æ¬ºÄÊ±2.5s			   
			 	Status = Flash_Program((Uint16 *)(0x3F4000),tp,4,&ProgStatus);	

				UARTPutByte(CalFlag);
			 }

			 break;
		 }
		 case 0xc0:
		 {
		     if(SciCom == 0xc0)//ÉÏ´«Ñ¹Á¦±ê¶¨ÏµÊý¿ªÊ¼
			 {
			   Col=0;
			 }
			 else if(SciCom == 0xc1)//ÉÏ´«±ê¶¨ÏµÊý
			 {
			  Colt=*(Uint16 *)(0x3DA000+Col);//FLASHI      : origin = 0x3DA000, length = 0x002000 
			  UARTPutByte(Colt>>8);
			  UARTPutByte(Colt);
			  Col++;
			 }
			 break;
		 }
		 case 0xd0:	//Çå¿ÕÑ¹Á¦ÎÂ¶È±ê¶¨Êý¾Ý
		  {
		   //FLASHJ      : origin = 0x3D8000, length = 0x002000
		     Status = Flash_Erase(SECTORJ,&EraseStatus); //³£ÎÂÏÂÊ¾²¨Æ÷²âÊÔºÄÊ±Ô¼1.3s,datasheetÉÏËµ4Æ¬8KµÄFLASH²Á³ýºÄÊ±¹²10s£¬ÔòÃ¿Æ¬ºÄÊ±2.5s		  		   
		     UARTPutByte(Status>>8);//·µ»Ø0x0000±íÊ¾²Ù×÷³É¹¦
			 UARTPutByte(Status);
			 if(Status==STATUS_SUCCESS)
			   Flash_Ptr = (Uint16 *)0x003D8000;			 
		     break;
	      }
		 case 0xe0:	//²ÉÑù»»ÄÜÆ÷·ùÖµ       
		  {
			 AbsTime(1);
			 tl=Delv1;
			 tl=tl*3000/4096;//3000ÎªÄÚ²¿ADµÄ²Î¿¼µçÑ¹3V,4096ÎªÄÚ²¿ADÎ»Êý12Î»£¬2^12=4096
			 UARTPutByte(tl>>8);
			 UARTPutByte(tl);

			 AbsTime(2);
			 tl=Delv2;
			 tl=tl*3000/4096;//3000ÎªÄÚ²¿ADµÄ²Î¿¼µçÑ¹3V,4096ÎªÄÚ²¿ADÎ»ý12Î»£¬2^12=4096
			 UARTPutByte(tl>>8);
			 UARTPutByte(tl);

			 AbsTime(3);
			 tl=Delv3;
			 tl=tl*3000/4096;//3000ÎªÄÚ²¿ADµÄ²Î¿¼µçÑ¹3V,4096ÎªÄÚ²¿ADÎ»Êý12Î»£¬2^12=4096
			 UARTPutByte(tl>>8);
			 UARTPutByte(tl);
	
	         AbsTime(4);
			 tl=Delv4;
			 tl=tl*3000/4096;//3000ÎªÄÚ²¿ADµÄ²Î¿¼µçÑ¹3V,4096ÎªÄÚ²¿ADÎ»Êý12Î»£¬2^12=4096
			 UARTPutByte(tl>>8);
			 UARTPutByte(tl);

			 CalP();//µ÷Õû»»ÄÜÆ÷·ùÖµ
		     break;
	      }
		case 0xf0:	//ÌáÈ¡²ÉÑù²¨ÐÎ
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
  	}
}

interrupt void SCITXINTB_ISR(void)     // SCI-B
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

