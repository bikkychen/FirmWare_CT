#include "DSP28_Device.h"
#include "Flash281x_API_Library.h"
#include <math.h>

FLASH_ST ProgStatus;
FLASH_ST EraseStatus;
Uint16 Status;
float32 ApiVersion;
char SciCom;
	
void DelaymS(unsigned char);
void InitFlash(void);

extern unsigned char secureRamFuncs_loadstart;
extern unsigned char secureRamFuncs_loadend;
extern unsigned char secureRamFuncs_runstart;
 
extern Uint16 FlashWriteFlag;
extern Uint32 HandShake;
Uint16 FlashWriteFlag_Old;

void main(void)
{  
	unsigned char i;

 	DINT;//DINT   asm(" setc INTM")

	HandShake=0;

	DelaymS(1); 

	//DSP时钟倍频
	InitSysCtrl(); 
    DelaymS(1); 

    InitGpio();
	InitEv();
    
	
	InitPieCtrl(); 
	InitPieVectTable();
	IER = 0x0000;
	IFR = 0x0000;
    XintfRegs.XINTCNF2.bit.CLKOFF = 1;   // DISABLE XCLKOUT
    
	InitEv();

 
	memcpy(&secureRamFuncs_runstart,
	&secureRamFuncs_loadstart,
	&secureRamFuncs_loadend - &secureRamFuncs_loadstart);
	InitFlash();
 
	memcpy(&Flash28_API_RunStart,
	&Flash28_API_LoadStart,
	&Flash28_API_LoadEnd - &Flash28_API_LoadStart); 


    ApiVersion=Flash_APIVersion();
    Flash_CPUScaleFactor = SCALE_FACTOR;
    Flash_CallbackPtr = 0;


	InitSci();  	   
    DINT;
	/*设置中断服务程序入口地址*/
	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.RXBINT = &SCIRXINTB_ISR;//接收中断
	PieCtrl.PIEIER9.bit.INTx3 = 1;
	EDIS;   // This is needed to disable write to EALLOW protected registers
	/*开中断*/
	IER |= M_INT9;
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

	 for(i=0;i<100;i++)
	 {
	  DelaymS(10); //等待数秒	
	  if( HandShake!=0)
	   {i=100;} 
	 }

	 if(HandShake==0)
	 {
	  ((void(*)(void))0x3E8000)();//跳到主程序
	 }

	  while(1)
	   {
	     if(FlashWriteFlag>0)
		 {
		     FlashWriteFlag_Old=FlashWriteFlag;
		     DelaymS(150);//仿真运行时350ms，实际运行可能时间更长
			 if(FlashWriteFlag_Old==FlashWriteFlag)
		  	 {
			   FlashWriteFlag=0;//一定时间内没有接收到新的页数据，则回到待命状态
			 }
		 }
	   }
} 	

#pragma CODE_SECTION(InitFlash, "secureRamFuncs")
void InitFlash(void)
{
asm(" EALLOW"); // Enable EALLOW protected register access
FlashRegs.FPWR.bit.PWR = 3; // Flash set to active mode
FlashRegs.FSTATUS.bit.V3STAT = 1; // Clear the 3VSTAT bit
FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF; // Sleep to standby cycles
FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF; // Standby to active cycles
FlashRegs.FBANKWAIT.bit.RANDWAIT = 3; // F280x Random access wait states
FlashRegs.FBANKWAIT.bit.PAGEWAIT = 3; // F280x Paged access wait states
FlashRegs.FOTPWAIT.bit.OPTWAIT = 5; // F280x OTP wait states
FlashRegs.FOPT.bit.ENPIPE = 1; // Enable the flash PIpeline
asm(" EDIS"); // Disable EALLOW protected register access
/*** Force a complete PIpeline flush to ensure that the write to the last register
configured occurs before returning. Safest thing is to wait 8 full cycles. ***/
asm(" RPT #60 || NOP");
} 


void DelaymS(unsigned char t)//150M时调准了，目前是64M，定时约为3.4t ms
{
	unsigned long l,k;

    for(k=0;k<t;k++)
		for(l=0;l<10800;l++);
}


void UARTPutByte( char b)
{char k;
 while(ScibTx_Ready() != 1);
 ScibRegs.SCITXBUF =b;
 for(k=0;k<500;k++);
}

