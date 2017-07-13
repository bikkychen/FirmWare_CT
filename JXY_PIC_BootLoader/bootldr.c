#include <p18f4550.h>

#define RX_TRIS TRISCbits.TRISC7//串口端口
#define TX_TRIS TRISCbits.TRISC6

#define BOOT_TIMEOUT  1       //启动超时时间/秒
#define PROG_START    0x800   //用户程序开始位置
#define PAGESIZE      256
#define MEM_TOP       0x8000       //FLASH最大存储空间

void Write_One_Byte(unsigned  long address, unsigned char data);
void Flash_Write64Bytes(unsigned  long address, unsigned char *Pdata);
void Flash_Erease64Bytes(unsigned  long address);
void SetTimer(void);

#pragma udata gg
 unsigned char buff[256];  //编程缓冲区 //
#pragma udata

 unsigned char dataflag,rectdat;
 unsigned int  bcount;
 unsigned char delay_time;
 unsigned  long flashaddr; 

#pragma code High_Interrupt_Vector=0X08
void high_ISR(void)
{
	_asm
	goto   PROG_START+0x08
	_endasm
}
#pragma code

#pragma code Low_Interrupt_Vector=0X18
void low_ISR(void)
{
	_asm
	goto    PROG_START+0x18
	_endasm
}
#pragma code
 
/*
#pragma code bbb=0x800
void aaa(void)
{
 int a;
a=1;
a++;
a--;
}
#pragma code
*/

// 写周期
void Write_Cycle(void)
{unsigned char flag=0;

	EECON1bits.EEPGD = 1;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	
	if(INTCONbits.GIE)//禁止全局中断
	{
		INTCONbits.GIE = 0;
		flag=1;
	}
	
	EECON2 = 0X55;
	EECON2 = 0XAA;
	EECON1bits.WR = 1;
	Nop();
	Nop();
	while (EECON1bits.WR);	
	EECON1bits.WREN = 0;
	
    if(flag){INTCONbits.GIE = 1;}//允许全局中断
}

 

//写函数，每次写入64字节
void Flash_Write64Bytes(unsigned  long address, unsigned char *Pdata)
{
	unsigned char count,i;
	unsigned char length;
	
	Flash_Erease64Bytes(address);
	
	for (count=0; count<8; count++)
	{		
		TBLPTRU = 0;//( ( (address + count * 8)>>8)>>8);
		
		for (length=0; length<8; length++)
		{
			TBLPTRL = ( (address + count * 8 + length) & 0xFF);
			TBLPTRH = ( ( (address + count * 8 + length)>>8) & 0XFF);
			TABLAT = *(Pdata + length + count * 8);
                  _asm
                  tblwt
			_endasm
/*
			if (length != 0)
			{
				_asm
				TBLWTPREINC
				_endasm
			}
			else
			{
				//asm("\tTBLWT*");
			}*/			
		}
		
		EECON1bits.FREE = 0;
		Write_Cycle();
	}
}

void Flash_Erease64Bytes(unsigned  long address)//大约耗时40ms
{ 
  unsigned char flag=0;

  if(address<PROG_START)
  {return ;}

 if(address>=MEM_TOP)
  {return ;}

  // 擦除程序空间 //
    TBLPTRU=0;
    TBLPTRL=(unsigned char)address;//擦除起始地址
    TBLPTRH=(unsigned char)(address>>8);

     EECON1bits.EEPGD=1;//指向程序存储器访问FLASH
     EECON1bits.CFGS=0;//访问FLASH和EEPROM    
     EECON1bits.WREN = 1;//允许写
     EECON1bits.FREE = 1; //允许擦除
     INTCONbits.GIE = 0;
     if(INTCONbits.GIE)//禁止全局中断
	{
		INTCONbits.GIE = 0;
		flag=1;
	}
     EECON2 = 0x55;
     EECON2 = 0xAA;
     EECON1bits.WR = 1;

    while(EECON1bits.WR);//查询写操作是否完成，完成后硬件自动清零    
    EECON1bits.WREN = 0;

    if(flag){INTCONbits.GIE = 1;}//允许全局中断
}
 


void SetTimer(void)
{
	T1CON=0XCE;          //使能TIMER1振荡器,0X4E预分频1：1，0X7E预分频8：1，异步模式;	
	
	PIE1bits.TMR1IE=0;   //不允许Timer1中断	
      PIR1bits.TMR1IF=0;	//清零TIMER1中断标志
	
//	TMR1H=0xF0;//定时器赋初值，1/8 s
	TMR1H=0x80;//定时1秒
	TMR1L=0x00;		
		
	T1CONbits.TMR1ON=1;//开启TMR1
}

void main(void)
{  int i;
    // 初始化串口
     RX_TRIS=1;				//串行通信端口使能的必须设置
	TX_TRIS=1;				//串行通信端口使能的必须设置
	SPBRGH=0;				//波特率57600(4MHz)
	//SPBRG=16;				// 16=115200(8MHz)
	SPBRG=34;				// 57600 (8MHZ)
	BAUDCON=0X08;			       //16位波特率发生器，禁止波特率检测，未监视RX引脚
	TXSTA=0X24;				//8位发送，使能发送，高速异步模式
	RCSTA=0X90;				//8位接收，使能接收     
    TRISB&=0xDF;//禁止上拉，要下拉，RB5=0，硬件设计上有保证的此行可省略

  // 某些型号的问题，需要清除这些寄存器
  INTCON3=0;  // Also serves to disable interrupts
  PIE2=0;
  INTCON=0;
/*
 for(i=0;i<256;i++)
   buff[i]=i;
   Flash_Write64Bytes(0x840,&buff[0]);
  while(1);
 */
  //使用超时判断方式
  //在指定时间内收到数据，进入编程状态，否则运行以前的程序

  SetTimer(); // 使用定时器定时1秒
  delay_time=0;
  while(1)
  {
    if (PIR1bits.RCIF)   //接受到串口数据
    {
      if(RCREG==0xf9)
      {
         TXREG = 0xf9;//向主控板回应一字节
         break;//联机成功，则不再等待
       }
    }

   if(PIR1bits.TMR1IF)//等待1秒定时标志
    {
      PIR1bits.TMR1IF=0;	//清零TIMER1中断标志
      SetTimer(); // 使用定时器定时1秒
      delay_time++;
    }

    if(delay_time==BOOT_TIMEOUT)
     {break;}  
  }
  PIR1bits.TMR1IF=0;	//清零TIMER1中断标志 
  T1CONbits.TMR1ON=0;//停止TMR1


  if (delay_time==BOOT_TIMEOUT)//联机失败
  {
    (*((void(*)(void))PROG_START))(); //直接进入用户程序
  }

  dataflag=0;//初始为命令状态

// 通过串口接收命令或数据 //
  for(;;)   // 循环
  {
	   while(!PIR1bits.RCIF)// 等待串口数据或命令
	   {
 		if(PIR1bits.TMR1IF)  //等待1秒定时标志
	      {
	       PIR1bits.TMR1IF=0;	//清零TIMER1中断标志 
             T1CONbits.TMR1ON=0;//停止TMR1
	       dataflag=0;//切换为命令状态
	      }
	   };       
	   
         SetTimer();

	   rectdat = RCREG;;          // 取数据或命令

	   if(dataflag==1)//数据状态
	   {
             bcount++;
		if(bcount==1)
             {flashaddr=rectdat;
		  flashaddr<<=8;}
            else if(bcount==2)
             {flashaddr|=rectdat;//页索引
              flashaddr*=(PAGESIZE);// 每页多少字节
		  flashaddr+=PROG_START;
		  }
            else
             {
              buff[bcount-3]=rectdat;
             }
              
            if(bcount==258)
             {
              for(i=0;i<4;i++)
              {
		  Flash_Write64Bytes(flashaddr+64*i,&buff[i*64]);
              }
              TXREG = 0xf8;   
              bcount=0;
              dataflag=0;//切换为命令状态
             }
            
	   }
	   else//命令状态
	   {
		switch(rectdat)
		    {
		     	case 0xf9://PIC握手
			TXREG = 0xf9;
			break;
		
			case 0xf8://PIC页写准备
			TXREG = 0xf8;
			dataflag=1;//切换为数据状态
			bcount = 0;//数据接收计数清零
			break;

			case 0xf7://PIC版本
			TXREG = 0x00;
			break;
	
		      case 0xb8: // 退出更新状态
		        TXREG = 0xe7;//给主控板回应一字节0xe7
		        (*((void(*)(void))PROG_START))(); //运行新程序
		      break;
		    }
	   }   
  }
}

