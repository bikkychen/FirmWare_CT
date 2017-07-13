#include <iom128v.h>								   	
#include <macros.h>
#include <stdio.h>

//最后修订:20160812
//20170226 增加PIC单片机在线更新功能，另外还增加了一个引导区握手命令0xf0
//20170701 适用新主控板CT_MAIN_V30

#define  BB     0x00        //固件版本号

#define INIMANIO {PORTG&=0xE7;DDRG|=0x18;}
#define M1_L     {PORTG&=0xf7;}
#define M1_H     {PORTG|=0x08;}
#define M2_L     {PORTG&=0xef;}
#define M2_H     {PORTG|=0x10;}
 
#define IF_MANINTH     if((PIND&0x01)==0x01)
#define IF_MANINTL     if((PIND&0x01)==0x00)
#define IF_MANINTF     if((EIFR&0x01)==0x01)
#define MANINT_CLR	{ EIFR|=0x01; }
#define MANINT_EN	{ EIMSK |= 0x01; }
#define MANINT_DIS	{ EIMSK &= 0xFE; }
#define INT_EN		{ SEI(); }
#define INT_DIS		{ CLI(); } 

#define DELAY10 for(Tt=0;Tt<4;Tt++);
#define DELAY20 for(Tt=0;Tt<4;Tt++);
#define DELAY40 for(Tt=0;Tt<16;Tt++); 
#define DELAY50 for(Tt=0;Tt<21;Tt++); 
#define DELAY80 for(Tt=0;Tt<32;Tt++); 
#define DELAY89 for(Tt=0;Tt<36;Tt++); 
#define DELAY100 for(Tt=0;Tt<42;Tt++); 
#define DELAY170 for(Tt=0;Tt<72;Tt++);
#define DELAY268 for(Tt=0;Tt<113;Tt++);
#define DELAY397 for(Tt=0;Tt<165;Tt++);
#define DELAY400 for(Tt=0;Tt<168;Tt++);
 
 
/*
unsigned char Rx[18];//解码接收9位共18个半位
unsigned int  T_dat,R_dat;//曼码发送数据
unsigned char T_com;//曼码发送命令
unsigned char T_dat_buf[22];//曼码发送缓冲 
unsigned char Int_count;
unsigned char IntFlag;//接收帧状态
 
unsigned int Tt;

unsigned char UpdateBegin;//接收主程序数据开始标志
unsigned int DataReCn;
unsigned char crc16hi,crc16lo,DownloadSpeed;
unsigned int Timer3Cn;
unsigned char DSP_PageCn;//DSP程序页计数

unsigned char Uart0RxCounter,Uart1RxCounter;
unsigned char Uart0RxData[128],Uart1RxData[128];
unsigned int Timer0Counter;
unsigned char TimeOutFlag;

char flash_buf[258]; //FALSH页缓冲区,M128的一个Flash页为256字节(128字) //暂只支持64K地址编程
long address = 0; 
unsigned int T2cn;

void Start(void);
void SendManchester(void); 
void Delay30Ms();
*/
unsigned char Rx[18];//解码接收9位共18个半位
unsigned int  T_dat,R_dat;//曼码发送数据
unsigned char T_com;//曼码发送命令
unsigned char T_dat_buf[22];//曼码发送缓冲 
unsigned char Int_count;
unsigned char IntFlag;//接收帧状态
 
unsigned int Tt;

unsigned char UpdateBegin;//接收主程序数据开始标志
unsigned int DataReCn;
unsigned char crc16hi,crc16lo,DownloadSpeed;
unsigned int Timer3Cn;
unsigned char DSP_PageCn;//DSP程序页计数

unsigned char Uart0RxCounter,Uart1RxCounter;
unsigned char Uart0RxData[128],Uart1RxData[128];
unsigned int Timer0Counter,Timer1Counter;
unsigned char TimeOutFlag;

char flash_buf[258]; //FALSH页缓冲区,M128的一个Flash页为256字节(128字) //暂只支持64K地址编程
long address = 0; 
unsigned int T2cn;

void Start(void);

void SendManchester(void); 
 
void Delay30Ms(void);


//擦除(code=0x03)和写入(code=0x05)一个Flash页 
void boot_page_ew(long p_address,char code) 
{ 
    asm("mov r30,r16\n" 
        "mov r31,r17\n" 
        "out 0x3b,r18\n");            //将页地址放入Z寄存器和RAMPZ的Bit0中 
    SPMCSR = code;                //寄存器SPMCSR中为操作码 
    asm("spm\n");                    //对指定Flash页进行操作 
}         
//填充Flash缓冲页中的一个字 
void boot_page_fill(unsigned int address,int data) 
{ 
    asm("mov r30,r16\n" 
        "mov r31,r17\n"             //Z寄存器中为填冲页内地址 
        "mov r0,r18\n" 
        "mov r1,r19\n");            //R0R1中为一个指令字 
    SPMCSR = 0x01; 
    asm("spm\n"); 
} 
//等待一个Flash页的写完成 
void wait_page_rw_ok(void) 
{ 
      while(SPMCSR & 0x40) 
     { 
         while(SPMCSR & 0x01); 
         SPMCSR = 0x11; 
         asm("spm\n"); 
     } 
} 
//更新一个Flash页的完整处理 
void write_one_page(void) 
{ 
    int i; 
    boot_page_ew(address,0x03);                    //擦除一个Flash页 
    
    wait_page_rw_ok();                            //等待擦除完成 
    for(i=0;i<256;i+=2)                //将数据填入Flash缓冲页中 
    { 
        boot_page_fill(i, flash_buf[i+2]+(flash_buf[i+3]<<8)); 
    } 
   boot_page_ew(address,0x05);                    //将缓冲页数据写入一个Flash页 
     
   wait_page_rw_ok();                            //等待写入完成 
}        
 

//UART1 initialize
// desired baud rate: 57600
// actual: baud rate:58824 (2.1%)
// char size: 8 bit
// parity: Disabled
void uart1_init(void)
{
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x02;
 UCSR1C = 0x06;
 UBRR1L = 0x10; //set baud rate lo 57600
 UBRR1H = 0x00; //set baud rate hi
 UCSR1B = 0x98;
}

#pragma interrupt_handler uart1_rx_isr:iv_USART1_RXC
void uart1_rx_isr(void)//串口1接收中断
{
 Uart1RxData[Uart1RxCounter]=UDR1;//接收串口数据,同时清空串口接收中断标志
 Uart1RxCounter++;
}

//UART0 initialize
// desired baud rate: 57600
// actual: baud rate:58824 (2.1%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x02;
 UCSR0C = 0x06;
 UBRR0L = 0x10; //set baud rate lo 57600
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

#pragma interrupt_handler usart0_isr:19
void usart0_isr(void) //串口接收中断
{ 
    Uart0RxData[Uart0RxCounter]=UDR0;//接收串口数据,同时清空串口接收中断标志
 	Uart0RxCounter++;
}




void quit(void) 
{
    MCUCR = 0x01; 
    MCUCR = 0x00;       //将中断向量表迁移到应用程序区头部 
    RAMPZ = 0x00;       //RAMPZ清零初始化 
    asm("jmp 0x0000\n");//跳转到Flash的0x0000处，执行用户的应用程序 
} 

void Start(void)
{
	INIMANIO
	uart0_init();
    uart1_init();
 
    ACSR|=0x10;//清模拟比较器中断标志
	ACSR&=0xf7;//关模拟比较器中断
	
	EICRA|=0x03; //INT0上升沿触发 ，每2位控制一个中断，共4个中断，0-低电平触发，1-保留，2-下降沿触发，3-上升沿触发 
	MANINT_CLR
	MANINT_EN
	INT_EN
  
	 M1_H
	 Delay30Ms();
	 Delay30Ms();
	 Delay30Ms();
	 Delay30Ms();
	 Delay30Ms();
	 M1_L
}

void crc16(unsigned char r_data[],unsigned int length)
{
        unsigned char cl,ch;
        unsigned char savehi,savelo;
        int ii,flag;
        crc16hi=0xFF;
        crc16lo=0xFF;
        cl=0x1;
        ch=0xA0;
        for (ii=0;ii<length-2;ii++)
        {
            crc16lo=(crc16lo ^ r_data[ii]);
                for (flag=0;flag<8;flag++)
                {
                       savehi=crc16hi;
                        savelo=crc16lo;
                       crc16hi=(crc16hi>>1);
                       crc16lo=(crc16lo>>1);
                        if ((savehi & 0x01)==0x01)
                                crc16lo=(crc16lo | 0x80);
                        if        ((savelo & 0x01)==0x01)
                        {
                                crc16hi = (crc16hi ^ ch);
								crc16lo = (crc16lo ^ cl);
                        }        
                }
        }
}

#pragma interrupt_handler timer3_ovf_isr:iv_TIM3_OVF
void timer3_ovf_isr(void)
{
  UpdateBegin=0;//回到待命状态
  TCCR3B = 0x00; //stop  
  ETIMSK &= 0xfb;//关定时器3中断
  ETIFR|=0x04; //清定时器3中断标志 
}

	
void SendUart0(unsigned char c,unsigned char s)   //串口发送数据
{//要求流量板在s个8ms内通过串口返回数据
unsigned char t; 	

  Uart0RxData[0]=0xff;
  Uart0RxData[1]=0xff;
  Uart0RxCounter=0; //清串口接受计数  
  
   t=UDR0;//开串口接收中断前读空缓存
   
  while(!(UCSR0A&(1<<UDRE0)));   // 等待发送缓冲器为空
  UDR0=c;   // 将数据放入缓冲器，发送数据
                   
   t=UDR0;//开串口接收中断前读空缓存
   UCSR0B|=0x80;//开串口接收中断
   
   //超时设定
	TCCR1B = 0x00; //stop
	if(s==0)
	 {
       TCNT1 = 22576;//5500ms 
	   TCCR1B = 0x05; //1024分频
	 }
	 else
	 {
	   TCNT1 = 1536;//8ms 
	   TCCR1B = 0x01; //1分频
	 }
 	
	TIFR|=0x04; //清定时器1中断标志
	while((TIFR&0x04)==0x00); 
	TCCR1B = 0x00; //stop
	TIFR|=0x04; //清定时器1中断标志 
	UCSR0B&=0x7f;//关串口接收中断
}

void SendUart1(unsigned char c,unsigned char s)   //串口发送数据
{//要求流量板在s个8ms内通过串口返回数据
unsigned char t; 	

  Uart1RxData[0]=0xff;
  Uart1RxData[1]=0xff;
  Uart1RxCounter=0; //清串口接受计数  
  
   t=UDR1;//开串口接收中断前读空缓存
   
  while(!(UCSR1A&(1<<UDRE1)));   // 等待发送缓冲器为空
  UDR1=c;   // 将数据放入缓冲器，发送数据
                   
   t=UDR1;//开串口接收中断前读空缓存
   UCSR1B|=0x80;//开串口接收中断
   
   //超时设定
	TCCR1B = 0x00; //stop
	if(s==0)
	 {
       TCNT1 = 22576;//5500ms 
	   TCCR1B = 0x05; //1024分频
	 }
	 else
	 {
	   TCNT1 = 1536;//8ms 
	   TCCR1B = 0x01; //1分频
	 }
 	
	TIFR|=0x04; //清定时器1中断标志
	while((TIFR&0x04)==0x00); 
	TCCR1B = 0x00; //stop
	TIFR|=0x04; //清定时器1中断标志 
	UCSR1B&=0x7f;//关串口接收中断
}

void SetTimer3(void)//页写开始后，如果有500ms时间内一个字节也没收到，则退出页写状态，回到待命状态
{  
    TCCR3B = 0x00; //stop    
    TCNT3H = 0xF0; //8M,500ms
    TCNT3L = 0xBE; //8M,500ms
	ETIFR|=0x04; //清定时器3中断标志
 	TCCR3B = 0x05; //1024分频
	ETIFR|=0x04; //清定时器3中断标志
	ETIMSK |= 0x04; //开定时器3中断
}

void main(void)
{
 	 unsigned int i,j,k;
	 unsigned char t; 	
     Delay30Ms();
	 
	 IntFlag=0;//无接收帧中断
     UpdateBegin=0;//无标定系数下发
	 T_dat=0;
	 R_dat=0;
	 DSP_PageCn=0;
	 
 	 Start();
	 
	 Delay30Ms();

	 
	TCCR3B = 0x00; //stop    
	TCNT3H = 0xd1; ////8M,1.5s
    TCNT3L = 0x20; ////8M,1.5s
	ETIFR|=0x04; //清定时器3中断标志
	ETIMSK &= 0xfb;//关定时器3中断
 	TCCR3B = 0x05; //1024分频
	ETIFR|=0x04; //清定时器3中断标志
	    
	 UpdateBegin=0;
	while( ((ETIFR&0x04)==0x00) && (UpdateBegin==0) )
	{
	 if((IntFlag==1))//3秒内收到开始更新命令
	 {  
	  if(R_dat==0xfa) //主控板握手
	   {
	    UpdateBegin=1;
	    T_com=0x0f;
	    T_dat=0xe7fa;	
		Delay30Ms();
		SendManchester();  
		IntFlag=0;//无命令帧
	    MANINT_CLR
	    MANINT_EN
	   }  	
	   else if(R_dat==0xf5)//电机板握手
	   {
		UpdateBegin=1;
		SendUart1(0xf5,1);  
		T_com=0x0f;
		T_dat=0xe7;
		T_dat<<=8; 	
		T_dat|=Uart0RxData[0];//成功的话应返回0xe7f5
		Delay30Ms();
		SendManchester(); 
		IntFlag=0;//无命令帧
		MANINT_CLR
	    MANINT_EN
		} 	
	   else if(R_dat==0xfb)//采集板DSP握手
	   {
	    UpdateBegin=1;
	  	SendUart0(0xb5,1);  
		T_com=0x0f;
		T_dat=Uart0RxData[0];//成功的话应返回0xe7fb
		T_dat<<=8; 	
		T_dat|=Uart0RxData[1];
	  	Delay30Ms();
      	SendManchester(); 
		IntFlag=0;//无命令帧
	    MANINT_CLR
	    MANINT_EN 
	   }
	   else if(R_dat==0xf9)//采集板PIC握手
	   {
	    UpdateBegin=1;
	  	SendUart0(0xf9,1);  
		T_com=0x0f;
		T_dat=0xe7;
		T_dat<<=8; 	
		T_dat|=Uart0RxData[0];//成功的话应返回0xe7f9
	  	Delay30Ms();
      	SendManchester(); 
		IntFlag=0;//无命令帧
	    MANINT_CLR
	    MANINT_EN
	   } 
	 }		
	 if(IntFlag==2)//校验位错误处理程序
   	 { 
	   IntFlag=0;//无命令帧
	   MANINT_CLR
	   MANINT_EN
   	 }									
	} 
	
	TCCR3B = 0x00; //stop    
	ETIFR|=0x04; //清定时器3中断标志
	ETIMSK &= 0xfb;//关定时器3中断
	
	if(UpdateBegin==0)//3秒内没收到开始更新命令，则直接跳到主程序
	{
	 quit();
	}
	
	UpdateBegin=0;
	while(1)
	{
 	 if(IntFlag==2)//校验位错误处理程序
   	 { 
	   IntFlag=0;//无命令帧
	   MANINT_CLR
	   MANINT_EN 
   	 }
   	else if(IntFlag==1)//接收到了正常命令帧
   	{
	 	 if(UpdateBegin>0)//接收主程序二进制数据
     	 { 	  
		      SetTimer3();	 //每收到一字节就重新开始定时500ms，超时后回到待命状态
			  					   
		      flash_buf[DataReCn]=R_dat;
			  DataReCn++;
			  
			  if(UpdateBegin==2)//如果是采集板页写准备了，直接把这个字节丢给采集板
			  {
			    while(!(UCSR0A&(1<<UDRE0)));   // 等待发送缓冲器为空
			    UDR0=R_dat; 
			  }
			  else if(UpdateBegin==3)//如果是电机板页写准备了，直接把这个字节丢给电机板
			  {
			    while(!(UCSR1A&(1<<UDRE1)));   // 等待发送缓冲器为空
			    UDR1=R_dat; 
			  }	 
			   
		 	 if(DataReCn==258)//2字节页索引，256字节页数据
		   	 {
			   TCCR3B = 0x00; //stop  
  			   ETIMSK &= 0xfb;//关定时器3中断
  			   ETIFR|=0x04; //清定时器3中断标志 
  			   		   
  			   if(UpdateBegin==1)//如果是主控板页写准备了，则此时写入一页
				{
                  address=flash_buf[0]*256+flash_buf[1];  
			      address*=256;	 
				  
			      if(address<0)
			          address=0;
				 
			     if(address>0xff00)//对应页索引为255，这是最后一页能写的页，目前限制固件大小为64K字节
			         address=0xff00;
				
				  
		          write_one_page();	//约20ms  
				  
			    }   
				  
			   UpdateBegin=0;//一页写完了，下一页又从页写准备开始
			   
	 		   T_com=0x0f;	      
			   crc16(flash_buf,258);//12ms
		       T_dat=crc16hi;
			   T_dat<<=8;
			   T_dat|=crc16lo;
		       Delay30Ms();
               SendManchester();	//将这一页的校验码上传给上位机，上位机判断后决定是继续烧写一页还是重烧这一页	   		        
			 }	 	 
	     }
  		 else//若不是下发标定系数，则进入以下各个命令的子语句
		 {
    		switch(R_dat)
     		{			
			   case 0x20://读版本号  
		 		  	   T_com=0x02;
					   T_dat=0x00;
					   Delay30Ms();
					   SendManchester();	
	  		   break;  
					   
				case 0xf0://引导区握手
					 T_com=0x0f;
	  			     T_dat=0xe7ff;//上位机实收到的数,不用高低换位				       
	  			     Delay30Ms();
      			     SendManchester();  
				break;
				
				case 0xf5://电机板握手
			         T_com=0x0f;
	  			 	 SendUart1(0xf5,1);  
					 T_dat=0xe7;
		 			 T_dat<<=8; 	
					 T_dat|=Uart1RxData[0];//成功的话应返回0xf5
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
			   break;	
			
			   case 0xf4://电机板页写准备			     
					 T_com=0x0f;
					 SendUart1(0xf4,1); 
	  			     T_dat=0xe7;
		 			 T_dat<<=8;
		 			 T_dat|=Uart1RxData[0];//正常情况下应是0xf4  
	  			     Delay30Ms();
      			     SendManchester(); 
					 UpdateBegin=3;//页写准备，共64K字节程序空间
					 DataReCn=0;
				     SetTimer3();
				break;
				
				case 0xf9://PIC握手，进入更新状态
					 T_com=0x0f;
	  			 	 SendUart0(0xf9,1);  
					 T_dat=0xe7;
		 			 T_dat<<=8; 	
					 T_dat|=Uart0RxData[0];//成功的话应返回0xf9
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
				break;
				
				case 0xf8://PIC页写准备			     
					 T_com=0x0f;
					 SendUart0(0xf8,1);//块写FLASHE准备 		
	  			     T_dat=0xe7;
		 			 T_dat<<=8;
		 			 T_dat|=Uart0RxData[0];//正常情况下应是0xf8  
	  			     Delay30Ms();
      			     SendManchester(); 
					 UpdateBegin=2;//页写准备，共64K字节程序空间
					 DataReCn=0;
				     SetTimer3();
				break;
				/*
				case 0xf7://PIC版本号
					 T_com=0x0f;
	  			 	 SendUart0(0xf7,1);  
					 T_dat=0xe7;
		 			 T_dat<<=8; 	
					 T_dat|=Uart0RxData[0];//成功的话应返回0x00
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
				break;
					
				case 0xf6://DSP版本号
					 T_com=0x0f;
	  			 	 SendUart0(0xf6,1);  
					  T_dat=Uart0RxData[0];//成功的话应返回0xe7
		 			 T_dat<<=8; 	
					 T_dat|=Uart0RxData[1];//成功的话应返回0x00
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
				break;
				   */
			   case 0xfa://主控板握手，进入更新状态
	  			    // SendUart0(0xb5,1);
					 T_com=0x0f;
	  			     T_dat=0xe7fa;//上位机实收到的数,不用高低换位				       
	  			     Delay30Ms();
      			     SendManchester();  
				break;
				
			   case 0xfb://DSP握手，进入更新状态
					 T_com=0x0f;
	  			 	 SendUart0(0xb5,1);  
					 T_dat=Uart0RxData[0];//成功的话应返回0xe7fb
		 			 T_dat<<=8; 	
					 T_dat|=Uart0RxData[1];
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
				break;	
				 		   			
				case 0xfc://主控板页写准备			     
					 T_com=0x0f;
	  			     T_dat=0xe7fc;//上位机实收到的数,不用高低换位
	  			     Delay30Ms();
      			     SendManchester(); 
					 UpdateBegin=1;//页写准备，主控板从页索引0开始烧写，最大页索引255，共64K字节程序空间
					 DataReCn=0;
				     SetTimer3();//开始计时，每500ms内至少要有一个字节收到
				break;
											
				case 0xfd://DSP页写准备			     
					 T_com=0x0f;
					 SendUart0(0xb7,1);//块写FLASHE准备 		
	  			     T_dat=Uart0RxData[0];//高字节，正常情况下应是0xe7
		 			 T_dat<<=8;
		 			 T_dat|=Uart0RxData[1];//低字节,正常情况下应是0xfd   
	  			     Delay30Ms();
      			     SendManchester(); 
					 UpdateBegin=2;//页写准备，采集板从页索引479开始往前烧写，最小页索引224，共64K字节程序空间
					 DataReCn=0;
				     SetTimer3();
				break;
				
				case 0xfe://DSP擦除，上位机设总超时6秒			     
					  SendUart0(0xb6,0);//擦除FLASHE,2秒超时 ,擦除FLASHD,2秒超时 ，共5.5秒超时				 
					  T_dat=Uart0RxData[0];//擦除成功的话应返回0xe7
		 			  T_dat<<=8; 				  	 
		 			  T_dat|=Uart0RxData[1];//擦除成功的话应返回0xfe				   
					  T_com=0x0f;
      			 	  SendManchester();	//上传擦除结果				 					
				break;	
				
				case 0xff://退出更新状态，运行正常程序
				 SendUart0(0xb8,1);
				 T_dat=Uart0RxData[0];//成功的话应返回0xe7
		 		 T_dat<<=8; 
				 T_dat|=0xff;
				 T_com=0x0f;
				 Delay30Ms();
				 SendManchester();	//上传擦除结果		
				 quit();
				break;

				default:
				break;
	  	    }//switch(R_dat&0xf0)
	     }//UpdateBegin
		 IntFlag=0;
    	 MANINT_CLR
	     MANINT_EN
	 }//IntFlag
    }//while(1)
 
}


#pragma interrupt_handler int0_isr:iv_INT0
void int0_isr(void)//外部中断0
{ 
   TCCR2 = 0x00; //stop
	INT_DIS
	MANINT_DIS
  
  DELAY10
  MANINT_CLR
  IF_MANINTL//高脉冲不足10us则直接跳出
  {
   goto End;
  }
   
   TCNT2 = 0x00;  
   TCCR2 = 0x03; //64分频,8M时钟，每个CLOCK耗时8us，最大定时2048us，每周期10个计数
   
   while(1)
   {
     T2cn=TCNT2;
	  if(T2cn>80)//设计值最多7个周期，在此多给一个周期，则超过8个周期还没来上升沿中断到来就直接跳出
	  {
	    goto End;
	  }
     IF_MANINTL
	 {
	   DELAY20
	   IF_MANINTL
	    {
	      break;
	    }
	 }
   }
   
  		   
    //开始判断同步头	
	while(1)//还没有上升沿到来,死等
	{
	  T2cn=TCNT2;
	  if(T2cn>80)//设计值最多7个周期，在此多给一个周期，则超过8个周期还没来上升沿中断到来就直接跳出
	  {
	    goto End;
	  }
	  IF_MANINTF
	  {
	   DELAY10
	   MANINT_CLR
	   IF_MANINTH//高脉冲维持了10us，则认为是下一个有效上升沿到来
	     {
   	     break;
		 }
	  }  
	}
	T2cn=TCNT2;
	TCNT2=0;
	if((T2cn<60)||(T2cn>80))//同步头本应是7个周期，但收到的结果为6~8个周期内都认可
	{
	 goto End;//没有收到正确的同步头，直接跳出
	}
 
Start:	//同步头解码正确，下面开始接收9位数据位	//按位从高到低接收数据位，8位数据加1位校验共9位 		      	
  for(Int_count=9;Int_count>0;Int_count--)
  {	
   while(1)
   {
     T2cn=TCNT2;
	  if(T2cn>80)//设计值最多7个周期，在此多给一个周期，则超过8个周期还没来上升沿中断到来就直接跳出
	  {
	    goto End;
	  }
     IF_MANINTL
	 {
	   DELAY20
	   IF_MANINTL
	    {
	      break;
	    }
	 }
   } 
   while(1)//还没有上升沿到来,死等
	{
	  T2cn=TCNT2;
	  if(T2cn>80)//设计值最多7个周期，在此多给一个周期，则超过8个周期还没来上升沿中断到来就直接跳出
	  {
	    goto End;
	  }
	  IF_MANINTF
	  {
	   DELAY10
	   MANINT_CLR
	   IF_MANINTH//高脉冲维持了10us，则认为是下一个有效上升沿到来
	     {
   	     break;
		 }
	  }  
	}
	T2cn=TCNT2;
	TCNT2=0;
	if(T2cn<20)//数据位最小是3位，可放宽到2位
	{
	 IntFlag=3;//接收数据位不完整
	 goto End;//数据位来得太早，退出
	}
	else if(T2cn<40)//数据位1本应是3个周期，在此我们认为2~4个周期都是可以的
	{
	 Rx[Int_count]=1;
	}
	else if(T2cn<60)//数据位0本应是5个周期，在此我们认为4~6个周期都是可以的
	{
	 Rx[Int_count]=0;
	}
	else if(T2cn<=80)//同步位本应是7周期，在此我们认为6~8周期都是可以的
	{
	 goto Start;//在接收数据位时收到了同步位，跳到数据位解码起始位处
	}
	else 
	{
	 IntFlag=3;//接收数据位不完整
	 goto End;//数据位来得太迟，退出
	}
  }   
	
    R_dat=0;
    if(Rx[2])
          R_dat|=0x01;
    if(Rx[3])
          R_dat|=0x02;
    if(Rx[4])
          R_dat|=0x04;
    if(Rx[5])
          R_dat|=0x08;
    if(Rx[6])
          R_dat|=0x10;
    if(Rx[7])
          R_dat|=0x20;
    if(Rx[8])
          R_dat|=0x40;
    if(Rx[9])
          R_dat|=0x80;
	
	Rx[0]=1;  
	for(Int_count=2;Int_count<10;Int_count++)  
       Rx[0]^=Rx[Int_count];  
	   
	if(Rx[0]==Rx[1])
	 {
		 IntFlag=1;//接收帧正常	
	 }
	else           
	 {
		 IntFlag=2;//接收帧校验错
	 }
 
   goto End1;//只要接收到正常的同步头，不管校验位是对是错，都暂时不开放INT0中断
     
End:
		MANINT_CLR
		MANINT_EN
		
End1:
   TCCR2 = 0x00; //stop
   INT_EN//总中断使能;  
}

 
void SendManchester(void) //曼码发送程序,总耗时最大约120+560+8400+720=9800us，最小约120+560+5040+720=6440us
{ //每位80us
 //同步：1高6低
 //1:    1高2低
 //0：   1高4低  
  unsigned char L;
  SREG&=0x7f;//关总中断 
  
    M1_H //先给总线放电，耗时约120us
	
  //模拟同步头数据
	//4位命令
	T_dat_buf[21]=(T_com&0x08)>>3;
	T_dat_buf[20]=(T_com&0x04)>>2;
	T_dat_buf[19]=(T_com&0x02)>>1;
	T_dat_buf[18]=T_com&0x01;
	//16位数据
	T_dat_buf[17]=(T_dat&0x8000)>>15;
	T_dat_buf[16]=(T_dat&0x4000)>>14;
	T_dat_buf[15]=(T_dat&0x2000)>>13;
	T_dat_buf[14]=(T_dat&0x1000)>>12;
	T_dat_buf[13]=(T_dat&0x0800)>>11;
    T_dat_buf[12]=(T_dat&0x0400)>>10;
    T_dat_buf[11]=(T_dat&0x0200)>>9;
	T_dat_buf[10]=(T_dat&0x0100)>>8;
	T_dat_buf[9]=(T_dat&0x0080)>>7;
	T_dat_buf[8]=(T_dat&0x0040)>>6;
	T_dat_buf[7]=(T_dat&0x0020)>>5;
	T_dat_buf[6]=(T_dat&0x0010)>>4;
	T_dat_buf[5]=(T_dat&0x0008)>>3;
	T_dat_buf[4]=(T_dat&0x0004)>>2;
	T_dat_buf[3]=(T_dat&0x0002)>>1;
	T_dat_buf[2]=T_dat&0x0001;
	//计算校验位，奇校验
	T_dat_buf[1]=1;
  	for(L=2;L<22;L++)
		{T_dat_buf[1]^=T_dat_buf[L];}
    DELAY80
    M1_L
	DELAY40
   
   
	//同步头,560us
	//充电开始，高80us	 
	M2_H 	
	DELAY80
	M2_L   //充电结束	
	//放电开始，低80*6=480us
	DELAY40
	M1_H    
    DELAY80
	DELAY80
	DELAY80
	DELAY80
	DELAY80
	M1_L    
	DELAY40  //放电结束
	
     //连续发送 ,全0时每位400us，共8400us，全1时每位240us，共5040us
 	for(L=21;L>0;L--)
	{   
	  if(T_dat_buf[L])//1，一高二低
        {         	 		
	        M2_H 
	        DELAY80
	        M2_L
			DELAY40
	        M1_H       
         	DELAY80
			M1_L
			DELAY40
		} 
       else 		//0，一高四低 				                
        {
		 	M2_H 
	        DELAY80
	        M2_L
			DELAY40
	        M1_H       
         	DELAY80
			DELAY80
			DELAY80
			M1_L
			DELAY40
		}
    }	       	   
		   //结束位，一高八低，720us
 	 	    M2_H 
		    DELAY80
	        M2_L
			DELAY40
			
			for(L=0;L>50;L++)
			{
			  DELAY80
			}
	
 	MANINT_CLR
 	INT_EN//总中断使能;  
}


 

void Delay30Ms(void)//8M主时钟频率，则最大定时7812.5ms
{
	TCCR1B = 0x00; //stop
    TCNT1 = 65301;//30ms  
 	TCCR1B = 0x05; //1024分频
	TIFR|=0x04; //清定时器1中断标志
	while((TIFR&0x04)==0x00); 
	TCCR1B = 0x00; //stop
	TIFR|=0x04; //清定时器1中断标志
}

 

/*

void EEPROM_write(unsigned int Address,unsigned char Data)  	//??EEPROM?
{//?????1MHz,???EEPROM??????8.5ms
    while(EECR&0x02);                   // ??????????
    EEAR=Address;
    EEDR=Data;                        	// ??????????
    EECR|=0x04;                         //??EEMWE
    EECR|=0x02;                         //??EEWE ??????
}

unsigned char EEPROM_read(unsigned int Address)  				//??EEPROM?
{
    while(EECR&0x02);      				//?????????
    EEAR = Address;                     //???????						
	EECR|=0x01;             			//??EERE ??????
    return EEDR;                   		//??????????
}   */