 /**************************************************
CTZK
//20161110 增加了全部采样的CRC16校验
//20161113 改进了流量数据中时间和相位由无符号短整型转为浮点数时的转换方法，以防上位机软件出现相位超大值
//20161224 将电机断流判断值由26mA改为13mA，因为目前调节电机工作时空载电流值仅为23~28mA，升级版本为V4.0
//20161229 修改收张电机的本电机同向和反向判断，修改调节电机的电机方向控制（与原来相反）
//20170308 流量采样增加校验帧，升级版本为V4.2
//20170607 电机断路电流由原来的13mA改为26mA 升级版本为V4.3
//20170608 改动电机过流档位并把最大档位提升到250mA，升级版本为V4.4
//20170609 加入调节电机微调功能，升级版本为V4.5
//20170613 增加电机断路电流上位机可设定(共6档)，增加调节电机微调时间由电机命令中自带参数，改上传电流由数字量为工程量（单位mA），堵转电流上传改为档位，版本升为V4.6
//20170614 上传的电缆电压也改为工程量，全部采样增加电缆电压
//20170622  为适应新高压电机驱动的电路板而改进一些功能，版本回退到V3.0
//20170701 为适应新电路板CT_MAIN_V30
//20170703 增加独立压力短节单芯内部总线通讯功能
//20170703 版本从6.0起步，为了与前一阶段的硬件相区别
//20170705 压力短节功能调通 V6.1
//20170711  版本回退到V1.0，方便以后的功能扩展
**************************************************/
#include <iom128v.h>								   	
#include <macros.h>
#include <stdio.h>

 
#define Debug 0
#define  BB     0x10       //固件版本号


#define M1_L     {PORTG&=0xf7;}
#define M1_H     {PORTG|=0x08;}
#define M2_L     {PORTG&=0xef;}
#define M2_H     {PORTG|=0x10;}

 


#define MANINT_CLR	{ EIFR|=0x01; }
#define MANINT_EN	{ EIMSK |= 0x01; }
#define MANINT_DIS	{ EIMSK &= 0xFE; }

#define INT_EN			{ SEI(); }
#define INT_DIS		{ CLI(); }

 
    
#define TPSBAUD  {TCNT3H = 0xFE;TCNT3L = 0x60;}
 

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

/* 接收电平值 */
#define GET_RX() (PIND & (1<<PIND1))  
/* 发送高电平 */   
#define SET_TX() (PORTD |= 0x40)
/* 发送低电平 */ 
#define CLR_TX() (PORTD &= 0xbf)
        

 

/* 定时器中断发生时的读写周期 */
enum timer_turn {   
 RX_TURN = 0,                /* 读周期 */    
 TX_TURN,                    /* 写周期 */
 };
 
/* 数据帧结构 */
__flash enum frame_bit {    BIT_0 = 0,    BIT_1,    BIT_2,    BIT_3,    BIT_4,    BIT_5,    BIT_6,    BIT_7,    BIT_STOP,    BIT_IDLE,    BIT_START,};

union FIB
{
 float f;
 unsigned int i[2];
 unsigned char b[4];
}myFIB;

 
 
 

unsigned char T_Flag;

unsigned int Timer0Counter;//定时器0超过4ms时的计数

unsigned char Uart0RxData[74];//串口0接收数据缓存
unsigned char Uart0RxCounter;

unsigned char Uart1RxData[4];//串口1接收数据缓存
unsigned char Uart1RxCounter;

unsigned char Rx[18];//解码接收9位共18个半位
int  T_dat,R_dat;//曼码发送数据
int TPS_PData,TPS_TData;
unsigned char T_com;//曼码发送命令

unsigned char TimeOutFlag;//发送超时标志

unsigned char T_dat_buf[22];//曼码发送缓冲 

unsigned int DataFlag;//下发标定系数开始标志
unsigned char Int_count;
unsigned char IntFlag;//接收帧状态

unsigned int Tt,k;
unsigned int T2cn;
unsigned char coeffdata[320];

unsigned char Uart0TxData[74];//串口发送数据缓存

unsigned char EEPAdd;//接收系数存放地址索引
unsigned int coeff_len,coeff_count;//接收系数长度，接收系数计数
unsigned int CompCounter;

unsigned int bf;

unsigned long lfib;//短整型转浮点数时用的临时变量
unsigned char crc16hi,crc16lo,crc16array[32];




void InitialIO(void);
void init_USART0(void);
void SendUart0_2(unsigned char len,unsigned int s);
void SendUart0_3(unsigned char len,unsigned int s,unsigned char cn);
void Pwm_startup(unsigned char motor);
void Start(void);
void SendManchester(void); 
unsigned char SampleADC(unsigned char ch);
void EEPROM_write(unsigned int Address,unsigned char Data);
unsigned char EEPROM_read(unsigned int Address);
void Delay_ms(unsigned int t);
void crc16(unsigned char r_data[],unsigned int length);
 
void int0_isr(void);

#pragma interrupt_handler int0_isr:iv_INT0
void int0_isr(void)//外部中断0
{ 
   TCCR2 = 0x00; //stop
   
    INT_DIS
	MANINT_DIS
   
  DELAY10
  EIFR|=0x01;//清INT0中断标志 
  if((PIND&0x01)==0x00)//高脉冲不足10us则直接跳出
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
     if((PIND&0x01)==0x00)
	 {
	   DELAY20
	   if((PIND&0x01)==0x00)
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
	  if((EIFR&0x01)==0x01)
	  {
	   DELAY10
	   EIFR|=0x01;//清INT0中断标志 
	   if((PIND&0x01)==0x01)//高脉冲维持了10us，则认为是下一个有效上升沿到来
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
     if((PIND&0x01)==0x00)
	 {
	   DELAY20
	   if((PIND&0x01)==0x00)
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
	  if((EIFR&0x01)==0x01)
	  {
	   DELAY10
	   EIFR|=0x01;//清INT0中断标志 
	   if((PIND&0x01)==0x01)//高脉冲维持了10us，则认为是下一个有效上升沿到来
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
   INT_EN
}

void InitialIO(void)
{//1输出，0输入
 PORTG&=0xe7; 
 DDRG|=0x18; 
 
 PORTD&=0xBF;
 DDRD|=0x40;
}


//UART1 initialize
// desired baud rate:9600
// actual baud rate:9615 (0.2%)
// char size: 8 bit
// parity: Disabled
void uart1_init(void)
{
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x02;
 UCSR1C = 0x06;
 UBRR1L = 0x67; //set baud rate lo
 UBRR1H = 0x00; //set baud rate hi
 UCSR1B = 0x98;
}

#pragma interrupt_handler uart1_rx_isr:iv_USART1_RXC
void uart1_rx_isr(void)//串口1接收中断
{
  if(Uart1RxCounter<4)
	{
    Uart1RxData[Uart1RxCounter]=UDR1;//接收串口数据,同时清空串口接收中断标志
 	Uart1RxCounter++;
	}
}


void SendUart1(unsigned char dat,unsigned int s)   //串口发送数据
{//要求流量板在s个8ms内通过串口返回数据
unsigned char t,i; 	

for(t=0;t<4;t++)
  Uart1RxData[t]=0xff;
  
Uart1RxCounter=0; //清串口接受计数  

while(!(UCSR1A&(1<<UDRE1)));   // 等待发送缓冲器为空
UDR1=dat;  

t=UDR1;//开串口接收中断前读空缓存
UCSR1B|=0x80;//开串口接收中断
                  
if(s>0)
{
 //超时设定
TimeOutFlag=0; //超时标志置0
Timer0Counter=s; //8ms定时循环次数
TCNT0=0x06;//定时8ms
TCCR0|=0x06;//256分频
TIFR|=0x01; //清定时器0中断标志
TIMSK|=0x01;//使能定时器0中断
while(TimeOutFlag==0);
TIMSK&=0xFE;//关定时器0中断

}

UCSR1B&=0x7f;//关串口接收中断
}




//TIMER3 initialize - prescale:1
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 19200Hz
// actual value: 19230.769Hz (0.2%)
void timer3_init(void)
{
 
 ETIMSK&=0xfb;//禁止time3溢出中断   
 ETIFR&=0xfb;  //清timer3中断标志  
 TCCR3B = 0x00; //stop
 TPSBAUD
 TCCR3B = 0x01; //start Timer
 ETIFR&=0xfb;           //清timer3中断标志
}

void SendTPS(unsigned char tx_buf)
{
    unsigned char i;
	
    timer3_init();   
	
	while((ETIFR&0xfb)==0);//等待 timer3中断标志
	TPSBAUD
	ETIFR&=0xfb;           //清timer3中断标志
    SET_TX();//同步位
	
	for(i=0; i<8;i++)
	{
	while((ETIFR&0xfb)==0);//等待 timer3中断标志
	TPSBAUD
	ETIFR&=0xfb;           //清timer3中断标志
	if (tx_buf & (1 << i))  
       {CLR_TX();}
    else
       {SET_TX();}
	}
	  
	while((ETIFR&0xfb)==0);//等待 timer3中断标志
	TPSBAUD
	CLR_TX();//结束位
	
	//以下是等结束位结束后，再多等待2位，防止连续发送时接收方忙不过来
	while((ETIFR&0xfb)==0);//等待 timer3中断标志
	TPSBAUD
	while((ETIFR&0xfb)==0);//等待 timer3中断标志
	TPSBAUD
	while((ETIFR&0xfb)==0);//等待 timer3中断标志
	TPSBAUD
	
	TCCR3B = 0x00; //stop  
	ETIFR&=0xfb;           //清timer3中断标志
    
}

unsigned char ReceiveTPS(void)
{
    unsigned char i,rx_buf;
	
	rx_buf=0xff;
	TCCR1B = 0x00; //stop
    TCNT1 = 65380;  //定时20ms，9600波特率下接收一个字节实际只需耗时1.04ms，压力板返回时故意延迟了2ms，但有时遇到采集中断则需要10ms以上
	TIFR|=0x04; //清定时器1中断标志 
 	TCCR1B = 0x05; //1024分频
	
	//DDRG|=0x01;
	//PORTG|=0x01;  
	//PORTG&=0xfe; 
    while((TIFR&0x04)==0x00) 
	{
	   if (!GET_RX())//检测到了低电平的起始位
	     {  PORTG|=0x01;    
            timer3_init(); 
	 /*
			for(i=0; i<8;i++)
			{
	 		  while((ETIFR&0xfb)==0);//等待 timer3中断标志
			  PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           //根据端口电平, 写接收缓冲相应位 
                {rx_buf |= (1 << i);}
              else
                {rx_buf &= ~(1 << i);}
		    }
			*/
			
			 while((ETIFR&0xfb)==0);//等待 timer3中断标志
			 // PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           /* 根据端口电平, 写接收缓冲相应位 */
                {rx_buf |= (1 << 0);}
              else
                {rx_buf &= ~(1 << 0);}
			
			while((ETIFR&0xfb)==0);//等待 timer3中断标志
			//  PORTG|=0x01; 
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           /* 根据端口电平, 写接收缓冲相应位 */
                {rx_buf |= (1 << 1);}
              else
                {rx_buf &= ~(1 << 1);}
				
			while((ETIFR&0xfb)==0);//等待 timer3中断标志
			//  PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           /* 根据端口电平, 写接收缓冲相应位 */
                {rx_buf |= (1 << 2);}
              else
                {rx_buf &= ~(1 << 2);}
			
			while((ETIFR&0xfb)==0);//等待 timer3中断标志
			 // PORTG|=0x01; 
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           /* 根据端口电平, 写接收缓冲相应位 */
                {rx_buf |= (1 << 3);}
              else
                {rx_buf &= ~(1 << 3);}
			
			while((ETIFR&0xfb)==0);//等待 timer3中断标志
			//  PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           /* 根据端口电平, 写接收缓冲相应位 */
                {rx_buf |= (1 << 4);}
              else
                {rx_buf &= ~(1 << 4);}
			
			while((ETIFR&0xfb)==0);//等待 timer3中断标志
			 // PORTG|=0x01; 
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           /* 根据端口电平, 写接收缓冲相应位 */
                {rx_buf |= (1 << 5);}
              else
                {rx_buf &= ~(1 << 5);}
			
			while((ETIFR&0xfb)==0);//等待 timer3中断标志
			 // PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           /* 根据端口电平, 写接收缓冲相应位 */
                {rx_buf |= (1 << 6);}
              else
                {rx_buf &= ~(1 << 6);}
			
			while((ETIFR&0xfb)==0);//等待 timer3中断标志
			 // PORTG|=0x01; 
			  TPSBAUD
			  ETIFR&=0xfb;           //清timer3中断标志
			  if (GET_RX())           /* 根据端口电平, 写接收缓冲相应位 */
                {rx_buf |= (1 << 7);}
              else
                {rx_buf &= ~(1 << 7);}
						
			 while((ETIFR&0xfb)==0);//等待 timer3中断标志
			// PORTG&=0xfe; 
			 TPSBAUD
			 ETIFR&=0xfb;           //清timer3中断标志
			 
			 TCCR3B = 0x00; //stop  
	         ETIFR&=0xfb;           //清timer3中断标志
			  
			if (GET_RX()) //检测到了高电平结束位
			 {break;}
		 }
	}
	TCCR1B = 0x00; //stop
	TIFR|=0x04; //清定时器1中断标志 
	
	return rx_buf;
}

void SampleTPS(unsigned char cmd)
{   
    unsigned char rx_fifo[4];
   
	SendTPS(cmd);  //采样第2路压力温度，数字量或工程量

	rx_fifo[0]=ReceiveTPS();
	rx_fifo[1]=ReceiveTPS();
	rx_fifo[2]=ReceiveTPS();
	rx_fifo[3]=ReceiveTPS();
	
	TPS_PData=rx_fifo[1];
	TPS_PData<<=8;
	TPS_PData|=rx_fifo[0];
	TPS_TData=rx_fifo[3];
	TPS_TData<<=8;
	TPS_TData|=rx_fifo[2];
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

#pragma interrupt_handler uart0_rx_isr:iv_USART0_RXC
void uart0_rx_isr(void)//串口0接收中断
{
    if(Uart0RxCounter<74)
	{
    Uart0RxData[Uart0RxCounter]=UDR0;//接收串口数据,同时清空串口接收中断标志
 	Uart0RxCounter++;
	}
}

#pragma interrupt_handler timer0_ovf_isr:iv_TIM0_OVF
void timer0_ovf_isr(void)
{
Timer0Counter--;
if(Timer0Counter==0)
{
 TCNT0=0;
 TimeOutFlag=1;
 TIMSK&=0xFE;//关定时器0中断
 UCSR0B&=0x7f;//关串口接收中断
 }
 else
 {
 TCNT0=0x06;//定时8ms
 }
}

 

void SendUart0_2(unsigned char len,unsigned int s)   //串口发送数据
{//要求流量板在s个8ms内通过串口返回数据
unsigned char t,i; 	

for(t=0;t<74;t++)
  Uart0RxData[t]=0xff;
  
Uart0RxCounter=0; //清串口接受计数  

for(t=0;t<len;t++)
{
while(!(UCSR0A&(1<<UDRE0)));   // 等待发送缓冲器为空
for(i=0;i<200;i++);
UDR0=Uart0TxData[t];  
}
                  
if(s>0)
{
 //超时设定
TimeOutFlag=0; //超时标志置0
Timer0Counter=s; //8ms定时循环次数
TCNT0=0x06;//定时8ms
TCCR0|=0x06;//256分频
TIFR|=0x01; //清定时器0中断标志
TIMSK|=0x01;//使能定时器0中断
t=UDR0;//开串口接收中断前读空缓存
UCSR0B|=0x80;//开串口接收中断
while(TimeOutFlag==0);
TIMSK&=0xFE;//关定时器0中断
UCSR0B&=0x7f;//关串口接收中断
}
}

void SendUart0_3(unsigned char len,unsigned int s,unsigned char cn)   //串口发送数据，不死等，收到采集板返回的cn个字节后即退出，如一直没收到cn个字节，则超时时间s*8ms到后也退出
{//要求流量板在s个8ms内通过串口返回数据
unsigned char t,i; 	

for(t=0;t<74;t++)
  Uart0RxData[t]=0xff;
  
Uart0RxCounter=0; //清串口接受计数  

for(t=0;t<len;t++)
{
while(!(UCSR0A&(1<<UDRE0)));   // 等待发送缓冲器为空
for(i=0;i<200;i++);
UDR0=Uart0TxData[t];  
}
                  
if(s>0)
{
 //超时设定
TimeOutFlag=0; //超时标志置0
Timer0Counter=s; //8ms定时循环次数
TCNT0=0x06;//定时8ms
TCCR0|=0x06;//256分频
TIFR|=0x01; //清定时器0中断标志
TIMSK|=0x01;//使能定时器0中断
t=UDR0;//开串口接收中断前读空缓存
UCSR0B|=0x80;//开串口接收中断
while(TimeOutFlag==0)
{
 if(Uart0RxCounter>=cn)
   break;
}
TIMSK&=0xFE;//关定时器0中断
UCSR0B&=0x7f;//关串口接收中断
}
}

 
   
unsigned char SampleADC(unsigned char ch)//耗时约5ms
{     unsigned char c;
      unsigned long adcl;
	  unsigned int ADdata;

	  ADMUX  = (0xc0+ch);//片内2.56V基准，选择单端输入通道 
	  ADCSRA =0xC3;//ADC使能，ADC开始转换，ADC自动触发使能（连续转换模式），8分频 
   
	  //第一次采样值不要
	  for(ADdata=0;ADdata<100;ADdata++);
	  while((ADCSRA&0x40)==0x40);//等待转换完成
	  ADdata=ADCL;
	  ADdata=ADCH;  
  
	  adcl=0;
	  for(c=0;c<32;c++)//32次值取平均
	  {
	   ADCSRA = 0xC3;//ADC使能，ADC开始转换，ADC自动触发使能（连续转换模式），8分频 
	   for(ADdata=0;ADdata<100;ADdata++);
	   while((ADCSRA&0x40)==0x40);//等待转换完成	
	   ADdata=ADCL;   
	   ADdata|=(ADCH<<8);
	   ADdata&=0x03ff;//10位有效转换结果
	   adcl+=ADdata;
	  }

	  adcl>>=7;//除以32，再除以4，相当于8位AD
	  return (adcl&0x000000ff);//取8位有效位
}

void Delay_ms(unsigned int t)//最大定时8388ms
{   
    float f;
	unsigned char a;
    unsigned int n;
	
	if(t==0){return;}
	
    TCCR1B = 0x00; //stop
	if(t>8388)
	   t=8388;
	f=t;
	f*=7.8125;
	n=f;
    TCNT1 = 65535-n;  
	TIFR|=0x04; //清定时器1中断标志 
 	TCCR1B = 0x05; //1024分频
    while((TIFR&0x04)==0x00) ; 
    TCCR1B = 0x00; //stop
}


void Start(void)
{
 InitialIO();
 uart0_init();
 uart1_init();

 IntFlag=0;//无接收帧中断
 DataFlag=0;//无标定系数下发 
 coeff_len=0;
 coeff_count=0;//标定系数计数
 

 
	EICRA|=0x03; //INT0上升沿触发 ，每2位控制一个中断，共4个中断，0-低电平触发，1-保留，2-下降沿触发，3-上升沿触发 
	MANINT_CLR
	MANINT_EN
	INT_EN
}

 

void main(void)
{ 
  unsigned char i;
  unsigned int add,at,mk,pt,j;
  unsigned long l;
  unsigned int TestDataBlockIndex,TestDataBlockCount;
  float f;
 
	for(l=0;l<1000;l++)
	{
		for(j=0;j<1000;j++);
	}

	Start();

   Delay_ms(50); 


   
	//采集总线电压，判断是否进入存储状态  
	if(SampleADC(0)<30)//Cable端的电压低于30V 
	{	  
		UCSR0B = 0x00;
		PORTE&=0xFD;
		DDRE|=0x02;
		PORTE&=0xFD;	 
		while(1); 
	}
	     
	while(1)//若Cable端为高电压，则进入正常的通讯状态
	{ 
		 

		if(IntFlag==2)//校验位错误，不上传任何响应，上位机按超时处理
		{ 
			IntFlag=0;//无命令帧
			EIFR|=0x01;//清INT0中断标志
			EIMSK|=0x01;//使能INT0中断	 
		}
		else if(IntFlag==1)//接收到了正常命令帧
		{
			if(DataFlag)
			{ 		  
				coeffdata[coeff_count]=R_dat;
				coeff_count++;	  

				if(coeff_count==coeff_len)
				{
					if(DataFlag==1)//下发压力温度系数
					{
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x88;
						Uart0TxData[3]=0;//压力温度系数包0
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x88))
						{T_dat&=0xff00; }
						else
						{T_dat=0x00aa; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x88;
						Uart0TxData[3]=1;//压力温度系数包1
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+64];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x88))
						{T_dat&=0x00ff; }
						else
						{T_dat|=0x5500; }

						DataFlag=0;
						coeff_len=0;	
						T_com=0x0A;
						SendManchester();		
					}
					else if(DataFlag==2)//下发流量系数
					{
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x88;
						Uart0TxData[3]=2;//流量系数包2
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x88))
						{T_dat&=0xff00; }
						else
						{T_dat=0x00aa; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x88;
						Uart0TxData[3]=3;//流量系数包3
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+64];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x88))
						{T_dat&=0x00ff; }
						else
						{T_dat|=0x5500; }

						DataFlag=0;
						coeff_len=0;	
						T_com=0x0B;
						SendManchester();		
					}	
					else if(DataFlag==3)//提取测试数据
					{
						DataFlag=0;
						coeff_len=0;	
						T_com=0x09;
						TestDataBlockIndex=coeffdata[1];
						TestDataBlockIndex<<=8;
						TestDataBlockIndex|=coeffdata[0];//首块地址/索引
						TestDataBlockCount=coeffdata[3];
						TestDataBlockCount<<=8;
						TestDataBlockCount|=coeffdata[2];//需提取的总包数，每包64字节，每块4096字节共64包

						for(k=0;k<TestDataBlockCount;k++)//按用户要求分包进行上传，每包64字节传32帧
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x9A;//读测试数据
							Uart0TxData[3]=8;

							//统一块索引
							Uart0TxData[4]=(k); 	
							Uart0TxData[5]=(k>>8); 	//包索引	
							Uart0TxData[6]=(TestDataBlockIndex);
							Uart0TxData[7]=(TestDataBlockIndex>>8);//首块索引

							/*
							// 单独块索引
							Uart0TxData[4]=(k%8); 	
							Uart0TxData[5]=0; 	//包索引	
							Uart0TxData[6]=(TestDataBlockIndex+k/8);
							Uart0TxData[7]=((TestDataBlockIndex+k/8)>>8);//首块索引
							*/

							SendUart0_2(9,6);//超时48ms
							for(bf=0;bf<32;bf++)
							{	   	
								T_dat=Uart0RxData[bf*2+1];//高字节
								T_dat<<=8;
								T_dat|=Uart0RxData[bf*2+0];//低字节
								SendManchester();	
								Delay_ms(10);	 
							}		
						}
					} 
					else if(DataFlag==4)//下发仪器信息
					{
						T_dat=0x0000;

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=0;//包0
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0001; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=1;//包1
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+64];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0002; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=2;//包2
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+128];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0004; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=3;//包3
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+192];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0008; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=4;//包4
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+256];  
						}		 
						SendUart0_2(74,20);//超时160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0010; }

						DataFlag=0;
						coeff_len=0;	
						T_com=0x0C;
						SendManchester();		
					}     
				}	 
			}

			else//若不是下发标定系数，则进入以下各个命令的子语句
			{ 
				T_com=(R_dat>>4);
				switch(R_dat&0xf0)
				{ 
					case 0x10://复位 	
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x8D;//流量板握手，超时40ms
						Uart0TxData[3]=31;
						SendUart0_2(9,5); 
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x8D))
						{T_dat=0x00;}
						else
						{T_dat=0xff;}
						T_dat<<=8;
						T_dat|=0x00;
						SendManchester();	
						break;

					case 0x20://主控板与采集板版本号
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x8D;//流量板握手，超时40ms
						Uart0TxData[3]=31;
						SendUart0_2(9,5); 
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x8D))
							{T_dat=Uart0RxData[3];}
						else
							{T_dat=0xff;}    
						T_dat<<=8;
						T_dat|=BB;
						SendManchester();				
						break;

					case 0x30://Cable电压,电机电压
						Delay_ms(30);   
						f=SampleADC(0);//Cable 
						f=f*2.56;
						f=f*101;
						f=f/256;
						T_dat=f;
						T_dat<<=8;  
						
						SendUart1(0x30,4);//读电机电压，32ms超时
						T_dat|=Uart1RxData[0];

						SendManchester();   
						break;

					case 0x40: //采集板压力和温度
					    if(R_dat==0x4f)// 采集第2个压力温度的数字量，再多发2帧共4字节
						{
						  Delay_ms(30);//等总线稳定，否则会影响到内部总线，则第一个字节收不正常
						  
						  SampleTPS(R_dat);//最快5ms返回，没有挂压力短节时最多20ms返回
						  
						  Delay_ms(30);	
						  T_dat=TPS_PData;
						  SendManchester();//压力2
						  
						  Delay_ms(30);	
						  T_dat=TPS_TData;
						  SendManchester();//温度2
						}
						else
						{
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x89;
						Uart0TxData[3]=9;
						SendUart0_2(9,50);//流量板直读检测，超时400ms

						T_dat=Uart0RxData[1];
						T_dat<<=8;
						T_dat|=Uart0RxData[0];
						SendManchester();//压力

						Delay_ms(30);	
						T_dat=Uart0RxData[3];
						T_dat<<=8;
						T_dat|=Uart0RxData[2];
						SendManchester();//温度
						}
						
						break;

					case 0x50://流量采样	    	          	  	   	  	 
						/*	
						Delay_ms(40); //40ms定时		
						myFIB.f=123.456; 
						T_dat=myFIB.i[1];
						SendManchester();
						Delay_ms(40);
						T_dat=myFIB.i[0];
						SendManchester();
						Delay_ms(40);
						myFIB.f=789.012; 
						T_dat=myFIB.i[1];
						SendManchester();
						Delay_ms(40);
						T_dat=myFIB.i[0];
						SendManchester();
						Delay_ms(40);
						*/
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x89;
						Uart0TxData[3]=9;
						SendUart0_2(9,50);//流量板直读检测，超时400ms

						lfib=Uart0RxData[5];
						lfib<<=8;
						lfib|=Uart0RxData[4];
						lfib&=0x0000ffff;
						myFIB.f=(unsigned long)lfib; 

						T_dat=myFIB.i[1];
						SendManchester();//流量-时间
						crc16array[0]=(unsigned char)(T_dat);
						crc16array[1]=(unsigned char)(T_dat>>8);

						Delay_ms(30);	
						T_dat=myFIB.i[0];
						SendManchester();//流量-时间
						crc16array[2]=(unsigned char)(T_dat);
						crc16array[3]=(unsigned char)(T_dat>>8);

						Delay_ms(30);	

						lfib=Uart0RxData[7];
						lfib<<=8;
						lfib|=Uart0RxData[6];
						lfib&=0x0000ffff;
						myFIB.f=(unsigned long)lfib; 

						T_dat=myFIB.i[1];
						SendManchester();//流量-相位1
						crc16array[4]=(unsigned char)(T_dat);
						crc16array[5]=(unsigned char)(T_dat>>8);


						Delay_ms(30);	
						T_dat=myFIB.i[0];
						SendManchester();//流量-相位2
						crc16array[6]=(unsigned char)(T_dat);
						crc16array[7]=(unsigned char)(T_dat>>8);

						Delay_ms(30);	
						crc16(crc16array,8);//0.75ms
						T_dat=crc16hi;
						T_dat<<=8;
						T_dat|=crc16lo;	  
						SendManchester();//上传CRC校验共2字节
						break;
					
					//收放电机 
					case 0x60:  
						if( !(((R_dat)==0x61)  || ((R_dat)==0x62) ) )//既不是正转，也不是反转，则立即返回且不响应上位机
							break;

					    SendUart1(R_dat,200);//1600ms超时
						T_dat=Uart1RxData[1];
						T_dat<<=8;
						T_dat|=Uart1RxData[0];//返回电机状态
						SendManchester();	 

						break;

					case 0x70://调节电机   
						if( ((R_dat)<0x71)  || ((R_dat)>0x7c)  )//既不是调大，也不是调小，也不微调大，也不是微调小，则立即返回且不响应上位机
						break;

				        SendUart1(R_dat,200);//1600ms超时
						T_dat=Uart1RxData[1];
						T_dat<<=8;
						T_dat|=Uart1RxData[0];
						SendManchester();	 
						break;

					case 0x80://电机相关
						if(R_dat==0x80)//只开电机电源，不开电机
						{        	  
							SendUart1(0x80,4);//32ms超时
						    T_dat=Uart1RxData[0];
							SendManchester();
						}	  
						else if(R_dat==0x81)//获取电机状态	
						{ 
						    SendUart1(0x81,4);//32ms超时
						
							//先处理收放电机
							T_dat=Uart1RxData[1];
						    T_dat<<=8;
						    T_dat|=Uart1RxData[0];
							SendManchester();

							//再处理调节电机 
							Delay_ms(30);//延迟10ms  
							T_dat=Uart1RxData[3];
						    T_dat<<=8;
						    T_dat|=Uart1RxData[2];
							SendManchester();
						}	  
						else if(R_dat==0x82)//电机停止，同时停止二个电机
						{	 
							SendUart1(0x82,30);//240ms超时
						    T_dat=Uart1RxData[0];
							SendManchester(); 
						}
						else if(R_dat==0x83)//读取电机堵转电流档位和断路档位
						{
							SendUart1(0x83,4);//32ms超时
							T_dat=Uart1RxData[1];
						    T_dat<<=8;
						    T_dat|=Uart1RxData[0];
							SendManchester();
						}	
						else if(R_dat>0x83)//还剩12档，用于设置电机堵转电流, 收放电机和调节电机各6档
						{
							SendUart1(R_dat,4);//32ms超时
						    T_dat=Uart1RxData[0];
							SendManchester();
						}	  
						break;



					case 0x90: //标定数据
						if(R_dat==0x90)//提取测试信息
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0xc8;
							Uart0TxData[3]=0;//测试信息包0
							SendUart0_2(9,4);// 超时32ms	 
							for(i=0;i<32;i++)
							{
								Delay_ms(20);
								T_dat=Uart0RxData[i*2+1];//高字节
								T_dat<<=8;
								T_dat|=Uart0RxData[i*2+0];//低字节
								SendManchester();		 
							}		

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0xc8;
							Uart0TxData[3]=1;//测试信息包1
							SendUart0_2(9,4);// 超时32ms	 
							for(i=0;i<32;i++)
							{
								T_dat=Uart0RxData[i*2+1];//高字节
								T_dat<<=8;
								T_dat|=Uart0RxData[i*2+0];//低字节
								SendManchester();
								Delay_ms(20);
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0xc8;
							Uart0TxData[3]=2;//测试信息包2
							SendUart0_2(9,4);// 超时32ms	 
							for(i=0;i<32;i++)
							{
								T_dat=Uart0RxData[i*2+1];//高字节
								T_dat<<=8;
								T_dat|=Uart0RxData[i*2+0];//低字节
								SendManchester();
								Delay_ms(20);
							}		

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0xc8;
							Uart0TxData[3]=3;//测试信息包3
							SendUart0_2(9,4);// 超时32ms	 
							for(i=0;i<32;i++)
							{
								T_dat=Uart0RxData[i*2+1];//高字节
								T_dat<<=8;
								T_dat|=Uart0RxData[i*2+0];//低字节
								SendManchester();
								Delay_ms(20);
							}			
						}
						else if(R_dat==0x91)//上传压力标定数据,20ms/帧 	
						{
							//先获取总标定点数    
							DataFlag=3;
							coeff_len=4;//上位机还需传4字节，分别为首块地址和包数
							coeff_count=0;
							TimeOutFlag=0;
							Timer0Counter=200;//1.6s定时
							TCNT0=0x06; //8ms
							TCCR0|=0x06;//256分频
							TIFR|=0x01; //清定时器0中断标志
							TIMSK|=0x01;//使能定时器0中断		
						}
						else if(R_dat==0x92)//清空标定数据,整机格式化  
						{ 	
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x8c;
							Uart0TxData[3]=8;
							Uart0TxData[4]=0;
							Uart0TxData[5]=0;
							Uart0TxData[6]=0xff;
							Uart0TxData[7]=0xff;
							SendUart0_3(9,5000,9);// 最大超时40秒
							if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x8c))
								{T_dat=0x0000;}
							else
								{T_dat=0xffff;}
							SendManchester();	 
						}
						else if(R_dat==0x93)//上传全部系数，共256字节分128帧上传
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=0;//系数包0
							SendUart0_2(9,4);// 超时32ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=1;//系数包1
							SendUart0_2(9,4);// 超时32ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();
									Delay_ms(20);
								}
							}		

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=2;//系数包2
							SendUart0_2(9,4);// 超时32ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=3;//系数包3
							SendUart0_2(9,4);// 超时32ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();
									Delay_ms(20);
								}
							}		
						}
						else if(R_dat>=0x9a)//设置电机断路电流 
						{
							SendUart1(R_dat,4);//32ms超时
						    T_dat=Uart1RxData[0];
							SendManchester();
						}
						 
						break;

					case 0xa0://压力标定系数
						if(R_dat==0xa0)//下发全部标定系数，共128字节,20ms/帧，共2560ms   
							{ 
							DataFlag=1;
							coeff_len=128;
							coeff_count=0;
							TimeOutFlag=0;
							Timer0Counter=1000;//8s定时
							TCNT0=0x06; //8ms
							TCCR0|=0x06;//256分频
							TIFR|=0x01; //清定时器0中断标志
							TIMSK|=0x01;//使能定时器0中断				    
						}
						else if(R_dat==0xa1)//上传标定系数,28ms/帧 ，共64帧128字节
						{    
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=0;//系数包0
							SendUart0_2(9,5);// 超时40ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=1;//系数包1
							SendUart0_2(9,5);// 超时40ms		 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();
									Delay_ms(20);
								}
							}		
						}
						break;


					case 0xB0://流量相关
						if(R_dat==0xb0) //下发流量系数
						{
							DataFlag=2;
							coeff_len=128;
							coeff_count=0;
							TimeOutFlag=0;
							Timer0Counter=1000;//8s定时
							TCNT0=0x06; //8ms
							TCCR0|=0x06;//256分频
							TIFR|=0x01; //清定时器0中断标志
							TIMSK|=0x01;//使能定时器0中断		
						}
						else  if(R_dat==0xb1)//上提流量系数
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=2;//系数包2
							SendUart0_2(9,5);// 超时40ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=3;//系数包3
							SendUart0_2(9,5);// 超时40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();
									Delay_ms(20);
								}
							}		
						}
						/*   if(R_dat==0xb1)//读幅值
						{
						SendUart0(0xb0,5);//40ms定时
						for(k=0;k<8;k+=2)
						{
						T_dat=Uart0RxData[k];
						T_dat<<=8;
						T_dat|=Uart0RxData[k+1];
						SendManchester();
						Delay_ms(40);
						}
						}
						else  if((R_dat>0xb0) && (R_dat<0xb5) )//1800点提取,连续提取流量测试数据，4个通道分别提取，命令为0xB1-0xB4
						{   
						SendUart0(R_dat,125);//1s定时	 
						for(k=0;k<1800;k++)//共1800帧数据，全部提取
						{
						SendUart0(0xFF,1);//8ms定时	   
						T_dat=Uart0RxData[0];
						T_dat<<=8;
						T_dat|=Uart0RxData[1];
						SendManchester();
						Delay_ms(42);
						}
						} */  
						break;

					case 0xc0://所有参数全采，上传16字节共8帧
						if( (R_dat==0xc0) || (R_dat==0xcf) )
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x89;
							Uart0TxData[3]=9;
							SendUart0_2(9,50);//流量板直读检测，超时400ms
							//if(Uart0RxCounter==64)//成功接收到压力、温度、流量数据，先低字节后高字节
							{
								T_dat=Uart0RxData[1];
								T_dat<<=8;
								T_dat|=Uart0RxData[0];
								SendManchester();//压力
								crc16array[0]=(unsigned char)(T_dat);
								crc16array[1]=(unsigned char)(T_dat>>8);


								Delay_ms(30);	
								T_dat=Uart0RxData[3];
								T_dat<<=8;
								T_dat|=Uart0RxData[2];
								SendManchester();//温度
								crc16array[2]=(unsigned char)(T_dat);
								crc16array[3]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	

								lfib=Uart0RxData[5];
								lfib<<=8;
								lfib|=Uart0RxData[4];
								lfib&=0x0000ffff;
								myFIB.f=(unsigned long)lfib; 

								T_dat=myFIB.i[1];
								SendManchester();//流量-时间1
								crc16array[4]=(unsigned char)(T_dat);
								crc16array[5]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	
								T_dat=myFIB.i[0];
								SendManchester();//流量-时间2
								crc16array[6]=(unsigned char)(T_dat);
								crc16array[7]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	

								lfib=Uart0RxData[7];
								lfib<<=8;
								lfib|=Uart0RxData[6];
								lfib&=0x0000ffff;
								myFIB.f=(unsigned long)lfib; 

								T_dat=myFIB.i[1];
								SendManchester();//流量-相位1
								crc16array[8]=(unsigned char)(T_dat);
								crc16array[9]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	
								T_dat=myFIB.i[0];
								SendManchester();//流量-相位2
								crc16array[10]=(unsigned char)(T_dat);
								crc16array[11]=(unsigned char)(T_dat>>8);

								 
								SendUart1(0x81,4);//电机状态，32ms超时
								
							    T_dat=Uart1RxData[1];
						        T_dat<<=8;
						        T_dat|=Uart1RxData[0];						 
								SendManchester();//收放电机状态
								crc16array[12]=(unsigned char)(T_dat);
								crc16array[13]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	 
								T_dat=Uart1RxData[3];
						        T_dat<<=8;
						        T_dat|=Uart1RxData[2];
								SendManchester();//调节电机状态
								crc16array[14]=(unsigned char)(T_dat);
								crc16array[15]=(unsigned char)(T_dat>>8);

								Delay_ms(1);	  					
						        SendUart1(0x31,4);//读电机电压，32ms超时
						        T_dat=Uart1RxData[0];
								SendManchester();
								crc16array[16]=(unsigned char)(T_dat);
								crc16array[17]=(unsigned char)(T_dat>>8);

								if(R_dat==0xcf)// 采集第2个压力温度的工程量，再多发2帧共4字节
								{
						  		 SampleTPS(R_dat);//正常情况下10ms返回，不挂压力短节的话20*4=80ms超时后返回
						  
						  		 Delay_ms(25);	
						  		 T_dat=TPS_PData;
						  		 SendManchester();//压力
								 crc16array[18]=(unsigned char)(T_dat);
								 crc16array[19]=(unsigned char)(T_dat>>8);
						  
						  		 Delay_ms(30);	
						  		 T_dat=TPS_TData;
						  		 SendManchester();//温度
								 crc16array[20]=(unsigned char)(T_dat);
								 crc16array[21]=(unsigned char)(T_dat>>8);
								 
								 Delay_ms(30);	
								 crc16(crc16array,22);//0.75ms
								 T_dat=crc16hi;
								 T_dat<<=8;
								 T_dat|=crc16lo;	  
								 SendManchester();//上传CRC校验共2字节，从收到命令到上传完毕共耗时约850ms
								}
								else
								{
								 Delay_ms(30);	
								 crc16(crc16array,18);//0.75ms
								 T_dat=crc16hi;
								 T_dat<<=8;
								 T_dat|=crc16lo;	  
								 SendManchester();//上传CRC校验共2字节，从收到命令到上传完毕共耗时约760ms
								}
							}	
						}
						else  if(R_dat==0xc5) //读仪器信息,共320字节，分5次从流量板中提取，每包64字节
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=0;//包0
							SendUart0_2(9,5);// 超时40ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=1;//包1
							SendUart0_2(9,5);// 超时40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();
									Delay_ms(20);
								}
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=2;//包2
							SendUart0_2(9,5);// 超时40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();
									Delay_ms(20);
								}
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=3;//包3
							SendUart0_2(9,5);// 超时40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();
									Delay_ms(20);
								}
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=4;//包4
							SendUart0_2(9,5);// 超时40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//高字节
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//低字节
									SendManchester();
									Delay_ms(20);
								}
							}	
						}
						else  if(R_dat==0xca) //写仪器信息
						{
							DataFlag=4;
							coeff_len=320;
							coeff_count=0;
							TimeOutFlag=0;
							Timer0Counter=2000;//16s定时
							TCNT0=0x06; //8ms
							TCCR0|=0x06;//256分频
							TIFR|=0x01; //清定时器0中断标志
							TIMSK|=0x01;//使能定时器0中断		
						}
						break;
					
					case 0xd0://压力板与电机板版本
					   if(R_dat==0xdf)//电机板与第2路压力板版本号	
					    {			 
						  SendUart1(0xdf,3);//电机板
						  T_dat=Uart1RxData[0];
						  T_dat<<=8;
						  
						  SendTPS(0xdf);//第2路压力板
						  T_dat|=ReceiveTPS();
						  
						  SendManchester();	
						}
					break;
					
					default: 
						break;	
				}
			}
			
		RDataEnd:
			IntFlag=0;
			EIFR|=0x01;//清INT0中断标志
			EIMSK|=0x01;//使能INT0中断 
		}  
	}
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
	        M1_H       
         	DELAY80
			DELAY80
			DELAY80
			DELAY80
			DELAY80
			DELAY80
			DELAY80
			M1_L
			DELAY40
	
 	EIFR|=0x08;//清INT3中断标志 
 	SREG|=0x80; //开总中断
}



void EEPROM_write(unsigned int Address,unsigned char Data)  	//内部EEPROM写
{//时钟频率为1MHz，典型的EEPROM字节写耗时约8.5ms
    while(EECR&0x02);                   // 等待上一次写操作结束
    EEAR=Address;
    EEDR=Data;                        	// 设置地址和数据寄存器
    EECR|=0x04;                         //置位EEMWE
    EECR|=0x02;                         //置位EEWE 以启动写操作
}

unsigned char EEPROM_read(unsigned int Address)  				//内部EEPROM读
{
    while(EECR&0x02);      				//等待上一次操作结束
    EEAR = Address;                     //设置地址寄存器						
	EECR|=0x01;             			//设置EERE 以启动读操作
    return EEDR;                   		//自数据寄存器返回数据
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