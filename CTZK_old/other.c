/*
 unsigned int ThisCurrent:10;
 unsigned int OtheRun:1;
 unsigned int OutofPhase:1;
 unsigned int UnderVoltage:1;
 unsigned int OverCurrent:1;
 unsigned int CommandStop:1;
 unsigned int ThisRun:1;
 */


/*
void init_USART1(void)  //USART0 初始化
{
	UCSR1B=0X00;
	UCSR1A=0X20;
	UCSR1C=0X06;
	UBRR1L=0X33;
	UBRR1H=0X00;
	UCSR1B=0X18;
}

void SendUart1(unsigned char c,unsigned int s)   //串口发送数据
{//要求流量板在s个8ms内通过串口返回数据
unsigned char t; 

for(t=0;t<200;t++)
  Uart1RxData[t]=0xff;
Uart1RxCounter=0; //清串口接受计数  	
while(!(UCSR1A&(1<<UDRE1)));   // 等待发送缓冲器为空
UDR1=c;                      // 将数据放入缓冲器，发送数据

if(s>0)
{
 //超时设定
TimeOutFlag=0; //超时标志置0
Timer0Counter=s; //8ms定时循环次数
TCNT0=0x6;//定时8ms
TCCR0|=0x06;//256分频
TIFR|=0x01; //清定时器0中断标志
TIMSK|=0x01;//使能定时器0中断
t=UDR1;//开串口接收中断前读空缓存
UCSR1B|=0x80;//开串口接收中断
while(TimeOutFlag==0);
TIMSK&=0xFE;//关定时器0中断
UCSR1B&=0x7f;//关串口接收中断
}
}
*/
/*
void sh_write(unsigned char value)
{
  unsigned char i;
  DDRD|=0x80;
  for (i=0x80;i>0;i/=2)
  {
  if(i & value)
    {
    SHSDA_H
    }
  else 
    {
    SHSDA_L
    }  
  SHSCL_H
  DELAY5
  SHSCL_L  
  DELAY5
  }
  SHSDA_L
  SHSCL_H
  DELAY5
  SHSCL_L
  DELAY5   
  DDRD&=0x7F;
  PORTD|=0x80;
}


char sh_read(void)
{
  unsigned char i,val=0;
  DDRD&=0x7F;
  PORTD|=0x80;
  DELAY5  
  for (i=0x80;i>0;i/=2)
  {
  SHSCL_H
  DELAY5
  if ((PIND&0x80)==0x80)
  val=(val|i);
  SHSCL_L
  DELAY5
  }
  DDRD|=0x80;
  SHSDA_L
  SHSCL_H  
  DELAY5
  SHSCL_L
  DELAY5
   
  DDRD&=0x7F;
  PORTD|=0x80;//内部上拉
  
  return val;  
}


void sh_start(void)
{
 DDRD|=0x80;
 
  SHSDA_H
  DELAY5
  SHSCL_L
  DELAY5
  SHSCL_H
  DELAY5
  SHSDA_L
  DELAY5
  SHSCL_L
  DELAY5
  SHSCL_H
  DELAY5
  SHSDA_H
  DELAY5
  SHSCL_L
  DELAY5  
}

void sh_reset(void)
{
  unsigned char i;
   DDRD|=0x80;
   
  SHSDA_H
  DELAY5
  SHSCL_L
  DELAY5
  for (i=0;i<9;i++)
  {
  SHSCL_H
  DELAY5
  SHSCL_L
  DELAY5
  }
  sh_start();
} 
*/
 
 /*
 if( (ETIFR&0x04)==0x04 )//1s定时时间到
	{ 
	 if(IntFlag==0)
	  {
	   TCNT3=34286;//定时1s
       TCCR3B|=0x04;//256分频
	   ETIFR|=0x04;//清定时器3中断标志
	 
	   if(Pumpflag==0)//读温度采湿度
        {
		   if(IntFlag==0)
		   {
			 pt=sh_read();
    		 pt<<=8; 
    		 pt|=sh_read();
			 Tem=pt;
    
	         Pumpflag=1;
			 
    		 sh_reset();
			 sh_write(0x05);//湿度
			}
	    }
	   else//读湿度采温度
	    {
		   if(IntFlag==0)
		   {
			 pt=sh_read();
    		 pt<<=8; 
    		 pt|=sh_read();
			 Hum=pt;
			 
	         Pumpflag=0;
			
			sh_reset();
			sh_write(0x03);//湿度 
		   }   
	   }
	  }
	}
 */
 
 /*
 void watchdog_init(void)
{
    WDR(); //this prevents a timout on enabling
    WDTCR |= (1<<WDCE) | (1<<WDE);
    WDTCR = 0x08; //WATCHDOG ENABLED - dont forget to issue WDRs
}*/

 