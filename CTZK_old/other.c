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
void init_USART1(void)  //USART0 ��ʼ��
{
	UCSR1B=0X00;
	UCSR1A=0X20;
	UCSR1C=0X06;
	UBRR1L=0X33;
	UBRR1H=0X00;
	UCSR1B=0X18;
}

void SendUart1(unsigned char c,unsigned int s)   //���ڷ�������
{//Ҫ����������s��8ms��ͨ�����ڷ�������
unsigned char t; 

for(t=0;t<200;t++)
  Uart1RxData[t]=0xff;
Uart1RxCounter=0; //�崮�ڽ��ܼ���  	
while(!(UCSR1A&(1<<UDRE1)));   // �ȴ����ͻ�����Ϊ��
UDR1=c;                      // �����ݷ��뻺��������������

if(s>0)
{
 //��ʱ�趨
TimeOutFlag=0; //��ʱ��־��0
Timer0Counter=s; //8ms��ʱѭ������
TCNT0=0x6;//��ʱ8ms
TCCR0|=0x06;//256��Ƶ
TIFR|=0x01; //�嶨ʱ��0�жϱ�־
TIMSK|=0x01;//ʹ�ܶ�ʱ��0�ж�
t=UDR1;//�����ڽ����ж�ǰ���ջ���
UCSR1B|=0x80;//�����ڽ����ж�
while(TimeOutFlag==0);
TIMSK&=0xFE;//�ض�ʱ��0�ж�
UCSR1B&=0x7f;//�ش��ڽ����ж�
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
  PORTD|=0x80;//�ڲ�����
  
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
 if( (ETIFR&0x04)==0x04 )//1s��ʱʱ�䵽
	{ 
	 if(IntFlag==0)
	  {
	   TCNT3=34286;//��ʱ1s
       TCCR3B|=0x04;//256��Ƶ
	   ETIFR|=0x04;//�嶨ʱ��3�жϱ�־
	 
	   if(Pumpflag==0)//���¶Ȳ�ʪ��
        {
		   if(IntFlag==0)
		   {
			 pt=sh_read();
    		 pt<<=8; 
    		 pt|=sh_read();
			 Tem=pt;
    
	         Pumpflag=1;
			 
    		 sh_reset();
			 sh_write(0x05);//ʪ��
			}
	    }
	   else//��ʪ�Ȳ��¶�
	    {
		   if(IntFlag==0)
		   {
			 pt=sh_read();
    		 pt<<=8; 
    		 pt|=sh_read();
			 Hum=pt;
			 
	         Pumpflag=0;
			
			sh_reset();
			sh_write(0x03);//ʪ�� 
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

 