 /**************************************************
CTZK
//20161110 ������ȫ��������CRC16У��
//20161113 �Ľ�������������ʱ�����λ���޷��Ŷ�����תΪ������ʱ��ת���������Է���λ������������λ����ֵ
//20161224 ����������ж�ֵ��26mA��Ϊ13mA����ΪĿǰ���ڵ������ʱ���ص���ֵ��Ϊ23~28mA�������汾ΪV4.0
//20161229 �޸����ŵ���ı����ͬ��ͷ����жϣ��޸ĵ��ڵ���ĵ��������ƣ���ԭ���෴��
//20170308 ������������У��֡�������汾ΪV4.2
//20170607 �����·������ԭ����13mA��Ϊ26mA �����汾ΪV4.3
//20170608 �Ķ����������λ�������λ������250mA�������汾ΪV4.4
//20170609 ������ڵ��΢�����ܣ������汾ΪV4.5
//20170613 ���ӵ����·������λ�����趨(��6��)�����ӵ��ڵ��΢��ʱ���ɵ���������Դ����������ϴ�������������Ϊ����������λmA������ת�����ϴ���Ϊ��λ���汾��ΪV4.6
//20170614 �ϴ��ĵ��µ�ѹҲ��Ϊ��������ȫ���������ӵ��µ�ѹ
//20170622  Ϊ��Ӧ�¸�ѹ��������ĵ�·����Ľ�һЩ���ܣ��汾���˵�V3.0
//20170701 Ϊ��Ӧ�µ�·��CT_MAIN_V30
//20170703 ���Ӷ���ѹ���̽ڵ�о�ڲ�����ͨѶ����
//20170703 �汾��6.0�𲽣�Ϊ����ǰһ�׶ε�Ӳ��������
//20170705 ѹ���̽ڹ��ܵ�ͨ V6.1
//20170711  �汾���˵�V1.0�������Ժ�Ĺ�����չ
**************************************************/
#include <iom128v.h>								   	
#include <macros.h>
#include <stdio.h>

 
#define Debug 0
#define  BB     0x10       //�̼��汾��


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

/* ���յ�ƽֵ */
#define GET_RX() (PIND & (1<<PIND1))  
/* ���͸ߵ�ƽ */   
#define SET_TX() (PORTD |= 0x40)
/* ���͵͵�ƽ */ 
#define CLR_TX() (PORTD &= 0xbf)
        

 

/* ��ʱ���жϷ���ʱ�Ķ�д���� */
enum timer_turn {   
 RX_TURN = 0,                /* ������ */    
 TX_TURN,                    /* д���� */
 };
 
/* ����֡�ṹ */
__flash enum frame_bit {    BIT_0 = 0,    BIT_1,    BIT_2,    BIT_3,    BIT_4,    BIT_5,    BIT_6,    BIT_7,    BIT_STOP,    BIT_IDLE,    BIT_START,};

union FIB
{
 float f;
 unsigned int i[2];
 unsigned char b[4];
}myFIB;

 
 
 

unsigned char T_Flag;

unsigned int Timer0Counter;//��ʱ��0����4msʱ�ļ���

unsigned char Uart0RxData[74];//����0�������ݻ���
unsigned char Uart0RxCounter;

unsigned char Uart1RxData[4];//����1�������ݻ���
unsigned char Uart1RxCounter;

unsigned char Rx[18];//�������9λ��18����λ
int  T_dat,R_dat;//���뷢������
int TPS_PData,TPS_TData;
unsigned char T_com;//���뷢������

unsigned char TimeOutFlag;//���ͳ�ʱ��־

unsigned char T_dat_buf[22];//���뷢�ͻ��� 

unsigned int DataFlag;//�·��궨ϵ����ʼ��־
unsigned char Int_count;
unsigned char IntFlag;//����֡״̬

unsigned int Tt,k;
unsigned int T2cn;
unsigned char coeffdata[320];

unsigned char Uart0TxData[74];//���ڷ������ݻ���

unsigned char EEPAdd;//����ϵ����ŵ�ַ����
unsigned int coeff_len,coeff_count;//����ϵ�����ȣ�����ϵ������
unsigned int CompCounter;

unsigned int bf;

unsigned long lfib;//������ת������ʱ�õ���ʱ����
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
void int0_isr(void)//�ⲿ�ж�0
{ 
   TCCR2 = 0x00; //stop
   
    INT_DIS
	MANINT_DIS
   
  DELAY10
  EIFR|=0x01;//��INT0�жϱ�־ 
  if((PIND&0x01)==0x00)//�����岻��10us��ֱ������
  {
   goto End;
  }
   
   TCNT2 = 0x00;  
   TCCR2 = 0x03; //64��Ƶ,8Mʱ�ӣ�ÿ��CLOCK��ʱ8us�����ʱ2048us��ÿ����10������
   
   while(1)
   {
     T2cn=TCNT2;
	  if(T2cn>80)//���ֵ���7�����ڣ��ڴ˶��һ�����ڣ��򳬹�8�����ڻ�û���������жϵ�����ֱ������
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
  		   
    //��ʼ�ж�ͬ��ͷ	
	while(1)//��û�������ص���,����
	{
	  T2cn=TCNT2;
	  if(T2cn>80)//���ֵ���7�����ڣ��ڴ˶��һ�����ڣ��򳬹�8�����ڻ�û���������жϵ�����ֱ������
	  {
	    goto End;
	  }
	  if((EIFR&0x01)==0x01)
	  {
	   DELAY10
	   EIFR|=0x01;//��INT0�жϱ�־ 
	   if((PIND&0x01)==0x01)//������ά����10us������Ϊ����һ����Ч�����ص���
	     {
   	     break;
		 }
	  }  
	}
	T2cn=TCNT2;
	TCNT2=0;
	if((T2cn<60)||(T2cn>80))//ͬ��ͷ��Ӧ��7�����ڣ����յ��Ľ��Ϊ6~8�������ڶ��Ͽ�
	{
	 goto End;//û���յ���ȷ��ͬ��ͷ��ֱ������
	}
 
Start:	//ͬ��ͷ������ȷ�����濪ʼ����9λ����λ	//��λ�Ӹߵ��ͽ�������λ��8λ���ݼ�1λУ�鹲9λ 		      	
  for(Int_count=9;Int_count>0;Int_count--)
  {	
   while(1)
   {
     T2cn=TCNT2;
	  if(T2cn>80)//���ֵ���7�����ڣ��ڴ˶��һ�����ڣ��򳬹�8�����ڻ�û���������жϵ�����ֱ������
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
   while(1)//��û�������ص���,����
	{
	  T2cn=TCNT2;
	  if(T2cn>80)//���ֵ���7�����ڣ��ڴ˶��һ�����ڣ��򳬹�8�����ڻ�û���������жϵ�����ֱ������
	  {
	    goto End;
	  }
	  if((EIFR&0x01)==0x01)
	  {
	   DELAY10
	   EIFR|=0x01;//��INT0�жϱ�־ 
	   if((PIND&0x01)==0x01)//������ά����10us������Ϊ����һ����Ч�����ص���
	     {
   	     break;
		 }
	  }  
	}
	T2cn=TCNT2;
	TCNT2=0;
	if(T2cn<20)//����λ��С��3λ���ɷſ���2λ
	{
	 IntFlag=3;//��������λ������
	 goto End;//����λ����̫�磬�˳�
	}
	else if(T2cn<40)//����λ1��Ӧ��3�����ڣ��ڴ�������Ϊ2~4�����ڶ��ǿ��Ե�
	{
	 Rx[Int_count]=1;
	}
	else if(T2cn<60)//����λ0��Ӧ��5�����ڣ��ڴ�������Ϊ4~6�����ڶ��ǿ��Ե�
	{
	 Rx[Int_count]=0;
	}
	else if(T2cn<=80)//ͬ��λ��Ӧ��7���ڣ��ڴ�������Ϊ6~8���ڶ��ǿ��Ե�
	{
	 goto Start;//�ڽ�������λʱ�յ���ͬ��λ����������λ������ʼλ��
	}
	else 
	{
	 IntFlag=3;//��������λ������
	 goto End;//����λ����̫�٣��˳�
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
		 IntFlag=1;//����֡����	
	 }
	else           
	 {
		 IntFlag=2;//����֡У���
	 }
 
   goto End1;//ֻҪ���յ�������ͬ��ͷ������У��λ�Ƕ��Ǵ�������ʱ������INT0�ж�
     
End:
		MANINT_CLR
		MANINT_EN
End1:
   TCCR2 = 0x00; //stop
   INT_EN
}

void InitialIO(void)
{//1�����0����
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
void uart1_rx_isr(void)//����1�����ж�
{
  if(Uart1RxCounter<4)
	{
    Uart1RxData[Uart1RxCounter]=UDR1;//���մ�������,ͬʱ��մ��ڽ����жϱ�־
 	Uart1RxCounter++;
	}
}


void SendUart1(unsigned char dat,unsigned int s)   //���ڷ�������
{//Ҫ����������s��8ms��ͨ�����ڷ�������
unsigned char t,i; 	

for(t=0;t<4;t++)
  Uart1RxData[t]=0xff;
  
Uart1RxCounter=0; //�崮�ڽ��ܼ���  

while(!(UCSR1A&(1<<UDRE1)));   // �ȴ����ͻ�����Ϊ��
UDR1=dat;  

t=UDR1;//�����ڽ����ж�ǰ���ջ���
UCSR1B|=0x80;//�����ڽ����ж�
                  
if(s>0)
{
 //��ʱ�趨
TimeOutFlag=0; //��ʱ��־��0
Timer0Counter=s; //8ms��ʱѭ������
TCNT0=0x06;//��ʱ8ms
TCCR0|=0x06;//256��Ƶ
TIFR|=0x01; //�嶨ʱ��0�жϱ�־
TIMSK|=0x01;//ʹ�ܶ�ʱ��0�ж�
while(TimeOutFlag==0);
TIMSK&=0xFE;//�ض�ʱ��0�ж�

}

UCSR1B&=0x7f;//�ش��ڽ����ж�
}




//TIMER3 initialize - prescale:1
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 19200Hz
// actual value: 19230.769Hz (0.2%)
void timer3_init(void)
{
 
 ETIMSK&=0xfb;//��ֹtime3����ж�   
 ETIFR&=0xfb;  //��timer3�жϱ�־  
 TCCR3B = 0x00; //stop
 TPSBAUD
 TCCR3B = 0x01; //start Timer
 ETIFR&=0xfb;           //��timer3�жϱ�־
}

void SendTPS(unsigned char tx_buf)
{
    unsigned char i;
	
    timer3_init();   
	
	while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
	TPSBAUD
	ETIFR&=0xfb;           //��timer3�жϱ�־
    SET_TX();//ͬ��λ
	
	for(i=0; i<8;i++)
	{
	while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
	TPSBAUD
	ETIFR&=0xfb;           //��timer3�жϱ�־
	if (tx_buf & (1 << i))  
       {CLR_TX();}
    else
       {SET_TX();}
	}
	  
	while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
	TPSBAUD
	CLR_TX();//����λ
	
	//�����ǵȽ���λ�������ٶ�ȴ�2λ����ֹ��������ʱ���շ�æ������
	while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
	TPSBAUD
	while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
	TPSBAUD
	while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
	TPSBAUD
	
	TCCR3B = 0x00; //stop  
	ETIFR&=0xfb;           //��timer3�жϱ�־
    
}

unsigned char ReceiveTPS(void)
{
    unsigned char i,rx_buf;
	
	rx_buf=0xff;
	TCCR1B = 0x00; //stop
    TCNT1 = 65380;  //��ʱ20ms��9600�������½���һ���ֽ�ʵ��ֻ���ʱ1.04ms��ѹ���巵��ʱ�����ӳ���2ms������ʱ�����ɼ��ж�����Ҫ10ms����
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־ 
 	TCCR1B = 0x05; //1024��Ƶ
	
	//DDRG|=0x01;
	//PORTG|=0x01;  
	//PORTG&=0xfe; 
    while((TIFR&0x04)==0x00) 
	{
	   if (!GET_RX())//��⵽�˵͵�ƽ����ʼλ
	     {  PORTG|=0x01;    
            timer3_init(); 
	 /*
			for(i=0; i<8;i++)
			{
	 		  while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			  PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           //���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ 
                {rx_buf |= (1 << i);}
              else
                {rx_buf &= ~(1 << i);}
		    }
			*/
			
			 while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			 // PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           /* ���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ */
                {rx_buf |= (1 << 0);}
              else
                {rx_buf &= ~(1 << 0);}
			
			while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			//  PORTG|=0x01; 
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           /* ���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ */
                {rx_buf |= (1 << 1);}
              else
                {rx_buf &= ~(1 << 1);}
				
			while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			//  PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           /* ���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ */
                {rx_buf |= (1 << 2);}
              else
                {rx_buf &= ~(1 << 2);}
			
			while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			 // PORTG|=0x01; 
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           /* ���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ */
                {rx_buf |= (1 << 3);}
              else
                {rx_buf &= ~(1 << 3);}
			
			while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			//  PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           /* ���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ */
                {rx_buf |= (1 << 4);}
              else
                {rx_buf &= ~(1 << 4);}
			
			while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			 // PORTG|=0x01; 
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           /* ���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ */
                {rx_buf |= (1 << 5);}
              else
                {rx_buf &= ~(1 << 5);}
			
			while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			 // PORTG&=0xfe;
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           /* ���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ */
                {rx_buf |= (1 << 6);}
              else
                {rx_buf &= ~(1 << 6);}
			
			while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			 // PORTG|=0x01; 
			  TPSBAUD
			  ETIFR&=0xfb;           //��timer3�жϱ�־
			  if (GET_RX())           /* ���ݶ˿ڵ�ƽ, д���ջ�����Ӧλ */
                {rx_buf |= (1 << 7);}
              else
                {rx_buf &= ~(1 << 7);}
						
			 while((ETIFR&0xfb)==0);//�ȴ� timer3�жϱ�־
			// PORTG&=0xfe; 
			 TPSBAUD
			 ETIFR&=0xfb;           //��timer3�жϱ�־
			 
			 TCCR3B = 0x00; //stop  
	         ETIFR&=0xfb;           //��timer3�жϱ�־
			  
			if (GET_RX()) //��⵽�˸ߵ�ƽ����λ
			 {break;}
		 }
	}
	TCCR1B = 0x00; //stop
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־ 
	
	return rx_buf;
}

void SampleTPS(unsigned char cmd)
{   
    unsigned char rx_fifo[4];
   
	SendTPS(cmd);  //������2·ѹ���¶ȣ��������򹤳���

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
void uart0_rx_isr(void)//����0�����ж�
{
    if(Uart0RxCounter<74)
	{
    Uart0RxData[Uart0RxCounter]=UDR0;//���մ�������,ͬʱ��մ��ڽ����жϱ�־
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
 TIMSK&=0xFE;//�ض�ʱ��0�ж�
 UCSR0B&=0x7f;//�ش��ڽ����ж�
 }
 else
 {
 TCNT0=0x06;//��ʱ8ms
 }
}

 

void SendUart0_2(unsigned char len,unsigned int s)   //���ڷ�������
{//Ҫ����������s��8ms��ͨ�����ڷ�������
unsigned char t,i; 	

for(t=0;t<74;t++)
  Uart0RxData[t]=0xff;
  
Uart0RxCounter=0; //�崮�ڽ��ܼ���  

for(t=0;t<len;t++)
{
while(!(UCSR0A&(1<<UDRE0)));   // �ȴ����ͻ�����Ϊ��
for(i=0;i<200;i++);
UDR0=Uart0TxData[t];  
}
                  
if(s>0)
{
 //��ʱ�趨
TimeOutFlag=0; //��ʱ��־��0
Timer0Counter=s; //8ms��ʱѭ������
TCNT0=0x06;//��ʱ8ms
TCCR0|=0x06;//256��Ƶ
TIFR|=0x01; //�嶨ʱ��0�жϱ�־
TIMSK|=0x01;//ʹ�ܶ�ʱ��0�ж�
t=UDR0;//�����ڽ����ж�ǰ���ջ���
UCSR0B|=0x80;//�����ڽ����ж�
while(TimeOutFlag==0);
TIMSK&=0xFE;//�ض�ʱ��0�ж�
UCSR0B&=0x7f;//�ش��ڽ����ж�
}
}

void SendUart0_3(unsigned char len,unsigned int s,unsigned char cn)   //���ڷ������ݣ������ȣ��յ��ɼ��巵�ص�cn���ֽں��˳�����һֱû�յ�cn���ֽڣ���ʱʱ��s*8ms����Ҳ�˳�
{//Ҫ����������s��8ms��ͨ�����ڷ�������
unsigned char t,i; 	

for(t=0;t<74;t++)
  Uart0RxData[t]=0xff;
  
Uart0RxCounter=0; //�崮�ڽ��ܼ���  

for(t=0;t<len;t++)
{
while(!(UCSR0A&(1<<UDRE0)));   // �ȴ����ͻ�����Ϊ��
for(i=0;i<200;i++);
UDR0=Uart0TxData[t];  
}
                  
if(s>0)
{
 //��ʱ�趨
TimeOutFlag=0; //��ʱ��־��0
Timer0Counter=s; //8ms��ʱѭ������
TCNT0=0x06;//��ʱ8ms
TCCR0|=0x06;//256��Ƶ
TIFR|=0x01; //�嶨ʱ��0�жϱ�־
TIMSK|=0x01;//ʹ�ܶ�ʱ��0�ж�
t=UDR0;//�����ڽ����ж�ǰ���ջ���
UCSR0B|=0x80;//�����ڽ����ж�
while(TimeOutFlag==0)
{
 if(Uart0RxCounter>=cn)
   break;
}
TIMSK&=0xFE;//�ض�ʱ��0�ж�
UCSR0B&=0x7f;//�ش��ڽ����ж�
}
}

 
   
unsigned char SampleADC(unsigned char ch)//��ʱԼ5ms
{     unsigned char c;
      unsigned long adcl;
	  unsigned int ADdata;

	  ADMUX  = (0xc0+ch);//Ƭ��2.56V��׼��ѡ�񵥶�����ͨ�� 
	  ADCSRA =0xC3;//ADCʹ�ܣ�ADC��ʼת����ADC�Զ�����ʹ�ܣ�����ת��ģʽ����8��Ƶ 
   
	  //��һ�β���ֵ��Ҫ
	  for(ADdata=0;ADdata<100;ADdata++);
	  while((ADCSRA&0x40)==0x40);//�ȴ�ת�����
	  ADdata=ADCL;
	  ADdata=ADCH;  
  
	  adcl=0;
	  for(c=0;c<32;c++)//32��ֵȡƽ��
	  {
	   ADCSRA = 0xC3;//ADCʹ�ܣ�ADC��ʼת����ADC�Զ�����ʹ�ܣ�����ת��ģʽ����8��Ƶ 
	   for(ADdata=0;ADdata<100;ADdata++);
	   while((ADCSRA&0x40)==0x40);//�ȴ�ת�����	
	   ADdata=ADCL;   
	   ADdata|=(ADCH<<8);
	   ADdata&=0x03ff;//10λ��Чת�����
	   adcl+=ADdata;
	  }

	  adcl>>=7;//����32���ٳ���4���൱��8λAD
	  return (adcl&0x000000ff);//ȡ8λ��Чλ
}

void Delay_ms(unsigned int t)//���ʱ8388ms
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
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־ 
 	TCCR1B = 0x05; //1024��Ƶ
    while((TIFR&0x04)==0x00) ; 
    TCCR1B = 0x00; //stop
}


void Start(void)
{
 InitialIO();
 uart0_init();
 uart1_init();

 IntFlag=0;//�޽���֡�ж�
 DataFlag=0;//�ޱ궨ϵ���·� 
 coeff_len=0;
 coeff_count=0;//�궨ϵ������
 

 
	EICRA|=0x03; //INT0�����ش��� ��ÿ2λ����һ���жϣ���4���жϣ�0-�͵�ƽ������1-������2-�½��ش�����3-�����ش��� 
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


   
	//�ɼ����ߵ�ѹ���ж��Ƿ����洢״̬  
	if(SampleADC(0)<30)//Cable�˵ĵ�ѹ����30V 
	{	  
		UCSR0B = 0x00;
		PORTE&=0xFD;
		DDRE|=0x02;
		PORTE&=0xFD;	 
		while(1); 
	}
	     
	while(1)//��Cable��Ϊ�ߵ�ѹ�������������ͨѶ״̬
	{ 
		 

		if(IntFlag==2)//У��λ���󣬲��ϴ��κ���Ӧ����λ������ʱ����
		{ 
			IntFlag=0;//������֡
			EIFR|=0x01;//��INT0�жϱ�־
			EIMSK|=0x01;//ʹ��INT0�ж�	 
		}
		else if(IntFlag==1)//���յ�����������֡
		{
			if(DataFlag)
			{ 		  
				coeffdata[coeff_count]=R_dat;
				coeff_count++;	  

				if(coeff_count==coeff_len)
				{
					if(DataFlag==1)//�·�ѹ���¶�ϵ��
					{
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x88;
						Uart0TxData[3]=0;//ѹ���¶�ϵ����0
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x88))
						{T_dat&=0xff00; }
						else
						{T_dat=0x00aa; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x88;
						Uart0TxData[3]=1;//ѹ���¶�ϵ����1
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+64];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x88))
						{T_dat&=0x00ff; }
						else
						{T_dat|=0x5500; }

						DataFlag=0;
						coeff_len=0;	
						T_com=0x0A;
						SendManchester();		
					}
					else if(DataFlag==2)//�·�����ϵ��
					{
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x88;
						Uart0TxData[3]=2;//����ϵ����2
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x88))
						{T_dat&=0xff00; }
						else
						{T_dat=0x00aa; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x88;
						Uart0TxData[3]=3;//����ϵ����3
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+64];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x88))
						{T_dat&=0x00ff; }
						else
						{T_dat|=0x5500; }

						DataFlag=0;
						coeff_len=0;	
						T_com=0x0B;
						SendManchester();		
					}	
					else if(DataFlag==3)//��ȡ��������
					{
						DataFlag=0;
						coeff_len=0;	
						T_com=0x09;
						TestDataBlockIndex=coeffdata[1];
						TestDataBlockIndex<<=8;
						TestDataBlockIndex|=coeffdata[0];//�׿��ַ/����
						TestDataBlockCount=coeffdata[3];
						TestDataBlockCount<<=8;
						TestDataBlockCount|=coeffdata[2];//����ȡ���ܰ�����ÿ��64�ֽڣ�ÿ��4096�ֽڹ�64��

						for(k=0;k<TestDataBlockCount;k++)//���û�Ҫ��ְ������ϴ���ÿ��64�ֽڴ�32֡
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x9A;//����������
							Uart0TxData[3]=8;

							//ͳһ������
							Uart0TxData[4]=(k); 	
							Uart0TxData[5]=(k>>8); 	//������	
							Uart0TxData[6]=(TestDataBlockIndex);
							Uart0TxData[7]=(TestDataBlockIndex>>8);//�׿�����

							/*
							// ����������
							Uart0TxData[4]=(k%8); 	
							Uart0TxData[5]=0; 	//������	
							Uart0TxData[6]=(TestDataBlockIndex+k/8);
							Uart0TxData[7]=((TestDataBlockIndex+k/8)>>8);//�׿�����
							*/

							SendUart0_2(9,6);//��ʱ48ms
							for(bf=0;bf<32;bf++)
							{	   	
								T_dat=Uart0RxData[bf*2+1];//���ֽ�
								T_dat<<=8;
								T_dat|=Uart0RxData[bf*2+0];//���ֽ�
								SendManchester();	
								Delay_ms(10);	 
							}		
						}
					} 
					else if(DataFlag==4)//�·�������Ϣ
					{
						T_dat=0x0000;

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=0;//��0
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0001; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=1;//��1
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+64];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0002; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=2;//��2
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+128];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0004; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=3;//��3
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+192];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0008; }

						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x84;
						Uart0TxData[3]=4;//��4
						Uart0TxData[4]=0x21;
						Uart0TxData[5]=0x22;
						Uart0TxData[6]=0x23;
						Uart0TxData[7]=0x24;
						Uart0TxData[8]=0x32;	 
						for(k=0;k<64;k++)
						{
							Uart0TxData[9+k]=coeffdata[k+256];  
						}		 
						SendUart0_2(74,20);//��ʱ160ms
						if(!((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x84)))
						{T_dat|=0x0010; }

						DataFlag=0;
						coeff_len=0;	
						T_com=0x0C;
						SendManchester();		
					}     
				}	 
			}

			else//�������·��궨ϵ������������¸�������������
			{ 
				T_com=(R_dat>>4);
				switch(R_dat&0xf0)
				{ 
					case 0x10://��λ 	
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x8D;//���������֣���ʱ40ms
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

					case 0x20://���ذ���ɼ���汾��
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x8D;//���������֣���ʱ40ms
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

					case 0x30://Cable��ѹ,�����ѹ
						Delay_ms(30);   
						f=SampleADC(0);//Cable 
						f=f*2.56;
						f=f*101;
						f=f/256;
						T_dat=f;
						T_dat<<=8;  
						
						SendUart1(0x30,4);//�������ѹ��32ms��ʱ
						T_dat|=Uart1RxData[0];

						SendManchester();   
						break;

					case 0x40: //�ɼ���ѹ�����¶�
					    if(R_dat==0x4f)// �ɼ���2��ѹ���¶ȵ����������ٶ෢2֡��4�ֽ�
						{
						  Delay_ms(30);//�������ȶ��������Ӱ�쵽�ڲ����ߣ����һ���ֽ��ղ�����
						  
						  SampleTPS(R_dat);//���5ms���أ�û�й�ѹ���̽�ʱ���20ms����
						  
						  Delay_ms(30);	
						  T_dat=TPS_PData;
						  SendManchester();//ѹ��2
						  
						  Delay_ms(30);	
						  T_dat=TPS_TData;
						  SendManchester();//�¶�2
						}
						else
						{
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x89;
						Uart0TxData[3]=9;
						SendUart0_2(9,50);//������ֱ����⣬��ʱ400ms

						T_dat=Uart0RxData[1];
						T_dat<<=8;
						T_dat|=Uart0RxData[0];
						SendManchester();//ѹ��

						Delay_ms(30);	
						T_dat=Uart0RxData[3];
						T_dat<<=8;
						T_dat|=Uart0RxData[2];
						SendManchester();//�¶�
						}
						
						break;

					case 0x50://��������	    	          	  	   	  	 
						/*	
						Delay_ms(40); //40ms��ʱ		
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
						SendUart0_2(9,50);//������ֱ����⣬��ʱ400ms

						lfib=Uart0RxData[5];
						lfib<<=8;
						lfib|=Uart0RxData[4];
						lfib&=0x0000ffff;
						myFIB.f=(unsigned long)lfib; 

						T_dat=myFIB.i[1];
						SendManchester();//����-ʱ��
						crc16array[0]=(unsigned char)(T_dat);
						crc16array[1]=(unsigned char)(T_dat>>8);

						Delay_ms(30);	
						T_dat=myFIB.i[0];
						SendManchester();//����-ʱ��
						crc16array[2]=(unsigned char)(T_dat);
						crc16array[3]=(unsigned char)(T_dat>>8);

						Delay_ms(30);	

						lfib=Uart0RxData[7];
						lfib<<=8;
						lfib|=Uart0RxData[6];
						lfib&=0x0000ffff;
						myFIB.f=(unsigned long)lfib; 

						T_dat=myFIB.i[1];
						SendManchester();//����-��λ1
						crc16array[4]=(unsigned char)(T_dat);
						crc16array[5]=(unsigned char)(T_dat>>8);


						Delay_ms(30);	
						T_dat=myFIB.i[0];
						SendManchester();//����-��λ2
						crc16array[6]=(unsigned char)(T_dat);
						crc16array[7]=(unsigned char)(T_dat>>8);

						Delay_ms(30);	
						crc16(crc16array,8);//0.75ms
						T_dat=crc16hi;
						T_dat<<=8;
						T_dat|=crc16lo;	  
						SendManchester();//�ϴ�CRCУ�鹲2�ֽ�
						break;
					
					//�շŵ�� 
					case 0x60:  
						if( !(((R_dat)==0x61)  || ((R_dat)==0x62) ) )//�Ȳ�����ת��Ҳ���Ƿ�ת�������������Ҳ���Ӧ��λ��
							break;

					    SendUart1(R_dat,200);//1600ms��ʱ
						T_dat=Uart1RxData[1];
						T_dat<<=8;
						T_dat|=Uart1RxData[0];//���ص��״̬
						SendManchester();	 

						break;

					case 0x70://���ڵ��   
						if( ((R_dat)<0x71)  || ((R_dat)>0x7c)  )//�Ȳ��ǵ���Ҳ���ǵ�С��Ҳ��΢����Ҳ����΢��С�������������Ҳ���Ӧ��λ��
						break;

				        SendUart1(R_dat,200);//1600ms��ʱ
						T_dat=Uart1RxData[1];
						T_dat<<=8;
						T_dat|=Uart1RxData[0];
						SendManchester();	 
						break;

					case 0x80://������
						if(R_dat==0x80)//ֻ�������Դ���������
						{        	  
							SendUart1(0x80,4);//32ms��ʱ
						    T_dat=Uart1RxData[0];
							SendManchester();
						}	  
						else if(R_dat==0x81)//��ȡ���״̬	
						{ 
						    SendUart1(0x81,4);//32ms��ʱ
						
							//�ȴ����շŵ��
							T_dat=Uart1RxData[1];
						    T_dat<<=8;
						    T_dat|=Uart1RxData[0];
							SendManchester();

							//�ٴ������ڵ�� 
							Delay_ms(30);//�ӳ�10ms  
							T_dat=Uart1RxData[3];
						    T_dat<<=8;
						    T_dat|=Uart1RxData[2];
							SendManchester();
						}	  
						else if(R_dat==0x82)//���ֹͣ��ͬʱֹͣ�������
						{	 
							SendUart1(0x82,30);//240ms��ʱ
						    T_dat=Uart1RxData[0];
							SendManchester(); 
						}
						else if(R_dat==0x83)//��ȡ�����ת������λ�Ͷ�·��λ
						{
							SendUart1(0x83,4);//32ms��ʱ
							T_dat=Uart1RxData[1];
						    T_dat<<=8;
						    T_dat|=Uart1RxData[0];
							SendManchester();
						}	
						else if(R_dat>0x83)//��ʣ12�����������õ����ת����, �շŵ���͵��ڵ����6��
						{
							SendUart1(R_dat,4);//32ms��ʱ
						    T_dat=Uart1RxData[0];
							SendManchester();
						}	  
						break;



					case 0x90: //�궨����
						if(R_dat==0x90)//��ȡ������Ϣ
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0xc8;
							Uart0TxData[3]=0;//������Ϣ��0
							SendUart0_2(9,4);// ��ʱ32ms	 
							for(i=0;i<32;i++)
							{
								Delay_ms(20);
								T_dat=Uart0RxData[i*2+1];//���ֽ�
								T_dat<<=8;
								T_dat|=Uart0RxData[i*2+0];//���ֽ�
								SendManchester();		 
							}		

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0xc8;
							Uart0TxData[3]=1;//������Ϣ��1
							SendUart0_2(9,4);// ��ʱ32ms	 
							for(i=0;i<32;i++)
							{
								T_dat=Uart0RxData[i*2+1];//���ֽ�
								T_dat<<=8;
								T_dat|=Uart0RxData[i*2+0];//���ֽ�
								SendManchester();
								Delay_ms(20);
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0xc8;
							Uart0TxData[3]=2;//������Ϣ��2
							SendUart0_2(9,4);// ��ʱ32ms	 
							for(i=0;i<32;i++)
							{
								T_dat=Uart0RxData[i*2+1];//���ֽ�
								T_dat<<=8;
								T_dat|=Uart0RxData[i*2+0];//���ֽ�
								SendManchester();
								Delay_ms(20);
							}		

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0xc8;
							Uart0TxData[3]=3;//������Ϣ��3
							SendUart0_2(9,4);// ��ʱ32ms	 
							for(i=0;i<32;i++)
							{
								T_dat=Uart0RxData[i*2+1];//���ֽ�
								T_dat<<=8;
								T_dat|=Uart0RxData[i*2+0];//���ֽ�
								SendManchester();
								Delay_ms(20);
							}			
						}
						else if(R_dat==0x91)//�ϴ�ѹ���궨����,20ms/֡ 	
						{
							//�Ȼ�ȡ�ܱ궨����    
							DataFlag=3;
							coeff_len=4;//��λ�����贫4�ֽڣ��ֱ�Ϊ�׿��ַ�Ͱ���
							coeff_count=0;
							TimeOutFlag=0;
							Timer0Counter=200;//1.6s��ʱ
							TCNT0=0x06; //8ms
							TCCR0|=0x06;//256��Ƶ
							TIFR|=0x01; //�嶨ʱ��0�жϱ�־
							TIMSK|=0x01;//ʹ�ܶ�ʱ��0�ж�		
						}
						else if(R_dat==0x92)//��ձ궨����,������ʽ��  
						{ 	
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x8c;
							Uart0TxData[3]=8;
							Uart0TxData[4]=0;
							Uart0TxData[5]=0;
							Uart0TxData[6]=0xff;
							Uart0TxData[7]=0xff;
							SendUart0_3(9,5000,9);// ���ʱ40��
							if((Uart0RxCounter==9)&&(Uart0RxData[0]==0x55)&&(Uart0RxData[1]==0x40)&&(Uart0RxData[2]==0x8c))
								{T_dat=0x0000;}
							else
								{T_dat=0xffff;}
							SendManchester();	 
						}
						else if(R_dat==0x93)//�ϴ�ȫ��ϵ������256�ֽڷ�128֡�ϴ�
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=0;//ϵ����0
							SendUart0_2(9,4);// ��ʱ32ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=1;//ϵ����1
							SendUart0_2(9,4);// ��ʱ32ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();
									Delay_ms(20);
								}
							}		

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=2;//ϵ����2
							SendUart0_2(9,4);// ��ʱ32ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=3;//ϵ����3
							SendUart0_2(9,4);// ��ʱ32ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();
									Delay_ms(20);
								}
							}		
						}
						else if(R_dat>=0x9a)//���õ����·���� 
						{
							SendUart1(R_dat,4);//32ms��ʱ
						    T_dat=Uart1RxData[0];
							SendManchester();
						}
						 
						break;

					case 0xa0://ѹ���궨ϵ��
						if(R_dat==0xa0)//�·�ȫ���궨ϵ������128�ֽ�,20ms/֡����2560ms   
							{ 
							DataFlag=1;
							coeff_len=128;
							coeff_count=0;
							TimeOutFlag=0;
							Timer0Counter=1000;//8s��ʱ
							TCNT0=0x06; //8ms
							TCCR0|=0x06;//256��Ƶ
							TIFR|=0x01; //�嶨ʱ��0�жϱ�־
							TIMSK|=0x01;//ʹ�ܶ�ʱ��0�ж�				    
						}
						else if(R_dat==0xa1)//�ϴ��궨ϵ��,28ms/֡ ����64֡128�ֽ�
						{    
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=0;//ϵ����0
							SendUart0_2(9,5);// ��ʱ40ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=1;//ϵ����1
							SendUart0_2(9,5);// ��ʱ40ms		 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();
									Delay_ms(20);
								}
							}		
						}
						break;


					case 0xB0://�������
						if(R_dat==0xb0) //�·�����ϵ��
						{
							DataFlag=2;
							coeff_len=128;
							coeff_count=0;
							TimeOutFlag=0;
							Timer0Counter=1000;//8s��ʱ
							TCNT0=0x06; //8ms
							TCCR0|=0x06;//256��Ƶ
							TIFR|=0x01; //�嶨ʱ��0�жϱ�־
							TIMSK|=0x01;//ʹ�ܶ�ʱ��0�ж�		
						}
						else  if(R_dat==0xb1)//��������ϵ��
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=2;//ϵ����2
							SendUart0_2(9,5);// ��ʱ40ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x87;
							Uart0TxData[3]=3;//ϵ����3
							SendUart0_2(9,5);// ��ʱ40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();
									Delay_ms(20);
								}
							}		
						}
						/*   if(R_dat==0xb1)//����ֵ
						{
						SendUart0(0xb0,5);//40ms��ʱ
						for(k=0;k<8;k+=2)
						{
						T_dat=Uart0RxData[k];
						T_dat<<=8;
						T_dat|=Uart0RxData[k+1];
						SendManchester();
						Delay_ms(40);
						}
						}
						else  if((R_dat>0xb0) && (R_dat<0xb5) )//1800����ȡ,������ȡ�����������ݣ�4��ͨ���ֱ���ȡ������Ϊ0xB1-0xB4
						{   
						SendUart0(R_dat,125);//1s��ʱ	 
						for(k=0;k<1800;k++)//��1800֡���ݣ�ȫ����ȡ
						{
						SendUart0(0xFF,1);//8ms��ʱ	   
						T_dat=Uart0RxData[0];
						T_dat<<=8;
						T_dat|=Uart0RxData[1];
						SendManchester();
						Delay_ms(42);
						}
						} */  
						break;

					case 0xc0://���в���ȫ�ɣ��ϴ�16�ֽڹ�8֡
						if( (R_dat==0xc0) || (R_dat==0xcf) )
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x89;
							Uart0TxData[3]=9;
							SendUart0_2(9,50);//������ֱ����⣬��ʱ400ms
							//if(Uart0RxCounter==64)//�ɹ����յ�ѹ�����¶ȡ��������ݣ��ȵ��ֽں���ֽ�
							{
								T_dat=Uart0RxData[1];
								T_dat<<=8;
								T_dat|=Uart0RxData[0];
								SendManchester();//ѹ��
								crc16array[0]=(unsigned char)(T_dat);
								crc16array[1]=(unsigned char)(T_dat>>8);


								Delay_ms(30);	
								T_dat=Uart0RxData[3];
								T_dat<<=8;
								T_dat|=Uart0RxData[2];
								SendManchester();//�¶�
								crc16array[2]=(unsigned char)(T_dat);
								crc16array[3]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	

								lfib=Uart0RxData[5];
								lfib<<=8;
								lfib|=Uart0RxData[4];
								lfib&=0x0000ffff;
								myFIB.f=(unsigned long)lfib; 

								T_dat=myFIB.i[1];
								SendManchester();//����-ʱ��1
								crc16array[4]=(unsigned char)(T_dat);
								crc16array[5]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	
								T_dat=myFIB.i[0];
								SendManchester();//����-ʱ��2
								crc16array[6]=(unsigned char)(T_dat);
								crc16array[7]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	

								lfib=Uart0RxData[7];
								lfib<<=8;
								lfib|=Uart0RxData[6];
								lfib&=0x0000ffff;
								myFIB.f=(unsigned long)lfib; 

								T_dat=myFIB.i[1];
								SendManchester();//����-��λ1
								crc16array[8]=(unsigned char)(T_dat);
								crc16array[9]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	
								T_dat=myFIB.i[0];
								SendManchester();//����-��λ2
								crc16array[10]=(unsigned char)(T_dat);
								crc16array[11]=(unsigned char)(T_dat>>8);

								 
								SendUart1(0x81,4);//���״̬��32ms��ʱ
								
							    T_dat=Uart1RxData[1];
						        T_dat<<=8;
						        T_dat|=Uart1RxData[0];						 
								SendManchester();//�շŵ��״̬
								crc16array[12]=(unsigned char)(T_dat);
								crc16array[13]=(unsigned char)(T_dat>>8);

								Delay_ms(30);	 
								T_dat=Uart1RxData[3];
						        T_dat<<=8;
						        T_dat|=Uart1RxData[2];
								SendManchester();//���ڵ��״̬
								crc16array[14]=(unsigned char)(T_dat);
								crc16array[15]=(unsigned char)(T_dat>>8);

								Delay_ms(1);	  					
						        SendUart1(0x31,4);//�������ѹ��32ms��ʱ
						        T_dat=Uart1RxData[0];
								SendManchester();
								crc16array[16]=(unsigned char)(T_dat);
								crc16array[17]=(unsigned char)(T_dat>>8);

								if(R_dat==0xcf)// �ɼ���2��ѹ���¶ȵĹ��������ٶ෢2֡��4�ֽ�
								{
						  		 SampleTPS(R_dat);//���������10ms���أ�����ѹ���̽ڵĻ�20*4=80ms��ʱ�󷵻�
						  
						  		 Delay_ms(25);	
						  		 T_dat=TPS_PData;
						  		 SendManchester();//ѹ��
								 crc16array[18]=(unsigned char)(T_dat);
								 crc16array[19]=(unsigned char)(T_dat>>8);
						  
						  		 Delay_ms(30);	
						  		 T_dat=TPS_TData;
						  		 SendManchester();//�¶�
								 crc16array[20]=(unsigned char)(T_dat);
								 crc16array[21]=(unsigned char)(T_dat>>8);
								 
								 Delay_ms(30);	
								 crc16(crc16array,22);//0.75ms
								 T_dat=crc16hi;
								 T_dat<<=8;
								 T_dat|=crc16lo;	  
								 SendManchester();//�ϴ�CRCУ�鹲2�ֽڣ����յ�����ϴ���Ϲ���ʱԼ850ms
								}
								else
								{
								 Delay_ms(30);	
								 crc16(crc16array,18);//0.75ms
								 T_dat=crc16hi;
								 T_dat<<=8;
								 T_dat|=crc16lo;	  
								 SendManchester();//�ϴ�CRCУ�鹲2�ֽڣ����յ�����ϴ���Ϲ���ʱԼ760ms
								}
							}	
						}
						else  if(R_dat==0xc5) //��������Ϣ,��320�ֽڣ���5�δ�����������ȡ��ÿ��64�ֽ�
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=0;//��0
							SendUart0_2(9,5);// ��ʱ40ms	
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									Delay_ms(20);
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();		 
								}	
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=1;//��1
							SendUart0_2(9,5);// ��ʱ40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();
									Delay_ms(20);
								}
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=2;//��2
							SendUart0_2(9,5);// ��ʱ40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();
									Delay_ms(20);
								}
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=3;//��3
							SendUart0_2(9,5);// ��ʱ40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();
									Delay_ms(20);
								}
							}	

							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x83;
							Uart0TxData[3]=4;//��4
							SendUart0_2(9,5);// ��ʱ40ms	 
							if(Uart0RxCounter==64) 
							{
								for(i=0;i<32;i++)
								{
									T_dat=Uart0RxData[i*2+1];//���ֽ�
									T_dat<<=8;
									T_dat|=Uart0RxData[i*2+0];//���ֽ�
									SendManchester();
									Delay_ms(20);
								}
							}	
						}
						else  if(R_dat==0xca) //д������Ϣ
						{
							DataFlag=4;
							coeff_len=320;
							coeff_count=0;
							TimeOutFlag=0;
							Timer0Counter=2000;//16s��ʱ
							TCNT0=0x06; //8ms
							TCCR0|=0x06;//256��Ƶ
							TIFR|=0x01; //�嶨ʱ��0�жϱ�־
							TIMSK|=0x01;//ʹ�ܶ�ʱ��0�ж�		
						}
						break;
					
					case 0xd0://ѹ����������汾
					   if(R_dat==0xdf)//��������2·ѹ����汾��	
					    {			 
						  SendUart1(0xdf,3);//�����
						  T_dat=Uart1RxData[0];
						  T_dat<<=8;
						  
						  SendTPS(0xdf);//��2·ѹ����
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
			EIFR|=0x01;//��INT0�жϱ�־
			EIMSK|=0x01;//ʹ��INT0�ж� 
		}  
	}
}

 

void SendManchester(void) //���뷢�ͳ���,�ܺ�ʱ���Լ120+560+8400+720=9800us����СԼ120+560+5040+720=6440us
{ //ÿλ80us
 //ͬ����1��6��
 //1:    1��2��
 //0��   1��4��  
  unsigned char L;
  SREG&=0x7f;//�����ж� 
  
    M1_H //�ȸ����߷ŵ磬��ʱԼ120us
	
  //ģ��ͬ��ͷ����
	//4λ����
	T_dat_buf[21]=(T_com&0x08)>>3;
	T_dat_buf[20]=(T_com&0x04)>>2;
	T_dat_buf[19]=(T_com&0x02)>>1;
	T_dat_buf[18]=T_com&0x01;
	//16λ����
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
	//����У��λ����У��
	T_dat_buf[1]=1;
  	for(L=2;L<22;L++)
		{T_dat_buf[1]^=T_dat_buf[L];}
    DELAY80
    M1_L
	DELAY40
   
   
	//ͬ��ͷ,560us
	//��翪ʼ����80us	 
	M2_H 	
	DELAY80
	M2_L   //������	
	//�ŵ翪ʼ����80*6=480us
	DELAY40
	M1_H    
    DELAY80
	DELAY80
	DELAY80
	DELAY80
	DELAY80
	M1_L    
	DELAY40  //�ŵ����
	
     //�������� ,ȫ0ʱÿλ400us����8400us��ȫ1ʱÿλ240us����5040us
 	for(L=21;L>0;L--)
	{   
	  if(T_dat_buf[L])//1��һ�߶���
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
       else 		//0��һ���ĵ� 				                
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
		   //����λ��һ�߰˵ͣ�720us
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
	
 	EIFR|=0x08;//��INT3�жϱ�־ 
 	SREG|=0x80; //�����ж�
}



void EEPROM_write(unsigned int Address,unsigned char Data)  	//�ڲ�EEPROMд
{//ʱ��Ƶ��Ϊ1MHz�����͵�EEPROM�ֽ�д��ʱԼ8.5ms
    while(EECR&0x02);                   // �ȴ���һ��д��������
    EEAR=Address;
    EEDR=Data;                        	// ���õ�ַ�����ݼĴ���
    EECR|=0x04;                         //��λEEMWE
    EECR|=0x02;                         //��λEEWE ������д����
}

unsigned char EEPROM_read(unsigned int Address)  				//�ڲ�EEPROM��
{
    while(EECR&0x02);      				//�ȴ���һ�β�������
    EEAR = Address;                     //���õ�ַ�Ĵ���						
	EECR|=0x01;             			//����EERE ������������
    return EEDR;                   		//�����ݼĴ�����������
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