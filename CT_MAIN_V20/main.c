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
//20170622  Ϊ��Ӧ�¸�ѹ��������ĵ�·����Ľ�һЩ���ܣ��汾���˵�V1.0
**************************************************/
#include <iom128v.h>								   	
#include <macros.h>
#include <stdio.h>

 
#define Debug 1
#define  BB     0x10        //�̼��汾��


#define M1_L    PORTD&=0xef;
#define M1_H    PORTD|=0x10;
#define M2_L    PORTD&=0xdf;
#define M2_H    PORTD|=0x20;

 
//�շŵ���ܽŶ���Ϳ�����
#define DIR1_PIN    ((PINA&0x04)>>2)
#define PWM1_PIN    ((PINA&0x02)>>1)
#define BREAKE1_PIN (PINA&0x01)

#define DIR1_H      {DDRA|=0x04;PORTA|=0x04;}
#define PWM1_H      {DDRA|=0x02;PORTA|=0x02;}
#define BREAKE1_H   {DDRA|=0x01;PORTA|=0x01;}

#define DIR1_L      {DDRA|=0x04;PORTA&=0xfb;}
#define PWM1_L      {DDRA|=0x02;PORTA&=0xFD;}
#define BREAKE1_L   {DDRA|=0x01;PORTA&=0xFE;}

//���ڵ���ܽŶ���Ϳ�����
#define PWM2_PIN    ((PINF&0x04)>>2)
#define BREAKE2_PIN ((PINF&0x02)>>1)
#define DIR2_PIN    (PINF&0x01)

#define PWM2_H      {DDRF|=0x04;PORTF|=0x04;}
#define BREAKE2_H   {DDRF|=0x02;PORTF|=0x02;}
#define DIR2_H      {DDRF|=0x01;PORTF|=0x01;}

#define PWM2_L      {DDRF|=0x04;PORTF&=0xfb;}
#define BREAKE2_L   {DDRF|=0x02;PORTF&=0xFD;}
#define DIR2_L      {DDRF|=0x01;PORTF&=0xFE;}

//�շŵ��������ת
#define Motor1Run_P ((PWM1_PIN==0) && (DIR1_PIN==1))
//�շŵ�����ڷ�ת
#define Motor1Run_N ((PWM1_PIN==1) && (DIR1_PIN==0))  
//���ڵ��������ת 
#define Motor2Run_P ((PWM2_PIN==0) && (DIR2_PIN==1))
//���ڵ�����ڷ�ת
#define Motor2Run_N ((PWM2_PIN==1) && (DIR2_PIN==0)) 

#define PIN_MANINT    (PIND & 0x01)

#define MANINT_CLR	{ EIFR|=0x01; }
#define MANINT_EN		{ EIMSK |= 0x01; }
#define MANINT_DIS	{ EIMSK &= 0xFE; }

#define INT_EN			{ SEI(); }
#define INT_DIS		{ CLI(); }


//�����Դ
#define VM_H        {BREAKE1_H}{BREAKE2_H}{DDRB|=0x80;PORTB|=0x80;}

//ͬʱ�ض������
#define MOTORSTOP   {BREAKE1_H}{BREAKE2_H}{DDRB|=0x80;PORTB&=0x7f;}{DIR1_L}{PWM1_L}{DIR2_L}{PWM2_L}

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

union FIB
{
 float f;
 unsigned int i[2];
 unsigned char b[4];
}myFIB;

union MotorStatus
{
 unsigned int i;
 struct 
 { 
   unsigned int RUN:1;//bit0,�������״̬��0��ֹͣ��1������
   unsigned int DIR:1;//bit1,������з���0����ת,�ձ�-����(DIR_L)��1����ת,�ű�-��С(DIR_H)
   unsigned int BrokenStop:1;//bit2, ��·ͣ
   unsigned int CommandStop:1;//bit3,�ֶ�ͣ
   unsigned int OverCurrent:1;//bit4,����ͣ
   unsigned int UnderVoltage:1;//bit5,Ƿѹͣ
   unsigned int StartResult:2;//bit6-bit7,������һ���������(0�������ϵ�������1�������ͬ�����У�2��������������У�3����һ�������������)
   
   unsigned int ThisCurrent:8;//bit8-bit15 ,  �����ǰ��ֹͣǰ���һ�ε���
 }s;
}Motor1Status,Motor2Status;//���״̬

unsigned char Motor1Gear,Motor2Gear;// ���������λ
unsigned char Motor1Thr,Motor2Thr;//�����ת������ֵ
unsigned char MotorIdelGear;//�����·��λ
unsigned char MotorIdelCur;// �����·����
 
unsigned char TPSCn,TPSBuf[4];

unsigned char T_Flag;

unsigned int Timer0Counter;//��ʱ��0����4msʱ�ļ���

unsigned char Uart0RxData[74];//���ڽ������ݻ���
unsigned char Uart0RxCounter;

unsigned char Rx[18];//�������9λ��18����λ
int  T_dat,R_dat;//���뷢������
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
unsigned char crc16hi,crc16lo,crc16array[18];

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
void CheckMotor(unsigned int t);
void crc16(unsigned char r_data[],unsigned int length);
void Gear2Thr(void);
void int0_isr(void);

#pragma interrupt_handler int0_isr:iv_INT0
void int0_isr(void)//�ⲿ�ж�0
{ 
   //SREG&=0x7f;//�����ж�
   //EIMSK&=0xFe;//��INT0  
   TCCR2 = 0x00; //stop
   
	INT_DIS
	MANINT_DIS
   
   #if Debug 
  // PORTD&=0xf7;
   //PORTD|=0x08;
  // PORTD&=0xf7;
   #endif
  
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
   
   #if Debug 
   //PORTD&=0xf7;
   #endif  		   
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
#if Debug 
 //  PORTD|=0x08;
   #endif
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
   #if Debug 
   //PORTD&=0xf7;
   #endif  
   goto End1;//ֻҪ���յ�������ͬ��ͷ������У��λ�Ƕ��Ǵ�������ʱ������INT0�ж�
     
End:
 	 	//EIFR|=0x01;//��INT0�жϱ�־ 
 		//EICRA|=0x0c; //INT0�����ش��� 
 		//EIMSK|=0x01;//ʹ��INT0�ж� 
	
		MANINT_CLR
		MANINT_EN
End1:
   TCCR2 = 0x00; //stop
    //SREG|=0x80;//���ж�ʹ��;  
   INT_EN
}

void InitialIO(void)
{//1�����0����
 PORTA=0x00;
 DDRA=0x07; //PA0=BREAKE1 PA1=PWM1 PA2=DIR1

 PORTB=0x00; 
 DDRB=0x80; //PB7=VM_CTRL
 
 PORTC=0x00;
 DDRC=0x00; 
 
 PORTD=0x00; 
 DDRD=0x38;//PD4=M2 PD5=M1 TX1=0
 
 PORTE=0x00; 
 DDRE=0x00;
 
 PORTF=0x00; 
 DDRF=0x07;//PF0=DIR2 PF1=BREAKE2 PF2=PWM2
 
 PORTG=0x00; 
 DDRG=0x00; 
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
 UCSR1B = 0x90;//ʹ�ܽ��պͽ����жϣ���ʹ�ܷ���
}

#pragma interrupt_handler uart1_rx_isr:iv_USART1_RXC
void uart1_rx_isr(void)//����1�����ж�
{
 if(TPSCn<4)
 {
  TPSBuf[TPSCn]=UDR1;
  TPSCn++;
 }
 else
 {
  UCSR1B&=0x7f;//�ش���1�����ж�
 }
}

void SendTPS(unsigned char b)
{
 unsigned char t,i;
 unsigned char PD[10];
 
 TPSCn=0;
 
 //while(!(UCSR1A&(1<<UDRE1)));   // �ȴ����ͻ�����Ϊ��
 //UDR1=b; 
 
 DDRD|=0x08;//TX1��ΪIO���
 t=PIND;
 for(i=0;i<8;i++)
 {
  if(((b>>i)&0x01)==0x00)//���෢��
  {
   PD[i+1]=(t|0x08);
  }
  else
  {
   PD[i+1]=(t&0xf7);
  }
 }
 PD[0]=1;//��ʼλ
 PD[9]=0;//����λ
 PORTD&=0xf7;// ������
 asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
 for(i=0;i<10;i++)//��ģ�ⷢ��10λ
 {
 PORTD=PD[i];
 asm("nop");
 }
 PORTD&=0xf7;//��󱣳ֵ�
 
 
 t=UDR1;//�����ڽ����ж�ǰ���ջ���
 UCSR1B|=0x80;//������1�����ж�
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

#pragma interrupt_handler timer3_ovf_isr:iv_TIM3_OVF
void timer3_ovf_isr(void)
{
 	 ETIMSK = 0x00; //Timer3�жϹر�
	 TCCR3B = 0x00; //stop
		
  		   if(PWM1_PIN==1)//�շŵ����������
   		   {
 		   Motor1Status.s.CommandStop=1;
   		   }
 		   if(PWM2_PIN==1)//���ڵ����������
   		   { 
 		   Motor2Status.s.CommandStop=1;
   		   }
 	 	   MOTORSTOP  //�ص��
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
while(TimeOutFlag==0)
{
CheckMotor(0);
}
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
CheckMotor(0);
 if(Uart0RxCounter>=cn)
   break;
}
TIMSK&=0xFE;//�ض�ʱ��0�ж�
UCSR0B&=0x7f;//�ش��ڽ����ж�
}
}


void Pwm_startup(unsigned char motor)//���������
{	unsigned char c,b;

	if(motor==1)
	{  if(DIR1_PIN==0)
	   {  PWM1_L	   
		  BREAKE1_L 
		  DELAY100
	     for(b=0;b<30;b++)//6ms���ý���ռ�ձ��������
		 {
		  PWM1_H
		  DELAY100
		  PWM1_L
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//150ms���ý���ռ�ձ��������
		 {
	       PWM1_L
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor1Status.s.UnderVoltage==0)//��������������û�з���Ƿѹ�����������
		   {PWM1_H}
		   else 
		   {PWM1_L}
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	   else
	   {
	      PWM1_H	   
		  BREAKE1_L 
		  DELAY100
	     for(b=0;b<30;b++)//6ms���ý���ռ�ձ��������
		 {
		  PWM1_L
		  DELAY100
		  PWM1_H
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//150ms���ý���ռ�ձ��������
		 {
	       PWM1_H
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor1Status.s.UnderVoltage==0)//��������������û�з���Ƿѹ�����������
		   {PWM1_L}
		   else 
		   {PWM1_H}
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	} 
	else if(motor==2)
	{  
	  if(DIR2_PIN==0)
	   { 
	      PWM2_L	   
		  BREAKE2_L 
		  DELAY100
	     for(b=0;b<30;b++)//6ms���ý���ռ�ձ��������
		 {
		  PWM2_H
		  DELAY100
		  PWM2_L
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//150ms���ý���ռ�ձ��������
		 {
	       PWM2_L
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor2Status.s.UnderVoltage==0)//��������������û�з���Ƿѹ�����������
		   {PWM2_H}
		   else 
		   {PWM2_L}
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	   else
	   {
	      PWM2_H	   
		  BREAKE2_L 
		  DELAY100
	      for(b=0;b<30;b++)//6ms���ý���ռ�ձ��������
		 {
		  PWM2_L
		  DELAY100
		  PWM2_H
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//150ms���ý���ռ�ձ��������
		 {
	       PWM2_H
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor2Status.s.UnderVoltage==0)//��������������û�з���Ƿѹ�����������
		   {PWM2_L}
		   else 
		   {PWM2_H}
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
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

void CheckMotor(unsigned int t)//�����תʱ��ʱ5us�����ʱ8388ms
{   
    float f;
	unsigned char a;
    unsigned int n;
    TCCR1B = 0x00; //stop
	if(t>8388)
	   t=8388;
	f=t;
	f*=7.8125;
	n=f;
    TCNT1 = 65535-n;  
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־ 
 	TCCR1B = 0x05; //1024��Ƶ
	
    while((TIFR&0x04)==0x00)//��ʱ�жϵ���ǰһֱ����������
   {   
    if( (Motor1Run_P) || (Motor1Run_N) )//�շŵ������ʱ��ʵʱ����������������������һ�εĲ���ֵ
    {
	  a=SampleADC(7);
	  
      if(a>Motor1Thr)//�����ص��
      {
	   MOTORSTOP//�ص��   
       Motor1Status.s.OverCurrent=1;//����״̬��1����ʾ����ϴ�ֹͣԭ��Ϊ����ͣ
	   Motor1Status.s.RUN=0;//ֹͣ״̬
	  }
	  else if(a<MotorIdelCur)//��·�ص�� 
	  {   
	   MOTORSTOP//�ص��
       Motor1Status.s.BrokenStop=1;//��·״̬��1����ʾ����ϴ�ֹͣԭ��Ϊ��·ͣ
	   Motor1Status.s.RUN=0;//ֹͣ״̬
	  }
	
	 Motor1Status.s.RUN=1;//����״̬
	 
	 f=a;
	 f*=1.042;
	 Motor1Status.s.ThisCurrent=f;
    }
    else
	{
	 Motor1Status.s.RUN=0;//ֹͣ״̬
	}
	 Motor1Status.s.DIR=DIR1_PIN;
	 
	  	   
    if( (Motor2Run_P) || (Motor2Run_N) )//���ڵ������ʱ��ʵʱ����������������������һ�εĲ���ֵ
    {
	 a=SampleADC(6);	
	
     if(a>Motor2Thr)//�����ص��
     {
	   MOTORSTOP//�ص��
       Motor2Status.s.OverCurrent=1;//����״̬��1����ʾ����ϴ�ֹͣԭ��Ϊ����ͣ
	   Motor2Status.s.RUN=0;//ֹͣ״̬
	 } 
	 else if(a<MotorIdelCur) //��·�ص��������С��26mA��Ϊ�Ƕ�·
	 {
	   MOTORSTOP//�ص��
       Motor2Status.s.BrokenStop=1;//��·״̬��1����ʾ����ϴ�ֹͣԭ��Ϊ��·ͣ
	   Motor2Status.s.RUN=0;//ֹͣ״̬
	 }
	 
	 Motor2Status.s.RUN=1;//����״̬
	 
	 f=a;
	 f*=1.042;
	 Motor2Status.s.ThisCurrent=f;
    }
	else
	{
	 Motor2Status.s.RUN=0;//ֹͣ״̬
	}
     Motor2Status.s.DIR=DIR2_PIN;  
	
    if(t==0){break;}//����ʱ����һ�ε��״̬��ֱ������
   } 
   
   TCCR1B = 0x00; //stop
}


void Start(void)
{
 InitialIO();
 uart0_init();
 
 MOTORSTOP//�ص��
 BREAKE1_L//�ſ�ɲ��
 BREAKE2_L//�ſ�ɲ��
 
 Motor1Status.s.RUN=0; 
 Motor1Status.s.DIR=DIR1_PIN;
 Motor1Status.s.BrokenStop=0;
 Motor1Status.s.CommandStop=0;
 Motor1Status.s.OverCurrent=0;
 Motor1Status.s.UnderVoltage=0;
 Motor1Status.s.StartResult=0;
 Motor1Status.s.ThisCurrent=0;
 
 Motor2Status.s.RUN=0; 
 Motor2Status.s.DIR=DIR2_PIN;
 Motor2Status.s.BrokenStop=0;
 Motor2Status.s.CommandStop=0;
 Motor2Status.s.OverCurrent=0;
 Motor2Status.s.UnderVoltage=0;
 Motor2Status.s.StartResult=0;
 Motor2Status.s.ThisCurrent=0;
 
 IntFlag=0;//�޽���֡�ж�
 DataFlag=0;//�ޱ궨ϵ���·� 
 coeff_len=0;
 coeff_count=0;//�궨ϵ������
 
 ACSR|=0x10;//��ģ��Ƚ����жϱ�־
 ACSR&=0xf7;//��ģ��Ƚ����ж�
 ACSR|=0x40;//ACBG��λ��ģ��Ƚ����������ڲ���϶��׼1.23V
 ACSR|=0x03;//��ģ��Ƚ���,AIN1�����Ƚ���������0:�Ƚ�������仯���жϣ�1��������2���½����жϣ�3:�������ж�   
 ACSR|=0x08;//��ģ��Ƚ����жϣ��κ�ʱ��Ҫ����Ƿѹ���	
 ACSR|=0x10;//��ģ��Ƚ����жϱ�־	
	

//EIFR|=0x01;//��INT0�жϱ�־,ÿλ����һ���жϣ��ɿ���8���ж�
 //EICRA|=0x0c; //INT0�����ش��� ��ÿ2λ����һ���жϣ���4���жϣ�0-�͵�ƽ������1-������2-�½��ش�����3-�����ش���
 //EIMSK|=0x01;//ʹ��INT0�ж� ,ÿλ����һ���жϣ��ɿ���8���ж�
 
 //SREG=0x80;//���ж�ʹ��
 
	EICRA|=0x03; //INT0�����ش��� ��ÿ2λ����һ���жϣ���4���жϣ�0-�͵�ƽ������1-������2-�½��ش�����3-�����ش���
	 
	MANINT_CLR
	MANINT_EN
	
	INT_EN
 
 M1_H
 CheckMotor(50);
 M1_L
 
}

void Gear2Thr(void)
{
//�����������λ����Ϊ�������������жϺ����н��бȽ�����
  if(Motor1Gear<1) {Motor1Gear=1;}//��ת������СΪ1��
  if(Motor1Gear>6) {Motor1Gear=6;}//��ת�������Ϊ6��
  if(Motor2Gear<1) {Motor2Gear=6;}//��ת������СΪ1��
  if(Motor2Gear>6) {Motor2Gear=6;}//��ת�������Ϊ6��
  //��1����100mA�𲽣�ÿ����ֵ30mA,0.96����/1mA����6��Լ250mA
  Motor1Thr= 96+ (Motor1Gear-1)*29;
  Motor2Thr= 96+ (Motor2Gear-1)*29;
  
  if(MotorIdelGear<1){MotorIdelGear=1;}//�����·������С1��
  if(MotorIdelGear>6){MotorIdelGear=6;}//�����·�������6��
  //��5mA�𲽣����30mA
  MotorIdelCur=(MotorIdelGear-1)*5+5;
  
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

	CheckMotor(50); 

	Motor1Gear=EEPROM_read(0);
	Motor2Gear=EEPROM_read(1);
	MotorIdelCur=EEPROM_read(2);
	Gear2Thr();
  
	//�ɼ����ߵ�ѹ���ж��Ƿ����洢״̬  
	if(SampleADC(4)<30)//Cable�˵ĵ�ѹ����30V 
	{	  
		UCSR0B = 0x00;
		PORTE&=0xFD;
		DDRE|=0x02;
		PORTE&=0xFD;	 
		while(1); 
	}
	     
	while(1)//��Cable��Ϊ�ߵ�ѹ�������������ͨѶ״̬
	{ 
		CheckMotor(0);//ʵʱ���е���������

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
								CheckMotor(10);	 
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

					case 0x20://���汾��
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

					case 0x30://Cable��ѹ,VIN/VR��ѹ
						CheckMotor(30);   
						f=SampleADC(5);//Cable CAB-R1
						//f=f*2.56;
						//f=f*101;
						//f=f/256;
						T_dat=f;
						CheckMotor(30);
						T_dat<<=8;   
						f=SampleADC(4);//VIN / VR  CAB-R2
						//f=f*2.56;
						//f=f*101;
						//f=f/256;
						i=f;
						T_dat|=i; 
						CheckMotor(30);
						SendManchester();   
						break;

					case 0x40: //�ɼ���ѹ�����¶�
						/*
						CheckMotor(40); //40ms��ʱ		
						T_dat=1234;//û��������ʱ����һ�̶���ֵ��ʾѹ��
						SendManchester();
						CheckMotor(40); //40ms��ʱ		 
						T_dat=5678;//û��������ʱ����һ�̶���ֵ��ʾ�¶�
						SendManchester();
						*/
						Uart0TxData[0]=0xE8;
						Uart0TxData[1]=0x40;
						Uart0TxData[2]=0x89;
						Uart0TxData[3]=9;
						SendUart0_2(9,50);//������ֱ����⣬��ʱ400ms

						T_dat=Uart0RxData[1];
						T_dat<<=8;
						T_dat|=Uart0RxData[0];
						SendManchester();//ѹ��

						CheckMotor(30);	
						T_dat=Uart0RxData[3];
						T_dat<<=8;
						T_dat|=Uart0RxData[2];
						SendManchester();//�¶�
						break;

					case 0x50://��������	    	          	  	   	  	 
						/*	
						CheckMotor(40); //40ms��ʱ		
						myFIB.f=123.456; 
						T_dat=myFIB.i[1];
						SendManchester();
						CheckMotor(40);
						T_dat=myFIB.i[0];
						SendManchester();
						CheckMotor(40);
						myFIB.f=789.012; 
						T_dat=myFIB.i[1];
						SendManchester();
						CheckMotor(40);
						T_dat=myFIB.i[0];
						SendManchester();
						CheckMotor(40);
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

						CheckMotor(30);	
						T_dat=myFIB.i[0];
						SendManchester();//����-ʱ��
						crc16array[2]=(unsigned char)(T_dat);
						crc16array[3]=(unsigned char)(T_dat>>8);

						CheckMotor(30);	

						lfib=Uart0RxData[7];
						lfib<<=8;
						lfib|=Uart0RxData[6];
						lfib&=0x0000ffff;
						myFIB.f=(unsigned long)lfib; 

						T_dat=myFIB.i[1];
						SendManchester();//����-��λ1
						crc16array[4]=(unsigned char)(T_dat);
						crc16array[5]=(unsigned char)(T_dat>>8);


						CheckMotor(30);	
						T_dat=myFIB.i[0];
						SendManchester();//����-��λ2
						crc16array[6]=(unsigned char)(T_dat);
						crc16array[7]=(unsigned char)(T_dat>>8);

						CheckMotor(30);	
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

						if( (Motor2Run_P) || (Motor2Run_N) )//���ڵ����������
						{
							Motor1Status.s.StartResult=3;//��һ�����������
							CheckMotor(100);//�ܿ���Ӧ��λ��
						}
						else if(Motor1Run_P)//�շŵ��������ת
						{
						  if(R_dat==0x61)
						  {
						    Motor1Status.s.StartResult=1;//���������ͬ������
						  }
						  else if(R_dat==0x62)
						  {
						   Motor1Status.s.StartResult=2;//��������ڷ�������
						  }
						  CheckMotor(100); //�ܿ���Ӧ��λ��
						}
						else if(Motor1Run_N)//�շŵ�����ڷ�ת
						{
						  if(R_dat==0x61)
						  {
						   Motor1Status.s.StartResult=2;//��������ڷ������� 
						  }
						  else if(R_dat==0x62)
						  {
						   Motor1Status.s.StartResult=1;//���������ͬ������
						  }
						  CheckMotor(100); //�ܿ���Ӧ��λ��
						}
						else //�����������ֹͣ״̬��������Ҫ�����������Լ1.5�������λ��
						{  
							//ȫ�����״̬�ó�ֵ����һ��ֹͣԭ��ȫ�����
							Motor1Status.s.BrokenStop=0;
							Motor1Status.s.CommandStop=0;
							Motor1Status.s.OverCurrent=0;
							Motor1Status.s.UnderVoltage=0;
							Motor1Status.s.StartResult=0;//��������

							if((R_dat)==0x61)//�ſ�����
							{ 
								DIR1_H       //�űۣ���ת
							}
							else if((R_dat)==0x62)          
							{ 
								DIR1_L      //�ձۣ���ת
							}
							else
							{
								return;//20170609
							}

							VM_H//�������Դ

							CheckMotor(500);//�������Դ����ȴ������ô��ʶ����ܵ�ѹ�ȶ�

							//CompCounter=0; //��������������0
							//EIFR|=0x01;//��INT0�жϱ�־
							//EIMSK&=0xfe;//��INT0�жϣ��������ǰ�ڲ��ü��������

							TCCR1B = 0x00; //stop
							TCNT1 = 61629;   //��ʱ500ms
							TIFR|=0x04; //�嶨ʱ��1�жϱ�־
							TCCR1B = 0x05; //1024��Ƶ
							TIFR|=0x04; //�嶨ʱ��1�жϱ�־

							Pwm_startup(1);//PWM��������ʱԼ200ms,�������ǰ�ڲ����е�����飬����Ƿѹ���		

							while((TIFR&0x04)==0x00); //�ȴ���ʱ�����ڼ������Ƿѹ�жϷ���
							TCCR1B = 0x00; //stop

							//EIFR|=0x01;//��INT0�жϱ�־
							//EIMSK|=0x01;//��INT0�жϣ��������500ms�󼴿�ʼ���������

							CheckMotor(500);//��ʱ��֤���ߵ�ѹ�ȶ�		
						}
						T_dat=Motor1Status.i;
						SendManchester();	 

						break;

					case 0x70://���ڵ��   
						if( ((R_dat)<0x71)  || ((R_dat)>0x7c)  )//�Ȳ��ǵ���Ҳ���ǵ�С��Ҳ��΢����Ҳ����΢��С�������������Ҳ���Ӧ��λ��
						break;

						i=R_dat%2;
						
						if( (Motor1Run_P) || (Motor1Run_N) )//�շŵ����������
						{
							Motor2Status.s.StartResult=3;//��һ�����������
							CheckMotor(100);//�ܿ���Ӧ��λ��
						}
						else if(Motor2Run_P)//���ڵ��������ת
						{
						  if(i==0)//ż�������ת
						  {
						   Motor2Status.s.StartResult=1;//���������ͬ������
						  }
						  else//���������ת
						  {
						   Motor2Status.s.StartResult=2;//��������ڷ�������
						  }
						  CheckMotor(100); //�ܿ���Ӧ��λ��
						}
						else if(Motor2Run_N)//���ڵ�����ڷ�ת
						{
						  if(i==0)//ż�������ת
						  {
						   Motor2Status.s.StartResult=2;//��������ڷ�������
						  }
						  else//���������ת
						  {
						   Motor2Status.s.StartResult=1;//���������ͬ������
						  }
						  CheckMotor(100); //�ܿ���Ӧ��λ��
						}					
						else //�����������ֹͣ״̬��������Ҫ�����������Լ1.5�������λ��
						{  
							//ȫ�����״̬�ó�ֵ����һ��ֹͣԭ��ȫ�����
							Motor2Status.s.BrokenStop=0;
							Motor2Status.s.CommandStop=0;
							Motor2Status.s.OverCurrent=0;
							Motor2Status.s.UnderVoltage=0;
							Motor2Status.s.StartResult=0;//��������

							if( i==1 )//�����������
							{ 
								DIR2_L      //��ת
							}
							else if( i==0 )    //ż�������С
							{ 
								DIR2_H      //��ת
							}
							else
							{
								return;//20170609
							}

							VM_H//�������Դ

							CheckMotor(500);//�������Դ����ȴ������ô��ʶ����ܵ�ѹ�ȶ�

							//CompCounter=0; //��������������0
							//EIFR|=0x01;//��INT0�жϱ�־
							//EIMSK&=0xfe;//��INT0�жϣ��������ǰ�ڲ��ü��������

							TCCR1B = 0x00; //stop
							TCNT1 = 61629;   //��ʱ500ms
							TIFR|=0x04; //�嶨ʱ��1�жϱ�־
							TCCR1B = 0x05; //1024��Ƶ
							TIFR|=0x04; //�嶨ʱ��1�жϱ�־

							Pwm_startup(2);//PWM��������ʱԼ200ms,�������ǰ�ڲ����е�����飬����Ƿѹ���					

							while((TIFR&0x04)==0x00); //�ȴ���ʱ�����ڼ������Ƿѹ�жϷ���
							TCCR1B = 0x00; //stop

							//EIFR|=0x01;//��INT0�жϱ�־
							//EIMSK|=0x01;//��INT0�жϣ��������500ms�󼴿�ʼ���������

							CheckMotor(500);//��ʱ��֤���ߵ�ѹ�ȶ�		 	 
						}
						T_dat=Motor2Status.i;
						SendManchester();		
						if(R_dat>0x72)//΢������Ҫ��ʱ����ʱ�����Զ�ֹͣ
						{
							ETIFR=0x04;//��Timer3����жϱ�־
							ETIMSK = 0x04; //Timer3����ж�ʹ�ܣ������жϹر�
							TCCR3B = 0x00;

							j=(R_dat-0x72+1)/2;
							j=j*7812;
							j=65536-j;
							j+=1;

							TCNT3H = j>>8;  //��С1�룬���5��
							TCNT3L = j;

							//TCNT3H = 0xE1;  //1��
							//TCNT3L = 0x7C;

							//TCNT3H = 0xC2; //2��
							//TCNT3L = 0xF7;

							// TCNT3H = 0xA4; //3��
							// TCNT3L = 0x73;

							TCCR3A = 0x00;
							TCCR3B = 0x05; //8Mʱ�ӣ�1024��Ƶ����ʱ1��
							ETIFR=0x04;//��Timer3����жϱ�־
						} 
						break;

					case 0x80://������
						if(R_dat==0x80)//ֻ�������Դ���������
						{        	  
							CheckMotor(30);//�ӳ�10ms  
							VM_H//�������Դ
							CheckMotor(30);//�ӳ�10ms  
							T_dat=0x0000;   
							SendManchester();
						}	  
						else if(R_dat==0x81)//��ȡ���״̬	
						{ 
							//�ȴ����շŵ��
							CheckMotor(30);//�ӳ�10ms  
							T_dat=Motor1Status.i;	   
							SendManchester();

							//�ٴ������ڵ�� 
							CheckMotor(30);//�ӳ�10ms  
							T_dat=Motor2Status.i;	    
							SendManchester();
						}	  
						else if(R_dat==0x82)//���ֹͣ��ͬʱֹͣ�������
						{	 
							if( (Motor1Run_P) || (Motor1Run_N) )//�շŵ����������
							{
								Motor1Status.s.CommandStop=1;
							}
							if( (Motor2Run_P) || (Motor2Run_N) )//���ڵ����������
							{ 
								Motor2Status.s.CommandStop=1;
							}
							MOTORSTOP  //�ص��
							CheckMotor(200);
							T_dat=0x0000;	
							SendManchester();	 
						}
						else if(R_dat==0x83)//��ȡ�����ת������λ�Ͷ�·��λ
						{
							Motor1Gear=EEPROM_read(0);//�շŵ��
							Motor2Gear=EEPROM_read(1);//���ڵ��
							MotorIdelGear=EEPROM_read(2); //�����·
							Gear2Thr();
							CheckMotor(30);//�ӳ�30ms  	
							T_dat=MotorIdelGear;
							T_dat<<=4;
							T_dat|=Motor2Gear;	 
							T_dat<<=4;
							T_dat|=Motor1Gear; 
							SendManchester();
						}	
						else if(R_dat>0x83)//��ʣ12�����������õ����ת����, �շŵ���͵��ڵ����6��
						{
							if(R_dat==0x84)
							{
								Motor1Gear=1; 
								EEPROM_write(0,Motor1Gear);
							}
							else if(R_dat==0x85)
							{
								Motor1Gear=2; 
								EEPROM_write(0,Motor1Gear);
							}
							else if(R_dat==0x86)
							{
								Motor1Gear=3; 
								EEPROM_write(0,Motor1Gear);
							}
							else if(R_dat==0x87)
							{
								Motor1Gear=4; 
								EEPROM_write(0,Motor1Gear);
							}
							else if(R_dat==0x88)
							{
								Motor1Gear=5; 
								EEPROM_write(0,Motor1Gear);
							}
							else if(R_dat==0x89)
							{
								Motor1Gear=6; 
								EEPROM_write(0,Motor1Gear);
							}
							else if(R_dat==0x8a)
							{
								Motor2Gear=1; 
								EEPROM_write(1,Motor2Gear);
							}
							else if(R_dat==0x8b)
							{
								Motor2Gear=2;
								EEPROM_write(1,Motor2Gear);
							}
							else if(R_dat==0x8c)
							{
								Motor2Gear=3;
								EEPROM_write(1,Motor2Gear);
							}
							else if(R_dat==0x8d)
							{
								Motor2Gear=4;
								EEPROM_write(1,Motor2Gear);
							}
							else if(R_dat==0x8e)
							{
								Motor2Gear=5;
								EEPROM_write(1,Motor2Gear);
							}
							else if(R_dat==0x8f)
							{
								Motor2Gear=6;
								EEPROM_write(1,Motor2Gear);
							}	  
							Gear2Thr();
							CheckMotor(30);//�ӳ�30ms  	
							T_dat=0;	    
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
								CheckMotor(20);
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
								CheckMotor(20);
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
								CheckMotor(20);
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
								CheckMotor(20);
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
									CheckMotor(20);
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
									CheckMotor(20);
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
									CheckMotor(20);
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
									CheckMotor(20);
								}
							}		
						}
						else if(R_dat==0x9a)//�����·����Ϊ1��
						{
							MotorIdelGear=1;
							EEPROM_write(2,MotorIdelGear);  
							Gear2Thr();
							CheckMotor(30);//�ӳ�30ms  	
							T_dat=0;	    
							SendManchester(); 
						}
						else if(R_dat==0x9b)//�����·����Ϊ2��
						{
							MotorIdelGear=2;
							EEPROM_write(2,MotorIdelGear);  
							Gear2Thr();
							CheckMotor(30);//�ӳ�30ms  	
							T_dat=0;	    
							SendManchester(); 
						}
						else if(R_dat==0x9c)//�����·����Ϊ3��
						{
							MotorIdelGear=3;
							EEPROM_write(2,MotorIdelGear);  
							Gear2Thr();
							CheckMotor(30);//�ӳ�30ms  	
							T_dat=0;	    
							SendManchester(); 
						}
						else if(R_dat==0x9d)//�����·����Ϊ4��
						{
							MotorIdelGear=4;
							EEPROM_write(2,MotorIdelGear);  
							Gear2Thr();
							CheckMotor(30);//�ӳ�30ms  	
							T_dat=0;	    
							SendManchester(); 
						}
						else if(R_dat==0x9e)//�����·����Ϊ5��
						{
							MotorIdelGear=5;
							EEPROM_write(2,MotorIdelGear);  
							Gear2Thr();
							CheckMotor(30);//�ӳ�30ms  	
							T_dat=0;	    
							SendManchester(); 
						}
						else if(R_dat==0x9f)//�����·����Ϊ6��
						{
							MotorIdelGear=6;
							EEPROM_write(2,MotorIdelGear);  
							Gear2Thr();
							CheckMotor(30);//�ӳ�30ms  	
							T_dat=0;	    
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
									CheckMotor(20);
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
									CheckMotor(20);
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
									CheckMotor(20);
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
									CheckMotor(20);
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
						CheckMotor(40);
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
						CheckMotor(42);
						}
						} */  
						break;

					case 0xc0://���в���ȫ�ɣ��ϴ�16�ֽڹ�8֡
						if(R_dat==0xc0)
						{
							Uart0TxData[0]=0xE8;
							Uart0TxData[1]=0x40;
							Uart0TxData[2]=0x89;
							Uart0TxData[3]=9;
							SendUart0_2(9,50);//������ֱ����⣬��ʱ400ms
							if(Uart0RxCounter==64)//�ɹ����յ�ѹ�����¶ȡ��������ݣ��ȵ��ֽں���ֽ�
							{
								T_dat=Uart0RxData[1];
								T_dat<<=8;
								T_dat|=Uart0RxData[0];
								SendManchester();//ѹ��
								crc16array[0]=(unsigned char)(T_dat);
								crc16array[1]=(unsigned char)(T_dat>>8);


								CheckMotor(30);	
								T_dat=Uart0RxData[3];
								T_dat<<=8;
								T_dat|=Uart0RxData[2];
								SendManchester();//�¶�
								crc16array[2]=(unsigned char)(T_dat);
								crc16array[3]=(unsigned char)(T_dat>>8);

								CheckMotor(30);	

								lfib=Uart0RxData[5];
								lfib<<=8;
								lfib|=Uart0RxData[4];
								lfib&=0x0000ffff;
								myFIB.f=(unsigned long)lfib; 

								T_dat=myFIB.i[1];
								SendManchester();//����-ʱ��1
								crc16array[4]=(unsigned char)(T_dat);
								crc16array[5]=(unsigned char)(T_dat>>8);

								CheckMotor(30);	
								T_dat=myFIB.i[0];
								SendManchester();//����-ʱ��2
								crc16array[6]=(unsigned char)(T_dat);
								crc16array[7]=(unsigned char)(T_dat>>8);

								CheckMotor(30);	

								lfib=Uart0RxData[7];
								lfib<<=8;
								lfib|=Uart0RxData[6];
								lfib&=0x0000ffff;
								myFIB.f=(unsigned long)lfib; 

								T_dat=myFIB.i[1];
								SendManchester();//����-��λ1
								crc16array[8]=(unsigned char)(T_dat);
								crc16array[9]=(unsigned char)(T_dat>>8);

								CheckMotor(30);	
								T_dat=myFIB.i[0];
								SendManchester();//����-��λ2
								crc16array[10]=(unsigned char)(T_dat);
								crc16array[11]=(unsigned char)(T_dat>>8);

								CheckMotor(30);	 
								T_dat=Motor1Status.i;
								SendManchester();//�շŵ��״̬
								crc16array[12]=(unsigned char)(T_dat);
								crc16array[13]=(unsigned char)(T_dat>>8);

								CheckMotor(30);	 
								T_dat=Motor2Status.i;
								SendManchester();//���ڵ��״̬
								crc16array[14]=(unsigned char)(T_dat);
								crc16array[15]=(unsigned char)(T_dat>>8);

								CheckMotor(30);	
								f=SampleADC(4);//Cable CAB-R1
								f=f*2.56;
								f=f*101;
								f=f/256;
								T_dat=f;
								CheckMotor(30);
								T_dat<<=8;   
								f=SampleADC(5);//VIN / VR CAB-R2
								f=f*2.56;
								f=f*101;
								f=f/256;
								i=f;
								T_dat|=i; 
								SendManchester();//2·���µ�ѹ
								crc16array[16]=(unsigned char)(T_dat);
								crc16array[17]=(unsigned char)(T_dat>>8);

								CheckMotor(30);	
								crc16(crc16array,18);//0.75ms
								T_dat=crc16hi;
								T_dat<<=8;
								T_dat|=crc16lo;	  
								SendManchester();//�ϴ�CRCУ�鹲2�ֽ�
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
									CheckMotor(20);
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
									CheckMotor(20);
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
									CheckMotor(20);
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
									CheckMotor(20);
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
									CheckMotor(20);
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

#pragma interrupt_handler E_comp:24
void E_comp(void)  //ģ��Ƚ����ж�,���ߵ�ѹ�����͵�28.5V������ͣ���
{   
  if( (Motor1Run_P) || (Motor1Run_N) )//�շŵ��Ƿѹ��ת
   {
 	 Motor1Status.s.UnderVoltage=1;
	 Motor1Status.s.RUN=0;//ֹͣ״̬
   }
  if( (Motor2Run_P) || (Motor2Run_N) )//���ڵ��Ƿѹ��ת
   { 
 	 Motor2Status.s.UnderVoltage=1;
	 Motor2Status.s.RUN=0;//ֹͣ״̬
   } 
   MOTORSTOP  //�ص�� 
}

/*
#pragma interrupt_handler int0_isr:2
void int0_isr(void)//�ⲿ�ж�0��������ת
{
  CompCounter++; 
  if(CompCounter>1000)
  {
   MOTORSTOP  //�ص��
  
   if(Motor1Status.s.ThisRun>0)//�շŵ��������ת
    {
	 Motor1Status.s.ThisRun=0;
	 Motor1Status.s.CommandStop=0;
 	 Motor1Status.s.OverCurrent=1;
 	 Motor1Status.s.UnderVoltage=0;
	}
   if(Motor2Status.s.ThisRun>0)//���ڵ��������ת
    {
	  Motor2Status.s.ThisRun=0;
	  Motor2Status.s.CommandStop=0;
 	  Motor2Status.s.OverCurrent=1;
 	  Motor2Status.s.UnderVoltage=0;
	}
  }
}
*/


/*
#pragma interrupt_handler ad_isr:22
void ad_isr(void)  //adת���ж�
{
  ADCData[ADCCn]=ADCH;
  ADCData[ADCCn]<<=8;
  ADCData[ADCCn]|=ADCL;//��ADת�����
  ADCCn++;
  ADCData[ADCCn]&=0x03ff;//10λ��Чת�����
  if(ADCCn==129)
   { 
   ADCSRA&=0xF7;//��AD�ж�
   ADCSRA|=0x10;//��AD�жϱ�־
   ADCSRA=0;//��ADC
   }
}
*/



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