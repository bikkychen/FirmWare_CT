
#include <iom128v.h>								   	
#include <macros.h>
#include <stdio.h>

 //20170703
 //��Ӧ�����ĵ����,R2��Ϊ30K
 
#define  BB     0x10        //�̼��汾��

#define INT_EN		{ SEI(); }
#define INT_DIS		{ CLI(); }
 
//����ܽŶ���
#define MT1_P1_Set()     (PORTG |= (1<<PORTG1))  
#define MT1_P1_Clr()     (PORTG &= ~(1<<PORTG1)) 
#define MT1_N1_Set()     (PORTG |= (1<<PORTG0))  
#define MT1_N1_Clr()     (PORTG &= ~(1<<PORTG0)) 
#define MT1_P2_Set()     (PORTD |= (1<<PORTD6))  
#define MT1_P2_Clr()     (PORTD &= ~(1<<PORTD6)) 
#define MT1_N2_Set()     (PORTD |= (1<<PORTD7))  
#define MT1_N2_Clr()     (PORTD &= ~(1<<PORTD7)) 

#define MT2_P1_Set()     (PORTA |= (1<<PORTA4))  
#define MT2_P1_Clr()     (PORTA &= ~(1<<PORTA4)) 
#define MT2_N1_Set()     (PORTA |= (1<<PORTA5))  
#define MT2_N1_Clr()     (PORTA &= ~(1<<PORTA5)) 
#define MT2_P2_Set()     (PORTA |= (1<<PORTA2))  
#define MT2_P2_Clr()     (PORTA &= ~(1<<PORTA2)) 
#define MT2_N2_Set()     (PORTA |= (1<<PORTA3))  
#define MT2_N2_Clr()     (PORTA &= ~(1<<PORTA3)) 


//�����Դ
#define VM_Open()        (PORTA |= (1<<PORTA6))
#define VM_Close()       (PORTA &= ~(1<<PORTA6)) 

//ͬʱ�ض������
#define MOTORSTOP   {MT1_P2_Clr();MT1_N2_Clr();MT2_P2_Clr();MT2_N2_Clr();VM_Close();MT1_P1_Clr();MT1_N1_Clr();MT2_P1_Clr();MT2_N1_Clr();}

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
#define DELAY500 for(Tt=0;Tt<210;Tt++);

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
unsigned char R_dat; 
unsigned char IntFlag;
unsigned char Tt;


void InitialIO(void);
void Pwm_startup(unsigned char motor);
void Start(void);
unsigned char SampleADC(unsigned char ch);
void EEPROM_write(unsigned int Address,unsigned char Data);
unsigned char EEPROM_read(unsigned int Address);
void CheckMotor(unsigned int t);
void Gear2Thr(void);
void uart1_rx_isr(void);
void SendUart1(unsigned char dat);

void InitialIO(void)
{//1�����0����
 PORTA=0x00;
 DDRA=0x7c;  

 PORTD=0x00; 
 DDRD=0xc0; 

 PORTG=0x00; 
 DDRG=0x03; 
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
  R_dat=UDR1;
  IntFlag=1;
}

void SendUart1(unsigned char dat)
{
  while(!(UCSR1A&(1<<UDRE1)));   // �ȴ����ͻ�����Ϊ��
  UDR1=dat;  
}


#pragma interrupt_handler timer3_ovf_isr:iv_TIM3_OVF
void timer3_ovf_isr(void)//΢��ʱ�������ʱ�ж����Զ��ص��
{
 	ETIMSK = 0x00; //Timer3�жϹر�
	TCCR3B = 0x00; //stop
	 
	MOTORSTOP  //�ص��
		
    if(Motor1Status.s.RUN==1)//�շŵ����������
   	{
		Motor1Status.s.RUN=0;
 		Motor1Status.s.CommandStop=1;
   	}
	
 	if(Motor2Status.s.RUN==1)//���ڵ����������
   	{ 
		Motor2Status.s.RUN=0;
 		Motor2Status.s.CommandStop=1;
   	} 	     
}


void Pwm_startup(unsigned char motor)//���������
{	unsigned char c,b;

	if(motor==1)
	{  
	   Motor1Status.s.RUN=1;
	   
	   if(Motor1Status.s.DIR==1)//��ת
	   { 
	     MT1_N1_Clr();// ����MOS��
		 DELAY500
		 DELAY500
		 MT1_N2_Clr();// ����MOS��
		 DELAY500
		 DELAY500
		 
	     MT1_P1_Set();//����MOS��
		 DELAY500
		 
	     for(b=0;b<30;b++)//3ms���õ�ռ�ձ��������
		 {
		  MT1_P2_Clr();//�ر���MOs�� 
		  DELAY100
		  MT1_P2_Set();//����MOS��
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//130ms���ý���ռ�ձ��������
		 {
	       MT1_P2_Clr();//�ر���MOs�� 
		   
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor1Status.s.UnderVoltage==0)//��������������û�з���Ƿѹ�����������
		   {
		     MT1_P2_Set();//����MOS��
		   }
		   else 
		   {
		     MT1_P2_Clr();//�ر���MOs�� 
		   }
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	   else//��ת
	   {
	      MT1_P1_Clr();// ����MOS��
		  DELAY500
		  DELAY500
		  MT1_P2_Clr();// ����MOS��
		  DELAY500
		  DELAY500
		 
	      MT1_N1_Set();//����MOS��
		  DELAY500
		 
	     for(b=0;b<30;b++)//3ms���õ�ռ�ձ��������
		 {
		  MT1_N2_Clr();//�ر���MOs�� 
		  DELAY100
		  MT1_N2_Set();//����MOS��
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//130ms���ý���ռ�ձ��������
		 {
	       MT1_N2_Clr();//�ر���MOs�� 
		   
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor1Status.s.UnderVoltage==0)//��������������û�з���Ƿѹ�����������
		   {
		     MT1_N2_Set();//����MOS��
		   }
		   else 
		   {
		     MT1_N2_Clr();//�ر���MOs�� 
		   }
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	} 
	else if(motor==2)
	{  
	  Motor2Status.s.RUN=1;
	  
	  if(Motor2Status.s.DIR==1)//��ת
	   { 
	     MT2_N1_Clr();// ����MOS��
		 DELAY500
		 DELAY500
		 MT2_N2_Clr();// ����MOS��
		 DELAY500
		 DELAY500
		 
	     MT2_P1_Set();//����MOS��
		 DELAY500
		 
	     for(b=0;b<30;b++)//3ms���õ�ռ�ձ��������
		 {
		  MT2_P2_Clr();//�ر���MOs�� 
		  DELAY100
		  MT2_P2_Set();//����MOS��
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//130ms���ý���ռ�ձ��������
		 {
	       MT2_P2_Clr();//�ر���MOs�� 
		   
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor2Status.s.UnderVoltage==0)//��������������û�з���Ƿѹ�����������
		   {
		     MT2_P2_Set();//����MOS��
		   }
		   else 
		   {
		     MT2_P2_Clr();//�ر���MOs�� 
		   }
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	   else //��ת
	   {
	      MT2_P1_Clr();// ����MOS��
		  DELAY500
		  DELAY500
		  MT2_P2_Clr();// ����MOS��
		  DELAY500
		  DELAY500
		 
	      MT2_N1_Set();//����MOS��
		  DELAY500
		 
	     for(b=0;b<30;b++)//3ms���õ�ռ�ձ��������
		 {
		  MT2_N2_Clr();//�ر���MOs�� 
		  DELAY100
		  MT2_N2_Set();//����MOS��
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//130ms���ý���ռ�ձ��������
		 {
	       MT2_N2_Clr();//�ر���MOs�� 
		   
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor2Status.s.UnderVoltage==0)//��������������û�з���Ƿѹ�����������
		   {
		     MT2_N2_Set();//����MOS��
		   }
		   else 
		   {
		     MT2_N2_Clr();//�ر���MOs�� 
		   }
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	}
	
}

   
unsigned char SampleADC(unsigned char ch)//��ʱԼ5ms������8λ��Ч���
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

void CheckMotor(unsigned int t)//�����תʱ��ʱ5us�����ʱ8388ms������tΪ��ʱʱ�䣬��λms
{   //���������Ƭ�������ܽ��ϣ�û�����ʱԼ70mV������������ʱԼ80mV������������Ҫ�������λ������ 
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
	TIFR |= 0x04; //�嶨ʱ��1�жϱ�־ 
 	TCCR1B = 0x05; //1024��Ƶ
	
   while((TIFR&0x04)==0x00)//��ʱ�жϵ���ǰһֱ����������
   {   
    if( Motor1Status.s.RUN==1 )//�շŵ������ʱ��ʵʱ����������������������һ�εĲ���ֵ
    {
	  a=SampleADC(0);
	  if(a>5){a-=5;}// ����Ӳ����λ��ÿ������������10mV����Ƭ����ADC����ƫС�����������ֻ����50m���ɡ�
	  else{a=0;}
	  
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
	  
	  f=a;
	  f*=1.087;//����������1.2ŷ���Ŵ���7.667��8λAD��2.56V�ο�����
	  Motor1Status.s.ThisCurrent=f;
    }
   
    if( Motor2Status.s.RUN==1 )//���ڵ������ʱ��ʵʱ����������������������һ�εĲ���ֵ
    {
	 a=SampleADC(1);	
	 if(a>5){a-=5;}// ����Ӳ����λ��ÿ������������10mV����Ƭ����ADC����ƫС�����������ֻ����50m���ɡ�
	  else{a=0;}
	  
     if(a>Motor2Thr)//�����ص��
     {
	   MOTORSTOP//�ص��
       Motor2Status.s.OverCurrent=1;//����״̬��1����ʾ����ϴ�ֹͣԭ��Ϊ����ͣ
	   Motor2Status.s.RUN=0;//ֹͣ״̬
	 } 
	 else if(a<MotorIdelCur) //��·�ص��������С��xx mA��Ϊ�Ƕ�·
	 {
	   MOTORSTOP//�ص��
       Motor2Status.s.BrokenStop=1;//��·״̬��1����ʾ����ϴ�ֹͣԭ��Ϊ��·ͣ
	   Motor2Status.s.RUN=0;//ֹͣ״̬
	 }

	  f=a;
	  f*=1.087;//����������1.2ŷ���Ŵ���7.667��8λAD��2.56V�ο�����
	  Motor2Status.s.ThisCurrent=f;
    }

    if(t==0){break;}//����ʱ����һ�ε��״̬��ֱ������
   } 
   
   TCCR1B = 0x00; //stop
}


void Start(void)
{
 InitialIO();
 uart1_init();
 
 MOTORSTOP//�ص��
 
 Motor1Status.s.RUN=0; 
 Motor1Status.s.DIR=0;
 Motor1Status.s.BrokenStop=0;
 Motor1Status.s.CommandStop=0;
 Motor1Status.s.OverCurrent=0;
 Motor1Status.s.UnderVoltage=0;
 Motor1Status.s.StartResult=0;
 Motor1Status.s.ThisCurrent=0;
 
 Motor2Status.s.RUN=0; 
 Motor2Status.s.DIR=0;
 Motor2Status.s.BrokenStop=0;
 Motor2Status.s.CommandStop=0;
 Motor2Status.s.OverCurrent=0;
 Motor2Status.s.UnderVoltage=0;
 Motor2Status.s.StartResult=0;
 Motor2Status.s.ThisCurrent=0;
 
 IntFlag=0;//�޽���֡�ж�
 
 ACSR|=0x10;//��ģ��Ƚ����жϱ�־
 ACSR&=0xf7;//��ģ��Ƚ����ж�
 ACSR|=0x40;//ACBG��λ��ģ��Ƚ����������ڲ���϶��׼1.23V
 ACSR|=0x03;//��ģ��Ƚ���,AIN1�����Ƚ���������0:�Ƚ�������仯���жϣ�1��������2���½����жϣ�3:�������ж�   
 ACSR|=0x08;//��ģ��Ƚ����жϣ��κ�ʱ��Ҫ����Ƿѹ���	
 ACSR|=0x10;//��ģ��Ƚ����жϱ�־	
	
 INT_EN
}

void Gear2Thr(void)
{
  //����������1.2ŷ���Ŵ���7.667��8λAD��2.56V�ο����㣬ÿ�������ֱ�ʾ1.087mA
//�����������λ����Ϊ�������������жϺ����н��бȽ�����
  if(Motor1Gear<1) {Motor1Gear=1;}//��ת������СΪ1��
  if(Motor1Gear>6) {Motor1Gear=6;}//��ת�������Ϊ6��
  if(Motor2Gear<1) {Motor2Gear=1;}//��ת������СΪ1��
  if(Motor2Gear>6) {Motor2Gear=6;}//��ת�������Ϊ6��
  //��1����100mA�𲽣�ÿ����ֵ30mA,��6��Լ250mA
  Motor1Thr= 108+ (Motor1Gear-1)*27;
  Motor2Thr= 108+ (Motor2Gear-1)*27;
  
  if(MotorIdelGear<1){MotorIdelGear=1;}//�����·������С1��
  if(MotorIdelGear>6){MotorIdelGear=6;}//�����·�������6��
  //��10mA�𲽣����60mA
  MotorIdelCur=(MotorIdelGear-1)*11+10;
  
}

void main(void)
{ 
  unsigned char i;
  unsigned int j;
  unsigned long l;
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
	Gear2Thr();//��λת��Ϊ��ֵ
  

	while(1)
	{ 
		CheckMotor(0);//ʵʱ���е���������

		if(IntFlag==1)//���յ�����������֡
		{
				switch(R_dat&0xf0)
				{ 		
				    case 0xd0://���汾��
					   SendUart1(BB);  
					break;		
					 
					case 0x30:// ���������ѹ,R7ȡ30kʱ������ܲ��Լ88V
						f=SampleADC(2);
						f=f*0.34333;//���ڲ��ο�2.56V��8λAD��������ѹ����ֱ���1M��30K������					
						i=f;
						 if( (Motor1Status.s.RUN==1) || (Motor2Status.s.RUN==1) )
						 {
						  i+=5;//Ϊ���û��������ÿ�Щ
						 }
						SendUart1(i);   
						break;

					case 0x60:  //�շŵ�� 
						if( !(((R_dat)==0x61)  || ((R_dat)==0x62) ) )//�Ȳ�����ת��Ҳ���Ƿ�ת�������������Ҳ���Ӧ��λ��
							break;

						if( Motor2Status.s.RUN==1 )//���ڵ����������
						{
							Motor1Status.s.StartResult=3;//��һ�����������
						}
						else if((Motor1Status.s.RUN==1)&&(Motor1Status.s.DIR==1))//�շŵ��������ת
						{
						  if(R_dat==0x61)
						  {
						    Motor1Status.s.StartResult=1;//���������ͬ������
						  }
						  else if(R_dat==0x62)
						  {
						   Motor1Status.s.StartResult=2;//��������ڷ�������
						  }

						}
						else if((Motor1Status.s.RUN==1)&&(Motor1Status.s.DIR==0))//�շŵ�����ڷ�ת
						{
						  if(R_dat==0x61)
						  {
						   Motor1Status.s.StartResult=2;//��������ڷ������� 
						  }
						  else if(R_dat==0x62)
						  {
						   Motor1Status.s.StartResult=1;//���������ͬ������
						  }
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
								  //�űۣ���ת
								Motor1Status.s.DIR=1;
							}
							else if((R_dat)==0x62)          
							{ 
								 //�ձۣ���ת
								Motor1Status.s.DIR=0;
							}
							else
							{
								return;//20170609
							}

							VM_Open();//�������Դ

							CheckMotor(500);//�������Դ����ȴ������ô��ʶ����ܵ�ѹ�ȶ�


							TCCR1B = 0x00; //stop
							TCNT1 = 61629;   //��ʱ500ms
							TIFR|=0x04; //�嶨ʱ��1�жϱ�־
							TCCR1B = 0x05; //1024��Ƶ
							TIFR|=0x04; //�嶨ʱ��1�жϱ�־
							Pwm_startup(1);//PWM����,�������ǰ��500ms�ڲ����е�����飬����Ƿѹ���		
							while((TIFR&0x04)==0x00); //�ȴ�500ms��ʱ�����ڼ������Ƿѹ�жϷ���
							TCCR1B = 0x00; //stop

							CheckMotor(500);//��ʱ��֤���ߵ�ѹ�ȶ�		
						}
			 
			            //����2�ֽڵ��״̬
						SendUart1(Motor1Status.i); CheckMotor(2);
						SendUart1(Motor1Status.i>>8);

						break;

					case 0x70://���ڵ��   
						if( ((R_dat)<0x71)  || ((R_dat)>0x7c)  )//�Ȳ��ǵ���Ҳ���ǵ�С��Ҳ��΢����Ҳ����΢��С�������������Ҳ���Ӧ��λ��
						break;

						i=R_dat%2;
						
						if(Motor1Status.s.RUN==1)//�շŵ����������
						{
							Motor2Status.s.StartResult=3;//��һ�����������
						}
						else if((Motor2Status.s.RUN==1)&&(Motor2Status.s.DIR==1))//���ڵ��������ת
						{
						  if(i==0)//ż�������ת
						  {
						   Motor2Status.s.StartResult=1;//���������ͬ������
						  }
						  else//���������ת
						  {
						   Motor2Status.s.StartResult=2;//��������ڷ�������
						  }
						}
						else if((Motor2Status.s.RUN==1)&&(Motor2Status.s.DIR==0))//���ڵ�����ڷ�ת
						{
						  if(i==0)//ż�������ת
						  {
						   Motor2Status.s.StartResult=2;//��������ڷ�������
						  }
						  else//���������ת
						  {
						   Motor2Status.s.StartResult=1;//���������ͬ������
						  }
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
								//��ת
								Motor2Status.s.DIR=0;
							}
							else if( i==0 )    //ż�������С
							{ 
								//��ת
								Motor2Status.s.DIR=1;
							}
							else
							{
								return;//20170609
							}

							VM_Open();//�������Դ

							CheckMotor(500);//�������Դ����ȴ������ô��ʶ����ܵ�ѹ�ȶ�


							TCCR1B = 0x00; //stop
							TCNT1 = 61629;   //��ʱ500ms
							TIFR|=0x04; //�嶨ʱ��1�жϱ�־
							TCCR1B = 0x05; //1024��Ƶ
							TIFR|=0x04; //�嶨ʱ��1�жϱ�־
							Pwm_startup(2);//PWM�������������ǰ��500ms�ڲ����е�����飬����Ƿѹ���					
							while((TIFR&0x04)==0x00); //�ȴ�500ms��ʱ�����ڼ������Ƿѹ�жϷ���
							TCCR1B = 0x00; //stop

							CheckMotor(500);//��ʱ��֤���ߵ�ѹ�ȶ�		 	 
						}
						
						//����2�ֽڵ��״̬
						SendUart1(Motor2Status.i); CheckMotor(2);
						SendUart1(Motor2Status.i>>8);
						 	
						if(R_dat>0x72)//΢������Ҫ��ʱ����ʱ�����Զ�ֹͣ
						{
							ETIFR=0x04;//��Timer3����жϱ�־
							ETIMSK = 0x04; //Timer3����ж�ʹ�ܣ������жϹر�
							TCCR3B = 0x00;

							j=(R_dat-0x72+1)/2;
							j+=1;// Ϊ����λ�������ܿ��������仯 ��ÿ�������һ��
							j=j*7812;
							j=65536-j;
							j+=1;

							TCNT3H = j>>8;  //��С1�룬���5��
							TCNT3L = j;

							TCCR3A = 0x00;
							TCCR3B = 0x05; //8Mʱ�ӣ�1024��Ƶ����ʱ1��
							ETIFR=0x04;//��Timer3����жϱ�־
						} 
						break;

					case 0x80://������
						if(R_dat==0x80)//ֻ�������Դ���������
						{        	  						 
							VM_Open();//�������Դ
						    SendUart1(0); 
						}	  
						else if(R_dat==0x81)//��ȡ���״̬	
						{ 
							CheckMotor(2); 
							//����2�ֽڵ��״̬
						     SendUart1(Motor1Status.i); 
							 CheckMotor(2); 
						     SendUart1(Motor1Status.i>>8); 
							 CheckMotor(2); 
							//����2�ֽڵ��״̬
						    SendUart1(Motor2Status.i);  
							CheckMotor(2); 
						    SendUart1(Motor2Status.i>>8); 
						}	  
						else if(R_dat==0x82)//���ֹͣ��ͬʱֹͣ�������
						{	 
						    MOTORSTOP  //�ص��
							if(Motor1Status.s.RUN==1)//�շŵ����������
							{
							    Motor1Status.s.RUN=0;
								Motor1Status.s.CommandStop=1;
							}
							if(Motor2Status.s.RUN==1)//���ڵ����������
							{ 
							    Motor2Status.s.RUN=0;
								Motor2Status.s.CommandStop=1;
							}
							
							CheckMotor(180);//�������ȶ����ٻط�
							SendUart1(0);  
						}
						else if(R_dat==0x83)//��ȡ�����ת������λ�Ͷ�·��λ
						{
							Motor1Gear=EEPROM_read(0);//�շŵ��
							Motor2Gear=EEPROM_read(1);//���ڵ��
							MotorIdelGear=EEPROM_read(2); //�����·
							Gear2Thr();

							i=Motor2Gear;	 
							i<<=4;
							i|=Motor1Gear; 
							SendUart1(i); 
							CheckMotor(2);
							i=MotorIdelGear;
							SendUart1(i); 
						}	
						else if(R_dat>0x89)//0x8a~0x8f,���õ��ڵ��������ת��λ
						{
						    Motor2Gear=R_dat-0x89;//��Ӧ1~6�� 
							EEPROM_write(1,Motor2Gear);
							Gear2Thr();
							SendUart1(0); 
						}	
						else if(R_dat>0x83)//0x84~0x89,�����շŵ����ת��λ
						{
						    Motor1Gear=R_dat-0x83;//��Ӧ1~6�� 
							EEPROM_write(0,Motor1Gear);
							Gear2Thr();
							SendUart1(0); 
						}	  
						break;

					case 0x90: 
						if(R_dat>0x99)//0x9a~0x99�����õ����·������λ
						{
							MotorIdelGear=R_dat-0x99;
							EEPROM_write(2,MotorIdelGear);  
							Gear2Thr(); 
							SendUart1(0);  
						}				
						break;

					default: 
						break;	
				}
			IntFlag=0;
		}  
	}
}

#pragma interrupt_handler E_comp:24
void E_comp(void)  //ģ��Ƚ����ж�,���ߵ�ѹ�����͵�42.23V������ͣ���(R2Ҫ�ĳ�30k)
{   
  MOTORSTOP  //�ص�� 
  if(Motor1Status.s.RUN==1)//�շŵ��Ƿѹ��ת
   {
 	 Motor1Status.s.UnderVoltage=1;
	 Motor1Status.s.RUN=0;//ֹͣ״̬
   }
  if(Motor2Status.s.RUN==1)//���ڵ��Ƿѹ��ת
   { 
 	 Motor2Status.s.UnderVoltage=1;
	 Motor2Status.s.RUN=0;//ֹͣ״̬
   }   
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

 