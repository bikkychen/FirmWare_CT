#include <iom128v.h>								   	
#include <macros.h>
#include <stdio.h>

//����޶�:20160812
//20170226 ����PIC��Ƭ�����߸��¹��ܣ����⻹������һ����������������0xf0
//20170701 ���������ذ�CT_MAIN_V30

#define  BB     0x00        //�̼��汾��

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
unsigned char Rx[18];//�������9λ��18����λ
unsigned int  T_dat,R_dat;//���뷢������
unsigned char T_com;//���뷢������
unsigned char T_dat_buf[22];//���뷢�ͻ��� 
unsigned char Int_count;
unsigned char IntFlag;//����֡״̬
 
unsigned int Tt;

unsigned char UpdateBegin;//�������������ݿ�ʼ��־
unsigned int DataReCn;
unsigned char crc16hi,crc16lo,DownloadSpeed;
unsigned int Timer3Cn;
unsigned char DSP_PageCn;//DSP����ҳ����

unsigned char Uart0RxCounter,Uart1RxCounter;
unsigned char Uart0RxData[128],Uart1RxData[128];
unsigned int Timer0Counter;
unsigned char TimeOutFlag;

char flash_buf[258]; //FALSHҳ������,M128��һ��FlashҳΪ256�ֽ�(128��) //��ֻ֧��64K��ַ���
long address = 0; 
unsigned int T2cn;

void Start(void);
void SendManchester(void); 
void Delay30Ms();
*/
unsigned char Rx[18];//�������9λ��18����λ
unsigned int  T_dat,R_dat;//���뷢������
unsigned char T_com;//���뷢������
unsigned char T_dat_buf[22];//���뷢�ͻ��� 
unsigned char Int_count;
unsigned char IntFlag;//����֡״̬
 
unsigned int Tt;

unsigned char UpdateBegin;//�������������ݿ�ʼ��־
unsigned int DataReCn;
unsigned char crc16hi,crc16lo,DownloadSpeed;
unsigned int Timer3Cn;
unsigned char DSP_PageCn;//DSP����ҳ����

unsigned char Uart0RxCounter,Uart1RxCounter;
unsigned char Uart0RxData[128],Uart1RxData[128];
unsigned int Timer0Counter,Timer1Counter;
unsigned char TimeOutFlag;

char flash_buf[258]; //FALSHҳ������,M128��һ��FlashҳΪ256�ֽ�(128��) //��ֻ֧��64K��ַ���
long address = 0; 
unsigned int T2cn;

void Start(void);

void SendManchester(void); 
 
void Delay30Ms(void);


//����(code=0x03)��д��(code=0x05)һ��Flashҳ 
void boot_page_ew(long p_address,char code) 
{ 
    asm("mov r30,r16\n" 
        "mov r31,r17\n" 
        "out 0x3b,r18\n");            //��ҳ��ַ����Z�Ĵ�����RAMPZ��Bit0�� 
    SPMCSR = code;                //�Ĵ���SPMCSR��Ϊ������ 
    asm("spm\n");                    //��ָ��Flashҳ���в��� 
}         
//���Flash����ҳ�е�һ���� 
void boot_page_fill(unsigned int address,int data) 
{ 
    asm("mov r30,r16\n" 
        "mov r31,r17\n"             //Z�Ĵ�����Ϊ���ҳ�ڵ�ַ 
        "mov r0,r18\n" 
        "mov r1,r19\n");            //R0R1��Ϊһ��ָ���� 
    SPMCSR = 0x01; 
    asm("spm\n"); 
} 
//�ȴ�һ��Flashҳ��д��� 
void wait_page_rw_ok(void) 
{ 
      while(SPMCSR & 0x40) 
     { 
         while(SPMCSR & 0x01); 
         SPMCSR = 0x11; 
         asm("spm\n"); 
     } 
} 
//����һ��Flashҳ���������� 
void write_one_page(void) 
{ 
    int i; 
    boot_page_ew(address,0x03);                    //����һ��Flashҳ 
    
    wait_page_rw_ok();                            //�ȴ�������� 
    for(i=0;i<256;i+=2)                //����������Flash����ҳ�� 
    { 
        boot_page_fill(i, flash_buf[i+2]+(flash_buf[i+3]<<8)); 
    } 
   boot_page_ew(address,0x05);                    //������ҳ����д��һ��Flashҳ 
     
   wait_page_rw_ok();                            //�ȴ�д����� 
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
void uart1_rx_isr(void)//����1�����ж�
{
 Uart1RxData[Uart1RxCounter]=UDR1;//���մ�������,ͬʱ��մ��ڽ����жϱ�־
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
void usart0_isr(void) //���ڽ����ж�
{ 
    Uart0RxData[Uart0RxCounter]=UDR0;//���մ�������,ͬʱ��մ��ڽ����жϱ�־
 	Uart0RxCounter++;
}




void quit(void) 
{
    MCUCR = 0x01; 
    MCUCR = 0x00;       //���ж�������Ǩ�Ƶ�Ӧ�ó�����ͷ�� 
    RAMPZ = 0x00;       //RAMPZ�����ʼ�� 
    asm("jmp 0x0000\n");//��ת��Flash��0x0000����ִ���û���Ӧ�ó��� 
} 

void Start(void)
{
	INIMANIO
	uart0_init();
    uart1_init();
 
    ACSR|=0x10;//��ģ��Ƚ����жϱ�־
	ACSR&=0xf7;//��ģ��Ƚ����ж�
	
	EICRA|=0x03; //INT0�����ش��� ��ÿ2λ����һ���жϣ���4���жϣ�0-�͵�ƽ������1-������2-�½��ش�����3-�����ش��� 
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
  UpdateBegin=0;//�ص�����״̬
  TCCR3B = 0x00; //stop  
  ETIMSK &= 0xfb;//�ض�ʱ��3�ж�
  ETIFR|=0x04; //�嶨ʱ��3�жϱ�־ 
}

	
void SendUart0(unsigned char c,unsigned char s)   //���ڷ�������
{//Ҫ����������s��8ms��ͨ�����ڷ�������
unsigned char t; 	

  Uart0RxData[0]=0xff;
  Uart0RxData[1]=0xff;
  Uart0RxCounter=0; //�崮�ڽ��ܼ���  
  
   t=UDR0;//�����ڽ����ж�ǰ���ջ���
   
  while(!(UCSR0A&(1<<UDRE0)));   // �ȴ����ͻ�����Ϊ��
  UDR0=c;   // �����ݷ��뻺��������������
                   
   t=UDR0;//�����ڽ����ж�ǰ���ջ���
   UCSR0B|=0x80;//�����ڽ����ж�
   
   //��ʱ�趨
	TCCR1B = 0x00; //stop
	if(s==0)
	 {
       TCNT1 = 22576;//5500ms 
	   TCCR1B = 0x05; //1024��Ƶ
	 }
	 else
	 {
	   TCNT1 = 1536;//8ms 
	   TCCR1B = 0x01; //1��Ƶ
	 }
 	
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־
	while((TIFR&0x04)==0x00); 
	TCCR1B = 0x00; //stop
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־ 
	UCSR0B&=0x7f;//�ش��ڽ����ж�
}

void SendUart1(unsigned char c,unsigned char s)   //���ڷ�������
{//Ҫ����������s��8ms��ͨ�����ڷ�������
unsigned char t; 	

  Uart1RxData[0]=0xff;
  Uart1RxData[1]=0xff;
  Uart1RxCounter=0; //�崮�ڽ��ܼ���  
  
   t=UDR1;//�����ڽ����ж�ǰ���ջ���
   
  while(!(UCSR1A&(1<<UDRE1)));   // �ȴ����ͻ�����Ϊ��
  UDR1=c;   // �����ݷ��뻺��������������
                   
   t=UDR1;//�����ڽ����ж�ǰ���ջ���
   UCSR1B|=0x80;//�����ڽ����ж�
   
   //��ʱ�趨
	TCCR1B = 0x00; //stop
	if(s==0)
	 {
       TCNT1 = 22576;//5500ms 
	   TCCR1B = 0x05; //1024��Ƶ
	 }
	 else
	 {
	   TCNT1 = 1536;//8ms 
	   TCCR1B = 0x01; //1��Ƶ
	 }
 	
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־
	while((TIFR&0x04)==0x00); 
	TCCR1B = 0x00; //stop
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־ 
	UCSR1B&=0x7f;//�ش��ڽ����ж�
}

void SetTimer3(void)//ҳд��ʼ�������500msʱ����һ���ֽ�Ҳû�յ������˳�ҳд״̬���ص�����״̬
{  
    TCCR3B = 0x00; //stop    
    TCNT3H = 0xF0; //8M,500ms
    TCNT3L = 0xBE; //8M,500ms
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
 	TCCR3B = 0x05; //1024��Ƶ
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
	ETIMSK |= 0x04; //����ʱ��3�ж�
}

void main(void)
{
 	 unsigned int i,j,k;
	 unsigned char t; 	
     Delay30Ms();
	 
	 IntFlag=0;//�޽���֡�ж�
     UpdateBegin=0;//�ޱ궨ϵ���·�
	 T_dat=0;
	 R_dat=0;
	 DSP_PageCn=0;
	 
 	 Start();
	 
	 Delay30Ms();

	 
	TCCR3B = 0x00; //stop    
	TCNT3H = 0xd1; ////8M,1.5s
    TCNT3L = 0x20; ////8M,1.5s
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
	ETIMSK &= 0xfb;//�ض�ʱ��3�ж�
 	TCCR3B = 0x05; //1024��Ƶ
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
	    
	 UpdateBegin=0;
	while( ((ETIFR&0x04)==0x00) && (UpdateBegin==0) )
	{
	 if((IntFlag==1))//3�����յ���ʼ��������
	 {  
	  if(R_dat==0xfa) //���ذ�����
	   {
	    UpdateBegin=1;
	    T_com=0x0f;
	    T_dat=0xe7fa;	
		Delay30Ms();
		SendManchester();  
		IntFlag=0;//������֡
	    MANINT_CLR
	    MANINT_EN
	   }  	
	   else if(R_dat==0xf5)//���������
	   {
		UpdateBegin=1;
		SendUart1(0xf5,1);  
		T_com=0x0f;
		T_dat=0xe7;
		T_dat<<=8; 	
		T_dat|=Uart0RxData[0];//�ɹ��Ļ�Ӧ����0xe7f5
		Delay30Ms();
		SendManchester(); 
		IntFlag=0;//������֡
		MANINT_CLR
	    MANINT_EN
		} 	
	   else if(R_dat==0xfb)//�ɼ���DSP����
	   {
	    UpdateBegin=1;
	  	SendUart0(0xb5,1);  
		T_com=0x0f;
		T_dat=Uart0RxData[0];//�ɹ��Ļ�Ӧ����0xe7fb
		T_dat<<=8; 	
		T_dat|=Uart0RxData[1];
	  	Delay30Ms();
      	SendManchester(); 
		IntFlag=0;//������֡
	    MANINT_CLR
	    MANINT_EN 
	   }
	   else if(R_dat==0xf9)//�ɼ���PIC����
	   {
	    UpdateBegin=1;
	  	SendUart0(0xf9,1);  
		T_com=0x0f;
		T_dat=0xe7;
		T_dat<<=8; 	
		T_dat|=Uart0RxData[0];//�ɹ��Ļ�Ӧ����0xe7f9
	  	Delay30Ms();
      	SendManchester(); 
		IntFlag=0;//������֡
	    MANINT_CLR
	    MANINT_EN
	   } 
	 }		
	 if(IntFlag==2)//У��λ����������
   	 { 
	   IntFlag=0;//������֡
	   MANINT_CLR
	   MANINT_EN
   	 }									
	} 
	
	TCCR3B = 0x00; //stop    
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
	ETIMSK &= 0xfb;//�ض�ʱ��3�ж�
	
	if(UpdateBegin==0)//3����û�յ���ʼ���������ֱ������������
	{
	 quit();
	}
	
	UpdateBegin=0;
	while(1)
	{
 	 if(IntFlag==2)//У��λ����������
   	 { 
	   IntFlag=0;//������֡
	   MANINT_CLR
	   MANINT_EN 
   	 }
   	else if(IntFlag==1)//���յ�����������֡
   	{
	 	 if(UpdateBegin>0)//�������������������
     	 { 	  
		      SetTimer3();	 //ÿ�յ�һ�ֽھ����¿�ʼ��ʱ500ms����ʱ��ص�����״̬
			  					   
		      flash_buf[DataReCn]=R_dat;
			  DataReCn++;
			  
			  if(UpdateBegin==2)//����ǲɼ���ҳд׼���ˣ�ֱ�Ӱ�����ֽڶ����ɼ���
			  {
			    while(!(UCSR0A&(1<<UDRE0)));   // �ȴ����ͻ�����Ϊ��
			    UDR0=R_dat; 
			  }
			  else if(UpdateBegin==3)//����ǵ����ҳд׼���ˣ�ֱ�Ӱ�����ֽڶ��������
			  {
			    while(!(UCSR1A&(1<<UDRE1)));   // �ȴ����ͻ�����Ϊ��
			    UDR1=R_dat; 
			  }	 
			   
		 	 if(DataReCn==258)//2�ֽ�ҳ������256�ֽ�ҳ����
		   	 {
			   TCCR3B = 0x00; //stop  
  			   ETIMSK &= 0xfb;//�ض�ʱ��3�ж�
  			   ETIFR|=0x04; //�嶨ʱ��3�жϱ�־ 
  			   		   
  			   if(UpdateBegin==1)//��������ذ�ҳд׼���ˣ����ʱд��һҳ
				{
                  address=flash_buf[0]*256+flash_buf[1];  
			      address*=256;	 
				  
			      if(address<0)
			          address=0;
				 
			     if(address>0xff00)//��Ӧҳ����Ϊ255���������һҳ��д��ҳ��Ŀǰ���ƹ̼���СΪ64K�ֽ�
			         address=0xff00;
				
				  
		          write_one_page();	//Լ20ms  
				  
			    }   
				  
			   UpdateBegin=0;//һҳд���ˣ���һҳ�ִ�ҳд׼����ʼ
			   
	 		   T_com=0x0f;	      
			   crc16(flash_buf,258);//12ms
		       T_dat=crc16hi;
			   T_dat<<=8;
			   T_dat|=crc16lo;
		       Delay30Ms();
               SendManchester();	//����һҳ��У�����ϴ�����λ������λ���жϺ�����Ǽ�����дһҳ����������һҳ	   		        
			 }	 	 
	     }
  		 else//�������·��궨ϵ������������¸�������������
		 {
    		switch(R_dat)
     		{			
			   case 0x20://���汾��  
		 		  	   T_com=0x02;
					   T_dat=0x00;
					   Delay30Ms();
					   SendManchester();	
	  		   break;  
					   
				case 0xf0://����������
					 T_com=0x0f;
	  			     T_dat=0xe7ff;//��λ��ʵ�յ�����,���øߵͻ�λ				       
	  			     Delay30Ms();
      			     SendManchester();  
				break;
				
				case 0xf5://���������
			         T_com=0x0f;
	  			 	 SendUart1(0xf5,1);  
					 T_dat=0xe7;
		 			 T_dat<<=8; 	
					 T_dat|=Uart1RxData[0];//�ɹ��Ļ�Ӧ����0xf5
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
			   break;	
			
			   case 0xf4://�����ҳд׼��			     
					 T_com=0x0f;
					 SendUart1(0xf4,1); 
	  			     T_dat=0xe7;
		 			 T_dat<<=8;
		 			 T_dat|=Uart1RxData[0];//���������Ӧ��0xf4  
	  			     Delay30Ms();
      			     SendManchester(); 
					 UpdateBegin=3;//ҳд׼������64K�ֽڳ���ռ�
					 DataReCn=0;
				     SetTimer3();
				break;
				
				case 0xf9://PIC���֣��������״̬
					 T_com=0x0f;
	  			 	 SendUart0(0xf9,1);  
					 T_dat=0xe7;
		 			 T_dat<<=8; 	
					 T_dat|=Uart0RxData[0];//�ɹ��Ļ�Ӧ����0xf9
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
				break;
				
				case 0xf8://PICҳд׼��			     
					 T_com=0x0f;
					 SendUart0(0xf8,1);//��дFLASHE׼�� 		
	  			     T_dat=0xe7;
		 			 T_dat<<=8;
		 			 T_dat|=Uart0RxData[0];//���������Ӧ��0xf8  
	  			     Delay30Ms();
      			     SendManchester(); 
					 UpdateBegin=2;//ҳд׼������64K�ֽڳ���ռ�
					 DataReCn=0;
				     SetTimer3();
				break;
				/*
				case 0xf7://PIC�汾��
					 T_com=0x0f;
	  			 	 SendUart0(0xf7,1);  
					 T_dat=0xe7;
		 			 T_dat<<=8; 	
					 T_dat|=Uart0RxData[0];//�ɹ��Ļ�Ӧ����0x00
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
				break;
					
				case 0xf6://DSP�汾��
					 T_com=0x0f;
	  			 	 SendUart0(0xf6,1);  
					  T_dat=Uart0RxData[0];//�ɹ��Ļ�Ӧ����0xe7
		 			 T_dat<<=8; 	
					 T_dat|=Uart0RxData[1];//�ɹ��Ļ�Ӧ����0x00
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
				break;
				   */
			   case 0xfa://���ذ����֣��������״̬
	  			    // SendUart0(0xb5,1);
					 T_com=0x0f;
	  			     T_dat=0xe7fa;//��λ��ʵ�յ�����,���øߵͻ�λ				       
	  			     Delay30Ms();
      			     SendManchester();  
				break;
				
			   case 0xfb://DSP���֣��������״̬
					 T_com=0x0f;
	  			 	 SendUart0(0xb5,1);  
					 T_dat=Uart0RxData[0];//�ɹ��Ļ�Ӧ����0xe7fb
		 			 T_dat<<=8; 	
					 T_dat|=Uart0RxData[1];
	  			 	 Delay30Ms();
      			 	 SendManchester(); 
				break;	
				 		   			
				case 0xfc://���ذ�ҳд׼��			     
					 T_com=0x0f;
	  			     T_dat=0xe7fc;//��λ��ʵ�յ�����,���øߵͻ�λ
	  			     Delay30Ms();
      			     SendManchester(); 
					 UpdateBegin=1;//ҳд׼�������ذ��ҳ����0��ʼ��д�����ҳ����255����64K�ֽڳ���ռ�
					 DataReCn=0;
				     SetTimer3();//��ʼ��ʱ��ÿ500ms������Ҫ��һ���ֽ��յ�
				break;
											
				case 0xfd://DSPҳд׼��			     
					 T_com=0x0f;
					 SendUart0(0xb7,1);//��дFLASHE׼�� 		
	  			     T_dat=Uart0RxData[0];//���ֽڣ����������Ӧ��0xe7
		 			 T_dat<<=8;
		 			 T_dat|=Uart0RxData[1];//���ֽ�,���������Ӧ��0xfd   
	  			     Delay30Ms();
      			     SendManchester(); 
					 UpdateBegin=2;//ҳд׼�����ɼ����ҳ����479��ʼ��ǰ��д����Сҳ����224����64K�ֽڳ���ռ�
					 DataReCn=0;
				     SetTimer3();
				break;
				
				case 0xfe://DSP��������λ�����ܳ�ʱ6��			     
					  SendUart0(0xb6,0);//����FLASHE,2�볬ʱ ,����FLASHD,2�볬ʱ ����5.5�볬ʱ				 
					  T_dat=Uart0RxData[0];//�����ɹ��Ļ�Ӧ����0xe7
		 			  T_dat<<=8; 				  	 
		 			  T_dat|=Uart0RxData[1];//�����ɹ��Ļ�Ӧ����0xfe				   
					  T_com=0x0f;
      			 	  SendManchester();	//�ϴ��������				 					
				break;	
				
				case 0xff://�˳�����״̬��������������
				 SendUart0(0xb8,1);
				 T_dat=Uart0RxData[0];//�ɹ��Ļ�Ӧ����0xe7
		 		 T_dat<<=8; 
				 T_dat|=0xff;
				 T_com=0x0f;
				 Delay30Ms();
				 SendManchester();	//�ϴ��������		
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
void int0_isr(void)//�ⲿ�ж�0
{ 
   TCCR2 = 0x00; //stop
	INT_DIS
	MANINT_DIS
  
  DELAY10
  MANINT_CLR
  IF_MANINTL//�����岻��10us��ֱ������
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
     IF_MANINTL
	 {
	   DELAY20
	   IF_MANINTL
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
	  IF_MANINTF
	  {
	   DELAY10
	   MANINT_CLR
	   IF_MANINTH//������ά����10us������Ϊ����һ����Ч�����ص���
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
     IF_MANINTL
	 {
	   DELAY20
	   IF_MANINTL
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
	  IF_MANINTF
	  {
	   DELAY10
	   MANINT_CLR
	   IF_MANINTH//������ά����10us������Ϊ����һ����Ч�����ص���
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
   INT_EN//���ж�ʹ��;  
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
			
			for(L=0;L>50;L++)
			{
			  DELAY80
			}
	
 	MANINT_CLR
 	INT_EN//���ж�ʹ��;  
}


 

void Delay30Ms(void)//8M��ʱ��Ƶ�ʣ������ʱ7812.5ms
{
	TCCR1B = 0x00; //stop
    TCNT1 = 65301;//30ms  
 	TCCR1B = 0x05; //1024��Ƶ
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־
	while((TIFR&0x04)==0x00); 
	TCCR1B = 0x00; //stop
	TIFR|=0x04; //�嶨ʱ��1�жϱ�־
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