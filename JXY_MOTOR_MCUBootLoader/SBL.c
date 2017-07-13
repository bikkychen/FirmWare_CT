#include <iom128v.h>								   	
#include <macros.h>
#include <stdio.h>

 
//�������������

#define  BB     0x00        //�̼��汾��

#define INT_EN		{ SEI(); }
#define INT_DIS		{ CLI(); }

unsigned int   R_dat; 
unsigned char IntFlag;//����֡״̬
unsigned char UpdateBegin;//�������������ݿ�ʼ��־
unsigned int DataReCn;
unsigned char crc16hi,crc16lo,DownloadSpeed;
unsigned int Timer3Cn;
 

unsigned char  Uart1RxCounter;
unsigned char  Uart1RxData[128];

unsigned char TimeOutFlag;

char flash_buf[258]; //FALSHҳ������,M128��һ��FlashҳΪ256�ֽ�(128��) //��ֻ֧��64K��ַ���
long address = 0; 
unsigned int T2cn;

void Start(void);

 
 
void Delay30Ms(void);

//void EEPROM_write(unsigned int Address,unsigned char Data);
//unsigned char EEPROM_read(unsigned int Address) ;




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
 R_dat=UDR1;//���մ�������,ͬʱ��մ��ڽ����жϱ�־
 IntFlag=1;
}

 

void quit(void) 
{
    MCUCR = 0x01; 
    MCUCR = 0x00;       //���ж�������Ǩ�Ƶ�Ӧ�ó�����ͷ�� 
    RAMPZ = 0x00;       //RAMPZ�����ʼ�� 
    asm("jmp 0x0000\n");//��ת��Flash��0x0000����ִ���û���Ӧ�ó��� 
} 

 
 

#pragma interrupt_handler timer3_ovf_isr:iv_TIM3_OVF
void timer3_ovf_isr(void)
{ 
  UpdateBegin=0;//�ص�����״̬
  TCCR3B = 0x00; //stop  
  ETIMSK &= 0xfb;//�ض�ʱ��3�ж�
  ETIFR|=0x04; //�嶨ʱ��3�жϱ�־ 
}

	


void SendUart1(unsigned char c)   //���ڷ�������
{
  while(!(UCSR1A&(1<<UDRE1)));   // �ȴ����ͻ�����Ϊ��
  UDR1=c;   // �����ݷ��뻺��������������                 
}

void SetTimer3_500ms(void)//ҳд��ʼ�������500msʱ����һ���ֽ�Ҳû�յ������˳�ҳд״̬���ص�����״̬
{  
    TCCR3B = 0x00; //stop    
    TCNT3H = 0xF0; //8M,500ms
    TCNT3L = 0xBE; //8M,500ms
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
 	TCCR3B = 0x05; //1024��Ƶ
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
	ETIMSK |= 0x04; //����ʱ��3�ж�
}

void SetTimer3_3s(void)
{
	TCCR3B = 0x00; //stop    
	TCNT3H = 0xa4; ////8M,3s
    TCNT3L = 0x73; ////8M,3s
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
	ETIMSK &= 0xfb;//�ض�ʱ��3�ж�
 	TCCR3B = 0x05; //1024��Ƶ
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
}

void main(void)
{
 	 unsigned int i,j,k;
	 unsigned char t; 	
	 
     Delay30Ms();

	 uart1_init();
	 INT_EN

	 IntFlag=0;//�޽���֡�ж�
     UpdateBegin=0;//�ޱ궨ϵ���·�
	 
	SetTimer3_3s();	
	
	     
	UpdateBegin=0;
	
	while( ((ETIFR&0x04)==0x00) && (UpdateBegin==0) )
	{
		if((IntFlag==1))//3�����յ���ʼ��������
		{  
		    IntFlag=0;
			if(R_dat==0xf5)//���������
			{
			    UpdateBegin=1;
				SendUart1(0xf5);  
			} 	
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
 	if(IntFlag==1)//���յ�����������֡
   	{
	 	 if(UpdateBegin>0)//�������������������
     	 { 	  
		      SetTimer3_500ms();	 //ÿ�յ�һ�ֽھ����¿�ʼ��ʱ500ms����ʱ��ص�����״̬
			  					   
		      flash_buf[DataReCn]=R_dat;
			  DataReCn++;
			   
			  
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
			   //�ڴ˲��ü���У�飬���ذ��ȥ���� 
			 }	 	 
	     }
  		 else//�������·��궨ϵ������������¸�������������
		 {
    		switch(R_dat)
     		{			
			   case 0xf5://���������
			         SendUart1(0xf5); 
			   break;	   
				
				case 0xf4://�����ҳд׼��
					 SendUart1(0xf4); 
					 UpdateBegin=1;//ҳд׼�� 
					 DataReCn=0;
				     SetTimer3_500ms();
				break;
	
				case 0xff://�˳�����״̬��������������
				 SendUart1(0xff); 	
				 quit();
				break;

				default:
				break;
	  	    }//switch(R_dat&0xf0)
	     }//UpdateBegin
		 IntFlag=0;
	 }//IntFlag
    }//while(1)
 
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



