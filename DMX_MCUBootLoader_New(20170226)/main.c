


#include <macros.h>
#include <stdio.h>
#include	"HAL.H"			// ������Ƭ�����޸�HAL*Ӳ�������ļ����ļ�

#define DBG 0


unsigned int flash_bufferPoint; 
char flash_buf[256]; //FALSHҳ������,M128��һ��FlashҳΪ256�ֽ�(128��) 
long address = 0; 
   

unsigned char tx_data[64];
unsigned char UpdateFlag=0;



extern void USBStart(void);
extern void EP2SendConst(UINT8 mLen, const unsigned char  *mBuf);
extern unsigned char	UsbConfig ,UsbReset;	 

//USB���ջ��������ݽṹ
typedef struct RXDATA
{
 unsigned char flag;
 unsigned char len;
 unsigned char buf[64];
};
struct RXDATA RxData;


 
unsigned char UpdateBegin;//�������������ݿ�ʼ��־
unsigned int DataReCn;
unsigned char crc16hi,crc16lo,DownloadSpeed;
unsigned int Timer3Cn;
	   
extern void	Init374Device( void );  // ��ʼ��USB�豸
extern void EP2Send(UINT8 mLen, PUINT8 mBuf);
extern unsigned char	UsbConfig ;	// USB���ñ�־

void DelayMs(unsigned int t);

//OSC=8M Hz
//UART0 initialize
// desired baud rate: 38400
// actual: baud rate:37500 (2.4%)
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x0B; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x18;
}



void port_init(void)
{
 PORTA = 0x00; 
 DDRA  = 0x00; 
 
 PORTB = 0x10; 
 DDRB  = 0x10; 
 
 PORTC = 0x00; 
 DDRC  = 0x00;
 
 PORTD = 0x00; 
 DDRD  = 0x00; 
 
 PORTE = 0x00;
 DDRE  = 0x00;
 
 PORTF = 0x00; 
 DDRF  = 0x00; 
 
 PORTG = 0x00;
 DDRG  = 0x00;
}

void DelayMs(unsigned int t)
{
   float f;
   unsigned int n;
   if(t>8388)
     t=8388;
   f=t;
   f*=15.625;
   n=f;
   TCNT1=65536-n;
   TCCR1B=0x05;//������ʱ����1024��Ƶ
   TIFR|=0x04;//�嶨ʱ��1�жϱ�־
   while((TIFR&0x04)==0x00);
   TCCR1B=0x00;//ֹͣ��ʱ��
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
//call this routine to initialize all peripherals
void init_devices(void)
{
 //stop errant interrupts until set up
 CLI(); //disable all interrupts
 XDIV  = 0x00; //xtal divider
 XMCRA = 0x00; //external memory
 port_init();
#if DBG
 uart0_init();
#endif

 MCUCR = 0x00;
 EICRA = 0x00; //extended ext ints
 EICRB = 0x00; //extended ext ints
 EIMSK = 0x00;
 TIMSK = 0x00; //timer interrupt sources
 ETIMSK = 0x00; //extended timer interrupt sources
 SEI(); //re-enable interrupts
 //all peripherals are now initialized
}


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

/*
//����һ��Flashҳ���������� 
void write_one_page(void) 
{ 
    int i; 
    boot_page_ew(address,0x03);                    //����һ��Flashҳ 
    wait_page_rw_ok();                            //�ȴ�������� 
	 
    for(i=0;i<256;i+=2)                //����������Flash����ҳ�� 
    { 
        boot_page_fill(i, flash_buf[i]+(flash_buf[i+1]<<8)); 
    } 
    boot_page_ew(address,0x05);                    //������ҳ����д��һ��Flashҳ 
    wait_page_rw_ok();                            //�ȴ�д����� 
}        
*/

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


void delay(unsigned int t) //y=x+3(us)
{
 while(t--);
}
 

void quit(void) 
{
 	 UCSR0B = 0x00; //disable 
	 
    MCUCR = 0x01; 
    MCUCR = 0x00;       //���ж�������Ǩ�Ƶ�Ӧ�ó�����ͷ�� 
    RAMPZ = 0x00;       //RAMPZ�����ʼ�� 
    asm("jmp 0x0000\n");//��ת��Flash��0x0000����ִ���û���Ӧ�ó��� 
} 

//������ 
void main(void) 
{ 
   unsigned int i ; 
   unsigned char j;
   unsigned char int0,miso;
  
   delay(1000);
	  
   init_devices();

   RxData.flag=0;
 
 
	 
     
   //�ڲ������ⲿ����ʱ�����������󣬿���������ߣ�5us����
   SFIOR&=0xfb;   //��������ֹ,bit2=0
   
   PORTB|=0x08;  //PB3�ڲ�����
   DDRB&=0xf7;   //PB3��Ϊ���� 
   
   PORTD|=0x01;  //PD0�ڲ�����
   DDRD&=0xfe;   //PD0��Ϊ���� 
   
   SFIOR&=0xfb;   //��������ֹ,bit2=0
   
   PORTB|=0x08;  //PB3�ڲ�����
   DDRB&=0xf7;   //PB3��Ϊ����   
   PORTB|=0x08;  //PB3�ڲ�����
   
   PORTD|=0x01;  //PD0�ڲ�����
   DDRD&=0xfe;   //PD0��Ϊ���� 
   PORTD|=0x01;  //PD0�ڲ�����
   
   DelayMs(100);
   
 
 
      CLI();
      EIMSK = 0x41;
	  SEI();
   #if DBG
      printf("Communication Status\r\n");
   #endif
      RxData.flag=0;
   
      //�ڲ������ⲿ����ʱ�����������󣬿����������
	  //��ʼ��SPI����
	  
	  
	  DDRB|=0x07;//PB0/1/2��Ϊ��� 
      DelayMs(30);
      SPCR = 0x00; //SPI�����Ƚ�ֹ����ʹ�ܣ������ʹ�ܲ��ɹ�
	  DelayMs(30);
	  /*
	  SPCR = 0x5d; //setup SPICLK=F0sc/16
	  SPSR = 0x00; //setup SPIʱ��Ƶ�ʲ�˫��
	  SPCR = 0x5d; //setup SPICLK=F0sc/16
	  */
	  
	  SPCR = 0x5f; //setup SPI SPICLK=Fosc/128
	  SPSR = 0x01; //setup SPI SPIʱ��Ƶ��˫��
	  SPCR = 0x5f; //setup SPI SPICLK=Fosc/128
	  
	  address=0 ;
      flash_bufferPoint=0; 
      UpdateFlag=0;
	  
	  USBStart();
	  
	#if DBG   
	  printf("Init374Device Begin\r\n");
	#endif
	  
	 for(j=0;j<100;j++)	  //����3�룬�ȴ�PC������USB���������ǰ�װ���������ܳ�ʱ���ƣ�ֱ��������װ���
	 {
	  while(UsbReset==1)
	   {
	    if(UsbConfig == 1)
		 {
		  break;
		 }
	   }
	  if(UsbConfig == 1) 
	    {
	     break; //�ȴ�USB������װ�ɹ�
		}
	  DelayMs(30); 
	  }
	 

	 if(j>=100)//�ڹ涨ʱ����û�м���������˵������ط���û�����ϵ��ԣ���ֱ�ӽ��뵽����״̬
	  {
	     quit();
	  }
	  
	 
	  #if DBG
	  printf("Init374Device End\r\n");
	  #endif
	  
	 
	  #if DBG
	  printf("UsbConfig OK\r\n");
	  #endif
       
	  UCSR0B = 0x00; //disable 
	   
	 TCCR3B = 0x00; //stop    
	 TCNT3H = 0x48; //16M,3s
      TCNT3L = 0xE5;
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
	ETIMSK &= 0xfb;//�ض�ʱ��3�ж�
 	TCCR3B = 0x05; //1024��Ƶ
	ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
	    
	UpdateBegin=0;
	while( ((ETIFR&0x04)==0x00) && (UpdateBegin==0) )
	{
	 if((RxData.flag==1))//3�����յ���ʼ��������
	 {  
	  if(RxData.buf[4]==0x01)
	   {
	    		tx_data[0]=0xe7;  //֡ͷ
				tx_data[1]=0xe7; //֡ͷ
				tx_data[2]=0x00; //��ַ1
				tx_data[3]=0x00;   //��ַ2
				tx_data[4]=RxData.buf[4]; //����
				tx_data[5]=0x02; //���ݳ��ȵ��ֽ�
				tx_data[6]=0x00; //���ݳ��ȸ��ֽ�
				tx_data[7]=0x01; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //У��
				tx_data[15]=0x00; //У��
      			EP2Send(16, tx_data);
 				UpdateBegin=1;
	   }  	
	   else  //�յ���������һ���˳�����
	   {
	   		 TCCR3B = 0x00; //stop    
			 ETIFR|=0x04; //�嶨ʱ��3�жϱ�־
			 ETIMSK &= 0xfb;//�ض�ʱ��3�ж�
	
	     		tx_data[0]=0xe7;  //֡ͷ
				tx_data[1]=0xe7; //֡ͷ
				tx_data[2]=0x00; //��ַ1
				tx_data[3]=0x00;   //��ַ2
				tx_data[4]=RxData.buf[4]; //����
				tx_data[5]=0x02; //���ݳ��ȵ��ֽ�
				tx_data[6]=0x00; //���ݳ��ȸ��ֽ�
				tx_data[7]=RxData.buf[4]; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //У��
				tx_data[15]=0x00; //У��
      			EP2Send(16, tx_data);		
		 		quit();
	   } 
	   RxData.flag=0;  
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
 	  if(RxData.flag==1)//���յ�����������֡
   	  {
	 	 if(UpdateBegin>0)//�������������������
     	 { 	  
		      SetTimer3();	 //ÿ�յ�һ�ֽھ����¿�ʼ��ʱ500ms����ʱ��ص�����״̬
			  
		      flash_buf[DataReCn]=RxData.buf[4];//�����ֽڵ������ֽ�
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
			   
			   crc16(flash_buf,258);//12ms
		        		    
				tx_data[0]=0xe7;  //֡ͷ
				tx_data[1]=0xe7; //֡ͷ
				tx_data[2]=0x00; //��ַ1
				tx_data[3]=0x00;   //��ַ2
				tx_data[4]=02; //�������ҳд׼����������λ����������������ֽ�
				tx_data[5]=0x02; //���ݳ��ȵ��ֽ�
				tx_data[6]=0x00; //���ݳ��ȸ��ֽ�
				tx_data[7]=crc16lo; 
				tx_data[8]=crc16hi; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //У��
				tx_data[15]=0x00; //У��
      			EP2Send(16, tx_data);	 //����һҳ��У�����ϴ�����λ������λ���жϺ�����Ǽ�����дһҳ����������һҳ	 		        
			 }	 	 
	     }
  		 else//����������״̬����������¸�������������
		 {
    		switch(RxData.buf[4]) 
     		{		
			  case 0x00://�����ǰ汾
				tx_data[0]=0xe7;  //֡ͷ
				tx_data[1]=0xe7; //֡ͷ
				tx_data[2]=0x00; //��ַ1
				tx_data[3]=0x00;   //��ַ2
				tx_data[4]=RxData.buf[4]; //����
				tx_data[5]=0x01; //���ݳ��ȵ��ֽ�
				tx_data[6]=0x00; //���ݳ��ȸ��ֽ�
				tx_data[7]=0x00; 
				tx_data[8]=0x00; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //У��
				tx_data[15]=0x00; //У��
      			EP2Send(16, tx_data);	
				break;
				
			   case 0x01://���������֣��������״̬  			      			       
	  			tx_data[0]=0xe7;  //֡ͷ
				tx_data[1]=0xe7; //֡ͷ
				tx_data[2]=0x00; //��ַ1
				tx_data[3]=0x00;   //��ַ2
				tx_data[4]=RxData.buf[4]; //����
				tx_data[5]=0x02; //���ݳ��ȵ��ֽ�
				tx_data[6]=0x00; //���ݳ��ȸ��ֽ�
				tx_data[7]=0x01; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //У��
				tx_data[15]=0x00; //У��
      			EP2Send(16, tx_data);
				break;
						 		   			
			case 0x02://������ҳд׼��		
				UpdateBegin=1;//ҳд׼������ҳ����0��ʼ��д�����ҳ����255����64K�ֽڳ���ռ�
				DataReCn=0;    
				SetTimer3();//��ʼ��ʱ��ÿ500ms������Ҫ��һ���ֽ��յ�
				
				tx_data[0]=0xe7;  //֡ͷ
				tx_data[1]=0xe7; //֡ͷ
				tx_data[2]=0x00; //��ַ1
				tx_data[3]=0x00;   //��ַ2
				tx_data[4]=RxData.buf[4]; //����
				tx_data[5]=0x02; //���ݳ��ȵ��ֽ�
				tx_data[6]=0x00; //���ݳ��ȸ��ֽ�
				tx_data[7]=0x02; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //У��
				tx_data[15]=0x00; //У��
      			EP2Send(16, tx_data);			
				break;
											
				
			case 0x03://�������˳�����״̬��������������
				tx_data[0]=0xe7;  //֡ͷ
				tx_data[1]=0xe7; //֡ͷ
				tx_data[2]=0x00; //��ַ1
				tx_data[3]=0x00;   //��ַ2
				tx_data[4]=RxData.buf[4]; //����
				tx_data[5]=0x02; //���ݳ��ȵ��ֽ�
				tx_data[6]=0x00; //���ݳ��ȸ��ֽ�
				tx_data[7]=0x03; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //У��
				tx_data[15]=0x00; //У��
      			EP2Send(16, tx_data);	
				 quit();
				break;

				default:
				break;
	  	    }//switch
	     }//UpdateBegin
		 RxData.flag=0;
	 }//RxData.flag
    }//while(1)
      
	 
} 