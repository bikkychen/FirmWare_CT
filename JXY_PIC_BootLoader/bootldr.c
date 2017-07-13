#include <p18f4550.h>

#define RX_TRIS TRISCbits.TRISC7//���ڶ˿�
#define TX_TRIS TRISCbits.TRISC6

#define BOOT_TIMEOUT  1       //������ʱʱ��/��
#define PROG_START    0x800   //�û�����ʼλ��
#define PAGESIZE      256
#define MEM_TOP       0x8000       //FLASH���洢�ռ�

void Write_One_Byte(unsigned  long address, unsigned char data);
void Flash_Write64Bytes(unsigned  long address, unsigned char *Pdata);
void Flash_Erease64Bytes(unsigned  long address);
void SetTimer(void);

#pragma udata gg
 unsigned char buff[256];  //��̻����� //
#pragma udata

 unsigned char dataflag,rectdat;
 unsigned int  bcount;
 unsigned char delay_time;
 unsigned  long flashaddr; 

#pragma code High_Interrupt_Vector=0X08
void high_ISR(void)
{
	_asm
	goto   PROG_START+0x08
	_endasm
}
#pragma code

#pragma code Low_Interrupt_Vector=0X18
void low_ISR(void)
{
	_asm
	goto    PROG_START+0x18
	_endasm
}
#pragma code
 
/*
#pragma code bbb=0x800
void aaa(void)
{
 int a;
a=1;
a++;
a--;
}
#pragma code
*/

// д����
void Write_Cycle(void)
{unsigned char flag=0;

	EECON1bits.EEPGD = 1;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	
	if(INTCONbits.GIE)//��ֹȫ���ж�
	{
		INTCONbits.GIE = 0;
		flag=1;
	}
	
	EECON2 = 0X55;
	EECON2 = 0XAA;
	EECON1bits.WR = 1;
	Nop();
	Nop();
	while (EECON1bits.WR);	
	EECON1bits.WREN = 0;
	
    if(flag){INTCONbits.GIE = 1;}//����ȫ���ж�
}

 

//д������ÿ��д��64�ֽ�
void Flash_Write64Bytes(unsigned  long address, unsigned char *Pdata)
{
	unsigned char count,i;
	unsigned char length;
	
	Flash_Erease64Bytes(address);
	
	for (count=0; count<8; count++)
	{		
		TBLPTRU = 0;//( ( (address + count * 8)>>8)>>8);
		
		for (length=0; length<8; length++)
		{
			TBLPTRL = ( (address + count * 8 + length) & 0xFF);
			TBLPTRH = ( ( (address + count * 8 + length)>>8) & 0XFF);
			TABLAT = *(Pdata + length + count * 8);
                  _asm
                  tblwt
			_endasm
/*
			if (length != 0)
			{
				_asm
				TBLWTPREINC
				_endasm
			}
			else
			{
				//asm("\tTBLWT*");
			}*/			
		}
		
		EECON1bits.FREE = 0;
		Write_Cycle();
	}
}

void Flash_Erease64Bytes(unsigned  long address)//��Լ��ʱ40ms
{ 
  unsigned char flag=0;

  if(address<PROG_START)
  {return ;}

 if(address>=MEM_TOP)
  {return ;}

  // ��������ռ� //
    TBLPTRU=0;
    TBLPTRL=(unsigned char)address;//������ʼ��ַ
    TBLPTRH=(unsigned char)(address>>8);

     EECON1bits.EEPGD=1;//ָ�����洢������FLASH
     EECON1bits.CFGS=0;//����FLASH��EEPROM    
     EECON1bits.WREN = 1;//����д
     EECON1bits.FREE = 1; //�������
     INTCONbits.GIE = 0;
     if(INTCONbits.GIE)//��ֹȫ���ж�
	{
		INTCONbits.GIE = 0;
		flag=1;
	}
     EECON2 = 0x55;
     EECON2 = 0xAA;
     EECON1bits.WR = 1;

    while(EECON1bits.WR);//��ѯд�����Ƿ���ɣ���ɺ�Ӳ���Զ�����    
    EECON1bits.WREN = 0;

    if(flag){INTCONbits.GIE = 1;}//����ȫ���ж�
}
 


void SetTimer(void)
{
	T1CON=0XCE;          //ʹ��TIMER1����,0X4EԤ��Ƶ1��1��0X7EԤ��Ƶ8��1���첽ģʽ;	
	
	PIE1bits.TMR1IE=0;   //������Timer1�ж�	
      PIR1bits.TMR1IF=0;	//����TIMER1�жϱ�־
	
//	TMR1H=0xF0;//��ʱ������ֵ��1/8 s
	TMR1H=0x80;//��ʱ1��
	TMR1L=0x00;		
		
	T1CONbits.TMR1ON=1;//����TMR1
}

void main(void)
{  int i;
    // ��ʼ������
     RX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	TX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	SPBRGH=0;				//������57600(4MHz)
	//SPBRG=16;				// 16=115200(8MHz)
	SPBRG=34;				// 57600 (8MHZ)
	BAUDCON=0X08;			       //16λ�����ʷ���������ֹ�����ʼ�⣬δ����RX����
	TXSTA=0X24;				//8λ���ͣ�ʹ�ܷ��ͣ������첽ģʽ
	RCSTA=0X90;				//8λ���գ�ʹ�ܽ���     
    TRISB&=0xDF;//��ֹ������Ҫ������RB5=0��Ӳ��������б�֤�Ĵ��п�ʡ��

  // ĳЩ�ͺŵ����⣬��Ҫ�����Щ�Ĵ���
  INTCON3=0;  // Also serves to disable interrupts
  PIE2=0;
  INTCON=0;
/*
 for(i=0;i<256;i++)
   buff[i]=i;
   Flash_Write64Bytes(0x840,&buff[0]);
  while(1);
 */
  //ʹ�ó�ʱ�жϷ�ʽ
  //��ָ��ʱ�����յ����ݣ�������״̬������������ǰ�ĳ���

  SetTimer(); // ʹ�ö�ʱ����ʱ1��
  delay_time=0;
  while(1)
  {
    if (PIR1bits.RCIF)   //���ܵ���������
    {
      if(RCREG==0xf9)
      {
         TXREG = 0xf9;//�����ذ��Ӧһ�ֽ�
         break;//�����ɹ������ٵȴ�
       }
    }

   if(PIR1bits.TMR1IF)//�ȴ�1�붨ʱ��־
    {
      PIR1bits.TMR1IF=0;	//����TIMER1�жϱ�־
      SetTimer(); // ʹ�ö�ʱ����ʱ1��
      delay_time++;
    }

    if(delay_time==BOOT_TIMEOUT)
     {break;}  
  }
  PIR1bits.TMR1IF=0;	//����TIMER1�жϱ�־ 
  T1CONbits.TMR1ON=0;//ֹͣTMR1


  if (delay_time==BOOT_TIMEOUT)//����ʧ��
  {
    (*((void(*)(void))PROG_START))(); //ֱ�ӽ����û�����
  }

  dataflag=0;//��ʼΪ����״̬

// ͨ�����ڽ������������ //
  for(;;)   // ѭ��
  {
	   while(!PIR1bits.RCIF)// �ȴ��������ݻ�����
	   {
 		if(PIR1bits.TMR1IF)  //�ȴ�1�붨ʱ��־
	      {
	       PIR1bits.TMR1IF=0;	//����TIMER1�жϱ�־ 
             T1CONbits.TMR1ON=0;//ֹͣTMR1
	       dataflag=0;//�л�Ϊ����״̬
	      }
	   };       
	   
         SetTimer();

	   rectdat = RCREG;;          // ȡ���ݻ�����

	   if(dataflag==1)//����״̬
	   {
             bcount++;
		if(bcount==1)
             {flashaddr=rectdat;
		  flashaddr<<=8;}
            else if(bcount==2)
             {flashaddr|=rectdat;//ҳ����
              flashaddr*=(PAGESIZE);// ÿҳ�����ֽ�
		  flashaddr+=PROG_START;
		  }
            else
             {
              buff[bcount-3]=rectdat;
             }
              
            if(bcount==258)
             {
              for(i=0;i<4;i++)
              {
		  Flash_Write64Bytes(flashaddr+64*i,&buff[i*64]);
              }
              TXREG = 0xf8;   
              bcount=0;
              dataflag=0;//�л�Ϊ����״̬
             }
            
	   }
	   else//����״̬
	   {
		switch(rectdat)
		    {
		     	case 0xf9://PIC����
			TXREG = 0xf9;
			break;
		
			case 0xf8://PICҳд׼��
			TXREG = 0xf8;
			dataflag=1;//�л�Ϊ����״̬
			bcount = 0;//���ݽ��ռ�������
			break;

			case 0xf7://PIC�汾
			TXREG = 0x00;
			break;
	
		      case 0xb8: // �˳�����״̬
		        TXREG = 0xe7;//�����ذ��Ӧһ�ֽ�0xe7
		        (*((void(*)(void))PROG_START))(); //�����³���
		      break;
		    }
	   }   
  }
}

