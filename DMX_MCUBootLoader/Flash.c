#include "iom128v.h"
#include "macros.h"
#include "comm_def.h"

unsigned char z,x,a;
unsigned char t[8];

//unsigned char well_block[80];  //��N�ھ�����ʼ��
unsigned char well_counter; //���ж��ٿھ�
unsigned char num;

//  7   6   5   4   3   2    1     0
//  NC  NC /WE ALE CLE /CE  /RE  RDY/B
// A0~A7 �е�ַ������ҳ���ֽ�������ÿ����ҳ256�ֽڣ���ҳ512�ֽ�
// A8~A22 �е�ַ����ҳ����, ��65536ҳ, ÿ32ҳ��һ��, ��2048��

//����4��������µ�·ͼ����,ǰ���������⴦��
#define ADD_W(a)     {DDRA=0xff;PORTA=a;PORTC=0xf3;PORTC=0xd3;PORTC=0xf3;}
#define COM_W(a)     {DDRA=0xff;PORTA=a;PORTC=0xeb;PORTC=0xcb;PORTC=0xeb;}
//F�ڱ�������Ӧ�����ڲ������������һ�ֽڲ�������
#define DATA_W(a)    {DDRA=0xff;PORTA=a;PORTC=0xe3;PORTC=0xc3;PORTC=0xe3;}
#define DATA_R(a)    {DDRA=0x00;PORTA=0xff;PORTC=0xe3;PORTC=0xe1;PORTC=0xe1;PORTC=0xe1;a=PINA;PORTC=0xe3;}

#define CS_OFF       PORTC|=0xe7;
//Ӳ���Ķ�ʱ����Ӧ��Ӧ�Ķ�
#define BUSY         asm("nop");asm("nop");asm("nop");while((PINC&0x01)==0x00);



//����FLASH������غ���
unsigned  int FlashID(void);
unsigned char FlashDataWrite(unsigned int Shift,unsigned int PageIndex,volatile unsigned char * Dat,unsigned int Len);
void FlashDataRead(unsigned int Shift,unsigned int PageIndex,unsigned char * Dat,unsigned int Len);
unsigned char EraseBlock(unsigned int Index);


//�ⲿ���ú���
extern void delay(unsigned int t);

unsigned  int FlashID(void)
{unsigned int i; 
 unsigned char a;
 COM_W(0x90)
 ADD_W(0x00)

 DATA_R(a)   //a=0xEC
 i=a<<8; 
 DATA_R(a)  //a=0x75
 i=i+a;
 
 CS_OFF
 
 return i;
}

unsigned char FlashDataWrite(unsigned int Shift,unsigned int PageIndex,volatile unsigned char * Dat,unsigned int Len)
{unsigned int i;
 unsigned char a;
 COM_W(Shift>>8)       //ҳ�����ʼ��ַ
 COM_W(0x80)           //ҳ��̿�ʼ����
 ADD_W(Shift)          //ҳ��ƫ��
 ADD_W(PageIndex)      //ҳ������ַ��
 ADD_W(PageIndex>>8)   //ҳ������ַ��
 BUSY
 

 for(i=0;i<Len;i++)
 {DATA_W(*Dat++)
 }
 COM_W(0x10)
 BUSY
 
 COM_W(0x70)
 DATA_R(a)

 CS_OFF

 if((a&0x01)==0)
  return 0;    //ҳ��̳ɹ�
 else
  return 1;   //ҳ���ʧ��
}


void FlashDataRead(unsigned int Shift,unsigned int PageIndex,unsigned char * Dat,unsigned int Len)
{unsigned int i;
 COM_W(Shift>>8)//������ֶ������
 ADD_W(Shift) //ҳ��ƫ��
 ADD_W(PageIndex) //ҳ�����͵�ַ
 ADD_W(PageIndex>>8) //ҳ�����ߵ�ַ
 
 // delay(10);
  BUSY
  for(i=0;i<Len;i++) 
   {DATA_R(*Dat++)
   }
   
   CS_OFF
}




unsigned char EraseBlock(unsigned int Index)
{unsigned char a,b;
 unsigned int i;
 Index*=32;
 a=Index;
 b=Index>>8;
 COM_W(0x60)
 ADD_W(a)
 ADD_W(b)
 COM_W(0xd0)
 
 BUSY
 
 COM_W(0x70)
 DATA_R(a)
 CS_OFF
 
 if((a&0x01)==0)
  return 0;      //ҳ��̳ɹ�
 else
  return 1;    //ҳ���ʧ��
}



unsigned char FlashDataWrite2(unsigned int Shift,unsigned int PageIndex,const unsigned char * Dat,unsigned int Len)
{unsigned int i;
 unsigned char a;
 COM_W(Shift>>8)       //ҳ�����ʼ��ַ
 COM_W(0x80)           //ҳ��̿�ʼ����
 ADD_W(Shift)          //ҳ��ƫ��
 ADD_W(PageIndex)      //ҳ������ַ��
 ADD_W(PageIndex>>8)   //ҳ������ַ��
 
 BUSY
 
 DDRA=0xff;
 for(i=0;i<Len;i++)
 {
 PORTA=*Dat++;
 PORTC=0xe3;
 PORTC=0xc3;
 PORTC=0xe3;
 }
 COM_W(0x10)
 
 BUSY
 
 COM_W(0x70)
 DATA_R(a)

 CS_OFF
 
 if((a&0x01)==0)
  return 0;    //ҳ��̳ɹ�
 else
  return 1;   //ҳ���ʧ��
}

