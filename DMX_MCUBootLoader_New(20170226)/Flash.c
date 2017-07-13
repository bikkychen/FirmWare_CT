#include "iom128v.h"
#include "macros.h"
#include "comm_def.h"

unsigned char z,x,a;
unsigned char t[8];

//unsigned char well_block[80];  //第N口井的起始块
unsigned char well_counter; //共有多少口井
unsigned char num;

//  7   6   5   4   3   2    1     0
//  NC  NC /WE ALE CLE /CE  /RE  RDY/B
// A0~A7 列地址，即半页内字节索引，每个半页256字节，整页512字节
// A8~A22 行地址，即页索引, 共65536页, 每32页称一块, 共2048块

//以下4行需根据新电路图更改,前二行作特殊处理
#define ADD_W(a)     {DDRA=0xff;PORTA=a;PORTC=0xf3;PORTC=0xd3;PORTC=0xf3;}
#define COM_W(a)     {DDRA=0xff;PORTA=a;PORTC=0xeb;PORTC=0xcb;PORTC=0xeb;}
//F口变低输入后，应将其内部上拉，否则第一字节不能输入
#define DATA_W(a)    {DDRA=0xff;PORTA=a;PORTC=0xe3;PORTC=0xc3;PORTC=0xe3;}
#define DATA_R(a)    {DDRA=0x00;PORTA=0xff;PORTC=0xe3;PORTC=0xe1;PORTC=0xe1;PORTC=0xe1;a=PINA;PORTC=0xe3;}

#define CS_OFF       PORTC|=0xe7;
//硬件改动时下行应相应改动
#define BUSY         asm("nop");asm("nop");asm("nop");while((PINC&0x01)==0x00);



//外扩FLASH操作相关函数
unsigned  int FlashID(void);
unsigned char FlashDataWrite(unsigned int Shift,unsigned int PageIndex,volatile unsigned char * Dat,unsigned int Len);
void FlashDataRead(unsigned int Shift,unsigned int PageIndex,unsigned char * Dat,unsigned int Len);
unsigned char EraseBlock(unsigned int Index);


//外部引用函数
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
 COM_W(Shift>>8)       //页编程起始地址
 COM_W(0x80)           //页编程开始命令
 ADD_W(Shift)          //页内偏移
 ADD_W(PageIndex)      //页索引地址低
 ADD_W(PageIndex>>8)   //页索引地址高
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
  return 0;    //页编程成功
 else
  return 1;   //页编程失败
}


void FlashDataRead(unsigned int Shift,unsigned int PageIndex,unsigned char * Dat,unsigned int Len)
{unsigned int i;
 COM_W(Shift>>8)//读命令，分二种情况
 ADD_W(Shift) //页内偏移
 ADD_W(PageIndex) //页索引低地址
 ADD_W(PageIndex>>8) //页索引高地址
 
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
  return 0;      //页编程成功
 else
  return 1;    //页编程失败
}



unsigned char FlashDataWrite2(unsigned int Shift,unsigned int PageIndex,const unsigned char * Dat,unsigned int Len)
{unsigned int i;
 unsigned char a;
 COM_W(Shift>>8)       //页编程起始地址
 COM_W(0x80)           //页编程开始命令
 ADD_W(Shift)          //页内偏移
 ADD_W(PageIndex)      //页索引地址低
 ADD_W(PageIndex>>8)   //页索引地址高
 
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
  return 0;    //页编程成功
 else
  return 1;   //页编程失败
}

