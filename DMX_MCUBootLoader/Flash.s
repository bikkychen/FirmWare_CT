	.module Flash.c
	.area text(rom, con, rel)
	.dbfile Flash.c
	.dbfile D:\Bikky\JDYQ\USB_Load\Flash.c
	.dbfunc e FlashID _FlashID fi
;              a -> R18
;              i -> R16,R17
	.even
_FlashID::
	.dbline -1
	.dbline 41
; #include "iom128v.h"
; #include "macros.h"
; #include "comm_def.h"
; 
; unsigned char z,x,a;
; unsigned char t[8];
; 
; //unsigned char well_block[80];  //第N口井的起始块
; unsigned char well_counter; //共有多少口井
; unsigned char num;
; 
; //  7   6   5   4   3   2    1     0
; //  NC  NC /WE ALE CLE /CE  /RE  RDY/B
; // A0~A7 列地址，即半页内字节索引，每个半页256字节，整页512字节
; // A8~A22 行地址，即页索引, 共65536页, 每32页称一块, 共2048块
; 
; //以下4行需根据新电路图更改,前二行作特殊处理
; #define ADD_W(a)     {DDRA=0xff;PORTA=a;PORTC=0xf3;PORTC=0xd3;PORTC=0xf3;}
; #define COM_W(a)     {DDRA=0xff;PORTA=a;PORTC=0xeb;PORTC=0xcb;PORTC=0xeb;}
; //F口变低输入后，应将其内部上拉，否则第一字节不能输入
; #define DATA_W(a)    {DDRA=0xff;PORTA=a;PORTC=0xe3;PORTC=0xc3;PORTC=0xe3;}
; #define DATA_R(a)    {DDRA=0x00;PORTA=0xff;PORTC=0xe3;PORTC=0xe1;PORTC=0xe1;PORTC=0xe1;a=PINA;PORTC=0xe3;}
; 
; #define CS_OFF       PORTC|=0xe7;
; //硬件改动时下行应相应改动
; #define BUSY         asm("nop");asm("nop");asm("nop");while((PINC&0x01)==0x00);
; 
; 
; 
; //外扩FLASH操作相关函数
; unsigned  int FlashID(void);
; unsigned char FlashDataWrite(unsigned int Shift,unsigned int PageIndex,volatile unsigned char * Dat,unsigned int Len);
; void FlashDataRead(unsigned int Shift,unsigned int PageIndex,unsigned char * Dat,unsigned int Len);
; unsigned char EraseBlock(unsigned int Index);
; 
; 
; //外部引用函数
; extern void delay(unsigned int t);
; 
; unsigned  int FlashID(void)
; {unsigned int i; 
	.dbline 43
;  unsigned char a;
;  COM_W(0x90)
	.dbline 43
	ldi R24,255
	out 0x1a,R24
	.dbline 43
	ldi R24,144
	out 0x1b,R24
	.dbline 43
	ldi R24,235
	out 0x15,R24
	.dbline 43
	ldi R24,203
	out 0x15,R24
	.dbline 43
	ldi R24,235
	out 0x15,R24
	.dbline 43
	.dbline 44
;  ADD_W(0x00)
	.dbline 44
	ldi R24,255
	out 0x1a,R24
	.dbline 44
	clr R2
	out 0x1b,R2
	.dbline 44
	ldi R24,243
	out 0x15,R24
	.dbline 44
	ldi R24,211
	out 0x15,R24
	.dbline 44
	ldi R24,243
	out 0x15,R24
	.dbline 44
	.dbline 46
; 
;  DATA_R(a)   //a=0xEC
	.dbline 46
	out 0x1a,R2
	.dbline 46
	ldi R24,255
	out 0x1b,R24
	.dbline 46
	ldi R24,227
	out 0x15,R24
	.dbline 46
	ldi R24,225
	out 0x15,R24
	.dbline 46
	out 0x15,R24
	.dbline 46
	out 0x15,R24
	.dbline 46
	in R18,0x19
	.dbline 46
	ldi R24,227
	out 0x15,R24
	.dbline 46
	.dbline 47
;  i=a<<8; 
	mov R16,R18
	clr R17
	mov R17,R16
	clr R16
	.dbline 48
;  DATA_R(a)  //a=0x75
	.dbline 48
	out 0x1a,R2
	.dbline 48
	ldi R24,255
	out 0x1b,R24
	.dbline 48
	ldi R24,227
	out 0x15,R24
	.dbline 48
	ldi R24,225
	out 0x15,R24
	.dbline 48
	out 0x15,R24
	.dbline 48
	out 0x15,R24
	.dbline 48
	in R18,0x19
	.dbline 48
	ldi R24,227
	out 0x15,R24
	.dbline 48
	.dbline 49
;  i=i+a;
	mov R2,R18
	clr R3
	add R16,R2
	adc R17,R3
	.dbline 51
;  
;  CS_OFF
	in R24,0x15
	ori R24,231
	out 0x15,R24
	.dbline 53
;  
;  return i;
	.dbline -2
L1:
	.dbline 0 ; func end
	ret
	.dbsym r a 18 c
	.dbsym r i 16 i
	.dbend
	.dbfunc e FlashDataWrite _FlashDataWrite fc
;              a -> R10
;              i -> R20,R21
;            Len -> y+6
;            Dat -> R10,R11
;      PageIndex -> R18,R19
;          Shift -> R16,R17
	.even
_FlashDataWrite::
	xcall push_xgset300C
	ldd R10,y+4
	ldd R11,y+5
	.dbline -1
	.dbline 57
; }
; 
; unsigned char FlashDataWrite(unsigned int Shift,unsigned int PageIndex,volatile unsigned char * Dat,unsigned int Len)
; {unsigned int i;
	.dbline 59
;  unsigned char a;
;  COM_W(Shift>>8)       //页编程起始地址
	.dbline 59
	ldi R24,255
	out 0x1a,R24
	.dbline 59
	movw R2,R16
	mov R2,R3
	clr R3
	out 0x1b,R2
	.dbline 59
	ldi R24,235
	out 0x15,R24
	.dbline 59
	ldi R24,203
	out 0x15,R24
	.dbline 59
	ldi R24,235
	out 0x15,R24
	.dbline 59
	.dbline 60
;  COM_W(0x80)           //页编程开始命令
	.dbline 60
	ldi R24,255
	out 0x1a,R24
	.dbline 60
	ldi R24,128
	out 0x1b,R24
	.dbline 60
	ldi R24,235
	out 0x15,R24
	.dbline 60
	ldi R24,203
	out 0x15,R24
	.dbline 60
	ldi R24,235
	out 0x15,R24
	.dbline 60
	.dbline 61
;  ADD_W(Shift)          //页内偏移
	.dbline 61
	ldi R24,255
	out 0x1a,R24
	.dbline 61
	out 0x1b,R16
	.dbline 61
	ldi R24,243
	out 0x15,R24
	.dbline 61
	ldi R24,211
	out 0x15,R24
	.dbline 61
	ldi R24,243
	out 0x15,R24
	.dbline 61
	.dbline 62
;  ADD_W(PageIndex)      //页索引地址低
	.dbline 62
	ldi R24,255
	out 0x1a,R24
	.dbline 62
	out 0x1b,R18
	.dbline 62
	ldi R24,243
	out 0x15,R24
	.dbline 62
	ldi R24,211
	out 0x15,R24
	.dbline 62
	ldi R24,243
	out 0x15,R24
	.dbline 62
	.dbline 63
;  ADD_W(PageIndex>>8)   //页索引地址高
	.dbline 63
	ldi R24,255
	out 0x1a,R24
	.dbline 63
	movw R2,R18
	mov R2,R3
	clr R3
	out 0x1b,R2
	.dbline 63
	ldi R24,243
	out 0x15,R24
	.dbline 63
	ldi R24,211
	out 0x15,R24
	.dbline 63
	ldi R24,243
	out 0x15,R24
	.dbline 63
	.dbline 64
;  BUSY
	nop
	.dbline 64
	nop
	.dbline 64
	nop
L3:
	.dbline 64
L4:
	.dbline 64
	sbis 0x13,0
	rjmp L3
X0:
	.dbline 67
;  
; 
;  for(i=0;i<Len;i++)
	clr R20
	clr R21
	xjmp L9
L6:
	.dbline 68
;  {DATA_W(*Dat++)
	.dbline 68
	.dbline 68
	ldi R24,255
	out 0x1a,R24
	.dbline 68
	movw R30,R10
	ld R2,Z+
	movw R10,R30
	out 0x1b,R2
	.dbline 68
	ldi R24,227
	out 0x15,R24
	.dbline 68
	ldi R24,195
	out 0x15,R24
	.dbline 68
	ldi R24,227
	out 0x15,R24
	.dbline 68
	.dbline 69
;  }
L7:
	.dbline 67
	subi R20,255  ; offset = 1
	sbci R21,255
L9:
	.dbline 67
	ldd R0,y+6
	ldd R1,y+7
	cp R20,R0
	cpc R21,R1
	brlo L6
X1:
	.dbline 70
;  COM_W(0x10)
	.dbline 70
	ldi R24,255
	out 0x1a,R24
	.dbline 70
	ldi R24,16
	out 0x1b,R24
	.dbline 70
	ldi R24,235
	out 0x15,R24
	.dbline 70
	ldi R24,203
	out 0x15,R24
	.dbline 70
	ldi R24,235
	out 0x15,R24
	.dbline 70
	.dbline 71
;  BUSY
	nop
	.dbline 71
	nop
	.dbline 71
	nop
L10:
	.dbline 71
L11:
	.dbline 71
	sbis 0x13,0
	rjmp L10
X2:
	.dbline 73
;  
;  COM_W(0x70)
	.dbline 73
	ldi R24,255
	out 0x1a,R24
	.dbline 73
	ldi R24,112
	out 0x1b,R24
	.dbline 73
	ldi R24,235
	out 0x15,R24
	.dbline 73
	ldi R24,203
	out 0x15,R24
	.dbline 73
	ldi R24,235
	out 0x15,R24
	.dbline 73
	.dbline 74
;  DATA_R(a)
	.dbline 74
	clr R2
	out 0x1a,R2
	.dbline 74
	ldi R24,255
	out 0x1b,R24
	.dbline 74
	ldi R24,227
	out 0x15,R24
	.dbline 74
	ldi R24,225
	out 0x15,R24
	.dbline 74
	out 0x15,R24
	.dbline 74
	out 0x15,R24
	.dbline 74
	in R10,0x19
	.dbline 74
	ldi R24,227
	out 0x15,R24
	.dbline 74
	.dbline 76
; 
;  CS_OFF
	in R24,0x15
	ori R24,231
	out 0x15,R24
	.dbline 78
; 
;  if((a&0x01)==0)
	sbrc R10,0
	rjmp L13
X3:
	.dbline 79
;   return 0;    //页编程成功
	clr R16
	xjmp L2
L13:
	.dbline 81
;  else
;   return 1;   //页编程失败
	ldi R16,1
	.dbline -2
L2:
	.dbline 0 ; func end
	xjmp pop_xgset300C
	.dbsym r a 10 c
	.dbsym r i 20 i
	.dbsym l Len 6 i
	.dbsym r Dat 10 pc
	.dbsym r PageIndex 18 i
	.dbsym r Shift 16 i
	.dbend
	.dbfunc e FlashDataRead _FlashDataRead fV
;              i -> R20,R21
;            Len -> y+6
;            Dat -> R10,R11
;      PageIndex -> R18,R19
;          Shift -> R16,R17
	.even
_FlashDataRead::
	xcall push_xgset300C
	ldd R10,y+4
	ldd R11,y+5
	.dbline -1
	.dbline 86
; }
; 
; 
; void FlashDataRead(unsigned int Shift,unsigned int PageIndex,unsigned char * Dat,unsigned int Len)
; {unsigned int i;
	.dbline 87
;  COM_W(Shift>>8)//读命令，分二种情况
	.dbline 87
	ldi R24,255
	out 0x1a,R24
	.dbline 87
	movw R2,R16
	mov R2,R3
	clr R3
	out 0x1b,R2
	.dbline 87
	ldi R24,235
	out 0x15,R24
	.dbline 87
	ldi R24,203
	out 0x15,R24
	.dbline 87
	ldi R24,235
	out 0x15,R24
	.dbline 87
	.dbline 88
;  ADD_W(Shift) //页内偏移
	.dbline 88
	ldi R24,255
	out 0x1a,R24
	.dbline 88
	out 0x1b,R16
	.dbline 88
	ldi R24,243
	out 0x15,R24
	.dbline 88
	ldi R24,211
	out 0x15,R24
	.dbline 88
	ldi R24,243
	out 0x15,R24
	.dbline 88
	.dbline 89
;  ADD_W(PageIndex) //页索引低地址
	.dbline 89
	ldi R24,255
	out 0x1a,R24
	.dbline 89
	out 0x1b,R18
	.dbline 89
	ldi R24,243
	out 0x15,R24
	.dbline 89
	ldi R24,211
	out 0x15,R24
	.dbline 89
	ldi R24,243
	out 0x15,R24
	.dbline 89
	.dbline 90
;  ADD_W(PageIndex>>8) //页索引高地址
	.dbline 90
	ldi R24,255
	out 0x1a,R24
	.dbline 90
	movw R2,R18
	mov R2,R3
	clr R3
	out 0x1b,R2
	.dbline 90
	ldi R24,243
	out 0x15,R24
	.dbline 90
	ldi R24,211
	out 0x15,R24
	.dbline 90
	ldi R24,243
	out 0x15,R24
	.dbline 90
	.dbline 93
;  
;  // delay(10);
;   BUSY
	nop
	.dbline 93
	nop
	.dbline 93
	nop
L16:
	.dbline 93
L17:
	.dbline 93
	sbis 0x13,0
	rjmp L16
X4:
	.dbline 94
;   for(i=0;i<Len;i++) 
	clr R20
	clr R21
	xjmp L22
L19:
	.dbline 95
;    {DATA_R(*Dat++)
	.dbline 95
	.dbline 95
	clr R2
	out 0x1a,R2
	.dbline 95
	ldi R24,255
	out 0x1b,R24
	.dbline 95
	ldi R24,227
	out 0x15,R24
	.dbline 95
	ldi R24,225
	out 0x15,R24
	.dbline 95
	out 0x15,R24
	.dbline 95
	out 0x15,R24
	.dbline 95
	in R2,0x19
	movw R30,R10
	st Z+,R2
	movw R10,R30
	.dbline 95
	ldi R24,227
	out 0x15,R24
	.dbline 95
	.dbline 96
;    }
L20:
	.dbline 94
	subi R20,255  ; offset = 1
	sbci R21,255
L22:
	.dbline 94
	ldd R0,y+6
	ldd R1,y+7
	cp R20,R0
	cpc R21,R1
	brlo L19
X5:
	.dbline 98
;    
;    CS_OFF
	in R24,0x15
	ori R24,231
	out 0x15,R24
	.dbline -2
L15:
	.dbline 0 ; func end
	xjmp pop_xgset300C
	.dbsym r i 20 i
	.dbsym l Len 6 i
	.dbsym r Dat 10 pc
	.dbsym r PageIndex 18 i
	.dbsym r Shift 16 i
	.dbend
	.dbfunc e EraseBlock _EraseBlock fc
;              i -> <dead>
;              b -> R12
;              a -> R10
;          Index -> R12,R13
	.even
_EraseBlock::
	xcall push_xgset003C
	movw R12,R16
	.dbline -1
	.dbline 105
; }
; 
; 
; 
; 
; unsigned char EraseBlock(unsigned int Index)
; {unsigned char a,b;
	.dbline 107
;  unsigned int i;
;  Index*=32;
	ldi R16,32
	ldi R17,0
	movw R18,R12
	xcall empy16s
	movw R12,R16
	.dbline 108
;  a=Index;
	mov R10,R12
	.dbline 109
;  b=Index>>8;
	mov R12,R13
	clr R13
	.dbline 110
;  COM_W(0x60)
	.dbline 110
	ldi R24,255
	out 0x1a,R24
	.dbline 110
	ldi R24,96
	out 0x1b,R24
	.dbline 110
	ldi R24,235
	out 0x15,R24
	.dbline 110
	ldi R24,203
	out 0x15,R24
	.dbline 110
	ldi R24,235
	out 0x15,R24
	.dbline 110
	.dbline 111
;  ADD_W(a)
	.dbline 111
	ldi R24,255
	out 0x1a,R24
	.dbline 111
	out 0x1b,R16
	.dbline 111
	ldi R24,243
	out 0x15,R24
	.dbline 111
	ldi R24,211
	out 0x15,R24
	.dbline 111
	ldi R24,243
	out 0x15,R24
	.dbline 111
	.dbline 112
;  ADD_W(b)
	.dbline 112
	ldi R24,255
	out 0x1a,R24
	.dbline 112
	out 0x1b,R12
	.dbline 112
	ldi R24,243
	out 0x15,R24
	.dbline 112
	ldi R24,211
	out 0x15,R24
	.dbline 112
	ldi R24,243
	out 0x15,R24
	.dbline 112
	.dbline 113
;  COM_W(0xd0)
	.dbline 113
	ldi R24,255
	out 0x1a,R24
	.dbline 113
	ldi R24,208
	out 0x1b,R24
	.dbline 113
	ldi R24,235
	out 0x15,R24
	.dbline 113
	ldi R24,203
	out 0x15,R24
	.dbline 113
	ldi R24,235
	out 0x15,R24
	.dbline 113
	.dbline 115
;  
;  BUSY
	nop
	.dbline 115
	nop
	.dbline 115
	nop
L24:
	.dbline 115
L25:
	.dbline 115
	sbis 0x13,0
	rjmp L24
X6:
	.dbline 117
;  
;  COM_W(0x70)
	.dbline 117
	ldi R24,255
	out 0x1a,R24
	.dbline 117
	ldi R24,112
	out 0x1b,R24
	.dbline 117
	ldi R24,235
	out 0x15,R24
	.dbline 117
	ldi R24,203
	out 0x15,R24
	.dbline 117
	ldi R24,235
	out 0x15,R24
	.dbline 117
	.dbline 118
;  DATA_R(a)
	.dbline 118
	clr R2
	out 0x1a,R2
	.dbline 118
	ldi R24,255
	out 0x1b,R24
	.dbline 118
	ldi R24,227
	out 0x15,R24
	.dbline 118
	ldi R24,225
	out 0x15,R24
	.dbline 118
	out 0x15,R24
	.dbline 118
	out 0x15,R24
	.dbline 118
	in R10,0x19
	.dbline 118
	ldi R24,227
	out 0x15,R24
	.dbline 118
	.dbline 119
;  CS_OFF
	in R24,0x15
	ori R24,231
	out 0x15,R24
	.dbline 121
;  
;  if((a&0x01)==0)
	sbrc R10,0
	rjmp L27
X7:
	.dbline 122
;   return 0;      //页编程成功
	clr R16
	xjmp L23
L27:
	.dbline 124
;  else
;   return 1;    //页编程失败
	ldi R16,1
	.dbline -2
L23:
	.dbline 0 ; func end
	xjmp pop_xgset003C
	.dbsym l i 1 i
	.dbsym r b 12 c
	.dbsym r a 10 c
	.dbsym r Index 12 i
	.dbend
	.dbfunc e FlashDataWrite2 _FlashDataWrite2 fc
;              a -> R10
;              i -> R20,R21
;            Len -> y+6
;            Dat -> R10,R11
;      PageIndex -> R18,R19
;          Shift -> R16,R17
	.even
_FlashDataWrite2::
	xcall push_xgset300C
	ldd R10,y+4
	ldd R11,y+5
	.dbline -1
	.dbline 130
; }
; 
; 
; 
; unsigned char FlashDataWrite2(unsigned int Shift,unsigned int PageIndex,const unsigned char * Dat,unsigned int Len)
; {unsigned int i;
	.dbline 132
;  unsigned char a;
;  COM_W(Shift>>8)       //页编程起始地址
	.dbline 132
	ldi R24,255
	out 0x1a,R24
	.dbline 132
	movw R2,R16
	mov R2,R3
	clr R3
	out 0x1b,R2
	.dbline 132
	ldi R24,235
	out 0x15,R24
	.dbline 132
	ldi R24,203
	out 0x15,R24
	.dbline 132
	ldi R24,235
	out 0x15,R24
	.dbline 132
	.dbline 133
;  COM_W(0x80)           //页编程开始命令
	.dbline 133
	ldi R24,255
	out 0x1a,R24
	.dbline 133
	ldi R24,128
	out 0x1b,R24
	.dbline 133
	ldi R24,235
	out 0x15,R24
	.dbline 133
	ldi R24,203
	out 0x15,R24
	.dbline 133
	ldi R24,235
	out 0x15,R24
	.dbline 133
	.dbline 134
;  ADD_W(Shift)          //页内偏移
	.dbline 134
	ldi R24,255
	out 0x1a,R24
	.dbline 134
	out 0x1b,R16
	.dbline 134
	ldi R24,243
	out 0x15,R24
	.dbline 134
	ldi R24,211
	out 0x15,R24
	.dbline 134
	ldi R24,243
	out 0x15,R24
	.dbline 134
	.dbline 135
;  ADD_W(PageIndex)      //页索引地址低
	.dbline 135
	ldi R24,255
	out 0x1a,R24
	.dbline 135
	out 0x1b,R18
	.dbline 135
	ldi R24,243
	out 0x15,R24
	.dbline 135
	ldi R24,211
	out 0x15,R24
	.dbline 135
	ldi R24,243
	out 0x15,R24
	.dbline 135
	.dbline 136
;  ADD_W(PageIndex>>8)   //页索引地址高
	.dbline 136
	ldi R24,255
	out 0x1a,R24
	.dbline 136
	movw R2,R18
	mov R2,R3
	clr R3
	out 0x1b,R2
	.dbline 136
	ldi R24,243
	out 0x15,R24
	.dbline 136
	ldi R24,211
	out 0x15,R24
	.dbline 136
	ldi R24,243
	out 0x15,R24
	.dbline 136
	.dbline 138
;  
;  BUSY
	nop
	.dbline 138
	nop
	.dbline 138
	nop
L30:
	.dbline 138
L31:
	.dbline 138
	sbis 0x13,0
	rjmp L30
X8:
	.dbline 140
;  
;  DDRA=0xff;
	ldi R24,255
	out 0x1a,R24
	.dbline 141
;  for(i=0;i<Len;i++)
	clr R20
	clr R21
	xjmp L36
L33:
	.dbline 142
;  {
	.dbline 143
;  PORTA=*Dat++;
	movw R30,R10
	ld R2,Z+
	movw R10,R30
	out 0x1b,R2
	.dbline 144
;  PORTC=0xe3;
	ldi R24,227
	out 0x15,R24
	.dbline 145
;  PORTC=0xc3;
	ldi R24,195
	out 0x15,R24
	.dbline 146
;  PORTC=0xe3;
	ldi R24,227
	out 0x15,R24
	.dbline 147
;  }
L34:
	.dbline 141
	subi R20,255  ; offset = 1
	sbci R21,255
L36:
	.dbline 141
	ldd R0,y+6
	ldd R1,y+7
	cp R20,R0
	cpc R21,R1
	brlo L33
X9:
	.dbline 148
;  COM_W(0x10)
	.dbline 148
	ldi R24,255
	out 0x1a,R24
	.dbline 148
	ldi R24,16
	out 0x1b,R24
	.dbline 148
	ldi R24,235
	out 0x15,R24
	.dbline 148
	ldi R24,203
	out 0x15,R24
	.dbline 148
	ldi R24,235
	out 0x15,R24
	.dbline 148
	.dbline 150
;  
;  BUSY
	nop
	.dbline 150
	nop
	.dbline 150
	nop
L37:
	.dbline 150
L38:
	.dbline 150
	sbis 0x13,0
	rjmp L37
X10:
	.dbline 152
;  
;  COM_W(0x70)
	.dbline 152
	ldi R24,255
	out 0x1a,R24
	.dbline 152
	ldi R24,112
	out 0x1b,R24
	.dbline 152
	ldi R24,235
	out 0x15,R24
	.dbline 152
	ldi R24,203
	out 0x15,R24
	.dbline 152
	ldi R24,235
	out 0x15,R24
	.dbline 152
	.dbline 153
;  DATA_R(a)
	.dbline 153
	clr R2
	out 0x1a,R2
	.dbline 153
	ldi R24,255
	out 0x1b,R24
	.dbline 153
	ldi R24,227
	out 0x15,R24
	.dbline 153
	ldi R24,225
	out 0x15,R24
	.dbline 153
	out 0x15,R24
	.dbline 153
	out 0x15,R24
	.dbline 153
	in R10,0x19
	.dbline 153
	ldi R24,227
	out 0x15,R24
	.dbline 153
	.dbline 155
; 
;  CS_OFF
	in R24,0x15
	ori R24,231
	out 0x15,R24
	.dbline 157
;  
;  if((a&0x01)==0)
	sbrc R10,0
	rjmp L40
X11:
	.dbline 158
;   return 0;    //页编程成功
	clr R16
	xjmp L29
L40:
	.dbline 160
;  else
;   return 1;   //页编程失败
	ldi R16,1
	.dbline -2
L29:
	.dbline 0 ; func end
	xjmp pop_xgset300C
	.dbsym r a 10 c
	.dbsym r i 20 i
	.dbsym l Len 6 i
	.dbsym r Dat 10 pc
	.dbsym r PageIndex 18 i
	.dbsym r Shift 16 i
	.dbend
	.area bss(ram, con, rel)
	.dbfile D:\Bikky\JDYQ\USB_Load\Flash.c
_num::
	.blkb 1
	.dbsym e num _num c
_well_counter::
	.blkb 1
	.dbsym e well_counter _well_counter c
_t::
	.blkb 8
	.dbsym e t _t A[8:8]c
_a::
	.blkb 1
	.dbsym e a _a c
_x::
	.blkb 1
	.dbsym e x _x c
_z::
	.blkb 1
	.dbsym e z _z c
; }
; 
