//2004-09-21 注释
//数据类型定义，对不同的编译器，此文件应作相应修改，ICC-AVR编译器如下。
#ifndef __comm_def_h__
#define __comm_def_h__

#define int8	char
#define int16	int
#define int32	long
#define uint8	unsigned char
#define uint16	unsigned int
#define uint32	unsigned long

//以下是交换一个16位二进制数的高低部分
#define SWAP(x)			((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))	
//以变量x为基址移位y后的内存值					
#define SPLIT(x,y)      (*((uint8*)(&x)+y))  




#endif

