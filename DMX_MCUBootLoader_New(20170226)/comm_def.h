//2004-09-21 ע��
//�������Ͷ��壬�Բ�ͬ�ı����������ļ�Ӧ����Ӧ�޸ģ�ICC-AVR���������¡�
#ifndef __comm_def_h__
#define __comm_def_h__

#define int8	char
#define int16	int
#define int32	long
#define uint8	unsigned char
#define uint16	unsigned int
#define uint32	unsigned long

//�����ǽ���һ��16λ���������ĸߵͲ���
#define SWAP(x)			((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))	
//�Ա���xΪ��ַ��λy����ڴ�ֵ					
#define SPLIT(x,y)      (*((uint8*)(&x)+y))  




#endif

