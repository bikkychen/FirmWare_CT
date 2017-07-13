
#include <string.h>
#include <stdio.h>
#include	"HAL.H"			// 其它单片机需修改HAL*硬件抽象层的几个文件
/*
#define	CH374_SPI_SCS0			{DDRB|=0x10;PORTB&=0xef;}		 
#define	CH374_SPI_SCS1			{DDRB|=0x10;PORTB|=0x10;}
*/
#define	CH374_SPI_SCS0			{DDRB|=0x01;PORTB&=0xFE;}		 
#define	CH374_SPI_SCS1			{DDRB|=0x01;PORTB|=0x01;}	
	
extern unsigned char EP2SendBusy;
/*
#define	CH374_SPI_SCS0			PORTB&=0xfe;		 
#define	CH374_SPI_SCS1			PORTB|=0x01;		 
 
#define	CH374_SPI_SDI0			PORTB&=0xfb;		 
#define	CH374_SPI_SDI1			PORTB|=0x04;		 

#define	CH374_SPI_SDO			(PINB&0xf7)		   

#define	CH374_SPI_SCK1			PORTB|=0x02;		 
#define	CH374_SPI_SCK0			PORTB&=0xfd;	 
*/
void EP2Send(UINT8 mLen, PUINT8 mBuf);
void EP2SendConst(UINT8 mLen, const unsigned char  *mBuf);

UINT8 Read374Byte( UINT8 mAddr )         // 从指定寄存器读取数据 
{
	CH374_SPI_SCS0   //SPI片选有效
	
	SPDR = mAddr; // 启动数据传输  发送读地址
    while((SPSR & 0x80)==0x00); // 等待传输结束 
	
	SPDR = 0xc0; // 启动数据传输  发送读命令 CMD_SPI_374READ=0xc0
    while((SPSR & 0x80)==0x00); // 等待传输结束  
	
	SPDR = 0x00;
    while((SPSR & 0x80)==0x00); // 等待传输结束 
	
    CH374_SPI_SCS1              // SPI停止
	return SPDR;
}

void Write374Byte( UINT8 mAddr, UINT8 mData )  // 向指定寄存器写入数据 
{

	CH374_SPI_SCS0   //SPI片选有效
	
	SPDR = mAddr; // 启动数据传输  发送读地址
    while((SPSR & 0x80)==0x00); // 等待传输结束 
	
	SPDR = 0x80; // 启动数据传输  发送写命令 CMD_SPI_374WRITE=0x80
    while((SPSR & 0x80)==0x00); // 等待传输结束  
	
	SPDR = mData;
    while((SPSR & 0x80)==0x00); // 等待传输结束 
	
    CH374_SPI_SCS1              // SPI停止

}

void Read374Block( UINT8 mAddr, UINT8 mLen, PUINT8 mBuf )  // 从指定起始地址读出数据块 
{
	CH374_SPI_SCS0    //SPI片选有效
	 
	SPDR = mAddr; // 启动数据传输  发送读地址
    while((SPSR & 0x80)==0x00); // 等待传输结束 
	
	SPDR = 0xc0; // 启动数据传输  发送读命令 CMD_SPI_374READ=0xc0
    while((SPSR & 0x80)==0x00); // 等待传输结束 
	
	while ( mLen -- )
	{
	 SPDR = 0x00;
     while((SPSR & 0x80)==0x00); // 等待传输结束 
	 *mBuf++ = SPDR;
	 }
	CH374_SPI_SCS1              //SPI停止
}

void Write374Block( UINT8 mAddr, UINT8 mLen, PUINT8 mBuf )  // 向指定起始地址写入数据块 
{
	CH374_SPI_SCS0     //SPI片选有效
	 
	SPDR = mAddr; // 启动数据传输  发送读地址
    while((SPSR & 0x80)==0x00); // 等待传输结束 
	
	SPDR = 0x80; // 启动数据传输  发送写命令 CMD_SPI_374WRITE=0x80
    while((SPSR & 0x80)==0x00); // 等待传输结束  
	
	while ( mLen -- ) 
	{
	 SPDR = *mBuf++;
     while((SPSR & 0x80)==0x00); // 等待传输结束 
	 }
	
	CH374_SPI_SCS1             //SPI停止
}

void Write374Block2( UINT8 mAddr, UINT8 mLen,const unsigned char  *mBuf )  // 向指定起始地址写入数据块 
{
	CH374_SPI_SCS0      //SPI片选有效
	 
	SPDR = mAddr; // 启动数据传输  发送读地址
    while((SPSR & 0x80)==0x00); // 等待传输结束 
	
	SPDR = 0x80; // 启动数据传输  发送写命令 CMD_SPI_374WRITE=0x80
    while((SPSR & 0x80)==0x00); // 等待传输结束  
	
	while ( mLen -- ) 
	{
	 SPDR = *mBuf++;
     while((SPSR & 0x80)==0x00); // 等待传输结束 
	 }
	
	CH374_SPI_SCS1              //SPI停止
}

void EP2SendConst(UINT8 mLen, const unsigned char  *mBuf)
{   while(EP2SendBusy);
	Write374Block2( RAM_ENDP2_TRAN, mLen,mBuf );
	Write374Byte( REG_USB_LENGTH, mLen );
	Write374Byte( REG_USB_ENDP2, M_SET_EP2_TRAN_ACK( Read374Byte( REG_USB_ENDP2 ) ) ^ BIT_EP2_RECV_TOG );
	EP2SendBusy=1;
}

void EP2Send(UINT8 mLen, PUINT8 mBuf)
{   while(EP2SendBusy);
	Write374Block( RAM_ENDP2_TRAN, mLen,mBuf );
	Write374Byte( REG_USB_LENGTH, mLen );
	Write374Byte( REG_USB_ENDP2, M_SET_EP2_TRAN_ACK( Read374Byte( REG_USB_ENDP2 ) ) ^ BIT_EP2_RECV_TOG );
	EP2SendBusy=1;
}

/*
void	Spi374OutByte( UINT8 d )  
{   
	UINT8	i;
	for ( i = 0; i < 8; i ++ ) 
	{
		CH374_SPI_SCK0
		if ( d & 0x80 ) 
		 CH374_SPI_SDI1
		else 
		 CH374_SPI_SDI0
		 
		CH374_SPI_SCK1     // CH374在时钟上升沿采样输入  
		d<<=1;
	}
	
	CH374_SPI_SCK0
}

UINT8	Spi374InByte( void )   
{   
	UINT8	i, d;
	d = 0;
	
 
	for ( i = 0; i < 8; i ++ ) 
	{   d<<=1;
	
		CH374_SPI_SCK1 

		CH374_SPI_SCK0        // CH374在时钟下降沿输出  
 
		
		if ( CH374_SPI_SDO ) 
		      d|=0x01;	
	}
	
	CH374_SPI_SCK0
	return( d );
}


UINT8	Read374Byte( UINT8 addr )  
{
	UINT8	d;
	CH374_SPI_SCS0
	Spi374OutByte( addr );
	Spi374OutByte( 0xc0);
	d = Spi374InByte( );
	CH374_SPI_SCS1
	return( d );
}

void	Write374Byte( UINT8 addr, UINT8 mData )   
{
	CH374_SPI_SCS0
	Spi374OutByte( addr );
	Spi374OutByte( 0x80 );
	Spi374OutByte( mData );
	CH374_SPI_SCS1
}

void	Read374Block( UINT8 addr, UINT8 mLen, PUINT8 mBuf )   
{
	CH374_SPI_SCS0
	Spi374OutByte( addr );
	Spi374OutByte( 0xc0 );
	while ( mLen -- ) 
	 *mBuf++ = Spi374InByte( );
    CH374_SPI_SCS1
}

void	Write374Block( UINT8 addr, UINT8 mLen, PUINT8 mBuf )  
{
	CH374_SPI_SCS0
	Spi374OutByte( addr );
	Spi374OutByte( 0x80 );
	while ( mLen -- ) 
	 Spi374OutByte( *mBuf++ );
    CH374_SPI_SCS1
}

void Write374Block2( UINT8 addr, UINT8 mLen,const unsigned char  *mBuf )  // 向指定起始地址写入数据块 
{
	CH374_SPI_SCS0
	Spi374OutByte( addr );
	Spi374OutByte( 0x80 );
	while ( mLen -- ) 
	 Spi374OutByte( *mBuf++ );
    CH374_SPI_SCS1
}
*/