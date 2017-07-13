#include "define.h"

//20170705 改进详情见纪要
struct struct_manRcvInfo
{	
	uint8	flag;
	uint8	bitsCnt;
	uint8 	sta;	
	
	uint8	cmd;
	uint8	layer;
	//uint8	rcvPara;
	uint8	para;
	
	uint8	error;
};

/********************************
        全局变量定义   20140913 校验和0xaff0      V1.0  FOR CZYLJ系列压力计
            4芯通信接口兼容--------天石能源科技
*********************************/  

//20170710 修改模式、放大倍数、单点字节和压力零位，直接赋值，不再读配置
//20170710 系数直接固化在程序里
//20170710 采样方法还用原来的，读一次后马上采一次，实时性稍差，但可靠。 
//20170711 版本回退到V1.0，方便以后的功能升级
 #define SOFT_VERSION	0x10
	 
#pragma udata DATA_G1 
union Lch4 value;//PT_value,;	//压力、温度变量
union Allinfo para;			//仪器信息共用体，240字节
#pragma udata 

#pragma udata DATA_G2 
union Lch2 valid_block,AD_value;		//FLASH有效块,AD转换值
uint valid_excursion,Vbat_excursion;		//FLASH有效偏移量
uchar Rinter,data74[74]={0};//串口接收中断标志，发送包或接收包的存放数组
uchar T1_flag=0,Pgain=0,Int_dealy;	//TIMER1中断标志，压力放大倍数
#pragma udata


#pragma udata DATA_G3

struct struct_testRslt g_testRslt;

uint8 g_testRsltTempH = 0;
uint8 g_testRsltTempL = 0;

uint8 g_testCnt = 0;
uint8 g_testFlag = 0;

uint8 g_cmdFlag = 0;

uint8 g_rcvCmd = 0;

#define MYDBU1    0
#define MYDBU2	0

#pragma udata

//压力温度系数
#pragma udata DATA_PT
union union_ptCoff g_unPtCoff;
struct struct_calcRslt g_calcRslt;
#pragma udata


#define DELAY_0US5	{ _asm nop _endasm }
#define DELAY_1US	{ _asm nop nop _endasm }
#define DELAY_2US	{ _asm nop nop nop nop _endasm }
#define DELAY_5US	{ _asm nop nop nop nop nop nop nop nop nop nop _endasm }
#define DELAY_6US	{ _asm nop nop nop nop nop nop nop nop nop nop nop nop _endasm }
#define DELAY_10US	{ _asm nop nop nop nop nop nop nop nop nop nop nop nop nop nop nop nop nop nop nop nop _endasm }
#define DELAY_26US	{DELAY_6US DELAY_10US DELAY_10US }

#define DELAY_20US	{DELAY_10US DELAY_10US}
#define DELAY_50US	{DELAY_20US DELAY_20US  DELAY_10US}
#define DELAY_60US	{DELAY_20US DELAY_20US  DELAY_20US}
#define DELAY_100US	{DELAY_50US  DELAY_50US}

#define MAN_DELAY_10US		{DELAY_10US }
#define MAN_DELAY_20US		{DELAY_10US DELAY_10US}
#define MAN_DELAY_40US		{DELAY_10US DELAY_10US  DELAY_10US  DELAY_10US}
#define MAN_DELAY_80US		{MAN_DELAY_40US MAN_DELAY_40US }
#define MAN_DELAY_60US		{MAN_DELAY_40US MAN_DELAY_20US }

#define INT_DIS		{INTCONbits.GIE = 0;}
#define INT_EN		{INTCONbits.GIE = 1;}



///******************************************************************
//中断函数定义必须在main()函数前面，相应中断才能进入相应中断处理函数
//*******************************************************************
//接收到的地面数据
//		数据格式:(bits)
//				  0 , 同步头;  
//				  1～16, data;  17, 奇校验
//注：同步头和校验位在中断中处理。para在main（）进行处理
//	16bits data格式：4bits 层位，4bits 命令， 8bits数据
//信号硬件格式：
//				同步头：160us高，160us零
//				1：80us高，160us零
//				0：80us高，320us零	
//				判断数据时：在上升脉冲时开始计时，到下一个上升脉冲时读取计数，并重新开始计数
//				如果读出的计数在250（正负50）us内，则为1；在400（正负50）us内，则为0	
//*******************************************************************
void RX_TMIE1_inter(void);
#pragma code High_Interrupt_Vector=0X08
void high_ISR(void)
{
	_asm
	goto RX_TMIE1_inter
	_endasm
}
#pragma code
#pragma interrupt RX_TMIE1_inter
void RX_TMIE1_inter(void)
{
	uint8 i,j, calcVerify;
	//uint16 rcvData;
	//uint8 rcvDataBits[26];			//由高位到低位，依次接收到的数据位：17,同步头；16-1，数据位；0，校验位
	uint8 rcvVerify;
	uint8 timerVal;
	
	INT_DIS
	//Nop();Nop();Nop();			//空操作，防止睡眠唤醒后内部振荡器不稳定
	
	//debug
	//DEBUG1_1
	//DEBUG1_0
	
	//timer3定时中断
	if ( (PIE2bits.TMR3IE != 0) && (PIR2bits.TMR3IF != 0) )
	//if (PIR2bits.TMR3IF != 0) 
	{
		//debug
		#if MYDBU1
		//DEBUG1_1
		#endif
			
		TMR3H = 0x0B;
		TMR3L = 0xDC;
	
		PIR2bits.TMR3IF = 0;
		
		g_testCnt++;
		if (g_testCnt >= 4)
			g_testCnt = 0;
		
		g_testFlag = 1;
		
		//debug
		#if MYDBU1
		//DEBUG1_0
		#endif
	}
					
	//INT0中断
	if (INTCONbits.INT0IF != 0)
	{
		INTCONbits.INT0IF = 0;
		
	}
	
	if(PIE1bits.TMR1IE&PIR1bits.TMR1IF)
	{
		INTCON=0X00;		//禁止中断
		for(Int_dealy=183;Int_dealy>0;Int_dealy--);//1.464mS，20101029新加
		PIR1bits.TMR1IF=0;	//清零TIMER1中断标志
		T1CONbits.TMR1ON=0;	//关闭TIMER1	
		T1_flag=1;			//TIMER1中断标志置1
		INTCON=0XC0;		//重开中断
	}
	
	if( (PIE1bits.RCIE != 0) && (PIR1bits.RCIF != 0) )	//PIE1bits.RCIE != 0) && (PIR1bits.RCIF != 0) 
	{	
		g_rcvCmd = RCREG;// 读串口缓存并清串口接收中断
		//if(RCSTAbits.OERR==0)
		{
			g_cmdFlag = 1;
		}		
	}
	
	if(RCSTAbits.OERR==1)
	{
	    RCSTAbits.CREN=0;
	    RCSTAbits.CREN=1;
	}	
	INT_EN
}

void Init_Flash(void)
{
	uchar temp;	
	
	Write_Flash_unpro(0);	//FLASH取消片保护		
	temp=Read_status(0); 	//读FLASH状态寄存器	
	if(temp&0X0C)
	{
		Write_Flash_unpro(0);//防止第一次取消片保护失败
		Delay_mS(10);
	}
	if(para.info.mem_num==1)
	{
		Write_Flash_unpro(1025);	//FLASH取消片保护		
		temp=Read_status(1025); 	//读FLASH状态寄存器	
		if(temp&0X0C)
		{
			Write_Flash_unpro(1025);//防止第一次取消片保护失败
			Delay_mS(10);
		}
	}
}

void Init_Int(void)
{
	INTCONbits.PEIE = 1;
	INTCON2bits.INTEDG0 = 1;
	
	INTCONbits.INT0IF = 0;
	INTCONbits.INT0IE = 1;
	
	PIR1bits.RCIF = 0;
	PIE1bits.RCIE = 1;
	
	//PIE1bits.TMR1IE = 1;
	PIE2bits.TMR3IE = 1;	

}

void Init_Uart(void)
{	
	
	RX_TRIS=1;				//串行通信端口使能的必须设置
	TX_TRIS=1;				//串行通信端口使能的必须设置
	TX_TRIS=0;
	
	SPBRGH=0;				//波特率
	//SPBRG=16;				// 16=57600(4MHz),51=19200
	SPBRG=103;				// 16=115200(8MHz), 34=57600(8MHz), 207=9600, 103 = 19200
	BAUDCON=0X08;			//16位波特率发生器，禁止波特率检测，未监视RX引脚
	TXSTA=0X24;				//8位发送，使能发送，高速异步模式
	RCSTA=0X10;				//暂禁止串口，8位接收，使能接收

	RCSTAbits.SPEN=1;		//使能串行通信
}

/****************************************************
                      主函数
*****************************************************/
void main(void)
{	
	uchar temp;	
	
	Delay_mS(30);	
	
	OSCCON=0X72;			 
	while(OSCCONbits.IOFS==0);    	
	Delay_mS(10);			//上电延时10mS
	
	Ini_port();		    	//初始化端口，打开AVDD
	INT_DIS
	
	//初始化
	Init_Uart();
	Init_Flash();
	//Timer0_Init();	
	Timer3_Init();
		
	//初始化变量  *********************************
	Delay_mS(30);			//延时30mS
	Get_para();				//获取测试信息
	resetAD7799();
	
	//配置中断
	Init_Int();
			
	//debug
	while (0)
	{		
		//Tx(0x55);
		
		Delay_mS(2000);
		DEBUG1_1 
		DelayMs(100);
		DEBUG1_0 		
	}

	ReadMode();	
	
	//Com_mode();			//通讯模式
	
	while(1);	
}

void Ini_port(void)
{	
	INTCON=0X00;			        //禁止所有中断
	LATA=0X00;				//A口输出不锁存
	TRISA=0XFF;				//A口为输入	
	//PORTA=0X00;				//A口置初值		
	ADCON1=0X3A;			//AN0~AN4模拟输入 ,选择外部Vref电压基准。
	CMCON=0X07;				//A0、A1、A2、A3为IO口，比较器关

	LATB=0;					//B口输出不锁存
	TRISB=0XE3;				//B口B1,B2,B3为输出，其余为输入
	PORTB=0X04;				//B口置初值
	
	LATC=0;					//C口输出不锁存
	TRISC=0XBF;				//C口为输入
	PORTC=0X00; 				//C口为输入

	LATD=0X00;				//D口输出不锁存
	TRISD=0X47;				//D口为	
	PORTD=0X88;

	LATE=0X00;				//E口输出不锁存
	TRISE=0XFB;				//E口为输入	


	UCONbits.USBEN=0;		//禁止USB模块和支持电路
	SSPCON1bits.SSPEN=0;	 
	ADCON0bits.ADON=0;		//禁止AD转换模块

	
}

void Get_pgain(void)
{	
	para.info.p_gain=R2550(Pgain_addr);		//获取压力放大倍数
	para.info.p_gain=8;//20170710
	switch(para.info.p_gain)
	{
		case 2:{Pgain=0X01;break;}//2倍
		case 4:{Pgain=0X02;break;}//4倍
		case 8:{Pgain=0X03;break;}//8倍
		case 16:{Pgain=0X04;break;}//16倍
		case 32:{Pgain=0X05;break;}//32倍		
		case 64:{Pgain=0X06;break;}//64倍
		case 128:{Pgain=0X07;break;}//128倍
		default:{Pgain=0X04;break;}//默认16倍
	}	
	
	para.info.te_mode=R2550(TESTMODE_addr);	
	para.info.pointbyte=R2550(pointbyte_addr);
	para.info.adjzero.ch2[0]=R2550(Adjzero_addr);	//获取调节零位				
	para.info.adjzero.ch2[1]=R2550(Adjzero_addr+1);
	
	para.info.te_mode=0;//20170710
	para.info.pointbyte=4;//20170710
	para.info.adjzero.ch2[0]=0x88;//20170710
	para.info.adjzero.ch2[1]=0x13;//20170710
}

void Com_mode(void)
{
	uchar i;
//	Get_para();//获取测试信息

	RX_TRIS=1;				//串行通信端口使能的必须设置
	TX_TRIS=1;				//串行通信端口使能的必须设置
	TX_TRIS=0;
	
	SPBRGH=0;				//波特率57600(4MHz)
	SPBRG=16;				// 16=57600(4MHz),51=19200
	BAUDCON=0X08;			       //16位波特率发生器，禁止波特率检测，未监视RX引脚
	TXSTA=0X24;				//8位发送，使能发送，高速异步模式
	RCSTA=0X10;				//暂禁止串口，8位接收，使能接收

	OSCCON=0X72;
	Delay_mS(60);		//20101209新加
	//Ini_port();				//初始化端口，打开AVDD	
	
	i=R2550(Well1_addr);//空读操作，20101209新加
	RCSTAbits.SPEN=1;	//使能串行通信

	INTCON=0XC0;	 	//开中断
	while(1)
	{	
		Rinter=1;		//接收中断标志置1
		PIE1bits.RCIE=1;//允许串口接收中断
		while(Rinter);	//等待接收有效命令包
		data74[0]=Rx();		//接收第0个字节
		if(data74[0]==0XAA)//若包头为AA，则再接收两个命令字节
		{
			data74[1]=Rx();		//接收第1个字节
			data74[2]=Rx();		//接收第2个字节
			switch(data74[2])//若为有效命令字节，则接收其它字节
			{
				case 0X83://读仪器信息
				{			
					Txpack(Info_block);//发送相应包
					break;
				}
				case 0X84://写仪器信息
				{							
					//Rxpack(Info_block);//接收相应包
					Rx_infpack(Info_block);//接收相应包
					break;
				}
				case 0X85://读时间表
				{			
					Txpack(Timetab_block);//发送相应包
					break;
				}
				case 0X86://写时间表
				{							
					Rxpack(Timetab_block);//接收相应包
					break;
				}
				case 0X87://读校对表
				{			
					Txpack(Caltab_block);//发送相应包
					break;
				}
				case 0X88://写校对表
				{						
					Rxpack(Caltab_block);//接收相应包
					break;
				}
				case 0X89://单点测试
				{				
					Get_pgain();
					Single_point();	
					break;
				}	
				case 0X8A://读测试数据
				{
					Send_data();
					break;		
				}
				
				case 0X8C://删除数据
				{
					for(i=3;i<9;i++)
						data74[i]=Rx();
					Delete_data();					
					Send_vrf(8);
					break;					
				}
				case 0X8D://握手，使上位机判别仪器的波特率
				{
					for(i=3;i<9;i++)
						data74[i]=Rx();
					Send_vrf(8);
					break;	
				}
			
				case 0XC8://读测试信息
				{
					Send_testinfo();
					break;	
				}
				
				//ZFCY
				case 0X90://调试命令
				{
					Debug();
					break;	
				}	
			
				default:break;
			}
		}						
	}
}



void ReadMode(void)
{
	uchar temp,go_on,YU,Sample_flag,T1H;
	ulong SHANG,SHANG_temp;
	union Lch2 pnormal;
	union Lch8 time;
	uchar cmd;
	uchar i, j;
	unsigned int w;
	unsigned int u16Temp1;
	uchar u8Temp1;
	
	DEBUG1_1
	DelayMs(100);
	DEBUG1_0
		
	ReadPtCoff();
	Get_pgain();//读压力放大倍数
	PreAD7799_P1();		
	resetAD7799();

	AllParaTest();	
	ReadPtCoff();
	AllParaCalc();
	
//	Timer3_Restart();
	
	g_rcvCmd = RCREG;// 读串口缓存并清串口接收中断
	g_rcvCmd = RCREG;// 读串口缓存并清串口接收中断
	g_rcvCmd = RCREG;// 读串口缓存并清串口接收中断
	INT_EN
	
	
		AllParaTest();		     
	      AllParaCalc();
	      	
	while (1)
	{	
	
	
		if (g_cmdFlag != 0)
		{
			INT_DIS
			 
			g_cmdFlag = 0;
			
			switch (g_rcvCmd)
			{
				//读版本号
				case ENUM_CMD_READVER:
					SendByteByUart(SOFT_VERSION);
					break;
				
				//读采样工程量
				case ENUM_CMD_READTESTRSLT:					
					SendAllDataByUart();
					break;
				//读采样数字量
				case ENUM_CMD_READTESTORG:
					SendAllDataOrgByUart();				
					break;
					
				default:
					break;
			}
			
			g_rcvCmd = RCREG;// 读串口缓存并清串口接收中断
			g_rcvCmd = RCREG;// 读串口缓存并清串口接收中断
			g_rcvCmd = RCREG;// 读串口缓存并清串口接收中断
			g_rcvCmd = RCREG;// 读串口缓存并清串口接收中断
			RCSTAbits.CREN=0;//禁止串口并清串口溢出中断标志
	            RCSTAbits.CREN=1;//使能串口
			INT_EN
			
			AllParaTest();		     
	            AllParaCalc();
		}

	/*		
		if (g_testFlag != 0)
		{
			g_testFlag = 0;
			
			switch (g_testCnt)
			{
				case 0:
					//debug
					#if MYDBU2
					DEBUG1_1
					#endif
					
					PressTestStart();
					
					//debug
					#if MYDBU2
					DEBUG1_0
					#endif
					
					break;
					
				case 1:
					//debug
					#if MYDBU2
					DEBUG1_1
					#endif
					
					PressTestGetRslt();
					g_testRslt.unPress.i8[0] = g_testRsltTempL;
					g_testRslt.unPress.i8[1] = g_testRsltTempH;
					DelayMs(1);
					
					TempTestStart();
					
					//debug
					#if MYDBU2
					DEBUG1_0
					#endif
					
					break;
				
				case 2:
					//debug
					#if MYDBU2
					DEBUG1_1
					#endif
					
					TempTestGetRslt();
					g_testRslt.unTemp.i8[0] = g_testRsltTempL;
					g_testRslt.unTemp.i8[1] = g_testRsltTempH;
					//DelayMs(1);
					
					VoltTestStart();	
					
					//debug
					#if MYDBU2
					DEBUG1_0
					#endif
					
					break;
				
				case 3:
					//debug
					#if MYDBU2
					DEBUG1_1
					#endif
					
					VoltTestGetRslt();
					g_testRslt.vol = g_testRsltTempL;
					//g_testRslt.unTemp.i8[1] = g_testRsltTempH;
					
					AllParaCalc();
					
					//debug
					#if MYDBU2
					DEBUG1_0
					#endif
					
										//debug
					#if MYDBU2
					SendAllDataByUart();
					#endif
					
					break;
					
				default:
					break;
			}
			
		}*/
			
				
	}
			
	Sleep_forever();
}


void SendAllDataByUart(void)
{
	
	#if MYDBU1
		g_calcRslt.unPress.i8[0] = 0x34;
		g_calcRslt.unPress.i8[1] = 0x12;
		g_calcRslt.unTemp.i8[0] = 0x78;
		g_calcRslt.unTemp.i8[1] = 0x56;
	#endif
	
	DelayMs(2);	
	TXREG = g_calcRslt.unPress.i8[0];
	DelayMs(2);
	TXREG = g_calcRslt.unPress.i8[1];
	DelayMs(2);	
	TXREG = g_calcRslt.unTemp.i8[0];
	DelayMs(2);
	TXREG = g_calcRslt.unTemp.i8[1];
}

void SendAllDataOrgByUart(void)
{
	#if MYDBU1
		 g_testRslt.unPress.i8[0] = 0x11;
		 g_testRslt.unPress.i8[1] = 0x22;
		 g_testRslt.unTemp.i8[0] = 0x33;
		 g_testRslt.unTemp.i8[1] = 0x44;
	#endif
	
	DelayMs(2);
	TXREG = g_testRslt.unPress.i8[0];
	DelayMs(2);
	TXREG = g_testRslt.unPress.i8[1];	
	DelayMs(2);	
	TXREG = g_testRslt.unTemp.i8[0];
	DelayMs(2);
	TXREG = g_testRslt.unTemp.i8[1];
}

void SendByteByUart(uint8 dataSend)
{	
	DelayMs(2);
	TXREG = dataSend;	
}

//测试所有参数,将数据保存在缓存中
void AllParaTest(void)
{
	
	//测试压力温度
	ReadAD_MODE0();				
	//DEBUG
	//data74[1] = 0x00;
	//data74[0] = 0x00;
	g_testRslt.unPress.i8[0] = data74[0];
	g_testRslt.unPress.i8[1] = data74[1];
	g_testRslt.unTemp.i8[0] = data74[2];
	g_testRslt.unTemp.i8[1] = data74[3];
	
	//测试流量
	g_testRslt.unReserve.i8[1] = 0;
	g_testRslt.unReserve.i8[0] = 0;
	
	//读电机位置
	//g_testRslt.pos = g_motorPos;
	
	//测试电压
	AD_10BITNew();		//  ADCON2=0x21; 
	g_testRslt.vol = AD_value.ch2[1] + 1;
}

//对测试得到的参数进行计算
void AllParaCalc(void)
{
	uint16 pRslt, tRslt;
	uint8 vRslt;
	
	//计算压力温度     

	PTCalc(g_testRslt.unPress.i16, g_testRslt.unTemp.i16, &pRslt, &tRslt);
//	PTCalc(13465, 30834, &pRslt, &tRslt);//10MPa 20C
	
	g_calcRslt.unPress.i16 = pRslt;
	g_calcRslt.unTemp.i16 = tRslt;
	
	//计算电压
	VoltCalc(g_testRslt.vol, &vRslt);
	g_calcRslt.vol = vRslt;
	//g_calcRslt.pos = g_testRslt.pos;
}

void DelayMs(uint16 ms)
{
	uint16 w, l;
	
	for (w=ms; w!=0; w--)
	{
		for(l=131; l!=0; l--);
		DELAY_2US
		DELAY_1US
	}	
}
	
//调试专用
void Debug(void)
{
	uchar i;
	uint j;	
	uchar para1, para2;
	
	for(i=3;i<9;i++)
		data74[i]=Rx();
	
	para1 = data74[3];
	para2 = data74[4];
	
	switch (para1)
	{
		case 0x00:
			PIN_POWER_CTRL = (1^PIN_POWER_CTRL);
			break;
		
		case 0x01:
			//MotorRun(para2);
			break;
		
		case 0x02:
 			//SendMan();
			break;
			
		case 0x03:
			ReadAD_MODE0();
			Tx(data74[1]);
			Tx(data74[0]);
			Tx(data74[3]);
			Tx(data74[2]);
			break;
		
		case 0x04:
			AD_10BIT();
			Tx(AD_value.ch2[1]);
			Tx(AD_value.ch2[0]);
			AD_10BIT1();
			Tx(AD_value.ch2[1]);
			Tx(AD_value.ch2[0]);
				
		default:
			break;
	}
	
	Send_vrf(8);
}


//******************************************************************************
//  读压力温度校对表
//******************************************************************************
void ReadPtCoff(void)
{
	uint8 i, j;
	
//	Read_DF321page(Caltab_block, 0x0160, 80, g_unPtCoff.data);	
	g_unPtCoff.coff.a0=3.4313084234e+002;
	g_unPtCoff.coff.a1=-2.9784603138e-002;
	g_unPtCoff.coff.a2=8.3903920797e-007;
	g_unPtCoff.coff.a3=-7.9913758218e-012;
	g_unPtCoff.coff.b0=-5.4147818918e-002;
	g_unPtCoff.coff.b1=4.6354973968e-006;
	g_unPtCoff.coff.b2=-1.2923221066e-010;
	g_unPtCoff.coff.b3=1.2161616302e-015;
	g_unPtCoff.coff.c0=2.0619483508e-006;
	g_unPtCoff.coff.c1=-1.7322774729e-010;
	g_unPtCoff.coff.c2=4.8564247329e-015;
	g_unPtCoff.coff.c3=-4.5465565329e-020;
	g_unPtCoff.coff.d0=-2.4728763278e-011;
	g_unPtCoff.coff.d1=2.0818218408e-015;
	g_unPtCoff.coff.d2=-5.8485179596e-020;
	g_unPtCoff.coff.d3=5.4858617746e-025;
	
	g_unPtCoff.coff.e0=-4.3511282394e+002;
	g_unPtCoff.coff.e1=2.4150675885e-002;
	g_unPtCoff.coff.e2=-4.8912954880e-007;
	g_unPtCoff.coff.e3=6.0679645051e-012;
}	

//******************************************************************************
//   压力温度工程量计算  
//******************************************************************************
void PTCalc(uint16 pHz, uint16 tHz, uint16 *pRlst, uint16 *tRlst)
{
    uint8 i;
	uint16 x,y;
	double pVal, tVal;
	double dbTemp1, dbTemp2;
	double coffA, coffB, coffC, coffD;
	
	tVal = g_unPtCoff.coff.e0 + g_unPtCoff.coff.e1 * tHz + g_unPtCoff.coff.e2 * tHz * tHz +  g_unPtCoff.coff.e3 * tHz * tHz * tHz;
		
	coffA = g_unPtCoff.coff.a0 + g_unPtCoff.coff.a1 * tHz + g_unPtCoff.coff.a2 * tHz * tHz + g_unPtCoff.coff.a3 * tHz * tHz * tHz;
	coffB = g_unPtCoff.coff.b0 + g_unPtCoff.coff.b1 * tHz + g_unPtCoff.coff.b2 * tHz * tHz + g_unPtCoff.coff.b3 * tHz * tHz * tHz;
	coffC = g_unPtCoff.coff.c0 + g_unPtCoff.coff.c1 * tHz + g_unPtCoff.coff.c2 * tHz * tHz + g_unPtCoff.coff.c3 * tHz * tHz * tHz;
	coffD = g_unPtCoff.coff.d0 + g_unPtCoff.coff.d1 * tHz + g_unPtCoff.coff.d2 * tHz * tHz + g_unPtCoff.coff.d3 * tHz * tHz * tHz;
	
	pVal = coffA + coffB * pHz + coffC * pHz * pHz + coffD * pHz * pHz * pHz;
	

	//读工程量进行处理，以整形数返回	
	pVal+=5;
	pVal *= 1000;
	if(pVal<0){pVal=0;}//处理负数
	*pRlst = (uint16)pVal;
	
	tVal += 50;
	tVal *= 100;
	if(tVal<0){tVal=0;}//处理负数
	*tRlst = (uint16)tVal;
}

void VoltCalc(uint8 voltHz, uint8 *pVolt)
{
	
	*pVolt = voltHz;
}

