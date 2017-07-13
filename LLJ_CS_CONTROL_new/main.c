#include "define.h"
/********************************
        全局变量定义   20160916 校验和0x9d15   V1.0  FOR V1.0电路板 CDLJA系列流量计    使用内部8MHz-----PWM输出信号
          读加密后校验和0x9d15        4芯接口兼容--------天石能源科技
*********************************/   
/*
20170521改进：
1、版本号升级为V3.0
2、在device.c文件的Get_para()子函数中固定存储数量为1，固定测试模式为8
3、在main.c文件的Get_pgain()子函数中固定放大倍数为8，测试模式为8，单点字节数为8，压力采样零位为5000
4、在main.c文件的Test0_mode()子函数中在读取时间表项后立即改当前时间表项的采样点数为0x8000，采样间隔为5
5、在main.c文件的Test_mode()子函数中屏蔽了判断预留字节的代码段
6、在main.c文件的Test_mode()函数中将压力传感器采样值判断功能屏蔽
7、在main.c文件的Test_mode()函数中将井数大于32改为大于等于32
8、在main.c文件的Test_mode()函数中每口井建档时写入一个固定的时间表到这口井的首块
*/

union Lch4 value;//PT_value,;	//压力、温度变量
union Allinfo para;			//仪器信息共用体，240字节
union Lch2 valid_block,AD_value;		//FLASH有效块,AD转换值
uint valid_excursion,Vbat_excursion;		//FLASH有效偏移量
uchar Rinter,data74[74]={0};//串口接收中断标志，发送包或接收包的存放数组
uchar T1_flag=0,Pgain=0,Int_dealy;	//TIMER1中断标志，压力放大倍数
//uchar Pgain=0,Int_dealy;

# pragma udata wlBuf
uint g_wlValAr[16];		//定义数组保存涡轮脉冲计数值

uchar g_wlCnt;
uchar g_wlModeFlag = 0;
uchar g_wlCntHigh, g_wlCntLow;
uchar MYDBU = 1;
# pragma udata

#define SOFT_VERSION	0x30
/******************************************************************
 中断函数定义必须在main()函数前面，相应中断才能进入相应中断处理函数
*******************************************************************/
void RX_TMIE1_inter(void);
#pragma code High_Interrupt_Vector=0X808
//#pragma code High_Interrupt_Vector=0X08
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
	Nop();Nop();Nop();			//空操作，防止睡眠唤醒后内部振荡器不稳定
	if(PIE1bits.TMR1IE&PIR1bits.TMR1IF)
	{
		INTCON=0X00;		//禁止中断
		
		if (g_wlModeFlag == 0)
		{
			for(Int_dealy=183;Int_dealy>0;Int_dealy--);//1.464mS，20101029新加
			PIR1bits.TMR1IF=0;	//清零TIMER1中断标志
			T1CONbits.TMR1ON=0;	//关闭TIMER1	
			T1_flag=1;			//TIMER1中断标志置1
			data74[50]=1;
			INTCON=0XC0;		//重开中断	
		}	
		else
		{
			PIR1bits.TMR1IF=0;	//清零TIMER1中断标志
			TMR1H=0xF0;
			TMR1L=0x00;		//定时器赋初值，1/8 s
			
			 g_wlCntLow = TMR0L;
			 g_wlCntHigh = TMR0H;
			 
			 TMR0H = 0;
			 TMR0L = 0;
			 
			g_wlValAr[g_wlCnt] = g_wlCntHigh;
			g_wlValAr[g_wlCnt] = (g_wlValAr[g_wlCnt]<<8);
			g_wlValAr[g_wlCnt] += g_wlCntLow; 
			g_wlCnt++;
			 if (g_wlCnt>= 16)
			 {
				 g_wlCnt = 0;
			} 
			 
			 INTCON=0XC0;		//重开中断	
		}	
		
	}
	if(PIR1bits.RCIF)
	{
		PIE1bits.RCIE=0;	//禁止串口接收中断
		Rinter=0;			//接收中断标志置0
	}
}
/****************************************************
                      主函数
*****************************************************/
void main(void)
{	
	uchar temp;				//复位时内部振荡器的默认输出频率1MHz
	Delay_mS(60);			//上电延时，1MHz内部振荡器频率时，约为120ms
	//OSCCON=0X72;    //0X72;	0X62		//执行SLEEP指令后进入休眠模式，外部振荡器，频率8MHz
	OSCCON=0X40;
	//while(OSCCONbits.IOFS==0);//0X62等待内部4MHz时钟稳定	
	Delay_mS(200);			//上电延时100mS				
	//Open_avdd();			//打开AVDD
	Ini_port();		    //初始化端口，打开AVDD

	Nop();
	//Delay_mS(122);
	Nop();
	//AD_value.l=RecAD7799();
	Nop();
	
	Delay_mS(60);			//延时30mS
	

	Write_Flash_unpro(0);	//FLASH取消片保护		
	temp=Read_status(0); 	//读FLASH状态寄存器	
	if(temp&0X0C)
	{
		Write_Flash_unpro(0);//防止第一次取消片保护失败
		Delay_mS(10);
	}
	
	/*
	AD_value.l=0x5678;
	Write_DF321page(5,2,2,AD_value.ch2);
	AD_value.l=0x0000;*/	
//	Read_DF321page(5,0,2,AD_value.ch2);

	Get_para();//获取测试信息
	
	resetAD7799();



	if (MYDBU != 0)
	{
		Ccp1_Init();	
	}
	

		
	if(RX==0)				//若状态口为1：进入测试模式；为0：进入通讯模式
		Test_mode();		//测试模式
	else
	{	
		Com_mode();			//通讯模式
	}
	while(1);
	
}

void Ini_port(void)
{	
	INTCON=0X00;			//禁止所有中断
	LATA=0X00;				//A口输出不锁存
	TRISA=0X10;				//A4口为输入	
	PORTA=0X20;				//0x30---A口置初值SHDN=1
		
	ADCON1=0X0E;			//A0为模拟输入、其余A1、A2、A3等为IO口
	CMCON=0X07;				//比较器关

	LATB=0;					//B口输出不锁存
	TRISB=0X08;				//B口B1,B2,B3为输出，其余为输入
	PORTB=0X01;				//B口置初值
	
	LATC=0;					//C口输出不锁存
	TRISC=0X82;				//C口为输入
	PORTC=0X03; 				//C口为输入

	LATD=0X00;				//D口输出不锁存
	TRISD=0X10;				//D口为	
	PORTD=0X30;

	LATE=0X00;				//E口输出不锁存
	TRISE=0X00;				//E口为输出
	PORTE=0X00;

	T0CON=0X28;
	UCONbits.USBEN=0;		//禁止USB模块和支持电路
	SSPCON1bits.SSPEN=0;	        //禁止SPI、I2C功能
	ADCON0bits.ADON=0;		//禁止AD转换模块

	
}

void Ccp1_Init(void)
{
	//add pwm
	TRISC &= ~(1<<2);				//RC2为输出
	
	//set pwm frequency: period = (PR2 + 1) * 4 * Tosc * (TMR2 prescale)
	//Tosc = 1 / 8MHz; TMR2 prescale = 1
	
	PR2 = 99;  //20kHz
	//g_PWMPeriodScale = PR2 + 1;	
	//set pwm duty cycle: duty cycle = (CCPR1L:CCP1CON<5:4>) * Tosc * (TMR2 prescale)
	CCP1CONbits.DC1B1 = 1;
	CCP1CONbits.DC1B0 = 0; 
	CCPR1L = 0; 
	
/*
	PR2 = 0;  //2MHz
	//g_PWMPeriodScale = PR2 + 1;	
	//set pwm duty cycle: duty cycle = (CCPR1L:CCP1CON<5:4>) * Tosc * (TMR2 prescale)
	CCP1CONbits.DC1B1 = 1;
	CCP1CONbits.DC1B0 = 0; 
	CCPR1L = 0; 
	*/
	
	/*
	PR2 = 1;   //1MHz
	//g_PWMPeriodScale = PR2 + 1;	
	//set pwm duty cycle: duty cycle = (CCPR1L:CCP1CON<5:4>) * Tosc * (TMR2 prescale)
	CCP1CONbits.DC1B1 = 0;
	CCP1CONbits.DC1B0 = 0; 
	//CCPR1L = g_PWMPeriodScale >> 1;//
	CCPR1L = 1;  */

	//set timer2
	T2CONbits.T2CKPS1 = 0;	
	T2CONbits.T2CKPS0 = 0; 
	T2CONbits.TMR2ON = 1;

	//set CCP1 as pwm
	CCP1CONbits.CCP1M3 = 1;	
	CCP1CONbits.CCP1M2 = 1; 
	CCP1CONbits.CCP1M1 = 0;	
	CCP1CONbits.CCP1M0 = 0;
}

void Init_Timer1(void)
{
	T1CON=0XCE;          //使能TIMER1振荡器,0X4E预分频1：1，0X7E预分频8：1，异步模式;	
	
	PIE1bits.TMR1IE=1;   //允许Timer1中断	
	
	TMR1H=0xF0;
	TMR1L=0x00;		//定时器赋初值，1/8 s
		
	T1CONbits.TMR1ON=1;//开启TMR1
}

void Init_Timer0(void)
{
	uchar i;
	g_wlCnt = 0;
	for (i=0; i<8; i++)
	{
		g_wlValAr[i] = 0;
	}
	
	T0CON = 0x28;
	
	INTCONbits.TMR0IF = 0;
	
	TMR0H = 0;
	TMR0L = 0;
	
	T0CONbits.TMR0ON=1;	//开启TMR0		
}

void Init_WlSample(void)
{
	g_wlModeFlag = 1;
	
	Init_Timer0();
	Init_Timer1();	
}
			
void Open_avdd(void)
{		
	TRISB=0X08;				//B口B1,B2,B3为输出，其余为输入
	PORTB=0X01;				//B口置初值CS为高
	
	TRISC=0X02;				//C口为输入TRISC=0X82;
	PORTC=0X03;				//C口清零

	TRISE=0X00;				//E口为输入
	PORTE=0X00;

	TRISD=0X10;				//D口为	
	PORTD=0X30;

	TRISA=0X10;				//A口A4为输入T0
	PORTA=0X20;
	//set timer2

	//T2CONbits.TMR2ON = 1;

	//set CCP1 as pwm
	//CCP1CONbits.CCP1M3 = 1;	
	//CCP1CONbits.CCP1M2 = 1; 
	//CCP1CONbits.CCP1M1 = 0;	
	//CCP1CONbits.CCP1M0 = 0;

	SHDN=1;					//打开AVDD,对FLASH和AD供电后，才对相应IO口操作
	//Nop();Nop();			
	//Nop();Nop();			
	//FLASH端口
	//F1CS1					//外设FLASH片选端置1	
	//AD7799端口
	//CS1						//外设AD7799片选端置1	
}

void Close_avdd(void)
{//关电前将与AVDD有关的口设为输出0
	TRISA=0X10;				//
	PORTA=0X10;				//SHDN=1 SHDN1=0  SHDN输出0，关闭AVDD
	SHDN=0;	
	//set timer2

//	T2CONbits.TMR2ON = 0;

	//set CCP1 as pwm
//	CCP1CONbits.CCP1M3 = 0;	
//	CCP1CONbits.CCP1M2 = 0; 
//	CCP1CONbits.CCP1M1 = 0;	
//	CCP1CONbits.CCP1M0 = 0;
			
	TRISC=0X02;		// 1  0  0  0  0  0  1  0  //82							
	PORTC=0X03;	    //C0,C1,C7值不变//PORTC&=0X83;
	
	TRISB=0X00;				//0000  0000与FLASH有关的4个口线输出0
	PORTB=0X00;				//B口清零

	TRISE=0X00;		//
	PORTE=0X00;	

	
	TRISD=0X00;    //0000   0000
	PORTD=0X00;             //
	
}

void Get_pgain(void)
{	
	para.info.p_gain=R2550(Pgain_addr);		//获取压力放大倍数
	para.info.p_gain=8;//20170521
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
	
	para.info.te_mode=8;//20170521
	para.info.pointbyte=8;//20170521
	para.info.adjzero.ch2[0]=0x88;//20170521
	para.info.adjzero.ch2[1]=0x13;//20170521
}

void Com_mode(void)
{
	uchar i;
//	Get_para();//获取测试信息
	
	RX_TRIS=1;				//串行通信端口使能的必须设置
	TX_TRIS=1;				//串行通信端口使能的必须设置
	SPBRGH=0;				//波特率57600(4MHz)
	//SPBRG=16;				// 16=115200(8MHz)
	SPBRG=34;				// 57600 (8MHZ)
	BAUDCON=0X08;			       //16位波特率发生器，禁止波特率检测，未监视RX引脚
	TXSTA=0X24;				//8位发送，使能发送，高速异步模式
	RCSTA=0X10;				//暂禁止串口，8位接收，使能接收

	//OSCCON=0X72;
	//Delay_mS(60);		//20101209新加
	//Ini_port();				//初始化端口，打开AVDD	
	//resetAD7799();
	i=R2550(Well1_addr);//空读操作，20101209新加
	RCSTAbits.SPEN=1;	//使能串行通信
	INTCON=0XC0;	 	//开中断
	
	 Init_WlSample();
	
	while(1)
	{	
		Rinter=1;		//接收中断标志置1
		PIE1bits.RCIE=1;//允许串口接收中断
		while(Rinter);	//等待接收有效命令包
		data74[0]=Rx();		//接收第0个字节
		if(data74[0]==0XE8)//若包头为E8，则再接收两个命令字节
		{
			data74[1]=Rx();		//接收第1个字节
			if(data74[1]==0X40)//若包头为40，则再接收两个命令字节
			{
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
				//新的读测试数据：命令中保护起始地址和起始包数
				case 0x9A:
				{
					Send_dataNew();
					break;
				}
				case 0X8B://读块数据
				{
					
					Txblockpack();//发送块相应数据包
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
						
						//
						data74[3]=SOFT_VERSION;//固件版本
						
					Send_vrf(8);
					break;	
				}
				case 0XD8://写测试信息
				{							
					Rx_Testinfo();//接收相应包
					break;
				}
				case 0XC8://读测试信息
				{
					Send_testinfo();
					break;	
				}	
				case 0X8E://读ID信息
				{			
					Send_IDinfo();//发送相应包
					break;
				}
				case 0X8F://读格式化信息
				{			
					Txpack(err_block);//发送相应包
					break;
				}
				default:break;
			}
			}
		}							
	}
}

void Test_mode(void)
{
	union Lch8 time;//20170521
	uchar temp,i,mm,Mode=0;
	uchar Etabaddr;
	uint j,bn;
	//ulong SHANG,SHANG_temp;
	union Lch2 pnormal,tnormal,t1normal;
	//union Lch8 time;

	//Get_para();//获取测试信息
	T1CON=0X4E;          //使能TIMER1振荡器,0X4E预分频1：1，0X7E预分频8：1，异步模式;
	RCSTAbits.SPEN=0;//禁止异步收发
	PORTCbits.RC7=0;//RX
	TRISCbits.TRISC7=0;//RX_TRIS
	RX=0;//指示灯初始状态：灭
	TX_TRIS=0;
	bn=BLOCKNUM;
	if(para.info.mem_num==1)
	{
		bn=BLOCKNUM+1024;
	}

	/*******************************************************
		若已经有32口井，则连续闪烁2次以报警
		延时10S后，若还未断电，则删除数据，从头开始存储
	*******************************************************/
	if(para.info.well>=32)
	{
		RX=0;Delay_mS(3000);   
		for(temp=0;temp<2;temp++)
		{//FOR循环共延时8S，也为32768起振提供了时间
			RX=1;Delay_mS(3000);//亮3S	
			RX=0;Delay_mS(1000);//灭1S
		}
		Delay_mS(10000);
		//循环单元中的当前块，也是第32口井的末块，也是需删除的最大块数
		i=Loop_addr+((para.info.loop_point)<<1);//得到当前块
		pnormal.ch2[0]=R2550(i);
		i++;
		pnormal.ch2[1]=R2550(i);		
		//pnormal.l+=2; 					//末口井块数+=2，多删了2块
		
		data74[4]=0X05;
		data74[5]=0X00;					//首块：3
		data74[6]=pnormal.ch2[0];
		data74[7]=pnormal.ch2[1];		//块数
		Delete_data();					//删除数据
		Get_para();						//重新获取测试信息
	}
/*********************************************
	若井数有效，则判断首块valid_block是否有效
**********************************************/	
	if(para.info.well==0)
		valid_block.l=0X05;
	else
	{//从循环位置指针所指的循环单元中，找出最后块，得到新井的首块
		temp=Loop_addr+((para.info.loop_point)<<1);
		valid_block.ch2[0]=R2550(temp);
		temp++;
		valid_block.ch2[1]=R2550(temp);
	}

	do
	{
		Etabaddr=(valid_block.l>>3);
		Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
		i=data74[9];
		i>>=(valid_block.l&0X07);
		i&=0X01;
		if(i==0)//若是好块
		{
			Delay_mS(15);//20101102新加
			Read_DF321page(valid_block.l,0,4,value.ch4);
			if(value.l==0XFFFFFFFF)		
				break;	//遇到好的新块则跳出
		}	
		valid_block.l++;
	}while(valid_block.l<bn);

	if(valid_block.l==bn)
		Sleep_forever();//超出有效存储块范围
/*********************************************
	若井数有效，且首块也有效，则判断AD是否正常
**********************************************/
	Get_pgain();//读压力放大倍数
	//resetAD7799();
//	Read_pt(1);  //指示灯闪烁前判断AD是否正常
//	pnormal.ch2[0]=AD_value.ch2[0];
//	pnormal.ch2[1]=AD_value.ch2[1];
	PreAD7799_P1();
	//PreAD7799_T1();
	ReadAD_MODE0();
	
	//20170521
	/*
	tnormal.ch2[0]=data74[0];
	tnormal.ch2[1]=data74[1];
	if((tnormal.l>=0)&&(tnormal.l<0XFFFF))
		temp=0;//AD正常
	else
		temp=1;//AD不正常
	while(temp)
	{	RX=1;  //电池指示灯亮
	}*/
/*********************************************
	若井数有效，首块有效，AD正常，则继续进行
**********************************************/
		
	if(para.info.loop_point>6)
		para.info.loop_point=0;
	else
		para.info.loop_point++;
	temp=Loop_addr+((para.info.loop_point)<<1);
	WW2550(temp,valid_block.ch2[0]);   //把首块写入循环位置
	temp++;
	WW2550(temp,valid_block.ch2[1]);
	WW2550(Looppoint_addr,para.info.loop_point);//更新循环位置指针
	//AD_10BIT();  //电池电压	
	temp=Well1_addr+(para.info.well*7);  //6);//-----20130425
	WW2550(temp,valid_block.ch2[0]);   //存入新井首块
	temp++;
	WW2550(temp,valid_block.ch2[1]);
	temp++;

	WW2550(Well_addr,para.info.well+1);//井数+1
	WW2550(Count_addr,0X00);		   //置计算标志为0

	//for(i=0;i<5;i++)
	//{//采样上电时的压力零位，存入新井首块的时间表之后
	//为防止FLASH操作影响电压进而影响AD，因此把AD采集放在操作FLASH之前
	//	Read_pt();
	//	temp=i;
	//	temp<<=2;
	//	zero[temp+0]=PT_value.ch4[0];
	//	zero[temp+1]=PT_value.ch4[1];
	//	zero[temp+2]=PT_value.ch4[2];
	//	zero[temp+3]=PT_value.ch4[3];
//	}
	
//	Delay_mS(1000);//20101102为适应标检时的试上电而加
	for(i=0;i<64;i++)
	{                  //读取时间表,并写入新井首块
		pnormal.l=i;
		pnormal.l<<=3;	
		Read_DF321page(Timetab_block,pnormal.l,8,data74);		 
		time.tab.looptime.l=0x8000;//20170521
      	time.tab.interval.l=5;//20170521
		Delay_mS(10);
	//	Write_DF321page(valid_block.l,pnormal.l,8,data74);//20170521
		Write_DF321page(valid_block.l,pnormal.l,8,time.ch8);//20170521
		Delay_mS(15);
	}			
		
	pnormal.l=512;
	switch(para.info.te_mode)
	{
		case 0:
		{
			ReadAD_MODE0();			
			Write_DF321page(valid_block.l,pnormal.l,4,data74);
			Delay_mS(18);
			pnormal.l+=4;		
			break;
		} 
		case 1:
		{
			ReadAD_MODE1();		
			Write_DF321page(valid_block.l,pnormal.l,6,data74);
			Delay_mS(18);
			pnormal.l+=6;		
			break;
		} 	
		case 8:
		{
			ReadAD_MODE8();		
			Write_DF321page(valid_block.l,pnormal.l,8,data74);
			Delay_mS(18);
			pnormal.l+=8;		
			break;
		} 
		case 9:
		{
			ReadAD_MODE9();		
			Write_DF321page(valid_block.l,pnormal.l,8,data74);
			Delay_mS(18);
			pnormal.l+=8;
			break;
		} 
		default:
		{
			ReadAD_MODE0();			
			Write_DF321page(valid_block.l,pnormal.l,4,data74);
			Delay_mS(18);
			pnormal.l+=4;		
			break;
		} 
	}		
	//若正常，则连续闪烁3次,并进入定时采点阶段
	for(temp=0;temp<3;temp++)
	{//FOR循环共延时6S，也为32768起振提供了时间
		RX=1;Delay_mS(2000);//亮
		RX=0;Delay_mS(2000);//灭
	}
//20170521
/*
	temp=para.info.te_mode+1;     
	if(para.info.YULIU!=temp)   // 5/temp
	{
		RX=1;
		while(1);
	}  
*/	
	    
	temp=Erase_DF321block(Vbattab_block);
	//有效块为valid_block
	Mode=para.info.te_mode; 

	valid_excursion=512+para.info.pointbyte;//yh20110419  有效偏移量为528	
	//para.info.adjzero.l
	Vbat_excursion=0;
	      //para.info.adjzero.ch2[1]>>5;
	//Mode=Mode&0x07;

	Test0_mode();//正常压力计采样模式

	/*switch(Mode)
	{
		case 0://
		{	
			Test0_mode();//正常压力计采样模式
			break;
		}
		case 3://
		{	
			Test1_mode();//进入压力、张力和温度采样,单点考虑6个字节的采样
			break;
		}
		
		default:break;
	}*/		
}


void Test0_mode(void)
{
	uchar temp,go_on,YU,Sample_flag,T1H;
	ulong SHANG,SHANG_temp;
	union Lch2 pnormal;
	union Lch4 F1_value,F2_value;
	union Lch8 time;
	
	T1CON=0X4E;          //使能TIMER1振荡器,0X4E预分频1：1，0X7E预分频8：1，异步模式;
	RCSTAbits.SPEN=0;    //禁止异步收发
	PORTCbits.RC7=0;     //RX
	TRISCbits.TRISC7=0;  //RX_TRIS
	RX=0;                //指示灯初始状态：灭
	TX_TRIS=0;
	
	//T3CON=0X0E;          //使能TIMER3振荡器,预分频1：1，65536=2S
	
	INTCON=0XC0;	     //睡眠前使能中断
	PIE1bits.TMR1IE=1;   //允许Timer1中断	
	//PIE2bits.TMR3IE=1;   //允许Timer3中断
	//IPR2bits.TMR3IP=1;   //TMR3 溢出中断高优先级位
	//go_on=1;
	Read_DF321page(Timetab_block,0,8,time.ch8);//读取第一项时间表，8字节
      time.tab.looptime.l=0x8000;//20170521
      time.tab.interval.l=5;//20170521
	for(temp=1;(temp<=64)&&(time.tab.interval.l>1)&&(time.tab.looptime.l>0);temp++)
	{	
		//SHANG=(time.tab.interval.l-1)>>4;		//商
		//YU=(time.tab.interval.l-1)&0XF;		//余数
		SHANG=(time.tab.interval.l-2)>>1;		//商
		YU=(time.tab.interval.l-2)&0X1;		//余数
		//T1H=YU;	
		//T1H=256-(YU<<4);
		Sample_flag=0;
		if(time.tab.looptime.l>0)	//把对长整型的判断转为对字符型的判断，使定时尽量准确
			go_on=1;
		else
			go_on=0;
		if(time.tab.interval.l>2)    //采样间隔>2S进低功耗20160915
			Close_avdd();              //20160915
						
		do
		{				
			if(time.tab.interval.l>2)				//即采样间隔>2S
			{				
				SHANG_temp=SHANG;
				while(SHANG_temp>0)
				{
					TMR1L=6;		//定时器赋初值，2s
					TMR1H=0X00;	
					T1CONbits.TMR1ON=1;//开启TMR1
					SHANG_temp--;
					Sleep();
				}
				T1H=YU;
				if(T1H>0)
				{
					TMR1L=6;		//定时器赋初值，2s
					TMR1H=0X80;	
					T1CONbits.TMR1ON=1;//开启TMR1
					Sleep();
				}
				
			}
				Open_avdd();
				F1_value.l=0;
				F2_value.l=0;
				
				EXC=1;
				CS0
				TMR1L=26;			//20160915新改，定时器赋初值
				TMR1H=0XF0;
				T1_flag=0;				
				T1CONbits.TMR1ON=1;	//开启TMR1
				while(T1_flag==0);
				//TEST=0;				
				EXC=0;					
				T1_flag=0;
				TMR1L=16;			//20160915新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				while(T1_flag==0);	
							
				EXC=1;				
				T1_flag=0;
				TMR1L=16;			//20160915新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				while(T1_flag==0);
				
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160915新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				while(T1_flag==0);
			//-------------------采集温度--20160915
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//20160915新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				ReadAD7799_16T1();
				data74[2]=value.ch4[1];
				data74[3]=value.ch4[2];
				while(T1_flag==0);
			//-------------------采集压力--20160915	
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160915新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				ReadAD7799_16P1();
				data74[0]=value.ch4[0];
				data74[1]=value.ch4[1];
				while(T1_flag==0);	
			//--------------------------------------------
				EXC=1;				
				T1_flag=0;
				TMR1L=16;			//20160915新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				while(T1_flag==0);
				
			//-------------------采集流量--20160916
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160916新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				Delay_mS(106);
				//TEST=1;
				ReadAD7799_16F1();
				//TEST=0;
				F1_value.l+=AD_value.l;
				while(T1_flag==0);
				
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				Delay_mS(106);
				ReadAD7799_16F2();
				F2_value.l+=AD_value.l;
				while(T1_flag==0);
				//-----------------------------1
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160916新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				Delay_mS(106);
				ReadAD7799_16F1();
				F1_value.l+=AD_value.l;
				while(T1_flag==0);
				
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				Delay_mS(106);
				ReadAD7799_16F2();
				F2_value.l+=AD_value.l;
				while(T1_flag==0);
				//-----------------------------2
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160916新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				Delay_mS(106);
				ReadAD7799_16F1();
				F1_value.l+=AD_value.l;
				while(T1_flag==0);
				
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				Delay_mS(106);
				ReadAD7799_16F2();
				F2_value.l+=AD_value.l;
				while(T1_flag==0);
				//-----------------------------3
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160916新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				Delay_mS(106);
				ReadAD7799_16F1();
				F1_value.l+=AD_value.l;
				while(T1_flag==0);
				
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//20160915新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				Delay_mS(106);
				ReadAD7799_16F2();
				F2_value.l+=AD_value.l;
				while(T1_flag==0);
			    //------------------------------4
			    EXC=0;
				F1_value.l=(F1_value.l)>>2;
				data74[4]=F1_value.ch4[0];
				data74[5]=F1_value.ch4[1];
				F2_value.l=(F2_value.l)>>2;
				data74[6]=F2_value.ch4[0];
				data74[7]=F2_value.ch4[1];		
					
				T1_flag=0;
				TMR1L=16;			//20160915新改，定时器赋初值
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//开启TMR1
				AD_value.l=3;
				CS1
				//RX=1;         //20130618				
				Sample_save();	//采样存储一次
				//RX=0;         //20130618	
				//TEST=0;			
				time.tab.looptime.l--;//循环次数减1	
				if(time.tab.interval.l>2)				//即采样间隔>2S
					Close_avdd();

				if(time.tab.looptime.l>0)
					go_on=1;
				else
					go_on=0;
				while(T1_flag==0);
				
				//T1CON=0X7E;			 				
			//}
			
		}while(go_on);				//若循环次数>0，则继续循环，否则加载下一项时间表
		//Open_avdd();
		//Delay_mS(25); 
		pnormal.l=temp;
		pnormal.l<<=3;
		Read_DF321page(Timetab_block,pnormal.l,8,time.ch8);//加载下一项时间表
		time.tab.looptime.l=0x8000;//20170521
      	time.tab.interval.l=5;//20170521
	}
	Sleep_forever();
}