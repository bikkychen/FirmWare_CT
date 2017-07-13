#include <p18f4550.h>

/********************************
	  变量类型定义
*********************************/
#define uchar unsigned char
#define uint unsigned int
#define uslong unsigned short long
#define ulong unsigned long

#define uint8 unsigned char
#define uint16 unsigned int

union Lch2
{
	uint l;
	uchar ch2[2];
};
union Lch3
{
	uslong l;
	uchar ch3[3];
};
union Lch4
{
	ulong l;
	uchar ch4[4];
};
struct Inter_loop
{
	union Lch4 interval;
	union Lch4 looptime;
};
union Lch8
{
	uchar ch8[8];
	struct Inter_loop tab;
};

union union_uint16
{
 	uint16 i16;
	uint8  i8[2];
};

//各参数测试得到的数字量
struct struct_testRslt
{
	union union_uint16	unPress;
	union union_uint16	unTemp;
	union union_uint16	unReserve;
	uint8	pos;
	uint8	vol;	
};
//各参数计算得到的工程量
struct struct_calcRslt
{
	union union_uint16	unPress;
	union union_uint16	unTemp;
	union union_uint16	unReserve;
	uint8	pos;
	uint8	vol;	
};



struct Testinfo
{ 
  	uchar well_info[224];  		//32口井信息()，224字节   
	uchar well;					//总井数，1字节
	union Lch3 points; 			//总点数，3字节
	uchar loop_point;			//循环指针，1字节
	uchar DataFull;				//数据满标志，1字节
	uchar count;				//计算标志，1字节
	uchar p_gain;  				//压力放大倍数，1字节 
	uchar te_mode;				//测试方式，1字节
	uchar mem_num;  			//存储器数量，1字节
	union Lch2 adjzero;     	//零位调节值2	
	union Lch2 CSZERO;     	    //参数零位2
	uchar pointbyte;  			//单点字节数，1字节 
	uchar YULIU;     	        //预留1
} ;

union Allinfo
{
	uchar data[240];
	struct Testinfo info;		//240字节
};
/*
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

*/
struct struct_manSendInfo
{			
	uint8	cmd;
	uint8	layer;
	//uint8	rcvPara;
	uint16	data;	
};


//定义数据类型，保存接收到的压力温度校对表
typedef struct struct_ptCoff
{
    double a0;
	double a1;
	double a2;
	double a3;
	
	double b0;
	double b1;
	double b2;
	double b3;
	
	double c0;
	double c1;
	double c2;
	double c3;
	
	double d0;
	double d1;
	double d2;
	double d3;
	
	double e0;
	double e1;
	double e2;
	double e3;	
};
	
typedef union union_ptCoff
{
    struct struct_ptCoff coff;
	unsigned char data[80];
};
/*
typedef struct struct_coff
{
    unsigned char coffOrg[352];
	union union_ptCoff ptCoff;
};
	
typedef union union_coff
{
    struct struct_coff coff;
	unsigned char data[512];
};
*/

/**************************
    宏定义
***************************/
//第一块FLASH空间分配，共1024块【0，1023】，每块大小为4096字节
#define err_block 0		//坏块信息块
#define err_addr 0X0000
#define Info_block 1		//仪器信息块
#define Info_addr 0X1000
#define Timetab_block 2 	//时间表块
#define Timetab_addr 0X2000
#define Caltab_block 3  	//校对表块
#define Caltab_addr 0X3000
#define Vbattab_block 4  	//电池电压块
#define Vbattab_addr 0X4000

#define Data_addr 0X5000	//数据存储的起始地址
#define BLOCKNUM 1024    //有效块数，实为1024块，只用到856块
#define ROTATEBIT 12		//每块大小为4096=2^12字节
#define BLOCKYU 0XFFF		//4095=0XFFF
//******************************
//PIC18LF2550内部256字节EEPROM空间分配
#define Loop_addr 0X00		//循环单元起始地址
#define Well1_addr 0X10		//第一口井的首块地址
#define Vbat1_addr 0X12	     //第一口井电池电压地址
#define Well_addr 0XF0      //总井数地址1
#define Points_addr 0XF1    //总点数地址3

#define Looppoint_addr 0XF4  //循环指针地址1
#define DataFull_addr  0XF5    //数据存满标志1
#define Count_addr  0XF6      //计算标志地址1
#define Pgain_addr 0XF7      //压力放大倍数地址1
#define TESTMODE_addr 0XF8    //测试方式1
#define memnum_addr  0XF9    //存储器数量1
#define Adjzero_addr 0XFA    //零位调节2
#define CSzero_addr  0XFC     //参数2零位2
#define pointbyte_addr  0XFE     //单点字节数1
#define YULIU_addr   0XFF    //预留字节数1

#define ENUM_EADDR_MOTORPOS	0XF5			//电机位置

/**************************
    单片机端口宏定义
***************************/
//#define STA PORTDbits.RC7		//PORTDbits.RD5状态端口
//#define STA_IS_IN TRISC|=0X80           //TRISD|=0X01
#define SHDN PORTDbits.RD4		//AVDD控制端口
#define SHDN_TRIS TRISDbits.TRISD4
#define RX_TRIS TRISCbits.TRISC7//串口端口
#define TX_TRIS TRISCbits.TRISC6

#define RX  PORTCbits.RC7
#define TX_8M  OSCCONbits.IRCF0

//ZFCY *************************************************
#define PIN_MAN_RX	PORTBbits.RB0
#define PIN_RM_INT	PORTBbits.RB0
#define PIN_MAN_TX	PORTEbits.RE2
#define PIN_MOTOR_CTRL1	PORTDbits.RD1
#define PIN_MOTOR_CTRL2	PORTDbits.RD2
//#define PIN_MOTOR_CTRL3	PORTDbits.RD0
//#define PIN_MOTOR_CTRL4	PORTDbits.RD1
#define PIN_POWER_CTRL		PORTDbits.RD0

//******AD7799端口
#define SCLK PORTDbits.RD4
#define SCLK_TRIS TRISDbits.TRISD4
#define CS PORTDbits.RD3
#define CS_TRIS TRISDbits.TRISD3
#define CS0 {CS=0;Nop();Nop();Nop();Nop();Nop();}
#define CS1 {Nop();Nop();Nop();Nop();Nop();CS=1;}
#define DIN PORTDbits.RD6
#define DIN_TRIS TRISDbits.TRISD6
#define DOUT PORTDbits.RD5
#define DOUT_TRIS TRISDbits.TRISD5
//******FLASH端口
#define SCK PORTBbits.RB4
#define SCK_TRIS TRISBbits.TRISB4
#define SI PORTBbits.RB3
#define SI_TRIS TRISBbits.TRISB3
#define SO PORTBbits.RB1
#define SO_TRIS TRISBbits.TRISB1
#define F1CS PORTBbits.RB2
#define F1CS_TRIS TRISBbits.TRISB2
#define F1CS0 {F1CS=0;Nop();Nop();}
#define F1CS1 {Nop();Nop();F1CS=1;}

#define F2CS PORTDbits.RD7
#define F2CS_TRIS TRISDbits.TRISD7
#define F2CS0 {F2CS=0;Nop();Nop();}
#define F2CS1 {Nop();Nop();F2CS=1;}

//曼码发送宏定义
#define MAN_SEND_POS {PORTE=0X04;}
#define MAN_SEND_ZERO {PORTE=0X00;}

//调试口
#define	DEBUG1_1	{ PORTBbits.RB3 = 1; }
#define	DEBUG1_0	{ PORTBbits.RB3 = 0; }



typedef struct struct_manRcvInfo
{
	uint8	rcvFlag;
	uint8	rcvBitsCnt;
	uint8 	rcvSta;
	
	
	uint8	rcvCmd;
	uint8	rcvLayer;
	//uint8	rcvPara;
	uint16	rcvData;
	
	uint8	rcvError;		
};

//全局变量及函数宏定义 **************************************
//读版本
#define ENUM_CMD_READVER	0xDF
//采集工程量
#define ENUM_CMD_READTESTRSLT	0xCF
//采集数字量
#define ENUM_CMD_READTESTORG	0x4F

#define ENUM_WORKSTA_READ		0
#define ENUM_WORKSTA_MOTORRUN	1

#define TIMER0_START	{ T0CON |= 0x80; }
#define TIMER0_STOP	{ T0CON &= 0x7F; }





/****************************
        函数声明
*****************************/
void Com_mode(void);			//通讯模式
void Test_mode(void);			//测试模式
void Test0_mode(void);			//正常压力计测试模式
//void Test1_mode(void);			//压力、张力、温度，6个字节
void Read_mode(void);
void Read0_mode(void);
void AllParaTest(void);
void SendMan(void);
void ManSendBit1(void);
void ManSendBit0(void);
void AllParaTest(void);
void AllParaCalc(void);
void ReadPtCoff(void);
void PTCalc(uint16 pHz, uint16 tHz, uint16 *pRlst, uint16 *tRlst);

void Debug(void);
void MotorRun(uchar direction);

void Get_pgain(void);			//根据放大倍数，获取AD压力通道对应的控制命令字节
void Ini_port(void);			//单片机上电后的端口初始化函数
void SendByteByUart(uint8 dataSend);

void Sample_Send(void);


void Debug(void);
void MotorRun(uchar direction);
void DelayMs(uint16 ms);

void Timer0_Init(void);
void Timer3_Init(void);
void MotorCurOvInit(void);
void AD_10BITNew(void);

//void Inicom_port(void);
//****************************
extern void Delay_mS(uint);		//毫秒延时
extern void Delay_uS(uchar);		//微秒延时
extern void Tx(uchar);			//串口发送函数
extern uchar Rx(void);			//串口接收函数
extern void Get_para(void);		//获取测试信息，即240字节的内
extern void Send_vrf(uchar);	//发送校验包，9字节或74字节
extern void Txpack(uint);		//发送数据包
extern void Txblockpack(void);
extern void Rxpack(uint);		//接收数据包
extern void Rx_infpack(uint);	//接收仪器信息数据包
extern void Rx_Testinfo(void);	//接收测试信息数据包
extern void Single_point(void);	//单点检测
extern void Send_data(void);	//发送测试数据
extern void Delete_data(void);	//格式化存储器或删除数据
extern void Send_testinfo(void);//发送测试信息，256字节=240有用+16无用
extern void Send_IDinfo(void);//发送ID信息，
extern void Sample_save(void);	//采样压力温度并存储
//extern void Sample_save2(void);	//快速采样压力温度并存储
//extern void Sample_save_600(uint);	//快速采样压力温度并存储600点
extern void Sleep_forever(void);//永久睡眠
/***************************
PIC18LF2550内部EEPROM操作
****************************/
extern uchar R2550(uchar);		//读EEPROM
extern void W2550(uchar,uchar);
extern void WW2550(uchar,uchar);//写EEPROM
/*****************************
       模拟SPI操作AD7799
******************************/
extern uchar RecAD7799(void);	//读AD7799一个字节
extern void WriteAD7799(uchar); //写AD7799
extern void resetAD7799(void);  //复位AD7799
extern void PreAD7799_P1(void); //预读压力
extern void PreAD7799_T1(void); //预读温度
extern void ReadAD7799_16P1(void); //读压力
extern void ReadAD7799_16P2(void); //读压力
extern void ReadAD7799_16T1(void); //读温度
extern void ReadAD7799_16T2(void); //读温度
extern void ReadAD7799_24P1(void); //24位转换读压力
extern void ReadAD7799_24T1(void); //24位转换读温度
extern void ReadAD_MODE0(void);	    //采样读压力温度
extern void ReadAD_MODE1(void);	    //采样读压力温度
extern void ReadAD_MODE2(void);	    //采样读压力温度
extern void ReadAD_MODE6(void);	    //采样读压力温度
//extern void Read_pt_fast(void);	//快采压力温度
extern void AD_10BIT(void);	    //内部AD采集

//ZFCY
extern void AD_10BIT1(void);

//extern void Read_vbat(void);
/*****************************
       模拟SPI操作FLASH
******************************/
extern void SpiDF321_OutByte(uchar); //单片机输出一个字节
extern uchar SpiDF321_InByte(void);	 //单片机输入一个字节
//extern void Read_ID(uchar *);
extern void Write_EN(uint);			 //写使能
extern uchar Read_status(uint);		 //读状态寄存器

extern void Write_Flash_unpro(uint); //取消片保护
extern void Read_DF321page(uint,uint,uint,uchar *);	 //从指定起始位置读取指定数量的字节
extern void Read_DF321ID(uint,uchar *);	 //ID
extern void Write_DF321page(uint,uint,uint, uchar *);//从指定起始位置写入指定数量的字节
extern uchar Erase_DF321block(uint); //擦除一块，[0，855]


void Init_Flash(void);
void Init_Int(void);
void Init_Uart(void);
void ReadMode(void);
void SendAllDataByUart(void);
void SendAllDataOrgByUart(void);
void AllParaTest(void);
void AllParaCalc(void);
void VoltCalc(uint8 voltHz, uint8 *pVolt);

void Timer3_Init(void);
void Timer3_Restart(void);
void Timer3_Stop(void);
void AD_10BITNew(void);
void  PressTestStart(void);
void PressTestGetRslt(void);
void TempTestStart(void);
void TempTestGetRslt(void);
void TestVol(void);
void VoltTestStart(void);
void VoltTestGetRslt(void);













/****************************
    全局变量声明
*****************************/
extern union Lch4 value;	//PT_value,压力、温度变量
extern union Allinfo para;			//仪器信息共用体，240字节
extern union Lch2 valid_block,AD_value;		//FLASH有效块，AD转换值
extern uint valid_excursion,Vbat_excursion;		//FLASH有效偏移量
extern uchar data74[74];			//发送包或接收包的存放数组
extern uchar Pgain;

extern uint8 g_testRsltTempH ;
extern uint8 g_testRsltTempL ;









