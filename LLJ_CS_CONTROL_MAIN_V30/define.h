#include <p18f4550.h>

#define SOFT_VER 0x10
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
#define BLOCKNUM 1024    //856    	//有效块数，实为1024块，只用到856块
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
/**************************
    单片机端口宏定义
***************************/
#define SHDN PORTAbits.RA5		//AVDD控制端口
#define SHDN_TRIS TRISAbits.TRISA5
#define RX_TRIS TRISCbits.TRISC7//串口端口
#define TX_TRIS TRISCbits.TRISC6
#define RX  PORTCbits.RC7
//#define TX_8M  OSCCONbits.IRCF0

#define CLOSE PORTEbits.RE0		//时钟控制端口
#define CLOSE_TRIS TRISEbits.TRISE0
#define CDT PORTEbits.RE1		//接收信号控制端口
#define CDT_TRIS TRISEbits.TRISE1
#define CTRL PORTEbits.RE2		//接收信号检测端口
#define CTRL_TRIS TRISEbits.TRISE2

//******AD8343端口
#define CS PORTDbits.RD5
#define CS_TRIS TRISDbits.TRISD5
#define CS0 {CS=0;Nop();Nop();Nop();Nop();Nop();}
#define CS1 {Nop();Nop();Nop();Nop();Nop();CS=1;}
//#define DIN PORTDbits.RD3
//#define DIN_TRIS TRISDbits.TRISD3
//#define DOUT PORTDbits.RD2
//#define DOUT_TRIS TRISDbits.TRISD2
//******FLASH端口
#define SCK PORTBbits.RB1
#define SCK_TRIS TRISBbits.TRISB1
#define FSO PORTBbits.RB2
#define FSO_TRIS TRISBbits.TRISB2
#define FSI PORTBbits.RB3
#define FSI_TRIS TRISBbits.TRISB3
#define F1CS PORTBbits.RB0
#define F1CS_TRIS TRISBbits.TRISB0
#define F1CS0 {F1CS=0;Nop();Nop();Nop();Nop();}
#define F1CS1 {Nop();Nop();Nop();Nop();F1CS=1;}

#define F2CS PORTBbits.RB4
#define F2CS_TRIS TRISBbits.TRISB4
#define F2CS0 {F2CS=0;Nop();Nop();}
#define F2CS1 {Nop();Nop();F2CS=1;}
/********************************
	  变量类型定义
*********************************/
#define uchar unsigned char
#define uint unsigned int
#define uslong unsigned short long
#define ulong unsigned long
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

/****************************
        函数声明
*****************************/
void Com_mode(void);			//通讯模式
void Test_mode(void);			//测试模式
void Test0_mode(void);			//正常压力计测试模式
//void Test1_mode(void);			//压力、张力、温度，6个字节

void Get_pgain(void);			//根据放大倍数，获取AD压力通道对应的控制命令字节
void Ini_port(void);			//单片机上电后的端口初始化函数
//void Inicom_port(void);
void Open_avdd(void);			//打开AVDD
void Close_avdd(void);			//关闭AVDD
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
       模拟SPI操作AD8343
******************************/
extern uint RecAD8343(void);	//读AD8343两个字节
extern void WriteAD8343(uchar); //写AD8343
extern void resetAD8343(void);  //复位AD8343
extern void PreAD8343_P1(void); //预读压力
extern void PreAD8343_T1(void); //预读温度
extern void ReadAD8343_16P1(void); //读压力
extern void ReadAD8343_16T1(void); //读温度
extern void ReadT0_8(void);        //T0计数器，测传播时间
extern void Meas_Time(void);        //测一次传播时间
extern void ReadAD8343_16F1(void); //16位转换读流量
extern void ReadAD8343_VBat(void); //16位转换读电池电压

extern void ReadAD_MODE0(void);	    //采样读压力温度
extern void ReadAD_MODE1(void);	    //采样读压力温度
extern void ReadAD_MODE2(void);	    //采样读压力温度
extern void ReadAD_MODE8(void);	    //采样读流量压力温度
//extern void Read_pt_fast(void);	//快采压力温度
//extern void AD_10BIT(void);	    //内部AD采集
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

/****************************
    全局变量声明
*****************************/
extern union Lch4 value;	//PT_value,压力、温度变量
extern union Allinfo para;			//仪器信息共用体，240字节
extern union Lch2 valid_block,AD_value;		//FLASH有效块，AD转换值
extern uint valid_excursion,Vbat_excursion;		//FLASH有效偏移量
extern uchar data74[74];			//发送包或接收包的存放数组
extern uchar Pgain;
