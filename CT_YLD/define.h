#include <p18f4550.h>

/********************************
	  �������Ͷ���
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

//���������Եõ���������
struct struct_testRslt
{
	union union_uint16	unPress;
	union union_uint16	unTemp;
	union union_uint16	unReserve;
	uint8	pos;
	uint8	vol;	
};
//����������õ��Ĺ�����
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
  	uchar well_info[224];  		//32�ھ���Ϣ()��224�ֽ�   
	uchar well;					//�ܾ�����1�ֽ�
	union Lch3 points; 			//�ܵ�����3�ֽ�
	uchar loop_point;			//ѭ��ָ�룬1�ֽ�
	uchar DataFull;				//��������־��1�ֽ�
	uchar count;				//�����־��1�ֽ�
	uchar p_gain;  				//ѹ���Ŵ�����1�ֽ� 
	uchar te_mode;				//���Է�ʽ��1�ֽ�
	uchar mem_num;  			//�洢��������1�ֽ�
	union Lch2 adjzero;     	//��λ����ֵ2	
	union Lch2 CSZERO;     	    //������λ2
	uchar pointbyte;  			//�����ֽ�����1�ֽ� 
	uchar YULIU;     	        //Ԥ��1
} ;

union Allinfo
{
	uchar data[240];
	struct Testinfo info;		//240�ֽ�
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


//�����������ͣ�������յ���ѹ���¶�У�Ա�
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
    �궨��
***************************/
//��һ��FLASH�ռ���䣬��1024�顾0��1023����ÿ���СΪ4096�ֽ�
#define err_block 0		//������Ϣ��
#define err_addr 0X0000
#define Info_block 1		//������Ϣ��
#define Info_addr 0X1000
#define Timetab_block 2 	//ʱ����
#define Timetab_addr 0X2000
#define Caltab_block 3  	//У�Ա��
#define Caltab_addr 0X3000
#define Vbattab_block 4  	//��ص�ѹ��
#define Vbattab_addr 0X4000

#define Data_addr 0X5000	//���ݴ洢����ʼ��ַ
#define BLOCKNUM 1024    //��Ч������ʵΪ1024�飬ֻ�õ�856��
#define ROTATEBIT 12		//ÿ���СΪ4096=2^12�ֽ�
#define BLOCKYU 0XFFF		//4095=0XFFF
//******************************
//PIC18LF2550�ڲ�256�ֽ�EEPROM�ռ����
#define Loop_addr 0X00		//ѭ����Ԫ��ʼ��ַ
#define Well1_addr 0X10		//��һ�ھ����׿��ַ
#define Vbat1_addr 0X12	     //��һ�ھ���ص�ѹ��ַ
#define Well_addr 0XF0      //�ܾ�����ַ1
#define Points_addr 0XF1    //�ܵ�����ַ3

#define Looppoint_addr 0XF4  //ѭ��ָ���ַ1
#define DataFull_addr  0XF5    //���ݴ�����־1
#define Count_addr  0XF6      //�����־��ַ1
#define Pgain_addr 0XF7      //ѹ���Ŵ�����ַ1
#define TESTMODE_addr 0XF8    //���Է�ʽ1
#define memnum_addr  0XF9    //�洢������1
#define Adjzero_addr 0XFA    //��λ����2
#define CSzero_addr  0XFC     //����2��λ2
#define pointbyte_addr  0XFE     //�����ֽ���1
#define YULIU_addr   0XFF    //Ԥ���ֽ���1

#define ENUM_EADDR_MOTORPOS	0XF5			//���λ��

/**************************
    ��Ƭ���˿ں궨��
***************************/
//#define STA PORTDbits.RC7		//PORTDbits.RD5״̬�˿�
//#define STA_IS_IN TRISC|=0X80           //TRISD|=0X01
#define SHDN PORTDbits.RD4		//AVDD���ƶ˿�
#define SHDN_TRIS TRISDbits.TRISD4
#define RX_TRIS TRISCbits.TRISC7//���ڶ˿�
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

//******AD7799�˿�
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
//******FLASH�˿�
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

//���뷢�ͺ궨��
#define MAN_SEND_POS {PORTE=0X04;}
#define MAN_SEND_ZERO {PORTE=0X00;}

//���Կ�
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

//ȫ�ֱ����������궨�� **************************************
//���汾
#define ENUM_CMD_READVER	0xDF
//�ɼ�������
#define ENUM_CMD_READTESTRSLT	0xCF
//�ɼ�������
#define ENUM_CMD_READTESTORG	0x4F

#define ENUM_WORKSTA_READ		0
#define ENUM_WORKSTA_MOTORRUN	1

#define TIMER0_START	{ T0CON |= 0x80; }
#define TIMER0_STOP	{ T0CON &= 0x7F; }





/****************************
        ��������
*****************************/
void Com_mode(void);			//ͨѶģʽ
void Test_mode(void);			//����ģʽ
void Test0_mode(void);			//����ѹ���Ʋ���ģʽ
//void Test1_mode(void);			//ѹ�����������¶ȣ�6���ֽ�
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

void Get_pgain(void);			//���ݷŴ�������ȡADѹ��ͨ����Ӧ�Ŀ��������ֽ�
void Ini_port(void);			//��Ƭ���ϵ��Ķ˿ڳ�ʼ������
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
extern void Delay_mS(uint);		//������ʱ
extern void Delay_uS(uchar);		//΢����ʱ
extern void Tx(uchar);			//���ڷ��ͺ���
extern uchar Rx(void);			//���ڽ��պ���
extern void Get_para(void);		//��ȡ������Ϣ����240�ֽڵ���
extern void Send_vrf(uchar);	//����У�����9�ֽڻ�74�ֽ�
extern void Txpack(uint);		//�������ݰ�
extern void Txblockpack(void);
extern void Rxpack(uint);		//�������ݰ�
extern void Rx_infpack(uint);	//����������Ϣ���ݰ�
extern void Rx_Testinfo(void);	//���ղ�����Ϣ���ݰ�
extern void Single_point(void);	//������
extern void Send_data(void);	//���Ͳ�������
extern void Delete_data(void);	//��ʽ���洢����ɾ������
extern void Send_testinfo(void);//���Ͳ�����Ϣ��256�ֽ�=240����+16����
extern void Send_IDinfo(void);//����ID��Ϣ��
extern void Sample_save(void);	//����ѹ���¶Ȳ��洢
//extern void Sample_save2(void);	//���ٲ���ѹ���¶Ȳ��洢
//extern void Sample_save_600(uint);	//���ٲ���ѹ���¶Ȳ��洢600��
extern void Sleep_forever(void);//����˯��
/***************************
PIC18LF2550�ڲ�EEPROM����
****************************/
extern uchar R2550(uchar);		//��EEPROM
extern void W2550(uchar,uchar);
extern void WW2550(uchar,uchar);//дEEPROM
/*****************************
       ģ��SPI����AD7799
******************************/
extern uchar RecAD7799(void);	//��AD7799һ���ֽ�
extern void WriteAD7799(uchar); //дAD7799
extern void resetAD7799(void);  //��λAD7799
extern void PreAD7799_P1(void); //Ԥ��ѹ��
extern void PreAD7799_T1(void); //Ԥ���¶�
extern void ReadAD7799_16P1(void); //��ѹ��
extern void ReadAD7799_16P2(void); //��ѹ��
extern void ReadAD7799_16T1(void); //���¶�
extern void ReadAD7799_16T2(void); //���¶�
extern void ReadAD7799_24P1(void); //24λת����ѹ��
extern void ReadAD7799_24T1(void); //24λת�����¶�
extern void ReadAD_MODE0(void);	    //������ѹ���¶�
extern void ReadAD_MODE1(void);	    //������ѹ���¶�
extern void ReadAD_MODE2(void);	    //������ѹ���¶�
extern void ReadAD_MODE6(void);	    //������ѹ���¶�
//extern void Read_pt_fast(void);	//���ѹ���¶�
extern void AD_10BIT(void);	    //�ڲ�AD�ɼ�

//ZFCY
extern void AD_10BIT1(void);

//extern void Read_vbat(void);
/*****************************
       ģ��SPI����FLASH
******************************/
extern void SpiDF321_OutByte(uchar); //��Ƭ�����һ���ֽ�
extern uchar SpiDF321_InByte(void);	 //��Ƭ������һ���ֽ�
//extern void Read_ID(uchar *);
extern void Write_EN(uint);			 //дʹ��
extern uchar Read_status(uint);		 //��״̬�Ĵ���

extern void Write_Flash_unpro(uint); //ȡ��Ƭ����
extern void Read_DF321page(uint,uint,uint,uchar *);	 //��ָ����ʼλ�ö�ȡָ���������ֽ�
extern void Read_DF321ID(uint,uchar *);	 //ID
extern void Write_DF321page(uint,uint,uint, uchar *);//��ָ����ʼλ��д��ָ���������ֽ�
extern uchar Erase_DF321block(uint); //����һ�飬[0��855]


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
    ȫ�ֱ�������
*****************************/
extern union Lch4 value;	//PT_value,ѹ�����¶ȱ���
extern union Allinfo para;			//������Ϣ�����壬240�ֽ�
extern union Lch2 valid_block,AD_value;		//FLASH��Ч�飬ADת��ֵ
extern uint valid_excursion,Vbat_excursion;		//FLASH��Чƫ����
extern uchar data74[74];			//���Ͱ�����հ��Ĵ������
extern uchar Pgain;

extern uint8 g_testRsltTempH ;
extern uint8 g_testRsltTempL ;









