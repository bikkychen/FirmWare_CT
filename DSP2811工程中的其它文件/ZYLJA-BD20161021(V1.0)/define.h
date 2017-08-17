#include <p18f4550.h>
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
#define BLOCKNUM 1024    //856    	//��Ч������ʵΪ1024�飬ֻ�õ�856��
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
/**************************
    ��Ƭ���˿ں궨��
***************************/
//#define STA PORTDbits.RC7		//PORTDbits.RD5״̬�˿�
//#define STA_IS_IN TRISC|=0X80           //TRISD|=0X01
#define SHDN PORTEbits.RE1		//AVDD���ƶ˿�
#define SHDN_TRIS TRISEbits.TRISE1
#define RX_TRIS TRISCbits.TRISC7//���ڶ˿�
#define TX_TRIS TRISCbits.TRISC6

#define RX  PORTCbits.RC7
#define TX_8M  OSCCONbits.IRCF0
//******AD7799�˿�
#define SCLK PORTDbits.RD4
#define SCLK_TRIS TRISDbits.TRISD4
#define CS PORTCbits.RC5
#define CS_TRIS TRISCbits.TRISC5
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
/********************************
	  �������Ͷ���
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

/****************************
        ��������
*****************************/
void Com_mode(void);			//ͨѶģʽ
void Test_mode(void);			//����ģʽ
void Test0_mode(void);			//����ѹ���Ʋ���ģʽ
//void Test1_mode(void);			//ѹ�����������¶ȣ�6���ֽ�

void Get_pgain(void);			//���ݷŴ�������ȡADѹ��ͨ����Ӧ�Ŀ��������ֽ�
void Ini_port(void);			//��Ƭ���ϵ��Ķ˿ڳ�ʼ������
//void Inicom_port(void);
		//�ر�AVDD
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
//extern void Read_pt_fast(void);	//���ѹ���¶�
extern void AD_10BIT(void);	    //�ڲ�AD�ɼ�
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

/****************************
    ȫ�ֱ�������
*****************************/
extern union Lch4 value;	//PT_value,ѹ�����¶ȱ���
extern union Allinfo para;			//������Ϣ�����壬240�ֽ�
extern union Lch2 valid_block,AD_value;		//FLASH��Ч�飬ADת��ֵ
extern uint valid_excursion,Vbat_excursion;		//FLASH��Чƫ����
extern uchar data74[74];			//���Ͱ�����հ��Ĵ������
extern uchar Pgain;
