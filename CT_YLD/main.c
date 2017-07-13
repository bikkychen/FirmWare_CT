#include "define.h"

//20170705 �Ľ��������Ҫ
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
        ȫ�ֱ�������   20140913 У���0xaff0      V1.0  FOR CZYLJϵ��ѹ����
            4оͨ�Žӿڼ���--------��ʯ��Դ�Ƽ�
*********************************/  

//20170710 �޸�ģʽ���Ŵ����������ֽں�ѹ����λ��ֱ�Ӹ�ֵ�����ٶ�����
//20170710 ϵ��ֱ�ӹ̻��ڳ�����
//20170710 ������������ԭ���ģ���һ�κ����ϲ�һ�Σ�ʵʱ���Բ���ɿ��� 
//20170711 �汾���˵�V1.0�������Ժ�Ĺ�������
 #define SOFT_VERSION	0x10
	 
#pragma udata DATA_G1 
union Lch4 value;//PT_value,;	//ѹ�����¶ȱ���
union Allinfo para;			//������Ϣ�����壬240�ֽ�
#pragma udata 

#pragma udata DATA_G2 
union Lch2 valid_block,AD_value;		//FLASH��Ч��,ADת��ֵ
uint valid_excursion,Vbat_excursion;		//FLASH��Чƫ����
uchar Rinter,data74[74]={0};//���ڽ����жϱ�־�����Ͱ�����հ��Ĵ������
uchar T1_flag=0,Pgain=0,Int_dealy;	//TIMER1�жϱ�־��ѹ���Ŵ���
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

//ѹ���¶�ϵ��
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
//�жϺ������������main()����ǰ�棬��Ӧ�жϲ��ܽ�����Ӧ�жϴ�����
//*******************************************************************
//���յ��ĵ�������
//		���ݸ�ʽ:(bits)
//				  0 , ͬ��ͷ;  
//				  1��16, data;  17, ��У��
//ע��ͬ��ͷ��У��λ���ж��д���para��main�������д���
//	16bits data��ʽ��4bits ��λ��4bits ��� 8bits����
//�ź�Ӳ����ʽ��
//				ͬ��ͷ��160us�ߣ�160us��
//				1��80us�ߣ�160us��
//				0��80us�ߣ�320us��	
//				�ж�����ʱ������������ʱ��ʼ��ʱ������һ����������ʱ��ȡ�����������¿�ʼ����
//				��������ļ�����250������50��us�ڣ���Ϊ1����400������50��us�ڣ���Ϊ0	
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
	//uint8 rcvDataBits[26];			//�ɸ�λ����λ�����ν��յ�������λ��17,ͬ��ͷ��16-1������λ��0��У��λ
	uint8 rcvVerify;
	uint8 timerVal;
	
	INT_DIS
	//Nop();Nop();Nop();			//�ղ�������ֹ˯�߻��Ѻ��ڲ��������ȶ�
	
	//debug
	//DEBUG1_1
	//DEBUG1_0
	
	//timer3��ʱ�ж�
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
					
	//INT0�ж�
	if (INTCONbits.INT0IF != 0)
	{
		INTCONbits.INT0IF = 0;
		
	}
	
	if(PIE1bits.TMR1IE&PIR1bits.TMR1IF)
	{
		INTCON=0X00;		//��ֹ�ж�
		for(Int_dealy=183;Int_dealy>0;Int_dealy--);//1.464mS��20101029�¼�
		PIR1bits.TMR1IF=0;	//����TIMER1�жϱ�־
		T1CONbits.TMR1ON=0;	//�ر�TIMER1	
		T1_flag=1;			//TIMER1�жϱ�־��1
		INTCON=0XC0;		//�ؿ��ж�
	}
	
	if( (PIE1bits.RCIE != 0) && (PIR1bits.RCIF != 0) )	//PIE1bits.RCIE != 0) && (PIR1bits.RCIF != 0) 
	{	
		g_rcvCmd = RCREG;// �����ڻ��沢�崮�ڽ����ж�
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
	
	Write_Flash_unpro(0);	//FLASHȡ��Ƭ����		
	temp=Read_status(0); 	//��FLASH״̬�Ĵ���	
	if(temp&0X0C)
	{
		Write_Flash_unpro(0);//��ֹ��һ��ȡ��Ƭ����ʧ��
		Delay_mS(10);
	}
	if(para.info.mem_num==1)
	{
		Write_Flash_unpro(1025);	//FLASHȡ��Ƭ����		
		temp=Read_status(1025); 	//��FLASH״̬�Ĵ���	
		if(temp&0X0C)
		{
			Write_Flash_unpro(1025);//��ֹ��һ��ȡ��Ƭ����ʧ��
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
	
	RX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	TX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	TX_TRIS=0;
	
	SPBRGH=0;				//������
	//SPBRG=16;				// 16=57600(4MHz),51=19200
	SPBRG=103;				// 16=115200(8MHz), 34=57600(8MHz), 207=9600, 103 = 19200
	BAUDCON=0X08;			//16λ�����ʷ���������ֹ�����ʼ�⣬δ����RX����
	TXSTA=0X24;				//8λ���ͣ�ʹ�ܷ��ͣ������첽ģʽ
	RCSTA=0X10;				//�ݽ�ֹ���ڣ�8λ���գ�ʹ�ܽ���

	RCSTAbits.SPEN=1;		//ʹ�ܴ���ͨ��
}

/****************************************************
                      ������
*****************************************************/
void main(void)
{	
	uchar temp;	
	
	Delay_mS(30);	
	
	OSCCON=0X72;			 
	while(OSCCONbits.IOFS==0);    	
	Delay_mS(10);			//�ϵ���ʱ10mS
	
	Ini_port();		    	//��ʼ���˿ڣ���AVDD
	INT_DIS
	
	//��ʼ��
	Init_Uart();
	Init_Flash();
	//Timer0_Init();	
	Timer3_Init();
		
	//��ʼ������  *********************************
	Delay_mS(30);			//��ʱ30mS
	Get_para();				//��ȡ������Ϣ
	resetAD7799();
	
	//�����ж�
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
	
	//Com_mode();			//ͨѶģʽ
	
	while(1);	
}

void Ini_port(void)
{	
	INTCON=0X00;			        //��ֹ�����ж�
	LATA=0X00;				//A�����������
	TRISA=0XFF;				//A��Ϊ����	
	//PORTA=0X00;				//A���ó�ֵ		
	ADCON1=0X3A;			//AN0~AN4ģ������ ,ѡ���ⲿVref��ѹ��׼��
	CMCON=0X07;				//A0��A1��A2��A3ΪIO�ڣ��Ƚ�����

	LATB=0;					//B�����������
	TRISB=0XE3;				//B��B1,B2,B3Ϊ���������Ϊ����
	PORTB=0X04;				//B���ó�ֵ
	
	LATC=0;					//C�����������
	TRISC=0XBF;				//C��Ϊ����
	PORTC=0X00; 				//C��Ϊ����

	LATD=0X00;				//D�����������
	TRISD=0X47;				//D��Ϊ	
	PORTD=0X88;

	LATE=0X00;				//E�����������
	TRISE=0XFB;				//E��Ϊ����	


	UCONbits.USBEN=0;		//��ֹUSBģ���֧�ֵ�·
	SSPCON1bits.SSPEN=0;	 
	ADCON0bits.ADON=0;		//��ֹADת��ģ��

	
}

void Get_pgain(void)
{	
	para.info.p_gain=R2550(Pgain_addr);		//��ȡѹ���Ŵ���
	para.info.p_gain=8;//20170710
	switch(para.info.p_gain)
	{
		case 2:{Pgain=0X01;break;}//2��
		case 4:{Pgain=0X02;break;}//4��
		case 8:{Pgain=0X03;break;}//8��
		case 16:{Pgain=0X04;break;}//16��
		case 32:{Pgain=0X05;break;}//32��		
		case 64:{Pgain=0X06;break;}//64��
		case 128:{Pgain=0X07;break;}//128��
		default:{Pgain=0X04;break;}//Ĭ��16��
	}	
	
	para.info.te_mode=R2550(TESTMODE_addr);	
	para.info.pointbyte=R2550(pointbyte_addr);
	para.info.adjzero.ch2[0]=R2550(Adjzero_addr);	//��ȡ������λ				
	para.info.adjzero.ch2[1]=R2550(Adjzero_addr+1);
	
	para.info.te_mode=0;//20170710
	para.info.pointbyte=4;//20170710
	para.info.adjzero.ch2[0]=0x88;//20170710
	para.info.adjzero.ch2[1]=0x13;//20170710
}

void Com_mode(void)
{
	uchar i;
//	Get_para();//��ȡ������Ϣ

	RX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	TX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	TX_TRIS=0;
	
	SPBRGH=0;				//������57600(4MHz)
	SPBRG=16;				// 16=57600(4MHz),51=19200
	BAUDCON=0X08;			       //16λ�����ʷ���������ֹ�����ʼ�⣬δ����RX����
	TXSTA=0X24;				//8λ���ͣ�ʹ�ܷ��ͣ������첽ģʽ
	RCSTA=0X10;				//�ݽ�ֹ���ڣ�8λ���գ�ʹ�ܽ���

	OSCCON=0X72;
	Delay_mS(60);		//20101209�¼�
	//Ini_port();				//��ʼ���˿ڣ���AVDD	
	
	i=R2550(Well1_addr);//�ն�������20101209�¼�
	RCSTAbits.SPEN=1;	//ʹ�ܴ���ͨ��

	INTCON=0XC0;	 	//���ж�
	while(1)
	{	
		Rinter=1;		//�����жϱ�־��1
		PIE1bits.RCIE=1;//�����ڽ����ж�
		while(Rinter);	//�ȴ�������Ч�����
		data74[0]=Rx();		//���յ�0���ֽ�
		if(data74[0]==0XAA)//����ͷΪAA�����ٽ������������ֽ�
		{
			data74[1]=Rx();		//���յ�1���ֽ�
			data74[2]=Rx();		//���յ�2���ֽ�
			switch(data74[2])//��Ϊ��Ч�����ֽڣ�����������ֽ�
			{
				case 0X83://��������Ϣ
				{			
					Txpack(Info_block);//������Ӧ��
					break;
				}
				case 0X84://д������Ϣ
				{							
					//Rxpack(Info_block);//������Ӧ��
					Rx_infpack(Info_block);//������Ӧ��
					break;
				}
				case 0X85://��ʱ���
				{			
					Txpack(Timetab_block);//������Ӧ��
					break;
				}
				case 0X86://дʱ���
				{							
					Rxpack(Timetab_block);//������Ӧ��
					break;
				}
				case 0X87://��У�Ա�
				{			
					Txpack(Caltab_block);//������Ӧ��
					break;
				}
				case 0X88://дУ�Ա�
				{						
					Rxpack(Caltab_block);//������Ӧ��
					break;
				}
				case 0X89://�������
				{				
					Get_pgain();
					Single_point();	
					break;
				}	
				case 0X8A://����������
				{
					Send_data();
					break;		
				}
				
				case 0X8C://ɾ������
				{
					for(i=3;i<9;i++)
						data74[i]=Rx();
					Delete_data();					
					Send_vrf(8);
					break;					
				}
				case 0X8D://���֣�ʹ��λ���б������Ĳ�����
				{
					for(i=3;i<9;i++)
						data74[i]=Rx();
					Send_vrf(8);
					break;	
				}
			
				case 0XC8://��������Ϣ
				{
					Send_testinfo();
					break;	
				}
				
				//ZFCY
				case 0X90://��������
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
	Get_pgain();//��ѹ���Ŵ���
	PreAD7799_P1();		
	resetAD7799();

	AllParaTest();	
	ReadPtCoff();
	AllParaCalc();
	
//	Timer3_Restart();
	
	g_rcvCmd = RCREG;// �����ڻ��沢�崮�ڽ����ж�
	g_rcvCmd = RCREG;// �����ڻ��沢�崮�ڽ����ж�
	g_rcvCmd = RCREG;// �����ڻ��沢�崮�ڽ����ж�
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
				//���汾��
				case ENUM_CMD_READVER:
					SendByteByUart(SOFT_VERSION);
					break;
				
				//������������
				case ENUM_CMD_READTESTRSLT:					
					SendAllDataByUart();
					break;
				//������������
				case ENUM_CMD_READTESTORG:
					SendAllDataOrgByUart();				
					break;
					
				default:
					break;
			}
			
			g_rcvCmd = RCREG;// �����ڻ��沢�崮�ڽ����ж�
			g_rcvCmd = RCREG;// �����ڻ��沢�崮�ڽ����ж�
			g_rcvCmd = RCREG;// �����ڻ��沢�崮�ڽ����ж�
			g_rcvCmd = RCREG;// �����ڻ��沢�崮�ڽ����ж�
			RCSTAbits.CREN=0;//��ֹ���ڲ��崮������жϱ�־
	            RCSTAbits.CREN=1;//ʹ�ܴ���
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

//�������в���,�����ݱ����ڻ�����
void AllParaTest(void)
{
	
	//����ѹ���¶�
	ReadAD_MODE0();				
	//DEBUG
	//data74[1] = 0x00;
	//data74[0] = 0x00;
	g_testRslt.unPress.i8[0] = data74[0];
	g_testRslt.unPress.i8[1] = data74[1];
	g_testRslt.unTemp.i8[0] = data74[2];
	g_testRslt.unTemp.i8[1] = data74[3];
	
	//��������
	g_testRslt.unReserve.i8[1] = 0;
	g_testRslt.unReserve.i8[0] = 0;
	
	//�����λ��
	//g_testRslt.pos = g_motorPos;
	
	//���Ե�ѹ
	AD_10BITNew();		//  ADCON2=0x21; 
	g_testRslt.vol = AD_value.ch2[1] + 1;
}

//�Բ��Եõ��Ĳ������м���
void AllParaCalc(void)
{
	uint16 pRslt, tRslt;
	uint8 vRslt;
	
	//����ѹ���¶�     

	PTCalc(g_testRslt.unPress.i16, g_testRslt.unTemp.i16, &pRslt, &tRslt);
//	PTCalc(13465, 30834, &pRslt, &tRslt);//10MPa 20C
	
	g_calcRslt.unPress.i16 = pRslt;
	g_calcRslt.unTemp.i16 = tRslt;
	
	//�����ѹ
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
	
//����ר��
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
//  ��ѹ���¶�У�Ա�
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
//   ѹ���¶ȹ���������  
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
	

	//�����������д���������������	
	pVal+=5;
	pVal *= 1000;
	if(pVal<0){pVal=0;}//������
	*pRlst = (uint16)pVal;
	
	tVal += 50;
	tVal *= 100;
	if(tVal<0){tVal=0;}//������
	*tRlst = (uint16)tVal;
}

void VoltCalc(uint8 voltHz, uint8 *pVolt)
{
	
	*pVolt = voltHz;
}

