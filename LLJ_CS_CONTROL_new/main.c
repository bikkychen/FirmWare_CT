#include "define.h"
/********************************
        ȫ�ֱ�������   20160916 У���0x9d15   V1.0  FOR V1.0��·�� CDLJAϵ��������    ʹ���ڲ�8MHz-----PWM����ź�
          �����ܺ�У���0x9d15        4о�ӿڼ���--------��ʯ��Դ�Ƽ�
*********************************/   
/*
20170521�Ľ���
1���汾������ΪV3.0
2����device.c�ļ���Get_para()�Ӻ����й̶��洢����Ϊ1���̶�����ģʽΪ8
3����main.c�ļ���Get_pgain()�Ӻ����й̶��Ŵ���Ϊ8������ģʽΪ8�������ֽ���Ϊ8��ѹ��������λΪ5000
4����main.c�ļ���Test0_mode()�Ӻ������ڶ�ȡʱ�����������ĵ�ǰʱ�����Ĳ�������Ϊ0x8000���������Ϊ5
5����main.c�ļ���Test_mode()�Ӻ������������ж�Ԥ���ֽڵĴ����
6����main.c�ļ���Test_mode()�����н�ѹ������������ֵ�жϹ�������
7����main.c�ļ���Test_mode()�����н���������32��Ϊ���ڵ���32
8����main.c�ļ���Test_mode()������ÿ�ھ�����ʱд��һ���̶���ʱ�����ھ����׿�
*/

union Lch4 value;//PT_value,;	//ѹ�����¶ȱ���
union Allinfo para;			//������Ϣ�����壬240�ֽ�
union Lch2 valid_block,AD_value;		//FLASH��Ч��,ADת��ֵ
uint valid_excursion,Vbat_excursion;		//FLASH��Чƫ����
uchar Rinter,data74[74]={0};//���ڽ����жϱ�־�����Ͱ�����հ��Ĵ������
uchar T1_flag=0,Pgain=0,Int_dealy;	//TIMER1�жϱ�־��ѹ���Ŵ���
//uchar Pgain=0,Int_dealy;

# pragma udata wlBuf
uint g_wlValAr[16];		//�������鱣�������������ֵ

uchar g_wlCnt;
uchar g_wlModeFlag = 0;
uchar g_wlCntHigh, g_wlCntLow;
uchar MYDBU = 1;
# pragma udata

#define SOFT_VERSION	0x30
/******************************************************************
 �жϺ������������main()����ǰ�棬��Ӧ�жϲ��ܽ�����Ӧ�жϴ�����
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
	Nop();Nop();Nop();			//�ղ�������ֹ˯�߻��Ѻ��ڲ��������ȶ�
	if(PIE1bits.TMR1IE&PIR1bits.TMR1IF)
	{
		INTCON=0X00;		//��ֹ�ж�
		
		if (g_wlModeFlag == 0)
		{
			for(Int_dealy=183;Int_dealy>0;Int_dealy--);//1.464mS��20101029�¼�
			PIR1bits.TMR1IF=0;	//����TIMER1�жϱ�־
			T1CONbits.TMR1ON=0;	//�ر�TIMER1	
			T1_flag=1;			//TIMER1�жϱ�־��1
			data74[50]=1;
			INTCON=0XC0;		//�ؿ��ж�	
		}	
		else
		{
			PIR1bits.TMR1IF=0;	//����TIMER1�жϱ�־
			TMR1H=0xF0;
			TMR1L=0x00;		//��ʱ������ֵ��1/8 s
			
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
			 
			 INTCON=0XC0;		//�ؿ��ж�	
		}	
		
	}
	if(PIR1bits.RCIF)
	{
		PIE1bits.RCIE=0;	//��ֹ���ڽ����ж�
		Rinter=0;			//�����жϱ�־��0
	}
}
/****************************************************
                      ������
*****************************************************/
void main(void)
{	
	uchar temp;				//��λʱ�ڲ�������Ĭ�����Ƶ��1MHz
	Delay_mS(60);			//�ϵ���ʱ��1MHz�ڲ�����Ƶ��ʱ��ԼΪ120ms
	//OSCCON=0X72;    //0X72;	0X62		//ִ��SLEEPָ����������ģʽ���ⲿ������Ƶ��8MHz
	OSCCON=0X40;
	//while(OSCCONbits.IOFS==0);//0X62�ȴ��ڲ�4MHzʱ���ȶ�	
	Delay_mS(200);			//�ϵ���ʱ100mS				
	//Open_avdd();			//��AVDD
	Ini_port();		    //��ʼ���˿ڣ���AVDD

	Nop();
	//Delay_mS(122);
	Nop();
	//AD_value.l=RecAD7799();
	Nop();
	
	Delay_mS(60);			//��ʱ30mS
	

	Write_Flash_unpro(0);	//FLASHȡ��Ƭ����		
	temp=Read_status(0); 	//��FLASH״̬�Ĵ���	
	if(temp&0X0C)
	{
		Write_Flash_unpro(0);//��ֹ��һ��ȡ��Ƭ����ʧ��
		Delay_mS(10);
	}
	
	/*
	AD_value.l=0x5678;
	Write_DF321page(5,2,2,AD_value.ch2);
	AD_value.l=0x0000;*/	
//	Read_DF321page(5,0,2,AD_value.ch2);

	Get_para();//��ȡ������Ϣ
	
	resetAD7799();



	if (MYDBU != 0)
	{
		Ccp1_Init();	
	}
	

		
	if(RX==0)				//��״̬��Ϊ1���������ģʽ��Ϊ0������ͨѶģʽ
		Test_mode();		//����ģʽ
	else
	{	
		Com_mode();			//ͨѶģʽ
	}
	while(1);
	
}

void Ini_port(void)
{	
	INTCON=0X00;			//��ֹ�����ж�
	LATA=0X00;				//A�����������
	TRISA=0X10;				//A4��Ϊ����	
	PORTA=0X20;				//0x30---A���ó�ֵSHDN=1
		
	ADCON1=0X0E;			//A0Ϊģ�����롢����A1��A2��A3��ΪIO��
	CMCON=0X07;				//�Ƚ�����

	LATB=0;					//B�����������
	TRISB=0X08;				//B��B1,B2,B3Ϊ���������Ϊ����
	PORTB=0X01;				//B���ó�ֵ
	
	LATC=0;					//C�����������
	TRISC=0X82;				//C��Ϊ����
	PORTC=0X03; 				//C��Ϊ����

	LATD=0X00;				//D�����������
	TRISD=0X10;				//D��Ϊ	
	PORTD=0X30;

	LATE=0X00;				//E�����������
	TRISE=0X00;				//E��Ϊ���
	PORTE=0X00;

	T0CON=0X28;
	UCONbits.USBEN=0;		//��ֹUSBģ���֧�ֵ�·
	SSPCON1bits.SSPEN=0;	        //��ֹSPI��I2C����
	ADCON0bits.ADON=0;		//��ֹADת��ģ��

	
}

void Ccp1_Init(void)
{
	//add pwm
	TRISC &= ~(1<<2);				//RC2Ϊ���
	
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
	T1CON=0XCE;          //ʹ��TIMER1����,0X4EԤ��Ƶ1��1��0X7EԤ��Ƶ8��1���첽ģʽ;	
	
	PIE1bits.TMR1IE=1;   //����Timer1�ж�	
	
	TMR1H=0xF0;
	TMR1L=0x00;		//��ʱ������ֵ��1/8 s
		
	T1CONbits.TMR1ON=1;//����TMR1
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
	
	T0CONbits.TMR0ON=1;	//����TMR0		
}

void Init_WlSample(void)
{
	g_wlModeFlag = 1;
	
	Init_Timer0();
	Init_Timer1();	
}
			
void Open_avdd(void)
{		
	TRISB=0X08;				//B��B1,B2,B3Ϊ���������Ϊ����
	PORTB=0X01;				//B���ó�ֵCSΪ��
	
	TRISC=0X02;				//C��Ϊ����TRISC=0X82;
	PORTC=0X03;				//C������

	TRISE=0X00;				//E��Ϊ����
	PORTE=0X00;

	TRISD=0X10;				//D��Ϊ	
	PORTD=0X30;

	TRISA=0X10;				//A��A4Ϊ����T0
	PORTA=0X20;
	//set timer2

	//T2CONbits.TMR2ON = 1;

	//set CCP1 as pwm
	//CCP1CONbits.CCP1M3 = 1;	
	//CCP1CONbits.CCP1M2 = 1; 
	//CCP1CONbits.CCP1M1 = 0;	
	//CCP1CONbits.CCP1M0 = 0;

	SHDN=1;					//��AVDD,��FLASH��AD����󣬲Ŷ���ӦIO�ڲ���
	//Nop();Nop();			
	//Nop();Nop();			
	//FLASH�˿�
	//F1CS1					//����FLASHƬѡ����1	
	//AD7799�˿�
	//CS1						//����AD7799Ƭѡ����1	
}

void Close_avdd(void)
{//�ص�ǰ����AVDD�йصĿ���Ϊ���0
	TRISA=0X10;				//
	PORTA=0X10;				//SHDN=1 SHDN1=0  SHDN���0���ر�AVDD
	SHDN=0;	
	//set timer2

//	T2CONbits.TMR2ON = 0;

	//set CCP1 as pwm
//	CCP1CONbits.CCP1M3 = 0;	
//	CCP1CONbits.CCP1M2 = 0; 
//	CCP1CONbits.CCP1M1 = 0;	
//	CCP1CONbits.CCP1M0 = 0;
			
	TRISC=0X02;		// 1  0  0  0  0  0  1  0  //82							
	PORTC=0X03;	    //C0,C1,C7ֵ����//PORTC&=0X83;
	
	TRISB=0X00;				//0000  0000��FLASH�йص�4���������0
	PORTB=0X00;				//B������

	TRISE=0X00;		//
	PORTE=0X00;	

	
	TRISD=0X00;    //0000   0000
	PORTD=0X00;             //
	
}

void Get_pgain(void)
{	
	para.info.p_gain=R2550(Pgain_addr);		//��ȡѹ���Ŵ���
	para.info.p_gain=8;//20170521
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
	
	para.info.te_mode=8;//20170521
	para.info.pointbyte=8;//20170521
	para.info.adjzero.ch2[0]=0x88;//20170521
	para.info.adjzero.ch2[1]=0x13;//20170521
}

void Com_mode(void)
{
	uchar i;
//	Get_para();//��ȡ������Ϣ
	
	RX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	TX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	SPBRGH=0;				//������57600(4MHz)
	//SPBRG=16;				// 16=115200(8MHz)
	SPBRG=34;				// 57600 (8MHZ)
	BAUDCON=0X08;			       //16λ�����ʷ���������ֹ�����ʼ�⣬δ����RX����
	TXSTA=0X24;				//8λ���ͣ�ʹ�ܷ��ͣ������첽ģʽ
	RCSTA=0X10;				//�ݽ�ֹ���ڣ�8λ���գ�ʹ�ܽ���

	//OSCCON=0X72;
	//Delay_mS(60);		//20101209�¼�
	//Ini_port();				//��ʼ���˿ڣ���AVDD	
	//resetAD7799();
	i=R2550(Well1_addr);//�ն�������20101209�¼�
	RCSTAbits.SPEN=1;	//ʹ�ܴ���ͨ��
	INTCON=0XC0;	 	//���ж�
	
	 Init_WlSample();
	
	while(1)
	{	
		Rinter=1;		//�����жϱ�־��1
		PIE1bits.RCIE=1;//�����ڽ����ж�
		while(Rinter);	//�ȴ�������Ч�����
		data74[0]=Rx();		//���յ�0���ֽ�
		if(data74[0]==0XE8)//����ͷΪE8�����ٽ������������ֽ�
		{
			data74[1]=Rx();		//���յ�1���ֽ�
			if(data74[1]==0X40)//����ͷΪ40�����ٽ������������ֽ�
			{
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
				//�µĶ��������ݣ������б�����ʼ��ַ����ʼ����
				case 0x9A:
				{
					Send_dataNew();
					break;
				}
				case 0X8B://��������
				{
					
					Txblockpack();//���Ϳ���Ӧ���ݰ�
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
						
						//
						data74[3]=SOFT_VERSION;//�̼��汾
						
					Send_vrf(8);
					break;	
				}
				case 0XD8://д������Ϣ
				{							
					Rx_Testinfo();//������Ӧ��
					break;
				}
				case 0XC8://��������Ϣ
				{
					Send_testinfo();
					break;	
				}	
				case 0X8E://��ID��Ϣ
				{			
					Send_IDinfo();//������Ӧ��
					break;
				}
				case 0X8F://����ʽ����Ϣ
				{			
					Txpack(err_block);//������Ӧ��
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

	//Get_para();//��ȡ������Ϣ
	T1CON=0X4E;          //ʹ��TIMER1����,0X4EԤ��Ƶ1��1��0X7EԤ��Ƶ8��1���첽ģʽ;
	RCSTAbits.SPEN=0;//��ֹ�첽�շ�
	PORTCbits.RC7=0;//RX
	TRISCbits.TRISC7=0;//RX_TRIS
	RX=0;//ָʾ�Ƴ�ʼ״̬����
	TX_TRIS=0;
	bn=BLOCKNUM;
	if(para.info.mem_num==1)
	{
		bn=BLOCKNUM+1024;
	}

	/*******************************************************
		���Ѿ���32�ھ�����������˸2���Ա���
		��ʱ10S������δ�ϵ磬��ɾ�����ݣ���ͷ��ʼ�洢
	*******************************************************/
	if(para.info.well>=32)
	{
		RX=0;Delay_mS(3000);   
		for(temp=0;temp<2;temp++)
		{//FORѭ������ʱ8S��ҲΪ32768�����ṩ��ʱ��
			RX=1;Delay_mS(3000);//��3S	
			RX=0;Delay_mS(1000);//��1S
		}
		Delay_mS(10000);
		//ѭ����Ԫ�еĵ�ǰ�飬Ҳ�ǵ�32�ھ���ĩ�飬Ҳ����ɾ����������
		i=Loop_addr+((para.info.loop_point)<<1);//�õ���ǰ��
		pnormal.ch2[0]=R2550(i);
		i++;
		pnormal.ch2[1]=R2550(i);		
		//pnormal.l+=2; 					//ĩ�ھ�����+=2����ɾ��2��
		
		data74[4]=0X05;
		data74[5]=0X00;					//�׿飺3
		data74[6]=pnormal.ch2[0];
		data74[7]=pnormal.ch2[1];		//����
		Delete_data();					//ɾ������
		Get_para();						//���»�ȡ������Ϣ
	}
/*********************************************
	��������Ч�����ж��׿�valid_block�Ƿ���Ч
**********************************************/	
	if(para.info.well==0)
		valid_block.l=0X05;
	else
	{//��ѭ��λ��ָ����ָ��ѭ����Ԫ�У��ҳ����飬�õ��¾����׿�
		temp=Loop_addr+((para.info.loop_point)<<1);
		valid_block.ch2[0]=R2550(temp);
		temp++;
		valid_block.ch2[1]=R2550(temp);
	}

	do
	{
		Etabaddr=(valid_block.l>>3);
		Read_DF321page(0,Etabaddr,1,data74+9);//�������
		i=data74[9];
		i>>=(valid_block.l&0X07);
		i&=0X01;
		if(i==0)//���Ǻÿ�
		{
			Delay_mS(15);//20101102�¼�
			Read_DF321page(valid_block.l,0,4,value.ch4);
			if(value.l==0XFFFFFFFF)		
				break;	//�����õ��¿�������
		}	
		valid_block.l++;
	}while(valid_block.l<bn);

	if(valid_block.l==bn)
		Sleep_forever();//������Ч�洢�鷶Χ
/*********************************************
	��������Ч�����׿�Ҳ��Ч�����ж�AD�Ƿ�����
**********************************************/
	Get_pgain();//��ѹ���Ŵ���
	//resetAD7799();
//	Read_pt(1);  //ָʾ����˸ǰ�ж�AD�Ƿ�����
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
		temp=0;//AD����
	else
		temp=1;//AD������
	while(temp)
	{	RX=1;  //���ָʾ����
	}*/
/*********************************************
	��������Ч���׿���Ч��AD���������������
**********************************************/
		
	if(para.info.loop_point>6)
		para.info.loop_point=0;
	else
		para.info.loop_point++;
	temp=Loop_addr+((para.info.loop_point)<<1);
	WW2550(temp,valid_block.ch2[0]);   //���׿�д��ѭ��λ��
	temp++;
	WW2550(temp,valid_block.ch2[1]);
	WW2550(Looppoint_addr,para.info.loop_point);//����ѭ��λ��ָ��
	//AD_10BIT();  //��ص�ѹ	
	temp=Well1_addr+(para.info.well*7);  //6);//-----20130425
	WW2550(temp,valid_block.ch2[0]);   //�����¾��׿�
	temp++;
	WW2550(temp,valid_block.ch2[1]);
	temp++;

	WW2550(Well_addr,para.info.well+1);//����+1
	WW2550(Count_addr,0X00);		   //�ü����־Ϊ0

	//for(i=0;i<5;i++)
	//{//�����ϵ�ʱ��ѹ����λ�������¾��׿��ʱ���֮��
	//Ϊ��ֹFLASH����Ӱ���ѹ����Ӱ��AD����˰�AD�ɼ����ڲ���FLASH֮ǰ
	//	Read_pt();
	//	temp=i;
	//	temp<<=2;
	//	zero[temp+0]=PT_value.ch4[0];
	//	zero[temp+1]=PT_value.ch4[1];
	//	zero[temp+2]=PT_value.ch4[2];
	//	zero[temp+3]=PT_value.ch4[3];
//	}
	
//	Delay_mS(1000);//20101102Ϊ��Ӧ���ʱ�����ϵ����
	for(i=0;i<64;i++)
	{                  //��ȡʱ���,��д���¾��׿�
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
	//����������������˸3��,�����붨ʱ�ɵ�׶�
	for(temp=0;temp<3;temp++)
	{//FORѭ������ʱ6S��ҲΪ32768�����ṩ��ʱ��
		RX=1;Delay_mS(2000);//��
		RX=0;Delay_mS(2000);//��
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
	//��Ч��Ϊvalid_block
	Mode=para.info.te_mode; 

	valid_excursion=512+para.info.pointbyte;//yh20110419  ��Чƫ����Ϊ528	
	//para.info.adjzero.l
	Vbat_excursion=0;
	      //para.info.adjzero.ch2[1]>>5;
	//Mode=Mode&0x07;

	Test0_mode();//����ѹ���Ʋ���ģʽ

	/*switch(Mode)
	{
		case 0://
		{	
			Test0_mode();//����ѹ���Ʋ���ģʽ
			break;
		}
		case 3://
		{	
			Test1_mode();//����ѹ�����������¶Ȳ���,���㿼��6���ֽڵĲ���
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
	
	T1CON=0X4E;          //ʹ��TIMER1����,0X4EԤ��Ƶ1��1��0X7EԤ��Ƶ8��1���첽ģʽ;
	RCSTAbits.SPEN=0;    //��ֹ�첽�շ�
	PORTCbits.RC7=0;     //RX
	TRISCbits.TRISC7=0;  //RX_TRIS
	RX=0;                //ָʾ�Ƴ�ʼ״̬����
	TX_TRIS=0;
	
	//T3CON=0X0E;          //ʹ��TIMER3����,Ԥ��Ƶ1��1��65536=2S
	
	INTCON=0XC0;	     //˯��ǰʹ���ж�
	PIE1bits.TMR1IE=1;   //����Timer1�ж�	
	//PIE2bits.TMR3IE=1;   //����Timer3�ж�
	//IPR2bits.TMR3IP=1;   //TMR3 ����жϸ����ȼ�λ
	//go_on=1;
	Read_DF321page(Timetab_block,0,8,time.ch8);//��ȡ��һ��ʱ���8�ֽ�
      time.tab.looptime.l=0x8000;//20170521
      time.tab.interval.l=5;//20170521
	for(temp=1;(temp<=64)&&(time.tab.interval.l>1)&&(time.tab.looptime.l>0);temp++)
	{	
		//SHANG=(time.tab.interval.l-1)>>4;		//��
		//YU=(time.tab.interval.l-1)&0XF;		//����
		SHANG=(time.tab.interval.l-2)>>1;		//��
		YU=(time.tab.interval.l-2)&0X1;		//����
		//T1H=YU;	
		//T1H=256-(YU<<4);
		Sample_flag=0;
		if(time.tab.looptime.l>0)	//�ѶԳ����͵��ж�תΪ���ַ��͵��жϣ�ʹ��ʱ����׼ȷ
			go_on=1;
		else
			go_on=0;
		if(time.tab.interval.l>2)    //�������>2S���͹���20160915
			Close_avdd();              //20160915
						
		do
		{				
			if(time.tab.interval.l>2)				//���������>2S
			{				
				SHANG_temp=SHANG;
				while(SHANG_temp>0)
				{
					TMR1L=6;		//��ʱ������ֵ��2s
					TMR1H=0X00;	
					T1CONbits.TMR1ON=1;//����TMR1
					SHANG_temp--;
					Sleep();
				}
				T1H=YU;
				if(T1H>0)
				{
					TMR1L=6;		//��ʱ������ֵ��2s
					TMR1H=0X80;	
					T1CONbits.TMR1ON=1;//����TMR1
					Sleep();
				}
				
			}
				Open_avdd();
				F1_value.l=0;
				F2_value.l=0;
				
				EXC=1;
				CS0
				TMR1L=26;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;
				T1_flag=0;				
				T1CONbits.TMR1ON=1;	//����TMR1
				while(T1_flag==0);
				//TEST=0;				
				EXC=0;					
				T1_flag=0;
				TMR1L=16;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				while(T1_flag==0);	
							
				EXC=1;				
				T1_flag=0;
				TMR1L=16;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				while(T1_flag==0);
				
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				while(T1_flag==0);
			//-------------------�ɼ��¶�--20160915
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				ReadAD7799_16T1();
				data74[2]=value.ch4[1];
				data74[3]=value.ch4[2];
				while(T1_flag==0);
			//-------------------�ɼ�ѹ��--20160915	
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				ReadAD7799_16P1();
				data74[0]=value.ch4[0];
				data74[1]=value.ch4[1];
				while(T1_flag==0);	
			//--------------------------------------------
				EXC=1;				
				T1_flag=0;
				TMR1L=16;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				while(T1_flag==0);
				
			//-------------------�ɼ�����--20160916
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160916�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
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
				T1CONbits.TMR1ON=1;	//����TMR1
				Delay_mS(106);
				ReadAD7799_16F2();
				F2_value.l+=AD_value.l;
				while(T1_flag==0);
				//-----------------------------1
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160916�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				Delay_mS(106);
				ReadAD7799_16F1();
				F1_value.l+=AD_value.l;
				while(T1_flag==0);
				
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				Delay_mS(106);
				ReadAD7799_16F2();
				F2_value.l+=AD_value.l;
				while(T1_flag==0);
				//-----------------------------2
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160916�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				Delay_mS(106);
				ReadAD7799_16F1();
				F1_value.l+=AD_value.l;
				while(T1_flag==0);
				
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				Delay_mS(106);
				ReadAD7799_16F2();
				F2_value.l+=AD_value.l;
				while(T1_flag==0);
				//-----------------------------3
				EXC=0;	
				T1_flag=0;
				TMR1L=16;			//20160916�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				Delay_mS(106);
				ReadAD7799_16F1();
				F1_value.l+=AD_value.l;
				while(T1_flag==0);
				
				EXC=1;	
				T1_flag=0;
				TMR1L=16;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
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
				TMR1L=16;			//20160915�¸ģ���ʱ������ֵ
				TMR1H=0XF0;					
				T1CONbits.TMR1ON=1;	//����TMR1
				AD_value.l=3;
				CS1
				//RX=1;         //20130618				
				Sample_save();	//�����洢һ��
				//RX=0;         //20130618	
				//TEST=0;			
				time.tab.looptime.l--;//ѭ��������1	
				if(time.tab.interval.l>2)				//���������>2S
					Close_avdd();

				if(time.tab.looptime.l>0)
					go_on=1;
				else
					go_on=0;
				while(T1_flag==0);
				
				//T1CON=0X7E;			 				
			//}
			
		}while(go_on);				//��ѭ������>0�������ѭ�������������һ��ʱ���
		//Open_avdd();
		//Delay_mS(25); 
		pnormal.l=temp;
		pnormal.l<<=3;
		Read_DF321page(Timetab_block,pnormal.l,8,time.ch8);//������һ��ʱ���
		time.tab.looptime.l=0x8000;//20170521
      	time.tab.interval.l=5;//20170521
	}
	Sleep_forever();
}