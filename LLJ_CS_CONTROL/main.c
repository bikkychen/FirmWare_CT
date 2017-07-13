#include "define.h"
/********************************
        ȫ�ֱ�������   20151118 У���0x34ac      V1.5  FOR CSLJAϵ��������
          ���ܺ�У���0x03a4         4о�ӿڼ���--------��ʯ��Դ�Ƽ�
*********************************/   
union Lch4 value;//PT_value,;	//ѹ�����¶ȱ���
union Allinfo para;			//������Ϣ�����壬240�ֽ�
union Lch2 valid_block,AD_value;		//FLASH��Ч��,ADת��ֵ
uint valid_excursion,Vbat_excursion;		//FLASH��Чƫ����
uchar Rinter,data74[74]={0};//���ڽ����жϱ�־�����Ͱ�����հ��Ĵ������
uchar T1_flag=0,Pgain=0,Int_dealy;	//TIMER1�жϱ�־��ѹ���Ŵ���

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
		for(Int_dealy=183;Int_dealy>0;Int_dealy--);//1.464mS��20101029�¼�
		PIR1bits.TMR1IF=0;	//����TIMER1�жϱ�־
		T1CONbits.TMR1ON=0;	//�ر�TIMER1	
		T1_flag=1;			//TIMER1�жϱ�־��1
		INTCON=0XC0;		//�ؿ��ж�
	}
	if(PIR1bits.RCIF)
	{
		PIE1bits.RCIE=0;	//��ֹ���ڽ����ж�
		Rinter=0;			//�����жϱ�־��0
	}
}

void Debug(void)
{
	union Lch8 time;
	unsigned char i;
	unsigned int j;
	
	time.tab.interval.l = 0x01020304;
	time.tab.looptime.l = 0x05060708;	
	
	for (i=0; i<64; i++)
	{
		data74[9+i] = i;
	}	
	//if(temp_block==Caltab_block)
	
	j = 0;
	j<<=6;
	if (1)
	{//��ΪУ�Ա���Ϊ4096�ֽڣ�Ϊ��ֹ����дFLASH������дǰ���һ����ʱ
		Delay_mS(5);
		Write_DF321page(9,j,64,data74+9);
		Delay_mS(85);
	}
	
	
	for (i=0; i<64; i++)
	{
		data74[9+i] = 0;
	}
	j = 0;
	j<<=6;
	Read_DF321page(9,j,64,data74+9);
	
	//*****************************************
	for (i=0; i<64; i++)
	{
		data74[9+i] = i;
	}	
	//if(temp_block==Caltab_block)
	
	j = 2;
	j<<=6;
	if (1)
	{//��ΪУ�Ա���Ϊ4096�ֽڣ�Ϊ��ֹ����дFLASH������дǰ���һ����ʱ
		Delay_mS(5);
		Write_DF321page(Caltab_block,j,64,data74+9);
		Delay_mS(5);
	}
	
	
	for (i=0; i<64; i++)
	{
		data74[9+i] = 0;
	}
	j = 2;
	j<<=6;
	Read_DF321page(Caltab_block,j,64,data74+9);
	
	Delay_mS(10);
}

/****************************************************
                      ������
*****************************************************/
void main(void)
{	
	uchar temp;				//��λʱ�ڲ�������Ĭ�����Ƶ��1MHz
	Delay_mS(60);			//�ϵ���ʱ��1MHz�ڲ�����Ƶ��ʱ��ԼΪ120ms
	OSCCON=0X40;			//ִ��SLEEPָ����������ģʽ���ⲿ������Ƶ��8MHz
	//while(OSCCONbits.IOFS==0);//�ȴ��ڲ�4MHzʱ���ȶ�	
	Delay_mS(200);			//�ϵ���ʱ100mS				
	Open_avdd();			//��AVDD
	Ini_port();		    //��ʼ���˿ڣ���AVDD

//Debug();

	Nop();
	//WriteAD8343(0XA6);
	Nop();
	//AD_value.l=RecAD8343();
	Nop();

	Delay_mS(60);			//��ʱ30mS
	Get_para();//��ȡ������Ϣ
	//resetAD8343();

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

	//Debug();
	
	Delay_mS(4000);
	if(RX==0)				//��״̬��Ϊ1���������ģʽ��Ϊ0������ͨѶģʽ
		Test_mode();		//����ģʽ
	else
		Com_mode();			//ͨѶģʽ
	while(1);
	
}


	
void Ini_port(void)
{	
	INTCON=0X00;			//��ֹ�����ж�
	LATA=0X00;				//A�����������
	TRISA=0X10;				//A��Ϊ����	
	PORTA=0X30;				//A���ó�ֵ
		
	ADCON1=0X0F;			//A0��A1��A2��A3ΪIO��
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
	TRISE=0X04;				//E��Ϊ����
	PORTE=0X00;

	T0CON=0X28;
	UCONbits.USBEN=0;		//��ֹUSBģ���֧�ֵ�·
	SSPCON1bits.SSPEN=0;	        //��ֹSPI��I2C����
	ADCON0bits.ADON=0;		//��ֹADת��ģ��

	
}

void Open_avdd(void)
{		
	TRISB=0X08;				//B��B1,B2,B3Ϊ���������Ϊ����
	PORTB=0X01;				//B���ó�ֵCSΪ��
	
	TRISC=0X82;				//C��Ϊ����TRISC=0XBF;
	PORTC=0X03;				//C������

	TRISE=0X04;				//E��Ϊ����
	PORTE=0X00;

	TRISD=0X10;				//D��Ϊ	
	PORTD=0X30;

	TRISA=0X10;				//A��A4Ϊ����T0
	PORTA=0X30;

	SHDN=1;					//��AVDD,��FLASH��AD����󣬲Ŷ���ӦIO�ڲ���
	//Nop();Nop();			
	//Nop();Nop();			
	//FLASH�˿�
	//F1CS1					//����FLASHƬѡ����1	
	//AD8343�˿�
	//CS1						//����AD8343Ƭѡ����1	
}

void Close_avdd(void)
{//�ص�ǰ����AVDD�йصĿ���Ϊ���0
	TRISA=0X10;				//
	PORTA=0X00;				//SHDN���0���ر�AVDD
	SHDN=0;	
				
	TRISC=0X82;		// 1  0  0  0  1  0  1  0  //8A							
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
}

void Com_mode(void)
{
	uchar i;
//	Get_para();//��ȡ������Ϣ

	RX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	TX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	SPBRGH=0;				//������57600(4MHz)
	//SPBRG=16;				// 16=57600(4MHz),51=19200
	//SPBRG=25;				// baud = clk / (4 * (N + 1) )
	SPBRG=34;
	BAUDCON=0X08;			       //16λ�����ʷ���������ֹ�����ʼ�⣬δ����RX����
	TXSTA=0X24;				//8λ���ͣ�ʹ�ܷ��ͣ������첽ģʽ
	RCSTA=0X10;				//�ݽ�ֹ���ڣ�8λ���գ�ʹ�ܽ���

	//OSCCON=0X72;
	//Delay_mS(60);		//20101209�¼�
	//Ini_port();				//��ʼ���˿ڣ���AVDD	
	//resetAD8343();
	i=R2550(Well1_addr);//�ն�������20101209�¼�
	RCSTAbits.SPEN=1;	//ʹ�ܴ���ͨ��
	INTCON=0XC0;	 	//���ж�
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
					//TxPackDebug();
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
					//20160529
					data74[3] = SOFT_VER;
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
	uchar temp,i,mm,Mode=0;
	uchar Etabaddr;
	uint j,bn;
	//ulong SHANG,SHANG_temp;
	union Lch2 pnormal,tnormal,t1normal;
	union Lch8 time;

	//Get_para();//��ȡ������Ϣ
	T1CON=0X7E;//ʹ��TIMER1����,Ԥ��Ƶ8��1���첽ģʽ;
	RCSTAbits.SPEN=0;//��ֹ�첽�շ�
	PORTCbits.RC7=0;//RX
	TRISCbits.TRISC7=0;//RX_TRIS
	RX=0;//ָʾ�Ƴ�ʼ״̬����
	TX_TRIS=0;
	bn=BLOCKNUM;
	

	/*******************************************************
		���Ѿ���32�ھ�����������˸2���Ա���
		��ʱ10S������δ�ϵ磬��ɾ�����ݣ���ͷ��ʼ�洢
	*******************************************************/
	if(para.info.well==32)
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
	//resetAD8343();
//	Read_pt(1);  //ָʾ����˸ǰ�ж�AD�Ƿ�����
//	pnormal.ch2[0]=AD_value.ch2[0];
//	pnormal.ch2[1]=AD_value.ch2[1];
	PreAD8343_P1();
	//PreAD8343_T1();
	ReadAD_MODE0();
	tnormal.ch2[0]=data74[0];
	tnormal.ch2[1]=data74[1];
	if((tnormal.l>=0)&&(tnormal.l<0XFFFF))
		temp=0;//AD����
	else
		temp=1;//AD������

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

//	Delay_mS(1000);//20101102Ϊ��Ӧ���ʱ�����ϵ����
	for(i=0;i<64;i++)
	{                  //��ȡʱ���,��д���¾��׿�
		pnormal.l=i;
		pnormal.l<<=3;	
		Read_DF321page(Timetab_block,pnormal.l,8,data74);
		
		//д��̶�ʱ���, 20160605	
		//time.tab.interval.l = 3;
		//time.tab.looptime.l = 0xFFFFFFFF;
		//for (temp=0; temp<8; temp++)
		//	data74[temp] = time.ch8[temp];
		
		Delay_mS(10);	
		Write_DF321page(valid_block.l,pnormal.l,8,data74);
		Delay_mS(15);
	}			
		
	pnormal.l=512;

	ReadAD_MODE8();		
	Write_DF321page(valid_block.l,pnormal.l,8,data74);
	Delay_mS(18);
	pnormal.l+=8;
	


	temp=para.info.te_mode+1;    // /*
	
	
	temp=Erase_DF321block(Vbattab_block);
	//��Ч��Ϊvalid_block
	Mode=para.info.te_mode; 

	valid_excursion=512+8;//yh20110419  ��Чƫ����Ϊ528	
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
	union Lch8 time;
	

	
	T1CON=0X7E;          //ʹ��TIMER1����,Ԥ��Ƶ8��1���첽ģʽ;
	RCSTAbits.SPEN=0;    //��ֹ�첽�շ�
	PORTCbits.RC7=0;     //RX
	TRISCbits.TRISC7=0;  //RX_TRIS
	RX=0;                //ָʾ�Ƴ�ʼ״̬����
	TX_TRIS=0;

	INTCON=0XC0;	     //˯��ǰʹ���ж�
	PIE1bits.TMR1IE=1;   //����Timer1�ж�	

	//go_on=1;
	Read_DF321page(Timetab_block,0,8,time.ch8);//��ȡ��һ��ʱ���8�ֽ�
	
	
	//д��̶�ʱ���, 20160605	
	time.tab.interval.l = 3;
	time.tab.looptime.l = 0xFFFFFFFF;
	

	//uart debug *****************
	RX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	TX_TRIS=1;				//����ͨ�Ŷ˿�ʹ�ܵı�������
	SPBRGH=0;				//������57600(4MHz)
	//SPBRG=16;				// 16=57600(4MHz),51=19200
	//SPBRG=25;				// baud = clk / (4 * (N + 1) )
	SPBRG=34;
	BAUDCON=0X08;			       //16λ�����ʷ���������ֹ�����ʼ�⣬δ����RX����
	TXSTA=0X24;				//8λ���ͣ�ʹ�ܷ��ͣ������첽ģʽ
	RCSTA=0X10;				//�ݽ�ֹ���ڣ�8λ���գ�ʹ�ܽ���

	//OSCCON=0X72;
	//Delay_mS(60);		//20101209�¼�
	//Ini_port();				//��ʼ���˿ڣ���AVDD	
	//resetAD8343();
	R2550(Well1_addr);//�ն�������20101209�¼�
	RCSTAbits.SPEN=1;	//ʹ�ܴ���ͨ��
	//uart debug *****************
	
			
	for(temp=1;(temp<=64)&&(time.tab.interval.l>0)&&(time.tab.looptime.l>0);temp++)
	{	
		SHANG=(time.tab.interval.l-1)>>4;		//��
		YU=(time.tab.interval.l-1)&0XF;		//����	
		T1H=256-(YU<<4);
		Sample_flag=0;
		if(time.tab.looptime.l>0)	//�ѶԳ����͵��ж�תΪ���ַ��͵��жϣ�ʹ��ʱ����׼ȷ
			go_on=1;
		else
			go_on=0;
		if((time.tab.interval.l>1)&&(time.tab.interval.l!=1111))
			Close_avdd();
						
		do
		{				
			if((time.tab.interval.l>16)&&(time.tab.interval.l!=1111))				//���������>16S
			{				
				SHANG_temp=SHANG;
				while(SHANG_temp>0)
				{
					TMR1L=6;		//��ʱ������ֵ��16s
					TMR1H=0;	
					T1CONbits.TMR1ON=1;//����TMR1
					SHANG_temp--;
					Sleep();
				}
				if(YU>0)
				{				
					TMR1L=6;      
					TMR1H=T1H;
					T1CONbits.TMR1ON=1;//����TMR1					
					Sleep();
				} 
					
				TMR1L=6;			//20101029�¸ģ���ʱ������ֵ
				TMR1H=0XF0;
				T1_flag=0;				
				T1CONbits.TMR1ON=1;	//����TMR1
				Open_avdd();
				AD_value.l=3;
				//RX=1;         //20130618
				Sample_save();	//�����洢һ��
				//RX=0;         //20130618				
				time.tab.looptime.l--;//ѭ��������1	
				Close_avdd();
				if(time.tab.looptime.l>0)
					go_on=1;
				else
					go_on=0;
				while(T1_flag==0);			 				
			}
			else if(time.tab.interval.l>1&&time.tab.interval.l<=16)			//ʱ������2~16S
			{
				TMR1L=6;
				TMR1H=T1H;
				
				T1CONbits.TMR1ON=1;//����TMR1
				Sleep();
				
				TMR1L=6;			//20101029�¸ģ���ʱ������ֵ
				TMR1H=0XF0;
				T1_flag=0;				
				T1CONbits.TMR1ON=1;	//����TMR1
				Open_avdd();
				AD_value.l=2;
				//RX=1;         //20130618
				
				//uart debug *****************
				Tx(0x50);
				
				Sample_save();	//�����洢һ��
				
				//uart debug *****************
				Tx(para.info.te_mode);
				
				//RX=0;         //20130618				
				time.tab.looptime.l--;//ѭ��������1	
				Close_avdd();				
				if(time.tab.looptime.l>0)
					go_on=1;
				else
					go_on=0;
				
				while(T1_flag==0);
			}		
			else if(time.tab.interval.l==1)			//ʱ������1S
			{
				TMR1L=6;			//20101029�¸ģ���ʱ������ֵ
				TMR1H=0XF0;
				T1_flag=0;				
				T1CONbits.TMR1ON=1;	//����TMR1
				AD_value.l=1;
				//RX=1;         //20130618
				Sample_save();	//0.5S�����洢һ��
				//RX=0;         //20130618				
				time.tab.looptime.l--;//ѭ��������1					
				if(time.tab.looptime.l>0)
					go_on=1;
				else
					go_on=0;
				while(T1_flag==0);
			}
			else if(time.tab.interval.l==1111)				//0.5S
			{					
				TMR1L=6;			
				TMR1H=0XF8;
				T1_flag=0;				
				T1CONbits.TMR1ON=1;	     //����TMR1
				AD_value.l=0;
				//RX=1;                  //20130618
				Sample_save();	         //0.5S�����洢һ��
				//RX=0;                  //20130618				
				time.tab.looptime.l--;   //ѭ��������1					
				if(time.tab.looptime.l>0)
					go_on=1;
				else
					go_on=0;
				while(T1_flag==0);	
			}		
			
		}while(go_on);				//��ѭ������>0�������ѭ�������������һ��ʱ���
		Open_avdd();
		Delay_mS(25); 
		pnormal.l=temp;
		pnormal.l<<=3;
		Read_DF321page(Timetab_block,pnormal.l,8,time.ch8);//������һ��ʱ���
	}
	Sleep_forever();
}