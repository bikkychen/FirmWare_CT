#include "DSP28_Device.h"
#include "fft.h"
#include "fir.h"
#include "Flash281x_API_Library.h"
#include <math.h>

//V5.0����Ҫ��
//���ĸ����߶���
//����ѹ���¶Ȳ������ߵı䶯����������������
//���Ӷ�д���������ڷ�ֵ�Ĺ���
//����һ��У׼��������ֵ�Ĺ���
//�Ż��������ײ������㷨
//��ѭ���в�ʱ��ʼ��ѹ���¶Ȳɼ���ADC������оƬ����


// FLASHB���������ײ���ֵ 
// FLASHI��ѹ���¶ȱ궨ϵ��
// FLASHJ��ѹ���¶ȱ궨����

//20151216:ɾ��һ��У׼��������ֵ�Ĺ���,ɾ���ϵ���ȡ��ֵ�Ĺ���

//20160728:�����ϵ�ȴ�ʱ�䣬�����������ֵʱ��ֵ���ȵ����⣬�Ż�һ��У׼�ײ���ֵ���㷨

//20161102   
//1����������ֵ��������ֵ��������
//2���ϵ�ʱ����һ�λ�������ֵ����ֵ��������
//3��ÿ16�����������������ֵ�����жϣ�����ֵ����1000mV��˳��ʱ������500nsʱ������һ����ֵ��������������
//4���ϵ����з�ֵ����ֵ�������ٿ����ж�

#define NNH     {asm(" RPT #21 || NOP");}
#define NNL     {asm(" RPT #21 || NOP");}

//LM2941���Ե�Դ�ضϿ��ƺ궨��
//#define N2_EN     GpioDataRegs.GPBDAT.bit.GPIOB1	 

#define PI  3.1415926535897

#define MAXP 2950
#define MINP 2450

#define N 256
#pragma DATA_SECTION(ipcb, "FFTipcb");
#pragma DATA_SECTION(mag, "FFTmag");
RFFT32  fft=RFFT32_256P_DEFAULTS;   
long ipcb[N+2];
long mag[N/2+1];  
const long win[N/2]=HAMMING256; 

long ADSData,tdv,pdv,pdo,addpt;

int16 tdd[20],pdd[20],td,pd,wd;

volatile Uint16 ADBuffer[2];
Uint16 *Flash_Ptr;
Uint32 Length;
FLASH_ST ProgStatus;
FLASH_ST EraseStatus;
Uint16 Status;
Uint16 tp[8];//�������ײ��ж���ֵ + ���������ڱ궨��ֵ

//DSP�У�unsigned charʵ��Ϊ16λ����2�ֽ�
asm (" .def _ADC");
unsigned char ADC[ADC_LEN];//ԭʼ��������

unsigned char Maxtemp[5],Maxcount[5],Maxcomper,Firstplace,Maxplace,Minplace;//����������ʱ����
unsigned long add,avr1024,lx,ln;	//��׼��ѹ����MAX��MINʱ�õ�avr1024
float  angle;		//��ʼ��λ
unsigned long  Delv;//��ֵ��
Uint16  Delv1,Delv2,Delv3,Delv4;
unsigned char Maxcount1[4],Maxcount2[4],Maxcount3[4],Maxcount4[4];
unsigned char tp1[4],tp2[4],tp3[4],tp4[4];
unsigned long Max,Max1,Max2,Min,Min1,Min2;//��ֵ���/Сֵ  
long Maxx;			//��������/��֮��Ĳ�ֵ
char Fpar;			//����������Ĳ��ҷ���
float  t;			//���δ����ľ���ʱ��
unsigned char start,CalBegin;//���ղ��ε���ʼλ��,ʵ�ʼ������ʼλ��

Uint16 i,k,j,m,n;//ѭ���õĹ�������
unsigned char tf;//��ʱѭ���ñ���
unsigned char F;

char SciCom;	//RX����ֵ

char FlowBegin;//����������ʼ��־
char StopFlag;//��������ֹͣ��־
char FlowReq;//��������ֵ�����־
char HaltFlag;//���������IDLEģʽ��־

char AbsT[1200];
float DT[4][100];
char RDD[4];
char RDDa[4];
float LDT[3];
char DTC[4];
unsigned char dd[4];

unsigned char AT[5];

float32 ApiVersion;

typedef union RR
{float f;
 char c[2];
 }b;

union RR RD[4];
union RR RDa[4];
float t1,t2,t3,t4;//��,�������Ƶ�˳,��������ʱ�� 
char DataReqFlag,DataReqCount,RanReqFlag;//�������Σ���������ֵ
char ResetReqFlag;
char VerFlag,CalFlag;

#if RELEASE
typedef struct ST
{float v;
 unsigned char c;
 float         f;
 }a;

struct ST std[100];
#endif

float DT_Max,DT_Min;
char q1,q2,q3;
float df[4][10];
char SC[4];
unsigned char start_ss,start_sn,start_xs,start_xn;

void Sample(void);//�����ڰ���AbsTime()��FIR()�����������������ʱ�䲢����FIR�˲�
unsigned char AbsTime(unsigned char ch);//�������������ľ���ʱ��
void InitAdc(void);
void SendFlow(void);//�����˲���ľ���ʱ��
void InitFlash(void);
void UARTPutByte( char b);
void DataCheck(char c);

void DelaymS(unsigned char);

void ADS1222_INIT(void);
void ADS1222_sample(unsigned char Ch);
 
void ISRSciCom05(void);
void ISRSciCom06(void);
void ISRSciCom0A(void);
void ISRSciCom0B(void);
unsigned char CalThr(void);
void CalP(void);//������������ֵ

#if RELEASE
extern unsigned char secureRamFuncs_loadstart;
extern unsigned char secureRamFuncs_loadend;
extern unsigned char secureRamFuncs_runstart;
extern unsigned char AbsTimeFuncs_loadstart;
extern unsigned char AbsTimeFuncs_loadend;
extern unsigned char AbsTimeFuncs_runstart;
#endif

void main(void)
{  
	unsigned char i;

	 HaltFlag=0;

	DINT;//DINT   asm(" setc INTM")

	DelaymS(7); 
	//DSPʱ�ӱ�Ƶ
	InitSysCtrl(); 
    DelaymS(7); 


	//�ϵ�Լ1����ٸ��˷ź�ģ�⿪���ϵ�
    InitGpio();
	InitEv();
    
	
	InitPieCtrl(); 
	InitPieVectTable();
	IER = 0x0000;
	IFR = 0x0000;
    XintfRegs.XINTCNF2.bit.CLKOFF = 1;   // DISABLE XCLKOUT
    
	InitEv();

    

#if RELEASE
	memcpy(&secureRamFuncs_runstart,
	&secureRamFuncs_loadstart,
	&secureRamFuncs_loadend - &secureRamFuncs_loadstart);
	InitFlash();
	memcpy(&AbsTimeFuncs_runstart,
	&AbsTimeFuncs_loadstart,
	&AbsTimeFuncs_loadend - &AbsTimeFuncs_loadstart); 
	memcpy(&Flash28_API_RunStart,
	&Flash28_API_LoadStart,
	&Flash28_API_LoadEnd - &Flash28_API_LoadStart); 
#endif	

   ApiVersion=Flash_APIVersion();
   Flash_CPUScaleFactor = SCALE_FACTOR;
   Flash_CallbackPtr = NULL;

   tp[0]=*(Uint16 *)(0x3F4000);//FLASHB      : origin = 0x3F4000, length = 0x002000
   tp[1]=*(Uint16 *)(0x3F4001);//FLASHB      : origin = 0x3F4000, length = 0x002000
   tp[2]=*(Uint16 *)(0x3F4002);//FLASHB      : origin = 0x3F4000, length = 0x002000
   tp[3]=*(Uint16 *)(0x3F4003);//FLASHB      : origin = 0x3F4000, length = 0x002000
/*
   tp[4]=*(Uint16 *)(0x3F4004);
   tp[5]=*(Uint16 *)(0x3F4005);
   tp[6]=*(Uint16 *)(0x3F4006);
   tp[7]=*(Uint16 *)(0x3F4007);
   */

   Flash_Ptr = (Uint16 *)0x003D8000;//FLASHJ=0x3D8000~0x3D9FFF, �� 0x2000 Words = 4096 ��
   for(i=0;i<0x1000;i++)
     {
       if((*Flash_Ptr)==(0xffff)) 
	       break;
	   else
	     Flash_Ptr+=2;
	 }


	InitSci();  	   
    DINT;
	/*�����жϷ��������ڵ�ַ*/
	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.RXBINT = &SCIRXINTB_ISR;//�����ж�
	PieCtrl.PIEIER9.bit.INTx3 = 1;
	EDIS;   // This is needed to disable write to EALLOW protected registers



    DelaymS(300);//�ڴ�����ʱ1s,�ܼ��ϵ�2���12V��Դ�������ذ巢�͹��������ʱ��

    if(HaltFlag)
        EN=1;   //��12V��Դ
	else
	    EN=0;   //��12V��Դ

	InitAdc();

    //FFTģ���ʼ��
 	fft.ipcbptr=ipcb;
	fft.magptr=mag;  
	fft.winptr=(long *)win;
	fft.init(&fft); 

    
	 FlowBegin = 0;
	 StopFlag=0;
	 FlowReq=0;
	 DataReqFlag=0;
	 RanReqFlag=0;
	 ResetReqFlag=0;
	 VerFlag=0;

    Fpar=0x44;
	RDD[0]=0;RDD[1]=0;RDD[2]=0;RDD[3]=0;
    RD[0].f=0.0;RD[1].f=0.0;RD[2].f=0.0;RD[3].f=0.0;
    SC[0]=0;SC[1]=0;SC[2]=0;SC[3]=0;
	DTC[0]=0;DTC[1]=0;DTC[2]=0;DTC[3]=0;
	for(i=0;i<10;i++)
	  {df[0][i]=0;df[1][i]=0;df[2][i]=0;df[3][i]=0;}	

	RDa[0].f=0.0;	RDa[1].f=0.0;	RDa[2].f=0.0;	RDa[3].f=0.0;

    start_ss=0;
    start_sn=0;
    start_xs=0;
    start_xn=0;

    ADS1222_INIT();
	ADS1222_sample(1);pd=ADSData;	
	ADS1222_sample(0);td=ADSData;
	
	EN=0;//��12V��Դ
	
	DelaymS(100);

	Delv1=(MAXP+MINP)/2;
	Delv2=Delv1;
	Delv3=Delv1;
	Delv4=Delv1;


	 CalP();//������������ֵ
	 if( ((Delv1>1365)&&(Delv2>1365)&&(abs(RDa[0].f-RDa[1].f)>500)) || ((Delv3>1365)&&(Delv4>1365)&&(abs(RDa[2].f-RDa[3].f)>500)) )
		{
		  CalThr();
		}
	Status = Flash_Erase(SECTORB,&EraseStatus); //������ʾ�������Ժ�ʱԼ1.3s,datasheet��˵4Ƭ8K��FLASH������ʱ��10s����ÿƬ��ʱ2.5s			   
	Status = Flash_Program((Uint16 *)(0x3F4000),tp,4,&ProgStatus);	

		/*���ж�*/
	IER |= M_INT9;
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

	while(1) 
    { 

  	 if(HaltFlag)//���������幦��
   	 {	  
	      EN=1;//��12V��Դ		  DelaymS(10);
		  EALLOW;
		  DINT;//���ж�
		  AdcRegs.ADCTRL1.bit.RESET=1;//ADCģ�鸴λ
		  SysCtrlRegs.PCLKCR.bit.SCIENCLKB=0;
		  SysCtrlRegs.PCLKCR.bit.ADCENCLK=0;
		  EDIS;		 
		   	  
	      while(1)
		  {	 	  		  
	         pdv=0;
			 tdv=0;
	         for(i=0;i<25;i++)//13*2*180=4.28s
			 {	         
				 ADS1222_sample(1);
				 pdv+=ADSData;		 
			     ADS1222_sample(0);
				 tdv+=ADSData;			 	 	 
			 }
			 pdv/=25;
			 tdv/=25;
	         
			 if( ((pdv-pdo)<5) && ((pdv-pdo)>-5) )
			 {
			     ADBuffer[0]=pdv;
		         ADBuffer[1]=tdv;
				 if(((Uint32)Flash_Ptr)<((Uint32)0x3DA000))
				 {
			     	 Status = Flash_Program(Flash_Ptr,ADBuffer,2,&ProgStatus);//дһ��word��ʱ35us
			    	 Flash_Ptr+=2;
				 }
			 }
			  pdo=pdv;
		 }
	   }
  

     if((DataReqFlag) && (ResetReqFlag==0))//��ȡ����
	 {
	  FlowBegin=0;
	  StopFlag=0x05;//�����
	  if( (DataReqFlag>0xf0) && (DataReqFlag<0xf5) )
	   {
	    DataReqCount=0;
		 if(AbsTime(DataReqFlag&0x0f)!=5)
		     AbsTime(DataReqFlag&0x0f);
	   }

	   if(DataReqFlag==0xff)
	    {
		 UARTPutByte(ADC[DataReqCount]>>8);
		 UARTPutByte(ADC[DataReqCount]);
		 DataReqCount++;
		 if(DataReqCount>=1801)
           {FlowBegin=0;
	        StopFlag=0;//�����
	       }
		}

		if(DataReqFlag==0xfe)
	    {
		 UARTPutByte(ADC[DataReqCount+500]>>8);
		 UARTPutByte(ADC[DataReqCount+500]);
		 DataReqCount++;
		 if(DataReqCount>=901)
           {FlowBegin=0;
	        StopFlag=0;//�����
	       }
		}

	   DataReqFlag=0;
	 }	
	

	   if( (FlowBegin==1) && (StopFlag<10) )//������ѹ�����¶Ȳ���
	   {//Լ450msѭ��һ��
	   
	  	 ADS1222_sample(2);
		 wd=ADSData;

         ADS1222_sample(1);//70ms
		 pd=ADSData;

         ADS1222_sample(0);//70ms
		 td=ADSData;
	   
	    DTC[0]=0;DTC[1]=0;DTC[2]=0;DTC[3]=0;
	    start_ss=0;
        start_sn=0;
        start_xs=0;
        start_xn=0;	

		for(i=0;i<16;i++)
		  {	
		    if(AbsTime(1)==5)//3.5ms
		     {DT[0][DTC[0]]=t;DTC[0]++;}

			if(AbsTime(2)==5)//3.5ms
		     {DT[1][DTC[1]]=t;DTC[1]++;}			 		     
 	        
		    if(AbsTime(3)==5) //3.5ms
		     {DT[2][DTC[2]]=t;DTC[2]++;}
			
			if(AbsTime(4)==5)//3.5ms
		     {DT[3][DTC[3]]=t;DTC[3]++;}		     			 	 

			 CalP();//������������ֵ

			 if(FlowBegin!=1) 
			    i=100;
		  }

			DataCheck(2);//1ms
			DataCheck(3);//1ms
            if((RD[0].f)>=(RD[1].f))
			 {
			   while( ((RD[0].f)-(RD[1].f))>=1000)
			      RD[0].f-=1000;
			 }
            if((RD[1].f)>=(RD[0].f))
			 {
			   while( ((RD[1].f)-(RD[0].f))>=1000)
			      RD[1].f-=1000;
			 }
            DINT;
			RDa[0].f=RD[0].f;
			RDa[1].f=RD[1].f;
			EINT;

			DataCheck(0);//1ms
			DataCheck(1);//1ms	
            if((RD[2].f)>=(RD[3].f))
			 {
			   while( ((RD[2].f)-(RD[3].f))>=1000)
			      RD[2].f-=1000;
			 }
            if((RD[3].f)>=(RD[2].f))
			 {
			   while( ((RD[3].f)-(RD[2].f))>=1000)
			      RD[3].f-=1000;
			 }
            DINT;
			RDa[2].f=RD[2].f;
			RDa[3].f=RD[3].f;
			EINT;

		    StopFlag++;		
		    
		    if( ((Delv1>1365)&&(Delv2>1365)&&(abs(RDa[0].f-RDa[1].f)>500)) || ((Delv3>1365)&&(Delv4>1365)&&(abs(RDa[2].f-RDa[3].f)>500)) )
			{
			  CalThr();
			}
	
       }

	  	   
	  if( (FlowBegin==0) && (StopFlag==0) )//ѹ�����¶Ȳ���
	   {//Լ230msѭ��һ��
        AbsTime(1);
		ADS1222_sample(2);//70
		wd=ADSData;
        AbsTime(2);

        ADS1222_sample(1);//70
		pd=ADSData;
        AbsTime(3);
        ADS1222_sample(0);//70
		td=ADSData;	
        AbsTime(4);

		CalP();//������������ֵ
	   }
	  
	   
	   
	  

	 if(StopFlag>9)
	   {
		    FlowBegin=0;
		    StopFlag=0;

	        RDD[0]=0;RDD[1]=0;RDD[2]=0;RDD[3]=0;
			RD[0].f=0.0;RD[1].f=0.0;RD[2].f=0.0;RD[3].f=0.0;
			SC[0]=0;SC[1]=0;SC[2]=0;SC[3]=0;
			DTC[0]=0;DTC[1]=0;DTC[2]=0;DTC[3]=0;
			for(i=0;i<10;i++)
			   {df[0][i]=0;df[1][i]=0;df[2][i]=0;df[3][i]=0;}	

			for(i=0;i<100;i++)
		    {std[i].v=0;
			 std[i].c=0;
			 std[i].f=0;}
	   }


   }  	
} 	


void DataCheck(char c)
{
  
char i,k,j;
 float f,g;

#if RELEASE 
  if(DTC[c]==0)
   {RDD[c]=0;
	RD[c].f=0.0;
	SC[c]=0;
	for(i=0;i<10;i++)
	  {df[c][i]=0;}	
	return;
   }

	for(i=0;i<100;i++)
	  std[i].c=0;
	k=1;//Ĭ�ϵ�һ�������Ǳ�׼
    std[0].v=DT[c][0];	
	std[0].f=DT[c][0];
    std[0].c=1;
    for(i=1;i<DTC[c];i++)
	 {
	  for(j=0;j<k;)//�ֱ������׼�Ա�
	   { if( ((DT[c][i]-std[j].v)<200.0) && ((std[j].v-DT[c][i])<200.0) )
	      { std[j].c++;
		    std[j].f+=DT[c][i];
			break;
		  }
          j++;
		  if(j==k)//��֮ǰ���κα�׼��������,����������Լ�����һ����׼
		  {
		    std[k].v=DT[c][i];
			std[k].f=DT[c][i];
			std[k].c=1;
			k++;j++;
		  }
	   }
	 }

	 for(i=1;i<DTC[c];i++)//�ҳ��������ݵ����ı�׼
	 {if(std[i-1].c>std[i].c)
	   {std[i].c=std[i-1].c;
	    std[i].f=std[i-1].f;
	    }
	 }

	 f=std[DTC[c]-1].f/std[DTC[c]-1].c;

#else
        DT_Max=DT[c][0];
		for(i=1;i<DTC[c];i++)
			{if(DT_Max<DT[c][i])
			   DT_Max=DT[c][i];
			}
		DT_Min=DT[c][0];
		for(i=1;i<DTC[c];i++)
			{if(DT_Min>DT[c][i])
			   DT_Min=DT[c][i];
			}
		LDT[0]=0.0;
		LDT[1]=0.0;
		LDT[2]=0.0;
		q1=0;q2=0;q3=0;
		for(i=0;i<DTC[c];i++)
		{
			if((DT_Max-DT[c][i])<200)
			{LDT[0]+=DT[c][i];
			q1++;
			}
			else if((DT[c][i]-DT_Min)<200)
			{LDT[1]+=DT[c][i];
			q2++;
			}
			else
			{LDT[2]+=DT[c][i];
			q3++;
			}
		}
		if((q1>=q2)&&(q1>=q3))
	 	  {f=q1;
	 	  f=LDT[0]/(f);
		  }
		if((q2>=q1)&&(q2>=q3))
	 	  {f=q2;
	 	   f=LDT[1]/(f);
           }
		if((q3>=q2)&&(q3>=q1))
	 	  {f=q3;
	 	  f=LDT[2]/(f);
		  }
#endif

          df[c][9]=df[c][8];
		  df[c][8]=df[c][7];
		  df[c][7]=df[c][6];
		  df[c][6]=df[c][5];
		  df[c][5]=df[c][4];
		  df[c][4]=df[c][3];
		  df[c][3]=df[c][2];
		  df[c][2]=df[c][1];
		  df[c][1]=df[c][0];
		  df[c][0]=f;
		  if(SC[c]<10)
		      SC[c]++;
		  f=0.0;
		  for(i=0;i<SC[c];i++)
		    {f+=df[c][i];}
		  g=SC[c];
		  f=f/g;		 
          RD[c].f=f;
		  f-=100000.0;
		  f*=4.0;
		  RDD[c]=f;
		  
}

void CalP()//��ֵ�Զ�������2000mV����
{
	//	if((ch==1)||(ch==2))
	{
		if( (Delv1<=MINP) && (Delv2<=MINP) )//˳���涼С��1800mV������Ҫ����Ŵ���
		{
			ISRSciCom0A();
		}
		else if( (Delv1>=MAXP) && (Delv2>=MAXP) )//˳���涼����2200mV������Ҫ��С�Ŵ���
		{
			ISRSciCom05();
		}
		else if( (Delv1>MAXP) && (Delv2<MINP) )
		{
			if(((Delv1+Delv2)/2)> MAXP )
			{
				ISRSciCom05();
			}
			else if(((Delv1+Delv2)/2)<MINP)
			{
				if(Delv1<3800)
					ISRSciCom0A();
			}
		}
		else if( (Delv1<MINP) && (Delv2>MAXP) )
		{
			if(((Delv1+Delv2)/2)>MAXP)
			{
				ISRSciCom05();
			}
			else if(((Delv1+Delv2)/2)<MINP )
			{
				if(Delv2<3800)
					ISRSciCom0A();
			}
		}
		else if( (Delv1>=MINP)&&(Delv1<=MAXP)&&(Delv2<=MINP) )
		{
			if(((Delv1+Delv2)/2)<MINP)
			{
				ISRSciCom0A();
			}
		}
		else if( (Delv2>=MINP)&&(Delv2<=MAXP)&&(Delv1<=MINP) )
		{
			if(((Delv1+Delv2)/2)<MINP )
			{
				ISRSciCom0A();
			}
		}
		else if( (Delv1>=MINP)&&(Delv1<=MAXP)&&(Delv2>=MAXP) )
		{
			if(((Delv1+Delv2)/2)>MAXP )
			{
				ISRSciCom05();
			}
		}
		else if( (Delv2>=MINP)&&(Delv2<=MAXP)&&(Delv1>=MAXP) )
		{
			if(((Delv1+Delv2)/2)>MAXP )
			{
				ISRSciCom05();
			}
		}	 
	}

//	if((ch==3)||(ch==4))
	{
		if( (Delv3<=MINP) && (Delv4<=MINP) )//˳���涼С��1800mV������Ҫ����Ŵ���
		{
			ISRSciCom0B();
		}
		else if( (Delv3>=MAXP) && (Delv4>=MAXP) )//˳���涼����2200mV������Ҫ��С�Ŵ���
		{
			ISRSciCom06();
		}
		else if( (Delv3>MAXP) && (Delv4<MINP) )
		{
			if(((Delv3+Delv4)/2)> MAXP )
			{
				ISRSciCom06();
			}
			else if(((Delv3+Delv4)/2)<MINP)
			{
				if(Delv3<3800)
					ISRSciCom0B();
			}
		}
		else if( (Delv3<MINP) && (Delv4>MAXP) )
		{
			if(((Delv3+Delv4)/2)>MAXP)
			{
				ISRSciCom06();
			}
			else if(((Delv3+Delv4)/2)<MINP )
			{
				if(Delv4<3800)
					ISRSciCom0B();
			}
		}
		else if( (Delv3>=MINP)&&(Delv3<=MAXP)&&(Delv4<=MINP) )
		{
			if(((Delv3+Delv4)/2)<MINP)
			{
				ISRSciCom0B();
			}
		}
		else if( (Delv4>=MINP)&&(Delv4<=MAXP)&&(Delv3<=MINP) )
		{
			if(((Delv3+Delv4)/2)<MINP )
			{
				ISRSciCom0B();
			}
		}
		else if( (Delv3>=MINP)&&(Delv3<=MAXP)&&(Delv4>=MAXP) )
		{
			if(((Delv3+Delv4)/2)>MAXP )
			{
				ISRSciCom06();
			}
		}
		else if( (Delv4>=MINP)&&(Delv4<=MAXP)&&(Delv3>=MAXP) )
		{
			if(((Delv3+Delv4)/2)>MAXP )
			{
				ISRSciCom06();
			}
		}	 
	}
}

//��������ʱ��,ͨ����(ch)��ʾ������,1-��������˳��,
//2-������������,3-��������˳��,4-������������
#pragma CODE_SECTION(AbsTime, "AbsTimeFuncs")
unsigned char AbsTime(unsigned char ch)
{    	



    Delv=0;

    EALLOW;
	SysCtrlRegs.PCLKCR.bit.ADCENCLK=1;
	EDIS; 
     if(ch==1||ch==2)
	 {

    AdcRegs.ADCTRL3.bit.ADCPWDN=1;//ADC������·�ϵ� 
	AdcRegs.ADCTRL3.bit.ADCCLKPS=1; 
	AdcRegs.ADCTRL1.bit.CPS=0;//������ʱ��HSPCLK����Ƶ
	AdcRegs.ADCTRL3.bit.SMODE_SEL=0;//˳�����ģʽ	
	AdcRegs.MAX_CONV.bit.MAX_CONV=0;//���ת��ͨ����
	AdcRegs.CHSELSEQ1.bit.CONV00=0;//ADC����ͨ��ѡ������		
	AdcRegs.ADC_ST_FLAG.bit.INT_SEQ1_CLR=1;//���SEQ1�жϱ�ʾ
	AdcRegs.ADC_ST_FLAG.bit.INT_SEQ2_CLR=1;	//���SEQ2�жϱ�ʾ	
	AdcRegs.ADCTRL2.bit.EVB_SOC_SEQ=0;//������������ʹ��EVB
	AdcRegs.ADCTRL2.bit.RST_SEQ1=1;//����������λ��CONV00
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=0;//ʹ��SEQ1���ж�����
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;//ÿ��SEQ1���н���ʱ�����ж�
	AdcRegs.ADCTRL2.bit.EVA_SOC_SEQ1=0;//EVA���ܴ���SEQ1
	AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1=0;//�ⲿ�źŲ��ܴ���SEQ1 
    }
	 if(ch==3||ch==4)
	{
	 

    AdcRegs.ADCTRL3.bit.ADCPWDN=1;//ADC������·�ϵ� 
	AdcRegs.ADCTRL3.bit.ADCCLKPS=1; 
	AdcRegs.ADCTRL1.bit.CPS=0;//������ʱ��HSPCLK����Ƶ
	AdcRegs.ADCTRL3.bit.SMODE_SEL=0;//˳�����ģʽ	
	AdcRegs.MAX_CONV.bit.MAX_CONV=0;//���ת��ͨ����
	AdcRegs.CHSELSEQ1.bit.CONV00=1;//ADC���ͨ��ѡ�����?	
	AdcRegs.ADC_ST_FLAG.bit.INT_SEQ1_CLR=1;//���SEQ1�жϱ�ʾ
	AdcRegs.ADC_ST_FLAG.bit.INT_SEQ2_CLR=1;	//���SEQ2�жϱ�ʾ	
	AdcRegs.ADCTRL2.bit.EVB_SOC_SEQ=0;//������������ʹ��EVB
	AdcRegs.ADCTRL2.bit.RST_SEQ1=1;//����������λ��CONV00
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=0;//ʹ��SEQ1���ж�����
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;//ÿ��SEQ1���н���ʱ�����ж�
	AdcRegs.ADCTRL2.bit.EVA_SOC_SEQ1=0;//EVA���ܴ���SEQ1
	AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1=0;//�ⲿ�źŲ��ܴ���SEQ1 
      
	}DINT;
	//����ָ��Ϊ������,����һ��ָ�Ҫ�ټ�������,��ΪҪ��Ŀ��Ĵ�����ֵ�Ƶ�DP��ȥ.
    //����64������,�ȷ�������, ����1us,���ӳ�64us

   // EALLOW;

	switch(ch)
	{
	 case 1:
	    IN1=0;EN1=0;EN2=1;EN3=1;EN4=1;
	 break;
	 case 2:
	    IN1=1;EN1=0;EN2=1;EN3=1;EN4=1;
	 break;
	 case 3:
	    IN2=0;EN1=1;EN2=1;EN3=0;EN4=1;
	 break;
	 case 4:
	    IN2=1;EN1=1;EN2=1;EN3=0;EN4=1;
	 break;
	 default:
	    return 0;
	}

    S1=0;S2=0; S1=0;S2=0; S1=0;S2=0; S1=0;S2=0; S1=0;S2=0; S1=0;S2=0; S1=0;S2=0;

//��������22����
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
    S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL
	S1=0;S2=1;NNH S2=0;S1=1;NNL

 
//�����൱����ʱ

	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	S1=0;S2=0;NNH S2=0;S1=0;NNL
	
	S2=0;S1=0;



	switch(ch)
	{
	 case 1:
		IN1=0;
		EN1=1;
		EN2=0;
		EN3=1;
		EN4=1;
	 break;
	 case 2:
		IN1=1;
		EN1=1;
		EN2=0;
		EN3=1;
		EN4=1;
	 break;
	 case 3:
		IN2=0;
		EN1=1;
	    EN2=1;
		EN3=1;
		EN4=0;
	 break;
	 case 4:
		IN2=1;
		EN1=1;
	    EN2=1;
		EN3=1;
		EN4=0;
	 break;
	 default:
	    return 0;
	}

   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us
   asm(" RPT #126 || NOP");//128��ָ������=2us

   AdcRegs.ADCTRL2.bit.SOC_SEQ1=1;//ADC��ʼת��

   asm(" RPT #126 || NOP");//128��ָ������=2us

   asm(" MOVL XAR2, #7108H");  //��ָ������
   asm(" MOVL XAR3, #_ADC");   //��ָ������

			 
		//����ʱ50000+2734.375+15.625*21+26000=79062.75ns	 	
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
#if  RELEASE
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
#endif
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����	   
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
    asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4��?
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����
	asm(" MOV ACC,*XAR2");	  	   asm(" MOV *XAR3++,ACC");//4����


	EN1=1;EN2=1;EN3=1;EN4=1;//����ģ�⿪�ز�ʹ��	
    EINT;

   
    
	AdcRegs.ADCTRL1.bit.RESET=1;//ADCģ�鸴λ
	NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
	AdcRegs.ADCTRL1.bit.SUSMOD=0;
	AdcRegs.ADCTRL1.bit.ACQ_PS=0;//�������ڴ�С = 1 ADCLK
	AdcRegs.ADCTRL1.bit.CONT_RUN=1;//������
	AdcRegs.ADCTRL1.bit.SEQ_CASC=0;//��������ģʽ
    AdcRegs.ADCTRL3.bit.ADCCLKPS=3; 
  	AdcRegs.ADCTRL3.bit.ADCBGRFDN=3;//��϶�Ͳο���·�ϵ�

   // EDIS;

    EALLOW;
	SysCtrlRegs.PCLKCR.bit.ADCENCLK=0;
	EDIS;


	for(k=0;k<ADC_LEN;k++)
	{
		 ADC[k]=(ADC[k])>>4;
	}
	     

	 //5��ƽ���˲�
	  AT[0]=ADC[0];
	  AT[1]=ADC[1];
	  AT[2]=ADC[2];
	  AT[3]=ADC[3];
	  AT[4]=ADC[4];
     for(k=2;k<(ADC_LEN-3);k++)
	 {
	  ADC[k]=AT[0]+AT[1]+AT[2]+AT[3]+AT[4];
      ADC[k]/=5;
	  AT[0]=AT[1];
	  AT[1]=AT[2];
	  AT[2]=AT[3];
	  AT[3]=AT[4];
	  AT[4]=ADC[k+3];
	 }

  //���ֵ
	avr1024=0;
    for(k=50;k<1074;k++)
	  avr1024+=ADC[k];
    avr1024/=1024;


    //�����ֵ���ҳ������Сֵ
	i=0;
	Max=0;
	j=0;

 	Max=ADC[50];
	for(k=50;k<1750;k++)
		{
		 if(ADC[k]>Max)
			{ Max=ADC[k];
			  Maxplace=k;
			}
		}

	Min=ADC[50];
	for(k=50;k<1750;k++)
		{

		 if(ADC[k]<Min)
			{ Min=ADC[k];
			 Minplace=k;
			}
		}

     Delv=Max-Min;

	 switch (ch)
	 {
	   case 1:
	   Delv1=Delv;
	  break;
	   case 2:
	   Delv2=Delv;
	  break;
	   case 3:
	   Delv3=Delv;
	  break;
	   case 4:
	   Delv4=Delv;
	  break;
	 }
   //  if(RanReqFlag==1)//��ֵ����
	//    return 1;


    if(Delv<682)//500mV,��ֵ̫С,�����ٽ��м���,����
	   return 2;




  j = Delv / 50;//20151222�ģ���ǰ��j = Delv / 100
  i = 0;
  k = 50;
  Maxcount[0]=0;

            while (1) 
            {
                if (((ADC[k]) > avr1024))
                    if (((ADC[k] - avr1024) > j))
                        if (((ADC[k]) > (ADC[k - 2])) && ((ADC[k]) > (ADC[k + 2])))
                            if (((ADC[k - 1]) > (ADC[k - 3])) && ((ADC[k + 1]) > (ADC[k + 3])))
                                if (((ADC[k - 2]) > (ADC[k - 4])) && ((ADC[k + 2]) > (ADC[k + 4]))) 		 
                                {
                                    Maxcount[i] = k;
                                    Maxtemp[i] = ADC[k];
                                    i++;
                                    switch (i) 
                                    {
                                        case 2:
                                            if (((Maxcount[1] - Maxcount[0]) < 12) || ((Maxcount[1] - Maxcount[0]) > 20) || (Maxtemp[1] <= Maxtemp[0]))
                                            {
                                                i = 1;
                                                Maxtemp[0] = Maxtemp[1];
                                                Maxcount[0] = Maxcount[1];
                                            }
                                            break;
                                        case 3:
                                            if (((Maxcount[2] - Maxcount[1]) < 12) || ((Maxcount[2] - Maxcount[1]) > 20) || (Maxtemp[2] <= Maxtemp[1]))
                                            {
                                                i = 1;
                                                Maxtemp[0] = Maxtemp[2];
                                                Maxcount[0] = Maxcount[2];
                                            }
                                            break;
                                        case 4:
                                            if (((Maxcount[3] - Maxcount[2]) < 12) || ((Maxcount[3] - Maxcount[2]) > 20) || (Maxtemp[3] <= Maxtemp[2]))
                                            {
                                                i = 1;
                                                Maxtemp[0] = Maxtemp[3];
                                                Maxcount[0] = Maxcount[3];
                                            }
                                            break;
                                        case 5:
                                            if (((Maxcount[4] - Maxcount[3]) < 12) || ((Maxcount[4] - Maxcount[3]) > 20) || (Maxtemp[4] <= Maxtemp[3]))
                                            {
                                                i = 1;
                                                Maxtemp[0] = Maxtemp[4];
                                                Maxcount[0] = Maxcount[4];
                                            }
                                            break;                                         
                                    }
                                    k += 8;
                                }
                if (i == 4)
                    break;
                k++;
                if (k >= 1795)
                    return 3; 
            }

            if (ADC[Maxcount[0] + 1] > ADC[Maxcount[0]])
            {
                Maxcount[0] += 1;
                Maxtemp[0] = ADC[Maxcount[0]];
            }
            if (ADC[Maxcount[1] + 1] > ADC[Maxcount[1]])
            {
                Maxcount[1] += 1;
                Maxtemp[1] = ADC[Maxcount[1]];
            }
            if (ADC[Maxcount[2] + 1] > ADC[Maxcount[2]])
            {
                Maxcount[2] += 1;
                Maxtemp[2] = ADC[Maxcount[2]];
            }
            if (ADC[Maxcount[3] + 1] > ADC[Maxcount[3]])
            {
                Maxcount[3] += 1;
                Maxtemp[3] = ADC[Maxcount[3]];
            }
			if (ADC[Maxcount[4] + 1] > ADC[Maxcount[4]])
            {
                Maxcount[4] += 1;
                Maxtemp[4] = ADC[Maxcount[4]];
            }


			//��¼���βɼ��Ĳ������ݣ��Ա������������м����ײ���ֵ
			if(ch==1)
			{
			  for(k=0;k<4;k++)
			  {
			   Maxcount1[k]=Maxcount[k];
			   tp1[k]=Delv/((Maxtemp[k+1]+Maxtemp[k])/2-avr1024);
			  }
			}
			else if(ch==2)
			{
			  for(k=0;k<4;k++)
			  {
			   Maxcount2[k]=Maxcount[k];
			   tp2[k]=Delv/((Maxtemp[k+1]+Maxtemp[k])/2-avr1024);
			  }
			}
			else if(ch==3)
			{
			  for(k=0;k<4;k++)
			  {
			   Maxcount3[k]=Maxcount[k];
			   tp3[k]=Delv/((Maxtemp[k+1]+Maxtemp[k])/2-avr1024);
			  }
			}
			else if(ch==4)
			{
			  for(k=0;k<4;k++)
			  {
			   Maxcount4[k]=Maxcount[k];
			   tp4[k]=Delv/((Maxtemp[k+1]+Maxtemp[k])/2-avr1024);
			  }
			}

	    Firstplace=Maxcount[0];
		if(tp[ch-1]==0xff)		  
		  j=Delv/16;
		else
          j=Delv/tp[ch-1];

		Firstplace=0;
		for(k=0;k<5;k++)//��Ҫ���ҳ��ײ��Ĳ��壬ʵ���Ͽ����ǵ�4����
		{
		 if((Maxtemp[k]-avr1024)>j)
		  {
		   Firstplace=Maxcount[k];
		   break;
		  }
		}

		if(Firstplace==0)
		  return 4;//û���ҵ��ʺ��������ײ�

	    Firstplace+=16*0; 

		for(k=Firstplace;k<Firstplace+8;k++)
		{ if((ADC[k-1]>avr1024)&&(ADC[k]<=avr1024))
		    {
		     start=k;
		     break;
		     }
		}


        //��ȡ��256�����������FFT����
		for(i=0;i<256;i++)
		 {
	    ipcb[i]=(ADC[i+start]-avr1024);	// ipcb[i]=((ADC[i+start])-avr);
		 ipcb[i]<<=17;
		// ipcb[i]=(long)131072*2*sin(2*PI*i*0.0625+31.001*PI/180);
		 }
	    RFFT32_brev(ipcb,ipcb,256);
	    fft.calc(&fft);
	    fft.split(&fft);
	    fft.mag(&fft); 
	   // frq=SF/512.0*fft.peakfrq;//������������������ʱ��Ƶ���������,����Ϻ������.
		angle=atan2((double)ipcb[fft.peakfrq*2+1],(double)ipcb[fft.peakfrq*2]);//+2.613928;//1.695869;
		//angle=angle*180/PI;
		angle*=57.295779513082320876798154814105;
		angle+=90.0;

		if(angle<0)
		  angle+=360;	
		angle-=180.0;

		//�����������ľ���ʱ��
        t=79062.75+start*62.5-(angle*1000.0/360.0)-4000.0;

		
	    return 5;
	   
	  		 	    								 
}


#if RELEASE
#pragma CODE_SECTION(InitFlash, "secureRamFuncs")
void InitFlash(void)
{
asm(" EALLOW"); // Enable EALLOW protected register access
FlashRegs.FPWR.bit.PWR = 3; // Flash set to active mode
FlashRegs.FSTATUS.bit.V3STAT = 1; // Clear the 3VSTAT bit
FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF; // Sleep to standby cycles
FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF; // Standby to active cycles
FlashRegs.FBANKWAIT.bit.RANDWAIT = 3; // F280x Random access wait states
FlashRegs.FBANKWAIT.bit.PAGEWAIT = 3; // F280x Paged access wait states
FlashRegs.FOTPWAIT.bit.OPTWAIT = 5; // F280x OTP wait states
FlashRegs.FOPT.bit.ENPIPE = 1; // Enable the flash PIpeline
asm(" EDIS"); // Disable EALLOW protected register access
/*** Force a complete PIpeline flush to ensure that the write to the last register
configured occurs before returning. Safest thing is to wait 8 full cycles. ***/
asm(" RPT #60 || NOP");
} 
#endif


void InitAdc(void)
{   

	AdcRegs.ADCTRL1.bit.RESET=1;//ADCģ�鸴λ
	NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
	AdcRegs.ADCTRL1.bit.SUSMOD=0;
	AdcRegs.ADCTRL1.bit.ACQ_PS=0;//�������ڴ�С = 1 ADCLK
	AdcRegs.ADCTRL1.bit.CONT_RUN=1;//������
	AdcRegs.ADCTRL1.bit.SEQ_CASC=0;//��������ģʽ
    AdcRegs.ADCTRL3.bit.ADCCLKPS=3; 
  	AdcRegs.ADCTRL3.bit.ADCBGRFDN=3;//��϶�Ͳο���·ϵ?
    EALLOW;
	SysCtrlRegs.PCLKCR.bit.ADCENCLK=0;
	EDIS;

}	


void DelaymS(unsigned char t)//150Mʱ��׼�ˣ�Ŀǰ��64M����ʱԼΪ3.4t ms
{
	unsigned long l,k;

    for(k=0;k<t;k++)
		for(l=0;l<10800;l++);
}



void UARTPutByte( char b)
{char k;
 while(ScibTx_Ready() != 1);
 ScibRegs.SCITXBUF =b;
 for(k=0;k<500;k++);
}




void ADS1222_INIT(void)
{
unsigned int Z;
long ln;

     BEN=1;


	ln=0;
    while(MISO==0)
	{ ln++;
	  {if((ln)>100000) break;} 
	}
	  
	for(Z=0;Z<26;Z++)
	  {SCLK=1;
	   asm(" RPT #64 || NOP");
	   SCLK=0;
	   asm(" RPT #40 || NOP");		
	  }
		  
	ln=0;
    while(MISO==0)
	{ ln++;
	  {if((ln)>100000) break;} 
	}

	for(Z=0;Z<26;Z++)
	  {SCLK=1;
	   asm(" RPT #64 || NOP");
	   SCLK=0;
	   asm(" RPT #40 || NOP");		
	  }
}

void ADS1222_sample(unsigned char Ch)//ѹ���¶�ͨ��Լ80ms
{
    unsigned int x,y;
	long tb,ln;
	
    BEN=1;
	
	if(Ch<2)//�ɼ�ѹ���¶�
    {
	TEN=0;	
	MUX=Ch;

	ln=0;
    while(MISO)
	{ ln++;
	  {if((ln)>100000) break;} 
	}

      asm(" RPT #12 || NOP");
      asm(" RPT #12 || NOP");
	  SCLK=0;
	  asm(" RPT #12 || NOP");
	  asm(" RPT #12 || NOP");

	    tb=0; 
		for(x=0;x<24;x++)
		{
		    SCLK=1; 
			asm(" RPT #22 || NOP");	
			SCLK=0; 
			tb=tb<<1;
			if((MISO)==1)
			  tb|=0X01;	
		    asm(" RPT #12 || NOP");			   
		 } 
		    SCLK=1; 
		    asm(" RPT #12 || NOP");		
		    SCLK=0; 
			asm(" RPT #12 || NOP");	


    ADSData=0;
    SCLK=0;

	for(y=0;y<20;y++)//�л�ͨ�����һ�����14ms,����ÿ�����4.2ms
	  { 
	  	ln=0;
	    while(MISO)
		{ ln++;
		  {if((ln)>100000) break;} 
		}

      asm(" RPT #12 || NOP");
      asm(" RPT #12 || NOP");
	  SCLK=0;
	  asm(" RPT #12 || NOP");
	  asm(" RPT #12 || NOP");

		tb=0;
	   for(x=0;x<24;x++)
		{
	     SCLK=1; 
	 	 asm(" RPT #22 || NOP");	
		 SCLK=0; 
		 tb=tb<<1;
		 if((MISO)==1)
		   tb|=0X01;
		 asm(" RPT #12 || NOP");					   
		 } 
		SCLK=1; 
		asm(" RPT #12 || NOP");	
		SCLK=0;
		asm(" RPT #12 || NOP");	
		asm(" RPT #12 || NOP");
		              
		 tb&=0x00ffffff;
	 if(Ch)//ѹ��
		    {tb>>=6;
		    if(tb&0x00020000)
	 	     tb|=0xfffc0000;
			 pdd[y]=tb;
	 	     }  
	 else //�¶�
		  {  tb>>=8;
			 if(tb&0x00008000)
	 	     tb|=0xffff0000;
			pdd[y]=tb;
	 	  }
	  } 
	  for(i=0;i<19;i++)
		  {
		   if(pdd[i]>pdd[i+1])
		    {addpt= pdd[i+1];
		     pdd[i+1]= pdd[i];
			  pdd[i]=addpt;
			}
		  }
		 
		  for(i=0;i<18;i++)
		  {
		   if(pdd[i]>pdd[i+1])
		    {addpt= pdd[i+1];
		     pdd[i+1]= pdd[i];
			  pdd[i]=addpt;
			}
		  }

		  
		  for(i=0;i<17;i++)
		  {
		   if(pdd[i]<pdd[i+1])
		    {addpt= pdd[i+1];
		     pdd[i+1]= pdd[i];
			  pdd[i]=addpt;
			}
		  }


		  for(i=0;i<16;i++)
		  {
		   if(pdd[i]<pdd[i+1])
		    {addpt= pdd[i+1];
		     pdd[i+1]= pdd[i];
			  pdd[i]=addpt;
			}
		  }

			addpt=0;
		    for(i=0;i<16;i++)
			{ addpt+=pdd[i];}

			ADSData=addpt/16;		
	}
	else//�ɼ������¶�
	{
	 TEN=1;
	 
      	ln=0;
	    while(MISO)
		{ ln++;
		  {if((ln)>100000) break;} 
		}

	    asm(" RPT #12 || NOP");
        asm(" RPT #12 || NOP");
	    SCLK=0;
	    asm(" RPT #12 || NOP");
	    asm(" RPT #12 || NOP");

		tb=0;
		for(x=0;x<24;x++)
		  {
		    SCLK=1;
			asm(" RPT #22 || NOP");
			SCLK=0; 	
			tb=tb<<1;
			if((MISO)==1)
			  tb|=0X01; 
			asm(" RPT #12 || NOP");
		  } 
		SCLK=1; 
		asm(" RPT #12 || NOP");	
		SCLK=0; 
		asm(" RPT #12 || NOP");	
		ADSData=0;	
		
		for(y=0;y<1;y++)//�л�ͨ�����һ�����14ms,����ÿ�����4.2ms
		{			
			ln=0;
		    while(MISO)
			{ ln++;
			  {if((ln)>100000) break;} 
			}

	      asm(" RPT #12 || NOP");
	      asm(" RPT #12 || NOP");
		  SCLK=0;
		  asm(" RPT #12 || NOP");
		  asm(" RPT #12 || NOP");
	  	 
		    tb=0;
			for(x=0;x<24;x++)
			{
			    SCLK=1; 
				asm(" RPT #22 || NOP");	
			    SCLK=0; 	
				tb=tb<<1;
				if((MISO)==1)
				  tb|=0X01;	
				asm(" RPT #12 || NOP");   
			 } 
			SCLK=1; 
			asm(" RPT #12 || NOP");	
			SCLK=0; 
			asm(" RPT #12 || NOP");
         
			 tb&=0x00ffffff;
			 tb>>=8;//ȡ16λ��Ч��
		   if(tb&0x00008000) 
		 	  tb|=0xffff0000;
		   ADSData+=tb;		 
	    }
	 ADSData/=1;
	 }
}


void ISRSciCom05(void)
{
 	EALLOW;
	 UD1=1;
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 CS1=0;
     asm(" RPT #254 || NOP");//256��ָ����?4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 UD1=0;
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 UD1=1;
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 UD1=0;
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us			 
	 CS1=1;		
	 EDIS;
}

void ISRSciCom06(void)
{
	EALLOW;
	 UD2=1;
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 CS2=0;
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 UD2=0;
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 UD2=1;
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 UD2=0;
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us			 
	 CS2=1;		
	 EDIS;
}

void ISRSciCom0A(void)
{
	EALLOW;
	 UD1=0;
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 CS1=0;
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 UD1=1;
	 asm(" RPT #254 || NOP");//256��ָ����?4us
	 asm(" RPT #254 || NOP");//256������?4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 CS1=1;
	 EDIS;
}

void ISRSciCom0B(void)
{
	EALLOW;
	 UD2=0;
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 CS2=0;
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 UD2=1;
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
     asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 asm(" RPT #254 || NOP");//256��ָ������=4us
	 CS2=1;
	 EDIS;
}



unsigned char CalThr(void)
{
//����һ�ֽڣ�����0x00��ʾ�ϡ����������ײ���ֵ����У׼�ɹ�
//�����ֽڸߵ͸����ֽڱ�ʾ���������Ƶ�У׼״̬
//���������Ƶ��ײ���ֵУ׼��������жϲ���ӡ
//���¸�λ��ʾ������Ϣ����Ϊ���㣬���ӡ��������Ϣ
//bit0 ��˳��ֵ����ʧ��
//bit1 �����ֵ����ʧ��
//bit2 ��˳û���ҵ����ʵ��ײ���ֵ
//bit3 ����û���ҵ����ʵ��ײ���ֵ
//bit4 ��˳��ֵ����ʧ��
//bit5 �����ֵ����ʧ��
//bit6 ��˳û���ҵ����ʵ��ײ���ֵ
//bit7 ����û���ҵ����ʵ��ײ���ֵ

  unsigned char r,k,i;
  r=0;
	
	for(k=0;k<10;k++)//20160802
	{
		    AbsTime(1);//3.5ms
				DelaymS(1);
			AbsTime(3);//3.5ms	
				DelaymS(1);		 		             
		    AbsTime(2); //3.5ms	
		    	DelaymS(1);		
			AbsTime(4);//3.5ms	
				DelaymS(1);	     			 	 
		    CalP();//������������ֵ
	}

	if( AbsTime(1)<=3)
	{
		DelaymS(1);
		if( AbsTime(1)<=3)
		{
			r|=0x01;
		}
	}

	DelaymS(1);
	if( AbsTime(2)<=3)
	{
		DelaymS(1);
		if( AbsTime(2)<=3)
		{
			r|=0x02;
		}
	}

	if((r&0x0003)==0)
	{
		if(Maxcount1[0]>Maxcount2[0])
		{
		    for(i=0;i<4;i++)
			{
				for(k=0;k<4;k++)
				{
					if((fabs(Maxcount1[i]-Maxcount2[k])<8))
					{
						tp[0]=tp1[i];
						tp[1]=tp2[k];
						i=5;k=5;	
					}
				}

				if(k==3)//û���ҵ����ʵ��ײ���ֵ
				{
					r|=0x04;
				}
			}
		}
		else 
		{
		    for(i=0;i<4;i++)
		    {
				for(k=0;k<4;k++)
				{
					if((fabs(Maxcount2[i]-Maxcount1[k])<8))
					{
						tp[0]=tp1[k];
						tp[1]=tp2[i];
						i=5;k=5;	
					}
				}

				if(k==3)//û���ҵ����ʵ��ײ���ֵ
				{
					r|=0x08;
				}
			}

		}
	}

	DelaymS(1);
	if( AbsTime(3)<=3)
	{
		DelaymS(1);
		if( AbsTime(3)<=3)
		{
			r|=0x10;
		}
	}

	DelaymS(1);
	if( AbsTime(4)<=3)
	{
		DelaymS(1);
		if( AbsTime(4)<=3)
		{
			r|=0x20;
		}
	}

	if((r&0x0030)==0)
	{
		if(Maxcount3[0]>Maxcount4[0])
		{
			for(i=0;i<4;i++)
			{
				for(k=0;k<4;k++)
				{
					if((fabs(Maxcount3[i]-Maxcount4[k])<8))
					{
						tp[2]=tp3[i];
						tp[3]=tp4[k];
						i=5;k=5;
					}
				}

				if(k==3)//û���ҵ����ʵ��ײ���ֵ
				{
					r|=0x40;
				}
			}
		}
		else 
		{
		    for(i=0;i<4;i++)
			{
				for(k=0;k<4;k++)
				{
					if((fabs(Maxcount4[i]-Maxcount3[k])<8))
					{
						tp[2]=tp3[k];
						tp[3]=tp4[i];
						i=5;k=5;
					}
				}

				if(k==3)//û���ҵ����ʵ��ײ���ֵ
				{
					r|=0x80;
				}
			}

		}
	}


	return r;
}
