
#include <iom128v.h>								   	
#include <macros.h>
#include <stdio.h>

 //20170703
 //适应新做的电机板,R2改为30K
 //20170716  开电机电源时要关模拟比较器中断（后级电容充电时约20ms，把总线电压下拉到约40V左右，正好是欠压中断的门槛），老主控板因为欠压门槛是28V，所以没事，老电机控制小板没事的原因暂不知，可能是后级电容充电比较慢。
 //20170719 电机断路档位最小为5mA，步进为10mA，电机动作时电机电压+3
#define  BB     0x12        //固件版本号

#define INT_EN		{ SEI(); }
#define INT_DIS		{ CLI(); }
 
//电机管脚定义
#define MT1_P1_Set()     (PORTG |= (1<<PORTG1))  
#define MT1_P1_Clr()     (PORTG &= ~(1<<PORTG1)) 
#define MT1_N1_Set()     (PORTG |= (1<<PORTG0))  
#define MT1_N1_Clr()     (PORTG &= ~(1<<PORTG0)) 
#define MT1_P2_Set()     (PORTD |= (1<<PORTD6))  
#define MT1_P2_Clr()     (PORTD &= ~(1<<PORTD6)) 
#define MT1_N2_Set()     (PORTD |= (1<<PORTD7))  
#define MT1_N2_Clr()     (PORTD &= ~(1<<PORTD7)) 

#define MT2_P1_Set()     (PORTA |= (1<<PORTA4))  
#define MT2_P1_Clr()     (PORTA &= ~(1<<PORTA4)) 
#define MT2_N1_Set()     (PORTA |= (1<<PORTA5))  
#define MT2_N1_Clr()     (PORTA &= ~(1<<PORTA5)) 
#define MT2_P2_Set()     (PORTA |= (1<<PORTA2))  
#define MT2_P2_Clr()     (PORTA &= ~(1<<PORTA2)) 
#define MT2_N2_Set()     (PORTA |= (1<<PORTA3))  
#define MT2_N2_Clr()     (PORTA &= ~(1<<PORTA3)) 


//开电机电源，先关模拟比较器中断，再开电机电源 ，再延时50ms(实际上开电源只需要20ms)，再清模拟比较器中断，再开模拟比较器中断
#define VM_OPEN        {ACSR&=0xf7;PORTA |= (1<<PORTA6);CheckMotor(50);ACSR|=0x10;ACSR|=0x08;}
//关电机电源
#define VM_Close()       (PORTA &= ~(1<<PORTA6)) 

//同时关二个电机
#define MOTORSTOP   {MT1_P2_Clr();MT1_N2_Clr();MT2_P2_Clr();MT2_N2_Clr();VM_Close();MT1_P1_Clr();MT1_N1_Clr();MT2_P1_Clr();MT2_N1_Clr();}

#define DELAY10 for(Tt=0;Tt<4;Tt++);
#define DELAY20 for(Tt=0;Tt<4;Tt++);
#define DELAY40 for(Tt=0;Tt<16;Tt++); 
#define DELAY50 for(Tt=0;Tt<21;Tt++); 
#define DELAY80 for(Tt=0;Tt<32;Tt++); 
#define DELAY89 for(Tt=0;Tt<36;Tt++); 
#define DELAY100 for(Tt=0;Tt<42;Tt++); 
#define DELAY170 for(Tt=0;Tt<72;Tt++);
#define DELAY268 for(Tt=0;Tt<113;Tt++);
#define DELAY397 for(Tt=0;Tt<165;Tt++);
#define DELAY400 for(Tt=0;Tt<168;Tt++);
#define DELAY500 for(Tt=0;Tt<210;Tt++);

union FIB
{
 float f;
 unsigned int i[2];
 unsigned char b[4];
}myFIB;

union MotorStatus
{
 unsigned int i;
 struct 
 { 
   unsigned int RUN:1;//bit0,电机运行状态，0：停止，1：运行
   unsigned int DIR:1;//bit1,电机运行方向，0：反转,收臂-调大(DIR_L)，1：正转,张臂-调小(DIR_H)
   unsigned int BrokenStop:1;//bit2, 断路停
   unsigned int CommandStop:1;//bit3,手动停
   unsigned int OverCurrent:1;//bit4,过流停
   unsigned int UnderVoltage:1;//bit5,欠压停
   unsigned int StartResult:2;//bit6-bit7,电机最近一次启动结果(0：正常上电启动，1：本电机同向运行，2：本电机反向运行，3：另一个电机正在运行)
   
   unsigned int ThisCurrent:8;//bit8-bit15 ,  电机当前或停止前最近一次电流
 }s;
}Motor1Status,Motor2Status;//电机状态

unsigned char Motor1Gear,Motor2Gear;// 电机过流档位
unsigned char Motor1Thr,Motor2Thr;//电机堵转过流阈值
unsigned char MotorIdelGear;//电机断路档位
unsigned char MotorIdelCur;// 电机断路电流
unsigned char R_dat; 
unsigned char IntFlag;
unsigned char Tt;


void InitialIO(void);
void Pwm_startup(unsigned char motor);
void Start(void);
unsigned char SampleADC(unsigned char ch);
void EEPROM_write(unsigned int Address,unsigned char Data);
unsigned char EEPROM_read(unsigned int Address);
void CheckMotor(unsigned int t);
void Gear2Thr(void);
void uart1_rx_isr(void);
void SendUart1(unsigned char dat);

void InitialIO(void)
{//1输出，0输入
 PORTA=0x00;
 DDRA=0x7c;  

 PORTD=0x00; 
 DDRD=0xc0; 

 PORTG=0x00; 
 DDRG=0x03; 
}


//UART1 initialize
// desired baud rate:9600
// actual baud rate:9615 (0.2%)
// char size: 8 bit
// parity: Disabled
void uart1_init(void)
{
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x02;
 UCSR1C = 0x06;
 UBRR1L = 0x67; //set baud rate lo
 UBRR1H = 0x00; //set baud rate hi
 UCSR1B = 0x98;
}

#pragma interrupt_handler uart1_rx_isr:iv_USART1_RXC
void uart1_rx_isr(void)//串口1接收中断
{
  R_dat=UDR1;
  IntFlag=1;
}

void SendUart1(unsigned char dat)
{
  while(!(UCSR1A&(1<<UDRE1)));   // 等待发送缓冲器为空
  UDR1=dat;  
}


#pragma interrupt_handler timer3_ovf_isr:iv_TIM3_OVF
void timer3_ovf_isr(void)//微调时用这个定时中断来自动关电机
{
 	ETIMSK = 0x00; //Timer3中断关闭
	TCCR3B = 0x00; //stop
	 
	MOTORSTOP  //关电机
		
    if(Motor1Status.s.RUN==1)//收放电机正在运行
   	{
		Motor1Status.s.RUN=0;
 		Motor1Status.s.CommandStop=1;
   	}
	
 	if(Motor2Status.s.RUN==1)//调节电机正在运行
   	{ 
		Motor2Status.s.RUN=0;
 		Motor2Status.s.CommandStop=1;
   	} 	     
}


void Pwm_startup(unsigned char motor)//电机软启动
{	unsigned char c,b;

	if(motor==1)
	{  
	   Motor1Status.s.RUN=1;
	   
	   if(Motor1Status.s.DIR==1)//正转
	   { 
	     MT1_N1_Clr();// 关上MOS管
		 DELAY500
		 DELAY500
		 MT1_N2_Clr();// 关下MOS管
		 DELAY500
		 DELAY500
		 
	     MT1_P1_Set();//打开上MOS管
		 DELAY500
		 
	     for(b=0;b<30;b++)//3ms采用等占空比启动电机
		 {
		  MT1_P2_Clr();//关闭下MOs管 
		  DELAY100
		  MT1_P2_Set();//打开下MOS管
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//130ms采用降低占空比启动电机
		 {
	       MT1_P2_Clr();//关闭下MOs管 
		   
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor1Status.s.UnderVoltage==0)//如意启动过程中没有发生欠压，则继续启动
		   {
		     MT1_P2_Set();//打开下MOS管
		   }
		   else 
		   {
		     MT1_P2_Clr();//关闭下MOs管 
		   }
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	   else//反转
	   {
	      MT1_P1_Clr();// 关上MOS管
		  DELAY500
		  DELAY500
		  MT1_P2_Clr();// 关下MOS管
		  DELAY500
		  DELAY500
		 
	      MT1_N1_Set();//打开上MOS管
		  DELAY500
		 
	     for(b=0;b<30;b++)//3ms采用等占空比启动电机
		 {
		  MT1_N2_Clr();//关闭下MOs管 
		  DELAY100
		  MT1_N2_Set();//打开下MOS管
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//130ms采用降低占空比启动电机
		 {
	       MT1_N2_Clr();//关闭下MOs管 
		   
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor1Status.s.UnderVoltage==0)//如意启动过程中没有发生欠压，则继续启动
		   {
		     MT1_N2_Set();//打开下MOS管
		   }
		   else 
		   {
		     MT1_N2_Clr();//关闭下MOs管 
		   }
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	} 
	else if(motor==2)
	{  
	  Motor2Status.s.RUN=1;
	  
	  if(Motor2Status.s.DIR==1)//正转
	   { 
	     MT2_N1_Clr();// 关上MOS管
		 DELAY500
		 DELAY500
		 MT2_N2_Clr();// 关下MOS管
		 DELAY500
		 DELAY500
		 
	     MT2_P1_Set();//打开上MOS管
		 DELAY500
		 
	     for(b=0;b<30;b++)//3ms采用等占空比启动电机
		 {
		  MT2_P2_Clr();//关闭下MOs管 
		  DELAY100
		  MT2_P2_Set();//打开下MOS管
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//130ms采用降低占空比启动电机
		 {
	       MT2_P2_Clr();//关闭下MOs管 
		   
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor2Status.s.UnderVoltage==0)//如意启动过程中没有发生欠压，则继续启动
		   {
		     MT2_P2_Set();//打开下MOS管
		   }
		   else 
		   {
		     MT2_P2_Clr();//关闭下MOs管 
		   }
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	   else //反转
	   {
	      MT2_P1_Clr();// 关上MOS管
		  DELAY500
		  DELAY500
		  MT2_P2_Clr();// 关下MOS管
		  DELAY500
		  DELAY500
		 
	      MT2_N1_Set();//打开上MOS管
		  DELAY500
		 
	     for(b=0;b<30;b++)//3ms采用等占空比启动电机
		 {
		  MT2_N2_Clr();//关闭下MOs管 
		  DELAY100
		  MT2_N2_Set();//打开下MOS管
		  DELAY100
		 }
 
    	 for(c=0;c<200;c++)//130ms采用降低占空比启动电机
		 {
	       MT2_N2_Clr();//关闭下MOs管 
		   
		   for(b=0;b<(200-c);b++)
		    {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			
	       if(Motor2Status.s.UnderVoltage==0)//如意启动过程中没有发生欠压，则继续启动
		   {
		     MT2_N2_Set();//打开下MOS管
		   }
		   else 
		   {
		     MT2_N2_Clr();//关闭下MOs管 
		   }
		   
	       for(b=0;b<(c+50);b++)
		   {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
		 }	
	   }
	}
	
}

   
unsigned char SampleADC(unsigned char ch)//耗时约5ms，返回8位有效结果
{     unsigned char c;
      unsigned long adcl;
	  unsigned int ADdata;

	  ADMUX  = (0xc0+ch);//片内2.56V基准，选择单端输入通道 
	  ADCSRA =0xC3;//ADC使能，ADC开始转换，ADC自动触发使能（连续转换模式），8分频 
   
	  //第一次采样值不要
	  for(ADdata=0;ADdata<100;ADdata++);
	  while((ADCSRA&0x40)==0x40);//等待转换完成
	  ADdata=ADCL;
	  ADdata=ADCH;  
  
	  adcl=0;
	  for(c=0;c<32;c++)//32次值取平均
	  {
	   ADCSRA = 0xC3;//ADC使能，ADC开始转换，ADC自动触发使能（连续转换模式），8分频 
	   for(ADdata=0;ADdata<100;ADdata++);
	   while((ADCSRA&0x40)==0x40);//等待转换完成	
	   ADdata=ADCL;   
	   ADdata|=(ADCH<<8);
	   ADdata&=0x03ff;//10位有效转换结果
	   adcl+=ADdata;
	  }

	  adcl>>=7;//除以32，再除以4，相当于8位AD
	  return (adcl&0x000000ff);//取8位有效位
}

void CheckMotor(unsigned int t)//电机不转时耗时5us，最大定时8388ms，参数t为定时时间，单位ms
{   //电机电流单片机采样管脚上，没开电机时约70mV，开电机后空载时约80mV，于是下文需要把这个零位减掉。 
    float f;
	unsigned char a;
    unsigned int n;
    TCCR1B = 0x00; //stop
	if(t>8388)
	   t=8388;
	f=t;
	f*=7.8125;
	n=f;
    TCNT1 = 65535-n;  
	TIFR |= 0x04; //清定时器1中断标志 
 	TCCR1B = 0x05; //1024分频
	
   while((TIFR&0x04)==0x00)//定时中断到来前一直采样监测电流
   {   
    if( Motor1Status.s.RUN==1 )//收放电机运行时才实时检测电流，否则电流保存最后一次的测试值
    {
	  a=SampleADC(0);
	  if(a>5){a-=5;}// 减掉硬件零位，每个数字量代表10mV，单片机的ADC采样偏小，于是在这儿只抵消50m即可。
	  else{a=0;}
	  
      if(a>Motor1Thr)//过流关电机
      {
	   MOTORSTOP//关电机   
       Motor1Status.s.OverCurrent=1;//过流状态置1，表示电机上次停止原因为过流停
	   Motor1Status.s.RUN=0;//停止状态
	  }
	  else if(a<MotorIdelCur)//断路关电机 
	  {   
	   MOTORSTOP//关电机
       Motor1Status.s.BrokenStop=1;//断路状态置1，表示电机上次停止原因为断路停
	   Motor1Status.s.RUN=0;//停止状态
	  }
	  
	  f=a;
	  f*=1.087;//按采样电阻1.2欧，放大倍数7.667，8位AD，2.56V参考计算
	  Motor1Status.s.ThisCurrent=f;
    }
   
    if( Motor2Status.s.RUN==1 )//调节电机运行时才实时检测电流，否则电流保存最后一次的测试值
    {
	 a=SampleADC(1);	
	 if(a>5){a-=5;}// 减掉硬件零位，每个数字量代表10mV，单片机的ADC采样偏小，于是在这儿只抵消50m即可。
	  else{a=0;}
	  
     if(a>Motor2Thr)//过流关电机
     {
	   MOTORSTOP//关电机
       Motor2Status.s.OverCurrent=1;//过流状态置1，表示电机上次停止原因为过流停
	   Motor2Status.s.RUN=0;//停止状态
	 } 
	 else if(a<MotorIdelCur) //断路关电机，电流小于xx mA认为是断路
	 {
	   MOTORSTOP//关电机
       Motor2Status.s.BrokenStop=1;//断路状态置1，表示电机上次停止原因为断路停
	   Motor2Status.s.RUN=0;//停止状态
	 }

	  f=a;
	  f*=1.087;//按采样电阻1.2欧，放大倍数7.667，8位AD，2.56V参考计算
	  Motor2Status.s.ThisCurrent=f;
    }

    if(t==0){break;}//不定时则检测一次电机状态后直接跳出
   } 
   
   TCCR1B = 0x00; //stop
}


void Start(void)
{
 InitialIO();
 uart1_init();
 
 MOTORSTOP//关电机
 
 Motor1Status.s.RUN=0; 
 Motor1Status.s.DIR=0;
 Motor1Status.s.BrokenStop=0;
 Motor1Status.s.CommandStop=0;
 Motor1Status.s.OverCurrent=0;
 Motor1Status.s.UnderVoltage=0;
 Motor1Status.s.StartResult=0;
 Motor1Status.s.ThisCurrent=0;
 
 Motor2Status.s.RUN=0; 
 Motor2Status.s.DIR=0;
 Motor2Status.s.BrokenStop=0;
 Motor2Status.s.CommandStop=0;
 Motor2Status.s.OverCurrent=0;
 Motor2Status.s.UnderVoltage=0;
 Motor2Status.s.StartResult=0;
 Motor2Status.s.ThisCurrent=0;
 
 IntFlag=0;//无接收帧中断
 
 ACSR|=0x10;//清模拟比较器中断标志
 ACSR&=0xf7;//关模拟比较器中断
 ACSR|=0x40;//ACBG置位，模拟比较器正级接内部能隙基准1.23V
 ACSR|=0x03;//开模拟比较器,AIN1连到比较器负极，0:比较器输出变化即中断，1：保留，2：下降沿中断，3:上升沿中断   
 ACSR|=0x08;//开模拟比较器中断，任何时候都要进行欠压监测	
 ACSR|=0x10;//清模拟比较器中断标志	
	
 INT_EN
}

void Gear2Thr(void)
{
  //按采样电阻1.2欧，放大倍数7.667，8位AD，2.56V参考计算，每个采样字表示1.087mA
//将电机过流档位换算为数字量，方便中断函数中进行比较运算
  if(Motor1Gear<1) {Motor1Gear=1;}//堵转电流最小为1档
  if(Motor1Gear>6) {Motor1Gear=6;}//堵转电流最大为6档
  if(Motor2Gear<1) {Motor2Gear=1;}//堵转电流最小为1档
  if(Motor2Gear>6) {Motor2Gear=6;}//堵转电流最大为6档
  //第1档从100mA起步，每档差值30mA,第6档约250mA
  Motor1Thr= 108+ (Motor1Gear-1)*27;
  Motor2Thr= 108+ (Motor2Gear-1)*27;
  
  if(MotorIdelGear<1){MotorIdelGear=1;}//电机断路电流最小1档
  if(MotorIdelGear>6){MotorIdelGear=6;}//电机断路电流最大6档
  //从5mA起步，最大55mA
  MotorIdelCur=(MotorIdelGear-1)*11+4;
  
}

void main(void)
{ 
  unsigned char i;
  unsigned int j;
  unsigned long l;
  float f;
 
	for(l=0;l<1000;l++)
	{
		for(j=0;j<1000;j++);
	}

	Start();

	CheckMotor(50); 

	Motor1Gear=EEPROM_read(0);
	Motor2Gear=EEPROM_read(1);
	MotorIdelCur=EEPROM_read(2);
	Gear2Thr();//档位转换为阈值
  

	while(1)
	{ 
		CheckMotor(0);//实时进行电机过流检测

		if(IntFlag==1)//接收到了正常命令帧
		{
				switch(R_dat&0xf0)
				{ 		
				    case 0xd0://读版本号
					   SendUart1(BB);  
					break;		
					 
					case 0x30:// 采样电机电压,R7取30k时，最大能测得约88V
						f=SampleADC(2);
						f=f*0.34333;//按内部参考2.56V，8位AD采样，分压电阻分别是1M和30K来计算					
						i=f;
						 if( (Motor1Status.s.RUN==1) || (Motor2Status.s.RUN==1) )
						 {
						  i+=3;//为了用户看起来好看些
						 }
						SendUart1(i);   
						break;

					case 0x60:  //收放电机 
						if( !(((R_dat)==0x61)  || ((R_dat)==0x62) ) )//既不是正转，也不是反转，则立即返回且不响应上位机
							break;

						if( Motor2Status.s.RUN==1 )//调节电机正在运行
						{
							Motor1Status.s.StartResult=3;//另一电机正在运行
						}
						else if((Motor1Status.s.RUN==1)&&(Motor1Status.s.DIR==1))//收放电机正在正转
						{
						  if(R_dat==0x61)
						  {
						    Motor1Status.s.StartResult=1;//本电机正在同向运行
						  }
						  else if(R_dat==0x62)
						  {
						   Motor1Status.s.StartResult=2;//本电机正在反向运行
						  }

						}
						else if((Motor1Status.s.RUN==1)&&(Motor1Status.s.DIR==0))//收放电机正在反转
						{
						  if(R_dat==0x61)
						  {
						   Motor1Status.s.StartResult=2;//本电机正在反向运行 
						  }
						  else if(R_dat==0x62)
						  {
						   Motor1Status.s.StartResult=1;//本电机正在同向运行
						  }
						}
						else //二个电机都是停止状态，按命令要求启动电机，约1.5秒后反馈上位机
						{  
							//全部电机状态置初值，上一次停止原因全部清除
							Motor1Status.s.BrokenStop=0;
							Motor1Status.s.CommandStop=0;
							Motor1Status.s.OverCurrent=0;
							Motor1Status.s.UnderVoltage=0;
							Motor1Status.s.StartResult=0;//正常启动

							if((R_dat)==0x61)//张开命令
							{ 
								  //张臂，正转
								Motor1Status.s.DIR=1;
							}
							else if((R_dat)==0x62)          
							{ 
								 //收臂，反转
								Motor1Status.s.DIR=0;
							}
							else
							{
								return;//20170609
							}

							VM_OPEN//开电机电源

							CheckMotor(500);//开电机电源后需等待半秒让大功率二极管电压稳定


							TCCR1B = 0x00; //stop
							TCNT1 = 61629;   //定时500ms
							TIFR|=0x04; //清定时器1中断标志
							TCCR1B = 0x05; //1024分频
							TIFR|=0x04; //清定时器1中断标志
							Pwm_startup(1);//PWM启动,电机启动前期500ms内不进行电流检查，但有欠压监测		
							while((TIFR&0x04)==0x00); //等待500ms定时到，期间可能有欠压中断发生
							TCCR1B = 0x00; //stop

							CheckMotor(500);//延时保证总线电压稳定		
						}
			 
			            //返回2字节电机状态
						SendUart1(Motor1Status.i); CheckMotor(2);
						SendUart1(Motor1Status.i>>8);

						break;

					case 0x70://调节电机   
						if( ((R_dat)<0x71)  || ((R_dat)>0x7c)  )//既不是调大，也不是调小，也不微调大，也不是微调小，则立即返回且不响应上位机
						break;

						i=R_dat%2;
						
						if(Motor1Status.s.RUN==1)//收放电机正在运行
						{
							Motor2Status.s.StartResult=3;//另一电机正在运行
						}
						else if((Motor2Status.s.RUN==1)&&(Motor2Status.s.DIR==1))//调节电机正在正转
						{
						  if(i==0)//偶数命令，正转
						  {
						   Motor2Status.s.StartResult=1;//本电机正在同向运行
						  }
						  else//奇数命令，反转
						  {
						   Motor2Status.s.StartResult=2;//本电机正在反向运行
						  }
						}
						else if((Motor2Status.s.RUN==1)&&(Motor2Status.s.DIR==0))//调节电机正在反转
						{
						  if(i==0)//偶数命令，正转
						  {
						   Motor2Status.s.StartResult=2;//本电机正在反向运行
						  }
						  else//奇数命令，反转
						  {
						   Motor2Status.s.StartResult=1;//本电机正在同向运行
						  }
						}					
						else //二个电机都是停止状态，按命令要求启动电机，约1.5秒后反馈上位机
						{  
							//全部电机状态置初值，上一次停止原因全部清除
							Motor2Status.s.BrokenStop=0;
							Motor2Status.s.CommandStop=0;
							Motor2Status.s.OverCurrent=0;
							Motor2Status.s.UnderVoltage=0;
							Motor2Status.s.StartResult=0;//正常启动

							if( i==1 )//奇数命令，调大
							{ 
								//反转
								Motor2Status.s.DIR=0;
							}
							else if( i==0 )    //偶数命令，调小
							{ 
								//正转
								Motor2Status.s.DIR=1;
							}
							else
							{
								return;//20170609
							}

							VM_OPEN//开电机电源

							CheckMotor(500);//开电机电源后需等待半秒让大功率二极管电压稳定


							TCCR1B = 0x00; //stop
							TCNT1 = 61629;   //定时500ms
							TIFR|=0x04; //清定时器1中断标志
							TCCR1B = 0x05; //1024分频
							TIFR|=0x04; //清定时器1中断标志
							Pwm_startup(2);//PWM启动，电机启动前期500ms内不进行电流检查，但有欠压监测					
							while((TIFR&0x04)==0x00); //等待500ms定时到，期间可能有欠压中断发生
							TCCR1B = 0x00; //stop

							CheckMotor(500);//延时保证总线电压稳定		 	 
						}
						
						//返回2字节电机状态
						SendUart1(Motor2Status.i); CheckMotor(2);
						SendUart1(Motor2Status.i>>8);
						 	
						if(R_dat>0x72)//微调，需要定时，定时到后自动停止
						{
							ETIFR=0x04;//清Timer3溢出中断标志
							ETIMSK = 0x04; //Timer3溢出中断使能，其它中断关闭
							TCCR3B = 0x00;

							j=(R_dat-0x72+1)/2;
							j+=1;// 为了上位机软件能看到电流变化 ，每档都多加一秒
							j=j*7812;
							j=65536-j;
							j+=1;

							TCNT3H = j>>8;  //最小1秒，最多5秒
							TCNT3L = j;

							TCCR3A = 0x00;
							TCCR3B = 0x05; //8M时钟，1024分频，定时1秒
							ETIFR=0x04;//清Timer3溢出中断标志
						} 
						break;

					case 0x80://电机相关
						if(R_dat==0x80)//只开电机电源，不开电机
						{        	  						 
							VM_OPEN//开电机电源
						    SendUart1(0); 
						}	  
						else if(R_dat==0x81)//获取电机状态	
						{ 
							CheckMotor(2); 
							//返回2字节电机状态
						     SendUart1(Motor1Status.i); 
							 CheckMotor(2); 
						     SendUart1(Motor1Status.i>>8); 
							 CheckMotor(2); 
							//返回2字节电机状态
						    SendUart1(Motor2Status.i);  
							CheckMotor(2); 
						    SendUart1(Motor2Status.i>>8); 
						}	  
						else if(R_dat==0x82)//电机停止，同时停止二个电机
						{	 
						    MOTORSTOP  //关电机
							if(Motor1Status.s.RUN==1)//收放电机正在运行
							{
							    Motor1Status.s.RUN=0;
								Motor1Status.s.CommandStop=1;
							}
							if(Motor2Status.s.RUN==1)//调节电机正在运行
							{ 
							    Motor2Status.s.RUN=0;
								Motor2Status.s.CommandStop=1;
							}
							
							CheckMotor(180);//让总线稳定后再回发
							SendUart1(0);  
						}
						else if(R_dat==0x83)//读取电机堵转电流档位和断路档位
						{
							Motor1Gear=EEPROM_read(0);//收放电机
							Motor2Gear=EEPROM_read(1);//调节电机
							MotorIdelGear=EEPROM_read(2); //电机断路
							Gear2Thr();

							i=Motor2Gear;	 
							i<<=4;
							i|=Motor1Gear; 
							SendUart1(i); 
							CheckMotor(2);
							i=MotorIdelGear;
							SendUart1(i); 
						}	
						else if(R_dat>0x89)//0x8a~0x8f,设置调节电机过流堵转档位
						{
						    Motor2Gear=R_dat-0x89;//对应1~6档 
							EEPROM_write(1,Motor2Gear);
							Gear2Thr();
							SendUart1(0); 
						}	
						else if(R_dat>0x83)//0x84~0x89,设置收放电机堵转档位
						{
						    Motor1Gear=R_dat-0x83;//对应1~6档 
							EEPROM_write(0,Motor1Gear);
							Gear2Thr();
							SendUart1(0); 
						}	  
						break;

					case 0x90: 
						if(R_dat>0x99)//0x9a~0x99，设置电机断路电流档位
						{
							MotorIdelGear=R_dat-0x99;
							EEPROM_write(2,MotorIdelGear);  
							Gear2Thr(); 
							SendUart1(0);  
						}				
						break;

					default: 
						break;	
				}
			IntFlag=0;
		}  
	}
}

#pragma interrupt_handler E_comp:24
void E_comp(void)  //模拟比较器中断,总线电压被拉低到42.23V后，立即停电机(R2要改成30k)
{   
  MOTORSTOP  //关电机 
  if(Motor1Status.s.RUN==1)//收放电机欠压堵转
   {
 	 Motor1Status.s.UnderVoltage=1;
	 Motor1Status.s.RUN=0;//停止状态
   }
  if(Motor2Status.s.RUN==1)//调节电机欠压堵转
   { 
 	 Motor2Status.s.UnderVoltage=1;
	 Motor2Status.s.RUN=0;//停止状态
   }   
}


void EEPROM_write(unsigned int Address,unsigned char Data)  	//内部EEPROM写
{//时钟频率为1MHz，典型的EEPROM字节写耗时约8.5ms
    while(EECR&0x02);                   // 等待上一次写操作结束
    EEAR=Address;
    EEDR=Data;                        	// 设置地址和数据寄存器
    EECR|=0x04;                         //置位EEMWE
    EECR|=0x02;                         //置位EEWE 以启动写操作
}

unsigned char EEPROM_read(unsigned int Address)  				//内部EEPROM读
{
    while(EECR&0x02);      				//等待上一次操作结束
    EEAR = Address;                     //设置地址寄存器						
	EECR|=0x01;             			//设置EERE 以启动读操作
    return EEDR;                   		//自数据寄存器返回数据
}    

 