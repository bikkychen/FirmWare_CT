



//**********************************************
//电机控制相关宏定义

//收放电机
#define MT1_EN			{ PORTA &= 0xFE; }
#define MT1_DIS		{ PORTA |= 0x01; }

#define MT1_STOP		{ PORTA &= 0xF9; }

#define MT1_RUNP_H	{ PORTA |= 0x02; }
#define MT1_RUNP_L	{ PORTA &= 0xFD; }

#define MT1_RUNN_H	{ PORTA |= 0x04; }
#define MT1_RUNN_L	{ PORTA &= 0xFB; }

//调节电机
#define MT2_EN			{ PORTF &= 0xFD; }
#define MT2_DIS		{ PORTF |= 0x02; }

#define MT2_STOP		{ PORTF &= 0xFA; }

#define MT2_RUNP_H	{ PORTF |= 0x04; }
#define MT2_RUNP_L	{ PORTF &= 0xFB; }

#define MT2_RUNN_H	{ PORTF |= 0x01; }
#define MT2_RUNN_L	{ PORTF &= 0xFE; }

//电机状态引脚
#define PIN_MT1_RUNP		( PINA & 0x02 )
#define PIN_MT1_RUNN		( PINA & 0x04 )
#define PIN_MT2_RUNP		( PINF & 0x04 )
#define PIN_MT2_RUNN		( PINF & 0x01 )

//电机电源
#define MT_PWON       { DDRB|=0x80; PORTB|=0x80; }
#define MT_PWOFF      { DDRB|=0x80; PORTB&=0x7F; }

#define VM_H        {DDRB|=0x80;PORTB|=0x80;}



void Mt1_Run(void)
{
	//非正转，非反转，退出
	if( !(((R_dat)==0x61)  || ((R_dat)==0x62) ) )
		break;
	
	//电机2正在运行
	if ( (PIN_MT2_RUNP != 0) || (PIN_MT2_RUNN != 0) )
	{
		Motor1Status.s.StartResult=3;
		CheckMotor(100);
	}
	//电机1正在正转
	else if (PIN_MT1_RUNP != 0)
	{
		if (R_dat==0x61)
		{
			Motor1Status.s.StartResult=1;
		}
		else if (R_dat==0x62)
		{
			Motor1Status.s.StartResult=2;
		}
		CheckMotor(100);
	}
	else if (PIN_MT1_RUNN != 0)
	{
		if (R_dat==0x61)
		{
			Motor1Status.s.StartResult=2;
		}
		else if (R_dat==0x62)
		{
			Motor1Status.s.StartResult=1;
		}
		CheckMotor(100);
	}
	else
	{
		Motor1Status.s.BrokenStop=0;
		Motor1Status.s.CommandStop=0;
		Motor1Status.s.OverCurrent=0;
		Motor1Status.s.UnderVoltage=0;
		Motor1Status.s.StartResult=0;
		
		MT_PWON
		
		CheckMotor(500);	// stay for stable

		TCCR1B = 0x00;		//stop
		TCNT1 = 61629;		// timer1, 500ms
		TIFR|=0x04; 		// clear int flag
		TCCR1B = 0x05;		//
		TIFR|=0x04;			// 
		
		if (R_dat==0x61)
		{
			Pwm_startup(1, 1);	// 200ms, no current check
		}
		else if (R_dat==0x62)
		{
			Pwm_startup(1, 0);	// 200ms, no current check
		}
				
		while((TIFR&0x04)==0x00); 	//wait for timer1, low voltage int may occur 
		TCCR1B = 0x00; 					//stop

		CheckMotor(500);	// stay for stable	
	}
	T_dat=Motor1Status.i;
	SendManchester();
	
	break;
	
}

void Mt2_Run(void)
{
	if( ((R_dat)<0x71)  || ((R_dat)>0x7c)  )
		break;
	
	i=R_dat%2;
	//电机1正在运行
	if ( (PIN_MT1_RUNP != 0) || (PIN_MT1_RUNN != 0) )
	{
		Motor2Status.s.StartResult=3;
		CheckMotor(100);
	}
	//电机1正在正转
	else if (PIN_MT2_RUNP != 0)
	{
		if (i == 0)
		{
			Motor2Status.s.StartResult=1;
		}
		else if (i == 1)
		{
			Motor2Status.s.StartResult=2;
		}
		CheckMotor(100);
	}
	else if (PIN_MT2_RUNN != 0)
	{
		if (i == 0)
		{
			Motor2Status.s.StartResult=2;
		}
		else if (i == 1)
		{
			Motor2Status.s.StartResult=1;
		}
		CheckMotor(100);
	}
	else
	{
		Motor2Status.s.BrokenStop=0;
		Motor2Status.s.CommandStop=0;
		Motor2Status.s.OverCurrent=0;
		Motor2Status.s.UnderVoltage=0;
		Motor2Status.s.StartResult=0;
		
		MT_PWON
		
		CheckMotor(500);	// stay for stable

		TCCR1B = 0x00;		//stop
		TCNT1 = 61629;		// timer1, 500ms
		TIFR|=0x04; 		// clear int flag
		TCCR1B = 0x05;		//
		TIFR|=0x04;			// 
		
		if (i == 0)
		{
			Pwm_startup(2, 1);	// 200ms, no current check
		}
		else if (i == 1)
		{
			Pwm_startup(2, 0);	// 200ms, no current check
		}
				
		while((TIFR&0x04)==0x00); 	//wait for timer1, low voltage int may occur 
		TCCR1B = 0x00; 					//stop

		CheckMotor(500);	// stay for stable	
	}
	T_dat=Motor2Status.i;
	SendManchester();
	
	if(R_dat>0x72)
	{
		ETIFR=0x04;
		ETIMSK = 0x04;
		TCCR3B = 0x00;

		j=(R_dat-0x72+1)/2;
		j=j*7812;
		j=65536-j;
		j+=1;

		TCNT3H = j>>8;
		TCNT3L = j;

		//TCNT3H = 0xE1;  //1??
		//TCNT3L = 0x7C;

		//TCNT3H = 0xC2; //2??
		//TCNT3L = 0xF7;

		// TCNT3H = 0xA4; //3??
		// TCNT3L = 0x73;

		TCCR3A = 0x00;
		TCCR3B = 0x05; 	//8MHz, 1024 prescale, time 1s
		ETIFR=0x04;
	} 
	
	break;
}
	
void Pwm_startup(unsigned char motor, unsigned char dir)
{	
	unsigned char c,b;

	if(motor == 1)
	{      
		if (dir == 1)
		{
			 for(b=0;b<30;b++)	//6ms
			 {
				  MT1_RUNP_H
				  DELAY100
				  MT1_RUNP_L
				  DELAY100
			 }
	 
			 for(c=0;c<200;c++)//150ms
			 {
				MT1_RUNP_L
				for(b=0;b<(200-c);b++)
				{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}

				if(Motor1Status.s.UnderVoltage==0)
				{MT1_RUNP_H}
				else 
				{MT1_RUNP_L}

				for(b=0;b<(c+50);b++)
				{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			 }	
		}
	    else
		{
			for(b=0;b<30;b++)	//6ms
			 {
				  MT1_RUNN_H
				  DELAY100
				  MT1_RUNN_L
				  DELAY100
			 }
	 
			 for(c=0;c<200;c++)//150ms
			 {
				MT1_RUNN_L
				for(b=0;b<(200-c);b++)
				{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}

				if(Motor1Status.s.UnderVoltage==0)
				{MT1_RUNN_H}
				else 
				{MT1_RUNN_L}

				for(b=0;b<(c+50);b++)
				{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			 }
		}
	} 
	else if(motor==2)
	{  
	     if (dir == 1)
		{
			 for(b=0;b<30;b++)	//6ms
			 {
				  MT2_RUNP_H
				  DELAY100
				  MT2_RUNP_L
				  DELAY100
			 }
	 
			 for(c=0;c<200;c++)//150ms
			 {
				MT2_RUNP_L
				for(b=0;b<(200-c);b++)
				{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}

				if(Motor1Status.s.UnderVoltage==0)
				{MT2_RUNP_H}
				else 
				{MT2_RUNP_L}

				for(b=0;b<(c+50);b++)
				{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			 }	
		}
	    else
		{
			for(b=0;b<30;b++)	//6ms
			 {
				  MT2_RUNN_H
				  DELAY100
				  MT2_RUNN_L
				  DELAY100
			 }
	 
			 for(c=0;c<200;c++)//150ms
			 {
				MT2_RUNN_L
				for(b=0;b<(200-c);b++)
				{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}

				if(Motor1Status.s.UnderVoltage==0)
				{MT2_RUNN_H}
				else 
				{MT2_RUNN_L}

				for(b=0;b<(c+50);b++)
				{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}
			 }
		}
	}
}

