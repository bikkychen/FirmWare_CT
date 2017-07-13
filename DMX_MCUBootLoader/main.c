


#include <macros.h>
#include <stdio.h>
#include	"HAL.H"			// 其它单片机需修改HAL*硬件抽象层的几个文件

#define DBG 0


unsigned int flash_bufferPoint; 
char flash_buf[256]; //FALSH页缓冲区,M128的一个Flash页为256字节(128字) 
long address = 0; 
   

unsigned char tx_data[64];
unsigned char UpdateFlag=0;



extern void USBStart(void);
extern void EP2SendConst(UINT8 mLen, const unsigned char  *mBuf);
extern unsigned char	UsbConfig ,UsbReset;	 

//USB接收缓冲区数据结构
typedef struct RXDATA
{
 unsigned char flag;
 unsigned char len;
 unsigned char buf[64];
};
struct RXDATA RxData;


 
unsigned char UpdateBegin;//接收主程序数据开始标志
unsigned int DataReCn;
unsigned char crc16hi,crc16lo,DownloadSpeed;
unsigned int Timer3Cn;
	   
extern void	Init374Device( void );  // 初始化USB设备
extern void EP2Send(UINT8 mLen, PUINT8 mBuf);
extern unsigned char	UsbConfig ;	// USB配置标志

void DelayMs(unsigned int t);

//OSC=8M Hz
//UART0 initialize
// desired baud rate: 38400
// actual: baud rate:37500 (2.4%)
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x0B; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x18;
}



void port_init(void)
{
 PORTA = 0x00; 
 DDRA  = 0x00; 
 
 PORTB = 0x10; 
 DDRB  = 0x10; 
 
 PORTC = 0x00; 
 DDRC  = 0x00;
 
 PORTD = 0x00; 
 DDRD  = 0x00; 
 
 PORTE = 0x00;
 DDRE  = 0x00;
 
 PORTF = 0x00; 
 DDRF  = 0x00; 
 
 PORTG = 0x00;
 DDRG  = 0x00;
}

void DelayMs(unsigned int t)
{
   float f;
   unsigned int n;
   if(t>8388)
     t=8388;
   f=t;
   f*=15.625;
   n=f;
   TCNT1=65536-n;
   TCCR1B=0x05;//启动定时器，1024分频
   TIFR|=0x04;//清定时器1中断标志
   while((TIFR&0x04)==0x00);
   TCCR1B=0x00;//停止定时器
}


void crc16(unsigned char r_data[],unsigned int length)
{
        unsigned char cl,ch;
        unsigned char savehi,savelo;
        int ii,flag;
        crc16hi=0xFF;
        crc16lo=0xFF;
        cl=0x1;
        ch=0xA0;
        for (ii=0;ii<length-2;ii++)
        {
            crc16lo=(crc16lo ^ r_data[ii]);
                for (flag=0;flag<8;flag++)
                {
                       savehi=crc16hi;
                        savelo=crc16lo;
                       crc16hi=(crc16hi>>1);
                       crc16lo=(crc16lo>>1);
                        if ((savehi & 0x01)==0x01)
                                crc16lo=(crc16lo | 0x80);
                        if        ((savelo & 0x01)==0x01)
                        {
                                crc16hi = (crc16hi ^ ch);
								crc16lo = (crc16lo ^ cl);
                        }        
                }
        }
}

#pragma interrupt_handler timer3_ovf_isr:iv_TIM3_OVF
void timer3_ovf_isr(void)
{
  UpdateBegin=0;//回到待命状态
  TCCR3B = 0x00; //stop  
  ETIMSK &= 0xfb;//关定时器3中断
  ETIFR|=0x04; //清定时器3中断标志 
}


void SetTimer3(void)//页写开始后，如果有500ms时间内一个字节也没收到，则退出页写状态，回到待命状态
{  
    TCCR3B = 0x00; //stop    
    TCNT3H = 0xF0; //8M,500ms
    TCNT3L = 0xBE; //8M,500ms
	ETIFR|=0x04; //清定时器3中断标志
 	TCCR3B = 0x05; //1024分频
	ETIFR|=0x04; //清定时器3中断标志
	ETIMSK |= 0x04; //开定时器3中断
}
//call this routine to initialize all peripherals
void init_devices(void)
{
 //stop errant interrupts until set up
 CLI(); //disable all interrupts
 XDIV  = 0x00; //xtal divider
 XMCRA = 0x00; //external memory
 port_init();
#if DBG
 uart0_init();
#endif

 MCUCR = 0x00;
 EICRA = 0x00; //extended ext ints
 EICRB = 0x00; //extended ext ints
 EIMSK = 0x00;
 TIMSK = 0x00; //timer interrupt sources
 ETIMSK = 0x00; //extended timer interrupt sources
 SEI(); //re-enable interrupts
 //all peripherals are now initialized
}


//擦除(code=0x03)和写入(code=0x05)一个Flash页 
void boot_page_ew(long p_address,char code) 
{ 
    asm("mov r30,r16\n" 
        "mov r31,r17\n" 
        "out 0x3b,r18\n");            //将页地址放入Z寄存器和RAMPZ的Bit0中 
    SPMCSR = code;                //寄存器SPMCSR中为操作码 
    asm("spm\n");                    //对指定Flash页进行操作 
}         
//填充Flash缓冲页中的一个字 
void boot_page_fill(unsigned int address,int data) 
{ 
    asm("mov r30,r16\n" 
        "mov r31,r17\n"             //Z寄存器中为填冲页内地址 
        "mov r0,r18\n" 
        "mov r1,r19\n");            //R0R1中为一个指令字 
    SPMCSR = 0x01; 
    asm("spm\n"); 
} 
//等待一个Flash页的写完成 
void wait_page_rw_ok(void) 
{ 
      while(SPMCSR & 0x40) 
     { 
         while(SPMCSR & 0x01); 
         SPMCSR = 0x11; 
         asm("spm\n"); 
     } 
} 

/*
//更新一个Flash页的完整处理 
void write_one_page(void) 
{ 
    int i; 
    boot_page_ew(address,0x03);                    //擦除一个Flash页 
    wait_page_rw_ok();                            //等待擦除完成 
	 
    for(i=0;i<256;i+=2)                //将数据填入Flash缓冲页中 
    { 
        boot_page_fill(i, flash_buf[i]+(flash_buf[i+1]<<8)); 
    } 
    boot_page_ew(address,0x05);                    //将缓冲页数据写入一个Flash页 
    wait_page_rw_ok();                            //等待写入完成 
}        
*/

//更新一个Flash页的完整处理 
void write_one_page(void) 
{ 
    int i; 
    boot_page_ew(address,0x03);                    //擦除一个Flash页 
    
    wait_page_rw_ok();                            //等待擦除完成 
    for(i=0;i<256;i+=2)                //将数据填入Flash缓冲页中 
    { 
        boot_page_fill(i, flash_buf[i+2]+(flash_buf[i+3]<<8)); 
    } 
    boot_page_ew(address,0x05);                    //将缓冲页数据写入一个Flash页 
   
   
    wait_page_rw_ok();                            //等待写入完成 
}        


void delay(unsigned int t) //y=x+3(us)
{
 while(t--);
}
 

void quit(void) 
{
 	 UCSR0B = 0x00; //disable 
	 
    MCUCR = 0x01; 
    MCUCR = 0x00;       //将中断向量表迁移到应用程序区头部 
    RAMPZ = 0x00;       //RAMPZ清零初始化 
    asm("jmp 0x0000\n");//跳转到Flash的0x0000处，执行用户的应用程序 
} 

//主程序 
void main(void) 
{ 
   unsigned int i ; 
   unsigned char j;
   unsigned char int0,miso;
  
   delay(1000);
	  
   init_devices();

   RxData.flag=0;
 
 
	 
     
   //内部上拉外部下拉时，输出变输入后，口线立即变高，5us后变低
   SFIOR&=0xfb;   //上拉不禁止,bit2=0
   
   PORTB|=0x08;  //PB3内部上拉
   DDRB&=0xf7;   //PB3设为输入 
   
   PORTD|=0x01;  //PD0内部上拉
   DDRD&=0xfe;   //PD0设为输入 
   
   SFIOR&=0xfb;   //上拉不禁止,bit2=0
   
   PORTB|=0x08;  //PB3内部上拉
   DDRB&=0xf7;   //PB3设为输入   
   PORTB|=0x08;  //PB3内部上拉
   
   PORTD|=0x01;  //PD0内部上拉
   DDRD&=0xfe;   //PD0设为输入 
   PORTD|=0x01;  //PD0内部上拉
   
   DelayMs(100);
   
 
 
      CLI();
      EIMSK = 0x41;
	  SEI();
   #if DBG
      printf("Communication Status\r\n");
   #endif
      RxData.flag=0;
   
      //内部上拉外部下拉时，输入变输出后，口线立即变低
	  //初始化SPI总线
	  
	  
	  DDRB|=0x07;//PB0/1/2设为输出 
      DelayMs(30);
      SPCR = 0x00; //SPI必须先禁止，再使能，否则会使能不成功
	  DelayMs(30);
	  /*
	  SPCR = 0x5d; //setup SPICLK=F0sc/16
	  SPSR = 0x00; //setup SPI时钟频率不双倍
	  SPCR = 0x5d; //setup SPICLK=F0sc/16
	  */
	  
	  SPCR = 0x5f; //setup SPI SPICLK=Fosc/128
	  SPSR = 0x01; //setup SPI SPI时钟频率双倍
	  SPCR = 0x5f; //setup SPI SPICLK=Fosc/128
	  
	  address=0 ;
      flash_bufferPoint=0; 
      UpdateFlag=0;
	  
	  USBStart();
	  
	#if DBG   
	  printf("Init374Device Begin\r\n");
	#endif
	  
	 for(j=0;j<100;j++)	  //常亮3秒，等待PC机加载USB驱动，如是安装驱动，则不受超时限制，直到驱动安装完毕
	 {
	  while(UsbReset==1)
	   {
	    if(UsbConfig == 1)
		 {
		  break;
		 }
	   }
	  if(UsbConfig == 1) 
	    {
	     break; //等待USB驱动安装成功
		}
	  DelayMs(30); 
	  }
	 

	 if(j>=100)//在规定时间内没有加载驱动，说明地面回放仪没有连上电脑，则直接进入到采样状态
	  {
	     quit();
	  }
	  
	 
	  #if DBG
	  printf("Init374Device End\r\n");
	  #endif
	  
	 
	  #if DBG
	  printf("UsbConfig OK\r\n");
	  #endif
       
	  UCSR0B = 0x00; //disable 
	   
	 TCCR3B = 0x00; //stop    
	 TCNT3H = 0x48; //16M,3s
      TCNT3L = 0xE5;
	ETIFR|=0x04; //清定时器3中断标志
	ETIMSK &= 0xfb;//关定时器3中断
 	TCCR3B = 0x05; //1024分频
	ETIFR|=0x04; //清定时器3中断标志
	    
	UpdateBegin=0;
	while( ((ETIFR&0x04)==0x00) && (UpdateBegin==0) )
	{
	 if((RxData.flag==1))//3秒内收到开始更新命令
	 {  
	  if(RxData.buf[4]==0x01)
	   {
	    		tx_data[0]=0xe7;  //帧头
				tx_data[1]=0xe7; //帧头
				tx_data[2]=0x00; //地址1
				tx_data[3]=0x00;   //地址2
				tx_data[4]=RxData.buf[4]; //命令
				tx_data[5]=0x02; //数据长度低字节
				tx_data[6]=0x00; //数据长度高字节
				tx_data[7]=0x01; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //校验
				tx_data[15]=0x00; //校验
      			EP2Send(16, tx_data);
 				UpdateBegin=1;
	   }  	
	   else  //收到其它命令一律退出更新
	   {
	   		 TCCR3B = 0x00; //stop    
			 ETIFR|=0x04; //清定时器3中断标志
			 ETIMSK &= 0xfb;//关定时器3中断
	
	     		tx_data[0]=0xe7;  //帧头
				tx_data[1]=0xe7; //帧头
				tx_data[2]=0x00; //地址1
				tx_data[3]=0x00;   //地址2
				tx_data[4]=RxData.buf[4]; //命令
				tx_data[5]=0x02; //数据长度低字节
				tx_data[6]=0x00; //数据长度高字节
				tx_data[7]=RxData.buf[4]; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //校验
				tx_data[15]=0x00; //校验
      			EP2Send(16, tx_data);		
		 		quit();
	   } 
	   RxData.flag=0;  
	 }		 							
	} 
	
	TCCR3B = 0x00; //stop    
	ETIFR|=0x04; //清定时器3中断标志
	ETIMSK &= 0xfb;//关定时器3中断
	
	if(UpdateBegin==0)//3秒内没收到开始更新命令，则直接跳到主程序
	{
	 quit();
	}  
	   
    UpdateBegin=0;
	while(1)
	{
 	  if(RxData.flag==1)//接收到了正常命令帧
   	  {
	 	 if(UpdateBegin>0)//接收主程序二进制数据
     	 { 	  
		      SetTimer3();	 //每收到一字节就重新开始定时500ms，超时后回到待命状态
			  
		      flash_buf[DataReCn]=RxData.buf[4];//命令字节当数据字节
			  DataReCn++;
			  
			   
		 	 if(DataReCn==258)//2字节页索引，256字节页数据
		   	 {
			   TCCR3B = 0x00; //stop  
  			   ETIMSK &= 0xfb;//关定时器3中断
  			   ETIFR|=0x04; //清定时器3中断标志 
  			   
			   
  			   if(UpdateBegin==1)//如果是主控板页写准备了，则此时写入一页
				{
                  address=flash_buf[0]*256+flash_buf[1];  
			      address*=256;	 
				  
			      if(address<0)
			          address=0;
				 
			     if(address>0xff00)//对应页索引为255，这是最后一页能写的页，目前限制固件大小为64K字节
			         address=0xff00;
				
		          write_one_page();	//约20ms    
			    }   
				
			   UpdateBegin=0;//一页写完了，下一页又从页写准备开始
			   
			   crc16(flash_buf,258);//12ms
		        		    
				tx_data[0]=0xe7;  //帧头
				tx_data[1]=0xe7; //帧头
				tx_data[2]=0x00; //地址1
				tx_data[3]=0x00;   //地址2
				tx_data[4]=02; //命令，暂用页写准备这个命令，上位机不关心这个返回字节
				tx_data[5]=0x02; //数据长度低字节
				tx_data[6]=0x00; //数据长度高字节
				tx_data[7]=crc16lo; 
				tx_data[8]=crc16hi; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //校验
				tx_data[15]=0x00; //校验
      			EP2Send(16, tx_data);	 //将这一页的校验码上传给上位机，上位机判断后决定是继续烧写一页还是重烧这一页	 		        
			 }	 	 
	     }
  		 else//若不是数据状态，则进入以下各个命令的子语句
		 {
    		switch(RxData.buf[4]) 
     		{		
			  case 0x00://地面仪版本
				tx_data[0]=0xe7;  //帧头
				tx_data[1]=0xe7; //帧头
				tx_data[2]=0x00; //地址1
				tx_data[3]=0x00;   //地址2
				tx_data[4]=RxData.buf[4]; //命令
				tx_data[5]=0x01; //数据长度低字节
				tx_data[6]=0x00; //数据长度高字节
				tx_data[7]=0x00; 
				tx_data[8]=0x00; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //校验
				tx_data[15]=0x00; //校验
      			EP2Send(16, tx_data);	
				break;
				
			   case 0x01://地面仪握手，进入更新状态  			      			       
	  			tx_data[0]=0xe7;  //帧头
				tx_data[1]=0xe7; //帧头
				tx_data[2]=0x00; //地址1
				tx_data[3]=0x00;   //地址2
				tx_data[4]=RxData.buf[4]; //命令
				tx_data[5]=0x02; //数据长度低字节
				tx_data[6]=0x00; //数据长度高字节
				tx_data[7]=0x01; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //校验
				tx_data[15]=0x00; //校验
      			EP2Send(16, tx_data);
				break;
						 		   			
			case 0x02://地面仪页写准备		
				UpdateBegin=1;//页写准备，从页索引0开始烧写，最大页索引255，共64K字节程序空间
				DataReCn=0;    
				SetTimer3();//开始计时，每500ms内至少要有一个字节收到
				
				tx_data[0]=0xe7;  //帧头
				tx_data[1]=0xe7; //帧头
				tx_data[2]=0x00; //地址1
				tx_data[3]=0x00;   //地址2
				tx_data[4]=RxData.buf[4]; //命令
				tx_data[5]=0x02; //数据长度低字节
				tx_data[6]=0x00; //数据长度高字节
				tx_data[7]=0x02; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //校验
				tx_data[15]=0x00; //校验
      			EP2Send(16, tx_data);			
				break;
											
				
			case 0x03://地面仪退出更新状态，运行正常程序
				tx_data[0]=0xe7;  //帧头
				tx_data[1]=0xe7; //帧头
				tx_data[2]=0x00; //地址1
				tx_data[3]=0x00;   //地址2
				tx_data[4]=RxData.buf[4]; //命令
				tx_data[5]=0x02; //数据长度低字节
				tx_data[6]=0x00; //数据长度高字节
				tx_data[7]=0x03; 
				tx_data[8]=0xe7; 
				tx_data[9]=0x00; 
				tx_data[10]=0x00; 
				tx_data[11]=0x00; 
				tx_data[12]=0x00; 
				tx_data[13]=0x00; 
				tx_data[14]=0x00; //校验
				tx_data[15]=0x00; //校验
      			EP2Send(16, tx_data);	
				 quit();
				break;

				default:
				break;
	  	    }//switch
	     }//UpdateBegin
		 RxData.flag=0;
	 }//RxData.flag
    }//while(1)
      
	 
} 
