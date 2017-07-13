#include "define.h"

extern uint g_wlValAr[8];		//定义数组保存涡轮脉冲计数值

unsigned char coffBuf[256];

void Delay_mS(uint t)
{
	uint i;
	uchar j;
	for(i=t;i>0;i--)
		for(j=139;j>0;j--);//1mS

}
void Delay_uS(uchar t)  //11+4*t(us)
{
	uchar i;
	for(i=0;i<t;i++)
	{
		Nop();
	}
}
void Tx(uchar ch)			//发送一个字节
{
	while(TXSTAbits.TRMT==0);
	TXREG=ch;
}

uchar Rx(void)				//接收一个字节
{	uchar temp;
	while(!PIR1bits.RCIF);  //等待产生接收中断
	//while(!BAUDCONbits.RCIDL);//等待接收完成
	temp=RCREG;
	return temp;	
}

void Send_vrf(uchar N)		//发送应答包，长度为9或74
{					
	uchar i;
	data74[N]=0;
	Tx(0X55);
	data74[N]+=0X55;
	for(i=1;i<N;i++)
	{
		Tx(data74[i]);
		data74[N]+=data74[i];
	}
	Tx(data74[N]);
}

void Send_IDinfo()		//发送应答包，长度为9
{					
	uchar i,N;
	union Lch2 blockno;
	
	for(i=3;i<9;i++)
		data74[i]=Rx();
	N=data74[3];
	blockno.ch2[0]=data74[4];
	blockno.ch2[1]=data74[5];
	if(N==2)
	{
		Read_DF321ID(blockno.l,data74);	 //
	}
	if(N==3)
	{
		CS0
		WriteAD7799(0X20);//指定ID寄存器
		data74[0]=RecAD7799();
		CS1
	}
	data74[8]=0;
	for(i=0;i<8;i++)
	{
		Tx(data74[i]);
		data74[8]+=data74[i];
	}
	Tx(data74[8]);
}

void Send_davrf(uchar N)		//发送测试数据，长度为64
{					
	uchar i;
	//data74[N]=0;
	//Tx(0X55);
	//data74[N]+=0X55;
	for(i=0;i<N;i++)
	{
		Tx(data74[9+i]);
		//data74[N]+=data74[i];
	}
	//Tx(data74[N]);
}

void Txpack(uint temp_block)
{							//发送74字节应答包
	uchar i;
	uint j;	
	for(i=3;i<9;i++)
		data74[i]=Rx();
	j=data74[3];
	j<<=6;
	Read_DF321page(temp_block,j,64,data74+9);
	//Send_vrf(73);
	Send_davrf(64);
}

void Txblockpack(void)
{							//发送74字节应答包
	uchar i;
	uint j,temp_block;	
	union Lch2 blockno;
	for(i=3;i<9;i++)
		data74[i]=Rx();
	blockno.ch2[0]=data74[4];
	blockno.ch2[1]=data74[5];
	temp_block=blockno.l;        //data74[4]+(data74[5]<<8);
	j=data74[3];
	j<<=6;
	Read_DF321page(temp_block,j,64,data74+9);
	//Send_vrf(73);
	Send_davrf(64);
}

void Rx_infpack(uint temp_block)//接收仪器信息数据包，发送9字节应答包
{
	uchar i;
	uint j;
	for(i=3;i<74;i++)
		data74[i]=Rx();	
	if(data74[3]==0)//擦除仪器信息块、时间表块、校对表块
	{
		data74[73]=Erase_DF321block(temp_block);
		for(i=0;i<9;i++)
		{
			WW2550(Pgain_addr+i,data74[9+i]);
		}
		
	}
	j=data74[3];
   	j<<=6;
	Write_DF321page(temp_block,j,64,data74+9);
	Send_vrf(8);
}

void Rxpack(uint temp_block)//接收数据包，发送8字节应答包
{
	uchar i;
	uint j;
	for(i=3;i<74;i++)
		data74[i]=Rx();
		
	if (temp_block == Caltab_block)
	{
		Read_DF321page(temp_block,0,64,coffBuf+0);
		Read_DF321page(temp_block,64,64,coffBuf+64);
		Read_DF321page(temp_block,128,64,coffBuf+128);
		Read_DF321page(temp_block,192,64,coffBuf+192);	
		
		Erase_DF321block(temp_block);		
	}
			
	if(data74[3]==0)//擦除仪器信息块、时间表块、校对表块
		data74[73]=Erase_DF321block(temp_block);
	j=data74[3];
   	j<<=6;
	if(temp_block==Caltab_block)
	{//因为校对表长度为4096字节，为防止连续写FLASH出错，在写前后加一定延时
		//Delay_mS(5);
		//Write_DF321page(temp_block,j,64,data74+9);
		//Delay_mS(5);
		
		for (i=0; i<64; i++)
		{
			coffBuf[j + i] = data74[9 + i];	
		}
		
		Delay_mS(5);
		Write_DF321page(temp_block,0,64,coffBuf+0);
		Delay_mS(5);
		Write_DF321page(temp_block,64,64,coffBuf+64);
		Delay_mS(5);
		Write_DF321page(temp_block,128,64,coffBuf+128);
		Delay_mS(5);
		Write_DF321page(temp_block,192,64,coffBuf+192);
		Delay_mS(5);		
			
	}
	else
		Write_DF321page(temp_block,j,64,data74+9);
	Send_vrf(8);
}

void Single_point(void)		//直读检测
{	
	uchar i,yy;
	for(i=3;i<9;i++)
		data74[i]=Rx();
	yy=para.info.pointbyte>>1;	
	//PreAD7799_P1();
	PreAD7799_T1();

	switch(para.info.te_mode)
	{
		case 0:
		{
			ReadAD_MODE0();
			data74[9]=data74[0];
			data74[10]=data74[1];
			data74[11]=data74[2];
			data74[12]=data74[3];
			AD_10BIT();
			//ReadAD7799_VBat();  //电池电压
			data74[13]=AD_value.ch2[0];
			data74[14]=AD_value.ch2[1];
			break;
		} 			
		case 1:
		{
			ReadAD_MODE1();
			data74[9]=data74[0];
			data74[10]=data74[1];
			data74[11]=data74[2];
			data74[12]=data74[3];
			data74[13]=data74[4];
			data74[14]=data74[5];
			//ReadAD7799_VBat();  //电池电压
			AD_10BIT();
			data74[15]=AD_value.ch2[0];
			data74[16]=AD_value.ch2[1];
			break;
		} 					
		case 8:
		{
			ReadAD_MODE8();
			data74[9]=data74[0];
			data74[10]=data74[1];
			data74[11]=data74[2];
			data74[12]=data74[3];
			data74[13]=data74[4];
			data74[14]=data74[5];
			data74[15]=data74[6];
			data74[16]=data74[7];
			//ReadAD7799_VBat();  //电池电压
			//AD_10BIT();
			//data74[17]=AD_value.ch2[0];
			//data74[18]=AD_value.ch2[1];
			break;
		} 				
		case 9:
		{
			ReadAD_MODE9();
			data74[9]=data74[0];
			data74[10]=data74[1];
			data74[11]=data74[2];
			data74[12]=data74[3];
			data74[13]=data74[4];
			data74[14]=data74[5];
			data74[15]=data74[6];
			data74[16]=data74[7];
			//ReadAD7799_VBat();  //电池电压
			//AD_10BIT();
			//data74[17]=AD_value.ch2[0];
			//data74[18]=AD_value.ch2[1];
			break;
		} 		
		default:
		{
			ReadAD_MODE0();
			data74[9]=data74[0];
			data74[10]=data74[1];
			data74[11]=data74[2];
			data74[12]=data74[3];
			//ReadAD7799_VBat();  //电池电压
			AD_10BIT();
			data74[13]=AD_value.ch2[0];
			data74[14]=AD_value.ch2[1];
			break;
		} 		
	}			
	//Send_vrf(73);
	data74[72]=para.info.te_mode;
	Send_davrf(64);
}


void SingleTest(void)		//直读检测
{	
	uchar i,yy;
	
	//PreAD7799_T1();

	ReadAD_MODE8();
	
	//ReadAD7799_VBat();  //电池电压
	//AD_10BIT();
	//data74[17]=AD_value.ch2[0];
	//data74[18]=AD_value.ch2[1];
}

void Get_para(void)			//获取240字节的测试信息
{	
	uchar i,j,k,yy,temp,big_index,E2550addr;
	union Lch2 End_block,big_block;
	union Lch3 temppoint;
	uchar Etabaddr;
	uint  Vbatpoint,bn;
	Delay_mS(10);
	i=R2550(Well1_addr);//此代码行可有可无
	for(i=0;i<240;i++)
	{
		temp=Well1_addr+i;
		para.data[i]=R2550(temp);
	}
	bn=BLOCKNUM;
//	if(para.info.mem_num<1 || para.info.mem_num>2 )//20170521
		para.info.mem_num=1;
	para.info.mem_num-=1;
	if(para.info.mem_num>0)
		bn=BLOCKNUM+1024;

	para.info.te_mode=8;//20170521
	switch(para.info.te_mode)
	{
		case 0:{yy=2;break;} 			
		case 1:{yy=3;break;} 				
		case 2:{yy=3;break;} 				
		case 8:{yy=4;break;} 
		case 9:{yy=4;break;}	
		default:{yy=2;break;} 	
	}	
	if(para.info.count==0)//若还没有经过计算
	{		
//		//首先得到循环单元中最大的末块及对应单元号
		big_block.ch2[0]=R2550(Loop_addr);
		big_block.ch2[1]=R2550(Loop_addr+1);
		big_index=0;
		for(i=1;i<8;i++)
		{
			temp=Loop_addr+(i<<1);
			End_block.ch2[0]=R2550(temp);
			temp++;
			End_block.ch2[1]=R2550(temp);
			if(End_block.l>big_block.l)
			{
				big_block.l=End_block.l;
				big_index=i;
			}
		}
		
		if(para.info.loop_point>7)
		{//若循环指针存错，令循环指针指向最大末块，正确取值区间[0,7]
			para.info.loop_point=big_index;
			WW2550(Looppoint_addr,para.info.loop_point);
		}
		else
		{
			i=Loop_addr+((para.info.loop_point)<<1);//得到循环指针指向的循环单元中的末块
			End_block.ch2[0]=R2550(i);
			i++;
			End_block.ch2[1]=R2550(i);
			if(big_block.l>End_block.l)//两者应该相等
			{//若不相等，找到正确的末块及所在循环单元，并使循环指针指向该单元
				for(valid_block.l=End_block.l;valid_block.l<bn;(valid_block.l)++)
				{
					Etabaddr=(valid_block.l>>3);
					Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
					i=data74[9];
					i>>=(valid_block.l&0X07);
					i&=0X01;
					if(i==0)//若是好块
					{
						Read_DF321page(valid_block.l,0,2,AD_value.ch2);
						if(AD_value.ch2[1]==0XFF)	//若是新块	
						{
							para.info.loop_point=big_index;
							temp=Loop_addr+((para.info.loop_point)<<1);
							WW2550(temp,valid_block.ch2[0]);
							temp++;
							WW2550(temp,valid_block.ch2[1]);
							WW2550(Looppoint_addr,para.info.loop_point);
							break;	//遇到好的新块则跳出
						}
					}	
				}
			}
		}		

		i=(para.info.well-1)*7;   //6;----20130425
		valid_block.ch2[0]=para.data[i];//得到末口井的首块
		i++;
		valid_block.ch2[1]=para.data[i];
		i=Loop_addr+((para.info.loop_point)<<1);//得到末口井的末块，末块>=首块
		End_block.ch2[0]=R2550(i);
		i++;
		End_block.ch2[1]=R2550(i);
		//for循环：由首块、末块及勘误表得出末口井点数及总点数
		if(End_block.l<valid_block.l)//若出现了异常，末块小于首块
			temppoint.l=0X7A120;//对该口井点数赋最大值500000点，以便读出几乎半个FLASH
		else
		{
			temppoint.l=0;
			for(valid_excursion=valid_block.l;valid_excursion<End_block.l;valid_excursion++)
			{
				Etabaddr=(valid_excursion>>3);
				Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
				i=data74[9];
				i>>=(valid_excursion&0X07);
				i&=0X01;
				if(i==0)//若是好块
					temppoint.l+=2048;//2048个2字节
			}
			for(valid_excursion=0;valid_excursion<4096;valid_excursion+=2)
			{
				Read_DF321page(End_block.l,valid_excursion,2,AD_value.ch2);
				if(AD_value.l<0XFFFF)
				//{
					temppoint.l++;//2字节数+1
				//}
				else
				{
					if(valid_excursion==4094)	
						break;
					Read_DF321page(End_block.l,valid_excursion+2,2,AD_value.ch2);
					if(AD_value.l==0XFFFF)	
						break;
					else
						temppoint.l++;//2字节数+1
				}
			}
			if(temppoint.l>(256+yy))
			{	
				(temppoint.l)-=(256+yy);
				(temppoint.l)/=yy;
			}
			else
				temppoint.l=0;
		}
		

		if(temppoint.l)//若点数有效
		{
			(para.info.points.l)+=(temppoint.l);
			i=(para.info.well-1)*7+4;//更新对应数组中的末口井点数及总点数
			for(k=0;k<3;k++)
			{			
				j=i+k;
				para.data[j]=temppoint.ch3[k];
				j=Well1_addr+i+k;
				temp=temppoint.ch3[k];
				WW2550(j,temp);
				j=Points_addr+k;
				temp=para.info.points.ch3[k];
				WW2550(j,temp);
			}
			Vbatpoint=(temppoint.l&0X7FF);
			if(Vbatpoint>0)
			{
				Vbatpoint=((Vbatpoint-1)<<1);
				Read_DF321page(Vbattab_block,Vbatpoint,2,AD_value.ch2);
				i=(para.info.well-1)*7;
				temp=AD_value.ch2[0];
				WW2550(Vbat1_addr+i,temp);
				temp=AD_value.ch2[1];
				WW2550(Vbat1_addr+i+1,temp); 
			}
		}
		else//若点数无效，即为0，则上口井作废 20100810新加 
		{
			valid_block.ch2[0]=para.data[(para.info.well-1)*7+0];//上口井的首块
			valid_block.ch2[1]=para.data[(para.info.well-1)*7+1];
			valid_excursion=valid_block.l;
			//Etabaddr=(valid_excursion>>3);
			//Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
			//temp=data74[9];
			Delay_mS(10);			
			i=Erase_DF321block(valid_excursion);
			Delay_mS(10);			
			para.info.well--;//井数自减1，并存储
			WW2550(Well_addr,para.info.well);
		}
		para.info.count=1;//置计算标志为1
		WW2550(Count_addr,0X01);
	}	
}


void Send_data(void)			//发送数据
{
	uint temp,bn;
	uchar Etabaddr;
	union Lch2 packnum;
	for(temp=3;temp<9;temp++)
		data74[temp]=Rx();

	packnum.ch2[0]=data74[4];	 //包数
	packnum.ch2[1]=data74[5];
	valid_block.ch2[0]=data74[6];//首块
	valid_block.ch2[1]=data74[7];

	Get_para();//获取测试信息
	bn=BLOCKNUM;
	if(para.info.mem_num>0)
		bn=BLOCKNUM+1024;

	while(valid_block.l<bn&&packnum.l>0)
	{
		Etabaddr=(valid_block.l>>3);
		Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
		temp=data74[9];
		temp>>=(valid_block.l&0X07);
		temp&=0X01;
 		if(temp==0)//若是好块
		{	
			for(temp=0;temp<64&&packnum.l>0;temp++)
			{
				valid_excursion=temp;
				valid_excursion<<=6;
				Read_DF321page(valid_block.l,valid_excursion,64,data74+9);
				//Send_vrf(73);
				Send_davrf(64);
				packnum.l--;
				if(packnum.l==0)
					break;
			}
		}
		valid_block.l++;
	}
}

void Send_dataNew(void)			//发送数据
{
	uint temp,bn;
	uchar Etabaddr;
	union Lch2 packnum;
	for(temp=3;temp<9;temp++)
		data74[temp]=Rx();

	packnum.ch2[0]=data74[4];	 //包数
	packnum.ch2[1]=data74[5];
	valid_block.ch2[0]=data74[6];//首块
	valid_block.ch2[1]=data74[7];
	
	Get_para();//获取测试信息
	bn=BLOCKNUM;
	if(para.info.mem_num>0)
		bn=BLOCKNUM+1024;

	//while(valid_block.l<bn&&packnum.l>0)
	if ( (valid_block.l<bn) && (packnum.l<64) )
	{
		Etabaddr=(valid_block.l>>3);
		Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
		temp=data74[9];
		temp>>=(valid_block.l&0X07);
		temp&=0X01;
 		if(temp==0)//若是好块
		{	
			//for(temp=0;temp<64&&packnum.l>0;temp++)
			//{
				valid_excursion=packnum.l;	//temp
				valid_excursion<<=6;
				Read_DF321page(valid_block.l,valid_excursion,64,data74+9);
				//Send_vrf(73);
				Send_davrf(64);
			//	packnum.l--;
			//	if(packnum.l==0)
			//		break;
			//}
		}
		valid_block.l++;
	}
	else
	{
		while ( (valid_block.l<bn) && (packnum.l>=64) )
		{
			//切换首块地址
			Etabaddr=(valid_block.l>>3);
			Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
			temp=data74[9];
			temp>>=(valid_block.l&0X07);
			temp&=0X01;
	 		if(temp==0)//若是好块
	 		{
		 		packnum.l -= 64;
		 	}
		 	valid_block.l++;		
		}
		
		while ( (valid_block.l<bn) && (packnum.l<64) )
		{
			Etabaddr=(valid_block.l>>3);
			Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
			temp=data74[9];
			temp>>=(valid_block.l&0X07);
			temp&=0X01;
	 		if(temp==0)//若是好块
			{	
				//for(temp=0;temp<64&&packnum.l>0;temp++)
				//{
					valid_excursion=packnum.l;	//temp
					valid_excursion<<=6;
					Read_DF321page(valid_block.l,valid_excursion,64,data74+9);
					//Send_vrf(73);
					Send_davrf(64);
				//	packnum.l--;
				//	if(packnum.l==0)
				//		break;
				//}
				
				break;
			}
			valid_block.l++;
		} 	
	}	
}

void Delete_data(void)			//格式化存储器，或删除数据Erase
{//不删除：仪器信息块，校对表块，时间表块
	uchar i,err,num,Etabaddr;
	uint j,bn,errnum;
	union Lch2 blockaddr;
	union Lch2 blo_num;
	//num=para.info.mem_num;//
	bn=BLOCKNUM;
	if(para.info.mem_num==1)
	{
		bn=BLOCKNUM+1024;
	}
	blockaddr.ch2[0]=data74[4];
	blockaddr.ch2[1]=data74[5];//首块
	blo_num.ch2[0]=data74[6];
	blo_num.ch2[1]=data74[7];//块数
	err=0;errnum=0;
	if(blockaddr.l==0X0000 && blo_num.l==0XFFFF)
	{//擦除整个FLASH，BLOCKNUM块
		data74[3]=Erase_DF321block(0);
		
		for(j=5;j<bn;j++)
		{	
			i=Erase_DF321block(j);//i代表擦除后的反馈信息：0X00好；0X20坏
			err>>=1;
			if(i)
			{
				err|=0X80;
				errnum++;
			}
			if((j&0X07)==0X07)
			{//满8块写入1个字节
				Etabaddr=(j>>3);
				data74[9]=err;
				Write_DF321page(0,Etabaddr,1,data74+9);
				err=0;
			}
		}
		blo_num.l=errnum; //bn;
		j=(bn>>3);
		data74[9]=blo_num.ch2[0];
		data74[10]=blo_num.ch2[1];
		Write_DF321page(0,j,2,data74+9);
		//WW2550(Errnum_addr,blo_num.ch2[0]);
		//WW2550(Errnum_addr+1,blo_num.ch2[1]);
	}
	else
	{
		i=Loop_addr+((para.info.loop_point)<<1);//得到当前块
		blo_num.ch2[0]=R2550(i);
		i++;
		blo_num.ch2[1]=R2550(i);
										
		bn=blo_num.l+1;
		for(j=5;j<bn;j++)
		{
		
			i=Erase_DF321block(j);	
										
		}						
	}
	Delay_mS(10);

	for(i=0;i<246;i++)
	{//清零信息:循环位置、井首块、点数、井数、总点数、循环指针,数据满标志
		WW2550(Loop_addr+i,0X00);
	}
	WW2550(Count_addr,0X01);//置计算标志为1
}
void Send_testinfo(void)	//发送256字节的测试信息
{
	uchar i,temp;
	for(i=3;i<9;i++)
		data74[i]=Rx();
	//temp=data74[3];			
	temp=data74[3]<<6;  //比较优化
	//temp<<=6;
	for(i=0;i<64;i++)
	{
		data74[i+9]=R2550(temp+i);
	}		
	Send_davrf(64);
}

void Rx_Testinfo()//接收仪器测试信息数据包，发送9字节应答包
{
	uchar i;
	uint j;
	for(i=3;i<74;i++)
		data74[i]=Rx();	
	j=data74[3];
	j<<=6;
	for(i=0;i<64;i++)
	{
		WW2550(j+i,data74[9+i]);
	}
	
	Send_vrf(8);
}
/*****************************************
              模拟SPI操作FLASH
*******************************************/

void SpiDF321_OutByte(uchar b)  // SPI输出1个8位数据
{  
	uchar i,k;
	for(i=0;i<8;i++) 
	{		
		SCK=0;
		if(b&0x80) 
			FSO=1;
		else 
			FSO=0; 	
	    SCK=1;     			  // 在时钟上升沿采样输
		b<<=1;    			  // 数据位是高位在前 
	}
}

uchar SpiDF321_InByte(void)   // SPI输入8个位数据  
{   
	uchar i,b,k;
	b=0;
	for (i=0;i<8;i++) 
	{
	    SCK=0;     			 // DF321在时钟下降沿输出 
		b<<=1;     			 // 数据位是高位在前  
		if(FSI) 
		  b++;
		SCK=1;
	}
	return b;
}

void Write_EN(uint block)			 //写使能
{
    F1CS0 //SPI片选有效		
	SpiDF321_OutByte( 0x06);
	F1CS1
  				//SPI停止
}

uchar Read_status(uint block)
{	
	uchar b=0;
    F1CS0	  //SPI片选有效		
    SpiDF321_OutByte(0x05);
	b=SpiDF321_InByte();
	F1CS1	  				//SPI停止

	return b;
}

void Write_Flash_unpro(uint block)
{
	uchar temp;
    temp=Read_status(block);   //读FLASH状态寄存器
	if(temp&0X80)
	{	Write_EN(block);	 	  //写使能
	    F1CS0 //SPI片选有效
	   	
		SpiDF321_OutByte(0x01);
		SpiDF321_OutByte(0x00);
		F1CS1	         //SPI停止
	
	}
	Write_EN(block);	 		 //写使能
	F1CS0  //SPI片选有效
   
	SpiDF321_OutByte(0x01);
	SpiDF321_OutByte(0x00);
	F1CS1	  			//SPI停止

//	Delay_mS(10);
}

void Read_DF321page(uint block,uint excursion,uint mLen, uchar *mBuf )  
{	// 从指定起始地址读出一定字节的数据  
	union Lch3 Flashaddr;
	
	F1CS0	
	Flashaddr.l=block;
	Flashaddr.l<<=ROTATEBIT;
	Flashaddr.l+=excursion;
    	
	SpiDF321_OutByte(0x03);
	SpiDF321_OutByte(Flashaddr.ch3[2]);
      SpiDF321_OutByte(Flashaddr.ch3[1]);
	SpiDF321_OutByte(Flashaddr.ch3[0]);
	while(mLen--) 
	{	*mBuf=SpiDF321_InByte();
		mBuf++;
	}
	F1CS1		   //SPI停止
}

void Read_DF321ID(uint block, uchar *mBuf )  
{	// 从指定起始地址读出一定字节的数据 
	uchar i;
	i=4; 
	
	F1CS0	

	SpiDF321_OutByte(0x9F);

	while(i--) 
	{	*mBuf=SpiDF321_InByte();
		mBuf++;
	}
	F1CS1		   //SPI停止
}	 
	 
void Write_DF321page(uint block,uint excursion,uint mLen,uchar *mBuf)  
{	// 向指定起始地址写入一定字节的数据
    uchar b;
    union Lch3 Flashaddr;
	Write_EN(block);	//写使能
    F1CS0
	Flashaddr.l=block;	

    Flashaddr.l<<=ROTATEBIT;
    Flashaddr.l+=excursion;   
    				 
   SpiDF321_OutByte(0x02);
   SpiDF321_OutByte(Flashaddr.ch3[2]);
   SpiDF321_OutByte(Flashaddr.ch3[1]);
   SpiDF321_OutByte(Flashaddr.ch3[0]);
   while(mLen--) 
   {
		SpiDF321_OutByte(*mBuf);
  		for(b=12;b>0;b--);//120uS	//连续写字节之间稍加延时
		mBuf++;
	}
   F1CS1		     				//SPI停止
 
   for(b=36;b>0;b--);			//连续写一定字节后，延时360uS
 //  b=Read_status();
}  

uchar Erase_DF321block(uint BlockAddr)  //BlockAddr要擦除的块
{	//擦除命令 0x20擦4k	0x52擦32k 0xD8擦64k  0xC7擦整片
    uchar b=0;
    uint temp_addr,tempnum;
	Delay_mS(10);						//擦除前延时10mS
	tempnum=BlockAddr;
	Write_EN(BlockAddr);

	F1CS0
	temp_addr=(tempnum%16)<<12;       //块具体地址
		  
	SpiDF321_OutByte(0x20);	   
	SpiDF321_OutByte(tempnum>>4);
	SpiDF321_OutByte(temp_addr>>8);
	SpiDF321_OutByte(temp_addr);
	F1CS1			    //SPI停止

	Delay_mS(10);		//擦除命令发出后延时10mS
	//等待块擦除完毕方法：1、查询；2、延时等待
	do
	{
		b=Read_status(BlockAddr);//读FLASH状态寄存器
		b&=0X01;
	}while(b);	
	b=Read_status(BlockAddr);	//擦除后读状态寄存器，以判断是否是坏块
	b&=0x20;			//若b=0X20则为坏块，b=0x00则为好块
	return b;
}

/************************************
PIC18LF2550内部EEPROM操作
*************************************/
uchar R2550(uchar temp_addr)
{	uchar temp;
	EEADR=temp_addr;
	EECON1=0X01;
	temp=EEDATA;
	return(temp);
}

void WW2550(uchar temp_addr,uchar data)
{
	W2550(temp_addr,data);
	if(EECON1bits.WRERR)
		W2550(temp_addr,data);
}
	
void W2550(uchar temp_addr,uchar data)
{
	EEADR=temp_addr;
	EEDATA=data;
	EECON1=0X04;
	EECON2=0X55;
	EECON2=0XAA;
	EECON1bits.WR=1;
	while(EECON1bits.WR);
	EECON1bits.WREN=0;
}

/***************************************
//         模拟SPI操作ADS7799
***************************************/
uchar RecAD7799(void) //259.5US--20140803
{//读取AD7799一个字节
	uchar i,j,b=0,k;
	for(i=0;i<8;i++)
	{
		SCK=0;
		for(k=8;k>0;k--);	
		SCK=1;
		for(k=8;k>0;k--);//30uS
		b=b<<1;
		if(FSI)
			b|=0X01;
	}
	return b;
}
	
void WriteAD7799(uchar b)//传命令字57US--20140803
{//写入AD7799一个字节
	uchar i,k;
	for(i=0;i<8;i++)
	{
		if(((b<<i)&0X80))
			FSO=1;
		else
			FSO=0;
		SCK=0;
		for(k=8;k>0;k--);//30uS
		SCK=1;
		for(k=8;k>0;k--);	
	}
	Nop();Nop();Nop();Nop();Nop();
}

void resetAD7799(void)//------14.94ms-----20130928
{//连续发至少32个高电平，使AD7799复位
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	WriteAD7799(0XFF);
	WriteAD7799(0XFF);
	WriteAD7799(0XFF);
	WriteAD7799(0XFF);
    WriteAD7799(0XFF);
	WriteAD7799(0XFF);
	CS1
	Delay_mS(22);  //
}

void PreAD7799_P1(void)//通道3 ----14.59ms-----20160828
{	
	//TX=1;
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	WriteAD7799(0X08);//指定模式寄存器
	WriteAD7799(0X00);//单次转换模式0X20
	WriteAD7799(0X01); //转换频率500Hz,T_settle=4mS

	WriteAD7799(0X10);//指定配置寄存器
	WriteAD7799(Pgain);//有极性，增益不定
	WriteAD7799(0X12);//缓冲，通道3 	
	Delay_mS(10);// 留出1mS的余量

	WriteAD7799(0X58);//读AD转换结果
	value.ch4[2]=RecAD7799();
	value.ch4[1]=RecAD7799();
	value.ch4[0]=RecAD7799();
	Nop();Nop();
	CS1
	//TX=0;
}

void PreAD7799_T1(void)//通道2 ----14.59ms-----20130928
{	
	//TX=1;
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	WriteAD7799(0X10);//指定配置寄存器
	WriteAD7799(0X11);//0X10无极性，增益1//0X11无极性，增益2
	WriteAD7799(0X01);//无缓冲，通道2 	

	WriteAD7799(0X08);//指定模式寄存器
	WriteAD7799(0X00);//单次转换模式0X20
	WriteAD7799(0X01); //转换频率500Hz,T_settle=4mS	
	Delay_mS(10);// 留出1mS的余量
	WriteAD7799(0X58);//读AD转换结果
	value.ch4[2]=RecAD7799();
	value.ch4[1]=RecAD7799();
	value.ch4[0]=RecAD7799();
	Nop();Nop();
	CS1
	//TX=0;
}

void ReadAD7799_16P1(void)//通道3 ----110ms-----20160912
{	
	//RX=1;
	WriteAD7799(0X08);//指定模式寄存器
	WriteAD7799(0X00);//单次转换模式0X20
	WriteAD7799(0X08); //0X07转换频率33.3Hz,T_settle=60mS//0X08转换频率19.6Hz,T_settle=101mS	
						//0X0A转换频率16.7Hz,T_settle=120mS
	WriteAD7799(0X10);//指定配置寄存器
	WriteAD7799(Pgain);//有极性，增益不定
	//WriteAD7799(0x03);//有极性，增益不定
	WriteAD7799(0X12);//缓冲，通道3 
	
	Delay_mS(105); //(0X0A)留出3mS的余量3ms-----20160912
	Delay_mS(105); 

	WriteAD7799(0X58);//读AD转换结果
	value.l=0;	
	value.ch4[2]=RecAD7799();
	value.ch4[1]=RecAD7799();
	value.ch4[0]=RecAD7799();

	//若实际值正好为0，则传送的高、中两字节为0X1388=5000（十进制）
	value.l=(value.l)<<1;
	value.ch4[3]=0;
	value.l=(value.l)>>8;
	value.l+=para.info.adjzero.l;  //5000

}

void ReadAD7799_16T1(void)  //通道2 ----67ms-----20160912
{
	WriteAD7799(0X08);//指定模式寄存器
	WriteAD7799(0X00);//单次转换模式0X20
	WriteAD7799(0X07);//0X07转换频率33.3Hz,T_settle=60mS
	//0X08转换频率19.6Hz,T_settle=101mS//0X0A转换频率16.7Hz,T_settle=120mS
	WriteAD7799(0X10);//指定配置寄存器
	WriteAD7799(0X10);//0X10无极性，增益1//0X11无极性，增益2
	WriteAD7799(0X01);//无缓冲，通道2 
		
	Delay_mS(124);//留出10mS的余量	
	
	WriteAD7799(0X58);//读AD转换结果
	value.l=0;	
	value.ch4[2]=RecAD7799();
	value.ch4[1]=RecAD7799();
	value.ch4[0]=RecAD7799();

}

void ReadAD7799_16F1(void)  //测F1的值，通道2 ----70.4ms-----20160916
{	
	WriteAD7799(0X08);//指定模式寄存器
	WriteAD7799(0X00);//单次转换模式0X20
	WriteAD7799(0X07);//0X07转换频率33.3Hz,T_settle=60mS
	//0X08转换频率19.6Hz,T_settle=101mS//0X0A转换频率16.7Hz,T_settle=120mS
	//TEST=1;
	WriteAD7799(0X10);//指定配置寄存器
	WriteAD7799(0X10);//无极性，增益1
	WriteAD7799(0X00);//无缓冲，通道1 
		
	Delay_mS(128);//留出4mS的余量	
	
	WriteAD7799(0X58);//读AD转换结果
	//TEST=0;	
	AD_value.ch2[1]=RecAD7799();
	AD_value.ch2[0]=RecAD7799();
	RecAD7799();
	
	//RX=0;
}

void ReadAD7799_16F2(void)  //测F2的值，通道2 ----70.4ms-----20160916
{	
	//RX=1;
	WriteAD7799(0X08);//指定模式寄存器
	WriteAD7799(0X00);//单次转换模式0X20
	WriteAD7799(0X07);//0X07转换频率33.3Hz,T_settle=60mS
	//0X08转换频率19.6Hz,T_settle=101mS//0X0A转换频率16.7Hz,T_settle=120mS
	WriteAD7799(0X10);//指定配置寄存器
	WriteAD7799(0X10);//无极性，增益1
	WriteAD7799(0X00);//无缓冲，通道1 
		
	Delay_mS(128);//留出4mS的余量	
	
	WriteAD7799(0X58);//读AD转换结果
	
	AD_value.ch2[1]=RecAD7799();
	AD_value.ch2[0]=RecAD7799();
	RecAD7799();
	//RX=0;
}

void AD_10BIT(void)			//4ms,内部单片机AD单次采集AN0
{
	//uint b=0;
	ADCON1=0X0E;			//RA0为单片机内部AD，其余IO复用口全部设为IO口
	ADCON0=0X00;			//选择A/D 输入通道AN0
	ADCON2=0XA1;            //数据左对齐21，选择A/D 采集时间8TAD,选择A/D 转换时钟001 = FOSC/8,101 = FOSC/16
	ADCON0bits.ADON=1;		//使能AD转换模块
	Delay_mS(8);            //2
	//if(TX_8M)
	//Delay_mS(4);
	ADCON0bits.GO_DONE=1;   //启动AD转换
	Nop();Nop();Nop();
	while(ADCON0bits.GO_DONE==1);
	//Delay_mS(1);
	//AD_value.ch2[0]=ADRESL;
	//AD_value.ch2[1]=ADRESH;
	//b=ADRESH;
	AD_value.ch2[0]=ADRESL;
	AD_value.ch2[1]=ADRESH;
	//return b;
}

void ReadAD_MODE0(void)  //---220ms---20140514
{
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	
	ReadAD7799_16T1();
	data74[2]=value.ch4[1];
	data74[3]=value.ch4[2];
	
	ReadAD7799_16P1();
	data74[0]=value.ch4[0];
	data74[1]=value.ch4[1];	
	Nop();Nop();
	CS1						
}

void ReadAD_MODE1(void)  //---362ms---20140514
{
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	ReadAD7799_16T1();
	data74[2]=value.ch4[1];
	data74[3]=value.ch4[2];
	
	ReadAD7799_16P1();
	data74[0]=value.ch4[0];
	data74[1]=value.ch4[1];
	Nop();Nop();
	CS1					
}

void ReadAD_MODE8(void)  //---484ms---20140514
{
	unsigned long  u32Temp1;
	unsigned char i;
	
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	//INTCON=0XC0;	     //睡眠前使能中断
	//T1CON=0X4E;
	Delay_mS(20);
	//data74[50]=0;
	//PIE1bits.TMR1IE=1;   //允许Timer1中断	
	

//-------------------采集温度--20160915

	ReadAD7799_16T1();
	data74[2]=value.ch4[1];
	data74[3]=value.ch4[2];

//-------------------采集压力--20160915	
	
	Delay_mS(30);
	ReadAD7799_16P1();
	data74[0]=value.ch4[0];
	data74[1]=value.ch4[1];	
//--------------------------------------------

		
//-------------------采集流量-
	u32Temp1 = 0;
	for (i=0; i<16; i++)
	{
		u32Temp1 += g_wlValAr[i];	
		//u32Temp1 += 32;
	}
	u32Temp1 /= 2;
		
	data74[4]=u32Temp1;
	data74[5]=(u32Temp1>>8);
	data74[6]=(u32Temp1>>16);
	data74[7]=(u32Temp1>>24);
	
	//Delay_mS(200);
	Nop();Nop();
	CS1					
}


void ReadAD_MODE9(void)  //---302ms---20140514
{
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	//INTCON=0XC0;	     //睡眠前使能中断
	//T1CON=0X4E;
	Delay_mS(20);
	//data74[50]=0;
	//PIE1bits.TMR1IE=1;   //允许Timer1中断	
	
	EXC=1;
	//TEST=1;
	Delay_mS(250);
	//TEST=0;	
					
	EXC=0;	
	Delay_mS(250);
//-------------------采集温度--20160915
	EXC=1;	
	Delay_mS(116);
	ReadAD7799_16T1();
	data74[2]=value.ch4[1];
	data74[3]=value.ch4[2];

//-------------------采集压力--20160915	
	EXC=0;	
	Delay_mS(30);
	ReadAD7799_16P1();
	data74[0]=value.ch4[0];
	data74[1]=value.ch4[1];	
//--------------------------------------------
	EXC=1;				
	Delay_mS(250);
		
//-------------------采集流量--20160915
	EXC=0;	
	Delay_mS(106);
	//TEST=1;
	ReadAD7799_16F1();
	//TEST=0;
	data74[4]=AD_value.ch2[0];
	data74[5]=AD_value.ch2[1];
	Delay_mS(6);
	
	EXC=1;	
	Delay_mS(106);
	ReadAD7799_16F2();
	data74[6]=AD_value.ch2[0];
	data74[7]=AD_value.ch2[1];
	Delay_mS(6);
	EXC=0;	
	
	//Delay_mS(200);
	Nop();Nop();
	CS1			
}

void Sample_save(void)      //-----54mS----20160915
{//采样、存储、更新循环指针和循环单元
	uchar temp,nn,yy,ts;
	uint bn,yj=0;	
	uchar Etabaddr;
	union Lch2 temp_value,time_value;
	//Delay_mS(100);				//打开AVDD后延时
	bn=BLOCKNUM;
	time_value.l=AD_value.l;
	if(para.info.mem_num>0)
	{
		bn=BLOCKNUM<<1;
	}
	ts=0;
	//resetAD7799();
	//AD_10BIT();
	//ReadAD7799_VBat();  //电池电压
	temp_value.l=AD_value.l;
	//PreAD7799_P1();  //PreAD7799_T1();
	//yy=para.info.pointbyte>>1;//RX=0;
	switch(para.info.te_mode)
	{
		case 0:{yy=2;break;}  //Sample_save()---423ms---20140514			
		case 1:{yy=3;break;}  //Sample_save()---586ms---20140514				
		case 8:{yy=4;break;}  //Sample_save()---526ms---20140514				
		case 9:{yy=4;break;}  //Sample_save()---708ms---20140514	
		default:{yy=2;break;}  //Sample_save()---423ms---20140514	
	}
	temp=Read_status(Vbattab_block); 	//读FLASH状态寄存器	
	if(temp&0X0C)
	{
		Write_Flash_unpro(Vbattab_block);//防止第一次取消片保护失败
		Delay_mS(2);
	}
	temp=Read_status(Vbattab_block); 	//读FLASH状态寄存器	
	if(temp&0X0C)
	{
		Write_Flash_unpro(Vbattab_block);//防止第一次取消片保护失败
		Delay_mS(2);
	}
//	if(time_value.l>1)
//	{	
//		Write_Flash_unpro(Vbattab_block);	//FLASH取消片保护
//		Delay_mS(2);	
//		temp=Read_status(Vbattab_block); 	//读FLASH状态寄存器
//		if(temp&0X0C)
//			Write_Flash_unpro(Vbattab_block);//防止第一次取消片保护失败
//		Delay_mS(2);
//	}
	//for(temp=0;temp<3;temp++)
	//{
	 	//AD_value.l=time_value.l;
		AD_value.l=temp_value.l;
		Write_DF321page(Vbattab_block,Vbat_excursion,2,AD_value.ch2);
		Delay_mS(18);		//读FLASH之前延时
		Read_DF321page(Vbattab_block,Vbat_excursion,2,temp_value.ch2);
		if(AD_value.l!=temp_value.l)   
		{
			Write_DF321page(Vbattab_block,Vbat_excursion,2,AD_value.ch2);
			Delay_mS(18);
			//WW2550(CSzero_addr,AD_value.ch2[0]);
			//WW2550(CSzero_addr+1,AD_value.ch2[1]);
			
  		}               
	//}
	Vbat_excursion+=2;
	yj=Vbat_excursion&BLOCKYU; //除以4096的余数
	if(yj==0)					//
	{		 					//并更新循环指针，循环单元中的内容，并采集电池电压
		temp=Vbat1_addr+(para.info.well*7);    //6);//-----20130425
		WW2550(temp,AD_value.ch2[0]);	 	  //采存电池电压
		temp++;
		WW2550(temp,AD_value.ch2[1]);	
		Erase_DF321block(Vbattab_block);	 
		Vbat_excursion=0;      //偏移量重新从0开始	
	}
	for(nn=0;nn<yy;nn++)
	{			
		yj=valid_excursion&BLOCKYU; //除以4096的余数
		if(yj==0)					//即刚进入一个新块，需要判断是否是坏块
		{		 					//并更新循环指针，循环单元中的内容，并采集电池电压
			do
			{	
				valid_block.l++;
				if(valid_block.l==bn)
					Sleep_forever();//超出有效存储块范围
				Etabaddr=(valid_block.l>>3);
				Read_DF321page(0,Etabaddr,1,data74+9);//读勘误表
				temp=data74[9];
				temp>>=(valid_block.l&0X07);
				temp&=0X01;
			}while(temp);	  //遇到好块，DO-WHILE循环结束
			valid_excursion=0;//偏移量重新从0开始
			if(para.info.loop_point>6)
				para.info.loop_point=0;
			else
				para.info.loop_point++;
			temp=Loop_addr+((para.info.loop_point)<<1);
			WW2550(temp,valid_block.ch2[0]);
			temp++;
			WW2550(temp,valid_block.ch2[1]);
			WW2550(Looppoint_addr,para.info.loop_point);
		}
		
		if((ts==0)&&(valid_block.l>=BLOCKNUM))
		{			
			temp=Read_status(valid_block.l); 	//读FLASH状态寄存器
	 		if(temp&0X0C)
				Write_Flash_unpro(valid_block.l);//防止第一次取消片保护失败
			Delay_mS(2);
			temp=Read_status(valid_block.l); 	//读FLASH状态寄存器
	 		if(temp&0X0C)
				Write_Flash_unpro(valid_block.l);//防止第一次取消片保护失败
			ts=1;
			Delay_mS(2);
		}  
		AD_value.ch2[0]=data74[0+nn+nn];
		AD_value.ch2[1]=data74[1+nn+nn];
		//for(temp=0;temp<5;temp++)
		//{
			//Delay_mS(15);		//写FLASH之前延时
			Write_DF321page(valid_block.l,valid_excursion,2,AD_value.ch2);//PT_value.ch4);	
			Delay_mS(18);		//读FLASH之前延时
			Read_DF321page(valid_block.l,valid_excursion,2,temp_value.ch2);//value.ch4);
			if(AD_value.l!=temp_value.l)   
			{
				Write_DF321page(valid_block.l,valid_excursion,2,AD_value.ch2);//PT_value.ch4);	
				Delay_mS(18);
				//WW2550(Well_addr-7+nn+nn,AD_value.ch2[0]);
				//WW2550(Well_addr-6+nn+nn,AD_value.ch2[1]);
				//if(nn==1)
				//{
				//   INTCON=0X00;		//禁止所有中断
				//   RX=1;
				//   T1CON=0X70;	
				//   while(1);
				//}	
 			}     
		//}
		valid_excursion+=2;
	} 
	
}

void Sleep_forever(void)
{
	INTCON=0X00;			//禁止所有中断
	T1CON=0X70;				//停止32768振荡,关定时器1
	Close_avdd();			//睡眠前关AVDD
	Sleep();				//进入永久睡眠	
}
