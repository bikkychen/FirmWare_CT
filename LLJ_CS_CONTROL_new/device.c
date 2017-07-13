#include "define.h"

extern uint g_wlValAr[8];		//�������鱣�������������ֵ

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
void Tx(uchar ch)			//����һ���ֽ�
{
	while(TXSTAbits.TRMT==0);
	TXREG=ch;
}

uchar Rx(void)				//����һ���ֽ�
{	uchar temp;
	while(!PIR1bits.RCIF);  //�ȴ����������ж�
	//while(!BAUDCONbits.RCIDL);//�ȴ��������
	temp=RCREG;
	return temp;	
}

void Send_vrf(uchar N)		//����Ӧ���������Ϊ9��74
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

void Send_IDinfo()		//����Ӧ���������Ϊ9
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
		WriteAD7799(0X20);//ָ��ID�Ĵ���
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

void Send_davrf(uchar N)		//���Ͳ������ݣ�����Ϊ64
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
{							//����74�ֽ�Ӧ���
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
{							//����74�ֽ�Ӧ���
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

void Rx_infpack(uint temp_block)//����������Ϣ���ݰ�������9�ֽ�Ӧ���
{
	uchar i;
	uint j;
	for(i=3;i<74;i++)
		data74[i]=Rx();	
	if(data74[3]==0)//����������Ϣ�顢ʱ���顢У�Ա��
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

void Rxpack(uint temp_block)//�������ݰ�������8�ֽ�Ӧ���
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
			
	if(data74[3]==0)//����������Ϣ�顢ʱ���顢У�Ա��
		data74[73]=Erase_DF321block(temp_block);
	j=data74[3];
   	j<<=6;
	if(temp_block==Caltab_block)
	{//��ΪУ�Ա���Ϊ4096�ֽڣ�Ϊ��ֹ����дFLASH������дǰ���һ����ʱ
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

void Single_point(void)		//ֱ�����
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
			//ReadAD7799_VBat();  //��ص�ѹ
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
			//ReadAD7799_VBat();  //��ص�ѹ
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
			//ReadAD7799_VBat();  //��ص�ѹ
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
			//ReadAD7799_VBat();  //��ص�ѹ
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
			//ReadAD7799_VBat();  //��ص�ѹ
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


void SingleTest(void)		//ֱ�����
{	
	uchar i,yy;
	
	//PreAD7799_T1();

	ReadAD_MODE8();
	
	//ReadAD7799_VBat();  //��ص�ѹ
	//AD_10BIT();
	//data74[17]=AD_value.ch2[0];
	//data74[18]=AD_value.ch2[1];
}

void Get_para(void)			//��ȡ240�ֽڵĲ�����Ϣ
{	
	uchar i,j,k,yy,temp,big_index,E2550addr;
	union Lch2 End_block,big_block;
	union Lch3 temppoint;
	uchar Etabaddr;
	uint  Vbatpoint,bn;
	Delay_mS(10);
	i=R2550(Well1_addr);//�˴����п��п���
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
	if(para.info.count==0)//����û�о�������
	{		
//		//���ȵõ�ѭ����Ԫ������ĩ�鼰��Ӧ��Ԫ��
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
		{//��ѭ��ָ������ѭ��ָ��ָ�����ĩ�飬��ȷȡֵ����[0,7]
			para.info.loop_point=big_index;
			WW2550(Looppoint_addr,para.info.loop_point);
		}
		else
		{
			i=Loop_addr+((para.info.loop_point)<<1);//�õ�ѭ��ָ��ָ���ѭ����Ԫ�е�ĩ��
			End_block.ch2[0]=R2550(i);
			i++;
			End_block.ch2[1]=R2550(i);
			if(big_block.l>End_block.l)//����Ӧ�����
			{//������ȣ��ҵ���ȷ��ĩ�鼰����ѭ����Ԫ����ʹѭ��ָ��ָ��õ�Ԫ
				for(valid_block.l=End_block.l;valid_block.l<bn;(valid_block.l)++)
				{
					Etabaddr=(valid_block.l>>3);
					Read_DF321page(0,Etabaddr,1,data74+9);//�������
					i=data74[9];
					i>>=(valid_block.l&0X07);
					i&=0X01;
					if(i==0)//���Ǻÿ�
					{
						Read_DF321page(valid_block.l,0,2,AD_value.ch2);
						if(AD_value.ch2[1]==0XFF)	//�����¿�	
						{
							para.info.loop_point=big_index;
							temp=Loop_addr+((para.info.loop_point)<<1);
							WW2550(temp,valid_block.ch2[0]);
							temp++;
							WW2550(temp,valid_block.ch2[1]);
							WW2550(Looppoint_addr,para.info.loop_point);
							break;	//�����õ��¿�������
						}
					}	
				}
			}
		}		

		i=(para.info.well-1)*7;   //6;----20130425
		valid_block.ch2[0]=para.data[i];//�õ�ĩ�ھ����׿�
		i++;
		valid_block.ch2[1]=para.data[i];
		i=Loop_addr+((para.info.loop_point)<<1);//�õ�ĩ�ھ���ĩ�飬ĩ��>=�׿�
		End_block.ch2[0]=R2550(i);
		i++;
		End_block.ch2[1]=R2550(i);
		//forѭ�������׿顢ĩ�鼰�����ó�ĩ�ھ��������ܵ���
		if(End_block.l<valid_block.l)//���������쳣��ĩ��С���׿�
			temppoint.l=0X7A120;//�Ըÿھ����������ֵ500000�㣬�Ա�����������FLASH
		else
		{
			temppoint.l=0;
			for(valid_excursion=valid_block.l;valid_excursion<End_block.l;valid_excursion++)
			{
				Etabaddr=(valid_excursion>>3);
				Read_DF321page(0,Etabaddr,1,data74+9);//�������
				i=data74[9];
				i>>=(valid_excursion&0X07);
				i&=0X01;
				if(i==0)//���Ǻÿ�
					temppoint.l+=2048;//2048��2�ֽ�
			}
			for(valid_excursion=0;valid_excursion<4096;valid_excursion+=2)
			{
				Read_DF321page(End_block.l,valid_excursion,2,AD_value.ch2);
				if(AD_value.l<0XFFFF)
				//{
					temppoint.l++;//2�ֽ���+1
				//}
				else
				{
					if(valid_excursion==4094)	
						break;
					Read_DF321page(End_block.l,valid_excursion+2,2,AD_value.ch2);
					if(AD_value.l==0XFFFF)	
						break;
					else
						temppoint.l++;//2�ֽ���+1
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
		

		if(temppoint.l)//��������Ч
		{
			(para.info.points.l)+=(temppoint.l);
			i=(para.info.well-1)*7+4;//���¶�Ӧ�����е�ĩ�ھ��������ܵ���
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
		else//��������Ч����Ϊ0�����Ͽھ����� 20100810�¼� 
		{
			valid_block.ch2[0]=para.data[(para.info.well-1)*7+0];//�Ͽھ����׿�
			valid_block.ch2[1]=para.data[(para.info.well-1)*7+1];
			valid_excursion=valid_block.l;
			//Etabaddr=(valid_excursion>>3);
			//Read_DF321page(0,Etabaddr,1,data74+9);//�������
			//temp=data74[9];
			Delay_mS(10);			
			i=Erase_DF321block(valid_excursion);
			Delay_mS(10);			
			para.info.well--;//�����Լ�1�����洢
			WW2550(Well_addr,para.info.well);
		}
		para.info.count=1;//�ü����־Ϊ1
		WW2550(Count_addr,0X01);
	}	
}


void Send_data(void)			//��������
{
	uint temp,bn;
	uchar Etabaddr;
	union Lch2 packnum;
	for(temp=3;temp<9;temp++)
		data74[temp]=Rx();

	packnum.ch2[0]=data74[4];	 //����
	packnum.ch2[1]=data74[5];
	valid_block.ch2[0]=data74[6];//�׿�
	valid_block.ch2[1]=data74[7];

	Get_para();//��ȡ������Ϣ
	bn=BLOCKNUM;
	if(para.info.mem_num>0)
		bn=BLOCKNUM+1024;

	while(valid_block.l<bn&&packnum.l>0)
	{
		Etabaddr=(valid_block.l>>3);
		Read_DF321page(0,Etabaddr,1,data74+9);//�������
		temp=data74[9];
		temp>>=(valid_block.l&0X07);
		temp&=0X01;
 		if(temp==0)//���Ǻÿ�
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

void Send_dataNew(void)			//��������
{
	uint temp,bn;
	uchar Etabaddr;
	union Lch2 packnum;
	for(temp=3;temp<9;temp++)
		data74[temp]=Rx();

	packnum.ch2[0]=data74[4];	 //����
	packnum.ch2[1]=data74[5];
	valid_block.ch2[0]=data74[6];//�׿�
	valid_block.ch2[1]=data74[7];
	
	Get_para();//��ȡ������Ϣ
	bn=BLOCKNUM;
	if(para.info.mem_num>0)
		bn=BLOCKNUM+1024;

	//while(valid_block.l<bn&&packnum.l>0)
	if ( (valid_block.l<bn) && (packnum.l<64) )
	{
		Etabaddr=(valid_block.l>>3);
		Read_DF321page(0,Etabaddr,1,data74+9);//�������
		temp=data74[9];
		temp>>=(valid_block.l&0X07);
		temp&=0X01;
 		if(temp==0)//���Ǻÿ�
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
			//�л��׿��ַ
			Etabaddr=(valid_block.l>>3);
			Read_DF321page(0,Etabaddr,1,data74+9);//�������
			temp=data74[9];
			temp>>=(valid_block.l&0X07);
			temp&=0X01;
	 		if(temp==0)//���Ǻÿ�
	 		{
		 		packnum.l -= 64;
		 	}
		 	valid_block.l++;		
		}
		
		while ( (valid_block.l<bn) && (packnum.l<64) )
		{
			Etabaddr=(valid_block.l>>3);
			Read_DF321page(0,Etabaddr,1,data74+9);//�������
			temp=data74[9];
			temp>>=(valid_block.l&0X07);
			temp&=0X01;
	 		if(temp==0)//���Ǻÿ�
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

void Delete_data(void)			//��ʽ���洢������ɾ������Erase
{//��ɾ����������Ϣ�飬У�Ա�飬ʱ����
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
	blockaddr.ch2[1]=data74[5];//�׿�
	blo_num.ch2[0]=data74[6];
	blo_num.ch2[1]=data74[7];//����
	err=0;errnum=0;
	if(blockaddr.l==0X0000 && blo_num.l==0XFFFF)
	{//��������FLASH��BLOCKNUM��
		data74[3]=Erase_DF321block(0);
		
		for(j=5;j<bn;j++)
		{	
			i=Erase_DF321block(j);//i���������ķ�����Ϣ��0X00�ã�0X20��
			err>>=1;
			if(i)
			{
				err|=0X80;
				errnum++;
			}
			if((j&0X07)==0X07)
			{//��8��д��1���ֽ�
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
		i=Loop_addr+((para.info.loop_point)<<1);//�õ���ǰ��
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
	{//������Ϣ:ѭ��λ�á����׿顢�������������ܵ�����ѭ��ָ��,��������־
		WW2550(Loop_addr+i,0X00);
	}
	WW2550(Count_addr,0X01);//�ü����־Ϊ1
}
void Send_testinfo(void)	//����256�ֽڵĲ�����Ϣ
{
	uchar i,temp;
	for(i=3;i<9;i++)
		data74[i]=Rx();
	//temp=data74[3];			
	temp=data74[3]<<6;  //�Ƚ��Ż�
	//temp<<=6;
	for(i=0;i<64;i++)
	{
		data74[i+9]=R2550(temp+i);
	}		
	Send_davrf(64);
}

void Rx_Testinfo()//��������������Ϣ���ݰ�������9�ֽ�Ӧ���
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
              ģ��SPI����FLASH
*******************************************/

void SpiDF321_OutByte(uchar b)  // SPI���1��8λ����
{  
	uchar i,k;
	for(i=0;i<8;i++) 
	{		
		SCK=0;
		if(b&0x80) 
			FSO=1;
		else 
			FSO=0; 	
	    SCK=1;     			  // ��ʱ�������ز�����
		b<<=1;    			  // ����λ�Ǹ�λ��ǰ 
	}
}

uchar SpiDF321_InByte(void)   // SPI����8��λ����  
{   
	uchar i,b,k;
	b=0;
	for (i=0;i<8;i++) 
	{
	    SCK=0;     			 // DF321��ʱ���½������ 
		b<<=1;     			 // ����λ�Ǹ�λ��ǰ  
		if(FSI) 
		  b++;
		SCK=1;
	}
	return b;
}

void Write_EN(uint block)			 //дʹ��
{
    F1CS0 //SPIƬѡ��Ч		
	SpiDF321_OutByte( 0x06);
	F1CS1
  				//SPIֹͣ
}

uchar Read_status(uint block)
{	
	uchar b=0;
    F1CS0	  //SPIƬѡ��Ч		
    SpiDF321_OutByte(0x05);
	b=SpiDF321_InByte();
	F1CS1	  				//SPIֹͣ

	return b;
}

void Write_Flash_unpro(uint block)
{
	uchar temp;
    temp=Read_status(block);   //��FLASH״̬�Ĵ���
	if(temp&0X80)
	{	Write_EN(block);	 	  //дʹ��
	    F1CS0 //SPIƬѡ��Ч
	   	
		SpiDF321_OutByte(0x01);
		SpiDF321_OutByte(0x00);
		F1CS1	         //SPIֹͣ
	
	}
	Write_EN(block);	 		 //дʹ��
	F1CS0  //SPIƬѡ��Ч
   
	SpiDF321_OutByte(0x01);
	SpiDF321_OutByte(0x00);
	F1CS1	  			//SPIֹͣ

//	Delay_mS(10);
}

void Read_DF321page(uint block,uint excursion,uint mLen, uchar *mBuf )  
{	// ��ָ����ʼ��ַ����һ���ֽڵ�����  
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
	F1CS1		   //SPIֹͣ
}

void Read_DF321ID(uint block, uchar *mBuf )  
{	// ��ָ����ʼ��ַ����һ���ֽڵ����� 
	uchar i;
	i=4; 
	
	F1CS0	

	SpiDF321_OutByte(0x9F);

	while(i--) 
	{	*mBuf=SpiDF321_InByte();
		mBuf++;
	}
	F1CS1		   //SPIֹͣ
}	 
	 
void Write_DF321page(uint block,uint excursion,uint mLen,uchar *mBuf)  
{	// ��ָ����ʼ��ַд��һ���ֽڵ�����
    uchar b;
    union Lch3 Flashaddr;
	Write_EN(block);	//дʹ��
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
  		for(b=12;b>0;b--);//120uS	//����д�ֽ�֮���Լ���ʱ
		mBuf++;
	}
   F1CS1		     				//SPIֹͣ
 
   for(b=36;b>0;b--);			//����дһ���ֽں���ʱ360uS
 //  b=Read_status();
}  

uchar Erase_DF321block(uint BlockAddr)  //BlockAddrҪ�����Ŀ�
{	//�������� 0x20��4k	0x52��32k 0xD8��64k  0xC7����Ƭ
    uchar b=0;
    uint temp_addr,tempnum;
	Delay_mS(10);						//����ǰ��ʱ10mS
	tempnum=BlockAddr;
	Write_EN(BlockAddr);

	F1CS0
	temp_addr=(tempnum%16)<<12;       //������ַ
		  
	SpiDF321_OutByte(0x20);	   
	SpiDF321_OutByte(tempnum>>4);
	SpiDF321_OutByte(temp_addr>>8);
	SpiDF321_OutByte(temp_addr);
	F1CS1			    //SPIֹͣ

	Delay_mS(10);		//�������������ʱ10mS
	//�ȴ��������Ϸ�����1����ѯ��2����ʱ�ȴ�
	do
	{
		b=Read_status(BlockAddr);//��FLASH״̬�Ĵ���
		b&=0X01;
	}while(b);	
	b=Read_status(BlockAddr);	//�������״̬�Ĵ��������ж��Ƿ��ǻ���
	b&=0x20;			//��b=0X20��Ϊ���飬b=0x00��Ϊ�ÿ�
	return b;
}

/************************************
PIC18LF2550�ڲ�EEPROM����
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
//         ģ��SPI����ADS7799
***************************************/
uchar RecAD7799(void) //259.5US--20140803
{//��ȡAD7799һ���ֽ�
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
	
void WriteAD7799(uchar b)//��������57US--20140803
{//д��AD7799һ���ֽ�
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
{//����������32���ߵ�ƽ��ʹAD7799��λ
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

void PreAD7799_P1(void)//ͨ��3 ----14.59ms-----20160828
{	
	//TX=1;
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	WriteAD7799(0X08);//ָ��ģʽ�Ĵ���
	WriteAD7799(0X00);//����ת��ģʽ0X20
	WriteAD7799(0X01); //ת��Ƶ��500Hz,T_settle=4mS

	WriteAD7799(0X10);//ָ�����üĴ���
	WriteAD7799(Pgain);//�м��ԣ����治��
	WriteAD7799(0X12);//���壬ͨ��3 	
	Delay_mS(10);// ����1mS������

	WriteAD7799(0X58);//��ADת�����
	value.ch4[2]=RecAD7799();
	value.ch4[1]=RecAD7799();
	value.ch4[0]=RecAD7799();
	Nop();Nop();
	CS1
	//TX=0;
}

void PreAD7799_T1(void)//ͨ��2 ----14.59ms-----20130928
{	
	//TX=1;
	CS0
	Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	WriteAD7799(0X10);//ָ�����üĴ���
	WriteAD7799(0X11);//0X10�޼��ԣ�����1//0X11�޼��ԣ�����2
	WriteAD7799(0X01);//�޻��壬ͨ��2 	

	WriteAD7799(0X08);//ָ��ģʽ�Ĵ���
	WriteAD7799(0X00);//����ת��ģʽ0X20
	WriteAD7799(0X01); //ת��Ƶ��500Hz,T_settle=4mS	
	Delay_mS(10);// ����1mS������
	WriteAD7799(0X58);//��ADת�����
	value.ch4[2]=RecAD7799();
	value.ch4[1]=RecAD7799();
	value.ch4[0]=RecAD7799();
	Nop();Nop();
	CS1
	//TX=0;
}

void ReadAD7799_16P1(void)//ͨ��3 ----110ms-----20160912
{	
	//RX=1;
	WriteAD7799(0X08);//ָ��ģʽ�Ĵ���
	WriteAD7799(0X00);//����ת��ģʽ0X20
	WriteAD7799(0X08); //0X07ת��Ƶ��33.3Hz,T_settle=60mS//0X08ת��Ƶ��19.6Hz,T_settle=101mS	
						//0X0Aת��Ƶ��16.7Hz,T_settle=120mS
	WriteAD7799(0X10);//ָ�����üĴ���
	WriteAD7799(Pgain);//�м��ԣ����治��
	//WriteAD7799(0x03);//�м��ԣ����治��
	WriteAD7799(0X12);//���壬ͨ��3 
	
	Delay_mS(105); //(0X0A)����3mS������3ms-----20160912
	Delay_mS(105); 

	WriteAD7799(0X58);//��ADת�����
	value.l=0;	
	value.ch4[2]=RecAD7799();
	value.ch4[1]=RecAD7799();
	value.ch4[0]=RecAD7799();

	//��ʵ��ֵ����Ϊ0�����͵ĸߡ������ֽ�Ϊ0X1388=5000��ʮ���ƣ�
	value.l=(value.l)<<1;
	value.ch4[3]=0;
	value.l=(value.l)>>8;
	value.l+=para.info.adjzero.l;  //5000

}

void ReadAD7799_16T1(void)  //ͨ��2 ----67ms-----20160912
{
	WriteAD7799(0X08);//ָ��ģʽ�Ĵ���
	WriteAD7799(0X00);//����ת��ģʽ0X20
	WriteAD7799(0X07);//0X07ת��Ƶ��33.3Hz,T_settle=60mS
	//0X08ת��Ƶ��19.6Hz,T_settle=101mS//0X0Aת��Ƶ��16.7Hz,T_settle=120mS
	WriteAD7799(0X10);//ָ�����üĴ���
	WriteAD7799(0X10);//0X10�޼��ԣ�����1//0X11�޼��ԣ�����2
	WriteAD7799(0X01);//�޻��壬ͨ��2 
		
	Delay_mS(124);//����10mS������	
	
	WriteAD7799(0X58);//��ADת�����
	value.l=0;	
	value.ch4[2]=RecAD7799();
	value.ch4[1]=RecAD7799();
	value.ch4[0]=RecAD7799();

}

void ReadAD7799_16F1(void)  //��F1��ֵ��ͨ��2 ----70.4ms-----20160916
{	
	WriteAD7799(0X08);//ָ��ģʽ�Ĵ���
	WriteAD7799(0X00);//����ת��ģʽ0X20
	WriteAD7799(0X07);//0X07ת��Ƶ��33.3Hz,T_settle=60mS
	//0X08ת��Ƶ��19.6Hz,T_settle=101mS//0X0Aת��Ƶ��16.7Hz,T_settle=120mS
	//TEST=1;
	WriteAD7799(0X10);//ָ�����üĴ���
	WriteAD7799(0X10);//�޼��ԣ�����1
	WriteAD7799(0X00);//�޻��壬ͨ��1 
		
	Delay_mS(128);//����4mS������	
	
	WriteAD7799(0X58);//��ADת�����
	//TEST=0;	
	AD_value.ch2[1]=RecAD7799();
	AD_value.ch2[0]=RecAD7799();
	RecAD7799();
	
	//RX=0;
}

void ReadAD7799_16F2(void)  //��F2��ֵ��ͨ��2 ----70.4ms-----20160916
{	
	//RX=1;
	WriteAD7799(0X08);//ָ��ģʽ�Ĵ���
	WriteAD7799(0X00);//����ת��ģʽ0X20
	WriteAD7799(0X07);//0X07ת��Ƶ��33.3Hz,T_settle=60mS
	//0X08ת��Ƶ��19.6Hz,T_settle=101mS//0X0Aת��Ƶ��16.7Hz,T_settle=120mS
	WriteAD7799(0X10);//ָ�����üĴ���
	WriteAD7799(0X10);//�޼��ԣ�����1
	WriteAD7799(0X00);//�޻��壬ͨ��1 
		
	Delay_mS(128);//����4mS������	
	
	WriteAD7799(0X58);//��ADת�����
	
	AD_value.ch2[1]=RecAD7799();
	AD_value.ch2[0]=RecAD7799();
	RecAD7799();
	//RX=0;
}

void AD_10BIT(void)			//4ms,�ڲ���Ƭ��AD���βɼ�AN0
{
	//uint b=0;
	ADCON1=0X0E;			//RA0Ϊ��Ƭ���ڲ�AD������IO���ÿ�ȫ����ΪIO��
	ADCON0=0X00;			//ѡ��A/D ����ͨ��AN0
	ADCON2=0XA1;            //���������21��ѡ��A/D �ɼ�ʱ��8TAD,ѡ��A/D ת��ʱ��001 = FOSC/8,101 = FOSC/16
	ADCON0bits.ADON=1;		//ʹ��ADת��ģ��
	Delay_mS(8);            //2
	//if(TX_8M)
	//Delay_mS(4);
	ADCON0bits.GO_DONE=1;   //����ADת��
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
	//INTCON=0XC0;	     //˯��ǰʹ���ж�
	//T1CON=0X4E;
	Delay_mS(20);
	//data74[50]=0;
	//PIE1bits.TMR1IE=1;   //����Timer1�ж�	
	

//-------------------�ɼ��¶�--20160915

	ReadAD7799_16T1();
	data74[2]=value.ch4[1];
	data74[3]=value.ch4[2];

//-------------------�ɼ�ѹ��--20160915	
	
	Delay_mS(30);
	ReadAD7799_16P1();
	data74[0]=value.ch4[0];
	data74[1]=value.ch4[1];	
//--------------------------------------------

		
//-------------------�ɼ�����-
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
	//INTCON=0XC0;	     //˯��ǰʹ���ж�
	//T1CON=0X4E;
	Delay_mS(20);
	//data74[50]=0;
	//PIE1bits.TMR1IE=1;   //����Timer1�ж�	
	
	EXC=1;
	//TEST=1;
	Delay_mS(250);
	//TEST=0;	
					
	EXC=0;	
	Delay_mS(250);
//-------------------�ɼ��¶�--20160915
	EXC=1;	
	Delay_mS(116);
	ReadAD7799_16T1();
	data74[2]=value.ch4[1];
	data74[3]=value.ch4[2];

//-------------------�ɼ�ѹ��--20160915	
	EXC=0;	
	Delay_mS(30);
	ReadAD7799_16P1();
	data74[0]=value.ch4[0];
	data74[1]=value.ch4[1];	
//--------------------------------------------
	EXC=1;				
	Delay_mS(250);
		
//-------------------�ɼ�����--20160915
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
{//�������洢������ѭ��ָ���ѭ����Ԫ
	uchar temp,nn,yy,ts;
	uint bn,yj=0;	
	uchar Etabaddr;
	union Lch2 temp_value,time_value;
	//Delay_mS(100);				//��AVDD����ʱ
	bn=BLOCKNUM;
	time_value.l=AD_value.l;
	if(para.info.mem_num>0)
	{
		bn=BLOCKNUM<<1;
	}
	ts=0;
	//resetAD7799();
	//AD_10BIT();
	//ReadAD7799_VBat();  //��ص�ѹ
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
	temp=Read_status(Vbattab_block); 	//��FLASH״̬�Ĵ���	
	if(temp&0X0C)
	{
		Write_Flash_unpro(Vbattab_block);//��ֹ��һ��ȡ��Ƭ����ʧ��
		Delay_mS(2);
	}
	temp=Read_status(Vbattab_block); 	//��FLASH״̬�Ĵ���	
	if(temp&0X0C)
	{
		Write_Flash_unpro(Vbattab_block);//��ֹ��һ��ȡ��Ƭ����ʧ��
		Delay_mS(2);
	}
//	if(time_value.l>1)
//	{	
//		Write_Flash_unpro(Vbattab_block);	//FLASHȡ��Ƭ����
//		Delay_mS(2);	
//		temp=Read_status(Vbattab_block); 	//��FLASH״̬�Ĵ���
//		if(temp&0X0C)
//			Write_Flash_unpro(Vbattab_block);//��ֹ��һ��ȡ��Ƭ����ʧ��
//		Delay_mS(2);
//	}
	//for(temp=0;temp<3;temp++)
	//{
	 	//AD_value.l=time_value.l;
		AD_value.l=temp_value.l;
		Write_DF321page(Vbattab_block,Vbat_excursion,2,AD_value.ch2);
		Delay_mS(18);		//��FLASH֮ǰ��ʱ
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
	yj=Vbat_excursion&BLOCKYU; //����4096������
	if(yj==0)					//
	{		 					//������ѭ��ָ�룬ѭ����Ԫ�е����ݣ����ɼ���ص�ѹ
		temp=Vbat1_addr+(para.info.well*7);    //6);//-----20130425
		WW2550(temp,AD_value.ch2[0]);	 	  //�ɴ��ص�ѹ
		temp++;
		WW2550(temp,AD_value.ch2[1]);	
		Erase_DF321block(Vbattab_block);	 
		Vbat_excursion=0;      //ƫ�������´�0��ʼ	
	}
	for(nn=0;nn<yy;nn++)
	{			
		yj=valid_excursion&BLOCKYU; //����4096������
		if(yj==0)					//���ս���һ���¿飬��Ҫ�ж��Ƿ��ǻ���
		{		 					//������ѭ��ָ�룬ѭ����Ԫ�е����ݣ����ɼ���ص�ѹ
			do
			{	
				valid_block.l++;
				if(valid_block.l==bn)
					Sleep_forever();//������Ч�洢�鷶Χ
				Etabaddr=(valid_block.l>>3);
				Read_DF321page(0,Etabaddr,1,data74+9);//�������
				temp=data74[9];
				temp>>=(valid_block.l&0X07);
				temp&=0X01;
			}while(temp);	  //�����ÿ飬DO-WHILEѭ������
			valid_excursion=0;//ƫ�������´�0��ʼ
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
			temp=Read_status(valid_block.l); 	//��FLASH״̬�Ĵ���
	 		if(temp&0X0C)
				Write_Flash_unpro(valid_block.l);//��ֹ��һ��ȡ��Ƭ����ʧ��
			Delay_mS(2);
			temp=Read_status(valid_block.l); 	//��FLASH״̬�Ĵ���
	 		if(temp&0X0C)
				Write_Flash_unpro(valid_block.l);//��ֹ��һ��ȡ��Ƭ����ʧ��
			ts=1;
			Delay_mS(2);
		}  
		AD_value.ch2[0]=data74[0+nn+nn];
		AD_value.ch2[1]=data74[1+nn+nn];
		//for(temp=0;temp<5;temp++)
		//{
			//Delay_mS(15);		//дFLASH֮ǰ��ʱ
			Write_DF321page(valid_block.l,valid_excursion,2,AD_value.ch2);//PT_value.ch4);	
			Delay_mS(18);		//��FLASH֮ǰ��ʱ
			Read_DF321page(valid_block.l,valid_excursion,2,temp_value.ch2);//value.ch4);
			if(AD_value.l!=temp_value.l)   
			{
				Write_DF321page(valid_block.l,valid_excursion,2,AD_value.ch2);//PT_value.ch4);	
				Delay_mS(18);
				//WW2550(Well_addr-7+nn+nn,AD_value.ch2[0]);
				//WW2550(Well_addr-6+nn+nn,AD_value.ch2[1]);
				//if(nn==1)
				//{
				//   INTCON=0X00;		//��ֹ�����ж�
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
	INTCON=0X00;			//��ֹ�����ж�
	T1CON=0X70;				//ֹͣ32768��,�ض�ʱ��1
	Close_avdd();			//˯��ǰ��AVDD
	Sleep();				//��������˯��	
}
