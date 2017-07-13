/* CH374芯片 应用层 V1.0 */
/* USB设备,模拟CH372或CH375的TEST程序与计算机通讯 */

#include	"HAL.H"			// 其它单片机需修改HAL*硬件抽象层的几个文件
 
typedef struct RX
{
 unsigned char flag;
 unsigned char len;
 unsigned char buf[64];
};
extern struct RX RxData;

unsigned char EP2SendBusy=0;

// 设备描述符
const	UINT8C	MyDevDescr[] = {0x12, 0x01, 0x10, 0x01,
								0xFF, 0x80, 0x37, 0x08,
								0x15, 0x38, 0x01, 0x28,  // 厂商ID和产品ID
								0x00, 0x01, 0x01, 0x02,
								0x00,  0x01 };
// 配置描述符
const	UINT8C	MyCfgDescr[] = { 0x09, 0x02, 0x27, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
								 0x09, 0x04, 0x00, 0x00, 0x03, 0xFF, 0x80, 0x37, 0x00,
								 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
								 0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
								 0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x00 };
// 语言描述符
const	UINT8C	MyLangDescr[] = { 0x04, 0x03, 0x09, 0x04 };

// 厂家信息
const	UINT8C	MyManuInfo[] = { 0x0E, 0x03, 'G', 0, 'Z', 0, 'T', 0, 'S', 0, 'N', 0, 'Y', 0 };
// 产品信息
//const	UINT8C	MyProdInfo[] = { 0x16, 0x03, 'S', 0, 'H', 0, 'Y', 0, '-', 0, '1', 0 , ' ', 0, 'V', 0 , '1', 0 , '.', 0 , '0', 0 };
const	UINT8C	MyProdInfo[] = { 0x12, 0x03, 'T', 0, 'S', 0, 'N', 0, 'Y', 0, '-', 0 , '1', 0 , '.', 0 , '0', 0  };

void	USB_DeviceInterrupt( void );  // USB设备中断服务程序
void	Init374Device( void );  // 初始化USB设备
void USBStart(void);
unsigned char	UsbConfig = 0,UsbReset = 0;	// USB配置标志


#pragma interrupt_handler int6_isr:iv_INT6
void int6_isr(void)
{
    unsigned int k;
	UINT8	s,l,i,m=0;
	static	UINT8	SetupReq, SetupLen;
	static	PUINT8	pDescr;
	
	s = Read374Byte( REG_INTER_FLAG );  // 获取中断状态

    if(!(s & BIT_IF_INTER_FLAG))  //0x0f
	    return;	    
	
	if ( s & BIT_IF_BUS_RESET )    //0x02
	{  // USB总线复位
	    UsbReset = 1;
		Write374Byte( REG_USB_ADDR, 0x00 );  // 清USB设备地址
		Write374Byte( REG_USB_ENDP0, M_SET_EP0_TRAN_NAK( 0 ) );
		Write374Byte( REG_USB_ENDP1, M_SET_EP1_TRAN_NAK( 0 ) );
		Write374Byte( REG_USB_ENDP2, M_SET_EP2_TRAN_NAK( 0 ) );
		Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_BUS_RESET );  // 清中断标志
	}
	else if ( s & BIT_IF_TRANSFER )  //0x01
	{  // USB传输
		s = Read374Byte( REG_USB_STATUS );  //USB状态, 只读  REG_USB_STATUS=0x0a
		 //printf("BIT_IF_TRANSFER s=%x\r\n",s);
		switch( s & BIT_STAT_PID_ENDP )   //USB传输的事务和端点号：BIT_STAT_PID_ENDP=0x0f
		{  // USB设备中断状态
			case USB_INT_EP2_OUT:  //USB端点2的OUT  USB_INT_EP2_OUT=0x02
			 {  
			   // 批量端点下传成功 
				//if ( s & BIT_STAT_TOG_MATCH ) //0x10;
				{   RxData.flag=0;
					RxData.len = Read374Byte( REG_USB_LENGTH );
					Read374Block( RAM_ENDP2_RECV,RxData.len,RxData.buf );
                    RxData.flag=1;
					//printf("USB_INT_EP2_OUT len=%x\r\n",RxData.len);
		        }
				break;
			}
			case USB_INT_EP2_IN:  //USB_INT_EP2_IN=0x0a, USB端点2的IN 
			{  // 批量端点上传成功,未处理
				Write374Byte( REG_USB_ENDP2, M_SET_EP2_TRAN_NAK( Read374Byte( REG_USB_ENDP2 ) ) ^ BIT_EP2_TRAN_TOG );
				EP2SendBusy=0;
				//printf("USB_INT_EP2_IN\r\n");
				break;
			}
			case USB_INT_EP1_IN:   // 0x09,USB端点1的IN 
			{  // 中断端点上传成功,未处理
				Write374Byte( REG_USB_ENDP1, M_SET_EP1_TRAN_NAK( Read374Byte( REG_USB_ENDP1 ) ) ^ BIT_EP1_TRAN_TOG );
				break;
			}
			case USB_INT_EP0_SETUP:    //USB_INT_EP0_SETUP=0x0C	USB端点0的SETUP 
			{  // 控制传输
				USB_SETUP_REQ	SetupReqBuf;
				
				l = Read374Byte( REG_USB_LENGTH );
				if ( l == sizeof( USB_SETUP_REQ ) ) 
				{
					Read374Block( RAM_ENDP0_RECV, l, (PUINT8)&SetupReqBuf );
					SetupLen = SetupReqBuf.wLengthL;
					if ( SetupReqBuf.wLengthH || SetupLen > 0x7F ) SetupLen = 0x7F;  // 限制总长度
					l = 0;  // 默认为成功并且上传0长度
					
					/*if ( ( SetupReqBuf.bType & DEF_USB_REQ_TYPE ) != DEF_USB_REQ_STAND ) 
					{  // 只支持标准请求 
						l = 0xFF;  // 操作失败
					}
					else */
					{  // 标准请求
						SetupReq = SetupReqBuf.bReq;  // 请求码
						//printf("SetupReq:%x\r\n",SetupReq);
						
						switch( SetupReq ) 
						{   
							case DEF_USB_GET_DESCR:    //USB接收描述；DEF_USB_GET_DESCR=0x06
							    switch( SetupReqBuf.wValueH ) 
								{
									case 1:
										pDescr = (PUINT8)( &MyDevDescr[0] );
										l = sizeof( MyDevDescr );
										break;
									case 2:
										pDescr = (PUINT8)( &MyCfgDescr[0] );
										l = sizeof( MyCfgDescr );
										//l = SetupReqBuf.wLengthL;
										break;
									case 3:
										switch( SetupReqBuf.wValueL ) 
										{
											case 1:
												pDescr = (PUINT8)( &MyManuInfo[0] );
												l = sizeof( MyManuInfo );
												break;
											case 2:
												pDescr = (PUINT8)( &MyProdInfo[0] );
												l = sizeof( MyProdInfo );
												break;
											case 0:
												pDescr = (PUINT8)( &MyLangDescr[0] );
												l = sizeof( MyLangDescr );
												break;
											default:
												l = 0xFF;  // 操作失败
												break;
										}
										break;
									default:
										l = 0xFF;  // 操作失败
										break;
								}
								if ( SetupLen > l ) SetupLen = l;  // 限制总长度
								l = SetupLen >= RAM_ENDP0_SIZE ? RAM_ENDP0_SIZE : SetupLen;  // 本次传输长度
								Write374Block2( RAM_ENDP0_TRAN, l, pDescr );  /* 加载上传数据 */
								SetupLen -= l;
								pDescr += l;
								break;
							case DEF_USB_SET_ADDRESS:  //0x05
								SetupLen = SetupReqBuf.wValueL;  // 暂存USB设备地址
								break;
							case DEF_USB_GET_CONFIG:   //0x08
								Write374Byte( RAM_ENDP0_TRAN, UsbConfig );
								if ( SetupLen >= 1 ) l = 1;
								break;
							case DEF_USB_SET_CONFIG:  //0x09
								UsbConfig = SetupReqBuf.wValueL;
								break;
							case DEF_USB_CLR_FEATURE:  //USB标准设备请求 0x01
								if ( ( SetupReqBuf.bType & 0x1F ) == 0x02 ) 
								{  // 不是端点不支持
									switch( SetupReqBuf.wIndexL ) 
									{
										case 0x82:
											Write374Byte( REG_USB_ENDP2, M_SET_EP2_TRAN_NAK( Read374Byte( REG_USB_ENDP2 ) ) );
											break;
										case 0x02:
											Write374Byte( REG_USB_ENDP2, M_SET_EP2_RECV_ACK( Read374Byte( REG_USB_ENDP2 ) ) );
											break;
										case 0x81:
											Write374Byte( REG_USB_ENDP1, M_SET_EP1_TRAN_NAK( Read374Byte( REG_USB_ENDP1 ) ) );
											break;
										case 0x01:
											Write374Byte( REG_USB_ENDP1, M_SET_EP1_RECV_ACK( Read374Byte( REG_USB_ENDP1 ) ) );
											break;
										default:
											l = 0xFF;  // 操作失败
											break;
									}
								}
								else l = 0xFF;  // 操作失败
								break;
							case DEF_USB_GET_INTERF:
								Write374Byte( RAM_ENDP0_TRAN, 0 );
								if ( SetupLen >= 1 ) l = 1;
								break;
							case DEF_USB_GET_STATUS:
								Write374Byte( RAM_ENDP0_TRAN, 0 );
								Write374Byte( RAM_ENDP0_TRAN + 1, 0 );
								if ( SetupLen >= 2 ) l = 2;
								else l = SetupLen;
								break;
							case 0xfe:
							   	Write374Byte( RAM_ENDP0_TRAN, 0 );
								//Write374Byte( REG_USB_LENGTH, 1 );
					           // Write374Byte( REG_USB_ENDP0, M_SET_EP0_TRAN_ACK( Read374Byte( REG_USB_ENDP0 ), l ) ^ BIT_EP0_TRAN_TOG );
								//printf("GET_MAX_LUN:\r\n");
								break;
							default:
								l = 0xFF;  // 操作失败
								break;
						}
					}
				}
				else l = 0xFF;  // 操作失败
				if ( l == 0xFF ) 
				{  // 操作失败
					Write374Byte( REG_USB_ENDP0, M_SET_EP0_RECV_STA( M_SET_EP0_TRAN_STA( 0 ) ) );  // STALL
				}
				else if ( l <= RAM_ENDP0_SIZE ) 
				{  // 上传数据
					Write374Byte( REG_USB_ENDP0, M_SET_EP0_TRAN_ACK( M_SET_EP0_RECV_ACK( Read374Byte( REG_USB_ENDP0 ) ), l ) | BIT_EP0_TRAN_TOG );  // DATA1
				}
				else 
				{  // 下传数据或其它
					Write374Byte( REG_USB_ENDP0, M_SET_EP0_TRAN_NAK( M_SET_EP0_RECV_ACK( Read374Byte( REG_USB_ENDP0 ) ) ) | BIT_EP0_RECV_TOG );  // DATA1
				}
				//ARD=2;
				break;
			}
			case USB_INT_EP0_IN:   // 0x08  USB端点0的IN
			{
				switch( SetupReq ) 
				{
					case DEF_USB_GET_DESCR:
						l = SetupLen >= RAM_ENDP0_SIZE ? RAM_ENDP0_SIZE : SetupLen;  // 本次传输长度
						Write374Block2( RAM_ENDP0_TRAN, l, pDescr );  /* 加载上传数据 */
						SetupLen -= l;
						pDescr += l;
						Write374Byte( REG_USB_ENDP0, M_SET_EP0_TRAN_ACK( Read374Byte( REG_USB_ENDP0 ), l ) ^ BIT_EP0_TRAN_TOG );
						//printf("DEF_USB_GET_DESCR:\r\n");
						break;
					case DEF_USB_SET_ADDRESS:
						Write374Byte( REG_USB_ADDR, SetupLen );
					default:
						Write374Byte( REG_USB_ENDP0, M_SET_EP0_TRAN_NAK( 0 ) );  // 结束
						break;
				}
				break;
			}
			case USB_INT_EP0_OUT:     //0x00， USB端点0的OUT 
			{
				switch( SetupReq ) 
				{
//					case download:
//						get_data;
//						break;
					case DEF_USB_GET_DESCR:
					 //Write374Byte( RAM_ENDP0_TRAN,00 );
					default:
						Write374Byte( REG_USB_ENDP0, M_SET_EP0_TRAN_NAK( 0 ) );  // 结束
						break;
				}
				break;
			}
			default: 
			{
				break;
			}
		}
		Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_TRANSFER );  // 清中断标志
	}
	else if ( s & BIT_IF_USB_SUSPEND ) 
	{  // USB总线挂起
		Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_USB_SUSPEND );  // 清中断标志
		Write374Byte( REG_SYS_CTRL, Read374Byte( REG_SYS_CTRL ) | BIT_CTRL_OSCIL_OFF );  // 时钟振荡器停止振荡,进入睡眠状态
	}
	else if ( s & BIT_IF_WAKE_UP ) 
	{  // 芯片唤醒完成
		Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_WAKE_UP );  // 清中断标志
	}
	else 
	{  // 意外的中断,不可能发生的情况,除了硬件损坏
		Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_INTER_FLAG );  // 清中断标志
	}
	
}

void	Init374Device( void )  // 初始化USB设备
{     
	Write374Byte( REG_USB_ADDR, 0x00 );
	Write374Byte( REG_USB_ENDP0, M_SET_EP0_TRAN_NAK( 0 ) );
	Write374Byte( REG_USB_ENDP1, M_SET_EP1_TRAN_NAK( 0 ) );
	Write374Byte( REG_USB_ENDP2, M_SET_EP2_TRAN_NAK( 0 ) );
	Write374Byte( REG_INTER_FLAG, BIT_IF_USB_PAUSE | BIT_IF_INTER_FLAG );  // 清所有中断标志
	Write374Byte( REG_INTER_EN, BIT_IE_TRANSFER | BIT_IE_BUS_RESET | BIT_IE_USB_SUSPEND );  // 允许传输完成中断和USB总线复位中断以及USB总线挂起中断,芯片唤醒完成中断
	Write374Byte( REG_SYS_CTRL, BIT_CTRL_OE_POLAR );  // 对于CH374T或者UEN引脚悬空的CH374S必须置BIT_CTRL_OE_POLAR为1
	Write374Byte( REG_USB_SETUP, BIT_SETP_TRANS_EN | BIT_SETP_PULLUP_EN );  // 启动USB设备
}


void USBStart(void)
{
      unsigned int i;
 
 
	  Write374Byte(REG_SYS_CTRL,0x08);	   //CH374软复位
	  for(i=0;i<50000;i++);//18ms
	  Write374Byte(REG_SYS_CTRL,0x00);
	  for(i=0;i<50000;i++);//18ms 
 
	 
	 Init374Device();
	// while(UsbConfig == 0); //等待USB驱动安装成功 
}
