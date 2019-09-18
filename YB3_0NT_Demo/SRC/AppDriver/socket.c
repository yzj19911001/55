/************************************************************************************************************/
/*** \文件名    socket.c																  *******************/
/*** \作者      李玉株																  ***********************/
/*** \时间      2013年9月5日														  ***********************/
/*** \文件说明  1、部分原代码来源于WIZnet官网 1.2.3版本									  ***********************/
/***   			2、本文件实现了W5300的基本的socket端口访问功能，同时实现了各写一下的数据通信功能等***********/
/************************************************************************************************************/
#include "socket.h"
#include "JZStructure.h"
__align(2) uint8   Rx_Buffer[W5300BUFSIZE];
__align(2) uint8   Tx_Buffer[W5300BUFSIZE];

/** 默认端口号（递加）*/
uint16   W5300_source_port = 4000;

/*** 首包发送标志位*/
uint8    check_sendok_flag[MAX_SOCK_NUM];

/*******************************************************************************
* 函数名  : W5300_Socket_Init
* 描述    : W5300d端口初始化函数
* 输入    : socket s ，protocol 工作方式，port 端口 ，flag        
* 输出    : 无
* 返回值  : 无
*******************************************************************************/ 
void W5300_Socket_Init(SOCKET s, uint8 protocol, uint16 port, uint16 flag)
{
	W5300_WRITE_REG(Sn_MR(s),(uint16)(protocol | flag | Sn_MR_ND )); // 设置相应的端口为指定通信协议 & flag, | Sn_MR_ALIGN
    
    W5300_WRITE_REG(Sn_MSSR(s),0x05B4); //MTU设置为1460个字节
	  W5300_WRITE_REG(Sn_KPALVTR(s),0x02); //设置保持活跃时间10s 
	
	if (port != 0) 
		W5300_WRITE_REG(Sn_PORTR(s),port);
	else 
	{
		W5300_WRITE_REG(Sn_PORTR(s),W5300_source_port);	
		W5300_source_port++;     // 若没有设置端口则设置为默认端口号.		 
	}
	W5300_WRITE_REG(Sn_IMR(s),0x001F);//socket 中断使能	(禁止发送完成中断20130918）
	setSn_CR(s, Sn_CR_OPEN);      // 打开端口 
	check_sendok_flag[s] = 1;     // 初始化发送标志
}
/*******************************************************************************
* 函数名  : W5300_Socket_Close
* 描述    : 关闭W5300端口
* 输入    : 端口号s        
* 输出    : 无
* 返回值  : 无
*******************************************************************************/ 
void W5300_Socket_Close(SOCKET s)
{
	uint16 loop_cnt =0;
	uint8 destip[4];
	if( ((getSn_MR(s)& 0x0F) == Sn_MR_TCP) && (getSn_TX_FSR(s) != getW5300_TxMAX(s)) ) 
	{ 
		while(getSn_TX_FSR(s) != getW5300_TxMAX(s))
		{
			if(loop_cnt++ > 10)
			{		
				// 修改目标IP地址
				destip[0] = 0;destip[1] = 0;destip[2] = 0;destip[3] = 1;
				W5300_Socket_Init(s,Sn_MR_UDP,0x3000,0);
				W5300_Socket_Sendto(s,(uint8*)"x",1,destip,0x3000); // 发送假代码到未知目标地址(0.0.0.1).
				break; 
			}
			Delay_10ms(10);
		}
	}
	setSn_IR(s ,0x00FF);          // 清保留的中断标志.
	setSn_CR(s ,Sn_CR_CLOSE);     // 关闭端口     
}

/*******************************************************************************
* 函数名  : W5300_Socket_Connect
* 描述    : 连接服务器端口
* 输入    : 端口号s       addr:IP+子网+网关 12字节 
* 输出    : 无
* 返回值  : 成功1，失败0
*******************************************************************************/ 
uint8 W5300ConnectCnt;
uint8 W5300_Socket_Connect(SOCKET s, uint8 * addr, uint16 port)
{
	if (((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF) && (addr[3] == 0xFF)) ||
	  ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||       
	  (port == 0x00))
	{
	  return 0;
	}

	// 目标地址 
	W5300_WRITE_REG(Sn_DIPR(s),((uint16)addr[0]<<8)+(uint16)addr[1]);
	W5300_WRITE_REG(Sn_DIPR2(s),((uint16)addr[2]<<8)+(uint16)addr[3]);
	
	// 端口号
	W5300_WRITE_REG(Sn_DPORTR(s),port);  	
	// 连接
	clearSUBR();

	setSn_CR(s,Sn_CR_CONNECT);
	while( (uint8)W5300_READ_REG(Sn_SSR(s)) != SOCK_SYNSENT )
	{
		if((uint8)W5300_READ_REG(Sn_SSR(s)) == SOCK_ESTABLISHED)
		{
			break;
		}
		if(getSn_IR(s) & Sn_IR_TIMEOUT)
		{
			setSn_IR(s,(Sn_IR_TIMEOUT));
			break;
		}
		if(W5300ConnectCnt>3)
		{
		  W5300ConnectCnt = 0;
		  break;
		}
		else
		  W5300ConnectCnt++;
	}	
	setSUBR();
	return 1;   
}
/*******************************************************************************
* 函数名  : W5300_Socket_Disconnect
* 描述    : 断开连接
* 输入    : 端口号s        
* 输出    : 无
* 返回值  : 无
*******************************************************************************/ 
void W5300_Socket_Disconnect(SOCKET s)
{
   setSn_CR(s,Sn_CR_DISCON);     //断开连接
}
/*******************************************************************************
* 函数名  : W5300_Socket_Listen
* 描述    : 打开侦听，用于TCP服务器模式
* 输入    : 端口号s        
* 输出    : 无
* 返回值  : 成功1，失败0
*******************************************************************************/ 
uint8    W5300_Socket_Listen(SOCKET s)
{
   if (getSn_SSR(s) != SOCK_INIT)
   {
      return 0;
   }
   setSn_CR(s,Sn_CR_LISTEN);     // 侦听   
   return 1;
}  
/*******************************************************************************
* 函数名  : S_tx_process
* 描述    : 端口发送函数
* 输入    : 端口号s，buf，len        
* 输出    : 无
* 返回值  : ret(发送长度）
*******************************************************************************/

uint32 S_tx_process(SOCKET s, uint8 * buf, uint32 len)
{
	uint8 status=0;
	uint32 ret=0;
	uint32 freesize=0;
	uint16 Send_Err_cout = 0;  //测试发现，在W5300发送数据的过程中，如果发送不成功，会在等待上一发送命令完成循环中死掉 	
//	uint8 Sn_IR_Status=0;
//	uint8 Sn_SRR_Status=0;
	
	ret = len;
	if (len > getW5300_TxMAX(s))// check size not to exceed MAX size. 
		ret = getW5300_TxMAX(s);
//		return 0; 
 /*
* \note if you want to use non blocking function, <b>"do{}while(freesize < ret)"</b> code block 
* can be replaced with the below code. \n
* \code 
*       while((freesize = getSn_TX_FSR(s))==0); 
*       ret = freesize;
* \endcode
*/   
	do                                   
	{
		freesize = getSn_TX_FSR(s);
		status = getSn_SSR(s);
		
		if( Send_Err_cout > 3)//add 20160927+20170207 避免插拔网线死机问题
		{
			Send_Err_cout = 0;
			return 0;
		}
		if ((status != SOCK_ESTABLISHED) && (status != SOCK_CLOSE_WAIT)) 
			return 0;
	} while (freesize < ret);  //剩余存储空间不够发送
	Send_Err_cout = 0;
	wiz_write_buf(s,buf,ret);                 // 数据拷贝
 
	if(!check_sendok_flag[s])                 // 第一次发送，跳过.
	{
		while (!(getSn_IR(s) & Sn_IR_SENDOK))  // 等待上一发送命令完成.
		{  
			
			if (getSn_SSR(s) == SOCK_CLOSED)    // 检查超时和不正常关闭
			{
				return 0;
			}
			else
			{
//				Sn_IR_Status=getSn_IR(s);
//				Sn_SRR_Status=getSn_SSR(s);
			    Send_Err_cout ++;
				if(Send_Err_cout >= 200)//次数又3改为200次 因为检测时间太短，尚未达到发送超时时间重置端口不合理
				{
				//   Flag_NetConnect = 0;
				   switch(s)
				   {
				       case 0:
					        W5300_Socket_Init(0, Sn_MR_TCP, S0_Port, 1);
						    break;
					   case 1:
					   		W5300_Socket_Init(1, Sn_MR_TCP, S1_Port, 1);
							break;
					   case 2:
					   		W5300_Socket_Init(2, Sn_MR_TCP, S2_Port, 1);
							break;
				   }
					Net4_DisconnectNum++;
				   break;	
				}
			}
		}
		Send_Err_cout = 0;
		setSn_IR(s, Sn_IR_SENDOK);             // 清标志位	
	}
	else 
	{
	   check_sendok_flag[s] = 0;
	}
			
	// send
	setSn_TX_WRSR(s,ret);   //发送长度
	setSn_CR(s,Sn_CR_SEND); //执行发送   
	return ret;
}
/*******************************************************************************
* 函数名  : W5300_Socket_Recv
* 描述    : 端口接收函数（TCP模式）
* 输入    : 端口号s，buf，len        
* 输出    : 无
* 返回值  :pack_size(接收长度）
*******************************************************************************/ 
uint32   W5300_Socket_Recv(SOCKET s, uint8 * buf, uint32 len)
{
	uint16 pack_size=0;
	uint16 mr = getMR();							// 20130905
	
	if(W5300_READ_REG(Sn_MR(s)) & Sn_MR_ALIGN)  //若接收到的数据包数位偶数，该位置1
	{
		wiz_read_buf(s, buf, (uint32)len);
		setSn_CR(s,Sn_CR_RECV);
		return len;
	}											 //当接收到数据的时候会出现前2个或几个字节冲掉的情况，如何处理？

	wiz_read_buf(s,(uint8*)&pack_size,2);        // 提取PACKET-INFO(数据包长度)
	//if( (*(vint16*)MR) & MR_FS )				// Modified 20130905 Fixed bug. (when data received, 1byte data is crashed)
	if( mr & MR_FS )	  //FS 置1为小端模式（W5300默认为大端模式），所以需要在模式寄存器中该位置1
		pack_size = ((((pack_size << 8 ) & 0xFF00)) | ((pack_size >> 8)& 0x00FF));
	wiz_read_buf(s, buf, (uint32)pack_size);     // 数据拷贝   
	setSn_CR(s,Sn_CR_RECV);                      // 清接收标志
	 
   /*
   * \warning  send a packet for updating window size. This code block must be only used when W5300 do only receiving data.
   */
   // ------------------------
   // WARNING CODE BLOCK START 
   
   // M_15052008 : Replace Sn_CR_SEND with Sn_CR_SEND_KEEP.
//	if(!(getSn_IR(s) & Sn_IR_SENDOK))
//	{
//	   setSn_TX_WRSR(s,0);                    // size = 0
//	   setSn_CR(s,Sn_CR_SEND);                // send
//	   while(!(getSn_IR(s) & Sn_IR_SENDOK));  // wait SEND command completion
//	   setSn_IR(s,Sn_IR_SENDOK);              // clear Sn_IR_SENDOK bit
//	}
	
   // M_04072008 : Replace Sn_CR_SEND_KEP with Sn_CR_SEND.
//	if(getSn_RX_RSR(s) == 0)                     // send the window-update packet when the window size is full
//	{
//	   uint8 keep_time = 0;
//	   if((keep_time=getSn_KPALVTR(s)) != 0x00) setSn_KPALVTR(s,0x00); // disables the auto-keep-alive-process
//	   setSn_CR(s,Sn_CR_SEND_KEEP);              // send a keep-alive packet by command
//	   setSn_KPALVTR(s,keep_time);               // restore the previous keep-alive-timer value
//	}
#if 0
	if(getSn_RX_RSR(s) == 0)                     // check if the window size is full or not
	{ /* Sn_RX_RSR can be compared with another value instead of ‘0’,
	  according to the host performance of receiving data */
	  setSn_TX_WRSR(s,1);                       // size : 1 byte dummy size
	  W5300_WRITE_REG(Sn_TX_FIFOR(s),0x0000);   // write dummy data into tx memory
	  setSn_CR(s,Sn_CR_SEND);                   // 发送                         
	  while(!(getSn_IR(s) & Sn_IR_SENDOK));     // 等待发送成功 
	  setSn_IR(s,Sn_IR_SENDOK);                 // 清标志位      
	}    
#endif   
                                                                     
	return (uint32)pack_size;
}
/*******************************************************************************
* 函数名  : W5300_Socket_Sendto
* 描述    : 端口接收函数（UDP模式等）
* 输入    : 端口号s，buf，len ,addr, port        
* 输出    : 无
* 返回值  :ret(发送长度）
*******************************************************************************/ 
uint32   W5300_Socket_Sendto(SOCKET s, uint8 * buf, uint32 len, uint8 * addr, uint16 port)
{
	uint8 status=0;
	uint8 isr=0;
	uint32 ret=0;
	if
	(
		((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
		((port == 0x00)) ||(len == 0)
	)
	{	
		return 0;
	} 
	if (len > getW5300_TxMAX(s)) 
		ret = getW5300_TxMAX(s); // 检查长度未超过MAX size.
	else 
		ret = len; 	
	// 设置目标ip地址
	W5300_WRITE_REG(Sn_DIPR(s),(((uint16)addr[0])<<8) + (uint16) addr[1]);
	W5300_WRITE_REG(Sn_DIPR2(s),(((uint16)addr[2])<<8) + (uint16) addr[3]);
	W5300_WRITE_REG(Sn_DPORTR(s),port);
	Delay_1us(10);
	wiz_write_buf(s, buf, ret);                              // 数据拷贝
	// 发送
	setSn_TX_WRSR(s,ret);
	clearSUBR();
	setSn_CR(s, Sn_CR_SEND);  //执行该语句后才可以读出相应的目标IP地址和目标端口 ，但是读出的端口不正确，却可以发送成功
	Delay_1us(10);
	while (!((isr = (getSn_IR(s))) & Sn_IR_SENDOK))            // 等待发送完成
	{
		status = getSn_SSR(s);                                // 注 ---------------------------------------
		if ((status == SOCK_CLOSED) || (isr & Sn_IR_TIMEOUT)) // Sn_IR_TIMEOUT 会影响剩余发送缓冲区的大小（减小） Sn_TX_FSR
		{                                                     // -----------------------------------------------
		 setSn_IR(s,Sn_IR_TIMEOUT);
		 return 0;
	  	}
	}
	setSUBR();
	setSn_IR(s, Sn_IR_SENDOK); // 清标志位
	return ret;   
}

/*******************************************************************************
* 函数名  : W5300_Socket_Recvfrom
* 描述    : W5300端口接收函数（UDP模式等）
* 输入    : 端口号s，port,buf，addr，len        
* 输出    : 无
* 返回值  : data_len
*******************************************************************************/ 
uint32   W5300_Socket_Recvfrom(SOCKET s, uint8 * buf, uint32 len, uint8 * addr, uint16  *port)
{
   uint16 head[4];
   uint32 data_len=0;
   uint16 crc[2];
   
   #ifdef __DEF_W5300_DBG__
      printf("recvfrom()\r\n");
   #endif
   
   if ( len > 0 )
   {
      switch (W5300_READ_REG(Sn_MR(s)) & 0x07)       // 检查端口模式
      {                                            // -----------------------------
         case Sn_MR_UDP :                          // UDP模式 
            wiz_read_buf(s, (uint8*)head, 8);      // 提取PACKET-INFO
            if(*((vuint16*)MR) & MR_FS)            // 小端模式
            {
               head[0] = ((((head[0] << 8 ) & 0xFF00)) | ((head[0] >> 8)& 0x00FF));
               head[1] = ((((head[1] << 8 ) & 0xFF00)) | ((head[1] >> 8)& 0x00FF));
               head[2] = ((((head[2] << 8 ) & 0xFF00)) | ((head[2] >> 8)& 0x00FF));
               head[3] = ((((head[3] << 8 ) & 0xFF00)) | ((head[3] >> 8)& 0x00FF));
            }
            addr[0] = (uint8)(head[0] >> 8);       // 目标IP地址
            addr[1] = (uint8)head[0];
            addr[2] = (uint8)(head[1]>>8);
            addr[3] = (uint8)head[1];
            *port = head[2];                       // 目标端口号
            data_len = (uint32)head[3];            // 数据长度
            
            #ifdef __DEF_W5300_DBG__
               printf("UDP msg arrived:%d(0x%04x)\r\n",data_len,data_len);
               printf("source Port : %d\r\n", *port);
               printf("source IP : %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
            #endif
   
            wiz_read_buf(s, buf, data_len);        // 数据拷贝.
            break;
                                                   // -----------------------
         case Sn_MR_IPRAW :                        // IPRAW 模式
            wiz_read_buf(s, (uint8*)head, 6);      // 提取PACKET-INFO 
            if(*((vuint16*)MR) & MR_FS)            // 小端模式
            {
               head[0] = ((((head[0] << 8 ) & 0xFF00)) | ((head[0] >> 8)& 0x00FF));
               head[1] = ((((head[1] << 8 ) & 0xFF00)) | ((head[1] >> 8)& 0x00FF));
               head[2] = ((((head[2] << 8 ) & 0xFF00)) | ((head[2] >> 8)& 0x00FF));
            }   			
            addr[0] = (uint8)(head[0] >> 8);       // 目标IP地址
            addr[1] = (uint8)head[0];
            addr[2] = (uint8)(head[1]>>8);
            addr[3] = (uint8)head[1];
            data_len = (uint32)head[2];            // 数据包长度
            
            #ifdef __DEF_W5300_DBG__
               printf("IP RAW msg arrived\r\n");
               printf("source IP : %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
            #endif
            
            wiz_read_buf(s, buf, data_len);        // 数据拷贝.
            break;                                 
                                                   // -----------------------
         case Sn_MR_MACRAW :                       // MACRAW 模式
            wiz_read_buf(s,(uint8*)head,2);        // 提取PACKET-INFO
            if(*((vuint16*)MR) & MR_FS)            // 小端模式
               head[0] = ((((head[0] << 8 ) & 0xFF00)) | ((head[0] >> 8)& 0x00FF));
            data_len = (uint32)head[0];            // 数据包长度
            wiz_read_buf(s, buf, data_len);        // 数据拷贝.
            wiz_read_buf(s,(uint8*)crc, 4);        // 提取CRC位，并忽略.

            break;
         default :
            break;
      }
      setSn_CR(s,Sn_CR_RECV);                      // 进行接收
   }
   #ifdef __DEF_W5300_DBG__
      printf("recvfrom() end ..\r\n");
   #endif
   
   return data_len;   
}

/*******************************************************************************
* 函数名  : W5300_loopback_tcps
* 描述    : TCP回环测试函数（TCP服务器模式）
* 输入    : 端口号s，port,buf，mode        
* 输出    : 无
* 返回值  : 无
*******************************************************************************/ 
uint32 errcount_net = 0;
void W5300_loopback_tcps(SOCKET s, uint16 port, uint8* buf, uint16 mode)
{
	uint32 len;
	uint32 len_send;
	
	switch(getSn_SSR(s))                // 检查端口状态
	{                                   // ------------
		case SOCK_ESTABLISHED:           // 建立连接
			if(getSn_IR(s) & Sn_IR_CON)   //检查是否为连接中断
			{
				setSn_IR(s,Sn_IR_CON);     //清中断状态
			}
			if((len=getSn_RX_RSR(s)) > 0) // 检查是否接收到数据
			{
				len = W5300_Socket_Recv(s,buf,len);     // 接收
				len_send = S_tx_process(s,buf,len); // 发送
				/*此处需要测试最多可以等多长时间才可以使PACKET-INFO数据不被冲掉*/
				if(len != len_send)  // 当前面打着断点而上位机不停发送的时候会导致该种情况发生（INFO内容被有效数据冲掉）
				{
					errcount_net ++	;
//					while(1);
				}
			}
			break;
		                   // ---------------
		case SOCK_CLOSE_WAIT:               // 被动关闭
			W5300_Socket_Disconnect(s);                // 断开连接 
			break;
		                   // --------------
		case SOCK_CLOSED:                   // 关闭端口
			W5300_Socket_Close(s);                        // 关闭端口
			W5300_Socket_Init(s,Sn_MR_TCP,port,mode);   //以TCP模式打开端口 
			break;
		                   // ------------------------------
		case SOCK_INIT:                     // 端口以TCP模式打开
			W5300_Socket_Listen(s);                       // 打开监听
			break;
		default:
			break;
	}
}

/*******************************************************************************
* 函数名  : W5300_State_management
* 描述    : 网络状态管理函数，在网络接收到数据后的状态管理
* 输入    : 端口号s，port,mode        
* 输出    : 无
* 返回值  : 无
*******************************************************************************/ 
uint8 W5300_State_management(SOCKET s, uint16 port, uint16 mode,uint8 *JG_IP,uint16 JG_DPort)
{
	uint8 ret = 0;	//表示网络的状态,1表示可以发送接收数据，0不可以
	
	switch(getSn_SSR(s))                // 检查端口状态
	{                                   // ------------
		case SOCK_ESTABLISHED:           // 建立连接  TCP模式下有此状态
			if(getSn_IR(s) & Sn_IR_CON)   //检查是否为连接中断
			{
				setSn_IR(s,Sn_IR_CON);     //清中断状态
			}
			ret = 1;
			break;
		case SOCK_UDP:  // UDP 模式下状态
			ret = 1;
			break;			
		                   // ---------------
		case SOCK_CLOSE_WAIT:               // 被动关闭
			W5300_Socket_Disconnect(s);                // 断开连接 
			break;
		                   // --------------
		case SOCK_CLOSED:                   // 关闭端口
			W5300_Socket_Close(s);                        // 关闭端口
			if(mode==Sn_MR_TCP)
			{
				W5300_Socket_Init(s,Sn_MR_TCP,port,1);   //以TCP模式打开端口 
			}
			else if(mode==Sn_MR_UDP)
			{
				W5300_Udp_socket(s,Sn_MR_TCP,port,1);   //以UDP模式打开
			}			
			break;
		                   // ------------------------------
		case SOCK_INIT:                     // 端口以TCP模式打开
			W5300_Socket_Connect(s,JG_IP,JG_DPort);
			break;
		default:
			W5300_Socket_Close(s);                        // 关闭端口
			if(mode==Sn_MR_TCP)
			{
				W5300_Socket_Init(s,Sn_MR_TCP,port,1);   //以TCP模式打开端口 
			}
			else if(mode==Sn_MR_UDP)
			{
				W5300_Udp_socket(s,Sn_MR_TCP,port,1);    //以UDP模式打开
			}
			break;
	}
	return ret;
}


/*******************************************************************************
* 函数名  : W5300_loopback_udp
* 描述    : TCP回环测试函数（TCP服务器模式）
* 输入    : 端口号s，port,buf，mode        
* 输出    : 无
* 返回值  : 无
*******************************************************************************/
void W5300_loopback_udp(SOCKET s, uint16 port, uint8* buf, uint16 mode)
{
	uint32 len;
	uint8 destip[4] = {192,168,1,76};//对端IP地址
	uint16 destport = 1000; //对端端口号  
	switch(getSn_SSR(s))
	{ 	                                                     
		case SOCK_UDP:                                      
			if((len=getSn_RX_RSR(s)) > 0)                   //检查是否接收到数据
			{
				len = W5300_Socket_Recvfrom(s,buf,len,destip,&destport);  // 从目标IP地址和目标端口接收数据
				if(len != W5300_Socket_Sendto(s,buf,len,destip,destport))  // 向目标地址发送数据
				{
	//			   printf("%d : Sendto Fail.len=%d,",s,len);
	//			   printf("%d.%d.%d.%d(%d)\r\n",destip[0],destip[1],destip[2],destip[3],destport);
					while(1);
				}
			}
			break;
	                                                     
		case SOCK_CLOSED:                        
			W5300_Socket_Close(s);                                       // 关闭端口
			W5300_Udp_socket(s,Sn_MR_UDP,port,mode);                  // 以UDP的方式打开端口
			break;
		default:
			break;
	}
}

void W5300_Udp_socket(SOCKET s, uint8 protocol, uint16 port, uint16 flag)
{
	W5300_WRITE_REG(Sn_MR(s),(uint16)(protocol | flag)); // 设置端口工作模式
	if (port != 0)
		W5300_WRITE_REG(Sn_PORTR(s),port);
	else 
	{
		W5300_WRITE_REG(Sn_PORTR(s),W5300_source_port);
		W5300_source_port++;     // 若没有设置端口，则设置为默认.
	}
	setSn_CR(s, Sn_CR_OPEN);      // 打开端口    
	check_sendok_flag[s] = 1;     // 初始化发送标志.
}

/*******************************************************************************************************************
** 函数名称:S_rx_process				
**
** 功能描述:接收各SOCKET对应的网络数据 
**
** 输    入:s--对应socket 
**	         
** 输    出:接收到的数据长度 
**
** 注意事项:①当getSn_RX_RSR得到剩余缓存空间为0，跳出本次接收，返回长度为0 
**         :③对s不做检查，注意W5300的S个数上限
**         :③接收的数据存储于Rx_Buffer+s*MAX_RECSIZE 的位置
**
** 设计日期:20170106 
**
********************************************************************************************************************/
uint32 S_rx_process(SOCKET s)
{
	uint32 Rx_Size=0;//接收数据长度
	uint32 fr_Size=0;//缓存剩余空间
	//第一步 得到剩余空间
	fr_Size = getSn_RX_RSR(s);//得到接收缓存剩余空间
	if(fr_Size>0)
	{
		Rx_Size=W5300_Socket_Recv(s,Rx_Buffer+s*MAX_RECSIZE,fr_Size);
		setSn_IR(s,Sn_IR_RECV);     //清中断缓存
	}
	return Rx_Size;
}

