/*************************************************************************************************/
/*** \文件名    w5300.c																  ************/
/*** \作者      李玉株																  ************/
/*** \时间      2013年9月5日														  ************/
/*** \文件说明  1、部分原代码来源于WIZnet官网 1.2.2版本									  ************/
/***   			2、本文件实现了W5300的基本的I/O寄存器访问功能，同时实现了初始化功能等。***********/
/*************************************************************************************************/
#include "w5300.h"
#include "socket.h"
#include "Timer0.h"

static NetInfo  m_sniLocal;  //本机的网络信息

/*****网络参数*****/
static uint8 W5300_GATEWAY[4] ;     /* 网关IP地址 */
static uint8 W5300_SUBNET[4] ;		 /* 子网掩码 */
static uint8 W5300_MAC_ADDR[6] ;	 /* 物理地址 */
static uint8 W5300_IP_ADDR[4] ;	 /* 本机IP地址 */
static uint8 IPVAL[4];
/*****源端口号*****/
uint16 S0_Port;	//4000
uint16 S1_Port;	//4001
uint16 S2_Port;	//4002
uint16 S3_Port;	//4002
/*****目的IP地址*****/
uint8  S0_IP[4];
uint8  S1_IP[4];
uint8  S2_IP[4];
uint8  S3_IP[4];
/*****目的端口号*****/
uint16 S0_DPort;	   //JG0
uint16 S1_DPort;	   //JG1
uint16 S2_DPort;	   //JG2
uint16 S3_DPort;	   //发给远程服务器

/*****各端口协议模式*****/
uint8 S0_Mode;//
uint8 S1_Mode;//
uint8 S2_Mode;//
uint8 S3_Mode;//

uint8 W5300_InterruptFlag = 0;	//用于硬件检测

/**TX memory size variables */
uint32 TXMEM_SIZE[MAX_SOCK_NUM]; 

/** RX memory size variables */
uint32 RXMEM_SIZE[MAX_SOCK_NUM];

/** The variables for SOCKETn interrupt */
volatile uint8 SOCK_INT[MAX_SOCK_NUM];


//////////////////////////////////////////////////////////////////////////////////
//网口IO初始化
void NET_GPIO_init(void)
{
	P0_MUX_CLR = (1 << 1) | ( 1 << 0 );			//Pin:B5 D7 设置为P0.0 P0.1
	P0_DIR_CLR = (1 << 1) | ( 1 << 0 );			//P0.0 P0.1 设置为输入,接SPD COL

	P2_MUX_CLR = (1 << 0);						//Pin:D9,配置为 GPIO_02
	P2_DIR_CLR = (1 << 27 );					//GPIO_02 设置为输入,接W5300_nINT

	P3_OUTP_SET = ( 1 << 11 );					//复位引脚置高
}

/**********************************************************************
程序名: InitNetSetting
输入: m_sniLocal	本地网络参数
输出: 无
返回: 无
说明：初始化网络参数 ，端口号，IP等
**********************************************************************/
void  InitNetSetting(NetInfo *p_sniLocal)
{
	uint8 i;
	memcpy(&m_sniLocal,p_sniLocal,sizeof(m_sniLocal));
	
	//本机IP初始化
	for(i=0;i<4;i++)
	{
	   W5300_IP_ADDR[i] = m_sniLocal.au8IPAddr[i];			// 加载IP地址
	   W5300_SUBNET[i] = m_sniLocal.au8SubMask[i];			// 加载子网掩码
	   W5300_GATEWAY[i] = m_sniLocal.au8GatewayIP[i];			// 加载网关参数
	}


	//加载MAC地址
	for(i=0; i<6; i++)
		W5300_MAC_ADDR[i] = m_sniLocal.au8MACAddr[i];

	//Socket0对应IP和端口初始化  用于接收激光器的波形
	S0_Port = m_sniLocal.u32LocalPortNO ;  //源端口号
	for(i=0; i<4; i++)
	{
		S0_IP[i] = m_sniLocal.SocketIP[0][i];	 // 目的IP地址			
	}
	S0_DPort = m_sniLocal.SocketPort[0] ;  //目的端口号
	S0_Mode=Sn_MR_TCP;
	
	//Socket1对应IP和端口初始化 用于连接外部平台服务器
	S1_Port = m_sniLocal.u32LocalPortNO+1;  //源端口号
	for(i=0; i<4; i++)
	{
			S1_IP[i] = m_sniLocal.SocketIP[1][i];		 // 目的IP地址	
	}
	S1_DPort = m_sniLocal.SocketPort[1];  //目的端口号
    S1_Mode=Sn_MR_TCP;
	
	//Socket2对应IP和端口初始化
	S2_Port = m_sniLocal.u32LocalPortNO+2;  //源端口号
	for(i=0; i<4; i++)
	{
			S2_IP[i] = m_sniLocal.SocketIP[2][i];		 // 目的IP地址	
	}
	S2_DPort = m_sniLocal.SocketPort[2];  //目的端口号
	S2_Mode=Sn_MR_TCP;
	
	//Socket3对应IP和端口初始化 用于网口设置参数
	S3_Port = m_sniLocal.u32LocalPortNO+3;  //源端口号
	for(i=0; i<4; i++)
	{
		S3_IP[i] = m_sniLocal.SocketIP[3][i];		// ??IP??2	
	}
	S3_DPort = m_sniLocal.SocketPort[3];  //目的端口号
	S3_Mode=Sn_MR_TCP;
}


/**********************************************************************
程序名: IRQ_W5300
输入: 无
输出: 无
返回: 无
说明：网络端口中断处理函数
**********************************************************************/
uint32 W5300_T0TC[8] = {0};
//volatile uint32 W5300_i = 0;
void IRQ_W5300(void)
{
   uint16 int_val;
   uint16 idx;
   
   int_val = W5300_READ_REG(IR);
   while(int_val) 
   {          
      for(idx = 0 ; idx < MAX_SOCK_NUM ; idx++)
      {
         if (int_val & IR_SnINT(idx))  // check the SOCKETn interrupt
         {
            SOCK_INT[idx] |= (uint8)W5300_READ_REG(Sn_IR(idx)); // Save the interrupt stauts to SOCK_INT[idx]
            W5300_WRITE_REG(Sn_IR(idx),(uint16)SOCK_INT[idx]);  // Clear the interrupt status bit of SOCKETn
			if(SOCK_INT[idx]&Sn_IR_TIMEOUT)
			{
		  	  W5300_Socket_Close(idx);
	      	  setSn_IR(idx,Sn_IR_TIMEOUT);
			}
			if(SOCK_INT[idx]&Sn_IR_DISCON)
			{
		  	  W5300_Socket_Close(idx);
	      	  setSn_IR(idx,Sn_IR_DISCON);
			}
         }
      }

	  if(SOCK_INT[W5300_SOCKET0]&Sn_IR_RECV) //接收到数据
	  {
		OSSemPost(g_NetS0Recv);
        W5300_T0TC[0]++;  
	  }
	  if(SOCK_INT[W5300_SOCKET1]&Sn_IR_RECV) //接收到数据
	  {
		 OSSemPost(g_NetS1Recv);
         W5300_T0TC[1]++;
	  }	  
	  if(SOCK_INT[W5300_SOCKET2]&Sn_IR_RECV) //接收到数据
	  {
		 OSSemPost(g_NetS2Recv);
         W5300_T0TC[2]++;
	  }
	  if(SOCK_INT[W5300_SOCKET3]&Sn_IR_RECV) //接收到数据
	  {
		 OSSemPost(g_NetS3Recv);
         W5300_T0TC[3]++;
	  }
	      
      if (int_val & (IR_IPCF << 8))    // check the IP conflict interrupt
      {
      }
      if (int_val & (IR_DPUR << 8))    // check the unreachable destination interrupt
      {
      }
      W5300_WRITE_REG(IR, int_val);// & 0xFF00);

	  int_val = W5300_READ_REG(IR);	   //重新读取IR寄存器状态
   }
}

/*******************************************************************************
* 函数名  : W5300_READ_REG
* 描述    : W5300读寄存器函数
* 输入    : addr         
* 输出    : 无
* 返回值  : data
*******************************************************************************/
uint16   W5300_READ_REG(uint32 addr)
{
#if (__DEF_W5300_ADDRESS_MODE__ == __DEF_W5300_DIRECT_MODE__)   
      return (*((vuint16*)addr));
#else
      vuint16  data;  
      W5300_CRITICAL_SECTION_ENTER(); 
      *((vuint16*)IDM_AR) = (uint16)addr;
      data = *((vuint16*)IDM_DR);   
      W5300_CRITICAL_SECTION_EXIT();
      return data;
#endif
}
/*******************************************************************************
* 函数名  : W5300_WRITE_REG
* 描述    : W5300写寄存器函数
* 输入    : addr         
* 输出    : 无
* 返回值  : data
*******************************************************************************/
void     W5300_WRITE_REG(uint32 addr,uint16 data)
{
#if (__DEF_W5300_ADDRESS_MODE__ == __DEF_W5300_DIRECT_MODE__)   
      (*((vuint16*)addr)) = data;

#else
      W5300_CRITICAL_SECTION_ENTER();
      *((vuint16*)IDM_AR) = addr;
      *((vuint16*)IDM_DR) = data;   
      W5300_CRITICAL_SECTION_EXIT();

#endif   
}


/*******************************************************************************
* Function Name  : getMR
* Description    : Get the data form MR register.
* Input          : None
* Output         : None
* Return         : Register data
*******************************************************************************/
uint16 getMR(void)
{
   return *((vint16*)MR);
}
/*******************************************************************************
* 函数名  : W5300_WRITE_REG
* 描述    : W5300写寄存器函数
* 输入    : addr         
* 输出    : 无
* 返回值  : data
*******************************************************************************/
void setMR(uint16 val)
{
   *((vint16*)MR) = val;   
}

void     getSHAR(uint8 * addr)
{
   addr[0] = (uint8)(W5300_READ_REG(SHAR)>>8);
   addr[1] = (uint8)W5300_READ_REG(SHAR);
   addr[2] = (uint8)(W5300_READ_REG(SHAR2)>>8);
   addr[3] = (uint8)W5300_READ_REG(SHAR2);
   addr[4] = (uint8)(W5300_READ_REG(SHAR4)>>8);
   addr[5] = (uint8)W5300_READ_REG(SHAR4);  
}
void     setSHAR(uint8 * addr)
{
   W5300_WRITE_REG(SHAR,(((uint16)addr[0])<<8)+addr[1]);
   W5300_WRITE_REG(SHAR2,((uint16)addr[2]<<8)+addr[3]);
   W5300_WRITE_REG(SHAR4,((uint16)addr[4]<<8)+addr[5]);
}

void     getGAR(uint8 * addr)
{
	addr[0] = (uint8)(W5300_READ_REG(GAR)>>8);
	addr[1] = (uint8)W5300_READ_REG(GAR);
	addr[2] = (uint8)(W5300_READ_REG(GAR2)>>8);
	addr[3] = (uint8)W5300_READ_REG(GAR2);  
}
void     setGAR(uint8 * addr)
{
	W5300_WRITE_REG(GAR, ((uint16)addr[0]<<8)+(uint16)addr[1]);
	W5300_WRITE_REG(GAR2,((uint16)addr[2]<<8)+(uint16)addr[3]);  
}

void     getSUBR(uint8 * addr)
{
	addr[0] = (uint8)W5300_SUBNET[0];
	addr[1] = (uint8)W5300_SUBNET[1];
	addr[2] = (uint8)W5300_SUBNET[2];
	addr[3] = (uint8)W5300_SUBNET[3]; 
}
void     saveSUBR(uint8 * addr)
{
	W5300_SUBNET[0] = addr[0];
	W5300_SUBNET[1] = addr[1];
	W5300_SUBNET[2] = addr[2];
	W5300_SUBNET[3] = addr[3];
}
void     setSUBR()
{
	W5300_WRITE_REG(SUBR, ((uint16)W5300_SUBNET[0]<<8)+(uint16)W5300_SUBNET[1]);
	W5300_WRITE_REG(SUBR2, ((uint16)W5300_SUBNET[2]<<8)+(uint16)W5300_SUBNET[3]);
}
void	clearSUBR()
{
    getSIPR(IPVAL);
	if(IPVAL[0] ==0 &&IPVAL[1] ==0 &&IPVAL[2] ==0&&IPVAL[3] ==0)
    {   
        W5300_WRITE_REG(SUBR, ((uint16)0));
        W5300_WRITE_REG(SUBR2,((uint16)0));   
    }
}

/***********************************
 * COMMON Register Access Function *
 ***********************************/
//void ApplySubnet()
//{
//	W5300_WRITE_REG(SUBR, ((uint16)W5300_SUBNET[0]<<8)+(uint16)W5300_SUBNET[1]);
//	W5300_WRITE_REG(SUBR2,((uint16)W5300_SUBNET[2]<<8)+(uint16)W5300_SUBNET[3]);   
//}
//void ClearSubnet()
//{
//	W5300_WRITE_REG(SUBR, ((uint16)0));
//	W5300_WRITE_REG(SUBR2,((uint16)0));   
//}

/* Network Information */

void     getSIPR(uint8 * addr)
{
	addr[0] = (uint8)(W5300_READ_REG(SIPR)>>8);
	addr[1] = (uint8)W5300_READ_REG(SIPR);
	addr[2] = (uint8)(W5300_READ_REG(SIPR2)>>8);
	addr[3] = (uint8)W5300_READ_REG(SIPR2);	
}
void     setSIPR(uint8 * addr)
{
	W5300_WRITE_REG(SIPR,((uint16)addr[0]<<8)+(uint16)addr[1]);
	W5300_WRITE_REG(SIPR2,((uint16)addr[2]<<8)+(uint16)addr[3]);   
}

/* PIN "BRYDn" */

uint8 getPn_BRDYR(uint8 p)
{
   return (uint8)W5300_READ_REG(Pn_BRDYR(p));
}
void setPn_BRDYR(uint8 p, uint8 cfg)
{
   W5300_WRITE_REG(Pn_BRDYR(p),cfg);   
}

/***********************************
 * SOCKET Register Access Function *
 ***********************************/

/* SOCKET control */

uint16   getSn_MR(SOCKET s)
{
   return W5300_READ_REG(Sn_MR(s));
}
void     setSn_MR(SOCKET s, uint16 mode)
{
   W5300_WRITE_REG(Sn_MR(s),mode);
} 
uint8    getSn_CR(SOCKET s)
{
   return W5300_READ_REG(Sn_CR(s));
}
void     setSn_CR(SOCKET s, uint16 com)
{
   W5300_WRITE_REG(Sn_CR(s),com);
   while(W5300_READ_REG(Sn_CR(s)) & 0x00ff);//W5300接收到命令后会自动清零，但是命令会继续执行（只有低8位有效） changed by lyz 20130923 
}

uint8    getSn_IMR(SOCKET s)
{
   return (uint8)W5300_READ_REG(Sn_IMR(s));
}
void     setSn_IMR(SOCKET s, uint8 mask)
{
   W5300_WRITE_REG(Sn_IMR(s),mask);
}

uint8    getSn_IR(SOCKET s)
{
#ifdef __DEF_W5300_INT__   
      return SOCK_INT[s];
#else                        
      return (uint8)W5300_READ_REG(Sn_IR(s));
   #endif   
}
void     setSn_IR(SOCKET s, uint8 ir)
{
   #ifdef __DEF_W5300_INT__    
      SOCK_INT[s] = SOCK_INT[s] & ~(ir);
   #else                        
      W5300_WRITE_REG(Sn_IR(s),ir);
   #endif   
}


/* SOCKET information */

uint8    getSn_SSR(SOCKET s)
{
   uint8 ssr, ssr1;
   ssr = (uint8)W5300_READ_REG(Sn_SSR(s));     // 第一次

   while(1)
   {
      ssr1 = (uint8)W5300_READ_REG(Sn_SSR(s)); // 第二次
      if(ssr == ssr1) break;                 // 只有在两次读取的值都相同的时候，才认为是有效的.
      ssr = ssr1;                           
   }
   return ssr;
}

void     getSn_DHAR(SOCKET s, uint8* addr)
{
   addr[0] = (uint8)(W5300_READ_REG(Sn_DHAR(s))>>8);
   addr[1] = (uint8)W5300_READ_REG(Sn_DHAR(s));
   addr[2] = (uint8)(W5300_READ_REG(Sn_DHAR2(s))>>8);
   addr[3] = (uint8)W5300_READ_REG(Sn_DHAR2(s));
   addr[4] = (uint8)(W5300_READ_REG(Sn_DHAR4(s))>>8);
   addr[5] = (uint8)W5300_READ_REG(Sn_DHAR4(s));
}

void     setSn_DHAR(SOCKET s, uint8* addr)
{
   W5300_WRITE_REG(Sn_DHAR(s),  ((uint16)(addr[0]<<8)) + addr[1]);
   W5300_WRITE_REG(Sn_DHAR2(s), ((uint16)(addr[2]<<8)) + addr[3]);
   W5300_WRITE_REG(Sn_DHAR4(s), ((uint16)(addr[4]<<8)) + addr[5]);
}

void     setSn_DPORTR(SOCKET s, uint16 port)
{
   W5300_WRITE_REG(Sn_DPORTR(s),port);
}

void     getSn_DIPR(SOCKET s, uint8* addr)
{
   addr[0] = (uint8)(W5300_READ_REG(Sn_DIPR(s))>>8);
   addr[1] = (uint8)W5300_READ_REG(Sn_DIPR(s));
   addr[2] = (uint8)(W5300_READ_REG(Sn_DIPR2(s))>>8);  
   addr[3] = (uint8)W5300_READ_REG(Sn_DIPR2(s));   
}

void     setSn_DIPR(SOCKET s, uint8* addr)
{
   W5300_WRITE_REG(Sn_DIPR(s),  ((uint16)(addr[0]<<8)) + addr[1]);
   W5300_WRITE_REG(Sn_DIPR2(s), ((uint16)(addr[2]<<8)) + addr[3]);  
}

uint16   getSn_MSSR(SOCKET s)
{
   return W5300_READ_REG(Sn_MSSR(s));
}

void     setSn_MSSR(SOCKET s, uint16 mss)
{
   W5300_WRITE_REG(Sn_MSSR(s),mss);
}


/* SOCKET 通信 */

uint8 getSn_KPALVTR(SOCKET s)
{
   return (uint8)(W5300_READ_REG(Sn_KPALVTR(s)) >> 8);
}

void     setSn_KPALVTR(SOCKET s, uint8 time)
{
   uint16 keepalive=0;
   keepalive = (W5300_READ_REG(Sn_KPALVTR(s)) & 0x00FF) + ((uint16)time<<8);
   W5300_WRITE_REG(Sn_KPALVTR(s),keepalive);
}

uint32   getSn_TX_WRSR(SOCKET s)
{
   uint32 tx_write_size=0;
   tx_write_size = W5300_READ_REG(Sn_TX_WRSR(s));
   tx_write_size = (tx_write_size << 16) + W5300_READ_REG(Sn_TX_WRSR2(s));
   return tx_write_size;
}
void     setSn_TX_WRSR(SOCKET s, uint32 size)
{
   W5300_WRITE_REG(Sn_TX_WRSR(s), (uint16)(size >> 16));
   W5300_WRITE_REG(Sn_TX_WRSR2(s), (uint16)size);
}

uint32   getSn_TX_FSR(SOCKET s)	 //获取发送FIFO剩余存储空间
{
   uint32 free_tx_size=0;
   uint32 free_tx_size1=0;
   while(1)
   {
      free_tx_size = W5300_READ_REG(Sn_TX_FSR(s));                                                                
      free_tx_size = (free_tx_size << 16) + W5300_READ_REG(Sn_TX_FSR2(s));                                                       
      if(free_tx_size == free_tx_size1) break;   // 只有在两次读取的值都相同的时候，才认为是有效的.                                                          
      free_tx_size1 = free_tx_size;                                                                                        
   }                                                                       
   return free_tx_size;                                                    
}                                                                          

uint32   getSn_RX_RSR(SOCKET s)	  //获取接收FIFO剩余存储空间
{
   uint32 received_rx_size=0;
   uint32 received_rx_size1=1;
   uint8 loopcnt=0;
   while(1)
   {
		received_rx_size = W5300_READ_REG(Sn_RX_RSR(s));
		received_rx_size = (received_rx_size << 16) + W5300_READ_REG(Sn_RX_RSR2(s));                                       
		if(received_rx_size == received_rx_size1) break;  // 只有在两次读取的值都相同的时候，才认为是有效的.                     
		received_rx_size1 = received_rx_size;
		//以下代码防止拔网线陷入死循环而跳出 20170112
		loopcnt++;
		if(loopcnt>10)
		{
		   break;
		}
   }                                                                                      
   return received_rx_size;   
}


void     setSn_TX_FIFOR(SOCKET s, uint16 data)
{
   W5300_WRITE_REG(Sn_TX_FIFOR(s),data);
}

uint16   getSn_RX_FIFOR(SOCKET s)
{
   return W5300_READ_REG(Sn_RX_FIFOR(s));
}

/* IP 头 */

uint8    getSn_PROTOR(SOCKET s)
{
   return (uint8)W5300_READ_REG(Sn_PROTOR(s));
}
void     setSn_PROTOR(SOCKET s, uint8 pronum)
{
   uint16 protocolnum;
   protocolnum = W5300_READ_REG(Sn_PROTOR(s)) & 0xFF00 + pronum;
   W5300_WRITE_REG(Sn_PROTOR(s),protocolnum);
}

uint8    getSn_TTLR(SOCKET s)
{
   return (uint8)W5300_READ_REG(Sn_TTLR(s));
}
void     setSn_TTLR(SOCKET s, uint8 ttl)
{
   W5300_WRITE_REG(Sn_TTLR(s),ttl);
}

uint8    getSn_FRAGR(SOCKET s)
{
   return (uint8)W5300_READ_REG(Sn_FRAGR(s));
}

void     setSn_FRAGR(SOCKET s, uint8 frag)
{
   W5300_WRITE_REG(Sn_FRAGR(s),frag);
}

/*******
 * ETC *
 *******/

/*******************************************************************************
* 函数名  : W5300_Init
* 描述    : W5300初始化函数
* 输入    : 无         
* 输出    : 无
* 返回值  : 无
*******************************************************************************/ 
void W5300_Init(NetInfo *p_sniLocal)
{
	uint8 tx_mem_conf[8] = {2,2,2,2,2,2,2,2};                  // 用于配置发送缓冲区的大小。总大小为128k
//	uint8 rx_mem_conf[8] = {14,14,14,14,14,14,14,14};          // 用于配置接收缓冲区的大小
	uint8 rx_mem_conf[8] = {28,28,28,28,0,0,0,0};          // 用于配置接收缓冲区的大小		
	volatile uint32 tmp1,tmp2;
	
	NET_GPIO_init();   //配置引脚
	sic2IrqFuncSet(2 , 0 ,(unsigned int)IRQ_W5300);    //中断引脚GPIO_02作为W5300中断信号  下降沿触发中断
    InitNetSetting(p_sniLocal);  //设置网络参数
 
/*1、设置主机接口，配置W5300模式寄存器，选择直接地址寻址*/
	W5300_HardRest();		//硬复位
	
	W5300_WRITE_REG(MR, MR_RST);       //软件复位 
	Delay_1ms(25);  

/*2、设置网络信息，W5300硬件参数设置*/
	setSHAR(W5300_MAC_ADDR);								   //设置MAC地址
	setGAR(W5300_GATEWAY);                                     // 网关
	setSUBR();                                     			   // 子网掩码
	setSIPR(W5300_IP_ADDR);                                    // IP地址
//	getSHAR(W5300_MAC_ADDR);
//	getGAR(W5300_GATEWAY);
//	getSUBR(W5300_SUBNET);
//	getSIPR(W5300_IP_ADDR);

/*3、寄存器配置，分配W5300内部发送/接收缓冲区大小*/
   	W5300_WRITE_REG(MR, getMR() | MR_WDF(7) | MR_FS);  // 采用小端模式	 
	Delay_1us(10);
	//中断配置
	W5300_WRITE_REG(IMR_NET, ((IR_IPCF|IR_DPUR) << 8) | 0x0007); // 中断使能：ip地址冲突、目标地址未到达、端口0~2中断 	
	W5300_WRITE_REG(RTR, 2000);     // 发送超时时间1500/10 = 150ms,标准单位100us
	W5300_WRITE_REG(RCR, 7);          // 重发次数
	if(!TX_RX_Buf_Config(tx_mem_conf,rx_mem_conf))  //等待缓冲区配置结束   //每个分配8K      
	{
	  while(1);		 
	}

	W5300_Socket_Init(0, Sn_MR_TCP, S0_Port, 1); 		//JG0
	W5300_Socket_Init(1, Sn_MR_TCP, S1_Port, 1); 		//JG1
	W5300_Socket_Init(2, Sn_MR_TCP, S2_Port, 1); 		//JG2
	W5300_Udp_socket(3,Sn_MR_UDP, S3_Port, 1);         //数据上报


/*5、网关检测（用于存在网关的情况）*/
//	register_w5300 = Detect_Gateway(0);
//	if(!register_w5300)
//	{
//		while(1);
//	}
}

/*******************************************************************************
Use SOCKET0 to detect Gatewat. If don't use Gateway, this step can be skipped
Return TRUE if detecte Gateway
Return FALSE if no Gateway
*******************************************************************************/
uint8 Detect_Gateway(SOCKET s)
{
	W5300_WRITE_REG(Sn_MR(s),Sn_MR_TCP);// TCP协议
	W5300_WRITE_REG(Sn_CR(s),Sn_CR_OPEN);	//打开端口
	if(SOCK_INIT != (SOCK_INIT & W5300_READ_REG(Sn_SSR(s)))) //判断端口是否打开
	{
		W5300_WRITE_REG(Sn_CR(s),Sn_CR_CLOSE);
		return false;
	}
	setSIPR(W5300_IP_ADDR);             //设置源IP地址
	W5300_WRITE_REG(Sn_CR(s),Sn_CR_CONNECT); //进行连接
	do
	{
		if(W5300_InterruptFlag)
		{
			W5300_InterruptFlag = 0;
		    return true;
		}
		else
		{
		  	if(0xff != W5300_READ_REG(Sn_DHAR(s)))	//目标硬件地址检测
		    	return true;
		}
	}
	while(1);	
}

/*******************************************************************************
* 函数名  : W5300_HardRest
* 描述    : W5300硬件复位函数（硬件复位后需要重新设置网络参数）（软件复位同）
* 输入    : 无         
* 输出    : 无
* 返回值  : 无
*******************************************************************************/
void W5300_HardRest()
{
	P3_OUTP_CLR = ( 1 << 11 ); //引脚E8,GPO_11,	接W5300_nRST
	Delay_1ms(1);        // 2us least
	P3_OUTP_SET = ( 1 << 11 );
	Delay_1ms(25);       // 10ms least//注：硬件复位后好多已经设定的寄存器参数会全部清零，这个时候则需要对W5300进行重新初始化
}


/*******************************************************************************
* 函数名  : TX_RX_Buf_Config
* 描述    : 配置W5300的发送/接收缓冲区大小// Internal memory operation
* 输入    : tx_size rx_size         
* 输出    : 无
* 返回值  : 0失败；1成功
*******************************************************************************/ 
uint8  TX_RX_Buf_Config(uint8* tx_size, uint8* rx_size)
{
	uint16 i;
	uint16 ssum=0,rsum=0;
	uint32 mem_cfg = 0;
	
	for(i=0; i < MAX_SOCK_NUM; i++)
	{
	if(tx_size[i] > 64)
	{
#ifdef __DEF_W5300_DBG__
		printf("Illegal Channel(%d) TX Memory Size.\r\n",i);
#endif
		return 0;
	}
	if(rx_size[i] > 64)
	{
#ifdef __DEF_W5300_DBG__         
		printf("Illegal Channel(%d) RX Memory Size.\r\n",i);
#endif
		return 0;
	}
	ssum += (uint16)tx_size[i];
	rsum += (uint16)rx_size[i];
	TXMEM_SIZE[i] = ((uint32)tx_size[i]) << 10;
	RXMEM_SIZE[i] = ((uint32)rx_size[i]) << 10;
	}
	if( (ssum % 8) || ((ssum + rsum) != 128) )
	{
#ifdef __DEF_W5300_DBG__
		printf("Illegal Memory Allocation\r\n");
#endif
		return 0;
	}	
	W5300_WRITE_REG(TMS01R,((uint16)tx_size[0] << 8) + (uint16)tx_size[1]);
	W5300_WRITE_REG(TMS23R,((uint16)tx_size[2] << 8) + (uint16)tx_size[3]);
	W5300_WRITE_REG(TMS45R,((uint16)tx_size[4] << 8) + (uint16)tx_size[5]);
	W5300_WRITE_REG(TMS67R,((uint16)tx_size[6] << 8) + (uint16)tx_size[7]);
	
	W5300_WRITE_REG(RMS01R,((uint16)rx_size[0] << 8) + (uint16)rx_size[1]);
	W5300_WRITE_REG(RMS23R,((uint16)rx_size[2] << 8) + (uint16)rx_size[3]);
	W5300_WRITE_REG(RMS45R,((uint16)rx_size[4] << 8) + (uint16)rx_size[5]);
	W5300_WRITE_REG(RMS67R,((uint16)rx_size[6] << 8) + (uint16)rx_size[7]);
	
	for(i=0; i <ssum/8 ; i++)
	{
		mem_cfg <<= 1;
		mem_cfg |= 1;
	} 	
	W5300_WRITE_REG(MTYPER,mem_cfg);
	
#ifdef __DEF_W5300_DBG__
	printf("Total TX Memory Size = %dKB\r\n",ssum);
	printf("Total RX Memory Size = %dKB\r\n",rsum);
	printf("Ch : TX SIZE : RECV SIZE\r\n");
	printf("%02d : %07dKB : %07dKB \r\n", 0, (uint8)(W5300_READ_REG(TMS01R)>>8),(uint8)(W5300_READ_REG(RMS01R)>>8));
	printf("%02d : %07dKB : %07dKB \r\n", 1, (uint8)W5300_READ_REG(TMS01R),(uint8)W5300_READ_REG(RMS01R));
	printf("%02d : %07dKB : %07dKB \r\n", 2, (uint8)(W5300_READ_REG(TMS23R)>>8),(uint8)(W5300_READ_REG(RMS23R)>>8));
	printf("%02d : %07dKB : %07dKB \r\n", 3, (uint8)W5300_READ_REG(TMS23R),(uint8)W5300_READ_REG(RMS23R));
	printf("%02d : %07dKB : %07dKB \r\n", 4, (uint8)(W5300_READ_REG(TMS45R)>>8),(uint8)(W5300_READ_REG(RMS45R)>>8));
	printf("%02d : %07dKB : %07dKB \r\n", 5, (uint8)W5300_READ_REG(TMS45R),(uint8)W5300_READ_REG(RMS45R));
	printf("%02d : %07dKB : %07dKB \r\n", 6, (uint8)(W5300_READ_REG(TMS67R)>>8),(uint8)(W5300_READ_REG(RMS67R)>>8));
	printf("%02d : %07dKB : %07dKB \r\n", 7, (uint8)W5300_READ_REG(TMS67R),(uint8)W5300_READ_REG(RMS67R));
	printf("\r\nMTYPER=%04x\r\n",W5300_READ_REG(MTYPER));
#endif 	
	return 1;
}
/*******************************************************************************
* 函数名  : wiz_write_buf
* 描述    : 写W5300的发送缓冲区
* 输入    : 端口s、buf、len         
* 输出    : 无
* 返回值  : len
*******************************************************************************/ 
uint32   wiz_write_buf(SOCKET s,uint8* buf,uint32 len)
{
#if (__DEF_W5300_ADDRESS_MODE__ == __DEF_W5300_DIRECT_MODE__)
   #if (__DEF_W5300_BUF_OP__ == __DEF_C__)
      uint32 idx=0;
      W5300_CRITICAL_SECTION_ENTER();
      for (idx = 0; idx < len; idx+=2)
         W5300_WRITE_REG(Sn_TX_FIFOR(s),*((uint16*)(buf+idx))); 
      W5300_CRITICAL_SECTION_EXIT();
   #elif (__DEF_W5300_BUF_OP__ == __DEF_MCU_DEP_INLINE_ASM__)
      register uint16 data;
      register uint16 loop;
      loop = len;
      W5300_CRITICAL_SECTION_ENTER();
      __asm
      {
      fifo_write_loop:
        LDRH    data, [buf], #2
        STRH    data, [Sn_TX_FIFOR(s)]
        SUBS    loop,loop, #2
        BGT     fifo_write_loop
      } 
      W5300_CRITICAL_SECTION_EXIT();
   #elif (__DEF_W5300_BUF_OP__ == __DEF_MCU_DEP_DMA__)
      #error "You should do implement this block."
   #endif
#elif (__DEF_W5300_ADDRESS_MODE__ == __DEF_W5300_INDIRECT_MODE__)
   #if (__DEF_W5300_BUF_OP__ == __DEF_C__)
      uint32 idx=0;
      W5300_CRITICAL_SECTION_ENTER();
      *((vuint16*)IDM_AR) = Sn_TX_FIFOR(s);
      for (idx = 0; idx < len; idx+=2)
          *((vuint16*)IDM_DR) = *((uint16*)(buf+idx));
      W5300_CRITICAL_SECTION_EXIT();

  #elif (__DEF_W5300_BUF_OP__ == __DEF_MCU_DEP_INLINE_ASM__)
      register uint16 data=0;
      register uint16 loop;
      loop = len;
      W5300_CRITICAL_SECTION_ENTER();    
      *((vuint16*)IDM_AR) = Sn_TX_FIFOR(s);
      __asm
      {
      fifo_write_loop:
        LDRH    data, [buf], #2
        STRH    data, [IDM_DR]
        SUBS    loop,loop, #2
        BGT     fifo_write_loop      
      }
      W5300_CRITICAL_SECTION_EXIT();
   #elif (__DEF_W5300_BUF_OP__ == __DEF_MCU_DEP_DMA__)
      #error "You should do implement this block."
   #else
      #error "Undefined __DEF_W5300_BUF_OP__"
   #endif 
#else
   #error "Undefined __DEF_W5300_ADDRESS_MODE__"   
#endif
    return len;   
}

/*******************************************************************************
* 函数名  : wiz_read_buf
* 描述    : 读W5300的接收缓冲区
* 输入    : 端口s、buf、len         
* 输出    : 无
* 返回值  : len
*******************************************************************************/ 
uint32   wiz_read_buf(SOCKET s, uint8* buf,uint32 len)
{

#if (__DEF_W5300_ADDRESS_MODE__ == __DEF_W5300_DIRECT_MODE__)
   #if (__DEF_W5300_BUF_OP__ == __DEF_C__)
      uint32 idx;
      W5300_CRITICAL_SECTION_ENTER();
      for (idx = 0; idx < len; idx+=2)
	  {
         *((uint16*)(buf+idx)) = W5300_READ_REG(Sn_RX_FIFOR(s));	  
	  }
      W5300_CRITICAL_SECTION_EXIT();
   #elif (__DEF_W5300_BUF_OP__ == __DEF_MCU_DEP_INLINE_ASM__)
      register uint16 data=0;
      register int16 loop;
      loop = (int16) len;
      W5300_CRITICAL_SECTION_ENTER();
      __asm
      {
      fifo_read_loop:
        LDRH    data, [Sn_RX_FIFOR(s)]
        STRH    data, [buf], #2
        SUBS    loop,loop, #2
        BGT     fifo_read_loop
      }
      W5300_CRITICAL_SECTION_EXIT();
   #elif (__DEF_W5300_BUF_OP__ == __DEF_MCU_DEP_DMA__)
      #error "You should do implement this block."
   #else
      #error "Undefined __DEF_W5300_BUF_OP__"
   #endif 
#elif (__DEF_W5300_ADDRESS_MODE__ == __DEF_W5300_INDIRECT_MODE__)
   #if (__DEF_W5300_BUF_OP__ == __DEF_C__)
      uint32 idx=0;
      W5300_CRITICAL_SECTION_ENTER();
      *((vuint16*)IDM_AR) = Sn_RX_FIFOR(s);
      for (idx = 0; idx < len; idx+=2)
         *((uint16*)(buf+idx)) = *((vuint16*)IDM_DR);
      W5300_CRITICAL_SECTION_EXIT();

   #elif (__DEF_W5300_BUF_OP__ == __DEF_MCU_DEP_INLINE_ASM__)
      register uint16 data=0;
      register int16 loop;
      loop = (int16)len;
      W5300_CRITICAL_SECTION_ENTER();      
      *((vuint16*)IDM_AR) = Sn_RX_FIFOR(s);
      __asm
      {
      fifo_read_loop:
        LDRH    data, [IDM_DR]
        STRH    data, [buf], #2
        SUBS    loop,loop, #2
        BGT     fifo_read_loop      
      }
      W5300_CRITICAL_SECTION_EXIT();
   #elif (__DEF_W5300_BUF_OP__ == __DEF_MCU_DEP_DMA__)
      #error "You should do implement this block."
   #else
      #error "Undefined __DEF_W5300_BUF_OP__"
   #endif 
#else
   #error "Undefined __DEF_W5300_ADDRESS_MODE__"   
#endif
    return len;
}


uint32   getW5300_TxMAX(SOCKET s)
{
   return TXMEM_SIZE[s];
}

uint32   getW5300_RxMAX(SOCKET s)
{
   return RXMEM_SIZE[s];
}

/*******************************************************************************
各个级别的延时函数for W5300
*******************************************************************************/
void Delay_1us(uint32 us)
{
   uint32 i,j;
   for(i = 0; i < us ; i++)
   {
      for(j = 0; j < 50; j++);
   }
}

void Delay_1ms(uint32 ms)
{
   uint32 i;
	 for(; ms != 0; ms--)	
 		 for (i=0; i<8300; i++); 
}

void Delay_10ms(uint32 ms)
{
   uint32 i;
   for(i = 0; i < ms ; i++)
   {
     Delay_1ms(10);
   }
}


