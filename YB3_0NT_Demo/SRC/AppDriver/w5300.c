/*************************************************************************************************/
/*** \�ļ���    w5300.c																  ************/
/*** \����      ������																  ************/
/*** \ʱ��      2013��9��5��														  ************/
/*** \�ļ�˵��  1������ԭ������Դ��WIZnet���� 1.2.2�汾									  ************/
/***   			2�����ļ�ʵ����W5300�Ļ�����I/O�Ĵ������ʹ��ܣ�ͬʱʵ���˳�ʼ�����ܵȡ�***********/
/*************************************************************************************************/
#include "w5300.h"
#include "socket.h"
#include "Timer0.h"

static NetInfo  m_sniLocal;  //������������Ϣ

/*****�������*****/
static uint8 W5300_GATEWAY[4] ;     /* ����IP��ַ */
static uint8 W5300_SUBNET[4] ;		 /* �������� */
static uint8 W5300_MAC_ADDR[6] ;	 /* �����ַ */
static uint8 W5300_IP_ADDR[4] ;	 /* ����IP��ַ */
static uint8 IPVAL[4];
/*****Դ�˿ں�*****/
uint16 S0_Port;	//4000
uint16 S1_Port;	//4001
uint16 S2_Port;	//4002
uint16 S3_Port;	//4002
/*****Ŀ��IP��ַ*****/
uint8  S0_IP[4];
uint8  S1_IP[4];
uint8  S2_IP[4];
uint8  S3_IP[4];
/*****Ŀ�Ķ˿ں�*****/
uint16 S0_DPort;	   //JG0
uint16 S1_DPort;	   //JG1
uint16 S2_DPort;	   //JG2
uint16 S3_DPort;	   //����Զ�̷�����

/*****���˿�Э��ģʽ*****/
uint8 S0_Mode;//
uint8 S1_Mode;//
uint8 S2_Mode;//
uint8 S3_Mode;//

uint8 W5300_InterruptFlag = 0;	//����Ӳ�����

/**TX memory size variables */
uint32 TXMEM_SIZE[MAX_SOCK_NUM]; 

/** RX memory size variables */
uint32 RXMEM_SIZE[MAX_SOCK_NUM];

/** The variables for SOCKETn interrupt */
volatile uint8 SOCK_INT[MAX_SOCK_NUM];


//////////////////////////////////////////////////////////////////////////////////
//����IO��ʼ��
void NET_GPIO_init(void)
{
	P0_MUX_CLR = (1 << 1) | ( 1 << 0 );			//Pin:B5 D7 ����ΪP0.0 P0.1
	P0_DIR_CLR = (1 << 1) | ( 1 << 0 );			//P0.0 P0.1 ����Ϊ����,��SPD COL

	P2_MUX_CLR = (1 << 0);						//Pin:D9,����Ϊ GPIO_02
	P2_DIR_CLR = (1 << 27 );					//GPIO_02 ����Ϊ����,��W5300_nINT

	P3_OUTP_SET = ( 1 << 11 );					//��λ�����ø�
}

/**********************************************************************
������: InitNetSetting
����: m_sniLocal	�����������
���: ��
����: ��
˵������ʼ��������� ���˿ںţ�IP��
**********************************************************************/
void  InitNetSetting(NetInfo *p_sniLocal)
{
	uint8 i;
	memcpy(&m_sniLocal,p_sniLocal,sizeof(m_sniLocal));
	
	//����IP��ʼ��
	for(i=0;i<4;i++)
	{
	   W5300_IP_ADDR[i] = m_sniLocal.au8IPAddr[i];			// ����IP��ַ
	   W5300_SUBNET[i] = m_sniLocal.au8SubMask[i];			// ������������
	   W5300_GATEWAY[i] = m_sniLocal.au8GatewayIP[i];			// �������ز���
	}


	//����MAC��ַ
	for(i=0; i<6; i++)
		W5300_MAC_ADDR[i] = m_sniLocal.au8MACAddr[i];

	//Socket0��ӦIP�Ͷ˿ڳ�ʼ��  ���ڽ��ռ������Ĳ���
	S0_Port = m_sniLocal.u32LocalPortNO ;  //Դ�˿ں�
	for(i=0; i<4; i++)
	{
		S0_IP[i] = m_sniLocal.SocketIP[0][i];	 // Ŀ��IP��ַ			
	}
	S0_DPort = m_sniLocal.SocketPort[0] ;  //Ŀ�Ķ˿ں�
	S0_Mode=Sn_MR_TCP;
	
	//Socket1��ӦIP�Ͷ˿ڳ�ʼ�� ���������ⲿƽ̨������
	S1_Port = m_sniLocal.u32LocalPortNO+1;  //Դ�˿ں�
	for(i=0; i<4; i++)
	{
			S1_IP[i] = m_sniLocal.SocketIP[1][i];		 // Ŀ��IP��ַ	
	}
	S1_DPort = m_sniLocal.SocketPort[1];  //Ŀ�Ķ˿ں�
    S1_Mode=Sn_MR_TCP;
	
	//Socket2��ӦIP�Ͷ˿ڳ�ʼ��
	S2_Port = m_sniLocal.u32LocalPortNO+2;  //Դ�˿ں�
	for(i=0; i<4; i++)
	{
			S2_IP[i] = m_sniLocal.SocketIP[2][i];		 // Ŀ��IP��ַ	
	}
	S2_DPort = m_sniLocal.SocketPort[2];  //Ŀ�Ķ˿ں�
	S2_Mode=Sn_MR_TCP;
	
	//Socket3��ӦIP�Ͷ˿ڳ�ʼ�� �����������ò���
	S3_Port = m_sniLocal.u32LocalPortNO+3;  //Դ�˿ں�
	for(i=0; i<4; i++)
	{
		S3_IP[i] = m_sniLocal.SocketIP[3][i];		// ??IP??2	
	}
	S3_DPort = m_sniLocal.SocketPort[3];  //Ŀ�Ķ˿ں�
	S3_Mode=Sn_MR_TCP;
}


/**********************************************************************
������: IRQ_W5300
����: ��
���: ��
����: ��
˵��������˿��жϴ�����
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

	  if(SOCK_INT[W5300_SOCKET0]&Sn_IR_RECV) //���յ�����
	  {
		OSSemPost(g_NetS0Recv);
        W5300_T0TC[0]++;  
	  }
	  if(SOCK_INT[W5300_SOCKET1]&Sn_IR_RECV) //���յ�����
	  {
		 OSSemPost(g_NetS1Recv);
         W5300_T0TC[1]++;
	  }	  
	  if(SOCK_INT[W5300_SOCKET2]&Sn_IR_RECV) //���յ�����
	  {
		 OSSemPost(g_NetS2Recv);
         W5300_T0TC[2]++;
	  }
	  if(SOCK_INT[W5300_SOCKET3]&Sn_IR_RECV) //���յ�����
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

	  int_val = W5300_READ_REG(IR);	   //���¶�ȡIR�Ĵ���״̬
   }
}

/*******************************************************************************
* ������  : W5300_READ_REG
* ����    : W5300���Ĵ�������
* ����    : addr         
* ���    : ��
* ����ֵ  : data
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
* ������  : W5300_WRITE_REG
* ����    : W5300д�Ĵ�������
* ����    : addr         
* ���    : ��
* ����ֵ  : data
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
* ������  : W5300_WRITE_REG
* ����    : W5300д�Ĵ�������
* ����    : addr         
* ���    : ��
* ����ֵ  : data
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
   while(W5300_READ_REG(Sn_CR(s)) & 0x00ff);//W5300���յ��������Զ����㣬������������ִ�У�ֻ�е�8λ��Ч�� changed by lyz 20130923 
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
   ssr = (uint8)W5300_READ_REG(Sn_SSR(s));     // ��һ��

   while(1)
   {
      ssr1 = (uint8)W5300_READ_REG(Sn_SSR(s)); // �ڶ���
      if(ssr == ssr1) break;                 // ֻ�������ζ�ȡ��ֵ����ͬ��ʱ�򣬲���Ϊ����Ч��.
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


/* SOCKET ͨ�� */

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

uint32   getSn_TX_FSR(SOCKET s)	 //��ȡ����FIFOʣ��洢�ռ�
{
   uint32 free_tx_size=0;
   uint32 free_tx_size1=0;
   while(1)
   {
      free_tx_size = W5300_READ_REG(Sn_TX_FSR(s));                                                                
      free_tx_size = (free_tx_size << 16) + W5300_READ_REG(Sn_TX_FSR2(s));                                                       
      if(free_tx_size == free_tx_size1) break;   // ֻ�������ζ�ȡ��ֵ����ͬ��ʱ�򣬲���Ϊ����Ч��.                                                          
      free_tx_size1 = free_tx_size;                                                                                        
   }                                                                       
   return free_tx_size;                                                    
}                                                                          

uint32   getSn_RX_RSR(SOCKET s)	  //��ȡ����FIFOʣ��洢�ռ�
{
   uint32 received_rx_size=0;
   uint32 received_rx_size1=1;
   uint8 loopcnt=0;
   while(1)
   {
		received_rx_size = W5300_READ_REG(Sn_RX_RSR(s));
		received_rx_size = (received_rx_size << 16) + W5300_READ_REG(Sn_RX_RSR2(s));                                       
		if(received_rx_size == received_rx_size1) break;  // ֻ�������ζ�ȡ��ֵ����ͬ��ʱ�򣬲���Ϊ����Ч��.                     
		received_rx_size1 = received_rx_size;
		//���´����ֹ������������ѭ�������� 20170112
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

/* IP ͷ */

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
* ������  : W5300_Init
* ����    : W5300��ʼ������
* ����    : ��         
* ���    : ��
* ����ֵ  : ��
*******************************************************************************/ 
void W5300_Init(NetInfo *p_sniLocal)
{
	uint8 tx_mem_conf[8] = {2,2,2,2,2,2,2,2};                  // �������÷��ͻ������Ĵ�С���ܴ�СΪ128k
//	uint8 rx_mem_conf[8] = {14,14,14,14,14,14,14,14};          // �������ý��ջ������Ĵ�С
	uint8 rx_mem_conf[8] = {28,28,28,28,0,0,0,0};          // �������ý��ջ������Ĵ�С		
	volatile uint32 tmp1,tmp2;
	
	NET_GPIO_init();   //��������
	sic2IrqFuncSet(2 , 0 ,(unsigned int)IRQ_W5300);    //�ж�����GPIO_02��ΪW5300�ж��ź�  �½��ش����ж�
    InitNetSetting(p_sniLocal);  //�����������
 
/*1�����������ӿڣ�����W5300ģʽ�Ĵ�����ѡ��ֱ�ӵ�ַѰַ*/
	W5300_HardRest();		//Ӳ��λ
	
	W5300_WRITE_REG(MR, MR_RST);       //�����λ 
	Delay_1ms(25);  

/*2������������Ϣ��W5300Ӳ����������*/
	setSHAR(W5300_MAC_ADDR);								   //����MAC��ַ
	setGAR(W5300_GATEWAY);                                     // ����
	setSUBR();                                     			   // ��������
	setSIPR(W5300_IP_ADDR);                                    // IP��ַ
//	getSHAR(W5300_MAC_ADDR);
//	getGAR(W5300_GATEWAY);
//	getSUBR(W5300_SUBNET);
//	getSIPR(W5300_IP_ADDR);

/*3���Ĵ������ã�����W5300�ڲ�����/���ջ�������С*/
   	W5300_WRITE_REG(MR, getMR() | MR_WDF(7) | MR_FS);  // ����С��ģʽ	 
	Delay_1us(10);
	//�ж�����
	W5300_WRITE_REG(IMR_NET, ((IR_IPCF|IR_DPUR) << 8) | 0x0007); // �ж�ʹ�ܣ�ip��ַ��ͻ��Ŀ���ַδ����˿�0~2�ж� 	
	W5300_WRITE_REG(RTR, 2000);     // ���ͳ�ʱʱ��1500/10 = 150ms,��׼��λ100us
	W5300_WRITE_REG(RCR, 7);          // �ط�����
	if(!TX_RX_Buf_Config(tx_mem_conf,rx_mem_conf))  //�ȴ����������ý���   //ÿ������8K      
	{
	  while(1);		 
	}

	W5300_Socket_Init(0, Sn_MR_TCP, S0_Port, 1); 		//JG0
	W5300_Socket_Init(1, Sn_MR_TCP, S1_Port, 1); 		//JG1
	W5300_Socket_Init(2, Sn_MR_TCP, S2_Port, 1); 		//JG2
	W5300_Udp_socket(3,Sn_MR_UDP, S3_Port, 1);         //�����ϱ�


/*5�����ؼ�⣨���ڴ������ص������*/
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
	W5300_WRITE_REG(Sn_MR(s),Sn_MR_TCP);// TCPЭ��
	W5300_WRITE_REG(Sn_CR(s),Sn_CR_OPEN);	//�򿪶˿�
	if(SOCK_INIT != (SOCK_INIT & W5300_READ_REG(Sn_SSR(s)))) //�ж϶˿��Ƿ��
	{
		W5300_WRITE_REG(Sn_CR(s),Sn_CR_CLOSE);
		return false;
	}
	setSIPR(W5300_IP_ADDR);             //����ԴIP��ַ
	W5300_WRITE_REG(Sn_CR(s),Sn_CR_CONNECT); //��������
	do
	{
		if(W5300_InterruptFlag)
		{
			W5300_InterruptFlag = 0;
		    return true;
		}
		else
		{
		  	if(0xff != W5300_READ_REG(Sn_DHAR(s)))	//Ŀ��Ӳ����ַ���
		    	return true;
		}
	}
	while(1);	
}

/*******************************************************************************
* ������  : W5300_HardRest
* ����    : W5300Ӳ����λ������Ӳ����λ����Ҫ������������������������λͬ��
* ����    : ��         
* ���    : ��
* ����ֵ  : ��
*******************************************************************************/
void W5300_HardRest()
{
	P3_OUTP_CLR = ( 1 << 11 ); //����E8,GPO_11,	��W5300_nRST
	Delay_1ms(1);        // 2us least
	P3_OUTP_SET = ( 1 << 11 );
	Delay_1ms(25);       // 10ms least//ע��Ӳ����λ��ö��Ѿ��趨�ļĴ���������ȫ�����㣬���ʱ������Ҫ��W5300�������³�ʼ��
}


/*******************************************************************************
* ������  : TX_RX_Buf_Config
* ����    : ����W5300�ķ���/���ջ�������С// Internal memory operation
* ����    : tx_size rx_size         
* ���    : ��
* ����ֵ  : 0ʧ�ܣ�1�ɹ�
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
* ������  : wiz_write_buf
* ����    : дW5300�ķ��ͻ�����
* ����    : �˿�s��buf��len         
* ���    : ��
* ����ֵ  : len
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
* ������  : wiz_read_buf
* ����    : ��W5300�Ľ��ջ�����
* ����    : �˿�s��buf��len         
* ���    : ��
* ����ֵ  : len
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
�����������ʱ����for W5300
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


