/************************************************************************************************************/
/*** \�ļ���    socket.c																  *******************/
/*** \����      ������																  ***********************/
/*** \ʱ��      2013��9��5��														  ***********************/
/*** \�ļ�˵��  1������ԭ������Դ��WIZnet���� 1.2.3�汾									  ***********************/
/***   			2�����ļ�ʵ����W5300�Ļ�����socket�˿ڷ��ʹ��ܣ�ͬʱʵ���˸�дһ�µ�����ͨ�Ź��ܵ�***********/
/************************************************************************************************************/
#include "socket.h"
#include "JZStructure.h"
__align(2) uint8   Rx_Buffer[W5300BUFSIZE];
__align(2) uint8   Tx_Buffer[W5300BUFSIZE];

/** Ĭ�϶˿ںţ��ݼӣ�*/
uint16   W5300_source_port = 4000;

/*** �װ����ͱ�־λ*/
uint8    check_sendok_flag[MAX_SOCK_NUM];

/*******************************************************************************
* ������  : W5300_Socket_Init
* ����    : W5300d�˿ڳ�ʼ������
* ����    : socket s ��protocol ������ʽ��port �˿� ��flag        
* ���    : ��
* ����ֵ  : ��
*******************************************************************************/ 
void W5300_Socket_Init(SOCKET s, uint8 protocol, uint16 port, uint16 flag)
{
	W5300_WRITE_REG(Sn_MR(s),(uint16)(protocol | flag | Sn_MR_ND )); // ������Ӧ�Ķ˿�Ϊָ��ͨ��Э�� & flag, | Sn_MR_ALIGN
    
    W5300_WRITE_REG(Sn_MSSR(s),0x05B4); //MTU����Ϊ1460���ֽ�
	  W5300_WRITE_REG(Sn_KPALVTR(s),0x02); //���ñ��ֻ�Ծʱ��10s 
	
	if (port != 0) 
		W5300_WRITE_REG(Sn_PORTR(s),port);
	else 
	{
		W5300_WRITE_REG(Sn_PORTR(s),W5300_source_port);	
		W5300_source_port++;     // ��û�����ö˿�������ΪĬ�϶˿ں�.		 
	}
	W5300_WRITE_REG(Sn_IMR(s),0x001F);//socket �ж�ʹ��	(��ֹ��������ж�20130918��
	setSn_CR(s, Sn_CR_OPEN);      // �򿪶˿� 
	check_sendok_flag[s] = 1;     // ��ʼ�����ͱ�־
}
/*******************************************************************************
* ������  : W5300_Socket_Close
* ����    : �ر�W5300�˿�
* ����    : �˿ں�s        
* ���    : ��
* ����ֵ  : ��
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
				// �޸�Ŀ��IP��ַ
				destip[0] = 0;destip[1] = 0;destip[2] = 0;destip[3] = 1;
				W5300_Socket_Init(s,Sn_MR_UDP,0x3000,0);
				W5300_Socket_Sendto(s,(uint8*)"x",1,destip,0x3000); // ���ͼٴ��뵽δ֪Ŀ���ַ(0.0.0.1).
				break; 
			}
			Delay_10ms(10);
		}
	}
	setSn_IR(s ,0x00FF);          // �屣�����жϱ�־.
	setSn_CR(s ,Sn_CR_CLOSE);     // �رն˿�     
}

/*******************************************************************************
* ������  : W5300_Socket_Connect
* ����    : ���ӷ������˿�
* ����    : �˿ں�s       addr:IP+����+���� 12�ֽ� 
* ���    : ��
* ����ֵ  : �ɹ�1��ʧ��0
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

	// Ŀ���ַ 
	W5300_WRITE_REG(Sn_DIPR(s),((uint16)addr[0]<<8)+(uint16)addr[1]);
	W5300_WRITE_REG(Sn_DIPR2(s),((uint16)addr[2]<<8)+(uint16)addr[3]);
	
	// �˿ں�
	W5300_WRITE_REG(Sn_DPORTR(s),port);  	
	// ����
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
* ������  : W5300_Socket_Disconnect
* ����    : �Ͽ�����
* ����    : �˿ں�s        
* ���    : ��
* ����ֵ  : ��
*******************************************************************************/ 
void W5300_Socket_Disconnect(SOCKET s)
{
   setSn_CR(s,Sn_CR_DISCON);     //�Ͽ�����
}
/*******************************************************************************
* ������  : W5300_Socket_Listen
* ����    : ������������TCP������ģʽ
* ����    : �˿ں�s        
* ���    : ��
* ����ֵ  : �ɹ�1��ʧ��0
*******************************************************************************/ 
uint8    W5300_Socket_Listen(SOCKET s)
{
   if (getSn_SSR(s) != SOCK_INIT)
   {
      return 0;
   }
   setSn_CR(s,Sn_CR_LISTEN);     // ����   
   return 1;
}  
/*******************************************************************************
* ������  : S_tx_process
* ����    : �˿ڷ��ͺ���
* ����    : �˿ں�s��buf��len        
* ���    : ��
* ����ֵ  : ret(���ͳ��ȣ�
*******************************************************************************/

uint32 S_tx_process(SOCKET s, uint8 * buf, uint32 len)
{
	uint8 status=0;
	uint32 ret=0;
	uint32 freesize=0;
	uint16 Send_Err_cout = 0;  //���Է��֣���W5300�������ݵĹ����У�������Ͳ��ɹ������ڵȴ���һ�����������ѭ�������� 	
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
		
		if( Send_Err_cout > 3)//add 20160927+20170207 ������������������
		{
			Send_Err_cout = 0;
			return 0;
		}
		if ((status != SOCK_ESTABLISHED) && (status != SOCK_CLOSE_WAIT)) 
			return 0;
	} while (freesize < ret);  //ʣ��洢�ռ䲻������
	Send_Err_cout = 0;
	wiz_write_buf(s,buf,ret);                 // ���ݿ���
 
	if(!check_sendok_flag[s])                 // ��һ�η��ͣ�����.
	{
		while (!(getSn_IR(s) & Sn_IR_SENDOK))  // �ȴ���һ�����������.
		{  
			
			if (getSn_SSR(s) == SOCK_CLOSED)    // ��鳬ʱ�Ͳ������ر�
			{
				return 0;
			}
			else
			{
//				Sn_IR_Status=getSn_IR(s);
//				Sn_SRR_Status=getSn_SSR(s);
			    Send_Err_cout ++;
				if(Send_Err_cout >= 200)//������3��Ϊ200�� ��Ϊ���ʱ��̫�̣���δ�ﵽ���ͳ�ʱʱ�����ö˿ڲ�����
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
		setSn_IR(s, Sn_IR_SENDOK);             // ���־λ	
	}
	else 
	{
	   check_sendok_flag[s] = 0;
	}
			
	// send
	setSn_TX_WRSR(s,ret);   //���ͳ���
	setSn_CR(s,Sn_CR_SEND); //ִ�з���   
	return ret;
}
/*******************************************************************************
* ������  : W5300_Socket_Recv
* ����    : �˿ڽ��պ�����TCPģʽ��
* ����    : �˿ں�s��buf��len        
* ���    : ��
* ����ֵ  :pack_size(���ճ��ȣ�
*******************************************************************************/ 
uint32   W5300_Socket_Recv(SOCKET s, uint8 * buf, uint32 len)
{
	uint16 pack_size=0;
	uint16 mr = getMR();							// 20130905
	
	if(W5300_READ_REG(Sn_MR(s)) & Sn_MR_ALIGN)  //�����յ������ݰ���λż������λ��1
	{
		wiz_read_buf(s, buf, (uint32)len);
		setSn_CR(s,Sn_CR_RECV);
		return len;
	}											 //�����յ����ݵ�ʱ������ǰ2���򼸸��ֽڳ�����������δ���

	wiz_read_buf(s,(uint8*)&pack_size,2);        // ��ȡPACKET-INFO(���ݰ�����)
	//if( (*(vint16*)MR) & MR_FS )				// Modified 20130905 Fixed bug. (when data received, 1byte data is crashed)
	if( mr & MR_FS )	  //FS ��1ΪС��ģʽ��W5300Ĭ��Ϊ���ģʽ����������Ҫ��ģʽ�Ĵ����и�λ��1
		pack_size = ((((pack_size << 8 ) & 0xFF00)) | ((pack_size >> 8)& 0x00FF));
	wiz_read_buf(s, buf, (uint32)pack_size);     // ���ݿ���   
	setSn_CR(s,Sn_CR_RECV);                      // ����ձ�־
	 
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
	{ /* Sn_RX_RSR can be compared with another value instead of ��0��,
	  according to the host performance of receiving data */
	  setSn_TX_WRSR(s,1);                       // size : 1 byte dummy size
	  W5300_WRITE_REG(Sn_TX_FIFOR(s),0x0000);   // write dummy data into tx memory
	  setSn_CR(s,Sn_CR_SEND);                   // ����                         
	  while(!(getSn_IR(s) & Sn_IR_SENDOK));     // �ȴ����ͳɹ� 
	  setSn_IR(s,Sn_IR_SENDOK);                 // ���־λ      
	}    
#endif   
                                                                     
	return (uint32)pack_size;
}
/*******************************************************************************
* ������  : W5300_Socket_Sendto
* ����    : �˿ڽ��պ�����UDPģʽ�ȣ�
* ����    : �˿ں�s��buf��len ,addr, port        
* ���    : ��
* ����ֵ  :ret(���ͳ��ȣ�
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
		ret = getW5300_TxMAX(s); // ��鳤��δ����MAX size.
	else 
		ret = len; 	
	// ����Ŀ��ip��ַ
	W5300_WRITE_REG(Sn_DIPR(s),(((uint16)addr[0])<<8) + (uint16) addr[1]);
	W5300_WRITE_REG(Sn_DIPR2(s),(((uint16)addr[2])<<8) + (uint16) addr[3]);
	W5300_WRITE_REG(Sn_DPORTR(s),port);
	Delay_1us(10);
	wiz_write_buf(s, buf, ret);                              // ���ݿ���
	// ����
	setSn_TX_WRSR(s,ret);
	clearSUBR();
	setSn_CR(s, Sn_CR_SEND);  //ִ�и�����ſ��Զ�����Ӧ��Ŀ��IP��ַ��Ŀ��˿� �����Ƕ����Ķ˿ڲ���ȷ��ȴ���Է��ͳɹ�
	Delay_1us(10);
	while (!((isr = (getSn_IR(s))) & Sn_IR_SENDOK))            // �ȴ��������
	{
		status = getSn_SSR(s);                                // ע ---------------------------------------
		if ((status == SOCK_CLOSED) || (isr & Sn_IR_TIMEOUT)) // Sn_IR_TIMEOUT ��Ӱ��ʣ�෢�ͻ������Ĵ�С����С�� Sn_TX_FSR
		{                                                     // -----------------------------------------------
		 setSn_IR(s,Sn_IR_TIMEOUT);
		 return 0;
	  	}
	}
	setSUBR();
	setSn_IR(s, Sn_IR_SENDOK); // ���־λ
	return ret;   
}

/*******************************************************************************
* ������  : W5300_Socket_Recvfrom
* ����    : W5300�˿ڽ��պ�����UDPģʽ�ȣ�
* ����    : �˿ں�s��port,buf��addr��len        
* ���    : ��
* ����ֵ  : data_len
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
      switch (W5300_READ_REG(Sn_MR(s)) & 0x07)       // ���˿�ģʽ
      {                                            // -----------------------------
         case Sn_MR_UDP :                          // UDPģʽ 
            wiz_read_buf(s, (uint8*)head, 8);      // ��ȡPACKET-INFO
            if(*((vuint16*)MR) & MR_FS)            // С��ģʽ
            {
               head[0] = ((((head[0] << 8 ) & 0xFF00)) | ((head[0] >> 8)& 0x00FF));
               head[1] = ((((head[1] << 8 ) & 0xFF00)) | ((head[1] >> 8)& 0x00FF));
               head[2] = ((((head[2] << 8 ) & 0xFF00)) | ((head[2] >> 8)& 0x00FF));
               head[3] = ((((head[3] << 8 ) & 0xFF00)) | ((head[3] >> 8)& 0x00FF));
            }
            addr[0] = (uint8)(head[0] >> 8);       // Ŀ��IP��ַ
            addr[1] = (uint8)head[0];
            addr[2] = (uint8)(head[1]>>8);
            addr[3] = (uint8)head[1];
            *port = head[2];                       // Ŀ��˿ں�
            data_len = (uint32)head[3];            // ���ݳ���
            
            #ifdef __DEF_W5300_DBG__
               printf("UDP msg arrived:%d(0x%04x)\r\n",data_len,data_len);
               printf("source Port : %d\r\n", *port);
               printf("source IP : %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
            #endif
   
            wiz_read_buf(s, buf, data_len);        // ���ݿ���.
            break;
                                                   // -----------------------
         case Sn_MR_IPRAW :                        // IPRAW ģʽ
            wiz_read_buf(s, (uint8*)head, 6);      // ��ȡPACKET-INFO 
            if(*((vuint16*)MR) & MR_FS)            // С��ģʽ
            {
               head[0] = ((((head[0] << 8 ) & 0xFF00)) | ((head[0] >> 8)& 0x00FF));
               head[1] = ((((head[1] << 8 ) & 0xFF00)) | ((head[1] >> 8)& 0x00FF));
               head[2] = ((((head[2] << 8 ) & 0xFF00)) | ((head[2] >> 8)& 0x00FF));
            }   			
            addr[0] = (uint8)(head[0] >> 8);       // Ŀ��IP��ַ
            addr[1] = (uint8)head[0];
            addr[2] = (uint8)(head[1]>>8);
            addr[3] = (uint8)head[1];
            data_len = (uint32)head[2];            // ���ݰ�����
            
            #ifdef __DEF_W5300_DBG__
               printf("IP RAW msg arrived\r\n");
               printf("source IP : %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
            #endif
            
            wiz_read_buf(s, buf, data_len);        // ���ݿ���.
            break;                                 
                                                   // -----------------------
         case Sn_MR_MACRAW :                       // MACRAW ģʽ
            wiz_read_buf(s,(uint8*)head,2);        // ��ȡPACKET-INFO
            if(*((vuint16*)MR) & MR_FS)            // С��ģʽ
               head[0] = ((((head[0] << 8 ) & 0xFF00)) | ((head[0] >> 8)& 0x00FF));
            data_len = (uint32)head[0];            // ���ݰ�����
            wiz_read_buf(s, buf, data_len);        // ���ݿ���.
            wiz_read_buf(s,(uint8*)crc, 4);        // ��ȡCRCλ��������.

            break;
         default :
            break;
      }
      setSn_CR(s,Sn_CR_RECV);                      // ���н���
   }
   #ifdef __DEF_W5300_DBG__
      printf("recvfrom() end ..\r\n");
   #endif
   
   return data_len;   
}

/*******************************************************************************
* ������  : W5300_loopback_tcps
* ����    : TCP�ػ����Ժ�����TCP������ģʽ��
* ����    : �˿ں�s��port,buf��mode        
* ���    : ��
* ����ֵ  : ��
*******************************************************************************/ 
uint32 errcount_net = 0;
void W5300_loopback_tcps(SOCKET s, uint16 port, uint8* buf, uint16 mode)
{
	uint32 len;
	uint32 len_send;
	
	switch(getSn_SSR(s))                // ���˿�״̬
	{                                   // ------------
		case SOCK_ESTABLISHED:           // ��������
			if(getSn_IR(s) & Sn_IR_CON)   //����Ƿ�Ϊ�����ж�
			{
				setSn_IR(s,Sn_IR_CON);     //���ж�״̬
			}
			if((len=getSn_RX_RSR(s)) > 0) // ����Ƿ���յ�����
			{
				len = W5300_Socket_Recv(s,buf,len);     // ����
				len_send = S_tx_process(s,buf,len); // ����
				/*�˴���Ҫ���������Եȶ೤ʱ��ſ���ʹPACKET-INFO���ݲ������*/
				if(len != len_send)  // ��ǰ����Ŷϵ����λ����ͣ���͵�ʱ��ᵼ�¸������������INFO���ݱ���Ч���ݳ����
				{
					errcount_net ++	;
//					while(1);
				}
			}
			break;
		                   // ---------------
		case SOCK_CLOSE_WAIT:               // �����ر�
			W5300_Socket_Disconnect(s);                // �Ͽ����� 
			break;
		                   // --------------
		case SOCK_CLOSED:                   // �رն˿�
			W5300_Socket_Close(s);                        // �رն˿�
			W5300_Socket_Init(s,Sn_MR_TCP,port,mode);   //��TCPģʽ�򿪶˿� 
			break;
		                   // ------------------------------
		case SOCK_INIT:                     // �˿���TCPģʽ��
			W5300_Socket_Listen(s);                       // �򿪼���
			break;
		default:
			break;
	}
}

/*******************************************************************************
* ������  : W5300_State_management
* ����    : ����״̬����������������յ����ݺ��״̬����
* ����    : �˿ں�s��port,mode        
* ���    : ��
* ����ֵ  : ��
*******************************************************************************/ 
uint8 W5300_State_management(SOCKET s, uint16 port, uint16 mode,uint8 *JG_IP,uint16 JG_DPort)
{
	uint8 ret = 0;	//��ʾ�����״̬,1��ʾ���Է��ͽ������ݣ�0������
	
	switch(getSn_SSR(s))                // ���˿�״̬
	{                                   // ------------
		case SOCK_ESTABLISHED:           // ��������  TCPģʽ���д�״̬
			if(getSn_IR(s) & Sn_IR_CON)   //����Ƿ�Ϊ�����ж�
			{
				setSn_IR(s,Sn_IR_CON);     //���ж�״̬
			}
			ret = 1;
			break;
		case SOCK_UDP:  // UDP ģʽ��״̬
			ret = 1;
			break;			
		                   // ---------------
		case SOCK_CLOSE_WAIT:               // �����ر�
			W5300_Socket_Disconnect(s);                // �Ͽ����� 
			break;
		                   // --------------
		case SOCK_CLOSED:                   // �رն˿�
			W5300_Socket_Close(s);                        // �رն˿�
			if(mode==Sn_MR_TCP)
			{
				W5300_Socket_Init(s,Sn_MR_TCP,port,1);   //��TCPģʽ�򿪶˿� 
			}
			else if(mode==Sn_MR_UDP)
			{
				W5300_Udp_socket(s,Sn_MR_TCP,port,1);   //��UDPģʽ��
			}			
			break;
		                   // ------------------------------
		case SOCK_INIT:                     // �˿���TCPģʽ��
			W5300_Socket_Connect(s,JG_IP,JG_DPort);
			break;
		default:
			W5300_Socket_Close(s);                        // �رն˿�
			if(mode==Sn_MR_TCP)
			{
				W5300_Socket_Init(s,Sn_MR_TCP,port,1);   //��TCPģʽ�򿪶˿� 
			}
			else if(mode==Sn_MR_UDP)
			{
				W5300_Udp_socket(s,Sn_MR_TCP,port,1);    //��UDPģʽ��
			}
			break;
	}
	return ret;
}


/*******************************************************************************
* ������  : W5300_loopback_udp
* ����    : TCP�ػ����Ժ�����TCP������ģʽ��
* ����    : �˿ں�s��port,buf��mode        
* ���    : ��
* ����ֵ  : ��
*******************************************************************************/
void W5300_loopback_udp(SOCKET s, uint16 port, uint8* buf, uint16 mode)
{
	uint32 len;
	uint8 destip[4] = {192,168,1,76};//�Զ�IP��ַ
	uint16 destport = 1000; //�Զ˶˿ں�  
	switch(getSn_SSR(s))
	{ 	                                                     
		case SOCK_UDP:                                      
			if((len=getSn_RX_RSR(s)) > 0)                   //����Ƿ���յ�����
			{
				len = W5300_Socket_Recvfrom(s,buf,len,destip,&destport);  // ��Ŀ��IP��ַ��Ŀ��˿ڽ�������
				if(len != W5300_Socket_Sendto(s,buf,len,destip,destport))  // ��Ŀ���ַ��������
				{
	//			   printf("%d : Sendto Fail.len=%d,",s,len);
	//			   printf("%d.%d.%d.%d(%d)\r\n",destip[0],destip[1],destip[2],destip[3],destport);
					while(1);
				}
			}
			break;
	                                                     
		case SOCK_CLOSED:                        
			W5300_Socket_Close(s);                                       // �رն˿�
			W5300_Udp_socket(s,Sn_MR_UDP,port,mode);                  // ��UDP�ķ�ʽ�򿪶˿�
			break;
		default:
			break;
	}
}

void W5300_Udp_socket(SOCKET s, uint8 protocol, uint16 port, uint16 flag)
{
	W5300_WRITE_REG(Sn_MR(s),(uint16)(protocol | flag)); // ���ö˿ڹ���ģʽ
	if (port != 0)
		W5300_WRITE_REG(Sn_PORTR(s),port);
	else 
	{
		W5300_WRITE_REG(Sn_PORTR(s),W5300_source_port);
		W5300_source_port++;     // ��û�����ö˿ڣ�������ΪĬ��.
	}
	setSn_CR(s, Sn_CR_OPEN);      // �򿪶˿�    
	check_sendok_flag[s] = 1;     // ��ʼ�����ͱ�־.
}

/*******************************************************************************************************************
** ��������:S_rx_process				
**
** ��������:���ո�SOCKET��Ӧ���������� 
**
** ��    ��:s--��Ӧsocket 
**	         
** ��    ��:���յ������ݳ��� 
**
** ע������:�ٵ�getSn_RX_RSR�õ�ʣ�໺��ռ�Ϊ0���������ν��գ����س���Ϊ0 
**         :�۶�s������飬ע��W5300��S��������
**         :�۽��յ����ݴ洢��Rx_Buffer+s*MAX_RECSIZE ��λ��
**
** �������:20170106 
**
********************************************************************************************************************/
uint32 S_rx_process(SOCKET s)
{
	uint32 Rx_Size=0;//�������ݳ���
	uint32 fr_Size=0;//����ʣ��ռ�
	//��һ�� �õ�ʣ��ռ�
	fr_Size = getSn_RX_RSR(s);//�õ����ջ���ʣ��ռ�
	if(fr_Size>0)
	{
		Rx_Size=W5300_Socket_Recv(s,Rx_Buffer+s*MAX_RECSIZE,fr_Size);
		setSn_IR(s,Sn_IR_RECV);     //���жϻ���
	}
	return Rx_Size;
}

