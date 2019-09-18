#include "Task_Update_App.h"
#include "norflash.h"
#include "PrtlProc.h"


uint8 l_u8FileBuf[UPDATE_CACHESIZE] = {0};  // 250KB  ����������ݵĴ������
__align(4) uint8 g_u8ReadFileBuf[UPDATE_CACHESIZE] = {0};  //��norflash������д��ȥ�ĳ����ļ����������յ��ĳ����ļ��Ƚ�
uint8 JG_1[UPDATE_MAXPACKET][1025]={0};		  // 250��  ÿ�����ݵ���ʱ�洢����洢
uint16 JG_3[UPDATE_MAXPACKET]={0};		  // 250��  ÿ�����ݵ���ʱ�洢����洢
uint8 JG_7[8]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
uint8 JG_5=0;   //�Ƿ��а�ȱʧ�ı�־��Ϣ

void Task_Update_App(void *tdata)
{
	uint8 err = 0;
//	uint32 i = 0;
	uint32 l_Ret = 0;
	uint32 l_u32SendLen = 0;
	uint32 l_u32TcpSendLen = 0;
	uint8  l_u8SendBuf[UPDATE_MAXPACKET] ={0};	//���ͻ���
	uint8  l_u8TcpSendBuf[UPDATE_MAXPACKET]={0};	//�����Ժ�ķ��ͻ���
	uint32 l_u32FileSize = 0;	    // ��ʼ����ָ�����ļ���С
	uint32 l_u32FilePkg = 0;	    // ��ʼ����ָ�����ܰ���
	uint32 l_u32RecvSize = 0;		// ���յ����ļ���С
	uint32 l_u32CurSize = 0;		// ��ǰ�����ļ���С
	uint32 l_u32RecvPkg = 0;		// ���յ����ܰ���
	uint32 l_u32CurPkg = 0;			// ���յ��ĵ�ǰ�����
	uint32 tempBytes_ZYJ=0;
	uint8  wsl[2]={0};
	uint16 JG_2=0;
	uint32 JG_4=0;
//	uint8  JG_CRC=0;
	uint8  JG_JIAHE=0;
	tdata=tdata;
	//��ʼ��״̬����
	g_Update_App_Step = UPDATE_APP_DEF;
	ZYJ_MJH   		  = 0;			//�������� ��Ĭ����Ҫ����02 08���ݰ���
	g_bStarted        = 0;			//��ʾ���ڿ�ʼ���³����״̬��
	while(1)
	{		
		OSSemPend(g_UpdateApp,0,&err);
		if(g_Update_App_Step == START_UPDATE_APP)	 //��ʼ����
		{
           ////////////////////////////////////////////////////////////////
		   err = 0;
//  	       i = 0;
           l_Ret = 0;
	       l_u32SendLen = 0;
	       l_u32TcpSendLen = 0;
           memset(l_u8SendBuf,0,sizeof(l_u8SendBuf));
           memset(l_u8TcpSendBuf,0,sizeof(l_u8TcpSendBuf));
	       l_u32FileSize = 0;	
	       l_u32FilePkg = 0;	
	       l_u32RecvSize = 0;	
	       l_u32CurSize = 0;	
	       l_u32RecvPkg = 0;		
	       l_u32CurPkg = 0;		
           tempBytes_ZYJ=0;
           memset(wsl,0,sizeof(wsl));
	       JG_2=0;
	       JG_4=0;
//           JG_CRC=0;
	       JG_JIAHE=0;
//	       MIN_Remote=0;		  
           memset(JG_3,0,sizeof(JG_3));
		   memset(JG_1,0,sizeof(JG_1));
		   ////////////////////////////////////////////////////////////////

		   l_u32SendLen = 0;
		   memset(l_u8SendBuf,0,sizeof(l_u8SendBuf));
		   l_u8SendBuf[l_u32SendLen++] = (g_u8FrmNo >> 4 & 0x0f) | (g_u8FrmNo << 4 & 0xf0);	 //����֡���к�
		   l_u8SendBuf[l_u32SendLen++] = g_u8ParseBuf[0] + 0x10; 							 //ָ�����
		   l_u8SendBuf[l_u32SendLen++] = g_u8ParseBuf[1];		 							 //��ָ�����
		   l_u32FileSize = g_u8ParseBuf[2] << 24 | g_u8ParseBuf[3] << 16 |					 //���򳤶�
					      g_u8ParseBuf[4] <<  8 | g_u8ParseBuf[5];
		   l_u32FilePkg  = g_u8ParseBuf[6] << 24 | g_u8ParseBuf[7] << 16 |					 //�������
						  g_u8ParseBuf[8] <<  8 | g_u8ParseBuf[9];
			//20151201 ����
			if (l_u32FileSize > UPDATE_CACHESIZE || l_u32FilePkg > UPDATE_MAXPACKET)
			{	//�ļ���С���������С�����ļ����ݰ��������õ�������ݰ�
				ResetSystem();  
			}
		   JG_JIAHE    =  g_u8ParseBuf[10]; 		    									 //�Ӻ�У��
		   if(0xBB != g_u8ParseBuf[11]) { ResetSystem();}			    					 //�����ֽ�0xBB
		   l_u8SendBuf[l_u32SendLen++] = 0x00;      										 //��ʼ���³���ɹ��ı�־
		   l_Ret = PktCodeC(l_u8SendBuf,l_u32SendLen,l_u8TcpSendBuf,&l_u32TcpSendLen);		 //���
		   if(l_Ret != 0) {ResetSystem();}          										 //���ʧ��
		   //UART1_SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen);						     //��ʼ���³���Ļظ���Ϣ��
		   SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen,g_Update_App_CH);									 //��ʼ���³���Ļظ���Ϣ��
		}
		else if(g_Update_App_Step == UPDATE_APP_DOING) //���ڸ���
		{
			l_u32CurSize = 	g_u8ParseBuf[2] << 24 | g_u8ParseBuf[3] << 16 |	   //��ǰ���Ĵ�С
							g_u8ParseBuf[4] <<  8 | g_u8ParseBuf[5];
			l_u32CurPkg  = 	g_u8ParseBuf[6] << 24 | g_u8ParseBuf[7] << 16 |	   //��ǰ���İ���
							g_u8ParseBuf[8] <<  8 | g_u8ParseBuf[9];
			//////////////////////////////////
			while ((U5LSR & 0x40) == 0);	
		    U5THR = l_u32CurPkg;

			///////////////////////////////////��ǰ����Ŵ�1��ʼ
			if (l_u32CurPkg > 0 && (l_u32CurPkg <= l_u32FilePkg)&&((l_u32CurSize + l_u32RecvSize)<= l_u32FileSize))//���źͳ�����Ϣ����
			{	
				memcpy(&JG_1[l_u32CurPkg-1], &g_u8ParseBuf[10], l_u32CurSize);   //l_u32CurPkg�ð����������				
				JG_3[l_u32CurPkg-1]=l_u32CurSize;								   //��¼l_u32CurPkg�ð����ݴ�С��
				if(JG_1[l_u32CurPkg-1][1024]!=0xAA)							   //����ð���������λ�����ش������ǾͲ����ۼӼ�¼
				{
				  l_u32RecvSize += l_u32CurSize;							   //�����ֽ����ۼ�
				  l_u32RecvPkg++;											   //��������ۼ�
				}
				JG_1[l_u32CurPkg-1][1024]=0xAA;								   //��¼l_u32CurPkg�ð���������
			}
		}
		else if(g_Update_App_Step == COMPLETE_UPDATE_APP)		 //�������
		{
			 while ((U5LSR & 0x40) == 0);	
		     U5THR = 0xEE;
			 JG_5=0x00; 	                     				 //Ĭ�ϰ�����������
			 l_u32SendLen = 0;
			 memset(l_u8SendBuf,0,sizeof(l_u8SendBuf));
			 l_u8SendBuf[l_u32SendLen++] = 0x19; 				 //�а�ȱʧ�������
			 l_u8SendBuf[l_u32SendLen++] = 0xAA; 				 //����ֽ�
			 for(JG_2=0;JG_2<l_u32FilePkg;JG_2++)				 //Ѳ�쿴��һ��ȱʧ
			 {
			  if(JG_1[JG_2][1024]!=0xAA)         				 //�ð�ȱʧ
			  {
			  	 l_u8SendBuf[l_u32SendLen++]=0x11;				 //�ð�ȱʧ��������Ϣ�ֽ�
				 JG_5=0x11; 					  				 //��¼�������в��ְ�ȱʧ
			  }
			  else if(JG_1[JG_2][1024]==0xAA)					 //�ð���ȱʧ
			  {
			     l_u8SendBuf[l_u32SendLen++]=0x22;				 //�ð���ȱʧ��������Ϣ�ֽ�
			  }			 
			 }
			 while(l_u32SendLen<=251)							 //�ﹻ250����ÿ���Ƿ�ȱʧ����Ϣ
			 {
			  l_u32SendLen++;					  
			 }
			 l_u8SendBuf[l_u32SendLen++] = 0xAA; 	             //�����l_u32SendLenӦ��Ϊ252������ֽڡ�			 
			 l_Ret = PktCodeC(l_u8SendBuf,l_u32SendLen,l_u8TcpSendBuf,&l_u32TcpSendLen);
			 if(l_Ret != 0) {ResetSystem();}  
			 if(JG_5==0x11)										
			 {													 //�в��ְ�ȱʧ ��Ҫȥ֪ͨ��λ�������ط�
			   	 //UART1_SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen);
				 SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen,g_Update_App_CH);	
			 }
			 else if(l_u32RecvSize > 0 && l_u32RecvPkg > 0 && (l_u32RecvSize == l_u32FileSize)&&(l_u32RecvPkg == l_u32FilePkg))   //û���κΰ�ȱʧ��������ϢҲ��ȷ
			 {			
				for(JG_2=0;JG_2<l_u32FilePkg;JG_2++)    						   //�����а���������
			    {
				  memcpy(&l_u8FileBuf[JG_4],&JG_1[JG_2],JG_3[JG_2]);			   //���ϵ�l_u8FileBuf��������
				  JG_4+=JG_3[JG_2];
				}
				if(Checkjiahe_Remote(l_u8FileBuf,JG_4)!=JG_JIAHE)    { ResetSystem() ;}	//bin�ļ��ļӺ�У��ʧ��	
				ReadNORFlash(NORDATA_NEW, 4, wsl);										//ʶ�������Ҫ���ص��ĸ�����ȥ
	            if(wsl[0]==0xbb&&wsl[1]==0xaa)	  //���ص��ڶ�����
	            {
				  if((err=EraseBlock(NORDATA_ZYJ_2))==0)         { ResetSystem() ;}  //64k��Block����
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x10000))==0) { ResetSystem() ;} 
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x20000))==0) { ResetSystem() ;} 
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x30000))==0) { ResetSystem() ;}	
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x40000))==0) { ResetSystem() ;}	
			      WriteNORFlash(NORDATA_ZYJ_2+4,l_u32FileSize,l_u8FileBuf);  	     //д��������
				  OSTimeDly(100);
				  ReadNORFlash(NORDATA_ZYJ_2+4, l_u32FileSize, g_u8ReadFileBuf); //ȡ��ǰ20�ֽ�����
				  if (TRUE != CompareStr(l_u8FileBuf,g_u8ReadFileBuf,l_u32FileSize)	) //��д���ݲ���ͬ
				  	 ResetSystem();
		          tempBytes_ZYJ=l_u32RecvSize;
		          WordProgram(NORDATA_ZYJ_2, tempBytes_ZYJ & 0xffff);				 //�����С��
		          WordProgram(NORDATA_ZYJ_2+2, tempBytes_ZYJ >> 16);				 //�����С��
				  while ((U5LSR & 0x40) == 0);	U5THR = 0x1A;while ((U5LSR & 0x40) == 0);	U5THR = 0x1A;		
	            }
	            else if(wsl[0]==0xdd&&wsl[1]==0xcc)//���ص���һ���� 
	            {
				  if((err=EraseBlock(NORDATA_ZYJ_1))==0)         { ResetSystem() ;}	 //64k��Block����	
			      if((err=EraseBlock(NORDATA_ZYJ_1+0x10000))==0) { ResetSystem() ;}	    
			      if((err=EraseBlock(NORDATA_ZYJ_1+0x20000))==0) { ResetSystem() ;}	    
			      if((err=EraseBlock(NORDATA_ZYJ_1+0x30000))==0) { ResetSystem() ;}		
			      if((err=EraseBlock(NORDATA_ZYJ_1+0x40000))==0) { ResetSystem() ;}		
			      WriteNORFlash(NORDATA_ZYJ_1+4,l_u32FileSize,l_u8FileBuf);  		 //д��������
				  OSTimeDly(100);
				  ReadNORFlash(NORDATA_ZYJ_1+4, l_u32FileSize, g_u8ReadFileBuf); //ȡ��ǰ20�ֽ�����
				  if (TRUE != CompareStr(l_u8FileBuf,g_u8ReadFileBuf,l_u32FileSize)	) //��д���ݲ���ͬ
				  	 ResetSystem();
		          tempBytes_ZYJ=l_u32RecvSize;
		          WordProgram(NORDATA_ZYJ_1, tempBytes_ZYJ & 0xffff);	             //�����С��
		          WordProgram(NORDATA_ZYJ_1+2, tempBytes_ZYJ >> 16);	             //�����С��
				  while ((U5LSR & 0x40) == 0);	U5THR = 0x2A;while ((U5LSR & 0x40) == 0);	U5THR = 0x2A;//������Ϣ	
	            }
	            else							   //Ĭ�����ص��ڶ�����
	            {
				  if((err=EraseBlock(NORDATA_ZYJ_2))==0)         { ResetSystem() ;}	 //64k��Block����
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x10000))==0) { ResetSystem() ;} 
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x20000))==0) { ResetSystem() ;}
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x30000))==0) { ResetSystem() ;}
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x40000))==0) { ResetSystem() ;}	
			      WriteNORFlash(NORDATA_ZYJ_2+4,l_u32FileSize,l_u8FileBuf);  		 //д��������
				  OSTimeDly(100);
				  ReadNORFlash(NORDATA_ZYJ_2+4, l_u32FileSize, g_u8ReadFileBuf); //ȡ���ֽ�����
				  if (TRUE != CompareStr(l_u8FileBuf,g_u8ReadFileBuf,l_u32FileSize)	) //��д���ݲ���ͬ
				  	 ResetSystem();
		          tempBytes_ZYJ=l_u32RecvSize;
		          WordProgram(NORDATA_ZYJ_2, tempBytes_ZYJ & 0xffff);	             //�����С��
		          WordProgram(NORDATA_ZYJ_2+2, tempBytes_ZYJ >> 16);	             //�����С��
				  while ((U5LSR & 0x40) == 0);	U5THR = 0x3A;while ((U5LSR & 0x40) == 0);	U5THR = 0x3A;//������Ϣ	
	            }				
                if(wsl[0]==0xbb&&wsl[1]==0xaa)	 									 //�ɹ����µ��ڶ�����
	            {																	 
				 if((err=EraseSector(NORDATA_NEW))==0)         { ResetSystem() ;}
				 if((err=WordProgram(NORDATA_NEW, 0xccdd))==0) { ResetSystem() ;}
	            }
	            else if(wsl[0]==0xdd&&wsl[1]==0xcc) 								 //�ɹ����µ���һ����
	            {
				  if((err=EraseSector(NORDATA_NEW))==0)         { ResetSystem() ;}
				  if((err=WordProgram(NORDATA_NEW, 0xaabb))==0) { ResetSystem() ;}	
	            }
	            else							
	            {																	 //Ĭ�ϳɹ����µ��ڶ�����
				  if((err=EraseSector(NORDATA_NEW))==0)         { ResetSystem() ;}
				  if((err=WordProgram(NORDATA_NEW, 0xccdd))==0) { ResetSystem() ;}				  
	            }
				BeepON();
                OSTimeDly(500);OSTimeDly(500);
                BeepOFF();
                /////////////////////////////////////////////////////////////
			    l_u32SendLen = 0;
			    memset(l_u8SendBuf,0,sizeof(l_u8SendBuf));
			    l_u8SendBuf[l_u32SendLen++] = (g_u8FrmNo >> 4 & 0x0f) | (g_u8FrmNo << 4 & 0xf0);//����֡���к�
			    l_u8SendBuf[l_u32SendLen++] = g_u8ParseBuf[0] + 0x10; 						   //ָ�����
			    l_u8SendBuf[l_u32SendLen++] = g_u8ParseBuf[1];								   //��ָ�����
			    l_u8SendBuf[l_u32SendLen++] = 0x00;											   //���³���ɹ��ķ��ر�־
			    l_Ret = PktCodeC(l_u8SendBuf,l_u32SendLen,l_u8TcpSendBuf,&l_u32TcpSendLen);	
			    if(l_Ret != 0) {ResetSystem();}  
			    //UART1_SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen);					    //���³ɹ��ķ�����Ϣ��
				SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen,g_Update_App_CH);							//���³ɹ��ķ�����Ϣ��
				g_bStarted = 0;
				g_Update_App_CH=CH_NOCH;								  
			    OSTimeDly(500);OSTimeDly(500);
			    ResetSystem() ;																   //���³ɹ�������
			 }
			 else
			 {
			 	//UART1_SendBuf_full(JG_7,  8);
				SendBuf_full(JG_7,  8,g_Update_App_CH);
			 }
		}
	}
}

uint32 PktCodeC(uint8 *ibuf, uint32 ilen, uint8 *obuf, uint32 *olen)
{
	uint32 i = 0;
	uint32 slen = 0;
	uint8 chk = 0;	
	if (ilen > 2048)
	{
		return 1;
	}	
	obuf[slen++] = 0xff;
	obuf[slen++] = 0xff;
	chk = 0;
	for (i = 0 ; i < ilen; i++)
	{
		chk ^= ibuf[i];
		
		if(ibuf[i] == 0xff)	    
		{
			obuf[slen++] = 0xfe;
			obuf[slen++] = 0x01;
		}
		else if(ibuf[i] == 0xfe) 
		{
			obuf[slen++] = 0xfe;
			obuf[slen++] = 0x00;
		}
		else
		{
			obuf[slen++] = ibuf[i];
		}
	}

	if(chk == 0xff)
	{
		obuf[slen++] = 0xfe;
		obuf[slen++] = 0x01;
	}
	else if(chk == 0xfe)
	{
		obuf[slen++] = 0xfe;
		obuf[slen++] = 0x00;
	}
	else
	{
		obuf[slen++] = chk;
	}		
	obuf[slen++] = 0xff;	
    *olen = slen;	 
	return 0;
}

uint32 PktDecode(uint8 *ibuf, uint32 ilen, uint8 *r_u8FrmNo, uint8 *obuf, uint32 *olen)
{
	uint32 i = 0;
	uint32 j = 0;
	uint8 l_u8Chk = 0;
	if ((ilen < 5) || (ilen > 2048))
	{
		return 1;
	} 
	if ((0xFF != ibuf[0]) || (0xFF != ibuf[1]) || (0xFF != ibuf[ilen - 1]))
	{
		return 1; 
	}
	i = 2;
	*r_u8FrmNo = ibuf[i++];  	
	do 
	{
		if (0xff == ibuf[i]) 
		{ 
			break;
		} 
		else 
		{
			obuf[j] = ibuf[i];
			if (0xFE == ibuf[i]) 		   
			{
				obuf[j] |= ibuf[i + 1];
				i++;	
			}
			l_u8Chk ^= obuf[j];
			i++;
			j++;
		}
	} while (1);
	l_u8Chk ^= *r_u8FrmNo;	      
	if ((j > 1) && (0 == l_u8Chk))
	{
		*olen = j - 1;            
		return 0;
	}
	else						  
	{
		*olen = 0;
		return 1; 
	}
}

uint8 Checkjiahe_Remote(uint8 *p_pu8Ptr_a,uint32 p_u16Len_a) //�Ӻ�У��
{
  uint32 l_u32I_a=0;
  uint8  CRC_end_a=0;
  for (l_u32I_a = 0; l_u32I_a < p_u16Len_a; l_u32I_a++)
  {
  	 CRC_end_a |= *(p_pu8Ptr_a+l_u32I_a);
  }
  return CRC_end_a;
}

//�Ƚ������ַ����Ƿ���ͬ
uint8 CompareStr(uint8 *p_u8Str1, uint8 *p_u8Str2, uint32 u32StrLen)
{
	uint8   Ret = FALSE; //��ͬ
	uint32  i  = 0;

	if ( u32StrLen > UPDATE_CACHESIZE)
		return Ret;
	for (; i < u32StrLen; i++)
	{
		if (p_u8Str1[i] != p_u8Str2[i])
			return Ret;
	}

	Ret = TRUE;

	return Ret;
}


