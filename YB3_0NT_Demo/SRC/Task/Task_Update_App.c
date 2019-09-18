#include "Task_Update_App.h"
#include "norflash.h"
#include "PrtlProc.h"


uint8 l_u8FileBuf[UPDATE_CACHESIZE] = {0};  // 250KB  用来最后数据的打包整理
__align(4) uint8 g_u8ReadFileBuf[UPDATE_CACHESIZE] = {0};  //从norflash处读刚写进去的程序文件，用于与收到的程序文件比较
uint8 JG_1[UPDATE_MAXPACKET][1025]={0};		  // 250包  每包数据的临时存储区域存储
uint16 JG_3[UPDATE_MAXPACKET]={0};		  // 250包  每包数据的临时存储区域存储
uint8 JG_7[8]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
uint8 JG_5=0;   //是否有包缺失的标志信息

void Task_Update_App(void *tdata)
{
	uint8 err = 0;
//	uint32 i = 0;
	uint32 l_Ret = 0;
	uint32 l_u32SendLen = 0;
	uint32 l_u32TcpSendLen = 0;
	uint8  l_u8SendBuf[UPDATE_MAXPACKET] ={0};	//发送缓存
	uint8  l_u8TcpSendBuf[UPDATE_MAXPACKET]={0};	//编码以后的发送缓存
	uint32 l_u32FileSize = 0;	    // 开始更新指令中文件大小
	uint32 l_u32FilePkg = 0;	    // 开始更新指令中总包数
	uint32 l_u32RecvSize = 0;		// 接收到的文件大小
	uint32 l_u32CurSize = 0;		// 当前包的文件大小
	uint32 l_u32RecvPkg = 0;		// 接收到的总包数
	uint32 l_u32CurPkg = 0;			// 接收到的当前包序号
	uint32 tempBytes_ZYJ=0;
	uint8  wsl[2]={0};
	uint16 JG_2=0;
	uint32 JG_4=0;
//	uint8  JG_CRC=0;
	uint8  JG_JIAHE=0;
	tdata=tdata;
	//初始化状态更新
	g_Update_App_Step = UPDATE_APP_DEF;
	ZYJ_MJH   		  = 0;			//机器重启 是默认需要传输02 08数据包的
	g_bStarted        = 0;			//表示处于开始更新程序的状态中
	while(1)
	{		
		OSSemPend(g_UpdateApp,0,&err);
		if(g_Update_App_Step == START_UPDATE_APP)	 //开始更新
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
		   l_u8SendBuf[l_u32SendLen++] = (g_u8FrmNo >> 4 & 0x0f) | (g_u8FrmNo << 4 & 0xf0);	 //数据帧序列号
		   l_u8SendBuf[l_u32SendLen++] = g_u8ParseBuf[0] + 0x10; 							 //指令代码
		   l_u8SendBuf[l_u32SendLen++] = g_u8ParseBuf[1];		 							 //子指令代码
		   l_u32FileSize = g_u8ParseBuf[2] << 24 | g_u8ParseBuf[3] << 16 |					 //程序长度
					      g_u8ParseBuf[4] <<  8 | g_u8ParseBuf[5];
		   l_u32FilePkg  = g_u8ParseBuf[6] << 24 | g_u8ParseBuf[7] << 16 |					 //程序块数
						  g_u8ParseBuf[8] <<  8 | g_u8ParseBuf[9];
			//20151201 增加
			if (l_u32FileSize > UPDATE_CACHESIZE || l_u32FilePkg > UPDATE_MAXPACKET)
			{	//文件大小超过缓存大小或者文件数据包超过设置的最大数据包
				ResetSystem();  
			}
		   JG_JIAHE    =  g_u8ParseBuf[10]; 		    									 //加和校验
		   if(0xBB != g_u8ParseBuf[11]) { ResetSystem();}			    					 //特殊字节0xBB
		   l_u8SendBuf[l_u32SendLen++] = 0x00;      										 //开始更新程序成功的标志
		   l_Ret = PktCodeC(l_u8SendBuf,l_u32SendLen,l_u8TcpSendBuf,&l_u32TcpSendLen);		 //组包
		   if(l_Ret != 0) {ResetSystem();}          										 //组包失败
		   //UART1_SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen);						     //开始更新程序的回复信息包
		   SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen,g_Update_App_CH);									 //开始更新程序的回复信息包
		}
		else if(g_Update_App_Step == UPDATE_APP_DOING) //正在更新
		{
			l_u32CurSize = 	g_u8ParseBuf[2] << 24 | g_u8ParseBuf[3] << 16 |	   //当前包的大小
							g_u8ParseBuf[4] <<  8 | g_u8ParseBuf[5];
			l_u32CurPkg  = 	g_u8ParseBuf[6] << 24 | g_u8ParseBuf[7] << 16 |	   //当前包的包号
							g_u8ParseBuf[8] <<  8 | g_u8ParseBuf[9];
			//////////////////////////////////
			while ((U5LSR & 0x40) == 0);	
		    U5THR = l_u32CurPkg;

			///////////////////////////////////当前包序号从1开始
			if (l_u32CurPkg > 0 && (l_u32CurPkg <= l_u32FilePkg)&&((l_u32CurSize + l_u32RecvSize)<= l_u32FileSize))//包号和长度信息正常
			{	
				memcpy(&JG_1[l_u32CurPkg-1], &g_u8ParseBuf[10], l_u32CurSize);   //l_u32CurPkg该包数据入队列				
				JG_3[l_u32CurPkg-1]=l_u32CurSize;								   //记录l_u32CurPkg该包数据大小量
				if(JG_1[l_u32CurPkg-1][1024]!=0xAA)							   //如果该包数据是上位机误重传，我们就不做累加记录
				{
				  l_u32RecvSize += l_u32CurSize;							   //代码字节量累加
				  l_u32RecvPkg++;											   //代码包号累加
				}
				JG_1[l_u32CurPkg-1][1024]=0xAA;								   //记录l_u32CurPkg该包数据正常
			}
		}
		else if(g_Update_App_Step == COMPLETE_UPDATE_APP)		 //更新完成
		{
			 while ((U5LSR & 0x40) == 0);	
		     U5THR = 0xEE;
			 JG_5=0x00; 	                     				 //默认包号是正常的
			 l_u32SendLen = 0;
			 memset(l_u8SendBuf,0,sizeof(l_u8SendBuf));
			 l_u8SendBuf[l_u32SendLen++] = 0x19; 				 //有包缺失的命令号
			 l_u8SendBuf[l_u32SendLen++] = 0xAA; 				 //填充字节
			 for(JG_2=0;JG_2<l_u32FilePkg;JG_2++)				 //巡检看哪一包缺失
			 {
			  if(JG_1[JG_2][1024]!=0xAA)         				 //该包缺失
			  {
			  	 l_u8SendBuf[l_u32SendLen++]=0x11;				 //该包缺失的特殊信息字节
				 JG_5=0x11; 					  				 //记录这其中有部分包缺失
			  }
			  else if(JG_1[JG_2][1024]==0xAA)					 //该包不缺失
			  {
			     l_u8SendBuf[l_u32SendLen++]=0x22;				 //该包不缺失的特殊信息字节
			  }			 
			 }
			 while(l_u32SendLen<=251)							 //筹够250包的每包是否缺失的信息
			 {
			  l_u32SendLen++;					  
			 }
			 l_u8SendBuf[l_u32SendLen++] = 0xAA; 	             //这里的l_u32SendLen应该为252【填充字节】			 
			 l_Ret = PktCodeC(l_u8SendBuf,l_u32SendLen,l_u8TcpSendBuf,&l_u32TcpSendLen);
			 if(l_Ret != 0) {ResetSystem();}  
			 if(JG_5==0x11)										
			 {													 //有部分包缺失 需要去通知上位机进行重发
			   	 //UART1_SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen);
				 SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen,g_Update_App_CH);	
			 }
			 else if(l_u32RecvSize > 0 && l_u32RecvPkg > 0 && (l_u32RecvSize == l_u32FileSize)&&(l_u32RecvPkg == l_u32FilePkg))   //没有任何包缺失，长度信息也正确
			 {			
				for(JG_2=0;JG_2<l_u32FilePkg;JG_2++)    						   //把所有包进行整合
			    {
				  memcpy(&l_u8FileBuf[JG_4],&JG_1[JG_2],JG_3[JG_2]);			   //整合到l_u8FileBuf数组里面
				  JG_4+=JG_3[JG_2];
				}
				if(Checkjiahe_Remote(l_u8FileBuf,JG_4)!=JG_JIAHE)    { ResetSystem() ;}	//bin文件的加和校验失败	
				ReadNORFlash(NORDATA_NEW, 4, wsl);										//识别程序需要下载到哪个区域去
	            if(wsl[0]==0xbb&&wsl[1]==0xaa)	  //下载到第二区域
	            {
				  if((err=EraseBlock(NORDATA_ZYJ_2))==0)         { ResetSystem() ;}  //64k的Block擦除
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x10000))==0) { ResetSystem() ;} 
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x20000))==0) { ResetSystem() ;} 
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x30000))==0) { ResetSystem() ;}	
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x40000))==0) { ResetSystem() ;}	
			      WriteNORFlash(NORDATA_ZYJ_2+4,l_u32FileSize,l_u8FileBuf);  	     //写代码数据
				  OSTimeDly(100);
				  ReadNORFlash(NORDATA_ZYJ_2+4, l_u32FileSize, g_u8ReadFileBuf); //取得前20字节数据
				  if (TRUE != CompareStr(l_u8FileBuf,g_u8ReadFileBuf,l_u32FileSize)	) //读写数据不相同
				  	 ResetSystem();
		          tempBytes_ZYJ=l_u32RecvSize;
		          WordProgram(NORDATA_ZYJ_2, tempBytes_ZYJ & 0xffff);				 //代码大小量
		          WordProgram(NORDATA_ZYJ_2+2, tempBytes_ZYJ >> 16);				 //代码大小量
				  while ((U5LSR & 0x40) == 0);	U5THR = 0x1A;while ((U5LSR & 0x40) == 0);	U5THR = 0x1A;		
	            }
	            else if(wsl[0]==0xdd&&wsl[1]==0xcc)//下载到第一区域 
	            {
				  if((err=EraseBlock(NORDATA_ZYJ_1))==0)         { ResetSystem() ;}	 //64k的Block擦除	
			      if((err=EraseBlock(NORDATA_ZYJ_1+0x10000))==0) { ResetSystem() ;}	    
			      if((err=EraseBlock(NORDATA_ZYJ_1+0x20000))==0) { ResetSystem() ;}	    
			      if((err=EraseBlock(NORDATA_ZYJ_1+0x30000))==0) { ResetSystem() ;}		
			      if((err=EraseBlock(NORDATA_ZYJ_1+0x40000))==0) { ResetSystem() ;}		
			      WriteNORFlash(NORDATA_ZYJ_1+4,l_u32FileSize,l_u8FileBuf);  		 //写代码数据
				  OSTimeDly(100);
				  ReadNORFlash(NORDATA_ZYJ_1+4, l_u32FileSize, g_u8ReadFileBuf); //取得前20字节数据
				  if (TRUE != CompareStr(l_u8FileBuf,g_u8ReadFileBuf,l_u32FileSize)	) //读写数据不相同
				  	 ResetSystem();
		          tempBytes_ZYJ=l_u32RecvSize;
		          WordProgram(NORDATA_ZYJ_1, tempBytes_ZYJ & 0xffff);	             //代码大小量
		          WordProgram(NORDATA_ZYJ_1+2, tempBytes_ZYJ >> 16);	             //代码大小量
				  while ((U5LSR & 0x40) == 0);	U5THR = 0x2A;while ((U5LSR & 0x40) == 0);	U5THR = 0x2A;//调试信息	
	            }
	            else							   //默认下载到第二区域
	            {
				  if((err=EraseBlock(NORDATA_ZYJ_2))==0)         { ResetSystem() ;}	 //64k的Block擦除
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x10000))==0) { ResetSystem() ;} 
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x20000))==0) { ResetSystem() ;}
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x30000))==0) { ResetSystem() ;}
			      if((err=EraseBlock(NORDATA_ZYJ_2+0x40000))==0) { ResetSystem() ;}	
			      WriteNORFlash(NORDATA_ZYJ_2+4,l_u32FileSize,l_u8FileBuf);  		 //写代码数据
				  OSTimeDly(100);
				  ReadNORFlash(NORDATA_ZYJ_2+4, l_u32FileSize, g_u8ReadFileBuf); //取得字节数据
				  if (TRUE != CompareStr(l_u8FileBuf,g_u8ReadFileBuf,l_u32FileSize)	) //读写数据不相同
				  	 ResetSystem();
		          tempBytes_ZYJ=l_u32RecvSize;
		          WordProgram(NORDATA_ZYJ_2, tempBytes_ZYJ & 0xffff);	             //代码大小量
		          WordProgram(NORDATA_ZYJ_2+2, tempBytes_ZYJ >> 16);	             //代码大小量
				  while ((U5LSR & 0x40) == 0);	U5THR = 0x3A;while ((U5LSR & 0x40) == 0);	U5THR = 0x3A;//调试信息	
	            }				
                if(wsl[0]==0xbb&&wsl[1]==0xaa)	 									 //成功更新到第二区域
	            {																	 
				 if((err=EraseSector(NORDATA_NEW))==0)         { ResetSystem() ;}
				 if((err=WordProgram(NORDATA_NEW, 0xccdd))==0) { ResetSystem() ;}
	            }
	            else if(wsl[0]==0xdd&&wsl[1]==0xcc) 								 //成功更新到第一区域
	            {
				  if((err=EraseSector(NORDATA_NEW))==0)         { ResetSystem() ;}
				  if((err=WordProgram(NORDATA_NEW, 0xaabb))==0) { ResetSystem() ;}	
	            }
	            else							
	            {																	 //默认成功更新到第二区域
				  if((err=EraseSector(NORDATA_NEW))==0)         { ResetSystem() ;}
				  if((err=WordProgram(NORDATA_NEW, 0xccdd))==0) { ResetSystem() ;}				  
	            }
				BeepON();
                OSTimeDly(500);OSTimeDly(500);
                BeepOFF();
                /////////////////////////////////////////////////////////////
			    l_u32SendLen = 0;
			    memset(l_u8SendBuf,0,sizeof(l_u8SendBuf));
			    l_u8SendBuf[l_u32SendLen++] = (g_u8FrmNo >> 4 & 0x0f) | (g_u8FrmNo << 4 & 0xf0);//数据帧序列号
			    l_u8SendBuf[l_u32SendLen++] = g_u8ParseBuf[0] + 0x10; 						   //指令代码
			    l_u8SendBuf[l_u32SendLen++] = g_u8ParseBuf[1];								   //子指令代码
			    l_u8SendBuf[l_u32SendLen++] = 0x00;											   //更新程序成功的返回标志
			    l_Ret = PktCodeC(l_u8SendBuf,l_u32SendLen,l_u8TcpSendBuf,&l_u32TcpSendLen);	
			    if(l_Ret != 0) {ResetSystem();}  
			    //UART1_SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen);					    //更新成功的返回信息包
				SendBuf_full(l_u8TcpSendBuf,l_u32TcpSendLen,g_Update_App_CH);							//更新成功的返回信息包
				g_bStarted = 0;
				g_Update_App_CH=CH_NOCH;								  
			    OSTimeDly(500);OSTimeDly(500);
			    ResetSystem() ;																   //更新成功后重启
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

uint8 Checkjiahe_Remote(uint8 *p_pu8Ptr_a,uint32 p_u16Len_a) //加和校验
{
  uint32 l_u32I_a=0;
  uint8  CRC_end_a=0;
  for (l_u32I_a = 0; l_u32I_a < p_u16Len_a; l_u32I_a++)
  {
  	 CRC_end_a |= *(p_pu8Ptr_a+l_u32I_a);
  }
  return CRC_end_a;
}

//比较两个字符串是否相同
uint8 CompareStr(uint8 *p_u8Str1, uint8 *p_u8Str2, uint32 u32StrLen)
{
	uint8   Ret = FALSE; //不同
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


