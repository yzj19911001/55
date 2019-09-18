#include "Task_Uart5Rec.h"
#include "Task_Update_App.h"
#include "PrtlProc.h"
#include "PCF8563.h"

void Task_Uart5Rec(void *tdata)
{
	uint8 err=0;
	 while(1)
	 {
		OSSemPend(g_Uart5Recv,0,&err);
		if(err==OS_NO_ERR)
		{
			RecCom5Data();
		}
		else
		{
			OSTimeDly(200);
		}		 
	 }	
}


uint8 RecCom5Data(void)
{
	uint8 U5RecBuff[UARTRCVBUFSIZE]={0};
	uint16 index = 0;
	uint16 temp_head = 0;  //处理数据的队列头
	uint16 temp_tail = 0;  //处理数据的队列尾
	uint16 temp_length = 0;  //处理数据的长度
	uint16 frame_length = 0;  //待处理数据帧长
	uart5_timeout = 0;	 //串口接收超时控制
	temp_head =  g_UART5_Proc_Index;
	temp_tail =  g_UART5_Recv_Index;
	if(temp_head != temp_tail )		 //缓存队列不为空
	{
		if(temp_head < temp_tail)	//求待处理缓存数据长度
		{
		 	temp_length =  temp_tail - temp_head;
		}
		else
		{
			temp_length = UARTRCVBUFSIZE-(temp_head - temp_tail);
		}

		while( ((temp_head+1)&(UARTRCVBUFSIZE-1)) != temp_tail  && temp_length )		 //遍历每个帧头 确定是否为有效帧
		{
		  	if(g_u8U5RcvBuf[temp_head] == 0xFF && (g_u8U5RcvBuf[(temp_head+1)&(UARTRCVBUFSIZE-1)] == 0xFF) )	  //判断帧头
			{
			   if( temp_length >= MIN_FRAME_LENGTH )  //待处理缓存长度大于最小包长7   能使用 g_u8U5RcvBuf[ (temp_head+3)%1024 ]
			   {
			   		if( (g_u8U5RcvBuf[(temp_head + 2)&(UARTRCVBUFSIZE-1) ] == 0x80 || g_u8U5RcvBuf[(temp_head + 2)&(UARTRCVBUFSIZE-1) ] == 0x81 )&& g_u8U5RcvBuf[(temp_head + 3)&(UARTRCVBUFSIZE-1) ] == 0x3B)
					{ //寻找结束FF
						if( temp_length > 15 )	//寻找结束FF,长度可以包含结束的FF
						{
							frame_length = 15;
							while( g_u8U5RcvBuf[((temp_head + frame_length)&(UARTRCVBUFSIZE-1)) ] != 0xFF &&  ((temp_head + frame_length)&(UARTRCVBUFSIZE-1)) != temp_tail )  //寻找后续的第一个FF
								  frame_length++;
						}
						else   //寻找结束FF,长度无法包含结束的FF
						{
							 	break;	//等待下一次接收完成
						}
						if( g_u8U5RcvBuf[(temp_head + frame_length)&(UARTRCVBUFSIZE-1) ] == 0xFF  && temp_length < MAX_FRAME_LENGTH_FF)	//寻找到FF
						{
							frame_length = frame_length +1;
								memset(U5RecBuff , 0 , UARTRCVBUFSIZE);
							for(index = 0; index <frame_length ;index++ )//适应后续的解析函数，需要提取出有效数据
							{
								U5RecBuff[index] =  g_u8U5RcvBuf[ (temp_head+index)% UARTRCVBUFSIZE ]	;
							}
						}
						else if	( g_u8U5RcvBuf[(temp_head + frame_length)&(UARTRCVBUFSIZE-1) ] != 0xFF  && temp_length < MAX_FRAME_LENGTH_FF )
							frame_length = temp_length+1 ;	//等待下一次接收完成
						else
							frame_length = 	MAX_FRAME_LENGTH_FF + 1;	
					}
					else if(g_u8U5RcvBuf[(temp_head + 2)&(UARTRCVBUFSIZE-1) ] == 0x82 && g_u8U5RcvBuf[(temp_head + 3)&(UARTRCVBUFSIZE-1) ] == 0x3B)
					{
						frame_length =  8;	//数据帧长度
						memset(U5RecBuff , 0 , UARTRCVBUFSIZE);
						for(index = 0; index <frame_length;index++ )//适应后续的解析函数，需要提取出有效数据
						{
							U5RecBuff[index] =  g_u8U5RcvBuf[ (temp_head+index)% UARTRCVBUFSIZE ]	;
						}
						if( U5RecBuff[7] != 0xFF )  //无效帧
						{
							frame_length = 	MAX_FRAME_LENGTH_FF + 1;//舍弃当前帧头
						}
					}
					else if(  g_u8U5RcvBuf[(temp_head+2)&(UARTRCVBUFSIZE-1)] == 0x02 &&  g_u8U5RcvBuf[(temp_head+3)&(UARTRCVBUFSIZE-1)] == 0x02 && g_u8U5RcvBuf[(temp_head+4)&(UARTRCVBUFSIZE-1)] == 0x02 && g_u8U5RcvBuf[(temp_head+5)&(UARTRCVBUFSIZE-1)] == 0x02)
					{	//激光透传数据
						if( ( temp_length >= MIN_FRAME_LENGTH_TC ))						
						{
							frame_length =(g_u8U5RcvBuf[ (temp_head+6)&(UARTRCVBUFSIZE-1) ]<<24) + (g_u8U5RcvBuf[ (temp_head+7)&(UARTRCVBUFSIZE-1) ]<<16) +(g_u8U5RcvBuf[ (temp_head+8)&(UARTRCVBUFSIZE-1) ]<<8) + g_u8U5RcvBuf[(temp_head+9)&(UARTRCVBUFSIZE-1)] +18 ; //求数据包长度，包含头尾校验
						   if(temp_length >= (frame_length) && frame_length <= MAX_FRAME_LENGTH_TC )	 //	 是否包含完整一包  frame_length
						   {
						   		if( g_u8U5RcvBuf[(temp_head+frame_length-1)&(UARTRCVBUFSIZE-1)] == 0xEE )	  //判断帧尾校验
								{	
									memset(U5RecBuff , 0 , UARTRCVBUFSIZE);
									for(index = 0; index <frame_length + 6;index++ )//适应后续的解析函数，需要提取出有效数据
									{
										U5RecBuff[index] =  g_u8U5RcvBuf[ (temp_head+index)% UARTRCVBUFSIZE ]	;
									}
									
							   		temp_head = (frame_length + temp_head )&(UARTRCVBUFSIZE-1);  //确定新的处理起始点;
									temp_length =  temp_length -frame_length ;
									g_UART5_Proc_Index = temp_head;		//队列头赋值
									g_LaserPass.PassCH=CH_COM1;//透传通道为COM5
									PassThroughToJG(U5RecBuff+2);			   //处理数据
							   	}
								else	//不是完整的一包 继续寻找帧头
								{
									temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //继续检查下一字节；
					 				temp_length--;
								}	
						   }
						   else		
						   {
						   		if(frame_length > MAX_FRAME_LENGTH_TC)
								{
									temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //包长过长，舍弃，继续检查下一字节；
					 				temp_length--;	
								}
								else  //剩下的数据不足当前包长度且包长有效		 考虑加入时间限定，不能无限等待
									break;	//等待下一次接收完成	
						   }
						}
						else
						{
								break;		//等待下一次接收完成		
						}
					}
					else
					{	
						//正常交互数据
					   if(g_u8U5RcvBuf[ (temp_head+2)&(UARTRCVBUFSIZE-1) ]==0xFF)
					   {
							temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //多个FF连着 去掉前面的
							temp_length--;
							continue;
					   }
					   frame_length = (g_u8U5RcvBuf[ (temp_head+3)&(UARTRCVBUFSIZE-1) ]<<8) + g_u8U5RcvBuf[(temp_head+4)&(UARTRCVBUFSIZE-1)]; //求数据包长度，包含头尾校验
					   if(temp_length >= (frame_length) && frame_length <= MAX_FRAME_LENGTH )	 //	 是否包含完整一包  frame_length
					   {
					   		if( CheckCrc(g_u8U5RcvBuf+temp_head,frame_length-2)== 1 )	  //判断帧尾校验
							{	
								memset(U5RecBuff , 0 , UARTRCVBUFSIZE);
								for(index = 0; index <frame_length + 6;index++ )//适应后续的解析函数，需要提取出有效数据
								{
									U5RecBuff[index] =  g_u8U5RcvBuf[ (temp_head+index)&(UARTRCVBUFSIZE-1) ]	;
								}
								AnalyzeCom5Data(U5RecBuff, frame_length);			   //处理数据
						   		temp_head = (frame_length + temp_head )&(UARTRCVBUFSIZE-1);  //确定新的处理起始点;
								temp_length =  temp_length -frame_length ;
						   	}
							else	//不是完整的一包 继续寻找帧头
							{
								temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //继续检查下一字节；
				 				temp_length--;
							}	
						  
					   }
					   else		
					   {
					   		if(frame_length > MAX_FRAME_LENGTH)
							{
								temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //包长过长，舍弃，继续检查下一字节；
				 				temp_length--;	
							}
							else  //剩下的数据不足当前包长度且包长有效		 考虑加入时间限定，不能无限等待
								break;	//等待下一次接收完成	
					   }
					}

				   if(temp_length >= frame_length && frame_length <= MAX_FRAME_LENGTH_FF )	 //	 是否包含完整一包  frame_length
				   {
				   		if (!PktDecode(U5RecBuff ,frame_length,&g_u8FrmNo,g_u8ParseBuf,&g_u32ParseLen  ))//校验成功
						{
							temp_head = (frame_length + temp_head)&(UARTRCVBUFSIZE-1);  //确定新的处理起始点; //占用的缓存较大，考虑先清除这部分缓存
							temp_length =  temp_length - frame_length ;
							g_UART5_Proc_Index = temp_head;		//队列头赋值
							switch( U5RecBuff[4] )
							{
								case 0x11:
										GetRTCTime(&g_sstCurTime_Remote);		 //获取开始更新程序的起始时间
   										MIN_Remote=g_sstCurTime_Remote.u8Minute; 
										g_Update_App_Step = START_UPDATE_APP;
										g_Update_App_CH=CH_COM1;
										g_bStarted = 1;	//uint8 型标志变量
										ZYJ_MJH=0x01;
										OSSemPost(g_UpdateApp);
								break;
								case 0x12:
									if(g_bStarted == 1)	
									{
										 g_Update_App_Step = UPDATE_APP_DOING;
										 ZYJ_MJH=0x01;
										 OSSemPost(g_UpdateApp);
									}
								break;
								case 0x13:
									if(g_Update_App_Step == UPDATE_APP_DOING)
									{
										g_Update_App_Step = COMPLETE_UPDATE_APP;
										ZYJ_MJH=0x01; 
										OSSemPost(g_UpdateApp);
									}
								break;
								default:
									g_Update_App_Step = UPDATE_APP_DEF;	 //更新标志清零
									ZYJ_MJH   		  = 0;			
									g_bStarted        = 0;			
								break;
							}
						}
						else	//不是完整的一包 继续寻找帧头
						{
							temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //继续检查下一字节；
			 				temp_length--;
						}	
				   }
				   else		
				   {
						if(frame_length > MAX_FRAME_LENGTH_FF || frame_length == 0)
						{
							temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //包长过长，舍弃，继续检查下一字节；
			 				temp_length--;	
						}
						else  //剩下的数据不足当前包长度且包长有效
							break;	//等待下一次接收完成	
				   }
			   			
			   }
			   else	//
			   {
			   		break;		//等待下一次接收完成															  
			   }
			}
			else  //未找到帧头
		  	{	
				temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //继续检查下一字节；
			 	temp_length--;
			}
		}
		if(temp_length == 1 && g_u8U5RcvBuf[temp_head] != 0xFF )	    //剩下的一个字节不是FF
		{
			temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	  
			temp_length--;
		}
   		g_UART5_Proc_Index = temp_head;		//队列头赋值
   }
	return 0;
}

//串口5协议解析
void AnalyzeCom5Data(uint8 *pUartBuf, uint16 len)
{
	uint8 CmdCode=0;
	uint8 CH=CH_COM2;
	uint8 PTRL=PTRL_FF;
	CmdCode=pUartBuf[2];
	switch(CmdCode)
	{
		case CMD_GETSYSPARAM:
			GetSysParam(CH,PTRL);
			break;
		case CMD_SETSYSPARAM:
			SetSysParam(pUartBuf,CH,PTRL);
			break;
		case CMD_GETMDLPARAM:
			GetModelParam(CH,PTRL);
			break;	
		case CMD_SETMDLPARAM:
			SetModelParam(pUartBuf,CH,PTRL);
			break;
		case CMD_GETRESETINF:
			GetCtlResetInfo(CH,PTRL);
			break;		
		case CMD_CLRRESETINF:
			ClrCtlResetInfo(CH,PTRL);
			break;
		case CMD_CTLSYSRESET:
			CtlSysReset(CH,PTRL);
			break;
		case CMD_CTL2FACTROY:
			Ctl2FactoryState(CH,PTRL);
			break;
		case CMD_GETNETPARAM:
			GetNetParam(CH,PTRL);
			break;
		case CMD_SETNETPARAM:
			SetNetParam(pUartBuf,CH,PTRL);
			break;
		case CMD_GETUARTINFO:
			GetUartInfoParam(CH,PTRL);
			break;
		case CMD_SETUARTINFO:
			SetUartInfo(pUartBuf,CH,PTRL);
			break;
		case CMD_GETSYSTIMES:
			GetSysTime(CH,PTRL);
			break;
		case CMD_SETSYSTIMES:
			SetSysTime(pUartBuf,CH,PTRL);
			break;
		case CMD_GETAVITHRES:
			
			break;
		case CMD_SETAVITHRES:
			
			break;
		case CMD_GETSINGWAVE:
			GetSingleWave(CH,PTRL);
			break;
		case CMD_GETAQCSTATE:
			GetAqcWvState(CH,PTRL);
			break;
		case CMD_CLRAQCSTATE:
			ClrAqcWvState(CH,PTRL);
			break;
		case CMD_GETVEHOUTTH:
			GetVehOutTH(CH,PTRL);
			break;
		case CMD_SETVEHOUTTH:
			SetVehOutTH(pUartBuf,CH,PTRL);
			break;
		case CMD_GETWEIGHTAT:
			GetWeightAtSet(CH,PTRL);
			break;
		case CMD_SETWEIGHTAT:
			SetWeightAtSet(pUartBuf,CH,PTRL);
			break;
		case DLL_OPENCHANNEL:
			DllOpenCHannel(CH,PTRL);
			break;	
		case DLL_GETDEVSTATE:
			DllGetDeviState(CH,PTRL);
			break;	
		case DLL_VHCHECKINFO:
			break;
		case DLL_DEVISYNCHRO:
			break;
		case DLL_MANCLOSEOUT:
			break;
		case DLL_CLOSECHANEL:
			break;
		default:
			break;
	}
}


