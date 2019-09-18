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
	uint16 temp_head = 0;  //�������ݵĶ���ͷ
	uint16 temp_tail = 0;  //�������ݵĶ���β
	uint16 temp_length = 0;  //�������ݵĳ���
	uint16 frame_length = 0;  //����������֡��
	uart5_timeout = 0;	 //���ڽ��ճ�ʱ����
	temp_head =  g_UART5_Proc_Index;
	temp_tail =  g_UART5_Recv_Index;
	if(temp_head != temp_tail )		 //������в�Ϊ��
	{
		if(temp_head < temp_tail)	//������������ݳ���
		{
		 	temp_length =  temp_tail - temp_head;
		}
		else
		{
			temp_length = UARTRCVBUFSIZE-(temp_head - temp_tail);
		}

		while( ((temp_head+1)&(UARTRCVBUFSIZE-1)) != temp_tail  && temp_length )		 //����ÿ��֡ͷ ȷ���Ƿ�Ϊ��Ч֡
		{
		  	if(g_u8U5RcvBuf[temp_head] == 0xFF && (g_u8U5RcvBuf[(temp_head+1)&(UARTRCVBUFSIZE-1)] == 0xFF) )	  //�ж�֡ͷ
			{
			   if( temp_length >= MIN_FRAME_LENGTH )  //�������泤�ȴ�����С����7   ��ʹ�� g_u8U5RcvBuf[ (temp_head+3)%1024 ]
			   {
			   		if( (g_u8U5RcvBuf[(temp_head + 2)&(UARTRCVBUFSIZE-1) ] == 0x80 || g_u8U5RcvBuf[(temp_head + 2)&(UARTRCVBUFSIZE-1) ] == 0x81 )&& g_u8U5RcvBuf[(temp_head + 3)&(UARTRCVBUFSIZE-1) ] == 0x3B)
					{ //Ѱ�ҽ���FF
						if( temp_length > 15 )	//Ѱ�ҽ���FF,���ȿ��԰���������FF
						{
							frame_length = 15;
							while( g_u8U5RcvBuf[((temp_head + frame_length)&(UARTRCVBUFSIZE-1)) ] != 0xFF &&  ((temp_head + frame_length)&(UARTRCVBUFSIZE-1)) != temp_tail )  //Ѱ�Һ����ĵ�һ��FF
								  frame_length++;
						}
						else   //Ѱ�ҽ���FF,�����޷�����������FF
						{
							 	break;	//�ȴ���һ�ν������
						}
						if( g_u8U5RcvBuf[(temp_head + frame_length)&(UARTRCVBUFSIZE-1) ] == 0xFF  && temp_length < MAX_FRAME_LENGTH_FF)	//Ѱ�ҵ�FF
						{
							frame_length = frame_length +1;
								memset(U5RecBuff , 0 , UARTRCVBUFSIZE);
							for(index = 0; index <frame_length ;index++ )//��Ӧ�����Ľ�����������Ҫ��ȡ����Ч����
							{
								U5RecBuff[index] =  g_u8U5RcvBuf[ (temp_head+index)% UARTRCVBUFSIZE ]	;
							}
						}
						else if	( g_u8U5RcvBuf[(temp_head + frame_length)&(UARTRCVBUFSIZE-1) ] != 0xFF  && temp_length < MAX_FRAME_LENGTH_FF )
							frame_length = temp_length+1 ;	//�ȴ���һ�ν������
						else
							frame_length = 	MAX_FRAME_LENGTH_FF + 1;	
					}
					else if(g_u8U5RcvBuf[(temp_head + 2)&(UARTRCVBUFSIZE-1) ] == 0x82 && g_u8U5RcvBuf[(temp_head + 3)&(UARTRCVBUFSIZE-1) ] == 0x3B)
					{
						frame_length =  8;	//����֡����
						memset(U5RecBuff , 0 , UARTRCVBUFSIZE);
						for(index = 0; index <frame_length;index++ )//��Ӧ�����Ľ�����������Ҫ��ȡ����Ч����
						{
							U5RecBuff[index] =  g_u8U5RcvBuf[ (temp_head+index)% UARTRCVBUFSIZE ]	;
						}
						if( U5RecBuff[7] != 0xFF )  //��Ч֡
						{
							frame_length = 	MAX_FRAME_LENGTH_FF + 1;//������ǰ֡ͷ
						}
					}
					else if(  g_u8U5RcvBuf[(temp_head+2)&(UARTRCVBUFSIZE-1)] == 0x02 &&  g_u8U5RcvBuf[(temp_head+3)&(UARTRCVBUFSIZE-1)] == 0x02 && g_u8U5RcvBuf[(temp_head+4)&(UARTRCVBUFSIZE-1)] == 0x02 && g_u8U5RcvBuf[(temp_head+5)&(UARTRCVBUFSIZE-1)] == 0x02)
					{	//����͸������
						if( ( temp_length >= MIN_FRAME_LENGTH_TC ))						
						{
							frame_length =(g_u8U5RcvBuf[ (temp_head+6)&(UARTRCVBUFSIZE-1) ]<<24) + (g_u8U5RcvBuf[ (temp_head+7)&(UARTRCVBUFSIZE-1) ]<<16) +(g_u8U5RcvBuf[ (temp_head+8)&(UARTRCVBUFSIZE-1) ]<<8) + g_u8U5RcvBuf[(temp_head+9)&(UARTRCVBUFSIZE-1)] +18 ; //�����ݰ����ȣ�����ͷβУ��
						   if(temp_length >= (frame_length) && frame_length <= MAX_FRAME_LENGTH_TC )	 //	 �Ƿ��������һ��  frame_length
						   {
						   		if( g_u8U5RcvBuf[(temp_head+frame_length-1)&(UARTRCVBUFSIZE-1)] == 0xEE )	  //�ж�֡βУ��
								{	
									memset(U5RecBuff , 0 , UARTRCVBUFSIZE);
									for(index = 0; index <frame_length + 6;index++ )//��Ӧ�����Ľ�����������Ҫ��ȡ����Ч����
									{
										U5RecBuff[index] =  g_u8U5RcvBuf[ (temp_head+index)% UARTRCVBUFSIZE ]	;
									}
									
							   		temp_head = (frame_length + temp_head )&(UARTRCVBUFSIZE-1);  //ȷ���µĴ�����ʼ��;
									temp_length =  temp_length -frame_length ;
									g_UART5_Proc_Index = temp_head;		//����ͷ��ֵ
									g_LaserPass.PassCH=CH_COM1;//͸��ͨ��ΪCOM5
									PassThroughToJG(U5RecBuff+2);			   //��������
							   	}
								else	//����������һ�� ����Ѱ��֡ͷ
								{
									temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //���������һ�ֽڣ�
					 				temp_length--;
								}	
						   }
						   else		
						   {
						   		if(frame_length > MAX_FRAME_LENGTH_TC)
								{
									temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //�������������������������һ�ֽڣ�
					 				temp_length--;	
								}
								else  //ʣ�µ����ݲ��㵱ǰ�������Ұ�����Ч		 ���Ǽ���ʱ���޶����������޵ȴ�
									break;	//�ȴ���һ�ν������	
						   }
						}
						else
						{
								break;		//�ȴ���һ�ν������		
						}
					}
					else
					{	
						//������������
					   if(g_u8U5RcvBuf[ (temp_head+2)&(UARTRCVBUFSIZE-1) ]==0xFF)
					   {
							temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //���FF���� ȥ��ǰ���
							temp_length--;
							continue;
					   }
					   frame_length = (g_u8U5RcvBuf[ (temp_head+3)&(UARTRCVBUFSIZE-1) ]<<8) + g_u8U5RcvBuf[(temp_head+4)&(UARTRCVBUFSIZE-1)]; //�����ݰ����ȣ�����ͷβУ��
					   if(temp_length >= (frame_length) && frame_length <= MAX_FRAME_LENGTH )	 //	 �Ƿ��������һ��  frame_length
					   {
					   		if( CheckCrc(g_u8U5RcvBuf+temp_head,frame_length-2)== 1 )	  //�ж�֡βУ��
							{	
								memset(U5RecBuff , 0 , UARTRCVBUFSIZE);
								for(index = 0; index <frame_length + 6;index++ )//��Ӧ�����Ľ�����������Ҫ��ȡ����Ч����
								{
									U5RecBuff[index] =  g_u8U5RcvBuf[ (temp_head+index)&(UARTRCVBUFSIZE-1) ]	;
								}
								AnalyzeCom5Data(U5RecBuff, frame_length);			   //��������
						   		temp_head = (frame_length + temp_head )&(UARTRCVBUFSIZE-1);  //ȷ���µĴ�����ʼ��;
								temp_length =  temp_length -frame_length ;
						   	}
							else	//����������һ�� ����Ѱ��֡ͷ
							{
								temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //���������һ�ֽڣ�
				 				temp_length--;
							}	
						  
					   }
					   else		
					   {
					   		if(frame_length > MAX_FRAME_LENGTH)
							{
								temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //�������������������������һ�ֽڣ�
				 				temp_length--;	
							}
							else  //ʣ�µ����ݲ��㵱ǰ�������Ұ�����Ч		 ���Ǽ���ʱ���޶����������޵ȴ�
								break;	//�ȴ���һ�ν������	
					   }
					}

				   if(temp_length >= frame_length && frame_length <= MAX_FRAME_LENGTH_FF )	 //	 �Ƿ��������һ��  frame_length
				   {
				   		if (!PktDecode(U5RecBuff ,frame_length,&g_u8FrmNo,g_u8ParseBuf,&g_u32ParseLen  ))//У��ɹ�
						{
							temp_head = (frame_length + temp_head)&(UARTRCVBUFSIZE-1);  //ȷ���µĴ�����ʼ��; //ռ�õĻ���ϴ󣬿���������ⲿ�ֻ���
							temp_length =  temp_length - frame_length ;
							g_UART5_Proc_Index = temp_head;		//����ͷ��ֵ
							switch( U5RecBuff[4] )
							{
								case 0x11:
										GetRTCTime(&g_sstCurTime_Remote);		 //��ȡ��ʼ���³������ʼʱ��
   										MIN_Remote=g_sstCurTime_Remote.u8Minute; 
										g_Update_App_Step = START_UPDATE_APP;
										g_Update_App_CH=CH_COM1;
										g_bStarted = 1;	//uint8 �ͱ�־����
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
									g_Update_App_Step = UPDATE_APP_DEF;	 //���±�־����
									ZYJ_MJH   		  = 0;			
									g_bStarted        = 0;			
								break;
							}
						}
						else	//����������һ�� ����Ѱ��֡ͷ
						{
							temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //���������һ�ֽڣ�
			 				temp_length--;
						}	
				   }
				   else		
				   {
						if(frame_length > MAX_FRAME_LENGTH_FF || frame_length == 0)
						{
							temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //�������������������������һ�ֽڣ�
			 				temp_length--;	
						}
						else  //ʣ�µ����ݲ��㵱ǰ�������Ұ�����Ч
							break;	//�ȴ���һ�ν������	
				   }
			   			
			   }
			   else	//
			   {
			   		break;		//�ȴ���һ�ν������															  
			   }
			}
			else  //δ�ҵ�֡ͷ
		  	{	
				temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	   //���������һ�ֽڣ�
			 	temp_length--;
			}
		}
		if(temp_length == 1 && g_u8U5RcvBuf[temp_head] != 0xFF )	    //ʣ�µ�һ���ֽڲ���FF
		{
			temp_head = (temp_head+1)&(UARTRCVBUFSIZE-1);	  
			temp_length--;
		}
   		g_UART5_Proc_Index = temp_head;		//����ͷ��ֵ
   }
	return 0;
}

//����5Э�����
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


