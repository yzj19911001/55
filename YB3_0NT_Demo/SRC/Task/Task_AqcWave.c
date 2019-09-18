#include "Task_AqcWave.h"
#include "common.h"
#include "Task_Data_JG.h"
#include "Timer0.h"
#include "sddriver.h"
#include "Task_Data_JG.h"


/***************************************************************/
FramSave g_WvSave;//存储波形记录结构体
uint32   g_SdWrErr;//sd卡写操作失败计数

/*******************************************************************************************************************
** 函数名称:Task_AqcWave
**
** 功能描述:过车波形采集任务
**
** 输    入: 无
**
** 输    出: 无
**
** 注意事项: 如果一次写不成功会跳过该次写操作；无其他异常处置方法可选;
**           根据对应的控制器信号设置好控制器的类型 方便中途检测插拔sd卡
** 设计日期: 20161209
**
********************************************************************************************************************/
void Task_AqcWave(void *tdata)
{
    uint16 nTmpI=0;
	uint32 l_u32SdNo=0;
    tdata=tdata;

	Init_FrameSave();
	
    while (1)
    {
		
#if CONTROLLER==WLC864I89		
		if(SD_INSERT)//高电平为无卡 
		{
			l_u32SdNo++;
			OSTimeDly(50);
			continue;
		}
		else
		{
			//低电平表示sd卡插入 
			if(l_u32SdNo>50)//如果检测到无卡超过了50次 则需进行初始化
			{
				SDCardInit();
				ReadAddrInfoFsd();
			}
			l_u32SdNo=0;
		}
#else
		if(g_SdWrErr>50)
		{
			SDCardInit();
			ReadAddrInfoFsd();
			g_SdWrErr=0;
		}		
#endif		
        if(g_WvSave.OCacheIndx!=g_WvSave.ICacheIndx)
        {
            for(nTmpI=0; nTmpI<g_WvSave.IFrIndx[g_WvSave.OCacheIndx]; nTmpI++)
            {
                //Fram_Sv2SD(g_WvSave.FrameSaveBuf[g_WvSave.OCacheIndx][nTmpI],Full_Size*3+BHeaderSize,g_WvSave.CurSvAddr);
			        	Fram_Sv2SD_Ex(g_WvSave.FrameSaveBuf[g_WvSave.OCacheIndx][nTmpI],F1BlockNum,g_WvSave.CurSvAddr);
                g_WvSave.WrSdCnt++;//已写入sd卡帧数计数
            }
            //将出缓存的索引自增
            g_WvSave.OCacheIndx=(g_WvSave.OCacheIndx+1)%CACHENUM;
            //地址写入到sd卡中
		       	SaveAddrInfoTsd();
           
        }
        else
        {
            OSTimeDly(50);
        }
        OSTimeDly(50);
    }

}


/*******************************************************************************************************************
** 函数名称:Init_FrameSave
**
** 功能描述:初始化过车波形采集的相关地址及变量
**
** 输    入: 无
**
** 输    出: 无
**
** 注意事项: 无
**
** 设计日期: 20161209
**
********************************************************************************************************************/
void Init_FrameSave(void)
{
    //清空波形存储结构体
    memset(&g_WvSave,0,sizeof(g_WvSave));
	
	//初始化存储地址
	ReadAddrInfoFsd();
	
	//使能标志位
	if(g_SysSetUp.u8SDEnable==1)
	{
		g_WvSave.Disabled=ISOPENED;
	}
	else
	{
		g_WvSave.Disabled=ISCLOSED;
		SetDefualtAddr();//不使能使其初始化复位 20170112
	}
}
/*******************************************************************************************************************
** 函数名称:SaveAddrInfoTsd				
**
** 功能描述:将存波形相关的地址信息存储至sd卡指定扇区内 
**
** 输    入:无
**	         
** 输    出:1--写入成功;0--写入失败； 
**
** 注意事项:写入时注意地址的逻辑性检查 
**
** 设计日期:20161223 
**
********************************************************************************************************************/
uint8 SaveAddrInfoTsd(void)
{
	uint8 l_u8Buff[SECTORSIZE]={0};
	uint16 TmpI=0;

	l_u8Buff[TmpI++]='W';
	l_u8Buff[TmpI++]='A';
	l_u8Buff[TmpI++]='V';
	l_u8Buff[TmpI++]='E';
	//长度位后面添加2位
	TmpI=6;
	l_u8Buff[TmpI++]=g_WvSave.FgFlag;//是否发生存储覆盖
	l_u8Buff[TmpI++]=(STARTSECTORNO_FRAMESAVE>>24)&0xFF;//存储起始地址
	l_u8Buff[TmpI++]=(STARTSECTORNO_FRAMESAVE>>16)&0xFF;//存储起始地址
	l_u8Buff[TmpI++]=(STARTSECTORNO_FRAMESAVE>>8)&0xFF;//存储起始地址
	l_u8Buff[TmpI++]=STARTSECTORNO_FRAMESAVE&0xFF;//存储起始地址
	l_u8Buff[TmpI++]=(g_WvSave.CurSvAddr>>24)&0xFF;//存储当前地址
	l_u8Buff[TmpI++]=(g_WvSave.CurSvAddr>>16)&0xFF;//存储当前地址
	l_u8Buff[TmpI++]=(g_WvSave.CurSvAddr>>8)&0xFF;//存储当前地址
	l_u8Buff[TmpI++]=g_WvSave.CurSvAddr&0xFF;//存储当前地址
	l_u8Buff[TmpI++]=(ENDSECTORNO_FRAMESAVE>>24)&0xFF;//存储终止地址
	l_u8Buff[TmpI++]=(ENDSECTORNO_FRAMESAVE>>16)&0xFF;//存储终止地址
	l_u8Buff[TmpI++]=(ENDSECTORNO_FRAMESAVE>>8)&0xFF;//存储终止地址
	l_u8Buff[TmpI++]=ENDSECTORNO_FRAMESAVE&0xFF;//存储终止地址
	l_u8Buff[4]=((TmpI+2)>>8)&0xFF;
	l_u8Buff[5]=(TmpI+2)&0xFF;
	AddCrc16(l_u8Buff,	TmpI);
	
	if(full_write_sd(SAVEADDRINFO_SDADDR,l_u8Buff))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/*******************************************************************************************************************
** 函数名称:SetDefualtAddr				
**
** 功能描述:读取存储于sd卡指定扇区的波形存储相关地址信息，并进行该扇区内信息的检验
**
** 输    入:无
**			          
** 输    出:无 
**
** 注意事项:无 
**
** 设计日期:20161226 
**
********************************************************************************************************************/
void ReadAddrInfoFsd(void)
{
	uint8 l_u8Buff[SECTORSIZE]={0};
	uint16 FramLen=0;
	uint32 CurAddr=0;
	uint8 flag=0;
	
	if(0==full_read_sd(SAVEADDRINFO_SDADDR,l_u8Buff))
	{
		if(l_u8Buff[0]=='W'&&l_u8Buff[1]=='A'
		&&l_u8Buff[2]=='V'&&l_u8Buff[3]=='E')
		{
			FramLen=(l_u8Buff[4]<<8)+l_u8Buff[5];
			if(CheckCrc(l_u8Buff,FramLen-2))
			{
				flag=l_u8Buff[6];
				CurAddr=(l_u8Buff[11]<<24)+(l_u8Buff[12]<<16)+(l_u8Buff[13]<<8)+l_u8Buff[14];
				if(CurAddr>STARTSECTORNO_FRAMESAVE-1&&CurAddr<ENDSECTORNO_FRAMESAVE+1)
				{
					g_WvSave.CurSvAddr=CurAddr;
				}
				else
				{
					g_WvSave.CurSvAddr=STARTSECTORNO_FRAMESAVE;
				}
				if(flag!=0xAA&&flag!=0xBB)
				{
					g_WvSave.FgFlag=0xAA;
				}
				else
				{
					g_WvSave.FgFlag=flag;
				}
			
				SaveAddrInfoTsd();
				
				return;
			}
		}
	}

	SetDefualtAddr();
}

/*******************************************************************************************************************
** 函数名称:SetDefualtAddr				
**
** 功能描述:将存波形的相关地址信息初始化默认设置
**
** 输    入:无 
**	         
** 输    出:无 
**
** 注意事项:无 
**
** 设计日期:20161226
**
********************************************************************************************************************/
void SetDefualtAddr(void)
{
	g_WvSave.FgFlag=0xAA;
	g_WvSave.CurSvAddr=STARTSECTORNO_FRAMESAVE;
	SaveAddrInfoTsd();
}

/*******************************************************************************************************************
** 函数名称:Fram_Sv2SD()
**
** 功能描述: 将缓存中的波形数据写入到SD卡中
**
** 输    入: pData--波形数据内容所在指针
**	         nSize--波形数据内容长度
**	         Sd_Addr--写入SD卡的起始地址
** 输    出: 1--写入成功
**           0--写入失败
** 注意事项:如果写sd卡失败则跳出写 返回存储失败 写单块操作
**
** 设计日期:20161209
**
********************************************************************************************************************/
uint8 Fram_Sv2SD(uint8 *pData,uint16 nSize,uint32 Sd_Addr)
{
    uint16 wSize=0 ;
    while(wSize<nSize)
    {
        if(full_write_sd(Sd_Addr,&pData[wSize])!=0)
        {
			g_SdWrErr++;
            break;
        }
        else
        {
			g_SdWrErr=0;
            g_WvSave.CurSvAddr=STARTSECTORNO_FRAMESAVE+(g_WvSave.CurSvAddr+1-STARTSECTORNO_FRAMESAVE)%SIZESECTOR_FRAMESAVE;
            Sd_Addr=g_WvSave.CurSvAddr;
            wSize=wSize+SECTORSIZE;
			if(g_WvSave.CurSvAddr==STARTSECTORNO_FRAMESAVE)
			{
				g_WvSave.FgFlag=0xBB;//发生覆盖
			}
        }
    }
    return 1;
}



/*******************************************************************************************************************
** 函数名称:Fram_Sv2SD_Ex()
**
** 功能描述: 将缓存中的波形数据写入到SD卡中
**
** 输    入: pData--波形数据内容所在指针
**	         nNum--存储的扇区个数
**	         Sd_Addr--写入SD卡的起始地址
** 输    出: 1--写入成功
**           0--写入失败
** 注意事项:如果写sd卡失败则跳出写 返回存储失败;使用写多块操作
**
** 设计日期:20170111
**
********************************************************************************************************************/
uint8 Fram_Sv2SD_Ex(uint8 *pData,uint8 nNum,uint32 Sd_Addr)
{
	if(full_Mutil_Wr_sd(Sd_Addr,pData,nNum))
	{
		g_SdWrErr++;
	}
	else
	{
		g_SdWrErr=0;
		g_WvSave.CurSvAddr=STARTSECTORNO_FRAMESAVE+(g_WvSave.CurSvAddr+nNum-STARTSECTORNO_FRAMESAVE)%SIZESECTOR_FRAMESAVE;
		if(g_WvSave.CurSvAddr==STARTSECTORNO_FRAMESAVE)
		{
			g_WvSave.FgFlag=0xBB;//发生覆盖
		}	
	}
    return 1;
}


/*******************************************************************************************************************
** 函数名称:Catch_Frame_RT()
**
** 功能描述:根据有车标识状态实时在线抓取三路波形
**
** 输    入:pVehicle -- 车辆记录结构体
**			nCmd     -- 帧类型 1--正在过车波形; 2--收尾波形
**
** 输    出:无
**
** 注意事项:空白帧在出车收尾阶段已经添加进来
**
** 设计日期: 20161209
**
********************************************************************************************************************/
void Catch_Frame_RT(VehicleSet* pVehicle,uint8 nCmd)
{
	//增加缓存数据的头
    uint8 nTmpIdx=0;
	uint8 nAxIdx=0;
    uint8 IcIdx=g_WvSave.ICacheIndx;//缓存索引
    uint16 IfrIdx=g_WvSave.IFrIndx[IcIdx];//帧索引

    if(g_WvSave.Disabled==ISOPENED)
    {
		//生成BlockHeader
		g_WvSave.SvIndx++;//单车帧序号
		g_WvSave.SwSdCnt++;//应写入sd卡帧数计数
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]='W';//0
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]='A';//1
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]='V';//2
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]='E';//3
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=(g_WvSave.SvIndx>>8) & 0xFF;//4-5
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=g_WvSave.SvIndx & 0xFF;//
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=YEAR-2016;//6
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=MONTH;//7
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=DAY;//8
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=HOUR;//9
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=MIN;//10
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=SEC;//11
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=F1BlockNum;//占扇区数//12
		
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->JudgeSet.u8JudgePattern;//车型
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->AxleSet.AxleSum;//轴数 69 
		
		for(nAxIdx=0;nAxIdx<pVehicle->AxleSet.AxleSum;nAxIdx++)
		{
			g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->AxleSet.Axle[nAxIdx].AxleType;//轴型	
		}
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->JudgeSet.u8JudgeRule;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->JudgeSet.u8ModifyRule;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->JudgeSet.u8SpeVehCode;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsHopper;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsHeadJump;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsTailJump;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsContainer;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsCargo;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsTruckTail;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsFillWin;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsGlassWin;
		g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][nTmpIdx++]=pVehicle->FeatureSet.IsSideUiform;
		
		
		//拷贝波形
		memcpy(&g_WvSave.FrameSaveBuf[IcIdx][IfrIdx][BHeaderSize],\
			   JG0_CurBuff,Full_Size*sizeof(uint8));




		//入缓存数组索引自增
		g_WvSave.IFrIndx[IcIdx]=g_WvSave.IFrIndx[IcIdx]+1;

		//判断是否存满
		if(nCmd==2)
		{
			g_WvSave.ICacheIndx=(g_WvSave.ICacheIndx+1)%CACHENUM ;
			g_WvSave.IFrIndx[g_WvSave.ICacheIndx]=0 ;
			g_WvSave.SvIndx=0; 
			//抛出信号进入存SD卡操作
			//OSSemPost(g_Event_FrameSave);
		}
		else
		{
			if(g_WvSave.IFrIndx[IcIdx]==FRAMESIZE)
			{
				//存满之后 切换缓存
				g_WvSave.ICacheIndx=(g_WvSave.ICacheIndx+1)%CACHENUM ;
				g_WvSave.IFrIndx[g_WvSave.ICacheIndx]=0 ;
				//抛出信号进入存SD卡操作
				//OSSemPost(g_Event_FrameSave);
			}
		}		
    }
}
