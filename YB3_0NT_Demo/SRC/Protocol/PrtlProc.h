#ifndef _PRTL_PROC_H_
#define _PRTL_PROC_H_

#include "JZGlobal.h"

//命令码
#define CMD_OPERATEERRS 0xFE //查询设置错误
#define CMD_GETSYSPARAM 0x01 //查询系统设参数
#define CMD_SETSYSPARAM 0x02 //设置系统参数
#define CMD_GETMDLPARAM 0x03 //查询模型参数
#define CMD_SETMDLPARAM 0x04 //设置模型参数
#define CMD_GETRESETINF 0x05 //查询重启信息
#define CMD_CLRRESETINF 0x06 //清空重启信息 
#define CMD_CTLSYSRESET 0x07 //重启控制器
#define CMD_CTL2FACTROY 0x08 //恢复出厂设置
#define CMD_GETNETPARAM 0x09 //获取网络参数
#define CMD_SETNETPARAM 0x0A //设置网络参数
#define CMD_GETUARTINFO 0x0B //查询串口参数
#define CMD_SETUARTINFO 0x0C //设置串口参数
#define CMD_GETSYSTIMES 0x0D //查询系统时间
#define CMD_SETSYSTIMES 0x0E//设置系统时间
#define CMD_GETAVITHRES 0x0F //获取车型识别阈值
#define CMD_SETAVITHRES 0x10 //设置车型识别阈值
#define CMD_GETSINGWAVE 0x11 //获取单帧波形
#define CMD_UPLOADVEHS  0x12 //调试时上传单车指令
#define CMD_GETAQCSTATE 0x13 //获取激光波形接收处理状态
#define CMD_CLRAQCSTATE 0x14 //清空激光波形接收处理状态
#define CMD_GETVEHOUTTH 0x15 //获取车辆收尾阈值参数
#define CMD_SETVEHOUTTH 0x16 //设置车辆收尾阈值参数
#define CMD_GETWEIGHTAT 0x17 //查询称台交互参数设置
#define CMD_SETWEIGHTAT 0x18 //设置称台交互参数设置


#define DLL_OPENCHANNEL 0x30  //打开设备通道
#define DLL_GETDEVSTATE 0x31  //设备状态
#define DLL_VHCHECKINFO 0x32  //车辆信息
#define DLL_DEVISYNCHRO 0x33  //设备同步
#define DLL_MANCLOSEOUT 0x34  //手动收尾
#define DLL_CLOSECHANEL 0x35  //关闭设备通道
#define DLL_ADD1VEHICLE 0x36  //增加一辆车
#define DLL_DEL1VEHICLE 0x37  //减去一辆车
#define DLL_UPLDQUEINFO 0x38  //上传队列信息




//错误码
#define ERR_OK 0x01 //成功
#define ERR_TD 0x02 //铁电操作失败
#define ERR_LG 0x03 //逻辑错误
#define ERR_AL 0x04 //内存分配错误
#define ERR_AB 0x05 //参数异常

void PassThroughToJG(uint8 *TouchuanBuff);//透传
uint8 SeekSocketNo(uint8 *p_IPbuff);//通过IP端口搜寻对应Socket
void GetSysTime(uint8 CH,uint8 PRTL);//获取系统时间
void SetSysTime(uint8 *pBuff,uint8 CH,uint8 PRTL);//设置系统时间
void GetSysParam(uint8 CH,uint8 PRTL);//获取系统参数
void SetSysParam(uint8 *pBuff,uint8 CH,uint8 PRTL);//设置系统参数
void GetModelParam(uint8 CH,uint8 PRTL);//获取模型参数
void SetModelParam(uint8 *pBuff,uint8 CH,uint8 PRTL);//设置模型参数
void GetCtlResetInfo(uint8 CH,uint8 PRTL);//获取控制器重启信息
void ClrCtlResetInfo(uint8 CH,uint8 PRTL);//清空控制器重启信息
void CtlSysReset(uint8 CH,uint8 PRTL);//重启控制器
void GetNetParam(uint8 CH,uint8 PRTL);//获取网络参数
void SetNetParam(uint8 *pBuff,uint8 CH,uint8 PRTL);//设置网络参数
void GetUartInfoParam(uint8 CH,uint8 PRTL);//获取系统参数
void SetUartInfo(uint8 *pBuff,uint8 CH,uint8 PRTL);//设置系统参数
void GetSingleWave(uint8 CH,uint8 PRTL);//获取单帧波形	
void Ctl2FactoryState(uint8 CH,uint8 PRTL);//恢复控制器至出厂状态
void PassThroughToUart(uint8 SocketIdx);//透传至串口
void PassThroughToJG(uint8 *ToJGBuff);//透传至激光
void ClrAqcWvState(uint8 CH,uint8 PRTL);//清空采集波形的状态
void GetAqcWvState(uint8 CH,uint8 PRTL);//获取采集波形的状态
void GetVehOutTH(uint8 CH,uint8 PRTL);//查询出车阈值参数
void SetVehOutTH(uint8 *pBuff,uint8 CH,uint8 PRTL);//设置出车阈值设置
void GetWeightAtSet(uint8 CH,uint8 PRTL);//查询与称台交互状态
void SetWeightAtSet(uint8 *pBuff,uint8 CH,uint8 PRTL);//设置称台交互参数

void DllGetDeviState(uint8 CH,uint8 PRTL);//DLL查询设备状态
void DllOpenCHannel(uint8 CH,uint8 PRTL);//DLL命令打开设备
void DllReplyManOut(uint8 CH,uint8 PRTL);//DLL应答手动收尾操作
void DllCloseCHannel(uint8 CH,uint8 PRTL);//DLL命令关闭设备
void DllAdd1VehInfo(uint8 *pBuff,uint8 CH,uint8 PRTL);//DLL增加1辆车
void DllDel1VehInfo(uint8 *pBuff,uint8 CH,uint8 PRTL);//DLL删除1辆车
void DllUpLoadQueInfo(uint8 *pBuff,uint8 CH,uint8 PRTL);//DLL上传队列信息

JZG_EXT void SendBuf_full(uint8  *RcvBufPt, uint32  Snd_Len,uint8 CH);//全通道发送函数

#endif
