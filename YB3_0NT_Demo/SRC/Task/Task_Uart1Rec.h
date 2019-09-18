#ifndef __TASK_SENDUART1_H__
#define	__TASK_SENDUART1_H__

#include "JZGlobal.h"

void Task_Uart1Rec(void *tdata);//串口1接收处理任务
void AnalyzeCom1Data(uint8 *pUartDataBuf, uint16 len);//串口1协议解析函数
uint8 RecCom1Data(void);//串口1循环接收解析函数

#endif
