#ifndef __TASK_SENDUART1_H__
#define	__TASK_SENDUART1_H__

#include "JZGlobal.h"

void Task_Uart1Rec(void *tdata);//����1���մ�������
void AnalyzeCom1Data(uint8 *pUartDataBuf, uint16 len);//����1Э���������
uint8 RecCom1Data(void);//����1ѭ�����ս�������

#endif
