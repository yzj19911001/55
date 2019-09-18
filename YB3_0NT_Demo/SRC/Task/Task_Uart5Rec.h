/******************************************************************************
  * @file    Task_Uart5Rec.h
  * @author  Jiang Nande
  * @version V1.0.0
  * @date    17-1-2017
  * @brief   Header file of Task_Uart5Rec.c module.
******************************************************************************
  * @attention
  *
  * <h2><center>&copy COPYRIGHT(c) 2017 Wanji Technology(WJ)</center></h2>
	*
	<h2><center>&</center></h2>
	*
  * All rights reserved.
******************************************************************************/
#ifndef _Task_Uart5Rec_H_
#define _Task_Uart5Rec_H_

#include "config.h"




void Task_Uart5Rec(void *tdata);//串口5接收处理任务
void AnalyzeCom5Data(uint8 *pUartDataBuf, uint16 len);//串口5协议解析函数
uint8 RecCom5Data(void);//串口5循环接收解析函数



#endif
