#ifndef _Uart1_H_
#define _Uart1_H_

#include "config.h"

#include "JZGlobal.h"

JZG_EXT void UART1_SendBuf (uint8  *RcvBufPt,  uint32  Snd_Len);
JZG_EXT void UART1_Init (uint32 p_iBaudRate);
JZG_EXT void UART1_SendBuf_full(uint8  *RcvBufPt,  uint32  Snd_Len);		 	//¶ÀÏí·½Ê½²Ù×÷UART1
	
#endif
