/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart5.h
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		Uart5��ʼ�����жϴ���
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__UART5_H
#define	__UART5_H

#include "config.h"

#ifdef	__UART5_C
#define	U5_EXT
#else
#define	U5_EXT	extern
#endif

void IRQ_UART5(void);		//�жϴ���

//��ʼ��
U5_EXT	void	UART5_Init(int p_iBaudRate);				

//����ָ���������ֽ�
U5_EXT	uint16 	U5ReciveByte(uint8 *p_pu8RcvDataBuf, uint8 p_u16RcvNum);

//����ָ������������
U5_EXT	void	U5SendBytes(uint8 * p_u8SendBuf, uint32 p_u32Len);

U5_EXT void UART5_Init(int p_iBaudRate);

U5_EXT void UART5_SendBuf_full(uint8  *RcvBufPt,  uint32  Snd_Len);		 	//����ʽ����UART5

#endif		//__UART5_H
