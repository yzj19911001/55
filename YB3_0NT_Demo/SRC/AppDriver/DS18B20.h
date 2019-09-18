/****************************************Copyright (c)****************************************************
**                                     BEIJING  WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			DS18B20.h
** Last modified Date:  20110602
** Last Version:		1.0
** Descriptions:		温度操作驱动
**
**--------------------------------------------------------------------------------------------------------
** Created by:			HE NA
** Created date:		20110602
** Version:				1.0
** Descriptions:		温度检测
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__DS18B20_H
#define	__DS18B20_H

#ifdef __DS18B20_C
#define	DS18B20_EXT
#else
#define	DS18B20_EXT	extern
#endif

#include "config.h"


DS18B20_EXT void DS18B20Init(void);
DS18B20_EXT int  ReadTemperature(void);
DS18B20_EXT void DS18B20IDRD(void);





DS18B20_EXT uint8 DS18B20ID[8];

#endif



