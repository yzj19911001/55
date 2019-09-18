#ifndef __COMMON_H__
#define __COMMON_H__

#include "config.h"

#ifdef	__COMMON_C
#define	CMN_EXT	
#else
#define	CMN_EXT	extern
#endif
/***************************************/
/***************************************/



/***************************************/
/**********任务优先级*******************/
/***************************************/
#define		TaskJG0PRIO				3
#define		TaskDataJGPRIO	    	4
#define   	TASKUART1RECPRIO	    7
#define		TASKChecknetPRIO	    14
#define		TASKTiPoPRIO			2           //时钟任务
#define		TASKUART5RECPRIO		6
#define		TASKSTARTPRIO		    16 
#define 	TASKCHECKDIRPRIO  	    5
#define     TaskAqcWavePRIO         10
/***************************************/
/**********任务优先级*******************/
/***************************************/

CMN_EXT	uint16	YEAR;				//年
CMN_EXT	uint8	MONTH;				//月
CMN_EXT	uint8	DAY;					//日
CMN_EXT	uint8	WEEK;					//星期几
CMN_EXT	uint8	HOUR;					//时
CMN_EXT	uint8	MIN;					//分
CMN_EXT	uint8	SEC;					//秒

#define CRC_OK      0
#define CRC_ERROR   1

CMN_EXT uint16 crc_16(uint8 *ptr, uint16 len);

CMN_EXT void crc_create(uint8 *ptr, uint16 len);
CMN_EXT uint8 crc_check(uint8 *ptr, uint16 len);
CMN_EXT uint16 CRC16(uint8 *puchMsg, uint16 usDataLen);
CMN_EXT uint16 CRCSum(uint8 *data, uint32 length);
CMN_EXT uint8 Check_CRCSum(uint8 *ptr, uint16 len);
CMN_EXT uint8 b2bcd(uint8 b);
CMN_EXT uint8 bcd2b(uint8 bcd);
CMN_EXT	uint16	AddCrc16( uint8 * p_pu8Ptr,uint16 p_u16Len);		//校验，写CRC
CMN_EXT	uint8	CheckCrc( uint8 * p_pu8Ptr,uint16 p_u16Len);		//校验，不写CRC
CMN_EXT	uint8	bcd(uint32 p_u32Value,uint8 p_u8Pos);				//获取bcd数字


#endif
