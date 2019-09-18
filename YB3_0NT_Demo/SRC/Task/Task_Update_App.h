#ifndef Task_Update_App_H_
#define Task_Update_App_H_

#include "Common.h"
#include "JZGlobal.h"
#include "Uart1.h"
#include "Uart5.h"


#define	NORDATA_ZYJ_1		0x010000		/* 存储数据的NOR偏移首地址[0M+64K]*/
#define	NORDATA_ZYJ_2		0x110000		/* 存储数据的NOR偏移首地址[1M+64K]*/

#define UPDATE_MAXPACKET    500
#define UPDATE_CACHESIZE    UPDATE_MAXPACKET*1024

// TCP更新程序过程标志
#define UPDATE_APP_DEF		0x00
#define START_UPDATE_APP	0x01
#define UPDATE_APP_DOING	0x02
#define COMPLETE_UPDATE_APP		0x03
#define	NORDATA_NEW		0x1A0000



JZG_EXT uint32 PktCodeC(uint8 *ibuf, uint32 ilen, uint8 *obuf, uint32 *olen);
JZG_EXT uint32 PktDecode(uint8 *ibuf, uint32 ilen, uint8 *r_u8FrmNo, uint8 *obuf, uint32 *olen);
JZG_EXT uint8 Checkjiahe_Remote(uint8 *p_pu8Ptr_a,uint32 p_u16Len_a);
JZG_EXT uint8 CompareStr(uint8 *p_u8Str1, uint8 *p_u8Str2, uint32 u32StrLen);


#endif
