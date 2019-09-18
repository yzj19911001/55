#ifndef	_W5300_Conf_H_
#define	_W5300_Conf_H_
#include "config.h"

#define	W5300_LINK      ( P3_INP_STATE & ( 1 << 6 ) )   		// GPI06连接检测脚，低电平表示连接正常

typedef uint8 SOCKET;

#ifndef __cplusplus
typedef int				boolint;
#define	true			1
#define false			0
#endif

#ifndef NULL
#define NULL (void *)0
#endif


//#define __DEF_W5300_DBG__      /**< Involves debug code */
#define __DEF_W5300_INT__    /**< Involves ISR routine */
//#define __DEF_W5300_PPP__    /**< Involves PPP service routines and md5.h & md5.c */

/**
 * SOCKET count of W5300 
 */
#define	MAX_SOCK_NUM		8

#define __DEF_W5300_DIRECT_MODE__     1    /**< Direct address mode */
#define __DEF_W5300_INDIRECT_MODE__   2    /**< Indirect address mode */
/**
 * It is used to decide to W5300 host interface mode. 
 */
#define __DEF_W5300_ADDRESS_MODE__           __DEF_W5300_DIRECT_MODE__		  //直接模式
//#define __DEF_W5300_ADDRESS_MODE__         __DEF_W5300_INDIRECT_MODE__	  //间接模式
/**
 * Define the base address of W5300 on your target host system.
 */
#define __DEF_W5300_MAP_BASE__ 	0xE1000000	  //w5300在系统中的基地址

#if (__DEF_W5300_ADDRESS_MODE__ == __DEF_W5300_DIRECT_MODE__)
   #define COMMON_REG_BASE   __DEF_W5300_MAP_BASE__          /**< The base address of COMMON_REG */
   #define SOCKET_REG_BASE   (__DEF_W5300_MAP_BASE__ + 0x0200) /**< The base address of SOCKET_REG */   
#else
   #define COMMON_REG_BASE     0                               
   #define SOCKET_REG_BASE     0x0200                          
#endif

#define SOCKET_REG_SIZE    0x40     // SOCKET Regsiter Count per Channel

#define __DEF_C__                  0   /**< Using C code */
#define __DEF_MCU_DEP_INLINE_ASM__ 1   /**< Using inline ASM code */
#define __DEF_MCU_DEP_DMA__        2   /**< Using DMA controller */

/**
 * It define how to access to the intenal TX/RX memory of W5300.
 */
#define __DEF_W5300_BUF_OP__      __DEF_C__
//#define __DEF_W5300_BUF_OP__    __DEF_MCU_DEP_INLINE_ASM__
//#define __DEF_W5300_BUF_OP__    __DEF_MCU_DEP_DMA__

/**
 * Enter a critical section
 */
#define W5300_CRITICAL_SECTION_ENTER() {} //{OsSwiHandle1(1);}

/**
 * Exit a critical section
 */
#define W5300_CRITICAL_SECTION_EXIT() {} //{OsSwiHandle1(2);}

#endif

