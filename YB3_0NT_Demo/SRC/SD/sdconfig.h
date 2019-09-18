/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdconfig.h
** Last modified Date:	2005-3-11
** Last Version:		V2.0
** Descriptions:		SD/MMC ����дģ��: �����ļ�
**						Soft Packet of SD/MMC Card: configuration header file
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		��ʼ�汾 The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng
** Modified date:		2005-3-11
** Version:				V2.0
** Descriptions:		������LPC22xx��Ӳ������ 	
**						add LPC22xx hardware configuration
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/

#ifndef __SDCONFIG_H__
#define __SDCONFIG_H__


typedef unsigned char  INT8U;                   /* �޷���8λ���ͱ���                        */
typedef signed   char  INT8;                    /* �з���8λ���ͱ���                        */
typedef unsigned short INT16U;                  /* �޷���16λ���ͱ���                       */
typedef signed   short INT16;                   /* �з���16λ���ͱ���                       */
typedef unsigned int   INT32U;                  /* �޷���32λ���ͱ���                       */
typedef signed   int   INT32;                   /* �з���32λ���ͱ���                       */
typedef float          FP32;                    /* �����ȸ�������32λ���ȣ�                 */
typedef double         FP64;                    /* ˫���ȸ�������64λ���ȣ�                 */

/* ZLG/SD ������Ĳ������� */
#define SD_UCOSII_EN			1			/* ��(1)��(0)��UCOS-II�����б������ */

#define SDBUS_MODE_EN           1           /* ��(1)��(0)ʹ�ܷ��ʿ���SD����ģʽ */

#define SD_WIDTHBUS_EN          1           /* ��(1)��(0)ʹ��SD���Ŀ�����ģʽ(��SD����ģʽ) */

#define SD_CRC_EN		   		1			/* ��(1)��(0)�����ݴ���ʱʹ��CRCУ�� */


/* ���к���, ����û�����Ҫ, ����Ϊ 0 �ü�ָ������ */

#define SD_ReadMultiBlock_EN    0			/* ��(1)��(0)ʹ�ܶ���麯�� */

#define SD_WriteMultiBlock_EN   1			/* ��(1)��(0)ʹ��д��麯�� */

#define SD_EraseBlock_EN		1			/* ��(1)��(0)ʹ�ܲ������� */


/* �����������������ͷ�ļ� */
#include "config.h"
#include "sdcommon.h"

#if SDBUS_MODE_EN
#include "sdhal.h"
#else
#include "sdspihal.h"
#endif

#include "sdcrc.h"
#include "sdcmd.h"
#include "sddriver.h"

#endif
