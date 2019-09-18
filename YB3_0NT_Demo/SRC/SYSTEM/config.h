/****************************************Copyright (c)****************************************************
**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
**                                     
**                               http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           config.h
** Last modified Date:  2004-09-17
** Last Version:        1.0
** Descriptions:        User Configurable File
**
**--------------------------------------------------------------------------------------------------------
** Created By:          Chenmingji
** Created date:        2004-09-17
** Version: 1.0
** Descriptions:        First version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:  		LinEnqiang
** Modified date: 		2007-12-15
** Version: 1.0
** Descriptions: 		For LPC3200
**
*********************************************************************************************************/
#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef  __cplusplus
extern "C" {
#endif

#ifndef TRUE
#define TRUE  1
#endif                                                                  /*  TRUE                        */

#ifndef FALSE
#define FALSE 0
#endif                                                                  /*  FALSE                       */

typedef unsigned char  uint8;                                           /*  �޷���8λ���ͱ���           */
typedef signed   char  int8;                                            /*  �з���8λ���ͱ���           */
typedef unsigned short uint16;                                          /*  �޷���16λ���ͱ���          */
typedef signed   short int16;                                           /*  �з���16λ���ͱ���          */
typedef unsigned int   uint32;                                          /*  �޷���32λ���ͱ���          */
typedef signed   int   int32;                                           /*  �з���32λ���ͱ���          */
typedef float          fp32;                                            /*  �����ȸ�������32λ���ȣ�    */
typedef double         fp64;                                            /*  ˫���ȸ�������64λ���ȣ�    */

typedef unsigned char  INT8U;                                           /*  �޷���8λ���ͱ���           */
typedef signed   char  INT8S;                                           /*  �з���8λ���ͱ���           */
typedef unsigned short INT16U;                                          /*  �޷���16λ���ͱ���          */
typedef signed   short INT16S;                                          /*  �з���16λ���ͱ���          */
typedef unsigned int   INT32U;                                          /*  �޷���32λ���ͱ���          */
typedef signed   int   INT32S;                                          /*  �з���32λ���ͱ���          */
typedef float          FP32;                                            /*  �����ȸ�������32λ���ȣ�    */
typedef double         FP64;                                            /*  ˫���ȸ�������64λ���ȣ�    */

typedef  volatile unsigned char  vuint8;    /* �޷���8λ���ͱ��� */
typedef  volatile signed   char  vint8;     /* �з���8λ���ͱ��� */
typedef  volatile unsigned short vuint16;   /* �޷���16λ���ͱ��� */
typedef  volatile signed   short vint16;    /* �з���16λ���ͱ��� */
typedef volatile  unsigned long   vuint32;  /* �޷���32λ���ͱ��� */
typedef  volatile signed   long   vint32;   /* �з���32λ���ͱ��� */ 

/*********************************************************************************************************
  ARM���������
  ARM specital code
*********************************************************************************************************/
#include    "LPC3200.h"
#include    "target.h"
#include    "vicControl.h"

/*********************************************************************************************************
  Ӧ�ó�������
  Application Program Configurations
*********************************************************************************************************/
#include    <stdio.h>
#include    <rt_sys.h>
#include    <ctype.h>
#include    <stdlib.h>
#include    <setjmp.h>
#include    <rt_misc.h>	  
#include	<string.h>
#include	<math.h>

#define		YBVERSION	30
#define		TEST_PROBE	1		//20131127 

#include	"DEFConfig.h"

#define		UBR_2400	2400		//���ڲ�����
#define		UBR_4800	4800		//���ڲ�����
#define		UBR_9600	9600		//���ڲ�����
#define		UBR_19200	19200		//���ڲ�����
#define		UBR_38400	38400		//���ڲ�����
#define		UBR_57600	57600		//���ڲ�����
#define		UBR_115200	115200		//���ڲ�����
#define		UBR_230400	230400		//���ڲ�����
#define		UBR_460800	460800		//���ڲ�����

#define		UARTDLM				0                                                
#define		UARTDLL				1

#include	"Includes.h"
#include	"JZGlobal.h"
#include	"GPIO.h"

/*********************************************************************************************************
  �����ӵ�����
  Configuration of the example
*********************************************************************************************************/
#define	USE_USB			0
#define	USE_PLL397      0


#define Fosc            13000000					                    /*	OSC = 13.0MHz               */

#define Fuclk	        48000000    

#define Fcclk           (Fosc * 20)                                     /* ������208MHz                 */

#define Fcco            (Fcclk * 1)                                     /* 
                                                                         * ������156MHz ��320MHz ��Χ�ڣ� 
                                                                         * Fcco������Fcclk��1��2��4��8��
                                                                         * 16��
                                                                         */

#define Fhclk           (Fcclk / 2)					                    /* 
                                                                         * Fhclk ͨ����Fcclk �� 2 ��Ƶ
                                                                         * ����������Fcclk ��ͬ��Ƶ�ʻ� 
                                                                         * Fcclk �� 4 ��Ƶ�����С�
                                                                         * FhclkƵ�����ò�Ӧ����104MHz 
                                                                         * �����Fosc
                                                                         */

#define Fpclk	        (Fcclk / 20)

#define Fdclk	        (Fhclk * 0)                        	            /* 
                                                                         * ���ʹ��DDR SDRAM,��ʱ�ӱ���
                                                                         * ���Ϊ 2 �� FhclkƵ��,��Fhclk
                                                                         * ΪFcclk��2��4��Ƶ,����Ϊ,0
                                                                         */
                                                                         
#define Fsclk	        (Fcclk / 8)                                     /* SD��ʱ��,����Fcclk��1~15��Ƶ */


#if (USE_PLL397 == 1)&&(USE_USB == 1)
   #error  "USB couldn't be used together with PLL397"                        
#endif
#if Fcclk > 260000000
   #error  "Fcclk couldn't be higher than 260MHz"                        
#endif

/*
 * �ж�FCCO�����Ƿ���ȷ
 */
#if !((Fcco / Fcclk == 1) || (Fcco / Fcclk == 2) ||   \
      (Fcco / Fcclk == 4) || (Fcco / Fcclk == 8) ||   \
      (Fcco / Fcclk == 16)) 
     #error  "The parameter Fcco is incorrect! "  
#endif
            
      

#ifdef __cplusplus
    }
#endif                                                                  /*  __cplusplus                 */
#endif                                                                  /*  __CONFIG_H                  */ 
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
