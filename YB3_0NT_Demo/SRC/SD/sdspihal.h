/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdspihal.c
** Last modified Date:	2007-07-12
** Last Version:		V3.0
** Descriptions:		SD/MMC����д�����: Ӳ�������ͷ�ļ� -- SPIģʽ
**
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng	
** Modified date:		2007-07-08
** Version:				V3.0
** Descriptions:		1. ��SPI_Clk400k()��SPI_ClkToMax()�������ϲ�Ϊһ������SdSpiHal_SetMCIClock()
**                      2. ������������  SdSpiHal_ ��Ϊ��ͷ
**						3. ���ӶԶ࿨��֧��
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/


#ifndef __SDSPIHAL_H__
#define __SDSPIHAL_H__


/*
****************************
   SPI ����ģʽ
****************************
*/

#if (!SDBUS_MODE_EN)

/***********************************************************
LPC2364/6 û��MCI������, ����û�취��SDģʽ��дSD/MMC��, ��������SPI

ģʽ��дSD/MMC��. ʹ�õ���������:

 LPC2364/6            ��������    SmartARM2300������JP1

P0.15(SCK0)  ----->  CLK/SCK        CLK

P0.17(MISO0) ----->  DAT0/DO        D0      

P0.18(MOSI0) ----->  CMD/DI         CMD

P2.13(����GPIO) -->  DAT3/CS        D3 

P0.21        ----->  (���ƿ�����)   PWR

P2.8         ----->  CARD_INSERT    CD

P3.25        ----->  CARD_WP        WP               

��ʹ��SmartARM2300, Ӳ����������;

1. ��������: P0.15 --> CLK(JP1), P0.17 --> D0(JP1), P0.18 --> CMD(JP1)

2. �̽�����: D3(JP1),  PWR(JP1),  WP(JP1),  CD(JP1)  

************************************************************/

/*************************************
   LPC2364/6 Ӳ������(ʹ��SPI����) 
**************************************/

/* SCK���� */
#define  SPI_SCK				(0x01 << 15)	                // P0.15					
#define  SPI_SCK_GPIO()			PINSEL0 &= ~(0x03U << 30)		/* ���� SCK ��ΪGPIO�� */
#define  SPI_SCK_OUT()			IO0DIR |= SPI_SCK				/* ���� SCK ��Ϊ����� */
#define	 SPI_SCK_CLR()			IO0CLR = SPI_SCK				/* �� SCK Ϊ�͵�ƽ */	

/* MISO ���� */	
#define  SPI_MISO				(0x01 << 17)                    // P0.17						    
#define  SPI_MISO_GPIO()		PINSEL1 &= ~(0x03 << 2)		    /* ���� MISO ��ΪGPIO�� */
#define  SPI_MISO_OUT()			IO0DIR |= SPI_MISO				/* ���� MISO ��Ϊ����� */
#define	 SPI_MISO_CLR()			IO0CLR = SPI_MISO				/* �� MISO Ϊ�͵�ƽ */

/* MOSI ���� */	
#define  SPI_MOSI				(0x01 << 18)                    // P0.18                     
#define  SPI_MOSI_GPIO()		PINSEL1 &= ~(0x03 << 4)	    	/* ���� MOSI ��ΪGPIO�� */
#define  SPI_MOSI_OUT()			IO0DIR |= SPI_MOSI				/* ���� MOSI ��Ϊ����� */
#define	 SPI_MOSI_CLR()			IO0CLR = SPI_MOSI				/* �� MISO Ϊ�͵�ƽ */

/* P2.13 Ϊ CS ���� */
#define  SPI_CS_GPIO()          PINSEL4 &= ~(0x03 << 26);
#define  SPI_CS_OUT()           FIO2DIR |= (0x01 << 13);        /* ���� CS ��Ϊ����� */
#define	 SPI_CS_SET()	        FIO2SET |= (0x01 << 13);	    /* �� CS Ϊ�ߵ�ƽ */
#define  SPI_CS_CLR()           FIO2CLR |= (0x01 << 13);        /* �� CS Ϊ�͵�ƽ */


/* ��ʼ�� IO ��ΪSPI�ӿ� */
#define  SPI_INIT()				{PINSEL0 &= ~(0x03U << 30); PINSEL1 &= ~((0x03 << 2) + (0x03 << 4));\
                                 PINSEL0 |= (0x03U << 30);  PINSEL1 |= (0x03 << 2) + (0x03 << 4);}  

/* P0.21 ��Դ�������� */
#define  SD_POWER				(0x01 << 21)	
#define  SD_POWER_GPIO()		PINSEL1 &= ~(0x03 << 10)		/* ���� POWER ��ΪGPIO�� */
#define  SD_POWER_OUT()			IO0DIR |= SD_POWER				/* ���� POWER ��Ϊ����� */
#define  SD_POWER_OFF()			IO0SET |= SD_POWER				/* �� POWER Ϊ�ߵ�ƽ */
#define  SD_POWER_ON()			IO0CLR |= SD_POWER				/* �� POWER Ϊ�͵�ƽ */


/* P2.8 ����ȫ���뿨��������� */
#define  SD_INSERT				(0x01 << 8)		
#define  SD_INSERT_GPIO()		PINSEL4 &= ~(0x03 << 16)		/* ���� INSERT ��ΪGPIO�� */	
#define  SD_INSERT_IN()			FIO2DIR &= ~SD_INSERT			/* ���� INSERT ��Ϊ����� */	
#define  SD_INSERT_STATUS()  	(FIO2PIN & SD_INSERT)			/* ��ȡ INSERT �ڵ�״̬ */

/* P3.25 ��д����������� */
#define  SD_WP					(0x01 << 25)		
#define  SD_WP_GPIO()			PINSEL7 &= ~(0x03 << 18);		/* ���� WP ��ΪGPIO�� */	
#define  SD_WP_IN()				FIO3DIR &= ~SD_WP				/* ���� WP ��Ϊ����� */	
#define  SD_WP_STATUS()  		(FIO3DIR & SD_WP)				/* ��ȡ WP �ڵ�״̬   */

																
// ��⿨�Ƿ���ȫ���� 
extern INT8U SdHal_CheckCard(sd_struct *sds);

// ��⿨�Ƿ�д����
extern INT8U SdHal_CheckCardWP(sd_struct *sds);

// ��ʼ�����ʿ���Ӳ����ʼ��
extern INT8U SdSpiHal_Initialize(sd_struct *sds);

// ���ö�дSD/MMC��ʱ��
extern void SdSpiHal_SetMCIClock(sd_struct *sds, INT32U ClockRate);

// ͨ��SPI����������һ���ֽڵ���
extern void SdSpiHal_SendByte(sd_struct *sds, INT8U byte);

// ͨ��SPI�������ӿ�����һ���ֽ�
extern INT8U SdSpiHal_RecByte(sd_struct *sds);

// ͨ��SPI��CS����ѡ��
extern void SdSpiHal_CSAssert(sd_struct *sds);

// ͨ��SPI��CS���Ų�ѡ��	
extern void SdSpiHal_CSDeassert(sd_struct *sds);

#endif


#endif