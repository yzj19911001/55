/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sddriver.h
** Last modified Date:	2007-5-19
** Last Version:		V2.0
** Descriptions:		sddriver.c ��ͷ�ļ�
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
** Modified date:		2005-3-10
** Version:				V2.0
** Descriptions:		�����˶�MMC����֧��,�����˶�UCOS-II��֧��,ʹ��ģ�鲻����������ǰ��̨ϵͳ,��������
**						��UCOS-II��
**------------------------------------------------------------------------------------------------------
** Modified by:         Ming Yuan Zheng
** Modified date:       2007-5-19
** Version:	            V3.0
** Descriptions:        1. �����˶�SD/MMC�� SD ����ģʽ��֧��(��ͬģʽ��API��������ͬ), ��API���������������sd_struct�ṹ��, ����֧�ֶ࿨��д
**
**                      2. SPIģʽ�º�SDģʽ�¹��� SDMMC_GetCardInfo() �� SDMMC_CalTimeout()����
**
**                      3. sd_struct�ṹ�������˳�Ա�����Ա����, ���ʿ���ʱ�Ӽ����ʿ������ʱ�Ӹ�Ϊ�ɶ�̬��ȡ
**
**                      4. SPI ģʽ�µ������ʹ��SpiCmd_ ��ͷ, SD ģʽ�µ�����ʹ�� SdCmd_ ��ͷ
**
**                      5. SPI ģʽ����Ӳ����صĺ���ʹ�� SdSpiHal_ ��ͷ, SD ģʽ����Ӳ����صĺ���ʹ�� SdHal_ ��ͷ
********************************************************************************************************/

#ifndef __SDDRIVER_H__
#define __SDDRIVER_H__

#include "config.h"
#include "sdcommon.h"

    	/*
		*************************************************************

				�û�API����:	��, д, �� SD��  API������������
 			
		*************************************************************
		*/
		
/* ������ error code */
#define   SD_NO_ERR			     	0x00			// ����ִ�гɹ�
#define   SD_ERR_NO_CARD		 	0x01			// ��û����ȫ���뵽������
#define   SD_ERR_USER_PARAM      	0x02			// �û�ʹ��API����ʱ����ڲ����д���
#define   SD_ERR_CARD_PARAM		 	0x03			// ���в����д����뱾ģ�鲻���ݣ�
#define	  SD_ERR_VOL_NOTSUSP        0x04			// ����֧��3.3V����
#define   SD_ERR_OVER_CARDRANGE		0x05			// ����������������Χ
#define   SD_ERR_UNKNOWN_CARD       0x06			// �޷�ʶ����

#define   SD_ERR_CURRENT_STATE      0x07            // ����ǰ״̬����ȷ(ֻ������SDģʽ)
#define   SD_ERR_READY_FOR_DATA     0x08            // �����ݻ�����δ����


/* SD������ܷ��صĴ����� */
#define   SD_ERR_CMD_RESPTYPE	 	0x10			// �������ʹ���
#define   SD_ERR_CMD_TIMEOUT     	0x11			// ��������Ӧ��ʱ
#define   SD_ERR_CMD_RESP		 	0x12			// ��������Ӧ����
#define   SD_ERR_CMD_NOTSUSP		0x13			// ��֧�ָ�����
#define   SD_ERR_CMD_RESPCRC        0x14			// ����CRC��Ӧ����


/* ������������ */
#define   SD_ERR_DATA_CRC16      	0x20			// ������CRC16У�鲻ͨ��
#define   SD_ERR_DATA_START_TOK		0x21			// ���������ʱ�����ݿ�ʼ���Ʋ���ȷ
#define	  SD_ERR_DATA_RESP		 	0x22			// д�������ʱ����������Ӧ���Ʋ���ȷ
#define   SD_ERR_DATA_STARTBIT      0x23            // д�������ʱ, ������ʼλ����ȷ


/* �ȴ������� */
#define   SD_ERR_TIMEOUT_WAIT    	0x30			// д�������ʱ��������ʱ����
#define   SD_ERR_TIMEOUT_READ    	0x31			// ��������ʱ����
#define	  SD_ERR_TIMEOUT_WRITE	 	0x32			// д������ʱ����
#define   SD_ERR_TIMEOUT_ERASE   	0x33			// ����������ʱ����
#define	  SD_ERR_TIMEOUT_WAITIDLE 	0x34			// ��ʼ����ʱ���ȴ����˳�����״̬��ʱ����


/* д�������ܷ��صĴ����� */
#define	  SD_ERR_WRITE_BLK			0x40			// д�����ݴ���
#define	  SD_ERR_WRITE_BLKNUMS      0x41			// д���ʱ����Ҫд��Ŀ�����ȷд��Ŀ�����һ��
#define   SD_ERR_WRITE_PROTECT		0x42			// ����ǵ�д�������ش���д����λ��


/* ʹ��UCOS-IIʱ���ܳ��ֵĴ��� */
#define   SD_ERR_CREATE_SEMSD		0xA0			// ����SD/MMC�������ź���ʧ��
#define   SD_ERR_WAIT_WRSEM         0xA1            // �ȴ���д�ź���ʧ��


/* SD_STRUCT�ṹ����ܳ��ֵĴ��� */
#define   SD_ERR_SD_STRUCT          0xB0            // SD_STRUCT�ṹ���Աֵ���ڴ���


/* �������ܵĴ��� */
#define   SD_ERR_HARDWARE           0xE0            // Ӳ������        


/**************************************************
	
			�����궨��

***************************************************/

#define   CARDTYPE_UNKNOWN			0x00
#define	  CARDTYPE_SD				0x01			// ����ΪSD ��
#define   CARDTYPE_MMC				0x02			// ����ΪMMC��
#define   CARDTYPE_SDHC              0x03           //����ΪSDHC��                                     */


/* for determin version, like card type   */
#define SD_V1     0x00                                        /* version 1                              */
#define SD_V2     0x01                                        /* version 2                              */
#define SD_NA     0x02                                        /* unkown                                 */

#define   SD_READREG_TIMEOUT		1				// ���Ĵ����ĳ�ʱʱ��
	

extern INT8U SD_Initialize(sd_struct *sds);
extern INT8U SD_WriteBlock(sd_struct *sds, INT32U blockaddr, INT8U *sendbuf);
extern INT8U SD_ReadBlock(sd_struct *sds, INT32U blockaddr, INT8U *recbuf);

extern INT8U SD_ReadMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *recbuf);
extern INT8U SD_WriteMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *sendbuf);
extern INT8U SD_EraseBlock(sd_struct *sds, INT32U startaddr, INT32U blocknum);
extern INT8U SD_Ver2ActiveInit(sd_struct *sds);

extern	INT8U ReadSDCardBlock(INT32U blockaddr, INT8U *recbuf);
extern	INT8U WriteSDCardBlock(INT32U blockaddr, INT8U *sendbuf);
extern	void SDCardInit(void);

	    /*
	    *************************************************************
	    
	     			����Ϊ������õ�����SD����صĺ궨��
	     	
	    *************************************************************
	    */
	    	    	    
/* Mask off the bits in the OCR corresponding to voltage range 3.2V to
 * 3.4V, OCR bits 20 and 21 */
#define MSK_OCR_33			 	0x00100000

/* �����ڳ�ʼ���׶�,�ȴ�SD���˳�����״̬�����Դ��� */
/* Number of tries to wait for the card to go idle during initialization */
#define SD_IDLE_WAIT_MAX     	8000

/* SD�����ʱʱ��(��λ 8clock)*/
/* timeout of command */
#define SD_CMD_TIMEOUT   	 	100

extern uint8 full_read_sd(uint32 sd_add, uint8 *buf);
extern uint8 full_write_sd(uint32 sd_add, uint8 *buf);
extern uint8 full_Mutil_Wr_sd(uint32 sd_add, uint8 *buf,uint8 Num);//дSD����������������ж���SD������Щ����	 

#endif            





