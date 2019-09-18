/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdhal.h
** Last modified Date:	2007-7-15
** Last Version:		V1.0
** Descriptions:		SD/MMC����дģ�� ---- SDģʽ Ӳ��������ļ�ͷ�ļ�
**
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2007-7-15
** Version:				V1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:				
** Descriptions:		
**
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/

#ifndef __SDHAL_H__
#define __SDHAL_H__


/**************************************
   LPC23xx Ӳ������(ʹ��SD����) 
**************************************/

/*
    �ر�ע��: ���ʹ����DMA��ʽ��д��, ��д��ʱ, DMA��ռ��LPC23xx USB��DMA����(8K Bytes)!!
*/

#if SDBUS_MODE_EN
#define LPC23xx_MCI_DMA_EN	    1         /* ��(1)��(0)ʹ��LPC23xx��DMA����     ԭʼΪ0*/
#else
#define LPC23xx_MCI_DMA_EN	    1          /* ֵ�̶���0 */
#endif

#if SDBUS_MODE_EN


///* GPO_01 ��Դ�������� */
//#define  SD_POWER_BIT			(0x01 << 1)	
//#define  SD_POWER_OFF()			P3_OUTP_SET |= SD_POWER_BIT				/* �� POWER Ϊ�ߵ�ƽ */
//#define  SD_POWER_ON()			P3_OUTP_CLR |= SD_POWER_BIT				/* �� POWER Ϊ�͵�ƽ */

/* GPO_04 ��Դ�������� */
#define  SD_POWER_BIT			(0x01 << 4)	
#define  SD_POWER_OFF()			P3_OUTP_SET |= SD_POWER_BIT				/* �� POWER Ϊ�ߵ�ƽ */
#define  SD_POWER_ON()			P3_OUTP_CLR |= SD_POWER_BIT				/* �� POWER Ϊ�͵�ƽ */


///* GPI_04 ����ȫ���뿨���������CD */
//#define  SD_INSERT_BIT			(0x01 << 4)		
//#define  SD_INSERT_STATUS()  	(P3_INP_STATE & SD_INSERT_BIT)			/* ��ȡ INSERT �ڵ�״̬ */

/* GPI_25 ����ȫ���뿨���������CD */
#define  SD_INSERT_BIT			(0x01 << 25)		
#define  SD_INSERT_STATUS()  	(P3_INP_STATE & SD_INSERT_BIT)			/* ��ȡ INSERT �ڵ�״̬ */

///* GPI_09 ��д����������� */
//#define  SD_WP_BIT				(0x01 << 9)		
//#define  SD_WP_STATUS()  		(P3_INP_STATE & SD_WP_BIT)				/* ��ȡ WP �ڵ�״̬   */

/* GPIO_00 ��д����������� */
#define  SD_WP_BIT				(0x01 << 10)		
#define  SD_WP_STATUS()  		(P3_INP_STATE & SD_WP_BIT)				/* ��ȡ WP �ڵ�״̬   */


#if LPC23xx_MCI_DMA_EN

//#define DMA_SRC			0x7FD00000		/* This is the area original data is stored									
//#define DMA_DST			0x7FD01000		/* This is the area, after writing to the SD/MMC, data read from the SD/MMC card. */

//#define DMA_SRC			0x80000000		/* This is the area original data is stored	or data to be written to the SD/MMC card. */
//#define DMA_DST			0x81000000		//2012.01.11									

//#define DMA_MCIFIFO		0xE008C080		
#define DMA_MCIFIFO		0x20098080		//2012.01.17
#define DMA_SIZE		SD_BLOCKSIZE	/* DMA_SIZE is the same BLOCK_LENGTH defined in mci.h */

/* DMA mode */
#define M2M				0x00
#define M2P				0x01
#define P2M				0x02
#define P2P				0x03

extern INT32U DMA_Init(INT32U ChannelNum, INT32U DMAMode);

#endif


/*
****************************
   SD ����ģʽ
****************************
*/

/* SD/MMC Command list, per MMC spec. SD Memory Card Spec. Simplified version */
#define GO_IDLE_STATE		0		/* GO_IDLE_STATE(MMC) or RESET(SD) */
#define SEND_OP_COND		1		/* SEND_OP_COND(MMC) or ACMD41(SD) */
#define ALL_SEND_CID		2		/* ALL SEND_CID */
#define SET_RELATIVE_ADDR	3		/* SET_RELATE_ADDR */
#define SET_ACMD_BUS_WIDTH	6
#define SELECT_CARD			7		/* SELECT/DESELECT_CARD */
#define SEND_CSD			9		/* SEND_CSD */
#define STOP_TRANSMISSION	12		/* Stop either READ or WRITE operation */
#define SEND_STATUS			13		/* SEND_STATUS */
#define SET_BLOCK_LEN		16		/* SET_BLOCK_LEN */
#define READ_SINGLE_BLOCK	17		/* READ_SINGLE_BLOCK */
#define WRITE_BLOCK			24		/* WRITE_BLOCK */
#define SEND_APP_OP_COND	41		/* ACMD41 for SD card */		
#define APP_CMD				55		/* APP_CMD, the following will a ACMD */


#define EXPECT_NO_RESP		0
#define EXPECT_SHORT_RESP	1
#define EXPECT_LONG_RESP	2


#define MCLKDIV_SLOW	0xAA    // 0xff
#define MCLKDIV_NORMAL	0x1f//0x0f	//3.25MHz	// ��������SD���Ķ�дʱ�� //0x0f hong 8.8 Y//20130826 ��дSD�����ӻ��޸�

#define BLOCK_NUM			0x80	
 
#define FIFO_SIZE			16	   

/* MCI Status register bit information */
#define MCI_CMD_CRC_FAIL	1 << 0
#define MCI_DATA_CRC_FAIL	1 << 1
#define MCI_CMD_TIMEOUT		1 << 2
#define MCI_DATA_TIMEOUT	1 << 3
#define MCI_TX_UNDERRUN		1 << 4
#define MCI_RX_OVERRUN		1 << 5
#define MCI_CMD_RESP_END	1 << 6
#define MCI_CMD_SENT		1 << 7
#define MCI_DATA_END		1 << 8
#define MCI_START_BIT_ERR	1 << 9
#define MCI_DATA_BLK_END	1 << 10
#define MCI_CMD_ACTIVE		1 << 11
#define MCI_TX_ACTIVE		1 << 12
#define MCI_RX_ACTIVE		1 << 13
#define MCI_TX_HALF_EMPTY	1 << 14
#define MCI_RX_HALF_FULL	1 << 15
#define MCI_TX_FIFO_FULL	1 << 16
#define MCI_RX_FIFO_FULL	1 << 17
#define MCI_TX_FIFO_EMPTY	1 << 18
#define MCI_RX_FIFO_EMPTY	1 << 19
#define MCI_TX_DATA_AVAIL	1 << 20
#define MCI_RX_DATA_AVAIL	1 << 21

#define CMD_INT_MASK		(MCI_CMD_CRC_FAIL | MCI_CMD_TIMEOUT | MCI_CMD_RESP_END \
                             | MCI_CMD_SENT | MCI_CMD_ACTIVE)
                             
#define DATA_ERR_INT_MASK	(MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_TX_UNDERRUN \
			                 | MCI_RX_OVERRUN | MCI_START_BIT_ERR)
			
#define DATA_INT_MASK		(MCI_DATA_END | MCI_DATA_BLK_END | MCI_TX_ACTIVE \
			                 | MCI_RX_ACTIVE)
			                 
#define FIFO_INT_MASK		(MCI_TX_HALF_EMPTY | MCI_RX_HALF_FULL | MCI_TX_FIFO_FULL \
			               | MCI_RX_FIFO_FULL | MCI_TX_FIFO_EMPTY | MCI_RX_DATA_AVAIL \
			               | MCI_TX_DATA_AVAIL | MCI_RX_FIFO_EMPTY)
			               
#define DATA_TX_INT_MASK	MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_TX_UNDERRUN \
			| MCI_START_BIT_ERR | MCI_DATA_END | MCI_DATA_BLK_END | MCI_TX_ACTIVE
			
#define DATA_RX_INT_MASK	MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_RX_OVERRUN \
			| MCI_START_BIT_ERR | MCI_DATA_END | MCI_DATA_BLK_END | MCI_RX_ACTIVE


extern void SdHal_EnableMCIBusWidth(sd_struct *sds, INT32U bWidth);
extern INT8U SdHal_Initialize(sd_struct *sds);
extern void SdHal_OpenDrainCmd(sd_struct *sds, INT8U bOpenDrain);



extern void SdHal_SetMCIClock(sd_struct *sds, INT32U ClockRate);
extern INT8U SdHal_SendCmd(sd_struct *sds, INT8U cmd, INT32U argument, INT8U resptype, INT32U *resp, INT8U resplen);
extern INT8U SdHal_CheckCard(sd_struct *sds);
extern INT8U SdHal_CheckCardWP(sd_struct *sds);

extern INT8U SdHal_BeforeReadBlock(sd_struct *sds);
extern INT8U SdHal_ReadBlock(sd_struct *sds, INT8U *recbuf);

extern INT8U SdHal_BeforeWriteBlock(sd_struct *sds);
extern INT8U SdHal_WriteBlock(sd_struct *sds, INT8U *sendbuf);
extern INT8U SdHal_WaitReadyforData(sd_struct *sds);
extern INT8U SdHal_WaitBusy(sd_struct *sds, INT32U busytype);

extern void SdHal_EnableMCIBusWidth(sd_struct *sds, INT32U bWidth);
extern uint32 MCI_count;



/*
#if !SD_UCOSII_EN
extern void  __irq MCI_Handler(void);
#endif
*/

#if SD_UCOSII_EN
extern void  MCI_Handler(void);
#else
extern void  MCI_Handler(void);
#endif

#endif



#endif

