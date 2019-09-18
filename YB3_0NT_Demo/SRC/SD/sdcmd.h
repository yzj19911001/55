/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdcmd.h
** Last modified Date:	2005-3-11
** Last Version:		V2.0
** Descriptions:		SD/MMC ��дģ��: ����� ---- SD/MMC ��SPIģʽ֧�ֵ�����(ͷ�ļ�) 
**						Soft Packet of SD/MMC Card : commands that SD/MMC card supported 
**						in spi mode(header file)
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		SD/MMC ��дģ��: ����� ---- SD��SPIģʽ֧�ֵ����� 
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng
** Modified date:		2005-3-11
** Version:				V2.0
** Descriptions:		�����˶�MMC����֧��,��ʹ�ñ�ģ��ȿ���������ǰ��̨ϵͳ,Ҳ��������uCOS-II��.
**
**------------------------------------------------------------------------------------------------------
** Modified by:         Ming Yuan Zheng
** Modified date:       2007-5-19
** Version:	            V3.0
** Descriptions:        1. ��ԭ��SPIģʽ�µĺ����������޸�, ��Ϊ�� SpiCmd_ Ϊ��ͷ, ������������ SdSpi_ Ϊ��ͷ
**
**                      2. �����˶�SD����ģʽ��֧��, �������� SdCmd_ Ϊ��ͷ
**
********************************************************************************************************/

#ifndef __SDCMD_H__
#define __SDCMD_H__
#include "sddriver.h"
#include "sdcommon.h"


/*
 *******************************************************
	
		���ļ���Ҫ�� SD/MMC ���������������Ӧ�궨��
		
		����:  SD����ģʽ��SPIģʽ
	
 *******************************************************
*/


/*********************************************************************************
                    
                    SD/MMC �� SD����ģʽ�����Ӧ

**********************************************************************************/

#if    SDBUS_MODE_EN


/* ������Ӧ���� define command's response */
#define R0 		0								/* ����SDģʽ */
#define R1 		1
#define R1B 	2
#define R2 		3
#define R3	 	4
#define R6 		7
#define R7      8 
/*
*******************************

     SD/MMC�� ���

*******************************
*/

/******************************** ������� Basic command set **************************/
/* ��λSD �� Reset cards to idle state */
#define CMD0 0
#define CMD0_R R0


/* ������SD�����ϵ�SD����CID�Ĵ��� asks any device to send their CID numbers on the CMD Line*/
#define  CMD1   	1
#define	 CMD1_R	 	R3

/* ������SD�����ϵ�SD����CID�Ĵ��� asks any device to send their CID numbers on the CMD Line*/
#define CMD2 2
#define CMD2_R R2

/* Ҫ���豸����һ���µ���Ե�ַ asks the device to publish a new relative adderss */
#define CMD3 3
#define CMD3_R R6

/* ѡ�����ģʽ*/
#define	CMD6 6
#define CMD6_R R1

/* ѡ��/�Ͽ�ѡ�� select/deselect card */
#define	CMD7 7
#define CMD7_R R1

/* type currently unkown, for sdhc use */
#define CMD8 8
//#define CMD8_R 	R7  	//4.17-fan-�޸�ǰ
#define CMD8_R 	R1 	        //4.17-fan-�޸ĺ�

/* ��CSD�Ĵ��� Card sends the CSD */
#define CMD9 9
#define CMD9_R R2

/* ��CID�Ĵ��� Card sends CID */
#define CMD10 10
#define CMD10_R R2

/* ֹͣ�����ʱ�����ݴ��� Stop a multiple block (stream) read/write operation */
#define CMD12 12
#define CMD12_R R1B

/* �� Card_Status �Ĵ��� Get the addressed card's status register */
#define CMD13 13
#define CMD13_R R1

/* �����豸����ֹͣ״̬ set the device to inactive state */
#define CMD15 15
#define CMD15_R R0

/* ���ÿ�ĳ��� Set the block length */
#define CMD16 16
#define CMD16_R R1

/* ������ Read a single block */
#define CMD17 17
#define CMD17_R R1

/* �����,ֱ����������CMD12Ϊֹ Read multiple blocks until a CMD12 */
#define CMD18 18
#define CMD18_R R1

/***************************** ��д��� Block write commands *************************/
/* д���� Write a block of the size selected with CMD16 */
#define CMD24 24
#define CMD24_R R1

/* д��� Multiple block write until a CMD12 */
#define CMD25 25
#define CMD25_R R1

/* дCSD�Ĵ��� Program the programmable bits of the CSD */
#define CMD27 27
#define CMD27_R R1

/***************************** д���� Write protection *****************************/
/* Set the write protection bit of the addressed group */
#define CMD28 28
#define CMD28_R R1B

/* Clear the write protection bit of the addressed group */
#define CMD29 29
#define CMD29_R R1B

/* Ask the card for the status of the write protection bits */
#define CMD30 30
#define CMD30_R R1

/***************************** �������� Erase commands *******************************/
/* ���ò��������ʼ��ַ Set the address of the first write block to be erased */
#define CMD32 32
#define CMD32_R R1

/* ���ò��������ֹ��ַ Set the address of the last write block to be erased */
#define CMD33 33
#define CMD33_R R1


/* ���ò��������ʼ��ַ(��������MMC��) */
#define CMD35 35
#define CMD35_R R1

/* ���ò��������ֹ��ַ(��������MMC��) */
#define CMD36 36
#define CMD36_R R1


/* ������ѡ��Ŀ� Erase the selected write blocks */
#define CMD38 38
#define CMD38_R R1B

/***************************** �������� Lock Card commands ***************************/
/* ����/��λ���������/������ Set/reset the password or lock/unlock the card */
#define CMD42 42
#define CMD42_R	R1B
/* Commands from 42 to 54, not defined here */

/***************************** Ӧ������ Application-specific commands ****************/
/* ��ֹ��һ������ΪӦ������  Flag that the next command is application-specific */
#define CMD55 55
#define CMD55_R R1

/* Ӧ�������ͨ��I/O  General purpose I/O for application-specific commands */
#define CMD56 56
#define CMD56_R R1

/* ��OCR�Ĵ���,ֻ����SPIģʽ  Read the OCR (SPI mode only) */
#define CMD58 58
#define CMD58_R R3

/* ʹ�ܻ��ֹ CRC Turn CRC on or off */    //SD��û�д˹���
#define CMD59 59						   //SD��û�д˹���
#define CMD59_R R1						   //SD��û�д˹���

/***************************** Ӧ������ Application-specific commands ***************/
#define ACMD6 6
#define ACMD6_R R1

/* ��ȡ SD Status�Ĵ��� Get the SD card's status */
#define ACMD13 13
#define ACMD13_R R1

/* �õ���д�뿨�еĿ�ĸ��� Get the number of written write blocks (Minus errors ) */
#define ACMD22 22
#define ACMD22_R R1

/* ��д֮ǰ,����Ԥ�Ȳ����Ŀ�ĸ��� Set the number of write blocks to be pre-erased before writing */
#define ACMD23 23
#define ACMD23_R R1

/* ��ȡOCR�Ĵ��� Get the card's OCR (SD mode) */
#define ACMD41 41
#define ACMD41_R R3

/* ����/�Ͽ�CD/DATA[3]�����ϵ��������� Connect or disconnect the 50kOhm internal pull-up on CD/DAT[3] */
#define ACMD42 42
#define ACMD42_R R1

/* ��ȡSCR�Ĵ��� Get the SD configuration register */
#define ACMD51 51
#define ACMD51_R R1



/*
*  SD ����ģʽ��, Card_Status�Ĵ������״̬λ
*/
#define CARD_STATUS_ACMD_ENABLE		(1 << 5)			/* ������SD�� */
#define CARD_STATUS_ERASE_RESET		(1 << 13)

/* ������״̬���� */
#define CARD_STATUS_RDY_DATA		(0x01 << 8)

#define CARD_STATUS_CURRENT_STATE	(0x0F << 9)
#define CARD_STATUS_IDLE            (0x00 << 9)         /* ���� */
#define CARD_STATUS_READY          	(0x01 << 9)         /* ���� */
#define CARD_STATUS_IDENT          	(0x02 << 9)         /* ʶ�� */
#define CARD_STATUS_STBY          	(0x03 << 9)         /* ���� */
#define CARD_STATUS_TRAN         	(0x04 << 9)         /* ���ݷ��� */
#define CARD_STATUS_DATA          	(0x05 << 9)         /* ���� */
#define CARD_STATUS_REC         	(0x06 << 9)         /* ���ݽ��� */
#define CARD_STATUS_PRG         	(0x07 << 9)         /* ����� */
#define CARD_STATUS_DIS          	(0x08 << 9)         /* ���Ͽ� */


/* sdcmd.c�ļ������ĺ��� */
extern INT8U SdCmd_Go_Idle_State(sd_struct *sds);
extern INT8U SD_CheckSDVersion(sd_struct *sds);
extern INT8U SdCmd_Send_OP_Cond(sd_struct *sds);
extern INT8U SdCmd_Send_ACMD(sd_struct *sds);

extern INT8U SdCmd_SD_App_OP_Cond(sd_struct *sds);
extern INT8U SdCmd_All_Send_CID(sd_struct *sds, INT8U cidlen, INT8U *cidbuf);

extern INT8U SdCmd_Set_Relative_Addr(sd_struct *sds);
extern INT8U SdCmd_Send_CSD(sd_struct *sds, INT8U csdlen, INT8U *csdbuf);

extern INT8U SdCmd_Stop_Transmission(sd_struct *sds);
extern INT8U SdCmd_Send_Status(sd_struct *sds, INT32U *status);
extern INT8U SdCmd_Select_Card(sd_struct *sds);

extern INT8U SdCmd_Set_Bus_Width(sd_struct *sds, INT8U bWideBus);
extern INT8U SdCmd_Set_BlockLen(sd_struct *sds, INT32U blockLen);
extern INT8U SdCmd_Read_Single_Block(sd_struct *sds, INT32U blockadddr);
extern INT8U SdCmd_Read_Multiple_Block(sd_struct *sds, INT32U blockadddr);

extern INT8U SdCmd_Deselect_Card(sd_struct *sds, INT32U RCA);
extern INT8U SdCmd_Write_Single_Block(sd_struct *sds, INT32U blockadddr);
extern INT8U SdCmd_Write_Multiple_Block(sd_struct *sds, INT32U blockadddr);

extern INT8U SdCmd_Erase_Wr_Blk_Start(sd_struct *sds, INT32U blockadddr);
extern INT8U SdCmd_Erase_Wr_Blk_End(sd_struct *sds, INT32U blockadddr);
extern INT8U SdCmd_Erase(sd_struct *sds);

#endif // end of SDBUS_MODE_EN









/*********************************************************************************
                    
                    SD/MMC �� SPI ����ģʽ�����Ӧ

**********************************************************************************/

#if (!SDBUS_MODE_EN)

/* R1��R2���ֽڴ����� R1 and upper byte of R2 error code */
#define MSK_IDLE          		  0x01
#define MSK_ERASE_RST     		  0x02
#define MSK_ILL_CMD       		  0x04
#define MSK_CRC_ERR       		  0x08
#define MSK_ERASE_SEQ_ERR 		  0x10
#define MSK_ADDR_ERR      		  0x20
#define MSK_PARAM_ERR     		  0x40


/* R2���ֽڴ����� lower byte of R2 error code */
#define MSK_TOK_ERROR             0x01
#define MSK_TOK_CC_ERROR          0x02
#define MSK_TOK_ECC_FAILED        0x04
#define MSK_TOK_CC_OUTOFRANGE     0x08
#define MSK_TOK_CC_LOCKED         0x10


/* �������� Data Tokens */
#define SD_TOK_READ_STARTBLOCK    0xFE
#define SD_TOK_WRITE_STARTBLOCK   0xFE
#define SD_TOK_READ_STARTBLOCK_M  0xFE
#define SD_TOK_WRITE_STARTBLOCK_M 0xFC
#define SD_TOK_STOP_MULTI         0xFD

/* ������Ӧ���� Data Response Tokens */
#define SD_RESP_DATA_MSK		  0x0F		//������Ӧ����
#define SD_RESP_DATA_ACCETPTED	  0x05		//���ݱ�����
#define SD_RESP_DATA_REJECT_CRC	  0x0B      //����CRC��������ܾ�
#define SD_RESP_DATA_REJECT_WRITE 0x0D		//����д��������ܾ�

/* �ȴ����� Wait Type */
#define SD_WAIT_READ			  0x00		//���ȴ�
#define SD_WAIT_WRITE			  0x01		//д�ȴ�
#define SD_WAIT_ERASE		 	  0x02		//�����ȴ�



/* ������Ӧ���� define command's response */
#define R1    1
#define R1B   2
#define R2    3
#define R3    4


/*
*********************************************

     SD/MMC��SPIģʽ�����

*********************************************
*/

/******************************** ������� Basic command set **************************/
/* ��λSD �� Reset cards to idle state */
#define CMD0 0
#define CMD0_R R1

/* ��OCR�Ĵ��� Read the OCR (MMC mode, do not use for SD cards) */
#define CMD1 1
#define CMD1_R R1

/* ѡ�����ģʽ*/
#define	CMD6 6
#define CMD6_R R1

/* ��CSD�Ĵ��� Card sends the CSD */
#define CMD9 9
#define CMD9_R R1

/* ��CID�Ĵ��� Card sends CID */
#define CMD10 10
#define CMD10_R R1

/* ֹͣ�����ʱ�����ݴ��� Stop a multiple block (stream) read/write operation */
#define CMD12 12
#define CMD12_R R1B

/* �� Card_Status �Ĵ��� Get the addressed card's status register */
#define CMD13 13
#define CMD13_R R2

/***************************** ������ Block read commands **************************/

/* ���ÿ�ĳ��� Set the block length */
#define CMD16 16
#define CMD16_R R1

/* ������ Read a single block */
#define CMD17 17
#define CMD17_R R1

/* �����,ֱ����������CMD12Ϊֹ Read multiple blocks until a CMD12 */
#define CMD18 18
#define CMD18_R R1

/***************************** ��д��� Block write commands *************************/
/* д���� Write a block of the size selected with CMD16 */
#define CMD24 24
#define CMD24_R R1

/* д��� Multiple block write until a CMD12 */
#define CMD25 25
#define CMD25_R R1

/* дCSD�Ĵ��� Program the programmable bits of the CSD */
#define CMD27 27
#define CMD27_R R1

/***************************** д���� Write protection *****************************/
/* Set the write protection bit of the addressed group */
#define CMD28 28
#define CMD28_R R1B

/* Clear the write protection bit of the addressed group */
#define CMD29 29
#define CMD29_R R1B

/* Ask the card for the status of the write protection bits */
#define CMD30 30
#define CMD30_R R1

/***************************** �������� Erase commands *******************************/
/* ���ò��������ʼ��ַ(ֻ����SD��) Set the address of the first write block to be erased(only for SD) */
#define CMD32 32
#define CMD32_R R1

/* ���ò��������ֹ��ַ(ֻ����SD��) Set the address of the last write block to be erased(only for SD) */
#define CMD33 33
#define CMD33_R R1

/* ���ò��������ʼ��ַ(ֻ����MMC��) Set the address of the first write block to be erased(only for MMC) */
#define CMD35 35
#define CMD35_R R1

/* ���ò��������ֹ��ַ(ֻ����MMC��) Set the address of the last write block to be erased(only for MMC) */
#define CMD36 36
#define CMD36_R R1

/* ������ѡ��Ŀ� Erase the selected write blocks */
#define CMD38 38
#define CMD38_R R1B

/***************************** �������� Lock Card commands ***************************/
/* ����/��λ���������/������ Set/reset the password or lock/unlock the card */
#define CMD42 42
#define CMD42_R	R1B
/* Commands from 42 to 54, not defined here */

/***************************** Ӧ������ Application-specific commands ****************/
/* ��ֹ��һ������ΪӦ������  Flag that the next command is application-specific */
#define CMD55 55
#define CMD55_R R1

/* Ӧ�������ͨ��I/O  General purpose I/O for application-specific commands */
#define CMD56 56
#define CMD56_R R1

/* ��OCR�Ĵ���  Read the OCR (SPI mode only) */
#define CMD58 58
#define CMD58_R R3

/* ʹ�ܻ��ֹ CRC Turn CRC on or off */
#define CMD59 59
#define CMD59_R R1

/***************************** Ӧ������ Application-specific commands ***************/
/* ��ȡ SD Status�Ĵ��� Get the SD card's status */
#define ACMD13 13
#define ACMD13_R R2

/* �õ���д�뿨�еĿ�ĸ��� Get the number of written write blocks (Minus errors ) */
#define ACMD22 22
#define ACMD22_R R1

/* ��д֮ǰ,����Ԥ�Ȳ����Ŀ�ĸ��� Set the number of write blocks to be pre-erased before writing */
#define ACMD23 23
#define ACMD23_R R1

/* ��ȡOCR�Ĵ��� Get the card's OCR (SD mode) */
#define ACMD41 41
#define ACMD41_R R1

/* ����/�Ͽ�CD/DATA[3]�����ϵ��������� Connect or disconnect the 50kOhm internal pull-up on CD/DAT[3] */
#define ACMD42 42
#define ACMD42_R R1

/* ��ȡSCR�Ĵ��� Get the SD configuration register */
#define ACMD51 51
#define ACMD51_R R1


extern INT8U SdSpi_SendCmd(sd_struct *sds, INT8U cmd, INT8U *param, INT8U resptype, INT8U *resp);
extern void SdSpi_PackParam(INT8U *parameter, INT32U value);							// ��װ����

extern INT8U SdSpi_BlockCommand(sd_struct *sds, INT8U cmd, INT8U resptype, INT32U parameter);
 																					    // ������
	
extern INT8U SpiCmd_Go_Idle_State(sd_struct *sds);										// ��λSD��		
extern INT8U SpiCmd_Send_OP_Cond(sd_struct *sds);

extern INT8U SpiCmd_Send_CSD(sd_struct *sds, INT8U csdlen, INT8U *recbuf);			    // ��CSD 			
extern INT8U SpiCmd_Send_CID(sd_struct *sds, INT8U cidlen, INT8U *recbuf);				// ��CID			
extern INT8U SpiCmd_Stop_Transmission(sd_struct *sds);									// ֹͣ����			

extern INT8U SpiCmd_Send_Status(sd_struct *sds, INT8U len, INT8U *buffer);			    // ��Card Status
extern INT8U SpiCmd_Set_BlockLen(sd_struct *sds, INT32U length);						// ���ÿ鳤��

extern INT8U SpiCmd_Read_Single_Block(sd_struct *sds, INT32U blockaddr);				// ������
extern INT8U SpiCmd_Read_Multiple_Block(sd_struct *sds, INT32U blockaddr);				// �����

extern INT8U SpiCmd_Write_Single_Block(sd_struct *sds, INT32U blockaddr);				// д����
extern INT8U SpiCmd_Write_Multiple_Block(sd_struct *sds, INT32U blockaddr);				// д���

extern INT8U SpiCmd_Program_CSD(sd_struct *sds, INT8U len, INT8U *buff);				// дCSD�Ĵ��� 

extern INT8U SdCmd_Erase_Wr_Blk_Start(sd_struct *sds, INT32U startblock);			    // ������ʼ��ַ		
extern INT8U SdCmd_Erase_Wr_Blk_End(sd_struct *sds, INT32U endblock);				    // ������ֹ��ַ
extern INT8U SdCmd_Erase(sd_struct *sds);											    // ������ѡ�Ŀ�
    
extern INT8U SpiCmd_ReadOCR(sd_struct *sds, INT32U *ocrbuf);                            // ��OCR         
extern INT8U SpiCmd_CRC_ON_OFF(sd_struct *sds, INT8U bEnable);						    // ʹ��CRCУ��

// ����Ӧ������
extern	INT8U SpiCmd_ReadSD_Status(sd_struct *sds, INT8U sdslen, INT8U *recbuf);	    // ��SD_Status	
extern INT8U SpiCmd_Send_SCR(sd_struct *sds, INT8U scrlen, INT8U *recbuf);			    // ��SCR
extern INT8U SpiCmd_Send_Num_WR_Blcoks(sd_struct *sds, INT32U *blocknum);			    // �õ���ȷд����� 


// ����������
extern INT8U SdSpi_ReadRegister(sd_struct *sds, INT32U len, INT8U *recbuf);						// ���Ĵ���
extern INT8U SdSpi_ReadBlockData(sd_struct *sds, INT32U len, INT8U *recbuf);					// ��������
extern INT8U SdSpi_WriteBlockData(sd_struct *sds, INT8U bmulti, INT32U len, INT8U *sendbuf);	// д������
	
// ��������
extern void SdSpi_StopMultiToken(sd_struct *sds);					// ֹͣ���д����	 
extern INT8U SdSpi_WaitBusy(sd_struct *sds, INT8U waittype);		// æ
extern void SdSpi_Delay(sd_struct *sds, INT8U value);		        // ����SPIʱ��


//--------------hena
extern INT8U SdCmd_Speed_Mode(sd_struct *sds);
#endif  // end of !SDBUS_MODE_EN



#endif // end of __SDCMD_H__
























