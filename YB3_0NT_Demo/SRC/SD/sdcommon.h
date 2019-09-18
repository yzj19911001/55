
#ifndef __SDCOMMON_H__
#define __SDCOMMON_H__

#include "config.h"

#ifdef  SDCOMMON_GLOBALS
#define SDCOMMON_EXT 
#else
#define SDCOMMON_EXT extern
#endif

/* SD����Ϣ�ṹ�嶨�� */ 
/* the information structure variable of SD Card*/
typedef struct SD_STRUCT
{	
	INT32U card_posnum;				/* ������,�࿨ʱʹ�� */
	INT8U card_type;                /* ���� */
	
	INT32U block_num;				/* ���п������ */
	INT32U block_len;				/* ���Ŀ鳤��(��λ:�ֽ�) */
	INT32U erase_unit;				/* һ�οɲ����Ŀ���� */
	
	INT32U timeout_read;			/* ���鳬ʱʱ��(��λ: 1 clock) */
	INT32U timeout_write;			/* д�鳬ʱʱ��(��λ: 1 clock) */
	INT32U timeout_erase;			/* ���鳬ʱʱ��(��λ: 1 clock) */
	
	INT32U RCA;                     /* ����RCA��ַ */
	INT32U host_clkval;             /* ����ʱ��ֵ(Hz)  */
	INT32U card_transpeed;          /* ������������ٶ�(Hz) */
	
#if SD_UCOSII_EN
	INT32U timeout_read_os;			/* UCOS-II��,���鳬ʱʱ��(��λ: os tick) */
	INT32U timeout_write_os;		/* UCOS-II��,д�鳬ʱʱ��(��λ: os tick) */
	INT32U timeout_erase_os;		/* UCOS-II��,���鳬ʱʱ��(��λ: os tick) */

	OS_EVENT *pSemSD;	            /* ������Ȩ���ź��� */
	OS_EVENT *pSemSdmmcRW;          /* ����д�ȴ��ź��� */
#endif 	
	
}sd_struct;
//SDCOMMON_EXT  sd_struct sd_info;

/* SD/MMC �����ݿ鳤�� */
#define SD_BLOCKSIZE_NBITS		9  
#define SD_BLOCKSIZE 			(1 << SD_BLOCKSIZE_NBITS)		


/* �ȴ������ĵȴ����� */
#define BUSY_TYPE_READ        0
#define BUSY_TYPE_RDY_DATA    1
#define BUSY_TYPE_PROG        2
#define BUSY_TYPE_ERASE       3      


/* ��д��ʱ��: ���ټ���׼�ٶ� */
#define   SD_RATE_SLOW		  1
#define   SD_RATE_NORMAL	  2


/*****************************************
   CSD �Ĵ�����ض���
*****************************************/
/* CSD��һЩ����ֽ�λ��(���ֽ���ǰ) */
#define TAAC_POS 				1			//TAAC
#define NSAC_POS				2			//NSAC

#define TRAN_SPEED_POS      	3           //TRAN_SPEED 

#define READ_BL_LEN_POS			5			//READ_BL_LEN

#define C_SIZE_POS1				6			//C_SIZE upper  2-bit
#define C_SIZE_POS2				7			//C_SIZE middle 8-bit
#define C_SIZE_POS3				8			//C_SIZE lower	2-bit

#define C_SIZE_MULT_POS1		9			//C_SIZE_MULT upper 2-bit
#define C_SIZE_MULT_POS2		10			//C_SIZE_MULT lower 1-bit	

#define C_SIZE_HC_POS1		    9			//C_SIZE_MULT upper 2-bit
#define C_SIZE_HC_POS2		   10			//C_SIZE_MULT lower 1-bit	

#define SECTOR_SIZE_POS1		10			//SECTOR_SIZE upper 5-bit
#define SECTOR_SIZE_POS2		11			//SECTOR_SIZE lower 2-bit

#define R2WFACTOR_POS 			12			//R2WFACTOR_POS

/**/
#define ERASE_GRP_SIZE_POS   	10			//MMC�� ERASE_GRP_SIZE 5-bit
#define ERASE_GRP_MULTI_POS1 	10			//MMC�� ERASE_GRP_MULTI 2-bit
#define ERASE_GRP_MULTI_POS2 	11			//MMC�� ERASE_GRP_MULTI 3-bit


//CSD��һЩ�������
#define TAAC_MSK				0x07		//TAAC ������
#define NSAC_MSK				0x78		//NSAC ������

#define TRAN_SPEED_MSK1    	 	0x07        //TRAN_SPEED �����ٶ�ָ������
#define TRAN_SPEED_MSK2     	0x78        //TRAN_SPEED �����ٶ�β�� 

#define READ_BL_LEN_MSK			0x0F		//READ_BL_LEN ������

#define C_SIZE_MSK1				0x03		//C_SIZE ��2λ����
#define C_SIZE_MSK3				0xC0		//C_SIZE ��2λ����

#define C_SIZE_MULT_MSK1 		0x03		//C_SIZE_MULT �ĸ�2λ����
#define C_SIZE_MULT_MSK2 		0x80		//C_SIZE_MULT �ĵ�2λ����

#define R2WFACTOR_MSK			0x1C		//R2WFACTOR ����

#define SECTOR_SIZE_MSK1		0x3F		//SECTOR_SIZE �ĸ�5λ
#define SECTOR_SIZE_MSK2		0x80		//SECTOR_SIZE �ĵ�2λ

#define ERASE_GRP_SIZE_MSK 		0x7C		//MMC�� ERASE_GRP_SIZE ����
#define ERASE_GRP_MULTI_MSK1 	0x03		//MMC�� ERASE_GRP_MULTI ��2λ����
#define ERASE_GRP_MULTI_MSK2 	0xE0		//MMC�� ERASE_GRP_NULTI ��3λ����


#endif

