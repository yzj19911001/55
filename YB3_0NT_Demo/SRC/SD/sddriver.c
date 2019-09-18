										 /****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sddriver.c
** Last modified Date:	2007-5-19
** Last Version:		V2.0
** Descriptions:		SD/MMC����д�����: �ṩ���û�API����: ��ʼ��,��,д,����
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


#include "sdconfig.h"
#include "sdcommon.h"
#include "sddriver.h"
#include "config.h"
#include "WT_Task.h"

#define OCR_HC_CCS              0x40000000
#define	GetSDInsertStatus		(P3_INP_STATE & SD_INSERT_BIT)		//SD������״̬


/* ��ʱʱ�䵥λ��(��λ:0.000000001ns) timeout unit table */
const INT32U time_unit[8] = {1000000000,100000000,10000000,
							 1000000,100000,10000,1000,100};

/* ��ʱʱ��� timeout value table */							 
const INT8U time_value[16] = {0,10,12,13,15,20,25,30,
                              35,40,45,50,55,60,70,80};
 
/* ��ʱʱ�������� timeout factor table */                              
const INT8U r2w_fator[8] = {1,2,4,8,16,32,64,128};

/* ������ٶ�β����, ԭΪС��, ���� 10 ת��Ϊ���� */
const INT8U tr_spval[16] = {0, 10, 12, 13, 15, 20, 25, 30, 35,
                          40, 45, 50, 55, 60, 70, 80};

/* ������ٶ�ָ����, ��λ clock, ��100K ~ 100M */
const INT32U tr_spexp[4] = {100000, 1000000, 10000000, 100000000};



static sd_struct sd_info;                          
	/*       
	***************************************************************************************************
		
	     �û�API����:  ��ʼ��,��,д,�� SD/MMC��
				
	***************************************************************************************************
	*/

void SD_RequestOSSem(sd_struct *sds);
void SD_ReleaseOSSem(sd_struct *sds);
INT8U SDMMC_GetCardInfo(sd_struct *sds);
INT8U SDMMC_CalTimeout(sd_struct *sds);


/*
*****************************************************************************
   ���������SD����ģʽ, ������LPC2368, LPC2378 �ȴ��� SD ��������΢������
*****************************************************************************
*/

#if SDBUS_MODE_EN

INT8U SDMMC_ConfigCard(sd_struct *sds);
INT8U SDMMC_IdentifyCard(sd_struct *sds);


//==============================
/*SD��д���麯��*/
INT8U ReadSDCardBlock(INT32U blockaddr, INT8U *recbuf)
{
	uint8 sdreadstat;

	sdreadstat=SD_ReadBlock(&sd_info, blockaddr, recbuf);
	if(sdreadstat!=0x00)
	{
		OSTimeDly(10);//��Ϊ10*2 20170109
		sdreadstat=SD_ReadBlock(&sd_info, blockaddr, recbuf);
	}
	if(sdreadstat!=0x00)
	{
		OSTimeDly(10);//��Ϊ10*2 20170109
		sdreadstat=SD_ReadBlock(&sd_info, blockaddr, recbuf);
	}
	
	return	sdreadstat;
		
}

//==============================
//==============================
/*SD�������麯��*/
INT8U WriteSDCardBlock(INT32U blockaddr, INT8U *sendbuf)
{
	uint8 sdwritestat;
	sdwritestat=SD_WriteBlock(&sd_info, blockaddr, sendbuf);

	return	sdwritestat;
}

//==============================
/*SD����ʼ������*/
void SDCardInit(void)
{
	uint8	l_u8Status;
	P2_DIR_CLR = (1<<25);
	l_u8Status	= SD_Initialize(&sd_info);                              /* ��ʼ��SD��                   */
	if (l_u8Status != SD_NO_ERR) 
	{
		BuzzerBeep(2,50);																								//��ʼ��ʧ�ܣ�����������ʾ
	}
	else
	{
	  BuzzerBeep(1,50);
	}
    micIrqFuncSet(13, 3, (uint32)MCI_Handler);												/* ����SD/MMC�����������ж����� */
}


/*******************************************************************************************************************
** ��������: SD_Initialize()				
**
** ��������: SD/MMC ����ʼ��: ��λ��,ʶ��,��ȡ�������Ϣ,ʹ����ʶ��ģʽ�������ݴ���ģʽ 
**
** �䡡  ��: sd_struct *sds: SD/MMC����Ϣ�ṹ�� 
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SD_Initialize(sd_struct *sds)

{
	INT8U ret, cidbuf[16];
	
	if (sds == NULL) return SD_ERR_USER_PARAM;          /* ������ڲ������� */  
		
#if SD_UCOSII_EN
	if (sds->pSemSD == NULL)
	{
    	sds->pSemSD = OSSemCreate(1);					/* �������ʿ��ź��� */ 
		if (sds->pSemSD == NULL)					
		 	return SD_ERR_CREATE_SEMSD;	   				
	}
	
	if (sds->pSemSdmmcRW == NULL)
	{
    	sds->pSemSdmmcRW = OSSemCreate(0); 				/* ������д�����ȴ��ź��� */ 
		if (sds->pSemSdmmcRW == NULL)	   				
		 	return SD_ERR_CREATE_SEMSD;	   				
	}					    
#endif
	SD_RequestOSSem(sds);				   				/* �� OS �����ȡ���ʿ��ź��� */
	
	if (sds->RCA != 0)  sds->RCA = 0;
	
	SdHal_Initialize(sds);				   			    /* HAL���ʼ�� */
 	
 	
	ret = SdCmd_Go_Idle_State(sds);		   				/* CMD0: ��λ��, ʹ���������״̬ */
	if (ret != SD_NO_ERR)	
		goto SDINIT_END;

	
	ret = SD_CheckSDVersion(sds);                       /* CMD8:ʶ���ͣ�������������  */
                                                        /* ͨ��                         */

    if (ret == SD_V1)																		
    {																					
        ret = SDMMC_IdentifyCard(sds);                  /* CMD1��ACMD41: ʶ��, ����� */
                                                        /* ��״̬                       */
        if (ret != SD_NO_ERR)
        goto SDINIT_END;
    }																					
    else if (ret == SD_V2)
    {
        ret = SD_Ver2ActiveInit(sds);                   /* ACMD41: ʶ��, �������״̬ */
        
        if (ret != SD_NO_ERR)
        {
            goto SDINIT_END;
        }
    }
    else
    {
        ret = SD_ERR_UNKNOWN_CARD;
       
        goto SDINIT_END;
    } 
	  
	ret = SdCmd_All_Send_CID(sds, 16, cidbuf);          /* CMD2: ��CID, ����ʶ��״̬ */
    if (ret != SD_NO_ERR)
	    goto SDINIT_END;
	    
	ret = SdCmd_Set_Relative_Addr(sds);	   				/* CMD3: ��ȡRCA, �������״̬ */
   	if (ret != SD_NO_ERR)	
   		goto SDINIT_END;
 
 
  
    ret = SDMMC_GetCardInfo(sds);		   				/* CMD9: ��ȡCSD, ��ȡ�������� */
    if (ret != SD_NO_ERR)
    	goto SDINIT_END;

	ret = SDMMC_CalTimeout(sds);		   				/* ���㳬ʱʱ��ֵ */
	if (ret != SD_NO_ERR)
	    goto SDINIT_END;


	SdHal_SetMCIClock(sds, SD_RATE_NORMAL);  			/* ���÷��ʿ���clockΪ��׼clock */		
									    
  	ret = SDMMC_ConfigCard(sds);		   				/* ���ÿ���ز���, ���ڴ���״̬   */
    if (ret != SD_NO_ERR)
        goto SDINIT_END;
		
	 
SDINIT_END:
	SD_ReleaseOSSem(sds);	                            	/* �ͷŷ��ʿ��ź��� */
	return ret;  	
}


/*******************************************************************************************************************
** ��������: SD_ReadBlock()				
**
** ��������: ��SD/MMC���ж���һ�����ݿ�
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT32U blockaddr: �Կ�Ϊ��λ�Ŀ��ַ, ����, ����ʼ��0 ~ 511�ֽ�Ϊ���ַ0, 512 ~ 1023�ֽڵĿ��ַΪ1
**	         
** �� �� ��: INT8U *recbuf   : ���ջ�����,���ȹ̶�Ϊ 512 �ֽ�	
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SD_ReadBlock(sd_struct *sds, INT32U blockaddr, INT8U *recbuf)
{
	INT8U ret;

    if ((sds == NULL) || (recbuf == NULL))          
        return SD_ERR_USER_PARAM;                   /* ������ڲ������� */ 

	SD_RequestOSSem(sds);							/* ��OS������ʿ��ź��� */
	
	if (blockaddr > sds->block_num)	
	{
		ret = SD_ERR_OVER_CARDRANGE;				/* ����������������Χ */
	    goto SDRD_BLK_END;
	}
	
	ret = SdCmd_Select_Card(sds);					/* CMD7: ���봫��״̬  */
    if (ret != SD_NO_ERR)
    	goto SDRD_BLK_END;
  			
	ret = SdHal_BeforeReadBlock(sds);               /* ���Ͷ�����֮ǰ��׼������ */
	if (ret != SD_NO_ERR)
		goto SDRD_BLK_END;
	
	ret = SdCmd_Read_Single_Block(sds, blockaddr);	/* CMD17: ���Ͷ��������� */						
 	if (ret != SD_NO_ERR)
 		goto SDRD_BLK_END;	
	
	ret = SdHal_ReadBlock(sds, recbuf);             /* �ӿ��������� */
	if (ret != SD_NO_ERR)
		goto SDRD_BLK_END;
  	
SDRD_BLK_END:
    SdCmd_Deselect_Card(sds, 0);				/* CMD7: �˳�����״̬ */
	SD_ReleaseOSSem(sds);                           /* �ͷŷ��ʿ��ź��� */
    return ret;						
}


/*******************************************************************************************************************
** ��������: SD_ReadMultiBlock()				
**
** ��������: ��SD/MMC���ж���������ݿ�
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT32U blockaddr: �Կ�Ϊ��λ�Ŀ��ַ
**	         INT32U blocknum : Ҫ��ȡ�Ŀ�ĸ���
**
** �� �� ��: INT8U *recbuf   : ���ջ�����, ����512 * blocknum �ֽ�	
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
#if SD_ReadMultiBlock_EN
INT8U SD_ReadMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *recbuf)
{
	INT8U ret;
	INT32U i;
	
    if ((sds == NULL) || (recbuf == NULL)) 
        return SD_ERR_USER_PARAM;                   /* ������ڲ������� */	
	
	SD_RequestOSSem(sds);							/* ��OS������ʿ��ź��� */

	if (blockaddr > sds->block_num)	
	{
		ret = SD_ERR_OVER_CARDRANGE;				/* ����������������Χ */
	    goto SDRD_MBLK_END;
	}

	ret = SdCmd_Select_Card(sds);					/* CMD7: ���봫��״̬  */
    if (ret != SD_NO_ERR)
    	goto SDRD_MBLK_END;
   		
	ret = SdHal_BeforeReadBlock(sds);               /* ���Ͷ�����֮ǰ��׼������ */    
	if (ret != SD_NO_ERR)
		goto SDRD_MBLK_END;
			
	ret = SdCmd_Read_Multiple_Block(sds, blockaddr);/* ���Ͷ�������� */						
 	if (ret != SD_NO_ERR)
 		goto SDRD_MBLK_END;
	
    for (i = 0; i < blocknum; i++)
    {		
    	ret = SdHal_ReadBlock(sds, recbuf);         /* �������Կ������� */
    	if (ret == SD_NO_ERR)
       		recbuf = recbuf + SD_BLOCKSIZE;
    	else
            goto SDRD_MBLK_END; 
        
        if (i < (blocknum - 1))
        {
            ret = SdHal_BeforeReadBlock(sds);       /* �������Կ�����֮ǰ��׼������ */  
	        if (ret != SD_NO_ERR)
		        goto SDRD_MBLK_END;      
        }
    }
 
  	ret = SdCmd_Stop_Transmission(sds);             /* ֹͣ���� */
//  	if (ret != SD_NO_ERR)
//		goto SDRD_MBLK_END;  	

SDRD_MBLK_END:
    SdCmd_Deselect_Card(sds, 0);				/* CMD7: �˳�����״̬ */
	SD_ReleaseOSSem(sds);
    return ret;						
}
#endif


/*******************************************************************************************************************
** ��������: SD_WriteBlock()				
**
** ��������: ��SD/MMC����д��һ����	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT32U blockaddr: �Կ�Ϊ��λ�Ŀ��ַ, ����, ����ʼ��0 ~ 511�ֽ�Ϊ���ַ0, 512 ~ 1023�ֽڵĿ��ַΪ1
**           INT8U *sendbuf  : ���ͻ�����,���ȹ̶�Ϊ 512 �ֽ�	
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SD_WriteBlock(sd_struct *sds, INT32U blockaddr, INT8U *sendbuf)
{
	INT8U ret;

    if ((sds == NULL) || (sendbuf == NULL)) 
        return SD_ERR_USER_PARAM;                    /* ������ڲ������� */

	SD_RequestOSSem(sds);							 /* ��OS������ʿ����ź��� */
	
	if (blockaddr > sds->block_num)	
	{
		ret = SD_ERR_OVER_CARDRANGE;				 /* ����������������Χ */
		goto SDWR_BLK_END;
	}
	
	ret = SdCmd_Select_Card(sds);					 /* CMD7: ���봫��״̬  */
    if (ret != SD_NO_ERR)
        goto SDWR_BLK_END;
 	
	ret = SdHal_BeforeWriteBlock(sds);               /* д����֮ǰ��׼������ */
	if (ret != SD_NO_ERR)
		goto SDWR_BLK_END;		
 
	ret = SdCmd_Write_Single_Block(sds, blockaddr);	 /* CMD24, ����д�������� */						
 	if (ret != SD_NO_ERR)
 		goto SDWR_BLK_END;
  
	ret = SdHal_WriteBlock(sds, sendbuf);            /* ������д������ */   
	if (ret != SD_NO_ERR)
		goto SDWR_BLK_END;
		
	ret = SdHal_WaitBusy(sds, BUSY_TYPE_PROG);      /* �ȴ��������� */
	if (ret != SD_NO_ERR)
        goto SDWR_BLK_END;
           
SDWR_BLK_END:
    SdCmd_Deselect_Card(sds, 0);              /* CMD7: �˳�����״̬ */  
	SD_ReleaseOSSem(sds);
	return ret;                                     /* ���ز������ */	
}


/*******************************************************************************************************************
** ��������: SD_WriteMultiBlock()				
**
** ��������: ��SD/MMC����д�������ݿ�	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT32U blockaddr: �Կ�Ϊ��λ�Ŀ��ַ
**           INT32U blocknum : Ҫд��Ŀ�ĸ���
**           INT8U *sendbuf  : ���ͻ�����, ���� 512 * blocknum �ֽ�	
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
INT8U SD_WriteMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *sendbuf)
{
	INT8U ret;
	INT32U i;

    if ((sds == NULL) || (sendbuf == NULL)) 
        return SD_ERR_USER_PARAM;                       /* ������ڲ������� */

	SD_RequestOSSem(sds);                               /* ��OS������ʿ��ź��� */


	ret = SdCmd_Select_Card(sds);                       /* CMD7: ���봫��״̬  */
    if (ret != SD_NO_ERR)
        goto SDWR_MBLK_END;
 	
	ret = SdHal_BeforeWriteBlock(sds);                  /* ��������ǰ��׼������ */
	if (ret != SD_NO_ERR)
		goto SDWR_MBLK_END;		
 
	ret = SdCmd_Write_Multiple_Block(sds, blockaddr);   /* CMD25, ����д������� */						
 	if (ret != SD_NO_ERR)
 		goto SDWR_MBLK_END;

    for (i = 0; i < blocknum; i++)
    {
		ret = SdHal_WriteBlock(sds, sendbuf);           /* д������ */
 		if (ret == SD_NO_ERR)
 		{	
 			sendbuf = sendbuf + SD_BLOCKSIZE;
 		}
 		else
 		{												/* дʧ��  */
			SdCmd_Stop_Transmission(sds);				/* ֹͣ���ݴ��䲢�ȴ�æ���� */	
			SdHal_WaitBusy(sds, BUSY_TYPE_PROG);
			goto SDWR_MBLK_END;
		}
		
		ret = SdHal_WaitReadyforData(sds);              /* �ȴ����ɽ������� */
        if (ret != SD_NO_ERR)
		   goto SDWR_MBLK_END;	
		
		if (i < (blocknum - 1))
		{		
 		    ret = SdHal_BeforeWriteBlock(sds);          /* �ٴ�׼���������� */
	        if (ret != SD_NO_ERR)
		        goto SDWR_MBLK_END;		    
	    }
	}

    ret = SdCmd_Stop_Transmission(sds);                 /* ֹͣ���ݴ��� */
    if (ret != SD_NO_ERR)
	    goto SDWR_MBLK_END;

    ret = SdHal_WaitBusy(sds, BUSY_TYPE_PROG);          /* �ȴ�����̽��� */
    if (ret != SD_NO_ERR)
        goto SDWR_MBLK_END;

SDWR_MBLK_END:
    SdCmd_Deselect_Card(sds, 0);					/* CMD7, �˳�����״̬ */  
	SD_ReleaseOSSem(sds);
	return ret;
}
#endif

/*******************************************************************************************************************
** ��������: SD_Ver2ActiveInit()
**
** ��������: SD2.00 SDHC cardʶ��
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SD_Ver2ActiveInit(sd_struct *sds)
{
    INT32U resp[4]={0},ret;
    INT32U retry;
    retry = 0;
    do
    {
        ret = SdHal_SendCmd(sds, CMD55, 0x00000000, CMD55_R, resp, 0x01);
                                                                        /* CMD55:�¸�����ΪӦ�ó������� */
        if (ret != SD_NO_ERR)
        {
            return ret;
        }
        ret = SdHal_SendCmd(sds, ACMD41, 0x40FF8000, ACMD41_R, resp, 0x01);
                                                                        /* ACMD41:������OCR������Ӧ     */
        if (ret != SD_NO_ERR)
        {
            return SD_ERR_UNKNOWN_CARD;                                 /* ����ʶ��                   */
        }
        retry++;
        if(retry > SD_IDLE_WAIT_MAX )                                   /* ��ʱ����                     */
        {
            return SD_ERR_TIMEOUT_WAITIDLE;
        }

    }while ((resp[0] & 0x80000000) == 0);                               /* �ȴ���ʼ�����HCS = 1        */

    if ((resp[0] & OCR_HC_CCS) == 0)
    {
        sds->card_type = CARDTYPE_SD;                                   /* ����ΪSD��                   */
    }
    else
    {
        sds->card_type = CARDTYPE_SDHC;                                 /* ���� HCSD ��                 */
    }

    return SD_NO_ERR;
}
/*******************************************************************************************************************/
/*******************************************************************************************************************
** ��������: SDMMC_IdentifyCard()		
**
** ��������: ʶ��: SD, MMC, �򲻿ɿ���
**
** �䡡  ��: sd_struct *sds: SD/MMC����Ϣ�ṹ�� 
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: ִ�к�, ���ͱ�����sds->card_type��
********************************************************************************************************************/
INT8U SDMMC_IdentifyCard(sd_struct *sds)
{
	if (SdCmd_Send_OP_Cond(sds) != SD_NO_ERR)			/* CMD1: ������ʺ���MMC�� */
	{
		if (SdCmd_SD_App_OP_Cond(sds) == SD_NO_ERR)     /* ACMD41: ��ȡOCR �Ĵ��� */
		{                                               
			sds->card_type = CARDTYPE_SD;               /* ����Ϊ SD�� */
		}
		else
		{		
			sds->card_type = CARDTYPE_UNKNOWN;          /* ����ʶ��Ŀ� */
			return SD_ERR_UNKNOWN_CARD;						
		}
	}
	else
		sds->card_type = CARDTYPE_MMC;	                /* ����Ϊ MMC�� */
		
	return SD_NO_ERR;		
}


/*******************************************************************************************************************
** ��������: SDMMC_ConfigCard()				
**
** ��������: �Կ�����ز�����������, �翨���߿��, �����ݿ��С
**
** �䡡  ��: sd_struct *sds: SD/MMC����Ϣ�ṹ��
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SDMMC_ConfigCard(sd_struct *sds)
{
	INT8U ret;
	
	SdHal_OpenDrainCmd(sds, 0);                         /* ��ֹCMD�ߵĿ�©ģʽ */
		
    // ������봫��״̬, ���ÿ�����ز���    	
 	ret = SdCmd_Select_Card(sds);			            /* CMD7,���봫��״̬   */
    if (ret != SD_NO_ERR)
       return ret;
 
   	if ((sds->card_type == CARDTYPE_SD)||(sds->card_type == CARDTYPE_SDHC))		//20120104	
    {                                                   /* ֻ��SD���ȿ�Ϊ����������, �ֿ�Ϊ���������� */
#if SD_WIDTHBUS_EN   
		SdHal_EnableMCIBusWidth(sds, 1);
		ret = SdCmd_Set_Bus_Width(sds, 1);    			/* ACMD6���ÿ�Ϊ���������߷�ʽ */
#else
		SdHal_EnableMCIBusWidth(sds, 0);
        ret = SdCmd_Set_Bus_Width(sds, 0);    			/* ACMD6���ÿ�Ϊ���������߷�ʽ */
#endif  
    	if (ret != SD_NO_ERR)
    		return ret; 
    }	
    else	   
		SdHal_EnableMCIBusWidth(sds, 0);                /* MMC��ֻ��Ϊ���������߷�ʽ */
 	
 	ret = SdCmd_Set_BlockLen(sds, SD_BLOCKSIZE);        /* CMD16, �ÿ����ݴ�СΪ: 512�ֽ� */
 	if (ret != SD_NO_ERR)
 	    return ret;
  	
       if (sds->block_len != SD_BLOCKSIZE) {
  		sds->block_num *= (sds->block_len / SD_BLOCKSIZE);
  		sds->block_len = SD_BLOCKSIZE;
  	}

 	SdCmd_Deselect_Card(sds, 0);		            /* CMD7, �˳�����״̬ */  
 	return ret;	
}

#endif


/*******************************************************************************************************************
** ��������: SDMMC_GetCardInfo()				
**
** ��������: ���SD/MMC������Ϣ	
**
** �䡡  ��: sd_struct *sds: �����Խṹ��
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SDMMC_GetCardInfo(sd_struct *sds)
{
    INT32U tmp,i,j;
    INT8U csdbuf[16],ret;

#if SDBUS_MODE_EN
    ret = SdCmd_Send_CSD(sds, 16, csdbuf);                              /* ��CSD�Ĵ���    read CSD regi */
                                                                        /* ster                         */
#else
    ret = SpiCmd_Send_CSD(sds, 16, csdbuf);
#endif
    if (ret != SD_NO_ERR)
    return ret;

    /* ��������󳤶�  */                                             /* calculate the size of a sect */
                                                                        /* or                           */
    sds->block_len = 1 << (csdbuf[READ_BL_LEN_POS] & READ_BL_LEN_MSK);  /* (2 ^ READ_BL_LEN)            */

    if (sds->card_type == CARDTYPE_SDHC)                                /* ��������     High Capacity c */
                                                                        /* ard                          */
    {
        /* ���㿨�п�ĸ��� */                                          /* calculate the sector numbers */
                                                                        /* of the SD Card               */
//        sds->block_num = (((csdbuf[C_SIZE_HC_POS1] << 8) +
//        csdbuf[C_SIZE_HC_POS1]) + 1) << 10;                           /* (C_SIZE + 1)* 1K  */	//4.16-�޸�ǰ

//		  sds->block_num = (((csdbuf[C_SIZE_HC_POS1] << 8) +
//                           csdbuf[C_SIZE_HC_POS1]) + 1) << 9;			//(C_SIZE + 1)* 512   //4.16-fan-��һ���޸�


		  tmp = csdbuf[7] & 0x0f;
		  tmp = tmp << 16;
	  	  i = csdbuf[8]<< 8;
		  j = csdbuf[9];
		  tmp = tmp | i | j;
		  //tmp = ((csdbuf[7] & 0x0f)<<12 + csdbuf[8]<<4 + csdbuf[9]);
		
		  sds->block_num = 1024* (tmp + 1);								  //4.16-fan-�ڶ����޸�
	}



    else
    {

        /* ���㿨�п�ĸ��� */                                          /* calculate the sector numbers */
                                                                        /* of the SD Card               */
        sds->block_num = ((csdbuf[C_SIZE_POS1] & C_SIZE_MSK1) << 10) +
        (csdbuf[C_SIZE_POS2] << 2) +
        ((csdbuf[C_SIZE_POS3] & C_SIZE_MSK3) >> 6) + 1;                 /* (C_SIZE + 1)                 */

        tmp = ((csdbuf[C_SIZE_MULT_POS1] & C_SIZE_MULT_MSK1) << 1) +
        ((csdbuf[C_SIZE_MULT_POS2] & C_SIZE_MULT_MSK2) >> 7) + 2;       /* (C_SIZE_MULT + 2)            */

        /* ��ÿ��п������ */                                          /* get the block numbers in car */
                                                                        /* d                            */
        sds->block_num = sds->block_num * (1 << tmp);                   /* (C_SIZE + 1) * 2 ^ (C_SIZE_M */
                                                                        /* ULT + 2)                     */
    }
                                                                        /* ��������ĵ�λ(��λ: ��)     */
    if (sds->card_type == CARDTYPE_MMC)
    {
        tmp  = ((csdbuf[ERASE_GRP_SIZE_POS] & ERASE_GRP_SIZE_MSK) >> 2) + 1;
                                                                        /* (ERASE_GRP_SIZE + 1)         */

                                                                        /* (ERASE_GRP_SIZE + 1) * (ERAS */
                                                                        /* E_GRP_MULTI_MSK1 + 1)        */
        tmp *= ((csdbuf[ERASE_GRP_MULTI_POS1] & ERASE_GRP_MULTI_MSK1) << 3) +
        ((csdbuf[ERASE_GRP_MULTI_POS2] & ERASE_GRP_MULTI_MSK2) >> 5) + 1;
    }
    else                                                                /* calculate the size of sector */
    tmp = ((csdbuf[SECTOR_SIZE_POS1] & SECTOR_SIZE_MSK1) << 1) +
    ((csdbuf[SECTOR_SIZE_POS2] & SECTOR_SIZE_MSK2) >> 7) + 1;           /* SD: SECTOR_SIZE              */

    sds->erase_unit = tmp;                                              /* ������λ(��)                 */

    sds->card_transpeed = tr_spexp[csdbuf[TRAN_SPEED_POS] & TRAN_SPEED_MSK1] *
    tr_spval[(csdbuf[TRAN_SPEED_POS] & TRAN_SPEED_MSK2) >> 3] / 10;

    return SD_NO_ERR;                                                   /* ����ִ�гɹ� return perform  */
                                                                        /* sucessfully                  */

}




/*******************************************************************************************************************
** ��������: SDMMC_CalTimeout()				
**
** ��������: �����/д/����ʱʱ��
**
** �䡡  ��: INT8U *csdbuf : CSD�Ĵ�������, ����Ϊ16�ֽ�
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: ���еĳ�ʱʱ�䶼����һ��ClockΪ��λ
********************************************************************************************************************/

INT8U SDMMC_CalTimeout(sd_struct *sds)
{
	INT32U tmp;
	INT8U time_u,time_v,fator;
	INT8U csdbuf[16],ret;
	#if SDBUS_MODE_EN
		ret = SdCmd_Send_CSD(sds, 16, csdbuf);								/* ��CSD�Ĵ��� */
	#else
	    ret = SpiCmd_Send_CSD(sds, 16, csdbuf);
	#endif
		if (ret != SD_NO_ERR)	
			return ret;	
	if (sds->card_type == CARDTYPE_SD) 
	{
	//	INT32U t_100ms = sds->host_clkval / 10;                             /* 100ms ����, �����ٸ�clock */
	//	INT32U t_250ms = sds->host_clkval / 4;                              /* 250ms ����, �����ٸ�clock */
		INT32U t_100ms = sds->host_clkval / 2;                             /* 100ms ����, �����ٸ�clock */
		INT32U t_250ms = sds->host_clkval / 2;                             /* 250ms ����, �����ٸ�clock */
					
		sds->timeout_read = t_100ms;								        /* Ĭ�϶���ʱΪ100ms */
		sds->timeout_write = t_250ms;							            /* Ĭ��д��ʱΪ250ms */
		sds->timeout_erase = t_250ms;                                       /* Ĭ�ϲ���ʱΪ250ms */
			
		time_u = (csdbuf[TAAC_POS] & TAAC_MSK);								/* ����ʱʱ�䵥λ read timeout unit */
		time_v = (csdbuf[TAAC_POS] & NSAC_MSK) >> 3;						/* ����ʱʱ��ֵ   read timeout value */
		fator = (csdbuf[R2WFACTOR_POS] & R2WFACTOR_MSK) >> 2;				/* ����ʱʱ������ read timeout factor */
		
		if(time_v == 0)	return SD_ERR_CARD_PARAM;							/* �������д��� card parameter is error */
		
		tmp = sds->host_clkval * time_value[time_v] / 10 / time_unit[time_u];	/* TAAC * f (��λ unit: clock) */
		tmp = tmp + csdbuf[NSAC_POS] * 100;									    /* TAAC * f + NSAC * 100 (��λ unit: clock) */
		
		/* ����õ��ĳ�ʱֵ the timeout value of being calculated */
		sds->timeout_read = tmp*4;
		sds->timeout_write = tmp * r2w_fator[fator];						/* (TAAC * f + NSAC * 100) * R2WFACTOR (��λ unit:clock)*/
		
		if (sds->card_type == CARDTYPE_SD)
		{
			sds->timeout_read  = (sds->timeout_read * 100);     			/* ʵ��ֵΪ����ֵ��100�� */
			sds->timeout_write = (sds->timeout_write * 100);
			if (sds->timeout_read > t_100ms)						        /* ȡ����ֵ��Ĭ��ֵ�е���Сֵ */
				sds->timeout_read = t_100ms;
			
			if (sds->timeout_write > t_250ms)
				sds->timeout_write = t_250ms;
		}
		else
		{
			sds->timeout_read  = (sds->timeout_read * 10);      			/* ʵ��ֵΪ����ֵ��10�� */
			sds->timeout_write = (sds->timeout_write * 10);
		}
	}
	if (sds->card_type == CARDTYPE_SDHC)	 			//fan-��ӣ�SDHC���Ķ�д��ʱʱ����㷽����ͬ
	{
//		INT32U t_100ms = (sds->host_clkval /10)*400;    /* ���ٸ�clock */
//		INT32U t_250ms = (sds->host_clkval / 4)*400;
		INT32U t_100ms = 0x1dc230;                      /* ���ٸ�clock */
		INT32U t_250ms = 0x3b8360;
		sds->timeout_read = t_100ms;								        
		sds->timeout_write = t_250ms;							           
		sds->timeout_erase = t_250ms;    
	} 	
	
#if SD_UCOSII_EN
    if (sds->host_clkval == 0)  return SD_ERR_SD_STRUCT;
	sds->timeout_read_os  = OS_TICKS_PER_SEC/10 ; 	//
	sds->timeout_write_os = OS_TICKS_PER_SEC/4 ;	//
	sds->timeout_erase_os = sds->timeout_write_os;						/* (��λ unit: os tick) */

//	sds->timeout_read_os  = sds->timeout_read * OS_TICKS_PER_SEC / sds->host_clkval; 
//	sds->timeout_write_os = sds->timeout_write * OS_TICKS_PER_SEC / sds->host_clkval;
//	sds->timeout_erase_os = sds->timeout_write_os;						/* (��λ unit: os tick) */
#endif	

    /* SPIģʽ����UCOS������ */
#if	(!SDBUS_MODE_EN)
#if (!SD_UCOSII_EN)
    sds->timeout_read = sds->timeout_read / 8;                         /* SPI����ģʽ������Ϊ8clockΪ��λ */
    sds->timeout_write = sds->timeout_write / 8;
#endif
#endif

	sds->timeout_erase = sds->timeout_write;
	
	return SD_NO_ERR;
}

/*******************************************************************************************************************
** ��������: SD_EraseBlock()				
**
** ��������: ����SD/MMC���еĿ�
**
** �䡡  ��: sd_struct *sds   : �����Խṹ��
**	         INT32U startaddr : ��������ʼ��ַ, ��ַΪ�Կ�Ϊ��λ�Ŀ��ַ
**           INT32U blocknum  : �����Ŀ���            
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: startaddr �� blocknum ����Ϊ sds.erase_unit ��������, ��Ϊ�еĿ�ֻ���� sds.erase_unit Ϊ��λ���в���
********************************************************************************************************************/
#if SD_EraseBlock_EN
INT8U SD_EraseBlock(sd_struct *sds, INT32U startaddr, INT32U blocknum)
{
	INT32 tmp;
	INT8U ret;
	
	if (sds == NULL) return SD_ERR_USER_PARAM;                              /* ������ڲ������� */
	
	SD_RequestOSSem(sds);                                                   /* ��OS������ʿ����ź��� */
//    if (!SdHal_CheckCard(sds))							
//    {
//    	ret = SD_ERR_NO_CARD;                                               /* ��û��ȫ���뿨���� */	
//    	goto SD_ERASE_END;
//	}
	
	if ((startaddr + blocknum) > sds->block_num)	
	{
		ret = SD_ERR_OVER_CARDRANGE;	
		goto SD_ERASE_END;                                                  /* ����������������Χ */
	}
		
	if (SdHal_CheckCardWP(sds))
	{
	    ret = SD_ERR_WRITE_PROTECT;                                         /* ����д���� */
		goto SD_ERASE_END;						
	}	

#if SDBUS_MODE_EN
	ret = SdCmd_Select_Card(sds);					                        /* CMD7: ���봫��״̬  */
    if (ret != SD_NO_ERR)
    	goto SD_ERASE_END;
#endif

	tmp = blocknum - sds->erase_unit;	
	while(tmp >= 0)												            /* ÿ�β���sds->erase_unit �� */
	{	
		ret = SdCmd_Erase_Wr_Blk_Start(sds, startaddr);			            /* ѡ����ʼ���ַ */
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;
		
		ret = SdCmd_Erase_Wr_Blk_End(sds, startaddr + sds->erase_unit - 1);	/* ѡ����ֹ���ַ */
		if (ret != SD_NO_ERR)
	        goto SD_ERASE_END;
			
		ret = SdCmd_Erase(sds);							                    /* ������ѡ��Ŀ� */
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;

		startaddr += sds->erase_unit;								        /* ��ʼ��ַ���� */
		blocknum  -= sds->erase_unit;
		tmp = blocknum - sds->erase_unit;
	};		
	
	if (blocknum > 0)											            /* ��������sds->erase_unit�Ŀ� */
	{
		ret = SdCmd_Erase_Wr_Blk_Start(sds, startaddr);
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;
		
		ret = SdCmd_Erase_Wr_Blk_End(sds, startaddr + blocknum - 1);
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;
			
		ret = SdCmd_Erase(sds);
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;
	}

SD_ERASE_END:
#if SDBUS_MODE_EN
    SdCmd_Deselect_Card(sds, 0);                                      /* CMD7: �˳�����״̬ */  
#endif
	SD_ReleaseOSSem(sds);
	return ret;											                    /* ���ز������ */
}
#endif


/*******************************************************************************************************************
** ��������: SD_RequestOSSem()				
**
** ��������: �����ϵͳ������ʿ����ź���
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
void SD_RequestOSSem(sd_struct *sds)
{
#if SD_UCOSII_EN
	INT8U ret;
	
	OSSemPend(sds->pSemSD, 0, &ret);				/* �ȴ����ʿ��ź�������  */
#endif
}


/*******************************************************************************************************************
** ��������: SD_ReleaseOSSem()				
**
** ��������: �ͷŷ��ʿ����ź���
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
void SD_ReleaseOSSem(sd_struct *sds)
{
#if SD_UCOSII_EN
	OSSemPost(sds->pSemSD);							/* �����ʿ��ź�����������ϵͳ */
#endif
}


/*******************************************************************************************************************
** ��������: SD_GetZLGSDVer()				
**
** ��������: �õ�ZLG/SD������İ汾��	
**
** �䡡  ��: �� 
**	         
** �� �� ��: ��	
**
** �� �� ֵ: �汾��  Version
**
** �汾��¼: V1.0   ֧�ַ���SD ��,ֻ����ǰ��̨ϵͳ����
			 
			 V2.0   ֧�ַ���SD��MMC��,��������ǰ��ϵͳ��UCOS-II
			 
			 V3.0   ֧�ֲ������� SD/MMC ����SDģʽ��SPIģʽ, ���Ӳ���ֲ��������
********************************************************************************************************************/
INT16U SD_GetZLGSDVer(void)
{
	return 0x0300;						/* �汾��Ϊ 3.00 */
}




/*
*****************************************************************************************
   ��������� SPI ����ģʽ, ������LPC21xx, LPC22xx, LPC2364/6 �ȴ���SPI��������΢������
*****************************************************************************************
*/

#if (!SDBUS_MODE_EN)

INT8U SdSpi_ActiveInit(sd_struct *sds);
			
/*******************************************************************************************************************
** ��������: SD_Initialize()				
**
** ��������: SD/MMC ����ʼ��: ��λ��,ʶ��,��ȡ�������Ϣ,ʹ���������ݴ���ģʽ 
**
** �䡡  ��: sd_struct *sds: SD/MMC����Ϣ�ṹ�� 
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/			
INT8U SD_Initialize(sd_struct *sds)
{
	INT32U ocr;
	INT8U ret;
	
#if SD_UCOSII_EN
	if (sds->pSemSD == NULL)
	{
    	sds->pSemSD = OSSemCreate(1);				/* ��������SD/MMC���ź��� create SD/MMC semaphore */ 
		if (sds->pSemSD == NULL)					
		 	return SD_ERR_CREATE_SEMSD;
	}				    
#endif	
    
    SD_RequestOSSem(sds);
   	SdSpiHal_Initialize(sds);						/* ��ʼ����дSD����Ӳ������ Initialize the hardware that access SD Card */
    
//    if (!SdHal_CheckCard(sds))						/* ��鿨�Ƿ���� check weather card is inserted */
//    {
//    	ret = SD_ERR_NO_CARD;   
//    	goto SD_ERR;
//    }
         
    SdSpiHal_CSAssert(sds);							/* 1. ��CSΪ�� assert CS */  
	SdSpi_Delay(sds, 25);						    /* 2. ������ʱ 74 clock delay more than 74 clock */
    SdSpiHal_CSDeassert(sds);						/* 3. ��CSΪ�� dessert CS */
    SdSpi_Delay(sds, 2);							/* 4. ��ʱ2(8 clock) delay 2(8 clock) */
    
    ret = SpiCmd_Go_Idle_State(sds);				/* 5. ����CMDO���λSD�� send CMD0 command to reset sd card */
    if (ret != SD_NO_ERR)
        goto SD_ERR;									
 	
 	ret = SdSpi_ActiveInit(sds);					/* 6. ��������ʼ������. active card initialize process */
 	if (ret != SD_NO_ERR)
 		goto SD_ERR;
        
   	ret = SpiCmd_ReadOCR(sds, &ocr);  				/* 7. ��OCR�Ĵ���,��ѯ��֧�ֵĵ�ѹֵ read OCR register,get the supported voltage */
    if (ret != SD_NO_ERR)
        goto SD_ERR;
    
    if ((ocr & MSK_OCR_33) != MSK_OCR_33)
    { 
        ret = SD_ERR_VOL_NOTSUSP;					/* ��֧��3.3V,���ش�����  not support 3.3V,return error code */
    	goto SD_ERR;
    }
    
    SdSpiHal_SetMCIClock(sds, SD_RATE_NORMAL);  	/* 8. ���÷��ʿ���clockΪ��׼clock */
        
#if SD_CRC_EN        
    ret = SpiCmd_CRC_ON_OFF(sds, 1);				/* ʹ��CRCУ�� enable CRC check */
#else
    ret = SpiCmd_CRC_ON_OFF(sds, 0);				/* ��ֹCRCУ�� disable CRC check */
#endif   
	if (ret != SD_NO_ERR)  
	  	goto SD_ERR;

    ret = SpiCmd_Set_BlockLen(sds, SD_BLOCKSIZE);	/* 9. ���ÿ�ĳ���: 512Bytes Set the block length: 512Bytes */
    if (ret != SD_NO_ERR)  
        goto SD_ERR;
        
    ret = SDMMC_GetCardInfo(sds);					/* 10. ��CSD�Ĵ���,��ȡSD����Ϣ read CSD register, get the information of SD card */    
	if (ret != SD_NO_ERR)
		goto SD_ERR;

	ret = SDMMC_CalTimeout(sds);					/* ���㳬ʱʱ��ֵ */
	if (ret != SD_NO_ERR)
	    goto SD_ERR;

	SD_ReleaseOSSem(sds);	
	return SD_NO_ERR;								/* ��ʼ���ɹ� initialize sucessfully */

SD_ERR:	
	SD_ReleaseOSSem(sds);
	return ret;
}


/*******************************************************************************************************************
** ��������: SD_ReadBlock()				
**
** ��������: SPIģʽ��, ��SD/MMC���ж���һ�����ݿ�
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT32U blockaddr: �Կ�Ϊ��λ�Ŀ��ַ, ����, ����ʼ��0 ~ 511�ֽ�Ϊ���ַ0, 512 ~ 1023�ֽڵĿ��ַΪ1
**	         
** �� �� ��: INT8U *recbuf   : ���ջ�����,���ȹ̶�Ϊ 512 �ֽ�	
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SD_ReadBlock(sd_struct *sds, INT32U blockaddr, INT8U *recbuf)
{
	INT8U ret; 
	
	SD_RequestOSSem(sds);							    /* ��OS������ʿ��ź��� request semaphore acessed SD/MMC to OS */
//    if (!SdHal_CheckCard(sds))							
//    {
//    	SD_ReleaseOSSem(sds);	    
//    	return SD_ERR_NO_CARD;   					    /* ��û��ȫ���뿨�� card is not inserted entirely */
//	}
		
	if (blockaddr > sds->block_num)	
	{
		SD_ReleaseOSSem(sds);	
		return SD_ERR_OVER_CARDRANGE;				    /* ����������������Χ operate over the card range */
	}
	
 	ret = SpiCmd_Read_Single_Block(sds, blockaddr);		/* ���������� read single blocks command */						
 	if (ret != SD_NO_ERR)
 	{
 		SD_ReleaseOSSem(sds);									
 		return ret;
 	}
 	
  	ret = SdSpi_ReadBlockData(sds, SD_BLOCKSIZE, recbuf); /* �������� read data from sd card */	
	SD_ReleaseOSSem(sds);								  /* �黹���ʿ��ź��� return semaphore acessed SD/MMC to OS */
	
	return ret;
}


/*******************************************************************************************************************
** ��������: SD_ReadMultiBlock()				
**
** ��������: SPIģʽ��, ��SD/MMC���ж���������ݿ�
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT32U blockaddr: �Կ�Ϊ��λ�Ŀ��ַ
**	         INT32U blocknum : Ҫ��ȡ�Ŀ�ĸ���
**
** �� �� ��: INT8U *recbuf   : ���ջ�����, ����512 * blocknum �ֽ�	
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
#if SD_ReadMultiBlock_EN
INT8U SD_ReadMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *recbuf)
{
    INT32U i;
    INT8U ret;
       
	SD_RequestOSSem(sds);							    /* ��OS������ʿ��ź��� request semaphore acessed SD/MMC to OS */
//    if (!SdHal_CheckCard(sds))							
//    {
//    	SD_ReleaseOSSem(sds);	
//    	return SD_ERR_NO_CARD;   					    /* ��û��ȫ���뿨�� card is not inserted entirely */
//	}
		
	if ((blockaddr + blocknum) > sds->block_num)	
	{
		SD_ReleaseOSSem(sds);	
		return SD_ERR_OVER_CARDRANGE;				    /* ����������������Χ operate over the card range */
	}       
    
	ret = SpiCmd_Read_Multiple_Block(sds, blockaddr);   /* ��������� read multiple blocks command */
	if (ret != SD_NO_ERR)
	{	
		SD_ReleaseOSSem(sds);						    /* �黹���ʿ��ź��� return semaphore acessed SD/MMC to OS */						
		return ret;
    }
    
    for (i = 0; i < blocknum; i++)
    {												    /* �������� read data from SD/MMC card */
    	ret = SdSpi_ReadBlockData(sds, SD_BLOCKSIZE, recbuf);
    	if (ret == SD_NO_ERR)
       		recbuf = recbuf + SD_BLOCKSIZE;
    	else
    	{
    		SD_ReleaseOSSem(sds);
    		return ret;
    	}
    }
 	 
 	ret = SpiCmd_Stop_Transmission(sds);			    /* �������ݴ��� stop transmission operation */ 
 	
 	SD_ReleaseOSSem(sds);
	return ret; 
}

#endif


/*******************************************************************************************************************
** ��������: SD_WriteBlock()				
**
** ��������: SPIģʽ��, ��SD/MMC����д��һ����	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT32U blockaddr: �Կ�Ϊ��λ�Ŀ��ַ, ����, ����ʼ��0 ~ 511�ֽ�Ϊ���ַ0, 512 ~ 1023�ֽڵĿ��ַΪ1
**           INT8U *sendbuf  : ���ͻ�����,���ȹ̶�Ϊ 512 �ֽ�	
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SD_WriteBlock(sd_struct *sds, INT32U blockaddr, INT8U *sendbuf)
{
	INT8U ret,tmp[2];
	
	SD_RequestOSSem(sds);										/* ��OS������ʿ��ź��� request semaphore acessed SD/MMC to OS */
//    if (!SdHal_CheckCard(sds))						
//    {
//    	SD_ReleaseOSSem(sds);	
//    	return SD_ERR_NO_CARD;   								/* ��û��ȫ���뿨�� card is not inserted entirely */
//	}	
	
	if (blockaddr > sds->block_num)	
	{
		SD_ReleaseOSSem(sds);	
		return SD_ERR_OVER_CARDRANGE;							/* ����������������Χ operate over the card range */
	}
	
	if (SdHal_CheckCardWP(sds))
	{
		SD_ReleaseOSSem(sds);
		return SD_ERR_WRITE_PROTECT;							/* ����д���� */
	}
	
	ret = SpiCmd_Write_Single_Block(sds, blockaddr);			/* д�������� write single block */
	if (ret != SD_NO_ERR)
	{	
		SD_ReleaseOSSem(sds);
		return ret;
	}
	
	ret = SdSpi_WriteBlockData(sds, 0, SD_BLOCKSIZE, sendbuf);	/* д������ write data */
 	if (ret == SD_NO_ERR)										/* ��Card Status�Ĵ���, ���д���Ƿ�ɹ� */
 	{															/* read Card Status register to check write wheather sucessfully */
 		ret = SpiCmd_Send_Status(sds, 2, tmp);
 		if (ret != SD_NO_ERR)
 		{
 			SD_ReleaseOSSem(sds);
 			return ret;											/* ���Ĵ���ʧ�� read register fail */
		}

 		if((tmp[0] != 0) || (tmp[1] != 0))
 		{
 			SD_ReleaseOSSem(sds);
			ret = SD_ERR_WRITE_BLK; 			     			/* ��Ӧָʾдʧ�� response indicate write fail */
 		}
 	}
  
    SD_ReleaseOSSem(sds);
 	return ret;													/* ����д���� return the result of writing */									
}


/*******************************************************************************************************************
** ��������: SD_WriteMultiBlock()				
**
** ��������: SPIģʽ��, ��SD/MMC����д�������ݿ�	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT32U blockaddr: �Կ�Ϊ��λ�Ŀ��ַ
**           INT32U blocknum : Ҫд��Ŀ�ĸ���
**           INT8U *sendbuf  : ���ͻ�����, ���� 512 * blocknum �ֽ�	
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
INT8U SD_WriteMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *sendbuf)
{
	INT32U i;
	INT8U ret,tmp[2];
	
	SD_RequestOSSem(sds);										/* ��OS������ʿ��ź��� request semaphore acessed SD/MMC to OS */
//    if (!SdHal_CheckCard(sds))							
//    {
//    	SD_ReleaseOSSem(sds);	
//    	return SD_ERR_NO_CARD;   								/* ��û��ȫ���뿨�� card is not inserted entirely */
//	}
		
	if ((blockaddr + blocknum) > sds->block_num)	
	{
		SD_ReleaseOSSem(sds);	
		return SD_ERR_OVER_CARDRANGE;							/* ����������������Χ operate over the card range */
	}
		
	if (SdHal_CheckCardWP(sds))
	{
		SD_ReleaseOSSem(sds);
		return SD_ERR_WRITE_PROTECT;							/* ����д���� */
	}
	
	ret = SpiCmd_Write_Multiple_Block(sds, blockaddr);			/* д������� write multiple blocks command */
	if (ret != SD_NO_ERR)
	{
		SD_ReleaseOSSem(sds);
		return ret;
	}
	
    for (i = 0; i < blocknum; i++)
    {															
 		ret = SdSpi_WriteBlockData(sds, 1, SD_BLOCKSIZE, sendbuf);	/* д������ write data */
 		if (ret == SD_NO_ERR)	
 			sendbuf = sendbuf + SD_BLOCKSIZE;
 		else
 		{														/* дʧ�� write fail */
			SpiCmd_Stop_Transmission(sds);						/* ֹͣ���ݴ��� stop data transmission */	
			SdSpi_WaitBusy(sds, SD_WAIT_WRITE);					/* �ȴ� waiting */
			SD_ReleaseOSSem(sds);
			return ret;
		}
	}
   
    SdSpi_StopMultiToken(sds);									/* ��������ֹͣ���� send data stop token */
      
    ret = SdSpi_WaitBusy(sds, SD_WAIT_WRITE);					/* �ȴ�д������ wait for finishing writing */
    if (ret != SD_NO_ERR)
    {
    	SD_ReleaseOSSem(sds);
    	return SD_ERR_TIMEOUT_WRITE;
    }
    
    if (sds->card_type == CARDTYPE_SD)
    {
   		ret = SpiCmd_Send_Num_WR_Blcoks(sds, &i);				/* ����ȷд��Ŀ��� read the blocks that be written correctly */
   		if (ret != SD_NO_ERR)
   		{
   			SD_ReleaseOSSem(sds);
   		  	return ret;
   		}
   		if(i != blocknum)
			ret =  SD_ERR_WRITE_BLKNUMS;						/* ��ȷд��������� the blocks that be written correctly is error */
   	}
   	else
   	{
   	 	ret = SpiCmd_Send_Status(sds, 2, tmp);
 		if (ret != SD_NO_ERR)
 		{
 			SD_ReleaseOSSem(sds);
 			return ret;											/* ���Ĵ���ʧ�� read register fail */
		}
 		if((tmp[0] != 0) || (tmp[1] != 0))
			ret = SD_ERR_WRITE_BLK; 			     			/* ��Ӧָʾдʧ�� response indicate write fail */
   	}
   	 	
   	SD_ReleaseOSSem(sds);   	     
	return ret;													/* ����д��ɹ� return write sucessfully */			
}
#endif


	/*****************************************************************
	
	    		����Ϊ�ӳ���
	 
	*****************************************************************/

/*******************************************************************************************************************
** ��������: SdSpi_ActiveInit()				
**
** ��������: SPIģʽ��, ���,����ÿ���	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ����˵��: SpiCmd_Send_OP_Cond()�����ظ�����CMD0������ֱ����ӦR1��Bit0(Idle)λΪ0����ʾ���ڲ���ʼ��������ɡ�
		     ����Ӧ��IdleλΪ0ʱ��SD������ȫ����SPIģʽ�ˡ���Ȼ�ظ���������CMD1���д������Ƶģ�
		     ������Ϊ�궨��SD_IDLE_WAIT_MAX.
********************************************************************************************************************/
INT8U SdSpi_ActiveInit(sd_struct *sds)
{
	INT8U param[4] = {0,0,0,0},resp[5],ret;
	
    ret = SpiCmd_Send_OP_Cond(sds);                         /* ��������ʼ������, ʹ���˳�Idle״̬ */
    if (ret != SD_NO_ERR)
        return ret;    
    	
    ret = SdSpi_SendCmd(sds, CMD55, param, CMD55_R, resp);
    if (ret != SD_NO_ERR)
       	return ret;
       	
    ret = SdSpi_SendCmd(sds, ACMD41, param, ACMD41_R, resp);/* �����ڲ���ʼ������ active card to initialize process internal */
    if (ret != SD_NO_ERR)	
    	return SD_ERR_UNKNOWN_CARD;
    
    if ((resp[0] & 0xFE) == 0)
      	sds->card_type = CARDTYPE_SD;						/* ��SD�� the card is SD card */	
    else	
    	sds->card_type = CARDTYPE_MMC;						/* ��MMC�� the card is MMC card */

	return SD_NO_ERR;
}

#endif  //end of #if (!SDBUS_MODE_EN)

 //ȡ��SD���������ж���SD������Щ����
uint8 full_write_sd(uint32 sd_add, uint8 *buf)	   
{
	uint8 err=0;
	uint8 stat;
	
	OSSemPend(g_SdOpMutex,5000,&err);
	if(err==0)				
	{				
		//sic2IrqDisable(2);
		stat = WriteSDCardBlock(sd_add,buf);		 
		//sic2IrqEnable(2);
		OSSemPost(g_SdOpMutex);

		if (stat == 0)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 1;
	}	
}

//�洢SD���������ж���SD������Щ����
uint8 full_read_sd(uint32 sd_add, uint8 *buf)   
{	
	uint8 err=0;
	uint8 stat;

	OSSemPend(g_SdOpMutex, 5000, &err);
	if (err == 0)
	{
		//sic2IrqDisable(2);
		stat = ReadSDCardBlock(sd_add, buf);
		//sic2IrqEnable(2);
		OSSemPost(g_SdOpMutex);

		if (stat==0)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 1;
	}
}

 //дSD����������������ж���SD������Щ����
uint8 full_Mutil_Wr_sd(uint32 sd_add, uint8 *buf,uint8 Num)	   
{
	uint8 err=0;
	uint8 stat;
	OSSemPend(g_SdOpMutex,5000,&err);
	if(err==0)				
	{				
		//sic2IrqDisable(2);
		stat = SD_WriteMultiBlock(&sd_info,sd_add, Num, buf);		 
		//sic2IrqEnable(2);
		OSSemPost(g_SdOpMutex);

		if (stat == 0)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 1;
	}	
}
















