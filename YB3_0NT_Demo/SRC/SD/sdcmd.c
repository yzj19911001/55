/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdcmd.c
** Last modified Date:	2007-5-19
** Last Version:		V2.0
** Descriptions:		SD/MMC �����: SD/MMC��SPIģʽ��SDģʽ֧�ֵ����� 
**						
**
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

#include "sdconfig.h"
#include "sddriver.h"


			/*********************************************

			 ���ļ�ʵ�� SD/MMC �� SDģʽ�µĸ� CMD �� ACMD

			**********************************************/

#if SDBUS_MODE_EN

static void word_to_byte(INT32U value, INT8U *pdst);

#define SEND_IF_COND            0x000001AA
#define OCR_WITHOUT_BUSY		0x00FF8000	
#define OCR_WITH_BUSY			0x80000000
#define ACMD41_ARGUMENT         0X40ff8000


/*************
    CMD 0 
*************/
/*******************************************************************************************************************
** ��������: SdCmd_Go_Idle_State()				
**
** ��������: CMD0, ��λ���п�, ʹ������Idle״̬		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Go_Idle_State(sd_struct *sds)
{
	return (SdHal_SendCmd(sds, CMD0, 0, CMD0_R, 0, 0));			/* ���� CMD0 ��ȡ����Ӧ */
}


/*************
    CMD 1 
*************/
/*******************************************************************************************************************
** ��������: SdCmd_Send_OP_Cond()				
**
** ��������: CMD1(������MMC��), ʹ���˳�Idle״̬, ����Ready״̬(ͨ���ж�MMC����OCR�Ĵ���ֵ)		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Send_OP_Cond(sd_struct *sds)
{
	INT32U OCRVal;
	INT8U ret;
	INT32U count = 0;

 	do 
    {	
		ret = SdHal_SendCmd(sds, CMD1, OCR_WITHOUT_BUSY, CMD1_R, &OCRVal, 1);	/* ���� CMD1 ��ȡ����Ӧ */
		if (ret != SD_NO_ERR)
			return ret;

        if (count >= SD_IDLE_WAIT_MAX)
            return SD_ERR_TIMEOUT_WAITIDLE;									    /* ��ʱ,���ش����� */
     	
      	count ++;

    }while ((OCRVal & OCR_WITH_BUSY) == 0x00000000);
	
    if ((OCRVal & MSK_OCR_33) != MSK_OCR_33)
        return SD_ERR_VOL_NOTSUSP;											    /* ����֧3.2V ~ 3.3V��ѹ */			
	
	return SD_NO_ERR;
}

/*************
    CMD 8
*************/
/*********************************************************************************************************
** ��������: SD_CheckSDVersion()
**
** ��������: ʶ�𿨰汾
**
** �䡡  ��: sd_struct *sds: SD/MMC����Ϣ�ṹ��
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: ִ�к�, ���ͱ�����sds->card_type��
*********************************************************************************************************/
INT8U SD_CheckSDVersion(sd_struct *sds)
{
    INT8U  ret;
    INT32U resp[4];
																		   //SEND_IF_COND = 0x000001AA
    ret = SdHal_SendCmd(sds, CMD8, SEND_IF_COND, CMD8_R, resp, 1);	   
    
    if (ret != SD_NO_ERR) 
	{
        return ret;                                                      
    } 
    if ((resp[0] & SEND_IF_COND) == SEND_IF_COND)									  
        return (0x01);                                     /* ����Ϊversion 1 */	  																					  //4.17-fan-�޸ģ�ȥ����
    else																			 
        return (0x00);                                     /* ����Ϊversion 2 */	 
//	return SD_NO_ERR;

   
}
/*************
    CMD 55 
*************/
/*******************************************************************************************************************
** ��������: SdCmd_Send_ACMD()				
**
** ��������: CMD55(������SD��), �ڷ������� ACMDxx ����֮ǰ, �����ȷ��ͱ�����		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: ��δʹ�� Set_Relative_Addr ����ȡ�ÿ���RCA��ַ֮ǰ, sds->RCAΪ0.
********************************************************************************************************************/
INT8U SdCmd_Send_ACMD(sd_struct *sds)
{
	INT32U argument;	
	INT32U devstatus = 0;
	INT8U ret;

	if ((sds->card_type == CARDTYPE_SD)||(sds->card_type == CARDTYPE_SDHC))			//20120104						/* ���ݿ���ȡ��ͬ�Ĳ��� */
		argument = sds->RCA;											
	else
		argument = 0x00000000;	
				  
	ret = SdHal_SendCmd(sds, CMD55, argument, CMD55_R, &devstatus, 1);	/* ���� CMD55 ��ȡ����Ӧ */	
	if (ret != SD_NO_ERR)
		return ret;
		
	if (devstatus & CARD_STATUS_ACMD_ENABLE)						
		return SD_NO_ERR;											    /* ֧�� CMD55 ���� */
		
	return SD_ERR_CMD_NOTSUSP;										    /* ��֧�ָ����� */
}


/*************
    ACMD 41 
*************/
/*******************************************************************************************************************
** ��������: SdCmd_SD_App_OP_Cond()				
**
** ��������: ACMD41(������SD��), ʹ���˳�Idle״̬, ������Ready״̬(ͨ���ж�SD����OCR�Ĵ���ֵ)		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**           
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: ��δʹ�� Set_Relative_Addr ����ȡ�ÿ���RCA��ַ֮ǰ, sds->RCAΪ0.
********************************************************************************************************************/
INT8U SdCmd_SD_App_OP_Cond(sd_struct *sds)
{
	INT8U ret;
	INT32U OCRVal, count = 0;

 	do 
    {	
       	ret = SdCmd_Send_ACMD(sds);                                               /* �ȷ��� CMD55 ���� */
		if (ret != SD_NO_ERR)
			return ret;
           
		ret = SdHal_SendCmd(sds, ACMD41, OCR_WITHOUT_BUSY, ACMD41_R, &OCRVal, 1); /* ���� ACMD41 ��ȡ����Ӧ */
		if (ret != SD_NO_ERR)
			return ret;

        if (count >= SD_IDLE_WAIT_MAX)
            return SD_ERR_TIMEOUT_WAITIDLE;									      /* ��ʱ,���ش����� */
     	
      	count ++;

    }while ((OCRVal & OCR_WITH_BUSY) == 0x00000000);
	
    if ((OCRVal & MSK_OCR_33) != MSK_OCR_33)
        return SD_ERR_VOL_NOTSUSP;											      /* ����֧3.2V ~ 3.3V��ѹ */			
	
	return SD_NO_ERR;	
}


/*************
    CMD 2
*************/
/*******************************************************************************************************************
** ��������: SdCmd_All_Send_CID()				
**
** ��������: ��ȡ���п���CID�Ĵ�������	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**           INT8U cidlen    : CID �Ĵ�������, �̶�Ϊ16�ֽ�
**	
** �� �� ��: INT8U *cidbuf   : CID �Ĵ�������, ����Ϊ16�ֽ�
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: �������Argument�򲻱ع���
********************************************************************************************************************/
INT8U SdCmd_All_Send_CID(sd_struct *sds, INT8U cidlen, INT8U *cidbuf)
{
	INT8U ret;
	INT32U resp[4];
	
	if ((cidlen < 16) || (cidbuf == 0))
	    return SD_ERR_USER_PARAM;
		
	ret = SdHal_SendCmd(sds, CMD2, 0, CMD2_R, resp, 4);
	if (ret != SD_NO_ERR)
		return ret;	
	
	word_to_byte(*resp, cidbuf);                            /* ����Ӧ������ȡ��CID�Ĵ������� */
	word_to_byte(*(resp + 1), cidbuf + 4);
	word_to_byte(*(resp + 2), cidbuf + 8);
	word_to_byte(*(resp + 3), cidbuf + 12);
			
	return SD_NO_ERR;
}


/*****************************
    CMD 3: ���ÿ��ĵ�ַ RCA 
******************************/
/*******************************************************************************************************************
** ��������: SdCmd_Set_Relative_Addr()				
**
** ��������: CMD3, ��ȡ����RCA��ַ, Ȼ�󱣴浽sds�ṹ����		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: �ú������� sds->RCA ��Ա��ֵ
********************************************************************************************************************/
INT8U SdCmd_Set_Relative_Addr(sd_struct *sds)
{	
	INT32U argument;
    INT32U resp;
    INT8U ret;

    /* �����һ��SD��, ʹ��SET_RELATIVE_ADDR���ڴӿ���ȡ��ַ����RCA��ʹ�ø�ֵ,
       �����һ��MMC��, ʹ��Ĭ�ϵ�RCA��ַ 0x00010000. */
    if (sds->card_type == CARDTYPE_SD || sds->card_type == CARDTYPE_SDHC)
    {
        argument = 0;
    }
    else
    {                                                                   /* �����δ֪�Ļ�MMC��, �̶�RCA */
                                                                        /* ��ַ                         */
        argument = 0x00010000;
    }

    ret = SdHal_SendCmd(sds, CMD3, argument, CMD3_R, &resp, 1);
    if (ret != 0)
    return ret;

    if (sds->card_type == CARDTYPE_SD || sds->card_type == CARDTYPE_SDHC)
    {
                                                                        /* ��Ӧ���� IDENT ״̬          */
        if ((resp & CARD_STATUS_CURRENT_STATE) == CARD_STATUS_IDENT)
        sds->RCA = resp & 0xFFFF0000;
        else
        return SD_ERR_CURRENT_STATE;
    }
    else
    sds->RCA = argument;

    return SD_NO_ERR;
}
/***********************************
     CMD 6: ѡ�񿨸���ģʽ
***********************************/
INT8U SdCmd_Speed_Mode(sd_struct *sds)
{
	INT32U resp;
	INT8U ret;
	ret = SdHal_SendCmd(sds, CMD6, 0x80fffff1, CMD6_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	return SD_NO_ERR;
}

/********************************************
    CMD 9, ����CMD3���ͺ�ſɷ���, ��ȡ CSD 
*********************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Send_CSD()				
**
** ��������: CMD9, ��ȡ����CSD�Ĵ���ֵ		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**           INT8U csdlen    : CSD �Ĵ�������
**	
** �� �� ��: INT8U *csdbuf   : CSD�Ĵ�������, bit127 ��csdbuf[0] 
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: ��������Ҫ sds->RCA ��Ϊ�������
********************************************************************************************************************/
INT8U SdCmd_Send_CSD(sd_struct *sds, INT8U csdlen, INT8U *csdbuf)
{	
	INT8U ret;
	INT32U resp[4];
	
	if ((csdlen < 16) || (csdbuf == 0))
	    return SD_ERR_USER_PARAM;	
	
	ret = SdHal_SendCmd(sds, CMD9, sds->RCA, CMD9_R, resp, 4);
	if (ret != SD_NO_ERR)
		return ret;
		
	word_to_byte(*resp, csdbuf);                            /* ����Ӧ������ȡ��CSD�Ĵ������� */
	word_to_byte(*(resp + 1), csdbuf + 4);
	word_to_byte(*(resp + 2), csdbuf + 8);
	word_to_byte(*(resp + 3), csdbuf + 12);
	
	return SD_NO_ERR;
}


/**********************************************************
    CMD 7: ѡ��, ʹ���� Stand-by �� Transfer ֮���л�
***********************************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Select_Card()				
**
** ��������: CMD7, ʹ���� Stand-by �� Transfer ֮���л�	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	        
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: ��������Ҫ sds->RCA ��Ϊ�������
********************************************************************************************************************/
INT8U SdCmd_Select_Card(sd_struct *sds)
{
	INT32U resp;
	INT8U ret;

	ret = SdHal_SendCmd(sds, CMD7, sds->RCA, CMD7_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	return SD_NO_ERR;
}


/**********************************************************
    CMD 7: ȡ��ѡ��, ʹ���� Stand-by �� Transfer ֮���л�
***********************************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Select_Card()				
**
** ��������: CMD7, ʹ���� Stand-by �� Transfer ֮���л�	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	         INT32U RCA      : ����RCA��ַ
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: ��������������ΪRCA, ȡֵΪ0
********************************************************************************************************************/
INT8U SdCmd_Deselect_Card(sd_struct *sds, INT32U RCA)
{
	INT32U resp;
	INT8U ret;

	ret = SdHal_SendCmd(sds, CMD7, RCA, CMD0_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	return SD_NO_ERR;
}


/***********************************
  CMD12: ֹͣ����Stop Transmission
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Send_Stop()				
**
** ��������: CMD12, ֹͣ����
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: �������argument�򲻱ع���, ���������øú���ֹͣ���Ͷ�����ݵ���ʱ, ����ȴ�CARD_Status�Ĵ�����
             READY_FOR_DATAλ��1
********************************************************************************************************************/
INT8U SdCmd_Stop_Transmission(sd_struct *sds)
{
	INT8U ret;
	INT32U resp;
	
	ret = SdHal_SendCmd(sds, CMD12, 0, CMD12_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
		
	return SD_NO_ERR;
}


/******************************
    CMD 13: ��ȡ����ǰ״̬
*******************************/
/*******************************************************************************************************************
** ��������: SdCmd_Send_Status()				
**
** ��������: CMD13, ��ȡ����ǰ״̬, �����������ݴ���ģʽ��(����ʶ��ģʽ�����ģʽ��), �Ϳ���ʹ�ñ�����		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	
** �� �� ��: INT32U *status  : ��״̬
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: ��������Ҫ sds->RCA ��Ϊ�������
********************************************************************************************************************/
INT8U SdCmd_Send_Status(sd_struct *sds, INT32U *status)
{
	INT32U resp;
	INT8U ret;
	
	ret = SdHal_SendCmd(sds, CMD13, sds->RCA, CMD13_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	*status = resp;
	return SD_NO_ERR;
}

		
/*********************************************
    CMD 15: ʹ������ǻ״̬
*********************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Go_Inactive_State()				
**
** ��������: CMD15, ʹ������ǻ״̬	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: ��������Ҫ sds->RCA ��Ϊ�������
********************************************************************************************************************/
INT8U SdCmd_Go_Inactive_State(sd_struct *sds)
{
	INT32U resp;
	INT8U ret;

	ret = SdHal_SendCmd(sds, CMD15, sds->RCA, CMD15_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	return SD_NO_ERR;	
}


/*********************************************
    CMD 16: ���ÿ鳤��, ���뷢��CMD7����
*********************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Set_BlockLen()				
**
** ��������: CMD16, ���ÿ��Ŀ鳤��	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**           INT32U blockLen : ����ĳ���
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Set_BlockLen(sd_struct *sds, INT32U blockLen)
{
	INT32U resp;
	INT8U ret;

	ret = SdHal_SendCmd(sds, CMD16, blockLen, CMD16_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
		
	/* bit 9 through 12 should be in transfer state now. bit 8 is ready. */
	if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)	 
	    return SD_ERR_CURRENT_STATE;
	 
	if (!(resp & CARD_STATUS_RDY_DATA))	     
	    return SD_ERR_READY_FOR_DATA;
	
	return SD_NO_ERR;	
}


/**************************************
	ACMD6: �������߿��, ֻ�ʺ���SD��
***************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Set_Bus_Width()				
**
** ��������: ACMD6, ���ÿ��������ݵ����߿��(����SD��, ����Ϊ1bit, Ҳ����Ϊ4bits)
**			 ����MMC��, ��������Ч			
**
** �䡡  ��: sd_struct *sds:  SD/MMC����Ϣ�ṹ��
**           INT8U bWideBus:  �Ƿ�ʹ�ܿ�����(4bit data bus).
**							  1:  ʹ��       0: ��ֹ
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Set_Bus_Width(sd_struct *sds, INT8U bWideBus)
{
	INT32U argument;
	INT32U resp;
	INT8U ret;
	
	if (bWideBus > 0)
		argument = 0x02;		// '02' = 4bits data bus
	else
		argument = 0x0;			// '00'	= 1bit data bus

    ret = SdCmd_Send_ACMD(sds);
	if (ret != SD_NO_ERR)
		return ret;

	ret = SdHal_SendCmd(sds, ACMD6, argument, ACMD6_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	/* bit 9 through 12 should be in transfer state now. bit 8 is ready. */
	if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)	 
	    return SD_ERR_CURRENT_STATE;
	 
	if (!(resp & CARD_STATUS_RDY_DATA))	     
	    return SD_ERR_READY_FOR_DATA;
	
	return SD_NO_ERR;		
}


/***********************************
  CMD17: ���Ͷ���������
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Read_Single_Block()				
**
** ��������: CMD17, ���Ͷ���������
**
** �䡡  ��: sd_struct *sds:  SD/MMC����Ϣ�ṹ��
**           INT32U blockadddr:  ���ַ. 0 ~ 511�ֽ�ȡֵΪ1. 512 ~ 1023�ֽ�ȡֵΪ2
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Read_Single_Block(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;
    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }
    ret = SdHal_SendCmd(sds, CMD17, blockadddr, CMD17_R, &resp, 1);
    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;
}


/***********************************
  CMD18: ���Ͷ��������
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Read_Multiple_Block()				
**
** ��������: CMD18, ���Ͷ��������
**
** �䡡  ��: sd_struct *sds:  SD/MMC����Ϣ�ṹ��
**           INT32U blockadddr:  ���ַ
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Read_Multiple_Block(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;
    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    ret = SdHal_SendCmd(sds, CMD18, blockadddr, CMD18_R, &resp, 1);
    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;
}


/***********************************
  CMD24: ����д��������
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Write_Single_Block()				
**
** ��������: CMD24, ����д��������
**
** �䡡  ��: sd_struct *sds:  SD/MMC����Ϣ�ṹ��
**           INT32U blockadddr:  ���ַ
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Write_Single_Block(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;
    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    ret = SdHal_SendCmd(sds, CMD24, blockadddr, CMD24_R, &resp, 1);
    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;	
}


/***********************************
  CMD25: ����д�������
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Write_Multiple_Block()				
**
** ��������: CMD25, ����д�������
**
** �䡡  ��: sd_struct *sds:  SD/MMC����Ϣ�ṹ��
**           INT32U blockadddr:  ���ַ
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Write_Multiple_Block(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;

    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    ret = SdHal_SendCmd(sds, CMD25, blockadddr, CMD25_R, &resp, 1);
    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;
}



/***********************************
  CMD32��CMD35: ���ò�����ʼ��ַ
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Erase_Wr_Blk_Start()				
**
** ��������: CMD32��CMD35, ���ò�����ʼ��ַ
**
** �䡡  ��: sd_struct *sds:  SD/MMC����Ϣ�ṹ��
**           INT32U blockadddr:  ���ַ
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Erase_Wr_Blk_Start(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;

    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    if (sds->card_type == CARDTYPE_SD || sds->card_type == CARDTYPE_SDHC)
    ret = SdHal_SendCmd(sds, CMD32, blockadddr, CMD32_R, &resp, 1);
    else
    ret = SdHal_SendCmd(sds, CMD35, blockadddr, CMD35_R, &resp, 1);

    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;		
}


/***********************************
  CMD33��CMD36: ���ò�����ֹ��ַ
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Erase_Wr_Blk_End()				
**
** ��������: CMD33��CMD36, ���ò�����ֹ��ַ
**
** �䡡  ��: sd_struct *sds:  SD/MMC����Ϣ�ṹ��
**           INT32U blockadddr:  ���ַ
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdCmd_Erase_Wr_Blk_End(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;

    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    if (sds->card_type == CARDTYPE_SD || sds->card_type == CARDTYPE_SDHC)
    ret = SdHal_SendCmd(sds, CMD33, blockadddr, CMD33_R, &resp, 1);
    else
    ret = SdHal_SendCmd(sds, CMD36, blockadddr, CMD36_R, &resp, 1);

    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;	
}


/***********************************
  CMD38: ���Ͳ�������
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Erase()				
**
** ��������: CMD38, ������ѡ��ָ���Ŀ�
**
** �䡡  ��: sd_struct *sds:  SD/MMC����Ϣ�ṹ��
**
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: ��ִ�б�����֮ǰ, ����ִ��SdCmd_Erase_Wr_Blk_Start()��SdCmd_Erase_Wr_Blk_End()
********************************************************************************************************************/
INT8U SdCmd_Erase(sd_struct *sds)
{
	INT8U ret;
	INT32U i,resp;
		
	ret = SdHal_SendCmd(sds, CMD38, 0, CMD38_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;

	for(i=0;i<0x1000;i++);										/* ��ʱһ��ʱ��,�Ա������� */
	
    ret = SdHal_WaitBusy(sds, BUSY_TYPE_ERASE);                 /* �ȴ�����̽��� */
	return ret;		
}


// ��32λ��ת��Ϊ�ֽں���
static void word_to_byte(INT32U value, INT8U *pdst)
{
    pdst[0] = (INT8U)(value >> 24);
    pdst[1] = (INT8U)(value >> 16);
    pdst[2] = (INT8U)(value >> 8);
    pdst[3] = (INT8U)(value);
}

#endif



/*
************************************************
	
	 ����ΪSD/MMC����SPI����

************************************************
*/

#if (!SDBUS_MODE_EN)

/*******************************************************************************************************************
** ��������: SdSpi_SendCmd()				
**
** ��������: SPIģʽ��, ����һ��SD/MMC����, �������Ӧ			
**
** �䡡  ��: sd_struct *sds:  ����Ϣ�ṹ��, ��ʹ��sds->card_posnum��Ա�������ֿ���
**
**			 INT8U cmd:       ��������(�����[5:0]λ)
**
**		     INT32U argument: �������, ����Ϊ4�ֽ�
**
**			 INT8U resptype : ��Ӧ���� 
**							  	
** �� �� ��: INT8U *resp	: ��Ӧ����, ����Ϊ1-5�ֽ�, ��ӦΪR3ʱ, R1 �� resp[4], OCR���ݵ�bit31�� resp[3].
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdSpi_SendCmd(sd_struct *sds, INT8U cmd, INT8U *param, INT8U resptype, INT8U *resp)
{
	INT32 i,rlen;
	INT8U tmp;
    
	SdSpiHal_CSAssert(sds);
   
    SdSpiHal_SendByte(sds, (cmd & 0x3F) | 0x40);	 /* ��������ͷ�������� send command header and word */
    
    for (i = 3; i >= 0; i--)
        SdSpiHal_SendByte(sds, param[i]);			 /* ���Ͳ��� send parameters */

#if SD_CRC_EN
	tmp = SD_GetCmdByte6((cmd & 0x3F) | 0x40, param);
	SdSpiHal_SendByte(sds, tmp);
#else
    SdSpiHal_SendByte(sds, 0x95);					 /* CRCУ����,ֻ���ڵ�1������ CRC,only used for the first command */
#endif 
    
    rlen = 0;
    switch (resptype)								 /* ���ݲ�ͬ������,�õ���ͬ����Ӧ���� */
    {												 /* according various command,get the various response length */
  		case R1:
   	 	case R1B: rlen = 1;  break;
       		 
    	case R2:  rlen = 2;	 break;
       		 
   		case R3:  rlen = 5;	 break;
       		 
    	default:  SdSpiHal_SendByte(sds, 0xFF);	
      		      SdSpiHal_CSDeassert(sds);						 
        	      return SD_ERR_CMD_RESPTYPE;		 /* ����������Ӧ���ʹ��� return error of command response type */
    		      break;
    }
    
    i = 0;				
    do 												 /* �ȴ���Ӧ,��Ӧ�Ŀ�ʼλΪ0 */
    {												 /* Wait for a response,a response is a start bit(zero) */ 
        tmp = SdSpiHal_RecByte(sds);
        i++;
    }while (((tmp & 0x80) != 0) && (i < SD_CMD_TIMEOUT));
    
    if (i >= SD_CMD_TIMEOUT)
    {												 
        SdSpiHal_CSDeassert(sds);
        return SD_ERR_CMD_TIMEOUT;					 /* �������ʱ return response timeout of command */
    }
    
    for (i = rlen - 1; i >= 0; i--)
    {
        resp[i] = tmp;
        tmp = SdSpiHal_RecByte(sds);				 /* ѭ���������8clock  at the last recycle,clock out 8 clock */
    }
      
    SdSpiHal_CSDeassert(sds);
    return SD_NO_ERR;								 /* ����ִ�гɹ� return perform sucessfully */
}


/*******************************************************************************************************************
** ��������: SdSpi_PackParam()				
**
** ��������: ��32λ�Ĳ���תΪ�ֽ���ʽ	
**
** �䡡  ��: INT8U *parameter: �ֽڲ���������
**           INT32U value    : 32λ����	
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
void SdSpi_PackParam(INT8U *parameter, INT32U value)
{
    parameter[3] = (INT8U)(value >> 24);
    parameter[2] = (INT8U)(value >> 16);
    parameter[1] = (INT8U)(value >> 8);
    parameter[0] = (INT8U)(value);
}


/*******************************************************************************************************************
** ��������: SdSpi_BlockCommand()				
**
** ��������: ������	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**           INT8U cmd	     : ������
**           INT8U resptype  : ��Ӧ����
**           INT32U parameter: ���������
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdSpi_BlockCommand(sd_struct *sds, INT8U cmd, INT8U resptype, INT32U parameter)
{
	INT8U param[4],resp,ret;
	
	parameter <<= SD_BLOCKSIZE_NBITS;					 /* ������ַ:����9λ adjust address: move 9 bits left */

	SdSpi_PackParam(param, parameter);					 /* ������ת��Ϊ�ֽ���ʽ change the parameter to bytes form */	

	ret = SdSpi_SendCmd(sds, cmd, param, resptype, &resp);
	if (ret != SD_NO_ERR)
	   	 return ret;							 		 /* �������ݴ���ʧ�� stop transmission operation fail */
	
	if (resp != 0)
		 return SD_ERR_CMD_RESP;		 				 /* ��Ӧ���� response is error */
		 
	return SD_NO_ERR;
}

	
/*************
    CMD 0 
*************/
/*******************************************************************************************************************
** ��������: SpiCmd_Go_Idle_State()				
**
** ��������: CMD0, ��λ��ѡ��Ŀ�, ʹ������Idle״̬		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SpiCmd_Go_Idle_State(sd_struct *sds)
{
	INT8U param[4] = {0,0,0,0},resp;
	
    return (SdSpi_SendCmd(sds, CMD0, param, CMD0_R, &resp));	/* ��λ���� command that reset card */
}


/*************
    CMD 1 
*************/
/*******************************************************************************************************************
** ��������: SpiCmd_Send_OP_Cond()				
**
** ��������: CMD1, ʹ���˳�Idle״̬(�ж���Ӧ R1 �� bit0 λ)		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SpiCmd_Send_OP_Cond(sd_struct *sds)
{
	INT8U param[4] = {0,0,0,0},resp[5],ret;
	INT32U i = 0;
	
 	do 
    {														/* ����CMD1, ��ѯ����״̬, send CMD1 to poll card status */
        ret = SdSpi_SendCmd(sds, CMD1, param, CMD1_R, resp);
        if (ret != SD_NO_ERR)
       		return ret;
        i ++;
    }while (((resp[0] & MSK_IDLE) == MSK_IDLE) && (i <= SD_IDLE_WAIT_MAX));
    														/* �����ӦR1�����λIdleλΪ1,�����ѭ�� */
    														/* if response R1 Idle bit is 1,continue recycle */    
    if (i >= SD_IDLE_WAIT_MAX)
        return SD_ERR_TIMEOUT_WAITIDLE;						/* ��ʱ,���ش��� time out,return error */		
	
	return SD_NO_ERR;
}


/*************
    ACMD 41 
*************/
/*******************************************************************************************************************
** ��������: SpiCmd_SD_App_OP_Cond()				
**
** ��������: ACMD41(������SD��), ��������ʼ������	
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**           
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: 
********************************************************************************************************************/
INT8U SpiCmd_SD_App_OP_Cond(sd_struct *sds)
{
    INT8U param[4] = {0,0,0,0},resp[5],ret;

    ret = SdSpi_SendCmd(sds, CMD55, param, CMD55_R, resp);
    if (ret != SD_NO_ERR)
       	return ret;
       	
    ret = SdSpi_SendCmd(sds, ACMD41, param, ACMD41_R, resp);/* �����ڲ���ʼ������ active card to initialize process internal */
    if (ret != SD_NO_ERR)	
    	return ret;
		
	return SD_NO_ERR;	
}


/*************
    CMD 9 
*************/
/*******************************************************************************************************************
** ��������: SpiCmd_Send_CSD()				
**
** ��������: CMD9, ��ȡ��ѡ��Ŀ���CSD�Ĵ�������		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**           INT8U csdlen    : CSD �Ĵ�������
**	
** �� �� ��: INT8U *csdbuf   : CSD�Ĵ�������, bit127 ��csdbuf[0] 
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: 
********************************************************************************************************************/
INT8U SpiCmd_Send_CSD(sd_struct *sds, INT8U csdlen, INT8U *recbuf)
{
    INT8U param[4] = {0,0,0,0},resp,ret;
  
    ret = SdSpi_SendCmd(sds, CMD9, param, CMD9_R, &resp);		/* ��CSD�Ĵ������� command that read CSD register */
    if (ret != SD_NO_ERR) 									
        return ret;									
  
    if (resp != 0)
        return SD_ERR_CMD_RESP;								    /* ��Ӧ���� response is error */
    
	return (SdSpi_ReadRegister(sds, csdlen, recbuf));           /* ��ȡ�Ĵ������� */
}


/*************
    CMD10
*************/
/*******************************************************************************************************************
** ��������: SpiCmd_Send_CID()				
**
** ��������: ��ȡ��ѡ��Ŀ���CID�Ĵ���ֵ		
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**           INT8U cidlen    : CID �Ĵ�������, �̶�Ϊ16�ֽ�
**	
** �� �� ��: INT8U *cidbuf   : CID �Ĵ�������, ����Ϊ16�ֽ�
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: 
********************************************************************************************************************/
INT8U SpiCmd_Send_CID(sd_struct *sds, INT8U cidlen, INT8U *recbuf)
{
    INT8U param[4] = {0,0,0,0},resp,ret;
 
    ret = SdSpi_SendCmd(sds, CMD10, param, CMD10_R, &resp);		/* ��CID�Ĵ������� command that read CID register */
    if ( ret != SD_NO_ERR)
   		return ret;			  									
   
    if (resp != 0)
        return SD_ERR_CMD_RESP;									/* ��Ӧ���� response is error */
      
  	return (SdSpi_ReadRegister(sds, cidlen, recbuf));
}


/***********************************
  CMD12: ֹͣ����Stop Transmission
************************************/
/*******************************************************************************************************************
** ��������: SdCmd_Send_Stop()				
**
** ��������: CMD12, ֹͣ���ݴ�������
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: �������argument�򲻱ع���, ��������Ҫֹͣ��������ݿ��ֹͣд������ݿ�ʱ���øú���
********************************************************************************************************************/
INT8U SpiCmd_Stop_Transmission(sd_struct *sds)
{
	INT8U param[4] = {0,0,0,0},resp;
	
 	return (SdSpi_SendCmd(sds, CMD12, param, CMD12_R, &resp));		/* �������ݴ�������ʧ�� stop transmission command fail */
}



/*********
  CMD13
**********/
/*******************************************************************************************************************
** ��������: SpiCmd_Send_Status()				
**
** ��������: ��SD/MMC���� Card Status �Ĵ���
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	         INT8U len       : �Ĵ�������(�̶�Ϊ2)  
**
** �� �� ��: INT8U *recbuf : ���ջ�����
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: 
********************************************************************************************************************/
INT8U SpiCmd_Send_Status(sd_struct *sds, INT8U len, INT8U *buffer)
{
    INT8U param[4] = {0,0,0,0};

    return (SdSpi_SendCmd(sds, CMD13, param, CMD13_R, buffer)); 	/* �� Card Status �Ĵ��� */
}


/*********
  CMD16 
**********/
/*******************************************************************************************************************
** ��������: SpiCmd_Set_BlockLen()				
**
** ��������: ���ÿ������ݿ鳤��
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	         INT32U length	 : ���ݿ�ĳ���ֵ  
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: ͨ������һ�������ݿ鳤����һ��д���ݿ鳤��һ��
********************************************************************************************************************/
INT8U SpiCmd_Set_BlockLen(sd_struct *sds, INT32U length)
{
	INT8U param[4],resp,ret;
  
    SdSpi_PackParam(param, length);					/* ������ת��Ϊ�ֽ���ʽ change the parameter to bytes form */
          												
    ret = SdSpi_SendCmd(sds, CMD16, param, CMD16_R, &resp);
    if (ret != SD_NO_ERR)
 		return ret;									/* ���ÿ�ĳ���Ϊlengthʧ�� set the length of block to length fail */
	
	if (resp != 0)
    	return SD_ERR_CMD_RESP;			   			/* ��Ӧ���� response is error */
    
    return SD_NO_ERR; 								/* ����ִ�гɹ� return perform sucessfully */			
}


/*********
  CMD17 
**********/
/*******************************************************************************************************************
** ��������: SpiCmd_Read_Single_Block()				
**
** ��������: ����������
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	         INT32U blockaddr: ���ַ  
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: blockaddr���Կ�Ϊ��λ�Ŀ��ַ
********************************************************************************************************************/
INT8U SpiCmd_Read_Single_Block(sd_struct *sds, INT32U blockaddr)
{
	return (SdSpi_BlockCommand(sds, CMD17, CMD17_R, blockaddr)); /* ���������� command that read single block */
}


/*********
  CMD18 
**********/
/*******************************************************************************************************************
** ��������: SpiCmd_Read_Single_Block()				
**
** ��������: ���������
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	         INT32U blockaddr: ���ַ  
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: blockaddr���Կ�Ϊ��λ�Ŀ��ַ
********************************************************************************************************************/
INT8U SpiCmd_Read_Multiple_Block(sd_struct *sds, INT32U blockaddr)
{
	return (SdSpi_BlockCommand(sds, CMD18, CMD18_R, blockaddr)); /* ��������� command that read multiple block */
}


/*********
  CMD24 
**********/
/*******************************************************************************************************************
** ��������: SpiCmd_Write_Single_Block()				
**
** ��������: д��������
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	         INT32U blockaddr: ���ַ  
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: blockaddr���Կ�Ϊ��λ�Ŀ��ַ
********************************************************************************************************************/
INT8U SpiCmd_Write_Single_Block(sd_struct *sds, INT32U blockaddr)
{
	return (SdSpi_BlockCommand(sds, CMD24, CMD24_R, blockaddr)); /* д�������� command that write single block */
}


/*********
  CMD25 
**********/
/*******************************************************************************************************************
** ��������: SpiCmd_Write_Multiple_Block()				
**
** ��������: д�������
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	         INT32U blockaddr: ���ַ  
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: blockaddr���Կ�Ϊ��λ�Ŀ��ַ
********************************************************************************************************************/
INT8U SpiCmd_Write_Multiple_Block(sd_struct *sds, INT32U blockaddr)
{
	return (SdSpi_BlockCommand(sds, CMD25, CMD25_R, blockaddr)); /* д������� command that write multiple block */
}


/*********
  CMD25 
**********/
/*******************************************************************************************************************
** ��������: SpiCmd_Program_CSD()				
**
** ��������: дCSD�Ĵ���
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ��
**	         INT8U *buff   	 : CSD�Ĵ�������  
**           INT8U len	  	 : CSD�Ĵ�������  
** 
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: 
********************************************************************************************************************/
INT8U SpiCmd_Program_CSD(sd_struct *sds, INT8U len, INT8U *buff)
{
	INT8U param[4] = {0,0,0,0},resp,ret;
	
	if (len != 16) return SD_ERR_USER_PARAM;

	ret = SdSpi_SendCmd(sds, CMD27, param, CMD27_R, &resp); 	/* ����дCSD�Ĵ������� send command that write CSD */
	if (ret != SD_NO_ERR)
		return ret;
		        
    if (resp != 0)    
        return SD_ERR_CMD_RESP;
		
	buff[15] = (SdSpi_GetCRC7(buff, 15) << 1) + 0x01;  	        /* ����CSD�е�crc λ�� calculate crc field in CSD */
		
	return(SdSpi_WriteBlockData(sds, 0, 16, buff));
}


/********************************************************************************************************************
** ��������: INT8U SD_GetCRC7()						Name:	  INT8U SD_GetCRC7()
** ��������: ����CRC7								Function: calculate crc7
** �䡡  ��: INT8U *pSource: ����					Input:    INT8U *pSource: data
			 INT16U len    : ���ݳ���						  INT16U len   : data length
** �� �� ��: CRC7��									Output:	  CRC7 code
*********************************************************************************************************************/
INT8U SdSpi_GetCRC7(INT8U *pSource, INT16U len)
{
	INT8U i = 0, j;
	INT8U reg = 0;
	
	do
	{
	    for (j = 0; j < 8; j++)
	    {
			reg <<= 1;
			reg ^= ((((pSource[i] << j) ^ reg) & 0x80) ? 0x9 : 0);
	    }
	    
	    i++;
	    
	}while(i < len);
	
	return reg;
}	
	

#if SD_EraseBlock_EN
/***************
  CMD32��CMD35 
***************/
/*******************************************************************************************************************
** ��������: SdCmd_Erase_Wr_Blk_Start()				
**
** ��������: ���ÿ������ʼ��ַ
**
** �䡡  ��: sd_struct *sds    : SD/MMC����Ϣ�ṹ��
**	         INT32U startblock : ���ַ
** 
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: startblock���Կ�Ϊ��λ�Ŀ��ַ
********************************************************************************************************************/
INT8U SdCmd_Erase_Wr_Blk_Start(sd_struct *sds, INT32U startblock)
{
	if (sds->card_type == CARDTYPE_SD ||(sds->card_type == CARDTYPE_SDHC))
		return (SdSpi_BlockCommand(sds, CMD32, CMD32_R, startblock));	/* ���Ͳ�����ʼ���ַ send the start block address of erasing operation */
	else
		return (SdSpi_BlockCommand(sds, CMD35, CMD35_R, startblock));	/* ���Ͳ�����ʼ���ַ send the start block address of erasing operation */
}


/*****************
  CMD33 �� CMD36
*****************/
/*******************************************************************************************************************
** ��������: SdCmd_Erase_Wr_Blk_End()				
**
** ��������: ���ÿ������ֹ��ַ
**
** �䡡  ��: sd_struct *sds    : SD/MMC����Ϣ�ṹ��
**	         INT32U endblock   : ���ַ
** 
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: startblock���Կ�Ϊ��λ�Ŀ��ַ
********************************************************************************************************************/
INT8U SdCmd_Erase_Wr_Blk_End(sd_struct *sds, INT32U endblock)
{
	if (sds->card_type == CARDTYPE_SD ||(sds->card_type == CARDTYPE_SDHC))
		return (SdSpi_BlockCommand(sds, CMD33, CMD33_R, endblock));     /* ���Ͳ�����ֹ���ַ send the end block address of erasing operation */
	else
		return (SdSpi_BlockCommand(sds, CMD36, CMD36_R, endblock));     /* ���Ͳ�����ֹ���ַ send the end block address of erasing operation */
}


/*****************
  CMD38
*****************/
/*******************************************************************************************************************
** ��������: SdCmd_Erase()				
**
** ��������: ������ѡ�еĿ�
**
** �䡡  ��: sd_struct *sds    : SD/MMC����Ϣ�ṹ��
** 
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע	 ��: startblock���Կ�Ϊ��λ�Ŀ��ַ
********************************************************************************************************************/
INT8U SdCmd_Erase(sd_struct *sds)
{
	INT8U param[4],resp,tmp;
	
	SdSpi_PackParam(param, 0);
	
	tmp = SdSpi_SendCmd(sds, CMD38, param, CMD38_R, &resp);	 	    /* ������ѡ��Ŀ�  erase blocks selected */
	if (tmp != SD_NO_ERR)
		return tmp;							 	
	
	if (SdSpi_WaitBusy(sds, SD_WAIT_ERASE) != SD_NO_ERR)			/* �ȴ�������� wait for finishing erasing */
		return SD_ERR_TIMEOUT_ERASE;
	else
		return SD_NO_ERR;									
}	
#endif


/*****************
  CMD58
*****************/
/*******************************************************************************************************************
** ��������: SpiCmd_ReadOCR()				
**
** ��������: ����ѡ��Ŀ��Ĳ��������Ĵ���OCR
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**
** �� �� ��: INT32U *ocrbuf   : OCR�Ĵ�������, ����Ϊ1 
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SpiCmd_ReadOCR(sd_struct *sds, INT32U *ocrbuf)
{
    INT8U param[4] = {0,0,0,0},resp[5],tmp;

    tmp = SdSpi_SendCmd(sds, CMD58, param, CMD58_R, resp);	/* �� OCR �Ĵ������� */
    if (tmp != SD_NO_ERR)								/* read OCR register command */
    	return tmp;		 										
    												
    if (resp[4] != 0)
        return SD_ERR_CMD_RESP;			 				/* ��Ӧ���� response is error */
          
    *ocrbuf = (resp[3] << 24) + (resp[2] << 16) +
              (resp[1] << 8) + resp[0];                 /* ����OCR�Ĵ������ݵ����ջ����� */
            
    return SD_NO_ERR;
}


/********
  CMD59
*********/
/*******************************************************************************************************************
** ��������: SpiCmd_CRC_ON_OFF()				
**
** ��������: SPIģʽ��, ʹ�������뿨֮������ݴ����Ƿ�ʹ��CRCУ��
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**           INT8U bEnable    : 1: ʹ��;  0: ��ֹ 
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
#if SD_CRC_EN
INT8U SpiCmd_CRC_ON_OFF(sd_struct *sds, INT8U bEnable)
{
	INT8U param[4],resp,ret;
		
	if (bEnable == 1)
		param[0] = 1;											/* ʹ��crc enable crc */
	else
		param[0] = 0; 											/* ��ֹcrc disalbe crc */

	ret = SdSpi_SendCmd(sds, CMD59, param, CMD59_R, &resp);		/* "ʹ��/��ֹCRC"���� enable/disable crc command */
	if (ret != SD_NO_ERR)
        return ret;							 
    
    if (resp != 0)    
       	return SD_ERR_CMD_RESP;									/* ��Ӧ���� response is error */		
 	
	return SD_NO_ERR;
}
#endif


/********
  ACMD13
*********/
/*******************************************************************************************************************
** ��������: SpiCmd_ReadSD_Status()				
**
** ��������: ��SD����SD Status �Ĵ���(MMC��û�и�����)
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**           INT8U sdslen  	  : �Ĵ�������(�̶�Ϊ64) 
**
** �� �� ��: INT8U *recbuf 	  : ���ջ�����
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: ֻ��SD������SD Status �Ĵ���
********************************************************************************************************************/
INT8U SpiCmd_ReadSD_Status(sd_struct *sds, INT8U sdslen, INT8U *recbuf)
{
    INT8U param[4] = {0,0,0,0},resp[2],ret;
    
    ret = SdSpi_SendCmd(sds, CMD55, param, CMD55_R, resp);		/* ��������Ϊһ��Ӧ������ */
    if (ret != SD_NO_ERR)
    	return ret;											    /* command that the followed commnad is a specific application */
    												 
    if (resp[0] != 0)
        return SD_ERR_CMD_RESP;								    /* ��Ӧ���� response is error */
    
    ret = SdSpi_SendCmd(sds, ACMD13, param, ACMD13_R, resp);	/* �� SD_Status ���� */
    if (ret != SD_NO_ERR)
    	return ret;											    /* command that read SD_Status register */
   												
    if ((resp[0] != 0) || (resp[1] != 0))
        return SD_ERR_CMD_RESP;								    /* ��Ӧ���� response is error */
        
	return (SdSpi_ReadBlockData(sds, sdslen, recbuf));			/* �����Ĵ������� read the content of the register */
}


/********
  ACMD51
*********/
/*******************************************************************************************************************
** ��������: SpiCmd_Send_SCR()				
**
** ��������: ��SD���� SCR �Ĵ���(MMC��û�и�����)
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**           INT8U sdslen  	  : �Ĵ�������(�̶�Ϊ64) 
**
** �� �� ��: INT8U *recbuf 	  : ���ջ�����
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: MMC��û�иüĴ���
********************************************************************************************************************/
INT8U SpiCmd_Send_SCR(sd_struct *sds, INT8U scrlen, INT8U *recbuf)
{
    INT8U param[4] = {0,0,0,0},resp,ret;
    
    ret = SdSpi_SendCmd(sds, CMD55, param, CMD55_R, &resp);		/* ��������Ϊһ��Ӧ������ */
    if (ret != SD_NO_ERR)								        /* command that the followed commnad is a specific application */
    	return ret;													
    												 
    if (resp != 0)
        return SD_ERR_CMD_RESP;							        /* ��Ӧ���� response is error */
    
    ret = SdSpi_SendCmd(sds, ACMD51, param, ACMD51_R, &resp);   /* ���Ͷ� SD Status ����*/
    if (ret != SD_NO_ERR)								        /* command that read SD Status register */
   		return ret;													
				    															
    if (resp != 0)
        return SD_ERR_CMD_RESP;						 	        /* ��Ӧ���� response is error */
        
	return (SdSpi_ReadBlockData(sds, scrlen, recbuf));	 		/* �����Ĵ������� read the content of the register */
}


/********
  ACMD22
*********/
/*******************************************************************************************************************
** ��������: SpiCmd_Send_Num_WR_Blcoks()				
**
** ��������: �õ���ȷд��Ŀ���
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**
** �� �� ��: INT32U *blocknum : ���صĿ���, ���������ȹ̶�Ϊ1
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: MMC��û�и�����
********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
INT8U SpiCmd_Send_Num_WR_Blcoks(sd_struct *sds, INT32U *blocknum)
{
    INT8U tmp[4] = {0,0,0,0},resp,ret;
  
    ret = SdSpi_SendCmd(sds, CMD55, tmp, CMD55_R, &resp);	 /* ��������Ϊһ��Ӧ������ */
    if (ret != SD_NO_ERR) 								     /* command that the followed commnad is a specific application */
    	return ret;
    	 
    if (resp != 0)
    	return SD_ERR_CMD_RESP;    	
 											
   	ret = SdSpi_SendCmd(sds, ACMD22, tmp, ACMD22_R, &resp); /* ��ȡ��ȷд��Ŀ������� */
   	if (ret != SD_NO_ERR)								    /* command that read the numbers of block written correctly */
   		return ret;											    
   		 														
	if (resp != 0)
    	return SD_ERR_CMD_RESP;							    /* ��Ӧ���� response is error */
    		
    ret = SdSpi_ReadBlockData(sds, 4, tmp);				    /* ������ read the numbvers of block */
    if (ret != SD_NO_ERR)
    	return ret;
    	
    *blocknum = (tmp[0] << 24) + (tmp[1] << 16) + (tmp[2] << 8) + tmp[3];	
    													    /* ת��Ϊ32λ change to 32 bits */

	return SD_NO_ERR;    								    /* ����ִ�гɹ� return perform sucessfully */		
}
#endif



		/*********************************************************
		
		    			����ΪһЩ���ݴ��亯��
		
		**********************************************************/

/*******************************************************************************************************************
** ��������: SdSpi_ReadRegister()				
**
** ��������: SPIģʽ��, ��SD/MMC���Ĵ�������
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**           INT32U len	      : �����ֽڳ���
**
** �� �� ��: INT8U *recbuf    : ���ջ�����
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: �ú���ֻ��Ӧ���ڶ��Ĵ���ʱʹ��, ���ʺ��ڶ����ݿ�
********************************************************************************************************************/
INT8U SdSpi_ReadRegister(sd_struct *sds, INT32U len, INT8U *recbuf)
{	
	INT32U i = 0;
	INT8U resp;

    SdSpiHal_CSAssert(sds);
    do{    											/* �ȴ�������ʼ���� wait for data start token */
        resp = SdSpiHal_RecByte(sds);
    	i++;
    }while((resp == 0xFF) && (i < SD_READREG_TIMEOUT));    
    
    if (i >= SD_READREG_TIMEOUT)
    {
    	SdSpiHal_CSDeassert(sds);
    	return SD_ERR_TIMEOUT_READ;					/* ��ʱ, ���ش��� timeout, return error */
  	}
  	
   	if (resp != SD_TOK_READ_STARTBLOCK)				
   	{												/* �����յ�������ʼ���� not receive data start token */
		recbuf[0] = resp;							
		i = 1;										/* ����len - 1���ֽ�Ҫ���� still len - 1 bytes will be received */
   	}
   	else
   		i = 0;										/* �յ�������ʼ����,����len���ֽ�Ҫ���� received data start token,still len bytes will be received */
   	  	
    for (; i < len; i++)
   		recbuf[i] = SdSpiHal_RecByte(sds);			/* �������� receive data */
   		
    i = SdSpiHal_RecByte(sds);								
    i = (i << 8) + SdSpiHal_RecByte(sds);   		/* ��ȡ16λCRC get 16-bit CRC */	

#if SD_CRC_EN 
   	if (i != SD_GetCRC16(recbuf, len))
   	{												/* CRCУ����� CRC check is error */
   		SdSpiHal_SendByte(sds, 0xFF);
   		SdSpiHal_CSDeassert(sds);		
  		return SD_ERR_DATA_CRC16;					/* ����RCR16����  return error of CRC16 */				
  	}    
#endif   
  
    SdSpiHal_SendByte(sds, 0xFF);					/* ����֮ǰ����8��clock  clock out 8 clk before return */
    SdSpiHal_CSDeassert(sds);	
    	
	return SD_NO_ERR;
}	


#define  SD_UCOSII_SMALLWAIT  256

/*******************************************************************************************************************
** ��������: SdSpi_ReadBlockData()				
**
** ��������: SPIģʽ��, �ӿ��ж�ȡ���ݿ�
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**           INT32U len	      : �����ֽڳ���
**
** �� �� ��: INT8U *recbuf    : ���ջ�����
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: �ú���ֻ���ڶ����ݿ�, �����ʺ��ڶ����Ĵ�������
********************************************************************************************************************/
INT8U SdSpi_ReadBlockData(sd_struct *sds, INT32U len, INT8U *recbuf)
{
    INT8U tmp;
    INT32U i = 0,timeout;
    
#if SD_UCOSII_EN    
    timeout = SD_UCOSII_SMALLWAIT;						/* ���ٵĵȴ�ʱ�� small wait time */
#else
	timeout = sds->timeout_read;						/* �ȴ��������ݿ�ʼ�����ʱ�� wait time that receive data start token */
#endif  
    
    SdSpiHal_CSAssert(sds);    
    do
    { 											    	/* �ȴ��������ݿ�ʼ����0xFE  wait for receiving data start token 0xFE */
        tmp = SdSpiHal_RecByte(sds);
        i++;
    }while((tmp == 0xFF) && (i < timeout));
	
#if SD_UCOSII_EN    
    if (i >= timeout)
   	{													/* �����ȴ�(��������) continue to wait(suspend task) */
   		timeout = sds->timeout_read;
   		i = 0;
	    do
    	{
        	OSTimeDly(1);								/* ���������1 tick(suspend task 1 tick) */
        	tmp = SdSpiHal_RecByte(sds);
       	 	i++;
   		}while((tmp == 0xFF) && (i < timeout));
	}
#endif

	if (i >= timeout)
	{
		SdSpiHal_CSDeassert(sds);
       	return SD_ERR_TIMEOUT_READ;						/* ���ض���ʱ������  return error timeout error code of reading */
	}
	
	if (tmp != SD_TOK_READ_STARTBLOCK)					/* �����ʼ���ƴ��� read start block token is error */
	{
		SdSpiHal_SendByte(sds, 0xFF);
		SdSpiHal_CSDeassert(sds);
		return SD_ERR_DATA_START_TOK;
	}
	
	for (i = 0; i < len; i++)
   		recbuf[i] = SdSpiHal_RecByte(sds);				/* �������� receive data */
   		
    i = SdSpiHal_RecByte(sds);							
    i = (i << 8) + SdSpiHal_RecByte(sds);    			/* ��ȡ16λCRC get 16-bit CRC */	

#if SD_CRC_EN 
   	if (i != SD_GetCRC16(recbuf, len))
   	{	
   		SdSpiHal_SendByte(sds, 0xFF); 					/* CRCУ����� CRC check is error */
   		SdSpiHal_CSDeassert(sds);		
  		return SD_ERR_DATA_CRC16;						/* ����RCR16����  return error of CRC16 */				
  	}    
#endif   

	SdSpiHal_SendByte(sds, 0xFF); 
	SdSpiHal_CSDeassert(sds);

  	return SD_NO_ERR;									/* ���غ���ִ�гɹ� return function perform sucessfully */
}


/*******************************************************************************************************************
** ��������: SdSpi_WriteBlockData()				
**
** ��������: SPIģʽ��, ��д���ݿ�
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**           INT8U bmulti     : �Ƿ�Ϊ������1:��0:��
**           INT32U len	      : �����ֽڳ���
**           INT8U *sendbuf   : ���ͻ�����
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: 
********************************************************************************************************************/
INT8U SdSpi_WriteBlockData(sd_struct *sds, INT8U bmulti, INT32U len, INT8U *sendbuf)
{
	INT16U i;
	INT8U tmp;

	SdSpiHal_CSAssert(sds);
		
    SdSpiHal_SendByte(sds, 0xFF);							/* ��ʼ��������֮ǰ����8��clock clock out 8 clk before start */
    
    if (bmulti == 1)
        SdSpiHal_SendByte(sds, SD_TOK_WRITE_STARTBLOCK_M);	/* д��鿪ʼ���� start token of write multi blocks */
	else
		SdSpiHal_SendByte(sds, SD_TOK_WRITE_STARTBLOCK);	/* д���鿪ʼ���� start token of write single block */

	for (i = 0; i < len; i++)
        SdSpiHal_SendByte(sds, sendbuf[i]);					/* �������� send data */

#if SD_CRC_EN	
	i = SD_GetCRC16(sendbuf,len);							/* ����CRC16 calculate CRC16 */
#endif

	SdSpiHal_SendByte(sds, (i >> 8) & 0xFF);
	SdSpiHal_SendByte(sds, i & 0xFF); 						/* ����CRC16У���� send CRC16 check code */
			    
	tmp = SdSpiHal_RecByte(sds);
  	if ((tmp & SD_RESP_DATA_MSK) != SD_RESP_DATA_ACCETPTED)	
  	{		
   		SdSpiHal_SendByte(sds, 0xFF);						/* ����֮ǰ����8��clock  clock out 8 clk before return */
   		SdSpiHal_CSDeassert(sds);
   		return SD_ERR_DATA_RESP;							/* ������Ӧ���� data response error */
    }
        
    SdSpiHal_CSDeassert(sds);
     		
    if (SdSpi_WaitBusy(sds, SD_WAIT_WRITE) != SD_NO_ERR)			
    	return SD_ERR_TIMEOUT_WRITE;						/* д�볬ʱ write time out */
    else
    	return SD_NO_ERR; 									/* д����ȷ write right */
}


/*******************************************************************************************************************
** ��������: SdSpi_StopMultiToken()				
**
** ��������: SPIģʽ��, ���Ͷ��дֹͣ����
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: 
********************************************************************************************************************/
void SdSpi_StopMultiToken(sd_struct *sds)
{
	SdSpiHal_CSAssert(sds);
	
	SdSpiHal_SendByte(sds, 0xFF);							/* �ȷ���8��clock send 8 clock first */
	SdSpiHal_SendByte(sds, SD_TOK_STOP_MULTI);				/* ����ֹͣ���ݴ������� send stop transmission token */
	SdSpiHal_RecByte(sds);
	
    SdSpiHal_CSDeassert(sds);
}


/*******************************************************************************************************************
** ��������: SdSpi_WaitBusy()				
**
** ��������: SPIģʽ��, ��ѯSD���Ƿ���æ״̬
**
** �䡡  ��: sd_struct *sds   : SD/MMC����Ϣ�ṹ��
**           INT32U waittype  : ��ʱ����, ��ȡֵ: SD_WAIT_WRITE �� SD_WAIT_ERASE
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdSpi_WaitBusy(sd_struct *sds, INT8U waittype)
{
    INT32U timeout, i = 0;
    INT8U tmp;
    
  	if (waittype == SD_WAIT_WRITE)
  		timeout = sds->timeout_write;				/* �ȴ�����Ϊд���� wait type is write operation */
  	else
  		timeout = sds->timeout_erase;   			/* �ȴ�����Ϊ�������� wait type is erase operation */
    	
#if SD_UCOSII_EN
        timeout = SD_UCOSII_SMALLWAIT;				/* ���ٵĵȴ�ʱ�� small wait time */
#endif
   
	SdSpiHal_CSAssert(sds);
   	do
   	{ 												/* �ȴ�æ���� wait for being busy end */
        tmp = SdSpiHal_RecByte(sds);
        i++;
    }while ((tmp != 0xFF) && (i < timeout));		/* æʱ�յ���ֵΪ0 always receive 0 when card is busy */    

#if SD_UCOSII_EN
	if (i >= timeout)
	{												/* ���ٵȴ�����æ, after small wait, card is still busy */
  		if (waittype == SD_WAIT_WRITE)
  			timeout = sds->timeout_write;
  		else
  			timeout = sds->timeout_erase;   
			
		i = 0;
   		do
   		{ 	
   			OSTimeDly(1);							/* ����ϵͳ����1 tick  OS suspend 1 tick */
       		tmp = SdSpiHal_RecByte(sds);
       		i++;
    	}while ((tmp != 0xFF) && (i < timeout));	/* æʱ�յ���ֵΪ0 always receive 0 when card is busy */    
	}
#endif	

	if(i < timeout) 
		tmp = SD_NO_ERR;							/* ����0,��ʾû��ʱ return 0 indicate that operation is not time out */
	else 
		tmp = SD_ERR_TIMEOUT_WAIT;					/* ���ش�����,��ʾ��ʱ return error code indicate that operation is time out */

	SdSpiHal_SendByte(sds, 0xFF);
	SdSpiHal_CSDeassert(sds);							
	return tmp;										/* ����ִ�н�� */
}


/*******************************************************************************************************************
** ��������: SdSpi_Delay()				
**
** ��������: SPIģʽ��, SPI������ʱ, ��8��clockΪ��λ
**
** �䡡  ��: sd_struct *sds : SD/MMC����Ϣ�ṹ��
**           INT8U value    : ��ʱֵ,������255
**	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
void SdSpi_Delay(sd_struct *sds, INT8U value)
{
    INT8U i;

    for (i = 0; i < value; i++)
        SdSpiHal_SendByte(sds, 0xFF);				/* ����0xFF clock out 0xFF */
}


#endif 




