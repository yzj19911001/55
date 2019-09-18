/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdhal.c
** Last modified Date:	2007-5-20
** Last Version:		V2.0
** Descriptions:		SD/MMC����дģ��: Ӳ������� ---- SD����Ӳ����صĲ�������
**
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
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

#include "sdconfig.h"
#include "LPC3200.h"
#include "string.h"
#include "sddriver.h"

#if SDBUS_MODE_EN

#if LPC23xx_MCI_DMA_EN
//INT32U *src_addr; 
//INT32U *dest_addr;
//uint8 src_addr[2048];
//uint8 dest_addr[2048];
#endif

//union {
//uint8 a[4];
//INT32U b;
//}  senduart5;
//uint32 MCI_count=0;
 

/* mci_struct �ṹ���Ա RxTxFlag ȡֵ */
#define  MCI_RESET_FLAG             0xFF
#define  MCI_CUR_REC                0x01
#define  MCI_CUR_SEND               0x02
uint8 DMA_SRC[2048]; //(INT32U)DMA_SRC
uint8 DMA_DST[2048];

/* MCI���ݴ���ṹ�� */
typedef struct MCI_STRUCT
{
    INT8U CurTran;              // ��ǰ���ݴ�������, ��ΪMCI_CUR_REC��MCI_CUR_SEND
    INT8U RxTxFlag;             // ���ݽ���/���ͱ�־, ���ڱ�ʶ����/����ִ�н��

    INT8U *pRecBuf;             // �������ݻ�����ָ��
    INT8U *pSendBuf;            // �������ݻ�����ָ��
    INT32U RecCnt;              // �������ݼ�����
    INT32U SendCnt;             // �������ݼ�����
    
    sd_struct *psds;            // ��ǰsd_struct�ṹ��ָ��
}mci_struct;

volatile mci_struct lpc_mci_struct;    // mci_struct�ṹ�����
volatile mci_struct *lpc_mci = &lpc_mci_struct;


void SD_PowerOff(void);
void MCI_Delay(INT16U val);
void MCI_SendCmd(INT32U CmdIndex, INT32U Argument, INT32U ExpectResp, INT32U AllowTimeout );
INT8U MCI_GetCmdResp(INT32U ExpectCmdData, INT32U ExpectResp, INT8U resplen, INT32U *CmdResp);

	/**********************************************
		
		Ӳ�������ӿں���ʵ���ļ�
	
	***********************************************/

/*********************************************************************************************************
** ��������: SdHal_Initialize()				
**
** ��������: ��ʼ�����ʿ���Ӳ������		
**
** �䡡  ��: sd_struct *sds:  ����Ϣ�ṹ��
**							  	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ��Ҫ����: 1. �ȸ����µ�, �ٸ����ϵ�;     2. ���÷��ʿ����ٶ�С�ڻ���� 400KHz;
**
**           3. ʹ CMD �ߴ��ڿ�©״̬;      4. ��ʼ�����Ӳ���Ĵ���Ϊ�ʵ���״̬
*********************************************************************************************************/
INT8U SdHal_Initialize(sd_struct *sds)
{
    // ���µ粢��ʱ
    SD_PowerOff(); 
	MCI_Delay(1);				            

		// ���ϵ粢��ʼ��MCI�������
		SD_POWER_ON();
		MCI_Delay(2);	

    MS_CTRL = 0x222;                                                    /* ʹ��SD��ʱ��    ʱ��2��Ƶ   0x222          */ 
	           
    MCIPower = 0x02;
	MCI_Delay(1);                                                       /* ��ʱ                         */ 
    MCIPower |= 0x01;
    MCIPower |= (1 << 6 );	                                            /* CMD�ߴ��ڿ�©״̬            */

	// ���÷��ʿ�ʱ��С�� 400KHz
	SdHal_SetMCIClock(sds, SD_RATE_SLOW); 				                /*��SD����ʼ��ʱӦ������ʱ��С��400K����ʼ����ɺ������ʱ��Ƶ��*/
    SdHal_EnableMCIBusWidth(sds,1);
	// ��� MCI ��ؼĴ���		
    MCICommand  = 0;						    
	MCIClear    = 0x7FF;						                        /* ���MCI״̬�Ĵ���             */
	MCIDataCtrl = 0;						
    MCIMask0    = 0;							
    MCIMask1    = 0;                                                    /* �������� MCI �ж�             */
    
	return SD_NO_ERR;
}												


/*******************************************************************************************************************
** ��������: SD_PowerOff()				
**
** ��������: �����µ�, ��	
**
** �䡡  ��: ��
**							  	
** �� �� ��: ��
**
** �� �� ֵ: ��
********************************************************************************************************************/
void SD_PowerOff(void)
{

	SD_POWER_OFF();							/* �ر� SD ����Դ  turn off power of sd card */
	

	MCIPower = 0x00;                        /* ���õ�Դ���ƼĴ��� */
    MCIClock = 0;
    MCIClock &= ~(1 << 8);                  /* �ر�MCICLKʱ�� */	
}


/*******************************************************************************************************************
** ��������: SdHal_OpenDrainCmd()				
**
** ��������: ���� CMD �ź��ߵĿ�©״̬ 	
**
** �䡡  ��: sd_struct *sds  : ����Ϣ�ṹ��
**           INT8U bOpenDrain: > 0: CMD �ź��߿�©;  = 0: CMD �ź��ߴ��ڷǿ�© 
**          							  	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ��Ҫ����: 1. ��ʼ�����Ӳ���Ĵ���;  2. �ȸ����µ�, ���ϵ�;  3. ���÷��ʿ����ٶ�С�ڻ���� 400KHz
**
**           4. ʹ CMD �ߴ��ڿ�©״̬
********************************************************************************************************************/
void SdHal_OpenDrainCmd(sd_struct *sds, INT8U bOpenDrain)
{
    if (bOpenDrain)
        MCIPower |= (1 << 6 );	            /* CMD�ߴ��ڿ�©״̬ */ 
            
    else
        MCIPower &= ~(1 << 6 );	            /* CMD�ߴ��ڷǿ�©״̬ */
              
}


/*******************************************************************************************************************
** ��������: SdHal_SendCmd()				
**
** ��������: ����һ��SD/MMC����, �������Ӧ			
**
** �䡡  ��: sd_struct *sds:  ����Ϣ�ṹ��, ��ʹ��sds->card_posnum��Ա�������ֿ���
			 INT8U cmd:       ��������(�����[5:0]λ)
**		     INT32U argument: �������
**			 INT8U resptype:  ��Ӧ����, ȡֵ����ΪR0(������Ҫ��Ӧ), R1, R1B, R2, R3, R6 
**			 INT8U resplen:   ��Ӧ����, 
**                            ����ӦΪR0ʱ, ȡֵΪ0(��Ӧrespָ��Ҳ��Ϊ��).
**                            ����ӦΪR1,R1B,R3,R6ʱ, ȡֵ4; 
**                            ����ӦΪR2ʱ, ȡֵΪ16(R2��bit[127:0]).
**							  	
** �� �� ��: INT32U *resp:    ��Ӧ����, bit127��resp[0]��bit32
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdHal_SendCmd(sd_struct *sds, INT8U cmd, INT32U argument, INT8U resptype, INT32U *resp, INT8U resplen)
{
    INT32U ExpectResp, i;
    INT8U ret;
    
    if (resptype == R0)
    {
        ExpectResp = EXPECT_NO_RESP;                        /* ��������Ӧ������ */
    } 
    else if ((resptype == R1) || (resptype == R1B) || (resptype == R3) || (resptype == R6)|| (resptype == R7))	 //
    {
        ExpectResp = EXPECT_SHORT_RESP;                     /* ����Ӧ */
    }  
    else if (resptype == R2)
    {
        ExpectResp = EXPECT_LONG_RESP;                      /* ����Ӧ */
    } 
    else 
    {
        return SD_ERR_CMD_RESPTYPE;							/* �������Ӧ���� */
    }
                
    MCI_SendCmd(cmd, argument, ExpectResp, 0);				/* ���������ȡ��Ӧ */
    ret = MCI_GetCmdResp(cmd, ExpectResp, resplen, resp);
    if (ret != SD_NO_ERR)
        return ret;
       
    for (i = 0; i < 0x200; i++);	                        /* hena2011.6.14ԭʼ0x200 ��ʱ, ȷ�� MCICommand �Ĵ����ڹ涨ʱ���ڲ��ٴα�д�� */
    return ret;
}


/*******************************************************************************************************************
** ��������: SdHal_EnableMCIBusWidth()				
**
** ��������: ʹ��SD/MMC�������Ƿ�ʹ�ܿ�����		
**
** �䡡  ��: sd_struct *sds: ����Ϣ�ṹ��, ��ʹ��sds->card_posnum��Ա�������ֿ���
**           INT32U bWidth : > 0:  ʹ�ܿ�����;     0: ��ֹ������     
**							  	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
void SdHal_EnableMCIBusWidth(sd_struct *sds, INT32U bWidth)
{
	if (bWidth != 0)
		MCIClock |= (1 << 11);

	else
	    MCIClock &= ~(1 << 11);
}


/*******************************************************************************************************************
** ��������: SdHal_SetMCIClock()				
**
** ��������: ���ö�дSD/MMC��ʱ��		
**
** �䡡  ��: sd_struct *sds  :  ����Ϣ�ṹ��
**           INT32U ClockRate:	Ҫ���ʱ���ٶ�.  ȡֵSD_RATE_SLOW   ʱ, ���÷��ʿ��ٶ�С�� 400KHz		
**			  	                                 ȡֵSD_RATE_NORMAL ʱ, ���÷��ʿ��ٶ�Ϊ��������ٶȻ�������������ٶ�
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: ��������������ClockRateȡ��ͬ��ֵʱ, �������Ӧ�� sds->clkval ֵ, �������0. 
**
********************************************************************************************************************/
void SdHal_SetMCIClock(sd_struct *sds, INT32U ClockRate)
{
    INT32U i;
    INT32 div = 0;
     
	if (ClockRate == SD_RATE_SLOW)
	{
		div = MCLKDIV_SLOW;	                            /* ����ʱ��С�ڵ���400KHz */  //382KHz
		sds->host_clkval = Fhclk / (div + 1) / 2;     /* MCI ��������Ƶ�� = MCLK / (2 * (div + 1)) */
	}
	else if (ClockRate == SD_RATE_NORMAL) 
	{
	    div = MCLKDIV_NORMAL;	
	        		               
		sds->host_clkval = Fhclk / (div + 1) / 2;       /* MCI ��������Ƶ�� = MCLK / (2 * (div + 1)) */
	}
	
	MCIClock &= ~0xFF;
	MCIClock = (1 << 8)  | div;		                    //SD_CLKʹ��   �����߿���ʱ����SD_CLK | (1 << 9)	
	for (i = 0; i < 0x10; i++);	                        /* ���´�дMCIClockǰ, ��ʱ 3MCLK + 2PCLK */
}


/*******************************************************************************************************************
** ��������: SdHal_CheckCard()				
**
** ��������: ��⿨�Ƿ���ȫ���뿨����
**
** �䡡  ��: sd_struct *sds:  ����Ϣ�ṹ��
**	         
** �� �� ��: ��
**
** �� �� ֵ: 1: ����ȫ����	   0: ��û����ȫ����
********************************************************************************************************************/
INT8U SdHal_CheckCard(sd_struct *sds)
{
	if (SD_INSERT_STATUS() != 0)
		return 0;								/* δ��ȫ���� */
	else
		return 1;								/* ��ȫ���� */
}


/*******************************************************************************************************************
** ��������: SdHal_CheckCardWP()				
**
** ��������: ��⿨д����
**
** �䡡  ��: sd_struct *sds:  ����Ϣ�ṹ��
**	         
** �� �� ��: ��
**
** �� �� ֵ: 1: ����д����	  0: ��δд����
********************************************************************************************************************/
INT8U SdHal_CheckCardWP(sd_struct *sds)
{
	if (SD_WP_STATUS() != 0)
		return 1;								/* д���� */
	else
		return 0;								/* δд���� */
}



/*******************************************************************************************************************
** ��������: SdHal_BeforeReadBlock()				
**
** ��������: �ڷ��Ͷ���������֮ǰ, ��Ӳ����������һЩ����Ļ���
**
** �䡡  ��: sd_struct *sds  :  ����Ϣ�ṹ�� 
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdHal_BeforeReadBlock(sd_struct *sds)
{	

 MCIMask0 |= (DATA_RX_INT_MASK) | (FIFO_INT_MASK);	    /* ��ʹ�ܽ����ж� */
	MCIMask1 = MCIMask0;

	MCIClear = 0x7FF;							            /* �������Ĵ��������б�־ */
	MCIDataCtrl = 0;							            /* ���ݿ��ƼĴ�����0  */
			 
	MCIDataTimer = sds->timeout_read;                       /* �����ݿ鳬ʱֵ */
	MCIDataLength = SD_BLOCKSIZE;				            /* �鳤�� */
    MCIFifoCnt = 0;                                         /* ��ս���FIFO������*/
    
    lpc_mci->RecCnt = 0;                                     /* ��ʼ��mci_struct�ṹ��ȫ�ֱ���lpc_mci */
    lpc_mci->RxTxFlag = MCI_RESET_FLAG;
    lpc_mci->psds = sds;    
    lpc_mci->CurTran = MCI_CUR_REC; 
         
	return SD_NO_ERR;
}


/*******************************************************************************************************************
** ��������: SdHal_ReadBlock()				
**
** ��������: �ڷ��Ͷ���������֮��, ��Ӳ��������ʵ�ֶ�ȡ����
**
** �䡡  ��: sd_struct *sds  :  ����Ϣ�ṹ�� 
**	         
** �� �� ��: INT8U *recbuf: ���ջ�����, ��СΪ512�ֽ�
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdHal_ReadBlock(sd_struct *sds, INT8U *recbuf)
{
	INT32U DataCtrl = 0;
	INT8U ret;
	INT32U i;				//20130826 ��дSD�����ӻ��޸�
	//INT32U i=0;
	lpc_mci->pRecBuf = recbuf;	

#if LPC23xx_MCI_DMA_EN
    // DMA ��������ʼ��
	DMA_Init(1, P2M);                                                           
//	DMACC1Configuration |= 0x00001 | (0x04 << 1) | (0x00 << 6) | (0x06 << 11);		 //hong
	DMACC1Config|= 0x00001 | (0x04 << 1) | (0x00 << 6) | (0x06 << 11);
	// �������ݿ��ƼĴ���: ��,�鴫��,DMA����,�鳤��
	DataCtrl |= ((1 << 0) | (1 << 1) | (1 << 3) |(0 << 2) | (SD_BLOCKSIZE_NBITS << 4));	    
#else
    // �������ݿ��ƼĴ���: ��,�鴫��,�鳤��
	DataCtrl |= ((1 << 0) | (1 << 1) | (SD_BLOCKSIZE_NBITS << 4));
#endif
	
#if SD_UCOSII_EN
    OS_ENTER_CRITICAL();		//20130826 ��дSD�����ӻ��޸�
    MCIDataCtrl = DataCtrl;						                /* �������ݴ��� */
	for(i=0;i<512;i++);			//20130826 ��дSD�����ӻ��޸�
	OS_EXIT_CRITICAL();			//20130826 ��дSD�����ӻ��޸�
    OSSemPend(sds->pSemSdmmcRW, sds->timeout_read_os, &ret);    /* ����ȴ�״̬ */
    if (ret == OS_NO_ERR)   
        ret = lpc_mci->RxTxFlag;                                 /* �������ݽ��ս�� */                            
    else if (ret == OS_TIMEOUT)
        ret = SD_ERR_TIMEOUT_READ;                              /* �������ݳ�ʱ */            
    else
        ret = SD_ERR_WAIT_WRSEM;                                /* �ȴ��ź���ʧ�� */
    
 //   OS_EXIT_CRITICAL();			//20130826 ��дSD�����ӻ��޸�
#else	
	MCIDataCtrl = DataCtrl;                             /* �������ݴ��� */	 
	do
	{                                                  				 /* �ȴ����ݽ��� */
		if (lpc_mci->RxTxFlag == SD_NO_ERR)	
			break;	//�˴������⣬������lpc_mci->RxTxFlag״̬���䣬���޷�����ѭ��
	                                                      /* ���ݽ��ճɹ� */	
	}while (lpc_mci->RxTxFlag == MCI_RESET_FLAG);
	
	ret = lpc_mci->RxTxFlag;                                    /* ���ؽ��ս�� */
#endif	

	MCIMask0 &= ~((DATA_RX_INT_MASK) | (FIFO_INT_MASK));        /* ����ֹ�����ж� */
	MCIMask1 = MCIMask0;  	

#if LPC23xx_MCI_DMA_EN
//  src_addr = (INT32U *)DMA_SRC;                               /* ȡ�� DMA Դ��ַ */         
//	dest_addr = (INT32U *)DMA_DST;                              /* ȡ�� DMA Ŀ���ַ */    


    if (ret == SD_NO_ERR)
        memcpy(recbuf, DMA_DST, 512);                         /* ���ؽ��յ������� */

//        memcpy(recbuf, dest_addr, 512);                         /* ���ؽ��յ������� */
#endif

    return ret;
}





/*******************************************************************************************************************
** ��������: SdHal_BeforeWriteBlock()				
**
** ��������: �ڷ���д��������֮ǰ, ��Ӳ����������һЩ����Ļ���
**
** �䡡  ��: �� 
**	         
** �� �� ��: INT8U *recbuf: ���ջ�����, ��СΪ512�ֽ�
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdHal_BeforeWriteBlock(sd_struct *sds)
{
	MCIMask0 = DATA_TX_INT_MASK | (FIFO_INT_MASK);	      /* ��ʹ��TX�ж� */
	MCIMask0 = MCIMask0&(~0x105000);		//hong,2012.08.10
	MCIMask1 = MCIMask0; 

	MCIClear = 0x7FF;							          /* �������Ĵ��������б�־ */
	MCIDataCtrl = 0;							          /* ���ݿ��ƼĴ�����0 */
		 
	MCIDataTimer = sds->timeout_write;                    /* д��д���ݳ�ʱֵ */
	MCIDataLength = SD_BLOCKSIZE;				          /* ���ݿ�ĳ��� */
    MCIFifoCnt = 0;  
    
    lpc_mci->psds = sds;                                   /* ��ʼ��lpc_mci�ṹ�������Ա */  
    lpc_mci->RxTxFlag = MCI_RESET_FLAG; 
    lpc_mci->SendCnt = 0;
    lpc_mci->CurTran = MCI_CUR_SEND;   
	return SD_NO_ERR;
}


/*******************************************************************************************************************
** ��������: SdHal_WriteBlock()				
**
** ��������: �ڷ���д��������֮��, ��Ӳ���������򿨷�������
**
** �䡡  ��: �� 
**	         
** �� �� ��: INT8U *recbuf: ���ջ�����, ��СΪ512�ֽ�
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdHal_WriteBlock(sd_struct *sds, INT8U *sendbuf)
{
	INT32U DataCtrl = 0;
	INT32U i;					//20130826 ��дSD�����ӻ��޸�
	INT8U  ret;
//	INT32U  i=0;
//	INT32U *sendbuffer;

#if LPC23xx_MCI_DMA_EN

//  src_addr = (INT32U *)DMA_SRC;          //hong                
//	dest_addr = (INT32U *)DMA_DST;	   


	memcpy(DMA_SRC, sendbuf, 512);                              /* ����Ҫд������ݵ�DMAԴ��ַ */
	
	DMA_Init(0, M2P);                                            /* DMA��������ʼ�� */
//	DMACC0Configuration |= 0x00001 | (0x00 << 1) | (0x04 << 6) | (0x05 << 11); //hong
	DMACC0Config |= 0x00001 | (0x00 << 1) | (0x04 << 6) | (0x05 << 11);
	/* �������ݿ��ƼĴ���: д, �鴫��, DMA, ���ݳ��� */
	DataCtrl |= ((1 << 0) | (1 << 3) | (SD_BLOCKSIZE_NBITS << 4)|(0<<2)|(0<<1));
//	MCIDataCtrl = DataCtrl;        //20130826 ��дSD�����ӻ��޸�         
#else
 	lpc_mci->pSendBuf = sendbuf;
 	
 	/* �������ݿ��ƼĴ���: ��, �鴫��, ���ݳ��� */
 	DataCtrl |= ((1 << 0) | (SD_BLOCKSIZE_NBITS << 4));
#endif

#if SD_UCOSII_EN
	OS_ENTER_CRITICAL(); 	   //��ֹ�������
	MCIDataCtrl = DataCtrl;    /* �������ݴ��� */
	for(i=0;i<512;i++);		   //��ʱ				//20130826 ��дSD�����ӻ��޸�
	OS_EXIT_CRITICAL();		   //ʹ���������		//20130826 ��дSD�����ӻ��޸�
	OSSemPend(sds->pSemSdmmcRW, sds->timeout_write_os, &ret);    /* ����ȴ�״̬ */
//	OS_EXIT_CRITICAL();									//20130424 hyw  //20130826 ��дSD�����ӻ��޸�
    if (ret == OS_NO_ERR)
	    ret = lpc_mci->RxTxFlag;                                 /* ����д�����ݽ�� */
    else if (ret == OS_TIMEOUT)
        ret = SD_ERR_TIMEOUT_WRITE;                              /* ���صȴ���ʱ */
    else
        ret = SD_ERR_WAIT_WRSEM;                                 /* OSSemPend()ִ��ʧ�� */
    
//    OS_EXIT_CRITICAL();
#else
    MCIDataCtrl = DataCtrl;                                      /* �������ݴ��� */
	do
	{
//  	    i++;
	    if (lpc_mci->RxTxFlag == SD_NO_ERR)
	        break;                                               /* �������ݳɹ� */   
     	}while (lpc_mci->RxTxFlag == MCI_RESET_FLAG);
	ret = lpc_mci->RxTxFlag;                                      /* ���ط�������ִ�н�� */
#endif

	MCIMask0 &= ~(DATA_TX_INT_MASK | FIFO_INT_MASK);	         /* ����ֹTX�ж� */
	MCIMask1 = MCIMask0;  	
  
	return ret;
}


/*******************************************************************************************************************
** ��������: SDMMC_WaitReadyforData()				
**
** ��������: �ȴ����ɽ�������(�ȴ������пյĽ��ջ�����), ���ȴ���״̬�Ĵ����� bit8 Ϊ1, 
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	         
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdHal_WaitReadyforData(sd_struct *sds)
{    
    return (SdHal_WaitBusy(sds, BUSY_TYPE_RDY_DATA));
}


#define   SD_UCOSII_SMALLWAIT		256				            /* ������UCOS-IIʱ��С�ȴ�ʱ��(����OSTimeDly(1)����ϵͳ����) */
#define   CMD13_CLK_NUM             160                         /* һ��CMD13��Լ��Ҫ160��clk */   

/*******************************************************************************************************************
** ��������: SdHal_WaitBusy()				
**
** ��������: �ȴ�����̻��������, ���ȴ���״̬�ɱ��״̬��Ϊ����״̬: prg --> tran
**
** �䡡  ��: sd_struct *sds  : SD/MMC����Ϣ�ṹ�� 
**	         INT32U BusyType : �ȴ�æ����, ȡֵΪ: SD_WAIT_WRITE �� SD_WAIT_ERASE
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U SdHal_WaitBusy(sd_struct *sds, INT32U busytype)
{
    INT8U ret;
    INT32U status, timeout = 0, cnt = 0;
    INT32U stmask = CARD_STATUS_CURRENT_STATE;                
    INT32U stwait = CARD_STATUS_PRG;                            /* �ȴ�ʱ, �����ڱ��״̬ */
    
    if (busytype == BUSY_TYPE_RDY_DATA)
    {
        timeout = sds->timeout_write / CMD13_CLK_NUM;           /* д��ʱ�ȴ�ʱ�� */
        stmask = CARD_STATUS_RDY_DATA;
        stwait = 0;                                             /* �ȴ�ʱ, ���Ľ��ջ�����δ���� */
    }
    else if (busytype == BUSY_TYPE_PROG)
        timeout = sds->timeout_write / CMD13_CLK_NUM;           /* ������ʱ�ȴ�ʱ�� */
    else if (busytype == BUSY_TYPE_ERASE)
        timeout = sds->timeout_erase / CMD13_CLK_NUM;
            
#if SD_UCOSII_EN 
    timeout = SD_UCOSII_SMALLWAIT;                              /* �Ƚ��п��ٲ�ѯ, ��ÿ��1 os tick��ѯ */
#endif

    do	
	{	
		ret = SdCmd_Send_Status(sds, &status);                  /* ��ȡ����״̬ */
		if (ret != SD_NO_ERR)
			return ret;
			
//	    if (!SdHal_CheckCard(sds))                              /* ��⿨�Ƿ�γ� */
// 		    return SD_ERR_NO_CARD; 	
			
		cnt++;			
	}while(((status & stmask) == stwait) && (cnt < timeout));

#if SD_UCOSII_EN
	if (cnt >= timeout)
	{                                                           /* ���ٵȴ�����æ */
  		if ((busytype == BUSY_TYPE_RDY_DATA) || (busytype == BUSY_TYPE_PROG))
  			timeout = sds->timeout_write_os;
  		else
  			timeout = sds->timeout_erase_os;   
			
		cnt = 0;
   		do
   		{ 	
   			OSTimeDly(1);                                       /* ����ϵͳ����1 tick */
   			
		    ret = SdCmd_Send_Status(sds, &status);              /* ��ȡ����״̬ */
		    if (ret != SD_NO_ERR)
		    	return ret;
			
//	        if (!SdHal_CheckCard(sds))                          /* ��⿨�Ƿ�γ� */
// 		        return SD_ERR_NO_CARD; 	   			
   			
   			cnt++;
    	}while(((status & stmask) == stwait) && (cnt < timeout));
	}
#endif	
	
	return SD_NO_ERR;
}


/*******************************************************************************************************************
** ��������: MCI_Delay()				
**
** ��������: ��ʱ����	
**
** �䡡  ��: INT16U val:  ��ʱֵ, 1��Լ�൱��5ms
**							  	
** �� �� ��: ��
**
** �� �� ֵ: �� 
********************************************************************************************************************/
void MCI_Delay(INT16U val)
{
#if !SD_UCOSII_EN
    INT16U i,j;
    for (i = 0; i < val; i++)
        for (j = 0; j < 0xC000; j++);
#else
    OSTimeDly(val); 
#endif   
}


/*******************************************************************************************************************
** ��������: MCI_SendCmd()				
**
** ��������: �����������, ������ȡ��Ӧ
**
** �䡡  ��: INT32U CmdIndex    : ��������
**           INT32U Argument    : �������
**           INT32U ExpectResp  : �����õ��Ļظ� 
**           INT32U AllowTimeout: �Ƿ�����ʱ
**	         
** �� �� ��: ��
**
** �� �� ֵ: ��
********************************************************************************************************************/
void MCI_SendCmd(INT32U CmdIndex, INT32U Argument, INT32U ExpectResp, INT32U AllowTimeout)
{
    INT32U CmdData = 0, i;
	INT32U CmdStatus;
		
	while ((CmdStatus = (INT32U)MCIStatus) & MCI_CMD_ACTIVE)    /* �������ڴ����� */
	{
		MCICommand = 0;                                 
		MCIClear = CmdStatus | MCI_CMD_ACTIVE;          /* ������״̬ */
		for(i = 0; i < 0x20 ; i++);						//hena2011.6.14	   ԭʼ0x20
	}	
		
	CmdData |= (CmdIndex & 0x3F);	                    /* ��������ֻ��bit0 ~ bit5 ��Ч */
	if (ExpectResp == EXPECT_NO_RESP)			        /* ����Ӧ���� */
	{
		CmdData &= ~((1 << 6) | (1 << 7));		        /* �������Ӧ�ͳ���Ӧλ */
	}
	else if (ExpectResp == EXPECT_SHORT_RESP)	        /* ϣ���õ�����Ӧ */
	{
		CmdData |= (1 << 6);
	}
	else if (ExpectResp == EXPECT_LONG_RESP)	        /* ϣ���õ�����Ӧ */
	{
		CmdData |= (1 << 6) | (1 << 7);
	}

	if (AllowTimeout)			                        /* �Ƿ�����ʱ�ж� */
		CmdData |= (1 << 8);
	else
		CmdData &= ~(1 << 8);

	CmdData |= (1 << 10);		                        /* ʹ�ܷ������� */
	MCIArgument = Argument;	                            /* �������     */
	MCICommand = CmdData;                               /* ��������� */
}


/*******************************************************************************************************************
** ��������: MCI_GetCmdResp()				
**
** ��������: �ӿ���ȡ��Ӧ, �ú�����MCI_SendCmd()�ɶ�ʹ��
**
** �䡡  ��: INT32U ExpectCmdData: SD/MMC ������
**           INT32U ExpectResp   : ϣ������Ӧ, ȡֵΪ: EXPECT_NO_RESP, EXPECT_SHORT_RESP, EXPECT_LONG_RESP
**           INT8U  resplen      : ��Ӧ����
**           
**	         
** �� �� ��: INT32U *CmdResp     : ��Ӧ����, ����Ϊresplen
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
********************************************************************************************************************/
INT8U MCI_GetCmdResp(INT32U ExpectCmdData, INT32U ExpectResp, INT8U resplen, INT32U *CmdResp)
{
	INT32U LastCmdIndex;
	INT32U CmdRespStatus = 0;

	while (1)
	{                                                       /* ����ѭ���ȴ�������Ӧ */
		CmdRespStatus =  (INT32U)MCIStatus;
		if (CmdRespStatus & MCI_CMD_TIMEOUT)
		{                                                   /* ��ʱ����, ��ʱ���ڹ̶�Ϊ64��MCICLKʱ������ */
		    MCIClear = CmdRespStatus | MCI_CMD_TIMEOUT;
			MCICommand = 0;                                 /* �������Ĵ����������Ĵ��� */
			MCIArgument = 0xFFFFFFFF;
			return SD_ERR_CMD_TIMEOUT;                      /* ������Ӧ���ʱ���� */
		}
		
		if (CmdRespStatus & MCI_CMD_CRC_FAIL)
		{                                                   /* CRCУ��ʧ�ܴ��� */
			MCIClear = CmdRespStatus | MCI_CMD_CRC_FAIL;
			LastCmdIndex =  (INT32U)MCICommand & 0x003F;
			if ((LastCmdIndex == SEND_OP_COND) || (LastCmdIndex == SEND_APP_OP_COND) 
					|| (LastCmdIndex == STOP_TRANSMISSION))
			{                                               /* �������� SEND_OP_COND��STOP_TRANSMISSION��CRCУ�� */
				MCICommand = 0;
				MCIArgument = 0xFFFFFFFF;	 
				break;
			}
			else
				return SD_ERR_CMD_RESPCRC;                  /* ������Ӧ����CRCУ��ʧ�ܴ��� */
		}
		else if (CmdRespStatus & MCI_CMD_RESP_END)
		{
			MCIClear = CmdRespStatus | MCI_CMD_RESP_END;
			break;	                                        /* ��Ӧ�ѳɹ����յ�,�˳�ѭ�� */
		}
		else if ((CmdRespStatus & MCI_CMD_SENT) && (ExpectResp == EXPECT_NO_RESP))
		{                                                   /* ������������Ӧ */
			MCIClear = MCI_CMD_SENT;
			return SD_NO_ERR;                               /* ��������*/
		}		
	}
	
	if ((MCIRespCmd & 0x3F) != ExpectCmdData)
	{
	    /* ��ӦΪ��R1, ����R2��R3, ����������Ӧ�����Ƿ��뷢�͵������ */
		if ((ExpectCmdData != SEND_OP_COND) && (ExpectCmdData != SEND_APP_OP_COND) 
			 && (ExpectCmdData != ALL_SEND_CID) && (ExpectCmdData != SEND_CSD))
		{
			return SD_ERR_CMD_RESP;                         /* ������Ӧ���� */
		}
	}

    /* �����ȡ����Ӧ���ݼ�����Ӧ���� */
	if ((ExpectResp == EXPECT_SHORT_RESP) && (resplen == 1))
	{                                                       
   		*CmdResp =  (INT32U)MCIResponse0;                            /* ��ȡ����Ӧ���� */
	}
	else if ((ExpectResp ==  (INT32U)EXPECT_LONG_RESP) && (resplen == 4))
	{
		*CmdResp =  (INT32U)MCIResponse0;                            /* ��ȡ����Ӧ���� */
		*(CmdResp+1) =  (INT32U)MCIResponse1;
		*(CmdResp+2) =  (INT32U)MCIResponse2;
		*(CmdResp+3) =  (INT32U)MCIResponse3;
	}	
	
	return SD_NO_ERR;	                                    /* ��Ӧ��ȷ. */
}



/**********************************

 ���沿��Ϊ MCI �жϴ�����غ���

***********************************/


/*******************************************************************************************************************
** ��������: MCI_DataEndProcess()				
**
** ��������: MCI ���ݽ���������
********************************************************************************************************************/
uint32 Test_timer[8]={0};//hong
void MCI_DataEndProcess(void)
{
	uint8 err=0;
#if LPC23xx_MCI_DMA_EN
    if (lpc_mci->CurTran == MCI_CUR_SEND)
    {
//        while ( !(DMACRawIntTCStatus & 0x01) );			  //hong
		Test_timer[0] = T0TC;//hong
	//	while ( !(DMACRawIntTCStat & 0x01) ); //hong
		while ( !(DMACRawIntTCStat & 0x01) )
		{
			Test_timer[1] = T0TC;
			if((Test_timer[1] - Test_timer[0])>2)
			{
				err = 1;
				break;
			}
		}
	    DMACIntTCClear |= 0x01;
		lpc_mci->RxTxFlag = SD_NO_ERR;                       /* DMA ���տ����ݳɹ� */
		if(err == 1)
		{
			lpc_mci->RxTxFlag = 0x20;
		}
    }
    else
    {
//        while ( !(DMACRawIntTCStatus & 0x02) );
        while ( !(DMACRawIntTCStat & 0x02) );

		DMACIntTCClear |= 0x02;
		lpc_mci->RxTxFlag = SD_NO_ERR;                       /* DMA ���Ϳ����ݳɹ� */
    }
    MCIDataCtrl = 0;			                            /* ���ݿ��ƼĴ�����0  */
    #if SD_UCOSII_EN
	    OSSemPost(lpc_mci->psds->pSemSdmmcRW);               /* ʹ�ȴ��������� */
    #endif	

#else
   
    MCIClear |= MCI_DATA_BLK_END;
    if ((lpc_mci->SendCnt >= SD_BLOCKSIZE) || 
        (lpc_mci->RecCnt >= SD_BLOCKSIZE))	   
	{                                                       /* ����������/���ɹ� */
	    lpc_mci->RxTxFlag = SD_NO_ERR;
	    #if SD_UCOSII_EN
	    	OSSemPost(lpc_mci->psds->pSemSdmmcRW);
		#endif
	} 

#endif	    
	
}


/*******************************************************************************************************************
** ��������: MCI_TxActiveProcess()				
**
** ��������: MCI �������ݴ�����
********************************************************************************************************************/
void MCI_TxActiveProcess(void)
{
	INT32U DataWord, blockSize,i;
	
	MCIClear |= MCI_TX_ACTIVE;
    if (lpc_mci->SendCnt < SD_BLOCKSIZE )
	{
	    blockSize = 0;
		if (MCIStatus & MCI_TX_FIFO_EMPTY)                  /* FIFOȫ�� */   
	        blockSize = FIFO_SIZE * 4;                      
		else if (MCIStatus & MCI_TX_HALF_EMPTY)             /* FIFO��� */
		   	blockSize = (FIFO_SIZE / 2) * 4; 
		else if (!(MCIStatus & MCI_TX_FIFO_FULL))           /* FIFOȫ�� */
	     	blockSize = 1 * 4; 
		   
		for (i = 0; i < blockSize; i += 4)
    	{                                                   /* ѭ����FIFO��д������ */
    		DataWord = ((INT32U)lpc_mci->pSendBuf[lpc_mci->SendCnt]) + 
    		           (((INT32U)lpc_mci->pSendBuf[lpc_mci->SendCnt + 1]) << 8) +
    		           (((INT32U)lpc_mci->pSendBuf[lpc_mci->SendCnt + 2]) << 16) +
    		           (((INT32U)lpc_mci->pSendBuf[lpc_mci->SendCnt + 3]) << 24);
    		MCIFIFO = DataWord;                             /* ��FIFOд��һ��32λ������ */
    		lpc_mci->SendCnt += 4;
    		if (lpc_mci->SendCnt >= (SD_BLOCKSIZE))          
     			break;	                                    /* �ѷ�����һ�����ݿ� */
    	}     	    
    }
}


/*******************************************************************************************************************
** ��������: MCI_RxActiveProcess()				
**
** ��������: MCI �������ݴ�����
********************************************************************************************************************/
void MCI_RxActiveProcess(void)
{ 
	INT32U DataWord, blockSize, i;
	INT32U Status,loop_cnt;			//20130826 ��дSD�����ӻ��޸�
	loop_cnt=0;						//20130826 ��дSD�����ӻ��޸�
    if (lpc_mci->RecCnt < SD_BLOCKSIZE )
	{
		
	    blockSize = 0;
	    if (!(MCIStatus & MCI_RX_FIFO_FULL))                 /* FIFOȫ�� */          
	   	  	blockSize = FIFO_SIZE * 4; 
	   	else if (MCIStatus & MCI_RX_HALF_FULL)               /* FIFO���� */
	   		blockSize = (FIFO_SIZE/2) * 4; 
	   	else if (MCIStatus & MCI_RX_FIFO_EMPTY)              /* FIFOȫ�� */
	   		blockSize = 1 * 4; 
	   	
	   	for ( i = 0; i < blockSize; i += 4 )
	   	{     
			loop_cnt++;                                               /* ѭ����FIFO�ж�ȡ���� */		//20130826 ��дSD�����ӻ��޸�
		    while (!((Status =  (INT32U)MCIStatus) & MCI_RX_DATA_AVAIL))
		    {
		        if ((Status & MCI_RX_OVERRUN) != 0)          /* �ж��Ƿ���FIFO������� */
		        {
		            MCIDataCtrl = 0;
		            MCIClear    = 0x7FF;
		            MCIMask0    = 0;
		            MCIMask1    = 0;
		            lpc_mci->RxTxFlag = SD_ERR_HARDWARE;
		            return;
		        }
				//20130826 ��дSD�����ӻ��޸�
				if(loop_cnt>=200000)
				{
					MCIDataCtrl = 0;
		            MCIClear    = 0x7FF;
		            MCIMask0    = 0;
		            MCIMask1    = 0;
		            lpc_mci->RxTxFlag = SD_ERR_HARDWARE;
		            return;	
				}
				//����

		    }
		    DataWord =  (INT32U)MCIFIFO;                              /* ��FIFO�ж�ȡһ��32λ������ */
		    lpc_mci->pRecBuf[lpc_mci->RecCnt] = DataWord & 0xFF;
		    lpc_mci->pRecBuf[lpc_mci->RecCnt + 1] = (uint8)(DataWord >> 8) & 0xFF; 
		    lpc_mci->pRecBuf[lpc_mci->RecCnt + 2] = (DataWord >> 16) & 0xFF;
		    lpc_mci->pRecBuf[lpc_mci->RecCnt + 3] = (DataWord >> 24) & 0xFF;
		    lpc_mci->RecCnt += 4;                             /* ���������� */
            if (lpc_mci->RecCnt >=  (INT32U)(SD_BLOCKSIZE))
     			break;		    
	    }
	}
}


/*******************************************************************************************************************
** ��������: MCI_DataInterruptProcess()				
**
** ��������: �ⲿ���Ǵ����д���Ĺؼ�����. ����FIFO�ж�û�д���, ��������Ҫʹ�� TX_ACTIVE �� RX_ACTIVE ������д.
********************************************************************************************************************/
void MCI_DataInterruptProcess( void )
{
	INT32U Status  =  (INT32U)MCIStatus;
 	if (Status &  MCI_DATA_END)                              /* ���ݽ����ж� */
 		MCIClear = Status | MCI_DATA_END;
	
	if (Status & MCI_DATA_BLK_END)                           /* ���ݿ�����ж� */
	{
		MCI_DataEndProcess();
	    return;
	}
//	P3_OUTP_CLR = (1 << 9);	 //��
		
#if !LPC23xx_MCI_DMA_EN	
	if (MCIStatus & MCI_TX_ACTIVE)
        MCI_TxActiveProcess();                               /* ���������жϴ��� */

	if (MCIStatus & MCI_RX_ACTIVE)
        MCI_RxActiveProcess();                               /* ���������жϴ��� */
#endif        
}


/*******************************************************************************************************************
** ��������: MCI_DataErrorProcess()				
**
** ��������: MCI ��������: ��MCI�д������ʱ, �ú���������
********************************************************************************************************************/
void MCI_DataErrorProcess( void )
{
	INT32U Status =  (INT32U)MCIStatus;
	
    if (Status &  (INT32U)MCI_DATA_CRC_FAIL)
		lpc_mci->RxTxFlag = SD_ERR_DATA_CRC16;                /* ��/�����ݿ�ʱ����CRCУ����� */         

	if (Status & MCI_DATA_TIMEOUT)
	{
        if (lpc_mci->CurTran == MCI_CUR_SEND)
            lpc_mci->RxTxFlag = SD_ERR_TIMEOUT_WRITE;         /* �������ݳ�ʱ���� */
        else    
            lpc_mci->RxTxFlag = SD_ERR_TIMEOUT_READ;          /* �������ݳ�ʱ���� */
    }
    	
	if (Status & MCI_TX_UNDERRUN)                            /* ��������ʱFIFO������� */
	    lpc_mci->RxTxFlag = SD_ERR_HARDWARE;

	if (Status & MCI_RX_OVERRUN)                             /* ��������ʱFIFO������� */
	    lpc_mci->RxTxFlag = SD_ERR_HARDWARE;

	if ( Status &  MCI_START_BIT_ERR )                       /* ������ģʽ�����������ź�����û�м�⵽��ʼλ */
	    lpc_mci->RxTxFlag = SD_ERR_DATA_STARTBIT;
	
#if SD_UCOSII_EN
	OSSemPost(lpc_mci->psds->pSemSdmmcRW);                    /* ʹ�ȴ��������� */
#endif
}


/*******************************************************************************************************************
** ��������: MCI_Exception	�� MCI_Handler		
**
** ��������: MCI �жϴ�����
********************************************************************************************************************/

void   MCI_Handler(void)
{	   
	 INT32U Status =  (INT32U)MCIStatus;                    /* ��ȡMCI��ǰ״̬ */

    if (Status & CMD_INT_MASK)
	{                                                       /* �����жϴ��� */
		MCIClear = Status & CMD_INT_MASK;
	}
	else if (MCIStatus & DATA_ERR_INT_MASK)
	{                                                       /* ���ݴ����жϴ��� */
		MCIClear |= DATA_ERR_INT_MASK;
		MCI_DataErrorProcess();                 
	//	MCIClear |= DATA_ERR_INT_MASK;                      /* ������ݴ����ж�λ */
	}											
	else if (MCIStatus & DATA_INT_MASK)
	{                                                       /* �����жϴ��� */
		MCI_DataInterruptProcess();
//		MCIClear |= DATA_INT_MASK;			  //HONG

	}
    else if (MCIStatus & FIFO_INT_MASK)
	{
		;
	}
	MIC_SR |= 13;
}



/*************************
LPC23xx DMA ��ʼ������
**************************/

#if LPC23xx_MCI_DMA_EN

/*******************************************************************************************************************
** ��������: DMA_Init()				
**
** ��������: LPC23xx DMA��������ʼ��
**
** �䡡  ��: INT32U ChannelNum: DMA ������ͨ���� 
**	         INT32U DMAMode:    DMA ģʽ
** �� �� ��: ��
**
** �� �� ֵ: TRUE:   ��ȷ;    FALSE:   ����
********************************************************************************************************************/
INT32U DMA_Init(INT32U ChannelNum, INT32U DMAMode)
{
	/* USB RAM is used for test.
	Please note, Ethernet has its own SRAM, but GPDMA can't access
	that. GPDMA can access USB SRAM and IRAM. Ethernet DMA controller can 
	access both IRAM and Ethernet SRAM. */
	if ( ChannelNum == 0 )
	{
		DMACIntErrClr = 0x01;   		 //���ͨ��0�Ĵ����ж�
		if ( DMAMode == M2M )
		{
			/* Ch0 is set for M2M tranfer from AHB1 to AHB2 SRAM */
			DMACC0SrcAddr = (INT32U)DMA_SRC;
			DMACC0DestAddr = (INT32U)DMA_DST;
			/* Terminal Count Int enable */
			DMACC0Control = (DMA_SIZE & 0x0FFF) | (0x04 << 12) | (0x04 << 15) 
				| (0x02 << 18) | (0x02 << 21) | (1 << 26) | (1 << 27) | 0x80000000;	
		}
		else if ( DMAMode == M2P )
		{
			/* Ch0 set for M2P transfer from mempry to MCI FIFO. */
			DMACC0SrcAddr = (INT32U)DMA_SRC;
			DMACC0DestAddr = DMA_MCIFIFO;
			/* The burst size is set to 8, the size is 8 bit too. */
			/* Terminal Count Int enable */
			DMACC0Control = (DMA_SIZE & 0x0FFF) | (0x01 << 12) | (0x02 << 15)
				| (0x02 << 18) | (0x02 << 21) | (1 << 26) | 0x80000000;
		}
		else if ( DMAMode == P2M )
		{
			/* Ch0 set for P2M transfer from MCI FIFO to memory. */
			DMACC0SrcAddr = DMA_MCIFIFO;
			DMACC0DestAddr = (INT32U)DMA_DST;
			/* The burst size is set to 8, the size is 8 bit too. */
			/* Terminal Count Int enable */
			DMACC0Control = (DMA_SIZE & 0x0FFF) | (0x04 << 12) | (0x04 << 15)
				| (0x02 << 18) | (0x02 << 21) | (1 << 27) | 0x80000000;
		}
		else
		{
			return ( FALSE );
		}
	}
	else if ( ChannelNum == 1 )
	{   
		DMACIntErrClr = 0x02;
		if ( DMAMode == M2M )
		{
			/* Ch1 is set for M2M tranfer */
			DMACC1SrcAddr = (INT32U)DMA_SRC;
			DMACC1DestAddr = (INT32U)DMA_DST;
			/* Terminal Count Int enable */
			DMACC1Control = (DMA_SIZE & 0x0FFF) | (0x04 << 12) | (0x04 << 15) 
				| (0x02 << 18) | (0x02 << 21) | (1 << 26) | (1 << 27) | 0x80000000;	
		}
		else if ( DMAMode == M2P )
		{
			/* Ch1 set for M2P transfer from mempry to MCI FIFO. */
			DMACC1SrcAddr = (INT32U)DMA_SRC;
			DMACC1DestAddr = DMA_MCIFIFO;
			/* The burst size is set to 8, the size is 8 bit too. */
			/* Terminal Count Int enable */
			DMACC1Control = (DMA_SIZE & 0x0FFF) | (0x04 << 12) | (0x04 << 15)
				| (0x02 << 18) | (0x02 << 21) | (1 << 26) | 0x80000000;
		}
		else if ( DMAMode == P2M )
		{
			/* Ch1 set for P2M transfer from MCI_FIFO to memory. */
			DMACC1SrcAddr = DMA_MCIFIFO;
			DMACC1DestAddr = (INT32U)DMA_DST;
			/* The burst size is set to 8, the size is 8 bit too. */
			/* Terminal Count Int enable */
			DMACC1Control = (DMA_SIZE & 0x0FFF) | (0x02 << 12) | (0x04 << 15)
				| (0x02 << 18) | (0x02 << 21) | (1 << 27) | 0x80000000;	  	   //���ô���������ݻ����
		}
		else
		{
			return ( FALSE );
		}
	}
	else
	{
		return ( FALSE );
	}
	
//	DMACConfiguration = 0x01;	//hong	/* Enable DMA channels, little endian */
	DMACConfig = 0x01;	/* Enable DMA channels, little endian */

//	while ( !(DMACConfiguration & 0x01) );   //hong
	while ( !(DMACConfig & 0x01) );   

	 
    return (TRUE);
}

#endif



#endif




