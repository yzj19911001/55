/****************************************Copyright (c)****************************************************
**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
**                                     
**                               http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           target.c
** Last modified Date:  2004-09-17
** Last Version:        1.0
** Descriptions:        header file of the specific codes for LPC2200 target boards
**                      Every project should include a copy of this file, user may modify it as needed
**--------------------------------------------------------------------------------------------------------
** Created by:          Chenmingji
** Created date:        2004-02-02
** Version:             1.0
** Descriptions:        The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         LinEnqiang
** Modified date:       2008-12-12
** Version:             1.01
** Descriptions:        Renewed the template, added more compiler supports
**
*********************************************************************************************************/
#define IN_TARGET
#include "config.h"
#include "IRQ.h"

#define WAITWEN0        0x07                                            /* ����EMCStaticWaitWen1       */
#define WAITOEN0        0x07                                            /* ����EMCStaticWaitOen1       */ 
#define WAITRD0         0x1f                                            /* ����EMCStaticWaitRd1        */
#define WAITPAGE0       0x00                                            /* ����EMCStaticWaitPage1      */
#define WAITWR0         0x1f                                            /* ����EMCStaticWaitWr1        */
#define WAITTURN0       0x0f                                            /* ����EMCStaticWaitTurn1      */

#define WAITWEN1        0x03                                            /* ����EMCStaticWaitWen1       */
#define WAITOEN1        0x03                                            /* ����EMCStaticWaitOen1       */ 
#define WAITRD1         0x02                                            /* ����EMCStaticWaitRd1        */
#define WAITPAGE1       0x00                                            /* ����EMCStaticWaitPage1      */
#define WAITWR1         0x04                                            /* ����EMCStaticWaitWr1        */
#define WAITTURN1       0x0f                                            /* ����EMCStaticWaitTurn1      */

#define WAITWEN2        0x0F                                            /* ����EMCStaticWaitWen2       */
#define WAITOEN2        0x0F                                            /* ����EMCStaticWaitOen2       */ 
#define WAITRD2         0x0F                                            /* ����EMCStaticWaitRd2        */
#define WAITPAGE2       0x1F                                            /* ����EMCStaticWaitPage2      */
#define WAITWR2         0x0F                                            /* ����EMCStaticWaitWr2        */
#define WAITTURN2       0x0F                                            /* ����EMCStaticWaitTurn2      */

#define WAITWEN3        0x0F                                            /* ����EMCStaticWaitWen3       */
#define WAITOEN3        0x0F                                            /* ����EMCStaticWaitOen3       */ 
#define WAITRD3         0x0F                                            /* ����EMCStaticWaitRd3        */
#define WAITPAGE3       0x1F                                            /* ����EMCStaticWaitPage3      */
#define WAITWR3         0x0F                                            /* ����EMCStaticWaitWr3        */
#define WAITTURN3       0x0F                                            /* ����EMCStaticWaitTurn3      */


#define BCFG_08DEF      0x00000000                                      /*  8Bit Bus                  */
#define BCFG_16DEF      0x00000001                                      /* 16Bit Bus                  */
#define BCFG_32DEF      0x00000002                                      /* 32Bit Bus                  */

    /*
     *                  |  ҳģʽ    |  Ƭѡ����  |�ֽڶ�λ״̬|  �ӳ��ȴ�  |  д������  |   д����   |
     *                  |    PM      |    PC      |    PB      |     EW     |     B      |     P      |
     *                  |0:����1:ʹ��| 0:�� 1:��  |            |0:����1:ʹ��|0:����1:ʹ��|0:����1:ʹ��|
     */
#define BCFG0           ( (0x00<<03) | (0x00<<06) | (0x01<<07) | (0x00<<8)  | (0x00<<19) | (0x00<<20) )   
#define BCFG1           ( (0x00<<03) | (0x00<<06) | (0x01<<07) | (0x01<<8)  | (0x00<<19) | (0x00<<20) )
#define BCFG2           ( (0x00<<03) | (0x00<<06) | (0x01<<07) | (0x00<<8)  | (0x00<<19) | (0x00<<20) )
#define BCFG3           ( (0x00<<03) | (0x00<<06) | (0x01<<07) | (0x00<<8)  | (0x00<<19) | (0x00<<20) )

#define STATICCFG0  	( BCFG_16DEF | BCFG0 ) 
#define STATICCFG1  	( BCFG_16DEF | BCFG1 ) 
#define STATICCFG2      ( BCFG_16DEF | BCFG2 )
#define STATICCFG3      ( BCFG_16DEF | BCFG3 )

/*********************************************************************************************************
** Function name:       FIQ_Exception
** Descriptions:        FIQ�жϷ������
** input parameters:    None
** output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void FIQ_Exception (void)
{
                                                                        /*  1.����û�FIQ�жϷ������   */
    while(1) {                                                          /*  ��һ���滻Ϊ�Լ��Ĵ���      */
    	;
    }	
}

/*********************************************************************************************************
** Function name:           TargetInit
**
** Descriptions:            Initialize the target board; it is called in a necessary place, change it as
**                          needed
**
** input parameters:        None
** output parameters:       None
** Returned value:          None
**
** Created by:              Chenmingji
** Created Date:            2004/02/02
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
extern void vicInitial(void);
void TargetInit (void)
{   
    vicInitial();
	IRQEnable();
	
	/****************************
	* �޸ĸ��жϳ�ʼ���Ĳ������޸�
	* InitAllIRQ��������Ҫֱ���޸�
	* �ú���
	*
	******************************/
    
    //FIQEnable();
}

/*********************************************************************************************************
** Function name:           TargetResetInit
** Descriptions:            Initialize the target
** input parameters:        None
** output parameters:       None
** Returned value:          None
** Created by:              Chenmingji
** Created Date:            2004/02/02
**--------------------------------------------------------------------------------------------------------
** Modified by:             LinEnqiang
** Modified date:           2008-12-12
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
extern void Reset(void);
void TargetResetInit (void)
{
    
#ifndef __IN_RAM
    uint8 i;
    uint32 *cp1,*cp2;
    cp1=(uint32*)Reset;
	#ifdef __OUT_CHIP
		cp1 ++;	 		//��䲻��ɾ�������޷���������
	#endif
    cp2=(uint32*)0x08000000; 
	//cp1++;				//���ԣ����Ƿ���������
    for(i=0;i<16;i++)
	{  
		while(*cp2 != *cp1)
			*cp2 = *cp1;
		cp2 ++;
		cp1 ++;
	}   
#endif  
    EMCStaticConfig1    = STATICCFG1;
    EMCStaticWaitWen1	= WAITWEN1;
    EMCStaticWaitOen1	= WAITOEN1;
    EMCStaticWaitRd1	= WAITRD1;
    EMCStaticWaitPage1	= WAITPAGE1;      
    EMCStaticWaitWr1    = WAITWR1;
    EMCStaticWaitTurn1  = WAITTURN1;
	EMCStaticExtendedWait = 0x02;

	BOOT_MAP = 0x01;
    
    /* 
     * AHB ����ʱ������
     */
    EMCAHBControl0     = 0x00000001;
    EMCAHBTimeout0     = 0x00000190;
    EMCAHBControl2     = 0x00000001;
    EMCAHBTimeout2     = 0x00000190;
    EMCAHBControl3     = 0x00000001;
    EMCAHBTimeout3     = 0x00000190;
    EMCAHBControl4     = 0x00000001;
    EMCAHBTimeout4     = 0x00000190; 
    
    /* 
     * ʹ��CPU������ֱ������ģʽ����ʹ��HCLK_PLL
     */  
    PWR_CTRL = 0x0012;                                                                              

#if USE_PLL397 == 1
    PLL397_CTRL	= 0x00;
    OSC_CTRL	= 0x00;
    SYSCLK_CTRL	= (0x50 << 2) |
                  (0x01 << 1) ;
    while (0 == (PLL397_CTRL & 0x01) ) {
        if(0 != (PLL397_CTRL & (0x01 << 10))) {
            break;
        }
    }
    
    OSC_CTRL = 0x03;           
#else
    SYSCLK_CTRL = (0x50 << 2) |
                  (0x00 << 0) ;
    OSC_CTRL = 0x00;
#endif
    
#if (Fcco / Fcclk) == 1                                                 
    HCLKPLL_CTRL = ((1 << 16)   |                                       /* PLL �ڲ���ģʽ               */
                    (0 << 15)   |                                       /* ����PLL ����                 */
                    (1 << 14)   |                                       /* ��·���÷�Ƶ��               */
                    (1 << 13)   |                                       /* ���÷�Ƶ���������           */
                    (0 << 11)   |                                       /* P(��Ч)                      */        
                    (0 << 9)    |                                       /* N                            */
                    (((Fcclk / Fosc) - 1) << 1));                       /* M                            */	  
#endif
#if (Fcco / Fcclk) == 2
    HCLKPLL_CTRL = ((1 << 16)   |                                       /* PLL �ڲ���ģʽ               */
                    (0 << 15)   |                                       /* ����PLL ����                 */
                    (0 << 14)   |                                       /* ʹ�ú��÷�Ƶ��               */
                    (1 << 13)   |                                       /* ���÷�Ƶ���������           */
                    (0 << 11)   |                                       /* P(��Ч)                      */        
                    (0 << 9)    |                                       /* N                            */
                    (((Fcclk / Fosc) - 1) << 1));                       /* M                            */	 
#endif
#if (Fcco / Fcclk) == 4
    HCLKPLL_CTRL = ((1 << 16)   |                                       /* PLL �ڲ���ģʽ               */
                    (0 << 15)   |                                       /* ����PLL ����                 */
                    (0 << 14)   |                                       /* ʹ�ú��÷�Ƶ��               */
                    (1 << 13)   |                                       /* ���÷�Ƶ���������           */
                    (1 << 11)   |                                       /* P(��Ч)                      */        
                    (0 << 9)    |                                       /* N                            */
                    (((Fcclk / Fosc) - 1) << 1));                       /* M                            */	
#endif
#if (Fcco / Fcclk) == 8
    HCLKPLL_CTRL = ((1 << 16)   |                                       /* PLL �ڲ���ģʽ               */
                    (0 << 15)   |                                       /* ����PLL ����                 */
                    (0 << 14)   |                                       /* ʹ�ú��÷�Ƶ��               */
                    (1 << 13)   |                                       /* ���÷�Ƶ���������           */
                    (2 << 11)   |                                       /* P(��Ч)                      */        
                    (0 << 9)    |                                       /* N                            */
                    (((Fcclk / Fosc) - 1) << 1));                       /* M                            */	
#endif
#if (Fcco / Fcclk) == 16
    HCLKPLL_CTRL = ((1 << 16)   |                                       /* PLL �ڲ���ģʽ               */
                    (0 << 15)   |                                       /* ����PLL ����                 */
                    (0 << 14)   |                                       /* ʹ�ú��÷�Ƶ��               */
                    (1 << 13)   |                                       /* ���÷�Ƶ���������           */
                    (3 << 11)   |                                       /* P(��Ч)                      */        
                    (0 << 9)    |                                       /* N                            */
                    (((Fcclk / Fosc) - 1) << 1));                       /* M                            */	
#endif

    while (!(HCLKPLL_CTRL & 0x01 ));
#if (Fdclk != 0)
    HCLKDIV_CTRL = (((Fcclk / Fdclk) & 0x03) << 7)      | 
                   ((((Fcclk / Fpclk) - 1) & 0x1F) << 2)| 
                   (((Fcclk / Fhclk / 2) & 0x03) << 0);
#else
            
    HCLKDIV_CTRL = ((((Fcclk / Fpclk) - 1) & 0x1F) << 2)| 
                   (((Fcclk / Fhclk / 2) & 0x03) << 0);
#endif
    PWR_CTRL |= ( 1 << 2 );                                             /* 
                                                                         * ����RUN ģʽ��ARM, HCLK ��PLL
                                                                         * ����ṩʱ��Դ
                                                                         */   
    return;
}

/*********************************************************************************************************
  ����ΪһЩ��ϵͳ��صĿ⺯����ʵ��
  ����������ads�Ĳο���������⺯���ֲ�
  �û����Ը����Լ���Ҫ���޸�
*********************************************************************************************************/

#pragma import(__use_no_semihosting_swi)
#pragma import(__use_two_region_memory)

void _ttywrch (int ch)
{
    ch = ch;
}

/*********************************************************************************************************
  End File
*********************************************************************************************************/
