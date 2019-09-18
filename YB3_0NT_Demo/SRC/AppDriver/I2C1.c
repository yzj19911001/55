/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           I2CINT.c
** Last modified Date:  2009-02-03
** Last Version:        1.00
** Descriptions:        for lpc3200
**--------------------------------------------------------------------------------------------------------
** Created by:          Lilingyu
** Created date:        2009-02-03
** Version:             1.00
** Descriptions:        The original version
*********************************************************************************************************/
#define	__I2C1_C
#include "I2C1.h"

#define I2C_CLK_100K 1
#define I2C_CLK_400K 4

#define  _BIT(x)     (1 << (x))


/**********************************************************************
* i2c_txrx register definitions
**********************************************************************/
#define I2C_START_FLAG    _BIT(8)                                       /* generate a START before      */
                                                                        /* this B                       */
#define I2C_STOP_FLAG     _BIT(9)                                       /* generate a STOP after this B */

/*********************************************************************************************************
* i2c_stat register definitions
*********************************************************************************************************/
#define I2C_TDI     _BIT(0)                                             /* Transaction Done Interrupt   */
#define I2C_AFI     _BIT(1)                                             /* Arbitration Failure Interrupt*/
#define I2C_NAI     _BIT(2)                                             /* No Acknowledge Interrupt     */
#define I2C_DRMI    _BIT(3)                                             /* Master Data Request Interrupt*/
#define I2C_DRSI    _BIT(4)                                             /* Slave Data Request Interrupt */
#define I2C_ACTIVE  _BIT(5)                                             /* Busy bus indicator           */
#define I2C_SCL     _BIT(6)                                             /* The current SCL signal value */
#define I2C_SDA     _BIT(7)                                             /* The current SDA signal value */
#define I2C_RFF     _BIT(8)                                             /* Receive FIFO Full            */
#define I2C_RFE     _BIT(9)                                             /* Receive FIFO Empty           */
#define I2C_TFF     _BIT(10)                                            /* Transmit FIFO Full           */
#define I2C_TFE     _BIT(11)                                            /* Transmit FIFO Empty          */
#define I2C_TFFS    _BIT(12)                                            /* Slave Transmit FIFO Full     */
#define I2C_TFES    _BIT(13)                                            /* Slave Transmit FIFO Empty    */

/*********************************************************************************************************
* i2c_ctrl register definitions
*********************************************************************************************************/
#define I2C_TDIE    _BIT(0)                                             /* Transaction Done Int Enable  */
#define I2C_AFIE    _BIT(1)                                             /* Arbitration Failure Int Ena  */
#define I2C_NAIE    _BIT(2)                                             /* No Acknowledge Int Enable    */
#define I2C_DRMIE   _BIT(3)                                             /* Master Data Request Int Ena  */
#define I2C_DRSIE   _BIT(4)                                             /* Slave Data Request Int Ena   */
#define I2C_RFFIE   _BIT(5)                                             /* Receive FIFO Full Int Ena    */
#define I2C_RFDAIE  _BIT(6)                                             /* Rcv Data Available Int Ena   */
#define I2C_TFFIE   _BIT(7)                                             /* Trnsmt FIFO Not Full Int Ena */
#define I2C_RESET   _BIT(8)                                             /* Soft Reset                   */
#define I2C_SEVEN   _BIT(9)                                             /* Seven-bit slave address      */
#define I2C_TFFSIE  _BIT(10)                                            /* Slave Trnsmt FIFO Not Full IE*/

/*********************************************************************************************************
* ִ�н��
*********************************************************************************************************/
#define   I2C_BEGIN              0x01
#define   I2C_DONE               0x02
#define   I2C_NAK                0x03
#define   I2C_ARF                0x04
#define   I2C_ERR                0xFF

/*********************************************************************************************************
* I2C������
*********************************************************************************************************/
#define		I2C_CMD_WR_EX_ADDR	0x01
#define		I2C_CMD_WR_DATA		0x02
#define		I2C_CMD_RD_DATA		0x03

#define		ONE_BYTE_SUBA		1
#define		TWO_BYTE_SUBA		2
#define		X_ADD_8_SUBA		3

//extern	uint8	m_u8ACKFlag;

static	void	I2C1_Excetion(void);		//�жϴ���

static	uint8	Wait_I2c_End(uint32  dly);

/*********************************************************************************************************
�������ں�I2C�жϴ�����Ϣ��ȫ�ֱ��� 
*********************************************************************************************************/
volatile uint16    I2C1_sla;                                             /* I2C������ַ                  */
volatile uint8     I2C1_Status;                                          /* I2C״̬��־                  */
volatile uint8     *I2C1_recvbuf;                                        /* �������ݴ洢��ַ             */
volatile uint8     *I2C1_sendbuf;                                        /* д�����ݴ洢��ַ             */
volatile uint32 	I2C1_recvnum;                                        /* �������ݸ���                 */
volatile uint32 	I2C1_sendnum;                                        /* д�����ݸ���                 */
volatile uint32 	I2C1_recvNullnum;                                    /* ��Ҫ���������ݸ���           */

/****************************************************************************************************************
* �������ƣ�I2CInit()
* �������ܣ�I2C ��ʼ��
* ��ڲ�����i2c_clk        I2Cʱ�ӿ�ѡ����:I2C_CLK_100K,I2C_CLK_400K
* ���ز�����TURE | FALSE
*****************************************************************************************************************/
uint8 I2C1Init(uint8 i2c_clk)
{
    I2CCLK_CTRL = 0x03;                                                 /* ʹ��I2C                      */

    I2C1_CTRL = (1 << 8);                                               /* Rset I2C                     */


    while ((I2C1_CTRL & (1 << 8)) != 0);								/*��λ���λΪ��λֵΪ0*/
    
    if (i2c_clk  ==  I2C_CLK_100K)
	{                                     /* ����I2Cʱ��Ϊ100K            */
      //  I2C1_CLKHI    = 520;                                            /* HCLK = 104M                  */
       // I2C1_CLKLO    = 520;
		      I2C1_CLKHI    = 655;                                            /* HCLK = 104M                  */
          I2C1_CLKLO    = 655;
    }
	else 
	   if (i2c_clk  ==  I2C_CLK_400K)
	   {                               /* ����I2Cʱ��Ϊ400K            */
           I2C1_CLKHI    = 96;
           I2C1_CLKLO    = 166;
       }
	   else
	   {
        return FALSE;
       }
//	return (TRUE);
    return((uint8)sic1IrqFuncSet(19, 2, (unsigned int)I2C1_Excetion));   /* I2C1_INT:19 I2C2_INT:18      */
}


/****************************************************************************************************************
* �������ƣ�Wait_I2c_End
* �������ܣ������ʱ���ṩ��I2C���ߵȴ���ʱʹ��
* ��ڲ�����dly        ��ʱ������ֵԽ����ʱԽ��
* ���ڲ�����TURE | FALSE
*****************************************************************************************************************/
uint8  Wait_I2c_End(uint32  dly)
{
    uint32  i;
    if (I2C1_Status == I2C_DONE)
    return (TRUE);
    for (; dly > 0; dly--)
        for (i = 0; i < 50000; i++){
            if (I2C1_Status == I2C_DONE) return (TRUE);
        }
    return (FALSE);
}

/**********************************************************************************************************
** �������� ��I2C_ReadNByte
** �������� �������ӵ�ַ���������ַ��ʼ��ȡN�ֽ�����
** ��ڲ��� ��sla            �����ӵ�ַ
**            suba_type      �ӵ�ַ�ṹ    1�����ֽڵ�ַ    2��˫�ֽڵ�ַ    n�����ֽڵ�ַ
**            suba           �����ڲ��ӵ�ַ
**            *s             ��Ҫд������ݵ�ָ��
**            num            ��Ҫд������ݵĸ���
** ���ڲ��� ��TURE | FALSE
**********************************************************************************************************/
uint8 I2C1_ReadNByte (uint16 sla, uint8 suba_type, uint32 suba, uint8 *tp, uint32 num)
{
    uint8 err;
    uint32	l_u32Suba;
	uint32	l_u32Temp;
    err = TRUE;
	if (suba_type == 2)
		l_u32Suba	= (suba&0xffff0000) | ((suba&0x0000ff00)>>8) | ((suba & 0x000000ff)<<8);
	else
		l_u32Suba	= suba;
	
	l_u32Temp = 0;
    while (I2C1_STAT & I2C_ACTIVE)                                     /* I2C�����Ƿ���У�            */
   	{
		l_u32Temp++;
		if( l_u32Temp >= 0x00ffffff )
			break;	
	}
    I2C1_STAT = 0x3;                                                    /* ��AFI��TDI��־               */
    I2C1_CTRL = (1 << 8);                                               /* Rset i2c                     */
    while ((I2C1_CTRL & (1 << 8)) != 0);                                /* �ȴ���λ                     */
    
    if (num > 0)                                                        /* �ж�num�����ĺϷ���          */
    {
        I2C1_Status = I2C_BEGIN;                                         /* I2C״̬����                  */
        if (suba_type)
		{                                                 /* �Ƿ�Ϊ���ӵ�ַ������         */
            /*
             * �����ӵ�ַ 
             */
            I2C1_sendnum = suba_type;                                    /* �ӵ�ַ�ֽڸ���               */
            I2C1_sendbuf = (uint8*) & l_u32Suba;
            I2C1_TX = (sla & 0xfe) | I2C_START_FLAG;                    /* ����������ַ����I2C����      */
            I2C1_CTRL = (I2C_TFFIE |                                    /* ����FIFOδ���ж�             */
                         I2C_NAIE  |                                    /* ������Ӧ���ж�               */
                         I2C_AFIE  |                                    /* �ٲ�ʧ���ж�                 */
                         I2C_TDIE );                                    /* ��������ж�                 */
			
			err  &= Wait_I2c_End(10);
            /*
             * ������ 
             */
            I2C1_recvNullnum = num;
            I2C1_recvnum = 0;
            I2C1_recvbuf = tp;
            I2C1_Status = I2C_BEGIN;
            while (I2C1_STAT & I2C_TFF);                                /* ����FIFO�Ƿ�δ��             */
            I2C1_TX = ((sla|0x01) | I2C_START_FLAG);                    /* �ٴ�����I2C����              */
            I2C1_CTRL |= (I2C_RFDAIE|                                   /* ʹ��I2C��������Ч�ж�        */
            I2C_TFFIE);                                                 /* ʹ�ܷ���FIFOδ���ж�         */
        }
		else
		{                                                          /* ���ӵ�ַ��                   */
            I2C1_recvNullnum = num;
            I2C1_recvnum = 0;
            I2C1_recvbuf = tp;
            while (I2C1_STAT & I2C_TFF);                                /* ����FIFO�Ƿ�δ��             */
            I2C_TX = sla|I2C_START_FLAG;
            I2C1_CTRL = (I2C_RFDAIE |                                   /* �������ݿ����ж�             */
            I2C_TFFIE  |                                                /* ����FIFOδ���ж�             */
            I2C_NAIE   |                                                /* ������Ӧ���ж�               */
            I2C_AFIE   |                                                /* �ٲ�ʧ���ж�                 */
            I2C_TDIE );                                                 /* ��������ж�                 */
        }
        while ((I2C1_STAT & 0x01)== 0);                                /* �ȴ�I2C�������              */
        return err;
    }
    return (FALSE);
}

/*********************************************************************************************************
** �������� ��I2C_WriteNByte
** �������� �������ӵ�ַ����д��N�ֽ�����
** ��ڲ��� ��sla         �����ӵ�ַ
**            suba_type   �ӵ�ַ�ṹ    1�����ֽڵ�ַ    3��8+X�ṹ    2��˫�ֽڵ�ַ
**            suba        �����ڲ������ַ
**            *s          ��Ҫд������ݵ�ָ��
**            num         ��Ҫд������ݵĸ���
** ���ڲ��� ��TRUE        �����ɹ�
**            FALSE       ����ʧ��
*********************************************************************************************************/
uint8 I2C1_WriteNByte(uint16 sla, uint8 suba_type, uint32 suba, uint8 *tp, uint32 num)
{
    uint8 err;
	uint32	l_u32Suba;
	uint32	l_u32Temp;

    err = TRUE;
    if (suba_type == 2)
		l_u32Suba	= (suba&0xffff0000) | ((suba&0x0000ff00)>>8) | ((suba & 0x000000ff)<<8);
	else
		l_u32Suba	= suba;

	l_u32Temp = 0;
    while (I2C1_STAT & I2C_ACTIVE)                                     /* I2C�����Ƿ���У�            */
	{
		l_u32Temp++;
		if( l_u32Temp >= 0x00ffffff )
			break;	
	}
    I2C1_STAT = 0x3;                                                    /* ��AFI��TDI��־               */
    I2C1_CTRL = (1 << 8);                                               /* Rset i2c                     */
    while ((I2C1_CTRL & (1 << 8)) != 0);                                /* �ȴ���λ                     */
    
    if (num > 0)                                                        /* �ж�num�����ĺϷ���          */
    {
        I2C1_Status = I2C_BEGIN;                                         /* ����I2C״̬                  */
        if (suba_type)
        {                                                 /* �Ƿ�Ϊ���ӵ�ַд����         */
            /*
             * ���������ӵ�ַ 
             */
            I2C1_sendnum  = suba_type;
            I2C1_sendbuf  = (uint8*) &l_u32Suba;
            I2C1_TX = (sla & 0xfe) | I2C_START_FLAG;                    /* ����������ַ����I2C����      */
            I2C1_CTRL = (I2C_TFFIE |                                    /* ����FIFOδ���ж�             */
                         I2C_NAIE  |                                    /* ������Ӧ���ж�               */
                         I2C_AFIE  |                                    /* �ٲ�ʧ���ж�                 */
                         I2C_TDIE );                                    /* ��������ж�                 */
            err  &= Wait_I2c_End(10);
            /*
             * �������� 
             */
            I2C1_sendbuf  = tp;
            I2C1_sendnum  = num-1;
            I2C1_Status = I2C_BEGIN;
            if (num  ==  1){
                while (I2C1_STAT & I2C_TFF);                            /* ����FIFO�Ƿ���               */
                I2C1_TX = ((*I2C1_sendbuf)|I2C_STOP_FLAG);
            }
            else{
                I2C1_CTRL |= I2C_TFFIE;                                 /* ʹ�ܷ���FIFOδ���ж�         */
                                                                        /* �ٴη�������                 */
                err  &= Wait_I2c_End(20);
                while (I2C1_STAT & I2C_TFF);                            /* ����FIFO�Ƿ�δ��             */
                I2C1_TX = ((*I2C1_sendbuf)|I2C_STOP_FLAG);               /* �������һ��������ֹͣλ     */
            }
        }
        else{                                                           /* ���ӵ�ַд                   */
            I2C1_CTRL = (I2C_TFFIE |                                    /* ����FIFOδ���ж�             */
                         I2C_NAIE  |                                    /* ������Ӧ���ж�               */
                         I2C_AFIE  |                                    /* �ٲ�ʧ���ж�                 */
                         I2C_TDIE );                                    /* ��������ж�                 */
            I2C1_sendnum = num-1;
            I2C1_sendbuf = tp;
            I2C1_TX = sla|I2C_START_FLAG;                               /* ����������ַ����I2C����      */
            err  &= Wait_I2c_End(20);
            while (I2C1_STAT & I2C_TFF);                                /* ����FIFO�Ƿ�δ��             */
            I2C1_TX = ((*I2C1_sendbuf)|I2C_STOP_FLAG);                   /* �������һ��������ֹͣλ     */
        }
        return err;
    }
    return FALSE;
}

/**********************************************************************************************************
** �������� ��I2C_Excetion
** �������� ��Ӳ��I2C�жϷ������
** ��ڲ��� ����
** ���ڲ��� ����
** ˵    �� ����
**********************************************************************************************************/
void I2C1_Excetion(void)
{
    uint32 sts;
    uint32 bsend = 0;
    
    while ((I2C1_STAT & (1 << 9))  ==  0)
	{                                                                   /* ����FIFO��Ϊ��               */
        *I2C1_recvbuf++ =I2C1_RX;                                        /* ��ȡ����                     */
        I2C1_recvnum++;
    }
    while ((bsend<I2C1_sendnum) && ((I2C1_STAT & (1 << 10))== 0))
	{      /* ����FIFOδ�����������ݵȴ�   */
                                                                        /* ����                         */
        I2C1_TX = (uint8)*I2C1_sendbuf++;
//        I2C1_TX = (uint32)*I2C1_sendbuf++;
        bsend++;
    }
    I2C1_sendnum -= bsend;
    while ((I2C1_recvNullnum > 0) && ((I2C1_STAT & (1 << 10))== 0))
	{                                                                   /* ����FIFOδ��������������Ҫ   */
                                                                        /* ����                         */
        if (I2C1_recvNullnum  ==  1){                                    /* �Ƿ�Ϊ���һ����Ҫ���յ����� */
            I2C1_TX = (0xff|I2C_STOP_FLAG);                             /* ����ֹͣλ                   */
        }else{
            I2C1_TX = 0xff;
        }
        I2C1_recvNullnum--;
    }
    if ((I2C1_sendnum + I2C1_recvNullnum)== 0)
	{                                                                  /* ���û��������Ҫ����         */
        I2C1_CTRL  &= ~I2C_TFFIE;                                      /* ��ֹ����FIFOδ���ж�         */
        I2C1_Status = I2C_DONE;
    }
    sts = I2C1_STAT;                                                   /* ����I2C״̬                  */
    if ((sts & (1 << 5)) != 0)
	{
		sts = sts;
    }
	else 
		if ((sts & (1 << 2))  !=  0)									   /* ��Ӧ��                       */
		{                                                              
			I2C1_Status = I2C_NAK;
			I2C1_CTRL = 0x00; 
//			m_u8ACKFlag	= 0;                                             /* ��ֹ�ж�                     */
		}
		else
		{
//			m_u8ACKFlag	= 1;		//��Ӧ��	 
			if ((sts & (1 << 0)) != 0)
			{                                                            /* �ѷ���ֹͣλ                 */
				I2C1_Status = I2C_DONE;
				while ((I2C1_STAT & (1 << 9))== 0)
				{                                                         /* ����FIFOδ��                 */
					*I2C1_recvbuf++ = I2C_RX;
					I2C1_recvnum++;
				}
				I2C1_CTRL = 0;                                            /* ��ֹ�ж�                     */
			}
			else
			{
				I2C1_Status = I2C_ERR;
				I2C1_CTRL = 0;
			}
		}
}
