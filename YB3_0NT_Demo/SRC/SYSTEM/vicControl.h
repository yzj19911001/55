/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**                                      
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               vicControl.h
** Latest modified Date:    2008-12-12
** Latest Version:          1.0
** Descriptions:            VIC���ƴ���
**
**--------------------------------------------------------------------------------------------------------
** Created by:              LinEnqiang
** Created date:            2008-12-12
** Version:                 1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef _VIC_CONTROL_H_
#define _VIC_CONTROL_H_

#ifdef __cplusplus
    extern "C" {
#endif
                                                                  /*  __cplusplus                 */
/*********************************************************************************************************
  �жϴ�������ֵ �궨��
*********************************************************************************************************/
#define INT_LEVEL_TYPE_LOW      0x02                                    /*  �͵�ƽ�����ж�ģʽ          */
#define INT_LEVEL_TYPE_HIGH     0x03                                    /*  �ߵ�ƽ�����ж�ģʽ          */

#define INT_EDGE_TYPE_DOWN      0x00                                    /*  �½��ش����ж�ģʽ          */
#define INT_EDGE_TYPE_UP        0x01                                    /*  �����ش����ж�ģʽ          */


/*********************************************************************************************************
  IRQ��FIQ �ж� ȫ�ֿ��� ����
*********************************************************************************************************/

/*********************************************************************************************************
** Function name:           swiHandle1
** Descriptions:            SWI��������
** input parameters:        iHandle: �������ֹ���                          
** output parameters:       ���ݹ��ܾ���
** Returned value:          ���ݹ��ܾ���
*********************************************************************************************************/
__swi(0x00) void swiHandle1(int iHandle);

/*********************************************************************************************************
** Function name:           IRQDisable
** Descriptions:            �ر�ȫ�� IRQ �ж�
** input parameters:        none
** output parameters:       none
** Returned value:          none                          
*********************************************************************************************************/
__inline unsigned int IRQDisable() 
{
	swiHandle1(1);		   
	return 0;
}
/*********************************************************************************************************
** Function name:           IRQEnable
** Descriptions:            ��ȫ�� IRQ �ж�
** input parameters:        none
** output parameters:       none
** Returned value:          none                          
*********************************************************************************************************/
__inline unsigned int IRQEnable() 
{
	swiHandle1(12); 
	return 0;
}

/*********************************************************************************************************
  ͨ������ IRQ��FIQ �жϹ���
*********************************************************************************************************/
/*********************************************************************************************************
** Function name:           swiHandle
** Descriptions:            SWI��������
** input parameters:        iHandle: �������ֹ���
**                          ����:    ���ݹ��ܾ���
** output parameters:       ���ݹ��ܾ���
** Returned value:          ���ݹ��ܾ���
*********************************************************************************************************/
__swi(0x00) unsigned int swiHandle (int iHandle, unsigned int, unsigned int, unsigned int);

/*********************************************************************************************************
** Function name:           micIrqFuncSet
** Descriptions:            ������ѡ������жϴ������͡��жϷ�������ַ����ʹ���ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
**                          uiType:     �жϴ������� -- 0:�½��� 1:������ 2:�͵�ƽ 3:�ߵ�ƽ
**                          uiFuncAddr: �жϷ�������ַ
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int micIrqFuncSet (unsigned int uiChannel,
                                     unsigned int uiType,
                                     unsigned int uiFuncAddr)
{
    return swiHandle(0x100, uiChannel, uiType, uiFuncAddr);
}

/*********************************************************************************************************
** Function name:           micIrqFuncClr
** Descriptions:            �����ѡ�����IRQ��Դ
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int micIrqFuncClr (unsigned int uiChannel)
{
    return swiHandle(0x101, uiChannel ,0 ,0);
}

/*********************************************************************************************************
** Function name:           micIrqEnable
** Descriptions:            ʹ����Ӧ������ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int micIrqEnable (unsigned int uiChannel)
{
    return swiHandle(0x102, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           micIrqDisable
** Descriptions:            ��ֹ��Ӧ������ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int micIrqDisable (unsigned int uiChannel)
{
    return swiHandle(0x103, uiChannel, 0, 0);
}
/*********************************************************************************************************
** Function name:           micFiqSet
** Descriptions:            ���ò�ʹ����ѡ�ж�ͨ����ΪFIQ�ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
**                          uiType:     �жϴ������� -- 0:�½��� 1:������ 2:�͵�ƽ 3:�ߵ�ƽ
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int micFiqSet (unsigned int uiChannel, unsigned int uiType)
{
    return swiHandle(0x104, uiChannel, uiType, 0);
}

/*********************************************************************************************************
** Function name:           micFiqClr
** Descriptions:            �����ѡ�ж�ͨ���ŵ�FIQ�ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int micFiqClr (unsigned int uiChannel)
{
    return swiHandle(0x105, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           sic1IrqFuncSet
** Descriptions:            ������ѡ������жϴ������͡��жϷ�������ַ����ʹ���ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
**                          uiType:     �жϴ������� -- 0:�½��� 1:������ 2:�͵�ƽ 3:�ߵ�ƽ
**                          uiFuncAddr: �жϷ�������ַ
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic1IrqFuncSet (unsigned int uiChannel,
                                      unsigned int uiType,
                                      unsigned int uiFuncAddr)
{
    return swiHandle(0x106, uiChannel, uiType, uiFuncAddr);
}

/*********************************************************************************************************
** Function name:           vicIrqFuncClr
** Descriptions:            �����ѡ�����IRQ��Դ
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic1IrqFuncClr (unsigned int uiChannel)
{
    return swiHandle(0x107, uiChannel ,0 ,0);
}

/*********************************************************************************************************
** Function name:           vicIrqEnable
** Descriptions:            ʹ����Ӧ������ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic1IrqEnable (unsigned int uiChannel)
{
    return swiHandle(0x108, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           vicIrqDisable
** Descriptions:            ��ֹ��Ӧ������ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic1IrqDisable (unsigned int uiChannel)
{
    return swiHandle(0x109, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           vicFiqSet
** Descriptions:            ���ò�ʹ����ѡ�ж�ͨ����ΪFIQ�ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
**                          uiType:     �жϴ������� -- 0:�½��� 1:������ 2:�͵�ƽ 3:�ߵ�ƽ
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic1FiqSet (unsigned int uiChannel, unsigned int uiType)
{
    return swiHandle(0x10A, uiChannel, uiType, 0);
}

/*********************************************************************************************************
** Function name:           vicFiqClr
** Descriptions:            �����ѡ�ж�ͨ���ŵ�FIQ�ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic1FiqClr (unsigned int uiChannel)
{
    return swiHandle(0x10B, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           sic2IrqFuncSet
** Descriptions:            ������ѡ������жϴ������͡��жϷ�������ַ����ʹ���ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
**                          uiType:     �жϴ������� -- 0:�½��� 1:������ 2:�͵�ƽ 3:�ߵ�ƽ
**                          uiFuncAddr: �жϷ�������ַ
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic2IrqFuncSet (unsigned int uiChannel,
                                      unsigned int uiType,
                                      unsigned int uiFuncAddr)
{
    return swiHandle(0x10C, uiChannel, uiType, uiFuncAddr);
}

/*********************************************************************************************************
** Function name:           vicIrqFuncClr
** Descriptions:            �����ѡ�����IRQ��Դ
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic2IrqFuncClr (unsigned int uiChannel)
{
    return swiHandle(0x10D, uiChannel ,0 ,0);
}

/*********************************************************************************************************
** Function name:           vicIrqEnable
** Descriptions:            ʹ����Ӧ������ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic2IrqEnable (unsigned int uiChannel)
{
    return swiHandle(0x10E, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           vicIrqDisable
** Descriptions:            ��ֹ��Ӧ������ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic2IrqDisable (unsigned int uiChannel)
{
    return swiHandle(0x10F, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           vicFiqSet
** Descriptions:            ���ò�ʹ����ѡ�ж�ͨ����ΪFIQ�ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
**                          uiType:     �жϴ������� -- 0:�½��� 1:������ 2:�͵�ƽ 3:�ߵ�ƽ
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic2FiqSet (unsigned int uiChannel, unsigned int uiType)
{
    return swiHandle(0x110, uiChannel, uiType, 0);
}

/*********************************************************************************************************
** Function name:           vicFiqClr
** Descriptions:            �����ѡ�ж�ͨ���ŵ�FIQ�ж�
** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
** output parameters:       none
** Returned value:          1:          �ɹ�
**                          0:          ʧ��
*********************************************************************************************************/
__inline unsigned int sic2FiqClr (unsigned int uiChannel)
{
    return swiHandle(0x111, uiChannel, 0, 0);
}

#ifdef __cplusplus
    }
#endif                                                                  /*  __cplusplus                 */

#endif                                                                  /*  _VIC_CONTROL_H_             */
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

