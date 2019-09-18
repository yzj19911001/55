/****************************************Copyright (c)****************************************************
**                                     BEIJING  WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			DS2460.c
** Last modified Date:  2011-04-13
** Last Version:		1.0
** Descriptions:		2460/ESAM��غ���
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110413
** Version:				1.0
** Descriptions:		2460/ESAM
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
//#define	__DS2460_C
//#include "DS2460.h"
//#include "I2C1.h"
//#include "SHA.h"

/****************************************************
**
**   DS2460��ر�������
**
****************************************************/
//#define		DS2460_ADDR_W		0x80
//#define		DS2460_ADDR_R		0x81

//#define		MACOUTPUT			0		//�Ƿ�ʹ�ö�����ķ�ʽ���MAC:1,�ǣ�0����

//uint8	getMK(void);
//uint8	WriteCSK(uint8 * p_au8CSK);
//uint8	generateCSK(void);
//uint8	generateRND(void);
//uint8	ComputeArmMAC(void);

//struct	EEPInfo
//{
//	//�ܹ�112��Byte
//	uint8	Flag;			//1		���
//							//0000 0000	������
//							//��1~4λ��	�Ƿ�Ϸ���0110��ʾ�Ϸ�
//							//����λ��	����
//	uint16	UpdateTimes;	//2		ˢ�������
//	
//
//	
//}	m_eepInfo;

//uint8	m_au8MK[8];				//MK
//uint8	m_au8SN[8];				//ROM SN
//uint8	m_au8CSK[8];			//CSK
//uint8	m_Input_buffer[64];		//���뻺����������buffer
//uint8	m_au8MACARM[20];		//���ɵ�MACֵ
//uint8	m_u8ACKFlag;			//Ӧ���־��0��ʾ��Ӧ��1��ʾ��Ӧ��

//uint8	m_u8ChkResult;			//У������0��ʾͨ����1��ʾ��ͨ��
//uint8	m_au8RND[64];			//�����

//#if	MACOUTPUT == 0				//������ֱ�Ӷ�MAC OUTput buffer
//uint8	m_u8Command	= 0x8C;		//0x8C	10001100	ѡ��E-SECRET1��Ϊ���룬MAC OUTput buffer��Ϊ���
//#else							//����ֱ�Ӷ�MAC OUTput buffer
//uint8	m_au8MAC2460[20];		//2460��MACֵ
//uint8	m_u8Command	= 0x94;		//0x94	10010100	ѡ��E-SECRET2��Ϊ���룬MAC OUTput buffer��Ϊ���
//#endif

//uint8	m_au8InputBuffer1[64] = {0x3c,0xe2,0x99,0x18,		//û��	  
//                0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,
//                0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,
//				0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,
//				0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,
//				0x28,0x29,0x2A,0x2B,0x00,0x00,0x00,0x36,0x2C,0x2D,
//				0x2E,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xB8};


//#ifdef	USEMODE
///*********************************************************************************************************
//** Function name:		CheckESAM
//** Descriptions:		ESAMУ��
//**
//** input parameters:	None
//** output parameters:	None
//** Returned value:		У������	1(>0)��	ʧ��
//**									0��		�ɹ�
//** Created by:			ZHANG Ye
//** Created Date:		20110413
//**--------------------------------------------------------------------------------------------------------
//** Modified by:			
//** Modified date:		
//**--------------------------------------------------------------------------------------------------------
//*********************************************************************************************************/
//uint8 CheckESAM(void)
//{
//#if	MACOUTPUT > 0				//����ֱ�Ӷ�MAC OUTput buffer
//	uint8	i;
//#endif
//
//
//	//��ȡSN,MK
//	ReadSN(m_au8SN);
//	getMK();
//	OSTimeDly(1);	//10ms
//
//	//����CSK
//	generateCSK();
//	OSTimeDly(1);	//10ms
//
//	//���������
//	generateRND();
//	OSTimeDly(1);	//10ms
//
//	//����MAC
//	ComputeArmMAC();
//	OSTimeDly(1);	//10ms
//
//	//ʹ2460����MAC
//	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x00, m_au8RND, 64);
//	OSTimeDly(1);	//10ms
//
//	//д����������������MAC
//	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x5C, &m_u8Command, 1);
//	OSTimeDly(1);	//10ms
//
//#if	MACOUTPUT == 0				//������ֱ�Ӷ�MAC OUTput buffer
//	
//	//��I2CдMAC
//	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x5E, m_au8MACARM, 20);
//	OSTimeDly(1);	//10ms
//			
//	//���ݵ�20λ�Ƿ���Ӧ���ж���֤�Ƿ�ͨ��
//	if (m_u8ACKFlag == 0)	//��Ӧ��ʧ��
//		return	1;
//	else					//��Ӧ��ͨ��
//		return	0;		
//
//#else							//����ֱ�Ӷ�MAC OUTput buffer
//	I2C_ReadNByte(DS2460_ADDR_R, 1, 0x40, m_au8MAC2460, 20);
//	OSTimeDly(1);	//10ms
//
//	for (i=0; i<20; i++)
//	{
//		if (m_au8MAC2460[i] != m_au8MACARM[i])
//			return	1;
//	}
//	return	0;
//	
//#endif		
//	
//}
//
//#endif

/*********************************************************************************************************
** Function name:		getMK
** Descriptions:		��ȡMK
**
** input parameters:	None
** output parameters:	None
** Returned value:		���������	1(>0)��	ʧ��
**									0��		�ɹ�
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//uint8 getMK(void)
//{
//	uint8 *	l_pu8Temp;
//	l_pu8Temp = m_au8MK;
//
//	//��ʱ
//	*(l_pu8Temp++)	= 0x11;
//	*(l_pu8Temp++)	= 0x22;
//	*(l_pu8Temp++)	= 0x33;
//	*(l_pu8Temp++)	= 0x44;
//	*(l_pu8Temp++)	= 0x55;
//	*(l_pu8Temp++)	= 0x66;
//	*(l_pu8Temp++)	= 0x77;
//	*(l_pu8Temp++)	= 0x88;
//	l_pu8Temp	= 0;
//	return	0;
//}

/*********************************************************************************************************
** Function name:		ComputeArmMAC
** Descriptions:		����Arm��MAC
**
** input parameters:	None
** output parameters:	None
** Returned value:		���������	1(>0)��	ʧ��
**									0��		�ɹ�
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//uint8 ComputeArmMAC(void)
//{
//	uint8	l_u8Cnt;
//	long	A, B, C, D, E;
//
//	//�˴�Ӧʹ��ʱ��
//	srand(0);
//	
//	for(l_u8Cnt=0; l_u8Cnt<4; l_u8Cnt++)
//	{
//		m_au8RND[l_u8Cnt] = m_au8CSK[l_u8Cnt];		//��Կ����λ
//	}
//	for(l_u8Cnt=48; l_u8Cnt<52; l_u8Cnt++)		
//	{
//		m_au8RND[l_u8Cnt] = m_au8CSK[l_u8Cnt-44];	//��Կ����λ
//	}
//	
//	ComputeSHAEE(m_au8RND, &A, &B, &C, &D, &E);
//
//	m_au8MACARM[0]	= (E)&0xff;
//	m_au8MACARM[1]	= (E>>8)&0xff;
//	m_au8MACARM[2]	= (E>>16)&0xff;
//	m_au8MACARM[3]	= (E>>24)&0xff;
//	m_au8MACARM[4]	= (D)&0xff;
//	m_au8MACARM[5]	= (D>>8)&0xff;
//	m_au8MACARM[6]	= (D>>16)&0xff;
//	m_au8MACARM[7]	= (D>>24)&0xff;
//	m_au8MACARM[8]	= (C)&0xff;
//	m_au8MACARM[9]	= (C>>8)&0xff;
//	m_au8MACARM[10]	= (C>>16)&0xff;
//	m_au8MACARM[11]	= (C>>24)&0xff;
//	m_au8MACARM[12]	= (B)&0xff;
//	m_au8MACARM[13]	= (B>>8)&0xff;
//	m_au8MACARM[14]	= (B>>16)&0xff;
//	m_au8MACARM[15]	= (B>>24)&0xff;
//	m_au8MACARM[16]	= (A)&0xff;
//	m_au8MACARM[17]	= (A>>8)&0xff;
//	m_au8MACARM[18]	= (A>>16)&0xff;
//	m_au8MACARM[19]	= (A>>24)&0xff;	
//
//	return	0;
//}

/*********************************************************************************************************
** Function name:		generateRND
** Descriptions:		���������Buf
**
** input parameters:	None
** output parameters:	None
** Returned value:		���������	1(>0)��	ʧ��
**									0��		�ɹ�
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//uint8 generateRND(void)
//{
//	uint8	l_u8Cnt;
//
//	//�˴�Ӧʹ��ʱ��
//	srand(0);
//	
//	for(l_u8Cnt=0; l_u8Cnt<4; l_u8Cnt++)
//	{
//		m_au8RND[l_u8Cnt] = 0x00;				//�ᱻ�滻����4λû��Ҫ��
//	}
//	for(l_u8Cnt =4; l_u8Cnt<48; l_u8Cnt++)
//	{
//		m_au8RND[l_u8Cnt] = rand() & 0xff;		//�����
//	}
//	for(l_u8Cnt=48; l_u8Cnt<52; l_u8Cnt++)		
//	{
//		m_au8RND[l_u8Cnt] = 0x00;				//�ᱻ�滻����4λû��Ҫ��
//	}
//	for(l_u8Cnt =52; l_u8Cnt<55; l_u8Cnt++)
//	{
//		m_au8RND[l_u8Cnt] = rand() & 0xff;		//�����
//	}
//	m_au8RND[55] = 0x80;
//	for(l_u8Cnt=56; l_u8Cnt<62;l_u8Cnt++)
//	{
//		m_au8RND[l_u8Cnt] = 0x0;
//	}
//	m_au8RND[62]=0x01;
//	m_au8RND[63]=0xB8;
//
//	return	0;
//}

/*********************************************************************************************************
** Function name:		generateCSK
** Descriptions:		����MK��SN����CSK
**
** input parameters:	None
** output parameters:	None
** Returned value:		���������	1(>0)��	ʧ��
**									0��		�ɹ�
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//uint8 generateCSK(void)
//{
//	uint8	l_u8Cnt;
//	long	A, B, C, D, E;
//	uint8 *	l_au8Temp;
//
//	//����BUF
//	for(l_u8Cnt=0; l_u8Cnt<4; l_u8Cnt++)
//	{
//		m_au8InputBuffer1[l_u8Cnt] = *(m_au8SN+l_u8Cnt);	  
//	}
//	for(l_u8Cnt =4; l_u8Cnt<48; l_u8Cnt++)
//	{
//		m_au8InputBuffer1[l_u8Cnt] = *(m_au8MK+(l_u8Cnt%4));
//	}
//	for(l_u8Cnt=48; l_u8Cnt<52; l_u8Cnt++)		
//	{
//		m_au8InputBuffer1[l_u8Cnt] = *(m_au8SN+l_u8Cnt-44);	
//	}
//	for(l_u8Cnt =52; l_u8Cnt<55; l_u8Cnt++)
//	{
//		m_au8InputBuffer1[l_u8Cnt] = *(m_au8MK+(l_u8Cnt%4)+4);
//	}
//	m_au8InputBuffer1[55] = 0x80;
//	for(l_u8Cnt=56; l_u8Cnt<62;l_u8Cnt++)
//	{
//		m_au8InputBuffer1[l_u8Cnt] = 0x0;
//	}
//	m_au8InputBuffer1[62]=0x01;
//	m_au8InputBuffer1[63]=0xB8;
//	
//	//����ó���Կ	
//	ComputeSHAEE(m_au8InputBuffer1, &A, &B, &C, &D, &E);
//
//	l_au8Temp	= m_au8CSK;
//
//	*(l_au8Temp++)	= (E)&0xff;
//	*(l_au8Temp++)	= (E>>8)&0xff;
//	*(l_au8Temp++)	= (E>>16)&0xff;
//	*(l_au8Temp++)	= (E>>24)&0xff;
//	*(l_au8Temp++)	= (D)&0xff;
//	*(l_au8Temp++)	= (D>>8)&0xff;
//	*(l_au8Temp++)	= (D>>16)&0xff;
//	*(l_au8Temp++)	= (D>>24)&0xff;
//
//	l_au8Temp	= 0;
//
//	return	0;
//}

/*********************************************************************************************************
** Function name:		WriteCSK
** Descriptions:		дCSK
**
** input parameters:	p_au8CSK
** output parameters:	None
** Returned value:		���������	1(>0)��	ʧ��
**									0��		�ɹ�
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//uint8 WriteCSK(uint8 * p_au8CSK)
//{
//#if	MACOUTPUT == 0				//������ֱ�Ӷ�MAC OUTput buffer
//	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x60, p_au8CSK, 8);   //E_SECRET1 HIDE=YES��MAC������ɶ�
//#else							//����ֱ�Ӷ�MAC OUTput buffer
//	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x68, p_au8CSK, 8);   //E_SECRET2 HIDE=NO��MAC����ɶ�
//#endif
//
//	OSTimeDly(1);	//10ms
//	return	0;
//}

//#ifdef	PUBMODE
//
///*********************************************************************************************************
//** Function name:		BurnCSK
//** Descriptions:		��дCSK
//**
//** input parameters:	None
//** output parameters:	None
//** Returned value:		���������	1(>0)��	ʧ��
//**									0��		�ɹ�
//** Created by:			ZHANG Ye
//** Created Date:		20110413
//**--------------------------------------------------------------------------------------------------------
//** Modified by:			
//** Modified date:		
//**--------------------------------------------------------------------------------------------------------
//*********************************************************************************************************/
//uint8 BurnCSK(void)
//{
//	//��ȡSN,MK
//	ReadSN(m_au8SN);
//	getMK();
//	OSTimeDly(1);	//10ms
//
//	//����CSK
//	generateCSK();
//	OSTimeDly(1);	//10ms
//
//	return	WriteCSK(m_au8CSK);
//}
//
//#endif

/*********************************************************************************************************
** Function name:		ReadSN
** Descriptions:		��ȡ�豸SN
**
** input parameters:	None
** output parameters:	p_au8SN
** Returned value:		���������	1(>0)��	ʧ��
**									0��		�ɹ�
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//uint8 ReadSN(uint8 * p_au8SN)
//{
//	I2C1_ReadNByte(DS2460_ADDR_R, 1, 0xF0, p_au8SN, 8);
//	Delay(10);		//��ʱ10ms
//	return	0;
//}

//#if (defined USEMODE) || (defined UPDATEMODE)
//
///*********************************************************************************************************
//** Function name:		ReadEEPROM
//** Descriptions:		��ȡEEPROM����112Byte����
//**
//** input parameters:	None
//** output parameters:	p_au8EEPROM
//** Returned value:		���������	1(>0)��	ʧ��
//**									0��		�ɹ�
//** Created by:			ZHANG Ye
//** Created Date:		20110413
//**--------------------------------------------------------------------------------------------------------
//** Modified by:			
//** Modified date:		
//**--------------------------------------------------------------------------------------------------------
//*********************************************************************************************************/
//ESAM_EXT	uint8 ReadEEPROM(uint8 * p_au8EEPROM)
//{
//	I2C1_ReadNByte(DS2460_ADDR_R, 1, 0x80, p_au8EEPROM, 112);
//	Delay(10);		//��ʱ10ms
//	return	0;
//}
//
//#endif

//#if (defined PUBMODE) || (defined UPDATEMODE)
//
///*********************************************************************************************************
//** Function name:		WriteEEPROM
//** Descriptions:		��EEPROMд������112Byte����
//**
//** input parameters:	p_au8EEPROM
//** output parameters:	None
//** Returned value:		���������	1(>0)��	ʧ��
//**									0��		�ɹ�
//** Created by:			ZHANG Ye
//** Created Date:		20110413
//**--------------------------------------------------------------------------------------------------------
//** Modified by:			
//** Modified date:		
//**--------------------------------------------------------------------------------------------------------
//*********************************************************************************************************/
//ESAM_EXT	uint8 WriteEEPROM(uint8 * p_au8EEPROM)
//{
//	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x80, p_au8EEPROM, 112);
//	Delay(10);		//��ʱ10ms
//	return	0;
//}
//
//#endif
