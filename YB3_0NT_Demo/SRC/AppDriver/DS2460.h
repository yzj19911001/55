//#ifndef  __DS2460_H
//#define  __DS2460_H
//
//#include "Config.h"
//
//#ifdef __DS2460_C
//#define ESAM_EXT
//#else
//#define ESAM_EXT  extern
//#endif

/*********************************************************************************************************
** �궨��˵����
**		��ϵͳ�����ж��壬PUBMODΪ����ģʽ��USEMODEΪ��������ģʽ��UPDATEMODEΪˢ����ģʽ
**		Ĭ��ΪUSEMODE	
*********************************************************************************************************/

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
//*********************************************************************************************************/
////ESAM_EXT	uint8 CheckESAM(void);
//
//#endif

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
//*********************************************************************************************************/
//ESAM_EXT	uint8 BurnCSK(void);
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
*********************************************************************************************************/
//ESAM_EXT	uint8 ReadSN(uint8 * p_au8SN);

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
//*********************************************************************************************************/
//ESAM_EXT	uint8 ReadEEPROM(uint8 * p_au8EEPROM);
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
//*********************************************************************************************************/
//ESAM_EXT	uint8 WriteEEPROM(uint8 * p_au8EEPROM);
//
//#endif

//#endif		//__DS2460_H
