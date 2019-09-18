/****************************************Copyright (c)****************************************************
**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
**                                     
**                               http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           PCF8563.c
** Last modified Date:  20110413
** Last Version:        1.00
** Descriptions:        for lpc3200
**--------------------------------------------------------------------------------------------------------
** Created by:          ZHANG Ye
** Created date:        20110413
** Version:             1.00
** Descriptions:        The original version
*********************************************************************************************************/
#define	__PCF8563_C
#include "PCF8563.h"

#define		PCF8563ADRW		0xA2	//д
#define		PCF8563ADRR		0xA3	//��

#define		BCD_to_HEC(b)	(((b>>4)*10)+(b&0x0f))
#define		HEC_to_BCD(h)	(((h/10)<<4)|(h%10))

#define		StopRTC()		RTC_CTRL	|= 0x40
#define		StartRTC()		RTC_CTRL	&= 0xbf


uint16 m_u16TotalDays[12]		= {0,31,59,90,120,151,181,212,243,273,304,334};		/* �������·����ڱ�				*/
uint16 m_u16TotalDaysLeap[12]	= {0,31,60,91,121,152,182,213,244,274,305,335};		/* �����·����ڱ�				*/

uint16 m_u16MonthDays[12]		= {31,28,31,30,31,30,31,31,30,31,30,31};			/* �������·����ڱ�				*/
uint16 m_u16MonthDaysLeap[12]	= {31,29,31,30,31,30,31,31,30,31,30,31};			/* �����·����ڱ�				*/

static	uint8	SetRTC(RTCDateTime * p_RdtTime);
static	uint8	GET8563(RTCDateTime * p_RdtTime);
static	uint8	SET8563(RTCDateTime * p_RdtTime);

/*********************************************************************************************************
** Function name:		RTC8563Init
** Descriptions:		RTC��8563��ʼ��
** input parameters:	none 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110503	  
**-------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye	
** Modified date:		20110518	
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void RTC8563Init(void)
{
	uint8 temp;

    StopRTC();
	//����8563
    temp = 0;
    I2C2_WriteNByte(PCF8563ADRW, 1, 0x00, &temp, 1);	

	//����RTC
	RTC_CTRL   = 0x10;					/* ��λ������   Ӳ����λ        */
    RTC_CTRL   = 0x00;					/* ֹͣ������һֱ��λ           */
	
	Synchro8563();						//ͬ��
	StartRTC();
}

/********************************************************************************************************
* ��    �ƣ�SetRTC
* ��    �ܣ�����RTC
* ��ڲ�����p_RdtTime    ָ��ʱ��ṹ���ָ��
* ���ڲ�����FALSE or TRUE
*********************************************************************************************************/
uint8 SetRTC(RTCDateTime * p_RdtTime)
{
	uint32	l_u32SecCnt;
	uint32	l_u32Temp;
	RTCDateTime	l_rdtTemp;

	l_u32SecCnt	= 0;

	l_rdtTemp.u16Year		= p_RdtTime -> u16Year;  
	l_rdtTemp.u8Month		= p_RdtTime -> u8Month;
	l_rdtTemp.u8Day			= p_RdtTime -> u8Day;
	l_rdtTemp.u8Week		= p_RdtTime -> u8Week;
	l_rdtTemp.u8Hour		= p_RdtTime -> u8Hour;
	l_rdtTemp.u8Minute		= p_RdtTime -> u8Minute;
	l_rdtTemp.u8Second		= p_RdtTime -> u8Second;    

	StopRTC();                     		/* ����ʱ��ʱ�ȹرռ�����       */
    
	if (l_rdtTemp.u16Year > 2099) 						/* �����겻�ܳ���99�꣬			*/
	{											    
		l_rdtTemp.u16Year =  2099;						/* ����ȡ99��					*/
    }
	else if (l_rdtTemp.u16Year < 2000)
	{
		l_rdtTemp.u16Year =  2000;
	}
    
    l_u32Temp 		=  l_rdtTemp.u16Year - 2000;
    l_u32SecCnt		=  (l_u32Temp * 365 + (l_u32Temp / 4));   		/* ͨ����ݼ�������             */
    if (l_u32Temp % 4) 												/* �������ʱ�򲹳�����         */
	{
        l_u32SecCnt += 1;
    }                                          						

    if (l_rdtTemp.u8Month > 12) 										/* �����²��ܳ���12��,����ȡ12	*/
	{										
        l_rdtTemp.u8Month	= 12;
    }
   
    if (l_u32Temp % 4) 													/* �����꣬�·�����				*/
	{                                                        
        l_u32Temp	=  (uint8)(l_rdtTemp.u8Month - 1);
        l_u32SecCnt += (uint16)m_u16TotalDays[l_u32Temp]; 
		
		l_u32Temp	= (uint16)m_u16MonthDays[(uint8)(l_rdtTemp.u8Month - 1)];
		if(l_rdtTemp.u8Day < l_u32Temp)
		{                                                        
        	l_u32Temp	= l_rdtTemp.u8Day;
		}
    }
    else 																/* ���꣬�·�����				*/
	{                                                              
        l_u32Temp	=  (uint8)(l_rdtTemp.u8Month - 1);
        l_u32SecCnt += (uint16)m_u16TotalDaysLeap[l_u32Temp];

		l_u32Temp	= (uint16)m_u16MonthDaysLeap[(uint8)(l_rdtTemp.u8Month - 1)];
		if(l_rdtTemp.u8Day < l_u32Temp)
		{                                                        
        	l_u32Temp	= l_rdtTemp.u8Day;
		}
    }

    l_u32Temp	= (uint8)(l_u32Temp - 1);
    l_u32SecCnt += l_u32Temp;               							/* ���ϵþ���2000-01- 01������  */
    l_u32SecCnt	*= 86400;                    							/* һ�������Ϊ86400��          */

    if (l_rdtTemp.u8Hour > 23) 											/* ����Сʱ���ܳ���23Сʱ��		*/
	{																	/* ����23Сʱ					*/
        l_rdtTemp.u8Hour	= 23;											
    }
    
    l_u32Temp	= l_rdtTemp.u8Hour;
    l_u32SecCnt	+= l_u32Temp * 3600;                     				/* һСʱ3600��                 */
    
    if (l_rdtTemp.u8Minute > 59) 											/* ���÷��Ӳ��ܳ���59�֣�		*/
	{																	/* ����ȡ59��					*/
        l_rdtTemp.u8Minute	=  59;												
    }
    
    l_u32Temp	= l_rdtTemp.u8Minute;
    l_u32SecCnt += l_u32Temp * 60;   									/* һ����60��                   */

    if (l_rdtTemp.u8Second > 59) 										/* �����벻�ܳ���59�룬			*/
	{																	/* ����ȡ59��					*/
        l_rdtTemp.u8Second	= 59;												
    }
    
    l_u32Temp	= l_rdtTemp.u8Second;
    l_u32SecCnt += l_u32Temp;

	
	RTC_UCOUNT	= l_u32SecCnt;                       				/* ������õ���ֵ����������     */
    RTC_DCOUNT	= (uint32)(0xffffffff - l_u32SecCnt);     			/* �����������͵ݼ�����������   */
    
    StartRTC();
    return	TRUE;
}

/********************************************************************************************************
* ��    �ƣ�SET8563
* ��    �ܣ�����PCF8563
* ��ڲ�����p_RdtTime    ָ��ʱ��ṹ���ָ��
* ���ڲ�����FALSE or TRUE
*********************************************************************************************************/
uint8 SET8563(RTCDateTime * p_RdtTime)
{
	uint8 temp[7];
	uint8 u16Year;
    
	temp[0] = HEC_to_BCD(p_RdtTime->u8Second);
	temp[1] = HEC_to_BCD(p_RdtTime->u8Minute);
	temp[2] = HEC_to_BCD(p_RdtTime->u8Hour);
	temp[3] = HEC_to_BCD(p_RdtTime->u8Day);
	temp[4] = HEC_to_BCD(p_RdtTime->u8Week);
	temp[5] = HEC_to_BCD(p_RdtTime->u8Month);
    
	if (p_RdtTime->u16Year >= 2000){
		temp[5] |= 0x80;
		u16Year = p_RdtTime->u16Year - 2000;
		temp[6] = HEC_to_BCD(u16Year);
	}else{
		u16Year = p_RdtTime->u16Year - 1900;
		temp[6] = HEC_to_BCD(u16Year);
	}
	return	I2C2_WriteNByte(PCF8563ADRW, 1, 0x02, temp, 7);
}

/********************************************************************************************************
* ��    �ƣ�GET8563
* ��    �ܣ���ȡPCF8563
* ��ڲ�����p_RdtTime
* ���ڲ�����FALSE or TRUE
*********************************************************************************************************/
uint8 GET8563(RTCDateTime * p_RdtTime)
{
	uint8 temp[7];
    
    if (I2C2_ReadNByte(PCF8563ADRR, 1, 0x02, temp, 7)==FALSE) {
        return FALSE;
    }
    
    p_RdtTime -> u8Second	= BCD_to_HEC((temp[0]&0x7f));
    p_RdtTime -> u8Minute	= BCD_to_HEC((temp[1]&0x7f));
    p_RdtTime -> u8Hour		= BCD_to_HEC((temp[2]&0x3f));
    p_RdtTime -> u8Day		= BCD_to_HEC((temp[3]&0x3f));
    p_RdtTime -> u8Week		= BCD_to_HEC((temp[4]&0x07));
    
    if (temp[5]&0x80){
        p_RdtTime -> u8Month	= BCD_to_HEC((temp[5]&0x1f));
        p_RdtTime -> u16Year	= BCD_to_HEC(temp[6])+2000;
    }else{
        p_RdtTime -> u8Month	= BCD_to_HEC((temp[5]&0x1f));
        p_RdtTime -> u16Year	= BCD_to_HEC(temp[6])+1900;
    }
    return TRUE;
}

/********************************************************************************************************
* ��    �ƣ�Synchro8563
* ��    �ܣ���8563�ж�ȡʱ�䣬������RTC
* ��ڲ�����none
* ���ڲ�����FALSE or TRUE
*********************************************************************************************************/
void Synchro8563(void)
{
	RTCDateTime		l_rdtTemp;

	GET8563(&l_rdtTemp);		//��ȡ8563

	SetRTC(&l_rdtTemp);				//��������ʱ�䱣�ֵ�RTC
}

/********************************************************************************************************
* ��    �ƣ�SetRTCTime
* ��    �ܣ�����RTCʱ��
* ��ڲ�����p_pstTime
* ���ڲ�������
*********************************************************************************************************/
void SetRTCTime(SystemTime * p_pstTime)
{ 
	SET8563(p_pstTime);		//����8563
	SetRTC(p_pstTime);		//����RTC
}


/********************************************************************************************************
* ��    �ƣ�GetRTCTime
* ��    �ܣ���ȡRTCʱ��
* ��ڲ�����p_pstTime
* ���ڲ�������
*********************************************************************************************************/
void GetRTCTime(SystemTime * p_pstTime)
{

	GET8563((RTCDateTime *)p_pstTime);
//	uint32	l_u32SecCnt	= 0;
//	uint32	l_u32Temp;
//	uint16	l_u16Days;
//	uint32	l_u32WeekTemp;
//
//	uint32	i;
//	l_u32WeekTemp	= 6;		//2000-01-01 ������
//    l_u32SecCnt		= (uint32)RTC_UCOUNT;
//	
//	l_u32WeekTemp	+= (l_u32SecCnt / 86400);
//	p_pstTime -> u8Week	= l_u32WeekTemp % 7;
//
//    l_u32Temp		= (uint8)(l_u32SecCnt / 126230400);         /* 4�꣬���������һ������ܹ�  */
//                                                      			/* ����Ϊ126230400��            */
//    l_u32Temp		= (uint8)(l_u32Temp * 4);
//    l_u32SecCnt		= l_u32SecCnt % 126230400;
//
//    if (l_u32SecCnt >= 31622400) 
//	{
//        l_u32Temp ++;                                 			/* ��һ�����꣬����Ϊ31622400   */
//        l_u32SecCnt	= l_u32SecCnt - 31622400;
//        if (l_u32SecCnt >= 31536000) 
//		{
//            l_u32Temp ++;                            			/* �ڶ�������꣬����Ϊ31536000 */
//            l_u32SecCnt	= l_u32SecCnt - 31536000;
//        } 
//		if (l_u32SecCnt >= 31536000) 
//		{
//            l_u32Temp ++;                                   	/* ����������꣬����Ϊ31536000 */
//            l_u32SecCnt = l_u32SecCnt-31536000;
//        }
//    }
//
//    p_pstTime->u16Year	= l_u32Temp + 2000;                    		/* ���ϲ������õ���ǰ���     */
//
//	l_u16Days		= l_u32SecCnt / 86400;
//	l_u32SecCnt		= l_u32SecCnt % 86400;
//
//    if (l_u32Temp % 4) 
//	{
//        for (i = 11; i > 0; i--) 
//		{
//            if (l_u16Days >= m_u16TotalDays[i]) 
//			{
//				l_u16Days	-= m_u16TotalDays[i];
//                break;
//            }
//        }
//    }
//    else 
//	{                                                   		/* ���꣬�·ݱ�ΪTable1         */
//        for (i = 11; i > 0; i--) 
//		{
//            if (l_u16Days >= m_u16TotalDaysLeap[i]) 
//			{
//				l_u16Days	-= m_u16TotalDaysLeap[i];
//                break;
//            }
//        }
//    }
//    i	+=1 ;
//    p_pstTime->u8Month		= i;                             			/* ����õ��·���               */
//
//    i	= l_u16Days + 1;
//    p_pstTime->u8Day   	= i;                                			/* ÿ��86400�룬ȡ������        */
//
//    i	= (uint8)(l_u32SecCnt / 3600);
//    p_pstTime->u8Hour		= i;                                        	/* ÿСʱ3600�룬ȡ��Сʱ��     */
//
//    l_u32SecCnt		=  l_u32SecCnt % 3600;
//    i	= (uint8)(l_u32SecCnt / 60);
//    p_pstTime->u8Minute	= i;                                     		/* ȡ�÷���                     */
//
//    l_u32SecCnt		= l_u32SecCnt % 60;
//    p_pstTime->u8Second	= (uint8)l_u32SecCnt;                       	/* ȡ������                     */

}
