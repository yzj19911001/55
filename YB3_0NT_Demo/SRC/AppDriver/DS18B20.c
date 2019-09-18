/****************************************Copyright (c)****************************************************
**                                     BEIJING  WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			DS18B20.c
** Last modified Date:  20110530
** Last Version:		1.0
** Descriptions:		�¶Ȳ�������
**
**--------------------------------------------------------------------------------------------------------
** Created by:			HE NA
** Created date:		20110530
** Version:				1.0
** Descriptions:		�¶�
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__DS18B20_C
#include "DS18B20.h"

#define  DQ          (1<<25)	  /*DS18B20*/
#define  DQ_STATE    (1<<10)	  /*DS18B20*/

uint8 DS18B20ID[8]={0};      //zg

 /*********************************************************************************************************
** Function name:           Delayus
**
** Descriptions:            ��ʱ   Delayus(8)��ʱ1us
**
** input parameters:        us : ��ʱ����
** output parameters:       None
** Returned value:          None
**
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void Delayus(uint32 p_ulus)
{
    uint32 i = 0;
   
    for (; p_ulus; p_ulus--) 
	{   
	   for (i = 0; i < 3; i++);
    }	
}
/*********************************************************************************************************
** Function name:           Delayms
**
** Descriptions:            ��ʱ
**
** input parameters:        ms : ��ʱ����
** output parameters:       None
** Returned value:          None
**
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void Delayms(uint32 p_ulms)
{
    uint32 i = 0;
   
    for (; p_ulms; p_ulms--) 
	{   
	    for (i = 0; i < 6500; i++);
    }	
}

//�� 1-wire �����϶�ȡһ���ֽ�
uint8 ReadByte(void)
{
    uint8 i;
    uint8 l_u8Value = 0;
    for(i=8;i>0;i--)
    {
        l_u8Value>>=1;
        P2_DIR_SET = DQ;	  //GPIO����Ϊ���ģʽ
        P3_OUTP_CLR = DQ;     //����͵�ƽ
        Delayus(8);			  //1

        P2_DIR_CLR = DQ;	  //GPIO����Ϊ����ģʽ
        Delayus(24); 		  //3
       
        if((P3_INP_STATE&DQ_STATE)!=0) 
		{
			l_u8Value|=0x80;
		}    
        Delayus(500); //60   wait for rest of timeslot
        Delayus(8);	  //1
    }
    return(l_u8Value);
}

//�� 1-WIRE ������дһ���ֽ�
void WriteByte(uint8 p_u8Val)
{
    uint8 i;
	uint8 l_u8Temp;
    for (i=8; i>0; i--) // writes byte, one bit at a time
    {
        P2_DIR_SET = DQ;	  //GPIO����Ϊ���ģʽ
        P3_OUTP_CLR = DQ;  //����͵�ƽ
        Delayus(40);//  5   pull DQ low to start timeslot

		l_u8Temp = p_u8Val&0x01;
		if(l_u8Temp==0)
		{
			P3_OUTP_CLR = DQ;  //����͵�ƽ
			Delayus(480); //  60   hold value for remainder of timeslot
            P3_OUTP_SET = DQ;  //����ߵ�ƽ
			Delayus(8);	  //1
		}
		if(l_u8Temp==0x01)
		{
			P3_OUTP_SET = DQ;  //����ߵ�ƽ
			Delayus(480); // 60   hold value for remainder of timeslot
            P3_OUTP_SET = DQ;  //����ߵ�ƽ
			Delayus(8);	  //1
		}
        p_u8Val=p_u8Val/2;
     }
     Delayus(440);	//55
}
//��λ
uint8 DSReset(void)
{
   uint8 l_u8Presence;

   P2_DIR_SET = DQ;	  //GPIO����Ϊ���ģʽ
   P3_OUTP_SET = DQ;
   Delayus(600);
   P3_OUTP_CLR = DQ;  //����͵�ƽ
   Delayus(4000); // leave it low for 480us	  500

   P2_DIR_CLR = DQ;	  //GPIO����Ϊ����ģʽ
   Delayus(300); // wait for l_u8Presence		  35

   l_u8Presence = P3_INP_STATE & DQ_STATE;
   
   P2_DIR_SET = DQ;	  //GPIO����Ϊ���ģʽ
   P3_OUTP_SET = DQ;
   Delayus(3500); // wait for end of timeslot  465
   return(l_u8Presence); // l_u8Presence signal returned
}   // 0=l_u8Presence, 1 = no part

//��ȡ�¶�
int ReadTemperature(void)
{
   union{
          uint8 c[2];
          uint16 x;
        }temp;
   DSReset();
   WriteByte(0xCC); // Skip ROM
   WriteByte(0x44); // Start Conversion
   Delayms(80);
   P2_DIR_CLR = DQ;	  //GPIO����Ϊ����ģʽ
   while((P3_INP_STATE&DQ_STATE)==0);
   DSReset();
   WriteByte(0xCC); // Skip ROM
   WriteByte(0xBE); // Read Scratch Pad	��RAM�ж�ȡ����
   
   temp.c[0]=ReadByte();
   temp.c[1]=(uint16)(ReadByte());
   temp.x =	(temp.c[1]<<8)|temp.c[0];
   temp.x>>=4;		     //ReadByte�õ���ֵ����λΪ�¶ȵ�С������ֵ��Ӧ����ֻ�õ������¶ȣ����Խ�С���������ʡȥ

   return temp.x;
}

void DS18B20Init(void)
{
    DSReset();
    WriteByte(0xCC); // Skip ROM
    WriteByte(0x4E); // ��RAM��д����
    WriteByte(0x00); // ��RAM��д���ݵĸߵ�ַ
    WriteByte(0x00); // ��RAM��д���ݵĵ͵�ַ
    WriteByte(0x1F);
}

void DS18B20IDRD(void)				    //zg  ��ȡID
{  
   uint8 i=0;
   DSReset(); 
   Delayus(100);
   WriteByte(0x33); // read ROM
   
   for (i=0;i<8;i++)
     {
	   DS18B20ID[i] = ReadByte();
	 }
}

