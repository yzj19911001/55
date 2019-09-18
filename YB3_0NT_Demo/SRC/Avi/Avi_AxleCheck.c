#include "stdafx.h"
#include "Avi_AxleCheck.h"
#include "Avi_MathProc.h"
#ifdef _KeilEnVi
#include "PrtlProc.h"
#endif

int t_car2 = 0;
uint16 car_height_temp = 0;//С����־ ��ʱ
///min_dis_X[1]	���������ķֽ�� ���뼤��������
///min_dis_X[2]	���������ķֽ�� �������߶�
///min_dis_X[3]	�鲢��߽���� ���45�ڵĺϲ�
///min_dis_X[5]	�߽������
///min_dis_X[6]	ʱ���
///min_dis_X[7]	֡��������
///min_dis_X[8]	ÿ֡�ĸ߶���Ϣ
///min_dis_X[9]	ѹ��֡��Ӧ��ԭʼ����
///min_dis_X[10]���������򵽵���ĵ���
int32  min_dis_X_t[12] ;

///min_dis_X[1]	���������ķֽ�� ���뼤��������
///min_dis_X[2]	���������ķֽ�� �������߶�
///min_dis_X[3]	�鲢��߽���� ���45�ڵĺϲ�
///min_dis_X[5]	�߽������
///min_dis_X[6]	ʱ���
///min_dis_X[7]	֡��������
///min_dis_X[8]	ÿ֡�ĸ߶���Ϣ
///min_dis_X[9]	ѹ��֡��Ӧ��ԭʼ����
///min_dis_X[10]���������򵽵���ĵ���
int32  min_dis_X[12][FRAME_MAXCNT]={0} ;

uint16	point_below_height[POINT_SUM] = {0};
uint16 beyond_axle_ground = 0;//Խ������ɨ�赽�����ϵĵ�

unsigned short int point_count = 0;//��Ч����
unsigned int empty_frame_count = 0;//�޳�֡��

unsigned int vehicle_count = 0;//������
unsigned int vehicle_new = 0;//�³�����

uint32	height_over_count = 0; //���ߴ���2300��֡��
uint32	ground_over_count = 0;
uint32	ground_car = 0;//���̸�
uint8	car_height = 0;
unsigned char	car_axle =0;		//С�� �����ж�
uint8	check_axle = 0;
uint8	over_Flag = 0;
///�߶���ֵ�µ���Ч�� ���뼤�����ľ���
int16 belowHeigh_X[FRAME_MAXCNT][4] = {0};
int16 belowHeigh_X_t[4] = {0};
uint32	g_FrameCnt = 0;//֡���

///�ٽ���ļ���ֵ
int32  neighout_max_X = 0;
///�ٽ���ļ�Сֵ
int32  neighout_min_X = 0;
///��������
uint32 search_count = 0 ;
///��ֵ�������
uint16 index_top  = 0;

uint32	index_extract = 0 ;
uint32	search_count_add = 0;

uint16 axle_count = 0;
uint16 axle_count_record = 0;
///axle_index[0] ���ֳ���֡ ԭʼ֡���
///axle_index[1]//������ʼ֡���
///axle_index[2]//�������֡���
///axle_index[3]//��������
///axle_index[4]//����ͶӰ���x
///axle_index[5]//������벿�ֱ仯��
///axle_index[6]//�����Ұ벿�ֱ仯��
///axle_index[8]//������ʼ֡ ԭʼ֡���
///axle_index[9]//���ֽ���֡ ԭʼ֡���
///axle_index[10]//�߶���ֵ�µĵ��X���ֵ
///axle_index[11]//�����ٶ�
///axle_index[12]//������

int32 axle_index[MAX_AXLE_COUNT][15] = {0};

int32 axle_min_X=65535;
///�����ٽ���
uint16	near_point_count[POINT_SUM] = {0};

int	MAX_Height = 0;//��̥�ж���ֵ

int	MAX_Height2 = 0;//С��̥�ж���ֵ

uint32	g_Frame_Time = 0;//֡ʱ���
uint32	g_Frame_Time_pre = 0;//ǰһ֡ʱ���

uint16  Mid_Point_Cal = 0;
///��·����ʼ���루��Լ�������
uint16  Road_StartX = 0 ;
///��·�Ľ������루��Լ�������
uint16  Road_EndX = 0 ;
uint16 Laser_Height = 0 ;


/************************************************************************/
/* //����ʶ���� ��ȡ������Ϣ �洢���ṹ����                             */
/************************************************************************/
void AxleRecongnize(VehicleSet *pVehicle)
{
	g_FrameCnt=pVehicle->u16FrameCnt;
	g_Frame_Time=pVehicle->CurFrmInfo.u32TimeStamp;
	get_veh_info_frame(g_FiltX,g_FiltZ,g_CoorX,g_CoorZ);
	pre_process();
	judge_tyre();
	CheckAxleState(pVehicle);
}


int get_veh_info_frame(int32 *pFiltX,int32 *pFiltZ,int32 *pX,int32 *pZ)
{
	unsigned short int l_u16data_start = 1;
	unsigned short int l_u16data_start1 = 1;
	unsigned short int l_u16data_start2 = 1;
	int min_dis = 65535;
	int c =0;
//	unsigned short int min_dis_index = 0;
	unsigned short int index = 0;
	unsigned char continue_flag = 0;
	unsigned short int height_max_Xmin = 0;
	unsigned short int height_max_Xmax = 0;
	int cur_max_height = 0;
	unsigned short int l_u16Index2 = 0;
	int sort_data = 0;
	int a =0;
	int available_point = 0;
	unsigned short int height_max = 0;
	memset(point_below_height,0,sizeof(point_below_height));
	memset(min_dis_X_t,0,sizeof(min_dis_X_t));
	point_count = 0;
	if ( check_axle ==2 )//��������
	{
		clear_global();
	}

	for( index = Mid_Point_Cal; index > l_u16data_start; index--)
	{
		if( pFiltZ[index] > 350 && l_u16data_start1 == 1
			&& pFiltX[index] > Road_StartX
			&& pFiltX[index] < Road_EndX)
		{
			l_u16data_start1 = index;
		}
		if(pFiltZ[index] > 520 && l_u16data_start2==1 &&
			pFiltX[index] > Road_StartX &&
			pFiltX[index] < Road_EndX)
		{
			l_u16data_start2 = index;
		}
		if(pFiltZ[index] > 500  && pFiltZ[index] > 
			height_max && pFiltX[index] > Road_StartX && 
			pFiltX[index] < Road_EndX)
			height_max = pFiltZ[index];
	}


	min_dis_X_t[6] =g_Frame_Time;
	if( g_FrameCnt > 0)
	{
		if( (min_dis_X_t[6]- g_Frame_Time_pre) > 16 &&
			(min_dis_X_t[6]- g_Frame_Time_pre ) < 24 )
		{
			min_dis_X_t[7] = 1;
		}
		else
		{
			min_dis_X_t[7] =  (min_dis_X_t[6]- g_Frame_Time_pre) / 20;
		}
	}
	else
	{
		min_dis_X_t[7] = 1;
	}
	g_Frame_Time_pre = g_Frame_Time;
	available_point = 0;
	cur_max_height = 0;
	continue_flag = 0;
	height_max_Xmin = Road_EndX;
	height_max_Xmax = Road_StartX;
	for( index = Mid_Point_Cal; index >= l_u16data_start; index--)//�������е�
	{
		if(pZ[index] > 500 && ABS(height_max-pFiltZ[index]) < 100 &&
			pX[index] > Road_StartX && pX[index] < Road_EndX)
		{
			if(height_max_Xmin > pX[index])
				height_max_Xmin = pX[index];
			if(height_max_Xmax < pX[index])
				height_max_Xmax = pX[index];
		}

		if(  (pFiltX[index] > 200 && pFiltX[index]<=Road_StartX && pFiltZ[index] > SafeIsland_Height && pFiltZ[index] < 4800 )//��ȫ���ϵ���Ч�� ��դ����ʱ�����˻򳬿�Ӱ�졣			
			|| (pFiltX[index] > Road_StartX && pFiltX[index]< Road_EndX && pFiltZ[index] > 150 && pFiltZ[index] < 4800 ) )//������Χ�ڵ� ��Ч����
			available_point += 1;


		if( pFiltX[index] > Road_StartX && pFiltX[index]< Road_EndX)
		{

			if( index > l_u16data_start1)
			{
				if( pFiltZ[index] > 100)
				{
					if( pFiltZ[index-1] > 100 || pFiltZ[index] > 130)
						continue_flag = 1;
				}
				c =  pFiltX[index]* (-45)/100 + pFiltZ[index];

				if(c < min_dis && continue_flag == 0)
				{
					min_dis = c;
//					min_dis_index = index;

					min_dis_X_t[1] = pFiltX[index];
					min_dis_X_t[2] = pFiltZ[index];
					min_dis_X_t[5] = index;
				}
			}

			if(index > l_u16data_start2)
			{
				if(pFiltZ[index] < 520 && pFiltZ[index] > 100)
				{
					if(pFiltZ[index] > cur_max_height + 5)
					{
						cur_max_height = pFiltZ[index];
						point_count+=1;
						point_below_height[point_count] = pFiltX[index];
						l_u16Index2 = point_count;
						sort_data = point_below_height[point_count];
						while( l_u16Index2 > 1 && sort_data > point_below_height[l_u16Index2-1])
						{
							a = point_below_height[l_u16Index2 - 1];
							point_below_height[l_u16Index2-1] = point_below_height[l_u16Index2];
							point_below_height[l_u16Index2] = a;
							l_u16Index2 = l_u16Index2-1;
						}

					}
				}
			}
		}
	}
	if( height_max > 700 && height_max < 1350 && (height_max_Xmax-height_max_Xmin) > 400)//��¼ÿһ֡�еĸ߶���Ϣ���ж�С��
		min_dis_X_t[8] = height_max;
	else
		min_dis_X_t[8] = -1 * height_max;

	beyond_axle_ground = 0;
	for( index = Mid_Point_Cal; index >= l_u16data_start; index--)//�������е�
	{
		if ( pFiltX[index] > min_dis_X_t[1] + 200 && pFiltZ[index] <150 && pFiltX[index] < min_dis_X_t[1] + 2000 && pFiltX[index] < Road_EndX  )
		{
			beyond_axle_ground = beyond_axle_ground+1;
		}
	}
	min_dis_X_t[10] = beyond_axle_ground;

	if (min_dis_X_t[1] > ground_car)
	{
		ground_over_count = ground_over_count+1;
		if (ground_over_count >10 && car_height ==1)
			car_height = 0;
	}

	if ( ABS(min_dis_X_t[8] ) > 2350 )
	{
		height_over_count++;
		if (height_over_count >3 && car_height ==1 )
		{
			car_height = 0;
		}
	}

	if (available_point > 1 )//�����д��ڳ���//�����������õ���ֽ�����жϣ��˷�������ȫ������
	{
		if (vehicle_new == 0 )
			vehicle_new = 1;
		empty_frame_count = 0;
	}
	else
	{
		if(empty_frame_count < 0x0FFFFFFF )
			empty_frame_count++;
		else
			empty_frame_count = 0x0FFFFFFF;
	}


//	if (vehicle_new == 1)//����֡���ۼ�
//	{
//		g_FrameCnt++;
//	}

	return 0;
}


int pre_process()
{
	unsigned short int index = 0;
	unsigned short int index_set = 0;
//	unsigned short int index1 = 0;
//	unsigned short int index2 = 0;
	unsigned short int height_count = 0;
	unsigned short int beyond_DOUBLE_AXLE = 0;
	over_Flag = 0;
	check_axle = 0;
	memset( belowHeigh_X_t , 0 ,sizeof(belowHeigh_X_t) );
	belowHeigh_X_t[1] = Road_StartX;
	if(point_count>2)
	{
		belowHeigh_X_t[1] = point_below_height[1];
		belowHeigh_X_t[2] = point_below_height[point_count];
		belowHeigh_X_t[3] = belowHeigh_X_t[1] - belowHeigh_X_t[2];

		for(index = 1; index <=point_count;index++ )
		{
			if( point_below_height[index] -  point_below_height[point_count] > DOUBLE_AXLE )
				beyond_DOUBLE_AXLE= beyond_DOUBLE_AXLE+1;

		}	

		if( beyond_DOUBLE_AXLE > 4 && point_below_height[1] -  min_dis_X_t[1] > 100 )
			belowHeigh_X_t[3]  =  belowHeigh_X_t[1]  -  belowHeigh_X_t[2]  ;
		else
			belowHeigh_X_t[3]  =   belowHeigh_X_t[2] - belowHeigh_X_t[1]  ;

		//	belowHeigh_X_t[4]  =  beyond_DOUBLE_AXLE;
	}

	if (g_FrameCnt == 1)
	{
		neighout_max_X =  min_dis_X_t[1];
		neighout_min_X =  min_dis_X_t[1] ;
		index_extract = 1;
		search_count_add = 0;
		index_top = 1;
		for( index = 1; index <= 10; index++)
			min_dis_X[index][index_extract] = min_dis_X_t[index];
		min_dis_X[3][index_extract] = min_dis_X[1][index_extract];
		search_count = 1;
		return 0;
	}
	//Ѱ���ڽ���ļ���Сֵ
	if(min_dis_X_t[1] > neighout_max_X)
		neighout_max_X = min_dis_X_t[1] ;

	if( min_dis_X_t[1] < neighout_min_X)
		neighout_min_X = min_dis_X_t[1] ;

	search_count = search_count+1;//����֡���ۼ�

	//������֮֡�� ����С��֡���кϲ�����
	if( neighout_max_X - neighout_min_X > 45)
	{
		if (axle_count >0)
		{
			if (neighout_max_X - neighout_min_X > 60 + (neighout_min_X - axle_index[axle_count][4]) * 15/1000)//������������
				memset(near_point_count, 0, sizeof(near_point_count));
			else if (near_point_count[min_dis_X_t[5]] > 9)//�ֽ�� Ϊ��ͬ���֡������ֵ ��֡������
				return RET_TRUE;
		}
		else
		{
			if (neighout_max_X - neighout_min_X > 60)//������������
				memset(near_point_count, 0, sizeof(near_point_count));
			else if (near_point_count[min_dis_X_t[5]] > 9)//�ֽ�� Ϊ��ͬ���֡������ֵ ��֡������
				return RET_TRUE;
		}


		index_extract = index_extract+1;
		search_count_add = search_count_add+1;
		for(index = 1; index <= 10; index++)
			min_dis_X[index][index_extract] = min_dis_X_t[index];
		min_dis_X[9][index_extract] = g_FrameCnt;//����ԭ֡���
		for( index = 1;  index<= 3; index++)
			belowHeigh_X[index_extract][index] = belowHeigh_X_t[index];
		min_dis_X[3][index_extract] = min_dis_X[1][index_extract];

		if (search_count == 2 || search_count_add == 1) //���� ֡
		{
			min_dis_X[3][index_extract] = min_dis_X[1][index_extract];

			if (index_extract>=2 && min_dis_X[3][index_extract]< min_dis_X[3][index_extract-1] && min_dis_X[3][index_extract-2]  < min_dis_X[3][index_extract-1]
			&&ABS(axle_min_X -  min_dis_X[3][index_extract-1] ) > 65*min_dis_X[3][index_top] / (  Laser_Height -65 ) )
				over_Flag = 1;//�������Ƹı�

			if(  search_count > 2 )  //search_count_add == 1 ��ʱ��neighout_min_X���ܲ��Ǳ����min_dis_X�е�����
			{
				if(  min_dis_X[1][index_extract] > min_dis_X[1][index_extract-1]  )
					neighout_min_X = min_dis_X[1][index_extract-1];
				else
					neighout_min_X = min_dis_X[1][index_extract];
			}	

			if(axle_min_X> neighout_min_X) //���¼�ֵ
			{
				axle_min_X = neighout_min_X; //���¼�ֵ
				if (min_dis_X[3][index_extract] < min_dis_X[3][index_extract-1])
					index_top = index_extract;
				else
					index_top = index_extract-1;
			}
		}
		else
		{

			index_set =  index_extract - search_count_add;
			for(index= index_extract - search_count_add+1; index <= index_extract-1; index++ ) //������������
			{
				min_dis_X[3][index] =    min_dis_X[3][index_extract - search_count_add];
				if( min_dis_X[5][index] > min_dis_X[5][index_set] )
					index_set = index;
			}


			if (min_dis_X[3][index_extract-1] >  min_dis_X[3][index_extract])   //���¾ֲ���ֵ
			{
				neighout_max_X = min_dis_X[3][index_extract-1]; //%���¾ֲ���ֵ
				neighout_min_X = min_dis_X[3][index_extract] ;


				if(ABS(axle_min_X -  min_dis_X[3][index_extract-1] ) > 65*min_dis_X[3][index_top] / (  Laser_Height -65 )) 
					over_Flag = 1;
				else
				{
					index_top = index_extract;
					axle_min_X = neighout_min_X ;//%���¼���ֵ
				}	
			}	
			else
			{
				neighout_max_X = min_dis_X[3][index_extract] ;// %���¾ֲ���ֵ
				neighout_min_X = min_dis_X[3][index_extract-1] ;
				if (axle_min_X >= neighout_min_X) //%���¼���ֵ
				{
					index_top = index_set;
					axle_min_X = neighout_min_X ;//���¼���ֵ
				}
			}
		}

		search_count_add = 0;
		check_axle = 1;
		car_height_temp = 0;
		height_count = 0;
		if (height_over_count <= 1 && axle_count == 0  && ( (Laser_Height > 1700)  || ( Laser_Height > 1100 && Mid_Point_Cal > 280 ) )  )
		{
			for(index = 2; index <= index_extract; index++)
			{
				if(index >= index_top-2  && index <= index_top+2 && (min_dis_X[8][index]<1350 && min_dis_X[8][index]>600))
					car_height_temp = 1;

				if ((ABS(min_dis_X[8][index]) > 1350 || (ABS(min_dis_X[8][index]) < 300 && ABS(min_dis_X[8][index]) > 0)) && index <= index_top)
					height_count = height_count +1; 
			}
			if(height_count == 0) 
				car_height_temp = 1;
		}



	}
	else
	{
		if ( near_point_count[min_dis_X_t[5]] <=9 )
		{
			near_point_count[min_dis_X_t[5]] =  near_point_count[min_dis_X_t[5]] +1;
			index_extract = index_extract + 1;
			search_count_add = search_count_add+1;
			for(index = 1; index <= 10; index++)
				min_dis_X[index][index_extract] = min_dis_X_t[index];
			min_dis_X[9][index_extract] = g_FrameCnt;
			for(index = 1; index <= 3; index++)
				belowHeigh_X[index_extract][index] = belowHeigh_X_t[index];
		}
	}
	if (neighout_max_X - neighout_min_X < 45 &&
		index_extract >= 2 && min_dis_X[1][index_extract]<= min_dis_X[1][index_extract - 1] && min_dis_X[1][index_extract - 2]  + 150 < min_dis_X[1][index_extract - 1]
	&& ABS(axle_min_X - min_dis_X[1][index_extract - 1]) > 65 * min_dis_X[1][index_top] / (Laser_Height - 65) 		)
	{
		over_Flag = 1;//�������Ƹı�
		check_axle = 1;
		min_dis_X[3][index_extract] = min_dis_X[1][index_extract];
	}


	if (index_extract>=MAX_FRAME-1 )//��������֡���������ж�
	{
		check_axle = 2;
		car_height_temp =1;//��������Ҫ��
	}

	if (check_axle)
	{
		MAX_Height = 320*min_dis_X[3][index_top] / (  Laser_Height -320 );
		if( car_height_temp || car_height == 1)
			MAX_Height = 140*min_dis_X[3][index_top] / ( Laser_Height - 140);
		MAX_Height2 = 35*min_dis_X[3][index_top] / (  Laser_Height -35 );
	}
	return RET_TRUE;
}

int judge_tyre()
{
	unsigned char	change_Flag = 0;
	unsigned char	continuity_end = 0;//��̥��������Բ��ɣ� �������εĲ���֡�����ԣ��ж��Ƿ�֡
	unsigned char	continuity_start = 0;//��̥��������Բ��ɣ� ��ʼ���εĲ���֡�����ԣ��ж��Ƿ�֡
	unsigned char	fake_axle = 0;//�����־

	unsigned char	arc_end = 0;//��̥��������Բ��ɣ� ��������
	unsigned char	arc_start = 0;//��ʼ����
	unsigned char  miss_axle= 0; //�󳵿���ͨ��ֻ��5֡������£�����������������
	unsigned short int index = 0;
	unsigned short int	temp_count = 0;//��̥��������Բ��ɣ� ��Բ����ƽ�����ֽ׶Ρ�
	unsigned short int	temp_count2 = 0;//��̥��������Բ��ɣ� ��Բ����ƽ�����ֽ׶Ρ�
	unsigned short int	index_start_temp = 2;
	unsigned short int	close_count = 0;
	unsigned short int	belowHeigh_X_count = 0;//�߶���ֵ�µĵ���
	unsigned short int	belowHeigh_X_max = 0;
	unsigned short int	belowHeigh_X_min = 0;
	unsigned short int  index_max_diff = index_extract;
	unsigned short int	index_start = index_top;
	unsigned short int	index_end = index_top;
	unsigned short int	index_axle_end = index_extract-1;
	unsigned short int	index_axle_start = 1;
	int	second_axle_min_X_end = axle_min_X ;//%�ڶ�����ֵ
	int	second_axle_min_X_start = axle_min_X ;//%�ڶ�����ֵ
	unsigned short int height_count = 0;
	unsigned short int last_axle_frame_count = 0;//��һ�����֡��
	unsigned short int index1 = index_top;
	unsigned short int index2 = index_top;
	short int	max_diff_count = 0;
	short int	max_diff_count2 = 0;
	int	belowHeigh_X_avg = 0;//�߶���ֵ�µĵ��X��ֵ
	int	belowHeigh_width_max = 0;//�߶���ֵ�µĵ��X���ֵ
	int t_car = 0;
	int beyond_axle_ground_max = 0;
	int min_dis_tmep = 0;
	unsigned char	car_axle =0; //С�� �����ж�
	//int t_car2 = 0;
	t_car2 = 0;


	if(check_axle >= 1)
	{

		if(  index_top >1 &&  index_extract-2 >=index_top  )
		{
			max_diff_count =  ABS( min_dis_X[1][index_extract] -min_dis_X[1][index_extract-1] );
			for( index = index_extract-1 ; index >=index_top+1 ; index--)
			{
				if(  max_diff_count <  min_dis_X[1][index] -min_dis_X[1][index-1] &&    min_dis_X[1][index] -min_dis_X[1][index-1] > MAX_Height /2 )
				{
					max_diff_count = min_dis_X[1][index] -min_dis_X[1][index-1];
					index_max_diff = index;
				}	
			}

			if(  index_max_diff < index_extract && max_diff_count  > 1.2 * ( min_dis_X[1][index_max_diff+1] -min_dis_X[1][index_max_diff] ) ) 
			{
				change_Flag = 1;
			}	
		}	

		if( (axle_min_X <= neighout_min_X &&(  ABS( min_dis_X[1][index_top] -neighout_max_X + MAX_Height ) < MAX_Height / 6  || axle_min_X < neighout_max_X - MAX_Height  )  )
			|| ( over_Flag && axle_min_X < neighout_min_X  &&   axle_min_X < (neighout_max_X - MAX_Height*3/10) && min_dis_X[3][index_extract-1]  > min_dis_X[3][index_extract]  )
			||change_Flag
			|| ( index_extract-2 >=index_top &&  (min_dis_X[3][index_extract-1]  - axle_min_X) > MAX_Height/2 &&  (min_dis_X[3][index_extract-1] - min_dis_X[3][index_extract-2])  > MAX_Height/3 && (min_dis_X[3][index_extract] - min_dis_X[3][index_extract-1])  < (min_dis_X[3][index_extract-1] - min_dis_X[3][index_extract-2])   ) )
		{

			if (index_top < index_extract  && index_extract > 3 && index_top > 1 )  //�����ж�
			{
				for(index =index_top+1; index <= index_extract; index++ ) // %Ѱ����Ľ���λ�� �����޸ķ���Ϊ���������ж�
				{
					if(	(min_dis_X[3][index-1] > min_dis_X[3][index] && min_dis_X[3][index-1]-axle_min_X > MAX_Height/4 )
						||(min_dis_X[3][index]   - min_dis_X[3][index-1]> MAX_Height/2  && (min_dis_X[3][index-1] -axle_min_X) >MAX_Height/3 )
						|| min_dis_X[3][index]  > (axle_min_X + MAX_Height) //�����㵥���Գ���Ѱ�Ҿ���
						||( (index-2) >=index_top &&  min_dis_X[3][index-1]- min_dis_X[3][index_top]> MAX_Height*4/5 && min_dis_X[3][index-1] - min_dis_X[3][index-2] > MAX_Height/4 && 
						(min_dis_X[3][index] - min_dis_X[3][index-1]) < (min_dis_X[3][index-1] - min_dis_X[3][index-2]) )
						||  (change_Flag && index > index_max_diff ))
						break;

					if (min_dis_X[5][index-1] >= min_dis_X[5][index] )
					{
						index_axle_end = index;
					}

				}

				max_diff_count =    min_dis_X[5][index_axle_end-1] -min_dis_X[5][index_axle_end] ;
				continuity_end = 1 ;

				for(index =index_axle_end-1; index >= index_top; index--) //%������ĺ�벿��
				{
					if(min_dis_X[5][index]> min_dis_X[5][index+1])
						temp_count = temp_count +1;
					if( max_diff_count < min_dis_X[5][index] - min_dis_X[5][index+1])
						max_diff_count = min_dis_X[5][index] - min_dis_X[5][index+1] ;

					if(min_dis_X[5][index] == min_dis_X[5][index_top])
					{
						second_axle_min_X_end =min_dis_X[3][index+1];
						index_end = index;
						break;
					}
				}

				for(index = index_end+1; index<=index_axle_end+1; index++)
				{
					if ( index<=index_extract && min_dis_X[7][index] !=1 )
						continuity_end = continuity_end + min_dis_X[7][index] ;
				}


				if (temp_count >= 3)
				{
					if( max_diff_count*10 <= ( min_dis_X[5][index_top] - min_dis_X[5][index_axle_end] ) *60/7 && (min_dis_X[3][index_axle_end] - axle_min_X) > 60 && (second_axle_min_X_end - axle_min_X) < MAX_Height/4 +45 )
						arc_end =2;
				}
				else if (temp_count  ==2)
				{
					if(  (min_dis_X[3][index_axle_end] - axle_min_X) < MAX_Height && (min_dis_X[1][index_axle_end] - axle_min_X) > MAX_Height2  && (second_axle_min_X_end - axle_min_X) < MAX_Height *2/5 )
						arc_end =1;
				}

				//%Ѱ������ǰ�벿��
				index_start_temp = 2;
				if (axle_count > 0)
					index_start_temp = axle_index[axle_count][2] ;

				index_axle_start = index_top;
				min_dis_tmep = min_dis_X[3][index_axle_end]  -  min_dis_X[3][index_top]; //;%�������λ������͵�ĸ߶Ȳ�
				for( index = index_top -1; index>=index_start_temp; index--) // %Ѱ�ҵ�����ǰ�벿����ʼ��
				{

					if( (min_dis_X[3][index] < min_dis_X[3][index+1] && min_dis_X[3][index+1] > axle_min_X+MAX_Height/3  ) || (min_dis_X[3][index] - min_dis_X[3][index+1] > MAX_Height/2 &&
						min_dis_X[3][index+1]-axle_min_X > MAX_Height/3) || min_dis_X[3][index] > axle_min_X+MAX_Height || ((index+2)>=index_top && min_dis_X[1][index+1]-
						min_dis_X[1][index+2] > MAX_Height/4 && min_dis_X[1][index] - min_dis_X[1][index+1]<min_dis_X[1][index+1]-min_dis_X[1][index+2] ) 
						/* ||  (change_Flag && min_dis_X[1][index]  > min_dis_X[1][index_max_diff] )*/)
						break;
					if( ABS(min_dis_X[3][index] -  min_dis_X[3][index_axle_end]) < min_dis_tmep+MAX_Height )
					{
						index_axle_start = index;
						min_dis_tmep = ABS(min_dis_X[3][index]  -  min_dis_X[3][index_axle_end]) ;
					}
				}
				max_diff_count2 =   min_dis_X[5][index_axle_start+1]-min_dis_X[5][index_axle_start]  ;
				continuity_start =  1 ;
				if (index_axle_start < index_top)
				{
					for(index = index_axle_start; index <=  index_top-1; index++ )
					{
						if ( min_dis_X[5][index]  < min_dis_X[5][index+1] ) //��������
							temp_count2 = temp_count2 +1;
						if ( max_diff_count2 < (min_dis_X[5][index+1] - min_dis_X[5][index]))
							max_diff_count2 =min_dis_X[5][index+1] - min_dis_X[5][index] ;

						if( min_dis_X[5][index+1] == min_dis_X[5][index_top])
						{
							second_axle_min_X_start = min_dis_X[3][index] ;
							index_start = index+1;
							break;
						}
					}
				}


				for( index = index_axle_start-1; index <=index_start; index++ )
				{
					if (min_dis_X[7][index]!=1)
						continuity_start = continuity_start + min_dis_X[7][index] ;
				}


				if(temp_count2 >= 3)
				{

					if(max_diff_count2*10 <= ( min_dis_X[5][index_top] - min_dis_X[5][index_axle_start]) *60/7&&  (second_axle_min_X_start - min_dis_X[3][index_start]) < (MAX_Height/4+45))
						arc_start =2;
				}
				else if(temp_count2  ==2 )
				{
					if( min_dis_X[3][index_axle_start] - axle_min_X > MAX_Height2 && min_dis_X[3][index_axle_start] - axle_min_X < MAX_Height &&  second_axle_min_X_start - min_dis_X[3][index_start]< MAX_Height/2)
						arc_start =1;
				}

				if ( (car_height_temp || car_height))
					car_axle = 1;

				for(index = index_axle_start; index<= index_axle_end; index++ )
				{
					if(  min_dis_X[5][index_top] - min_dis_X[5][index] < 2)
						close_count = close_count +1;
				}

				if (  continuity_start == 1&&  continuity_end == 1
					&&  temp_count2<=3 && temp_count <=3 && ( (temp_count2>0&&temp_count>0)||(car_height&&temp_count>=0&&temp_count2>=0) ) && temp_count+temp_count2<=4
					&&	close_count*10 >= (index_axle_end-index_axle_start+1)*10/3 && index_axle_end-index_axle_start>=3 && index_axle_end-index_axle_start<=4
					&& second_axle_min_X_start-axle_min_X<MAX_Height/3 && second_axle_min_X_end-axle_min_X<MAX_Height/3
					&&(min_dis_X[3][index_axle_start]-axle_min_X>MAX_Height/8 || min_dis_X[3][index_axle_end]-axle_min_X>MAX_Height/8 ))
				{
					car_axle = 1;
				}

				if(	car_axle == 1)
				{
					beyond_axle_ground = 0;

					for (index =index_axle_start; index <=index_axle_end; index ++ )
					{
						t_car+= belowHeigh_X[index][2] ;
						beyond_axle_ground = beyond_axle_ground +min_dis_X[10][index];
						if ( beyond_axle_ground_max < min_dis_X[10][index] )
						{ 
							beyond_axle_ground_max = min_dis_X[10][index];
						}
					}
					t_car = t_car / ( index_axle_end -index_axle_start+1 );

					for (index =index_axle_start; index <=index_axle_end ; index++ )
					{
						t_car2 =t_car2 + ABS( belowHeigh_X[index][2]-t_car );
					}

					t_car2 = t_car2 *10/ (index_axle_end - index_axle_start+1); //%С���֣�������̥ƽ����
					beyond_axle_ground = beyond_axle_ground *10/ (index_axle_end - index_axle_start+1) ;
					if(index_axle_start < index_top)
					{
						for( index1 = index_axle_start+1; index1 <=index_top; index1++ )
						{
							if (min_dis_X[3][index1] > min_dis_X[3][index1-1])
								break;
							if(index1 == index_top)
								break;
						}
					}


					if (index_axle_end >index_top)
					{
						for( index2=index_axle_end-1; index2 >= index_top; index2--)
						{
							if (min_dis_X[3][index2] > min_dis_X[3][index2+1] )
								break;
							if(index2 == index_top)
								break;
						}
					}


					if ( /*t_car2 < 800 &&t_car2 > 10  &&*/  beyond_axle_ground < 30 && beyond_axle_ground_max < 6  &&  index1 == index2 && index1!=0 ) //%С���֣���������������
					{

					}
					else
						car_axle = 0;
				}


				if( car_axle ==0 && car_height==0 && index_axle_end - index_axle_start == 4 && temp_count +temp_count2 <=4 && temp_count +temp_count2 >=3
					&& ABS( min_dis_X[3][index_axle_end]  - min_dis_X[3][index_axle_start] ) < MAX_Height/4 )  
					miss_axle = 1;
				//%&&  Ѱ�ҵ������벿�� ��Ҫ���㵥���ԣ����ڼ�ֵ�ľ����ֵС��200�����ٶ��йأ��������ݼ�ֵ����ȷ���ٶȣ�

				if (( arc_end >= 1 && arc_start >= 1 )//%������������
					|| ( arc_end == 2 && continuity_start!=1 ) || ( arc_start == 2 && continuity_end != 1  )
					||( arc_end >= 1  &&  continuity_start !=1   )||   (  arc_start >= 1 && continuity_end != 1   )
					|| car_axle || miss_axle)
					//	%%Ѱ�ҵ�����
				{
					belowHeigh_X_avg = 0;
					belowHeigh_width_max = 0;
					belowHeigh_X_count = 0;; //�߶���ֵ�µĵ���
					belowHeigh_X_max = Road_StartX;
					belowHeigh_X_min = Road_EndX;
					for(index =index_axle_start; index <=index_axle_end; index++ )
					{
						if (min_dis_X[1][index] < (axle_min_X+MAX_Height/5) && belowHeigh_X[index][2]>0 )  //%���㼫ֵ�㴦 600mm���µ����������������,��̥�����
						{
							belowHeigh_X_avg = belowHeigh_X_avg+ belowHeigh_X[index][2] ;
							belowHeigh_X_count =belowHeigh_X_count+1;
							if ( belowHeigh_X[index][2] > belowHeigh_X_max )
								belowHeigh_X_max = belowHeigh_X[index][2];

							if (belowHeigh_X[index][2]< belowHeigh_X_min)
								belowHeigh_X_min = belowHeigh_X[index][2];

							if ( belowHeigh_width_max < belowHeigh_X[index][3] )
							{
								belowHeigh_width_max = belowHeigh_X[index][3];
							}
							/*	belowHeigh_width_max =belowHeigh_width_max + belowHeigh_X[index][3];*/
						}
					}

					if (belowHeigh_X_count >= (index_axle_end - index_axle_start + 1) / 10 && belowHeigh_X_count > 0)
					{
						belowHeigh_X_avg = belowHeigh_X_avg / belowHeigh_X_count;
						belowHeigh_width_max = belowHeigh_width_max; /// belowHeigh_X_count;
					}
					else
						belowHeigh_X_avg = 0;


					if ( (car_axle && (axle_min_X  <  belowHeigh_X_avg +150  && axle_min_X  >  belowHeigh_X_avg -50) ) ||   (  car_height != 1 && ( axle_min_X  <  belowHeigh_X_avg +173  && axle_min_X  >  belowHeigh_X_avg -50 ) ) ||   (  car_height == 1 && ( axle_min_X  <  belowHeigh_X_avg +145  && axle_min_X  >  belowHeigh_X_avg -50 ) )
						|| ( temp_count > 6 && temp_count2 > 6 && axle_min_X  <  belowHeigh_X_avg +240  && axle_min_X  >  belowHeigh_X_avg -50  ) )
					{
						if(axle_count)
						{
							last_axle_frame_count = axle_index[axle_count][9]-axle_index[axle_count][8];
							if( ( continuity_start==1&&continuity_end==1)&&(min_dis_X[9][index_top] - axle_index[axle_count][9])<last_axle_frame_count*3/5 /*��ǰһ�����֡�� */
								&&( ( (min_dis_X[9][index_axle_end]  -min_dis_X[9][index_axle_start])*10 <= last_axle_frame_count*13/5)  || ( axle_min_X-axle_index[axle_count][4] >90 + ( index_axle_end -axle_index[axle_count][2])*4/5 )   ) 
								&& (last_axle_frame_count <= (axle_index[axle_count][6]+axle_index[axle_count][5]) *28/10/*�Ƿ�ͣ��*/)) //�������֡���ȣ� min_dis_X �����
								fake_axle = 1;
							if ( belowHeigh_width_max > DOUBLE_AXLE &&  car_height == 0 && min_dis_X[9][index_axle_end] -  min_dis_X[9][index_axle_start] <3 )  
								fake_axle = 2;
							if ((car_height || car_axle)   && axle_min_X-axle_index[1][4] >90 + ( index_axle_end -axle_index[axle_count][2])*4/5 )//С������������֮���axle_min_X��ϵ
								fake_axle = 3;
							if ((car_height || car_axle)  && (index_top - axle_index[axle_count][2]  ) < (axle_index[axle_count][5] +axle_index[axle_count][6] )*16/10    ) //С������������ܽ� ����
								fake_axle = 6;
							if (car_height == 0   && axle_min_X > axle_index[axle_count][4] + 200 && (min_dis_X[9][index_top] - axle_index[axle_count][9])<last_axle_frame_count*7/5 && (last_axle_frame_count <= (axle_index[axle_count][6]+axle_index[axle_count][5]) *35/10) )
								fake_axle = 4;
							if (ABS( axle_index[axle_count][4] - axle_min_X ) > 800 )//��ǰһ���ƫ��
								fake_axle = 5;
							//if ((last_axle_frame_count > (axle_index[axle_count][6]+axle_index[axle_count][5]) *28/10/*�Ƿ�ͣ��*/) && ((axle_index[axle_count][6]+axle_index[axle_count][5])*4 + axle_index[axle_count][9])>min_dis_X[9][index_axle_end] && ( axle_index[axle_count][5]+axle_index[axle_count][6])/3 > ( temp_count+temp_count2) ) //ǰ�Ỻ�����л�ͣ��������Ͻ�������ж�
							////	fake_axle = 6;
							if( car_height ==0 && belowHeigh_X_count  < 2 &&  index_axle_end - index_axle_start > 4 )
								fake_axle = 7;
							if (miss_axle  && ABS( axle_index[axle_count][4] - axle_min_X ) > 200 ) //ͨ�������Ŀ����˶����������ֲ���λ�ƺܴ�
								fake_axle = 8;

						}
						if (belowHeigh_X_max - belowHeigh_X_min > 150) // ���ֵ�ƽ����
							fake_axle = 9;

						if(fake_axle == 0)
						{
							if (axle_count > MAX_AXLE_COUNT-1 )
							{
								axle_count = MAX_AXLE_COUNT-1 ;
							}
							axle_count = axle_count+1;
							axle_index[axle_count][1] = index_axle_start;//������ʼ
							axle_index[axle_count][2] = index_axle_end;//���ֽ���
							axle_index[axle_count][4] = axle_min_X;//����ͶӰ�������
							axle_index[axle_count][5] = temp_count;
							axle_index[axle_count][6] = temp_count2;
							axle_index[axle_count][8] = min_dis_X[9][index_axle_start];
							axle_index[axle_count][9] = min_dis_X[9][index_axle_end];
							axle_index[axle_count][10] = belowHeigh_width_max;
							axle_index[axle_count][0] = g_FrameCnt;
							if( continuity_start == 1 && continuity_end == 1)
								axle_index[axle_count][7] = 1;
							else
								axle_index[axle_count][7] = 0;

							car_height_temp = 0;
							height_count = 0;
							if(height_over_count <= 1 && axle_count == 1  && ( (Laser_Height > 1700)  || ( Laser_Height > 1100 && Mid_Point_Cal > 280 ) )  )
							{
								for(index = 2; index <= index_extract; index++)
								{
									if(index >= index_axle_start && index <= index_axle_end && (min_dis_X[8][index]<1350 && min_dis_X[8][index]>600))
										car_height_temp = 1;

									if (((ABS(min_dis_X[8][index]) > 1350) || (ABS(min_dis_X[8][index]) < 300 && ABS(min_dis_X[8][index]) > 0)) && index <= index_top)
										height_count = height_count +1; 
								}

								if(car_height_temp || height_count ==0 )
									car_height  = 1;
							}
							if ( axle_count == 1)
							{
								ground_car =  axle_index[axle_count][4]+ 420*axle_index[axle_count][4] / ( Laser_Height -420 );
								ground_over_count = 0;
							}


							if(belowHeigh_width_max< DOUBLE_AXLE /*||(car_height &&  belowHeigh_width_max <DOUBLE_AXLE*175/100 ) */  ) 
								axle_index[axle_count][3] = 1;
							else
								axle_index[axle_count][3] = 2;
							axle_index[axle_count][11] = axle_speed(axle_count);

							//�ж�����
							axle_index[axle_count][12] = axle_group(axle_count);
						}

						for( index = index_axle_start ; index <index_axle_end; index++ )
						{
							min_dis_X[4][index] = min_dis_X[3][index] ;
						}
					}
				}


				index_top = index_extract;
				neighout_min_X = neighout_max_X;
				axle_min_X = neighout_min_X ;//���¼�ֵ
			}
			else //%�����ж�
			{
				index_top = index_extract;
				neighout_min_X = neighout_max_X;
				axle_min_X = neighout_min_X ;//%���¼�ֵ
			}
		}

		neighout_max_X = min_dis_X[3][index_extract]; //%����Ѱ����ȥ
		neighout_min_X = min_dis_X[3][index_extract];
		search_count = 1;
	}
	else
	{

	}

	if (over_Flag)
	{
		over_Flag = 0;
		index_top = index_extract;
		axle_min_X = neighout_min_X ;//%���¼���ֵ
	}
	return 0;
}

int	axle_group(int axle_present)
{
	unsigned char axle_group_flag = 0;//�����־
//	unsigned char tyre_type_pre = 0;
	unsigned char tyre_type = 0;
	unsigned short int	index_axle_end_pre = 0;//ǰһ�������
//	unsigned short int	index_axle_start_pre = 0;//ǰһ����ʼ��
	unsigned short int	index_axle_end = 0;//��ǰ�������
	unsigned short int	index_axle_start = 0;//��ǰ����ʼ��
//	int interval_frame_count = 0; //����֮�䴩���֡
//	int ground_edge = 0;//�����Ե�ڵ����Ͷ���X����
//	int index_interval = 0;
	int	diff = 0;
	int	ratio  = 0;
	if ( axle_present <2 )
		return 0;
//	tyre_type_pre = axle_index[axle_present-1][3];
	tyre_type = axle_index[axle_present][3];
	index_axle_end_pre = axle_index[axle_present-1][2];
//	index_axle_start_pre = axle_index[axle_present-1][1];
	index_axle_end = axle_index[axle_present][2];
	index_axle_start = axle_index[axle_present][1];

#define ratio_threshold 17
	diff= index_axle_end - index_axle_end_pre;
	ratio = diff * 10 / (index_axle_end - index_axle_start + 1);
	if (ratio < ratio_threshold)
	{
		axle_group_flag = 1;
	}
	if (tyre_type == 1 && axle_present == 2 && ratio < ratio_threshold && ratio > 12) //�Եڶ�����Ϊ��̥��ǿ������Ϊ������
	{
		axle_group_flag = 0;
	}


	//	//���������Ե�ڵ����Ͷ���
	//		ground_edge  = min_dis_X[3][index_top] + 520*(min_dis_X[3][index_top]  )/ (  Laser_Height -520 );;
	//
	//while (index_axle_end_pre <index_axle_start &&  min_dis_X[1][index_axle_end_pre] <  ground_edge )
	//	index_axle_end_pre++;
	//while (index_axle_start > index_axle_end_pre  &&  min_dis_X[1][index_axle_start] <  ground_edge && index_axle_end_pre > 1 )
	//	index_axle_start -- ;
	//
	//	for (index_interval = index_axle_end_pre ; index_interval <index_axle_start ;index_interval++  )
	//	{
	//		if ( min_dis_X[1][index_interval] < ground_edge   )
	//		{
	//			interval_frame_count = interval_frame_count + 1 ;
	//		}
	//	}
	//	if (interval_frame_count <1 )
	//		axle_group_flag = 1;

	return axle_group_flag;

}

int32 axle_speed(int axle_present)
{
	int32 speed = 0;
	unsigned short int	index_axle_end = 0;
	unsigned short int	index_axle_start = 0;
	int vehicle_diameter = 0;//������ֱ��
	int diff_diameter = 0;//��ظ߶���ֱ���Ĳ�ֵ
	int axle_below_height_end = 0;
	int axle_below_height_start = 0;
	int axle_below_height_max = 0;
	int	move_length_start = 0;
	int	move_length_end = 0;
	int frame_count_axle =  axle_index[axle_present][9] - axle_index[axle_present][8]+1;
	index_axle_end = axle_index[axle_present][2];
	index_axle_start = axle_index[axle_present][1];

	axle_below_height_end	=    Laser_Height * (min_dis_X[1][index_axle_end] -axle_min_X) / min_dis_X[1][index_axle_end];
	axle_below_height_start =    Laser_Height * (min_dis_X[1][index_axle_start] -axle_min_X) / min_dis_X[1][index_axle_start];
	axle_below_height_max = axle_below_height_end;//���������ߵ㣬������ֱ̥��
	if (axle_below_height_start > axle_below_height_max )
		axle_below_height_max = axle_below_height_start;
	if (car_axle || car_height)
		vehicle_diameter = 420;
	else
		vehicle_diameter = 600;
	diff_diameter = vehicle_diameter - axle_below_height_end ;
	move_length_start =(int) sqrt((double)(vehicle_diameter *vehicle_diameter - diff_diameter *diff_diameter));
	diff_diameter = vehicle_diameter - axle_below_height_start ;
	move_length_end = (int) sqrt((double)(vehicle_diameter *vehicle_diameter - diff_diameter *diff_diameter));

	speed = 180*(move_length_end + move_length_start ) / (frame_count_axle *100);

	return speed;
}

//���������ȫ�ֱ���
int	clear_global()
{
	axle_count = 0;
	axle_count_record = 0;
	memset(axle_index,0,sizeof(axle_index));
	memset(min_dis_X,0,sizeof(min_dis_X));
	memset(belowHeigh_X,0,sizeof(belowHeigh_X));
	g_FrameCnt = 0;
	index_extract = 0;
	search_count_add = 0;
	car_height =0;
	height_over_count = 0;
	empty_frame_count = 0;
	vehicle_new = 0;
	check_axle = 0;
	//#ifdef VC_DUBUG2
	//	memset(min_dis_X_t2,0,sizeof(min_dis_X_t2));
	//	memset(belowHeigh_X_t2,0,sizeof(belowHeigh_X_t2));
	//#endif
	return 0;
}

/************************************************************************/
/* ����������Ҫ�Ĳ���                                                   */
/************************************************************************/
void SetAxleParam()
{
	Laser_Height=Smp.JGHeight;
	Road_StartX=Smp.StartX;
	Road_EndX=Smp.StartX+Smp.LaneWidth;
}


/************************************************************************/
/* ��鵱ǰ���Ƿ���β                                                   */
/************************************************************************/
void CheckAxleState(VehicleSet *pVehicle)
{
	if (axle_count != axle_count_record )
	{
		///axle_index[0] ���ֳ���֡ ԭʼ֡���
		///axle_index[1]//������ʼ֡���
		///axle_index[2]//�������֡���
		///axle_index[3]//��������
		///axle_index[4]//����ͶӰ���x
		///axle_index[5]//������벿�ֱ仯��
		///axle_index[6]//�����Ұ벿�ֱ仯��
		///axle_index[8]//������ʼ֡ ԭʼ֡���
		///axle_index[9]//���ֽ���֡ ԭʼ֡���
		///axle_index[10]//�߶���ֵ�µĵ��X���ֵ
		///axle_index[11]//�����ٶ�
		///axle_index[12]//������

		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleOriIndex=axle_index[axle_count][0];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleStaIndex=axle_index[axle_count][1];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleEndIndex=axle_index[axle_count][2];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleType=axle_index[axle_count][3];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleCastX=axle_index[axle_count][4];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleLeftDelt=axle_index[axle_count][5];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleRightDlet=axle_index[axle_count][6];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleOriStaI=axle_index[axle_count][8];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleOriEndI=axle_index[axle_count][9];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleWidthMax=axle_index[axle_count][10];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleSpeed=axle_index[axle_count][11];
		pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum].AxleGroup=axle_index[axle_count][12];
		pVehicle->AxleSet.AxleSum=pVehicle->AxleSet.AxleSum+1;
		axle_count_record = axle_count;
#ifdef _KeilEnVi
		ReportAxleInfo(axle_index[axle_count][3]);
#endif	
	}
}
#ifdef _KeilEnVi
//��������ϱ�����̨
void ReportAxleInfo(uint8 AxleType)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	g_WeigthAt.u16wAckTmCnt=0;
	if(g_WeigthAt.u8wAxleReprot)
	{
		UpBuff[TmpI++]=0x40;
		UpBuff[TmpI++]=0x80;
		UpBuff[TmpI++]=0xC0;
		UpBuff[TmpI++]=AxleType;
		SendBuf_full(UpBuff,TmpI,g_WeigthAt.u8wAtChannel);
	}
}

//�ϱ�������Ϣ
void ReprotAckSInfo(void)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	
	if(g_WeigthAt.u8wAckEnable)
	{
		UpBuff[TmpI++]=0x40;
		UpBuff[TmpI++]=0x80;
		UpBuff[TmpI++]=0xC0;
		SendBuf_full(UpBuff,TmpI,g_WeigthAt.u8wAtChannel);
	}
}


#endif	

//�������ȫ�ֱ���
void AxleClearGlobal()
{
	clear_global();
}
