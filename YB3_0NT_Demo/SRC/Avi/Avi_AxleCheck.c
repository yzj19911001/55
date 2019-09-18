#include "stdafx.h"
#include "Avi_AxleCheck.h"
#include "Avi_MathProc.h"
#ifdef _KeilEnVi
#include "PrtlProc.h"
#endif

int t_car2 = 0;
uint16 car_height_temp = 0;//小车标志 临时
///min_dis_X[1]	车身与地面的分界点 距离激光器距离
///min_dis_X[2]	车身与地面的分界点 距离地面高度
///min_dis_X[3]	归并后边界距离 相差45内的合并
///min_dis_X[5]	边界点索引
///min_dis_X[6]	时间戳
///min_dis_X[7]	帧的连续性
///min_dis_X[8]	每帧的高度信息
///min_dis_X[9]	压缩帧对应的原始索引
///min_dis_X[10]超过轮轴后打到地面的点数
int32  min_dis_X_t[12] ;

///min_dis_X[1]	车身与地面的分界点 距离激光器距离
///min_dis_X[2]	车身与地面的分界点 距离地面高度
///min_dis_X[3]	归并后边界距离 相差45内的合并
///min_dis_X[5]	边界点索引
///min_dis_X[6]	时间戳
///min_dis_X[7]	帧的连续性
///min_dis_X[8]	每帧的高度信息
///min_dis_X[9]	压缩帧对应的原始索引
///min_dis_X[10]超过轮轴后打到地面的点数
int32  min_dis_X[12][FRAME_MAXCNT]={0} ;

uint16	point_below_height[POINT_SUM] = {0};
uint16 beyond_axle_ground = 0;//越过车轮扫描到地面上的点

unsigned short int point_count = 0;//有效点数
unsigned int empty_frame_count = 0;//无车帧数

unsigned int vehicle_count = 0;//车辆数
unsigned int vehicle_new = 0;//新车进入

uint32	height_over_count = 0; //车高大于2300的帧数
uint32	ground_over_count = 0;
uint32	ground_car = 0;//底盘高
uint8	car_height = 0;
unsigned char	car_axle =0;		//小车 车轮判断
uint8	check_axle = 0;
uint8	over_Flag = 0;
///高度阈值下的有效点 距离激光器的距离
int16 belowHeigh_X[FRAME_MAXCNT][4] = {0};
int16 belowHeigh_X_t[4] = {0};
uint32	g_FrameCnt = 0;//帧序号

///临近域的极大值
int32  neighout_max_X = 0;
///临近域的极小值
int32  neighout_min_X = 0;
///搜索点数
uint32 search_count = 0 ;
///极值点的索引
uint16 index_top  = 0;

uint32	index_extract = 0 ;
uint32	search_count_add = 0;

uint16 axle_count = 0;
uint16 axle_count_record = 0;
///axle_index[0] 车轮出轴帧 原始帧序号
///axle_index[1]//轮轴起始帧序号
///axle_index[2]//轮轴结束帧序号
///axle_index[3]//车轮类型
///axle_index[4]//车轮投影最近x
///axle_index[5]//车轮左半部分变化量
///axle_index[6]//车轮右半部分变化量
///axle_index[8]//车轮起始帧 原始帧序号
///axle_index[9]//车轮结束帧 原始帧序号
///axle_index[10]//高度阈值下的点的X相差值
///axle_index[11]//轮轴速度
///axle_index[12]//轴组标记

int32 axle_index[MAX_AXLE_COUNT][15] = {0};

int32 axle_min_X=65535;
///计算临近点
uint16	near_point_count[POINT_SUM] = {0};

int	MAX_Height = 0;//轮胎判断阈值

int	MAX_Height2 = 0;//小轮胎判断阈值

uint32	g_Frame_Time = 0;//帧时间戳
uint32	g_Frame_Time_pre = 0;//前一帧时间戳

uint16  Mid_Point_Cal = 0;
///道路的起始距离（相对激光器）
uint16  Road_StartX = 0 ;
///道路的结束距离（相对激光器）
uint16  Road_EndX = 0 ;
uint16 Laser_Height = 0 ;


/************************************************************************/
/* //轮轴识别器 提取轮轴信息 存储到结构体中                             */
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
	if ( check_axle ==2 )//缓存已满
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
	for( index = Mid_Point_Cal; index >= l_u16data_start; index--)//遍历所有点
	{
		if(pZ[index] > 500 && ABS(height_max-pFiltZ[index]) < 100 &&
			pX[index] > Road_StartX && pX[index] < Road_EndX)
		{
			if(height_max_Xmin > pX[index])
				height_max_Xmin = pX[index];
			if(height_max_Xmax < pX[index])
				height_max_Xmax = pX[index];
		}

		if(  (pFiltX[index] > 200 && pFiltX[index]<=Road_StartX && pFiltZ[index] > SafeIsland_Height && pFiltZ[index] < 4800 )//安全岛上的有效点 光栅功能时，行人或超宽车影响。			
			|| (pFiltX[index] > Road_StartX && pFiltX[index]< Road_EndX && pFiltZ[index] > 150 && pFiltZ[index] < 4800 ) )//车道范围内的 有效点数
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
	if( height_max > 700 && height_max < 1350 && (height_max_Xmax-height_max_Xmin) > 400)//记录每一帧中的高度信息，判断小车
		min_dis_X_t[8] = height_max;
	else
		min_dis_X_t[8] = -1 * height_max;

	beyond_axle_ground = 0;
	for( index = Mid_Point_Cal; index >= l_u16data_start; index--)//遍历所有点
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

	if (available_point > 1 )//车道中存在车辆//后续加入利用地面分界点来判断，克服车辆完全打飞情况
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


//	if (vehicle_new == 1)//车辆帧数累加
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
	//寻找邻近域的极大极小值
	if(min_dis_X_t[1] > neighout_max_X)
		neighout_max_X = min_dis_X_t[1] ;

	if( min_dis_X_t[1] < neighout_min_X)
		neighout_min_X = min_dis_X_t[1] ;

	search_count = search_count+1;//遍历帧数累加

	//对相邻帧之间 相差较小的帧进行合并处理
	if( neighout_max_X - neighout_min_X > 45)
	{
		if (axle_count >0)
		{
			if (neighout_max_X - neighout_min_X > 60 + (neighout_min_X - axle_index[axle_count][4]) * 15/1000)//计数数组清零
				memset(near_point_count, 0, sizeof(near_point_count));
			else if (near_point_count[min_dis_X_t[5]] > 9)//分界点 为相同点的帧超过阈值 该帧不处理
				return RET_TRUE;
		}
		else
		{
			if (neighout_max_X - neighout_min_X > 60)//计数数组清零
				memset(near_point_count, 0, sizeof(near_point_count));
			else if (near_point_count[min_dis_X_t[5]] > 9)//分界点 为相同点的帧超过阈值 该帧不处理
				return RET_TRUE;
		}


		index_extract = index_extract+1;
		search_count_add = search_count_add+1;
		for(index = 1; index <= 10; index++)
			min_dis_X[index][index_extract] = min_dis_X_t[index];
		min_dis_X[9][index_extract] = g_FrameCnt;//保存原帧序号
		for( index = 1;  index<= 3; index++)
			belowHeigh_X[index_extract][index] = belowHeigh_X_t[index];
		min_dis_X[3][index_extract] = min_dis_X[1][index_extract];

		if (search_count == 2 || search_count_add == 1) //相邻 帧
		{
			min_dis_X[3][index_extract] = min_dis_X[1][index_extract];

			if (index_extract>=2 && min_dis_X[3][index_extract]< min_dis_X[3][index_extract-1] && min_dis_X[3][index_extract-2]  < min_dis_X[3][index_extract-1]
			&&ABS(axle_min_X -  min_dis_X[3][index_extract-1] ) > 65*min_dis_X[3][index_top] / (  Laser_Height -65 ) )
				over_Flag = 1;//增长趋势改变

			if(  search_count > 2 )  //search_count_add == 1 此时的neighout_min_X可能不是保存的min_dis_X中的数据
			{
				if(  min_dis_X[1][index_extract] > min_dis_X[1][index_extract-1]  )
					neighout_min_X = min_dis_X[1][index_extract-1];
				else
					neighout_min_X = min_dis_X[1][index_extract];
			}	

			if(axle_min_X> neighout_min_X) //更新极值
			{
				axle_min_X = neighout_min_X; //更新极值
				if (min_dis_X[3][index_extract] < min_dis_X[3][index_extract-1])
					index_top = index_extract;
				else
					index_top = index_extract-1;
			}
		}
		else
		{

			index_set =  index_extract - search_count_add;
			for(index= index_extract - search_count_add+1; index <= index_extract-1; index++ ) //求最大的索引点
			{
				min_dis_X[3][index] =    min_dis_X[3][index_extract - search_count_add];
				if( min_dis_X[5][index] > min_dis_X[5][index_set] )
					index_set = index;
			}


			if (min_dis_X[3][index_extract-1] >  min_dis_X[3][index_extract])   //更新局部极值
			{
				neighout_max_X = min_dis_X[3][index_extract-1]; //%更新局部极值
				neighout_min_X = min_dis_X[3][index_extract] ;


				if(ABS(axle_min_X -  min_dis_X[3][index_extract-1] ) > 65*min_dis_X[3][index_top] / (  Laser_Height -65 )) 
					over_Flag = 1;
				else
				{
					index_top = index_extract;
					axle_min_X = neighout_min_X ;//%更新极大值
				}	
			}	
			else
			{
				neighout_max_X = min_dis_X[3][index_extract] ;// %更新局部极值
				neighout_min_X = min_dis_X[3][index_extract-1] ;
				if (axle_min_X >= neighout_min_X) //%更新极大值
				{
					index_top = index_set;
					axle_min_X = neighout_min_X ;//更新极大值
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
		over_Flag = 1;//增长趋势改变
		check_axle = 1;
		min_dis_X[3][index_extract] = min_dis_X[1][index_extract];
	}


	if (index_extract>=MAX_FRAME-1 )//超过保存帧数，出轴判断
	{
		check_axle = 2;
		car_height_temp =1;//降低轮轴要求
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
	unsigned char	continuity_end = 0;//轮胎由两个半圆组成， 结束弧形的波形帧连续性，判断是否丢帧
	unsigned char	continuity_start = 0;//轮胎由两个半圆组成， 起始弧形的波形帧连续性，判断是否丢帧
	unsigned char	fake_axle = 0;//错轴标志

	unsigned char	arc_end = 0;//轮胎由两个半圆组成， 结束弧形
	unsigned char	arc_start = 0;//起始弧形
	unsigned char  miss_axle= 0; //大车快速通过只有5帧的情况下，难以满足判轴条件
	unsigned short int index = 0;
	unsigned short int	temp_count = 0;//轮胎由两个半圆组成， 该圆弧不平滑，分阶段。
	unsigned short int	temp_count2 = 0;//轮胎由两个半圆组成， 该圆弧不平滑，分阶段。
	unsigned short int	index_start_temp = 2;
	unsigned short int	close_count = 0;
	unsigned short int	belowHeigh_X_count = 0;//高度阈值下的点数
	unsigned short int	belowHeigh_X_max = 0;
	unsigned short int	belowHeigh_X_min = 0;
	unsigned short int  index_max_diff = index_extract;
	unsigned short int	index_start = index_top;
	unsigned short int	index_end = index_top;
	unsigned short int	index_axle_end = index_extract-1;
	unsigned short int	index_axle_start = 1;
	int	second_axle_min_X_end = axle_min_X ;//%第二个极值
	int	second_axle_min_X_start = axle_min_X ;//%第二个极值
	unsigned short int height_count = 0;
	unsigned short int last_axle_frame_count = 0;//上一个轴的帧数
	unsigned short int index1 = index_top;
	unsigned short int index2 = index_top;
	short int	max_diff_count = 0;
	short int	max_diff_count2 = 0;
	int	belowHeigh_X_avg = 0;//高度阈值下的点的X均值
	int	belowHeigh_width_max = 0;//高度阈值下的点的X相差值
	int t_car = 0;
	int beyond_axle_ground_max = 0;
	int min_dis_tmep = 0;
	unsigned char	car_axle =0; //小车 车轮判断
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

			if (index_top < index_extract  && index_extract > 3 && index_top > 1 )  //出轴判定
			{
				for(index =index_top+1; index <= index_extract; index++ ) // %寻找轴的结束位置 后续修改反向为特征描述判断
				{
					if(	(min_dis_X[3][index-1] > min_dis_X[3][index] && min_dis_X[3][index-1]-axle_min_X > MAX_Height/4 )
						||(min_dis_X[3][index]   - min_dis_X[3][index-1]> MAX_Height/2  && (min_dis_X[3][index-1] -axle_min_X) >MAX_Height/3 )
						|| min_dis_X[3][index]  > (axle_min_X + MAX_Height) //不满足单调性超过寻找距离
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

				for(index =index_axle_end-1; index >= index_top; index--) //%遍历轴的后半部分
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

				//%寻找轮轴前半部分
				index_start_temp = 2;
				if (axle_count > 0)
					index_start_temp = axle_index[axle_count][2] ;

				index_axle_start = index_top;
				min_dis_tmep = min_dis_X[3][index_axle_end]  -  min_dis_X[3][index_top]; //;%轮轴结束位置与最低点的高度差
				for( index = index_top -1; index>=index_start_temp; index--) // %寻找到轮轴前半部分起始点
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
						if ( min_dis_X[5][index]  < min_dis_X[5][index+1] ) //单调点数
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

					t_car2 = t_car2 *10/ (index_axle_end - index_axle_start+1); //%小车轮，满足轮胎平面性
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


					if ( /*t_car2 < 800 &&t_car2 > 10  &&*/  beyond_axle_ground < 30 && beyond_axle_ground_max < 6  &&  index1 == index2 && index1!=0 ) //%小车轮，满足轮廓单调性
					{

					}
					else
						car_axle = 0;
				}


				if( car_axle ==0 && car_height==0 && index_axle_end - index_axle_start == 4 && temp_count +temp_count2 <=4 && temp_count +temp_count2 >=3
					&& ABS( min_dis_X[3][index_axle_end]  - min_dis_X[3][index_axle_start] ) < MAX_Height/4 )  
					miss_axle = 1;
				//%&&  寻找到轮轴后半部分 需要满足单调性，相邻极值的距离差值小于200（和速度有关，考虑依据极值点数确定速度）

				if (( arc_end >= 1 && arc_start >= 1 )//%两侧曲线满足
					|| ( arc_end == 2 && continuity_start!=1 ) || ( arc_start == 2 && continuity_end != 1  )
					||( arc_end >= 1  &&  continuity_start !=1   )||   (  arc_start >= 1 && continuity_end != 1   )
					|| car_axle || miss_axle)
					//	%%寻找到轮轴
				{
					belowHeigh_X_avg = 0;
					belowHeigh_width_max = 0;
					belowHeigh_X_count = 0;; //高度阈值下的点数
					belowHeigh_X_max = Road_StartX;
					belowHeigh_X_min = Road_EndX;
					for(index =index_axle_start; index <=index_axle_end; index++ )
					{
						if (min_dis_X[1][index] < (axle_min_X+MAX_Height/5) && belowHeigh_X[index][2]>0 )  //%计算极值点处 600mm以下到激光器的最近距离,轮胎最大宽度
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
							if( ( continuity_start==1&&continuity_end==1)&&(min_dis_X[9][index_top] - axle_index[axle_count][9])<last_axle_frame_count*3/5 /*与前一轴相隔帧数 */
								&&( ( (min_dis_X[9][index_axle_end]  -min_dis_X[9][index_axle_start])*10 <= last_axle_frame_count*13/5)  || ( axle_min_X-axle_index[axle_count][4] >90 + ( index_axle_end -axle_index[axle_count][2])*4/5 )   ) 
								&& (last_axle_frame_count <= (axle_index[axle_count][6]+axle_index[axle_count][5]) *28/10/*是否停车*/)) //相邻轴的帧数比， min_dis_X 距离差
								fake_axle = 1;
							if ( belowHeigh_width_max > DOUBLE_AXLE &&  car_height == 0 && min_dis_X[9][index_axle_end] -  min_dis_X[9][index_axle_start] <3 )  
								fake_axle = 2;
							if ((car_height || car_axle)   && axle_min_X-axle_index[1][4] >90 + ( index_axle_end -axle_index[axle_count][2])*4/5 )//小车的两个车轮之间的axle_min_X关系
								fake_axle = 3;
							if ((car_height || car_axle)  && (index_top - axle_index[axle_count][2]  ) < (axle_index[axle_count][5] +axle_index[axle_count][6] )*16/10    ) //小车两轴间隔距离很近 多轴
								fake_axle = 6;
							if (car_height == 0   && axle_min_X > axle_index[axle_count][4] + 200 && (min_dis_X[9][index_top] - axle_index[axle_count][9])<last_axle_frame_count*7/5 && (last_axle_frame_count <= (axle_index[axle_count][6]+axle_index[axle_count][5]) *35/10) )
								fake_axle = 4;
							if (ABS( axle_index[axle_count][4] - axle_min_X ) > 800 )//与前一轴的偏移
								fake_axle = 5;
							//if ((last_axle_frame_count > (axle_index[axle_count][6]+axle_index[axle_count][5]) *28/10/*是否停车*/) && ((axle_index[axle_count][6]+axle_index[axle_count][5])*4 + axle_index[axle_count][9])>min_dis_X[9][index_axle_end] && ( axle_index[axle_count][5]+axle_index[axle_count][6])/3 > ( temp_count+temp_count2) ) //前轴缓慢运行或停车，相隔较近后轴的判定
							////	fake_axle = 6;
							if( car_height ==0 && belowHeigh_X_count  < 2 &&  index_axle_end - index_axle_start > 4 )
								fake_axle = 7;
							if (miss_axle  && ABS( axle_index[axle_count][4] - axle_min_X ) > 200 ) //通过修正的快速运动车辆，车轮不会位移很大
								fake_axle = 8;

						}
						if (belowHeigh_X_max - belowHeigh_X_min > 150) // 车轮的平整度
							fake_axle = 9;

						if(fake_axle == 0)
						{
							if (axle_count > MAX_AXLE_COUNT-1 )
							{
								axle_count = MAX_AXLE_COUNT-1 ;
							}
							axle_count = axle_count+1;
							axle_index[axle_count][1] = index_axle_start;//车轮起始
							axle_index[axle_count][2] = index_axle_end;//车轮结束
							axle_index[axle_count][4] = axle_min_X;//车轮投影最近距离
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

							//判断轴组
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
				axle_min_X = neighout_min_X ;//更新极值
			}
			else //%出轴判定
			{
				index_top = index_extract;
				neighout_min_X = neighout_max_X;
				axle_min_X = neighout_min_X ;//%更新极值
			}
		}

		neighout_max_X = min_dis_X[3][index_extract]; //%继续寻找下去
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
		axle_min_X = neighout_min_X ;//%更新极大值
	}
	return 0;
}

int	axle_group(int axle_present)
{
	unsigned char axle_group_flag = 0;//轴组标志
//	unsigned char tyre_type_pre = 0;
	unsigned char tyre_type = 0;
	unsigned short int	index_axle_end_pre = 0;//前一轴结束点
//	unsigned short int	index_axle_start_pre = 0;//前一轴起始点
	unsigned short int	index_axle_end = 0;//当前轴结束点
	unsigned short int	index_axle_start = 0;//当前轴起始点
//	int interval_frame_count = 0; //联轴之间穿插的帧
//	int ground_edge = 0;//轮轴边缘在地面的投射点X坐标
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
	if (tyre_type == 1 && axle_present == 2 && ratio < ratio_threshold && ratio > 12) //对第二个轴为单胎的强制修正为非联轴
	{
		axle_group_flag = 0;
	}


	//	//计算轮轴边缘在地面的投射点
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
	int vehicle_diameter = 0;//轮轴外直径
	int diff_diameter = 0;//离地高度与直径的差值
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
	axle_below_height_max = axle_below_height_end;//利用离地最高点，估算轮胎直径
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

//出车后清空全局变量
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
/* 设置轮轴需要的参数                                                   */
/************************************************************************/
void SetAxleParam()
{
	Laser_Height=Smp.JGHeight;
	Road_StartX=Smp.StartX;
	Road_EndX=Smp.StartX+Smp.LaneWidth;
}


/************************************************************************/
/* 检查当前轴是否收尾                                                   */
/************************************************************************/
void CheckAxleState(VehicleSet *pVehicle)
{
	if (axle_count != axle_count_record )
	{
		///axle_index[0] 车轮出轴帧 原始帧序号
		///axle_index[1]//轮轴起始帧序号
		///axle_index[2]//轮轴结束帧序号
		///axle_index[3]//车轮类型
		///axle_index[4]//车轮投影最近x
		///axle_index[5]//车轮左半部分变化量
		///axle_index[6]//车轮右半部分变化量
		///axle_index[8]//车轮起始帧 原始帧序号
		///axle_index[9]//车轮结束帧 原始帧序号
		///axle_index[10]//高度阈值下的点的X相差值
		///axle_index[11]//轮轴速度
		///axle_index[12]//轴组标记

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
//轮轴过轴上报给称台
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

//上报心跳信息
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

//轮轴清空全局变量
void AxleClearGlobal()
{
	clear_global();
}
