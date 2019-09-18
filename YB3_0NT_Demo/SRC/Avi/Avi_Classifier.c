#include "stdafx.h"
#include "Avi_Classifier.h"
#include "Avi_FeatureCal.h"
#include "Avi_SpeCheck.h"


/************************************************************************/
/* 车型识别器                                                           */
/************************************************************************/
void VehIdentifier(VehicleSet *pVehicle)
{
	//多特征融合
	MutiFeatureFusion(pVehicle);
	//进行车型分分类
	VehPatternClassify(pVehicle);
	//车型调整与纠偏
	VehPatternModify(pVehicle);

}

/************************************************************************/
/* 车型识别分类                                                         */
/************************************************************************/
void VehPatternClassify(VehicleSet *pVehicle)
{
	switch (pVehicle->AxleSet.AxleSum)
	{
	case 0:
	case 1:
		OneAxleClassify(pVehicle);
		break;
	case 2:
		TwoAxleClassify(pVehicle);
		break;
	case 3:
		ThreeAxleClassify(pVehicle);
		break;
	case 4:
	case 5:
	case 6:
		MutiAxleClassify(pVehicle);
		break;
	default:
		MutiAxleClassify(pVehicle);
		break;
	}
}


/************************************************************************/
/* 一轴分类器                                                           */
/* 判定准则码范围01-31    255-出错                                      */
/************************************************************************/
void OneAxleClassify(VehicleSet *pVehicle)
{
	if (pVehicle->AxleSet.AxleSum<ONE_AXLE)
	{
		pVehicle->JudgeSet.u8JudgeRule=1;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
	else if (ONE_AXLE  ==  pVehicle->AxleSet.AxleSum) //扫描到1个轴
	{
		pVehicle->JudgeSet.u8JudgePattern = FIRST_KECHE;    //客1
		pVehicle->JudgeSet.u8JudgeRule = 16;  //2轴
	}
	else
	{
		pVehicle->JudgeSet.u8JudgeRule=255;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
}


/************************************************************************/
/* 二轴分类器                                                           */
/* 判定准则码范围32-79    255-出错                                      */
/************************************************************************/
void TwoAxleClassify(VehicleSet *pVehicle)
{
	if (pVehicle->AxleSet.AxleSum == TWO_AXLE)
	{
		if (pVehicle->Dp.DpHeight1_2 < CHASSISHEIGHTTHRES) //小于底盘阈值高
		{
			if (pVehicle->EdgeSet.u16VehHeight > 3200)  //车高
			{
				//客4
				pVehicle->JudgeSet.u8JudgePattern=FOURTH_KECHE;
				pVehicle->JudgeSet.u8JudgeRule=32;
			}
			else if (pVehicle->EdgeSet.u16VehHeight > 2600)
			{
				if (pVehicle->EdgeSet.u16HeadHeight < 2000) //车头高较低
				{
					//客2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=33;
				}
				else
				{
					//客3
					pVehicle->JudgeSet.u8JudgePattern=THIRD_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=34;
				}
					
			}
			else if (pVehicle->EdgeSet.u16VehHeight > 2000)
			{
				if (pVehicle->EdgeSet.u16HeadHeight >1250)
				{
					//客2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=35;
				}
				else
				{
					//客1
					pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=36;
				}

			}
			else
			{
				//客1或客2	
				if (pVehicle->FeatureSet.IsJinBei>0)
				{
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=37;
					pVehicle->FeatureSet.IsJinBei=1;
				}
				else
				{
					pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=38;
				}
			}
		}
		else
		{
			//修改判定准则 在底盘高较高的情况下，优先底盘高判定
			if (pVehicle->FeatureSet.IsHopper) //判定准则开始码40
			{
				if (pVehicle->EdgeSet.u16HeadHeight<2200)
				{
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=39;
				}
				else 
				{
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=40;
				}
				
			}
			else if (pVehicle->FeatureSet.IsHeadJump
				||pVehicle->FeatureSet.IsTailJump)//判定准则开始码50
			{
				//车头跳变货车判定
				if (pVehicle->EdgeSet.u16HeadHeight<2200)
				{
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=50;
				}
				else 
				{
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=51;
				}
			}
			else if (pVehicle->FeatureSet.IsCastGrd||pVehicle->FeatureSet.IsFillWin)//判定准则开始码60
			{
				//投掷到地上货车判定
				if (pVehicle->EdgeSet.u16HeadHeight<2200)
				{
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=60;
				}
				else 
				{
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=61;
				}
			}
			else if (pVehicle->FeatureSet.IsContainer)//判定准则开始码70
			{
				//箱货判定
				if (pVehicle->EdgeSet.u16HeadHeight<2200)
				{
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=70;
				}
				else 
				{
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=71;
				}
			}
			else if (pVehicle->FeatureSet.IsCargo)//判定准则开始码80
			{
				//载货判定
				if (pVehicle->EdgeSet.u16HeadHeight<2200)
				{
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=80;
				}
				else 
				{
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=81;
				}
			}
			else if (pVehicle->FeatureSet.IsTruckTail)//判定准则开始码90
			{
				//货车尾部判定
				if (pVehicle->EdgeSet.u16HeadHeight<2200)
				{
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=90;
				}
				else 
				{
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
					pVehicle->JudgeSet.u8JudgeRule=91;
				}

			}
			else 
			{
	
				if (pVehicle->FeatureSet.IsGlassWin
					 && pVehicle->EdgeSet.u16VehHeight > 2000
					 && pVehicle->EdgeSet.u16HeadHeight >1250)//车窗特征 判定准则开始码100
				{
					if (pVehicle->EdgeSet.u16HeadHeight > 3200)  //车高
					{
						//客4
						pVehicle->JudgeSet.u8JudgePattern=FOURTH_KECHE;
						pVehicle->JudgeSet.u8JudgeRule=100;
					}
					else if (pVehicle->EdgeSet.u16VehHeight > 2600)
					{
						if (pVehicle->EdgeSet.u16HeadHeight < 2000) //车头高较低
						{
							//客2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=101;
						}
						else
						{
							//客3
							pVehicle->JudgeSet.u8JudgePattern=THIRD_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=102;
						}
					}
					else if (pVehicle->EdgeSet.u16VehHeight > 2000)
					{
						if (pVehicle->EdgeSet.u16HeadHeight >1250)
						{
							//客2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=103;
						}
						else
						{
							//客1
							pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=104;
						}
					}
					else
					{
						//客1	
						pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
						pVehicle->JudgeSet.u8JudgeRule=105;
					}
				}
				else if (pVehicle->FeatureSet.IsSideUiform
					 && pVehicle->EdgeSet.u16VehHeight > 2000
					 && pVehicle->EdgeSet.u16HeadHeight >1250)//侧面一致性特征 判定准则开始码110
				{
					if (pVehicle->EdgeSet.u16HeadHeight > 3200)  //车高
					{
						//客4
						pVehicle->JudgeSet.u8JudgePattern=FOURTH_KECHE;
						pVehicle->JudgeSet.u8JudgeRule=110;
					}
					else if (pVehicle->EdgeSet.u16VehHeight > 2600)
					{
						if (pVehicle->EdgeSet.u16HeadHeight < 2000) //车头高较低
						{
							//客2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=111;
						}
						else
						{
							//客3
							pVehicle->JudgeSet.u8JudgePattern=THIRD_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=112;
						}
					}
					else if (pVehicle->EdgeSet.u16VehHeight > 2000)
					{
						if (pVehicle->EdgeSet.u16HeadHeight >1250)
						{
							//客2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=113;
						}
						else
						{
							//客1
							pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=114;
						}
					}
					else
					{
						//客1	
						pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
						pVehicle->JudgeSet.u8JudgeRule=115;
					}
				}
				else
				{
					//判定准则开始码120  底盘高特征 20170729 停车导致底盘不准 优先使用车窗与侧面一致性
					if (pVehicle->u16FrameCnt>MAX_CUT)
					{
						if ((pVehicle->FeatureSet.IsGlassWin && pVehicle->FeatureSet.IsSideUiform))
						{
							if (pVehicle->EdgeSet.u16VehHeight > 2000)
							{
								if (pVehicle->EdgeSet.u16HeadHeight >1250)
								{
									//客2
									pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
									pVehicle->JudgeSet.u8JudgeRule=120;
								}
								else
								{
									//客1
									pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
									pVehicle->JudgeSet.u8JudgeRule=121;
								}
							}
							else
							{
								//客1	
								pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
								pVehicle->JudgeSet.u8JudgeRule=122;
							}
						}
					}
					else
					{
						//未停车 底盘可信度较高 直接判定
						if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //车高2200以下	
						{
							//货1
							pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
							pVehicle->JudgeSet.u8JudgeRule=123;
						}
						else
						{
							//货2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
							pVehicle->JudgeSet.u8JudgeRule=124;
						}
					}
					
				}
			}
		}
	}
	else
	{
		pVehicle->JudgeSet.u8JudgeRule=255;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
}

/************************************************************************/
/* 三轴分类器                                                           */
/* 判定准则码范围200-230       255-出错                                  */
/************************************************************************/
void ThreeAxleClassify(VehicleSet *pVehicle)
{
	if (pVehicle->AxleSet.AxleSum == THREE_AXLE)
	{
		if (pVehicle->EdgeSet.u16VehHeight < 2000 || pVehicle->EdgeSet.u16HeadHeight < 1800)  //SUV车型的轴数比较容易判出 
		{
			if (pVehicle->Dp.DpHeight1_2>CHASSISHEIGHTTHRES)
			{
				//货1
				pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				pVehicle->JudgeSet.u8JudgeRule=200;
			}
			else
			{
				 //客1
				pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
				pVehicle->JudgeSet.u8JudgeRule=201;
			}
		}
		else
		{
			//货3
			pVehicle->JudgeSet.u8JudgePattern=THIRD_HUOCHE;
			pVehicle->JudgeSet.u8JudgeRule=202;

		}
	}
	else
	{
		pVehicle->JudgeSet.u8JudgeRule=255;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
}

/************************************************************************/
/* >3轴分类器                                                           */
/* 判定准则码范围231-254     255-出错                                   */
/************************************************************************/
void MutiAxleClassify(VehicleSet *pVehicle)
{
	if (pVehicle->AxleSet.AxleSum>3)
	{
		if (pVehicle->AxleSet.AxleSum > FOUR_AXLE) //大于4轴
		{
			//货5
			pVehicle->JudgeSet.u8JudgePattern=FIFTH_HUOCHE;
			pVehicle->JudgeSet.u8JudgeRule=231;
		}
		else 
		{
			//货4
			pVehicle->JudgeSet.u8JudgePattern=FOURTH_HUOCHE;
			pVehicle->JudgeSet.u8JudgeRule=232;
		}
	}
	else
	{
		pVehicle->JudgeSet.u8JudgeRule=255;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
}


/************************************************************************/
/* 车型调整新                                                           */
/* 调整码分布01-90								                        */
/************************************************************************/
void VehPatternModify(VehicleSet *pVehicle)
{
//	uint16  l_u8Index=0;
//	uint16  l_u16Height=0;

	if (pVehicle->JudgeSet.u8JudgePattern<FIRST_HUOCHE
		&&(pVehicle->Dp.DpHeight1_2>0||pVehicle->Dp.DpHeight2_3>0))
	{
		if (pVehicle->FeatureSet.IsHopper>0)
		{
			if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //车高2200以下	
			{
				//货1
				pVehicle->JudgeSet.u8ModifyRule=1;
				pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
			}
			else
			{
				//货2
				pVehicle->JudgeSet.u8ModifyRule=2;
				pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
			}
		}

		if ((pVehicle->FeatureSet.IsHeadJump>0 
			|| pVehicle->FeatureSet.IsTailJump)&& 
			pVehicle->FeatureSet.IsSideUiform<1)
		{
			//高度跳变
			if (pVehicle->JudgeSet.u8JudgePattern<FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //车高2200以下	
				{
					//货1
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
					pVehicle->JudgeSet.u8ModifyRule=3;
				}
				else
				{
					//货2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
					pVehicle->JudgeSet.u8ModifyRule=4;
				}
			}
		}

		if (pVehicle->FeatureSet.IsCargo>0||pVehicle->FeatureSet.IsCastGrd>0)
		{
			if (pVehicle->JudgeSet.u8JudgePattern<FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //车高2200以下	
				{
					//货1
					pVehicle->JudgeSet.u8ModifyRule=5;
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				}
				else
				{
					//货2
					pVehicle->JudgeSet.u8ModifyRule=6;
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
				}
			}

		}

		if (pVehicle->FeatureSet.IsTruckTail>0)
		{
			//货车尾部
			if (pVehicle->JudgeSet.u8JudgePattern < FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //车高2200以下	
				{
					//货1
					pVehicle->JudgeSet.u8ModifyRule=7;
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				}
				else
				{
					//货2
					pVehicle->JudgeSet.u8ModifyRule=8;
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
				}
			}
		}

		if (pVehicle->FeatureSet.IsContainer>0)
		{
			//货箱
			if (pVehicle->JudgeSet.u8JudgePattern < FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //车高2200以下	
				{
					//货1
					pVehicle->JudgeSet.u8ModifyRule=9;
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				}
				else
				{
					//货2
					pVehicle->JudgeSet.u8ModifyRule=10;
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
				}
			}
		}

		if (pVehicle->FeatureSet.IsFillWin>0)
		{
			//封闭货车
			if (pVehicle->JudgeSet.u8JudgePattern < FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //车高2200以下	
				{
					//货1
					pVehicle->JudgeSet.u8ModifyRule=13;
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				}
				else
				{
					//货2
					pVehicle->JudgeSet.u8ModifyRule=14;
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
				}
			}
		}
	}

	//20170731 增加
	if (pVehicle->FeatureSet.IsCar)
	{
		if (pVehicle->JudgeSet.u8JudgePattern<SECOND_HUOCHE)
		{
			if (pVehicle->EdgeSet.u16VehHeight > 3200)  //车高
			{
				//客4
				pVehicle->JudgeSet.u8JudgePattern=FOURTH_KECHE;
				pVehicle->JudgeSet.u8ModifyRule=15;
			}
			else if (pVehicle->EdgeSet.u16VehHeight > 2600)
			{
				if (pVehicle->EdgeSet.u16HeadHeight < 2000) //车头高较低
				{
					//客2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8ModifyRule=16;
				}
				else
				{
					//客3
					pVehicle->JudgeSet.u8JudgePattern=THIRD_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=17;
				}

			}
			else if (pVehicle->EdgeSet.u16VehHeight > 2000)
			{
				if (pVehicle->EdgeSet.u16HeadHeight >1250)
				{
					//客2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8ModifyRule=18;
				}
				else
				{
					//客1
					pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
					pVehicle->JudgeSet.u8ModifyRule=19;
				}

			}
			else
			{
				//客1或客2	
				if (pVehicle->FeatureSet.IsJinBei>0)
				{
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8ModifyRule=20;
					pVehicle->FeatureSet.IsJinBei=1;
				}
				else
				{
					pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
					pVehicle->JudgeSet.u8ModifyRule=21;
				}
			}
		}
	}
	else if (pVehicle->EdgeSet.u16VehHeight < 1500)
	{
		//客1
		pVehicle->JudgeSet.u8ModifyRule=15;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
}
