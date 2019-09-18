#include "stdafx.h"
#include "Avi_Classifier.h"
#include "Avi_FeatureCal.h"
#include "Avi_SpeCheck.h"


/************************************************************************/
/* ����ʶ����                                                           */
/************************************************************************/
void VehIdentifier(VehicleSet *pVehicle)
{
	//�������ں�
	MutiFeatureFusion(pVehicle);
	//���г��ͷַ���
	VehPatternClassify(pVehicle);
	//���͵������ƫ
	VehPatternModify(pVehicle);

}

/************************************************************************/
/* ����ʶ�����                                                         */
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
/* һ�������                                                           */
/* �ж�׼���뷶Χ01-31    255-����                                      */
/************************************************************************/
void OneAxleClassify(VehicleSet *pVehicle)
{
	if (pVehicle->AxleSet.AxleSum<ONE_AXLE)
	{
		pVehicle->JudgeSet.u8JudgeRule=1;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
	else if (ONE_AXLE  ==  pVehicle->AxleSet.AxleSum) //ɨ�赽1����
	{
		pVehicle->JudgeSet.u8JudgePattern = FIRST_KECHE;    //��1
		pVehicle->JudgeSet.u8JudgeRule = 16;  //2��
	}
	else
	{
		pVehicle->JudgeSet.u8JudgeRule=255;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
}


/************************************************************************/
/* ���������                                                           */
/* �ж�׼���뷶Χ32-79    255-����                                      */
/************************************************************************/
void TwoAxleClassify(VehicleSet *pVehicle)
{
	if (pVehicle->AxleSet.AxleSum == TWO_AXLE)
	{
		if (pVehicle->Dp.DpHeight1_2 < CHASSISHEIGHTTHRES) //С�ڵ�����ֵ��
		{
			if (pVehicle->EdgeSet.u16VehHeight > 3200)  //����
			{
				//��4
				pVehicle->JudgeSet.u8JudgePattern=FOURTH_KECHE;
				pVehicle->JudgeSet.u8JudgeRule=32;
			}
			else if (pVehicle->EdgeSet.u16VehHeight > 2600)
			{
				if (pVehicle->EdgeSet.u16HeadHeight < 2000) //��ͷ�߽ϵ�
				{
					//��2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=33;
				}
				else
				{
					//��3
					pVehicle->JudgeSet.u8JudgePattern=THIRD_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=34;
				}
					
			}
			else if (pVehicle->EdgeSet.u16VehHeight > 2000)
			{
				if (pVehicle->EdgeSet.u16HeadHeight >1250)
				{
					//��2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=35;
				}
				else
				{
					//��1
					pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=36;
				}

			}
			else
			{
				//��1���2	
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
			//�޸��ж�׼�� �ڵ��̸߽ϸߵ�����£����ȵ��̸��ж�
			if (pVehicle->FeatureSet.IsHopper) //�ж�׼��ʼ��40
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
				||pVehicle->FeatureSet.IsTailJump)//�ж�׼��ʼ��50
			{
				//��ͷ��������ж�
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
			else if (pVehicle->FeatureSet.IsCastGrd||pVehicle->FeatureSet.IsFillWin)//�ж�׼��ʼ��60
			{
				//Ͷ�������ϻ����ж�
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
			else if (pVehicle->FeatureSet.IsContainer)//�ж�׼��ʼ��70
			{
				//����ж�
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
			else if (pVehicle->FeatureSet.IsCargo)//�ж�׼��ʼ��80
			{
				//�ػ��ж�
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
			else if (pVehicle->FeatureSet.IsTruckTail)//�ж�׼��ʼ��90
			{
				//����β���ж�
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
					 && pVehicle->EdgeSet.u16HeadHeight >1250)//�������� �ж�׼��ʼ��100
				{
					if (pVehicle->EdgeSet.u16HeadHeight > 3200)  //����
					{
						//��4
						pVehicle->JudgeSet.u8JudgePattern=FOURTH_KECHE;
						pVehicle->JudgeSet.u8JudgeRule=100;
					}
					else if (pVehicle->EdgeSet.u16VehHeight > 2600)
					{
						if (pVehicle->EdgeSet.u16HeadHeight < 2000) //��ͷ�߽ϵ�
						{
							//��2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=101;
						}
						else
						{
							//��3
							pVehicle->JudgeSet.u8JudgePattern=THIRD_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=102;
						}
					}
					else if (pVehicle->EdgeSet.u16VehHeight > 2000)
					{
						if (pVehicle->EdgeSet.u16HeadHeight >1250)
						{
							//��2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=103;
						}
						else
						{
							//��1
							pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=104;
						}
					}
					else
					{
						//��1	
						pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
						pVehicle->JudgeSet.u8JudgeRule=105;
					}
				}
				else if (pVehicle->FeatureSet.IsSideUiform
					 && pVehicle->EdgeSet.u16VehHeight > 2000
					 && pVehicle->EdgeSet.u16HeadHeight >1250)//����һ�������� �ж�׼��ʼ��110
				{
					if (pVehicle->EdgeSet.u16HeadHeight > 3200)  //����
					{
						//��4
						pVehicle->JudgeSet.u8JudgePattern=FOURTH_KECHE;
						pVehicle->JudgeSet.u8JudgeRule=110;
					}
					else if (pVehicle->EdgeSet.u16VehHeight > 2600)
					{
						if (pVehicle->EdgeSet.u16HeadHeight < 2000) //��ͷ�߽ϵ�
						{
							//��2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=111;
						}
						else
						{
							//��3
							pVehicle->JudgeSet.u8JudgePattern=THIRD_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=112;
						}
					}
					else if (pVehicle->EdgeSet.u16VehHeight > 2000)
					{
						if (pVehicle->EdgeSet.u16HeadHeight >1250)
						{
							//��2
							pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=113;
						}
						else
						{
							//��1
							pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
							pVehicle->JudgeSet.u8JudgeRule=114;
						}
					}
					else
					{
						//��1	
						pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
						pVehicle->JudgeSet.u8JudgeRule=115;
					}
				}
				else
				{
					//�ж�׼��ʼ��120  ���̸����� 20170729 ͣ�����µ��̲�׼ ����ʹ�ó��������һ����
					if (pVehicle->u16FrameCnt>MAX_CUT)
					{
						if ((pVehicle->FeatureSet.IsGlassWin && pVehicle->FeatureSet.IsSideUiform))
						{
							if (pVehicle->EdgeSet.u16VehHeight > 2000)
							{
								if (pVehicle->EdgeSet.u16HeadHeight >1250)
								{
									//��2
									pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
									pVehicle->JudgeSet.u8JudgeRule=120;
								}
								else
								{
									//��1
									pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
									pVehicle->JudgeSet.u8JudgeRule=121;
								}
							}
							else
							{
								//��1	
								pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
								pVehicle->JudgeSet.u8JudgeRule=122;
							}
						}
					}
					else
					{
						//δͣ�� ���̿��ŶȽϸ� ֱ���ж�
						if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //����2200����	
						{
							//��1
							pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
							pVehicle->JudgeSet.u8JudgeRule=123;
						}
						else
						{
							//��2
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
/* ���������                                                           */
/* �ж�׼���뷶Χ200-230       255-����                                  */
/************************************************************************/
void ThreeAxleClassify(VehicleSet *pVehicle)
{
	if (pVehicle->AxleSet.AxleSum == THREE_AXLE)
	{
		if (pVehicle->EdgeSet.u16VehHeight < 2000 || pVehicle->EdgeSet.u16HeadHeight < 1800)  //SUV���͵������Ƚ������г� 
		{
			if (pVehicle->Dp.DpHeight1_2>CHASSISHEIGHTTHRES)
			{
				//��1
				pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				pVehicle->JudgeSet.u8JudgeRule=200;
			}
			else
			{
				 //��1
				pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
				pVehicle->JudgeSet.u8JudgeRule=201;
			}
		}
		else
		{
			//��3
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
/* >3�������                                                           */
/* �ж�׼���뷶Χ231-254     255-����                                   */
/************************************************************************/
void MutiAxleClassify(VehicleSet *pVehicle)
{
	if (pVehicle->AxleSet.AxleSum>3)
	{
		if (pVehicle->AxleSet.AxleSum > FOUR_AXLE) //����4��
		{
			//��5
			pVehicle->JudgeSet.u8JudgePattern=FIFTH_HUOCHE;
			pVehicle->JudgeSet.u8JudgeRule=231;
		}
		else 
		{
			//��4
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
/* ���͵�����                                                           */
/* ������ֲ�01-90								                        */
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
			if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //����2200����	
			{
				//��1
				pVehicle->JudgeSet.u8ModifyRule=1;
				pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
			}
			else
			{
				//��2
				pVehicle->JudgeSet.u8ModifyRule=2;
				pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
			}
		}

		if ((pVehicle->FeatureSet.IsHeadJump>0 
			|| pVehicle->FeatureSet.IsTailJump)&& 
			pVehicle->FeatureSet.IsSideUiform<1)
		{
			//�߶�����
			if (pVehicle->JudgeSet.u8JudgePattern<FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //����2200����	
				{
					//��1
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
					pVehicle->JudgeSet.u8ModifyRule=3;
				}
				else
				{
					//��2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
					pVehicle->JudgeSet.u8ModifyRule=4;
				}
			}
		}

		if (pVehicle->FeatureSet.IsCargo>0||pVehicle->FeatureSet.IsCastGrd>0)
		{
			if (pVehicle->JudgeSet.u8JudgePattern<FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //����2200����	
				{
					//��1
					pVehicle->JudgeSet.u8ModifyRule=5;
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				}
				else
				{
					//��2
					pVehicle->JudgeSet.u8ModifyRule=6;
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
				}
			}

		}

		if (pVehicle->FeatureSet.IsTruckTail>0)
		{
			//����β��
			if (pVehicle->JudgeSet.u8JudgePattern < FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //����2200����	
				{
					//��1
					pVehicle->JudgeSet.u8ModifyRule=7;
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				}
				else
				{
					//��2
					pVehicle->JudgeSet.u8ModifyRule=8;
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
				}
			}
		}

		if (pVehicle->FeatureSet.IsContainer>0)
		{
			//����
			if (pVehicle->JudgeSet.u8JudgePattern < FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //����2200����	
				{
					//��1
					pVehicle->JudgeSet.u8ModifyRule=9;
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				}
				else
				{
					//��2
					pVehicle->JudgeSet.u8ModifyRule=10;
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
				}
			}
		}

		if (pVehicle->FeatureSet.IsFillWin>0)
		{
			//��ջ���
			if (pVehicle->JudgeSet.u8JudgePattern < FIFTH_HUOCHE)
			{
				if (pVehicle->EdgeSet.u16HeadHeight <= 2200)  //����2200����	
				{
					//��1
					pVehicle->JudgeSet.u8ModifyRule=13;
					pVehicle->JudgeSet.u8JudgePattern=FIRST_HUOCHE;
				}
				else
				{
					//��2
					pVehicle->JudgeSet.u8ModifyRule=14;
					pVehicle->JudgeSet.u8JudgePattern=SECOND_HUOCHE;
				}
			}
		}
	}

	//20170731 ����
	if (pVehicle->FeatureSet.IsCar)
	{
		if (pVehicle->JudgeSet.u8JudgePattern<SECOND_HUOCHE)
		{
			if (pVehicle->EdgeSet.u16VehHeight > 3200)  //����
			{
				//��4
				pVehicle->JudgeSet.u8JudgePattern=FOURTH_KECHE;
				pVehicle->JudgeSet.u8ModifyRule=15;
			}
			else if (pVehicle->EdgeSet.u16VehHeight > 2600)
			{
				if (pVehicle->EdgeSet.u16HeadHeight < 2000) //��ͷ�߽ϵ�
				{
					//��2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8ModifyRule=16;
				}
				else
				{
					//��3
					pVehicle->JudgeSet.u8JudgePattern=THIRD_KECHE;
					pVehicle->JudgeSet.u8JudgeRule=17;
				}

			}
			else if (pVehicle->EdgeSet.u16VehHeight > 2000)
			{
				if (pVehicle->EdgeSet.u16HeadHeight >1250)
				{
					//��2
					pVehicle->JudgeSet.u8JudgePattern=SECOND_KECHE;
					pVehicle->JudgeSet.u8ModifyRule=18;
				}
				else
				{
					//��1
					pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
					pVehicle->JudgeSet.u8ModifyRule=19;
				}

			}
			else
			{
				//��1���2	
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
		//��1
		pVehicle->JudgeSet.u8ModifyRule=15;
		pVehicle->JudgeSet.u8JudgePattern=FIRST_KECHE;
	}
}
