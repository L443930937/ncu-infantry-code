/*******************************************************************************
                      ��Ȩ���� (C), 2017-,NCUROBOT
 *******************************************************************************
  �� �� ��   : communication.c
  �� �� ��   : ����
  ��    ��   : NCUERM
  ��������   : 2018��7��
  ����޸�   :
  ��������   : ���ֽ���
  �����б�   :void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
																			float dstVmmps_Y,float dstVmmps_W)

*******************************************************************************/

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "mecanum_calc.h"
/* �ڲ��Զ����������� --------------------------------------------------------*/
/* �ڲ��궨�� ----------------------------------------------------------------*/
#define MyAbs(x) ((x > 0) ? (x) : (-x))
#define Front_ratio  1.0f
#define Brak_ratio   1.0f

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/

/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

/* ����ԭ������ ----------------------------------------------------------*/


void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W)
{
     //��̨���󣬼��������׼��ת����
			wheel[0] = -(-dstVmmps_X + dstVmmps_Y + dstVmmps_W*Front_ratio);
			wheel[1] = -(-(dstVmmps_X + dstVmmps_Y - dstVmmps_W*Front_ratio));
			wheel[2] = -(-(-dstVmmps_X + dstVmmps_Y - dstVmmps_W*Brak_ratio));
			wheel[3] = -(dstVmmps_X + dstVmmps_Y + dstVmmps_W*Brak_ratio);	
}


void Rotate_Component_clac( fp32 X_speed , fp32 Y_speed , float X_speed_out , float Y_speed_out ,fp32 yaw_angle )
{

//  if(yaw_angle<0)
//  {
//         if(X_speed != 0)
//         {
//                X_speed_out = X_speed;
//                Y_speed_out = X_speed*arm_sin_f32(yaw_angle*radian_ratio)/arm_cos_f32(yaw_angle*radian_ratio);
//         }else if(Y_speed != 0)
//          {
//                Y_speed_out = Y_speed;
//                X_speed_out = -Y_speed*arm_sin_f32(yaw_angle*radian_ratio)/arm_cos_f32(yaw_angle*radian_ratio);
//          }
//  }else if(yaw_angle>0)
//  {
//         if(X_speed != 0)
//         {
//                X_speed_out = X_speed;
//                Y_speed_out = -X_speed*arm_sin_f32(yaw_angle*radian_ratio)/arm_cos_f32(yaw_angle*radian_ratio);
//         }else if(Y_speed != 0)
//          {
//                Y_speed_out = Y_speed;
//                X_speed_out = Y_speed*arm_sin_f32(yaw_angle*radian_ratio)/arm_cos_f32(yaw_angle*radian_ratio);
//          }
//  }else 
//  {
//         X_speed_out = X_speed;
//         Y_speed_out = Y_speed;
//  }
  
}












