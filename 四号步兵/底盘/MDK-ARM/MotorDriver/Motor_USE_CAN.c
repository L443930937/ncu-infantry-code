/*******************************************************************************
*                     ��Ȩ���� (C), 2017-,NCUROBOT
********************************************************************************
* �� �� ��   : Motor_USE_CAN.c
* �� �� ��   : ����
* ��    ��   : NCURM
* ��������   : 2018��7��
* ����޸�   :
* ��������   : �����ģ����ʹ��CAN���п��Ƶĵ��
* �����б�   :
*ʹ��CANͨѶ�ĵ������̨���   		 ���̵��	 	 	  �������
*				 	��Ӧ�ͺţ� c620						3508					 C2000
*�ӿں�����
*					Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
*					Chassis_Motor( CAN_HandleTypeDef * hcan,
*								  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Motor_USE_CAN.h"
#include "SystemState.h"
#include "protocol.h"
#include "chassis_task.h"
/* �ڲ��Զ����������� --------------------------------------------------------*/

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/
/*******************Ħ���ֵ���͵��̵���Ĳ�������***************************/
moto_measure_t   moto_chassis_get[4] = {0};//4 �� 3508
moto_measure_t   moto_dial_get = {0};  //c2006
moto_measure_t   pit_get;
moto_measure_t   yaw_get;
/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/
//Ϊcan���ͷֱ𴴽����棬��ֹ���ڷ��͵�ʱ����ֻ��һ���ڴ���໥����
static CanTxMsgTypeDef	 Chassis_Motor_Data;

static CanTxMsgTypeDef  CANSend_Error;
static CanTxMsgTypeDef  CANSend_Cp;
/* ����ԭ������ ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: ���̵����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			Chassis_Motor_Data.Data[0]=iq1>>8;
			Chassis_Motor_Data.Data[1]=iq1;
			Chassis_Motor_Data.Data[2]=iq2>>8;
			Chassis_Motor_Data.Data[3]=iq2;
			Chassis_Motor_Data.Data[4]=iq3>>8;
			Chassis_Motor_Data.Data[5]=iq3;
			Chassis_Motor_Data.Data[6]=iq4>>8;
			Chassis_Motor_Data.Data[7]=iq4;
	
			hcan->pTxMsg = &Chassis_Motor_Data;
			HAL_CAN_Transmit(hcan,0);
}	

/**
	**************************************************************
	** Descriptions: ���̵��ʧ�ܺ���
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan)
{
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			Chassis_Motor_Data.Data[0]=0x00;
			Chassis_Motor_Data.Data[1]=0x00;
			Chassis_Motor_Data.Data[2]=0x00;
			Chassis_Motor_Data.Data[3]=0x00;
			Chassis_Motor_Data.Data[4]=0x00;
			Chassis_Motor_Data.Data[5]=0x00;
			Chassis_Motor_Data.Data[6]=0x00;
			Chassis_Motor_Data.Data[7]=0x00;
	
			hcan->pTxMsg = &Chassis_Motor_Data;
			HAL_CAN_Transmit(hcan,5);
}	

/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��6623����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_6623(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->real_current  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);
	ptr->speed_rpm = ptr->real_current;
//	ptr->hall = hcan->pRxMsg->Data[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��3508����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_3508(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->speed_rpm  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->real_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);
	ptr->hall = hcan->pRxMsg->Data[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**
	**************************************************************
	** Descriptions:��ȡ�������ֵ��ƫ��ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 |hcan->pRxMsg->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
	**************************************************************
	** Descriptions: ��ȡ������ܽǶ�ֵ
	** Input: 	
	**			   *P:��Ҫ��ȡ�ܽǶ�ֵ�ĵ�ַ
	**				
	** Output: NULL
	**************************************************************
**/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//?????
		res1 = p->angle + 8192 - p->last_angle;	//??,delta=+
		res2 = p->angle - p->last_angle;				//??	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//??	delta -
		res2 = p->angle - p->last_angle;				//??	delta +
	}
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}


/**
	**************************************************************
	** Descriptions: ����ͨ�Ž���
	** Input: 	ң��������
	**			   *P:�ṹ��
	**				
	** Output: NULL
	**************************************************************
**/
void CAN_GET_YK(RC_Ctl_t * RC , CAN_HandleTypeDef * hcan)
{
	    RC->key.v = (hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	    RC->rc.ch0 = (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]) ;
	    RC->rc.ch1 = (hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]) ;
	    RC->rc.s1  =  hcan->pRxMsg->Data[6];
	    RC->rc.s2  =  hcan->pRxMsg->Data[7];
	
}	

void CAN_GET_YT(moto_measure_t * YT , CAN_HandleTypeDef * hcan)
{
	    YT->angle = (int16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	    YT->total_angle  =(int16_t) (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]) ;
	    YT->flag =  (uint8_t)hcan->pRxMsg->Data[4];
	    YT->flag1 = (uint8_t)hcan->pRxMsg->Data[5];
      Remote.Mode = (uint8_t)hcan->pRxMsg->Data[6];
}	

void CAN_GET_Error(SystemStateDef * Error , CAN_HandleTypeDef * hcan)
{
    
	    Error->OutLine_Flag = (hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	    Error->task_OutLine_Flag  = (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]) ;
	
}	


void CAN_GET_Cilent( CAN_HandleTypeDef * hcan)
{
  Shoot_mouse.mode = hcan->pRxMsg->Data[0];
  
  
}

/**
	**************************************************************
	** Descriptions: ����ͨ�ŷ���
	** Input: 	ң��������
	**			   *P:�ṹ��
	**				
	** Output: NULL
	**************************************************************
**/

void CAN_Send_Error( CAN_HandleTypeDef * hcan, int16_t OutLine_Flag, int16_t task_OutLine_Flag )//����
{
			CANSend_Error.DLC = 0x08;
			CANSend_Error.IDE = CAN_ID_STD;
			CANSend_Error.RTR = CAN_RTR_DATA;
			CANSend_Error.StdId = 0x911;

			CANSend_Error.Data[0]=OutLine_Flag>>8;
			CANSend_Error.Data[1]=OutLine_Flag;
			CANSend_Error.Data[2]=task_OutLine_Flag>>8;
			CANSend_Error.Data[3]=task_OutLine_Flag;
			CANSend_Error.Data[4]=Robot.level;
			CANSend_Error.Data[5]=(Robot.remainHp*10)/Robot.maxHp;
			CANSend_Error.Data[6]=Robot.heat.shoot_17_cooling_rate >>8;
			CANSend_Error.Data[7]=Robot.heat.shoot_17_cooling_rate ;
	
			hcan->pTxMsg = &CANSend_Error;
			HAL_CAN_Transmit(hcan,30);
}	

void CAN_Send_cp(CAN_HandleTypeDef * hcan)
{
            CANSend_Cp.DLC = 0x08;
			CANSend_Cp.IDE = CAN_ID_STD;
			CANSend_Cp.RTR = CAN_RTR_DATA;
			CANSend_Cp.StdId = 0x021;

			CANSend_Cp.Data[0]=Robot.heat.shoot_17_cooling_limit>>8;//��������
			CANSend_Cp.Data[1]=Robot.heat.shoot_17_cooling_limit;
			CANSend_Cp.Data[2]=Robot.heat.shoot_17_heat>>8;//��ǰ����
			CANSend_Cp.Data[3]=Robot.heat.shoot_17_heat;
			CANSend_Cp.Data[4]=Robot.id;
			CANSend_Cp.Data[5]=0;
			CANSend_Cp.Data[6]=0;
			CANSend_Cp.Data[7]=0;
	
			hcan->pTxMsg = &CANSend_Cp;
			HAL_CAN_Transmit(hcan,30);
}

