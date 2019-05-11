/* 包含头文件----------------------------------------------------------------*/
#include "data_pro_task.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/
#define press_times  20
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
//extern osSemaphoreId Dubs_BinarySemHandle;
/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/

#define REMOTE_PERIOD 1 

/* 外部变量声明--------------------------------------------------------------*/

/* 调用的外部函数原型声明------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
int16_t XY_speed_max = 2500;
int16_t XY_speed_min = -2500; 
int16_t W_speed_max = 2500;
int16_t W_speed_min = -2500; 
uint8_t press_counter;
uint8_t shot_anjian_counter=0;
uint8_t shot_frequency = 20;
int8_t chassis_gimble_Mode_flg;
//volatile float remain_power=0.0;   //底盘功率 _待续
//float power; 				 //底盘功率 _测试

//float chassis_Current; 
//float	 chassis_Volt; 
/* 内部函数原型声明-----------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	与遥控器进行对接，对遥控器的数据进行处理，实现对底盘、云台、发射机构的控制
	*	@retval	
****************************************************************************************/
void RemoteControlProcess()  
{

							moto_3508_set.dstVmmps_X=((RC_Ctl.rc.ch0-0x400)*10);
							moto_3508_set.dstVmmps_Y=((RC_Ctl.rc.ch1-0x400)*10);
		

                if(RC_Ctl.rc.s1==1)
                {
                    Chassis.mode=1; 
                }
                  else if(RC_Ctl.rc.s1==2)
                  {
										Chassis.mode=3;
                  }
                  else
                  {
                     Chassis.mode=0;
                  }
								
							
					
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	对键鼠的数据进行处理
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
	
	
	if(RC_Ctl.key.v & 0x10 )// 
					{

						XY_speed_max = 5000;//(NORMAL_SPEED_MAX)*3.5;
						XY_speed_min = -5000;//(NORMAL_SPEED_MIN)*3.5;
					}else 
					{
							XY_speed_max = 3000;//(NORMAL_SPEED_MAX)*3.5;
						XY_speed_min = -3000;//(NORMAL_SPEED_MIN)*3.5;
					}
			
					if(RC_Ctl.key.v & 0x01)                       moto_3508_set.dstVmmps_Y += ACC_SPEED;//按下W键
					else if(RC_Ctl.key.v & 0x02)                  moto_3508_set.dstVmmps_Y -= ACC_SPEED;//按下S键
					else{  
							 	if(moto_3508_set.dstVmmps_Y>-DEC_SPEED&&moto_3508_set.dstVmmps_Y<DEC_SPEED) 	 moto_3508_set.dstVmmps_Y = 0;
								if(moto_3508_set.dstVmmps_Y>0) 	                   moto_3508_set.dstVmmps_Y -= DEC_SPEED;
								if(moto_3508_set.dstVmmps_Y<0) 		                 moto_3508_set.dstVmmps_Y += DEC_SPEED;
					}


					if(RC_Ctl.key.v & 0x04)                        moto_3508_set.dstVmmps_X -= ACC_SPEED; //按下D键
					else if(RC_Ctl.key.v & 0x08)    		           moto_3508_set.dstVmmps_X += ACC_SPEED;//按下A键
					else{
									if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
									if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
									if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
					}

					
					
					//车体旋转
					if(RC_Ctl.key.v & 0x40)
					{
              moto_3508_set.dstVmmps_W -= ACC_SPEED;
            if(moto_3508_set.dstVmmps_W<W_speed_min) moto_3508_set.dstVmmps_W = W_speed_min;
					}else if(RC_Ctl.key.v & 0x80)
          {
             moto_3508_set.dstVmmps_W += ACC_SPEED;
             if(moto_3508_set.dstVmmps_W>W_speed_max) moto_3508_set.dstVmmps_W = W_speed_max;
          }else
          {
            	 moto_3508_set.dstVmmps_W = 0;
//								if(moto_3508_set.dstVmmps_W>0) 	                   moto_3508_set.dstVmmps_W -= DEC_SPEED;
//								if(moto_3508_set.dstVmmps_W<0) 		                 moto_3508_set.dstVmmps_W += DEC_SPEED;
          }
 
          if(RC_Ctl.key.v & 0x200)//扭腰
          {
            Chassis.mode= 3;
          }else if(RC_Ctl.key.v & 0x800 && RC_Ctl.key.v & 0x20)
          {
            Chassis.mode= 1;
          }
								
}



/* 任务主体部分 -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	遥控数据接收及处理任务
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
	uint32_t NotifyValue;
	
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	
	for(;;)
	{
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14); //GRE_main
			CAN_Send_cp(&hcan2);
      CAN_Send_Error(&hcan2,SystemState.OutLine_Flag,SystemState.task_OutLine_Flag);
			RefreshTaskOutLineTime(RemoteDataTask_ON);
				switch(RC_Ctl.rc.s2)
				{
					case 1: RemoteControlProcess();break; 
					case 3: hard_brak();break;
					case 2: MouseKeyControlProcess();break;
					default :break;
				}					
				
			VAL_LIMIT(moto_3508_set.dstVmmps_W, W_speed_min, W_speed_max);
      VAL_LIMIT(moto_3508_set.dstVmmps_Y, -6000, 6000);
			VAL_LIMIT(moto_3508_set.dstVmmps_X, -6000, 6000);

       press_counter++;
		
				if(Remote.Mode)//遥控器掉线
				{
						Remote_Disable();
				}
		
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}




