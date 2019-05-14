/* 包含头文件----------------------------------------------------------------*/
#include "chassis_task.h"
#include "SystemState.h"
#include "user_lib.h"
/* 内部宏定义----------------------------------------------------------------*/
#define radian_ratio 0.00076694f   //360/8191*0.01745
/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId Chassis_QueueHandle;

/* 内部常量定义--------------------------------------------------------------
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
float pid_calc(pid_t* pid, float get, float set);
void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
													float dstVmmps_Y,float dstVmmps_W);
----------------------------------------------------------------------------
*/
/* 外部变量声明--------------------------------------------------------------*/
moto3508_type  moto_3508_set = {.flag = 0}; 
extern float power;		//功率  	_测试变量
int8_t chassis_disable_flg;
/* 调用的外部函数原型声明----------------------------------------------------------*/

/* 内部变量------------------------------------------------------------------*/
pid_t pid_3508_pos;     		 //底盘电机位置环
pid_t pid_3508_spd[4];			 //底盘电机速度环
pid_t pid_3508_current[4];	 //底盘电机电流环节	
pid_t pid_chassis_follow = {0};//底盘跟随位置环
pid_t pid_chassis_follow_spd = {0};//底盘跟随速度环

Mode_Set Chassis;
Mode_Set Shoot_mouse;
static float Current_set[4] = {0};  //传递给功率限制的缓存

//测试变量
int16_t angle[2];

#define CHASSIS_PERIOD 5

/* 内部函数原型声明----------------------------------------------------------*/
void Chassis_pid_init(void)
{
	
	 PID_struct_init(&pid_3508_pos, POSITION_PID, 10000, 1000,
									1.5f,	0.0f,	20.0f);  // motos angular rate closeloop.pid:1.5,0.0,20.0
	 pid_3508_pos.deadband=150;
	
	
	 PID_struct_init(&pid_chassis_follow, POSITION_PID,5000,100,
	                2.5f, 0.0f , 1.0f  );
//	 pid_chassis_follow.deadband=50;
	 PID_struct_init(&pid_chassis_follow_spd, POSITION_PID,2000,100,
	                1.5f, 0.01f , 2.0f  );
	
	
		for(int i=0; i<4; i++)
		{ 
			PID_struct_init(&pid_3508_spd[i], POSITION_PID, 10000, 2000,
										1.5f,	0.1f,	0.1f	);  //4 motos angular rate closeloop.
		}
	
		PID_struct_init(&pid_3508_current[0], POSITION_PID, 6000, 500,
								0.65f,	0.01f,	0.1f	);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[1], POSITION_PID, 6000, 500,
								0.6f,	0.01f,	0.01f	);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[2], POSITION_PID, 6000, 500,
								0.6f,	0.01f,	0.01f	);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[3], POSITION_PID, 6000, 500,
									0.6f,	0.01f,	0.01f	);  //4 motos angular rate closeloop.
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	底盘控制任务
	*	@retval	
****************************************************************************************/
void Chassis_Contrl_Task(void const * argument)
{
	static float  wheel[4]={0,0,0,0};
	static uint8_t niuyao_flag=0;
	static int16_t Speed_W;
  static int16_t Speed_X;
  static int16_t Speed_Y;
  static int16_t speed_w;
  static fp32 Angle_gap;
	osDelay(1000);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	

      chassis_disable_flg=0;
	    Chassis_pid_init();
	for(;;)
	{
	  IMU_Get_Data();

    RefreshTaskOutLineTime(ChassisContrlTask_ON);
		
    Angle_gap = yaw_get.angle-Middle_angle;
		if(yaw_get.flag==2) Chassis.mode=1;
		else if(yaw_get.flag1==3)  Chassis.mode=1;
		else if(yaw_get.flag1==1 && Chassis.mode!=3)  Chassis.mode=0;
    
		switch(Chassis.mode)
		{	
			case 1:	{	//分离	
			motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W);
			}break;
			case 0:	{	//底盘跟随云台
                
			pid_calc(&pid_chassis_follow,yaw_get.angle,Middle_angle);
			speed_w=pid_calc(&pid_chassis_follow_spd,-(imu_data.gz),pid_chassis_follow.pos_out);
		  if(abs(Angle_gap)<100)  speed_w=0;

			motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y, -speed_w); 																																												
			}break;
			case 3 : 
      { //扭腰
				
				if(yaw_get.angle<3387)      
				{
					niuyao_flag=0;
				}
				if(yaw_get.angle>4387)
				{
					niuyao_flag=1;
				}
				 
				if(niuyao_flag) 
				{
					
					  Speed_W-=60; 
					        if(Speed_W<-2600)  Speed_W=-2600;
						
				}
				else 
				{
					
					  Speed_W+=60;
					         if(Speed_W>2600) Speed_W=2600;
					
          
				}
        
        //扭腰模式下的XY模式限幅
        if(moto_3508_set.dstVmmps_X>1500)
        {
          moto_3508_set.dstVmmps_X = 1500;
        }else if(moto_3508_set.dstVmmps_X<-1500)
        {
           moto_3508_set.dstVmmps_X = -1500;
        }   
        if(moto_3508_set.dstVmmps_Y>1500)
        {
          moto_3508_set.dstVmmps_Y = 1500;
        }else if(moto_3508_set.dstVmmps_Y<-1500)
        {
           moto_3508_set.dstVmmps_Y = -1500;
        }

        Speed_X = moto_3508_set.dstVmmps_X*arm_cos_f32(-Angle_gap*radian_ratio)+moto_3508_set.dstVmmps_Y*arm_sin_f32(-Angle_gap*radian_ratio);
        Speed_Y = -moto_3508_set.dstVmmps_X*arm_sin_f32(-Angle_gap*radian_ratio)+moto_3508_set.dstVmmps_Y*arm_cos_f32(-Angle_gap*radian_ratio);
				
				motor_move_setvmmps(wheel,Speed_X,Speed_Y,-Speed_W);
			}break;
				
		}
		for(int i=0; i<4; i++)
			{		
				pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
			}
//			
//      	if(chassis_gimble_Mode_flg==1&&abs(yaw_get.total_angle)>2000)//电机失能保护
//				{
//			        gimbal_disable_flg=1;
//			 		    chassis_disable_flg=1;
//		  			   
//				}
		
		/**********功率限制*********/

		/**********功率限制*********/

			Current_set[0] = pid_3508_spd[0].pos_out;
			Current_set[1] = pid_3508_spd[1].pos_out;
			Current_set[2] = pid_3508_spd[2].pos_out;
			Current_set[3] = pid_3508_spd[3].pos_out;			

		  Super_Capacitance(Current_set);
	
			
			pid_3508_spd[0].pos_out = Current_set[0];			
			pid_3508_spd[1].pos_out = Current_set[1];
			pid_3508_spd[2].pos_out = Current_set[2];
			pid_3508_spd[3].pos_out = Current_set[3];


				switch(Chassis_motor.Mode)//Chassis_motor.Mode
				{
					case 1:  pid_3508_spd[0].pos_out=0;
					break;
					case 2:  pid_3508_spd[1].pos_out=0;
					break;
					case 3:  pid_3508_spd[2].pos_out=0;
					break;
					case 4:  pid_3508_spd[3].pos_out=0;
					break;
					default: break;
				}
				
			if(Chassis_motor.Mode == 5 ||   Remote.Mode == 1)
			{
        
				  Chassis_Motor_Disable(&hcan1);
			}
			else
			{
					Chassis_Motor(&hcan1,
												pid_3508_spd[0].pos_out,
												pid_3508_spd[1].pos_out, 
												pid_3508_spd[2].pos_out, 
												pid_3508_spd[3].pos_out);						
			}
				
//      printf("%d\r\n",Robot.heat.shoot_17_cooling_rate);
			osDelayUntil(&xLastWakeTime, CHASSIS_PERIOD);
  }
    
  
}




/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	紧急停止函数
	*	@retval	
****************************************************************************************/
void hard_brak()
{
		moto_3508_set.dstVmmps_X=0;
		moto_3508_set.dstVmmps_Y=0;
		moto_3508_set.dstVmmps_W=0;
}