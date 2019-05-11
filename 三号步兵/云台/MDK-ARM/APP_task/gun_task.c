/* 包含头文件----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
#include "gimbal_task.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
#define GUN_PERIOD  10
#define Mocha_PERIOD  100
#define BLOCK_TIME 2000
#define REVERSE_TIME 2000
#define DELAY_TIME  1000
/* 外部变量声明--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
Mode_Set bopan = {0};	
Mode_Set Shoot_heat = {0};
ramp_function_source_t shoot;
extern uint8_t shot_frequency;
uint8_t finish_flag = 0;
static uint8_t Delay_flg;

//Power_Heat * power_heat;
/* 外部函数原型声明-----------------------------------------------------------

-----------------------------------------------------------------------------
-*/
/* 内部变量------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环
/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
  
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.2f,	0.0000f,	2.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.1f,	0.0f	);  

} 

void Mocha_Pid_Init()
{
	
		PID_struct_init(&pid_rub_spd[0], POSITION_PID, 10, 1,
									0.05f,	0.0f,	0.0f	);  
		PID_struct_init(&pid_rub_spd[1], POSITION_PID, 10, 1,
									0.05f,	0.0f,	0.0f	);  
	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	发射任务
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 

	osDelay(2000);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	Gun_Pid_Init();
	
  uint8_t motor_stop_flag=0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
	static uint8_t set_cnt = 0;
  static float Delay_time ;
  /*设定发弹*/
    bopan.flag=1;
	for(;;)
	{
		
		RefreshTaskOutLineTime(GunTask_ON);
		  
     if(Shoot.flag == 2 )  
				{
					bopan.mode = bopan_shangdao_mode;
					Delay_flg=0;
				}else if(Shoot.shoot_flg  == 1 && bopan.mode == bopan_shangdao_mode)  bopan.mode=bopan_Stop;
             else if(Shoot.flag == 1 && bopan.mode == bopan_danfa_mode &&  Shoot.cnt == Shoot.danfa_cnt+2)  Delay_flg = 1;       //检测到有子弹发射出去&遥控器指令为单发时，先暂停一段时间 
                 else if(Shoot.shoot_flg == 1 && bopan.mode == bopan_danfa_mode)  
                    {
                          bopan.mode = bopan_danfa_mode;  //未检测到有子弹发出&与遥控器指令为单发时，进入单发模式
                          Delay_flg = 0;
                    }else if(bopan.mode == bopan_Duofa_mode && Shoot.cnt == Shoot.last_cnt+6 && Shoot.flag == 1)  Delay_flg = 1; 
										  else if(Shoot.shoot_flg == 1 && bopan.mode == bopan_Duofa_mode)
											{
                            bopan.mode=bopan_Duofa_mode;  //未检测到有子弹发出&与遥控器指令为多发时，进入多发模式
                            Delay_flg = 0;
											}else
												{        
													 Delay_flg = 0;
												}                       
    
    
		    if(Delay_flg || (Delay_time-moto_dial_get.cmd_time<DELAY_TIME &&Delay_time-moto_dial_get.cmd_time>0))      
         {
             Delay_time =  GetSystemTimer();
             bopan.mode = bopan_guodu_mode;
         }     
				      
				 bopan_check(set_cnt);//卡单检测	
//         if(!Shoot_heat.flag)
//           bopan.mode = bopan_Stop;
 /*判断发射模式*/
	 if(Mocha_Bopan)
	 {
    switch(bopan.mode)
    {
			case bopan_Stop://停止
			{
              set_angle=0;
              set_speed=0;
              set_cnt=0;
              moto_dial_get.cmd_time=GetSystemTimer();
			}break;
      case bopan_danfa_mode://按键单发模式
      {
              moto_dial_get.cmd_time=GetSystemTimer();
              set_speed=-2000;
              set_cnt=1;

              Shoot_heat.heat_forecast = 30;
      }break;
      case bopan_Duofa_mode://固定角度模式
      {
              moto_dial_get.cmd_time=GetSystemTimer();
              set_speed=-1500;
              set_cnt=1;
        Shoot_heat.heat_forecast = 30 ;
      }break;
      case bopan_Lianfa_mode://连发模式
      { 
            moto_dial_get.cmd_time=GetSystemTimer();
            set_speed=-3000;
            set_cnt=1;
				 Shoot_heat.heat_forecast = 28;
      }break;
			case bopan_shangdao_mode://上弹
      { 
            moto_dial_get.cmd_time=GetSystemTimer();
            set_speed=-500;
            set_cnt=1;
      }break;
      
      case bopan_guodu_mode://过渡
      { 
               set_speed=0;
               set_cnt=0;
      }break;
      
			case bopan_fanzhuan_mode://反转
			{
				
			moto_dial_get.reverse_time=GetSystemTimer();		
				
			if( moto_dial_get.reverse_time-moto_dial_get.REVE_time > REVERSE_TIME )//反转设定
			{
				bopan.flag=1;
				moto_dial_get.round_cnt=0;
				moto_dial_get.offset_angle=moto_dial_get.angle;
				moto_dial_get.total_angle=0;	
				set_cnt=0;
			}else 
			{			
				set_speed=1000;
			}
        
			/*pid位置环*/
			}break;
			default :break;
    }
     /*速度环*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     /*驱动拨弹电机*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
		 set_speed=0;

    
     if(bopan.last_mode!=Mocha_Duofa_mode && bopan.mode == Mocha_Duofa_mode)  Shoot.last_cnt = Shoot.cnt;
				else if(bopan.last_mode!=Mocha_danfa_mode && bopan.mode == Mocha_danfa_mode)  Shoot.danfa_cnt = Shoot.cnt;
         bopan.last_mode=bopan.mode;
     osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	  }
	}
}


void Mocha_Task(void const *argument)
{
	
	osDelay(500);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	ramp_init(&shoot,0.05,150,100);//抹茶轮斜坡
	
	for(;;)
	{
		
	if(Mocha_Bopan) 
	{
		
   switch(Shoot.mode)
    {
			case Mocha_Stop://停止
			{
				shoot.max_value=103;
				ramp_calc(&shoot,103);
	      Friction_Wheel_Motor(shoot.out,shoot.out);
			}break;
			
			case Mocha_danfa_mode://单发模式
      {
				shoot.max_value=128;
				ramp_calc(&shoot,128);
				Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
			
			case Mocha_Duofa_mode://固定角度模式
      {
				shoot.max_value=120;
				ramp_calc(&shoot,120);
				Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
      case Mocha_Lianfa_mode://连发模式
      {
				shoot.max_value=112;
				ramp_calc(&shoot,112);
				Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
			case 10://反转
			{
				
				
			}break;
			default :break;
			
		}
	}
  

	 osDelayUntil(&xLastWakeTime,Mocha_PERIOD);
		
	}
}



/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	   
	*	@retval	
****************************************************************************************/
void Shoot_Heat_Task(void const *argument)
{
  
  osDelay(500);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  
  
  for(;;)
  {
    
    Shoot_heat.val = Robot.heat.shoot_17_cooling_limit - Robot.heat.shoot_17_heat;//剩余热量
    
    if(Shoot_heat.val<Shoot_heat.heat_forecast)
    {
      Shoot_heat.flag = 0;
    }else Shoot_heat.flag = 1;
    
    osDelayUntil(&xLastWakeTime,10);
  }
   
}




//函数
/***************************************************************************************
**
	*	@brief	void bopan_check( uint8_t  Set_cnt )
	*	@param
	*	@supplement	  拨盘堵转检测
	*	@retval	
****************************************************************************************/
void bopan_check( uint8_t  Set_cnt )
{
	
	    if(bopan.flag  == 1)
			{
				  moto_dial_get.REVE_time=GetSystemTimer();//不反转计时
		     /*判断拨盘是否转到位置*/			
					if(my_abs(moto_dial_get.round_cnt) >=4*Set_cnt)
						{
							
										moto_dial_get.round_cnt=0;
										moto_dial_get.offset_angle=moto_dial_get.angle;
										moto_dial_get.total_angle=0;
										moto_dial_get.run_time=GetSystemTimer();
										bopan.flag=1;
						}
						else
						{
									if( my_abs(moto_dial_get.run_time-moto_dial_get.cmd_time)>BLOCK_TIME )//堵转判定
									{
									   	bopan.flag=0;	
									}
						}
					}
								
					if(bopan.flag == 0)   bopan.mode = bopan_fanzhuan_mode;//反转
	
}






