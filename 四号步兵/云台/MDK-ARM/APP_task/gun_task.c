/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
#include "gimbal_task.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
#define GUN_PERIOD  10
#define Mocha_PERIOD  100
#define BLOCK_TIME 2000
#define REVERSE_TIME 2000
#define DELAY_TIME  1000
/* �ⲿ��������--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
Mode_Set bopan = {0};	
Mode_Set Shoot_heat = {0};
ramp_function_source_t shoot;
extern uint8_t shot_frequency;
uint8_t finish_flag = 0;
static uint8_t Delay_flg;

//Power_Heat * power_heat;
/* �ⲿ����ԭ������-----------------------------------------------------------

-----------------------------------------------------------------------------
-*/
/* �ڲ�����------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //���̵��λ�û�
pid_t pid_dial_spd  = {0};	//���̵���ٶȻ�
/* �ڲ�����ԭ������----------------------------------------------------------*/
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
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	��������
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
  /*�趨����*/
    bopan.flag=1;
	for(;;)
	{
		
		RefreshTaskOutLineTime(GunTask_ON);
		  
     if(Shoot.flag == 2 )  
				{
					bopan.mode = bopan_shangdao_mode;
					Delay_flg=0;
				}else if(Shoot.shoot_flg  == 1 && bopan.mode == bopan_shangdao_mode)  bopan.mode=bopan_Stop;
             else if(Shoot.flag == 1 && bopan.mode == bopan_danfa_mode &&  Shoot.cnt == Shoot.danfa_cnt+2)  Delay_flg = 1;       //��⵽���ӵ������ȥ&ң����ָ��Ϊ����ʱ������ͣһ��ʱ�� 
                 else if(Shoot.shoot_flg == 1 && bopan.mode == bopan_danfa_mode)  
                    {
                          bopan.mode = bopan_danfa_mode;  //δ��⵽���ӵ�����&��ң����ָ��Ϊ����ʱ�����뵥��ģʽ
                          Delay_flg = 0;
                    }else if(bopan.mode == bopan_Duofa_mode && Shoot.cnt == Shoot.last_cnt+6 && Shoot.flag == 1)  Delay_flg = 1; 
										  else if(Shoot.shoot_flg == 1 && bopan.mode == bopan_Duofa_mode)
											{
                            bopan.mode=bopan_Duofa_mode;  //δ��⵽���ӵ�����&��ң����ָ��Ϊ�෢ʱ������෢ģʽ
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
				      
				 bopan_check(set_cnt);//�������	
//         if(!Shoot_heat.flag)
//           bopan.mode = bopan_Stop;
 /*�жϷ���ģʽ*/
	 if(Mocha_Bopan)
	 {
    switch(bopan.mode)
    {
			case bopan_Stop://ֹͣ
			{
              set_angle=0;
              set_speed=0;
              set_cnt=0;
              moto_dial_get.cmd_time=GetSystemTimer();
			}break;
      case bopan_danfa_mode://��������ģʽ
      {
              moto_dial_get.cmd_time=GetSystemTimer();
              set_speed=-2000;
              set_cnt=1;

              Shoot_heat.heat_forecast = 30;
      }break;
      case bopan_Duofa_mode://�̶��Ƕ�ģʽ
      {
              moto_dial_get.cmd_time=GetSystemTimer();
              set_speed=-1500;
              set_cnt=1;
        Shoot_heat.heat_forecast = 30 ;
      }break;
      case bopan_Lianfa_mode://����ģʽ
      { 
            moto_dial_get.cmd_time=GetSystemTimer();
            set_speed=-3000;
            set_cnt=1;
				 Shoot_heat.heat_forecast = 28;
      }break;
			case bopan_shangdao_mode://�ϵ�
      { 
            moto_dial_get.cmd_time=GetSystemTimer();
            set_speed=-500;
            set_cnt=1;
      }break;
      
      case bopan_guodu_mode://����
      { 
               set_speed=0;
               set_cnt=0;
      }break;
      
			case bopan_fanzhuan_mode://��ת
			{
				
			moto_dial_get.reverse_time=GetSystemTimer();		
				
			if( moto_dial_get.reverse_time-moto_dial_get.REVE_time > REVERSE_TIME )//��ת�趨
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
        
			/*pidλ�û�*/
			}break;
			default :break;
    }
     /*�ٶȻ�*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     /*�����������*/
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
	ramp_init(&shoot,0.05,150,100);//Ĩ����б��
	
	for(;;)
	{
		
	if(Mocha_Bopan) 
	{
		
   switch(Shoot.mode)
    {
			case Mocha_Stop://ֹͣ
			{
				shoot.max_value=102;
				ramp_calc(&shoot,102);
	      Friction_Wheel_Motor(shoot.out,shoot.out);
			}break;
			
			case Mocha_danfa_mode://����ģʽ
      {
				shoot.max_value=130;
				ramp_calc(&shoot,130);
				Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
			
			case Mocha_Duofa_mode://�̶��Ƕ�ģʽ
      {
				shoot.max_value=120;
				ramp_calc(&shoot,120);
				Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
      case Mocha_Lianfa_mode://����ģʽ
      {
				shoot.max_value=115;
				ramp_calc(&shoot,115);
				Friction_Wheel_Motor(shoot.out,shoot.out);
      }break;
			case 10://��ת
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
    
    Shoot_heat.val = Robot.heat.shoot_17_cooling_limit - Robot.heat.shoot_17_heat;//ʣ������
    
    if(Shoot_heat.val<Shoot_heat.heat_forecast)
    {
      Shoot_heat.flag = 0;
    }else Shoot_heat.flag = 1;
    
    osDelayUntil(&xLastWakeTime,10);
  }
   
}




//����
/***************************************************************************************
**
	*	@brief	void bopan_check( uint8_t  Set_cnt )
	*	@param
	*	@supplement	  ���̶�ת���
	*	@retval	
****************************************************************************************/
void bopan_check( uint8_t  Set_cnt )
{
	
	    if(bopan.flag  == 1)
			{
				  moto_dial_get.REVE_time=GetSystemTimer();//����ת��ʱ
		     /*�жϲ����Ƿ�ת��λ��*/			
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
									if( my_abs(moto_dial_get.run_time-moto_dial_get.cmd_time)>BLOCK_TIME )//��ת�ж�
									{
									   	bopan.flag=0;	
									}
						}
					}
								
					if(bopan.flag == 0)   bopan.mode = bopan_fanzhuan_mode;//��ת
	
}






