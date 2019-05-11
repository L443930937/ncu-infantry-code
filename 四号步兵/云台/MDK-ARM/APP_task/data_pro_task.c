/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "data_pro_task.h"
#include "SystemState.h"
#include "user_lib.h"
#include "main.h"
#include "AX-12A.h"
/* �ڲ��궨��----------------------------------------------------------------*/
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
/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
pid_t pid_minipc_yaw={0};
pid_t pid_minipc_pit={0};
Mode_Set Shoot_mouse={0};
#define REMOTE_PERIOD 1
#define MINIPC_PERIOD 10
/* �ⲿ��������--------------------------------------------------------------*/

/* ���õ��ⲿ����ԭ������------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* �ڲ�����------------------------------------------------------------------*/
static uint8_t press_counter;

//volatile float remain_power=0.0;   //���̹��� _����
//float power; 				 //���̹��� _����

//float chassis_Current; 
//float	 chassis_Volt; 
/* �ڲ�����ԭ������-----------------------------------------------------------*/


void Minipc_Pid_Init()
{
		PID_struct_init(&pid_minipc_yaw, POSITION_PID, 50, 1,
									1.2f,	0.01f, 1.0f);  
		PID_struct_init(&pid_minipc_pit, POSITION_PID, 6000, 5000,
									1.2f,	0.01f, 1.0f	);   

}
/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	��ң�������жԽӣ���ң���������ݽ��д���ʵ�ֶԵ��̡���̨����������Ŀ���
	*	@retval	
****************************************************************************************/
void RemoteControlProcess()  
{
          if(Gimbal.mode == 1 ) // ��̨���޺��ٸ�������
					{
								pit_set.expect_remote = pit_set.expect_remote +(0x400-RC_Ctl.rc.ch3)/20;	
								yaw_tly_set.expect_remote = yaw_tly_set.expect_remote +(0x400-RC_Ctl.rc.ch2)/20;	
            
					}else if(Gimbal.mode == 3)
					{
							pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
							yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;
					}
		      
					

              if(RC_Ctl.rc.s1==1)
                {    
                     Shoot.mode=1;
                     if(shoot.out == 130) 
                  bopan.mode=1;    
                   Set_AX6(2,0x0ff,0xff);
//        					Minipc.flag=1;
//       						Minipc.mode=1;
                     
                }
                else if(RC_Ctl.rc.s1==2)
                {
                     Shoot.mode=3;
                     if(shoot.out == 115) 
                  bopan.mode=3;    
                  Set_AX6(2,0x3ff,0xff);
                }else 
                {
							    bopan.mode=0;
                  Gimbal.mode=1;
									Minipc.flag=0;
									Minipc.mode=0;
                }			
					
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	�Լ�������ݽ��д���
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
			static uint8_t shut_flag;
      static uint8_t shoot_mouse_flg;
//  					Minipc.flag=2;
//						Minipc.mode=2;
//	
					//��꣨�ƶ��ٶ�*1000/50��
					pit_set.expect_remote = pit_set.expect_remote+RC_Ctl.mouse.y*1.7;	
					yaw_tly_set.expect_remote = yaw_tly_set.expect_remote-RC_Ctl.mouse.x*2;	
				
					
					if(RC_Ctl.mouse.press_l==1)        //����������
					{															
								Shoot_mouse.flag=1;
										
					} else
                {
                  Shoot_mouse.flag=0;
                }
				 	if(RC_Ctl.key.v & 0x100)//r���л�����ģʽ
					{
						press_counter++;
								if(press_counter>=50)  // ����ϵͳ��ӡ  ��ģʽ
								{
									
											press_counter=0;
											Shoot_mouse.mode++;
									
								}
								if(Shoot_mouse.mode>3) Shoot_mouse.mode=1;
								
              }
								//����ģʽ�л�
					if(Minipc.flag == 0)
          {
												 switch(Shoot_mouse.mode )
												 {
															 case 1:
															 {
																	Shoot.mode=1; 
                                 if(shoot.out == 130 && Shoot_mouse.flag)  bopan.mode = 1;
                                 else bopan.mode = 0;
															 }break;
															 case 2:
															 {
																	 Shoot.mode=2; 
                                 if(shoot.out == 120  && Shoot_mouse.flag)  bopan.mode = 2;
                                 else bopan.mode = 0;
															 }
															 case 3:
															 {
																	 Shoot.mode=3; 
                                 if(shoot.out == 115  && Shoot_mouse.flag)  bopan.mode = 3;
                                 else bopan.mode = 0;
                             
															 }break;
															 
															 default : break;
                             }
                   }       
                             
                             
                         if( RC_Ctl.key.v & 0x40 ) //Q
                         {
                           Gimbal.mode = 3;
                           yaw_tly_set.expect_remote = tly.final_angle ; //���������ǵ�ֵ
                         }else if( RC_Ctl.key.v & 0x80 )//E
                         {
                           Gimbal.mode = 3;
                           yaw_tly_set.expect_remote = tly.final_angle ; //���������ǵ�ֵ
                         }else 
                         {
                           Gimbal.mode = 1;
                           yaw_set.expect = yaw_get.total_angle;
                           
                         }
                         
                        
                         
                         
                         
                         
                         CAN_Send_cilent(&hcan2,Shoot_mouse.mode,0,0,0);
                
                   
}

/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	����ֹͣ����
	*	@retval	
****************************************************************************************/
void hard_brak()
{

	 Gimbal.mode=1;
	
}


/* �������岿�� -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	ң�����ݽ��ռ���������
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
	uint32_t NotifyValue;

	
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	
	for(;;)
	{
		
		   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
			
			NotifyValue=0;
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14); //GRE_main
			
			RefreshTaskOutLineTime(RemoteDataTask_ON);
			Remote_Ctrl();
			Send_MiniPC_Data(Minipc.flag,Minipc.mode,0);
			CAN_Send_YK(&hcan2,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
				switch(RC_Ctl.rc.s2)
				{
					case 1: RemoteControlProcess();break; 
					case 2: MouseKeyControlProcess();break; 
					case 3: hard_brak();break;
					default :break;
				}					

            press_counter++;
		}
		
		if(Remote.Mode)//ң��������
		{
		    Remote_Disable();
		}
		
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}

/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	�Ӿ����ݴ�������
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
	static uint8_t frist_flag = 0;
  
  static int16_t  frist_angle;//pit�ĽǶ�ƫ��
	minipc_rx.state_flag = 0;
	minipc_rx.angle_pit  = 0;
	minipc_rx.angle_yaw  = 0;
  uint32_t NotifyValue;
	Minipc_Pid_Init();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
		frist_angle  =  pit_get.total_angle-pit_set.expect_pc;
	for(;;)
	{
		

	   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{

			Get_MiniPC_Data();
			
			if(Minipc.flag)
			{
						switch(Minipc.mode)
						{
									case 1 ://����
									{
								      pid_calc(&pid_minipc_yaw,minipc_rx.angle_yaw,0);
											yaw_set.expect_pc +=pid_minipc_yaw.pos_out;
											pit_set.expect_pc += minipc_rx.angle_pit;
									}break ;
									
									case 2 ://���						
									{
										
											 Shoot.mode = 1; //������Ĩ����
											if( frist_flag == 0)
                      {
												if(shoot.out == 130 )  frist_flag=1;
											
													yaw_set.expect_pc=minipc_rx.angle_yaw-yaw_get.offset_angle;
													pit_set.expect_pc=minipc_rx.angle_pit-pit_get.offset_angle;

											}
										
                     if(minipc_rx.state_flag)
                     {
                       yaw_set.expect_pc=minipc_rx.angle_yaw-yaw_get.offset_angle;
											pit_set.expect_pc=minipc_rx.angle_pit-pit_get.offset_angle;

                     }
										else if( frist_flag && my_abs(pit_set.err)<10&& my_abs(yaw_set.err)<20)
											{
										    
									   			yaw_set.expect_pc=minipc_rx.angle_yaw-yaw_get.offset_angle ;               
													pit_set.expect_pc=minipc_rx.angle_pit-pit_get.offset_angle ;
	                         bopan.mode = 1;
                     
											}else     bopan.mode = 0;
                      
                        yaw_set.err = yaw_set.expect_pc-yaw_get.total_angle;
                        pit_set.err = pit_set.expect_pc-pit_get.total_angle;
                      
							
									}break;
									default :  
										break;
						}
					}
			osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
		}
	}
}

