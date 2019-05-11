/* 包含头文件----------------------------------------------------------------*/
#include "led_task.h"
#include "SystemState.h"
#include "gpio.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "usart.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
#define LED_PERIOD  600
#define Check_PERIOD  100
/* 外部变量声明--------------------------------------------------------------*/

SystemStateDef Remote;
SystemStateDef Gimbal_Motor;
SystemStateDef JY61;
/* 外部函数原型声明-----------------------------------------------------------

-----------------------------------------------------------------------------
-*/
/* 内部变量------------------------------------------------------------------*/
/* 内部函数原型声明----------------------------------------------------------*/

/* 任务主体部分 -------------------------------------------------------------*/



void Led_Task(void const * argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		RefreshTaskOutLineTime(LedTask_ON);
		

				if((SystemState.OutLine_Flag&0x01))//遥控器掉线
				{
						HAL_GPIO_TogglePin(GPIOG,LED1_Pin);
					  Remote.Mode=1;
					  printf("remote over");
						osDelayUntil(&xLastWakeTime,200);		  
				}else  
				{
						Remote.Mode=0;
						HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_RESET);
				}
				
				if((SystemState.OutLine_Flag&0x02))
				{
						HAL_GPIO_TogglePin(GPIOG,LED2_Pin);
					  Gimbal_Motor.Mode=1;
					  printf("motor_Y over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_RESET);
						Gimbal_Motor.Mode=0;
				}

				if((SystemState.OutLine_Flag&0x04))
				{
						HAL_GPIO_TogglePin(GPIOG,LED3_Pin);
				  	Gimbal_Motor.Mode=2;
					  printf("motor_P over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						Gimbal_Motor.Mode=0;
						HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_RESET);
				}
				if((SystemState.OutLine_Flag&0x08))
				{
						HAL_GPIO_TogglePin(GPIOG,LED4_Pin);
					  Gimbal_Motor.Mode=3;
					  printf("motor_B over");
						osDelayUntil(&xLastWakeTime,200);
				} else 
        {	
					Gimbal_Motor.Mode=0;
					HAL_GPIO_WritePin(GPIOG,LED4_Pin,GPIO_PIN_RESET);
				}
				if((SystemState.OutLine_Flag&0x10))
				{
					  JY61.Mode=1;
					  printf("JY61 over");
						HAL_GPIO_TogglePin(GPIOG,LED5_Pin);
						osDelayUntil(&xLastWakeTime,200);
					  
				} else  
				{
					JY61.Mode=0;
					HAL_GPIO_WritePin(GPIOG,LED5_Pin,GPIO_PIN_RESET);
				}
	  	osDelayUntil(&xLastWakeTime,LED_PERIOD);
		
	 }
 
}



void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
	if((SystemState.task_OutLine_Flag&0x01))
				{
					printf("testTask GG \n\t");
					osDelayUntil(&xLastWakeTime,100);
				}
				
				if((SystemState.task_OutLine_Flag&0x02))
				{
						printf("RemoteDataTask GG \n\t");
			      Remote_Disable();
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag&0x04))
				{
						printf("GimbalContrlTask GG \n\t");
					  Cloud_Platform_Motor_Disable(&hcan1);
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag&0x08))
				{
						printf("GunTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag&0x10))
				{
						printf("LedTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 


				if((SystemState.task_OutLine_Flag&0x20))
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 

		
				osDelayUntil(&xLastWakeTime,Check_PERIOD);
	
			}
	
	
}
