/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "Power_restriction.h"
#include "SystemState.h"
#include "user_lib.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;
/* 任务相关信息定义----------------------------------------------------------*/

/* 内部常量定义--------------------------------------------------------------*/
#define GIMBAL_PERIOD 5
/* 外部变量声明--------------------------------------------------------------*/
Pos_Set  yaw_set={0};
Pos_Set  yaw_tly_set={0};
Pos_Set  pit_set={0};
Mode_Set Gimbal={0};
Mode_Set Minipc={0};
Mode_Set Shoot={0};

/* 调用的外部函数原型声明------------------------------------------------------------
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
float pid_calc(pid_t* pid, float get, float set);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_yaw_jy901 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_pit_dashen = {0};       //大神符参数
pid_t pid_pit_dashen_spd = {0};
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环
pid_t pid_yaw_jy901_spd = {0};
pid_t pid_pit_jy901 = {0};
pid_t pid_pit_jy901_spd = {0};

pid_t pid_yaw_saber = {0};  //外接陀螺仪 /*目前只用于位置环*/
pid_t pid_yaw_saber_spd = {0};
pid_t pid_pit_saber = {0};
pid_t pid_pit_saber_spd = {0};

//zimiao
pid_t pid_yaw_zimiao = {0};        //
pid_t pid_yaw_zimiao_spd = {0};
pid_t pid_pit_zimiao = {0};        //
pid_t pid_pit_zimiao_spd = {0};
/* 内部函数原型声明----------------------------------------------------------*/
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
	
  /* yaw axis motor pid parameter */
  //大神符模式
   PID_struct_init(&pid_pit_dashen, POSITION_PID, 5000, 1000,
                  4.0f, 0.05f, 0.5f);  
   PID_struct_init(&pid_pit_dashen_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 500,
                  6.0f, 0.05f, 5.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
		
  //自瞄模式
   PID_struct_init(&pid_pit_zimiao, POSITION_PID, 5000, 1000,
                  4.0f, 0.03f, 2.0f);  //4.0 0.03 2.0
   PID_struct_init(&pid_pit_zimiao_spd, POSITION_PID, 5000, 1000,
                  5.0f, 0.0f, 0.0f );  //1.5 0.0 0.0
  
	 PID_struct_init(&pid_yaw_zimiao, POSITION_PID, 5000, 300,
                  15.0f, 0.01f, 25.0f); // 6.0 0.05 5.0 //15.0 0.01 25.0
	 PID_struct_init(&pid_yaw_zimiao_spd, POSITION_PID, 5000, 1000,
                  1.5f, 0.0f, 0.0f );  
  
  //陀螺仪模式
  PID_struct_init(&pid_pit, POSITION_PID, 5000, 100,
                  4.5f, 0.01f, 0.0f); 
  PID_struct_init(&pid_pit_jy901_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  
	 PID_struct_init(&pid_yaw_jy901, POSITION_PID, 5000, 500,
                  3.0f, 0.02f, 5.0f); 
	 PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 5000, 500,
                  2.0, 0.0f, 1.0f );	
	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{

	yaw_set.mode   = 0;
//	Gim 
	Gimbal.flag=0;
	Gimbal.mode=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	gimbal_pid_init();
	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)		
    {
			
		 IMU_Get_Data();
		 CAN_Send_YT(&hcan2,yaw_get.angle,yaw_get.total_angle,Minipc.mode,Gimbal.mode,Remote.Mode);//主控发送
			
	   RefreshTaskOutLineTime(GimbalContrlTask_ON);
      
		 	
			if(Minipc.flag == 2)   //打符
			{
            Gimbal.mode=3;
            yaw_set.expect = yaw_set.expect_pc;
            pit_set.expect = pit_set.expect_pc;
        
            yaw_tly_set.expect_remote = tly.final_angle;
            pit_set.expect_remote = pit_get.total_angle;
			}else if(Minipc.flag == 1  && Gimbal.flag == 0) //自瞄 不超限
			{
            Gimbal.mode=2;
            yaw_set.expect =  yaw_set.expect_pc;
            pit_set.expect =  pit_set.expect_pc;
        
            yaw_tly_set.expect_remote = tly.final_angle;
            pit_set.expect_remote = pit_get.total_angle;
			}else   //遥控
			{
            yaw_tly_set.expect = yaw_tly_set.expect_remote;
            pit_set.expect = pit_set.expect_remote;
			}
			
			
			if(yaw_get.angle<2044)//云台超限处理 you
			{
            yaw_set.expect=2100-yaw_get.offset_angle;
            Gimbal.mode=3;
            Gimbal.flag=2;     //超限标志位
			}else if(yaw_get.angle>5966) //zuo
			{
            yaw_set.expect=  5900 - yaw_get.offset_angle ;
            Gimbal.mode=3;
            Gimbal.flag=2;     //超限标志位       
			}
			else if ( Gimbal.flag == 2)
			{   
            Gimbal.flag=0;
        yaw_tly_set.expect = tly.final_angle + minipc_rx.angle_yaw;
			}
     
		


 
#if 0			
			if ((pit_get.angle > 1400) && (pit_get.angle < 4600))
			{
//				if (pit_set.expect_remote <= pit_set.expect_remote_last)
//					goto pit_calc;
				pit_set.expect_remote =  1350 - pit_get.offset_angle ;
				pit_set.expect =	1350 - pit_get.offset_angle ;
			}
			if ((pit_get.angle >4600) && (pit_get.angle <7800))
			{
//				if (pit_set.expect_remote >= pit_set.expect_remote_last)
//					goto pit_calc;
				pit_set.expect_remote =  7950 - pit_get.offset_angle ;
				pit_set.expect =	7950 - pit_get.offset_angle ;
			}
#endif
			if (pit_get.angle < 4142)
			{
				
				pit_set.expect_remote =  4250 - pit_get.offset_angle ;
				pit_set.expect =	4250 - pit_get.offset_angle ;
			}
			else if (pit_get.angle > 5880)
			{
				
				pit_set.expect_remote =  5800 - pit_get.offset_angle ;
				pit_set.expect =	5800 - pit_get.offset_angle ;
			}
	

			switch(Gimbal.mode)
			{	
				case 3: {
					          
										 pid_calc(&pid_yaw, yaw_get.total_angle,yaw_set.expect);
										 pid_calc(&pid_yaw_spd,(imu_data.gx)/30, pid_yaw.pos_out);
					    Yaw_Current_Value= (-pid_yaw_spd.pos_out);
          
                     pid_calc(&pid_pit_dashen, pit_get.total_angle, pit_set.expect);
                     pid_calc(&pid_pit_dashen_spd,(imu_data.gz)/16.4, pid_pit_dashen.pos_out);
                
              Pitch_Current_Value=(-pid_pit_dashen_spd.pos_out); 
          
				        }break;                                                                           //编码器模式
				case 1: {
										 pid_calc(&pid_yaw_jy901,(tly.final_angle),yaw_tly_set.expect);
										 pid_calc(&pid_yaw_jy901_spd,(imu_data.gx)/30, pid_yaw_jy901.pos_out);
					    Yaw_Current_Value= (-pid_yaw_jy901_spd.pos_out);
          
          			//pit轴
					pit_calc:	pid_calc(&pid_pit, pit_get.total_angle,pit_set.expect);
                    pid_calc(&pid_pit_jy901_spd,(imu_data.gz)/16.4, pid_pit.pos_out);
                
              Pitch_Current_Value=(-pid_pit_jy901_spd.pos_out); 
				        }break;//陀螺仪模式
         case 2: {//zimiao
										 pid_calc(&pid_yaw_zimiao,yaw_get.total_angle,yaw_set.expect);
										 pid_calc(&pid_yaw_zimiao_spd,(imu_data.gx)/30, pid_yaw_zimiao.pos_out);
					    Yaw_Current_Value= (-pid_yaw_zimiao_spd.pos_out);
          
          			//pit轴
                    pid_calc(&pid_pit_zimiao, pit_get.total_angle, pit_set.expect);
                    pid_calc(&pid_pit_zimiao_spd,(imu_data.gz)/16.4, pid_pit_zimiao.pos_out);
                
              Pitch_Current_Value=(-pid_pit_zimiao_spd.pos_out); 
				        }break;
//         case 4 ://回中
//         {
//           
//         }break;
				default:
				break;
				
			}                                                                             
		 		
					Cloud_Platform_Motor(&hcan1,-Yaw_Current_Value,Pitch_Current_Value);
					pit_set.expect_remote_last = pit_set.expect_remote;
			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
			
   }
 
}
