#include "BSP.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

xQueueHandle UART1_RX_QueHandle;//串口1接收队列
xQueueHandle UART2_RX_QueHandle;//串口2接收队列
xQueueHandle UART6_RX_QueHandle;//串口6接收队列
xQueueHandle UART8_RX_QueHandle;//串口8接收队列

void Power_Init(void)
{
#if BoardNew

HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);   //power1
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);   //power2
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);   //power3
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //power4

#endif
	HAL_Delay(50);
}

/**
	**************************************************************
	** Descriptions:	JY61休眠/解休眠
	** Input:	huart  发送指令的串口，波特率要求为115200
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void JY61_SLEEPorUNSLEEP(UART_HandleTypeDef *huart)
{
	uint8_t buff[3] = {0xff,0xaa,0x60};
	//休眠,解休眠
	HAL_UART_Transmit(huart,buff,3,10);
}

/**
	**************************************************************
	** Descriptions: JY61帧对齐函数
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void JY61_Frame(void)
{
	static uint8_t JY61_Frame_flag = 0;
	static	uint8_t JY61_Frame_Num = 0;
	
while( UART8_RX_DATA[0] != 0x55 ||  JY61_Frame_flag == 1)
{
	
	if(UART8_RX_DATA[0] != 0x55 && JY61_Frame_flag == 0)
	{
				
				HAL_UART_DMAPause(&huart8);
				*UART8_RX_DATA = 0;
				JY61_Frame_flag = 1;
				
	}
	if(JY61_Frame_flag == 1)//休眠一次，必须解休眠
	{
			JY61_Frame_Num++;
			
					if(JY61_Frame_Num == 25)
					 {
						 
//								JY61_SLEEPorUNSLEEP(&huart8);
//								JY61_Frame_flag = 0;
//								JY61_Frame_Num = 0;
							
								HAL_UART_Receive_DMA(&huart8,UART8_RX_DATA,SizeofJY901);	//陀螺仪接收

				   } else if(JY61_Frame_Num == 50)
							 {
								   HAL_UART_DMAResume(&huart8);
							 } else if(JY61_Frame_Num > 100  )
									 {
										 JY61_Frame_flag = 0;
							       JY61_Frame_Num = 0;
									 }

	 }
}
	
}



void JY901_Init(void)
{
	uint8_t JY901[6][5] = {
													{0xff,0xaa,0x24,0x01,0x00},//六轴算法
													{0xff,0xaa,0x02,0x00,0x00},//开启自动校准
													{0xff,0xaa,0x02,0x0c,0x00},//回传内容:0x0c是输出速度和角度//0x08是只输出角度
													{0xff,0xaa,0x03,0x0b,0x00},//回传速率:200hz
													{0xff,0xaa,0x00,0x00,0x00},//保存当前配置
													{0xff,0xaa,0x04,0x06,0x00}//设置串口波特率:115200
												};
		
	HAL_UART_Transmit_DMA(&huart8,JY901[2],5);
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart8,JY901[3],5);
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart8,JY901[4],5);	
	HAL_Delay(100);
	if(HAL_UART_Transmit_DMA(&huart8,JY901[2],5) == HAL_OK )	
	{
		printf("JY901 Init \n\r");
	}
		if(HAL_UART_Transmit_DMA(&huart8,JY901[4],5) == HAL_OK)	
	{
		printf("JY901 Init save\n\r");
	}
}











void ConfigureTimerForRunTimeStats(void)  //时间统计
{
	FreeRTOSRunTimeTicks = 0;
	MX_TIM3_Init(); //周期50us，频率20K
}


void BSP_Init(void)
{
	
	/*引脚和引脚时钟*/
  MX_GPIO_Init();
	HAL_Delay(1000);
	Power_Init();
	/*dma*/
  MX_DMA_Init();
	/*can*/
	MX_CAN1_Init();
	MX_CAN2_Init();	
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	/*定时器*/
  MX_TIM5_Init();
	MX_TIM6_Init();
	SystemState_Inite();
  /*ADC*/
	MX_ADC1_Init();
	/*串口*/
  MX_UART8_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();	
	/*SPI*/
	MX_SPI5_Init();

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	
	/*使能DMA接收*/
	Bsp_UART_Receive_IT(&huart1,USART1_RX_DATA,SizeofRemote); //这一步的目的是创建一段接受内存
	Bsp_UART_Receive_IT(&huart8,UART8_RX_DATA,SizeofJY901);
	Bsp_UART_Receive_IT(&huart6,USART6_RX_DATA,SizeofMinipc);


/*开启ADC的DMA接收，注意缓存不能小于2，不能设置为_IO型即易变量*/
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, 10); 

	/*使能can中断*/
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); 
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	
	/*陀螺仪*/
	 MPU6500_Init();
	/*摩擦轮*/
	GUN_Init();
	HAL_Delay(1000);
  
  huart6.gState = HAL_UART_STATE_READY;
}
