/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "freertostask.h"
#include "mpu6050.h"
#include "oled.h"
#include "nrf_demo.h"
#include "nrf24l01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SATURATE 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void OLED_ShowPacket(const MousePacket_t *p)
{
	/* 行2: X:+010 Y:+010 */
	OLED_ShowSignedNum(2, 3, p->x, 3);
	OLED_ShowSignedNum(2, 10, p->y, 3);

	/* 行3: L:0 R:0 GX:+010 */
	OLED_ShowChar(3, 3, (p->buttons & 0x01) ? '1' : '0');  /* 左键 */
	OLED_ShowChar(3, 7, (p->buttons & 0x02) ? '1' : '0');  /* 右键 */
	OLED_ShowSignedNum(3, 12, p->gx, 3);

	/* 行4: GY:+010 GZ:+010 */
	OLED_ShowSignedNum(4, 4, p->gy, 3);
	OLED_ShowSignedNum(4, 12, p->gz, 3);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile unsigned long g_ulRunTimeCounter = 0;
int8_t ButtonState=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_Base_Start_IT(&htim3);
	HAL_Delay(500);
  MPU6050_Init();
	HAL_Delay(500);
	//OLED_Init();
	NRF_Demo_Init();		
		
	uint32_t mpu_last_time=0;
	uint32_t nrf_last_time=0;
	uint32_t stop_last_time=0;
	uint16_t calibration=0;
	MousePacket_t nrftx_pack;
	int16_t Gyro_x=0;
	int16_t Gyro_y=0;
  int16_t Gyro_z=0;
	int16_t gyro_x_dps;
	int16_t gyro_z_dps;
	int16_t gyro_y_dps;
	int8_t tmp0=0;
	int8_t tmp1=0;
	int16_t lastgx=0;
	int16_t lastgz=0;
	int8_t lastb=0;
	int8_t x=0;
	int8_t y=0;
	int8_t botton=0;
	int16_t appgz_offest=0,appgy_offest=0,appgx_offest=0;
	mpu_last_time=HAL_GetTick();
	stop_last_time=HAL_GetTick();
	nrf_last_time=HAL_GetTick();
	uint8_t sta;
	
	//freertos_task_start();
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(HAL_GetTick()-mpu_last_time>5){
			MPU6050_GetData((int16_t*)&Gyro_x, (int16_t*)&Gyro_y, (int16_t*)&Gyro_z);
			gyro_x_dps = (Gyro_x / 64) -appgx_offest;
			gyro_y_dps = (Gyro_y / 64) -appgy_offest;
			gyro_z_dps = (Gyro_z / 64) -appgz_offest;			
			
			gyro_x_dps=15*gyro_x_dps/16+lastgx/16;
			gyro_z_dps=15*gyro_z_dps/16+lastgz/16;
			
			if(gyro_z_dps>2||gyro_z_dps<-2){
				tmp0=(gyro_x_dps>0)?gyro_x_dps:-gyro_x_dps;				
			}
			if(gyro_x_dps>2||gyro_x_dps<-2){
				tmp1=(gyro_z_dps>0)?gyro_z_dps:-gyro_z_dps;				
			}
			
			x=(gyro_z_dps>0)?-(4*gyro_z_dps+tmp0)/8: (tmp0-4*gyro_z_dps)/8;tmp0=0;		
			//out.y=-gyro_x_dps/4;
			y=(gyro_x_dps>0)?-(4*gyro_x_dps+tmp1)/12: (tmp1-4*gyro_x_dps)/12;tmp1=0;		
			if (x > SATURATE) x = SATURATE;
      if (x < -SATURATE) x = -SATURATE;
      if (y > SATURATE) y = SATURATE;
      if (y < -SATURATE) y = -SATURATE;
			
			botton=ButtonState;
		
			if(botton!=lastb || x>2 ||y>2||x<-2||y<-2){							
				calibration=0;
				
	
			nrftx_pack.buttons=botton;
			nrftx_pack.x=x;
			nrftx_pack.y=y;
			nrftx_pack.gx=Gyro_x;
			nrftx_pack.gy=Gyro_y;
			nrftx_pack.gz=Gyro_z;
			nrftx_pack.seq++;

			sta = NRF24L01_TxPacket((uint8_t*)&nrftx_pack);
				
			}
			else{
				//xQueueSend(mygyro,&stopdata,10);
				calibration++;
				if(calibration==60000){
					;
					calibration=0;
				}
			}
			
			lastb=botton;
			lastgx=gyro_x_dps;
			lastgz=gyro_z_dps;
		}
		
		
		
		
		/*if(HAL_GetTick()-nrf_last_time>=10){
			uint8_t sta;
	
			nrftx_pack.buttons=botton;
			nrftx_pack.x=x;
			nrftx_pack.y=y;
			nrftx_pack.gx=Gyro_x;
			nrftx_pack.gy=Gyro_y;
			nrftx_pack.gz=Gyro_z;
			nrftx_pack.seq++;

			sta = NRF24L01_TxPacket((uint8_t*)&nrftx_pack);
			
		}*/
		
		if(HAL_GetTick()-stop_last_time>=500){
			OLED_ShowPacket(&nrftx_pack);
			if (sta == TX_OK)       OLED_ShowString(1, 1, "TX OK   ");
			else if (sta == MAX_TX) OLED_ShowString(1, 1, "TX MAXRT"); 
			else                    OLED_ShowString(1, 1, "TX FAIL ");
		}
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM2){
			 HAL_IncTick();
		}
		if(htim->Instance==TIM3){
			 //g_ulRunTimeCounter++;
		}
			
}





/* USER CODE BEGIN 4 */
/*void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{

    ( void ) xTask;
    ( void ) pcTaskName;
    

    while( 1 )
    {

    }
}*/



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    
    if (GPIO_Pin == GPIO_PIN_3)
    {
				uint8_t state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
        
				if(state) ButtonState |= 0x01;
				else ButtonState &= ~0x01;
    }
		if (GPIO_Pin == GPIO_PIN_4)
    {
				uint8_t state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
        
				if(state)ButtonState |= 0x02;
				else ButtonState &= ~0x02;
    }
		
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
