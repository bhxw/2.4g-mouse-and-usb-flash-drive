
#include "freertostask.h"


typedef struct {
    uint32_t magic;
    int16_t gyro_x_offset;
    int16_t gyro_y_offset;
    int16_t gyro_z_offset;

} CalibrationData;

#define CALIBRATION_FLASH_ADDR  0x0800FC00 // 64KB Flash的最后1KB起始地址

void Save_Calibration_To_Flash(CalibrationData *pData) {

FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t PageError = 0;
uint32_t arr = CALIBRATION_FLASH_ADDR;
HAL_FLASH_Unlock(); 

EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; 
EraseInitStruct.PageAddress = 0x0800FC00;          
EraseInitStruct.NbPages = 1;                       

if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
	printf("%d",PageError);
	HAL_FLASH_Lock();
	return ;
}

if(HAL_FLASH_Program(TYPEPROGRAM_WORD, arr, pData->magic)!= HAL_OK){
	printf("WriteERROR!!");
	HAL_FLASH_Lock();
	return ;
}
arr+=4;
if(HAL_FLASH_Program(TYPEPROGRAM_WORD, arr, pData->gyro_x_offset)){
	printf("WriteERROR!!");
	HAL_FLASH_Lock();
	return ;
}
arr+=4;
if(HAL_FLASH_Program(TYPEPROGRAM_WORD, arr, pData->gyro_y_offset)){
	printf("WriteERROR!!");
	HAL_FLASH_Lock();
	return ;
}
arr+=4;
if(HAL_FLASH_Program(TYPEPROGRAM_WORD, arr, pData->gyro_z_offset)){
	printf("WriteERROR!!");
	HAL_FLASH_Lock();
	return ;
}

HAL_FLASH_Lock(); 

}

void Load_Calibration_From_Flash(CalibrationData *pData) {
    CalibrationData *p_read = (CalibrationData *)CALIBRATION_FLASH_ADDR;
    
		pData->magic = p_read->magic;
    
    if (pData->magic == 0x5A5A5A5A) {
			pData->gyro_x_offset=p_read->gyro_x_offset;
			pData->gyro_y_offset=p_read->gyro_y_offset;
			pData->gyro_z_offset=p_read->gyro_z_offset;

		}
		else{;}
		return ;
}
extern PCD_HandleTypeDef hpcd_USB_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;
void Enter_Stop_Mode(void){
	
	HAL_SuspendTick();
	__HAL_RCC_PWR_CLK_ENABLE();
   
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
     GPIO_InitTypeDef g = {0};

         // —— 进 Stop 前：强制断开 ——
         USBD_Stop(&hUsbDeviceFS);
         HAL_PCD_DeInit(&hpcd_USB_FS);               // 关 USB 时钟 → PA12 脱离 USB 外设控制

         //__HAL_RCC_GPIOA_CLK_ENABLE();
         g.Pin  = GPIO_PIN_12;
         g.Mode = GPIO_MODE_OUTPUT_PP;               // PA12 (USB_DP) 配置为推挽输出
         g.Speed = GPIO_SPEED_FREQ_HIGH;
         HAL_GPIO_Init(GPIOA, &g);
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);   // D+ 强制拉低 → 主机看到拔出
         HAL_Delay(10);                              // 断开保持	

    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);


    HAL_ResumeTick();
    
   
	SystemClock_Config();
	MX_USB_DEVICE_Init();			 
	g.Mode = GPIO_MODE_INPUT;                   
         g.Pull = GPIO_NOPULL;
         HAL_GPIO_Init(GPIOA, &g); 
MX_USB_DEVICE_Init();				 
         uint32_t t0 = HAL_GetTick();
        while(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED &&
              (HAL_GetTick() - t0) < 3000){ }
}


#define SATURATE 120
#define KEY_GPIO_PORT    GPIOA
#define KEY0_GPIO_PIN     GPIO_PIN_4
#define KEY1_GPIO_PIN     GPIO_PIN_3
#define KEY_PRESS_LEVEL  1        


extern USBD_HandleTypeDef hUsbDeviceFS;

volatile int16_t g_shared_gyro_x=0;
volatile int16_t g_shared_gyro_y=0;
volatile int16_t g_shared_gyro_z=0;

void task1(void* pvPrama);
#define TASK1_STACK_SIZE 64
#define TASK1_PRIORITY 4
TaskHandle_t task1_tcb;

void task2(void* pvPrama);
#define TASK2_STACK_SIZE 64
#define TASK2_PRIORITY 5
TaskHandle_t task2_tcb;

void task3(void* pvPrama);
#define TASK3_STACK_SIZE 128
#define TASK3_PRIORITY 6
TaskHandle_t task3_tcb;

void task4(void* pvPrama);
#define TASK4_STACK_SIZE 64
#define TASK4_PRIORITY 5
TaskHandle_t task4_tcb;


void task5(void* pvPrama);
#define TASK5_STACK_SIZE 128
#define TASK5_PRIORITY 6
TaskHandle_t task5_tcb;

void vCalibration_task(void* pvPrama);
#define TASK6_STACK_SIZE 64
#define TASK6_PRIORITY 3
TaskHandle_t task6_tcb;


void vAutoCalibrationCheck(void* pvPrama);
#define TASK7_STACK_SIZE 64
#define TASK7_PRIORITY 3
TaskHandle_t task7_tcb;


char mybuff[300]={0};
QueueHandle_t myqueue;
QueueHandle_t mymotion;


QueueHandle_t sembotton;
QueueHandle_t semstop;


typedef struct mpudata{
	int16_t Gyro_x,Gyro_y,Gyro_z;
}mpudata;

typedef struct hiddata{
	int8_t botton,x,y,gl;
}hiddata;


volatile uint8_t g_abort_calibration = pdFALSE;

int8_t g_ButtonState=0,stopcnt=0;
int16_t gyro_dz_dps=0,gyro_dy_dps=0,gyro_dx_dps=0;
int16_t appgz_offest=0,appgy_offest=0,appgx_offest=0;
uint8_t data[10]={'a','m','s','i','l','o','v','e','u','!'};



void freertos_task_start(){

	myqueue = xQueueCreate(4,sizeof(mpudata));
	mymotion = xQueueCreate(4,sizeof(hiddata));

	sembotton=xSemaphoreCreateBinary();
	semstop=xSemaphoreCreateBinary();
	
	xTaskCreate(task1,"task1",TASK1_STACK_SIZE,NULL,TASK1_PRIORITY,&task1_tcb);
	xTaskCreate(task2,"task2",TASK2_STACK_SIZE,NULL,TASK2_PRIORITY,&task2_tcb);
	xTaskCreate(task3,"task3",TASK3_STACK_SIZE,NULL,TASK3_PRIORITY,&task3_tcb);
	//xTaskCreate(task4,"task4",TASK4_STACK_SIZE,NULL,TASK4_PRIORITY,&task4_tcb);
	xTaskCreate(task5,"task5",TASK5_STACK_SIZE,NULL,TASK5_PRIORITY,&task5_tcb);
	xTaskCreate(vCalibration_task,"task6",TASK6_STACK_SIZE,NULL,TASK6_PRIORITY,&task6_tcb);
	xTaskCreate(vAutoCalibrationCheck,"task7",TASK7_STACK_SIZE,NULL,TASK7_PRIORITY,&task7_tcb);

	vTaskStartScheduler();
	return ;
}

void task1(void* pvPrama){
	mpudata data;
	hiddata out={0,0,0,0};

	int16_t gyro_x_dps;
	int16_t gyro_z_dps;
	int16_t gyro_y_dps;
	int8_t tmp0=0;
	int8_t tmp1=0;
	int16_t lastgx=0;
	int16_t lastgz=0;
	int8_t lastb=0;
	while(1){
		
		//vTaskDelay(10);
		if(xQueueReceive(myqueue,&data,10)){
			gyro_x_dps =g_shared_gyro_x= (data.Gyro_x / 64) -appgx_offest;
			gyro_y_dps =g_shared_gyro_y= (data.Gyro_y / 64) -appgy_offest;
			gyro_z_dps =g_shared_gyro_z= (data.Gyro_z / 64) -appgz_offest;			
			
			gyro_x_dps=(15*gyro_x_dps+lastgx)/16;
			gyro_z_dps=(15*gyro_z_dps+lastgz)/16;
			
			if(gyro_z_dps>2||gyro_z_dps<-2){
				tmp0=(gyro_x_dps>0)?gyro_x_dps:-gyro_x_dps;				
			}
			if(gyro_x_dps>2||gyro_x_dps<-2){
				tmp1=(gyro_z_dps>0)?gyro_z_dps:-gyro_z_dps;				
			}
			
			out.x=(gyro_z_dps>0)?-(4*gyro_z_dps+tmp0)/8: (tmp0-4*gyro_z_dps)/8;tmp0=0;		
			//out.y=-gyro_x_dps/4;
			out.y=(gyro_x_dps>0)?-(4*gyro_x_dps+tmp1)/12: (tmp1-4*gyro_x_dps)/12;tmp1=0;		
			if (out.x > SATURATE) out.x = SATURATE;
      if (out.x < -SATURATE) out.x = -SATURATE;
      if (out.y > SATURATE) out.y = SATURATE;
      if (out.y < -SATURATE) out.y = -SATURATE;
			
			out.botton=g_ButtonState;
		
			if(out.botton!=lastb || out.x>2 ||out.y>2||out.x<-2||out.y<-2){
					
				xQueueSend(mymotion,&out,10);
				g_abort_calibration=pdTRUE;
			}
			else{
				//xQueueSend(mygyro,&stopdata,10);
				g_abort_calibration=pdFALSE;
			}
			
			lastb=out.botton;
			lastgx=gyro_x_dps;
			lastgz=gyro_z_dps;
			
			//printf("cccccccccccccccccccccccccccccccccccccccccccbotton=%d, x=%d, y=%d\r\n", out.botton, out.x, out.y);			
		}		
		
	}	

}

void task2(void* pvPrama){
	hiddata output;

	while(1){
		if(xQueueReceive(mymotion,&output,10)){			
				if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
                 hUsbDeviceFS.pClassData != NULL){
								 USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&output,sizeof(hiddata));
									HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
								 
								 }
				
			}
		
	}
		
}


void task3(void* pvPrama){	
	mpudata tmp;

	while(1){
		
		MPU6050_GetData((int16_t*)&tmp.Gyro_x, (int16_t*)&tmp.Gyro_y, (int16_t*)&tmp.Gyro_z);		
		xQueueSend(myqueue,&tmp,10);		
		vTaskDelay(10);
			
	}	
}

void task4(void* pvPrama){
		uint8_t last_stable0_level = 0;     
    uint8_t debounce_counter0 = 0;
		uint8_t last_stable1_level = 0;     
    uint8_t debounce_counter1 = 0;
    uint8_t DEBOUNCE_THRESHOLD = 3; 
    TickType_t SCAN_PERIOD_MS = pdMS_TO_TICKS(10); 
		
    while (1) {
			
					vTaskDelay(SCAN_PERIOD_MS);
					
					uint8_t current_raw0 = HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY0_GPIO_PIN);
					uint8_t current_raw1 = HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY1_GPIO_PIN);
				//printf("%d, %d\r\n", current_raw0, current_raw1);
						if (current_raw0 == last_stable0_level) {
            debounce_counter0++;
            if (debounce_counter0 >= DEBOUNCE_THRESHOLD) {


                if (current_raw0 == KEY_PRESS_LEVEL) {

                    g_ButtonState |= 0x01;
                } else {
                    g_ButtonState &= ~0x01;
                }

                debounce_counter0 = 0; 
            }
        } else {

            debounce_counter0 = 0;
            last_stable0_level = current_raw0;
        }
				
				
				if (current_raw1 == last_stable1_level) {
            debounce_counter1++;
            if (debounce_counter1 >= DEBOUNCE_THRESHOLD) {


                if (current_raw1 == KEY_PRESS_LEVEL) {

                    g_ButtonState |= 0x02;
                } else {
                    g_ButtonState &= ~0x02;
                }

                debounce_counter1 = 0; 
            }
        } else {

            debounce_counter1 = 0;
            last_stable1_level = current_raw1;
        }
				
				


    }
		
}

void task5(void* pvPrama){	
CalibrationData flashdata;
	while(1){
		
        vTaskGetRunTimeStats(mybuff);
        Load_Calibration_From_Flash(&flashdata);
        printf("\n--- Task CPU Usage ---\n%s", mybuff);
		    
		
		printf("magic=%d offx=%d offy=%d offz=%d\n", flashdata.magic,flashdata.gyro_x_offset,flashdata.gyro_y_offset,flashdata.gyro_z_offset);;
   printf("CNTR=%04X DADDR=%02X\r\n", hpcd_USB_FS.Instance->CNTR, hpcd_USB_FS.Instance->DADDR);    
    memset(mybuff,0,sizeof(mybuff));
		    vTaskList(mybuff);
        printf("Task Name\tState\tPriority\tStack\tNum\n");
        printf("%s", mybuff);
				memset(mybuff,0,sizeof(mybuff));
		    if(stopcnt>5){
					          printf("Entering Stop Mode...\r\n");
                    
					vTaskSuspend(task2_tcb);vTaskSuspend(task3_tcb);
										Enter_Stop_Mode();
                    // 唤醒后继续执行
										stopcnt=0;//vTaskDelay(100);
										
					
					vTaskResume(task2_tcb);vTaskResume(task3_tcb);
                    printf("Waked up!\r\n");
				}
        vTaskDelay(1000); // 每秒打印一次
		
    }	
	
}

void vCalibration_task(void* pvPrama){

		
    while(1) {
				xSemaphoreTake(semstop,0);
        int16_t newx=0,newy=0,newz=0;
        for (int i=0; i<200; i++) {
            if (g_abort_calibration) {
                g_abort_calibration = pdFALSE; 
                break;
            }
						newx+=g_shared_gyro_x;
						newy+=g_shared_gyro_y;
						newz+=g_shared_gyro_z;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (!g_abort_calibration) {
            newx/=200;newy/=200;newz/=200;
            if (newx < 15 && newx>-15 && newy < 15 && newy>-15 && newz < 15 && newz>-15) {
								CalibrationData TMP;
								appgx_offest=newx;
								appgy_offest=newy;
								appgz_offest=newz;
								TMP.magic=0x5A5A5A5A;
							TMP.gyro_x_offset=newx;
							TMP.gyro_y_offset=newy;
							TMP.gyro_z_offset=newz;
							Save_Calibration_To_Flash(&TMP);
            }
        }
    }



}

void vAutoCalibrationCheck(void* pvPrama) {
    static int16_t last_x = 0, last_y = 0, last_z = 0;
    static uint32_t stable_tick = 0;
    static uint8_t is_stable = 0;


  while(1){
		int16_t x = g_shared_gyro_x;
    int16_t y = g_shared_gyro_y;
    int16_t z = g_shared_gyro_z;

    int16_t delta = abs(x - last_x) + abs(y - last_y) + abs(z - last_z);
		if (delta < 10) { 
        if (!is_stable) {
            is_stable = 1;
            stable_tick = xTaskGetTickCount();
        } 
				else if ((xTaskGetTickCount() - stable_tick) >= pdMS_TO_TICKS(2000)) {
            xSemaphoreGive(semstop);
            is_stable = 0;
						stopcnt++;
        }
    } 
		else {
        is_stable = 0;
        stable_tick = 0;
    }
    last_x = x; last_y = y; last_z = z;
		vTaskDelay(50);
	}  
	
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
		return ch;

}

extern volatile unsigned long g_ulRunTimeCounter;

unsigned long ulGetRunTimeCounterValue( void )
{
    // 直接返回当前计数器的值
    return g_ulRunTimeCounter; 
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    
    if (GPIO_Pin == GPIO_PIN_3)
    {
				uint8_t state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
        
				if(state)g_ButtonState |= 0x01;
				else g_ButtonState &= ~0x01;
    }
		if (GPIO_Pin == GPIO_PIN_4)
    {
				uint8_t state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
        
				if(state)g_ButtonState |= 0x02;
				else g_ButtonState &= ~0x02;
    }
		
}

