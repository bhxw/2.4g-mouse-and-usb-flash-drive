#ifndef __FREERTOSTASK_
#define __FREERTOSTASK_
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "mpu6050.h"
#include "usb_device.h"
#include "usbd_hid.h"
//#include "oled.h"


void freertos_task_start(void);


#endif

