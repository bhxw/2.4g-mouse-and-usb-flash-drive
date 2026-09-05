#ifndef __MPU6050_H
#define __MPU6050_H

#include "MPU6050_Reg.h"
#include "i2c.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
void MPU6050_ReadReg(uint8_t RegAddress,uint8_t *Data);

void MPU6050_Init(void);
void MPU6050_GetID(uint8_t *Data);
void MPU6050_GetData(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);


#endif
