
#include "mpu6050.h"


#define MPU6050_ADDRESS		0xD0

#define MPU6050_MAX_DELAY 100



void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, MPU6050_MAX_DELAY);
	
}

void MPU6050_ReadReg(uint8_t RegAddress,uint8_t *Data)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, Data, 1, MPU6050_MAX_DELAY);

}

void MPU6050_Init(void)
{
	
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x08);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
}

void MPU6050_GetID(uint8_t *Data)
{
	MPU6050_ReadReg(MPU6050_WHO_AM_I,Data);
}

void MPU6050_GetData(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{

    uint8_t raw[14];  

 
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, raw, 14, 100) != HAL_OK) {

        return;
    }

    *GyroX = (short int)((raw[8] << 8) | raw[9]);
    *GyroY = (short int)((raw[10] << 8) | raw[11]);
    *GyroZ = (short int)((raw[12] << 8) | raw[13]);
}
