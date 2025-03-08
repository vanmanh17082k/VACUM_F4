/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define CONFIG_REG 0x1A
// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 10000;
const double Accel_Z_corrector = 14418.0;
uint32_t timer_mpu;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;
    uint8_t Data = 0;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 0x68) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        //Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // 1B ->8
        Data = 0x8;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // 1C ->10
        Data = 0x10;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // 1Khz
        Data = 0x03;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, i2c_timeout);

        return check;
    }
    return 0;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

//    DataStruct->Ax = DataStruct->Accel_X_RAW / 16710.0;
//    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16710.0;
//    DataStruct->Az = DataStruct->Accel_Z_RAW / 16710.0;

//    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, DataStruct->Ax ,DataStruct->Ax/0.92, DataStruct->time_run);
//    DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, DataStruct->Ay ,DataStruct->Ay/0.92, DataStruct->time_run);
//    DataStruct->KalmanAngleZ = Kalman_getAngle(&KalmanZ, DataStruct->Az ,DataStruct->Az/0.92, DataStruct->time_run);

}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[14];
//	uint8_t set_gyro_angles = 0;
	// Read 14 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

	DataStruct->Accel_X_RAW= (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Accel_Y_RAW= (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Accel_Z_RAW= (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10]<< 8 | Rec_Data[11]);
	DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12]<< 8 | Rec_Data[13]);

//	DataStruct->Ax = (float)DataStruct->Accel_X_RAW / 1.0;
//	DataStruct->Ay = (float)DataStruct->Accel_Y_RAW / 1.0;
//	DataStruct->Az = (float)DataStruct->Accel_Z_RAW / 1.0;
//	DataStruct->Gx = (float)DataStruct->Gyro_X_RAW / 1.0;
//	DataStruct->Gy = (float)DataStruct->Gyro_Y_RAW / 1.0;
//	DataStruct->Gz = (float)DataStruct->Gyro_Z_RAW / 1.0;
	DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, (float)DataStruct->Accel_X_RAW ,(float)DataStruct->Accel_X_RAW/4200, 0.01);
	DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, (float)DataStruct->Accel_Y_RAW ,(float)DataStruct->Accel_Y_RAW/4200, 0.01);
	DataStruct->KalmanAngleZ = Kalman_getAngle(&KalmanZ, (float)DataStruct->Accel_Z_RAW ,(float)DataStruct->Accel_Z_RAW/4200, 0.01);
}

void Get_Accel_Angles(I2C_HandleTypeDef *I2Cx,MPU6050_Data_t *data, float *roll, float *pitch) {
    MPU6050_Read_Accel(I2Cx, data);
    // Scale factor (ví dụ: ±2g = 16384 LSB/g)
    float accel_X = data->Accel_X_RAW / 16384.0f;
    float accel_Y = data->Accel_Y_RAW / 16384.0f;
    float accel_Z = data->Accel_Z_RAW / 16384.0f;
    
    *roll = atan2(accel_Y, accel_Z) * RAD_TO_DEG;
    *pitch = atan2(-accel_X, sqrt(accel_Y*accel_Y + accel_Z*accel_Z)) * RAD_TO_DEG;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;//B1:Tìm góc ước tính
    Kalman->angle += dt * rate;//Tìm tỷ lệ góc lệch với thời gian
    //B2 :Tìm lỗi dự đoán P bằng matrận 2x2:
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle); //
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;
    //B3:Tìm hệ số lỗi
    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;
    //B4:Tìm sự biến đổi giá trị cũ và mới
    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    //B5: Cập nhật lỗi cho lần sau
    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;
    //B6: Xuất output
    return Kalman->angle;
}

void MPU6050_ResetProcedure(I2C_HandleTypeDef *I2Cx) {
    // 1. Software reset
    uint8_t reset_cmd[2] = {0x6B, 0x80};
    HAL_I2C_Master_Transmit(I2Cx, MPU6050_ADDR, reset_cmd, 2, 100);
    HAL_Delay(100);
}
//https://github.com/TKJElectronics/KalmanFilter
