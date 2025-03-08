#ifndef MOTOR_DC_H_
#define MOTOR_DC_H_

#include "stm32f4xx_hal.h"
#include "mpu6050.h"
// Biến toàn cục (chỉ khai báo)
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define PWM_PIN_NUMBER 4
extern int32_t PWM[PWM_PIN_NUMBER];

// Khai báo các hàm
void setup_motor_dc();
void control_motor_dc(uint32_t speed_A, uint32_t speed_B);
void control_motor_A_forward(uint32_t speed_A);
void control_motor_B_forward(uint32_t speed_B);
void Turn(int16_t speed, int16_t target_angle);
#endif
