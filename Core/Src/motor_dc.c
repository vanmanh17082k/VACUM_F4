#include <math.h>
#include "stm32f4xx_hal.h"
#include "motor_dc.h"

extern I2C_HandleTypeDef hi2c1;
#define PWM_RESOLUTION  0xFFFF
void setup_motor_dc(){
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,0);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,0);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); 
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

}
void Move_Forward(int16_t speed) {
    Motor_Left_SetSpeed(speed);
    Motor_Right_SetSpeed(speed);
}

void Motor_Left_SetSpeed(int32_t speed_A){
    uint16_t speed = constrain(speed_A, -100, 100); // Giới hạn tốc độ
    
    if(speed > 0) { // Tiến
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,(speed * PWM_RESOLUTION) / 100);
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,0);
    }
    else if(speed < 0) { // Lùi
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,(-speed * PWM_RESOLUTION) / 100);
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,0);
    }
    else { // Dừng
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,0);
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,0);
    }
    PWM[0]= TIM3->CCR1;
    PWM[1]= TIM3->CCR2;
}

void Motor_Right_SetSpeed(int32_t speed_B){
    uint16_t speed = constrain(speed_B, -100, 100); // Giới hạn tốc độ
    
    if(speed > 0) { // Tiến
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,(speed * PWM_RESOLUTION) / 100);
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,0);
    }
    else if(speed < 0) { // Lùi
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,(-speed * PWM_RESOLUTION) / 100);
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,0);
    }
    else { // Dừng
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,0);
        __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,0);
    }
    PWM[2]= TIM3->CCR3;
    PWM[3]= TIM3->CCR4;
}

void Turn(int16_t speed, int16_t target_angle) {
    float current_angle = 0;
    float previous_angle = 0;
    uint32_t previous_time = HAL_GetTick();

    while (fabsf(current_angle) < target_angle) { //fabsf for float absolute value
        MPU6050_t mpu_data;
        MPU6050_Read_Gyro(&hi2c1, &mpu_data);  // Read gyro data
        uint32_t current_time = HAL_GetTick();
        float dt = (float)(current_time - previous_time) / 1000.0f; // Time in seconds

        if (dt > 0.01) { // Avoid division by zero and very small dt
          mpu_data.Gz_yaw = mpu_data.Gyro_Z_RAW / 65.5; // Scale factor for 250dps
          current_angle += mpu_data.Gz_yaw * dt;
          previous_time = current_time;
        }


        // Control the motors based on the rotation direction
        if (current_angle < 0) { // Rotate right. Adjust motor speeds as needed
            Motor_Left_SetSpeed(-speed);    
            Motor_Right_SetSpeed(speed);  
        } else { // Rotate left (if needed)
           Motor_Left_SetSpeed(speed);  
           Motor_Right_SetSpeed(-speed);
        }
        HAL_Delay(10); //Small delay to avoid high speed loop
    }

    // Stop the motors after rotation
    Motor_Left_SetSpeed(0);    
    Motor_Right_SetSpeed(0);  
}