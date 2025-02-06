#include <math.h>
#include "stm32f4xx_hal.h"
#include "motor_dc.h"


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
void control_motor_dc(uint32_t speed_A,uint32_t speed_B){
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PWM[0]);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,PWM[1]);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,PWM[2]);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,PWM[3]);

}

void control_motor_A_forward(uint32_t speed_A){
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PWM[0]);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,PWM[1]);
}

void control_motor_B_forward(uint32_t speed_A){
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,PWM[2]);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,PWM[3]);
}