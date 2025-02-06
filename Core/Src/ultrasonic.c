/*****************************************************************************************************************************
**********************************    Author  : Ehab Magdy Abdullah                      *************************************
**********************************    Linkedin: https://www.linkedin.com/in/ehabmagdyy/  *************************************
**********************************    Youtube : https://www.youtube.com/@EhabMagdyy      *************************************
******************************************************************************************************************************/

#include "ultrasonic.h"

extern TIM_HandleTypeDef htim2;
#define HC_SR04_number 4
uint32_t start_time[HC_SR04_number] = {0,0,0,0};
float distance_cm[HC_SR04_number]	= {0.0,0.0,0.0,0.0};
uint8_t edge_state[HC_SR04_number]  = {0,0,0,0};
uint32_t end_time=0;
void delay_us(uint16_t us) 
{
	edge_state[0] = 0;
	edge_state[1] = 0;
	edge_state[2] = 0;
	edge_state[3] = 0;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER (&htim2) < us);
}

void Ultrasonic_Get_Distance(void)
{
	/* Send Trigger Signal to the Ultrasonic Trigger Pin */
	HAL_GPIO_WritePin(ULTRASONUC_TRIGGER_PORT, ULTRASONIC_TRIGGER_PIN, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(ULTRASONUC_TRIGGER_PORT, ULTRASONIC_TRIGGER_PIN, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC3);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC4);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{	
	uint8_t channel;
	uint32_t timer_channel;
	uint32_t IRQ_port;
  	if (htim->Instance != TIM2) return;
	
  	// Xác định kênh
  	switch (htim->Channel) {
  	  case HAL_TIM_ACTIVE_CHANNEL_1: {channel = 0; timer_channel = TIM_CHANNEL_1; 	IRQ_port = TIM_IT_CC1;	break;}
  	  case HAL_TIM_ACTIVE_CHANNEL_2: {channel = 1; timer_channel = TIM_CHANNEL_2; 	IRQ_port = TIM_IT_CC2;	break;}
  	  case HAL_TIM_ACTIVE_CHANNEL_3: {channel = 2; timer_channel = TIM_CHANNEL_3; 	IRQ_port = TIM_IT_CC3;	break;}
	  case HAL_TIM_ACTIVE_CHANNEL_4: {channel = 3; timer_channel = TIM_CHANNEL_4; 	IRQ_port = TIM_IT_CC4;	break;}
  	  default: return;
  	}

  	// Kiểm tra cạnh Rising/Falling
  	if (edge_state[channel] == 0) {
  		// Rising Edge: Lưu thời điểm bắt đầu
  		start_time[channel] = HAL_TIM_ReadCapturedValue(htim, timer_channel);
  		// Chuyển sang bắt Falling Edge
		edge_state[channel] = 1;
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, timer_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
  	} else {
  	  	// Falling Edge: Tính khoảng cách
  	  	end_time = HAL_TIM_ReadCapturedValue(htim, timer_channel);
  	  	uint32_t pulse_width = end_time - start_time[channel];
  	  	distance_cm[channel] =(float)pulse_width * 0.0343 / 2;
  	  	// Chuyển lại về Rising Edge
		edge_state[channel] = 0;
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, timer_channel, TIM_INPUTCHANNELPOLARITY_RISING);
		__HAL_TIM_DISABLE_IT(htim, IRQ_port);
	}
}
