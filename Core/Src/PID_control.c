//References:
// https://www.servotecnica.com/en/resources/white-papers-en-mobile/dual-loop-advanced-control-techniques-for-real-world-drivetrains/
// https://controlguru.com/the-cascade-control-architecture/

/**
 * PID control.c
 * @author ChrisP @ M-HIVE

 * This library source code is for cascade double loop pid control for STM32 Drone Development online course.
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2020
 * Rev. 1.0
 *
 * Where to take the online course.
 * https://www.inflearn.com/course/STM32CubelDE-STM32F4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C (Korean language supported only)
 *
 * Where to buy MH-FC V2.2 STM32F4 Drone Flight Controller.
 * https://smartstore.naver.com/mhivestore/products/4961922335
 *
 * https://github.com/ChrisWonyeobPark
 * https://blog.naver.com/lbiith
 * https://cafe.naver.com/mhiveacademy
 * https://www.udemy.com/course/stm32-drone-programming/?referralCode=E24CB7B1CD9993855D45
 * https://www.inflearn.com/course/stm32cubelde-stm32f4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C
*/

#include "PID_control.h"
#include "stm32f4xx_hal.h"
float DT = 0;
#define MAX_PID_SP 400
#define MAX_PID_THROW_SP 100

void PID_Calculation(PIDSingle* axis, float set_point_agle, float input_agle/*ICM-20602 Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Rate Control) *************/
	axis->reference = set_point_agle;	//Set point of yaw heading @ yaw stick is not center.
	axis->meas_value = input_agle;			//Current ICM20602.gyro_z @ yaw stick is not center.
	axis->error_x[0] = input_agle - set_point_agle;
	axis->p_result = (float)((axis->error_x[0] * axis->kp));	 			//Calculate P result of yaw rate control
	axis->i_result += (float)((axis->error_x[0]* axis->ki));					//Calculate I result of yaw rate control
	axis->d_result = (float)(((axis->error_x[0] -axis->error_x[1])* axis->kd));				//Calculate D result of yaw rate control
	if(axis->i_result > MAX_PID_SP){axis->i_result  = MAX_PID_SP;}
	if(axis->i_result < -MAX_PID_SP){axis->i_result  = -MAX_PID_SP;}
	axis->pid_result = (axis->p_result + axis->i_result + axis->d_result); 	//Calculate PID result of yaw control
	/*******************************************************************/
	if (axis->pid_result > MAX_PID_SP) {axis->pid_result = MAX_PID_SP;}
	if (axis->pid_result < -MAX_PID_SP){axis->pid_result = -MAX_PID_SP;}
	axis->error_x[1] = axis->error_x[0];
}
void PID_Calculation_thr(PIDSingle* axis, float set_point_agle, float input_agle/*ICM-20602 Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Rate Control) *************/
	axis->reference = set_point_agle;	//Set point of yaw heading @ yaw stick is not center.
	axis->meas_value = input_agle;			//Current ICM20602.gyro_z @ yaw stick is not center.
	axis->error_x[0] = input_agle - set_point_agle;
	axis->p_result = (float)((axis->error_x[0] * axis->kp));	 			//Calculate P result of yaw rate control
	axis->i_result += (float)((axis->error_x[0]* axis->ki));					//Calculate I result of yaw rate control
	axis->d_result = (float)(((axis->error_x[0] -axis->error_x[1])* axis->kd));				//Calculate D result of yaw rate control
	if(axis->i_result > MAX_PID_THROW_SP){axis->i_result  = MAX_PID_THROW_SP;}
	if(axis->i_result < -MAX_PID_THROW_SP){axis->i_result  = -MAX_PID_THROW_SP;}
	axis->pid_result = (axis->p_result + axis->i_result + axis->d_result); 	//Calculate PID result of yaw control
	/*******************************************************************/
	if (axis->pid_result > MAX_PID_THROW_SP) {axis->pid_result = MAX_PID_THROW_SP;}
	if (axis->pid_result < -MAX_PID_THROW_SP){axis->pid_result = -MAX_PID_THROW_SP;}
	axis->error_x[1] = axis->error_x[0];
}
void Int_PID_Integrator(PIDSingle* axis,float kp_x,float ki_x,float kd_x)
{
	axis->kp = kp_x;
	axis->ki = ki_x;
	axis->kd = kd_x;
}

void Reset_PID_Integrator(PIDSingle* axis)
{
	axis->i_result = 0;
//	axis->error_x[0] = 0;
	axis->error_x[1] = 0;
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&roll.in);
	Reset_PID_Integrator(&roll.out);
	Reset_PID_Integrator(&pitch.in);
	Reset_PID_Integrator(&pitch.out);
	Reset_PID_Integrator(&yaw_heading);
	Reset_PID_Integrator(&yaw_rate);
}
