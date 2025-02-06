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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H
#ifdef __cplusplus
 extern "C" {
#endif


typedef struct _PIDSingle
{
	float kp;
	float ki;
	float kd;
	
	float reference;
	float meas_value;
	float meas_value_prev;
	float error_x[2];
	
	float p_result;
	float i_result;
	float d_result;
	
	float pid_result;
}PIDSingle;

typedef struct _PIDDouble
{
	PIDSingle in;
	PIDSingle out;
}PIDDouble;


extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;

void Reset_PID_Integrator(PIDSingle* axis);
void Reset_All_PID_Integrator(void);
void PID_Calculation(PIDSingle* axis, float set_point_agle, float agle/*ICM-20602 Angular Rate*/);
void PID_Calculation_thr(PIDSingle* axis, float set_point_agle, float input_agle/*ICM-20602 Angular Rate*/);
void Int_PID_Integrator(PIDSingle* axis,float kp_x,float ki_x,float kd_x);

#ifdef __cplusplus
}
#endif
#endif /*__PID_CONTROL_H */
