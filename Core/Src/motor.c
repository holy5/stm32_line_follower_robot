
#include "motor.h"
#include <stdlib.h>

//Motor

void initMotor(motor_params* motor_params,TIM_HandleTypeDef* htim){
	HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET); //Turn on driver
	HAL_TIM_PWM_Start(htim, motor_params->pwm_channel);
	setDutyCycle(motor_params,htim,0);
}

void stopMotor(motor_params* motor_params){
	HAL_GPIO_WritePin(motor_params->in_port, motor_params->in1_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor_params->in_port, motor_params->in2_pin, GPIO_PIN_RESET);
}

void standByMode(){
	HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
}

void setDutyCycle(motor_params* motor_params,TIM_HandleTypeDef* htim, uint32_t duty_cycle){
//	if(duty_cycle>100){
//		duty_cycle=100;
//	} else if (duty_cycle<15){
//		duty_cycle=0;
//	}
	float pw_resolution = (((float)(*htim).Init.Period + 1.0f) / 100.0f);
	uint16_t pw_desired = pw_resolution * duty_cycle;
	__HAL_TIM_SET_COMPARE(htim, motor_params->pwm_channel, pw_desired);
}
void setMotorPWM(motor_params* motor_params,TIM_HandleTypeDef*  htim, float duty_cycle_percentage){
// 100 means full speed forward; -100 means full speed backward
	HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
	if(duty_cycle_percentage>0){
		HAL_GPIO_WritePin(motor_params->in_port, motor_params->in1_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor_params->in_port, motor_params->in2_pin, GPIO_PIN_RESET);
		setDutyCycle(motor_params,htim, duty_cycle_percentage);
	}else if (duty_cycle_percentage<0){
		HAL_GPIO_WritePin(motor_params->in_port, motor_params->in1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor_params->in_port, motor_params->in2_pin, GPIO_PIN_SET);
		setDutyCycle(motor_params,htim, -duty_cycle_percentage);
	}
}
void readEncoder(motor_params* motor_params, TIM_HandleTypeDef *htim){
	motor_params->ucount = __HAL_TIM_GET_COUNTER(htim);
	motor_params->scount = (int16_t)motor_params->ucount;
	motor_params->position = (int16_t)(motor_params->scount/4);
}

void calculateRPM(motor_params* motor_params, int32_t current_time){


	motor_params->rpm = 60/(motor_params->pulse_per_rev*1e-5*abs(current_time - motor_params->prev_time));
	motor_params->prev_time = current_time;
}

//PID

float Kp, Ki, Kd, Ts, Outmin, Outmax, anti_windup_error;
int enable_anti_windup;
uint16_t time,last_time=0;
float error, prev_error=0, error_sum=0;
int8_t is_sat, is_same_sign;

void PID_Init(PID_Param *p)
{
	Kp=p->Kp;
	Ki=p->Ki;
	Kd=p->Kd;
	anti_windup_error=p->Anti_windup_error;
	Outmin=p->Outmin;
	Outmax=p->Outmax;
	enable_anti_windup=p->enable_anti_windup;
	is_sat = p->is_sat;
	is_same_sign = p->is_same_sign;

	if(p->Anti_windup_error==0){anti_windup_error=10;}
}

float PID_Calculation(float set_point,float current)
{
	error=(set_point - current);
	time = HAL_GetTick();
	Ts = time - last_time;
	error_sum+=error*Ts;

	float out;
	//	Anti windup v2
//	if(is_sat == 1 && is_same_sign ==1 ){
//		error_sum = 0;
//	}

	if(enable_anti_windup==1){
		if(anti_windup_error < abs(error)){
			out=Kp*(error)+Kd*(error-prev_error)/Ts;
		}
		else{
			out=(Kp*(error)) +( Ki*(error_sum)) + (Kd*(error-prev_error)/Ts);
		}
	}else{
		out=Kp*(error) + Ki*(error_sum) + Kd*(error-prev_error)/Ts;
	}

//	if((error>=0)^(out<0)){
//		is_same_sign =1;
//	}else{
//		is_same_sign=0;
//	}

	if (out > Outmax){
		is_sat = 1;
		out = Outmax;
	} else if(out < Outmin){
		is_sat = 1;
		out = Outmin;
	}else{
		is_sat = 0;
	}

	prev_error=error;
	last_time = time;
	return out;
}
