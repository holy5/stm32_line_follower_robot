/*
 * motor.h
 *
 *  Created on: Oct 8, 2023
 *      Author: lvquang
 */
#include "main.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define STBY_PIN GPIO_PIN_14
#define STBY_PORT GPIOB

typedef struct motor_params{
	uint16_t pulse_per_rev;
	uint32_t ucount;
	int16_t scount;
	int16_t position;
	int32_t prev_time;
	uint16_t rpm;
	int8_t direction;
	uint16_t pwm_channel;
	uint16_t in1_pin;
	uint16_t in2_pin;
	GPIO_TypeDef* in_port;
}motor_params;

typedef struct PID_Param {
	float Kp;
	float Ki;
	float Kd;
	int enable_anti_windup;
	float Anti_windup_error;
	float Outmin;
	float Outmax;
	int Anti_windup;
	int is_sat;
	int is_same_sign;
}PID_Param;

void PID_Init(PID_Param *p);

float PID_Calculation(float set_point, float current);

void initMotor(motor_params* motor_params,TIM_HandleTypeDef* htim);

void stopMotor(motor_params* motor_params);

void standByMode();

void setDutyCycle(motor_params* motor_params,TIM_HandleTypeDef* htim, uint32_t duty_cycle);

void setMotorPWM(motor_params* motor_params,TIM_HandleTypeDef* htim, float duty_cycle_percentage);

void readEncoder(motor_params* motor_params,TIM_HandleTypeDef *htim);

void calculateRPM(motor_params* motor_params, int32_t current_time);

#endif /* INC_MOTOR_H_ */
