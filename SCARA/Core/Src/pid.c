/*
 * pid.c
 *
 *  Created on: Oct 18, 2021
 *      Author: john
 */

#include "pid.h"

#define ERR_SUM_MAX	1500

struct pid_params
{
	float proportional;
	float integral;
	float derivative; //k_d = 1
	float controlSignal; //u - Also called as process variable (PV)
	float previousTime; //for calculating delta t
	float previousError; //for calculating the derivative (edot)
	float errorIntegral; //integral error
	float currentTime; //time in the moment of calculation
	float deltaTime; //time difference
	float errorValue; //error
	float edot; //derivative (de/dt)
};

static struct pid_params pid1;
static struct pid_params pid2;

void pid_init1(float kp, float ki, float kd)
{
	pid1.proportional = kp;
	pid1.integral = ki;
	pid1.derivative = kd; //k_d = 1
	pid1.controlSignal = 0; //u - Also called as process variable (PV)
	pid1.previousTime = 0; //for calculating delta t
	pid1.previousError=0; //for calculating the derivative (edot)
	pid1.errorIntegral=0; //integral error
	pid1.currentTime=0; //time in the moment of calculation
	pid1.deltaTime=0; //time difference
	pid1.errorValue=0; //error
	pid1.edot=0; //derivative (de/dt)
}

void pid_init2(float kp, float ki, float kd)
{
	pid2.proportional = kp;
	pid2.integral = ki;
	pid2.derivative = kd; //k_d = 1
	pid2.controlSignal = 0; //u - Also called as process variable (PV)
	pid2.previousTime = 0; //for calculating delta t
	pid2.previousError=0; //for calculating the derivative (edot)
	pid2.errorIntegral=0; //integral error
	pid2.currentTime=0; //time in the moment of calculation
	pid2.deltaTime=0; //time difference
	pid2.errorValue=0; //error
	pid2.edot=0; //derivative (de/dt)
}


float pid_calculate1(float set_val, float read_val,float currentTime)
{
	pid1.proportional = 50.0;
	pid1.integral =  0.05;
	pid1.derivative = 10.0;
	pid1.deltaTime = (currentTime - pid1.previousTime) / 1000000.0; //time difference in seconds
	pid1.previousTime = currentTime;//save the current time for the next iteration to get the time difference

	pid1.errorValue = set_val - read_val;

	pid1.edot = (pid1.errorValue - pid1.previousError) / pid1.deltaTime;//edot = de/dt - derivative term

	pid1.errorIntegral = pid1.errorIntegral + (pid1.errorValue * pid1.deltaTime);

	if(pid1.errorIntegral > ERR_SUM_MAX){
		pid1.errorIntegral = ERR_SUM_MAX;
	} else if (pid1.errorIntegral < -ERR_SUM_MAX){
		pid1.errorIntegral = -ERR_SUM_MAX;
	}

	pid1.controlSignal = (pid1.proportional * pid1.errorValue) + (pid1.derivative * pid1.edot) + (pid1.integral * pid1.errorIntegral);

	pid1.previousError = pid1.errorValue;

	return pid1.controlSignal;
}

float pid_calculate2(float set_val, float read_val,float currentTime)
{
	pid2.proportional = 40.0;
	pid2.integral =  0.05;
	pid2.derivative =5.0;
	pid2.deltaTime = (currentTime - pid2.previousTime) / 1000000.0; //time difference in seconds
	pid2.previousTime = currentTime;//save the current time for the next iteration to get the time difference

	pid2.errorValue = set_val - read_val;

	pid2.edot = (pid2.errorValue - pid2.previousError) / pid2.deltaTime;//edot = de/dt - derivative term

	pid2.errorIntegral = pid2.errorIntegral + (pid2.errorValue * pid2.deltaTime);

	if(pid2.errorIntegral > ERR_SUM_MAX){
		pid2.errorIntegral = ERR_SUM_MAX;
	} else if (pid2.errorIntegral < -ERR_SUM_MAX){
		pid2.errorIntegral = -ERR_SUM_MAX;
	}

	pid2.controlSignal = (pid2.proportional * pid2.errorValue) + (pid2.derivative * pid2.edot) + (pid2.integral * pid2.errorIntegral);

	pid2.previousError = pid2.errorValue;

	return pid2.controlSignal;
}

/*#include "pid.h"

#define ERR_SUM_MAX		255

struct pid_params
{
	float kp;
	float ki;
	float kd;
	float err;
	float err_sum;
	float err_last;
	int sampleTime;
};

static struct pid_params pid_params;
static struct pid_params pid_params1;

void pid_init(float kp, float ki, float kd)
{
	pid_params.kp = kp;
	pid_params.ki = ki;
	pid_params.kd = kd;
	pid_params.err = 0;
	pid_params.err_sum = 0;
	pid_params.err_last = 0;
	pid_params.sampleTime = 1;
}

float pid_calculate(float set_val, float read_val)
{
	float err_d, u;

	pid_params.err = set_val - read_val;
	pid_params.err_sum += pid_params1.err;

	if (pid_params.err_sum > ERR_SUM_MAX) {
		pid_params.err_sum = ERR_SUM_MAX;
	} else if (pid_params.err_sum < -ERR_SUM_MAX) {
		pid_params.err_sum = -ERR_SUM_MAX;
	}

	err_d = pid_params.err_last - pid_params.err;
	u = pid_params.kp * pid_params.err + pid_params.ki * pid_params.err_sum*pid_params.sampleTime
			+ pid_params.kd * err_d;

	//if(u<60 && u>-60) u = 0;

	return u;
}

void pid_init1(float kp, float ki, float kd)
{
	pid_params1.kp = kp;
	pid_params1.ki = ki;
	pid_params1.kd = kd;
	pid_params1.err = 0;
	pid_params1.err_sum = 0;
	pid_params1.err_last = 0;
	pid_params1.sampleTime = 1;
}

float pid_calculate1(float set_val, float read_val)
{
	float err_d, u;

	pid_params1.err = set_val - read_val;
	pid_params1.err_sum += pid_params1.err;

	if (pid_params1.err_sum > ERR_SUM_MAX) {
		pid_params1.err_sum = ERR_SUM_MAX;
	} else if (pid_params1.err_sum < -ERR_SUM_MAX) {
		pid_params1.err_sum = -ERR_SUM_MAX;
	}

	err_d = pid_params1.err_last - pid_params1.err;
	u = pid_params1.kp * pid_params1.err + pid_params1.ki * pid_params1.err_sum*pid_params1.sampleTime
			+ pid_params1.kd * err_d;

	//if(u<60 && u>-60) u = 0;

	return u;
}*/

