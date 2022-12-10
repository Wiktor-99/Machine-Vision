/*
 * pid.h
 *
 *  Created on: Oct 18, 2021
 *      Author: john
 */

#ifndef INC_PID_H_
#define INC_PID_H_


void pid_init1(float kp, float ki, float kd);
void pid_init2(float kp, float ki, float kd);
float pid_calculate1(float set_val, float read_val, float currentTime);
float pid_calculate2(float set_val, float read_val,float currentTime);

#endif

/*#ifndef INC_PID_H_
#define INC_PID_H_


void pid_init(float kp, float ki, float kd);

float pid_calculate(float set_val, float read_val);

void pid_init1(float kp, float ki, float kd);

float pid_calculate1(float set_val, float read_val);


#endif */
