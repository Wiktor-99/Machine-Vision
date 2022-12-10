/*
 * dc_driver.h
 *
 *  Created on: Oct 18, 2021
 *      Author: john
 */

#ifndef INC_DC_DRIVER_H_
#define INC_DC_DRIVER_H_

void moveStopJoint1();
void moveStopJoint2();
void moveLeftJoint1();
void moveRightJoint1();
void moveLeftJoint2();
void moveRightJoint2();
void setMotorSpeed1(TIM_HandleTypeDef *htim,long int valuePWM);
void setMotorSpeed2(TIM_HandleTypeDef *htim,long int valuePWM);


#endif /* INC_DC_DRIVER_H_ */
