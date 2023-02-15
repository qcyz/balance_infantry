#ifndef ___CHASSIS_TASK_H
#define ___CHASSIS_TASK_H

#include "chassis_struct_variables.h"

/* ************************freertos******************** */
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

/*OS控制任务周期以及启动时间*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME 2

/************底盘pid************/
#define CHASSIS_SPEED_KP 40.0f
#define CHASSIS_SPEED_KI 0.0f
#define CHASSIS_SPEED_KD 0.0f

#define CHASSIS_POSITION_KP 9.0f
#define CHASSIS_POSITION_KI 0.0f
#define CHASSIS_POSITION_KD 0.5f

#define CHASSIS_SPEED_LIMIT  3.0f
#define CHASSIS_PITCH_LIMIT 	12.0f

/**********************低通滤波比例**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K 0.0410f // 0.0110f 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高  |  0.26f  |  0.0097f  |  0.0510f
#define CHASSIS_MK_SECOND_FILTERING (0.8f)

/**********************运动加速度限制**********************/

#define STRAIGHT_ACCELERAD 3.5f    //直行底盘加速度限制
#define TRANSLATION_ACCELERAD 5.5f //平移底盘加速度限制
#define ROTATING_ACCELERAD 19.0f   //旋转底盘加速度限制

#define CHASSIS_ROTATION_SPEED 30               //小陀螺的旋转速度  2000

#define MOTOR_3508_CURRENT_LIMIT 15000

#define MOTOR_RADIUS 0.1f

#define BARYCENTER_ZERO_OFFSET	4.0f //重心偏移值

#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif


/**********************力矩转输出**********************/
#define TORQUE_CONSTANT 	0.31				//N.M/A
#define CURRENT_CONSTANT 	2048.0f/33.0f 		//LSB/A
#define CURRENT_FOLLOW		1.29399585			//kp=250

#define TORQUE_TO_OUTPUT	1/TORQUE_CONSTANT*CURRENT_CONSTANT*CURRENT_FOLLOW

void Task_Chassis(void const *argument);

#endif
