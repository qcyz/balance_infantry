/*
_ooOoo_
o8888888o
88" . "88
(| -_- |)
 O\ = /O
___/`---'\____
.   ' \\| |// `.
/ \\||| : |||// \
/ _||||| -:- |||||- \
| | \\\ - /// | |
| \_| ''\---/'' | |
\ .-\__ `-` ___/-. /
___`. .' /--.--\ `. . __
."" '< `.___\_<|>_/___.' >'"".
| | : `- \`.;`\ _ /`;.`/ - ` : | |
\ \ `-. \_ __\ /__ _/ .-` / /
======`-.____`-.___\_____/___.-`____.-'======
`=---='

		 .............................................
		  佛曰：bug泛滥，我已瘫痪！
*/

//  #include "gimbal_struct_variables.h"
#ifndef __GIMBAL_STRUCT_VARIABLES_H
#define __GIMBAL_STRUCT_VARIABLES_H

#include "main.h"
#include "struct_typedef.h"

/*  系统头文件 */
// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
#include <stdbool.h>
// #include "stdint.h"

///* ************************FreeRTOS******************** */
// #include "freertos.h"
// #include "task.h"
// #include "queue.h"
// #include "semphr.h"
// #include "cmsis_os.h"

/******************** BSP ********************/
#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"

/********************ALGORITHM********************/
#include "pid.h"
#include "fifo.h"
#include "filter.h"
#include "lqr.h"
/********************REFEREE********************/
// #include "crc.h"
// #include "referee_deal.h"

#include "ins_task.h"

typedef enum
{
	GIMBAL_MANUAL,		 // 手动状态
	GIMBAL_AUTOATTACK,	 // 自瞄状态
	GIMBAL_AUTOBUFF,	 // 打符状态
	GIMBAL_REPLENISHMEN, // 补给状态
} gimbal_behaviour_e;

typedef enum
{
	GIMBAL_ZERO_FORCE, // 云台无力
	GIMBAL_NORMAL,	   //
} gimbal_state_e;

// rm电机
typedef struct
{
	uint16_t position;
	int16_t speed;
	int16_t current;
	uint8_t temperature;
} motor_measure_t;


// p轴和y轴
typedef struct
{
	motor_measure_t *motor_measure;
	Encoder_t *motor_encoder;
	pid_parameter_t motor_pid;
	LQR_t motor_lqr;
	SlidAveFilterObj motor_filter;
	
	float motor_target;
	float motor_output;
	
} gimbal_motor_control_t;



// 火控
typedef struct
{
	motor_measure_t *right_motor;
	motor_measure_t *left_motor;
	motor_measure_t *fire_motor;
	
	Encoder_t *fire_motor_encoder;
	
	pid_parameter_t right_motor_speed_pid;
	pid_parameter_t left_motor_speed_pid;
	pid_parameter_t fire_motor_speed_pid;
	pid_parameter_t fire_motor_position_pid;
	
	bool full_automatic;
	bool feed_buttle;
	
	const RC_ctrl_t *fire_rc;
} gimbal_fire_control_t;


typedef struct
{
	uint8_t visual_buff_send[29];
	fifo_s_t *usb_fifo;
	float auto_yaw;
	float auto_pitch;
	float auto_pitch_speed;
	
	const gimbal_behaviour_e *gimbal_behaviour;
	const float *gimbal_yaw;
	const float *gimbal_pitch;
} gimbal_auto_control_t;

typedef struct
{
	const RC_ctrl_t *Gimbal_RC;
	const INS_t *Gimbal_INS;
	
	gimbal_behaviour_e gimbal_behaviour;
	gimbal_state_e gimbal_state;
	
	gimbal_motor_control_t Pitch_c;
	gimbal_motor_control_t Yaw_c;
	float chassis_gimbal_angel;
	
	const gimbal_fire_control_t *fire_c;
	const gimbal_auto_control_t **auto_c;
} gimbal_control_t;

#endif
