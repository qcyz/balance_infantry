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
#ifndef __CHASSIS_STRUCT_VARIABLES_H
#define __CHASSIS_STRUCT_VARIABLES_H

#include "main.h"
#include "struct_typedef.h"

/*  系统头文件 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "stdint.h"

/* ************************FreeRTOS******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"

/********************CONTROL********************/
#include "bsp_dr16.h"
#include "ins_task.h"
//#include "can2_receive.h"
//#include "capacitor_control.h"
//#include "upper_machine.h"
//#include "visual.h"

/********************ALGORITHM********************/
//#include "fifo_buff.h"
#include "pid.h"
#include "lqr.h"
#include "filter.h"
//#include "maths.h"
//#include "rm_motor.h"


/********************REFEREE********************/
//#include "crc.h"
//#include "referee_deal.h"

#include "bsp_Motor_Encoder.h"

/*模块工作属性*/
//#define WATCH_DOG                //启动看门狗
//#define FIRE_WORK                //射弹模式开启 (开拨弹轮)
#define POWER_LIMIT              //启动功率限制   

typedef enum
{
	CHASSIS_FOLLOW,	   //跟随
	CHASSIS_NO_FOLLOW, //不跟随
	CHASSIS_ROTATION,  //小陀螺
	CHASSIS_BATTERY,   //炮台模式
} chassis_behaviour_e;

typedef enum
{
	CHASSIS_ZERO_FORCE,	   //底盘无力
	CHASSIS_LOCK_POSITION, //底盘位置锁死
	CHASSIS_SPEED,		   //速度
} chassis_state_e;

typedef enum
{
    CHASSIS_MOTOR_L = 0x141,
    CHASSIS_MOTOR_R = 0x142
} chassis_motor_ID;
// rm电机统一数据结构体
typedef struct
{
	uint16_t position;
	int16_t speed;
	int16_t current;
	uint8_t temperature;
} motor_6020_t;
//超级电容
typedef struct
{
	float input_voltage;	   //输入电压
	float Capacitance_voltage; //电容电压
	float Input_current;	   //输入电流
	float Set_power;		   //设定功率
} supercapacitor_receive_t;	   //超级电容器



typedef struct  
{
    uint8_t Angle_Pid_Kp;
    uint8_t Angle_Pid_Ki;
    
    uint8_t Speed_Pid_Kp;
    uint8_t Speed_Pid_Ki;
    
    uint8_t Torqur_Pid_Kp;
    uint8_t Torqur_Pid_Ki;
    
    int32_t Accel;            //uint:1dps/s
    double Angle_Accel;
    double X_Accel;
    
    uint16_t Encoder;         //range:0~16384(14bit)
    uint16_t Encoder_Raw;     //range:0~16384(14bit)
    uint16_t Encoder_Offset;  //range:0~16384(14bit)
    
    double Muli_Angle;      //uint:0.01°/LSB(Angle of many turns)(+:clockwise -:anticlockwise) 
    double Last_Muli_Angle;      //uint:0.01°/LSB(Angle of many turns)(+:clockwise -:anticlockwise) 
    
    uint32_t Circle_Angle;    //uint:0.01°/LSB(range:0~36000*reduction_ratio-1)
    
    int8_t Temperature;       //uint:1℃/LSB
    uint16_t Voltage;         //uint:0.1V/LSB
    uint8_t Error_State;      //Temperatur and Voltage
    
    int16_t Torque_Current;   //uint:-2048~2048(-33A~33A)
    int16_t Speed;            //uint:1dps/LSB
    int16_t Last_Speed;            //uint:1dps/LSB
    
    
    int16_t A_Phase_Current;  //uint:1A/64LSB
    int16_t B_Phase_Current;  //uint:1A/64LSB
    int16_t C_Phase_Current;  //uint:1A/64LSB
    
    int16_t Power;            //uint:-1000~1000
    
    int16_t Power_Control;            //uint:-1000~1000
    int16_t Torque_Current_Control;   //uint:-2000~2000(-32A~32A?)
    int32_t Speed_Control;            //uint:0.01dps/LSB
    int32_t Angle_Control;            //uint:0.01°/LSB(target-actual)(0~35999)
    
    uint16_t Max_Speed;               //uint:1dps/LSB
    uint16_t Max_Position;            //uint:0.01°/LSB
    
    uint8_t Spin_Direction;           //0:clockwise  1:anticlockwise
	
	int16_t Motor_output;
	
} motor_9025_t;

typedef struct
{
	RC_ctrl_t *Chassis_RC; //底盘遥控数据
	INS_t *Chassis_INS;

	chassis_behaviour_e behaviour; //底盘模式
	chassis_state_e chassis_state;
	
	motor_6020_t *yaw_motor;
	Encoder_t *yaw_motor_encoder;
	
	motor_9025_t *chassis_motor[2];
	Encoder_t *Chassis_Motor_encoder[2];
	
	LQR_t chassis_balance_lqr;
	LQR_t chassis_yaw_lqr;
	
	pid_parameter_t chassis_speedX_pid;
    pid_parameter_t chassis_positionX_pid;	
	
	SlidAveFilterObj chassis_output_l_filter;
    SlidAveFilterObj chassis_output_r_filter;

	SlidAveFilterObj chassis_gory;
	
	fp32 chassis_barycenter;
	
	fp32 Chassis_Gimbal_Diference_Angle; //底盘与云台的差角

	supercapacitor_receive_t *super_cap_c; //超电
	fp32 chassis_speed_gain;			   //速度因子
	
	double Stop_Position;	


} chassis_control_t;



#endif
