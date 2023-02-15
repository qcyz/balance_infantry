#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "pid.h"
#include "lqr.h"
#include "can1_receive.h"
#include "can2_receive.h"
#include "can1_send.h"
#include "can2_send.h"
#include "LK9025_communication.h"
#include "string.h"
#include "bsp_Motor_Encoder.h"
#include "maths.h"
#include "user_lib.h"
#include "chassis_config.h"


chassis_control_t Chassis_Control;

static void Chassis_Init(chassis_control_t *Chassis_Control);
static void Chassis_Work(chassis_control_t *Chassis_Control_f);
static void motor_position_speed_pid_calculate(chassis_control_t *motor_position_speed_pid_calculate_f);
static void chassis_zero_fore_react(chassis_control_t *chassis_zero_fore_react_f);
static void chassis_state_react(chassis_control_t *chassis_state_react_f);
static void chassis_prevent_motion_distortion(chassis_control_t *chassis_prevent_motion_distortion_f);
static void chassis_get_gimbal_differece_angle(chassis_control_t *chassis_get_gimbal_differece_angle_f);




double k_banlance[4] = {0,-71.5271420011465,0,-18.2171785142389};
double k_yaw[2] = {14.1421356237310, 2.70149737510667};



float see_l_out =  0;
void Task_Chassis(void const *argument)
{
	Chassis_Init(&Chassis_Control);
	vTaskDelay(5);

	while (1)
	{
		taskENTER_CRITICAL(); //进入临界区
		

		Chassis_Work(&Chassis_Control);

		can2_chassis_to_gimbal(Chassis_Control.Chassis_RC);
//				Chassis_TO_Gimbal(&Chassis_Control);
		chassis_motor_9025_send(Chassis_Control.chassis_motor[0]->Motor_output,	\
								Chassis_Control.chassis_motor[1]->Motor_output);
		
		see_l_out = Chassis_Control.chassis_motor[0]->Motor_output;
		
		
		taskEXIT_CRITICAL(); //退出临界区

		//vTaskDelayUntil(&currentTime, 1); //绝对延时//vTaskDelay(2)
		vTaskDelay(1);
	}
}

void Chassis_Work(chassis_control_t *Chassis_Control_f)
{
	//底盘云台角度计算
	chassis_get_gimbal_differece_angle(Chassis_Control_f);

	//选择底盘模式
	chassis_behaviour_choose(Chassis_Control_f);

	//根据底盘模式计算x、yaw值
	chassis_behaviour_react(Chassis_Control_f);
	
	//底盘状态选择
	chassis_state_choose(Chassis_Control_f);

	//根据底盘状态计算电机输出力矩
	chassis_state_react(Chassis_Control_f);

	//防止运动失真
	chassis_prevent_motion_distortion(Chassis_Control_f);
	
}
/**
 * @brief          底盘数据初始化
 * @param[in]      *chassis_move_init_f：底盘主结构体
 * @retval         none
 */
static void Chassis_Init(chassis_control_t *chassis_data_init_f)
{
	memset(chassis_data_init_f, 0, sizeof(chassis_control_t));
	/*--------------------初始化指针--------------------*/
	//获取遥控的指针
	chassis_data_init_f->Chassis_RC = RC_Get_RC_Pointer();
	
	//获取陀螺仪指针
    chassis_data_init_f->Chassis_INS = Get_INS_Point();
	
	//获取底盘电机的指针
	chassis_data_init_f->chassis_motor[0] = get_chassis_motor_l_point();
	chassis_data_init_f->chassis_motor[1] = get_chassis_motor_r_point();

	chassis_data_init_f->yaw_motor = get_yaw_motor_measure_point();
	
	//获取超级电容的指针
	chassis_data_init_f->super_cap_c = get_supercap_control_point();


	/*--------------------初始化编码器--------------------*/
	chassis_data_init_f->Chassis_Motor_encoder[0] = Encoder_Init(LK9025, 1);
	chassis_data_init_f->Chassis_Motor_encoder[1] = Encoder_Init(LK9025, 2);
	chassis_data_init_f->yaw_motor_encoder = Encoder_Init(GM6020, 3);//YAW

	/*--------------------初始化pid--------------------*/
	/*底盘pid初始化*/
	PidInit(&chassis_data_init_f->chassis_speedX_pid, CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD, Output_Limit | StepIn);
	PidInit(&chassis_data_init_f->chassis_positionX_pid, CHASSIS_POSITION_KP, CHASSIS_POSITION_KI, CHASSIS_POSITION_KD, Output_Limit );

	PidInitMode(&chassis_data_init_f->chassis_speedX_pid, Output_Limit, CHASSIS_PITCH_LIMIT, 0);
	PidInitMode(&chassis_data_init_f->chassis_speedX_pid, StepIn, 0.3F, 0);
	
	PidInitMode(&chassis_data_init_f->chassis_positionX_pid, Output_Limit, CHASSIS_SPEED_LIMIT, 0);

	/*底盘lqr初始化*/
	LQR_Init(&chassis_data_init_f->chassis_balance_lqr, 4, 1, k_banlance);
	LQR_Init(&chassis_data_init_f->chassis_yaw_lqr, 2, 1, k_yaw);

	/*底盘输出滤波初始化*/
	sliding_average_filter_init(&chassis_data_init_f->chassis_output_l_filter, 10);
	sliding_average_filter_init(&chassis_data_init_f->chassis_output_r_filter, 10);
	
	chassis_data_init_f->behaviour = CHASSIS_FOLLOW;
	//速度因子
	chassis_data_init_f->chassis_speed_gain = 1;
}
void chassis_state_react(chassis_control_t *chassis_state_react_f)
{
	switch (chassis_state_react_f->chassis_state)
	{
	case CHASSIS_LOCK_POSITION:
		//位置速度环串级pid计算
		motor_position_speed_pid_calculate(chassis_state_react_f);
		//底盘lqr计算
		motor_lqr_calculate(chassis_state_react_f);
		break;
	case CHASSIS_SPEED:
		//底盘速度pid计算
		chassis_speed_pid_calculate(chassis_state_react_f);
		//底盘lqr计算
		motor_lqr_calculate(chassis_state_react_f);
	
		break;
	case CHASSIS_ZERO_FORCE:
		chassis_zero_fore_react(chassis_state_react_f);
		break;
	default:
		break;
	}
}

void chassis_prevent_motion_distortion(chassis_control_t *chassis_prevent_motion_distortion_f)
{
	chassis_prevent_motion_distortion_f->chassis_motor[0]->Motor_output = TORQUE_TO_OUTPUT * (-chassis_prevent_motion_distortion_f->chassis_balance_lqr.Output[0] + chassis_prevent_motion_distortion_f->chassis_yaw_lqr.Output[0]) / 2.0;
	chassis_prevent_motion_distortion_f->chassis_motor[1]->Motor_output = TORQUE_TO_OUTPUT * (chassis_prevent_motion_distortion_f->chassis_balance_lqr.Output[0] + chassis_prevent_motion_distortion_f->chassis_yaw_lqr.Output[0]) / 2.0;
	
	chassis_prevent_motion_distortion_f->chassis_motor[0]->Motor_output = sliding_average_filter(&chassis_prevent_motion_distortion_f->chassis_output_l_filter, chassis_prevent_motion_distortion_f->chassis_motor[0]->Motor_output);
	chassis_prevent_motion_distortion_f->chassis_motor[1]->Motor_output = sliding_average_filter(&chassis_prevent_motion_distortion_f->chassis_output_r_filter, chassis_prevent_motion_distortion_f->chassis_motor[1]->Motor_output);

}
void chassis_zero_fore_react(chassis_control_t *chassis_zero_fore_react_f)
{
	chassis_zero_fore_react_f->chassis_balance_lqr.Output[0] = 0;
	chassis_zero_fore_react_f->chassis_yaw_lqr.Output[0] = 0;
}
void motor_position_speed_pid_calculate(chassis_control_t *motor_position_speed_pid_calculate_f)
{
	motor_position_speed_control(&motor_position_speed_pid_calculate_f->chassis_speedX_pid, &motor_position_speed_pid_calculate_f->chassis_positionX_pid, 0, (motor_position_speed_pid_calculate_f->Chassis_Motor_encoder[0]->Encode_Record_Val- motor_position_speed_pid_calculate_f->Chassis_Motor_encoder[1]->Encode_Record_Val) / 2.0, (motor_position_speed_pid_calculate_f->chassis_motor[0]->Speed - motor_position_speed_pid_calculate_f->chassis_motor[0]->Speed) / 2.0f);
}


void chassis_get_gimbal_differece_angle(chassis_control_t *chassis_get_gimbal_differece_angle_f)
{
	chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle = ((float)(chassis_get_gimbal_differece_angle_f->yaw_motor_encoder->Encode_Actual_Val - YAW_ZERO_OFFSET)* 360.0f / 8192.0f);
    chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle = loop_float_constrain(chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle, -180.0f, 180.0f);
}
