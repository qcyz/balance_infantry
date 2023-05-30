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
#include "bsp_referee.h"

chassis_control_t Chassis_Control;

static void Chassis_Init(chassis_control_t *Chassis_Control);
static void Chassis_Work(chassis_control_t *Chassis_Control_f);



double k_banlance[4] = {0,-60.9048292577891,-0.00690578556486,-10.47729819946999};
double k_yaw[2] = {10.1421356237309,3.53567287862321};

void Task_Chassis(void const *argument)
{
	Chassis_Init(&Chassis_Control);
	vTaskDelay(5);

	while (1)
	{
		taskENTER_CRITICAL(); //进入临界区
		
		chassis_motor_read_turns();//TODO:存在临界区中接收不到can信号的问题

		Chassis_Work(&Chassis_Control);

		can2_chassis_to_gimbal(Chassis_Control.Chassis_RC);
		can2_chassis_to_gimbal_referee(Chassis_Control.Chassis_Referee, Chassis_Control.Chassis_INS->Pitch);
//			
		chassis_motor_9025_send(Chassis_Control.chassis_motor[0]->Motor_output,	\
								Chassis_Control.chassis_motor[1]->Motor_output);
		
//		chassis_barycenter_dispose(&Chassis_Control);    
		can1_cap_setmsg(Chassis_Control.Chassis_Referee->Robot_Status.chassis_power_limit - 2);
	
		taskEXIT_CRITICAL(); //退出临界区

		vTaskDelay(1);
	}
}

void Chassis_Work(chassis_control_t *Chassis_Control_f)
{
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
	
	//获取裁判系统指针
	chassis_data_init_f->Chassis_Referee = Get_referee_Address();
	
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
	PidInit(&chassis_data_init_f->chassis_speedX_pid, CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD, Output_Limit | StepIn | Separated_Integral | Integral_Limit);
	PidInit(&chassis_data_init_f->chassis_positionX_pid, CHASSIS_POSITION_KP, CHASSIS_POSITION_KI, CHASSIS_POSITION_KD, Output_Limit );

	PidInitMode(&chassis_data_init_f->chassis_speedX_pid, Output_Limit, CHASSIS_PITCH_LIMIT, 0);
	PidInitMode(&chassis_data_init_f->chassis_speedX_pid, StepIn, 0.3F, 0);
	PidInitMode(&chassis_data_init_f->chassis_speedX_pid, Separated_Integral, 0.3f, -0.3f);
    PidInitMode(&chassis_data_init_f->chassis_speedX_pid, Integral_Limit, 500, 0);
	
	PidInitMode(&chassis_data_init_f->chassis_positionX_pid, Output_Limit, CHASSIS_SPEED_LIMIT, 0);

	/*底盘lqr初始化*/
	LQR_Init(&chassis_data_init_f->chassis_balance_lqr, 4, 1, k_banlance);
	LQR_Init(&chassis_data_init_f->chassis_yaw_lqr, 2, 1, k_yaw);

	/*底盘输出滤波初始化*/
	sliding_average_filter_init(&chassis_data_init_f->chassis_output_l_filter, 10);
	sliding_average_filter_init(&chassis_data_init_f->chassis_output_r_filter, 10);
	
	/*底盘输出滤波初始化*/
	sliding_average_filter_init(&chassis_data_init_f->chassis_gory, 10);
	
	chassis_data_init_f->behaviour = CHASSIS_NO_FOLLOW;
	chassis_data_init_f->chassis_state = CHASSIS_ZERO_FORCE;
	//速度因子
	chassis_data_init_f->chassis_speed_gain = 1;
	chassis_data_init_f->chassis_barycenter = BARYCENTER_ZERO_OFFSET;
}

chassis_control_t *Get_Chassis_Control_Point(void)
{
	return &Chassis_Control;
}