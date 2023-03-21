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




double k_banlance[4] = {0,-36.9048292577891,-1.00690578556486,-5.47729819946999};
double k_yaw[2] = {20.1421356237309,2.33567287862321};


float see_r_out =  0;

float see_l_out =  0;
void Task_Chassis(void const *argument)
{
	Chassis_Init(&Chassis_Control);
	vTaskDelay(5);

	while (1)
	{
		taskENTER_CRITICAL(); //�����ٽ���
		
		chassis_motor_read_turns();

		Chassis_Work(&Chassis_Control);

		can2_chassis_to_gimbal(Chassis_Control.Chassis_RC);
//				Chassis_TO_Gimbal(&Chassis_Control);
		chassis_motor_9025_send(Chassis_Control.chassis_motor[0]->Motor_output,	\
								Chassis_Control.chassis_motor[1]->Motor_output);
		
//		chassis_barycenter_dispose(&Chassis_Control);
		see_l_out =  Chassis_Control.chassis_motor[0]->Motor_output;
		see_r_out =  Chassis_Control.chassis_motor[1]->Motor_output;
		taskEXIT_CRITICAL(); //�˳��ٽ���

		//vTaskDelayUntil(&currentTime, 1); //������ʱ//vTaskDelay(2)
		vTaskDelay(1);
	}
}

void Chassis_Work(chassis_control_t *Chassis_Control_f)
{
	//������̨�Ƕȼ���
	chassis_get_gimbal_differece_angle(Chassis_Control_f);

	//ѡ�����ģʽ
	chassis_behaviour_choose(Chassis_Control_f);

	//���ݵ���ģʽ����x��yawֵ
	chassis_behaviour_react(Chassis_Control_f);
	
	//����״̬ѡ��
	chassis_state_choose(Chassis_Control_f);

	//���ݵ���״̬�������������
	chassis_state_react(Chassis_Control_f);

	//��ֹ�˶�ʧ��
	chassis_prevent_motion_distortion(Chassis_Control_f);
	
}
/**
 * @brief          �������ݳ�ʼ��
 * @param[in]      *chassis_move_init_f���������ṹ��
 * @retval         none
 */
static void Chassis_Init(chassis_control_t *chassis_data_init_f)
{
	memset(chassis_data_init_f, 0, sizeof(chassis_control_t));
	/*--------------------��ʼ��ָ��--------------------*/
	//��ȡң�ص�ָ��
	chassis_data_init_f->Chassis_RC = RC_Get_RC_Pointer();
	
	//��ȡ������ָ��
    chassis_data_init_f->Chassis_INS = Get_INS_Point();
	
	//��ȡ���̵����ָ��
	chassis_data_init_f->chassis_motor[0] = get_chassis_motor_l_point();
	chassis_data_init_f->chassis_motor[1] = get_chassis_motor_r_point();

	chassis_data_init_f->yaw_motor = get_yaw_motor_measure_point();
	
	//��ȡ�������ݵ�ָ��
	chassis_data_init_f->super_cap_c = get_supercap_control_point();


	/*--------------------��ʼ��������--------------------*/
	chassis_data_init_f->Chassis_Motor_encoder[0] = Encoder_Init(LK9025, 1);
	chassis_data_init_f->Chassis_Motor_encoder[1] = Encoder_Init(LK9025, 2);
	chassis_data_init_f->yaw_motor_encoder = Encoder_Init(GM6020, 3);//YAW

	/*--------------------��ʼ��pid--------------------*/
	/*����pid��ʼ��*/
	PidInit(&chassis_data_init_f->chassis_speedX_pid, CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD, Output_Limit | StepIn);
	PidInit(&chassis_data_init_f->chassis_positionX_pid, CHASSIS_POSITION_KP, CHASSIS_POSITION_KI, CHASSIS_POSITION_KD, Output_Limit );

	PidInitMode(&chassis_data_init_f->chassis_speedX_pid, Output_Limit, CHASSIS_PITCH_LIMIT, 0);
	PidInitMode(&chassis_data_init_f->chassis_speedX_pid, StepIn, 0.3F, 0);
	
	PidInitMode(&chassis_data_init_f->chassis_positionX_pid, Output_Limit, CHASSIS_SPEED_LIMIT, 0);

	/*����lqr��ʼ��*/
	LQR_Init(&chassis_data_init_f->chassis_balance_lqr, 4, 1, k_banlance);
	LQR_Init(&chassis_data_init_f->chassis_yaw_lqr, 2, 1, k_yaw);

	/*��������˲���ʼ��*/
	sliding_average_filter_init(&chassis_data_init_f->chassis_output_l_filter, 10);
	sliding_average_filter_init(&chassis_data_init_f->chassis_output_r_filter, 10);
	
	/*��������˲���ʼ��*/
	sliding_average_filter_init(&chassis_data_init_f->chassis_gory, 10);
	
	chassis_data_init_f->behaviour = CHASSIS_FOLLOW;
	//�ٶ�����
	chassis_data_init_f->chassis_speed_gain = 1;
	chassis_data_init_f->chassis_barycenter = BARYCENTER_ZERO_OFFSET;
}

void chassis_state_react(chassis_control_t *chassis_state_react_f)
{
	switch (chassis_state_react_f->chassis_state)
	{
	case CHASSIS_LOCK_POSITION:
		//λ���ٶȻ�����pid����
		motor_position_speed_pid_calculate(chassis_state_react_f);
		//����lqr����
		motor_lqr_calculate(chassis_state_react_f);
		break;
	case CHASSIS_SPEED:
		//�����ٶ�pid����
		chassis_speed_pid_calculate(chassis_state_react_f);
		//����lqr����
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
	
	//ƽ�����
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
	motor_position_speed_control(&motor_position_speed_pid_calculate_f->chassis_speedX_pid, &motor_position_speed_pid_calculate_f->chassis_positionX_pid, motor_position_speed_pid_calculate_f->Stop_Position,get_chassis_position(motor_position_speed_pid_calculate_f), get_chassis_speed(motor_position_speed_pid_calculate_f));
}

void chassis_get_gimbal_differece_angle(chassis_control_t *chassis_get_gimbal_differece_angle_f)
{
	chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle = ((float)(chassis_get_gimbal_differece_angle_f->yaw_motor_encoder->Encode_Actual_Val - YAW_ZERO_OFFSET)* 360.0f / 8192.0f);
    chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle = loop_float_constrain(chassis_get_gimbal_differece_angle_f->Chassis_Gimbal_Diference_Angle, -180.0f, 180.0f);
}
