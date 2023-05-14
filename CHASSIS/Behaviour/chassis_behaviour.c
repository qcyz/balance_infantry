#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "chassis_config.h"
#include "bsp_dr16.h"
#include "maths.h"
#include "user_lib.h"
#include "pid.h"

//原始控制量
float Chassis_x = 0.0f;
float Chassis_yaw = 0.0f;


/********************函数声明********************/
void f_CHASSIS_FOLLOW(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_NO_FOLLOW(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_ROTATION(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_BATTERY(chassis_control_t *Chassis_behaviour_react_f);
void f_CHASSIS_SLIP(chassis_control_t *Chassis_behaviour_react_f);
void forward_test(chassis_control_t *Chassis_behaviour_react_f);


void chassis_behaviour_choose(chassis_control_t *Chassis_behaviour_f)
{
    //用于记录上一次数据
    chassis_behaviour_e last_behaviour;
    static chassis_behaviour_e rc_behaviour = CHASSIS_FOLLOW;
    static chassis_behaviour_e kb_behaviour = CHASSIS_FOLLOW;

    //手柄
    last_behaviour = rc_behaviour;
    switch (Chassis_behaviour_f->Chassis_RC->rc.s1)
    {
    case RC_SW_UP:
        rc_behaviour = CHASSIS_ROTATION;
        break;
    case RC_SW_MID:
        rc_behaviour = CHASSIS_FOLLOW;
        break;
    case RC_SW_DOWN:
        rc_behaviour = CHASSIS_NO_FOLLOW;
        break;
    default:
        break;
    }
    //如果挡位发生改变，设置对应的模式
    if (last_behaviour != rc_behaviour)
    {
        Chassis_behaviour_f->behaviour = rc_behaviour;
    }

    //键鼠
    last_behaviour = kb_behaviour;
    //**Q
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.Q)
    {
        kb_behaviour = CHASSIS_NO_FOLLOW; //底盘不跟随模式
    }
    //**E
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.E)
    {
        kb_behaviour = CHASSIS_ROTATION; //小陀螺模式
    }
    //**F
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.F)
    {
        kb_behaviour = CHASSIS_FOLLOW; //底盘跟随模式
    }
    //**R
	if (Chassis_behaviour_f->Chassis_RC->kb.bit.R)
    {
        kb_behaviour = CHASSIS_SLIP; //打滑模式下，底盘不跟随云台
    }
    //如果模式发生改变，设置对应的模式
    if (last_behaviour != kb_behaviour)
    {
        Chassis_behaviour_f->behaviour = kb_behaviour;
    }
}

void chassis_behaviour_react(chassis_control_t *Chassis_behaviour_react_f)
{

    switch (Chassis_behaviour_react_f->behaviour)
    {
    case CHASSIS_FOLLOW:
        f_CHASSIS_FOLLOW(Chassis_behaviour_react_f);
        break;
    case CHASSIS_ROTATION:
        f_CHASSIS_ROTATION(Chassis_behaviour_react_f);
        break;
    case CHASSIS_BATTERY:
        f_CHASSIS_BATTERY(Chassis_behaviour_react_f);
        break;
	case CHASSIS_NO_FOLLOW:
        f_CHASSIS_NO_FOLLOW(Chassis_behaviour_react_f);
        break;
	case CHASSIS_SLIP:
        f_CHASSIS_SLIP(Chassis_behaviour_react_f);
        break;
    default:
        break;
    }

}
void chassis_state_choose(chassis_control_t *chassis_state_choose_f)
{
    chassis_state_e last_state;
    last_state = chassis_state_choose_f->chassis_state;
    if (chassis_state_choose_f->behaviour == CHASSIS_ROTATION)
    {
        chassis_state_choose_f->chassis_state = CHASSIS_STAND;
    }
    else
    {
        chassis_state_choose_f->chassis_state = CHASSIS_SPEED;
    }
	
	
	if (chassis_state_choose_f->Chassis_RC->rc.s2 == RC_SW_DOWN || chassis_state_choose_f->behaviour == CHASSIS_SLIP)
    {
        chassis_state_choose_f->chassis_state = CHASSIS_ZERO_FORCE;
    }
	
	
    if (last_state != chassis_state_choose_f->chassis_state)
    {
		chassis_state_choose_f->Stop_Position = get_chassis_position(chassis_state_choose_f);
		
        EncoderValZero(chassis_state_choose_f->Chassis_Motor_encoder[0]);
        EncoderValZero(chassis_state_choose_f->Chassis_Motor_encoder[1]);
        
//        pid_clear(&chassis_state_choose_f->chassis_positionX_pid);
//        pid_clear(&chassis_state_choose_f->chassis_speedX_pid);
    }

    
}

void chassis_state_react(chassis_control_t *chassis_state_react_f)
{
	switch (chassis_state_react_f->chassis_state)
	{
	case CHASSIS_STAND:
		//位置速度环pid数据清除
		pid_clear(&chassis_state_react_f->chassis_speedX_pid);
		chassis_speed_pid_calculate(chassis_state_react_f);
		//forward_test(chassis_state_react_f);

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
		chassis_state_react_f->chassis_balance_lqr.Output[0] = Chassis_x * 4.0f;
		//chassis_state_react_f->chassis_yaw_lqr.Output[0] = Chassis_yaw;
		break;
	default:
		break;
	}
}


void chassis_prevent_motion_distortion(chassis_control_t *chassis_prevent_motion_distortion_f)
{
	
	chassis_prevent_motion_distortion_f->chassis_motor[0]->Motor_output = TORQUE_TO_OUTPUT * (-chassis_prevent_motion_distortion_f->chassis_balance_lqr.Output[0] + chassis_prevent_motion_distortion_f->chassis_yaw_lqr.Output[0]) / 2.0;
	chassis_prevent_motion_distortion_f->chassis_motor[1]->Motor_output = TORQUE_TO_OUTPUT * (chassis_prevent_motion_distortion_f->chassis_balance_lqr.Output[0] + chassis_prevent_motion_distortion_f->chassis_yaw_lqr.Output[0]) / 2.0;
	
//	//限幅
//	chassis_prevent_motion_distortion_f->chassis_motor[0]->Motor_output = float_constrain(chassis_prevent_motion_distortion_f->chassis_motor[0]->Motor_output,-1300, 1300);
//	chassis_prevent_motion_distortion_f->chassis_motor[1]->Motor_output = float_constrain(chassis_prevent_motion_distortion_f->chassis_motor[1]->Motor_output,-1300, 1300);

	//平滑输出
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


void chassis_speed_pid_calculate(chassis_control_t *chassis_speed_pid_calculate_f)
{
    PidCalculate(&chassis_speed_pid_calculate_f->chassis_speedX_pid, Chassis_x, get_chassis_speed(chassis_speed_pid_calculate_f));
}

void motor_lqr_calculate(chassis_control_t *lqr_calculate_f)
{
	double banlance_system_state[4] ={	(lqr_calculate_f->chassis_motor[0]->Muli_Angle - lqr_calculate_f->chassis_motor[1]->Muli_Angle) / 2.0 / RADIAN_COEF * MOTOR_RADIUS, \
										(lqr_calculate_f->Chassis_INS->Pitch + lqr_calculate_f->chassis_barycenter) / RADIAN_COEF - lqr_calculate_f->chassis_speedX_pid.out / RADIAN_COEF,		\
										(lqr_calculate_f->chassis_motor[0]->Speed - lqr_calculate_f->chassis_motor[1]->Speed) / 2.0 / RADIAN_COEF * MOTOR_RADIUS, \
										lqr_calculate_f->Chassis_INS->Gyro[0]		\
	};
	double yaw_system_state[2] ={	-float_min_distance(-Chassis_yaw, lqr_calculate_f->Chassis_INS->Yaw, -180, 180) / RADIAN_COEF, \
									lqr_calculate_f->Chassis_INS->Gyro[2]	\
	};
	
	
	//LQR数据更新
	LQR_Data_Update(&lqr_calculate_f->chassis_balance_lqr, banlance_system_state);
	LQR_Data_Update(&lqr_calculate_f->chassis_yaw_lqr, yaw_system_state);
	
	//LQR数据计算
	LQR_Calculate(&lqr_calculate_f->chassis_balance_lqr);
	LQR_Calculate(&lqr_calculate_f->chassis_yaw_lqr);
	
	
}


double get_chassis_position(chassis_control_t *get_position_f)
{
	return (get_position_f->chassis_motor[0]->Muli_Angle - get_position_f->chassis_motor[1]->Muli_Angle) / 2.0 / RADIAN_COEF * MOTOR_RADIUS;
}

double get_chassis_speed(chassis_control_t *get_speed_f)
{
	return (get_speed_f->chassis_motor[0]->Speed - get_speed_f->chassis_motor[1]->Speed) / 2.0 / RADIAN_COEF * MOTOR_RADIUS;
}

float see_position_speed;
float see_position;
float see_position_t;
//底盘重心调整
void chassis_barycenter_dispose(chassis_control_t *barycenter_dispose_f)
{
	float last_posion;
	last_posion = see_position_t;
    if ((user_abs(get_chassis_speed(barycenter_dispose_f)) < 0.1) && (Chassis_x == 0)	&& user_abs(barycenter_dispose_f->chassis_yaw_lqr.Output[0]) < 0.4)
	{
		see_position_speed = get_chassis_speed(barycenter_dispose_f);
		see_position = (get_chassis_position(barycenter_dispose_f) - barycenter_dispose_f->Stop_Position) * 100;
		see_position_t = see_position_t + (get_chassis_position(barycenter_dispose_f) - barycenter_dispose_f->Stop_Position);
		barycenter_dispose_f->chassis_barycenter += see_position / 5000;
	}

}



void f_CHASSIS_FOLLOW(chassis_control_t *Chassis_behaviour_react_f)
{
	Chassis_x = -((Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] / 660.0f + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.S + Chassis_behaviour_react_f->Chassis_RC->kb.bit.W)) * CHASSIS_X_SEN);
    Chassis_yaw += (Chassis_behaviour_react_f->Chassis_RC->rc.ch[2] / 660.f + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.A + Chassis_behaviour_react_f->Chassis_RC->kb.bit.D)) / CHASSIS_TASK_Hz * CHASSIS_YAW_SEN;
	
//4096 8192 4120
	if(Chassis_behaviour_react_f->yaw_motor->position > (YAW_ZERO_OFFSET - 8192 / 4) && Chassis_behaviour_react_f->yaw_motor->position < (YAW_ZERO_OFFSET + 8192 / 4))
	{
		Chassis_yaw = -((Chassis_behaviour_react_f->yaw_motor->position - YAW_ZERO_OFFSET) / 8192.0f * 360.0f + Chassis_behaviour_react_f->Chassis_INS->Yaw);
	}else
	{
		Chassis_yaw = -((Chassis_behaviour_react_f->yaw_motor->position - YAW_ZERO_OFFSET - 8192 / 2) / 8192.0f * 360.0f + Chassis_behaviour_react_f->Chassis_INS->Yaw);
		Chassis_x = (-Chassis_x);
	}
}
void f_CHASSIS_NO_FOLLOW(chassis_control_t *Chassis_behaviour_react_f)
{
	Chassis_x = -((Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] / 660.0f + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.S + Chassis_behaviour_react_f->Chassis_RC->kb.bit.W)) * CHASSIS_X_SEN);
    Chassis_yaw += (Chassis_behaviour_react_f->Chassis_RC->rc.ch[2] / 660.f + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.A + Chassis_behaviour_react_f->Chassis_RC->kb.bit.D)) / CHASSIS_TASK_Hz * CHASSIS_YAW_SEN;
	
}

void f_CHASSIS_ROTATION(chassis_control_t *Chassis_behaviour_react_f)
{

	
	if(abs(Chassis_behaviour_react_f->Chassis_RC->rc.ch[3]) > 1)
	{

		Chassis_yaw += (CHASSIS_ROTATION_SPEED + 100) / 1000.0f;
	}else
	{
		Chassis_behaviour_react_f->chassis_motor[0]->Speed = 0.0f;
		Chassis_behaviour_react_f->chassis_motor[1]->Speed = 0.0f;
		Chassis_yaw += (CHASSIS_ROTATION_SPEED + 100) / 1000.0f;
	}
	Chassis_x = arm_cos_f32(loop_float_constrain((Chassis_behaviour_react_f->Chassis_INS->Yaw - Chassis_behaviour_react_f->Chassis_RC->rc.ch[2] / 660.0f * 90.0f), -180.0f ,180.0f)/ RADIAN_COEF) * Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] / 4000.0f;
}
void f_CHASSIS_BATTERY(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_x = 0;
    Chassis_yaw = 0;
}
void f_CHASSIS_SLIP(chassis_control_t *Chassis_behaviour_react_f)
{ 
	Chassis_x = -((Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] / 660.0f + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.S + Chassis_behaviour_react_f->Chassis_RC->kb.bit.W)) * CHASSIS_X_SEN);
}
//向前（test）
void forward_test_plan2(chassis_control_t *Chassis_behaviour_react_f)
{
	double banlance_system_state[4] ={	(Chassis_behaviour_react_f->chassis_motor[0]->Muli_Angle - Chassis_behaviour_react_f->chassis_motor[1]->Muli_Angle) / 2.0 / RADIAN_COEF * MOTOR_RADIUS, \
										(Chassis_behaviour_react_f->Chassis_INS->Pitch + Chassis_behaviour_react_f->chassis_barycenter) / RADIAN_COEF - cos(loop_float_constrain((Chassis_behaviour_react_f->Chassis_INS->Yaw - Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] / 4.0f), -180.0f ,180.0f)) * 5.0f / RADIAN_COEF,		\
										(Chassis_behaviour_react_f->chassis_motor[0]->Speed - Chassis_behaviour_react_f->chassis_motor[1]->Speed) / 2.0 / RADIAN_COEF * MOTOR_RADIUS, \
										Chassis_behaviour_react_f->Chassis_INS->Gyro[0]		\

	};

	double yaw_system_state[2] ={	-float_min_distance(-Chassis_yaw, Chassis_behaviour_react_f->Chassis_INS->Yaw, -180, 180) / RADIAN_COEF, \
									Chassis_behaviour_react_f->Chassis_INS->Gyro[2]	\
	};

//	Chassis_x = cos(Chassis_behaviour_react_f->Chassis_INS->Yaw) * 2.0f;
//	Chassis_yaw += CHASSIS_ROTATION_SPEED / 1000.0f;
	
	//LQR数据更新
	LQR_Data_Update(&Chassis_behaviour_react_f->chassis_balance_lqr, banlance_system_state);
	LQR_Data_Update(&Chassis_behaviour_react_f->chassis_yaw_lqr, yaw_system_state);
	
	//LQR数据计算
	LQR_Calculate(&Chassis_behaviour_react_f->chassis_balance_lqr);
	LQR_Calculate(&Chassis_behaviour_react_f->chassis_yaw_lqr);
	
	
}

//向前（test）
float see_cos_yaw = 0.0f;
void forward_test(chassis_control_t *Chassis_behaviour_react_f)
{
	double banlance_system_state[4] ={	(Chassis_behaviour_react_f->chassis_motor[0]->Muli_Angle - Chassis_behaviour_react_f->chassis_motor[1]->Muli_Angle) / 2.0 / RADIAN_COEF * MOTOR_RADIUS, \
										(Chassis_behaviour_react_f->Chassis_INS->Pitch + Chassis_behaviour_react_f->chassis_barycenter) / RADIAN_COEF - arm_cos_f32(loop_float_constrain((Chassis_behaviour_react_f->Chassis_INS->Yaw - Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] / 4.0f), -180.0f ,180.0f) / RADIAN_COEF) * 6.0f / RADIAN_COEF,		\
										(Chassis_behaviour_react_f->chassis_motor[0]->Speed - Chassis_behaviour_react_f->chassis_motor[1]->Speed) / 2.0 / RADIAN_COEF * MOTOR_RADIUS, \
										Chassis_behaviour_react_f->Chassis_INS->Gyro[0]		\

	};

	double yaw_system_state[2] ={	-float_min_distance(-Chassis_yaw, Chassis_behaviour_react_f->Chassis_INS->Yaw, -180, 180) / RADIAN_COEF, \
									Chassis_behaviour_react_f->Chassis_INS->Gyro[2]	\
	};
	//if(abs(Chassis_behaviour_react_f->Chassis_INS->Yaw) > 180.0f)
//	Chassis_x = cos(Chassis_behaviour_react_f->Chassis_INS->Yaw) * 2.0f;
//	Chassis_yaw += CHASSIS_ROTATION_SPEED / 1000.0f;
	
	see_cos_yaw = arm_cos_f32(loop_float_constrain((Chassis_behaviour_react_f->Chassis_INS->Yaw - Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] / 4.0f), -180.0f ,180.0f) / RADIAN_COEF) * 4.0f;
	//LQR数据更新
	LQR_Data_Update(&Chassis_behaviour_react_f->chassis_balance_lqr, banlance_system_state);
	LQR_Data_Update(&Chassis_behaviour_react_f->chassis_yaw_lqr, yaw_system_state);
	
	//LQR数据计算
	LQR_Calculate(&Chassis_behaviour_react_f->chassis_balance_lqr);
	LQR_Calculate(&Chassis_behaviour_react_f->chassis_yaw_lqr);
	
	
}

