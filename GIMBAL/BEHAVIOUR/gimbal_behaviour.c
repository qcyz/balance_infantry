
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "maths.h"
#include "user_lib.h"
#include "gimbal_config.h"
#include "virtual_task.h"
#include "can2_receive.h"

float Gimbal_pitch = 0.0f;
float Gimbal_yaw = 0.0f;
chassis_control_t *Chassis_Control;

static void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f);
static void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f);
static void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f);
float torque_to_voltage_6020(float torque);



float *get_Gimbal_pitch_point(void)
{
    return &Gimbal_pitch;
}

float *get_Gimbal_yaw_point(void)
{
    return &Gimbal_yaw;
}

void gimbal_behaviour_choose(gimbal_control_t *gimbal_behaviour_choose_f)
{
    gimbal_behaviour_e last_behaviour;
    static gimbal_behaviour_e rc_behaviour = GIMBAL_MANUAL;
    static gimbal_behaviour_e kb_behaviour = GIMBAL_MANUAL;

    // 手柄
    last_behaviour = rc_behaviour;
    switch (gimbal_behaviour_choose_f->Gimbal_RC->rc.s2)
    {
    case RC_SW_UP:
        rc_behaviour = GIMBAL_MANUAL;
        break;
    case RC_SW_MID:
        rc_behaviour = GIMBAL_AUTOATTACK;
        break;
    case RC_SW_DOWN:
        rc_behaviour = GIMBAL_AUTOBUFF;
        break;
    default:
        break;
    }
	//test 
	

    // 如果挡位发生改变，设置对应的模式
    if (last_behaviour != rc_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = rc_behaviour;
    }

    // 键鼠
    last_behaviour = kb_behaviour;
    //**c
    if (gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.C)
    {
        kb_behaviour = GIMBAL_MANUAL;
    }
	//**v
    if (gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.V)
    {
        kb_behaviour = GIMBAL_TOPBUFF;
    }
	//**v
    if (gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.B)
    {
        kb_behaviour = GIMBAL_AUTOBUFF;
    }
	
	if (gimbal_behaviour_choose_f->Gimbal_RC->mouse.press_r)
    {
        kb_behaviour = GIMBAL_AUTOATTACK;
    }
    // 如果模式发生改变，设置对应的模式
    if (last_behaviour != kb_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = kb_behaviour;
    }
	
	//test
}
float see_gimbal_yaw = 0;
void gimbal_behaviour_react(gimbal_control_t *gimbal_behaviour_react_f)
{
	float pitch_up_limit = PITCH_ANGLE_LIMIT_UP;
	float pitch_down_limit = PITCH_ANGLE_LIMIT_DOWN;
	Chassis_Control = get_chassis_control_point();
	
	//解算云台pitch限幅
	pitch_up_limit = PITCH_ANGLE_LIMIT_UP + Chassis_Control->pitch * arm_cos_f32(loop_float_constrain((gimbal_behaviour_react_f->Yaw_c.motor_measure->position - YAW_ZERO_OFFSET) / 8192.0f * 360.0f , -180.0f, 180.0f) / RADIAN_COEF);
	pitch_down_limit = PITCH_ANGLE_LIMIT_DOWN + Chassis_Control->pitch * arm_cos_f32(loop_float_constrain((gimbal_behaviour_react_f->Yaw_c.motor_measure->position - YAW_ZERO_OFFSET) / 8192.0f * 360.0f , -180.0f, 180.0f) / RADIAN_COEF);

	
    Gimbal_pitch -= gimbal_behaviour_react_f->Gimbal_RC->mouse.y * MOUSE_PITCH_SPEED;
    Gimbal_pitch += gimbal_behaviour_react_f->Gimbal_RC->rc.ch[1] * RC_PITCH_SPEED;
	
	
    value_limit(Gimbal_pitch, pitch_down_limit, pitch_up_limit);

    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->mouse.x * MOUSE_YAW_SPEED;
    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->rc.ch[0] * RC_YAW_SPEED;
	Gimbal_yaw = loop_float_constrain(Gimbal_yaw, -180,180);

	
    switch (gimbal_behaviour_react_f->gimbal_behaviour)
    {
    case GIMBAL_MANUAL:
        f_GIMBAL_MANUAL(gimbal_behaviour_react_f);
        break;
    case GIMBAL_AUTOATTACK:
        f_GIMBAL_AUTOATTACK(gimbal_behaviour_react_f);
        break;
    case GIMBAL_AUTOBUFF:
        f_GIMBAL_AUTOBUFF(gimbal_behaviour_react_f);
        break;
		case GIMBAL_TOPBUFF:
				f_GIMBAL_AUTOBUFF(gimbal_behaviour_react_f);
    default:
        break;
    }
}

void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f)
{
	f_GIMBAL_MANUAL_f->Yaw_c.motor_target = float_min_distance(Gimbal_yaw, f_GIMBAL_MANUAL_f->Gimbal_INS->Yaw, -180, 180);
	f_GIMBAL_MANUAL_f->Pitch_c.motor_target = float_min_distance(Gimbal_pitch, f_GIMBAL_MANUAL_f->Gimbal_INS->Pitch, -180, 180);;
	
}

void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f)
{
	
	
	if(((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw != 0) || ((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch != 0))
	{
		Gimbal_pitch = (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch + f_GIMBAL_AUTOATTACK_f->Gimbal_INS->Pitch;
		Gimbal_yaw = (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw + f_GIMBAL_AUTOATTACK_f->Gimbal_INS->Yaw;
		
		Gimbal_pitch = loop_float_constrain(Gimbal_pitch, -180,180);
		Gimbal_yaw = loop_float_constrain(Gimbal_yaw, -180,180);

	
		f_GIMBAL_AUTOATTACK_f->Yaw_c.motor_target = float_min_distance(Gimbal_yaw, f_GIMBAL_AUTOATTACK_f->Gimbal_INS->Yaw, -180, 180);
		f_GIMBAL_AUTOATTACK_f->Pitch_c.motor_target = float_min_distance(Gimbal_pitch, f_GIMBAL_AUTOATTACK_f->Gimbal_INS->Pitch, -180, 180);;
//	
//		f_GIMBAL_AUTOATTACK_f->Yaw_c.motor_target = (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw;
//		f_GIMBAL_AUTOATTACK_f->Pitch_c.motor_target = (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch;
		gimbal_clear_virtual_recive();
	}
	else
	{
		f_GIMBAL_MANUAL(f_GIMBAL_AUTOATTACK_f);
	}
}
void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f)
{
	if(((*f_GIMBAL_AUTOBUFF_f->auto_c)->auto_yaw != 0) || ((*f_GIMBAL_AUTOBUFF_f->auto_c)->auto_pitch != 0))
	{
		Gimbal_pitch = (*f_GIMBAL_AUTOBUFF_f->auto_c)->auto_pitch + f_GIMBAL_AUTOBUFF_f->Gimbal_INS->Pitch;
		Gimbal_yaw = (*f_GIMBAL_AUTOBUFF_f->auto_c)->auto_yaw + f_GIMBAL_AUTOBUFF_f->Gimbal_INS->Yaw;
		
		Gimbal_pitch = loop_float_constrain(Gimbal_pitch, -180,180);
		Gimbal_yaw = loop_float_constrain(Gimbal_yaw, -180,180);

	
		f_GIMBAL_AUTOBUFF_f->Yaw_c.motor_target = float_min_distance(Gimbal_yaw, f_GIMBAL_AUTOBUFF_f->Gimbal_INS->Yaw, -180, 180);
		f_GIMBAL_AUTOBUFF_f->Pitch_c.motor_target = float_min_distance(Gimbal_pitch, f_GIMBAL_AUTOBUFF_f->Gimbal_INS->Pitch, -180, 180);;

		gimbal_clear_virtual_recive();
	}
	else
	{
		f_GIMBAL_MANUAL(f_GIMBAL_AUTOBUFF_f);
	}
}


void gimbal_motor_calculate(gimbal_control_t *motor_calculate_f)
{
	double pitch_system_state[2] = {((-motor_calculate_f->Pitch_c.motor_target) / 57.295779513f), motor_calculate_f->Gimbal_INS->Gyro[0]};
	double yaw_system_state[2] = {((-motor_calculate_f->Yaw_c.motor_target) / 57.295779513f), motor_calculate_f->Gimbal_INS->Gyro[2]};
	
	//Pitch与Yaw轴PID计算
	PidCalculate(&motor_calculate_f->Pitch_c.motor_pid, Gimbal_pitch, motor_calculate_f->Gimbal_INS->Pitch);
	PidCalculate(&motor_calculate_f->Yaw_c.motor_pid, Gimbal_yaw, motor_calculate_f->Gimbal_INS->Yaw);
	


	//Pitch与Yaw轴LQR计算
	if(motor_calculate_f->gimbal_behaviour == GIMBAL_MANUAL)
	{
		LQR_Data_Update(&motor_calculate_f->Pitch_c.motor_lqr, pitch_system_state);
		LQR_Calculate(&motor_calculate_f->Pitch_c.motor_lqr);
		motor_calculate_f->Pitch_c.motor_lqr.Output[0] = sliding_average_filter(&motor_calculate_f->Pitch_c.motor_filter, motor_calculate_f->Pitch_c.motor_lqr.Output[0]);
		motor_calculate_f->Pitch_c.motor_output = torque_to_voltage_6020(motor_calculate_f->Pitch_c.motor_lqr.Output[0]) + motor_calculate_f->Pitch_c.motor_pid.out;

		
		LQR_Data_Update(&motor_calculate_f->Yaw_c.motor_lqr, yaw_system_state);
		LQR_Calculate(&motor_calculate_f->Yaw_c.motor_lqr);
		motor_calculate_f->Yaw_c.motor_lqr.Output[0] = sliding_average_filter(&motor_calculate_f->Yaw_c.motor_filter, motor_calculate_f->Yaw_c.motor_lqr.Output[0]);
		motor_calculate_f->Yaw_c.motor_output = torque_to_voltage_6020(motor_calculate_f->Yaw_c.motor_lqr.Output[0]) + motor_calculate_f->Yaw_c.motor_pid.out;

	}
	else if(motor_calculate_f->gimbal_behaviour == GIMBAL_AUTOATTACK || motor_calculate_f->gimbal_behaviour == GIMBAL_AUTOBUFF || motor_calculate_f->gimbal_behaviour == GIMBAL_TOPBUFF)
	{
		LQR_Data_Update(&motor_calculate_f->Pitch_c.virtual_motor_lqr, pitch_system_state);
		LQR_Calculate(&motor_calculate_f->Pitch_c.virtual_motor_lqr);
		motor_calculate_f->Pitch_c.virtual_motor_lqr.Output[0] = sliding_average_filter(&motor_calculate_f->Pitch_c.motor_filter, motor_calculate_f->Pitch_c.virtual_motor_lqr.Output[0]);
		motor_calculate_f->Pitch_c.motor_output = torque_to_voltage_6020(motor_calculate_f->Pitch_c.virtual_motor_lqr.Output[0]) + motor_calculate_f->Pitch_c.motor_pid.out;

		
		LQR_Data_Update(&motor_calculate_f->Yaw_c.virtual_motor_lqr, yaw_system_state);
		LQR_Calculate(&motor_calculate_f->Yaw_c.virtual_motor_lqr);
		motor_calculate_f->Yaw_c.virtual_motor_lqr.Output[0] = sliding_average_filter(&motor_calculate_f->Yaw_c.motor_filter, motor_calculate_f->Yaw_c.virtual_motor_lqr.Output[0]);
		motor_calculate_f->Yaw_c.motor_output = torque_to_voltage_6020(motor_calculate_f->Yaw_c.virtual_motor_lqr.Output[0]) + motor_calculate_f->Yaw_c.motor_pid.out;

	}
	
	
	

	//限幅
	motor_calculate_f->Pitch_c.motor_output = abs_limit(motor_calculate_f->Pitch_c.motor_output, 25000);
	motor_calculate_f->Yaw_c.motor_output = abs_limit(motor_calculate_f->Yaw_c.motor_output, 25000);

}

float torque_to_voltage_6020(float torque)
{
	float voltage = 0.0f;
		
	float current = torque / 0.741f * 10000;
	voltage = (current - 128.3507f) / 0.7778f;
	voltage = abs_limit(voltage,25000);
	
	return voltage;
		
}
