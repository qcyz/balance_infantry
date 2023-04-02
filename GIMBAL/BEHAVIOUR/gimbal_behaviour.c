#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "maths.h"
#include "user_lib.h"
#include "gimbal_config.h"
#include "virtual_task.h"

float Gimbal_pitch = 0.0f;
float Gimbal_yaw = 0.0f;

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
    // 如果模式发生改变，设置对应的模式
    if (last_behaviour != kb_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = kb_behaviour;
    }
}
float see_gimbal_yaw = 0;
void gimbal_behaviour_react(gimbal_control_t *gimbal_behaviour_react_f)
{
    Gimbal_pitch -= gimbal_behaviour_react_f->Gimbal_RC->mouse.y * MOUSE_PITCH_SPEED;
    Gimbal_pitch += gimbal_behaviour_react_f->Gimbal_RC->rc.ch[1] * RC_PITCH_SPEED;
    value_limit(Gimbal_pitch, PITCH_ANGLE_LIMIT_DOWN, PITCH_ANGLE_LIMIT_UP);

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
    default:
        break;
    }
}

void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f)
{
	f_GIMBAL_MANUAL_f->Yaw_c.motor_target = float_min_distance(Gimbal_yaw, f_GIMBAL_MANUAL_f->Gimbal_INS->Yaw, -180, 180);
	f_GIMBAL_MANUAL_f->Pitch_c.motor_target = float_min_distance(Gimbal_pitch, f_GIMBAL_MANUAL_f->Gimbal_INS->Pitch, -180, 180);;
	
}
float k1 = 80.0f;
void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f)
{
	
//	
//	if(((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw != 0) || ((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch != 0))
//	{
		Gimbal_pitch += (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch / k1;
		Gimbal_yaw += (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw / k1;
		f_GIMBAL_AUTOATTACK_f->Yaw_c.motor_target = float_min_distance(Gimbal_yaw, f_GIMBAL_AUTOATTACK_f->Gimbal_INS->Yaw, -180, 180);
		f_GIMBAL_AUTOATTACK_f->Pitch_c.motor_target = float_min_distance(Gimbal_pitch, f_GIMBAL_AUTOATTACK_f->Gimbal_INS->Pitch, -180, 180);;
//	
//		f_GIMBAL_AUTOATTACK_f->Yaw_c.motor_target = (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw;
//		f_GIMBAL_AUTOATTACK_f->Pitch_c.motor_target = (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch;
		gimbal_clear_virtual_recive();
//	}
//	else
//	{
//		f_GIMBAL_MANUAL(f_GIMBAL_AUTOATTACK_f);
//	}
}
void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f)
{
}


void gimbal_motor_calculate(gimbal_control_t *motor_calculate_f)
{
	double pitch_system_state[2] = {((-motor_calculate_f->Pitch_c.motor_target) / 57.295779513f), motor_calculate_f->Gimbal_INS->Gyro[0]};
	double yaw_system_state[2] = {((-motor_calculate_f->Yaw_c.motor_target) / 57.295779513f), motor_calculate_f->Gimbal_INS->Gyro[2]};
	
	LQR_Data_Update(&motor_calculate_f->Pitch_c.motor_lqr, pitch_system_state);
	LQR_Calculate(&motor_calculate_f->Pitch_c.motor_lqr);
	PidCalculate(&motor_calculate_f->Pitch_c.motor_pid, Gimbal_pitch, motor_calculate_f->Gimbal_INS->Pitch );
	motor_calculate_f->Pitch_c.motor_lqr.Output[0] = sliding_average_filter(&motor_calculate_f->Pitch_c.motor_filter, motor_calculate_f->Pitch_c.motor_lqr.Output[0]);
	motor_calculate_f->Pitch_c.motor_output = torque_to_voltage_6020(motor_calculate_f->Pitch_c.motor_lqr.Output[0]) + motor_calculate_f->Pitch_c.motor_pid.out;
	motor_calculate_f->Pitch_c.motor_output = abs_limit(motor_calculate_f->Pitch_c.motor_output, 25000);
	
	
	LQR_Data_Update(&motor_calculate_f->Yaw_c.motor_lqr, yaw_system_state);
	LQR_Calculate(&motor_calculate_f->Yaw_c.motor_lqr);
	PidCalculate(&motor_calculate_f->Yaw_c.motor_pid, Gimbal_yaw, motor_calculate_f->Gimbal_INS->Yaw);
	motor_calculate_f->Yaw_c.motor_lqr.Output[0] = sliding_average_filter(&motor_calculate_f->Yaw_c.motor_filter, motor_calculate_f->Yaw_c.motor_lqr.Output[0]);
	motor_calculate_f->Yaw_c.motor_output = torque_to_voltage_6020(motor_calculate_f->Yaw_c.motor_lqr.Output[0]) + motor_calculate_f->Yaw_c.motor_pid.out;
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
