#include "chassis_behaviour.h"
#include "chassis_task.h"
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
        rc_behaviour = CHASSIS_FOLLOW;
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
        kb_behaviour = CHASSIS_FOLLOW; //底盘跟随模式
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
    //**G 补给模式
    if (Chassis_behaviour_f->Chassis_RC->kb.bit.G)
    {
        //kb_behaviour = CHASSIS_NO_FOLLOW; //补给模式下，底盘不跟随云台
    }
    //如果模式发生改变，设置对应的模式
    if (last_behaviour != kb_behaviour)
    {
        Chassis_behaviour_f->behaviour = kb_behaviour;
    }
}

void chassis_behaviour_react(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_x = (Chassis_behaviour_react_f->Chassis_RC->rc.ch[3] / 660.0f + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.A + Chassis_behaviour_react_f->Chassis_RC->kb.bit.D)) * CHASSIS_X_SEN;
    Chassis_yaw += (Chassis_behaviour_react_f->Chassis_RC->rc.ch[2] / 660.f + (-Chassis_behaviour_react_f->Chassis_RC->kb.bit.S + Chassis_behaviour_react_f->Chassis_RC->kb.bit.W)) / 1000.0f * CHASSIS_YAW_SEN;
	
	
	
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
    default:
        break;
    }

}
void chassis_state_choose(chassis_control_t *chassis_state_choose_f)
{
    chassis_state_e last_state;
    last_state = chassis_state_choose_f->chassis_state;
    if ((user_abs(chassis_state_choose_f->chassis_speedX_pid.SetValue) < chassis_state_choose_f->chassis_speedX_pid.stepIn))
    {
        chassis_state_choose_f->chassis_state = CHASSIS_SPEED;
    }
    else
    {
        chassis_state_choose_f->chassis_state = CHASSIS_SPEED;
    }
	
	
	if (chassis_state_choose_f->Chassis_RC->rc.s2 == RC_SW_DOWN)
    {
        chassis_state_choose_f->chassis_state = CHASSIS_ZERO_FORCE;
    }
	
	
    if (last_state != chassis_state_choose_f->chassis_state)
    {
        EncoderValZero(chassis_state_choose_f->Chassis_Motor_encoder[0]);
        EncoderValZero(chassis_state_choose_f->Chassis_Motor_encoder[1]);
        
        pid_clear(&chassis_state_choose_f->chassis_positionX_pid);
        pid_clear(&chassis_state_choose_f->chassis_speedX_pid);
    }

    
}
void chassis_speed_pid_calculate(chassis_control_t *chassis_speed_pid_calculate_f)
{
    PidCalculate(&chassis_speed_pid_calculate_f->chassis_speedX_pid, Chassis_x, (chassis_speed_pid_calculate_f->chassis_motor[0]->Speed - chassis_speed_pid_calculate_f->chassis_motor[1]->Speed) / 2.0 / RADIAN_COEF * MOTOR_RADIUS);
}
float see_lqr_speed = 0.0f;

void motor_lqr_calculate(chassis_control_t *lqr_calculate_f)
{
	lqr_calculate_f->chassis_balance_lqr.Input[0] = (lqr_calculate_f->chassis_motor[0]->Muli_Angle - lqr_calculate_f->chassis_motor[1]->Muli_Angle) / 2.0 / RADIAN_COEF * MOTOR_RADIUS;;
	lqr_calculate_f->chassis_balance_lqr.Input[1] = (lqr_calculate_f->Chassis_INS->Pitch + BARYCENTER_ZERO_OFFSET) / RADIAN_COEF;
	lqr_calculate_f->chassis_balance_lqr.Input[2] = (lqr_calculate_f->chassis_motor[0]->Speed - lqr_calculate_f->chassis_motor[1]->Speed) / 2.0 / RADIAN_COEF * MOTOR_RADIUS;
	lqr_calculate_f->chassis_balance_lqr.Input[3] = lqr_calculate_f->Chassis_INS->Gyro[0] ;		//弧度/s

	
	lqr_calculate_f->chassis_yaw_lqr.Input[0] = -float_min_distance(-Chassis_yaw, lqr_calculate_f->Chassis_INS->Yaw, -180, 180) / 57.0;
	
	lqr_calculate_f->chassis_yaw_lqr.Input[1] = lqr_calculate_f->Chassis_INS->Gyro[2];
	
	lqr_calculate_f->chassis_balance_lqr.Input[1] -=  lqr_calculate_f->chassis_speedX_pid.out / RADIAN_COEF;
	
	lqr_calculate_f->chassis_balance_lqr.Output[0] = lqr_calculate_f->chassis_balance_lqr.Input[0] * lqr_calculate_f->chassis_balance_lqr.k[0]    	\
													+ lqr_calculate_f->chassis_balance_lqr.Input[1] * lqr_calculate_f->chassis_balance_lqr.k[1]		\
													+ lqr_calculate_f->chassis_balance_lqr.Input[2] * lqr_calculate_f->chassis_balance_lqr.k[2]		\
													+ lqr_calculate_f->chassis_balance_lqr.Input[3] * lqr_calculate_f->chassis_balance_lqr.k[3];
	
	lqr_calculate_f->chassis_yaw_lqr.Output[0] = lqr_calculate_f->chassis_yaw_lqr.Input[0] * lqr_calculate_f->chassis_yaw_lqr.k[0]		\
												+ lqr_calculate_f->chassis_yaw_lqr.Input[1] * lqr_calculate_f->chassis_yaw_lqr.k[1];
	
	see_lqr_speed = lqr_calculate_f->chassis_balance_lqr.Output[0];

}




void f_CHASSIS_FOLLOW(chassis_control_t *Chassis_behaviour_react_f)
{
    //Chassis_yaw = Chassis_behaviour_react_f->Chassis_Gimbal_Diference_Angle;
}

void f_CHASSIS_ROTATION(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_yaw = CHASSIS_ROTATION_SPEED;
	Chassis_x = 0;
}
void f_CHASSIS_BATTERY(chassis_control_t *Chassis_behaviour_react_f)
{
    Chassis_x = 0;
    Chassis_yaw = 0;
}
