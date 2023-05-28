#include "fire_Task.h"
#include "bsp_dr16.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "string.h"
#include "can1_receive.h"
#include "can1_send.h"
#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"
#include "gimbal_config.h"
#include "bsp_referee.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

gimbal_fire_control_t fire_control_p;
extern TIM_HandleTypeDef htim1;
static void fire_task_init(gimbal_fire_control_t *fire_init_f);
static void fire_pid_calculate(gimbal_fire_control_t *fire_pid_calculate_f);
static void fire_behaviour_choose(gimbal_fire_control_t *fire_behaviour_choose_f);
int16_t pwm_set = 800;

void fire_Task(void const *argument)
{
    fire_task_init(&fire_control_p);
    while (1)
    {
        fire_behaviour_choose(&fire_control_p); // �������ϵͳ TODO:

        fire_pid_calculate(&fire_control_p);
        //		//can1_gimbal_setmsg_to_fire(0,0,fire_control_p.fire_motor_speed_pid.out);
        can1_gimbal_setmsg_to_fire(fire_control_p.left_motor_speed_pid.out, fire_control_p.right_motor_speed_pid.out, fire_control_p.fire_motor_speed_pid.out);
        vTaskDelay(1);
    }
}
void fire_task_init(gimbal_fire_control_t *fire_init_f)
{
    // ��ò���ָ��
    fire_init_f->right_motor = get_right_motor_measure_point();
    fire_init_f->left_motor = get_left_motor_measure_point();
    fire_init_f->fire_motor = get_fire_motor_measure_point();

    fire_init_f->fire_motor_encoder = Encoder_Init(M2006, 3);

    fire_init_f->fire_rc  = RC_Get_RC_Pointer();
    fire_init_f->referee = Get_referee_Address();
    // fire
    PidInit(&fire_init_f->left_motor_speed_pid, 11, 0, 0, Output_Limit);
    PidInit(&fire_init_f->right_motor_speed_pid, 11, 0, 0, Output_Limit);
    PidInit(&fire_init_f->fire_motor_speed_pid, 10, 0, 0, Output_Limit);
    PidInit(&fire_init_f->fire_motor_position_pid, 1, 0.1, 0, Output_Limit | Integral_Limit);
    PidInitMode(&fire_init_f->left_motor_speed_pid, Output_Limit, 16000, 0);
    PidInitMode(&fire_init_f->right_motor_speed_pid, Output_Limit, 16000, 0);
    PidInitMode(&fire_init_f->fire_motor_speed_pid, Output_Limit, 16000, 0);
    PidInitMode(&fire_init_f->fire_motor_position_pid, Output_Limit, 10000, 0);
    PidInitMode(&fire_init_f->fire_motor_position_pid, Integral_Limit, 100, 0);

    fire_init_f->full_automatic = true;
    fire_init_f->feed_buttle = false;
    fire_init_f->fire_sw = true;
    fire_init_f->replenish_flag = false;

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, OPEN_MAGAZINE);
}
void fire_behaviour_choose(gimbal_fire_control_t *fire_behaviour_choose_f)
{
    static uint16_t last_B = 0;
    static uint16_t last_CTRL = 0;
    static uint16_t last_shift = 0;
    static uint8_t last_ch4 = 0;
    uint16_t last_press;


    static uint16_t last_C = 0;


    // B	����ȫ�Զ��л�
    last_press = last_B;
    last_B = fire_behaviour_choose_f->fire_rc->kb.bit.B;
    if ((last_press == false) && (last_B == true)) // �����ش���
    {
        fire_behaviour_choose_f->full_automatic = !fire_behaviour_choose_f->full_automatic;
    }



    //��������
    last_press = last_CTRL;
    last_CTRL = fire_behaviour_choose_f->fire_rc->kb.bit.CTRL;
    if ((last_press == false) && (last_CTRL == true)) // �����ش���
    {
        if(fire_behaviour_choose_f->replenish_flag)
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, OPEN_MAGAZINE);
        }
        else
        {
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, CLOSE_MAGAZINE);
        }
        fire_behaviour_choose_f->replenish_flag ^= 1;
    }

    //Ħ���ֿ���

//    if((fire_behaviour_choose_f->fire_rc->rc.ch[4] < -500 && last_ch4 == 1) || (last_shift == false && fire_behaviour_choose_f->fire_rc->kb.bit.SHIFT == true))
//    {
//        fire_behaviour_choose_f->fire_sw = !fire_behaviour_choose_f->fire_sw;;
//    }
//    if(fire_behaviour_choose_f->fire_rc->rc.ch[4] > -500)
//    {
//        last_ch4 = 1;
//    }
//    else
//    {
//        last_ch4 = 0;
//    }
//    last_shift = fire_behaviour_choose_f->fire_rc->kb.bit.SHIFT;

    // ����ϵͳ��������
    switch (fire_behaviour_choose_f->referee->Robot_Status.shooter_id1_17mm_speed_limit)
    {
    case 15:
        fire_behaviour_choose_f->left_motor_speed_pid.SetValue = -FIRE_SPEED_15;
        fire_behaviour_choose_f->right_motor_speed_pid.SetValue = FIRE_SPEED_15;
        break;
    case 18:
        fire_behaviour_choose_f->left_motor_speed_pid.SetValue = -FIRE_SPEED_18;
        fire_behaviour_choose_f->right_motor_speed_pid.SetValue = FIRE_SPEED_18;
        break;
    case 22:

        fire_behaviour_choose_f->left_motor_speed_pid.SetValue = -FIRE_SPEED_22;
        fire_behaviour_choose_f->right_motor_speed_pid.SetValue = FIRE_SPEED_22;
        break;
    case 30:
        fire_behaviour_choose_f->left_motor_speed_pid.SetValue = -FIRE_SPEED_30;
        fire_behaviour_choose_f->right_motor_speed_pid.SetValue = FIRE_SPEED_30;
        break;
    default:
        fire_behaviour_choose_f->left_motor_speed_pid.SetValue = -FIRE_SPEED_15;
        fire_behaviour_choose_f->right_motor_speed_pid.SetValue = FIRE_SPEED_15;
        break;
    }

    // ���ն������ TODO:
}

void fire_pid_calculate(gimbal_fire_control_t *fire_pid_calculate_f)
{

    if (((fire_pid_calculate_f->fire_rc->mouse.press_l != 0) || (fire_pid_calculate_f->fire_rc->rc.ch[4] > 0)) && (fire_pid_calculate_f->referee->Power_Heat.shooter_id1_17mm_cooling_heat + 40 <= fire_pid_calculate_f->referee->Robot_Status.shooter_id1_17mm_cooling_limit))
    {
        if (fire_pid_calculate_f->full_automatic) // ȫ�Զ���������
        {
            fire_pid_calculate_f->fire_motor_speed_pid.SetValue = 5000; // TODO:
            fire_pid_calculate_f->fire_motor_speed_pid.out = motor_speed_control(&fire_pid_calculate_f->fire_motor_speed_pid,
                    fire_pid_calculate_f->fire_motor_speed_pid.SetValue,
                    fire_pid_calculate_f->fire_motor->speed);
        }
        else // ��ȫ�Զ�ʹ�ñջ�����
        {
            fire_pid_calculate_f->fire_motor_speed_pid.out = motor_position_speed_control(&fire_pid_calculate_f->fire_motor_speed_pid,
                    &fire_pid_calculate_f->fire_motor_position_pid,
                    (fire_pid_calculate_f->fire_motor_encoder->Encode_Record_Val + 1368),
                    fire_pid_calculate_f->fire_motor_encoder->Encode_Record_Val,
                    fire_pid_calculate_f->fire_motor->speed);
        }
    }

    else
    {
        fire_pid_calculate_f->fire_motor_speed_pid.out = 0;
    }


    if(fire_pid_calculate_f->fire_sw == true)
    {
        fire_pid_calculate_f->left_motor_speed_pid.out = motor_speed_control(&fire_pid_calculate_f->left_motor_speed_pid,
                -fire_pid_calculate_f->left_motor_speed_pid.SetValue,
                fire_pid_calculate_f->left_motor->speed);
        fire_pid_calculate_f->right_motor_speed_pid.out = motor_speed_control(&fire_pid_calculate_f->right_motor_speed_pid,
                -fire_pid_calculate_f->right_motor_speed_pid.SetValue,
                fire_pid_calculate_f->right_motor->speed);

    }
    else
    {
        fire_pid_calculate_f->left_motor_speed_pid.out = 0;
        fire_pid_calculate_f->right_motor_speed_pid.out = 0;
    }
}

const gimbal_fire_control_t *get_fire_control_point(void)
{
    return (const gimbal_fire_control_t *)&fire_control_p;
}
