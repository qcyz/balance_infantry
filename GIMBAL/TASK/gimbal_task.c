/*--------------------- FIRMWARE --------------------*/
#include "string.h"
#include "usbd_cdc_if.h"

/*--------------------- TASK --------------------*/
#include "gimbal_task.h"
#include "fire_Task.h"
#include "virtual_task.h"
/*--------------------- COMMUINICATE --------------------*/
#include "can1_receive.h"
#include "can2_receive.h"
#include "can1_send.h"
#include "can2_send.h"

/*--------------------- CONTROL --------------------*/
#include "gimbal_behaviour.h"
#include "gimbal_struct_variables.h"
#include "gimbal_auto.h"
#include "gimbal_config.h"

/*--------------------- ALGORITHM --------------------*/
#include "pid.h"
#include "maths.h"
#include "lqr.h"
#include "filter.h"
#include "user_lib.h"

/*--------------------- BSP --------------------*/
#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"


gimbal_control_t Gimbal_Control;

static void Gimbal_Work(gimbal_control_t *Gimbal_Work_f);
static void Gimbal_Init(gimbal_control_t *Gimbal_Init_f);

double k_yaw_lqr[2] = {-7.96227766016838, -0.23467065322830};
double k_pitch_lqr[2] = {-5.94183025734807, -0.3};



/**
 * @brief  ��̨������
 * @param
 * @retval void
 */


float see_k1;
float see_k2;


void Gimbal_Task(void const *argument)
{

    Gimbal_Init(&Gimbal_Control);
    while (1)
    {
		
			
		taskENTER_CRITICAL();         		// �����ٽ���
		Gimbal_Work(&Gimbal_Control); 		// ��̨״̬����    //��̨����
		taskEXIT_CRITICAL();              	// �˳��ٽ���
		
		can1_gimbal_setmsg_to_motor(Gimbal_Control.Yaw_c.motor_output, Gimbal_Control.Pitch_c.motor_output);
			
		can2_gimbal_to_chassis(Gimbal_Control.Yaw_c.motor_measure->position);
		vTaskDelay(1); // ������ʱ//vTaskDelay(2);
    }
}

void Gimbal_Init(gimbal_control_t *Gimbal_Init_f)
{
    memset(Gimbal_Init_f, 0, sizeof(gimbal_control_t));
    /*--------------------��ȡָ��--------------------*/
    // ��ȡң����ָ��(����)
    Gimbal_Init_f->Gimbal_RC = RC_Get_RC_Pointer();
	
	// ��ȡ������ָ��
    Gimbal_Init_f->Gimbal_INS = Get_INS_Point();
	
    // ��ȡ��̨���ָ��
    Gimbal_Init_f->Pitch_c.motor_measure = get_pitch_motor_measure_point();
    Gimbal_Init_f->Yaw_c.motor_measure = get_yaw_motor_measure_point();
	
   	
	//��ȡ����ָ��
	Gimbal_Init_f->auto_c = get_auto_control_point();
	

    /*--------------------��ʼ��������--------------------*/
    Gimbal_Init_f->Pitch_c.motor_encoder = Encoder_Init(GM6020, 1);
    Gimbal_Init_f->Yaw_c.motor_encoder = Encoder_Init(GM6020, 2);


    /*--------------------��ʼ��lqr--------------------*/
    LQR_Init(&Gimbal_Init_f->Pitch_c.motor_lqr, 2, 1, k_pitch_lqr);
	LQR_Init(&Gimbal_Init_f->Yaw_c.motor_lqr, 2, 1, k_yaw_lqr);
   
   
   /*--------------------��ʼ��pid--------------------*/
	PidInit(&Gimbal_Init_f->Pitch_c.motor_pid, GIMBAL_PITCH_P, GIMBAL_PITCH_I, GIMBAL_PITCH_D, Integral_Limit | Separated_Integral);
    PidInitMode(&Gimbal_Init_f->Pitch_c.motor_pid, Separated_Integral, 5.0f, -5.0f);
    PidInitMode(&Gimbal_Init_f->Pitch_c.motor_pid, Integral_Limit, 1000, 0);
	
	PidInit(&Gimbal_Init_f->Yaw_c.motor_pid, GIMBAL_YAW_P, GIMBAL_YAW_I, GIMBAL_YAW_D, Integral_Limit | Separated_Integral);
    PidInitMode(&Gimbal_Init_f->Yaw_c.motor_pid, Separated_Integral, 10.0f, -10.0f);
    PidInitMode(&Gimbal_Init_f->Yaw_c.motor_pid, Integral_Limit, 600, 0);
   
   /*------------��ʼ��yaw����pitch����˲�------------*/
	sliding_average_filter_init(&Gimbal_Init_f->Pitch_c.motor_filter, 10);
	sliding_average_filter_init(&Gimbal_Init_f->Yaw_c.motor_filter, 10);
   
	Gimbal_Init_f->gimbal_state = GIMBAL_ZERO_FORCE;
	Gimbal_Init_f->gimbal_behaviour = GIMBAL_MANUAL;
   
    EncoderValZero(Gimbal_Init_f->Yaw_c.motor_encoder);
    Gimbal_Init_f->chassis_gimbal_angel = 0;
}

void Gimbal_Work(gimbal_control_t *Gimbal_Work_f)
{
    gimbal_behaviour_choose(Gimbal_Work_f);

    gimbal_behaviour_react(Gimbal_Work_f);

	gimbal_motor_calculate(&Gimbal_Control);

}

gimbal_behaviour_e *get_gimbal_behaviour_point(void)
{
    return &Gimbal_Control.gimbal_behaviour;
}
