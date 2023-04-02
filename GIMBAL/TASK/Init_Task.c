#include "Init_Task.h"
#include "cmsis_os.h"
#include "cmsis_armcc.h"

/************************* hardware ************************/
#include "can1_receive.h"
#include "can2_receive.h"

/************************* Task ************************/
#include "Task_Safe.h"
#include "ins_task.h"
#include "gimbal_task.h"
#include "virtual_task.h"
#include "fire_task.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* ************************bsp******************** */
#include "spi.h"
#include "bsp_dwt.h"
#include "bsp_dr16.h"


extern osThreadId Init_TASKHandle;
extern osThreadId defaultTaskHandle;

osThreadId Safe_TASKHandle;
osThreadId UI_TASKHandle;
osThreadId TASK_GIMBALHandle;
osThreadId ShootTask_Handler;
osThreadId INS_TASKHandle;
osThreadId Virtual_TASKHandle;


void Init_Task(void const *argument)
{
	taskENTER_CRITICAL(); //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���

	// CAN�˲�����ʼ��
	CAN1_filter_config();
	CAN2_filter_config();


	//ң������ʼ��
	ECF_RC_Init();

	//����ϵͳ
	//referee_system_init();

	//������ȫ����
	osThreadDef(Safe_TASK, Safe_Task, osPriorityNormal, 0, 128);
	Safe_TASKHandle = osThreadCreate(osThread(Safe_TASK), NULL);

	//������̨����
	osThreadDef(GIMBAL_TASK, Gimbal_Task, osPriorityHigh, 0, 256);
	TASK_GIMBALHandle = osThreadCreate(osThread(GIMBAL_TASK), NULL);

	//�����Ӿ�����
	osThreadDef(Virtual_TASK, Virtual_Task, osPriorityAboveNormal , 0, 128);
	Virtual_TASKHandle = osThreadCreate(osThread(Virtual_TASK), NULL);
	
	//����INS����
	osThreadDef(INS_TASK, INS_Task, osPriorityNormal, 0, 1024);
	INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

#ifdef FIRE_WORK //����
	//�����������
	osThreadDef(SHOOT_TASK, fire_Task, osPriorityAboveNormal, 0, 128);
	ShootTask_Handler = osThreadCreate(osThread(SHOOT_TASK), NULL);

#endif

	vTaskDelete(Init_TASKHandle); //ɾ����ʼ����
	vTaskDelete(defaultTaskHandle);
	taskEXIT_CRITICAL();		  //�˳��ٽ���
}

/**
 * @brief      ʧ�ش���
 * @param[in]  none
 * @retval     none
 * @attention
 */
void out_of_control(void)
{
	//���������
	vTaskSuspend(TASK_GIMBALHandle);
	vTaskSuspend(ShootTask_Handler);

	//���ʧ�ر�����������
	//vTaskResume(OutOf_Control_THandle);
}

/**
 * @brief      ���� | ���ʧ��
 * @param[in]  none
 * @retval     none
 * @attention
 */
void normal_control(void)
{
	//�������
	vTaskResume(TASK_GIMBALHandle);
	vTaskResume(ShootTask_Handler);

	//ʧ�ر������������������
	//    vTaskSuspend(OutOf_Control_THandle);
}
