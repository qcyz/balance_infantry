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
	taskENTER_CRITICAL(); //为了保证对PORTA寄存器的访问不被中断，将访问操作放入临界区。进入临界区

	// CAN滤波器初始化
	CAN1_filter_config();
	CAN2_filter_config();


	//遥控器初始化
	ECF_RC_Init();

	//裁判系统
	//referee_system_init();

	//创建安全任务
	osThreadDef(Safe_TASK, Safe_Task, osPriorityNormal, 0, 128);
	Safe_TASKHandle = osThreadCreate(osThread(Safe_TASK), NULL);

	//创建云台任务
	osThreadDef(GIMBAL_TASK, Gimbal_Task, osPriorityHigh, 0, 256);
	TASK_GIMBALHandle = osThreadCreate(osThread(GIMBAL_TASK), NULL);

	//创建视觉任务
	osThreadDef(Virtual_TASK, Virtual_Task, osPriorityAboveNormal , 0, 128);
	Virtual_TASKHandle = osThreadCreate(osThread(Virtual_TASK), NULL);
	
	//创建INS任务
	osThreadDef(INS_TASK, INS_Task, osPriorityNormal, 0, 1024);
	INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

#ifdef FIRE_WORK //火力
//	//创建火控任务
	osThreadDef(SHOOT_TASK, fire_Task, osPriorityAboveNormal, 0, 128);
	ShootTask_Handler = osThreadCreate(osThread(SHOOT_TASK), NULL);

#endif

	vTaskDelete(Init_TASKHandle); //删除开始任务
	vTaskDelete(defaultTaskHandle);
	taskEXIT_CRITICAL();		  //退出临界区
}

/**
 * @brief      失控处理
 * @param[in]  none
 * @retval     none
 * @attention
 */
void out_of_control(void)
{
	//将任务挂起
	vTaskSuspend(TASK_GIMBALHandle);
	vTaskSuspend(ShootTask_Handler);

	//解挂失控保护控制任务
	//vTaskResume(OutOf_Control_THandle);
}

/**
 * @brief      正常 | 解除失控
 * @param[in]  none
 * @retval     none
 * @attention
 */
void normal_control(void)
{
	//解挂任务
	vTaskResume(TASK_GIMBALHandle);
	vTaskResume(ShootTask_Handler);

	//失控保护控制任务任务挂起
	//    vTaskSuspend(OutOf_Control_THandle);
}
