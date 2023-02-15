#include "can1_receive.h"
#include "can2_receive.h"
#include "LK9025_communication.h"
#include "chassis_struct_variables.h"
#include "bsp_Motor_Encoder.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern void chassis_can2_callback(CAN_HandleTypeDef *hcan);

/*--------------------����-----------------------*/
//�������ݱ���
static supercapacitor_receive_t Supercap_receive;

//����
supercapacitor_receive_t *get_supercap_control_point(void)
{
	return &Supercap_receive;
}



/**
 * @brief		CAN1�˲�������
 * @param		none
 *	@retval		none
 */
void CAN1_filter_config(void)
{
	CAN_FilterTypeDef CAN1_FIilter_InitStruct;

	CAN1_FIilter_InitStruct.FilterActivation = ENABLE;			 //�����˲���
	CAN1_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;	 //����ģʽ
	CAN1_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32λ����
	CAN1_FIilter_InitStruct.FilterIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterBank = 0;
	//	CAN1_FIilter_InitStruct.SlaveStartFilterBank = 14;
	CAN1_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0;	   //ָ����������
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_FIilter_InitStruct);			   //����ָ������CAN���չ�����
	HAL_CAN_Start(&hcan1);											   //����can1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //�����ж�
}

/**
 * @brief		HAl��can1�Ļص�����
 * @param		��������� CAN�ľ��
 * @retval   none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{
//		chassis_can1_callback(hcan);
		LK9025_can1_callback(hcan);
	}
	if (hcan == &hcan2)
	{
		chassis_can2_callback(hcan);
	}
}

/*************************************can1����*************************************/
/**
 * @brief      ���̰�can1���ջص�����
 * @param[in]  *rx_message: can1���սṹ��
 * @retval     none
 * @attention  �� chassis_app.h ��ע�ᣬ�жϵ��ã�can1_callback(&rx1_message);��
 */
void chassis_can1_callback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef Rxmessage; //������Ϣ�ṹ��
	uint8_t Rx_Data[8];			   //���յ���Ϣ���������

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Rxmessage, Rx_Data) == HAL_OK) //��ȡ���յ���Ϣ
	{
		switch (Rxmessage.StdId)
		{
		/*���̵��*/
		case 0x211: //�������ݽ���
		{
			Supercap_receive.input_voltage = (float)(Rx_Data[0] / 100.0f);
			Supercap_receive.Capacitance_voltage = (float)(Rx_Data[1] / 100.0f);
			Supercap_receive.Input_current = (float)(Rx_Data[2] / 100.0f);
			Supercap_receive.Set_power = (float)(Rx_Data[3] / 100.0f);
			break;
		}
		default:
		{
			break;
		}
		}
	}
}

/**
 * @brief      ���س������ݵ�ѹ
 * @param[in]  none
 * @retval     �������ݵ�ѹ
 * @attention  ���̰����
 */
float re_capacitance_voltage(void)
{
	return (Supercap_receive.Capacitance_voltage);
}