#include "can1_send.h"
#include "gimbal_struct_variables.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //发送的信息
int16_t see_pitch_output = 0;
void can1_gimbal_setmsg_to_pitch(int16_t pitch)
{
    uint32_t send_mail_box;
    uint8_t Data[8]; //发送数据的数组
	uint16_t i = 0;	
	see_pitch_output = pitch;
    Txmessage.StdId = 0x1FF;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = 0x08;
    Data[0] = 0;
    Data[1] = 0;
    Data[2] = pitch >> 8;
    Data[3] = pitch;
    Data[4] = 0;
    Data[5] = 0;
    Data[6] = 0;
    Data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box);
	while((HAL_CAN_IsTxMessagePending(&hcan1, send_mail_box) == SET) && (i < 0xfff))i++;
}
int16_t see_yaw_output = 0;
void can1_gimbal_setmsg_to_yaw(int16_t yaw)
{
    uint32_t send_mail_box;
    uint8_t Data[8]; //发送数据的数组
	uint16_t i = 0;	

	see_yaw_output = yaw;
    Txmessage.StdId = 0x1FF;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = 0x08;
    Data[0] = yaw >> 8;
    Data[1] = yaw;
    Data[2] = 0;
    Data[3] = 0;
    Data[4] = 0;
    Data[5] = 0;
    Data[6] = 0;
    Data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box);
	while((HAL_CAN_IsTxMessagePending(&hcan1, send_mail_box) == SET) && (i < 0xfff))i++;
}
void can1_gimbal_setmsg_to_motor(int16_t yaw, int16_t pitch)
{
    uint32_t send_mail_box;
    uint8_t Data[8]; //发送数据的数组
	uint16_t i = 0;	

	see_yaw_output = yaw;
    Txmessage.StdId = 0x1FF;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = 0x08;
    Data[0] = yaw >> 8;
    Data[1] = yaw;
    Data[2] = pitch >> 8;
    Data[3] = pitch;
    Data[4] = 0;
    Data[5] = 0;
    Data[6] = 0;
    Data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box);
	while((HAL_CAN_IsTxMessagePending(&hcan1, send_mail_box) == SET) && (i < 0xfff))i++;
}


void can1_gimbal_setmsg_to_fire(int16_t left, int16_t right, int16_t fire)
{
    uint32_t send_mail_box;
		uint8_t Data[8]; //发送数据的数组
	
    Txmessage.StdId = 0x200;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = 0x08;
    Data[0] = left >> 8;
    Data[1] = left;
    Data[2] = right >> 8;
    Data[3] = right;
    Data[4] = fire >> 8;
    Data[5] = fire;
    Data[6] = 0;
    Data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box);
}
