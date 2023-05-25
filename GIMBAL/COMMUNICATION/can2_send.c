#include "can2_send.h"
#include "can.h"

static CAN_TxHeaderTypeDef Txmessage; //发送的信息
extern gimbal_control_t Gimbal_Control;

void can2_gimbal_setmsg_to_yaw(int16_t yaw)
{
    uint32_t send_mail_box;
    uint8_t Data[8]; //发送数据的数组

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

    HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data, &send_mail_box);
}
/**
 * @brief		云台发送角度给底盘
 * @param		none
 *	@retval		none
 */
bool see_flag = 0;
void can2_gimbal_to_chassis(uint16_t position, int8_t behaviour, bool replenish_flag)
{
	uint32_t send_mail_box; //发送邮箱
	uint8_t Data_Tx[8];		//发送数据的数组
	int Register; //防止汇编那警告的
	
	Txmessage.StdId = 0x300;	  //根据820r设置标识符
	Txmessage.IDE = CAN_ID_STD;	  //指定将要传输的消息的标识符的类型
	Txmessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
	Txmessage.DLC = 8;
	see_flag = replenish_flag;
	Data_Tx[0] = position >> 8;
    Data_Tx[1] = position;
	Data_Tx[2] = behaviour;
    Data_Tx[3] = replenish_flag;
	Data_Tx[4] = 0;
	Data_Tx[5] = 0;
	Data_Tx[6] = 0;
	Data_Tx[7] = 0;
	
	HAL_CAN_AddTxMessage(&hcan2, &Txmessage, Data_Tx, &send_mail_box); //将一段数据通过 CAN 总线发送
}
