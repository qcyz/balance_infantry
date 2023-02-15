#include "LK9025_communication.h"
#include "can.h"

/* private variable */
static CAN_TxHeaderTypeDef Txmessage;		

motor_9025_t motor_9025_l;
motor_9025_t motor_9025_r;


/* private funtion */
void write_torque_closed_loop_control(uint16_t motor_id, int16_t iq_control_set);
void read_muli_turns(uint16_t motor_id);







void chassis_motor_9025_send(int16_t motor_l_set, int16_t motor_r_set)
{
	write_torque_closed_loop_control(CHASSIS_MOTOR_L, motor_l_set);
	write_torque_closed_loop_control(CHASSIS_MOTOR_R, motor_r_set);	
}

void chassis_motor_read_turns(void)
{
	read_muli_turns(CHASSIS_MOTOR_L);
	read_muli_turns(CHASSIS_MOTOR_R);	
}




void write_torque_closed_loop_control(uint16_t motor_id, int16_t iq_control_set)
{
    uint32_t send_mail_box;				
    uint8_t  Data[8];						
    uint16_t i=0;
	  
    Txmessage.StdId = motor_id;                 
    Txmessage.IDE = CAN_ID_STD;			
    Txmessage.RTR = CAN_RTR_DATA;			   
    Txmessage.DLC = 8;
    Txmessage.TransmitGlobalTime = DISABLE;
    
    Data[0] = Torque_Closed_Loop_Control;
    Data[1] = 0;
    Data[2] = 0;
    Data[3] = 0;
    Data[4] = *(uint8_t *)(&iq_control_set);
    Data[5] = *((uint8_t *)(&iq_control_set) + 1);
    Data[6] = 0;
    Data[7] = 0;
    
    HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box);
	  	
    while((HAL_CAN_IsTxMessagePending(&hcan1, send_mail_box) == SET) && (i < 0xfff))i++;   


}
void read_muli_turns(uint16_t motor_id)
{
    uint32_t send_mail_box;				
    uint8_t  Data[8];						
    uint16_t i = 0;	
	  
    Txmessage.StdId = motor_id;                 
    Txmessage.IDE = CAN_ID_STD;			
    Txmessage.RTR = CAN_RTR_DATA;			   
    Txmessage.DLC = 8;
    Txmessage.TransmitGlobalTime = DISABLE;
    
    Data[0] = Read_Muli_Turns;
    Data[1] = 0;
    Data[2] = 0;
    Data[3] = 0;
    Data[4] = 0;
    Data[5] = 0;
    Data[6] = 0;
    Data[7] = 0;
    
    HAL_CAN_AddTxMessage(&hcan1, &Txmessage, Data, &send_mail_box);
	  
    while((HAL_CAN_IsTxMessagePending(&hcan1, send_mail_box) == SET) && (i < 0xfff))i++;
}
/**
 * @brief      LK9025回调函数
 * @param[in]  *rx_message: can1接收结构体
 * @retval     none
 * @attention  
 */
void LK9025_can1_callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef Rxmessage; //接收信息结构体
    uint8_t Rx_Data[8];            //接收的信息缓存的数组
    
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Rxmessage, Rx_Data) == HAL_OK) //读取接收的信息
    {
		motor_9025_t *rx_motor;
		if((Rxmessage.StdId - CHASSIS_MOTOR_L) == 0)  
			rx_motor = get_chassis_motor_l_point();
		else if((Rxmessage.StdId - CHASSIS_MOTOR_R) == 0) 
			rx_motor = get_chassis_motor_r_point();
		else return;
		switch (Rx_Data[0])
		{
			case Read_PID:
		    {
	            rx_motor->Angle_Pid_Kp = Rx_Data[2];
		        rx_motor->Angle_Pid_Ki= Rx_Data[3];
		        rx_motor->Speed_Pid_Kp = Rx_Data[4];
		        rx_motor->Speed_Pid_Ki = Rx_Data[5];
		        rx_motor->Torqur_Pid_Kp = Rx_Data[6];
		        rx_motor->Torqur_Pid_Ki = Rx_Data[7];
		        break;
		    }
			case Read_Muli_Turns:
		    {
		        int64_t angle = Rx_Data[1] | (int64_t)Rx_Data[2] << (8 * 1) | \
		    	  			  (int64_t)Rx_Data[3] << (8 * 2) | (int64_t)Rx_Data[4] << (8 * 3) | \
		    	  			  (int64_t)Rx_Data[5] << (8 * 4) | (int64_t)Rx_Data[6] << (8 * 5) |  \
		    	  			  (int64_t)Rx_Data[7] << (8 * 6);
		        angle = (int64_t)(angle << 8) / 256;
		        rx_motor->Muli_Angle = angle / 100;
		        break;
		    }
			case Torque_Closed_Loop_Control:
			{
			    rx_motor->Last_Speed = rx_motor->Speed;	
			    	
			    rx_motor->Temperature = Rx_Data[1];
			    rx_motor->Torque_Current = Rx_Data[3]<<8 | Rx_Data[2];
			    rx_motor->Speed = Rx_Data[5] << 8 | Rx_Data[4];
			    rx_motor->Encoder = Rx_Data[7] << 8 | Rx_Data[6];
			    break;
			}
			default :
			break;
		}
    }
}


//左轮电机
motor_9025_t *get_chassis_motor_l_point(void)
{
	return &motor_9025_l;
}

//左轮电机
motor_9025_t *get_chassis_motor_r_point(void)
{
	return &motor_9025_r;
}


