#ifndef LK9025_COMMUNICATION_H
#define LK9025_COMMUNICATION_H

#include "chassis_struct_variables.h"

typedef enum
{
    Read_PID = 0x30,
    PID_To_RAM = 0x31,
    PID_To_ROM = 0x32,
    Read_Accel = 0x33,
    Write_Accel = 0x34,
    Read_Encoder = 0x90,
    Write_Encoder_Offset_To_ROM = 0x91,
    Write_Now_Encoder_As_Zero_To_ROM = 0x19,
    Read_Muli_Turns = 0x92,
    Read_Single_Turns = 0x94,
    Delect_Turns = 0x95,
    Read_State_And_Error_1 = 0x9A,
    Delect_Error_State = 0x98,
    Read_State_2 = 0x9C,
    Read_State_3 = 0x9D,
    Motor_Close = 0x80,
    Motor_Stop = 0x81,
    Motor_Stop_To_Run = 0x88,
    Torque_Open_Loop_Control = 0xA0,
    Torque_Closed_Loop_Control = 0xA1,
    Speed_Closed_Loop_Control = 0xA2,	
    Position_Closed_Loop_Control_1 = 0xA3,	
    Position_Closed_Loop_Control_2 = 0xA4,	
    Position_Closed_Loop_Control_3 = 0xA5,	
    Position_Closed_Loop_Control_4 = 0xA6,	
    Position_Closed_Loop_Control_5 = 0xA7,	
    Position_Closed_Loop_Control_6 = 0xA8,	
    
    Mul_Motor_Torque_Control = 0x280 
  
} Command_9025_ID;

supercapacitor_receive_t *get_supercap_control_point(void);


void chassis_motor_9025_send(int16_t motor_l_set, int16_t motor_r_set);
void chassis_motor_read_turns(void);

motor_9025_t *get_chassis_motor_l_point(void);
motor_9025_t *get_chassis_motor_r_point(void);

void LK9025_can1_callback(CAN_HandleTypeDef *hcan);


#endif
