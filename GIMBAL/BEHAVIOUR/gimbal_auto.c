/*--------------------- CONTROL --------------------*/
#include "gimbal_auto.h"
#include "gimbal_struct_variables.h"

/*--------------------- FIRMWARE --------------------*/
#include "usbd_cdc_if.h"


/**
  * @brief      �������ݸ��Ӿ�
  * @param[in]  data: �з�װ�װ�  �죺0 | ����1
  * @param[in]  mode: ģʽѡ��    0��װ��ģʽ  1����������ģʽ  2������ģʽ
  * @param[in]  shoot_speed: ���٣�Ŀǰ��ƽ��ʵ���ٶȣ���ʵ���ǵ�ǰ��������-2
  * @retval     none
  * @attention  Э�飺֡ͷ0xFF  ����  ֡β0xFE
  */
void visual_send_data(uint8_t data, uint8_t mode, uint8_t shoot_speed,float yaw,float pitch)
{
    uint8_t SendBuff[29];
	
    SendBuff[0] = 0xFF;
    SendBuff[1] = data;
    SendBuff[2] = mode;
    SendBuff[3] = shoot_speed;
		
    SendBuff[28] = 0xFE;
	CDC_Transmit_FS(SendBuff, sizeof(SendBuff));
}

