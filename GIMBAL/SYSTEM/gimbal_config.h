/*code is far away from bug with the animal protecting
 *  ��������������
 *�����ߩ��������ߩ�
 *������������������ ��
 *������������������
 *�����ש������ס���
 *������������������
 *���������ߡ�������
 *������������������
 *������������������
 *��������������PC��BJ����
 *��������������������BUG��
 *����������������������
 *���������������������ǩ�
 *������������������������
 *���������������ש�����
 *���������ϩϡ����ϩ�
 *���������ߩ������ߩ�
 *������
 */

// #include "gimbal_config.h"
#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#define FIRE_WORK
#define PITCH_ZERO_OFFSET 100.0f 
#define YAW_ZERO_OFFSET 6300
/**********************pitch��PID����**********************/
#define GIMBAL_PITCH_P 0.0f
#define GIMBAL_PITCH_I 8.0f
#define GIMBAL_PITCH_D 0.0f

/**********************Yaw��PID����**********************/
#define GIMBAL_YAW_P 0.0f
#define GIMBAL_YAW_I 8.0f
#define GIMBAL_YAW_D 0.0f


/**********************��̨pitch�Ƕ�����**********************/
#define PITCH_ANGLE_LIMIT_UP 32.0f
#define PITCH_ANGLE_LIMIT_DOWN -24.0f

/**********************�������ң���ٶ�����**********************/
#define MOUSE_YAW_SPEED 0.010f   //���yaw���ٶ�����
#define MOUSE_PITCH_SPEED 0.006f //���pitch���ٶ�����
#define RC_YAW_SPEED 0.0005f     //ң����yaw���ٶ�����
#define RC_PITCH_SPEED 0.0005f   //ң����pitch���ٶ�����

#endif
