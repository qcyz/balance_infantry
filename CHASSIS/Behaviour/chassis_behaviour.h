#ifndef __CHASSIS_BEHAVIOUR_H
#define __CHASSIS_BEHAVIOUR_H

#include "chassis_struct_variables.h"

#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif


#ifndef BARYCENTER_ZERO_OFFSET
#define BARYCENTER_ZERO_OFFSET	4.0f //ÖØÐÄÆ«ÒÆÖµ
#endif

#define CHASSIS_X_SEN	2.0f		// m /s
#define CHASSIS_YAW_SEN 180.0f  	// ¡ã/s

void chassis_behaviour_react(chassis_control_t *Chassis_behaviour_react_f);
void chassis_behaviour_choose(chassis_control_t *Chassis_behaviour_f);
void chassis_speed_pid_calculate(chassis_control_t *chassis_speed_pid_calculate_f);
void chassis_state_choose(chassis_control_t *chassis_state_choose_f);
void motor_lqr_calculate(chassis_control_t *lqr_calculate_f);

chassis_behaviour_e *get_chassis_behaviour_point(void);
fp32 *get_chassis_x_point(void);
fp32 *get_chassis_yaw_point(void);

#endif
