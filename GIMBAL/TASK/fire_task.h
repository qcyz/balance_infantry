#ifndef __FIRE_TASK_H
#define __FIRE_TASK_H
#include "gimbal_struct_variables.h"
void fire_Task(void const *argument);
const gimbal_fire_control_t *get_fire_control_point(void);

#define FIRE_SPEED_15 4400
#define FIRE_SPEED_18 4800
#define FIRE_SPEED_22 5450
#define FIRE_SPEED_30 7040 //

#define OPEN_MAGAZINE 	400
#define CLOSE_MAGAZINE 	1050
#endif
