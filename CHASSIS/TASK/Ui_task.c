#include "Ui_task.h"
#include "chassis_task.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "can2_receive.h"
#include "rm_cilent_ui.h"
#include "user_lib.h"
#include "chassis_config.h"

#define MAX_X 1920
#define MAX_Y 1080


const static char *chassis_mode_ui[5] =
{
    "CHASSIS_FOLLOW",      //跟随
    "CHASSIS_SIDE_FOLLOW",
    "CHASSIS_NO_FOLLOW",   //不跟随
    "CHASSIS_ROTATION",    //小陀螺
    "CHASSIS_SLIP"
};    //炮台模式
const static char *gimbal_mode_ui[4] =
{
    "GIMBAL_MANUAL",		 // 手动状态
    "GIMBAL_AUTOATTACK",	 // 自瞄状态
    "GIMBAL_AUTOBUFF"	 // 打符状态
    "GIMBAL_TOPBUFF"
};

chassis_control_t *Chassis_Control_p = NULL;
gimbal_control_t *Gimbal_Control_p = NULL;
Graph_Data diff_ang, pose, path1, path2, reticle1, reticle2, circle1;

Float_Data CAP_VOLTAGE;
String_Data CH_MODE, GM_MODE, voltage, replenish;

static void ui_init(void);           //ui初始化
static void print_mode(void);        //底盘模式显示
static void print_chassis_gimbal_angle(void);
static void print_cap_voltage(void);
static void print_chassis_gimbal_pose(void);
static void print_chassis_path(void);
static void print_reticle(void);           //标线

/**
  * @brief      ui显示任务
  * @param[in]  *pvParameters
  * @retval     void
  * @attention
  */
void UI_Task(void const *argument)
{
    int count = 0;
    vTaskDelay(10);
    Chassis_Control_p = Get_Chassis_Control_Point();
    Gimbal_Control_p = get_gimbal_control_point();
    ui_init();
    while (1)
    {
        print_cap_voltage();
        print_chassis_gimbal_angle();
        print_mode();
        //print_chassis_gimbal_pose();
        //print_chassis_path();
        count++;
        if(count == 100)
        {
            ui_init();
            count = 0;
        }
        vTaskDelay(100);
    }
}



/**
  * @brief      ui初始化
  * @param[in]  void
  * @retval     void
  * @attention  没有的话，重启一下
  */
static void ui_init(void)
{

    /*--------------------电容ui加入--------------------*/

    /*--------------------底盘云台差角ui加入--------------------*/
    //以中点为圆心，云台方向为前，距离200为半径，画底盘灯条方位
    uint32_t start_angle = Chassis_Control_p->chassis_yaw_lqr.Input[0] * RADIAN_COEF - 45;
    uint32_t end_angle = Chassis_Control_p->chassis_yaw_lqr.Input[0] * RADIAN_COEF + 45;
    start_angle = loop_float_constrain(start_angle, -180.0f, 180.0f);
    end_angle = loop_float_constrain(end_angle, -180.0f, 180.0f);
    Arc_Draw(&diff_ang, "001", UI_Graph_ADD, 5, UI_Color_Main, 1, -1, 5, MAX_X / 2, MAX_Y / 2, 200, 200);
    My_Graph_Refresh(&diff_ang);

    /*--------------------底盘模式ui加入--------------------*/
    Char_Draw(&CH_MODE, "093", UI_Graph_ADD, 5, UI_Color_Main, 25, 5, 80, 880 - 80, chassis_mode_ui[Chassis_Control_p->behaviour]);
    My_Char_Refresh(CH_MODE);

    Char_Draw(&GM_MODE, "094", UI_Graph_ADD, 5, UI_Color_Yellow, 25, 5, 80, 700, gimbal_mode_ui[Gimbal_Control_p->behaviour]);
    My_Char_Refresh(GM_MODE);
    /*--------------------云台补弹ui加入--------------------*/
    if(Gimbal_Control_p->replenish_flag)
        Char_Draw(&replenish, "002", UI_Graph_ADD, 5, UI_Color_Green, 20, 4, 80, 600, "replenish:finish");
    else
        Char_Draw(&replenish, "002", UI_Graph_ADD, 5, UI_Color_Orange, 20, 4, 80, 600, "replenish:ing");
    My_Char_Refresh(replenish);
    /*--------------------云台底盘姿态描述-----------------*/
    //	Line_Draw(&pose, "003", UI_Graph_ADD, 5, UI_Color_Green, 25, 100,100,1000,1000);
    //	My_Graph_Refresh(&pose);
    /*--------------------底盘路线指引-----------------*/
    //    	Line_Draw(&path1, "004", UI_Graph_ADD, 5,UI_Color_Pink, 3, 1072, 543, 1456, 0);
    //    	My_Graph_Refresh(&path1);
    //    	Line_Draw(&path2, "005", UI_Graph_ADD, 5, UI_Color_Pink, 3, 464, 0, 848, 543);
    //    	My_Graph_Refresh(&path2);
    /*--------------------瞄准标线--------------------*/
    Line_Draw(&reticle1, "006", UI_Graph_ADD, 5, UI_Color_Green, 2, 960, 580, 960, 350);
    My_Graph_Refresh(&reticle1);
    Line_Draw(&reticle2, "007", UI_Graph_ADD, 5, UI_Color_Green, 2, 900, 468, 1020, 468);
    My_Graph_Refresh(&reticle2);
    //		 Circle_Draw(&circle1, "008", UI_Graph_ADD, 2, UI_Color_Purplish_red, 4, 960, 540, 30);
    //	  My_Graph_Refresh(&circle1);

}

/**
  * @brief      云台、底盘模式显示
  */
static void print_mode(void)
{
    //底盘
    Char_Draw(&CH_MODE, "093", UI_Graph_Change, 5, UI_Color_Main, 25, 5, 80, 880 - 80, chassis_mode_ui[Chassis_Control_p->behaviour]);
    My_Char_Refresh(CH_MODE);
    Char_Draw(&GM_MODE, "094", UI_Graph_Change, 5, UI_Color_Yellow, 25, 5, 80, 700, gimbal_mode_ui[Gimbal_Control_p->behaviour]);
    My_Char_Refresh(GM_MODE);
    if(Gimbal_Control_p->replenish_flag)
        Char_Draw(&replenish, "002", UI_Graph_Change, 5, UI_Color_Green, 20, 4, 80, 600, "replenish:finish");
    else
        Char_Draw(&replenish, "002", UI_Graph_Change, 5, UI_Color_Orange, 20, 4, 80, 600, "replenish:ing");
    My_Char_Refresh(replenish);

}

/**
  * @brief      电容电压显示
  */
static void print_cap_voltage(void)
{
    //    CAP_VOLTAGE.graph_Float = (int)(Chassis_Control_p->super_cap_c->Capacitance_voltage * 1000);
    //    CAP_VOLTAGE.operate_tpye = UI_Graph_Change;
    //    My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);
}

/**
  * @brief      地盘云台角度显示
  */
static void print_chassis_gimbal_angle(void)
{
    float chassis_gimbal_angle = -((Chassis_Control_p->yaw_motor->position - YAW_ZERO_OFFSET) / 8192.0f * 360.0f );
    int32_t start_angle = chassis_gimbal_angle - 20;
    int32_t end_angle = chassis_gimbal_angle + 20;
    start_angle = loop_float_constrain(start_angle, 0.0f, 360.0f);
    end_angle = loop_float_constrain(end_angle, 0.0f, 360.0f);


    Arc_Draw(&diff_ang, "001", UI_Graph_Change, 5, UI_Color_Main, (uint32_t)start_angle, (uint32_t)end_angle, 5, MAX_X / 2, MAX_Y / 2, 200, 200);
    My_Graph_Refresh(&diff_ang);
}

static void print_chassis_gimbal_pose(void)
{
    Line_Draw(&pose, "003", UI_Graph_Change, 5, UI_Color_Green, 5, 1072, 440, 1386, 0);
    My_Graph_Refresh(&pose);
}

static void print_chassis_path(void)
{
    Line_Draw(&path1, "004", UI_Graph_Change, 5, UI_Color_Pink, 3, 1072, 543, 1456, 0);
    My_Graph_Refresh(&path1);
    Line_Draw(&path2, "005", UI_Graph_Change, 5, UI_Color_Pink, 3, 464, 0, 848, 543);
    My_Graph_Refresh(&path2);
}

/**
  * @brief      瞄准标线
  */
static void print_reticle(void)
{
    Line_Draw(&reticle1, "006", UI_Graph_Change, 5, UI_Color_Green, 2, 960, 580, 960, 350);
    My_Graph_Refresh(&reticle1);
    Line_Draw(&reticle2, "007", UI_Graph_Change, 5, UI_Color_Green, 2, 900, 468, 1020, 468);
    My_Graph_Refresh(&reticle2);
    //	  Circle_Draw(&circle1, "008", UI_Graph_Change, 2, UI_Color_Purplish_red, 4, 960, 540, 30);
    //	  My_Graph_Refresh(&circle1);
}