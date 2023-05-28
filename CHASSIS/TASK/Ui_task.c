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
    "CHASSIS_FOLLOW",      //����
	"CHASSIS_SIDE_FOLLOW",
    "CHASSIS_NO_FOLLOW",   //������
    "CHASSIS_ROTATION",    //С����
    "CHASSIS_SLIP"
};    //��̨ģʽ
const static char *gimbal_mode_ui[3] =
{
    "GIMBAL_MANUAL",		 // �ֶ�״̬
    "GIMBAL_AUTOATTACK",	 // ����״̬
    "GIMBAL_AUTOBUFF"	 // ���״̬
};

chassis_control_t *Chassis_Control_p = NULL;
gimbal_control_t *Gimbal_Control_p = NULL;
Graph_Data diff_ang, pose, path1, path2, reticle1, reticle2,circle1;

Float_Data CAP_VOLTAGE;
String_Data CH_MODE, GM_MODE, voltage, replenish;

static void ui_init(void);           //ui��ʼ��
static void print_mode(void);        //����ģʽ��ʾ
static void print_chassis_gimbal_angle(void);
static void print_cap_voltage(void);
static void print_chassis_gimbal_pose(void);
static void print_chassis_path(void);
static void print_reticle(void);           //����

/**
  * @brief      ui��ʾ����
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
  * @brief      ui��ʼ��
  * @param[in]  void
  * @retval     void
  * @attention  û�еĻ�������һ��
  */
static void ui_init(void)
{

    /*--------------------����ui����--------------------*/

    /*--------------------������̨���ui����--------------------*/
    //���е�ΪԲ�ģ���̨����Ϊǰ������200Ϊ�뾶�������̵�����λ
    uint32_t start_angle = Chassis_Control_p->chassis_yaw_lqr.Input[0] * RADIAN_COEF - 45;
    uint32_t end_angle = Chassis_Control_p->chassis_yaw_lqr.Input[0] * RADIAN_COEF + 45;
    start_angle = loop_float_constrain(start_angle, -180.0f, 180.0f);
    end_angle = loop_float_constrain(end_angle, -180.0f, 180.0f);
    Arc_Draw(&diff_ang, "001", UI_Graph_ADD, 5, UI_Color_Main, 1, -1, 5, MAX_X / 2, MAX_Y / 2, 200, 200);
    My_Graph_Refresh(&diff_ang);

    /*--------------------����ģʽui����--------------------*/
    Char_Draw(&CH_MODE, "093", UI_Graph_ADD, 8, UI_Color_Main, 25, 5, 80, 880 - 80, chassis_mode_ui[Chassis_Control_p->behaviour]);
    My_Char_Refresh(CH_MODE);

    Char_Draw(&GM_MODE, "094", UI_Graph_ADD, 7, UI_Color_Yellow, 25, 5, 80, 700, gimbal_mode_ui[Gimbal_Control_p->behaviour]);
    My_Char_Refresh(GM_MODE);
    /*--------------------��̨����ui����--------------------*/
    if(Gimbal_Control_p->replenish_flag)
        Char_Draw(&replenish, "002", UI_Graph_ADD, 6, UI_Color_Green, 20, 4, 80, 600, "replenish:ing");
    else
        Char_Draw(&replenish, "002", UI_Graph_ADD, 6, UI_Color_Orange, 20, 4, 80, 600, "replenish:finish");
    My_Char_Refresh(replenish);
    /*--------------------��̨������̬����-----------------*/
    //	Line_Draw(&pose, "003", UI_Graph_ADD, 5, UI_Color_Green, 25, 100,100,1000,1000);
    //	My_Graph_Refresh(&pose);
    /*--------------------����·��ָ��-----------------*/
//    	Line_Draw(&path1, "004", UI_Graph_ADD, 5,UI_Color_Pink, 3, 1072, 543, 1456, 0);
//    	My_Graph_Refresh(&path1);
//    	Line_Draw(&path2, "005", UI_Graph_ADD, 5, UI_Color_Pink, 3, 464, 0, 848, 543);
//    	My_Graph_Refresh(&path2);
    /*--------------------��׼����--------------------*/
    Line_Draw(&reticle1, "006", UI_Graph_ADD, 5, UI_Color_Green, 3, 960, 580, 960, 350);
    My_Graph_Refresh(&reticle1);
    Line_Draw(&reticle2, "007", UI_Graph_ADD, 5, UI_Color_Green, 3, 900, 468, 1020, 468);
    My_Graph_Refresh(&reticle2);
		 Circle_Draw(&circle1, "008", UI_Graph_ADD, 2, UI_Color_Purplish_red, 4, 960, 540, 30);
	  My_Graph_Refresh(&circle1);

}

/**
  * @brief      ��̨������ģʽ��ʾ
  */
static void print_mode(void)
{
    //����
    Char_Draw(&CH_MODE, "093", UI_Graph_Change, 8, UI_Color_Main, 25, 5, 80, 880 - 80, chassis_mode_ui[Chassis_Control_p->behaviour]);
    My_Char_Refresh(CH_MODE);
    Char_Draw(&GM_MODE, "094", UI_Graph_Change, 7, UI_Color_Yellow, 25, 5, 80, 700, gimbal_mode_ui[Gimbal_Control_p->behaviour]);
    My_Char_Refresh(GM_MODE);
    if(Gimbal_Control_p->replenish_flag)
        Char_Draw(&replenish, "002", UI_Graph_Change, 6, UI_Color_Green, 20, 4, 80, 600, "replenish:ing");
    else
        Char_Draw(&replenish, "002", UI_Graph_Change, 6, UI_Color_Orange, 20, 4, 80, 600, "replenish:finish");
    My_Char_Refresh(replenish);

}

/**
  * @brief      ���ݵ�ѹ��ʾ
  */
static void print_cap_voltage(void)
{
    //    CAP_VOLTAGE.graph_Float = (int)(Chassis_Control_p->super_cap_c->Capacitance_voltage * 1000);
    //    CAP_VOLTAGE.operate_tpye = UI_Graph_Change;
    //    My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);
}

/**
  * @brief      ������̨�Ƕ���ʾ
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
  * @brief      ��׼����
  */
static void print_reticle(void)
{
    Line_Draw(&reticle1, "006", UI_Graph_Change, 5, UI_Color_Green, 3, 960, 580, 960, 350);
    My_Graph_Refresh(&reticle1);
    Line_Draw(&reticle2, "007", UI_Graph_Change, 5, UI_Color_Green, 3, 900, 468, 1020, 468);
    My_Graph_Refresh(&reticle2);
	  Circle_Draw(&circle1, "008", UI_Graph_Change, 2, UI_Color_Purplish_red, 4, 960, 540, 30);
	  My_Graph_Refresh(&circle1);
}