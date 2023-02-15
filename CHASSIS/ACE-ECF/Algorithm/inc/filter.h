#ifndef __FILTER_H__
#define __FILTER_H__
#include "struct_typedef.h"
#include "arm_math.h"
#include "mat.h"
#include "cmsis_os.h"


#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif
typedef struct
{
    float raw_value;
    float filtered_value[4];
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, p, Pminus, k;
} kalman_filter_t;

typedef struct
{
    float raw_value;
    float filtered_value[4];
    float xhat_data[4], xhatminus_data[4], z_data[2], Pminus_data[16], K_data[8];
    float P_data[16];
    float AT_data[16], HT_data[8];
    float A_data[16];
    float H_data[8];
    float Q_data[16];
    float R_data[4];
} kalman_filter_init_t;

typedef struct
{
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     // kalman����
    float A;      //ϵͳ����
    float B;
    float Q;
    float R;
    float H;
} extKalman_t;

void KalmanCreate(extKalman_t *p, float T_Q, float T_R);
float KalmanFilter(extKalman_t *p, float dat);


typedef struct _sliding_average_filter
{
    int w_size;
    int head;
    double sum;
    double *cache;
} SlidAveFilterObj;




//һ�׵�ͨ�˲�����
typedef __packed struct
{
    fp32 input;      //��������
    fp32 last_input; //�ϴ�����
    fp32 out;        //�˲����������
    fp32 num;        //�˲�����
} first_order_filter_type_t;

/* һ�׵�ͨ�˲� */
extern float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);

void sliding_average_filter_init(SlidAveFilterObj *filter, int w_size);

float sliding_average_filter(SlidAveFilterObj *filter, double k);

int sliding_average_filter_cache_add(SlidAveFilterObj *filter, double *cache, int w_size);
#endif
