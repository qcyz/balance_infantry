#include "filter.h"
#include "stdlib.h"
/*********************************************************************一阶卡尔曼***********************************************************************/
/**
 * @author  Liu heng
 * 一阶卡尔曼滤波器来自RoboMaster论坛
 *   一维卡尔曼滤波器
 *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器
 *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
 *          使用示例
 *          extKalman_t p;                  //定义一个卡尔曼滤波器结构体
 *          float SersorData;             //需要进行滤波的数据
 *          KalmanCreate(&p,20,200);      //初始化该滤波器的Q=20 R=200参数
 *          while(1)
 *          {
 *             SersorData = sersor();                     //获取数据
 *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波
 *          }
 */

/**
 * @name   kalmanCreate
 * @brief  创建一个卡尔曼滤波器
 * @param  p:  滤波器
 *         T_Q:系统噪声协方差
 *         T_R:测量噪声协方差
 *
 * @retval none
 * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
 *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
 */
void KalmanCreate(extKalman_t *p, float T_Q, float T_R)
{
  p->X_last = (float)0;
  p->P_last = 0;
  p->Q = T_Q;
  p->R = T_R;
  p->A = 1;
  p->B = 0;
  p->H = 1;
  p->X_mid = p->X_last;
}

/**
 * @name   KalmanFilter
 * @brief  卡尔曼滤波器
 * @param  p:  滤波器
 *         dat:待滤波数据
 * @retval 滤波后的数据
 * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
 *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
 *            以下是卡尔曼的5个核心公式
 *            一阶H'即为它本身,否则为转置矩阵
 */

float KalmanFilter(extKalman_t *p, float dat)
{
  p->X_mid = p->A * p->X_last;                    //计算当前时刻的预测结果(1)         x(k|k-1) = A*X(k-1|k-1) + B*U(k) + W(K)
  p->P_mid = p->A * p->P_last + p->Q;             //计算当前时刻预测结果的协方差(2)    p(k|k-1) = A*p(k-1|k-1)*A' + Q
  p->kg = p->P_mid / (p->P_mid + p->R);           //计算klaman增益(4)               kg(k) = p(k|k-1)*H' / (H*p(k|k-1)*H'+R)
  p->X_now = p->X_mid + p->kg * (dat - p->X_mid); //计算当前时刻的最优结果(3)         x(k|k) = X(k|k-1) + kg(k)*(Z(k) - H*X(k|k-1))
  p->P_now = (1 - p->kg) * p->P_mid;              //计算当前时刻最优结果的协方差(5)    p(k|k) = (I-kg(k)*H) * P(k|k-1)

  p->P_last = p->P_now; //状态更新
  p->X_last = p->X_now;

  return p->X_now; //输出预测结果x(k|k)
}


//一阶低通滤波计算
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
  first_order_filter_type->input = input;
  first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
  first_order_filter_type->last_input = first_order_filter_type->out;

  return first_order_filter_type->out;
}

//一阶低通滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num)
{
  if (first_order_filter_type == NULL)
  {
    return;
  }

  first_order_filter_type->input = 0;
  first_order_filter_type->last_input = 0;
  first_order_filter_type->num = num;
  first_order_filter_type->out = 0;
}

void sliding_average_filter_init(SlidAveFilterObj *filter, int w_size)
{
   
     filter->cache = (double*)user_malloc(sizeof(double) * w_size);
     memset(filter->cache, 0, sizeof(double) * w_size);

    
    filter->w_size = w_size;
    filter->sum = 0;
    filter->head = 0;
}

float sliding_average_filter(SlidAveFilterObj *filter, double k)
{
    filter->cache[filter->head] = k;
    filter->head = (filter->head + 1) % filter->w_size;
    filter->sum = filter->sum + k - filter->cache[filter->head];

    return (filter->sum / (double)filter->w_size);
}

int sliding_average_filter_cache_add(SlidAveFilterObj *filter, double *cache, int w_size)
{
    filter->cache = cache;
    filter->w_size = w_size;
    return 0;
}

