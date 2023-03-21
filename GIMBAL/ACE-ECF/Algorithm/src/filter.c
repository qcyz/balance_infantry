#include "filter.h"
#include "stdlib.h"
/*********************************************************************һ�׿�����***********************************************************************/
/**
 * @author  Liu heng
 * һ�׿������˲�������RoboMaster��̳
 *   һά�������˲���
 *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲���
 *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
 *          ʹ��ʾ��
 *          extKalman_t p;                  //����һ���������˲����ṹ��
 *          float SersorData;             //��Ҫ�����˲�������
 *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����Q=20 R=200����
 *          while(1)
 *          {
 *             SersorData = sersor();                     //��ȡ����
 *             SersorData = KalmanFilter(&p,SersorData);  //�����ݽ����˲�
 *          }
 */

/**
 * @name   kalmanCreate
 * @brief  ����һ���������˲���
 * @param  p:  �˲���
 *         T_Q:ϵͳ����Э����
 *         T_R:��������Э����
 *
 * @retval none
 * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
 *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
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
 * @brief  �������˲���
 * @param  p:  �˲���
 *         dat:���˲�����
 * @retval �˲��������
 * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
 *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
 *            �����ǿ�������5�����Ĺ�ʽ
 *            һ��H'��Ϊ������,����Ϊת�þ���
 */

float KalmanFilter(extKalman_t *p, float dat)
{
  p->X_mid = p->A * p->X_last;                    //���㵱ǰʱ�̵�Ԥ����(1)         x(k|k-1) = A*X(k-1|k-1) + B*U(k) + W(K)
  p->P_mid = p->A * p->P_last + p->Q;             //���㵱ǰʱ��Ԥ������Э����(2)    p(k|k-1) = A*p(k-1|k-1)*A' + Q
  p->kg = p->P_mid / (p->P_mid + p->R);           //����klaman����(4)               kg(k) = p(k|k-1)*H' / (H*p(k|k-1)*H'+R)
  p->X_now = p->X_mid + p->kg * (dat - p->X_mid); //���㵱ǰʱ�̵����Ž��(3)         x(k|k) = X(k|k-1) + kg(k)*(Z(k) - H*X(k|k-1))
  p->P_now = (1 - p->kg) * p->P_mid;              //���㵱ǰʱ�����Ž����Э����(5)    p(k|k) = (I-kg(k)*H) * P(k|k-1)

  p->P_last = p->P_now; //״̬����
  p->X_last = p->X_now;

  return p->X_now; //���Ԥ����x(k|k)
}


//һ�׵�ͨ�˲�����
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
  first_order_filter_type->input = input;
  first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
  first_order_filter_type->last_input = first_order_filter_type->out;

  return first_order_filter_type->out;
}

//һ�׵�ͨ�˲���ʼ��
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

