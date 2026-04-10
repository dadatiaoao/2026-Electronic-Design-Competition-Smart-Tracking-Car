#ifndef __gray_track_h_
#define __gray_track_h_
#include "headfile.h"
#define AD0_PIN		Pin_5
#define AD1_PIN		Pin_8
#define AD2_PIN		Pin_6
#define AD_PORT		GPIO_B

#define OUT_PIN   Pin_15
#define OUT_PORT  GPIO_A//���Ŷ���

//ȫ�ֱ���������1=��ɫ���棬0=���ߣ�
extern unsigned char D1,D2,D3,D4,D5,D6,D7,D8;
//��ʼ��������
void gray_init(void);
// �л�������ͨ�������ζ�ȡ
void gray_channel(unsigned char ch);
// һ���Ի�ȡ8·��������ֵ
void gray_read(void);
// ѭ�����ƺ���
void track(void);
// ��ȡ��⵽�Ĵ�����������
int get_detected_sensor_count(void);



#endif // __INFRARED_TRACKING_H__    








