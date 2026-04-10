#include "gray_track.h"
#include "headfile.h"
#include <stdio.h>

// =====================���Ŷ���=====================
                    ///������


unsigned char D1,D2,D3,D4,D5,D6,D7,D8;

// =====================��ʼ��=====================
void gray_init(void)
{
    gpio_init(AD_PORT, AD0_PIN, OUT_PP);
    gpio_init(AD_PORT, AD1_PIN, OUT_PP);
    gpio_init(AD_PORT, AD2_PIN, OUT_PP);


    gpio_init(OUT_PORT, OUT_PIN, IU);
}

// ===================== ѡ��ͨ�� =====================
void gray_channel(unsigned char ch)
{
    switch(ch)
    {
        case 1: gpio_set(AD_PORT,AD2_PIN,0); gpio_set(AD_PORT,AD1_PIN,0); gpio_set(AD_PORT,AD0_PIN,0); break;
        case 2: gpio_set(AD_PORT,AD2_PIN,0); gpio_set(AD_PORT,AD1_PIN,0); gpio_set(AD_PORT,AD0_PIN,1); break;
        case 3: gpio_set(AD_PORT,AD2_PIN,0); gpio_set(AD_PORT,AD1_PIN,1); gpio_set(AD_PORT,AD0_PIN,0); break;
        case 4: gpio_set(AD_PORT,AD2_PIN,0); gpio_set(AD_PORT,AD1_PIN,1); gpio_set(AD_PORT,AD0_PIN,1); break;
        case 5: gpio_set(AD_PORT,AD2_PIN,1); gpio_set(AD_PORT,AD1_PIN,0); gpio_set(AD_PORT,AD0_PIN,0); break;
        case 6: gpio_set(AD_PORT,AD2_PIN,1); gpio_set(AD_PORT,AD1_PIN,0); gpio_set(AD_PORT,AD0_PIN,1); break;
        case 7: gpio_set(AD_PORT,AD2_PIN,1); gpio_set(AD_PORT,AD1_PIN,1); gpio_set(AD_PORT,AD0_PIN,0); break;
        case 8: gpio_set(AD_PORT,AD2_PIN,1); gpio_set(AD_PORT,AD1_PIN,1); gpio_set(AD_PORT,AD0_PIN,1); break;
        default:break;
    }
    // 传感器多路切换只需短暂建立时间，避免阻塞控制周期
    //delay_us(80);
}

// =====================������=====================
void gray_read(void)
{
    // 1
    gray_channel(1);
    D1 = gpio_get(OUT_PORT,OUT_PIN) ? 1 : 0;

    // 2
    gray_channel(2);
    D2 = gpio_get(OUT_PORT,OUT_PIN) ? 1 : 0;

    // 3
    gray_channel(3);
    D3 = gpio_get(OUT_PORT,OUT_PIN) ? 1 : 0;

    // 4
    gray_channel(4);
    D4 = gpio_get(OUT_PORT,OUT_PIN) ? 1 : 0;

    // 5
    gray_channel(5);
    D5 = gpio_get(OUT_PORT,OUT_PIN) ? 1 : 0;

    // 6
    gray_channel(6);
    D6 = gpio_get(OUT_PORT,OUT_PIN) ? 1: 0;

    // 7
    gray_channel(7);
    D7 = gpio_get(OUT_PORT,OUT_PIN) ? 1 : 0;

    // 8
    gray_channel(8);
    D8 = gpio_get(OUT_PORT,OUT_PIN) ? 1 : 0;
}

// ===================== Ѳ�� =====================
void track()      
{
    static float last_error = 0.0f;
    static float filt_error = 0.0f;
    const int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    const uint8_t sensor[8] = {D1, D2, D3, D4, D5, D6, D7, D8};
    int sum = 0;
    int hit = 0;
    int i;
    float error;
    float turn;
    int dynamic_base;
    int left;
    int right;

    for(i = 0; i < 8; i++)
    {
        if(sensor[i])
        {
            sum += weight[i];
            hit++;
        }
    }

    if(hit > 0)
    {
        error = (float)sum / (float)hit;
    }
    else
    {
        // 丢线时保持上一拍方向，降低突发抖动
        error = last_error;
    }

    // 一阶低通，抑制传感器抖动
    filt_error = 0.70f * filt_error + 0.30f * error;
    turn = 16.0f * filt_error + 8.0f * (filt_error - last_error);
    last_error = filt_error;

    dynamic_base = basicspeed - (int)(4.0f * fabsf(filt_error));
    if(dynamic_base < 70)
    {
        dynamic_base = 70;
    }

    left = dynamic_base + (int)turn;
    right = dynamic_base - (int)turn;

    if(left < 40) left = 40;
    if(right < 40) right = 40;
    if(left > 255) left = 255;
    if(right > 255) right = 255;

    motor_target_set(left, right);
}

// ��ȡ��⵽�Ĵ�����������
int get_detected_sensor_count(void) {
    return D1 + D2 + D3 + D4 + D5 + D6 + D7 + D8;
}

