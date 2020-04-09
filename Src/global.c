/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:42
 * @LastEditors   :Robosea
 * @LastEditTime  :2020-02-25 17:53:23
 * @brief         :
 */
#include "global.h"

float g_depthspeed = 0.0;

u8 g_nowWokeMode = 0x03; //��ǰ����ģʽ  Ĭ����������ģʽ 01 ����ģʽ  02 �궨ģʽ  03 ����ģʽ

u8 g_HeartBeatCnt = 0; //��������

u8 g_runMode = 0; //����ģʽ��01 �ֶ�ģʽ  04 ����ģʽ  02 ����ģʽ  06 �����ģʽ

u16 Mid_pwm[6] = {1488, 1488, 1488, 1488, 1488, 1488}; //6�ƽ�����ֵ

u8 g_LostPC = 0; //Ĭ��ͨ������  0 Ϊͨ������  1 Ϊ����λ��ͨ�Ŷ�ʧ

u32 g_UpdateFlag = 0; //Ĭ�ϲ�����APP  0 Ϊ������APP  1 Ϊ����APP

u8 g_LEDWorkMode = 0; //Ĭ�ϲ�����APP  0 Ϊ������APP  1 Ϊ����APP

/**
  * @funNm  IIR_2OrderLpf_Init
  * @brief  ���׵�ͨ�˲�����ֵ����IIR�˲�����ʽ
  * @param SOTF_typeDef:���ױ任�ṹ��
  * @param fs:����Ƶ��
  * @param fc:��ֹƵ�� ע�⣺fc�������� fs >= 3.3*fc 
  * @retval void:void
  */
void IIR_2OrderLpf_Init(SOTF_typeDef *n, float fs, float fc)
{
    float Ohm = tan(M_PI * fc / fs);
    float c   = 1 + 2 * cos(M_PI / 4) * Ohm + Ohm * Ohm;
    n->num[0] = Ohm * Ohm / c;
    n->num[1] = Ohm * Ohm * 2 / c;
    n->num[2] = Ohm * Ohm / c;
    n->den[0] = 1;
    n->den[1] = 2 * (Ohm * Ohm - 1) / c;
    n->den[2] = (1 - 2 * cos(M_PI / 4) * Ohm + Ohm * Ohm) / c;
}

void Rocker_SOTF_Clear(SOTF_typeDef *gs)
{
    gs->uk   = 0;
    gs->uk_1 = 0;
    gs->uk_2 = 0;
    gs->yk   = 0;
    gs->yk_1 = 0;
    gs->yk_2 = 0;
}
float SOTFOutput(SOTF_typeDef *gs, float data)
{
    gs->uk   = data;
    gs->yk   = -(gs->den[1]) * (gs->yk_1) - (gs->den[2]) * (gs->yk_2) + (gs->num[0]) * (gs->uk) + (gs->num[1]) * (gs->uk_1) + (gs->num[2]) * (gs->uk_2);
    gs->yk_2 = gs->yk_1;
    gs->yk_1 = gs->yk;
    gs->uk_2 = gs->uk_1;
    gs->uk_1 = gs->uk;
    return (gs->yk);
}

void AccLimit_Init(AccLimit_t *lim, float AccMax, float ts)
{
    lim->ts   = ts;
    lim->amax = AccMax;
}

float AccLim(AccLimit_t *lim, float Val)
{
    /**??***/
    lim->a = (Val - lim->lastvel) / lim->ts;
    if (lim->a > (lim->amax))
        lim->a = (lim->amax);
    else if (lim->a < -(lim->amax))
        lim->a = -(lim->amax);
    if (fabs(lim->fa - Val) < 0.00001)
    {
        lim->a = 0;
    }
    lim->fa      = lim->fa + lim->a * lim->ts;
    lim->nowv    = lim->fa;
    lim->lastvel = lim->fa;
    return lim->nowv;
}
