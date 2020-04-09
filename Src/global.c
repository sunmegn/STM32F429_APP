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

u8 g_nowWokeMode = 0x03; //当前工作模式  默认正常工作模式 01 调试模式  02 标定模式  03 正常模式

u8 g_HeartBeatCnt = 0; //心跳计数

u8 g_runMode = 0; //控制模式，01 手动模式  04 定航模式  02 定深模式  06 定深定航模式

u16 Mid_pwm[6] = {1488, 1488, 1488, 1488, 1488, 1488}; //6推进器中值

u8 g_LostPC = 0; //默认通信正常  0 为通信正常  1 为与上位机通信丢失

u32 g_UpdateFlag = 0; //默认不更新APP  0 为不更新APP  1 为更新APP

u8 g_LEDWorkMode = 0; //默认不更新APP  0 为不更新APP  1 为更新APP

/**
  * @funNm  IIR_2OrderLpf_Init
  * @brief  二阶低通滤波器初值计算IIR滤波器形式
  * @param SOTF_typeDef:二阶变换结构体
  * @param fs:采样频率
  * @param fc:截止频率 注意：fc必须满足 fs >= 3.3*fc 
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
