/**
 * @author        :robosea
 * @version       :v1.0.0
 * @Date          :2019-12-16 11:15:42
 * @LastEditors:Robosea
 * @LastEditTime:2020-03-24 19:42:11
 * @FilePath      :\ROV_F429_APP-10-10\USER\DRIVER\pca9685.c
 * @brief         :PCA9685��IICת16·PWMģ��
 */
#include "pca9685.h"
#include <math.h>
/***************************************************
* PCA9685����
****************************************************/
void pca_write(u8 adrr, u8 data) //��PCAд����,adrrd��ַ,data����
{
    HAL_I2C_Mem_Write(&PCA9685_IIC, pca_adrr, adrr, 1, &data, 1, 0X10);
}

u8 pca_read(u8 adrr) //��PCA������
{
    u8 data;
    HAL_I2C_Master_Transmit(&PCA9685_IIC, pca_adrr, &adrr, 1, 0x100);
    HAL_I2C_Master_Receive(&PCA9685_IIC, pca_adrr | 0x01, &data, 1, 0x100);
    return data;
}
void pca_setfreq(float freq) //����PWMƵ��
{
    u8     prescale, oldmode, newmode;
    double prescaleval;
    freq *= 0.965f; //0.92
    prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    prescale = floor(prescaleval + 0.5f);

    oldmode = pca_read(pca_mode1);
    newmode = (oldmode & 0x7F) | 0x10; // sleep
    pca_write(pca_mode1, newmode);     // go to sleep
    pca_write(pca_pre, prescale);      // set the prescaler
    pca_write(pca_mode1, oldmode);
    delay_ms(2);

    pca_write(pca_mode1, oldmode | 0xa1);
}

void pca_setpwm(u8 num, u32 on, u32 off)
{
    pca_write(LED0_ON_L + 4 * num, on);
    pca_write(LED0_ON_H + 4 * num, on >> 8);
    pca_write(LED0_OFF_L + 4 * num, FACTOR * off / RATIO);
    pca_write(LED0_OFF_H + 4 * num, (FACTOR * off / RATIO) >> 8);
}
/*num:���PWM�������0~15��on:PWM��������ֵ0~4096,off:PWM�½�����ֵ0~4096
һ��PWM���ڷֳ�4096�ݣ���0��ʼ+1�������Ƶ�onʱ����Ϊ�ߵ�ƽ������������offʱ
����Ϊ�͵�ƽ��ֱ������4096���¿�ʼ�����Ե�on������0ʱ������ʱ,��on����0ʱ��
off/4096��ֵ����PWM��ռ�ձȡ�*/

/*
	�������ã���ʼ�����������
	������1.PWMƵ��
		    2.��ʼ��PWM
*/
void PCA_MG9XX_Init(float hz, u16 init_pwm)
{
    u32 off = 0;
    pca_write(pca_mode1, 0x0);
    pca_setfreq(hz); //����PWMƵ��
    off = init_pwm;
    pca_setpwm(0, 0, off);
    pca_setpwm(1, 0, off);
    pca_setpwm(2, 0, off);
    pca_setpwm(3, 0, off);
    pca_setpwm(4, 0, off);
    pca_setpwm(5, 0, off);
    pca_setpwm(6, 0, off);
    pca_setpwm(7, 0, off);
    pca_setpwm(8, 0, off);
    pca_setpwm(9, 0, MANIPULATOR_MID);
    pca_setpwm(10, 0, off);
    pca_setpwm(11, 0, off);
    pca_setpwm(12, 0, off);
    pca_setpwm(13, 0, off);
    pca_setpwm(14, 0, off);
    pca_setpwm(15, 0, off);
    //	delay_ms(500);
}

/*
	�������ã����ƶ��ת����
	������1.����˿ڣ���ѡ0~15��
		  2.��ʼ�Ƕȣ���ѡ0~180��
		  3.�����Ƕȣ���ѡ0~180��
		  4.ģʽѡ��0 ��ʾ����������ʱ������ʱ��Ҫ�ں������������ʱ�������Ҳ��ɵ��٣������������������ֵ��
					  1 ��ʾ����������ʱ������ʱ����Ҫ�ں������������ʱ�������Ҳ��ɵ��٣������������������ֵ��
					  2 ��ʾ�ٶȿɵ��������������ʾ�ٶ�ֵ��
		  5.�ٶȣ�������� 0 ������ֵ���� 1 ʱ�ٶ���죬��ֵԽ���ٶ�ԽС��
	ע�����ģʽ 0��1 ���ٶȱ�ģʽ 2 ������ٶȴ�
*/
void PCA_MG9XX(u8 num, u8 start_angle, u8 end_angle, u8 mode, u8 speed)
{
    u8  i;
    u32 off = 0;
    switch (mode)
    {
    case 0:
        off = (u32)(158 + end_angle * 2.2);
        pca_setpwm(num, 0, off);
        break;
    case 1:
        off = (u32)(158 + end_angle * 2.2);
        pca_setpwm(num, 0, off);
        if (end_angle > start_angle)
        {
            delay_ms((u16)((end_angle - start_angle) * 2.7));
        }
        else
        {
            delay_ms((u16)((start_angle - end_angle) * 2.7));
        }
        break;
    case 2:
        //			if(end_angle>start_angle)
        //			{
        //				for(i=start_angle;i<=end_angle;i++)
        //				{
        //					off=(u32)(158+i*2.2);
        //					pca_setpwm(num,0,off);
        //					delay_ms(2);
        //					delay_us(speed*250);
        //				}
        //			}
        //			else if(start_angle>end_angle)
        //			{
        //				for(i=start_angle;i>=end_angle;i--)
        //				{
        //					off=(u32)(158+i*2.2);
        //					pca_setpwm(num,0,off);
        //					delay_ms(2);
        ////					delay_us(speed*250);
        //				}
        //			}
        break;
    }
}

void PCA9685_Init(void)
{
    PCA_MG9XX_Init(50, 1450); //��ʼ���������  1450ȫͣ
}

int  test_pwm1 = 1500;
int  test_pwm2 = 1500;
int  test_pwm3 = 1500;
int  test_pwm4 = 1500;
int  test_pwm5 = 1500;
int  test_pwm6 = 1500;
void PCA9685_Test_Demo(void)
{
    PCA_MG9XX_Init(50, 1500); //��ʼ���������

    while (1)
    {
        HAL_Delay(500);
        pca_setpwm(0, 0, test_pwm1);  //pwm0   ------- 1���ƽ���
        pca_setpwm(1, 0, test_pwm2);  //pwm1   ------- 2���ƽ���
        pca_setpwm(2, 0, test_pwm3);  //pwm2   ------- 3���ƽ���
        pca_setpwm(4, 0, test_pwm4);  //pwm4   ------- 4���ƽ���
        pca_setpwm(6, 0, test_pwm5);  //pwm6   ------- 5���ƽ���
        pca_setpwm(15, 0, test_pwm6); //pwm15 ------- 6���ƽ���
    }
}
