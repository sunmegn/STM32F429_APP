#ifndef __PCA9685_H
#define __PCA9685_H

#include "includes.h"
#define pca_adrr             0x80

#define pca_mode1            0x0
#define pca_pre              0xFE

#define LED0_ON_L            0x6
#define LED0_ON_H            0x7
#define LED0_OFF_L           0x8
#define LED0_OFF_H           0x9
 
#define FACTOR               4096
#define RATIO                20000 

#define MANIPULATOR_MID					1500
#define MANIPULATOR_VAL(x) 		pca_setpwm(9,0,x);


#define MOTOR_PMW_MID        1470

typedef enum{
 HAND_GO_FRONT= 0,  	//0
 HAND_SHIFT_LEFT,     //1
 HAND_SHIFT_RIGHT,    //2
	
}MOveMode_t;

typedef struct
{
    short front_back;
    short left_right;
    short left_right_adjust;
    short up_down;
	  MOveMode_t mov_type;
}PwmMsg_t;

enum Motor_Channel{
// UL_MOTOR = 6,       //0
// UR_MOTOR = 1,           //1
// FL_MOTOR = 4,           //2
// FR_MOTOR = 0,           //3
// BL_MOTOR = 15,           //4
// BR_MOTOR = 2,           //5

	UL_MOTOR = 1,       //0
	UR_MOTOR = 6,           //1
	FL_MOTOR = 0,           //2
	FR_MOTOR = 4,           //3
	BL_MOTOR = 2,           //4
	BR_MOTOR = 15,           //5
};

void pca_write(u8 adrr,u8 data);
u8 pca_read(u8 adrr);
void PCA_MG9XX_Init(float hz,u16 init_pwm);
void pca_setfreq(float freq);
void pca_setpwm(u8 num, u32 on, u32 off);
void PCA_MG9XX(u8 num,u8 start_angle,u8 end_angle,u8 mode,u8 speed);
void PCA9685_Init(void);

#endif
