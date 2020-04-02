#ifndef MOTOR_ALGORITHM_H
#define MOTOR_ALGORITHM_H

#include "ssoni_mobile_msgs/motor_cmd.h"

#define motor_DIR1 26
#define motor_PWM1 12
#define motor_EN1A 6
#define motor_EN1B 5

#define motor_DIR2 19
#define motor_PWM2 13
#define motor_EN2A 22 
#define motor_EN2B 27

extern int pinum;

//text_input
extern int PWM_range;
extern int PWM_frequency;
extern float PID_limit;
extern float Motor_Steering_limit;
extern float Theta_RPM;
extern float Distance_Default_RPM;
extern float Distance_Modify_RPM;
extern float SS_Default_RPM;
extern float SS_Modify_RPM;
extern float SS_Cturn_Ratio;
extern float SS_Pturn_Ratio;
extern float Control_Cycle;
extern float Sync_Left;
extern float Sync_Right;

void Text_Input(void);
int Motor_Setup(void);
void PWM_Default(void);

extern volatile int EncoderCounter1;
extern volatile int EncoderCounter2;

void Interrupt1(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

extern bool joy_flag;

#endif // MOTOR_ALGORITHM_H
