#include <ros/ros.h>
#include <pigpiod_if2.h>

#include <std_msgs/Bool.h>

#include "ssoni_mobile_msgs/motor_cmd.h"

#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <mobile_robot/motor_algorithm.h>

#include <iostream>
#include <fstream>

int pinum;

volatile int EncoderCounter1;
volatile int EncoderCounter2;

bool switch_direction;
int Theta_Distance_Flag;

///////////Text_Input
int PWM_range;
int PWM_frequency;
float PID_limit;
float Motor_Steering_limit;
float Theta_RPM;
float Distance_Default_RPM;
float Distance_Modify_RPM;
float SS_Default_RPM;
float SS_Modify_RPM;
float SS_Cturn_Ratio;
float SS_Pturn_Ratio;
float Control_Cycle;
float Sync_Left;
float Sync_Right;
/////////////////////
void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/edie/Desktop/EDIE19_ID/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PID_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Motor_Steering_limit = atof(line.substr(found+2).c_str()); break;
      case 4: Theta_RPM = atof(line.substr(found+2).c_str()); break;
      case 5: Distance_Default_RPM = atof(line.substr(found+2).c_str()); break;
      case 6: Distance_Modify_RPM = atof(line.substr(found+2).c_str()); break;
      case 7: SS_Default_RPM = atof(line.substr(found+2).c_str()); break;
      case 8: SS_Modify_RPM = atof(line.substr(found+2).c_str()); break;
      case 9: SS_Cturn_Ratio = atof(line.substr(found+2).c_str()); break;
      case 10: SS_Pturn_Ratio = atof(line.substr(found+2).c_str()); break;
      case 11: Control_Cycle = atof(line.substr(found+2).c_str()); break;
      case 12: Sync_Left = atof(line.substr(found+2).c_str()); break;
      case 13: Sync_Right = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}

int Motor_Setup(void)
{
  pinum=pigpio_start(NULL, NULL);
  
  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }

  PWM_range = 512;
  PWM_frequency = 40000; 

  set_mode(pinum, motor_DIR1, PI_OUTPUT);
  set_mode(pinum, motor_DIR2, PI_OUTPUT);

  set_mode(pinum, motor_PWM1, PI_OUTPUT);
  set_mode(pinum, motor_PWM2, PI_OUTPUT);

  set_mode(pinum, motor_EN1A, PI_INPUT);
  set_mode(pinum, motor_EN1B, PI_INPUT);

  set_mode(pinum, motor_EN2A, PI_INPUT);
  set_mode(pinum, motor_EN2B, PI_INPUT);

  set_PWM_range(pinum, motor_PWM2, PWM_range);
  set_PWM_range(pinum, motor_PWM2, PWM_range);
  
  set_PWM_frequency(pinum, motor_PWM1, PWM_frequency);
  set_PWM_frequency(pinum, motor_PWM2, PWM_frequency);
     
  ROS_INFO("Setup Fin");
  return 0;
}

void PWM_Default(void)
{
  gpio_write(pinum, motor_DIR1, PI_LOW);
  gpio_write(pinum, motor_DIR2, PI_LOW);
  set_PWM_dutycycle(pinum, motor_PWM1, 0);
  set_PWM_dutycycle(pinum, motor_PWM2, 0);
}

void Motor_Controller(int motor_num,int direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);

  if(motor_num == 1)
  {
   local_PWM = Limit_Function(pwm);
   if(direction == 1)
   {
     gpio_write(pinum, motor_DIR1, PI_LOW);
     set_PWM_dutycycle(pinum, motor_PWM1, local_PWM);
     current_Direction = true;
   }
   else if(direction == -1)
   {
     gpio_write(pinum, motor_DIR1, PI_HIGH);
     set_PWM_dutycycle(pinum, motor_PWM1, local_PWM);
     current_PWM = local_PWM;
     current_Direction = false;
   }
  }
  else if(motor_num == 2)
  {
   local_PWM = Limit_Function(pwm);
   if(direction == 1)
   {
     gpio_write(pinum, motor_DIR2, PI_LOW);
     set_PWM_dutycycle(pinum, motor_PWM1, local_PWM);
     current_Direction = true;
   }
   else if(direction == -1)
   {
     gpio_write(pinum, motor_DIR2, PI_HIGH);
     set_PWM_dutycycle(pinum, motor_PWM2, local_PWM);
     current_PWM = local_PWM;
     current_Direction = false;
   }
  }
}

void Accel_Controller(int motor_num, int direction, int desired_pwm)
{
  int local_PWM;
  if(desired_pwm > current_PWM)
  {
    local_PWM = current_PWM + acceleration;
    Motor_Controller(motor_num, direction, local_PWM);
  }
  else if(desired_pwm < current_PWM)
  {
    local_PWM = current_PWM - acceleration;
    Motor_Controller(motor_num, direction, local_PWM);
  }
  else
  {
    local_PWM = current_PWM;
    Motor_Controller(motor_num, direction, local_PWM);
  }
  //ROS_INFO("Current_PWM is %d", current_PWM);
}

void speed_controller(int desired_speed)
{
  speed_static_encoder_pulse_ = (encoder_pulse1*2); // digital low pass filter  // basic 2 ch

   //speed_static_encoder_pulse_ = (encoder_pulse1+ encoder_pulse2);
  encoder_pulse1 = 0;
  encoder_pulse2 = 0;
  result_rpm =  (((speed_static_encoder_pulse_)*60*control_freqency_)/(encoder_pulse_per_rotation_*channel_));


  speed_error_ = desired_speed  -  result_rpm ;
  speed_control_ = ( p_gain_speed_ * speed_error_);

  pwm_value_motor = (pwm_value_motor + speed_control_);

  if (pwm_value_motor > 512)
  {
    pwm_value_motor = 512;
  }

  if (pwm_value_motor < 0)
  {
    pwm_value_motor = 0;
  }

}

double position_controller(int desired_angle, int max_rpm)
{
  position_static_encoder_pulse_ = (encoder_pulse_position1+ encoder_pulse_position2);

  if(((desired_angle*encoder_pulse_per_rotation_*channel_)/360) <= position_static_encoder_pulse_)
  {
    check_position = true;
    return 0;
  }
  else
  {
    position_error_ = ((desired_angle*encoder_pulse_per_rotation_*channel_)*360) - position_static_encoder_pulse_;
    position_control_ = p_gain_position_ * position_error_;

    if(position_control_ >  max_rpm)
    {
      position_control_ = max_rpm;
    }
    if(position_control_ < 0)
    {
      position_control_ = 0;
    }
    return position_control_;
  }

}

void Initialize()
{
  PWM_limit = 150;
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;

  bool motor1_ena_uprising = false;
  bool motor1_ena_falling = false;

  bool motor1_enb_uprising = false;
  bool motor1_enb_falling = false;

  bool motor2_ena_uprising = false;
  bool motor2_ena_falling = false;

  bool motor2_enb_uprising = false;
  bool motor2_enb_falling = false;

  Interrupt_Setting(); 
 
  switch_direction = true;
  Theta_Distance_Flag = 0;

  ROS_INFO("Initialize Complete");
}

void Interrupt1A_Falling(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pi,motor_EN1B) == 1)EncoderCounter1 ++;
  else EncoderCounter1 --;
}

void Interrupt1A_Rising(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
 
  EncoderCounter1 ++;
}

void Interrupt1B_Falling(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pi,motor_EN1A) == 1)EncoderCounter1 ++;
  else EncoderCounter1 --;
}

void Interrupt1B_Rising(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{

  EncoderCounter1 ++;
}

void Interrupt2A_Falling(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pi,motor_EN2B) == 1)EncoderCounter2 ++;
  else EncoderCounter2 --;
}

void Interrupt2A_Rising(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{

  EncoderCounter2 ++;
}

void Interrupt2B_Falling(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pi,motor_EN2A) == 1)EncoderCounter2 ++;
  else EncoderCounter2 --;
}

void Interrupt2B_Rising(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{

  EncoderCounter2 ++;
}

void Interrupt_Setting()
{

    callback(pinum, motor_EN1A, EITHER_EDGE, Interrupt1A_Falling);
    callback(pinum, motor_EN2A, EITHER_EDGE, Interrupt2A_Falling);
    callback(pinum, motor_EN1B, EITHER_EDGE, Interrupt1B_Falling);
    callback(pinum, motor_EN2B, EITHER_EDGE, Interrupt2B_Falling);
}

int Limit_Function(int pwm)
{
  int output;
  if (pwm > PWM_limit*2)
  {
    output = PWM_limit;
    ROS_WARN("PWM too fast!!!");
  }
  else if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)
  {
	output = 0;
    ROS_WARN("trash value!!!");
  }
  else output = pwm;
  return output; 
}

void left_motor_joy_callback(const ssoni_mobile_msgs::motor_cmd::ConstPtr& msg)
{
  int local_PWM = Limit_Function(msg->motor_desired_speed);
  if(joy_flag)
  {
    Accel_Controller(1, msg->motor_desired_direction,local_PWM);
  }
}

void right_motor_joy_callback(const ssoni_mobile_msgs::motor_cmd::ConstPtr& msg)
{
  int local_PWM = Limit_Function(msg->motor_desired_speed);
  if(joy_flag)
  {
    Accel_Controller(2, msg->motor_desired_direction,local_PWM);
  }
}

void left_motor_operator_callback(const ssoni_mobile_msgs::motor_cmd::ConstPtr& msg)
{
  int local_PWM = Limit_Function(msg->motor_desired_speed);
  if(!(joy_flag))
  {
    Accel_Controller(1, msg->motor_desired_direction, local_PWM);
  }
}

void right_motor_operator_callback(const ssoni_mobile_msgs::motor_cmd::ConstPtr& msg)
{
  int local_PWM = Limit_Function(msg->motor_desired_speed);
  if(!(joy_flag))
  {
    Accel_Controller(2, msg->motor_desired_direction, local_PWM);
  }
}

void mode_select_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if(msg->data == true)
  {
    printf("joy flag is true\n");
    joy_flag = true;
  }
  else if(msg->data == false)
  {
    printf("joy flag is false\n");
    joy_flag = false;
    
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0); 
  }
} 