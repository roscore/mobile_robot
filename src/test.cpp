#include <ros/ros.h>
#include <pigpiod_if2.h>

#include <std_msgs/Bool.h>

#include "ssoni_mobile_msgs/motor_cmd.h"

#include <stdlib.h>
#include <unistd.h>
#include <mobile_robot/motor_algorithm.h>

int main(int argc, char **argv)
{
   printf("Motor node Start \n");

  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;

  Initialize();
  ros::Rate loop_rate(10);

  ros::Subscriber left_motor_joy_sub = nh.subscribe("/motor_1", 10, left_motor_joy_callback);
  ros::Subscriber right_motor_joy_sub = nh.subscribe("/motor_2", 10, right_motor_joy_callback);

  ros::Subscriber left_motor_operator_sub = nh.subscribe("/heroehs/ssoni_robot/left_motor_cmd", 10, left_motor_operator_callback);
  ros::Subscriber right_motor_operator_sub = nh.subscribe("/heroehs/ssoni_robot/right_motor_cmd", 10, right_motor_operator_callback);

  ros::Subscriber mode_select_sub = nh.subscribe("/heroehs/ssoni_robot/mode_select", 10, mode_select_callback);

  while(ros::ok())
  {
    loop_rate.sleep();
    printf("EN1A %d, EN1B %d, EN2A %d, EN2B %d\n", gpio_read(0, motor_EN1A), gpio_read(0, motor_EN1B), gpio_read(0, motor_EN2A), gpio_read(0, motor_EN2B));
    ros::spinOnce();
  }

  Motor_Controller(1, true, 0);
  Motor_Controller(2, true, 0);

  return 0;
}