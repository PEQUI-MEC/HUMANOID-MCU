#ifndef ROSCOMMUNICATION_H_
#define ROSCOMMUNICATION_H_

#include <ServoManager.h>
#include <config.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

extern ros::NodeHandle nh;

extern std_msgs::String str_msg;
extern ros::Publisher control_cmd_pub;
extern ros::Subscriber<std_msgs::Int16MultiArray> joint_pos_sub;

void ros_setup();
void ros_spin();
void ros_check_connection(uint8_t led);

void joint_pos_callback(const std_msgs::Int16MultiArray& msg);

#endif
