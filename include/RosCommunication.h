#ifndef ROSCOMMUNICATION_H_
#define ROSCOMMUNICATION_H_

#include <ControlStatus.h>
#include <ServoManager.h>
#include <config.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <utils/DelayTimer.h>

class RosCommunication {
 public:
  RosCommunication();

  ros::NodeHandle nh;

  std_msgs::String str_msg;

  ros::Publisher cmd_pub;
  ros::Subscriber<std_msgs::Int16MultiArray> joint_pos_sub;
  ros::Subscriber<std_msgs::UInt8MultiArray> control_status_sub;

  time_t last_spin;
  bool connected;
  ControlStatus status;

  void setup();
  void spin();
  void check_connection();

  void publish_command(const char* cmd);
};

extern RosCommunication control;

void joint_pos_callback(const std_msgs::Int16MultiArray& msg);
void control_status_callback(const std_msgs::UInt8MultiArray& msg);

#endif
