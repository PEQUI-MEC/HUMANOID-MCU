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

// TODO: Clear ros_lib to include only necessary msgs

/**
 * Class that handles all communication with the computer through ros-serial.
 */
class RosCommunication {
 public:
  ros::NodeHandle nh;

  /**
   * String message used to publish the command topic.
   * @see #publish_command(const char* cmd)
   */
  std_msgs::String str_msg;

  /**
   * ROS publisher used to publish command messages.
   * @see #publish_command(const char* cmd)
   */
  ros::Publisher cmd_pub;

  /**
   * ROS subscriber that listens to the joint position messages.
   */
  ros::Subscriber<std_msgs::Int16MultiArray> joint_pos_sub;

  /**
   * ROS subscriber that listens to the control status messages.
   */
  ros::Subscriber<std_msgs::UInt8MultiArray> control_status_sub;

  /**
   * ROS subscriber that listens to the interpolate position messages.
   */
  ros::Subscriber<std_msgs::Int16MultiArray> interpolate_sub;

  /**
   * Millis of when the last spin happened.
   * @see #spin()
   */
  time_t last_spin;

  /**
   * Flag to indicate if the node handle is connected to the computer ROS node.
   * @see #check_connection()
   */
  bool connected;

  /**
   * Gait control status received in the last "control_status" message.
   * @see control_status_callback(const std_msgs::UInt8MultiArray& msg)
   */
  ControlStatus status;

  RosCommunication();

  /**
   * Sets the serial baud rate, setups the node handle, advertise the publishers
   * and setup the subscribers.
   */
  void setup();

  /**
   * Does a ros-serial spinOnce when it's time and update the last_spin member.
   * @see ros::NodeHandle#spinOnce()
   */
  void spin();

  /**
   * Checks if the connection on the node handle is active and update the
   * peripheral led that indicates the connection status.
   */
  void check_connection();

  /**
   * Publishes a command for the computer.
   * @param cmd The command to be published.
   */
  void publish_command(const char* cmd);
};

/**
 * Global RosCommunication instance.
 */
extern RosCommunication control;

/**
 * Callback executed when a joint position message is received.
 */
void joint_pos_callback(const std_msgs::Int16MultiArray& msg);

/**
 * Callback executed when a control status message is received.
 */
void control_status_callback(const std_msgs::UInt8MultiArray& msg);

/**
 * Callback executed when a interpolate position message is received.
 */
void interpolation_callback(const std_msgs::Int16MultiArray& msg);

#endif
