#include <ros_communication.h>

ros::NodeHandle nh;
time_t last_spin;

ros::Subscriber<std_msgs::Int16MultiArray> joint_pos_sub("Bioloid/joint_pos",
                                                         joint_pos_callback);

std_msgs::String str_msg;
ros::Publisher control_cmd_pub("Bioloid/control_command", &str_msg);

void ros_setup() {
  nh.getHardware()->setBaud(BAUD_RATE_CONTROL);
  nh.initNode();
  nh.advertise(control_cmd_pub);
  nh.subscribe(joint_pos_sub);
  last_spin = millis();
}

void ros_spin() {
  if (millis() - last_spin >= SPIN_PERIOD) {
    nh.spinOnce();
    last_spin = millis();
  }
}

void ros_check_connection(uint8_t led) {
  digitalWrite(led, nh.connected());
}

void joint_pos_callback(const std_msgs::Int16MultiArray& msg) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++)
    manager.set_position(i, msg.data[i]);

  manager.updated_joint_pos();
}
