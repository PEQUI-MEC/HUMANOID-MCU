#include <RosCommunication.h>

RosCommunication control;

RosCommunication::RosCommunication()
    : cmd_pub("PMH/control_command", &this->str_msg),
      joint_pos_sub("PMH/joint_pos", joint_pos_callback),
      ignore(PLAYTIME_SMOOTH * 10) {}

void RosCommunication::setup() {
  nh.getHardware()->setBaud(BAUD_RATE_CONTROL);
  nh.initNode();
  nh.advertise(cmd_pub);
  nh.subscribe(joint_pos_sub);
  last_spin = millis();
}

void RosCommunication::spin() {
  if (millis() - last_spin >= SPIN_PERIOD) {
    nh.spinOnce();
    last_spin = millis();
  }
}

void RosCommunication::check_connection(uint8_t led) {
  digitalWrite(led, nh.connected());
}

void RosCommunication::publish_command(const char* cmd) {
  str_msg.data = cmd;
  cmd_pub.publish(&str_msg);
}

// Callbacks

void joint_pos_callback(const std_msgs::Int16MultiArray& msg) {
  if (!control.ignore.has_finished())
    return;

  for (uint8_t i = 0; i < NUM_SERVOS; i++)
    manager.set_position(i, msg.data[i]);

  manager.updated_joint_pos();
}
