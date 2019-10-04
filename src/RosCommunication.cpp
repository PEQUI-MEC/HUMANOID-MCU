#include <RosCommunication.h>

RosCommunication control;

RosCommunication::RosCommunication()
    : cmd_pub("PMH/control_command", &this->str_msg),
      joint_pos_sub("PMH/joint_pos", joint_pos_callback),
      control_status_sub("PMH/control_status", control_status_callback) {
  status.state = ControlState::Unknown;
  status.is_mode_manual = true;
}

void RosCommunication::setup() {
  nh.getHardware()->setBaud(BAUD_RATE_CONTROL);
  nh.initNode();
  nh.advertise(cmd_pub);
  nh.subscribe(joint_pos_sub);
  nh.subscribe(control_status_sub);
  last_spin = millis();
}

void RosCommunication::spin() {
  if (millis() - last_spin >= SPIN_PERIOD) {
    nh.spinOnce();
    last_spin = millis();
  }
}

void RosCommunication::check_connection() {
  connected = nh.connected();
  digitalWrite(LED_CONNECTION, connected);
}

void RosCommunication::publish_command(const char* cmd) {
  str_msg.data = cmd;
  cmd_pub.publish(&str_msg);
}

// Callbacks

void joint_pos_callback(const std_msgs::Int16MultiArray& msg) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++)
    manager.set_position(i, msg.data[i]);

  if (manager.get_state() == ManagerState::Initial &&
      control.status.state == ControlState::Idle)
    manager.set_state(ManagerState::IdleReceived);
}

void control_status_callback(const std_msgs::UInt8MultiArray& msg) {
  control.status.state = static_cast<ControlState>(msg.data[0]);
  control.status.is_mode_manual = msg.data[1];
  digitalWrite(LED_CONTROL_MODE, control.status.is_mode_manual);
}
