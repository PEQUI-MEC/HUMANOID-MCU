#ifndef CONTROLSTATUS_H_
#define CONTROLSTATUS_H_

/**
 * Enumeration of all possible states of the gait control.
 */
enum class ControlState {
  Unknown = 0,
  Idle = 1,
  March = 2,
  Walk = 3,
  Turn = 4,
  Interpolate = 5,
  Fallen = 6
};

/**
 * Struct to group all information of the status of the gait control.
 *
 * @see RosCommunication#status
 * @see control_status_callback(const std_msgs::UInt8MultiArray& msg)
 */
struct ControlStatus {
  ControlState state;
  bool is_mode_manual;
};

#endif
