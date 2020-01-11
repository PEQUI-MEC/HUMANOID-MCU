#ifndef SERVOMANAGER_H_
#define SERVOMANAGER_H_

#include <Arduino.h>
#include <BodyServo.h>
#include <DMASerial.h>
#include <XYZrobotServo.h>
#include <config.h>
#include <peripherals.h>
#include <utils/DelayTimer.h>
#include <utils/generic_functions.h>

#undef min  // Workaround to fix compilation error
#undef max  // Workaround to fix compilation error
#include <array>

// TODO: Rework ServoManager to allow sending different commands besides
// position

/**
 * Enumeration of the states used by ServoManager
 */
enum class ManagerState {
  /**
   * Wait the servo to be connected and turned on.
   * @see #state_logic()
   */
  WaitServo,
  /**
   * The ServoManager is initialized and is sending commands with the initial
   * position to the servos.
   */
  Initial,
  /**
   * The first IDLE message was received from the gait control.
   */
  IdleReceived,
  /**
   * Sends the IDLE message to the servos with PLAYTIME_SMOOTH as playtime.
   */
  SendSmoothIdle,
  /**
   * Waits until the end of the smooth movement to the IDLE position.
   */
  WaitSmoothIdle,
  /**
   * The ServoManager is ready and should send the ready message to the gait
   * control.
   */
  Ready,
  /**
   * The ServoManager running and is constantly sending the position command to
   * the servos.
   */
  Running
};

/**
 * Class to manage the serial communication with the servos.
 */
class ServoManager {
 private:
  /**
   * Current state of the servo Manager.
   */
  ManagerState state;

  /**
   * Array with all the BodyServos managed by the ServoManager.
   */
  std::array<BodyServo, NUM_SERVOS> servos;

 public:
  /**
   * Serial used to send commands to the servos.
   * @see #send_pos_cmd()
   */
  DMASerial serial;

  /**
   * Non-blocking delay used to wait the playtime of the movement.
   */
  DelayTimer delay;

  /**
   * Buffer where the position command is assembled.
   * @see #assemble_pos_cmd()
   */
  uint8_t* cmd_buffer;

  /**
   * Flag to enable or disable sending position messages to servos.
   */
  bool enable;

  /**
   * Flag to enable or disable toque in the servos.
   */
  bool torque;

  /**
   * Playtime of the movement (units of 10ms).
   * E.g.: The value of 10 is equivalent to 100ms.
   */
  uint8_t playtime;

  /**
   * @param start_state The state the ServoManager will start it's execution.
   */
  ServoManager(ManagerState start_state = ManagerState::WaitServo);

  ~ServoManager();

  /**
   * @return Current state of the ServoManager.
   */
  ManagerState get_state();

  /**
   * Changes the current state of the ServoManager.
   * @param state New state.
   */
  void set_state(ManagerState state);

  /**
   * Runs the logic for the current state. The actions for the state are
   * executed and the conditions for state transition are checked.
   */
  void state_logic();

  /**
   * Returns the ServoManager to its Initial state and change position of all
   * servos to zero.
   */
  void reset();

  /**
   * Gets the index of the specified servo in the servos array.
   *
   * @param cid Control ID of the servo to be found.
   * @return Index of the servo in the servos array or -1 if no servo was found
   * with the specified cid.
   */
  uint8_t get_servo_index(uint8_t cid);

  /**
   * Sets the goal position of the servo with the specified cid.
   *
   * @param cid Control ID of the servo.
   * @param position Goal position to be set.
   */
  void set_position(uint8_t cid, int16_t position);

  /**
   * Checks if the servo with the specified ID is connected.
   *
   * Blinks the LED_BUILTIN in two stages and uses the XYZrobotServo llibrary to
   * send a status command to the servo.
   *
   * @param id Control ID or Real ID of the servo to check.
   * @param translate_id If set to true, the ID will be considered as a Control
   * ID and will be translated into a Real ID.
   * @return True if received a response for the status message without errors.
   *
   * @see XYZrobotServo#readStatus()
   */
  bool is_servo_connected(uint8_t id = CHECK_ID, bool translate_id = true);

  /**
   * Assembles in the buffer the position message based on each servo goal
   * position and the current playtime and torque configurations.
   *
   * Set up an SJOG command with the header, the data for each servo and the
   * checksum.
   *
   * @param buffer The buffer where the message will be written.
   */
  void assemble_pos_cmd(uint8_t* buffer);

  /**
   * Checks if a new position message can be sent, requests the assemble of the
   * message, transfers the message the the serial buffer and send it.
   *
   * @return True if the message was sent.
   */
  bool send_pos_cmd();
};

/**
 * Global ServoManager instance.
 */
extern ServoManager manager;

#endif
