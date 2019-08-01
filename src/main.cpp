#include <Arduino.h>
#include <DMAInterrupts.h>
#include <DMASerial.h>
#include <ServoManager.h>
#include <config.h>
#include <ros.h>
#include <setup.h>
#include <std_msgs/Int16MultiArray.h>
#include <utils/generic_functions.h>

uint8_t pos_cmd[SJOG_SIZE];
ServoManager manager;
DMASerial servo_serial(DMA1, SJOG_SIZE, DMA_IRQ_HANDLER_1);

uint8_t button_prev_state[] = {HIGH, HIGH, HIGH, HIGH, HIGH};

ros::NodeHandle nh;
time_t last_spin;
ros::Subscriber<std_msgs::Int16MultiArray> sub("Bioloid/joint_pos",
                                               joint_pos_callback);

void setup() {
  setup_pin_modes();
  setup_serial_baud_rate();

  digitalWrite(LED_BUILTIN, LOW);
  delay(INITIAL_DELAY);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  manager.set_state(STATE_INITIAL);

  nh.getHardware()->setBaud(BAUD_RATE_CONTROL);
  nh.initNode();
  nh.subscribe(sub);
  last_spin = millis();

  servo_serial.init(USART2, DMA_REQ_SRC_USART2_TX);
}

void loop() {
  if (!servo_serial.is_transfering() && manager.reset_delay()) {
    manager.assemble_pos_cmd(pos_cmd);
    servo_serial.set_data(pos_cmd, SJOG_SIZE);
    servo_serial.start();
    toggle_pin(LED_BUILTIN);
  }

  if (millis() - last_spin >= SPIN_PERIOD) {
    nh.spinOnce();
    last_spin = millis();
  }

  check_buttons();
}

void joint_pos_callback(const std_msgs::Int16MultiArray& msg) {
  if (msg.data_length != NUM_SERVOS + 1)
    return;

  for (uint8_t i = 0; i < NUM_SERVOS; i++)
    manager.set_position(i + 1, msg.data[i]);

  manager.set_state(msg.data[NUM_SERVOS]);
}

void check_buttons() {
  uint8_t state = digitalRead(BUTTON0);
  if (state == LOW && button_prev_state[0] == HIGH)
    run_button0_action();
  button_prev_state[0] = state;

  state = digitalRead(BUTTON1);
  if (state == LOW && button_prev_state[1] == HIGH)
    run_button1_action();
  button_prev_state[1] = state;

  state = digitalRead(BUTTON2);
  if (state == LOW && button_prev_state[2] == HIGH)
    run_button2_action();
  button_prev_state[2] = state;

  state = digitalRead(BUTTON3);
  if (state == LOW && button_prev_state[3] == HIGH)
    run_button3_action();
  button_prev_state[3] = state;

  state = digitalRead(BUTTON4);
  if (state == LOW && button_prev_state[4] == HIGH)
    run_button4_action();
  button_prev_state[4] = state;
}

void run_button0_action() {
  toggle_pin(LED0);
}
void run_button1_action() {
  toggle_pin(LED1);
}
void run_button2_action() {
  toggle_pin(LED2);
}
void run_button3_action() {
  toggle_pin(LED3);
}
void run_button4_action() {
  toggle_pin(LED4);
}
