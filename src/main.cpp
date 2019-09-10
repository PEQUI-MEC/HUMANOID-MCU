#include <Arduino.h>
#include <ServoManager.h>
#include <config.h>
#include <peripherals.h>
#include <ros_communication.h>

void setup_pin_modes() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON0, INPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
  pinMode(BUTTON4, INPUT);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
}

void setup_serial_baud_rate() {
  SERIAL_CONTROL.begin(BAUD_RATE_CONTROL);
  SERIAL_SERVOS.begin(BAUD_RATE_SERVOS);
  // SERIAL_DEBUG.begin(BAUD_RATE_DEBUG);
}

void setup() {
  setup_pin_modes();
  setup_serial_baud_rate();

  write_debug_led(LOW);
  delay(INITIAL_DELAY);
  write_debug_led(HIGH);
  delay(500);

  ros_setup();

  manager.serial.init(USART2, DMA_REQ_SRC_USART2_TX);
}

void loop() {
  manager.state_logic();
  manager.send_pos_cmd();

  ros_spin();
  ros_check_connection(LED3);

  check_buttons();
}
