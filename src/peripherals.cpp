#include <peripherals.h>

time_t btn_last_press[] = {0, 0, 0, 0, 0};

void run_button0_action() {
  if (manager.get_state() == ManagerState::WaitServo) {
    digitalWrite(LED0, HIGH);
    manager.set_state(ManagerState::Initial);
  }
}

void run_button1_action() {
  toggle_pin(LED1);
}

void run_button2_action() {
  toggle_pin(LED2);
}

void run_button3_action() {
  // toggle_pin(LED3);
}

void run_button4_action() {
  // toggle_pin(LED4);
}

void check_buttons() {
  time_t now = millis();
  if (digitalRead(BUTTON0) == LOW &&
      now - btn_last_press[0] >= BTN_PRESS_DELAY) {
    run_button0_action();
    btn_last_press[0] = now;
  }

  if (digitalRead(BUTTON1) == LOW &&
      now - btn_last_press[1] >= BTN_PRESS_DELAY) {
    run_button1_action();
    btn_last_press[1] = now;
  }

  if (digitalRead(BUTTON2) == LOW &&
      now - btn_last_press[2] >= BTN_PRESS_DELAY) {
    run_button2_action();
    btn_last_press[2] = now;
  }

  if (digitalRead(BUTTON3) == LOW &&
      now - btn_last_press[3] >= BTN_PRESS_DELAY) {
    run_button3_action();
    btn_last_press[3] = now;
  }

  if (digitalRead(BUTTON4) == LOW &&
      now - btn_last_press[4] >= BTN_PRESS_DELAY) {
    run_button4_action();
    btn_last_press[4] = now;
  }
}

void write_debug_led(uint8_t value) {
  digitalWrite(LED_BUILTIN, value);
  digitalWrite(LED4, value);
}

void toggle_debug_led() {
  toggle_pin(LED_BUILTIN);
  toggle_pin(LED4);
}
