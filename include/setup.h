#ifndef SETUP_H_
#define SETUP_H_

#include <Arduino.h>
#include <config.h>
#include <std_msgs/Int16MultiArray.h>

void setup_pin_modes();
void setup_serial_baud_rate();

// Main functions
void joint_pos_callback(const std_msgs::Int16MultiArray& msg);
void check_buttons();
void run_button0_action();
void run_button1_action();
void run_button2_action();
void run_button3_action();
void run_button4_action();

#endif
