#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include <Arduino.h>
#include <ServoManager.h>
#include <config.h>
#include <ros_communication.h>
#include <utils/generic_functions.h>

void check_buttons();
void run_button0_action();
void run_button1_action();
void run_button2_action();
void run_button3_action();
void run_button4_action();

void write_debug_led(uint8_t value);
void toggle_debug_led();

#endif
