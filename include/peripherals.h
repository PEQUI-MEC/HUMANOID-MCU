#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include <Arduino.h>
#include <RosCommunication.h>
#include <ServoManager.h>
#include <config.h>
#include <utils/generic_functions.h>

/**
 * Checks if the peripheral buttons are pressed and execute its action.
 */
void check_buttons();

/**
 * Executes the action when the peripheral button 0 is pressed
 * @see check_buttons()
 */
void run_button0_action();

/**
 * Executes the action when the peripheral button 1 is pressed
 * @see check_buttons()
 */
void run_button1_action();

/**
 * Executes the action when the peripheral button 2 is pressed
 * @see check_buttons()
 */
void run_button2_action();

/**
 * Executes the action when the peripheral button 3 is pressed
 * @see check_buttons()
 */
void run_button3_action();

/**
 * Executes the action when the peripheral button 4 is pressed
 * @see check_buttons()
 */
void run_button4_action();

/**
 * Helper function to write the same value on the peripheral LED for debug and
 * the LED_BUILTIN
 */
void write_debug_led(uint8_t value);

/**
 * Helper function to toggle the output value on the peripheral LED for debug
 * and the LED_BUILTIN
 */
void toggle_debug_led();

#endif
