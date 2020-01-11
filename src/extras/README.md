# Source Extras

The source extras are programs with their own setup and loop functions that are intented to do a different task than the regular execution of the robot.

## How to use

Normally the `platformio.ini` file is configured to use everything except the extras folder.

```ini
src_filter =
  +<*>
  -<extras/>
```

To use an extras program instead of the main program, you should remove the `main.cpp` file from the filter and add the file of the extras program you want to use:

```ini
src_filter =
  +<*>
  -<extras/>
  -<main.cpp>
  +<extras/compliance.cpp>
```

## List of extras

### Compliance

**File**: `compliance.cpp`

This program makes the robot hold itself up, while complying when you try to move it by hand. If you send an `r` character through the Serial, this sketch will print the positions of all the servos separated by commas. These positions can then be coppied to a csv file and be used in a move of the robot.

Note that this program changes the Max_PWM parameter in the RAM of the servos. If your servos are not responding very well after running this program, you might need to power cycle them to make them reload the Max_PWM parameter from EEPROM.
