#undef min  // Workaround para corrigir erro de compilação
#undef max  // Workaround para corrigir erro de compilação
#include <array>

#include <Arduino.h>
#include <XYZrobotServo.h>
#include <config.h>

// This is the maximum PWM value, out of 1023, to use when
// driving the servos. Setting it lower makes it easier to
// manipulate the arm by hand, but if you set it too low, the arm
// will not be able to hold itself up.
const uint16_t maxPwm = 80;

// This is how much the arm's position measurement has to differ
// from its target position before this sketch adjusts its target
// position.  If you set it too low, then the deflections caused
// by gravity will be large enough to make the arm move, and it
// will not hold itself up.  If you set it to high, it will be
// hard to accurately position the arm.
const uint16_t servoHysteresis = 5;

std::array<XYZrobotServo, NUM_SERVOS> servos = {
    {XYZrobotServo(SERIAL_SERVOS, RIGHT_ANKLE_ROLL_ID),
     XYZrobotServo(SERIAL_SERVOS, RIGHT_ANKLE_PITCH_ID),
     XYZrobotServo(SERIAL_SERVOS, RIGHT_KNEE_ID),
     XYZrobotServo(SERIAL_SERVOS, RIGHT_HIP_PITCH_ID),
     XYZrobotServo(SERIAL_SERVOS, RIGHT_HIP_ROLL_ID),
     XYZrobotServo(SERIAL_SERVOS, RIGHT_HIP_YAW_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_ANKLE_ROLL_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_ANKLE_PITCH_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_KNEE_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_HIP_PITCH_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_HIP_ROLL_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_HIP_YAW_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_ARM_PITCH_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_ARM_YAW_ID),
     XYZrobotServo(SERIAL_SERVOS, LEFT_ARM_ROLL_ID),
     XYZrobotServo(SERIAL_SERVOS, RIGHT_ARM_PITCH_ID),
     XYZrobotServo(SERIAL_SERVOS, RIGHT_ARM_YAW_ID),
     XYZrobotServo(SERIAL_SERVOS, RIGHT_ARM_ROLL_ID)}};

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
  SERIAL_SERVOS.setTimeout(20);

  delay(3000);
}

bool updateServo(XYZrobotServo& servo) {
  XYZrobotServoStatus status = servo.readStatus();
  if (servo.getLastError())
    return false;

  int16_t newPosRefSigned = constrain(
      (int16_t)status.posRef, (int16_t)(status.position - servoHysteresis),
      (int16_t)(status.position + servoHysteresis));

  // Convert posRef back to an unsigned number, handling the case
  // where it is negative.
  uint16_t newPosRef = newPosRefSigned < 0 ? 0 : newPosRefSigned;

  servo.setPosition(newPosRef);
  servo.writeMaxPwmRam(maxPwm);

  return true;
}

void updateServos() {
  for (auto servo : servos) {
    bool success = updateServo(servo);
    if (!success) {
      Serial.print(F("Error: Failed to communicate with servo "));
      Serial.println(servo.getId());
    }
  }
}

void handleSerialCommands() {
  if (!Serial.available())
    return;

  char input = Serial.read();

  if (input == 'r') {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      int16_t posRef = ((servos[i].readPosition() / 1024.0) * 3300.0) - 1650;
      Serial.print(posRef);
      if (i + 1 != NUM_SERVOS)
        Serial.print(F(", "));
    }

    Serial.println();
  }
}

void loop() {
  updateServos();
  handleSerialCommands();
  delay(20);
}
