#include <Arduino.h>
#include <XYZrobotServo.h>

bool connected = false;
uint8_t overload_treshold[2] = {0xCC, 0xCC};
uint8_t const_val[2];

XYZrobotServo servo(Serial2, 16);

uint16_t generate_sine_positions(uint16_t min, uint16_t max, float f) {
  uint16_t amplitude = (max - min) / 2;
  uint16_t reference = min + amplitude;
  float time = (float)millis() / 1000.0;

  return ((amplitude * cos(2 * M_PI * f * time)) + reference);
}

void handle_serial() {
  String s = Serial.readStringUntil(';');
  Serial.print(s);
  return;

  char cmd = s[0];

  if (cmd == 'x') {
    connected = true;
    return;
  }

  int val = s.substring(1).toInt(), addr = 24;
  const_val[0] = (val >> 8) & 0xFF;
  const_val[1] = val & 0xFF;

  if (cmd == 'p')
    addr = 24;
  else if (cmd == 'd')
    addr = 26;
  else if (cmd == 'i')
    addr = 28;
  else
    return;

  servo.ramWrite(addr, const_val, 2);
  if (servo.getLastError())
    connected = false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);

  Serial.println("Inicializado!");
}

void loop() {
  if (!connected) {
    digitalWrite(LED_BUILTIN, HIGH);
    servo.readStatus();
    if (servo.getLastError())
      return;

    servo.ramWrite(18, overload_treshold, 2);
    if (servo.getLastError())
      return;

    connected = true;
    Serial.flush();
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (Serial.available())
    handle_serial();

  uint16_t pos = generate_sine_positions(400, 600, 0.3);
  servo.setPosition(pos);
  if (servo.getLastError())
    connected = false;
}

