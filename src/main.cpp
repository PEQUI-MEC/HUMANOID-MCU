#include <Arduino.h>
#include <DMAInterrupts.h>
#include <DMASerial.h>

unsigned long last_led_change;

String str("Mensagem teste 0\n");
DMASerial serial(DMA1, DMA_CH4, str.length(), DMA_IRQ_HANDLER_1);

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  delay(5000);
  Serial.println("Inicializado!");

  serial.init(USART1, DMA_REQ_SRC_USART1_TX);
  last_led_change = millis();
}

void loop() {
  if (millis() - last_led_change >= 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    last_led_change = millis();
  }

  if (!serial.is_transfering()) {
    char c = str.charAt(str.length() - 2) + 1;
    if (c > '9')
      c = '0';
    str.setCharAt(str.length() - 2, c);
    serial.set_data(str);

    serial.start();
  }
}
