#include <Arduino.h>
#include <DMAInterrupts.h>
#include <DMASerial.h>
#include <ServoManager.h>
#include <config.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <utils/MockController.h>

uint8_t pos_cmd[SJOG_SIZE];
ServoManager manager;
// TODO: Resolver qual função define o canal
//  ou se o canal é definido pela USART passada no init
DMASerial servo_serial(DMA1, DMA_CH2, SJOG_SIZE, DMA_IRQ_HANDLER_1);

ros::NodeHandle nh;
time_t last_spin;

void ros_callback(const std_msgs::Int16MultiArray& msg) {
  if (msg.data_length != NUM_SERVOS)
    return;

  for (uint8_t i = 0; i < NUM_SERVOS; i++)
    manager.set_position(i + 1, msg.data[i]);
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("Bioloid/joint_pos_int",
                                               ros_callback);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(500000);
  Serial1.begin(115200);
  Serial2.begin(115200);

  delay(INITIAL_DELAY);
  digitalWrite(LED_BUILTIN, LOW);

  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(sub);
  last_spin = millis();

  servo_serial.init(USART3, DMA_REQ_SRC_USART3_TX);
}

void loop() {
  // MockController::generate_sine_positions(manager, -100, 100, 0.5);

  if (!servo_serial.is_transfering()) {
    manager.assemble_pos_cmd(pos_cmd);
    servo_serial.set_data(pos_cmd, SJOG_SIZE);
    servo_serial.start();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  if (millis() - last_spin >= SPIN_PERIOD) {
    nh.spinOnce();
    last_spin = millis();
  }
}
