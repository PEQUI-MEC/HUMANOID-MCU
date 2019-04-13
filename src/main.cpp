#include <Arduino.h>
#include <DMAInterrupts.h>
#include <DMASerial.h>
#include <ServoManager.h>
#include <config.h>
#include <utils/MockController.h>
// #include <ros.h>
// #include <std_msgs/Int16MultiArray.h>

uint8_t pos_cmd[POS_CMD_SIZE];
ServoManager manager;
DMASerial servo_serial(DMA1, DMA_CH4, POS_CMD_SIZE, DMA_IRQ_HANDLER_1);

// void ros_callback(const std_msgs::Int16MultiArray& msg) {
//   for (uint8_t i = 0; i < msg.data_length; i++)
//     manager.set_position(i + 1, msg.data[i]);
// }

// ros::NodeHandle nh;
// ros::Subscriber<std_msgs::Int16MultiArray> sub("msg_name", ros_callback);
// uint32_t last_spin;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(115200);

  delay(INITIAL_DELAY);
  Serial.println("Inicializado!");
  digitalWrite(LED_BUILTIN, HIGH);

  // nh.initNode();
  // nh.subscribe(sub);
  servo_serial.init(USART1, DMA_REQ_SRC_USART1_TX);
}

void loop() {
  MockController::generate_sine_positions(manager, -300, 300);

  if (!servo_serial.is_transfering()) {
    manager.assemble_pos_cmd(pos_cmd);
    Serial.println(manager.get_servo(1).get_position());
    Serial.println(manager.get_servo(1).get_abs_position());
    Serial.println();
    // servo_serial.set_data(pos_cmd, POS_CMD_SIZE);
    // servo_serial.start();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
  }

  // if (millis() - last_spin >= SPIN_PERIOD) {
  //   nh.spinOnce();
  //   last_spin = millis();
  // }
}
