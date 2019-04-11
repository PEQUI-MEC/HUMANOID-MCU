#include <BodyServo.h>

BodyServo::BodyServo(uint8_t id,
                     int16_t position,
                     int16_t offset,
                     bool reverse) {
  this->id = id;
  this->position = position;
  this->offset = offset;
  this->reverse = reverse;
}

BodyServo::~BodyServo() {}

uint8_t BodyServo::get_id(void) {
  return id;
}

int16_t BodyServo::get_position(void) {
  return position;
}

uint16_t BodyServo::get_abs_position(void) {
  // TODO: Implementar mapeamento corretamente
  return range_map(position, POS_MIN, POS_MAX, XYZ_POS_MIN, XYZ_POS_MAX);
}

void BodyServo::set_position(int16_t position) {
  this->position = position;
}
