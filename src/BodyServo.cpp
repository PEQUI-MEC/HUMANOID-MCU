#include <BodyServo.h>

BodyServo::BodyServo(uint8_t cid,
                     uint8_t rid,
                     int16_t position,
                     int16_t offset,
                     bool reverse) {
  this->cid = cid;
  this->rid = rid;
  this->position = position;
  this->offset = offset;
  this->reverse = reverse;
}

BodyServo::~BodyServo() {}

uint8_t BodyServo::get_cid(void) {
  return cid;
}

uint8_t BodyServo::get_rid(void) {
  return rid;
}

int16_t BodyServo::get_position(void) {
  return position;
}

uint16_t BodyServo::get_abs_position(void) {
  int16_t real = position + offset;
  if (reverse)
    real = -real;

  return range_map(real, POS_MIN, POS_MAX, XYZ_POS_MIN, XYZ_POS_MAX);
}

void BodyServo::set_position(int16_t position) {
  this->position = position;
}
