#include <Arduino.h>
#include <goBILDA_Pinpoint.h>
#include "utilities.hpp"

// I want perfect memory management so it can send it fast.

goBILDA::Pinpoint pinpoint = goBILDA::Pinpoint();

mutex_t pinpoint_mtx = mutex();
mutex_t writing_serial_mtx = mutex();
mutex_t reading_serial_mtx = mutex();

struct p_data
{
  float position_x = 0.0;
  float position_y = 0.0;
  float position_heading = 0.0;

  float velocity_x = 0.0;
  float velocity_y = 0.0;
  float velocity_heading = 0.0;

  std::vector<uint8_t> to_bytes()
  {
    std::vector<uint8_t> bytes;
    bytes.reserve(sizeof(float) * 6);

    push_to_buffer(bytes, position_x);
    push_to_buffer(bytes, position_y);
    push_to_buffer(bytes, position_heading);
    push_to_buffer(bytes, velocity_x);
    push_to_buffer(bytes, velocity_y);
    push_to_buffer(bytes, velocity_heading);

    return bytes;
  }
};

void setup()
{
  // port
  Serial.begin(115200);

  // configurating pinpoint
  pinpoint.begin();
  pinpoint.setEncoderDirections(goBILDA::EncoderDirection::Forward, goBILDA::EncoderDirection::Forward);

  // this might change
  pinpoint.setEncoderResolution(goBILDA::EncoderResolution::goBILDA_4_BAR_POD);

  // inits thread safe device, mutex look it up fool.
  mutex_init(&writing_serial_mtx);

  mutex_init(&reading_serial_mtx);

  mutex_init(&pinpoint_mtx);
}

// reading
void loop()
{
  mutex_enter_blocking(&reading_serial_mtx);

  mutex_exit(&reading_serial_mtx);

  sleep_ms(1);
}

void loop1()
{
  mutex_enter_blocking(&pinpoint_mtx);
  goBILDA::Pose2D position = pinpoint.getPosition();
  p_data position_data = p_data();

  position_data.position_x = position.x;
  position_data.position_y = position.y;
  position_data.position_heading = position.heading;

  position_data.velocity_x = pinpoint.getVelocityX();
  position_data.velocity_y = pinpoint.getVelocityY();
  position_data.velocity_heading = pinpoint.getVelocityHeading();

  mutex_exit(&pinpoint_mtx);

  // converts data into a buffer of bytes.

  std::vector<uint8_t> buffer = position_data.to_bytes();
  size_t buffer_size = buffer.size();

  mutex_enter_blocking(&writing_serial_mtx);

  // sends data and waits for it to complete.
  Serial.write(buffer.data(), buffer_size);

  mutex_exit(&writing_serial_mtx);

  sleep_ms(1);
}