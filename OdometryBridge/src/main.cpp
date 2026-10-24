#include <Arduino.h>
#include <goBILDA_Pinpoint.h>
#include <writing.hpp>
#include <reading.hpp>
#include <numbers>

goBILDA::Pinpoint pinpoint = goBILDA::Pinpoint();

void setup()
{
  // port
  Serial.begin(115200);

  // configurating pinpoint
  pinpoint.begin();
  pinpoint.setEncoderDirections(goBILDA::EncoderDirection::Backward, goBILDA::EncoderDirection::Forward);
  pinpoint.setOffsets(0.0f, 0.0f);

  pinpoint.setBulkReadScope({
    goBILDA::PinpointRegisters::EncoderValueX,
    goBILDA::PinpointRegisters::EncoderValueY,
    goBILDA::PinpointRegisters::PositionX,
    goBILDA::PinpointRegisters::PositionY,
    goBILDA::PinpointRegisters::QuaternionW,
    goBILDA::PinpointRegisters::QuaternionX,
    goBILDA::PinpointRegisters::QuaternionY,
    goBILDA::PinpointRegisters::QuaternionZ,
    goBILDA::PinpointRegisters::VelocityX,
    goBILDA::PinpointRegisters::VelocityY,
    goBILDA::PinpointRegisters::VelocityH
  });

      // this might change
      constexpr float diamater_mm = 38.0f;
  constexpr float circumference_mm = diamater_mm * std::numbers::pi;
  constexpr int ticks_per_revolution = 4069;

  constexpr float ticks_per_mm = ticks_per_revolution / circumference_mm; // 760 ticks per revolution / circumference_mm

  pinpoint.setEncoderResolution(ticks_per_mm);

  delay(2000);
}

bool test = false;

void loop()
{
  while (rp2040.fifo.available()) {
    reset_data* data_ptr = reinterpret_cast<reset_data*>(rp2040.fifo.pop());

    if (data_ptr->calibrate_imu) {
      pinpoint.recalibrateIMU();
    }

    if (data_ptr->set_position) {
      pinpoint.setPosition(goBILDA::Pose2D(data_ptr->position_x, data_ptr->position_y, data_ptr->position_heading));
    }

    delete data_ptr;
  }

  goBILDA::BulkReadData bulk_data = pinpoint.bulkRead();

  p_data position_data = p_data();
  position_data.encoder_x = bulk_data.EncoderX;
  position_data.encoder_y = bulk_data.EncoderY;
  position_data.position_x = bulk_data.Position.x;
  position_data.position_y = bulk_data.Position.y;
  position_data.quaternion_w = bulk_data.quaternion.w;
  position_data.quaternion_x = bulk_data.quaternion.x;
  position_data.quaternion_y = bulk_data.quaternion.y;
  position_data.quaternion_z = bulk_data.quaternion.z;
  position_data.velocity_x = bulk_data.VelocityX;
  position_data.velocity_y = bulk_data.VelocityY;
  position_data.velocity_heading = bulk_data.VelocityH;

  // converts data into a buffer of bytes.

  Buffer payload = position_data.to_bytes();

  Buffer buffer = create_frame(payload);
  size_t buffer_size = buffer.size();

  // sends data and waits for it to complete.
  // Serial.println("Buffer first " + String(buffer.pop()));
  Serial.write(buffer.data(), buffer_size);

  delay(5);
}

void setup1()
{
  while (!Serial) {
    delay(50);
  }

  SerialBuffer buffer = SerialBuffer(Serial);

  while (true) {
    Buffer payload;

    if (read_frame(buffer, payload)) {
      reset_data* data_ptr = new reset_data(reset_data::from_bytes(payload));

      rp2040.fifo.push(reinterpret_cast<uint32_t>(data_ptr));
    }

    delay(10);
  }
}

void loop1() {}