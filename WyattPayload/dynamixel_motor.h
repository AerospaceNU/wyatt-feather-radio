#ifndef DYNAMIXEL_MOTOR_H_
#define DYNAMIXEL_MOTOR_H_

#include <stdint.h>

#include "dynamixel_command_queue.h"

enum Toggle : uint8_t {
  OFF = 0,
  ON = 1,
};

enum ProfileConfig : uint8_t {
  VELOCITY_BASED = 0,
  TIME_BASED = 1,
};

enum DirectionMode : uint8_t {
  NORMAL = 0,
  REVERSE = 1,
};

enum OperatingMode : uint8_t {
  CURRENT = 0,
  VELOCITY = 1,
  POSITION = 3,
  EXT_POSITION = 4,
};

class DynamixelMotor {
 public:
  DynamixelMotor() {}

  bool init(const uint8_t id, DynamixelCommandQueue* commandQueue);

  static const constexpr uint32_t kMaxPayloadSize = 50;
  struct DynamixelPacket_t {
    uint8_t header[4];
    uint8_t id;
    uint8_t length_l;
    uint8_t length_h;
    uint8_t instruction;
    uint8_t payload[kMaxPayloadSize + 2];  // +2 for CRC space;
  };

  uint8_t clearPosition();

  uint8_t ping();

  uint8_t torqueEnable(Toggle toggle);

  uint8_t setDriveMode(ProfileConfig profileConfig, DirectionMode direction);

  uint8_t setOperatingMode(OperatingMode mode);

  uint8_t goalPosition(double degrees);

  uint8_t profileVelocity(double rpm);

  uint8_t profileAcceleration(double rpm2);

  uint8_t reboot();

 private:
  uint8_t m_id = 0;
  DynamixelCommandQueue* m_commandQueue = nullptr;
  DynamixelPacket_t m_txPacket = {};
  DynamixelPacket_t m_rxPacket = {};

  uint8_t processReadData(uint16_t size);

  uint8_t write(DynamixelPacket_t& buf);

  uint16_t prepareTxPacket();
};

#endif  // DYNAMIXEL_MOTOR_H_
