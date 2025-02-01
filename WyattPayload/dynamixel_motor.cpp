#include "dynamixel_motor.h"

#include <cmath>
#include <functional>
#include <iostream>

#include "crc.h"

const double DEGREES_TO_POS_UNIT = 1 / 0.088;
const double RPM_TO_VEL_UNIT = 1 / 0.229;
const double RPM2_TO_ACC_UNIT = 1 / 214.577;

bool DynamixelMotor::init(const uint8_t id,
                          DynamixelCommandQueue* commandQueue) {
  m_id = id;
  m_commandQueue = commandQueue;
  return true;
}

uint8_t DynamixelMotor::clearPosition() {
  m_txPacket.length_l = 0x08;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x10;

  // Clear position
  // (as opposed to clearing errors which I am not implementing)
  m_txPacket.payload[0] = 0x01;
  // Fixed values for this instruction
  m_txPacket.payload[1] = 0x44;
  m_txPacket.payload[2] = 0x58;
  m_txPacket.payload[3] = 0x4c;
  m_txPacket.payload[4] = 0x22;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::ping() {
  m_txPacket.length_l = 0x03;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x01;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::torqueEnable(Toggle toggle) {
  m_txPacket.length_l = 0x06;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x03;

  // Torque enable write location
  m_txPacket.payload[0] = 0x40;
  m_txPacket.payload[1] = 0x00;

  m_txPacket.payload[2] = toggle;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::setDriveMode(ProfileConfig profileConfig,
                                     DirectionMode direction) {
  uint8_t profileBit = profileConfig << 2;
  uint8_t directionBit = direction;
  uint8_t mode = 0x00 | profileBit | directionBit;

  m_txPacket.length_l = 0x06;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x03;

  // Drive mode write location
  m_txPacket.payload[0] = 0x0a;
  m_txPacket.payload[1] = 0x00;

  m_txPacket.payload[2] = mode;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::setOperatingMode(OperatingMode mode) {
  m_txPacket.length_l = 0x06;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x03;

  // Operating mode write location
  m_txPacket.payload[0] = 0x0b;
  m_txPacket.payload[1] = 0x00;

  m_txPacket.payload[2] = mode;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::goalPosition(double degrees) {
  int32_t degreeConversion = (int32_t)(degrees * DEGREES_TO_POS_UNIT);

  m_txPacket.length_l = 0x09;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x03;

  // Goal position write location
  m_txPacket.payload[0] = 0x74;
  m_txPacket.payload[1] = 0x00;

  m_txPacket.payload[2] = degreeConversion & 0xff;
  m_txPacket.payload[3] = (degreeConversion >> 8) & 0xff;
  m_txPacket.payload[4] = (degreeConversion >> 16) & 0xff;
  m_txPacket.payload[5] = (degreeConversion >> 24) & 0xff;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::profileVelocity(double rpm) {
  uint32_t rpmConversion = (uint32_t)(rpm * RPM_TO_VEL_UNIT);

  m_txPacket.length_l = 0x09;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x03;

  // Profile velocity write location
  m_txPacket.payload[0] = 0x70;
  m_txPacket.payload[1] = 0x00;

  m_txPacket.payload[2] = rpmConversion & 0xff;
  m_txPacket.payload[3] = (rpmConversion >> 8) & 0xff;
  m_txPacket.payload[4] = (rpmConversion >> 16) & 0xff;
  m_txPacket.payload[5] = (rpmConversion >> 24) & 0xff;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::profileAcceleration(double rpm2) {
  uint32_t rpm2Conversion = (uint32_t)(rpm2 * RPM2_TO_ACC_UNIT);

  m_txPacket.length_l = 0x09;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x03;

  // Profile velocity write location
  m_txPacket.payload[0] = 0x6c;
  m_txPacket.payload[1] = 0x00;

  m_txPacket.payload[2] = rpm2Conversion & 0xff;
  m_txPacket.payload[3] = (rpm2Conversion >> 8) & 0xff;
  m_txPacket.payload[4] = (rpm2Conversion >> 16) & 0xff;
  m_txPacket.payload[5] = (rpm2Conversion >> 24) & 0xff;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::reboot() {
  m_txPacket.length_l = 0x03;
  m_txPacket.length_h = 0x00;
  m_txPacket.instruction = 0x08;

  this->write(m_txPacket);
  return 0;
}

uint8_t DynamixelMotor::processReadData(uint16_t size) { return 0; }

uint8_t DynamixelMotor::write(DynamixelPacket_t& buf) {
  uint8_t writeLength = prepareTxPacket();

  m_commandQueue->sendMessage((uint8_t*)(&buf), writeLength,
                              (uint8_t*)(&m_rxPacket));
  return 0;
}

uint16_t DynamixelMotor::prepareTxPacket() {
  m_txPacket.header[0] = 0xff;
  m_txPacket.header[1] = 0xff;
  m_txPacket.header[2] = 0xfd;
  m_txPacket.header[3] = 0x00;
  m_txPacket.id = m_id;

  // Minus 3 bytes of instruction and CRC
  uint16_t payloadLength = (m_txPacket.length_h << 8) + m_txPacket.length_l - 3;
  // 4 bytes header, 1 byte id, instruction, 2 bytes length, CRC
  uint16_t packetLength = payloadLength + 10;
  // Calculate CRC and put high and low byte at end of payload
  uint16_t crc = getCRC(0, (uint8_t*)&m_txPacket, packetLength - 2);
  m_txPacket.payload[payloadLength] = crc & 0x00ff;
  m_txPacket.payload[payloadLength + 1] = crc >> 8;

  // 4 bytes header, 1 byte id, instruction, 2 bytes length, CRC
  return payloadLength + 10;
}
