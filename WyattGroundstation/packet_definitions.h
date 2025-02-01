#ifndef PACKET_DEFINITIONS_H_
#define PACKET_DEFINITIONS_H_

#include <inttypes.h>

#include "crc.h"

#define PYRO_DUP_COUNT 10

enum PacketType {
  Echo = 0,
  MotorControl = 1,
  PyroTrigger = 2,
};

typedef struct __attribute__((packed)) {
  float currentMotor1SettingAbsoluteDegrees;
  float currentMotor2SettingAbsoluteDegrees;
} EchoPacket;

typedef struct __attribute__((packed)) {
  float motor1SettingAbsoluteDegrees;
  float motor2SettingAbsoluteDegrees;
} MotorControlPacket;

typedef struct __attribute__((packed)) {
  uint32_t pyroToTrigger[PYRO_DUP_COUNT];
} PyroTriggerPacket;

union PacketPayload {
  EchoPacket echo;
  MotorControlPacket motorControl;
  PyroTriggerPacket pyroTrigger;
};

typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  uint32_t payload_length;
  uint8_t packet_type;
  PacketPayload payload;
  uint16_t crc;
} RadioPacket;

typedef struct __attribute__((packed)) {
  uint32_t timestamp_groundstation;
  double rssi;
  RadioPacket payload;
} GroundstationPacket;

const constexpr size_t kRadioPacketSize = sizeof(RadioPacket);

void makeCRC(RadioPacket* pkt) {
  size_t data_len = kRadioPacketSize - sizeof(pkt->crc);
  uint16_t crc = getCRC(0, (uint8_t*)pkt, data_len);
  pkt->crc = crc;
}

bool validCRC(RadioPacket* pkt) {
  size_t data_len = kRadioPacketSize - sizeof(pkt->crc);
  uint16_t crc = getCRC(0, (uint8_t*)pkt, data_len);
  return pkt->crc == crc;
}

void printPacket(RadioPacket* pkt) {
  if (!validCRC(pkt)) {
    Serial.println("Bad crc!");
    return;
  }
  Serial.print("Packet: Timestamp = ");
  Serial.print(pkt->timestamp_ms);
  Serial.print(", Payload Length = ");
  Serial.print(pkt->payload_length);
  if (pkt->packet_type == PacketType::Echo) {
    Serial.println(", Type = ECHO");
    Serial.print("Motor 1 Setting: ");
    Serial.print(pkt->payload.echo.currentMotor1SettingAbsoluteDegrees);
    Serial.print("Motor 2 Setting: ");
    Serial.println(pkt->payload.echo.currentMotor2SettingAbsoluteDegrees);
  } else if (pkt->packet_type == PacketType::MotorControl) {
    Serial.println(", Type = MOTORCONTROL");
    Serial.print("Motor 1 Setting: ");
    Serial.print(pkt->payload.motorControl.motor1SettingAbsoluteDegrees);
    Serial.print("Motor 2 Setting: ");
    Serial.println(pkt->payload.motorControl.motor2SettingAbsoluteDegrees);
  } else if (pkt->packet_type == PacketType::PyroTrigger) {
    Serial.println(", Type = PYROTRIGGER");
    uint32_t pyro_val = pkt->payload.pyroTrigger.pyroToTrigger[0];
    bool allMatch = true;
    for (int i = 1; i < PYRO_DUP_COUNT; ++i) {
      if (pkt->payload.pyroTrigger.pyroToTrigger[i] != pyro_val) {
        allMatch = false;
        break;
      }
    }
    if (allMatch) {
      Serial.print("Valid request for pyro ");
      Serial.println(pyro_val);
    } else {
      Serial.println("Invalid request! Non-matching pyros.");
    }
  }
}

#endif  // PACKET_DEFINITIONS_H_