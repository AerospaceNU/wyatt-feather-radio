#include <RadioLib.h>
#include <SPI.h>

#include "packet_definitions.h"
#define ECHO_INTERVAL 2000

uint8_t packet_buffer[256];

uint32_t last_echo_time = 0;
// Create RFM95 instance
SX1276 radio = new Module(8, 3, 4);  // (CS, INT, RST)

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool radioOperationDone = false;

static RadioPacket radioPacketRx;
uint32_t IMPORTANT = 134135;
static RadioPacket radioPacketTx;

static GroundstationPacket groundstationPacket;

volatile bool radioTransmitPacketReady = false;

void setFlag(void) {
  // we sent or received  packet, set the flag
  radioOperationDone = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }
  SPI.begin();

  // Initialize LoRa
  int state = radio.begin(915.0, 125.0, 7, 5);
  radio.setOutputPower(20);
  if (state == RADIOLIB_ERR_NONE) {
    // Serial.println("RFM95 initialized successfully!");
  } else {
    // Serial.print("LoRa init failed! Error code: ");
    // Serial.println(state);
    while (true);
  }
  radio.setDio0Action(setFlag, RISING);
  state = radio.startReceive();
  transmitFlag = false;
  if (state == RADIOLIB_ERR_NONE) {
    // Serial.println(F("success!"));
  } else {
    // Serial.print(F("failed, code "));
    // Serial.println(state);
    while (true) {
      delay(10);
    }
  }
}

void loop() {
  // if(receivedFlag) {
  //   receivedFlag = false;
  //   int state = radio.readData((uint8_t*)&radioPacketRx, kRadioPacketSize);

  //   // you can also read received data as byte array
  //   /*
  //     byte byteArr[8];
  //     int numBytes = radio.getPacketLength();
  //     int state = radio.readData(byteArr, numBytes);
  //   */

  //   if (state == RADIOLIB_ERR_NONE) {
  //     // packet was successfully received
  //     Serial.println(F("Received packet!"));

  //     // print data of the packet
  //     printPacket(&radioPacketTx);

  //     // print RSSI (Received Signal Strength Indicator)
  //     Serial.print(F("[SX1278] RSSI:\t\t"));
  //     Serial.print(radio.getRSSI());
  //     Serial.println(F(" dBm"));

  //   } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
  //     // packet was received, but is malformed
  //     Serial.println(F("[SX1278] CRC error!"));
  //   } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {

  //   } else {
  //     // some other error occurred
  //     Serial.print(F("[SX1278] Failed, code "));
  //     Serial.println(state);
  //   }
  // }

  // if (millis() - last_echo_time > ECHO_INTERVAL) {
  //   radio.standby();
  //   radio.clearPacketReceivedAction();
  //   last_echo_time = millis();
  //   radioPacketTx.timestamp_ms = millis();
  //   radioPacketTx.payload_length = 0; // todo maybe update
  //   radioPacketTx.packet_type = PacketType::MotorControl;
  //   radioPacketTx.payload.motorControl.motor1SettingAbsoluteDegrees = 123;
  //   radioPacketTx.payload.motorControl.motor2SettingAbsoluteDegrees = -123;
  //   makeCRC(&radioPacketTx);
  //   int state = radio.transmit((uint8_t*)&radioPacketTx, kRadioPacketSize);
  //   radio.finishTransmit();

  //   if (state == RADIOLIB_ERR_NONE) {
  //     // the packet was successfully transmitted
  //     Serial.println(F(" transmit success!"));

  //     // print measured data rate
  //     Serial.print(F("[SX1278] Datarate:\t"));
  //     Serial.print(radio.getDataRate());
  //     Serial.println(F(" bps"));

  //   } else {
  //     Serial.println("bad state");
  //   }
  //   // After transmitting, flush and switch to receiving mode again
  //   radio.standby();  // Put radio in idle mode
  //   delay(10);  // Wait a small moment before resuming receive
  //   radio.startReceive();  // Start receiving again
  // }

  if (Serial.available() > 0) {
    int index = 0;

    // Read the bytes into the buffer
    while (Serial.available() > 0 && index < sizeof(packet_buffer)) {
      packet_buffer[index] = Serial.read();
      index++;
    }

    GroundstationPacket* recv_pkt = (GroundstationPacket*)(packet_buffer);
    radioPacketTx = recv_pkt->payload;
    if (validCRC(&radioPacketTx)) {
     radioTransmitPacketReady = true;
    }
  }

  if (radioOperationDone) {
    // reset flag
    radioOperationDone = false;

    if (transmitFlag) {
      // the previous operation was transmission, listen for response
      // print the result
      if (transmissionState == RADIOLIB_ERR_NONE) {
        // packet was successfully sent
        // Serial.println(F("transmission finished!"));

      } else {
        // Serial.print(F("failed, code "));
        //  Serial.println(transmissionState);
      }

      // listen for response
      radio.startReceive();
      transmitFlag = false;

    } else {
      // the previous operation was reception
      // print data and send another packet
      int state = radio.readData((uint8_t*)&radioPacketRx, kRadioPacketSize);

      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        // Serial.println(F("[SX1278] Received packet!"));

        // print data of the packet
        // printPacket(&radioPacketRx);
        groundstationPacket.timestamp_groundstation = millis();
        groundstationPacket.rssi = radio.getRSSI();
        groundstationPacket.payload = radioPacketRx;
        Serial.write((uint8_t*)&groundstationPacket,
                     sizeof(groundstationPacket));

        // print RSSI (Received Signal Strength Indicator)
        // Serial.print(F("[SX1278] RSSI:\t\t"));
        // Serial.print(radio.getRSSI());
        // Serial.println(F(" dBm"));
      }
    }
  } else if (!transmitFlag) {
    if (radioTransmitPacketReady) {
      radioTransmitPacketReady = false;
      radioPacketTx.timestamp_ms = millis();
      radioPacketTx.payload_length = 0;  // todo maybe update
      makeCRC(&radioPacketTx);
      transmissionState =
          radio.startTransmit((uint8_t*)&radioPacketTx, kRadioPacketSize);
      transmitFlag = true;
    }
  }
  delay(10);
}