#include <RadioLib.h>
#include <SPI.h>

#include "dynamixel_command_queue.h"
#include "dynamixel_motor.h"
#include "packet_definitions.h"

#define NUM_DYNAMIXEL 2

const uint32_t camera_pin = 5;
const uint32_t camera_off_flag = 6;
bool camera_on = false;

// ms between echo packets
#define ECHO_INTERVAL 500
uint32_t last_echo_time = 0;

const uint32_t pyro_1_pin = 12;
const uint32_t pyro_2_pin = 9;
const uint32_t pyro_fire_time = 1000;
uint32_t pyro_1_start_time;
uint32_t pyro_2_start_time;
bool pyro_1_is_firing = false;
bool pyro_2_is_firing = false;

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool radioOperationDone = false;

static RadioPacket radioPacketRx;
static RadioPacket radioPacketTx;
void setFlag(void) { radioOperationDone = true; }

float motorPosition[NUM_DYNAMIXEL];
static DynamixelMotor dynamixelMotor[NUM_DYNAMIXEL];
static int dynamixelId[NUM_DYNAMIXEL] = {1, 2};
Uart motorUart(&sercom5, UART_PIN, UART_PIN, SERCOM_RX_PAD_1, UART_TX_PAD_0);
static DynamixelCommandQueue dynamixelCommandQueue{&motorUart};

SX1276 radio = new Module(8, 3, 4);  // (CS, INT, RST)
int count;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(pyro_1_pin, OUTPUT);
  pinMode(pyro_2_pin, OUTPUT);
  pinMode(camera_pin, OUTPUT);
  digitalWrite(pyro_1_pin, LOW);
  digitalWrite(pyro_2_pin, LOW);
  digitalWrite(camera_pin, LOW);
  // pinPeripheral(UART_PIN, PIO_SERCOM);
  // motorUart.begin(115200);

  // Set initial mode: TX
  // pinMode(UART_PIN, OUTPUT);
  delay(100);
  SPI.begin();

  for (int i = 0; i < NUM_DYNAMIXEL; ++i) {
    dynamixelMotor[i].init(dynamixelId[i], &dynamixelCommandQueue);
    dynamixelMotor[i].profileAcceleration(0);
    dynamixelMotor[i].profileVelocity(0);
    dynamixelMotor[i].setOperatingMode(OperatingMode::EXT_POSITION);
    dynamixelMotor[i].torqueEnable(Toggle::ON);
    dynamixelMotor[i].setDriveMode(ProfileConfig::VELOCITY_BASED,
                                   DirectionMode::NORMAL);
  }

  // Initialize LoRa
  int state = radio.begin(915.0, 125.0, 7, 5);
  radio.setOutputPower(20);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("RFM95 initialized successfully!");
  } else {
    Serial.print("LoRa init failed! Error code: ");
    Serial.println(state);
    while (true);
  }

  radio.setDio0Action(setFlag, RISING);
  state = radio.startReceive();
  transmitFlag = false;
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) {
      delay(10);
    }
  }
  count = 0;
}

void handlePacket(RadioPacket* pkt) {
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
    motorPosition[0] = pkt->payload.motorControl.motor1SettingAbsoluteDegrees;
    motorPosition[1] = pkt->payload.motorControl.motor2SettingAbsoluteDegrees;
    dynamixelMotor[0].goalPosition( motorPosition[0]);
    dynamixelMotor[1].goalPosition( motorPosition[1]);
  } else if (pkt->packet_type == PacketType::PyroTrigger) {
    Serial.println(", Type = PYROTRIGGER");
    int read_val = pkt->payload.pyroTrigger.pyroToTrigger[0];
    bool allMatch = true;
    for (int i = 1; i < PYRO_DUP_COUNT; ++i) {
      if (pkt->payload.pyroTrigger.pyroToTrigger[i] != read_val) {
        allMatch = false;
        break;
      }
    }
    if (allMatch) {
      Serial.print("Valid request for pyro ");
      Serial.println(read_val);
      if (read_val == pyro_1_pin) {
        digitalWrite(pyro_1_pin, HIGH);
        pyro_1_start_time = millis();
        pyro_1_is_firing = true;
      } else if (read_val == camera_pin) {
        digitalWrite(camera_pin, HIGH);
        camera_on = true;
      } else if (read_val == camera_off_flag) {

        digitalWrite(camera_pin, LOW);
        camera_on = false;
      }

    } else {
      Serial.println("Invalid request! Non-matching pyros.");
    }
  }
}

void loop() {
  if (pyro_1_is_firing && millis() - pyro_1_start_time > pyro_fire_time) {
    digitalWrite(pyro_1_pin, LOW);
  }
  if (pyro_1_is_firing && millis() - pyro_1_start_time > 3000) {
    digitalWrite(pyro_2_pin, HIGH);
    pyro_1_is_firing = false;
    pyro_2_is_firing = true;
    pyro_2_start_time = millis();
  }
  if (pyro_2_is_firing && millis() - pyro_2_start_time > pyro_fire_time) {
    digitalWrite(pyro_2_pin, LOW);
    pyro_2_is_firing = false;
  }

  dynamixelCommandQueue.tick();

  if (radioOperationDone) {
    // reset flag
    radioOperationDone = false;

    if (transmitFlag) {
      // the previous operation was transmission, set back to receive state
      if (transmissionState == RADIOLIB_ERR_NONE) {
        // packet was successfully sent
        Serial.println(F("transmission finished!"));
      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);
      }

      // listen for new response
      radio.startReceive();
      transmitFlag = false;

    } else {
      // the previous operation was reception
      // print data
      int state = radio.readData((uint8_t*)&radioPacketRx, kRadioPacketSize);

      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("[SX1278] Received packet!"));

        // print data of the packet
        handlePacket(&radioPacketRx);

        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("[SX1278] RSSI:\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
      }
    }
  } else if (!transmitFlag) {
    // If we aren't waiting on a current transmission, allow it to transmit if
    // wanted
    if (millis() - last_echo_time > ECHO_INTERVAL) {
      last_echo_time = millis();
      radioPacketTx.timestamp_ms = millis();
      radioPacketTx.payload_length = 0;  // todo maybe update
      radioPacketTx.packet_type = PacketType::Echo;
      radioPacketTx.payload.echo.currentMotor1SettingAbsoluteDegrees =
          motorPosition[0];
      radioPacketTx.payload.echo.currentMotor2SettingAbsoluteDegrees =
          motorPosition[1];
      makeCRC(&radioPacketTx);
      transmissionState =
          radio.startTransmit((uint8_t*)&radioPacketTx, kRadioPacketSize);
      transmitFlag = true;
    }
  }
  delay(10);
}

// Required for SERCOM2
void SERCOM2_Handler() { motorUart.IrqHandler(); }