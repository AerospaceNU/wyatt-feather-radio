#ifndef DYNAMIXEL_COMMAND_QUEUE_H_
#define DYNAMIXEL_COMMAND_QUEUE_H_

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <wiring_private.h>

#include "cpp_circular_buffer.h"

#define UART_PIN 10

class DynamixelCommandQueue {
 public:
  explicit DynamixelCommandQueue(Uart* uart)
      : m_uart{uart}, m_isAwaitResponse{false} {}

  uint8_t sendMessage(uint8_t* buffer, uint16_t writeLength,
                      uint8_t* callbackBuffer) {
    if (m_messageBuffer.full()) {
      return 1;
    } else {
      CommandData newCommand;
      memcpy(&(newCommand.message), buffer, writeLength);
      newCommand.messageSize = writeLength;
      newCommand.responseBuffer = callbackBuffer;
      m_messageBuffer.enqueue(newCommand);
      return 0;
    }
  }

  void processReadData(void*, size_t size) { m_isAwaitResponse = false; }

  void tick() {
    if (m_isAwaitResponse) {
      size_t idx = 0;
      while (m_uart->available() && idx < kMaxBufferSize) {
        m_currentMessageBuffer[idx++] = (uint8_t)m_uart->read();
      }
      Serial.print("Received ");
      Serial.print(idx);
      Serial.println(" bytes");
      processReadData(nullptr, idx);
    }

    if (m_messageBuffer.count() == 0) {
      return;
    }

    if (m_isAwaitResponse &&
        (millis() - m_prevSendTimeMs < kResponseTimeoutMs)) {
      return;
    }
    CommandData currentCommand;
    m_messageBuffer.peek(&currentCommand, 1);
    m_messageBuffer.dequeue(1);
    m_currentMessageBuffer = currentCommand.responseBuffer;
    pinMode(UART_PIN, OUTPUT);
    Serial.println("About to send");
    Serial1.write((uint8_t*)&(currentCommand.message),
                  currentCommand.messageSize);
    //m_uart->flush();
    Serial.print("Wrote ");
    Serial.print(currentCommand.messageSize);
    Serial.println("bytes to motors");
    m_prevSendTimeMs = millis();
    m_isAwaitResponse = false;
    pinMode(UART_PIN, INPUT);
  }

 private:
  static const constexpr uint32_t kMaxBufferSize = 256;
  static const constexpr uint32_t kMaxMessageCount = 100;
  static const constexpr uint32_t kResponseTimeoutMs = 5000;

  struct CommandData {
    uint8_t message[kMaxBufferSize];
    uint16_t messageSize;
    uint8_t* responseBuffer;
  };

  CircularBuffer<CommandData, 20> m_messageBuffer;
  Uart* m_uart;
  uint8_t* m_currentMessageBuffer;
  bool m_isAwaitResponse;
  uint32_t m_prevSendTimeMs;
};

#endif /* DYNAMIXEL_COMMAND_QUEUE_H_ */
