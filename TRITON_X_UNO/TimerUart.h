/********** TRITON-X fixed-pin timer UART
 * Purpose
 *   Implements a 9600 baud, 8N1 software UART on Arduino D4 and D13.
 * Hardware
 *   ESP32 GPIO13 TX connects to Arduino D4 RX. Arduino D13 TX connects
 *   to ESP32 GPIO14 RX through a 5 V-to-3.3 V divider.
 * Software
 *   Timer2 samples RX and generates TX. This avoids SoftwareSerial,
 *   which conflicts with the D10/D12 encoder pin-change interrupt.
 * Limitations
 *   Fixed at 9600 baud. Timer2 must not be used by another library.
 * Reference
 *   v4.0 Final, derived from MAPPING v1.4, C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_TIMER_UART_H
#define TRITON_X_TIMER_UART_H

#include <Arduino.h>
#include <avr/interrupt.h>

const uint8_t UART_RX_BUFFER_SIZE = 32;
const uint8_t UART_TX_BUFFER_SIZE = 64;
const uint8_t UART_RX_MASK = UART_RX_BUFFER_SIZE - 1;
const uint8_t UART_TX_MASK = UART_TX_BUFFER_SIZE - 1;
const uint8_t UART_TICKS_PER_BIT = 8;
const uint8_t UART_FIRST_SAMPLE_TICKS = 12;

volatile uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uartRxHead = 0;
volatile uint8_t uartRxTail = 0;
volatile uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];
volatile uint8_t uartTxHead = 0;
volatile uint8_t uartTxTail = 0;

volatile bool uartRxActive = false;
volatile uint8_t uartRxCountdown = 0;
volatile uint8_t uartRxBitIndex = 0;
volatile uint8_t uartRxByte = 0;
volatile bool uartTxActive = false;
volatile uint8_t uartTxCountdown = 0;
volatile uint8_t uartTxBitIndex = 0;
volatile uint16_t uartTxFrame = 0;

inline bool readUartRxPin(void) {
  return (PIND & _BV(PD4)) != 0;
}

inline void writeUartTxPin(bool high) {
  if (high) {
    PORTB |= _BV(PB5);
  } else {
    PORTB &= ~_BV(PB5);
  }
}

ISR(TIMER2_COMPA_vect) {
  bool rxHigh = readUartRxPin();

  if (!uartRxActive) {
    if (!rxHigh) {
      uartRxActive = true;
      uartRxCountdown = UART_FIRST_SAMPLE_TICKS;
      uartRxBitIndex = 0;
      uartRxByte = 0;
    }
  } else {
    if (uartRxCountdown > 0) {
      uartRxCountdown--;
    }

    if (uartRxCountdown == 0) {
      if (uartRxBitIndex < 8) {
        if (rxHigh) {
          uartRxByte |= (uint8_t)(1U << uartRxBitIndex);
        }
        uartRxBitIndex++;
        uartRxCountdown = UART_TICKS_PER_BIT;
      } else {
        if (rxHigh) {
          uint8_t nextHead =
            (uint8_t)((uartRxHead + 1U) & UART_RX_MASK);
          if (nextHead != uartRxTail) {
            uartRxBuffer[uartRxHead] = uartRxByte;
            uartRxHead = nextHead;
          }
        }
        uartRxActive = false;
      }
    }
  }

  if (uartTxActive) {
    if (uartTxCountdown > 0) {
      uartTxCountdown--;
    }

    if (uartTxCountdown == 0) {
      uartTxBitIndex++;
      if (uartTxBitIndex >= 10) {
        writeUartTxPin(true);
        uartTxActive = false;
      } else {
        bool bitHigh =
          (uartTxFrame & (uint16_t)(1U << uartTxBitIndex)) != 0;
        writeUartTxPin(bitHigh);
        uartTxCountdown = UART_TICKS_PER_BIT;
      }
    }
  } else if (uartTxTail != uartTxHead) {
    uint8_t outgoing = uartTxBuffer[uartTxTail];
    uartTxTail = (uint8_t)((uartTxTail + 1U) & UART_TX_MASK);
    uartTxFrame = ((uint16_t)1U << 9) | ((uint16_t)outgoing << 1);
    uartTxBitIndex = 0;
    uartTxCountdown = UART_TICKS_PER_BIT;
    writeUartTxPin(false);
    uartTxActive = true;
  }
}

/***** class FixedTimerUart
 * Purpose
 *   Provides the Arduino Print and Stream-like operations required by
 *   the integrated command and telemetry protocol.
 * Methods
 *   begin() configures Timer2 and the fixed pins.
 *   available() returns buffered RX bytes.
 *   read() removes one RX byte.
 *   write() queues one TX byte.
 * Hardware
 *   Arduino D4 RX and D13 TX.
 * Software
 *   Uses ring buffers and the TIMER2_COMPA interrupt.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
class FixedTimerUart : public Print {
  public:
    void begin(unsigned long baudRate) {
      (void)baudRate;
      pinMode(ESP_RX_PIN, INPUT_PULLUP);
      pinMode(ESP_TX_PIN, OUTPUT);
      digitalWrite(ESP_TX_PIN, HIGH);

      uint8_t oldSreg = SREG;
      cli();
      uartRxHead = 0;
      uartRxTail = 0;
      uartTxHead = 0;
      uartTxTail = 0;
      uartRxActive = false;
      uartTxActive = false;
      TCCR2A = 0;
      TCCR2B = 0;
      TCNT2 = 0;
      TCCR2A = _BV(WGM21);
      OCR2A = 25;
      TCCR2B = _BV(CS21);
      TIMSK2 |= _BV(OCIE2A);
      SREG = oldSreg;
    }

    int available(void) {
      uint8_t head = uartRxHead;
      uint8_t tail = uartRxTail;
      return (uint8_t)((head - tail) & UART_RX_MASK);
    }

    int read(void) {
      if (uartRxHead == uartRxTail) {
        return -1;
      }
      uint8_t value = uartRxBuffer[uartRxTail];
      uartRxTail = (uint8_t)((uartRxTail + 1U) & UART_RX_MASK);
      return value;
    }

    size_t write(uint8_t value) override {
      uint8_t nextHead =
        (uint8_t)((uartTxHead + 1U) & UART_TX_MASK);
      while (nextHead == uartTxTail) {
        /* Timer2 continues transmitting while the foreground waits. */
      }
      uartTxBuffer[uartTxHead] = value;
      uartTxHead = nextHead;
      return 1;
    }

    using Print::write;
};

#endif
