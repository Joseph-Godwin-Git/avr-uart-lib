/**
 * @file uart.h
 * @brief UART driver interface for AVR microcontrollers.
 *
 * Provides a templated UART interface with configurable register mappings
 * for supported AVR devices:
 *   - ATmega324PA
 *   - ATmega324PB
 *   - ATmega16
 *
 * Supports UART0 on all devices and UART1 where available.
 *
 * Part of the avr-uart-lib project:
 * https://github.com/Joseph-Godwin-Git/avr-uart-lib
 *
 * @author Joseph Godwin
 * @date 2025-05-29
 * @license MIT
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ringbuf.h"

#if defined(__AVR_ATmega16__)
#define REG(x) _SFR_IO8(x)
#elif defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega324PB__)
#define REG(x) _SFR_MEM8(x)
#endif

#define DEFAULT_BUFFER_SIZE 128  // Default RX buffer size for UART ring buffer

/**
 * @brief Templated UART interface class.
 *
 * Provides a generic, compile-time configurable UART driver for AVR
 * microcontrollers by abstracting register locations and bit positions.
 * Enables reuse across multiple devices (e.g., ATmega16, ATmega324PA/PB) with
 * different UART register mappings.
 *
 * Inherits from RingBuffer<size> to manage incoming data via a circular
 * buffer.
 *
 * All register parameters are I/O register offsets (as used with _SFR_IO8),
 * not full SRAM addresses.
 *
 * @tparam ubrrh   I/O offset of UBRRH or UBRRnH
 * @tparam ubrrl   I/O offset of UBRRL or UBRRnL
 * @tparam ucsra   I/O offset of UCSRA or UCSRnA
 * @tparam ucsrb   I/O offset of UCSRB or UCSRnB
 * @tparam ucsrc   I/O offset of UCSRC or UCSRnC
 * @tparam udr     I/O offset of UDR or UDRn
 * @tparam rxen    Bit position of RX enable in UCSRB
 * @tparam txen    Bit position of TX enable in UCSRB
 * @tparam rxcie   Bit position of RX Complete Interrupt Enable in UCSRB
 * @tparam udre    Bit position of UDRE (data register empty) in UCSRA
 * @tparam u2x     Bit position of double speed mode (U2X) in UCSRA
 * @tparam size    Size of the RX ring buffer (must be a power of two)
 */

template <uint8_t ubrrh,
          uint8_t ubrrl,
          uint8_t ucsra,
          uint8_t ucsrb,
          uint8_t ucsrc,
          uint8_t udr,
          uint8_t rxen,
          uint8_t txen,
          uint8_t rxcie,
          uint8_t udre,
          uint8_t u2x,
          uint8_t size>
class UartInterface : public RingBuffer<size>
{
 public:
  static const uint8_t ucsrc_8bitmode_mask = (1 << 2) | (1 << 1);  // UCSZ[1:0]
  /**
   * @brief Initializes the UART peripheral at the specified baud rate.
   *
   * Attempts to enable double-speed mode (U2X) for higher precision.
   * Falls back to normal speed if UBRR value exceeds 12-bit range.
   *
   * @param baud Desired baud rate (e.g., 9600, 115200).
   */
  void init(uint32_t baud)
  {
    uint16_t ubrr_value;
    REG(ucsra) = 0x00;
    REG(ucsrb) = 0x00;
    REG(ucsrc) = 0x00;

#if defined(__AVR_ATmega16__)
    REG(ucsrc) = (1 << 7) | ucsrc_8bitmode_mask;
#else
    REG(ucsrc) = ucsrc_8bitmode_mask;
#endif

    // Try U2X mode (double speed)
    REG(ucsra) = (1 << u2x);

    ubrr_value = (F_CPU / 4 / baud - 1) / 2;

    // Fallback to normal speed mode if UBRR is too large
    if(ubrr_value > 4095)
      {
        REG(ucsra) = 0;
        ubrr_value = (F_CPU / 8 / baud - 1) / 2;
      }

    // Set baud rate registers
    REG(ubrrh) = ubrr_value >> 8;
    REG(ubrrl) = ubrr_value;

    // Enable RX, TX, and RX Complete Interrupt
    REG(ucsrb) = (1 << rxen) | (1 << txen) | (1 << rxcie);

    // Disable UDRE interrupt by default
    REG(ucsrb) &= ~(1 << udre);
  }

  /**
   * @brief Writes a single byte to the UART.
   *
   * Blocks until the transmit register is ready.
   *
   * @param byte Byte to transmit.
   */
  void write(uint8_t byte)
  {
    while(!(REG(ucsra) & (1 << udre)))
      {
        ;
      }
    REG(udr) = byte;
  }

  /**
   * @brief Writes an array of bytes to the UART.
   *
   * @param bytes Pointer to byte array.
   * @param len Number of bytes to transmit.
   */
  void write(const uint8_t *bytes, size_t len)
  {
    if(!bytes)
      return;
    for(size_t i = 0; i < len; ++i)
      {
        write(bytes[i]);
      }
  }

  /**
   * @brief Writes a single character.
   *
   * @param c Character to transmit.
   */
  void write(char c) { write(static_cast<uint8_t>(c)); }

  /**
   * @brief Writes a C-string or character array.
   *
   * @param c Pointer to character array.
   * @param len Number of characters to send.
   */
  void write(const char *c, size_t len)
  {
    write(reinterpret_cast<const uint8_t *>(c), len);
  }

 private:
};

// -------- Device-Specific Typedefs -------- //

#if defined(__AVR_ATmega16__)
/**
 * @brief UART0 instance for ATmega16.
 */
typedef UartInterface<0x20,
                      0x09,
                      0x0B,
                      0x0A,
                      0x20,
                      0x0C,  // UBRRH, UBRRL, UCSRA, UCSRB, UCSRC, UDR
                      RXEN,
                      TXEN,
                      RXCIE,
                      UDRE,
                      U2X,
                      DEFAULT_BUFFER_SIZE>
  _UART0;

_UART0 Uart0;

ISR(USART_RXC_vect)
{
  uint8_t byte = UDR;
  Uart0.push(byte);
}

#elif defined(__AVR_ATmega324PA__)
/**
 * @brief UART0 and UART1 instances for ATmega324PA.
 */
typedef UartInterface<0xC5,
                      0xC4,
                      0xC0,
                      0xC1,
                      0xC2,
                      0xC6,  // UBRR0H, UBRR0L, UCSR0A, UCSR0B, UCSR0C, UDR0
                      RXEN0,
                      TXEN0,
                      RXCIE0,
                      UDRE0,
                      U2X0,
                      DEFAULT_BUFFER_SIZE>
  _UART0;

typedef UartInterface<0xCD,
                      0xCC,
                      0xC8,
                      0xC9,
                      0xCA,
                      0xCE,  // UBRR1H, UBRR1L, UCSR1A, UCSR1B, UCSR1C, UDR1
                      RXEN1,
                      TXEN1,
                      RXCIE1,
                      UDRE1,
                      U2X1,
                      DEFAULT_BUFFER_SIZE>
  _UART1;

_UART0 Uart0;
_UART1 Uart1;

ISR(USART0_RX_vect)
{
  uint8_t byte = UDR0;
  Uart0.push(byte);
}

ISR(USART1_RX_vect)
{
  uint8_t byte = UDR1;
  Uart1.push(byte);
}

#elif defined(__AVR_ATmega324PB__)
/**
 * @brief UART0 and UART1 instances for ATmega324PB.
 */
typedef UartInterface<0xC5,
                      0xC4,
                      0xC0,
                      0xC1,
                      0xC2,
                      0xC6,  // UBRR0H, UBRR0L, UCSR0A, UCSR0B, UCSR0C, UDR0
                      RXEN,
                      TXEN,
                      RXCIE,
                      UDRE,
                      U2X,
                      DEFAULT_BUFFER_SIZE>
  _UART0;

typedef UartInterface<0xCD,
                      0xCC,
                      0xC8,
                      0xC9,
                      0xCA,
                      0xCE,  // UBRR1H, UBRR1L, UCSR1A, UCSR1B, UCSR1C, UDR1
                      RXEN,
                      TXEN,
                      RXCIE,
                      UDRE,
                      U2X,
                      DEFAULT_BUFFER_SIZE>
  _UART1;

_UART0 Uart0;
_UART1 Uart1;

ISR(USART0_RX_vect)
{
  uint8_t byte = UDR0;
  Uart0.push(byte);
}

ISR(USART1_RX_vect)
{
  uint8_t byte = UDR1;
  Uart1.push(byte);
}

#endif  // Device-specific blocks

#endif  // UART_H
