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
#include "ringbuf.h"

#define DEFAULT_BUFFER_SIZE 128  // Default RX buffer size for UART ring buffer

/**
 * @brief Templated UART interface class.
 *
 * Allows compile-time configuration of register addresses and bit positions
 * to support multiple AVR microcontrollers with a single implementation.
 * Inherits from RingBuffer<size> to handle RX buffering.
 *
 * @tparam ubrrh   Address of UBRRH or UBRRnH
 * @tparam ubrrl   Address of UBRRL or UBRRnL
 * @tparam ucsra   Address of UCSRA or UCSRnA
 * @tparam ucsrb   Address of UCSRB or UCSRnB
 * @tparam udr     Address of UDR or UDRn
 * @tparam rxen    Bit position for RX enable
 * @tparam txen    Bit position for TX enable
 * @tparam rxcie   Bit position for RX complete interrupt enable
 * @tparam udre    Bit position for data register empty
 * @tparam u2x     Bit position for double speed mode
 * @tparam size    Size of the RX ring buffer
 */
template <uint8_t ubrrh, uint8_t ubrrl, uint8_t ucsra, uint8_t ucsrb,
          uint8_t udr, uint8_t rxen, uint8_t txen, uint8_t rxcie,
          uint8_t udre, uint8_t u2x, uint8_t size>
class UartInterface : public RingBuffer<size> {
public:
    /**
     * @brief Initializes the UART peripheral at the specified baud rate.
     *
     * Attempts to enable double-speed mode (U2X) for higher precision.
     * Falls back to normal speed if UBRR value exceeds 12-bit range.
     *
     * @param baud Desired baud rate (e.g., 9600, 115200).
     */
    void init(uint32_t baud) {
        uint16_t ubrr_value;
        
        // Try U2X mode (double speed)
        _SFR_IO8(ucsra) = 1 << u2x;
        ubrr_value = (F_CPU / 4 / baud - 1) / 2;

        // Fallback to normal speed mode if UBRR is too large
        if (ubrr_value > 4095) {
            _SFR_IO8(ucsra) = 0;
            ubrr_value = (F_CPU / 8 / baud - 1) / 2;
        }

        // Set baud rate registers
        _SFR_IO8(ubrrh) = ubrr_value >> 8;
        _SFR_IO8(ubrrl) = ubrr_value;

        // Enable RX, TX, and RX Complete Interrupt
        _SFR_IO8(ucsrb) = (1 << rxen) | (1 << txen) | (1 << rxcie);

        // Disable UDRE interrupt by default
        _SFR_IO8(ucsrb) &= ~(1 << udre);
    }

    /**
     * @brief Writes a single byte to the UART.
     *
     * Blocks until the transmit register is ready.
     *
     * @param byte Byte to transmit.
     */
    void write(uint8_t byte) {
        while (!(_SFR_IO8(ucsra) & (1 << udre))) {;}
        _SFR_IO8(udr) = byte;
    }

    /**
     * @brief Writes an array of bytes to the UART.
     *
     * @param bytes Pointer to byte array.
     * @param len Number of bytes to transmit.
     */
    void write(const uint8_t* bytes, size_t len) {
        if (!bytes) return;
        for (size_t i = 0; i < len; ++i) {
            write(bytes[i]);
        }
    }

    /**
     * @brief Writes a single character.
     *
     * @param c Character to transmit.
     */
    void write(char c) {
        write(static_cast<uint8_t>(c));
    }

    /**
     * @brief Writes a C-string or character array.
     *
     * @param c Pointer to character array.
     * @param len Number of characters to send.
     */
    void write(const char* c, size_t len) {
        write(reinterpret_cast<const uint8_t*>(c), len);
    }
};

// -------- Device-Specific Typedefs -------- //

#if defined(__AVR_ATmega16__)
/**
 * @brief UART0 instance for ATmega16.
 */
typedef UartInterface<
    _SFR_IO_ADDR(UBRRH), _SFR_IO_ADDR(UBRRL), _SFR_IO_ADDR(UCSRA), _SFR_IO_ADDR(UCSRB), _SFR_IO_ADDR(UDR),
    RXEN, TXEN, RXCIE, UDRE, U2X, DEFAULT_BUFFER_SIZE
> _UART0;

#elif defined(__AVR_ATmega324PA__)
/**
 * @brief UART0 and UART1 instances for ATmega324PA.
 */
typedef UartInterface<
    _SFR_IO_ADDR(UBRR0H), _SFR_IO_ADDR(UBRR0L), _SFR_IO_ADDR(UCSR0A), _SFR_IO_ADDR(UCSR0B), _SFR_IO_ADDR(UDR0),
    RXEN0, TXEN0, RXCIE0, UDRE0, U2X0, DEFAULT_BUFFER_SIZE
> _UART0;

typedef UartInterface<
    _SFR_IO_ADDR(UBRR1H), _SFR_IO_ADDR(UBRR1L), _SFR_IO_ADDR(UCSR1A), _SFR_IO_ADDR(UCSR1B), _SFR_IO_ADDR(UDR1),
    RXEN1, TXEN1, RXCIE1, UDRE1, U2X1, DEFAULT_BUFFER_SIZE
> _UART1;

#elif defined(__AVR_ATmega324PB__)
/**
 * @brief UART0 and UART1 instances for ATmega324PB.
 */
typedef UartInterface<
    _SFR_IO_ADDR(UBRR0H), _SFR_IO_ADDR(UBRR0L), _SFR_IO_ADDR(UCSR0A), _SFR_IO_ADDR(UCSR0B), _SFR_IO_ADDR(UDR0),
    RXEN, TXEN, RXCIE, UDRE, U2X, DEFAULT_BUFFER_SIZE
> _UART0;

typedef UartInterface<
    _SFR_IO_ADDR(UBRR1H), _SFR_IO_ADDR(UBRR1L), _SFR_IO_ADDR(UCSR1A), _SFR_IO_ADDR(UCSR1B), _SFR_IO_ADDR(UDR1),
    RXEN, TXEN, RXCIE, UDRE, U2X, DEFAULT_BUFFER_SIZE
> _UART1;

#endif // Device-specific blocks

#endif // UART_H
