/**
 * @file device_config.h
 * @brief MCU feature detection and compile-time configuration.
 *
 * Defines device-specific macros and register aliases for consistent
 * cross-device support in the avr-uart-lib project.
 *
 * Supported devices:
 *   - ATmega324PA
 *   - ATmega324PB
 *   - ATmega16
 *
 * Devices with multiple UARTs will define HAS_UART1, and older devices
 * (like ATmega16) will alias their register names to match the UART0
 * register naming scheme.
 *
 * Part of the avr-uart-lib project:
 * https://github.com/Joseph-Godwin-Git/avr-uart-lib
 * 
 * @author Joseph Godwin
 * @date 2025-05-29
 * @license MIT
 */

#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

// -------------------------------------------------------------
// Feature detection
// -------------------------------------------------------------

#if defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega324PB__)
    #define HAS_UART1
#endif

// -------------------------------------------------------------
// Register name unification for ATmega16
// -------------------------------------------------------------

#if defined(__AVR_ATmega16__)

// UART0-style aliases
#define UBRR0H  UBRRH
#define UBRR0L  UBRRL
#define UCSR0A  UCSRA
#define UCSR0B  UCSRB
#define UCSR0C  UCSRC
#define UDR0    UDR

#define RXC0    RXC
#define TXC0    TXC
#define UDRE0   UDRE
#define RXEN0   RXEN
#define TXEN0   TXEN
#define UCSZ00  UCSZ0
#define UCSZ01  UCSZ1

#endif // __AVR_ATmega16__

#endif // DEVICE_CONFIG_H
