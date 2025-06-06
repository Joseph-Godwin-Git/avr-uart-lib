/**
 * @file uart_definitions.h
 * @brief Generalized definitions to allow support for multiple devices.
 *
 * Provides standardized definitions for applicable Register / Bit access
 * as well as other supporting macros
 *
 * Part of the avr-uart-lib project:
 * https://github.com/Joseph-Godwin-Git/avr-uart-lib
 *
 * @author Joseph Godwin
 * @date 2025-06-29
 * @license MIT
 */

#ifndef UART_DEFINITIONS_H
#define UART_DEFINITIONS_H
#include <avr/io.h>

#if defined(__AVR_ATmega16__)
#define NUM_UARTS 1
#define REG(x) _SFR_IO8(x)
/* UART0 */
#define _USART0_RXC_vect USART_RXC_vect
#define _UBRR0H 0x20
#define _UBRR0L 0x09
#define _UCSRA0 0x0B
#define _UCSRB0 0x0A
#define _UCSRC0 0x20
#define _UDR0 0x0C
#define _RXEN0 RXEN
#define _TXEN0 TXEN
#define _RXCIE0 RXCIE
#define _UDRE0 UDRE
#define _U2X0 U2X
#endif  //__AVR_ATmega16__

#if defined(__AVR_ATmega324PA__)
#define NUM_UARTS 2
#define REG(x) _SFR_MEM8(x)
/* UART0 */
#define _USART0_RXC_vect USART0_RX_vect
#define _UBRR0H 0xC5
#define _UBRR0L 0xC4
#define _UCSRA0 0xC0
#define _UCSRB0 0xC1
#define _UCSRC0 0xC2
#define _UDR0 0xC6
#define _RXEN0 RXEN0
#define _TXEN0 TXEN0
#define _RXCIE0 RXCIE0
#define _UDRE0 UDRE0
#define _U2X0 U2X0
/* UART 1 */
#define _USART1_RXC_vect USART1_RX_vect
#define _UBRR1H 0xCD
#define _UBRR1L 0xCC
#define _UCSRA1 0xC8
#define _UCSRB1 0xC9
#define _UCSRC1 0xCA
#define _UDR1 0xCE
#define _RXEN1 RXEN1
#define _TXEN1 TXEN1
#define _RXCIE1 RXCIE1
#define _UDRE1 UDRE1
#define _U2X1 U2X1
#endif  //__AVR_ATmega324PA__

#if defined(__AVR_ATmega324PB__)
#define NUM_UARTS 2
#define REG(x) _SFR_MEM8(x)
/* UART0 */
#define _USART0_RXC_vect USART0_RX_vect
#define _UBRR0H 0xC5
#define _UBRR0L 0xC4
#define _UCSRA0 0xC0
#define _UCSRB0 0xC1
#define _UCSRC0 0xC2
#define _UDR0 0xC6
#define _RXEN0 RXEN
#define _TXEN0 TXEN
#define _RXCIE0 RXCIE
#define _UDRE0 UDRE
#define _U2X0 U2X
/* UART 1 */
#define _USART1_RXC_vect USART1_RX_vect
#define _UBRR1H 0xCD
#define _UBRR1L 0xCC
#define _UCSRA1 0xC8
#define _UCSRB1 0xC9
#define _UCSRC1 0xCA
#define _UDR1 0xCE
#define _RXEN1 RXEN
#define _TXEN1 TXEN
#define _RXCIE1 RXCIE
#define _UDRE1 UDRE
#define _U2X1 U2X
#endif  //__AVR_ATmega324PB__

#endif  // UART_DEFINITIONS_H