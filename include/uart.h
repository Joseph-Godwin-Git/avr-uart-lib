/**
 * @file uart.h
 * @brief UART driver interface for AVR microcontrollers.
 *
 * Provides UART initialization and basic communication functions
 * for supported devices:
 *   - ATmega324PA
 *   - ATmega324PB
 *   - ATmega16
 *
 * Supports UART0 on all devices and UART1 on those that include it.
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
#include <stdbool.h>
#include "device_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes UART0 with the specified baud rate.
 *
 * Configures baud rate, enables TX and RX, and sets 8N1 frame format.
 *
 * @param baud Desired baud rate (e.g. 9600, 115200).
 */
void uart0_init(uint32_t baud);

/**
 * @brief Transmits a single character over UART0.
 *
 * Blocks until the transmit buffer is ready.
 *
 * @param c Character to send.
 */
void uart0_put(char c);

/**
 * @brief Sends a null-terminated string over UART0.
 *
 * @param s Pointer to the string to send.
 */
void uart0_puts(const char *s);

/**
 * @brief Receives a character from UART0.
 *
 * Blocks until a character is received.
 *
 * @return Received character.
 */
char uart0_get(void);

/**
 * @brief Checks if data is available to read from UART0.
 *
 * @return true if data is available, false otherwise.
 */
bool uart0_available(void);

#if defined(HAS_UART1)

/**
 * @brief Initializes UART1 with the specified baud rate.
 *
 * Only applicable on devices that support UART1 (e.g. ATmega324PA/PB).
 *
 * @param baud Desired baud rate (e.g. 9600, 115200).
 */
void uart1_init(uint32_t baud);

/**
 * @brief Transmits a single character over UART1.
 *
 * Blocks until the transmit buffer is ready.
 *
 * @param c Character to send.
 */
void uart1_put(char c);

/**
 * @brief Sends a null-terminated string over UART1.
 *
 * @param s Pointer to the string to send.
 */
void uart1_puts(const char *s);

/**
 * @brief Receives a character from UART1.
 *
 * Blocks until a character is received.
 *
 * @return Received character.
 */
char uart1_get(void);

/**
 * @brief Checks if data is available to read from UART1.
 *
 * @return true if data is available, false otherwise.
 */
bool uart1_available(void);

#endif // HAS_UART1

#ifdef __cplusplus
}
#endif

#endif // UART_H
