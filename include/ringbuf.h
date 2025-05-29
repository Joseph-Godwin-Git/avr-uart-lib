/**
 * @file ringbuf.h
 * @brief Ring (circular) buffer implementation for AVR-based systems.
 *
 * Provides a lightweight, fixed-size ring buffer template for storing bytes.
 * Designed for use in interrupt-driven environments like UART RX handling.
 *
 * Template parameter BufferSize must be a power of two to enable optimized index wrapping.
 *
 * Part of the avr-uart-lib project:
 * https://github.com/Joseph-Godwin-Git/avr-uart-lib
 * 
 * @author Joseph Godwin
 * @date 2025-05-29
 * @license MIT
 */

#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>

/**
 * @brief A fixed-size ring (circular) buffer for storing bytes.
 * 
 * Template parameter controls the buffer capacity.
 * Suitable for use in interrupt-driven UART receive routines.
 */
template <uint8_t BufferSize>
class RingBuffer {
public:
    // Compile-time check: BufferSize must be a power of two for bitwise modulo optimization
    static_assert((BufferSize & (BufferSize - 1)) == 0, "RingBuffer: BufferSize must be a power of 2");

    /**
     * @brief Inserts a byte into the buffer if space is available.
     *
     * This function is safe to call from an ISR.
     * If the buffer is full, the byte will be dropped.
     *
     * @param data Byte to store.
     */
    void push(uint8_t data) {
        uint8_t next = (_writeIndex + 1) & (BufferSize - 1);
        if (next == _readIndex) {
            return;
        }
        _buffer[_writeIndex] = data;
        _writeIndex = next;
    }

    /**
     * @brief Removes the next byte from the buffer.
     *
     * @param byte Pointer to destination variable.
     * @return true if a byte was read; false if the buffer was empty.
     */
    bool pop(uint8_t *byte) {
        if(!peek(byte))
            return false;
        _readIndex = (_readIndex + 1) & (BufferSize - 1);
        return true;
    }

    /**
     * @brief Retrieves the next byte without removing it.
     *
     * @param byte Pointer to destination variable.
     * @return true if a byte is available; false if the buffer is empty.
     */
    bool peek(uint8_t *byte) const {
        if(isEmpty()) {
            return false;
        }

        *byte = _buffer[_readIndex];
        return true;
    }

    /**
     * @brief Checks if the buffer is empty.
     *
     * @return true if empty, false otherwise.
     */
    bool isEmpty(void) const {
        return (_writeIndex == _readIndex);
    }

    /**
     * @brief Checks if the buffer is full.
     * 
     * @return true if buffer is full; false otherwise.
     */
    bool isFull(void) const {
        return ((_writeIndex + 1) & (BufferSize - 1)) == _readIndex;
    }

    void flush(void) {
        _writeIndex = _readIndex;
    }

private:
    volatile uint8_t _buffer[BufferSize];
    volatile uint8_t _writeIndex = 0;
    volatile uint8_t _readIndex = 0;
};

#endif // UART_H
