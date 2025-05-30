/**
 * @file hello_world.cpp
 * @brief Example application demonstrating basic UART functionality.
 *
 * To compile this example using the provided Makefile:
 *
 *     make hello_world DEVICE=<target> F_CPU=<frequency>
 *
 * Where:
 *   <target>   : Target microcontroller
 *       - m16     → ATmega16
 *       - m324pa  → ATmega324PA
 *       - m324pb  → ATmega324PB
 *
 *   <frequency> : CPU clock frequency in Hz
 *
 * Example:
 *   Compile for an ATmega16 running at 8 MHz:
 *       make hello_world DEVICE=m16 F_CPU=8000000
 *
 * Ensure the selected device and clock frequency match your hardware setup.
 */

#include <util/delay.h>
#include "uart.h"

int main(void){
    _UART0 uart0;
    uart0.init(9600);

    #if defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega324PB__)
    _UART1 uart1;
    uart1.init(9600);
    #endif // __AVR_ATmega324PA__ || __AVR_ATmega324PB__
    

    while(1){
        uint8_t string0[] = "[UART0] Hello World!\r\n";
        uart0.write(string0, sizeof(string0));

        #if defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega324PB__)
        uint8_t string1[] = "[UART1] Hello World!\r\n";
        uart1.write(string1, sizeof(string1));
        #endif // __AVR_ATmega324PA__ || __AVR_ATmega324PB__

        _delay_ms(500);
    }
}

