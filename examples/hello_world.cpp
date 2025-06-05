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
    Uart0.init(9600);
    uint8_t string0[] = "[UART0] Hello World!\r\n";
    while(1){
        Uart0.write(string0, sizeof(string0));
        _delay_ms(500);
    }
}

