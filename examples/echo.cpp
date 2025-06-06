/**
 * @file echo.cpp
 * @brief Example application demonstrating basic UART functionality.
 *
 * To compile this example using the provided Makefile:
 *
 *     make echo DEVICE=<target> F_CPU=<frequency>
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

int main(void)
{
  Uart0.init(9600);
  sei();  // Enable Global Interrupts

  uint8_t echo[] = "ECHO: ";
  while(1)
    {
      uint8_t byte;
      if(Uart0.pop(&byte))
        {
          Uart0.write(echo, sizeof(echo));
          do
            {
              Uart0.write(byte);
          } while(Uart0.pop(&byte));
        }
    }
}