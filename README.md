# AVR UART Library
A lightweight and portable C UART driver for AVR microcontrollers. This library supports:
- ATmega324PA
- ATmega324PB
- ATmega16
Designed for low-level embedded systems programming, this library allows precise control over UART preipherals without relying on bloated frameworks or libraries.
---

## Features
- Minimal dependencies
- TX/RX support with optional interrupt-based receive
- Baud rate calculation
- Flexible initialization routines
- Modular structure for multi-device support
---

## Getting Started

### Supported MCUs
- ATmega324PA
- ATmega324PB
- ATmega16

### Directory Structure
avr-uart-lib/
├── include/
│ └── uart.h
├── src/
│ ├── uart.c
│ └── uart_m324pa.c
│ └── uart_m324pb.c
│ └── uart_m16.c
│ └── . . .
├── examples/
│ └── uart_echo.c
│ └── . . .
└── Makefile

### Basic Usage
```c
#include "uart.h"

int main(void) {
    uart0_init(9600);         // Initialize UART0 at 9600 baud
    uart0_puts("Hello UART"); // Send string
    while (1) {
        if (uart0_available()) {
            char c = uart0_get();
            uart0_put(c); // Echo
        }
    }
}
```

### Build and Flash
- Make sure `avr-gcc` and `avrdude` are installed.
- Use the provided Makefule or integrate the `src/` folder into your existing AVR project.

## TODO
- [ ] Add support for ATmega324PA
- [ ] Add support for ATmega324PB
- [ ] Add support for ATmega16