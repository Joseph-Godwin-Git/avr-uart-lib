# AVR UART Library
A lightweight and portable C UART driver for AVR microcontrollers. This library supports:
- ATmega324PA
- ATmega324PB
- ATmega16

<p>Designed for low-level embedded systems programming, this library allows precise control over UART preipherals without relying on bloated frameworks or libraries.</p>

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
avr-uart-lib/           \
├── include/            \
│ └── uart.h            \
│ └── ringbuf.h         \
├── examples/           \
│ └── hello_world.cpp   \
│ └── . . .             \
└── Makefile            

### Basic Usage
```c
#include <util/delay.h>
#include "uart.h"

int main(void){
    _UART0 uart0;
    uart0.init(9600);
    
    while(1){
        uint8_t string0[] = "[UART0] Hello World!\r\n";
        uart0.write(string0, sizeof(string0));
        _delay_ms(500);
    }
}
```

### Build and Flash
- Make sure `avr-gcc` and `avrdude` are installed.
- Use the provided Makefile or integrate the `include/` folder into your existing AVR project.

# Examples
## Echo
To compile this example using the provided makefile, run the following command. \
```make echo DEVICE=dev F_CPU=freq``` \

Where ```dev``` is the  target device:
- ```m16``` → ATmega16
- ```m324pa``` → ATmega324PA
- ```m324pb``` → ATmega324PB 

And ```freq``` is the device's clock frequency.

![me](https://github.com/Joseph-Godwin-Git/avr-uart-lib/blob/main/examples/resources/echo-example-gif.gif)


# TODO
- [x] Add support for ATmega324PA
- [x] Add support for ATmega324PB
- [x] Add support for ATmega16
- [ ] Replace hard-coded register addresses in template instantiations with centralized register mapping configurations
- [ ] Add RX Polling support
- [ ] Add RX Interrupt support