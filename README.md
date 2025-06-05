# AVR UART Library
A lightweight and portable C++ UART driver for AVR microcontrollers. This library supports:
- ATmega324PA
- ATmega324PB
- ATmega16

<p>Designed for low-level embedded systems programming, this library allows precise control over UART preipherals without relying on bloated frameworks or libraries.</p>

## Features
- Minimal dependencies
- TX/RX support with interrupt-based receive
- Baud rate calculation
- Modular structure for multi-device support
---

## Getting Started

### Supported MCUs
- ATmega324PA
- ATmega324PB
- ATmega16       

### Basic Usage
![me](https://github.com/Joseph-Godwin-Git/avr-uart-lib/blob/main/examples/resources/hello-world-screenshot.png)

### Build and Flash
- Make sure `avr-gcc` and `avrdude` are installed.
 - If using ATmega324PB, make sure the ATmega324PB device pack is installed.
- Use the provided Makefile or integrate the `include/` folder into your existing AVR project.

# Examples
## Hello World
To compile this example using the provided makefile, run the following command. \
```make hello_world DEVICE=dev F_CPU=freq``` 

Where ```dev``` is the  target device:
- ```m16``` → ATmega16
- ```m324pa``` → ATmega324PA
- ```m324pb``` → ATmega324PB 

And ```freq``` is the device's clock frequency.

![me](https://github.com/Joseph-Godwin-Git/avr-uart-lib/blob/main/examples/resources/hello-world-example-gif.gif)

## Echo
To compile this example using the provided makefile, run the following command. \
```make echo DEVICE=dev F_CPU=freq``` 

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
- [x] Add RX Interrupt support