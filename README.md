# Firmware for Ambiq Apollo 5

Edge Impulse firmware for Ambiq Apollo 5 EVB (apollo510_evb)

## Prerequisites
Apollo 5 EVB
GCC v13.x.x
[Segger Jlink software](https://www.segger.com/downloads/jlink/) version should be >= 8.00

## Import you model
You can deploy your model as a C++ library, then unzip the content of your deployment and copy the content in sr/edge-impulse.

## Build

To build the application:
```
make -j
```

To clean the build:
```
make clean
```

## Flash
To flash the board:
```
make deploy
```

## Debug on VsCode
> [!IMPORTANT]
> You need to install [JLink software](https://www.segger.com/downloads/jlink/) and [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) extension.
To start a debug session, make sure to connect the board to J6 (JLINK USB) and press F5.

## Connect consolle to the board
Connect USB-C cables to both APOLLO5 USB connector.
The board will show as a USB device.

## Camera connection

To connect the Arducam to an Apollo5 EVB's IOM2 interface, make the following connections:

1. Camera GND to any EVB GND
2. Camera 5V/VDD to any EVB 5V pin
3. Camera SCK to EVB Pin 25
4. Camera MISO to EVB Pin 27
5. Camera MOSI to EVB 26
6. Camera CS to EVB 29

Arducam pinout: https://docs.arducam.com/Arduino-SPI-camera/MEGA-SPI/MEGA-Quick-Start-Guide/