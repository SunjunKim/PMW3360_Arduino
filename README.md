# PMW3360_Arduino
Interfacing PixArt PMW3360 with Arduino

# Parts info
* PMW3360 Module: https://www.tindie.com/products/jkicklighter/pmw3360-motion-sensor/ by JACK Enterprises
* Base source code: https://github.com/mrjohnk/PMW3360DM-T2QU by mrjohnk
* Arduino Micro: https://store.arduino.cc/arduino-micro

# Pin connection
## Description
* MI = MISO
* MO = MOSI
* SS = Slave Select / Chip Select
* SC = SPI Clock
* MT = Motion (active low interrupt line)
* RS = Reset
* GD = Ground
* VI = Voltage in up to +5.5V

```
Module --- Arduino
    RS --- (NONE)
    GD --- GND
    MT --- Pin 3 (PD0, INT0) -> movement interrupt
    SS --- Pin 10 (PB6)
    SC --- SCK (PB1)
    MO --- MOSI (PB2)
    MI --- MISO (PB3)
    VI --- 5V
```

# Command (through serial)
* C[number] -> Change CPI
  * Example) C1200 -> Change CPI value to 1200.
